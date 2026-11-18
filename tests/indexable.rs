use camlib::{dexel::*, facing::FacingPlan, motion::*, pocket::Operation, stock::*, stream::StreamManifest, tool::{*, rotational::*}};

fn frustum(z: [f64; 2], inner: [f64; 2], outer: [f64; 2]) -> ProfileRegion {
    ProfileRegion::AnnularFrustum { z_mm: z, inner_radius_mm: inner, outer_radius_mm: outer }
}
fn tool(round: bool) -> ToolAssembly {
    let cutting = if round {
        ProfileRegion::CircularRing { center_rz_mm: [4., 1.], radius_mm: 1., z_mm: [0., 2.] }
    } else { frustum([0., 2.], [2.; 2], [4., 6.]) };
    ToolAssembly {
        id: "mill".into(),
        cutter: Cutter {
            id: "inserted".into(), name: "synthetic indexable cutter".into(),
            shape: CutterShape::Indexable { milling: Box::new(IndexableMilling {
                family: if round { MillingFamily::HighFeed } else { MillingFamily::Face },
                geometry: IndexableGeometry {
                    inserts: vec![MountedInsertGroup { id: "inserts".into(), count: 3, cutting: RotationalProfile { regions: vec![cutting] } }],
                    noncutting: RotationalProfile { regions: vec![frustum([2.5, 10.], [0.; 2], [5.; 2])] },
                },
                max_depth_of_cut_mm: 1., max_ramp_angle_deg: 3.,
                helical_hole_diameter_mm: Some([14., 18.]),
            }) },
            diameter_mm: if round { 10. } else { 12. }, cutting_length_mm: 2., overall_length_mm: 30.,
            shank_diameter_mm: 6., flutes: 3, center_cutting: false, material: "declared".into(), neck: None,
        },
        stickout_mm: 15., gauge_length_mm: 40.,
        holders: vec![HolderSection { z_min_mm: 15., z_max_mm: 40., radius_min_mm: 6., radius_max_mm: 6. }],
    }
}
fn spec() -> StockSpec {
    StockSpec { version: 1, units: "mm".into(), min_mm: [-10., -8., 0.], max_mm: [10., 8., 6.], cell_size_mm: 0.25, max_cells: 100_000 }
}
fn motion(a: Vec3, b: Vec3) -> Motion {
    Motion { id: 1, operation: "cut".into(), tool: "mill".into(), kind: MoveKind::Cut, curve: Curve::Line { start: a, end: b }, feed_mm_min: Some(500.) }
}
fn sweep(stock: &mut Stock, tool: &ToolAssembly, motion: &Motion) {
    let delta = stock.preview_sweep(&tool.prepare().unwrap(), motion, 1., SweepOptions::default(), &|| false).unwrap();
    stock.apply_delta(&delta).unwrap();
}
fn at(stock: &Stock, x: f64, y: f64) -> HeightBounds {
    let [dx, dy] = stock.cell_widths_mm();
    let i = ((x - stock.spec().min_mm[0]) / dx) as usize;
    let j = ((y - stock.spec().min_mm[1]) / dy) as usize;
    stock.cells()[j * stock.dimensions()[0] + i]
}

#[test]
fn short_insert_ring_cut_preserves_the_center_in_both_stock_models() {
    for round in [false, true] {
        let t = tool(round);
        let m = motion([0., 0., 5.5], [0.01, 0., 5.5]);
        let mut h = Stock::new(spec()).unwrap();
        sweep(&mut h, &t, &m);
        assert_eq!(at(&h, 0., 0.).lower_mm, 6.);
        assert_eq!(at(&h, 0., 0.).upper_mm, 6.);
        let s = spec();
        let mut d = DexelStock::new(DexelStockSpec { version: 1, units: "mm".into(), model: DexelModel::TriDexel,
            min_mm: s.min_mm, max_mm: s.max_mm, cell_size_mm: s.cell_size_mm, max_rays: 100_000, max_intervals: 1_000_000 }).unwrap();
        let delta = d.preview_sweep(&t.prepare().unwrap(), &m, 1., SweepOptions::default(), &|| false).unwrap();
        d.apply_delta(&delta).unwrap();
        assert_eq!(at(d.view(), 0., 0.).lower_mm, 6.);
        assert_eq!(at(d.view(), 0., 0.).upper_mm, 6.);
        assert!(h.volume().removed_lower_mm3 > 0.);
        assert!(d.volume().removed_lower_mm3 > 0.);
    }
}

#[test]
fn long_rows_enclose_the_analytic_shoulder_and_curved_floor() {
    for round in [false, true] {
        let mut stock = Stock::new(spec()).unwrap();
        sweep(&mut stock, &tool(round), &motion([-20., 0., 5.25], [20., 0., 5.25]));
        for (i, cell) in stock.cells().iter().enumerate() {
            let [x, y] = stock.cell_center(i).unwrap();
            if x.abs() > 8. { continue; }
            let height = |v: f64| {
                let rise = if round {
                    let d = (v.abs() - 4.).max(0.);
                    if d >= 1. { return 6.; }
                    1. - (1. - d * d).sqrt()
                } else { (v.abs() - 4.).max(0.) };
                (5.25 + rise).min(6.)
            };
            let low = height((y.abs() - 0.125).max(0.));
            let high = height(y.abs() + 0.125);
            assert!(cell.lower_mm <= low + 1e-10, "{round} {x} {y}: {cell:?} low {low}");
            assert!(cell.upper_mm >= high - 1e-10, "{round} {x} {y}: {cell:?} high {high}");
            if y.abs() < 3.5 { assert!(cell.upper_mm < 5.251, "{cell:?}"); }
        }
        let target = camlib::material::Target {
            version: 1, units: "mm".into(), id: "floor".into(), revision: "1".into(),
            machining_region: rectangle([-8., -5., 8., 5.]), floor_z_mm: 5.25, floor_error_mm: 0.,
        }.prepare(camlib::material::Options { scope_to_region: true, ..Default::default() }).unwrap();
        let report = target.check_stock(&stock, &|| false).unwrap();
        assert!(report.complete);
        assert!(report.residual.definite_cells > 0, "actual shoulder/curved-floor residual must be reported");
        assert_eq!(report.gouge.definite_cells, 0);
    }
}

fn rectangle(xy: [f64;4]) -> camlib::planar::PlanarRegion {
    use camlib::planar::*;
    PlanarRegion { version: 1, units: "mm".into(), name: "rectangle".into(), boundary_error_mm: 0., source: serde_json::Value::Null,
        components: vec![Component { outer: Boundary { vertices: vec![[xy[0],xy[1],0.],[xy[2],xy[1],0.],[xy[2],xy[3],0.],[xy[0],xy[3],0.]] }, holes: vec![] }] }
}

#[test]
fn fixture_checks_include_the_body_but_do_not_fill_the_insert_center() {
    use camlib::verification::*;
    let t = tool(false).prepare().unwrap();
    for (z, expected) in [([1., 1.5], Status::Clear), ([3., 4.], Status::Violation)] {
        let scene = Scene { version: 1, units: "mm".into(), id: "fixture".into(), revision: "1".into(), work_to_machine: WorkFrame::identity(), machine: None,
            bodies: vec![Prism { id: "fixture".into(), role: BodyRole::Fixture, representation: Representation::SolidPrism,
                footprint: rectangle([-0.5,-0.5,0.5,0.5]), z_min_mm:z[0], z_max_mm:z[1], z_error_mm:0. }] }
            .prepare(Options::default()).unwrap();
        let report = scene.verify_motion(&t, &motion([0.,0.,0.],[0.1,0.,0.]), &|| false).unwrap();
        assert_eq!(report.status, expected, "{report:?}");
    }
}

#[test]
fn all_three_dexel_grids_enclose_the_analytic_swept_ring() {
    let t = tool(false);
    let s = spec();
    let mut d = DexelStock::new(DexelStockSpec { version: 1, units: "mm".into(), model: DexelModel::TriDexel,
        min_mm: s.min_mm, max_mm: s.max_mm, cell_size_mm: 0.5, max_rays: 100_000, max_intervals: 1_000_000 }).unwrap();
    let m = motion([-2., 0., 5.25], [2., 0., 5.25]);
    let delta = d.preview_sweep(&t.prepare().unwrap(), &m, 1., SweepOptions::default(), &|| false).unwrap();
    d.apply_delta(&delta).unwrap();
    let grid = d.grid();
    let mut cleared = [0;3];
    for axis in 0..3 {
        let [u,v] = DexelGrid::cross(axis);
        for (i,ray) in d.rays(axis).iter().enumerate() {
            let c = d.ray_center(axis,i);
            for offset in [-0.49,0.,0.49] {
                for j in 1..80 {
                    let mut p = [0.;3];
                    p[u] = c[0]+offset*grid.step_mm[u];
                    p[v] = c[1]-offset*grid.step_mm[v];
                    p[axis] = s.min_mm[axis]+(s.max_mm[axis]-s.min_mm[axis])*(j as f64/80.);
                    // Exact line-sweep condition: radial distance ranges
                    // continuously from nearest segment point to farther end.
                    let h = p[2]-5.25;
                    let near2 = (p[0]-p[0].clamp(-2.,2.)).powi(2)+p[1]*p[1];
                    let far2 = (p[0]+2.).powi(2).max((p[0]-2.).powi(2))+p[1]*p[1];
                    let cut = (0. ..=2.).contains(&h) && near2 <= (4.+h).powi(2) && far2 >= 4.;
                    let contains = |spans: &Spans| spans.as_slice().iter().any(|s| s[0]<=p[axis] && p[axis]<=s[1]);
                    if contains(&ray.certain) { assert!(!cut, "certain material was cut: axis {axis}, {p:?}"); }
                    if !contains(&ray.possible) { cleared[axis]+=1; assert!(cut, "uncut material erased: axis {axis}, {p:?}"); }
                }
            }
        }
    }
    assert!(cleared.iter().all(|n| *n>100), "{cleared:?}");
}

#[test]
fn facing_multiple_insert_depth_layers_removes_the_whole_top() {
    let fixture: StreamManifest = serde_json::from_str(include_str!("../fixtures/operations/facing.json")).unwrap();
    let Operation::Facing(mut op) = fixture.operations[0].clone() else { panic!() };
    op.tool = "mill".into(); op.floor_z_mm = 3.; op.stepdown_mm = 0.5; op.stepover_mm = 4.; op.clearance_z_mm = 12.;
    op.edge_overlap_mm = Some(1.);
    let t = tool(false);
    let mut stock = Stock::new(spec()).unwrap();
    let mut plan = FacingPlan::prepare(op.clone(), &t, stock.spec(), [0., 0., 12.], 1, &|| false).unwrap();
    while let Some(m) = plan.next_motion_with_stock(&stock, &|| false).unwrap() { sweep(&mut stock, &t, &m); }
    for cell in stock.cells() {
        assert!(cell.lower_mm <= 3. && cell.upper_mm <= 3.001, "{cell:?}");
    }
    op.stepdown_mm = 1.5;
    let error = FacingPlan::prepare(op, &t, stock.spec(), [0., 0., 12.], 1, &|| false).err().unwrap();
    assert!(error.contains("max_depth_of_cut_mm"), "{error}");
}

#[test]
fn supplied_moves_cannot_bypass_depth_plunge_or_body_limits() {
    let stock = Stock::new(spec()).unwrap();
    let t = tool(false);
    for (m, reason) in [
        (motion([-20., 0., 4.8], [20., 0., 4.8]), "max_depth_of_cut_mm"),
        (motion([0., 0., 6.1], [0., 0., 5.5]), "cannot plunge"),
        (motion([-8., 0., 6.1], [8., 0., 5.]), "max_ramp_angle_deg"),
    ] {
        let error = stock.preview_sweep(&t.prepare().unwrap(), &m, 1., SweepOptions::default(), &|| false).err().unwrap();
        assert!(error.contains(reason), "{reason}: {error}");
    }
    let mut low_body = t;
    if let CutterShape::Indexable { milling } = &mut low_body.cutter.shape {
        milling.geometry.noncutting.regions = vec![frustum([0.25, 10.], [0.; 2], [5.; 2])];
    }
    let error = stock.preview_sweep(&low_body.prepare().unwrap(), &motion([-20., 0., 5.5], [20., 0., 5.5]), 1., SweepOptions::default(), &|| false).err().unwrap();
    assert!(error.contains("noncutting body"), "{error}");
    assert_eq!(stock.generation(), 0);
}

#[test]
fn helical_entry_checks_machined_hole_diameter_and_actual_stock() {
    let stock = Stock::new(spec()).unwrap();
    let t = tool(false).prepare().unwrap();
    // Radius 1 would pass if the minimum were checked with the maximum
    // 12 mm cutter diameter. Its floor-cutting diameter is only 8 mm.
    for (radius, allowed) in [(3., true), (0.5, false), (1., false), (4., false)] {
        let m = Motion {
            curve: Curve::Helix {
                start: [radius,0.,6.01], end: [radius,0.,5.5], center: [0.,0.,6.01],
                plane: Plane::Xy, sweep_rad: std::f64::consts::TAU,
            },
            ..motion([0.;3],[0.;3])
        };
        let result = stock.preview_sweep(&t,&m,1.,SweepOptions::default(),&|| false);
        if allowed {
            assert!(result.is_ok(), "{}", result.err().unwrap_or_default());
        } else {
            let error = result.err().unwrap();
            assert!(error.contains("helical_hole_diameter_mm"), "{error}");
        }
    }
}

#[test]
fn annular_sweeps_do_not_establish_global_heightfield_departure_proofs() {
    let mut stock = Stock::new(spec()).unwrap();
    let t = tool(false);
    sweep(&mut stock, &t, &motion([0.,0.,5.5],[0.01,0.,5.5]));
    let mut flat = t;
    flat.cutter.shape = CutterShape::Flat;
    flat.cutter.cutting_length_mm = 10.;
    let m = motion([-20.,0.,5.],[20.,0.,5.]);
    sweep(&mut stock,&flat,&m);
    assert!(!stock.departs_clear("mill",m.curve.end()));
}

#[test]
fn annular_quarter_arc_at_a_previously_cut_cell_boundary() {
    // Isolate the one cell affected on the fifth level of indexable-roughing.
    let stock = Stock::from_snapshot(StockSnapshot {
        spec: StockSpec {
            min_mm: [-0.25, -3.75, 0.],
            max_mm: [0., -3.5, 6.],
            ..spec()
        },
        generation: 0,
        cells: vec![HeightBounds { lower_mm: 3.5149039258643247, upper_mm: 3.6608631225419495 }],
    }).unwrap();
    let start = [-0.00032, -8.001282548341209, 3.5];
    let end = [-5.0012825483412096, -3.0003200000000003, 3.5];
    let m = Motion {
        curve: Curve::Arc { start, end, center: [-0.0003200000000012082, -3.0003200000000003, 3.5],
            plane: Plane::Xy, sweep_rad: -1.5707963267948968 },
        ..motion(start, end)
    };
    let delta = stock.preview_sweep(&tool(true).prepare().unwrap(), &m, 1., SweepOptions::default(), &|| false).unwrap();
    assert!(delta.changes().is_empty(), "{:?}", delta.changes());
}
