//! Deburr (#11): horizontal edges broken with a chamfer mill, and nothing else
//! touched. The checks here are independent of the planner: a dense
//! penetration oracle built from the tool assembly's own radius profile and
//! sampled points of the part, and the stock's actual heights.
use camlib::{
    deburr::{DeburrLevel, DeburrOperation, DeburrPlan},
    motion::{Coolant, Curve, Event, Motion, MoveKind, ProgramValidator, Vec3},
    planar::{Boundary, Component, PlanarRegion},
    stock::{Stock, StockSpec, SweepOptions},
    surface::SurfaceMesh,
    tool::{Cutter, CutterShape, HolderSection, ToolAssembly},
};
use std::sync::atomic::AtomicBool;

fn chamfer_mill() -> ToolAssembly {
    ToolAssembly {
        id: "c90".into(),
        cutter: Cutter {
            id: "c90".into(),
            name: String::new(),
            shape: CutterShape::Chamfer {
                included_angle_rad: std::f64::consts::FRAC_PI_2,
                tip_diameter_mm: 0.2,
            },
            diameter_mm: 6.,
            cutting_length_mm: 4.,
            overall_length_mm: 50.,
            shank_diameter_mm: 6.,
            flutes: 2,
            center_cutting: true,
            material: "carbide".into(),
            neck: None,
        },
        stickout_mm: 20.,
        gauge_length_mm: 60.,
        holders: vec![HolderSection {
            z_min_mm: 20.,
            z_max_mm: 60.,
            radius_min_mm: 8.,
            radius_max_mm: 16.,
        }],
    }
}

fn end_mill() -> ToolAssembly {
    let mut t = chamfer_mill();
    t.id = "em6".into();
    t.cutter.id = "em6".into();
    t.cutter.shape = CutterShape::Flat;
    t.cutter.cutting_length_mm = 14.;
    t
}

/// The 12 triangles of an axis-aligned box.
fn cuboid(lo: Vec3, hi: Vec3) -> Vec<[Vec3; 3]> {
    let p = |x: usize, y: usize, z: usize| {
        [
            if x == 0 { lo[0] } else { hi[0] },
            if y == 0 { lo[1] } else { hi[1] },
            if z == 0 { lo[2] } else { hi[2] },
        ]
    };
    let quad = |a: Vec3, b: Vec3, c: Vec3, d: Vec3| vec![[a, b, c], [a, c, d]];
    [
        quad(p(0, 0, 1), p(1, 0, 1), p(1, 1, 1), p(0, 1, 1)),
        quad(p(0, 0, 0), p(0, 1, 0), p(1, 1, 0), p(1, 0, 0)),
        quad(p(0, 0, 0), p(1, 0, 0), p(1, 0, 1), p(0, 0, 1)),
        quad(p(0, 1, 0), p(0, 1, 1), p(1, 1, 1), p(1, 1, 0)),
        quad(p(0, 0, 0), p(0, 0, 1), p(0, 1, 1), p(0, 1, 0)),
        quad(p(1, 0, 0), p(1, 1, 0), p(1, 1, 1), p(1, 0, 1)),
    ]
    .concat()
}

fn mesh(triangles: Vec<[Vec3; 3]>) -> SurfaceMesh {
    SurfaceMesh {
        version: 1,
        units: "mm".into(),
        triangles: std::sync::Arc::new(triangles),
        surface_error_mm: 0.,
        source: serde_json::json!({}),
    }
}

fn rect(lo: [f64; 2], hi: [f64; 2]) -> PlanarRegion {
    PlanarRegion {
        version: 1,
        units: "mm".into(),
        name: "face".into(),
        boundary_error_mm: 0.,
        components: vec![Component {
            outer: Boundary {
                vertices: vec![
                    [lo[0], lo[1], 0.],
                    [hi[0], lo[1], 0.],
                    [hi[0], hi[1], 0.],
                    [lo[0], hi[1], 0.],
                ],
            },
            holes: vec![],
        }],
        source: serde_json::json!({}),
    }
}

fn spec(top: f64) -> StockSpec {
    StockSpec {
        version: 1,
        units: "mm".into(),
        min_mm: [0., 0., 0.],
        max_mm: [40., 30., top],
        cell_size_mm: 0.1,
        max_cells: 1 << 22,
    }
}

fn operation(surface: SurfaceMesh, levels: Vec<DeburrLevel>) -> DeburrOperation {
    DeburrOperation {
        id: "deburr".into(),
        tool: "c90".into(),
        surface,
        levels,
        width_mm: 0.5,
        tip_clearance_mm: 0.3,
        clearance_z_mm: 70.,
        cutting_sense: "climb".into(),
        feed_mm_min: 600.,
        plunge_mm_min: 200.,
        boundary_tolerance_mm: 0.01,
        tolerance_mm: 1e-5,
        spindle_rpm: 10000.,
        work_offset: 1,
        tool_number: 2,
        length_register: 2,
        coolant: Coolant::Flood,
        retract_z_mm: None,
        feed_z_mm: None,
        limits: None,
    }
}

/// Cuts straight moves through the stock with the flat end mill.
fn machine(stock: &mut Stock, start: Vec3, path: &[Vec3]) {
    let em = end_mill();
    let mut position = start;
    for (i, end) in path.iter().enumerate() {
        let m = Motion {
            id: i as u64 + 1,
            operation: "rough".into(),
            tool: "em6".into(),
            kind: MoveKind::Cut,
            curve: Curve::Line {
                start: position,
                end: *end,
            },
            feed_mm_min: Some(500.),
        };
        commit(stock, &em, &m);
        position = *end;
    }
}

fn commit(stock: &mut Stock, tool: &ToolAssembly, motion: &Motion) {
    let t = tool.prepare().unwrap();
    let d = stock
        .preview_sweep(
            &t,
            motion,
            1.,
            SweepOptions::default(),
            &AtomicBool::new(false),
        )
        .unwrap();
    stock.apply_delta(&d).unwrap();
}

/// Height of the lowest tool surface above the tip at radius `r`, from the
/// assembly's own radius profile (non-decreasing upward for this tool).
fn tool_height(t: &camlib::tool::PreparedAssembly, gauge: f64, r: f64) -> Option<f64> {
    if t.radius_at(gauge).unwrap().radius_mm < r {
        return None;
    }
    let (mut lo, mut hi) = (0., gauge);
    for _ in 0..50 {
        let m = (lo + hi) / 2.;
        if t.radius_at(m).unwrap().radius_mm >= r {
            hi = m
        } else {
            lo = m
        }
    }
    Some(hi)
}

/// Dense sample points of the part's surface.
fn samples(triangles: &[[Vec3; 3]], spacing: f64) -> Vec<Vec3> {
    let mut out = vec![];
    for t in triangles {
        let edge = |a: Vec3, b: Vec3| (a[0] - b[0]).hypot(a[1] - b[1]).hypot(a[2] - b[2]);
        let n = ((edge(t[0], t[1]).max(edge(t[1], t[2])).max(edge(t[2], t[0]))) / spacing).ceil()
            as usize;
        for i in 0..=n {
            for j in 0..=n - i {
                let (u, v) = (i as f64 / n as f64, j as f64 / n as f64);
                out.push(std::array::from_fn(|k| {
                    (1. - u - v) * t[0][k] + u * t[1][k] + v * t[2][k]
                }));
            }
        }
    }
    out
}

/// Every sampled part point the tool reaches below its surface must lie in
/// the break zone of the level being cut: on the face within the width of its
/// boundary, or on the wall below the edge down to the break depth.
#[allow(clippy::too_many_arguments)]
fn assert_only_breaks(
    tool: &ToolAssembly,
    motions: &[Motion],
    points: &[Vec3],
    face: ([f64; 2], [f64; 2]),
    z_e: f64,
    width: f64,
    depth: f64,
    tip_z: f64,
) {
    let inset = |q: Vec3| {
        let (lo, hi) = face;
        let dx = (q[0] - lo[0]).min(hi[0] - q[0]);
        let dy = (q[1] - lo[1]).min(hi[1] - q[1]);
        dx.min(dy)
    };
    let slack = 0.02;
    let prepared_tool = tool.prepare().unwrap();
    let reach = prepared_tool
        .radius_at(tool.gauge_length_mm)
        .unwrap()
        .radius_mm;
    for m in motions.iter().filter(|m| m.kind != MoveKind::Rapid) {
        let prepared = m.curve.prepare(1e-5).unwrap();
        let n = (prepared.length_mm() / 0.2).ceil().max(1.) as usize;
        for i in 0..=n {
            let p = prepared.position(i as f64 / n as f64).unwrap();
            if p[2] > tip_z + 1e-9 && m.kind == MoveKind::Cut {
                continue;
            }
            for q in points {
                let r = (q[0] - p[0]).hypot(q[1] - p[1]);
                if r > reach || q[2] <= p[2] {
                    continue;
                }
                let Some(h) = tool_height(&prepared_tool, tool.gauge_length_mm, r) else {
                    continue;
                };
                if q[2] - (p[2] + h) <= 1e-6 {
                    continue;
                }
                let on_face = (q[2] - z_e).abs() <= 1e-6 && inset(*q) <= width + slack;
                let on_wall =
                    q[2] <= z_e + 1e-6 && q[2] >= z_e - depth - slack && inset(*q).abs() <= slack;
                assert!(
                    on_face || on_wall,
                    "{:?} at {p:?} cuts the part at {q:?} outside the break",
                    m.kind
                );
            }
        }
    }
}

#[test]
fn block_top_edges_are_broken_all_round_and_nothing_else() {
    let part = cuboid([5., 5., 0.], [35., 25., 10.]);
    let spec = spec(10.);
    let mut stock = Stock::new(spec.clone()).unwrap();
    // The finished part: clear the ring around the block with a flat end mill.
    machine(
        &mut stock,
        [2., 2., 12.],
        &[
            [2., 2., 0.],
            [38., 2., 0.],
            [38., 28., 0.],
            [2., 28., 0.],
            [2., 2., 0.],
        ],
    );
    let tool = chamfer_mill();
    let op = operation(
        mesh(part.clone()),
        vec![DeburrLevel {
            regions: vec![rect([5., 5.], [35., 25.])],
            z_mm: 10.,
        }],
    );
    let cancel = AtomicBool::new(false);
    let mut plan = DeburrPlan::prepare(op, &tool, &spec, [0., 0., 70.], 1, &cancel).unwrap();
    assert!(
        plan.report().skipped.is_empty(),
        "{:?}",
        plan.report().skipped
    );
    assert_eq!(plan.report().runs, 1);
    let depth = plan.report().wall_depth_mm;
    assert!(
        (depth - 0.5).abs() < 1e-9,
        "45 degrees: equal legs ({depth})"
    );
    let mut motions = vec![];
    while let Some(m) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        commit(&mut stock, &tool, &m);
        motions.push(m);
    }
    plan.finish_report(&cancel).unwrap();
    // The program validates.
    let program = camlib::motion::Program {
        version: 1,
        id: "p".into(),
        revision: "1".into(),
        units: "mm".into(),
        frame: camlib::motion::WorkFrame::identity(),
        tolerance_mm: 1e-5,
        initial_position_mm: [0., 0., 70.],
        tools: vec![tool.clone()],
        events: vec![],
    };
    let mut events = plan.prefix_events();
    events.extend(motions.iter().map(|m| Event::Motion { motion: m.clone() }));
    ProgramValidator::new(&program, None)
        .unwrap()
        .consume(&events)
        .unwrap();
    // Independent: only the break is cut.
    let tip = motions
        .iter()
        .filter(|m| m.kind == MoveKind::Cut)
        .map(|m| m.curve.start()[2])
        .next()
        .unwrap();
    assert!((tip - (10. - 0.8)).abs() < 1e-9, "tip {tip}");
    assert_only_breaks(
        &tool,
        &motions,
        &samples(&part, 0.25),
        ([5., 5.], [35., 25.]),
        10.,
        0.5,
        depth,
        tip,
    );
    // Stock: the 45 degree break inside the width, untouched beyond it.
    let guard = 2. * spec.cell_size_mm;
    let mut broken = 0;
    for i in 0..stock.cells().len() {
        let q = stock.cell_center(i).unwrap();
        if !(5. < q[0] && q[0] < 35. && 5. < q[1] && q[1] < 25.) {
            continue;
        }
        let x = (q[0] - 5.).min(35. - q[0]).min(q[1] - 5.).min(25. - q[1]);
        let h = stock.cells()[i];
        if x > 0.5 + guard {
            assert_eq!(h.lower_mm, 10., "untouched at {q:?}");
        } else if x < 0.5 - guard {
            assert!(
                h.upper_mm <= 10. - (0.5 - x) + guard,
                "broken at {q:?}: {h:?}"
            );
            broken += 1;
        }
    }
    assert!(broken > 100);
}

#[test]
fn edge_against_a_rising_wall_is_skipped_and_reported() {
    // A step: low face [5,20]x[5,25] at Z6, tall block [20,35]x[5,25] to Z12.
    let mut part = cuboid([5., 5., 0.], [20., 25., 6.]);
    part.extend(cuboid([20., 5., 0.], [35., 25., 12.]));
    let tool = chamfer_mill();
    let op = operation(
        mesh(part.clone()),
        vec![DeburrLevel {
            regions: vec![rect([5., 5.], [20., 25.])],
            z_mm: 6.,
        }],
    );
    let spec = spec(12.);
    let cancel = AtomicBool::new(false);
    let mut plan = DeburrPlan::prepare(op, &tool, &spec, [0., 0., 70.], 1, &cancel).unwrap();
    // The finished step: the ring around it to Z0, the low face to Z6.
    let mut stock = Stock::new(spec.clone()).unwrap();
    machine(
        &mut stock,
        // Half a cell into the walls, so the wall cells are certified cut.
        [2.05, 2.05, 14.],
        &[
            [2.05, 2.05, 0.],
            [37.95, 2.05, 0.],
            [37.95, 27.95, 0.],
            [2.05, 27.95, 0.],
            [2.05, 2.05, 0.],
            [2.05, 2.05, 14.],
        ],
    );
    // The low face as a pocket finish: every cell fully inside one sweep
    // (the pass along the tall wall trims it by 0.05).
    let mut face: Vec<Vec3> = vec![];
    for (i, y) in [5.4, 9.4, 13.4, 17.4, 21.4, 24.6].into_iter().enumerate() {
        let (a, b) = if i % 2 == 0 {
            (5.4, 17.05)
        } else {
            (17.05, 5.4)
        };
        face.extend([[a, y, 6.], [b, y, 6.]]);
    }
    face.extend([
        [5.4, 5.4, 6.],
        [17.05, 5.4, 6.],
        [17.05, 24.6, 6.],
        [17.05, 24.6, 14.],
    ]);
    machine(&mut stock, [5.4, 5.4, 14.], &face);
    let report = plan.report();
    assert!(report.runs >= 1);
    // Only stretches at the rising wall are skipped, and all of it is.
    assert!(
        report.skipped_length_mm >= 20.,
        "{}",
        report.skipped_length_mm
    );
    // The shank (radius 3) must stay clear of the wall at x = 20, with two
    // stock cell diagonals (0.28) of margin: nothing nearer x < 16.6 skipped.
    for s in &report.skipped {
        assert!(s.from_mm[0] >= 16.6 && s.to_mm[0] >= 16.6, "{s:?}");
    }
    assert!(
        report.cut_length_mm > 2. * 12. + 20.,
        "{}",
        report.cut_length_mm
    );
    // Inside the tall block (x = 20.4, y 5..25) the edge is concave: nothing
    // to break. The front and back edges near the wall are convex but the
    // shank cannot reach them: unreachable, reported.
    assert!(
        (19.9..20.9).contains(&report.concave_length_mm),
        "{}",
        report.concave_length_mm
    );
    assert!(
        report.unreachable_length_mm > 4. && report.unreachable_length_mm < 10.,
        "{}",
        report.unreachable_length_mm
    );
    let depth = report.wall_depth_mm;
    let mut motions = vec![];
    while let Some(m) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        commit(&mut stock, &tool, &m);
        motions.push(m);
    }
    plan.finish_report(&cancel).unwrap();
    let tip = motions
        .iter()
        .filter(|m| m.kind == MoveKind::Cut)
        .map(|m| m.curve.start()[2])
        .next()
        .unwrap();
    assert_only_breaks(
        &tool,
        &motions,
        &samples(&part, 0.25),
        ([5., 5.], [20., 25.]),
        6.,
        0.5,
        depth,
        tip,
    );
}
