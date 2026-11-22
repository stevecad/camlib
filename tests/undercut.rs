//! Undercut cutters (issue #8): lollipop and dovetail profiles, their sweep on
//! tri-dexel stock checked against an independently sampled sweep, and the
//! height field's refusals.
use camlib::{dexel::*, motion::*, stock::*, stream::*, tool::*};
use std::f64::consts::PI;

fn assembly(cutter: Cutter) -> Result<PreparedAssembly, String> {
    ToolAssembly {
        id: "t".into(),
        cutter,
        stickout_mm: 30.,
        gauge_length_mm: 60.,
        holders: vec![HolderSection {
            z_min_mm: 30.,
            z_max_mm: 60.,
            radius_min_mm: 6.,
            radius_max_mm: 6.,
        }],
    }
    .prepare()
}
fn cutter(shape: CutterShape, diameter: f64, cutting: f64, neck: Option<f64>) -> Cutter {
    Cutter {
        id: "c".into(),
        name: "test".into(),
        shape,
        diameter_mm: diameter,
        cutting_length_mm: cutting,
        overall_length_mm: 50.,
        shank_diameter_mm: 4.,
        flutes: 4,
        center_cutting: false,
        material: "carbide".into(),
        neck: neck.map(|d| Neck {
            diameter_mm: d,
            length_mm: 10.,
        }),
    }
}
fn lollipop() -> PreparedAssembly {
    // A 6 mm sphere meeting a 3 mm neck.
    let top = 3. + (9f64 - 2.25).sqrt();
    assembly(cutter(CutterShape::Lollipop, 6., top, Some(3.))).unwrap()
}
fn dovetail() -> PreparedAssembly {
    assembly(cutter(
        CutterShape::Dovetail {
            flank_angle_rad: 60f64.to_radians(),
        },
        8.,
        2.,
        Some(4.),
    ))
    .unwrap()
}
fn line(id: u64, a: Vec3, b: Vec3) -> Motion {
    Motion {
        id,
        operation: "op".into(),
        tool: "t".into(),
        kind: MoveKind::Cut,
        curve: Curve::Line { start: a, end: b },
        feed_mm_min: Some(300.),
    }
}

#[test]
fn profiles_and_their_validation() {
    let l = lollipop();
    let c = l.cutter();
    assert!((c.widest_height() - 3.).abs() < 1e-12);
    assert!((c.radius_at(3.).unwrap().radius_mm - 3.).abs() < 1e-12);
    // The sphere narrows above its equator, down to the neck.
    let top = c.definition().cutting_length_mm;
    assert!((c.radius_at(top).unwrap().radius_mm - 1.5).abs() < 1e-9);
    let h = c.heights_at_radius(1.5).unwrap();
    assert!((h[0] - (3. - 6.75f64.sqrt())).abs() < 1e-12 && (h[1] - top).abs() < 1e-9);
    assert!(c.heights_at_radius(3.1).is_none());
    let d = dovetail();
    let c = d.cutter();
    assert_eq!(c.widest_height(), 0.);
    let slope = 60f64.to_radians().tan();
    assert!((c.radius_at(1.).unwrap().radius_mm - (4. - 1. / slope)).abs() < 1e-12);
    let h = c.heights_at_radius(3.5).unwrap();
    assert!(h[0] == 0. && (h[1] - 0.5 * slope).abs() < 1e-12);
    // Refusals: a lollipop needs a neck that fits where the sphere ends, a
    // dovetail a flank that ends no narrower than its neck.
    assert!(assembly(cutter(CutterShape::Lollipop, 6., 5., None)).is_err());
    assert!(assembly(cutter(CutterShape::Lollipop, 6., 7., Some(3.))).is_err());
    assert!(assembly(cutter(CutterShape::Lollipop, 6., 5.9, Some(3.))).is_err());
    let narrow = CutterShape::Dovetail {
        flank_angle_rad: 45f64.to_radians(),
    };
    assert!(assembly(cutter(narrow, 8., 3., Some(4.))).is_err());
    assert!(CutterShape::Lollipop.undercuts() && !CutterShape::Ball.undercuts());
}

/// Whether `q` lies inside the cutting part at tip `p` (+Z), grown or shrunk
/// by |grow|, from the profile alone (independent of the dexel kernel).
fn inside(tool: &PreparedAssembly, p: Vec3, q: Vec3, grow: f64) -> bool {
    let c = tool.cutter();
    let lc = c.definition().cutting_length_mm;
    let h = q[2] - p[2];
    let rho = (q[0] - p[0]).hypot(q[1] - p[1]);
    let r = |z: f64| c.radius_at(z.clamp(0., lc)).unwrap().radius_mm;
    let g = grow.abs();
    if grow >= 0. {
        if h < -g || h > lc + g {
            return false;
        }
        let mut m = r(h - g).max(r(h + g));
        let w = c.widest_height();
        if h - g <= w && w <= h + g {
            m = m.max(r(w));
        }
        rho <= m + g
    } else {
        if h < g || h > lc - g {
            return false;
        }
        rho + g <= r(h - g).min(r(h + g))
    }
}

/// Certain material is never inside the sampled sweep (shrunk), and points
/// outside possible material are inside it (grown), at random points of
/// every ray.
#[test]
fn undercut_cutters_remove_within_their_bounds() {
    for tool in [lollipop(), dovetail()] {
        let spec = DexelStockSpec {
            version: 1,
            units: "mm".into(),
            model: DexelModel::TriDexel,
            min_mm: [0., 0., 0.],
            max_mm: [10., 10., 6.],
            cell_size_mm: 0.25,
            max_rays: 1_000_000,
            max_intervals: 10_000_000,
        };
        let mut stock = DexelStock::new(spec.clone()).unwrap();
        // Side cuts where the narrowing part of the cutter lies inside the
        // block (the dovetail's neck stays above the block's top).
        let z = if tool.cutter().widest_height() > 0. {
            [1.5, 1., 2.]
        } else {
            [4.5, 4.2, 4.6]
        };
        let moves = vec![
            line(1, [-5., 3., z[0]], [15., 3.5, z[0]]),
            line(2, [-5., 7., z[1]], [15., 6.5, z[2]]),
        ];
        for m in &moves {
            let d = stock
                .preview_sweep(&tool, m, 1., SweepOptions::default(), &|| false)
                .unwrap();
            stock.apply_delta(&d).unwrap();
        }
        let mut poses = vec![];
        for m in &moves {
            let p = m.curve.prepare(1e-5).unwrap();
            for i in 0..=8000 {
                poses.push(p.position(i as f64 / 8000.).unwrap());
            }
        }
        let tol = 0.01;
        let grid = stock.grid();
        let mut seed = 11u64;
        let mut next = || {
            seed = seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        let mut checked = [0; 2];
        for a in 0..3 {
            let [u, v] = DexelGrid::cross(a);
            for (index, ray) in stock.rays(a).iter().enumerate() {
                let c = stock.ray_center(a, index);
                for _ in 0..2 {
                    let mut q = [0.; 3];
                    q[u] = c[0] + (next() - 0.5) * grid.step_mm[u];
                    q[v] = c[1] + (next() - 0.5) * grid.step_mm[v];
                    q[a] = spec.min_mm[a] + next() * (spec.max_mm[a] - spec.min_mm[a]);
                    let t = q[a];
                    let within = |s: &Spans| s.as_slice().iter().any(|i| i[0] <= t && t <= i[1]);
                    if within(&ray.certain) {
                        checked[0] += 1;
                        assert!(!poses.iter().any(|p| inside(&tool, *p, q, -tol)), "{q:?}");
                    }
                    if !within(&ray.possible) {
                        checked[1] += 1;
                        assert!(poses.iter().any(|p| inside(&tool, *p, q, tol)), "{q:?}");
                    }
                }
            }
        }
        assert!(checked[0] > 100 && checked[1] > 50, "{checked:?}");
    }
}

/// A height field cannot hold what these cutters leave: its sweep and a
/// height-field run refuse them, naming tri-dexel stock.
#[test]
fn height_field_refuses_narrowing_cutters() {
    let spec = StockSpec {
        version: 1,
        units: "mm".into(),
        min_mm: [0.; 3],
        max_mm: [10., 10., 6.],
        cell_size_mm: 0.25,
        max_cells: 1_000_000,
    };
    let stock = Stock::new(spec.clone()).unwrap();
    for tool in [lollipop(), dovetail()] {
        let e = stock
            .preview_sweep(
                &tool,
                &line(1, [-5., 3., 1.5], [15., 3., 1.5]),
                1.,
                SweepOptions::default(),
                &|| false,
            )
            .unwrap_err();
        assert!(e.contains("tri-dexel"), "{e}");
        let manifest = StreamManifest {
            initial_stock: None,
            version: 1,
            run: RunIdentity {
                id: "r".into(),
                revision: "1".into(),
            },
            stock: spec.clone().into(),
            tools: vec![tool.definition().clone()],
            initial_position_mm: [0., 0., 20.],
            sweep: SweepOptions::default(),
            operations: vec![],
            verification: None,
            tool_change: None,
            rapid_retracts: false,
            turning: None,
            end_position: None,
        };
        let e = StreamReceiver::new(manifest).err().unwrap();
        assert!(e.contains("tri-dexel"), "{e}");
    }
    let _ = PI;
}
