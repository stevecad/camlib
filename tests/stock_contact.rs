use camlib::{
    motion::*,
    stock::*,
    stock_contact::{self, Options},
    tool::*,
    verification::{Status, ToolPart},
};
use std::{cell::Cell, sync::atomic::AtomicBool};
fn spec(cell: f64) -> StockSpec {
    StockSpec {
        version: 1,
        units: "mm".into(),
        min_mm: [0.; 3],
        max_mm: [20., 20., 10.],
        cell_size_mm: cell,
        max_cells: 1_048_576,
    }
}
fn tool(shape: CutterShape) -> ToolAssembly {
    ToolAssembly {
        id: "t1".into(),
        cutter: Cutter {
            id: "c1".into(),
            name: "test".into(),
            shape,
            diameter_mm: 2.,
            cutting_length_mm: 4.,
            overall_length_mm: 20.,
            shank_diameter_mm: 1.,
            flutes: 2,
            center_cutting: true,
            material: "carbide".into(),
            neck: None,
        },
        stickout_mm: 9.,
        gauge_length_mm: 25.,
        holders: vec![HolderSection {
            z_min_mm: 9.,
            z_max_mm: 25.,
            radius_min_mm: 2.,
            radius_max_mm: 3.,
        }],
    }
}
fn line(a: Vec3, b: Vec3, kind: MoveKind) -> Motion {
    Motion {
        id: 1,
        operation: "test".into(),
        tool: "t1".into(),
        kind,
        curve: Curve::Line { start: a, end: b },
        feed_mm_min: if kind == MoveKind::Rapid {
            None
        } else {
            Some(100.)
        },
    }
}
fn check(stock: &Stock, t: &ToolAssembly, m: &Motion) -> stock_contact::Report {
    stock_contact::check(
        stock,
        &t.prepare().unwrap(),
        m,
        Options::default(),
        &AtomicBool::new(false),
    )
    .unwrap()
}
fn prepared() -> Stock {
    let mut s = Stock::new(spec(0.25)).unwrap();
    let stats = s.enable_contact_queries(&AtomicBool::new(false)).unwrap();
    assert_eq!(stats.scanned_cells, 0);
    s
}
fn snapshot(lower: impl Fn([f64; 2]) -> f64, upper: impl Fn([f64; 2]) -> f64) -> Stock {
    let mut s = Stock::new(spec(0.25)).unwrap().snapshot();
    let dims = 80;
    for (i, h) in s.cells.iter_mut().enumerate() {
        let p = [
            (i % dims) as f64 * 0.25 + 0.125,
            (i / dims) as f64 * 0.25 + 0.125,
        ];
        *h = HeightBounds {
            lower_mm: lower(p),
            upper_mm: upper(p),
        };
    }
    let mut s = Stock::from_snapshot(s).unwrap();
    let stats = s.enable_contact_queries(&AtomicBool::new(false)).unwrap();
    assert_eq!(stats.scanned_cells, 6400);
    s
}

#[test]
fn rapid_between_clear_endpoints_hits_stock_and_air_sweeps_are_clear() {
    let s = prepared();
    let t = tool(CutterShape::Flat);
    let r = check(
        &s,
        &t,
        &line([-5., 10., 5.], [25., 10., 5.], MoveKind::Rapid),
    );
    assert_eq!(r.status, Status::Violation);
    assert!(r.fully_examined);
    assert_eq!(r.scope, "rapid_assembly");
    let f = &r.findings[0];
    assert_eq!(f.finding.part, Some(ToolPart::Cutter));
    let p = f.contact_point_mm.unwrap();
    assert!(p[0] > 0. && p[0] < 20. && p[1] > 0. && p[1] < 20. && p[2] > 0. && p[2] < 10.);
    assert!(
        f.finding.witness.as_ref().unwrap().fraction > 0.
            && f.finding.witness.as_ref().unwrap().fraction < 1.
    );
    for m in [
        line([-5., 10., 12.], [25., 10., 12.], MoveKind::Rapid),
        line([-5., -5., 5.], [25., -5., 5.], MoveKind::Rapid),
    ] {
        let r = check(&s, &t, &m);
        assert_eq!(r.status, Status::Clear);
        assert!(r.findings.is_empty());
    }
}

#[test]
fn feed_traverse_keeps_cutter_clearance_between_clear_endpoints() {
    let s = prepared();
    let t = tool(CutterShape::Flat);
    let mut m = line([-5., 10., 5.], [25., 10., 5.], MoveKind::Rapid);
    m.feed_mm_min = Some(600.);
    let r = check(&s, &t, &m);
    assert_eq!(r.status, Status::Violation);
    assert_eq!(r.scope, "rapid_assembly");
    assert!(r.fully_examined);
    assert!(
        r.findings
            .iter()
            .any(|f| f.finding.part == Some(ToolPart::Cutter))
    );
    m.kind = MoveKind::Link;
    assert_eq!(check(&s, &t, &m).scope, "fed_noncutting");
}

#[test]
fn all_rotational_profiles_and_conical_holder_witnesses_match_analytic_sections() {
    let s = prepared();
    for shape in [
        CutterShape::Flat,
        CutterShape::Ball,
        CutterShape::Bull {
            corner_radius_mm: 0.2,
        },
        CutterShape::Drill {
            point_angle_rad: std::f64::consts::FRAC_PI_2,
        },
        CutterShape::Chamfer {
            included_angle_rad: std::f64::consts::FRAC_PI_2,
            tip_diameter_mm: 0.2,
        },
    ] {
        let t = tool(shape.clone());
        let m = line([2., 10., 4.], [18., 10., 4.], MoveKind::Rapid);
        let r = check(&s, &t, &m);
        assert_eq!(r.status, Status::Violation);
        let f = r
            .findings
            .iter()
            .find(|f| f.finding.part == Some(ToolPart::Cutter))
            .unwrap();
        let w = f.finding.witness.as_ref().unwrap();
        let p = f.contact_point_mm.unwrap();
        let z = p[2] - w.tip_mm[2];
        let radius = match shape {
            CutterShape::Flat | CutterShape::Beam => 1.,
            CutterShape::Ball => (z.min(1.) * (2. - z.min(1.))).sqrt(),
            CutterShape::Bull {
                corner_radius_mm: c,
            } => 0.8 + (z.min(c) * (2. * c - z.min(c))).sqrt(),
            CutterShape::Drill { .. } => z.min(1.),
            CutterShape::Chamfer { .. } => (0.1 + z).min(1.),
        };
        assert!((p[0] - w.tip_mm[0]).hypot(p[1] - w.tip_mm[1]) < radius);
    }
    let mut t = tool(CutterShape::Flat);
    t.cutter.cutting_length_mm = 2.;
    t.stickout_mm = 4.;
    t.holders[0] = HolderSection {
        z_min_mm: 4.,
        z_max_mm: 25.,
        radius_min_mm: 4.,
        radius_max_mm: 5.,
    };
    for kind in [MoveKind::Rapid, MoveKind::Cut] {
        let r = check(&s, &t, &line([-3., 5., 4.], [-3., 15., 4.], kind));
        assert_eq!(r.status, Status::Violation);
        let f = r
            .findings
            .iter()
            .find(|f| f.finding.part == Some(ToolPart::Holder))
            .unwrap();
        let p = f.contact_point_mm.unwrap();
        assert!(p[0] > 0.);
        assert_eq!(f.finding.holder_section, Some(0));
        if kind == MoveKind::Cut {
            assert_eq!(r.scope, "fed_noncutting");
            assert!(
                r.findings
                    .iter()
                    .all(|f| f.finding.part != Some(ToolPart::Cutter))
            );
        }
    }
}

#[test]
fn cleared_corridor_and_empty_stock_clear_but_height_uncertainty_does_not() {
    let height = |p: [f64; 2]| if p[1] > 8. && p[1] < 12. { 2. } else { 10. };
    let corridor = snapshot(height, height);
    let t = tool(CutterShape::Flat);
    let m = line([1.5, 10., 5.], [18.5, 10., 5.], MoveKind::Rapid);
    let r = check(&corridor, &t, &m);
    assert_eq!(r.status, Status::Clear, "{r:?}");
    assert!(r.stats.nodes > 1);
    let unknown = snapshot(|_| 2., |_| 10.);
    let r = stock_contact::check(
        &unknown,
        &t.prepare().unwrap(),
        &m,
        Options {
            max_nodes: 2000,
            ..Options::default()
        },
        &AtomicBool::new(false),
    )
    .unwrap();
    assert_eq!(r.status, Status::Uncertain);
    assert!(r.findings.iter().all(|f| f.finding.witness.is_none()));
    let empty = snapshot(|_| 0., |_| 0.);
    assert_eq!(
        check(
            &empty,
            &t,
            &line([1., 10., -1.], [19., 10., -1.], MoveKind::Rapid)
        )
        .status,
        Status::Clear
    );
}

#[test]
fn tangency_is_uncertain_but_a_strict_guarded_gap_is_clear() {
    let s = prepared();
    let t = tool(CutterShape::Flat).prepare().unwrap();
    let tangent = line([-1., 1., 5.], [-1., 19., 5.], MoveKind::Rapid);
    let options = Options {
        max_nodes: 2000,
        ..Options::default()
    };
    let r = stock_contact::check(&s, &t, &tangent, options, &AtomicBool::new(false)).unwrap();
    assert_eq!(r.status, Status::Uncertain);
    assert!(r.findings.iter().all(|f| f.finding.witness.is_none()));
    let gap = line([-1.001, 1., 5.], [-1.001, 19., 5.], MoveKind::Rapid);
    assert_eq!(
        stock_contact::check(&s, &t, &gap, options, &AtomicBool::new(false))
            .unwrap()
            .status,
        Status::Clear
    );
}

#[test]
fn fed_vertical_entry_can_clear_shank_after_cutting_earlier_in_same_move() {
    let s = prepared();
    let t = tool(CutterShape::Flat);
    let m = line([10., 10., 12.], [10., 10., 2.], MoveKind::Plunge);
    let r = check(&s, &t, &m);
    assert_eq!(r.status, Status::Clear, "{r:?}");
    assert!(r.stats.removal_certificates > 0);
    assert_eq!(r.stats.parts_examined, 2);
    // At move end the pre-move stock overlaps the shank (Z6..11). Calling that
    // static overlap a collision would be wrong: the larger cutter came first.
    assert!(2. + t.cutter.cutting_length_mm < 10.);
}

#[test]
fn fed_arc_helix_and_non_xy_holder_hits_are_between_clear_endpoints() {
    let s = prepared();
    let mut t = tool(CutterShape::Flat);
    t.cutter.cutting_length_mm = 2.;
    t.stickout_mm = 4.;
    t.holders[0] = HolderSection {
        z_min_mm: 4.,
        z_max_mm: 25.,
        radius_min_mm: 4.,
        radius_max_mm: 5.,
    };
    let curves = [
        Curve::Arc {
            start: [-5., 0., 4.],
            end: [0., -5., 4.],
            center: [-5., -5., 4.],
            plane: Plane::Xy,
            sweep_rad: -std::f64::consts::FRAC_PI_2,
        },
        Curve::Helix {
            start: [-5., 0., 4.],
            end: [0., -5., 3.],
            center: [-5., -5., 4.],
            plane: Plane::Xy,
            sweep_rad: -std::f64::consts::FRAC_PI_2,
        },
        Curve::Arc {
            start: [5., -3., 10.],
            end: [15., -3., 10.],
            center: [10., -3., 10.],
            plane: Plane::Xz,
            sweep_rad: -std::f64::consts::PI,
        },
    ];
    for curve in curves {
        let m = Motion {
            id: 2,
            operation: "curved-stock-contact".into(),
            tool: "t1".into(),
            kind: MoveKind::Cut,
            curve,
            feed_mm_min: Some(100.),
        };
        let r = check(&s, &t, &m);
        assert_eq!(r.status, Status::Violation, "{r:?}");
        let f = r
            .findings
            .iter()
            .find(|f| f.finding.part == Some(ToolPart::Holder))
            .unwrap();
        let w = f.finding.witness.as_ref().unwrap();
        assert!(w.fraction > 0. && w.fraction < 1.);
        let p = f.contact_point_mm.unwrap();
        assert!(p[0] > 0. && p[1] > 0. && p[2] > 0. && p[2] < 10.);
    }
}

#[test]
fn index_tracks_commits_undo_clones_and_rejects_failed_mutations() {
    let mut s = prepared();
    let mut t = tool(CutterShape::Flat);
    t.cutter.cutting_length_mm = 12.;
    t.stickout_mm = 15.;
    t.cutter.overall_length_mm = 20.;
    t.gauge_length_mm = 25.;
    t.holders[0].z_min_mm = 15.;
    let before = s.clone();
    let m = line([2., 10., 3.], [18., 10., 3.], MoveKind::Cut);
    let prepared = t.prepare().unwrap();
    let delta = s
        .preview_sweep(
            &prepared,
            &m,
            1.,
            SweepOptions::default(),
            &AtomicBool::new(false),
        )
        .unwrap();
    s.apply_delta(&delta).unwrap();
    assert!(s.contact_index_stats().unwrap().changed_tiles > 0);
    let rapid = line([2., 10., 5.], [18., 10., 5.], MoveKind::Rapid);
    // Use a narrower tool for a strict interior-clear path through bounded cells.
    let mut narrow = tool(CutterShape::Flat);
    narrow.cutter.diameter_mm = 0.5;
    narrow.cutter.shank_diameter_mm = 0.5;
    assert_eq!(check(&s, &narrow, &rapid).status, Status::Clear);
    let mut rebuilt = Stock::from_snapshot(s.snapshot()).unwrap();
    rebuilt
        .enable_contact_queries(&AtomicBool::new(false))
        .unwrap();
    // History probes must not steal the cell reads needed to decide a clear
    // path. This shifted path cannot use an exact repeated-path certificate.
    let shifted = line([2., 10.25, 5.], [18., 10.25, 5.], MoveKind::Rapid);
    let reference = check(&rebuilt, &narrow, &shifted);
    assert_eq!(reference.status, Status::Clear);
    let bounded = stock_contact::check(
        &s,
        &narrow.prepare().unwrap(),
        &shifted,
        Options {
            max_cell_reads: reference.stats.cell_reads,
            ..Options::default()
        },
        &AtomicBool::new(false),
    )
    .unwrap();
    assert_eq!(
        bounded.status,
        Status::Clear,
        "history exhausted an otherwise sufficient cell budget"
    );
    assert_eq!(
        serde_json::to_value(check(&s, &narrow, &rapid)).unwrap(),
        serde_json::to_value(check(&rebuilt, &narrow, &rapid)).unwrap()
    );
    assert_eq!(check(&before, &narrow, &rapid).status, Status::Violation);
    assert!(s.apply_delta(&delta).is_err());
    assert_eq!(check(&s, &narrow, &rapid).status, Status::Clear);
    s.undo_delta(&delta).unwrap();
    assert_eq!(check(&s, &narrow, &rapid).status, Status::Violation);
    let mut restored = Stock::from_snapshot(s.snapshot()).unwrap();
    restored
        .enable_contact_queries(&AtomicBool::new(false))
        .unwrap();
    assert_eq!(check(&restored, &narrow, &rapid).status, Status::Violation);
}

#[test]
fn repeated_native_cut_proves_a_buried_shank_but_snapshot_and_budget_do_not() {
    let mut s = prepared();
    let mut cutter = tool(CutterShape::Flat);
    cutter.cutter.cutting_length_mm = 12.;
    cutter.stickout_mm = 15.;
    cutter.holders[0].z_min_mm = 15.;
    let cut = line([2., 10., 3.], [18., 10., 3.], MoveKind::Cut);
    let delta = s
        .preview_sweep(
            &cutter.prepare().unwrap(),
            &cut,
            1.,
            SweepOptions::default(),
            &|| false,
        )
        .unwrap();
    s.apply_delta(&delta).unwrap();
    let mut following = tool(CutterShape::Flat);
    following.cutter.cutting_length_mm = 2.;
    following.cutter.shank_diameter_mm = 2.;
    let motion = line([2., 10., 4.], [18., 10., 4.], MoveKind::Cut);
    let native = check(&s, &following, &motion);
    assert_eq!(native.status, Status::Clear);
    assert!(native.stats.removal_certificates > 0);
    let mut rebuilt = Stock::from_snapshot(s.snapshot()).unwrap();
    rebuilt.enable_contact_queries(&|| false).unwrap();
    assert_eq!(
        check(&rebuilt, &following, &motion).status,
        Status::Uncertain
    );
    let capped = stock_contact::check(
        &s,
        &following.prepare().unwrap(),
        &motion,
        Options {
            max_nodes: 1,
            ..Options::default()
        },
        &|| false,
    )
    .unwrap();
    assert_eq!(capped.status, Status::Uncertain);
    assert!(!capped.fully_examined);
}

#[test]
fn fed_neck_contact_above_every_prior_cut_is_definite_and_finding_caps_stay_partial() {
    let s = prepared();
    let mut t = tool(CutterShape::Flat);
    t.cutter.cutting_length_mm = 2.;
    t.cutter.neck = Some(Neck {
        diameter_mm: 1.,
        length_mm: 1.,
    });
    let m = line([2., 10., 5.], [18., 10., 5.], MoveKind::Cut);
    let r = stock_contact::check(
        &s,
        &t.prepare().unwrap(),
        &m,
        Options {
            max_findings: 1,
            ..Options::default()
        },
        &AtomicBool::new(false),
    )
    .unwrap();
    assert_eq!(r.status, Status::Violation);
    assert_eq!(r.findings[0].finding.part, Some(ToolPart::Neck));
    assert!(r.findings[0].contact_point_mm.unwrap()[2] > 7.);
    assert!(!r.fully_examined);
    assert!(r.stats.findings_truncated);
    assert!(r.stats.unexamined_parts > 0);
}

#[test]
fn budgets_cancellation_options_and_source_identity_are_explicit() {
    let mut s = prepared();
    let t = tool(CutterShape::Flat);
    let p = t.prepare().unwrap();
    let m = line([1., 10., 5.], [19., 10., 5.], MoveKind::Rapid);
    let r = stock_contact::check(
        &s,
        &p,
        &m,
        Options {
            max_nodes: 1,
            ..Options::default()
        },
        &AtomicBool::new(false),
    )
    .unwrap();
    assert_ne!(r.status, Status::Clear);
    assert!(!r.fully_examined);
    assert!(r.stats.work_exhausted);
    assert!(stock_contact::check(&s, &p, &m, Options::default(), &AtomicBool::new(true)).is_err());
    let calls = Cell::new(0);
    let cancel = || {
        calls.set(calls.get() + 1);
        calls.get() > 2
    };
    assert!(
        stock_contact::check(
            &s,
            &p,
            &line([1., -5., 5.], [19., -5., 5.], MoveKind::Rapid),
            Options::default(),
            &cancel
        )
        .is_err()
    );
    assert!(s.enable_contact_queries(&AtomicBool::new(true)).is_err());
    assert!(
        stock_contact::check(
            &s,
            &p,
            &m,
            Options {
                max_findings: 0,
                ..Options::default()
            },
            &AtomicBool::new(false)
        )
        .is_err()
    );
    let a = check(&s, &t, &m);
    let mut changed = m.clone();
    changed.id += 1;
    let b = check(&s, &t, &changed);
    assert_ne!(a.motion_sha256, b.motion_sha256);
    assert_eq!(a.pre_generation, 0);
    assert_eq!(a.stock, s.spec().clone());
}
