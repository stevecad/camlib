use camlib::{
    motion::{Curve, Motion, MoveKind, Plane, WorkFrame},
    planar::{Boundary, Component, PlanarRegion},
    tool::{Cutter, CutterShape, HolderSection, Neck, ToolAssembly},
    verification::*,
};
use sha2::{Digest, Sha256};
use std::{
    cell::Cell,
    f64::consts::{FRAC_PI_2, PI, TAU},
    sync::atomic::AtomicBool,
};

fn rectangle(x0: f64, y0: f64, x1: f64, y1: f64) -> Boundary {
    Boundary {
        vertices: vec![[x0, y0, 0.], [x1, y0, 0.], [x1, y1, 0.], [x0, y1, 0.]],
    }
}
fn body(id: &str, xy: [f64; 4], z: [f64; 2]) -> Prism {
    Prism {
        id: id.into(),
        role: BodyRole::Fixture,
        representation: Representation::SolidPrism,
        footprint: PlanarRegion {
            version: 1,
            units: "mm".into(),
            name: id.into(),
            boundary_error_mm: 0.,
            source: serde_json::Value::Null,
            components: vec![Component {
                outer: rectangle(xy[0], xy[1], xy[2], xy[3]),
                holes: vec![],
            }],
        },
        z_min_mm: z[0],
        z_max_mm: z[1],
        z_error_mm: 0.,
    }
}

#[test]
fn outer_envelopes_prove_separation_but_never_source_solid_collision() {
    let mut envelope = body("cad-bound", [-0.2, -0.2, 0.2, 0.2], [1., 2.]);
    envelope.representation = Representation::OuterEnvelope;
    let hit = line([-10., 0., 0.], [10., 0., 0.]);
    let report = check(scene(vec![envelope.clone()]), &hit);
    assert_eq!(report.status, Status::Uncertain);
    assert_eq!(report.coverage.enclosure_only_bodies, 1);
    assert!(
        report
            .findings
            .iter()
            .all(|f| f.witness.is_none() && f.status == Status::Uncertain)
    );
    let separated = check(
        scene(vec![envelope.clone()]),
        &line([-10., 10., 0.], [10., 10., 0.]),
    );
    assert_eq!(separated.status, Status::Clear);
    assert!(separated.findings.is_empty());
    let envelope_digest = scene(vec![envelope.clone()])
        .prepare(Options::default())
        .unwrap()
        .digest()
        .to_owned();
    envelope.representation = Representation::SolidPrism;
    let exact = check(scene(vec![envelope.clone()]), &hit);
    assert_eq!(exact.status, Status::Violation);
    assert_ne!(exact.scene_sha256, envelope_digest);
    let serialized = serde_json::to_string(&envelope).unwrap();
    assert!(!serialized.contains("representation"));
    envelope.representation = Representation::OuterEnvelope;
    envelope.role = BodyRole::ProtectedTarget;
    let r = check(scene(vec![envelope]), &hit);
    assert_eq!(r.status, Status::Uncertain);
    assert_eq!(r.findings[0].kind, FindingKind::Gouge);
}
fn scene(bodies: Vec<Prism>) -> Scene {
    Scene {
        version: 1,
        units: "mm".into(),
        id: "scene".into(),
        revision: "cad-1".into(),
        work_to_machine: WorkFrame::identity(),
        bodies,
        machine: None,
    }
}

#[test]
fn circular_void_enclosures_prove_whole_arcs_but_not_tangency_or_nested_islands() {
    let mut annulus = PlanarRegion::circle("annulus", [0., 0.], 8.).unwrap();
    annulus.components[0].holes.push(
        PlanarRegion::circle("hole", [0., 0.], 4.)
            .unwrap()
            .components
            .remove(0)
            .outer,
    );
    let mut b = body("annulus", [-8., -8., 8., 8.], [1., 2.]);
    b.footprint = annulus;
    let mut m = line([2.9997, 0., 0.], [2.9997, 0., 0.]);
    m.curve = Curve::Arc {
        start: [2.9997, 0., 0.],
        end: [2.9997, 0., 0.],
        center: [0., 0., 0.],
        plane: Plane::Xy,
        sweep_rad: TAU,
    };
    let r = check(scene(vec![b.clone()]), &m);
    assert_eq!(r.status, Status::Clear);
    assert!(r.stats.nodes < 20);
    m.curve = Curve::Arc {
        start: [3., 0., 0.],
        end: [3., 0., 0.],
        center: [0., 0., 0.],
        plane: Plane::Xy,
        sweep_rad: TAU,
    };
    assert_ne!(check(scene(vec![b.clone()]), &m).status, Status::Clear);
    // A second retained component inside the hole disables void shortcuts.
    b.footprint.components.push(
        PlanarRegion::circle("island", [0., 0.], 0.5)
            .unwrap()
            .components
            .remove(0),
    );
    m.curve = Curve::Line {
        start: [0., 0., 0.],
        end: [0., 0., 0.1],
    };
    assert_eq!(check(scene(vec![b]), &m).status, Status::Violation);
}
fn tool() -> ToolAssembly {
    ToolAssembly {
        id: "t1".into(),
        cutter: Cutter {
            id: "c1".into(),
            name: "2mm".into(),
            shape: CutterShape::Flat,
            diameter_mm: 2.,
            cutting_length_mm: 4.,
            overall_length_mm: 15.,
            shank_diameter_mm: 2.,
            flutes: 2,
            center_cutting: true,
            material: "carbide".into(),
            neck: Some(Neck {
                diameter_mm: 1.,
                length_mm: 2.,
            }),
        },
        stickout_mm: 8.,
        gauge_length_mm: 20.,
        holders: vec![HolderSection {
            z_min_mm: 8.,
            z_max_mm: 20.,
            radius_min_mm: 3.,
            radius_max_mm: 4.,
        }],
    }
}
fn motion(curve: Curve) -> Motion {
    Motion {
        id: 7,
        operation: "op".into(),
        tool: "t1".into(),
        kind: MoveKind::Cut,
        curve,
        feed_mm_min: Some(600.),
    }
}
fn line(a: [f64; 3], b: [f64; 3]) -> Motion {
    motion(Curve::Line { start: a, end: b })
}
fn check(s: Scene, m: &Motion) -> Report {
    s.prepare(Options::default())
        .unwrap()
        .verify_motion(&tool().prepare().unwrap(), m, &AtomicBool::new(false))
        .unwrap()
}

#[test]
fn a_clear_diagonal_rapid_does_not_certify_the_controller_dogleg() {
    let s = scene(vec![body("clamp", [4.8, -0.2, 5.2, 0.2], [-0.5, 2.])]);
    let mut diagonal = line([0., 0., 0.], [10., 10., 0.]);
    diagonal.kind = MoveKind::Rapid;
    diagonal.feed_mm_min = None;
    assert_eq!(check(s.clone(), &diagonal).status, Status::Clear);
    let mut dogleg = diagonal;
    dogleg.curve = Curve::Line {
        start: [0., 0., 0.],
        end: [10., 0., 0.],
    };
    let hit = check(s, &dogleg);
    assert_eq!(hit.status, Status::Violation);
    assert_eq!(hit.motion_id, 7);
    assert!(
        hit.findings
            .iter()
            .any(|f| f.object_id == "clamp" && f.part == Some(ToolPart::Cutter))
    );
}

#[test]
fn short_holder_hits_tall_fixture_even_when_cutter_clears() {
    let s = scene(vec![body("tall-clamp", [-0.2, 2.3, 0.2, 2.7], [10., 12.])]);
    let motion = line([-10., 0., 0.], [10., 0., 0.]);
    let short = check(s.clone(), &motion);
    assert_eq!(short.status, Status::Violation);
    assert!(
        short
            .findings
            .iter()
            .any(|f| f.object_id == "tall-clamp" && f.part == Some(ToolPart::Holder))
    );
    assert!(
        !short
            .findings
            .iter()
            .any(|f| f.part == Some(ToolPart::Cutter))
    );
    let mut long = tool();
    long.cutter.overall_length_mm = 30.;
    long.stickout_mm = 25.;
    long.gauge_length_mm = 40.;
    long.holders[0].z_min_mm = 25.;
    long.holders[0].z_max_mm = 40.;
    let clear = s
        .prepare(Options::default())
        .unwrap()
        .verify_motion(&long.prepare().unwrap(), &motion, &|| false)
        .unwrap();
    assert_eq!(clear.status, Status::Clear);
}

#[test]
fn line_between_clear_endpoints_catches_fixture_and_protected_target_gouge() {
    let fixture = body("clamp", [-0.2, -0.2, 0.2, 0.2], [0.5, 1.5]);
    let m = line([-10., 0., 0.], [10., 0., 0.]);
    // Independently: both endpoint axes are9.8mm from the clamp, tool radius1.
    let r = check(scene(vec![fixture.clone()]), &m);
    assert_eq!(r.status, Status::Violation);
    let f = &r.findings[0];
    assert_eq!(f.kind, FindingKind::FixtureContact);
    let w = f.witness.as_ref().unwrap();
    assert_eq!(w.fraction, 0.5);
    assert_eq!(w.tip_mm, [0., 0., 0.]);
    assert!(w.margin_lower_mm > 0.);
    assert_eq!(r.motion_id, 7);
    assert_eq!(r.operation, "op");
    assert!(!r.coverage.evolving_stock_contact && !r.coverage.remaining_material);
    let mut target = fixture;
    target.role = BodyRole::ProtectedTarget;
    assert_eq!(
        check(scene(vec![target]), &m).findings[0].kind,
        FindingKind::Gouge
    );
}

#[test]
fn continuous_separation_proves_clear_and_holes_remain_empty() {
    let m = line([-10., 0., 0.], [10., 0., 0.]);
    let r = check(scene(vec![body("side", [4., 3., 6., 4.], [0., 3.])]), &m);
    assert_eq!(r.status, Status::Clear);
    assert!(r.stats.pruned_intervals > 0);
    let mut donut = body("hole", [-5., -5., 5., 5.], [0., 3.]);
    donut.footprint.components[0]
        .holes
        .push(rectangle(-3., -3., 3., 3.));
    assert_eq!(
        check(scene(vec![donut]), &line([-1., 0., 0.], [1., 0., 0.])).status,
        Status::Clear
    );
}

#[test]
fn arcs_helices_and_non_xy_between_endpoint_excursions_are_checked() {
    let arc = motion(Curve::Arc {
        start: [5., 0., 0.],
        end: [-5., 0., 0.],
        center: [0., 0., 0.],
        plane: Plane::Xy,
        sweep_rad: PI,
    });
    let r = check(
        scene(vec![body("arc", [-0.1, 4.9, 0.1, 5.1], [0.5, 2.])]),
        &arc,
    );
    assert_eq!(r.status, Status::Violation);
    assert!((r.findings[0].witness.as_ref().unwrap().tip_mm[1] - 5.).abs() < 1e-10);
    let helix = motion(Curve::Helix {
        start: [5., 0., 0.],
        end: [5., 0., 8.],
        center: [0., 0., 0.],
        plane: Plane::Xy,
        sweep_rad: 2. * TAU,
    });
    let r = check(
        scene(vec![body("helix", [-0.1, 4.9, 0.1, 5.1], [1.5, 2.5])]),
        &helix,
    );
    assert_eq!(r.status, Status::Violation);
    assert!((r.findings[0].witness.as_ref().unwrap().fraction - 0.125).abs() < 1e-10);
    let xz = motion(Curve::Arc {
        start: [-5., 0., 5.],
        end: [5., 0., 5.],
        center: [0., 0., 5.],
        plane: Plane::Xz,
        sweep_rad: PI,
    });
    let r = check(
        scene(vec![body("over", [-0.1, -0.1, 0.1, 0.1], [10.5, 11.5])]),
        &xz,
    );
    assert!(
        r.findings
            .iter()
            .any(|f| f.part == Some(ToolPart::Cutter) && f.status == Status::Violation)
    );
}

#[test]
fn all_profiles_have_interior_witnesses_and_ball_tip_is_not_a_cylinder() {
    let m = line([-0.01, 0., 0.], [0.01, 0., 0.]);
    for shape in [
        CutterShape::Flat,
        CutterShape::Ball,
        CutterShape::Bull {
            corner_radius_mm: 0.4,
        },
        CutterShape::Drill {
            point_angle_rad: FRAC_PI_2,
        },
        CutterShape::Chamfer {
            included_angle_rad: FRAC_PI_2,
            tip_diameter_mm: 0.1,
        },
    ] {
        let mut t = tool();
        t.cutter.shape = shape;
        let s = scene(vec![body("tip", [0.1, -0.05, 0.2, 0.05], [0.25, 0.35])])
            .prepare(Options::default())
            .unwrap();
        let r = s
            .verify_motion(&t.prepare().unwrap(), &m, &AtomicBool::new(false))
            .unwrap();
        assert_eq!(r.status, Status::Violation, "{:?}", t.cutter.shape);
        assert_eq!(r.findings[0].part, Some(ToolPart::Cutter));
    }
    let s = scene(vec![body("near", [0.8, -0.05, 0.9, 0.05], [0.05, 0.1])])
        .prepare(Options::default())
        .unwrap();
    // Independent ball formula at z=.1: sqrt(1-.9²)<.44, well below.79 gap.
    let mut t = tool();
    t.cutter.shape = CutterShape::Ball;
    assert_eq!(
        s.verify_motion(&t.prepare().unwrap(), &m, &AtomicBool::new(false))
            .unwrap()
            .status,
        Status::Clear
    );
    assert_eq!(
        s.verify_motion(&tool().prepare().unwrap(), &m, &AtomicBool::new(false))
            .unwrap()
            .status,
        Status::Violation
    );
}

#[test]
fn neck_shank_and_holder_are_checked_even_for_fed_moves_and_rapid_cutters() {
    let m = line([-0.01, 0., 0.], [0.01, 0., 0.]);
    for (name, z, part) in [
        ("neck", [4.2, 4.8], ToolPart::Neck),
        ("shank", [6.2, 7.8], ToolPart::Shank),
        ("holder", [9., 10.], ToolPart::Holder),
    ] {
        let r = check(scene(vec![body(name, [0.1, -0.05, 0.2, 0.05], z)]), &m);
        assert_eq!(r.findings.len(), 1, "{r:?}");
        assert_eq!(r.findings[0].part, Some(part));
        assert_eq!(r.status, Status::Violation);
        if part == ToolPart::Holder {
            assert_eq!(r.findings[0].holder_section, Some(0));
        }
    }
    let mut rapid = m;
    rapid.kind = MoveKind::Rapid;
    rapid.feed_mm_min = None;
    assert_eq!(
        check(
            scene(vec![body("rapid", [0.1, -0.05, 0.2, 0.05], [0.5, 1.])]),
            &rapid
        )
        .status,
        Status::Violation
    );
}

#[test]
fn tangency_z_contact_source_error_and_resolution_never_report_clear() {
    let b = body("tangent", [-2., -1., 2., 1.], [0., 3.]);
    let s = scene(vec![b])
        .prepare(Options {
            spatial_resolution_mm: 0.05,
            ..Options::default()
        })
        .unwrap();
    let r = s
        .verify_motion(
            &tool().prepare().unwrap(),
            &line([-1., 2., 0.], [1., 2., 0.]),
            &AtomicBool::new(false),
        )
        .unwrap();
    assert_eq!(r.status, Status::Uncertain);
    assert!(r.findings.iter().all(|f| f.witness.is_none()));
    assert_eq!(
        check(
            scene(vec![body("top", [-2., -1., 2., 1.], [0., 3.])]),
            &line([-1., 0., 3.], [1., 0., 3.])
        )
        .status,
        Status::Uncertain
    );
    let mut b = body("error", [-2., -1., 2., 1.], [0., 3.]);
    b.footprint.boundary_error_mm = 0.1;
    let r = check(scene(vec![b]), &line([-1., 2.05, 0.], [1., 2.05, 0.]));
    assert_eq!(r.status, Status::Uncertain);
}

#[test]
fn circular_footprint_and_endpoint_mismatch_keep_error_in_bounds() {
    let mut b = body("circle", [-1., -1., 1., 1.], [0., 3.]);
    b.footprint = PlanarRegion::circle("round", [0., 0.], 1.).unwrap();
    let m = line([-5., 2.01, 0.], [5., 2.01, 0.]);
    assert_eq!(check(scene(vec![b.clone()]), &m).status, Status::Clear);
    let arc = motion(Curve::Arc {
        start: [5., 0., 0.],
        end: [-5., 0.000005, 0.],
        center: [0., 0., 0.],
        plane: Plane::Xy,
        sweep_rad: PI,
    });
    let r = check(scene(vec![b]), &arc);
    assert_eq!(r.status, Status::Clear);
}

#[test]
fn nominal_normal_acceleration_is_analytic_for_all_planes_directions_and_helical_pitch() {
    for plane in [Plane::Xy, Plane::Xz, Plane::Yz] {
        for sign in [-1., 1.] {
            let [u, _, w] = plane.axes();
            let mut start = [0.; 3];
            start[u] = 2.;
            let sweep = sign * 2. * TAU;
            let arc = motion(Curve::Arc {
                start,
                end: start,
                center: [0.; 3],
                plane,
                sweep_rad: sweep,
            });
            let mut s = scene(vec![]);
            s.machine = Some(MachineLimits {
                min_tip_mm: Some([-1000.; 3]),
                max_tip_mm: Some([1000.; 3]),
                max_feed_mm_min: 1000.,
                max_normal_accel_mm_s2: Some(40.),
                max_axis_feed_mm_min: None,
                min_gauge_mm: None,
                max_gauge_mm: None,
            });
            s.work_to_machine.origin_mm = [100., 20., -5.];
            s.work_to_machine.x = [0., 1., 0.];
            s.work_to_machine.y = [-1., 0., 0.];
            let a = arc
                .curve
                .prepare(1e-5)
                .unwrap()
                .nominal_normal_acceleration(600.)
                .unwrap();
            assert!((a - 50.).abs() < 1e-10);
            let report = check(s.clone(), &arc);
            assert_eq!(report.status, Status::Violation);
            assert!(report.coverage.nominal_normal_accel_checked);
            assert!(
                report
                    .findings
                    .iter()
                    .any(|f| f.kind == FindingKind::NormalAccelerationLimit && f.witness.is_none())
            );
            let mut end = start;
            end[w] = 2. * sweep;
            let helix = motion(Curve::Helix {
                start,
                end,
                center: [0.; 3],
                plane,
                sweep_rad: sweep,
            });
            // Pitch/radian2: curvature2/(2^2+2^2)=1/4, acceleration25.
            assert!(
                (helix
                    .curve
                    .prepare(1e-5)
                    .unwrap()
                    .nominal_normal_acceleration(600.)
                    .unwrap()
                    - 25.)
                    .abs()
                    < 1e-10
            );
            assert_eq!(check(s.clone(), &helix).status, Status::Clear);
            s.machine.as_mut().unwrap().max_normal_accel_mm_s2 = Some(50.);
            assert_eq!(check(s.clone(), &arc).status, Status::Uncertain);
            s.machine.as_mut().unwrap().max_normal_accel_mm_s2 = Some(80.);
            assert_eq!(check(s.clone(), &arc).status, Status::Clear);
            assert_eq!(
                check(s, &line([0.; 3], [10., 0., 0.])).status,
                Status::Clear
            );
        }
    }
}

#[test]
fn normal_acceleration_limits_bind_identity_scope_validation_and_finding_caps() {
    let mut s = scene(vec![]);
    s.machine = Some(MachineLimits {
        min_tip_mm: Some([-100.; 3]),
        max_tip_mm: Some([100.; 3]),
        max_feed_mm_min: 500.,
        max_normal_accel_mm_s2: None,
        max_axis_feed_mm_min: None,
        min_gauge_mm: None,
        max_gauge_mm: None,
    });
    let before = s.prepare(Options::default()).unwrap().digest().to_owned();
    s.machine.as_mut().unwrap().max_normal_accel_mm_s2 = Some(40.);
    let prepared = s
        .prepare(Options {
            max_findings: 1,
            ..Default::default()
        })
        .unwrap();
    assert_ne!(before, prepared.digest());
    let m = motion(Curve::Arc {
        start: [2., 0., 0.],
        end: [2., 0., 0.],
        center: [0.; 3],
        plane: Plane::Xy,
        sweep_rad: TAU,
    });
    let report = prepared
        .verify_motion(&tool().prepare().unwrap(), &m, &|| false)
        .unwrap();
    assert_eq!(report.findings.len(), 1);
    assert!(report.stats.findings_truncated);
    let mut rapid = m.clone();
    rapid.kind = MoveKind::Rapid;
    rapid.feed_mm_min = None;
    assert!(
        prepared
            .verify_motion(&tool().prepare().unwrap(), &rapid, &|| false)
            .is_err()
    );
    rapid.curve = Curve::Line {
        start: [0.; 3],
        end: [1., 0., 0.],
    };
    let report = prepared
        .verify_motion(&tool().prepare().unwrap(), &rapid, &|| false)
        .unwrap();
    assert!(!report.coverage.nominal_normal_accel_checked);
    assert_eq!(report.status, Status::Clear);
    assert!(
        prepared
            .verify_motion(&tool().prepare().unwrap(), &m, &|| true)
            .is_err()
    );
    for invalid in [0., -1., f64::NAN, f64::INFINITY, 1e10] {
        s.machine.as_mut().unwrap().max_normal_accel_mm_s2 = Some(invalid);
        assert!(s.prepare(Options::default()).is_err());
    }
}

#[test]
fn machine_travel_includes_arc_extrema_frames_and_declared_feed_limit() {
    let mut s = scene(vec![]);
    s.machine = Some(MachineLimits {
        min_tip_mm: Some([-6., -1., -50.]),
        max_tip_mm: Some([6., 4., 50.]),
        max_feed_mm_min: 500.,
        max_normal_accel_mm_s2: None,
        max_axis_feed_mm_min: None,
        min_gauge_mm: None,
        max_gauge_mm: None,
    });
    let arc = motion(Curve::Arc {
        start: [5., 0., 0.],
        end: [-5., 0., 0.],
        center: [0., 0., 0.],
        plane: Plane::Xy,
        sweep_rad: PI,
    });
    let r = check(s.clone(), &arc);
    assert_eq!(r.status, Status::Violation);
    assert!(r.findings.iter().any(|f| f.kind == FindingKind::FeedLimit));
    let travel = r
        .findings
        .iter()
        .find(|f| f.kind == FindingKind::MachineTravel)
        .unwrap();
    assert_eq!(travel.witness.as_ref().unwrap().fraction, 0.5);
    s.machine = Some(MachineLimits {
        min_tip_mm: Some([90., -10., -10.]),
        max_tip_mm: Some([110., 10., 30.]),
        max_feed_mm_min: 1000.,
        max_normal_accel_mm_s2: None,
        max_axis_feed_mm_min: None,
        min_gauge_mm: None,
        max_gauge_mm: None,
    });
    s.work_to_machine.origin_mm = [100., 0., 0.];
    s.work_to_machine.x = [0., 1., 0.];
    s.work_to_machine.y = [-1., 0., 0.];
    let r = check(s.clone(), &line([-1., 0., 0.], [1., 0., 0.]));
    assert_eq!(r.status, Status::Clear);
    assert!(r.coverage.machine_tip_travel_examined && r.coverage.declared_fed_rate_checked);
    s.work_to_machine.origin_mm = [0., 0., 0.];
    assert_eq!(
        check(s, &line([-1., 0., 0.], [1., 0., 0.])).status,
        Status::Violation
    );
}

#[test]
fn machine_travel_is_closed_a_tip_on_its_limit_is_inside() {
    // #166: a mill at home sits on its travel limit (X 0, Y 0, Z top).
    let mut s = scene(vec![]);
    s.machine = Some(MachineLimits {
        min_tip_mm: Some([0., 0., -515.]),
        max_tip_mm: Some([500., 400., -115.]),
        max_feed_mm_min: 10000.,
        max_normal_accel_mm_s2: None,
        max_axis_feed_mm_min: None,
        min_gauge_mm: None,
        max_gauge_mm: None,
    });
    s.work_to_machine.origin_mm = [200., 150., -300.];
    let home = [-200., -150., 185.];
    for (name, m) in [
        ("down from home", line(home, [-200., -150., 150.])),
        (
            "along the X0/Y0/Z-top edge",
            line(home, [-100., -150., 185.]),
        ),
        ("diagonal into home", line([-150., -100., 100.], home)),
        (
            "arc touching the top",
            motion(Curve::Arc {
                start: [-100., -150., 185.],
                end: [-110., -150., 175.],
                center: [-100., -150., 175.],
                plane: Plane::Xz,
                sweep_rad: -FRAC_PI_2,
            }),
        ),
    ] {
        let r = check(s.clone(), &m);
        assert_eq!(r.status, Status::Clear, "{name}: {:?}", r.findings);
        assert!(r.coverage.machine_tip_travel_examined, "{name}");
    }
    // One output unit beyond the limit is still a violation, at the endpoint.
    let r = check(
        s.clone(),
        &line([-200., -150., 150.], [-200., -150., 185.001]),
    );
    assert_eq!(r.status, Status::Violation);
    let travel = r
        .findings
        .iter()
        .find(|f| f.kind == FindingKind::MachineTravel)
        .unwrap();
    assert_eq!(travel.witness.as_ref().unwrap().fraction, 1.);
    // An arc whose middle crosses the top is still found by subdivision.
    let over = motion(Curve::Arc {
        start: [-100., -150., 181.],
        end: [-90., -150., 181.],
        center: [-95., -150., 181.],
        plane: Plane::Xz,
        sweep_rad: PI,
    });
    assert_eq!(check(s.clone(), &over).status, Status::Violation);
    // A rotated frame: the line ends exactly on the +Y limit.
    s.machine = Some(MachineLimits {
        min_tip_mm: Some([90., -10., -10.]),
        max_tip_mm: Some([110., 10., 30.]),
        max_feed_mm_min: 1000.,
        max_normal_accel_mm_s2: None,
        max_axis_feed_mm_min: None,
        min_gauge_mm: None,
        max_gauge_mm: None,
    });
    s.work_to_machine.origin_mm = [100., 0., 0.];
    s.work_to_machine.x = [0., 1., 0.];
    s.work_to_machine.y = [-1., 0., 0.];
    assert_eq!(
        check(s.clone(), &line([0., 0., 0.], [10., 0., 0.])).status,
        Status::Clear
    );
    assert_eq!(
        check(s, &line([0., 0., 0.], [10.001, 0., 0.])).status,
        Status::Violation
    );
}

#[test]
fn budgets_truncation_and_mid_query_cancellation_cannot_fabricate_completion() {
    let s = scene(vec![body("small", [4.9, -0.1, 5.1, 0.1], [0., 2.])]);
    let m = line([-10., 0., 0.], [10., 0., 0.]);
    let r = s
        .prepare(Options {
            max_nodes: 1,
            ..Options::default()
        })
        .unwrap()
        .verify_motion(&tool().prepare().unwrap(), &m, &AtomicBool::new(false))
        .unwrap();
    assert_eq!(r.status, Status::Uncertain);
    assert!(r.stats.budget_exhausted);
    assert!(r.stats.unexamined_pairs > 0);
    assert!(r.findings[0].witness.is_none());
    let mut bodies = vec![body("a", [-1., -1., 1., 1.], [0.5, 2.])];
    let mut other = bodies[0].clone();
    other.id = "b".into();
    bodies.push(other);
    let r = scene(bodies)
        .prepare(Options {
            max_findings: 1,
            ..Options::default()
        })
        .unwrap()
        .verify_motion(&tool().prepare().unwrap(), &m, &AtomicBool::new(false))
        .unwrap();
    assert_eq!(r.status, Status::Violation);
    assert!(r.stats.findings_truncated && r.stats.unexamined_pairs > 0);
    assert_eq!(r.findings.len(), 1);
    let prepared = s.prepare(Options::default()).unwrap();
    let polls = Cell::new(0);
    let cancel = || {
        polls.set(polls.get() + 1);
        polls.get() > 3
    };
    assert!(
        prepared
            .verify_motion(&tool().prepare().unwrap(), &m, &cancel)
            .unwrap_err()
            .contains("cancelled")
    );
    assert_eq!(
        prepared
            .verify_motion(&tool().prepare().unwrap(), &m, &AtomicBool::new(false))
            .unwrap()
            .status,
        Status::Violation
    );
}

#[test]
fn invalid_empty_duplicate_geometry_limits_tool_and_motion_are_rejected() {
    assert!(scene(vec![]).prepare(Options::default()).is_err());
    let b = body("x", [-1., -1., 1., 1.], [0., 3.]);
    assert!(
        scene(vec![b.clone(), b.clone()])
            .prepare(Options::default())
            .is_err()
    );
    let prepared = scene(vec![b]).prepare(Options::default()).unwrap();
    let mut m = line([-10., 0., 0.], [10., 0., 0.]);
    m.tool = "other".into();
    assert!(
        prepared
            .verify_motion(&tool().prepare().unwrap(), &m, &AtomicBool::new(false))
            .is_err()
    );
    assert!(
        scene(vec![])
            .prepare(Options {
                max_nodes: 0,
                ..Options::default()
            })
            .is_err()
    );
}

#[test]
fn independent_disk_capsule_oracle_checks_many_nontrivial_line_sweeps() {
    let mut fixture = body("circle", [-1., -1., 1., 1.], [0.5, 1.5]);
    fixture.footprint = PlanarRegion::circle("disk", [0., 0.], 0.4).unwrap();
    let s = scene(vec![fixture]).prepare(Options::default()).unwrap();
    let t = tool().prepare().unwrap();
    let mut checked = [0; 2];
    for i in 0..128 {
        // Independent analytic center-to-segment distance. Rotating endpoints
        // changes the full interval search, while the circular oracle stays exact.
        let angle = i as f64 * 0.391;
        let y = (i as f64 * 0.173).sin() * 3.;
        let rotate = |x: f64| {
            [
                x * angle.cos() - y * angle.sin(),
                x * angle.sin() + y * angle.cos(),
                0.,
            ]
        };
        let (a, b) = (rotate(-7.), rotate(4.));
        let v = [b[0] - a[0], b[1] - a[1]];
        let f = (-(a[0] * v[0] + a[1] * v[1]) / (v[0] * v[0] + v[1] * v[1])).clamp(0., 1.);
        let gap = (a[0] + f * v[0]).hypot(a[1] + f * v[1]) - 1.4;
        if gap.abs() < 0.005 {
            continue;
        }
        let result = s
            .verify_motion(&t, &line(a, b), &AtomicBool::new(false))
            .unwrap();
        let expected = if gap < 0. {
            checked[0] += 1;
            Status::Violation
        } else {
            checked[1] += 1;
            Status::Clear
        };
        assert_eq!(
            result.status, expected,
            "case{i}, analytic gap{gap}: {result:?}"
        );
        let reversed = s
            .verify_motion(&t, &line(b, a), &AtomicBool::new(false))
            .unwrap();
        assert_eq!(reversed.status, expected);
    }
    assert!(checked[0] > 20 && checked[1] > 20);
}

#[test]
fn streamed_scene_identity_matches_canonical_bytes_and_binds_options_and_provenance() {
    let mut s = scene(vec![body("x", [-1., -1., 1., 1.], [0., 3.])]);
    s.bodies[0].footprint.source =
        serde_json::json!({"cad_reference":"fixture.face","provenance":"x".repeat(100000)});
    let options = Options::default();
    let expected = format!(
        "{:x}",
        Sha256::digest(serde_json::to_vec(&(&s, options)).unwrap())
    );
    let prepared = s.prepare(options).unwrap();
    assert_eq!(prepared.digest(), expected);
    let changed = s
        .prepare(Options {
            max_nodes: options.max_nodes - 1,
            ..options
        })
        .unwrap();
    assert_ne!(prepared.digest(), changed.digest());
    s.bodies[0].footprint.source = serde_json::json!({"cad_reference":"fixture.other"});
    assert_ne!(prepared.digest(), s.prepare(options).unwrap().digest());
}

#[test]
fn analytic_circle_enclosure_resolves_tight_following_and_respects_sweep_direction() {
    let mut b = body("round", [-5., -5., 5., 5.], [0.5, 2.]);
    b.footprint = PlanarRegion::circle("round", [0., 0.], 5.).unwrap();
    // Preserve a real CAD near-semicircle bulge. Its diameter circle encloses it.
    for p in &mut b.footprint.components[0].outer.vertices {
        p[2] = 0.9999999999999999;
    }
    let radius = 6.0002;
    let m = motion(Curve::Arc {
        start: [radius, 0., 0.],
        end: [-radius, 0., 0.],
        center: [0., 0., 0.],
        plane: Plane::Xy,
        sweep_rad: PI,
    });
    let r = check(scene(vec![b]), &m);
    assert_eq!(r.status, Status::Clear);
    assert!(
        r.stats.nodes < 10,
        "analytic concentric separation should not subdivide"
    );
    for sign in [-1., 1.] {
        let m = motion(Curve::Arc {
            start: [5., 0., 0.],
            end: [0., sign * 5., 0.],
            center: [0., 0., 0.],
            plane: Plane::Xy,
            sweep_rad: sign * FRAC_PI_2,
        });
        // The nearest point on the full circle is OUTSIDE this quarter sweep.
        let mut far = body("far", [-1., -1., 1., 1.], [0.5, 2.]);
        far.footprint = PlanarRegion::circle("far", [0., -sign * 5.], 0.1).unwrap();
        let r = check(scene(vec![far]), &m);
        assert_eq!(r.status, Status::Clear);
        assert!(r.stats.nodes < 10);
        let mut hit = body("hit", [-1., -1., 1., 1.], [0.5, 2.]);
        hit.footprint =
            PlanarRegion::circle("hit", [5. / 2_f64.sqrt(), sign * 5. / 2_f64.sqrt()], 0.1)
                .unwrap();
        assert_eq!(check(scene(vec![hit]), &m).status, Status::Violation);
    }
    // No endpoint/radius guard may disappear in the analytic shortcut.
    let mut b = body("guard", [-1., -1., 1., 1.], [0.5, 2.]);
    b.footprint = PlanarRegion::circle("guard", [0., 0.], 5.).unwrap();
    b.footprint.boundary_error_mm = 0.001;
    assert_eq!(check(scene(vec![b]), &m).status, Status::Uncertain);
}

#[test]
fn explicit_target_floor_tolerance_keeps_strict_contact_as_the_default() {
    let mut target = body("floor", [-2., -1., 2., 1.], [0., 3.]);
    target.role = BodyRole::ProtectedTarget;
    let floor = line([-1., 0., 3.], [1., 0., 3.]);
    let strict = scene(vec![target.clone()])
        .prepare(Options::default())
        .unwrap();
    let options = Options {
        target_floor_tolerance_mm: Some(0.01),
        ..Default::default()
    };
    let prepared = scene(vec![target.clone()]).prepare(options).unwrap();
    assert_ne!(strict.digest(), prepared.digest());
    let tool = tool().prepare().unwrap();
    let verify = |m: &Motion| prepared.verify_motion(&tool, m, &|| false).unwrap();
    assert_eq!(
        strict
            .verify_motion(&tool, &floor, &|| false)
            .unwrap()
            .status,
        Status::Uncertain
    );
    let allowed = verify(&floor);
    assert_eq!(allowed.status, Status::Clear);
    assert_eq!(allowed.target_floor_tolerance_mm, Some(0.01));
    assert!(allowed.stats.pruned_intervals > 0);
    let mut rapid = floor.clone();
    rapid.kind = MoveKind::Rapid;
    rapid.feed_mm_min = None;
    assert_eq!(verify(&rapid).status, Status::Uncertain);
    // Interior gouges, including one reached only near the end of a descent,
    // cannot disappear into the allowed contact at the nominal top surface.
    for m in [
        line([-1., 0., 2.98], [1., 0., 2.98]),
        line([-1., 0., 3.1], [1., 0., 2.98]),
        // Both endpoints clear the floor, but this XZ arc dips below it.
        motion(Curve::Arc {
            start: [-1., 0., 3.1],
            end: [1., 0., 3.1],
            center: [0., 0., 3.1],
            plane: Plane::Xz,
            sweep_rad: -PI,
        }),
    ] {
        assert!(m.curve.prepare(1e-5).unwrap().bounds().min[2] < 2.99);
        let report = verify(&m);
        assert_eq!(report.status, Status::Violation, "{:?}", report.findings);
        assert!(report.findings.iter().any(|f| f.witness.is_some()));
    }
    assert_eq!(
        verify(&line([-1., 2., 0.], [1., 2., 0.])).status,
        Status::Uncertain
    );
    target.z_error_mm = 0.02;
    let uncertain = scene(vec![target])
        .prepare(options)
        .unwrap()
        .verify_motion(&tool, &floor, &|| false)
        .unwrap();
    assert_eq!(uncertain.status, Status::Uncertain);
    let legacy = serde_json::from_value::<Options>(serde_json::json!({
        "tolerance_mm": 0.00001, "spatial_resolution_mm": 0.001,
        "max_nodes": 100000, "max_boundary_tests": 10000000, "max_findings": 1024
    }))
    .unwrap();
    assert_eq!(legacy.target_floor_tolerance_mm, None);
}

#[test]
fn target_floor_tolerance_never_relaxes_fixtures_holders_or_work_limits() {
    let options = Options {
        target_floor_tolerance_mm: Some(0.01),
        ..Default::default()
    };
    let m = line([-1., 0., 3.], [1., 0., 3.]);
    let fixture = scene(vec![body("fixture", [-2., -1., 2., 1.], [0., 3.])])
        .prepare(options)
        .unwrap()
        .verify_motion(&tool().prepare().unwrap(), &m, &|| false)
        .unwrap();
    assert_eq!(fixture.status, Status::Uncertain);
    let mut holder_body = body("holder-contact", [2.2, -0.1, 2.4, 0.1], [7., 8.]);
    holder_body.role = BodyRole::ProtectedTarget;
    let report = scene(vec![holder_body])
        .prepare(options)
        .unwrap()
        .verify_motion(
            &tool().prepare().unwrap(),
            &line([0., 0., 0.], [0., 0.01, 0.]),
            &|| false,
        )
        .unwrap();
    assert_eq!(report.status, Status::Uncertain);
    assert!(
        report
            .findings
            .iter()
            .any(|f| f.part == Some(ToolPart::Holder))
    );
    let mut target = body("floor", [-2., -1., 2., 1.], [0., 3.]);
    target.role = BodyRole::ProtectedTarget;
    let exhausted = scene(vec![target.clone()])
        .prepare(Options {
            max_nodes: 1,
            ..options
        })
        .unwrap()
        .verify_motion(&tool().prepare().unwrap(), &m, &|| false)
        .unwrap();
    assert_eq!(exhausted.status, Status::Uncertain);
    assert!(exhausted.stats.budget_exhausted);
    for tolerance in [0., -0.01, f64::NAN, f64::INFINITY, 1.1] {
        assert!(
            scene(vec![target.clone()])
                .prepare(Options {
                    target_floor_tolerance_mm: Some(tolerance),
                    ..Default::default()
                })
                .is_err()
        );
    }
    target.z_max_mm = 0.005;
    assert!(scene(vec![target]).prepare(options).is_err());
}

#[test]
fn nominal_axis_feed_has_independent_line_and_interior_arc_oracles() {
    use camlib::motion::AxisFeedState;
    let frame = WorkFrame::identity();
    let curve = Curve::Line {
        start: [0.; 3],
        end: [3., 4., 0.],
    }
    .prepare(1e-5)
    .unwrap();
    assert_eq!(
        curve.nominal_axis_feed(1000., &frame).unwrap(),
        [600., 800., 0.]
    );
    let mut rotated = frame.clone();
    rotated.origin_mm = [100000., -200000., 300000.];
    rotated.x = [0., 1., 0.];
    rotated.y = [-1., 0., 0.];
    assert_eq!(
        curve.nominal_axis_feed(1000., &rotated).unwrap(),
        [800., 600., 0.]
    );
    let q = 2f64.sqrt();
    let curve = Curve::Arc {
        start: [q, q, 0.],
        end: [-q, q, 0.],
        center: [0.; 3],
        plane: Plane::Xy,
        sweep_rad: FRAC_PI_2,
    }
    .prepare(1e-5)
    .unwrap();
    let peak = curve.nominal_axis_feed(1000., &frame).unwrap();
    assert!((peak[0] - 1000.).abs() < 1e-9);
    assert!((peak[1] - 1000. / q).abs() < 1e-9);
    let reverse = Curve::Arc {
        start: [-q, q, 0.],
        end: [q, q, 0.],
        center: [0.; 3],
        plane: Plane::Xy,
        sweep_rad: -FRAC_PI_2,
    }
    .prepare(1e-5)
    .unwrap()
    .nominal_axis_feed(1000., &frame)
    .unwrap();
    for i in 0..3 {
        assert!((peak[i] - reverse[i]).abs() < 1e-9);
    }
    for sign in [-1., 1.] {
        let t = 0.1f64;
        let wrapped = Curve::Arc {
            start: [t.cos(), -sign * t.sin(), 0.],
            end: [t.cos(), sign * t.sin(), 0.],
            center: [0.; 3],
            plane: Plane::Xy,
            sweep_rad: sign * 0.2,
        }
        .prepare(1e-5)
        .unwrap()
        .nominal_axis_feed(1000., &frame)
        .unwrap();
        assert!((wrapped[0] - 1000. * t.sin()).abs() < 1e-9);
        assert!((wrapped[1] - 1000.).abs() < 1e-9);
    }
    let states = curve
        .nominal_axis_feed_limit(1000., &frame, [900., 900., 900.])
        .unwrap();
    assert_eq!(states[0].1, AxisFeedState::Exceeded);
    assert_eq!(states[1].1, AxisFeedState::Within);
    assert_eq!(
        curve
            .nominal_axis_feed_limit(1000., &frame, [1000.; 3])
            .unwrap()[0]
            .1,
        AxisFeedState::Boundary
    );
    // A short monotone interval must NOT receive full-circle tangent maxima.
    let angle = 0.1f64;
    let final_angle = 0.2f64;
    let short = Curve::Arc {
        start: [angle.cos(), angle.sin(), 0.],
        end: [final_angle.cos(), final_angle.sin(), 0.],
        center: [0.; 3],
        plane: Plane::Xy,
        sweep_rad: 0.1,
    }
    .prepare(1e-5)
    .unwrap();
    let peak = short.nominal_axis_feed(1000., &frame).unwrap();
    assert!((peak[0] - 1000. * final_angle.sin()).abs() < 1e-9);
    assert!((peak[1] - 1000. * angle.cos()).abs() < 1e-9);
    assert!(
        Curve::Line {
            start: [0.; 3],
            end: [0.; 3]
        }
        .prepare(1e-5)
        .is_err()
    );
}

#[test]
fn nominal_axis_feed_covers_all_planes_senses_pitch_and_tilted_interior_extrema() {
    for plane in [Plane::Xy, Plane::Xz, Plane::Yz] {
        let [u, v, w] = plane.axes();
        for sign in [-1., 1.] {
            for pitch in [-2., 2.] {
                let sweep = sign * 7. * TAU;
                let mut start = [0.; 3];
                start[u] = 2.;
                let mut end = start;
                end[w] = pitch * sweep;
                let path = Curve::Helix {
                    start,
                    end,
                    center: [0.; 3],
                    plane,
                    sweep_rad: sweep,
                }
                .prepare(1e-5)
                .unwrap();
                let peak = path
                    .nominal_axis_feed(1000., &WorkFrame::identity())
                    .unwrap();
                for i in [u, v, w] {
                    assert!((peak[i] - 1000. / 2f64.sqrt()).abs() < 1e-9);
                }
            }
        }
    }
    // Rotation mixes constant pitch with a sinusoid. On a full turn, the
    // absolute peak is |pitch contribution| + sinusoidal amplitude.
    let (sin, cos) = (PI / 6.).sin_cos();
    let frame = WorkFrame {
        origin_mm: [10., 20., 30.],
        x: [1., 0., 0.],
        y: [0., cos, sin],
        z: [0., -sin, cos],
    };
    for sense in [-1., 1.] {
        for pitch in [-2., 2.] {
            let sweep = sense * 3. * TAU;
            let path = Curve::Helix {
                start: [2., 0., 0.],
                end: [2., 0., pitch * sweep],
                center: [0.; 3],
                plane: Plane::Xy,
                sweep_rad: sweep,
            }
            .prepare(1e-5)
            .unwrap();
            let peak = path.nominal_axis_feed(1000., &frame).unwrap();
            let radial = 1000. / 2f64.sqrt();
            for (actual, expected) in
                peak.into_iter()
                    .zip([radial, radial * (cos + sin), radial * (sin + cos)])
            {
                assert!((actual - expected).abs() < 1e-9, "{actual} != {expected}");
            }
        }
    }
}

#[test]
fn axis_feed_verification_binds_limits_coverage_caps_cancel_and_identity() {
    let mut s = scene(vec![]);
    s.machine = Some(MachineLimits {
        min_tip_mm: Some([-100.; 3]),
        max_tip_mm: Some([100.; 3]),
        max_feed_mm_min: 2000.,
        max_normal_accel_mm_s2: None,
        max_axis_feed_mm_min: None,
        min_gauge_mm: None,
        max_gauge_mm: None,
    });
    let before = s.prepare(Options::default()).unwrap().digest().to_owned();
    s.machine.as_mut().unwrap().max_axis_feed_mm_min = Some([700., 700., 700.]);
    let prepared = s.prepare(Options::default()).unwrap();
    assert_ne!(before, prepared.digest());
    let m = Motion {
        feed_mm_min: Some(1000.),
        ..line([0.; 3], [3., 4., 0.])
    };
    let r = check(s.clone(), &m);
    assert_eq!(r.status, Status::Violation);
    assert!(r.coverage.nominal_axis_feed_checked);
    assert_eq!(r.findings.len(), 1);
    assert_eq!(r.findings[0].kind, FindingKind::AxisFeedLimit);
    assert!(r.findings[0].detail.contains("axis Y"));
    assert!(r.findings[0].witness.is_none());
    let rapid = Motion {
        kind: MoveKind::Rapid,
        feed_mm_min: None,
        ..m.clone()
    };
    let r = check(s.clone(), &rapid);
    assert_eq!(r.status, Status::Clear);
    assert!(!r.coverage.nominal_axis_feed_checked);
    assert!(
        prepared
            .verify_motion(&tool().prepare().unwrap(), &m, &|| true)
            .is_err()
    );
    s.machine.as_mut().unwrap().max_axis_feed_mm_min = Some([600., 800., 900.]);
    assert_eq!(check(s.clone(), &m).status, Status::Uncertain);
    s.machine.as_mut().unwrap().max_axis_feed_mm_min = Some([500.; 3]);
    let capped = s
        .prepare(Options {
            max_findings: 1,
            ..Default::default()
        })
        .unwrap()
        .verify_motion(&tool().prepare().unwrap(), &m, &|| false)
        .unwrap();
    assert_eq!(capped.findings.len(), 1);
    assert!(capped.stats.findings_truncated);
    for invalid in [0., -1., f64::NAN, f64::INFINITY, 1e8] {
        s.machine.as_mut().unwrap().max_axis_feed_mm_min = Some([1000., invalid, 1000.]);
        assert!(s.prepare(Options::default()).is_err());
    }
}

/// `tool()` under another id and gauge length (its holder reaches the gauge line).
fn tool_of(id: &str, gauge_length_mm: f64) -> ToolAssembly {
    let mut t = tool();
    t.id = id.into();
    t.gauge_length_mm = gauge_length_mm;
    t.holders[0].z_max_mm = gauge_length_mm;
    t
}
fn check_with(s: &Scene, t: &ToolAssembly, mut m: Motion) -> Report {
    m.tool = t.id.clone();
    s.clone()
        .prepare(Options::default())
        .unwrap()
        .verify_motion(&t.prepare().unwrap(), &m, &AtomicBool::new(false))
        .unwrap()
}
fn gauge_limits(min: [f64; 3], max: [f64; 3]) -> MachineLimits {
    MachineLimits {
        min_tip_mm: None,
        max_tip_mm: None,
        max_feed_mm_min: 10000.,
        max_normal_accel_mm_s2: None,
        max_axis_feed_mm_min: None,
        min_gauge_mm: Some(min),
        max_gauge_mm: Some(max),
    }
}

#[test]
fn gauge_line_travel_gives_each_tool_its_own_tip_travel() {
    // #176. The gauge line travels X 0..500, Y 0..400, Z -400..0 (machine).
    // A 60 mm tool's tip reaches Z -460..-60, a 115 mm tool's Z -515..-115.
    let mut s = scene(vec![]);
    s.machine = Some(gauge_limits([0., 0., -400.], [500., 400., 0.]));
    s.work_to_machine.origin_mm = [200., 150., -300.];
    let (short, long) = (tool_of("short", 60.), tool_of("long", 115.));
    // Work Z 240 is machine Z -60: the short tool's Z home, 55 mm above the
    // long tool's.
    let up = || line([0., 0., 150.], [0., 0., 240.]);
    let r = check_with(&s, &short, up());
    assert_eq!(r.status, Status::Clear, "{:?}", r.findings);
    assert!(r.coverage.machine_tip_travel_examined);
    let r = check_with(&s, &long, up());
    assert_eq!(r.status, Status::Violation);
    assert_eq!(r.tool, "long");
    let f = r
        .findings
        .iter()
        .find(|f| f.kind == FindingKind::MachineTravel)
        .unwrap();
    assert!(
        f.detail.contains("Tool long") && f.detail.contains("machine Z"),
        "{}",
        f.detail
    );
    let w = f.witness.as_ref().unwrap();
    assert_eq!(w.fraction, 1.);
    assert!((w.margin_lower_mm - 55.).abs() < 1e-3, "{w:?}");
    // The bottom is the other way round: the long tool reaches 55 mm lower.
    let down = || line([0., 0., -150.], [0., 0., -215.]);
    assert_eq!(check_with(&s, &long, down()).status, Status::Clear);
    let r = check_with(&s, &short, down());
    assert_eq!(r.status, Status::Violation);
    let f = &r.findings[0];
    assert!(
        f.detail.contains("Tool short") && f.detail.contains("machine Z"),
        "{}",
        f.detail
    );
    // X and Y do not depend on the tool, and the finding names the axis.
    for t in [&short, &long] {
        let r = check_with(&s, t, line([0., 0., 0.], [0., 251., 0.]));
        assert_eq!(r.status, Status::Violation);
        assert!(r.findings[0].detail.contains("machine Y"));
        assert_eq!(
            check_with(&s, t, line([0., 0., 0.], [300., 250., 0.])).status,
            Status::Clear
        );
    }
    // Arcs: decided from their bounds, or by subdivision where they cross.
    let arc = |top: f64| {
        motion(Curve::Arc {
            start: [0., 0., top],
            end: [-10., 0., top - 10.],
            center: [0., 0., top - 10.],
            plane: Plane::Xz,
            sweep_rad: -FRAC_PI_2,
        })
    };
    assert_eq!(check_with(&s, &short, arc(240.)).status, Status::Clear);
    assert_eq!(check_with(&s, &long, arc(240.)).status, Status::Violation);
    assert_eq!(check_with(&s, &long, arc(185.)).status, Status::Clear);
    // An arc that only crosses the long tool's top in its middle.
    let over = motion(Curve::Arc {
        start: [0., 0., 181.],
        end: [10., 0., 181.],
        center: [5., 0., 181.],
        plane: Plane::Xz,
        sweep_rad: PI,
    });
    assert_eq!(check_with(&s, &short, over.clone()).status, Status::Clear);
    assert_eq!(check_with(&s, &long, over).status, Status::Violation);
}

#[test]
fn gauge_line_travel_is_closed_a_tool_at_home_is_inside() {
    // The gauge line at Z home (machine Z 0) and at X0/Y0: on its limits.
    // Lengths and offsets that binary64 cannot represent exactly.
    for (length, origin_z) in [(115., -300.), (123.4, -300.1), (60.7, -87.3)] {
        let mut s = scene(vec![]);
        s.machine = Some(gauge_limits([0., 0., -400.], [500., 400., 0.]));
        s.work_to_machine.origin_mm = [200.3, 150.7, origin_z];
        let t = tool_of("t", length);
        let home = [-200.3, -150.7, 0. - length - origin_z];
        for m in [
            line(home, [home[0], home[1], home[2] - 30.]),
            line([home[0] + 50., home[1] + 50., home[2] - 30.], home),
            line(home, [home[0] + 100., home[1], home[2]]),
        ] {
            let r = check_with(&s, &t, m);
            assert_eq!(r.status, Status::Clear, "{length}: {:?}", r.findings);
        }
        // One output unit above home is outside, at the endpoint.
        let r = check_with(
            &s,
            &t,
            line([home[0], home[1], home[2] - 30.], [home[0], home[1], home[2] + 0.001]),
        );
        assert_eq!(r.status, Status::Violation, "{length}");
        assert_eq!(r.findings[0].witness.as_ref().unwrap().fraction, 1.);
    }
}

#[test]
fn one_tool_gets_the_same_travel_from_its_tip_box_or_the_gauge_line() {
    // tool(): gauge length 20. The tip box is the gauge box 20 mm lower.
    let mut tip = scene(vec![]);
    tip.machine = Some(MachineLimits {
        min_tip_mm: Some([-6., -1., -50.]),
        max_tip_mm: Some([6., 4., 10.]),
        min_gauge_mm: None,
        max_gauge_mm: None,
        ..gauge_limits([0.; 3], [0.; 3])
    });
    let mut gauge = scene(vec![]);
    gauge.machine = Some(gauge_limits([-6., -1., -30.], [6., 4., 30.]));
    let arc = |r: f64| {
        motion(Curve::Arc {
            start: [r, 0., 3.],
            end: [-r, 0., 3.],
            center: [0., 0., 3.],
            plane: Plane::Xy,
            sweep_rad: PI,
        })
    };
    for m in [
        line([0., 0., 0.], [6., 4., 10.]),
        line([0., 0., 0.], [6., 4., 10.5]),
        line([0., 0., -50.], [-6., -1., -50.]),
        line([0., 0., -50.], [-7., -1., -50.]),
        arc(3.),
        arc(4.),
        arc(5.),
    ] {
        let (a, b) = (check(tip.clone(), &m), check(gauge.clone(), &m));
        assert_eq!(a.status, b.status, "{:?}", m.curve);
        assert_eq!(a.findings.len(), b.findings.len());
        for (x, y) in a.findings.iter().zip(&b.findings) {
            assert_eq!(x.kind, y.kind);
            assert_eq!(x.fraction_interval, y.fraction_interval);
            assert_eq!(
                x.witness.as_ref().map(|w| (w.fraction, w.tip_mm)),
                y.witness.as_ref().map(|w| (w.fraction, w.tip_mm))
            );
        }
    }
    // The tip box's finding text is what it was before gauge-line travel.
    let r = check(tip, &line([0., 0., 0.], [6., 4., 10.5]));
    assert_eq!(
        r.findings[0].detail,
        "Tip exceeds configured travel after rigid work-to-machine transform"
    );
}

#[test]
fn machine_travel_is_declared_once_and_gauge_line_travel_refuses_what_it_cannot_model() {
    let prepare = |s: &Scene| s.prepare(Options::default()).map(|_| ());
    let mut s = scene(vec![]);
    // Both boxes, neither, or half of one.
    let mut both = gauge_limits([-10.; 3], [10.; 3]);
    both.min_tip_mm = Some([-10.; 3]);
    both.max_tip_mm = Some([10.; 3]);
    let mut neither = both.clone();
    (neither.min_tip_mm, neither.max_tip_mm) = (None, None);
    (neither.min_gauge_mm, neither.max_gauge_mm) = (None, None);
    let mut half = gauge_limits([-10.; 3], [10.; 3]);
    half.max_gauge_mm = None;
    for m in [both, neither, half] {
        s.machine = Some(m);
        let e = prepare(&s).unwrap_err();
        assert!(e.contains("min_gauge_mm") && e.contains("min_tip_mm"), "{e}");
    }
    // An empty gauge box.
    s.machine = Some(gauge_limits([0.; 3], [10., 10., 0.]));
    assert!(prepare(&s).unwrap_err().contains("invalid machine tip travel"));
    // A work frame whose Z is not machine Z: the tool axis is not modelled.
    s.machine = Some(gauge_limits([-10.; 3], [10.; 3]));
    s.work_to_machine.y = [0., 0., 1.];
    s.work_to_machine.z = [0., -1., 0.];
    assert!(prepare(&s).unwrap_err().contains("work Z axis along machine Z"));
    // Rotated about machine Z is an upright spindle still.
    s.work_to_machine.x = [0., 1., 0.];
    s.work_to_machine.y = [-1., 0., 0.];
    s.work_to_machine.z = [0., 0., 1.];
    prepare(&s).unwrap();
    assert_eq!(
        check(s.clone(), &line([-1., 0., -10.], [1., 0., -30.])).status,
        Status::Clear
    );
    assert_eq!(
        check(s.clone(), &line([-1., 0., -10.], [1., 0., -30.5])).status,
        Status::Violation
    );
    // A beam has no gauge length to derive its travel from.
    let mut beam = tool();
    beam.cutter.shape = CutterShape::Beam;
    beam.cutter.neck = None;
    let e = s
        .prepare(Options::default())
        .unwrap()
        .verify_motion(
            &beam.prepare().unwrap(),
            &line([0.; 3], [1., 0., 0.]),
            &AtomicBool::new(false),
        )
        .unwrap_err();
    assert!(e.contains("beam tool t1") && e.contains("min_tip_mm"), "{e}");
    // An existing scene (a tip box, no gauge fields) reads and writes as before.
    let json = r#"{"min_tip_mm":[0.0,0.0,-515.0],"max_tip_mm":[500.0,400.0,-115.0],"max_feed_mm_min":10000.0}"#;
    let m: MachineLimits = serde_json::from_str(json).unwrap();
    assert_eq!(serde_json::to_string(&m).unwrap(), json);
}
