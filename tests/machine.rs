//! Synthetic physical machine declarations; no production machine defaults.
use camlib::{machine::*, motion::*, stock::*, stream::*, tool::*, verification::Status};
use serde_json::json;
use std::sync::atomic::AtomicBool;
fn manifest() -> StreamManifest {
    StreamManifest {
        version: 1,
        run: RunIdentity {
            id: "machine-test".into(),
            revision: "1".into(),
        },
        stock: StockSpec {
            version: 1,
            units: "mm".into(),
            min_mm: [100., 100., 0.],
            max_mm: [110., 110., 10.],
            cell_size_mm: 1.,
            max_cells: 1000,
        }
        .into(),
        tools: vec![ToolAssembly {
            id: "t1".into(),
            cutter: Cutter {
                id: "c1".into(),
                name: "synthetic".into(),
                shape: CutterShape::Flat,
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
        }],
        initial_position_mm: [0., 0., 35.],
        sweep: SweepOptions::default(),
        operations: vec![],
        verification: None,
        tool_change: None,
        rapid_retracts: false,
        turning: None,
    }
}
fn box_(lo: Vec3, hi: Vec3) -> Solid {
    Solid::Box {
        min_mm: lo,
        max_mm: hi,
    }
}
fn model() -> Model {
    Model {
        version: 1,
        units: "mm".into(),
        id: "synthetic-mill".into(),
        revision: "1".into(),
        process: Process::Milling,
        axes: std::array::from_fn(|_| Axis {
            drive: Drive::Head,
            min_mm: -200.,
            max_mm: 200.,
        }),
        work_offset: 1,
        work_to_machine: WorkFrame::identity(),
        work_error_min_mm: [0.; 3],
        work_error_max_mm: [0.; 3],
        initial_axes_mm: [0., 0., 60.],
        initial_tool: Some("t1".into()),
        rapid_mode: RapidMode::Linear,
        controller_reference: "synthetic controller exact-stop linear G0".into(),
        tools: vec![ToolCalibration {
            tool: "t1".into(),
            length_register: 1,
            body_solids: vec![],
            gauge_offset_min_mm: [0., 0., 25.],
            gauge_offset_max_mm: [0., 0., 25.],
        }],
        bodies: vec![Body {
            id: "head".into(),
            component: "spindle_head".into(),
            mount: Mount::Head,
            solids: vec![box_([-1., -1., 0.], [1., 1., 10.])],
            indexed_solids: Default::default(),
        }],
        absent_components: [
            ("column".into(), "synthetic stand".into()),
            (
                "table".into(),
                "remote support excluded by declaration".into(),
            ),
            ("enclosure".into(), "open stand".into()),
        ]
        .into(),
        allowed_contacts: vec![AllowedContact {
            bodies: ["head".into(), "@tool".into()],
            reason: "mounted at spindle gauge".into(),
        }],
        tool_change_routes: [(
            "t1".into(),
            ToolRoute {
                reference: "synthetic complete indexing envelope".into(),
                before_index_mm: vec![],
                index_envelopes: vec![box_([-4., -4., -26.], [4., 4., 1.])],
                replaces: vec!["@tool".into()],
                after_index_mm: vec![],
            },
        )]
        .into(),
        program_end_route_mm: vec![],
        options: Options {
            tolerance_mm: 1e-5,
            spatial_resolution_mm: 0.05,
            max_axis_tests: 1_000_000,
            max_findings: 128,
            stock_contact: camlib::stock_contact::Options::default(),
        },
    }
}
fn definition(m: &Model) -> serde_json::Value {
    json!({"machine":{"frame":WorkFrame::identity(),"work_to_machine":m.work_to_machine,"work_offset":m.work_offset,"tools":[{"tool":"t1","length_register":1}]}})
}
fn session(m: Model) -> Session {
    Session::new(
        m.clone(),
        manifest(),
        &definition(&m),
        &AtomicBool::new(false),
    )
    .unwrap()
}
fn motion(end: Vec3, rapid: bool) -> Event {
    Event::Motion {
        motion: Motion {
            id: 7,
            operation: "move".into(),
            tool: "t1".into(),
            kind: if rapid {
                MoveKind::Rapid
            } else {
                MoveKind::Cut
            },
            curve: Curve::Line {
                start: [0., 0., 35.],
                end,
            },
            feed_mm_min: if rapid { None } else { Some(100.) },
        },
    }
}
fn fixture(m: &mut Model, lo: Vec3, hi: Vec3) {
    m.bodies.push(Body {
        id: "clamp".into(),
        component: "fixture".into(),
        mount: Mount::Fixed,
        solids: vec![box_(lo, hi)],
        indexed_solids: Default::default(),
    });
}
#[test]
fn clear_full_decoded_motion_and_named_spindle_crash() {
    let mut s = session(model());
    s.event(motion([10., 0., 35.], true), 3, &|| false).unwrap();
    let r = s.finish(1, &|| false).unwrap();
    assert!(r.complete);
    assert_eq!(r.status, Status::Clear);
    assert_eq!(r.decoded_motions, 1);
    let mut m = model();
    fixture(&mut m, [4., -2., 61.], [6., 2., 69.]);
    let mut s = session(m);
    s.event(motion([10., 0., 35.], true), 3, &|| false).unwrap();
    let r = s.finish(1, &|| false).unwrap();
    let hit = r
        .findings
        .iter()
        .find(|f| f.bodies.contains(&"head".into()) && f.bodies.contains(&"clamp".into()))
        .unwrap();
    assert_eq!(hit.status, Status::Violation);
    assert_eq!(hit.motion_id, Some(7));
    assert!(hit.fraction > 0. && hit.fraction < 1.);
}
#[test]
fn independent_axes_enclose_dogleg_interior_and_feed_traverses_stay_linear() {
    let mut m = model();
    fixture(&mut m, [9., -0.5, 35.], [11., 0.5, 40.]);
    let mut s = session(m.clone());
    s.event(motion([20., 20., 35.], true), 0, &|| false)
        .unwrap();
    assert_eq!(s.finish(1, &|| false).unwrap().status, Status::Clear);
    m.rapid_mode = RapidMode::AxisIndependent;
    let mut s = session(m.clone());
    s.event(motion([20., 20., 35.], true), 0, &|| false)
        .unwrap();
    let r = s.finish(1, &|| false).unwrap();
    assert_eq!(r.status, Status::Uncertain);
    assert!(
        r.findings
            .iter()
            .any(|f| f.bodies.contains(&"clamp".to_string()))
    );
    let mut s = session(m);
    s.event(motion([20., 20., 35.], false), 0, &|| false)
        .unwrap();
    assert_eq!(s.finish(1, &|| false).unwrap().status, Status::Clear);
}
#[test]
fn machine_travel_uses_offset_and_gauge_and_checks_curve_interiors() {
    let mut m = model();
    m.axes[2].max_mm = 65.;
    let mut s = session(m);
    s.event(motion([10., 0., 45.], true), 1, &|| false).unwrap();
    let r = s.finish(1, &|| false).unwrap();
    assert!(
        r.findings
            .iter()
            .any(|f| f.kind == "machine_travel" && f.status == Status::Violation)
    );
    let mut m = model();
    m.axes[1].max_mm = 3.;
    let mut s = session(m);
    s.event(
        Event::Motion {
            motion: Motion {
                id: 8,
                operation: "arc".into(),
                tool: "t1".into(),
                kind: MoveKind::Cut,
                curve: Curve::Arc {
                    start: [0., 0., 35.],
                    end: [10., 0., 35.],
                    center: [5., 0., 35.],
                    plane: Plane::Xy,
                    sweep_rad: -std::f64::consts::PI,
                },
                feed_mm_min: Some(100.),
            },
        },
        1,
        &|| false,
    )
    .unwrap();
    assert!(
        s.finish(1, &|| false)
            .unwrap()
            .findings
            .iter()
            .any(|f| f.kind == "machine_travel")
    );
}
#[test]
fn tool_change_routes_and_complete_index_enclosures_are_checked() {
    let mut m = model();
    fixture(&mut m, [4., -2., 61.], [6., 2., 69.]);
    let r = m.tool_change_routes.get_mut("t1").unwrap();
    r.before_index_mm = vec![[Some(10.), None, None]];
    r.after_index_mm = vec![[Some(0.), None, None]];
    let mut s = session(m.clone());
    s.event(
        Event::ToolChange {
            tool: "t1".into(),
            number: 1,
            length_register: 1,
            position_after_mm: [0., 0., 35.],
        },
        2,
        &|| false,
    )
    .unwrap();
    let r = s.finish(0, &|| false).unwrap();
    assert_eq!(r.tool_changes, 1);
    assert!(r.findings.iter().any(|f| f.status == Status::Violation
        && f.route.as_deref().unwrap_or("").contains("before_index")));
    m.tool_change_routes.get_mut("t1").unwrap().index_envelopes =
        vec![box_([-1., -1., -1.], [1., 1., 1.])];
    let mut s = session(m);
    assert!(
        s.event(
            Event::ToolChange {
                tool: "t1".into(),
                number: 1,
                length_register: 1,
                position_after_mm: [0., 0., 35.]
            },
            2,
            &|| false
        )
        .unwrap_err()
        .contains("does not enclose")
    );
    assert!(s.finish(0, &|| false).is_err());
}
#[test]
fn stock_and_end_routes_and_initial_pose_are_checked() {
    let m = model();
    let mut stock = manifest();
    stock.stock = StockSpec {
        version: 1,
        units: "mm".into(),
        min_mm: [4., -2., 61.],
        max_mm: [6., 2., 69.],
        cell_size_mm: 1.,
        max_cells: 1000,
    }
    .into();
    let mut s = Session::new(m.clone(), stock, &definition(&m), &|| false).unwrap();
    s.event(motion([10., 0., 35.], true), 0, &|| false).unwrap();
    let r = s.finish(1, &|| false).unwrap();
    assert!(
        r.findings
            .iter()
            .any(|f| f.kind == "machine_stock_contact" && f.status == Status::Violation)
    );
    let mut m = model();
    fixture(&mut m, [4., -2., 61.], [6., 2., 69.]);
    m.program_end_route_mm = vec![[Some(10.), None, None]];
    let mut s = session(m);
    assert!(
        s.finish(0, &|| false)
            .unwrap()
            .findings
            .iter()
            .any(|f| f.route.as_deref() == Some("program_end"))
    );
}
#[test]
fn malformed_models_missing_registers_and_incomplete_checks_refuse() {
    let mut m = model();
    m.absent_components.remove("enclosure");
    assert!(m.prepare(&|| false).is_err());
    let mut s = session(model());
    assert!(s.finish(1, &|| false).is_err());
    assert!(s.finish(0, &|| false).is_err());
    let mut s = session(model());
    assert!(
        s.event(Event::WorkOffset { number: 2 }, 1, &|| false)
            .is_err()
    );
    let mut m = model();
    m.tools[0].length_register = 2;
    assert!(Session::new(m.clone(), manifest(), &definition(&m), &|| false).is_err());
    assert!(model().prepare(&|| true).is_err());
}

#[test]
fn table_axis_moves_workpiece_fixture_past_stationary_head() {
    let mut m = model();
    m.axes[0].drive = Drive::Table;
    fixture(&mut m, [4., -2., 61.], [6., 2., 69.]);
    let mut s = session(m.clone());
    s.event(motion([10., 0., 35.], true), 1, &|| false).unwrap();
    assert_eq!(s.finish(1, &|| false).unwrap().status, Status::Clear);
    m.bodies.iter_mut().find(|b| b.id == "clamp").unwrap().mount = Mount::Workpiece;
    let mut s = session(m);
    s.event(motion([10., 0., 35.], true), 1, &|| false).unwrap();
    assert_eq!(s.finish(1, &|| false).unwrap().status, Status::Violation);
}
#[test]
fn incomplete_replay_preserves_prior_definite_collision() {
    let mut m = model();
    fixture(&mut m, [4., -2., 61.], [6., 2., 69.]);
    let mut s = session(m);
    s.event(motion([10., 0., 35.], true), 1, &|| false).unwrap();
    assert!(
        s.event(Event::WorkOffset { number: 2 }, 2, &|| false)
            .is_err()
    );
    let r = s.incomplete("wrong offset after collision".into());
    assert!(!r.complete);
    assert_eq!(r.status, Status::Violation);
    assert!(r.findings.iter().any(|f| f.status == Status::Violation));
    assert!(r.findings.iter().any(|f| f.status == Status::Uncertain));
}
#[test]
fn work_offset_and_calibration_bounds_affect_machine_clearance() {
    let mut m = model();
    m.work_to_machine.origin_mm = [100., 0., 0.];
    m.initial_axes_mm[0] = 100.;
    fixture(&mut m, [104., -2., 61.], [106., 2., 69.]);
    let mut s = session(m);
    s.event(motion([10., 0., 35.], true), 1, &|| false).unwrap();
    assert_eq!(s.finish(1, &|| false).unwrap().status, Status::Violation);
    let mut m = model();
    m.axes[0].max_mm = 10.5;
    m.work_error_min_mm = [-1., 0., 0.];
    m.work_error_max_mm = [1., 0., 0.];
    let mut s = session(m);
    s.event(motion([10., 0., 35.], true), 1, &|| false).unwrap();
    assert_eq!(s.finish(1, &|| false).unwrap().status, Status::Uncertain);
}

#[test]
fn convex_mesh_fixture_and_arc_helix_interiors_are_checked() {
    for helical in [false, true] {
        let mut m = model();
        fixture(&mut m, [4., 4., 61.], [6., 6., 69.]);
        m.bodies.last_mut().unwrap().solids = vec![Solid::ConvexMesh {
            vertices_mm: vec![[4., 4., 61.], [6., 4., 61.], [5., 6., 61.], [5., 5., 69.]],
            triangles: vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]],
        }];
        let mut s = session(m);
        let curve = if helical {
            Curve::Helix {
                start: [0., 0., 35.],
                end: [10., 0., 36.],
                center: [5., 0., 35.],
                plane: Plane::Xy,
                sweep_rad: -std::f64::consts::PI,
            }
        } else {
            Curve::Arc {
                start: [0., 0., 35.],
                end: [10., 0., 35.],
                center: [5., 0., 35.],
                plane: Plane::Xy,
                sweep_rad: -std::f64::consts::PI,
            }
        };
        s.event(
            Event::Motion {
                motion: Motion {
                    id: 8,
                    operation: "curve".into(),
                    tool: "t1".into(),
                    kind: MoveKind::Cut,
                    curve,
                    feed_mm_min: Some(100.),
                },
            },
            1,
            &|| false,
        )
        .unwrap();
        let r = s.finish(1, &|| false).unwrap();
        assert_eq!(r.status, Status::Violation, "{r:?}");
        assert!(
            r.findings
                .iter()
                .any(|f| f.motion_id == Some(8) && f.bodies.contains(&"clamp".into()))
        );
    }
}
