//! Synthetic physical machine declarations; no production machine defaults.
use camlib::{machine::*, motion::*, stock::*, stream::*, tool::*, verification::Status};
use serde_json::json;
use std::sync::atomic::AtomicBool;
fn manifest() -> StreamManifest {
    StreamManifest {
        initial_stock: None,
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
fn travel_finding(s: &mut Session, motions: u64) -> Option<Finding> {
    s.finish(motions, &|| false)
        .unwrap()
        .findings
        .iter()
        .find(|f| f.kind == "machine_travel")
        .cloned()
}
#[test]
fn axis_travel_follows_the_one_travel_rule() {
    // #246. The tool (gauge 25) starts at axis Z 60 and feeds to tip Z 40,
    // axis Z 65. The travel is closed, a decoded position is exact, and the
    // check's tolerance (1e-5 mm) decides nothing.
    let run = |m: Model, tip_z: f64| {
        let mut s = session(m);
        s.event(motion([10., 0., tip_z], false), 1, &|| false)
            .unwrap();
        travel_finding(&mut s, 1)
    };
    let mut m = model();
    m.axes[2].max_mm = 65.;
    assert!(run(m.clone(), 40.).is_none());
    let found = run(m.clone(), 40. + 1e-6).unwrap();
    assert_eq!(found.status, Status::Violation);
    assert_eq!(found.bodies[0], "axis_z");
    assert_eq!((found.motion_id, found.fraction), (Some(7), 1.));
    // The start pose on its limit is inside too (a machine at home).
    let mut home = model();
    home.axes[2].max_mm = 60.;
    assert!(run(home, 30.).is_none());
    // A gauge offset known to +-0.01 mm: the axis position is an interval.
    // Inside only if the whole interval is inside, a violation only if the
    // whole interval is beyond the limit, uncertain while it straddles it;
    // the finding states the interval and the limit.
    m.tools[0].gauge_offset_min_mm = [0., 0., 24.99];
    m.tools[0].gauge_offset_max_mm = [0., 0., 25.01];
    let found = run(m.clone(), 40.).unwrap();
    assert_eq!(found.status, Status::Uncertain);
    assert!(
        found.detail.contains("interval 64.990000 to 65.010000 mm")
            && found.detail.contains("straddles the maximum 65.000000 mm"),
        "{}",
        found.detail
    );
    assert!(run(m.clone(), 39.99).is_none());
    assert_eq!(
        run(m.clone(), 39.99 + 1e-6).unwrap().status,
        Status::Uncertain
    );
    assert_eq!(run(m.clone(), 40.01).unwrap().status, Status::Uncertain);
    let found = run(m.clone(), 40.01 + 1e-6).unwrap();
    assert_eq!(found.status, Status::Violation);
    assert!(
        found.detail.contains("whole interval is beyond the maximum"),
        "{}",
        found.detail
    );
    // A declared route waypoint is a machine-axis position: no calibration
    // interval, compared exactly. On the limit is inside, 1e-6 mm beyond is
    // a violation.
    for (x, expected) in [(200., None), (200. + 1e-6, Some(Status::Violation))] {
        let mut m = model();
        m.program_end_route_mm = vec![[Some(x), None, None]];
        let mut s = session(m);
        let found = travel_finding(&mut s, 0);
        assert_eq!(found.as_ref().map(|f| f.status), expected, "{x}");
        if let Some(found) = found {
            assert_eq!(found.route.as_deref(), Some("program_end"));
            assert_eq!(found.bodies[0], "axis_x");
        }
    }
    // The start position is declared the same way: on a limit it is inside,
    // beyond it the model is refused.
    let mut m = model();
    m.axes[2].max_mm = 60.;
    m.prepare(&|| false).map(|_| ()).unwrap();
    m.axes[2].max_mm = 60. - 1e-9;
    assert!(
        m.prepare(&|| false)
            .map(|_| ())
            .unwrap_err()
            .contains("initial position inside")
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

/// A mill at home on its X and Y minimums and its Z maximum (axis X 0 of
/// [0, 508], as real mills home), the work frame known to +-0.02 mm on every
/// axis. The tool (gauge 25) starts at tip [0, 0, 35] in work coordinates.
fn homed(frame: WorkFrame) -> Model {
    let mut m = model();
    m.axes[0].min_mm = 0.;
    m.axes[0].max_mm = 508.;
    m.axes[1].min_mm = 0.;
    m.axes[1].max_mm = 400.;
    m.axes[2].max_mm = 60.;
    m.work_to_machine = frame;
    m.work_error_min_mm = [-0.02; 3];
    m.work_error_max_mm = [0.02; 3];
    m.initial_axes_mm = [0., 0., 60.];
    m
}

/// A posted program for the session: `header` blocks belong to no decoded
/// record; each block of `blocks` is one record, with its decoded move (or
/// none: a control block, or words that decode to no move).
type Block<'a> = (&'a str, Option<(Vec3, Vec3)>);

fn posted(header: &[&str], blocks: &[Block]) -> (Vec<camlib::post::Line>, Vec<camlib::post::Checked>) {
    let mut lines = vec![];
    let mut checked = vec![];
    for text in header {
        lines.push(camlib::post::Line {
            number: lines.len() + 1,
            event_index: None,
            operation: None,
            motion_id: None,
            text: text.to_string(),
        });
    }
    for (index, (text, curve)) in blocks.iter().enumerate() {
        let number = lines.len() + 1;
        let id = index as u64 + 1;
        lines.push(camlib::post::Line {
            number,
            event_index: Some(index),
            operation: Some("op".into()),
            motion_id: curve.map(|_| id),
            text: text.to_string(),
        });
        checked.push(camlib::post::Checked {
            event_index: index,
            line_interval: [number, number],
            motion: curve.map(|(start, end)| Motion {
                id,
                operation: "op".into(),
                tool: "t1".into(),
                kind: MoveKind::Rapid,
                curve: Curve::Line { start, end },
                feed_mm_min: None,
            }),
            deviation_bound_mm: None,
            control_effects: Some(vec![]),
            cycle_event_index: None,
        });
    }
    (lines, checked)
}

fn dialect_definition(m: &Model) -> serde_json::Value {
    let mut d = definition(m);
    d["machine"]["dialect"] = json!({"axes": ["X", "Y", "Z"], "parentheses_comments": true});
    d
}

/// The machine-travel findings of `blocks` posted to the homed mill.
fn exact_axes_travel(m: Model, header: &[&str], blocks: &[Block]) -> Vec<Finding> {
    let (lines, checked) = posted(header, blocks);
    let mut s = Session::new(m.clone(), manifest(), &dialect_definition(&m), &|| false).unwrap();
    s.chunk(&lines, &checked, &|| false).unwrap();
    let motions = checked.iter().filter(|c| c.motion.is_some()).count() as u64;
    s.finish(motions, &|| false)
        .unwrap()
        .findings
        .iter()
        .filter(|f| f.kind == "machine_travel")
        .cloned()
        .collect()
}

const HOME_TIP: Vec3 = [0., 0., 35.];

#[test]
fn axes_at_home_are_exact_until_a_block_commands_them() {
    // The first moves from home: Z down, then the first XY rapid. Each
    // starts where the machine is, at home on the X and Y minimums.
    let first = [
        ("G0 Z10", Some((HOME_TIP, [0., 0., 10.]))),
        ("G0 X50 Y40", Some(([0., 0., 10.], [50., 40., 10.]))),
    ];
    let header = ["%", "(T1 FLAT 6MM - X Y Z HOME)", "G21 G90 G17 G54"];
    // Replayed as bare decoded events (which blocks commanded what is
    // unknown), every position carries the calibration interval: the start
    // at home straddles the X minimum, as the refused job reported.
    let m = homed(WorkFrame::identity());
    let mut s = Session::new(m.clone(), manifest(), &dialect_definition(&m), &|| false).unwrap();
    let (_, checked) = posted(&[], &first);
    for check in &checked {
        let motion = check.motion.clone().unwrap();
        s.event(Event::Motion { motion }, check.event_index, &|| false)
            .unwrap();
    }
    let report = s.finish(2, &|| false).unwrap().clone();
    assert_eq!(report.status, Status::Uncertain);
    // Both moves start at home, where the interval straddles the limits: X
    // and Y on their minimums (and Z on its maximum, for the first).
    for motion in [1, 2] {
        let found = report
            .findings
            .iter()
            .find(|f| f.kind == "machine_travel" && f.motion_id == Some(motion))
            .unwrap();
        assert_eq!((found.status, found.fraction), (Status::Uncertain, 0.));
    }
    let x = "axis X calibration interval -0.020000 to 0.020000 mm (expected position 0.000000 mm) straddles the minimum 0.000000 mm of declared [0, 508] machine travel";
    assert!(
        report
            .findings
            .iter()
            .any(|f| f.motion_id == Some(2) && f.detail.contains(x)),
        "{:?}",
        report.findings
    );
    // With the posted blocks: no block has commanded X or Y yet when each
    // move starts, so the machine is exactly at home there, on its limits,
    // inside. The interval enters an axis with its word: Z 10 and X 50 Y 40
    // are well inside with it. Comments name no axis.
    let found = exact_axes_travel(m.clone(), &header, &first);
    assert!(found.is_empty(), "{found:?}");
    // The same in a frame turned a quarter about Z: work X drives machine Y
    // and work Y machine X. A move along work X leaves machine X exactly at
    // home; one naming work Y commands machine X, at its minimum.
    let quarter = WorkFrame {
        origin_mm: [0., 0., 0.],
        x: [0., 1., 0.],
        y: [-1., 0., 0.],
        z: [0., 0., 1.],
    };
    let m = homed(quarter);
    let found = exact_axes_travel(
        m.clone(),
        &[],
        &[
            ("G0 Z10", Some((HOME_TIP, [0., 0., 10.]))),
            ("G0 X50", Some(([0., 0., 10.], [50., 0., 10.]))),
        ],
    );
    assert!(found.is_empty(), "{found:?}");
    let found = exact_axes_travel(
        m,
        &[],
        &[
            ("G0 Z10", Some((HOME_TIP, [0., 0., 10.]))),
            ("G0 X50 Y0", Some(([0., 0., 10.], [50., 0., 10.]))),
        ],
    );
    assert_eq!(found.len(), 1, "{found:?}");
    assert_eq!(
        (found[0].status, found[0].bodies[0].as_str(), found[0].fraction),
        (Status::Uncertain, "axis_x", 1.)
    );
}

#[test]
fn a_commanded_axis_carries_the_interval_and_real_overtravel_is_caught() {
    let m = homed(WorkFrame::identity());
    let down = ("G0 Z10", Some((HOME_TIP, [0., 0., 10.])));
    let one = |found: Vec<Finding>| {
        assert_eq!(found.len(), 1, "{found:?}");
        found[0].clone()
    };
    // A work-coordinate move ending beyond the X minimum by more than the
    // interval: a violation.
    let found = one(exact_axes_travel(
        m.clone(),
        &[],
        &[down, ("G0 X-0.03", Some(([0., 0., 10.], [-0.03, 0., 10.])))],
    ));
    assert_eq!(found.status, Status::Violation);
    assert_eq!((found.bodies[0].as_str(), found.fraction), ("axis_x", 1.));
    assert!(found.detail.contains("whole interval is beyond the minimum"), "{}", found.detail);
    // Ending within the interval of the limit: straddling, uncertain.
    let found = one(exact_axes_travel(
        m.clone(),
        &[],
        &[down, ("G0 X-0.01", Some(([0., 0., 10.], [-0.01, 0., 10.])))],
    ));
    assert_eq!(found.status, Status::Uncertain);
    // Back on the limit after a work move: the axis is at a commanded work
    // position now, known only to the interval, so it straddles the limit.
    let found = one(exact_axes_travel(
        m.clone(),
        &[],
        &[
            down,
            ("G0 X50", Some(([0., 0., 10.], [50., 0., 10.]))),
            ("G0 X0", Some(([50., 0., 10.], [0., 0., 10.]))),
        ],
    ));
    assert_eq!(found.status, Status::Uncertain);
    assert_eq!((found.motion_id, found.fraction), (Some(3), 1.));
    // A word equal to the decoded position still commands its axis: the
    // controller goes to that work position, which the interval places.
    // In a move's block it reaches the move's end ...
    let found = one(exact_axes_travel(
        m.clone(),
        &[],
        &[("G0 X0 Z10", Some((HOME_TIP, [0., 0., 10.])))],
    ));
    assert_eq!(found.status, Status::Uncertain);
    assert_eq!((found.bodies[0].as_str(), found.fraction), ("axis_x", 1.));
    // ... and a block that decodes to no move at all sends the axis there
    // too: held to travel where it stands, and at the next move's start.
    let found = exact_axes_travel(m.clone(), &[], &[("G0 X0", None), down]);
    assert_eq!(found.len(), 2, "{found:?}");
    for (f, motion) in found.iter().zip([None, Some(2)]) {
        assert_eq!(
            (f.status, f.bodies[0].as_str(), f.motion_id, f.fraction),
            (Status::Uncertain, "axis_x", motion, 0.)
        );
    }
    // Words in a block outside the decoded records count too, and so do a
    // record's blocks the chunk does not carry.
    let found = exact_axes_travel(m.clone(), &["G0 Y0"], &[down]);
    assert_eq!(found.len(), 2, "{found:?}");
    for (f, motion) in found.iter().zip([None, Some(1)]) {
        assert_eq!(
            (f.bodies[0].as_str(), f.motion_id, f.fraction),
            ("axis_y", motion, 0.)
        );
    }
    let (lines, checked) = posted(&[], &[down]);
    let mut s = Session::new(m.clone(), manifest(), &dialect_definition(&m), &|| false).unwrap();
    s.chunk(&lines[..0], &checked, &|| false).unwrap();
    assert_eq!(travel_finding(&mut s, 1).unwrap().status, Status::Uncertain);
    // A definition without the dialect's axis words: every block commands
    // every axis.
    let (lines, checked) = posted(&["G21"], &[down]);
    let mut s = Session::new(m.clone(), manifest(), &definition(&m), &|| false).unwrap();
    s.chunk(&lines, &checked, &|| false).unwrap();
    assert_eq!(travel_finding(&mut s, 1).unwrap().status, Status::Uncertain);
    // A declared route waypoint puts its axes back at a machine position:
    // after a tool change whose route homes Z, then X and Y, the next move
    // starts exactly at home again, on the limits, inside. Without those
    // waypoints the axes keep the interval their last work command gave them.
    let change = |home_xy: bool| {
        let mut m = m.clone();
        let route = m.tool_change_routes.get_mut("t1").unwrap();
        route.before_index_mm = vec![[None, None, Some(60.)]];
        if home_xy {
            route.before_index_mm.push([Some(0.), Some(0.), None]);
        }
        let mut stock = manifest();
        stock.tool_change = Some(ToolChangePose::Gauge(GaugePose {
            gauge_z_mm: 60.,
            xy_mm: home_xy.then_some([0., 0.]),
        }));
        let after = if home_xy { [0., 0., 35.] } else { [0., 0.01, 35.] };
        let (mut lines, mut checked) = posted(
            &[],
            &[
                down,
                ("G0 X0 Y0.01", Some(([0., 0., 10.], [0., 0.01, 10.]))),
                ("T1 M6", None),
                ("G0 Z10", Some((after, [after[0], after[1], 10.]))),
            ],
        );
        checked[2].control_effects = Some(vec![Event::ToolChange {
            tool: "t1".into(),
            number: 1,
            length_register: 1,
            position_after_mm: after,
        }]);
        lines[2].motion_id = None;
        let mut s = Session::new(m.clone(), stock, &dialect_definition(&m), &|| false).unwrap();
        s.chunk(&lines, &checked, &|| false).unwrap();
        let r = s.finish(3, &|| false).unwrap().clone();
        assert_eq!(r.tool_changes, 1);
        r.findings
            .into_iter()
            .filter(|f| f.kind == "machine_travel" && f.motion_id == Some(4))
            .collect::<Vec<_>>()
    };
    let found = change(true);
    assert!(found.is_empty(), "{found:?}");
    let found = change(false);
    assert_eq!(found.len(), 1, "{found:?}");
    assert_eq!(
        (found[0].status, found[0].bodies[0].as_str(), found[0].fraction),
        (Status::Uncertain, "axis_x", 0.)
    );
}
