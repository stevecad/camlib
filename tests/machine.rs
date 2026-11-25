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
        end_position: None,
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
            holds_axes: false,
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
            holds_axes: false,
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

#[test]
fn a_block_the_decoder_reads_as_moving_no_axis_commands_none() {
    // Haas and FANUC headers clear the local shift with G52 X0 Y0 Z0, and
    // Haas mirror image with G100 X0 Y0 Z0 (#369): offset and mode settings
    // whose axis words are parameters. Their decoders say so
    // (`Line::holds_axes`), and the axes stay exactly at home, on the
    // limits, until a move commands them.
    let m = homed(WorkFrame::identity());
    let header = ["%", "O1000", "G21 G90 G17", "G52 X0 Y0 Z0", "G100 X0 Y0 Z0"];
    let first = [
        ("G0 Z10", Some((HOME_TIP, [0., 0., 10.]))),
        ("G0 X50 Y40 Z10", Some(([0., 0., 10.], [50., 40., 10.]))),
    ];
    let findings = |held: bool| {
        let (mut lines, checked) = posted(&header, &first);
        for line in &mut lines[3..5] {
            line.holds_axes = held;
        }
        let mut s = Session::new(m.clone(), manifest(), &dialect_definition(&m), &|| false).unwrap();
        s.chunk(&lines, &checked, &|| false).unwrap();
        let report = s.finish(2, &|| false).unwrap().clone();
        (
            report.status,
            report
                .findings
                .into_iter()
                .filter(|f| f.kind == "machine_travel")
                .collect::<Vec<_>>(),
        )
    };
    let (status, found) = findings(true);
    assert!(found.is_empty(), "{found:?}");
    assert_eq!(status, Status::Clear);
    // The same blocks, unclassified (no decoder held them): every axis they
    // name is commanded at the decoded position, which straddles the limits
    // by the interval. The conservative rule stays the default.
    // The header's finding is at no move (Z, the first axis outside), and
    // the traverse then starts from an X known only to the interval.
    let (status, found) = findings(false);
    assert_eq!(status, Status::Uncertain);
    assert!(
        found
            .iter()
            .any(|f| f.bodies[0] == "axis_z" && f.motion_id.is_none()),
        "{found:?}"
    );
    assert!(
        found
            .iter()
            .any(|f| (f.bodies[0].as_str(), f.motion_id, f.fraction) == ("axis_x", Some(2), 0.)),
        "{found:?}"
    );
}

/// Posted by Python's `Job.post_nc` (python/tests/test_placed_axes.py): a
/// Facing whose start is the home pose of a LinuxCNC mill homed at X 0, Y 0
/// (its travel minimums) and Z 0 (its maximum), its work frame known to
/// +-0.02 mm (#367).
const HOMED_FACING: &str = include_str!("../fixtures/posts/rust/homed-facing.json");
const HOMED_FACING_NC: &str = include_str!("../fixtures/posts/rust/homed-facing.nc");

fn motion_blocks(program: &str) -> Vec<&str> {
    program
        .lines()
        .filter(|l| ["G0", "G1"].contains(&l.split(' ').next().unwrap_or("")))
        .collect()
}

#[test]
fn a_move_from_a_pose_on_the_travel_limits_writes_only_the_axes_it_moves() {
    let report = camlib::post_job::run(HOMED_FACING, &AtomicBool::new(false)).unwrap();
    let report: serde_json::Value = serde_json::from_str(&report).unwrap();
    // Released without accepting anything: the machine check holds X and Y,
    // which no block writes until the first traverse, exactly at home on
    // their minimums.
    assert_eq!(report["released"], true, "{report}");
    assert_eq!(report["accepted"], json!([]));
    assert_eq!(report["machine"]["status"], "clear");
    assert_eq!(report["machine"]["complete"], true);
    let program = report["program"].as_str().unwrap();
    assert_eq!(program, HOMED_FACING_NC);
    // Z down from home names Z only (not X-250 Y-200, which would command
    // X and Y through the work offset onto the limits); the traverse after
    // it names every axis, Z included, since the first block wrote Z.
    let blocks = motion_blocks(program);
    assert_eq!(blocks[..2], ["G0 Z12", "G0 X-5 Y3.2 Z12"]);
}

#[test]
fn a_z_home_tool_change_places_z_until_a_block_writes_it() {
    let input: serde_json::Value = serde_json::from_str(HOMED_FACING).unwrap();
    let mut machine = input["machine"].clone();
    machine.as_object_mut().unwrap().remove("crash_model");
    // M6 sends the gauge line to machine Z 0: work Z 150, the 60 mm tool's
    // tip at 90, as at the start.
    machine["tool_change_position_mode"] = json!("z_home");
    machine["tool_change_home_z_mm"] = json!(0.);
    let home = [-250., -200., 90.];
    let line = |id: u64, start: [f64; 3], end: [f64; 3]| {
        json!({"kind": "motion", "motion": {"id": id, "operation": "op", "tool": "face8",
            "curve": {"kind": "line", "start": start, "end": end}, "feed_mm_min": null, "kind": "rapid"}})
    };
    let change = |at: [f64; 3]| {
        json!({"kind": "tool_change", "tool": "face8", "number": 1, "length_register": 1, "position_after_mm": at})
    };
    let over = [-5., 3.2, 12.];
    let program = json!({
        "version": 1, "id": "z-home", "revision": "1", "units": "mm", "tolerance_mm": 1e-6,
        "frame": input["frame"], "initial_position_mm": home,
        "tools": input["manifest"]["tools"],
        "events": [
            {"kind": "work_offset", "number": 1},
            change(home),
            {"kind": "spindle", "rpm": 12000, "direction": "cw"},
            line(1, home, [home[0], home[1], 12.]),
            line(2, [home[0], home[1], 12.], over),
            {"kind": "spindle", "rpm": 0, "direction": "cw"},
            change([over[0], over[1], 90.]),
            {"kind": "spindle", "rpm": 12000, "direction": "cw"},
            line(3, [over[0], over[1], 90.], [0., 0., 90.]),
            line(4, [0., 0., 90.], [0., 0., 12.]),
        ],
    });
    let out = camlib::gcode::post_program(
        &json!({"program": program, "machine": machine}).to_string(),
        &AtomicBool::new(false),
    )
    .unwrap();
    let out: serde_json::Value = serde_json::from_str(&out).unwrap();
    let blocks = motion_blocks(out["program"].as_str().unwrap());
    // At the start every axis is placed; after the second M6, Z only: the
    // move that keeps it writes X and Y, which earlier blocks wrote, and no
    // Z, until the descent writes it.
    assert_eq!(
        blocks,
        [
            "G0 Z12",
            "G0 X-5 Y3.2 Z12",
            "G0 X0 Y0",
            "G0 X0 Y0 Z12"
        ],
        "{}",
        out["program"]
    );
}

/// A decoded move between two work positions of the tip.
fn line(id: u64, start: Vec3, end: Vec3, kind: MoveKind) -> Event {
    Event::Motion {
        motion: Motion {
            id,
            operation: "face".into(),
            tool: "t1".into(),
            kind,
            curve: Curve::Line { start, end },
            feed_mm_min: (kind != MoveKind::Rapid).then_some(100.),
        },
    }
}

/// The program's own error intervals (#412): the work frame known to
/// +-0.0254 mm on every axis, and rapids moving each axis independently (the
/// machine checker boxes the whole move) or together (the native stock
/// kernel checks the tool against the stock).
fn floor_model(rapid_mode: RapidMode) -> Model {
    let mut m = model();
    m.rapid_mode = rapid_mode;
    m.work_error_min_mm = [-0.0254; 3];
    m.work_error_max_mm = [0.0254; 3];
    m
}

/// Face the whole stock top (100..110 square, top 10) down to Z 8 in rows
/// 1 mm apart that run past both edges, then feed back over the stock corner
/// (the cutter overlapping it, its tip on the faced floor) and make `last`
/// from there. Returns the session's report and the last move's ID.
fn faced_then(m: Model, last: impl Fn(Vec3) -> (Vec3, MoveKind)) -> (Report, u64) {
    let mut s = session(m);
    let mut steps = vec![
        ([98.5, 100., 35.], MoveKind::Rapid),
        ([98.5, 100., 8.], MoveKind::Plunge),
    ];
    for row in 0..=10 {
        let y = 100. + row as f64;
        let (from, to) = if row % 2 == 0 { (98.5, 111.5) } else { (111.5, 98.5) };
        if row > 0 {
            steps.push(([from, y, 8.], MoveKind::Cut));
        }
        steps.push(([to, y, 8.], MoveKind::Cut));
    }
    steps.push(([109.5, 109.5, 8.], MoveKind::Cut));
    steps.push(last([109.5, 109.5, 8.]));
    let mut at = [0., 0., 35.];
    let n = steps.len();
    for (i, (to, kind)) in steps.into_iter().enumerate() {
        s.event(line(i as u64 + 1, at, to, kind), i, &|| false).unwrap();
        at = to;
    }
    (s.finish(n as u64, &|| false).unwrap().clone(), n as u64)
}

fn stock_findings(r: &Report, id: u64) -> Vec<&Finding> {
    r.findings
        .iter()
        .filter(|f| f.motion_id == Some(id) && f.bodies.contains(&"@stock".to_string()))
        .collect()
}

fn run(m: Model, moves: Vec<Event>) -> Report {
    let mut s = session(m);
    let n = moves.len();
    for (i, e) in moves.into_iter().enumerate() {
        s.event(e, i, &|| false).unwrap();
    }
    s.finish(n as u64, &|| false).unwrap().clone()
}

#[test]
fn a_straight_lift_off_the_floor_the_tool_cut_is_clear() {
    // The tool stands on the floor it has just cut, overlapping the stock
    // corner; nothing above that floor is under it. Only the work-error
    // interval reaches below the floor: the contact the cut itself made.
    // Lifting straight up adds none. Before #412 the independent-axis lift
    // was undecided (its outer bounds overlap the floor cells).
    for mode in [RapidMode::AxisIndependent, RapidMode::Linear] {
        let (r, id) = faced_then(floor_model(mode), |at| ([at[0], at[1], 35.], MoveKind::Rapid));
        assert!(stock_findings(&r, id).is_empty(), "{mode:?}: {r:?}");
        assert_eq!(r.status, Status::Clear, "{mode:?}: {r:?}");
    }
}

#[test]
fn a_lift_past_material_above_its_start_is_still_found() {
    for mode in [RapidMode::AxisIndependent, RapidMode::Linear] {
        let m = floor_model(mode);
        // From the faced floor, a lift that also moves sideways (by one
        // micrometre) is checked as before.
        let (r, id) = faced_then(m.clone(), |at| ([at[0] + 0.001, at[1], 35.], MoveKind::Rapid));
        if mode == RapidMode::AxisIndependent {
            assert!(!stock_findings(&r, id).is_empty(), "{mode:?}: {r:?}");
        }
        // A rapid that ends inside unfaced stock and lifts out of it again:
        // the lift starts with material above its tip, under and beside it.
        let r = run(
            m.clone(),
            vec![
                line(1, [0., 0., 35.], [105.5, 105.5, 35.], MoveKind::Rapid),
                line(2, [105.5, 105.5, 35.], [105.5, 105.5, 8.], MoveKind::Rapid),
                line(3, [105.5, 105.5, 8.], [105.5, 105.5, 35.], MoveKind::Rapid),
            ],
        );
        assert!(!stock_findings(&r, 2).is_empty(), "{mode:?}: {r:?}");
        let lift = stock_findings(&r, 3);
        if mode == RapidMode::AxisIndependent {
            assert!(!lift.is_empty(), "{mode:?}: {r:?}");
            assert!(lift.iter().all(|f| f.status != Status::Clear));
        }
    }
    // A plunge cut below the top, then straight up out of the hole past its
    // unfaced walls: the walls' cells stand above the tip, so the lift stays
    // undecided (the cutter's outer bounds overlap them).
    let r = run(
        floor_model(RapidMode::AxisIndependent),
        vec![
            line(1, [0., 0., 35.], [105.5, 105.5, 35.], MoveKind::Rapid),
            line(2, [105.5, 105.5, 35.], [105.5, 105.5, 7.], MoveKind::Plunge),
            line(3, [105.5, 105.5, 7.], [105.5, 105.5, 35.], MoveKind::Rapid),
        ],
    );
    let lift = stock_findings(&r, 3);
    assert!(!lift.is_empty(), "{r:?}");
    assert!(lift.iter().all(|f| f.status == Status::Uncertain));
}
