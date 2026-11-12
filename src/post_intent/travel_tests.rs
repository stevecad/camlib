//! #246: the plan check, the posted-output check and the kinematic machine
//! model's check hold one travel rule (`crate::travel`). Every case here is
//! put to all three.
use super::normal_limit_tests::fixture;
use super::*;
use crate::motion::{Curve, Motion, MoveKind, Plane, SpindleDirection};
use crate::verification::{FindingKind, MachineLimits, Options, Scene, Status};
use std::f64::consts::PI;

const BOX: [Vec3; 2] = [[0., 0., -300.], [300., 200., -150.]];

fn motion(curve: Curve) -> Motion {
    Motion {
        id: 1,
        operation: "op".into(),
        tool: "t1".into(),
        kind: MoveKind::Cut,
        curve,
        feed_mm_min: Some(600.),
    }
}

/// The plan check: is `curve` a machine-travel violation?
fn planned(frame: &WorkFrame, curve: &Curve) -> bool {
    let (_, _, header) = fixture(None, false);
    let scene = Scene {
        version: 1,
        units: "mm".into(),
        id: "scene".into(),
        revision: "1".into(),
        work_to_machine: frame.clone(),
        bodies: vec![],
        machine: Some(MachineLimits {
            min_tip_mm: Some(BOX[0]),
            max_tip_mm: Some(BOX[1]),
            max_feed_mm_min: 10000.,
            max_normal_accel_mm_s2: None,
            max_axis_feed_mm_min: None,
            min_gauge_mm: None,
            max_gauge_mm: None,
        }),
    };
    let report = scene
        .prepare(Options::default())
        .unwrap()
        .verify_motion(
            &header.tools[0].clone().prepare().unwrap(),
            &motion(curve.clone()),
            &|| false,
        )
        .unwrap();
    assert!(report.coverage.machine_tip_travel_examined);
    // Travel is decided, never left uncertain.
    assert_ne!(report.status, Status::Uncertain);
    report
        .findings
        .iter()
        .any(|f| f.kind == FindingKind::MachineTravel && f.status == Status::Violation)
}

/// The posted-output check: the plan's `source` move written as words that
/// decode to `decoded`.
fn posted(frame: &WorkFrame, source: &Curve, decoded: &Curve) -> Result<(), String> {
    let (checker, _, mut header) = fixture(None, false);
    header.initial_position_mm = source.start();
    let mut settings = checker.settings.clone();
    settings.decoded_initial_position_mm = decoded.start();
    settings.tip_travel_mm = Some(BOX);
    settings.work_to_machine = frame.clone();
    let (mut checker, _) = IntentChecker::begin(&header, settings, &["BEGIN".into()])?;
    let control = |position: Vec3| {
        vec![
            Event::WorkOffset { number: 1 },
            Event::ToolChange {
                tool: "t1".into(),
                number: 1,
                length_register: 1,
                position_after_mm: position,
            },
            Event::Spindle {
                rpm: 1000.,
                direction: SpindleDirection::Cw,
            },
        ]
    };
    let mut expected = control(source.start());
    expected.push(Event::Motion {
        motion: motion(source.clone()),
    });
    let mut actual = control(decoded.start());
    actual.push(Event::Motion {
        motion: motion(decoded.clone()),
    });
    checker
        .consume(
            &expected,
            &actual.into_iter().map(|e| vec![e]).collect::<Vec<_>>(),
            &vec![vec!["BLOCK".into()]; expected.len()],
            &|| false,
        )
        .map(|_| ())
}

/// The kinematic machine model's check (`crate::machine::Session`) of the
/// decoded move `curve`: its machine-travel finding, if any. The axes travel
/// the tip box raised by the tool's gauge length (the controlled point is
/// the gauge line); `calibration` is the declared interval of the gauge
/// offset about that length, on machine Z.
fn machine_model(
    frame: &WorkFrame,
    curve: &Curve,
    calibration: [f64; 2],
) -> Option<crate::machine::Finding> {
    use crate::machine::*;
    let (_, _, header) = fixture(None, false);
    let tool = header.tools[0].clone();
    let length = tool.gauge_length_mm;
    // The session starts inside the travel and approaches the move.
    let home = [2., 0., 5.];
    let mut manifest: crate::stream::StreamManifest = serde_json::from_str(include_str!(
        "../../fixtures/operations/adaptive-pocket.json"
    ))
    .unwrap();
    manifest.stock = crate::stock::StockSpec {
        version: 1,
        units: "mm".into(),
        min_mm: [5000., 5000., 0.],
        max_mm: [5010., 5010., 10.],
        cell_size_mm: 1.,
        max_cells: 1000,
    }
    .into();
    manifest.initial_stock = None;
    manifest.tools = vec![tool];
    manifest.initial_position_mm = home;
    manifest.operations = vec![];
    // The stock replay reads the decoded curves at the same tolerance.
    manifest.sweep.tolerance_mm = 0.01;
    manifest.sweep.chord_error_mm = manifest.sweep.chord_error_mm.max(0.01);
    manifest.verification = None;
    manifest.tool_change = None;
    manifest.turning = None;
    let solid = |lo: Vec3, hi: Vec3| Solid::Box {
        min_mm: lo,
        max_mm: hi,
    };
    let model = Model {
        version: 1,
        units: "mm".into(),
        id: "travel-rule".into(),
        revision: "1".into(),
        process: Process::Milling,
        axes: std::array::from_fn(|k| Axis {
            drive: Drive::Head,
            min_mm: BOX[0][k] + if k == 2 { length } else { 0. },
            max_mm: BOX[1][k] + if k == 2 { length } else { 0. },
        }),
        work_offset: 1,
        work_to_machine: frame.clone(),
        work_error_min_mm: [0.; 3],
        work_error_max_mm: [0.; 3],
        initial_axes_mm: {
            let q = frame.to_design(home);
            [q[0], q[1], q[2] + length]
        },
        initial_tool: Some("t1".into()),
        rapid_mode: RapidMode::Linear,
        controller_reference: "synthetic controller, exact-stop linear G0".into(),
        tools: vec![ToolCalibration {
            tool: "t1".into(),
            length_register: 1,
            body_solids: vec![],
            gauge_offset_min_mm: [0., 0., length + calibration[0]],
            gauge_offset_max_mm: [0., 0., length + calibration[1]],
        }],
        bodies: vec![Body {
            id: "head".into(),
            component: "spindle_head".into(),
            mount: Mount::Head,
            solids: vec![solid([-1., -1., 0.], [1., 1., 10.])],
            indexed_solids: Default::default(),
        }],
        absent_components: [
            ("column".into(), "synthetic stand".into()),
            ("table".into(), "synthetic stand".into()),
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
                index_envelopes: vec![solid([-500., -500., -length - 10.], [500., 500., 10.])],
                replaces: vec!["@tool".into()],
                after_index_mm: vec![],
            },
        )]
        .into(),
        program_end_route_mm: vec![],
        options: Options {
            // The output tolerance of the posted check here: a decoded arc's
            // endpoint gap must be accepted as a curve before it is judged.
            tolerance_mm: 0.01,
            spatial_resolution_mm: 0.05,
            max_axis_tests: 1_000_000,
            max_findings: 128,
            stock_contact: crate::stock_contact::Options {
                tolerance_mm: 0.01,
                spatial_resolution_mm: 0.05,
                ..Default::default()
            },
        },
    };
    let definition = serde_json::json!({"machine":{"frame":WorkFrame::identity(),"work_to_machine":frame,"work_offset":1,"tools":[{"tool":"t1","length_register":1}]}});
    let mut session = Session::new(model, manifest, &definition, &|| false).unwrap();
    let mut events = 0;
    if curve.start() != home {
        let approach = motion(line(home, curve.start()));
        session
            .event(Event::Motion { motion: approach }, 0, &|| false)
            .unwrap();
        events += 1;
    }
    // Motion IDs increase along a program: the approach is 1, the move 2.
    let mut checked = motion(curve.clone());
    checked.id = 2;
    session
        .event(Event::Motion { motion: checked }, 1, &|| false)
        .unwrap();
    session
        .finish(events + 1, &|| false)
        .unwrap()
        .findings
        .iter()
        .find(|f| f.kind == "machine_travel" && f.motion_id == Some(2))
        .cloned()
}

/// Every check on one move: `Ok` when all hold it inside, the posted
/// refusal when all hold it outside. They never disagree.
fn both(frame: &WorkFrame, curve: &Curve) -> Result<(), String> {
    let plan = planned(frame, curve);
    let post = posted(frame, curve, curve);
    let machine = machine_model(frame, curve, [0., 0.]);
    assert_eq!(plan, post.is_err(), "{frame:?} {curve:?}: {post:?}");
    assert_eq!(
        machine.as_ref().map(|f| f.status),
        plan.then_some(Status::Violation),
        "{frame:?} {curve:?}: {machine:?}"
    );
    post
}

fn line(start: Vec3, end: Vec3) -> Curve {
    Curve::Line { start, end }
}

fn turned(angle: f64, origin: Vec3) -> WorkFrame {
    let (s, c) = angle.sin_cos();
    WorkFrame {
        origin_mm: origin,
        x: [c, s, 0.],
        y: [-s, c, 0.],
        z: [0., 0., 1.],
    }
}

/// A machine point in work coordinates (the inverse of a rotation is its
/// transpose).
fn work(frame: &WorkFrame, q: Vec3) -> Vec3 {
    let d: Vec3 = std::array::from_fn(|i| q[i] - frame.origin_mm[i]);
    [frame.x, frame.y, frame.z].map(|axis| (0..3).map(|i| d[i] * axis[i]).sum())
}

#[test]
fn a_tip_on_a_limit_is_inside_and_a_line_end_a_nanometre_beyond_is_not() {
    let quarter = WorkFrame {
        origin_mm: [100.3, 50.7, -200.1],
        x: [0., 1., 0.],
        y: [-1., 0., 0.],
        z: [0., 0., 1.],
    };
    for frame in [
        turned(0., [100., 50., -200.]),
        turned(0., [100.3, 50.7, -200.1]),
        quarter,
    ] {
        let inside = work(&frame, [102., 50., -195.]);
        for corner in 0..8 {
            let q: Vec3 = std::array::from_fn(|i| BOX[usize::from(corner & (1 << i) != 0)][i]);
            // Into a corner of the travel (three limits at once), along its
            // edges, and out of it again: all on the limits, so inside.
            both(&frame, &line(inside, work(&frame, q))).unwrap();
            both(&frame, &line(work(&frame, q), inside)).unwrap();
            for axis in 0..3 {
                let mut along = q;
                along[axis] = BOX[usize::from(corner & (1 << axis) == 0)][axis];
                both(&frame, &line(work(&frame, q), work(&frame, along))).unwrap();
                // 1e-6 mm beyond one limit, at either end of the line: a
                // violation in the plan check and a refusal of the posted
                // program, naming the axis and the side.
                let mut beyond = q;
                let low = corner & (1 << axis) == 0;
                beyond[axis] += if low { -1e-6 } else { 1e-6 };
                for curve in [
                    line(inside, work(&frame, beyond)),
                    line(work(&frame, beyond), inside),
                ] {
                    let refusal = both(&frame, &curve).unwrap_err();
                    assert!(
                        refusal.contains(&format!("machine {}", ["X", "Y", "Z"][axis]))
                            && refusal.contains(if low { "below" } else { "above" })
                            && refusal.contains("exceeds configured machine travel"),
                        "{refusal}"
                    );
                }
            }
        }
    }
}

#[test]
fn a_rotated_work_frame_is_decided_on_the_move_itself() {
    for angle in [PI / 6., -2.1, 0.3] {
        let frame = turned(angle, [140., 90., -200.]);
        frame.validate().unwrap();
        // A diagonal that starts 1 mm from the X minimum: the corners of its
        // work-axis box are outside the travel, the line is not.
        let (a, b) = ([1., 20., -195.], [250., 180., -160.]);
        both(&frame, &line(work(&frame, a), work(&frame, b))).unwrap();
        // Its end 1e-9 mm inside the X maximum is inside, 1e-6 mm beyond is
        // outside (through a general rotation a work position is a rounded
        // computation, so "on the limit" is not a binary64 statement).
        let end = |x: f64| work(&frame, [x, 180., -160.]);
        both(&frame, &line(work(&frame, a), end(300. - 1e-9))).unwrap();
        let refusal = both(&frame, &line(work(&frame, a), end(300. + 1e-6))).unwrap_err();
        assert!(
            refusal.contains("machine X") && refusal.contains("above"),
            "{refusal}"
        );
    }
}

#[test]
fn an_arc_is_held_to_its_own_representation() {
    // A half turn in the work XY plane, tangent to the machine Y maximum
    // (200) at its middle when `reach` is 0; the frame is a pure offset.
    let frame = turned(0., [100., 50., -200.]);
    let arc = |reach: f64| Curve::Arc {
        start: [45., 140. + reach, 5.],
        end: [65., 140. + reach, 5.],
        center: [55., 140. + reach, 5.],
        plane: Plane::Xy,
        sweep_rad: -PI,
    };
    both(&frame, &arc(0.)).unwrap();
    // An exact arc 1e-6 mm beyond the limit is outside: the motion tolerance
    // (1e-5 mm in the plan check, 0.01 mm of output tolerance here) is not a
    // travel allowance.
    let refusal = both(&frame, &arc(1e-6)).unwrap_err();
    assert!(
        refusal.contains("machine Y") && refusal.contains("maximum"),
        "{refusal}"
    );
    // The same in a frame turned a quarter about Z: the work -Y side is the
    // machine X maximum.
    let quarter = WorkFrame {
        origin_mm: [100., 50., -200.],
        x: [0., 1., 0.],
        y: [-1., 0., 0.],
        z: [0., 0., 1.],
    };
    let arc = |reach: f64| Curve::Arc {
        start: [-10., -190. - reach, 5.],
        end: [10., -190. - reach, 5.],
        center: [0., -190. - reach, 5.],
        plane: Plane::Xy,
        sweep_rad: PI,
    };
    both(&quarter, &arc(0.)).unwrap();
    let refusal = both(&quarter, &arc(1e-6)).unwrap_err();
    assert!(
        refusal.contains("machine X") && refusal.contains("maximum"),
        "{refusal}"
    );
}

#[test]
fn a_decoded_arc_is_inside_only_by_its_endpoint_gap() {
    // The plan: an exact half turn whose middle is `inside` below the machine
    // Y maximum (200). Its words: the same arc with the end `gap` off the
    // circle drawn from the start about the centre, as rounded words leave
    // it. The controller's path is only known within that gap of the nominal
    // circle, so the gap shrinks what the posted check admits.
    let frame = turned(0., [100., 50., -200.]);
    let arc = |inside: f64, gap: f64| Curve::Arc {
        start: [45., 140. - inside, 5.],
        end: [65., 140. - inside + gap, 5.],
        center: [55., 140. - inside, 5.],
        plane: Plane::Xy,
        sweep_rad: -PI,
    };
    // Tangent, and the words are exact: both checks hold it inside.
    both(&frame, &arc(0., 0.)).unwrap();
    // Tangent with a gap of 0.0004 mm: the plan is clear, the posted form
    // is refused, naming the axis, the nominal extreme, the gap, the limit
    // and the remedies.
    assert!(!planned(&frame, &arc(0., 0.)));
    let refusal = posted(&frame, &arc(0., 0.), &arc(0., 4e-4)).unwrap_err();
    for part in [
        "decoded arc of tool t1",
        "machine travel on machine Y",
        "nominal extreme at 200.000000 mm",
        "maximum 200.000000 mm",
        "endpoint gap of 4.000e-4 mm",
        "more decimals",
        "gap inside the limit",
        "lower it to lines",
    ] {
        assert!(refusal.contains(part), "{part}: {refusal}");
    }
    // The machine model replays the same decoded arc: not shown inside, and
    // not shown outside either (the gap is an interval about the circle).
    let found = machine_model(&frame, &arc(0., 4e-4), [0., 0.]).unwrap();
    assert_eq!(found.status, Status::Uncertain);
    assert!(
        found.detail.contains("endpoint gap 4.000e-4 mm"),
        "{}",
        found.detail
    );
    // Inside the limit by less than the gap: still not shown inside.
    let refusal = posted(&frame, &arc(3e-4, 0.), &arc(3e-4, 4e-4)).unwrap_err();
    assert!(refusal.contains("3.000e-4 mm inside"), "{refusal}");
    assert_eq!(
        machine_model(&frame, &arc(3e-4, 4e-4), [0., 0.])
            .unwrap()
            .status,
        Status::Uncertain
    );
    // Inside by more than the gap: it posts.
    posted(&frame, &arc(5e-4, 0.), &arc(5e-4, 4e-4)).unwrap();
    assert!(machine_model(&frame, &arc(5e-4, 4e-4), [0., 0.]).is_none());
    // Beyond by more than the gap: outside whatever the controller does.
    assert_eq!(
        machine_model(&frame, &arc(-5e-4, 4e-4), [0., 0.])
            .unwrap()
            .status,
        Status::Violation
    );
    assert!(posted(&frame, &arc(-5e-4, 0.), &arc(-5e-4, 4e-4)).is_err());
    // The plan check keeps a planned arc's own gap the other way (its arc is
    // camlib's representation of one curve): a planned gap of 4e-6 mm, the
    // nominal middle 1e-6 mm beyond, is not shown outside; with no gap it is.
    assert!(!planned(&frame, &arc(-1e-6, 4e-6)));
    assert!(planned(&frame, &arc(-1e-6, 0.)));
    // Those words are never admitted by the posted check.
    let refusal = posted(&frame, &arc(-1e-6, 0.), &arc(-1e-6, 4e-6)).unwrap_err();
    assert!(refusal.contains("mm beyond"), "{refusal}");
}

#[test]
fn a_word_rounded_beyond_a_limit_is_refused_not_tolerated() {
    // The work offset is not on the machine's 0.001 mm output grid: the plan
    // ends on the X maximum (machine X 300), and its word, rounded to three
    // decimals, is 0.0004 mm beyond it.
    let frame = turned(0., [100.0004, 50., -200.]);
    let start = [2., 0., 5.];
    let plan = line(start, [199.9996, 150., 50.]);
    let words = line(start, [200., 150., 50.]);
    assert!(!planned(&frame, &plan));
    posted(&frame, &plan, &plan).unwrap();
    let refusal = posted(&frame, &plan, &words).unwrap_err();
    assert!(
        refusal.contains("exceeds configured machine travel on machine X")
            && refusal.contains("above its limit 300.000000 mm")
            && refusal.contains("rounded beyond the limit"),
        "{refusal}"
    );
    // The decoded move is outside by the plan check's rule too, and by the
    // machine model's, which replays the decoded words.
    assert!(planned(&frame, &words));
    assert_eq!(
        machine_model(&frame, &words, [0., 0.]).unwrap().status,
        Status::Violation
    );
    assert!(machine_model(&frame, &plan, [0., 0.]).is_none());
    // With the offset on the grid the same position is written exactly.
    let frame = turned(0., [100., 50., -200.]);
    let plan = line(start, [200., 150., 50.]);
    both(&frame, &plan).unwrap();
}

#[test]
fn a_calibration_interval_is_inside_only_whole_and_a_violation_only_whole() {
    // The machine model's tool gauge offset is known to +-0.01 mm on machine
    // Z. A position is then an interval on Z: inside only if the whole
    // interval is inside the closed travel, a violation only if the whole
    // interval is beyond a limit, uncertain otherwise. Nothing but the
    // declared interval makes it uncertain.
    let frame = turned(0., [100., 50., -200.]);
    let start = [2., 0., 5.];
    // Machine Z of the tip: -150 (the maximum) at work Z 50.
    let to = |z: f64| line(start, [150., 100., z]);
    let interval = [-0.01, 0.01];
    // On the limit: the plan and the posted check hold it inside; with the
    // interval it straddles the limit, and the finding states both.
    both(&frame, &to(50.)).unwrap();
    let found = machine_model(&frame, &to(50.), interval).unwrap();
    assert_eq!(found.status, Status::Uncertain);
    assert_eq!(found.bodies[0], "axis_z");
    assert_eq!(found.fraction, 1.);
    let length = fixture(None, false).2.tools[0].gauge_length_mm;
    for part in [
        format!("{:.6} to {:.6} mm", length - 150.01, length - 149.99),
        format!("maximum {:.6} mm", length - 150.),
        "straddles".into(),
    ] {
        assert!(found.detail.contains(&part), "{part}: {}", found.detail);
    }
    // The whole interval inside (its upper end on the limit, and well inside).
    assert!(machine_model(&frame, &to(49.99), interval).is_none());
    assert!(machine_model(&frame, &to(40.), interval).is_none());
    // Its upper end 1e-6 mm beyond the limit: straddling, not a violation.
    let found = machine_model(&frame, &to(49.99 + 1e-6), interval).unwrap();
    assert_eq!(found.status, Status::Uncertain);
    // Its lower end on the limit is still straddling (the limit is inside);
    // 1e-6 mm more and the whole interval is beyond: a violation.
    let found = machine_model(&frame, &to(50.01), interval).unwrap();
    assert_eq!(found.status, Status::Uncertain);
    let found = machine_model(&frame, &to(50.01 + 1e-6), interval).unwrap();
    assert_eq!(found.status, Status::Violation);
    assert!(
        found.detail.contains("whole interval is beyond"),
        "{}",
        found.detail
    );
    // Without an interval the same positions are inside or a violation, as
    // in the other checks: never uncertain, and no tolerance decides.
    assert!(machine_model(&frame, &to(50.), [0., 0.]).is_none());
    let refusal = both(&frame, &to(50. + 1e-6)).unwrap_err();
    assert!(refusal.contains("machine Z"), "{refusal}");
    // The interval is on Z only: X and Y stay exact.
    assert!(machine_model(&frame, &line(start, [200., 150., 40.]), interval).is_none());
    let found = machine_model(&frame, &line(start, [200. + 1e-6, 150., 40.]), interval).unwrap();
    assert_eq!(
        (found.status, found.bodies[0].as_str()),
        (Status::Violation, "axis_x")
    );
}
