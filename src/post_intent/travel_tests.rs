//! #246: the plan check and the posted-output check hold one travel rule
//! (`crate::travel`). Every case here is put to both.
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

/// Both checks on one move: `Ok` when both hold it inside, the posted
/// refusal when both hold it outside. They never disagree.
fn both(frame: &WorkFrame, curve: &Curve) -> Result<(), String> {
    let plan = planned(frame, curve);
    let post = posted(frame, curve, curve);
    assert_eq!(plan, post.is_err(), "{frame:?} {curve:?}: {post:?}");
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
        refusal.contains("machine Y") && refusal.contains("above"),
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
        refusal.contains("machine X") && refusal.contains("above"),
        "{refusal}"
    );
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
    // The decoded move is outside by the plan check's rule too.
    assert!(planned(&frame, &words));
    // With the offset on the grid the same position is written exactly.
    let frame = turned(0., [100., 50., -200.]);
    let plan = line(start, [200., 150., 50.]);
    both(&frame, &plan).unwrap();
}
