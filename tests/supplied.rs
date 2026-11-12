//! A caller-supplied toolpath (#157) is one more operation of the same run:
//! its moves leave the stream exactly as supplied, the stock it leaves is what
//! the next operation plans from, its findings gate export like a planned
//! move's, and the posted program holds its moves. Malformed input is refused
//! with the index of the move at fault.
use camlib::{
    motion::{Curve, Motion, MoveKind, Plane},
    operation::OperationPlan,
    pocket::Operation,
    regeneration,
    stock::Stock,
    stream::*,
    supplied::{SuppliedMove, SuppliedToolpathOperation},
};
use serde_json::{Value, json};
use std::{f64::consts::PI, sync::atomic::AtomicBool};

/// The standalone example job: face to Z 5 (mill4), pocket (mill4), rest
/// pocket (mill2) of a 24 x 16 x 6 block; stock contact is checked.
const INPUT: &str = include_str!("../fixtures/posts/rust/standalone-rest.json");

fn line(kind: MoveKind, start: [f64; 3], end: [f64; 3], feed: Option<f64>) -> SuppliedMove {
    SuppliedMove {
        kind,
        curve: Curve::Line { start, end },
        feed_mm_min: feed,
    }
}

/// A slot at Z 4 inside the pocket area, cut by "another planner": its own
/// approach from clearance, a plunge, a line, a half circle, a line, a
/// helix down to Z 3.5 and a fed retract. Awkward decimals on purpose.
fn slot() -> Vec<SuppliedMove> {
    let y = 6.123456789012345;
    vec![
        line(MoveKind::Rapid, [6., y, 8.], [6., y, 5.5], None),
        line(MoveKind::Plunge, [6., y, 5.5], [6., y, 4.], Some(150.)),
        line(MoveKind::Cut, [6., y, 4.], [12., y, 4.], Some(612.5)),
        SuppliedMove {
            kind: MoveKind::Cut,
            curve: Curve::Arc {
                start: [12., y, 4.],
                end: [12., y + 4., 4.],
                center: [12., y + 2., 4.],
                plane: Plane::Xy,
                sweep_rad: PI,
            },
            feed_mm_min: Some(500.),
        },
        line(MoveKind::Cut, [12., y + 4., 4.], [6., y + 4., 4.], Some(612.5)),
        SuppliedMove {
            kind: MoveKind::Lead,
            curve: Curve::Helix {
                start: [6., y + 4., 4.],
                end: [6., y, 3.5],
                center: [6., y + 2., 4.],
                plane: Plane::Xy,
                sweep_rad: PI,
            },
            feed_mm_min: Some(200.),
        },
        line(MoveKind::Retract, [6., y, 3.5], [6., y, 8.], Some(300.)),
    ]
}

fn operation(motions: Vec<SuppliedMove>) -> SuppliedToolpathOperation {
    SuppliedToolpathOperation {
        id: "outside".into(),
        tool: "mill4".into(),
        motions,
        clearance_z_mm: 8.,
        tolerance_mm: 1e-5,
        spindle_rpm: 9000.,
        work_offset: 1,
        tool_number: 7,
        length_register: 9,
        coolant: camlib::motion::Coolant::Mist,
        limits: None,
    }
}

type StreamOperation = Operation;
fn wrap(op: SuppliedToolpathOperation) -> Operation {
    Operation::SuppliedToolpath(op)
}

/// The job with the planned pocket replaced by `supplied`: a planned
/// operation, the supplied toolpath, another planned operation.
fn input(supplied: Option<&SuppliedToolpathOperation>) -> Value {
    let mut input: Value = serde_json::from_str(INPUT).unwrap();
    let ops = input["manifest"]["operations"].as_array().unwrap().clone();
    let mut list = vec![ops[0].clone()];
    if let Some(op) = supplied {
        list.push(serde_json::to_value(wrap(op.clone())).unwrap());
    }
    list.push(ops[2].clone());
    input["manifest"]["operations"] = list.into();
    input
}
fn manifest(input: &Value) -> StreamManifest {
    serde_json::from_value(input["manifest"].clone()).unwrap()
}

struct Run {
    motions: Vec<Motion>,
    /// Operation control frames, decoded.
    controls: Vec<Value>,
    /// The receiver's stock when each operation ended.
    stock_after: Vec<(String, Stock)>,
}
/// Run the manifest and replay every frame on an independent receiver (which
/// checks each stock change against its own stock before committing it).
fn run(m: StreamManifest) -> Result<Run, String> {
    let mut producer = StreamProducer::new(m.clone())?;
    let mut receiver = StreamReceiver::new(m.clone())?;
    let mut result = Run {
        motions: vec![],
        controls: vec![],
        stock_after: vec![],
    };
    regeneration::run_uncached(&mut producer, &AtomicBool::new(false), |frame| {
        match u16::from_le_bytes([frame[10], frame[11]]) {
            1 => result
                .motions
                .push(receiver.accept(frame, DEFAULT_FRAME_LIMIT)?),
            4 => receiver.accept_end(frame, DEFAULT_FRAME_LIMIT)?,
            7 => {
                let control = decode_operation_control(&m, frame, DEFAULT_FRAME_LIMIT)?;
                if control["stage"] == "end" {
                    result.stock_after.push((
                        control["operation"].as_str().unwrap().to_string(),
                        receiver.stock()?.clone(),
                    ));
                }
                result.controls.push(control);
            }
            _ => {}
        }
        Ok(())
    })?;
    Ok(result)
}
fn upper_at(stock: &Stock, x: f64, y: f64) -> f64 {
    let [nx, _] = stock.dimensions();
    let [dx, dy] = stock.cell_widths_mm();
    let min = stock.spec().min_mm;
    let (i, j) = (
        ((x - min[0]) / dx).floor() as usize,
        ((y - min[1]) / dy).floor() as usize,
    );
    stock.cells()[j * nx + i].upper_mm
}
fn cut_length_at(run: &Run, operation: &str, z: f64) -> f64 {
    run.motions
        .iter()
        .filter(|m| m.operation == operation && m.kind == MoveKind::Cut)
        .filter_map(|m| match m.curve {
            Curve::Line { start, end } if start[2] == z && end[2] == z => {
                Some((end[0] - start[0]).hypot(end[1] - start[1]))
            }
            _ => None,
        })
        .sum()
}

#[test]
fn supplied_moves_leave_the_stream_bit_for_bit_between_planned_operations() {
    let op = operation(slot());
    let result = run(manifest(&input(Some(&op)))).unwrap();
    let ids: Vec<&str> = result.stock_after.iter().map(|s| s.0.as_str()).collect();
    assert_eq!(ids, ["face", "outside", "rest"]);
    let begin = result
        .controls
        .iter()
        .find(|c| c["operation"] == "outside" && c["stage"] == "begin")
        .unwrap();
    let report = &begin["report"];
    assert_eq!(report["strategy"], "supplied_toolpath");
    assert_eq!(report["supplied_moves"], 7);
    // Facing ends at its clearance height, 8, which is the first supplied
    // point's: camlib adds one horizontal rapid to it.
    assert_eq!(report["approach_moves"], 1);
    let outside: Vec<&Motion> = result
        .motions
        .iter()
        .filter(|m| m.operation == "outside")
        .collect();
    assert_eq!(outside.len(), 8);
    let face_end = result
        .motions
        .iter()
        .rfind(|m| m.operation == "face")
        .unwrap()
        .curve
        .end();
    let first = op.motions[0].curve.start();
    assert_eq!(
        (outside[0].kind, &outside[0].curve, outside[0].feed_mm_min),
        (
            MoveKind::Rapid,
            &Curve::Line {
                start: face_end,
                end: first
            },
            None
        )
    );
    assert_eq!((face_end[2], first[2]), (8., 8.));
    let first_id = report["first_supplied_motion_id"].as_u64().unwrap();
    for (i, (streamed, supplied)) in outside[1..].iter().zip(&op.motions).enumerate() {
        assert_eq!(streamed.id, first_id + i as u64);
        assert_eq!(streamed.tool, "mill4");
        // Exactly the supplied move: same kind, same feed, and every
        // coordinate the same binary64 value (a fed retract stays fed).
        assert_eq!(streamed.kind, supplied.kind, "move {i}");
        assert_eq!(streamed.feed_mm_min, supplied.feed_mm_min, "move {i}");
        assert_eq!(
            serde_json::to_string(&streamed.curve).unwrap(),
            serde_json::to_string(&supplied.curve).unwrap(),
            "move {i}"
        );
        assert_eq!(streamed.curve, supplied.curve, "move {i}");
    }
    // Its declared spindle, coolant, tool and registers, like any operation.
    assert_eq!(
        begin["events"],
        json!([
            {"kind": "work_offset", "number": 1},
            {"kind": "tool_change", "tool": "mill4", "number": 7, "length_register": 9,
             "position_after_mm": face_end},
            {"kind": "spindle", "rpm": 9000.0, "direction": "cw"},
            {"kind": "coolant", "mode": "mist"},
        ])
    );
    let end = result
        .controls
        .iter()
        .find(|c| c["operation"] == "outside" && c["stage"] == "end")
        .unwrap();
    assert_eq!(end["report"]["moves_emitted"], 8);
    assert_eq!(
        end["events"],
        json!([{"kind": "coolant", "mode": "off"},
               {"kind": "spindle", "rpm": 0.0, "direction": "cw"}])
    );
}

#[test]
fn the_stock_a_supplied_toolpath_leaves_is_what_the_next_operation_sees() {
    let op = operation(slot());
    let with = run(manifest(&input(Some(&op)))).unwrap();
    let without = run(manifest(&input(None))).unwrap();
    let y = 6.123456789012345;
    // The receiver's own stock after each operation: faced to Z 5, then the
    // supplied slot at Z 4 (and the helix end at 3.5), nothing elsewhere.
    let faced = &with.stock_after[0].1;
    let cut = &with.stock_after[1].1;
    // (The stock's upper bound carries the sweep tolerance, 1e-5.)
    let near = |value: f64, z: f64| (z..=z + 1e-4).contains(&value);
    assert!(near(upper_at(faced, 9., y), 5.));
    assert!(near(upper_at(cut, 9., y), 4.), "{}", upper_at(cut, 9., y));
    assert!(upper_at(cut, 6., y) < 4. && upper_at(cut, 6., y) >= 3.5);
    assert!(near(upper_at(cut, 20., 2.), 5.));
    // The rest pocket (floor Z 3, in 1 mm steps: levels 4 and 3) plans from
    // that stock: at Z 4 it no longer cuts along the supplied slot.
    let (after_slot, from_faced) = (
        cut_length_at(&with, "rest", 4.),
        cut_length_at(&without, "rest", 4.),
    );
    assert!(from_faced > 0.);
    assert!(
        after_slot < from_faced - 10.,
        "rest cuts {after_slot} mm at Z 4 after the slot, {from_faced} mm without it"
    );
    // The rest pocket still finishes the pocket floor in both runs.
    for run in [&with, &without] {
        let last = &run.stock_after.last().unwrap().1;
        assert!(near(upper_at(last, 9., 8.), 3.));
    }
}

fn post(input: &Value) -> Result<Value, String> {
    let report = camlib::post_job::run(&input.to_string(), &AtomicBool::new(false))?;
    Ok(serde_json::from_str(&report).unwrap())
}

#[test]
fn the_posted_program_holds_the_supplied_moves_and_decodes_back() {
    let op = operation(slot());
    let report = post(&input(Some(&op))).unwrap();
    assert_eq!(report["released"], true, "{}", report["refused"]);
    assert_eq!(report["findings"], json!([]));
    // Every motion's block was decoded back and compared with its source.
    assert_eq!(report["motions"], report["posting"]["checked_motions"]);
    let program = report["program"].as_str().unwrap();
    // The supplied moves at the machine's 3 decimals, in order, between the
    // operation's own spindle/coolant words and the next tool change.
    let expected = [
        "M3 S9000",
        "M7",
        "G0 X6 Y6.123 Z8",
        "G0 X6 Y6.123 Z5.5",
        "G1 X6 Y6.123 Z4 F150",
        "G1 X12 Y6.123 Z4 F612.5",
        // The default post selects the plane before each arc, and an arc
        // in XY carries no Z word.
        "G17",
        "G3 X12 Y10.123 I0 J2 F500",
        "G1 X6 Y10.123 Z4 F612.5",
        "G17",
        "G3 X6 Y6.123 Z3.5 I0 J-2 F200",
        "G1 X6 Y6.123 Z8 F300",
    ];
    let lines: Vec<&str> = program.lines().collect();
    let at = lines
        .iter()
        .position(|l| *l == expected[0])
        .unwrap_or_else(|| panic!("{program}"));
    assert_eq!(&lines[at..at + expected.len()], expected, "{program}");
}

#[test]
fn a_supplied_rapid_through_stock_is_a_finding_that_gates_export() {
    // The plunge is declared a rapid: it ends 1 mm inside the faced stock.
    let mut moves = slot();
    let y = moves[0].curve.start()[1];
    moves[0] = line(MoveKind::Rapid, [6., y, 8.], [6., y, 4.], None);
    moves.remove(1);
    let op = operation(moves);
    let mut input = input(Some(&op));
    let report = post(&input).unwrap();
    assert_eq!(report["released"], false);
    assert!(report["program"].is_null());
    assert_eq!(report["refused"]["statuses"], json!(["violation"]));
    let findings = report["findings"].as_array().unwrap();
    assert_eq!(findings.len(), 1, "{findings:?}");
    // The rapid is supplied move 0: after the faced operation's moves and
    // camlib's one travel move.
    let run = run(manifest(&input)).unwrap();
    let faced = run.motions.iter().filter(|m| m.operation == "face").count() as u64;
    assert_eq!(findings[0]["completed"], faced + 2);
    assert_eq!(findings[0]["findings"][0]["kind"], "stock_contact");
    assert_eq!(findings[0]["findings"][0]["part"], "cutter");
    // Released only once the violation is accepted, by name.
    input["accept"] = json!(["unchecked_machine", "violation"]);
    let report = post(&input).unwrap();
    assert_eq!(report["released"], true);
    assert!(
        report["program"]
            .as_str()
            .unwrap()
            .contains("G0 X6 Y6.123 Z4\n")
    );
}

#[test]
fn a_run_with_a_supplied_toolpath_must_check_stock_contact() {
    let op = operation(slot());
    let mut input = input(Some(&op));
    input["manifest"]["verification"] = Value::Null;
    let error = StreamProducer::new(manifest(&input)).err().unwrap();
    assert!(
        error.contains("supplied toolpath outside") && error.contains("stock_contact"),
        "{error}"
    );
    // A planned job needs no such rule.
    let mut planned = self::input(None);
    planned["manifest"]["verification"] = Value::Null;
    StreamProducer::new(manifest(&planned)).unwrap();
}

fn prepare(
    op: SuppliedToolpathOperation,
    initial: [f64; 3],
    center_cutting: bool,
) -> Result<(Vec<Motion>, OperationPlan), String> {
    let m = manifest(&input(None));
    let mut tool = m.tools[0].clone();
    tool.cutter.center_cutting = center_cutting;
    let cancel = AtomicBool::new(false);
    let stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut plan = OperationPlan::prepare_with_stock(
        Operation::SuppliedToolpath(op),
        &tool,
        &stock,
        initial,
        41,
        &cancel,
    )?;
    let mut motions = vec![];
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel)? {
        motions.push(motion);
    }
    plan.finish_report(&cancel)?;
    Ok((motions, plan))
}

#[test]
fn travel_to_the_first_point_is_the_standard_approach_and_nothing_else() {
    // From higher up and elsewhere: straight down to the first point's
    // height, then across at it. Both at or above clearance.
    let op = operation(slot());
    let first = op.motions[0].curve.start();
    let (motions, _) = prepare(op.clone(), [1., 2., 30.], true).unwrap();
    assert_eq!(motions.len(), 9);
    assert_eq!(
        motions[0].curve,
        Curve::Line {
            start: [1., 2., 30.],
            end: [1., 2., 8.]
        }
    );
    assert_eq!(
        motions[1].curve,
        Curve::Line {
            start: [1., 2., 8.],
            end: first
        }
    );
    assert!(
        motions[..2]
            .iter()
            .all(|m| m.kind == MoveKind::Rapid && m.feed_mm_min.is_none())
    );
    assert_eq!(
        motions.iter().map(|m| m.id).collect::<Vec<_>>(),
        (41..50).collect::<Vec<_>>()
    );
    // Starting on the first point: no travel at all.
    let (motions, plan) = prepare(op.clone(), first, true).unwrap();
    assert_eq!(motions.len(), 7);
    let report = serde_json::to_value(plan.report()).unwrap();
    assert_eq!(report["approach_moves"], 0);
    assert_eq!(report["first_supplied_motion_id"], 41);
    assert_eq!(report["geometric_residual_status"], "unknown");
    // Only a vertical leg.
    let (motions, _) = prepare(op, [first[0], first[1], 12.], true).unwrap();
    assert_eq!(motions.len(), 8);
}

#[test]
fn every_malformed_move_is_refused_with_its_index() {
    let y = 6.123456789012345;
    type Change = Box<dyn Fn(&mut Vec<SuppliedMove>)>;
    let cases: Vec<(&str, Change, &str, &str)> = vec![
        (
            "gap",
            Box::new(|m| {
                m[3] = line(
                    MoveKind::Cut,
                    [12., 6.123456789012346, 4.],
                    [12., 10., 4.],
                    Some(500.),
                );
                m.truncate(4);
            }),
            "move 3 of 4",
            "consecutive moves must share that point exactly",
        ),
        (
            "cut without a feed",
            Box::new(|m| m[2].feed_mm_min = None),
            "move 2 of 7",
            "a cut move needs a feed",
        ),
        (
            "zero feed",
            Box::new(|m| m[1].feed_mm_min = Some(0.)),
            "move 1 of 7",
            "feed_mm_min 0 must be positive and finite",
        ),
        (
            "infinite feed",
            Box::new(|m| m[4].feed_mm_min = Some(f64::INFINITY)),
            "move 4 of 7",
            "must be positive and finite",
        ),
        (
            "rapid arc",
            Box::new(|m| {
                m[3].kind = MoveKind::Rapid;
                m[3].feed_mm_min = None;
            }),
            "move 3 of 7",
            "a rapid must be a line",
        ),
        (
            "cycle kind",
            Box::new(|m| m[1].kind = MoveKind::Tap),
            "move 1 of 7",
            "belongs to a cycle",
        ),
        (
            "probing kind",
            Box::new(|m| m[1].kind = MoveKind::ProbeMeasure),
            "move 1 of 7",
            "belongs to a cycle",
        ),
        (
            "non-finite coordinate",
            Box::new(|m| {
                let Curve::Line { end, .. } = &mut m[2].curve else {
                    unreachable!()
                };
                end[0] = f64::NAN;
            }),
            "move 2 of 7",
            "endpoints must be finite",
        ),
        (
            "zero-length line",
            Box::new(move |m| {
                m.insert(2, line(MoveKind::Cut, [6., y, 4.], [6., y, 4.], Some(100.)))
            }),
            "move 2 of 8",
            "nonzero length",
        ),
        (
            "arc end off its circle",
            Box::new(|m| {
                let Curve::Arc { center, .. } = &mut m[3].curve else {
                    unreachable!()
                };
                // 1 um off: the end is 2 um from where the sweep arrives.
                center[1] += 0.001;
            }),
            "move 3 of 7",
            "circular endpoint disagrees",
        ),
        (
            "helix without axial travel",
            Box::new(move |m| {
                let Curve::Helix { end, .. } = &mut m[5].curve else {
                    unreachable!()
                };
                end[2] = 4.;
                m[6] = line(MoveKind::Retract, [6., y, 4.], [6., y, 8.], Some(300.));
            }),
            "move 5 of 7",
            "helix must have nonzero axial displacement",
        ),
        (
            "start below clearance",
            Box::new(|m| {
                m.remove(0);
            }),
            "move 0 of 6",
            "starts at Z 5.5, below clearance_z_mm 8",
        ),
        (
            "end below clearance",
            Box::new(|m| {
                m.pop();
            }),
            "move 5 of 6",
            "ends at Z 3.5, below clearance_z_mm 8",
        ),
    ];
    for (name, change, index, message) in cases {
        let mut moves = slot();
        change(&mut moves);
        let error = prepare(operation(moves), [0., 0., 8.], true)
            .err()
            .unwrap_or_else(|| panic!("{name}: accepted"));
        assert!(
            error.starts_with(&format!("supplied toolpath outside: {index}: "))
                && error.contains(message),
            "{name}: {error}"
        );
    }
    // The arc that is refused at 1e-5 is a valid curve at a declared 0.01.
    let mut op = operation(slot());
    let Curve::Arc { center, .. } = &mut op.motions[3].curve else {
        unreachable!()
    };
    center[1] += 0.001;
    op.tolerance_mm = 0.01;
    prepare(op, [0., 0., 8.], true).unwrap();
}

#[test]
fn operation_parameters_have_no_defaults_and_are_checked() {
    type Change = Box<dyn Fn(&mut SuppliedToolpathOperation)>;
    let cases: Vec<(Change, &str)> = vec![
        (Box::new(|op| op.motions.clear()), "at least one move"),
        (
            Box::new(|op| op.clearance_z_mm = 6.),
            "clearance_z_mm 6 must be above the stock top 6",
        ),
        (
            Box::new(|op| op.spindle_rpm = 0.),
            "spindle_rpm 0 must be positive",
        ),
        (Box::new(|op| op.tool_number = 0), "tool_number"),
        (Box::new(|op| op.work_offset = 0), "work_offset 0"),
        (Box::new(|op| op.tolerance_mm = 0.5), "tolerance_mm 0.5"),
        (Box::new(|op| op.tool = "mill2".into()), "does not match"),
    ];
    for (change, message) in cases {
        let mut op = operation(slot());
        change(&mut op);
        let error = prepare(op, [0., 0., 8.], true).err().unwrap();
        assert!(
            error.starts_with("invalid supplied toolpath parameters: ") && error.contains(message),
            "{error}"
        );
    }
    // The operation starts at or above its clearance height, as planned ones do.
    let error = prepare(operation(slot()), [0., 0., 7.], true)
        .err()
        .unwrap();
    assert!(
        error.contains("start Z 7 is below clearance_z_mm 8"),
        "{error}"
    );
    // Every field is required: nothing stands in for the machine's values.
    let full = serde_json::to_value(wrap(operation(slot()))).unwrap();
    assert_eq!(full["kind"], "supplied_toolpath");
    for field in [
        "tool",
        "motions",
        "clearance_z_mm",
        "tolerance_mm",
        "spindle_rpm",
        "work_offset",
        "tool_number",
        "length_register",
        "coolant",
    ] {
        let mut missing = full.clone();
        missing.as_object_mut().unwrap().remove(field);
        assert!(
            serde_json::from_value::<StreamOperation>(missing).is_err(),
            "{field}"
        );
    }
    let mut unknown = full.clone();
    unknown["feed_mm_min"] = 600.0.into();
    assert!(serde_json::from_value::<StreamOperation>(unknown).is_err());
    // Serialised and read back, it is the same operation, to the last bit.
    let back: Operation = serde_json::from_str(&serde_json::to_string(&full).unwrap()).unwrap();
    let Operation::SuppliedToolpath(back) = back else {
        unreachable!()
    };
    assert_eq!(back.motions, slot());
}

#[test]
fn a_beam_or_a_probe_runs_no_supplied_toolpath() {
    let m = manifest(&input(None));
    let stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    for (shape, message) in [
        (camlib::tool::CutterShape::Beam, "three-axis milling"),
        (camlib::tool::CutterShape::Probe, "is a probe"),
    ] {
        let mut tool = m.tools[0].clone();
        tool.cutter.shape = shape;
        let error = OperationPlan::prepare_with_stock(
            Operation::SuppliedToolpath(operation(slot())),
            &tool,
            &stock,
            [0., 0., 8.],
            1,
            &AtomicBool::new(false),
        )
        .err()
        .unwrap();
        assert!(error.contains(message), "{error}");
    }
}

#[test]
fn a_cutter_that_is_not_center_cutting_descends_only_through_air() {
    // Into the full block (top Z 6): the plunge is supplied move 1.
    let error = prepare(operation(slot()), [0., 0., 8.], false)
        .err()
        .unwrap();
    assert!(
        error.starts_with("supplied toolpath outside: move 1 of 7: descends to Z 4")
            && error.contains("not center cutting"),
        "{error}"
    );
    // Beside the block the same descent meets nothing, then it cuts sideways.
    let moves = vec![
        line(MoveKind::Rapid, [-4., 8., 8.], [-4., 8., 6.5], None),
        line(MoveKind::Plunge, [-4., 8., 6.5], [-4., 8., 5.], Some(150.)),
        line(MoveKind::Cut, [-4., 8., 5.], [10., 8., 5.], Some(600.)),
        line(MoveKind::Retract, [10., 8., 5.], [10., 8., 8.], Some(600.)),
    ];
    let (motions, _) = prepare(operation(moves), [0., 0., 8.], false).unwrap();
    assert_eq!(motions.len(), 5);
}

#[test]
fn limits_apply_to_every_supplied_move() {
    use camlib::limits::{AvoidArea, Limits};
    let m = manifest(&input(None));
    let Operation::Pocket(pocket) = &m.operations[1] else {
        panic!("fixture")
    };
    // The pocket area (4..14 x 4..12) as a clamp up to Z 7: the slot's
    // plunge (move 1) is the first move inside it below its top.
    let mut op = operation(slot());
    op.limits = Some(Limits {
        contain: None,
        avoid: vec![AvoidArea {
            region: pocket.region.clone(),
            top_z_mm: 7.,
        }],
    });
    let error = prepare(op.clone(), [0., 0., 8.], true).err().unwrap();
    assert!(
        error.starts_with("supplied toolpath outside: move 0 of 7: passes through an avoid area"),
        "{error}"
    );
    // Above the clamp's top the same moves are allowed.
    op.limits.as_mut().unwrap().avoid[0].top_z_mm = 3.;
    prepare(op.clone(), [0., 0., 8.], true).unwrap();
    // As the containment, the 4 mm cutter's half circle about x 12 reaches
    // x 16, outside it: the first cutting move that leaves is refused.
    op.limits = Some(Limits {
        contain: Some(pocket.region.clone()),
        avoid: vec![],
    });
    let error = prepare(op, [0., 0., 8.], true).err().unwrap();
    assert!(
        error.contains("of 7: reaches into an avoid area below its top or out of the containment"),
        "{error}"
    );
}
