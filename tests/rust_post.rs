//! The Rust default post (issue #120) without Python: the program it posts for
//! a fixture job equals, byte for byte, the program the Python default post
//! wrote for it (fixtures/posts/rust/*.nc, produced by `Job.post_nc`;
//! python/tests/test_rust_post.py keeps the Python side equal to it), and
//! the same gate withholds it.
use camlib::gcode::Machine;
use camlib::gcode::decode::{Context, Decoder};
use camlib::motion::MoveKind;
use serde_json::Value;
use std::sync::atomic::AtomicBool;

const INPUT: &str = include_str!("../fixtures/posts/rust/standalone-rest.json");
const PROGRAM: &str = include_str!("../fixtures/posts/rust/standalone-rest.nc");

fn post(input: &Value) -> Result<Value, String> {
    let report = camlib::post_job::run(&input.to_string(), &AtomicBool::new(false))?;
    Ok(serde_json::from_str(&report).unwrap())
}
fn fixture() -> Value {
    serde_json::from_str(INPUT).unwrap()
}

#[test]
fn posts_the_python_program_byte_for_byte() {
    let report = post(&fixture()).unwrap();
    assert_eq!(report["released"], true);
    assert_eq!(report["program"].as_str().unwrap(), PROGRAM);
    assert_eq!(report["lines"], PROGRAM.lines().count());
    assert_eq!(report["accepted"], serde_json::json!(["unchecked_machine"]));
    assert_eq!(
        report["unchecked"],
        serde_json::json!(["scene", "material", "machine"])
    );
    assert_eq!(report["machine"]["status"], "unchecked_machine");
    // Every planned motion was decoded back and replayed.
    assert_eq!(report["motions"], report["posting"]["checked_motions"]);
}

#[test]
fn withholds_the_program_unless_findings_are_accepted() {
    let mut input = fixture();
    input["accept"] = serde_json::json!([]);
    let report = post(&input).unwrap();
    assert_eq!(report["released"], false);
    assert!(report["program"].is_null());
    assert_eq!(
        report["refused"]["statuses"],
        serde_json::json!(["unchecked_machine"])
    );
    input["accept"] = serde_json::json!(["everything"]);
    assert!(post(&input).unwrap_err().contains("accept takes"));
}

#[test]
fn the_machine_json_states_every_field_and_is_checked() {
    for (change, message) in [
        ("decimals", "missing field `decimals`"),
        ("rapid_mm_min", "missing field `rapid_mm_min`"),
    ] {
        let mut input = fixture();
        input["machine"].as_object_mut().unwrap().remove(change);
        assert!(post(&input).unwrap_err().contains(message), "{change}");
    }
    let mut input = fixture();
    input["machine"]["decimals"] = 1.into();
    assert!(
        post(&input)
            .unwrap_err()
            .contains("invalid machine output units/precision")
    );
    let mut input = fixture();
    input["machine"]["dialect"]["axes"] = serde_json::json!(["X", "Y", "X"]);
    assert!(
        post(&input)
            .unwrap_err()
            .contains("distinct uppercase letters")
    );
    let mut input = fixture();
    input["manifest"]["turning"] = serde_json::json!({});
    assert!(post(&input).is_err());
}

#[test]
fn a_declared_probing_back_end_is_validated_and_changes_no_other_output() {
    // The dialect states its probing back end (#7); a job without probing
    // posts the same program whatever is declared.
    let mut input = fixture();
    input["machine"]["dialect"]["probing"] = "g38".into();
    let report = post(&input).unwrap();
    assert_eq!(report["program"].as_str().unwrap(), PROGRAM);
    for (probing, feed, switching, message) in [
        ("renishaw", Value::Null, false, "probing must be None"),
        (
            "inspection_plus",
            Value::Null,
            false,
            "probe_fast_feed_mm_min",
        ),
        ("g38", Value::Null, true, "probe_switching exactly"),
        ("inspection_plus", (-1.).into(), false, "positive feed"),
    ] {
        let mut input = fixture();
        let d = &mut input["machine"]["dialect"];
        d["probing"] = probing.into();
        d["probe_fast_feed_mm_min"] = feed;
        d["probe_switching"] = switching.into();
        assert!(post(&input).unwrap_err().contains(message), "{probing}");
    }
    let mut input = fixture();
    input["machine"]["dialect"]
        .as_object_mut()
        .unwrap()
        .remove("probing");
    assert!(
        post(&input)
            .unwrap_err()
            .contains("missing field `probing`")
    );
}

#[test]
fn the_decoder_reads_only_declared_words() {
    let machine = Machine::from_json(fixture()["machine"].clone()).unwrap();
    let mut decoder = Decoder::new(&machine, [0., 0., 8.], Default::default()).unwrap();
    let lines: Vec<String> = [
        "G21", "G90", "G91.1", "G17", "G94", "G97", "G40", "G49", "G80", "G92.1", "G61.1", "M5",
        "M9",
    ]
    .map(String::from)
    .into();
    assert!(
        decoder
            .read(&lines, None)
            .unwrap()
            .iter()
            .all(|e| !matches!(e, camlib::motion::Event::Motion { .. }))
    );
    decoder.validate_initialized().unwrap();
    let context = Context {
        id: 1,
        operation: "op".into(),
        kind: MoveKind::Cut,
    };
    for (line, message) in [
        ("G5 X1", "undeclared controller command"),
        ("G1 X1 Q2 F100", "unused/unsupported NC words"),
        ("G1 X1 X2 F100", "duplicate NC word"),
        (
            "G0 X1",
            "NC motion with unknown attribution/tool/length/offset",
        ),
    ] {
        let error = decoder
            .clone()
            .read(&[line.to_string()], Some(&context))
            .unwrap_err();
        assert!(error.contains(message), "{line}: {error}");
    }
}

#[test]
fn only_a_table_machine_needs_measured_tip_positions() {
    // #177: a tool table states the tip after each change only where the
    // machine is declared by it ("table"); a z_home machine derives each tip
    // from the gauge line's home Z and the tool's gauge length.
    let mut machine = fixture()["machine"].clone();
    for tool in machine["tools"].as_array_mut().unwrap() {
        tool["position_after_mm"] = Value::Null;
    }
    machine["tool_change_position_mode"] = "table".into();
    let error = Machine::from_json(machine.clone()).unwrap_err();
    assert!(
        error.contains("position_after_mm") && error.contains("z_home"),
        "{error}"
    );
    machine["tool_change_position_mode"] = "z_home".into();
    machine["tool_change_home_z_mm"] = 80.0.into();
    let machine = Machine::from_json(machine).unwrap();
    let gauge = machine.tools[0].gauge_length_mm;
    let number = machine.tools[0].number;
    let mut decoder = Decoder::new(&machine, [1., 2., 8.], Default::default()).unwrap();
    decoder
        .read(&[format!("T{number}"), "M6".to_string()], None)
        .unwrap();
    let z = 80. - machine.work_to_machine.origin_mm[2] - gauge;
    assert_eq!(decoder.position, [1., 2., z]);
}

#[test]
fn canned_cycles_are_refused_outside_g17() {
    // #183: in G18 LinuxCNC drills along Y; camlib's cycles drill along Z.
    let machine = Machine::from_json(fixture()["machine"].clone()).unwrap();
    let header = [
        "G21", "G90", "G91.1", "G17", "G94", "G97", "G40", "G49", "G80", "G92.1", "G61.1", "M5",
        "M9",
    ]
    .map(String::from);
    for plane in ["G18", "G19"] {
        let mut decoder = Decoder::new(&machine, [0., 0., 8.], Default::default()).unwrap();
        decoder.read(&header, None).unwrap();
        decoder.read(&[plane.to_string()], None).unwrap();
        let error = decoder
            .read(&["G81 X1 Y1 Z-1 R1 F100".to_string()], None)
            .unwrap_err();
        assert!(error.contains("G17"), "{plane}: {error}");
    }
}

#[test]
fn cam_post_command_reports_on_stdout() {
    use std::io::Write;
    use std::process::{Command, Stdio};
    let mut child = Command::new(env!("CARGO_BIN_EXE_cam-post"))
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    child
        .stdin
        .take()
        .unwrap()
        .write_all(INPUT.as_bytes())
        .unwrap();
    let out = child.wait_with_output().unwrap();
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let report: Value = serde_json::from_slice(&out.stdout).unwrap();
    assert_eq!(report["program"].as_str().unwrap(), PROGRAM);
    let failed = Command::new(env!("CARGO_BIN_EXE_cam-post"))
        .arg("--help")
        .output()
        .unwrap();
    assert_eq!(failed.status.code(), Some(1));
}

/// The fixture's two-tool job on a machine whose tool change leaves the
/// spindle where it stands (#168): `first`/`second` are the gauge lengths of
/// the first and second tool used.
fn in_place(first: f64, second: f64) -> Value {
    let mut input = fixture();
    input["machine"]["tool_change_position_mode"] = "preserve_spindle".into();
    input["manifest"]["tool_change"] = serde_json::json!({"in_place": true});
    for (tool, gauge) in [("mill4", first), ("mill2", second)] {
        for t in input["manifest"]["tools"].as_array_mut().unwrap() {
            if t["id"] == tool {
                t["gauge_length_mm"] = gauge.into();
                t["holders"][0]["z_max_mm"] = gauge.into();
            }
        }
        for t in input["machine"]["tools"].as_array_mut().unwrap() {
            if t["tool"] == tool {
                t["gauge_length_mm"] = gauge.into();
            }
        }
    }
    input
}

#[test]
fn a_tool_change_in_place_moves_the_tip_by_the_gauge_length_difference() {
    // 115 mm then 75 mm: the spindle stays, so the shorter tool's tip is
    // 40 mm higher. The program posts, decodes and replays with that pose.
    let input = in_place(115., 75.);
    let report = post(&input).unwrap();
    assert_eq!(report["released"], true, "{report}");
    assert_eq!(report["motions"], report["posting"]["checked_motions"]);
    let program = report["program"].as_str().unwrap();
    // The first tool stops at X6 Y6 Z8; the second comes down from Z48.
    let after: Vec<&str> = program
        .lines()
        .skip_while(|l| *l != "T8")
        .take(7)
        .collect();
    assert_eq!(
        after,
        [
            "T8",
            "M6",
            "G43 H10",
            "M3 S12000",
            "M8",
            "G0 X5 Y5.375 Z48",
            "G0 X5 Y5.375 Z6.5"
        ]
    );
    // The decoder places each tool from the machine's declaration alone.
    let poses = |mode: &str| {
        let mut machine = input["machine"].clone();
        machine["tool_change_position_mode"] = mode.into();
        let machine = Machine::from_json(machine).unwrap();
        let mut decoder = Decoder::new(&machine, [0., 0., 8.], Default::default()).unwrap();
        let mut poses = vec![];
        for line in program.lines() {
            let context = Context {
                id: 1,
                operation: "op".into(),
                kind: MoveKind::Rapid,
            };
            if line == "T8" {
                // Where the first tool stopped.
                assert_eq!(decoder.position, [6., 6., 8.]);
            }
            for e in decoder.read(&[line.to_string()], Some(&context)).unwrap() {
                if let camlib::motion::Event::ToolChange {
                    tool,
                    position_after_mm,
                    ..
                } = e
                {
                    poses.push((tool, position_after_mm));
                }
            }
            if poses.len() == 2 {
                break;
            }
        }
        poses
    };
    assert_eq!(
        poses("preserve_spindle"),
        [
            ("mill4".to_string(), [0., 0., 8.]),
            ("mill2".to_string(), [6., 6., 48.])
        ]
    );
    assert_eq!(poses("preserve_tip")[1].1, [6., 6., 8.]);
    // The same plan on a machine that keeps the tip is refused before posting.
    let mut other = input.clone();
    other["machine"]["tool_change_position_mode"] = "preserve_tip".into();
    let error = post(&other).unwrap_err();
    assert!(
        error.contains("ToolChangeInPlace") && error.contains("preserve_spindle"),
        "{error}"
    );
    // And a plan that keeps the tip is refused on this machine.
    let mut other = input.clone();
    other["manifest"]
        .as_object_mut()
        .unwrap()
        .remove("tool_change");
    let error = post(&other).unwrap_err();
    assert!(
        error.contains("plan with Setup(tool_change=ToolChangeInPlace())"),
        "{error}"
    );
}

#[test]
fn a_longer_tool_changed_in_place_into_the_stock_is_found() {
    // 75 mm then 115 mm: the first tool stops at Z8, 2 mm above the stock,
    // so the longer tool changed in there would reach 32 mm below Z0.
    let error = post(&in_place(75., 115.)).unwrap_err();
    assert!(
        error.contains("start Z -32 is below clearance_z_mm 8")
            && error.contains("leaves the spindle where it stands")
            && error.contains("40 mm below the Z 8 where mill4 stopped"),
        "{error}"
    );
}

#[test]
fn preserve_spindle_is_refused_where_it_is_not_modelled() {
    let mut machine = in_place(115., 75.)["machine"].clone();
    Machine::from_json(machine.clone()).unwrap();
    machine["work_to_machine"]["z"] = serde_json::json!([0, 1, 0]);
    machine["work_to_machine"]["y"] = serde_json::json!([0, 0, -1]);
    let error = Machine::from_json(machine).unwrap_err();
    assert!(error.contains("work Z axis along machine Z"), "{error}");
}
