//! Python-authored Carvera bytes, independently decoded and replayed.
use camlib::gcode::{Machine, decode::Decoder};
use serde_json::{Value, json};
use std::{collections::BTreeSet, sync::atomic::AtomicBool};
const DRILL: &str = include_str!("../fixtures/posts/rust/carvera-drill.json");
const ARCS: &str = include_str!("../fixtures/posts/rust/carvera-arcs.json");
fn report(source: &str) -> Value {
    serde_json::from_str(&camlib::post_job::run(source, &AtomicBool::new(false)).unwrap()).unwrap()
}
#[test]
fn python_programs_are_byte_identical_and_checked() {
    for (source, nc) in [
        (
            DRILL,
            include_str!("../fixtures/posts/rust/carvera-drill.nc"),
        ),
        (ARCS, include_str!("../fixtures/posts/rust/carvera-arcs.nc")),
    ] {
        let r = report(source);
        assert_eq!(r["released"], true);
        assert_eq!(r["program"], nc);
        assert_eq!(r["motions"], r["posting"]["checked_motions"]);
        assert_eq!(
            r["posting"]["definition"]["machine"]["controller"]["name"],
            "carvera"
        );
    }
    let r = report(ARCS);
    assert!(r["posting"]["arc_output"]["native"].as_u64().unwrap() > 0);
    for k in ["lines_policy", "lines_machine", "lines_precision"] {
        assert_eq!(r["posting"]["arc_output"][k], 0);
    }
}
#[test]
fn stock_replay_applies_the_export_gate() {
    let mut source: Value = serde_json::from_str(DRILL).unwrap();
    source["accept"] = json!([]);
    let r = report(&source.to_string());
    assert_eq!(r["released"], false);
    assert!(r["program"].is_null());
    assert_eq!(r["refused"]["statuses"], json!(["unchecked_machine"]));
}
fn decoder() -> Decoder {
    let input: Value = serde_json::from_str(DRILL).unwrap();
    let m: Machine = serde_json::from_value(input["machine"].clone()).unwrap();
    let mut d = Decoder::new(&m, [0., 0., 12.], BTreeSet::new()).unwrap();
    for l in ["G21", "G90", "G17", "G94", "G54", "M5", "M400", "M9"] {
        d.read_line(l, None).unwrap();
    }
    d.validate_initialized().unwrap();
    d
}
#[test]
fn independent_decoder_rejects_unsupported_words_and_end_state() {
    for line in [
        "G20",
        "G43 H1",
        "G49",
        "G61",
        "G64",
        "G81 X1 Y1 Z0 R2 F100",
        "M8",
        "M4 S1000",
        "M0",
        "M1",
        "M6",
        "T1 M6 C1",
        "T1",
        "H1",
        "t1",
        "h1",
        "m06",
        "M30",
        "G53 Z-3",
        "G53 G0 Z-2",
    ] {
        assert!(decoder().read_line(line, None).is_err(), "{line}");
    }
    let mut d = decoder();
    d.read_line("G53 G0 Z-3", None).unwrap();
    d.read_line("M30", None).unwrap();
    assert!(d.ended);
    assert!(d.read_line("G21", None).is_err());
}
#[test]
fn refuses_inches_unknown_options_and_unplanned_retracts() {
    for (path, value, message) in [
        (
            vec!["machine", "output_units"],
            json!("inch"),
            "requires mm",
        ),
        (
            vec!["machine", "controller", "unknown"],
            json!(true),
            "unknown field",
        ),
        (
            vec!["machine", "controller", "automatic_tool_length"],
            json!(false),
            "automatic M6",
        ),
        (
            vec!["machine", "controller", "initial_gauge_length_mm"],
            json!(0),
            "plan the retract",
        ),
        (vec!["machine", "flood"], json!(true), "no flood"),
    ] {
        let mut source: Value = serde_json::from_str(DRILL).unwrap();
        let mut target = &mut source;
        for key in path {
            target = &mut target[key];
        }
        *target = value;
        let error =
            camlib::post_job::run(&source.to_string(), &AtomicBool::new(false)).unwrap_err();
        assert!(error.contains(message), "{error}");
    }
    for effect in [
        "units_inch",
        "length_on",
        "drill_cycle",
        "exact_stop",
        "spindle_ccw",
        "flood",
    ] {
        let mut source: Value = serde_json::from_str(DRILL).unwrap();
        source["machine"]["dialect"]["commands"]
            .as_array_mut()
            .unwrap()
            .push(json!({"effect":effect,"code":"G999"}));
        assert!(
            camlib::post_job::run(&source.to_string(), &AtomicBool::new(false))
                .unwrap_err()
                .contains("command mapping")
        );
    }
}

#[test]
fn explicit_program_matches_python_and_refuses_an_unplanned_final_retract() {
    let source = include_str!("../fixtures/posts/rust/carvera-program.json");
    let r: Value = serde_json::from_str(
        &camlib::gcode::post_program(source, &AtomicBool::new(false)).unwrap(),
    )
    .unwrap();
    assert_eq!(
        r["program"],
        include_str!("../fixtures/posts/rust/carvera-program.nc")
    );
    let mut input: Value = serde_json::from_str(source).unwrap();
    input["program"]["events"][4]["motion"]["curve"]["end"][2] = json!(11);
    assert!(
        camlib::gcode::post_program(&input.to_string(), &AtomicBool::new(false))
            .unwrap_err()
            .contains("plan the retract")
    );
}

#[test]
fn reference_limits_follow_makera_stock_six_tool_half_circle_subset() {
    let input: Value = serde_json::from_str(DRILL).unwrap();
    let m: Machine = serde_json::from_value(input["machine"].clone()).unwrap();
    for (field, value) in [
        ("max_sweep_rad", json!(std::f64::consts::PI + 0.01)),
        ("max_tool_number", json!(7)),
        ("max_length_register", json!(7)),
    ] {
        let mut data = serde_json::to_value(&m).unwrap();
        data[field] = value;
        assert!(Machine::from_json(data).is_err(), "{field}");
    }
}
#[test]
fn air_requires_an_immediately_preceding_m400() {
    let input: Value = serde_json::from_str(DRILL).unwrap();
    let m: Machine = serde_json::from_value(input["machine"].clone()).unwrap();
    let fresh = || Decoder::new(&m, [0., 0., 12.], BTreeSet::new()).unwrap();
    let mut d = fresh();
    assert!(d.read_line("M9", None).is_err());
    d.read_line("M400", None).unwrap();
    d.read_line("M9", None).unwrap();
    for line in ["M5", "G17", "", "M400", "M30"] {
        let mut d = fresh();
        d.read_line("M400", None).unwrap();
        assert!(d.read_line(line, None).is_err(), "{line}");
    }
    for line in ["M7", "M07", "M9", "M09", "M400 P1"] {
        assert!(fresh().read_line(line, None).is_err(), "{line}");
    }
}
#[test]
fn retract_comparison_uses_posted_resolution() {
    let input: Value = serde_json::from_str(DRILL).unwrap();
    let mut m: Machine = serde_json::from_value(input["machine"].clone()).unwrap();
    m.work_to_machine.origin_mm[2] = -88.4;
    assert_ne!(25.4 - 88.4 + 60., -3.);
    for (delta, accepted) in [
        (0., true),
        (0.00049, true),
        (0.00051, false),
        (-0.00051, false),
    ] {
        let mut d = Decoder::new(&m, [0., 0., 25.4 + delta], BTreeSet::new()).unwrap();
        let mut post = camlib::gcode::post::DefaultPost::new(m.clone());
        for line in post
            .call(camlib::gcode::post::Hook::Begin { tolerance_mm: 1e-6 }, 32)
            .unwrap()
        {
            d.read_line(&line, None).unwrap();
        }
        assert_eq!(d.read_line("G53 G0 Z-3", None).is_ok(), accepted);
    }
}
#[test]
fn plane_modes_and_planner_documentation() {
    use camlib::gcode::post::{DefaultPost, Hook};
    let input: Value =
        serde_json::from_str(include_str!("../fixtures/posts/rust/carvera-program.json")).unwrap();
    let m: Machine = serde_json::from_value(input["machine"].clone()).unwrap();
    let p: camlib::motion::Program = serde_json::from_value(input["program"].clone()).unwrap();
    let mut post = DefaultPost::new(m);
    post.position_mm = Some(p.initial_position_mm);
    let mut lines = post
        .call(
            Hook::Begin {
                tolerance_mm: p.tolerance_mm,
            },
            32,
        )
        .unwrap();
    for event in &p.events {
        lines.extend(post.call(Hook::Event(event), 8).unwrap());
        if let camlib::motion::Event::Motion { motion } = event {
            post.position_mm = Some(motion.curve.end());
        }
    }
    assert_eq!(lines.iter().filter(|l| l.as_str() == "G17").count(), 1);
}
#[test]
fn planner_premise_is_geometric_not_zero_corner_speed() {
    let doc = include_str!("../docs/carvera-post.md");
    assert!(!doc.contains("minimum planner speed configured to zero"));
    assert!(doc.contains("does not actually deviate"));
}

#[test]
fn mutated_tool_word_is_caught_by_decode_and_compare() {
    use camlib::{
        gcode::post::{DefaultPost, Hook},
        motion::Program,
        post_intent::{IntentChecker, Settings},
    };
    let mut input: Value =
        serde_json::from_str(include_str!("../fixtures/posts/rust/carvera-program.json")).unwrap();
    let mut other = input["machine"]["tools"][0].clone();
    other["tool"] = json!("other");
    other["number"] = json!(2);
    other["length_register"] = json!(2);
    input["machine"]["tools"]
        .as_array_mut()
        .unwrap()
        .push(other);
    let mut other = input["program"]["tools"][0].clone();
    other["id"] = json!("other");
    input["program"]["tools"]
        .as_array_mut()
        .unwrap()
        .push(other);
    let m: Machine = serde_json::from_value(input["machine"].clone()).unwrap();
    let mut header: Program = serde_json::from_value(input["program"].clone()).unwrap();
    let events = std::mem::take(&mut header.events);
    for mutate in [false, true] {
        let mut post = DefaultPost::new(m.clone());
        let lines = post
            .call(
                Hook::Begin {
                    tolerance_mm: header.tolerance_mm,
                },
                32,
            )
            .unwrap();
        let mut d = Decoder::new(&m, header.initial_position_mm, BTreeSet::new()).unwrap();
        d.read(&lines, None).unwrap();
        d.validate_initialized().unwrap();
        let settings = Settings {
            capabilities: m.capabilities(),
            tolerance_mm: m.tolerance_mm,
            feed_tolerance_mm_min: m.feed_tolerance_mm_min,
            traverse_feed_mm_min: m.traverse_feed_mm_min,
            max_normal_accel_mm_s2: m.max_normal_accel_mm_s2,
            max_axis_feed_mm_min: m.max_axis_feed_mm_min,
            work_to_machine: m.work_to_machine.clone(),
            tip_travel_mm: m.tip_travel_mm,
            gauge_travel_mm: m.gauge_travel_mm,
            definition: json!({"mutation_test":true}),
            run_id: "mutation".into(),
            decoded_initial_position_mm: d.position,
            lowering: None,
            motion_scheduling: None,
        };
        let (mut checker, _) = IntentChecker::begin(&header, settings, &lines).unwrap();
        for event in &events[..2] {
            let mut blocks = post.call(Hook::Event(event), 8).unwrap();
            if mutate {
                for block in &mut blocks {
                    if block == "T1 M6" {
                        *block = "T2 M6".into();
                    }
                }
            }
            let decoded = d.read(&blocks, None).unwrap();
            let result = checker.consume(
                std::slice::from_ref(event),
                &[decoded],
                &[blocks],
                &AtomicBool::new(false),
            );
            if mutate && matches!(event, camlib::motion::Event::ToolChange { .. }) {
                assert_eq!(d.active.as_ref().unwrap().number, 2);
                assert!(
                    result.is_err(),
                    "wrong encoder T word must fail independent intent comparison"
                );
            } else {
                assert!(result.is_ok());
            }
        }
    }
}
