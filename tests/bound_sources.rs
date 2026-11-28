//! Regression tests for #433: bounds with no source outside the motion checks.
//! Every test here fails on main before #433.

use camlib::{
    dynamics::{JunctionMode, Policy, Settings, schedule},
    gcode::{Machine, decode::Decoder},
    holes::{DrillingOperation, DrillingPlan},
    motion::{Curve, Event, Motion, MoveKind, Program, WorkFrame},
    planar::{Boundary, Component, PlanarRegion},
    pocket::Operation,
    stock::{Stock, StockSpec, SweepOptions},
    stream::StreamManifest,
    thermal::{ThermalCutOperation, ThermalCutPlan},
    tool::{Cutter, CutterShape, HolderSection, ToolAssembly},
};
use serde_json::{Value, json};
use std::{collections::BTreeSet, sync::atomic::AtomicBool};

const STANDALONE: &str = include_str!("../fixtures/posts/rust/standalone-rest.json");
const CARVERA_DRILL: &str = include_str!("../fixtures/posts/rust/carvera-drill.json");
const CARVERA_DRILL_NC: &str = include_str!("../fixtures/posts/rust/carvera-drill.nc");
const CARVERA_ARCS: &str = include_str!("../fixtures/posts/rust/carvera-arcs.json");
const CARVERA_ARCS_NC: &str = include_str!("../fixtures/posts/rust/carvera-arcs.nc");
const HAAS_PROGRAM: &str = include_str!("../fixtures/posts/rust/haas-program.json");

fn standalone_machine() -> Value {
    let mut v: Value = serde_json::from_str(STANDALONE).unwrap();
    v["machine"].take()
}

fn post_program(program: &Program, machine: &Value) -> Result<Value, String> {
    let payload = json!({
        "program": program,
        "machine": machine,
    });
    let out = camlib::gcode::post_program(&payload.to_string(), &AtomicBool::new(false))?;
    Ok(serde_json::from_str(&out).unwrap())
}

fn drill_fixture() -> StreamManifest {
    serde_json::from_str(include_str!("../fixtures/operations/holes.json")).unwrap()
}

fn drilling_op() -> DrillingOperation {
    match drill_fixture().operations.remove(0) {
        Operation::Drilling(p) => p,
        _ => panic!("expected drilling operation"),
    }
}

fn torch() -> ToolAssembly {
    ToolAssembly {
        id: "torch".into(),
        cutter: Cutter {
            id: "nozzle".into(),
            name: String::new(),
            shape: CutterShape::Beam,
            diameter_mm: 1.5,
            cutting_length_mm: 1.,
            overall_length_mm: 40.,
            shank_diameter_mm: 8.,
            flutes: 1,
            center_cutting: true,
            material: String::new(),
            neck: None,
        },
        stickout_mm: 30.,
        gauge_length_mm: 60.,
        holders: vec![HolderSection {
            z_min_mm: 30.,
            z_max_mm: 60.,
            radius_min_mm: 15.,
            radius_max_mm: 15.,
        }],
    }
}

fn simple_dwell_program(op_name: &str, seconds: f64, canned: bool) -> (Program, Value) {
    let mut machine = standalone_machine();
    let base: Value = serde_json::from_str(STANDALONE).unwrap();
    let tools: Vec<ToolAssembly> = serde_json::from_value(base["manifest"]["tools"].clone()).unwrap();
    let tool = tools[0].clone();
    let tool_num = machine["tools"][0]["number"].as_u64().unwrap() as u32;
    let len_reg = machine["tools"][0]["length_register"].as_u64().unwrap() as u32;
    let start = [0., 0., 12.];
    machine["tools"][0]["position_after_mm"] = json!(start);
    let mut events = vec![
        Event::WorkOffset { number: 1 },
        Event::ToolChange {
            tool: tool.id.clone(),
            number: tool_num,
            length_register: len_reg,
            position_after_mm: start,
        },
        Event::Spindle {
            rpm: 6000.,
            direction: camlib::motion::SpindleDirection::Cw,
        },
        Event::Coolant {
            mode: camlib::motion::Coolant::Flood,
        },
    ];
    if canned {
        events.push(Event::Cycle {
            cycle: camlib::motion::DrillCycle::Dwell,
            center_mm: [8., 8.],
            z_mm: 4.,
            r_mm: 11.,
            initial_z_mm: 12.,
            feed_mm_min: 150.,
            covers: 5,
            peck_mm: None,
            dwell_s: Some(seconds),
            reentry_mm: None,
            lift_mm: None,
            shift_mm: None,
            return_to_r: false,
        });
        events.push(Event::Motion {
            motion: Motion {
                id: 1,
                operation: op_name.into(),
                tool: tool.id.clone(),
                curve: Curve::Line {
                    start,
                    end: [8., 8., 12.],
                },
                feed_mm_min: None,
                kind: MoveKind::Rapid,
            },
        });
        events.push(Event::Motion {
            motion: Motion {
                id: 2,
                operation: op_name.into(),
                tool: tool.id.clone(),
                curve: Curve::Line {
                    start: [8., 8., 12.],
                    end: [8., 8., 11.],
                },
                feed_mm_min: None,
                kind: MoveKind::Rapid,
            },
        });
        events.push(Event::Motion {
            motion: Motion {
                id: 3,
                operation: op_name.into(),
                tool: tool.id.clone(),
                curve: Curve::Line {
                    start: [8., 8., 11.],
                    end: [8., 8., 4.],
                },
                feed_mm_min: Some(150.),
                kind: MoveKind::Plunge,
            },
        });
        events.push(Event::Dwell { seconds });
        events.push(Event::Motion {
            motion: Motion {
                id: 4,
                operation: op_name.into(),
                tool: tool.id.clone(),
                curve: Curve::Line {
                    start: [8., 8., 4.],
                    end: [8., 8., 12.],
                },
                feed_mm_min: None,
                kind: MoveKind::Rapid,
            },
        });
    } else {
        events.push(Event::Motion {
            motion: Motion {
                id: 1,
                operation: op_name.into(),
                tool: tool.id.clone(),
                curve: Curve::Line {
                    start,
                    end: [8., 8., 12.],
                },
                feed_mm_min: None,
                kind: MoveKind::Rapid,
            },
        });
        events.push(Event::Motion {
            motion: Motion {
                id: 2,
                operation: op_name.into(),
                tool: tool.id.clone(),
                curve: Curve::Line {
                    start: [8., 8., 12.],
                    end: [8., 8., 4.],
                },
                feed_mm_min: Some(150.),
                kind: MoveKind::Plunge,
            },
        });
        events.push(Event::Dwell { seconds });
        events.push(Event::Motion {
            motion: Motion {
                id: 3,
                operation: op_name.into(),
                tool: tool.id.clone(),
                curve: Curve::Line {
                    start: [8., 8., 4.],
                    end: [8., 8., 12.],
                },
                feed_mm_min: None,
                kind: MoveKind::Rapid,
            },
        });
    }
    events.push(Event::Spindle {
        rpm: 0.,
        direction: camlib::motion::SpindleDirection::Cw,
    });
    events.push(Event::Coolant {
        mode: camlib::motion::Coolant::Off,
    });
    let program = Program {
        version: 1,
        units: "mm".into(),
        id: "dwell-test".into(),
        revision: "1".into(),
        frame: WorkFrame::identity(),
        tools: vec![tool],
        initial_position_mm: start,
        tolerance_mm: 1e-5,
        events,
    };
    (program, machine)
}

fn plasma_pierce_program(pierce_delay_s: f64) -> (Program, Value) {
    let beam = torch();
    let op_json = json!({
        "id": "plasma",
        "tool": "torch",
        "parts": PlanarRegion {
            version: 1,
            units: "mm".into(),
            name: "plate".into(),
            boundary_error_mm: 0.,
            components: vec![Component {
                outer: Boundary {
                    vertices: vec![
                        [20., 20., 0.],
                        [70., 20., 0.],
                        [70., 60., 0.],
                        [20., 60., 0.],
                    ],
                },
                holes: vec![],
            }],
            source: Value::Null,
        },
        "feed_mm_min": 2500.,
        "cut_height_mm": 1.5,
        "height_mode": "program",
        "pierce_height_mm": 3.8,
        "travel_height_mm": 10.,
        "plunge_mm_min": 1000.,
        "pierce_delay_s": pierce_delay_s,
        "power_percent": 100.,
        "lead_in": { "length_mm": 3., "radius_mm": 2. },
        "lead_out": { "length_mm": 1., "radius_mm": 1. },
        "tolerance_mm": 1e-5,
        "work_offset": 1,
        "tool_number": 1,
        "length_register": 1,
    });
    let op: ThermalCutOperation = serde_json::from_value(op_json).unwrap();
    let stock_spec = StockSpec {
        version: 1,
        units: "mm".into(),
        min_mm: [0., 0., 0.],
        max_mm: [100., 80., 6.],
        cell_size_mm: 0.25,
        max_cells: 1_000_000,
    };
    let cancel = AtomicBool::new(false);
    let prepared = beam.prepare().unwrap();
    let mut stock = Stock::new(stock_spec.clone()).unwrap();
    let mut plan =
        ThermalCutPlan::prepare(op, &beam, &stock_spec, [0., 0., 20.], 1, &cancel).unwrap();
    let mut events = plan.prefix_events();
    events.extend(plan.take_events());
    while let Some(m) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        let d = stock
            .preview_sweep(&prepared, &m, 1., SweepOptions::default(), &cancel)
            .unwrap();
        stock.apply_delta(&d).unwrap();
        events.push(Event::Motion { motion: m });
        events.extend(plan.take_events());
    }
    events.extend(plan.suffix_events());
    let program = Program {
        version: 1,
        units: "mm".into(),
        id: "plasma-test".into(),
        revision: "1".into(),
        frame: WorkFrame::identity(),
        tools: vec![beam.clone()],
        initial_position_mm: [0., 0., 20.],
        tolerance_mm: 1e-5,
        events,
    };
    let mut machine = standalone_machine();
    machine["tools"] = json!([{
        "tool": "torch",
        "number": 1,
        "length_register": 1,
        "gauge_length_mm": beam.gauge_length_mm,
        "position_after_mm": [0., 0., 20.],
        "diameter_register": null,
    }]);
    machine["dialect"]["beam_s_max"] = json!(1.);
    (program, machine)
}

#[test]
fn dialect_without_dwell_max_refuses_g4_g82_and_pierce_delay_by_name() {
    // 1. G4 drilling dwell: omitted dwell_max_s refuses naming op, value, field.
    let (g4_prog, mut machine) = simple_dwell_program("spot", 0.5, false);
    machine["dialect"].as_object_mut().unwrap().remove("dwell_max_s");
    let err = post_program(&g4_prog, &machine).unwrap_err();
    assert!(
        err.contains("spot:")
            && err.contains("0.5 s")
            && err.contains("no dwell range")
            && err.contains("GCodeDialect.dwell_max_s"),
        "{err}"
    );
    // Declaring dwell_max_s = 1e5 posts cleanly.
    machine["dialect"]["dwell_max_s"] = json!(1e5);
    let posted_g4 = post_program(&g4_prog, &machine).unwrap();
    assert!(posted_g4["program"].as_str().unwrap().contains("G4 P0.5\n"));

    // G82 canned dwell cycle: omitted dwell_max_s refuses naming op, value, field.
    let (g82_prog, mut machine) = simple_dwell_program("holes", 0.5, true);
    machine["dialect"].as_object_mut().unwrap().remove("dwell_max_s");
    let err = post_program(&g82_prog, &machine).unwrap_err();
    assert!(
        err.contains("holes:")
            && err.contains("0.5 s")
            && err.contains("no dwell range")
            && err.contains("GCodeDialect.dwell_max_s"),
        "{err}"
    );
    machine["dialect"]["dwell_max_s"] = json!(1e5);
    let posted_g82 = post_program(&g82_prog, &machine).unwrap();
    assert!(posted_g82["program"].as_str().unwrap().contains("G82 X8 Y8 Z4 R11 P0.5 F150\n"));

    // Plasma pierce delay: omitted dwell_max_s refuses naming op, value, field.
    let (plasma_prog, mut plasma_machine) = plasma_pierce_program(0.5);
    plasma_machine["dialect"]
        .as_object_mut()
        .unwrap()
        .remove("dwell_max_s");
    let err = post_program(&plasma_prog, &plasma_machine).unwrap_err();
    assert!(
        err.contains("plasma:")
            && err.contains("0.5 s")
            && err.contains("no dwell range")
            && err.contains("GCodeDialect.dwell_max_s"),
        "{err}"
    );
    plasma_machine["dialect"]["dwell_max_s"] = json!(1e5);
    let posted_plasma = post_program(&plasma_prog, &plasma_machine).unwrap();
    assert!(posted_plasma["program"].as_str().unwrap().contains("G4 P0.5\n"));

    // Independent decoder also refuses G4 and G82 when dwell_max_s is absent.
    let mut m_no_dwell = standalone_machine();
    m_no_dwell["dialect"]
        .as_object_mut()
        .unwrap()
        .remove("dwell_max_s");
    let m_parsed = Machine::from_json(m_no_dwell).unwrap();
    assert_eq!(m_parsed.dialect.dwell_max_s, None);
    let mut dec = Decoder::new(&m_parsed, [0., 0., 12.], BTreeSet::new()).unwrap();
    for line in [
        "G21", "G90", "G91.1", "G17", "G94", "G97", "G40", "G49", "G80", "G92.1", "G61.1", "M5",
        "M9", "G54",
    ] {
        dec.read_line(line, None).unwrap();
    }
    let err = dec.read_line("G4 P0.5", None).unwrap_err();
    assert!(err.contains("GCodeDialect.dwell_max_s"), "{err}");
}

#[test]
fn declared_dwell_max_refuses_longer_dwell_by_name() {
    let (g4_prog, mut machine) = simple_dwell_program("spot", 10.5, false);
    machine["dialect"]["dwell_max_s"] = json!(10.0);
    let err = post_program(&g4_prog, &machine).unwrap_err();
    assert!(
        err.contains("spot:")
            && err.contains("10.5 s")
            && err.contains("10 s")
            && err.contains("GCodeDialect.dwell_max_s"),
        "{err}"
    );

    let (g82_prog, mut machine) = simple_dwell_program("holes", 10.5, true);
    machine["dialect"]["dwell_max_s"] = json!(10.0);
    let err = post_program(&g82_prog, &machine).unwrap_err();
    assert!(
        err.contains("holes:")
            && err.contains("10.5 s")
            && err.contains("10 s")
            && err.contains("GCodeDialect.dwell_max_s"),
        "{err}"
    );

    // Decoder also refuses a dwell above declared dwell_max_s naming the field.
    let m_parsed = Machine::from_json(machine).unwrap();
    let mut dec = Decoder::new(&m_parsed, [0., 0., 12.], BTreeSet::new()).unwrap();
    for line in [
        "G21", "G90", "G91.1", "G17", "G94", "G97", "G40", "G49", "G80", "G92.1", "G61.1", "M5",
        "M9", "G54",
    ] {
        dec.read_line(line, None).unwrap();
    }
    let err = dec.read_line("G4 P10.5", None).unwrap_err();
    assert!(
        err.contains("10.5 s") && err.contains("10 s") && err.contains("GCodeDialect.dwell_max_s"),
        "{err}"
    );
}

#[test]
fn centroid_dwell_range_327_67_still_applies() {
    let (ok_prog, mut machine) = simple_dwell_program("spot", 327.67, false);
    machine["dialect"]["dwell_resolution_s"] = json!(0.01);
    machine["dialect"]["dwell_max_s"] = json!(327.67);
    let out = post_program(&ok_prog, &machine).unwrap();
    assert!(out["program"].as_str().unwrap().contains("G4 P327.67\n"));

    let (too_long, _) = simple_dwell_program("spot", 327.68, false);
    let err = post_program(&too_long, &machine).unwrap_err();
    assert!(
        err.contains("spot:")
            && err.contains("327.68 s")
            && err.contains("327.67 s")
            && err.contains("GCodeDialect.dwell_max_s"),
        "{err}"
    );
}

#[test]
fn probe_fast_feed_is_held_to_machine_max_feed() {
    let mut m = standalone_machine();
    m["max_feed_mm_min"] = json!(5000.);
    m["dialect"]["probing"] = json!("inspection_plus");
    m["dialect"]["probe_fast_feed_mm_min"] = json!(6000.);
    let err = Machine::from_json(m.clone()).unwrap_err();
    assert!(
        err.contains("6000")
            && err.contains("5000")
            && err.contains("GCodeDialect.probe_fast_feed_mm_min")
            && err.contains("Machine.max_feed_mm_min"),
        "{err}"
    );

    // Equal to Machine.max_feed_mm_min is accepted.
    m["dialect"]["probe_fast_feed_mm_min"] = json!(5000.);
    Machine::from_json(m.clone()).unwrap();

    // Above the old 1e5 cap, within Machine.max_feed_mm_min, is accepted.
    m["max_feed_mm_min"] = json!(250_000.);
    m["dialect"]["probe_fast_feed_mm_min"] = json!(200_000.);
    Machine::from_json(m).unwrap();
}

#[test]
fn hole_dwell_and_thermal_delays_have_no_invented_cap_and_post_checks_dwell_max() {
    // Hole dwell_s = 2e5 plans (no longer refused by 1e5 cap).
    let mut op = drilling_op();
    op.dwell_s = Some(2e5);
    let mf = drill_fixture();
    DrillingPlan::prepare(
        op,
        &mf.tools[0],
        mf.stock.height().unwrap(),
        mf.initial_position_mm,
        1,
        &AtomicBool::new(false),
    )
    .unwrap();

    // Posting holds 2e5 s to dwell_max_s.
    let (long_drill, mut machine) = simple_dwell_program("spot", 2e5, false);
    machine["dialect"]["dwell_max_s"] = json!(1e5);
    let err = post_program(&long_drill, &machine).unwrap_err();
    assert!(
        err.contains("spot:")
            && err.contains("200000 s")
            && err.contains("100000 s")
            && err.contains("GCodeDialect.dwell_max_s"),
        "{err}"
    );
    machine["dialect"]["dwell_max_s"] = json!(3e5);
    let posted = post_program(&long_drill, &machine).unwrap();
    assert!(posted["program"].as_str().unwrap().contains("G4 P200000.\n"));

    // Thermal pierce_delay_s = 2000 plans (no longer refused by 1000 cap).
    let (plasma_prog, mut plasma_machine) = plasma_pierce_program(2000.);
    plasma_machine["dialect"]["dwell_max_s"] = json!(1000.);
    let err = post_program(&plasma_prog, &plasma_machine).unwrap_err();
    assert!(
        err.contains("plasma:")
            && err.contains("2000 s")
            && err.contains("1000 s")
            && err.contains("GCodeDialect.dwell_max_s"),
        "{err}"
    );
    plasma_machine["dialect"]["dwell_max_s"] = json!(3000.);
    let posted = post_program(&plasma_prog, &plasma_machine).unwrap();
    assert!(posted["program"].as_str().unwrap().contains("G4 P2000.\n"));
}

#[test]
fn timing_settings_beyond_old_caps_accepted_and_invalid_values_name_field() {
    let base: Value = serde_json::from_str(STANDALONE).unwrap();
    let tools: Vec<ToolAssembly> = serde_json::from_value(base["manifest"]["tools"].clone()).unwrap();
    let tool = tools[0].clone();
    let p = Program {
        version: 1,
        units: "mm".into(),
        id: "timing".into(),
        revision: "1".into(),
        frame: WorkFrame::identity(),
        tools: vec![tool.clone()],
        initial_position_mm: [0., 0., 5.],
        tolerance_mm: 1e-5,
        events: vec![
            Event::WorkOffset { number: 1 },
            Event::ToolChange {
                tool: tool.id.clone(),
                number: 1,
                length_register: 1,
                position_after_mm: [0., 0., 5.],
            },
            Event::Spindle {
                rpm: 1000.,
                direction: camlib::motion::SpindleDirection::Cw,
            },
            Event::Motion {
                motion: Motion {
                    id: 1,
                    operation: "cut".into(),
                    tool: tool.id,
                    curve: Curve::Line {
                        start: [0., 0., 5.],
                        end: [20., 0., 5.],
                    },
                    feed_mm_min: Some(1200.),
                    kind: MoveKind::Cut,
                },
            },
        ],
    };

    let high = Settings {
        policy: Policy {
            tangential_accel_mm_s2: 2e9,
            rapid_feed_mm_min: 2e7,
            junction_mode: JunctionMode::ExactStop,
            feed_reserve_fraction: 0.,
            per_block_s: 25.,
        },
        max_feed_mm_min: 2e7,
        max_normal_accel_mm_s2: Some(2e9),
        max_axis_feed_mm_min: Some([2e7, 2e7, 2e7]),
        work_to_machine: WorkFrame::identity(),
        feed_quantum_mm_min: 2e3,
        adjust_feeds: true,
    };
    let report = schedule(&p, high.clone(), &|| false).unwrap();
    assert!(report.motion_seconds > 25.);

    // Zero, negative, NaN, and infinite values are refused naming field and value.
    for bad in [0., -1., f64::NAN, f64::INFINITY] {
        let mut s = high.clone();
        s.policy.tangential_accel_mm_s2 = bad;
        let err = s.validate().unwrap_err();
        assert!(err.contains("tangential_accel_mm_s2") && err.contains(&bad.to_string()), "{err}");

        let mut s = high.clone();
        s.policy.rapid_feed_mm_min = bad;
        let err = s.validate().unwrap_err();
        assert!(err.contains("rapid_feed_mm_min") && err.contains(&bad.to_string()), "{err}");

        let mut s = high.clone();
        s.max_feed_mm_min = bad;
        let err = s.validate().unwrap_err();
        assert!(err.contains("max_feed_mm_min") && err.contains(&bad.to_string()), "{err}");

        let mut s = high.clone();
        s.feed_quantum_mm_min = bad;
        let err = s.validate().unwrap_err();
        assert!(err.contains("feed_quantum_mm_min") && err.contains(&bad.to_string()), "{err}");

        let mut s = high.clone();
        s.max_normal_accel_mm_s2 = Some(bad);
        let err = s.validate().unwrap_err();
        assert!(err.contains("max_normal_accel_mm_s2") && err.contains(&bad.to_string()), "{err}");

        let mut s = high.clone();
        s.max_axis_feed_mm_min = Some([1000., bad, 1000.]);
        let err = s.validate().unwrap_err();
        assert!(err.contains("max_axis_feed_mm_min[1]") && err.contains(&bad.to_string()), "{err}");
    }
    for bad in [-1., f64::NAN, f64::INFINITY] {
        let mut s = high.clone();
        s.policy.per_block_s = bad;
        let err = s.validate().unwrap_err();
        assert!(err.contains("per_block_s") && err.contains(&bad.to_string()), "{err}");
    }
}

#[test]
fn haas_settings_22_and_52_accept_values_above_100_and_refuse_invalid() {
    let mut input: Value = serde_json::from_str(HAAS_PROGRAM).unwrap();
    input["machine"]["controller"]["setting_22_mm"] = json!(150.);
    input["machine"]["controller"]["setting_52_mm"] = json!(150.);
    input["machine"]["dialect"]["peck_reentry_mm"] = json!(150.);
    input["machine"]["dialect"]["chip_break_lift_mm"] = json!(150.);
    input["machine"]["dialect"]["commands"]
        .as_array_mut()
        .unwrap()
        .push(json!({"effect": "chip_break_cycle", "code": "G73"}));
    Machine::from_json(input["machine"].clone()).unwrap();

    for (field, bad) in [
        ("setting_22_mm", json!(-1.)),
        ("setting_22_mm", json!(0.)),
        ("setting_52_mm", json!(-1.)),
    ] {
        let mut broken: Value = serde_json::from_str(HAAS_PROGRAM).unwrap();
        broken["machine"]["controller"][field] = bad;
        let err = Machine::from_json(broken["machine"].clone()).unwrap_err();
        assert!(err.contains(field), "{field}: {err}");
    }
}

#[test]
fn carvera_identity_check_agrees_with_documented_dialect_and_posts_byte_identically() {
    // Carvera arcs fixture with dwell_max_s omitted passes identity check and
    // posts byte-identically to the golden NC.
    let mut arcs: Value = serde_json::from_str(CARVERA_ARCS).unwrap();
    arcs["machine"]["dialect"]
        .as_object_mut()
        .unwrap()
        .remove("dwell_max_s");
    let out: Value = serde_json::from_str(
        &camlib::post_job::run(&arcs.to_string(), &AtomicBool::new(false)).unwrap(),
    )
    .unwrap();
    assert_eq!(out["released"], true);
    assert_eq!(out["program"].as_str().unwrap(), CARVERA_ARCS_NC);

    // Carvera drill fixture without dwell_max_s refuses the 0.5 s dwell by name;
    // with dwell_max_s = 1e5 it posts byte-identically to carvera-drill.nc.
    let mut drill: Value = serde_json::from_str(CARVERA_DRILL).unwrap();
    drill["machine"]["dialect"]
        .as_object_mut()
        .unwrap()
        .remove("dwell_max_s");
    let err = camlib::post_job::run(&drill.to_string(), &AtomicBool::new(false)).unwrap_err();
    assert!(
        err.contains("0.5 s") && err.contains("GCodeDialect.dwell_max_s"),
        "{err}"
    );
    drill["machine"]["dialect"]["dwell_max_s"] = json!(1e5);
    let out: Value = serde_json::from_str(
        &camlib::post_job::run(&drill.to_string(), &AtomicBool::new(false)).unwrap(),
    )
    .unwrap();
    assert_eq!(out["released"], true);
    assert_eq!(out["program"].as_str().unwrap(), CARVERA_DRILL_NC);
}
