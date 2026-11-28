//! The lift out of a pocket beside its wall (#430).
//!
//! A pocket ends on its wall loop, with the cutter against the wall. The
//! machine check boxes an independent-axis rapid against the stock's
//! whole-cell bounds, and the cells along the wall keep the wall's height.
//! A rapid lift from there could not be decided, so the program could not be
//! released. Such a lift now stays at feed.
use camlib::{
    machine::*,
    motion::*,
    operation::OperationPlan,
    stock::*,
    stream::StreamManifest,
};
use serde_json::{Value, json};
use std::sync::atomic::AtomicBool;

/// The issue's block, 104 x 64 x 21 mm with Z0 on top, at 0.5 mm cells. It
/// is faced to Z-1, then `second` runs, with one flat end mill of
/// `diameter` for both.
fn manifest(diameter: f64, second: Value) -> StreamManifest {
    let tool = json!({
        "id": "em",
        "cutter": {"id": "em", "name": "em", "shape": {"kind": "flat"},
            "diameter_mm": diameter, "cutting_length_mm": 26, "overall_length_mm": 75,
            "shank_diameter_mm": diameter, "flutes": 3, "center_cutting": true,
            "material": "carbide", "neck": null},
        "stickout_mm": 40, "gauge_length_mm": 90,
        "holders": [{"z_min_mm": 40, "z_max_mm": 90, "radius_min_mm": 12, "radius_max_mm": 12}]
    });
    serde_json::from_value(json!({
        "version": 1,
        "run": {"id": "wall-loop", "revision": "1"},
        "stock": {"version": 1, "units": "mm", "min_mm": [0, 0, -21], "max_mm": [104, 64, 0],
            "cell_size_mm": 0.5, "max_cells": 1048576},
        "tools": [tool],
        "initial_position_mm": [0, 0, 10],
        "sweep": {"tolerance_mm": 1e-5, "chord_error_mm": 0.002, "max_tip_step_mm": 0.1,
            "max_segments": 1000000, "max_scratch_cells": 4194304,
            "max_changed_cells": 4194304, "max_cell_visits": 1000000000},
        "rapid_retracts": true,
        "operations": [facing(diameter), second]
    }))
    .unwrap()
}
/// Face the block to Z-1 in rows half the cutter apart.
fn facing(diameter: f64) -> Value {
    json!({"kind": "facing", "id": "face", "tool": "em", "floor_z_mm": -1,
        "stepdown_mm": 1, "stepover_mm": diameter / 2., "clearance_z_mm": 10,
        "feed_mm_min": 1000, "plunge_mm_min": 300, "axis": "x", "mode": "zigzag",
        "reverse": false, "entry_air_gap_mm": 1, "edge_overlap_mm": null,
        "tolerance_mm": 1e-5, "spindle_rpm": 10000, "work_offset": 1,
        "tool_number": 1, "length_register": 1, "coolant": "flood"})
}
/// The 50 x 30 mm pocket X27..77 Y17..47, 9 mm deep in one level. Its
/// outline starts at corner `first` of (27,17), (77,17), (77,47), (27,47):
/// a CadQuery face lists (27,47) first.
fn pocket(kind: &str, stepover: f64, first: usize) -> Value {
    let mut corners = vec![json!([27, 17, 0]), json!([77, 17, 0]), json!([77, 47, 0]), json!([27, 47, 0])];
    corners.rotate_left(first);
    let mut p = json!({
        "kind": kind, "id": "pocket", "tool": "em",
        "region": {"version": 1, "units": "mm", "name": "pocket", "boundary_error_mm": 0,
            "components": [{"outer": {"vertices": corners}, "holes": []}]},
        "top_z_mm": 0, "floor_z_mm": -9, "stepdown_mm": 12, "stepover_mm": stepover,
        "clearance_z_mm": 10, "feed_mm_min": 1000, "plunge_mm_min": 300,
        "radial_allowance_mm": 0, "bottom_allowance_mm": 0, "tolerance_mm": 1e-5,
        "spindle_rpm": 10000, "work_offset": 1, "tool_number": 1, "length_register": 1,
        "coolant": "flood"
    });
    if kind == "adaptive_pocket" {
        p["max_engagement_deg"] = 90.into();
        p["entry"] = json!({"kind": "helix"});
    }
    p
}

/// The run as the stream plans it: every operation against the committed
/// stock, each move through the run's retract policy, then committed.
/// Every rapid is checked against the stock before it, independently of
/// the planner: it removes nothing, and the stream's stock-contact check
/// finds it clear.
fn run(m: &StreamManifest) -> Vec<Motion> {
    let cancel = AtomicBool::new(false);
    let tool = m.tools[0].prepare().unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    stock.enable_contact_queries(&cancel).unwrap();
    let mut position = m.initial_position_mm;
    let mut id = 1;
    let mut moves = vec![];
    for op in &m.operations {
        let mut plan =
            OperationPlan::prepare_with_stock(op.clone(), &m.tools[0], &stock, position, id, &cancel)
                .unwrap();
        while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
            let motion = m.retract_policy(motion, &stock, op.declares_retract_feed());
            assert_eq!(motion.curve.start(), position, "{motion:?}");
            position = motion.curve.end();
            id = motion.id + 1;
            let delta = stock
                .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
                .unwrap();
            if motion.kind == MoveKind::Rapid {
                assert_eq!(delta.stats().changed_cells, 0, "a rapid cuts: {motion:?}");
                let contact = camlib::stock_contact::check(
                    &stock,
                    &tool,
                    &motion,
                    camlib::stock_contact::Options::default(),
                    &cancel,
                )
                .unwrap();
                assert_eq!(contact.status, camlib::verification::Status::Clear, "{motion:?}: {contact:?}");
            }
            stock.apply_delta(&delta).unwrap();
            moves.push(motion);
        }
    }
    moves
}

/// The machine check of the moves (#412's model): the work frame known to
/// +-0.0254 mm on every axis, as the issue's machine declares it, and a
/// spindle head well above the tool.
fn machine_findings(m: &StreamManifest, moves: &[Motion], rapid_mode: RapidMode) -> Vec<Finding> {
    let tool = &m.tools[0];
    let gauge = tool.gauge_length_mm;
    let p = m.initial_position_mm;
    let model = Model {
        version: 1,
        units: "mm".into(),
        id: "synthetic-mill".into(),
        revision: "1".into(),
        process: Process::Milling,
        axes: std::array::from_fn(|_| Axis {
            drive: Drive::Head,
            min_mm: -400.,
            max_mm: 400.,
        }),
        work_offset: 1,
        work_to_machine: WorkFrame::identity(),
        work_error_min_mm: [-0.0254; 3],
        work_error_max_mm: [0.0254; 3],
        initial_axes_mm: [p[0], p[1], p[2] + gauge],
        initial_tool: Some(tool.id.clone()),
        rapid_mode,
        controller_reference: "synthetic controller".into(),
        tools: vec![ToolCalibration {
            tool: tool.id.clone(),
            length_register: 1,
            body_solids: vec![],
            gauge_offset_min_mm: [0., 0., gauge],
            gauge_offset_max_mm: [0., 0., gauge],
        }],
        bodies: vec![Body {
            id: "head".into(),
            component: "spindle_head".into(),
            mount: Mount::Head,
            solids: vec![Solid::Box {
                min_mm: [-20., -20., 0.],
                max_mm: [20., 20., 40.],
            }],
            indexed_solids: Default::default(),
        }],
        absent_components: [
            ("column".into(), "synthetic stand".into()),
            ("table".into(), "remote support excluded by declaration".into()),
            ("enclosure".into(), "open stand".into()),
        ]
        .into(),
        allowed_contacts: vec![AllowedContact {
            bodies: ["head".into(), "@tool".into()],
            reason: "mounted at spindle gauge".into(),
        }],
        tool_change_routes: [(
            tool.id.clone(),
            ToolRoute {
                reference: "synthetic complete indexing envelope".into(),
                before_index_mm: vec![],
                index_envelopes: vec![Solid::Box {
                    min_mm: [-15., -15., -gauge - 1.],
                    max_mm: [15., 15., 1.],
                }],
                replaces: vec!["@tool".into()],
                after_index_mm: vec![],
            },
        )]
        .into(),
        program_end_route_mm: vec![],
        options: Options {
            tolerance_mm: 1e-5,
            spatial_resolution_mm: 0.05,
            max_axis_tests: 100_000_000,
            max_findings: 128,
            stock_contact: camlib::stock_contact::Options::default(),
        },
    };
    let definition = json!({"machine": {"frame": WorkFrame::identity(),
        "work_to_machine": model.work_to_machine, "work_offset": 1,
        "tools": [{"tool": tool.id, "length_register": 1}]}});
    let mut session = Session::new(model, m.clone(), &definition, &AtomicBool::new(false)).unwrap();
    for (i, motion) in moves.iter().enumerate() {
        session
            .event(Event::Motion { motion: motion.clone() }, i, &|| false)
            .unwrap();
    }
    session
        .finish(moves.len() as u64, &|| false)
        .unwrap()
        .findings
        .clone()
}
fn released(m: &StreamManifest, moves: &[Motion]) {
    for mode in [RapidMode::AxisIndependent, RapidMode::Linear] {
        let findings = machine_findings(m, moves, mode);
        assert!(findings.is_empty(), "{mode:?}: {findings:#?}");
    }
}
fn near(p: Vec3, x: f64, y: f64, z: f64) -> bool {
    (p[0] - x).abs() < 1e-3 && (p[1] - y).abs() < 1e-3 && (p[2] - z).abs() < 1e-9
}
/// Whether `m` is a straight lift at feed from `from` to the clearance.
fn fed_lift(m: &Motion, from: Vec3) {
    assert_eq!(m.kind, MoveKind::Retract, "{m:?}");
    assert_eq!(m.feed_mm_min, Some(1000.), "{m:?}");
    assert_eq!(m.curve.start(), from, "{m:?}");
    assert_eq!(m.curve.end(), [from[0], from[1], 10.], "{m:?}");
}

/// The issue's setup: the wall loop is cut whole, its fourth side at feed,
/// and the tool leaves from where that side ends, straight up at feed. The
/// machine check releases it with independent-axis and with linear rapids.
/// Before #430 the lift was a rapid and the independent-axis check left it
/// undecided ("body swept into evolving stock"). The other outline orders
/// end on another corner (or lift at a row end against a wall): the same.
#[test]
fn a_pocket_is_left_beside_its_wall_at_feed_and_released() {
    let m = manifest(12., pocket("pocket", 3., 3));
    let moves = run(&m);
    let tail: Vec<_> = moves[moves.len() - 7..]
        .iter()
        .map(|m| (m.kind, m.curve.start(), m.curve.end()))
        .collect();
    let expected = [
        (MoveKind::Cut, [71., 39.5], [33., 39.5]),
        (MoveKind::Link, [33., 39.5], [33., 41.]),
        (MoveKind::Cut, [33., 41.], [33., 23.]),
        (MoveKind::Cut, [33., 23.], [71., 23.]),
        (MoveKind::Cut, [71., 23.], [71., 41.]),
        // The fourth side, along the wall at Y47.
        (MoveKind::Cut, [71., 41.], [33., 41.]),
    ];
    for (got, want) in tail.iter().zip(&expected) {
        assert_eq!(got.0, want.0, "{tail:#?}");
        assert!(near(got.1, want.1[0], want.1[1], -9.), "{tail:#?}");
        assert!(near(got.2, want.2[0], want.2[1], -9.), "{tail:#?}");
    }
    let last = moves.last().unwrap();
    fed_lift(last, tail[5].2);
    // The facing before it ends past the stock's edge, where no stock stands
    // within reach: that lift is still a rapid.
    let faced = moves.iter().rev().find(|m| m.operation == "face").unwrap();
    assert_eq!(faced.kind, MoveKind::Rapid, "{faced:?}");
    assert_eq!(faced.curve.start()[2], -1.);
    assert!(faced.curve.start()[0] > 104. + 6., "{faced:?}");
    released(&m, &moves);

    for first in 0..3 {
        let m = manifest(12., pocket("pocket", 3., first));
        let moves = run(&m);
        let fourth = &moves[moves.len() - 2];
        assert_eq!(fourth.kind, MoveKind::Cut);
        fed_lift(moves.last().unwrap(), fourth.curve.end());
        released(&m, &moves);
    }
}

/// AdaptivePocket on the same region ends its wall pass against the Y17
/// wall: at stepovers 0.6, 2 and 4 mm with the 12 mm end mill, and 2 mm
/// with a 6 mm one, it leaves straight up at feed and is released.
#[test]
fn an_adaptive_pocket_is_left_beside_its_wall_at_feed_and_released() {
    for (diameter, stepover) in [(12., 0.6), (12., 2.), (12., 4.), (6., 2.)] {
        let m = manifest(diameter, pocket("adaptive_pocket", stepover, 3));
        let moves = run(&m);
        let last = moves.last().unwrap();
        let before = &moves[moves.len() - 2];
        assert!(before.kind.cuts(), "{before:?}");
        fed_lift(last, before.curve.end());
        // Against the wall: the cutter's edge reaches Y17.
        let edge = last.curve.start()[1] - diameter / 2.;
        assert!((edge - 17.).abs() < 1e-3, "{diameter} {stepover}: {last:?}");
        released(&m, &moves);
    }
}

/// Facing planned as a pocket round a clamp that stands on the stock (#411,
/// #417): its rows end beside the stock the clamp leaves standing, and a
/// lift from there stays at feed, so the program is released (#417's PR
/// found such a facing's last lift undecided).
#[test]
fn a_facing_round_a_clamp_is_left_beside_the_clamp_at_feed_and_released() {
    let mut face = facing(12.);
    face["limits"] = json!({"avoid": [{
        "region": {"version": 1, "units": "mm", "name": "clamp", "boundary_error_mm": 0,
            "components": [{"outer": {"vertices": [
                [40, 24, 0], [64, 24, 0], [64, 40, 0], [40, 40, 0]]}, "holes": []}]},
        "top_z_mm": 12}]});
    let mut m = manifest(12., pocket("pocket", 3., 3));
    m.operations = vec![serde_json::from_value(face).unwrap()];
    let moves = run(&m);
    assert!(
        moves.iter().any(|m| m.kind == MoveKind::Retract),
        "a lift beside the clamp stays at feed"
    );
    released(&m, &moves);
}

/// The policy itself, against cut stock: a lift whose tool's whole
/// footprint stands on floor that was cut flat becomes a rapid; the same
/// lift beside the wall the cutter just cut stays at feed. Both start where
/// the last cut left the tool, so `departs_clear` proves each.
#[test]
fn a_retract_is_a_rapid_only_where_the_cell_bounds_show_it_clear() {
    let m = manifest(12., pocket("pocket", 3., 3));
    let cancel = AtomicBool::new(false);
    let tool = m.tools[0].prepare().unwrap();
    let line = |id: u64, kind: MoveKind, start: Vec3, end: Vec3| Motion {
        id,
        operation: "test".into(),
        tool: "em".into(),
        kind,
        curve: Curve::Line { start, end },
        feed_mm_min: (kind != MoveKind::Rapid).then_some(1000.),
    };
    // Rows 3 mm apart over Y20..44 at Z-5, from X20 to X80, then back along
    // the middle row to X50: the 12 mm cutter there stands on cut floor
    // with 9 mm of it on every side.
    let cut = |stock: &mut Stock, motion: &Motion| {
        let d = stock
            .preview_sweep(&tool, motion, 1., m.sweep, &cancel)
            .unwrap();
        stock.apply_delta(&d).unwrap();
    };
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut at = [20., 20., -5.];
    let mut id = 1;
    for row in 0..=8 {
        let y = 20. + 3. * row as f64;
        let to = [if row % 2 == 0 { 80. } else { 20. }, y, -5.];
        if row > 0 {
            let start = [at[0], y, -5.];
            cut(&mut stock, &line(id, MoveKind::Cut, at, start));
            id += 1;
            at = start;
        }
        cut(&mut stock, &line(id, MoveKind::Cut, at, to));
        id += 1;
        at = to;
    }
    let middle = [50., 32., -5.];
    cut(&mut stock, &line(id, MoveKind::Cut, at, [80., 32., -5.]));
    cut(&mut stock, &line(id + 1, MoveKind::Cut, [80., 32., -5.], middle));
    assert!(stock.departs_clear("em", middle));
    let lift = m.retract_policy(
        line(id + 2, MoveKind::Retract, middle, [50., 32., 10.]),
        &stock,
        false,
    );
    assert_eq!(lift.kind, MoveKind::Rapid, "{lift:?}");
    assert_eq!(lift.feed_mm_min, None);
    // From the last row's end the cutter stands against uncut stock.
    let mut wall = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let end = [80., 20., -5.];
    cut(&mut wall, &line(1, MoveKind::Cut, [20., 20., -5.], end));
    assert!(wall.departs_clear("em", end));
    let lift = m.retract_policy(line(2, MoveKind::Retract, end, [80., 20., 10.]), &wall, false);
    assert_eq!(lift.kind, MoveKind::Retract, "{lift:?}");
    assert_eq!(lift.feed_mm_min, Some(1000.));
    // Without `rapid_retracts` nothing becomes a rapid.
    let mut fed = m.clone();
    fed.rapid_retracts = false;
    let lift = fed.retract_policy(
        line(id + 2, MoveKind::Retract, middle, [50., 32., 10.]),
        &stock,
        false,
    );
    assert_eq!(lift.kind, MoveKind::Retract);
}
