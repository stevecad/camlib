//! Plasma/laser profile cutting: order, pierce points, kerf clearance, events
//! and the through-cut stock.
use camlib::{
    motion::*,
    planar::{Boundary, Component, PlanarRegion},
    stock::*,
    stock_contact,
    thermal::{ThermalCutOperation, ThermalCutPlan},
    tool::{Cutter, CutterShape, HolderSection, ToolAssembly},
    verification::Status,
};
use std::sync::atomic::AtomicBool;

const KERF: f64 = 1.5;

fn torch() -> ToolAssembly {
    ToolAssembly {
        id: "torch".into(),
        cutter: Cutter {
            id: "nozzle".into(),
            name: String::new(),
            shape: CutterShape::Beam,
            diameter_mm: KERF,
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

fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Boundary {
    Boundary {
        vertices: vec![[x0, y0, 0.], [x1, y0, 0.], [x1, y1, 0.], [x0, y1, 0.]],
    }
}
fn circle(c: [f64; 2], r: f64) -> Boundary {
    Boundary {
        vertices: vec![[c[0] + r, c[1], 1.], [c[0] - r, c[1], 1.]],
    }
}

fn spec() -> StockSpec {
    StockSpec {
        version: 1,
        units: "mm".into(),
        min_mm: [0., 0., 0.],
        max_mm: [200., 120., 6.],
        cell_size_mm: 0.25,
        max_cells: 1 << 22,
    }
}

/// A plate with a large hole holding a nested washer, a bolt hole and a
/// small hole; the washer has its own hole.
fn parts() -> PlanarRegion {
    PlanarRegion {
        version: 1,
        units: "mm".into(),
        name: "parts".into(),
        boundary_error_mm: 0.,
        components: vec![
            Component {
                outer: rect(20., 20., 120., 100.),
                holes: vec![
                    circle([60., 60.], 25.),
                    circle([100., 40.], 6.),
                    circle([100., 80.], 3.),
                ],
            },
            Component {
                outer: circle([60., 60.], 15.),
                holes: vec![circle([60., 60.], 5.)],
            },
        ],
        source: serde_json::Value::Null,
    }
}

fn operation(extra: serde_json::Value) -> ThermalCutOperation {
    let mut value = serde_json::json!({
        "id": "plasma", "tool": "torch", "parts": parts(), "feed_mm_min": 2500.,
        "cut_height_mm": 1.5, "height_mode": "program", "pierce_height_mm": 3.8,
        "travel_height_mm": 10., "plunge_mm_min": 1000., "pierce_delay_s": 0.5,
        "lead_in": {"length_mm": 2., "radius_mm": 3.}, "lead_out": {"length_mm": 1., "radius_mm": 2.},
        "overcut_mm": -0.5, "height_control": true,
        "small_holes": {"feed_percent": 60.},
        "tolerance_mm": 1e-5, "work_offset": 1, "tool_number": 1, "length_register": 1
    });
    for (k, v) in extra.as_object().unwrap() {
        value[k] = v.clone();
    }
    serde_json::from_value(value).unwrap()
}

fn run(op: ThermalCutOperation) -> Result<(Vec<Event>, ThermalCutPlan, Stock), String> {
    run_from(op, [0., 0., 40.])
}

fn run_from(op: ThermalCutOperation, initial: [f64; 3]) -> Result<(Vec<Event>, ThermalCutPlan, Stock), String> {
    let cancel = AtomicBool::new(false);
    let tool = torch();
    let prepared = tool.prepare().unwrap();
    let mut stock = Stock::new(spec()).unwrap();
    stock.enable_contact_queries(&cancel).unwrap();
    let mut plan = ThermalCutPlan::prepare(op, &tool, &spec(), initial, 1, &cancel)?;
    let mut events = plan.prefix_events();
    events.extend(plan.take_events());
    while let Some(m) = plan.next_motion_with_stock(&stock, &cancel)? {
        let report =
            stock_contact::check(&stock, &prepared, &m, Default::default(), &cancel).unwrap();
        assert_eq!(report.status, Status::Clear, "{m:?}");
        let d = stock
            .preview_sweep(&prepared, &m, 1., SweepOptions::default(), &cancel)
            .unwrap();
        stock.apply_delta(&d).unwrap();
        events.push(Event::Motion { motion: m });
        events.extend(plan.take_events());
    }
    events.extend(plan.suffix_events());
    plan.finish_report(&cancel).unwrap();
    // The whole program is valid: beam on for every cut, THC only with it.
    let program = Program {
        version: 1,
        id: "p".into(),
        revision: "1".into(),
        units: "mm".into(),
        tolerance_mm: 1e-5,
        frame: WorkFrame::identity(),
        initial_position_mm: initial,
        tools: vec![tool],
        events: events.clone(),
    };
    let mut v = ProgramValidator::new(&program, None).unwrap();
    v.consume(&events).unwrap();
    Ok((events, plan, stock))
}

fn top_at(stock: &Stock, p: [f64; 2]) -> (f64, f64) {
    let s = stock.spec();
    let nx = ((s.max_mm[0] - s.min_mm[0]) / s.cell_size_mm).round() as usize;
    let i = ((p[0] - s.min_mm[0]) / s.cell_size_mm) as usize;
    let j = ((p[1] - s.min_mm[1]) / s.cell_size_mm) as usize;
    let c = stock.cells()[j * nx + i];
    (c.lower_mm, c.upper_mm)
}

#[test]
fn cuts_inner_features_first_and_keeps_every_part() {
    let (events, plan, stock) = run(operation(serde_json::json!({}))).unwrap();
    let r = plan.report();
    // Washer (nested in the plate's big hole) first: its hole, its outline;
    // then the plate's holes, the plate outline last.
    assert_eq!(r.order.len(), 6);
    assert_eq!(r.order[0], (1, Some(0)));
    assert_eq!(r.order[1], (1, None));
    assert_eq!(*r.order.last().unwrap(), (0, None));
    assert_eq!(r.loops_completed, 6);
    // The 6 mm hole is small: 1.5 x the 6 mm sheet is 9 mm.
    assert_eq!(r.small_holes, 1);
    let count = |f: &dyn Fn(&Event) -> bool| events.iter().filter(|e| f(e)).count();
    assert_eq!(
        count(&|e| matches!(e, Event::Beam { power_percent } if *power_percent > 0.)),
        6
    );
    assert_eq!(
        count(&|e| matches!(e, Event::Beam { power_percent } if *power_percent == 0.)),
        6
    );
    assert_eq!(count(&|e| matches!(e, Event::Dwell { .. })), 6);
    // THC on every loop but the small hole.
    assert_eq!(
        count(&|e| matches!(e, Event::HeightControl { on: true })),
        5
    );
    // The small hole is cut at 60% feed.
    assert!(events.iter().any(|e| matches!(e, Event::Motion { motion } if motion.kind == MoveKind::Cut && motion.feed_mm_min == Some(1500.))));
    // Kerf lines are cut through; part material and scrap far away are not.
    let kerf_point = [20. - KERF / 2. - 0.005, 60.];
    assert_eq!(top_at(&stock, kerf_point), (0., 0.));
    assert_eq!(top_at(&stock, [70., 30.]), (6., 6.)); // plate
    assert_eq!(top_at(&stock, [60., 50.]), (6., 6.)); // washer
    assert_eq!(top_at(&stock, [180., 110.]), (6., 6.)); // scrap
    // Every cut and lead is at the cut height, travel at the travel height.
    for e in &events {
        if let Event::Motion { motion } = e {
            match motion.kind {
                MoveKind::Cut | MoveKind::Lead => {
                    assert_eq!(motion.curve.start()[2], 7.5);
                    assert_eq!(motion.curve.end()[2], 7.5);
                }
                MoveKind::Plunge => assert_eq!(motion.curve.start()[2], 9.8),
                _ => {}
            }
        }
    }
}

#[test]
fn fixed_height_mode_has_no_z_moves_and_refusals_name_the_problem() {
    let op = operation(serde_json::json!({
        "height_mode": "fixed", "pierce_height_mm": null, "travel_height_mm": null, "plunge_mm_min": null
    }));
    let (events, _, _) = run(op).unwrap();
    for e in &events[3..] {
        if let Event::Motion { motion } = e {
            assert_eq!(motion.curve.start()[2], 7.5, "{motion:?}");
            assert_eq!(motion.curve.end()[2], 7.5, "{motion:?}");
        }
    }
    // A lead longer than the gap to the next part is not proven clear.
    let err = run(operation(
        serde_json::json!({"lead_in": {"length_mm": 40., "radius_mm": 3.}}),
    ))
    .err()
    .unwrap();
    assert!(err.contains("not proven clear"), "{err}");
    // A milling cutter is not a beam.
    let mut mill = torch();
    mill.cutter.shape = CutterShape::Flat;
    let err = ThermalCutPlan::prepare(
        operation(serde_json::json!({})),
        &mill,
        &spec(),
        [0., 0., 40.],
        1,
        &AtomicBool::new(false),
    )
    .err()
    .unwrap();
    assert!(err.contains("beam tool"), "{err}");
}

#[test]
fn micro_joints_leave_bridges_on_outlines() {
    let op = operation(serde_json::json!({
        "micro_joints": {"width_mm": 1.0, "count": 4}
    }));
    let (events, plan, stock) = run(op).unwrap();
    // Two outlines (plate, washer), four joints each; the beam goes off and
    // on again over each.
    assert_eq!(plan.report().micro_joints, 8);
    let on = events
        .iter()
        .filter(|e| matches!(e, Event::Beam { power_percent } if *power_percent > 0.))
        .count();
    assert_eq!(on, 6 + 8);
    // The joint material at the middle of the plate's bottom edge... is
    // wherever a joint landed: some kerf cells along the outline stay solid.
    let s = stock.spec().clone();
    let solid_on_kerf = (0..400)
        .map(|k| [20. + 100. * k as f64 / 400., 20. - KERF / 2. - 0.005])
        .filter(|p| top_at(&stock, *p).1 == s.max_mm[2])
        .count();
    assert!(solid_on_kerf > 0, "a joint keeps the outline attached");
    // With wear compensation micro-joints are refused.
    let err = run(operation(serde_json::json!({
        "micro_joints": {"width_mm": 1.0, "count": 4}, "compensation": "wear"
    })))
    .err()
    .unwrap();
    assert!(err.contains("micro-joints"), "{err}");
}

#[test]
fn machining_limits_cover_beam_depth_and_travel() {
    let region = |x0, y0, x1, y1| {
        serde_json::json!({
            "version": 1, "units": "mm", "name": "limit", "source": null, "components": [{"outer": rect(x0,y0,x1,y1), "holes": []}],
            "boundary_error_mm": 0.0
        })
    };
    let prepare = |extra| {
        ThermalCutPlan::prepare(
            operation(extra),
            &torch(),
            &spec(),
            [0., 0., 40.],
            1,
            &AtomicBool::new(false),
        )
    };
    prepare(serde_json::json!({"limits": {
        "contain": region(0.,0.,200.,120.),
        "avoid": [{"region": region(180.,100.,190.,110.), "top_z_mm": 30.}]
    }}))
    .unwrap_or_else(|e| panic!("clear thermal limits: {e}"));
    // Below the nozzle but above the sheet bottom: the beam still crosses it.
    assert!(
        prepare(serde_json::json!({"limits": {"avoid": [
            {"region": region(15.,15.,125.,105.), "top_z_mm": 1.}
        ]}}))
        .is_err()
    );
    assert!(prepare(serde_json::json!({"limits": {"contain": region(40.,40.,80.,80.)}})).is_err());
    // Travel to the first pierce passes through this tall obstacle.
    assert!(
        prepare(serde_json::json!({"limits": {"avoid": [
            {"region": region(-5.,-5.,200.,120.), "top_z_mm": 50.}
        ]}}))
        .is_err()
    );
}

/// Five 26 x 26 squares in a row, 10 mm apart, each with a hole.
fn row() -> PlanarRegion {
    PlanarRegion {
        version: 1,
        units: "mm".into(),
        name: "row".into(),
        boundary_error_mm: 0.,
        components: (0..5)
            .map(|k| {
                let x = 10. + 36. * k as f64;
                Component {
                    outer: rect(x, 40., x + 26., 66.),
                    holes: vec![circle([x + 13., 53.], 5.)],
                }
            })
            .collect(),
        source: serde_json::Value::Null,
    }
}

#[test]
fn the_part_order_is_nearest_given_or_spread_and_holes_always_come_first() {
    let cut = |extra: serde_json::Value| {
        let mut op = operation(extra);
        op.parts = row();
        let (_, plan, _) = run(op).unwrap();
        let report = serde_json::to_value(plan.report()).unwrap();
        let order: Vec<(usize, Option<usize>)> = serde_json::from_value(report["order"].clone()).unwrap();
        // In every order: each part's hole, then its outline, one after the other.
        for pair in order.chunks(2) {
            assert_eq!(pair[0], (pair[0].0, Some(0)));
            assert_eq!(pair[1], (pair[0].0, None));
        }
        let parts: Vec<usize> = order.iter().filter(|l| l.1.is_none()).map(|l| l.0).collect();
        (
            parts,
            report["min_part_step_mm"].as_f64().unwrap(),
            report["rapid_length_mm"].as_f64().unwrap(),
        )
    };
    // Nearest (the default): along the row, each part next to the last.
    let (nearest, step, travel) = cut(serde_json::json!({}));
    assert_eq!(nearest, [0, 1, 2, 3, 4]);
    assert!((step - 36.).abs() < 1e-9);
    // Given, with the parts listed in another order.
    let mut op = operation(serde_json::json!({"part_order": {"kind": "given"}}));
    let mut parts = row();
    parts.components.swap(0, 3);
    op.parts = parts;
    let (_, plan, _) = run(op).unwrap();
    let given: Vec<usize> = plan.report().order.iter().filter(|l| l.1.is_none()).map(|l| l.0).collect();
    assert_eq!(given, [0, 1, 2, 3, 4]);
    // Spread: no two consecutive parts are neighbours while a part farther
    // than the declared distance remains, and the travel is what it costs.
    let (spread, step, spread_travel) = cut(serde_json::json!({"part_order": {"kind": "spread", "min_distance_mm": 60.}}));
    let mut sorted = spread.clone();
    sorted.sort();
    assert_eq!(sorted, [0, 1, 2, 3, 4]);
    assert!(step >= 60. - 1e-9, "{spread:?} {step}");
    assert!(spread_travel > travel);
    // The distance is declared, and must make sense.
    let mut op = operation(serde_json::json!({"part_order": {"kind": "spread", "min_distance_mm": 0.}}));
    op.parts = row();
    assert!(run(op).err().unwrap().contains("min_distance_mm 0 must be positive"));
    assert!(
        serde_json::from_value::<ThermalCutOperation>(serde_json::json!({"part_order": {"kind": "spread"}})).is_err()
    );
}

#[test]
fn rapids_go_round_parts_already_cut_free_or_the_cut_is_refused() {
    // Five parts in a row, cut in an order that sends the torch back along
    // the row: without the option the rapids cross parts already cut free.
    let cut = |extra: serde_json::Value| {
        let mut op = operation(extra);
        op.parts = row();
        run(op)
    };
    let spread = serde_json::json!({"kind": "spread", "min_distance_mm": 60.});
    // Independently of the planner: replay the moves, free a part when its
    // outline's cut is complete, and measure every later rapid against it.
    let squares: Vec<[f64; 4]> = (0..5).map(|k| [10. + 36. * k as f64, 40., 36. + 36. * k as f64, 66.]).collect();
    let gap = |a: [f64; 3], b: [f64; 3], s: &[f64; 4]| {
        // Least distance from the segment to the square, by sampling.
        (0..=400)
            .map(|k| {
                let t = k as f64 / 400.;
                let (x, y) = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]));
                let dx = (s[0] - x).max(x - s[2]).max(0.);
                let dy = (s[1] - y).max(y - s[3]).max(0.);
                dx.hypot(dy)
            })
            .fold(f64::INFINITY, f64::min)
    };
    let worst = |extra: serde_json::Value| -> (f64, serde_json::Value) {
        let (events, plan, _) = cut(extra).unwrap();
        let report = serde_json::to_value(plan.report()).unwrap();
        let outlines: Vec<usize> = plan.report().order.iter().filter(|l| l.1.is_none()).map(|l| l.0).collect();
        let all: Vec<Motion> = events
            .into_iter()
            .filter_map(|e| match e {
                Event::Motion { motion } => Some(motion),
                _ => None,
            })
            .collect();
        // A part is free once the torch has left its outline: count the
        // cut moves that lie on each square's kerf to find when.
        let mut free: Vec<usize> = vec![];
        let mut least = f64::INFINITY;
        let mut next = 0;
        let mut on_outline = false;
        for m in &all {
            let (a, b) = (m.curve.start(), m.curve.end());
            match m.kind {
                MoveKind::Cut => {
                    let s = &squares[outlines[next.min(4)]];
                    // On the outline's kerf: outside the square, within 2 mm.
                    let outside = a[0] < s[0] || a[0] > s[2] || a[1] < s[1] || a[1] > s[3];
                    on_outline |= outside && gap(a, b, s) < 2.;
                }
                MoveKind::Rapid if a[0] != b[0] || a[1] != b[1] => {
                    if on_outline {
                        free.push(outlines[next]);
                        next += 1;
                        on_outline = false;
                        // Leaving the part just cut: its own kerf side.
                        for &p in &free[..free.len() - 1] {
                            least = least.min(gap(a, b, &squares[p]));
                        }
                        continue;
                    }
                    for &p in &free {
                        least = least.min(gap(a, b, &squares[p]));
                    }
                }
                _ => {}
            }
        }
        (least, report)
    };
    let (crossing, report) = worst(serde_json::json!({"part_order": spread}));
    assert!(crossing < 1e-6, "{crossing}");
    assert!(report.get("rerouted_rapids").is_none());
    let (kept, report) = worst(serde_json::json!({"part_order": spread, "avoid_cut_parts": {"clearance_mm": 4.}}));
    assert!(kept >= 4. - 1e-6, "{kept}");
    assert!(report["rerouted_rapids"].as_u64().unwrap() > 0);
    // A clearance wider than the gaps between the parts cannot be kept.
    let error = cut(serde_json::json!({"part_order": spread, "avoid_cut_parts": {"clearance_mm": 30.}}))
        .err()
        .unwrap();
    assert!(error.contains("cannot keep 30 mm from the parts already cut free"), "{error}");
    let error = cut(serde_json::json!({"avoid_cut_parts": {"clearance_mm": 0.}})).err().unwrap();
    assert!(error.contains("clearance_mm 0 must be positive"), "{error}");
}

#[test]
fn a_crop_cut_is_the_last_cut_and_frees_the_offcut() {
    use camlib::nesting::remnant::{RemnantRequest, remnant};
    let crop = serde_json::json!({"points_mm": [[135., 0.], [135., 108.], [0., 108.]]});
    let (events, plan, stock) = run(operation(serde_json::json!({"crop": crop.clone()}))).unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    assert_eq!(report["crop_length_mm"], 108. + 135.);
    let plain = serde_json::to_value(run(operation(serde_json::json!({}))).unwrap().1.report()).unwrap();
    assert_eq!(report["pierces"].as_u64(), Some(plain["pierces"].as_u64().unwrap() + 1));
    let added = report["cut_length_mm"].as_f64().unwrap() - plain["cut_length_mm"].as_f64().unwrap();
    assert!((added - 243.).abs() < 1e-9);
    // The last two cutting moves are the crop cut, after every other cut,
    // at the cut height and feed.
    let cuts: Vec<&Motion> = events
        .iter()
        .filter_map(|e| match e {
            Event::Motion { motion } if motion.kind == MoveKind::Cut => Some(motion),
            _ => None,
        })
        .collect();
    let last = &cuts[cuts.len() - 2..];
    assert_eq!((last[0].curve.start(), last[0].curve.end()), ([135., 0., 7.5], [135., 108., 7.5]));
    assert_eq!(last[1].curve.end(), [0., 108., 7.5]);
    assert_eq!(last[1].feed_mm_min, Some(2500.));
    // The sheet is cut through along it; the parts are whole.
    assert_eq!(top_at(&stock, [135., 50.]), (0., 0.));
    assert_eq!(top_at(&stock, [60., 108.]), (0., 0.));
    assert_eq!(top_at(&stock, [70., 30.]), (6., 6.));
    // The remnant with the crop cut: the offcut is a piece of its own, the
    // other side of the kerf from the parts.
    let request: RemnantRequest = serde_json::from_value(serde_json::json!({
        "sheet": PlanarRegion {
            version: 1, units: "mm".into(), name: "sheet".into(), boundary_error_mm: 0.,
            components: vec![Component { outer: rect(0., 0., 200., 120.), holes: vec![] }],
            source: serde_json::Value::Null,
        },
        "parts": parts(), "kerf_mm": KERF,
        "lead_in": {"length_mm": 2., "radius_mm": 3.}, "lead_out": {"length_mm": 1., "radius_mm": 2.},
        "tolerance_mm": 1e-5, "remnant_margin_mm": 1., "crop": crop
    }))
    .unwrap();
    let left = remnant(&request).unwrap();
    let offcut = &left.region.components[0];
    // An L round the top and the right: every corner beyond the kerf's edge
    // and the margin (half the kerf, the pierce clearance, the margin).
    let reach = KERF / 2. + KERF + 1.;
    for v in &offcut.outer.vertices {
        assert!(v[0] >= 135. + reach || v[1] >= 108. + reach, "{v:?}");
    }
    let (points, nearest, required) = left.check.crop.unwrap();
    assert!(points > 50 && nearest >= required && required == reach);
    // No cell the cut removed lies in any piece of the remnant.
    let spec = spec();
    for j in 0..480 {
        for i in 0..800 {
            let p = [(i as f64 + 0.5) * spec.cell_size_mm, (j as f64 + 0.5) * spec.cell_size_mm];
            if top_at(&stock, p).1 < 6. {
                for c in &left.region.components {
                    let all = std::iter::once(&c.outer).chain(&c.holes);
                    assert!(all.clone().all(|b| b.vertices.iter().all(|v| v[2] == 0.)));
                    assert!(!inside(&c.outer, p) || c.holes.iter().any(|h| inside(h, p)), "{p:?}");
                }
            }
        }
    }
    // Through a part, or not a line: refused by name.
    let through = run(operation(serde_json::json!({"crop": {"points_mm": [[60., 0.], [60., 120.]]}})));
    assert!(through.err().unwrap().contains("crop cut"));
    let point = run(operation(serde_json::json!({"crop": {"points_mm": [[150., 0.]]}})));
    assert!(point.err().unwrap().contains("crop needs two points"));
}

/// Even-odd test against a boundary's vertices (straight-edged outlines).
fn inside(b: &Boundary, p: [f64; 2]) -> bool {
    let v = &b.vertices;
    let mut inside = false;
    for i in 0..v.len() {
        let (a, c) = (v[i], v[(i + 1) % v.len()]);
        if (a[1] > p[1]) != (c[1] > p[1]) && p[0] < a[0] + (p[1] - a[1]) * (c[0] - a[0]) / (c[1] - a[1]) {
            inside = !inside;
        }
    }
    inside
}

/// Where each loop starts: the start of the first cutting move after a lead.
fn loop_starts(events: &[Event]) -> Vec<[f64; 2]> {
    let mut out = vec![];
    let mut led = false;
    for e in events {
        if let Event::Motion { motion } = e {
            if motion.kind == MoveKind::Cut && led {
                let p = motion.curve.start();
                out.push([p[0], p[1]]);
            }
            led = motion.kind == MoveKind::Lead;
        }
    }
    out
}

/// The distance from p to the segment a-b (the test's own).
fn to_segment(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let (d, v) = ([b[0] - a[0], b[1] - a[1]], [p[0] - a[0], p[1] - a[1]]);
    let t = ((v[0] * d[0] + v[1] * d[1]) / (d[0] * d[0] + d[1] * d[1])).clamp(0., 1.);
    (v[0] - t * d[0]).hypot(v[1] - t * d[1])
}

#[test]
fn leads_keep_off_declared_edges_and_go_where_preferred() {
    let outside = |p: &[f64; 2]| p[0] < 20. || p[0] > 120. || p[1] < 20. || p[1] > 100.;
    let cut = |placement: serde_json::Value| {
        let (events, plan, _) = run(operation(serde_json::json!({"lead_placement": placement})))?;
        let report = serde_json::to_value(plan.report()).unwrap();
        Ok::<_, String>((loop_starts(&events), report["leads_moved"].as_u64().unwrap()))
    };
    let (plain_events, plain_plan, _) = run(operation(serde_json::json!({}))).unwrap();
    let plain = loop_starts(&plain_events);
    assert_eq!(plain.len(), 6);
    assert!(serde_json::to_value(plain_plan.report()).unwrap().get("leads_moved").is_none());
    let plate = *plain.iter().find(|p| outside(p)).unwrap();
    // The plate's bottom and left edges are to stay free of leads, 5 mm
    // round them. As drawn, its outline starts on one of them.
    let (bottom, left) = (([20., 20.], [120., 20.]), ([20., 20.], [20., 100.]));
    let near = |p: [f64; 2]| to_segment(p, bottom.0, bottom.1).min(to_segment(p, left.0, left.1));
    assert!(near(plate) < 5., "{plate:?}");
    let keep_off = serde_json::json!([
        {"start_mm": bottom.0, "segments": [{"kind": "line", "end_mm": bottom.1}]},
        {"start_mm": left.0, "segments": [{"kind": "line", "end_mm": left.1}]},
    ]);
    let (starts, moved) =
        cut(serde_json::json!({"keep_off": keep_off, "keep_off_distance_mm": 5.})).unwrap();
    assert_eq!(starts.len(), 6);
    // Every start is at least the 5 mm away, along with the kerf's offset
    // from the drawn edge; only the plate's outline had to move.
    assert!(starts.iter().all(|p| near(*p) > 5.7), "{starts:?}");
    assert_eq!(moved, 1);
    let unchanged = starts.iter().filter(|p| plain.contains(p)).count();
    assert_eq!(unchanged, 5);
    // A corner preferred: the one corner off both edges, met in the middle
    // of the arc the kerf path makes round it.
    let (starts, _) = cut(serde_json::json!({
        "keep_off": keep_off, "keep_off_distance_mm": 5., "prefer": {"kind": "corner"}
    }))
    .unwrap();
    let corner = *starts.iter().find(|p| outside(p)).unwrap();
    let d = (corner[0] - 120.).hypot(corner[1] - 100.);
    assert!(corner[0] > 120. && corner[1] > 100. && (0.74..0.78).contains(&d), "{corner:?}");
    assert!(((corner[0] - 120.) - (corner[1] - 100.)).abs() < 1e-9);
    // A side preferred, nothing kept off: every loop starts at its point
    // furthest that way.
    let (starts, _) = cut(serde_json::json!({
        "keep_off_distance_mm": 0., "prefer": {"kind": "toward", "direction": [-1., 0.]}
    }))
    .unwrap();
    let west = *starts.iter().find(|p| outside(p)).unwrap();
    assert!(west[0] < 20. && west[0] > 19.2, "{west:?}");
    // The whole outline kept off: refused by name, not planned anyway.
    let all = serde_json::json!([{"start_mm": [20., 20.], "segments": [
        {"kind": "line", "end_mm": [120., 20.]}, {"kind": "line", "end_mm": [120., 100.]},
        {"kind": "line", "end_mm": [20., 100.]}, {"kind": "line", "end_mm": [20., 20.]}
    ]}]);
    let error = cut(serde_json::json!({"keep_off": all, "keep_off_distance_mm": 5.})).unwrap_err();
    assert!(error.contains("part 0 outline") && error.contains("kept free of leads"), "{error}");
    // A distance must be declared sensibly.
    let error = cut(serde_json::json!({"keep_off_distance_mm": -1.})).unwrap_err();
    assert!(error.contains("keep_off_distance_mm"), "{error}");
}

/// The distance from p to the plate's rectangle (0 inside it).
fn off_plate(p: [f64; 3]) -> f64 {
    let dx = (20. - p[0]).max(p[0] - 120.).max(0.);
    let dy = (20. - p[1]).max(p[1] - 100.).max(0.);
    dx.hypot(dy)
}
/// The plate's outline: the cut moves that run outside its rectangle.
fn outline_cuts(events: &[Event]) -> Vec<&Motion> {
    events
        .iter()
        .filter_map(|e| match e {
            Event::Motion { motion }
                if motion.kind == MoveKind::Cut
                    && off_plate(motion.curve.start()) > 0.
                    && off_plate(motion.curve.end()) > 0. =>
            {
                Some(motion)
            }
            _ => None,
        })
        .collect()
}
fn length(c: &Curve) -> f64 {
    match c {
        Curve::Line { start, end } => (end[0] - start[0]).hypot(end[1] - start[1]),
        Curve::Arc {
            start,
            center,
            sweep_rad,
            ..
        }
        | Curve::Helix {
            start,
            center,
            sweep_rad,
            ..
        } => (start[0] - center[0]).hypot(start[1] - center[1]) * sweep_rad.abs(),
    }
}

#[test]
fn corner_loops_cut_straight_through_outside_corners_in_the_scrap() {
    // The kerf path runs 0.755 mm off the drawn edge (half the kerf and half
    // the boundary tolerance), round each corner of the plate on an arc of
    // that radius.
    let offset = KERF / 2. + 0.005;
    let (plain_events, plain_plan, _) = run(operation(serde_json::json!({}))).unwrap();
    let plain = serde_json::to_value(plain_plan.report()).unwrap();
    assert!(plain.get("corner_loops").is_none());
    let d = 3.;
    let (events, plan, _) = run(operation(serde_json::json!({
        "corner_loops": {"overshoot_mm": d, "min_angle_deg": 45., "max_angle_deg": 135.}
    })))
    .unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    // Every corner but the one the outline starts at, if it starts at one.
    let loops = report["corner_loops"].as_u64().unwrap() as usize;
    let cuts = outline_cuts(&events);
    let starts_at_corner = off_plate(outline_cuts(&plain_events)[0].curve.start()) > offset - 1e-9
        && matches!(outline_cuts(&plain_events)[0].curve, Curve::Arc { .. });
    assert_eq!(loops, if starts_at_corner { 3 } else { 4 });
    // A square corner's loop: on past the point where the two edges of the
    // kerf path meet by d, round 270 degrees at radius d, back on d before
    // it. That replaces the corner's quarter arc.
    let each = 2. * (offset + d) + 1.5 * std::f64::consts::PI * d
        - offset * std::f64::consts::FRAC_PI_2;
    let grown = report["cut_length_mm"].as_f64().unwrap() - plain["cut_length_mm"].as_f64().unwrap();
    assert!((grown - loops as f64 * each).abs() < 1e-6, "{grown} {each}");
    // In the scrap: no cut of the outline comes nearer the plate than the
    // kerf path does, and the loops reach d / tan(22.5 deg) beyond the
    // meeting point, which is offset * sqrt(2) from the drawn corner.
    let mut far = 0f64;
    for m in &cuts {
        let middle = m.curve.prepare(1e-6).unwrap().position(0.5).unwrap();
        for p in [m.curve.start(), middle, m.curve.end()] {
            assert!(off_plate(p) > offset - 1e-6, "{p:?}");
            far = far.max(off_plate(p));
        }
    }
    let reach = offset * 2f64.sqrt() + d / 22.5f64.to_radians().tan();
    assert!((far - reach).abs() < 1e-6, "{far} {reach}");
    // Straight through: at each looped corner, the meeting point lies on two
    // straight cuts (one edge running on, the next coming back), and no
    // quarter arc of the kerf's radius is left there.
    let mut through = 0;
    for corner in [[20., 20.], [120., 20.], [120., 100.], [20., 100.]] {
        let v = [
            corner[0] + if corner[0] < 70. { -offset } else { offset },
            corner[1] + if corner[1] < 60. { -offset } else { offset },
        ];
        let lines = cuts
            .iter()
            .filter(|m| match m.curve {
                Curve::Line { start, end } => {
                    to_segment(v, [start[0], start[1]], [end[0], end[1]]) < 1e-9
                        && (start[0] - v[0]).hypot(start[1] - v[1]) > 1.
                        && (end[0] - v[0]).hypot(end[1] - v[1]) > 1.
                }
                _ => false,
            })
            .count();
        let quarter = cuts.iter().any(|m| match m.curve {
            Curve::Arc { center, start, .. } => {
                (center[0] - corner[0]).hypot(center[1] - corner[1]) < 1e-9
                    && ((start[0] - center[0]).hypot(start[1] - center[1]) - offset).abs() < 1e-9
            }
            _ => false,
        });
        if lines == 2 {
            assert!(!quarter, "{corner:?}");
            through += 1;
        } else {
            assert!(quarter && lines == 0, "{corner:?} {lines}");
        }
    }
    assert_eq!(through, loops);
    // A loop that would cut another part is refused like a lead that would:
    // nothing is skipped quietly.
    let mut crowded = parts();
    crowded.components.push(Component {
        outer: rect(122., 102., 140., 115.),
        holes: vec![],
    });
    let error = run(operation(serde_json::json!({
        "parts": crowded,
        "corner_loops": {"overshoot_mm": d, "min_angle_deg": 45., "max_angle_deg": 135.}
    })))
    .map(|_| ())
    .unwrap_err();
    assert!(error.contains("kerf path") && error.contains("the loops need room"), "{error}");
    // Declared values only, and not with the options it is not planned with.
    for (extra, word) in [
        (serde_json::json!({"corner_loops": {"overshoot_mm": 0., "min_angle_deg": 45., "max_angle_deg": 135.}}), "overshoot_mm"),
        (serde_json::json!({"corner_loops": {"overshoot_mm": 3., "min_angle_deg": 100., "max_angle_deg": 90.}}), "min_angle_deg"),
        (serde_json::json!({"compensation": "wear", "corner_loops": {"overshoot_mm": 3., "min_angle_deg": 45., "max_angle_deg": 135.}}), "wear compensation"),
    ] {
        let error = run(operation(extra)).map(|_| ()).unwrap_err();
        assert!(error.contains(word), "{error}");
    }
}

#[test]
fn corner_slowdown_reduces_the_feed_round_corners_with_height_control_held_off() {
    let offset = KERF / 2. + 0.005;
    let (distance, percent) = (4., 50.);
    let (events, plan, _) = run(operation(serde_json::json!({
        "corner_slowdown": {"distance_mm": distance, "feed_percent": percent, "min_angle_deg": 60.}
    })))
    .unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    let corner = |p: [f64; 3]| {
        [[20., 20.], [120., 20.], [120., 100.], [20., 100.]]
            .iter()
            .map(|c| (p[0] - c[0]).hypot(p[1] - c[1]))
            .fold(f64::MAX, f64::min)
    };
    // Independently of the plan: a point of the kerf path within the
    // distance of a corner's arc, along the path, is within
    // hypot(distance + offset, offset) of the drawn corner, and one further
    // along a straight edge is at least hypot(distance, offset) from it.
    let (near, away) = ((distance + offset).hypot(offset), distance.hypot(offset));
    let (mut slow_length, mut slow_moves, mut thc) = (0., 0, false);
    for e in &events {
        match e {
            Event::HeightControl { on } => thc = *on,
            Event::Motion { motion } if motion.kind == MoveKind::Cut => {
                let (a, b) = (motion.curve.start(), motion.curve.end());
                let feed = motion.feed_mm_min.unwrap();
                if (feed - 2500. * percent / 100.).abs() < 1e-9 {
                    assert!(corner(a) < near + 1e-6 && corner(b) < near + 1e-6, "{motion:?}");
                    // The arc voltage rises as the torch slows: height
                    // control is off here.
                    assert!(!thc, "{motion:?}");
                    slow_length += length(&motion.curve);
                    slow_moves += 1;
                } else if off_plate(a) > 0. && off_plate(b) > 0. {
                    assert_eq!(feed, 2500.);
                    assert!(corner(a) > away - 1e-6 && corner(b) > away - 1e-6, "{motion:?}");
                    assert!(thc, "{motion:?}");
                }
            }
            _ => {}
        }
    }
    // Four corners, each slowed from 4 mm before its arc to 4 mm after it,
    // across the outline's start too; the cut stops 0.5 mm short of closing
    // (the overcut), inside a slowed stretch if the outline starts there.
    assert_eq!(report["corner_slowdowns"], 4);
    let whole = 4. * (2. * distance + offset * std::f64::consts::FRAC_PI_2);
    assert!(slow_moves >= 12, "{slow_moves}");
    assert!(
        (slow_length - whole).abs() < 1e-6 || (slow_length - (whole - 0.5)).abs() < 1e-6,
        "{slow_length} {whole}"
    );
    // The circles are no corners, and the small hole keeps its own feed.
    assert!(events.iter().any(|e| matches!(e, Event::Motion { motion }
        if motion.kind == MoveKind::Cut && motion.feed_mm_min == Some(1500.))));
    // With loops as well, the looped corners need no slow-down: only the
    // corner the outline starts at, if any, is slowed.
    let (_, plan, _) = run(operation(serde_json::json!({
        "corner_loops": {"overshoot_mm": 3., "min_angle_deg": 45., "max_angle_deg": 135.},
        "corner_slowdown": {"distance_mm": distance, "feed_percent": percent, "min_angle_deg": 60.}
    })))
    .unwrap();
    let both = serde_json::to_value(plan.report()).unwrap();
    assert_eq!(
        both["corner_slowdowns"].as_u64().unwrap() + both["corner_loops"].as_u64().unwrap(),
        4
    );
    // Without height control asked for, none is switched.
    let (events, _, _) = run(operation(serde_json::json!({
        "height_control": false,
        "corner_slowdown": {"distance_mm": distance, "feed_percent": percent, "min_angle_deg": 60.}
    })))
    .unwrap();
    assert!(!events.iter().any(|e| matches!(e, Event::HeightControl { .. })));
    for (extra, word) in [
        (serde_json::json!({"distance_mm": 0., "feed_percent": 50., "min_angle_deg": 60.}), "distance_mm"),
        (serde_json::json!({"distance_mm": 4., "feed_percent": 100., "min_angle_deg": 60.}), "feed_percent"),
        (serde_json::json!({"distance_mm": 4., "feed_percent": 50., "min_angle_deg": 0.}), "min_angle_deg"),
    ] {
        let error = run(operation(serde_json::json!({"corner_slowdown": extra})))
            .map(|_| ())
            .unwrap_err();
        assert!(error.contains(word), "{error}");
    }
}

/// Beam-on events (pierces) and the lead moves longer than any lead-in or
/// lead-out (the chain links), as the program runs them.
type Link = ([f64; 3], [f64; 3]);
fn pierces_and_links(events: &[Event]) -> (usize, Vec<Link>) {
    let mut pierces = 0;
    let mut links = vec![];
    let mut on = false;
    for e in events {
        match e {
            Event::Beam { power_percent } => {
                pierces += usize::from(*power_percent > 0.);
                on = *power_percent > 0.;
            }
            Event::Motion { motion } if motion.kind == MoveKind::Lead => {
                assert!(on, "a lead with the beam off: {motion:?}");
                if matches!(motion.curve, Curve::Line { .. }) && length(&motion.curve) > 5. {
                    links.push((motion.curve.start(), motion.curve.end()));
                }
            }
            _ => {}
        }
    }
    (pierces, links)
}

#[test]
fn chain_cutting_joins_outlines_through_the_scrap_with_one_pierce() {
    let cut = |extra: serde_json::Value| {
        let mut op = operation(extra);
        op.parts = row();
        run(op)
    };
    let squares: Vec<[f64; 4]> =
        (0..5).map(|k| [10. + 36. * k as f64, 40., 36. + 36. * k as f64, 66.]).collect();
    let off = |p: [f64; 2], s: &[f64; 4]| {
        let dx = (s[0] - p[0]).max(p[0] - s[2]).max(0.);
        let dy = (s[1] - p[1]).max(p[1] - s[3]).max(0.);
        dx.hypot(dy)
    };
    // Without the option: a pierce for every hole and every outline.
    let (events, plan, _) = cut(serde_json::json!({})).unwrap();
    let plain = serde_json::to_value(plan.report()).unwrap();
    assert_eq!(pierces_and_links(&events), (10, vec![]));
    assert_eq!(plain["pierces"], 10);
    assert!(plain.get("chain_links").is_none());
    // Chained: the holes first, then the five outlines on one pierce.
    let (events, plan, stock) = cut(serde_json::json!({"chain": {"max_link_mm": 45.}})).unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    let (pierces, links) = pierces_and_links(&events);
    assert_eq!((pierces, links.len()), (6, 4));
    assert_eq!((report["pierces"].as_u64(), report["chain_links"].as_u64()), (Some(6), Some(4)));
    let total: f64 = links
        .iter()
        .map(|(a, b)| (a[0] - b[0]).hypot(a[1] - b[1]))
        .sum();
    assert!((report["chain_length_mm"].as_f64().unwrap() - total).abs() < 1e-9);
    // Every hole before every outline; each part's hole before its outline.
    let order = report["order"].as_array().unwrap();
    assert!(order[..5].iter().all(|l| !l[1].is_null()) && order[5..].iter().all(|l| l[1].is_null()));
    // Independently of the planner: each link's kerf (0.75 mm either side
    // of it) stays off every part, and is no longer than declared.
    for (a, b) in &links {
        assert!((a[0] - b[0]).hypot(a[1] - b[1]) <= 45.);
        for k in 0..=400 {
            let t = k as f64 / 400.;
            let p = [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
            assert!(squares.iter().all(|s| off(p, s) > 0.75), "{p:?}");
        }
    }
    // The same feed-seconds less nothing, the dwells for four pierces less.
    assert!(
        (plain["dwell_seconds"].as_f64().unwrap() - report["dwell_seconds"].as_f64().unwrap() - 2.)
            .abs()
            < 1e-9
    );
    // Every part is still cut free and kept: its outline's kerf is open
    // and the part beside it stands.
    for s in &squares {
        let y = 0.5 * (s[1] + s[3]);
        assert!(top_at(&stock, [s[0] - 0.75, y]).1 <= 0.01, "{s:?}");
        assert!(top_at(&stock, [s[2] + 0.75, y]).1 <= 0.01, "{s:?}");
        assert!(top_at(&stock, [s[0] + 3., y]).0 >= 5.99, "{s:?}");
    }
    // Round parts with holes, staggered: the kerf paths are arcs and the
    // links leave them at an angle. Each link still stays its kerf's half
    // width off every disc, measured from the disc centres.
    let discs: Vec<[f64; 2]> = (0..5)
        .map(|k| [24. + 34. * k as f64, if k % 2 == 0 { 40. } else { 58. }])
        .collect();
    let mut op = operation(serde_json::json!({"chain": {"max_link_mm": 60.}}));
    op.parts = PlanarRegion {
        components: discs
            .iter()
            .map(|c| Component {
                outer: circle(*c, 12.),
                holes: vec![circle(*c, 4.)],
            })
            .collect(),
        ..row()
    };
    let (events, plan, _) = run(op).unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    let (pierces, links) = pierces_and_links(&events);
    assert!(!links.is_empty() && pierces == 10 - links.len(), "{pierces} {links:?}");
    assert_eq!(report["chain_links"].as_u64(), Some(links.len() as u64));
    for (a, b) in &links {
        for k in 0..=400 {
            let t = k as f64 / 400.;
            let p = [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
            assert!(
                discs.iter().all(|c| (p[0] - c[0]).hypot(p[1] - c[1]) > 12. + 0.75),
                "{p:?}"
            );
        }
    }
    // A link longer than declared is not cut: the next outline is pierced.
    let (events, plan, _) = cut(serde_json::json!({"chain": {"max_link_mm": 5.}})).unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    assert_eq!(pierces_and_links(&events), (10, vec![]));
    assert_eq!((report["pierces"].as_u64(), report["chain_links"].as_u64()), (Some(10), Some(0)));
    // Refusals name the problem.
    let error = cut(serde_json::json!({"chain": {"max_link_mm": 0.}})).err().unwrap();
    assert!(error.contains("max_link_mm"), "{error}");
    let error = cut(serde_json::json!({"chain": {"max_link_mm": 45.}, "common_line": {"min_length_mm": 5.}})).err().unwrap();
    assert!(error.contains("chain is not planned together with common_line"), "{error}");
}

#[test]
fn bridges_keep_neighbouring_parts_joined_by_an_uncut_strip() {
    let cut = |extra: serde_json::Value| {
        let mut op = operation(extra);
        op.parts = row();
        run(op)
    };
    let bridges = serde_json::json!({"width_mm": 3., "max_length_mm": 12., "max_parts": 8});
    let (events, plan, stock) = cut(serde_json::json!({"bridges": bridges})).unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    // The five holes, then one loop round all five parts: six pierces.
    let (pierces, _) = pierces_and_links(&events);
    assert_eq!(pierces, 6);
    assert_eq!(report["pierces"], 6);
    assert_eq!(report["bridged_groups"], serde_json::json!([[0, 1, 2, 3, 4]]));
    let planned = report["bridges"].as_array().unwrap();
    assert_eq!(planned.len(), 4);
    let order = report["order"].as_array().unwrap();
    assert!(order[..5].iter().all(|l| !l[1].is_null()));
    assert_eq!(order[5..].iter().filter(|l| l[1].is_null()).count(), 5);
    // In the stock, independently of the planner. Between parts k and k+1
    // (edges at x = 36 + 36 k and 46 + 36 k) the bridge is at the middle of
    // the facing edges, y = 53: the strip stands from one part to the other,
    // 3 mm wide, and the kerf either side of it is cut through.
    for (k, bridge) in planned.iter().enumerate() {
        let (x0, x1) = (36. + 36. * k as f64, 46. + 36. * k as f64);
        assert_eq!(bridge["parts"], serde_json::json!([k, k + 1]));
        assert!((bridge["length_mm"].as_f64().unwrap() - 10.).abs() < 1e-9);
        let at = bridge["at_mm"].as_array().unwrap();
        assert!((at[0].as_f64().unwrap() - 0.5 * (x0 + x1)).abs() < 1e-9);
        assert!((at[1].as_f64().unwrap() - 53.).abs() < 1e-9);
        for x in [x0 - 1., x0 + 0.5, 0.5 * (x0 + x1), x1 - 0.5, x1 + 1.] {
            for y in [52., 53., 54.] {
                assert!(top_at(&stock, [x, y]).0 >= 5.99, "strip cut at {x} {y}");
            }
        }
        // The crossing cuts: 1.5 mm (half the strip) plus half a kerf from
        // the middle line, either side.
        for y in [53. - 2.25, 53. + 2.25] {
            assert!(top_at(&stock, [0.5 * (x0 + x1), y]).1 <= 0.01, "{y}");
        }
        // And the parts' facing edges are cut elsewhere along them.
        for y in [44., 62.] {
            assert!(top_at(&stock, [x0 + 0.75, y]).1 <= 0.01);
            assert!(top_at(&stock, [x1 - 0.75, y]).1 <= 0.01);
        }
    }
    // Pairs only.
    let pairs = serde_json::json!({"width_mm": 3., "max_length_mm": 12., "max_parts": 2});
    let (events, plan, _) = cut(serde_json::json!({"bridges": pairs})).unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    assert_eq!(pierces_and_links(&events).0, 5 + 3);
    assert_eq!(report["bridged_groups"], serde_json::json!([[0, 1], [2, 3]]));
    // Parts further apart than a bridge may span are cut on their own.
    let short = serde_json::json!({"width_mm": 3., "max_length_mm": 8., "max_parts": 8});
    let (events, plan, _) = cut(serde_json::json!({"bridges": short})).unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    assert_eq!(pierces_and_links(&events).0, 10);
    assert!(report.get("bridges").is_none() && report.get("bridged_groups").is_none());
    // With rapids kept off freed parts: the group comes free as one.
    let (_, plan, _) = cut(serde_json::json!({
        "bridges": bridges, "avoid_cut_parts": {"clearance_mm": 6.}
    }))
    .unwrap();
    assert_eq!(serde_json::to_value(plan.report()).unwrap()["pierces"], 6);
    // Refusals name the problem.
    let error = cut(serde_json::json!({"bridges": {"width_mm": 0., "max_length_mm": 12., "max_parts": 8}}))
        .err()
        .unwrap();
    assert!(error.contains("width_mm"), "{error}");
    let error = cut(serde_json::json!({"bridges": bridges, "common_line": {"min_length_mm": 5.}}))
        .err()
        .unwrap();
    assert!(error.contains("bridges are not planned together with common_line"), "{error}");
}

#[test]
fn the_skeleton_is_cut_up_after_the_parts_clear_of_them_and_never_over_a_hole() {
    let cut = |extra: serde_json::Value| {
        let mut op = operation(extra);
        op.parts = row();
        run(op)
    };
    // The row's five 26 mm squares stand at x = 10 + 36 k, y = 40..66, each
    // with a 5 mm hole at its middle, on a 200 x 120 sheet.
    let squares: Vec<[f64; 4]> =
        (0..5).map(|k| [10. + 36. * k as f64, 40., 36. + 36. * k as f64, 66.]).collect();
    let off = |p: [f64; 3], s: &[f64; 4]| {
        let dx = (s[0] - p[0]).max(p[0] - s[2]).max(0.);
        let dy = (s[1] - p[1]).max(p[1] - s[3]).max(0.);
        dx.hypot(dy)
    };
    let rules = serde_json::json!({
        "max_piece_mm": 70., "part_clearance_mm": 0., "run_off_mm": 2., "area_mm": [0., 0., 200., 120.]
    });
    let (plain_events, plain_plan, _) = cut(serde_json::json!({})).unwrap();
    let plain = serde_json::to_value(plain_plan.report()).unwrap();
    let (events, plan, stock) = cut(serde_json::json!({"skeleton": rules})).unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    // The parts are cut exactly as without it, first.
    assert_eq!(events[..plain_events.len() - 1], plain_events[..plain_events.len() - 1]);
    let lines: Vec<&Motion> = events[plain_events.len() - 1..]
        .iter()
        .filter_map(|e| match e {
            Event::Motion { motion } if motion.kind == MoveKind::Cut => Some(motion),
            _ => None,
        })
        .collect();
    let count = report["skeleton_cuts"].as_u64().unwrap() as usize;
    assert_eq!(lines.len(), count);
    assert!(count >= 4, "{count}");
    assert_eq!(report["pierces"].as_u64().unwrap(), plain["pierces"].as_u64().unwrap() + count as u64);
    let total: f64 = lines.iter().map(|m| length(&m.curve)).sum();
    assert!((report["skeleton_length_mm"].as_f64().unwrap() - total).abs() < 1e-9);
    assert!(
        (report["cut_length_mm"].as_f64().unwrap() - plain["cut_length_mm"].as_f64().unwrap() - total).abs()
            < 1e-6
    );
    // 200 wide in three columns, 120 high in two rows: lines at x = 200/3
    // and 400/3, and at y = 60, which runs through the parts.
    let third = 200. / 3.;
    for m in &lines {
        let (a, b) = (m.curve.start(), m.curve.end());
        let vertical = a[0] == b[0];
        assert!(vertical || a[1] == b[1], "{m:?}");
        if vertical {
            assert!((a[0] - third).abs() < 1e-9 || (a[0] - 2. * third).abs() < 1e-9, "{a:?}");
        } else {
            assert_eq!(a[1], 60.);
        }
        // Independently of the planner: every point of the line keeps the
        // kerf's half width off every part, the pierce is the crater's
        // reach from them, and no point is over a hole.
        assert!(squares.iter().all(|s| off(a, s) >= 0.75 + 1.5), "{a:?}");
        for k in 0..=400 {
            let t = k as f64 / 400.;
            let p = [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), 0.];
            assert!(squares.iter().all(|s| off(p, s) >= 0.75), "{p:?}");
        }
    }
    // A line that reaches the area's edge runs off it by the declared 2 mm;
    // one that ends at a part stops in that part's kerf.
    let ends: Vec<[f64; 3]> = lines.iter().map(|m| m.curve.end()).collect();
    assert!(ends.iter().any(|e| e[1] == 122. || e[1] == -2. || e[0] == 202. || e[0] == -2.), "{ends:?}");
    assert!(
        ends.iter().any(|e| squares.iter().any(|s| (0.74..1.0).contains(&off(*e, s)))),
        "{ends:?}"
    );
    // The lace between two parts is cut through at y = 60.
    assert!(top_at(&stock, [41., 60.]).1 <= 0.01);
    // With rapids kept off freed parts the cut-up still plans.
    let (avoiding, plan, _) = cut(serde_json::json!({
        "skeleton": rules, "avoid_cut_parts": {"clearance_mm": 4.}
    }))
    .unwrap();
    let mut fired = false;
    let mut cut_up = 0;
    for e in &avoiding[plain_events.len() - 1..] {
        match e {
            Event::Beam { power_percent } => fired = *power_percent > 0.,
            Event::Motion { motion } if motion.kind == MoveKind::Cut && std::mem::take(&mut fired) => {
                cut_up += 1;
                let p = motion.curve.start();
                assert!(squares.iter().all(|s| off(p, s) >= 4.), "{p:?}");
            }
            _ => {}
        }
    }
    assert!(cut_up >= 2);
    // Its pierces then keep that clearance from every part too.
    assert!(serde_json::to_value(plan.report()).unwrap()["skeleton_cuts"].as_u64().unwrap() >= 2);
    // Refusals name the problem.
    let error = cut(serde_json::json!({"skeleton": {
        "max_piece_mm": 0., "part_clearance_mm": 0., "run_off_mm": 2., "area_mm": [0., 0., 200., 120.]
    }}))
    .err()
    .unwrap();
    assert!(error.contains("max_piece_mm"), "{error}");
}

#[test]
fn edge_starts_begin_loops_in_open_material_without_a_pierce() {
    // A square near the sheet's bottom edge and one above it (the sheet is
    // 200 x 120): each has a hole.
    let squares = [[80., 12., 106., 38.], [80., 48., 106., 74.]];
    let parts = || PlanarRegion {
        components: squares
            .iter()
            .map(|s| Component {
                outer: rect(s[0], s[1], s[2], s[3]),
                holes: vec![circle([0.5 * (s[0] + s[2]), 0.5 * (s[1] + s[3])], 5.)],
            })
            .collect(),
        ..row()
    };
    let cut = |extra: serde_json::Value| {
        let mut op = operation(extra);
        op.parts = parts();
        run(op)
    };
    // What fires the beam, where, and the straight beam-on moves before it.
    let starts = |events: &[Event]| {
        let (mut pierces, mut open) = (0, vec![]);
        let mut at = [0.; 3];
        let mut kerfs: Vec<([f64; 3], [f64; 3])> = vec![];
        for e in events {
            match e {
                Event::Motion { motion } => {
                    if motion.kind != MoveKind::Rapid
                        && let Curve::Line { start, end } = motion.curve
                    {
                        kerfs.push((start, end));
                    }
                    at = motion.curve.end();
                }
                Event::Beam { power_percent } if *power_percent > 0. => pierces += 1,
                Event::BeamStart { .. } => open.push((at, kerfs.clone())),
                _ => {}
            }
        }
        (pierces, open)
    };
    let (events, plan, _) = cut(serde_json::json!({})).unwrap();
    let plain = serde_json::to_value(plan.report()).unwrap();
    assert_eq!(starts(&events).0, 4);
    assert!(plain.get("edge_starts").is_none());
    // From the sheet's edge only: the lower square is 12 mm from the bottom
    // edge, the upper one out of a 20 mm lead's reach of every edge.
    let edge = serde_json::json!({"run_on_mm": 3., "max_lead_mm": 20.});
    let (events, plan, _) = cut(serde_json::json!({"edge_starts": edge})).unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    let (pierces, open) = starts(&events);
    assert_eq!((pierces, open.len()), (3, 1));
    assert_eq!((report["pierces"].as_u64(), report["edge_starts"].as_u64()), (Some(3), Some(1)));
    // Independently: the beam fires 3 mm beyond the sheet's edge, at the
    // cut height, with no pierce delay; the dwells are one fewer.
    let p = open[0].0;
    let outside = (0. - p[0]).max(p[0] - 200.).max(0. - p[1]).max(p[1] - 120.);
    assert!((outside - 3.).abs() < 1e-9, "{p:?}");
    assert_eq!(p[2], 6. + 1.5);
    assert!(
        (plain["dwell_seconds"].as_f64().unwrap() - report["dwell_seconds"].as_f64().unwrap() - 0.5).abs() < 1e-9
    );
    // With starts over earlier kerfs too: the upper square starts on the
    // kerf along the lower one's top edge, cut before it.
    let both = serde_json::json!({"run_on_mm": 3., "max_lead_mm": 20., "from_kerfs": true, "delay_s": 0.2});
    let (events, plan, stock) = cut(serde_json::json!({"edge_starts": both})).unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    let (pierces, open) = starts(&events);
    assert_eq!((pierces, open.len()), (2, 2));
    assert_eq!(report["edge_starts"].as_u64(), Some(2));
    let (p, kerfs) = &open[1];
    assert!(p[0] > 0. && p[0] < 200. && p[1] > 38. && p[1] < 48., "{p:?}");
    // On the centre line of a straight kerf cut before it (independently:
    // the nearest earlier straight beam-on move passes through it).
    let nearest = kerfs
        .iter()
        .map(|(a, b)| to_segment([p[0], p[1]], [a[0], a[1]], [b[0], b[1]]))
        .fold(f64::INFINITY, f64::min);
    assert!(nearest < 1e-9, "{nearest}");
    // Each open start pauses for the declared motion delay.
    assert!(
        (report["dwell_seconds"].as_f64().unwrap() - (2. * 0.5 + 2. * 0.2)).abs() < 1e-9,
        "{}",
        report["dwell_seconds"]
    );
    // Both parts are still cut free and kept.
    for s in &squares {
        let y = 0.5 * (s[1] + s[3]);
        assert!(top_at(&stock, [s[0] - 0.75, y]).1 <= 0.01);
        assert!(top_at(&stock, [s[0] + 3., y]).0 >= 5.99);
    }
    // A lead longer than declared is not taken: everything is pierced.
    let short = serde_json::json!({"run_on_mm": 3., "max_lead_mm": 5., "from_kerfs": true});
    let (events, plan, _) = cut(serde_json::json!({"edge_starts": short})).unwrap();
    assert_eq!(starts(&events), (4, vec![]));
    assert_eq!(serde_json::to_value(plan.report()).unwrap()["edge_starts"].as_u64(), Some(0));
    // Refusals name the problem.
    let error = cut(serde_json::json!({"edge_starts": {"run_on_mm": 3., "max_lead_mm": 0.}}))
        .err()
        .unwrap();
    assert!(error.contains("edge_starts"), "{error}");
}

#[test]
fn edge_start_already_at_the_initial_position_precedes_the_first_cut() {
    let mut op = operation(serde_json::json!({
        "height_mode": "fixed", "pierce_height_mm": null,
        "travel_height_mm": null, "plunge_mm_min": null,
        "edge_starts": {"run_on_mm": 3., "max_lead_mm": 20., "delay_s": 0.2}
    }));
    op.parts = PlanarRegion {
        components: vec![Component { outer: rect(10., 12., 36., 38.), holes: vec![] }],
        ..row()
    };
    let (events, _, _) = run(op.clone()).unwrap();
    let mut at = [0.; 3];
    for e in events {
        match e {
            Event::Motion { motion } => at = motion.curve.end(),
            Event::BeamStart { .. } => break,
            _ => {}
        }
    }
    assert!(at[0] < 0. || at[1] < 0.);
    let (events, plan, _) = run_from(op, at).unwrap();
    assert_eq!(plan.report().dwell_seconds, 0.2);
    let start = events.iter().position(|e| matches!(e, Event::BeamStart { .. })).unwrap();
    let first = events.iter().position(|e| matches!(e, Event::Motion { .. })).unwrap();
    assert!(start < first);
    assert!(matches!(&events[first], Event::Motion { motion } if motion.kind != MoveKind::Rapid));
}

#[test]
fn edge_start_refuses_a_kerf_not_proven_open_by_stock() {
    let mut op = operation(serde_json::json!({
        "edge_starts": {"run_on_mm": 3., "max_lead_mm": 20., "from_kerfs": true}
    }));
    op.parts = PlanarRegion {
        components: [rect(80., 12., 106., 38.), rect(80., 48., 106., 74.)]
            .into_iter()
            .map(|outer| Component { outer, holes: vec![] })
            .collect(),
        ..row()
    };
    // A 1.5 mm kerf cannot empty a 10 mm stock cell. The planned earlier
    // path is not evidence that the authoritative stock is open there.
    let spec = StockSpec { cell_size_mm: 10., ..spec() };
    let cancel = AtomicBool::new(false);
    let tool = torch();
    let prepared = tool.prepare().unwrap();
    let mut stock = Stock::new(spec.clone()).unwrap();
    let mut plan = ThermalCutPlan::prepare(op, &tool, &spec, [0., 0., 40.], 1, &cancel).unwrap();
    let mut starts = 0;
    loop {
        match plan.next_motion_with_stock(&stock, &cancel) {
            Ok(Some(motion)) => {
                let delta = stock.preview_sweep(&prepared, &motion, 1., SweepOptions::default(), &cancel).unwrap();
                stock.apply_delta(&delta).unwrap();
                starts += plan.take_events().iter().filter(|e| matches!(e, Event::BeamStart { .. })).count();
            }
            Ok(None) => panic!("an unproven kerf start was accepted"),
            Err(error) => {
                assert!(error.contains("not cut through"), "{error}");
                assert_eq!(starts, 1, "only the off-sheet start may have been emitted");
                assert!(!plan.take_events().iter().any(|e| matches!(e, Event::BeamStart { .. })));
                break;
            }
        }
    }
}

// Lead shapes (#400): each measured from the planned moves with the test's
// own geometry, not the planner's.

/// The distance from p to the rectangle x0..x1, y0..y1 (0 inside it).
fn to_rect(p: [f64; 2], r: [f64; 4]) -> f64 {
    let dx = (r[0] - p[0]).max(p[0] - r[2]).max(0.);
    let dy = (r[1] - p[1]).max(p[1] - r[3]).max(0.);
    dx.hypot(dy)
}

/// Points along a line or an arc of the XY plane.
fn samples(c: &Curve) -> Vec<[f64; 2]> {
    (0..=64)
        .map(|k| {
            let f = k as f64 / 64.;
            match c {
                Curve::Line { start, end } => [
                    start[0] + f * (end[0] - start[0]),
                    start[1] + f * (end[1] - start[1]),
                ],
                Curve::Arc {
                    start,
                    center,
                    sweep_rad,
                    ..
                } => {
                    let (s, co) = (f * sweep_rad).sin_cos();
                    let u = [start[0] - center[0], start[1] - center[1]];
                    [
                        center[0] + co * u[0] - s * u[1],
                        center[1] + s * u[0] + co * u[1],
                    ]
                }
                Curve::Helix { .. } => unreachable!(),
            }
        })
        .collect()
}

/// The moves of each loop's lead-in and lead-out: the lead moves before its
/// first cut and after its last.
fn leads_by_loop(events: &[Event]) -> Vec<(Vec<Curve>, Vec<Curve>)> {
    let mut out = vec![];
    let (mut lead_in, mut cut, mut lead_out) = (vec![], false, vec![]);
    for e in events {
        match e {
            Event::Motion { motion } if motion.kind == MoveKind::Lead => {
                if cut {
                    lead_out.push(motion.curve.clone());
                } else {
                    lead_in.push(motion.curve.clone());
                }
            }
            Event::Motion { motion } if motion.kind == MoveKind::Cut => cut = true,
            Event::Beam { power_percent } if *power_percent == 0. && cut => {
                out.push((std::mem::take(&mut lead_in), std::mem::take(&mut lead_out)));
                cut = false;
            }
            _ => {}
        }
    }
    out
}

fn xy(p: [f64; 3]) -> [f64; 2] {
    [p[0], p[1]]
}
fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}
/// The direction of travel of a curve at its start (`end`: its end).
fn heading(c: &Curve, end: bool) -> [f64; 2] {
    match c {
        Curve::Line { start, end: e } => {
            let l = dist(xy(*start), xy(*e));
            [(e[0] - start[0]) / l, (e[1] - start[1]) / l]
        }
        Curve::Arc {
            start,
            end: e,
            center,
            sweep_rad,
            ..
        } => {
            let p = if end { e } else { start };
            let r = dist(xy(*p), xy(*center));
            let s = sweep_rad.signum();
            [-s * (p[1] - center[1]) / r, s * (p[0] - center[0]) / r]
        }
        Curve::Helix { .. } => unreachable!(),
    }
}
fn angle_deg(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] * b[0] + a[1] * b[1]).clamp(-1., 1.).acos().to_degrees()
}

const PLATE: [f64; 4] = [40., 40., 70., 60.];
/// The kerf path's offset: half a kerf and half the boundary tolerance.
const OFFSET: f64 = KERF / 2. + 0.005;

fn plate_region(extra: Vec<Boundary>) -> PlanarRegion {
    PlanarRegion {
        components: std::iter::once(rect(PLATE[0], PLATE[1], PLATE[2], PLATE[3]))
            .chain(extra)
            .map(|outer| Component { outer, holes: vec![] })
            .collect(),
        ..row()
    }
}

fn shaped(lead_in: serde_json::Value, lead_out: serde_json::Value, start: [f64; 2], extra: Vec<Boundary>, more: serde_json::Value) -> Result<Vec<Event>, String> {
    let mut values = serde_json::json!({
        "lead_in": lead_in, "lead_out": lead_out, "overcut_mm": 0.,
        "lead_placement": {"keep_off_distance_mm": 0., "starts": [start]},
        "small_holes": null,
    });
    for (k, v) in more.as_object().unwrap() {
        values[k] = v.clone();
    }
    let mut op = operation(values);
    op.parts = plate_region(extra);
    run(op).map(|(events, _, _)| events)
}

#[test]
fn each_lead_shape_lies_as_declared_and_in_the_scrap() {
    use serde_json::json;
    // The plate's outline is cut clockwise: along its bottom edge towards
    // -x, the scrap below. Its loop starts in the middle of that edge.
    let s = [55., PLATE[1] - OFFSET];
    let (t, n) = ([-1., 0.], [0., -1.]);
    let along = |p: [f64; 2]| [(p[0] - s[0]) * t[0] + (p[1] - s[1]) * t[1], (p[0] - s[0]) * n[0] + (p[1] - s[1]) * n[1]];
    let near = |a: f64, b: f64| (a - b).abs() < 1e-6;
    let cases = [
        // A line at 30 degrees in, at 60 out.
        (json!({"length_mm": 5., "angle_deg": 30.}), json!({"length_mm": 4., "angle_deg": 60.})),
        // Arcs of 45 and 150 degrees.
        (json!({"radius_mm": 6., "sweep_deg": 45.}), json!({"radius_mm": 3., "sweep_deg": 150.})),
        // Line then arc in, arc then line out.
        (json!({"length_mm": 3., "radius_mm": 2., "sweep_deg": 60.}), json!({"length_mm": 3., "radius_mm": 2., "sweep_deg": 120.})),
        // In line with the edge out: the overrun along the cut.
        (json!({"length_mm": 2., "radius_mm": 3., "sweep_deg": 90.}), json!({"length_mm": 5., "angle_deg": 0.})),
    ];
    for (lead_in, lead_out) in cases {
        let events = shaped(lead_in.clone(), lead_out.clone(), [55., 40.], vec![], json!({})).unwrap();
        let loops = leads_by_loop(&events);
        assert_eq!(loops.len(), 1);
        let (lin, lout) = &loops[0];
        // Every lead in the scrap: its kerf off the plate.
        for c in lin.iter().chain(lout) {
            for p in samples(c) {
                assert!(to_rect(p, PLATE) >= KERF / 2., "{lead_in} {lead_out}: {p:?}");
            }
        }
        // The lead-in ends where the cut starts, running along the cut.
        let last = lin.last().unwrap();
        assert!(dist(xy(last.end()), s) < 1e-9, "{:?}", last.end());
        assert!(angle_deg(heading(last, true), t) < 1e-6 || lead_in.get("angle_deg").is_some());
        let first = &lout[0];
        assert!(dist(xy(first.start()), s) < 1e-9);
        for (lead, moves, inward) in [(&lead_in, lin, true), (&lead_out, lout, false)] {
            let length = lead.get("length_mm").and_then(|v| v.as_f64()).unwrap_or(0.);
            let radius = lead.get("radius_mm").and_then(|v| v.as_f64()).unwrap_or(0.);
            if let Some(angle) = lead.get("angle_deg").and_then(|v| v.as_f64()) {
                assert_eq!(moves.len(), 1);
                let (a, b) = (xy(moves[0].start()), xy(moves[0].end()));
                assert!(near(dist(a, b), length));
                // Its angle to the edge's tangent, on the scrap side.
                let far = if inward { a } else { b };
                let [u, v] = along(far);
                assert!(near(v.atan2(u.abs()).to_degrees(), angle), "{lead}: {u} {v}");
                assert!(if inward { u <= 1e-9 } else { u >= -1e-9 });
                continue;
            }
            let sweep = lead["sweep_deg"].as_f64().unwrap();
            let arc = moves.iter().find(|c| matches!(c, Curve::Arc { .. })).unwrap();
            let Curve::Arc { start, end, center, sweep_rad, .. } = arc else { unreachable!() };
            assert!(near(dist(xy(*start), xy(*center)), radius) && near(dist(xy(*end), xy(*center)), radius));
            assert!(near(sweep_rad.abs().to_degrees(), sweep), "{lead}");
            // Tangent to the cut, its centre on the scrap side.
            let [u, v] = along(xy(*center));
            assert!(near(u, 0.) && near(v, radius));
            let lines: Vec<&Curve> = moves.iter().filter(|c| matches!(c, Curve::Line { .. })).collect();
            assert_eq!(lines.len(), usize::from(length > 0.));
            if let Some(line) = lines.first() {
                assert!(near(dist(xy(line.start()), xy(line.end())), length));
                // Tangent to the arc where they meet.
                let (h_line, h_arc) = if inward {
                    (heading(line, false), heading(arc, false))
                } else {
                    (heading(line, false), heading(arc, true))
                };
                assert!(angle_deg(h_line, h_arc) < 1e-6, "{lead}");
            }
        }
    }
}

#[test]
fn in_line_leads_at_a_corner_run_along_both_edges_extensions() {
    use serde_json::json;
    let line = |l: f64| json!({"length_mm": l, "angle_deg": 0.});
    // The corner (70, 40): the outline, clockwise, comes down the right
    // edge and leaves along the bottom edge. A neighbour lies below and to
    // the right, clear of the leads' reach.
    let neighbour = [72.5, 10., 100., 36.];
    let events = shaped(line(6.), line(6.), [70., 40.], vec![rect(neighbour[0], neighbour[1], neighbour[2], neighbour[3])], json!({})).unwrap();
    let loops = leads_by_loop(&events);
    // The plate's outline: its loop starts at the corner.
    let (lin, lout) = loops
        .iter()
        .find(|(lin, _)| lin.iter().any(|c| xy(c.end())[0] < 71.))
        .unwrap();
    assert_eq!((lin.len(), lout.len()), (1, 1));
    let (p, s) = (xy(lin[0].start()), xy(lin[0].end()));
    let (e, q) = (xy(lout[0].start()), xy(lout[0].end()));
    let y = PLATE[1] - OFFSET;
    let x = PLATE[2] + OFFSET;
    // The lead-in comes in along the bottom edge's extension, from beyond
    // the corner, onto the bottom edge.
    assert!((p[1] - y).abs() < 1e-9 && (s[1] - y).abs() < 1e-9, "{p:?} {s:?}");
    assert!((s[0] - PLATE[2]).abs() < 1e-9 && (p[0] - (PLATE[2] + 6.)).abs() < 1e-9);
    // The lead-out runs on down the right edge's extension past the corner.
    assert!((e[0] - x).abs() < 1e-9 && (q[0] - x).abs() < 1e-9, "{e:?} {q:?}");
    assert!((e[1] - PLATE[1]).abs() < 1e-9 && (q[1] - (PLATE[1] - 6.)).abs() < 1e-9);
    // The two kerfs cross at the kerf path's corner (x, y), each running on
    // more than half a kerf past it: the corner is cut through, and no arc
    // round it is cut.
    assert!(p[0] - x >= KERF / 2. && y - q[1] >= KERF / 2.);
    let corner_arcs = events.iter().filter(|e| matches!(e, Event::Motion { motion } if motion.kind == MoveKind::Cut && matches!(motion.curve, Curve::Arc { center, .. } if (center[0] - PLATE[2]).abs() < 1e-9 && (center[1] - PLATE[1]).abs() < 1e-9))).count();
    assert_eq!(corner_arcs, 0);
    // Both clear of the neighbour and of the plate.
    for c in [&lin[0], &lout[0]] {
        for point in samples(c) {
            assert!(to_rect(point, neighbour) >= KERF / 2. && to_rect(point, PLATE) >= KERF / 2. - 1e-9);
        }
    }
    // A neighbour inside the lead-in's reach, beyond the corner: refused,
    // named.
    let error = shaped(line(6.), line(6.), [70., 40.], vec![rect(74., 30., 100., 50.)], json!({})).unwrap_err();
    assert!(error.contains("declared start (70.000, 40.000)") && error.contains("not clear"), "{error}");
    // Leads too short to cross: refused, naming the lengths needed.
    let error = shaped(line(1.), line(6.), [70., 40.], vec![], json!({})).unwrap_err();
    assert!(error.contains("half a kerf past") && error.contains("1.505"), "{error}");
    // The overrun is the lead-out's: no overcut with it.
    let error = shaped(line(6.), line(6.), [70., 40.], vec![], json!({"overcut_mm": 1.})).unwrap_err();
    assert!(error.contains("overcut_mm"), "{error}");
    // An in-line lead-in alone at the corner: along the bottom edge's
    // extension, and the corner's arc is cut before the lead-out leaves.
    let events = shaped(line(6.), json!({"length_mm": 1., "radius_mm": 2.}), [70., 40.], vec![], json!({})).unwrap();
    let (lin, lout) = &leads_by_loop(&events)[0];
    assert!((xy(lin[0].start())[1] - y).abs() < 1e-9);
    assert!(dist(xy(lout[0].start()), xy(lin[0].end())) < 1e-9);
}

#[test]
fn lead_shapes_need_their_parameters_and_compensation_its_straight_part() {
    use serde_json::json;
    let plain = json!({"length_mm": 2., "radius_mm": 2.});
    for (lead_in, more, message) in [
        (json!({"length_mm": 3., "angle_deg": 120.}), json!({}), "angle_deg"),
        (json!({"length_mm": 3., "radius_mm": 1., "angle_deg": 30.}), json!({}), "no radius_mm"),
        (json!({"radius_mm": 0., "sweep_deg": 30.}), json!({}), "sweep_deg"),
        (json!({"radius_mm": 2., "sweep_deg": 200.}), json!({}), "at most 180"),
        // Wear: G41 goes on over a straight lead-in.
        (json!({"radius_mm": 2., "sweep_deg": 90.}), json!({"compensation": "wear"}), "Lead.arc has none"),
        // Control: the written lead runs tangent into the cut.
        (json!({"length_mm": 3., "angle_deg": 45.}), json!({"compensation": "control"}), "Lead.line meets the cut"),
        // In line with a straight edge, the pierce would be on the kerf
        // path, against the part: refused (in line belongs at a corner).
        (json!({"length_mm": 5., "angle_deg": 0.}), json!({}), "pierce point"),
    ] {
        let error = shaped(lead_in, plain.clone(), [55., 40.], vec![], more).unwrap_err();
        assert!(error.contains(message), "{message}: {error}");
    }
    // In-line leads at a corner cross each other: not with wear.
    let line = json!({"length_mm": 6., "angle_deg": 0.});
    let error = shaped(line.clone(), line.clone(), [70., 40.], vec![], json!({"compensation": "wear"})).unwrap_err();
    assert!(error.contains("cutter compensation"), "{error}");
    // Wear with straight leads at an angle: G41 on over the lead-in, G40
    // before the lead-out.
    let events = shaped(json!({"length_mm": 4., "angle_deg": 45.}), json!({"length_mm": 3., "angle_deg": 60.}), [55., 40.], vec![], json!({"compensation": "wear"})).unwrap();
    let comp: Vec<_> = events.iter().filter_map(|e| match e { Event::CutterCompensation { side, .. } => Some(*side), _ => None }).collect();
    assert_eq!(comp, [CompensationSide::Left, CompensationSide::Off]);
}

#[test]
fn controller_kerf_lead_arcs_are_cut_in_quarter_turns() {
    use serde_json::json;
    let lead = json!({"length_mm": 3., "radius_mm": 2., "sweep_deg": 150.});
    let events = shaped(lead.clone(), lead, [55., 40.], vec![], json!({"compensation": "control"})).unwrap();
    let (lin, lout) = &leads_by_loop(&events)[0];
    for moves in [lin, lout] {
        let arcs: Vec<f64> = moves.iter().filter_map(|c| match c { Curve::Arc { sweep_rad, .. } => Some(sweep_rad.abs()), _ => None }).collect();
        assert_eq!(arcs.len(), 2);
        assert!(arcs.iter().all(|w| (w.to_degrees() - 75.).abs() < 1e-9), "{arcs:?}");
    }
}
