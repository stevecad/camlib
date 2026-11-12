//! Common-line cutting: two parts one kerf path apart share one cut. Checked
//! from the produced motions and the simulated sheet alone: every part edge
//! has a kerf exactly the applied offset beside it, no kerf enters a part,
//! nothing is cut twice, pierces and leads stay in the scrap, and the common
//! lines are cut before the outline that frees the parts.
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
/// Half the kerf plus half the default boundary tolerance (0.01 mm).
const OFFSET: f64 = KERF / 2. + 0.005;
/// Parts this far apart have one kerf path between them.
const SPACING: f64 = 2. * OFFSET;
type Rect = [f64; 4];

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

fn region(rects: &[Rect]) -> PlanarRegion {
    PlanarRegion {
        version: 1,
        units: "mm".into(),
        name: "parts".into(),
        boundary_error_mm: 0.,
        components: rects
            .iter()
            .map(|&[x0, y0, x1, y1]| Component {
                outer: Boundary {
                    vertices: vec![[x0, y0, 0.], [x1, y0, 0.], [x1, y1, 0.], [x0, y1, 0.]],
                },
                holes: vec![],
            })
            .collect(),
        source: serde_json::Value::Null,
    }
}

fn operation(rects: &[Rect], extra: serde_json::Value) -> ThermalCutOperation {
    let mut value = serde_json::json!({
        "id": "plasma", "tool": "torch", "parts": region(rects), "feed_mm_min": 2500.,
        "cut_height_mm": 1.5, "height_mode": "program", "pierce_height_mm": 3.8,
        "travel_height_mm": 10., "plunge_mm_min": 1000., "pierce_delay_s": 0.5,
        "lead_in": {"length_mm": 3., "radius_mm": 3.}, "lead_out": {"length_mm": 1., "radius_mm": 2.},
        "height_control": true, "common_line": {"min_length_mm": 5.},
        "tolerance_mm": 1e-5, "work_offset": 1, "tool_number": 1, "length_register": 1
    });
    for (k, v) in extra.as_object().unwrap() {
        value[k] = v.clone();
    }
    serde_json::from_value(value).unwrap()
}

fn run(op: ThermalCutOperation) -> Result<(Vec<Event>, ThermalCutPlan, Stock), String> {
    let cancel = AtomicBool::new(false);
    let tool = torch();
    let prepared = tool.prepare().unwrap();
    let mut stock = Stock::new(spec()).unwrap();
    stock.enable_contact_queries(&cancel).unwrap();
    let initial = [0., 0., 40.];
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
    ProgramValidator::new(&program, None)
        .unwrap()
        .consume(&events)
        .unwrap();
    Ok((events, plan, stock))
}

fn refusal(rects: &[Rect], extra: serde_json::Value) -> String {
    run(operation(rects, extra)).err().expect("refused")
}

fn solid(stock: &Stock, p: [f64; 2]) -> bool {
    let s = stock.spec();
    let nx = ((s.max_mm[0] - s.min_mm[0]) / s.cell_size_mm).round() as usize;
    let i = ((p[0] - s.min_mm[0]) / s.cell_size_mm) as usize;
    let j = ((p[1] - s.min_mm[1]) / s.cell_size_mm) as usize;
    let c = stock.cells()[j * nx + i];
    // The sheet is whole or cut through, never in between.
    assert!(
        (c.lower_mm, c.upper_mm) == (0., 0.) || (c.lower_mm, c.upper_mm) == (6., 6.),
        "{p:?}: {c:?}"
    );
    c.upper_mm == 6.
}

/// Distance from a point to a rectangle (0 inside).
fn to_rect(p: [f64; 2], r: Rect) -> f64 {
    let dx = (r[0] - p[0]).max(p[0] - r[2]).max(0.);
    let dy = (r[1] - p[1]).max(p[1] - r[3]).max(0.);
    dx.hypot(dy)
}

/// Points along an XY move, at most `step` apart (both ends included).
fn points(c: &Curve, step: f64) -> Vec<[f64; 2]> {
    match *c {
        Curve::Line { start, end } => {
            let n = ((start[0] - end[0]).hypot(start[1] - end[1]) / step).ceil() as usize + 1;
            (0..=n)
                .map(|k| {
                    let t = k as f64 / n as f64;
                    [
                        start[0] + t * (end[0] - start[0]),
                        start[1] + t * (end[1] - start[1]),
                    ]
                })
                .collect()
        }
        Curve::Arc {
            start,
            center,
            sweep_rad,
            ..
        } => {
            let r = (start[0] - center[0]).hypot(start[1] - center[1]);
            let a0 = (start[1] - center[1]).atan2(start[0] - center[0]);
            let n = (r * sweep_rad.abs() / step).ceil() as usize + 1;
            (0..=n)
                .map(|k| {
                    let a = a0 + sweep_rad * k as f64 / n as f64;
                    [center[0] + r * a.cos(), center[1] + r * a.sin()]
                })
                .collect()
        }
        Curve::Helix { .. } => panic!("no helix in a thermal cut"),
    }
}

/// Distance from a point to a straight move.
fn to_segment(p: [f64; 2], a: Vec3, b: Vec3) -> f64 {
    let (vx, vy) = (b[0] - a[0], b[1] - a[1]);
    let t = (((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / (vx * vx + vy * vy)).clamp(0., 1.);
    (p[0] - a[0] - t * vx).hypot(p[1] - a[1] - t * vy)
}

fn moves(events: &[Event], kind: MoveKind) -> Vec<&Motion> {
    events
        .iter()
        .filter_map(|e| match e {
            Event::Motion { motion } if motion.kind == kind => Some(motion),
            _ => None,
        })
        .collect()
}

/// Everything a cut sheet has to satisfy, from the motions and the stock.
fn check_sheet(rects: &[Rect], events: &[Event], stock: &Stock) {
    let cuts = moves(events, MoveKind::Cut);
    let leads = moves(events, MoveKind::Lead);
    // No kerf or lead enters a part: the beam center keeps the applied
    // offset from every part (so half a kerf with the boundary allowance).
    for m in cuts.iter().chain(&leads) {
        for p in points(&m.curve, 0.05) {
            for &r in rects {
                assert!(to_rect(p, r) >= OFFSET - 1e-9, "{m:?} at {p:?} near {r:?}");
            }
        }
    }
    // Each part is cut to size all the way round: every point of its edges
    // has a straight cut exactly the applied offset beside it. On a common
    // line that is the same cut for both parts.
    let lines: Vec<(Vec3, Vec3)> = cuts
        .iter()
        .filter_map(|m| match m.curve {
            Curve::Line { start, end } => Some((start, end)),
            _ => None,
        })
        .collect();
    for &[x0, y0, x1, y1] in rects {
        let corners = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];
        for k in 0..4 {
            let (a, b) = (corners[k], corners[(k + 1) % 4]);
            for i in 0..=200 {
                let t = i as f64 / 200.;
                let p = [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
                let nearest = lines
                    .iter()
                    .map(|(s, e)| to_segment(p, *s, *e))
                    .fold(f64::INFINITY, f64::min);
                assert!((nearest - OFFSET).abs() <= 1e-6, "{p:?}: {nearest}");
            }
        }
    }
    // Nothing is cut twice: no two straight cuts overlap along a length.
    for (i, (a0, a1)) in lines.iter().enumerate() {
        for (b0, b1) in &lines[i + 1..] {
            let length = (a1[0] - a0[0]).hypot(a1[1] - a0[1]);
            let u = [(a1[0] - a0[0]) / length, (a1[1] - a0[1]) / length];
            let across = |p: &Vec3| ((p[0] - a0[0]) * u[1] - (p[1] - a0[1]) * u[0]).abs();
            let along = |p: &Vec3| (p[0] - a0[0]) * u[0] + (p[1] - a0[1]) * u[1];
            if across(b0) < 1e-6 && across(b1) < 1e-6 {
                let (t0, t1) = (along(b0).min(along(b1)), along(b0).max(along(b1)));
                assert!(
                    t1.min(length) - t0.max(0.) <= 1e-6,
                    "{a0:?}-{a1:?} and {b0:?}-{b1:?} overlap"
                );
            }
        }
    }
    // No redundant vertex: two straight cuts in a row are never in line
    // (the outline runs across the end of a common line as one move).
    let mut before: Option<(Vec3, Vec3)> = None;
    for e in events {
        let Event::Motion { motion } = e else {
            before = None;
            continue;
        };
        let now = match motion.curve {
            Curve::Line { start, end } if motion.kind == MoveKind::Cut => Some((start, end)),
            _ => None,
        };
        if let (Some((a, b)), Some((c, d))) = (before, now) {
            let turn = (b[0] - a[0]) * (d[1] - c[1]) - (b[1] - a[1]) * (d[0] - c[0]);
            assert!(turn.abs() > 1e-9, "{a:?}-{b:?} and {c:?}-{d:?} are one line");
        }
        before = now;
    }
    // The simulated sheet: part material stays, and the sheet is cut through
    // along the kerf path all round every part (it is free).
    for &[x0, y0, x1, y1] in rects {
        let (nx, ny) = ((x1 - x0) as usize * 2, (y1 - y0) as usize * 2);
        for i in 0..=nx {
            for j in 0..=ny {
                let p = [
                    x0 + 0.3 + (x1 - x0 - 0.6) * i as f64 / nx as f64,
                    y0 + 0.3 + (y1 - y0 - 0.6) * j as f64 / ny as f64,
                ];
                assert!(solid(stock, p), "part material cut at {p:?}");
            }
        }
        let around = Boundary {
            vertices: vec![[x0, y0, 0.], [x1, y0, 0.], [x1, y1, 0.], [x0, y1, 0.]],
        };
        for k in 0..4 {
            let (a, b) = (around.vertices[k], around.vertices[(k + 1) % 4]);
            let length = (b[0] - a[0]).hypot(b[1] - a[1]);
            let u = [(b[0] - a[0]) / length, (b[1] - a[1]) / length];
            // Counter-clockwise outline: the scrap is on the right.
            let n = [u[1], -u[0]];
            for i in 0..=(length * 4.) as usize {
                let t = i as f64 / 4.;
                let p = [
                    a[0] + t * u[0] + OFFSET * n[0],
                    a[1] + t * u[1] + OFFSET * n[1],
                ];
                assert!(!solid(stock, p), "not cut through at {p:?}");
            }
            // Round the corner at b.
            for i in 0..=8 {
                let angle = std::f64::consts::FRAC_PI_2 * i as f64 / 8.;
                let p = [
                    b[0] + OFFSET * (n[0] * angle.cos() + u[0] * angle.sin()),
                    b[1] + OFFSET * (n[1] * angle.cos() + u[1] * angle.sin()),
                ];
                assert!(!solid(stock, p), "corner not cut free at {p:?}");
            }
        }
    }
    // Every pierce (where the beam turns on) keeps the pierce clearance
    // (one kerf by default) beyond half a kerf from every part.
    let mut at = [0., 0.];
    let mut pierces = 0;
    for e in events {
        match e {
            Event::Motion { motion } => {
                let end = motion.curve.end();
                at = [end[0], end[1]];
            }
            Event::Beam { power_percent } if *power_percent > 0. => {
                pierces += 1;
                for &r in rects {
                    assert!(
                        to_rect(at, r) >= KERF / 2. + KERF - 1e-6,
                        "pierce {at:?} near {r:?}"
                    );
                }
            }
            _ => {}
        }
    }
    assert!(pierces > 0);
}

/// The index of the first cut move along the vertical line x.
fn first_cut_at_x(events: &[Event], x: f64) -> usize {
    events
        .iter()
        .position(|e| {
            matches!(e, Event::Motion { motion } if motion.kind == MoveKind::Cut
            && (motion.curve.start()[0] - x).abs() < 1e-6 && (motion.curve.end()[0] - x).abs() < 1e-6)
        })
        .expect("a cut along the common line")
}

const A: Rect = [20., 20., 60., 50.];
fn right_of(r: Rect, gap: f64) -> Rect {
    [r[2] + gap, r[1], r[2] + gap + (r[2] - r[0]), r[3]]
}

#[test]
fn two_rectangles_a_kerf_apart_share_one_cut() {
    let rects = [A, right_of(A, SPACING)];
    let (events, plan, stock) = run(operation(&rects, serde_json::json!({}))).unwrap();
    let r = plan.report();
    assert_eq!(r.common_lines.len(), 1, "{:?}", r.diagnostics);
    let line = &r.common_lines[0];
    assert_eq!(line.parts, [0, 1]);
    assert!((line.shared_length_mm - 30.).abs() < 1e-9);
    // One pierce for the common line and one for the outline of the pair.
    assert_eq!((r.loops, r.pierces, r.loops_completed), (1, 2, 1));
    let on = events
        .iter()
        .filter(|e| matches!(e, Event::Beam { power_percent } if *power_percent > 0.))
        .count();
    assert_eq!(on, 2);
    check_sheet(&rects, &events, &stock);
    // The common line runs between the outline's top and bottom, and is the
    // first cut: both parts are still held by the sheet all round.
    let x = 60. + OFFSET;
    let first = first_cut_at_x(&events, x);
    assert!(
        events[..first]
            .iter()
            .all(|e| !matches!(e, Event::Motion { motion } if motion.kind == MoveKind::Cut))
    );
    let Event::Motion { motion } = &events[first] else {
        unreachable!()
    };
    let (y0, y1) = (motion.curve.start()[1], motion.curve.end()[1]);
    assert!((y0.min(y1) - (20. - OFFSET)).abs() < 1e-6 && (y0.max(y1) - (50. + OFFSET)).abs() < 1e-6);
    // Its lead-in is straight, along the line, from a pierce beyond the end.
    let leads = moves(&events, MoveKind::Lead);
    assert!((leads[0].curve.start()[0] - x).abs() < 1e-6 && (leads[0].curve.end()[0] - x).abs() < 1e-6);
    // The cut length is the outline of the pair plus the one common line.
    let outline = 2. * (80. + SPACING) + 2. * 30. + 2. * std::f64::consts::PI * OFFSET;
    let common = 30. + 2. * OFFSET;
    assert!((r.cut_length_mm - outline - common).abs() < 1e-6, "{}", r.cut_length_mm);
}

#[test]
fn common_line_is_opt_in_and_exact() {
    let rects = [A, right_of(A, SPACING)];
    // Without the option the parts are too close to cut: nothing changes.
    let err = refusal(&rects, serde_json::json!({"common_line": null}));
    assert!(err.contains("topology"), "{err}");
    // Exactly the kerf apart is not the spacing (the kerf path keeps the
    // boundary allowance from both parts): refused, naming the spacing.
    let err = refusal(&[A, right_of(A, KERF)], serde_json::json!({}));
    assert!(err.contains("1.51 mm apart"), "{err}");
    // Further apart: cut separately as always, and reported.
    let apart = [A, right_of(A, 6.)];
    let (events, plan, stock) = run(operation(&apart, serde_json::json!({}))).unwrap();
    assert!(plan.report().common_lines.is_empty());
    assert_eq!(plan.report().pierces, 2);
    assert!(plan.report().diagnostics.iter().any(|d| d.contains("nothing is shared")));
    check_sheet(&apart, &events, &stock);
    // The same parts without the option give the same motions.
    let (plain, _, _) = run(operation(&apart, serde_json::json!({"common_line": null}))).unwrap();
    assert_eq!(
        serde_json::to_value(&events).unwrap(),
        serde_json::to_value(&plain).unwrap()
    );
}

#[test]
fn a_row_of_three_shares_two_cuts() {
    let b = right_of(A, SPACING);
    let rects = [A, b, right_of(b, SPACING)];
    let (events, plan, stock) = run(operation(&rects, serde_json::json!({}))).unwrap();
    let r = plan.report();
    assert_eq!(r.common_lines.len(), 2, "{:?}", r.diagnostics);
    assert_eq!((r.loops, r.pierces), (1, 3));
    assert_eq!(r.order, vec![(0, None)]);
    check_sheet(&rects, &events, &stock);
    // Both common lines before the outline.
    let cuts = moves(&events, MoveKind::Cut);
    for m in &cuts[..2] {
        assert!((m.curve.start()[0] - m.curve.end()[0]).abs() < 1e-9, "{m:?}");
    }
    // Reversed direction (parts on the left of travel) is the same sheet.
    let (events, _, stock) = run(operation(&rects, serde_json::json!({"reverse": true}))).unwrap();
    check_sheet(&rects, &events, &stock);
}

#[test]
fn holes_come_before_the_common_line_and_the_outline_last() {
    let b = right_of(A, SPACING);
    let mut parts = region(&[A, b]);
    for (c, r) in parts.components.iter_mut().zip([A, b]) {
        let (x, y) = ((r[0] + r[2]) / 2., (r[1] + r[3]) / 2.);
        c.holes = vec![Boundary {
            vertices: vec![[x + 6., y, 1.], [x - 6., y, 1.]],
        }];
    }
    let mut op = operation(&[A, b], serde_json::json!({}));
    op.parts = parts;
    let (events, plan, _) = run(op).unwrap();
    let r = plan.report();
    assert_eq!(r.order, vec![(0, Some(0)), (1, Some(0)), (0, None)]);
    assert_eq!((r.loops, r.pierces, r.common_lines.len()), (3, 4, 1));
    // Cut moves in order: hole arcs, the common line, then the outline.
    let cuts = moves(&events, MoveKind::Cut);
    let common = cuts
        .iter()
        .position(|m| {
            (m.curve.start()[0] - 60. - OFFSET).abs() < 1e-6
                && (m.curve.end()[0] - 60. - OFFSET).abs() < 1e-6
        })
        .unwrap();
    assert!(common >= 2);
    assert!(
        cuts[..common]
            .iter()
            .all(|m| matches!(m.curve, Curve::Arc { .. }))
    );
    // Everything after it is the outline: no hole arc (radius 6 less the
    // offset) follows.
    for m in &cuts[common + 1..] {
        if let Curve::Arc { start, center, .. } = m.curve {
            let radius = (start[0] - center[0]).hypot(start[1] - center[1]);
            assert!((radius - OFFSET).abs() < 1e-6, "{m:?}");
        }
    }
}

#[test]
fn a_lead_never_crosses_a_kerf_cut_before_it() {
    // Strips 4 mm wide: the outline's lead-in passes the common line's
    // lead-in kerf with sheet between them.
    let strips = |w: f64| [[20., 20., 20. + w, 50.], [20. + w + SPACING, 20., 20. + 2. * w + SPACING, 50.]];
    let (events, _, stock) = run(operation(&strips(4.), serde_json::json!({}))).unwrap();
    check_sheet(&strips(4.), &events, &stock);
    // Strips 2 mm wide: the same lead-in would run along that open kerf
    // (its pierce is still in uncut sheet, clear of both parts).
    let err = refusal(&strips(2.), serde_json::json!({}));
    assert!(
        err.contains("lead-in of the outline of parts [0, 1]") && err.contains("kerf cut before it"),
        "{err}"
    );
    // So would a lead-out whose arc swings back under the common line's
    // lead-in kerf.
    let err = refusal(
        &strips(4.),
        serde_json::json!({"lead_out": {"length_mm": 1., "radius_mm": 5.}, "reverse": true}),
    );
    assert!(
        err.contains("lead-out of the outline of parts [0, 1]") && err.contains("kerf cut before it"),
        "{err}"
    );
}

/// A valid pair with a common line, and beside it a disc in the round bite
/// of a fourth part, their curved edges `gap` apart.
fn pair_and_curved(gap: f64) -> PlanarRegion {
    let mut parts = region(&[A, right_of(A, SPACING)]);
    let radius = 450f64.sqrt();
    let disc = Boundary {
        // It starts on the far side from the bite.
        vertices: vec![[40. - radius + gap, 85., 1.], [40. + radius - gap, 85., 1.]],
    };
    // Its left edge is an arc about the disc's center (40, 85).
    let bite = Boundary {
        vertices: vec![
            [55., 70., 0.],
            [90., 70., 0.],
            [90., 100., 0.],
            [55., 100., -(std::f64::consts::PI / 8.).tan()],
        ],
    };
    for outer in [disc, bite] {
        parts.components.push(Component {
            outer,
            holes: vec![],
        });
    }
    parts
}

#[test]
fn kerf_paths_stay_apart_outside_the_common_line() {
    let plan = |gap: f64| {
        let mut op = operation(&[A], serde_json::json!({}));
        op.parts = pair_and_curved(gap);
        run(op).map(|(_, plan, _)| plan.report().common_lines.len())
    };
    // A curved stretch closer than one kerf path: the two kerf paths have
    // changed sides without coinciding anywhere. Refused, as the offset of
    // the whole region refuses it in a job without common lines.
    let err = plan(SPACING - 0.004).err().unwrap();
    assert!(
        err.contains("kerf path of part 2 runs") && err.contains("outside a common line"),
        "{err}"
    );
    // Exactly one kerf path apart along the arc: not a common line either.
    let err = plan(SPACING).err().unwrap();
    assert!(err.contains("along an arc"), "{err}");
    // Further apart the curved edges are cut separately, the pair shared.
    assert_eq!(plan(SPACING + 0.4), Ok(1));
}

#[test]
fn what_is_not_exactly_a_common_line_is_refused() {
    let b = right_of(A, SPACING);
    // A shorter neighbour shares only part of the edge.
    let err = refusal(&[A, [b[0], 20., b[2], 40.]], serde_json::json!({}));
    assert!(err.contains("flush") || err.contains("complete edge"), "{err}");
    // The same height, shifted along the edge.
    let err = refusal(&[A, [b[0], 25., b[2], 55.]], serde_json::json!({}));
    assert!(err.contains("complete edge"), "{err}");
    // A 2 x 2 grid: common lines meet at the middle.
    let up = |r: Rect| [r[0], r[3] + SPACING, r[2], r[3] + SPACING + 30.];
    let err = refusal(&[A, b, up(A), up(b)], serde_json::json!({}));
    assert!(err.contains("grid"), "{err}");
    // Shorter than the declared minimum.
    let err = refusal(&[A, b], serde_json::json!({"common_line": {"min_length_mm": 31.}}));
    assert!(err.contains("min_length_mm"), "{err}");
    // The controller cannot trim a kerf with a part on both sides.
    let err = refusal(&[A, b], serde_json::json!({"compensation": "wear"}));
    assert!(err.contains("compensation"), "{err}");
    let err = refusal(
        &[A, b],
        serde_json::json!({"micro_joints": {"width_mm": 1., "count": 2}}),
    );
    assert!(err.contains("micro_joints"), "{err}");
    // No lead-in: the pierce would sit at the end of the line, beside both
    // parts.
    let err = refusal(
        &[A, b],
        serde_json::json!({"lead_in": {"length_mm": 0., "radius_mm": 3.}}),
    );
    assert!(err.contains("lengthen lead_in"), "{err}");
}
