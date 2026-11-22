//! ThermalCut(compensation="control") (#384): the program carries the part
//! side and the controller applies the whole kerf. Checked here without the
//! planner's geometry: the planned beam path is written as its part side by
//! this test, posted to inch words at four decimals, read back and offset as
//! the controller does (`controller_offset::replay`), and the result must be
//! the planned path within the posting resolution, with both of the
//! controller's corner joins clear of the parts.
use camlib::{
    controller_offset,
    motion::*,
    planar::{Boundary, Component, PlanarRegion},
    stock::*,
    thermal::{ThermalCutOperation, ThermalCutPlan},
    tool::{Cutter, CutterShape, HolderSection, ToolAssembly},
};
use std::sync::atomic::AtomicBool;

const KERF: f64 = 1.6;
const R: f64 = KERF / 2.;
/// The boundary margin the kerf path keeps beyond half the kerf (half the
/// default boundary tolerance; the parts are exact).
const MARGIN: f64 = 0.005;
const INCH: f64 = 25.4;

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

fn polygon(points: &[[f64; 2]]) -> Boundary {
    Boundary {
        vertices: points.iter().map(|p| [p[0], p[1], 0.]).collect(),
    }
}
fn circle(c: [f64; 2], r: f64) -> Boundary {
    Boundary {
        vertices: vec![[c[0] + r, c[1], 1.], [c[0] - r, c[1], 1.]],
    }
}
fn region(components: Vec<Component>) -> PlanarRegion {
    PlanarRegion {
        version: 1,
        units: "mm".into(),
        name: "parts".into(),
        boundary_error_mm: 0.,
        components,
        source: serde_json::Value::Null,
    }
}
const PLATE: [[f64; 2]; 4] = [[20., 20.], [120., 20.], [120., 80.], [20., 80.]];
const HOLE: ([f64; 2], f64) = ([70., 50.], 10.);
/// A 100 x 60 mm plate with a 20 mm hole.
fn plate() -> PlanarRegion {
    region(vec![Component {
        outer: polygon(&PLATE),
        holes: vec![circle(HOLE.0, HOLE.1)],
    }])
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

fn operation(parts: PlanarRegion, extra: serde_json::Value) -> ThermalCutOperation {
    let mut value = serde_json::json!({
        "id": "plasma", "tool": "torch", "parts": parts, "feed_mm_min": 2500.,
        "cut_height_mm": 1.5, "pierce_delay_s": 0.,
        "lead_in": {"length_mm": 3., "radius_mm": 2.}, "lead_out": {"length_mm": 1.5, "radius_mm": 1.5},
        "compensation": "control",
        "tolerance_mm": 1e-5, "work_offset": 1, "tool_number": 1, "length_register": 1
    });
    for (k, v) in extra.as_object().unwrap() {
        value[k] = v.clone();
    }
    serde_json::from_value(value).unwrap()
}

fn plan(op: ThermalCutOperation) -> Result<Vec<Event>, String> {
    let cancel = AtomicBool::new(false);
    let tool = torch();
    let prepared = tool.prepare().unwrap();
    let mut stock = Stock::new(spec()).unwrap();
    let initial = [0., 0., 7.5];
    let mut plan = ThermalCutPlan::prepare(op, &tool, &spec(), initial, 1, &cancel)?;
    let mut events = plan.prefix_events();
    events.extend(plan.take_events());
    while let Some(m) = plan.next_motion_with_stock(&stock, &cancel)? {
        let d = stock
            .preview_sweep(&prepared, &m, 1., SweepOptions::default(), &cancel)
            .unwrap();
        stock.apply_delta(&d).unwrap();
        events.push(Event::Motion { motion: m });
        events.extend(plan.take_events());
    }
    events.extend(plan.suffix_events());
    plan.finish_report(&cancel).unwrap();
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
    Ok(events)
}

/// Each compensated stretch: its side, amount, moves (the entry, the
/// compensated moves, the cancel) and which of them cut the loop.
type Stretch = (CompensationSide, f64, Vec<Curve>, Vec<bool>);
fn stretches(events: &[Event]) -> Vec<Stretch> {
    let mut out = vec![];
    // The stretch being read, and whether compensation went off in it.
    let mut open: Option<(Stretch, bool)> = None;
    for e in events {
        match e {
            Event::CutterCompensation {
                side,
                amount_mm: Some(a),
                ..
            } => open = Some(((*side, *a, vec![], vec![]), false)),
            Event::CutterCompensation {
                side: CompensationSide::Off,
                ..
            } => open.as_mut().unwrap().1 = true,
            Event::Motion { motion } => {
                if let Some(((side, a, mut moves, mut cuts), off)) = open.take() {
                    moves.push(motion.curve.clone());
                    cuts.push(motion.kind == MoveKind::Cut);
                    if off {
                        out.push((side, a, moves, cuts));
                    } else {
                        open = Some(((side, a, moves, cuts), off));
                    }
                }
            }
            _ => {}
        }
    }
    assert!(open.is_none());
    out
}

fn xy(p: Vec3) -> [f64; 2] {
    [p[0], p[1]]
}
fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}
fn unit_tangent(c: &Curve, end: bool) -> [f64; 2] {
    match c {
        Curve::Line { start, end: e } => {
            let (dx, dy) = (e[0] - start[0], e[1] - start[1]);
            let n = dx.hypot(dy);
            [dx / n, dy / n]
        }
        Curve::Arc {
            start,
            end: e,
            center,
            sweep_rad,
            ..
        } => {
            let p = if end { e } else { start };
            let (rx, ry) = (p[0] - center[0], p[1] - center[1]);
            let n = rx.hypot(ry) * sweep_rad.signum();
            [-ry / n, rx / n]
        }
        Curve::Helix { .. } => unreachable!(),
    }
}

/// This test's own part side of a stretch: lines moved by `k` along their
/// left normal, arcs' radii changed, consecutive pieces joined where their
/// moved carriers meet (or at their common moved point where tangent).
fn part_side(moves: &[Curve], k: f64) -> Vec<Curve> {
    let moved_point = |c: &Curve, end: bool| {
        let p = if end { c.end() } else { c.start() };
        let t = unit_tangent(c, end);
        [p[0] - k * t[1], p[1] + k * t[0]]
    };
    let inner = &moves[1..moves.len() - 1];
    let mut joints = vec![moved_point(&inner[0], false)];
    for w in inner.windows(2) {
        let (ta, tb) = (unit_tangent(&w[0], true), unit_tangent(&w[1], false));
        let turn = (ta[0] * tb[1] - ta[1] * tb[0]).atan2(ta[0] * tb[0] + ta[1] * tb[1]);
        if turn.abs() < 1e-7 {
            joints.push(moved_point(&w[1], false));
            continue;
        }
        // Only straight sides meet at the corners of these parts.
        let (Curve::Line { .. }, Curve::Line { .. }) = (&w[0], &w[1]) else {
            panic!("a corner between arcs in this test's parts")
        };
        let (p, q) = (moved_point(&w[0], true), moved_point(&w[1], false));
        let s = ((q[0] - p[0]) * tb[1] - (q[1] - p[1]) * tb[0]) / (ta[0] * tb[1] - ta[1] * tb[0]);
        joints.push([p[0] + s * ta[0], p[1] + s * ta[1]]);
    }
    joints.push(moved_point(&inner[inner.len() - 1], true));
    let z = moves[0].start()[2];
    let at = |p: [f64; 2]| [p[0], p[1], z];
    let mut out = vec![Curve::Line {
        start: moves[0].start(),
        end: at(joints[0]),
    }];
    for (i, c) in inner.iter().enumerate() {
        out.push(match c {
            Curve::Line { .. } => Curve::Line {
                start: at(joints[i]),
                end: at(joints[i + 1]),
            },
            Curve::Arc {
                center, sweep_rad, ..
            } => Curve::Arc {
                start: at(joints[i]),
                end: at(joints[i + 1]),
                center: *center,
                plane: Plane::Xy,
                sweep_rad: *sweep_rad,
            },
            Curve::Helix { .. } => unreachable!(),
        });
    }
    out.push(Curve::Line {
        start: at(joints[joints.len() - 1]),
        end: moves[moves.len() - 1].end(),
    });
    out
}

/// The written stretch as posted in inches at four decimals, and read back.
fn post_and_read(side: CompensationSide, amount: f64, written: &[Curve]) -> (f64, Vec<Curve>) {
    let word = |v: f64| format!("{:.4}", v / INCH);
    let mut text = vec![format!(
        "{} O{}",
        if side == CompensationSide::Left {
            "G41"
        } else {
            "G42"
        },
        word(amount)
    )];
    for (k, c) in written.iter().enumerate() {
        if k + 1 == written.len() {
            text.push("G40".into());
        }
        let (s, e) = (c.start(), c.end());
        text.push(match c {
            Curve::Line { .. } => format!("G1 X{} Y{}", word(e[0]), word(e[1])),
            Curve::Arc {
                center, sweep_rad, ..
            } => format!(
                "{} X{} Y{} I{} J{}",
                if *sweep_rad < 0. { "G2" } else { "G3" },
                word(e[0]),
                word(e[1]),
                word(center[0] - s[0]),
                word(center[1] - s[1])
            ),
            Curve::Helix { .. } => unreachable!(),
        });
    }
    // Read back: absolute words, I/J relative to the block's start.
    let z = written[0].start()[2];
    let mut at = written[0].start();
    let round = |v: f64| (v / INCH * 1e4).round() / 1e4 * INCH;
    at = [round(at[0]), round(at[1]), z];
    let (mut amount_read, mut out) = (0., vec![]);
    for line in &text {
        let words: Vec<&str> = line.split(' ').collect();
        let value = |c: char| {
            words
                .iter()
                .find(|w| w.starts_with(c))
                .map(|w| w[1..].parse::<f64>().unwrap() * INCH)
        };
        match words[0] {
            "G41" | "G42" => amount_read = value('O').unwrap(),
            "G40" => {}
            "G1" => {
                let end = [value('X').unwrap(), value('Y').unwrap(), z];
                out.push(Curve::Line { start: at, end });
                at = end;
            }
            g => {
                let end = [value('X').unwrap(), value('Y').unwrap(), z];
                let center = [at[0] + value('I').unwrap(), at[1] + value('J').unwrap(), z];
                let a0 = (at[1] - center[1]).atan2(at[0] - center[0]);
                let a1 = (end[1] - center[1]).atan2(end[0] - center[0]);
                let mut sweep = (a1 - a0).rem_euclid(std::f64::consts::TAU);
                if g == "G2" {
                    sweep -= std::f64::consts::TAU;
                }
                out.push(Curve::Arc {
                    start: at,
                    end,
                    center,
                    plane: Plane::Xy,
                    sweep_rad: sweep,
                });
                at = end;
            }
        }
    }
    (amount_read, out)
}

/// Distance from a point to the plate (0 inside it) and to the hole's
/// material (0 in the plate around it).
fn to_plate(p: [f64; 2]) -> f64 {
    let dx = (PLATE[0][0] - p[0]).max(p[0] - PLATE[2][0]).max(0.);
    let dy = (PLATE[0][1] - p[1]).max(p[1] - PLATE[2][1]).max(0.);
    let outside = dx.hypot(dy);
    let in_hole = (HOLE.1 - dist(p, HOLE.0)).max(0.);
    if outside > 0. { outside } else { in_hole }
}
fn sample(c: &Curve, n: usize) -> Vec<[f64; 2]> {
    let p = c.prepare(1e-6).unwrap();
    (0..=n)
        .map(|k| xy(p.position(k as f64 / n as f64).unwrap()))
        .collect()
}

#[test]
fn the_controller_cuts_the_planned_kerf_path_from_the_posted_part_side() {
    let events = plan(operation(plate(), serde_json::json!({}))).unwrap();
    let all = stretches(&events);
    assert_eq!(all.len(), 2, "the hole and the outline");
    // Posting resolution: half an inch word, through one intersection.
    let resolution = 0.5e-4 * INCH * 4.;
    let mut joins = 0;
    for (index, (side, amount, moves, cuts)) in all.iter().enumerate() {
        // The controller is on the scrap side, left of travel; its amount
        // is half the kerf.
        assert_eq!(*side, CompensationSide::Left);
        assert_eq!(*amount, R);
        let written = part_side(moves, -amount);
        let (read_amount, read) = post_and_read(*side, *amount, &written);
        assert!((read_amount - R).abs() <= 0.5e-4 * INCH);
        let replay = controller_offset::replay(&read, *side, read_amount, 0.01).unwrap();
        assert_eq!(replay.path.len(), moves.len());
        for (got, want) in replay.path.iter().zip(moves) {
            for (a, b) in [(got.start(), want.start()), (got.end(), want.end())] {
                assert!(
                    dist(xy(a), xy(b)) <= resolution,
                    "stretch {index}: replayed {got:?}, planned {want:?}"
                );
            }
            if let (Curve::Arc { center: c1, .. }, Curve::Arc { center: c2, .. }) = (got, want) {
                assert!(dist(xy(*c1), xy(*c2)) <= resolution);
            }
        }
        // The cut's written path is the part outline (beyond the boundary
        // margin): its sides lie the margin outside the plate, its arcs the
        // margin inside the hole's wall.
        for (c, _) in written.iter().zip(cuts).filter(|(_, cut)| **cut) {
            for p in sample(c, 8) {
                let d = if index == 0 {
                    HOLE.1 - dist(p, HOLE.0)
                } else {
                    to_plate(p)
                };
                // Along the sides; the corners are the margin's corner.
                assert!(
                    d >= MARGIN - 1e-9 && d <= MARGIN * 2f64.sqrt() + 1e-9,
                    "stretch {index}: written point {p:?} is {d} from the part"
                );
            }
        }
        // Both corner joins: the kerf of each stays clear of the plate (a
        // kerf's half width plus the margin from it), and the intersection
        // join reaches r / sin(theta/2) = r sqrt 2 from the written corner
        // at the plate's square corners.
        for j in &replay.arc_joins {
            joins += 1;
            assert!((j.reach_mm - R * 2f64.sqrt()).abs() <= resolution, "{j:?}");
            assert!(to_plate(j.intersection_mm) >= R + MARGIN - resolution, "{j:?}");
            for p in sample(&j.arc, 16) {
                assert!(to_plate(p) >= R + MARGIN - resolution, "{j:?}");
            }
        }
        // Outline clockwise, hole counter-clockwise (by the written loop's
        // signed area).
        let mut area = 0.;
        for c in &written[1..written.len() - 1] {
            let pts = sample(c, 8);
            for w in pts.windows(2) {
                area += w[0][0] * w[1][1] - w[1][0] * w[0][1];
            }
        }
        if index == 0 {
            assert!(area > 0., "the hole is cut counter-clockwise");
        } else {
            assert!(area < 0., "the outline is cut clockwise");
        }
    }
    assert_eq!(joins, 4, "the plate's four outside corners");
    // The planned beam path at an outside corner is the intersection join:
    // the two sides' kerf paths meet at the corner's diagonal.
    let tip = [20. - R - MARGIN, 80. + R + MARGIN];
    assert!(events.iter().any(|e| matches!(e, Event::Motion { motion }
        if dist(xy(motion.curve.end()), tip) < 1e-6)));
    // Reversed: the part on the left, the controller on the right.
    let reversed = plan(operation(plate(), serde_json::json!({"reverse": true}))).unwrap();
    for (side, amount, moves, _) in stretches(&reversed) {
        assert_eq!(side, CompensationSide::Right);
        let written = part_side(&moves, amount);
        let (read_amount, read) = post_and_read(side, amount, &written);
        let replay = controller_offset::replay(&read, side, read_amount, 0.01).unwrap();
        for (got, want) in replay.path.iter().zip(&moves) {
            assert!(dist(xy(got.end()), xy(want.end())) <= resolution);
        }
    }
}

#[test]
fn wear_compensation_offsets_to_the_scrap_side() {
    // Wear is added toward the scrap: the left of travel unless reversed.
    for (reverse, side) in [(false, CompensationSide::Left), (true, CompensationSide::Right)] {
        let events = plan(operation(
            plate(),
            serde_json::json!({"compensation": "wear", "reverse": reverse}),
        ))
        .unwrap();
        let mut found = 0;
        let mut on = None;
        for e in &events {
            match e {
                Event::CutterCompensation { side: s, .. } if *s != CompensationSide::Off => on = Some(*s),
                Event::CutterCompensation { .. } => on = None,
                Event::Motion { motion } if motion.kind == MoveKind::Cut => {
                    let Some(s) = on else { continue };
                    assert_eq!(s, side);
                    // Independently of the plan: a point a millimetre to
                    // that side of a cut is off the part.
                    let Curve::Line { start, end } = motion.curve else {
                        continue;
                    };
                    let t = unit_tangent(&motion.curve, false);
                    let k = if s == CompensationSide::Left { 1. } else { -1. };
                    let mid = [(start[0] + end[0]) / 2., (start[1] + end[1]) / 2.];
                    let p = [mid[0] - k * t[1], mid[1] + k * t[0]];
                    let q = [mid[0] + k * t[1], mid[1] - k * t[0]];
                    let scrap = |p: [f64; 2]| {
                        let in_plate = p[0] > 20. && p[0] < 120. && p[1] > 20. && p[1] < 80.;
                        !in_plate || dist(p, HOLE.0) < HOLE.1
                    };
                    assert!(scrap(p) && !scrap(q), "{motion:?}");
                    found += 1;
                }
                _ => {}
            }
        }
        assert!(found >= 4);
    }
}

#[test]
fn what_the_controller_cannot_form_is_refused() {
    let refused = |parts: PlanarRegion, extra: serde_json::Value| plan(operation(parts, extra)).unwrap_err();
    // A slot narrower than the kerf.
    let slot = region(vec![Component {
        outer: polygon(&[
            [20., 20.],
            [120., 20.],
            [120., 80.],
            [70.5, 80.],
            [70.5, 60.],
            [69.5, 60.],
            [69.5, 80.],
            [20., 80.],
        ]),
        holes: vec![],
    }]);
    let e = refused(slot.clone(), serde_json::json!({}));
    assert!(e.contains("narrower than the kerf"), "{e}");
    plan(operation(slot, serde_json::json!({"compensation": "computer"}))).unwrap();
    // An acute corner whose extended join reaches a neighbouring part: a
    // 6 degree point 5 mm from a plate. Cut with its corner rounded by the
    // kerf ("computer") it is clear.
    let acute = || {
        region(vec![
            Component {
                outer: polygon(&[[20., 40.], [60., 42.], [20., 44.]]),
                holes: vec![],
            },
            Component {
                outer: polygon(&[[65., 20.], [100., 20.], [100., 64.], [65., 64.]]),
                holes: vec![],
            },
        ])
    };
    let e = refused(acute(), serde_json::json!({}));
    assert!(e.contains("intersection") && e.contains("not clear"), "{e}");
    plan(operation(acute(), serde_json::json!({"compensation": "computer"}))).unwrap();
    for (extra, word) in [
        (serde_json::json!({"common_line": {"min_length_mm": 5.}}), "common_line"),
        (serde_json::json!({"micro_joints": {"width_mm": 1., "count": 2}}), "micro-joints"),
        (
            serde_json::json!({"corner_loops": {"overshoot_mm": 3., "min_angle_deg": 45., "max_angle_deg": 135.}}),
            "corner_loops",
        ),
        (serde_json::json!({"lead_in": {"length_mm": 3., "radius_mm": 0.}}), "radius_mm"),
        (serde_json::json!({"lead_out": {"length_mm": 0., "radius_mm": 1.}}), "length_mm"),
        (serde_json::json!({"edge_starts": {"run_on_mm": 1., "max_lead_mm": 20.}}), "edge_starts"),
    ] {
        let e = refused(plate(), extra);
        assert!(e.contains(word), "{word}: {e}");
    }
}
