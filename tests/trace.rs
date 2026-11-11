use camlib::{
    motion::{Curve, MoveKind},
    stock::{Stock, SweepOptions},
    stream::StreamManifest,
    trace::{TraceOperation, TracePath, TracePlan, TraceSegment},
};
use std::{f64::consts::PI, sync::atomic::AtomicBool};

fn fixture() -> StreamManifest {
    serde_json::from_str(include_str!("../fixtures/operations/contour.json")).unwrap()
}

fn operation(paths: Vec<TracePath>) -> TraceOperation {
    let m = fixture();
    TraceOperation {
        limits: None,
        side_offset: None,
        id: "engrave".into(),
        tool: m.tools[0].id.clone(),
        paths,
        top_z_mm: m.stock.height().unwrap().max_mm[2],
        depth_z_mm: m.stock.height().unwrap().max_mm[2] - 1.5,
        stepdown_mm: 0.6,
        clearance_z_mm: m.initial_position_mm[2],
        feed_mm_min: 400.,
        plunge_mm_min: 100.,
        tolerance_mm: 1e-5,
        spindle_rpm: 12000.,
        work_offset: 1,
        tool_number: 1,
        length_register: 1,
        coolant: camlib::motion::Coolant::Off,
        retract_z_mm: None,
        feed_z_mm: None,
    }
}

fn paths() -> Vec<TracePath> {
    vec![
        // Closed square.
        TracePath {
            start_mm: [10., 10.],
            segments: vec![
                TraceSegment::Line { end_mm: [20., 10.] },
                TraceSegment::Line { end_mm: [20., 20.] },
                TraceSegment::Line { end_mm: [10., 20.] },
                TraceSegment::Line { end_mm: [10., 10.] },
            ],
        },
        // Open path: a line then a clockwise half circle.
        TracePath {
            start_mm: [25., 5.],
            segments: vec![
                TraceSegment::Line { end_mm: [30., 5.] },
                TraceSegment::Arc {
                    center_mm: [30., 8.],
                    sweep_rad: -PI,
                },
            ],
        },
    ]
}

fn run(op: TraceOperation) -> (Vec<camlib::motion::Motion>, Stock, TracePlan) {
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let mut plan = TracePlan::prepare(
        op,
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    let tool = m.tools[0].prepare().unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut motions = vec![];
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        let d = stock
            .preview_sweep(&tool, &motion, 1., SweepOptions::default(), &cancel)
            .unwrap();
        stock.apply_delta(&d).unwrap();
        motions.push(motion);
    }
    plan.finish_report(&cancel).unwrap();
    (motions, stock, plan)
}

#[test]
fn trace_follows_paths_exactly_at_each_depth_and_removes_stock() {
    let m = fixture();
    let op = operation(paths());
    let (top, depth) = (op.top_z_mm, op.depth_z_mm);
    let clearance = op.clearance_z_mm;
    let (motions, stock, plan) = run(op);
    let report = plan.report();
    assert_eq!(
        (report.paths, report.segments, report.closed_paths),
        (2, 6, 1)
    );
    assert_eq!(report.depth_passes, 3);
    let cuts: Vec<_> = motions.iter().filter(|m| m.kind == MoveKind::Cut).collect();
    assert_eq!(cuts.len(), 6 * 3);
    assert_eq!(report.cut_segments_emitted, cuts.len());
    // Pass depths: top - 0.6, top - 1.2, then the exact final depth.
    let mut levels: Vec<f64> = cuts.iter().map(|m| m.curve.start()[2]).collect();
    levels.dedup();
    assert_eq!(levels, vec![top - 0.6, top - 1.2, depth]);
    for cut in &cuts {
        assert_eq!(cut.curve.start()[2], cut.curve.end()[2], "cuts are planar");
    }
    // The square closes exactly on its start; the arc end follows from its sweep.
    let square_last = cuts[3].curve.end();
    assert_eq!([square_last[0], square_last[1]], [10., 10.]);
    match &cuts[5].curve {
        Curve::Arc { end, sweep_rad, .. } => {
            assert_eq!(*sweep_rad, -PI);
            assert!(
                (end[0] - 30.).abs() < 1e-12 && (end[1] - 11.).abs() < 1e-12,
                "{end:?}"
            );
        }
        other => panic!("expected an arc, got {other:?}"),
    }
    assert_eq!(motions.last().unwrap().curve.end()[2], clearance);
    assert!(stock.volume().removed_upper_mm3 > 0., "real stock removal");
    for motion in &motions {
        if motion.kind == MoveKind::Rapid {
            assert!(motion.curve.start()[2] >= m.stock.height().unwrap().max_mm[2]);
            assert!(motion.curve.end()[2] >= m.stock.height().unwrap().max_mm[2]);
        }
    }
}

#[test]
fn trace_heights_and_precise_errors() {
    let m = fixture();
    let mut op = operation(paths());
    op.retract_z_mm = Some(m.stock.height().unwrap().max_mm[2] + 2.);
    op.feed_z_mm = Some(m.stock.height().unwrap().max_mm[2] + 0.5);
    let (motions, _, _) = run(op.clone());
    for motion in &motions {
        if motion.kind == MoveKind::Rapid {
            assert!(motion.curve.start()[2] >= m.stock.height().unwrap().max_mm[2] + 0.5);
            assert!(motion.curve.end()[2] >= m.stock.height().unwrap().max_mm[2] + 0.5);
        }
    }
    let cancel = AtomicBool::new(false);
    let error = |op: TraceOperation| {
        TracePlan::prepare(
            op,
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &cancel,
        )
        .err()
        .unwrap()
    };
    let mut bad = op.clone();
    bad.depth_z_mm = bad.top_z_mm;
    assert!(error(bad).contains("depth_z_mm"));
    let mut bad = op.clone();
    bad.paths[1].segments.push(TraceSegment::Arc {
        center_mm: [30., 11.],
        sweep_rad: 1.,
    });
    assert!(error(bad).contains("path 1, segment 2: arc"));
    let mut bad = op.clone();
    bad.paths[0].segments[0] = TraceSegment::Line { end_mm: [10., 10.] };
    assert!(error(bad).contains("segment 0: line"));
    let mut bad = op;
    bad.paths.clear();
    assert!(error(bad).contains("at least one path"));
}

// ---- Side offset (#174): independent checks, not using the planner's offset code ----

use camlib::trace::{TraceSide, TraceSideOffset};

/// Distance from `p` to one chain segment starting at `at`, the segment's end,
/// and, when the nearest point is strictly inside the segment, which side `p`
/// is on (+1 left of travel, -1 right).
fn segment_distance(
    at: [f64; 2],
    segment: &TraceSegment,
    p: [f64; 2],
) -> (f64, [f64; 2], Option<f64>) {
    match *segment {
        TraceSegment::Line { end_mm } => {
            let (dx, dy) = (end_mm[0] - at[0], end_mm[1] - at[1]);
            let (px, py) = (p[0] - at[0], p[1] - at[1]);
            let u = (px * dx + py * dy) / (dx * dx + dy * dy);
            let c = u.clamp(0., 1.);
            let distance = (px - c * dx).hypot(py - c * dy);
            let side = (u > 1e-9 && u < 1. - 1e-9).then(|| (dx * py - dy * px).signum());
            (distance, end_mm, side)
        }
        TraceSegment::Arc {
            center_mm,
            sweep_rad,
        } => {
            let radius = (at[0] - center_mm[0]).hypot(at[1] - center_mm[1]);
            let start = (at[1] - center_mm[1]).atan2(at[0] - center_mm[0]);
            let end_angle = start + sweep_rad;
            let end = [
                center_mm[0] + radius * end_angle.cos(),
                center_mm[1] + radius * end_angle.sin(),
            ];
            let from_center = (p[0] - center_mm[0]).hypot(p[1] - center_mm[1]);
            let mut turn = (p[1] - center_mm[1]).atan2(p[0] - center_mm[0]) - start;
            if sweep_rad < 0. {
                turn = -turn;
            }
            let turn = turn.rem_euclid(2. * PI);
            if turn > 1e-9 && turn < sweep_rad.abs() - 1e-9 {
                // Inside a counter-clockwise arc is its left.
                let side = (radius - from_center).signum() * sweep_rad.signum();
                ((from_center - radius).abs(), end, Some(side))
            } else {
                let d = |q: [f64; 2]| (p[0] - q[0]).hypot(p[1] - q[1]);
                (d(at).min(d(end)), end, None)
            }
        }
    }
}

/// Distance from `p` to the whole chain and the side of its nearest point.
fn chain_distance(path: &TracePath, p: [f64; 2]) -> (f64, Option<f64>) {
    let mut at = path.start_mm;
    let mut best = (f64::INFINITY, None);
    for segment in &path.segments {
        let (distance, end, side) = segment_distance(at, segment, p);
        if distance < best.0 {
            best = (distance, side);
        }
        at = end;
    }
    best
}

/// Points along a planar cutting move, at most `step` apart.
fn sample(curve: &Curve, step: f64) -> Vec<[f64; 2]> {
    match *curve {
        Curve::Line { start, end } => {
            let n = ((end[0] - start[0]).hypot(end[1] - start[1]) / step).ceil() as usize + 1;
            (0..=n)
                .map(|i| {
                    let t = i as f64 / n as f64;
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
            let radius = (start[0] - center[0]).hypot(start[1] - center[1]);
            let from = (start[1] - center[1]).atan2(start[0] - center[0]);
            let n = (radius * sweep_rad.abs() / step).ceil() as usize + 1;
            (0..=n)
                .map(|i| {
                    let a = from + sweep_rad * i as f64 / n as f64;
                    [center[0] + radius * a.cos(), center[1] + radius * a.sin()]
                })
                .collect()
        }
        Curve::Helix { .. } => panic!("trace does not cut helices"),
    }
}

/// East, a left turn north, a clockwise quarter arc to east, a left turn north.
fn step_chain() -> TracePath {
    TracePath {
        start_mm: [6., 10.],
        segments: vec![
            TraceSegment::Line { end_mm: [20., 10.] },
            TraceSegment::Line { end_mm: [20., 18.] },
            TraceSegment::Arc {
                center_mm: [24., 18.],
                sweep_rad: -PI / 2.,
            },
            TraceSegment::Line { end_mm: [32., 22.] },
            TraceSegment::Line { end_mm: [32., 30.] },
        ],
    }
}

fn side_error(op: TraceOperation, tool: &camlib::tool::ToolAssembly) -> String {
    let m = fixture();
    TracePlan::prepare(
        op,
        tool,
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &AtomicBool::new(false),
    )
    .err()
    .expect("refused")
}

#[test]
fn side_offset_keeps_the_cutter_beside_an_open_chain_and_cuts_to_it() {
    let m = fixture();
    let radius = m.tools[0].cutter.diameter_mm / 2.;
    let allowance = 1.;
    let offset = radius + allowance;
    let cell = m.stock.height().unwrap().cell_size_mm;
    let chain = step_chain();
    // The two left turns: concave for a left offset (the cutter cannot reach
    // into the corner), convex for a right offset (rounded by an arc).
    let corners = [[20., 10.], [32., 22.]];
    for (side, sign, cut_arcs) in [(TraceSide::Left, 1., 1), (TraceSide::Right, -1., 3)] {
        let mut op = operation(vec![chain.clone()]);
        op.side_offset = Some(TraceSideOffset {
            side,
            radial_allowance_mm: allowance,
        });
        let (top, depth) = (op.top_z_mm, op.depth_z_mm);
        let (motions, stock, plan) = run(op);
        assert_eq!(plan.report().side, Some(side));
        assert_eq!(plan.report().side_offset_mm, Some(offset));
        let cuts: Vec<_> = motions
            .iter()
            .filter(|m| m.kind == MoveKind::Cut && m.curve.start()[2] == depth)
            .collect();
        assert!(!cuts.is_empty());
        let arcs = cuts
            .iter()
            .filter(|c| matches!(c.curve, Curve::Arc { .. }))
            .count();
        assert_eq!(arcs, cut_arcs, "{side:?}: offset arcs stay arcs");
        // The path starts on the normal through the chain's start, on the chosen
        // side, and ends on the normal through its end: no wrap around the ends.
        let first = cuts[0].curve.start();
        assert!(
            (first[0] - 6.).abs() < 1e-9 && (first[1] - (10. + sign * offset)).abs() < 1e-9,
            "{side:?}: {first:?}"
        );
        let last = cuts.last().unwrap().curve.end();
        assert!(
            (last[0] - (32. - sign * offset)).abs() < 1e-9 && (last[1] - 30.).abs() < 1e-9,
            "{side:?}: {last:?}"
        );
        // Every point of every cutting move, at every depth, is at the offset
        // from the chain (never closer: no gouge; never farther: it cuts to the
        // edge), on the chosen side.
        let mut all = vec![];
        for motion in motions.iter().filter(|m| m.kind == MoveKind::Cut) {
            let z = motion.curve.start()[2];
            if z != motion.curve.end()[2] || z >= top {
                continue; // plunges
            }
            for p in sample(&motion.curve, 0.02) {
                let (distance, nearest_side) = chain_distance(&chain, p);
                assert!(
                    (distance - offset).abs() <= 1e-6,
                    "{side:?}: {p:?} is {distance} from the chain, not {offset}"
                );
                if let Some(s) = nearest_side {
                    assert_eq!(s, sign, "{side:?}: {p:?} is on the wrong side");
                }
                if z == depth {
                    all.push(p);
                }
            }
        }
        // The cutter reaches the offset beside the middle of every chain segment.
        for target in [
            [13., 10.],
            [20., 14.],
            [24. - 4. * (PI / 4.).cos(), 18. + 4. * (PI / 4.).sin()],
            [28., 22.],
            [32., 26.],
        ] {
            let nearest = all
                .iter()
                .map(|p| (p[0] - target[0]).hypot(p[1] - target[1]))
                .fold(f64::INFINITY, f64::min);
            assert!(
                nearest <= offset + 1e-3,
                "{side:?}: {target:?} is {nearest} from the path"
            );
        }
        // Stock: nothing within the allowance of the chain or on its other side
        // is cut; the band beside it on the chosen side is cut to depth.
        let (mut kept, mut cut) = (0, 0);
        for (i, h) in stock.cells().iter().enumerate() {
            let q = stock.cell_center(i).unwrap();
            let (distance, nearest_side) = chain_distance(&chain, q);
            if distance < allowance - cell || nearest_side == Some(-sign) {
                assert_eq!(h.lower_mm, top, "{side:?}: gouged {q:?}: {h:?}");
                kept += 1;
            } else if nearest_side == Some(sign)
                && distance > allowance + cell
                && distance < allowance + 2. * radius - cell
                && corners
                    .iter()
                    .all(|c| (q[0] - c[0]).hypot(q[1] - c[1]) > 2. * offset)
            {
                assert!(h.upper_mm < depth + 0.003, "{side:?}: uncut {q:?}: {h:?}");
                cut += 1;
            }
            assert!(h.lower_mm > depth - 0.003, "{side:?}: too deep {q:?}: {h:?}");
        }
        assert!(kept > 1000 && cut > 500, "{side:?}: {kept} kept, {cut} cut");
    }
}

#[test]
fn trace_without_a_side_offset_is_unchanged() {
    let op = operation(paths());
    let value = serde_json::to_value(&op).unwrap();
    assert!(value.get("side_offset").is_none(), "{value}");
    let (_, _, plan) = run(op);
    let report = serde_json::to_value(plan.report()).unwrap();
    assert!(report.get("side").is_none() && report.get("side_offset_mm").is_none());
    assert!(
        report["machining_scope"]
            .as_str()
            .unwrap()
            .contains("no radius offset")
    );
}

#[test]
fn side_offset_refuses_what_it_cannot_cut_exactly() {
    let m = fixture();
    let with = |path: TracePath, side, allowance| {
        let mut op = operation(vec![path]);
        op.side_offset = Some(TraceSideOffset {
            side,
            radial_allowance_mm: allowance,
        });
        op
    };
    // The arc of radius 4 curves to the right: a right offset of 2 + 2.5 has no arc.
    let error = side_error(with(step_chain(), TraceSide::Right, 2.5), &m.tools[0]);
    assert!(
        error.contains("path 0, segment 2: the arc of radius 4")
            && error.contains("smaller tool"),
        "{error}"
    );
    // A closed path belongs to Contour.
    let error = side_error(with(paths().remove(0), TraceSide::Left, 0.), &m.tools[0]);
    assert!(
        error.contains("path 0 is closed") && error.contains("Contour"),
        "{error}"
    );
    // A notch 3 mm wide cannot take a 4 mm cutter: its floor would be dropped.
    let notch = TracePath {
        start_mm: [5., 20.],
        segments: [[15., 20.], [15., 26.], [18., 26.], [18., 20.], [30., 20.]]
            .into_iter()
            .map(|end_mm| TraceSegment::Line { end_mm })
            .collect(),
    };
    let error = side_error(with(notch.clone(), TraceSide::Right, 0.), &m.tools[0]);
    assert!(error.contains("not reached"), "{error}");
    // The other side of the same chain is fine.
    run(with(notch, TraceSide::Left, 0.));
    // A chain that folds back on itself.
    let fold = TracePath {
        start_mm: [5., 20.],
        segments: vec![
            TraceSegment::Line { end_mm: [25., 20.] },
            TraceSegment::Line { end_mm: [5., 21.] },
        ],
    };
    let error = side_error(with(fold, TraceSide::Left, 0.), &m.tools[0]);
    assert!(error.contains("path 0"), "{error}");
    // Allowance and cutter.
    let error = side_error(with(step_chain(), TraceSide::Left, -0.1), &m.tools[0]);
    assert!(error.contains("radial_allowance_mm"), "{error}");
    let mut ball = m.tools[0].clone();
    ball.cutter.shape = camlib::tool::CutterShape::Ball;
    let error = side_error(with(step_chain(), TraceSide::Left, 0.), &ball);
    assert!(error.contains("flat end mill"), "{error}");
    // The same ball cutter traces on its centre as before.
    assert!(
        TracePlan::prepare(
            operation(vec![step_chain()]),
            &ball,
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &AtomicBool::new(false),
        )
        .is_ok()
    );
}
