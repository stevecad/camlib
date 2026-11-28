use camlib::{facing::*, motion::*, pocket::Operation, stock::*, stream::StreamManifest};
use std::{collections::BTreeMap, sync::atomic::AtomicBool};

fn fixture() -> StreamManifest {
    serde_json::from_str(include_str!("../fixtures/operations/facing.json")).unwrap()
}
fn operation() -> FacingOperation {
    match fixture().operations.remove(0) {
        Operation::Facing(p) => p,
        _ => panic!("fixture"),
    }
}
fn plan(op: FacingOperation, m: &StreamManifest) -> FacingPlan {
    FacingPlan::prepare(
        op,
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &AtomicBool::new(false),
    )
    .unwrap()
}
fn commit(stock: &mut Stock, motion: &Motion, definition: &StreamManifest) -> f64 {
    let t = definition.tools[0].prepare().unwrap();
    let d = stock
        .preview_sweep(&t, motion, 1., definition.sweep, &AtomicBool::new(false))
        .unwrap();
    let error = d.stats().max_path_error_mm + d.stats().numerical_guard_mm;
    stock.apply_delta(&d).unwrap();
    error
}

#[test]
fn facing_continuous_strip_coverage_exterior_entries_and_actual_floor_hold_in_all_modes() {
    let cancel = AtomicBool::new(false);
    for axis in [TraverseAxis::X, TraverseAxis::Y] {
        for mode in [FacingMode::Zigzag, FacingMode::OneWay] {
            for reverse in [false, true] {
                let m = fixture();
                let mut op = operation();
                op.axis = axis;
                op.mode = mode;
                op.reverse = reverse;
                // Exercise an explicit overlap and a partial final depth pass.
                op.floor_z_mm = 8.25;
                if reverse {
                    op.edge_overlap_mm = Some(0.5);
                }
                let mut p = plan(op.clone(), &m);
                let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
                let u = if axis == TraverseAxis::X { 0 } else { 1 };
                let v = 1 - u;
                let r = m.tools[0].cutter.diameter_mm / 2.;
                let mut strips: BTreeMap<u64, Vec<(f64, bool)>> = BTreeMap::new();
                let mut position = m.initial_position_mm;
                let mut error = 0f64;
                let mut emitted = 0;
                while let Some(motion) = p.next_motion_with_stock(&stock, &cancel).unwrap() {
                    assert_eq!(motion.curve.start(), position);
                    position = motion.curve.end();
                    emitted += 1;
                    let Curve::Line { start, end } = motion.curve else {
                        panic!("facing must emit lines")
                    };
                    assert!(start[2] >= op.floor_z_mm && end[2] >= op.floor_z_mm);
                    match motion.kind {
                        MoveKind::Cut => {
                            assert_eq!(start[2], end[2]);
                            assert_eq!(start[v], end[v]);
                            assert!(
                                start[u].min(end[u])
                                    <= m.stock.height().unwrap().min_mm[u]
                                        - r
                                        - op.entry_air_gap_mm
                            );
                            assert!(
                                start[u].max(end[u])
                                    >= m.stock.height().unwrap().max_mm[u]
                                        + r
                                        + op.entry_air_gap_mm
                            );
                            strips
                                .entry(start[2].to_bits())
                                .or_default()
                                .push((start[v], end[u] < start[u]));
                        }
                        MoveKind::Plunge | MoveKind::Link => {
                            // Whole segment stays beside the original stock;
                            // cutter disk never enters it during a depth entry/link.
                            assert_eq!(start[u], end[u]);
                            assert!(
                                start[u] + r < m.stock.height().unwrap().min_mm[u]
                                    || start[u] - r > m.stock.height().unwrap().max_mm[u]
                            );
                            if motion.kind == MoveKind::Plunge {
                                assert_eq!(start[v], end[v]);
                            } else {
                                assert_eq!(start[2], end[2]);
                            }
                        }
                        MoveKind::Rapid => assert!(start[2].min(end[2]) >= op.clearance_z_mm),
                        MoveKind::Retract => {
                            assert_eq!(start[0], end[0]);
                            assert_eq!(start[1], end[1]);
                            assert_eq!(end[2], op.clearance_z_mm);
                        }
                        _ => panic!("unexpected facing move"),
                    }
                    error = error.max(commit(&mut stock, &motion, &m));
                }
                assert_eq!(strips.len(), 2);
                for rows in strips.values() {
                    for (index, &(_, negative)) in rows.iter().enumerate() {
                        assert_eq!(
                            negative,
                            reverse ^ (mode == FacingMode::Zigzag && index % 2 == 1)
                        );
                    }
                    // Independent union of complete infinite-length cut strips,
                    // valid here because ALL finite lines overrun both X/Y edges.
                    let mut intervals: Vec<_> = rows.iter().map(|&(y, _)| (y - r, y + r)).collect();
                    intervals.sort_by(|a, b| a.0.total_cmp(&b.0));
                    assert!(intervals[0].0 <= m.stock.height().unwrap().min_mm[v]);
                    let mut through = intervals[0].1;
                    for &(low, high) in &intervals[1..] {
                        assert!(low <= through);
                        through = through.max(high);
                    }
                    assert!(through >= m.stock.height().unwrap().max_mm[v]);
                }
                // All edge/corner cells, not just a reachable interior sample.
                for h in stock.cells() {
                    assert!(
                        h.upper_mm <= op.floor_z_mm + error + 1e-8,
                        "{axis:?}/{mode:?}: {h:?}"
                    );
                    assert!(
                        h.lower_mm >= op.floor_z_mm - error - 1e-8,
                        "gouged floor: {h:?}"
                    );
                }
                p.finish_report(&cancel).unwrap();
                assert_eq!(p.report().cut_rows_emitted, p.report().rows_per_pass * 2);
                assert_eq!(p.report().nominal_uncovered_area_mm2, 0.);
                assert_eq!(position[2], op.clearance_z_mm);
                assert!(emitted > 4);
                if mode == FacingMode::Zigzag {
                    assert_eq!(p.report().clearance_returns, 2);
                } else {
                    assert_eq!(p.report().side_links_emitted, 0);
                    assert_eq!(p.report().clearance_returns, p.report().cut_rows_emitted);
                }
            }
        }
    }
}

#[test]
fn facing_narrow_stock_uses_one_center_row_and_non_center_cutting_tool() {
    let cancel = AtomicBool::new(false);
    let mut m = fixture();
    m.stock.height_mut().unwrap().max_mm[1] = 2.;
    m.tools[0].cutter.center_cutting = false;
    let op = operation();
    let mut p = plan(op.clone(), &m);
    assert_eq!(p.report().rows_per_pass, 1);
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut cuts = 0;
    while let Some(motion) = p.next_motion_with_stock(&stock, &cancel).unwrap() {
        if motion.kind == MoveKind::Cut {
            cuts += 1;
            assert_eq!(motion.curve.start()[1], 1.);
        }
        commit(&mut stock, &motion, &m);
    }
    assert_eq!(cuts, 2);
    assert!(stock.cells().iter().all(|h| h.upper_mm <= 8.001));
}

#[test]
fn facing_small_cutters_cover_without_a_hidden_fixed_overlap() {
    let cancel = AtomicBool::new(false);
    let mut m = fixture();
    m.stock.height_mut().unwrap().max_mm[0] = 1.;
    m.stock.height_mut().unwrap().max_mm[1] = 1.;
    m.stock.height_mut().unwrap().cell_size_mm = 0.05;
    m.tools[0].cutter.diameter_mm = 0.5;
    m.tools[0].cutter.shank_diameter_mm = 0.5;
    let mut op = operation();
    op.stepover_mm = 0.3;
    op.floor_z_mm = 9.;
    let mut p = plan(op, &m);
    assert_eq!(p.report().edge_overlap_mm, 0.05);
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    while let Some(motion) = p.next_motion_with_stock(&stock, &cancel).unwrap() {
        commit(&mut stock, &motion, &m);
    }
    assert!(stock.cells().iter().all(|h| h.upper_mm <= 9.001));
}

#[test]
fn facing_stale_stock_cancel_retry_and_early_report_do_not_advance_uncommitted_work() {
    let cancel = AtomicBool::new(false);
    let m = fixture();
    let mut p = plan(operation(), &m);
    let mut expected = plan(operation(), &m);
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    assert!(p.finish_report(&cancel).is_err());
    for _ in 0..5 {
        let motion = p.next_motion_with_stock(&stock, &cancel).unwrap().unwrap();
        assert_eq!(
            motion,
            expected
                .next_motion_with_stock(&stock, &cancel)
                .unwrap()
                .unwrap()
        );
        assert!(
            p.next_motion_with_stock(&stock, &cancel)
                .unwrap_err()
                .contains("commit")
        );
        commit(&mut stock, &motion, &m);
        assert!(
            p.next_motion_with_stock(&stock, &AtomicBool::new(true))
                .unwrap_err()
                .contains("cancel")
        );
    }
    assert_eq!(
        p.next_motion_with_stock(&stock, &cancel).unwrap(),
        expected.next_motion_with_stock(&stock, &cancel).unwrap()
    );
}

#[test]
fn facing_rejects_unsupported_tools_setup_and_unbounded_rows_before_motion() {
    let m = fixture();
    let cancel = AtomicBool::new(false);
    for change in 0..9 {
        let mut op = operation();
        match change {
            0 => op.floor_z_mm = -1.,
            1 => op.floor_z_mm = 10.,
            2 => op.stepover_mm = 8.,
            3 => op.edge_overlap_mm = Some(4.),
            4 => op.entry_air_gap_mm = 0.,
            5 => op.stepdown_mm = 1e-8,
            6 => op.tolerance_mm = f64::NAN,
            7 => op.clearance_z_mm = 9.,
            _ => op.work_offset = 0,
        }
        assert!(
            FacingPlan::prepare(
                op,
                &m.tools[0],
                m.stock.height().unwrap(),
                m.initial_position_mm,
                1,
                &cancel
            )
            .is_err(),
            "case{change}"
        );
    }
    let mut t = m.tools[0].clone();
    t.cutter.shape = camlib::tool::CutterShape::Ball;
    assert!(
        FacingPlan::prepare(
            operation(),
            &t,
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &cancel
        )
        .is_err()
    );
    assert!(
        FacingPlan::prepare(
            operation(),
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            u64::MAX,
            &cancel
        )
        .is_err()
    );
}
#[test]
fn facing_one_way_rows_link_at_retract_height_and_end_at_clearance() {
    let m = fixture();
    let top = m.stock.height().unwrap().max_mm[2];
    let mut op = operation();
    op.mode = FacingMode::OneWay;
    let clearance = op.clearance_z_mm;
    let (retract, feed) = (top + (clearance - top) / 2., top + (clearance - top) / 4.);
    op.retract_z_mm = Some(retract);
    op.feed_z_mm = Some(feed);
    let mut p = plan(op.clone(), &m);
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut motions = vec![];
    while let Some(motion) = p
        .next_motion_with_stock(&stock, &AtomicBool::new(false))
        .unwrap()
    {
        commit(&mut stock, &motion, &m);
        motions.push(motion);
    }
    let mut at_retract = 0;
    for motion in &motions {
        let (a, b) = (motion.curve.start(), motion.curve.end());
        match motion.kind {
            MoveKind::Rapid => {
                assert!(a[2] >= feed && b[2] >= feed, "{motion:?}");
                at_retract += usize::from(a[2] == retract && b[2] == retract);
            }
            MoveKind::Retract => assert_eq!(b[2], feed),
            MoveKind::Plunge => assert_eq!(a[2], feed),
            _ => {}
        }
    }
    assert!(at_retract > 0, "rows link at the retract height");
    assert_eq!(motions.last().unwrap().curve.end()[2], clearance);
    let mut bad = op;
    bad.feed_z_mm = Some(top);
    assert!(
        FacingPlan::prepare(
            bad,
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &AtomicBool::new(false)
        )
        .err()
        .unwrap()
        .contains("stock top")
    );
}

/// One rectangular avoid area (#411).
fn avoid(lo: [f64; 2], hi: [f64; 2], top: f64) -> camlib::limits::Limits {
    serde_json::from_value(serde_json::json!({"avoid": [{
        "region": {"version": 1, "units": "mm", "name": "jaw", "boundary_error_mm": 0.,
            "components": [{"outer": {"vertices": [
                [lo[0], lo[1], 0.], [hi[0], lo[1], 0.], [hi[0], hi[1], 0.], [lo[0], hi[1], 0.]]},
                "holes": []}]},
        "top_z_mm": top}]}))
    .unwrap()
}
/// The operation as a run plans it (limits included), its moves committed to
/// the stock, and its report.
fn run_limited(
    op: FacingOperation,
    m: &StreamManifest,
) -> Result<(Vec<serde_json::Value>, serde_json::Value), String> {
    let cancel = AtomicBool::new(false);
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut plan = camlib::operation::OperationPlan::prepare_with_stock(
        Operation::Facing(op),
        &m.tools[0],
        &stock,
        m.initial_position_mm,
        1,
        &cancel,
    )?;
    let mut moves = vec![];
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel)? {
        commit(&mut stock, &motion, m);
        moves.push(serde_json::to_value(&motion).unwrap());
    }
    Ok((moves, serde_json::to_value(plan.report()).unwrap()))
}

#[test]
fn facing_limits_that_cannot_reach_the_passes_leave_the_native_facing_unchanged() {
    let m = fixture();
    // A jaw beside the stock whose top is at or below the floor (8): no
    // facing move has its tip below it. Another above everything, but far
    // from every move; and one just past the cutter's reach across the rows.
    let cases = [
        avoid([-10., -9.], [50., -1.], 8.),
        avoid([-10., -9.], [50., 30.], 2.),
        avoid([100., 0.], [110., 24.], 50.),
        avoid([-10., 25.], [50., 30.], 50.),
    ];
    for axis in [TraverseAxis::X, TraverseAxis::Y] {
        for mode in [FacingMode::Zigzag, FacingMode::OneWay] {
            for reverse in [false, true] {
                let mut op = operation();
                (op.axis, op.mode, op.reverse) = (axis, mode, reverse);
                let plain = run_limited(op.clone(), &m).unwrap();
                assert_eq!(plain.1["strategy"], "facing");
                for (i, limits) in cases.iter().enumerate() {
                    if axis == TraverseAxis::Y && i == 3 {
                        // Rows along Y run out to Y 29, into that area.
                        continue;
                    }
                    let mut limited = op.clone();
                    limited.limits = Some(limits.clone());
                    let got = run_limited(limited, &m).unwrap();
                    assert_eq!(got, plain, "{axis:?} {mode:?} {reverse} case {i}");
                }
            }
        }
    }
    // Automatic axis selection runs too once the limits are left out.
    let mut op = operation();
    op.axis = TraverseAxis::Auto;
    op.auto_rapid_mm_min = Some(6000.);
    let plain = run_limited(op.clone(), &m).unwrap();
    op.limits = Some(cases[0].clone());
    assert_eq!(run_limited(op.clone(), &m).unwrap(), plain);
    // An area that reaches the passes still refuses automatic selection.
    op.limits = Some(avoid([15., 8.], [25., 16.], 12.));
    let error = run_limited(op, &m).unwrap_err();
    assert!(error.contains("automatic facing axis"), "{error}");
}

#[test]
fn facing_around_a_clamp_enters_in_air_and_refuses_what_it_cannot_honour() {
    let m = fixture();
    let spec = m.stock.height().unwrap().clone();
    let r = m.tools[0].cutter.diameter_mm / 2.;
    let clamp = avoid([15., 8.], [25., 16.], 12.);
    for gap in [1., 0.25, 3.] {
        let mut op = operation();
        op.stepover_mm = 3.;
        op.entry_air_gap_mm = gap;
        op.limits = Some(clamp.clone());
        let (moves, report) = run_limited(op, &m).unwrap();
        assert_ne!(report["strategy"], "facing");
        let mut descents = 0;
        let mut position = m.initial_position_mm;
        for v in &moves {
            let motion: Motion = serde_json::from_value(v.clone()).unwrap();
            assert!(
                !matches!(motion.curve, Curve::Helix { .. }),
                "no helical entry: {v}"
            );
            let (a, b) = (motion.curve.start(), motion.curve.end());
            assert_eq!(a, position);
            position = b;
            // Every move that takes the tip below the stock top from above
            // it (an entry) does so with the whole cutter at least the air
            // gap outside the stock.
            if a[2] >= spec.max_mm[2] && b[2] < spec.max_mm[2] {
                descents += 1;
                assert_eq!((a[0], a[1]), (b[0], b[1]), "vertical entry: {v}");
                let d = [0, 1]
                    .map(|k| (spec.min_mm[k] - b[k]).max(b[k] - spec.max_mm[k]).max(0.));
                assert!(
                    d[0].hypot(d[1]) >= r + gap,
                    "entry at {b:?} with gap {gap}: {v}"
                );
            }
        }
        assert!(descents > 0);
    }
    let refused = |change: &dyn Fn(&mut FacingOperation)| {
        let mut op = operation();
        op.stepover_mm = 3.;
        op.limits = Some(clamp.clone());
        change(&mut op);
        run_limited(op, &m).unwrap_err()
    };
    for (change, name) in [
        (
            &(|op: &mut FacingOperation| op.mode = FacingMode::OneWay) as &dyn Fn(&mut _),
            "mode \"one_way\"",
        ),
        (&|op: &mut FacingOperation| op.reverse = true, "reverse"),
        (&|op: &mut FacingOperation| op.axis = TraverseAxis::Y, "axis \"y\""),
    ] {
        let error = refused(change);
        assert!(
            error.contains(name) && error.contains("facing face"),
            "{error}"
        );
    }
}

/// Rectangular avoid areas: (low corner, high corner, top).
fn areas(boxes: &[([f64; 2], [f64; 2], f64)]) -> camlib::limits::Limits {
    let avoid: Vec<_> = boxes
        .iter()
        .map(|&(lo, hi, top)| avoid(lo, hi, top).avoid.remove(0))
        .collect();
    camlib::limits::Limits {
        contain: None,
        avoid,
    }
}
/// Points along a move, at most `step` apart (lines and XY arcs).
fn samples(curve: &Curve, step: f64) -> Vec<[f64; 3]> {
    match *curve {
        Curve::Line { start, end } => {
            let length = (0..3).map(|k| (end[k] - start[k]).powi(2)).sum::<f64>().sqrt();
            let n = ((length / step).ceil() as usize).max(1);
            (0..=n)
                .map(|i| {
                    let t = i as f64 / n as f64;
                    [0, 1, 2].map(|k| start[k] + t * (end[k] - start[k]))
                })
                .collect()
        }
        Curve::Arc {
            start,
            end,
            center,
            plane: Plane::Xy,
            sweep_rad,
        } => {
            let rho = (start[0] - center[0]).hypot(start[1] - center[1]);
            let a0 = (start[1] - center[1]).atan2(start[0] - center[0]);
            let n = (((rho * sweep_rad.abs()) / step).ceil() as usize).max(1);
            (0..=n)
                .map(|i| {
                    let t = i as f64 / n as f64;
                    let a = a0 + t * sweep_rad;
                    [
                        center[0] + rho * a.cos(),
                        center[1] + rho * a.sin(),
                        start[2] + t * (end[2] - start[2]),
                    ]
                })
                .collect()
        }
        _ => panic!("facing round limits plans lines and XY arcs only: {curve:?}"),
    }
}
fn to_box(p: [f64; 2], lo: [f64; 2], hi: [f64; 2]) -> f64 {
    let d = [0, 1].map(|k| (lo[k] - p[k]).max(p[k] - hi[k]).max(0.));
    d[0].hypot(d[1])
}

#[test]
fn facing_round_clamps_cuts_up_to_them_links_at_depth_and_leaves_only_what_they_block() {
    // Clamps on the stock's X- and X+ edges whose keep-out reaches the
    // passes (#422), and a low block on its Y- edge whose top lies between
    // the two depth passes (9 and 8): only the last pass goes round it.
    let m = fixture();
    let spec = m.stock.height().unwrap().clone();
    let r = m.tools[0].cutter.diameter_mm / 2.;
    // Their tops are under the 12 mm clearance plane, so rapids pass over.
    let boxes = [
        ([-1., 8.], [5., 16.], 11.5),
        ([35., 8.], [41., 16.], 11.5),
        ([17., -1.], [23., 3.], 8.5),
    ];
    let mut op = operation();
    op.limits = Some(areas(&boxes));
    let cancel = AtomicBool::new(false);
    let mut stock = Stock::new(spec.clone()).unwrap();
    let mut plan = camlib::operation::OperationPlan::prepare_with_stock(
        Operation::Facing(op.clone()),
        &m.tools[0],
        &stock,
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    let mut moves = vec![];
    let mut error = 0f64;
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        error = error.max(commit(&mut stock, &motion, &m));
        moves.push(motion);
    }
    let report = serde_json::to_value(plan.report()).unwrap();
    assert_eq!(report["strategy"], "facing_round_limits", "{report}");

    let mut position = m.initial_position_mm;
    let mut entries = 0;
    for motion in &moves {
        let (a, b) = (motion.curve.start(), motion.curve.end());
        assert_eq!(a, position);
        position = b;
        // Every rise is straight up.
        if b[2] > a[2] {
            assert_eq!((a[0], a[1]), (b[0], b[1]), "{motion:?}");
            assert!(matches!(motion.kind, MoveKind::Retract | MoveKind::Rapid));
        }
        // Every descent into the stock is vertical, with the whole cutter at
        // least the air gap outside it.
        if a[2] >= spec.max_mm[2] && b[2] < spec.max_mm[2] {
            entries += 1;
            assert_eq!((a[0], a[1]), (b[0], b[1]), "{motion:?}");
            let outside = to_box([b[0], b[1]], [spec.min_mm[0], spec.min_mm[1]], [
                spec.max_mm[0],
                spec.max_mm[1],
            ]);
            assert!(outside >= r + op.entry_air_gap_mm, "{motion:?}");
        }
        let points = samples(&motion.curve, 0.01);
        // No part of the cutter enters an area while its tip is below the
        // area's top; every centre keeps the region operations' clearance.
        for p in &points {
            for &(lo, hi, top) in &boxes {
                if p[2] < top {
                    let d = to_box([p[0], p[1]], lo, hi);
                    assert!(
                        d >= r + camlib::limits::REGION_CLEARANCE - 1e-9,
                        "{:?} {motion:?} at {p:?}: {d}",
                        motion.kind
                    );
                }
            }
        }
        // A cut brings the cutter over the stock (no pass in air).
        if motion.kind == MoveKind::Cut {
            let over = points.iter().any(|p| {
                to_box([p[0], p[1]], [spec.min_mm[0], spec.min_mm[1]], [
                    spec.max_mm[0],
                    spec.max_mm[1],
                ]) < r
            });
            assert!(over, "a cut in air: {motion:?}");
        }
    }
    assert_eq!(position[2], op.clearance_z_mm);
    // At most one entry for the rows and one for each area's edge per depth
    // pass (two areas at Z 9, three at Z 8): no entry per row.
    assert!(entries <= (1 + 2) + (1 + 3), "{entries} entries: {report}");
    assert_eq!(report["around_limits"]["entries"], entries);

    // What stands above the floor is only what the areas block: within the
    // clearance of an area active at the floor (all three), up to half a
    // cell's diagonal and the sweep's error (a cell keeps its highest point).
    // Over the low block, the first pass cut the stock down to its own depth.
    let half = 0.5 * spec.cell_size_mm * 2f64.sqrt();
    let reach = camlib::limits::REGION_CLEARANCE + 1e-3 + half + error;
    for (i, h) in stock.cells().iter().enumerate() {
        assert!(h.lower_mm >= op.floor_z_mm - error - 1e-8, "gouged floor: {h:?}");
        if h.upper_mm <= op.floor_z_mm + error + 1e-8 {
            continue;
        }
        let c = stock.cell_center(i).unwrap();
        let near = boxes.iter().any(|&(lo, hi, _)| to_box(c, lo, hi) <= reach);
        assert!(near, "material left at {c:?} where nothing blocks the cutter: {h:?}");
        if to_box(c, boxes[2].0, boxes[2].1) <= reach {
            assert!(h.upper_mm <= 9. + error + 1e-8, "over the low block {c:?}: {h:?}");
        }
    }
    let around = &report["around_limits"];
    assert!(around["unreachable_area_mm2"].as_f64().unwrap() > 0., "{report}");
    assert_eq!(report["nominal_uncovered_area_mm2"], around["unreachable_area_mm2"]);
}

#[test]
fn facing_round_a_containment_inside_the_stock_is_refused_not_plunged() {
    // Every row stretch then stops inside the stock at both ends: no entry
    // from air reaches them, and none is plunged into.
    let m = fixture();
    let mut op = operation();
    let mut limits = avoid([0., 0.], [0., 0.], 0.);
    limits.avoid.clear();
    limits.contain = Some(
        serde_json::from_value(serde_json::json!({
            "version": 1, "units": "mm", "name": "window", "boundary_error_mm": 0.,
            "components": [{"outer": {"vertices": [
                [5., 3., 0.], [35., 3., 0.], [35., 21., 0.], [5., 21., 0.]]}, "holes": []}]}))
        .unwrap(),
    );
    op.limits = Some(limits);
    let error = run_limited(op, &m).unwrap_err();
    assert!(error.contains("no entry from outside the stock"), "{error}");
}
