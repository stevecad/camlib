use camlib::{holes::*, motion::*, pocket::Operation, stock::*, stream::StreamManifest};
use std::{collections::BTreeMap, f64::consts::TAU, sync::atomic::AtomicBool};
fn fixture() -> StreamManifest {
    serde_json::from_str(include_str!("../fixtures/operations/holes.json")).unwrap()
}
fn drilling() -> DrillingOperation {
    match fixture().operations.remove(0) {
        Operation::Drilling(p) => p,
        _ => panic!("fixture"),
    }
}
fn bore() -> BoreOperation {
    match fixture().operations.remove(1) {
        Operation::Bore(p) => p,
        _ => panic!("fixture"),
    }
}
fn drill_plan(p: DrillingOperation) -> Result<DrillingPlan, String> {
    let m = fixture();
    DrillingPlan::prepare(
        p,
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &AtomicBool::new(false),
    )
}
fn bore_plan(p: BoreOperation) -> Result<BorePlan, String> {
    let m = fixture();
    BorePlan::prepare(
        p,
        &m.tools[1],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &AtomicBool::new(false),
    )
}
fn commit(stock: &mut Stock, m: &Motion) {
    let f = fixture();
    let tool = f
        .tools
        .iter()
        .find(|t| t.id == m.tool)
        .unwrap()
        .prepare()
        .unwrap();
    let d = stock
        .preview_sweep(&tool, m, 1., f.sweep, &AtomicBool::new(false))
        .unwrap();
    stock.apply_delta(&d).unwrap();
}

fn ordering(max_work: usize) -> camlib::ordering::HoleOrdering {
    use camlib::dynamics::{JunctionMode, Policy, Settings};
    camlib::ordering::HoleOrdering {
        nearest_neighbor: false,
        settings: Settings {
            policy: Policy {
                tangential_accel_mm_s2: 100.,
                rapid_feed_mm_min: 6000.,
                junction_mode: JunctionMode::ExactStop,
                feed_reserve_fraction: 0.,
                per_block_s: 0.,
            },
            max_feed_mm_min: 6000.,
            max_normal_accel_mm_s2: None,
            max_axis_feed_mm_min: None,
            work_to_machine: WorkFrame::identity(),
            feed_quantum_mm_min: 0.001,
            adjust_feeds: false,
        },
        max_work,
    }
}

fn preparation_fixture() -> StreamManifest {
    use camlib::tool::CutterShape;
    let mut m = fixture();
    let mut final_op = drilling();
    final_op.id = "final".into();
    final_op.peck_mm = None;
    let mut spot = final_op.clone();
    spot.id = "spot".into();
    spot.tool = "spot6".into();
    spot.depth_z_mm = 10. - 1. / 75_f64.to_radians().tan();
    let mut pilot = final_op.clone();
    pilot.id = "pilot".into();
    pilot.tool = "pilot2".into();
    pilot.depth_z_mm = 8.5;
    pilot.depth_reference = DepthReference::FullDiameter;
    for (id, diameter, angle) in [("spot6", 6., 150_f64), ("pilot2", 2., 140_f64)] {
        let mut t = m.tools[0].clone();
        t.id = id.into();
        t.cutter.diameter_mm = diameter;
        t.cutter.shape = CutterShape::Drill { point_angle_rad: angle.to_radians() };
        m.tools.push(t);
    }
    final_op.preparation = Some(DrillingPreparation { spot: Some("spot".into()), spot_diameter_mm: Some(2.), pilot: Some("pilot".into()) });
    m.operations = vec![Operation::Drilling(spot), Operation::Drilling(pilot), Operation::Drilling(final_op)];
    m
}

#[test]
fn drilling_preparation_checks_source_geometry_coverage_order_and_cancel() {
    let m = preparation_fixture();
    let cancel = AtomicBool::new(false);
    let report = preparation_report(&m, 2, &cancel).unwrap().unwrap();
    assert_eq!(report.holes, drilling().centers_mm.len());
    assert!((report.spot.unwrap().diameter_mm - 2.).abs() < 1e-12);
    assert!((report.pilot.unwrap().full_diameter_depth_mm - 1.5).abs() < 1e-12);
    assert!(preparation_report(&m, 2, &AtomicBool::new(true)).unwrap_err().contains("cancelled"));
    let mut bad = m.clone();
    bad.operations.swap(0, 1);
    assert!(preparation_report(&bad, 2, &cancel).unwrap_err().contains("order"));
    let mut bad = m.clone();
    let Operation::Drilling(spot) = &mut bad.operations[0] else { unreachable!() };
    spot.centers_mm.pop();
    assert!(preparation_report(&bad, 2, &cancel).unwrap_err().contains("cover"));
    let Operation::Drilling(final_op) = &m.operations[2] else { unreachable!() };
    assert!(DrillingPlan::prepare(final_op.clone(), &m.tools[0], m.stock.height().unwrap(), m.initial_position_mm, 1, &cancel).err().unwrap().contains("run"));
}

#[test]
fn drilling_preparation_no_metadata_default_and_duplicate_occurrences_are_not_skipped() {
    assert!(serde_json::to_value(drilling()).unwrap().get("preparation").is_none());
    let mut m = preparation_fixture();
    let Operation::Drilling(final_op) = &mut m.operations[2] else { unreachable!() };
    final_op.centers_mm.push(final_op.centers_mm[0]);
    assert!(preparation_report(&m, 2, &AtomicBool::new(false)).unwrap_err().contains("duplicate"));
}

#[test]
fn bounded_drilling_order_changes_only_clearance_travel_and_keeps_stock_and_exit() {
    let centers = vec![[34., 18.], [6., 6.], [34., 6.], [6., 18.], [20., 6.], [20., 18.]];
    for canned in [false, true] {
        let run = |ordered: bool| {
            let mut op = drilling();
            op.centers_mm = centers.clone();
            op.canned_cycle = canned;
            op.reentry_clearance_mm = 0.254;
            op.chip_break_lift_mm = 0.254;
            op.ordering = ordered.then(|| ordering(10_000));
            let mut plan = drill_plan(op).unwrap();
            let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
            let mut paths = Vec::new();
            while let Some(motion) = plan.next_motion_with_stock(&stock, &AtomicBool::new(false)).unwrap() {
                commit(&mut stock, &motion);
                paths.push(motion);
            }
            plan.finish_report(&AtomicBool::new(false)).unwrap();
            (plan, stock, paths)
        };
        let (given, old_stock, old_moves) = run(false);
        let (ordered, new_stock, new_moves) = run(true);
        let report = ordered.report().ordering.as_ref().unwrap();
        assert!(given.report().ordering.is_none());
        assert!(report.ordered_seconds < 0.8 * report.given_seconds);
        assert_eq!(*report.order.last().unwrap(), centers.len() - 1);
        let mut permutation = report.order.clone();
        permutation.sort_unstable();
        assert_eq!(permutation, (0..centers.len()).collect::<Vec<_>>());
        assert!(report.work <= 10_000 && !report.work_limit_reached);
        assert_eq!(old_moves.last().unwrap().curve.end(), new_moves.last().unwrap().curve.end());
        assert_eq!(given.report().cut_pecks, ordered.report().cut_pecks);
        assert_eq!(given.report().completed_holes, ordered.report().completed_holes);
        for (a, b) in old_stock.cells().iter().zip(new_stock.cells()) {
            assert_eq!((a.lower_mm, a.upper_mm), (b.lower_mm, b.upper_mm));
        }
        // Compare the actual fed paths independently, without the ordering report.
        let cutting = |moves: &[Motion]| {
            let mut result: Vec<_> = moves.iter().filter(|m| !m.kind.is_rapid()).map(|m| {
                let a = m.curve.start(); let b = m.curve.end();
                assert_eq!([a[0], a[1]], [b[0], b[1]]);
                serde_json::to_string(&(m.kind, &m.curve, m.feed_mm_min)).unwrap()
            }).collect();
            result.sort();
            result
        };
        assert_eq!(cutting(&old_moves), cutting(&new_moves));
        let rapid_time = |moves: &[Motion]| -> f64 {
            moves.iter().filter(|m| m.kind.is_rapid() && m.curve.start()[..2] != m.curve.end()[..2]).map(|m| {
                let a=m.curve.start(); let b=m.curve.end();
                if a[..2] != b[..2] {
                    assert!(a[2] >= 12. && b[2] >= 12.);
                }
                let length = (0..3).map(|k| (b[k]-a[k]).powi(2)).sum::<f64>().sqrt();
                // Independent exact-stop triangle/trapezoid, v=100 mm/s, a=100 mm/s².
                if length <= 100. { 2. * (length / 100.).sqrt() } else { 2. + (length - 100.) / 100. }
            }).sum()
        };
        assert!(rapid_time(&new_moves) < 0.8 * rapid_time(&old_moves));
    }
}

#[test]
fn drilling_order_budget_settings_ties_and_cancellation_are_explicit() {
    use std::sync::atomic::{AtomicUsize, Ordering};
    struct CancelAfter(AtomicUsize);
    impl CancelSignal for CancelAfter {
        fn is_cancelled(&self) -> bool { self.0.fetch_add(1, Ordering::Relaxed) >= 12 }
    }
    let mut op=drilling();
    op.centers_mm=vec![[34.,18.],[6.,6.],[34.,6.],[6.,18.],[20.,6.],[20.,18.]];
    op.ordering=Some(ordering(24));
    let plan=drill_plan(op.clone()).unwrap();
    let report=plan.report().ordering.as_ref().unwrap();
    assert_eq!(report.order, (0..6).collect::<Vec<_>>());
    assert!(report.work_limit_reached && report.work == 24);
    op.ordering=Some(ordering(23));
    assert!(drill_plan(op.clone()).err().unwrap().contains("max_work"));
    op.ordering=Some(ordering(10000));
    let f=fixture();
    let result=DrillingPlan::prepare(op.clone(), &f.tools[0], f.stock.height().unwrap(), f.initial_position_mm, 1, &CancelAfter(AtomicUsize::new(0)));
    assert!(result.err().unwrap().contains("cancelled"));
    op.ordering.as_mut().unwrap().settings.policy.tangential_accel_mm_s2=f64::NAN;
    assert!(drill_plan(op.clone()).is_err());
    op.ordering=Some(ordering(10000));
    op.centers_mm=vec![[10.,12.],[10.,12.],[28.,12.]];
    let a=drill_plan(op.clone()).unwrap(); let b=drill_plan(op).unwrap();
    let a=a.report().ordering.as_ref().unwrap(); let b=b.report().ordering.as_ref().unwrap();
    assert_eq!(a.order, vec![0,1,2]);
    assert_eq!(a.order,b.order);
    assert_eq!(a.given_seconds,a.ordered_seconds);
}

#[test]
fn drilling_tip_shoulder_peck_modes_and_whole_cell_cone_oracle_agree() {
    let cancel = AtomicBool::new(false);
    let slope = 59f64.to_radians().tan();
    let height = 2. / slope;
    for reference in [DepthReference::Tip, DepthReference::FullDiameter] {
        for mode in [PeckMode::FullRetract, PeckMode::ChipBreak] {
            let mut op = drilling();
            op.depth_z_mm = 4.3;
            op.depth_reference = reference;
            op.peck_mode = mode;
            let expected_tip = 4.3
                - if reference == DepthReference::FullDiameter {
                    height
                } else {
                    0.
                };
            let mut p = drill_plan(op.clone()).unwrap();
            assert!((p.report().tip_target_z_mm - expected_tip).abs() < 1e-12);
            assert!((p.report().point_height_mm - height).abs() < 1e-12);
            let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
            let mut position = fixture().initial_position_mm;
            let mut cut_z = Vec::new();
            while let Some(m) = p.next_motion_with_stock(&stock, &cancel).unwrap() {
                assert_eq!(m.curve.start(), position);
                position = m.curve.end();
                let Curve::Line { start, end } = m.curve else {
                    panic!("drill must emit lines")
                };
                if m.kind == MoveKind::Rapid {
                    assert!(start[2].min(end[2]) > 10.);
                } else {
                    assert_eq!(start[0], end[0]);
                    assert_eq!(start[1], end[1]);
                }
                if m.kind == MoveKind::Plunge {
                    cut_z.push(end[2]);
                    assert_eq!(m.feed_mm_min, Some(op.feed_mm_min));
                }
                assert!(end[2] >= expected_tip - 1e-10);
                commit(&mut stock, &m);
            }
            p.finish_report(&cancel).unwrap();
            let count = ((10. - expected_tip) / 2.).ceil() as usize;
            assert_eq!(p.report().pecks_per_hole, count);
            assert_eq!(p.report().cut_pecks, count * 2);
            assert_eq!(p.report().peck_returns, (count - 1) * 2);
            assert!((cut_z[count - 1] - expected_tip).abs() < 1e-12);
            assert_eq!(p.report().completed_holes, 2);
            // Independent pointwise cone/cylinder surface must fall in each WHOLE-cell interval.
            for (i, h) in stock.cells().iter().enumerate() {
                let q = stock.cell_center(i).unwrap();
                let mut expected = 10f64;
                for c in &op.centers_mm {
                    let d = (q[0] - c[0]).hypot(q[1] - c[1]);
                    if d <= 2. {
                        expected = expected.min(expected_tip + d / slope);
                    }
                }
                assert!(
                    h.lower_mm <= expected + 1e-8 && h.upper_mm >= expected - 1e-8,
                    "{q:?}: {h:?} expected{expected}"
                );
                if op
                    .centers_mm
                    .iter()
                    .all(|c| (q[0] - c[0]).hypot(q[1] - c[1]) > 2.5)
                {
                    assert_eq!(h.lower_mm, 10.);
                }
            }
        }
    }
}
#[test]
fn drilling_explicit_breakthrough_is_required_and_stock_stays_clipped_to_bottom() {
    let mut op = drilling();
    op.depth_reference = DepthReference::FullDiameter;
    op.depth_z_mm = 0.;
    assert!(drill_plan(op.clone()).is_err());
    op.allow_breakthrough_mm = 2.;
    let mut p = drill_plan(op).unwrap();
    assert!(p.report().tip_target_z_mm < 0.);
    assert!(p.report().breakthrough_mm > 1.);
    let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
    let mut lowest = 0f64;
    while let Some(m) = p
        .next_motion_with_stock(&stock, &AtomicBool::new(false))
        .unwrap()
    {
        lowest = lowest.min(m.curve.end()[2]);
        commit(&mut stock, &m);
    }
    assert!(lowest < 0.);
    assert!(stock.cells().iter().all(|h| h.lower_mm >= 0.));
    let ideal = 2. * std::f64::consts::PI * 4. * 10.;
    let v = stock.volume();
    assert!(v.removed_lower_mm3 <= ideal && v.removed_upper_mm3 >= ideal);
}
#[test]
fn bore_full_interior_helix_and_radial_intervals_cover_small_and_large_holes() {
    let cancel = AtomicBool::new(false);
    for (diameter, clockwise) in [(7., true), (12., false), (16., true)] {
        let mut op = bore();
        op.centers_mm = vec![[20., 12.]];
        op.diameter_mm = diameter;
        op.clockwise = clockwise;
        op.floor_z_mm = 4.3;
        op.stepdown_mm = 2.;
        let r = 2.;
        let wall = diameter / 2. - 0.005;
        let mut p = bore_plan(op.clone()).unwrap();
        let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
        let mut circles: BTreeMap<u64, Vec<f64>> = BTreeMap::new();
        let mut position = fixture().initial_position_mm;
        while let Some(m) = p.next_motion_with_stock(&stock, &cancel).unwrap() {
            assert_eq!(m.curve.start(), position);
            position = m.curve.end();
            let distance = |q: Vec3| (q[0] - 20.).hypot(q[1] - 12.);
            match m.curve {
                Curve::Helix {
                    start,
                    end,
                    center,
                    sweep_rad,
                    ..
                } => {
                    assert!(distance(start) <= r);
                    assert_eq!(sweep_rad, if clockwise { -TAU } else { TAU });
                    assert!(start[2] > end[2]);
                    assert_eq!(center[2], start[2]);
                    assert!(distance(start) + r <= wall + 1e-10);
                }
                Curve::Arc {
                    start,
                    center,
                    sweep_rad,
                    ..
                } => {
                    assert_eq!([center[0], center[1]], [20., 12.]);
                    assert_eq!(sweep_rad, if clockwise { -TAU } else { TAU });
                    let radius = distance(start);
                    assert!(radius + r <= wall + 1e-10);
                    circles.entry(start[2].to_bits()).or_default().push(radius);
                }
                Curve::Line { start, end } if m.kind != MoveKind::Rapid => {
                    assert!(distance(start).max(distance(end)) + r <= wall + 1e-10);
                }
                Curve::Line { start, end } => assert!(start[2].min(end[2]) >= 12.),
            }
            assert!(m.curve.start()[2].min(m.curve.end()[2]) >= 4.3);
            commit(&mut stock, &m);
        }
        assert_eq!(circles.len(), 3);
        for radii in circles.values_mut() {
            radii.sort_by(f64::total_cmp);
            assert!(radii[0] <= r);
            let mut through = radii[0] + r;
            for &radius in &radii[1..] {
                assert!(radius - r <= through);
                through = radius + r;
            }
            assert!(
                (through - wall).abs() < 1e-10,
                "unremoved bore core/wall, radii{radii:?}"
            );
        }
        assert_eq!(p.report().helical_turns, 3);
        assert!(p.report().core_cleared_nominal);
        p.finish_report(&cancel).unwrap();
        for (i, h) in stock.cells().iter().enumerate() {
            let q = stock.cell_center(i).unwrap();
            let d = (q[0] - 20.).hypot(q[1] - 12.);
            if d < wall - 0.5 {
                assert!(h.upper_mm <= 4.303, "uncleared interior {q:?}: {h:?}");
            }
            if d > diameter / 2. + 0.5 {
                assert_eq!(h.lower_mm, 10.);
            }
            assert!(h.lower_mm >= 4.297);
        }
    }
}
#[test]
fn holes_stale_cancel_and_early_report_preserve_next_motion() {
    let cancel = AtomicBool::new(false);
    let mut p = drill_plan(drilling()).unwrap();
    let mut expected = drill_plan(drilling()).unwrap();
    let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
    assert!(p.finish_report(&cancel).is_err());
    for _ in 0..7 {
        let m = p.next_motion_with_stock(&stock, &cancel).unwrap().unwrap();
        assert_eq!(
            m,
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
        commit(&mut stock, &m);
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
    let mut p = bore_plan(bore()).unwrap();
    let mut expected = bore_plan(bore()).unwrap();
    let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
    for _ in 0..10 {
        let m = p.next_motion_with_stock(&stock, &cancel).unwrap().unwrap();
        assert_eq!(
            m,
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
        commit(&mut stock, &m);
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
fn hole_parameter_and_work_limits_reject_before_emission() {
    for case in 0..5 {
        let mut op = drilling();
        match case {
            0 => op.centers_mm.clear(),
            1 => op.centers_mm = vec![[0., 0.]],
            2 => op.peck_mm = Some(0.),
            3 => op.retract_z_mm = Some(9.),
            _ => op.depth_z_mm = 11.,
        }
        assert!(drill_plan(op).is_err(), "drill{case}");
    }
    for case in 0..5 {
        let mut op = bore();
        match case {
            0 => op.diameter_mm = 4.,
            1 => op.radial_stepover_mm = Some(3.),
            2 => op.floor_z_mm = -1.,
            3 => op.boundary_tolerance_mm = 0.,
            _ => op.centers_mm = vec![[1., 1.]],
        }
        assert!(bore_plan(op).is_err(), "bore{case}");
    }
}
#[test]
fn bore_links_holes_at_retract_height_and_ends_at_clearance() {
    let m = fixture();
    let top = m.stock.height().unwrap().max_mm[2];
    let mut op = bore();
    assert!(op.centers_mm.len() > 1, "fixture needs several holes");
    let clearance = op.clearance_z_mm;
    let (retract, feed) = (top + (clearance - top) / 2., top + (clearance - top) / 4.);
    op.retract_z_mm = Some(retract);
    op.feed_z_mm = Some(feed);
    let mut p = bore_plan(op.clone()).unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut motions = vec![];
    while let Some(motion) = p
        .next_motion_with_stock(&stock, &AtomicBool::new(false))
        .unwrap()
    {
        commit(&mut stock, &motion);
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
            _ => {}
        }
    }
    assert!(at_retract > 0, "holes link at the retract height");
    assert_eq!(motions.last().unwrap().curve.end()[2], clearance);
    let mut bad = op;
    bad.feed_z_mm = Some(top);
    assert!(bore_plan(bad).err().unwrap().contains("stock top"));
}

#[test]
fn drilling_dwell_follows_the_final_depth_cut_of_each_hole() {
    let cancel = AtomicBool::new(false);
    for mode in [PeckMode::FullRetract, PeckMode::ChipBreak] {
        let mut op = drilling();
        op.peck_mode = mode;
        op.peck_mm = Some(1.);
        op.dwell_s = Some(0.4);
        let holes = op.centers_mm.len();
        let mut p = drill_plan(op).unwrap();
        let target = p.report().tip_target_z_mm;
        assert!(p.take_events().is_empty());
        let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
        let mut dwells = 0;
        while let Some(m) = p.next_motion_with_stock(&stock, &cancel).unwrap() {
            commit(&mut stock, &m);
            let events = p.take_events();
            let at_bottom = m.kind != MoveKind::Rapid
                && m.curve.end()[2] < m.curve.start()[2]
                && (m.curve.end()[2] - target).abs() < 1e-9;
            if at_bottom {
                assert!(
                    matches!(events.as_slice(), [Event::Dwell { seconds }] if *seconds == 0.4),
                    "{events:?}"
                );
                dwells += 1;
            } else {
                assert!(events.is_empty(), "{m:?} {events:?}");
            }
        }
        assert_eq!(dwells, holes);
    }
    for bad in [0., -1., f64::NAN, f64::INFINITY] {
        let mut op = drilling();
        op.dwell_s = Some(bad);
        assert!(drill_plan(op).err().unwrap().contains("dwell_s"));
    }
    // Previously refused only by the invented 1e5 cap (#433): the control's
    // dwell range is the dialect's dwell_max_s at post time.
    let mut long = drilling();
    long.dwell_s = Some(1e6);
    assert!(drill_plan(long).is_ok());
}

/// LinuxCNC interp_cycles.cc G81/G82/G83/G73 with G98, re-stated
/// independently: (end Z, rapid, dwell after).
fn linuxcnc_cycle(
    z0: f64,
    r: f64,
    z: f64,
    q: Option<f64>,
    chip_break: bool,
    delta: f64,
    dwell: bool,
) -> Vec<(f64, bool, bool)> {
    let mut out = vec![(r, true, false)];
    if let Some(q) = q {
        let mut current = r - q;
        while current > z {
            out.push((current, false, false));
            if !chip_break {
                out.push((r, true, false));
            }
            out.push((current + delta, true, false));
            current -= q;
        }
    }
    out.push((z, false, dwell));
    out.push((z0, true, false));
    out
}

#[test]
fn canned_cycle_moves_match_the_controller_cycles_and_announce_each_hole() {
    let cancel = AtomicBool::new(false);
    for (peck, mode, dwell) in [
        (None, PeckMode::FullRetract, None),
        (None, PeckMode::FullRetract, Some(0.3)),
        (Some(1.5), PeckMode::FullRetract, None),
        (Some(1.5), PeckMode::ChipBreak, None),
    ] {
        let mut op = drilling();
        op.canned_cycle = true;
        op.peck_mm = peck;
        op.peck_mode = mode;
        op.dwell_s = dwell;
        op.reentry_clearance_mm = 0.254;
        op.chip_break_lift_mm = 0.254;
        let centers = op.centers_mm.clone();
        let clearance = op.clearance_z_mm;
        let mut p = drill_plan(op).unwrap();
        let (r, z) = (p.report().retract_plane_z_mm, p.report().tip_target_z_mm);
        let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
        // Items in order: Ok(motion) or Err(event).
        let mut items: Vec<Result<Motion, Event>> = p.take_events().into_iter().map(Err).collect();
        while let Some(m) = p.next_motion_with_stock(&stock, &cancel).unwrap() {
            commit(&mut stock, &m);
            items.push(Ok(m));
            items.extend(p.take_events().into_iter().map(Err));
        }
        let mut hole = 0;
        let mut i = 0;
        while i < items.len() {
            let Err(Event::Cycle {
                center_mm,
                z_mm,
                r_mm,
                initial_z_mm,
                covers,
                ..
            }) = &items[i]
            else {
                // Only the lift to clearance precedes the first cycle.
                assert!(hole == 0 && i == 0, "{:?}", items[i]);
                i += 1;
                continue;
            };
            assert_eq!(*center_mm, centers[hole]);
            assert_eq!((*z_mm, *r_mm, *initial_z_mm), (z, r, clearance));
            let covered = &items[i + 1..i + 1 + *covers as usize];
            let expected = linuxcnc_cycle(
                clearance,
                r,
                z,
                peck,
                mode == PeckMode::ChipBreak,
                0.254,
                dwell.is_some(),
            );
            // Positioning at the initial level, then the Z moves.
            let Ok(first) = &covered[0] else { panic!() };
            assert_eq!(
                first.curve.end(),
                [centers[hole][0], centers[hole][1], clearance]
            );
            assert_eq!(first.kind, MoveKind::Rapid);
            let mut k = 1;
            for (end, rapid, dwell_after) in expected {
                let Ok(m) = &covered[k] else {
                    panic!("{:?}", covered[k])
                };
                assert_eq!(m.curve.end(), [centers[hole][0], centers[hole][1], end]);
                assert_eq!(m.kind == MoveKind::Rapid, rapid);
                assert_eq!(m.feed_mm_min.is_none(), rapid);
                k += 1;
                if dwell_after {
                    assert!(matches!(covered[k], Err(Event::Dwell { .. })));
                    k += 1;
                }
            }
            assert_eq!(k, covered.len());
            hole += 1;
            i += 1 + *covers as usize;
        }
        assert_eq!(hole, centers.len());
    }
    let mut op = drilling();
    op.canned_cycle = true;
    op.peck_mm = Some(1.);
    op.dwell_s = Some(1.);
    assert!(drill_plan(op).err().unwrap().contains("no dwell"));
}

#[test]
fn rapids_above_a_committed_plunge_are_certified_clear_of_stock() {
    use camlib::{stock_contact, verification::Status};
    let cancel = AtomicBool::new(false);
    let f = fixture();
    let assembly = f.tools[0].clone();
    let tool = assembly.prepare().unwrap();
    let mut stock = Stock::new(f.stock.height().unwrap().clone()).unwrap();
    stock.enable_contact_queries(&cancel).unwrap();
    let top = f.stock.height().unwrap().max_mm[2];
    let c = [10., 12.];
    let motion = |id: u64, kind: MoveKind, a: f64, b: f64, xy: [f64; 2]| Motion {
        id,
        operation: "holes".into(),
        tool: assembly.id.clone(),
        curve: Curve::Line {
            start: [xy[0], xy[1], a],
            end: [xy[0], xy[1], b],
        },
        kind,
        feed_mm_min: (kind != MoveKind::Rapid).then_some(100.),
    };
    let plunge = motion(1, MoveKind::Plunge, top + 1., top - 3., c);
    let d = stock
        .preview_sweep(&tool, &plunge, 1., f.sweep, &cancel)
        .unwrap();
    stock.apply_delta(&d).unwrap();
    let status = |stock: &Stock, m: &Motion| {
        stock_contact::check(stock, &tool, m, Default::default(), &cancel)
            .unwrap()
            .status
    };
    // Out of the hole, and back down to just above its bottom.
    assert_eq!(
        status(&stock, &motion(2, MoveKind::Rapid, top - 3., top + 1., c)),
        Status::Clear
    );
    assert_eq!(
        status(&stock, &motion(2, MoveKind::Rapid, top + 1., top - 2.7, c)),
        Status::Clear
    );
    // Not below the plunge, not beside it.
    assert_ne!(
        status(&stock, &motion(2, MoveKind::Rapid, top + 1., top - 3.5, c)),
        Status::Clear
    );
    assert_ne!(
        status(
            &stock,
            &motion(2, MoveKind::Rapid, top + 1., top - 2., [c[0] + 0.5, c[1]])
        ),
        Status::Clear
    );
    // Undo restores material: the certificate goes with it.
    stock.undo_delta(&d).unwrap();
    assert_ne!(
        status(&stock, &motion(2, MoveKind::Rapid, top - 3., top + 1., c)),
        Status::Clear
    );
}

#[test]
fn canned_boring_runs_from_r_with_spindle_stop_events() {
    use camlib::holes::{ReamingOperation, ReamingPlan};
    let cancel = AtomicBool::new(false);
    let f = fixture();
    let centers = drilling().centers_mm;
    let mut stock = Stock::new(f.stock.height().unwrap().clone()).unwrap();
    // Drill the pilots first (explicit moves).
    let mut drill = drill_plan(drilling()).unwrap();
    while let Some(m) = drill.next_motion_with_stock(&stock, &cancel).unwrap() {
        commit(&mut stock, &m);
    }
    let top = f.stock.height().unwrap().max_mm[2];
    let op: ReamingOperation = serde_json::from_value(serde_json::json!({
        "id": "bore", "tool": f.tools[1].id, "centers_mm": centers, "pilot_diameter_mm": 1.0,
        "depth_z_mm": top - 2., "clearance_z_mm": top + 2., "retract_z_mm": top + 1.,
        "feed_mm_min": 100., "retract_feed_mm_min": 300., "spindle_rpm": 800., "tolerance_mm": 1e-5,
        "work_offset": 1, "tool_number": 2, "length_register": 2, "coolant": "flood",
        "spindle_stop": true, "canned_cycle": true
    }))
    .unwrap();
    let mut p = ReamingPlan::prepare(
        op,
        &f.tools[1],
        f.stock.height().unwrap(),
        stock_position(&stock, f.initial_position_mm),
        100,
        &cancel,
    )
    .unwrap();
    let mut items: Vec<Result<Motion, Event>> = p.take_events().into_iter().map(Err).collect();
    while let Some(m) = p.next_motion_with_stock(&stock, &cancel).unwrap() {
        commit(&mut stock, &m);
        items.push(Ok(m));
        items.extend(p.take_events().into_iter().map(Err));
    }
    let cycles: Vec<usize> = (0..items.len())
        .filter(|&i| matches!(items[i], Err(Event::Cycle { .. })))
        .collect();
    assert_eq!(cycles.len(), centers.len());
    for &i in &cycles {
        let Err(Event::Cycle {
            cycle,
            covers,
            r_mm,
            initial_z_mm,
            ..
        }) = &items[i]
        else {
            unreachable!()
        };
        assert_eq!(*cycle, DrillCycle::BoreStop);
        assert_eq!(r_mm, initial_z_mm);
        let covered = &items[i + 1..i + 1 + *covers as usize];
        let shape: Vec<&str> = covered
            .iter()
            .map(|x| match x {
                Ok(m) if m.kind == MoveKind::Plunge => "plunge",
                Ok(m) if m.kind == MoveKind::Rapid => "rapid",
                Err(Event::Spindle { rpm, .. }) if *rpm == 0. => "stop",
                Err(Event::Spindle { .. }) => "start",
                other => panic!("{other:?}"),
            })
            .collect();
        let tail = ["plunge", "stop", "rapid", "start"];
        assert!(
            shape.ends_with(&tail) && shape.len() <= tail.len() + 1,
            "{shape:?}"
        );
    }
}

fn stock_position(_: &Stock, initial: Vec3) -> Vec3 {
    // The drill finished at its clearance above the last hole.
    let d = drilling();
    let c = d.centers_mm[d.centers_mm.len() - 1];
    let _ = initial;
    [c[0], c[1], d.clearance_z_mm]
}

fn checked_drill(op: DrillingOperation) -> (DrillingPlan, Stock, Vec<Motion>) {
    let cancel = AtomicBool::new(false);
    let f = fixture();
    let tool = f.tools[0].prepare().unwrap();
    let mut stock = Stock::new(f.stock.height().unwrap().clone()).unwrap();
    stock.enable_contact_queries(&cancel).unwrap();
    let mut plan = drill_plan(op).unwrap();
    let mut moves = Vec::new();
    while let Some(m) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        if m.kind.is_rapid() && m.curve.start()[..2] != m.curve.end()[..2] {
            let proof = camlib::stock_contact::check(&stock, &tool, &m, Default::default(), &cancel).unwrap();
            assert_eq!(proof.status, camlib::verification::Status::Clear);
        }
        commit(&mut stock, &m);
        moves.push(m);
    }
    plan.finish_report(&cancel).unwrap();
    (plan, stock, moves)
}

#[test]
fn r_plane_drilling_keeps_plunges_stock_and_exit_and_uses_checked_low_rapids() {
    for canned in [false, true] {
        for mode in [PeckMode::FullRetract, PeckMode::ChipBreak] {
            let mut op = drilling();
            op.canned_cycle = canned;
            op.peck_mode = mode;
            let (_, old, before) = checked_drill(op.clone());
            op.r_plane_travel = true;
            let (plan, new, after) = checked_drill(op);
            assert_eq!(before.last().unwrap().curve.end(), after.last().unwrap().curve.end());
            let plunges = |rows: &[Motion]| rows.iter().filter(|m| m.kind == MoveKind::Plunge)
                .map(|m| serde_json::to_string(&(&m.curve, m.feed_mm_min)).unwrap()).collect::<Vec<_>>();
            assert_eq!(plunges(&before), plunges(&after));
            for (a, b) in old.cells().iter().zip(new.cells()) {
                assert_eq!((a.lower_mm, a.upper_mm), (b.lower_mm, b.upper_mm));
            }
            let report = plan.report().r_plane_travel.as_ref().unwrap();
            assert_eq!(report.candidate_links, 1);
            assert_eq!(report.z_mm, 11.);
        }
    }
}

#[test]
fn decreasing_pecks_keep_full_depth_and_stock_in_both_retract_modes() {
    for mode in [PeckMode::FullRetract, PeckMode::ChipBreak] {
        let mut op = drilling();
        op.peck_mode = mode;
        op.peck_mm = Some(1.);
        let (_, old, _) = checked_drill(op.clone());
        op.peck_mm = None;
        op.peck_schedule = Some(PeckSchedule { first_mm: 3., decrement_mm: 1., minimum_mm: 1., max_pecks: 10 });
        let (plan, new, moves) = checked_drill(op.clone());
        assert_eq!(moves.iter().filter(|m| m.kind == MoveKind::Plunge).map(|m| m.curve.end()[2]).collect::<Vec<_>>(), vec![7.,5.,4.,7.,5.,4.]);
        assert_eq!(plan.report().pecks_per_hole, 3);
        assert_eq!(plan.report().peck_returns, 4);
        for (a, b) in old.cells().iter().zip(new.cells()) {
            assert_eq!((a.lower_mm, a.upper_mm), (b.lower_mm, b.upper_mm));
        }
        op.peck_schedule = Some(PeckSchedule { first_mm: 4., decrement_mm: 1., minimum_mm: 3., max_pecks: 10 });
        let (_, _, moves) = checked_drill(op);
        assert_eq!(moves.iter().filter(|m| m.kind == MoveKind::Plunge).map(|m| m.curve.end()[2]).collect::<Vec<_>>(), vec![6.,4.,6.,4.]);
    }
}

#[test]
fn peck_schedules_are_bounded_validated_and_cancellable_before_motion() {
    use std::sync::atomic::{AtomicUsize, Ordering};
    struct CancelAfter(AtomicUsize);
    impl CancelSignal for CancelAfter {
        fn is_cancelled(&self) -> bool { self.0.fetch_add(1, Ordering::Relaxed) >= 2 }
    }
    let mut op = drilling();
    op.peck_mm = None;
    for schedule in [
        PeckSchedule { first_mm: 1., decrement_mm: 0., minimum_mm: 1., max_pecks: 2 },
        PeckSchedule { first_mm: 1., decrement_mm: -1., minimum_mm: 1., max_pecks: 10 },
        PeckSchedule { first_mm: 1., decrement_mm: 0., minimum_mm: 2., max_pecks: 10 },
    ] {
        op.peck_schedule = Some(schedule);
        assert!(drill_plan(op.clone()).is_err());
    }
    op.peck_schedule = Some(PeckSchedule { first_mm: 1., decrement_mm: 0., minimum_mm: 1., max_pecks: 10 });
    let f = fixture();
    let prepare = |op| DrillingPlan::prepare(op, &f.tools[0], f.stock.height().unwrap(), f.initial_position_mm, 1, &CancelAfter(AtomicUsize::new(0)));
    assert!(prepare(op.clone()).err().unwrap().contains("peck schedule cancelled"));
    op.canned_cycle = true;
    assert!(drill_plan(op.clone()).err().unwrap().contains("explicit drilling"));
    op.peck_schedule = None;
    op.peck_mm = Some(0.001);
    // Valid controller returns must reach the inner preparation loop, not
    // be rejected by the existing return-distance guard first.
    op.reentry_clearance_mm = 0.0002;
    op.chip_break_lift_mm = 0.0002;
    let error = prepare(op.clone()).err().unwrap();
    assert!(error.contains("canned peck schedule cancelled"), "{error}");
    op.tolerance_mm = 1e-7;
    op.peck_mm = Some(0.000002);
    op.reentry_clearance_mm = 0.0000018;
    op.chip_break_lift_mm = 0.0000018;
    let error = drill_plan(op).err().unwrap();
    assert!(error.contains("covered-event limit"), "{error}");
}

fn boring_bar(f: &StreamManifest) -> camlib::tool::ToolAssembly {
    // 6 mm bore, 3 mm bar 20 mm long (inside the 4 mm pilot), 10 mm shank.
    let mut t = f.tools[1].clone();
    t.id = "bar6".into();
    t.cutter.id = "bar6".into();
    t.cutter.diameter_mm = 6.;
    t.cutter.cutting_length_mm = 2.;
    t.cutter.center_cutting = false;
    t.cutter.shank_diameter_mm = 10.;
    t.cutter.neck = Some(camlib::tool::Neck {
        diameter_mm: 3.,
        length_mm: 20.,
    });
    t.cutter.overall_length_mm = 45.;
    for h in &mut t.holders {
        h.radius_min_mm = 8.;
        h.radius_max_mm = 8.;
    }
    t
}

fn fine_bore_items(canned: bool, shift: f64) -> Result<Vec<Result<Motion, Event>>, String> {
    use camlib::holes::{ReamingOperation, ReamingPlan};
    use camlib::{stock_contact, verification::Status};
    let cancel = AtomicBool::new(false);
    let f = fixture();
    let centers = drilling().centers_mm;
    let mut stock = Stock::new(f.stock.height().unwrap().clone()).unwrap();
    stock.enable_contact_queries(&cancel).unwrap();
    let mut drill = drill_plan(drilling()).unwrap();
    while let Some(m) = drill.next_motion_with_stock(&stock, &cancel).unwrap() {
        commit(&mut stock, &m);
    }
    let bar = boring_bar(&f);
    let tool = bar.prepare().unwrap();
    let top = f.stock.height().unwrap().max_mm[2];
    let op: ReamingOperation = serde_json::from_value(serde_json::json!({
        "id": "bore", "tool": bar.id, "centers_mm": centers, "pilot_diameter_mm": 4.0,
        "depth_z_mm": top - 3., "clearance_z_mm": top + 2., "retract_z_mm": top + 1.,
        "feed_mm_min": 100., "retract_feed_mm_min": 300., "spindle_rpm": 800., "tolerance_mm": 1e-5,
        "work_offset": 1, "tool_number": 3, "length_register": 3, "coolant": "flood",
        "canned_cycle": canned, "dwell_s": 0.5,
        "fine_bore": {"shift_mm": shift, "edge_angle_deg": 90.0}
    }))
    .unwrap();
    let mut p = ReamingPlan::prepare(
        op,
        &bar,
        f.stock.height().unwrap(),
        stock_position(&stock, f.initial_position_mm),
        100,
        &cancel,
    )?;
    let mut items: Vec<Result<Motion, Event>> = p.take_events().into_iter().map(Err).collect();
    while let Some(m) = p.next_motion_with_stock(&stock, &cancel)? {
        // Every move, the stopped bar's shifts included, is certified clear
        // of the stock (no rubbing of the finished wall).
        let report = stock_contact::check(&stock, &tool, &m, Default::default(), &cancel).unwrap();
        assert_eq!(report.status, Status::Clear, "{m:?} {report:?}");
        let d = stock
            .preview_sweep(&tool, &m, 1., f.sweep, &cancel)
            .unwrap();
        if m.kind == MoveKind::Shift {
            assert!(d.changes().is_empty(), "a shift removes nothing");
        }
        stock.apply_delta(&d).unwrap();
        items.push(Ok(m));
        items.extend(p.take_events().into_iter().map(Err));
    }
    Ok(items)
}

#[test]
fn fine_boring_shifts_away_from_the_edge_inside_the_bore() {
    let top = fixture().stock.height().unwrap().max_mm[2];
    for canned in [false, true] {
        let items = fine_bore_items(canned, 0.5).unwrap();
        let shape: Vec<String> = items
            .iter()
            .map(|x| match x {
                Ok(m) => format!("{:?}", m.kind).to_lowercase(),
                Err(Event::Spindle { rpm, .. }) if *rpm == 0. => "stop".into(),
                Err(Event::Spindle { .. }) => "start".into(),
                Err(Event::SpindleOrient { edge_angle_deg }) => {
                    assert_eq!(*edge_angle_deg, 90.);
                    "orient".into()
                }
                Err(Event::Dwell { .. }) => "dwell".into(),
                Err(Event::Cycle {
                    cycle, shift_mm, ..
                }) => {
                    assert_eq!(*cycle, DrillCycle::FineBore);
                    assert_eq!(*shift_mm, Some(0.5));
                    "cycle".into()
                }
                Err(e) => format!("{e:?}"),
            })
            .collect();
        let hole = "plunge dwell orient shift shift shift start";
        assert_eq!(
            shape.join(" ").matches(hole).count(),
            2,
            "canned {canned}: {shape:?}"
        );
        // Edge along +Y: the bar shifts 0.5 mm toward -Y at depth, rises
        // to R there and shifts back over the hole.
        let shifts: Vec<&Motion> = items
            .iter()
            .filter_map(|x| x.as_ref().ok().filter(|m| m.kind == MoveKind::Shift))
            .collect();
        let (a, b) = (shifts[0].curve.start(), shifts[0].curve.end());
        assert!((a[1] - b[1] - 0.5).abs() < 1e-12 && a[0] == b[0] && a[2] == top - 3.);
        assert!(shifts.iter().all(|m| m.feed_mm_min.is_none()));
        assert_eq!(shifts[2].curve.end()[1], a[1]);
    }
    // The bar (3 mm in a 6 mm bore) leaves at most 1.5 mm of room.
    let err = fine_bore_items(false, 1.6).err().unwrap();
    assert!(err.contains("fine_bore shift_mm"), "{err}");
}

#[test]
fn fine_boring_needs_a_bar_diameter_and_an_oriented_stop() {
    use camlib::holes::{ReamingOperation, ReamingPlan};
    let f = fixture();
    let mut op: ReamingOperation = serde_json::from_value(serde_json::json!({
        "id": "bore", "tool": f.tools[1].id, "centers_mm": [[10.0, 12.0]], "pilot_diameter_mm": 2.0,
        "depth_z_mm": 7., "clearance_z_mm": 12., "feed_mm_min": 100., "retract_feed_mm_min": 300.,
        "spindle_rpm": 800., "tolerance_mm": 1e-5, "work_offset": 1, "tool_number": 2,
        "length_register": 2, "coolant": "flood", "fine_bore": {"shift_mm": 0.2, "edge_angle_deg": 0.0}
    }))
    .unwrap();
    let prepare = |op: ReamingOperation, tool: &camlib::tool::ToolAssembly| {
        ReamingPlan::prepare(
            op,
            tool,
            f.stock.height().unwrap(),
            [0., 0., 20.],
            1,
            &AtomicBool::new(false),
        )
        .err()
        .unwrap()
    };
    assert!(prepare(op.clone(), &f.tools[1]).contains("neck"));
    op.spindle_stop = true;
    assert!(prepare(op, &f.tools[1]).contains("drop spindle_stop"));
    // A shift without an oriented stop, or toward the edge, is refused.
    let program = |events: Vec<Event>| {
        let mut p = Program {
            version: 1,
            id: "p".into(),
            revision: "1".into(),
            units: "mm".into(),
            tolerance_mm: 1e-5,
            initial_position_mm: [0., 0., 5.],
            frame: WorkFrame::identity(),
            tools: vec![f.tools[1].clone()],
            events: vec![],
        };
        let mut all = vec![
            Event::WorkOffset { number: 1 },
            Event::ToolChange {
                tool: f.tools[1].id.clone(),
                number: 2,
                length_register: 2,
                position_after_mm: [0., 0., 5.],
            },
        ];
        all.extend(events);
        p.events = all;
        ProgramValidator::new(&p, None).and_then(|mut v| v.consume(&p.events))
    };
    let shift = |id: u64, a: Vec3, b: Vec3| Event::Motion {
        motion: Motion {
            id,
            operation: "bore".into(),
            tool: f.tools[1].id.clone(),
            kind: MoveKind::Shift,
            curve: Curve::Line { start: a, end: b },
            feed_mm_min: None,
        },
    };
    let orient = Event::SpindleOrient { edge_angle_deg: 0. };
    assert!(
        program(vec![shift(1, [0., 0., 5.], [-0.2, 0., 5.])])
            .unwrap_err()
            .contains("oriented spindle stop first")
    );
    assert!(
        program(vec![orient.clone(), shift(1, [0., 0., 5.], [0.2, 0., 5.])])
            .unwrap_err()
            .contains("away from the cutting edge")
    );
    program(vec![
        orient.clone(),
        shift(1, [0., 0., 5.], [-0.2, 0., 5.]),
        shift(2, [-0.2, 0., 5.], [-0.2, 0., 8.]),
        shift(3, [-0.2, 0., 8.], [0., 0., 8.]),
        Event::Spindle {
            rpm: 800.,
            direction: SpindleDirection::Cw,
        },
    ])
    .unwrap();
    assert!(
        program(vec![
            orient,
            shift(1, [0., 0., 5.], [-0.2, 0., 5.]),
            Event::Dwell { seconds: 1. }
        ])
        .unwrap_err()
        .contains("only oriented-spindle shifts")
    );
}

#[test]
fn oriented_shift_is_certified_only_inside_its_own_bore() {
    use camlib::{stock_contact, verification::Status};
    let cancel = AtomicBool::new(false);
    let f = fixture();
    let bar = boring_bar(&f);
    let tool = bar.prepare().unwrap();
    let mut stock = Stock::new(f.stock.height().unwrap().clone()).unwrap();
    stock.enable_contact_queries(&cancel).unwrap();
    let top = f.stock.height().unwrap().max_mm[2];
    let line = |id: u64, kind: MoveKind, a: Vec3, b: Vec3| Motion {
        id,
        operation: "bore".into(),
        tool: bar.id.clone(),
        curve: Curve::Line { start: a, end: b },
        kind,
        feed_mm_min: (kind == MoveKind::Plunge).then_some(100.),
    };
    let status = |stock: &Stock, m: &Motion| {
        stock_contact::check(stock, &tool, m, Default::default(), &cancel)
            .unwrap()
            .status
    };
    let c = [20., 12.];
    let bottom = [c[0], c[1], top - 3.];
    // No bore yet: not certified.
    assert_ne!(
        status(
            &stock,
            &line(1, MoveKind::Shift, bottom, [c[0], c[1] - 0.5, top - 3.])
        ),
        Status::Clear
    );
    // A solid plunge from above (no pilot needed for this sweep).
    let plunge = line(1, MoveKind::Plunge, [c[0], c[1], top + 1.], bottom);
    let d = stock
        .preview_sweep(&tool, &plunge, 1., f.sweep, &cancel)
        .unwrap();
    stock.apply_delta(&d).unwrap();
    // 0.5 + bar radius 1.5 <= bore radius 3; 1.6 + 1.5 > 3.
    let ok = line(2, MoveKind::Shift, bottom, [c[0], c[1] - 0.5, top - 3.]);
    assert_eq!(status(&stock, &ok), Status::Clear);
    assert_ne!(
        status(
            &stock,
            &line(2, MoveKind::Shift, bottom, [c[0], c[1] - 1.6, top - 3.])
        ),
        Status::Clear
    );
    let d = stock
        .preview_sweep(&tool, &ok, 1., f.sweep, &cancel)
        .unwrap();
    stock.apply_delta(&d).unwrap();
    // Up from the shifted pose stays in the bore; below its bottom does not.
    let s = [c[0], c[1] - 0.5];
    assert_eq!(
        status(
            &stock,
            &line(
                3,
                MoveKind::Shift,
                [s[0], s[1], top - 3.],
                [s[0], s[1], top + 1.]
            )
        ),
        Status::Clear
    );
    assert_ne!(
        status(
            &stock,
            &line(
                3,
                MoveKind::Shift,
                [s[0], s[1], top - 3.],
                [s[0], s[1], top - 3.5]
            )
        ),
        Status::Clear
    );
}


#[test]
fn multistart_drilling_preserves_actual_cuts_stock_and_final_pose() {
    let run=|nearest| {
        let mut manifest=fixture();
        let spec=manifest.stock.height_mut().unwrap();
        spec.max_mm=[100.,100.,10.]; spec.cell_size_mm=0.5;
        let mut op=drilling();
        op.centers_mm=vec![[46.,62.],[38.,86.],[38.,30.],[62.,70.],[14.,54.],
            [94.,46.],[70.,38.],[22.,6.],[62.,38.]];
        let mut settings=ordering(100_000); settings.nearest_neighbor=nearest; op.ordering=Some(settings);
        let cancel=AtomicBool::new(false);
        let mut plan=DrillingPlan::prepare(op,&manifest.tools[0],manifest.stock.height().unwrap(),manifest.initial_position_mm,1,&cancel).unwrap();
        let mut stock=Stock::new(manifest.stock.height().unwrap().clone()).unwrap();
        let mut moves=Vec::new();
        while let Some(m)=plan.next_motion_with_stock(&stock,&cancel).unwrap() { commit(&mut stock,&m); moves.push(m); }
        plan.finish_report(&cancel).unwrap();
        (plan,stock,moves)
    };
    let (old,a,am)=run(false); let (new,b,bm)=run(true);
    let old=old.report(); let new=new.report();
    assert!(new.ordering.as_ref().unwrap().ordered_seconds < 0.95*old.ordering.as_ref().unwrap().ordered_seconds);
    assert_eq!(old.cut_pecks,new.cut_pecks); assert_eq!(old.completed_holes,new.completed_holes);
    assert_eq!(am.last().unwrap().curve.end(),bm.last().unwrap().curve.end());
    for (a,b) in a.cells().iter().zip(b.cells()) { assert_eq!((a.lower_mm,a.upper_mm),(b.lower_mm,b.upper_mm)); }
    let cuts=|moves: &[Motion]| { let mut values:Vec<_>=moves.iter().filter(|m| !m.kind.is_rapid()).map(|m|serde_json::to_string(&(m.kind,&m.curve,m.feed_mm_min)).unwrap()).collect();values.sort();values };
    assert_eq!(cuts(&am),cuts(&bm));
    for m in bm.iter().filter(|m|m.kind.is_rapid() && m.curve.start()[..2]!=m.curve.end()[..2]) { assert!(m.curve.start()[2]>=12. && m.curve.end()[2]>=12.); }
}
