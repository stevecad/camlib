use camlib::{
    adaptive::*, motion::*, planar::*, pocket::Operation, stock::*, stream::StreamManifest,
};
use std::{
    f64::consts::{PI, TAU},
    sync::atomic::AtomicBool,
};
fn fixture() -> StreamManifest {
    serde_json::from_str(include_str!("../fixtures/operations/adaptive-pocket.json")).unwrap()
}
fn operation() -> AdaptivePocketOperation {
    let Operation::AdaptivePocket(p) = fixture().operations.remove(0) else {
        panic!("fixture")
    };
    p
}
fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}
fn angle_on(a: f64, start: f64, sweep: f64) -> bool {
    if sweep.abs() >= TAU - 1e-10 {
        return true;
    }
    let d = if sweep > 0. {
        (a - start).rem_euclid(TAU)
    } else {
        (start - a).rem_euclid(TAU)
    };
    d <= sweep.abs() + 1e-10 || d >= TAU - 1e-10
}
// Independent full-curve extrema from analytic line/circle stationary points.
fn curve_distance(c: &Curve, p: [f64; 2], maximum: bool) -> f64 {
    let a = c.start();
    let b = c.end();
    match *c {
        Curve::Line { .. } => {
            if maximum {
                return dist([a[0], a[1]], p).max(dist([b[0], b[1]], p));
            }
            let d = [b[0] - a[0], b[1] - a[1]];
            let l = d[0] * d[0] + d[1] * d[1];
            let t = if l > 0. {
                ((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / l
            } else {
                0.
            };
            let t = t.clamp(0., 1.);
            dist([a[0] + t * d[0], a[1] + t * d[1]], p)
        }
        Curve::Arc {
            center, sweep_rad, ..
        } => {
            let r = dist([a[0], a[1]], [center[0], center[1]]);
            let start = (a[1] - center[1]).atan2(a[0] - center[0]);
            let direction =
                (p[1] - center[1]).atan2(p[0] - center[0]) + if maximum { PI } else { 0. };
            let mut value = if maximum {
                dist([a[0], a[1]], p).max(dist([b[0], b[1]], p))
            } else {
                dist([a[0], a[1]], p).min(dist([b[0], b[1]], p))
            };
            if angle_on(direction, start, sweep_rad) {
                let d = dist([center[0], center[1]], p);
                value = if maximum {
                    value.max(d + r)
                } else {
                    value.min((d - r).abs())
                };
            }
            value
        }
        _ => panic!("unexpected helix"),
    }
}
fn exposed(q: f64, r: f64, a: f64) -> f64 {
    if q + r <= a {
        return 0.;
    }
    if q >= a + r {
        return 360.;
    }
    let x = (q * q + r * r - a * a) / (2. * q);
    2. * (-x / r).clamp(-1., 1.).acos().to_degrees()
}
fn notch_clearance(p: [f64; 2]) -> f64 {
    (11. - p[0]).max(0.).hypot((11. - p[1]).max(0.))
}
fn l_clearance(p: [f64; 2]) -> f64 {
    (p[0] - 3.)
        .min(p[1] - 3.)
        .min(23. - p[0])
        .min(23. - p[1])
        .min(notch_clearance(p))
}
fn assert_entire_l_sweep(c: &Curve, r: f64) {
    let a = c.start();
    let b = c.end();
    let mut points = vec![[a[0], a[1]], [b[0], b[1]]];
    match *c {
        Curve::Line { .. } => {
            let d = [b[0] - a[0], b[1] - a[1]];
            let l = d[0] * d[0] + d[1] * d[1];
            let mut ts = vec![];
            if l > 0. {
                ts.push(((11. - a[0]) * d[0] + (11. - a[1]) * d[1]) / l);
            }
            for k in 0..2 {
                if d[k] != 0. {
                    ts.push((11. - a[k]) / d[k]);
                }
            }
            points.extend(
                ts.into_iter()
                    .filter(|t| (0. ..=1.).contains(t))
                    .map(|t| [a[0] + t * d[0], a[1] + t * d[1]]),
            );
        }
        Curve::Arc {
            center, sweep_rad, ..
        } => {
            let rho = dist([a[0], a[1]], [center[0], center[1]]);
            let start = (a[1] - center[1]).atan2(a[0] - center[0]);
            let mut angles = vec![
                0.,
                PI / 2.,
                PI,
                3. * PI / 2.,
                (11. - center[1]).atan2(11. - center[0]),
            ];
            let x = (11. - center[0]) / rho;
            if x.abs() <= 1. {
                angles.extend([x.acos(), -x.acos()]);
            }
            let y = (11. - center[1]) / rho;
            if y.abs() <= 1. {
                angles.extend([y.asin(), PI - y.asin()]);
            }
            points.extend(
                angles
                    .into_iter()
                    .filter(|x| angle_on(*x, start, sweep_rad))
                    .map(|x| [center[0] + rho * x.cos(), center[1] + rho * x.sin()]),
            );
        }
        _ => panic!("unexpected helix"),
    }
    for p in points {
        assert!(
            l_clearance(p) >= r - 1e-7,
            "whole L sweep exits at{p:?}: {c:?}"
        );
    }
}
fn exact_height(c: &Curve, p: [f64; 2]) -> f64 {
    if curve_distance(c, p, false) <= 2. + 1e-10 {
        c.start()[2].min(c.end()[2])
    } else {
        f64::INFINITY
    }
}
fn unrotate_curve(mut c: Curve) -> Curve {
    let transform = |p: &mut Vec3| {
        let old = *p;
        p[0] = old[1];
        p[1] = 26. - old[0];
    };
    match &mut c {
        Curve::Line { start, end } => {
            transform(start);
            transform(end);
        }
        Curve::Arc {
            start, end, center, ..
        } => {
            transform(start);
            transform(end);
            transform(center);
        }
        _ => panic!("unexpected helix"),
    }
    c
}
fn curve_tangent(c: &Curve, end: bool) -> [f64; 2] {
    let a = c.start();
    let b = c.end();
    let d = match *c {
        Curve::Line { .. } => [b[0] - a[0], b[1] - a[1]],
        Curve::Arc {
            center, sweep_rad, ..
        } => {
            let p = if end { b } else { a };
            [
                -(p[1] - center[1]) * sweep_rad.signum(),
                (p[0] - center[0]) * sweep_rad.signum(),
            ]
        }
        _ => panic!("unexpected helix"),
    };
    let n = d[0].hypot(d[1]);
    [d[0] / n, d[1] / n]
}
#[test]
fn l_pocket_has_continuous_engagement_containment_coverage_and_actual_stock() {
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let tool = m.tools[0].prepare().unwrap();
    for (cw, angle) in [(true, 90.), (false, 60.)] {
        let mut op = operation();
        op.clockwise = cw;
        op.max_engagement_deg = angle;
        op.pocket.floor_z_mm = 9.;
        if !cw {
            for component in &mut op.pocket.region.components {
                for p in &mut component.outer.vertices {
                    let old = *p;
                    p[0] = 26. - old[1];
                    p[1] = old[0];
                }
            }
        }

        let mut plan = AdaptivePlan::prepare(
            op,
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &cancel,
        )
        .unwrap();
        assert_eq!(
            plan.report().navigation_components,
            1,
            "L route must join through the concavity"
        );
        let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let mut disk = None;
        let mut curves: Vec<Curve> = vec![];
        let mut old = m.initial_position_mm;
        let mut clear = 0;
        let mut seed = 0;
        while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
            assert_eq!(motion.curve.start(), old);
            old = motion.curve.end();
            if motion.kind == MoveKind::Plunge {
                disk = None;
            }
            if motion.kind == MoveKind::Cut
                || motion.kind == MoveKind::Link
                || motion.kind == MoveKind::Plunge
            {
                let original = if cw {
                    motion.curve.clone()
                } else {
                    unrotate_curve(motion.curve.clone())
                };
                assert_entire_l_sweep(&original, 2.);
            }
            if let Some(previous) = disk {
                if matches!(motion.kind, MoveKind::Cut | MoveKind::Link) {
                    let q = curve_distance(&motion.curve, previous, true);
                    if exposed(q, 2., 3.) > angle + 1e-7 {
                        // Independently inspect every potentially touched whole cell.
                        let half = stock.cell_widths_mm()[0].hypot(stock.cell_widths_mm()[1]) / 2.;
                        for (i, h) in stock.cells().iter().enumerate() {
                            if curve_distance(&motion.curve, stock.cell_center(i).unwrap(), false)
                                <= 2. + half + 1e-7
                            {
                                assert!(
                                    h.upper_mm <= motion.curve.start()[2],
                                    "uncertified engagement override: {motion:?}"
                                );
                            }
                        }
                        clear += 1;
                    }
                }
            } else if motion.kind == MoveKind::Cut {
                seed += 1;
                assert_eq!(motion.feed_mm_min, Some(150.));
            }
            if let Curve::Arc {
                center, sweep_rad, ..
            } = motion.curve
                && motion.kind == MoveKind::Cut
            {
                assert!((sweep_rad.abs() - TAU).abs() < 1e-10);
                disk = Some([center[0], center[1]]);
            }
            if let Some(previous) = curves.last()
                && motion.curve.start()[2] == motion.curve.end()[2]
                && previous.start()[2] == previous.end()[2]
                && motion.curve.start()[2] < 10.
            {
                let incoming = curve_tangent(previous, true);
                let outgoing = curve_tangent(&motion.curve, false);
                assert!(
                    incoming[0] * outgoing[0] + incoming[1] * outgoing[1] > 1. - 1e-8,
                    "non-tangent corner: {previous:?} -> {:?}",
                    motion.curve.clone()
                );
            }
            let delta = stock
                .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
                .unwrap();
            stock.apply_delta(&delta).unwrap();
            curves.push(motion.curve);
        }
        plan.finish_report(&cancel).unwrap();
        let report = plan.report();
        assert_eq!(seed, 1);
        assert_eq!(report.seed_orbits, 1);
        assert!(report.turn_arcs > 0);
        assert!(report.max_bounded_engagement_deg <= angle);
        assert!(report.controlled_orbits > 10);
        assert!(
            report
                .geometric_residual_area_mm2
                .is_some_and(|x| x > 0. && x < 15.)
        );
        for (i, h) in stock.cells().iter().enumerate() {
            let xy = stock.cell_center(i).unwrap();
            let actual = curves
                .iter()
                .map(|c| exact_height(c, xy))
                .fold(10f64, f64::min);
            assert!(
                h.lower_mm <= actual + 1e-6 && h.upper_mm >= actual - 1e-6,
                "exact stock outside interval {xy:?}: {actual} {h:?}"
            );
            let original_xy = if cw { xy } else { [xy[1], 26. - xy[0]] };
            if l_clearance(original_xy) > 3.01 {
                assert!(actual <= 9. + 1e-7, "reachable core uncut at{xy:?}");
            }
            if l_clearance(original_xy) < -1e-7 {
                assert_eq!(actual, 10.);
            }
        }
        assert!(stock.volume().removed_lower_mm3 > 180.);
        assert!(stock.volume().removed_lower_mm3 <= 256.);
        eprintln!(
            "cw{cw} angle{angle} motions{} controlled{} clear{clear}",
            report.motions, report.controlled_orbits
        );
    }
}
#[test]
fn cancellation_during_final_residual_keeps_report_pending_and_retryable() {
    use camlib::pocket::ResidualStatus;
    use std::cell::Cell;

    let m = fixture();
    let tool = m.tools[0].prepare().unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut plan = AdaptivePlan::prepare(
        operation(),
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &|| false,
    )
    .unwrap();
    while let Some(motion) = plan.next_motion_with_stock(&stock, &|| false).unwrap() {
        let delta = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &|| false)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
    }
    assert_eq!(
        plan.report().geometric_residual_status,
        ResidualStatus::Pending
    );
    let pending = serde_json::to_value(plan.report()).unwrap();
    // Cancel between entry and publication, without a timing-dependent thread.
    let entered = Cell::new(false);
    let cancel = || entered.replace(true);
    assert!(
        plan.finish_report(&cancel)
            .unwrap_err()
            .contains("cancelled")
    );
    assert_eq!(serde_json::to_value(plan.report()).unwrap(), pending);

    plan.finish_report(&|| false).unwrap();
    assert_eq!(
        plan.report().geometric_residual_status,
        ResidualStatus::Estimated
    );
    assert!(plan.report().geometric_residual.is_some());
    assert!(plan.report().geometric_residual_area_mm2.is_some());
    assert_eq!(
        serde_json::to_value(plan.report()).unwrap()["motions"],
        pending["motions"]
    );
    let completed = serde_json::to_value(plan.report()).unwrap();
    plan.finish_report(&|| false).unwrap();
    assert_eq!(serde_json::to_value(plan.report()).unwrap(), completed);
}

#[test]
fn cancel_stale_stock_and_invalid_parameters_do_not_consume_the_cursor() {
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut plan = AdaptivePlan::prepare(
        operation(),
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    let first = plan
        .next_motion_with_stock(&stock, &cancel)
        .unwrap()
        .unwrap();
    assert!(
        plan.next_motion_with_stock(&stock, &cancel)
            .unwrap_err()
            .contains("commit")
    );
    let delta = stock
        .preview_sweep(&m.tools[0].prepare().unwrap(), &first, 1., m.sweep, &cancel)
        .unwrap();
    stock.apply_delta(&delta).unwrap();
    cancel.store(true, std::sync::atomic::Ordering::Relaxed);
    assert!(plan.next_motion_with_stock(&stock, &cancel).is_err());
    cancel.store(false, std::sync::atomic::Ordering::Relaxed);
    let second = plan
        .next_motion_with_stock(&stock, &cancel)
        .unwrap()
        .unwrap();
    assert_eq!(second.id, first.id + 1);
    assert_eq!(second.curve.start(), first.curve.end());
    for radius in [0., -1., 3., f64::NAN] {
        let mut op = operation();
        op.orbit_radius_mm = Some(radius);
        assert!(
            AdaptivePlan::prepare(
                op,
                &m.tools[0],
                m.stock.height().unwrap(),
                m.initial_position_mm,
                1,
                &cancel
            )
            .is_err()
        );
    }
}

#[test]
fn circular_island_and_separate_components_keep_their_topology_and_single_seeds() {
    check_circular_island_and_separate_components(false);
}

#[test]
fn stock_front_circular_island_and_separate_components_complete_with_actual_stock() {
    check_circular_island_and_separate_components(true);
}

fn check_circular_island_and_separate_components(front: bool) {
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let tool = m.tools[0].prepare().unwrap();
    for (annulus, local) in [(true, false), (false, false), (true, true), (false, true)] {
        let mut op = operation();
        if front {
            op.path_strategy = AdaptivePathStrategy::StockFront;
        }
        if local {
            op.min_orbit_radius_mm = Some(0.25);
            op.local_orbit_tolerance_mm = Some(0.05);
        }
        op.pocket.floor_z_mm = 9.;
        op.pocket.stepover_mm = 1.;
        let (source, expected) = if annulus {
            let mut source = PlanarRegion::circle("annulus", [13., 13.], 10.).unwrap();
            source.components[0].holes.push(
                PlanarRegion::circle("hole", [13., 13.], 3.)
                    .unwrap()
                    .components
                    .remove(0)
                    .outer,
            );
            (source, 1)
        } else {
            let mut source = PlanarRegion::circle("two-pockets", [7., 13.], 4.).unwrap();
            source.components.extend(
                PlanarRegion::circle("second", [19., 13.], 4.)
                    .unwrap()
                    .components,
            );
            (source, 2)
        };
        op.pocket.region = source;
        let max_angle = op.max_engagement_deg;
        let stock_guard = m.stock.height().unwrap().cell_size_mm * 2_f64.sqrt() / 2.
            + op.boundary_tolerance_mm
            + 4. * m.sweep.tolerance_mm;
        let mut plan = AdaptivePlan::prepare(
            op,
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &cancel,
        )
        .unwrap();
        assert_eq!(plan.report().navigation_components, expected);
        let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let mut motions = 0;
        let mut nominal_seconds = 0.;
        while let Some(motion) = plan
            .next_motion_with_stock(&stock, &cancel)
            .unwrap_or_else(|error| {
                panic!(
                    "front={front} annulus={annulus} local={local} after{motions}moves: {error}; {}",
                    serde_json::to_string(plan.report()).unwrap()
                )
            })
        {
            if front {
                assert!(motions < 10000, "island/component case did not complete");
            }
            if matches!(
                motion.kind,
                MoveKind::Cut | MoveKind::Link | MoveKind::Plunge
            ) {
                if annulus {
                    assert!(curve_distance(&motion.curve, [13., 13.], false) >= 5. - 1e-7);
                    assert!(curve_distance(&motion.curve, [13., 13.], true) <= 8. + 1e-7);
                } else {
                    assert!([[7., 13.], [19., 13.]].iter().any(|&p| curve_distance(
                        &motion.curve,
                        p,
                        true
                    ) <= 2. + 1e-7));
                }
            }
            let delta = stock
                .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
                .unwrap();
            stock.apply_delta(&delta).unwrap();
            nominal_seconds += motion.curve.prepare(m.sweep.tolerance_mm).unwrap().length_mm()
                / motion.feed_mm_min.unwrap_or(6000.) * 60.;
            motions += 1;
        }
        plan.finish_report(&cancel).unwrap();
        if front {
            let report = plan.report().front.as_ref().unwrap();
            assert!(report.coverage_complete);
            assert_eq!(report.completed_components, expected);
            assert!(report.max_bounded_engagement_deg <= max_angle);
            assert!(report.front_moves > 0);
            if annulus {
                assert!(motions <= 2500, "annular motion quality: {motions}");
                assert!(
                    report.reentries <= 100,
                    "annular reentry quality: {}",
                    report.reentries
                );
                assert!(
                    nominal_seconds <= 500.,
                    "annular cycle quality: {nominal_seconds}"
                );
                assert!(report.channel_contours > 0);
            } else {
                assert!(motions <= 150, "circular pocket motion quality: {motions}");
                assert!(
                    report.reentries <= 4,
                    "circular pocket reentry quality: {}",
                    report.reentries
                );
                assert!(
                    nominal_seconds <= 60.,
                    "circular pocket cycle quality: {nominal_seconds}"
                );
                assert!(report.enclosed_contours > 0);
            }
            let (mut cleared, mut protected) = (0, 0);
            for (i, h) in stock.cells().iter().enumerate() {
                let p = stock.cell_center(i).unwrap();
                let d = dist(p, [13., 13.]);
                let reachable_interior = if annulus {
                    d > 3. + stock_guard && d < 10. - stock_guard
                } else {
                    [[7., 13.], [19., 13.]]
                        .iter()
                        .any(|&centre| dist(p, centre) < 4. - stock_guard)
                };
                if reachable_interior {
                    cleared += 1;
                    assert!(
                        h.upper_mm <= 9. + 4. * m.sweep.tolerance_mm,
                        "reachable stock remains at{p:?}: {h:?}; annulus={annulus} local={local}"
                    );
                }
                if annulus && d < 3. - stock_guard {
                    protected += 1;
                    assert!(
                        h.lower_mm >= 10. - 4. * m.sweep.tolerance_mm,
                        "protected island removed at{p:?}: {h:?}"
                    );
                }
            }
            assert!(cleared > 100, "remaining-stock check must not be vacuous");
            if annulus {
                assert!(protected > 10, "island check must not be vacuous");
            }
            eprintln!(
                "stock-front annulus={annulus} local={local}: {motions}moves, checked{cleared}cleared/{protected}protected cells; {}",
                serde_json::to_string(report).unwrap()
            );
        }
        assert_eq!(plan.report().seed_orbits, expected);
        assert!(motions > 10);
        assert!(stock.volume().removed_lower_mm3 > 40.);
        // The circular island remains stock, including its center cell.
        if annulus {
            for (i, h) in stock.cells().iter().enumerate() {
                if dist(stock.cell_center(i).unwrap(), [13., 13.]) < 2.5 {
                    assert!(h.upper_mm >= 10.);
                }
            }
        }
    }
}

#[test]
fn stock_front_uses_actual_larger_tool_stock_for_corner_remainders() {
    use camlib::{operation::OperationPlan, tool::PreparedAssembly};

    // This execution helper also independently encloses the entire axis curve
    // in the analytic rectangular source. It does not sample motion poses.
    fn execute(
        plan: &mut OperationPlan,
        stock: &mut Stock,
        tool: &PreparedAssembly,
        sweep: SweepOptions,
        radius: f64,
        mut position: Vec3,
        mut next: u64,
    ) -> (Vec<Motion>, f64) {
        let mut history = Vec::new();
        let mut cutting_length = 0.;
        while let Some(motion) = plan
            .next_motion_with_stock(stock, &|| false)
            .unwrap_or_else(|error| {
                panic!(
                    "upstream/rest incomplete after{}moves at{position:?}: {error}; {}",
                    history.len(),
                    serde_json::to_string(&plan.report()).unwrap()
                )
            })
        {
            assert!(history.len() < 10000, "upstream/rest did not complete");
            assert_eq!(motion.id, next);
            assert_eq!(motion.curve.start(), position);
            let path = motion.curve.prepare(1e-5).unwrap();
            if matches!(
                motion.kind,
                MoveKind::Cut | MoveKind::Link | MoveKind::Plunge
            ) {
                let bounds = path.bounds();
                for axis in 0..2 {
                    assert!(bounds.min[axis] >= 3. + radius - 1e-7);
                    assert!(bounds.max[axis] <= 23. - radius + 1e-7);
                }
                assert!(bounds.min[2] >= 9. - 1e-7);
            }
            if matches!(motion.kind, MoveKind::Cut | MoveKind::Link) {
                cutting_length += path.length_mm();
            }
            let delta = stock
                .preview_sweep(tool, &motion, 1., sweep, &|| false)
                .unwrap();
            stock.apply_delta(&delta).unwrap();
            position = motion.curve.end();
            next = motion.id + 1;
            history.push(motion);
        }
        plan.finish_report(&|| false).unwrap();
        if let OperationPlan::Adaptive(adaptive) = plan {
            let report = adaptive.report();
            let front = report.front.as_ref().unwrap();
            assert!(front.coverage_complete);
            assert!(front.max_bounded_engagement_deg <= report.max_engagement_deg);
        }
        (history, cutting_length)
    }

    let m = fixture();
    let mut op = operation();
    op.path_strategy = AdaptivePathStrategy::StockFront;
    op.pocket.floor_z_mm = 9.;
    op.pocket.region = PlanarRegion {
        version: 1,
        units: "mm".into(),
        name: "upstream rectangular pocket".into(),
        boundary_error_mm: 0.,
        source: serde_json::Value::Null,
        components: vec![Component {
            outer: Boundary {
                vertices: vec![[3., 3., 0.], [23., 3., 0.], [23., 23., 0.], [3., 23., 0.]],
            },
            holes: vec![],
        }],
    };
    let small = &m.tools[0];
    let mut large = small.clone();
    large.id = "upstream-d8".into();
    large.cutter.id = "em8".into();
    large.cutter.diameter_mm = 8.;
    large.cutter.shank_diameter_mm = 8.;
    let mut upstream = op.pocket.clone();
    upstream.id = "upstream".into();
    upstream.tool = large.id.clone();
    upstream.tool_number = 2;
    upstream.length_register = 2;
    upstream.stepover_mm = 3.;
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut rough = OperationPlan::prepare_with_stock(
        Operation::Pocket(upstream),
        &large,
        &stock,
        m.initial_position_mm,
        1,
        &|| false,
    )
    .unwrap();
    let (large_history, _) = execute(
        &mut rough,
        &mut stock,
        &large.prepare().unwrap(),
        m.sweep,
        4.,
        m.initial_position_mm,
        1,
    );
    let before = stock.volume();
    eprintln!("upstream D8 complete: {}moves", large_history.len());
    let last = large_history.last().unwrap();
    let (position, first_id) = (last.curve.end(), last.id + 1);
    let mut fresh_stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut fresh = OperationPlan::prepare_with_stock(
        Operation::AdaptivePocket(op.clone()),
        small,
        &fresh_stock,
        position,
        first_id,
        &|| false,
    )
    .unwrap();
    let (fresh_history, fresh_length) = execute(
        &mut fresh,
        &mut fresh_stock,
        &small.prepare().unwrap(),
        m.sweep,
        2.,
        position,
        first_id,
    );
    eprintln!(
        "fresh D4 complete: {}moves/{fresh_length}mm Cut+Link",
        fresh_history.len()
    );
    let mut rest = OperationPlan::prepare_with_stock(
        Operation::AdaptivePocket(op),
        small,
        &stock,
        position,
        first_id,
        &|| false,
    )
    .unwrap();
    let (rest_history, rest_length) = execute(
        &mut rest,
        &mut stock,
        &small.prepare().unwrap(),
        m.sweep,
        2.,
        position,
        first_id,
    );
    let report = serde_json::to_value(rest.report()).unwrap();
    eprintln!(
        "upstream/rest: large{}moves fresh{}moves/{fresh_length}mm rest{}moves/{rest_length}mm; {report}",
        large_history.len(),
        fresh_history.len(),
        rest_history.len(),
    );
    assert_eq!(report["front"]["coverage_complete"], true);
    assert!(
        report["front"]["max_bounded_engagement_deg"]
            .as_f64()
            .unwrap()
            <= 90.
    );
    assert!(stock.volume().removed_lower_mm3 > before.removed_lower_mm3 + 0.1);
    let floor_cut = |motion: &&Motion| {
        matches!(motion.kind, MoveKind::Cut | MoveKind::Link)
            && motion.curve.start()[2] == 9.
            && motion.curve.end()[2] == 9.
    };
    for probe in [[4., 4.], [22., 4.], [4., 22.], [22., 22.]] {
        let before_distance = large_history
            .iter()
            .filter(|motion| {
                matches!(
                    motion.kind,
                    MoveKind::Cut | MoveKind::Link | MoveKind::Plunge
                )
            })
            .map(|motion| curve_distance(&motion.curve, probe, false))
            .fold(f64::INFINITY, f64::min);
        let after_distance = rest_history
            .iter()
            .filter(floor_cut)
            .map(|motion| curve_distance(&motion.curve, probe, false))
            .fold(f64::INFINITY, f64::min);
        assert!(before_distance > 4. + 64. * m.sweep.tolerance_mm);
        assert!(after_distance < 2. - 64. * m.sweep.tolerance_mm);
    }
    let mut interior = 0;
    for (i, height) in stock.cells().iter().enumerate() {
        let p = stock.cell_center(i).unwrap();
        if p.into_iter().all(|v| (8. ..18.).contains(&v)) {
            interior += 1;
            assert!(height.upper_mm <= 9. + 4. * m.sweep.tolerance_mm);
        }
    }
    assert!(interior > 100);
    assert!(
        rest_length < fresh_length * 0.75,
        "rest cutting travel{rest_length}mm must save against fresh{fresh_length}mm"
    );
}

#[test]
fn precleared_stock_allows_long_advances_but_thin_material_does_not() {
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let tool = m.tools[0].prepare().unwrap();
    for top in [9., 9.000001] {
        let mut snapshot = Stock::new(m.stock.height().unwrap().clone())
            .unwrap()
            .snapshot();
        for h in &mut snapshot.cells {
            h.lower_mm = top;
            h.upper_mm = top;
        }
        let mut stock = Stock::from_snapshot(snapshot).unwrap();
        let mut op = operation();
        op.pocket.floor_z_mm = 9.;
        let mut plan = AdaptivePlan::prepare(
            op,
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &cancel,
        )
        .unwrap();
        while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
            let delta = stock
                .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
                .unwrap();
            stock.apply_delta(&delta).unwrap();
        }
        plan.finish_report(&cancel).unwrap();
        if top == 9. {
            assert!(plan.report().stock_clear_advances > 10);
            assert_eq!(plan.report().bounded_advances, 0);
        } else {
            assert!(plan.report().bounded_advances > 10);
        }
    }
    let op: Operation = Operation::AdaptivePocket(operation());
    let mut wire = serde_json::to_value(op).unwrap();
    assert!(wire.get("id").is_some());
    assert!(wire.get("tool").is_some());
    assert!(wire.get("pocket").is_none());
    let _: Operation = serde_json::from_value(wire.clone()).unwrap();
    wire.as_object_mut()
        .unwrap()
        .insert("made_up".into(), 1.into());
    assert!(serde_json::from_value::<Operation>(wire).is_err());
}

#[test]
fn rejected_clear_chord_restores_engagement_bound_on_a_tight_crescent() {
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let mut op = operation();
    op.pocket.floor_z_mm = 9.;
    op.pocket.stepover_mm = 0.5;
    let mut source = PlanarRegion::circle("tight-crescent", [13., 13.], 3.5).unwrap();
    // Vertical semicircle anchors retain an unsplit left-side arc after erosion.
    source.components[0].outer.vertices = vec![[13., 9.5, 1.], [13., 16.5, 1.]];
    source.components[0].holes.push(
        PlanarRegion::circle("offset-hole", [16., 13.], 0.1)
            .unwrap()
            .components
            .remove(0)
            .outer,
    );
    op.pocket.region = source;
    let mut snapshot = Stock::new(m.stock.height().unwrap().clone())
        .unwrap()
        .snapshot();
    snapshot.cells.fill(HeightBounds {
        lower_mm: 9.,
        upper_mm: 9.,
    });
    let mut stock = Stock::from_snapshot(snapshot).unwrap();
    let tool = m.tools[0].prepare().unwrap();
    let mut plan = AdaptivePlan::prepare(
        op,
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        let delta = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
    }
    plan.finish_report(&cancel).unwrap();
    assert!(plan.report().chord_fallbacks > 0);
    assert!(plan.report().bounded_advances > 0);
    assert!(plan.report().max_bounded_engagement_deg <= 90.);
}

#[test]
fn stock_front_emits_real_committed_source_admitted_material_side_moves() {
    let m = fixture();
    let cancel = || false;
    let tool = m.tools[0].prepare().unwrap();
    for (cw, rotate) in [(true, false), (false, false), (true, true)] {
        let mut op = operation();
        op.path_strategy = AdaptivePathStrategy::StockFront;
        op.cutting_sense = Some(if cw {
            CuttingSense::Climb
        } else {
            CuttingSense::Conventional
        });
        op.clockwise = cw;
        op.pocket.floor_z_mm = 9.;
        if rotate {
            for component in &mut op.pocket.region.components {
                for p in &mut component.outer.vertices {
                    let old = *p;
                    p[0] = 26. - old[1];
                    p[1] = old[0];
                }
            }
        }
        let r = tool.cutter().definition().diameter_mm / 2.;
        let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let mut plan = AdaptivePlan::prepare(
            op.clone(),
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &cancel,
        )
        .unwrap();
        let mut history: Vec<Motion> = vec![];
        let mut count = 0;
        let mut startup_removed = 0.;
        let mut front_length = 0.;
        let mut maximum_front_length: f64 = 0.;
        for _ in 0..100 {
            let next = plan.next_motion_with_stock(&stock, &cancel);
            let motion = match next {
                Ok(Some(m)) => m,
                other => panic!(
                    "front {cw} stopped after {count}: {other:?}; report={:?}, last={:?}",
                    plan.report().front,
                    history.last()
                ),
            };
            if motion.kind != MoveKind::Rapid {
                assert_entire_l_sweep(
                    &if rotate {
                        unrotate_curve(motion.curve.clone())
                    } else {
                        motion.curve.clone()
                    },
                    r,
                );
            }
            let front = plan.report().front.as_ref().unwrap();
            assert_eq!(
                plan.report().max_bounded_engagement_deg,
                front.max_bounded_engagement_deg
            );
            if front.front_moves > count {
                if count == 0 {
                    startup_removed = stock.volume().removed_lower_mm3;
                }
                count = front.front_moves;
                let previous = history.last().unwrap();
                // A proven-clear vertical re-entry ends at a stopped pose with
                // a newly selected heading. Flat consecutive cuts remain G1.
                if previous.curve.start()[2] == previous.curve.end()[2] {
                    let before = curve_tangent(&previous.curve, true);
                    let next = curve_tangent(&motion.curve, false);
                    assert!(
                        before[0] * next[0] + before[1] * next[1] >= 1. - 1e-9,
                        "front tangent discontinuity"
                    );
                } else {
                    assert_eq!(previous.kind, MoveKind::Plunge);
                }
                if let Curve::Arc { start, center, .. } = motion.curve {
                    assert!((start[0] - center[0]).hypot(start[1] - center[1]) >= r / 2. - 1e-8);
                }
                let length = motion
                    .curve
                    .prepare(op.pocket.tolerance_mm)
                    .unwrap()
                    .length_mm();
                front_length += length;
                maximum_front_length = maximum_front_length.max(length);
                // Independent sampled corroboration of material side and peak.
                // Actual admission is whole-interval, never these probes.
                let path = motion.curve.prepare(op.pocket.tolerance_mm).unwrap();
                for i in 0..=8 {
                    let f = i as f64 / 8.;
                    let p = path.position(f).unwrap();
                    let heading = match motion.curve {
                        Curve::Line { start, end } => (end[1] - start[1]).atan2(end[0] - start[0]),
                        Curve::Arc {
                            start,
                            center,
                            sweep_rad,
                            ..
                        } => {
                            (start[1] - center[1]).atan2(start[0] - center[0])
                                + f * sweep_rad
                                + sweep_rad.signum() * PI / 2.
                        }
                        _ => panic!("front helix"),
                    };
                    let mut exposed = 0;
                    for j in 0..720 {
                        let theta = TAU * (j as f64 + 0.5) / 720.;
                        let q = [p[0] + r * theta.cos(), p[1] + r * theta.sin()];
                        let empty = history.iter().any(|old| {
                            old.curve.start()[2] == old.curve.end()[2]
                                && old.curve.end()[2] <= p[2]
                                && old.kind != MoveKind::Rapid
                                && curve_distance(&old.curve, q, false) < r + 1e-9
                        });
                        if !empty {
                            exposed += 1;
                            let cross = heading.cos() * theta.sin() - heading.sin() * theta.cos();
                            assert!(
                                if cw { cross <= 1e-8 } else { cross >= -1e-8 },
                                "wrong side cw={cw} at {q:?}: {motion:?}"
                            );
                        }
                    }
                    assert!(exposed as f64 * 0.5 <= op.max_engagement_deg + 0.5);
                }
            }
            let delta = stock
                .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
                .unwrap();
            stock.apply_delta(&delta).unwrap();
            history.push(motion);
            if count >= 30 {
                break;
            }
        }
        eprintln!(
            "stock-front cw={cw},rotate={rotate}: {:?}, new lower removal={}mm3",
            plan.report().front,
            stock.volume().removed_lower_mm3 - startup_removed
        );
        assert!(count >= 10, "front did not steer");
        assert!(maximum_front_length >= r, "front emitted only tiny steps");
        assert!(
            front_length / count as f64 > r * 0.1,
            "front dominated by tiny steps"
        );
        assert!(
            stock.volume().removed_lower_mm3 > startup_removed + 1.,
            "front removed no useful new volume beyond seed: {startup_removed} -> {}",
            stock.volume().removed_lower_mm3
        );
        assert!(!plan.report().front.as_ref().unwrap().coverage_complete);
        assert!(plan.finish_report(&cancel).is_err());
    }
}

#[test]
fn stock_front_curved_recipes_use_native_commits() {
    let m = fixture();
    let tool = m.tools[0].prepare().unwrap();
    for (annulus, local) in [(true, false), (true, true), (false, false), (false, true)] {
        let mut op = operation();
        op.path_strategy = AdaptivePathStrategy::StockFront;
        op.pocket.floor_z_mm = 9.;
        op.pocket.stepover_mm = 1.;
        if local {
            op.min_orbit_radius_mm = Some(0.25);
            op.local_orbit_tolerance_mm = Some(0.05);
        }
        let mut source =
            PlanarRegion::circle("curved pocket", [13., 13.], if annulus { 10. } else { 4. })
                .unwrap();
        if annulus {
            source.components[0].holes.push(
                PlanarRegion::circle("hole", [13., 13.], 3.)
                    .unwrap()
                    .components
                    .remove(0)
                    .outer,
            );
        }
        op.pocket.region = source;
        let mut plan = AdaptivePlan::prepare(
            op,
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &|| false,
        )
        .unwrap();
        let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let mut checked = false;
        for _ in 0..25 {
            let Some(motion) = plan.next_motion_with_stock(&stock, &|| false).unwrap() else {
                break;
            };
            let report = plan.report().front.as_ref().unwrap();
            if report.channel_committed_moves + report.enclosed_committed_moves > 0 && !checked {
                let contours = (report.channel_contours, report.enclosed_contours);
                let committed = (
                    report.channel_committed_moves,
                    report.enclosed_committed_moves,
                );
                let mut partial = stock.clone();
                let delta = partial
                    .preview_sweep(&tool, &motion, 0.5, m.sweep, &|| false)
                    .unwrap();
                partial.apply_delta(&delta).unwrap();
                assert!(plan.next_motion_with_stock(&partial, &|| false).is_err());
                let mut foreign = stock.clone();
                let delta = foreign
                    .preview_sweep(&tool, &motion, 1., m.sweep, &|| false)
                    .unwrap();
                foreign.apply_delta(&delta).unwrap();
                let foreign = Stock::from_snapshot(foreign.snapshot()).unwrap();
                assert!(plan.next_motion_with_stock(&foreign, &|| false).is_err());
                let report = plan.report().front.as_ref().unwrap();
                assert_eq!(
                    (report.channel_contours, report.enclosed_contours),
                    contours
                );
                assert_eq!(
                    (
                        report.channel_committed_moves,
                        report.enclosed_committed_moves
                    ),
                    committed
                );
                checked = true;
            }
            let delta = stock
                .preview_sweep(&tool, &motion, 1., m.sweep, &|| false)
                .unwrap();
            stock.apply_delta(&delta).unwrap();
        }
        let report = plan.report().front.as_ref().unwrap();
        assert!(
            checked && report.channel_contours + report.enclosed_contours >= 2,
            "annulus={annulus} local={local}: curved recipe not selected: {report:?}"
        );
        assert_eq!(report.reentries, 0);
    }
}

#[test]
fn stock_front_requires_exact_native_full_commit_and_preserves_pending_on_refusal() {
    let m = fixture();
    let tool = m.tools[0].prepare().unwrap();
    let mut op = operation();
    op.path_strategy = AdaptivePathStrategy::StockFront;
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut plan = AdaptivePlan::prepare(
        op,
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &|| false,
    )
    .unwrap();
    let motion = plan
        .next_motion_with_stock(&stock, &|| false)
        .unwrap()
        .unwrap();
    assert!(plan.next_motion_with_stock(&stock, &|| false).is_err());
    let mut wrong = motion.clone();
    wrong.curve = Curve::Line {
        start: motion.curve.start(),
        end: [0., 1., 12.],
    };
    let delta = stock
        .preview_sweep(&tool, &wrong, 1., m.sweep, &|| false)
        .unwrap();
    let mut unrelated = stock.clone();
    unrelated.apply_delta(&delta).unwrap();
    assert!(
        plan.next_motion_with_stock(&unrelated, &|| false)
            .unwrap_err()
            .contains("mismatch")
    );
    let delta = stock
        .preview_sweep(&tool, &motion, 1., m.sweep, &|| false)
        .unwrap();
    let wire = serde_json::to_value(&delta).unwrap();
    assert!(wire.get("receipt").is_none());
    let decoded: StockDelta = serde_json::from_value(wire).unwrap();
    let mut replayed = stock.clone();
    replayed.apply_delta(&decoded).unwrap();
    assert!(plan.next_motion_with_stock(&replayed, &|| false).is_err());
    let mut other_tool = m.tools[0].clone();
    other_tool.cutter.diameter_mm = 3.;
    let other_tool = other_tool.prepare().unwrap();
    let other_delta = stock
        .preview_sweep(&other_tool, &motion, 1., m.sweep, &|| false)
        .unwrap();
    let mut changed_tool = stock.clone();
    changed_tool.apply_delta(&other_delta).unwrap();
    assert!(
        plan.next_motion_with_stock(&changed_tool, &|| false)
            .is_err()
    );
    let mut fractional = stock.clone();
    let partial = fractional
        .preview_sweep(&tool, &motion, 0.5, m.sweep, &|| false)
        .unwrap();
    fractional.apply_delta(&partial).unwrap();
    assert!(plan.next_motion_with_stock(&fractional, &|| false).is_err());
    let mut restored = stock.clone();
    restored.apply_delta(&delta).unwrap();
    let restored = Stock::from_snapshot(restored.snapshot()).unwrap();
    assert!(plan.next_motion_with_stock(&restored, &|| false).is_err());
    let mut undone = stock.clone();
    undone.apply_delta(&delta).unwrap();
    undone.undo_delta(&delta).unwrap();
    undone.apply_delta(&delta).unwrap();
    assert!(plan.next_motion_with_stock(&undone, &|| false).is_err());
    stock.apply_delta(&delta).unwrap();
    // Failed attempts did not consume the correct pending transaction.
    assert!(
        plan.next_motion_with_stock(&stock, &|| false)
            .unwrap()
            .is_some()
    );
}

#[test]
fn stock_front_cancellation_during_search_does_not_advance_pose_or_motion_id() {
    use std::cell::Cell;
    let m = fixture();
    let tool = m.tools[0].prepare().unwrap();
    let mut op = operation();
    op.path_strategy = AdaptivePathStrategy::StockFront;
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut reference_stock = stock.clone();
    let make = || {
        AdaptivePlan::prepare(
            op.clone(),
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &|| false,
        )
        .unwrap()
    };
    let (mut plan, mut reference) = (make(), make());
    loop {
        let motion = plan
            .next_motion_with_stock(&stock, &|| false)
            .unwrap()
            .unwrap();
        let other = reference
            .next_motion_with_stock(&reference_stock, &|| false)
            .unwrap()
            .unwrap();
        assert_eq!(motion, other);
        let delta = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &|| false)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        let other = reference_stock
            .preview_sweep(&tool, &other, 1., m.sweep, &|| false)
            .unwrap();
        reference_stock.apply_delta(&other).unwrap();
        if plan.report().seed_orbits == 1 {
            break;
        }
    }
    let expected = reference
        .next_motion_with_stock(&reference_stock, &|| false)
        .unwrap()
        .unwrap();
    let previous = plan.report().motions;
    let polls = Cell::new(0);
    let cancel = || {
        polls.set(polls.get() + 1);
        polls.get() >= 20
    };
    assert!(
        plan.next_motion_with_stock(&stock, &cancel)
            .unwrap_err()
            .contains("cancelled")
    );
    assert_eq!(plan.report().motions, previous);
    assert_eq!(
        plan.next_motion_with_stock(&stock, &|| false)
            .unwrap()
            .unwrap(),
        expected
    );
}

#[test]
fn stock_front_two_islands_continues_local_clearing_after_bulk() {
    use camlib::pocket::{HelicalEntry, PocketEntry};
    let mut m = fixture();
    m.stock.height_mut().unwrap().max_mm = [20., 16., 10.];
    m.stock.height_mut().unwrap().cell_size_mm = 0.2;
    m.tools[0].cutter.diameter_mm = 2.;
    m.tools[0].cutter.shank_diameter_mm = 2.;
    let tool = m.tools[0].prepare().unwrap();
    let mut op = operation();
    op.path_strategy = AdaptivePathStrategy::StockFront;
    op.pocket.floor_z_mm = 9.;
    op.pocket.stepover_mm = 0.25;
    op.pocket.radial_allowance_mm = 0.1;
    op.pocket.entry = Some(PocketEntry::StockEdgeOrHelix(HelicalEntry::default()));
    op.max_engagement_deg = 2. * 0.75f64.acos().to_degrees();
    op.orbit_radius_mm = Some(0.5);
    op.min_orbit_radius_mm = Some(0.2);
    op.local_orbit_tolerance_mm = Some(0.05);
    let rectangle = |a, b, c, d| Boundary {
        vertices: vec![[a, b, 0.], [c, b, 0.], [c, d, 0.], [a, d, 0.]],
    };
    op.pocket.region.name = "two islands".into();
    op.pocket.region.source = serde_json::Value::Null;
    op.pocket.region.components = vec![Component {
        outer: rectangle(-4., -4., 24., 20.),
        holes: vec![rectangle(5., 4., 8., 10.), rectangle(12., 4., 15., 10.)],
    }];
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut position = [-2., -2., 12.];
    let mut next_id = 1;
    let mut nominal_seconds = 0.;
    let mut checked_linear_commit = false;
    let mut plan = AdaptivePlan::prepare(
        op,
        &m.tools[0],
        m.stock.height().unwrap(),
        position,
        next_id,
        &|| false,
    )
    .unwrap();
    while let Some(motion) = plan
        .next_motion_with_stock(&stock, &|| false)
        .unwrap_or_else(|e| {
            panic!(
                "mixed contour/local clearing stopped at {next_id}: {e}; {:?}",
                plan.report().front
            )
        })
    {
        assert!(next_id <= 10000, "mixed clearing failed to complete");
        assert_eq!(motion.id, next_id);
        assert_eq!(motion.curve.start(), position);
        nominal_seconds += motion
            .curve
            .prepare(m.sweep.tolerance_mm)
            .unwrap()
            .length_mm()
            / motion.feed_mm_min.unwrap_or(6000.)
            * 60.;
        let front = plan.report().front.as_ref().unwrap();
        if front.linear_committed_moves > 0 && !checked_linear_commit {
            let before = (front.linear_contours, front.linear_committed_moves);
            let mut partial = stock.clone();
            let delta = partial
                .preview_sweep(&tool, &motion, 0.5, m.sweep, &|| false)
                .unwrap();
            partial.apply_delta(&delta).unwrap();
            assert!(plan.next_motion_with_stock(&partial, &|| false).is_err());
            let mut foreign = stock.clone();
            let delta = foreign
                .preview_sweep(&tool, &motion, 1., m.sweep, &|| false)
                .unwrap();
            foreign.apply_delta(&delta).unwrap();
            let foreign = Stock::from_snapshot(foreign.snapshot()).unwrap();
            assert!(plan.next_motion_with_stock(&foreign, &|| false).is_err());
            let front = plan.report().front.as_ref().unwrap();
            assert_eq!(
                (front.linear_contours, front.linear_committed_moves),
                before
            );
            checked_linear_commit = true;
        }
        let delta = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &|| false)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        position = motion.curve.end();
        next_id += 1;
    }
    plan.finish_report(&|| false).unwrap();
    let front = plan.report().front.as_ref().unwrap();
    assert!(front.coverage_complete);
    assert!(checked_linear_commit);
    assert!(next_id - 1 <= 1200, "multi-island motion quality");
    assert!(front.reentries <= 80, "multi-island reentry quality");
    assert!(
        nominal_seconds <= 350.,
        "multi-island nominal cycle quality"
    );
    assert!(
        front.unranked_contour_moves > 0,
        "must exercise fixed contours"
    );
    assert!(
        front.productive_candidates + front.navigation_moves + front.linear_committed_moves > 0,
        "must exercise continuation after fixed contours"
    );
    assert_eq!(
        front.front_moves,
        front.unranked_contour_moves + front.productive_candidates + front.navigation_moves
    );
    assert!(front.max_bounded_engagement_deg <= 2. * 0.75f64.acos().to_degrees());
    let (mut cleared, mut protected) = (0, 0);
    for (i, cell) in stock.cells().iter().enumerate() {
        let [x, y] = stock.cell_center(i).unwrap();
        let inside = [(5., 8.), (12., 15.)]
            .iter()
            .any(|&(a, b)| x > a + 0.2 && x < b - 0.2 && y > 4.2 && y < 9.8);
        let distance = |a: f64, b: f64| {
            (a - x)
                .max(0.)
                .max(x - b)
                .hypot((4. - y).max(0.).max(y - 10.))
        };
        if inside {
            assert!(
                cell.lower_mm >= 10. - 4. * m.sweep.tolerance_mm,
                "protected island cell {x},{y}: {cell:?}"
            );
            protected += 1;
        } else if distance(5., 8.) > 1.5 && distance(12., 15.) > 1.5 {
            assert!(
                cell.upper_mm <= 9. + 4. * m.sweep.tolerance_mm,
                "uncut reachable cell {x},{y}: {cell:?}"
            );
            cleared += 1;
        }
    }
    assert!(cleared > 100 && protected > 100);
    eprintln!(
        "mixed clearing: {}motions/{cleared}cleared/{protected}protected cells; {front:?}",
        next_id - 1
    );
}

#[test]
fn stock_front_repeated_convex_pocket_uses_native_coverage_but_rechecks_snapshot_uncertainty() {
    use camlib::pocket::{HelicalEntry, PocketEntry};

    let mut m = fixture();
    m.tools[0].cutter.diameter_mm = 2.;
    m.tools[0].cutter.shank_diameter_mm = 2.;
    m.stock.height_mut().unwrap().min_mm = [12., 10., 0.];
    m.stock.height_mut().unwrap().max_mm = [20., 18., 10.];
    m.stock.height_mut().unwrap().cell_size_mm = 0.2;
    let tool = m.tools[0].prepare().unwrap();
    let mut op = operation();
    op.path_strategy = AdaptivePathStrategy::StockFront;
    op.pocket.floor_z_mm = 9.;
    op.pocket.stepover_mm = 0.25;
    op.pocket.radial_allowance_mm = 0.1;
    op.pocket.entry = Some(PocketEntry::StockEdgeOrHelix(HelicalEntry::default()));
    op.max_engagement_deg = 82.81924421854173;
    op.orbit_radius_mm = Some(0.5);
    op.min_orbit_radius_mm = Some(0.2);
    op.local_orbit_tolerance_mm = Some(0.05);
    op.pocket.region.name = "convex pocket".into();
    op.pocket.region.source = serde_json::Value::Null;
    op.pocket.region.components = vec![Component {
        outer: Boundary {
            vertices: vec![
                [14., 12., 0.],
                [18., 12., 0.],
                [18., 16., 0.],
                [14., 16., 0.],
            ],
        },
        holes: vec![],
    }];
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let original_volume = stock.volume().remaining_upper_mm3;
    let mut position = [12., 10., 12.];
    let mut next_id = 1;
    for run in 0..3 {
        if run == 2 {
            // A height snapshot drops exact sweep authority. Its boundary cells
            // still straddle the floor, so it must not inherit the native skip.
            stock = Stock::from_snapshot(stock.snapshot()).unwrap();
        }
        let before = stock.volume().remaining_upper_mm3;
        let mut plan = AdaptivePlan::prepare(
            op.clone(),
            &m.tools[0],
            m.stock.height().unwrap(),
            position,
            next_id,
            &|| false,
        )
        .unwrap();
        while let Some(motion) = plan.next_motion_with_stock(&stock, &|| false).unwrap() {
            assert_eq!(motion.id, next_id);
            assert_eq!(motion.curve.start(), position);
            assert!(plan.report().motions < if run == 1 { 16 } else { 2000 });
            let delta = stock
                .preview_sweep(&tool, &motion, 1., m.sweep, &|| false)
                .unwrap();
            stock.apply_delta(&delta).unwrap();
            position = motion.curve.end();
            next_id = motion.id + 1;
        }
        plan.finish_report(&|| false).unwrap();
        let report = plan.report();
        let front = report.front.as_ref().unwrap();
        assert!(front.coverage_complete);
        assert_eq!(front.completed_components, 1);
        assert_eq!(
            front.front_moves,
            front.productive_candidates + front.navigation_moves + front.unranked_contour_moves
        );
        if run == 1 {
            assert_eq!(front.enclosed_contours, 0);
            assert_eq!(front.front_moves, 0);
            assert_eq!(front.unranked_contour_moves, 0);
            assert_eq!(front.reentries, 0);
        } else {
            assert!(front.enclosed_contours > 0);
            assert_eq!(front.productive_candidates, 0);
            assert_eq!(front.navigation_moves, 0);
            assert_eq!(front.unranked_contour_moves, front.front_moves);
        }
        if run == 0 {
            assert!(stock.volume().remaining_upper_mm3 < original_volume);
        } else {
            assert_eq!(stock.volume().remaining_upper_mm3, before);
        }
        eprintln!(
            "convex pocket run {run}: {} motions, {} enclosed contours",
            report.motions, front.enclosed_contours
        );
    }
}

#[test]
fn stock_front_does_not_invent_productive_material_in_an_upstream_snapshot() {
    let m = fixture();
    let tool = m.tools[0].prepare().unwrap();
    let mut op = operation();
    op.path_strategy = AdaptivePathStrategy::StockFront;
    op.pocket.floor_z_mm = 9.;
    let mut snapshot = Stock::new(m.stock.height().unwrap().clone())
        .unwrap()
        .snapshot();
    snapshot.generation = 37;
    for h in &mut snapshot.cells {
        h.lower_mm = 9.;
        h.upper_mm = 9.;
    }
    let mut stock = Stock::from_snapshot(snapshot).unwrap();
    let before = stock.volume().remaining_upper_mm3;
    let mut plan = AdaptivePlan::prepare(
        op,
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        100,
        &|| false,
    )
    .unwrap();
    while let Some(motion) = plan.next_motion_with_stock(&stock, &|| false).unwrap() {
        assert!(
            plan.report().motions < 32,
            "empty stock did not complete promptly"
        );
        let delta = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &|| false)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
    }
    plan.finish_report(&|| false).unwrap();
    let report = plan.report().front.as_ref().unwrap();
    assert_eq!(report.productive_candidates, 0);
    assert_eq!(report.navigation_moves, 0);
    assert_eq!(report.front_moves, 0);
    assert_eq!(report.reentries, 0);
    assert!(report.coverage_complete);
    assert_eq!(stock.volume().remaining_upper_mm3, before);
}

/// #190: the adaptive pocket planned from the stock (`engagement_front`), on
/// the L pocket. The orbit planner covers the region with overlapping orbits
/// whatever the stock holds; this one follows the boundary between cut and
/// uncut stock, so nothing is cut twice.
#[test]
fn engagement_front_pocket_clears_the_region_once_within_its_engagement() {
    use camlib::operation::OperationPlan;
    let cancel = AtomicBool::new(false);
    // The L: (3,3) (23,3) (23,11) (11,11) (11,23) (3,23), floor Z8, top Z10.
    let corners = [
        [3., 3.],
        [23., 3.],
        [23., 11.],
        [11., 11.],
        [11., 23.],
        [3., 23.],
    ];
    let inside = |p: [f64; 2]| {
        (p[0] > 3. && p[0] < 23. && p[1] > 3. && p[1] < 11.)
            || (p[0] > 3. && p[0] < 11. && p[1] > 3. && p[1] < 23.)
    };
    let to_wall = |p: [f64; 2]| {
        (0..6)
            .map(|k| {
                let (a, b) = (corners[k], corners[(k + 1) % 6]);
                let e = [b[0] - a[0], b[1] - a[1]];
                let t = (((p[0] - a[0]) * e[0] + (p[1] - a[1]) * e[1])
                    / (e[0] * e[0] + e[1] * e[1]))
                    .clamp(0., 1.);
                dist([a[0] + t * e[0], a[1] + t * e[1]], p)
            })
            .fold(f64::INFINITY, f64::min)
    };
    let length = |c: &Curve| c.prepare(1e-5).unwrap().length_mm();
    let run = |strategy: AdaptivePathStrategy| {
        let m = fixture();
        let mut op = operation();
        op.path_strategy = strategy;
        let tool = m.tools[0].prepare().unwrap();
        let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let mut plan = OperationPlan::prepare_with_stock(
            Operation::AdaptivePocket(op),
            &m.tools[0],
            &stock,
            m.initial_position_mm,
            1,
            &cancel,
        )
        .unwrap();
        let mut motions = vec![];
        while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
            assert!(motions.len() < 100_000);
            let delta = stock
                .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
                .unwrap();
            stock.apply_delta(&delta).unwrap();
            motions.push(motion);
        }
        plan.finish_report(&cancel).unwrap();
        (motions, stock, serde_json::to_value(plan.report()).unwrap())
    };
    let (orbits, _, _) = run(AdaptivePathStrategy::DiskFront);
    let (motions, stock, report) = run(AdaptivePathStrategy::EngagementFront);

    // Independent of the planner: at depth, the cutter (radius 2) stays
    // inside the L and off its walls, over every curve.
    for motion in &motions {
        let z = motion.curve.start()[2].min(motion.curve.end()[2]);
        if z >= 10. {
            continue;
        }
        let path = motion.curve.prepare(1e-5).unwrap();
        for i in 0..=64 {
            let p = path.position(i as f64 / 64.).unwrap();
            let q = [p[0], p[1]];
            assert!(
                inside(q) && to_wall(q) >= 2. - 1e-6,
                "cutter in the wall at {p:?}"
            );
        }
    }
    // The stock: everything the cutter can reach is at the floor, and
    // nothing outside the pocket is touched. (A 0.4 mm cell is 0.29 mm
    // across its half diagonal; a 2 mm cutter leaves the inside corners.)
    for (i, cell) in stock.cells().iter().enumerate() {
        let p = stock.cell_center(i).unwrap();
        let corner = corners.iter().any(|c| dist(*c, p) < 2.5);
        // With the wall pass, right up to the walls: a cell whose centre is
        // more than its half diagonal inside is wholly inside.
        if inside(p) && to_wall(p) > 0.3 && !corner {
            assert!(cell.upper_mm <= 8. + 1e-4, "uncut at {p:?}: {cell:?}");
        } else if !inside(p) && to_wall(p) > 0.6 {
            assert_eq!(cell.lower_mm, 10., "cut outside the pocket at {p:?}");
        }
    }
    // Two depth levels, each entered by one helix, every cutting stroke
    // proven within acos(1 - stepover / r) = acos(1 - 1.5 / 2).
    let limit = (1f64 - 1.5 / 2.).acos().to_degrees();
    assert_eq!(report["strategy"], "adaptive_pocket");
    assert_eq!(report["clearing"], "engagement_front");
    assert_eq!(report["completed_layers"], 2);
    assert!((report["max_engagement_deg"].as_f64().unwrap() - limit).abs() < 1e-9);
    for layer in report["layers"].as_array().unwrap() {
        let c = &layer["clearing"];
        assert_eq!(c["helical_entries"], 1);
        // One contour-style pass along the L's wall, none skipped.
        assert_eq!(c["wall_passes"], 1);
        assert!(c.get("wall_passes_skipped").is_none());
        // Only the stretch into each inside corner, where the contact is
        // not proven within the limit, runs as entry motion: a small part
        // of the 65 mm wall loop, not whole walls.
        let confined = c
            .get("confined_stroke_mm")
            .map_or(0., |v| v.as_f64().unwrap());
        assert!(confined < 8., "{confined} mm of entry-feed strokes: {c}");
        assert!(c["proven_max_engagement_deg"].as_f64().unwrap() <= limit);
        assert!(c["engagement_proved_strokes"].as_u64().unwrap() > 10);
    }
    // Far less motion than the orbits for the same pocket.
    let cut = |list: &[Motion]| -> f64 {
        list.iter()
            .filter(|m| !m.kind.is_rapid())
            .map(|m| length(&m.curve))
            .sum()
    };
    assert!(
        motions.len() * 3 < orbits.len() && cut(&motions) * 3. < cut(&orbits),
        "{} moves, {:.0} mm against {} moves, {:.0} mm of orbits",
        motions.len(),
        cut(&motions),
        orbits.len(),
        cut(&orbits)
    );
    // The radial width is the stepover: wider than the cutter radius is refused.
    let m = fixture();
    let mut op = operation();
    op.path_strategy = AdaptivePathStrategy::EngagementFront;
    op.pocket.stepover_mm = 2.5;
    let stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let error = OperationPlan::prepare_with_stock(
        Operation::AdaptivePocket(op),
        &m.tools[0],
        &stock,
        m.initial_position_mm,
        1,
        &cancel,
    )
    .err()
    .unwrap();
    assert!(
        error.contains("stepover_mm") && error.contains("cutter radius"),
        "{error}"
    );
}
