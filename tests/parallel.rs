use camlib::{
    facing::TraverseAxis,
    finishing::{FinishPattern, ParallelOperation, ParallelPlan},
    motion::{Curve, Motion, MoveKind},
    pocket::Operation,
    stock::{CancelSignal, Stock},
    stream::StreamManifest,
    surface::{SurfaceMesh, SurfaceWork, segment_triangle_sq},
};
use std::{cell::Cell, sync::atomic::AtomicBool};
fn fixture() -> StreamManifest {
    let m: StreamManifest =
        serde_json::from_str(include_str!("../fixtures/operations/parallel.json")).unwrap();
    m.validate().unwrap();
    m
}
fn operation(m: &StreamManifest) -> ParallelOperation {
    match &m.operations[0] {
        Operation::Parallel(p) => p.clone(),
        _ => panic!("fixture"),
    }
}
fn mesh(t: Vec<[[f64; 3]; 3]>) -> SurfaceMesh {
    SurfaceMesh {
        version: 1,
        units: "mm".into(),
        triangles: t.into(),
        surface_error_mm: 0.,
        source: serde_json::Value::Null,
    }
}

#[test]
fn floor_clipping_preserves_analytic_ramp_contact_and_rejects_unresolved_facets() {
    let cancel = AtomicBool::new(false);
    // Exact z=x/2 ramp over a 10x10 square. The selected bottom truncates
    // the ramp at z=2; base contact is the analytic max(2,x/2) target.
    let source = mesh(vec![
        [[0., 0., 0.], [10., 0., 5.], [10., 10., 5.]],
        [[0., 0., 0.], [10., 10., 5.], [0., 10., 0.]],
    ]);
    let clipped = source.above_floor(2., 1e-5, &cancel).unwrap();
    assert!(clipped.triangles.iter().flatten().all(|p| p[2] >= 2.));
    assert_eq!(source.triangles[0][0][2], 0.);
    let prepared = clipped.prepare(2., 1e-5, &cancel).unwrap();
    let mut work = SurfaceWork::default();
    for x in [0., 1., 2., 3., 4., 5., 6., 7.] {
        let actual = prepared
            .contact([x, 5.], 1., &mut work, &cancel)
            .unwrap()
            .center_z;
        // On this interval an unconstrained contact lies inside the ramp;
        // below the clipping crossing the independent base plane dominates.
        let expected = 3f64.max(x / 2. + 1.25f64.sqrt());
        assert!(
            (actual - expected).abs() < 1e-10,
            "x={x}: {actual} vs {expected}"
        );
    }
    assert!(
        source
            .above_floor(2., 1e-5, &AtomicBool::new(true))
            .is_err()
    );
    assert!(source.above_floor(6., 1e-5, &cancel).is_err());
    let narrow = mesh(vec![[[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]]]);
    assert!(narrow.above_floor(1. - 1e-8, 1e-5, &cancel).is_err());
}
// Independent exact lower ball envelope along an arbitrary 3D line. This tests
// actual removal at a point; stock lower bounds alone can straddle the target.
fn swept_height(curve: &Curve, xy: [f64; 2]) -> f64 {
    let a = curve.start();
    let b = curve.end();
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let dz = b[2] - a[2];
    let l2 = dx * dx + dy * dy;
    let t = if l2 > 0. {
        let projection = ((xy[0] - a[0]) * dx + (xy[1] - a[1]) * dy) / l2;
        let perpendicular = ((xy[0] - a[0]) * dy - (xy[1] - a[1]) * dx).powi(2) / l2;
        if perpendicular > 4. {
            return f64::INFINITY;
        }
        (projection - dz * ((4. - perpendicular) / (l2 * (l2 + dz * dz))).sqrt()).clamp(0., 1.)
    } else if dz < 0. {
        1.
    } else {
        0.
    };
    let distance = (xy[0] - a[0] - t * dx).powi(2) + (xy[1] - a[1] - t * dy).powi(2);
    if distance > 4. {
        f64::INFINITY
    } else {
        a[2] + t * dz + 2. - (4. - distance).sqrt()
    }
}
fn roof_center(x: f64) -> f64 {
    let d = x - 12.;
    let switch = 2. / 10f64.sqrt();
    if d.abs() <= switch {
        8. + (4. - d * d).sqrt()
    } else {
        8. - d.abs() / 3. + 2. * (10f64 / 9.).sqrt()
    }
}
fn assert_whole_roof_segment(curve: &Curve) {
    let a = curve.start();
    let b = curve.end();
    // The infinite-width analytic roof formula applies within the stock domain.
    // Exterior rounding is tested by the independent finite line sweep below.
    if a[1] < 0. || a[1] > 16. || b[1] < 0. || b[1] > 16. {
        return;
    }
    let mut positions = vec![a[0], b[0], 0., 24.];
    if a[0] != b[0] {
        let slope = (b[2] - a[2]) / (b[0] - a[0]);
        positions.push(12. - 2. * slope / (1. + slope * slope).sqrt());
        positions.extend([12. - 2. / 10f64.sqrt(), 12. + 2. / 10f64.sqrt()]);
    }
    for x in positions {
        if (0. ..=24.).contains(&x) && x >= a[0].min(b[0]) && x <= a[0].max(b[0]) {
            let z = if a[0] == b[0] {
                a[2].min(b[2])
            } else {
                a[2] + (b[2] - a[2]) * (x - a[0]) / (b[0] - a[0])
            };
            assert!(
                z + 2. >= roof_center(x) - 1e-9,
                "whole-segment roof gouge at{x}: {curve:?}"
            );
        }
    }
}
#[test]
fn analytic_face_edge_vertex_contact_and_excess_bounds() {
    let cancel = AtomicBool::new(false);
    let mut work = SurfaceWork::default();
    let face = mesh(vec![[[0., 0., 1.], [12., 0., 5.], [0., 12., 1.]]])
        .prepare(0., 1e-5, &cancel)
        .unwrap();
    let contact = face.contact([3., 3.], 2., &mut work, &cancel).unwrap();
    assert!((contact.center_z - (2. + 2. * (10f64 / 9.).sqrt())).abs() < 1e-12);
    let a = [2., 3., 1. + 2. / 3. + 2. * (10f64 / 9.).sqrt()];
    let b = [4., 3., 1. + 4. / 3. + 2. * (10f64 / 9.).sqrt()];
    assert!(face.excess_bound(a, b, contact, 2.).unwrap() < 1e-12);
    let flat = mesh(vec![[[0., 0., 4.], [12., 0., 4.], [0., 12., 4.]]])
        .prepare(0., 1e-5, &cancel)
        .unwrap();
    assert!(
        (flat
            .contact([3., -1.], 2., &mut work, &cancel)
            .unwrap()
            .center_z
            - (4. + 3f64.sqrt()))
        .abs()
            < 1e-12
    );
    assert!(
        (flat
            .contact([-1., -1.], 2., &mut work, &cancel)
            .unwrap()
            .center_z
            - (4. + 2f64.sqrt()))
        .abs()
            < 1e-12
    );
    assert_eq!(
        flat.contact([30., 30.], 2., &mut work, &cancel)
            .unwrap()
            .center_z,
        2.
    );
    let seam = mesh(vec![
        [[0., 0., 4.], [4., 0., 4.], [4., 4., 4.]],
        [[0., 0., 4.], [4., 4., 4.], [0., 4., 4.]],
    ])
    .prepare(0., 1e-5, &cancel)
    .unwrap();
    let c = seam.contact([2., 2.], 2., &mut work, &cancel).unwrap();
    assert!(
        seam.excess_bound([1.99, 2., 6.], [2.01, 2., 6.], c, 2.)
            .unwrap()
            < 0.001
    );
}
#[test]
fn cap_to_base_switch_bounds_the_interior_error_when_the_sphere_ends() {
    let cancel = AtomicBool::new(false);
    let radius = 2.005;
    let floor = 3.;
    let base = floor + radius;
    let top = 5.;
    let prepared = mesh(vec![
        [[-10., -10., top], [4., -10., top], [4., 10., top]],
        [[-10., -10., top], [4., 10., top], [-10., 10., top]],
    ])
    .prepare(floor, 1e-5, &cancel)
    .unwrap();
    let a = [
        4. + radius - 0.0003,
        0.,
        top + (radius * radius - (radius - 0.0003).powi(2)).sqrt(),
    ];
    let b = [4. + radius + 0.0002, 0., base];
    let center_x = (a[0] + b[0]) / 2.;
    let mut work = SurfaceWork::default();
    let contact = prepared
        .contact([center_x, 0.], radius, &mut work, &cancel)
        .unwrap();
    let bound = prepared.excess_bound(a, b, contact, radius).unwrap();
    // Independent analytic edge-offset and floor intersection. Both endpoint
    // errors are zero, but their connecting line sits above the contact graph
    // at this interior switch. Endpoint-only clamping would report zero.
    let switch_x = 4. + (radius * radius - (base - top).powi(2)).sqrt();
    let fraction = (switch_x - a[0]) / (b[0] - a[0]);
    let maximum = a[2] + fraction * (b[2] - a[2]) - base;
    assert!(maximum > 0.01);
    assert!(
        bound >= maximum - 1e-12 && bound <= maximum + 1e-10,
        "{bound} vs {maximum}"
    );
    // Crossing the same support switch at translated coordinates must retain
    // the certificate rather than depend on a bisection hitting it exactly.
    let shifted = mesh(vec![
        [[990., -10., top], [1004., -10., top], [1004., 10., top]],
        [[990., -10., top], [1004., 10., top], [990., 10., top]],
    ])
    .prepare(floor, 1e-5, &cancel)
    .unwrap();
    let c = shifted
        .contact([center_x + 1000., 0.], radius, &mut work, &cancel)
        .unwrap();
    let shifted_bound = shifted
        .excess_bound(
            [a[0] + 1000., a[1], a[2]],
            [b[0] + 1000., b[1], b[2]],
            c,
            radius,
        )
        .unwrap();
    assert!((shifted_bound - bound).abs() < 1e-8);
}
#[test]
fn continuous_distance_detects_crossings_and_downward_peak_solids() {
    let cancel = AtomicBool::new(false);
    let mut work = SurfaceWork::default();
    let t = [[0., 0., 8.], [1., 0., 8.], [0., 1., 8.]];
    assert_eq!(segment_triangle_sq([0.1, 0.1, 10.], [0.1, 0.1, 6.], t), 0.);
    assert!((segment_triangle_sq([0.1, 0.1, 10.], [0.2, 0.1, 10.], t) - 4.).abs() < 1e-12);
    let prepared = mesh(vec![t]).prepare(0., 1e-5, &cancel).unwrap();
    // Both ends and the entire top-skin distance look clear, but this crosses
    // the retained solid under the peak. Endpoints/midpoint are not admission.
    let a = [-3., 0.2, 2.];
    let b = [4., 0.2, 2.];
    assert!(segment_triangle_sq(a, b, t) > 4.);
    assert!(
        !prepared
            .segment_clear(a, b, 1., &mut work, &cancel)
            .unwrap()
    );
    assert!(
        prepared
            .segment_clear([-3., 0.2, 10.], [4., 0.2, 10.], 1., &mut work, &cancel)
            .unwrap()
    );
    let near_parallel = [[0., 0.001, 0.], [1e6, 0., 0.], [1e6, 0., -1.]];
    assert!(
        (segment_triangle_sq([0., 0., 1.], [1e6, 0.001, 1.], near_parallel) - 1.).abs() < 1e-10
    );
}
#[test]
fn continuous_segment_clearance_rejects_wholly_interior_and_stationary_queries() {
    let cancel = AtomicBool::new(false);
    for shift in [[0., 0., 0.], [100000., -200000., 1234.]] {
        let point = |x, y, z| [x + shift[0], y + shift[1], z + shift[2]];
        let surface = mesh(vec![[
            point(0., 0., 8.),
            point(20., 0., 8.),
            point(0., 20., 8.),
        ]])
        .prepare(shift[2], 1e-5, &cancel)
        .unwrap();
        let (a, b) = (point(3., 3., 4.), point(5., 3., 4.));
        // The complete line lies in the convex solid x>=0,y>=0,x+y<=20,
        // 0<=z<=8. It is >1mm from EVERY skin/base face. Distance only to
        // those boundary faces cannot establish distance to the filled solid.
        for (start, end) in [(a, b), (b, a), (a, a)] {
            assert!(
                !surface
                    .segment_clear(start, end, 1., &mut SurfaceWork::default(), &cancel)
                    .unwrap(),
                "wholly interior segment must not be reported clear: {start:?}->{end:?}"
            );
        }
    }
}

#[test]
fn continuous_segment_clearance_validates_queries_and_final_work_budget() {
    use std::cell::Cell;
    let surface = mesh(vec![[[0., 0., 8.], [20., 0., 8.], [0., 20., 8.]]])
        .prepare(0., 1e-5, &|| false)
        .unwrap();
    // Every expanded AABB axis overlaps, but Euclidean box separation can
    // exclude the complete line. Test the budget and cancellation contract
    // independently of which conservative acceleration rejects the source.
    let (a, b) = ([21.5, 21.5, 4.], [22., 21.5, 4.]);
    let mut measured_work = SurfaceWork::default();
    assert!(
        surface
            .segment_clear(a, b, 2., &mut measured_work, &|| false)
            .unwrap()
    );
    let required_work =
        measured_work.nodes + measured_work.contact_tests + measured_work.distance_tests;
    let mut almost_exhausted = SurfaceWork {
        distance_tests: 20_000_000 - (required_work - 1),
        ..SurfaceWork::default()
    };
    // Work is counted, never capped: the same query still completes.
    assert!(
        surface
            .segment_clear(a, b, 2., &mut almost_exhausted, &|| false)
            .unwrap()
    );
    assert!(
        surface
            .segment_clear(a, b, 2., &mut SurfaceWork::default(), &|| true)
            .is_err()
    );
    let polls = Cell::new(0);
    surface
        .segment_clear(a, b, 2., &mut SurfaceWork::default(), &|| {
            polls.set(polls.get() + 1);
            false
        })
        .unwrap();
    let total_polls = polls.get();
    assert!(total_polls >= 2, "entry and final cancellation checks");
    for cancelled_at in 1..=total_polls {
        let polls = Cell::new(0);
        let cancel = || {
            polls.set(polls.get() + 1);
            polls.get() >= cancelled_at
        };
        assert!(
            surface
                .segment_clear(a, b, 2., &mut SurfaceWork::default(), &cancel)
                .is_err(),
            "cancellation at poll {cancelled_at}, including final poll, must be observed"
        );
        assert_eq!(polls.get(), cancelled_at);
    }
    for radius in [0., -1., f64::INFINITY, f64::NAN] {
        assert!(
            surface
                .segment_clear(a, b, radius, &mut SurfaceWork::default(), &|| false)
                .is_err()
        );
    }
    for invalid in [
        [f64::NAN, 0., 4.],
        [0., f64::INFINITY, 4.],
        [0., 0., f64::NEG_INFINITY],
        [1e6 + 1., 0., 4.],
    ] {
        assert!(
            surface
                .segment_clear(a, invalid, 2., &mut SurfaceWork::default(), &|| false)
                .is_err()
        );
        assert!(
            surface
                .segment_clear(invalid, b, 2., &mut SurfaceWork::default(), &|| false)
                .is_err()
        );
    }
}

#[test]
fn roof_parallel_axis_direction_and_all_stock_cells_preserve_and_finish_target() {
    let m = fixture();
    let cancel = AtomicBool::new(false);
    for (axis, reverse) in [
        (TraverseAxis::X, false),
        (TraverseAxis::X, true),
        (TraverseAxis::Y, false),
    ] {
        let mut op = operation(&m);
        op.axis = axis;
        op.reverse = reverse;
        let mut p = ParallelPlan::prepare(
            op,
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &cancel,
        )
        .unwrap();
        let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let tool = m.tools[0].prepare().unwrap();
        let mut pos = m.initial_position_mm;
        let mut zspan = 0f64;
        let mut cuts = 0;
        let mut curves = vec![];
        while let Some(motion) = p.next_motion_with_stock(&stock, &cancel).unwrap() {
            assert_eq!(motion.curve.start(), pos);
            pos = motion.curve.end();
            if motion.kind == MoveKind::Rapid {
                assert!(motion.curve.start()[2] >= 12. && pos[2] >= 12.);
            }
            if motion.kind == MoveKind::Cut {
                assert_whole_roof_segment(&motion.curve);
                zspan = zspan.max((motion.curve.start()[2] - pos[2]).abs());
                cuts += 1;
            }
            curves.push(motion.curve.clone());
            let d = stock
                .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
                .unwrap();
            stock.apply_delta(&d).unwrap();
        }
        p.finish_report(&cancel).unwrap();
        assert_eq!(p.report().rows_completed, p.report().rows);
        assert!(p.report().max_certified_path_excess_mm.unwrap() <= 0.005);
        assert!(p.report().work.contact_tests > 0 && p.report().work.distance_tests > 0);
        if axis == TraverseAxis::X {
            assert!(zspan > 0.05 && cuts > p.report().rows);
        }
        for (i, h) in stock.cells().iter().enumerate() {
            let q = stock.cell_center(i).unwrap();
            let dx = stock.cell_widths_mm()[0] / 2.;
            let roof = |x: f64| 4. + x.min(24. - x) / 3.;
            let low = roof(q[0] - dx).min(roof(q[0] + dx));
            let high = if (q[0] - 12.).abs() <= dx {
                8.
            } else {
                roof(q[0] - dx).max(roof(q[0] + dx))
            };
            let actual = curves
                .iter()
                .map(|c| swept_height(c, q))
                .fold(10f64, f64::min);
            assert!(
                h.lower_mm <= actual + 1e-7 && h.upper_mm >= actual - 1e-7,
                "stock interval misses true sweep {q:?}: {h:?}, actual{actual}"
            );
            assert!(
                actual >= roof(q[0]) - 1e-7 && actual <= roof(q[0]) + 0.03,
                "target/cusp error at{q:?}:actual{actual}"
            );
            assert!(h.upper_mm >= low, "stock interval proves gouge {q:?}");
            assert!(
                h.upper_mm <= high + 0.15,
                "unfinished axis{axis:?} {q:?}: {h:?},target{high}"
            );
        }
        assert!(stock.volume().removed_lower_mm3 > 1400.);
    }
}
struct CancelAfter(Cell<usize>);
impl CancelSignal for CancelAfter {
    fn is_cancelled(&self) -> bool {
        let n = self.0.get();
        self.0.set(n.saturating_sub(1));
        n == 0
    }
}
#[test]
fn cancellation_and_stale_stock_preserve_the_surface_cursor() {
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let tool = m.tools[0].prepare().unwrap();
    let mut p = ParallelPlan::prepare(
        operation(&m),
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    let mut reference = ParallelPlan::prepare(
        operation(&m),
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    for index in 0..8 {
        if index == 3 {
            assert!(
                p.next_motion_with_stock(&stock, &CancelAfter(Cell::new(2)))
                    .is_err()
            );
        } else {
            assert!(
                p.next_motion_with_stock(&stock, &AtomicBool::new(true))
                    .is_err()
            );
        }
        let actual = p.next_motion_with_stock(&stock, &cancel).unwrap().unwrap();
        assert_eq!(
            actual,
            reference
                .next_motion_with_stock(&stock, &cancel)
                .unwrap()
                .unwrap()
        );
        assert!(
            p.next_motion_with_stock(&stock, &cancel)
                .unwrap_err()
                .contains("commit")
        );
        let delta = stock
            .preview_sweep(&tool, &actual, 1., m.sweep, &cancel)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        assert_eq!(actual.id, index + 1);
    }
    assert!(p.finish_report(&cancel).unwrap_err().contains("drain"));
}
#[test]
fn bad_surface_tool_reach_and_bounded_parameters_fail_before_motion() {
    let m = fixture();
    let cancel = AtomicBool::new(false);
    for case in 0..7 {
        let mut op = operation(&m);
        let mut tool = m.tools[0].clone();
        match case {
            0 => std::sync::Arc::make_mut(&mut op.surface.triangles)[0][0][0] = f64::NAN,
            1 => op.surface.surface_error_mm = -1.,
            2 => op.stepover_mm = 0.,
            3 => op.safety_margin_mm = 0.,
            4 => {
                tool.cutter.shape = camlib::tool::CutterShape::Drill {
                    point_angle_rad: 2.,
                }
            }
            5 => op.clearance_z_mm = 10.,
            _ => {
                std::sync::Arc::make_mut(&mut op.surface.triangles)[0][1] =
                    op.surface.triangles[0][0]
            }
        }
        assert!(
            ParallelPlan::prepare(
                op,
                &tool,
                m.stock.height().unwrap(),
                m.initial_position_mm,
                1,
                &cancel
            )
            .is_err(),
            "case{case}"
        );
    }
}
#[test]
fn parallel_rows_link_at_retract_height_never_rapid_below_feed_height() {
    let m = fixture();
    let top = m.stock.height().unwrap().max_mm[2];
    let cancel = AtomicBool::new(false);
    let mut op = operation(&m);
    let clearance = op.clearance_z_mm;
    let (retract, feed) = (top + (clearance - top) / 2., top + (clearance - top) / 4.);
    op.retract_z_mm = Some(retract);
    op.feed_z_mm = Some(feed);
    let mut p = ParallelPlan::prepare(
        op.clone(),
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let tool = m.tools[0].prepare().unwrap();
    let mut motions = vec![];
    while let Some(motion) = p.next_motion_with_stock(&stock, &cancel).unwrap() {
        let d = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
            .unwrap();
        stock.apply_delta(&d).unwrap();
        motions.push(motion);
    }
    let mut at_retract = 0;
    for motion in &motions {
        let (a, b) = (motion.curve.start(), motion.curve.end());
        if motion.kind == MoveKind::Rapid {
            assert!(a[2] >= feed && b[2] >= feed, "{motion:?}");
            at_retract += usize::from(a[2] == retract && b[2] == retract);
        }
        if motion.kind == MoveKind::Retract {
            assert!(b[2] >= feed && b[2] >= a[2], "{motion:?}");
        }
    }
    assert!(at_retract > 0, "rows link at the retract height");
    assert_eq!(motions.last().unwrap().curve.end()[2], clearance);
    let mut bad = op;
    bad.feed_z_mm = Some(top);
    assert!(
        ParallelPlan::prepare(
            bad,
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &cancel
        )
        .err()
        .unwrap()
        .contains("stock top")
    );
}

/// Run a plan on fresh stock; returns the motions, whether each changed any
/// stock cell, and the stock left.
fn run_rows(m: &StreamManifest, op: ParallelOperation) -> (Vec<(Motion, bool)>, Stock, usize) {
    let cancel = AtomicBool::new(false);
    let spec = m.stock.height().unwrap();
    let mut p =
        ParallelPlan::prepare(op, &m.tools[0], spec, m.initial_position_mm, 1, &cancel).unwrap();
    let mut stock = Stock::new(spec.clone()).unwrap();
    let tool = m.tools[0].prepare().unwrap();
    let mut pos = m.initial_position_mm;
    let mut motions = vec![];
    while let Some(motion) = p.next_motion_with_stock(&stock, &cancel).unwrap() {
        assert_eq!(motion.curve.start(), pos, "continuous motion");
        pos = motion.curve.end();
        let d = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
            .unwrap();
        let changed = !d.changes().is_empty();
        stock.apply_delta(&d).unwrap();
        motions.push((motion, changed));
    }
    p.finish_report(&cancel).unwrap();
    assert_eq!(p.report().rows_completed, p.report().rows);
    let rows = p.report().rows;
    (motions, stock, rows)
}
/// Per row, in cutting order: its cross-axis coordinate and the sign of
/// every cut's travel along the axis (read from the motions alone).
fn row_directions(motions: &[(Motion, bool)], axis: usize) -> Vec<(f64, f64)> {
    let mut rows: Vec<(f64, f64)> = vec![];
    for (motion, _) in motions {
        if motion.kind != MoveKind::Cut {
            continue;
        }
        let (a, b) = (motion.curve.start(), motion.curve.end());
        assert_eq!(a[1 - axis], b[1 - axis], "a raster cut stays on its row");
        let sign = (b[axis] - a[axis]).signum();
        assert!(sign != 0., "{motion:?}");
        match rows.last() {
            Some(&(c, s)) if c == a[1 - axis] => assert_eq!(s, sign, "a row turns back"),
            _ => rows.push((a[1 - axis], sign)),
        }
    }
    rows
}
// Issue #175: one-way rows. Direction, links and the surface left are read
// from the emitted motions and the swept stock, not from the planner.
#[test]
fn one_way_rows_cut_one_direction_link_by_rapid_and_leave_the_zigzag_surface() {
    let m = fixture();
    let top = m.stock.height().unwrap().max_mm[2];
    // The committed fixture has no one_way: the default is zigzag, and a
    // default operation serialises without the key (cache keys unchanged).
    let base = operation(&m);
    assert!(!base.one_way);
    assert!(
        serde_json::to_value(&base)
            .unwrap()
            .get("one_way")
            .is_none()
    );
    for (axis, reverse) in [
        (TraverseAxis::X, false),
        (TraverseAxis::X, true),
        (TraverseAxis::Y, false),
        (TraverseAxis::Y, true),
    ] {
        let k = if axis == TraverseAxis::X { 0 } else { 1 };
        let forward = if reverse { -1. } else { 1. };
        let mut op = operation(&m);
        op.axis = axis;
        op.reverse = reverse;
        let (retract, feed) = (top + 1., top + 0.5);
        op.retract_z_mm = Some(retract);
        op.feed_z_mm = Some(feed);
        // Either way a row lies between the exact no-gouge height and
        // safety_margin_mm + path_error_mm above it: the operation's tolerance.
        let band = op.path_error_mm + op.safety_margin_mm;
        let (zigzag, zig_stock, zig_rows) = run_rows(&m, op.clone());
        op.one_way = true;
        assert_eq!(serde_json::to_value(&op).unwrap()["one_way"], true);
        let (one_way, one_stock, rows) = run_rows(&m, op);
        assert_eq!(rows, zig_rows);

        // Default (unchanged): rows alternate, starting as `reverse` says.
        let zig = row_directions(&zigzag, k);
        assert_eq!(zig.len(), rows);
        for (i, (_, sign)) in zig.iter().enumerate() {
            assert_eq!(*sign, if i % 2 == 0 { forward } else { -forward });
        }
        // One way: the same rows in the same order, every one cut forward.
        let one = row_directions(&one_way, k);
        assert_eq!(
            one.iter().map(|r| r.0).collect::<Vec<_>>(),
            zig.iter().map(|r| r.0).collect::<Vec<_>>()
        );
        assert!(one.iter().all(|r| r.1 == forward), "{axis:?} {reverse}");
        // Nothing but a rapid ever travels against the cutting direction,
        // and those returns are links: at the retract height, above the
        // stock top, removing no stock; one per row change.
        let mut returns = 0;
        let mut cutting = false;
        for (motion, changed) in &one_way {
            let (a, b) = (motion.curve.start(), motion.curve.end());
            let along = (b[k] - a[k]) * forward;
            cutting |= motion.kind == MoveKind::Cut;
            if motion.kind != MoveKind::Rapid {
                assert!(along >= 0., "feed move against the direction: {motion:?}");
            } else {
                assert!(a[2] >= feed && b[2] >= feed, "{motion:?}");
                assert!(!changed, "a rapid removed stock: {motion:?}");
            }
            // (Before the first row the tool arrives from the start position.)
            if along < 0. && cutting {
                assert!(a[2] == retract && b[2] == retract, "{motion:?}");
                assert!(retract > top);
                // The full return: from beyond one stock side to the other.
                assert!(along.abs() > m.stock.height().unwrap().max_mm[k]);
                returns += 1;
            }
        }
        assert_eq!(returns, rows - 1);
        // Zigzag differs: it does cut the other way (the test would not
        // pass on zigzag output).
        assert!(zigzag.iter().any(|(motion, _)| motion.kind == MoveKind::Cut
            && (motion.curve.end()[k] - motion.curve.start()[k]) * forward < 0.));

        // The surface left, from the exact swept-ball envelope of the cuts
        // at every stock cell: equal to zigzag's within that tolerance, and
        // against the part (the X-axis roof ridge): never below it.
        let surface = |motions: &[(Motion, bool)], q: [f64; 2]| {
            motions
                .iter()
                .map(|(motion, _)| swept_height(&motion.curve, q))
                .fold(top, f64::min)
        };
        let roof = |x: f64| 4. + x.min(24. - x) / 3.;
        for (i, h) in one_stock.cells().iter().enumerate() {
            let q = one_stock.cell_center(i).unwrap();
            let (a, z) = (surface(&one_way, q), surface(&zigzag, q));
            assert!(
                (a - z).abs() <= band + 1e-9,
                "one-way surface differs from zigzag at {q:?}: {a} vs {z}"
            );
            assert!(a >= roof(q[0]) - 1e-7, "gouge at {q:?}: {a}");
            assert!(h.lower_mm <= a + 1e-7 && h.upper_mm >= a - 1e-7);
            let other = zig_stock.cells()[i];
            assert!((h.upper_mm - other.upper_mm).abs() <= band + 1e-9);
        }
    }
}
#[test]
fn one_way_keeps_its_direction_on_slope_limited_runs_and_is_refused_off_raster() {
    let m = fixture();
    let cancel = AtomicBool::new(false);
    // Slope-limited rows are cut as separate runs (vertical plunge entry):
    // every run still goes the same way.
    for reverse in [false, true] {
        let mut op = operation(&m);
        op.reverse = reverse;
        op.one_way = true;
        op.slope_from_deg = 10.;
        let (motions, _, _) = run_rows(&m, op);
        let forward = if reverse { -1. } else { 1. };
        let mut cuts = 0;
        for (motion, _) in &motions {
            let along = (motion.curve.end()[0] - motion.curve.start()[0]) * forward;
            if motion.kind != MoveKind::Rapid {
                assert!(along >= 0., "{motion:?}");
                cuts += usize::from(motion.kind == MoveKind::Cut);
            }
        }
        assert!(cuts > 0);
    }
    let mut op = operation(&m);
    op.one_way = true;
    op.pattern = FinishPattern::Spiral {
        center_mm: [12., 8.],
        inner_radius_mm: 0.,
        outer_radius_mm: None,
        outward: false,
        clockwise: false,
    };
    let error = ParallelPlan::prepare(
        op,
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &cancel,
    )
    .err()
    .unwrap();
    assert!(
        error.contains("one_way applies to the raster pattern only"),
        "{error}"
    );
}
