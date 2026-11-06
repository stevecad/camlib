use camlib::surface::*;
use camlib::waterline::{ContourOptions, prepare_contours};
use std::sync::atomic::AtomicBool;

fn options(h: f64, simplify: bool) -> ContourOptions {
    ContourOptions {
        method: camlib::waterline::ContourMethod::QuadTree,
        center_z_mm: h,
        radius_mm: 2.005,
        protected_radius_mm: 2.00016,
        normal_error_mm: 0.01,
        tolerance_mm: 1e-5,
        xy_min_mm: [0., 0.],
        xy_max_mm: [24., 16.],
        simplify,
    }
}
fn contains(ring: &[[f64; 2]], p: [f64; 2]) -> bool {
    let mut inside = false;
    for (a, b) in ring
        .iter()
        .zip(ring.iter().cycle().skip(1))
        .take(ring.len())
    {
        if (a[1] > p[1]) != (b[1] > p[1])
            && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0]
        {
            inside = !inside;
        }
    }
    inside
}
// Exact continuous distance from a horizontal segment to the rectangular prism.
// On each clipping interval the squared distance is a quadratic, so endpoints
// and its stationary point suffice; no motion sampling or production helper.
fn rectangular_distance_range(a: [f64; 2], b: [f64; 2], h: f64) -> (f64, f64) {
    let bounds = [[8., 16.], [6., 10.]];
    let delta = [b[0] - a[0], b[1] - a[1]];
    let mut times = vec![0., 1.];
    for k in 0..2 {
        if delta[k] != 0. {
            for edge in bounds[k] {
                let t = (edge - a[k]) / delta[k];
                if t > 0. && t < 1. {
                    times.push(t);
                }
            }
        }
    }
    times.sort_by(f64::total_cmp);
    let mut minimum = f64::INFINITY;
    let mut maximum = 0f64;
    for pair in times.windows(2) {
        let mid = (pair[0] + pair[1]) / 2.;
        let mut aa = 0.;
        let mut ab = 0.;
        for k in 0..2 {
            let x = a[k] + mid * delta[k];
            let (s, c) = if x < bounds[k][0] {
                (-delta[k], bounds[k][0] - a[k])
            } else if x > bounds[k][1] {
                (delta[k], a[k] - bounds[k][1])
            } else {
                (0., 0.)
            };
            aa += s * s;
            ab += s * c;
        }
        let stationary = if aa > 0. {
            (-ab / aa).clamp(pair[0], pair[1])
        } else {
            pair[0]
        };
        for t in [pair[0], pair[1], stationary] {
            let d2 = (0..2)
                .map(|k| {
                    (bounds[k][0] - a[k] - t * delta[k])
                        .max(a[k] + t * delta[k] - bounds[k][1])
                        .max(0.)
                        .powi(2)
                })
                .sum::<f64>()
                + (h - 8.).max(0.).powi(2);
            minimum = minimum.min(d2);
            maximum = maximum.max(d2);
        }
    }
    (minimum.sqrt(), maximum.sqrt())
}

#[test]
fn conservative_contours_match_rectangular_and_circular_level_oracles() {
    let cancel = AtomicBool::new(false);
    let mesh = SurfaceMesh {
        version: 1,
        units: "mm".into(),
        surface_error_mm: 0.,
        source: serde_json::Value::Null,
        triangles: vec![
            [[8., 6., 8.], [16., 6., 8.], [16., 10., 8.]],
            [[8., 6., 8.], [16., 10., 8.], [8., 10., 8.]],
        ]
        .into(),
    };
    let surface = mesh.prepare(0., 1e-5, &cancel).unwrap();
    for method in [
        camlib::waterline::ContourMethod::QuadTree,
        camlib::waterline::ContourMethod::Support,
    ] {
        for h in [7., 9.] {
            for simplify in [false, true] {
                let mut o = options(h, simplify);
                o.method = method;
                let result = prepare_contours(&surface, o, &cancel).unwrap();
                assert_eq!(result.loops.len(), 1);
                assert!(contains(&result.loops[0], [12., 8.]));
                assert!(result.report.max_normal_excess_mm <= o.normal_error_mm);
                if simplify && method == camlib::waterline::ContourMethod::QuadTree {
                    assert!(
                        result.report.removed_vertices > 0
                            && result.report.cut_edges < result.report.raw_edges
                    );
                }
                for ring in &result.loops {
                    for (a, b) in ring
                        .iter()
                        .zip(ring.iter().cycle().skip(1))
                        .take(ring.len())
                    {
                        let (low, high) = rectangular_distance_range(*a, *b, h);
                        assert!(
                            low >= o.protected_radius_mm - 1e-8,
                            "gouge {a:?}->{b:?},D{low}"
                        );
                        assert!(
                            high <= o.radius_mm + o.normal_error_mm + 1e-8,
                            "normal error {a:?}->{b:?},D{high}"
                        );
                    }
                }
            }
        }
    }
    let mut tiny = triangular_peak();
    let p = [12.34567, 8.23456];
    tiny.triangles = vec![[
        [p[0], p[1], 8.],
        [p[0] + 0.001, p[1], 8.],
        [p[0], p[1] + 0.001, 8.],
    ]]
    .into();
    let tiny = tiny.prepare(0., 1e-5, &cancel).unwrap();
    for method in [
        camlib::waterline::ContourMethod::QuadTree,
        camlib::waterline::ContourMethod::Support,
    ] {
        let mut o = options(10.00499, false);
        o.method = method;
        let contour = prepare_contours(&tiny, o, &cancel).unwrap();
        assert!(
            contour
                .loops
                .iter()
                .any(|r| contains(r, [p[0] + 0.0002, p[1] + 0.0002])),
            "small between-sample peak disappeared"
        );
    }
}
fn triangular_peak() -> SurfaceMesh {
    SurfaceMesh {
        version: 1,
        units: "mm".into(),
        surface_error_mm: 0.,
        source: serde_json::Value::Null,
        triangles: vec![[[0., 0., 8.], [4., 0., 8.], [0., 4., 8.]]].into(),
    }
}
#[test]
fn point_distance_is_to_solid_and_level_filter_excludes_inactive_base() {
    let cancel = AtomicBool::new(false);
    let mut work = SurfaceWork::default();
    let surface = triangular_peak().prepare(0., 1e-5, &cancel).unwrap();
    for p in [[1., 1., 1.], [1., 1., 7.], [20., 20., -1.]] {
        assert_eq!(
            surface
                .point_distance(p, &mut work, &cancel)
                .unwrap()
                .distance_mm,
            0.
        );
    }
    assert!(
        (surface
            .point_distance([1., 1., 10.], &mut work, &cancel)
            .unwrap()
            .distance_mm
            - 2.)
            .abs()
            < 1e-12
    );
    assert!(
        (surface
            .point_distance([-1., -1., 4.], &mut work, &cancel)
            .unwrap()
            .distance_mm
            - 2f64.sqrt())
        .abs()
            < 1e-12
    );
    let d = surface
        .point_distance([1., 1., 10.], &mut work, &cancel)
        .unwrap();
    assert_eq!(
        surface.witness_distance([1., 1., 10.], d.witness, &mut work),
        d.distance_mm
    );
    assert!(
        surface
            .level_distance([1., 1., 12.], 2., &mut work, &cancel)
            .unwrap()
            .is_none()
    );
    assert!(
        surface
            .level_distance([1., 1., 2.], 2., &mut work, &cancel)
            .is_err()
    );
    assert!(
        surface
            .point_distance([f64::NAN, 0., 1.], &mut work, &cancel)
            .is_err()
    );
    assert!(
        surface
            .point_distance([1., 1., 10.], &mut work, &AtomicBool::new(true))
            .is_err()
    );
}
#[test]
fn filled_shortcut_must_not_sweep_over_a_retained_island() {
    let cancel = AtomicBool::new(false);
    let mut work = SurfaceWork::default();
    let surface = triangular_peak().prepare(0., 1e-5, &cancel).unwrap();
    assert!(
        !surface
            .triangle_clear(
                [[-4., -4., 4.], [10., -4., 4.], [-4., 10., 4.]],
                0.5,
                &mut work,
                &cancel
            )
            .unwrap()
    );
    assert!(
        surface
            .triangle_clear(
                [[-4., -4., 10.], [10., -4., 10.], [-4., 10., 10.]],
                0.5,
                &mut work,
                &cancel
            )
            .unwrap()
    );
}

fn fixture() -> camlib::stream::StreamManifest {
    let m: camlib::stream::StreamManifest =
        serde_json::from_str(include_str!("../fixtures/operations/waterline.json")).unwrap();
    m.validate().unwrap();
    m
}
fn operation(m: &camlib::stream::StreamManifest) -> camlib::waterline::WaterlineOperation {
    match &m.operations[0] {
        camlib::pocket::Operation::Waterline(p) => p.clone(),
        _ => panic!("fixture"),
    }
}
fn swept_height(curve: &camlib::motion::Curve, xy: [f64; 2]) -> f64 {
    let a = curve.start();
    let b = curve.end();
    let d: [f64; 3] = std::array::from_fn(|i| b[i] - a[i]);
    let l2 = d[0] * d[0] + d[1] * d[1];
    let t = if l2 > 0. {
        let projection = ((xy[0] - a[0]) * d[0] + (xy[1] - a[1]) * d[1]) / l2;
        let perpendicular = ((xy[0] - a[0]) * d[1] - (xy[1] - a[1]) * d[0]).powi(2) / l2;
        if perpendicular > 4. {
            return f64::INFINITY;
        }
        (projection - d[2] * ((4. - perpendicular) / (l2 * (l2 + d[2] * d[2]))).sqrt())
            .clamp(0., 1.)
    } else if d[2] < 0. {
        1.
    } else {
        0.
    };
    let radius = (xy[0] - a[0] - t * d[0]).powi(2) + (xy[1] - a[1] - t * d[1]).powi(2);
    if radius > 4. {
        f64::INFINITY
    } else {
        a[2] + t * d[2] + 2. - (4. - radius).sqrt()
    }
}
#[test]
fn cad_frustum_levels_stream_actual_stock_and_preserve_every_target_cell() {
    use camlib::{motion::MoveKind, stock::Stock, waterline::WaterlinePlan};
    let m = fixture();
    let cancel = AtomicBool::new(false);
    for method in [
        camlib::waterline::ContourMethod::QuadTree,
        camlib::waterline::ContourMethod::Support,
    ] {
        let mut op = operation(&m);
        op.contour_method = method;
        op.top_z_mm = 9.;
        op.reverse = true;
        let mut plan = WaterlinePlan::prepare(
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
        let mut position = m.initial_position_mm;
        let mut curves = vec![];
        let mut levels = vec![];
        while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
            assert_eq!(motion.curve.start(), position);
            position = motion.curve.end();
            if motion.kind == MoveKind::Cut {
                assert_eq!(motion.curve.start()[2], position[2]);
                if !levels.contains(&position[2]) {
                    levels.push(position[2]);
                }
            }
            if motion.kind == MoveKind::Rapid {
                assert!(motion.curve.start()[2] >= 12. && position[2] >= 12.);
            }
            let delta = stock
                .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
                .unwrap();
            stock.apply_delta(&delta).unwrap();
            curves.push(motion.curve);
        }
        plan.finish_report(&cancel).unwrap();
        assert_eq!(levels, vec![7., 5., 3., 2.5]);
        assert_eq!(plan.report().levels, 5);
        assert_eq!(plan.report().levels_completed, 5);
        assert_eq!(plan.report().empty_levels, 1);
        assert_eq!(plan.report().last_level_z_mm, Some(2.5));
        assert!(plan.report().loops_completed >= 4);
        assert!(
            plan.report()
                .level_reports
                .iter()
                .all(|l| l.max_normal_excess_mm <= 0.01)
        );
        let tangent = 20f64.to_radians().tan();
        for (i, height) in stock.cells().iter().enumerate() {
            let xy = stock.cell_center(i).unwrap();
            let target = 2.
                + ((8. - (xy[0] - 12.).abs()) / tangent)
                    .min((5. - (xy[1] - 8.).abs()) / tangent)
                    .clamp(0., 6.);
            let actual = curves
                .iter()
                .map(|c| swept_height(c, xy))
                .fold(10f64, f64::min);
            assert!(
                actual >= target - 1e-7,
                "target gouged at{xy:?}:true{actual},target{target}"
            );
            assert!(
                height.lower_mm <= actual + 1e-7 && height.upper_mm >= actual - 1e-7,
                "interval misses exact sweep at{xy:?}:true{actual},{height:?}"
            );
        }
        assert!(stock.volume().removed_lower_mm3 > 100.);
    }
}
struct CancelAfter(std::cell::Cell<usize>);
impl camlib::stock::CancelSignal for CancelAfter {
    fn is_cancelled(&self) -> bool {
        let n = self.0.get();
        self.0.set(n.saturating_sub(1));
        n == 0
    }
}
#[test]
fn waterline_cancel_generation_and_invalid_work_inputs_do_not_advance_cursor() {
    use camlib::{stock::Stock, waterline::WaterlinePlan};
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let op = operation(&m);
    for case in 0..6 {
        let mut p = op.clone();
        let mut tool = m.tools[0].clone();
        match case {
            0 => p.bottom_z_mm = 0.,
            1 => p.stepdown_mm = 0.,
            2 => p.contour_error_mm = 0.,
            3 => std::sync::Arc::make_mut(&mut p.surface.triangles)[0][0][0] = f64::NAN,
            4 => tool.cutter.center_cutting = false,
            _ => p.clearance_z_mm = 10.,
        }
        assert!(
            WaterlinePlan::prepare(
                p,
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
    let mut p = WaterlinePlan::prepare(
        op.clone(),
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    let mut reference = WaterlinePlan::prepare(
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
    // Allow entry, then cancel during contour preparation. Forty coordinator
    // polls can outlast the whole parallel batch on a fast/many-core host, so
    // that counter never requested cancellation before the first motion.
    let interrupted = CancelAfter(std::cell::Cell::new(1));
    assert!(
        p.next_motion_with_stock(&stock, &interrupted)
            .unwrap_err()
            .contains("cancelled")
    );
    assert_eq!(interrupted.0.get(), 0);
    assert_eq!(p.report().level_reports.len(), 0);
    assert_eq!(stock.generation(), 0);
    for _ in 0..3 {
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
    }
    assert!(p.finish_report(&cancel).is_err());
    let surface = triangular_peak().prepare(0., 1e-5, &cancel).unwrap();
    let mut too_fine = options(9., false);
    too_fine.normal_error_mm = 0.00065;
    too_fine.xy_min_mm = [-500_000., -500_000.];
    too_fine.xy_max_mm = [500_000., 500_000.];
    assert!(prepare_contours(&surface, too_fine, &cancel).is_err());
}
#[test]
fn waterline_heights_keep_rapids_above_feed_height_and_end_at_clearance() {
    use camlib::{motion::MoveKind, stock::Stock, waterline::WaterlinePlan};
    use std::sync::atomic::AtomicBool;
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let mut op = operation(&m);
    let clearance = op.clearance_z_mm;
    let top = m.stock.height().unwrap().max_mm[2];
    let (retract, feed) = (
        clearance - (clearance - top) / 4.,
        clearance - (clearance - top) / 2.,
    );
    op.retract_z_mm = Some(retract);
    op.feed_z_mm = Some(feed);
    let mut p = WaterlinePlan::prepare(
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
    assert!(!motions.is_empty());
    for motion in &motions {
        let (a, b) = (motion.curve.start(), motion.curve.end());
        match motion.kind {
            MoveKind::Rapid => assert!(a[2] >= feed && b[2] >= feed, "{motion:?}"),
            MoveKind::Retract => assert_eq!(b[2], feed),
            MoveKind::Plunge => assert_eq!(a[2], feed),
            _ => {}
        }
    }
    assert_eq!(motions.last().unwrap().curve.end()[2], clearance);
    let mut bad = op;
    bad.feed_z_mm = Some(top);
    assert!(
        WaterlinePlan::prepare(
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
#[test]
fn slope_limits_keep_only_the_steep_parts_of_each_contour() {
    use camlib::{motion::MoveKind, stock::Stock, surface::SurfaceWork, waterline::WaterlinePlan};
    // Frustum fixture: 70 degree flanks and vertical walls.
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let cuts = |op: camlib::waterline::WaterlineOperation| {
        let mut plan = WaterlinePlan::prepare(
            op,
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &cancel,
        )
        .unwrap();
        let stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let mut stock = stock;
        let tool = m.tools[0].prepare().unwrap();
        let mut out = vec![];
        while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
            let d = stock
                .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
                .unwrap();
            stock.apply_delta(&d).unwrap();
            if motion.kind == MoveKind::Cut {
                out.push(motion);
            }
        }
        out
    };
    let all = cuts(operation(&m));
    let mut steep = operation(&m);
    // Contour contacts on the flanks and their seams measure 60..70 degrees.
    steep.slope_from_deg = 65.;
    let steep = cuts(steep);
    let op = operation(&m);
    let length = |cuts: &[camlib::motion::Motion]| {
        cuts.iter()
            .map(|c| {
                let (a, b) = (c.curve.start(), c.curve.end());
                (b[0] - a[0]).hypot(b[1] - a[1])
            })
            .sum::<f64>()
    };
    // Densified runs have more, shorter segments: compare cut length.
    assert!(!steep.is_empty() && length(&steep) < length(&all) - 1.);
    let surface = op.surface.prepare(op.floor_z_mm, 1e-5, &cancel).unwrap();
    let mut work = SurfaceWork::default();
    for cut in &steep {
        for p in [cut.curve.start(), cut.curve.end()] {
            let angle = surface
                .contact([p[0], p[1]], 2.005, &mut work, &cancel)
                .unwrap()
                .contact_angle(2.005)
                .to_degrees();
            assert!(
                angle >= 65. - 1e-6,
                "steep-only cut at contact angle {angle}"
            );
        }
    }
}
#[test]
fn parallel_quad_tree_gives_exactly_the_serial_contours() {
    use camlib::waterline::prepare_contours_with;
    let cancel = AtomicBool::new(false);
    // A sloped, faceted dome: many cells on several levels.
    let mut triangles = vec![];
    let z = |x: f64, y: f64| (6. - ((x - 12.).powi(2) + (y - 8.).powi(2)) / 12.).max(1.);
    for i in 0..48 {
        for j in 0..32 {
            let p = |a: usize, b: usize| {
                let (x, y) = (a as f64 * 0.5, b as f64 * 0.5);
                [x, y, z(x, y)]
            };
            triangles.push([p(i, j), p(i + 1, j), p(i + 1, j + 1)]);
            triangles.push([p(i, j), p(i + 1, j + 1), p(i, j + 1)]);
        }
    }
    let mesh = SurfaceMesh {
        version: 1,
        units: "mm".into(),
        surface_error_mm: 0.,
        source: serde_json::Value::Null,
        triangles: triangles.into(),
    };
    let surface = mesh.prepare(0., 1e-5, &cancel).unwrap();
    for h in [4.5, 6., 7.5] {
        let serial = prepare_contours(&surface, options(h, true), &cancel).unwrap();
        for workers in [2, 7, 16] {
            let parallel =
                prepare_contours_with(&surface, options(h, true), workers, &cancel).unwrap();
            assert_eq!(parallel.loops, serial.loops, "level {h}, {workers} workers");
            assert_eq!(parallel.report.grid_cells, serial.report.grid_cells);
        }
        assert!(!serial.loops.is_empty());
    }
}
