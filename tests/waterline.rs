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
        // #46: the 70 degree flanks get depth-1 in-between levels on the full
        // gaps (8, 6, 4); the short last gap (3, 2.5) would gain less than
        // contour_error_mm and gets none; the main levels and their report
        // fields below are unchanged.
        assert_eq!(levels, vec![8., 7., 6., 5., 4., 3., 2.5]);
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
            // A vertical plunge comes down from the feed height; a ramp
            // along a contour (a travelling plunge, #46) starts where the
            // level above or the clear stock height left the tool.
            MoveKind::Plunge if (a[0] - b[0]).hypot(a[1] - b[1]) <= 1e-9 => {
                assert_eq!(a[2], feed)
            }
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

// Slope limits take the ball's contact on its level (#257).

/// Cut length per level (tip Z in micrometres) of `op` run on the fixture's
/// stock, and its level cuts.
fn level_cuts(
    op: &camlib::waterline::WaterlineOperation,
) -> (
    std::collections::BTreeMap<i64, f64>,
    Vec<camlib::motion::Motion>,
) {
    use camlib::{motion::MoveKind, stock::Stock, waterline::WaterlinePlan};
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let mut plan = WaterlinePlan::prepare(
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
    let (mut lengths, mut cuts) = (std::collections::BTreeMap::new(), vec![]);
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        let d = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
            .unwrap();
        stock.apply_delta(&d).unwrap();
        let (a, b) = (motion.curve.start(), motion.curve.end());
        if motion.kind == MoveKind::Cut && a[2] == b[2] {
            *lengths.entry((a[2] * 1000.).round() as i64).or_insert(0.) +=
                (b[0] - a[0]).hypot(b[1] - a[1]);
            cuts.push(motion);
        }
    }
    (lengths, cuts)
}
fn mesh(triangles: Vec<[[f64; 3]; 3]>) -> SurfaceMesh {
    SurfaceMesh {
        version: 1,
        units: "mm".into(),
        surface_error_mm: 0.,
        source: serde_json::Value::Null,
        triangles: triangles.into(),
    }
}
/// A box 8 x 6 mm with its top face at `top`, vertical walls down to the base.
fn box_top(top: f64) -> Vec<[[f64; 3]; 3]> {
    let (a, b, c, d) = (
        [8., 5., top],
        [16., 5., top],
        [16., 11., top],
        [8., 11., top],
    );
    vec![[a, b, c], [a, c, d]]
}
/// The offset of the box's outline by `r`: what one level around it is long.
fn box_perimeter(r: f64) -> f64 {
    2. * (8. + 6.) + std::f64::consts::TAU * r
}
#[test]
fn a_steep_only_waterline_keeps_vertical_walls_whole_at_any_safety_margin() {
    // Below its top edge a vertical wall is a 90 degree contact at every
    // sample of the level's contour, wherever the sample lies between the
    // protected and the inflated offset (issue #257: found by Wookie1, whose
    // box this is). A ball dropped at the sample lands on the floor, or with
    // a larger safety margin on the wall's top edge.
    let m = fixture();
    for (margin, from) in [(0.005, 80.), (0.05, 80.), (0.1, 80.), (0.005, 89.)] {
        let mut op = operation(&m);
        op.surface = mesh(box_top(6.));
        (op.top_z_mm, op.bottom_z_mm, op.stepdown_mm) = (7., 1., 1.);
        op.safety_margin_mm = margin;
        op.slope_from_deg = from;
        let (lengths, _) = level_cuts(&op);
        let whole = box_perimeter(2. + margin);
        let mut walls = 0;
        for (z, length) in &lengths {
            let z = *z as f64 / 1000.;
            if z <= 4. {
                // The ball's center is at or below the top edge: the wall.
                walls += 1;
                assert!(
                    (0.98 * whole..=1.01 * whole).contains(length),
                    "margin {margin}, from {from}: level {z} keeps {length} of {whole} mm"
                );
            } else {
                // Rolling over the top edge at acos((z - 4) / r): under 80
                // degrees from 0.35 mm above.
                assert!(z < 4.4, "margin {margin}: level {z} kept {length} mm");
            }
        }
        assert!(walls >= 3, "margin {margin}: {lengths:?}");
    }
}
#[test]
fn a_steep_only_waterline_keeps_the_level_that_finishes_a_wall_to_its_floor() {
    // The last level a few micrometres above a floor at the wall's foot:
    // the ball touches the wall, not the floor it has not reached.
    let m = fixture();
    // The box on the machining base.
    let mut op = operation(&m);
    op.surface = mesh(box_top(6.));
    (op.top_z_mm, op.bottom_z_mm, op.stepdown_mm) = (5., 0.006, 1.);
    op.slope_from_deg = 80.;
    let (lengths, _) = level_cuts(&op);
    let whole = box_perimeter(2.005);
    let last = lengths[&6];
    assert!(
        (0.98 * whole..=1.01 * whole).contains(&last),
        "{last} of {whole} mm"
    );
    // The box on a plate whose top is a floor of the part at Z3.
    for tip in [3.01, 3.03, 3.06] {
        let mut op = operation(&m);
        let (a, b, c, d) = ([2., 2., 3.], [22., 2., 3.], [22., 14., 3.], [2., 14., 3.]);
        let mut triangles = box_top(6.);
        triangles.extend([[a, b, c], [a, c, d]]);
        op.surface = mesh(triangles);
        (op.top_z_mm, op.bottom_z_mm, op.stepdown_mm) = (6., tip, 1.);
        op.slope_from_deg = 80.;
        let (lengths, _) = level_cuts(&op);
        let last = lengths
            .get(&((tip * 1000.).round() as i64))
            .copied()
            .unwrap_or(0.);
        assert!(
            (0.98 * whole..=1.01 * whole).contains(&last),
            "tip {tip}: {last} of {whole} mm; {lengths:?}"
        );
    }
}
#[test]
fn a_sloped_wall_is_kept_by_its_own_slope_at_every_level() {
    // A flank at `angle` from horizontal, 12 mm long in y, its top edge at
    // x 10, z 8; a vertical wall behind it and at both ends. On the flank,
    // 3 mm clear of the ends, the contact angle is the flank's slope at
    // every level: kept or dropped as a whole, never level by level.
    let m = fixture();
    let flank = |angle: f64| {
        let run = 8. / angle.to_radians().tan();
        let (a, b, c, d) = (
            [10., 2., 8.],
            [10. + run, 2., 0.],
            [10. + run, 14., 0.],
            [10., 14., 8.],
        );
        mesh(vec![[a, b, c], [a, c, d]])
    };
    // Kept length per level beside the flank (x beyond its top edge), for y
    // in 5..11.
    let on_flank = |angle: f64, from: f64, to: f64| {
        let mut op = operation(&m);
        op.surface = flank(angle);
        (op.top_z_mm, op.bottom_z_mm, op.stepdown_mm) = (5.5, 1.5, 1.);
        (op.slope_from_deg, op.slope_to_deg) = (from, to);
        let (lengths, cuts) = level_cuts(&op);
        let mut kept = std::collections::BTreeMap::new();
        for z in lengths.keys() {
            kept.insert(*z, 0.);
        }
        for cut in cuts {
            let (a, b) = (cut.curve.start(), cut.curve.end());
            if (a[0] + b[0]) / 2. > 10.5 {
                let (low, high) = (a[1].min(b[1]).max(5.), a[1].max(b[1]).min(11.));
                *kept.get_mut(&((a[2] * 1000.).round() as i64)).unwrap() += (high - low).max(0.);
            }
        }
        (kept, lengths.len())
    };
    for (angle, from, to, whole) in [
        (70., 80., 90., false),
        (70., 60., 90., true),
        (70., 0., 60., false),
        (80., 85., 90., false),
        (80., 30., 85., true),
        (84., 30., 87., true),
        (84., 87., 90., false),
    ] {
        let (all, _) = on_flank(angle, 0., 90.);
        assert!(all.len() >= 4 && all.values().all(|l| (*l - 6.).abs() < 0.05));
        let (kept, _) = on_flank(angle, from, to);
        for z in all.keys() {
            let length = kept.get(z).copied().unwrap_or(0.);
            assert!(
                if whole {
                    (length - 6.).abs() < 0.05
                } else {
                    length == 0.
                },
                "{angle} degree flank, slope {from}..{to}: level {} keeps {length} mm",
                *z as f64 / 1000.
            );
        }
    }
}
/// A plate (8..16 x 5..11 at Z6) whose surroundings the stock has been
/// roughed around: up to Z6.3 within `reach` mm of the plate, below every
/// level (Z2) farther out.
fn roughed(
    m: &camlib::stream::StreamManifest,
    reach: f64,
) -> (camlib::stock::Stock, camlib::stock::StockSpec) {
    use camlib::stock::{HeightBounds, InitialStock, Stock};
    let spec = m.stock.height().unwrap().clone();
    let probe = Stock::new(spec.clone()).unwrap();
    let [nx, ny] = probe.dimensions();
    let [wx, wy] = probe.cell_widths_mm();
    let near = |x: f64, y: f64| {
        let dx = (8. - x).max(x - 16.).max(0.);
        let dy = (5. - y).max(y - 11.).max(0.);
        dx.hypot(dy)
    };
    let mut bounds = Vec::with_capacity(nx * ny);
    for j in 0..ny {
        for i in 0..nx {
            // Nearest point of the cell to the plate decides its height.
            let (x0, y0) = (i as f64 * wx, j as f64 * wy);
            let (cx, cy) = (x0 + 0.5 * wx, y0 + 0.5 * wy);
            let d = near(cx, cy).min(near(x0, y0)).min(near(x0 + wx, y0 + wy));
            let top = if d <= reach { 6.3 } else { 2. };
            bounds.push(HeightBounds {
                lower_mm: top,
                upper_mm: top,
            });
        }
    }
    let initial = InitialStock {
        source_id: "roughed-plate".into(),
        revision: "1".into(),
        source_sha256: "0".repeat(64),
        source_to_setup: camlib::motion::WorkFrame::identity(),
        uncertainty_mm: 0.,
        representation: "whole_cell_top_down".into(),
        bounds,
    };
    (Stock::from_initial(spec.clone(), &initial).unwrap(), spec)
}
fn plate(m: &camlib::stream::StreamManifest) -> camlib::waterline::WaterlineOperation {
    use camlib::surface::SurfaceMesh;
    let mut op = operation(m);
    let (a, b, c, d) = ([8., 5., 6.], [16., 5., 6.], [16., 11., 6.], [8., 11., 6.]);
    op.surface = SurfaceMesh {
        version: 1,
        units: "mm".into(),
        surface_error_mm: 0.,
        source: serde_json::Value::Null,
        triangles: vec![[a, b, c], [a, c, d]].into(),
    };
    op
}
#[test]
fn every_entry_plunge_ends_at_or_above_the_stock_under_the_ball() {
    use camlib::{motion::MoveKind, waterline::WaterlinePlan};
    // Roughing left a stair 3 mm wide against the plate (Z6.3): a fixed
    // 1 mm offset from the loop still comes down into it. Independently of
    // the plan, every plunge must end where the ball's surface over each
    // stock cell it can touch is at or above the cell's upper bound.
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let (mut stock, spec) = roughed(&m, 3.);
    let mut plan = WaterlinePlan::prepare(
        plate(&m),
        &m.tools[0],
        &spec,
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    let tool = m.tools[0].prepare().unwrap();
    let r = 2.;
    let mut plunges = 0;
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        // Vertical plunges only: a ramp along a contour (a travelling
        // plunge, #46) cuts the wall like the loop's own moves.
        let vertical = {
            let (a, b) = (motion.curve.start(), motion.curve.end());
            (a[0] - b[0]).hypot(a[1] - b[1]) <= 1e-9
        };
        if motion.kind == MoveKind::Plunge && vertical {
            let p = motion.curve.end();
            let [nx, ny] = stock.dimensions();
            let [wx, wy] = stock.cell_widths_mm();
            for j in 0..ny {
                for i in 0..nx {
                    let (x0, y0) = (i as f64 * wx, j as f64 * wy);
                    let d = (x0 - p[0]).max(p[0] - x0 - wx).max(0.).hypot(
                        (y0 - p[1]).max(p[1] - y0 - wy).max(0.),
                    );
                    if d < r {
                        let ball = p[2] + r - (r * r - d * d).sqrt();
                        let upper = stock.cells()[j * nx + i].upper_mm;
                        assert!(
                            ball >= upper - 1e-6,
                            "plunge to {p:?} enters stock (cell {i},{j} up to {upper}, ball at {ball})"
                        );
                    }
                }
            }
            plunges += 1;
        }
        let d = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
            .unwrap();
        stock.apply_delta(&d).unwrap();
    }
    // The island enters on a ramp from the band top (a lead from beside the
    // band into it would cut it from the side, #46 WL3) and its lower levels
    // follow by ramps: the one vertical plunge stops on the band top.
    assert!(plunges > 0);
    assert_eq!(plan.report().moved_entries, 0);
    assert!(plan.report().helical_entries >= 1);
}
/// Plan the whole operation, committing each move to the stock.
fn drain(
    m: &camlib::stream::StreamManifest,
    plan: &mut camlib::waterline::WaterlinePlan,
    stock: &mut camlib::stock::Stock,
) {
    let cancel = AtomicBool::new(false);
    let tool = m.tools[0].prepare().unwrap();
    while let Some(motion) = plan.next_motion_with_stock(stock, &cancel).unwrap() {
        let d = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
            .unwrap();
        stock.apply_delta(&d).unwrap();
    }
}
#[test]
fn a_loop_with_no_stock_free_entry_is_reported_not_hidden() {
    use camlib::waterline::WaterlinePlan;
    // Raw stock to Z10 stands above every level everywhere: no point beside
    // the wall is free of stock. Levels Z3.5 and Z2.5 (1 mm stepdown), so
    // the raw stock stands 6.5 mm - more than three stepdowns - above the
    // first; the roughed band below stands 2.8 mm above it, as a rough in
    // 3 mm steps (three stepdowns) leaves on a slope.
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let shallow = || {
        let mut op = plate(&m);
        op.stepdown_mm = 1.;
        op.top_z_mm = 3.5;
        op
    };
    let mut plan = WaterlinePlan::prepare(
        shallow(),
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    let mut stock = camlib::stock::Stock::new(m.stock.height().unwrap().clone()).unwrap();
    drain(&m, &mut plan, &mut stock);
    // Raw stock stands above every level: a point beyond the stock's edge
    // has no stock under the ball, but the lead from it onto the wall would
    // run through the raw stock, so no side entry is taken (#46: a side
    // entry only where its lead is pure air); the island enters on a ramp.
    assert_eq!(plan.report().moved_entries, 0);
    assert!(plan.report().helical_entries >= 1);
    // The ramp reaches its level with the raw stock standing more than
    // three stepdowns above it: counted, so the simulation warns to rough
    // first.
    assert!(plan.report().deep_stock_entries >= 1);
    // Roughed: a point beside the wall is clear, but the roughed band runs
    // to Z6.3 within 3 mm of the plate, so a lead from it onto the wall
    // below Z6.3 would cut the band from the side: the island enters on a
    // ramp instead (#46), and its lower levels are linked by ramps.
    let (mut stock, spec) = roughed(&m, 3.);
    let mut plan = WaterlinePlan::prepare(
        shallow(),
        &m.tools[0],
        &spec,
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    drain(&m, &mut plan, &mut stock);
    assert_eq!(plan.report().moved_entries, 0);
    assert!(plan.report().helical_entries >= 1);
    // Roughing in steps of three stepdowns is not counted (with a margin of
    // one stepdown, the 2.8 mm band would be).
    assert_eq!(plan.report().deep_stock_entries, 0);
}

/// Independent replay measure for the entry rule (#46): drives the plan
/// against the live stock (every move's sweep applied in order, like
/// `drain`) and measures every Plunge or Rapid move that descends, walking
/// it in steps of <= 0.1 mm and taking the deepest reach of the stock under
/// the ball disk below the ball tip (`Stock::top_under_disk` - tip z). A
/// descent that stays on one XY is a vertical plunge: it must meet no stock
/// deeper than the operation tolerance. A descending Plunge that travels is
/// a ramp entry along the loop: it cuts the wall like the loop's own moves,
/// so it may meet stock, and the test bounds that by the level gap the ramp
/// descends. Also records the move count, the first entry plunge's end, the
/// ramp's first start and last end, and the cut length per level, to
/// compare with values measured at 2017dcc.
#[derive(Debug)]
struct Replay {
    motions: usize,
    first_entry: Option<[f64; 3]>,
    cut_length_by_level: std::collections::BTreeMap<u64, f64>,
    vertical_plunges: usize,
    helix_moves: usize,
    worst_vertical: f64,
    worst_helix: f64,
    helix_first_start: Option<[f64; 3]>,
    helix_last_end: Option<[f64; 3]>,
    /// The first motion the run's stream rejects (`StreamProducer` and
    /// `StreamReceiver`, what the simulation and the UI play back): its
    /// index, the error, and the motion against the previous end. None: the
    /// whole plan streams.
    stream_error: Option<String>,
}

/// Every motion of a plan through the stream the simulation plays back: the
/// producer, and the receiver that rechecks each frame (attribution,
/// continuity, increasing ids). The first rejection, described.
fn stream_error(
    m: &camlib::stream::StreamManifest,
    motions: &[camlib::motion::Motion],
) -> Option<String> {
    use camlib::stream::{DEFAULT_FRAME_LIMIT, StreamProducer, StreamReceiver};
    let cancel = AtomicBool::new(false);
    let mut producer = StreamProducer::new(m.clone()).unwrap();
    let mut receiver = StreamReceiver::new(m.clone()).unwrap();
    let mut previous = m.initial_position_mm;
    for (i, motion) in motions.iter().enumerate() {
        let checked = producer
            .push_motion(motion.clone(), DEFAULT_FRAME_LIMIT, &cancel)
            .and_then(|frame| receiver.accept(&frame, DEFAULT_FRAME_LIMIT).map(|_| ()));
        if let Err(error) = checked {
            return Some(format!(
                "motion {i} (id {}, {:?}) starts at {:?}, the previous ended at {previous:?}: {error}",
                motion.id,
                motion.kind,
                motion.curve.start()
            ));
        }
        previous = motion.curve.end();
    }
    None
}

fn replay(
    m: &camlib::stream::StreamManifest,
    plan: &mut camlib::waterline::WaterlinePlan,
    stock: &mut camlib::stock::Stock,
) -> Replay {
    use camlib::motion::MoveKind;
    let cancel = AtomicBool::new(false);
    let tool = m.tools[0].prepare().unwrap();
    let r = 2.; // the fixture ball4
    let mut out = Replay {
        motions: 0,
        first_entry: None,
        cut_length_by_level: Default::default(),
        vertical_plunges: 0,
        helix_moves: 0,
        worst_vertical: f64::NEG_INFINITY,
        worst_helix: f64::NEG_INFINITY,
        helix_first_start: None,
        helix_last_end: None,
        stream_error: None,
    };
    let mut motions = vec![];
    while let Some(motion) = plan.next_motion_with_stock(stock, &cancel).unwrap() {
        motions.push(motion.clone());
        out.motions += 1;
        let a = motion.curve.start();
        let b = motion.curve.end();
        if motion.kind == MoveKind::Plunge && out.first_entry.is_none() {
            out.first_entry = Some(b);
        }
        if motion.kind == MoveKind::Cut {
            let key = (b[2] * 1e6).round() as u64;
            *out.cut_length_by_level.entry(key).or_default() +=
                ((b[0] - a[0]).hypot(b[1] - a[1])).hypot(b[2] - a[2]);
        }
        if b[2] < a[2] - 1e-12 && matches!(motion.kind, MoveKind::Plunge | MoveKind::Rapid) {
            let travels = (a[0] - b[0]).hypot(a[1] - b[1]) > 1e-9;
            let length = ((b[0] - a[0]).hypot(b[1] - a[1])).hypot(b[2] - a[2]);
            let steps = (length / 0.1).ceil().max(1.) as usize;
            let mut worst = f64::NEG_INFINITY;
            for i in 0..=steps {
                let t = i as f64 / steps as f64;
                let p = [
                    a[0] + t * (b[0] - a[0]),
                    a[1] + t * (b[1] - a[1]),
                    a[2] + t * (b[2] - a[2]),
                ];
                worst = worst.max(stock.top_under_disk([p[0], p[1]], r) - p[2]);
            }
            if travels && motion.kind == MoveKind::Plunge {
                out.helix_moves += 1;
                if out.helix_first_start.is_none() {
                    out.helix_first_start = Some(a);
                }
                out.helix_last_end = Some(b);
                out.worst_helix = out.worst_helix.max(worst);
            } else {
                out.vertical_plunges += 1;
                out.worst_vertical = out.worst_vertical.max(worst);
            }
        }
        let d = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
            .unwrap();
        stock.apply_delta(&d).unwrap();
    }
    out.stream_error = stream_error(m, &motions);
    out
}

#[test]
fn blocked_loop_starts_move_to_a_clear_entry() {
    use camlib::{
        motion::WorkFrame,
        stock::{HeightBounds, InitialStock, Stock},
        waterline::WaterlinePlan,
    };
    // The plate's Z6 loop starts at its contour's own start (measured at
    // 2017dcc; the contour depends only on the part, not the stock). A lump
    // of uncut stock at Z6.3 over every cell within 10.5 mm of that start
    // blocks every candidate beside it - they are searched up to 8 mm out
    // and the ball disk reaches 2.283 mm further - while points farther
    // along the loop have clear air: the loop must move its start to a
    // clear entry and never plunge into the lump (#46).
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let given_start = [16.201954570312502, 4.9751401953125];
    let tolerance = plate(&m).tolerance_mm;
    let spec = m.stock.height().unwrap().clone();
    let probe = Stock::new(spec.clone()).unwrap();
    let [nx, ny] = probe.dimensions();
    let [wx, wy] = probe.cell_widths_mm();
    let mut bounds = Vec::with_capacity(nx * ny);
    for j in 0..ny {
        for i in 0..nx {
            let (x0, y0) = (i as f64 * wx, j as f64 * wy);
            let dx = (x0 - given_start[0]).max(given_start[0] - x0 - wx).max(0.);
            let dy = (y0 - given_start[1]).max(given_start[1] - y0 - wy).max(0.);
            let top = if dx.hypot(dy) <= 10.5 { 6.3 } else { 2. };
            bounds.push(HeightBounds {
                lower_mm: top,
                upper_mm: top,
            });
        }
    }
    let initial = InitialStock {
        source_id: "roughed-plate".into(),
        revision: "1".into(),
        source_sha256: "0".repeat(64),
        source_to_setup: WorkFrame::identity(),
        uncertainty_mm: 0.,
        representation: "whole_cell_top_down".into(),
        bounds,
    };
    let mut stock = Stock::from_initial(spec.clone(), &initial).unwrap();
    let mut plan = WaterlinePlan::prepare(
        plate(&m),
        &m.tools[0],
        &spec,
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    let replay = replay(&m, &mut plan, &mut stock);
    // The whole plan streams: every motion starts where the previous one
    // ended, with an increasing id (what the simulation and the UI check).
    assert!(replay.stream_error.is_none(), "{:?}", replay.stream_error);
    // No vertical descent meets stock deeper than the tolerance. A side
    // entry is taken only where its lead onto the wall is pure air (#46
    // WL3): every lead from a clear point here crosses the lump or its
    // edge, so the island enters on a ramp along its loop, and the lower
    // levels follow by ramps from the level above.
    assert!(replay.vertical_plunges > 0);
    assert!(
        replay.worst_vertical <= tolerance + 1e-6,
        "worst vertical plunge met stock {} deep",
        replay.worst_vertical
    );
    assert_eq!(plan.report().moved_entries, 0);
    assert!(plan.report().helical_entries >= 1);
    // The loop still cuts its full contour: the cut lengths per level
    // measured at 2017dcc (the contour depends only on the part).
    for (z, len) in [
        (6_000_000, 28.968869111),
        (4_000_000, 40.629012855),
        (2_500_000, 40.629012855),
    ] {
        let cut = replay.cut_length_by_level.get(&z).copied().unwrap_or(0.);
        assert!(
            (cut - len).abs() <= 1e-6,
            "level Z{} cut length {cut}",
            z as f64 / 1e6
        );
    }
}

#[test]
fn fully_blocked_loops_enter_on_a_helix() {
    use camlib::{
        stock::{Stock, StockSpec},
        waterline::WaterlinePlan,
    };
    // An island 16..24 x 16..24 at Z6 in a 40x40 stock standing at Z7, the
    // single waterline level Z6: the stock under the ball stands a full
    // millimetre above the level at every point beside the loop, on and off
    // it, so no side entry exists anywhere on the loop. The entry must be
    // the ramp along the loop itself: a plunge to the lowest height the
    // stock under the ball at the loop start is clear at (the stock top Z7),
    // then a 3 degree ramp (the default) down to the level (#46).
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let spec = StockSpec {
        version: 1,
        units: "mm".into(),
        min_mm: [0., 0., 0.],
        max_mm: [40., 40., 7.],
        cell_size_mm: 0.4,
        max_cells: 1_048_576,
    };
    let mut op = operation(&m);
    let tolerance = op.tolerance_mm;
    op.surface = SurfaceMesh {
        version: 1,
        units: "mm".into(),
        surface_error_mm: 0.,
        source: serde_json::Value::Null,
        triangles: vec![
            [[16., 16., 6.], [24., 16., 6.], [24., 24., 6.]],
            [[16., 16., 6.], [24., 24., 6.], [16., 24., 6.]],
        ]
        .into(),
    };
    op.top_z_mm = 6.;
    op.bottom_z_mm = 6.;
    let mut stock = Stock::new(spec.clone()).unwrap();
    let mut plan = WaterlinePlan::prepare(
        op,
        &m.tools[0],
        &spec,
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    let replay = replay(&m, &mut plan, &mut stock);
    // The whole plan streams: every motion starts where the previous one
    // ended, with an increasing id (what the simulation and the UI check).
    assert!(replay.stream_error.is_none(), "{:?}", replay.stream_error);
    // The entry plunge stops at the stock top Z7, in air; the ramp
    // meets the wall it cuts, never deeper than the level gap it descends.
    assert!(replay.vertical_plunges > 0);
    assert!(
        replay.worst_vertical <= tolerance + 1e-6,
        "worst vertical plunge met stock {} deep",
        replay.worst_vertical
    );
    assert!(replay.helix_moves > 0);
    assert!(
        replay.worst_helix <= 1. + 1e-6,
        "the helix met stock {} deep, the level gap is 1",
        replay.worst_helix
    );
    assert_eq!(plan.report().helical_entries, 1);
    assert_eq!(plan.report().moved_entries, 0);
    // The ramp (the 3 degree default, #46) ends at the level, the level gap
    // over tan 3 degrees along the loop from where it began - shorter than
    // the loop, so it ends away from its start.
    let first = replay.helix_first_start.expect("a ramp");
    let last = replay.helix_last_end.expect("a ramp");
    assert!((last[2] - 6.).abs() <= 1e-9, "the ramp ends at {last:?}");
    assert!((first[2] - 7.).abs() <= 1e-9, "the ramp begins at {first:?}");
    assert!(
        (last[0] - first[0]).hypot(last[1] - first[1]) > 1.,
        "the ramp ends at {last:?}, it began at {first:?}: a whole lap"
    );
}

#[test]
fn open_runs_enter_on_a_helix_that_returns_to_their_start() {
    use camlib::{
        stock::{Stock, StockSpec},
        waterline::WaterlinePlan,
    };
    // The frustum fixture with slope limits from 65 degrees: the limits cut
    // each contour into open runs (the flanks' seams measure 60..70
    // degrees). An unroughed block standing at Z9, 1 mm above the top level,
    // reaching 20 mm past the part on every side, so every point beside
    // every run has stock above its level and the runs enter on a helix. An
    // open run ends away from its start, so its helix must come back along
    // it to the start the cut begins from: the whole plan streams (motion
    // continuity, as the simulation and the UI check it), and no vertical
    // plunge meets stock (#46).
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let mut op = operation(&m);
    op.slope_from_deg = 65.;
    let tolerance = op.tolerance_mm;
    let spec = StockSpec {
        version: 1,
        units: "mm".into(),
        min_mm: [-20., -20., 0.],
        max_mm: [44., 36., 9.],
        cell_size_mm: 0.4,
        max_cells: 1_048_576,
    };
    let mut stock = Stock::new(spec.clone()).unwrap();
    let mut plan = WaterlinePlan::prepare(op, &m.tools[0], &spec, m.initial_position_mm, 1, &cancel)
        .unwrap();
    let replay = replay(&m, &mut plan, &mut stock);
    // The whole plan streams: every motion starts where the previous one
    // ended, with an increasing id (what the simulation and the UI check).
    assert!(replay.stream_error.is_none(), "{:?}", replay.stream_error);
    assert!(plan.report().helical_entries > 0);
    assert!(replay.helix_moves > 0);
    assert!(
        replay.worst_vertical <= tolerance + 1e-6,
        "worst vertical plunge met stock {} deep",
        replay.worst_vertical
    );
}

#[test]
fn unblocked_loops_keep_their_start() {
    use camlib::waterline::WaterlinePlan;
    // The frustum fixture, roughed so the island enters beside its given
    // start: the whole-loop search never runs, no loop enters on a helix.
    // Its lower levels are reached by ramps from the level above (#46 WL3):
    // the first entry and every level's cut length are the ones measured
    // at 2017dcc; the moves are now 335 (184 with a retract per level).
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let tolerance = operation(&m).tolerance_mm;
    let (mut stock, spec) = roughed(&m, 3.);
    let mut plan = WaterlinePlan::prepare(
        operation(&m),
        &m.tools[0],
        &spec,
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    let replay = replay(&m, &mut plan, &mut stock);
    // The whole plan streams: every motion starts where the previous one
    // ended, with an increasing id (what the simulation and the UI check).
    assert!(replay.stream_error.is_none(), "{:?}", replay.stream_error);
    assert_eq!(plan.report().moved_entries, 0);
    assert_eq!(plan.report().helical_entries, 0);
    assert_eq!(replay.motions, 246);
    let first = replay.first_entry.expect("an entry plunge");
    assert!(
        (first[0] - 19.015079355936262).abs() <= 1e-9
            && (first[1] - 5.168476859157234).abs() <= 1e-9
            && (first[2] - 8.).abs() <= 1e-9,
        "first entry {first:?}"
    );
    assert!(replay.vertical_plunges > 0);
    assert!(
        replay.worst_vertical <= tolerance + 1e-6,
        "worst vertical plunge met stock {} deep",
        replay.worst_vertical
    );
    assert!(plan.report().ramp_links > 0);
}

// In-between levels on shallow slopes keep the scallop bounded (#46).

/// A faceted spherical cap: sphere radius 12 centred (12, 8, -6), everything
/// above z 1 (top at z 6), triangulated on a 0.25 mm XY grid over the
/// 24 x 16 stock like `parallel_quad_tree_gives_exactly_the_serial_contours`.
fn spherical_cap() -> SurfaceMesh {
    let z = |x: f64, y: f64| {
        let r2 = (x - 12.).powi(2) + (y - 8.).powi(2);
        if r2 >= 144. {
            1.
        } else {
            (-6. + (144. - r2).sqrt()).max(1.)
        }
    };
    let mut triangles = vec![];
    for i in 0..96 {
        for j in 0..64 {
            let p = |a: usize, b: usize| {
                let (x, y) = (a as f64 * 0.25, b as f64 * 0.25);
                [x, y, z(x, y)]
            };
            triangles.push([p(i, j), p(i + 1, j), p(i + 1, j + 1)]);
            triangles.push([p(i, j), p(i + 1, j + 1), p(i, j + 1)]);
        }
    }
    mesh(triangles)
}
/// The fixture's ball operation on the spherical cap, stepdown 1 (R = 2, so
/// scallop_mm = 2 - sqrt(3.75) + 0.01, K = 3, scallop_from_deg =
/// asin(min(1, 1/(2^K * D))) with D = 2 sqrt(R^2 - (R - scallop_mm)^2)).
fn cap_operation(m: &camlib::stream::StreamManifest) -> camlib::waterline::WaterlineOperation {
    let mut op = operation(m);
    op.surface = spherical_cap();
    op.stepdown_mm = 1.;
    op.top_z_mm = 5.8;
    op.bottom_z_mm = 1.5;
    op
}
/// The main level heights of an operation, the way the planner computes them.
fn main_levels(op: &camlib::waterline::WaterlineOperation) -> Vec<f64> {
    let levels = ((op.top_z_mm - op.bottom_z_mm) / op.stepdown_mm).ceil() as usize + 1;
    (0..levels)
        .map(|i| {
            if i + 1 == levels {
                op.bottom_z_mm
            } else {
                (op.top_z_mm - i as f64 * op.stepdown_mm).max(op.bottom_z_mm)
            }
        })
        .collect()
}
/// Every sub-level height the halving rule can place between main levels
/// (depths 1..=k_max, odd multiples of the gap over 2^k below the upper
/// level), whether or not a gap turns out eligible for them.
fn all_sub_levels(mains: &[f64], k_max: usize) -> Vec<f64> {
    let mut subs = vec![];
    for w in mains.windows(2) {
        let (za, zb) = (w[0], w[1]);
        for k in 1..=k_max {
            let step = (za - zb) / 2f64.powi(k as i32);
            let mut j = 1;
            while za - j as f64 * step > zb {
                subs.push(za - j as f64 * step);
                j += 2;
            }
        }
    }
    subs
}
#[test]
fn shallow_slopes_get_in_between_levels_and_the_scallop_stays_bounded() {
    use camlib::{motion::MoveKind, stock::Stock, waterline::WaterlinePlan};
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let op = cap_operation(&m);
    let mut plan = WaterlinePlan::prepare(
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
    // Ball-centre points of every cut, densified every 0.05 mm.
    let mut centres: Vec<[f64; 3]> = vec![];
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        let d = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
            .unwrap();
        stock.apply_delta(&d).unwrap();
        if motion.kind == MoveKind::Cut {
            let (a, b) = (motion.curve.start(), motion.curve.end());
            let len = ((b[0] - a[0]).hypot(b[1] - a[1])).hypot(b[2] - a[2]);
            let n = (len / 0.05).ceil().max(1.) as usize;
            for k in 0..=n {
                let t = k as f64 / n as f64;
                let p: [f64; 3] = std::array::from_fn(|i| a[i] + t * (b[i] - a[i]));
                centres.push([p[0], p[1], p[2] + 2.]);
            }
        }
    }
    plan.finish_report(&cancel).unwrap();
    let report = plan.report();
    // The bound and its reported values, from the formulas (#46): the same
    // scallop as a vertical wall plus the path tolerance, capped at R.
    let scallop_mm = (2. - (4.0f64 - 1. / 4.).sqrt() + op.contour_error_mm).min(2.);
    // The widest spacing along the surface that keeps the bound, the deepest
    // useful sub-level, and the angle below which the bound is not claimed.
    let d = 2. * (4.0f64 - (2. - scallop_mm) * (2. - scallop_mm)).sqrt();
    let mut k = 0usize;
    while 1. / 2f64.powi(k as i32 + 1) >= scallop_mm {
        k += 1;
    }
    let scallop_from_deg = (1. / (2f64.powi(k as i32) * d)).min(1.).asin().to_degrees();
    let bound =
        scallop_mm + op.contour_error_mm + op.safety_margin_mm + op.stock_to_leave_mm + 0.02;
    // INDEPENDENT measure (no planner angles): analytic sphere points against
    // the ball centres.
    let mut worst = (f64::NEG_INFINITY, [0f64; 3], 0f64);
    let mut checked = 0usize;
    for ti in 0..180 {
        let t = (ti as f64 * 0.5).to_radians();
        if t < scallop_from_deg.to_radians() + 2f64.to_radians() {
            continue;
        }
        for az in 0..180 {
            let a = (az as f64 * 2.).to_radians();
            let p = [
                12. + 12. * t.sin() * a.cos(),
                8. + 12. * t.sin() * a.sin(),
                -6. + 12. * t.cos(),
            ];
            let tip = p[2] + 2. * t.cos() - 2.;
            if !(tip > 1.5 + 0.2 && tip < 5.8 - 0.2) {
                continue;
            }
            if !(p[0] >= 2.5 && p[0] <= 21.5 && p[1] >= 2.5 && p[1] <= 13.5) {
                continue;
            }
            checked += 1;
            let e = centres
                .iter()
                .map(|c| ((p[0] - c[0]).hypot(p[1] - c[1])).hypot(p[2] - c[2]) - 2.)
                .fold(f64::INFINITY, f64::min);
            if e > worst.0 {
                worst = (e, p, t.to_degrees());
            }
        }
    }
    assert!(checked > 100, "independent measure checked only {checked} points");
    assert!(
        worst.0 <= bound,
        "scallop bound {bound} exceeded: worst e = {} at p = {:?}, contact angle {} deg",
        worst.0,
        worst.1,
        worst.2
    );
    assert!(report.scallop_levels > 0, "no in-between level cut anything");
    assert!(
        (report.scallop_mm.unwrap() - scallop_mm).abs() < 1e-12,
        "scallop_mm {} != {scallop_mm}",
        report.scallop_mm.unwrap()
    );
    assert!(
        (report.scallop_from_deg.unwrap() - scallop_from_deg).abs() < 1e-12,
        "scallop_from_deg {} != {scallop_from_deg}",
        report.scallop_from_deg.unwrap()
    );
}
#[test]
fn vertical_walls_get_no_in_between_levels() {
    use camlib::{motion::MoveKind, stock::Stock, waterline::WaterlinePlan};
    // A box with vertical walls: its top face is exactly horizontal, and a
    // vertical wall is a 90 degree contact, which the 1e-9 tie of the keep
    // test drops at depth 1, so no in-between level cuts anything (#46).
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let mut op = operation(&m);
    op.surface = mesh(box_top(6.));
    let mut plan = WaterlinePlan::prepare(
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
    let mut flat_cut_z = vec![];
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        let d = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
            .unwrap();
        stock.apply_delta(&d).unwrap();
        let (a, b) = (motion.curve.start(), motion.curve.end());
        if motion.kind == MoveKind::Cut && a[2] == b[2] {
            flat_cut_z.push(b[2]);
        }
    }
    plan.finish_report(&cancel).unwrap();
    assert_eq!(plan.report().scallop_levels, 0);
    // R = 2, stepdown 2: scallop_mm = 2 - sqrt(4 - 1) + 0.01, K = 2, so the
    // bound is not claimed below asin(min(1, 2/(2^K D))) degrees.
    let scallop_mm = (2. - (4.0f64 - 1.).sqrt() + op.contour_error_mm).min(2.);
    let d = 2. * (4.0f64 - (2. - scallop_mm) * (2. - scallop_mm)).sqrt();
    let mut k = 0usize;
    while 2. / 2f64.powi(k as i32 + 1) >= scallop_mm {
        k += 1;
    }
    assert_eq!(k, 2);
    let scallop_from_deg = (2. / (2f64.powi(k as i32) * d)).min(1.).asin().to_degrees();
    assert!(
        (plan.report().scallop_from_deg.unwrap() - scallop_from_deg).abs() < 1e-12,
        "scallop_from_deg {} != {scallop_from_deg}",
        plan.report().scallop_from_deg.unwrap()
    );
    let levels = main_levels(&op);
    assert_eq!(levels, vec![8., 6., 4., 2.5]);
    for z in flat_cut_z {
        assert!(
            levels.iter().any(|l| (l - z).abs() < 1e-9),
            "flat cut at {z} is not a main level {levels:?}"
        );
    }
}
#[test]
fn sloped_flanks_get_in_between_levels() {
    use camlib::{stock::Stock, waterline::WaterlinePlan};
    // The frustum fixture's 70 degree flanks are not vertical: with the same
    // scallop as a vertical wall they get in-between levels (#46).
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let op = operation(&m);
    let mut plan = WaterlinePlan::prepare(
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
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        let d = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
            .unwrap();
        stock.apply_delta(&d).unwrap();
    }
    plan.finish_report(&cancel).unwrap();
    assert!(
        plan.report().scallop_levels > 0,
        "no in-between level cut anything on the 70 degree flanks"
    );
}
#[test]
fn fine_stepdowns_add_no_levels_below_the_path_tolerance() {
    use camlib::{stock::Stock, waterline::WaterlinePlan};
    // At a 0.1 mm stepdown the wall scallop is ~0.0006 mm; an in-between
    // level must lower the scallop by more than contour_error_mm (0.01) to
    // be added, and nothing on the frustum gains that much: the 70 degree
    // flanks' spacing along the surface is already under the widest spacing
    // the bound allows, so no level cuts anything (#46).
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let mut op = operation(&m);
    op.stepdown_mm = 0.1;
    let mut plan = WaterlinePlan::prepare(
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
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        let d = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
            .unwrap();
        stock.apply_delta(&d).unwrap();
    }
    plan.finish_report(&cancel).unwrap();
    assert_eq!(plan.report().scallop_levels, 0);
}
#[test]
fn in_between_levels_respect_slope_limits() {
    use camlib::{motion::MoveKind, stock::Stock, surface::SurfaceWork, waterline::WaterlinePlan};
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let mut op = cap_operation(&m);
    op.slope_from_deg = 40.;
    let mut plan = WaterlinePlan::prepare(
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
    let surface = op.surface.prepare(op.floor_z_mm, 1e-5, &cancel).unwrap();
    let mut work = SurfaceWork::default();
    let mains = main_levels(&op);
    let mut sub_level_cuts = 0usize;
    // The cap is faceted on a 0.25 mm grid of a radius-12 sphere, so
    // neighbouring facet normals differ by up to atan(0.25*sqrt(2)/12) =
    // 1.69 degrees; the drop-ball measure here and the planner's level
    // contact can land on neighbouring facets, so a cut the planner admitted
    // at 40 degrees can measure 1.69 degrees less here.
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        let d = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
            .unwrap();
        stock.apply_delta(&d).unwrap();
        if motion.kind != MoveKind::Cut {
            continue;
        }
        let end = motion.curve.end();
        if !mains.iter().any(|l| (l - end[2]).abs() < 1e-9) {
            sub_level_cuts += 1;
        }
        let angle = surface
            .contact([end[0], end[1]], 2.005, &mut work, &cancel)
            .unwrap()
            .contact_angle(2.005)
            .to_degrees();
        assert!(
            angle >= 40. - 1.69 - 1e-6,
            "cut end at ({:?}) has contact angle {angle}",
            &end[..2]
        );
    }
    assert!(sub_level_cuts > 0, "no cut lay on an in-between level");
}
#[test]
fn ramp_cuts_in_between_levels_flat() {
    use camlib::{motion::MoveKind, stock::Stock, waterline::WaterlinePlan};
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let mut op = cap_operation(&m);
    op.ramp = true;
    let mut plan = WaterlinePlan::prepare(
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
    let mains = main_levels(&cap_operation(&m));
    let subs = all_sub_levels(&mains, 3);
    let mut sub_level_cuts = 0usize;
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        let d = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
            .unwrap();
        stock.apply_delta(&d).unwrap();
        if motion.kind != MoveKind::Cut {
            continue;
        }
        let (a, b) = (motion.curve.start(), motion.curve.end());
        if subs.iter().any(|s| (s - b[2]).abs() < 1e-9) {
            assert_eq!(a[2], b[2], "sub-level cut at z {} ramps from {}", b[2], a[2]);
            sub_level_cuts += 1;
        }
    }
    assert!(sub_level_cuts > 0, "no cut lay on an in-between level");
}

// Ball waterline cuts each island top to bottom and ramps between levels
// (#46, WL3).

/// Every motion of a plan with the deepest any stock cell sank under it
/// (preview of its sweep on the live stock, committed in order: an
/// independent measure of what the move actually cut), and the stream check
/// of the whole plan (`stream_error`).
fn trace(
    m: &camlib::stream::StreamManifest,
    plan: &mut camlib::waterline::WaterlinePlan,
    stock: &mut camlib::stock::Stock,
) -> (Vec<(camlib::motion::Motion, f64)>, Option<String>) {
    let cancel = AtomicBool::new(false);
    let tool = m.tools[0].prepare().unwrap();
    let mut out = vec![];
    while let Some(motion) = plan.next_motion_with_stock(stock, &cancel).unwrap() {
        let d = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
            .unwrap();
        let cut = d
            .changes()
            .iter()
            .map(|c| c.before.upper_mm - c.after.upper_mm)
            .fold(0., f64::max);
        stock.apply_delta(&d).unwrap();
        out.push((motion, cut));
    }
    let motions: Vec<_> = out.iter().map(|(m, _)| m.clone()).collect();
    let error = stream_error(m, &motions);
    (out, error)
}
/// Top faces of boxes `[x0, y0, x1, y1, top]` with vertical walls down to
/// the base.
fn boxes(list: &[[f64; 5]]) -> SurfaceMesh {
    mesh(
        list.iter()
            .flat_map(|&[x0, y0, x1, y1, z]| {
                let (a, b, c, d) = ([x0, y0, z], [x1, y0, z], [x1, y1, z], [x0, y1, z]);
                [[a, b, c], [a, c, d]]
            })
            .collect(),
    )
}
fn travels(motion: &camlib::motion::Motion) -> bool {
    let (a, b) = (motion.curve.start(), motion.curve.end());
    (a[0] - b[0]).hypot(a[1] - b[1]) > 1e-9
}
#[test]
fn two_islands_are_cut_one_at_a_time() {
    use camlib::{motion::MoveKind, stock::Stock, waterline::WaterlinePlan};
    // Two box islands 6 mm apart on the unroughed block, levels 6, 4, 2.5:
    // each island is one stack, cut top to bottom before the other, so the
    // cutting moves change island once (#46).
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let mut op = operation(&m);
    op.surface = boxes(&[[3., 5., 8., 11., 6.], [14., 5., 19., 11., 6.]]);
    op.top_z_mm = 6.;
    let tolerance = op.tolerance_mm;
    let spec = m.stock.height().unwrap().clone();
    let mut stock = Stock::new(spec.clone()).unwrap();
    let mut plan =
        WaterlinePlan::prepare(op, &m.tools[0], &spec, m.initial_position_mm, 1, &cancel).unwrap();
    let (moves, error) = trace(&m, &mut plan, &mut stock);
    assert!(error.is_none(), "{error:?}");
    let report = plan.report();
    assert_eq!((report.stacks, report.stacks_level_ordered), (2, 0));
    let islands: Vec<bool> = moves
        .iter()
        .filter(|(m, _)| m.kind == MoveKind::Cut)
        .map(|(m, _)| (m.curve.start()[0] + m.curve.end()[0]) / 2. > 11.)
        .collect();
    let switches = islands.windows(2).filter(|w| w[0] != w[1]).count();
    assert_eq!(switches, report.stacks - 1, "cutting moves change island {switches} times");
    for (motion, cut) in &moves {
        if motion.kind == MoveKind::Plunge && !travels(motion) {
            assert!(*cut <= tolerance + 1e-6, "a vertical plunge cut {cut} mm of stock");
        }
    }
}
#[test]
fn levels_link_with_a_ramp_that_the_next_level_recuts() {
    use camlib::{motion::MoveKind, waterline::WaterlinePlan};
    // One box island (top Z6.3) on roughed stock (to Z6.3 within 1 mm of
    // it), levels 6, 4 and 2.5 (closed loops): each level is reached from
    // the one above without a retract - a ramp down along the lower loop
    // from the upper level, then that loop cut in full over the ramp (#46).
    // Undeclared, the ramp is 3 degrees; with a declared angle its length is
    // the level gap over tan(angle) (at most one lap either way).
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let angle = 10f64.to_radians();
    for entry in [None, Some(serde_json::json!({"kind": "ramp", "angle_rad": angle, "feed_mm_min": 79.}))] {
        let mut op = operation(&m);
        // Every level (6, 4, 2.5) lies beside the box's walls, none on its
        // top face.
        op.surface = mesh(box_top(6.3));
        op.entry = entry.clone().map(|e| serde_json::from_value(e).unwrap());
        let expected_entry_feed = if entry.is_some() { 79. } else { op.plunge_mm_min };
        let error_mm = op.contour_error_mm;
        let (mut stock, spec) = roughed(&m, 1.);
        let mut plan =
            WaterlinePlan::prepare(op, &m.tools[0], &spec, m.initial_position_mm, 1, &cancel)
                .unwrap();
        let (moves, error) = trace(&m, &mut plan, &mut stock);
        assert!(error.is_none(), "{error:?}");
        assert_eq!(plan.report().ramp_links, 2, "{entry:?}");
        let motions: Vec<&camlib::motion::Motion> = moves.iter().map(|(m, _)| m).collect();
        let mut links = 0;
        let mut i = 0;
        while i < motions.len() {
            // A ramp: a run of travelling plunges; linked when it follows a
            // cut or a link lead at the level above (not an entry plunge).
            if !(motions[i].kind == MoveKind::Plunge && travels(motions[i])) {
                i += 1;
                continue;
            }
            let start = i;
            while i < motions.len() && motions[i].kind == MoveKind::Plunge && travels(motions[i]) {
                assert_eq!(motions[i].feed_mm_min, Some(expected_entry_feed));
                i += 1;
            }
            let before = motions[start - 1];
            if !matches!(before.kind, MoveKind::Cut | MoveKind::Lead) {
                continue;
            }
            links += 1;
            let (za, zb) = (motions[start].curve.start()[2], motions[i - 1].curve.end()[2]);
            assert!(za - zb > 1., "ramp from {za} to {zb}");
            assert_eq!(before.curve.end()[2], za, "the link stays at the upper level");
            // Monotonic descent along the lower loop.
            let mut length = 0.;
            for r in &motions[start..i] {
                let (a, b) = (r.curve.start(), r.curve.end());
                assert!(b[2] <= a[2] + 1e-12, "the ramp climbs: {a:?} -> {b:?}");
                length += (b[0] - a[0]).hypot(b[1] - a[1]);
            }
            // The flat lap at the lower level that follows.
            let lap: Vec<&camlib::motion::Motion> = motions[i..]
                .iter()
                .take_while(|m| m.kind == MoveKind::Cut)
                .copied()
                .collect();
            assert!(lap.iter().all(|m| m.curve.start()[2] == zb && m.curve.end()[2] == zb));
            let lap_length: f64 = lap
                .iter()
                .map(|m| {
                    let (a, b) = (m.curve.start(), m.curve.end());
                    (b[0] - a[0]).hypot(b[1] - a[1])
                })
                .sum();
            // The flat lap passes over every point of the ramp (overlap).
            let distance = |p: [f64; 3]| {
                lap.iter()
                    .map(|m| {
                        let (a, b) = (m.curve.start(), m.curve.end());
                        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
                        let l2 = dx * dx + dy * dy;
                        let t = if l2 > 0. {
                            (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2).clamp(0., 1.)
                        } else {
                            0.
                        };
                        (p[0] - a[0] - t * dx).hypot(p[1] - a[1] - t * dy)
                    })
                    .fold(f64::INFINITY, f64::min)
            };
            for r in &motions[start..i] {
                for p in [r.curve.start(), r.curve.end()] {
                    assert!(
                        distance(p) <= error_mm + 1e-6,
                        "ramp point {p:?} is {} from the flat lap",
                        distance(p)
                    );
                }
            }
            // Without an entry the ramp is 3 degrees (#46), at most a lap.
            let expected = match &entry {
                None => ((za - zb) / 3f64.to_radians().tan()).min(lap_length),
                Some(_) => (za - zb) / angle.tan(),
            };
            assert!(
                (length - expected).abs() <= 1e-6,
                "ramp {length} mm, expected {expected} ({entry:?})"
            );
            // No retract between the levels.
            assert!(
                !motions[start..i]
                    .iter()
                    .any(|m| matches!(m.kind, MoveKind::Rapid | MoveKind::Retract))
            );
        }
        assert_eq!(links, 2, "{entry:?}");
    }
}
#[test]
fn blocked_reorder_falls_back_to_level_order() {
    use camlib::{
        motion::WorkFrame,
        stock::{HeightBounds, InitialStock, Stock},
        waterline::WaterlinePlan,
    };
    // Islands A (near the start) and B, both top Z6.5, levels 6, 4, 2.5,
    // with a ridge of uncut stock to Z9 between them, beside B's straight
    // west wall, that only B's own levels cut away. The cutter's flutes are
    // 4.5 mm long - enough for each island's own walls in any order - but
    // finishing A first, its shank (from 4.5 mm above the tip) would meet
    // the ridge at A's level 4, so A's stack falls back to level order and
    // is finished after B has cut the ridge down (#46). Every move passes
    // the reach check against the live stock.
    let mut m = fixture();
    m.tools[0].cutter.cutting_length_mm = 4.5;
    let cancel = AtomicBool::new(false);
    let mut op = operation(&m);
    op.surface = boxes(&[[2., 5., 6., 11., 6.5], [11., 4., 15., 12., 6.5]]);
    op.top_z_mm = 6.;
    let spec = m.stock.height().unwrap().clone();
    let probe = Stock::new(spec.clone()).unwrap();
    let [nx, ny] = probe.dimensions();
    let [wx, wy] = probe.cell_widths_mm();
    let mut bounds = Vec::with_capacity(nx * ny);
    for j in 0..ny {
        for i in 0..nx {
            let (x0, y0) = (i as f64 * wx, j as f64 * wy);
            let over = |a: f64, b: f64, c: f64, d: f64| x0 < b && x0 + wx > a && y0 < d && y0 + wy > c;
            let top = if over(11., 15., 4., 12.) {
                6.5
            } else if over(8.5, 10., 6., 10.) {
                9.
            } else if over(2., 6., 5., 11.) {
                6.5
            } else {
                2.
            };
            bounds.push(HeightBounds {
                lower_mm: top,
                upper_mm: top,
            });
        }
    }
    let initial = InitialStock {
        source_id: "ridge".into(),
        revision: "1".into(),
        source_sha256: "0".repeat(64),
        source_to_setup: WorkFrame::identity(),
        uncertainty_mm: 0.,
        representation: "whole_cell_top_down".into(),
        bounds,
    };
    let mut stock = Stock::from_initial(spec.clone(), &initial).unwrap();
    // The stream replays from the same blank (its own reach check is a
    // second, independent proof of every move).
    m.initial_stock = Some(initial.clone());
    let mut plan = WaterlinePlan::prepare_with_stock(
        op,
        &m.tools[0],
        &stock,
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    let (_, error) = trace(&m, &mut plan, &mut stock);
    assert!(error.is_none(), "{error:?}");
    let report = plan.report();
    assert!(report.reach_proof.is_some(), "the reach check runs");
    assert_eq!(report.stacks, 2);
    assert!(report.stacks_level_ordered >= 1);
}
#[test]
fn entries_ramp_instead_of_a_side_lead_into_stock() {
    use camlib::{motion::MoveKind, waterline::WaterlinePlan};
    // The plate in a roughed band of stock to Z6.3 within 3 mm of it: a
    // point beside the wall clear of the band exists, but the lead from it
    // onto the wall at a level below Z6.3 would run into the band - the tool
    // coming in from the side into a full cut. The loops enter on a ramp
    // along themselves instead (#46): no horizontal lead cuts stock deeper
    // than the tolerance, measured by the sweep on the live stock.
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let tolerance = plate(&m).tolerance_mm;
    let (mut stock, spec) = roughed(&m, 3.);
    let mut plan =
        WaterlinePlan::prepare(plate(&m), &m.tools[0], &spec, m.initial_position_mm, 1, &cancel)
            .unwrap();
    let (moves, error) = trace(&m, &mut plan, &mut stock);
    assert!(error.is_none(), "{error:?}");
    // Entry leads only: a lead right after a vertical plunge. (A link lead
    // between two levels of an island is a ramp link, bounded by the level
    // gap like the ramp after it.)
    let worst = moves
        .windows(2)
        .filter(|w| {
            let (before, lead) = (&w[0].0, &w[1].0);
            before.kind == MoveKind::Plunge
                && !travels(before)
                && lead.kind == MoveKind::Lead
                && lead.curve.start()[2] == lead.curve.end()[2]
                && travels(lead)
        })
        .map(|w| w[1].1)
        .fold(0., f64::max);
    assert!(worst <= tolerance + 1e-6, "a side lead cut {worst} mm of stock");
    assert!(plan.report().helical_entries >= 1);
    for (motion, cut) in &moves {
        if motion.kind == MoveKind::Plunge && !travels(motion) {
            assert!(*cut <= tolerance + 1e-6, "a vertical plunge cut {cut} mm of stock");
        }
    }
}
#[test]
fn declared_ramps_stream_on_a_curved_part() {
    use camlib::{stock::Stock, waterline::WaterlinePlan};
    // The faceted cap with a declared 5 degree ramp (#46): a ramp ends
    // part-way along a loop, sometimes within rounding of a vertex, and the
    // loops of neighbouring levels can meet within rounding where the link
    // would be. Every motion must still stream - no sliver shorter than a
    // motion may be, every motion starting where the last one ended.
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let mut op = cap_operation(&m);
    op.entry = Some(
        serde_json::from_value(serde_json::json!({"kind": "ramp", "angle_rad": 5f64.to_radians()}))
            .unwrap(),
    );
    let spec = m.stock.height().unwrap().clone();
    let mut stock = Stock::new(spec.clone()).unwrap();
    let mut plan =
        WaterlinePlan::prepare(op, &m.tools[0], &spec, m.initial_position_mm, 1, &cancel).unwrap();
    let (moves, error) = trace(&m, &mut plan, &mut stock);
    assert!(error.is_none(), "{error:?}");
    assert!(plan.report().helical_entries + plan.report().ramp_links > 0);
    // Every line survives a post at three decimals: a sliver that rounds to
    // nothing is refused when the posted program is decoded.
    let round = |p: [f64; 3]| p.map(|v| (v * 1000.).round() as i64);
    for (motion, _) in &moves {
        if let camlib::motion::Curve::Line { start, end } = motion.curve {
            assert_ne!(round(start), round(end), "{:?} {start:?} -> {end:?} rounds to nothing", motion.kind);
        }
    }
}
#[test]
fn links_meet_no_more_than_the_lap_below_cuts() {
    use camlib::{
        motion::{MoveKind, WorkFrame},
        stock::{HeightBounds, InitialStock, Stock},
        waterline::WaterlinePlan,
    };
    // The faceted cap finished from a terraced roughed blank (1.5 mm steps,
    // 0.3 mm allowance: more stock over the cap than the in-between levels'
    // 0.125 to 0.5 mm gaps): levels are linked from the one above (#46), and
    // no link meets more stock than the level gap it ramps down or than the
    // lap it leads to cuts - never a deeper cut from the side than the
    // finishing pass itself takes. Measured by the sweeps on the live stock.
    let mut m = fixture();
    let cancel = AtomicBool::new(false);
    let op = cap_operation(&m);
    let spec = m.stock.height().unwrap().clone();
    let probe = Stock::new(spec.clone()).unwrap();
    let [nx, ny] = probe.dimensions();
    let [wx, wy] = probe.cell_widths_mm();
    let cap = |x: f64, y: f64| {
        let r2 = (x - 12.).powi(2) + (y - 8.).powi(2);
        if r2 >= 144. { 1. } else { (-6. + (144. - r2).sqrt()).max(1.) }
    };
    let mut bounds = Vec::with_capacity(nx * ny);
    for j in 0..ny {
        for i in 0..nx {
            let (x0, y0) = (i as f64 * wx, j as f64 * wy);
            // Roughed in 1.5 mm steps leaving 0.3 mm: the cell stands at the
            // first roughing level at or above the cap's highest point over
            // it plus the allowance (terraces up to 1.8 mm over the cap).
            let (cx, cy) = ((12f64).clamp(x0, x0 + wx), (8f64).clamp(y0, y0 + wy));
            let top = (1. + ((cap(cx, cy) + 0.3 - 1.) / 1.5).ceil() * 1.5).min(10.);
            bounds.push(HeightBounds {
                lower_mm: top,
                upper_mm: top,
            });
        }
    }
    let initial = InitialStock {
        source_id: "cap-allowance".into(),
        revision: "1".into(),
        source_sha256: "0".repeat(64),
        source_to_setup: WorkFrame::identity(),
        uncertainty_mm: 0.,
        representation: "whole_cell_top_down".into(),
        bounds,
    };
    let mut stock = Stock::from_initial(spec.clone(), &initial).unwrap();
    m.initial_stock = Some(initial);
    let mut plan =
        WaterlinePlan::prepare(op, &m.tools[0], &spec, m.initial_position_mm, 1, &cancel).unwrap();
    // The plan on the live stock: every motion, the deepest any cell sank
    // under it, and for a link the deepest away from the rim of the ball
    // where the link starts (a whole cell straddling that rim cannot say
    // whether its material lay inside the ball's reach there).
    let tool = m.tools[0].prepare().unwrap();
    let (r, half) = (2., 0.4f64.hypot(0.4) / 2.);
    let mut moves: Vec<(camlib::motion::Motion, f64, f64)> = vec![];
    // For each link: the stock before it, and then what the lap's cut that
    // arrives back where the link arrived (the flat lap passes over the whole
    // ramp and ends there) would have met on that stock - the stock the lap
    // meets where the link arrives, had the link not cut it first.
    let mut before_link: Option<(camlib::stock::StockSnapshot, [f64; 2], usize)> = None;
    let mut lap_would_cut: Vec<(usize, f64)> = vec![];
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        let end = motion.curve.end();
        if motion.kind == MoveKind::Lead && moves.last().is_some_and(|(m, _, _)| m.kind == MoveKind::Cut) {
            before_link = Some((stock.snapshot(), [end[0], end[1]], moves.len()));
        } else if motion.kind == MoveKind::Cut
            && before_link
                .as_ref()
                .is_some_and(|(_, p, _)| (end[0] - p[0]).hypot(end[1] - p[1]) <= 1e-6)
        {
            let (snapshot, _, link) = before_link.take().unwrap();
            let earlier = Stock::from_snapshot(snapshot).unwrap();
            let would = earlier
                .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
                .unwrap()
                .changes()
                .iter()
                .map(|c| c.before.upper_mm - c.after.upper_mm)
                .fold(0., f64::max);
            lap_would_cut.push((link, would));
        } else if motion.kind == MoveKind::Rapid {
            before_link = None;
        }
        let d = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
            .unwrap();
        let a = motion.curve.start();
        let (mut cut, mut off_rim) = (0f64, 0f64);
        for c in d.changes() {
            let sank = c.before.upper_mm - c.after.upper_mm;
            cut = cut.max(sank);
            let center = stock.cell_center(c.index as usize).unwrap();
            let from_start = (center[0] - a[0]).hypot(center[1] - a[1]);
            if (from_start - r).abs() > half {
                off_rim = off_rim.max(sank);
            }
        }
        stock.apply_delta(&d).unwrap();
        moves.push((motion, cut, off_rim));
    }
    let motions: Vec<_> = moves.iter().map(|(m, _, _)| m.clone()).collect();
    let error = stream_error(&m, &motions);
    assert!(error.is_none(), "{error:?}");
    let report = plan.report();
    // Measured: 12 of the 13 level changes linked (one refused by the stock
    // it would meet); with links bounded by the level gap alone, most of the
    // in-between levels here would retract instead.
    assert!(report.ramp_links >= 12, "{} links", report.ramp_links);
    // Every link (a lead right after a cut, at the upper level) cuts no
    // deeper than the level gap it ramps down, or than the lap it leads to
    // would cut where the link arrives (on the stock before the link).
    let mut links = 0;
    for (i, (motion, _, off_rim)) in moves.iter().enumerate() {
        if motion.kind != MoveKind::Lead || i == 0 || moves[i - 1].0.kind != MoveKind::Cut {
            continue;
        }
        let first = (i + 1..moves.len())
            .find(|&k| moves[k].0.kind == MoveKind::Cut)
            .unwrap();
        let would = lap_would_cut
            .iter()
            .find(|(k, _)| *k == i)
            .map(|(_, w)| *w)
            .unwrap();
        let gap = motion.curve.end()[2] - moves[first].0.curve.end()[2];
        assert!(
            *off_rim <= gap.max(would) + 1e-6,
            "link {i} cut {off_rim} mm; its gap {gap} mm, its lap would cut {would} mm there"
        );
        links += 1;
    }
    assert!(links > 0, "no link was checked");
}

/// A gable roof (#46): ridge along Y at X12 Z7, eaves at X4 and X20 Z4
/// (20.6 degree planes), on finishing stock 0.1 mm over it, with the slope
/// limited to 25..90 degrees and levels Z6.8 down to Z4.4 every 0.4 mm. Sets
/// the manifest's initial stock to it.
fn roof(
    m: &mut camlib::stream::StreamManifest,
) -> (camlib::waterline::WaterlineOperation, camlib::stock::Stock, camlib::stock::StockSpec) {
    use camlib::stock::{HeightBounds, InitialStock, Stock};
    let roof = |x: f64| 7. - ((x - 12.).abs() * 3. / 8.);
    let (a, b, c, d, e, f) = (
        [4., 4., 4.],
        [12., 4., 7.],
        [20., 4., 4.],
        [4., 12., 4.],
        [12., 12., 7.],
        [20., 12., 4.],
    );
    let mut op = operation(m);
    op.surface = mesh(vec![[a, b, e], [a, e, d], [b, c, f], [b, f, e]]);
    op.top_z_mm = 6.8;
    op.bottom_z_mm = 4.4;
    op.stepdown_mm = 0.4;
    op.slope_from_deg = 25.;
    let spec = m.stock.height().unwrap().clone();
    let probe = Stock::new(spec.clone()).unwrap();
    let [nx, ny] = probe.dimensions();
    let [wx, wy] = probe.cell_widths_mm();
    let mut bounds = Vec::with_capacity(nx * ny);
    for j in 0..ny {
        for i in 0..nx {
            let (x0, y0) = (i as f64 * wx, j as f64 * wy);
            let inside = x0 + wx > 4. && x0 < 20. && y0 + wy > 4. && y0 < 12.;
            let top = if inside {
                roof(x0.clamp(4., 20.)).max(roof((x0 + wx).clamp(4., 20.))) + 0.1
            } else {
                0.
            };
            bounds.push(HeightBounds {
                lower_mm: top,
                upper_mm: top,
            });
        }
    }
    let initial = InitialStock {
        source_id: "roof".into(),
        revision: "1".into(),
        source_sha256: "0".repeat(64),
        source_to_setup: camlib::motion::WorkFrame::identity(),
        uncertainty_mm: 0.,
        representation: "whole_cell_top_down".into(),
        bounds,
    };
    let stock = Stock::from_initial(spec.clone(), &initial).unwrap();
    m.initial_stock = Some(initial);
    (op, stock, spec)
}
#[test]
fn slope_limited_walls_are_cut_wall_by_wall_and_linked() {
    use camlib::waterline::WaterlinePlan;
    // A gable roof, ridge along Y at X12 Z7, eaves at X4 and X20 Z4 (20.6
    // degree planes), on finishing stock 0.1 mm over it. With the slope
    // limited to 25..90 degrees, each level keeps only the runs where the
    // ball rides the gable ends - two walls of one contour. Each wall is cut
    // top to bottom and its runs linked (#46), instead of a retract and an
    // entry for every run.
    let mut m = fixture();
    let cancel = AtomicBool::new(false);
    let (op, mut stock, spec) = roof(&mut m);
    let tolerance = op.tolerance_mm;
    let mut plan =
        WaterlinePlan::prepare(op, &m.tools[0], &spec, m.initial_position_mm, 1, &cancel).unwrap();
    let replay = replay(&m, &mut plan, &mut stock);
    assert!(replay.stream_error.is_none(), "{:?}", replay.stream_error);
    let report = plan.report();
    // Every vertical plunge stops in air.
    assert!(replay.worst_vertical <= tolerance + 1e-6, "{}", replay.worst_vertical);
    let pieces = report.loops_completed;
    assert!(pieces >= 10, "{pieces} runs");
    // The two walls still cut as linked stacks. Six links the centre-only
    // estimate used to allow need a new entry with whole-cell stock bounds.
    // Pin both outcomes so dropping the stock check or disabling linking is
    // observable, even when the surface verification accepts both paths.
    assert_eq!(report.ramp_links + replay.vertical_plunges, pieces);
    assert_eq!((pieces, report.ramp_links, replay.vertical_plunges), (30, 18, 12));
    assert!(report.links_refused_stock >= 6, "{} stock refusals", report.links_refused_stock);
}

#[test]
fn links_stay_out_of_avoid_areas_between_runs() {
    use camlib::limits::{AvoidArea, Limits};
    use camlib::planar::{Boundary, Component, PlanarRegion};
    use camlib::{motion::MoveKind, waterline::WaterlinePlan};
    // The plate's contours at Z5, Z4 and Z3 run along Y3 south of it; a small
    // avoid area on that line (X11..13, Y2.5..3.5, to Z7) cuts each contour
    // into one open run whose ends face each other across it. Beside the
    // plate there is no stock, so a straight link from one level's end to
    // the next level's start meets none and keeps its part clearance - and
    // would run through the avoid area. Every link and ramp is checked
    // against the limits too (#46): no move but a rapid comes within the
    // cutter radius of the area below its top.
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let mut op = plate(&m);
    op.stepdown_mm = 1.;
    op.top_z_mm = 5.;
    op.bottom_z_mm = 3.;
    let area = [11., 2.5, 13., 3.5];
    op.limits = Some(Limits {
        contain: None,
        avoid: vec![AvoidArea {
            region: PlanarRegion {
                version: 1,
                units: "mm".into(),
                name: "avoid".into(),
                boundary_error_mm: 0.,
                source: serde_json::Value::Null,
                components: vec![Component {
                    outer: Boundary {
                        vertices: vec![
                            [area[0], area[1], 0.],
                            [area[2], area[1], 0.],
                            [area[2], area[3], 0.],
                            [area[0], area[3], 0.],
                        ],
                    },
                    holes: vec![],
                }],
            },
            top_z_mm: 7.,
        }],
    });
    let (mut stock, spec) = roughed(&m, 0.);
    let mut plan =
        WaterlinePlan::prepare(op, &m.tools[0], &spec, m.initial_position_mm, 1, &cancel).unwrap();
    let (moves, error) = trace(&m, &mut plan, &mut stock);
    assert!(error.is_none(), "{error:?}");
    assert!(plan.report().ramp_links > 0, "the levels are linked");
    let distance = |p: [f64; 3]| {
        let dx = (area[0] - p[0]).max(p[0] - area[2]).max(0.);
        let dy = (area[1] - p[1]).max(p[1] - area[3]).max(0.);
        dx.hypot(dy)
    };
    for (motion, _) in &moves {
        if motion.kind == MoveKind::Rapid {
            continue;
        }
        let (a, b) = (motion.curve.start(), motion.curve.end());
        let steps = ((b[0] - a[0]).hypot(b[1] - a[1]) / 0.05).ceil().max(1.) as usize;
        for i in 0..=steps {
            let t = i as f64 / steps as f64;
            let p = [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * (b[2] - a[2])];
            assert!(
                p[2] >= 7. || distance(p) >= 2. - 1e-6,
                "a {:?} move from {a:?} to {b:?} passes {p:?}, {} mm from the avoid area",
                motion.kind,
                distance(p)
            );
        }
    }
}

/// Every travelling descent (a ramp) is no steeper than the ramp angle, and
/// every coordinate is finite.
fn assert_ramps_within(moves: &[(camlib::motion::Motion, f64)], angle_deg: f64) {
    use camlib::motion::MoveKind;
    let limit = angle_deg.to_radians().tan();
    let mut ramps = 0;
    for (motion, _) in moves {
        let (a, b) = (motion.curve.start(), motion.curve.end());
        assert!(
            a.iter().chain(b.iter()).all(|v| v.is_finite()),
            "a {:?} move from {a:?} to {b:?}",
            motion.kind
        );
        let run = (b[0] - a[0]).hypot(b[1] - a[1]);
        if motion.kind == MoveKind::Plunge && run > 1e-9 {
            ramps += 1;
            assert!(
                (a[2] - b[2]).abs() <= limit * run + 1e-9,
                "a ramp from {a:?} to {b:?} descends {} over {run} mm: {} degrees",
                a[2] - b[2],
                ((a[2] - b[2]).abs() / run).atan().to_degrees()
            );
        }
    }
    assert!(ramps > 0, "no ramp");
}
#[test]
fn ramps_on_short_runs_keep_the_ramp_angle() {
    use camlib::waterline::WaterlinePlan;
    // The roof's runs at the in-between levels are about 2 mm long and the
    // levels 0.4 mm apart: one out-and-back over a 2 mm run descends at
    // about 5.7 degrees. The ramps take as many passes as the 3 degree
    // default needs (#46).
    let mut m = fixture();
    let cancel = AtomicBool::new(false);
    let (op, mut stock, spec) = roof(&mut m);
    let mut plan =
        WaterlinePlan::prepare(op, &m.tools[0], &spec, m.initial_position_mm, 1, &cancel).unwrap();
    let (moves, error) = trace(&m, &mut plan, &mut stock);
    assert!(error.is_none(), "{error:?}");
    assert_ramps_within(&moves, 3.);
}
/// The XY length of every leg of every ramp that turns back (a run of
/// travelling plunge moves, split where the direction reverses):
/// independently of the plan, the span each reversal is made on.
fn reversed_ramp_legs(moves: &[(camlib::motion::Motion, f64)]) -> Vec<f64> {
    use camlib::motion::MoveKind;
    let mut out = vec![];
    let mut legs: Vec<f64> = vec![];
    let mut last: Option<([f64; 2], [f64; 3])> = None;
    let mut close = |legs: &mut Vec<f64>| {
        if legs.len() > 1 {
            out.extend(legs.iter().copied());
        }
        legs.clear();
    };
    for (motion, _) in moves {
        let (a, b) = (motion.curve.start(), motion.curve.end());
        let run = (b[0] - a[0]).hypot(b[1] - a[1]);
        if motion.kind != MoveKind::Plunge || run <= 1e-9 {
            close(&mut legs);
            last = None;
            continue;
        }
        let unit = [(b[0] - a[0]) / run, (b[1] - a[1]) / run];
        match last {
            Some((u, end)) if end == a && u[0] * unit[0] + u[1] * unit[1] >= -0.5 => {
                *legs.last_mut().unwrap() += run;
            }
            Some((_, end)) if end == a => legs.push(run),
            _ => {
                close(&mut legs);
                legs.push(run);
            }
        }
        last = Some((unit, b));
    }
    close(&mut legs);
    out
}
#[test]
fn open_runs_shorter_than_min_length_are_entered_after_a_retract() {
    use camlib::pocket::{PocketEntry, RampEntry};
    use camlib::motion::MoveKind;
    use camlib::waterline::WaterlinePlan;
    // #392: the roof's slope-limited runs at the in-between levels are
    // about 2 mm long, shorter than the 3 mm the tool is declared to
    // reverse a ramp on. A link down onto such a run (along it and back)
    // does not exist, so the run is entered after a retract; there the
    // stock beside it stands above the level, so the entry ramps out along
    // the level's whole contour and back. The whole operation was refused
    // ("open ramp span ... shorter than min_length_mm").
    let mut m = fixture();
    let cancel = AtomicBool::new(false);
    let (mut op, mut stock, spec) = roof(&mut m);
    let tolerance = op.tolerance_mm;
    let minimum = 3.;
    op.entry = Some(PocketEntry::Ramp(RampEntry {
        angle_rad: 3f64.to_radians(),
        min_length_mm: Some(minimum),
        feed_mm_min: None,
    }));
    let mut plan =
        WaterlinePlan::prepare(op, &m.tools[0], &spec, m.initial_position_mm, 1, &cancel).unwrap();
    let (moves, error) = trace(&m, &mut plan, &mut stock);
    assert!(error.is_none(), "{error:?}");
    let report = plan.report();
    assert!(report.links_refused_short_run > 0);
    assert!(report.contour_ramp_entries > 0);
    // Every check still holds on what was emitted: no ramp steeper than the
    // angle, every reversal on at least the declared span, and every
    // vertical plunge in air (it removes no stock).
    assert_ramps_within(&moves, 3.);
    let legs = reversed_ramp_legs(&moves);
    assert!(!legs.is_empty());
    for leg in legs {
        assert!(leg >= minimum - 1e-6, "a ramp reverses after {leg} mm");
    }
    for (motion, cut) in &moves {
        let (a, b) = (motion.curve.start(), motion.curve.end());
        if motion.kind == MoveKind::Plunge && (b[0] - a[0]).hypot(b[1] - a[1]) <= 1e-9 {
            assert!(*cut <= tolerance + 1e-9, "a plunge at {a:?} cuts {cut} mm");
        }
    }
    // Every slope-limited run is still cut.
    assert_eq!(report.loops_completed, 30);
}
#[test]
fn a_tiny_ramp_descent_streams_finite_moves() {
    use camlib::stock::{HeightBounds, InitialStock, Stock};
    use camlib::waterline::WaterlinePlan;
    // Finishing stock stands 0.0002 mm above the first level (Z2.5) all
    // round the plate and high beyond, so the loop is entered on a ramp
    // from Z2.5002: at 3
    // degrees 0.0038 mm long, which used to snap to nothing and divide by
    // zero (a NaN start). Every move stays finite, streams, and keeps the
    // angle (#46).
    let mut m = fixture();
    let cancel = AtomicBool::new(false);
    let mut op = plate(&m);
    op.stepdown_mm = 1.;
    op.top_z_mm = 2.5;
    op.bottom_z_mm = 2.4;
    // Within 4.4 mm of the plate the stock stands at Z2.5002 (the ball
    // there, 2 mm out, stays on it); beyond, at Z6, so no point beside the
    // loop offers a side entry.
    let spec = m.stock.height().unwrap().clone();
    let probe = Stock::new(spec.clone()).unwrap();
    let [nx, ny] = probe.dimensions();
    let [wx, wy] = probe.cell_widths_mm();
    let mut bounds = Vec::with_capacity(nx * ny);
    for j in 0..ny {
        for i in 0..nx {
            let (cx, cy) = ((i as f64 + 0.5) * wx, (j as f64 + 0.5) * wy);
            let d = (8. - cx).max(cx - 16.).max(0.).hypot((5. - cy).max(cy - 11.).max(0.));
            let top = if d <= 4.4 { 2.5002 } else { 6. };
            bounds.push(HeightBounds {
                lower_mm: top,
                upper_mm: top,
            });
        }
    }
    let initial = InitialStock {
        source_id: "skin".into(),
        revision: "1".into(),
        source_sha256: "0".repeat(64),
        source_to_setup: camlib::motion::WorkFrame::identity(),
        uncertainty_mm: 0.,
        representation: "whole_cell_top_down".into(),
        bounds,
    };
    let mut stock = Stock::from_initial(spec.clone(), &initial).unwrap();
    m.initial_stock = Some(initial);
    let mut plan =
        WaterlinePlan::prepare(op, &m.tools[0], &spec, m.initial_position_mm, 1, &cancel).unwrap();
    let (moves, error) = trace(&m, &mut plan, &mut stock);
    assert!(error.is_none(), "{error:?}");
    assert!(plan.report().helical_entries >= 1, "entered on a ramp");
    assert_ramps_within(&moves, 3.);
}
