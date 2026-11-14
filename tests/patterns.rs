//! Spiral, radial and project surface finishing on the hemisphere fixture,
//! checked against the mesh independently of the planner's own proofs.
use camlib::{
    finishing::{FinishPattern, ParallelOperation, ParallelPlan},
    motion::{Motion, MoveKind},
    pocket::Operation,
    stock::Stock,
    stream::StreamManifest,
    surface::segment_triangle_sq,
    trace::{TracePath, TraceSegment},
};
use std::{f64::consts::PI, sync::atomic::AtomicBool};

fn fixture() -> StreamManifest {
    let m: StreamManifest =
        serde_json::from_str(include_str!("../fixtures/operations/hemisphere.json")).unwrap();
    m.validate().unwrap();
    m
}
fn operation(m: &StreamManifest, pattern: FinishPattern) -> ParallelOperation {
    let mut op = match &m.operations[0] {
        Operation::Parallel(p) => p.clone(),
        _ => panic!("fixture"),
    };
    if !matches!(
        pattern,
        FinishPattern::Raster | FinishPattern::Spiral { .. }
    ) {
        op.stepover_mm = 0.;
    }
    op.pattern = pattern;
    // Passes entered inside the part under stock are ramped in by a
    // declared entry (a plunge there would cut into the stock).
    op.entry = Some(
        serde_json::from_value(serde_json::json!({
            "kind": "ramp",
            "angle_rad": 10f64.to_radians(),
            "min_length_mm": 1.0
        }))
        .unwrap(),
    );
    op
}

/// The fixture's dome roughed to 0.1 mm over its surface (and its floor at
/// Z2): the stock a finishing pattern meets after roughing. Each cell is
/// certain up to the lowest point of the blank over it and may hold material
/// up to the highest.
fn roughed_stock(m: &StreamManifest) -> Stock {
    let mut spec = m.stock.height().unwrap().clone();
    // Finer cells than the fresh-stock fixture: the dome's edge is steep.
    spec.cell_size_mm = 0.05;
    let blank = Stock::new(spec.clone()).unwrap();
    let [nx, ny] = blank.dimensions();
    let [sx, sy] = blank.cell_widths_mm();
    // Dome: 2 + sqrt(16 - r^2) about (8, 8), r <= 4; the blank adds 0.1.
    let height = |x: f64, y: f64| {
        let r2 = (x - 8.).powi(2) + (y - 8.).powi(2);
        2.1 + if r2 < 16. { (16. - r2).sqrt() } else { 0. }
    };
    let mut bounds = Vec::with_capacity(nx * ny);
    for j in 0..ny {
        for i in 0..nx {
            let (x0, y0) = (spec.min_mm[0] + i as f64 * sx, spec.min_mm[1] + j as f64 * sy);
            let mut lo = f64::MAX;
            let mut hi = f64::MIN;
            // The dome is monotone in the distance to its axis: its extremes
            // over a cell lie at the cell's nearest and farthest points.
            let near = (8f64.clamp(x0, x0 + sx), 8f64.clamp(y0, y0 + sy));
            let far = (
                if (8. - x0).abs() > (8. - x0 - sx).abs() { x0 } else { x0 + sx },
                if (8. - y0).abs() > (8. - y0 - sy).abs() { y0 } else { y0 + sy },
            );
            for (x, y) in [near, far] {
                let h = height(x, y);
                lo = lo.min(h);
                hi = hi.max(h);
            }
            bounds.push(camlib::stock::HeightBounds { lower_mm: lo, upper_mm: hi });
        }
    }
    let initial = camlib::stock::InitialStock {
        source_id: "roughed-dome".into(),
        revision: "1".into(),
        source_sha256: "0".repeat(64),
        source_to_setup: camlib::motion::WorkFrame::identity(),
        uncertainty_mm: 0.,
        representation: "whole_cell_top_down".into(),
        bounds,
    };
    Stock::from_initial(spec, &initial).unwrap()
}

/// Run the plan with stock and check, independently, that every feed move
/// keeps the ball (radius 2) clear of every triangle and above the floor.
fn run(m: &StreamManifest, op: ParallelOperation) -> (Vec<Motion>, ParallelPlan, Stock) {
    let cancel = AtomicBool::new(false);
    let floor = op.floor_z_mm;
    let triangles = op.surface.triangles.clone();
    let mut stock = roughed_stock(m);
    let mut plan = ParallelPlan::prepare(
        op,
        &m.tools[0],
        stock.spec(),
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    let tool = m.tools[0].prepare().unwrap();
    let mut position = m.initial_position_mm;
    let mut motions = vec![];
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        assert_eq!(motion.curve.start(), position, "continuous motion");
        position = motion.curve.end();
        if motion.kind == MoveKind::Rapid {
            assert!(
                motion.curve.start()[2] >= m.stock.height().unwrap().max_mm[2]
                    && motion.curve.end()[2] >= m.stock.height().unwrap().max_mm[2],
                "rapid below the stock top: {:?}",
                motion.curve
            );
        } else {
            let lift = |p: [f64; 3]| [p[0], p[1], p[2] + 2.];
            let (a, b) = (lift(motion.curve.start()), lift(motion.curve.end()));
            assert!(a[2].min(b[2]) >= floor + 2. - 1e-9, "below the floor");
            for t in triangles.iter() {
                let d = segment_triangle_sq(a, b, *t).sqrt();
                assert!(d >= 2. - 1e-7, "gouge {d} on {:?}", motion.curve);
            }
        }
        let d = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
            .unwrap();
        stock.apply_delta(&d).unwrap();
        motions.push(motion);
    }
    plan.finish_report(&cancel).unwrap();
    assert_eq!(position[2], 10., "ends at clearance");
    (motions, plan, stock)
}

#[test]
fn spiral_covers_the_dome_without_gouging() {
    let m = fixture();
    let op = operation(
        &m,
        FinishPattern::Spiral {
            center_mm: [8., 8.],
            inner_radius_mm: 0.,
            outer_radius_mm: None,
            outward: false,
            clockwise: false,
        },
    );
    let (motions, plan, stock) = run(&m, op);
    let report = plan.report();
    assert_eq!(report.strategy, "spiral");
    assert_eq!(report.rows_completed, report.rows);
    assert!(report.max_certified_path_excess_mm.unwrap() <= 0.005);
    // The 0.1 mm blank over the 100 mm2 dome holds about 10 mm3 (measured 11.7).
    assert!(stock.volume().removed_lower_mm3 > 8.);
    // Inward: the last cut ends at the center.
    let last = motions
        .iter()
        .rev()
        .find(|m| m.kind == MoveKind::Cut)
        .unwrap()
        .curve
        .end();
    assert!((last[0] - 8.).abs() < 1e-9 && (last[1] - 8.).abs() < 1e-9);
    // Successive cuts stay in one pass between entries: few plunges.
    let plunges = motions
        .iter()
        .filter(|m| m.kind == MoveKind::Plunge)
        .count();
    assert_eq!(plunges, report.rows);
}

#[test]
fn radial_spokes_and_project_curves() {
    let m = fixture();
    let op = operation(
        &m,
        FinishPattern::Radial {
            center_mm: [8., 8.],
            angular_step_deg: 10.,
            inner_radius_mm: 0.5,
            outer_radius_mm: Some(9.),
            start_angle_deg: 0.,
            alternate: true,
        },
    );
    let (_, plan, stock) = run(&m, op);
    assert_eq!(plan.report().strategy, "radial");
    assert_eq!(plan.report().rows, 36);
    assert!(stock.volume().removed_lower_mm3 > 5.);

    // A circle of radius 5 projected onto the dome, cut as one closed pass.
    let op = operation(
        &m,
        FinishPattern::Project {
            paths: vec![TracePath {
                start_mm: [13., 8.],
                segments: vec![TraceSegment::Arc {
                    center_mm: [8., 8.],
                    sweep_rad: 2. * PI,
                }],
            }],
        },
    );
    let (motions, plan, _) = run(&m, op);
    assert_eq!(plan.report().strategy, "project");
    assert_eq!(plan.report().rows, 1);
    for motion in motions.iter().filter(|m| m.kind == MoveKind::Cut) {
        for p in [motion.curve.start(), motion.curve.end()] {
            let r = (p[0] - 8.).hypot(p[1] - 8.);
            assert!(
                (r - 5.).abs() < 0.005 + 1e-9,
                "off the projected circle: {r}"
            );
        }
    }
}

#[test]
fn pattern_parameters_fail_precisely() {
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let error = |op: ParallelOperation| {
        ParallelPlan::prepare(
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
    let radial = FinishPattern::Radial {
        center_mm: [8., 8.],
        angular_step_deg: 10.,
        inner_radius_mm: 0.,
        outer_radius_mm: None,
        start_angle_deg: 0.,
        alternate: true,
    };
    let mut op = operation(&m, radial.clone());
    op.stepover_mm = 0.5;
    assert!(error(op).contains("stepover_mm 0.5 is not used by the radial pattern"));
    let mut op = operation(
        &m,
        FinishPattern::Spiral {
            center_mm: [8., 8.],
            inner_radius_mm: 3.,
            outer_radius_mm: Some(2.),
            outward: true,
            clockwise: false,
        },
    );
    assert!(error(op.clone()).contains("outer_radius_mm 2 must be finite and greater"));
    op.reverse = true;
    assert!(error(op).contains("reverse applies to the raster pattern only"));
    let op = operation(
        &m,
        FinishPattern::Project {
            paths: vec![TracePath {
                start_mm: [1., 1.],
                segments: vec![TraceSegment::Line { end_mm: [20., 1.] }],
            }],
        },
    );
    assert!(error(op).contains("path 0 point [20.0, 1.0] is outside the stock XY"));
    let FinishPattern::Radial { .. } = radial else {
        unreachable!()
    };
    let op = operation(
        &m,
        FinishPattern::Radial {
            center_mm: [8., 8.],
            angular_step_deg: 0.,
            inner_radius_mm: 0.,
            outer_radius_mm: None,
            start_angle_deg: 0.,
            alternate: true,
        },
    );
    assert!(error(op).contains("angular_step_deg 0 must be within (0, 180]"));
}

#[test]
fn slope_limits_split_shallow_and_steep_areas_of_the_dome() {
    // Dome of radius 4 about (8, 8, 2), ball radius 2: the ball center lies
    // on the contact normal, at XY distance 6 sin(angle) from the axis.
    let m = fixture();
    let distance = |p: [f64; 3]| (p[0] - 8.).hypot(p[1] - 8.);
    let mut shallow = operation(&m, FinishPattern::Raster);
    shallow.slope_to_deg = 30.;
    let (motions, plan, _) = run(&m, shallow);
    assert!(plan.report().rows > 0);
    let cuts: Vec<_> = motions.iter().filter(|m| m.kind == MoveKind::Cut).collect();
    assert!(!cuts.is_empty());
    for cut in &cuts {
        for p in [cut.curve.start(), cut.curve.end()] {
            let d = distance(p);
            assert!(
                !(3.6..5.5).contains(&d),
                "shallow cut on the steep flank at {d}"
            );
        }
    }
    let mut steep = operation(&m, FinishPattern::Raster);
    steep.slope_from_deg = 30.;
    let (motions, _, _) = run(&m, steep);
    let cuts: Vec<_> = motions.iter().filter(|m| m.kind == MoveKind::Cut).collect();
    assert!(!cuts.is_empty());
    for cut in &cuts {
        for p in [cut.curve.start(), cut.curve.end()] {
            let d = distance(p);
            assert!((2.4..6.1).contains(&d), "steep cut off the flank at {d}");
        }
    }
    let mut bad = operation(&m, FinishPattern::Raster);
    bad.slope_from_deg = 50.;
    bad.slope_to_deg = 40.;
    let error = ParallelPlan::prepare(
        bad,
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &AtomicBool::new(false),
    )
    .err()
    .unwrap();
    assert!(
        error.contains("slope_from_deg 50 and slope_to_deg 40 must satisfy"),
        "{error}"
    );
}

#[test]
fn scallop_offsets_cover_the_dome_without_gouging() {
    let m = fixture();
    let mut op = operation(&m, FinishPattern::Scallop);
    op.stepover_mm = 0.5;
    let (motions, plan, stock) = run(&m, op);
    assert_eq!(plan.report().strategy, "scallop");
    assert!(plan.report().rows > 10, "{}", plan.report().rows);
    assert_eq!(plan.report().rows_completed, plan.report().rows);
    assert!(motions.iter().any(|m| m.kind == MoveKind::Cut));
    assert!(stock.volume().removed_lower_mm3 > 8.);
}

#[test]
fn rest_finishing_cuts_only_where_the_stock_still_stands_above_the_target() {
    // First finish only the shallow cap of the dome (contact angle up to
    // 30 degrees, ball centers within 3 mm of the axis); the flanks keep the
    // stock. Rest finishing then cuts where stock remains: the flanks.
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let tool = m.tools[0].prepare().unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut position = m.initial_position_mm;
    let drive = |plan: &mut ParallelPlan, stock: &mut Stock, position: &mut [f64; 3]| {
        let mut cuts = vec![];
        while let Some(motion) = plan.next_motion_with_stock(stock, &cancel).unwrap() {
            let d = stock
                .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
                .unwrap();
            stock.apply_delta(&d).unwrap();
            *position = motion.curve.end();
            if motion.kind == MoveKind::Cut {
                cuts.push(motion);
            }
        }
        plan.finish_report(&cancel).unwrap();
        cuts
    };
    let mut cap = operation(&m, FinishPattern::Raster);
    cap.slope_to_deg = 30.;
    let mut plan =
        ParallelPlan::prepare_with_stock(cap, &m.tools[0], &stock, position, 1, &cancel).unwrap();
    drive(&mut plan, &mut stock, &mut position);
    let mut rest = operation(&m, FinishPattern::Raster);
    rest.rest_threshold_mm = Some(0.05);
    let mut plan = ParallelPlan::prepare_with_stock(
        rest.clone(),
        &m.tools[0],
        &stock,
        position,
        100_000,
        &cancel,
    )
    .unwrap();
    let cuts = drive(&mut plan, &mut stock, &mut position);
    assert!(!cuts.is_empty());
    for cut in &cuts {
        for p in [cut.curve.start(), cut.curve.end()] {
            let d = (p[0] - 8.).hypot(p[1] - 8.);
            assert!(d > 2., "rest cut on the finished cap at {d}");
        }
    }
    // Without the stock there is nothing to compare with.
    let error = ParallelPlan::prepare(
        rest,
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &cancel,
    )
    .err()
    .unwrap();
    assert!(error.contains("rest finishing needs the stock"), "{error}");
}
