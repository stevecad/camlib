use camlib::{motion::*, stock::*, tool::*};
use std::{
    cell::Cell,
    f64::consts::{FRAC_PI_2, PI, TAU},
    sync::atomic::AtomicBool,
};

fn spec(cell: f64) -> StockSpec {
    StockSpec {
        version: 1,
        units: "mm".into(),
        min_mm: [0., 0., 0.],
        max_mm: [20., 20., 10.],
        cell_size_mm: cell,
        max_cells: 1_048_576,
    }
}

#[test]
fn declared_initial_blank_is_validated_and_volume_starts_at_that_blank() {
    let spec = spec(1.);
    let initial = InitialStock {
        source_id: "measured-blank".into(),
        revision: "1".into(),
        source_sha256: "a".repeat(64),
        source_to_setup: WorkFrame::identity(),
        uncertainty_mm: 0.,
        representation: "whole_cell_top_down".into(),
        bounds: (0..400)
            .map(|i| {
                let z = if i % 20 < 10 { 0. } else { 10. };
                HeightBounds {
                    lower_mm: z,
                    upper_mm: z,
                }
            })
            .collect(),
    };
    let mut stock = Stock::from_initial(spec.clone(), &initial).unwrap();
    assert_eq!(stock.volume().remaining_upper_mm3, 2000.);
    assert_eq!(stock.volume().removed_upper_mm3, 0.);
    let tool = tool(CutterShape::Flat);
    let empty = line([3., 5., 6.], [7., 5., 6.]);
    let delta = preview(&stock, &tool, &empty, 1.);
    assert!(delta.changes().is_empty());
    stock.apply_delta(&delta).unwrap();
    let cut = line([12., 5., 6.], [17., 5., 6.]);
    stock
        .apply_delta(&preview(&stock, &tool, &cut, 1.))
        .unwrap();
    assert!(stock.volume().removed_lower_mm3 > 0.);
    assert!(stock.volume().removed_upper_mm3 < 1000.);
    // A restored snapshot still has no native machining-history certificate.
    assert!(
        !Stock::from_snapshot(stock.snapshot())
            .unwrap()
            .departs_clear("t1", cut.curve.end())
    );
    for invalid in [
        InitialStock {
            source_sha256: "invalid".into(),
            ..initial.clone()
        },
        InitialStock {
            uncertainty_mm: -1.,
            ..initial.clone()
        },
        InitialStock {
            bounds: vec![],
            ..initial.clone()
        },
        InitialStock {
            source_to_setup: WorkFrame {
                x: [2., 0., 0.],
                ..WorkFrame::identity()
            },
            ..initial.clone()
        },
        InitialStock {
            representation: "mesh-with-undercuts".into(),
            ..initial.clone()
        },
    ] {
        assert!(Stock::from_initial(spec.clone(), &invalid).is_err());
    }
}

#[test]
fn engagement_policy_owns_its_prefix_and_never_raises_authored_feeds() {
    use camlib::feed_policy::{FeedBand, FeedPolicy, ToolLoadLimit, UnresolvedPolicy, apply};
    let first = line([5., 5., 6.], [10., 5., 6.]);
    let mut back = line([10., 5., 6.], [5., 5., 6.]);
    back.id = 2;
    back.feed_mm_min = Some(100.);
    let program = Program {
        version: 1,
        id: "feed-policy".into(),
        revision: "1".into(),
        units: "mm".into(),
        tolerance_mm: 1e-6,
        frame: WorkFrame::identity(),
        initial_position_mm: first.curve.start(),
        tools: vec![tool(CutterShape::Flat).definition().clone()],
        events: vec![
            Event::WorkOffset { number: 1 },
            Event::ToolChange {
                tool: "t1".into(),
                number: 1,
                length_register: 1,
                position_after_mm: first.curve.start(),
            },
            Event::Spindle {
                rpm: 12000.,
                direction: SpindleDirection::Cw,
            },
            Event::Motion { motion: first },
            Event::Motion { motion: back },
        ],
    };
    let policy = FeedPolicy {
        bands: vec![FeedBand {
            at_least_angle_deg: 90.,
            max_feed_mm_min: 300.,
        }],
        unresolved: UnresolvedPolicy::Refuse,
        options: Default::default(),
        air_feed_mm_min: None,
        tool_limits: vec![],
    };
    let run = |policy: &FeedPolicy| {
        apply(
            &program,
            spec(1.),
            None,
            SweepOptions::default(),
            policy,
            &|| false,
        )
    };
    let report = run(&policy).unwrap();
    assert_eq!(report.observations[0].stock_generation, 0);
    assert_eq!(report.observations[1].stock_generation, 1);
    assert_eq!(report.observations[0].requested_feed_mm_min, Some(300.));
    assert_eq!(report.observations[1].requested_feed_mm_min, Some(100.));
    assert!(
        report
            .observations
            .iter()
            .all(|r| r.requested_feed_mm_min <= r.original_feed_mm_min)
    );
    let preserved = run(&FeedPolicy {
        options: camlib::engagement::EngagementOptions {
            max_work: 0,
            max_nodes: 0,
            ..Default::default()
        },
        unresolved: UnresolvedPolicy::Preserve,
        ..policy.clone()
    })
    .unwrap();
    assert_eq!(preserved.observations[0].status, "unresolved");
    assert_eq!(preserved.observations[0].requested_feed_mm_min, Some(1000.));
    for (axial, radial, axis) in [(0.1, 4., "axial"), (5., 0.1, "radial")] {
        let limits = FeedPolicy {
            tool_limits: vec![ToolLoadLimit {
                tool: "t1".into(),
                max_axial_mm: axial,
                max_radial_mm: radial,
                max_cut_feed_mm_min: 300.,
            }],
            ..policy.clone()
        };
        let error = run(&limits).unwrap_err();
        assert!(error.contains("motion 1 (tool 't1')"));
        assert!(error.contains(&format!("{axis} load bound")));
        assert!(error.contains("mm exceeds approved maximum 0.1 mm"));
        assert!(error.contains("reducing feed alone does not reduce"));
        assert!(error.contains("approved_load_limit_missing_or_exceeded"));
        assert!(!error.contains("Some("));
        let preserved = run(&FeedPolicy {
            unresolved: UnresolvedPolicy::Preserve,
            ..limits
        })
        .unwrap();
        assert_eq!(
            preserved.observations[0].reason.as_deref(),
            Some("approved_load_limit_missing_or_exceeded")
        );
        assert_eq!(preserved.observations[0].requested_feed_mm_min, Some(1000.));
    }
    assert!(
        run(&FeedPolicy {
            options: camlib::engagement::EngagementOptions {
                max_work: 0,
                max_nodes: 0,
                ..Default::default()
            },
            ..policy
        })
        .is_err()
    );
}
fn tool(shape: CutterShape) -> PreparedAssembly {
    ToolAssembly {
        id: "t1".into(),
        cutter: Cutter {
            id: "cutter".into(),
            name: "test".into(),
            shape,
            diameter_mm: 4.,
            cutting_length_mm: 30.,
            overall_length_mm: 50.,
            shank_diameter_mm: 4.,
            flutes: 2,
            center_cutting: true,
            material: "carbide".into(),
            neck: None,
        },
        stickout_mm: 35.,
        gauge_length_mm: 60.,
        holders: vec![HolderSection {
            z_min_mm: 35.,
            z_max_mm: 60.,
            radius_min_mm: 4.,
            radius_max_mm: 4.,
        }],
    }
    .prepare()
    .unwrap()
}
fn motion(curve: Curve) -> Motion {
    Motion {
        id: 1,
        operation: "pocket".into(),
        tool: "t1".into(),
        kind: MoveKind::Cut,
        curve,
        feed_mm_min: Some(1000.),
    }
}
fn preview(stock: &Stock, tool: &PreparedAssembly, motion: &Motion, f: f64) -> StockDelta {
    stock
        .preview_sweep(
            tool,
            motion,
            f,
            SweepOptions::default(),
            &AtomicBool::new(false),
        )
        .unwrap()
}
fn line(a: Vec3, b: Vec3) -> Motion {
    motion(Curve::Line { start: a, end: b })
}
fn height_at(stock: &Stock, p: [f64; 2]) -> HeightBounds {
    let spec = stock.spec();
    let [dx, dy] = stock.cell_widths_mm();
    let [nx, ny] = stock.dimensions();
    let x = ((p[0] - spec.min_mm[0]) / dx)
        .floor()
        .max(0.)
        .min((nx - 1) as f64) as usize;
    let y = ((p[1] - spec.min_mm[1]) / dy)
        .floor()
        .max(0.)
        .min((ny - 1) as f64) as usize;
    stock.cells()[y * nx + x]
}
fn enclose_samples(stock: &Stock, oracle: impl Fn([f64; 2]) -> f64) {
    let [dx, dy] = stock.cell_widths_mm();
    for (index, h) in stock.cells().iter().enumerate() {
        let p = stock.cell_center(index).unwrap();
        for [u, v] in [[0., 0.], [-0.5, -0.5], [-0.5, 0.5], [0.5, -0.5], [0.5, 0.5]] {
            let p = [p[0] + dx * u, p[1] + dy * v];
            let truth = oracle(p).clamp(stock.spec().min_mm[2], stock.spec().max_mm[2]);
            assert!(
                h.lower_mm <= truth + 1e-8 && truth <= h.upper_mm + 1e-8,
                "{p:?}: {h:?} fails to enclose {truth}"
            );
        }
    }
}
// Independent circle/line geometry and profile formulas, not the stock helpers.
fn profile_height(shape: &CutterShape, d: f64) -> f64 {
    if d > 2. {
        return f64::INFINITY;
    }
    match *shape {
        CutterShape::Flat | CutterShape::Beam => 0.,
        CutterShape::Ball | CutterShape::Probe => 2. - (4. - d * d).max(0.).sqrt(),
        CutterShape::Bull {
            corner_radius_mm: c,
        } => c - (c * c - (d - 2. + c).max(0.).powi(2)).max(0.).sqrt(),
        CutterShape::Drill { point_angle_rad: a } => d / (a / 2.).tan(),
        CutterShape::Chamfer {
            included_angle_rad: a,
            tip_diameter_mm: td,
        } => (d - td / 2.).max(0.) / (a / 2.).tan(),
        CutterShape::Indexable { .. } => {
            unreachable!("annular profiles have their own independent stock oracle")
        }
        CutterShape::Lollipop | CutterShape::Dovetail { .. } => {
            unreachable!("the height field refuses narrowing cutters")
        }
    }
}
fn flat_line_height(p: [f64; 2], a: Vec3, b: Vec3, f: f64) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1]];
    let q = [a[0] - p[0], a[1] - p[1]];
    let aa = d[0] * d[0] + d[1] * d[1];
    let bb = 2. * (q[0] * d[0] + q[1] * d[1]);
    let cc = q[0] * q[0] + q[1] * q[1] - 4.;
    let (lo, hi) = if aa == 0. {
        if cc > 0. {
            return 10.;
        }
        (0., f)
    } else {
        let disc = bb * bb - 4. * aa * cc;
        if disc < 0. {
            return 10.;
        }
        (
            (-bb - disc.sqrt()) / (2. * aa),
            (-bb + disc.sqrt()) / (2. * aa),
        )
    };
    let (lo, hi) = (lo.max(0.), hi.min(f));
    if lo > hi {
        return 10.;
    }
    a[2] + (b[2] - a[2]) * if b[2] > a[2] { lo } else { hi }
}

#[test]
fn grid_covers_exact_stock_extents_and_enforces_budgets() {
    let stock = Stock::new(spec(0.3)).unwrap();
    assert_eq!(stock.dimensions(), [67, 67]);
    let [dx, dy] = stock.cell_widths_mm();
    assert_eq!(dx * 67., 20.);
    assert_eq!(dy * 67., 20.);
    assert_eq!(
        stock.volume().remaining_lower_mm3,
        stock.volume().remaining_upper_mm3
    );
    assert!((stock.volume().remaining_lower_mm3 - 4000.).abs() < 1e-8);
    let mut invalid = spec(0.001);
    assert!(Stock::new(invalid.clone()).is_err());
    invalid.cell_size_mm = f64::NAN;
    assert!(Stock::new(invalid).is_err());
    for (axis, value) in [(0, 0.), (1, -1.), (2, 0.)] {
        let mut invalid = spec(0.2);
        invalid.max_mm[axis] = value;
        assert!(Stock::new(invalid).is_err());
    }
}

#[test]
fn cutting_lower_envelopes_match_independent_inverse_profiles() {
    for shape in [
        CutterShape::Flat,
        CutterShape::Ball,
        CutterShape::Bull {
            corner_radius_mm: 0.5,
        },
        CutterShape::Drill {
            point_angle_rad: FRAC_PI_2,
        },
        CutterShape::Chamfer {
            included_angle_rad: FRAC_PI_2,
            tip_diameter_mm: 1.,
        },
    ] {
        let tool = tool(shape.clone());
        for n in 0..101 {
            let r = 2. * n as f64 / 100.;
            assert!(
                (tool.cutter().tip_height_at_radius(r).unwrap() - profile_height(&shape, r)).abs()
                    < 1e-12
            );
        }
        for r in [-1., 2.0001, f64::NAN, f64::INFINITY] {
            assert_eq!(tool.cutter().tip_height_at_radius(r), None);
        }
    }
}

#[test]
fn horizontal_capsule_sweep_encloses_whole_cells_and_analytic_volume() {
    for shape in [
        CutterShape::Flat,
        CutterShape::Ball,
        CutterShape::Bull {
            corner_radius_mm: 0.5,
        },
        CutterShape::Drill {
            point_angle_rad: FRAC_PI_2,
        },
        CutterShape::Chamfer {
            included_angle_rad: FRAC_PI_2,
            tip_diameter_mm: 1.,
        },
    ] {
        let mut stock = Stock::new(spec(0.2)).unwrap();
        let before = stock.snapshot();
        let m = line([4., 10., 6.], [16., 10., 6.]);
        let delta = preview(&stock, &tool(shape.clone()), &m, 1.);
        assert_eq!(stock.snapshot(), before); // Preview cannot partially mutate.
        assert!(delta.stats().visited_cells > 0);
        stock.apply_delta(&delta).unwrap();
        enclose_samples(&stock, |p| {
            let near_x = p[0].clamp(4., 16.);
            6. + profile_height(&shape, (p[0] - near_x).hypot(p[1] - 10.))
        });
        if shape == CutterShape::Flat {
            let exact = (4. * 12. + 4. * PI) * 4.;
            let volume = stock.volume();
            assert!(volume.removed_lower_mm3 <= exact && exact <= volume.removed_upper_mm3);
            assert!(volume.removed_upper_mm3 - volume.removed_lower_mm3 < 60.);
            assert!(height_at(&stock, [10., 10.]).upper_mm < 6.0001); // Endpoints alone miss this cut.
            assert_eq!(height_at(&stock, [10., 0.]).upper_mm, 10.);
        }
    }
}

#[test]
fn ball_sweep_cell_bounds_are_reproducible_and_enclose_the_spherical_cap() {
    let mut stock = Stock::new(spec(0.5)).unwrap();
    let end = [9.1064453125, 9.1015625, 0.];
    let sweep = line([8., end[1], 0.], end);
    let delta = preview(&stock, &tool(CutterShape::Ball), &sweep, 1.);
    stock.apply_delta(&delta).unwrap();
    let h = height_at(&stock, [10.25, 10.25]);

    // The whole square [10, 10.5]^2 lies beyond the endpoint. Its exact
    // lowest and highest cuts are therefore at its near and far corners.
    // Evaluate the sphere there independently of the stock kernel.
    let height = |xy: f64| {
        let r2 = (xy - end[0]).powi(2) + (xy - end[1]).powi(2);
        2. - (4. - r2).sqrt()
    };
    assert!(h.lower_mm <= height(10.));
    assert!(h.upper_mm >= height(10.5));

    // Reference for the cell-center enclosure: evaluate each norm at
    // 90 decimal digits and round to binary64 before applying the ball
    // profile and the default sweep guard. Host hypot on glibc 2.39
    // rounds the endpoint distance one ulp lower for this dyadic input.
    assert_eq!(h.lower_mm, 0.45260574837785056);
    assert_eq!(h.upper_mm, 1.6801103187291515);
}

#[test]
fn ball_sweep_uses_a_reproducible_half_diagonal_for_rectangular_cells() {
    let mut blank = spec(1.);
    blank.max_mm = [0.4599609375, 0.2822265625, 10.];
    let mut stock = Stock::new(blank).unwrap();
    let [x, y] = stock.cell_center(0).unwrap();
    // The segment passes through the cell center, so its distance there
    // is exactly zero. Only the half-diagonal supplies radial uncertainty.
    let sweep = line([0., y, 0.], [x, y, 0.]);
    let delta = preview(&stock, &tool(CutterShape::Ball), &sweep, 1.);
    stock.apply_delta(&delta).unwrap();
    // Same high-precision reference as the endpoint test above.
    assert_eq!(stock.cells()[0].upper_mm, 0.018295936542288716);
}

#[test]
fn ramps_plunges_and_clipped_sweeps_enclose_independent_quadratic_oracle() {
    for (a, b, f) in [
        ([3., 3., 9.], [17., 17., 4.], 1.),
        ([10., 10., 9.], [10., 10., 4.], 1.),
        ([-10., 7., 6.], [40., 7., 6.], 1.),
        ([3., 3., 9.], [17., 17., 4.], 0.371),
    ] {
        let mut stock = Stock::new(spec(0.4)).unwrap();
        let m = line(a, b);
        let delta = preview(&stock, &tool(CutterShape::Flat), &m, f);
        stock.apply_delta(&delta).unwrap();
        enclose_samples(&stock, |p| flat_line_height(p, a, b, f));
    }
}

// Independent angular interval solution for a flat tool on a two-turn XY helix.
fn flat_circle_height(p: [f64; 2], sweep: f64, z0: f64, z1: f64, f: f64) -> f64 {
    let q = [p[0] - 10., p[1] - 10.];
    let rho = q[0].hypot(q[1]);
    let cosine = if rho == 0. {
        return 10.;
    } else {
        (rho * rho + 9. - 4.) / (6. * rho)
    };
    if cosine > 1. {
        return 10.;
    }
    let alpha = cosine.max(-1.).acos();
    let phi = q[1].atan2(q[0]);
    let mut height = 10_f64;
    for k in -5..=5 {
        let (a, b) = (
            (phi - alpha + k as f64 * TAU) / sweep,
            (phi + alpha + k as f64 * TAU) / sweep,
        );
        let (lo, hi) = (a.min(b).max(0.), a.max(b).min(f));
        if lo <= hi {
            height = height.min(z0 + (z1 - z0) * if z1 > z0 { lo } else { hi });
        }
    }
    height
}

#[test]
fn arcs_and_multiturn_helices_enclose_independent_angular_interval_oracle() {
    for (sweep, z1, f) in [
        (FRAC_PI_2, 6., 1.),
        (TAU, 6., 1.),
        (2. * TAU + FRAC_PI_2, 4., 1.),
        (-2. * TAU - FRAC_PI_2, 4., 0.413),
    ] {
        let z0 = if z1 == 6. { 6. } else { 9. };
        let angle = sweep;
        let start = [13., 10., z0];
        let end = [10. + 3. * angle.cos(), 10. + 3. * angle.sin(), z1];
        let curve = if z0 == z1 {
            Curve::Arc {
                start,
                end,
                center: [10., 10., z0],
                plane: Plane::Xy,
                sweep_rad: sweep,
            }
        } else {
            Curve::Helix {
                start,
                end,
                center: [10., 10., z0],
                plane: Plane::Xy,
                sweep_rad: sweep,
            }
        };
        let mut stock = Stock::new(spec(0.4)).unwrap();
        let delta = preview(&stock, &tool(CutterShape::Flat), &motion(curve), f);
        assert!(delta.stats().segments > 1);
        stock.apply_delta(&delta).unwrap();
        enclose_samples(&stock, |p| flat_circle_height(p, sweep, z0, z1, f));
    }
}

#[test]
fn non_xy_arcs_bound_between_endpoint_z_excursions() {
    let mut s = spec(0.2);
    s.min_mm = [-10., -4., -7.];
    s.max_mm = [10., 4., 4.];
    let mut stock = Stock::new(s).unwrap();
    let curve = Curve::Arc {
        start: [0., 0., 5.],
        end: [0., 0., 5.],
        center: [0., 0., 0.],
        plane: Plane::Xz,
        sweep_rad: TAU,
    };
    let delta = preview(&stock, &tool(CutterShape::Flat), &motion(curve), 1.);
    stock.apply_delta(&delta).unwrap();
    assert!(height_at(&stock, [0., 0.]).upper_mm < -4.9);
    assert!(height_at(&stock, [0., 0.]).lower_mm <= -5.);
}

#[test]
fn partial_prefixes_replay_deterministically_from_move_start() {
    let stock = Stock::new(spec(0.2)).unwrap();
    let tool = tool(CutterShape::Ball);
    let m = line([3., 10., 8.], [17., 10., 5.]);
    let baseline = stock.snapshot();
    for fraction in [0., 0.001, 0.371, 0.5, 1.] {
        let delta = preview(&stock, &tool, &m, fraction);
        let mut playback = Stock::from_snapshot(baseline.clone()).unwrap();
        playback.apply_delta(&delta).unwrap();
        let mut seek = Stock::from_snapshot(baseline.clone()).unwrap();
        let replay = preview(&seek, &tool, &m, fraction);
        seek.apply_delta(&replay).unwrap();
        assert_eq!(seek.snapshot(), playback.snapshot());
        if fraction == 0. {
            assert!(delta.changes().is_empty());
        }
    }
    let half = preview(&stock, &tool, &m, 0.5);
    let mut partial = stock.clone();
    partial.apply_delta(&half).unwrap();
    assert!(height_at(&partial, [7., 10.]).upper_mm < 10.);
    assert_eq!(height_at(&partial, [16., 10.]).lower_mm, 10.);
}

#[test]
fn transactional_deltas_undo_snapshots_and_reject_corruption() {
    let mut stock = Stock::new(spec(0.3)).unwrap();
    let initial = stock.snapshot();
    let tool = tool(CutterShape::Flat);
    let first = preview(&stock, &tool, &line([3., 5., 6.], [17., 5., 6.]), 1.);
    stock.apply_delta(&first).unwrap();
    let state = stock.snapshot();
    assert!(stock.apply_delta(&first).is_err());
    assert_eq!(stock.snapshot(), state);
    let second = preview(&stock, &tool, &line([3., 12., 5.], [17., 12., 5.]), 1.);
    let encoded = serde_json::to_value(&second).unwrap();
    for case in 0..6 {
        let mut bad = encoded.clone();
        match case {
            0 => bad["changes"][1]["index"] = bad["changes"][0]["index"].clone(),
            1 => bad["changes"][0]["index"] = serde_json::json!(100_000_000),
            2 => bad["changes"][1]["before"]["lower_mm"] = serde_json::json!(0.),
            3 => bad["changes"][1]["after"]["upper_mm"] = serde_json::json!(11.),
            4 => bad["base_generation"] = serde_json::json!(7),
            _ => bad["spec"]["cell_size_mm"] = serde_json::json!(0.4),
        }
        let bad: StockDelta = serde_json::from_value(bad).unwrap();
        assert!(stock.apply_delta(&bad).is_err());
        assert_eq!(stock.snapshot(), state);
    }
    stock.apply_delta(&second).unwrap();
    let final_state = stock.snapshot();
    assert_eq!(
        Stock::from_snapshot(final_state.clone())
            .unwrap()
            .snapshot(),
        final_state
    );
    assert!(stock.undo_delta(&first).is_err());
    assert_eq!(stock.snapshot(), final_state);
    stock.undo_delta(&second).unwrap();
    assert_eq!(stock.snapshot(), state);
    stock.undo_delta(&first).unwrap();
    assert_eq!(stock.snapshot(), initial);
    let mut invalid = initial;
    invalid.cells[0].lower_mm = 11.;
    assert!(Stock::from_snapshot(invalid).is_err());
}

#[test]
fn budgets_cancellation_and_insufficient_reach_preserve_state() {
    let mut s = spec(0.1);
    s.max_mm = [100., 100., 10.];
    let stock = Stock::new(s).unwrap();
    let before = stock.snapshot();
    let tool = tool(CutterShape::Flat);
    let m = line([1., 50., 5.], [99., 50., 5.]);
    let calls = Cell::new(0);
    let cancel = || {
        calls.set(calls.get() + 1);
        // Cancel during useful sweep work; copy/collection passes are no
        // longer present in the direct analytic kernel.
        calls.get() > 20
    };
    let error = stock
        .preview_sweep(&tool, &m, 1., SweepOptions::default(), &cancel)
        .unwrap_err();
    assert!(error.contains("cancelled"), "{error}");
    assert_eq!(calls.get(), 21);
    assert_eq!(stock.snapshot(), before);
    for options in [
        SweepOptions {
            max_scratch_cells: 1,
            ..Default::default()
        },
        SweepOptions {
            max_cell_visits: 1,
            ..Default::default()
        },
        SweepOptions {
            max_changed_cells: 1,
            ..Default::default()
        },
        SweepOptions {
            tolerance_mm: f64::NAN,
            ..Default::default()
        },
    ] {
        assert!(
            stock
                .preview_sweep(&tool, &m, 1., options, &AtomicBool::new(false))
                .is_err()
        );
        assert_eq!(stock.snapshot(), before);
    }
    let arc = motion(Curve::Arc {
        start: [53., 50., 5.],
        end: [53., 50., 5.],
        center: [50., 50., 5.],
        plane: Plane::Xy,
        sweep_rad: TAU,
    });
    assert!(
        stock
            .preview_sweep(
                &tool,
                &arc,
                1.,
                SweepOptions {
                    max_segments: 1,
                    ..Default::default()
                },
                &AtomicBool::new(false)
            )
            .is_err()
    );
    let deep = line([1., 50., -25.], [99., 50., -25.]);
    assert!(
        stock
            .preview_sweep(
                &tool,
                &deep,
                1.,
                SweepOptions::default(),
                &AtomicBool::new(false)
            )
            .unwrap_err()
            .contains("reach")
    );
    assert_eq!(stock.snapshot(), before);
}

#[test]
fn short_cutter_reach_uses_affected_whole_cell_uppers_and_counts_atomic_work() {
    let long_tool = tool(CutterShape::Flat);
    let mut definition = long_tool.definition().clone();
    definition.cutter.cutting_length_mm = 5.;
    let short_tool = definition.prepare().unwrap();
    let mut snapshot = Stock::new(spec(0.5)).unwrap().snapshot();
    snapshot.generation = 17;
    for cell in &mut snapshot.cells {
        cell.lower_mm = 6.;
        cell.upper_mm = 6.;
    }
    // Material far from the full path rectangle is allowed to remain tall.
    snapshot.cells[0].upper_mm = 10.;
    let stock = Stock::from_snapshot(snapshot).unwrap();
    let before = stock.snapshot();
    let motion = line([5., 10., 2.], [15., 10., 2.]);
    let delta = stock
        .preview_sweep(
            &short_tool,
            &motion,
            1.,
            SweepOptions::default(),
            &AtomicBool::new(false),
        )
        .unwrap();
    assert!(delta.stats().reach_cell_visits > 0);
    assert!(delta.stats().visited_cells > 0);
    assert!(!delta.changes().is_empty());
    assert_eq!(stock.snapshot(), before);
    let no_reach = stock
        .preview_sweep(
            &long_tool,
            &motion,
            1.,
            SweepOptions::default(),
            &AtomicBool::new(false),
        )
        .unwrap();
    assert_eq!(no_reach.stats().reach_cell_visits, 0);
    assert!(
        serde_json::to_value(no_reach.stats())
            .unwrap()
            .get("reach_cell_visits")
            .is_none()
    );
    let mut uncertain = before.clone();
    uncertain.cells[20 * 40 + 20].upper_mm = 8.;
    let uncertain = Stock::from_snapshot(uncertain).unwrap();
    assert!(
        uncertain
            .preview_sweep(
                &short_tool,
                &motion,
                1.,
                SweepOptions::default(),
                &AtomicBool::new(false)
            )
            .unwrap_err()
            .contains("reach")
    );
    assert!(
        stock
            .preview_sweep(
                &short_tool,
                &motion,
                1.,
                SweepOptions {
                    max_cell_visits: 1,
                    ..Default::default()
                },
                &AtomicBool::new(false)
            )
            .unwrap_err()
            .contains("budget")
    );
    let calls = Cell::new(0);
    let cancel = || {
        calls.set(calls.get() + 1);
        calls.get() > 10
    };
    assert!(
        stock
            .preview_sweep(&short_tool, &motion, 1., SweepOptions::default(), &cancel)
            .unwrap_err()
            .contains("cancelled")
    );
    assert_eq!(stock.snapshot(), before);
}

#[test]
fn full_sweep_envelope_cannot_infer_air_from_arc_or_helix_endpoints() {
    let stock = Stock::new(spec(0.5)).unwrap();
    let before = stock.snapshot();
    let cancel = AtomicBool::new(false);
    let arc = motion(Curve::Arc {
        start: [-3., 0., 5.],
        end: [-3., 20., 5.],
        center: [-3., 10., 5.],
        plane: Plane::Xy,
        sweep_rad: PI,
    });
    assert!(
        !stock
            .sweep_misses_stock_envelope(&arc, 2., 1e-5, &cancel)
            .unwrap()
    );
    let left = motion(Curve::Arc {
        start: [-3., 0., 5.],
        end: [-3., 20., 5.],
        center: [-3., 10., 5.],
        plane: Plane::Xy,
        sweep_rad: -PI,
    });
    assert!(
        stock
            .sweep_misses_stock_envelope(&left, 2., 1e-5, &cancel)
            .unwrap()
    );
    let helix = motion(Curve::Helix {
        start: [-3., 0., 4.],
        end: [-3., 20., 6.],
        center: [-3., 10., 4.],
        plane: Plane::Xy,
        sweep_rad: PI,
    });
    assert!(
        !stock
            .sweep_misses_stock_envelope(&helix, 2., 1e-5, &cancel)
            .unwrap()
    );
    for (z, clear) in [(10., false), (10.1, true)] {
        assert_eq!(
            stock
                .sweep_misses_stock_envelope(&line([5., 5., z], [6., 6., z]), 2., 1e-5, &cancel)
                .unwrap(),
            clear
        );
    }
    for (x, clear) in [(-2., false), (-2.000005, false), (-2.00004, true)] {
        assert_eq!(
            stock
                .sweep_misses_stock_envelope(&line([x, 5., 5.], [x, 6., 5.]), 2., 1e-5, &cancel)
                .unwrap(),
            clear
        );
    }
    assert!(
        stock
            .sweep_misses_stock_envelope(&arc, 2., 1e-5, &AtomicBool::new(true))
            .unwrap_err()
            .contains("cancelled")
    );
    assert!(
        stock
            .sweep_misses_stock_envelope(&arc, f64::NAN, 1e-5, &cancel)
            .is_err()
    );
    assert_eq!(stock.snapshot(), before);
}

#[test]
fn rapids_do_not_remove_stock_and_translation_preserves_bounds() {
    let stock = Stock::new(spec(0.4)).unwrap();
    let tool = tool(CutterShape::Flat);
    let m = line([3., 10., 6.], [17., 10., 6.]);
    let mut rapid = m.clone();
    rapid.kind = MoveKind::Rapid;
    rapid.feed_mm_min = None;
    assert!(preview(&stock, &tool, &rapid, 1.).changes().is_empty());
    let source = preview(&stock, &tool, &m, 1.);
    let mut moved_spec = spec(0.4);
    moved_spec.min_mm[0] += 100_000.;
    moved_spec.max_mm[0] += 100_000.;
    let moved = Stock::new(moved_spec).unwrap();
    let translated = line([100_003., 10., 6.], [100_017., 10., 6.]);
    assert_eq!(
        source.changes(),
        preview(&moved, &tool, &translated, 1.).changes()
    );
}

#[test]
fn feed_controlled_clearance_preserves_all_stock_and_validates_feed() {
    let stock = Stock::new(spec(0.4)).unwrap();
    let tool = tool(CutterShape::Flat);
    let before = stock.snapshot();
    let mut m = line([3., 10., 6.], [17., 10., 6.]);
    m.kind = MoveKind::Rapid;
    m.feed_mm_min = Some(600.);
    for fraction in [0., 0.371, 1.] {
        assert!(preview(&stock, &tool, &m, fraction).changes().is_empty());
    }
    assert_eq!(stock.snapshot(), before);
    for feed in [0., -1., f64::NAN, f64::INFINITY] {
        m.feed_mm_min = Some(feed);
        assert!(
            stock
                .preview_sweep(
                    &tool,
                    &m,
                    1.,
                    SweepOptions::default(),
                    &AtomicBool::new(false)
                )
                .is_err()
        );
    }
}

// Exact extrema over a complete rectangular cell for a full circular axis:
// radius-to-center ranges continuously from the rectangle minimum to maximum.
// This checks the entire footprint, not a set of sampled points.
#[test]
fn analytic_full_circles_enclose_exact_whole_rectangle_height_ranges() {
    let t = tool(CutterShape::Flat);
    for shift in [[0., 0.], [500_000., -300_000.]] {
        for radius in [1., 3.] {
            for sweep in [TAU, -2. * TAU] {
                let mut s = spec(0.3);
                for (k, offset) in shift.into_iter().enumerate() {
                    s.min_mm[k] += offset;
                    s.max_mm[k] += offset;
                }
                let mut stock = Stock::new(s).unwrap();
                let center = [10. + shift[0], 10. + shift[1], 6.];
                let start = [center[0] + radius, center[1], 6.];
                let m = motion(Curve::Arc {
                    start,
                    end: start,
                    center,
                    plane: Plane::Xy,
                    sweep_rad: sweep,
                });
                let d = preview(&stock, &t, &m, 1.);
                assert_eq!(d.stats().materialized_segments, 0);
                assert!(d.stats().analytic_path_queries > 0);
                assert_eq!(d.stats().scratch_cells, 0);
                assert_eq!(d.stats().scratch_bytes, 0);
                assert!(
                    d.stats().visited_cells <= SweepOptions::default().max_scratch_cells as u64
                );
                assert!(d.stats().segments > 1);
                stock.apply_delta(&d).unwrap();
                let [dx, dy] = stock.cell_widths_mm();
                let mut fully_cut = 0;
                for (i, h) in stock.cells().iter().enumerate() {
                    let p = stock.cell_center(i).unwrap();
                    let x = (p[0] - center[0]).abs();
                    let y = (p[1] - center[1]).abs();
                    let lo = (x - dx / 2.).max(0.).hypot((y - dy / 2.).max(0.));
                    let hi = (x + dx / 2.).hypot(y + dy / 2.);
                    let near = if lo <= radius && radius <= hi {
                        0.
                    } else {
                        (lo - radius).abs().min((hi - radius).abs())
                    };
                    let far = (lo - radius).abs().max((hi - radius).abs());
                    let exact_min = if near <= 2. { 6. } else { 10. };
                    let exact_max = if far <= 2. { 6. } else { 10. };
                    assert!(
                        h.lower_mm <= exact_min + 1e-8 && h.upper_mm >= exact_max - 1e-8,
                        "whole cell {p:?}: {h:?} doesn't enclose [{exact_min},{exact_max}]"
                    );
                    fully_cut += usize::from(h.upper_mm < 6.001);
                }
                assert!(fully_cut > 50);
                let exact_area = PI * ((radius + 2.).powi(2) - (radius - 2.).max(0.).powi(2));
                assert!(stock.volume().removed_lower_mm3 <= exact_area * 4.);
                assert!(stock.volume().removed_upper_mm3 >= exact_area * 4.);
            }
        }
    }
}

#[test]
fn analytic_arc_prefixes_respect_sense_endpoint_error_future_stock_and_replay() {
    let stock = Stock::new(spec(0.2)).unwrap();
    let t = tool(CutterShape::Flat);
    for sweep in [TAU, -TAU, 1000. * TAU, -1000. * TAU] {
        // Deliberately include a valid declared endpoint mismatch.
        let end = [13.000004, 10., 6.];
        let m = motion(Curve::Arc {
            start: [13., 10., 6.],
            end,
            center: [10., 10., 6.],
            plane: Plane::Xy,
            sweep_rad: sweep,
        });
        for fraction in [0., 1e-8, 0.001, 0.07, 0.23, 0.51, 1.] {
            let d = preview(&stock, &t, &m, fraction);
            let mut actual = stock.clone();
            actual.apply_delta(&d).unwrap();
            if fraction == 0. {
                assert!(d.changes().is_empty());
                continue;
            }
            assert!(d.stats().analytic_path_queries > 0);
            assert_eq!(d.stats().materialized_segments, 0);
            enclose_samples(&actual, |p| {
                let nominal = flat_circle_height(p, sweep, 6., 6., fraction);
                if fraction == 1. && (p[0] - end[0]).hypot(p[1] - end[1]) <= 2. {
                    nominal.min(6.)
                } else {
                    nominal
                }
            });
            if fraction * sweep.abs() <= FRAC_PI_2 {
                let future = height_at(&actual, [6., 10.]);
                assert_eq!(
                    future,
                    HeightBounds {
                        lower_mm: 10.,
                        upper_mm: 10.
                    }
                );
            }
            let mut replay = stock.clone();
            replay
                .apply_delta(&preview(&stock, &t, &m, fraction))
                .unwrap();
            assert_eq!(actual.snapshot(), replay.snapshot());
            actual.undo_delta(&d).unwrap();
            assert_eq!(actual.snapshot(), stock.snapshot());
        }
    }
}

#[test]
fn old_delta_json_without_new_work_counters_still_restores_stock() {
    let stock = Stock::new(spec(0.4)).unwrap();
    let d = preview(
        &stock,
        &tool(CutterShape::Flat),
        &line([3., 5., 6.], [17., 5., 6.]),
        1.,
    );
    let mut old = serde_json::to_value(&d).unwrap();
    old["stats"]
        .as_object_mut()
        .unwrap()
        .remove("materialized_segments");
    old["stats"]
        .as_object_mut()
        .unwrap()
        .remove("analytic_path_queries");
    let restored: StockDelta = serde_json::from_value(old).unwrap();
    assert_eq!(restored.stats().analytic_path_queries, 0);
    let mut a = stock.clone();
    let mut b = stock.clone();
    a.apply_delta(&d).unwrap();
    b.apply_delta(&restored).unwrap();
    assert_eq!(a.snapshot(), b.snapshot());
}

#[test]
fn direct_analytic_deltas_keep_prefix_order_limits_and_transactional_undo() {
    let tool = tool(CutterShape::Flat);
    for m in [
        line([2., 4., 6.], [18., 16., 6.]),
        motion(Curve::Arc {
            start: [13., 10., 6.],
            end: [10., 7., 6.],
            center: [10., 10., 6.],
            plane: Plane::Xy,
            sweep_rad: -TAU / 4.,
        }),
    ] {
        for fraction in [0.125, 0.5, 1.] {
            let mut stock = Stock::new(spec(0.2)).unwrap();
            let before = stock.snapshot();
            let d = preview(&stock, &tool, &m, fraction);
            assert_eq!(d.stats().scratch_cells, 0);
            assert_eq!(d.stats().scratch_bytes, 0);
            assert!(!d.changes().is_empty());
            assert!(d.changes().windows(2).all(|p| p[0].index < p[1].index));
            let cap = d.changes().len();
            let options = SweepOptions {
                max_changed_cells: cap,
                ..Default::default()
            };
            let exact = stock
                .preview_sweep(&tool, &m, fraction, options, &|| false)
                .unwrap();
            assert_eq!(d.changes(), exact.changes());
            let options = SweepOptions {
                max_changed_cells: cap - 1,
                ..options
            };
            assert!(
                stock
                    .preview_sweep(&tool, &m, fraction, options, &|| false)
                    .is_err()
            );
            assert_eq!(stock.snapshot(), before);
            stock.apply_delta(&d).unwrap();
            assert!(stock.apply_delta(&d).is_err());
            stock.undo_delta(&d).unwrap();
            assert_eq!(stock.snapshot(), before);
        }
    }
}
