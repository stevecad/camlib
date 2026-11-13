use camlib::{
    motion::{Curve, MoveKind},
    operation::OperationPlan,
    pocket::{HelicalEntry, Operation, PocketEntry},
    stock::Stock,
    stream::StreamManifest,
};
use camlib::{planar::ConstructionOptions, roughing::PreparedRoughingTarget, surface::SurfaceMesh};
use std::f64::consts::{PI, TAU};
use std::sync::atomic::AtomicBool;

/// Helical entries are emitted one full turn per motion. Number of motions
/// added by that split: helix motions that continue the previous helix.
fn continued_entry_turns<'a>(curves: impl IntoIterator<Item = &'a Curve>) -> usize {
    let mut last: Option<[f64; 3]> = None;
    let mut extra = 0;
    for curve in curves {
        if matches!(curve, Curve::Helix { .. }) {
            extra += usize::from(last == Some(curve.start()));
            last = Some(curve.end());
        } else {
            last = None;
        }
    }
    extra
}

fn mesh(triangles: Vec<[[f64; 3]; 3]>, error: f64) -> SurfaceMesh {
    SurfaceMesh {
        version: 1,
        units: "mm".into(),
        triangles: triangles.into(),
        surface_error_mm: error,
        source: serde_json::Value::Null,
    }
}
fn rect(x0: f64, y0: f64, x1: f64, y1: f64, z: f64) -> Vec<[[f64; 3]; 3]> {
    vec![
        [[x0, y0, z], [x1, y0, z], [x1, y1, z]],
        [[x0, y0, z], [x1, y1, z], [x0, y1, z]],
    ]
}
fn signed(region: &camlib::planar::PreparedRegion, p: [f64; 2]) -> f64 {
    region
        .query(p)
        .unwrap()
        .signed_clearance_mm
        .unwrap_or(f64::NEG_INFINITY)
}

#[test]
fn sloped_column_shadow_matches_an_independent_linear_solid_oracle() {
    let cancel = AtomicBool::new(false);
    for shift in [[0., 0., 0.], [10000., -20000., 30000.]] {
        let triangles = vec![
            [[0., 0., 2.], [12., 0., 8.], [12., 8., 8.]],
            [[0., 0., 2.], [12., 8., 8.], [0., 8., 2.]],
        ]
        .into_iter()
        .map(|t| t.map(|p| std::array::from_fn(|i| p[i] + shift[i])))
        .collect();
        let target = PreparedRoughingTarget::prepare(mesh(triangles, 0.02), 1e-5, &cancel).unwrap();
        for z in [2.5, 4., 6., 7.5] {
            let shadow = target.shadow(z + shift[2], 0.2, &mut 0, &cancel).unwrap();
            let region = shadow
                .region
                .prepare(ConstructionOptions::default())
                .unwrap();
            // Exact protected target is h(x,y)=2+x/2+axial allowance.
            // Source error adds a vertical reserve as well as XY uncertainty.
            let threshold = 2. * (z - 0.2 - 0.02 - 2.);
            for ix in 1..120 {
                for iy in 1..80 {
                    let p = [ix as f64 / 10., iy as f64 / 10.];
                    let s = signed(&region, [p[0] + shift[0], p[1] + shift[1]]);
                    if p[0] >= threshold + 0.01 {
                        assert!(s > 0., "missing retained column at {p:?},Z{z}");
                    }
                    if p[0] < threshold - 0.01 {
                        assert!(s < 0., "unexplained excess shadow at {p:?},Z{z}");
                    }
                }
            }
            assert!(shadow.report.boundary_error_mm >= 0.02);
            assert!(shadow.report.boundary_error_mm < 0.025);
        }
    }
}

#[test]
fn overhanging_roofs_holes_and_disconnected_solids_protect_the_entire_column() {
    let cancel = AtomicBool::new(false);
    let mut triangles = rect(0., 0., 12., 3., 8.);
    triangles.extend(rect(0., 7., 12., 10., 8.));
    triangles.extend(rect(0., 3., 4., 7., 8.));
    triangles.extend(rect(8., 3., 12., 7., 8.));
    triangles.extend(rect(20., 2., 24., 6., 6.));
    let target = PreparedRoughingTarget::prepare(mesh(triangles, 0.), 1e-5, &cancel).unwrap();
    let shadow = target.shadow(4., 0., &mut 0, &cancel).unwrap();
    let region = shadow
        .region
        .prepare(ConstructionOptions::default())
        .unwrap();
    assert_eq!(region.summary().components, 2);
    assert_eq!(region.summary().holes, 1);
    for p in [[2., 5.], [6., 1.], [6., 9.], [10., 5.], [22., 4.]] {
        assert!(signed(&region, p) > 0.5);
    }
    for p in [[6., 5.], [16., 4.], [-1., 5.]] {
        assert!(signed(&region, p) < -0.5);
    }
    // A horizontal section at Z4 would see none of these roofs. A cutter
    // extending upwards from Z4 must still protect every one of them.
    let high = target
        .shadow(7., 0., &mut 0, &cancel)
        .unwrap()
        .region
        .prepare(ConstructionOptions::default())
        .unwrap();
    assert_eq!(high.summary().components, 1);
    assert!(signed(&high, [22., 4.]) < 0.);
    assert!(
        target
            .shadow(9., 0., &mut 0, &cancel)
            .unwrap()
            .region
            .components
            .is_empty()
    );
}

#[test]
fn vertical_and_point_projections_are_enclosed_not_dropped() {
    let cancel = AtomicBool::new(false);
    let target = PreparedRoughingTarget::prepare(
        mesh(vec![[[5., 2., 2.], [5., 10., 2.], [5., 2., 8.]]], 0.),
        1e-5,
        &cancel,
    )
    .unwrap();
    let shadow = target.shadow(6., 0., &mut 0, &cancel).unwrap();
    assert_eq!(shadow.report.enclosed_degenerate_projections, 1);
    let region = shadow
        .region
        .prepare(ConstructionOptions::default())
        .unwrap();
    assert!(signed(&region, [5., 3.]) > 0.);
    assert!(signed(&region, [5., 7.]) < 0.);
    // A level just below the top leaves a tiny but real projected segment.
    let point = target.shadow(8., 0., &mut 0, &cancel).unwrap();
    assert!(!point.region.components.is_empty());
    assert!(
        point
            .region
            .prepare(ConstructionOptions::default())
            .unwrap()
            .summary()
            .area_mm2
            > 0.
    );
}

#[test]
fn invalid_work_and_cancelled_column_preparation_never_return_a_shadow() {
    let clear = AtomicBool::new(false);
    let cancelled = AtomicBool::new(true);
    assert!(
        PreparedRoughingTarget::prepare(mesh(rect(0., 0., 4., 4., 2.), 0.), 1e-5, &cancelled)
            .is_err()
    );
    let target =
        PreparedRoughingTarget::prepare(mesh(rect(0., 0., 4., 4., 2.), 0.), 1e-5, &clear).unwrap();
    assert!(target.shadow(1., 0., &mut 0, &cancelled).is_err());
    assert!(target.shadow(f64::NAN, 0., &mut 0, &clear).is_err());
    assert!(target.shadow(1., -1., &mut 0, &clear).is_err());
    // Work is counted, never capped.
    assert!(target.shadow(1., 0., &mut 20_000_000, &clear).is_ok());
    let a = target.shadow(1., 0., &mut 0, &clear).unwrap();
    let b = target.shadow(1., 0., &mut 0, &clear).unwrap();
    assert_eq!(
        serde_json::to_value(a.region).unwrap(),
        serde_json::to_value(b.region).unwrap()
    );
}

fn fixture(adaptive: bool) -> StreamManifest {
    serde_json::from_str(if adaptive {
        include_str!("../fixtures/operations/roughing-adaptive.json")
    } else {
        include_str!("../fixtures/operations/roughing.json")
    })
    .unwrap()
}

#[test]
fn deep_roughing_checks_layer_depth_and_preserves_shank_and_holder_clearance() {
    let cancel = AtomicBool::new(false);
    let base: StreamManifest = serde_json::from_str(include_str!(
        "../fixtures/operations/roughing-layer-depth.json"
    ))
    .unwrap();
    for (holder_bottom, shank_radius, should_run) in
        [(80., 6., true), (76., 6., false), (80., 9., false)]
    {
        let mut m = base.clone();
        m.tools[0].holders[0].z_min_mm = holder_bottom;
        m.tools[0].stickout_mm = holder_bottom;
        m.tools[0].cutter.shank_diameter_mm = 2. * shank_radius;
        let prepared = m.tools[0].prepare().unwrap();
        let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let mut plan = OperationPlan::prepare_with_stock(
            m.operations[0].clone(),
            &m.tools[0],
            &stock,
            m.initial_position_mm,
            1,
            &cancel,
        )
        .expect("total depth must not be compared with flute length");
        let mut minimum_z = 0_f64;
        let outcome = (|| -> Result<(), String> {
            while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel)? {
                minimum_z = minimum_z.min(motion.curve.end()[2]);
                // Independent analytic solid: a 20 mm square boss from -78
                // to 0, on a floor at -78. Enclose entire curve intervals,
                // rather than sampling endpoints or reusing mesh shadows.
                assert!(motion.curve.start()[2].min(motion.curve.end()[2]) >= -77.80001);
                if motion.curve.start()[2].min(motion.curve.end()[2]) <= 0. {
                    let mut spans = vec![(0., 1., 0)];
                    while let Some((a, b, depth)) = spans.pop() {
                        if box_distance(bounds(&motion.curve, a, b), [[30., 30.], [50., 50.]]) >= 6.
                        {
                            continue;
                        }
                        assert!(depth < 24, "unproven boss clearance: {:?}", motion.curve);
                        let middle = (a + b) / 2.;
                        spans.push((a, middle, depth + 1));
                        spans.push((middle, b, depth + 1));
                    }
                }
                let delta = stock.preview_sweep(&prepared, &motion, 1., m.sweep, &cancel)?;
                stock.apply_delta(&delta)?;
            }
            Ok(())
        })();
        if should_run {
            outcome.unwrap();
            assert!(minimum_z < -77., "did not reach final layer: {minimum_z}");
            assert!(stock.volume().removed_lower_mm3 > 400_000.);
        } else {
            let error = outcome.unwrap_err();
            assert!(error.contains("roughing layer"), "{error}");
            // A typed refusal naming the section and the stock it would
            // meet (#160), not prose about the original stock.
            let refusal = camlib::reach::ReachRefusal::parse(&error).expect(&error);
            assert_eq!(
                refusal.limit,
                if shank_radius > 6. {
                    camlib::reach::ReachLimit::Shank
                } else {
                    camlib::reach::ReachLimit::Holder
                }
            );
            assert_eq!(
                (refusal.operation.as_str(), refusal.tool.as_str()),
                ("rough", "em12")
            );
            assert!(refusal.top_z_mm <= 0. && refusal.top_z_mm > refusal.z_mm);
            assert_eq!(
                refusal.depth_available_mm,
                if shank_radius > 6. { 75. } else { 76. }
            );
            assert!(error.contains("would meet remaining stock"), "{error}");
            assert!(!error.contains("original stock"), "{error}");
        }
    }
}

/// #160: a part whose outline changes with depth. Each layer's passes differ
/// from the layers above, so no move repeats an earlier one, and the part
/// stands inside the bounding box of every pass round it. The shank and
/// holder are clear all the same: the upper layers cut closer to the part
/// than the lower ones do. The guard used to accept a move only if its whole
/// padded bounding box was clear or it lay exactly on one earlier sweep, and
/// refused this at the first layer deeper than the flutes.
#[test]
fn deep_roughing_of_a_tapered_part_is_cleared_by_the_stock_its_upper_layers_cut() {
    let cancel = AtomicBool::new(false);
    let mut m: StreamManifest = serde_json::from_str(include_str!(
        "../fixtures/operations/roughing-layer-depth.json"
    ))
    .unwrap();
    let Operation::Roughing(op) = &mut m.operations[0] else {
        panic!("fixture")
    };
    // Contour-parallel passes go round the part.
    op.clearing = camlib::roughing::RoughingClearing::Offset {
        stepover_mm: 6.,
        coverage_margin_mm: 0.01,
    };
    op.omit_empty_cuts = false;
    // The 20 mm square boss becomes a diamond frustum: the square turned 45
    // degrees, 20 mm across the flats at the top (Z0) and 40 mm at its foot
    // (Z-78). Its walls are diagonal, so the bounding box of a pass along
    // one holds the part.
    for tri in std::sync::Arc::make_mut(&mut op.target.triangles) {
        for p in tri {
            if (p[0] == 30. || p[0] == 50.) && (p[1] == 30. || p[1] == 50.) {
                let scale = if p[2] == 0. { 1. } else { 2. };
                let (x, y) = (p[0] - 40., p[1] - 40.);
                let h = std::f64::consts::FRAC_1_SQRT_2 * scale;
                p[0] = 40. + (x - y) * h;
                p[1] = 40. + (x + y) * h;
            }
        }
    }
    m.validate().unwrap();
    let prepared = m.tools[0].prepare().unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut plan = OperationPlan::prepare_with_stock(
        m.operations[0].clone(),
        &m.tools[0],
        &stock,
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    let mut minimum_z = 0_f64;
    let mut deep = 0;
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        let z = motion.curve.start()[2].min(motion.curve.end()[2]);
        minimum_z = minimum_z.min(z);
        if z <= 0. {
            // Independent of the planner: the cutter (radius 6) stays off
            // the diamond |x - 40| + |y - 40| <= d, whose d at height z is
            // sqrt(2) (10 - 10 z / 78).
            let d = std::f64::consts::SQRT_2 * (10. - 10. * z / 78.);
            let corners = [[d, 0.], [0., d], [-d, 0.], [0., -d]];
            for i in 0..=64 {
                let p = position(&motion.curve, i as f64 / 64.);
                let q = [p[0] - 40., p[1] - 40.];
                assert!(
                    q[0].abs() + q[1].abs() > d,
                    "cutter centre in the part at {p:?}"
                );
                let away = (0..4)
                    .map(|k| {
                        let (a, b) = (corners[k], corners[(k + 1) % 4]);
                        let e = [b[0] - a[0], b[1] - a[1]];
                        let t = (((q[0] - a[0]) * e[0] + (q[1] - a[1]) * e[1])
                            / (e[0] * e[0] + e[1] * e[1]))
                            .clamp(0., 1.);
                        (q[0] - a[0] - t * e[0]).hypot(q[1] - a[1] - t * e[1])
                    })
                    .fold(f64::INFINITY, f64::min);
                assert!(away >= 6. - 1e-6, "cutter in the part at {p:?}: {away}");
            }
        }
        if z < -75. && motion.kind == MoveKind::Cut {
            // Deeper than the flutes: the shank (radius 6, from 75 mm above
            // the tip) is inside the stock's height range here. The stock
            // left within its radius of the path is below it.
            deep += 1;
            let shank_bottom = z + 75.;
            for t in [0., 0.5, 1.] {
                let p = position(&motion.curve, t);
                for i in 0..12 {
                    let a = TAU * i as f64 / 12.;
                    let q = [p[0] + 5.5 * a.cos(), p[1] + 5.5 * a.sin()];
                    let cell = stock
                        .cells()
                        .iter()
                        .enumerate()
                        .find(|(k, _)| {
                            let c = stock.cell_center(*k).unwrap();
                            (c[0] - q[0]).abs() <= 0.25 && (c[1] - q[1]).abs() <= 0.25
                        })
                        .map(|(_, c)| c.upper_mm);
                    if let Some(upper) = cell {
                        assert!(upper <= shank_bottom + 1e-6, "stock at {q:?} up to {upper}");
                    }
                }
            }
        }
        let delta = stock
            .preview_sweep(&prepared, &motion, 1., m.sweep, &cancel)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
    }
    assert!(
        minimum_z < -77.,
        "did not reach the final layer: {minimum_z}"
    );
    assert!(deep > 10, "{deep} cuts deeper than the flutes");
}

/// The #122 fixture with its 20 mm square boss drafted: `draft_mm` wider
/// on every side at its foot (Z-78) than at its top.
fn drafted_boss(draft_mm: f64, offset: bool) -> StreamManifest {
    let mut m: StreamManifest = serde_json::from_str(include_str!(
        "../fixtures/operations/roughing-layer-depth.json"
    ))
    .unwrap();
    let Operation::Roughing(op) = &mut m.operations[0] else {
        panic!("fixture")
    };
    if offset {
        op.clearing = camlib::roughing::RoughingClearing::Offset {
            stepover_mm: 6.,
            coverage_margin_mm: 0.01,
        };
    }
    op.omit_empty_cuts = false;
    for tri in std::sync::Arc::make_mut(&mut op.target.triangles) {
        for p in tri {
            if (p[0] == 30. || p[0] == 50.) && (p[1] == 30. || p[1] == 50.) && p[2] != 0. {
                p[0] += draft_mm * (p[0] - 40.).signum();
                p[1] += draft_mm * (p[1] - 40.).signum();
            }
        }
    }
    m.validate().unwrap();
    m
}

/// The XY distance from `p` to the path of `c`, written out here.
fn path_distance(c: &Curve, p: [f64; 2]) -> f64 {
    let (a, b) = (c.start(), c.end());
    let ends = (a[0] - p[0])
        .hypot(a[1] - p[1])
        .min((b[0] - p[0]).hypot(b[1] - p[1]));
    match circle(c) {
        None => {
            let e = [b[0] - a[0], b[1] - a[1]];
            let w = [p[0] - a[0], p[1] - a[1]];
            let l = e[0] * e[0] + e[1] * e[1];
            let t = if l > 0. {
                (w[0] * e[0] + w[1] * e[1]) / l
            } else {
                -1.
            };
            if (0. ..=1.).contains(&t) {
                (w[0] * e[1] - w[1] * e[0]).abs() / l.sqrt()
            } else {
                ends
            }
        }
        Some((o, sweep)) => {
            let r = (a[0] - o[0]).hypot(a[1] - o[1]);
            let turn = ((p[1] - o[1]).atan2(p[0] - o[0]) - (a[1] - o[1]).atan2(a[0] - o[0]))
                .rem_euclid(TAU);
            let on = sweep.abs() >= TAU
                || if sweep >= 0. {
                    turn <= sweep
                } else {
                    TAU - turn <= -sweep
                };
            if on {
                ((p[0] - o[0]).hypot(p[1] - o[1]) - r).abs()
            } else {
                ends
            }
        }
    }
}

/// #200: a straight-shank end mill (shank as wide as the cutter) deeper
/// than its flutes beside a drafted wall. Each layer's passes lie a little
/// inside those of the layers above, so the wall passes repeat no earlier
/// move, and the shank clears the cut of the top layers by the draft across
/// them only: about 9 um for the 0.01 mm draft and 0.09 um for the 0.0001 mm
/// one (the sweeps' own path error is 0.01 um each). It is clear all the
/// same, because the layers above cut everything it passes through. The
/// planner used to refuse the first layer deeper than the flutes (0.01 mm
/// draft, contour-parallel and raster), or admit a move that the stock
/// sweep then refused and stock contact could not resolve (0.2 mm).
#[test]
fn deep_roughing_beside_a_drafted_wall_runs_with_a_shank_as_wide_as_the_cutter() {
    let cancel = AtomicBool::new(false);
    for (draft, offset) in [(0.01, true), (0.2, true), (0.01, false), (0.0001, true)] {
        let m = drafted_boss(draft, offset);
        assert_eq!(
            m.tools[0].cutter.shank_diameter_mm,
            m.tools[0].cutter.diameter_mm
        );
        let prepared = m.tools[0].prepare().unwrap();
        let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        stock.enable_contact_queries(&cancel).unwrap();
        let mut plan = OperationPlan::prepare_with_stock(
            m.operations[0].clone(),
            &m.tools[0],
            &stock,
            m.initial_position_mm,
            1,
            &cancel,
        )
        .unwrap();
        // Every flat move so far: the cutter (radius 6) emptied the column
        // over its footprint from the move's Z up.
        let mut cut: Vec<Curve> = vec![];
        let (mut minimum_z, mut deep, mut new_paths) = (0_f64, 0, 0);
        while let Some(motion) = plan
            .next_motion_with_stock(&stock, &cancel)
            .unwrap_or_else(|e| panic!("draft {draft}, offset {offset}: {e}"))
        {
            let (a, b) = (motion.curve.start(), motion.curve.end());
            let z = a[2].min(b[2]);
            minimum_z = minimum_z.min(z);
            if z < -75. && !motion.kind.is_rapid() {
                deep += 1;
                // Independent of the planner and of the proof: the shank
                // (radius 6, from 75 mm above the tip) sampled along the
                // move; every sample inside the stock is within the cutter
                // radius of the path of an earlier flat move at or below
                // the shank's bottom.
                let bottom = z + 75.;
                // A move along one earlier move's path (to a nanometre) is a
                // repeat; the others are the case at issue.
                let repeat = cut.iter().any(|c| {
                    c.start()[2] <= bottom
                        && (0..=16).all(|i| {
                            let p = position(&motion.curve, i as f64 / 16.);
                            path_distance(c, [p[0], p[1]]) <= 1e-9
                        })
                });
                for i in 0..=16 {
                    let p = position(&motion.curve, i as f64 / 16.);
                    for ring in 1..=3 {
                        for j in 0..24 {
                            let angle = TAU * j as f64 / 24.;
                            let rho = 2. * ring as f64;
                            let q = [p[0] + rho * angle.cos(), p[1] + rho * angle.sin()];
                            assert!(
                                q.iter().any(|v| !(0. ..=80.).contains(v))
                                    || cut.iter().any(|c| c.start()[2] <= bottom
                                        && path_distance(c, q) <= 6. + 1e-9),
                                "shank at {q:?} above Z{bottom} is in uncut stock: {:?}",
                                motion.curve
                            );
                        }
                    }
                }
                new_paths += usize::from(!repeat);
                // The independent stock-contact verification of the move,
                // against the stock as committed before it.
                let report = camlib::stock_contact::check(
                    &stock,
                    &prepared,
                    &motion,
                    camlib::stock_contact::Options::default(),
                    &cancel,
                )
                .unwrap();
                assert_eq!(
                    report.status,
                    camlib::verification::Status::Clear,
                    "{:?}: {:?}",
                    motion.curve,
                    report.findings.first().map(|f| &f.finding.detail)
                );
            }
            // The stock sweep's own cutting-length check admits it too.
            let delta = stock
                .preview_sweep(&prepared, &motion, 1., m.sweep, &cancel)
                .unwrap_or_else(|e| panic!("draft {draft}: {:?}: {e}", motion.curve));
            stock.apply_delta(&delta).unwrap();
            if a[2] == b[2] {
                cut.push(motion.curve);
            }
        }
        assert!(
            minimum_z < -77.,
            "did not reach the final layer: {minimum_z}"
        );
        assert!(deep > 20, "{deep} moves deeper than the flutes");
        // The case at issue: moves that lie on no earlier move's path.
        assert!(new_paths > 0, "{new_paths} of {deep} deep moves are new");
    }
}

/// #200: the proof gives nothing away. A shank 0.1 mm wider than the
/// cutter does not fit where the cutter alone has been (the 0.01 mm draft
/// leaves it 9 um, it needs 50): the first move deeper than the flutes
/// that passes the wall is refused, with the typed refusal.
#[test]
fn deep_roughing_refuses_a_shank_a_hair_wider_than_the_cut_above() {
    let cancel = AtomicBool::new(false);
    for offset in [true, false] {
        let mut m = drafted_boss(0.01, offset);
        m.tools[0].cutter.shank_diameter_mm = 12.1;
        let prepared = m.tools[0].prepare().unwrap();
        let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let mut plan = OperationPlan::prepare_with_stock(
            m.operations[0].clone(),
            &m.tools[0],
            &stock,
            m.initial_position_mm,
            1,
            &cancel,
        )
        .unwrap();
        let error = loop {
            match plan.next_motion_with_stock(&stock, &cancel) {
                Ok(Some(motion)) => {
                    let delta = stock
                        .preview_sweep(&prepared, &motion, 1., m.sweep, &cancel)
                        .unwrap();
                    stock.apply_delta(&delta).unwrap();
                }
                Ok(None) => panic!("a shank wider than the cut ran to the floor"),
                Err(error) => break error,
            }
        };
        let refusal = camlib::reach::ReachRefusal::parse(&error).expect(&error);
        assert_eq!(refusal.limit, camlib::reach::ReachLimit::Shank);
        assert_eq!(refusal.depth_available_mm, 75.);
        assert!(refusal.z_mm < -75. && refusal.top_z_mm > refusal.z_mm + 75.);
        assert!(error.contains("would meet remaining stock"), "{error}");
    }
}

fn circle(c: &Curve) -> Option<([f64; 3], f64)> {
    match *c {
        Curve::Arc {
            center, sweep_rad, ..
        }
        | Curve::Helix {
            center, sweep_rad, ..
        } => Some((center, sweep_rad)),
        _ => None,
    }
}
fn position(c: &Curve, t: f64) -> [f64; 3] {
    let a = c.start();
    let b = c.end();
    if let Some((o, s)) = circle(c) {
        let r = (a[0] - o[0]).hypot(a[1] - o[1]);
        let angle = (a[1] - o[1]).atan2(a[0] - o[0]) + s * t;
        [
            o[0] + r * angle.cos(),
            o[1] + r * angle.sin(),
            a[2] + t * (b[2] - a[2]),
        ]
    } else {
        std::array::from_fn(|i| a[i] + t * (b[i] - a[i]))
    }
}
fn bounds(c: &Curve, a: f64, b: f64) -> [[f64; 2]; 2] {
    let p = position(c, a);
    let q = position(c, b);
    let mut bbox = [
        [p[0].min(q[0]), p[1].min(q[1])],
        [p[0].max(q[0]), p[1].max(q[1])],
    ];
    if let Some((o, s)) = circle(c) {
        let start = c.start();
        let phase = (start[1] - o[1]).atan2(start[0] - o[0]);
        let r = (start[0] - o[0]).hypot(start[1] - o[1]);
        let u = phase + s * a;
        let v = phase + s * b;
        for k in 0..4 {
            let angle = k as f64 * PI / 2.;
            if ((u.min(v) - angle) / TAU).ceil() <= ((u.max(v) - angle) / TAU).floor() {
                let p = [o[0] + r * angle.cos(), o[1] + r * angle.sin()];
                for i in 0..2 {
                    bbox[0][i] = bbox[0][i].min(p[i]);
                    bbox[1][i] = bbox[1][i].max(p[i]);
                }
            }
        }
    }
    for x in &mut bbox[0] {
        *x -= 1e-9;
    }
    for x in &mut bbox[1] {
        *x += 1e-9;
    }
    bbox
}
fn box_distance(a: [[f64; 2]; 2], b: [[f64; 2]; 2]) -> f64 {
    (0..2)
        .map(|i| (a[0][i] - b[1][i]).max(b[0][i] - a[1][i]).max(0.).powi(2))
        .sum::<f64>()
        .sqrt()
}
// Entire-curve enclosure against the EXACT original frustum and rectangular
// opening, independently of triangle clipping, polygon Booleans and planner fit.
fn certify_part(c: &Curve) {
    certify_part_radius(c, 1.1);
}
fn certify_part_radius(c: &Curve, radius: f64) {
    let level = c.start()[2].min(c.end()[2]) - 0.2;
    if level > 10. {
        return;
    }
    let (part, hole) = if level <= 2. {
        ([[4., 4.], [28., 24.]], false)
    } else {
        let h = (level - 2.) / 8.;
        (
            [[8. + 3. * h, 8. + 2. * h], [24. - 3. * h, 20. - 2. * h]],
            true,
        )
    };
    let mut stack = vec![(0., 1., 0)];
    let mut visits = 0;
    while let Some((a, b, depth)) = stack.pop() {
        visits += 1;
        assert!(visits < 200_000, "part certificate work bound");
        let xy = bounds(c, a, b);
        if box_distance(xy, part) >= radius {
            continue;
        }
        if hole
            && xy[0][0] >= 14. + radius
            && xy[1][0] <= 18. - radius
            && xy[0][1] >= 12. + radius
            && xy[1][1] <= 16. - radius
        {
            continue;
        }
        assert!(
            depth < 32,
            "uncertified retained-part column at {c:?}, interval{a}..{b}"
        );
        let m = (a + b) / 2.;
        stack.push((a, m, depth + 1));
        stack.push((m, b, depth + 1));
    }
}
// Exact contact parameter intervals for a D2 flat cutter, including axial helix
// motion. No native sweep, prepared region or chord samples are used.
fn exact_height(c: &Curve, p: [f64; 2]) -> f64 {
    exact_height_for(c, p, 1., 10.)
}
fn exact_height_for(c: &Curve, p: [f64; 2], cutter_radius: f64, top: f64) -> f64 {
    let a = c.start();
    let b = c.end();
    if let Some((o, sweep)) = circle(c) {
        let r = (a[0] - o[0]).hypot(a[1] - o[1]);
        let rho = (p[0] - o[0]).hypot(p[1] - o[1]);
        if rho == 0. {
            return if r <= cutter_radius {
                a[2].min(b[2])
            } else {
                top
            };
        }
        let cos = (rho * rho + r * r - cutter_radius * cutter_radius) / (2. * rho * r);
        if cos > 1. + 1e-12 {
            return top;
        }
        let alpha = cos.clamp(-1., 1.).acos();
        let phi = (p[1] - o[1]).atan2(p[0] - o[0]);
        let start = (a[1] - o[1]).atan2(a[0] - o[0]);
        let end = start + sweep;
        let first = ((start.min(end) - phi - alpha) / TAU).ceil() as i64;
        let last = ((start.max(end) - phi + alpha) / TAU).floor() as i64;
        let mut z = top;
        for k in [first, last] {
            let x = (phi - alpha + k as f64 * TAU - start) / sweep;
            let y = (phi + alpha + k as f64 * TAU - start) / sweep;
            let lo = x.min(y).max(0.);
            let hi = x.max(y).min(1.);
            if lo <= hi {
                z = z.min(a[2] + (b[2] - a[2]) * if b[2] < a[2] { hi } else { lo });
            }
        }
        z
    } else {
        let d = [b[0] - a[0], b[1] - a[1]];
        let q = [a[0] - p[0], a[1] - p[1]];
        let aa = d[0] * d[0] + d[1] * d[1];
        let bb = 2. * (q[0] * d[0] + q[1] * d[1]);
        let cc = q[0] * q[0] + q[1] * q[1] - cutter_radius * cutter_radius;
        let (lo, hi) = if aa == 0. {
            if cc > 0. {
                return top;
            }
            (0., 1.)
        } else {
            let disc = bb * bb - 4. * aa * cc;
            if disc < 0. {
                return top;
            }
            (
                (-bb - disc.sqrt()) / (2. * aa),
                (-bb + disc.sqrt()) / (2. * aa),
            )
        };
        let lo = lo.max(0.);
        let hi = hi.min(1.);
        if lo > hi {
            top
        } else {
            a[2] + (b[2] - a[2]) * if b[2] < a[2] { hi } else { lo }
        }
    }
}

/// Independent, sampled engagement check for any path at one depth: no
/// prover, planner or stock code. Earlier feed moves are kept as centre
/// samples; a point of the cutter's leading half-circle is in material when it
/// lies in the stock footprint and no earlier centre (of an earlier move, or
/// earlier on this one) is within the cutter radius. Sampling can only miss
/// thin slivers, so it is a refutation check with a stated slack, not a proof.
struct LeadingOracle {
    r: f64,
    step: f64,
    footprint: [[f64; 2]; 2],
    cells: std::collections::HashMap<(i32, i32), Vec<[f64; 2]>>,
    worst_deg: f64,
}
impl LeadingOracle {
    fn new(r: f64, footprint: [[f64; 2]; 2]) -> Self {
        Self {
            r,
            step: r / 20.,
            footprint,
            cells: Default::default(),
            worst_deg: 0.,
        }
    }
    fn key(&self, p: [f64; 2]) -> (i32, i32) {
        (
            (p[0] / self.r).floor() as i32,
            (p[1] / self.r).floor() as i32,
        )
    }
    fn cleared(&self, p: [f64; 2]) -> bool {
        // Discs at samples `step` apart cover the swept strip but for
        // slivers step^2 / (8 r) deep.
        let reach = self.r + self.step * self.step / (8. * self.r) + 1e-9;
        let k = self.key(p);
        (k.0 - 1..=k.0 + 1).any(|x| {
            (k.1 - 1..=k.1 + 1).any(|y| {
                self.cells
                    .get(&(x, y))
                    .is_some_and(|v| v.iter().any(|q| (q[0] - p[0]).hypot(q[1] - p[1]) <= reach))
            })
        })
    }
    fn poses(&self, c: &Curve) -> Vec<([f64; 2], f64)> {
        let len = match circle(c) {
            Some((o, s)) => (c.start()[0] - o[0]).hypot(c.start()[1] - o[1]) * s.abs(),
            None => (c.end()[0] - c.start()[0]).hypot(c.end()[1] - c.start()[1]),
        };
        let n = (len / self.step).ceil().max(1.) as usize;
        (0..=n)
            .map(|i| {
                let t = i as f64 / n as f64;
                let p = position(c, t);
                let heading = match circle(c) {
                    Some((o, s)) => (p[1] - o[1]).atan2(p[0] - o[0]) + s.signum() * PI / 2.,
                    None => (c.end()[1] - c.start()[1]).atan2(c.end()[0] - c.start()[0]),
                };
                ([p[0], p[1]], heading)
            })
            .collect()
    }
    /// A flat feed move the cutter makes: clears its sweep; with `check`,
    /// its leading contact is measured first, pose by pose.
    fn feed(&mut self, c: &Curve, check: bool) {
        for (i, (p, heading)) in self.poses(c).into_iter().enumerate() {
            if check && i > 0 {
                let mut hit = 0;
                for d in -89..=89 {
                    let a = heading + (d as f64).to_radians();
                    let q = [p[0] + self.r * a.cos(), p[1] + self.r * a.sin()];
                    let inside = q[0] > self.footprint[0][0]
                        && q[0] < self.footprint[1][0]
                        && q[1] > self.footprint[0][1]
                        && q[1] < self.footprint[1][1];
                    hit += usize::from(inside && !self.cleared(q));
                }
                self.worst_deg = self.worst_deg.max(hit as f64);
            }
            let k = self.key(p);
            self.cells.entry(k).or_default().push(p);
        }
    }
}

#[test]
fn real_cad_roughing_streams_both_entries_and_preserves_whole_columns_and_every_stock_cell() {
    let cancel = AtomicBool::new(false);
    for adaptive in [false, true] {
        for clockwise in [false, true] {
            let mut m = fixture(adaptive);
            let Operation::Roughing(op) = &mut m.operations[0] else {
                panic!("fixture")
            };
            op.clockwise = clockwise;
            m.validate().unwrap();
            let mut plan = OperationPlan::prepare(
                m.operations[0].clone(),
                &m.tools[0],
                m.stock.height().unwrap(),
                m.initial_position_mm,
                17,
                &cancel,
            )
            .unwrap();
            let tool = m.tools[0].prepare().unwrap();
            let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
            let mut curves = vec![];
            let mut current = m.initial_position_mm;
            let mut outside = 0;
            let mut helices = 0;
            let mut last_helix_end: Option<[f64; 3]> = None;
            let limit = 0.75f64.acos().to_degrees();
            let mut leading = LeadingOracle::new(1., [[0., 0.], [32., 28.]]);
            let mut depth = 10.;
            while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
                assert_eq!(motion.id, 17 + curves.len() as u64);
                assert_eq!(motion.operation, "rough");
                assert_eq!(motion.tool, "t1");
                assert_eq!(motion.curve.start(), current);
                current = motion.curve.end();
                assert!(current[2] >= 2.2 && current[2] <= 12.);
                if motion.kind == MoveKind::Rapid {
                    assert!(motion.curve.start()[2] >= 12. && current[2] >= 12.);
                }
                certify_part(&motion.curve);
                if matches!(motion.kind, MoveKind::Retract)
                    || matches!(motion.curve, Curve::Helix { .. })
                {
                    let before = serde_json::to_value(plan.report()).unwrap();
                    assert!(
                        plan.next_motion_with_stock(&stock, &AtomicBool::new(true))
                            .is_err()
                    );
                    assert!(
                        plan.next_motion_with_stock(&stock, &cancel).is_err(),
                        "uncommitted generation accepted"
                    );
                    assert_eq!(serde_json::to_value(plan.report()).unwrap(), before);
                }
                let flat = motion.curve.start()[2] == current[2]
                    && !matches!(motion.curve, Curve::Helix { .. });
                if adaptive && flat && motion.kind != MoveKind::Rapid {
                    if current[2] < depth - 1e-7 {
                        // A new layer: nothing is cleared at this depth yet.
                        assert!(leading.worst_deg <= limit + 3., "{}", leading.worst_deg);
                        leading = LeadingOracle::new(1., [[0., 0.], [32., 28.]]);
                        depth = current[2];
                    }
                    // Independent of the planner's own proof: every cutting
                    // and stay-down link stroke keeps its leading contact
                    // within acos(1 - w/r). Entry strokes (helix floors,
                    // confined material) only clear.
                    leading.feed(
                        &motion.curve,
                        matches!(motion.kind, MoveKind::Cut | MoveKind::Link),
                    );
                }
                if motion.kind == MoveKind::Plunge && current[2] < 10. {
                    let beside =
                        box_distance(bounds(&motion.curve, 0., 1.), [[0., 0.], [32., 28.]]) > 1.;
                    if !adaptive {
                        assert!(beside, "air entry disk hits stock");
                    } else if !beside {
                        // engagement_front also enters through stock it has
                        // cut: the exact heights every earlier motion left
                        // under the cutter are at or below the plunge's end.
                        for radial in [0., 0.5, 1.] {
                            for i in 0..16 {
                                let angle = TAU * i as f64 / 16.;
                                let p = [
                                    current[0] + radial * angle.cos(),
                                    current[1] + radial * angle.sin(),
                                ];
                                if p[0] < 0. || p[0] > 32. || p[1] < 0. || p[1] > 28. {
                                    continue;
                                }
                                let height = curves
                                    .iter()
                                    .map(|c| exact_height(c, p))
                                    .fold(10f64, f64::min);
                                assert!(
                                    height <= current[2] + 1e-6,
                                    "plunge into material at {p:?}: {height} > {}",
                                    current[2]
                                );
                            }
                        }
                    }
                    outside += usize::from(beside);
                }
                if let Some((_, s)) = circle(&motion.curve)
                    && matches!(motion.curve, Curve::Helix { .. })
                {
                    // Raster follows `clockwise`. engagement_front's direction
                    // is its cutting sense (climb by default): a bore cut
                    // counter-clockwise, whatever `clockwise` says.
                    assert_eq!(s < 0., clockwise && !adaptive);
                    // One helix motion per turn; count descents, not turns.
                    helices += usize::from(last_helix_end != Some(motion.curve.start()));
                    last_helix_end = Some(motion.curve.end());
                }
                let delta = stock
                    .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
                    .unwrap();
                stock.apply_delta(&delta).unwrap();
                curves.push(motion.curve);
            }
            plan.finish_report(&cancel).unwrap();
            let r = serde_json::to_value(plan.report()).unwrap();
            let floor = r["target_z_mm"].as_f64().unwrap();
            assert_eq!(r["completed_layers"], 4);
            assert_eq!(r["layers"].as_array().unwrap().len(), 4);
            eprintln!(
                "adaptive={adaptive} clockwise={clockwise}: {} moves, edge entries {} (beside the stock {outside}), helices {} ({helices})",
                curves.len(),
                r["stock_edge_entries"],
                r["helical_entries"]
            );
            assert_eq!(r["helical_entries"], helices);
            if adaptive {
                // Entries from beside the stock, and through stock already cut.
                assert!(r["stock_edge_entries"].as_u64().unwrap() >= outside as u64);
            } else {
                assert_eq!(r["stock_edge_entries"], outside);
            }
            assert!(outside >= 4 && helices >= 4);
            assert_eq!(r["motions"], curves.len());
            if adaptive {
                assert!(leading.worst_deg <= limit + 3., "{}", leading.worst_deg);
                assert!((r["max_engagement_deg"].as_f64().unwrap() - limit).abs() < 1e-12);
                for layer in r["layers"].as_array().unwrap() {
                    let c = &layer["clearing"];
                    assert_eq!(c["strategy"], "engagement_front");
                    assert!((c["engagement_limit_deg"].as_f64().unwrap() - limit).abs() < 1e-12);
                    assert!(c["proven_max_engagement_deg"].as_f64().unwrap() <= limit);
                    assert!(c["engagement_proved_strokes"].as_u64().unwrap() > 0);
                }
            }
            for (i, cell) in stock.cells().iter().enumerate() {
                let p = stock.cell_center(i).unwrap();
                let height = curves
                    .iter()
                    .map(|c| exact_height(c, p))
                    .fold(10f64, f64::min);
                assert!(
                    cell.lower_mm <= height + 1e-6 && cell.upper_mm >= height - 1e-6,
                    "{p:?}: {cell:?} misses exact height{height}"
                );
                if p[0] < 5.
                    || p[0] > 27.
                    || p[1] < 5.
                    || p[1] > 23.
                    || (p[0] > 15.4 && p[0] < 16.6 && p[1] > 13.4 && p[1] < 14.6)
                {
                    assert!(
                        height <= floor + 1e-6,
                        "uncut reachable floor {p:?}:height{height}"
                    );
                }
            }
        }
    }
}

/// SAME frozen physical input as the historical 90651-motion adaptive job.
/// Complete operation/quality regression, not a production latency benchmark.
#[test]
fn stock_front_bulk_contours_need_full_native_commits_and_remove_actual_stock() {
    for sense in ["climb", "conventional"] {
        let saved: serde_json::Value =
            serde_json::from_str(include_str!("../fixtures/regressions/retained-body.json"))
                .unwrap();
        let mut selected =
            saved["cases"]["adaptive"]["policies"]["omit_proven_empty"]["input"].clone();
        selected["operations"][0]["clearing"]["path_strategy"] = "stock_front".into();
        selected["operations"][0]["clearing"]["cutting_sense"] = sense.into();
        let m: StreamManifest = serde_json::from_value(selected).unwrap();
        let tool = m.tools[0].prepare().unwrap();
        let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let mut plan = OperationPlan::prepare_with_stock(
            m.operations[0].clone(),
            &m.tools[0],
            &stock,
            m.initial_position_mm,
            1,
            &|| false,
        )
        .unwrap();
        let front = |plan: &OperationPlan| {
            serde_json::to_value(plan.report()).unwrap()["layers"][0]["clearing"]["front"].clone()
        };
        let mut history: Vec<camlib::motion::Motion> = vec![];
        let mut tested_refusal = false;
        let tangent = |curve: &Curve, end: bool| {
            let a = if end { curve.end() } else { curve.start() };
            let v = match *curve {
                Curve::Line { start, end } => [end[0] - start[0], end[1] - start[1]],
                Curve::Arc {
                    center, sweep_rad, ..
                } => [
                    -(a[1] - center[1]) * sweep_rad.signum(),
                    (a[0] - center[0]) * sweep_rad.signum(),
                ],
                _ => panic!("flat contour expected"),
            };
            let norm = v[0].hypot(v[1]);
            [v[0] / norm, v[1] / norm]
        };
        for _ in 0..120 {
            let motion = plan
                .next_motion_with_stock(&stock, &|| false)
                .unwrap()
                .unwrap();
            assert_eq!(motion.id, history.len() as u64 + 1);
            assert_eq!(
                motion.curve.start(),
                history
                    .last()
                    .map_or(m.initial_position_mm, |old| old.curve.end())
            );
            certify_part(&motion.curve);
            if let Some(old) = history.last()
                && matches!(old.kind, MoveKind::Cut | MoveKind::Lead)
                && matches!(motion.kind, MoveKind::Cut | MoveKind::Lead)
                && old.curve.start()[2] == old.curve.end()[2]
                && motion.curve.start()[2] == motion.curve.end()[2]
            {
                let a = tangent(&old.curve, true);
                let b = tangent(&motion.curve, false);
                assert!(
                    a[0] * b[0] + a[1] * b[1] >= 1. - 1e-9,
                    "bulk G1 discontinuity"
                );
            }
            let r = front(&plan);
            let last_first_contour =
                !tested_refusal && r["bulk_contours"] == 0 && r["bulk_committed_moves"] == 10;
            if last_first_contour {
                let partial = stock
                    .preview_sweep(&tool, &motion, 0.5, m.sweep, &|| false)
                    .unwrap();
                let mut fractional = stock.clone();
                fractional.apply_delta(&partial).unwrap();
                assert!(plan.next_motion_with_stock(&fractional, &|| false).is_err());
                assert_eq!(front(&plan)["bulk_contours"], 0);
                assert!(front(&plan)["bulk_material_bounds_mm"].is_null());
                let full = stock
                    .preview_sweep(&tool, &motion, 1., m.sweep, &|| false)
                    .unwrap();
                let mut restored = stock.clone();
                restored.apply_delta(&full).unwrap();
                let restored = Stock::from_snapshot(restored.snapshot()).unwrap();
                assert!(plan.next_motion_with_stock(&restored, &|| false).is_err());
                assert_eq!(front(&plan)["bulk_contours"], 0);
            }
            let delta = stock
                .preview_sweep(&tool, &motion, 1., m.sweep, &|| false)
                .unwrap();
            stock.apply_delta(&delta).unwrap();
            history.push(motion);
            if last_first_contour {
                assert!(
                    plan.next_motion_with_stock(&stock, &|| true)
                        .unwrap_err()
                        .contains("cancelled")
                );
                assert_eq!(front(&plan)["bulk_contours"], 0);
                tested_refusal = true;
            }
            if front(&plan)["bulk_contours"].as_u64().unwrap() >= 2 {
                break;
            }
        }
        let report = front(&plan);
        eprintln!(
            "bulk {sense}: {} motions, lower removal={}, {report}",
            history.len(),
            stock.volume().removed_lower_mm3
        );
        assert!(tested_refusal);
        assert_eq!(report["bulk_contours"], 2);
        assert_eq!(report["bulk_abandoned"], 0);
        assert!(stock.volume().removed_lower_mm3 > 50.);
        assert!(
            report["max_bounded_engagement_deg"].as_f64().unwrap()
                <= 2. * 0.75f64.acos().to_degrees()
        );
        let b: [[f64; 2]; 2] =
            serde_json::from_value(report["bulk_material_bounds_mm"].clone()).unwrap();
        for i in 0..stock.cells().len() {
            let p = stock.cell_center(i).unwrap();
            if (0..2).any(|k| p[k] < b[0][k] || p[k] > b[1][k]) {
                let height = history
                    .iter()
                    .filter(|m| m.kind != MoveKind::Rapid)
                    .map(|m| exact_height(&m.curve, p))
                    .fold(10f64, f64::min);
                assert!(
                    height <= 8. + 1e-6,
                    "claimed shell contains untouched stock at {p:?}: {height}"
                );
            }
        }
        assert_eq!(stock.spec(), m.stock.height().unwrap());
    }
}

#[test]
fn stock_front_saved_frustum_completes_all_layers_with_checked_stock_and_quality() {
    let saved: serde_json::Value =
        serde_json::from_str(include_str!("../fixtures/regressions/retained-body.json")).unwrap();
    let original = saved["cases"]["adaptive"]["policies"]["omit_proven_empty"]["input"].clone();
    let mut selected = original.clone();
    selected["operations"][0]["clearing"]["path_strategy"] = "stock_front".into();
    // The selector is the sole changed input: no wider engagement, altered
    // cutter, simpler CAD, stock spacing, allowances, feeds or new seed policy.
    let mut restored = selected.clone();
    restored["operations"][0]["clearing"]
        .as_object_mut()
        .unwrap()
        .remove("path_strategy");
    assert_eq!(restored, original);
    let m: StreamManifest = serde_json::from_value(selected).unwrap();
    m.validate().unwrap();
    assert_eq!(m.tools[0].cutter.diameter_mm, 2.);
    let tool = m.tools[0].prepare().unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut plan = OperationPlan::prepare_with_stock(
        m.operations[0].clone(),
        &m.tools[0],
        &stock,
        m.initial_position_mm,
        1,
        &|| false,
    )
    .unwrap();
    let mut history: Vec<camlib::motion::Motion> = Vec::new();
    let mut current = m.initial_position_mm;
    let mut nominal = 0.;
    let started = std::time::Instant::now();
    let summary = |report: serde_json::Value| {
        serde_json::json!({
            "completed_layers":report["completed_layers"], "active_layer":report["active_layer"],
            "motions":report["motions"], "max_engagement_deg":report["max_engagement_deg"],
            "layers":report["layers"].as_array().unwrap().iter().map(|layer| serde_json::json!({
                "z":layer["tip_z_mm"], "front":layer["clearing"]["front"],
                "motions":layer["clearing"]["motions"]
            })).collect::<Vec<_>>()
        })
    };
    loop {
        let motion = match plan.next_motion_with_stock(&stock, &|| false) {
            Ok(Some(motion)) => motion,
            Ok(None) => break,
            Err(error) => {
                if let Some(path) = std::env::var_os("CAMLIB_FRUSTUM_FAILURE_TRACE") {
                    let trace = serde_json::json!({
                        "complete": false,
                        "error": error,
                        "manifest": m,
                        "motions": history,
                        "stock": stock.snapshot(),
                        "report": plan.report(),
                    });
                    std::fs::write(path, serde_json::to_vec(&trace).unwrap()).unwrap();
                }
                panic!(
                    "frustum incomplete after {} motions at {current:?}: {error}; last={:?}; {}",
                    history.len(),
                    history.last(),
                    summary(serde_json::to_value(plan.report()).unwrap())
                )
            }
        };
        assert!(
            history.len() < 100000,
            "frustum exceeded bounded diagnostic work without completing"
        );
        assert_eq!(motion.id, history.len() as u64 + 1);
        assert_eq!(motion.curve.start(), current);
        assert_eq!(motion.tool, "t1");
        assert_eq!(motion.operation, "rough");
        current = motion.curve.end();
        if motion.kind == MoveKind::Rapid {
            assert!(motion.curve.start()[2] >= 12. && current[2] >= 12.);
        }
        // Continuous extrema/subdivision proof against the analytic retained
        // solid, independent of mesh clipping, source offsets and stock sweeps.
        certify_part(&motion.curve);
        if motion.kind == MoveKind::Plunge
            && current[2] < 10.
            && box_distance(bounds(&motion.curve, 0., 1.), [[0., 0.], [32., 28.]]) < 1.
        {
            // Corroborate noncutting inside-stock descents from independently
            // reconstructed full-motion heights. This is NOT a sweep proof;
            // the production reentry disk requires whole-region clearance.
            for radial in [0., 0.5, 1.] {
                for i in 0..16 {
                    let angle = TAU * i as f64 / 16.;
                    let p = [
                        current[0] + radial * angle.cos(),
                        current[1] + radial * angle.sin(),
                    ];
                    if p[0] < 0. || p[0] > 32. || p[1] < 0. || p[1] > 28. {
                        continue;
                    }
                    let height = history
                        .iter()
                        .filter(|old| old.kind != MoveKind::Rapid)
                        .map(|old| exact_height(&old.curve, p))
                        .fold(10f64, f64::min);
                    assert!(
                        height <= current[2] + 1e-6,
                        "descent into material at{p:?}: {height} > {}",
                        current[2]
                    );
                }
            }
        }
        nominal += motion
            .curve
            .prepare(m.sweep.tolerance_mm)
            .unwrap()
            .length_mm()
            / motion.feed_mm_min.unwrap_or(6000.)
            * 60.;
        let delta = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &|| false)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        history.push(motion);
        if history.len().is_multiple_of(256) {
            eprintln!(
                "frustum {} motions {:?} nominal={nominal}s lower={} {}",
                history.len(),
                started.elapsed(),
                stock.volume().removed_lower_mm3,
                summary(serde_json::to_value(plan.report()).unwrap())
            );
        }
    }
    plan.finish_report(&|| false).unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    eprintln!(
        "frustum COMPLETE {}motions nominal={nominal}s {}",
        history.len(),
        summary(report.clone())
    );
    assert_eq!(report["completed_layers"], 4);
    let angle = 2. * 0.75f64.acos().to_degrees();
    for layer in report["layers"].as_array().unwrap() {
        let clearing = &layer["clearing"];
        assert_eq!(clearing["front"]["coverage_complete"], true);
        assert!(clearing["front"]["completed_components"].as_u64().unwrap() > 0);
        // Exterior convex material exclusion and completed enclosed fan.
        assert_eq!(clearing["front"]["coverage_envelopes"], 2);
        // This case completes through fixed recipes. Candidate-potential
        // counters must not imply productive or empty material along them.
        assert_eq!(clearing["front"]["productive_candidates"], 0);
        assert_eq!(clearing["front"]["navigation_moves"], 0);
        assert_eq!(
            clearing["front"]["unranked_contour_moves"],
            clearing["front"]["front_moves"]
        );
        assert!(
            clearing["front"]["unranked_contour_moves"]
                .as_u64()
                .unwrap()
                > 0
        );
        assert!(clearing["max_bounded_engagement_deg"].as_f64().unwrap() <= angle);
        assert_eq!(
            clearing["max_bounded_engagement_deg"],
            clearing["front"]["max_bounded_engagement_deg"]
        );
    }
    let floor = report["target_z_mm"].as_f64().unwrap();
    for (i, cell) in stock.cells().iter().enumerate() {
        let p = stock.cell_center(i).unwrap();
        let height = history
            .iter()
            .filter(|m| m.kind != MoveKind::Rapid)
            .map(|m| exact_height(&m.curve, p))
            .fold(10f64, f64::min);
        assert!(
            cell.lower_mm <= height + 1e-6 && cell.upper_mm >= height - 1e-6,
            "{p:?}: {cell:?} misses independent exact height {height}"
        );
        if p[0] < 5.
            || p[0] > 27.
            || p[1] < 5.
            || p[1] > 23.
            || (p[0] > 15.4 && p[0] < 16.6 && p[1] > 13.4 && p[1] < 14.6)
        {
            assert!(
                height <= floor + 1e-6,
                "uncut reachable floor{p:?}: {height}"
            );
        }
    }
    assert!(
        history.len() <= 10000,
        "complete frustum path quality: {} >10000 motions",
        history.len()
    );
    assert!(
        nominal <= 3000.,
        "complete frustum nominal cycle quality: {nominal} >3000s"
    );
}

fn engagement_front_frustum() -> StreamManifest {
    let saved: serde_json::Value =
        serde_json::from_str(include_str!("../fixtures/regressions/retained-body.json")).unwrap();
    let mut input = saved["cases"]["adaptive"]["policies"]["omit_proven_empty"]["input"].clone();
    let op = &mut input["operations"][0];
    op["omit_empty_cuts"] = false.into();
    let clearing = op["clearing"].as_object_mut().unwrap();
    for key in [
        "orbit_radius_mm",
        "min_orbit_radius_mm",
        "local_orbit_tolerance_mm",
    ] {
        clearing.remove(key);
    }
    clearing.insert("path_strategy".into(), "engagement_front".into());
    serde_json::from_value(input).unwrap()
}

#[test]
fn engagement_front_frustum_clears_every_layer_without_touching_the_part() {
    let m = engagement_front_frustum();
    m.validate().unwrap();
    let tool = m.tools[0].prepare().unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut plan = OperationPlan::prepare_with_stock(
        m.operations[0].clone(),
        &m.tools[0],
        &stock,
        m.initial_position_mm,
        1,
        &|| false,
    )
    .unwrap();
    let mut history: Vec<camlib::motion::Motion> = Vec::new();
    let mut current = m.initial_position_mm;
    let mut nominal = 0.;
    while let Some(motion) = plan.next_motion_with_stock(&stock, &|| false).unwrap() {
        assert!(history.len() < 60000, "unbounded engagement_front path");
        assert_eq!(motion.id, history.len() as u64 + 1);
        assert_eq!(motion.curve.start(), current);
        current = motion.curve.end();
        if motion.kind == MoveKind::Rapid {
            assert!(motion.curve.start()[2] >= 12. && current[2] >= 12.);
        }
        // Independent analytic part certificate (not the planner's region).
        certify_part(&motion.curve);
        if motion.kind == MoveKind::Plunge && current[2] < 10. {
            // Vertical entries must descend through cut or outside stock:
            // exact heights left by every earlier motion around the axis.
            for radial in [0., 0.5, 1.] {
                for i in 0..16 {
                    let angle = TAU * i as f64 / 16.;
                    let p = [
                        current[0] + radial * angle.cos(),
                        current[1] + radial * angle.sin(),
                    ];
                    if p[0] < 0. || p[0] > 32. || p[1] < 0. || p[1] > 28. {
                        continue;
                    }
                    let height = history
                        .iter()
                        .filter(|old| old.kind != MoveKind::Rapid)
                        .map(|old| exact_height(&old.curve, p))
                        .fold(10f64, f64::min);
                    assert!(
                        height <= current[2] + 1e-6,
                        "plunge into material at {p:?}: {height} > {}",
                        current[2]
                    );
                }
            }
        }
        nominal += motion
            .curve
            .prepare(m.sweep.tolerance_mm)
            .unwrap()
            .length_mm()
            / motion.feed_mm_min.unwrap_or(6000.)
            * 60.;
        let delta = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &|| false)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        history.push(motion);
    }
    plan.finish_report(&|| false).unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    assert_eq!(report["completed_layers"], 4);
    let mut helices = 0;
    for layer in report["layers"].as_array().unwrap() {
        let clearing = &layer["clearing"];
        eprintln!("layer Z{}: {clearing}", layer["tip_z_mm"]);
        assert_eq!(clearing["strategy"], "engagement_front");
        assert_eq!(clearing["geometric_residual_status"], "estimated");
        assert!(clearing["passes"].as_u64().unwrap() > 0);
        assert!(
            clearing["max_estimated_engagement_mm"].as_f64().unwrap() <= 0.25 * 1.3,
            "{clearing}"
        );
        helices += clearing["helical_entries"].as_u64().unwrap();
    }
    // The 4 x 4 mm opening in the frustum top is enclosed: it needs a helix.
    assert!(helices > 0);
    assert_eq!(report["helical_entries"].as_u64().unwrap(), helices);
    let floor = report["target_z_mm"].as_f64().unwrap();
    for (i, cell) in stock.cells().iter().enumerate() {
        let p = stock.cell_center(i).unwrap();
        let height = history
            .iter()
            .filter(|m| m.kind != MoveKind::Rapid)
            .map(|m| exact_height(&m.curve, p))
            .fold(10f64, f64::min);
        assert!(
            cell.lower_mm <= height + 1e-6 && cell.upper_mm >= height - 1e-6,
            "{p:?}: {cell:?} misses independent exact height {height}"
        );
        // The reachable floor outside the base and inside the opening is cut.
        if p[0] < 5.
            || p[0] > 27.
            || p[1] < 5.
            || p[1] > 23.
            || (p[0] > 15.4 && p[0] < 16.6 && p[1] > 13.4 && p[1] < 14.6)
        {
            assert!(
                height <= floor + 1e-6,
                "uncut reachable floor {p:?}: {height}"
            );
        }
    }
    eprintln!(
        "engagement_front frustum: {} motions, nominal {nominal:.0} s",
        history.len()
    );
    assert!(nominal <= 2400., "nominal cycle {nominal} s");
}

/// #219: `Roughing` with adaptive clearing takes a `RampEntry`. The frustum's
/// 4 x 4 mm opening is enclosed, with 1.8 mm of travel for a 2 mm cutter:
/// each layer ramps into it back and forth instead of boring a helix, from
/// the floor the layer above left (the flutes span the whole depth, so the
/// layer's own top is the stock top, 2 to 6 mm of air higher).
#[test]
fn engagement_front_ramps_into_the_enclosed_opening_of_the_frustum() {
    use camlib::pocket::RampEntry;
    let angle = 10f64;
    let mut m = engagement_front_frustum();
    let Operation::Roughing(op) = &mut m.operations[0] else {
        panic!("fixture")
    };
    op.entry = PocketEntry::Ramp(RampEntry {
        angle_rad: angle.to_radians(),
        min_length_mm: Some(1.),
        feed_mm_min: Some(90.),
    });
    m.validate().unwrap();
    let tool = m.tools[0].prepare().unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut plan = OperationPlan::prepare_with_stock(
        m.operations[0].clone(),
        &m.tools[0],
        &stock,
        m.initial_position_mm,
        1,
        &|| false,
    )
    .unwrap();
    let mut history: Vec<camlib::motion::Motion> = Vec::new();
    let mut current = m.initial_position_mm;
    // The exact height every earlier fed motion left at `p`.
    let left = |history: &[camlib::motion::Motion], p: [f64; 2]| {
        history
            .iter()
            .filter(|old| old.kind != MoveKind::Rapid)
            .map(|old| exact_height(&old.curve, p))
            .fold(10f64, f64::min)
    };
    let (mut legs, mut air) = (0, 0f64);
    while let Some(motion) = plan.next_motion_with_stock(&stock, &|| false).unwrap() {
        assert!(history.len() < 60000, "unbounded engagement_front path");
        assert_eq!(motion.curve.start(), current);
        let start = current;
        current = motion.curve.end();
        assert!(!matches!(motion.curve, Curve::Helix { .. }), "{motion:?}");
        // Independent analytic part certificate (not the planner's region).
        certify_part(&motion.curve);
        let travel = (current[0] - start[0]).hypot(current[1] - start[1]);
        if current[2] < start[2] && travel == 0. {
            // A straight descent meets nothing under the cutter. (Just
            // inside its edge: a ramp comes down in the slot the layer
            // above cut along the same path, whose wall is the cutter's
            // own edge.)
            for radial in [0., 0.5, 0.98] {
                for i in 0..16 {
                    let a = TAU * i as f64 / 16.;
                    let p = [current[0] + radial * a.cos(), current[1] + radial * a.sin()];
                    if p[0] < 0. || p[0] > 32. || p[1] < 0. || p[1] > 28. {
                        continue;
                    }
                    let height = left(&history, p);
                    assert!(
                        height <= current[2] + 0.011,
                        "straight descent into material at {p:?}: {height} > {}",
                        current[2]
                    );
                }
            }
        } else if current[2] < start[2] {
            // A ramp leg: within the angle, at the entry's own feed, inside
            // the opening, and starting on the material under its path.
            assert!(
                (start[2] - current[2]) / travel <= angle.to_radians().tan() * (1. + 1e-9),
                "{motion:?}"
            );
            assert_eq!(motion.kind, MoveKind::Lead);
            assert_eq!(motion.feed_mm_min, Some(90.));
            let mut under = f64::NEG_INFINITY;
            for i in 0..=8 {
                let t = i as f64 / 8.;
                let p = [
                    start[0] + (current[0] - start[0]) * t,
                    start[1] + (current[1] - start[1]) * t,
                ];
                assert!(
                    p[0] > 15. && p[0] < 17. && p[1] > 13. && p[1] < 15.,
                    "{motion:?}"
                );
                under = under.max(left(&history, p));
            }
            air = air.max(start[2] - under);
            legs += 1;
        }
        let delta = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &|| false)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        history.push(motion);
    }
    plan.finish_report(&|| false).unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    assert_eq!(report["completed_layers"], 4);
    assert_eq!(report["helical_entries"], 0);
    let mut reported_legs = 0;
    for layer in report["layers"].as_array().unwrap() {
        let clearing = &layer["clearing"];
        assert_eq!(clearing["helical_entries"], 0);
        // One ramp into the opening per layer. What its pass leaves beside
        // the slot is cut from the slot at depth, not ramped into again
        // through the air the slot left.
        assert_eq!(clearing["ramp_entries"], 1, "{clearing}");
        assert!(clearing["slot_side_entries"].as_u64().unwrap() >= 1);
        // 1.8 to 2 mm at 10 degrees needs over 10 mm and the opening has
        // under 2.6: every ramp reverses.
        assert!(clearing["ramp_legs"].as_u64().unwrap() >= 4, "{clearing}");
        assert!(clearing["ramp_slot_mm"].as_f64().unwrap() < 2.6);
        reported_legs += clearing["ramp_legs"].as_u64().unwrap();
        assert!(clearing["max_ramp_angle_deg"].as_f64().unwrap() <= angle + 1e-9);
        let limit = clearing["engagement_limit_deg"].as_f64().unwrap();
        assert!(clearing["proven_max_engagement_deg"].as_f64().unwrap() <= limit);
    }
    assert_eq!(legs, reported_legs as usize);
    assert!(air <= 0.05, "a ramp starts {air} mm above the material");
    // The opening's reachable floor is cut.
    let floor = report["target_z_mm"].as_f64().unwrap();
    for (i, _) in stock.cells().iter().enumerate() {
        let p = stock.cell_center(i).unwrap();
        if p[0] > 15.4 && p[0] < 16.6 && p[1] > 13.4 && p[1] < 14.6 {
            let height = left(&history, p);
            assert!(
                height <= floor + 1e-6,
                "uncut opening floor {p:?}: {height}"
            );
        }
    }
    // Offset and raster layers refuse a ramp entry, saying where one is made.
    for clearing in [
        serde_json::json!({"kind": "raster", "stepover_mm": 0.8}),
        serde_json::json!({"kind": "offset", "stepover_mm": 0.8}),
    ] {
        let mut refused = m.clone();
        let Operation::Roughing(op) = &mut refused.operations[0] else {
            panic!("fixture")
        };
        op.clearing = serde_json::from_value(clearing).unwrap();
        let error = OperationPlan::prepare(
            refused.operations[0].clone(),
            &refused.tools[0],
            refused.stock.height().unwrap(),
            refused.initial_position_mm,
            1,
            &|| false,
        )
        .map(|_| ())
        .unwrap_err();
        for part in [
            "roughing rough: RampEntry is not supported here",
            "AdaptiveClearing",
        ] {
            assert!(error.contains(part), "{error}");
        }
    }
}

/// Plan the frustum to the end, committing each motion; returns the motions
/// and the nominal cycle (rapids at 6000 mm/min).
fn run_engagement_front(m: &StreamManifest) -> (Vec<camlib::motion::Motion>, f64) {
    let tool = m.tools[0].prepare().unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut plan = OperationPlan::prepare_with_stock(
        m.operations[0].clone(),
        &m.tools[0],
        &stock,
        m.initial_position_mm,
        1,
        &|| false,
    )
    .unwrap();
    let (mut history, mut nominal) = (Vec::new(), 0.);
    while let Some(motion) = plan.next_motion_with_stock(&stock, &|| false).unwrap() {
        assert!(history.len() < 60000, "unbounded engagement_front path");
        certify_part(&motion.curve);
        nominal += motion
            .curve
            .prepare(m.sweep.tolerance_mm)
            .unwrap()
            .length_mm()
            / motion.feed_mm_min.unwrap_or(6000.)
            * 60.;
        let delta = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &|| false)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        history.push(motion);
    }
    (history, nominal)
}

/// #167: adaptive roughing around a 100 x 60 x 20 mm block in a 130 x 90 mm
/// stock, one 7 mm layer, 10 mm cutter, 3 mm radial engagement. The old
/// default (disk_front) orbited through the air beside the stock (17,792
/// moves, 137 m of cut, 75 m of it wholly outside the stock footprint) and
/// engagement_front could not start without a helix.
#[test]
fn issue_167_open_block_is_cleared_from_the_stock_edge_within_its_engagement() {
    let base: StreamManifest = serde_json::from_str(include_str!(
        "../fixtures/operations/roughing-open-block.json"
    ))
    .unwrap();
    base.validate().unwrap();
    let footprint = [[-15., -15.], [115., 75.]];
    let (r, w) = (5., 3.);
    let limit = (1f64 - w / r).acos().to_degrees();
    let mut paths = vec![];
    for helix in [false, true] {
        let mut m = base.clone();
        let Operation::Roughing(op) = &mut m.operations[0] else {
            panic!("fixture")
        };
        // The issue's two entries: StockEdgeEntry(helix=None), StockEdgeEntry().
        assert!(matches!(op.entry, PocketEntry::StockEdge(_)));
        if helix {
            op.entry = PocketEntry::StockEdgeOrHelix(HelicalEntry {
                angle_rad: 3f64.to_radians(),
                ..HelicalEntry::default()
            });
        }
        let tool = m.tools[0].prepare().unwrap();
        let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let mut plan = OperationPlan::prepare_with_stock(
            m.operations[0].clone(),
            &m.tools[0],
            &stock,
            m.initial_position_mm,
            1,
            &|| false,
        )
        .unwrap();
        let mut leading = LeadingOracle::new(r, footprint);
        let mut motions = vec![];
        let (mut cut, mut link, mut plunges) = (0., 0., 0);
        while let Some(motion) = plan.next_motion_with_stock(&stock, &|| false).unwrap() {
            assert!(motions.len() < 1000, "unbounded path");
            let c = &motion.curve;
            assert!(
                !matches!(c, Curve::Helix { .. }) && motion.kind != MoveKind::Lead,
                "an open block needs no helix and has no confined material: {motion:?}"
            );
            let length = c.prepare(1e-5).unwrap().length_mm();
            match motion.kind {
                MoveKind::Plunge => {
                    // Every descent is in the air beside the stock, or over
                    // stock this operation has cut down to the layer.
                    let beside = box_distance(bounds(c, 0., 1.), footprint) > r;
                    if plunges == 0 {
                        assert!(beside, "the first entry is from beside the stock: {c:?}");
                    } else if !beside {
                        let p = [c.end()[0], c.end()[1]];
                        for i in 0..16 {
                            let a = TAU * i as f64 / 16.;
                            let q = [p[0] + 0.99 * r * a.cos(), p[1] + 0.99 * r * a.sin()];
                            let inside = q[0] > footprint[0][0]
                                && q[0] < footprint[1][0]
                                && q[1] > footprint[0][1]
                                && q[1] < footprint[1][1];
                            assert!(!inside || leading.cleared(q), "plunge into stock at {q:?}");
                        }
                    }
                    plunges += 1;
                }
                MoveKind::Cut | MoveKind::Link => {
                    // No cutting through the air beside the stock: the
                    // cutter is never more than a short approach away.
                    assert!(
                        box_distance(bounds(c, 0., 1.), footprint) < 1.5 * r,
                        "cut beside the stock: {c:?}"
                    );
                    if motion.kind == MoveKind::Cut {
                        cut += length;
                    } else {
                        link += length;
                    }
                    leading.feed(c, true);
                }
                _ => {}
            }
            let delta = stock
                .preview_sweep(&tool, &motion, 1., m.sweep, &|| false)
                .unwrap();
            stock.apply_delta(&delta).unwrap();
            motions.push(motion);
        }
        plan.finish_report(&|| false).unwrap();
        // Independent of the planner and its proof: the leading contact of
        // every cutting and link stroke, sampled (3 degrees of slack).
        assert!(leading.worst_deg <= limit + 3., "{}", leading.worst_deg);
        let report = serde_json::to_value(plan.report()).unwrap();
        let c = &report["layers"][0]["clearing"];
        assert_eq!(c["strategy"], "engagement_front");
        assert!((c["engagement_limit_deg"].as_f64().unwrap() - limit).abs() < 1e-9);
        let proven = c["proven_max_engagement_deg"].as_f64().unwrap();
        assert!(proven <= limit && proven > 0.9 * limit, "{proven}");
        assert!(c["proven_max_engagement_mm"].as_f64().unwrap() <= w);
        assert!(c["engagement_proved_strokes"].as_u64().unwrap() >= 50);
        assert_eq!(c["helical_entries"], 0);
        assert!(c.get("confined_strokes").is_none());
        // One pass per ring of material, not a hundred nibbles at a strip
        // (each pass starts at the upstream end of what it cuts), and no
        // more cutting than the material needs: 5,600 mm2 at up to 3 mm.
        let passes = c["passes"].as_u64().unwrap();
        assert!(passes <= 12, "{passes} passes");
        assert!(link < 250., "{link} mm of links");
        assert!((1860. ..2700.).contains(&cut), "{cut} mm of cut");
        assert!(motions.len() < 400, "{} motions", motions.len());
        // Everything outside the part and its allowance is down to the layer.
        let floor = report["target_z_mm"].as_f64().unwrap();
        for (i, cell) in stock.cells().iter().enumerate() {
            let p = stock.cell_center(i).unwrap();
            if p[0] < -1.5 || p[0] > 101.5 || p[1] < -1.5 || p[1] > 61.5 {
                assert!(cell.upper_mm <= floor + 1e-4, "uncut at {p:?}: {cell:?}");
            } else if p[0] > 0.5 && p[0] < 99.5 && p[1] > 0.5 && p[1] < 59.5 {
                assert_eq!(cell.lower_mm, 21., "the part's stock is untouched at {p:?}");
            }
        }
        paths.push(
            motions
                .iter()
                .map(|m| (m.kind, m.curve.clone()))
                .collect::<Vec<_>>(),
        );
    }
    // Allowing a helix changes nothing where none is needed.
    assert_eq!(paths[0], paths[1]);
    // The old default is refused, with the reason.
    let mut m = base.clone();
    let Operation::Roughing(op) = &mut m.operations[0] else {
        panic!("fixture")
    };
    let camlib::roughing::RoughingClearing::Adaptive { path_strategy, .. } = &mut op.clearing
    else {
        panic!("clearing")
    };
    *path_strategy = camlib::adaptive::AdaptivePathStrategy::DiskFront;
    let error = OperationPlan::prepare(
        m.operations[0].clone(),
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &AtomicBool::new(false),
    )
    .err()
    .unwrap();
    assert!(
        error.contains("disk_front") && error.contains("engagement_front"),
        "{error}"
    );
}

#[test]
fn engagement_front_rapid_descent_stops_above_proven_cut_stock() {
    let base = engagement_front_frustum();
    let mut v = serde_json::to_value(&base).unwrap();
    v["operations"][0]["rapid_descent_clearance_mm"] = 0.5.into();
    let m: StreamManifest = serde_json::from_value(v).unwrap();
    let (history, nominal) = run_engagement_front(&m);
    let (_, before) = run_engagement_front(&base);
    let mut deep = 0;
    for (i, motion) in history.iter().enumerate() {
        let (a, b) = (motion.curve.start(), motion.curve.end());
        if motion.kind != MoveKind::Rapid || b[2] >= a[2] || b[2] >= 12. {
            continue;
        }
        deep += 1;
        // Independent exact heights left by every earlier cut, over the
        // cutter's disk: the rapid stops at least the clearance above them.
        for radial in [0., 0.5, 1.] {
            for k in 0..16 {
                let angle = TAU * k as f64 / 16.;
                let p = [b[0] + radial * angle.cos(), b[1] + radial * angle.sin()];
                if p[0] < 0. || p[0] > 32. || p[1] < 0. || p[1] > 28. {
                    continue;
                }
                let height = history[..i]
                    .iter()
                    .filter(|old| old.kind != MoveKind::Rapid)
                    .map(|old| exact_height(&old.curve, p))
                    .fold(10f64, f64::min);
                assert!(
                    height <= b[2] - 0.5 + 0.02,
                    "rapid to {b:?} stops {} above material at {p:?}",
                    b[2] - height
                );
            }
        }
    }
    eprintln!(
        "rapid descents below the feed height: {deep}; nominal {before:.0} -> {nominal:.0} s"
    );
    assert!(deep > 0);
    assert!(nominal < before);
    // Opt-in for engagement_front only, and positive.
    let edit = |f: &dyn Fn(&mut serde_json::Value)| {
        let mut v = serde_json::to_value(&base).unwrap();
        f(&mut v["operations"][0]);
        let m = serde_json::from_value::<StreamManifest>(v).unwrap();
        let stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        OperationPlan::prepare_with_stock(
            m.operations[0].clone(),
            &m.tools[0],
            &stock,
            m.initial_position_mm,
            1,
            &|| false,
        )
        .err()
        .unwrap_or_default()
    };
    assert!(edit(&|op| op["rapid_descent_clearance_mm"] = 0.0.into()).contains("positive"));
    assert!(
        edit(&|op| {
            op["rapid_descent_clearance_mm"] = 0.5.into();
            op["clearing"] = serde_json::json!({"kind": "offset", "stepover_mm": 0.5});
        })
        .contains("engagement_front clearing only")
    );
}

/// Top of the material left by the earlier cuts (independent exact heights)
/// under the cutter's disk at `p`, or over the whole bore of radius `bore`
/// about `p`: 10 (the stock top) where nothing has cut.
fn material_top(history: &[camlib::motion::Motion], i: usize, p: [f64; 2], bore: f64) -> f64 {
    let mut top = f64::NEG_INFINITY;
    for radial in [0., 0.25, 0.5, 0.75, 1.] {
        for k in 0..24 {
            let angle = TAU * k as f64 / 24.;
            let d = radial * (1. + bore);
            let q = [p[0] + d * angle.cos(), p[1] + d * angle.sin()];
            if q[0] < 0. || q[0] > 32. || q[1] < 0. || q[1] > 28. {
                continue;
            }
            let height = history[..i]
                .iter()
                .filter(|old| old.kind != MoveKind::Rapid)
                .map(|old| exact_height(&old.curve, q))
                .fold(10f64, f64::min);
            top = top.max(height);
        }
    }
    top
}

#[test]
fn engagement_front_entries_feed_only_a_clearance_through_proven_air() {
    // With a rapid-descent clearance, an entry beside material plunges a few
    // stock cells back (where the cell bounds let the rapid come down) and
    // feeds across at depth, and a helix ramps only through material: none
    // feeds down through air already cleared by more than the clearance
    // (plus the 0.5 mm stock cells' bound slack).
    let base = engagement_front_frustum();
    let mut v = serde_json::to_value(&base).unwrap();
    v["operations"][0]["rapid_descent_clearance_mm"] = 0.5.into();
    let run =
        |v: &serde_json::Value| run_engagement_front(&serde_json::from_value(v.clone()).unwrap()).0;
    let history = run(&v);
    let (mut plunges, mut helices, mut air) = (0, 0, 0f64);
    for (i, motion) in history.iter().enumerate() {
        let (a, b) = (motion.curve.start(), motion.curve.end());
        if b[2] >= a[2] - 1e-9 || b[2] >= 10. {
            continue;
        }
        let top = match (motion.kind, &motion.curve) {
            (MoveKind::Plunge, Curve::Line { .. }) => {
                plunges += 1;
                material_top(&history, i, [b[0], b[1]], 0.)
            }
            (MoveKind::Lead, Curve::Helix { center, .. }) => {
                helices += 1;
                let bore = (a[0] - center[0]).hypot(a[1] - center[1]);
                material_top(&history, i, [center[0], center[1]], bore)
            }
            _ => continue,
        };
        air = air.max(a[2] - top.max(b[2]));
    }
    eprintln!("{plunges} plunges, {helices} helix turns; worst air feed {air:.3} mm");
    assert!(plunges > 0 && helices > 0);
    assert!(
        air <= 0.5 + 0.6,
        "an entry fed {air} mm through cleared air"
    );

    // ramp_clearance_mm: each helix starts that far above the material its
    // bore meets (proven conservatively, so possibly a cell bound higher).
    v["operations"][0]["ramp_clearance_mm"] = 0.3.into();
    let history = run(&v);
    let mut starts = 0;
    for (i, motion) in history.iter().enumerate() {
        let Curve::Helix { center, start, .. } = motion.curve else {
            continue;
        };
        if matches!(history[i - 1].curve, Curve::Helix { .. }) {
            continue;
        }
        starts += 1;
        let bore = (start[0] - center[0]).hypot(start[1] - center[1]);
        let above = start[2] - material_top(&history, i, [center[0], center[1]], bore);
        assert!(
            (0.3 - 1e-6..=0.3 + 0.6).contains(&above),
            "helix at {start:?} starts {above} mm above the material"
        );
    }
    assert!(starts > 0);
}

#[test]
fn engagement_front_refuses_orbits_empty_cut_omission_and_helix_only_entry() {
    let base = engagement_front_frustum();
    let prepare = |m: &StreamManifest| {
        let stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        OperationPlan::prepare_with_stock(
            m.operations[0].clone(),
            &m.tools[0],
            &stock,
            m.initial_position_mm,
            1,
            &|| false,
        )
        .err()
        .unwrap_or_default()
    };
    let edit = |f: &dyn Fn(&mut serde_json::Value)| {
        let mut v = serde_json::to_value(&base).unwrap();
        f(&mut v["operations"][0]);
        serde_json::from_value::<StreamManifest>(v).unwrap()
    };
    let orbit = edit(&|op| op["clearing"]["orbit_radius_mm"] = 0.5.into());
    assert!(prepare(&orbit).contains("does not use orbits"));
    let omit = edit(&|op| op["omit_empty_cuts"] = true.into());
    assert!(prepare(&omit).contains("omit_empty_cuts"));
    let helix = edit(&|op| op["entry"]["kind"] = "helix".into());
    assert!(prepare(&helix).contains("StockEdgeEntry"));
}

#[test]
fn roughing_preflight_rejects_enclosed_outside_only_entry_reach_and_work_before_motion() {
    let cancel = AtomicBool::new(false);
    for mode in 0..5 {
        let mut m = fixture(false);
        let Operation::Roughing(op) = &mut m.operations[0] else {
            panic!("fixture")
        };
        match mode {
            0 => op.entry = PocketEntry::StockEdge(HelicalEntry::default()),
            1 => op.stepdown_mm = 0.0002,
            2 => std::sync::Arc::make_mut(&mut op.target.triangles)[0][0][0] = 100.,
            // Four-mm flutes can now cut this deeper part in two-mm layers
            // (#122). One-mm flutes genuinely cannot span the first layer.
            3 => m.tools[0].cutter.cutting_length_mm = 1.,
            _ => op.radial_allowance_mm = -1.,
        }
        // Mode 1 (50,000 layers) is refused by the memory guard under an
        // explicit budget, not by a fixed layer cap.
        let budget = if mode == 1 { 1 << 20 } else { u64::MAX };
        assert!(
            camlib::resources::with_memory_budget(budget, || OperationPlan::prepare(
                m.operations[0].clone(),
                &m.tools[0],
                m.stock.height().unwrap(),
                m.initial_position_mm,
                1,
                &cancel
            ))
            .is_err(),
            "bad mode{mode}"
        );
    }
}

#[test]
fn radial_width_is_an_upper_limit_and_can_use_a_stricter_native_angle() {
    let mut m = fixture(true);
    let Operation::Roughing(op) = &mut m.operations[0] else {
        panic!("fixture")
    };
    let camlib::roughing::RoughingClearing::Adaptive {
        max_radial_engagement_mm,
        ..
    } = &mut op.clearing
    else {
        panic!("clearing")
    };
    *max_radial_engagement_mm = 1.;
    let plan = OperationPlan::prepare(
        m.operations[0].clone(),
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &AtomicBool::new(false),
    )
    .unwrap();
    let r = serde_json::to_value(plan.report()).unwrap();
    assert_eq!(r["max_radial_engagement_mm"], 1.);
    // The whole radius: a quarter of the circumference ahead of the cutter.
    assert_eq!(r["max_engagement_deg"], 90.);
}

struct ScheduledRun {
    report: serde_json::Value,
    motions: Vec<camlib::motion::Motion>,
    stock: Stock,
    nominal: f64,
}
fn scheduled_run(m: &StreamManifest) -> ScheduledRun {
    let cancel = AtomicBool::new(false);
    let mut plan = OperationPlan::prepare(
        m.operations[0].clone(),
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    let tool = m.tools[0].prepare().unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let (mut motions, mut nominal, mut position) = (vec![], 0., m.initial_position_mm);
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        assert_eq!(motion.id, motions.len() as u64 + 1);
        assert_eq!(motion.curve.start(), position);
        position = motion.curve.end();
        assert_eq!(motion.operation, "rough");
        assert_eq!(motion.tool, "t1");
        if motion.kind == MoveKind::Rapid {
            assert!(motion.curve.start()[2] >= 12. && position[2] >= 12.);
        }
        if motion.kind == MoveKind::Plunge && position[2] < m.stock.height().unwrap().max_mm[2] {
            assert!(
                box_distance(
                    bounds(&motion.curve, 0., 1.),
                    [
                        [
                            m.stock.height().unwrap().min_mm[0],
                            m.stock.height().unwrap().min_mm[1]
                        ],
                        [
                            m.stock.height().unwrap().max_mm[0],
                            m.stock.height().unwrap().max_mm[1]
                        ]
                    ]
                ) > 1.
            );
        }
        if motion.kind == MoveKind::Retract || matches!(motion.curve, Curve::Helix { .. }) {
            let before = serde_json::to_value(plan.report()).unwrap();
            assert!(
                plan.next_motion_with_stock(&stock, &AtomicBool::new(true))
                    .is_err()
            );
            assert!(plan.next_motion_with_stock(&stock, &cancel).is_err());
            assert_eq!(serde_json::to_value(plan.report()).unwrap(), before);
        }
        nominal += motion
            .curve
            .prepare(m.sweep.tolerance_mm)
            .unwrap()
            .length_mm()
            / motion.feed_mm_min.unwrap_or(6000.)
            * 60.;
        let d = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
            .unwrap();
        stock.apply_delta(&d).unwrap();
        motions.push(motion);
    }
    plan.finish_report(&cancel).unwrap();
    ScheduledRun {
        report: serde_json::to_value(plan.report()).unwrap(),
        motions,
        stock,
        nominal,
    }
}
fn set_routing(m: &mut StreamManifest) {
    let Operation::Roughing(op) = &mut m.operations[0] else {
        panic!("rough")
    };
    let camlib::roughing::RoughingClearing::Raster { routing, .. } = &mut op.clearing else {
        panic!("raster")
    };
    *routing = Some(camlib::pocket::RasterRouting::Economical);
}
fn rotate_curve(mut c: Curve, angle: f64) -> Curve {
    let rotate = |p: &mut [f64; 3]| {
        let (x, y) = (p[0] - 16., p[1] - 14.);
        p[0] = 16. + x * angle.cos() - y * angle.sin();
        p[1] = 14. + x * angle.sin() + y * angle.cos();
    };
    match &mut c {
        Curve::Line { start, end } => {
            rotate(start);
            rotate(end);
        }
        Curve::Arc {
            start, end, center, ..
        }
        | Curve::Helix {
            start, end, center, ..
        } => {
            rotate(start);
            rotate(end);
            rotate(center);
        }
    }
    c
}
fn cut_multiset(run: &ScheduledRun) -> Vec<String> {
    let mut result: Vec<_> = run
        .motions
        .iter()
        .filter(|m| m.kind == MoveKind::Cut)
        .map(|m| {
            let mut c = m.curve.clone();
            if let Curve::Line { start, end } = &mut c
                && *start > *end
            {
                std::mem::swap(start, end);
            }
            serde_json::to_string(&c).unwrap()
        })
        .collect();
    result.sort();
    result
}
fn boundary_loops_preserved(old: &ScheduledRun, new: &ScheduledRun) {
    for layer in old.report["layers"].as_array().unwrap() {
        let z = layer["tip_z_mm"].as_f64().unwrap();
        let raster = layer["clearing"]["raster_strokes"].as_u64().unwrap() as usize;
        let cuts: Vec<_> = old
            .motions
            .iter()
            .filter(|m| m.kind == MoveKind::Cut && (m.curve.start()[2] - z).abs() < 1e-9)
            .map(|m| m.curve.clone())
            .collect();
        let mut i = raster;
        while i < cuts.len() {
            let start = cuts[i].start();
            let mut ring = vec![];
            loop {
                let c = cuts[i].clone();
                let closed = c.end() == start;
                ring.push(c);
                i += 1;
                if closed {
                    break;
                }
            }
            let found = (0..new.motions.len()).any(|index| {
                (0..ring.len()).any(|rotate| {
                    (0..ring.len()).all(|k| {
                        new.motions.get(index + k).is_some_and(|m| {
                            m.kind == MoveKind::Cut && m.curve == ring[(rotate + k) % ring.len()]
                        })
                    })
                })
            });
            assert!(
                found,
                "lost original continuous oriented boundary loop at Z{z}"
            );
        }
    }
}
#[test]
fn economical_raster_preserves_cuts_whole_part_and_stock_with_half_the_navigation_and_feed_time() {
    for (angle, clockwise, floor) in [
        (0f64, true, 2.),
        (0., false, 2.),
        (0.61, true, 2.),
        (0.61, false, 0.),
    ] {
        let mut m = fixture(false);
        let Operation::Roughing(op) = &mut m.operations[0] else {
            panic!("rough")
        };
        op.clockwise = clockwise;
        op.floor_z_mm = floor;
        // The golden counts below were taken with the exact shadow.
        op.contour_tolerance_mm = Some(0.);
        if angle != 0. {
            for triangle in std::sync::Arc::make_mut(&mut op.target.triangles) {
                for p in triangle {
                    let (x, y) = (p[0] - 16., p[1] - 14.);
                    p[0] = 16. + x * angle.cos() - y * angle.sin();
                    p[1] = 14. + x * angle.sin() + y * angle.cos();
                }
            }
            m.stock.height_mut().unwrap().min_mm = [-4., -4., 0.];
            m.stock.height_mut().unwrap().max_mm = [36., 32., 10.];
            m.stock.height_mut().unwrap().cell_size_mm = 0.8;
        }
        let legacy = scheduled_run(&m);
        set_routing(&mut m);
        let optimized = scheduled_run(&m);
        assert_eq!(
            cut_multiset(&legacy),
            cut_multiset(&optimized),
            "lost or altered cutting stroke"
        );
        boundary_loops_preserved(&legacy, &optimized);
        assert_eq!(
            legacy.report["layers"].as_array().unwrap().len(),
            optimized.report["layers"].as_array().unwrap().len()
        );
        let nav = |r: &ScheduledRun| {
            r.report["layers"]
                .as_array()
                .unwrap()
                .iter()
                .map(|l| l["clearing"]["entry_navigation_moves"].as_u64().unwrap())
                .sum::<u64>()
        };
        eprintln!(
            "angle={angle},clockwise={clockwise},floor={floor}: nav {} -> {}, nominal {} -> {}, motions {} -> {}",
            nav(&legacy),
            nav(&optimized),
            legacy.nominal,
            optimized.nominal,
            legacy.motions.len(),
            optimized.motions.len()
        );
        if angle == 0. && clockwise {
            assert_eq!(nav(&legacy), 1463);
            // Both routing policies now reuse committed air during entry. Keep
            // the original frozen cycle target: improving the baseline too must
            // not silently replace the acceptance reference with a new one.
            let original_nominal = 8303.866024692436;
            assert!(legacy.nominal <= original_nominal + 0.001);
            assert!(nav(&optimized) * 2 <= nav(&legacy));
            assert!(optimized.nominal * 2. <= original_nominal);
            assert!(optimized.nominal < legacy.nominal);
        }
        for layer in optimized.report["layers"].as_array().unwrap() {
            assert_eq!(layer["clearing"]["routing"]["policy"], "economical");
            assert!(layer["clearing"]["routing"]["work"].as_u64().unwrap() <= 20_000_000);
            assert!(layer["clearing"]["routing"]["candidates"].as_u64().unwrap() <= 2_000_000);
        }
        for motion in &optimized.motions {
            certify_part(&rotate_curve(motion.curve.clone(), -angle));
        }
        for (i, cell) in optimized.stock.cells().iter().enumerate() {
            let p = optimized.stock.cell_center(i).unwrap();
            let height = optimized
                .motions
                .iter()
                .map(|m| exact_height(&m.curve, p))
                .fold(10f64, f64::min);
            let old = legacy
                .motions
                .iter()
                .map(|m| exact_height(&m.curve, p))
                .fold(10f64, f64::min);
            assert!(
                height <= old + 1e-6,
                "lost previous stock coverage at{p:?}: old{old},new{height}"
            );
            assert!(cell.lower_mm <= height + 1e-6 && cell.upper_mm >= height - 1e-6);
        }
    }
}

#[test]
fn offset_roughing_preserves_whole_part_and_every_stock_cell_with_complete_reachable_core() {
    for (angle, clockwise, floor) in [
        (0f64, true, 2.),
        (0., false, 2.),
        (0.61, true, 2.),
        (0.61, false, 0.),
    ] {
        let mut manifest = fixture(false);
        let Operation::Roughing(op) = &mut manifest.operations[0] else {
            panic!("rough")
        };
        op.clearing = camlib::roughing::RoughingClearing::Offset {
            stepover_mm: 0.8,
            coverage_margin_mm: 0.01,
        };
        op.clockwise = clockwise;
        op.floor_z_mm = floor;
        if angle != 0. {
            for triangle in std::sync::Arc::make_mut(&mut op.target.triangles) {
                for p in triangle {
                    let (x, y) = (p[0] - 16., p[1] - 14.);
                    p[0] = 16. + x * angle.cos() - y * angle.sin();
                    p[1] = 14. + x * angle.sin() + y * angle.cos();
                }
            }
            manifest.stock.height_mut().unwrap().min_mm = [-4., -4., 0.];
            manifest.stock.height_mut().unwrap().max_mm = [36., 32., 10.];
            manifest.stock.height_mut().unwrap().cell_size_mm = 0.8;
        }
        let run = scheduled_run(&manifest);
        assert_eq!(run.report["clearing"], "offset");
        assert_eq!(
            run.report["completed_layers"],
            if floor == 0. { 5 } else { 4 }
        );
        let target = run.report["target_z_mm"].as_f64().unwrap();
        for layer in run.report["layers"].as_array().unwrap() {
            let r = &layer["clearing"];
            assert_eq!(r["raster_strokes"], 0);
            assert_eq!(r["boundary_strokes"], r["xy_strokes"]);
            assert_eq!(r["offset"]["applied_spacing_mm"], 0.8);
            assert!(r["offset"]["coverage_tiles"].as_u64().unwrap() > 0);
            assert!(r["offset"]["coverage_max_bound_mm"].as_f64().unwrap() <= 0.99);
        }
        for motion in &run.motions {
            certify_part(&rotate_curve(motion.curve.clone(), -angle));
            if let Curve::Helix { sweep_rad, .. } = motion.curve
                && !matches!(run.motions[motion.id as usize].curve, Curve::Helix { .. })
            {
                assert_eq!(sweep_rad < 0., clockwise);
                let next = &run.motions[motion.id as usize];
                // Shared Pocket entry attributes its real floor circle as Lead.
                assert_eq!(next.kind, MoveKind::Lead);
                assert!(matches!(next.curve, Curve::Arc { .. }));
                assert!((circle(&next.curve).unwrap().1.abs() - TAU).abs() < 1e-9);
                assert_eq!(next.curve.start()[2], next.curve.end()[2]);
            }
        }
        for (i, cell) in run.stock.cells().iter().enumerate() {
            let p = run.stock.cell_center(i).unwrap();
            let height = run
                .motions
                .iter()
                .map(|m| exact_height(&m.curve, p))
                .fold(10f64, f64::min);
            assert!(
                cell.lower_mm <= height + 1e-6 && cell.upper_mm >= height - 1e-6,
                "offset stock bound misses exact swept height at{p:?}: {cell:?}/{height}"
            );
            let x = p[0] - 16.;
            let y = p[1] - 14.;
            let q = [
                16. + x * angle.cos() + y * angle.sin(),
                14. - x * angle.sin() + y * angle.cos(),
            ];
            // Strictly inside cutter-accessible core, independent of planner
            // regions/offsets. At deeper floor the base also remains protected.
            if q[0] < 2.8
                || q[0] > 29.2
                || q[1] < 2.8
                || q[1] > 25.2
                || (floor == 2. && q[0] > 15.4 && q[0] < 16.6 && q[1] > 13.4 && q[1] < 14.6)
            {
                assert!(
                    height <= target + 1e-6,
                    "offset missed reachable core{p:?}: {height}"
                );
            }
        }
        if angle == 0. && clockwise {
            // Different sweep partitions can produce different conservative
            // cell bounds. Check actual centre contact independently, rather
            // than treating interval-hash equality as coverage equivalence.
            let mut raster_manifest = manifest.clone();
            let Operation::Roughing(op) = &mut raster_manifest.operations[0] else {
                panic!("rough")
            };
            op.clearing = camlib::roughing::RoughingClearing::Raster {
                stepover_mm: 0.8,
                routing: Some(camlib::pocket::RasterRouting::Economical),
            };
            let raster = scheduled_run(&raster_manifest);
            let mut changed_bounds = 0;
            let mut changed_contacts = 0;
            for (i, cell) in run.stock.cells().iter().enumerate() {
                let p = run.stock.cell_center(i).unwrap();
                let old = raster
                    .motions
                    .iter()
                    .map(|m| exact_height(&m.curve, p))
                    .fold(10f64, f64::min);
                let new = run
                    .motions
                    .iter()
                    .map(|m| exact_height(&m.curve, p))
                    .fold(10f64, f64::min);
                assert!(
                    new <= old + 1e-6,
                    "offset lost raster centre coverage at{p:?}: old{old},new{new}"
                );
                changed_contacts += usize::from((new - old).abs() > 1e-6);
                changed_bounds += usize::from(
                    cell.lower_mm != raster.stock.cells()[i].lower_mm
                        || cell.upper_mm != raster.stock.cells()[i].upper_mm,
                );
            }
            eprintln!(
                "offset vs current economical raster: {changed_bounds} different conservative cell bounds; {changed_contacts} different exact centre contacts"
            );
        }
        eprintln!(
            "offset angle={angle}, clockwise={clockwise}, floor={floor}: {} motions, {} nominal seconds",
            run.motions.len(),
            run.nominal
        );
    }
}

#[test]
fn facing_admits_authored_roughing_depths_and_short_flutes_from_committed_whole_cell_bounds() {
    let cancel = AtomicBool::new(false);
    for (adaptive, clockwise) in [(false, true), (false, false), (true, true)] {
        let mut m = fixture(adaptive);
        m.stock.height_mut().unwrap().max_mm[2] = 12.;
        m.stock.height_mut().unwrap().cell_size_mm = 0.8;
        m.initial_position_mm[2] = 14.;
        m.tools[0].cutter.cutting_length_mm = 8.;
        let Operation::Roughing(op) = &mut m.operations[0] else {
            panic!("rough")
        };
        op.top_z_mm = 10.;
        op.clearance_z_mm = 14.;
        op.clockwise = clockwise;
        if !adaptive {
            op.clearing = camlib::roughing::RoughingClearing::Offset {
                stepover_mm: 0.8,
                coverage_margin_mm: 0.01,
            };
        }
        let prepare = |position, first_id| {
            OperationPlan::prepare(
                m.operations[0].clone(),
                &m.tools[0],
                m.stock.height().unwrap(),
                position,
                first_id,
                &cancel,
            )
            .unwrap()
        };
        let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let mut plan = prepare(m.initial_position_mm, 1);
        let pending = serde_json::to_value(plan.report()).unwrap();
        assert!(
            plan.next_motion_with_stock(&stock, &cancel)
                .unwrap_err()
                .contains("committed stock upper")
        );
        assert_eq!(serde_json::to_value(plan.report()).unwrap(), pending);

        let face: StreamManifest =
            serde_json::from_str(include_str!("../fixtures/operations/facing.json")).unwrap();
        let Operation::Facing(mut face_op) = face.operations[0].clone() else {
            panic!("face")
        };
        face_op.floor_z_mm = 10.;
        face_op.stepdown_mm = 2.;
        face_op.clearance_z_mm = 14.;
        let mut face_plan = OperationPlan::prepare(
            Operation::Facing(face_op),
            &face.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &cancel,
        )
        .unwrap();
        let face_tool = face.tools[0].prepare().unwrap();
        let mut position = m.initial_position_mm;
        let mut count = 0;
        while let Some(motion) = face_plan.next_motion_with_stock(&stock, &cancel).unwrap() {
            assert_eq!(motion.curve.start(), position);
            position = motion.curve.end();
            let delta = stock
                .preview_sweep(&face_tool, &motion, 1., m.sweep, &cancel)
                .unwrap();
            stock.apply_delta(&delta).unwrap();
            count += 1;
        }
        face_plan.finish_report(&cancel).unwrap();
        assert!(stock.cells().iter().all(|c| c.upper_mm < 10.00016));
        let admitted_generation = stock.generation();
        let mut plan = prepare(position, count + 1);
        let pending = serde_json::to_value(plan.report()).unwrap();
        let calls = std::cell::Cell::new(0);
        let cancel_scan = || {
            let n = calls.get() + 1;
            calls.set(n);
            n > 2
        };
        assert!(
            plan.next_motion_with_stock(&stock, &cancel_scan)
                .unwrap_err()
                .contains("cancelled")
        );
        assert_eq!(serde_json::to_value(plan.report()).unwrap(), pending);
        let mut uncertain = stock.snapshot();
        uncertain.cells.last_mut().unwrap().upper_mm = 12.;
        let uncertain = Stock::from_snapshot(uncertain).unwrap();
        assert!(
            plan.next_motion_with_stock(&uncertain, &cancel)
                .unwrap_err()
                .contains("remains high or uncertain")
        );
        assert_eq!(serde_json::to_value(plan.report()).unwrap(), pending);

        let tool = m.tools[0].prepare().unwrap();
        let (mut curves, mut reach_work) = (vec![], 0);
        while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
            assert_eq!(motion.curve.start(), position);
            assert_eq!(motion.id, count + curves.len() as u64 + 1);
            certify_part(&motion.curve);
            if matches!(motion.curve, Curve::Helix { .. }) {
                assert!(motion.curve.start()[2] <= 10.00016);
            }
            let before = serde_json::to_value(plan.report()).unwrap();
            assert!(
                plan.next_motion_with_stock(&stock, &cancel).is_err(),
                "uncommitted admitted layer accepted"
            );
            assert_eq!(serde_json::to_value(plan.report()).unwrap(), before);
            let delta = stock
                .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
                .unwrap();
            reach_work += delta.stats().reach_cell_visits;
            stock.apply_delta(&delta).unwrap();
            position = motion.curve.end();
            curves.push(motion.curve);
        }
        plan.finish_report(&cancel).unwrap();
        assert!(
            reach_work > 0,
            "original-stock reach fast path hid short-flute case"
        );
        let r = serde_json::to_value(plan.report()).unwrap();
        let h = &r["height_schedule"];
        assert_eq!(h["authored_top_z_mm"], 10.);
        assert_eq!(h["stock_top_check"]["generation"], admitted_generation);
        assert_eq!(h["stock_top_check"]["checked_cells"], stock.cells().len());
        assert_eq!(h["skipped_air_depths"], 0);
        let levels: Vec<_> = r["layers"]
            .as_array()
            .unwrap()
            .iter()
            .map(|l| l["tip_z_mm"].as_f64().unwrap())
            .collect();
        assert_eq!(&levels[..3], &[8., 6., 4.]);
        let floor = r["target_z_mm"].as_f64().unwrap();
        assert_eq!(levels[3], floor);
        for (i, cell) in stock.cells().iter().enumerate() {
            let p = stock.cell_center(i).unwrap();
            let height = curves
                .iter()
                .map(|c| exact_height(c, p))
                .fold(10f64, f64::min);
            assert!(
                cell.lower_mm <= height + 1e-6 && cell.upper_mm >= height - 1e-6,
                "admitted-stock height {p:?}: {height}/{cell:?}"
            );
            if p[0] < 5.
                || p[0] > 27.
                || p[1] < 5.
                || p[1] > 23.
                || (p[0] > 15.4 && p[0] < 16.6 && p[1] > 13.4 && p[1] < 14.6)
            {
                assert!(height <= floor + 1e-6, "admitted-stock missed core {p:?}");
            }
        }
    }
}

#[test]
fn authored_air_depths_are_explicit_and_clearing_budget_is_enforced_before_motion() {
    let mut m = fixture(false);
    let Operation::Roughing(op) = &mut m.operations[0] else {
        panic!("rough")
    };
    op.top_z_mm = 12.;
    op.clearance_z_mm = 14.;
    op.clearing_tolerance_mm = Some(0.01);
    m.initial_position_mm[2] = 14.;
    let plan = OperationPlan::prepare(
        m.operations[0].clone(),
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &AtomicBool::new(false),
    )
    .unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    assert_eq!(report["height_schedule"]["skipped_air_depths"], 1);
    assert_eq!(report["layers"][0]["tip_z_mm"], 8.);
    assert!(report["maximum_construction_error_mm"].as_f64().unwrap() <= 0.01);
    let Operation::Roughing(op) = &mut m.operations[0] else {
        unreachable!()
    };
    op.clearing_tolerance_mm = Some(0.00001);
    assert!(
        OperationPlan::prepare(
            m.operations[0].clone(),
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &AtomicBool::new(false)
        )
        .err()
        .unwrap()
        .contains("clearing_tolerance_mm")
    );
}

#[test]
fn stock_aware_preparation_skips_cleared_authored_depths_and_binds_its_real_generation() {
    let mut m = fixture(false);
    m.stock.height_mut().unwrap().max_mm[2] = 12.;
    m.stock.height_mut().unwrap().cell_size_mm = 0.8;
    m.initial_position_mm[2] = 14.;
    m.tools[0].cutter.cutting_length_mm = 8.;
    let Operation::Roughing(op) = &mut m.operations[0] else {
        panic!("rough")
    };
    op.top_z_mm = 12.;
    op.clearance_z_mm = 14.;
    op.require_part_at_first_depth = true;
    op.clearing = camlib::roughing::RoughingClearing::Offset {
        stepover_mm: 0.8,
        coverage_margin_mm: 0.01,
    };
    let mut snapshot = Stock::new(m.stock.height().unwrap().clone())
        .unwrap()
        .snapshot();
    for c in &mut snapshot.cells {
        c.lower_mm = 10.;
        c.upper_mm = 10.;
    }
    snapshot.generation = 31;
    let stock = Stock::from_snapshot(snapshot.clone()).unwrap();
    let cancel = AtomicBool::new(false);
    let mut plan = OperationPlan::prepare_with_stock(
        m.operations[0].clone(),
        &m.tools[0],
        &stock,
        m.initial_position_mm,
        32,
        &cancel,
    )
    .unwrap();
    let r = serde_json::to_value(plan.report()).unwrap();
    assert_eq!(r["height_schedule"]["authored_top_z_mm"], 12.);
    assert_eq!(r["height_schedule"]["entry_top_z_mm"], 10.);
    assert_eq!(r["height_schedule"]["skipped_air_depths"], 1);
    assert_eq!(r["height_schedule"]["stock_top_check"]["generation"], 31);
    assert_eq!(r["layers"][0]["tip_z_mm"], 8.);
    let mut foreign = snapshot.clone();
    foreign.generation += 1;
    assert!(
        plan.next_motion_with_stock(&Stock::from_snapshot(foreign).unwrap(), &cancel)
            .is_err()
    );
    assert_eq!(serde_json::to_value(plan.report()).unwrap(), r);
    assert_eq!(
        plan.next_motion_with_stock(&stock, &cancel)
            .unwrap()
            .unwrap()
            .id,
        32
    );
    let Operation::Roughing(op) = &mut m.operations[0] else {
        unreachable!()
    };
    op.stepdown_mm = 1.;
    let fresh = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let error = OperationPlan::prepare_with_stock(
        m.operations[0].clone(),
        &m.tools[0],
        &fresh,
        m.initial_position_mm,
        1,
        &cancel,
    )
    .err()
    .unwrap();
    assert!(error.contains("top_mm") && error.contains("above retained part"));
    assert_eq!(stock.snapshot(), snapshot);
}

// Independent analytic contact oracle, restricted by its own full-curve bounds.
// The restriction reduces test work only; every affected cell centre is visited.
fn exact_centres(run: &ScheduledRun) -> Vec<f64> {
    exact_centres_with(&run.stock, run.motions.iter().map(|m| (&m.curve, 1.)))
}
fn exact_centres_with<'a>(
    stock: &Stock,
    curves: impl Iterator<Item = (&'a Curve, f64)>,
) -> Vec<f64> {
    let [nx, ny] = stock.dimensions();
    let step = stock.cell_widths_mm();
    let lo = stock.spec().min_mm;
    let mut heights = vec![10f64; nx * ny];
    for (curve, radius) in curves {
        let b = bounds(curve, 0., 1.);
        let a: [usize; 2] = std::array::from_fn(|k| {
            (((b[0][k] - (radius + 0.000001) - lo[k]) / step[k] - 0.5)
                .ceil()
                .max(0.) as usize)
                .min([nx, ny][k])
        });
        let z: [usize; 2] = std::array::from_fn(|k| {
            (((b[1][k] + (radius + 0.000001) - lo[k]) / step[k] - 0.5)
                .floor()
                .max(-1.)
                + 1.)
                .max(0.) as usize
        });
        for y in a[1]..z[1].min(ny) {
            for x in a[0]..z[0].min(nx) {
                let i = y * nx + x;
                heights[i] = heights[i].min(exact_height_for(
                    curve,
                    stock.cell_center(i).unwrap(),
                    radius,
                    10.,
                ));
            }
        }
    }
    heights
}

#[test]
fn empty_cut_policy_preserves_material_pose_entries_and_exposure_across_strategies() {
    use camlib::roughing::RoughingClearing;
    for (strategy, clockwise, angle) in [
        ("raster", true, 0f64),
        ("offset", true, 0.),
        ("offset", false, 0.61),
    ] {
        // Adaptive roughing (engagement_front) plans every pass from the
        // committed stock, so it has no empty cuts to omit and refuses the
        // option (engagement_front_refuses_...).
        let mut m = fixture(false);
        let Operation::Roughing(op) = &mut m.operations[0] else {
            panic!("rough")
        };
        op.clockwise = clockwise;
        if strategy == "raster" {
            op.clearing = RoughingClearing::Raster {
                stepover_mm: 0.8,
                routing: Some(camlib::pocket::RasterRouting::Economical),
            };
        } else if strategy == "offset" {
            op.clearing = RoughingClearing::Offset {
                stepover_mm: 0.8,
                coverage_margin_mm: 0.01,
            };
        }
        if angle != 0. {
            for tri in std::sync::Arc::make_mut(&mut op.target.triangles) {
                for p in tri {
                    let (x, y) = (p[0] - 16., p[1] - 14.);
                    p[0] = 16. + x * angle.cos() - y * angle.sin();
                    p[1] = 14. + x * angle.sin() + y * angle.cos();
                }
            }
            m.stock.height_mut().unwrap().min_mm = [-4., -4., 0.];
            m.stock.height_mut().unwrap().max_mm = [36., 32., 10.];
            m.stock.height_mut().unwrap().cell_size_mm = 0.8;
        }
        let old = scheduled_run(&m);
        assert!(
            old.report["layers"][0]["clearing"]
                .get("empty_cuts")
                .is_none()
        );
        let Operation::Roughing(op) = &mut m.operations[0] else {
            unreachable!()
        };
        op.omit_empty_cuts = true;
        let new = scheduled_run(&m);
        let cuts = |run: &ScheduledRun| {
            run.motions
                .iter()
                .filter(|x| x.kind == MoveKind::Cut)
                .count()
        };
        assert!(
            cuts(&new) < cuts(&old),
            "no empty cuts omitted for{strategy}"
        );
        assert!(
            new.nominal < old.nominal,
            "worse path metric {strategy}:{} vs{}",
            new.nominal,
            old.nominal
        );
        let mut skipped = 0;
        for layer in new.report["layers"].as_array().unwrap() {
            let proof = &layer["clearing"]["empty_cuts"];
            skipped += proof["outside_original_stock"].as_u64().unwrap()
                + proof["empty_committed_stock"].as_u64().unwrap();
            assert!(!proof["query_work_exhausted"].as_bool().unwrap());
            assert!(proof["query_work"].as_u64().unwrap() <= 2_000_000);
        }
        assert!(skipped > 0);
        let (a, b) = (exact_centres(&old), exact_centres(&new));
        let floor = new.report["target_z_mm"].as_f64().unwrap();
        for (i, cell) in new.stock.cells().iter().enumerate() {
            assert!(
                b[i] <= a[i] + 1e-6,
                "lost actual centre coverage {strategy} cell{i}:{} ->{}",
                a[i],
                b[i]
            );
            assert!(
                cell.lower_mm <= b[i] + 1e-6 && cell.upper_mm >= b[i] - 1e-6,
                "cell{i} misses exact height"
            );
            let p = new.stock.cell_center(i).unwrap();
            let (x, y) = (p[0] - 16., p[1] - 14.);
            let q = [
                16. + x * angle.cos() + y * angle.sin(),
                14. - x * angle.sin() + y * angle.cos(),
            ];
            if q[0] < 2.8
                || q[0] > 29.2
                || q[1] < 2.8
                || q[1] > 25.2
                || (q[0] > 15.4 && q[0] < 16.6 && q[1] > 13.4 && q[1] < 14.6)
            {
                assert!(b[i] <= floor + 1e-6, "missed accessible core");
            }
        }
        for (i, motion) in new.motions.iter().enumerate() {
            certify_part(&rotate_curve(motion.curve.clone(), -angle));
            if let Curve::Helix { sweep_rad, .. } = motion.curve
                && !matches!(new.motions[i + 1].curve, Curve::Helix { .. })
            {
                assert_eq!(sweep_rad < 0., clockwise);
                let next = &new.motions[i + 1];
                assert_eq!(next.kind, MoveKind::Lead);
                assert!((circle(&next.curve).unwrap().1.abs() - TAU).abs() < 1e-9);
                assert_eq!(next.curve.start()[2], next.curve.end()[2]);
            }
        }
        eprintln!(
            "{strategy}/{clockwise}/{angle}: {} ->{} moves, {} ->{} nominal seconds, {skipped} empty guides/orbits",
            old.motions.len(),
            new.motions.len(),
            old.nominal,
            new.nominal
        );
    }
}

#[test]
fn actual_cadquery_empty_cutting_changes_only_bounds_not_reachable_material() {
    let input: StreamManifest =
        serde_json::from_str(include_str!("../fixtures/regressions/imported-body.json")).unwrap();
    let cancel = AtomicBool::new(false);
    let run = |enabled: bool| {
        let mut m = input.clone();
        for op in &mut m.operations {
            if let Operation::Roughing(op) = op {
                op.omit_empty_cuts = enabled;
                // The golden counts below were taken with the exact shadow.
                op.contour_tolerance_mm = Some(0.);
            }
        }
        let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let mut actual = m.initial_position_mm;
        let mut motions = vec![];
        for op in &m.operations {
            let tool = m.tools.iter().find(|t| t.id == op.tool()).unwrap();
            let assembly = tool.prepare().unwrap();
            let mut plan = OperationPlan::prepare_with_stock(
                op.clone(),
                tool,
                &stock,
                actual,
                motions.len() as u64 + 1,
                &cancel,
            )
            .unwrap();
            while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
                assert_eq!(motion.id, motions.len() as u64 + 1);
                assert_eq!(motion.curve.start(), actual);
                if motion.operation == "rough" {
                    // Exact retained solid: [0,24]x[0,16] up to Z5, with
                    // [4,14]x[4,12] opening down to Z3. Whole-interval XY
                    // certificate includes real radial .1 and axial .2 stock.
                    let mut stack = vec![(0., 1., 0)];
                    while let Some((a, b, depth)) = stack.pop() {
                        let z = position(&motion.curve, a)[2].min(position(&motion.curve, b)[2]);
                        let q = bounds(&motion.curve, a, b);
                        if z >= 5.2
                            || box_distance(q, [[0., 0.], [24., 16.]]) >= 2.1
                            || (q[0][0] >= 6.1
                                && q[1][0] <= 11.9
                                && q[0][1] >= 6.1
                                && q[1][1] <= 9.9)
                        {
                            continue;
                        }
                        assert!(
                            depth < 28,
                            "cadquery retained prism contact {q:?} at {a}..{b}"
                        );
                        let mid = (a + b) / 2.;
                        stack.push((a, mid, depth + 1));
                        stack.push((mid, b, depth + 1));
                    }
                }
                let delta = stock
                    .preview_sweep(&assembly, &motion, 1., m.sweep, &cancel)
                    .unwrap();
                stock.apply_delta(&delta).unwrap();
                actual = motion.curve.end();
                motions.push(motion);
            }
            plan.finish_report(&cancel).unwrap();
        }
        (stock, motions)
    };
    let (old, a) = run(false);
    let (new, b) = run(true);
    // Golden counts from before entry helices were split one turn per motion.
    let ca = continued_entry_turns(a.iter().map(|m| &m.curve));
    let cb = continued_entry_turns(b.iter().map(|m| &m.curve));
    // Golden counts from before entry helices were split one turn per motion.
    assert_eq!((a.len() - ca, b.len() - cb), (282, 54));
    let mut changed = 0;
    for (i, cell) in new.cells().iter().enumerate() {
        let p = new.cell_center(i).unwrap();
        let x = a
            .iter()
            .map(|m| exact_height_for(&m.curve, p, 2., 6.))
            .fold(6f64, f64::min);
        let y = b
            .iter()
            .map(|m| exact_height_for(&m.curve, p, 2., 6.))
            .fold(6f64, f64::min);
        assert!(
            y <= x + 1e-6,
            "lost actual cadquery coverage cell{i}: {x}->{y}"
        );
        assert!(
            cell.lower_mm <= y + 1e-6 && cell.upper_mm >= y - 1e-6,
            "new cell{i} misses exact height"
        );
        let prev = &old.cells()[i];
        assert!(
            prev.lower_mm <= x + 1e-6 && prev.upper_mm >= x - 1e-6,
            "old cell{i} misses exact height"
        );
        if p[0] > 6.2 && p[0] < 11.8 && p[1] > 6.2 && p[1] < 9.8 {
            assert!(y < 3.201, "missed reachable opening");
        }
        if p[0] < 4. || p[0] > 14. || p[1] < 4. || p[1] > 12. {
            assert!(y >= 5. - 1e-6, "gouged retained top");
        }
        changed += usize::from(prev != cell);
    }
    eprintln!(
        "Actual cadquery: {changed} conservative cell intervals differ; every cell contains exact sweep height and no exact centre coverage lost; whole retained prism certified"
    );
    assert!(changed > 0);
}

// Independent setup: a retained top at Z2 is safely below the facing floor Z3.
// The actual whole-stock Facing pass, not a fabricated empty snapshot, prepares
// committed material for subsequent Roughing authored top5/floor3.
fn already_faced_roughing(adaptive: bool) -> (StreamManifest, Stock, [f64; 3], u64) {
    let cancel = AtomicBool::new(false);
    let mut m = fixture(adaptive);
    m.stock.height_mut().unwrap().max_mm = [12., 8., 6.];
    m.initial_position_mm = [0., 0., 8.];
    let Operation::Roughing(op) = &mut m.operations[0] else {
        panic!("rough")
    };
    op.target = mesh(rect(2., 2., 10., 6., 2.), 0.);
    op.top_z_mm = 5.;
    op.floor_z_mm = 3.;
    op.stepdown_mm = 1.;
    op.bottom_allowance_mm = 0.;
    op.clearance_z_mm = 8.;
    if !adaptive {
        op.clearing = camlib::roughing::RoughingClearing::Offset {
            stepover_mm: 0.8,
            coverage_margin_mm: 0.01,
        };
    }
    let f: StreamManifest =
        serde_json::from_str(include_str!("../fixtures/operations/facing.json")).unwrap();
    let Operation::Facing(mut face) = f.operations[0].clone() else {
        panic!("face")
    };
    face.floor_z_mm = 3.;
    face.stepdown_mm = 1.;
    face.clearance_z_mm = 8.;
    let mut plan = OperationPlan::prepare(
        Operation::Facing(face),
        &f.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    let tool = f.tools[0].prepare().unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut position = m.initial_position_mm;
    let mut next_id = 1;
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        assert_eq!(motion.id, next_id);
        assert_eq!(motion.curve.start(), position);
        position = motion.curve.end();
        next_id += 1;
        let delta = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
    }
    plan.finish_report(&cancel).unwrap();
    assert!(
        stock
            .cells()
            .iter()
            .all(|c| c.lower_mm >= 3. - 1e-4 && c.upper_mm <= 3. + 1e-4)
    );
    (m, stock, position, next_id)
}

#[test]
fn already_cleared_roughing_finishes_without_motion_controls_or_stock_changes() {
    for strategy in ["offset", "raster", "adaptive"] {
        let (mut m, stock, position, next_id) = already_faced_roughing(strategy == "adaptive");
        if strategy == "raster" {
            let Operation::Roughing(op) = &mut m.operations[0] else {
                unreachable!()
            };
            op.clearing = camlib::roughing::RoughingClearing::Raster {
                stepover_mm: 0.8,
                routing: Some(camlib::pocket::RasterRouting::Economical),
            };
        }
        let before = serde_json::to_value(stock.snapshot()).unwrap();
        let mut plan = OperationPlan::prepare_with_stock(
            m.operations[0].clone(),
            &m.tools[0],
            &stock,
            position,
            next_id,
            &|| false,
        )
        .unwrap_or_else(|e| {
            panic!("already-cleared roughing must complete as a verified no-op: {e}")
        });
        assert!(plan.prefix_events().is_empty());
        assert!(plan.suffix_events().is_empty());
        let report = serde_json::to_value(plan.report()).unwrap();
        assert_eq!(report["no_op"]["reason"], "committed_stock_below_cut_floor");
        assert_eq!(report["no_op"]["generation"], stock.generation());
        assert_eq!(report["no_op"]["checked_cells"], stock.cells().len());
        assert!(
            report["no_op"]["maximum_upper_z_mm"].as_f64().unwrap()
                <= report["target_z_mm"].as_f64().unwrap()
        );
        assert!(
            plan.next_motion_with_stock(&stock, &|| false)
                .unwrap()
                .is_none()
        );
        plan.finish_report(&|| false).unwrap();
        let report = serde_json::to_value(plan.report()).unwrap();
        assert_eq!(report["motions"], 0);
        assert_eq!(report["layers"], serde_json::json!([]));
        assert_eq!(report["geometric_residual_status"], "unknown");
        assert_eq!(serde_json::to_value(stock.snapshot()).unwrap(), before);
    }
}

#[test]
fn empty_roughing_proof_is_strict_and_cannot_cross_stock_instances_or_generations() {
    use camlib::motion::{Motion, MoveKind};
    let (m, stock, position, next_id) = already_faced_roughing(false);
    let prepare = |s: &Stock| {
        OperationPlan::prepare_with_stock(
            m.operations[0].clone(),
            &m.tools[0],
            s,
            position,
            next_id,
            &|| false,
        )
    };
    let mut plan = prepare(&stock).unwrap();
    let pending = serde_json::to_value(plan.report()).unwrap();
    assert!(plan.next_motion_with_stock(&stock, &|| true).is_err());
    assert_eq!(serde_json::to_value(plan.report()).unwrap(), pending);
    // Public reconstruction with identical spec/height/generation cannot inherit
    // the live plan's private native-stock proof. A new plan must scan it anew.
    let restored = Stock::from_snapshot(stock.snapshot()).unwrap();
    assert!(plan.next_motion_with_stock(&restored, &|| false).is_err());
    assert!(
        prepare(&restored)
            .unwrap()
            .next_motion_with_stock(&restored, &|| false)
            .unwrap()
            .is_none()
    );
    let mut different_spec = stock.snapshot();
    different_spec.spec.max_mm[2] = 7.;
    let different_spec = Stock::from_snapshot(different_spec).unwrap();
    assert!(
        plan.next_motion_with_stock(&different_spec, &|| false)
            .is_err()
    );
    let floor = pending["target_z_mm"].as_f64().unwrap();
    let mut near = stock.snapshot();
    near.cells.last_mut().unwrap().upper_mm = floor + 4e-5;
    let near = Stock::from_snapshot(near).unwrap();
    assert!(plan.next_motion_with_stock(&near, &|| false).is_err());
    let error = match prepare(&near) {
        Ok(_) => panic!("air-depth skip tolerance must not prove an empty operation"),
        Err(e) => e,
    };
    assert!(error.contains("above the cut floor"), "{error}");
    let mut high = stock.snapshot();
    high.cells.last_mut().unwrap().upper_mm = 5.5;
    let high = Stock::from_snapshot(high).unwrap();
    assert!(prepare(&high).is_err());
    // A real committed air traverse changes generation without changing stock
    // heights, exercising the generation guard on the same native identity.
    let mut advanced = stock.clone();
    let air = Motion {
        id: next_id,
        operation: "air".into(),
        tool: m.tools[0].id.clone(),
        curve: Curve::Line {
            start: position,
            end: [position[0] + 0.1, position[1], position[2]],
        },
        kind: MoveKind::Rapid,
        feed_mm_min: None,
    };
    let delta = advanced
        .preview_sweep(&m.tools[0].prepare().unwrap(), &air, 1., m.sweep, &|| false)
        .unwrap();
    advanced.apply_delta(&delta).unwrap();
    assert_eq!(advanced.cells(), stock.cells());
    assert_ne!(advanced.generation(), stock.generation());
    assert!(plan.next_motion_with_stock(&advanced, &|| false).is_err());
    assert_eq!(serde_json::to_value(plan.report()).unwrap(), pending);
    assert!(
        plan.next_motion_with_stock(&stock, &|| false)
            .unwrap()
            .is_none()
    );
    assert!(plan.finish_report(&|| true).is_err());
    plan.finish_report(&|| false).unwrap();
}

#[test]
fn empty_roughing_still_validates_tool_source_pose_setup_entry_and_all_strategy_parameters() {
    use camlib::roughing::{RoughingClearing, RoughingOperation};
    for adaptive in [false, true] {
        let (m, stock, position, next_id) = already_faced_roughing(adaptive);
        let Operation::Roughing(base) = m.operations[0].clone() else {
            panic!("rough")
        };
        let invalid: &[fn(&mut RoughingOperation)] = &[
            |o| o.id.clear(),
            |o| o.tool = "unknown".into(),
            |o| o.feed_mm_min = 0.,
            |o| o.feed_mm_min = f64::NAN,
            |o| o.plunge_mm_min = 0.,
            |o| o.spindle_rpm = f64::INFINITY,
            |o| o.work_offset = 0,
            |o| o.tool_number = 0,
            |o| o.length_register = 0,
            |o| o.stepdown_mm = 0.,
            |o| o.radial_allowance_mm = -1.,
            |o| o.clearance_z_mm = f64::NAN,
            |o| std::sync::Arc::make_mut(&mut o.target.triangles).clear(),
            |o| std::sync::Arc::make_mut(&mut o.target.triangles)[0][0][0] = -100.,
            |o| {
                o.entry = PocketEntry::Helix(HelicalEntry {
                    angle_rad: 0.,
                    ..Default::default()
                })
            },
            |o| o.clearing_tolerance_mm = Some(1e-5),
        ];
        for (i, edit) in invalid.iter().enumerate() {
            let mut bad = base.clone();
            edit(&mut bad);
            assert!(
                OperationPlan::prepare_with_stock(
                    Operation::Roughing(bad),
                    &m.tools[0],
                    &stock,
                    position,
                    next_id,
                    &|| false
                )
                .is_err(),
                "invalid authoring case {i}, adaptive={adaptive}"
            );
        }
        for initial in [[f64::NAN, 0., 8.], [0., 0., 7.], [1e6 + 1., 0., 8.]] {
            assert!(
                OperationPlan::prepare_with_stock(
                    m.operations[0].clone(),
                    &m.tools[0],
                    &stock,
                    initial,
                    next_id,
                    &|| false
                )
                .is_err()
            );
        }
        let mut tool = m.tools[0].clone();
        tool.cutter.center_cutting = false;
        assert!(
            OperationPlan::prepare_with_stock(
                m.operations[0].clone(),
                &tool,
                &stock,
                position,
                next_id,
                &|| false
            )
            .is_err()
        );
        // No cutter sweep exists, so flutes need not span an already-removed
        // original blank. Actual layers retain their ordinary reach checks.
        tool.cutter.center_cutting = true;
        tool.cutter.cutting_length_mm = 0.5;
        assert!(
            OperationPlan::prepare_with_stock(
                m.operations[0].clone(),
                &tool,
                &stock,
                position,
                next_id,
                &|| false
            )
            .is_ok()
        );
        let variants: Vec<RoughingClearing> = if adaptive {
            let mutations: &[fn(&mut RoughingClearing)] = &[
                |c| {
                    if let RoughingClearing::Adaptive {
                        orbit_radius_mm, ..
                    } = c
                    {
                        *orbit_radius_mm = Some(f64::NAN)
                    }
                },
                |c| {
                    if let RoughingClearing::Adaptive {
                        min_orbit_radius_mm,
                        ..
                    } = c
                    {
                        *min_orbit_radius_mm = Some(0.)
                    }
                },
                |c| {
                    if let RoughingClearing::Adaptive {
                        local_orbit_tolerance_mm,
                        ..
                    } = c
                    {
                        *local_orbit_tolerance_mm = Some(1e-8)
                    }
                },
                |c| {
                    // #167: disk_front never reads the stock; Roughing
                    // refuses it.
                    if let RoughingClearing::Adaptive { path_strategy, .. } = c {
                        *path_strategy = camlib::adaptive::AdaptivePathStrategy::DiskFront
                    }
                },
                |c| {
                    if let RoughingClearing::Adaptive {
                        max_radial_engagement_mm,
                        ..
                    } = c
                    {
                        *max_radial_engagement_mm = 0.
                    }
                },
                |c| {
                    if let RoughingClearing::Adaptive {
                        max_radial_engagement_mm,
                        ..
                    } = c
                    {
                        *max_radial_engagement_mm = 1.5
                    }
                },
            ];
            mutations
                .iter()
                .map(|f| {
                    let mut c = base.clearing.clone();
                    f(&mut c);
                    c
                })
                .collect()
        } else {
            vec![
                RoughingClearing::Offset {
                    stepover_mm: 0.,
                    coverage_margin_mm: 0.01,
                },
                RoughingClearing::Offset {
                    stepover_mm: 0.8,
                    coverage_margin_mm: 0.,
                },
                RoughingClearing::Offset {
                    stepover_mm: 0.8,
                    coverage_margin_mm: 1.,
                },
                RoughingClearing::Raster {
                    stepover_mm: f64::NAN,
                    routing: None,
                },
            ]
        };
        for clearing in variants {
            let mut bad = base.clone();
            bad.clearing = clearing;
            assert!(
                OperationPlan::prepare_with_stock(
                    Operation::Roughing(bad),
                    &m.tools[0],
                    &stock,
                    position,
                    next_id,
                    &|| false
                )
                .is_err()
            );
        }
    }
}

#[test]
fn stock_front_convex_contour_requires_complete_native_receipts() {
    check_stock_front_contour_receipts("convex_contour", "convex_contours");
}

#[test]
fn stock_front_enclosed_contour_requires_complete_native_receipts() {
    check_stock_front_contour_receipts("enclosed_contour", "enclosed_contours");
}

fn check_stock_front_contour_receipts(status: &str, counter: &str) {
    let m: StreamManifest =
        serde_json::from_str(include_str!("../fixtures/regressions/stock-front.json")).unwrap();
    let tool = m.tools[0].prepare().unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut plan = OperationPlan::prepare_with_stock(
        m.operations[0].clone(),
        &m.tools[0],
        &stock,
        m.initial_position_mm,
        1,
        &|| false,
    )
    .unwrap();
    let mut checked = false;
    for _ in 0..10000 {
        let motion = plan
            .next_motion_with_stock(&stock, &|| false)
            .unwrap()
            .expect("must reach the selected contour transition");
        let report = serde_json::to_value(plan.report()).unwrap();
        let convex = report["layers"][0]["clearing"]["front"]["status"] == status;
        let delta = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &|| false)
            .unwrap();
        if convex && motion.kind == MoveKind::Cut {
            assert_eq!(report["layers"][0]["clearing"]["front"][counter], 0);
            assert_eq!(
                report["layers"][0]["clearing"]["front"]["coverage_envelopes"],
                usize::from(status == "enclosed_contour")
            );
            let mut invalid = Vec::new();
            let mut partial = stock.clone();
            partial
                .apply_delta(
                    &stock
                        .preview_sweep(&tool, &motion, 0.5, m.sweep, &|| false)
                        .unwrap(),
                )
                .unwrap();
            invalid.push(partial);
            let mut altered = motion.clone();
            altered.id += 1;
            let mut wrong_motion = stock.clone();
            wrong_motion
                .apply_delta(
                    &stock
                        .preview_sweep(&tool, &altered, 1., m.sweep, &|| false)
                        .unwrap(),
                )
                .unwrap();
            invalid.push(wrong_motion);
            let wire: camlib::stock::StockDelta =
                serde_json::from_value(serde_json::to_value(&delta).unwrap()).unwrap();
            let mut replayed = stock.clone();
            replayed.apply_delta(&wire).unwrap();
            invalid.push(replayed);
            let mut restored = stock.clone();
            restored.apply_delta(&delta).unwrap();
            invalid.push(Stock::from_snapshot(restored.snapshot()).unwrap());
            let mut undone = stock.clone();
            undone.apply_delta(&delta).unwrap();
            undone.undo_delta(&delta).unwrap();
            undone.apply_delta(&delta).unwrap();
            invalid.push(undone);
            let mut coarse = m.sweep;
            coarse.tolerance_mm *= 2.;
            let mut inaccurate = stock.clone();
            inaccurate
                .apply_delta(
                    &stock
                        .preview_sweep(&tool, &motion, 1., coarse, &|| false)
                        .unwrap(),
                )
                .unwrap();
            invalid.push(inaccurate);
            let mut changed_tool = m.tools[0].clone();
            changed_tool.cutter.diameter_mm *= 0.9;
            let mut other_tool = stock.clone();
            other_tool
                .apply_delta(
                    &stock
                        .preview_sweep(
                            &changed_tool.prepare().unwrap(),
                            &motion,
                            1.,
                            m.sweep,
                            &|| false,
                        )
                        .unwrap(),
                )
                .unwrap();
            invalid.push(other_tool);
            for invalid_stock in invalid {
                assert!(
                    plan.next_motion_with_stock(&invalid_stock, &|| false)
                        .is_err()
                );
            }
            stock.apply_delta(&delta).unwrap();
            assert!(plan.next_motion_with_stock(&stock, &|| true).is_err());
            let next = plan
                .next_motion_with_stock(&stock, &|| false)
                .unwrap()
                .unwrap();
            assert_eq!(next.id, motion.id + 1);
            assert_eq!(next.curve.start(), motion.curve.end());
            let report = serde_json::to_value(plan.report()).unwrap();
            assert_eq!(
                report["layers"][0]["clearing"]["front"][counter], 0,
                "one cut cannot certify the complete contour"
            );
            assert_eq!(
                report["layers"][0]["clearing"]["front"]["coverage_envelopes"],
                usize::from(status == "enclosed_contour"),
                "partial/refused receipts must not publish new coverage"
            );
            checked = true;
            break;
        }
        stock.apply_delta(&delta).unwrap();
    }
    assert!(checked, "test must exercise an actual selected contour");
}

#[test]
fn smaller_raster_rest_rougher_uses_committed_multilevel_stock_with_column_protection() {
    smaller_rest_rougher(false, false);
}

#[test]
fn smaller_offset_rest_rougher_preserves_stock_coverage_and_counterclockwise_contours() {
    smaller_rest_rougher(true, false);
}

#[test]
fn smaller_offset_rest_rougher_preserves_stock_coverage_and_clockwise_contours() {
    smaller_rest_rougher(true, true);
}

fn smaller_rest_rougher(offset: bool, clockwise: bool) {
    let cancel = AtomicBool::new(false);
    let mut m = fixture(false);
    // Resolve the submillimetre corner stock left by the larger cutter.
    m.stock.height_mut().unwrap().cell_size_mm = 0.2;
    let mut coarse = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut first = OperationPlan::prepare_with_stock(
        m.operations[0].clone(),
        &m.tools[0],
        &coarse,
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    let prepared = m.tools[0].prepare().unwrap();
    let mut position = m.initial_position_mm;
    let mut coarse_curves = vec![];
    while let Some(motion) = first.next_motion_with_stock(&coarse, &cancel).unwrap() {
        position = motion.curve.end();
        coarse_curves.push(motion.curve.clone());
        let delta = coarse
            .preview_sweep(&prepared, &motion, 1., m.sweep, &cancel)
            .unwrap();
        coarse.apply_delta(&delta).unwrap();
    }
    first.finish_report(&cancel).unwrap();
    let mut tool = m.tools[0].clone();
    tool.id = "small".into();
    tool.cutter.id = "small-bit".into();
    tool.cutter.diameter_mm = 1.;
    tool.cutter.shank_diameter_mm = 1.;
    let assembly = tool.prepare().unwrap();
    let mut cut_lengths = vec![];
    let mut full_heights = vec![];
    for rest in [false, true] {
        let mut stock = Stock::from_snapshot(coarse.snapshot()).unwrap();
        let Operation::Roughing(mut op) = m.operations[0].clone() else {
            panic!()
        };
        op.id = "rest".into();
        op.tool = tool.id.clone();
        op.rest_tolerance_mm = rest.then_some(0.01);
        op.clockwise = clockwise;
        op.clearing = if offset {
            camlib::roughing::RoughingClearing::Offset {
                stepover_mm: 0.4,
                coverage_margin_mm: 0.01,
            }
        } else {
            camlib::roughing::RoughingClearing::Raster {
                stepover_mm: 0.4,
                routing: None,
            }
        };
        let mut plan = OperationPlan::prepare_with_stock(
            Operation::Roughing(op),
            &tool,
            &stock,
            position,
            1,
            &cancel,
        )
        .unwrap();
        let mut curves = vec![];
        let mut length = 0.;
        let mut arcs = 0;
        let mut checked_arc_receipt = false;
        while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
            certify_part_radius(&motion.curve, 0.6);
            if motion.kind == MoveKind::Cut {
                arcs += usize::from(matches!(motion.curve, Curve::Arc { .. }));
                length += motion.curve.prepare(1e-5).unwrap().length_mm();
            }
            let before = serde_json::to_value(plan.report()).unwrap();
            assert!(
                plan.next_motion_with_stock(&stock, &AtomicBool::new(true))
                    .is_err()
            );
            assert!(
                plan.next_motion_with_stock(&stock, &cancel).is_err(),
                "uncommitted rest motion accepted"
            );
            assert_eq!(serde_json::to_value(plan.report()).unwrap(), before);
            let delta = stock
                .preview_sweep(&assembly, &motion, 1., m.sweep, &cancel)
                .unwrap();
            if offset
                && rest
                && !checked_arc_receipt
                && motion.kind == MoveKind::Cut
                && matches!(motion.curve, Curve::Arc { .. })
            {
                let partial = stock
                    .preview_sweep(&assembly, &motion, 0.5, m.sweep, &cancel)
                    .unwrap();
                let mut changed = motion.clone();
                changed.id += 1;
                let wrong_motion = stock
                    .preview_sweep(&assembly, &changed, 1., m.sweep, &cancel)
                    .unwrap();
                let mut wrong = tool.clone();
                wrong.cutter.diameter_mm *= 0.9;
                let wrong_tool = stock
                    .preview_sweep(&wrong.prepare().unwrap(), &motion, 1., m.sweep, &cancel)
                    .unwrap();
                let wire: camlib::stock::StockDelta =
                    serde_json::from_value(serde_json::to_value(&delta).unwrap()).unwrap();
                for rejected in [partial, wrong_motion, wrong_tool, wire] {
                    let mut invalid = stock.clone();
                    invalid.apply_delta(&rejected).unwrap();
                    assert!(plan.next_motion_with_stock(&invalid, &cancel).is_err());
                    assert_eq!(serde_json::to_value(plan.report()).unwrap(), before);
                }
                let mut valid = stock.clone();
                valid.apply_delta(&delta).unwrap();
                let restored = Stock::from_snapshot(valid.snapshot()).unwrap();
                assert!(plan.next_motion_with_stock(&restored, &cancel).is_err());
                checked_arc_receipt = true;
            }
            stock.apply_delta(&delta).unwrap();
            curves.push(motion.curve);
        }
        plan.finish_report(&cancel).unwrap();
        let report = serde_json::to_value(plan.report()).unwrap();
        assert_eq!(report["completed_layers"], 4);
        if offset {
            assert!(!rest || checked_arc_receipt);
            assert!(arcs > 0, "rest={rest}: lost analytic contours");
            for layer in report["layers"].as_array().unwrap() {
                let clearing = &layer["clearing"];
                assert!(clearing["offset"]["arc_strokes"].as_u64().unwrap() > 0);
                assert_eq!(clearing["routing"]["reversed_rasters"], 0);
                assert!(!clearing["rest_query_work_exhausted"].as_bool().unwrap());
                if rest {
                    assert!(
                        clearing["offset"]["coverage_scope"]
                            .as_str()
                            .unwrap()
                            .contains("before rest filtering")
                    );
                }
            }
        }
        if rest {
            assert!(
                report["layers"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .map(|r| r["clearing"]["rest_skipped_strokes"].as_u64().unwrap())
                    .sum::<u64>()
                    > 0
            );
        }
        assert!(
            stock.volume().removed_lower_mm3 > coarse.volume().removed_lower_mm3,
            "rest={rest}, length={length}, coarse={:?}, fine={:?}",
            coarse.volume(),
            stock.volume()
        );
        let exact = exact_centres_with(
            &stock,
            coarse_curves
                .iter()
                .map(|c| (c, 1.))
                .chain(curves.iter().map(|c| (c, 0.5))),
        );
        if rest {
            for (i, (&remaining, &full)) in exact.iter().zip(&full_heights).enumerate() {
                assert!(
                    remaining <= full + 0.010001,
                    "rest missed cell{i}: remaining={remaining}, full={full}"
                );
            }
        } else {
            full_heights = exact.clone();
        }
        for (i, (cell, exact)) in stock.cells().iter().zip(exact).enumerate() {
            assert!(
                cell.lower_mm <= exact + 1e-7 && cell.upper_mm >= exact - 1e-7,
                "cell{i}: {cell:?} vs {exact}"
            );
        }
        cut_lengths.push(length);
    }
    assert!(cut_lengths[1] < cut_lengths[0] * 0.9, "{cut_lengths:?}");
}

#[test]
fn roughing_reports_preserve_progress_across_reads_rejections_and_finalization() {
    let mut m: StreamManifest =
        serde_json::from_str(include_str!("../fixtures/regressions/stock-front.json")).unwrap();
    // The golden counts below were taken with the exact shadow.
    if let Operation::Roughing(op) = &mut m.operations[0] {
        op.contour_tolerance_mm = Some(0.);
    }
    let tool = m.tools[0].prepare().unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut plan = OperationPlan::prepare_with_stock(
        m.operations[0].clone(),
        &m.tools[0],
        &stock,
        m.initial_position_mm,
        1,
        &|| false,
    )
    .unwrap();
    let prepared = serde_json::to_value(plan.report()).unwrap();
    assert_eq!(prepared["motions"], 0);
    let mut count = 0u64;
    let mut curves = Vec::new();
    let mut observed_layers = std::collections::BTreeSet::new();
    while let Some(motion) = plan.next_motion_with_stock(&stock, &|| false).unwrap() {
        count += 1;
        curves.push(motion.curve.clone());
        // Leave long intervals without readers, then demand the current full
        // public report. Rejected calls must preserve the last visible progress.
        if count <= 5 || count.is_multiple_of(97) {
            let before = serde_json::to_value(plan.report()).unwrap();
            assert_eq!(before["motions"], count);
            let layers = before["layers"].as_array().unwrap();
            assert_eq!(
                layers
                    .iter()
                    .map(|l| l["clearing"]["motions"].as_u64().unwrap())
                    .sum::<u64>(),
                count
            );
            let active = before["active_layer"].as_u64().unwrap();
            observed_layers.insert(active);
            assert_eq!(before["completed_layers"], active);
            for layer in layers.iter().take(active as usize) {
                assert_eq!(layer["clearing"]["geometric_residual_status"], "estimated");
                assert_eq!(layer["clearing"]["front"]["coverage_complete"], true);
            }
            assert_eq!(serde_json::to_value(plan.report()).unwrap(), before);
            assert!(plan.next_motion_with_stock(&stock, &|| true).is_err());
            assert_eq!(serde_json::to_value(plan.report()).unwrap(), before);
            assert!(plan.next_motion_with_stock(&stock, &|| false).is_err());
            assert_eq!(serde_json::to_value(plan.report()).unwrap(), before);
            assert!(plan.finish_report(&|| false).is_err());
            assert_eq!(serde_json::to_value(plan.report()).unwrap(), before);
        }
        let delta = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &|| false)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
    }
    // Golden count from before entry helices were split one turn per motion.
    assert_eq!(count - continued_entry_turns(&curves) as u64, 5366);
    assert_eq!(observed_layers.len(), 4);
    let drained = serde_json::to_value(plan.report()).unwrap();
    assert_eq!(drained["motions"], count);
    assert_eq!(drained["completed_layers"], 4);
    assert_eq!(drained["geometric_residual_status"], "pending");
    assert!(plan.finish_report(&|| true).is_err());
    assert_eq!(serde_json::to_value(plan.report()).unwrap(), drained);
    plan.finish_report(&|| false).unwrap();
    let finished = serde_json::to_value(plan.report()).unwrap();
    assert_eq!(finished["geometric_residual_status"], "estimated");
    assert_eq!(finished["layers"], drained["layers"]);
    assert!(
        plan.next_motion_with_stock(&stock, &|| false)
            .unwrap()
            .is_none()
    );
    assert_eq!(serde_json::to_value(plan.report()).unwrap(), finished);
    assert_eq!(
        prepared["motions"], 0,
        "an earlier owned report must remain unchanged"
    );
}

/// A chamfered round boss tessellated face by face (issue #70's pattern): the
/// wall's top ring is rotated against its bottom ring, and the chamfer's
/// bottom ring follows the wall's bottom ring, so the two faces do not share
/// vertices along their common edge at Z `wall`.
fn separately_tessellated_boss(wall: f64) -> Vec<[[f64; 3]; 3]> {
    let n = 40;
    let step = TAU / n as f64;
    let at = |r: f64, angle: f64, z: f64| [r * angle.cos(), r * angle.sin(), z];
    let mut triangles = vec![];
    for i in 0..n {
        let (a, b) = (i as f64 * step, (i + 1) as f64 * step);
        let twist = 0.1 * step;
        // Wall: radius 3, Z 0..wall, top ring rotated by `twist`.
        let (t0, t1) = (at(3., a - twist, wall), at(3., b - twist, wall));
        let (b0, b1) = (at(3., a, 0.), at(3., b, 0.));
        triangles.push([t1, t0, b0]);
        triangles.push([b0, b1, t1]);
        // Chamfer: radius 3 at Z wall to radius 2 at Z wall + 1.
        let (c0, c1) = (at(3., a, wall), at(3., b, wall));
        let (d0, d1) = (at(2., a, wall + 1.), at(2., b, wall + 1.));
        triangles.push([c0, c1, d0]);
        triangles.push([c1, d1, d0]);
        // Top face.
        triangles.push([[0., 0., wall + 1.], d0, d1]);
    }
    triangles
}

#[test]
fn separately_tessellated_faces_give_a_shadow_for_the_cutter() {
    let cancel = AtomicBool::new(false);
    // A mesh that claims to be exact (error 0): without a cutter the exact
    // union keeps slivers between the two faces' edges; one touches another
    // boundary within the tolerance, and the refusal says where and why.
    let exact = PreparedRoughingTarget::prepare(
        mesh(separately_tessellated_boss(11.), 0.),
        1e-5,
        &cancel,
    )
    .unwrap();
    let refused = exact.shadow(11., 0.2, &mut 0, &cancel).err().unwrap();
    assert!(refused.contains("intersect or touch near ("), "{refused}");
    assert!(refused.contains("not watertight"), "{refused}");
    // For a 12 mm cutter the slivers are filled: no cutter disk fits in one.
    let filled = exact
        .shadow_for_cutter(11., 0.2, 6., &mut 0, &cancel)
        .unwrap();
    assert!(filled.report.filled_holes > 0);
    assert!(filled.report.seam_closing.is_none());
    // Lenses between the two faces' chords along the radius 3 circle only.
    assert!(
        filled.report.filled_hole_area_mm2 < 0.5,
        "{:?}",
        filled.report
    );
    // The same mesh declaring the 0.01 mm it is good for (#241): its seams
    // (the chords' sagitta, 0.0092) are below that error and are closed, with
    // or without a cutter. This was the refusal above before; a seam below
    // the mesh's own declared error says nothing about the part.
    let target = PreparedRoughingTarget::prepare(
        mesh(separately_tessellated_boss(11.), 0.01),
        1e-5,
        &cancel,
    )
    .unwrap();
    for (shadow, cutter) in [
        (filled, 6.),
        (target.shadow(11., 0.2, &mut 0, &cancel).unwrap(), 0.),
        (
            target
                .shadow_for_cutter(11., 0.2, 6., &mut 0, &cancel)
                .unwrap(),
            6.,
        ),
    ] {
        // With a cutter the slivers are filled as holes too small for it and
        // the exact union stands; without one the union is not a valid
        // region and is closed by the declared error.
        if shadow.report.source_error_mm > 0. {
            if cutter == 0. {
                assert!(shadow.report.seam_closing.is_some(), "{:?}", shadow.report);
            }
            let added = shadow.report.seam_closing.map_or(0., |closing| {
                assert_eq!(closing.distance_mm, 0.01);
                closing.added_area_mm2
            });
            assert!(
                added + shadow.report.filled_hole_area_mm2 < 0.5,
                "{:?}",
                shadow.report
            );
        }
        let region = shadow
            .region
            .prepare(ConstructionOptions::default())
            .unwrap();
        assert_eq!(
            (region.summary().components, region.summary().holes),
            (1, 0)
        );
        // Every projected facet point stays protected (filling and closing
        // only add area).
        for triangle in separately_tessellated_boss(11.) {
            for p in triangle {
                assert!(signed(&region, [p[0], p[1]]) > -1e-4, "{p:?}");
            }
        }
        assert!(signed(&region, [0., 0.]) > 1.);
        assert!(signed(&region, [3.2, 0.]) < 0.);
    }
}

#[test]
fn holes_a_cutter_fits_in_stay_open() {
    // A square frame around a 10 x 10 opening (100 mm2), a hole of the
    // shadow. It is filled only below the area of a disk of half the cutter
    // radius: open for radius 3 (7 mm2) and 10 (78.5 mm2), filled for radius
    // 12 (113 mm2), where no cutter disk fits in it.
    let cancel = AtomicBool::new(false);
    let mut triangles = vec![];
    for (x0, y0, x1, y1) in [
        (0., 0., 20., 5.),
        (0., 15., 20., 20.),
        (0., 5., 5., 15.),
        (15., 5., 20., 15.),
    ] {
        triangles.extend(rect(x0, y0, x1, y1, 4.));
    }
    let target = PreparedRoughingTarget::prepare(mesh(triangles, 0.), 1e-5, &cancel).unwrap();
    for (radius, holes) in [(0., 1), (3., 1), (10., 1), (12., 0)] {
        let shadow = target
            .shadow_for_cutter(2., 0., radius, &mut 0, &cancel)
            .unwrap();
        let region = shadow
            .region
            .prepare(ConstructionOptions::default())
            .unwrap();
        assert_eq!(region.summary().holes, holes, "cutter radius {radius}");
        assert_eq!(shadow.report.filled_holes, 1 - holes);
        assert!(signed(&region, [2.5, 10.]) > 0.);
        if holes == 1 {
            assert!(signed(&region, [10., 10.]) < -4.);
        }
    }
    assert!(
        target
            .shadow_for_cutter(2., 0., f64::NAN, &mut 0, &cancel)
            .is_err()
    );
}

/// A cone of `n` facets: its shadow at a level below its base is a fine
/// polygon (n vertices) around a hole-free disc of radius 10.
fn fine_cone(n: usize) -> Vec<[[f64; 3]; 3]> {
    (0..n)
        .map(|k| {
            let (a, b) = (TAU * k as f64 / n as f64, TAU * (k + 1) as f64 / n as f64);
            [
                [10. * a.cos(), 10. * a.sin(), 2.],
                [10. * b.cos(), 10. * b.sin(), 2.],
                [0., 0., 6.],
            ]
        })
        .collect()
}

#[test]
fn simplified_shadows_contain_the_exact_one_with_few_vertices() {
    let cancel = AtomicBool::new(false);
    for n in [256, 4096] {
        let target =
            PreparedRoughingTarget::prepare(mesh(fine_cone(n), 0.), 1e-5, &cancel).unwrap();
        let exact = target
            .shadow_for_cutter(1., 0., 3., &mut 0, &cancel)
            .unwrap();
        let simple = target
            .shadow_simplified(1., 0., 3., 0.075, &mut 0, &cancel)
            .unwrap();
        let report = simple.report.simplified.as_ref().unwrap();
        assert!(report.reason.is_none(), "{report:?}");
        assert_eq!(report.exact_vertices, exact.report.output_vertices);
        let region = simple
            .region
            .prepare(ConstructionOptions::default())
            .unwrap();
        // Every vertex of the exact shadow is inside the simplified one.
        for c in &exact.region.components {
            for v in &c.outer.vertices {
                assert!(signed(&region, [v[0], v[1]]) > 0., "{n}: {v:?}");
            }
        }
        // ... which is no more than the tolerance and its guard bigger.
        for c in &simple.region.components {
            for v in &c.outer.vertices {
                assert!(
                    (v[0].hypot(v[1]) - 10.).abs() < 0.075 * 2f64.sqrt() + 1e-3,
                    "{v:?}"
                );
            }
        }
        // Its size no longer follows the facets: the same for 256 and 4096.
        assert!(
            simple.report.output_vertices < 160,
            "{n}: {}",
            simple.report.output_vertices
        );
    }
    // A hole (a ring of fine facets around an opening a 6 mm cutter fits)
    // shrinks, so the protected area grows, and stays open.
    let mut ring = vec![];
    let n = 2048;
    for k in 0..n {
        let (a, b) = (TAU * k as f64 / n as f64, TAU * (k + 1) as f64 / n as f64);
        let at = |r: f64, t: f64| [r * t.cos(), r * t.sin(), 4.];
        ring.push([at(8., a), at(14., a), at(14., b)]);
        ring.push([at(8., a), at(14., b), at(8., b)]);
    }
    let target = PreparedRoughingTarget::prepare(mesh(ring, 0.), 1e-5, &cancel).unwrap();
    let simple = target
        .shadow_simplified(2., 0., 3., 0.075, &mut 0, &cancel)
        .unwrap();
    let region = simple
        .region
        .prepare(ConstructionOptions::default())
        .unwrap();
    assert_eq!(region.summary().holes, 1);
    for r in [8f64, 14.] {
        for k in 0..64 {
            let t = TAU * k as f64 / 64.;
            assert!(
                signed(&region, [r * t.cos(), r * t.sin()]) > 0.,
                "r {r} at {t}"
            );
        }
    }
    assert!(signed(&region, [0., 0.]) < -7.);
    // 0 keeps the exact shadow.
    assert!(
        target
            .shadow_simplified(2., 0., 3., 0., &mut 0, &cancel)
            .unwrap()
            .report
            .simplified
            .is_none()
    );
}

/// A flat ring of `n` facets at Z8 around (16, 14): outer radius 8, inner
/// `inner`. Its shadow is an annulus whose hole the cutter may just fit in.
fn fine_ring(inner: f64, n: usize) -> Vec<[[f64; 3]; 3]> {
    let at = |r: f64, t: f64| [16. + r * t.cos(), 14. + r * t.sin(), 8.];
    (0..n)
        .flat_map(|k| {
            let (a, b) = (TAU * k as f64 / n as f64, TAU * (k + 1) as f64 / n as f64);
            [
                [at(inner, a), at(8., a), at(8., b)],
                [at(inner, a), at(8., b), at(inner, b)],
            ]
        })
        .collect()
}

#[test]
fn a_gap_the_simplified_shadow_narrows_past_every_entry_keeps_the_exact_shadow() {
    // #123: the hole leaves the 2 mm cutter ~0.2 mm of play with the exact
    // shadow, room for a helical entry; the simplified shadow (grown by up to
    // twice 0.075 mm) leaves less than the smallest helix, while the cutter
    // still fits. The enclosed hole has no stock-edge entry, so the layer
    // must keep the exact shadow instead of refusing the operation.
    let cancel = AtomicBool::new(false);
    let mut m = fixture(false);
    let Operation::Roughing(op) = &mut m.operations[0] else {
        panic!("fixture")
    };
    op.target = mesh(fine_ring(1.5, 2048), 0.);
    op.radial_allowance_mm = 0.3;
    op.floor_z_mm = 6.;
    m.validate().unwrap();
    let prepare = |contour: Option<f64>| {
        let mut op = m.operations[0].clone();
        if let Operation::Roughing(op) = &mut op {
            op.contour_tolerance_mm = contour;
        }
        OperationPlan::prepare(
            op,
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &cancel,
        )
    };
    // The exact shadow plans: the hole is cleared through a helical entry.
    let exact = serde_json::to_value(prepare(Some(0.)).unwrap().report()).unwrap();
    let layers = exact["layers"].as_array().unwrap();
    assert!(!layers.is_empty());
    for layer in layers {
        assert!(layer["shadow"]["simplified"].is_null(), "{layer}");
    }
    // By default every layer is simplified, finds no entry for the hole and
    // falls back to the exact shadow, with the reason reported.
    let plan = prepare(None).unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    let layers = report["layers"].as_array().unwrap();
    assert_eq!(layers.len(), exact["layers"].as_array().unwrap().len());
    for layer in layers {
        let simplified = &layer["shadow"]["simplified"];
        assert_eq!(simplified["tolerance_mm"], 0.075, "{layer}");
        let reason = simplified["reason"].as_str().unwrap();
        assert!(
            reason.contains("no entry reaches") && reason.contains("no source-admitted entry"),
            "{reason}"
        );
        assert_eq!(
            simplified["exact_vertices"], layer["shadow"]["output_vertices"],
            "{layer}"
        );
    }
    // ... and otherwise plans exactly as with the exact shadow.
    for (layer, exact) in layers.iter().zip(exact["layers"].as_array().unwrap()) {
        assert_eq!(layer["clearing"], exact["clearing"]);
        assert_eq!(layer["tip_z_mm"], exact["tip_z_mm"]);
    }
}

/// Analytic height of the `roughing.json` frustum at a point on its +X slope
/// (independent of every planner region): 2 at x = 8 rising to 10 at x = 11.
fn frustum_x_slope_height(p: [f64; 2]) -> f64 {
    2. + (p[0] - 8.) * 8. / 3.
}

#[test]
fn step_up_layers_climb_each_stepdown_and_cut_the_wall_stair_without_touching_the_part() {
    for clearing in [
        camlib::roughing::RoughingClearing::Offset {
            stepover_mm: 0.8,
            coverage_margin_mm: 0.01,
        },
        camlib::roughing::RoughingClearing::Raster {
            stepover_mm: 0.8,
            routing: Some(camlib::pocket::RasterRouting::Economical),
        },
    ] {
        let mut manifest = fixture(false);
        let Operation::Roughing(op) = &mut manifest.operations[0] else {
            panic!("rough")
        };
        op.clearing = clearing.clone();
        let base = scheduled_run(&manifest);
        let Operation::Roughing(op) = &mut manifest.operations[0] else {
            panic!("rough")
        };
        op.stepup_mm = Some(0.5);
        let run = scheduled_run(&manifest);

        // Schedule: each full layer, then its step-ups climbing back up,
        // evenly spaced at most 0.5 apart and below the layer above it.
        let layers = run.report["layers"].as_array().unwrap();
        let full: Vec<f64> = base.report["layers"]
            .as_array()
            .unwrap()
            .iter()
            .map(|l| l["tip_z_mm"].as_f64().unwrap())
            .collect();
        let mut above = 10.;
        let mut index = 0;
        for &z in &full {
            assert_eq!(layers[index]["tip_z_mm"].as_f64().unwrap(), z);
            assert!(layers[index].get("step_up_of_z_mm").is_none());
            index += 1;
            let mut last = z;
            while index < layers.len() && layers[index].get("step_up_of_z_mm").is_some() {
                let up = layers[index]["tip_z_mm"].as_f64().unwrap();
                assert_eq!(layers[index]["step_up_of_z_mm"].as_f64().unwrap(), z);
                assert!(up > last && up - last <= 0.5 + 1e-9 && up < above);
                last = up;
                index += 1;
            }
            assert!(
                above - last <= 0.5 + 1e-9,
                "stair left between Z{last} and Z{above}"
            );
            above = z;
        }
        assert_eq!(index, layers.len());
        assert_eq!(run.report["completed_layers"], layers.len());

        // Independent part oracle on every motion: no column of the part cut.
        for motion in &run.motions {
            certify_part(&motion.curve);
        }
        // Never leaves more anywhere, and the +X wall stair is cut down:
        // leftover above the analytic slope drops from about a full stepdown
        // to at most one step-up plus the allowances.
        let (mut base_worst, mut worst) = (0f64, 0f64);
        for (i, _) in run.stock.cells().iter().enumerate() {
            let p = run.stock.cell_center(i).unwrap();
            let height = |r: &ScheduledRun| {
                r.motions
                    .iter()
                    .map(|m| exact_height(&m.curve, p))
                    .fold(10f64, f64::min)
            };
            let (old, new) = (height(&base), height(&run));
            assert!(
                new <= old + 1e-6,
                "step-up left more at {p:?}: {new} > {old}"
            );
            if (8.6..10.4).contains(&p[0]) && (11.0..17.0).contains(&p[1]) {
                let part = frustum_x_slope_height(p);
                base_worst = base_worst.max(old - part);
                worst = worst.max(new - part);
            }
        }
        eprintln!(
            "step-up {clearing:?}: wall leftover {base_worst:.3} -> {worst:.3} mm; {} -> {} motions; {:.1} -> {:.1} nominal s",
            base.motions.len(),
            run.motions.len(),
            base.nominal,
            run.nominal
        );
        assert!(
            base_worst > 1.5,
            "fixture no longer shows the stair: {base_worst}"
        );
        assert!(worst < 1.0, "step-up layers left {worst} mm on the wall");
    }
}

#[test]
fn step_up_must_be_smaller_than_stepdown() {
    let cancel = AtomicBool::new(false);
    for bad in [0., -0.5, 2., 3., f64::NAN] {
        let mut manifest = fixture(false);
        let Operation::Roughing(op) = &mut manifest.operations[0] else {
            panic!("rough")
        };
        op.stepup_mm = Some(bad);
        let error = OperationPlan::prepare(
            manifest.operations[0].clone(),
            &manifest.tools[0],
            manifest.stock.height().unwrap(),
            manifest.initial_position_mm,
            1,
            &cancel,
        )
        .err()
        .unwrap_or_else(|| panic!("stepup_mm {bad} accepted"));
        assert!(error.contains("stepup_mm"), "{error}");
    }
}

#[test]
fn engagement_front_clears_thin_remnants_from_the_side_not_by_helix() {
    // At 0.6 mm engagement the passes along the stock faces leave slivers
    // too thin for a pass to start on. They lie beside cleared stock, so
    // they are cleared at depth from there: the only helices are the
    // operation's first entry (nothing is cut yet) and the bores into the
    // enclosed 4 x 4 mm opening. Before, the top layer also bored helices
    // onto slivers at the stock faces.
    let base = engagement_front_frustum();
    let mut v = serde_json::to_value(&base).unwrap();
    v["operations"][0]["clearing"]["max_radial_engagement_mm"] = 0.6.into();
    let m: StreamManifest = serde_json::from_value(v).unwrap();
    let (history, _) = run_engagement_front(&m);
    let mut starts = vec![];
    for (i, motion) in history.iter().enumerate() {
        if let Curve::Helix { center, .. } = motion.curve
            && !matches!(history[i - 1].curve, Curve::Helix { .. })
        {
            starts.push([center[0], center[1]]);
        }
    }
    assert!(starts.len() > 1);
    for c in &starts[1..] {
        assert!(
            (13.0..=19.0).contains(&c[0]) && (11.0..=17.0).contains(&c[1]),
            "helix at {c:?} outside the enclosed opening; helices {starts:?}"
        );
    }
}

#[test]
fn engagement_front_links_feed_fast_only_through_cleared_air() {
    // With a link feed, a stay-down link runs at it only where the stock
    // left by every earlier cut (independent exact heights) holds nothing
    // the cutter could touch at the link's depth; the rest keeps the cutting
    // feed.
    let base = engagement_front_frustum();
    let mut v = serde_json::to_value(&base).unwrap();
    v["operations"][0]["link_feed_mm_min"] = 5000.0.into();
    let cutting = v["operations"][0]["feed_mm_min"].as_f64().unwrap();
    let m: StreamManifest = serde_json::from_value(v).unwrap();
    let (history, nominal) = run_engagement_front(&m);
    let (_, before) = run_engagement_front(&base);
    let (mut fast, mut slow) = (0f64, 0f64);
    for (i, motion) in history.iter().enumerate() {
        if motion.kind != MoveKind::Link {
            continue;
        }
        let Curve::Line { start, end } = motion.curve else {
            continue;
        };
        let length = (end[0] - start[0]).hypot(end[1] - start[1]);
        if motion.feed_mm_min != Some(5000.) {
            assert_eq!(motion.feed_mm_min, Some(cutting));
            slow += length;
            continue;
        }
        fast += length;
        let n = (length / 0.1).ceil() as usize;
        for k in 0..=n {
            let t = k as f64 / n.max(1) as f64;
            let p = [
                start[0] + (end[0] - start[0]) * t,
                start[1] + (end[1] - start[1]) * t,
            ];
            let top = material_top(&history, i, p, 0.);
            assert!(
                top <= start[2] + 1e-6,
                "fast link {i} at {p:?} meets material {} above its depth",
                top - start[2]
            );
        }
    }
    eprintln!(
        "links: {fast:.0} mm fast, {slow:.0} mm at cutting feed; nominal {before:.0} -> {nominal:.0} s"
    );
    assert!(fast > 0.);
    assert!(nominal < before);
}

#[test]
fn engagement_front_rapid_descent_and_ramp_clearances_connect_in_any_order() {
    // Audit #23: a rapid-descent stop below the helix start (rapid clearance
    // under the ramp clearance) left a gap the stream refused. Below, equal
    // and above, with and without the feed-height cap, every motion starts
    // where the previous one ended.
    for (rapid, ramp) in [(0.1, 2.0), (1.0, 1.0), (0.5, 0.3), (0.1, 50.0)] {
        let mut v = serde_json::to_value(engagement_front_frustum()).unwrap();
        v["operations"][0]["rapid_descent_clearance_mm"] = rapid.into();
        v["operations"][0]["ramp_clearance_mm"] = ramp.into();
        let m: StreamManifest = serde_json::from_value(v).unwrap();
        let (history, _) = run_engagement_front(&m);
        let mut at = m.initial_position_mm;
        let mut helices = 0;
        for motion in &history {
            let start = motion.curve.start();
            assert!(
                (0..3).all(|k| (start[k] - at[k]).abs() < 1e-9),
                "rapid {rapid} ramp {ramp}: motion {} starts at {start:?}, tool at {at:?}",
                motion.id
            );
            helices += usize::from(matches!(motion.curve, Curve::Helix { .. }));
            at = motion.curve.end();
        }
        assert!(helices > 0, "rapid {rapid} ramp {ramp}: no helix exercised");
    }
}

#[test]
fn engagement_front_thins_link_feed_only_as_far_as_the_chip_allows() {
    // With link chip thinning, a stay-down link that may touch stock runs
    // faster than the cutting feed only as far as radial chip thinning
    // allows: on a straight move a tooth at angle t from the feed direction
    // cuts fz * cos(t) and meets material only at lateral offset r * sin(t),
    // so with no material within lateral offset s of the line the chip is at
    // most fz * sqrt(1 - (s/r)^2). Independently of the planner's stock
    // bounds, the smallest lateral offset of material left by every earlier
    // cut (exact heights, sampled at 0.02 mm and taken 0.02 mm closer) must
    // keep feed * that chip factor within the cutting feed's chip at the
    // clearing's engagement (Sandvik Coromant, radial chip thinning).
    // A 0.8 mm engagement (of the 1 mm radius) leaves links room to thin
    // with the 0.4 mm stock cells.
    let mut v = serde_json::to_value(engagement_front_frustum()).unwrap();
    v["operations"][0]["clearing"]["max_radial_engagement_mm"] = 0.8.into();
    v["operations"][0]["link_feed_mm_min"] = 5000.0.into();
    let (_, linked) = run_engagement_front(&serde_json::from_value(v.clone()).unwrap());
    v["operations"][0]["link_chip_thinning"] = true.into();
    let cutting = v["operations"][0]["feed_mm_min"].as_f64().unwrap();
    let m: StreamManifest = serde_json::from_value(v).unwrap();
    let (history, nominal) = run_engagement_front(&m);
    let (r, d) = (1f64, 2f64);
    let factor = |ae: f64| {
        let x = (ae / d).min(0.5);
        2. * (x - x * x).max(0.).sqrt()
    };
    let limit = cutting * factor(0.8);
    let (mut thinned, mut worst) = (0f64, 0f64);
    for (i, motion) in history.iter().enumerate() {
        let feed = motion.feed_mm_min.unwrap_or(0.);
        if motion.kind != MoveKind::Link || feed <= cutting {
            continue;
        }
        let Curve::Line { start, end } = motion.curve else {
            panic!("link {i} is not a line");
        };
        let (dx, dy) = (end[0] - start[0], end[1] - start[1]);
        let length = dx.hypot(dy);
        let (ux, uy) = (dx / length, dy / length);
        let near: Vec<_> = history[..i]
            .iter()
            .filter(|old| old.kind != MoveKind::Rapid)
            .filter(|old| {
                // A disc holding the move's whole tool footprint.
                let (a, b) = (old.curve.start(), old.curve.end());
                let (c, extent) = match circle(&old.curve) {
                    Some((o, _)) => ([o[0], o[1]], (a[0] - o[0]).hypot(a[1] - o[1])),
                    None => (
                        [(a[0] + b[0]) / 2., (a[1] + b[1]) / 2.],
                        (b[0] - a[0]).hypot(b[1] - a[1]) / 2.,
                    ),
                };
                let reach = 2. * r + extent + 1e-9;
                let t = (((c[0] - start[0]) * ux + (c[1] - start[1]) * uy) / length).clamp(0., 1.);
                (c[0] - start[0] - t * dx).hypot(c[1] - start[1] - t * dy) <= reach
            })
            .collect();
        let mut lateral = r;
        let steps = ((length + 2. * r) / 0.02).ceil() as usize;
        for k in 0..=steps {
            let u = -r + (length + 2. * r) * k as f64 / steps as f64;
            for j in 0..=100 {
                let w = -r + 2. * r * j as f64 / 100.;
                if w.abs() >= lateral {
                    continue;
                }
                let along = u.clamp(0., length);
                if (u - along).hypot(w) > r {
                    continue;
                }
                let q = [start[0] + u * ux - w * uy, start[1] + u * uy + w * ux];
                if q[0] < 0. || q[0] > 32. || q[1] < 0. || q[1] > 28. {
                    continue;
                }
                let top = near
                    .iter()
                    .map(|old| exact_height(&old.curve, q))
                    .fold(10f64, f64::min);
                if top > start[2] + 1e-6 {
                    lateral = w.abs();
                }
            }
        }
        // No material sampled within reach: an air link (checked by
        // engagement_front_links_feed_fast_only_through_cleared_air).
        let chip = if lateral >= r {
            0.
        } else {
            feed * factor(r - (lateral - 0.02).max(0.))
        };
        worst = worst.max(chip / limit);
        assert!(
            chip <= limit * (1. + 1e-9),
            "link {i} at {feed} mm/min meets material {lateral:.3} mm off its line: chip {:.3} of the cutting chip",
            chip / limit
        );
        if feed < 5000. {
            thinned += length;
        }
    }
    eprintln!(
        "thinned {thinned:.0} mm; worst chip {worst:.3} of the cutting chip; nominal {linked:.0} -> {nominal:.0} s"
    );
    assert!(thinned > 0.);
    assert!(nominal < linked);
}

#[test]
fn engagement_front_link_chip_thinning_needs_a_link_feed() {
    let mut v = serde_json::to_value(engagement_front_frustum()).unwrap();
    v["operations"][0]["link_chip_thinning"] = true.into();
    let m: StreamManifest = serde_json::from_value(v).unwrap();
    let tool = &m.tools[0];
    let stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let error = OperationPlan::prepare_with_stock(
        m.operations[0].clone(),
        tool,
        &stock,
        m.initial_position_mm,
        1,
        &|| false,
    )
    .err()
    .unwrap();
    assert!(
        error.contains("link_chip_thinning needs link_feed_mm_min"),
        "{error}"
    );
}

/// A closed axis-aligned box mesh (outward triangles) for a synthetic target.
fn closed_box(x0: f64, y0: f64, z0: f64, x1: f64, y1: f64, z1: f64) -> Vec<[[f64; 3]; 3]> {
    let p = [
        [x0, y0, z0],
        [x1, y0, z0],
        [x1, y1, z0],
        [x0, y1, z0],
        [x0, y0, z1],
        [x1, y0, z1],
        [x1, y1, z1],
        [x0, y1, z1],
    ];
    let quad = |a, b, c, d| vec![[p[a], p[b], p[c]], [p[a], p[c], p[d]]];
    [
        quad(4, 5, 6, 7), // top (+z)
        quad(0, 3, 2, 1), // bottom (-z)
        quad(0, 1, 5, 4), // y0 face (-y)
        quad(3, 7, 6, 2), // y1 face (+y)
        quad(0, 4, 7, 3), // x0 face (-x)
        quad(1, 2, 6, 5), // x1 face (+x)
    ]
    .concat()
}

/// Independent part certificate for the bar test (the frustum test's
/// certificate, for this part): no cutter centre enters the retained bar
/// x 0..60, y 12..28 grown by the cutter radius plus the radial allowance.
fn certify_bar(c: &Curve) {
    if c.start()[2].min(c.end()[2]) > 12. {
        return;
    }
    let part = [[0., 12.], [60., 28.]];
    let mut stack = vec![(0., 1., 0)];
    let mut visits = 0;
    while let Some((a, b, depth)) = stack.pop() {
        visits += 1;
        assert!(visits < 200_000, "part certificate work bound");
        if box_distance(bounds(c, a, b), part) >= 3.1 {
            continue;
        }
        assert!(
            depth < 32,
            "uncertified retained-bar column at {c:?}, interval{a}..{b}"
        );
        let m = (a + b) / 2.;
        stack.push((a, m, depth + 1));
        stack.push((m, b, depth + 1));
    }
}

/// The bar job: stock (0,0,0)-(60,40,12) at cell 0.4, a full-length bar
/// x 0..60, y 12..28, z 0..12 retained, a flat 6 mm tool, engagement_front
/// clearing at 0.6 mm width, stepdown 6, floor 1. Both side strips run the
/// stock's whole 60 mm length.
fn bar_manifest(top_z: f64, ends: bool) -> StreamManifest {
    serde_json::from_value(bar_input(top_z, ends)).unwrap()
}

/// The bar job's manifest JSON, for tests that change it before parsing.
fn bar_input(top_z: f64, ends: bool) -> serde_json::Value {
    // `top_z` is the stock and part height; above 12 the first layer still
    // cuts at Z6 (one deep stepdown) and the tool is lengthened to reach.
    let saved: serde_json::Value =
        serde_json::from_str(include_str!("../fixtures/regressions/retained-body.json")).unwrap();
    let mut input = saved["cases"]["adaptive"]["policies"]["omit_proven_empty"]["input"].clone();
    input["stock"]["min_mm"] = serde_json::json!([0.0, 0.0, 0.0]);
    input["stock"]["max_mm"] = serde_json::json!([60.0, 40.0, top_z]);
    input["stock"]["cell_size_mm"] = 0.4.into();
    if top_z > 12. {
        let tool = &mut input["tools"][0];
        tool["cutter"]["cutting_length_mm"] = (top_z + 6.).into();
        tool["cutter"]["overall_length_mm"] = (top_z + 26.).into();
        tool["stickout_mm"] = (top_z + 16.).into();
        tool["gauge_length_mm"] = (top_z + 46.).into();
        tool["holders"][0]["z_min_mm"] = (top_z + 16.).into();
        tool["holders"][0]["z_max_mm"] = (top_z + 46.).into();
    }
    input["initial_position_mm"] = serde_json::json!([0.0, 0.0, top_z + 2.]);
    input["tools"][0]["cutter"]["diameter_mm"] = 6.0.into();
    input["tools"][0]["cutter"]["shank_diameter_mm"] = 6.0.into();
    for holder in input["tools"][0]["holders"].as_array_mut().unwrap() {
        holder["radius_max_mm"] = 6.0.into();
        holder["radius_min_mm"] = 6.0.into();
    }
    let op = input["operations"][0].as_object_mut().unwrap();
    op["omit_empty_cuts"] = false.into();
    op["top_z_mm"] = top_z.into();
    op["floor_z_mm"] = 1.0.into();
    op["stepdown_mm"] = (top_z - 6.).into();
    op["clearance_z_mm"] = (top_z + 2.).into();
    op["radial_allowance_mm"] = 0.1.into();
    op["bottom_allowance_mm"] = 0.0.into();
    op["target"] = serde_json::json!({
        "version": 1,
        "units": "mm",
        "triangles": if ends {
            // End walls close both strips: x 0..4 and 56..60, full height.
            [
                closed_box(0., 12., 0., 60., 28., top_z),
                closed_box(0., 0., 0., 4., 40., top_z),
                closed_box(56., 0., 0., 60., 40., top_z),
            ]
            .concat()
        } else {
            closed_box(0., 12., 0., 60., 28., top_z)
        },
        "surface_error_mm": 0.0,
        "source": null,
    });
    let clearing = op["clearing"].as_object_mut().unwrap();
    for key in [
        "orbit_radius_mm",
        "min_orbit_radius_mm",
        "local_orbit_tolerance_mm",
    ] {
        clearing.remove(key);
    }
    clearing.insert("path_strategy".into(), "engagement_front".into());
    clearing.insert("max_radial_engagement_mm".into(), 0.6.into());
    input
}

/// Run the bar job to completion, committing every motion to stock, and
/// return (mean first-layer cut length per pass mm, report). Every motion
/// is certified against the bar, and every vertical entry is checked to
/// descend only through cut stock or air (the frustum test's check).
fn run_bar(top_z: f64, ends: bool) -> (f64, serde_json::Value) {
    let (mean, report, _) = run_bar_job(bar_manifest(top_z, ends), top_z);
    (mean, report)
}

/// `run_bar` for a bar job's manifest, returning its motions as well.
fn run_bar_job(
    m: StreamManifest,
    top_z: f64,
) -> (f64, serde_json::Value, Vec<camlib::motion::Motion>) {
    m.validate().unwrap();
    let tool = m.tools[0].prepare().unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut plan = OperationPlan::prepare_with_stock(
        m.operations[0].clone(),
        &m.tools[0],
        &stock,
        m.initial_position_mm,
        1,
        &|| false,
    )
    .unwrap();
    let mut history: Vec<camlib::motion::Motion> = Vec::new();
    let mut current = m.initial_position_mm;
    let (mut first_cut, mut first_passes) = (0f64, 0usize);
    let (mut in_cut, mut run_cut, mut run_on_wall) = (false, 0f64, false);
    while let Some(motion) = plan.next_motion_with_stock(&stock, &|| false).unwrap() {
        assert!(history.len() < 60000, "unbounded engagement_front path");
        assert_eq!(motion.curve.start(), current);
        current = motion.curve.end();
        certify_bar(&motion.curve);
        if motion.kind == MoveKind::Plunge && current[2] < top_z {
            for radial in [0., 1.5, 3.] {
                for i in 0..16 {
                    let angle = TAU * i as f64 / 16.;
                    let p = [
                        current[0] + radial * angle.cos(),
                        current[1] + radial * angle.sin(),
                    ];
                    if p[0] < 0. || p[0] > 60. || p[1] < 0. || p[1] > 40. {
                        continue;
                    }
                    let height = history
                        .iter()
                        .filter(|old| old.kind != MoveKind::Rapid)
                        .map(|old| exact_height_for(&old.curve, p, 3., top_z))
                        .fold(top_z, f64::min);
                    assert!(
                        height <= current[2] + 1e-6,
                        "plunge into material at {p:?}: {height} > {}",
                        current[2]
                    );
                }
            }
        }
        let length = motion
            .curve
            .prepare(m.sweep.tolerance_mm)
            .unwrap()
            .length_mm();
        // The first layer cuts at Z6: count the cut length and the number
        // of the front's passes (a pass is a maximal run of consecutive
        // cutting motions; the layer's wall pass is not one of the front's:
        // how it is split depends on the heights, see `plan_wall_runs`).
        let first_layer = motion.kind == MoveKind::Cut
            && (motion.curve.start()[2] - 6.).abs() < 1e-6
            && (motion.curve.end()[2] - 6.).abs() < 1e-6;
        if first_layer {
            in_cut = true;
            run_cut += length;
            // A wall pass runs on the bar's exact offset (3.1 mm); the
            // front's passes keep about two grid cells further off.
            run_on_wall |= box_distance(bounds(&motion.curve, 0., 1.), [[0., 12.], [60., 28.]]) < 3.15;
        } else {
            if in_cut && !run_on_wall {
                first_passes += 1;
                first_cut += run_cut;
            }
            (in_cut, run_cut, run_on_wall) = (false, 0., false);
        }
        let delta = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &|| false)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        history.push(motion);
    }
    plan.finish_report(&|| false).unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    let layers = report["layers"].as_array().unwrap();
    assert_eq!(report["completed_layers"], layers.len());
    for layer in layers {
        let clearing = &layer["clearing"];

        assert_eq!(clearing["strategy"], "engagement_front");
        assert!(clearing["passes"].as_u64().unwrap() > 0);
        assert!(
            clearing["max_estimated_engagement_mm"].as_f64().unwrap() <= 1.3 * 0.6,
            "{clearing}"
        );
    }
    assert!(first_passes > 0, "no first-layer passes planned");
    (first_cut / first_passes as f64, report, history)
}

/// The least and the most stock left on the bar's two walls (y 12 and y 28,
/// x 0..60) at height `z`, every 0.05 mm along them: the distance from the
/// wall point to the nearest cutter centre of any feed move at or below
/// `z`, less the cutter radius. Worked out from the motions' own curves,
/// with nothing of the planner or the stock model in it.
fn bar_wall_stock(history: &[camlib::motion::Motion], z: f64) -> (f64, f64) {
    let near = |c: &Curve, p: [f64; 2]| -> f64 {
        let (a, b) = (c.start(), c.end());
        let to = |q: [f64; 3]| (p[0] - q[0]).hypot(p[1] - q[1]);
        match circle(c) {
            Some((o, sweep)) => {
                let radius = (a[0] - o[0]).hypot(a[1] - o[1]);
                let start = (a[1] - o[1]).atan2(a[0] - o[0]);
                let turn = ((p[1] - o[1]).atan2(p[0] - o[0]) - start) * sweep.signum();
                if turn.rem_euclid(TAU) <= sweep.abs() {
                    ((p[0] - o[0]).hypot(p[1] - o[1]) - radius).abs()
                } else {
                    to(a).min(to(b))
                }
            }
            None => {
                let d = [b[0] - a[0], b[1] - a[1]];
                let length2 = d[0] * d[0] + d[1] * d[1];
                let t = if length2 > 0. {
                    (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / length2).clamp(0., 1.)
                } else {
                    0.
                };
                (p[0] - a[0] - t * d[0]).hypot(p[1] - a[1] - t * d[1])
            }
        }
    };
    let feeds: Vec<&Curve> = history
        .iter()
        .filter(|m| m.kind != MoveKind::Rapid && m.kind != MoveKind::Retract)
        .filter(|m| m.curve.start()[2].max(m.curve.end()[2]) <= z + 1e-9)
        .map(|m| &m.curve)
        .collect();
    let (mut least, mut most) = (f64::INFINITY, 0f64);
    for wall in [12., 28.] {
        for i in 0..=1200 {
            let p = [i as f64 * 0.05, wall];
            let left = feeds
                .iter()
                .map(|c| near(c, p))
                .fold(f64::INFINITY, f64::min)
                - 3.;
            least = least.min(left);
            most = most.max(left);
        }
    }
    (least, most)
}

/// Adaptive roughing leaves the radial allowance on the walls, and no more.
///
/// The front's passes end at a wall a stepover apart; between two pass
/// ends the round cutter leaves a cusp (up to 0.4 mm here, where 0.1 mm was
/// asked for; a step-up used to find one and nibble its top). Each layer
/// now ends with a pass along the walls (`plan_wall_runs`).
#[test]
fn engagement_front_wall_pass_leaves_the_allowance_on_the_walls() {
    let (_, report, history) = run_bar_job(bar_manifest(12., false), 12.);
    for layer in report["layers"].as_array().unwrap() {
        let z = layer["tip_z_mm"].as_f64().unwrap();
        let (least, most) = bar_wall_stock(&history, z);
        // The allowance, and the planner's own margins on top of it (the
        // shadow's and the offset's, 1.3 um together here): never less.
        assert!(
            least >= 0.1 && most <= 0.102,
            "wall stock at Z{z}: {least:.4}..{most:.4} mm, asked 0.1: {}",
            layer["clearing"]
        );
        assert!(
            layer["clearing"]["wall_passes"].as_u64().unwrap() >= 1,
            "{}",
            layer["clearing"]
        );
    }
}

/// A wall pass runs only where material stands beside the wall.
///
/// The bar's walls run the stock's whole length and end in air. With a
/// plunge as fast as the feed and a feed height 2 mm above the stock, going
/// round the bar's end through the air (26 mm) takes longer than a new
/// entry, so each wall is its own run: none of its cutting moves lies
/// wholly beyond the stock, and the walls are still left with the allowance.
#[test]
fn engagement_front_wall_runs_end_where_the_stock_does() {
    let mut input = bar_input(12., false);
    input["operations"][0]["plunge_mm_min"] = input["operations"][0]["feed_mm_min"].clone();
    let m: StreamManifest = serde_json::from_value(input).unwrap();
    let (_, report, history) = run_bar_job(m, 12.);
    for layer in report["layers"].as_array().unwrap() {
        let z = layer["tip_z_mm"].as_f64().unwrap();
        let (least, most) = bar_wall_stock(&history, z);
        // The allowance, and the planner's own margins on top of it (the
        // shadow's and the offset's, 1.3 um together here): never less.
        assert!(
            least >= 0.1 && most <= 0.102,
            "wall stock at Z{z}: {least:.4}..{most:.4} mm, asked 0.1: {}",
            layer["clearing"]
        );
        assert_eq!(layer["clearing"]["wall_passes"], 2, "{}", layer["clearing"]);
    }
    let stock = [[0., 0.], [60., 40.]];
    // A run's lead-in arc (a quarter turn of 0.4 mm radius here) may lie
    // beyond the stock; nothing longer does.
    for motion in &history {
        if motion.kind == MoveKind::Cut
            && box_distance(bounds(&motion.curve, 0., 1.), stock) >= 3.
        {
            let length = motion.curve.prepare(1e-6).unwrap().length_mm();
            assert!(
                length < 1.,
                "cutting move of {length:.2} mm wholly in air: {:?}",
                motion.curve
            );
        }
    }
}

/// Step-ups beside vertical walls find nothing to cut: the full layer below
/// already cut every wall down to its allowance. Each layer hands its
/// cleared-height memory to the next; an empty step-up (no motion) used to
/// drop it, so the step-up after it steered from whole-cell stock bounds,
/// saw material along every wall the full layer had cut, and ran passes in
/// air there (on alternate step-ups).
#[test]
fn engagement_front_empty_step_up_keeps_the_cleared_memory() {
    let mut input = bar_input(12., false);
    // Walls off the 0.4 mm stock cells, so whole-cell bounds overlap them.
    input["operations"][0]["target"]["triangles"] =
        serde_json::to_value(closed_box(0., 12.15, 0., 60., 27.85, 12.)).unwrap();
    input["operations"][0]["stepdown_mm"] = 11.0.into();
    input["operations"][0]["stepup_mm"] = 2.0.into();
    let m: StreamManifest = serde_json::from_value(input).unwrap();
    m.validate().unwrap();
    let tool = m.tools[0].prepare().unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut plan = OperationPlan::prepare_with_stock(
        m.operations[0].clone(),
        &m.tools[0],
        &stock,
        m.initial_position_mm,
        1,
        &|| false,
    )
    .unwrap();
    let mut cut_at = std::collections::HashMap::<u64, f64>::new();
    while let Some(motion) = plan.next_motion_with_stock(&stock, &|| false).unwrap() {
        if motion.kind == MoveKind::Cut {
            let (a, b) = (motion.curve.start(), motion.curve.end());
            *cut_at.entry(a[2].to_bits()).or_default() += (b[0] - a[0]).hypot(b[1] - a[1]);
        }
        let delta = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &|| false)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
    }
    plan.finish_report(&|| false).unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    let layers = report["layers"].as_array().unwrap();
    let ups: Vec<_> = layers
        .iter()
        .filter(|l| l["step_up_of_z_mm"].is_number())
        .collect();
    assert!(
        ups.len() >= 3,
        "expected several step-ups: {}",
        layers.len()
    );
    // Every step-up cuts nothing at all: the full layer's passes end at the
    // wall a stepover apart and leave cusps on it between their ends, and
    // its wall pass takes them (`plan_wall_runs`). Before roughing had a wall
    // pass the lowest step-up found one of those cusps and took a stroke of
    // 0.31 mm at it; the defect this test is named for ran passes the length
    // of the walls (124 mm at Z6.5).
    for l in &ups {
        let z = l["tip_z_mm"].as_f64().unwrap();
        let cut = cut_at.get(&z.to_bits()).copied().unwrap_or(0.);
        assert_eq!(
            (cut, l["clearing"]["passes"].as_u64()),
            (0., Some(0)),
            "step-up at Z{z} beside vertical walls: {}",
            l["clearing"]
        );
        // It steered from the cleared memory handed on by the layer before
        // it (through an empty step-up too), not from whole-cell bounds.
        assert_eq!(l["clearing"]["stock_top_carried"], true, "{}", l["clearing"]);
    }
}

/// Run the bar job (end walls, a high retract) with `travel_clearance_mm`
/// set or not, committing every motion. Every rapid that runs across below
/// the retract height is checked against the exact stock left by the cuts
/// before it (from the cut curves, not the stock model): over the whole
/// rectangle its ends span, grown by the cutter radius, so the straight
/// route and both axis-first doglegs clear the stock and the bar. Returns
/// (rapid and retract length mm, report).
fn run_bar_travel(clearance: Option<f64>) -> (f64, serde_json::Value) {
    let mut input = bar_input(12., true);
    input["initial_position_mm"] = serde_json::json!([0.0, 0.0, 30.0]);
    let op = &mut input["operations"][0];
    op["clearance_z_mm"] = 30.0.into();
    op["retract_z_mm"] = 30.0.into();
    if let Some(c) = clearance {
        op["travel_clearance_mm"] = c.into();
    }
    let m: StreamManifest = serde_json::from_value(input).unwrap();
    m.validate().unwrap();
    let tool = m.tools[0].prepare().unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut plan = OperationPlan::prepare_with_stock(
        m.operations[0].clone(),
        &m.tools[0],
        &stock,
        m.initial_position_mm,
        1,
        &|| false,
    )
    .unwrap();
    let mut history: Vec<camlib::motion::Motion> = Vec::new();
    let mut current = m.initial_position_mm;
    let mut travel = 0.;
    let (r, top) = (3., 12.);
    while let Some(motion) = plan.next_motion_with_stock(&stock, &|| false).unwrap() {
        assert!(history.len() < 60000, "unbounded engagement_front path");
        assert_eq!(motion.curve.start(), current);
        current = motion.curve.end();
        certify_bar(&motion.curve);
        let (a, b) = (motion.curve.start(), motion.curve.end());
        if matches!(motion.kind, MoveKind::Rapid | MoveKind::Retract) {
            travel += ((b[0] - a[0]).hypot(b[1] - a[1])).hypot(b[2] - a[2]);
        }
        let across = (b[0] - a[0]).hypot(b[1] - a[1]) > 1e-9;
        if motion.kind == MoveKind::Rapid && across && a[2] < 30. - 1e-9 {
            assert!(
                (a[2] - b[2]).abs() < 1e-12,
                "rapid across changes Z: {motion:?}"
            );
            let (x0, x1) = (a[0].min(b[0]) - r, a[0].max(b[0]) + r);
            let (y0, y1) = (a[1].min(b[1]) - r, a[1].max(b[1]) + r);
            let step = 0.2;
            let (nx, ny) = (
                ((x1 - x0) / step).ceil() as usize,
                ((y1 - y0) / step).ceil() as usize,
            );
            for i in 0..=nx {
                for j in 0..=ny {
                    let p = [x0 + i as f64 * step, y0 + j as f64 * step];
                    if p[0] < 0. || p[0] > 60. || p[1] < 0. || p[1] > 40. {
                        continue;
                    }
                    // The bar and the end walls are never cut.
                    let kept = (12. ..=28.).contains(&p[1]) || p[0] <= 4. || p[0] >= 56.;
                    let height = if kept {
                        top
                    } else {
                        history
                            .iter()
                            .filter(|old| old.kind != MoveKind::Rapid)
                            .map(|old| exact_height_for(&old.curve, p, r, top))
                            .fold(top, f64::min)
                    };
                    assert!(
                        a[2] > height + 0.5,
                        "rapid across at Z{} over stock at {p:?} (Z{height}): {motion:?}",
                        a[2]
                    );
                }
            }
        }
        let delta = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &|| false)
            .unwrap();
        if matches!(motion.kind, MoveKind::Rapid | MoveKind::Retract) {
            assert!(
                delta.changes().iter().all(|c| c.after == c.before),
                "rapid removes stock: {motion:?}"
            );
        }
        stock.apply_delta(&delta).unwrap();
        history.push(motion);
    }
    plan.finish_report(&|| false).unwrap();
    (travel, serde_json::to_value(plan.report()).unwrap())
}

/// With `travel_clearance_mm`, rapids between passes run just above the
/// stock their route can meet instead of climbing to the retract height,
/// proven clear over the whole rectangle of each route (dogleg-safe).
#[test]
fn engagement_front_travel_clearance_lowers_rapids_over_cut_stock() {
    let lowered = |report: &serde_json::Value| -> u64 {
        report["layers"]
            .as_array()
            .unwrap()
            .iter()
            .map(|l| l["clearing"]["lowered_travels"].as_u64().unwrap())
            .sum()
    };
    let (high, high_report) = run_bar_travel(None);
    let (low, low_report) = run_bar_travel(Some(1.));
    eprintln!(
        "rapid travel {high:.0} mm at retract, {low:.0} mm lowered ({} lowered)",
        lowered(&low_report)
    );
    assert_eq!(lowered(&high_report), 0);
    assert!(lowered(&low_report) > 0, "no rapid was lowered");
    assert!(low < 0.8 * high, "lowered rapids did not shorten travel");
}

/// The strips closed by end walls (so later passes start beside the stock,
/// proven only by the committed sweeps), with the operation's top 48 mm
/// above the Z6 layer (a deep layer, as under a tall part). A vertical entry into cut stock is
/// proven by bisecting the column between the top and the layer; with a
/// fixed 12 halvings that search could not resolve below (top - z) / 4096,
/// over its 0.01 mm acceptance once top - z exceeded ~41 mm, so every such
/// entry was refused and the layer fell back to short stay-down passes.
#[test]
fn engagement_front_deep_layer_entries_are_proven() {
    let z6 = |report: &serde_json::Value| {
        report["layers"]
            .as_array()
            .unwrap()
            .iter()
            .find(|l| (l["tip_z_mm"].as_f64().unwrap() - 6.).abs() < 1e-6)
            .expect("a Z6 layer")["clearing"]
            .clone()
    };
    let (shallow_mean, shallow) = run_bar(12., true);
    let (deep_mean, deep) = run_bar(54., true);
    let (shallow, deep) = (z6(&shallow), z6(&deep));
    let count = |c: &serde_json::Value, key: &str| c[key].as_u64().unwrap();
    // The same strips, only deeper: entries beside the cut stock are proven
    // as on the shallow layer, not refused and replaced by helixes.
    // The same strips, only deeper: exactly the entries the shallow layer
    // proves, no more refused and none replaced by a helix.
    assert_eq!(
        count(&deep, "unproven_air_starts"),
        count(&shallow, "unproven_air_starts"),
        "deep layer refused entries the shallow one proves: {deep}"
    );
    assert_eq!(
        count(&deep, "helical_entries"),
        count(&shallow, "helical_entries")
    );
    assert!((deep_mean - shallow_mean).abs() < 0.5, "{deep_mean} {shallow_mean}");
}

// travel_clearance_mm and a clamp declared as an avoid area.

#[derive(Debug, Default)]
struct ClampStats {
    lowered: u64,
    /// Rapids the shared link guard (as `regeneration::run` applies it) refuses.
    guard_refusals: usize,
    first_refusal: Option<String>,
    /// Rapids across below the clamp top that the link guard accepts although
    /// the rectangle of their ends, grown by the widest tool radius, meets the
    /// clamp: clear on the straight route only, not on an axis-first dogleg.
    dogleg_only: usize,
    first_dogleg: Option<String>,
    /// Rapids across below the stock top whose whole cutter rectangle lies
    /// outside the stock footprint (nothing the planner knows is there).
    low_outside: usize,
    lowest_outside: f64,
}

/// The bar job with a clamp on top of the retained bar, declared as an avoid
/// area: x 26..34, y 16..24, top Z25 (the bar top is Z12, the retract Z30).
fn run_bar_clamp(clearance: Option<f64>, ends: bool) -> ClampStats {
    let clamp = [26., 16., 34., 24.];
    let clamp_top = 25.;
    // The widest tool section: the holder.
    let r = 6.;
    let mut input = bar_input(12., ends);
    input["initial_position_mm"] = serde_json::json!([0.0, 0.0, 30.0]);
    let op = &mut input["operations"][0];
    op["clearance_z_mm"] = 30.0.into();
    op["retract_z_mm"] = 30.0.into();
    op["limits"] = serde_json::json!({"avoid": [{
        "region": {
            "version": 1, "units": "mm", "name": "clamp", "boundary_error_mm": 0.0,
            "components": [{"outer": {"vertices": [
                [clamp[0], clamp[1], 0.0], [clamp[2], clamp[1], 0.0],
                [clamp[2], clamp[3], 0.0], [clamp[0], clamp[3], 0.0]]}, "holes": []}],
            "source": null
        },
        "top_z_mm": clamp_top
    }]});
    if let Some(c) = clearance {
        op["travel_clearance_mm"] = c.into();
    }
    let m: StreamManifest = serde_json::from_value(input).unwrap();
    m.validate().unwrap();
    let tool = m.tools[0].prepare().unwrap();
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut plan = OperationPlan::prepare_with_stock(
        m.operations[0].clone(),
        &m.tools[0],
        &stock,
        m.initial_position_mm,
        1,
        &|| false,
    )
    .unwrap();
    let guard =
        camlib::limits::LinkGuard::new(m.operations[0].limits(), &m.tools[0], m.sweep.tolerance_mm)
            .unwrap()
            .expect("the avoid area gives a link guard");
    let mut stats = ClampStats {
        lowest_outside: f64::INFINITY,
        ..Default::default()
    };
    let mut count = 0;
    while let Some(motion) = plan.next_motion_with_stock(&stock, &|| false).unwrap() {
        count += 1;
        assert!(count < 60000, "unbounded engagement_front path");
        let (a, b) = (motion.curve.start(), motion.curve.end());
        let refused = guard.check(&motion).err();
        if let Some(e) = &refused {
            stats.guard_refusals += 1;
            stats.first_refusal.get_or_insert_with(|| e.clone());
        }
        let across = (b[0] - a[0]).hypot(b[1] - a[1]) > 1e-9;
        if motion.kind == MoveKind::Rapid && across {
            let (x0, x1) = (a[0].min(b[0]) - r, a[0].max(b[0]) + r);
            let (y0, y1) = (a[1].min(b[1]) - r, a[1].max(b[1]) + r);
            let z = a[2].min(b[2]);
            let meets = x0 < clamp[2] && x1 > clamp[0] && y0 < clamp[3] && y1 > clamp[1];
            if z < clamp_top && meets && refused.is_none() {
                stats.dogleg_only += 1;
                stats
                    .first_dogleg
                    .get_or_insert_with(|| format!("{a:?} -> {b:?}"));
            }
            if z < 12. && (x1 < 0. || x0 > 60. || y1 < 0. || y0 > 40.) {
                stats.low_outside += 1;
                stats.lowest_outside = stats.lowest_outside.min(z);
            }
        }
        let delta = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &|| false)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
    }
    plan.finish_report(&|| false).unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    stats.lowered = report["layers"]
        .as_array()
        .unwrap()
        .iter()
        .map(|l| l["clearing"]["lowered_travels"].as_u64().unwrap())
        .sum();
    stats
}

/// Without the option every rapid crosses the clamp above its top (retract
/// Z30 > clamp top Z25). With it, a lowered rapid must not pass the clamp
/// below its top on any route a controller may take.
#[test]
fn engagement_front_travel_clearance_keeps_lowered_rapids_above_a_clamp() {
    for ends in [true, false] {
        let plain = run_bar_clamp(None, ends);
        eprintln!("ends={ends} travel_clearance_mm=None: {plain:?}");
        assert_eq!((plain.guard_refusals, plain.dogleg_only), (0, 0));
        let low = run_bar_clamp(Some(1.), ends);
        eprintln!("ends={ends} travel_clearance_mm=1: {low:?}");
        assert!(low.lowered > 0);
        assert_eq!(
            (low.guard_refusals, low.dogleg_only),
            (0, 0),
            "ends={ends}: lowered rapids pass the clamp below its top: {low:?}"
        );
    }
}
