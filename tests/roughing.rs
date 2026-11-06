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

type ClearDisk = ([f64; 2], f64);
#[derive(Default)]
struct ExposureOracle {
    cells: std::collections::HashMap<(i32, i32), Vec<ClearDisk>>,
    last: Option<ClearDisk>,
}
fn farthest(c: &Curve, a: f64, b: f64, p: [f64; 2]) -> f64 {
    let x = position(c, a);
    let y = position(c, b);
    let mut q = (x[0] - p[0])
        .hypot(x[1] - p[1])
        .max((y[0] - p[0]).hypot(y[1] - p[1]));
    if let Some((o, s)) = circle(c) {
        let t = c.start();
        let phase = (t[1] - o[1]).atan2(t[0] - o[0]);
        let r = (t[0] - o[0]).hypot(t[1] - o[1]);
        let angle = (o[1] - p[1]).atan2(o[0] - p[0]);
        let u = phase + s * a;
        let v = phase + s * b;
        if ((u.min(v) - angle) / TAU).ceil() <= ((u.max(v) - angle) / TAU).floor() {
            q = q.max((o[0] - p[0]).hypot(o[1] - p[1]) + r);
        }
    }
    q
}
impl ExposureOracle {
    fn key(p: [f64; 2]) -> (i32, i32) {
        ((p[0] / 2.).floor() as i32, (p[1] / 2.).floor() as i32)
    }
    fn add(&mut self, d: ClearDisk) {
        self.cells.entry(Self::key(d.0)).or_default().push(d);
        self.last = Some(d);
    }
    fn proves(c: &Curve, a: f64, b: f64, d: ClearDisk) -> bool {
        let q = farthest(c, a, b, d.0);
        q + 1. <= d.1 + 1e-10 || (q > 0. && (d.1 * d.1 - q * q - 1.) / (2. * q) >= 0.75 - 1e-8)
    }
    fn check(&mut self, c: &Curve) {
        let mut intervals = vec![(0., 1., 0)];
        let mut visits = 0;
        while let Some((a, b, depth)) = intervals.pop() {
            visits += 1;
            assert!(visits < 200_000, "engagement certificate work bound");
            if box_distance(bounds(c, a, b), [[0., 0.], [32., 28.]]) >= 1. {
                continue;
            }
            if self.last.is_some_and(|d| Self::proves(c, a, b, d)) {
                continue;
            }
            let p = position(c, (a + b) / 2.);
            let key = Self::key([p[0], p[1]]);
            let mut found = None;
            'search: for x in key.0 - 2..=key.0 + 2 {
                for y in key.1 - 2..=key.1 + 2 {
                    if let Some(disks) = self.cells.get(&(x, y)) {
                        for &d in disks.iter().rev() {
                            if Self::proves(c, a, b, d) {
                                found = Some(d);
                                break 'search;
                            }
                        }
                    }
                }
            }
            if found.is_some() {
                self.last = found;
                continue;
            }
            assert!(
                depth < 28,
                "whole-motion exposure exceeds 2acos(.75) at{c:?} fraction{a}..{b}"
            );
            let m = (a + b) / 2.;
            intervals.push((a, m, depth + 1));
            intervals.push((m, b, depth + 1));
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
            let mut exposure = ExposureOracle::default();
            let mut depth = 10.;
            let mut needs_floor_circle = false;
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
                if adaptive {
                    let z = motion.curve.start()[2].min(current[2]);
                    if z < depth - 1e-7 {
                        exposure = ExposureOracle::default();
                        depth = z;
                    }
                    if let Some((o, s)) = circle(&motion.curve) {
                        let p = motion.curve.start();
                        let radius = (p[0] - o[0]).hypot(p[1] - o[1]);
                        if box_distance([[o[0], o[1]], [o[0], o[1]]], [[0., 0.], [32., 28.]])
                            >= 1. + radius
                        {
                            exposure.add(([o[0], o[1]], 1. + radius));
                        }
                        if needs_floor_circle && !matches!(motion.curve, Curve::Helix { .. }) {
                            assert_eq!(motion.kind, MoveKind::Cut);
                            assert!((s.abs() - TAU).abs() < 1e-9);
                            assert_eq!(p[2], current[2]);
                        }
                    } else {
                        assert!(
                            !needs_floor_circle,
                            "helix failed to clear a real floor circle"
                        );
                    }
                    if matches!(motion.kind, MoveKind::Cut | MoveKind::Link) && !needs_floor_circle
                    {
                        exposure.check(&motion.curve);
                    }
                    needs_floor_circle = matches!(motion.curve, Curve::Helix { .. });
                    if motion.kind == MoveKind::Cut {
                        let (o, s) = circle(&motion.curve).unwrap();
                        let p = motion.curve.start();
                        assert!((s.abs() - TAU).abs() < 1e-9);
                        exposure.add(([o[0], o[1]], 1. + (p[0] - o[0]).hypot(p[1] - o[1])));
                    }
                }
                if motion.kind == MoveKind::Plunge && current[2] < 10. {
                    assert!(
                        box_distance(bounds(&motion.curve, 0., 1.), [[0., 0.], [32., 28.]]) > 1.,
                        "air entry disk hits stock"
                    );
                    outside += 1;
                }
                if let Some((_, s)) = circle(&motion.curve)
                    && matches!(motion.curve, Curve::Helix { .. })
                {
                    assert_eq!(s < 0., clockwise);
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
            assert_eq!(r["stock_edge_entries"], outside);
            assert_eq!(r["helical_entries"], helices);
            assert!(outside >= 4 && helices >= 4);
            assert_eq!(r["motions"], curves.len());
            if adaptive {
                let phi = 2. * 0.75f64.acos().to_degrees();
                assert!((r["max_engagement_deg"].as_f64().unwrap() - phi).abs() < 1e-12);
                for layer in r["layers"].as_array().unwrap() {
                    assert!(
                        layer["clearing"]["max_bounded_engagement_deg"]
                            .as_f64()
                            .unwrap()
                            <= phi
                    );
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
            3 => m.tools[0].cutter.cutting_length_mm = 4.,
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
    assert_eq!(r["max_engagement_deg"], 170.);
    assert!(1. - (170f64.to_radians() / 2.).cos() < 1.);
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

// Independent whole-disk/whole-cell oracle: scan all cell rectangles, with no
// production curve bounds, distance query, budget or empty-cut helper involved.
fn certify_empty_disk(stock: &Stock, curve: &Curve) -> ClearDisk {
    let (c, sweep) = circle(curve).unwrap();
    assert!((sweep.abs() - TAU).abs() < 1e-9);
    assert_eq!(curve.start(), curve.end());
    let p = curve.start();
    let radius = 1. + (p[0] - c[0]).hypot(p[1] - c[1]);
    let half = stock.cell_widths_mm().map(|x| x / 2.);
    for (i, cell) in stock.cells().iter().enumerate() {
        let q = stock.cell_center(i).unwrap();
        let distance = ((c[0] - q[0]).abs() - half[0])
            .max(0.)
            .hypot(((c[1] - q[1]).abs() - half[1]).max(0.));
        if distance <= radius + 1e-8 {
            assert!(
                cell.upper_mm <= p[2],
                "omitted disk overlaps high/uncertain cell{i}"
            );
        }
    }
    ([c[0], c[1]], radius)
}

#[test]
fn empty_cut_policy_preserves_material_pose_entries_and_exposure_across_strategies() {
    use camlib::roughing::RoughingClearing;
    for (strategy, clockwise, angle) in [
        ("raster", true, 0f64),
        ("offset", true, 0.),
        ("offset", false, 0.61),
        ("adaptive", true, 0.),
        ("adaptive", false, 0.),
    ] {
        let mut m = fixture(strategy == "adaptive");
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
        let mut exposure = ExposureOracle::default();
        let mut depth = 10.;
        let mut replay_stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let replay_tool = m.tools[0].prepare().unwrap();
        let cancel = AtomicBool::new(false);
        let mut old_at = 0;
        for (i, motion) in new.motions.iter().enumerate() {
            certify_part(&rotate_curve(motion.curve.clone(), -angle));
            if let Curve::Helix { sweep_rad, .. } = motion.curve
                && !matches!(new.motions[i + 1].curve, Curve::Helix { .. })
            {
                assert_eq!(sweep_rad < 0., clockwise);
                let next = &new.motions[i + 1];
                assert_eq!(
                    next.kind,
                    if strategy == "adaptive" {
                        MoveKind::Cut
                    } else {
                        MoveKind::Lead
                    }
                );
                assert!((circle(&next.curve).unwrap().1.abs() - TAU).abs() < 1e-9);
                assert_eq!(next.curve.start()[2], next.curve.end()[2]);
            }
            if strategy == "adaptive" {
                let z = motion.curve.start()[2].min(motion.curve.end()[2]);
                if z < depth - 1e-7 {
                    exposure = ExposureOracle::default();
                    depth = z;
                }
                // The original oracle only knew executed clearing disks.
                // Independently prove every newly omitted baseline orbit empty
                // before admitting it as an exposure premise. Other original
                // primitives must remain byte-identical and in order.
                while old.motions[old_at].curve != motion.curve {
                    let omitted = &old.motions[old_at];
                    assert_eq!(omitted.kind, MoveKind::Cut);
                    exposure.add(certify_empty_disk(&replay_stock, &omitted.curve));
                    old_at += 1;
                }
                assert_eq!(old.motions[old_at].kind, motion.kind);
                assert_eq!(old.motions[old_at].feed_mm_min, motion.feed_mm_min);
                old_at += 1;
                if let Some((o, _)) = circle(&motion.curve) {
                    let p = motion.curve.start();
                    let radius = (p[0] - o[0]).hypot(p[1] - o[1]);
                    if box_distance([[o[0], o[1]], [o[0], o[1]]], [[0., 0.], [32., 28.]])
                        >= 1. + radius
                    {
                        exposure.add(([o[0], o[1]], 1. + radius));
                    }
                }
                let seed = i > 0 && matches!(new.motions[i - 1].curve, Curve::Helix { .. });
                if matches!(motion.kind, MoveKind::Cut | MoveKind::Link) && !seed {
                    exposure.check(&motion.curve);
                }
                if motion.kind == MoveKind::Cut {
                    let (o, _) = circle(&motion.curve).unwrap();
                    let p = motion.curve.start();
                    exposure.add(([o[0], o[1]], 1. + (p[0] - o[0]).hypot(p[1] - o[1])));
                }
                let delta = replay_stock
                    .preview_sweep(&replay_tool, motion, 1., m.sweep, &cancel)
                    .unwrap();
                replay_stock.apply_delta(&delta).unwrap();
            }
        }
        if strategy == "adaptive" {
            assert_eq!(old_at, old.motions.len());
            assert_eq!(replay_stock.cells(), new.stock.cells());
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
                    if let RoughingClearing::Adaptive {
                        boundary_tolerance_mm,
                        ..
                    } = c
                    {
                        *boundary_tolerance_mm = 0.
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
                    if let RoughingClearing::Adaptive { cutting_sense, .. } = c {
                        *cutting_sense = Some(camlib::adaptive::CuttingSense::Climb)
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
    let target = PreparedRoughingTarget::prepare(
        mesh(separately_tessellated_boss(11.), 0.01),
        1e-5,
        &cancel,
    )
    .unwrap();
    // Without a cutter the exact union keeps slivers between the two faces'
    // edges; one touches another boundary within the tolerance, and the
    // refusal says where and why.
    let refused = target.shadow(11., 0.2, &mut 0, &cancel).err().unwrap();
    assert!(refused.contains("intersect or touch near ("), "{refused}");
    assert!(refused.contains("not watertight"), "{refused}");
    // For a 12 mm cutter the slivers are filled: no cutter disk fits in one.
    let shadow = target
        .shadow_for_cutter(11., 0.2, 6., &mut 0, &cancel)
        .unwrap();
    assert!(shadow.report.filled_holes > 0);
    // Lenses between the two faces' chords along the radius 3 circle only.
    assert!(
        shadow.report.filled_hole_area_mm2 < 0.5,
        "{:?}",
        shadow.report
    );
    let region = shadow
        .region
        .prepare(ConstructionOptions::default())
        .unwrap();
    assert_eq!(
        (region.summary().components, region.summary().holes),
        (1, 0)
    );
    // Every projected facet point stays protected (filling only adds area).
    for triangle in separately_tessellated_boss(11.) {
        for p in triangle {
            assert!(signed(&region, [p[0], p[1]]) > -1e-4, "{p:?}");
        }
    }
    assert!(signed(&region, [0., 0.]) > 1.);
    assert!(signed(&region, [3.2, 0.]) < 0.);
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
