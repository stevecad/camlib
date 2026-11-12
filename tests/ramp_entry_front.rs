//! Ramp entry (`RampEntry`) in adaptive clearing (#219): `AdaptivePocket`
//! planned by the engagement front. Checked from the emitted motions alone
//! against an independent height map (not the engine's stock): what is
//! material, how steep each descent is, and how much each move removes.
use camlib::{
    adaptive::*, motion::*, operation::OperationPlan, pocket::*, stock::*, stream::StreamManifest,
};
use std::sync::atomic::AtomicBool;

const RADIUS: f64 = 2.;
const CELL: f64 = 0.1;

/// Column tops on a fine grid, cut by a flat cutter of `RADIUS`.
struct Heights {
    nx: usize,
    ny: usize,
    top: Vec<f64>,
}
impl Heights {
    fn new(max: [f64; 3]) -> Self {
        let (nx, ny) = ((max[0] / CELL) as usize, (max[1] / CELL) as usize);
        Self {
            nx,
            ny,
            top: vec![max[2]; nx * ny],
        }
    }
    fn cells(&self, p: [f64; 2], reach: f64) -> Vec<usize> {
        let span = |v: f64, n: usize| {
            let lo = ((v - reach) / CELL).floor().max(0.) as usize;
            let hi = (((v + reach) / CELL).ceil().max(0.) as usize).min(n);
            lo..hi
        };
        let mut found = vec![];
        for j in span(p[1], self.ny) {
            for i in span(p[0], self.nx) {
                let c = [(i as f64 + 0.5) * CELL, (j as f64 + 0.5) * CELL];
                if (c[0] - p[0]).hypot(c[1] - p[1]) <= reach {
                    found.push(j * self.nx + i);
                }
            }
        }
        found
    }
    /// The highest material under the cutter at `p`, well inside its edge.
    fn under(&self, p: [f64; 2]) -> f64 {
        self.cells(p, RADIUS - 3. * CELL)
            .into_iter()
            .map(|i| self.top[i])
            .fold(f64::NEG_INFINITY, f64::max)
    }
    /// Cut at `p`; returns the plan area (mm2) of the columns it lowered by
    /// more than a hair.
    fn cut(&mut self, p: Vec3) -> f64 {
        let mut area = 0.;
        for i in self.cells([p[0], p[1]], RADIUS) {
            if self.top[i] > p[2] + 1e-3 {
                area += CELL * CELL;
            }
            self.top[i] = self.top[i].min(p[2]);
        }
        area
    }
    fn at(&self, p: [f64; 2]) -> f64 {
        self.top[(p[1] / CELL) as usize * self.nx + (p[0] / CELL) as usize]
    }
}
/// Points along a curve, at most 0.04 mm apart, ends included.
fn samples(c: &Curve) -> Vec<Vec3> {
    let path = c.prepare(1e-5).unwrap();
    let n = (path.length_mm() / 0.04).ceil().max(1.) as usize;
    (0..=n)
        .map(|i| match i {
            0 => c.start(),
            i if i == n => c.end(),
            i => path.position(i as f64 / n as f64).unwrap(),
        })
        .collect()
}
fn horizontal(c: &Curve) -> f64 {
    let (a, b) = (c.start(), c.end());
    match *c {
        Curve::Line { .. } => (a[0] - b[0]).hypot(a[1] - b[1]),
        Curve::Arc {
            center, sweep_rad, ..
        }
        | Curve::Helix {
            center, sweep_rad, ..
        } => (a[0] - center[0]).hypot(a[1] - center[1]) * sweep_rad.abs(),
    }
}
fn fixture() -> StreamManifest {
    serde_json::from_str(include_str!("../fixtures/operations/adaptive-pocket.json")).unwrap()
}
fn ramp(angle_deg: f64, min_length_mm: Option<f64>) -> PocketEntry {
    PocketEntry::Ramp(RampEntry {
        angle_rad: angle_deg.to_radians(),
        min_length_mm,
        feed_mm_min: None,
    })
}
struct Seen {
    motions: Vec<Motion>,
    heights: Heights,
    stock: Stock,
    report: serde_json::Value,
    /// Sloped descents, and how far above the material under its own path
    /// the highest of them started.
    ramps: usize,
    air_above_ramp: f64,
    /// Largest plan area removed per mm of travel over a stretch of at
    /// least 1 mm of cutting moves, and of entry (lead) moves.
    cut_rate: f64,
    lead_rate: f64,
}
/// Plan the adaptive pocket of the L fixture after `edit`, committing every
/// motion, and check each move against the independent height map.
fn run(angle_deg: f64, edit: impl FnOnce(&mut AdaptivePocketOperation)) -> Result<Seen, String> {
    run_with(fixture(), angle_deg, edit)
}
fn run_with(
    m: StreamManifest,
    angle_deg: f64,
    edit: impl FnOnce(&mut AdaptivePocketOperation),
) -> Result<Seen, String> {
    let cancel = AtomicBool::new(false);
    let Operation::AdaptivePocket(mut op) = m.operations[0].clone() else {
        panic!("fixture")
    };
    op.path_strategy = AdaptivePathStrategy::EngagementFront;
    edit(&mut op);
    // The wire format round-trips the entry.
    let wire = serde_json::to_value(&op).unwrap();
    let op: AdaptivePocketOperation = serde_json::from_value(wire).map_err(|e| e.to_string())?;
    let entry_feed = op.pocket.plunge_mm_min;
    let spec = m.stock.height().unwrap();
    let mut stock = Stock::new(spec.clone()).unwrap();
    let mut plan = OperationPlan::prepare_with_stock(
        Operation::AdaptivePocket(op),
        &m.tools[0],
        &stock,
        m.initial_position_mm,
        1,
        &cancel,
    )?;
    let tool = m.tools[0].prepare().unwrap();
    let mut heights = Heights::new(spec.max_mm);
    let limit = angle_deg.to_radians().tan();
    let mut position = m.initial_position_mm;
    let (mut motions, mut ramps, mut air_above_ramp) = (vec![], 0, 0f64);
    // (area, travel) of the open stretch of cutting and of lead moves.
    let mut window = [(0f64, 0f64); 2];
    let mut rate = [0f64; 2];
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel)? {
        assert!(motions.len() < 100_000);
        let c = &motion.curve;
        assert_eq!(c.start(), position, "discontinuous {motion:?}");
        position = c.end();
        let (a, b) = (c.start(), c.end());
        let travel = horizontal(c);
        assert!(!matches!(c, Curve::Helix { .. }), "a helix: {motion:?}");
        if b[2] < a[2] {
            if travel == 0. {
                // A straight descent: nothing under the cutter above its end.
                let under = heights.under([b[0], b[1]]);
                assert!(
                    under <= b[2] + 0.02,
                    "straight descent into material (top {under}) by {motion:?}"
                );
            } else {
                let slope = (a[2] - b[2]) / travel;
                assert!(
                    slope <= limit * (1. + 1e-9),
                    "ramp steeper than {angle_deg} degrees: {motion:?}"
                );
                assert_eq!(motion.kind, MoveKind::Lead, "{motion:?}");
                assert_eq!(motion.feed_mm_min, Some(entry_feed), "{motion:?}");
                // The material under the leg when it starts.
                let under = samples(c)
                    .iter()
                    .map(|p| heights.under([p[0], p[1]]))
                    .fold(f64::NEG_INFINITY, f64::max);
                air_above_ramp = air_above_ramp.max(a[2] - under);
                ramps += 1;
            }
        } else if motion.kind == MoveKind::Rapid {
            assert!(a[2].min(b[2]) >= spec.max_mm[2], "{motion:?}");
        }
        let which = match motion.kind {
            MoveKind::Cut | MoveKind::Link => Some(0),
            MoveKind::Lead if a[2] == b[2] => Some(1),
            _ => None,
        };
        let mut last = c.start();
        for p in samples(c) {
            let area = heights.cut(p);
            if let Some(w) = which {
                window[w].0 += area;
                window[w].1 += (p[0] - last[0]).hypot(p[1] - last[1]);
                if window[w].1 >= 1. {
                    rate[w] = rate[w].max(window[w].0 / window[w].1);
                    window[w] = (0., 0.);
                }
            }
            last = p;
        }
        let delta = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        motions.push(motion);
    }
    plan.finish_report(&cancel).unwrap();
    Ok(Seen {
        motions,
        heights,
        stock,
        report: serde_json::to_value(plan.report()).unwrap(),
        ramps,
        air_above_ramp,
        cut_rate: rate[0],
        lead_rate: rate[1],
    })
}

/// The most plan area a cutter of `RADIUS` can remove per mm of travel with
/// a contact angle of `limit_deg`: the chord of that arc, head on. A full
/// slot removes the diameter.
fn chord(limit_deg: f64) -> f64 {
    2. * RADIUS * (limit_deg.to_radians() / 2.).sin()
}
/// What every ramped run must show, whatever the region: the report agrees
/// with the motions, no helix was made, no ramp runs above the material,
/// and full-width cutting is entry motion only.
fn check_entries(seen: &Seen, angle_deg: f64) {
    let limit = (1f64 - 1.5 / RADIUS).acos().to_degrees();
    let report = &seen.report;
    assert_eq!(report["helical_entries"], 0);
    let mut legs = 0;
    for layer in report["layers"].as_array().unwrap() {
        let c = &layer["clearing"];
        assert_eq!(c["ramp_entries"], 1, "{c}");
        assert_eq!(c["helical_entries"], 0);
        assert!(c["ramp_slot_mm"].as_f64().unwrap() > 0.);
        assert!(c["max_ramp_angle_deg"].as_f64().unwrap() <= angle_deg + 1e-9);
        // Every cutting stroke after the ramp is proven within the limit.
        assert!((c["engagement_limit_deg"].as_f64().unwrap() - limit).abs() < 1e-9);
        assert!(c["proven_max_engagement_deg"].as_f64().unwrap() <= limit);
        assert!(c["engagement_proved_strokes"].as_u64().unwrap() > 0);
        legs += c["ramp_legs"].as_u64().unwrap() as usize;
    }
    assert_eq!(report["ramp_entries"], 2);
    assert!(report["max_ramp_angle_deg"].as_f64().unwrap() <= angle_deg + 1e-9);
    // The sloped descents seen are exactly the legs reported.
    assert_eq!(seen.ramps, legs);
    // No leg starts above the material under its own path (a hundredth of a
    // millimetre of skin, and the map's own blur): the second level's ramp
    // starts on the first level's floor, not at the pocket top.
    assert!(
        seen.air_above_ramp <= 0.05,
        "a ramp starts {} mm above the material",
        seen.air_above_ramp
    );
    // Independent of the planner's proof: no stretch of cutting or link
    // moves removes more than a contact angle within the limit can, while
    // the ramp's flat pass is the full slot it is reported to be.
    assert!(
        seen.cut_rate <= chord(limit) * 1.05 + 0.05,
        "cutting moves remove {} mm2/mm, a contact within {limit:.1} degrees at most {}",
        seen.cut_rate,
        chord(limit)
    );
    assert!(seen.lead_rate > 2. * RADIUS * 0.95, "{}", seen.lead_rate);
    // Entry moves run at the entry feed, cutting moves at the cutting feed.
    for motion in &seen.motions {
        match motion.kind {
            MoveKind::Lead | MoveKind::Plunge => assert_eq!(motion.feed_mm_min, Some(150.)),
            MoveKind::Cut | MoveKind::Link => assert_eq!(motion.feed_mm_min, Some(600.)),
            _ => {}
        }
    }
}

/// The L pocket (two 1 mm levels, 4 mm cutter, 1.5 mm stepover): a helix
/// fits here, and a `RampEntry` replaces it.
#[test]
fn adaptive_pocket_ramps_into_enclosed_material_and_proves_every_cut_after_it() {
    let dist = |a: [f64; 2], b: [f64; 2]| (a[0] - b[0]).hypot(a[1] - b[1]);
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
    let seen = run(5., |op| op.pocket.entry = Some(ramp(5., None))).unwrap();
    check_entries(&seen, 5.);
    // 1 mm at 5 degrees needs 11.4 mm and the L has it: one leg a level, as
    // steep as declared and no longer than it has to be.
    for layer in seen.report["layers"].as_array().unwrap() {
        let c = &layer["clearing"];
        assert_eq!(c["ramp_legs"], 1);
        assert!((c["max_ramp_angle_deg"].as_f64().unwrap() - 5.).abs() < 1e-6);
        let slot = c["ramp_slot_mm"].as_f64().unwrap();
        assert!(slot <= 1.01 / 5f64.to_radians().tan(), "{slot} mm of slot");
    }
    // At depth the cutter stays inside the L and off its walls.
    for motion in &seen.motions {
        if motion.curve.start()[2].min(motion.curve.end()[2]) >= 10. {
            continue;
        }
        for p in samples(&motion.curve) {
            let q = [p[0], p[1]];
            assert!(
                inside(q) && to_wall(q) >= RADIUS - 1e-6 && p[2] >= 8.,
                "cutter in the wall at {p:?}"
            );
        }
    }
    // Everything the cutter can reach is at the floor (the sloped floors
    // the ramps left included), in the engine's stock and the independent
    // map; nothing outside the pocket is touched.
    for (i, cell) in seen.stock.cells().iter().enumerate() {
        let p = seen.stock.cell_center(i).unwrap();
        let corner = corners.iter().any(|c| dist(*c, p) < 2.5);
        if inside(p) && to_wall(p) > 0.3 && !corner {
            assert!(cell.upper_mm <= 8. + 1e-4, "uncut at {p:?}: {cell:?}");
            assert!(seen.heights.at(p) <= 8. + 1e-9, "uncut at {p:?}");
        } else if !inside(p) && to_wall(p) > 0.6 {
            assert_eq!(cell.lower_mm, 10., "cut outside the pocket at {p:?}");
            assert_eq!(seen.heights.at(p), 10., "cut outside the pocket at {p:?}");
        }
    }
    // A job that asks for no ramp reports none: its report has no new field.
    let m = fixture();
    let Operation::AdaptivePocket(mut op) = m.operations[0].clone() else {
        panic!("fixture")
    };
    op.path_strategy = AdaptivePathStrategy::EngagementFront;
    let plan = OperationPlan::prepare(
        Operation::AdaptivePocket(op),
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &AtomicBool::new(false),
    )
    .unwrap();
    let plain = serde_json::to_string(&plan.report()).unwrap();
    assert!(!plain.contains("ramp_"), "{plain}");
}

/// A slot 20 x 5 mm for a 4 mm cutter: no helix fits.
fn slot(op: &mut AdaptivePocketOperation) {
    let component = &mut op.pocket.region.components[0];
    component.holes.clear();
    component.outer.vertices = vec![
        [23., 9.5, 0.],
        [3., 9.5, 0.],
        [3., 14.5, 0.],
        [23., 14.5, 0.],
    ];
}

#[test]
fn adaptive_pocket_ramps_along_a_slot_no_helix_fits() {
    let check_floor = |seen: &Seen| {
        for motion in &seen.motions {
            if motion.kind == MoveKind::Rapid {
                continue;
            }
            for p in samples(&motion.curve) {
                assert!(
                    p[0] >= 5. - 1e-6
                        && p[0] <= 21. + 1e-6
                        && p[1] >= 11.5 - 1e-6
                        && p[1] <= 12.5 + 1e-6
                        && p[2] >= 8.,
                    "outside the slot: {motion:?}"
                );
            }
        }
        let mut checked = 0;
        for (i, cell) in seen.stock.cells().iter().enumerate() {
            let p = seen.stock.cell_center(i).unwrap();
            let within =
                |m: f64| p[0] > 3. + m && p[0] < 23. - m && p[1] > 9.5 + m && p[1] < 14.5 - m;
            // Away from the corner radii the cutter leaves.
            if within(0.3) && p[0] >= 5. && p[0] <= 21. {
                assert!(cell.upper_mm <= 8. + 1e-4, "uncut at {p:?}: {cell:?}");
                assert!(seen.heights.at(p) <= 8. + 1e-9, "uncut at {p:?}");
                checked += 1;
            } else if !within(-0.6) {
                assert_eq!(cell.lower_mm, 10., "cut outside the slot at {p:?}");
            }
        }
        assert!(checked > 300, "{checked}");
    };
    let reversals = |seen: &Seen| {
        seen.motions
            .windows(2)
            .filter(|w| {
                let sloped =
                    |m: &Motion| m.curve.start()[2] > m.curve.end()[2] && horizontal(&m.curve) > 0.;
                let x = |m: &Motion| m.curve.end()[0] - m.curve.start()[0];
                sloped(&w[0]) && sloped(&w[1]) && x(&w[0]) * x(&w[1]) < 0.
            })
            .count()
    };
    // The helix this pocket would otherwise need has no room: nothing at
    // all is cut (reported as residual area), which is why a slot needs a
    // ramp.
    let helix = run(4., slot);
    assert!(helix.unwrap().motions.is_empty());
    // 1 mm at 4 degrees needs 14.3 mm of the slot's 16: one leg.
    let seen = run(4., |op| {
        slot(op);
        op.pocket.entry = Some(ramp(4., None));
    })
    .unwrap();
    check_entries(&seen, 4.);
    assert_eq!(reversals(&seen), 0);
    check_floor(&seen);
    // At 2 degrees it needs 28.6 mm: back and forth over the whole slot,
    // every leg shallower than declared, which the tool must be declared
    // able to do.
    let seen = run(2., |op| {
        slot(op);
        op.pocket.entry = Some(ramp(2., Some(2.)));
    })
    .unwrap();
    check_entries(&seen, 2.);
    assert_eq!(reversals(&seen), 2);
    for layer in seen.report["layers"].as_array().unwrap() {
        assert_eq!(layer["clearing"]["ramp_legs"], 2);
    }
    check_floor(&seen);
    // Not declared, or declared longer than the slot: refused by name before
    // any motion, not left uncut and not plunged.
    for (entry, part) in [
        (ramp(2., None), "min_length_mm is not set"),
        (ramp(2., Some(18.)), "shorter than min_length_mm 18.0000"),
    ] {
        let error = run(2., |op| {
            slot(op);
            op.pocket.entry = Some(entry);
        })
        .err()
        .unwrap();
        for part in [
            "no ramp entry into the enclosed material",
            "level at Z9",
            part,
        ] {
            assert!(error.contains(part), "{error}");
        }
    }
}

#[test]
fn adaptive_ramp_refusals_name_the_reason() {
    // The orbit strategies start from a fitted seed circle.
    for strategy in [
        AdaptivePathStrategy::DiskFront,
        AdaptivePathStrategy::StockFront,
    ] {
        let error = run(4., |op| {
            op.pocket.entry = Some(ramp(4., Some(2.)));
            op.path_strategy = strategy;
        })
        .err()
        .unwrap();
        for part in [
            "RampEntry is not supported here",
            "seed circle",
            "engagement_front",
        ] {
            assert!(error.contains(part), "{error}");
        }
    }
    // An invalid ramp is refused before planning.
    let error = run(4., |op| op.pocket.entry = Some(ramp(0., None)))
        .err()
        .unwrap();
    assert!(error.contains("invalid ramp entry"), "{error}");
    // A cutter that is not center cutting is refused with a ramp too.
    let cancel = AtomicBool::new(false);
    let mut m = fixture();
    m.tools[0].cutter.center_cutting = false;
    let Operation::AdaptivePocket(op) = &mut m.operations[0] else {
        panic!("fixture")
    };
    op.path_strategy = AdaptivePathStrategy::EngagementFront;
    op.pocket.entry = Some(ramp(4., None));
    let error = OperationPlan::prepare(
        m.operations[0].clone(),
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &cancel,
    )
    .map(|_| ())
    .unwrap_err();
    assert!(
        error.contains("a RampEntry does not lift the need for a center-cutting end mill"),
        "{error}"
    );
}

/// Flutes shorter than the pocket is deep: the second level's ramp runs with
/// the shank (as wide as the cutter) in what the first level cut. The
/// planner proves that from the committed cuts before it ramps, and the
/// stock sweep (which refuses a shank in material) accepts every move.
#[test]
fn adaptive_ramp_below_the_flutes_has_its_shank_proven_clear() {
    let mut m = fixture();
    m.tools[0].cutter.cutting_length_mm = 1.5;
    let seen = run_with(m, 5., |op| op.pocket.entry = Some(ramp(5., None))).unwrap();
    check_entries(&seen, 5.);
    // The second level's ramp is below the first level's floor.
    let lowest = seen
        .motions
        .iter()
        .filter(|m| m.curve.end()[2] < m.curve.start()[2] && horizontal(&m.curve) > 0.)
        .map(|m| m.curve.end()[2])
        .fold(f64::INFINITY, f64::min);
    assert_eq!(lowest, 8.);
}
