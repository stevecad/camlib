//! Ramp entry (`RampEntry`) for Pocket and Contour, checked from the emitted
//! motions alone: an independent height map (not the engine's stock) decides
//! what is material, and the declared region, cutter and angle are the only
//! other inputs.
use camlib::{
    contour::ContourOperation, motion::*, operation::OperationPlan, pocket::*, stock::*,
    stream::StreamManifest,
};
use std::sync::atomic::AtomicBool;

const RADIUS: f64 = 2.;
const CELL: f64 = 0.1;

/// The independent stock: a grid of column tops, cut by a flat cutter of
/// `RADIUS` sampled densely along every move.
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
    /// The highest material under the cutter at `p`, well inside its edge
    /// (the grid and the sampling blur the edge by less than this margin).
    fn under(&self, p: [f64; 2]) -> f64 {
        self.cells(p, RADIUS - 3. * CELL)
            .into_iter()
            .map(|i| self.top[i])
            .fold(f64::NEG_INFINITY, f64::max)
    }
    fn cut(&mut self, p: Vec3) {
        for i in self.cells([p[0], p[1]], RADIUS) {
            self.top[i] = self.top[i].min(p[2]);
        }
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
struct Seen {
    ramps: usize,
    steepest: f64,
    heights: Heights,
    motions: Vec<Motion>,
}
/// Run the operation, committing every motion to the engine's stock, and
/// check every descending move against the independent height map:
/// a straight descent only through air, and a sloped one no steeper than
/// `angle_deg`, at `ramp_feed`.
fn run(m: &StreamManifest, angle_deg: f64, ramp_feed: f64) -> (Seen, Stock) {
    let spec = m.stock.height().unwrap();
    let stock = Stock::new(spec.clone()).unwrap();
    run_on(m, angle_deg, ramp_feed, stock, Heights::new(spec.max_mm))
}
/// `run` on stock earlier operations have cut (and the independent map of
/// the same cuts), planned from that stock as a stream plans it.
fn run_on(
    m: &StreamManifest,
    angle_deg: f64,
    ramp_feed: f64,
    mut stock: Stock,
    heights: Heights,
) -> (Seen, Stock) {
    let cancel = AtomicBool::new(false);
    let spec = m.stock.height().unwrap();
    let mut plan = OperationPlan::prepare_with_stock(
        m.operations[0].clone(),
        &m.tools[0],
        &stock,
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    let tool = m.tools[0].prepare().unwrap();
    let mut seen = Seen {
        ramps: 0,
        steepest: 0.,
        heights,
        motions: vec![],
    };
    let limit = angle_deg.to_radians().tan();
    let mut position = m.initial_position_mm;
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        let c = &motion.curve;
        assert_eq!(c.start(), position, "discontinuous {motion:?}");
        position = c.end();
        let (a, b) = (c.start(), c.end());
        let travel = horizontal(c);
        if b[2] < a[2] {
            if travel == 0. {
                // A straight descent: nothing under the cutter above its end.
                let under = seen.heights.under([b[0], b[1]]);
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
                assert_eq!(motion.kind, MoveKind::Plunge, "{motion:?}");
                assert_eq!(motion.feed_mm_min, Some(ramp_feed), "{motion:?}");
                seen.ramps += 1;
                seen.steepest = seen.steepest.max(slope);
            }
        } else if motion.kind == MoveKind::Rapid {
            // Rapids stay above everything.
            assert!(a[2].min(b[2]) >= spec.max_mm[2], "{motion:?}");
        }
        for p in samples(c) {
            seen.heights.cut(p);
        }
        let delta = stock
            .preview_sweep(&tool, &motion, 1., SweepOptions::default(), &cancel)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        seen.motions.push(motion);
    }
    (seen, stock)
}

fn pocket_manifest(region: [[f64; 2]; 2], entry: Option<PocketEntry>) -> StreamManifest {
    let mut m: StreamManifest =
        serde_json::from_str(include_str!("../fixtures/operations/helical-pocket.json")).unwrap();
    let Operation::Pocket(p) = &mut m.operations[0] else {
        panic!("fixture");
    };
    let [lo, hi] = region;
    let component = &mut p.region.components[0];
    component.holes.clear();
    component.outer.vertices = vec![
        [hi[0], lo[1], 0.],
        [lo[0], lo[1], 0.],
        [lo[0], hi[1], 0.],
        [hi[0], hi[1], 0.],
    ];
    p.id = "slot".into();
    p.entry = entry;
    m
}
fn ramp(angle_deg: f64) -> RampEntry {
    RampEntry {
        angle_rad: angle_deg.to_radians(),
        min_length_mm: None,
        feed_mm_min: None,
    }
}
/// A ramp that may reverse on spans of `min_length_mm` or more.
fn zigzag(angle_deg: f64, min_length_mm: f64) -> RampEntry {
    RampEntry {
        min_length_mm: Some(min_length_mm),
        ..ramp(angle_deg)
    }
}
// A slot 26 x 5 mm for a 4 mm cutter, 4 mm deep in two levels.
const SLOT: [[f64; 2]; 2] = [[3., 9.5], [29., 14.5]];

fn check_pocket(mut entry: RampEntry, feed: Option<f64>, zigzag: bool) {
    let angle_deg = entry.angle_rad.to_degrees();
    entry.feed_mm_min = feed;
    let m = pocket_manifest(SLOT, Some(PocketEntry::Ramp(entry)));
    m.validate().unwrap();
    let (seen, stock) = run(&m, angle_deg, feed.unwrap_or(150.));
    check_slot(&seen, &stock, zigzag, 6., Some(10.));
}
/// The slot's checks on a finished run: cutter inside, reversals as
/// expected, floor at `floor` everywhere, and (given `outside`) everything
/// beside the slot still at that height.
fn check_slot(seen: &Seen, stock: &Stock, zigzag: bool, floor: f64, outside: Option<f64>) {
    // Every fed move keeps the cutter inside the slot: the region is convex
    // and every move a straight line, so its ends decide.
    for motion in &seen.motions {
        if motion.kind == MoveKind::Rapid {
            continue;
        }
        assert!(matches!(motion.curve, Curve::Line { .. }), "{motion:?}");
        for p in [motion.curve.start(), motion.curve.end()] {
            assert!(
                p[0] >= SLOT[0][0] + RADIUS - 1e-9
                    && p[0] <= SLOT[1][0] - RADIUS + 1e-9
                    && p[1] >= SLOT[0][1] + RADIUS - 1e-9
                    && p[1] <= SLOT[1][1] - RADIUS + 1e-9
                    && p[2] >= floor - 1e-12,
                "outside the slot: {motion:?}"
            );
        }
    }
    // Ramps were made, in both levels, and they turn around on the slot
    // exactly when the angle needs more travel than it has.
    assert!(seen.ramps >= 2, "{} ramps", seen.ramps);
    let reversals = seen
        .motions
        .windows(2)
        .filter(|w| {
            let slope =
                |m: &Motion| m.curve.start()[2] > m.curve.end()[2] && horizontal(&m.curve) > 0.;
            let x = |m: &Motion| m.curve.end()[0] - m.curve.start()[0];
            slope(&w[0]) && slope(&w[1]) && x(&w[0]) * x(&w[1]) < 0.
        })
        .count();
    assert_eq!(reversals > 0, zigzag, "{reversals} reversals");
    // The whole floor is at depth afterwards (the ramps' sloped floors
    // included), in the independent map and in the engine's stock.
    let mut checked = 0;
    for j in 0..seen.heights.ny {
        for i in 0..seen.heights.nx {
            let q = [(i as f64 + 0.5) * CELL, (j as f64 + 0.5) * CELL];
            let inside = |margin: f64| {
                q[0] >= SLOT[0][0] + margin
                    && q[0] <= SLOT[1][0] - margin
                    && q[1] >= SLOT[0][1] + margin
                    && q[1] <= SLOT[1][1] - margin
            };
            // Away from the corner radii the cutter leaves.
            if inside(0.2) && q[0] >= SLOT[0][0] + RADIUS && q[0] <= SLOT[1][0] - RADIUS {
                assert_eq!(seen.heights.at(q), floor, "floor left high at {q:?}");
                checked += 1;
            }
            if let Some(level) = outside
                && !inside(-0.2)
            {
                assert_eq!(seen.heights.at(q), level, "cut outside the slot at {q:?}");
            }
        }
    }
    assert!(checked > 5000);
    for (i, h) in stock.cells().iter().enumerate() {
        let q = stock.cell_center(i).unwrap();
        if q[0] >= SLOT[0][0] + RADIUS
            && q[0] <= SLOT[1][0] - RADIUS
            && q[1] >= SLOT[0][1] + 0.5
            && q[1] <= SLOT[1][1] - 0.5
        {
            assert!(
                h.upper_mm < floor + 0.003,
                "engine stock left high at {q:?}: {h:?}"
            );
        }
    }
}

#[test]
fn pocket_ramp_zigzags_on_a_short_slot_and_cuts_the_floor() {
    // 2 mm at 4 degrees needs 28.6 mm; the slot's passes are 22 mm long.
    // The reversal needs the declared minimum span (2 mm here).
    check_pocket(zigzag(4., 2.), None, true);
}

#[test]
fn pocket_ramp_is_one_leg_at_the_declared_angle_when_the_pass_is_long_enough() {
    // 2 mm at 10 degrees needs 11.3 mm; its own feed is used. A ramp that
    // never reverses needs no min_length_mm.
    check_pocket(ramp(10.), Some(90.), false);
    let m = pocket_manifest(SLOT, Some(PocketEntry::Ramp(ramp(10.))));
    let (seen, _) = run(&m, 10., 150.);
    // Not shallower than asked either: the ramp is as short as the angle allows.
    assert!(
        (seen.steepest - 10f64.to_radians().tan()).abs() < 1e-3,
        "{}",
        seen.steepest
    );
}

#[test]
fn pocket_ramp_report_and_default_serialisation() {
    let cancel = AtomicBool::new(false);
    let report = |entry: Option<PocketEntry>| {
        let m = pocket_manifest(SLOT, entry);
        let spec = m.stock.height().unwrap().clone();
        let Operation::Pocket(p) = m.operations[0].clone() else {
            panic!("fixture");
        };
        let mut plan =
            PocketPlan::prepare(p, &m.tools[0], &spec, m.initial_position_mm, 1, &cancel).unwrap();
        let tool = m.tools[0].prepare().unwrap();
        let mut stock = Stock::new(spec).unwrap();
        while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
            let delta = stock
                .preview_sweep(&tool, &motion, 1., SweepOptions::default(), &cancel)
                .unwrap();
            stock.apply_delta(&delta).unwrap();
        }
        serde_json::to_value(plan.report()).unwrap()
    };
    let ramped = report(Some(PocketEntry::Ramp(zigzag(4., 2.))));
    assert!(ramped["ramp_entries"].as_u64().unwrap() >= 2, "{ramped}");
    let angle = ramped["max_ramp_angle_deg"].as_f64().unwrap();
    assert!(angle > 0. && angle <= 4. + 1e-9, "{angle}");
    // Without the entry nothing about it is reported or serialised.
    let plain = report(None);
    assert!(plain.get("ramp_entries").is_none() && plain.get("max_ramp_angle_deg").is_none());
    let m = pocket_manifest(SLOT, None);
    assert!(
        serde_json::to_value(&m.operations[0])
            .unwrap()
            .get("entry")
            .is_none()
    );
    let m = pocket_manifest(SLOT, Some(PocketEntry::Ramp(ramp(4.))));
    let entry = serde_json::to_value(&m.operations[0]).unwrap()["entry"].clone();
    assert_eq!(
        entry,
        serde_json::json!({"kind": "ramp", "angle_rad": 4f64.to_radians()})
    );
}

#[test]
fn pocket_ramp_refuses_a_pass_too_short_to_ramp_on() {
    let cancel = AtomicBool::new(false);
    // A 5.5 mm square: the 4 mm cutter's passes are 1.5 mm long, shorter
    // than the 2 mm minimum zigzag leg (the cutter radius).
    let prepare = |entry: RampEntry| {
        let m = pocket_manifest([[10., 9.], [15.5, 14.5]], Some(PocketEntry::Ramp(entry)));
        let spec = m.stock.height().unwrap().clone();
        OperationPlan::prepare(
            m.operations[0].clone(),
            &m.tools[0],
            &spec,
            m.initial_position_mm,
            1,
            &cancel,
        )
        // The refusal comes at preparation, before any motion.
        .map(|_| ())
    };
    // No minimum declared: camlib assumes none, and says what it is.
    let error = prepare(ramp(4.)).unwrap_err();
    for part in [
        "pocket slot",
        "no ramp entry",
        "Z8",
        "has to reverse",
        "min_length_mm is not set",
        "shortest span this tool may reverse a ramp on",
        "HelicalEntry",
    ] {
        assert!(error.contains(part), "{part:?} missing from: {error}");
    }
    // Declared, and the pass is shorter.
    let error = prepare(zigzag(4., 2.)).unwrap_err();
    for part in [
        "pocket slot",
        "shorter than min_length_mm 2.0000",
        "a smaller angle does not help",
    ] {
        assert!(error.contains(part), "{part:?} missing from: {error}");
    }
    // Declared short enough: the ramp is made.
    let m = pocket_manifest(
        [[10., 9.], [15.5, 14.5]],
        Some(PocketEntry::Ramp(zigzag(4., 1.))),
    );
    let (seen, _) = run(&m, 4., 150.);
    assert!(seen.ramps > 2);
    // Without stock feedback a ramp is not planned at all.
    let m = pocket_manifest(SLOT, Some(PocketEntry::Ramp(ramp(10.))));
    let Operation::Pocket(p) = m.operations[0].clone() else {
        panic!("fixture");
    };
    let mut plan = PocketPlan::prepare(
        p,
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    assert!(
        plan.next_motion(&cancel)
            .unwrap_err()
            .contains("committed stock")
    );
    // Invalid angles are refused before planning.
    for angle in [0., -3., 90., f64::NAN] {
        let error = prepare(ramp(angle)).unwrap_err();
        assert!(error.contains("invalid ramp entry"), "{error}");
    }
    let error = prepare(zigzag(4., 0.)).unwrap_err();
    assert!(
        error.contains("invalid ramp entry: min_length_mm"),
        "{error}"
    );
}

/// Replace the fixture pocket's region outline (x, y, bulge vertices).
fn outline(m: &mut StreamManifest, vertices: Vec<[f64; 3]>) {
    let Operation::Pocket(p) = &mut m.operations[0] else {
        panic!("fixture");
    };
    p.region.components[0].outer.vertices = vertices;
}

#[test]
fn pocket_ramp_follows_arcs_as_helical_pieces() {
    // The slot with semicircular ends (radius 2.5): the cutter centre stays
    // within 0.5 mm of the axis y = 12, x 5.5..26.5. Its wall pass is two
    // lines joined by half circles, and the ramp into it runs over them.
    let mut m = pocket_manifest(SLOT, Some(PocketEntry::Ramp(zigzag(4., 2.))));
    outline(
        &mut m,
        vec![
            [5.5, 9.5, 0.],
            [26.5, 9.5, 1.],
            [26.5, 14.5, 0.],
            [5.5, 14.5, 1.],
        ],
    );
    m.validate().unwrap();
    let (seen, stock) = run(&m, 4., 150.);
    let axis = |p: [f64; 3]| (p[0] - p[0].clamp(5.5, 26.5)).hypot(p[1] - 12.);
    let mut helical = 0;
    for motion in &seen.motions {
        if motion.kind == MoveKind::Rapid {
            continue;
        }
        let (a, b) = (motion.curve.start(), motion.curve.end());
        // The centre set (within 0.5 mm of the axis) is convex: a line is
        // inside when its ends are, an arc when it is one of the end circles.
        assert!(axis(a) <= 0.5 + 1e-7 && axis(b) <= 0.5 + 1e-7, "{motion:?}");
        assert!(a[2].min(b[2]) >= 6. - 1e-12, "{motion:?}");
        match motion.curve {
            Curve::Line { .. } => {}
            Curve::Arc { center, .. } | Curve::Helix { center, .. } => {
                assert!(
                    (center[0] == 5.5 || center[0] == 26.5) && center[1] == 12.,
                    "{motion:?}"
                );
                let r = (a[0] - center[0]).hypot(a[1] - center[1]);
                assert!(r <= 0.5 + 1e-7, "{motion:?}");
                if matches!(motion.curve, Curve::Helix { .. }) {
                    // `run` has checked its slope, kind and feed.
                    assert!(b[2] < a[2], "{motion:?}");
                    helical += 1;
                }
            }
        }
    }
    assert!(helical >= 2, "{helical} helical ramp pieces");
    // The whole rounded slot is at the floor, in both stocks.
    let mut checked = 0;
    for j in 0..seen.heights.ny {
        for i in 0..seen.heights.nx {
            let q = [(i as f64 + 0.5) * CELL, (j as f64 + 0.5) * CELL];
            let d = axis([q[0], q[1], 0.]);
            if d <= 2.3 {
                assert_eq!(seen.heights.at(q), 6., "floor left high at {q:?}");
                checked += 1;
            }
            if d >= 2.7 {
                assert_eq!(seen.heights.at(q), 10., "cut outside the slot at {q:?}");
            }
        }
    }
    assert!(checked > 5000);
    for (i, h) in stock.cells().iter().enumerate() {
        let q = stock.cell_center(i).unwrap();
        if axis([q[0], q[1], 0.]) <= 2. {
            assert!(h.upper_mm < 6.003, "engine stock left high at {q:?}: {h:?}");
        }
    }
}

/// Cut a plain (plunge entry) pocket first, into a fresh stock and map.
fn precut(region: [[f64; 2]; 2], floor: f64) -> (Stock, Heights) {
    let mut m = pocket_manifest(region, None);
    let Operation::Pocket(p) = &mut m.operations[0] else {
        panic!("fixture");
    };
    p.id = "before".into();
    p.floor_z_mm = floor;
    let cancel = AtomicBool::new(false);
    let spec = m.stock.height().unwrap();
    let mut stock = Stock::new(spec.clone()).unwrap();
    let mut heights = Heights::new(spec.max_mm);
    let mut plan = OperationPlan::prepare_with_stock(
        m.operations[0].clone(),
        &m.tools[0],
        &stock,
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    let tool = m.tools[0].prepare().unwrap();
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        for p in samples(&motion.curve) {
            heights.cut(p);
        }
        let delta = stock
            .preview_sweep(&tool, &motion, 1., SweepOptions::default(), &cancel)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
    }
    (stock, heights)
}

#[test]
fn pocket_ramp_on_uneven_stock_descends_straight_only_through_air() {
    // Half of the slot is already 2 mm deep: the ramps run partly in air,
    // and the straight descents to them still meet no material.
    let (stock, heights) = precut([[3., 9.5], [16., 14.5]], 8.);
    let m = pocket_manifest(SLOT, Some(PocketEntry::Ramp(zigzag(4., 2.))));
    let (seen, stock) = run_on(&m, 4., 150., stock, heights);
    assert!(seen.ramps >= 2);
    check_slot(&seen, &stock, true, 6., Some(10.));
}

#[test]
fn pocket_ramp_below_a_lower_authored_top_starts_at_that_top() {
    // A wider pocket has taken the top 2 mm; the slot starts at Z8 inside
    // it. The first ramp starts at Z8, in air above the earlier floor.
    let (stock, heights) = precut([[2., 8.], [30., 16.]], 8.);
    let mut m = pocket_manifest(SLOT, Some(PocketEntry::Ramp(zigzag(4., 2.))));
    let Operation::Pocket(p) = &mut m.operations[0] else {
        panic!("fixture");
    };
    p.top_z_mm = 8.;
    p.floor_z_mm = 4.;
    let (seen, stock) = run_on(&m, 4., 150., stock, heights);
    assert!(seen.ramps >= 2);
    // No fed move of the slot is above its top, bar the descents to it.
    let sloped_top = seen
        .motions
        .iter()
        .filter(|m| m.curve.end()[2] < m.curve.start()[2] && horizontal(&m.curve) > 0.)
        .map(|m| m.curve.start()[2])
        .fold(f64::NEG_INFINITY, f64::max);
    assert_eq!(sloped_top, 8.);
    check_slot(&seen, &stock, true, 4., None);
}

/// The highest any sloped descent of `motions` starts above the material
/// under its own path, replayed on `heights` (the map as it was before
/// them): a ramp that starts where the material does has none.
fn air_above_ramps(motions: &[Motion], mut heights: Heights) -> f64 {
    let mut air = 0f64;
    for motion in motions {
        let c = &motion.curve;
        if c.end()[2] < c.start()[2] && horizontal(c) > 0. {
            let under = samples(c)
                .iter()
                .map(|p| heights.under([p[0], p[1]]))
                .fold(f64::NEG_INFINITY, f64::max);
            air = air.max(c.start()[2] - under);
        }
        for p in samples(c) {
            heights.cut(p);
        }
    }
    air
}

/// #219: a rest pocket ramps into each leftover piece along its own guide,
/// from the material the stock holds there, not from the pocket top.
#[test]
fn rest_pocket_ramps_into_leftover_pieces_from_the_material_top() {
    // Right of X16 the slot is already 2 mm deep.
    let before = || precut([[16., 9.5], [29., 14.5]], 8.);
    let rest = |entry: RampEntry| {
        let mut m = pocket_manifest(SLOT, Some(PocketEntry::Ramp(entry)));
        let Operation::Pocket(p) = &mut m.operations[0] else {
            panic!("fixture");
        };
        p.rest_tolerance_mm = Some(0.01);
        m
    };
    let m = rest(zigzag(4., 2.));
    m.validate().unwrap();
    let (stock, heights) = before();
    let (seen, stock) = run_on(&m, 4., 150., stock, heights);
    // `run_on` has checked every descent: straight only through air, sloped
    // within 4 degrees at the entry feed. Both levels were ramped into.
    assert!(seen.ramps >= 2, "{} ramps", seen.ramps);
    check_slot(&seen, &stock, true, 6., Some(10.));
    // The first level's ramp is where stock was left: left of the earlier
    // pocket (whose cutter centre came as far as X18), from the pocket top.
    // It does not run on over that pocket's floor, through air.
    let first_level: Vec<_> = seen
        .motions
        .iter()
        .filter(|m| {
            let (a, b) = (m.curve.start(), m.curve.end());
            b[2] < a[2] && horizontal(&m.curve) > 0. && b[2] >= 8.
        })
        .collect();
    assert!(first_level.len() >= 2, "{} legs", first_level.len());
    assert_eq!(first_level[0].curve.start()[2], 10.);
    for motion in first_level {
        let (a, b) = (motion.curve.start(), motion.curve.end());
        // (Rest pieces are a cutter radius long: the last one with stock
        // on it ends at X19.)
        assert!(a[0].max(b[0]) < 19.01, "a ramp through air: {motion:?}");
    }
    // No ramp starts above the material under it: the second level's start
    // 0.01 mm above the first's floor, not at the pocket top 2 mm higher.
    let air = air_above_ramps(&seen.motions, before().1);
    assert!(air <= 0.05, "a ramp starts {air} mm above the material");
    let lowest_start = seen
        .motions
        .iter()
        .filter(|m| m.curve.end()[2] < m.curve.start()[2] && horizontal(&m.curve) > 0.)
        .map(|m| m.curve.start()[2])
        .fold(f64::INFINITY, f64::min);
    // (The legs of the second level's zigzag start lower still.)
    assert!(lowest_start < 8., "{lowest_start}");
    let second_level = seen
        .motions
        .iter()
        .filter(|m| m.curve.end()[2] < 8. && horizontal(&m.curve) > 0.)
        .map(|m| m.curve.start()[2])
        .fold(f64::NEG_INFINITY, f64::max);
    assert!((8. ..8.02).contains(&second_level), "{second_level}");

    // One leg where the leftover's guide is long enough for it.
    let (stock, heights) = before();
    let (seen, stock) = run_on(&rest(ramp(10.)), 10., 150., stock, heights);
    check_slot(&seen, &stock, false, 6., Some(10.));

    // A reversal the tool has not been declared to make is refused when the
    // piece is reached, naming the place and the fix; nothing is plunged.
    let cancel = AtomicBool::new(false);
    let m = rest(ramp(4.));
    let (mut stock, _) = before();
    let mut plan = OperationPlan::prepare_with_stock(
        m.operations[0].clone(),
        &m.tools[0],
        &stock,
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    let tool = m.tools[0].prepare().unwrap();
    let error = loop {
        match plan.next_motion_with_stock(&stock, &cancel) {
            Ok(Some(motion)) => {
                assert!(
                    motion.curve.end()[2] >= 10.,
                    "moved into the stock: {motion:?}"
                );
                let delta = stock
                    .preview_sweep(&tool, &motion, 1., SweepOptions::default(), &cancel)
                    .unwrap();
                stock.apply_delta(&delta).unwrap();
            }
            Ok(None) => panic!("a ramp that has to reverse was accepted"),
            Err(error) => break error,
        }
    };
    for part in ["pocket slot", "no ramp entry", "min_length_mm is not set"] {
        assert!(error.contains(part), "{error}");
    }
}

/// #219, decided: a ramp entry does not lift the pocket's center-cutting
/// requirement. Every ramp turns back along the slope it has just cut, and a
/// cutter that is not center cutting must be lifted a cutter-specific
/// distance first (Sandvik Coromant, "Ramping: two axis linear and
/// circular"), which camlib has no data for.
#[test]
fn pocket_ramp_does_not_admit_a_cutter_that_is_not_center_cutting() {
    let prepare = |entry: Option<PocketEntry>, rest: bool| {
        let mut m = pocket_manifest(SLOT, entry);
        m.tools[0].cutter.center_cutting = false;
        let Operation::Pocket(p) = &mut m.operations[0] else {
            panic!("fixture");
        };
        p.rest_tolerance_mm = rest.then_some(0.01);
        OperationPlan::prepare_with_stock(
            m.operations[0].clone(),
            &m.tools[0],
            &Stock::new(m.stock.height().unwrap().clone()).unwrap(),
            m.initial_position_mm,
            1,
            &AtomicBool::new(false),
        )
        .map(|_| ())
    };
    // One leg (10 degrees fits the slot) or a zigzag, declared or not, in a
    // pocket or a rest pocket: refused before any motion, with the reason.
    for entry in [ramp(10.), ramp(4.), zigzag(4., 2.)] {
        for rest in [false, true] {
            let error = prepare(Some(PocketEntry::Ramp(entry.clone())), rest).unwrap_err();
            for part in [
                "pocket slot: a RampEntry does not lift the need for a center-cutting end mill",
                "turning back along the slope",
                "must first be lifted",
            ] {
                assert!(error.contains(part), "{error}");
            }
        }
    }
    // Without a ramp it is refused as before.
    let helix = PocketEntry::Helix(HelicalEntry::default());
    for entry in [None, Some(helix)] {
        let error = prepare(entry, false).unwrap_err();
        assert!(error.contains("flat center-cutting end mill"), "{error}");
    }
    // The same cutter, center cutting, ramps.
    let m = pocket_manifest(SLOT, Some(PocketEntry::Ramp(ramp(10.))));
    assert!(m.tools[0].cutter.center_cutting);
    let (seen, stock) = run(&m, 10., 150.);
    check_slot(&seen, &stock, false, 6., Some(10.));
}

fn contour(entry: Option<PocketEntry>) -> StreamManifest {
    let mut m: StreamManifest =
        serde_json::from_str(include_str!("../fixtures/operations/contour.json")).unwrap();
    let Operation::Contour(c) = &mut m.operations[0] else {
        panic!("fixture");
    };
    c.entry = entry;
    m
}

#[test]
fn contour_ramp_descends_around_each_loop_then_cuts_it_flat() {
    // An annulus (radii 4 and 8 about (20, 20)) kept; 8 mm deep, stepdown 2.
    // One descent per loop: the outer loop (62.9 mm) is bound by the
    // stepdown (4 laps of 2 mm), the hole's (12.5 mm) by the 3 degrees
    // (8 mm needs 152.6 mm: 13 laps).
    for (lead, feed) in [(0., None), (1.5, Some(80.))] {
        let mut entry = ramp(3.);
        entry.feed_mm_min = feed;
        let mut m = contour(Some(PocketEntry::Ramp(entry)));
        let Operation::Contour(c) = &mut m.operations[0] else {
            panic!("fixture");
        };
        c.lead_in_mm = lead;
        c.lead_out_mm = lead;
        // Tangent leads fit outside the part only: the hole is too small.
        let hole = lead == 0.;
        if !hole {
            c.boundaries = camlib::contour::Boundaries::Outer;
        }
        m.validate().unwrap();
        let (seen, stock) = run(&m, 3., feed.unwrap_or(150.));
        let mut laps = [0., 0.];
        for motion in &seen.motions {
            let (a, b) = (motion.curve.start(), motion.curve.end());
            if motion.kind == MoveKind::Rapid {
                continue;
            }
            match motion.curve {
                Curve::Arc {
                    center, sweep_rad, ..
                }
                | Curve::Helix {
                    center, sweep_rad, ..
                } => {
                    assert!((center[0] - 20.).abs() < 1e-8 && (center[1] - 20.).abs() < 1e-8);
                    // The whole circle clears the kept part by the cutter radius.
                    let r = (a[0] - 20.).hypot(a[1] - 20.);
                    assert!(r - RADIUS > 8. || r + RADIUS < 4., "{motion:?}");
                    if b[2] < a[2] {
                        laps[(r < 8.) as usize] += sweep_rad.abs() / std::f64::consts::TAU;
                        // No lap goes deeper than the stepdown.
                        let per_lap = (a[2] - b[2]) / sweep_rad.abs() * std::f64::consts::TAU;
                        assert!(per_lap <= 2. + 1e-9, "{motion:?}");
                    }
                }
                Curve::Line { .. } => {
                    // Plunges, leads and retracts: beside the part.
                    for p in [a, b] {
                        let r = (p[0] - 20.).hypot(p[1] - 20.);
                        assert!(r - RADIUS > 8. || r + RADIUS < 4., "{motion:?}");
                    }
                }
            }
            assert!(a[2].min(b[2]) >= 2., "{motion:?}");
        }
        let expected = if hole { 13. } else { 0. };
        assert!(
            (laps[0] - 4.).abs() < 1e-9 && (laps[1] - expected).abs() < 1e-9,
            "{laps:?}"
        );
        // One entry per loop: the tool comes straight down twice, in air.
        let descents = seen
            .motions
            .iter()
            .filter(|m| m.kind == MoveKind::Plunge && horizontal(&m.curve) == 0.)
            .count();
        assert_eq!(descents, 1 + hole as usize);
        // The bands are at the floor all round (no ramp left), the part whole.
        for j in 0..seen.heights.ny {
            for i in 0..seen.heights.nx {
                let q = [(i as f64 + 0.5) * CELL, (j as f64 + 0.5) * CELL];
                let r = (q[0] - 20.).hypot(q[1] - 20.);
                if (hole && r < 3.7) || (8.3..11.7).contains(&r) {
                    assert_eq!(seen.heights.at(q), 2., "band left high at {q:?}");
                }
                if (4.2..7.8).contains(&r) || (!hole && r < 7.8) {
                    assert_eq!(seen.heights.at(q), 10., "part cut at {q:?}");
                }
            }
        }
        for (i, h) in stock.cells().iter().enumerate() {
            let q = stock.cell_center(i).unwrap();
            let r = (q[0] - 20.).hypot(q[1] - 20.);
            if (4.5..7.5).contains(&r) {
                assert_eq!(h.lower_mm, 10., "protected {q:?}");
            }
            if (hole && r < 3.5) || (8.5..11.5).contains(&r) {
                assert!(h.upper_mm < 2.003, "band {q:?}: {h:?}");
            }
        }
    }
}

#[test]
fn contour_ramp_refusals_and_default_serialisation() {
    let cancel = AtomicBool::new(false);
    let prepare = |m: &StreamManifest| {
        OperationPlan::prepare(
            m.operations[0].clone(),
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &cancel,
        )
        .map(|_| ())
    };
    let op = |m: &mut StreamManifest, f: &dyn Fn(&mut ContourOperation)| {
        let Operation::Contour(c) = &mut m.operations[0] else {
            panic!("fixture");
        };
        f(c);
    };
    // Tabs come back down at 45 degrees (or straight): not with a ramp.
    let mut m = contour(Some(PocketEntry::Ramp(ramp(3.))));
    op(&mut m, &|c| {
        c.tabs = Some(camlib::contour::Tabs {
            width_mm: 3.,
            height_mm: 1.,
            count: Some(2),
            spacing_mm: None,
            positions_mm: None,
            position_tolerance_mm: None,
            shape: camlib::contour::TabShape::Triangle,
        })
    });
    let error = prepare(&m).unwrap_err();
    assert!(
        error.contains("ramp entry cannot be combined with holding tabs"),
        "{error}"
    );
    // A helix has nowhere to go on a profile.
    let m = contour(Some(PocketEntry::Helix(HelicalEntry::default())));
    let error = prepare(&m).unwrap_err();
    assert!(error.contains("must be a RampEntry"), "{error}");
    // Limits that block part of a loop are lifted over, and the tool comes
    // straight back down: refused at preparation with a ramp, planned without.
    let blocked = |entry: Option<PocketEntry>| {
        let mut m = contour(entry);
        op(&mut m, &|c| {
            let mut area = c.region.clone();
            area.components[0].holes.clear();
            area.components[0].outer.vertices = vec![
                [27., 18., 0.],
                [34., 18., 0.],
                [34., 22., 0.],
                [27., 22., 0.],
            ];
            c.boundaries = camlib::contour::Boundaries::Outer;
            c.limits = Some(camlib::limits::Limits {
                contain: None,
                avoid: vec![camlib::limits::AvoidArea {
                    region: area,
                    top_z_mm: 11.,
                }],
            });
        });
        prepare(&m)
    };
    blocked(None).unwrap();
    let error = blocked(Some(PocketEntry::Ramp(ramp(3.)))).unwrap_err();
    assert!(
        error.contains(
            "contour profile: a ramp entry cannot be combined with limits that block part of loop 0"
        ),
        "{error}"
    );
    let m = contour(Some(PocketEntry::Ramp(ramp(95.))));
    assert!(prepare(&m).unwrap_err().contains("invalid ramp entry"));
    // More laps than the bound: refused, naming the fix.
    let m = contour(Some(PocketEntry::Ramp(ramp(0.005))));
    let error = prepare(&m).unwrap_err();
    assert!(
        error.contains("laps") && error.contains("steeper angle"),
        "{error}"
    );
    // Absent unless set.
    let m = contour(None);
    assert!(
        serde_json::to_value(&m.operations[0])
            .unwrap()
            .get("entry")
            .is_none()
    );
}

#[test]
fn roughing_and_adaptive_refuse_a_ramp_entry_by_name() {
    let cancel = AtomicBool::new(false);
    let mut m: StreamManifest =
        serde_json::from_str(include_str!("../fixtures/operations/helical-adaptive.json")).unwrap();
    let Operation::AdaptivePocket(a) = &mut m.operations[0] else {
        panic!("fixture");
    };
    a.pocket.entry = Some(PocketEntry::Ramp(ramp(3.)));
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
        error.contains("RampEntry is not supported") && error.contains("HelicalEntry"),
        "{error}"
    );
    let mut m: StreamManifest =
        serde_json::from_str(include_str!("../fixtures/operations/roughing.json")).unwrap();
    let Operation::Roughing(r) = &mut m.operations[0] else {
        panic!("fixture");
    };
    r.entry = PocketEntry::Ramp(ramp(3.));
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
        error.contains("roughing rough: RampEntry is not supported")
            && error.contains("StockEdgeEntry"),
        "{error}"
    );
}
