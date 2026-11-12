//! Material an operation was asked to remove and cannot get into (#250) is
//! never left without a word.
//!
//! A pocket operation (`Pocket`, a rest pocket, `AdaptivePocket` on every
//! path strategy) is asked to clear its region: an area of it the cutter
//! fits in and no entry reaches refuses it, by name and before any motion,
//! and so does a region the cutter fits nowhere in. A `Roughing` layer
//! legitimately has such areas: each is listed in the layer's report and
//! counted in its residual, and the operation is refused only when it would
//! make no motion at all.
//!
//! The rule checked for every clearing operation and strategy, from the
//! outcome alone: a slot the cutter fits, full of stock, is either refused
//! or cut to its floor. What is material and what was cut is read from the
//! stock the emitted moves were swept through (the stock kernel, not a
//! planner's region) and from the slot's own corner coordinates.
use camlib::{
    adaptive::*, motion::*, operation::OperationPlan, pocket::*, stock::*, stream::StreamManifest,
};
use std::sync::atomic::AtomicBool;

const TOP: f64 = 10.;
const FLOOR: f64 = 8.;

fn fixture() -> StreamManifest {
    serde_json::from_str(include_str!("../fixtures/operations/adaptive-pocket.json")).unwrap()
}
/// The fixture's adaptive pocket, planned by the engagement front (the
/// Python API's default; a manifest without `path_strategy` is disk_front).
fn adaptive(m: &StreamManifest) -> AdaptivePocketOperation {
    let Operation::AdaptivePocket(mut op) = m.operations[0].clone() else {
        panic!("fixture")
    };
    op.path_strategy = AdaptivePathStrategy::EngagementFront;
    op
}
type Rect = [[f64; 2]; 2];
fn rectangle(r: Rect) -> Vec<[f64; 3]> {
    vec![
        [r[1][0], r[0][1], 0.],
        [r[0][0], r[0][1], 0.],
        [r[0][0], r[1][1], 0.],
        [r[1][0], r[1][1], 0.],
    ]
}
/// A slot 20 mm long and `width` wide for the fixture's 4 mm cutter.
fn slot_rect(width: f64) -> Rect {
    [[3., 12. - width / 2.], [23., 12. + width / 2.]]
}
fn set_region(pocket: &mut PocketOperation, vertices: Vec<[f64; 3]>) {
    let component = &mut pocket.region.components[0];
    component.holes.clear();
    component.outer.vertices = vertices;
}
fn ramp(angle_deg: f64) -> Option<PocketEntry> {
    Some(PocketEntry::Ramp(RampEntry {
        angle_rad: angle_deg.to_radians(),
        min_length_mm: Some(2.),
        feed_mm_min: None,
    }))
}
fn helix() -> Option<PocketEntry> {
    Some(PocketEntry::Helix(HelicalEntry::default()))
}

struct Run {
    motions: Vec<Motion>,
    stock: Stock,
    report: serde_json::Value,
    error: Option<String>,
}
/// Plan `op` on `stock`, committing every motion to it, to the end or to
/// the first refusal.
fn run_on(m: &StreamManifest, op: Operation, mut stock: Stock) -> Run {
    let cancel = AtomicBool::new(false);
    let tool = m.tools[0].prepare().unwrap();
    let mut motions = vec![];
    let mut plan = match OperationPlan::prepare_with_stock(
        op,
        &m.tools[0],
        &stock,
        m.initial_position_mm,
        1,
        &cancel,
    ) {
        Ok(plan) => plan,
        Err(error) => {
            return Run {
                motions,
                stock,
                report: serde_json::Value::Null,
                error: Some(error),
            };
        }
    };
    let mut position = m.initial_position_mm;
    let error = loop {
        match plan.next_motion_with_stock(&stock, &cancel) {
            Ok(Some(motion)) => {
                assert!(motions.len() < 100_000);
                assert_eq!(motion.curve.start(), position, "discontinuous {motion:?}");
                position = motion.curve.end();
                let delta = stock
                    .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
                    .unwrap();
                stock.apply_delta(&delta).unwrap();
                motions.push(motion);
            }
            Ok(None) => break plan.finish_report(&cancel).err(),
            Err(error) => break Some(error),
        }
    };
    Run {
        motions,
        report: serde_json::to_value(plan.report()).unwrap(),
        stock,
        error,
    }
}
fn run(m: &StreamManifest, op: Operation) -> Run {
    let stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let started = std::time::Instant::now();
    let seen = run_on(m, op, stock);
    eprintln!(
        "run: {} motions in {:.1} s",
        seen.motions.len(),
        started.elapsed().as_secs_f64()
    );
    seen
}
/// Stock cells whose centres lie in `r` shrunk by `margin`.
fn cells_in(stock: &Stock, r: Rect, margin: f64) -> Vec<HeightBounds> {
    let cells: Vec<_> = stock
        .cells()
        .iter()
        .enumerate()
        .filter(|(i, _)| {
            let p = stock.cell_center(*i).unwrap();
            p[0] > r[0][0] + margin
                && p[0] < r[1][0] - margin
                && p[1] > r[0][1] + margin
                && p[1] < r[1][1] - margin
        })
        .map(|(_, cell)| *cell)
        .collect();
    assert!(cells.len() > 20, "{} cells in {r:?}", cells.len());
    cells
}
/// Nothing in `r` was touched: every cell still certainly stands at the top.
fn untouched(stock: &Stock, r: Rect) -> bool {
    cells_in(stock, r, 0.)
        .iter()
        .all(|cell| cell.lower_mm == TOP)
}
/// Everything the cutter (radius 2) can reach in the slot `r` is at `floor`:
/// its whole width between the end radii, a stock cell off the walls.
fn cut_to(stock: &Stock, r: Rect, floor: f64) -> bool {
    let between = [[r[0][0] + 2., r[0][1]], [r[1][0] - 2., r[1][1]]];
    cells_in(stock, between, 0.3)
        .iter()
        .all(|cell| cell.upper_mm <= floor + 1e-4)
}
/// The point a refusal names: "... at (x, y) ...".
fn named_point(error: &str) -> [f64; 2] {
    let rest = error.split_once(" at (").expect(error).1;
    let (x, rest) = rest.split_once(", ").expect(error);
    let y = rest.split_once(')').expect(error).0;
    [x.parse().expect(error), y.parse().expect(error)]
}
fn inside(p: [f64; 2], r: Rect) -> bool {
    p[0] > r[0][0] && p[0] < r[1][0] && p[1] > r[0][1] && p[1] < r[1][1]
}

/// The issue's case: a 20 x 5 mm slot, a 4 mm cutter, the default entry.
#[test]
fn adaptive_pocket_refuses_a_slot_no_helix_fits_and_plans_it_with_a_ramp() {
    let m = fixture();
    let slot = slot_rect(5.);
    // No entry given (a helix by default), and a helix asked for.
    for entry in [None, helix()] {
        let mut op = adaptive(&m);
        set_region(&mut op.pocket, rectangle(slot));
        op.pocket.entry = entry.clone();
        let seen = run(&m, Operation::AdaptivePocket(op));
        let error = seen.error.expect("a slot no helix fits must be refused");
        // Refused before any motion, with the stock as it was.
        assert!(seen.motions.is_empty(), "{:?}", seen.motions);
        assert!(untouched(&seen.stock, slot));
        // It names the operation, the place, the level, the entry tried
        // and what to do.
        for part in [
            "no entry into the enclosed material of adaptive at (",
            "for the level at Z9",
            "adaptive pocket level 0 at Z9",
            "the cutter of radius 2 mm fits there",
            "no helix fits",
            "the entry's radii are [1.0, 0.5, 0.25, 0.125, 0.1] mm",
            "entry=RampEntry(angle_deg)",
            "a smaller cutter",
            "an earlier operation that opens a start there",
        ] {
            assert!(error.contains(part), "{entry:?}: {part:?} not in: {error}");
        }
        // The place is in the slot, on a tool centre the cutter can take.
        let p = named_point(&error);
        assert!(inside(p, [[5., 11.5], [21., 12.5]]), "{p:?}: {error}");
    }
    // With a ramp that fits (1 mm a level at 4 degrees needs 14.3 mm of the
    // slot's 16) it plans, and the slot is cut to its floor.
    let mut op = adaptive(&m);
    set_region(&mut op.pocket, rectangle(slot));
    op.pocket.entry = ramp(4.);
    let seen = run(&m, Operation::AdaptivePocket(op));
    assert_eq!(seen.error, None);
    assert_eq!(seen.report["ramp_entries"], 2);
    assert!(cut_to(&seen.stock, slot, FLOOR));
    // A slot wide enough for the helix plans with the default entry: the
    // refusal is about the entry, not about slots.
    let wide = slot_rect(7.);
    let mut op = adaptive(&m);
    set_region(&mut op.pocket, rectangle(wide));
    let seen = run(&m, Operation::AdaptivePocket(op));
    assert_eq!(seen.error, None);
    assert!(seen.report["helical_entries"].as_u64().unwrap() >= 1);
    assert!(cut_to(&seen.stock, wide, FLOOR));
}

/// Not an error: a level with nothing left to cut, and a slot an earlier
/// operation has opened a start in.
#[test]
fn no_material_and_an_opened_start_are_not_refused() {
    let m = fixture();
    let slot = slot_rect(5.);
    let whole = |entry: Option<PocketEntry>| {
        let mut op = adaptive(&m);
        set_region(&mut op.pocket, rectangle(slot));
        op.pocket.entry = entry;
        Operation::AdaptivePocket(op)
    };
    // The slot already cut to the floor (by the ramped pocket): the same
    // pocket with the default entry has nothing to enter. It bores nothing
    // and takes nothing more off the floor.
    let cleared = run(&m, whole(ramp(4.)));
    assert_eq!(cleared.error, None);
    let again = run_on(&m, whole(None), cleared.stock.clone());
    assert_eq!(again.error, None, "nothing left to cut is not an error");
    assert_eq!(again.report["helical_entries"], 0);
    assert!(
        again
            .motions
            .iter()
            .all(|motion| !matches!(motion.curve, Curve::Helix { .. }))
    );
    assert!(cut_to(&again.stock, slot, FLOOR));
    // An earlier operation opens a start: a plain pocket (it plunges) takes
    // the first 8 mm of the slot to the floor. The adaptive pocket then
    // enters there, through stock proven cut, and clears the rest: no helix,
    // no ramp, and no straight descent into standing stock.
    let start: Rect = [[3., 9.5], [11., 14.5]];
    let mut opener = adaptive(&m).pocket;
    opener.id = "opener".into();
    set_region(&mut opener, rectangle(start));
    let opened = run(&m, Operation::Pocket(opener));
    assert_eq!(opened.error, None);
    assert!(untouched(&opened.stock, [[14., 9.5], [23., 14.5]]));
    let mut stock = opened.stock.clone();
    let cancel = AtomicBool::new(false);
    let tool = m.tools[0].prepare().unwrap();
    let mut plan = OperationPlan::prepare_with_stock(
        whole(None),
        &m.tools[0],
        &stock,
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    let mut cuts = 0;
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        let (a, b) = (motion.curve.start(), motion.curve.end());
        assert!(!matches!(motion.curve, Curve::Helix { .. }), "{motion:?}");
        if b[2] < a[2] {
            // Every descent is straight down, onto a floor already cut:
            // nothing certain stands above its end under the tool centre.
            assert_eq!([a[0], a[1]], [b[0], b[1]], "{motion:?}");
            let under = stock
                .cells()
                .iter()
                .enumerate()
                .filter(|(i, _)| {
                    let p = stock.cell_center(*i).unwrap();
                    (p[0] - b[0]).hypot(p[1] - b[1]) < 1.
                })
                .map(|(_, cell)| cell.lower_mm)
                .fold(f64::NEG_INFINITY, f64::max);
            assert!(under <= b[2] + 1e-4, "descent into stock: {motion:?}");
        }
        cuts += usize::from(motion.kind == MoveKind::Cut);
        let delta = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
    }
    plan.finish_report(&cancel).unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    assert!(cuts > 0);
    assert_eq!(report["helical_entries"], 0);
    assert!(report["stock_edge_entries"].as_u64().unwrap() >= 1);
    assert!(cut_to(&stock, slot, FLOOR));
}

/// A region with a pocket the helix fits and, apart from it, a slot it does
/// not: the slot is an area of the region no entry reaches, and the pocket
/// is refused for it before any motion (as `Pocket` refuses an area with
/// no entry). A channel that opens into the pocket is no such area: it is
/// entered from the stock the pocket cleared. A channel too tight for the
/// planner is left, and counted in the level's residual with its reason.
#[test]
fn an_area_no_entry_reaches_is_refused_and_a_channel_off_a_pocket_is_cut_or_counted() {
    let m = fixture();
    // The fixture's L, and a 10 x 5 mm slot in the corner the L leaves.
    let slot: Rect = [[13., 15.], [23., 20.]];
    let both = |entry: Option<PocketEntry>| {
        let mut op = adaptive(&m);
        let mut second = op.pocket.region.components[0].clone();
        second.outer.vertices = rectangle(slot);
        op.pocket.region.components.push(second);
        op.pocket.entry = entry;
        Operation::AdaptivePocket(op)
    };
    let seen = run(&m, both(None));
    let error = seen.error.expect("the slot must be refused");
    assert!(
        error.contains("no entry into the enclosed material of adaptive"),
        "{error}"
    );
    assert!(error.contains("adaptive pocket level 0 at Z9"), "{error}");
    let p = named_point(&error);
    assert!(inside(p, slot), "{p:?} is not in the slot: {error}");
    // Before any motion: nothing of the L was cut either.
    assert!(seen.motions.is_empty());
    assert!(untouched(&seen.stock, slot));
    // With a ramp the whole region is cut.
    let seen = run(&m, both(ramp(8.)));
    assert_eq!(seen.error, None);
    assert!(cut_to(&seen.stock, slot, FLOOR));

    // The L with a channel 9 mm long off its lower arm.
    let with_channel = |width: f64| {
        let (x0, x1) = (17.5 - width / 2., 17.5 + width / 2.);
        let mut op = adaptive(&m);
        set_region(
            &mut op.pocket,
            vec![
                [23., 3., 0.],
                [3., 3., 0.],
                [3., 23., 0.],
                [11., 23., 0.],
                [11., 11., 0.],
                [x0, 11., 0.],
                [x0, 20., 0.],
                [x1, 20., 0.],
                [x1, 11., 0.],
                [23., 11., 0.],
            ],
        );
        (run(&m, Operation::AdaptivePocket(op)), [[x0, 11.], [x1, 20.]])
    };
    // 5 mm wide: cut to the floor, up to the radius its closed end keeps.
    let (seen, channel) = with_channel(5.);
    assert_eq!(seen.error, None);
    let reached = cells_in(&seen.stock, [channel[0], [channel[1][0], 18.]], 0.3);
    assert!(
        reached.iter().all(|cell| cell.upper_mm <= FLOOR + 1e-4),
        "the channel is not cut to the floor"
    );
    // 4.2 mm wide: the cutter fits (0.1 mm a side) but the engagement front
    // has no steering position in it. It opens into the L, where the
    // planner works, so it is not an area without an entry. Whatever is
    // left of it must be in each level's residual: the residual is counted
    // from the region and the stock, not from where the planner can stand.
    // Checked against the stock itself (the kernel's cells, not the
    // planner's grid).
    let alone = run(&m, Operation::AdaptivePocket(adaptive(&m)));
    assert_eq!(alone.error, None);
    let (seen, channel) = with_channel(4.2);
    assert_eq!(seen.error, None);
    // Stock cells of the channel beyond the cutter's reach from the arm
    // that still certainly stand at the top, as an area.
    let deep: Rect = [[channel[0][0], 13.5], [channel[1][0], 20.]];
    let standing = cells_in(&seen.stock, deep, 0.)
        .iter()
        .filter(|cell| cell.lower_mm == TOP)
        .count() as f64
        * 0.4
        * 0.4;
    eprintln!("tight channel: {standing} mm2 of stock cells stand");
    let layers = seen.report["layers"].as_array().unwrap();
    for (level, without) in layers
        .iter()
        .zip(alone.report["layers"].as_array().unwrap())
    {
        let part = |level: &serde_json::Value, name: &str| {
            assert_eq!(level["clearing"]["geometric_residual_status"], "estimated");
            level["clearing"][name].as_f64().unwrap()
        };
        for level in [level, without] {
            let total = part(level, "geometric_residual_area_mm2");
            let sum = part(level, "residual_reachable_area_mm2")
                + part(level, "residual_unresolved_area_mm2")
                + part(level, "residual_unreachable_area_mm2");
            assert!((total - sum).abs() < 1e-9, "{level}");
        }
        // Nothing stands in the channel that the level does not count: its
        // residual exceeds the L's own by at least the standing cells
        // (less the cells the walls cross, a 0.4 mm band each side).
        let more = part(level, "geometric_residual_area_mm2")
            - part(without, "geometric_residual_area_mm2");
        assert!(
            more >= standing - 2. * 0.4 * 6.5 - 1.,
            "{more} mm2 more residual, {standing} mm2 standing: {level}"
        );
    }
    // It is cut: the level's wall pass follows the walls' exact offset,
    // into the channel. Under 1 mm2 of its cells keep certain stock.
    assert!(standing < 1., "{standing}");
}

/// Every pocket operation and strategy on slots the cutter fits: refused,
/// or cut. Never neither.
#[test]
fn no_pocket_strategy_plans_nothing_in_a_slot_the_cutter_fits() {
    let m = fixture();
    let mut outcomes = vec![];
    let mut check = |name: String, slot: Rect, seen: Run| {
        match &seen.error {
            Some(error) => {
                assert!(untouched(&seen.stock, slot), "{name}: {error}");
                assert!(seen.motions.is_empty(), "{name}: {error}");
                // It says what to use instead.
                assert!(
                    ["RampEntry", "entry strategy", "helical entry"]
                        .iter()
                        .any(|part| error.contains(part)),
                    "{name}: {error}"
                );
            }
            None => {
                // A stock cell and a half off the walls: the tightest slot
                // leaves the cutter 0.15 mm of play.
                let between = [[slot[0][0] + 2., slot[0][1]], [slot[1][0] - 2., slot[1][1]]];
                assert!(
                    cells_in(&seen.stock, between, 0.6)
                        .iter()
                        .all(|cell| cell.upper_mm <= FLOOR + 1e-4),
                    "{name} planned without an error and left the slot uncut"
                );
            }
        }
        eprintln!(
            "{name}: {}",
            seen.error.as_deref().unwrap_or("cut to the floor")
        );
        outcomes.push((name, seen.error));
    };
    // 5 mm: no helix fits. 4.5 mm: the engagement front has a lane two grid
    // cells wide. 4.2 mm: the cutter fits, with less play than the
    // engagement front plans in.
    for width in [5., 4.5, 4.2] {
        let slot = slot_rect(width);
        for strategy in [
            AdaptivePathStrategy::EngagementFront,
            AdaptivePathStrategy::DiskFront,
            AdaptivePathStrategy::StockFront,
        ] {
            for entry in [None, helix(), ramp(4.)] {
                let front = strategy == AdaptivePathStrategy::EngagementFront;
                if !front && entry.as_ref().is_some_and(|e| matches!(e, PocketEntry::Ramp(_))) {
                    // Refused by name before planning (#219).
                    continue;
                }
                let mut op = adaptive(&m);
                set_region(&mut op.pocket, rectangle(slot));
                op.path_strategy = strategy;
                op.pocket.entry = entry.clone();
                check(
                    format!("adaptive {strategy:?} {width} mm {entry:?}"),
                    slot,
                    run(&m, Operation::AdaptivePocket(op)),
                );
            }
        }
        for rest in [None, Some(0.01)] {
            for entry in [None, helix(), ramp(4.)] {
                let mut op = adaptive(&m).pocket;
                set_region(&mut op, rectangle(slot));
                op.entry = entry.clone();
                op.rest_tolerance_mm = rest;
                check(
                    format!("pocket rest={rest:?} {width} mm {entry:?}"),
                    slot,
                    run(&m, Operation::Pocket(op)),
                );
            }
        }
    }
    let outcome = |name: &str| {
        outcomes
            .iter()
            .find(|(n, _)| n == name)
            .unwrap_or_else(|| panic!("{name}"))
            .1
            .clone()
    };
    // The engagement front in the 5 mm slot: refused with a helix, cut with
    // a ramp.
    for entry in [None, helix()] {
        let error = outcome(&format!("adaptive EngagementFront 5 mm {entry:?}")).unwrap();
        assert!(error.contains("no entry into the enclosed material"), "{error}");
    }
    assert_eq!(
        outcome(&format!("adaptive EngagementFront 5 mm {:?}", ramp(4.))),
        None
    );
    // In the 4.2 mm slot it has no position at all (0.1 mm of play a side,
    // and it needs 0.188): it says so and names a Pocket, which does cut
    // that slot.
    let error = outcome(&format!("adaptive EngagementFront 4.2 mm {:?}", ramp(4.))).unwrap();
    for part in [
        "adaptive has material at (",
        "less play to its walls than this planner works in",
        "Use a Pocket there",
        "adaptive pocket level 0 at Z9",
    ] {
        assert!(error.contains(part), "{part:?} not in: {error}");
    }
    assert!(error.contains("it needs 0.188 mm on each side"), "{error}");
    assert!(inside(named_point(&error), slot_rect(4.2)), "{error}");
    assert_eq!(
        outcome(&format!("pocket rest=None 4.2 mm {:?}", None::<PocketEntry>)),
        None
    );
    // The orbit strategies sweep a disk of radius 3 (cutter 2 + orbit 1):
    // none fits a 5 mm slot, and that is refused with the sweep's size.
    for strategy in ["DiskFront", "StockFront"] {
        let error = outcome(&format!("adaptive {strategy} 5 mm None")).unwrap();
        for part in [
            "adaptive pocket adaptive: no orbit fits the area at (",
            "sweep a disk of radius 3 mm (cutter radius 2 + orbit radius 1)",
            "though the cutter fits it",
            "path_strategy engagement_front",
            "a smaller orbit_radius_mm",
        ] {
            assert!(error.contains(part), "{part:?} not in: {error}");
        }
        assert!(inside(named_point(&error), slot_rect(5.)), "{error}");
    }
    // A helix radius that does not fit is refused the same way by the
    // entry fit, for a Pocket and for the engagement front: what, where,
    // what was tried, the options.
    for front in [false, true] {
        let wide = HelicalEntry {
            radius_mm: Some(0.75),
            ..HelicalEntry::default()
        };
        let mut op = adaptive(&m);
        set_region(&mut op.pocket, rectangle(slot_rect(5.)));
        op.pocket.entry = Some(PocketEntry::Helix(wide));
        let seen = run(
            &m,
            if front {
                Operation::AdaptivePocket(op)
            } else {
                Operation::Pocket(op.pocket)
            },
        );
        let error = seen.error.unwrap();
        assert!(seen.motions.is_empty());
        for part in [
            "no source-admitted entry/route into area 0 of adaptive at (",
            "tool centres within X",
            "the cutter of radius 2 mm fits there",
            "no helix of the entry's radii [0.75] mm fits",
            "a circle of radius 2 + h mm clear of the walls",
            "entry=RampEntry(angle_deg)",
            "a smaller cutter",
        ] {
            assert!(error.contains(part), "{part:?} not in: {error}");
        }
        assert!(inside(named_point(&error), slot_rect(5.)), "{error}");
    }
    // A smaller orbit, as that message says, does fit and cut the slot.
    let slot = slot_rect(5.);
    let mut op = adaptive(&m);
    set_region(&mut op.pocket, rectangle(slot));
    op.path_strategy = AdaptivePathStrategy::DiskFront;
    op.orbit_radius_mm = Some(0.25);
    let seen = run(&m, Operation::AdaptivePocket(op));
    assert_eq!(seen.error, None);
    assert!(cut_to(&seen.stock, slot, FLOOR));
    // A region the cutter fits nowhere in (3 mm for a 4 mm cutter): the
    // operation can do nothing it was asked to, and every pocket operation
    // says so before any motion.
    let narrow = slot_rect(3.);
    let mut ops = vec![];
    for strategy in [
        AdaptivePathStrategy::EngagementFront,
        AdaptivePathStrategy::DiskFront,
        AdaptivePathStrategy::StockFront,
    ] {
        let mut op = adaptive(&m);
        set_region(&mut op.pocket, rectangle(narrow));
        op.path_strategy = strategy;
        ops.push(("adaptive pocket adaptive", Operation::AdaptivePocket(op)));
    }
    for rest in [None, Some(0.01)] {
        let mut op = adaptive(&m).pocket;
        set_region(&mut op, rectangle(narrow));
        op.rest_tolerance_mm = rest;
        ops.push(("pocket adaptive", Operation::Pocket(op)));
    }
    for (name, op) in ops {
        let seen = run(&m, op);
        let error = seen.error.expect(name);
        assert!(seen.motions.is_empty());
        for part in [
            &format!("{name}: the cutter fits nowhere in the region"),
            "a tool centre needs 2 mm to every wall (cutter radius 2 mm + radial allowance 0 mm)",
            "no part of the region is 4 mm wide",
            "Use a smaller cutter",
        ] {
            assert!(error.contains(part), "{part:?} not in: {error}");
        }
    }
}

fn roughing(clearing: serde_json::Value, entry: serde_json::Value, width: f64) -> StreamManifest {
    roughing_from(0., clearing, entry, width)
}
/// The plate starts at `x_min`: from 0 it covers the whole stock, and
/// anything more leaves a strip of stock beside it for the layers to cut.
fn roughing_from(
    x_min: f64,
    clearing: serde_json::Value,
    entry: serde_json::Value,
    width: f64,
) -> StreamManifest {
    let mut v: serde_json::Value =
        serde_json::from_str(include_str!("../fixtures/operations/roughing-adaptive.json"))
            .unwrap();
    // A plate as large as the 32 x 28 x 10 mm stock, flat on top, with one
    // enclosed slot 12 mm long down to Z6.
    let (y0, y1) = (14. - width / 2., 14. + width / 2.);
    let mut triangles = vec![];
    for (r, z) in [
        ([[x_min, 0.], [32., y0]], TOP),
        ([[x_min, y1], [32., 28.]], TOP),
        ([[x_min, y0], [10., y1]], TOP),
        ([[22., y0], [32., y1]], TOP),
        ([[10., y0], [22., y1]], 6.),
    ] {
        let [[x0, y0], [x1, y1]] = r;
        triangles.push([[x0, y0, z], [x1, y0, z], [x1, y1, z]]);
        triangles.push([[x0, y0, z], [x1, y1, z], [x0, y1, z]]);
    }
    let op = &mut v["operations"][0];
    op["target"] = serde_json::json!({
        "version": 1, "units": "mm", "triangles": triangles,
        "surface_error_mm": 0., "source": null,
    });
    op["floor_z_mm"] = 6.0.into();
    op["contour_tolerance_mm"] = 0.0.into();
    op["clearing"] = clearing;
    op["entry"] = entry;
    serde_json::from_value(v).unwrap()
}

/// `Roughing` over a part with an enclosed slot 2.8 mm wide for its 2 mm
/// cutter (0.1 mm radial allowance): the same rule for each clearing.
#[test]
fn roughing_refuses_an_enclosed_slot_no_entry_fits() {
    let front = || {
        serde_json::json!({
            "kind": "adaptive", "path_strategy": "engagement_front",
            "max_radial_engagement_mm": 0.5, "orbit_radius_mm": null,
            "min_orbit_radius_mm": null, "local_orbit_tolerance_mm": null,
            "boundary_tolerance_mm": 0.01,
        })
    };
    let stock_edge_or_helix =
        || serde_json::json!({"kind": "stock_edge_or_helix", "angle_rad": 0.05235987755982989});
    let slot = |width: f64| -> Rect { [[10., 14. - width / 2.], [22., 14. + width / 2.]] };
    // Bottom allowance 0.2 over the slot floor at Z6.
    let floor = 6.2;
    // The slot is cut: between its end radii and two stock cells off its
    // walls (roughing makes no wall pass), no stock cell certainly holds
    // material above the floor. (Certain material, the cells' lower bound:
    // the planner leaves slivers too thin to
    // cut, which it reports as residual area, and a cell with one in it
    // keeps its upper bound.)
    let cut = |seen: &Run, r: Rect| {
        let stock = &seen.stock;
        let mut worst = (f64::NEG_INFINITY, [0., 0.]);
        let mut count = 0;
        for (i, cell) in stock.cells().iter().enumerate() {
            let p = stock.cell_center(i).unwrap();
            if p[0] > r[0][0] + 1.9
                && p[0] < r[1][0] - 1.9
                && p[1] > r[0][1] + 0.8
                && p[1] < r[1][1] - 0.8
            {
                count += 1;
                if cell.lower_mm > worst.0 {
                    worst = (cell.lower_mm, p);
                }
            }
        }
        assert!(count > 20, "{count}");
        // (The stock's own height reserve is under a micron.)
        assert!(
            worst.0 <= floor + 1e-3,
            "stock left at Z{} at {:?} in {r:?}: {}",
            worst.0,
            worst.1,
            seen.report
        );
        true
    };
    // The engagement front: refused at the first layer, before any motion
    // (the part covers the whole stock: there is nothing else to cut).
    const NARROW: f64 = 2.8;
    let m = roughing(front(), stock_edge_or_helix(), NARROW);
    m.validate().unwrap();
    let seen = run(&m, m.operations[0].clone());
    let error = seen.error.expect("an enclosed slot no helix fits");
    for part in [
        "no entry into the enclosed material of rough at (",
        "for the level at Z8",
        "roughing layer 0 at Z8",
        "no helix fits",
        "entry=RampEntry(angle_deg)",
    ] {
        assert!(error.contains(part), "{part:?} not in: {error}");
    }
    assert!(inside(named_point(&error), slot(NARROW)), "{error}");
    assert!(seen.motions.is_empty());
    assert!(untouched(&seen.stock, slot(NARROW)));
    // An entry that allows no helix was refused before, and still is,
    // before any motion.
    let m = roughing(
        front(),
        serde_json::json!({"kind": "stock_edge", "angle_rad": 0.05235987755982989}),
        NARROW,
    );
    let seen = run(&m, m.operations[0].clone());
    let error = seen.error.expect("no stock-edge entry into an enclosed slot");
    assert!(error.contains("entry"), "{error}");
    assert!(seen.motions.is_empty());
    // With a ramp it plans and the slot is cut.
    let m = roughing(
        front(),
        serde_json::json!({"kind": "ramp", "angle_rad": 0.1, "min_length_mm": 1.0}),
        NARROW,
    );
    let seen = run(&m, m.operations[0].clone());
    assert_eq!(seen.error, None);
    assert!(cut(&seen, slot(NARROW)));
    // A slot the helix fits plans with the default entry.
    let m = roughing(front(), stock_edge_or_helix(), 5.);
    let seen = run(&m, m.operations[0].clone());
    assert_eq!(seen.error, None);
    assert!(seen.report["helical_entries"].as_u64().unwrap() >= 1);
    assert!(cut(&seen, slot(5.)));
    // Roughing that does cut something is not refused for the slot: a
    // layer legitimately has material its cutter cannot enter. Each layer
    // lists the area in its report, typed, and counts it in its residual.
    let m = roughing_from(1.5, front(), stock_edge_or_helix(), NARROW);
    let seen = run(&m, m.operations[0].clone());
    assert_eq!(seen.error, None);
    assert!(seen.motions.iter().any(|motion| motion.kind == MoveKind::Cut));
    assert!(untouched(&seen.stock, slot(NARROW)));
    let layers = seen.report["layers"].as_array().unwrap();
    assert_eq!(layers.len(), 2);
    for layer in layers {
        let areas = layer["clearing"]["unentered_areas"].as_array().unwrap();
        assert_eq!(areas.len(), 1, "{layer}");
        let area = &areas[0];
        assert_eq!(area["reason"], "no_helix_fits");
        assert_eq!(area["level_z_mm"], layer["tip_z_mm"]);
        assert_eq!(area["cutter_radius_mm"], 1.);
        assert_eq!(
            area["radii_tried_mm"],
            serde_json::json!([0.5, 0.25, 0.125, 0.1])
        );
        assert!(area["available_mm"].as_f64().unwrap() < 0.25);
        let point = |name: &str| {
            let p = area[name].as_array().unwrap();
            [p[0].as_f64().unwrap(), p[1].as_f64().unwrap()]
        };
        // The place and the extent are tool centres in the slot, and the
        // extent spans its length between the end radii.
        for name in ["at_mm", "min_mm", "max_mm"] {
            assert!(inside(point(name), slot(NARROW)), "{name}: {area}");
        }
        assert!(point("max_mm")[0] - point("min_mm")[0] > 9., "{area}");
        // Its stock is in the residual: what the cutter reaches from those
        // centres (over 9 mm of them, swept 2 mm wide) is left.
        let left = layer["clearing"]["residual_reachable_area_mm2"]
            .as_f64()
            .unwrap();
        assert!(left > 18., "{left}: {layer}");
        assert!(layer["clearing"]["geometric_residual_area_mm2"].as_f64().unwrap() >= left);
    }
    // A slot 2.3 mm wide leaves the cutter (1.1 mm with the allowance)
    // 0.05 mm a side, less than the engagement front plans in. With a ramp
    // entry nothing else stands in its way, and the layer lists it with
    // that reason and counts its stock: the part of the residual the
    // cutter reaches in the region and the planner's positions do not.
    let ramped = || serde_json::json!({"kind": "ramp", "angle_rad": 0.1, "min_length_mm": 1.0});
    let tight = run(&m, {
        let m = roughing_from(1.5, front(), ramped(), 2.3);
        m.operations[0].clone()
    });
    let wide = run(&m, {
        let m = roughing_from(1.5, front(), ramped(), NARROW);
        m.operations[0].clone()
    });
    assert_eq!(tight.error, None);
    assert_eq!(wide.error, None);
    assert!(untouched(&tight.stock, slot(2.3)));
    for (layer, cut) in tight.report["layers"]
        .as_array()
        .unwrap()
        .iter()
        .zip(wide.report["layers"].as_array().unwrap())
    {
        let areas = layer["clearing"]["unentered_areas"].as_array().unwrap();
        assert_eq!(areas.len(), 1, "{layer}");
        assert_eq!(areas[0]["reason"], "tighter_than_planner_resolves");
        let at = areas[0]["at_mm"].as_array().unwrap();
        assert!(
            inside(
                [at[0].as_f64().unwrap(), at[1].as_f64().unwrap()],
                slot(2.3)
            ),
            "{layer}"
        );
        assert!(cut["clearing"].get("unentered_areas").is_none(), "{cut}");
        let unresolved = |layer: &serde_json::Value| {
            layer["clearing"]["residual_unresolved_area_mm2"]
                .as_f64()
                .unwrap()
        };
        // The slot's stock within the cutter's reach: about 12 x 2 mm.
        assert!(unresolved(layer) > unresolved(cut) + 15., "{layer}");
    }
    // stock_front, whose orbits sweep more than the cutter (radius 1 + 0.5
    // orbit, 0.1 off the walls: 3.2 mm, in a 2.8 mm slot): the layer lists
    // the slot as an area no orbit fits in.
    let orbits = serde_json::json!({
        "kind": "adaptive", "path_strategy": "stock_front",
        "max_radial_engagement_mm": 0.5, "orbit_radius_mm": null,
        "min_orbit_radius_mm": null, "local_orbit_tolerance_mm": null,
        "boundary_tolerance_mm": 0.01,
    });
    let m = roughing_from(1.5, orbits, stock_edge_or_helix(), NARROW);
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
    for layer in report["layers"].as_array().unwrap() {
        let areas = layer["clearing"]["unentered_areas"].as_array().unwrap();
        assert_eq!(areas.len(), 1, "{layer}");
        assert_eq!(areas[0]["reason"], "no_orbit_fits");
        assert_eq!(areas[0]["needed_mm"], 0.5);
        let at = areas[0]["at_mm"].as_array().unwrap();
        assert!(
            inside(
                [at[0].as_f64().unwrap(), at[1].as_f64().unwrap()],
                slot(NARROW)
            ),
            "{layer}"
        );
    }
    // Raster and offset layers fit their entries before any motion: refused
    // there, or cut.
    for clearing in [
        serde_json::json!({"kind": "raster", "stepover_mm": 0.8}),
        serde_json::json!({"kind": "offset", "stepover_mm": 0.8}),
    ] {
        let m = roughing(clearing.clone(), stock_edge_or_helix(), NARROW);
        let seen = run(&m, m.operations[0].clone());
        match &seen.error {
            Some(error) => {
                assert!(error.contains("entry"), "{clearing}: {error}");
                assert!(seen.motions.is_empty());
            }
            None => assert!(cut(&seen, slot(NARROW)), "{clearing} left the slot"),
        }
        eprintln!("roughing {clearing}: {:?}", seen.error);
    }
}
