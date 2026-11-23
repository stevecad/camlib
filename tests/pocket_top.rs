use camlib::{
    adaptive::{AdaptivePathStrategy, AdaptivePocketOperation},
    motion::{Coolant, Motion, MoveKind},
    operation::OperationPlan,
    planar::{Boundary, Component, PlanarRegion},
    pocket::{Operation, PocketOperation},
    stock::{Stock, SweepOptions},
    stream::StreamManifest,
};
use std::sync::atomic::AtomicBool;

fn fixture() -> StreamManifest {
    serde_json::from_str(include_str!("../fixtures/operations/facing.json")).unwrap()
}
fn operation(top: f64, adaptive: bool) -> Operation {
    let pocket = PocketOperation {
        max_stay_down_mm: None,
        limits: None,
        id: "pocket-after-facing".into(),
        tool: "face8".into(),
        region: PlanarRegion {
            version: 1,
            units: "mm".into(),
            name: "pocket".into(),
            boundary_error_mm: 0.,
            source: serde_json::Value::Null,
            components: vec![Component {
                outer: Boundary {
                    vertices: vec![[6., 4., 0.], [34., 4., 0.], [34., 20., 0.], [6., 20., 0.]],
                },
                holes: vec![],
            }],
        },
        top_z_mm: top,
        floor_z_mm: 5.,
        stepdown_mm: 2.,
        stepover_mm: 3.,
        feed_mm_min: 600.,
        plunge_mm_min: 150.,
        clearance_z_mm: 12.,
        radial_allowance_mm: 0.,
        bottom_allowance_mm: 0.,
        tolerance_mm: 1e-5,
        spindle_rpm: 12000.,
        work_offset: 1,
        tool_number: 1,
        length_register: 1,
        coolant: Coolant::Flood,
        rest_tolerance_mm: None,
        boundary_clockwise: None,
        entry: None,
        retract_z_mm: None,
        feed_z_mm: None,
    };
    if adaptive {
        Operation::AdaptivePocket(AdaptivePocketOperation {
            max_corner_engagement_deg: None,
            pocket,
            path_strategy: AdaptivePathStrategy::EngagementFront,
            cutting_sense: None,
            max_engagement_deg: 90.,
            rapid_descent_clearance_mm: None,
            ramp_clearance_mm: None,
            link_feed_mm_min: None,
            link_chip_thinning: false,
        })
    } else {
        Operation::Pocket(pocket)
    }
}
fn commit(stock: &mut Stock, motion: &Motion, m: &StreamManifest) {
    let delta = stock
        .preview_sweep(
            &m.tools[0].prepare().unwrap(),
            motion,
            1.,
            SweepOptions::default(),
            &|| false,
        )
        .unwrap();
    stock.apply_delta(&delta).unwrap();
}
fn faced(m: &StreamManifest) -> (Stock, [f64; 3], u64) {
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
    let mut position = m.initial_position_mm;
    let mut next = 1;
    while let Some(motion) = plan.next_motion_with_stock(&stock, &|| false).unwrap() {
        position = motion.curve.end();
        next = motion.id + 1;
        commit(&mut stock, &motion, m);
    }
    assert!(stock.cells().iter().all(|c| c.upper_mm <= 8.00016));
    (stock, position, next)
}

#[test]
fn pocket_and_adaptive_use_faced_stock_top_without_resetting_stock_or_extra_depths() {
    let m = fixture();
    for adaptive in [false, true] {
        let (stock, position, next) = faced(&m);
        let mut results = Vec::new();
        for top in [10., 8.] {
            let mut current = Stock::from_snapshot(stock.snapshot()).unwrap();
            let mut plan = OperationPlan::prepare_with_stock(
                operation(top, adaptive),
                &m.tools[0],
                &current,
                position,
                next,
                &|| false,
            )
            .unwrap();
            let report = serde_json::to_value(plan.report()).unwrap();
            if top == 8. {
                assert_eq!(report["stock_top"]["generation"], current.generation());
                if adaptive {
                    // Levels from the operation top, a stepdown apart.
                    let tips: Vec<f64> = report["layers"]
                        .as_array()
                        .unwrap()
                        .iter()
                        .map(|l| l["tip_z_mm"].as_f64().unwrap())
                        .collect();
                    // The faced stock model stands its sweep guard (within
                    // the operation's tolerance) above Z8: that is the
                    // faced floor, and the levels are whole stepdowns
                    // below it (#344, #381).
                    assert_eq!(tips, [6., 5.]);
                } else {
                    assert_eq!(report["depth_passes"], 2);
                }
            }
            let mut count = 0;
            while let Some(motion) = plan.next_motion_with_stock(&current, &|| false).unwrap() {
                if motion.kind == MoveKind::Cut {
                    assert!(motion.curve.start()[2] <= 8. && motion.curve.end()[2] >= 5.);
                }
                commit(&mut current, &motion, &m);
                count += 1;
            }
            assert_eq!(current.spec(), m.stock.height().unwrap());
            results.push((count, current));
        }
        // The extra air depth must disappear: a Pocket plans one depth pass
        // fewer; the engagement front plans each level from the stock, so
        // its level in the air above the faced top makes no move.
        if adaptive {
            assert!(
                results[1].0 <= results[0].0,
                "a move in the air depth: {} / {}",
                results[1].0,
                results[0].0
            );
        } else {
            assert!(
                results[1].0 < results[0].0,
                "the extra air depth must disappear"
            );
        }
        for (a, b) in results[0].1.cells().iter().zip(results[1].1.cells()) {
            assert!((a.lower_mm - b.lower_mm).abs() < 1e-9);
            assert!((a.upper_mm - b.upper_mm).abs() < 1e-9);
        }
    }
}

#[test]
fn lower_top_rejects_uncut_and_same_generation_foreign_stock_but_accepts_restored_faced_stock() {
    let m = fixture();
    for adaptive in [false, true] {
        let original = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let error = OperationPlan::prepare_with_stock(
            operation(8., adaptive),
            &m.tools[0],
            &original,
            m.initial_position_mm,
            1,
            &|| false,
        )
        .err()
        .unwrap();
        assert!(error.contains("raise the operation top"), "{error}");
        // Without committed stock a lower top cannot be proven.
        assert!(
            OperationPlan::prepare(
                operation(8., adaptive),
                &m.tools[0],
                m.stock.height().unwrap(),
                m.initial_position_mm,
                1,
                &|| false
            )
            .is_err()
        );
        let (stock, position, next) = faced(&m);
        let mut plan = OperationPlan::prepare_with_stock(
            operation(8., adaptive),
            &m.tools[0],
            &stock,
            position,
            next,
            &|| false,
        )
        .unwrap();
        // Stock above the top where the cutter can reach it (cell (80, 48)
        // is at X20 Y12, inside the region) is refused on the rescan.
        let inside = 48 * 160 + 80;
        let mut changed = stock.snapshot();
        changed.cells[inside].upper_mm = 9.;
        let foreign = Stock::from_snapshot(changed).unwrap();
        let error = plan
            .next_motion_with_stock(&foreign, &|| false)
            .unwrap_err();
        assert!(
            error.contains(&format!("cell {inside}")) && error.contains("raise the operation top"),
            "{error}"
        );
        // Outside the region it is judged by the assembly (issue #129):
        // 1 mm above the top, far below the shank, it is accepted.
        let mut changed = stock.snapshot();
        changed.cells[0].upper_mm = 9.;
        let outside = Stock::from_snapshot(changed).unwrap();
        let mut probe = OperationPlan::prepare_with_stock(
            operation(8., adaptive),
            &m.tools[0],
            &outside,
            position,
            next,
            &|| false,
        )
        .unwrap();
        assert!(
            probe
                .next_motion_with_stock(&outside, &|| false)
                .unwrap()
                .is_some()
        );
        let restored = Stock::from_snapshot(stock.snapshot()).unwrap();
        assert!(
            plan.next_motion_with_stock(&restored, &|| false)
                .unwrap()
                .is_some()
        );
        assert!(
            plan.next_motion_with_stock(&restored, &|| false).is_err(),
            "commit is required before advancing"
        );
    }
}

#[test]
fn cancelled_faced_stock_inspection_does_not_change_stock() {
    let m = fixture();
    let (stock, position, next) = faced(&m);
    let before = stock.snapshot();
    let cancelled = AtomicBool::new(true);
    assert!(
        OperationPlan::prepare_with_stock(
            operation(8., false),
            &m.tools[0],
            &stock,
            position,
            next,
            &cancelled
        )
        .is_err()
    );
    assert_eq!(stock.generation(), before.generation);
    assert_eq!(stock.cells(), before.cells);
}

/// A top within the stock-model band below uncut stock is accepted as the
/// stock top, but the first level must still be at most a stepdown below the
/// stock it meets (#344 review): measured on the stock each motion removes.
#[test]
fn top_inside_the_tolerance_band_never_cuts_deeper_than_a_stepdown() {
    let m = fixture();
    for adaptive in [false, true] {
        let mut op = operation(9.9, adaptive);
        let pocket = match &mut op {
            Operation::Pocket(p) => p,
            Operation::AdaptivePocket(a) => &mut a.pocket,
            _ => unreachable!(),
        };
        pocket.tolerance_mm = 0.01;
        pocket.stepdown_mm = 1.;
        pocket.floor_z_mm = 7.;
        let stepdown = pocket.stepdown_mm;
        let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let mut plan = OperationPlan::prepare_with_stock(
            op,
            &m.tools[0],
            &stock,
            m.initial_position_mm,
            1,
            &|| false,
        )
        .unwrap();
        let mut deepest: f64 = 0.;
        while let Some(motion) = plan.next_motion_with_stock(&stock, &|| false).unwrap() {
            let before: Vec<f64> = stock.cells().iter().map(|c| c.upper_mm).collect();
            commit(&mut stock, &motion, &m);
            for (b, a) in before.iter().zip(stock.cells()) {
                deepest = deepest.max(b - a.upper_mm);
            }
        }
        assert!(
            deepest <= stepdown + 1e-6,
            "adaptive {adaptive}: one motion removed {deepest} mm of stock depth, more than the stepdown {stepdown}"
        );
    }
}

#[test]
fn adaptive_last_level_reaches_the_floor_for_a_fractional_final_step() {
    let m = fixture();
    let stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let floor = 8. - 5e-10;
    let op = band_operation(10., true, |p| p.floor_z_mm = floor);
    let plan = OperationPlan::prepare_with_stock(
        op,
        &m.tools[0],
        &stock,
        m.initial_position_mm,
        1,
        &|| false,
    )
    .unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    let tips: Vec<_> = report["layers"]
        .as_array()
        .unwrap()
        .iter()
        .map(|layer| layer["tip_z_mm"].as_f64().unwrap())
        .collect();
    assert_eq!(tips, [9., 8., floor]);
}

fn band_operation(top: f64, adaptive: bool, edit: impl Fn(&mut PocketOperation)) -> Operation {
    let mut op = operation(top, adaptive);
    let pocket = match &mut op {
        Operation::Pocket(p) => p,
        Operation::AdaptivePocket(a) => &mut a.pocket,
        _ => unreachable!(),
    };
    pocket.tolerance_mm = 0.01;
    pocket.stepdown_mm = 1.;
    pocket.floor_z_mm = 7.;
    edit(pocket);
    op
}

/// #344 review: the depths are planned from the stock seen at preparation.
/// Stock supplied later that stands higher within the band, where the cutter
/// reaches, would make the next cut deeper than a stepdown: refused on the
/// rescan, before any motion.
#[test]
fn a_rescan_refuses_stock_above_the_top_the_depths_were_planned_from() {
    let m = fixture();
    for adaptive in [false, true] {
        let blank = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let mut lower = blank.snapshot();
        for cell in &mut lower.cells {
            cell.lower_mm = 9.9;
            cell.upper_mm = 9.9;
        }
        let lower = Stock::from_snapshot(lower).unwrap();
        let mut plan = OperationPlan::prepare_with_stock(
            band_operation(9.9, adaptive, |_| ()),
            &m.tools[0],
            &lower,
            m.initial_position_mm,
            1,
            &|| false,
        )
        .unwrap();
        let error = plan
            .next_motion_with_stock(&blank, &|| false)
            .unwrap_err();
        assert!(
            error.contains("stock changed after planning") && error.contains("Z10"),
            "adaptive {adaptive}: {error}"
        );
    }
}

/// #344 review: a floor above the authored top is refused even where the
/// depths would be planned from higher stock.
#[test]
fn a_floor_above_the_authored_top_is_refused_on_stock_in_the_band() {
    let m = fixture();
    for adaptive in [false, true] {
        let blank = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let error = OperationPlan::prepare_with_stock(
            band_operation(9.9, adaptive, |p| p.floor_z_mm = 9.92),
            &m.tools[0],
            &blank,
            m.initial_position_mm,
            1,
            &|| false,
        )
        .err()
        .unwrap();
        assert!(
            error.contains("must be below top_z_mm 9.9"),
            "adaptive {adaptive}: {error}"
        );
    }
}

#[test]
fn lower_top_requires_the_planned_sweep_before_advancing_to_the_next_layer() {
    let m = fixture();
    for adaptive in [false, true] {
        let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let blank = stock.snapshot();
        let mut plan = OperationPlan::prepare_with_stock(
            band_operation(9.9, adaptive, |_| ()),
            &m.tools[0],
            &stock,
            m.initial_position_mm,
            1,
            &|| false,
        )
        .unwrap();
        // Stop during the entry into the second level, with moves still queued.
        for _ in 0..1000 {
            let motion = plan.next_motion_with_stock(&stock, &|| false).unwrap().unwrap();
            let z = motion.curve.end()[2];
            if z < 8.9 && motion.kind.cuts() {
                let mut restored = blank.clone();
                restored.generation = stock.generation() + 1;
                let foreign = Stock::from_snapshot(restored).unwrap();
                let err = plan.next_motion_with_stock(&foreign, &|| false).unwrap_err();
                assert!(err.contains("commit the complete preceding motion"), "{err}");

                let mut partial = stock.clone();
                let delta = partial.preview_sweep(
                    &m.tools[0].prepare().unwrap(), &motion, 0.5, SweepOptions::default(), &|| false,
                ).unwrap();
                partial.apply_delta(&delta).unwrap();
                let err = plan.next_motion_with_stock(&partial, &|| false).unwrap_err();
                assert!(err.contains("commit the complete preceding motion"), "{err}");

                commit(&mut stock, &motion, &m);
                assert!(plan.next_motion_with_stock(&stock, &|| false).unwrap().is_some());
                break;
            }
            commit(&mut stock, &motion, &m);
            assert!(motion.id < 1000, "did not reach the second level");
        }
    }
}

#[test]
fn stock_in_the_gap_between_pockets_does_not_raise_the_planning_top() {
    let m = fixture();
    let mut low = Stock::new(m.stock.height().unwrap().clone()).unwrap().snapshot();
    for cell in &mut low.cells {
        cell.lower_mm = 9.9;
        cell.upper_mm = 9.9;
    }
    let mut raised = low.clone();
    raised.cells[48 * 160 + 80].upper_mm = 10.;
    let low = Stock::from_snapshot(low).unwrap();
    let raised = Stock::from_snapshot(raised).unwrap();
    let op = band_operation(9.9, true, |p| {
        p.region.components[0].outer.vertices =
            vec![[1., 1., 0.], [12., 1., 0.], [12., 22., 0.], [1., 22., 0.]];
        let mut second = p.region.components[0].clone();
        for vertex in &mut second.outer.vertices {
            vertex[0] += 26.;
        }
        p.region.components.push(second);
    });
    let prepare = |stock| {
        OperationPlan::prepare_with_stock(
            op.clone(), &m.tools[0], stock, m.initial_position_mm, 1, &|| false,
        )
        .unwrap()
    };
    let report = serde_json::to_value(prepare(&raised).report()).unwrap();
    assert_eq!(report["stock_top"]["maximum_upper_z_mm"], 10.);
    assert!(report["stock_top"].get("reachable_upper_z_mm").is_none());
    assert!(prepare(&low).next_motion_with_stock(&raised, &|| false).unwrap().is_some());
}

/// #381: a 100 x 60 plate in stock 1 mm taller, faced to exactly Z20 with a
/// 6 mm end mill, then an adaptive pocket from that faced floor to Z12.
fn faced_plate(cell: f64) -> (StreamManifest, Stock, [f64; 3], u64) {
    let mut m: serde_json::Value =
        serde_json::from_str(include_str!("../fixtures/operations/facing.json")).unwrap();
    m["stock"]["max_mm"] = serde_json::json!([100., 60., 21.]);
    m["stock"]["cell_size_mm"] = cell.into();
    m["tools"][0]["cutter"]["diameter_mm"] = 6.into();
    m["tools"][0]["cutter"]["shank_diameter_mm"] = 6.into();
    m["initial_position_mm"] = serde_json::json!([0., 0., 25.]);
    let face = &mut m["operations"][0];
    face["floor_z_mm"] = 20.into();
    face["stepover_mm"] = 4.into();
    face["clearance_z_mm"] = 25.into();
    let m: StreamManifest = serde_json::from_value(m).unwrap();
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
    let mut position = m.initial_position_mm;
    let mut next = 1;
    while let Some(motion) = plan.next_motion_with_stock(&stock, &|| false).unwrap() {
        position = motion.curve.end();
        next = motion.id + 1;
        commit(&mut stock, &motion, &m);
    }
    (m, stock, position, next)
}

fn plate_pocket(
    stepdown: f64,
    entry: camlib::pocket::PocketEntry,
    top: f64,
    floor: f64,
) -> Operation {
    let mut op = operation(top, true);
    let Operation::AdaptivePocket(a) = &mut op else {
        unreachable!()
    };
    let p = &mut a.pocket;
    p.id = "plate-pocket".into();
    p.region.components[0].outer.vertices = vec![
        [20., 15., 0.],
        [80., 15., 0.],
        [80., 45., 0.],
        [20., 45., 0.],
    ];
    p.floor_z_mm = floor;
    p.stepdown_mm = stepdown;
    p.stepover_mm = 1.5;
    p.clearance_z_mm = 25.;
    p.entry = Some(entry);
    op
}

/// #381: an adaptive pocket from a floor a Facing just cut, to a whole
/// number of stepdowns below it. The faced stock model stands its sweep
/// guard above Z20; the levels used to be planned from that guard, which
/// left a last level a guard deep at the floor (the whole region a sheet
/// that thick), and its engagement proof refused it. The levels are whole
/// stepdowns from the faced floor, each one cuts, the last is at the floor,
/// and the pocket is released with either entry. No motion removes more
/// than a stepdown of the stock model (as #344 requires).
#[test]
fn adaptive_pocket_from_a_faced_floor_cuts_whole_stepdowns_to_the_floor() {
    use camlib::pocket::{HelicalEntry, PocketEntry, RampEntry};
    let (m, faced, position, next) = faced_plate(0.5);
    let tool = m.tools[0].prepare().unwrap();
    let inside = |p: [f64; 2], margin: f64| {
        (20. + margin..=80. - margin).contains(&p[0])
            && (15. + margin..=45. - margin).contains(&p[1])
    };
    let centres: Vec<[f64; 2]> = (0..faced.cells().len())
        .map(|i| faced.cell_center(i).unwrap())
        .collect();
    // The faced floor: the model's upper bound is above Z20, by no more than
    // the sweep guard, where the pocket is.
    let faced_upper = faced
        .cells()
        .iter()
        .zip(&centres)
        .filter(|(_, p)| inside(**p, 0.))
        .map(|(c, _)| c.upper_mm)
        .fold(f64::MIN, f64::max);
    assert!(
        faced_upper > 20. && faced_upper <= 20. + 1e-5,
        "{faced_upper}"
    );
    for (stepdown, expected) in [
        (2., vec![18., 16., 14., 12.]),
        (3., vec![17., 14., 12.]),
        (8., vec![12.]),
    ] {
        for entry in [
            PocketEntry::Helix(HelicalEntry::default()),
            PocketEntry::Ramp(RampEntry {
                angle_rad: 5f64.to_radians(),
                min_length_mm: Some(10.),
                feed_mm_min: None,
            }),
        ] {
            let what = format!("stepdown {stepdown}, {entry:?}");
            let mut stock = Stock::from_snapshot(faced.snapshot()).unwrap();
            let mut plan = OperationPlan::prepare_with_stock(
                plate_pocket(stepdown, entry, 20., 12.),
                &m.tools[0],
                &stock,
                position,
                next,
                &|| false,
            )
            .unwrap();
            let report = serde_json::to_value(plan.report()).unwrap();
            let tips: Vec<f64> = report["layers"]
                .as_array()
                .unwrap()
                .iter()
                .map(|l| l["tip_z_mm"].as_f64().unwrap())
                .collect();
            assert_eq!(tips, expected, "{what}");
            let mut deepest: f64 = 0.;
            // Stock cells each level's cutting motions changed.
            let mut changed = vec![0usize; tips.len()];
            while let Some(motion) = plan
                .next_motion_with_stock(&stock, &|| false)
                .unwrap_or_else(|e| panic!("{what}: {e}"))
            {
                let delta = stock
                    .preview_sweep(&tool, &motion, 1., SweepOptions::default(), &|| false)
                    .unwrap();
                let before: Vec<f64> = stock.cells().iter().map(|c| c.upper_mm).collect();
                stock.apply_delta(&delta).unwrap();
                for (b, a) in before.iter().zip(stock.cells()) {
                    deepest = deepest.max(b - a.upper_mm);
                }
                let end = motion.curve.end();
                if motion.kind.cuts()
                    && motion.curve.start()[2] == end[2]
                    && let Some(level) = tips.iter().position(|z| *z == end[2])
                {
                    changed[level] += delta.changes().len();
                }
            }
            assert!(
                deepest <= stepdown + 1e-9,
                "{what}: one motion removed {deepest} mm of the stock model, more than the stepdown"
            );
            assert!(changed.iter().all(|n| *n > 0), "{what}: {changed:?}");
            // The floor is reached at exactly Z12 inside the pocket.
            let left = stock
                .cells()
                .iter()
                .zip(&centres)
                .filter(|(_, p)| inside(**p, 4.))
                .map(|(c, _)| c.upper_mm)
                .fold(f64::MIN, f64::max);
            assert!(left <= 12. + 1e-5, "{what}: stock up to Z{left}");
        }
    }
    // A Pocket from the same faced floor: whole stepdowns, no extra depth.
    let mut op = plate_pocket(2., PocketEntry::Helix(HelicalEntry::default()), 20., 12.);
    if let Operation::AdaptivePocket(a) = op {
        op = Operation::Pocket(a.pocket);
    }
    let plan =
        OperationPlan::prepare_with_stock(op, &m.tools[0], &faced, position, next, &|| false)
            .unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    assert_eq!(report["depth_passes"], 4);
}

/// #381: stock more than the operation's tolerance above the top, where the
/// cutter reaches, still raises the top the depths are planned from (#344),
/// however close to the tolerance it is.
#[test]
fn stock_just_beyond_the_tolerance_above_the_top_still_raises_the_planning_top() {
    let m = fixture();
    // Top Z9, floor Z5, stepdown 2 (`operation`), tolerance 1e-5.
    for (excess, planned) in [(1e-5, 9.), (1.1e-5, 9. + 1.1e-5)] {
        let mut stock = Stock::new(m.stock.height().unwrap().clone())
            .unwrap()
            .snapshot();
        for cell in &mut stock.cells {
            cell.lower_mm = 9.;
            cell.upper_mm = 9.;
        }
        // Cell (80, 48), at X20 Y12, inside the region.
        stock.cells[48 * 160 + 80].upper_mm = 9. + excess;
        let stock = Stock::from_snapshot(stock).unwrap();
        for adaptive in [false, true] {
            let plan = OperationPlan::prepare_with_stock(
                operation(9., adaptive),
                &m.tools[0],
                &stock,
                m.initial_position_mm,
                1,
                &|| false,
            )
            .unwrap();
            let report = serde_json::to_value(plan.report()).unwrap();
            assert_eq!(
                report["stock_top"]["reachable_upper_z_mm"].as_f64(),
                Some(9. + excess)
            );
            // From the raised top, 4.000011 mm takes three stepdowns.
            let depths = if planned > 9. { 3 } else { 2 };
            if adaptive {
                let first = report["layers"][0]["tip_z_mm"].as_f64().unwrap();
                assert_eq!(first, planned - 2., "excess {excess}");
                assert_eq!(report["layers"].as_array().unwrap().len(), depths);
            } else {
                assert_eq!(report["depth_passes"], depths, "excess {excess}");
            }
        }
    }
}
