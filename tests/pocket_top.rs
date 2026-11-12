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
            pocket,
            path_strategy: AdaptivePathStrategy::DiskFront,
            cutting_sense: None,
            orbit_radius_mm: Some(1.),
            min_orbit_radius_mm: None,
            local_orbit_tolerance_mm: None,
            max_engagement_deg: 90.,
            boundary_tolerance_mm: 0.01,
            clockwise: true,
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
                assert_eq!(report["depth_passes"], 2);
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
        assert!(
            results[1].0 < results[0].0,
            "the extra air depth must disappear"
        );
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
        assert!(
            OperationPlan::prepare_with_stock(
                operation(8., adaptive),
                &m.tools[0],
                &original,
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
