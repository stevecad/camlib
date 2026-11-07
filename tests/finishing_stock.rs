use camlib::{
    operation::OperationPlan,
    pocket::Operation,
    stock::{HeightBounds, Stock},
    stream::StreamManifest,
};
use std::cell::Cell;

fn case(waterline: bool) -> StreamManifest {
    let source = if waterline {
        include_str!("../fixtures/operations/waterline.json")
    } else {
        include_str!("../fixtures/operations/parallel.json")
    };
    let mut m: StreamManifest = serde_json::from_str(source).unwrap();
    m.stock.height_mut().unwrap().max_mm[2] = 12.;
    m.initial_position_mm[2] = 14.;
    m.tools[0].cutter.cutting_length_mm = if waterline { 8. } else { 10. };
    match &mut m.operations[0] {
        Operation::Parallel(p) => p.clearance_z_mm = 14.,
        Operation::Waterline(p) => {
            p.clearance_z_mm = 14.;
        }
        _ => unreachable!(),
    }
    m
}

#[test]
fn short_ball_finishing_uses_remaining_stock_and_preserves_complete_paths() {
    for waterline in [false, true] {
        let m = case(waterline);
        let original = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let mut snapshot = original.snapshot();
        snapshot.cells.fill(HeightBounds {
            lower_mm: 9.,
            upper_mm: 9.,
        });
        let mut actual = Stock::from_snapshot(snapshot.clone()).unwrap();
        let prepare = |stock: &Stock, cancel: &dyn Fn() -> bool| {
            OperationPlan::prepare_with_stock(
                m.operations[0].clone(),
                &m.tools[0],
                stock,
                m.initial_position_mm,
                1,
                &cancel,
            )
        };
        // Stock standing higher above the floor than the flutes reach.
        // Waterline refuses it while preparing. Parallel checks the depth it
        // actually cuts, move by move against the stock (issue #88): the
        // roof keeps its tip at Z4 or above, 8 mm under the 12 mm stock top
        // and within the 10 mm flutes, so it is accepted; with 7 mm flutes
        // the same passes really cut deeper and are refused.
        let refused = |tool: &camlib::tool::ToolAssembly, stock: &Stock| {
            let prepared = tool.prepare().unwrap();
            match OperationPlan::prepare_with_stock(
                m.operations[0].clone(),
                tool,
                stock,
                m.initial_position_mm,
                1,
                &|| false,
            ) {
                Err(_) => true,
                Ok(mut plan) => {
                    let mut stock = stock.clone();
                    loop {
                        match plan.next_motion_with_stock(&stock, &|| false) {
                            Err(e) => break e.contains("cuts deeper than the cutting length"),
                            Ok(Some(motion)) => {
                                let delta = stock
                                    .preview_sweep(&prepared, &motion, 1., m.sweep, &|| false)
                                    .unwrap();
                                stock.apply_delta(&delta).unwrap();
                            }
                            Ok(None) => break false,
                        }
                    }
                }
            }
        };
        let mut seven = m.tools[0].clone();
        seven.cutter.cutting_length_mm = 7.;
        if waterline {
            assert!(prepare(&original, &|| false).is_err());
        } else {
            assert!(!refused(&m.tools[0], &original));
            assert!(refused(&seven, &original));
        }
        let polls = Cell::new(0);
        assert!(
            prepare(&actual, &|| {
                polls.set(polls.get() + 1);
                polls.get() >= 2
            })
            .is_err()
        );
        let mut plan = prepare(&actual, &|| false).unwrap();
        let report = serde_json::to_value(plan.report()).unwrap();
        assert_eq!(report["stock_top"]["maximum_upper_z_mm"], 9.);
        assert_eq!(report["stock_top"]["original_top_z_mm"], 12.);
        // A single uncertain high column invalidates the whole-stock premise.
        snapshot.cells.last_mut().unwrap().upper_mm = 12.;
        let unknown = Stock::from_snapshot(snapshot).unwrap();
        if waterline {
            assert!(prepare(&unknown, &|| false).is_err());
        } else {
            assert!(refused(&seven, &unknown));
        }
        assert!(plan.next_motion_with_stock(&unknown, &|| false).is_err());
        assert!(plan.next_motion_with_stock(&actual, &|| true).is_err());

        let mut long_tool = m.tools[0].clone();
        long_tool.cutter.cutting_length_mm = 20.;
        let mut reference_stock = actual.clone();
        let mut reference = OperationPlan::prepare_with_stock(
            m.operations[0].clone(),
            &long_tool,
            &reference_stock,
            m.initial_position_mm,
            1,
            &|| false,
        )
        .unwrap();
        let short = m.tools[0].prepare().unwrap();
        let long = long_tool.prepare().unwrap();
        let mut motions = 0;
        while let Some(motion) = plan.next_motion_with_stock(&actual, &|| false).unwrap() {
            assert_eq!(
                Some(motion.clone()),
                reference
                    .next_motion_with_stock(&reference_stock, &|| false)
                    .unwrap()
            );
            let delta = actual
                .preview_sweep(&short, &motion, 1., m.sweep, &|| false)
                .unwrap();
            actual.apply_delta(&delta).unwrap();
            let delta = reference_stock
                .preview_sweep(&long, &motion, 1., m.sweep, &|| false)
                .unwrap();
            reference_stock.apply_delta(&delta).unwrap();
            motions += 1;
        }
        assert!(motions > 20);
        assert!(
            reference
                .next_motion_with_stock(&reference_stock, &|| false)
                .unwrap()
                .is_none()
        );
        plan.finish_report(&|| false).unwrap();
        assert_eq!(actual.cells(), reference_stock.cells());
        assert!(actual.volume().removed_lower_mm3 > original.volume().removed_lower_mm3);
    }
}

#[test]
fn cad_stock_face_roundoff_does_not_change_target_or_admit_real_overhang() {
    let mut base = case(false);
    base.tools[0].cutter.cutting_length_mm = 20.;
    let prepare = |m: &StreamManifest| {
        OperationPlan::prepare(
            m.operations[0].clone(),
            &m.tools[0],
            m.stock.height().unwrap(),
            m.initial_position_mm,
            1,
            &|| false,
        )
    };
    assert!(prepare(&base).is_ok());
    let mut coincident = base.clone();
    if let Operation::Parallel(op) = &mut coincident.operations[0] {
        for p in std::sync::Arc::make_mut(&mut op.surface.triangles)
            .iter_mut()
            .flatten()
        {
            if p[0] == coincident.stock.height().unwrap().min_mm[0] {
                p[0] -= 4. * f64::EPSILON;
            }
        }
    }
    assert!(prepare(&coincident).is_ok());
    if let Operation::Parallel(op) = &mut coincident.operations[0] {
        for p in std::sync::Arc::make_mut(&mut op.surface.triangles)
            .iter_mut()
            .flatten()
        {
            p[0] -= 0.001;
        }
    }
    assert!(prepare(&coincident).is_err());
}
