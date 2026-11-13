use camlib::{fixtures, motion::*, planar::*, pocket::*, stock::*, tool::*};
use std::{collections::BTreeSet, sync::atomic::AtomicBool};

fn rectangle(x0: f64, y0: f64, x1: f64, y1: f64) -> Boundary {
    Boundary {
        vertices: vec![[x0, y0, 0.], [x1, y0, 0.], [x1, y1, 0.], [x0, y1, 0.]],
    }
}
fn region() -> PlanarRegion {
    PlanarRegion {
        version: 1,
        units: "mm".into(),
        name: "pocket".into(),
        boundary_error_mm: 0.,
        source: serde_json::Value::Null,
        components: vec![Component {
            outer: rectangle(0., 0., 20., 16.),
            holes: vec![rectangle(7., 5., 13., 11.)],
        }],
    }
}
fn tool() -> ToolAssembly {
    ToolAssembly {
        id: "t1".into(),
        cutter: Cutter {
            id: "c1".into(),
            name: "4mm".into(),
            shape: CutterShape::Flat,
            diameter_mm: 4.,
            cutting_length_mm: 20.,
            overall_length_mm: 50.,
            shank_diameter_mm: 4.,
            flutes: 2,
            center_cutting: true,
            material: "carbide".into(),
            neck: None,
        },
        stickout_mm: 25.,
        gauge_length_mm: 50.,
        holders: vec![HolderSection {
            z_min_mm: 25.,
            z_max_mm: 50.,
            radius_min_mm: 5.,
            radius_max_mm: 5.,
        }],
    }
}
fn stock() -> StockSpec {
    StockSpec {
        version: 1,
        units: "mm".into(),
        min_mm: [-2., -2., 0.],
        max_mm: [22., 18., 10.],
        cell_size_mm: 0.25,
        max_cells: 1_048_576,
    }
}
fn operation(r: PlanarRegion) -> PocketOperation {
    PocketOperation {
        max_stay_down_mm: None,
        limits: None,
        id: "pocket-1".into(),
        tool: "t1".into(),
        region: r,
        top_z_mm: 10.,
        floor_z_mm: 3.,
        stepdown_mm: 3.,
        stepover_mm: 1.5,
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
    }
}
fn plan(op: PocketOperation) -> PocketPlan {
    PocketPlan::prepare(
        op,
        &tool(),
        &stock(),
        [0., 0., 12.],
        1,
        &AtomicBool::new(false),
    )
    .unwrap()
}
fn drain(p: &mut PocketPlan) -> Vec<Motion> {
    let mut result = Vec::new();
    while let Some(m) = p.next_motion(&AtomicBool::new(false)).unwrap() {
        result.push(m);
    }
    result
}

#[test]
fn pocket_boundary_direction_reverses_exact_arcs_without_changing_raster_or_length() {
    let mut paths = Vec::new();
    for clockwise in [false, true] {
        let mut op = operation(PlanarRegion::circle("round", [10., 8.], 6.).unwrap());
        op.boundary_clockwise = Some(clockwise);
        let mut p = plan(op);
        let motions = drain(&mut p);
        let arcs: Vec<_> = motions
            .iter()
            .filter_map(|m| match m.curve {
                Curve::Arc { sweep_rad, .. } => Some(sweep_rad),
                _ => None,
            })
            .collect();
        assert!(!arcs.is_empty());
        assert!(arcs.iter().all(|s| (*s < 0.) == clockwise));
        let length: f64 = motions
            .iter()
            .filter(|m| m.kind == MoveKind::Cut)
            .map(|m| m.curve.prepare(1e-5).unwrap().length_mm())
            .sum();
        let raster: Vec<_> = motions
            .iter()
            .filter(|m| m.kind == MoveKind::Cut && matches!(m.curve, Curve::Line { .. }))
            .map(|m| m.curve.clone())
            .collect();
        paths.push((length, raster));
    }
    assert!((paths[0].0 - paths[1].0).abs() < 1e-8);
    assert_eq!(paths[0].1.len(), paths[1].1.len());
    for (a, b) in paths[0].1.iter().zip(&paths[1].1) {
        for (p, q) in [(a.start(), b.start()), (a.end(), b.end())] {
            for axis in 0..3 {
                assert!((p[axis] - q[axis]).abs() < 1e-8);
            }
        }
    }
}

#[test]
fn small_tool_rest_path_uses_large_tool_stock_and_omits_cleared_strokes() {
    let cancel = AtomicBool::new(false);
    let large = tool();
    let mut material = Stock::new(stock()).unwrap();
    let mut rough = plan(operation(region()));
    let mut next = 1;
    let mut position = [0., 0., 12.];
    while let Some(m) = rough.next_motion(&cancel).unwrap() {
        let d = material
            .preview_sweep(
                &large.prepare().unwrap(),
                &m,
                1.,
                SweepOptions::default(),
                &cancel,
            )
            .unwrap();
        material.apply_delta(&d).unwrap();
        next = m.id + 1;
        position = m.curve.end();
    }
    let after_large = material.volume();
    let mut small = large.clone();
    small.id = "small".into();
    small.cutter.diameter_mm = 2.;
    small.cutter.shank_diameter_mm = 2.;
    let prepared = small.prepare().unwrap();
    let mut op = operation(region());
    op.id = "rest".into();
    op.tool = small.id.clone();
    op.stepover_mm = 0.75;
    let mut normal =
        PocketPlan::prepare(op.clone(), &small, &stock(), position, next, &cancel).unwrap();
    let normal_moves = drain(&mut normal);
    let normal_length: f64 = normal_moves
        .iter()
        .filter(|m| m.kind == MoveKind::Cut)
        .map(|m| m.curve.prepare(1e-5).unwrap().length_mm())
        .sum();
    op.rest_tolerance_mm = Some(0.01);
    let mut rest = PocketPlan::prepare(op, &small, &stock(), position, next, &cancel).unwrap();
    assert!(rest.next_motion(&cancel).is_err());
    let mut cut_length = 0.;
    let from_generation = material.generation();
    while let Some(m) = rest.next_motion_with_stock(&material, &cancel).unwrap() {
        if m.kind == MoveKind::Cut {
            cut_length += m.curve.prepare(1e-5).unwrap().length_mm();
        }
        let d = material
            .preview_sweep(&prepared, &m, 1., SweepOptions::default(), &cancel)
            .unwrap();
        material.apply_delta(&d).unwrap();
    }
    assert!(
        rest.report().rest_skipped_strokes > 0,
        "{:?}",
        rest.report()
    );
    assert!(rest.report().rest_merged_pieces > 0);
    assert_eq!(
        rest.report().rest_candidates,
        (rest.report().xy_strokes * rest.report().depth_passes) as u64
    );
    assert!(
        cut_length < normal_length * 0.75,
        "rest{cut_length} baseline{normal_length}"
    );
    assert!(material.volume().removed_lower_mm3 > after_large.removed_lower_mm3);
    assert_eq!(rest.report().rest_from_generation, Some(from_generation));
    assert_eq!(
        rest.report().rest_through_generation,
        Some(material.generation())
    );
    // Independent source geometry: the protected square island stays full.
    for (i, h) in material.cells().iter().enumerate() {
        let [x, y] = material.cell_center(i).unwrap();
        if x > 7.5 && x < 12.5 && y > 5.5 && y < 10.5 {
            assert_eq!(h.lower_mm, 10.);
        }
        // Deep reachable interior away from walls/island must be cleared.
        if x > 3. && x < 17. && y > 2. && y < 3.5 {
            assert!(h.upper_mm <= 3.011, "{x},{y}: {h:?}");
        }
    }
}

#[test]
fn rest_coalesced_arcs_keep_source_groups_sense_and_continuous_island_separation() {
    let cancel = AtomicBool::new(false);
    let mut r = region();
    r.components[0].holes = vec![
        PlanarRegion::circle("island", [10., 8.], 3.)
            .unwrap()
            .components
            .remove(0)
            .outer,
    ];
    let base = operation(r);
    let baseline = drain(&mut plan(base.clone()));
    let largest = baseline
        .iter()
        .filter_map(|m| {
            if let Curve::Arc { sweep_rad, .. } = m.curve {
                Some(sweep_rad.abs())
            } else {
                None
            }
        })
        .fold(0., f64::max);
    let mut op = base;
    op.rest_tolerance_mm = Some(0.01);
    let mut p = plan(op);
    let mut material = Stock::new(stock()).unwrap();
    let prepared = tool().prepare().unwrap();
    let mut largest_merged = 0f64;
    while let Some(m) = p.next_motion_with_stock(&material, &cancel).unwrap() {
        match &m.curve {
            Curve::Arc {
                start,
                center,
                sweep_rad,
                ..
            } => {
                assert!(sweep_rad.abs() <= largest + 1e-12);
                assert!(baseline.iter().any(|b| matches!(&b.curve,Curve::Arc {center:c,sweep_rad:s,..} if c==center && s.signum()==sweep_rad.signum())));
                let radius = (start[0] - 10.).hypot(start[1] - 8.);
                assert!(
                    radius - 2. - 1e-5 >= 3.,
                    "continuous circle clearance {m:?}"
                );
                largest_merged = largest_merged.max(sweep_rad.abs());
            }
            Curve::Line { start, end } if m.kind != MoveKind::Rapid => {
                let dx = end[0] - start[0];
                let dy = end[1] - start[1];
                let len2 = dx * dx + dy * dy;
                let t = if len2 == 0. {
                    0.
                } else {
                    ((10. - start[0]) * dx + (8. - start[1]) * dy) / len2
                }
                .clamp(0., 1.);
                assert!(
                    (start[0] + t * dx - 10.).hypot(start[1] + t * dy - 8.) >= 5. - 1e-8,
                    "continuous line crosses island: {m:?}"
                );
            }
            _ => {}
        }
        let d = material
            .preview_sweep(&prepared, &m, 1., SweepOptions::default(), &cancel)
            .unwrap();
        material.apply_delta(&d).unwrap();
    }
    assert!(p.report().rest_merged_pieces > 0);
    assert!(largest_merged > 0.75, "arcs stayed fragmented");
}

#[test]
fn cancelled_rest_lookahead_does_not_advance_the_next_motion() {
    use std::cell::Cell;
    struct CancelAfter(Cell<usize>);
    impl CancelSignal for CancelAfter {
        fn is_cancelled(&self) -> bool {
            let n = self.0.get() + 1;
            self.0.set(n);
            n >= 12
        }
    }
    let mut op = operation(region());
    op.rest_tolerance_mm = Some(0.01);
    let mut candidate = plan(op.clone());
    let mut expected = plan(op);
    let material = Stock::new(stock()).unwrap();
    assert!(
        candidate
            .next_motion_with_stock(&material, &CancelAfter(Cell::new(0)))
            .unwrap_err()
            .contains("cancel")
    );
    assert!(
        candidate.report().rest_candidates > 1,
        "did not cancel in lookahead"
    );
    assert_eq!(
        candidate
            .next_motion_with_stock(&material, &AtomicBool::new(false))
            .unwrap(),
        expected
            .next_motion_with_stock(&material, &AtomicBool::new(false))
            .unwrap()
    );
}

#[test]
fn rest_long_links_need_known_cleared_stock_and_save_nominal_feed_time() {
    let cancel = AtomicBool::new(false);
    let mut spec = stock();
    spec.cell_size_mm = 0.1;
    let mut material = Stock::new(spec.clone()).unwrap();
    let mut snap = material.snapshot();
    for (i, h) in snap.cells.iter_mut().enumerate() {
        let [x, y] = material.cell_center(i).unwrap();
        let patch =
            ((3.7..4.1).contains(&x) || (16.3..16.7).contains(&x)) && (3.7..4.1).contains(&y);
        *h = HeightBounds {
            lower_mm: if patch { 10. } else { 3. },
            upper_mm: if patch { 10. } else { 3. },
        };
    }
    material = Stock::from_snapshot(snap).unwrap();
    let mut r = region();
    r.components[0].holes.clear();
    let mut op = operation(r);
    op.rest_tolerance_mm = Some(0.01);
    let mut p = PocketPlan::prepare(op.clone(), &tool(), &spec, [0., 0., 12.], 1, &cancel).unwrap();
    let prepared = tool().prepare().unwrap();
    let mut long = 0;
    while let Some(m) = p.next_motion_with_stock(&material, &cancel).unwrap() {
        let length = m.curve.prepare(op.tolerance_mm).unwrap().length_mm();
        if m.kind == MoveKind::Link && length > 4. * op.stepover_mm {
            long += 1;
            let height = op.clearance_z_mm - m.curve.start()[2];
            assert!(length / op.feed_mm_min <= height / op.feed_mm_min + height / op.plunge_mm_min);
            let mut work = 1_000_000;
            assert!(
                !material
                    .may_remove_material(
                        &m,
                        2.,
                        op.rest_tolerance_mm.unwrap(),
                        op.tolerance_mm,
                        &mut work,
                        &cancel
                    )
                    .unwrap()
            );
        }
        let delta = material
            .preview_sweep(&prepared, &m, 1., SweepOptions::default(), &cancel)
            .unwrap();
        material.apply_delta(&delta).unwrap();
    }
    assert!(long > 0, "no cleared-space links generated");
    assert_eq!(p.report().rest_long_links, long);
}

#[test]
fn rest_does_not_advance_on_partial_foreign_or_wrong_tool_commits() {
    let mut op = operation(region());
    op.rest_tolerance_mm = Some(0.01);
    let mut p = plan(op);
    let mut material = Stock::new(stock()).unwrap();
    let mut wrong_spec = stock();
    wrong_spec.cell_size_mm *= 2.;
    assert!(
        p.next_motion_with_stock(&Stock::new(wrong_spec).unwrap(), &|| false)
            .is_err()
    );
    let assembly = tool();
    let prepared = assembly.prepare().unwrap();
    let motion = loop {
        let motion = p
            .next_motion_with_stock(&material, &|| false)
            .unwrap()
            .unwrap();
        if motion.kind == MoveKind::Cut {
            break motion;
        }
        let delta = material
            .preview_sweep(&prepared, &motion, 1., SweepOptions::default(), &|| false)
            .unwrap();
        material.apply_delta(&delta).unwrap();
    };
    let sweep = |tool: &PreparedAssembly, motion: &Motion, fraction| {
        material
            .preview_sweep(tool, motion, fraction, SweepOptions::default(), &|| false)
            .unwrap()
    };
    let full = sweep(&prepared, &motion, 1.);
    let partial = sweep(&prepared, &motion, 0.5);
    let mut wrong = motion.clone();
    wrong.id += 10;
    let wrong_motion = sweep(&prepared, &wrong, 1.);
    let mut other_tool = assembly.clone();
    other_tool.cutter.diameter_mm = 2.;
    let wrong_tool = sweep(&other_tool.prepare().unwrap(), &motion, 1.);
    for delta in [partial, wrong_motion, wrong_tool] {
        let mut invalid = material.clone();
        invalid.apply_delta(&delta).unwrap();
        assert!(
            p.next_motion_with_stock(&invalid, &|| false).is_err(),
            "rest advanced past an uncommitted full motion"
        );
    }
    let mut valid = material.clone();
    valid.apply_delta(&full).unwrap();
    let restored = Stock::from_snapshot(valid.snapshot()).unwrap();
    assert!(p.next_motion_with_stock(&restored, &|| false).is_err());
    material.apply_delta(&full).unwrap();
    let next = p
        .next_motion_with_stock(&material, &|| false)
        .unwrap()
        .unwrap();
    assert_eq!(next.id, motion.id + 1);
    assert_eq!(next.curve.start(), motion.curve.end());
}

#[test]
fn rest_requires_fresh_stock_and_conservative_queries_retain_unknown_material() {
    let cancel = AtomicBool::new(false);
    let mut op = operation(region());
    op.rest_tolerance_mm = Some(0.01);
    let mut p = plan(op);
    let mut material = Stock::new(stock()).unwrap();
    let m = p
        .next_motion_with_stock(&material, &cancel)
        .unwrap()
        .unwrap();
    assert!(
        p.next_motion_with_stock(&material, &cancel)
            .unwrap_err()
            .contains("commit")
    );
    let delta = material
        .preview_sweep(
            &tool().prepare().unwrap(),
            &m,
            1.,
            SweepOptions::default(),
            &cancel,
        )
        .unwrap();
    material.apply_delta(&delta).unwrap();
    assert!(
        p.next_motion_with_stock(&material, &cancel)
            .unwrap()
            .is_some()
    );
    let probe = Motion {
        id: 99,
        operation: "rest-query".into(),
        tool: "t1".into(),
        kind: MoveKind::Cut,
        curve: Curve::Line {
            start: [2., 2., 10.],
            end: [3., 2., 10.],
        },
        feed_mm_min: Some(100.),
    };
    let mut budget = 0;
    assert!(
        material
            .may_remove_material(&probe, 2., 0.01, 1e-5, &mut budget, &cancel)
            .unwrap()
    );
    let mut budget = 1000;
    assert!(
        !material
            .may_remove_material(&probe, 2., 0.01, 1e-5, &mut budget, &cancel)
            .unwrap()
    );
    let low = Motion {
        curve: Curve::Line {
            start: [2., 2., 3.],
            end: [3., 2., 3.],
        },
        ..probe
    };
    assert!(
        material
            .may_remove_material(&low, 2., 0.01, 1e-5, &mut budget, &cancel)
            .unwrap()
    );
    assert!(
        material
            .may_remove_material(&low, 2., 0.01, 1e-5, &mut budget, &AtomicBool::new(true))
            .is_err()
    );
}

#[test]
fn full_depth_program_has_safe_rapids_attribution_and_independent_stock_volume() {
    let op = operation(region());
    let mut p = plan(op.clone());
    let mut events = p.prefix_events();
    assert_eq!(p.report().depth_passes, 3);
    let exact_residual = 4. * (4. - std::f64::consts::PI);
    assert_eq!(
        p.report().geometric_residual_status,
        ResidualStatus::Pending
    );
    let moves = drain(&mut p);
    p.finish_report(&AtomicBool::new(false)).unwrap();
    assert!(
        (p.report().geometric_residual_area_mm2.unwrap() - exact_residual).abs() < 0.02,
        "{:?}",
        p.report()
    );
    let mut depths = BTreeSet::new();
    let original = camlib::geometry::Region::prepare(&camlib::Fixture {
        version: 1,
        units: "mm".into(),
        name: "oracle".into(),
        outer: vec![[0., 0.], [20., 0.], [20., 16.], [0., 16.]],
        holes: vec![vec![[7., 5.], [13., 5.], [13., 11.], [7., 11.]]],
        boundary_error_mm: 0.,
        source: serde_json::Value::Null,
        checks: vec![],
    })
    .unwrap();
    let mut material = Stock::new(stock()).unwrap();
    let prepared = tool().prepare().unwrap();
    for m in &moves {
        assert_eq!(m.operation, "pocket-1");
        assert_eq!(m.tool, "t1");
        let c = m.curve.prepare(op.tolerance_mm).unwrap();
        if m.kind == MoveKind::Rapid {
            assert!(c.bounds().min[2] >= 12.);
        } else if m.kind != MoveKind::Retract {
            // Independent dense oracle regression, not a claim of sweep proof.
            for i in 0..=40 {
                let q = c.position(i as f64 / 40.).unwrap();
                assert!(
                    original.clearance([q[0], q[1]]).unwrap() >= 2. - 1e-8,
                    "{m:?} at {q:?}"
                );
            }
        }
        if m.kind == MoveKind::Cut {
            depths.insert(m.curve.end()[2] as i32);
        }
        let delta = material
            .preview_sweep(
                &prepared,
                m,
                1.,
                SweepOptions::default(),
                &AtomicBool::new(false),
            )
            .unwrap();
        material.apply_delta(&delta).unwrap();
        events.push(Event::Motion { motion: m.clone() });
    }
    assert_eq!(depths, BTreeSet::from([3, 4, 7]));
    assert_eq!(p.position()[2], 12.);
    events.extend(p.suffix_events());
    let program = Program {
        version: 1,
        id: "test".into(),
        revision: "1".into(),
        units: "mm".into(),
        tolerance_mm: op.tolerance_mm,
        frame: WorkFrame::identity(),
        initial_position_mm: [0., 0., 12.],
        tools: vec![tool()],
        events,
    };
    assert_eq!(program.validate(None).unwrap().motions, moves.len());
    let expected = (320. - 36. - exact_residual) * 7.;
    let v = material.volume();
    assert!(
        v.removed_lower_mm3 <= expected && v.removed_upper_mm3 >= expected,
        "{v:?} vs {expected}"
    );
    assert!(expected - v.removed_lower_mm3 < 250., "{v:?}");
    let (nx, ny) = (96, 80);
    for iy in 0..ny {
        for ix in 0..nx {
            let center = [
                -2. + (ix as f64 + 0.5) * 0.25,
                -2. + (iy as f64 + 0.5) * 0.25,
            ];
            let d = original.clearance(center).unwrap();
            let h = material.cells()[iy * nx + ix];
            if d > 2.5 {
                assert!(
                    h.upper_mm <= 3.00002,
                    "uncleared interior {center:?}: {h:?}"
                );
            }
            if d < -0.25 {
                assert_eq!(h.lower_mm, 10., "protected material {center:?}: {h:?}");
            }
        }
    }
}

#[test]
fn analytic_round_island_keeps_arcs_and_cannot_link_across_the_island() {
    let mut r = region();
    r.components[0].holes = vec![
        PlanarRegion::circle("island", [10., 8.], 3.)
            .unwrap()
            .components
            .remove(0)
            .outer,
    ];
    let mut p = plan(operation(r));
    let moves = drain(&mut p);
    assert!(moves.iter().any(|m| matches!(m.curve, Curve::Arc { .. })));
    let mut links = 0;
    for m in &moves {
        if matches!(m.kind, MoveKind::Link | MoveKind::Cut | MoveKind::Plunge) {
            if m.kind == MoveKind::Link {
                links += 1;
            }
            for i in 0..=80 {
                let q = m
                    .curve
                    .prepare(1e-5)
                    .unwrap()
                    .position(i as f64 / 80.)
                    .unwrap();
                assert!((q[0] - 10.).hypot(q[1] - 8.) >= 5. - 1e-8, "{m:?}: {q:?}");
            }
        }
    }
    assert!(links > 0);
    assert!(moves.iter().filter(|m| m.kind == MoveKind::Retract).count() > 3);
}

#[test]
fn circular_outer_extrema_tangent_rows_and_reversed_orientation_clear() {
    let mut r = PlanarRegion::circle("round", [10., 8.], 7.).unwrap();
    // Rotate the two semicircle vertices so scan bounds require true arc extrema.
    r.components[0].outer.vertices = vec![[10., 1., -1.], [10., 15., -1.]];
    let mut p = plan(operation(r));
    let moves = drain(&mut p);
    let cuts: Vec<_> = moves.iter().filter(|m| m.kind == MoveKind::Cut).collect();
    assert!(cuts.iter().any(|m| m.curve.start()[1] < 4.));
    assert!(cuts.iter().any(|m| m.curve.start()[1] > 12.));
    for m in cuts {
        for i in 0..=80 {
            let q = m
                .curve
                .prepare(1e-5)
                .unwrap()
                .position(i as f64 / 80.)
                .unwrap();
            assert!((q[0] - 10.).hypot(q[1] - 8.) <= 5. + 1e-8);
        }
    }
    // The island's lower tangent is exactly a raster row. Keep its double root.
    let mut r = region();
    r.components[0].holes = vec![
        PlanarRegion::circle("i", [10., 7.5], 2.)
            .unwrap()
            .components
            .remove(0)
            .outer,
    ];
    let mut op = operation(r);
    op.stepover_mm = 1.;
    let mut p = plan(op);
    assert!(!drain(&mut p).is_empty());
}

#[test]
fn allowances_tool_changes_split_channels_and_empty_regions_are_explicit() {
    let mut op = operation(region());
    op.radial_allowance_mm = 0.3;
    op.bottom_allowance_mm = 0.4;
    let mut p = plan(op);
    assert_eq!(p.report().target_z_mm, 3.4);
    for m in drain(&mut p) {
        assert!(m.curve.prepare(1e-5).unwrap().bounds().min[2] >= 3.4);
    }
    let r = PlanarRegion::from_fixture(&fixtures::narrow_channel());
    let mut op = operation(r.clone());
    op.stepover_mm = 1.;
    let mut t = tool();
    t.cutter.diameter_mm = 4.2;
    t.cutter.shank_diameter_mm = 4.2;
    let mut p = PocketPlan::prepare(
        op.clone(),
        &t,
        &stock(),
        [0., 0., 12.],
        1,
        &AtomicBool::new(false),
    )
    .unwrap();
    assert_eq!(p.report().center_components, 2);
    assert!(!drain(&mut p).is_empty());
    p.finish_report(&AtomicBool::new(false)).unwrap();
    assert_ne!(
        p.report().geometric_residual_status,
        ResidualStatus::Pending
    );
    assert!(
        p.report()
            .geometric_residual_area_mm2
            .is_none_or(|v| v > 0.)
    );
    op.radial_allowance_mm = 100.;
    // As a `Pocket` operation this is refused (#250): the cutter fits
    // nowhere in the region, so the operation can do nothing it was asked
    // to. The planner itself still plans it as an explicitly empty plan
    // (below), which is what a roughing layer needs: there the cutter
    // legitimately fits nowhere at some depths.
    let refused = camlib::operation::OperationPlan::prepare(
        Operation::Pocket(op.clone()),
        &tool(),
        &stock(),
        [0., 0., 12.],
        1,
        &AtomicBool::new(false),
    )
    .map(|_| ())
    .unwrap_err();
    assert!(
        refused.contains("pocket pocket-1: the cutter fits nowhere in the region"),
        "{refused}"
    );
    let mut p = plan(op);
    assert_eq!(p.report().center_components, 0);
    assert!(drain(&mut p).is_empty());
    p.finish_report(&AtomicBool::new(false)).unwrap();
    assert!(
        p.report()
            .geometric_residual_area_mm2
            .is_none_or(|v| v >= 0.)
    );
    assert!(drain(&mut p).is_empty());
    let mut p = plan(operation(r));
    assert_eq!(p.report().center_components, 2);
    assert!(!drain(&mut p).is_empty());
}

#[test]
fn invalid_reach_parameters_unsupported_tools_budgets_and_cancellation_reject() {
    let base = operation(region());
    for bad in [f64::NAN, -1., 0., 2.1] {
        let mut op = base.clone();
        op.stepover_mm = bad;
        assert!(
            PocketPlan::prepare(
                op,
                &tool(),
                &stock(),
                [0., 0., 12.],
                1,
                &AtomicBool::new(false)
            )
            .is_err()
        );
    }
    let mut t = tool();
    t.cutter.center_cutting = false;
    assert!(
        PocketPlan::prepare(
            base.clone(),
            &t,
            &stock(),
            [0., 0., 12.],
            1,
            &AtomicBool::new(false)
        )
        .is_err()
    );
    t = tool();
    t.cutter.cutting_length_mm = 4.;
    assert!(
        PocketPlan::prepare(
            base.clone(),
            &t,
            &stock(),
            [0., 0., 12.],
            1,
            &AtomicBool::new(false)
        )
        .is_err()
    );
    assert!(
        PocketPlan::prepare(
            base.clone(),
            &tool(),
            &stock(),
            [0., 0., 11.],
            1,
            &AtomicBool::new(false)
        )
        .is_err()
    );
    assert!(
        PocketPlan::prepare(
            base.clone(),
            &tool(),
            &stock(),
            [0., 0., 12.],
            u64::MAX,
            &AtomicBool::new(false)
        )
        .is_err()
    );
    assert!(
        PocketPlan::prepare(
            base.clone(),
            &tool(),
            &stock(),
            [0., 0., 12.],
            1,
            &AtomicBool::new(true)
        )
        .is_err()
    );
    let mut op = base;
    op.region.components[0].outer = rectangle(0., 0., 10000., 10000.);
    op.region.components[0].holes.clear();
    op.stepover_mm = 0.00016;
    // 62 million raster rows: refused early by the memory guard (projected
    // from the first rows), not by a fixed cap, and without allocating it.
    let error = camlib::resources::with_memory_budget(256 << 20, || {
        PocketPlan::prepare(
            op,
            &tool(),
            &stock(),
            [0., 0., 12.],
            1,
            &AtomicBool::new(false),
        )
        .err()
        .unwrap()
    });
    assert!(error.contains("pocket raster strokes needs"), "{error}");
    let mut p = plan(operation(region()));
    let pos = p.position();
    assert!(p.next_motion(&AtomicBool::new(true)).is_err());
    assert_eq!(p.position(), pos);
}

#[test]
fn exact_collinear_subdivisions_produce_identical_motion_and_report_work_is_deferred() {
    let mut sparse = plan(operation(PlanarRegion::from_fixture(
        &fixtures::rectangle_island(1),
    )));
    let mut dense = plan(operation(PlanarRegion::from_fixture(
        &fixtures::rectangle_island(128),
    )));
    assert_eq!(
        dense.report().geometric_residual_status,
        ResidualStatus::Pending
    );
    assert!(dense.finish_report(&AtomicBool::new(false)).is_err());
    assert_eq!(drain(&mut sparse), drain(&mut dense));
    assert!(
        dense
            .report()
            .diagnostics
            .iter()
            .any(|d| d.contains("1024 -> 8"))
    );
    assert!(dense.finish_report(&AtomicBool::new(true)).is_err());
    assert_eq!(
        dense.report().geometric_residual_status,
        ResidualStatus::Pending
    );
    dense.finish_report(&AtomicBool::new(false)).unwrap();
    assert_ne!(
        dense.report().geometric_residual_status,
        ResidualStatus::Pending
    );
}

/// Selectable retract/feed heights (Fusion-style planes): the cutting path and
/// removed stock are unchanged; only the non-cutting heights move, within
/// stock top < feed <= retract <= clearance.
#[test]
fn retract_and_feed_heights_change_only_noncutting_moves_within_validated_planes() {
    let base = operation(region());
    let (top, clearance) = (base.top_z_mm, base.clearance_z_mm);
    for (retract, feed) in [
        (Some(top + 0.5), None),
        (None, Some(top + 0.5)),
        (Some(clearance + 0.1), None),
        (Some(f64::NAN), None),
        (Some(11.), Some(11.5)),
    ] {
        let mut bad = base.clone();
        bad.retract_z_mm = retract;
        bad.feed_z_mm = feed;
        if retract == Some(top + 0.5) && feed.is_none() {
            // feed defaults to retract: 10.5 is valid.
            assert!(
                PocketPlan::prepare(
                    bad,
                    &tool(),
                    &stock(),
                    [0., 0., 12.],
                    1,
                    &AtomicBool::new(false)
                )
                .is_ok()
            );
            continue;
        }
        if feed == Some(top + 0.5) && retract.is_none() {
            assert!(
                PocketPlan::prepare(
                    bad,
                    &tool(),
                    &stock(),
                    [0., 0., 12.],
                    1,
                    &AtomicBool::new(false)
                )
                .is_ok()
            );
            continue;
        }
        assert!(
            PocketPlan::prepare(
                bad,
                &tool(),
                &stock(),
                [0., 0., 12.],
                1,
                &AtomicBool::new(false)
            )
            .is_err(),
            "accepted retract {retract:?} feed {feed:?}"
        );
    }
    for feed in [top, top - 1.] {
        let mut bad = base.clone();
        bad.feed_z_mm = Some(feed);
        assert!(
            PocketPlan::prepare(
                bad,
                &tool(),
                &stock(),
                [0., 0., 12.],
                1,
                &AtomicBool::new(false)
            )
            .is_err()
        );
    }

    let (retract, feed) = (11.5, 10.5);
    let mut op = base.clone();
    op.retract_z_mm = Some(retract);
    op.feed_z_mm = Some(feed);
    let default_moves = drain(&mut plan(base.clone()));
    let moves = drain(&mut plan(op));
    let cuts = |m: &[Motion]| {
        m.iter()
            .filter(|m| m.kind == MoveKind::Cut)
            .map(|m| m.curve.clone())
            .collect::<Vec<_>>()
    };
    assert_eq!(cuts(&default_moves), cuts(&moves));
    let mut relinks = 0;
    for m in &moves {
        let (a, b) = (m.curve.start(), m.curve.end());
        match m.kind {
            MoveKind::Rapid if a[0] != b[0] || a[1] != b[1] => {
                assert!(
                    a[2] >= retract && b[2] >= retract,
                    "XY rapid below retract height"
                );
                assert_eq!(a[2], b[2]);
                relinks += usize::from(a[2] == retract);
            }
            MoveKind::Rapid => assert!(a[2].min(b[2]) >= feed, "vertical rapid below feed height"),
            MoveKind::Plunge => assert_eq!(a[2], feed),
            MoveKind::Retract => assert_eq!(b[2], feed),
            _ => {}
        }
    }
    assert!(
        relinks > 0,
        "fixture must exercise travel at the retract height"
    );
    assert_eq!(moves.last().unwrap().curve.end()[2], clearance);
    // Identical material removal, less time at feed through air.
    let removal = |moves: &[Motion]| {
        let mut s = Stock::new(stock()).unwrap();
        let t = tool().prepare().unwrap();
        for m in moves {
            let d = s
                .preview_sweep(&t, m, 1., SweepOptions::default(), &AtomicBool::new(false))
                .unwrap();
            s.apply_delta(&d).unwrap();
        }
        s.cells().to_vec()
    };
    assert_eq!(removal(&default_moves), removal(&moves));
    let fed = |moves: &[Motion]| -> f64 {
        moves
            .iter()
            .filter(|m| m.kind != MoveKind::Rapid)
            .map(|m| m.curve.prepare(1e-5).unwrap().length_mm())
            .sum()
    };
    assert!(fed(&moves) < fed(&default_moves));
}

/// Regression for a real gouge: a pocket around a dense, slightly noisy
/// polygonal island (as produced by tessellated CAD or mesh slices). The
/// region offset used to lose the island, and the plain-plunge pocket then
/// machined the whole island to the floor. Every cutting move must keep the
/// cutter radius from the ORIGINAL island and pocket walls (dense independent
/// oracle on the planned curves).
#[test]
fn pocket_around_noisy_dense_island_never_enters_the_island() {
    let mut seed = 7u64;
    let mut rnd = || {
        seed = seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((seed >> 11) as f64) / ((1u64 << 53) as f64)
    };
    let (c, r, n, noise) = ([10., 8.], 3.5, 600usize, 4.3e-4);
    let mut island = Vec::new();
    for i in 0..n {
        let t = -(i as f64) / n as f64 * std::f64::consts::TAU + rnd() * 0.2 / n as f64;
        let k = r + (rnd() - 0.5) * 2. * noise;
        island.push([c[0] + k * t.cos(), c[1] + k * t.sin(), 0.]);
        if rnd() < 0.05 {
            let t2 = t - 1e-4 / r;
            island.push([c[0] + k * t2.cos(), c[1] + k * t2.sin(), 0.]);
        }
    }
    let mut pocket_region = region();
    pocket_region.components[0].holes = vec![Boundary {
        vertices: island.clone(),
    }];
    let moves = drain(&mut plan(operation(pocket_region)));
    let walls: Vec<([f64; 2], [f64; 2])> = island
        .iter()
        .zip(island.iter().cycle().skip(1))
        .map(|(a, b)| ([a[0], a[1]], [b[0], b[1]]))
        .chain(
            [[0., 0.], [20., 0.], [20., 16.], [0., 16.]]
                .iter()
                .zip([[20., 0.], [20., 16.], [0., 16.], [0., 0.]].iter())
                .map(|(a, b)| (*a, *b)),
        )
        .collect();
    let distance = |p: [f64; 2], (a, b): ([f64; 2], [f64; 2])| {
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let t = (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)).clamp(0., 1.);
        (p[0] - a[0] - t * dx).hypot(p[1] - a[1] - t * dy)
    };
    let mut cuts = 0;
    for m in moves.iter().filter(|m| m.kind == MoveKind::Cut) {
        cuts += 1;
        let prepared = m.curve.prepare(1e-5).unwrap();
        for i in 0..=200 {
            let q = prepared.position(i as f64 / 200.).unwrap();
            let nearest = walls
                .iter()
                .map(|&w| distance([q[0], q[1]], w))
                .fold(f64::INFINITY, f64::min);
            assert!(
                nearest >= 2. - 1e-6,
                "cutter centre {q:?} is {nearest} mm from a wall (radius 2)"
            );
        }
    }
    assert!(cuts > 10);
}
