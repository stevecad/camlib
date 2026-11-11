use camlib::{
    contour::*, motion::*, planar::*, pocket::Operation, stock::*, stream::StreamManifest,
};
use std::{f64::consts::TAU, sync::atomic::AtomicBool};
fn fixture() -> StreamManifest {
    serde_json::from_str(include_str!("../fixtures/operations/contour.json")).unwrap()
}
fn operation() -> ContourOperation {
    match fixture().operations.remove(0) {
        Operation::Contour(p) => p,
        _ => panic!("fixture"),
    }
}
fn plan(op: ContourOperation) -> Result<ContourPlan, String> {
    let m = fixture();
    ContourPlan::prepare(
        op,
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &AtomicBool::new(false),
    )
}
fn commit(stock: &mut Stock, m: &Motion) {
    let t = fixture().tools[0].prepare().unwrap();
    let d = stock
        .preview_sweep(&t, m, 1., SweepOptions::default(), &AtomicBool::new(false))
        .unwrap();
    stock.apply_delta(&d).unwrap();
}

#[test]
fn contour_cad_annulus_keeps_protected_part_and_cuts_outer_and_hole_bands_continuously() {
    for reverse in [false, true] {
        let mut op = operation();
        op.reverse = reverse;
        let mut p = plan(op.clone()).unwrap();
        let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
        let cancel = AtomicBool::new(false);
        let mut position = fixture().initial_position_mm;
        let mut arcs = 0;
        while let Some(m) = p.next_motion_with_stock(&stock, &cancel).unwrap() {
            assert_eq!(m.curve.start(), position);
            position = m.curve.end();
            if let Curve::Arc {
                start,
                center,
                sweep_rad,
                ..
            } = m.curve
            {
                assert!((center[0] - 20.).abs() < 1e-8 && (center[1] - 20.).abs() < 1e-8);
                let radius = (start[0] - 20.).hypot(start[1] - 20.);
                // Exact entire-circle clearance, independently of offset/scene checks.
                if radius > 8. {
                    assert!(radius - 2. > 8.);
                    assert_eq!(sweep_rad.is_sign_negative(), reverse);
                } else {
                    assert!(radius + 2. < 4.);
                    assert_eq!(sweep_rad.is_sign_negative(), !reverse);
                }
                assert!(sweep_rad.abs() <= TAU / 2. + 1e-10);
                arcs += 1;
            }
            assert!(m.curve.start()[2] >= 2. && m.curve.end()[2] >= 2.);
            commit(&mut stock, &m);
        }
        p.finish_report(&cancel).unwrap();
        assert_eq!(arcs, 16);
        assert_eq!(p.report().loops_completed, 8);
        assert!(p.report().preflight_nodes < 100);
        assert!(p.report().wall_completion.contains("not_certified"));
        for (i, h) in stock.cells().iter().enumerate() {
            let q = stock.cell_center(i).unwrap();
            let r = (q[0] - 20.).hypot(q[1] - 20.);
            if (4.5..7.5).contains(&r) || r > 12.5 {
                assert_eq!(h.lower_mm, 10., "protected {q:?}:{h:?}");
            }
            if r < 3.5 || (8.5..11.5).contains(&r) {
                assert!(h.upper_mm < 2.003, "uncut band {q:?}:{h:?}");
            }
            assert!(h.lower_mm >= 1.997);
        }
    }
}
#[test]
fn contour_rectangle_retains_corner_arcs_and_continuously_admits_tangent_leads() {
    let mut op = operation();
    op.boundaries = Boundaries::Outer;
    op.floor_z_mm = 8.;
    op.lead_in_mm = 1.;
    op.lead_out_mm = 1.;
    op.region = PlanarRegion {
        version: 1,
        units: "mm".into(),
        name: "rectangle".into(),
        boundary_error_mm: 0.,
        source: serde_json::Value::Null,
        components: vec![Component {
            outer: Boundary {
                vertices: vec![
                    [10., 10., 0.],
                    [20., 10., 0.],
                    [20., 20., 0.],
                    [10., 20., 0.],
                ],
            },
            holes: vec![],
        }],
    };
    // The default ring start can be on a corner, so the source checker (rather
    // than an endpoint test) decides whether the requested tangents are legal.
    let mut p = plan(op).unwrap();
    assert!(p.report().retained_analytic_arcs >= 4);
    let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
    let mut lead = 0;
    while let Some(m) = p
        .next_motion_with_stock(&stock, &AtomicBool::new(false))
        .unwrap()
    {
        if m.kind == MoveKind::Lead {
            lead += 1;
        }
        commit(&mut stock, &m);
    }
    assert_eq!(lead, 2);
    assert_eq!(p.report().loops_completed, 1);
    for (i, h) in stock.cells().iter().enumerate() {
        let q = stock.cell_center(i).unwrap();
        if q[0] > 10.5 && q[0] < 19.5 && q[1] > 10.5 && q[1] < 19.5 {
            assert_eq!(h.lower_mm, 10.);
        }
    }
}
#[test]
fn contour_allowances_partial_depth_and_direction_are_explicit() {
    let mut op = operation();
    op.radial_allowance_mm = 0.2;
    op.bottom_allowance_mm = 0.3;
    op.stepdown_mm = 3.;
    let mut p = plan(op).unwrap();
    assert_eq!(p.report().target_z_mm, 2.3);
    assert_eq!(p.report().depth_passes, 3);
    assert!((p.report().path_safety_margin_mm - 0.005).abs() < 1e-12);
    let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
    let mut lowest = f64::INFINITY;
    while let Some(m) = p
        .next_motion_with_stock(&stock, &AtomicBool::new(false))
        .unwrap()
    {
        lowest = lowest.min(m.curve.end()[2]);
        commit(&mut stock, &m);
    }
    assert_eq!(lowest, 2.3);
}
#[test]
fn contour_does_not_silently_erase_holes_or_admit_a_lead_through_retained_material() {
    let mut small = operation();
    small.region.components[0].holes = vec![
        PlanarRegion::circle("small", [20., 20.], 1.5)
            .unwrap()
            .components
            .remove(0)
            .outer,
    ];
    assert!(plan(small.clone()).unwrap_err_string().contains("topology"));
    small.boundaries = Boundaries::Outer;
    assert!(plan(small).is_ok());
    let mut bad = operation();
    bad.boundaries = Boundaries::Holes;
    bad.lead_in_mm = 2.;
    assert!(plan(bad).unwrap_err_string().contains("admission"));
}
// Keep error assertions readable without requiring Debug for a native plan.
trait ErrorString {
    fn unwrap_err_string(self) -> String;
}
impl ErrorString for Result<ContourPlan, String> {
    fn unwrap_err_string(self) -> String {
        match self {
            Err(e) => e,
            Ok(_) => panic!("unexpected accepted contour"),
        }
    }
}
#[test]
fn contour_cancel_stale_and_invalid_inputs_preserve_next_motion() {
    let cancel = AtomicBool::new(false);
    let mut p = plan(operation()).unwrap();
    let mut expected = plan(operation()).unwrap();
    let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
    assert!(p.finish_report(&cancel).is_err());
    for _ in 0..5 {
        let m = p.next_motion_with_stock(&stock, &cancel).unwrap().unwrap();
        assert_eq!(
            m,
            expected
                .next_motion_with_stock(&stock, &cancel)
                .unwrap()
                .unwrap()
        );
        assert!(
            p.next_motion_with_stock(&stock, &cancel)
                .unwrap_err()
                .contains("commit")
        );
        commit(&mut stock, &m);
        assert!(
            p.next_motion_with_stock(&stock, &AtomicBool::new(true))
                .unwrap_err()
                .contains("cancel")
        );
    }
    assert_eq!(
        p.next_motion_with_stock(&stock, &cancel).unwrap(),
        expected.next_motion_with_stock(&stock, &cancel).unwrap()
    );
    let mut bad = operation();
    bad.region.boundary_error_mm = 0.1;
    assert!(plan(bad).unwrap_err_string().contains("uncertainty"));
    let mut bad = operation();
    bad.stepdown_mm = 0.;
    assert!(plan(bad).is_err());
}
#[test]
fn contour_retract_and_feed_heights_link_between_passes_and_end_at_clearance() {
    let top = fixture().stock.height().unwrap().max_mm[2];
    let mut op = operation();
    op.stepdown_mm = 3.;
    let clearance = op.clearance_z_mm;
    let (retract, feed) = (top + (clearance - top) / 2., top + (clearance - top) / 4.);
    assert!(retract < clearance);
    op.retract_z_mm = Some(retract);
    op.feed_z_mm = Some(feed);
    let mut p = plan(op.clone()).unwrap();
    assert!(p.report().depth_passes > 1);
    let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
    let mut motions = vec![];
    while let Some(m) = p
        .next_motion_with_stock(&stock, &AtomicBool::new(false))
        .unwrap()
    {
        commit(&mut stock, &m);
        motions.push(m);
    }
    let (mut travels_at_retract, mut descents) = (0, 0);
    for m in &motions {
        let (a, b) = (m.curve.start(), m.curve.end());
        match m.kind {
            // Rapids never go below the feed height (which is above the stock).
            MoveKind::Rapid => {
                assert!(a[2] >= feed && b[2] >= feed, "{m:?}");
                if a[2] == b[2] && (a[0], a[1]) != (b[0], b[1]) {
                    assert!(b[2] == retract || b[2] == clearance, "{m:?}");
                    travels_at_retract += usize::from(b[2] == retract);
                }
                if b[2] == feed && a[2] > feed {
                    descents += 1;
                }
            }
            // Leaving a pass: feed rate up to the feed height only.
            MoveKind::Retract => assert_eq!(b[2], feed),
            MoveKind::Plunge => assert_eq!(a[2], feed),
            _ => {}
        }
    }
    assert!(travels_at_retract > 0, "passes link at the retract height");
    assert!(descents > 1);
    assert_eq!(
        motions.last().unwrap().curve.end()[2],
        clearance,
        "operation ends at clearance"
    );

    for (r, f, expected) in [
        (Some(retract), Some(top), "stock top"),
        (Some(clearance + 1.), None, "clearance"),
        (Some(feed), Some(retract), "not be above retract"),
    ] {
        let mut bad = op.clone();
        bad.retract_z_mm = r;
        bad.feed_z_mm = f;
        assert!(plan(bad).err().unwrap().contains(expected));
    }
}

#[test]
fn holding_tabs_leave_material_on_the_lower_passes_only() {
    let cancel = AtomicBool::new(false);
    for shape in [TabShape::Triangle, TabShape::Rectangle] {
        let mut op = operation();
        op.boundaries = Boundaries::Outer;
        op.tabs = Some(Tabs {
            width_mm: 2.,
            height_mm: 3.,
            count: Some(4),
            spacing_mm: None,
            positions_mm: None,
            position_tolerance_mm: None,
            shape,
        });
        let mut p = plan(op).unwrap();
        let spec = fixture().stock.height().unwrap().clone();
        let mut stock = Stock::new(spec.clone()).unwrap();
        let mut top_hits = 0;
        while let Some(m) = p.next_motion_with_stock(&stock, &cancel).unwrap() {
            commit(&mut stock, &m);
            if m.kind != MoveKind::Rapid && m.curve.end()[2] == 5. && m.curve.start()[2] < 5. {
                top_hits += 1;
            }
            // The tool never cuts below the floor or above the tab top
            // while lifted.
            assert!(m.curve.end()[2] >= 2. - 1e-9);
        }
        // Two tabbed passes (Z4, Z2), four tabs each.
        assert_eq!(top_hits, 8, "{shape:?}");
        // Around the part, between its edge (r 8) and the cut's outside
        // (r 12): four tabs stand to Z5, the rest is cut to the floor.
        let height = |x: f64, y: f64| {
            let i = ((x - spec.min_mm[0]) / spec.cell_size_mm) as usize;
            let j = ((y - spec.min_mm[1]) / spec.cell_size_mm) as usize;
            let n = ((spec.max_mm[0] - spec.min_mm[0]) / spec.cell_size_mm).ceil() as usize;
            stock.cells()[j * n + i]
        };
        let tab: Vec<bool> = (0..720)
            .map(|k| {
                let a = k as f64 * TAU / 720.;
                let h = height(20. + 10. * a.cos(), 20. + 10. * a.sin());
                // Never above the tab top; triangular ramps leave a slope.
                assert!(h.lower_mm <= 5. + 1e-9, "{h:?}");
                if shape == TabShape::Rectangle {
                    assert!(h.upper_mm > 4.5 || h.upper_mm < 2.5, "{h:?}");
                }
                h.upper_mm > 4.5
            })
            .collect();
        let groups = (0..720)
            .filter(|&k| tab[k] && !tab[(k + 719) % 720])
            .count();
        assert_eq!(groups, 4, "{shape:?}");
    }
    // Rectangular tabs move straight up: not with cutter compensation.
    let mut op = operation();
    op.boundaries = Boundaries::Outer;
    op.lead_in_mm = 2.;
    op.lead_out_mm = 2.;
    op.compensation = Compensation::Wear;
    op.tabs = Some(Tabs {
        width_mm: 2.,
        height_mm: 3.,
        count: Some(4),
        spacing_mm: None,
        positions_mm: None,
        position_tolerance_mm: None,
        shape: TabShape::Rectangle,
    });
    assert!(plan(op.clone()).err().unwrap().contains("triangular"));
    op.tabs.as_mut().unwrap().count = Some(40);
    op.tabs.as_mut().unwrap().shape = TabShape::Triangle;
    assert!(plan(op).err().unwrap().contains("do not fit"));
}

// --- Tabs at given positions (#172) ---------------------------------------
// The fixture part is an annulus around (20, 20): outer radius 8, hole radius
// 4, cut with a 4 mm tool from Z10 to the floor at Z2 in 2 mm steps. Tabs
// 2 mm wide and 3 mm high stand to Z5: the passes at Z4 and Z2 rise over them.
fn tabs_at(points: &[[f64; 2]], shape: TabShape) -> Tabs {
    Tabs {
        width_mm: 2.,
        height_mm: 3.,
        count: None,
        spacing_mm: None,
        positions_mm: Some(points.to_vec()),
        position_tolerance_mm: Some(0.01),
        shape,
    }
}
/// The stock's height bounds carry the sweep's chord error (0.002 mm in the
/// fixture): a standing tab or a cut floor is told within this.
const TIGHT: f64 = 0.01;
fn on_circle(radius: f64, degrees: f64) -> [f64; 2] {
    let a = degrees.to_radians();
    [20. + radius * a.cos(), 20. + radius * a.sin()]
}
/// The smallest angle (rad) between the direction of p from the part center
/// and any of the given directions (degrees).
fn off_angle(p: [f64; 2], degrees: &[f64]) -> f64 {
    let a = (p[1] - 20.).atan2(p[0] - 20.);
    degrees
        .iter()
        .map(|d| ((a - d.to_radians() + TAU / 2.).rem_euclid(TAU) - TAU / 2.).abs())
        .fold(f64::INFINITY, f64::min)
}
/// Runs the plan through the stock; returns the motions and the stock.
fn run(mut p: ContourPlan) -> (Vec<Motion>, Stock, ContourPlan) {
    let cancel = AtomicBool::new(false);
    let mut stock = Stock::new(fixture().stock.height().unwrap().clone()).unwrap();
    let mut motions = vec![];
    while let Some(m) = p.next_motion_with_stock(&stock, &cancel).unwrap() {
        commit(&mut stock, &m);
        motions.push(m);
    }
    (motions, stock, p)
}
fn cell(stock: &Stock, x: f64, y: f64) -> HeightBounds {
    let spec = stock.spec();
    let i = ((x - spec.min_mm[0]) / spec.cell_size_mm) as usize;
    let j = ((y - spec.min_mm[1]) / spec.cell_size_mm) as usize;
    let n = ((spec.max_mm[0] - spec.min_mm[0]) / spec.cell_size_mm).ceil() as usize;
    stock.cells()[j * n + i]
}

#[test]
fn tabs_at_given_positions_stand_there_and_nowhere_else() {
    // Unevenly placed, the first 0.004 mm off the boundary (inside the
    // 0.01 mm position tolerance).
    let angles = [40., 150., 290.];
    let points = [on_circle(8.004, 40.), on_circle(8., 150.), on_circle(8., 290.)];
    for shape in [TabShape::Triangle, TabShape::Rectangle] {
        let mut op = operation();
        op.boundaries = Boundaries::Outer;
        op.tabs = Some(tabs_at(&points, shape));
        let (motions, stock, _) = run(plan(op).unwrap());
        // From the motions: the tool rises to the tab top (Z5) once per tab
        // on each of the two lower passes, and travels level there over the
        // tab width plus the tool diameter (3 mm either side of the point,
        // measured on the tool-center circle), centered on each point.
        let mut rises = 0;
        let mut reach = [0f64; 3];
        for m in motions.iter().filter(|m| m.kind != MoveKind::Rapid) {
            let (a, b) = (m.curve.start(), m.curve.end());
            assert!(a[2] >= 2. - 1e-9 && b[2] >= 2. - 1e-9);
            rises += usize::from(b[2] == 5. && a[2] < 5.);
            let radius = (a[0] - 20.).hypot(a[1] - 20.);
            for p in [a, b] {
                let off = off_angle([p[0], p[1]], &angles) * radius;
                if a[2] == 5. && b[2] == 5. {
                    assert!(off <= 3. + 1e-6, "level at the tab top {off} mm from a tab");
                    let k = (0..3)
                        .find(|&k| off_angle([p[0], p[1]], &angles[k..=k]) * radius == off)
                        .unwrap();
                    reach[k] = reach[k].max(off);
                } else if p[2] < 5. - 1e-9 && shape == TabShape::Rectangle {
                    // Below the tab top the tool stays off the tabs.
                    assert!(off >= 3. - 1e-6, "{p:?} is {off} mm from a tab");
                }
            }
        }
        assert_eq!(rises, 6, "{shape:?}");
        assert!(reach.iter().all(|r| (r - 3.).abs() < 1e-6), "{reach:?}");
        // From the stock, around the middle of the cut (r 10): full height
        // (Z5) within the tab width of each point, the floor (Z2) everywhere
        // beyond the tab and its ramps. 0.5 mm either way covers the cells.
        let ramp = if shape == TabShape::Triangle { 3. } else { 0. };
        let (mut standing, mut cut) = (0, 0);
        for k in 0..1440 {
            let p = on_circle(10., k as f64 / 4.);
            let off = off_angle(p, &angles) * 10.;
            let h = cell(&stock, p[0], p[1]);
            assert!(h.upper_mm <= 5. + TIGHT, "{h:?}");
            if off < 1. - 0.5 {
                assert!(h.lower_mm >= 5. - TIGHT, "{shape:?} {k} {h:?}");
                standing += 1;
            } else if off > 1. + ramp + 0.5 {
                assert!(h.upper_mm <= 2. + TIGHT, "{shape:?} {k} {h:?}");
                cut += 1;
            }
        }
        assert!(standing >= 3 * 10 && cut > 700, "{standing} {cut}");
    }
}

#[test]
fn tabs_at_positions_go_to_the_loop_each_point_is_on() {
    // One point on the outer boundary and one on the hole.
    let mut op = operation();
    op.tabs = Some(tabs_at(&[on_circle(8., 150.), on_circle(4., 180.)], TabShape::Rectangle));
    let (motions, stock, p) = run(plan(op).unwrap());
    assert!(p.report().diagnostics.iter().all(|d| !d.contains("tab")));
    let level = |r: f64| {
        motions
            .iter()
            .filter(|m| m.kind != MoveKind::Rapid)
            .filter(|m| m.curve.start()[2] == 5. && m.curve.end()[2] == 5.)
            .filter(|m| ((m.curve.start()[0] - 20.).hypot(m.curve.start()[1] - 20.) - r).abs() < 0.1)
            .count()
    };
    assert!(level(10.) >= 2 && level(2.) >= 2);
    // The hole's tab stands at the point (the tool circles at r 2 there and
    // is 4 mm wide); the far side of the hole is cut to the floor.
    assert!(cell(&stock, 20. - 3.5, 20.).lower_mm >= 5. - TIGHT);
    assert!(cell(&stock, 20. + 3.5, 20.).upper_mm <= 2. + TIGHT);
    assert!(cell(&stock, 20., 20. + 3.5).upper_mm <= 2. + TIGHT);

    // Points on the outer boundary only: the hole gets no tab, and the
    // report says so.
    let mut op = operation();
    op.tabs = Some(tabs_at(&[on_circle(8., 150.)], TabShape::Rectangle));
    let (motions, stock, p) = run(plan(op).unwrap());
    let said: Vec<&String> = p
        .report()
        .diagnostics
        .iter()
        .filter(|d| d.contains("no tab position"))
        .collect();
    assert_eq!(said.len(), 1, "{:?}", p.report().diagnostics);
    assert!(said[0].contains("hole 0 of component 0"), "{said:?}");
    for m in motions.iter().filter(|m| m.kind != MoveKind::Rapid) {
        let (a, b) = (m.curve.start(), m.curve.end());
        if (a[0] - 20.).hypot(a[1] - 20.) < 3. && (b[0] - 20.).hypot(b[1] - 20.) < 3. {
            // Hole passes stay flat at their depth.
            assert!(a[2] != 5. && b[2] != 5., "{m:?}");
        }
    }
    for k in 0..360 {
        let q = on_circle(3.5, k as f64);
        assert!(cell(&stock, q[0], q[1]).upper_mm <= 2. + TIGHT, "{k}");
    }
    assert!(cell(&stock, 20. + 10. * 150f64.to_radians().cos(), 20. + 10. * 150f64.to_radians().sin()).lower_mm >= 5. - TIGHT);
}

#[test]
fn tabs_at_the_even_positions_are_the_evenly_spaced_tabs() {
    // count = 4 centers the tabs at 1/8, 3/8, 5/8, 7/8 of the loop from its
    // start (at angle 0): the same points given explicitly make the same path.
    let mut even = operation();
    even.boundaries = Boundaries::Outer;
    even.tabs = Some(Tabs {
        count: Some(4),
        positions_mm: None,
        position_tolerance_mm: None,
        ..tabs_at(&[], TabShape::Triangle)
    });
    let mut given = even.clone();
    given.tabs = Some(tabs_at(
        &[45., 135., 225., 315.].map(|d| on_circle(8., d)),
        TabShape::Triangle,
    ));
    let (a, _, _) = run(plan(even.clone()).unwrap());
    let (b, _, _) = run(plan(given).unwrap());
    assert_eq!(a.len(), b.len());
    for (a, b) in a.iter().zip(&b) {
        assert_eq!(a.kind, b.kind);
        for (p, q) in [(a.curve.start(), b.curve.start()), (a.curve.end(), b.curve.end())] {
            assert!((0..3).all(|k| (p[k] - q[k]).abs() < 1e-9), "{a:?} {b:?}");
        }
    }
    // Without positions the operation serializes as before.
    let json = serde_json::to_value(&even).unwrap();
    assert_eq!(
        json["tabs"],
        serde_json::json!({"width_mm": 2.0, "height_mm": 3.0, "count": 4, "shape": "triangle"})
    );
}

#[test]
fn tabs_at_positions_refuse_what_cannot_be_placed() {
    let refused = |tabs: Tabs, boundaries: Boundaries| {
        let mut op = operation();
        op.boundaries = boundaries;
        let said = format!("not refused: {tabs:?}");
        op.tabs = Some(tabs);
        plan(op).err().expect(&said)
    };
    // 1 mm off the boundary: named, with its distance, not moved.
    let e = refused(tabs_at(&[on_circle(9., 90.)], TabShape::Rectangle), Boundaries::Outer);
    assert!(e.contains("(20, 29) is not on a selected part boundary"), "{e}");
    assert!(e.contains("1.0000 mm away") && e.contains("(20.0000, 28.0000)"), "{e}");
    // On the hole, which is not selected.
    let e = refused(tabs_at(&[on_circle(4., 180.)], TabShape::Rectangle), Boundaries::Outer);
    assert!(e.contains("not on a selected part boundary") && e.contains("4.0000 mm away"), "{e}");
    // Two tabs 20 degrees (3.5 mm of tool path) apart overlap.
    let e = refused(
        tabs_at(&[on_circle(8., 150.), on_circle(8., 170.)], TabShape::Rectangle),
        Boundaries::Outer,
    );
    assert!(e.contains("overlap") && e.contains("apart"), "{e}");
    // A tab on the loop start, where the tool enters and leaves (read from
    // the plan without tabs: with no lead, the first plunge ends there).
    let mut plain = operation();
    plain.boundaries = Boundaries::Outer;
    let (motions, _, _) = run(plan(plain).unwrap());
    let entry = motions.iter().find(|m| m.kind == MoveKind::Plunge).unwrap().curve.end();
    let start = (entry[1] - 20.).atan2(entry[0] - 20.).to_degrees();
    assert!(off_angle(on_circle(8., 150.), &[start]) > 0.5, "{start}");
    for degrees in [start + 5., start - 5.] {
        let e = refused(tabs_at(&[on_circle(8., degrees)], TabShape::Rectangle), Boundaries::Outer);
        assert!(e.contains("where the tool enters and leaves"), "{e}");
    }
    // A triangular tab 3 mm wide needs 6.5 mm either side of its center
    // (half its width, the tool radius, the 3 mm ramp): more than half the
    // hole's 12.5 mm loop, wherever it is put.
    let mut wide = tabs_at(&[on_circle(4., 180.)], TabShape::Triangle);
    wide.width_mm = 3.;
    let e = refused(wide, Boundaries::Holes);
    assert!(
        e.contains("where the tool enters and leaves") && e.contains("loop"),
        "{e}"
    );
    // Positions and count/spacing are mutually exclusive; the tolerance is
    // stated, not assumed.
    let mut both = tabs_at(&[on_circle(8., 150.)], TabShape::Rectangle);
    both.count = Some(2);
    assert!(refused(both.clone(), Boundaries::Outer).contains("not both"));
    both.count = None;
    both.spacing_mm = Some(10.);
    assert!(refused(both.clone(), Boundaries::Outer).contains("not both"));
    both.spacing_mm = None;
    both.position_tolerance_mm = None;
    assert!(refused(both.clone(), Boundaries::Outer).contains("need position_tolerance_mm"));
    both.positions_mm = Some(vec![]);
    both.position_tolerance_mm = Some(0.01);
    assert!(refused(both.clone(), Boundaries::Outer).contains("1..1000 points"));
    both.positions_mm = None;
    both.count = Some(4);
    assert!(refused(both, Boundaries::Outer).contains("applies only to positions_mm"));
}
