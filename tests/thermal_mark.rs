//! Plasma/laser marking (#18): marks before cuts, inside their parts, with
//! the sheet left whole under them; the typed program keeps marking and
//! cutting apart; the stock model refuses a mark over cut sheet.
use camlib::{
    motion::*,
    planar::{Boundary, Component, PlanarRegion},
    stock::*,
    stock_contact,
    thermal::{ThermalCutOperation, ThermalCutPlan},
    thermal_mark::{ThermalMarkOperation, ThermalMarkPlan},
    tool::{Cutter, CutterShape, HolderSection, ToolAssembly},
    verification::Status,
};
use std::sync::atomic::AtomicBool;

const KERF: f64 = 1.5;
const MARK_WIDTH: f64 = 0.8;
const INITIAL: Vec3 = [0., 0., 40.];

fn torch() -> ToolAssembly {
    ToolAssembly {
        id: "torch".into(),
        cutter: Cutter {
            id: "nozzle".into(),
            name: String::new(),
            shape: CutterShape::Beam,
            diameter_mm: KERF,
            cutting_length_mm: 1.,
            overall_length_mm: 40.,
            shank_diameter_mm: 8.,
            flutes: 1,
            center_cutting: true,
            material: String::new(),
            neck: None,
        },
        stickout_mm: 30.,
        gauge_length_mm: 60.,
        holders: vec![HolderSection {
            z_min_mm: 30.,
            z_max_mm: 60.,
            radius_min_mm: 15.,
            radius_max_mm: 15.,
        }],
    }
}

fn circle(c: [f64; 2], r: f64) -> Boundary {
    Boundary {
        vertices: vec![[c[0] + r, c[1], 1.], [c[0] - r, c[1], 1.]],
    }
}

fn spec() -> StockSpec {
    StockSpec {
        version: 1,
        units: "mm".into(),
        min_mm: [0., 0., 0.],
        max_mm: [200., 120., 6.],
        cell_size_mm: 0.25,
        max_cells: 1 << 22,
    }
}

/// A plate (part 0) with a large hole, a bolt hole and a small hole, and a
/// washer (part 1) nested in the large hole.
fn parts() -> PlanarRegion {
    PlanarRegion {
        version: 1,
        units: "mm".into(),
        name: "parts".into(),
        boundary_error_mm: 0.,
        components: vec![
            Component {
                outer: Boundary {
                    vertices: vec![
                        [20., 20., 0.],
                        [120., 20., 0.],
                        [120., 100., 0.],
                        [20., 100., 0.],
                    ],
                },
                holes: vec![
                    circle([60., 60.], 25.),
                    circle([100., 40.], 6.),
                    circle([100., 80.], 3.),
                ],
            },
            Component {
                outer: circle([60., 60.], 15.),
                holes: vec![circle([60., 60.], 5.)],
            },
        ],
        source: serde_json::Value::Null,
    }
}

/// Whether p, with `margin` around it, lies in the plate (0) or the washer
/// (1): analytic, independent of the planner and of `planar`.
fn in_part(p: [f64; 2], margin: f64) -> Option<usize> {
    let d = |c: [f64; 2]| (p[0] - c[0]).hypot(p[1] - c[1]);
    let washer = d([60., 60.]);
    if washer <= 15. - margin && washer >= 5. + margin {
        return Some(1);
    }
    let plate = p[0] >= 20. + margin
        && p[0] <= 120. - margin
        && p[1] >= 20. + margin
        && p[1] <= 100. - margin
        && washer >= 25. + margin
        && d([100., 40.]) >= 6. + margin
        && d([100., 80.]) >= 3. + margin;
    plate.then_some(0)
}

/// Part numbers as strokes: "7" on the plate, "1" and an arc on the washer.
fn part_numbers() -> serde_json::Value {
    serde_json::json!([
        {"start_mm": [26., 92.], "segments": [
            {"kind": "line", "end_mm": [34., 92.]},
            {"kind": "line", "end_mm": [29., 80.]}]},
        {"start_mm": [68., 57.], "segments": [{"kind": "line", "end_mm": [68., 63.]}]},
        {"start_mm": [50., 60.], "segments": [
            {"kind": "arc", "center_mm": [60., 60.], "sweep_rad": -1.2}]},
    ])
}

fn mark(extra: serde_json::Value) -> ThermalMarkOperation {
    let mut value = serde_json::json!({
        "id": "numbers", "tool": "torch", "parts": parts(), "paths": part_numbers(),
        "feed_mm_min": 4000., "mark_height_mm": 2.5, "height_mode": "program",
        "travel_height_mm": 10., "power_percent": 15., "mark_width_mm": MARK_WIDTH,
        "tolerance_mm": 1e-5, "work_offset": 1, "tool_number": 1, "length_register": 1
    });
    for (k, v) in extra.as_object().unwrap() {
        value[k] = v.clone();
    }
    serde_json::from_value(value).unwrap()
}

fn cut() -> ThermalCutOperation {
    serde_json::from_value(serde_json::json!({
        "id": "plasma", "tool": "torch", "parts": parts(), "feed_mm_min": 2500.,
        "cut_height_mm": 1.5, "height_mode": "program", "pierce_height_mm": 3.8,
        "travel_height_mm": 10., "plunge_mm_min": 1000., "pierce_delay_s": 0.5,
        "lead_in": {"length_mm": 2., "radius_mm": 3.}, "lead_out": {"length_mm": 1., "radius_mm": 2.},
        "overcut_mm": -0.5,
        "tolerance_mm": 1e-5, "work_offset": 1, "tool_number": 1, "length_register": 1
    }))
    .unwrap()
}

struct Run {
    stock: Stock,
    events: Vec<Event>,
    position: Vec3,
    next_id: u64,
}
impl Run {
    fn new() -> Self {
        let cancel = AtomicBool::new(false);
        let mut stock = Stock::new(spec()).unwrap();
        stock.enable_contact_queries(&cancel).unwrap();
        Self {
            stock,
            events: vec![],
            position: INITIAL,
            next_id: 1,
        }
    }
    /// Commit a motion as the stream does: contact check, sweep, apply.
    fn commit(&mut self, m: Motion) {
        let cancel = AtomicBool::new(false);
        let tool = torch().prepare().unwrap();
        let report =
            stock_contact::check(&self.stock, &tool, &m, Default::default(), &cancel).unwrap();
        assert_eq!(report.status, Status::Clear, "{m:?}");
        let d = self
            .stock
            .preview_sweep(&tool, &m, 1., SweepOptions::default(), &cancel)
            .unwrap();
        if m.kind == MoveKind::Mark {
            assert!(d.changes().is_empty(), "a mark removed stock: {m:?}");
        }
        self.stock.apply_delta(&d).unwrap();
        self.position = m.curve.end();
        self.next_id = m.id + 1;
        self.events.push(Event::Motion { motion: m });
    }
    fn mark(&mut self, op: ThermalMarkOperation) -> Result<ThermalMarkPlan, String> {
        let cancel = AtomicBool::new(false);
        let mut plan = ThermalMarkPlan::prepare_with_stock(
            op,
            &torch(),
            &self.stock,
            self.position,
            self.next_id,
            &cancel,
        )?;
        self.events.extend(plan.prefix_events());
        self.events.extend(plan.take_events());
        while let Some(m) = plan.next_motion_with_stock(&self.stock, &cancel)? {
            self.commit(m);
            self.events.extend(plan.take_events());
        }
        plan.finish_report(&cancel).unwrap();
        Ok(plan)
    }
    fn cut(&mut self, op: ThermalCutOperation) -> ThermalCutPlan {
        let cancel = AtomicBool::new(false);
        let mut plan = ThermalCutPlan::prepare(
            op,
            &torch(),
            &spec(),
            self.position,
            self.next_id,
            &cancel,
        )
        .unwrap();
        self.events.extend(plan.prefix_events());
        self.events.extend(plan.take_events());
        while let Some(m) = plan.next_motion_with_stock(&self.stock, &cancel).unwrap() {
            self.commit(m);
            self.events.extend(plan.take_events());
        }
        plan.finish_report(&cancel).unwrap();
        plan
    }
    fn validate(&self) -> Result<(), String> {
        validate(&self.events)
    }
}

fn validate(events: &[Event]) -> Result<(), String> {
    let program = Program {
        version: 1,
        id: "p".into(),
        revision: "1".into(),
        units: "mm".into(),
        tolerance_mm: 1e-5,
        frame: WorkFrame::identity(),
        initial_position_mm: INITIAL,
        tools: vec![torch()],
        events: events.to_vec(),
    };
    ProgramValidator::new(&program, None)?.consume(events)
}

fn cell(stock: &Stock, p: [f64; 2]) -> HeightBounds {
    let s = stock.spec();
    let nx = ((s.max_mm[0] - s.min_mm[0]) / s.cell_size_mm).round() as usize;
    let i = ((p[0] - s.min_mm[0]) / s.cell_size_mm) as usize;
    let j = ((p[1] - s.min_mm[1]) / s.cell_size_mm) as usize;
    stock.cells()[j * nx + i]
}

fn marks(events: &[Event]) -> Vec<&Motion> {
    events
        .iter()
        .filter_map(|e| match e {
            Event::Motion { motion } if motion.kind == MoveKind::Mark => Some(motion),
            _ => None,
        })
        .collect()
}

#[test]
fn part_numbers_are_marked_then_cut_and_the_sheet_stays_whole_under_them() {
    let mut run = Run::new();
    let plan = run.mark(mark(serde_json::json!({}))).unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    assert_eq!(report["strategy"], "thermal_mark");
    assert_eq!(report["parts"], serde_json::json!([0, 1, 1]));
    assert_eq!(report["containment"], "proven");
    assert_eq!(report["paths_completed"], 3);
    assert_eq!(report["segments"], 4);
    let expected = 8. + 13. + 6. + 10. * 1.2;
    assert!((report["mark_length_mm"].as_f64().unwrap() - expected).abs() < 1e-9);
    // Marking removed nothing at all.
    let whole = Stock::new(spec()).unwrap();
    assert_eq!(run.stock.cells(), whole.cells());
    let marked = run.events.len();
    run.cut(cut());
    // One typed program: every mark under the marking beam, every cut under
    // the cutting beam, fired again after the marks.
    run.validate().unwrap();
    let all = marks(&run.events);
    assert_eq!(all.len(), 4);
    let first_cut = run
        .events
        .iter()
        .position(|e| matches!(e, Event::Motion { motion } if motion.kind == MoveKind::Cut))
        .unwrap();
    assert!(first_cut > marked, "a cut before the last mark");
    let beams: Vec<_> = run
        .events
        .iter()
        .filter(|e| matches!(e, Event::Beam { .. } | Event::BeamMark { .. }))
        .collect();
    assert_eq!(
        beams[..6],
        [
            &Event::BeamMark { power_percent: 15. },
            &Event::Beam { power_percent: 0. },
            &Event::BeamMark { power_percent: 15. },
            &Event::Beam { power_percent: 0. },
            &Event::BeamMark { power_percent: 15. },
            &Event::Beam { power_percent: 0. },
        ]
    );
    assert_eq!(beams[6], &Event::Beam { power_percent: 100. });
    // Every mark: at the marking height and feed, inside its part by half
    // the mark width (analytic), and over sheet the cut left whole.
    for m in all {
        assert_eq!(m.feed_mm_min, Some(4000.));
        let path = m.curve.prepare(1e-5).unwrap();
        let owner = in_part(
            {
                let p = path.position(0.5).unwrap();
                [p[0], p[1]]
            },
            MARK_WIDTH / 2.,
        );
        assert!(owner.is_some(), "{m:?}");
        for k in 0..=200 {
            let p = path.position(k as f64 / 200.).unwrap();
            assert_eq!(p[2], 8.5);
            assert_eq!(in_part([p[0], p[1]], MARK_WIDTH / 2.), owner, "{p:?}");
            let c = cell(&run.stock, [p[0], p[1]]);
            assert_eq!((c.lower_mm, c.upper_mm), (6., 6.), "cut under a mark at {p:?}");
        }
    }
    // The cut itself went through (the kerf beside the plate's left edge).
    let kerf = cell(&run.stock, [20. - KERF / 2., 60.]);
    assert_eq!((kerf.lower_mm, kerf.upper_mm), (0., 0.));
}

#[test]
fn marks_on_the_scrap_or_across_an_edge_are_refused_unless_asked() {
    let line = |a: [f64; 2], b: [f64; 2]| {
        serde_json::json!([{"start_mm": a, "segments": [{"kind": "line", "end_mm": b}]}])
    };
    // In the bolt hole (scrap), outside every part, across the plate's
    // edge, into a hole, and too close to an edge for the mark's width.
    for (paths, expected) in [
        (line([98., 40.], [102., 40.]), "outside every part"),
        (line([5., 5.], [15., 5.]), "outside every part"),
        (line([25., 60.], [15., 60.]), "not proven inside part 0"),
        (line([90., 40.], [97., 40.]), "not proven inside part 0"),
        (line([20.3, 30.], [20.3, 50.]), "not proven inside part 0"),
        // From the washer across its kerf gap into the plate.
        (line([70., 60.], [90., 60.]), "not proven inside part 1"),
    ] {
        let mut run = Run::new();
        let error = run
            .mark(mark(serde_json::json!({ "paths": paths })))
            .err()
            .unwrap();
        assert!(error.contains(expected), "{error}");
        assert!(error.contains("allow_off_part"), "{error}");
        assert!(run.events.is_empty());
        // Asked for: planned, and reported as not checked.
        let plan = run
            .mark(mark(
                serde_json::json!({ "paths": paths, "allow_off_part": true }),
            ))
            .unwrap();
        assert_eq!(plan.report().containment, "not_checked");
    }
    // The edge clearance is part of the proof.
    let near = line([22., 30.], [22., 50.]);
    let mut run = Run::new();
    run.mark(mark(serde_json::json!({ "paths": near })))
        .unwrap();
    let error = Run::new()
        .mark(mark(
            serde_json::json!({ "paths": near, "edge_clearance_mm": 2. }),
        ))
        .err()
        .unwrap();
    assert!(error.contains("edge clearance 2 mm"), "{error}");
}

#[test]
fn a_mark_after_its_part_was_cut_is_refused() {
    let mut run = Run::new();
    run.cut(cut());
    let error = run.mark(mark(serde_json::json!({}))).err().unwrap();
    assert!(
        error.contains("part 0 is already cut") && error.contains("before the cut"),
        "{error}"
    );
    // Asking for marks off the parts does not lift the order.
    let error = run
        .mark(mark(serde_json::json!({ "allow_off_part": true })))
        .err()
        .unwrap();
    assert!(error.contains("already cut"), "{error}");
    // A part cut elsewhere on the sheet does not stop marking this one.
    let mut run = Run::new();
    let far = PlanarRegion {
        components: vec![Component {
            outer: circle([170., 60.], 12.),
            holes: vec![],
        }],
        ..parts()
    };
    let mut other = cut();
    other.parts = far;
    run.cut(other);
    run.mark(mark(serde_json::json!({}))).unwrap();
    run.validate().unwrap();
}

/// The order check does not depend on how the cut's kerf falls on the grid:
/// a fine grid, a part whose edges lie off the cell lines, and a cut with
/// the widest boundary tolerance (its kerf 0.05 mm off the outline).
#[test]
fn a_cut_part_is_found_on_a_fine_grid_with_a_wide_cut_margin() {
    let cancel = AtomicBool::new(false);
    let spec = StockSpec {
        version: 1,
        units: "mm".into(),
        min_mm: [0., 0., 0.],
        max_mm: [30., 30., 6.],
        cell_size_mm: 0.05,
        max_cells: 1 << 22,
    };
    let square = PlanarRegion {
        components: vec![Component {
            outer: Boundary {
                vertices: vec![
                    [10.02, 10.02, 0.],
                    [20.03, 10.02, 0.],
                    [20.03, 20.03, 0.],
                    [10.02, 20.03, 0.],
                ],
            },
            holes: vec![],
        }],
        ..parts()
    };
    let tool = torch();
    let prepared = tool.prepare().unwrap();
    let mut stock = Stock::new(spec.clone()).unwrap();
    let mut op = cut();
    op.parts = square.clone();
    op.boundary_tolerance_mm = 0.1;
    let mut plan = ThermalCutPlan::prepare(op, &tool, &spec, INITIAL, 1, &cancel).unwrap();
    let mut next_id = 1;
    let mut position = INITIAL;
    while let Some(m) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        let d = stock
            .preview_sweep(&prepared, &m, 1., SweepOptions::default(), &cancel)
            .unwrap();
        stock.apply_delta(&d).unwrap();
        next_id = m.id + 1;
        position = m.curve.end();
    }
    // A mark in the middle of the part, far from every kerf.
    let mut op = mark(serde_json::json!({"paths": [
        {"start_mm": [13., 15.], "segments": [{"kind": "line", "end_mm": [17., 15.]}]}
    ]}));
    op.parts = square;
    let error = ThermalMarkPlan::prepare_with_stock(op, &tool, &stock, position, next_id, &cancel)
        .err()
        .unwrap();
    assert!(error.contains("part 0 is already cut"), "{error}");
}

#[test]
fn the_stock_refuses_a_mark_over_cut_sheet_or_off_the_sheet() {
    let cancel = AtomicBool::new(false);
    let tool = torch().prepare().unwrap();
    let mut stock = Stock::new(spec()).unwrap();
    let motion = |id, kind, a: [f64; 2], b: [f64; 2]| Motion {
        id,
        operation: "op".into(),
        tool: "torch".into(),
        kind,
        curve: Curve::Line {
            start: [a[0], a[1], 8.],
            end: [b[0], b[1], 8.],
        },
        feed_mm_min: Some(1000.),
    };
    // A through cut along y = 50.
    let d = stock
        .preview_sweep(
            &tool,
            &motion(1, MoveKind::Cut, [40., 50.], [80., 50.]),
            1.,
            SweepOptions::default(),
            &cancel,
        )
        .unwrap();
    assert!(!d.changes().is_empty());
    stock.apply_delta(&d).unwrap();
    let sweep = |m: &Motion| stock.preview_sweep(&tool, m, 1., SweepOptions::default(), &cancel);
    // Whole sheet: accepted, nothing removed.
    let clear = sweep(&motion(2, MoveKind::Mark, [40., 60.], [80., 60.])).unwrap();
    assert!(clear.changes().is_empty());
    // Across the kerf, and along its edge (possibly cut cells).
    for (a, b) in [([60., 40.], [60., 60.]), ([40., 50.9], [80., 50.9])] {
        let error = sweep(&motion(2, MoveKind::Mark, a, b)).unwrap_err();
        assert!(
            error.contains("cut through") && error.contains("before the cuts"),
            "{error}"
        );
    }
    // Beyond the sheet edge.
    let error = sweep(&motion(2, MoveKind::Mark, [190., 60.], [205., 60.])).unwrap_err();
    assert!(error.contains("leaves the sheet"), "{error}");
    // Only a beam marks.
    let mut mill = torch();
    mill.cutter.shape = CutterShape::Flat;
    let error = stock
        .preview_sweep(
            &mill.prepare().unwrap(),
            &motion(2, MoveKind::Mark, [40., 60.], [80., 60.]),
            1.,
            SweepOptions::default(),
            &cancel,
        )
        .unwrap_err();
    assert!(error.contains("beam tool"), "{error}");
}

#[test]
fn the_typed_program_keeps_marking_and_cutting_apart() {
    let line = |id, kind, a: Vec3, b: Vec3| Event::Motion {
        motion: Motion {
            id,
            operation: "op".into(),
            tool: "torch".into(),
            kind,
            curve: Curve::Line { start: a, end: b },
            feed_mm_min: (kind != MoveKind::Rapid).then_some(1000.),
        },
    };
    let prefix = || {
        vec![
            Event::WorkOffset { number: 1 },
            Event::ToolChange {
                tool: "torch".into(),
                number: 1,
                length_register: 1,
                position_after_mm: INITIAL,
            },
            line(1, MoveKind::Rapid, INITIAL, [10., 10., 8.]),
        ]
    };
    let (a, b, c) = ([10., 10., 8.], [30., 10., 8.], [30., 30., 8.]);
    let with = |tail: Vec<Event>| {
        let mut events = prefix();
        events.extend(tail);
        validate(&events)
    };
    let on = Event::BeamMark { power_percent: 20. };
    let cutting = Event::Beam { power_percent: 100. };
    let off = Event::Beam { power_percent: 0. };
    // Mark, off, cut: valid.
    with(vec![
        on.clone(),
        line(2, MoveKind::Mark, a, b),
        off.clone(),
        cutting.clone(),
        line(3, MoveKind::Cut, b, c),
        off.clone(),
    ])
    .unwrap();
    for (tail, expected) in [
        // A mark needs the marking beam: not the beam off, not the cutting beam.
        (vec![line(2, MoveKind::Mark, a, b)], "marking motion requires"),
        (
            vec![cutting.clone(), line(2, MoveKind::Mark, a, b)],
            "marking motion requires",
        ),
        // No cut (or lead, or plunge) under the marking beam.
        (
            vec![on.clone(), line(2, MoveKind::Cut, a, b)],
            "only marking motions are fed",
        ),
        (
            vec![on.clone(), line(2, MoveKind::Lead, a, b)],
            "only marking motions are fed",
        ),
        // The marking beam is turned off before the cutting beam fires,
        // and the cutting beam before the marking beam.
        (
            vec![on.clone(), cutting.clone()],
            "turn it off before firing the cutting beam",
        ),
        (
            vec![cutting.clone(), on.clone()],
            "turn the beam off before firing it in the marking process",
        ),
        (
            vec![Event::BeamMark { power_percent: 0. }],
            "marking beam power must be above 0",
        ),
        // A mark stays at one height.
        (
            vec![
                on.clone(),
                line(2, MoveKind::Mark, a, [30., 10., 7.]),
            ],
            "line or an XY arc at one height",
        ),
    ] {
        let error = with(tail).unwrap_err();
        assert!(error.contains(expected), "{error}");
    }
    // A marking move is wire-encoded as its own kind and never cuts.
    assert!(!MoveKind::Mark.cuts() && !MoveKind::Mark.is_rapid());
    assert_eq!(
        serde_json::to_value(MoveKind::Mark).unwrap(),
        serde_json::json!("mark")
    );
}

#[test]
fn refusals_name_the_problem() {
    for (extra, expected) in [
        (serde_json::json!({"power_percent": 0.}), "power_percent"),
        (serde_json::json!({"mark_width_mm": 0.}), "mark_width_mm"),
        (serde_json::json!({"mark_height_mm": 0.}), "mark_height_mm"),
        (serde_json::json!({"feed_mm_min": 0.}), "feed_mm_min"),
        (serde_json::json!({"paths": []}), "at least one path"),
        (
            serde_json::json!({"height_mode": "fixed"}),
            "travel_height_mm needs height_mode",
        ),
        (
            serde_json::json!({"travel_height_mm": 1.}),
            "at least mark_height_mm",
        ),
    ] {
        let error = Run::new().mark(mark(extra)).err().unwrap();
        assert!(error.contains(expected), "{error}");
    }
    // A milling cutter cannot mark.
    let cancel = AtomicBool::new(false);
    let mut mill = torch();
    mill.cutter.shape = CutterShape::Flat;
    let stock = Stock::new(spec()).unwrap();
    let error = ThermalMarkPlan::prepare_with_stock(
        mark(serde_json::json!({})),
        &mill,
        &stock,
        INITIAL,
        1,
        &cancel,
    )
    .err()
    .unwrap();
    assert!(error.contains("beam tool"), "{error}");
    // Fixed height: one Z from the first move on.
    let mut run = Run::new();
    run.mark(mark(
        serde_json::json!({"height_mode": "fixed", "travel_height_mm": null}),
    ))
    .unwrap();
    let z: Vec<f64> = run
        .events
        .iter()
        .filter_map(|e| match e {
            Event::Motion { motion } => Some(motion.curve.end()[2]),
            _ => None,
        })
        .collect();
    assert!(z.iter().all(|z| *z == 8.5), "{z:?}");
}
