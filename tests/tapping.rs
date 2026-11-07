use camlib::{
    holes::{DrillingOperation, DrillingPlan, Hand, TappingOperation, TappingPlan},
    motion::{Curve, Event, Motion, MoveKind, ProgramValidator, SpindleDirection},
    pocket::Operation,
    stock::Stock,
    stream::StreamManifest,
    tool::ToolAssembly,
};
use std::sync::atomic::AtomicBool;

fn fixture() -> StreamManifest {
    serde_json::from_str(include_str!("../fixtures/operations/holes.json")).unwrap()
}
/// M5 tap modeled by its 4.2 mm drill diameter, 12 mm thread.
fn tap_tool() -> ToolAssembly {
    let mut tool = fixture().tools[1].clone();
    tool.id = "m5".into();
    tool.cutter.id = "m5-tap".into();
    tool.cutter.diameter_mm = 4.;
    tool.cutter.cutting_length_mm = 12.;
    tool.cutter.shank_diameter_mm = 4.;
    tool.cutter.center_cutting = false;
    tool
}
fn operation() -> TappingOperation {
    TappingOperation {
        limits: None,
        id: "tap".into(),
        tool: "m5".into(),
        centers_mm: vec![[10., 12.], [28., 12.]],
        pitch_mm: 0.8,
        hand: Hand::Right,
        depth_z_mm: 5.,
        bottom_clearance_mm: None,
        clearance_z_mm: 12.,
        spindle_rpm: 500.,
        tolerance_mm: 1e-5,
        work_offset: 1,
        tool_number: 3,
        length_register: 3,
        coolant: camlib::motion::Coolant::Flood,
        retract_z_mm: Some(11.),
        feed_z_mm: Some(10.5),
    }
}
fn commit(stock: &mut Stock, tool: &ToolAssembly, motion: &Motion) {
    let tool = tool.prepare().unwrap();
    let d = stock
        .preview_sweep(&tool, motion, 1., fixture().sweep, &AtomicBool::new(false))
        .unwrap();
    stock.apply_delta(&d).unwrap();
}
fn drilled(depth: f64) -> Stock {
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let Operation::Drilling(mut drill) = fixture().operations.remove(0) else {
        panic!()
    };
    drill.depth_z_mm = depth;
    let drill: DrillingOperation = drill;
    let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let mut plan = DrillingPlan::prepare(
        drill,
        &m.tools[0],
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &cancel,
    )
    .unwrap();
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        commit(&mut stock, &m.tools[0], &motion);
    }
    stock
}
fn run(stock: &mut Stock) -> Result<(Vec<Motion>, TappingPlan), String> {
    run_op(stock, operation())
}
fn run_op(stock: &mut Stock, op: TappingOperation) -> Result<(Vec<Motion>, TappingPlan), String> {
    let m = fixture();
    let cancel = AtomicBool::new(false);
    let tool = tap_tool();
    let mut plan = TappingPlan::prepare(
        op,
        &tool,
        m.stock.height().unwrap(),
        m.initial_position_mm,
        1,
        &cancel,
    )?;
    let mut motions = vec![];
    while let Some(motion) = plan.next_motion_with_stock(stock, &cancel)? {
        commit(stock, &tool, &motion);
        motions.push(motion);
    }
    plan.finish_report(&cancel)?;
    Ok((motions, plan))
}

#[test]
fn taps_each_drilled_hole_as_one_synchronized_pair() {
    // The fixture drill is 4 mm; drilled to Z 2 the hole is open below the
    // tap depth 5 less one pitch (bottom clearance) within the proven radius.
    let mut stock = drilled(2.);
    let (motions, plan) = run(&mut stock).unwrap();
    let report = plan.report();
    assert_eq!(report.synchronized_feed_mm_min, 400.);
    assert_eq!(report.r_plane_z_mm, 10.5);
    assert_eq!(report.completed_holes, 2);
    let taps: Vec<_> = motions.iter().filter(|m| m.kind == MoveKind::Tap).collect();
    assert_eq!(taps.len(), 4);
    for pair in taps.chunks(2) {
        let (down, up) = (&pair[0], &pair[1]);
        assert_eq!(down.curve.start()[2], 10.5);
        assert_eq!(down.curve.end()[2], 5.);
        assert_eq!(up.curve.start(), down.curve.end());
        assert_eq!(up.curve.end(), down.curve.start());
        assert_eq!(down.feed_mm_min, Some(400.));
        assert_eq!(up.feed_mm_min, Some(400.));
    }
    // A program with the pair validates; a broken pair does not.
    let m = fixture();
    let program = camlib::motion::Program {
        version: 1,
        id: "p".into(),
        revision: "1".into(),
        units: "mm".into(),
        frame: camlib::motion::WorkFrame::identity(),
        tolerance_mm: 1e-5,
        initial_position_mm: m.initial_position_mm,
        tools: vec![tap_tool()],
        events: vec![],
    };
    let mut events = plan.prefix_events();
    events.extend(motions.iter().map(|m| Event::Motion { motion: m.clone() }));
    let mut validator = ProgramValidator::new(&program, None).unwrap();
    validator.consume(&events).unwrap();
    let down = events
        .iter()
        .position(|e| matches!(e, Event::Motion { motion } if motion.kind == MoveKind::Tap))
        .unwrap();
    let mut broken = events[..=down].to_vec();
    let Event::Motion { motion } = &events[down + 1] else {
        unreachable!()
    };
    let mut wrong = motion.clone();
    wrong.curve = Curve::Line {
        start: motion.curve.start(),
        end: [motion.curve.end()[0], motion.curve.end()[1], 11.],
    };
    broken.push(Event::Motion { motion: wrong });
    let error = ProgramValidator::new(&program, None)
        .unwrap()
        .consume(&broken)
        .err()
        .unwrap();
    assert!(error.contains("tapping return must reverse"), "{error}");
}

#[test]
fn shallow_or_missing_holes_are_refused_before_the_tap_goes_down() {
    let m = fixture();
    let mut fresh = Stock::new(m.stock.height().unwrap().clone()).unwrap();
    let error = run(&mut fresh).err().unwrap();
    assert!(
        error.contains("tapping hole 0: the stock is not proven open"),
        "{error}"
    );
    let mut shallow = drilled(6.);
    let error = run(&mut shallow).err().unwrap();
    assert!(error.contains("drill the hole deeper"), "{error}");
}

fn program() -> camlib::motion::Program {
    let m = fixture();
    camlib::motion::Program {
        version: 1,
        id: "p".into(),
        revision: "1".into(),
        units: "mm".into(),
        frame: camlib::motion::WorkFrame::identity(),
        tolerance_mm: 1e-5,
        initial_position_mm: m.initial_position_mm,
        tools: vec![tap_tool()],
        events: vec![],
    }
}

/// #10: a left-hand tap turns counterclockwise into the hole (M4), with the
/// same synchronized pair; the spindle direction is the thread hand.
#[test]
fn left_hand_tap_runs_the_spindle_counterclockwise() {
    let mut stock = drilled(2.);
    let mut op = operation();
    op.hand = Hand::Left;
    let (motions, plan) = run_op(&mut stock, op).unwrap();
    assert_eq!(plan.report().hand, Hand::Left);
    assert_eq!(plan.report().synchronized_feed_mm_min, 400.);
    let events = plan.prefix_events();
    let spindles: Vec<_> = events
        .iter()
        .filter_map(|e| match e {
            Event::Spindle { rpm, direction } => Some((*rpm, *direction)),
            _ => None,
        })
        .collect();
    assert_eq!(spindles, vec![(500., SpindleDirection::Ccw)]);
    let (right, right_plan) = run(&mut drilled(2.)).unwrap();
    assert!(right_plan.prefix_events().iter().any(|e| matches!(
        e,
        Event::Spindle {
            direction: SpindleDirection::Cw,
            ..
        }
    )));
    // Same motions; only the spindle sense differs.
    assert_eq!(
        right.iter().map(|m| &m.curve).collect::<Vec<_>>(),
        motions.iter().map(|m| &m.curve).collect::<Vec<_>>()
    );
    let mut all = events.clone();
    all.extend(motions.iter().map(|m| Event::Motion { motion: m.clone() }));
    ProgramValidator::new(&program(), None)
        .unwrap()
        .consume(&all)
        .unwrap();
    // Serialized only when left (right is the designation default).
    let json = serde_json::to_value(operation()).unwrap();
    assert!(json.get("hand").is_none());
    let mut left = operation();
    left.hand = Hand::Left;
    assert_eq!(serde_json::to_value(left).unwrap()["hand"], "left");
}

#[test]
fn tapping_with_the_spindle_stopped_is_refused() {
    let mut stock = drilled(2.);
    let (motions, plan) = run(&mut stock).unwrap();
    let mut events: Vec<_> = plan
        .prefix_events()
        .into_iter()
        .map(|e| match e {
            Event::Spindle { direction, .. } => Event::Spindle { rpm: 0., direction },
            e => e,
        })
        .collect();
    events.extend(motions.iter().map(|m| Event::Motion { motion: m.clone() }));
    let error = ProgramValidator::new(&program(), None)
        .unwrap()
        .consume(&events)
        .err()
        .unwrap();
    assert!(error.contains("spindle running"), "{error}");
}
