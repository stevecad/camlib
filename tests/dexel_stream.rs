//! Tri-dexel runs in the stream (issue #1): committed frames re-checked by
//! the receiver, checkpoints, partial moves and refusals.
use camlib::{dexel::*, motion::*, stock::*, stream::*, tool::*};
use std::{io::Cursor, sync::atomic::AtomicBool};

fn manifest() -> StreamManifest {
    StreamManifest {
        tool_change: None,
        initial_stock: None,
        rapid_retracts: false,
        turning: None,
        end_position: None,
        version: 1,
        run: RunIdentity {
            id: "run-1".into(),
            revision: "cad-sha-1".into(),
        },
        stock: RunStockSpec::Dexel(DexelStockSpec {
            version: 1,
            units: "mm".into(),
            model: DexelModel::TriDexel,
            min_mm: [0.; 3],
            max_mm: [20., 20., 10.],
            cell_size_mm: 0.25,
            max_rays: 1_000_000,
            max_intervals: 10_000_000,
        }),
        tools: vec![ToolAssembly {
            id: "t1".into(),
            cutter: Cutter {
                id: "cutter".into(),
                name: "4 mm ball".into(),
                shape: CutterShape::Ball,
                diameter_mm: 4.,
                cutting_length_mm: 30.,
                overall_length_mm: 50.,
                shank_diameter_mm: 4.,
                flutes: 2,
                center_cutting: true,
                material: "carbide".into(),
                neck: None,
            },
            stickout_mm: 35.,
            gauge_length_mm: 60.,
            holders: vec![HolderSection {
                z_min_mm: 35.,
                z_max_mm: 60.,
                radius_min_mm: 4.,
                radius_max_mm: 4.,
            }],
        }],
        initial_position_mm: [3., 5., 12.],
        sweep: SweepOptions::default(),
        operations: vec![],
        verification: None,
    }
}
fn motions() -> Vec<Motion> {
    [
        (
            MoveKind::Rapid,
            Curve::Line {
                start: [3., 5., 12.],
                end: [3., 5., 6.],
            },
        ),
        (
            MoveKind::Cut,
            Curve::Line {
                start: [3., 5., 6.],
                end: [17., 5., 6.],
            },
        ),
        (
            MoveKind::Cut,
            Curve::Line {
                start: [17., 5., 6.],
                end: [17., 15., 5.],
            },
        ),
        (
            MoveKind::Cut,
            Curve::Arc {
                start: [17., 15., 5.],
                end: [14., 18., 5.],
                center: [14., 15., 5.],
                plane: Plane::Xy,
                sweep_rad: std::f64::consts::FRAC_PI_2,
            },
        ),
        (
            MoveKind::Cut,
            Curve::Helix {
                start: [14., 18., 5.],
                end: [14., 18., 4.],
                center: [14., 15., 5.],
                plane: Plane::Xy,
                sweep_rad: std::f64::consts::TAU,
            },
        ),
    ]
    .into_iter()
    .enumerate()
    .map(|(i, (kind, curve))| Motion {
        id: i as u64 + 10,
        operation: "pocket".into(),
        tool: "t1".into(),
        kind,
        curve,
        feed_mm_min: (kind != MoveKind::Rapid).then_some(1000.),
    })
    .collect()
}
fn rays(stock: &RunStock) -> Vec<Ray> {
    let d = stock.dexel().unwrap();
    (0..3).flat_map(|a| d.rays(a).to_vec()).collect()
}
static NONE: AtomicBool = AtomicBool::new(false);

#[test]
fn receiver_rechecks_and_matches_the_producer() {
    let m = manifest();
    let mut producer = StreamProducer::new(m.clone()).unwrap();
    let mut receiver = StreamReceiver::new(m.clone()).unwrap();
    let mut frames = vec![];
    for motion in motions() {
        let frame = producer
            .push_motion(motion, DEFAULT_FRAME_LIMIT, &NONE)
            .unwrap();
        receiver.accept(&frame, DEFAULT_FRAME_LIMIT).unwrap();
        frames.push(frame);
    }
    assert_eq!(rays(producer.run_stock()), rays(receiver.run_stock()));
    // The height view follows the Z rays on both sides.
    assert_eq!(
        producer.stock().unwrap().cells(),
        receiver.stock().unwrap().cells()
    );
    let v = receiver.run_stock().dexel().unwrap().volume();
    assert!(v.removed_lower_mm3 > 50., "{v:?}");
    // Repeated, out-of-order and foreign frames are refused.
    assert!(receiver.accept(&frames[4], DEFAULT_FRAME_LIMIT).is_err());
    let mut fresh = StreamReceiver::new(m.clone()).unwrap();
    assert!(fresh.accept(&frames[1], DEFAULT_FRAME_LIMIT).is_err());
    let mut other = m.clone();
    other.run.revision = "cad-sha-2".into();
    let mut foreign = StreamReceiver::new(other).unwrap();
    assert!(foreign.accept(&frames[0], DEFAULT_FRAME_LIMIT).is_err());
    // A height-field chunk frame is not a tri-dexel run's frame.
    let mut height = m.clone();
    height.stock = StockSpec {
        version: 1,
        units: "mm".into(),
        min_mm: [0.; 3],
        max_mm: [20., 20., 10.],
        cell_size_mm: 0.25,
        max_cells: 1_000_000,
    }
    .into();
    let mut hp = StreamProducer::new(height).unwrap();
    let hf = hp
        .push_motion(motions().remove(0), DEFAULT_FRAME_LIMIT, &NONE)
        .unwrap();
    assert!(fresh.accept(&hf, DEFAULT_FRAME_LIMIT).is_err());
    // A flipped byte fails the frame checksum.
    let mut bad = frames[0].clone();
    let n = bad.len() - 1;
    bad[n] ^= 1;
    assert!(fresh.accept(&bad, DEFAULT_FRAME_LIMIT).is_err());
}

#[test]
fn changes_that_add_material_or_disagree_are_refused() {
    let m = manifest();
    let mut producer = StreamProducer::new(m.clone()).unwrap();
    let all = motions();
    let mut frames = vec![];
    for motion in all.iter().take(3) {
        frames.push(
            producer
                .push_motion(motion.clone(), DEFAULT_FRAME_LIMIT, &NONE)
                .unwrap(),
        );
    }
    let chunk = decode_dexel_chunk_frame(&m, &frames[2], DEFAULT_FRAME_LIMIT).unwrap();
    let mut receiver = StreamReceiver::new(m.clone()).unwrap();
    for f in &frames[..2] {
        receiver.accept(f, DEFAULT_FRAME_LIMIT).unwrap();
    }
    let stock = receiver.run_stock().dexel().unwrap().clone();
    assert!(!chunk.changes.is_empty());
    // As sent, the changes apply.
    let mut ok = stock.clone();
    ok.apply_changes(&chunk.motion, 1., &chunk.changes).unwrap();
    // Material that comes back.
    let mut grow = chunk.changes.clone();
    grow[0].after = grow[0].before.clone();
    grow[0].after.possible = Spans::One([0., 20.]);
    grow[0].after.certain = Spans::One([0., 20.]);
    assert!(
        stock
            .clone()
            .apply_changes(&chunk.motion, 1., &grow)
            .is_err()
    );
    // An old value this receiver does not hold.
    let mut stale = chunk.changes.clone();
    stale[0].before.possible = Spans::Empty;
    stale[0].before.certain = Spans::Empty;
    assert!(
        stock
            .clone()
            .apply_changes(&chunk.motion, 1., &stale)
            .is_err()
    );
    // Certain material outside the possible material.
    let mut inverted = chunk.changes.clone();
    inverted[0].after.certain = inverted[0].before.possible.clone();
    inverted[0].after.possible = Spans::Empty;
    assert!(
        stock
            .clone()
            .apply_changes(&chunk.motion, 1., &inverted)
            .is_err()
    );
    // Out of order.
    let mut order = chunk.changes.clone();
    order.reverse();
    if order.len() > 1 {
        assert!(
            stock
                .clone()
                .apply_changes(&chunk.motion, 1., &order)
                .is_err()
        );
    }
}

#[test]
fn checkpoints_restore_and_continue() {
    let m = manifest();
    let mut producer = StreamProducer::new(m.clone()).unwrap();
    let all = motions();
    let mut frames = vec![];
    let mut checkpoint = None;
    for (i, motion) in all.into_iter().enumerate() {
        frames.push(
            producer
                .push_motion(motion, DEFAULT_FRAME_LIMIT, &NONE)
                .unwrap(),
        );
        if i == 2 {
            checkpoint = Some(producer.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap());
        }
    }
    let checkpoint = checkpoint.unwrap();
    for restored in [
        StreamReceiver::from_checkpoint(m.clone(), &checkpoint, CHECKPOINT_FRAME_LIMIT).unwrap(),
        StreamReceiver::from_checkpoint_reader(
            m.clone(),
            &mut Cursor::new(&checkpoint),
            CHECKPOINT_FRAME_LIMIT,
        )
        .unwrap(),
    ] {
        let mut r = restored;
        assert_eq!(r.completed(), 3);
        for f in &frames[3..] {
            r.accept(f, DEFAULT_FRAME_LIMIT).unwrap();
        }
        assert_eq!(rays(r.run_stock()), rays(producer.run_stock()));
        assert_eq!(
            r.stock().unwrap().cells(),
            producer.stock().unwrap().cells()
        );
    }
    // A checkpoint of another run, or a truncated one, is refused.
    let mut other = m.clone();
    other.run.id = "run-2".into();
    assert!(StreamReceiver::from_checkpoint(other, &checkpoint, CHECKPOINT_FRAME_LIMIT).is_err());
    assert!(
        StreamReceiver::from_checkpoint_reader(
            m.clone(),
            &mut Cursor::new(&checkpoint[..checkpoint.len() - 8]),
            CHECKPOINT_FRAME_LIMIT
        )
        .is_err()
    );
    // A producer resumes from it and produces the same frames.
    let mut resumed =
        StreamProducer::from_checkpoint(m.clone(), &checkpoint, CHECKPOINT_FRAME_LIMIT).unwrap();
    for (motion, frame) in motions().into_iter().zip(&frames).skip(3) {
        assert_eq!(
            &resumed
                .push_motion(motion, DEFAULT_FRAME_LIMIT, &NONE)
                .unwrap(),
            frame
        );
    }
}

#[test]
fn partial_moves_preview_from_the_receiver() {
    let m = manifest();
    let mut producer = StreamProducer::new(m.clone()).unwrap();
    let all = motions();
    let mut receiver = StreamReceiver::new(m.clone()).unwrap();
    for motion in all.iter().take(2) {
        let f = producer
            .push_motion(motion.clone(), DEFAULT_FRAME_LIMIT, &NONE)
            .unwrap();
        receiver.accept(&f, DEFAULT_FRAME_LIMIT).unwrap();
    }
    let next = &all[2];
    let none = receiver.preview_dexel(next, 0., &NONE).unwrap();
    let half = receiver.preview_dexel(next, 0.5, &NONE).unwrap();
    let whole = receiver.preview_dexel(next, 1., &NONE).unwrap();
    let start = receiver.run_stock().dexel().unwrap().volume();
    let (n, h, w) = (none.volume(), half.volume(), whole.volume());
    assert_eq!(n.remaining_upper_mm3, start.remaining_upper_mm3);
    assert!(w.remaining_upper_mm3 < h.remaining_upper_mm3);
    assert!(h.remaining_upper_mm3 < n.remaining_upper_mm3);
    let f = producer
        .push_motion(next.clone(), DEFAULT_FRAME_LIMIT, &NONE)
        .unwrap();
    receiver.accept(&f, DEFAULT_FRAME_LIMIT).unwrap();
    let committed = receiver.run_stock().dexel().unwrap();
    for a in 0..3 {
        assert_eq!(committed.rays(a), whole.rays(a));
    }
    // The producer refuses a height-field preview frame for a dexel run.
    assert!(
        producer
            .preview_motion(&all[3], 0.5, DEFAULT_FRAME_LIMIT, &NONE)
            .unwrap_err()
            .contains("preview_dexel")
    );
}

#[test]
fn manifests_refuse_what_dexel_stock_cannot_model() {
    let mut m = manifest();
    m.tools[0].cutter.shape = CutterShape::Beam;
    assert!(
        StreamReceiver::new(m)
            .err()
            .unwrap()
            .contains("milling cutters")
    );
    let mut m = manifest();
    if let RunStockSpec::Dexel(s) = &mut m.stock {
        s.max_rays = 10;
    }
    assert!(StreamReceiver::new(m).err().unwrap().contains("max_rays"));
    // The manifest's JSON names the model, so a height-field spec never
    // reads as a dexel one (and the other way round).
    let json = serde_json::to_string(&manifest()).unwrap();
    assert!(json.contains("\"model\":\"tri_dexel\""));
    let back: StreamManifest = serde_json::from_str(&json).unwrap();
    assert!(back.stock.dexel().is_some());
}
