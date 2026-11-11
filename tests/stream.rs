use camlib::{motion::*, stock::*, stream::*, tool::*};
use std::{io::Cursor, sync::atomic::AtomicBool};

fn manifest() -> StreamManifest {
    StreamManifest {
        initial_stock: None,
        tool_change: None,
        rapid_retracts: false,
        turning: None,
        version: 1,
        run: RunIdentity {
            id: "run-1".into(),
            revision: "cad-sha-1".into(),
        },
        stock: StockSpec {
            version: 1,
            units: "mm".into(),
            min_mm: [0.; 3],
            max_mm: [20., 20., 10.],
            cell_size_mm: 0.2,
            max_cells: 1_048_576,
        }
        .into(),
        tools: vec![ToolAssembly {
            id: "t1".into(),
            cutter: Cutter {
                id: "cutter".into(),
                name: "4 mm flat".into(),
                shape: CutterShape::Flat,
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
        initial_position_mm: [3., 5., 6.],
        sweep: SweepOptions::default(),
        operations: vec![],
        verification: None,
    }
}
fn motions() -> Vec<Motion> {
    [
        Curve::Line {
            start: [3., 5., 6.],
            end: [17., 5., 6.],
        },
        Curve::Line {
            start: [17., 5., 6.],
            end: [17., 15., 5.],
        },
        Curve::Arc {
            start: [17., 15., 5.],
            end: [14., 18., 5.],
            center: [14., 15., 5.],
            plane: Plane::Xy,
            sweep_rad: std::f64::consts::FRAC_PI_2,
        },
        Curve::Helix {
            start: [14., 18., 5.],
            end: [14., 18., 4.],
            center: [14., 15., 5.],
            plane: Plane::Xy,
            sweep_rad: std::f64::consts::TAU,
        },
    ]
    .into_iter()
    .enumerate()
    .map(|(i, curve)| Motion {
        id: i as u64 + 10,
        operation: "pocket".into(),
        tool: "t1".into(),
        kind: MoveKind::Cut,
        curve,
        feed_mm_min: Some(1000.),
    })
    .collect()
}
fn frames(manifest: &StreamManifest) -> Vec<Vec<u8>> {
    let mut p = StreamProducer::new(manifest.clone()).unwrap();
    motions()
        .into_iter()
        .map(|m| {
            p.push_motion(m, DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
                .unwrap()
        })
        .collect()
}

fn control_manifest() -> StreamManifest {
    let mut m = manifest();
    m.version = 2;
    let mut second = m.tools[0].clone();
    second.id = "edge2".into();
    second.cutter.diameter_mm = 2.;
    m.tools.push(second);
    m
}
fn tool_event(tool: &str, edge: u32, pose: Vec3) -> Event {
    Event::ToolChange {
        tool: tool.into(),
        number: 1,
        length_register: edge,
        position_after_mm: pose,
    }
}
fn rpm(value: f64) -> Event {
    Event::Spindle {
        rpm: value,
        direction: SpindleDirection::Cw,
    }
}
fn setup_controls(p: &mut StreamProducer, r: &mut StreamReceiver, pose: Vec3) -> Vec<Vec<u8>> {
    [
        Event::WorkOffset { number: 1 },
        tool_event("t1", 1, pose),
        rpm(12000.),
    ]
    .into_iter()
    .map(|e| {
        let frame = p
            .push_control(e.clone(), DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
            .unwrap();
        assert_eq!(r.accept_control(&frame, DEFAULT_FRAME_LIMIT).unwrap(), e);
        frame
    })
    .collect()
}

#[test]
fn typed_controls_reset_pose_select_edges_and_never_remove_stock() {
    let m = control_manifest();
    let mut p = StreamProducer::new(m.clone()).unwrap();
    let mut r = StreamReceiver::new(m.clone()).unwrap();
    let initial = r.stock().unwrap().snapshot();
    let pose = [5., 5., 6.];
    setup_controls(&mut p, &mut r, pose);
    assert_eq!(r.completed(), 0);
    assert_eq!(r.position(), pose);
    assert_eq!(r.stock().unwrap().snapshot(), initial);
    let mut first = motions().remove(0);
    first.curve = Curve::Line {
        start: pose,
        end: [17., 5., 6.],
    };
    let chunk = p
        .push_motion(first.clone(), DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
        .unwrap();
    r.accept(&chunk, DEFAULT_FRAME_LIMIT).unwrap();
    assert_ne!(r.stock().unwrap().cells(), initial.cells);
    assert_eq!(r.stock().unwrap().snapshot().generation, 1);
    let after_first = r.stock().unwrap().snapshot();
    for event in [
        rpm(0.),
        Event::Coolant { mode: Coolant::Off },
        Event::Stop { optional: true },
        tool_event("edge2", 2, [3., 15., 8.]),
    ] {
        let frame = p
            .push_control(event.clone(), DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
            .unwrap();
        assert_eq!(
            decode_control(&m, &frame, DEFAULT_FRAME_LIMIT)
                .unwrap()
                .event,
            event
        );
        r.accept_control(&frame, DEFAULT_FRAME_LIMIT).unwrap();
        assert_eq!(r.stock().unwrap().snapshot(), after_first);
    }
    assert_eq!(r.position(), [3., 15., 8.]);
    assert_eq!(r.controls(), p.controls());
    let state = r.controls().unwrap();
    assert_eq!((state.cursor.count, state.unverified_tool_routes), (7, 2));
    assert_eq!(state.selected.as_ref().unwrap().length_register, 2);
    let frame = p
        .push_control(rpm(3000.), DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
        .unwrap();
    r.accept_control(&frame, DEFAULT_FRAME_LIMIT).unwrap();
    let second = Motion {
        id: 11,
        tool: "edge2".into(),
        curve: Curve::Line {
            start: r.position(),
            end: [17., 15., 8.],
        },
        ..first
    };
    r.accept(
        &p.push_motion(second, DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
            .unwrap(),
        DEFAULT_FRAME_LIMIT,
    )
    .unwrap();
    assert_eq!(r.stock().unwrap().snapshot(), p.stock().unwrap().snapshot());
    assert_eq!(r.stock().unwrap().snapshot().generation, 2);
}

#[test]
fn missing_noop_and_substituted_equal_count_control_prefixes_are_rejected() {
    let m = control_manifest();
    let mut p = StreamProducer::new(m.clone()).unwrap();
    let mut r = StreamReceiver::new(m.clone()).unwrap();
    setup_controls(&mut p, &mut r, m.initial_position_mm);
    let stop = p
        .push_control(
            Event::Stop { optional: true },
            DEFAULT_FRAME_LIMIT,
            &AtomicBool::new(false),
        )
        .unwrap();
    let chunk = p
        .push_motion(
            motions().remove(0),
            DEFAULT_FRAME_LIMIT,
            &AtomicBool::new(false),
        )
        .unwrap();
    let before = r.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap();
    assert!(r.accept(&chunk, DEFAULT_FRAME_LIMIT).is_err());
    assert_eq!(r.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap(), before);
    r.accept_control(&stop, DEFAULT_FRAME_LIMIT).unwrap();
    assert!(r.accept_control(&stop, DEFAULT_FRAME_LIMIT).is_err());
    r.accept(&chunk, DEFAULT_FRAME_LIMIT).unwrap();

    // Same count and final RPM/pose/tool, different actual history.
    let mut p = StreamProducer::new(m.clone()).unwrap();
    let mut q = StreamProducer::new(m.clone()).unwrap();
    let mut a = StreamReceiver::new(m.clone()).unwrap();
    let mut b = StreamReceiver::new(m.clone()).unwrap();
    setup_controls(&mut p, &mut a, m.initial_position_mm);
    setup_controls(&mut q, &mut b, m.initial_position_mm);
    for value in [9000., 12000.] {
        a.accept_control(
            &p.push_control(rpm(value), DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
                .unwrap(),
            DEFAULT_FRAME_LIMIT,
        )
        .unwrap();
    }
    for value in [8000., 12000.] {
        b.accept_control(
            &q.push_control(rpm(value), DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
                .unwrap(),
            DEFAULT_FRAME_LIMIT,
        )
        .unwrap();
    }
    assert_eq!(
        a.controls().unwrap().cursor.count,
        b.controls().unwrap().cursor.count
    );
    let end = p.finish(DEFAULT_FRAME_LIMIT).unwrap();
    assert!(b.accept_end(&end, DEFAULT_FRAME_LIMIT).is_err());
    assert!(!b.is_finished());
    // Query-cache acknowledgement carries its exact controls after the base cursor.
    assert_ne!(
        p.restored_cursor(DEFAULT_FRAME_LIMIT).unwrap(),
        q.restored_cursor(DEFAULT_FRAME_LIMIT).unwrap()
    );
    let chunk = q
        .push_motion(
            motions().remove(0),
            DEFAULT_FRAME_LIMIT,
            &AtomicBool::new(false),
        )
        .unwrap();
    assert!(a.accept(&chunk, DEFAULT_FRAME_LIMIT).is_err());
    assert!(!a.is_finished());
    a.accept_end(&end, DEFAULT_FRAME_LIMIT).unwrap();
}

#[test]
fn invalid_cancelled_and_oversized_control_transitions_preserve_the_entire_prefix() {
    use std::cell::Cell;
    struct CancelAtCommit(Cell<usize>);
    impl CancelSignal for CancelAtCommit {
        fn is_cancelled(&self) -> bool {
            let n = self.0.get() + 1;
            self.0.set(n);
            n >= 2
        }
    }
    let m = control_manifest();
    let mut p = StreamProducer::new(m.clone()).unwrap();
    let mut r = StreamReceiver::new(m.clone()).unwrap();
    setup_controls(&mut p, &mut r, m.initial_position_mm);
    let before = p.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap();
    for event in [
        tool_event("edge2", 2, [0., 0., 5.]),
        Event::WorkOffset { number: 2 },
        rpm(-1.),
        rpm(f64::NAN),
        Event::Motion {
            motion: motions().remove(0),
        },
    ] {
        assert!(
            p.push_control(event, DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
                .is_err()
        );
        assert_eq!(p.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap(), before);
    }
    for signal in [
        &AtomicBool::new(true) as &dyn CancelSignal,
        &CancelAtCommit(Cell::new(0)),
    ] {
        // CancelSignal is deliberately queried both before preparation and commit.
        struct Signal<'a>(&'a dyn CancelSignal);
        impl CancelSignal for Signal<'_> {
            fn is_cancelled(&self) -> bool {
                self.0.is_cancelled()
            }
        }
        assert!(
            p.push_control(
                Event::Stop { optional: false },
                DEFAULT_FRAME_LIMIT,
                &Signal(signal)
            )
            .is_err()
        );
        assert_eq!(p.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap(), before);
    }
    assert!(
        p.push_control(Event::Stop { optional: false }, 20, &AtomicBool::new(false))
            .is_err()
    );
    assert_eq!(p.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap(), before);
    let off = p
        .push_control(rpm(0.), DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
        .unwrap();
    r.accept_control(&off, DEFAULT_FRAME_LIMIT).unwrap();
    let before = p.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap();
    for event in [
        tool_event("unknown", 1, [0.; 3]),
        tool_event("t1", 0, [0.; 3]),
        tool_event("t1", 1, [f64::NAN; 3]),
        Event::WorkOffset { number: 0 },
    ] {
        assert!(
            p.push_control(event, DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
                .is_err()
        );
        assert_eq!(p.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap(), before);
    }
    let mut wrong = motions().remove(0);
    wrong.tool = "edge2".into();
    assert!(
        p.push_motion(wrong, DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
            .is_err()
    );
    assert!(
        p.push_motion(
            motions().remove(0),
            DEFAULT_FRAME_LIMIT,
            &AtomicBool::new(false)
        )
        .is_err()
    );
    let mut rapid = motions().remove(0);
    rapid.kind = MoveKind::Rapid;
    rapid.feed_mm_min = None;
    p.push_motion(rapid, DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
        .unwrap();
}

#[test]
fn version2_checkpoint_replays_initial_and_later_transfers_and_partial_sweeps() {
    let m = control_manifest();
    let mut p = StreamProducer::new(m.clone()).unwrap();
    let mut r = StreamReceiver::new(m.clone()).unwrap();
    let initial = r.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap();
    let pose = [4., 5., 6.];
    setup_controls(&mut p, &mut r, pose);
    let before_move = r.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap();
    let restored = StreamReceiver::from_checkpoint_reader(
        m.clone(),
        &mut Cursor::new(&before_move),
        CHECKPOINT_FRAME_LIMIT,
    )
    .unwrap();
    assert_eq!(restored.completed(), 0);
    assert_eq!(restored.position(), pose);
    assert_eq!(restored.controls(), r.controls());
    let mut first = motions().remove(0);
    first.curve = Curve::Line {
        start: pose,
        end: [17., 5., 6.],
    };
    r.accept(
        &p.push_motion(first, DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
            .unwrap(),
        DEFAULT_FRAME_LIMIT,
    )
    .unwrap();
    let after_move = r.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap();
    let mut transfers = vec![];
    for event in [rpm(0.), tool_event("edge2", 2, [3., 15., 8.]), rpm(4000.)] {
        let frame = p
            .push_control(event, DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
            .unwrap();
        r.accept_control(&frame, DEFAULT_FRAME_LIMIT).unwrap();
        transfers.push(frame);
    }
    let next = Motion {
        id: 11,
        operation: "edge2-pass".into(),
        tool: "edge2".into(),
        kind: MoveKind::Cut,
        curve: Curve::Line {
            start: r.position(),
            end: [17., 15., 8.],
        },
        feed_mm_min: Some(1000.),
    };
    let mut query = StreamProducer::query_from_checkpoint_reader(
        m.clone(),
        &mut Cursor::new(&after_move),
        CHECKPOINT_FRAME_LIMIT,
    )
    .unwrap();
    assert!(
        query
            .preview_motion(&next, 0.4, DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
            .is_err()
    );
    for frame in &transfers {
        query
            .accept_cached_chunk(frame, DEFAULT_FRAME_LIMIT)
            .unwrap();
    }
    assert_eq!(query.position(), r.position());
    assert_eq!(query.controls(), r.controls());
    for fraction in [0., 0.371, 0.5, 1.] {
        let partial = r
            .preview_partial(&next, fraction, &AtomicBool::new(false))
            .unwrap();
        let checkpoint = r.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap();
        let mut reload =
            StreamReceiver::from_checkpoint(m.clone(), &checkpoint, CHECKPOINT_FRAME_LIMIT)
                .unwrap();
        let again = reload
            .preview_partial(&next, fraction, &AtomicBool::new(false))
            .unwrap();
        assert_eq!(partial.changes(), again.changes());
        assert_eq!(partial.base_generation(), again.base_generation());
        assert_eq!(
            p.preview_motion(
                &next,
                fraction,
                DEFAULT_FRAME_LIMIT,
                &AtomicBool::new(false)
            )
            .unwrap(),
            query
                .preview_motion(
                    &next,
                    fraction,
                    DEFAULT_FRAME_LIMIT,
                    &AtomicBool::new(false)
                )
                .unwrap()
        );
        reload
            .restore_checkpoint(&initial, CHECKPOINT_FRAME_LIMIT)
            .unwrap();
        assert_eq!(reload.controls(), Some(&ControlState::default()));
        assert_eq!(reload.position(), m.initial_position_mm);
    }
    assert!(
        query
            .push_control(rpm(0.), DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
            .is_err()
    );
    assert!(query.finish(DEFAULT_FRAME_LIMIT).is_err());
}

fn reframe(original: &[u8], body: &[u8]) -> Vec<u8> {
    let mut bytes = original[..20].to_vec();
    bytes[12..16].copy_from_slice(&(body.len() as u32).to_le_bytes());
    bytes.extend_from_slice(body);
    // Independent bit-at-a-time checksum, not the production table/helper.
    let mut crc = !0u32;
    for &b in bytes[..16].iter().chain(&bytes[20..]) {
        crc ^= b as u32;
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xedb88320
            } else {
                crc >> 1
            };
        }
    }
    bytes[16..20].copy_from_slice(&(!crc).to_le_bytes());
    bytes
}
fn identity_size(m: &StreamManifest) -> usize {
    2 + m.run.id.len() + 2 + m.run.revision.len() + 32
}

#[test]
fn corrupt_foreign_and_invalid_control_frames_reject_atomically() {
    let m = control_manifest();
    let mut p = StreamProducer::new(m.clone()).unwrap();
    let frame = p
        .push_control(
            Event::WorkOffset { number: 1 },
            DEFAULT_FRAME_LIMIT,
            &AtomicBool::new(false),
        )
        .unwrap();
    let mut r = StreamReceiver::new(m.clone()).unwrap();
    let initial = r.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap();
    let parsed = decode_control(&m, &frame, DEFAULT_FRAME_LIMIT).unwrap();
    let mut candidates = vec![frame[..frame.len() - 1].to_vec()];
    let mut bad = frame.clone();
    *bad.last_mut().unwrap() ^= 1;
    candidates.push(bad);
    for field in 0..4 {
        let mut changed = parsed.clone();
        match field {
            0 => changed.completed = 1,
            1 => changed.before.count = 1,
            2 => changed.before.chain_sha256[0] ^= 1,
            _ => changed.position_before[0] += 1.,
        }
        candidates.push(encode_control(&m, &changed, DEFAULT_FRAME_LIMIT).unwrap());
    }
    let json_start = identity_size(&m) + 8 + 40 + 24;
    for raw in [
        b"{\"kind\":\"work_offset\",\"number\":1,\"unknown\":0}".to_vec(),
        serde_json::to_vec(&Event::Motion {
            motion: motions().remove(0),
        })
        .unwrap(),
        vec![b' '; 4097],
    ] {
        let mut body = frame[20..20 + json_start].to_vec();
        body.extend_from_slice(&(raw.len() as u32).to_le_bytes());
        body.extend_from_slice(&raw);
        candidates.push(reframe(&frame, &body));
    }
    let mut other = m.clone();
    other.run.id = "other-run".into();
    candidates.push(encode_control(&other, &parsed, DEFAULT_FRAME_LIMIT).unwrap());
    for bytes in candidates {
        assert!(r.accept_control(&bytes, DEFAULT_FRAME_LIMIT).is_err());
        assert_eq!(r.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap(), initial);
    }
    r.accept_control(&frame, DEFAULT_FRAME_LIMIT).unwrap();
    assert!(r.accept_control(&frame, DEFAULT_FRAME_LIMIT).is_err());
    assert_eq!(r.controls().unwrap().cursor.count, 1);
}

#[test]
fn control_checkpoint_json_has_bounded_file_reads_and_rejects_invalid_state() {
    struct BoundedReader(Cursor<Vec<u8>>);
    impl std::io::Read for BoundedReader {
        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            assert!(buf.len() <= 65536);
            std::io::Read::read(&mut self.0, buf)
        }
    }
    let m = control_manifest();
    let mut p = StreamProducer::new(m.clone()).unwrap();
    let mut r = StreamReceiver::new(m.clone()).unwrap();
    setup_controls(&mut p, &mut r, [4., 5., 6.]);
    let checkpoint = r.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap();
    let loaded = StreamReceiver::from_checkpoint_reader(
        m.clone(),
        &mut BoundedReader(Cursor::new(checkpoint.clone())),
        CHECKPOINT_FRAME_LIMIT,
    )
    .unwrap();
    assert_eq!(
        loaded.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap(),
        checkpoint
    );
    let offset = identity_size(&m) + 8 + 24 + 1 + 8 + 8 + 4;
    let size =
        u32::from_le_bytes(checkpoint[20 + offset..24 + offset].try_into().unwrap()) as usize;
    let state: serde_json::Value =
        serde_json::from_slice(&checkpoint[24 + offset..24 + offset + size]).unwrap();
    for field in 0..7 {
        let mut bad = state.clone();
        match field {
            0 => bad["cursor"]["count"] = 0.into(),
            1 => bad["selected"]["tool"] = "unknown".into(),
            2 => bad["selected"]["position_after_mm"][0] = 9.into(),
            3 => bad["unverified_tool_routes"] = 99.into(),
            4 => bad["work_offset"] = 0.into(),
            5 => bad["spindle_rpm"] = (-1).into(),
            _ => bad["unknown"] = 0.into(),
        }
        let raw = serde_json::to_vec(&bad).unwrap();
        let mut body = checkpoint[20..20 + offset].to_vec();
        body.extend_from_slice(&(raw.len() as u32).to_le_bytes());
        body.extend_from_slice(&raw);
        body.extend_from_slice(&checkpoint[24 + offset + size..]);
        let forged = reframe(&checkpoint, &body);
        assert!(
            r.restore_checkpoint(&forged, CHECKPOINT_FRAME_LIMIT)
                .is_err()
        );
        assert_eq!(r.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap(), checkpoint);
        assert!(
            StreamReceiver::from_checkpoint_reader(
                m.clone(),
                &mut Cursor::new(&forged),
                CHECKPOINT_FRAME_LIMIT
            )
            .is_err()
        );
    }
    let mut body = checkpoint[20..].to_vec();
    body[offset..offset + 4].copy_from_slice(&4097u32.to_le_bytes());
    let too_large = reframe(&checkpoint, &body);
    assert!(
        StreamReceiver::from_checkpoint_reader(
            m.clone(),
            &mut Cursor::new(&too_large),
            CHECKPOINT_FRAME_LIMIT
        )
        .is_err()
    );
    for length in [0, 19, 24 + offset, checkpoint.len() - 1] {
        assert!(
            StreamReceiver::from_checkpoint_reader(
                m.clone(),
                &mut Cursor::new(&checkpoint[..length]),
                CHECKPOINT_FRAME_LIMIT
            )
            .is_err()
        );
    }
}

#[test]
fn control_only_initial_and_trailing_states_finish_without_stock_generation_changes() {
    let m = control_manifest();
    let mut p = StreamProducer::new(m.clone()).unwrap();
    let mut r = StreamReceiver::new(m.clone()).unwrap();
    let initial = r.stock().unwrap().snapshot();
    for e in [
        Event::Stop { optional: false },
        tool_event("t1", 1, [100., 100., 50.]),
        tool_event("edge2", 2, [-100., -100., 20.]),
        Event::Coolant { mode: Coolant::Off },
    ] {
        let frame = p
            .push_control(e, DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
            .unwrap();
        r.accept_control(&frame, DEFAULT_FRAME_LIMIT).unwrap();
    }
    let end = p.finish(DEFAULT_FRAME_LIMIT).unwrap();
    r.accept_end(&end, DEFAULT_FRAME_LIMIT).unwrap();
    assert!(r.is_finished());
    assert_eq!(r.completed(), 0);
    assert_eq!(r.stock().unwrap().snapshot(), initial);
    assert_eq!(r.position(), [-100., -100., 20.]);
    assert_eq!(r.controls().unwrap().unverified_tool_routes, 2);
    assert!(
        p.push_control(rpm(0.), DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
            .is_err()
    );
    let mut legacy = manifest();
    let unnegotiated = ControlChunk {
        completed: 0,
        before: ControlCursor::default(),
        position_before: legacy.initial_position_mm,
        event: Event::Stop { optional: false },
    };
    assert!(encode_control(&legacy, &unnegotiated, DEFAULT_FRAME_LIMIT).is_err());
    // Version1 still emits the exact old layout with no control field.
    legacy.run.id = "old-stream".into();
    let mut old = StreamProducer::new(legacy.clone()).unwrap();
    assert!(
        old.push_control(rpm(0.), DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
            .is_err()
    );
    let chunk = old
        .push_motion(
            motions().remove(0),
            DEFAULT_FRAME_LIMIT,
            &AtomicBool::new(false),
        )
        .unwrap();
    assert!(
        decode_chunk(&legacy, &chunk, DEFAULT_FRAME_LIMIT)
            .unwrap()
            .control_cursor
            .is_none()
    );
}

fn checked_manifest() -> StreamManifest {
    use camlib::{material, planar::*, stream_verification::*, verification};
    let mut m = manifest();
    m.verification = Some(Config {
        stock_contact: None,
        scene: Some(SceneCheck {
            scene: verification::Scene {
                version: 1,
                units: "mm".into(),
                id: "scene".into(),
                revision: "scene-1".into(),
                work_to_machine: WorkFrame::identity(),
                bodies: vec![],
                machine: Some(verification::MachineLimits {
                    min_tip_mm: Some([-100.; 3]),
                    max_tip_mm: Some([100.; 3]),
                    max_feed_mm_min: 500.,
                    max_normal_accel_mm_s2: None,
                    max_axis_feed_mm_min: None,
                    min_gauge_mm: None,
                    max_gauge_mm: None,
                }),
            },
            options: verification::Options::default(),
        }),
        material: vec![MaterialCheck {
            after_operation: None,
            target: material::Target {
                version: 1,
                units: "mm".into(),
                id: "target".into(),
                revision: "target-1".into(),
                floor_z_mm: 5.,
                floor_error_mm: 0.,
                machining_region: PlanarRegion {
                    version: 1,
                    units: "mm".into(),
                    name: "target".into(),
                    boundary_error_mm: 0.,
                    source: serde_json::Value::Null,
                    components: vec![Component {
                        outer: Boundary {
                            vertices: vec![
                                [-1., -1., 0.],
                                [21., -1., 0.],
                                [21., 21., 0.],
                                [-1., 21., 0.],
                            ],
                        },
                        holes: vec![],
                    }],
                },
            }
            .into(),
            options: material::Options {
                max_findings: 8,
                ..Default::default()
            },
        }],
    });
    m
}

#[test]
fn configured_checks_stream_after_matching_stock_and_keep_scope_explicit() {
    use camlib::stream_verification::{Binding, Check, Tracker};
    let m = checked_manifest();
    let mut p = StreamProducer::new(m.clone()).unwrap();
    let mut r = StreamReceiver::new(m.clone()).unwrap();
    let mut t = Tracker::new(std::sync::Arc::new(Binding::new(&m).unwrap()), 0);
    for motion in motions() {
        let (chunk, check) = p
            .push_checked_motion(motion.clone(), DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
            .unwrap();
        let check = check.unwrap();
        assert!(t.accept(&check, r.completed(), Some(&motion)).is_err());
        r.accept(&chunk, DEFAULT_FRAME_LIMIT).unwrap();
        let before = r.stock().unwrap().snapshot();
        let result = t.accept(&check, r.completed(), Some(&motion)).unwrap();
        let Check::Motion { report, .. } = result.check else {
            panic!("wrong report");
        };
        let report = report.unwrap();
        assert!(!report.coverage.evolving_stock_contact && !report.coverage.controller_execution);
        assert!(t.accept(&check, r.completed(), Some(&motion)).is_err());
        assert_eq!(before, r.stock().unwrap().snapshot());
    }
    assert_eq!(
        (t.progress.clear, t.progress.violation, t.progress.uncertain),
        (0, 4, 0)
    );
    assert!(p.finish(DEFAULT_FRAME_LIMIT).is_err());
    let start = p.start_material_verification(None).unwrap().unwrap();
    t.accept(&start, 4, None).unwrap();
    assert!(t.finish(4).is_err());
    assert!(p.start_material_verification(None).is_err());
    let bytes = p
        .finish_material_verification(&AtomicBool::new(false))
        .unwrap();
    let record = t.accept(&bytes, 4, None).unwrap();
    let Check::Material { report, .. } = record.check else {
        panic!("wrong report");
    };
    assert!(report.complete);
    assert_eq!(report.context.generation, 4);
    assert_eq!(report.context.run.unwrap().run, m.run);
    assert!(report.residual.definite_cells > 0);
    t.finish(4).unwrap();
    r.accept_end(&p.finish(DEFAULT_FRAME_LIMIT).unwrap(), DEFAULT_FRAME_LIMIT)
        .unwrap();
    assert!(r.is_finished());
}

#[test]
fn configured_motion_failure_cancellation_and_frame_budget_do_not_commit_checks_or_stock() {
    let m = checked_manifest();
    let mut p = StreamProducer::new(m.clone()).unwrap();
    let original = p.stock().unwrap().snapshot();
    assert!(
        p.push_motion(
            motions()[0].clone(),
            DEFAULT_FRAME_LIMIT,
            &AtomicBool::new(false)
        )
        .is_err()
    );
    assert!(
        p.push_checked_motion(
            motions()[0].clone(),
            DEFAULT_FRAME_LIMIT,
            &AtomicBool::new(true)
        )
        .is_err()
    );
    assert!(
        p.push_checked_motion(motions()[0].clone(), 1024, &AtomicBool::new(false))
            .is_err()
    );
    assert_eq!(p.stock().unwrap().snapshot(), original);
    let (chunk, check) = p
        .push_checked_motion(
            motions()[0].clone(),
            DEFAULT_FRAME_LIMIT,
            &AtomicBool::new(false),
        )
        .unwrap();
    let mut t = camlib::stream_verification::Tracker::new(
        std::sync::Arc::new(camlib::stream_verification::Binding::new(&m).unwrap()),
        0,
    );
    let mut r = StreamReceiver::new(m).unwrap();
    let motion = r.accept(&chunk, DEFAULT_FRAME_LIMIT).unwrap();
    t.accept(&check.unwrap(), 1, Some(&motion)).unwrap();
    assert_eq!(t.progress.records, 1);
    let start = p.start_material_verification(None).unwrap().unwrap();
    t.accept(&start, 1, None).unwrap();
    assert!(
        p.finish_material_verification(&AtomicBool::new(true))
            .is_err()
    );
    assert!(p.finish(DEFAULT_FRAME_LIMIT).is_err());
    let material = p
        .finish_material_verification(&AtomicBool::new(false))
        .unwrap();
    let record = t.accept(&material, 1, None).unwrap();
    let camlib::stream_verification::Check::Material { sweep_proof, .. } = record.check else {
        panic!("material report expected");
    };
    let proof = sweep_proof.unwrap();
    assert_eq!(proof.proven_moves + proof.unproven_moves, 1);
    assert_eq!(t.progress.material_reports, 1);
}

#[test]
fn report_binding_rejects_changed_check_inputs_run_and_source_motion() {
    use camlib::stream_verification::Binding;
    let m = checked_manifest();
    let mut p = StreamProducer::new(m.clone()).unwrap();
    let motion = motions()[0].clone();
    let (_, check) = p
        .push_checked_motion(motion.clone(), DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
        .unwrap();
    let check = check.unwrap();
    for variant in 0..4 {
        let mut changed = m.clone();
        match variant {
            0 => changed.run.id = "new-run".into(),
            1 => {
                changed
                    .verification
                    .as_mut()
                    .unwrap()
                    .scene
                    .as_mut()
                    .unwrap()
                    .options
                    .max_nodes -= 1
            }
            2 => {
                let camlib::material::Definition::Prismatic(target) =
                    &mut changed.verification.as_mut().unwrap().material[0].target
                else {
                    panic!("expected prismatic fixture")
                };
                target.floor_z_mm += 1.;
            }
            _ => {
                changed
                    .verification
                    .as_mut()
                    .unwrap()
                    .scene
                    .as_mut()
                    .unwrap()
                    .scene
                    .revision = "new-scene".into()
            }
        }
        assert!(
            Binding::new(&changed)
                .unwrap()
                .decode(&check, 1, Some(&motion))
                .is_err()
        );
    }
    let binding = Binding::new(&m).unwrap();
    let mut changed_motion = motion.clone();
    changed_motion.feed_mm_min = Some(499.);
    assert!(binding.decode(&check, 1, Some(&changed_motion)).is_err());
    assert!(binding.decode(&check, 1, None).is_err());
    let mut corrupt = check.clone();
    *corrupt.last_mut().unwrap() ^= 1;
    assert!(binding.decode(&corrupt, 1, Some(&motion)).is_err());
    binding.decode(&check, 1, Some(&motion)).unwrap();
}

#[test]
fn resumed_checks_report_only_the_newly_examined_suffix() {
    use camlib::stream_verification::{Binding, Tracker};
    let m = checked_manifest();
    let mut p = StreamProducer::new(m.clone()).unwrap();
    p.push_checked_motion(
        motions()[0].clone(),
        DEFAULT_FRAME_LIMIT,
        &AtomicBool::new(false),
    )
    .unwrap();
    let checkpoint = p.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap();
    let mut resumed =
        StreamProducer::from_checkpoint(m.clone(), &checkpoint, CHECKPOINT_FRAME_LIMIT).unwrap();
    let (_, check) = resumed
        .push_checked_motion(
            motions()[1].clone(),
            DEFAULT_FRAME_LIMIT,
            &AtomicBool::new(false),
        )
        .unwrap();
    let mut tracker = Tracker::new(std::sync::Arc::new(Binding::new(&m).unwrap()), 1);
    let record = tracker
        .accept(&check.unwrap(), 2, Some(&motions()[1]))
        .unwrap();
    assert_eq!(record.progress.from_completed, 1);
    assert_eq!(record.progress.through_completed, 2);
    assert_eq!(record.progress.violation, 1);
}

#[test]
fn material_only_check_uses_actual_final_stock_and_default_manifest_is_unchanged() {
    let plain = manifest();
    let bytes = plain.canonical_bytes().unwrap();
    assert!(
        !String::from_utf8(bytes.clone())
            .unwrap()
            .contains("verification")
    );
    let roundtrip: StreamManifest = serde_json::from_slice(&bytes).unwrap();
    assert_eq!(roundtrip.canonical_bytes().unwrap(), bytes);
    let mut m = checked_manifest();
    m.verification.as_mut().unwrap().scene = None;
    let mut p = StreamProducer::new(m).unwrap();
    for motion in motions() {
        let (_, check) = p
            .push_checked_motion(motion, DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
            .unwrap();
        assert!(check.is_none());
    }
    assert!(p.start_material_verification(None).unwrap().is_some());
    p.finish_material_verification(&AtomicBool::new(false))
        .unwrap();
    p.finish(DEFAULT_FRAME_LIMIT).unwrap();
}

#[test]
fn read_only_query_skips_verifiers_and_stock_index_but_keeps_checked_cache_identity() {
    let mut manifest = checked_manifest();
    manifest.verification.as_mut().unwrap().stock_contact =
        Some(camlib::stock_contact::Options::default());
    let mut writer = StreamProducer::new(manifest.clone()).unwrap();
    let mut query = StreamProducer::new_query(manifest.clone()).unwrap();
    assert!(writer.stock().unwrap().contact_index_stats().is_some());
    assert!(query.stock().unwrap().contact_index_stats().is_none());
    let cancel = AtomicBool::new(false);
    assert!(
        query
            .push_checked_motion(motions()[0].clone(), DEFAULT_FRAME_LIMIT, &cancel)
            .is_err()
    );
    assert!(
        query
            .push_motion(motions()[0].clone(), DEFAULT_FRAME_LIMIT, &cancel)
            .is_err()
    );
    assert!(query.finish(DEFAULT_FRAME_LIMIT).is_err());
    assert!(query.start_material_verification(None).is_err());
    let (chunk, _) = writer
        .push_checked_motion(motions()[0].clone(), DEFAULT_FRAME_LIMIT, &cancel)
        .unwrap();
    query
        .accept_cached_chunk(&chunk, DEFAULT_FRAME_LIMIT)
        .unwrap();
    assert_eq!(
        query.stock().unwrap().cells(),
        writer.stock().unwrap().cells()
    );
    assert!(query.stock().unwrap().contact_index_stats().is_none());
    let checkpoint = writer.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap();
    query
        .restore_checkpoint(&checkpoint, CHECKPOINT_FRAME_LIMIT)
        .unwrap();
    assert!(query.stock().unwrap().contact_index_stats().is_none());
    let loaded = StreamProducer::query_from_checkpoint_reader(
        manifest,
        &mut Cursor::new(checkpoint),
        CHECKPOINT_FRAME_LIMIT,
    )
    .unwrap();
    assert!(loaded.stock().unwrap().contact_index_stats().is_none());
    assert_eq!(loaded.completed(), 1);
    query
        .preview_motion(&motions()[1], 0.25, DEFAULT_FRAME_LIMIT, &cancel)
        .unwrap();
}

#[test]
fn binary_chunks_commit_exact_same_stock_and_source_motion() {
    let manifest = manifest();
    let mut producer = StreamProducer::new(manifest.clone()).unwrap();
    let mut receiver = StreamReceiver::new(manifest).unwrap();
    let mut payload = Vec::new();
    for motion in motions() {
        let bytes = producer
            .push_motion(motion.clone(), DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
            .unwrap();
        let decoded = receiver.accept(&bytes, DEFAULT_FRAME_LIMIT).unwrap();
        assert_eq!(decoded, motion);
        assert_eq!(
            producer.stock().unwrap().snapshot(),
            receiver.stock().unwrap().snapshot()
        );
        assert_eq!(producer.completed(), receiver.completed());
        payload.extend_from_slice(&bytes);
    }
    assert_eq!(receiver.completed(), 4);
    assert!(receiver.stock().unwrap().volume().removed_lower_mm3 > 0.);
    let mut pipe = Cursor::new(payload);
    for _ in 0..4 {
        assert!(
            read_frame(&mut pipe, DEFAULT_FRAME_LIMIT)
                .unwrap()
                .is_some()
        );
    }
    assert!(
        read_frame(&mut pipe, DEFAULT_FRAME_LIMIT)
            .unwrap()
            .is_none()
    );
}

#[test]
fn missing_repeated_stale_and_wrong_manifest_frames_preserve_receiver() {
    let manifest = manifest();
    let chunks = frames(&manifest);
    let mut receiver = StreamReceiver::new(manifest.clone()).unwrap();
    let initial = receiver.stock().unwrap().snapshot();
    assert!(receiver.accept(&chunks[1], DEFAULT_FRAME_LIMIT).is_err());
    assert_eq!(receiver.stock().unwrap().snapshot(), initial);
    receiver.accept(&chunks[0], DEFAULT_FRAME_LIMIT).unwrap();
    let after = receiver.stock().unwrap().snapshot();
    assert!(receiver.accept(&chunks[0], DEFAULT_FRAME_LIMIT).is_err());
    assert_eq!(receiver.stock().unwrap().snapshot(), after);
    for case in 0..4 {
        let mut other = manifest.clone();
        match case {
            0 => other.run.id = "new-run".into(),
            1 => other.run.revision = "new-cad-revision".into(),
            2 => other.stock.height_mut().unwrap().cell_size_mm = 0.4,
            _ => other.tools[0].cutter.diameter_mm = 3.,
        }
        let mut new = StreamReceiver::new(other).unwrap();
        let baseline = new.stock().unwrap().snapshot();
        let error = new.accept(&chunks[0], DEFAULT_FRAME_LIMIT).unwrap_err();
        assert!(error.contains("stale/foreign"), "{error}");
        assert_eq!(new.stock().unwrap().snapshot(), baseline);
        assert_eq!(new.completed(), 0);
    }
}

#[test]
fn checkpoint_seek_replay_and_partial_move_reconstruction_are_deterministic() {
    let manifest = manifest();
    let chunks = frames(&manifest);
    let mut receiver = StreamReceiver::new(manifest.clone()).unwrap();
    let empty = receiver.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap();
    receiver.accept(&chunks[0], DEFAULT_FRAME_LIMIT).unwrap();
    let checkpoint = receiver.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap();
    let at_one = receiver.stock().unwrap().snapshot();
    for fraction in [0., 0.371, 0.5, 1.] {
        let partial = receiver
            .preview_partial(&motions()[1], fraction, &AtomicBool::new(false))
            .unwrap();
        assert_eq!(receiver.stock().unwrap().snapshot(), at_one);
        let mut show = receiver.stock().unwrap().clone();
        show.apply_delta(&partial).unwrap();
        let shown = show.snapshot();
        receiver
            .restore_checkpoint(&checkpoint, CHECKPOINT_FRAME_LIMIT)
            .unwrap();
        let repeated = receiver
            .preview_partial(&motions()[1], fraction, &AtomicBool::new(false))
            .unwrap();
        let mut seek = receiver.stock().unwrap().clone();
        seek.apply_delta(&repeated).unwrap();
        assert_eq!(seek.snapshot(), shown);
    }
    for bytes in &chunks[1..] {
        receiver.accept(bytes, DEFAULT_FRAME_LIMIT).unwrap();
    }
    let final_stock = receiver.stock().unwrap().snapshot();
    receiver
        .restore_checkpoint(&checkpoint, CHECKPOINT_FRAME_LIMIT)
        .unwrap();
    assert_eq!(receiver.completed(), 1);
    assert_eq!(receiver.stock().unwrap().snapshot(), at_one);
    for bytes in &chunks[1..] {
        receiver.accept(bytes, DEFAULT_FRAME_LIMIT).unwrap();
    }
    assert_eq!(receiver.stock().unwrap().snapshot(), final_stock);
    receiver
        .restore_checkpoint(&empty, CHECKPOINT_FRAME_LIMIT)
        .unwrap();
    assert_eq!(receiver.completed(), 0);
    assert_eq!(receiver.position(), manifest.initial_position_mm);
}

#[test]
fn malformed_binary_lengths_crc_and_payloads_are_rejected_without_mutation() {
    let manifest = manifest();
    let chunks = frames(&manifest);
    let mut receiver = StreamReceiver::new(manifest.clone()).unwrap();
    let before = receiver.stock().unwrap().snapshot();
    let source = &chunks[0];
    for len in (0..source.len()).step_by(127).chain([source.len() - 1]) {
        assert!(
            receiver
                .accept(&source[..len], DEFAULT_FRAME_LIMIT)
                .is_err()
        );
        assert_eq!(receiver.stock().unwrap().snapshot(), before);
    }
    let mut bad = source.clone();
    bad[100] ^= 128;
    assert!(
        receiver
            .accept(&bad, DEFAULT_FRAME_LIMIT)
            .unwrap_err()
            .contains("checksum")
    );
    let mut extra = source.clone();
    extra.push(0);
    assert!(receiver.accept(&extra, DEFAULT_FRAME_LIMIT).is_err());
    let mut oversized = source[..20].to_vec();
    oversized[12..16].copy_from_slice(&u32::MAX.to_le_bytes());
    // A header claiming the maximum length with no payload is rejected as
    // truncated, without allocating the claimed 4 GiB (memory grows only with
    // bytes that arrive; this exact case once exhausted a test host).
    assert!(
        read_frame(&mut Cursor::new(oversized.clone()), DEFAULT_FRAME_LIMIT)
            .unwrap_err()
            .contains("truncated")
    );
    // An explicit caller limit is still enforced before reading.
    assert!(
        read_frame(&mut Cursor::new(oversized), 8 * 1024 * 1024)
            .unwrap_err()
            .contains("limit")
    );
    assert!(read_frame(&mut Cursor::new(&source[..19]), DEFAULT_FRAME_LIMIT).is_err());
    assert!(
        read_frame(
            &mut Cursor::new(&source[..source.len() - 1]),
            DEFAULT_FRAME_LIMIT
        )
        .is_err()
    );
    // Valid CRC/structure but wrong old cell value must still fail atomically.
    let mut forged = decode_chunk(&manifest, source, DEFAULT_FRAME_LIMIT).unwrap();
    let mut json = serde_json::to_value(&forged.delta).unwrap();
    json["changes"][1]["before"]["lower_mm"] = serde_json::json!(9.);
    forged.delta = serde_json::from_value(json).unwrap();
    let forged = encode_chunk(&manifest, &forged, DEFAULT_FRAME_LIMIT).unwrap();
    assert!(receiver.accept(&forged, DEFAULT_FRAME_LIMIT).is_err());
    assert_eq!(receiver.stock().unwrap().snapshot(), before);
}

#[test]
fn frame_budget_cancellation_and_discontinuous_motion_preserve_producer_prefix() {
    let manifest = manifest();
    let mut producer = StreamProducer::new(manifest).unwrap();
    let initial = producer.stock().unwrap().snapshot();
    let first = motions().remove(0);
    assert!(
        producer
            .push_motion(first.clone(), 128, &AtomicBool::new(false))
            .is_err()
    );
    assert!(
        producer
            .push_motion(first.clone(), DEFAULT_FRAME_LIMIT, &AtomicBool::new(true))
            .is_err()
    );
    assert_eq!(producer.stock().unwrap().snapshot(), initial);
    assert_eq!(producer.completed(), 0);
    producer
        .push_motion(first, DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
        .unwrap();
    let state = producer.stock().unwrap().snapshot();
    assert!(
        producer
            .push_motion(
                motions().remove(0),
                DEFAULT_FRAME_LIMIT,
                &AtomicBool::new(false)
            )
            .is_err()
    );
    assert_eq!(producer.stock().unwrap().snapshot(), state);
    assert_eq!(producer.completed(), 1);
}

#[test]
fn checkpoint_invalid_heights_rejected_even_with_valid_checksum() {
    let mut receiver = StreamReceiver::new(manifest()).unwrap();
    let initial = receiver.stock().unwrap().snapshot();
    let mut bad = receiver.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap();
    let len = bad.len();
    bad[len - 8..].copy_from_slice(&f64::NAN.to_le_bytes());
    // Independent bit-at-a-time CRC reference; production uses a lookup table.
    let mut crc = !0u32;
    for &b in bad[..16].iter().chain(&bad[20..]) {
        crc ^= b as u32;
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xedb88320
            } else {
                crc >> 1
            };
        }
    }
    bad[16..20].copy_from_slice(&(!crc).to_le_bytes());
    assert!(
        receiver
            .restore_checkpoint(&bad, CHECKPOINT_FRAME_LIMIT)
            .unwrap_err()
            .contains("height bounds")
    );
    assert_eq!(receiver.stock().unwrap().snapshot(), initial);
    assert_eq!(receiver.completed(), 0);
}

#[test]
fn explicit_finish_requires_the_entire_committed_prefix() {
    let manifest = manifest();
    let setup = encode_manifest(&manifest, DEFAULT_FRAME_LIMIT).unwrap();
    let decoded = decode_manifest(&setup, DEFAULT_FRAME_LIMIT).unwrap();
    assert_eq!(decoded.digest().unwrap(), manifest.digest().unwrap());
    let mut producer = StreamProducer::new(manifest.clone()).unwrap();
    let mut receiver = StreamReceiver::new(decoded).unwrap();
    let chunk = producer
        .push_motion(
            motions().remove(0),
            DEFAULT_FRAME_LIMIT,
            &AtomicBool::new(false),
        )
        .unwrap();
    let end = producer.finish(DEFAULT_FRAME_LIMIT).unwrap();
    assert!(!receiver.is_finished());
    assert!(receiver.accept_end(&end, DEFAULT_FRAME_LIMIT).is_err());
    receiver.accept(&chunk, DEFAULT_FRAME_LIMIT).unwrap();
    receiver.accept_end(&end, DEFAULT_FRAME_LIMIT).unwrap();
    assert!(receiver.is_finished());
    assert!(receiver.accept_end(&end, DEFAULT_FRAME_LIMIT).is_err());
    assert!(
        producer
            .push_motion(
                motions().remove(1),
                DEFAULT_FRAME_LIMIT,
                &AtomicBool::new(false)
            )
            .is_err()
    );
    assert!(producer.finish(DEFAULT_FRAME_LIMIT).is_err());
}

#[test]
fn native_process_emits_removed_stock_before_input_is_finished() {
    use std::{
        io::Write,
        process::{Command, Stdio},
    };
    let manifest = manifest();
    let mut child = Command::new(env!("CARGO_BIN_EXE_cam-sim-stream"))
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    let mut input = child.stdin.take().unwrap();
    let mut output = child.stdout.take().unwrap();
    serde_json::to_writer(&mut input, &manifest).unwrap();
    input.write_all(b"\n").unwrap();
    input.flush().unwrap();
    let setup = read_frame(&mut output, DEFAULT_FRAME_LIMIT)
        .unwrap()
        .unwrap();
    let mut receiver =
        StreamReceiver::new(decode_manifest(&setup, DEFAULT_FRAME_LIMIT).unwrap()).unwrap();
    let m = motions().remove(0);
    writeln!(
        input,
        "{}",
        serde_json::json!({"command":"motions","motions":[m]})
    )
    .unwrap();
    input.flush().unwrap();
    let chunk = read_frame(&mut output, DEFAULT_FRAME_LIMIT)
        .unwrap()
        .unwrap();
    receiver.accept(&chunk, DEFAULT_FRAME_LIMIT).unwrap();
    assert!(receiver.stock().unwrap().volume().removed_lower_mm3 > 0.);
    assert!(child.try_wait().unwrap().is_none()); // Producer is still awaiting further job work.
    assert!(!receiver.is_finished());
    input.write_all(b"{\"command\":\"finish\"}\n").unwrap();
    input.flush().unwrap();
    drop(input);
    let end = read_frame(&mut output, DEFAULT_FRAME_LIMIT)
        .unwrap()
        .unwrap();
    receiver.accept_end(&end, DEFAULT_FRAME_LIMIT).unwrap();
    assert!(receiver.is_finished());
    assert!(
        read_frame(&mut output, DEFAULT_FRAME_LIMIT)
            .unwrap()
            .is_none()
    );
    assert!(child.wait().unwrap().success());
}

#[test]
fn native_typed_event_process_streams_controls_removal_and_trailing_checkpoint() {
    use std::{
        io::Write,
        process::{Command, Stdio},
    };
    let m = control_manifest();
    let mut child = Command::new(env!("CARGO_BIN_EXE_cam-sim-stream"))
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    let mut input = child.stdin.take().unwrap();
    let mut output = child.stdout.take().unwrap();
    writeln!(input, "{}", serde_json::to_string(&m).unwrap()).unwrap();
    input.flush().unwrap();
    let header = read_frame(&mut output, DEFAULT_FRAME_LIMIT)
        .unwrap()
        .unwrap();
    assert_eq!(
        decode_manifest(&header, DEFAULT_FRAME_LIMIT)
            .unwrap()
            .version,
        2
    );
    let mut receiver = StreamReceiver::new(m.clone()).unwrap();
    let events = vec![
        Event::WorkOffset { number: 1 },
        tool_event("t1", 1, m.initial_position_mm),
        rpm(12000.),
        Event::Motion {
            motion: motions().remove(0),
        },
    ];
    writeln!(
        input,
        "{}",
        serde_json::json!({"command":"events","events":events})
    )
    .unwrap();
    input.flush().unwrap();
    for _ in 0..3 {
        receiver
            .accept_control(
                &read_frame(&mut output, DEFAULT_FRAME_LIMIT)
                    .unwrap()
                    .unwrap(),
                DEFAULT_FRAME_LIMIT,
            )
            .unwrap();
    }
    receiver
        .accept(
            &read_frame(&mut output, DEFAULT_FRAME_LIMIT)
                .unwrap()
                .unwrap(),
            DEFAULT_FRAME_LIMIT,
        )
        .unwrap();
    assert!(receiver.stock().unwrap().volume().removed_lower_mm3 > 0.);
    assert!(child.try_wait().unwrap().is_none());
    assert!(!receiver.is_finished());
    let second = Motion {
        id: 11,
        tool: "edge2".into(),
        curve: Curve::Line {
            start: [3., 15., 8.],
            end: [17., 15., 8.],
        },
        ..motions().remove(0)
    };
    let events = vec![
        rpm(0.),
        tool_event("edge2", 2, [3., 15., 8.]),
        rpm(4000.),
        Event::Motion { motion: second },
        Event::Stop { optional: true },
    ];
    writeln!(
        input,
        "{}",
        serde_json::json!({"command":"events","events":events})
    )
    .unwrap();
    input.flush().unwrap();
    let before = receiver.stock().unwrap().snapshot();
    for _ in 0..3 {
        receiver
            .accept_control(
                &read_frame(&mut output, DEFAULT_FRAME_LIMIT)
                    .unwrap()
                    .unwrap(),
                DEFAULT_FRAME_LIMIT,
            )
            .unwrap();
        assert_eq!(receiver.stock().unwrap().snapshot(), before);
    }
    receiver
        .accept(
            &read_frame(&mut output, DEFAULT_FRAME_LIMIT)
                .unwrap()
                .unwrap(),
            DEFAULT_FRAME_LIMIT,
        )
        .unwrap();
    receiver
        .accept_control(
            &read_frame(&mut output, DEFAULT_FRAME_LIMIT)
                .unwrap()
                .unwrap(),
            DEFAULT_FRAME_LIMIT,
        )
        .unwrap();
    writeln!(input, "{{\"command\":\"checkpoint\"}}").unwrap();
    input.flush().unwrap();
    let checkpoint = read_frame(&mut output, CHECKPOINT_FRAME_LIMIT)
        .unwrap()
        .unwrap();
    let restored = StreamReceiver::from_checkpoint(m, &checkpoint, CHECKPOINT_FRAME_LIMIT).unwrap();
    assert_eq!(restored.controls(), receiver.controls());
    assert_eq!(restored.completed(), 2);
    assert_eq!(
        restored.stock().unwrap().snapshot(),
        receiver.stock().unwrap().snapshot()
    );
    assert_eq!(restored.controls().unwrap().cursor.count, 7);
    writeln!(input, "{{\"command\":\"finish\"}}").unwrap();
    input.flush().unwrap();
    drop(input);
    receiver
        .accept_end(
            &read_frame(&mut output, DEFAULT_FRAME_LIMIT)
                .unwrap()
                .unwrap(),
            DEFAULT_FRAME_LIMIT,
        )
        .unwrap();
    assert!(
        read_frame(&mut output, DEFAULT_FRAME_LIMIT)
            .unwrap()
            .is_none()
    );
    assert!(child.wait().unwrap().success());
}

#[test]
fn native_query_replays_mixed_control_cache_and_returns_exact_prefix() {
    use std::{
        io::Write,
        process::{Command, Stdio},
    };
    let m = control_manifest();
    let mut p = StreamProducer::new(m.clone()).unwrap();
    let mut receiver = StreamReceiver::new(m.clone()).unwrap();
    setup_controls(&mut p, &mut receiver, m.initial_position_mm);
    let first = motions().remove(0);
    p.push_motion(first, DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
        .unwrap();
    let checkpoint = p.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap();
    let expected_restore = p.restored_cursor(DEFAULT_FRAME_LIMIT).unwrap();
    let mut cache = Vec::new();
    for e in [rpm(0.), tool_event("edge2", 2, [3., 15., 8.]), rpm(4000.)] {
        cache.extend(
            p.push_control(e, DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
                .unwrap(),
        );
    }
    let next = Motion {
        id: 11,
        tool: "edge2".into(),
        curve: Curve::Line {
            start: [3., 15., 8.],
            end: [17., 15., 8.],
        },
        ..motions().remove(0)
    };
    cache.extend(
        p.push_motion(next, DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
            .unwrap(),
    );
    let dir = std::env::temp_dir().join(format!(
        "camlib-transfer-query-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    std::fs::create_dir(&dir).unwrap();
    let cp = dir.join("checkpoint.bin");
    let history = dir.join("events.bin");
    std::fs::write(&cp, checkpoint).unwrap();
    std::fs::write(&history, &cache).unwrap();
    let mut child = Command::new(env!("CARGO_BIN_EXE_cam-sim-stream"))
        .arg("--query")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    let mut input = child.stdin.take().unwrap();
    let mut output = child.stdout.take().unwrap();
    writeln!(input, "{}", serde_json::to_string(&m).unwrap()).unwrap();
    input.flush().unwrap();
    read_frame(&mut output, DEFAULT_FRAME_LIMIT)
        .unwrap()
        .unwrap();
    writeln!(
        input,
        "{}",
        serde_json::json!({"command":"restore","path":cp})
    )
    .unwrap();
    input.flush().unwrap();
    assert_eq!(
        read_frame(&mut output, DEFAULT_FRAME_LIMIT)
            .unwrap()
            .unwrap(),
        expected_restore
    );
    writeln!(
        input,
        "{}",
        serde_json::json!({"command":"cache","path":history,"offset":0,"length":cache.len()})
    )
    .unwrap();
    input.flush().unwrap();
    assert_eq!(
        read_frame(&mut output, DEFAULT_FRAME_LIMIT)
            .unwrap()
            .unwrap(),
        p.restored_cursor(DEFAULT_FRAME_LIMIT).unwrap()
    );
    let next = Motion {
        id: 12,
        tool: "edge2".into(),
        curve: Curve::Line {
            start: [17., 15., 8.],
            end: [17., 5., 7.],
        },
        ..motions().remove(0)
    };
    writeln!(
        input,
        "{}",
        serde_json::json!({"command":"partial","motion":next,"fraction":0.371})
    )
    .unwrap();
    input.flush().unwrap();
    assert_eq!(
        read_frame(&mut output, DEFAULT_FRAME_LIMIT)
            .unwrap()
            .unwrap(),
        p.preview_motion(&next, 0.371, DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
            .unwrap()
    );
    // Query workers consume committed history but cannot generate control effects.
    writeln!(
        input,
        "{}",
        serde_json::json!({"command":"events","events":[rpm(0.)]})
    )
    .unwrap();
    input.flush().unwrap();
    drop(input);
    assert!(
        read_frame(&mut output, DEFAULT_FRAME_LIMIT)
            .unwrap()
            .is_none()
    );
    let result = child.wait_with_output().unwrap();
    assert!(!result.status.success());
    assert!(String::from_utf8_lossy(&result.stderr).contains("writable version 2"));
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn typed_control_routes_remain_separate_from_configured_motion_verification() {
    let mut m = checked_manifest();
    m.version = 2;
    let mut p = StreamProducer::new(m.clone()).unwrap();
    let mut r = StreamReceiver::new(m.clone()).unwrap();
    setup_controls(&mut p, &mut r, m.initial_position_mm);
    let motion = motions().remove(0);
    let (chunk, check) = p
        .push_checked_motion(motion.clone(), DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
        .unwrap();
    r.accept(&chunk, DEFAULT_FRAME_LIMIT).unwrap();
    let mut tracker = camlib::stream_verification::Tracker::new(
        std::sync::Arc::new(camlib::stream_verification::Binding::new(&m).unwrap()),
        0,
    );
    tracker.accept(&check.unwrap(), 1, Some(&motion)).unwrap();
    assert_eq!(tracker.progress.records, 1);
    assert_eq!(r.controls().unwrap().unverified_tool_routes, 1);
    let before = r.stock().unwrap().snapshot();
    let frame = p
        .push_control(rpm(0.), DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
        .unwrap();
    r.accept_control(&frame, DEFAULT_FRAME_LIMIT).unwrap();
    let frame = p
        .push_control(
            tool_event("t1", 2, [3., 15., 8.]),
            DEFAULT_FRAME_LIMIT,
            &AtomicBool::new(false),
        )
        .unwrap();
    r.accept_control(&frame, DEFAULT_FRAME_LIMIT).unwrap();
    assert_eq!(r.stock().unwrap().snapshot(), before);
    assert_eq!(r.controls().unwrap().unverified_tool_routes, 2);
    assert_eq!(tracker.progress.records, 1); // No false sweep report for either route.
}

#[test]
fn checkpoint_file_reader_uses_bounded_blocks_and_matches_byte_decoder() {
    use std::io::Read;
    struct SmallReads(std::io::Cursor<Vec<u8>>);
    impl Read for SmallReads {
        fn read(&mut self, target: &mut [u8]) -> std::io::Result<usize> {
            assert!(
                target.len() <= 65536,
                "checkpoint reader requested an unbounded block"
            );
            let count = target.len().min(7);
            self.0.read(&mut target[..count])
        }
    }
    let manifest = manifest();
    let mut producer = StreamProducer::new(manifest.clone()).unwrap();
    for motion in motions().into_iter().take(2) {
        producer
            .push_motion(motion, DEFAULT_FRAME_LIMIT, &AtomicBool::new(false))
            .unwrap();
    }
    let checkpoint = producer.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap();
    let decoded =
        StreamReceiver::from_checkpoint(manifest.clone(), &checkpoint, CHECKPOINT_FRAME_LIMIT)
            .unwrap();
    let mut reader = SmallReads(Cursor::new(checkpoint.clone()));
    let streamed = StreamReceiver::from_checkpoint_reader(
        manifest.clone(),
        &mut reader,
        CHECKPOINT_FRAME_LIMIT,
    )
    .unwrap();
    assert_eq!(
        streamed.stock().unwrap().snapshot(),
        decoded.stock().unwrap().snapshot()
    );
    assert_eq!(streamed.position(), decoded.position());
    for size in [0, 19, checkpoint.len() - 1] {
        assert!(
            StreamReceiver::from_checkpoint_reader(
                manifest.clone(),
                &mut Cursor::new(&checkpoint[..size]),
                CHECKPOINT_FRAME_LIMIT
            )
            .is_err()
        );
    }
    let mut corrupted = checkpoint.clone();
    let last = corrupted.len() - 1;
    corrupted[last] ^= 128;
    assert!(
        StreamReceiver::from_checkpoint_reader(
            manifest.clone(),
            &mut Cursor::new(corrupted),
            CHECKPOINT_FRAME_LIMIT
        )
        .err()
        .unwrap()
        .contains("checksum")
    );
    let mut extra = checkpoint;
    extra.push(0);
    assert!(
        StreamReceiver::from_checkpoint_reader(
            manifest,
            &mut Cursor::new(extra),
            CHECKPOINT_FRAME_LIMIT
        )
        .is_err()
    );
}

#[test]
fn explicit_motion_material_stages_bind_committed_cursor_and_require_all_reports() {
    let mut m = checked_manifest();
    let config = m.verification.as_mut().unwrap();
    config.scene = None;
    let final_check = config.material[0].clone();
    config.material[0].after_operation = Some("rough".into());
    config.material.push(final_check);
    let mut p = StreamProducer::new(m.clone()).unwrap();
    let moves = motions();
    let cancel = AtomicBool::new(false);
    for motion in &moves[..2] {
        p.push_checked_motion(motion.clone(), DEFAULT_FRAME_LIMIT, &cancel)
            .unwrap();
    }
    assert!(
        p.start_material_verification(Some("unknown"))
            .unwrap()
            .is_none()
    );
    let started = p
        .start_material_verification(Some("rough"))
        .unwrap()
        .unwrap();
    assert!(
        p.push_checked_motion(moves[2].clone(), DEFAULT_FRAME_LIMIT, &cancel)
            .is_err()
    );
    assert!(
        p.finish_material_verification(&AtomicBool::new(true))
            .is_err()
    );
    assert_eq!(p.completed(), 2);
    let finished = p.finish_material_verification(&cancel).unwrap();
    let mut tracker = camlib::stream_verification::Tracker::new(
        std::sync::Arc::new(camlib::stream_verification::Binding::new(&m).unwrap()),
        0,
    );
    tracker.accept(&started, 2, None).unwrap();
    tracker.accept(&finished, 2, None).unwrap();
    assert!(
        tracker
            .accept(&started, 2, None)
            .unwrap_err()
            .contains("repeated material stage")
    );
    assert!(tracker.accept(&finished, 2, None).is_err());
    assert!(
        p.start_material_verification(Some("rough"))
            .unwrap()
            .is_none()
    );
    for motion in &moves[2..] {
        p.push_checked_motion(motion.clone(), DEFAULT_FRAME_LIMIT, &cancel)
            .unwrap();
    }
    assert!(p.finish(DEFAULT_FRAME_LIMIT).is_err());
    assert!(p.start_material_verification(None).unwrap().is_some());
    p.finish_material_verification(&cancel).unwrap();
    p.finish(DEFAULT_FRAME_LIMIT).unwrap();
}

#[test]
fn finishing_material_stage_binds_scope_and_incomplete_access_without_hiding_raw_checks() {
    use camlib::{material::*, stream_verification::Binding, surface::SurfaceMesh};
    for budget in [1, 100000] {
        let mut m = checked_manifest();
        m.stock.height_mut().unwrap().max_mm = [2., 2., 6.];
        m.stock.height_mut().unwrap().cell_size_mm = 0.5;
        let config = m.verification.as_mut().unwrap();
        config.scene = None;
        config.material[0].target = SurfaceTarget {
            version: 1,
            units: "mm".into(),
            id: "surface".into(),
            revision: "1".into(),
            surface: SurfaceMesh {
                version: 1,
                units: "mm".into(),
                triangles: vec![
                    [[-1., -1., 2.], [3., -1., 2.], [3., 3., 2.]],
                    [[-1., -1., 2.], [3., 3., 2.], [-1., 3., 2.]],
                ]
                .into(),
                surface_error_mm: 0.,
                source: serde_json::Value::Null,
            },
            base_z_mm: 0.,
            source_enclosure: SurfaceEnclosure::Bounded,
            contact_witnesses: None,
            inner_patches: vec![],
            finishing: Some(FinishingScope {
                floor_z_mm: 3.,
                tool_radius_mm: 1.,
                max_access_work: budget,
                ..Default::default()
            }),
        }
        .into();
        let binding = Binding::new(&m).unwrap();
        let mut p = StreamProducer::new(m.clone()).unwrap();
        p.start_material_verification(None).unwrap().unwrap();
        let bytes = p
            .finish_material_verification(&AtomicBool::new(false))
            .unwrap();
        binding.decode(&bytes, 0, None).unwrap();
        let offset = 20 + identity_size(&m);
        let original: serde_json::Value = serde_json::from_slice(&bytes[offset..]).unwrap();
        let encode = |value: &serde_json::Value| {
            let mut body = bytes[20..offset].to_vec();
            body.extend(serde_json::to_vec(value).unwrap());
            reframe(&bytes, &body)
        };
        assert_eq!(original["check"]["report"]["complete"], budget > 1);
        for field in [
            "scope",
            "complete",
            "classified_cells",
            "inner_contact_available",
        ] {
            let mut changed = original.clone();
            let f = &mut changed["check"]["report"]["finishing"];
            match field {
                "scope" => f["scope"]["floor_z_mm"] = serde_json::json!(2.),
                "complete" => f["complete"] = serde_json::json!(budget == 1),
                "classified_cells" => f["classified_cells"] = serde_json::json!(999999),
                _ => f["inner_contact_available"] = serde_json::json!(false),
            }
            assert!(
                binding
                    .decode(&encode(&changed), 0, None)
                    .unwrap_err()
                    .contains("coverage mismatch")
            );
        }
        let f = &original["check"]["report"]["finishing"];
        assert_eq!(
            f["within_allowance"]["volume_mm3"],
            serde_json::json!([0., 0.])
        );
        assert_eq!(f["within_allowance"]["assessment"], "within_tolerance");
        for field in [
            "finding_cells",
            "missing_inner_contact_cells",
            "stock_height_interval_cells",
        ] {
            let mut changed = original.clone();
            changed["check"]["report"]["finishing"]["uncertainty"][field] =
                serde_json::json!(999999);
            assert!(binding.decode(&encode(&changed), 0, None).is_err());
        }
        if budget > 1 {
            let mut changed = original.clone();
            changed["check"]["report"]["finishing"]["findings"][0]["uncertainty"]["missing_inner_contact"] =
                serde_json::json!(true);
            assert!(binding.decode(&encode(&changed), 0, None).is_err());
        }
        // Older retained reports have no uncertainty metadata. Absence remains
        // explicit and their conservative incomplete allowance band is readable.
        let mut legacy = original.clone();
        let f = &mut legacy["check"]["report"]["finishing"];
        f.as_object_mut().unwrap().remove("uncertainty");
        for finding in f["findings"].as_array_mut().unwrap() {
            finding.as_object_mut().unwrap().remove("uncertainty");
        }
        if budget == 1 {
            f["within_allowance"]["assessment"] = serde_json::json!("incomplete");
            f["within_allowance"]["volume_mm3"] = serde_json::json!([0., 24.]);
        }
        binding.decode(&encode(&legacy), 0, None).unwrap();
        let mut omitted = original;
        omitted["check"]["report"]
            .as_object_mut()
            .unwrap()
            .remove("finishing");
        assert!(
            binding
                .decode(&encode(&omitted), 0, None)
                .unwrap_err()
                .contains("coverage mismatch")
        );
    }
}

#[test]
fn retained_sweep_proofs_bind_source_budget_replay_history_and_checked_move_totals() {
    use camlib::{material::*, stream_verification::Binding, surface::SurfaceMesh};
    for mode in ["raw", "initial", "resumed", "spent"] {
        let mut m = checked_manifest();
        m.stock.height_mut().unwrap().max_mm = [4., 4., 3.];
        m.stock.height_mut().unwrap().cell_size_mm = 0.25;
        m.initial_position_mm = [1.95, 1., 1.];
        m.tools[0].cutter.diameter_mm = 0.1;
        m.tools[0].cutter.shank_diameter_mm = 0.1;
        let config = m.verification.as_mut().unwrap();
        config.scene = None;
        config.material[0].options = Options {
            comparison_tolerance_mm: 0.02,
            gouge_tolerance_mm: (mode == "spent").then_some(0.0005),
            max_findings: 1024,
            ..Options::default()
        };
        config.material[0].target = SurfaceTarget {
            version: 1,
            units: "mm".into(),
            id: "wall".into(),
            revision: "1".into(),
            surface: SurfaceMesh {
                version: 1,
                units: "mm".into(),
                triangles: vec![
                    [[2., 0., 2.], [4., 0., 2.], [4., 4., 2.]],
                    [[2., 0., 2.], [4., 4., 2.], [2., 4., 2.]],
                ]
                .into(),
                surface_error_mm: 0.001,
                source: serde_json::Value::Null,
            },
            base_z_mm: 0.,
            source_enclosure: SurfaceEnclosure::Bounded,
            contact_witnesses: None,
            inner_patches: vec![],
            finishing: None,
        }
        .into();
        if mode == "initial" {
            m.initial_stock = Some(InitialStock {
                source_id: "measured".into(),
                revision: "1".into(),
                source_sha256: "a".repeat(64),
                source_to_setup: WorkFrame::identity(),
                uncertainty_mm: 0.,
                representation: "whole_cell_top_down".into(),
                bounds: vec![
                    HeightBounds {
                        lower_mm: 3.,
                        upper_mm: 3.
                    };
                    256
                ],
            });
        }
        let cancel = AtomicBool::new(false);
        let mut producer = StreamProducer::new(m.clone()).unwrap();
        let motion = |id, start, end| Motion {
            id,
            operation: "tangent".into(),
            tool: "t1".into(),
            kind: MoveKind::Cut,
            curve: Curve::Line { start, end },
            feed_mm_min: Some(100.),
        };
        producer
            .push_checked_motion(
                motion(1, m.initial_position_mm, [1.95, 3., 1.]),
                DEFAULT_FRAME_LIMIT,
                &cancel,
            )
            .unwrap();
        let completed = if mode == "resumed" {
            let checkpoint = producer.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap();
            producer =
                StreamProducer::from_checkpoint(m.clone(), &checkpoint, CHECKPOINT_FRAME_LIMIT)
                    .unwrap();
            producer
                .push_checked_motion(
                    motion(2, [1.95, 3., 1.], [1.95, 3.5, 1.]),
                    DEFAULT_FRAME_LIMIT,
                    &cancel,
                )
                .unwrap();
            2
        } else {
            1
        };
        producer.start_material_verification(None).unwrap().unwrap();
        let bytes = producer.finish_material_verification(&cancel).unwrap();
        let binding = Binding::new(&m).unwrap();
        binding.decode(&bytes, completed, None).unwrap();
        let offset = 20 + identity_size(&m);
        let original: serde_json::Value = serde_json::from_slice(&bytes[offset..]).unwrap();
        let encode = |value: &serde_json::Value| {
            let mut body = bytes[20..offset].to_vec();
            body.extend(serde_json::to_vec(value).unwrap());
            reframe(&bytes, &body)
        };
        let proof = &original["check"]["sweep_proof"];
        // A resume from the producer's own checkpoint keeps the proof
        // states and their history (#108); a declared blank never has it.
        assert_eq!(proof["history_complete"], mode != "initial");
        assert!(
            original["check"]["report"]["gouge"]["possible_only_cells"]
                .as_u64()
                .unwrap()
                > 0
        );
        if mode == "raw" || mode == "resumed" {
            assert!(proof["decided_cells"].as_u64().unwrap() > 0);
        } else {
            assert_eq!(proof["decided_cells"], 0);
            assert!(proof["unavailable"].is_string());
        }
        for (field, value) in [
            ("surface_error_mm", serde_json::json!(0.)),
            ("tolerance_mm", serde_json::json!(0.1)),
            ("history_complete", serde_json::json!(mode == "initial")),
            ("proven_moves", serde_json::json!(completed + 1)),
            ("unproven_moves", serde_json::json!(u64::MAX)),
        ] {
            let mut changed = original.clone();
            changed["check"]["sweep_proof"][field] = value;
            if mode == "spent" && field == "history_complete" {
                // A conservative incomplete-history claim with no decisions
                // is legitimate, including a checkpoint restored at zero.
                binding.decode(&encode(&changed), completed, None).unwrap();
                continue;
            }
            assert!(
                binding.decode(&encode(&changed), completed, None).is_err(),
                "{mode}: {field}"
            );
        }
        if mode == "raw" || mode == "resumed" {
            let mut changed = original.clone();
            changed["check"]["sweep_proof"]["unavailable"] = serde_json::json!("proof unavailable");
            assert!(binding.decode(&encode(&changed), completed, None).is_err());
            let mut changed = original.clone();
            let report = &mut changed["check"]["report"];
            report["complete"] = serde_json::json!(false);
            report["stats"]["classified_cells"] = serde_json::json!(255);
            report["stats"]["unexamined_cells"] = serde_json::json!(1);
            report["residual"]["assessment"] = serde_json::json!("incomplete");
            report["gouge"]["assessment"] = serde_json::json!("incomplete");
            assert!(binding.decode(&encode(&changed), completed, None).is_err());
        } else {
            let mut changed = original.clone();
            changed["check"]["sweep_proof"]["history_complete"] = serde_json::json!(true);
            changed["check"]["sweep_proof"]["unavailable"] = serde_json::Value::Null;
            changed["check"]["sweep_proof"]["decided_cells"] = serde_json::json!(1);
            assert!(binding.decode(&encode(&changed), completed, None).is_err());
        }
        if mode == "spent" {
            assert!(
                proof["unavailable"]
                    .as_str()
                    .unwrap()
                    .contains("effective gouge tolerance 0.0005")
            );
            let mut changed = original.clone();
            changed["check"]["sweep_proof"]["proven_moves"] = serde_json::json!(1);
            changed["check"]["sweep_proof"]["unproven_moves"] = serde_json::json!(0);
            assert!(binding.decode(&encode(&changed), completed, None).is_err());
        }
    }
}

#[test]
fn checkpoint_restores_never_promote_bounded_cells_into_raw_stock_proof_history() {
    use camlib::{
        material::*,
        stream_verification::{Binding, Check},
        surface::SurfaceMesh,
    };
    let mut m = checked_manifest();
    m.stock.height_mut().unwrap().max_mm = [2., 2., 3.];
    m.stock.height_mut().unwrap().cell_size_mm = 0.5;
    m.initial_position_mm = [0.5, 0.5, 2.5];
    let config = m.verification.as_mut().unwrap();
    config.scene = None;
    config.material[0].target = SurfaceTarget {
        version: 1,
        units: "mm".into(),
        id: "bounded-source".into(),
        revision: "1".into(),
        surface: SurfaceMesh {
            version: 1,
            units: "mm".into(),
            triangles: vec![
                [[-1., -1., 2.], [3., -1., 2.], [3., 3., 2.]],
                [[-1., -1., 2.], [3., 3., 2.], [-1., 3., 2.]],
            ]
            .into(),
            surface_error_mm: 0.01,
            source: serde_json::Value::Null,
        },
        base_z_mm: 0.,
        source_enclosure: SurfaceEnclosure::Bounded,
        contact_witnesses: None,
        inner_patches: vec![],
        finishing: None,
    }
    .into();
    config.material[0].options = Options {
        comparison_tolerance_mm: 0.05,
        ..Options::default()
    };
    let cancel = AtomicBool::new(false);
    let mut fresh = StreamProducer::new(m.clone()).unwrap();
    let original = fresh.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap();
    // Keep the zero cursor and all manifest attribution intact, but replace
    // every valid bounded cell with pre-erased material and recompute CRC.
    let mut body = original[20..].to_vec();
    // The cells end where the producer's kept proof section (#108) begins.
    let cells_end = body
        .windows(8)
        .rposition(|w| w == b"CAMPRF01")
        .unwrap_or(body.len());
    let cells_start = cells_end - 16 * 16;
    for cell in body[cells_start..cells_end].as_chunks_mut::<16>().0 {
        cell[..8].copy_from_slice(&0.5_f64.to_le_bytes());
        cell[8..].copy_from_slice(&0.5_f64.to_le_bytes());
    }
    let altered = reframe(&original, &body);
    let check = |producer: &mut StreamProducer, raw_history: bool, missing: bool, moves: u64| {
        assert_eq!(producer.completed(), moves);
        producer.start_material_verification(None).unwrap().unwrap();
        let bytes = producer.finish_material_verification(&cancel).unwrap();
        let record = Binding::new(&m)
            .unwrap()
            .decode(&bytes, moves, None)
            .unwrap();
        let Check::Material {
            report,
            sweep_proof,
            ..
        } = record.check
        else {
            panic!("material report expected");
        };
        assert!(report.complete);
        assert_eq!(report.gouge.definite_cells, 0);
        assert_eq!(report.gouge.possible_only_cells > 0, missing);
        let proof = sweep_proof.unwrap();
        assert_eq!(proof.history_complete, raw_history);
        assert_eq!(proof.proven_moves + proof.unproven_moves, moves);
        assert_eq!(proof.decided_cells, 0);
        assert_eq!(proof.unavailable.is_some(), !raw_history);
    };
    // A genuinely fresh raw grid still starts with complete proof history.
    check(&mut fresh, true, false, 0);
    let mut bytes =
        StreamProducer::from_checkpoint(m.clone(), &altered, CHECKPOINT_FRAME_LIMIT).unwrap();
    check(&mut bytes, false, true, 0);
    let mut streamed = StreamProducer::from_checkpoint_reader(
        m.clone(),
        &mut Cursor::new(&altered),
        CHECKPOINT_FRAME_LIMIT,
    )
    .unwrap();
    check(&mut streamed, false, true, 0);

    let mut inplace = StreamProducer::new(m.clone()).unwrap();
    inplace
        .push_checked_motion(
            Motion {
                id: 1,
                operation: "clear".into(),
                tool: "t1".into(),
                kind: MoveKind::Cut,
                curve: Curve::Line {
                    start: m.initial_position_mm,
                    end: [1.5, 0.5, 2.5],
                },
                feed_mm_min: Some(100.),
            },
            DEFAULT_FRAME_LIMIT,
            &cancel,
        )
        .unwrap();
    check(&mut inplace, true, false, 1);
    // Completed stages, old counters, and previously complete history reset.
    inplace
        .restore_checkpoint(&altered, CHECKPOINT_FRAME_LIMIT)
        .unwrap();
    check(&mut inplace, false, true, 0);
    // Restore an ordinary raw-looking checkpoint too: replay remains usable,
    // but neither its zero cursor nor its cells grant a provenance upgrade.
    inplace
        .restore_checkpoint(&original, CHECKPOINT_FRAME_LIMIT)
        .unwrap();
    inplace.start_material_verification(None).unwrap().unwrap();
    // Restoring while a material stage is active clears that pending stage.
    inplace
        .restore_checkpoint(&altered, CHECKPOINT_FRAME_LIMIT)
        .unwrap();
    check(&mut inplace, false, true, 0);
}
