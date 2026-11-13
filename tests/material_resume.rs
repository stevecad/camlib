//! A stream resumed from its producer's checkpoint keeps the material
//! check's per-move proof (#108): it decides the same cells as the run
//! that never stopped. A checkpoint without the proof state (a consumer's)
//! decides nothing after the resume, and a malformed proof section is
//! refused.
use camlib::{
    material::{self, SurfaceEnclosure, SurfaceTarget, SweepProofReport},
    motion::*,
    stock::*,
    stream::*,
    stream_verification::{Binding, Check, Config, MaterialCheck},
    surface::SurfaceMesh,
    tool::*,
};
use std::{io::Cursor, sync::atomic::AtomicBool};

/// A 30 x 20 x 10 block on Z 0 (closed mesh) in 40 x 30 x 12 stock.
fn cuboid(lo: Vec3, hi: Vec3) -> Vec<[Vec3; 3]> {
    let p =
        |x: usize, y: usize, z: usize| [[lo[0], hi[0]][x], [lo[1], hi[1]][y], [lo[2], hi[2]][z]];
    let quad = |a: Vec3, b: Vec3, c: Vec3, d: Vec3| vec![[a, b, c], [a, c, d]];
    [
        quad(p(0, 0, 1), p(1, 0, 1), p(1, 1, 1), p(0, 1, 1)),
        quad(p(0, 0, 0), p(0, 1, 0), p(1, 1, 0), p(1, 0, 0)),
        quad(p(0, 0, 0), p(1, 0, 0), p(1, 0, 1), p(0, 0, 1)),
        quad(p(0, 1, 0), p(0, 1, 1), p(1, 1, 1), p(1, 1, 0)),
        quad(p(0, 0, 0), p(0, 0, 1), p(0, 1, 1), p(0, 1, 0)),
        quad(p(1, 0, 0), p(1, 1, 0), p(1, 1, 1), p(1, 0, 1)),
    ]
    .concat()
}

fn manifest() -> StreamManifest {
    StreamManifest {
        tool_change: None,
        initial_stock: None,
        rapid_retracts: false,
        turning: None,
        version: 1,
        run: RunIdentity {
            id: "resume".into(),
            revision: "1".into(),
        },
        stock: StockSpec {
            version: 1,
            units: "mm".into(),
            min_mm: [0., 0., 0.],
            max_mm: [40., 30., 12.],
            cell_size_mm: 0.25,
            max_cells: 1 << 20,
        }
        .into(),
        tools: vec![ToolAssembly {
            id: "em6".into(),
            cutter: Cutter {
                id: "em6".into(),
                name: String::new(),
                shape: CutterShape::Flat,
                diameter_mm: 6.,
                cutting_length_mm: 16.,
                overall_length_mm: 60.,
                shank_diameter_mm: 6.,
                flutes: 2,
                center_cutting: true,
                material: "carbide".into(),
                neck: None,
            },
            stickout_mm: 30.,
            gauge_length_mm: 80.,
            holders: vec![HolderSection {
                z_min_mm: 30.,
                z_max_mm: 80.,
                radius_min_mm: 12.,
                radius_max_mm: 20.,
            }],
        }],
        initial_position_mm: [-4., -4., 20.],
        sweep: SweepOptions::default(),
        operations: vec![],
        verification: Some(Config {
            stock_contact: None,
            scene: None,
            material: vec![MaterialCheck {
                after_operation: None,
                target: SurfaceTarget {
                    version: 1,
                    units: "mm".into(),
                    id: "block".into(),
                    revision: "1".into(),
                    surface: SurfaceMesh {
                        version: 1,
                        units: "mm".into(),
                        triangles: cuboid([5., 5., 0.], [35., 25., 10.]).into(),
                        surface_error_mm: 0.0005,
                        source: serde_json::Value::Null,
                    },
                    base_z_mm: 0.,
                    source_enclosure: SurfaceEnclosure::Bounded,
                    contact_witnesses: None,
                    inner_patches: vec![],
                    finishing: None,
                }
                .into(),
                options: material::Options {
                    comparison_tolerance_mm: 0.005,
                    ..Default::default()
                },
            }],
        }),
    }
}

/// A posted-style program (3 decimals): face the top, contour the block
/// tangent to its walls with arcs at the corners, and one pass 0.05 mm
/// into the far wall (unprovable: its cells stay undecided).
fn motions() -> Vec<Motion> {
    let z = 10.;
    let mut curves = vec![Curve::Line {
        start: [-4., -4., 20.],
        end: [-4., -4., z],
    }];
    // Facing passes across the top, tool bottom on it.
    for (i, y) in [-1., 4., 9., 14., 19., 24., 29., 31.]
        .into_iter()
        .enumerate()
    {
        let (a, b) = if i % 2 == 0 { (-4., 44.) } else { (44., -4.) };
        let last = curves.last().unwrap().end();
        curves.push(Curve::Line {
            start: last,
            end: [a, y, z],
        });
        curves.push(Curve::Line {
            start: [a, y, z],
            end: [b, y, z],
        });
    }
    let last = curves.last().unwrap().end();
    curves.push(Curve::Line {
        start: last,
        end: [last[0], last[1], 15.],
    });
    // Contour at Z 4, centre 3 mm off each wall; corner arcs radius 3.
    let h = 4.;
    let path = [
        ([5., 2., h], [35., 2., h], [35., 5., h], [38., 5., h]),
        ([38., 5., h], [38., 25., h], [35., 25., h], [35., 28., h]),
        ([35., 28., h], [5., 28., h], [5., 25., h], [2., 25., h]),
        ([2., 25., h], [2., 5., h], [5., 5., h], [5., 2., h]),
    ];
    curves.push(Curve::Line {
        start: [last[0], last[1], 15.],
        end: [5., 2., 15.],
    });
    curves.push(Curve::Line {
        start: [5., 2., 15.],
        end: [5., 2., h],
    });
    for (start, end, center, arc_end) in path {
        curves.push(Curve::Line { start, end });
        curves.push(Curve::Arc {
            start: end,
            end: arc_end,
            center,
            plane: Plane::Xy,
            sweep_rad: std::f64::consts::FRAC_PI_2,
        });
    }
    // 0.05 mm into the wall at y = 25, along a short stretch.
    curves.push(Curve::Line {
        start: [5., 2., h],
        end: [5., 2., 15.],
    });
    curves.push(Curve::Line {
        start: [5., 2., 15.],
        end: [15., 28.05 - 0.1, 15.],
    });
    curves.push(Curve::Line {
        start: [15., 27.95, 15.],
        end: [15., 27.95, h],
    });
    curves.push(Curve::Line {
        start: [15., 27.95, h],
        end: [20., 27.95, h],
    });
    curves.push(Curve::Line {
        start: [20., 27.95, h],
        end: [20., 27.95, 20.],
    });
    let mut previous = [-4., -4., 20.];
    curves
        .into_iter()
        .enumerate()
        .map(|(i, mut curve)| {
            // Join any gap the hand-written path left (a rapid would be the
            // posted program's; here the next motion starts at the end).
            if let Curve::Line { start, .. } = &mut curve {
                *start = previous;
            }
            previous = curve.end();
            Motion {
                id: i as u64 + 1,
                operation: "contour".into(),
                tool: "em6".into(),
                kind: MoveKind::Cut,
                curve,
                feed_mm_min: Some(900.),
            }
        })
        .collect()
}

/// The final material record (report JSON and proof) of a producer.
fn finish(p: &mut StreamProducer, m: &StreamManifest) -> (serde_json::Value, SweepProofReport) {
    let cancel = AtomicBool::new(false);
    p.start_material_verification(None).unwrap().unwrap();
    let bytes = p.finish_material_verification(&cancel).unwrap();
    let record = Binding::new(m)
        .unwrap()
        .decode(&bytes, p.completed(), None)
        .unwrap();
    p.finish(DEFAULT_FRAME_LIMIT).unwrap();
    let Check::Material {
        report,
        sweep_proof,
        ..
    } = record.check
    else {
        panic!("expected a material record")
    };
    (serde_json::to_value(&report).unwrap(), sweep_proof.unwrap())
}

fn push(p: &mut StreamProducer, motions: &[Motion]) {
    let cancel = AtomicBool::new(false);
    for motion in motions {
        p.push_checked_motion(motion.clone(), DEFAULT_FRAME_LIMIT, &cancel)
            .unwrap();
    }
}

fn same(a: &SweepProofReport, b: &SweepProofReport) -> bool {
    (
        a.proven_moves,
        a.unproven_moves,
        a.history_complete,
        a.decided_cells,
        &a.unavailable,
    ) == (
        b.proven_moves,
        b.unproven_moves,
        b.history_complete,
        b.decided_cells,
        &b.unavailable,
    )
}

/// Rewrite a frame's body and checksum (the frame header is 20 bytes; the
/// CRC covers its first 16 and the body).
fn reframe(original: &[u8], body: &[u8]) -> Vec<u8> {
    let mut header = original[..20].to_vec();
    header[12..16].copy_from_slice(&(body.len() as u32).to_le_bytes());
    let mut crc = crc32fast::Hasher::new();
    crc.update(&header[..16]);
    crc.update(body);
    header[16..20].copy_from_slice(&crc.finalize().to_le_bytes());
    [header, body.to_vec()].concat()
}

#[test]
fn a_resumed_replay_decides_the_same_cells_as_an_uninterrupted_one() {
    let m = manifest();
    let moves = motions();
    let mut whole = StreamProducer::new(m.clone()).unwrap();
    push(&mut whole, &moves);
    let (report, proof) = finish(&mut whole, &m);
    // Not vacuous: wall cells are possible-only, the proof decides most of
    // them, and the pass into the wall keeps some undecided.
    let possible = report["gouge"]["possible_only_cells"].as_u64().unwrap() as usize;
    assert!(proof.history_complete && proof.unproven_moves >= 1);
    assert!(
        proof.decided_cells > 0 && proof.decided_cells < possible,
        "{proof:?} of {possible}"
    );

    for k in [1, moves.len() / 2, moves.len() - 2] {
        let mut first = StreamProducer::new(m.clone()).unwrap();
        push(&mut first, &moves[..k]);
        let checkpoint = first.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap();
        // From the frame, from a file reader, and by rewinding a producer
        // that ran further (its later motions must be forgotten).
        let mut resumed = vec![
            StreamProducer::from_checkpoint(m.clone(), &checkpoint, CHECKPOINT_FRAME_LIMIT)
                .unwrap(),
            StreamProducer::from_checkpoint_reader(
                m.clone(),
                &mut Cursor::new(checkpoint.clone()),
                CHECKPOINT_FRAME_LIMIT,
            )
            .unwrap(),
        ];
        let mut rewound = StreamProducer::new(m.clone()).unwrap();
        push(&mut rewound, &moves);
        rewound
            .restore_checkpoint(&checkpoint, CHECKPOINT_FRAME_LIMIT)
            .unwrap();
        resumed.push(rewound);
        for mut p in resumed {
            assert_eq!(p.completed(), k as u64);
            push(&mut p, &moves[k..]);
            let (r, q) = finish(&mut p, &m);
            assert_eq!(r, report, "resumed at {k}");
            assert!(same(&q, &proof), "resumed at {k}: {q:?} vs {proof:?}");
        }
        // A checkpoint taken by a resumed producer keeps the proof too.
        let mut again =
            StreamProducer::from_checkpoint(m.clone(), &checkpoint, CHECKPOINT_FRAME_LIMIT)
                .unwrap();
        push(&mut again, &moves[k..k + 1]);
        let second = again.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap();
        let mut p =
            StreamProducer::from_checkpoint(m.clone(), &second, CHECKPOINT_FRAME_LIMIT).unwrap();
        push(&mut p, &moves[k + 1..]);
        assert!(same(&finish(&mut p, &m).1, &proof));
    }
}

#[test]
fn a_checkpoint_without_the_proof_state_decides_nothing_after_it() {
    let m = manifest();
    let moves = motions();
    let k = moves.len() / 2;
    // A consumer's checkpoint: the same stock and cursor, no proof states.
    let mut receiver = StreamReceiver::new(m.clone()).unwrap();
    let mut producer = StreamProducer::new(m.clone()).unwrap();
    let cancel = AtomicBool::new(false);
    for motion in &moves[..k] {
        let (chunk, _) = producer
            .push_checked_motion(motion.clone(), DEFAULT_FRAME_LIMIT, &cancel)
            .unwrap();
        receiver.accept(&chunk, DEFAULT_FRAME_LIMIT).unwrap();
    }
    let plain = receiver.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap();
    let kept = producer.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap();
    assert!(kept.len() > plain.len());
    // The receiver reads both and ignores the proof section.
    let mut other =
        StreamReceiver::from_checkpoint(m.clone(), &kept, CHECKPOINT_FRAME_LIMIT).unwrap();
    assert_eq!(
        other.stock().unwrap().snapshot(),
        receiver.stock().unwrap().snapshot()
    );
    other
        .restore_checkpoint(&plain, CHECKPOINT_FRAME_LIMIT)
        .unwrap();

    let resumed =
        StreamProducer::from_checkpoint(m.clone(), &plain, CHECKPOINT_FRAME_LIMIT).unwrap();
    // A fresh producer rewound to it (it had seen no motion) must not keep
    // claiming the whole history.
    let mut fresh = StreamProducer::new(m.clone()).unwrap();
    fresh
        .restore_checkpoint(&plain, CHECKPOINT_FRAME_LIMIT)
        .unwrap();
    for mut p in [resumed, fresh] {
        push(&mut p, &moves[k..]);
        let (_, proof) = finish(&mut p, &m);
        assert!(!proof.history_complete);
        assert_eq!(proof.decided_cells, 0);
        assert!(
            proof
                .unavailable
                .as_deref()
                .unwrap()
                .contains("without this check's proof state")
        );
    }
}

#[test]
fn malformed_or_foreign_proof_sections_are_refused() {
    let m = manifest();
    let moves = motions();
    let mut p = StreamProducer::new(m.clone()).unwrap();
    push(&mut p, &moves[..5]);
    let checkpoint = p.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap();
    let section = checkpoint
        .windows(8)
        .position(|w| w == b"CAMPRF01")
        .expect("proof section");
    let body = &checkpoint[20..];
    let at = section - 20;
    let refused = |body: Vec<u8>| {
        let bytes = reframe(&checkpoint, &body);
        let mut rewound = StreamProducer::new(m.clone()).unwrap();
        push(&mut rewound, &moves[..2]);
        let before = rewound.completed();
        assert!(
            StreamProducer::from_checkpoint(m.clone(), &bytes, CHECKPOINT_FRAME_LIMIT).is_err()
        );
        assert!(
            StreamProducer::from_checkpoint_reader(
                m.clone(),
                &mut Cursor::new(bytes.clone()),
                CHECKPOINT_FRAME_LIMIT
            )
            .is_err()
        );
        // A failed restore leaves the producer where it was.
        assert!(
            rewound
                .restore_checkpoint(&bytes, CHECKPOINT_FRAME_LIMIT)
                .is_err()
        );
        assert_eq!(rewound.completed(), before);
    };
    // Unchanged: accepted.
    StreamProducer::from_checkpoint(
        m.clone(),
        &reframe(&checkpoint, body),
        CHECKPOINT_FRAME_LIMIT,
    )
    .unwrap();
    // Wrong tag, wrong stage count, a truncated state, a trailing byte,
    // a state for another grid. (The tag is followed by the 32-byte digest
    // of the cells the states were computed with.)
    let mut bad = body.to_vec();
    bad[at] ^= 1;
    refused(bad);
    let mut bad = body.to_vec();
    bad[at + 8 + 32] = 2;
    refused(bad);
    refused(body[..body.len() - 1].to_vec());
    refused([body, &[0]].concat());
    let mut bad = body.to_vec();
    let cells = at + 8 + 32 + 4 + 8 + 17;
    bad[cells] ^= 1;
    refused(bad);
    // States kept for other cells (here: the digest differs) restore the
    // stock but not the proof: the history before the resume is unseen.
    let mut other_cells = body.to_vec();
    other_cells[at + 8] ^= 1;
    let mut resumed = StreamProducer::from_checkpoint(
        m.clone(),
        &reframe(&checkpoint, &other_cells),
        CHECKPOINT_FRAME_LIMIT,
    )
    .unwrap();
    push(&mut resumed, &moves[5..]);
    let (_, proof) = finish(&mut resumed, &m);
    assert!(!proof.history_complete);
    assert_eq!(proof.decided_cells, 0);
    // A checksum mismatch alone is refused as before.
    let mut corrupt = checkpoint.clone();
    *corrupt.last_mut().unwrap() ^= 1;
    assert!(StreamProducer::from_checkpoint(m.clone(), &corrupt, CHECKPOINT_FRAME_LIMIT).is_err());
    // Another run's checkpoint (here: the comparison tolerance differs, so
    // the manifest digest does) is refused before its proof is read.
    let mut other = m.clone();
    other.verification.as_mut().unwrap().material[0]
        .options
        .comparison_tolerance_mm = 0.004;
    assert!(StreamProducer::from_checkpoint(other, &checkpoint, CHECKPOINT_FRAME_LIMIT).is_err());
}

/// The producer's cleared-column section (`CAMCOL01`) of a checkpoint: where
/// it starts in the frame, and the frame body before it.
fn column_section(checkpoint: &[u8]) -> usize {
    checkpoint
        .windows(8)
        .rposition(|w| w == b"CAMCOL01")
        .expect("column section")
}

fn at_half() -> (StreamManifest, Vec<Motion>, usize, Vec<u8>) {
    let m = manifest();
    let moves = motions();
    let k = moves.len() / 2;
    let mut p = StreamProducer::new(m.clone()).unwrap();
    push(&mut p, &moves[..k]);
    let checkpoint = p.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap();
    (m, moves, k, checkpoint)
}

#[test]
fn the_cleared_columns_survive_a_checkpoint_round_trip() {
    let (m, moves, k, checkpoint) = at_half();
    let section = column_section(&checkpoint);
    // Not vacuous: the section holds columns, not just its header.
    assert!(checkpoint.len() - section > 100, "{}", checkpoint.len() - section);
    let mut resumed =
        StreamProducer::from_checkpoint(m.clone(), &checkpoint, CHECKPOINT_FRAME_LIMIT).unwrap();
    // Reading then writing again is byte for byte the same checkpoint.
    assert_eq!(resumed.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap(), checkpoint);
    // And the resumed stock lowers the same cells as one that never stopped.
    let mut whole = StreamProducer::new(m.clone()).unwrap();
    push(&mut whole, &moves);
    push(&mut resumed, &moves[k..]);
    assert_eq!(
        resumed.stock().unwrap().cells(),
        whole.stock().unwrap().cells()
    );
}

#[test]
fn a_checkpoint_without_the_column_section_resumes_with_looser_bounds_only() {
    let (m, moves, k, checkpoint) = at_half();
    let section = column_section(&checkpoint);
    // An older or consumer-written checkpoint: the same frame, no columns.
    let plain = reframe(&checkpoint, &checkpoint[20..section]);
    let mut resumed =
        StreamProducer::from_checkpoint(m.clone(), &plain, CHECKPOINT_FRAME_LIMIT).unwrap();
    let mut whole = StreamProducer::new(m.clone()).unwrap();
    push(&mut whole, &moves);
    push(&mut resumed, &moves[k..]);
    let (a, b) = (resumed.stock().unwrap().cells(), whole.stock().unwrap().cells());
    assert_eq!(a.len(), b.len());
    // Never lower than the uninterrupted run's (it only lacks the union
    // rule's proof), and the lower bounds are identical.
    let mut looser = 0;
    for (a, b) in a.iter().zip(b) {
        assert_eq!(a.lower_mm, b.lower_mm);
        assert!(a.upper_mm >= b.upper_mm);
        looser += usize::from(a.upper_mm > b.upper_mm);
    }
    println!("{looser} cells keep a looser upper bound");
}

#[test]
fn malformed_column_sections_are_refused() {
    let (m, _, _, checkpoint) = at_half();
    let at = column_section(&checkpoint);
    let body = &checkpoint[20..];
    let section = at - 20;
    let refused = |body: Vec<u8>, why: &str| {
        let bytes = reframe(&checkpoint, &body);
        assert!(
            StreamProducer::from_checkpoint(m.clone(), &bytes, CHECKPOINT_FRAME_LIMIT).is_err(),
            "{why}"
        );
        assert!(
            StreamProducer::from_checkpoint_reader(
                m.clone(),
                &mut Cursor::new(bytes),
                CHECKPOINT_FRAME_LIMIT
            )
            .is_err(),
            "{why} (reader)"
        );
    };
    // Unchanged: accepted.
    StreamProducer::from_checkpoint(
        m.clone(),
        &reframe(&checkpoint, body),
        CHECKPOINT_FRAME_LIMIT,
    )
    .unwrap();
    refused(body[..body.len() - 1].to_vec(), "truncated");
    refused(body[..section + 12].to_vec(), "header only");
    refused([body, &[0u8][..]].concat(), "trailing byte");
    let edit = |name: &str, offset: usize, bytes: &[u8]| {
        let mut changed = body.to_vec();
        changed[section + offset..section + offset + bytes.len()].copy_from_slice(bytes);
        refused(changed, name);
    };
    edit("unproven flag", 8 + 32, &[2]);
    // Layout: magic 8, digest 32, unproven 1, operation (u16 length + text),
    // union_from u32, count u32, then the columns.
    let operation = u16::from_le_bytes(body[section + 41..section + 43].try_into().unwrap());
    let columns = section + 43 + operation as usize + 8;
    let (first, count) = (columns, (body.len() - columns) / (8 + 1 + 8 * 16));
    assert!(count > 0);
    let edit = |name: &str, offset: usize, bytes: &[u8]| {
        let mut changed = body.to_vec();
        changed[first + offset..first + offset + bytes.len()].copy_from_slice(bytes);
        refused(changed, name);
    };
    edit("tolerance NaN", 0, &f64::NAN.to_le_bytes());
    edit("tolerance too large", 0, &1.0f64.to_le_bytes());
    edit("unknown sweep kind", 8, &[9]);
    // Words after the tag: 0 floor, 1 radius, 2 error, 3..7 box, then the
    // shape (line: a, b, unit, length; arc: a, b, centre, radius, angle,
    // sweep).
    let word = |i: usize| 9 + 8 * i;
    edit("radius zero", word(1), &0f64.to_le_bytes());
    edit("negative path error", word(2), &(-1f64).to_le_bytes());
    edit("endpoint NaN", word(9), &f64::NAN.to_le_bytes());
    edit("endpoint off its path", word(9), &1.0e3f64.to_le_bytes());
}

#[test]
fn column_sections_for_other_cells_are_dropped_not_trusted() {
    let (m, _, _, checkpoint) = at_half();
    let at = column_section(&checkpoint);
    // Corrupt the digest of the cells the columns were committed with.
    let mut body = checkpoint[20..].to_vec();
    body[at - 20 + 8] ^= 0xff;
    let forged = reframe(&checkpoint, &body);
    let resumed =
        StreamProducer::from_checkpoint(m.clone(), &forged, CHECKPOINT_FRAME_LIMIT).unwrap();
    // The resumed stock keeps none of them: its next checkpoint has a column
    // section with no columns (a fresh producer's).
    let again = resumed.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap();
    let fresh = StreamProducer::new(m.clone())
        .unwrap()
        .checkpoint(CHECKPOINT_FRAME_LIMIT)
        .unwrap();
    assert_eq!(
        again.len() - column_section(&again),
        fresh.len() - column_section(&fresh)
    );
    assert!(checkpoint.len() - at > again.len() - column_section(&again));
}
