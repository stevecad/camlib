use camlib::{
    pocket::Operation,
    regeneration::{self, CacheOptions},
    stream::*,
};
use serde_json::Value;
use std::{
    collections::HashMap,
    path::PathBuf,
    sync::atomic::{AtomicBool, AtomicU64, Ordering},
    time::{SystemTime, UNIX_EPOCH},
};
struct Directory(PathBuf);
impl Directory {
    fn new() -> Self {
        Self::at_time(
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos(),
        )
    }
    fn at_time(timestamp: u128) -> Self {
        // Clock ticks can repeat across parallel tests; they are not unique IDs.
        // https://doc.rust-lang.org/std/time/struct.SystemTime.html#platform-specific-behavior
        static NEXT: AtomicU64 = AtomicU64::new(0);
        let p = std::env::temp_dir().join(format!(
            "cam-regeneration-test-{}-{timestamp}-{}",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed),
        ));
        // Fail rather than sharing ownership of an existing directory.
        std::fs::create_dir(&p).unwrap();
        Self(p)
    }
    fn options(&self) -> CacheOptions {
        CacheOptions {
            directory: self.0.clone(),
            max_entry_bytes: 64 * 1024 * 1024,
            max_total_bytes: 1024 * 1024 * 1024,
            dependencies: HashMap::new(),
        }
    }
}
impl Drop for Directory {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}
#[test]
fn directory_cleanup_preserves_other_instances_at_the_same_clock_tick() {
    let first = Directory::at_time(0);
    let second = Directory::at_time(0);
    let first_path = first.0.clone();
    let marker = second.0.join("owned-by-second");
    std::fs::write(&marker, b"keep").unwrap();
    drop(first);
    assert_eq!(std::fs::read(marker).unwrap(), b"keep");
    assert_ne!(first_path, second.0);
}

fn manifest() -> StreamManifest {
    let mut m: StreamManifest =
        serde_json::from_str(include_str!("../fixtures/operations/facing.json")).unwrap();
    m.stock.height_mut().unwrap().max_mm = [12., 10., 4.];
    m.stock.height_mut().unwrap().cell_size_mm = 0.5;
    m.initial_position_mm = [0., 0., 8.];
    let Operation::Facing(p) = &mut m.operations[0] else {
        panic!("fixture")
    };
    p.floor_z_mm = 3.;
    p.clearance_z_mm = 8.;
    p.stepdown_mm = 1.;
    p.stepover_mm = 4.;
    m
}
fn execute(m: StreamManifest, options: CacheOptions) -> (StreamProducer, Vec<Vec<u8>>, Vec<Value>) {
    let mut producer = StreamProducer::new(m.clone()).unwrap();
    let mut frames = vec![];
    let mut reports = vec![];
    regeneration::run(&mut producer, options, &AtomicBool::new(false), |frame| {
        if u16::from_le_bytes([frame[10], frame[11]]) == 7 {
            let report = decode_operation_control(&m, frame, DEFAULT_FRAME_LIMIT)?;
            if report["stage"] == "end" {
                reports.push(report);
            }
        }
        frames.push(frame.to_vec());
        Ok(())
    })
    .unwrap();
    (producer, frames, reports)
}
#[test]
fn warm_cache_reuses_real_deltas_with_new_identity_and_rebased_ids() {
    let directory = Directory::new();
    let mut first = manifest();
    let mut second = first.operations[0].clone();
    let Operation::Facing(p) = &mut second else {
        panic!()
    };
    p.id = "second".into();
    first.operations.push(second);
    let (cold, _, reports) = execute(first.clone(), directory.options());
    assert!(
        reports
            .iter()
            .all(|r| r["report"]["regeneration"]["cache_hit"] == false)
    );
    let mut warm = first.clone();
    warm.run.id = "warm-run".into();
    warm.run.revision = "new-revision".into();
    let mut extra = warm.operations[0].clone();
    let Operation::Facing(p) = &mut extra else {
        panic!()
    };
    p.id = "extra-air-pass".into();
    warm.operations.insert(1, extra);
    let (producer, frames, reports) = execute(warm.clone(), directory.options());
    assert_eq!(reports[0]["report"]["regeneration"]["cache_hit"], true);
    assert_eq!(reports[1]["report"]["regeneration"]["cache_hit"], false);
    assert_eq!(reports[2]["report"]["regeneration"]["cache_hit"], true);
    assert_eq!(reports[2]["report"]["regeneration"]["computed_sweeps"], 0);
    assert_eq!(
        producer.stock().unwrap().cells(),
        cold.stock().unwrap().cells()
    );
    assert!(producer.completed() > cold.completed());
    let mut receiver = StreamReceiver::new(warm).unwrap();
    let mut last = 0;
    for frame in &frames {
        match u16::from_le_bytes([frame[10], frame[11]]) {
            1 => {
                let motion = receiver.accept(frame, DEFAULT_FRAME_LIMIT).unwrap();
                assert!(motion.id > last);
                last = motion.id;
            }
            4 => receiver.accept_end(frame, DEFAULT_FRAME_LIMIT).unwrap(),
            _ => {}
        }
    }
    assert!(receiver.is_finished());
    assert_eq!(
        receiver.stock().unwrap().cells(),
        producer.stock().unwrap().cells()
    );
    let mut wrong = StreamReceiver::new(first).unwrap();
    let chunk = frames
        .iter()
        .find(|f| u16::from_le_bytes([f[10], f[11]]) == 1)
        .unwrap();
    assert!(wrong.accept(chunk, DEFAULT_FRAME_LIMIT).is_err());
    assert_eq!(wrong.completed(), 0);
}
#[test]
fn parameter_dependencies_and_corruption_are_not_silent_hits() {
    let directory = Directory::new();
    let m = manifest();
    execute(m.clone(), directory.options());
    let mut changed = m.clone();
    let Operation::Facing(p) = &mut changed.operations[0] else {
        panic!()
    };
    p.feed_mm_min = 700.;
    let (_, _, r) = execute(changed, directory.options());
    assert_eq!(r[0]["report"]["regeneration"]["cache_hit"], false);
    let mut options = directory.options();
    options
        .dependencies
        .insert(m.operations[0].id().into(), "a".repeat(64));
    let (_, _, r) = execute(m.clone(), options);
    assert_eq!(r[0]["report"]["regeneration"]["cache_hit"], false);
    let folder = directory.0.join("camlib-regeneration-v1");
    let index = std::fs::read_dir(&folder)
        .unwrap()
        .map(|e| e.unwrap().path())
        .find(|p| p.to_string_lossy().ends_with(".index.json"))
        .unwrap();
    let data: Value = serde_json::from_slice(&std::fs::read(&index).unwrap()).unwrap();
    let artifact = folder.join(format!(
        "{}.{}.jsonl",
        data["key"].as_str().unwrap(),
        data["sha256"].as_str().unwrap()
    ));
    let mut bytes = std::fs::read(&artifact).unwrap();
    bytes[0] ^= 1;
    std::fs::write(artifact, bytes).unwrap();
    // Select the exact corrupted input key through the same source/feed case.
    let mut candidates = vec![(m.clone(), directory.options())];
    let mut feed = m.clone();
    let Operation::Facing(p) = &mut feed.operations[0] else {
        panic!()
    };
    p.feed_mm_min = 700.;
    candidates.push((feed, directory.options()));
    let mut dep = directory.options();
    dep.dependencies
        .insert(m.operations[0].id().into(), "a".repeat(64));
    candidates.push((m, dep));
    let mut rejected = false;
    for (m, o) in candidates {
        let mut producer = StreamProducer::new(m).unwrap();
        let mut count = 0;
        let result = regeneration::run(&mut producer, o, &AtomicBool::new(false), |_| {
            count += 1;
            Ok(())
        });
        if let Err(error) = result {
            assert!(error.contains("checksum"));
            assert_eq!(count, 0);
            assert_eq!(producer.completed(), 0);
            rejected = true;
        }
    }
    assert!(rejected);
}
#[test]
fn cache_limits_leave_generation_usable_and_failed_output_does_not_publish() {
    let directory = Directory::new();
    let m = manifest();
    let mut options = directory.options();
    options.max_entry_bytes = 1024;
    let (producer, _, reports) = execute(m.clone(), options);
    assert!(producer.completed() > 0);
    assert_eq!(reports[0]["report"]["regeneration"]["saved"], false);
    assert!(
        !std::fs::read_dir(directory.0.join("camlib-regeneration-v1"))
            .unwrap()
            .any(|e| e.unwrap().path().to_string_lossy().ends_with(".index.json"))
    );
    let failed = Directory::new();
    let mut producer = StreamProducer::new(m.clone()).unwrap();
    let mut chunks = 0;
    let result = regeneration::run(
        &mut producer,
        failed.options(),
        &AtomicBool::new(false),
        |frame| {
            if u16::from_le_bytes([frame[10], frame[11]]) == 1 {
                chunks += 1;
                if chunks == 2 {
                    return Err("consumer cancelled".into());
                }
            }
            Ok(())
        },
    );
    assert!(result.is_err());
    assert!(
        !std::fs::read_dir(failed.0.join("camlib-regeneration-v1"))
            .unwrap()
            .any(|e| e.unwrap().path().to_string_lossy().ends_with(".index.json"))
    );
    let (_, _, r) = execute(m, failed.options());
    assert_eq!(r[0]["report"]["regeneration"]["cache_hit"], false);
}

#[test]
fn zero_motion_operations_have_complete_reusable_entries() {
    use camlib::{motion::Coolant, planar::PlanarRegion, pocket::PocketOperation};
    // The operation with no motion is a rest pocket with nothing left: the
    // facing before it has taken the stock below the pocket's floor. (This
    // used to be a pocket too small for its cutter, which is now refused,
    // #250: an operation whose cutter fits nowhere can do nothing it was
    // asked to, while a rest pocket with nothing left is a valid empty
    // plan.)
    let directory = Directory::new();
    let mut m = manifest();
    let (faced, _, _) = execute(m.clone(), Directory::new().options());
    assert!(faced.completed() > 0);
    let tool = m.tools[0].id.clone();
    m.operations.push(Operation::Pocket(PocketOperation {
        max_stay_down_mm: None,
        limits: None,
        id: "empty".into(),
        tool,
        region: PlanarRegion::circle("faced-away", [6., 5.], 4.9).unwrap(),
        top_z_mm: 4.,
        floor_z_mm: 3.5,
        stepdown_mm: 1.,
        stepover_mm: 2.,
        feed_mm_min: 600.,
        plunge_mm_min: 150.,
        clearance_z_mm: 8.,
        radial_allowance_mm: 0.,
        bottom_allowance_mm: 0.,
        tolerance_mm: 1e-5,
        spindle_rpm: 12000.,
        work_offset: 1,
        tool_number: 1,
        length_register: 1,
        coolant: Coolant::Flood,
        rest_tolerance_mm: Some(0.01),
        boundary_clockwise: None,
        entry: None,
        retract_z_mm: None,
        feed_z_mm: None,
    }));
    let (cold, _, r) = execute(m.clone(), directory.options());
    // Every motion of the run is the facing's.
    assert_eq!(cold.completed(), faced.completed());
    assert_eq!(r[1]["report"]["regeneration"]["saved"], true);
    m.run.revision = "warm-empty".into();
    let (warm, _, r) = execute(m, directory.options());
    assert_eq!(warm.completed(), faced.completed());
    assert_eq!(r[1]["report"]["regeneration"]["cache_hit"], true);
}
#[test]
fn typed_delta_old_values_are_checked_even_with_a_valid_artifact_checksum() {
    use sha2::{Digest, Sha256};
    let directory = Directory::new();
    let m = manifest();
    execute(m.clone(), directory.options());
    let folder = directory.0.join("camlib-regeneration-v1");
    let index = std::fs::read_dir(&folder)
        .unwrap()
        .map(|e| e.unwrap().path())
        .find(|p| p.to_string_lossy().ends_with(".index.json"))
        .unwrap();
    let mut data: Value = serde_json::from_slice(&std::fs::read(&index).unwrap()).unwrap();
    let old = folder.join(format!(
        "{}.{}.jsonl",
        data["key"].as_str().unwrap(),
        data["sha256"].as_str().unwrap()
    ));
    let original = std::fs::read(old).unwrap();
    let mut lines: Vec<Value> = original
        .split(|b| *b == b'\n')
        .filter(|s| !s.is_empty())
        .map(|s| serde_json::from_slice(s).unwrap())
        .collect();
    let record = lines
        .iter_mut()
        .find(|r| {
            r.get("delta")
                .and_then(|d| d.get("changes"))
                .and_then(Value::as_array)
                .is_some_and(|v| !v.is_empty())
        })
        .unwrap();
    record["delta"]["changes"][0]["before"]["lower_mm"] = serde_json::json!(3.75);
    let mut bytes = vec![];
    for line in lines {
        serde_json::to_writer(&mut bytes, &line).unwrap();
        bytes.push(b'\n');
    }
    let digest = format!("{:x}", Sha256::digest(&bytes));
    let path = folder.join(format!(
        "{}.{}.jsonl",
        data["key"].as_str().unwrap(),
        digest
    ));
    std::fs::write(path, &bytes).unwrap();
    data["sha256"] = digest.into();
    data["bytes"] = (bytes.len() as u64).into();
    std::fs::write(index, serde_json::to_vec(&data).unwrap()).unwrap();
    let mut producer = StreamProducer::new(m).unwrap();
    let error = regeneration::run(
        &mut producer,
        directory.options(),
        &AtomicBool::new(false),
        |_| Ok(()),
    )
    .unwrap_err();
    assert!(error.contains("old state"));
    assert!(
        producer
            .stock()
            .unwrap()
            .cells()
            .iter()
            .all(|h| h.lower_mm == 4. && h.upper_mm == 4.)
    );
}

#[test]
fn sha256_backend_preserves_independent_padding_block_and_large_payload_identities() {
    use sha2::{Digest, Sha256};
    assert_eq!(
        format!("{:x}", Sha256::digest(b"")),
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
    );
    assert_eq!(
        format!("{:x}", Sha256::digest(b"abc")),
        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
    );
    let oracle: serde_json::Value =
        serde_json::from_str(include_str!("../fixtures/integrity/sha256-identity.json")).unwrap();
    for case in oracle["cases"].as_array().unwrap() {
        let n = case["length"].as_u64().unwrap() as usize;
        let bytes: Vec<u8> = (0..n).map(|i| (i % 251) as u8).collect();
        assert_eq!(format!("{:x}", Sha256::digest(&bytes)), case["sha256"]);
        for chunk in [1, 7, 63, 64, 65, 4093, 65536] {
            if n > 1024 && chunk < 4093 {
                continue;
            }
            let mut h = Sha256::new();
            for b in bytes.chunks(chunk) {
                h.update(b);
            }
            assert_eq!(
                format!("{:x}", h.finalize()),
                case["sha256"],
                "length{n},chunk{chunk}"
            );
        }
    }
}

#[test]
fn cold_prefix_publishes_real_stock_before_cache_identity_or_files_and_warm_still_hits() {
    use camlib::stream::StreamReceiver;
    let directory = Directory::new();
    let m = manifest();
    let folder = directory.0.join("camlib-regeneration-v1");
    std::fs::create_dir_all(&folder).unwrap();
    std::fs::write(folder.join("unrelated-cache-data"), b"other work").unwrap();
    let mut receiver = StreamReceiver::new(m.clone()).unwrap();
    let mut producer = StreamProducer::new(m.clone()).unwrap();
    let mut first = false;
    let mut end = None;
    regeneration::run(&mut producer, directory.options(), &|| false, |frame| {
        match u16::from_le_bytes([frame[10], frame[11]]) {
            1 => {
                receiver.accept(frame, DEFAULT_FRAME_LIMIT)?;
                if !first && receiver.stock().unwrap().volume().removed_lower_mm3 > 0.001 {
                    first = true;
                    assert!(std::fs::read_dir(&folder).unwrap().all(|e|e.unwrap().file_name()=="unrelated-cache-data"),
                        "cold prefix wrote cache before actual stock publication in a nonempty persistent directory");
                }
            }
            7 => {
                let c = decode_operation_control(&m, frame, DEFAULT_FRAME_LIMIT)?;
                if c["stage"] == "begin" {
                    assert!(c["report"]["regeneration"]["key"].is_null());
                } else {
                    end = Some(c);
                }
            }
            4 => receiver.accept_end(frame, DEFAULT_FRAME_LIMIT)?,
            _ => (),
        }
        Ok(())
    })
    .unwrap();
    assert!(first && receiver.is_finished());
    assert_eq!(
        producer.stock().unwrap().cells(),
        receiver.stock().unwrap().cells()
    );
    let end = end.unwrap();
    assert!(end["report"]["regeneration"]["saved"].as_bool().unwrap());
    assert_eq!(
        end["report"]["regeneration"]["key"].as_str().unwrap().len(),
        64
    );
    let (warm, _, reports) = execute(m, directory.options());
    assert!(
        reports
            .iter()
            .all(|r| r["report"]["regeneration"]["cache_hit"] == true)
    );
    assert_eq!(
        warm.stock().unwrap().cells(),
        producer.stock().unwrap().cells()
    );
}

#[test]
fn cold_prefix_without_definite_removal_flushes_at_bound_and_advisory_hints_cannot_authorize_hits()
{
    use camlib::stream::StreamReceiver;
    let directory = Directory::new();
    let mut m = manifest();
    let Operation::Facing(op) = &mut m.operations[0] else {
        panic!("face")
    };
    op.floor_z_mm = m.stock.height().unwrap().max_mm[2] - 1e-6;
    op.stepover_mm = 0.25;
    let mut receiver = StreamReceiver::new(m.clone()).unwrap();
    let mut producer = StreamProducer::new(m.clone()).unwrap();
    let mut checked = false;
    regeneration::run(&mut producer, directory.options(), &|| false, |frame| {
        if u16::from_le_bytes([frame[10], frame[11]]) == 1 {
            receiver.accept(frame, DEFAULT_FRAME_LIMIT)?;
            if receiver.completed() == 33 {
                checked = true;
                assert!(
                    std::fs::read_dir(directory.0.join("camlib-regeneration-v1"))
                        .unwrap()
                        .any(|e| e.unwrap().path().extension().is_some_and(|x| x == "tmp")),
                    "long air prefix exceeded record bound without initializing writer"
                );
            }
        }
        Ok(())
    })
    .unwrap();
    assert!(checked);
    assert_eq!(producer.stock().unwrap().volume().removed_lower_mm3, 0.);
    // Keep advisory hints but remove the real index/artifact. The next run
    // must freshly plan and verify; a hint never substitutes for a cache key.
    for entry in std::fs::read_dir(directory.0.join("camlib-regeneration-v1")).unwrap() {
        let path = entry.unwrap().path();
        if !path.to_string_lossy().ends_with(".input-hint") {
            std::fs::remove_file(path).unwrap();
        }
    }
    let (_, _, reports) = execute(m, directory.options());
    assert!(
        reports
            .iter()
            .all(|r| r["report"]["regeneration"]["cache_hit"] == false)
    );
}

/// Two facing operations, the second with a tool 40 mm longer (gauge
/// lengths 60 and 100 mm), on a machine that changes tools where the spindle
/// stands; `retract`: the stated gauge Z of the retract before the change.
fn in_place_manifest(retract: Option<f64>) -> StreamManifest {
    let mut m = manifest();
    assert_eq!(m.tools[0].gauge_length_mm, 60.);
    let mut long = m.tools[0].clone();
    long.id = "long".into();
    long.gauge_length_mm = 100.;
    long.holders[0].z_max_mm = 100.;
    m.tools.push(long);
    let mut second = m.operations[0].clone();
    let Operation::Facing(p) = &mut second else {
        panic!("fixture")
    };
    p.id = "second".into();
    p.tool = "long".into();
    m.operations.push(second);
    m.tool_change = Some(ToolChangePose::InPlace(InPlacePose {
        in_place: true,
        retract_gauge_z_mm: retract,
    }));
    m
}
/// (motions, the motion count at each operation's end) of a run's frames,
/// as a consumer accepts them.
fn consumed(m: &StreamManifest, frames: &[Vec<u8>]) -> (Vec<camlib::motion::Motion>, Vec<u64>) {
    let mut receiver = StreamReceiver::new(m.clone()).unwrap();
    let (mut motions, mut ends) = (vec![], vec![]);
    for frame in frames {
        match u16::from_le_bytes([frame[10], frame[11]]) {
            1 => motions.push(receiver.accept(frame, DEFAULT_FRAME_LIMIT).unwrap()),
            4 => receiver.accept_end(frame, DEFAULT_FRAME_LIMIT).unwrap(),
            7 => {
                let control = decode_operation_control(m, frame, DEFAULT_FRAME_LIMIT).unwrap();
                if control["stage"] == "end" {
                    ends.push(control["completed"].as_u64().unwrap());
                }
            }
            _ => {}
        }
    }
    assert!(receiver.is_finished());
    (motions, ends)
}
fn uncached(m: &StreamManifest) -> Result<Vec<Vec<u8>>, String> {
    let mut producer = StreamProducer::new(m.clone())?;
    let mut frames = vec![];
    regeneration::run_uncached(&mut producer, &AtomicBool::new(false), |frame| {
        frames.push(frame.to_vec());
        Ok(())
    })?;
    Ok(frames)
}

#[test]
fn a_stated_retract_is_planned_before_a_tool_change_in_place() {
    use camlib::motion::{Curve, MoveKind};
    // Gauge Z 110: the 60 mm tool's tip goes to Z 50, the 100 mm tool is
    // changed in with its tip at Z 10, above its clearance 8.
    let m = in_place_manifest(Some(110.));
    m.validate().unwrap();
    let (motions, ends) = consumed(&m, &uncached(&m).unwrap());
    let first: Vec<_> = motions.iter().filter(|x| x.operation == "face").collect();
    let retract = first.last().unwrap();
    let before = first[first.len() - 2].curve.end();
    assert_eq!(before[2], 8.);
    assert_eq!(
        (retract.kind, retract.feed_mm_min, retract.tool.as_str()),
        (MoveKind::Rapid, None, "face8")
    );
    assert_eq!(
        retract.curve,
        Curve::Line {
            start: before,
            end: [before[0], before[1], 50.]
        }
    );
    // It is the first operation's last motion, before its end.
    assert_eq!(ends[0], retract.id);
    let second = motions.iter().find(|x| x.operation == "second").unwrap();
    assert_eq!(second.id, retract.id + 1);
    assert_eq!(second.curve.start(), [before[0], before[1], 10.]);
    // Only before a change: the last operation ends where it stopped.
    assert_ne!(motions.last().unwrap().curve.end()[2], 50.);

    // The same job with its first clearance raised by hand plans without
    // a retract; stating one adds exactly that motion to the first operation.
    let by_hand = |retract| {
        let mut m = in_place_manifest(retract);
        m.initial_position_mm = [0., 0., 48.];
        let Operation::Facing(p) = &mut m.operations[0] else {
            panic!("fixture")
        };
        p.clearance_z_mm = 48.;
        let (motions, _) = consumed(&m, &uncached(&m).unwrap());
        motions
            .into_iter()
            .filter(|x| x.operation == "face")
            .map(|x| (x.kind, x.curve))
            .collect::<Vec<_>>()
    };
    let (plain, stated) = (by_hand(None), by_hand(Some(110.)));
    assert_eq!(stated[..stated.len() - 1], plain[..]);
    // Already exactly at the stated gauge line: no motion.
    assert_eq!(by_hand(Some(108.)), plain);

    // With a regeneration cache, cold and warm, the run is the same: the
    // retract is never part of an operation's cache entry.
    let directory = Directory::new();
    let (_, cold, reports) = execute(m.clone(), directory.options());
    assert!(
        reports
            .iter()
            .all(|r| r["report"]["regeneration"]["cache_hit"] == false)
    );
    let (_, warm, reports) = execute(m.clone(), directory.options());
    assert!(
        reports
            .iter()
            .all(|r| r["report"]["regeneration"]["cache_hit"] == true)
    );
    for frames in [cold, warm] {
        let (again, again_ends) = consumed(&m, &frames);
        assert_eq!(again, motions);
        assert_eq!(again_ends, ends);
    }
    // The first operation's entry also serves a job without the retract.
    let mut one = in_place_manifest(None);
    one.operations.truncate(1);
    let (_, _, reports) = execute(one, directory.options());
    assert_eq!(reports[0]["report"]["regeneration"]["cache_hit"], true);
}

#[test]
fn a_change_in_place_is_refused_as_a_tool_change_with_and_without_a_retract() {
    // Without a retract the 100 mm tool's tip arrives 40 mm lower, at Z -32.
    let error = uncached(&in_place_manifest(None)).unwrap_err();
    assert!(
        error.starts_with(
            "operation second cannot start after its tool change (start Z -32 is below clearance_z_mm 8): the tool change from face8 to long leaves the spindle where it stands, so the longer tool's tip is at Z -32, 40 mm below the Z 8 where face8 stopped: state a retract before the change (ToolChangeInPlace(retract_gauge_z_mm=...))"
        ),
        "{error}"
    );
    assert!(!error.contains("invalid"), "{error}");
    // A retract too low for the longer tool: gauge Z 105 leaves its tip at Z 5.
    let error = uncached(&in_place_manifest(Some(105.))).unwrap_err();
    assert!(
        error.starts_with("operation second cannot start after its tool change (start Z 5 is below clearance_z_mm 8): the tool change from face8 to long is made in place after the stated retract to gauge Z 105 ")
            && error.contains("leaves long's tip at Z 5: state a retract_gauge_z_mm of at least the operation's clearance_z_mm plus long's gauge length 100"),
        "{error}"
    );
    // A retract never lowers the tool: gauge Z 60 is below the gauge line
    // (Z 68) where the first operation left it.
    let error = uncached(&in_place_manifest(Some(60.))).unwrap_err();
    assert!(
        error.starts_with("tool change from face8 to long in place: the stated retract to gauge Z 60 ")
            && error.contains("would lower face8's tip to Z 0, below the Z 8 where operation face left it")
            && error.contains("at least 68"),
        "{error}"
    );
    // The cached run refuses the same.
    let directory = Directory::new();
    for retract in [None, Some(105.), Some(60.)] {
        let m = in_place_manifest(retract);
        let mut producer = StreamProducer::new(m.clone()).unwrap();
        let cached = regeneration::run(
            &mut producer,
            directory.options(),
            &AtomicBool::new(false),
            |_| Ok(()),
        )
        .unwrap_err();
        assert_eq!(cached, uncached(&m).unwrap_err());
    }
    // An unrelated refusal of the operation after a change keeps its own
    // words (the change is only noted).
    let mut m = in_place_manifest(None);
    let Operation::Facing(p) = &mut m.operations[1] else {
        panic!("fixture")
    };
    p.feed_mm_min = -1.;
    let error = uncached(&m).unwrap_err();
    assert!(
        !error.contains("cannot start after its tool change")
            && error.contains("leaves the spindle where it stands"),
        "{error}"
    );
    // After a stated retract the change is not its likely cause.
    m.tool_change = in_place_manifest(Some(110.)).tool_change;
    let error = uncached(&m).unwrap_err();
    assert!(!error.contains("tool change"), "{error}");

    // The producer places a change with a stated retract only from where
    // that retract leaves the old tool, and plans the retract itself.
    let m = in_place_manifest(Some(110.));
    let cancel = AtomicBool::new(false);
    let mut p = StreamProducer::new(m).unwrap();
    assert_eq!(p.change_tool("face8").unwrap(), [0., 0., 8.]);
    assert_eq!(p.tool_change_retract("face", "face8", "face8").unwrap(), None);
    let retract = p
        .tool_change_retract("face", "face8", "long")
        .unwrap()
        .unwrap();
    let error = p.change_tool("long").unwrap_err();
    assert!(error.contains("made no move"), "{error}");
    let mut skipped = retract.clone();
    skipped.curve = camlib::motion::Curve::Line {
        start: [0., 0., 8.],
        end: [0., 0., 49.],
    };
    p.push_checked_motion(skipped, DEFAULT_FRAME_LIMIT, &cancel)
        .unwrap();
    let error = p.change_tool("long").unwrap_err();
    assert!(
        error.contains("the stated retract to gauge Z 110 was not made (face8's tip is at Z 49, not Z 50)"),
        "{error}"
    );
    let rest = p
        .tool_change_retract("face", "face8", "long")
        .unwrap()
        .unwrap();
    assert_eq!((rest.id, rest.curve.end()), (2, [0., 0., 50.]));
    p.push_checked_motion(rest, DEFAULT_FRAME_LIMIT, &cancel)
        .unwrap();
    assert_eq!(p.change_tool("long").unwrap(), [0., 0., 10.]);
}

#[test]
fn a_stated_end_position_is_planned_after_the_last_operation() {
    use camlib::motion::{Curve, MoveKind};
    // Two facing operations with one tool (gauge length 60 mm), ending at
    // their clearance Z 8 (gauge line Z 68).
    let job = |end: Option<GaugePose>| {
        let mut m = manifest();
        let mut second = m.operations[0].clone();
        let Operation::Facing(p) = &mut second else {
            panic!("fixture")
        };
        p.id = "second".into();
        m.operations.push(second);
        m.end_position = end;
        m
    };
    let plain = job(None);
    assert!(!serde_json::to_string(&plain).unwrap().contains("end_position"));
    let (plain_motions, plain_ends) = consumed(&plain, &uncached(&plain).unwrap());
    let stop = plain_motions.last().unwrap().curve.end();
    assert_eq!(stop[2], 8.);

    let stated = job(Some(GaugePose {
        gauge_z_mm: 100.,
        xy_mm: Some([-5., 20.]),
    }));
    stated.validate().unwrap();
    let text = serde_json::to_string(&stated).unwrap();
    assert!(text.contains(r#""end_position":{"gauge_z_mm":100.0,"xy_mm":[-5.0,20.0]}"#));
    assert_eq!(
        serde_json::from_str::<StreamManifest>(&text)
            .unwrap()
            .end_position,
        stated.end_position
    );
    let (motions, ends) = consumed(&stated, &uncached(&stated).unwrap());
    // Everything up to the last operation's own end is the same plan.
    assert_eq!(motions[..plain_motions.len()], plain_motions[..]);
    assert_eq!(ends[0], plain_ends[0]);
    // Then a Z rapid to gauge Z 100 (tip Z 40) and an XY rapid, both
    // motions of the last operation, before its end.
    let tail: Vec<_> = motions[plain_motions.len()..].iter().collect();
    assert_eq!(tail.len(), 2);
    assert_eq!(
        tail[0].curve,
        Curve::Line {
            start: stop,
            end: [stop[0], stop[1], 40.]
        }
    );
    assert_eq!(
        tail[1].curve,
        Curve::Line {
            start: [stop[0], stop[1], 40.],
            end: [-5., 20., 40.]
        }
    );
    for (k, motion) in tail.iter().enumerate() {
        assert_eq!(
            (motion.kind, motion.feed_mm_min, motion.operation.as_str()),
            (MoveKind::Rapid, None, "second")
        );
        assert_eq!(motion.id, (plain_motions.len() + 1 + k) as u64);
    }
    assert_eq!(ends[1], motions.len() as u64);

    // Z only; already at that height (XY only); already there (nothing).
    let tails = |end| {
        let m = job(Some(end));
        let (motions, _) = consumed(&m, &uncached(&m).unwrap());
        motions[plain_motions.len()..]
            .iter()
            .map(|x| x.curve.end())
            .collect::<Vec<_>>()
    };
    assert_eq!(
        tails(GaugePose {
            gauge_z_mm: 100.,
            xy_mm: None
        }),
        [[stop[0], stop[1], 40.]]
    );
    assert_eq!(
        tails(GaugePose {
            gauge_z_mm: 68.,
            xy_mm: Some([-5., 20.])
        }),
        [[-5., 20., 8.]]
    );
    assert!(
        tails(GaugePose {
            gauge_z_mm: 68.,
            xy_mm: Some([stop[0], stop[1]])
        })
        .is_empty()
    );

    // The end move only goes up.
    let low = job(Some(GaugePose {
        gauge_z_mm: 67.,
        xy_mm: None,
    }));
    let error = uncached(&low).unwrap_err();
    assert!(
        error.starts_with("end position: the stated gauge Z 67 (EndPosition(gauge_z_mm=67)) would lower face8's tip to Z 7, below the Z 8 where the last operation second left it")
            && error.ends_with("state a gauge_z_mm of at least 68"),
        "{error}"
    );

    // With a regeneration cache, cold and warm, the run is the same, and
    // the operations' entries are the operations alone: they also serve
    // the job without an end position.
    let directory = Directory::new();
    let (_, cold, _) = execute(stated.clone(), directory.options());
    let (_, warm, reports) = execute(stated.clone(), directory.options());
    assert!(
        reports
            .iter()
            .all(|r| r["report"]["regeneration"]["cache_hit"] == true)
    );
    for frames in [cold, warm] {
        assert_eq!(consumed(&stated, &frames), (motions.clone(), ends.clone()));
    }
    let (_, frames, reports) = execute(plain.clone(), directory.options());
    assert!(
        reports
            .iter()
            .all(|r| r["report"]["regeneration"]["cache_hit"] == true)
    );
    assert_eq!(consumed(&plain, &frames), (plain_motions, plain_ends));
}

#[test]
fn an_end_position_is_only_for_planned_milling_runs() {
    let end = Some(GaugePose {
        gauge_z_mm: 100.,
        xy_mm: None,
    });
    let mut m = manifest();
    m.end_position = end.clone();
    m.validate().unwrap();
    // A run that is given its moves has no last operation to end after.
    let mut given = m.clone();
    given.operations.clear();
    let error = given.validate().unwrap_err();
    assert!(error.contains("an end position is planned after the last operation of a milling operation run"), "{error}");
    // A cutting table has no spindle gauge line.
    let mut table = m.clone();
    table.tools[0].cutter.shape = camlib::tool::CutterShape::Beam;
    let error = table.validate().unwrap_err();
    assert!(
        error.contains("a cutting table has no spindle gauge line"),
        "{error}"
    );
    for bad in [f64::NAN, f64::INFINITY, 1e7] {
        let mut m = m.clone();
        m.end_position = Some(GaugePose {
            gauge_z_mm: bad,
            xy_mm: None,
        });
        assert!(m.validate().unwrap_err().contains("end position needs finite"));
        m.end_position = Some(GaugePose {
            gauge_z_mm: 100.,
            xy_mm: Some([0., bad]),
        });
        assert!(m.validate().unwrap_err().contains("end position needs finite"));
    }
}
