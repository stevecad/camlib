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
