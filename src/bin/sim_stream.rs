//! Bounded JSON setup/motion batches in; compact committed stock/motion frames out.
//! Process ownership provides cancellation; a blocked output pipe is backpressure.
use camlib::{
    motion::{Event, Motion},
    stream::*,
};
use serde::Deserialize;
use std::{
    io::{self, BufRead, Read, Seek, SeekFrom, Write},
    sync::atomic::AtomicBool,
};

#[derive(Deserialize)]
#[serde(tag = "command", rename_all = "snake_case", deny_unknown_fields)]
enum Command {
    Events {
        events: Vec<Event>,
    },
    Motions {
        motions: Vec<Motion>,
    },
    Material {
        after_operation: String,
    },
    Partial {
        motion: Motion,
        fraction: f64,
    },
    Restore {
        path: std::path::PathBuf,
    },
    Cache {
        path: std::path::PathBuf,
        offset: u64,
        length: u64,
    },
    Checkpoint,
    Operations {
        #[serde(default)]
        cache: Option<camlib::regeneration::CacheOptions>,
        /// Planning progress frames (#198): at this interval in
        /// milliseconds, or 0 for whenever the planner's counts change.
        /// Absent: none, and the output is the run's frames only.
        #[serde(default)]
        progress_ms: Option<u64>,
    },
    Finish,
}
fn load_checkpoint(
    manifest: &StreamManifest,
    path: &std::path::Path,
    query: bool,
) -> Result<StreamProducer, String> {
    let file = std::fs::File::open(path).map_err(|e| e.to_string())?;
    if file.metadata().map_err(|e| e.to_string())?.len() > CHECKPOINT_FRAME_LIMIT as u64 {
        return Err("checkpoint file exceeds byte budget".into());
    }
    let mut reader = io::BufReader::with_capacity(65536, file);
    if query {
        StreamProducer::query_from_checkpoint_reader(
            manifest.clone(),
            &mut reader,
            CHECKPOINT_FRAME_LIMIT,
        )
    } else {
        StreamProducer::from_checkpoint_reader(
            manifest.clone(),
            &mut reader,
            CHECKPOINT_FRAME_LIMIT,
        )
    }
}
fn line(reader: &mut impl BufRead) -> Result<Option<Vec<u8>>, String> {
    let mut bytes = Vec::new();
    // Requests come from the owning process; read the whole line.
    reader
        .read_until(b'\n', &mut bytes)
        .map_err(|e| e.to_string())?;
    if bytes.is_empty() {
        return Ok(None);
    }
    if bytes.last() != Some(&b'\n') {
        return Err("input line lacks newline".into());
    }
    Ok(Some(bytes))
}
fn run() -> Result<(), String> {
    let mut input = io::stdin().lock();
    let setup = line(&mut input)?.ok_or("stream requires manifest as first JSON line")?;
    let manifest: StreamManifest = serde_json::from_slice(&setup).map_err(|e| e.to_string())?;
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    let query = args.len() == 1 && args[0] == "--query";
    let producer = if args.len() == 2 && args[0] == "--resume" {
        load_checkpoint(&manifest, std::path::Path::new(&args[1]), false)?
    } else if !args.is_empty() && !query {
        return Err("expected --query, --resume CHECKPOINT_FILE or no arguments".into());
    } else if query {
        StreamProducer::new_query(manifest.clone())?
    } else {
        StreamProducer::new(manifest.clone())?
    };
    let mut producer = Some(producer);
    // Locked for each frame, not for the run: a run that reports planning
    // progress writes those frames from its timer thread.
    let output = io::stdout();
    let send = |bytes: &[u8]| -> Result<(), String> {
        let mut output = output.lock();
        output
            .write_all(bytes)
            .and_then(|()| output.flush())
            .map_err(|e| e.to_string())
    };
    send(&encode_manifest(
        &manifest,
        camlib::stream::MANIFEST_LIMIT + 20,
    )?)?;
    let cancel = AtomicBool::new(false);
    while let Some(bytes) = line(&mut input)? {
        let command: Command = serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
        match command {
            Command::Events { events } => {
                if query || manifest.version != 2 || !manifest.operations.is_empty() {
                    return Err("typed events require a writable version 2 authored run".into());
                }
                if events.len() > 4096 {
                    return Err("event batch exceeds 4096 events".into());
                }
                for event in events {
                    let producer = producer.as_mut().ok_or("producer unavailable")?;
                    if let Event::Motion { motion } = event {
                        let (chunk, check) =
                            producer.push_checked_motion(motion, DEFAULT_FRAME_LIMIT, &cancel)?;
                        send(&chunk)?;
                        if let Some(check) = check {
                            send(&check)?;
                        }
                    } else {
                        send(&producer.push_control(event, DEFAULT_FRAME_LIMIT, &cancel)?)?;
                    }
                }
            }
            Command::Restore { path } => {
                if !query {
                    return Err("restore requires a read-only query worker".into());
                }
                // Only a disposable query cache is dropped. The actual producer
                // and displayed stock remain in their separate owned states.
                drop(producer.take());
                let loaded = load_checkpoint(&manifest, &path, true)?;
                send(&loaded.restored_cursor(DEFAULT_FRAME_LIMIT)?)?;
                producer = Some(loaded);
            }
            Command::Cache {
                path,
                offset,
                length,
            } => {
                if !query {
                    return Err("cached chunks require a read-only query worker".into());
                }
                if length == 0 || length > DEFAULT_FRAME_LIMIT as u64 {
                    return Err("query cache span exceeds byte budget".into());
                }
                let mut file = std::fs::File::open(path).map_err(|e| e.to_string())?;
                if offset
                    .checked_add(length)
                    .is_none_or(|end| end > file.metadata().map(|m| m.len()).unwrap_or(0))
                {
                    return Err("query cache span lies outside retained file".into());
                }
                file.seek(SeekFrom::Start(offset))
                    .map_err(|e| e.to_string())?;
                let mut span = io::BufReader::with_capacity(65536, file.take(length));
                let mut count = 0;
                while let Some(frame) = read_frame(&mut span, DEFAULT_FRAME_LIMIT)? {
                    count += 1;
                    if count > 4096 {
                        return Err("query cache span exceeds motion budget".into());
                    }
                    producer
                        .as_mut()
                        .ok_or("query cache unavailable")?
                        .accept_cached_chunk(&frame, DEFAULT_FRAME_LIMIT)?;
                }
                if count == 0 || span.into_inner().limit() != 0 {
                    return Err("truncated query cache span".into());
                }
                send(
                    &producer
                        .as_ref()
                        .ok_or("query cache unavailable")?
                        .restored_cursor(DEFAULT_FRAME_LIMIT)?,
                )?;
            }
            Command::Partial { motion, fraction } => send(
                &producer
                    .as_ref()
                    .ok_or("query cache unavailable")?
                    .preview_motion(&motion, fraction, DEFAULT_FRAME_LIMIT, &cancel)?,
            )?,
            Command::Motions { motions } => {
                if query {
                    return Err("read-only query worker cannot commit motions".into());
                }
                if !manifest.operations.is_empty() {
                    return Err(
                        "manifest-bound operations require native operation generation".into(),
                    );
                }
                if motions.len() > 4096 {
                    return Err("motion batch exceeds 4096 moves".into());
                }
                if manifest.turning.is_some() {
                    // Given lathe moves (a posted program read back): the
                    // turning kernel checks each and computes its stock.
                    for motion in motions {
                        let chunk = producer
                            .as_mut()
                            .ok_or("producer unavailable")?
                            .push_turn_replayed(motion, DEFAULT_FRAME_LIMIT, &cancel)?;
                        send(&chunk)?;
                    }
                    continue;
                }
                for motion in motions {
                    let (chunk, check) = producer
                        .as_mut()
                        .ok_or("producer unavailable")?
                        .push_checked_motion(motion, DEFAULT_FRAME_LIMIT, &cancel)?;
                    send(&chunk)?;
                    if let Some(check) = check {
                        send(&check)?;
                    }
                }
            }
            Command::Material { after_operation } => {
                if query || !manifest.operations.is_empty() {
                    return Err(
                        "explicit material stages require writable explicit-motion run".into(),
                    );
                }
                let p = producer.as_mut().ok_or("producer unavailable")?;
                let mut found = false;
                while let Some(started) = p.start_material_verification(Some(&after_operation))? {
                    found = true;
                    send(&started)?;
                    send(&p.finish_material_verification(&cancel)?)?;
                }
                if !found {
                    return Err("material stage is not configured or already complete".into());
                }
            }
            Command::Operations { cache, progress_ms } => {
                if query
                    || producer.as_ref().ok_or("producer unavailable")?.completed() != 0
                    || manifest.operations.is_empty() && manifest.turning.is_none()
                {
                    return Err("operations require an unstarted writable run with manifest-bound operations".into());
                }
                if let Some(ms) = progress_ms {
                    camlib::regeneration::run_with_progress(
                        producer.as_mut().ok_or("producer unavailable")?,
                        cache,
                        &cancel,
                        match ms {
                            0 => camlib::progress::Cadence::EveryChange,
                            ms => camlib::progress::Cadence::Interval(
                                std::time::Duration::from_millis(ms),
                            ),
                        },
                        send,
                    )?;
                    return Ok(());
                }
                if manifest.turning.is_some() {
                    // Lathe runs are planned and streamed by their turning
                    // planner (no regeneration cache).
                    camlib::regeneration::run_turning(
                        producer.as_mut().ok_or("producer unavailable")?,
                        &cancel,
                        send,
                    )?;
                    return Ok(());
                }
                if let Some(cache) = cache {
                    camlib::regeneration::run(
                        producer.as_mut().ok_or("producer unavailable")?,
                        cache,
                        &cancel,
                        send,
                    )?;
                    return Ok(());
                }
                camlib::regeneration::run_uncached(
                    producer.as_mut().ok_or("producer unavailable")?,
                    &cancel,
                    send,
                )?;
                return Ok(());
            }
            Command::Checkpoint => send(
                &producer
                    .as_ref()
                    .ok_or("producer unavailable")?
                    .checkpoint(CHECKPOINT_FRAME_LIMIT)?,
            )?,
            Command::Finish => {
                if !manifest.operations.is_empty() {
                    return Err(
                        "manifest-bound operation job requires operation generation before finish"
                            .into(),
                    );
                }
                let producer = producer.as_mut().ok_or("producer unavailable")?;
                while let Some(start) = producer.start_material_verification(None)? {
                    send(&start)?;
                    send(&producer.finish_material_verification(&cancel)?)?;
                }
                send(&producer.finish(DEFAULT_FRAME_LIMIT)?)?;
                return Ok(());
            }
        }
    }
    Err("input ended without explicit finish; committed prefix remains partial".into())
}
fn main() {
    if let Err(error) = run() {
        eprintln!("simulation stream: {error}");
        std::process::exit(1);
    }
}
