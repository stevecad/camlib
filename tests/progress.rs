//! Planning progress (#198): the updates a run sends while it plans, and that
//! they are no part of the run.
use camlib::{
    progress::{Basis, Cadence, Progress},
    regeneration,
    stream::*,
};
use std::{sync::atomic::AtomicBool, time::Duration};

fn fixture(source: &str) -> StreamManifest {
    serde_json::from_str(source).unwrap()
}
/// The run's frames as sent, with its progress frames in place.
fn run(manifest: &StreamManifest, progress: Option<Cadence>) -> Vec<Vec<u8>> {
    let mut producer = StreamProducer::new(manifest.clone()).unwrap();
    let cancel = AtomicBool::new(false);
    let mut frames = vec![];
    let send = |frame: &[u8]| {
        frames.push(frame.to_vec());
        Ok(())
    };
    match progress {
        Some(cadence) => {
            regeneration::run_with_progress(&mut producer, None, &cancel, cadence, send)
        }
        None => regeneration::run_uncached(&mut producer, &cancel, send),
    }
    .unwrap();
    frames
}
fn kind(frame: &[u8]) -> u16 {
    u16::from_le_bytes([frame[10], frame[11]])
}
/// The progress updates of a run and what is left without them.
fn split(manifest: &StreamManifest, frames: Vec<Vec<u8>>) -> (Vec<Progress>, Vec<Vec<u8>>) {
    let mut updates = vec![];
    let mut rest = vec![];
    for frame in frames {
        if is_progress(&frame) {
            assert_eq!(kind(&frame), 12);
            updates.push(decode_progress(manifest, &frame).unwrap());
        } else {
            rest.push(frame);
        }
    }
    (updates, rest)
}
/// The acceptance of #198 on one operation's updates: no fraction before the
/// planner states one, then one that never decreases and ends at 1.
fn check_one_operation(updates: &[Progress], id: &str, basis: Basis) {
    assert!(updates.len() >= 3, "{} updates", updates.len());
    for update in updates {
        assert_eq!(
            (update.operation.as_str(), update.index, update.operations),
            (id, 0, 1)
        );
    }
    assert_eq!((updates[0].basis, updates[0].fraction), (None, None));
    let mut previous = 0.;
    for update in &updates[1..] {
        assert_eq!(update.basis, Some(basis));
        let fraction = update.fraction.unwrap();
        assert!(
            (previous..=1.).contains(&fraction),
            "{previous} then {fraction}"
        );
        previous = fraction;
        assert!(update.done.unwrap() <= update.total.unwrap());
    }
    let last = updates.last().unwrap();
    assert_eq!(last.fraction, Some(1.));
    assert_eq!(last.done, last.total);
}

/// Two depth levels of adaptive clearing with the engagement front, with a
/// 6 mm tool: a few seconds instead of the fixture's minute.
fn adaptive() -> StreamManifest {
    let mut value: serde_json::Value = serde_json::from_str(include_str!(
        "../fixtures/operations/roughing-adaptive.json"
    ))
    .unwrap();
    let operation = &mut value["operations"][0];
    assert_eq!(operation["clearing"]["path_strategy"], "engagement_front");
    operation["floor_z_mm"] = 6.0.into();
    operation["clearing"]["max_radial_engagement_mm"] = 1.0.into();
    let cutter = &mut value["tools"][0]["cutter"];
    cutter["diameter_mm"] = 6.0.into();
    cutter["shank_diameter_mm"] = 6.0.into();
    serde_json::from_value(value).unwrap()
}

#[test]
fn roughing_reports_layers_done_of_layers_planned() {
    let manifest = fixture(include_str!("../fixtures/operations/roughing.json"));
    let plain = run(&manifest, None);
    let (updates, rest) = split(&manifest, run(&manifest, Some(Cadence::EveryChange)));
    // Progress is no part of the run: without its frames, the same bytes.
    assert_eq!(rest, plain);
    assert!(plain.iter().all(|frame| !is_progress(frame)));
    check_one_operation(&updates, "rough", Basis::Layers);
    // Depth levels 10 -> 2 mm in 2 mm steps: each count once, in order.
    let total = updates[1].total.unwrap();
    assert_eq!(total, 4);
    let done: Vec<_> = updates[1..].iter().map(|u| u.done.unwrap()).collect();
    assert_eq!(done, (0..=total).collect::<Vec<_>>());
    for update in &updates[1..] {
        assert_eq!(
            update.fraction,
            Some(update.done.unwrap() as f64 / total as f64)
        );
        assert_eq!(update.area_left_mm2, None);
    }
}

#[test]
fn adaptive_clearing_reports_the_area_left_in_each_layer() {
    let manifest = adaptive();
    let plain = run(&manifest, None);
    let sent = run(&manifest, Some(Cadence::EveryChange));
    // The first update precedes every frame of the operation.
    assert!(is_progress(&sent[0]));
    let (updates, rest) = split(&manifest, sent);
    assert_eq!(rest, plain);
    check_one_operation(&updates, "rough", Basis::Area);
    let total = updates[1].total.unwrap();
    assert_eq!(total, 2);
    let mut within = 0;
    for layer in 0..total {
        let areas: Vec<(f64, f64)> = updates
            .iter()
            .filter(|u| u.done == Some(layer))
            .filter_map(|u| u.area_left_mm2.zip(u.area_start_mm2))
            .collect();
        // Every layer is refreshed at its start and as it empties.
        assert!(areas.len() >= 2, "layer {layer}: {areas:?}");
        let start = areas[0].1;
        assert!(start > 0. && areas[0].0 == start);
        for &(left, of) in &areas {
            assert!((0. ..=of).contains(&left) && of >= start);
        }
        assert!(areas.last().unwrap().0 < 0.5 * start);
        // Inside a layer the fraction is the layer's share of the area.
        for update in updates.iter().filter(|u| u.done == Some(layer)) {
            let Some((left, of)) = update.area_left_mm2.zip(update.area_start_mm2) else {
                continue;
            };
            let stated = (layer as f64 + 1. - left / of) / total as f64;
            let fraction = update.fraction.unwrap();
            // Never below what the counts state; above only where held.
            assert!(fraction >= stated - 1e-12, "{fraction} < {stated}");
            assert!(fraction <= (layer + 1) as f64 / total as f64);
            within += usize::from(fraction > layer as f64 / total as f64 && left > 0.);
        }
    }
    assert!(within >= total, "{within} updates inside a layer");
}

#[test]
fn surface_finishing_reports_passes_done_of_passes_planned() {
    let manifest = fixture(include_str!("../fixtures/operations/parallel.json"));
    let plain = run(&manifest, None);
    let (updates, rest) = split(&manifest, run(&manifest, Some(Cadence::EveryChange)));
    assert_eq!(rest, plain);
    check_one_operation(&updates, "finish-roof", Basis::Passes);
    let total = updates[1].total.unwrap();
    assert!(total > 4, "{total} passes");
    let done: Vec<_> = updates[1..].iter().map(|u| u.done.unwrap()).collect();
    assert_eq!(done, (0..=total).collect::<Vec<_>>());
}

#[test]
fn an_operation_with_nothing_to_count_reports_no_fraction() {
    let manifest = fixture(include_str!("../fixtures/operations/holes.json"));
    let plain = run(&manifest, None);
    let sent = run(&manifest, Some(Cadence::EveryChange));
    // Each operation's update comes before its begin control (kind 7).
    let mut begun = 0;
    for pair in sent.windows(2) {
        if is_progress(&pair[0]) {
            assert_eq!(kind(&pair[1]), 7);
            begun += 1;
        }
    }
    let (updates, rest) = split(&manifest, sent);
    assert_eq!(rest, plain);
    assert_eq!(begun, updates.len());
    let ids: Vec<_> = manifest.operations.iter().map(|o| o.id()).collect();
    assert_eq!(updates.len(), ids.len());
    for (index, update) in updates.iter().enumerate() {
        assert_eq!(
            (update.operation.as_str(), update.index, update.operations),
            (ids[index], index, ids.len())
        );
        assert_eq!(
            (update.basis, update.fraction, update.done, update.total),
            (None, None, None, None)
        );
    }
}

#[test]
fn updates_at_an_interval_leave_the_run_unchanged() {
    let manifest = fixture(include_str!("../fixtures/operations/roughing.json"));
    let plain = run(&manifest, None);
    let (updates, rest) = split(
        &manifest,
        run(&manifest, Some(Cadence::Interval(Duration::from_millis(1)))),
    );
    assert_eq!(rest, plain);
    // How many arrive depends on the clock; what they say does not.
    let fractions: Vec<f64> = updates.iter().filter_map(|u| u.fraction).collect();
    assert!(fractions.windows(2).all(|pair| pair[0] <= pair[1]));
    assert_eq!(fractions.last(), Some(&1.));
    assert_eq!(updates[0].fraction, None);
}

#[test]
fn a_progress_frame_is_bound_to_its_run() {
    let manifest = fixture(include_str!("../fixtures/operations/holes.json"));
    let sent = run(&manifest, Some(Cadence::EveryChange));
    let frame = sent.iter().find(|frame| is_progress(frame)).unwrap();
    let mut other = manifest.clone();
    other.run.revision.push('x');
    assert!(decode_progress(&other, frame).is_err());
    assert!(decode_progress(&manifest, &sent[sent.len() - 1]).is_err());
}

#[test]
fn a_cached_run_reports_planning_and_then_reuse_without_a_fraction() {
    let manifest = fixture(include_str!("../fixtures/operations/roughing.json"));
    let directory = std::env::temp_dir().join(format!(
        "cam-progress-test-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    std::fs::create_dir(&directory).unwrap();
    let cached = |progress: Option<Cadence>| {
        let options = regeneration::CacheOptions {
            directory: directory.clone(),
            max_entry_bytes: 64 * 1024 * 1024,
            max_total_bytes: 1024 * 1024 * 1024,
            dependencies: Default::default(),
        };
        let mut producer = StreamProducer::new(manifest.clone()).unwrap();
        let cancel = AtomicBool::new(false);
        let mut frames = vec![];
        let send = |frame: &[u8]| {
            frames.push(frame.to_vec());
            Ok(())
        };
        match progress {
            Some(cadence) => regeneration::run_with_progress(
                &mut producer,
                Some(options),
                &cancel,
                cadence,
                send,
            ),
            None => regeneration::run(&mut producer, options, &cancel, send),
        }
        .unwrap();
        frames
    };
    let motions = |frames: &[Vec<u8>]| -> Vec<Vec<u8>> {
        frames.iter().filter(|f| kind(f) == 1).cloned().collect()
    };
    // Planned, and saved to the cache: layers, as without a cache.
    let (planned, cold) = split(&manifest, cached(Some(Cadence::EveryChange)));
    check_one_operation(&planned, "rough", Basis::Layers);
    // Reused: replayed, not planned, so one update without a fraction.
    let (reused, warm) = split(&manifest, cached(Some(Cadence::EveryChange)));
    assert_eq!(reused.len(), 1);
    assert_eq!((reused[0].basis, reused[0].fraction), (None, None));
    assert_eq!(reused[0].operation, "rough");
    // The same moves and stock changes, and the same frames as a reuse
    // that was not asked for progress.
    assert_eq!(motions(&warm), motions(&cold));
    assert_eq!(motions(&cold), motions(&run(&manifest, None)));
    let plain = cached(None);
    std::fs::remove_dir_all(&directory).unwrap();
    assert_eq!(warm.len(), plain.len());
    assert_eq!(motions(&warm), motions(&plain));
}
