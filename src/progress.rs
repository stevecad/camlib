//! Planning progress (#198): which operation a run is planning and how far
//! along it is, for an application to show a person.
//!
//! A progress update is not part of the run. It is a transient frame
//! (`stream` kind 12) the run's driver sends only when asked: it changes no
//! motion id, stock, checkpoint or posted program, and a recorded run never
//! stores it. A run streamed with progress and the same run streamed without
//! it are the same frames once the progress frames are dropped.
//!
//! The fraction is only what a planner can count. Each planner states where
//! it stands (`OperationPlan::progress`): depth levels done of those planned,
//! finishing passes done of those planned, or, for adaptive clearing, the
//! reachable material area left in the level against the area the level
//! started with. An operation that can count none of these reports no
//! fraction, never an estimate.
use crate::stock::CancelSignal;
use serde::{Deserialize, Serialize};
use std::{
    cell::{Cell, RefCell},
    sync::{
        Arc, Condvar, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    time::Duration,
};

/// What a fraction counts.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Basis {
    /// Depth levels done of depth levels planned.
    Layers,
    /// Depth levels done of those planned, and within the current level the
    /// reachable material area cleared of the area the level started with.
    Area,
    /// Finishing passes done of passes planned.
    Passes,
}

/// Where a planner stands, in its own counts.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Planned {
    Layers {
        done: usize,
        total: usize,
    },
    Passes {
        done: usize,
        total: usize,
    },
    /// Adaptive clearing. `area_mm2` is the current level's reachable
    /// material area (left, at the level's start) as of the planner's last
    /// candidate refresh; `None` before the level's first refresh.
    Area {
        layers_done: usize,
        layers: usize,
        area_mm2: Option<(f64, f64)>,
    },
}
impl Planned {
    pub fn basis(&self) -> Basis {
        match self {
            Self::Layers { .. } => Basis::Layers,
            Self::Passes { .. } => Basis::Passes,
            Self::Area { .. } => Basis::Area,
        }
    }
    /// (done, total) in whole levels or passes.
    pub fn counts(&self) -> (usize, usize) {
        match *self {
            Self::Layers { done, total } | Self::Passes { done, total } => (done, total),
            Self::Area {
                layers_done,
                layers,
                ..
            } => (layers_done, layers),
        }
    }
    /// The completed fraction these counts state, in 0..=1; `None` when
    /// nothing is planned to count against.
    pub fn fraction(&self) -> Option<f64> {
        let (done, total) = self.counts();
        if total == 0 {
            return None;
        }
        let done = done.min(total);
        // The share of the current level: only adaptive clearing has one.
        let within = match *self {
            Self::Area {
                area_mm2: Some((left, start)),
                ..
            } if done < total && start > 0. && left.is_finite() && start.is_finite() => {
                (1. - left / start).clamp(0., 1.)
            }
            _ => 0.,
        };
        Some(((done as f64 + within) / total as f64).clamp(0., 1.))
    }
    /// The same basis with every level or pass done.
    fn finished(&self) -> Self {
        match *self {
            Self::Layers { total, .. } => Self::Layers { done: total, total },
            Self::Passes { total, .. } => Self::Passes { done: total, total },
            Self::Area { layers, .. } => Self::Area {
                layers_done: layers,
                layers,
                area_mm2: None,
            },
        }
    }
}

/// One progress update, as sent (JSON in a kind 12 frame).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Progress {
    /// The operation being planned, and its index among the setup's
    /// `operations` operations.
    pub operation: String,
    pub index: usize,
    pub operations: usize,
    /// What `fraction` counts; `None` (with no fraction) where the planner
    /// has no honest measure.
    pub basis: Option<Basis>,
    /// Levels or passes done and planned.
    pub done: Option<usize>,
    pub total: Option<usize>,
    /// Completed fraction in 0..=1. It never decreases within an operation
    /// and is 1 in the operation's last update.
    pub fraction: Option<f64>,
    /// Adaptive clearing: the current level's reachable material area left
    /// and at the level's start.
    pub area_left_mm2: Option<f64>,
    pub area_start_mm2: Option<f64>,
}

/// When progress updates are sent.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Cadence {
    /// At each operation's start and end and at this interval between
    /// (1 ms to 1 s), also while a planner is busy before its first move.
    Interval(Duration),
    /// At each operation's start and end and whenever the planner's counts
    /// change (after a move): deterministic, and as many updates as changes.
    EveryChange,
}

/// Sets a flag at an interval. The planning thread reads the flag where it
/// already polls for cancellation, so no other thread touches the output.
struct Ticker {
    stop: Arc<(Mutex<bool>, Condvar)>,
    thread: Option<std::thread::JoinHandle<()>>,
}
impl Ticker {
    fn start(interval: Duration, due: Arc<AtomicBool>) -> Result<Self, String> {
        let stop = Arc::new((Mutex::new(false), Condvar::new()));
        let shared = stop.clone();
        let thread = std::thread::Builder::new()
            .name("camlib-progress".into())
            .spawn(move || {
                let (lock, wake) = &*shared;
                let mut stopped = lock.lock().unwrap_or_else(|e| e.into_inner());
                while !*stopped {
                    let (guard, timeout) = wake
                        .wait_timeout(stopped, interval)
                        .unwrap_or_else(|e| e.into_inner());
                    stopped = guard;
                    if timeout.timed_out() {
                        due.store(true, Ordering::Relaxed);
                    }
                }
            })
            .map_err(|e| {
                format!("planning progress at an interval needs a timer thread, which could not start ({e}); run without progress")
            })?;
        Ok(Self {
            stop,
            thread: Some(thread),
        })
    }
}
impl Drop for Ticker {
    fn drop(&mut self) {
        *self.stop.0.lock().unwrap_or_else(|e| e.into_inner()) = true;
        self.stop.1.notify_all();
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

type Sink<'a> = &'a mut dyn FnMut(&[u8]) -> Result<(), String>;
struct State<'a> {
    send: Sink<'a>,
    operation: String,
    index: usize,
    planned: Option<Planned>,
    fraction: Option<f64>,
    /// What the last update of this operation said.
    sent: Option<(Option<Planned>, Option<f64>)>,
    failure: Option<String>,
}

/// A run's output and its progress. Every frame of the run goes out through
/// `send`; given to a planner as its cancellation signal, it also sends a
/// progress update when one is due, from the planner's own cancellation
/// polls. The planners therefore need no further parameter, and cancellation
/// latency bounds how late an update can be.
pub(crate) struct Meter<'a, C: CancelSignal> {
    cancel: &'a C,
    identity: Vec<u8>,
    operations: usize,
    cadence: Option<Cadence>,
    due: Arc<AtomicBool>,
    failed: Cell<bool>,
    state: RefCell<State<'a>>,
    _ticker: Option<Ticker>,
}
impl<'a, C: CancelSignal> Meter<'a, C> {
    pub(crate) fn new(
        cancel: &'a C,
        manifest: &crate::stream::StreamManifest,
        operations: usize,
        cadence: Option<Cadence>,
        send: Sink<'a>,
    ) -> Result<Self, String> {
        let due = Arc::new(AtomicBool::new(false));
        let ticker = match cadence {
            Some(Cadence::Interval(interval)) => {
                if !(Duration::from_millis(1)..=Duration::from_secs(1)).contains(&interval) {
                    return Err(format!(
                        "planning progress interval {interval:?} is outside 1 ms to 1 s; give an interval in that range"
                    ));
                }
                Some(Ticker::start(interval, due.clone())?)
            }
            _ => None,
        };
        Ok(Self {
            cancel,
            identity: crate::stream::identity_bytes(manifest)?,
            operations,
            cadence,
            due,
            failed: Cell::new(false),
            state: RefCell::new(State {
                send,
                operation: String::new(),
                index: 0,
                planned: None,
                fraction: None,
                sent: None,
                failure: None,
            }),
            _ticker: ticker,
        })
    }
    /// Send one frame of the run.
    pub(crate) fn send(&self, bytes: &[u8]) -> Result<(), String> {
        (self.state.borrow_mut().send)(bytes)
    }
    /// Planning of operation `index` starts: its first update, without a
    /// fraction until the planner states one.
    pub(crate) fn begin(&self, operation: &str, index: usize) -> Result<(), String> {
        {
            let mut state = self.state.borrow_mut();
            state.operation = operation.into();
            state.index = index;
            state.planned = None;
            state.fraction = None;
            state.sent = None;
        }
        self.emit();
        self.result(Ok(()))
    }
    /// The planner's counts after preparing or after a move.
    pub(crate) fn report(&self, planned: Option<Planned>) -> Result<(), String> {
        if self.cadence.is_none() {
            return Ok(());
        }
        // Nothing planned to count against (no levels) is no basis.
        let planned = planned.filter(|p| p.fraction().is_some());
        let changed = {
            let mut state = self.state.borrow_mut();
            let changed = planned.is_some() && state.planned != planned;
            if changed {
                state.planned = planned;
                // The one place the fraction is made monotonic: adaptive
                // clearing may find more reachable material on a refresh.
                let fraction = planned.and_then(|p| p.fraction());
                state.fraction = match (state.fraction, fraction) {
                    (Some(a), Some(b)) => Some(a.max(b)),
                    (a, b) => b.or(a),
                };
            }
            changed
        };
        if changed && self.cadence == Some(Cadence::EveryChange) {
            self.emit();
        }
        self.result(Ok(()))
    }
    /// The operation is planned: the last update, complete, when the planner
    /// stated a basis.
    pub(crate) fn end(&self) -> Result<(), String> {
        {
            let mut state = self.state.borrow_mut();
            let Some(planned) = state.planned else {
                return Ok(());
            };
            state.planned = Some(planned.finished());
            state.fraction = Some(1.);
            // Already said, when the last move completed the last count.
            if state.sent == Some((state.planned, state.fraction)) {
                return Ok(());
            }
        }
        self.emit();
        self.result(Ok(()))
    }
    /// A planner's result, or the output failure that stopped it: a failed
    /// progress write reads as cancellation to the planner.
    pub(crate) fn result<T>(&self, result: Result<T, String>) -> Result<T, String> {
        match self.state.borrow_mut().failure.take() {
            Some(failure) => Err(failure),
            None => result,
        }
    }
    fn emit(&self) {
        if self.cadence.is_none() {
            return;
        }
        // Held only while a frame is written: never while a planner runs.
        let Ok(mut state) = self.state.try_borrow_mut() else {
            return;
        };
        if state.failure.is_some() {
            return;
        }
        let (basis, counts) = match &state.planned {
            Some(p) => (Some(p.basis()), Some(p.counts())),
            None => (None, None),
        };
        let area = match state.planned {
            Some(Planned::Area { area_mm2, .. }) => area_mm2,
            _ => None,
        };
        let progress = Progress {
            operation: state.operation.clone(),
            index: state.index,
            operations: self.operations,
            basis,
            done: counts.map(|c| c.0.min(c.1)),
            total: counts.map(|c| c.1),
            fraction: state.fraction,
            area_left_mm2: area.map(|a| a.0),
            area_start_mm2: area.map(|a| a.1),
        };
        let sent = crate::stream::encode_progress_prepared(&self.identity, &progress)
            .and_then(|frame| (state.send)(&frame));
        state.sent = Some((state.planned, state.fraction));
        if let Err(error) = sent {
            state.failure = Some(error);
            self.failed.set(true);
        }
    }
}
impl<C: CancelSignal> CancelSignal for Meter<'_, C> {
    fn is_cancelled(&self) -> bool {
        // One relaxed load more than the caller's own signal.
        if self.due.load(Ordering::Relaxed) {
            self.due.store(false, Ordering::Relaxed);
            self.emit();
        }
        self.failed.get() || self.cancel.is_cancelled()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::stream::{StreamManifest, decode_progress};

    fn manifest() -> StreamManifest {
        serde_json::from_str(include_str!("../fixtures/operations/adaptive-pocket.json"))
            .expect("fixture manifest")
    }

    #[test]
    fn fractions_count_levels_passes_and_area_within_a_level() {
        assert_eq!(Planned::Layers { done: 0, total: 0 }.fraction(), None);
        assert_eq!(
            Planned::Layers { done: 3, total: 8 }.fraction(),
            Some(0.375)
        );
        assert_eq!(Planned::Passes { done: 9, total: 8 }.fraction(), Some(1.));
        let area = |layers_done, area_mm2| Planned::Area {
            layers_done,
            layers: 4,
            area_mm2,
        };
        assert_eq!(area(1, None).fraction(), Some(0.25));
        // Half of the second level's area is gone: 1.5 of 4 levels.
        assert_eq!(area(1, Some((50., 100.))).fraction(), Some(0.375));
        // More left than at the start counts as nothing done, not less.
        assert_eq!(area(1, Some((120., 100.))).fraction(), Some(0.25));
        assert_eq!(area(4, Some((50., 100.))).fraction(), Some(1.));
    }

    #[test]
    fn the_fraction_never_decreases_and_ends_at_one() {
        let manifest = manifest();
        let mut frames = vec![];
        let mut send = |bytes: &[u8]| {
            frames.push(bytes.to_vec());
            Ok(())
        };
        let cancel = AtomicBool::new(false);
        {
            let meter =
                Meter::new(&cancel, &manifest, 2, Some(Cadence::EveryChange), &mut send).unwrap();
            meter.begin("rough", 1).unwrap();
            let area = |layers_done, left| {
                Some(Planned::Area {
                    layers_done,
                    layers: 2,
                    area_mm2: Some((left, 100.)),
                })
            };
            meter.report(area(0, 100.)).unwrap();
            meter.report(area(0, 40.)).unwrap();
            // A refresh that finds more reachable material.
            meter.report(area(0, 70.)).unwrap();
            // Unchanged counts send nothing.
            meter.report(area(0, 70.)).unwrap();
            meter.report(area(1, 90.)).unwrap();
            meter.end().unwrap();
        }
        let updates: Vec<Progress> = frames
            .iter()
            .map(|f| decode_progress(&manifest, f).unwrap())
            .collect();
        let fractions: Vec<_> = updates.iter().map(|u| u.fraction).collect();
        assert_eq!(
            fractions,
            [None, Some(0.), Some(0.3), Some(0.3), Some(0.55), Some(1.)]
        );
        assert_eq!(updates[0].basis, None);
        assert!(
            updates
                .iter()
                .all(|u| u.operation == "rough" && u.index == 1 && u.operations == 2)
        );
        // The counts are the planner's own, also where the fraction is held.
        assert_eq!(updates[3].area_left_mm2, Some(70.));
        let last = updates.last().unwrap();
        assert_eq!(
            (last.basis, last.done, last.total),
            (Some(Basis::Area), Some(2), Some(2))
        );
        assert_eq!(last.area_left_mm2, None);
    }

    #[test]
    fn an_operation_without_a_basis_reports_no_fraction_and_no_final_update() {
        let manifest = manifest();
        let mut frames = vec![];
        let mut send = |bytes: &[u8]| {
            frames.push(bytes.to_vec());
            Ok(())
        };
        let cancel = AtomicBool::new(false);
        {
            let meter =
                Meter::new(&cancel, &manifest, 1, Some(Cadence::EveryChange), &mut send).unwrap();
            meter.begin("drill", 0).unwrap();
            meter.report(None).unwrap();
            meter.end().unwrap();
        }
        assert_eq!(frames.len(), 1);
        let update = decode_progress(&manifest, &frames[0]).unwrap();
        assert_eq!(
            (update.basis, update.fraction, update.done),
            (None, None, None)
        );
    }

    #[test]
    fn a_busy_planner_sends_an_update_at_each_interval_from_its_cancel_polls() {
        let manifest = manifest();
        let mut frames = vec![];
        let mut send = |bytes: &[u8]| {
            frames.push(bytes.to_vec());
            Ok(())
        };
        let cancel = AtomicBool::new(false);
        let interval = Duration::from_millis(20);
        {
            let meter = Meter::new(
                &cancel,
                &manifest,
                1,
                Some(Cadence::Interval(interval)),
                &mut send,
            )
            .unwrap();
            meter.begin("slow", 0).unwrap();
            // A planner that only polls for cancellation, sending no move.
            let started = std::time::Instant::now();
            while started.elapsed() < Duration::from_millis(400) {
                assert!(!meter.is_cancelled());
                std::thread::sleep(Duration::from_millis(1));
            }
        }
        // 400 ms at 20 ms: about twenty; a loaded machine still gives several.
        assert!(frames.len() >= 5, "{} updates", frames.len());
        for frame in &frames {
            let update = decode_progress(&manifest, frame).unwrap();
            assert_eq!((update.operation.as_str(), update.fraction), ("slow", None));
        }
    }

    #[test]
    fn without_progress_nothing_is_sent_and_no_timer_runs() {
        let manifest = manifest();
        let mut frames: Vec<Vec<u8>> = vec![];
        let mut send = |bytes: &[u8]| {
            frames.push(bytes.to_vec());
            Ok(())
        };
        let cancel = AtomicBool::new(false);
        {
            let meter = Meter::new(&cancel, &manifest, 1, None, &mut send).unwrap();
            assert!(meter._ticker.is_none());
            meter.begin("op", 0).unwrap();
            meter
                .report(Some(Planned::Layers { done: 1, total: 2 }))
                .unwrap();
            assert!(!meter.is_cancelled());
            meter.end().unwrap();
            cancel.store(true, Ordering::Relaxed);
            assert!(meter.is_cancelled());
        }
        assert!(frames.is_empty());
    }

    #[test]
    fn a_failed_progress_write_stops_the_planner_and_is_the_error() {
        let manifest = manifest();
        let mut send = |_: &[u8]| Err("pipe closed".to_string());
        let cancel = AtomicBool::new(false);
        let meter =
            Meter::new(&cancel, &manifest, 1, Some(Cadence::EveryChange), &mut send).unwrap();
        assert_eq!(meter.begin("op", 0), Err("pipe closed".into()));
        assert!(meter.is_cancelled());
    }

    #[test]
    fn an_interval_outside_one_millisecond_to_one_second_is_refused() {
        let manifest = manifest();
        let cancel = AtomicBool::new(false);
        for interval in [Duration::ZERO, Duration::from_millis(1001)] {
            let mut send = |_: &[u8]| Ok(());
            let error = Meter::new(
                &cancel,
                &manifest,
                1,
                Some(Cadence::Interval(interval)),
                &mut send,
            )
            .err()
            .unwrap();
            assert!(error.contains("outside 1 ms to 1 s"), "{error}");
        }
    }
}
