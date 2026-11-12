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
use serde::{Deserialize, Serialize};
use std::{
    cell::RefCell,
    sync::{Condvar, Mutex, MutexGuard},
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
    /// (1 ms to 1 s), also while a planner is busy before its first move:
    /// a timer thread sends them, whatever the planner is doing.
    Interval(Duration),
    /// At each operation's start and end and whenever the planner's counts
    /// change (after a move): deterministic, and as many updates as changes.
    EveryChange,
}

/// A run's output, as its driver was given it.
pub(crate) type Output<'a> = &'a mut dyn FnMut(&[u8]) -> Result<(), String>;
/// The output of a run that reports progress: its timer thread writes too.
pub(crate) type SharedOutput<'a> = &'a mut (dyn FnMut(&[u8]) -> Result<(), String> + Send);

fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    // A panic elsewhere must not hide behind a poisoned lock here.
    mutex.lock().unwrap_or_else(|e| e.into_inner())
}

struct State {
    operation: String,
    index: usize,
    /// Between `begin` and `end`: only then is an update sent.
    planning: bool,
    planned: Option<Planned>,
    fraction: Option<f64>,
    /// What the last update of this operation said.
    sent: Option<(Option<Planned>, Option<f64>)>,
    failure: Option<String>,
}
/// What the planning thread and the timer thread share. Lock order: `state`,
/// then `sink`; a frame is written whole under `sink`.
struct Shared<'a> {
    sink: Mutex<SharedOutput<'a>>,
    state: Mutex<State>,
    stopped: Mutex<bool>,
    wake: Condvar,
    identity: Vec<u8>,
    operations: usize,
}
impl Shared<'_> {
    /// Send the operation's current update, while it is being planned.
    fn emit(&self, state: &mut State) {
        if !state.planning || state.failure.is_some() {
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
        state.sent = Some((state.planned, state.fraction));
        let sent = crate::stream::encode_progress_prepared(&self.identity, &progress)
            .and_then(|frame| (lock(&self.sink))(&frame));
        if let Err(error) = sent {
            state.failure = Some(error);
        }
    }
    /// The timer thread: an update at each interval until stopped.
    fn tick(&self, interval: Duration) {
        let mut stopped = lock(&self.stopped);
        while !*stopped {
            let (guard, timeout) = self
                .wake
                .wait_timeout(stopped, interval)
                .unwrap_or_else(|e| e.into_inner());
            stopped = guard;
            if timeout.timed_out() && !*stopped {
                // Not under `stopped`: a write may wait on the reader.
                drop(stopped);
                self.emit(&mut lock(&self.state));
                stopped = lock(&self.stopped);
            }
        }
    }
}
/// Stops the timer thread when the run's body returns or unwinds.
struct Stop<'a, 'b>(&'a Shared<'b>);
impl Drop for Stop<'_, '_> {
    fn drop(&mut self) {
        *lock(&self.0.stopped) = true;
        self.0.wake.notify_all();
    }
}

/// A run's output and its progress. Every frame of the run goes out through
/// `send`. With progress, a timer thread sends the operation's update at
/// each interval while the planning thread plans, so an update is due
/// however long a planner call takes; the two write whole frames under one
/// lock, and an operation's updates all precede its end.
///
/// The planners are not involved: they take no new parameter and keep the
/// caller's own cancel signal, so a run instantiates the engine for exactly
/// the signal types it did before (#194). The driver reads
/// `OperationPlan::progress` after `prepare` and after each move.
pub(crate) struct Meter<'a> {
    local: RefCell<Option<Output<'a>>>,
    shared: Option<Shared<'a>>,
    cadence: Option<Cadence>,
}
impl<'a> Meter<'a> {
    /// A run without progress: frames go straight to `send`.
    pub(crate) fn plain(send: Output<'a>) -> Self {
        Self {
            local: RefCell::new(Some(send)),
            shared: None,
            cadence: None,
        }
    }
    pub(crate) fn new(
        manifest: &crate::stream::StreamManifest,
        operations: usize,
        cadence: Cadence,
        send: SharedOutput<'a>,
    ) -> Result<Self, String> {
        if let Cadence::Interval(interval) = cadence
            && !(Duration::from_millis(1)..=Duration::from_secs(1)).contains(&interval)
        {
            return Err(format!(
                "planning progress interval {interval:?} is outside 1 ms to 1 s; give an interval in that range"
            ));
        }
        Ok(Self {
            local: RefCell::new(None),
            shared: Some(Shared {
                sink: Mutex::new(send),
                state: Mutex::new(State {
                    operation: String::new(),
                    index: 0,
                    planning: false,
                    planned: None,
                    fraction: None,
                    sent: None,
                    failure: None,
                }),
                stopped: Mutex::new(false),
                wake: Condvar::new(),
                identity: crate::stream::identity_bytes(manifest)?,
                operations,
            }),
            cadence: Some(cadence),
        })
    }
    /// Run the body of the run, with the timer thread beside it when
    /// updates are sent at an interval.
    pub(crate) fn run<T>(&self, body: impl FnOnce() -> Result<T, String>) -> Result<T, String> {
        let (Some(shared), Some(Cadence::Interval(interval))) = (&self.shared, self.cadence) else {
            return body();
        };
        std::thread::scope(|scope| {
            std::thread::Builder::new()
                .name("camlib-progress".into())
                .spawn_scoped(scope, || shared.tick(interval))
                .map_err(|e| {
                    format!("planning progress at an interval needs a timer thread, which could not start ({e}); run without progress")
                })?;
            let _stop = Stop(shared);
            let result = body();
            self.failure(&mut lock(&shared.state)).and(result)
        })
    }
    /// Send one frame of the run.
    pub(crate) fn send(&self, bytes: &[u8]) -> Result<(), String> {
        match &self.shared {
            Some(shared) => (lock(&shared.sink))(bytes),
            None => (self
                .local
                .borrow_mut()
                .as_mut()
                .expect("a run without progress has its output"))(bytes),
        }
    }
    /// A progress write that failed is the run's error.
    fn failure(&self, state: &mut State) -> Result<(), String> {
        state.failure.take().map_or(Ok(()), Err)
    }
    /// Planning of operation `index` starts: its first update, without a
    /// fraction until the planner states one.
    pub(crate) fn begin(&self, operation: &str, index: usize) -> Result<(), String> {
        let Some(shared) = &self.shared else {
            return Ok(());
        };
        let mut state = lock(&shared.state);
        state.operation = operation.into();
        state.index = index;
        state.planning = true;
        state.planned = None;
        state.fraction = None;
        state.sent = None;
        shared.emit(&mut state);
        self.failure(&mut state)
    }
    /// The planner's counts after preparing or after a move.
    pub(crate) fn report(&self, planned: Option<Planned>) -> Result<(), String> {
        let Some(shared) = &self.shared else {
            return Ok(());
        };
        // Nothing planned to count against (no levels) is no basis.
        let planned = planned.filter(|p| p.fraction().is_some());
        let mut state = lock(&shared.state);
        if planned.is_some() && state.planned != planned {
            state.planned = planned;
            // The one place the fraction is made monotonic: adaptive
            // clearing may find more reachable material on a refresh.
            let fraction = planned.and_then(|p| p.fraction());
            state.fraction = match (state.fraction, fraction) {
                (Some(a), Some(b)) => Some(a.max(b)),
                (a, b) => b.or(a),
            };
            if self.cadence == Some(Cadence::EveryChange) {
                shared.emit(&mut state);
            }
        }
        self.failure(&mut state)
    }
    /// The operation is planned: the last update, complete, when the planner
    /// stated a basis. No update of this operation follows.
    pub(crate) fn end(&self) -> Result<(), String> {
        let Some(shared) = &self.shared else {
            return Ok(());
        };
        let mut state = lock(&shared.state);
        if let Some(planned) = state.planned {
            state.planned = Some(planned.finished());
            state.fraction = Some(1.);
            // Already said, when the last move completed the last count.
            if state.sent != Some((state.planned, state.fraction)) {
                shared.emit(&mut state);
            }
        }
        state.planning = false;
        self.failure(&mut state)
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
    fn updates(manifest: &StreamManifest, frames: &[Vec<u8>]) -> Vec<Progress> {
        frames
            .iter()
            .map(|f| decode_progress(manifest, f).unwrap())
            .collect()
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
        {
            let meter = Meter::new(&manifest, 2, Cadence::EveryChange, &mut send).unwrap();
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
            // After its end an operation sends nothing more.
            meter.report(area(1, 10.)).unwrap();
        }
        let updates = updates(&manifest, &frames);
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
        {
            let meter = Meter::new(&manifest, 1, Cadence::EveryChange, &mut send).unwrap();
            meter.begin("drill", 0).unwrap();
            meter.report(None).unwrap();
            // No levels to count against is no basis either.
            meter
                .report(Some(Planned::Layers { done: 0, total: 0 }))
                .unwrap();
            meter.end().unwrap();
        }
        assert_eq!(frames.len(), 1);
        let update = &updates(&manifest, &frames)[0];
        assert_eq!(
            (update.basis, update.fraction, update.done),
            (None, None, None)
        );
    }

    /// A planner that does not return, and polls nothing, for 400 ms: the
    /// timer thread still sends the operation's update at each interval, the
    /// run's own frames go out between them, and nothing follows the
    /// operation's end.
    #[test]
    fn a_busy_planner_sends_an_update_at_each_interval() {
        let manifest = manifest();
        let frames = Mutex::new(vec![]);
        let mut send = |bytes: &[u8]| {
            lock(&frames).push(bytes.to_vec());
            Ok(())
        };
        let busy = || std::thread::sleep(Duration::from_millis(400));
        let interval = Cadence::Interval(Duration::from_millis(20));
        let meter = Meter::new(&manifest, 1, interval, &mut send).unwrap();
        let (planned, after) = meter
            .run(|| {
                meter.begin("slow", 0)?;
                busy();
                meter.report(Some(Planned::Layers { done: 1, total: 4 }))?;
                busy();
                meter.send(b"a frame of the run")?;
                let planned = lock(&frames).len();
                meter.end()?;
                let ended = lock(&frames).len();
                busy();
                Ok((planned, lock(&frames).len() - ended))
            })
            .unwrap();
        // 800 ms at 20 ms: about forty; a loaded machine still gives several.
        assert!(planned >= 10, "{planned} frames");
        assert_eq!(after, 0);
        let frames = frames.into_inner().unwrap();
        let (run, progress): (Vec<_>, Vec<_>) = frames
            .into_iter()
            .partition(|f| f.as_slice() == b"a frame of the run");
        assert_eq!(run.len(), 1);
        let updates = updates(&manifest, &progress);
        let fractions: Vec<_> = updates.iter().map(|u| u.fraction).collect();
        assert_eq!(fractions[0], None);
        assert!(fractions.contains(&Some(0.25)));
        assert_eq!(fractions.last(), Some(&Some(1.)));
        assert!(fractions.windows(2).all(|pair| pair[0] <= pair[1]));
    }

    #[test]
    fn without_progress_frames_go_straight_out_and_nothing_else() {
        let mut frames: Vec<Vec<u8>> = vec![];
        let mut send = |bytes: &[u8]| {
            frames.push(bytes.to_vec());
            Ok(())
        };
        {
            let meter = Meter::plain(&mut send);
            meter
                .run(|| {
                    meter.begin("op", 0)?;
                    meter.report(Some(Planned::Layers { done: 1, total: 2 }))?;
                    meter.send(b"frame")?;
                    meter.end()
                })
                .unwrap();
        }
        assert_eq!(frames, [b"frame".to_vec()]);
    }

    #[test]
    fn a_failed_progress_write_is_the_error() {
        let manifest = manifest();
        let mut send = |_: &[u8]| Err("pipe closed".to_string());
        let meter = Meter::new(&manifest, 1, Cadence::EveryChange, &mut send).unwrap();
        assert_eq!(meter.begin("op", 0), Err("pipe closed".into()));
        // From the timer thread too: the run's body ends with it.
        let mut send = |_: &[u8]| Err("pipe closed".to_string());
        let interval = Cadence::Interval(Duration::from_millis(5));
        let meter = Meter::new(&manifest, 1, interval, &mut send).unwrap();
        lock(&meter.shared.as_ref().unwrap().state).planning = true;
        let result = meter.run(|| {
            std::thread::sleep(Duration::from_millis(200));
            Ok(())
        });
        assert_eq!(result, Err("pipe closed".into()));
    }

    #[test]
    fn an_interval_outside_one_millisecond_to_one_second_is_refused() {
        let manifest = manifest();
        for interval in [Duration::ZERO, Duration::from_millis(1001)] {
            let mut send = |_: &[u8]| Ok(());
            let error = Meter::new(&manifest, 1, Cadence::Interval(interval), &mut send)
                .err()
                .unwrap();
            assert!(error.contains("outside 1 ms to 1 s"), "{error}");
        }
    }
}
