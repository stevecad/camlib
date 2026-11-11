//! One posting run: a port of the Python `PostEngine` for the default post
//! and decoder. Every hook's blocks are decoded by `super::decode` and the
//! native intent checker (`crate::post_intent`) compares the decoded events
//! with the source events before a chunk is accepted.
use super::decode::{Context, Decoder};
use super::machine::Machine;
use super::post::{DefaultPost, Hook};
use crate::motion::{Curve, Event, Program};
use crate::post_intent::{IntentChecker, Output, Settings};
use crate::stock::CancelSignal;
use serde::Serialize;
use std::collections::{BTreeSet, VecDeque};

pub use crate::post_lowering::Options as LoweringOptions;

/// How lowered source arcs and helices were written (`PostEngine.arc_output`).
#[derive(Clone, Debug, Default, Serialize)]
pub struct ArcOutput {
    pub native: u64,
    pub lines_policy: u64,
    pub lines_machine: u64,
    pub lines_precision: u64,
}

/// Each event's written blocks and their decoded events.
type Formatted = (Vec<Vec<String>>, Vec<Vec<Event>>);

pub struct Engine {
    machine: Machine,
    lowering: Option<LoweringOptions>,
    post: DefaultPost,
    decoder: Decoder,
    session: IntentChecker,
    /// Started source events not yet fully checked, oldest first.
    sources: VecDeque<Event>,
    pub arc_output: ArcOutput,
    /// The posting definition recorded with the run (machine, post, decoder).
    pub definition: serde_json::Value,
    finished: bool,
}

fn is_curve(event: &Event) -> bool {
    matches!(
        event,
        Event::Motion { motion } if matches!(motion.curve, Curve::Arc { .. } | Curve::Helix { .. })
    )
}
fn is_closed_curve(event: &Event) -> bool {
    let Event::Motion { motion } = event else {
        return false;
    };
    match motion.curve {
        Curve::Arc {
            start, end, plane, ..
        }
        | Curve::Helix {
            start, end, plane, ..
        } => {
            let [u, v, _] = plane.axes();
            start[u] == end[u] && start[v] == end[v]
        }
        Curve::Line { .. } => false,
    }
}
fn cycle_covers(event: &Event) -> Option<u32> {
    match event {
        Event::Cycle { covers, .. } => Some(*covers),
        _ => None,
    }
}
fn context(event: &Event) -> Option<Context> {
    match event {
        Event::Motion { motion } => Some(Context {
            id: motion.id,
            operation: motion.operation.clone(),
            kind: motion.kind,
        }),
        _ => None,
    }
}
fn is_stop_or_off(events: &[Event]) -> bool {
    events.iter().all(|a| match a {
        Event::Spindle { rpm, .. } => *rpm == 0.,
        Event::Coolant { mode } => *mode == crate::motion::Coolant::Off,
        _ => false,
    })
}

impl Engine {
    /// `PostEngine.begin`: the initialization blocks, checked.
    pub fn begin(
        header: &Program,
        machine: Machine,
        lowering: Option<LoweringOptions>,
        run_id: String,
        cancel: &impl CancelSignal,
    ) -> Result<(Self, Output), String> {
        let m = &machine;
        if m.schedule_post_feeds {
            return Err(
                "schedule_post_feeds is not supported by the Rust post yet: post without it, or with the Python post"
                    .into(),
            );
        }
        if header.frame != m.frame {
            return Err("program and machine work frame differ".into());
        }
        if header.tools.iter().any(|t| {
            !m.tools
                .iter()
                .any(|s| s.tool == t.id && s.gauge_length_mm == t.gauge_length_mm)
        }) {
            return Err("machine tool table does not match program assemblies/lengths".into());
        }
        let beams: BTreeSet<String> = header
            .tools
            .iter()
            .filter(|t| t.cutter.shape == crate::tool::CutterShape::Beam)
            .map(|t| t.id.clone())
            .collect();
        let mut post = DefaultPost::new(machine.clone());
        // Cutter radii (a probe's stylus ball): probing cycles are laid out
        // from them, and the decoder reads them back with them.
        post.radii = header
            .tools
            .iter()
            .map(|t| (t.id.clone(), t.cutter.diameter_mm / 2.))
            .collect();
        let mut decoder = Decoder::new(m, header.initial_position_mm, beams)?;
        decoder.set_probing_tools(&header.tools);
        post.geometry_tolerance_mm = header.tolerance_mm;
        let lines = post.call(
            Hook::Begin {
                tolerance_mm: header.tolerance_mm,
            },
            32,
        )?;
        let actions = decoder.read(&lines, None)?;
        if decoder.ended || !is_stop_or_off(&actions) {
            return Err("initialization must not move/select tools or change source setup".into());
        }
        decoder.validate_initialized()?;
        let identity = super::controller::identity(m.controller.as_ref());
        let definition = serde_json::json!({
            "machine": serde_json::to_value(m).map_err(|e| e.to_string())?,
            "post": identity.post,
            "post_version": identity.post_version,
            "decoder": identity.decoder,
            "decoder_version": identity.decoder_version,
        });
        let settings = Settings {
            capabilities: m.capabilities(),
            tolerance_mm: m.tolerance_mm,
            feed_tolerance_mm_min: m.feed_tolerance_mm_min,
            traverse_feed_mm_min: m.traverse_feed_mm_min,
            max_normal_accel_mm_s2: m.max_normal_accel_mm_s2,
            max_axis_feed_mm_min: m.max_axis_feed_mm_min,
            work_to_machine: m.work_to_machine.clone(),
            tip_travel_mm: m.tip_travel_mm,
            gauge_travel_mm: m.gauge_travel_mm,
            definition: definition.clone(),
            run_id,
            decoded_initial_position_mm: decoder.position,
            lowering: lowering.clone(),
            motion_scheduling: None,
        };
        let (session, first) = IntentChecker::begin(header, settings, &lines)?;
        if cancel.is_cancelled() {
            return Err("post cancelled; batch not committed".into());
        }
        Ok((
            Self {
                machine,
                lowering,
                post,
                decoder,
                session,
                sources: VecDeque::new(),
                arc_output: ArcOutput::default(),
                definition,
                finished: false,
            },
            first,
        ))
    }

    /// The decoded position, where the controller is.
    pub fn position_mm(&self) -> crate::motion::Vec3 {
        self.decoder.position
    }

    /// Write and decode events (motion ids renumbered after the decoded
    /// motions when `renumber`). Returns the blocks and decoded events.
    fn format(
        &mut self,
        events: &[Event],
        originals: &[Event],
        renumber: bool,
        cancel: &impl CancelSignal,
    ) -> Result<Formatted, String> {
        let mut blocks = Vec::with_capacity(events.len());
        let mut actual = Vec::with_capacity(events.len());
        let mut next_id: Option<u64> = None;
        for (event, original) in events.iter().zip(originals) {
            if cancel.is_cancelled() {
                return Err("post cancelled; batch not committed".into());
            }
            let mut typed = event.clone();
            if let Event::Motion { motion } = &mut typed {
                if renumber {
                    let id = *next_id.get_or_insert(motion.id);
                    motion.id = id;
                }
                if motion.kind.is_rapid()
                    && motion.feed_mm_min.is_none()
                    && let Some(feed) = self.machine.traverse_feed_mm_min
                {
                    motion.feed_mm_min = Some(feed);
                }
            }
            self.post.position_mm = Some(self.decoder.position);
            self.post.source_event = Some(original.clone());
            let block = self.post.call(Hook::Event(&typed), 8)?;
            let decoded = if let Event::Probe { cycle, covers } = &typed {
                self.decoder.read_probe(&block, cycle, *covers)?
            } else {
                let context = context(&typed);
                self.decoder.read(&block, context.as_ref())?
            };
            if self.decoder.ended || self.decoder.tool_pending {
                return Err("event ended program or left a partial tool transition".into());
            }
            if renumber
                && matches!(typed, Event::Motion { .. })
                && decoded.iter().any(|e| matches!(e, Event::Motion { .. }))
                && let Some(id) = next_id.as_mut()
            {
                *id += 1;
            }
            blocks.push(block);
            actual.push(decoded);
        }
        Ok((blocks, actual))
    }

    /// `PostEngine.consume` (direct mode): at most 64 events, one chunk.
    pub fn consume(
        &mut self,
        events: &[Event],
        cancel: &impl CancelSignal,
    ) -> Result<Output, String> {
        if self.finished {
            return Err("generic post lifecycle: begin/finish".into());
        }
        if self.lowering.is_some() {
            return Err("lowering uses start/drain or consume_events".into());
        }
        if events.len() > 64 {
            return Err("post consume accepts at most 64 events".into());
        }
        let (blocks, actual) = self.format(events, events, false, cancel)?;
        self.session.consume(events, &actual, &blocks, cancel)
    }

    /// How a lowered source arc/helix is written (`PostEngine._arc_output`).
    fn arc_reason(&self, event: &Event) -> Option<&'static str> {
        let Event::Motion { motion } = event else {
            return None;
        };
        let (plane, sweep, helix) = match motion.curve {
            Curve::Arc {
                plane, sweep_rad, ..
            } => (plane, sweep_rad, false),
            Curve::Helix {
                plane, sweep_rad, ..
            } => (plane, sweep_rad, true),
            Curve::Line { .. } => return None,
        };
        if !self.lowering.as_ref().is_some_and(|l| l.native_arcs) {
            return Some("lines_policy");
        }
        let m = &self.machine;
        if !m.planes.contains(&plane)
            || sweep.abs() > m.max_sweep_rad
            || !(if helix { m.helices } else { m.arcs })
        {
            return Some("lines_machine");
        }
        let mut post = self.post.clone();
        post.position_mm = Some(self.decoder.position);
        Some(if post.writes_arc(&motion.curve) {
            "native"
        } else {
            "lines_precision"
        })
    }
    fn count(&mut self, reason: Option<&'static str>) {
        match reason {
            Some("native") => self.arc_output.native += 1,
            Some("lines_policy") => self.arc_output.lines_policy += 1,
            Some("lines_machine") => self.arc_output.lines_machine += 1,
            Some("lines_precision") => self.arc_output.lines_precision += 1,
            _ => {}
        }
    }
    /// `PostEngine._start_many`: start lowered sources, all or none.
    fn start_many(&mut self, events: Vec<Event>, cancel: &impl CancelSignal) -> Result<(), String> {
        let reasons: Vec<_> = events.iter().map(|e| self.arc_reason(e)).collect();
        let mut next = self.session.clone();
        for (event, reason) in events.iter().zip(&reasons) {
            next.start_with(event.clone(), *reason == Some("native"), cancel)?;
        }
        self.session = next;
        for reason in reasons {
            self.count(reason);
        }
        self.sources.extend(events);
        Ok(())
    }
    /// `PostEngine.drain`: format/decode/check at most 64 planned segments.
    fn drain(&mut self, cancel: &impl CancelSignal) -> Result<Output, String> {
        let preview = self.session.preview(64, cancel)?;
        let events: Vec<Event> = preview.iter().map(|p| p.event.clone()).collect();
        let ranges: Vec<[f64; 2]> = preview.iter().map(|p| p.source_fraction_interval).collect();
        let originals: Vec<Event> = preview
            .iter()
            .map(|p| {
                self.sources
                    .get(p.source)
                    .cloned()
                    .ok_or("lowered segment without its source")
            })
            .collect::<Result<_, _>>()?;
        let (blocks, actual) = self.format(&events, &originals, true, cancel)?;
        let out = self
            .session
            .check_lowered(&ranges, &actual, &blocks, cancel)?;
        // Sources whose last segment was in this batch are complete.
        for _ in 0..ranges.iter().filter(|r| r[1] == 1.).count() {
            self.sources.pop_front();
        }
        Ok(out)
    }

    /// `PostEngine.consume_events` for a complete, finite event list: the
    /// same batches (at most 64 events, never ending inside a canned cycle).
    pub fn consume_events(
        &mut self,
        events: Vec<Event>,
        cancel: &impl CancelSignal,
        mut sink: impl FnMut(Output) -> Result<(), String>,
    ) -> Result<(), String> {
        if self.finished || self.session.has_pending() {
            return Err("drain pending source before consuming events".into());
        }
        let mut it = events.into_iter();
        let mut held: Option<Event> = None;
        if self.lowering.is_none() {
            loop {
                let mut batch = vec![];
                let mut covered = 0u32;
                while batch.len() < 64 {
                    if held.is_none() {
                        held = it.next();
                        if held.is_none() {
                            break;
                        }
                    }
                    if let Some(covers) = cycle_covers(held.as_ref().unwrap()) {
                        if covered > 0 {
                            return Err("a canned cycle starts inside another one".into());
                        }
                        if covers + 1 > 64 {
                            return Err(format!(
                                "a canned cycle covering {covers} events cannot be checked in one batch (at most 63): post this hole as explicit moves (canned_cycle=False)"
                            ));
                        }
                        if batch.len() + covers as usize + 1 > 64 {
                            break;
                        }
                        covered = covers + 1;
                    }
                    batch.push(held.take().unwrap());
                    covered = covered.saturating_sub(1);
                }
                if covered > 0 {
                    return Err(format!(
                        "the events end inside a canned cycle ({covered} covered events missing)"
                    ));
                }
                if batch.is_empty() {
                    return Ok(());
                }
                sink(self.consume(&batch, cancel)?)?;
            }
        }
        let native_arcs = self.lowering.as_ref().is_some_and(|l| l.native_arcs);
        loop {
            if cancel.is_cancelled() {
                return Err("post cancelled; batch not committed".into());
            }
            let mut done = false;
            let mut batch: Vec<Event> = vec![];
            let mut covered = 0u32;
            while covered > 0 || self.sources.len() + batch.len() < 64 {
                let queued = !self.sources.is_empty() || !batch.is_empty();
                if held.is_none() {
                    held = it.next();
                    if held.is_none() {
                        if covered > 0 {
                            return Err(format!(
                                "the events end inside a canned cycle ({covered} covered events missing)"
                            ));
                        }
                        done = true;
                        break;
                    }
                }
                let event = held.as_ref().unwrap();
                if let Some(covers) = cycle_covers(event) {
                    if covered > 0 {
                        return Err("a canned cycle starts inside another one".into());
                    }
                    if queued {
                        break; // the cycle starts a batch of its own
                    }
                    if covers + 1 > 64 {
                        return Err(format!(
                            "a canned cycle covering {covers} events cannot be checked in one batch (at most 63): post this hole as explicit moves (canned_cycle=False)"
                        ));
                    }
                    covered = covers;
                    batch.push(held.take().unwrap());
                    continue;
                }
                if covered == 0
                    && queued
                    && (native_arcs && is_curve(event) || is_closed_curve(event))
                {
                    break;
                }
                batch.push(held.take().unwrap());
                covered = covered.saturating_sub(1);
            }
            let group = batch.first().is_some_and(|e| cycle_covers(e).is_some());
            if !batch.is_empty() {
                self.start_many(batch, cancel)?;
            }
            if self.session.has_pending() {
                let chunk = self.drain(cancel)?;
                if group && self.session.has_pending() {
                    return Err("a canned cycle and its covered moves did not fit one checked batch; post this hole as explicit moves (canned_cycle=False)".into());
                }
                sink(chunk)?;
            }
            if done {
                while self.session.has_pending() {
                    let chunk = self.drain(cancel)?;
                    sink(chunk)?;
                }
                return Ok(());
            }
        }
    }

    /// `PostEngine.finish`: the closing blocks, checked.
    pub fn finish(&mut self, cancel: &impl CancelSignal) -> Result<Output, String> {
        if self.finished || self.session.has_pending() {
            return Err("generic finish lifecycle: drain pending source".into());
        }
        self.post.position_mm = Some(self.decoder.position);
        let lines = self.post.call(Hook::Finish, 8)?;
        let actions = self.decoder.read(&lines, None)?;
        if !self.decoder.ended || !is_stop_or_off(&actions) {
            return Err(
                "finish needs stopped spindle/coolant and end; no extra motion/effects".into(),
            );
        }
        let out = self.session.finish(&lines, cancel)?;
        self.finished = true;
        Ok(out)
    }
}
