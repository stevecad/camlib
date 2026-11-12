//! Controller-independent comparison of source and independently decoded events.
//! The Python decoder owns syntax/declared controller semantics. This module
//! proves nominal geometric agreement under that model, not hardware execution.
pub use crate::post_scheduling::{FeedAttribution, SchedulingOutput, StreamTiming};
use crate::{
    motion::{Capabilities, Curve, Event, Program, ProgramValidator, Vec3, WorkFrame},
    post::{Checked, Chunk, Identity, Line, check_chunk_size, curve_difference},
    stock::CancelSignal,
    tool::valid_id,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Settings {
    pub capabilities: Capabilities,
    pub tolerance_mm: f64,
    pub feed_tolerance_mm_min: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub traverse_feed_mm_min: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_normal_accel_mm_s2: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_axis_feed_mm_min: Option<Vec3>,
    pub work_to_machine: WorkFrame,
    pub tip_travel_mm: Option<[Vec3; 2]>,
    /// The spindle gauge line's travel (machine coordinates); each decoded
    /// tool's tip is checked against it lowered by that tool's gauge length
    /// (`verification::MachineLimits::min_gauge_mm`). Not with
    /// `tip_travel_mm`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gauge_travel_mm: Option<[Vec3; 2]>,
    pub definition: serde_json::Value,
    pub run_id: String,
    pub decoded_initial_position_mm: Vec3,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lowering: Option<crate::post_lowering::Options>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub motion_scheduling: Option<crate::dynamics::Settings>,
}

#[derive(Serialize)]
pub struct Output {
    #[serde(flatten)]
    pub chunk: Chunk,
    pub mappings: Vec<crate::post_lowering::Mapping>,
    pub run_id: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub progress: Option<crate::post_lowering::Progress>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lowering_policy: Option<crate::post_lowering::Options>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub nominal_normal_acceleration: Option<NormalAccelerationChecks>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub nominal_axis_feed: Option<AxisFeedChecks>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub motion_scheduling: Option<SchedulingOutput>,
}

/// Cumulative actual checks against declared/configured feed, not controller
/// timing or junction/tangential/axis/blend/jerk dynamics.
#[derive(Clone, Serialize)]
pub struct NormalAccelerationChecks {
    pub limit_mm_s2: f64,
    pub source_feed_curves_checked: u64,
    pub decoded_feed_curves_checked: u64,
    pub source_rapid_speed_unknown: u64,
    pub decoded_rapid_speed_unknown: u64,
    pub scope: &'static str,
}

/// Peak machine-axis feed on original and independently decoded nominal curves.
#[derive(Clone, Serialize)]
pub struct AxisFeedChecks {
    pub limits_mm_min: Vec3,
    pub source_feed_curves_checked: u64,
    pub decoded_feed_curves_checked: u64,
    pub source_rapid_speed_unknown: u64,
    pub decoded_rapid_speed_unknown: u64,
    pub scope: &'static str,
}

#[derive(Serialize)]
pub struct Preview {
    pub event: Event,
    pub source_fraction_interval: [f64; 2],
    /// Which started source this segment belongs to (0: the oldest pending).
    pub source: usize,
}

/// Started sources not yet fully checked (several may be queued so one
/// checked batch covers many short sources).
const MAX_QUEUED_SOURCES: usize = 64;

#[derive(Clone)]
pub struct IntentChecker {
    source: ProgramValidator,
    decoded: ProgramValidator,
    settings: Settings,
    identity: Identity,
    source_tolerance: f64,
    line: usize,
    finished: bool,
    pending: std::collections::VecDeque<crate::post_lowering::SourceCursor>,
    posted: u64,
    normal_acceleration: Option<NormalAccelerationChecks>,
    axis_feed: Option<AxisFeedChecks>,
    scheduling: Option<crate::post_scheduling::State>,
    pending_scheduled: Option<crate::post_scheduling::Scheduled>,
    /// The last written canned cycle: its event index and the last event it covers.
    cycle: Option<(usize, usize)>,
    /// A cutter-compensation change written in the block of the move that
    /// follows it (FANUC/Haas start-up and cancel), not yet seen there.
    deferred_compensation: Option<Event>,
    /// Gauge length per program tool, for gauge-line travel.
    gauge_lengths: std::collections::HashMap<String, f64>,
}

/// Transport bound on the blocks one source event writes: 8, or for a
/// probing cycle, which writes all its moves and their checks, 512 (a cycle
/// has at most a few dozen moves of at most four blocks each).
const PROBE_BLOCK_LINES: usize = 512;
fn block_lines(event: &Event) -> usize {
    if matches!(event, Event::Probe { .. }) {
        PROBE_BLOCK_LINES
    } else {
        8
    }
}

fn cancelled(cancel: &impl CancelSignal) -> Result<(), String> {
    if cancel.is_cancelled() {
        Err("post cancelled; batch not committed".into())
    } else {
        Ok(())
    }
}

// A stationary block may approximate a line or a strictly minor source arc.
// The whole subcurve must fit the output budget AND (for arcs) radius/4.
// Reject whole-source quarter/major/full turns even if one interval is tiny:
// controller precision must never erase their circular winding.
fn stationary_bound(
    curve: &crate::motion::Curve,
    range: [f64; 2],
    point: Vec3,
    tolerance: f64,
) -> Result<f64, String> {
    use crate::motion::Curve;
    let distance = |p: Vec3| {
        (0..3)
            .map(|k| (p[k] - point[k]).powi(2))
            .sum::<f64>()
            .sqrt()
    };
    if let Curve::Line { start, end } = curve {
        return Ok(range
            .into_iter()
            .map(|t| distance(std::array::from_fn(|k| start[k] + t * (end[k] - start[k]))))
            .fold(0., f64::max));
    }
    let (center, plane, sweep) = match curve {
        Curve::Arc {
            center,
            plane,
            sweep_rad,
            ..
        }
        | Curve::Helix {
            center,
            plane,
            sweep_rad,
            ..
        } => (*center, *plane, *sweep_rad),
        _ => unreachable!(),
    };
    if sweep.abs() >= std::f64::consts::FRAC_PI_2 {
        return Err("stationary output cannot erase circular winding".into());
    }
    let prepared = curve.prepare(tolerance)?;
    let [u, v, _] = plane.axes();
    let start = curve.start();
    let radius = (start[u] - center[u]).hypot(start[v] - center[v]);
    let angle = sweep.abs() * (range[1] - range[0]);
    let endpoint =
        distance(prepared.position(range[0])?).max(distance(prepared.position(range[1])?));
    // Helix axial coordinates interpolate linearly; only the planar sagitta
    // separates the analytic subcurve from its chord.
    let sagitta = 2. * radius * (angle / 4.).sin().powi(2);
    let scale = start
        .iter()
        .chain(&curve.end())
        .chain(&center)
        .chain(&point)
        .map(|x| x.abs())
        .fold(1. + radius, f64::max);
    let bound =
        endpoint + sagitta + crate::post::endpoint_gap(curve) + 1024. * f64::EPSILON * scale;
    if bound > radius / 4. {
        return Err("stationary circular output exceeds radius-relative bound".into());
    }
    Ok(bound)
}

impl IntentChecker {
    pub fn begin(
        header: &Program,
        settings: Settings,
        lines: &[String],
    ) -> Result<(Self, Output), String> {
        if !header.events.is_empty()
            || !valid_id(&settings.run_id)
            || !settings.tolerance_mm.is_finite()
            || !(1e-9..=0.1).contains(&settings.tolerance_mm)
            || !settings.feed_tolerance_mm_min.is_finite()
            || !(1e-9..=0.1).contains(&settings.feed_tolerance_mm_min)
            || settings.definition.is_null()
            || lines.len() > 32
        {
            return Err("invalid generic post header/settings/initialization cap".into());
        }
        if settings.traverse_feed_mm_min.is_some_and(|feed| {
            !feed.is_finite() || feed <= 0. || feed > settings.capabilities.max_feed_mm_min
        }) {
            return Err(
                "configured traverse feed must be positive, finite and within machine limits"
                    .into(),
            );
        }
        if settings
            .max_normal_accel_mm_s2
            .is_some_and(|a| !a.is_finite() || !(1e-6..=1e9).contains(&a))
        {
            return Err("invalid nominal normal acceleration limit".into());
        }
        if settings.max_axis_feed_mm_min.is_some_and(|limits| {
            limits
                .iter()
                .any(|v| !v.is_finite() || !(1e-6..=1e7).contains(v))
        }) {
            return Err("invalid nominal axis feed limits".into());
        }
        settings.work_to_machine.validate()?;
        if settings.tip_travel_mm.is_some() && settings.gauge_travel_mm.is_some() {
            return Err("machine travel is declared once: tip_travel_mm (the tip box of one tool length) or gauge_travel_mm (the spindle gauge line's box, checked per tool)".into());
        }
        if let Some([min, max]) = settings.tip_travel_mm.or(settings.gauge_travel_mm) {
            for i in 0..3 {
                if !min[i].is_finite() || !max[i].is_finite() || min[i] >= max[i] {
                    return Err("invalid configured machine tip travel".into());
                }
            }
        }
        if settings.gauge_travel_mm.is_some() {
            if settings.work_to_machine.z != [0., 0., 1.] {
                return Err("gauge_travel_mm needs the work Z axis along machine Z (work_to_machine.z = [0, 0, 1]): the tip is the gauge line lowered by the gauge length along the spindle axis".into());
            }
            if let Some(t) = header
                .tools
                .iter()
                .find(|t| t.cutter.shape == crate::tool::CutterShape::Beam)
            {
                return Err(format!(
                    "gauge_travel_mm gives a spindle tool's tip travel from its gauge length; beam tool {} has none: declare tip_travel_mm",
                    t.id
                ));
            }
        }
        let scheduling = if let Some(policy) = &settings.motion_scheduling {
            if policy.max_feed_mm_min != settings.capabilities.max_feed_mm_min
                || policy.max_normal_accel_mm_s2 != settings.max_normal_accel_mm_s2
                || policy.max_axis_feed_mm_min != settings.max_axis_feed_mm_min
                || policy.work_to_machine != settings.work_to_machine
            {
                return Err("scheduling must use the declared machine/frame limits".into());
            }
            Some(crate::post_scheduling::State::new(header, policy.clone())?)
        } else {
            None
        };
        let mut source_caps = settings.capabilities.clone();
        if scheduling.is_some() {
            source_caps.max_feed_mm_min = 1e7;
        }
        if let Some(options) = &settings.lowering {
            options.validate(settings.tolerance_mm)?;
            source_caps.arcs = true;
            source_caps.helices = true;
            source_caps.planes = vec![
                crate::motion::Plane::Xy,
                crate::motion::Plane::Xz,
                crate::motion::Plane::Yz,
            ];
            source_caps.max_sweep_rad = 1000. * std::f64::consts::TAU;
        }
        let source = ProgramValidator::new(header, Some(&source_caps))?;
        let mut decoded_header = header.clone();
        decoded_header.initial_position_mm = settings.decoded_initial_position_mm;
        decoded_header.tolerance_mm = settings.tolerance_mm;
        if (0..3)
            .map(|i| {
                (header.initial_position_mm[i] - settings.decoded_initial_position_mm[i]).powi(2)
            })
            .sum::<f64>()
            .sqrt()
            > settings.tolerance_mm
        {
            return Err("decoded initial position exceeds output tolerance".into());
        }
        let decoded = ProgramValidator::new(&decoded_header, Some(&settings.capabilities))?;
        let identity = Identity {
            program_id: header.id.clone(),
            revision: header.revision.clone(),
            abi: match (
                settings.motion_scheduling.is_some(),
                settings.lowering.is_some(),
            ) {
                (true, true) => 8,
                (true, false) => 7,
                (false, true) => 6,
                (false, false) => 5,
            },
            header_setup_sha256: format!(
                "{:x}",
                Sha256::digest(
                    serde_json::to_vec(&(header, &settings)).map_err(|e| e.to_string())?
                )
            ),
        };
        let normal_acceleration = settings.max_normal_accel_mm_s2.map(|limit_mm_s2| NormalAccelerationChecks {
            limit_mm_s2, source_feed_curves_checked: 0, decoded_feed_curves_checked: 0,
            source_rapid_speed_unknown: 0, decoded_rapid_speed_unknown: 0,
            scope: if settings.motion_scheduling.is_some() {"scheduled_nominal_source_and_decoded_curve_interior; authored_feeds_retained_in_scheduling_records"} else {"nominal_analytic_segment_interior_at_declared_or_configured_traverse_feed_only"},
        });
        let axis_feed = settings.max_axis_feed_mm_min.map(|limits_mm_min| AxisFeedChecks {
            limits_mm_min, source_feed_curves_checked: 0, decoded_feed_curves_checked: 0,
            source_rapid_speed_unknown: 0, decoded_rapid_speed_unknown: 0,
            scope: if settings.motion_scheduling.is_some() {"scheduled_nominal_source_and_decoded_machine_axis_feed; authored_feeds_retained_in_scheduling_records"} else {"nominal_analytic_machine_axis_segment_interior_at_declared_or_configured_traverse_feed_only"},
        });
        let mut state = Self {
            source,
            decoded,
            settings,
            identity,
            source_tolerance: header.tolerance_mm,
            line: 0,
            finished: false,
            pending: Default::default(),
            posted: 0,
            normal_acceleration,
            axis_feed,
            scheduling,
            pending_scheduled: None,
            cycle: None,
            deferred_compensation: None,
            gauge_lengths: header
                .tools
                .iter()
                .map(|t| (t.id.clone(), t.gauge_length_mm))
                .collect(),
        };
        state.check_point(header.initial_position_mm, None)?;
        state.check_point(state.settings.decoded_initial_position_mm, None)?;
        let mut chunk = state.chunk(None, lines)?;
        chunk.decoded_initial_position_mm = Some(state.settings.decoded_initial_position_mm);
        let mut output = state.output(chunk, vec![], vec![]);
        output.lowering_policy = state.settings.lowering.clone();
        check_chunk_size(&output)?;
        Ok((state, output))
    }
    /// The travel of `tool`'s tip, when one is checked: the declared tip
    /// box, or the gauge line's box lowered by the tool's gauge length. The
    /// start pose has no program tool (`None`): on the gauge line it is
    /// checked as the start of the first move, with that move's tool; the
    /// machine is already there.
    fn tip_travel(&self, tool: Option<&str>) -> Result<Option<crate::travel::Travel>, String> {
        if let Some([min, max]) = self.settings.tip_travel_mm {
            return Ok(Some(crate::travel::Travel::tip(min, max)));
        }
        let (Some([min, max]), Some(tool)) = (self.settings.gauge_travel_mm, tool) else {
            return Ok(None);
        };
        let length = *self
            .gauge_lengths
            .get(tool)
            .ok_or_else(|| format!("gauge-line travel: tool {tool} is not a program tool"))?;
        // The gauge line is the tip raised by the gauge length (G43).
        Ok(Some(crate::travel::Travel::gauge_line(min, max, length)))
    }
    /// The refusal for a decoded position outside the travel. The decoded
    /// words are what the controller executes, so this is never tolerated,
    /// whatever put the position there (`crate::travel`).
    fn travel_refusal(&self, found: &crate::travel::Outside, tool: Option<&str>) -> String {
        let axis = ["X", "Y", "Z"][found.axis];
        let k = found.axis;
        if let Some(gap) = found.nominal_gap_mm {
            // A point of a decoded arc's nominal circle, not a decoded end.
            let side = if found.low { "minimum" } else { "maximum" };
            let (at, limit) = (found.machine_mm[k], found.limit_mm);
            let tool = tool.unwrap_or("?");
            let state = if found.beyond_mm > 0. {
                format!("{:.3e} mm beyond", found.beyond_mm)
            } else {
                format!("{:.3e} mm inside", found.beyond_mm.abs())
            };
            return format!(
                "decoded arc of tool {tool} is not inside configured machine travel on machine {axis}: its nominal extreme at {at:.6} mm is {state} the tip's travel {side} {limit:.6} mm, and its words leave an endpoint gap of {gap:.3e} mm (the end is that far off the circle drawn from the start about the centre). Controllers reconcile the two radii differently, so the executed path is only known within the gap of the nominal circle and must be inside the limit by the gap. Write the arc with more decimals, keep it the gap inside the limit, or lower it to lines (a machine with arcs=False, posted with lowering)"
            );
        }
        let subject = match (self.settings.gauge_travel_mm, tool) {
            (Some([min, max]), Some(tool)) => {
                let length = self.gauge_lengths.get(tool).copied().unwrap_or(0.);
                let raise = if k == 2 { length } else { 0. };
                format!(
                    "decoded tool {tool} (gauge length {length} mm) exceeds configured machine travel on machine {axis}: gauge line at {:.6} mm, travel {:.6} to {:.6} mm",
                    found.machine_mm[k] + raise,
                    min[k],
                    max[k]
                )
            }
            _ => format!(
                "decoded tool tip exceeds configured machine travel on machine {axis}: tip at {:.6} mm, {} its limit {:.6} mm",
                found.machine_mm[k],
                if found.low { "below" } else { "above" },
                found.limit_mm
            ),
        };
        format!(
            "{subject} (beyond by {:.3e} mm). The controller executes the posted words and stops on its travel limit. A limit itself is inside; if the planned position is inside, the word was rounded beyond the limit: keep the plan one output step inside it, or put the work offset and the limit on the output grid, so a position on the limit is written exactly",
            found.beyond_mm
        )
    }
    /// A decoded position (a tip position, work coordinates) against the
    /// machine's travel, with `tool` in the spindle: the one travel rule
    /// (`crate::travel`), as the plan check (`verification::check_travel`).
    fn check_point(&self, point: Vec3, tool: Option<&str>) -> Result<(), String> {
        let Some(travel) = self.tip_travel(tool)? else {
            return Ok(());
        };
        match crate::travel::point_outside(&self.settings.work_to_machine, &travel, point) {
            Some(found) => Err(self.travel_refusal(&found, tool)),
            None => Ok(()),
        }
    }
    /// A decoded move against the machine's travel, in closed form: its
    /// declared ends and, for an arc or helix, where each machine coordinate
    /// is stationary, in any work frame.
    fn check_curve(&self, curve: &Curve, tool: &str) -> Result<(), String> {
        let Some(travel) = self.tip_travel(Some(tool))? else {
            return Ok(());
        };
        let path = curve.prepare(self.settings.tolerance_mm)?;
        // A decoded arc is inside only by its endpoint gap: the controller's
        // path between a start and an end radius that disagree is not known
        // closer than that (`travel::ArcGap::Executed`).
        match crate::travel::curve_outside(
            &self.settings.work_to_machine,
            &travel,
            &path,
            crate::travel::ArcGap::Executed,
        )? {
            Some(found) => Err(self.travel_refusal(&found, Some(tool))),
            None => Ok(()),
        }
    }
    fn checked_normal_acceleration(
        &self,
        event: &Event,
        source: bool,
    ) -> Result<Option<NormalAccelerationChecks>, String> {
        let Some(mut checks) = self.normal_acceleration.clone() else {
            return Ok(None);
        };
        let Event::Motion { motion } = event else {
            return Ok(Some(checks));
        };
        let feed = motion.feed_mm_min.or_else(|| {
            (source && motion.kind.is_rapid())
                .then_some(self.settings.traverse_feed_mm_min)
                .flatten()
        });
        let Some(feed) = feed else {
            if source {
                checks.source_rapid_speed_unknown += 1;
            } else {
                checks.decoded_rapid_speed_unknown += 1;
            }
            return Ok(Some(checks));
        };
        let tolerance = if source {
            self.source_tolerance
        } else {
            self.settings.tolerance_mm
        };
        let (acceleration, state) = motion
            .curve
            .prepare(tolerance)?
            .nominal_normal_acceleration_limit(feed, checks.limit_mm_s2)?;
        if state != crate::motion::NormalAccelerationState::Within {
            return Err(format!(
                "{} motion {} nominal normal acceleration {acceleration:.9} mm/s^2 is {:?} at configured limit {:.9}; no batch committed",
                if source { "source" } else { "decoded" },
                motion.id,
                state,
                checks.limit_mm_s2
            ));
        }
        if source {
            checks.source_feed_curves_checked += 1;
        } else {
            checks.decoded_feed_curves_checked += 1;
        }
        Ok(Some(checks))
    }
    fn checked_axis_feed(
        &self,
        event: &Event,
        source: bool,
    ) -> Result<Option<AxisFeedChecks>, String> {
        let Some(mut checks) = self.axis_feed.clone() else {
            return Ok(None);
        };
        let Event::Motion { motion } = event else {
            return Ok(Some(checks));
        };
        let feed = motion.feed_mm_min.or_else(|| {
            (source && motion.kind.is_rapid())
                .then_some(self.settings.traverse_feed_mm_min)
                .flatten()
        });
        let Some(feed) = feed else {
            if source {
                checks.source_rapid_speed_unknown += 1;
            } else {
                checks.decoded_rapid_speed_unknown += 1;
            }
            return Ok(Some(checks));
        };
        let tolerance = if source {
            self.source_tolerance
        } else {
            self.settings.tolerance_mm
        };
        let peaks = motion.curve.prepare(tolerance)?.nominal_axis_feed_limit(
            feed,
            &self.settings.work_to_machine,
            checks.limits_mm_min,
        )?;
        for (axis, (peak, state)) in peaks.into_iter().enumerate() {
            if state != crate::motion::AxisFeedState::Within {
                return Err(format!(
                    "{} motion {} nominal axis {} feed {peak:.9} mm/min is {:?} at configured limit {:.9}; no batch committed",
                    if source { "source" } else { "decoded" },
                    motion.id,
                    ["X", "Y", "Z"][axis],
                    state,
                    checks.limits_mm_min[axis]
                ));
            }
        }
        if source {
            checks.source_feed_curves_checked += 1;
        } else {
            checks.decoded_feed_curves_checked += 1;
        }
        Ok(Some(checks))
    }
    fn chunk(&mut self, event: Option<(&Event, usize)>, lines: &[String]) -> Result<Chunk, String> {
        let mut output = vec![];
        for text in lines {
            if text.len() > 1024
                || !text.is_ascii()
                || text.bytes().any(|b| b < 32 && b != b'\t' || b == 127)
            {
                return Err("post line exceeds ASCII/one-block/1024byte limit".into());
            }
            self.line += 1;
            let motion = event.and_then(|(e, _)| {
                if let Event::Motion { motion } = e {
                    Some(motion)
                } else {
                    None
                }
            });
            output.push(Line {
                number: self.line,
                event_index: event.map(|(_, i)| i),
                operation: motion.map(|m| m.operation.clone()),
                motion_id: motion.map(|m| m.id),
                text: text.clone(),
            });
        }
        Ok(Chunk {
            decoded_initial_position_mm: None,
            identity: self.identity.clone(),
            lines: output,
            checked: vec![],
            scope: if self.settings.lowering.is_some() {
                "python_declared_decoder_lowered_nominal_intent_only"
            } else {
                "python_declared_decoder_nominal_intent_only"
            },
        })
    }
    fn decoded_event(
        &mut self,
        expected: &Event,
        actual: &[Event],
        lines: &[String],
        index: usize,
    ) -> Result<Chunk, String> {
        let start_line = self.line + 1;
        // Blocks: every event writes at least one, except a canned-cycle
        // marker written as explicit moves and the events a written cycle
        // block covers (decoded from that block).
        let mut cycle_event_index = None;
        // A compensation change carried by its move's block: that block
        // decodes to the change and then the move, in that order.
        let full = actual;
        let mut actual = actual;
        let mut carried = false;
        if let Some(change) = self.deferred_compensation.take() {
            if !matches!(expected, Event::Motion { .. })
                || actual.len() != 2
                || actual[0] != change
                || !matches!(actual[1], Event::Motion { .. })
            {
                return Err(format!(
                    "event {index}: the cutter compensation change must be written in this move's block"
                ));
            }
            carried = true;
        }
        if matches!(expected, Event::CutterCompensation { .. })
            && lines.is_empty()
            && actual.is_empty()
            && self
                .cycle
                .is_none_or(|(c, last)| index <= c || index > last)
        {
            self.deferred_compensation = Some(expected.clone());
        } else if let Event::Cycle { covers, .. } = expected {
            if !actual.is_empty() {
                return Err(format!(
                    "event {index}: a canned cycle block decodes to no immediate effect"
                ));
            }
            self.cycle = (!lines.is_empty()).then_some((index, index + *covers as usize));
        } else if let Event::Probe { covers, .. } = expected {
            // A probing cycle is always written as the controller's probing
            // blocks (they decode to the cycle; its moves come from them).
            if lines.is_empty() {
                return Err(format!(
                    "event {index}: a probing cycle must be written as probing blocks"
                ));
            }
            self.cycle = Some((index, index + *covers as usize));
        } else if let Some((cycle, last)) = self.cycle
            && index > cycle
            && index <= last
        {
            if !lines.is_empty() {
                return Err(format!(
                    "event {index}: a move covered by a written canned cycle writes no block"
                ));
            }
            cycle_event_index = Some(cycle);
        } else if lines.is_empty() {
            return Err(format!("event {index}: post wrote no block"));
        }
        self.decoded.consume(actual)?;
        if carried {
            actual = &actual[1..];
        }
        for event in actual {
            self.normal_acceleration = self.checked_normal_acceleration(event, false)?;
            self.axis_feed = self.checked_axis_feed(event, false)?;
        }
        let mut chunk = self.chunk(Some((expected, index)), lines)?;
        // Preserve exactly the decoded control effects, including explicit
        // coolant-off and immaterial stopped-spindle direction. No candidate
        // record escapes if comparison, quota or cancellation later fails.
        let control_effects = full
            .iter()
            .filter(|event| !matches!(event, Event::Motion { .. }))
            .cloned()
            .collect();
        // A canonical exclusive coolant change may contain an explicit off.
        if matches!(expected, Event::Coolant { .. })
            && actual.len() == 2
            && matches!(
                actual[0],
                Event::Coolant {
                    mode: crate::motion::Coolant::Off
                }
            )
        {
            actual = &actual[1..];
        }
        let (motion, deviation) = match (expected, actual) {
            (Event::Motion { motion: a }, []) => {
                let point = self.decoded.summary.final_position_mm;
                let bound = stationary_bound(&a.curve, [0., 1.], point, self.source_tolerance)?;
                if bound > self.settings.tolerance_mm {
                    return Err(format!(
                        "event {index}: stationary output deviation {bound}mm exceeds output tolerance"
                    ));
                }
                self.check_point(point, Some(&a.tool))?;
                (None, Some(bound))
            }
            (Event::Motion { motion: a }, [Event::Motion { motion: b }]) => {
                let expected_feed = a.feed_mm_min.or_else(|| {
                    a.kind
                        .is_rapid()
                        .then_some(self.settings.traverse_feed_mm_min)
                        .flatten()
                });
                if a.id != b.id
                    || a.operation != b.operation
                    || a.tool != b.tool
                    || a.kind != b.kind
                    || match (expected_feed, b.feed_mm_min) {
                        // Per revolution: the feed word was compared with
                        // its FeedPerRev; the bound follows each curve.
                        (Some(_), Some(_))
                            if self.source.feed_per_rev().is_some()
                                && !a.kind.is_rapid()
                                && !matches!(
                                    a.kind,
                                    crate::motion::MoveKind::Tap | crate::motion::MoveKind::Thread
                                ) =>
                        {
                            false
                        }
                        (Some(x), Some(y)) => (x - y).abs() > self.settings.feed_tolerance_mm_min,
                        (None, None) => false,
                        // A straight controller rapid (inside a canned
                        // cycle) where the post writes rapids as a feed.
                        (Some(_), None) => {
                            cycle_event_index.is_none()
                                || a.feed_mm_min.is_some()
                                || !b.kind.is_rapid()
                        }
                        _ => true,
                    }
                {
                    return Err(format!(
                        "event {}: decoded motion identity/tool/feed differs",
                        index
                    ));
                }
                let bound = curve_difference(&a.curve, &b.curve, self.source_tolerance)?;
                if bound > self.settings.tolerance_mm {
                    return Err(format!(
                        "event {}: decoded continuous deviation {bound}mm exceeds output tolerance",
                        index
                    ));
                }
                self.check_curve(&b.curve, &b.tool)?;
                (Some(b.clone()), Some(bound))
            }
            (
                Event::Spindle {
                    rpm: a,
                    direction: x,
                },
                [
                    Event::Spindle {
                        rpm: b,
                        direction: y,
                    },
                ],
            ) if (a - b).abs() <= 0.0005 && (*a == 0. || x == y) => (None, None),
            (
                Event::ToolChange {
                    tool: a,
                    number: an,
                    length_register: ah,
                    position_after_mm: ap,
                },
                [
                    Event::ToolChange {
                        tool: b,
                        number: bn,
                        length_register: bh,
                        position_after_mm: bp,
                    },
                ],
            ) if a == b && an == bn && ah == bh => {
                // A pose-preserving tool selection retains the preceding
                // decoded (rounded) position. Compare its point deviation just
                // as motion coordinates, while tool/register IDs remain exact.
                let bound = (0..3).map(|k| (ap[k] - bp[k]).powi(2)).sum::<f64>().sqrt();
                if bound > self.settings.tolerance_mm {
                    return Err(format!(
                        "event {index}: decoded tool-change pose deviation {bound}mm exceeds output tolerance: the plan starts tool {a} at {ap:?} after its change, the machine leaves it at {bp:?}; plan with the machine's tool-change pose (Setup.tool_change: ToolChangeTable.from_machine for a table, ToolChangePosition for Z home, ToolChangeInPlace when the spindle stays, None to keep the tip)"
                    ));
                }
                self.check_point(*bp, Some(b))?;
                (None, (bound > 0.).then_some(bound))
            }
            // A controller's dwell resolution may round the time: never more
            // than 0.5 ms short, at most 10 ms long.
            (Event::Cycle { .. }, []) => (None, None),
            // The decoded cycle: what is measured and stored exactly, the
            // search within the output tolerance (its words are rounded).
            (
                Event::Probe {
                    cycle: a,
                    covers: n,
                },
                [
                    Event::Probe {
                        cycle: b,
                        covers: m,
                    },
                ],
            ) if n == m
                && a.feature == b.feature
                && a.result == b.result
                && a.retract_z_mm == b.retract_z_mm
                && (a.search_mm - b.search_mm).abs() <= self.settings.tolerance_mm
                && [
                    (a.position_feed_mm_min, b.position_feed_mm_min),
                    (Some(a.measure_feed_mm_min), Some(b.measure_feed_mm_min)),
                    (Some(a.return_feed_mm_min), Some(b.return_feed_mm_min)),
                ]
                .iter()
                .all(|f| match f {
                    (Some(x), Some(y)) => (x - y).abs() <= self.settings.feed_tolerance_mm_min,
                    (None, None) => true,
                    _ => false,
                }) =>
            {
                (None, None)
            }
            // Laser power words have a controller resolution (GRBL: S of
            // $30): within half a percent.
            (Event::Beam { power_percent: a }, [Event::Beam { power_percent: b }])
                if (a - b).abs() <= 0.5 && (*a == 0.) == (*b == 0.) =>
            {
                (None, None)
            }
            // A marking beam decodes as a marking beam (its process words,
            // or on a machine that marks by power alone its attribution),
            // its power within the same resolution.
            (Event::BeamMark { power_percent: a }, [Event::BeamMark { power_percent: b }])
                if (a - b).abs() <= 0.5 =>
            {
                (None, None)
            }
            (Event::CutterCompensation { .. }, []) if self.deferred_compensation.is_some() => {
                (None, None)
            }
            (
                Event::SurfaceSpeed {
                    m_min: a,
                    max_rpm: am,
                    direction: x,
                },
                [
                    Event::SurfaceSpeed {
                        m_min: b,
                        max_rpm: bm,
                        direction: y,
                    },
                ],
            ) if (a - b).abs() <= 0.0005 && (am - bm).abs() <= 0.0005 && x == y => (None, None),
            // The per-revolution feed within what keeps the feed per minute
            // inside the feed tolerance at the machine's top speed.
            (Event::FeedPerRev { mm_rev: a }, [Event::FeedPerRev { mm_rev: b }])
                if (a - b).abs()
                    <= self.settings.feed_tolerance_mm_min
                        / self.settings.capabilities.max_spindle_rpm.max(1.)
                    && (*a == 0.) == (*b == 0.) =>
            {
                (None, None)
            }
            (Event::Dwell { seconds: a }, [Event::Dwell { seconds: b }])
                if *b >= a - 0.0005 && *b <= a + 0.01 =>
            {
                (None, None)
            }
            (_, [single])
                if serde_json::to_value(expected).ok() == serde_json::to_value(single).ok() =>
            {
                if let Event::ToolChange {
                    tool,
                    position_after_mm,
                    ..
                } = single
                {
                    self.check_point(*position_after_mm, Some(tool))?;
                }
                (None, None)
            }
            _ => {
                return Err(format!(
                    "event {}: decoded effects differ from source",
                    index
                ));
            }
        };
        chunk.checked.push(Checked {
            event_index: index,
            line_interval: [start_line, self.line],
            motion,
            deviation_bound_mm: deviation,
            control_effects: Some(control_effects),
            cycle_event_index,
        });
        Ok(chunk)
    }
    fn source_map(
        expected: &Event,
        hash: String,
        index: usize,
        range: [f64; 2],
        check: &Checked,
    ) -> crate::post_lowering::Mapping {
        let id = if let Event::Motion { motion } = expected {
            Some(motion.id)
        } else {
            None
        };
        crate::post_lowering::Mapping {
            event_index: index,
            source_event_sha256: hash,
            source_motion_id: id,
            posted_motion_id: check.motion.as_ref().map(|m| m.id),
            source_fraction_interval: id.map(|_| range),
            line_interval: check.line_interval,
            deviation_bound_mm: check.deviation_bound_mm,
        }
    }
    pub fn preview_source(
        &self,
        source: &[Event],
        cancel: &impl CancelSignal,
    ) -> Result<Vec<Event>, String> {
        if self.finished
            || !self.pending.is_empty()
            || self.settings.lowering.is_some()
            || source.len() > 64
        {
            return Err("direct scheduling preview lifecycle/batch cap".into());
        }
        cancelled(cancel)?;
        let mut validator = self.source.clone();
        validator.consume(source)?;
        let mut events = Vec::with_capacity(source.len());
        for (i, event) in source.iter().enumerate() {
            cancelled(cancel)?;
            events.push(if let Some(policy) = &self.scheduling {
                policy
                    .prepare(
                        event,
                        self.source.summary.events + i,
                        self.source_tolerance,
                        self.settings.traverse_feed_mm_min,
                    )?
                    .event
            } else {
                event.clone()
            });
        }
        check_chunk_size(&events)?;
        cancelled(cancel)?;
        Ok(events)
    }
    pub fn consume(
        &mut self,
        source: &[Event],
        decoded: &[Vec<Event>],
        blocks: &[Vec<String>],
        cancel: &impl CancelSignal,
    ) -> Result<Output, String> {
        if self.finished
            || self.settings.lowering.is_some()
            || source.len() > 64
            || source.len() != decoded.len()
            || source.len() != blocks.len()
            || blocks
                .iter()
                .zip(source)
                .any(|(b, e)| b.len() > block_lines(e))
            || decoded.iter().any(|b| b.len() > 8)
        {
            return Err(
                "generic post lifecycle/batch/block cap; lowering uses start/preview/check".into(),
            );
        }
        cancelled(cancel)?;
        let mut next = self.clone();
        let first = next.source.summary.events;
        next.source.consume(source)?;
        let mut out = next.chunk(None, &[])?;
        let mut mappings = vec![];
        let mut records = vec![];
        for (i, ((expected, actual), lines)) in source.iter().zip(decoded).zip(blocks).enumerate() {
            cancelled(cancel)?;
            let authored = expected;
            let scheduled = next
                .scheduling
                .as_ref()
                .map(|p| {
                    p.prepare(
                        authored,
                        first + i,
                        next.source_tolerance,
                        next.settings.traverse_feed_mm_min,
                    )
                })
                .transpose()?;
            let expected = scheduled.as_ref().map_or(authored, |s| &s.event);
            next.normal_acceleration = next.checked_normal_acceleration(expected, true)?;
            next.axis_feed = next.checked_axis_feed(expected, true)?;
            let chunk = next.decoded_event(expected, actual, lines, first + i)?;
            let hash = format!(
                "{:x}",
                Sha256::digest(serde_json::to_vec(authored).map_err(|e| e.to_string())?)
            );
            let mapping = Self::source_map(authored, hash, first + i, [0., 1.], &chunk.checked[0]);
            if let (Some(policy), Some(scheduled)) = (&mut next.scheduling, &scheduled)
                && let Some(record) = policy.record(
                    scheduled,
                    &mapping,
                    out.checked.len(),
                    chunk.checked[0].motion.as_ref(),
                    next.settings.tolerance_mm,
                )?
            {
                records.push(record);
            }
            mappings.push(mapping);
            out.lines.extend(chunk.lines);
            out.checked.extend(chunk.checked);
        }
        let output = next.output(out, mappings, records);
        check_chunk_size(&output)?;
        cancelled(cancel)?;
        *self = next;
        Ok(output)
    }
    fn output(
        &self,
        chunk: Chunk,
        mappings: Vec<crate::post_lowering::Mapping>,
        records: Vec<FeedAttribution>,
    ) -> Output {
        Output {
            chunk,
            mappings,
            run_id: self.settings.run_id.clone(),
            progress: self
                .settings
                .lowering
                .as_ref()
                .map(|_| crate::post_lowering::Progress {
                    run_id: self.settings.run_id.clone(),
                    source_events_started: self.source.summary.events,
                    source_events_completed: self.source.summary.events - self.pending.len(),
                    posted_motions: self.posted,
                    pending_source_fraction: self.pending.front().map(|p| p.next),
                }),
            lowering_policy: None,
            nominal_normal_acceleration: self.normal_acceleration.clone(),
            nominal_axis_feed: self.axis_feed.clone(),
            motion_scheduling: self.scheduling.as_ref().map(|s| SchedulingOutput {
                summary: s.summary.clone(),
                records,
            }),
        }
    }
    pub fn has_pending(&self) -> bool {
        !self.pending.is_empty()
    }
    pub fn start(&mut self, event: Event, cancel: &impl CancelSignal) -> Result<(), String> {
        self.start_with(event, false, cancel)
    }
    /// `whole`: request one native block for an arc/helix (requires the
    /// lowering option `native_arcs` and a curve within the declared output
    /// capabilities). The decoded block is certified like any other.
    pub fn start_with(
        &mut self,
        event: Event,
        whole: bool,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        // Sources queue for one checked batch, except with feed scheduling
        // (its state follows each checked record).
        if self.finished
            || self.pending.len() >= MAX_QUEUED_SOURCES
            || self.scheduling.is_some() && !self.pending.is_empty()
        {
            return Err("drain pending source before start/finish".into());
        }
        // A closed curve's lowered chain ends at its decoded start, which is
        // known only once every earlier source has been output.
        if !self.pending.is_empty()
            && let Event::Motion { motion } = &event
            && crate::post_lowering::closed_axes(&motion.curve).is_some()
        {
            return Err("drain pending sources before starting a closed arc/helix".into());
        }
        let options = self
            .settings
            .lowering
            .as_ref()
            .ok_or("lowering was not enabled")?;
        cancelled(cancel)?;
        let mut source = self.source.clone();
        source.consume(std::slice::from_ref(&event))?;
        let scheduled = self
            .scheduling
            .as_ref()
            .map(|p| {
                p.prepare(
                    &event,
                    self.source.summary.events,
                    self.source_tolerance,
                    self.settings.traverse_feed_mm_min,
                )
            })
            .transpose()?;
        let event = scheduled
            .as_ref()
            .map_or(event.clone(), |s| s.event.clone());
        // Check the ORIGINAL before chord/radius lowering; line output cannot
        // evade a configured source-curve limit. Do not commit counters yet.
        let normal_acceleration = self.checked_normal_acceleration(&event, true)?;
        let axis_feed = self.checked_axis_feed(&event, true)?;
        if whole {
            let caps = &self.settings.capabilities;
            let allowed = options.native_arcs
                && match &event {
                    Event::Motion { motion } => match &motion.curve {
                        Curve::Arc {
                            plane, sweep_rad, ..
                        } => {
                            caps.arcs
                                && caps.planes.contains(plane)
                                && sweep_rad.abs() <= caps.max_sweep_rad
                        }
                        Curve::Helix {
                            plane, sweep_rad, ..
                        } => {
                            caps.helices
                                && caps.planes.contains(plane)
                                && sweep_rad.abs() <= caps.max_sweep_rad
                        }
                        Curve::Line { .. } => false,
                    },
                    _ => false,
                };
            if !allowed {
                return Err("native arc requires lowering native_arcs and a curve within the machine's arc/helix/plane/sweep capabilities".into());
            }
        }
        let total = if whole {
            1
        } else if let Event::Motion { motion } = &event {
            let n = crate::post_lowering::segment_count(&motion.curve, options.chord_tolerance_mm)?;
            if n > options.max_total_segments.saturating_sub(self.posted) {
                return Err("job segment work budget exceeded before output".into());
            }
            n
        } else {
            1
        };
        if total > options.max_segments_per_source {
            return Err("source segment work budget exceeded before output".into());
        }
        // The decoded start is taken when the source begins emitting.
        let mut pending = crate::post_lowering::SourceCursor::new(
            event,
            self.source.summary.events,
            self.source_tolerance,
            Some(total),
            self.decoded.summary.final_position_mm,
        )?;
        pending.whole = whole;
        if let Some(s) = &scheduled {
            pending.hash = format!(
                "{:x}",
                Sha256::digest(serde_json::to_vec(&s.authored).map_err(|e| e.to_string())?)
            );
        }
        cancelled(cancel)?;
        self.pending_scheduled = scheduled;
        self.source = source;
        self.pending.push_back(pending);
        self.normal_acceleration = normal_acceleration;
        self.axis_feed = axis_feed;
        Ok(())
    }
    pub fn preview(
        &self,
        limit: usize,
        cancel: &impl CancelSignal,
    ) -> Result<Vec<Preview>, String> {
        if self.finished {
            return Err("generic post already finished".into());
        }
        if self.pending.is_empty() {
            return Err("no pending source".into());
        }
        cancelled(cancel)?;
        // Segments of the queued sources in order, motion ids counting
        // motion segments only.
        let mut out = vec![];
        let mut motions = 0;
        for (source, pending) in self.pending.iter().enumerate() {
            if out.len() >= limit {
                break;
            }
            for range in pending.ranges(limit - out.len())? {
                let event = pending.event_for(range, self.posted + motions + 1)?;
                motions += u64::from(matches!(event, Event::Motion { .. }));
                out.push(Preview {
                    event,
                    source_fraction_interval: range,
                    source,
                });
            }
        }
        check_chunk_size(&out)?;
        cancelled(cancel)?;
        Ok(out)
    }
    pub fn check_lowered(
        &mut self,
        ranges: &[[f64; 2]],
        decoded: &[Vec<Event>],
        blocks: &[Vec<String>],
        cancel: &impl CancelSignal,
    ) -> Result<Output, String> {
        if self.finished
            || ranges.is_empty()
            || ranges.len() > 64
            || ranges.len() != decoded.len()
            || ranges.len() != blocks.len()
            || blocks.iter().any(|b| b.len() > PROBE_BLOCK_LINES)
            || decoded.iter().any(|b| b.len() > 8)
        {
            return Err("generic lowering lifecycle/batch/block cap".into());
        }
        cancelled(cancel)?;
        // The batch must be exactly the next segments of the queued sources.
        let mut expected: Vec<[f64; 2]> = vec![];
        for pending in &self.pending {
            if expected.len() >= ranges.len() {
                break;
            }
            expected.extend(pending.ranges(ranges.len() - expected.len())?);
        }
        if self.pending.is_empty() || expected != ranges {
            return Err("source interval gap/overlap or modified lowering plan".into());
        }
        let mut next = self.clone();
        let mut out = next.chunk(None, &[])?;
        let mut mappings = vec![];
        let mut records = vec![];
        for (range, (actual, lines)) in ranges.iter().zip(decoded.iter().zip(blocks)) {
            cancelled(cancel)?;
            let start = next.decoded.summary.final_position_mm;
            let p = next
                .pending
                .front_mut()
                .ok_or("extra intervals after source completion")?;
            if p.emitted == 0 {
                // A queued source starts where the decoded output is now.
                p.actual_start = start;
            }
            let p = &*p;
            let event = p.event_for(*range, next.posted + 1)?;
            let original = p.event.clone();
            let hash = p.hash.clone();
            let index = p.event_index;
            let actual_start = p.actual_start;
            let whole = p.whole;
            if lines.len() > block_lines(&event) {
                return Err("generic lowering lifecycle/batch/block cap".into());
            }
            let mut chunk = next.decoded_event(&event, actual, lines, index)?;
            if let Event::Motion { motion } = &original {
                let decoded = chunk.checked[0].motion.as_ref();
                let has_motion = decoded.is_some();
                let bound = if let Some(decoded) = decoded {
                    if whole {
                        // Native arc: independent whole-curve comparison of the
                        // decoded words against the source curve.
                        if std::mem::discriminant(&motion.curve)
                            != std::mem::discriminant(&decoded.curve)
                        {
                            return Err("native arc decoded as a different curve type".into());
                        }
                        crate::post::curve_difference(
                            &motion.curve,
                            &decoded.curve,
                            next.source_tolerance,
                        )?
                    } else {
                        crate::post_lowering::certify(
                            motion,
                            *range,
                            decoded,
                            next.source_tolerance,
                        )?
                    }
                } else {
                    stationary_bound(
                        &motion.curve,
                        *range,
                        next.decoded.summary.final_position_mm,
                        next.source_tolerance,
                    )?
                };
                if bound > next.settings.tolerance_mm {
                    return Err(format!(
                        "source {} interval {range:?}: continuous lowering error {bound}mm exceeds output tolerance",
                        motion.id
                    ));
                }
                if range[1] == 1.
                    && let Some(decoded) = decoded
                {
                    crate::post_lowering::check_closure(
                        &motion.curve,
                        actual_start,
                        decoded.curve.end(),
                    )?;
                }
                chunk.checked[0].deviation_bound_mm = Some(bound);
                for line in &mut chunk.lines {
                    line.motion_id = Some(motion.id);
                }
                next.posted += u64::from(has_motion);
            }
            let mapping = Self::source_map(&original, hash, index, *range, &chunk.checked[0]);
            if let (Some(policy), Some(scheduled)) = (&mut next.scheduling, &next.pending_scheduled)
                && let Some(record) = policy.record(
                    scheduled,
                    &mapping,
                    out.checked.len(),
                    chunk.checked[0].motion.as_ref(),
                    next.settings.tolerance_mm,
                )?
            {
                records.push(record);
            }
            mappings.push(mapping);
            out.lines.extend(chunk.lines);
            out.checked.extend(chunk.checked);
            let p = next.pending.front_mut().unwrap();
            p.next = range[1];
            p.emitted += 1;
            if range[1] == 1. {
                next.pending.pop_front();
                next.pending_scheduled = None;
            }
        }
        let output = next.output(out, mappings, records);
        check_chunk_size(&output)?;
        cancelled(cancel)?;
        *self = next;
        Ok(output)
    }
    pub fn finish(
        &mut self,
        lines: &[String],
        cancel: &impl CancelSignal,
    ) -> Result<Output, String> {
        if self.finished
            || !self.pending.is_empty()
            || self.deferred_compensation.is_some()
            || lines.is_empty()
            || lines.len() > 8
        {
            return Err("generic post finish lifecycle/cap".into());
        }
        cancelled(cancel)?;
        let mut next = self.clone();
        let chunk = next.chunk(None, lines)?;
        let output = next.output(chunk, vec![], vec![]);
        check_chunk_size(&output)?;
        cancelled(cancel)?;
        next.finished = true;
        *self = next;
        Ok(output)
    }
}

#[cfg(test)]
mod normal_limit_tests {
    use super::*;
    use crate::motion::{Curve, Motion, MoveKind, Plane, SpindleDirection};
    use std::{cell::Cell, f64::consts::PI};

    pub(super) fn fixture(limit: Option<f64>, lower: bool) -> (IntentChecker, Vec<Event>, Program) {
        let manifest: crate::stream::StreamManifest =
            serde_json::from_str(include_str!("../fixtures/operations/adaptive-pocket.json"))
                .unwrap();
        let initial = [2., 0., 5.];
        let header = Program {
            version: 1,
            id: "curve-post".into(),
            revision: "1".into(),
            units: "mm".into(),
            tolerance_mm: 1e-6,
            frame: WorkFrame::identity(),
            tools: manifest.tools,
            initial_position_mm: initial,
            events: vec![],
        };
        let settings: Settings = serde_json::from_value(serde_json::json!({
            "capabilities":{"id":"plain_machine","planes":["xy","xz","yz"],"arcs":true,"helices":true,"max_sweep_rad":100.,"max_feed_mm_min":10000.,"max_spindle_rpm":30000.,"max_tool_number":99999,"max_length_register":99999,"max_work_offset":9,"tool_changes":true,"flood":true,"mist":true},
            "tolerance_mm":0.01,"feed_tolerance_mm_min":0.1,"work_to_machine":WorkFrame::identity(),"tip_travel_mm":null,
            "definition":{"plain":"machine"},"run_id":"curves","decoded_initial_position_mm":initial,
            "max_normal_accel_mm_s2":limit,"lowering":if lower {Some(crate::post_lowering::Options::default())}else{None}
        })).unwrap();
        let (checker, _) = IntentChecker::begin(&header, settings, &["BEGIN".into()]).unwrap();
        let init = vec![
            Event::WorkOffset { number: 1 },
            Event::ToolChange {
                tool: "t1".into(),
                number: 1,
                length_register: 1,
                position_after_mm: initial,
            },
            Event::Spindle {
                rpm: 1000.,
                direction: SpindleDirection::Cw,
            },
        ];
        (checker, init, header)
    }
    pub(super) fn initialize(checker: &mut IntentChecker, init: &[Event], lower: bool) {
        if lower {
            for event in init {
                checker.start(event.clone(), &|| false).unwrap();
                checker
                    .check_lowered(
                        &[[0., 1.]],
                        &[vec![event.clone()]],
                        &[vec!["CONTROL".into()]],
                        &|| false,
                    )
                    .unwrap();
            }
        } else {
            checker
                .consume(
                    init,
                    &init.iter().cloned().map(|e| vec![e]).collect::<Vec<_>>(),
                    &vec![vec!["CONTROL".into()]; init.len()],
                    &|| false,
                )
                .unwrap();
        }
    }
    fn arc(center: f64, end: f64, feed: f64) -> Event {
        Event::Motion {
            motion: Motion {
                id: 1,
                operation: "curve".into(),
                tool: "t1".into(),
                kind: MoveKind::Cut,
                feed_mm_min: Some(feed),
                curve: Curve::Arc {
                    start: [2., 0., 5.],
                    end: [end, 0., 5.],
                    center: [center, 0., 5.],
                    plane: Plane::Xy,
                    sweep_rad: PI,
                },
            },
        }
    }

    #[test]
    fn stationary_output_checks_whole_line_and_cannot_erase_circular_winding() {
        let (mut checker, init, _) = fixture(None, false);
        initialize(&mut checker, &init, false);
        let tiny = Event::Motion {
            motion: Motion {
                id: 1,
                operation: "link".into(),
                tool: "t1".into(),
                kind: MoveKind::Link,
                feed_mm_min: Some(600.),
                curve: Curve::Line {
                    start: [2., 0., 5.],
                    end: [2.0001, 0., 5.],
                },
            },
        };
        let out = checker
            .consume(
                &[tiny],
                &[vec![]],
                &[vec!["G1 X2 Y0 Z5 F600".into()]],
                &|| false,
            )
            .unwrap();
        assert!(out.chunk.checked[0].motion.is_none());
        assert!((out.chunk.checked[0].deviation_bound_mm.unwrap() - 0.0001).abs() < 1e-12);
        assert_eq!(out.mappings[0].source_motion_id, Some(1));
        assert_eq!(out.mappings[0].posted_motion_id, None);
        let far = Event::Motion {
            motion: Motion {
                id: 2,
                operation: "cut".into(),
                tool: "t1".into(),
                kind: MoveKind::Cut,
                feed_mm_min: Some(600.),
                curve: Curve::Line {
                    start: [2.0001, 0., 5.],
                    end: [3., 0., 5.],
                },
            },
        };
        assert!(
            checker
                .consume(
                    std::slice::from_ref(&far),
                    &[vec![]],
                    &[vec!["G1 X2 Y0 Z5 F600".into()]],
                    &|| false
                )
                .is_err()
        );
        let mut actual = far.clone();
        if let Event::Motion { motion } = &mut actual {
            motion.curve = Curve::Line {
                start: [2., 0., 5.],
                end: [3., 0., 5.],
            };
        }
        checker
            .consume(
                &[far],
                &[vec![actual]],
                &[vec!["G1 X3 Y0 Z5 F600".into()]],
                &|| false,
            )
            .unwrap();
        let (mut checker, init, _) = fixture(None, false);
        initialize(&mut checker, &init, false);
        let turn = Event::Motion {
            motion: Motion {
                id: 1,
                operation: "turn".into(),
                tool: "t1".into(),
                kind: MoveKind::Cut,
                feed_mm_min: Some(600.),
                curve: Curve::Arc {
                    start: [2., 0., 5.],
                    end: [2., 0., 5.],
                    center: [1.9999, 0., 5.],
                    plane: Plane::Xy,
                    sweep_rad: std::f64::consts::TAU,
                },
            },
        };
        assert!(
            checker
                .consume(&[turn], &[vec![]], &[vec!["NO MOTION".into()]], &|| false)
                .err()
                .unwrap()
                .contains("circular")
        );
    }
    #[test]
    fn stationary_minor_curve_enclosure_covers_interior_and_rejects_large_erasure() {
        for plane in [Plane::Xy, Plane::Xz, Plane::Yz] {
            let [u, v, z] = plane.axes();
            for sign in [-1., 1.] {
                let mut a = [20., -30., 40.];
                let center = a;
                a[u] += 3.;
                let sweep: f64 = sign * 0.0001;
                let mut b = center;
                b[u] += 3. * sweep.cos();
                b[v] += 3. * sweep.sin();
                b[z] += 0.0001;
                let curve = Curve::Helix {
                    start: a,
                    end: b,
                    center,
                    plane,
                    sweep_rad: sweep,
                };
                let bound = stationary_bound(&curve, [0., 1.], a, 1e-5).unwrap();
                assert!(bound < 0.001);
                let path = curve.prepare(1e-5).unwrap();
                for i in 0..=100 {
                    let p = path.position(i as f64 / 100.).unwrap();
                    let distance = (0..3).map(|k| (p[k] - a[k]).powi(2)).sum::<f64>().sqrt();
                    assert!(distance <= bound);
                }
                let mut far = a;
                far[u] += 1.;
                assert!(stationary_bound(&curve, [0., 1.], far, 1e-5).is_err());
            }
        }
    }
    #[test]
    fn malformed_typed_limits_refuse_begin_and_none_is_omitted_from_settings() {
        let (checker, _, header) = fixture(Some(40.), false);
        for value in [0., -1., f64::NAN, f64::INFINITY, 1e10] {
            let mut settings = checker.settings.clone();
            settings.max_normal_accel_mm_s2 = Some(value);
            assert!(IntentChecker::begin(&header, settings, &["BEGIN".into()]).is_err());
        }
        let mut settings = checker.settings.clone();
        settings.max_normal_accel_mm_s2 = None;
        assert!(
            !serde_json::to_value(&settings)
                .unwrap()
                .as_object()
                .unwrap()
                .contains_key("max_normal_accel_mm_s2")
        );
        let (_, plain) = IntentChecker::begin(&header, settings, &["BEGIN".into()]).unwrap();
        assert_ne!(
            plain.chunk.identity.header_setup_sha256,
            checker.identity.header_setup_sha256
        );
    }
    #[test]
    fn source_decoded_boundary_and_cancel_failures_preserve_batch_and_scope_counters() {
        let (mut checker, init, _) = fixture(Some(40.), false);
        initialize(&mut checker, &init, false);
        let before = checker.line;
        let bad = arc(0., -2., 600.);
        assert!(
            checker
                .consume(
                    std::slice::from_ref(&bad),
                    &[vec![bad.clone()]],
                    &[vec!["BAD".into()]],
                    &|| { false }
                )
                .err()
                .unwrap()
                .contains("source motion")
        );
        assert_eq!(checker.line, before);
        assert_eq!(
            checker
                .normal_acceleration
                .as_ref()
                .unwrap()
                .source_feed_curves_checked,
            0
        );
        let good = arc(0., -2., 480.);
        let polls = Cell::new(0);
        assert!(
            checker
                .consume(
                    std::slice::from_ref(&good),
                    &[vec![good.clone()]],
                    &[vec!["GOOD".into()]],
                    &|| {
                        polls.set(polls.get() + 1);
                        polls.get() >= 3
                    }
                )
                .err()
                .unwrap()
                .contains("cancelled")
        );
        assert_eq!(checker.line, before);
        let result = checker
            .consume(
                std::slice::from_ref(&good),
                &[vec![good.clone()]],
                &[vec!["GOOD".into()]],
                &|| false,
            )
            .unwrap();
        let checks = result.nominal_normal_acceleration.unwrap();
        assert_eq!(
            (
                checks.source_feed_curves_checked,
                checks.decoded_feed_curves_checked
            ),
            (1, 1)
        );
        assert_eq!(result.chunk.lines[0].number, before + 1);

        let (mut checker, init, _) = fixture(Some(49.995), false);
        initialize(&mut checker, &init, false);
        let source = arc(-0.0004, -2.0008, 600.);
        let decoded = arc(0., -2., 600.);
        assert!(
            checker
                .consume(
                    std::slice::from_ref(&source),
                    &[vec![decoded]],
                    &[vec!["ROUNDED".into()]],
                    &|| false
                )
                .err()
                .unwrap()
                .contains("decoded motion")
        );
        assert_eq!(
            checker
                .normal_acceleration
                .as_ref()
                .unwrap()
                .source_feed_curves_checked,
            0
        );
        checker
            .consume(
                std::slice::from_ref(&source),
                &[vec![source.clone()]],
                &[vec!["EXACT".into()]],
                &|| false,
            )
            .unwrap();

        let (mut checker, init, _) = fixture(Some(50.), false);
        initialize(&mut checker, &init, false);
        let at_limit = arc(0., -2., 600.);
        assert!(
            checker
                .consume(
                    std::slice::from_ref(&at_limit),
                    &[vec![at_limit.clone()]],
                    &[vec!["LIMIT".into()]],
                    &|| false
                )
                .err()
                .unwrap()
                .contains("Boundary")
        );
    }
    #[test]
    fn lowering_checks_original_before_chords_and_disabled_limits_preserve_old_wire() {
        let (mut checker, init, _) = fixture(Some(40.), true);
        initialize(&mut checker, &init, true);
        assert!(
            checker
                .start(arc(0., -2., 600.), &|| false)
                .err()
                .unwrap()
                .contains("source motion")
        );
        assert!(!checker.has_pending());
        assert_eq!(
            checker
                .normal_acceleration
                .as_ref()
                .unwrap()
                .source_feed_curves_checked,
            0
        );
        checker.start(arc(0., -2., 480.), &|| false).unwrap();
        assert!(checker.has_pending());
        assert_eq!(
            checker
                .normal_acceleration
                .as_ref()
                .unwrap()
                .source_feed_curves_checked,
            1
        );
        let (mut checker, init, _) = fixture(None, false);
        initialize(&mut checker, &init, false);
        let fast = arc(0., -2., 600.);
        let out = checker
            .consume(
                std::slice::from_ref(&fast),
                &[vec![fast.clone()]],
                &[vec!["FAST".into()]],
                &|| false,
            )
            .unwrap();
        assert!(out.nominal_normal_acceleration.is_none());
        assert!(
            !serde_json::to_value(out)
                .unwrap()
                .as_object()
                .unwrap()
                .contains_key("nominal_normal_acceleration")
        );
    }
}

#[cfg(test)]
mod gauge_travel_tests {
    use super::normal_limit_tests::fixture;
    use super::*;
    use crate::motion::{Curve, Motion, MoveKind, SpindleDirection};

    /// A program of two tools, `t1` and a copy `long` 55 mm longer, each
    /// rising to work Z `top`; the gauge line travels Z -400..0 and the work
    /// origin is at machine Z -300.
    fn run(top: f64, tilt: bool) -> Result<(), String> {
        let (checker, _, mut header) = fixture(None, false);
        let mut long = header.tools[0].clone();
        long.id = "long".into();
        long.gauge_length_mm += 55.;
        long.holders.last_mut().unwrap().z_max_mm += 55.;
        long.prepare().unwrap();
        header.tools.push(long);
        let short_length = header.tools[0].gauge_length_mm;
        let start = [2., 0., 5.];
        let mut settings = checker.settings.clone();
        settings.gauge_travel_mm = Some([[-500., -500., -400.], [500., 500., 0.]]);
        settings.work_to_machine.origin_mm = [0., 0., -300.];
        if tilt {
            settings.work_to_machine.y = [0., 0., 1.];
            settings.work_to_machine.z = [0., -1., 0.];
        }
        let (mut checker, _) = IntentChecker::begin(&header, settings, &["BEGIN".into()])?;
        let mut events = vec![Event::WorkOffset { number: 1 }];
        for (i, tool) in ["t1", "long"].into_iter().enumerate() {
            events.extend([
                Event::ToolChange {
                    tool: tool.into(),
                    number: i as u32 + 1,
                    length_register: i as u32 + 1,
                    position_after_mm: start,
                },
                Event::Spindle {
                    rpm: 1000.,
                    direction: SpindleDirection::Cw,
                },
                Event::Motion {
                    motion: Motion {
                        id: 2 * i as u64 + 1,
                        operation: "op".into(),
                        tool: tool.into(),
                        kind: MoveKind::Cut,
                        curve: Curve::Line {
                            start,
                            end: [2., 0., top - short_length],
                        },
                        feed_mm_min: Some(600.),
                    },
                },
                Event::Motion {
                    motion: Motion {
                        id: 2 * i as u64 + 2,
                        operation: "op".into(),
                        tool: tool.into(),
                        kind: MoveKind::Cut,
                        curve: Curve::Line {
                            start: [2., 0., top - short_length],
                            end: start,
                        },
                        feed_mm_min: Some(600.),
                    },
                },
                Event::Spindle {
                    rpm: 0.,
                    direction: SpindleDirection::Cw,
                },
            ]);
        }
        checker
            .consume(
                &events,
                &events.iter().cloned().map(|e| vec![e]).collect::<Vec<_>>(),
                &vec![vec!["BLOCK".into()]; events.len()],
                &|| false,
            )
            .map(|_| ())
    }

    #[test]
    fn decoded_moves_are_checked_against_each_tools_own_gauge_line_travel() {
        // Work Z 300 is machine Z 0: `top` is the gauge line's height with
        // the short tool. 245: the long tool's gauge line is at its limit.
        run(245., false).unwrap();
        run(200., false).unwrap();
        // 300: the short tool at Z home, on its limit; the long tool's gauge
        // line would be 55 mm above it.
        let e = run(300., false).unwrap_err();
        assert!(
            e.contains("tool long") && e.contains("machine Z") && e.contains("55.0"),
            "{e}"
        );
        // Above home is outside for the short tool too, named first.
        let e = run(300.001, false).unwrap_err();
        assert!(e.contains("tool t1") && e.contains("machine Z"), "{e}");
        assert!(
            run(200., true)
                .unwrap_err()
                .contains("work Z axis along machine Z")
        );
    }

    #[test]
    fn travel_is_declared_once_and_a_tip_box_is_recorded_as_before() {
        let (checker, _, header) = fixture(None, false);
        let mut settings = checker.settings.clone();
        assert!(
            !serde_json::to_value(&settings)
                .unwrap()
                .as_object()
                .unwrap()
                .contains_key("gauge_travel_mm")
        );
        settings.tip_travel_mm = Some([[-10.; 3], [10.; 3]]);
        settings.gauge_travel_mm = settings.tip_travel_mm;
        let e = IntentChecker::begin(&header, settings.clone(), &["BEGIN".into()])
            .err()
            .unwrap();
        assert!(e.contains("declared once"), "{e}");
        settings.tip_travel_mm = None;
        settings.gauge_travel_mm = Some([[10.; 3], [-10.; 3]]);
        assert!(IntentChecker::begin(&header, settings, &["BEGIN".into()]).is_err());
    }
}

#[cfg(test)]
#[path = "post_intent/scheduling_tests.rs"]
mod scheduling_tests;

#[cfg(test)]
#[path = "post_intent/travel_tests.rs"]
mod travel_tests;
