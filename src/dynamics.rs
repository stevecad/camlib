//! Nominal path timing and explicit feed scheduling; not a controller emulator.
use crate::{
    motion::{Curve, Event, Motion, Program, ProgramValidator, Vec3, WorkFrame},
    stock::CancelSignal,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum JunctionMode {
    ExactStop,
    Tangent,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Policy {
    pub tangential_accel_mm_s2: f64,
    pub rapid_feed_mm_min: f64,
    pub junction_mode: JunctionMode,
    /// Reserve on configured speed/normal-acceleration caps, not geometry error.
    pub feed_reserve_fraction: f64,
    /// Seconds added to every segment that ends at rest: a controller's
    /// per-stop cost that tangential acceleration does not model, such as
    /// acceleration/deceleration after interpolation, block processing and
    /// in-position checking. Zero (the default) is omitted from identity.
    #[serde(default, skip_serializing_if = "is_zero")]
    pub per_block_s: f64,
}
fn is_zero(x: &f64) -> bool {
    *x == 0.
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Settings {
    pub policy: Policy,
    pub max_feed_mm_min: f64,
    pub max_normal_accel_mm_s2: Option<f64>,
    pub max_axis_feed_mm_min: Option<Vec3>,
    pub work_to_machine: WorkFrame,
    /// Output feed quantum in mm/min; scheduled feeds round downward.
    pub feed_quantum_mm_min: f64,
    pub adjust_feeds: bool,
}
#[derive(Clone, Debug, Serialize)]
pub struct Segment {
    pub event_index: usize,
    pub motion_id: u64,
    pub length_mm: f64,
    pub original_feed_mm_min: Option<f64>,
    pub scheduled_feed_mm_min: Option<f64>,
    pub speed_cap_mm_s: f64,
    pub entry_mm_s: f64,
    pub exit_mm_s: f64,
    pub peak_mm_s: f64,
    pub seconds: f64,
    pub rapid_estimate: bool,
}
#[derive(Debug, Serialize)]
pub struct Report {
    pub version: u32,
    pub identity: String,
    pub settings: Settings,
    pub motion_seconds: f64,
    pub feed_limited_motions: usize,
    pub unmodeled_state_events: usize,
    pub segments: Vec<Segment>,
    pub scope: &'static str,
}

impl Settings {
    pub fn validate(&self) -> Result<(), String> {
        let positive = |x: f64| x.is_finite() && x > 0.;
        // Numeric safety only: these values are the machine's declarations
        // (#433).
        if !positive(self.policy.tangential_accel_mm_s2) {
            return Err(format!(
                "motion policy: tangential_accel_mm_s2 {} must be positive and finite (MotionPolicy.tangential_accel_mm_s2)",
                self.policy.tangential_accel_mm_s2
            ));
        }
        if !positive(self.policy.rapid_feed_mm_min) {
            return Err(format!(
                "motion policy: rapid_feed_mm_min {} must be positive and finite (MotionPolicy.rapid_feed_mm_min)",
                self.policy.rapid_feed_mm_min
            ));
        }
        if !positive(self.max_feed_mm_min) {
            return Err(format!(
                "motion policy: max_feed_mm_min {} must be positive and finite (Machine.max_feed_mm_min)",
                self.max_feed_mm_min
            ));
        }
        // Definition: a fraction in [0, 1) so (1 - reserve) stays positive.
        if !self.policy.feed_reserve_fraction.is_finite()
            || !(0.0..1.0).contains(&self.policy.feed_reserve_fraction)
        {
            return Err(format!(
                "motion policy: feed_reserve_fraction {} must be finite and in [0, 1) (MotionPolicy.feed_reserve_fraction)",
                self.policy.feed_reserve_fraction
            ));
        }
        // Numeric safety only: finite and nonnegative (#433).
        if !(self.policy.per_block_s.is_finite() && self.policy.per_block_s >= 0.) {
            return Err(format!(
                "motion policy: per_block_s {} must be finite and nonnegative (MotionPolicy.per_block_s)",
                self.policy.per_block_s
            ));
        }
        // Numeric safety only: positive and finite (derived from
        // Machine.output_units and Machine.decimals); quantization checks that
        // the resulting scheduled feed is finite and positive.
        if !positive(self.feed_quantum_mm_min) {
            return Err(format!(
                "motion policy: feed_quantum_mm_min {} must be positive and finite",
                self.feed_quantum_mm_min
            ));
        }
        if let Some(x) = self.max_normal_accel_mm_s2
            && !positive(x)
        {
            return Err(format!(
                "motion policy: max_normal_accel_mm_s2 {x} must be positive and finite (Machine.max_normal_accel_mm_s2)"
            ));
        }
        if let Some(v) = self.max_axis_feed_mm_min {
            for (k, x) in v.into_iter().enumerate() {
                if !positive(x) {
                    return Err(format!(
                        "motion policy: max_axis_feed_mm_min[{k}] {x} must be positive and finite (Machine.max_axis_feed_mm_min)"
                    ));
                }
            }
        }
        self.work_to_machine.validate()
    }
}
fn cancelled(cancel: &impl CancelSignal) -> Result<(), String> {
    if cancel.is_cancelled() {
        Err("motion scheduling cancelled; no result committed".into())
    } else {
        Ok(())
    }
}
fn tangent(curve: &Curve, end: bool, length: f64) -> Vec3 {
    let (a, b) = (curve.start(), curve.end());
    match curve {
        Curve::Line { .. } => std::array::from_fn(|k| (b[k] - a[k]) / length),
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
        } => {
            let [u, v, w] = plane.axes();
            let angle =
                (a[v] - center[v]).atan2(a[u] - center[u]) + if end { *sweep_rad } else { 0. };
            let radius = (a[u] - center[u]).hypot(a[v] - center[v]);
            let (s, c) = angle.sin_cos();
            let mut t = [0.; 3];
            t[u] = -s * radius * sweep_rad / length;
            t[v] = c * radius * sweep_rad / length;
            t[w] = (b[w] - a[w]) / length;
            t
        }
    }
}

/// Shared per-motion cap: callers validate settings once and keep state atomic.
pub(crate) fn capped_segment(
    motion: &Motion,
    event_index: usize,
    tolerance: f64,
    settings: &Settings,
) -> Result<Segment, String> {
    motion.validate_feed(None)?;
    let path = motion.curve.prepare(tolerance)?;
    let length = path.length_mm();
    let original = motion.feed_mm_min;
    let requested = original.unwrap_or(settings.policy.rapid_feed_mm_min);
    let mut cap = if original.is_some() {
        settings.max_feed_mm_min
    } else {
        settings.policy.rapid_feed_mm_min
    };
    if let Some(limit) = settings.max_normal_accel_mm_s2 {
        let at_unit = path.nominal_normal_acceleration(60.)?;
        if at_unit > 0. {
            cap = cap.min(60. * (limit / at_unit).sqrt());
        }
    }
    if let Some(limits) = settings.max_axis_feed_mm_min {
        let peaks = path.nominal_axis_feed(60., &settings.work_to_machine)?;
        for k in 0..3 {
            if peaks[k] > 0. {
                cap = cap.min(60. * limits[k] / peaks[k]);
            }
        }
    }
    cap *= 1. - settings.policy.feed_reserve_fraction;
    let mut feed = requested;
    if requested > cap {
        if original.is_some() && !settings.adjust_feeds {
            return Err(format!(
                "motion {} feed exceeds configured scheduling cap; enable adjust_feeds",
                motion.id
            ));
        }
        if original.is_some() {
            // Leave one word quantum below an active limit for independent
            // decoded-word checks. Geometry rounding remains independently checked.
            feed =
                ((cap / settings.feed_quantum_mm_min).floor() - 1.) * settings.feed_quantum_mm_min;
        } else {
            feed = cap;
        }
    }
    if !feed.is_finite() || feed < 1e-6 {
        return Err(format!(
            "motion {} has no representable positive scheduled feed",
            motion.id
        ));
    }
    let speed = feed / 60.;
    let mut segment = Segment {
        event_index,
        motion_id: motion.id,
        length_mm: length,
        original_feed_mm_min: original,
        scheduled_feed_mm_min: original.map(|_| feed),
        speed_cap_mm_s: speed,
        entry_mm_s: 0.,
        exit_mm_s: 0.,
        peak_mm_s: 0.,
        seconds: 0.,
        rapid_estimate: original.is_none(),
    };
    time_segment(&mut segment, 0., 0., settings.policy.tangential_accel_mm_s2);
    segment.seconds += settings.policy.per_block_s;
    Ok(segment)
}

pub(crate) fn time_segment(s: &mut Segment, u: f64, v: f64, a: f64) {
    let peak = (a * s.length_mm + (u * u + v * v) / 2.)
        .sqrt()
        .min(s.speed_cap_mm_s)
        .max(u.max(v));
    let acc = (peak - u).max(0.);
    let dec = (peak - v).max(0.);
    let ramps = (acc * (peak + u) + dec * (peak + v)) / (2. * a);
    s.entry_mm_s = u;
    s.exit_mm_s = v;
    s.peak_mm_s = peak;
    s.seconds = (acc + dec) / a + (s.length_mm - ramps).max(0.) / peak;
}

/// Use the actual decoded feed, without applying a scheduling reserve twice.
pub(crate) fn decoded_seconds(
    motion: &Motion,
    tolerance: f64,
    settings: &Settings,
) -> Result<f64, String> {
    if motion.feed_mm_min.is_none() {
        return Ok(capped_segment(motion, 0, tolerance, settings)?.seconds);
    }
    let mut segment = Segment {
        event_index: 0,
        motion_id: motion.id,
        length_mm: motion.curve.prepare(tolerance)?.length_mm(),
        original_feed_mm_min: motion.feed_mm_min,
        scheduled_feed_mm_min: motion.feed_mm_min,
        speed_cap_mm_s: motion.feed_mm_min.unwrap() / 60.,
        entry_mm_s: 0.,
        exit_mm_s: 0.,
        peak_mm_s: 0.,
        seconds: 0.,
        rapid_estimate: false,
    };
    time_segment(&mut segment, 0., 0., settings.policy.tangential_accel_mm_s2);
    Ok(segment.seconds + settings.policy.per_block_s)
}

/// O(n) forward/backward lookahead on one complete, validated program. Returns
/// feeds and timing indexed to original events; it never edits curves or events.
/// Rapid speeds are estimates only and are never converted into programmed feeds.
pub fn schedule(
    program: &Program,
    settings: Settings,
    cancel: &impl CancelSignal,
) -> Result<Report, String> {
    cancelled(cancel)?;
    settings.validate()?;
    if program.events.len() > 1_000_000 {
        return Err("motion scheduling exceeds one million events".into());
    }
    let mut validator = ProgramValidator::new(program, None)?;
    // Validation/cancellation in bounded chunks, without cloning the whole job.
    for chunk in program.events.chunks(256) {
        cancelled(cancel)?;
        validator.consume(chunk)?;
    }
    let mut segments: Vec<Segment> = Vec::with_capacity(validator.summary.motions);
    let mut joins = Vec::with_capacity(validator.summary.motions + 1);
    joins.push(0.);
    let mut previous: Option<(usize, Vec3)> = None;
    let mut limited = 0;
    let mut unmodeled = 0;
    let mut identity = Sha256::new();
    identity.update(b"camlib-motion-policy-v1\0");
    identity.update(serde_json::to_vec(&settings).map_err(|e| e.to_string())?);
    // Header identity without duplicating retained events.
    identity.update(
        serde_json::to_vec(&(
            program.version,
            &program.id,
            &program.revision,
            &program.units,
            program.tolerance_mm,
            &program.frame,
            program.initial_position_mm,
            &program.tools,
        ))
        .map_err(|e| e.to_string())?,
    );
    for (index, event) in program.events.iter().enumerate() {
        cancelled(cancel)?;
        identity.update(serde_json::to_vec(event).map_err(|e| e.to_string())?);
        let Event::Motion { motion } = event else {
            previous = None;
            unmodeled += 1;
            continue;
        };
        let segment = capped_segment(motion, index, program.tolerance_mm, &settings)?;
        let length = segment.length_mm;
        let original = segment.original_feed_mm_min;
        let speed = segment.speed_cap_mm_s;
        limited += usize::from(original.is_some() && original != segment.scheduled_feed_mm_min);
        let start_tangent = tangent(&motion.curve, false, length);
        if settings.policy.junction_mode == JunctionMode::Tangent
            && original.is_some()
            && let Some((prior_index, end_tangent)) = previous
        {
            let Event::Motion { motion: prior } = &program.events[prior_index] else {
                unreachable!()
            };
            // Numerical equality only, not a user machining/blend tolerance.
            // Non-tangent corners always stop; no unmodeled corner rounding.
            if prior.feed_mm_min.is_some()
                && prior.tool == motion.tool
                && prior.curve.end() == motion.curve.start()
                && (0..3).all(|k| (start_tangent[k] - end_tangent[k]).abs() <= 64. * f64::EPSILON)
            {
                *joins.last_mut().unwrap() = speed.min(segments.last().unwrap().speed_cap_mm_s);
            }
        }
        previous = Some((index, tangent(&motion.curve, true, length)));
        segments.push(segment);
        joins.push(0.);
    }
    let a = settings.policy.tangential_accel_mm_s2;
    for i in (0..segments.len()).rev() {
        cancelled(cancel)?;
        joins[i] =
            joins[i].min((joins[i + 1] * joins[i + 1] + 2. * a * segments[i].length_mm).sqrt());
    }
    for i in 0..segments.len() {
        cancelled(cancel)?;
        joins[i + 1] =
            joins[i + 1].min((joins[i] * joins[i] + 2. * a * segments[i].length_mm).sqrt());
    }
    let mut seconds = 0.;
    for (i, s) in segments.iter_mut().enumerate() {
        cancelled(cancel)?;
        time_segment(s, joins[i], joins[i + 1], a);
        if joins[i + 1] == 0. {
            s.seconds += settings.policy.per_block_s;
        }
        seconds += s.seconds;
        identity.update(s.scheduled_feed_mm_min.unwrap_or(0.).to_le_bytes());
    }
    cancelled(cancel)?;
    Ok(Report {
        version: 1,
        identity: format!("{:x}", identity.finalize()),
        settings,
        motion_seconds: seconds,
        feed_limited_motions: limited,
        unmodeled_state_events: unmodeled,
        segments,
        scope: "Nominal line/arc/helix path; constant scalar tangential acceleration, optional separate normal/axis-speed limits, no jerk or axis-acceleration model. Stops at state events, rapid boundaries and non-tangent corners; each stop adds the policy's per-block seconds. Rapid speed is an authored estimate. Excludes tool-change/spindle/operator dwell, endpoint correction and controller behavior; not physical cycle time.",
    })
}
