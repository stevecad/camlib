//! Bounded source-to-linear lowering and independently checked source intervals.
use crate::{
    motion::{Curve, Event, Motion, Plane, PreparedCurve, Vec3},
    post::{self, Chunk},
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::f64::consts::FRAC_PI_2;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Options {
    pub chord_tolerance_mm: f64,
    pub max_segments_per_source: u64,
    pub max_total_segments: u64,
    /// Keep an arc/helix as one native G2/G3 block when the machine accepts
    /// it and exact words exist; lower only the others (per-move fallback,
    /// like CAM post processors' linearize). Every block is still decoded and
    /// certified against the source curve.
    #[serde(default, skip_serializing_if = "is_false")]
    pub native_arcs: bool,
}
fn is_false(value: &bool) -> bool {
    !*value
}
impl Default for Options {
    fn default() -> Self {
        Self {
            chord_tolerance_mm: 0.0025,
            max_segments_per_source: 1_000_000,
            max_total_segments: 10_000_000,
            native_arcs: false,
        }
    }
}
impl Options {
    pub(crate) fn validate(&self, tolerance: f64) -> Result<(), String> {
        if !self.chord_tolerance_mm.is_finite()
            || self.chord_tolerance_mm <= 0.
            || self.chord_tolerance_mm > tolerance / 2.
        {
            return Err(
                "chord_tolerance_mm must be positive and at most half the output tolerance_mm"
                    .into(),
            );
        }
        if self.max_segments_per_source == 0
            || self.max_segments_per_source > 1_000_000
            || self.max_total_segments == 0
            || self.max_total_segments > 10_000_000
        {
            return Err("invalid bounded lowering/chord budget".into());
        }
        Ok(())
    }
}
#[derive(Debug, Serialize, Deserialize)]
pub struct Mapping {
    pub event_index: usize,
    pub source_event_sha256: String,
    pub source_motion_id: Option<u64>,
    pub posted_motion_id: Option<u64>,
    pub source_fraction_interval: Option<[f64; 2]>,
    pub line_interval: [usize; 2],
    pub deviation_bound_mm: Option<f64>,
}
#[derive(Debug, Serialize)]
pub struct Progress {
    pub run_id: String,
    pub source_events_started: usize,
    pub source_events_completed: usize,
    pub posted_motions: u64,
    pub pending_source_fraction: Option<f64>,
}
#[derive(Debug, Serialize)]
pub struct Output {
    #[serde(flatten)]
    pub output: Chunk,
    pub mappings: Vec<Mapping>,
    pub progress: Progress,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lowering_policy: Option<Options>,
}
#[derive(Clone)]
pub(crate) struct SourceCursor {
    pub event: Event,
    pub hash: String,
    pub event_index: usize,
    pub prepared: Option<PreparedCurve>,
    pub total: Option<u64>,
    pub emitted: u64,
    pub next: f64,
    pub actual_start: Vec3,
    /// Emit the source curve itself as one interval [0, 1] (native arc).
    pub whole: bool,
}

impl SourceCursor {
    pub fn new(
        event: Event,
        event_index: usize,
        tolerance: f64,
        total: Option<u64>,
        actual_start: Vec3,
    ) -> Result<Self, String> {
        let prepared = if let Event::Motion { motion } = &event {
            Some(motion.curve.prepare(tolerance)?)
        } else {
            None
        };
        let hash = format!(
            "{:x}",
            Sha256::digest(serde_json::to_vec(&event).map_err(|e| e.to_string())?)
        );
        Ok(Self {
            event,
            hash,
            event_index,
            prepared,
            total,
            emitted: 0,
            next: 0.,
            actual_start,
            whole: false,
        })
    }
    pub fn ranges(&self, limit: usize) -> Result<Vec<[f64; 2]>, String> {
        if !(1..=64).contains(&limit) {
            return Err("cursor preview needs 1..64 segments".into());
        }
        let total = self.total.ok_or("cursor has no generated uniform plan")?;
        let n = (total - self.emitted).min(limit as u64);
        Ok((0..n)
            .map(|i| {
                [
                    if i == 0 {
                        self.next
                    } else {
                        (self.emitted + i) as f64 / total as f64
                    },
                    (self.emitted + i + 1) as f64 / total as f64,
                ]
            })
            .collect())
    }
    pub fn event_for(&self, range: [f64; 2], posted_id: u64) -> Result<Event, String> {
        let [a, b] = range;
        if !a.is_finite() || !b.is_finite() || a < 0. || b <= a || b > 1. {
            return Err("invalid source interval".into());
        }
        if let Event::Motion { motion } = &self.event {
            let curve = self.prepared.as_ref().unwrap();
            let mut motion = motion.clone();
            motion.id = posted_id;
            if self.whole {
                if a != 0. || b != 1. {
                    return Err("whole native curve requires interval [0, 1]".into());
                }
                return Ok(Event::Motion { motion });
            }
            let mut end = curve.position(b)?;
            if b == 1.
                && let Some([u, v]) = closed_axes(&motion.curve)
            {
                // A closed source's chain ends where it began: the decoded
                // start, not the source end rounded on its own (which differs
                // whenever an earlier block's words moved the start).
                // `certify` still bounds this endpoint against the source.
                end[u] = self.actual_start[u];
                end[v] = self.actual_start[v];
            }
            motion.curve = Curve::Line {
                start: curve.position(a)?,
                end,
            };
            Ok(Event::Motion { motion })
        } else if a == 0. && b == 1. {
            Ok(self.event.clone())
        } else {
            Err("control source requires whole interval".into())
        }
    }
}
fn circular(curve: &Curve) -> Option<(f64, f64, Plane, Vec3)> {
    if let Curve::Arc {
        start,
        center,
        plane,
        sweep_rad,
        ..
    }
    | Curve::Helix {
        start,
        center,
        plane,
        sweep_rad,
        ..
    } = curve
    {
        let [u, v, _] = plane.axes();
        Some((
            (start[u] - center[u]).hypot(start[v] - center[v]),
            *sweep_rad,
            *plane,
            *center,
        ))
    } else {
        None
    }
}
pub(crate) fn segment_count(curve: &Curve, budget: f64) -> Result<u64, String> {
    if let Some((radius, sweep, _, _)) = circular(curve) {
        let budget = budget.min(radius / 16.);
        let angle = (4. * (budget / (2. * radius)).sqrt().min(1.).asin()).min(FRAC_PI_2);
        let count = (sweep.abs() / angle).ceil().max(1.);
        if !count.is_finite() || count > 1_000_000. {
            return Err("lowering segment work budget exceeded before output".into());
        }
        Ok(count as u64)
    } else {
        Ok(1)
    }
}
/// Independent full-interval bound. Does not trust the generator's segment count,
/// angular-step choice, chord endpoints or claimed bound.
pub fn certify(
    source: &Motion,
    range: [f64; 2],
    decoded: &Motion,
    tolerance: f64,
) -> Result<f64, String> {
    let [a, b] = range;
    if !a.is_finite() || !b.is_finite() || a < 0. || b > 1. || b <= a {
        return Err("invalid source interval".into());
    }
    if source.tool != decoded.tool
        || source.operation != decoded.operation
        || source.kind != decoded.kind
    {
        return Err("lowered motion attribution/kind mismatch".into());
    }
    let Curve::Line { start, end } = &decoded.curve else {
        return Err("lowering certificate requires decoded line".into());
    };
    decoded.curve.prepare(tolerance)?;
    let prepared = source.curve.prepare(tolerance)?;
    let distance = |x: Vec3, y: Vec3| (x[0] - y[0]).hypot(x[1] - y[1]).hypot(x[2] - y[2]);
    let endpoint_error =
        distance(*start, prepared.position(a)?).max(distance(*end, prepared.position(b)?));
    let bounds = prepared.bounds();
    let scale = bounds
        .min
        .iter()
        .chain(bounds.max.iter())
        .chain(start.iter())
        .chain(end.iter())
        .fold(1f64, |v, x| v.max(x.abs()));
    let scale = if let Some((_, _, _, center)) = circular(&source.curve) {
        center.iter().fold(scale, |v, x| v.max(x.abs()))
    } else {
        scale
    };
    let scale = if let Some((radius, sweep, _, _)) = circular(&source.curve) {
        scale.max(radius * (1. + sweep.abs()))
    } else {
        scale
    };
    let numeric = 1024. * f64::EPSILON * scale;
    let (sagitta, radius_limit) =
        if let Some((radius, sweep, plane, center)) = circular(&source.curve) {
            let angle = sweep.abs() * (b - a);
            if angle > FRAC_PI_2 + 64. * f64::EPSILON {
                return Err("source interval exceeds certified angular span".into());
            }
            let [u, v, _] = plane.axes();
            let original_start = source.curve.start();
            let mid_angle = (original_start[v] - center[v]).atan2(original_start[u] - center[u])
                + sweep * (a + b) / 2.;
            let forward = ((end[u] - start[u]) * (-mid_angle.sin())
                + (end[v] - start[v]) * mid_angle.cos())
                * sweep.signum();
            if forward <= numeric {
                return Err("lowered circular progress reversed/collapsed".into());
            }
            (2. * radius * (angle / 4.).sin().powi(2), radius / 4.)
        } else {
            (0., f64::INFINITY)
        };
    let bound = sagitta + endpoint_error + post::endpoint_gap(&source.curve) + numeric;
    if bound > radius_limit {
        return Err("lowered circular path cannot preserve radius-relative winding bound".into());
    }
    Ok(bound)
}
/// The in-plane axes of an arc or helix whose source start and end coincide
/// in its plane (a full turn), or None.
pub(crate) fn closed_axes(curve: &Curve) -> Option<[usize; 2]> {
    let (_, _, plane, _) = circular(curve)?;
    let [u, v, _] = plane.axes();
    let (a, b) = (curve.start(), curve.end());
    (a[u] == b[u] && a[v] == b[v]).then_some([u, v])
}
pub(crate) fn check_closure(curve: &Curve, start: Vec3, end: Vec3) -> Result<(), String> {
    if let Some([u, v]) = closed_axes(curve)
        && (start[u] != end[u] || start[v] != end[v])
    {
        return Err(format!(
            "lowering cannot preserve declared planar closure at controller precision: \
             closed source starts at {start:?} but its output ends at {end:?}"
        ));
    }
    Ok(())
}
