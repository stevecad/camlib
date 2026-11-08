//! XY scan patterns for drop-cutter surface finishing: spiral, radial and
//! curves projected onto the surface (the usual Spiral, Radial and Project
//! strategies of commercial CAM). Each pattern yields XY polylines; the
//! surface finishing plan lifts every vertex onto the protected surface and
//! certifies every chord between them (path error and continuous clearance).
use crate::trace::{TracePath, TraceSegment};
use serde::{Deserialize, Serialize};
use std::f64::consts::{PI, TAU};

/// How the cutter covers the surface.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum FinishPattern {
    /// Straight rows along `axis`, `stepover_mm` apart, entered from outside
    /// the stock.
    #[default]
    Raster,
    /// One Archimedean spiral about `center_mm` with ring pitch `stepover_mm`.
    Spiral {
        center_mm: [f64; 2],
        #[serde(default)]
        inner_radius_mm: f64,
        /// Default: reach every point of the stock (plus the cutter radius).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        outer_radius_mm: Option<f64>,
        /// Start at the inner radius and work outward (default inward).
        #[serde(default)]
        outward: bool,
        /// Clockwise seen from +Z (default counter-clockwise).
        #[serde(default)]
        clockwise: bool,
    },
    /// Straight spokes from `center_mm`, `angular_step_deg` apart.
    Radial {
        center_mm: [f64; 2],
        angular_step_deg: f64,
        #[serde(default)]
        inner_radius_mm: f64,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        outer_radius_mm: Option<f64>,
        /// Angle of the first spoke, counter-clockwise from +X.
        #[serde(default)]
        start_angle_deg: f64,
        /// Alternate spoke direction (outward, inward, ...) to shorten links;
        /// otherwise every spoke runs outward.
        #[serde(default = "yes")]
        alternate: bool,
    },
    /// Given XY curves, followed on the surface (tool tip on the offset
    /// surface; the curves are not offset by the tool radius).
    Project { paths: Vec<TracePath> },
    /// Passes morphed between two XY guide curves (both followed from their
    /// start), at most `stepover_mm` apart: blends between two edges or rails.
    Morph { from: TracePath, to: TracePath },
    /// Concave creases where the ball touches two faces at once (inside
    /// corners, valleys): contact normals across a crease differ by at least
    /// `min_angle_deg`.
    /// Constant stepover: offsets of the stock outline (grown by the tool
    /// radius) measured along the surface, `stepover_mm` apart.
    Scallop,
    /// Constant stepover measured along the part's surface from a seed:
    /// iso-lines of the geodesic distance, `stepover_mm` apart, the spacing
    /// measured and the scallop reported (#12).
    Geodesic {
        seed: GeodesicSeed,
        /// Tool-centre mode (#119): the stepover measured along the surface
        /// the ball's centre moves on, not the part's surface.
        #[serde(default, skip_serializing_if = "std::ops::Not::not")]
        tool_centre: bool,
    },
    Pencil {
        #[serde(default = "pencil_angle")]
        min_angle_deg: f64,
        /// Detection grid spacing. Default: a quarter of the tool radius.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        sample_mm: Option<f64>,
    },
}
/// Where geodesic passes start: the distance along the surface is measured
/// from here.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum GeodesicSeed {
    /// The region's boundary (the stock outline grown by the tool radius):
    /// passes work inward.
    Boundary,
    /// A point (XY): rings around it, the first half a stepover out.
    Point { xy_mm: [f64; 2] },
    /// A curve (XY polyline), itself the first pass: passes parallel to it
    /// along the surface.
    Curve { points_mm: Vec<[f64; 2]> },
}
fn pencil_angle() -> f64 {
    20.
}
fn yes() -> bool {
    true
}
impl FinishPattern {
    pub fn is_raster(&self) -> bool {
        matches!(self, Self::Raster)
    }
    pub fn name(&self) -> &'static str {
        match self {
            Self::Raster => "parallel",
            Self::Spiral { .. } => "spiral",
            Self::Radial { .. } => "radial",
            Self::Project { .. } => "project",
            Self::Morph { .. } => "morph",
            Self::Pencil { .. } => "pencil",
            Self::Scallop => "scallop",
            Self::Geodesic { .. } => "geodesic",
        }
    }
}

/// Angle step whose chord stays within `sagitta` of a circle of `radius`.
fn arc_step(radius: f64, sagitta: f64) -> f64 {
    if radius <= sagitta {
        PI / 8.
    } else {
        (2. * (1. - sagitta / radius).acos()).clamp(1e-6, PI / 8.)
    }
}

fn check_points(count: f64, what: &str) -> Result<(), String> {
    if !count.is_finite() || count > u32::MAX as f64 {
        return Err(format!("{what} needs too many vertices ({count:.0})"));
    }
    crate::resources::check_memory(count as u128 * 16, what)
}

/// Default outer radius: the farthest corner of the region from the center.
pub(crate) fn reach(center: [f64; 2], lo: [f64; 2], hi: [f64; 2]) -> f64 {
    [lo[0], hi[0]]
        .iter()
        .flat_map(|x| [lo[1], hi[1]].map(|y| (x - center[0]).hypot(y - center[1])))
        .fold(0., f64::max)
}

pub(crate) fn spiral(
    center: [f64; 2],
    inner: f64,
    outer: f64,
    pitch: f64,
    outward: bool,
    clockwise: bool,
    sagitta: f64,
) -> Result<Vec<[f64; 2]>, String> {
    let turns = (outer - inner) / pitch;
    // Each turn needs at most 2*pi / arc_step(inner) vertices.
    check_points(
        (turns + 1.) * TAU / arc_step(inner.max(pitch).min(outer), sagitta) + 2.,
        "spiral pattern",
    )?;
    let total = TAU * turns;
    let sign = if clockwise { -1. } else { 1. };
    let point = |theta: f64| {
        let r = inner + pitch * theta / TAU;
        [
            center[0] + r * (sign * theta).cos(),
            center[1] + r * (sign * theta).sin(),
        ]
    };
    let mut points = vec![point(0.)];
    let mut theta = 0.;
    while theta < total {
        let r = inner + pitch * theta / TAU;
        theta = (theta + arc_step(r, sagitta)).min(total);
        points.push(point(theta));
    }
    if !outward {
        points.reverse();
    }
    Ok(points)
}

pub(crate) fn radial(
    center: [f64; 2],
    inner: f64,
    outer: f64,
    step_rad: f64,
    start_rad: f64,
    alternate: bool,
) -> Vec<Vec<[f64; 2]>> {
    let spokes = (TAU / step_rad - 1e-9).ceil().max(1.) as usize;
    let step = TAU / spokes as f64;
    (0..spokes)
        .map(|k| {
            let angle = start_rad + k as f64 * step;
            let (s, c) = angle.sin_cos();
            let a = [center[0] + inner * c, center[1] + inner * s];
            let b = [center[0] + outer * c, center[1] + outer * s];
            if alternate && k % 2 == 1 {
                vec![b, a]
            } else {
                vec![a, b]
            }
        })
        .collect()
}

/// Flatten one path to XY vertices; arcs within `sagitta`.
pub(crate) fn flatten(
    path: &TracePath,
    index: usize,
    sagitta: f64,
    guard: f64,
) -> Result<Vec<[f64; 2]>, String> {
    let mut points = vec![path.start_mm];
    let mut at = path.start_mm;
    if !at.iter().all(|v| v.is_finite()) {
        return Err(format!("path {index}: start must be finite"));
    }
    if path.segments.is_empty() {
        return Err(format!("path {index} needs at least one segment"));
    }
    for (i, segment) in path.segments.iter().enumerate() {
        match *segment {
            TraceSegment::Line { end_mm } => {
                if !end_mm.iter().all(|v| v.is_finite())
                    || (end_mm[0] - at[0]).hypot(end_mm[1] - at[1]) <= guard
                {
                    return Err(format!(
                        "path {index}, segment {i}: line needs a finite end away from its start"
                    ));
                }
                points.push(end_mm);
                at = end_mm;
            }
            TraceSegment::Arc {
                center_mm,
                sweep_rad,
            } => {
                let radius = (at[0] - center_mm[0]).hypot(at[1] - center_mm[1]);
                if !center_mm.iter().all(|v| v.is_finite())
                    || !sweep_rad.is_finite()
                    || sweep_rad == 0.
                    || sweep_rad.abs() > TAU
                    || radius <= guard
                {
                    return Err(format!(
                        "path {index}, segment {i}: arc needs a finite center away from its start and a sweep within (0, 2*pi] radians"
                    ));
                }
                let count = (sweep_rad.abs() / arc_step(radius, sagitta)).ceil();
                check_points(count, "projected arc")?;
                let start = (at[1] - center_mm[1]).atan2(at[0] - center_mm[0]);
                for k in 1..=count as usize {
                    let angle = start + sweep_rad * k as f64 / count;
                    points.push([
                        center_mm[0] + radius * angle.cos(),
                        center_mm[1] + radius * angle.sin(),
                    ]);
                }
                at = *points.last().unwrap();
            }
        }
    }
    Ok(points)
}

/// Clip a polyline to the rectangle `lo..hi`; each visible run is one pass.
/// Passes between two polylines: both resampled at the same arc-length
/// fractions, then interpolated so that neighbouring passes are at most
/// `stepover` apart everywhere; alternate passes are reversed (zigzag).
pub(crate) fn morph(
    a: &[[f64; 2]],
    b: &[[f64; 2]],
    stepover: f64,
    spacing: f64,
) -> Vec<Vec<[f64; 2]>> {
    let length = |p: &[[f64; 2]]| {
        p.windows(2)
            .map(|w| (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]))
            .sum::<f64>()
    };
    let n = ((length(a).max(length(b)) / spacing).ceil() as usize).max(1);
    let resample = |p: &[[f64; 2]]| -> Vec<[f64; 2]> {
        let total = length(p);
        let mut out = Vec::with_capacity(n + 1);
        let (mut i, mut acc) = (0, 0.);
        for k in 0..=n {
            let target = total * k as f64 / n as f64;
            while i + 2 < p.len() {
                let seg = (p[i + 1][0] - p[i][0]).hypot(p[i + 1][1] - p[i][1]);
                if acc + seg >= target {
                    break;
                }
                acc += seg;
                i += 1;
            }
            let seg = (p[i + 1][0] - p[i][0]).hypot(p[i + 1][1] - p[i][1]);
            let t = if seg > 0. {
                ((target - acc) / seg).clamp(0., 1.)
            } else {
                0.
            };
            out.push([
                p[i][0] + t * (p[i + 1][0] - p[i][0]),
                p[i][1] + t * (p[i + 1][1] - p[i][1]),
            ]);
        }
        out
    };
    let (ra, rb) = (resample(a), resample(b));
    let widest = ra
        .iter()
        .zip(&rb)
        .map(|(p, q)| (q[0] - p[0]).hypot(q[1] - p[1]))
        .fold(0., f64::max);
    let passes = ((widest / stepover).ceil() as usize).max(1);
    (0..=passes)
        .map(|k| {
            let s = k as f64 / passes as f64;
            let mut pass: Vec<[f64; 2]> = ra
                .iter()
                .zip(&rb)
                .map(|(p, q)| [p[0] + s * (q[0] - p[0]), p[1] + s * (q[1] - p[1])])
                .collect();
            if k % 2 == 1 {
                pass.reverse();
            }
            pass
        })
        .collect()
}
pub(crate) fn clip(points: &[[f64; 2]], lo: [f64; 2], hi: [f64; 2]) -> Vec<Vec<[f64; 2]>> {
    let mut passes = vec![];
    let mut current: Vec<[f64; 2]> = vec![];
    let inside = |p: [f64; 2]| (0..2).all(|k| p[k] >= lo[k] && p[k] <= hi[k]);
    let close = |current: &mut Vec<[f64; 2]>, passes: &mut Vec<Vec<[f64; 2]>>| {
        if current.len() >= 2 {
            passes.push(std::mem::take(current));
        } else {
            current.clear();
        }
    };
    let push = |current: &mut Vec<[f64; 2]>, p: [f64; 2]| {
        if current.last() != Some(&p) {
            current.push(p);
        }
    };
    if points.len() == 1 && inside(points[0]) {
        return vec![];
    }
    for w in points.windows(2) {
        let (p, q) = (w[0], w[1]);
        let d = [q[0] - p[0], q[1] - p[1]];
        // Liang-Barsky.
        let (mut t0, mut t1) = (0f64, 1f64);
        let mut visible = true;
        for k in 0..2 {
            for (num, den) in [(p[k] - lo[k], -d[k]), (hi[k] - p[k], d[k])] {
                if den == 0. {
                    if num < 0. {
                        visible = false;
                    }
                } else {
                    let t = num / den;
                    if den < 0. {
                        t0 = t0.max(t);
                    } else {
                        t1 = t1.min(t);
                    }
                }
            }
        }
        if !visible || t0 > t1 {
            close(&mut current, &mut passes);
            continue;
        }
        let at = |t: f64| {
            if t == 0. {
                p
            } else if t == 1. {
                q
            } else {
                [p[0] + t * d[0], p[1] + t * d[1]]
            }
        };
        if t0 > 0. {
            close(&mut current, &mut passes);
        }
        push(&mut current, at(t0));
        push(&mut current, at(t1));
        if t1 < 1. {
            close(&mut current, &mut passes);
        }
    }
    close(&mut current, &mut passes);
    passes
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spiral_pitch_and_chords() {
        let s = 0.001;
        let points = spiral([0., 0.], 0., 10., 1., true, false, s).unwrap();
        assert_eq!(points[0], [0., 0.]);
        let last = points.last().unwrap();
        assert!((last[0].hypot(last[1]) - 10.).abs() < 1e-9);
        // Radius grows by exactly the pitch per turn and never decreases.
        let mut r = 0.;
        for p in &points {
            let q = p[0].hypot(p[1]);
            assert!(q >= r - 1e-12);
            r = q;
        }
        let inward = spiral([0., 0.], 0., 10., 1., false, true, s).unwrap();
        assert_eq!(inward.last().unwrap(), &[0., 0.]);
    }

    #[test]
    fn clipping_splits_runs() {
        let passes = clip(
            &[[-5., 0.], [5., 0.], [5., 10.], [0., 10.], [0., 20.]],
            [-1., -1.],
            [6., 11.],
        );
        assert_eq!(
            passes,
            vec![vec![[-1., 0.], [5., 0.], [5., 10.], [0., 10.], [0., 11.]]]
        );
        let passes = clip(
            &[[0., 0.], [10., 0.], [10., 2.], [0., 2.]],
            [2., -1.],
            [8., 3.],
        );
        assert_eq!(
            passes,
            vec![vec![[2., 0.], [8., 0.]], vec![[8., 2.], [2., 2.]]]
        );
    }

    #[test]
    fn radial_spokes_alternate() {
        let spokes = radial([0., 0.], 1., 5., PI / 2., 0., true);
        assert_eq!(spokes.len(), 4);
        assert_eq!(spokes[0], vec![[1., 0.], [5., 0.]]);
        assert!((spokes[1][0][1] - 5.).abs() < 1e-12 && (spokes[1][1][1] - 1.).abs() < 1e-12);
    }
}
