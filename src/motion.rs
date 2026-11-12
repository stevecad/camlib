//! Controller-independent tip motion in mm, feed in mm/min, angles in radians.
//! These contracts validate geometry/state, not collision-free machining.
use crate::tool::{ToolAssembly, valid_id};
use serde::{Deserialize, Serialize};
use std::collections::HashSet;
use std::f64::consts::{FRAC_PI_2, TAU};

pub type Vec3 = [f64; 3];
fn point_ok(p: Vec3) -> bool {
    p.iter().all(|v| v.is_finite() && v.abs() <= 1e6)
}
fn distance(a: Vec3, b: Vec3) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1]).hypot(a[2] - b[2])
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Plane {
    Xy,
    Xz,
    Yz,
}
impl Plane {
    /// Right-handed (u,v,w) axes. XZ is traversed Z→X about +Y.
    pub fn axes(self) -> [usize; 3] {
        match self {
            Self::Xy => [0, 1, 2],
            Self::Xz => [2, 0, 1],
            Self::Yz => [1, 2, 0],
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Curve {
    Line {
        start: Vec3,
        end: Vec3,
    },
    Arc {
        start: Vec3,
        end: Vec3,
        center: Vec3,
        plane: Plane,
        sweep_rad: f64,
    },
    Helix {
        start: Vec3,
        end: Vec3,
        center: Vec3,
        plane: Plane,
        sweep_rad: f64,
    },
}

#[derive(Clone, Copy, Debug, Serialize)]
pub struct Bounds3 {
    pub min: Vec3,
    pub max: Vec3,
}
impl Bounds3 {
    fn endpoints(a: Vec3, b: Vec3) -> Self {
        Self {
            min: std::array::from_fn(|i| a[i].min(b[i])),
            max: std::array::from_fn(|i| a[i].max(b[i])),
        }
    }
}

#[derive(Clone)]
pub struct PreparedCurve {
    curve: Curve,
    radius: f64,
    angle: f64,
    length_mm: f64,
    bounds: Bounds3,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NormalAccelerationState {
    Within,
    Boundary,
    Exceeded,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AxisFeedState {
    Within,
    Boundary,
    Exceeded,
}

impl Curve {
    pub fn start(&self) -> Vec3 {
        match *self {
            Self::Line { start, .. } | Self::Arc { start, .. } | Self::Helix { start, .. } => start,
        }
    }
    pub fn end(&self) -> Vec3 {
        match *self {
            Self::Line { end, .. } | Self::Arc { end, .. } | Self::Helix { end, .. } => end,
        }
    }
    pub fn prepare(&self, tolerance_mm: f64) -> Result<PreparedCurve, String> {
        if !tolerance_mm.is_finite() || !(1e-9..=0.1).contains(&tolerance_mm) {
            return Err("motion tolerance must be finite, in [1e-9, 0.1] mm".into());
        }
        let (start, end) = (self.start(), self.end());
        if !point_ok(start) || !point_ok(end) {
            return Err("motion endpoints must be finite, within ±1e6 mm".into());
        }
        let mut bounds = Bounds3::endpoints(start, end);
        let (radius, angle, length_mm) = match *self {
            Self::Line { .. } => {
                let length = distance(start, end);
                if length < 1e-9 {
                    return Err("line must have nonzero length >=1e-9 mm".into());
                }
                (0., 0., length)
            }
            Self::Arc {
                center,
                plane,
                sweep_rad,
                ..
            }
            | Self::Helix {
                center,
                plane,
                sweep_rad,
                ..
            } => {
                if !point_ok(center)
                    || !sweep_rad.is_finite()
                    || !(1e-12..=1000. * TAU).contains(&sweep_rad.abs())
                {
                    return Err("circular motion needs finite center and nonzero signed sweep, at most 1000 turns".into());
                }
                let [u, v, w] = plane.axes();
                if center[w] != start[w] {
                    return Err("circle center must lie in the start plane".into());
                }
                let axial = end[w] - start[w];
                if matches!(self, Self::Arc { .. }) && axial != 0. {
                    return Err(
                        "arc endpoints must share the plane height; use helix for axial motion"
                            .into(),
                    );
                }
                if matches!(self, Self::Helix { .. }) && axial == 0. {
                    return Err("helix must have nonzero axial displacement; use arc".into());
                }
                let radius = (start[u] - center[u]).hypot(start[v] - center[v]);
                if radius < 1e-6 {
                    return Err("circular motion radius must be at least 1e-6 mm".into());
                }
                let angle = (start[v] - center[v]).atan2(start[u] - center[u]);
                let final_angle = angle + sweep_rad;
                let predicted = [
                    center[u] + radius * final_angle.cos(),
                    center[v] + radius * final_angle.sin(),
                ];
                if (predicted[0] - end[u]).hypot(predicted[1] - end[v]) > tolerance_mm {
                    return Err("circular endpoint disagrees with center, radius, direction or sweep beyond tolerance".into());
                }
                // A tolerated endpoint gap must not hide the nominal path's
                // terminal extremum. The axial coordinate is already bounded
                // by start/end, for both planar arcs and helices.
                for (axis, coordinate) in [(u, predicted[0]), (v, predicted[1])] {
                    bounds.min[axis] = bounds.min[axis].min(coordinate);
                    bounds.max[axis] = bounds.max[axis].max(coordinate);
                }
                // Include all interior in-plane extrema swept by the curve.
                for quadrant in 0..4 {
                    let target = quadrant as f64 * FRAC_PI_2;
                    let travel = if sweep_rad > 0. {
                        (target - angle).rem_euclid(TAU)
                    } else {
                        (angle - target).rem_euclid(TAU)
                    };
                    if travel <= sweep_rad.abs() {
                        let axis = if quadrant % 2 == 0 { u } else { v };
                        let coordinate = center[axis] + if quadrant < 2 { radius } else { -radius };
                        bounds.min[axis] = bounds.min[axis].min(coordinate);
                        bounds.max[axis] = bounds.max[axis].max(coordinate);
                    }
                }
                if !point_ok(bounds.min) || !point_ok(bounds.max) {
                    return Err("circular motion leaves supported coordinate range".into());
                }
                (radius, angle, (radius * sweep_rad).hypot(axial))
            }
        };
        Ok(PreparedCurve {
            curve: self.clone(),
            radius,
            angle,
            length_mm,
            bounds,
        })
    }
}

impl PreparedCurve {
    pub fn curve(&self) -> &Curve {
        &self.curve
    }
    pub fn length_mm(&self) -> f64 {
        self.length_mm
    }
    /// Normal acceleration of the NOMINAL analytic segment at declared feed.
    /// Constant on lines/circular helices; excludes junctions, tangential
    /// acceleration, blending, endpoint correction and controller time laws.
    pub fn nominal_normal_acceleration(&self, feed_mm_min: f64) -> Result<f64, String> {
        if !feed_mm_min.is_finite() || !(1e-6..=1e7).contains(&feed_mm_min) {
            return Err("normal acceleration needs a finite declared feed in mm/min".into());
        }
        let sweep = match self.curve {
            Curve::Line { .. } => return Ok(0.),
            Curve::Arc { sweep_rad, .. } | Curve::Helix { sweep_rad, .. } => sweep_rad.abs(),
        };
        // |dT/ds| = R*(dtheta/ds)^2. For a helix this is
        // R/(R^2+pitch_per_radian^2); using prepared length avoids squaring
        // a potentially huge pitch for very short angular sweeps.
        Ok(self.radius * (feed_mm_min / 60. * sweep / self.length_mm).powi(2))
    }
    /// Shared guarded comparison for nominal curve limits in verification/posts.
    pub fn nominal_normal_acceleration_limit(
        &self,
        feed_mm_min: f64,
        limit_mm_s2: f64,
    ) -> Result<(f64, NormalAccelerationState), String> {
        if !limit_mm_s2.is_finite() || !(1e-6..=1e9).contains(&limit_mm_s2) {
            return Err("invalid nominal normal acceleration limit".into());
        }
        let acceleration = self.nominal_normal_acceleration(feed_mm_min)?;
        let guard = 4096. * f64::EPSILON * (1. + acceleration + limit_mm_s2);
        let state = if acceleration > limit_mm_s2 + guard {
            NormalAccelerationState::Exceeded
        } else if acceleration >= limit_mm_s2 - guard {
            NormalAccelerationState::Boundary
        } else {
            NormalAccelerationState::Within
        };
        Ok((acceleration, state))
    }
    /// Peak absolute machine-axis velocity over the entire NOMINAL analytic
    /// segment at declared feed. Translation has no velocity contribution.
    /// No junction, acceleration, blending or controller time-law model.
    pub fn nominal_axis_feed(&self, feed_mm_min: f64, frame: &WorkFrame) -> Result<Vec3, String> {
        if !feed_mm_min.is_finite() || !(1e-6..=1e7).contains(&feed_mm_min) {
            return Err("axis feed needs a finite declared feed in mm/min".into());
        }
        frame.validate()?;
        let columns = [frame.x, frame.y, frame.z];
        let (start, end) = (self.curve.start(), self.curve.end());
        let (plane, sweep) = match self.curve {
            Curve::Line { .. } => {
                return Ok(std::array::from_fn(|i| {
                    let component: f64 = (0..3)
                        .map(|j| (end[j] - start[j]) / self.length_mm * columns[j][i])
                        .sum();
                    feed_mm_min * component.abs()
                }));
            }
            Curve::Arc {
                plane, sweep_rad, ..
            }
            | Curve::Helix {
                plane, sweep_rad, ..
            } => (plane, sweep_rad),
        };
        let [u, v, w] = plane.axes();
        // Normalize before multiplying by feed; avoids huge pitch/radian for
        // tiny sweeps. Global sweep sign cancels under the final absolute value.
        let radial = feed_mm_min * (self.radius * sweep.abs() / self.length_mm);
        let axial = feed_mm_min * ((end[w] - start[w]) / self.length_mm) * sweep.signum();
        let angle_guard = 64. * f64::EPSILON * (1. + self.angle.abs() + sweep.abs());
        Ok(std::array::from_fn(|i| {
            let (a, b, c) = (
                -radial * columns[u][i],
                radial * columns[v][i],
                axial * columns[w][i],
            );
            let eval = |theta: f64| {
                let (sin, cos) = theta.sin_cos();
                (a * sin + b * cos + c).abs()
            };
            let mut maximum = eval(self.angle).max(eval(self.angle + sweep));
            // f'=a*cos(theta)-b*sin(theta); two stationary phases per turn.
            let phase = a.atan2(b);
            for target in [phase, phase + std::f64::consts::PI] {
                let travel = if sweep > 0. {
                    (target - self.angle).rem_euclid(TAU)
                } else {
                    (self.angle - target).rem_euclid(TAU)
                };
                if sweep.abs() + angle_guard >= TAU || travel <= sweep.abs() + angle_guard {
                    maximum = maximum.max(eval(target));
                }
            }
            maximum
        }))
    }
    pub fn nominal_axis_feed_limit(
        &self,
        feed_mm_min: f64,
        frame: &WorkFrame,
        limits: Vec3,
    ) -> Result<[(f64, AxisFeedState); 3], String> {
        if limits
            .iter()
            .any(|v| !v.is_finite() || !(1e-6..=1e7).contains(v))
        {
            return Err("invalid nominal axis feed limits".into());
        }
        let peak = self.nominal_axis_feed(feed_mm_min, frame)?;
        Ok(std::array::from_fn(|i| {
            let guard = 4096. * f64::EPSILON * (1. + peak[i] + limits[i]);
            let state = if peak[i] > limits[i] + guard {
                AxisFeedState::Exceeded
            } else if peak[i] >= limits[i] - guard {
                AxisFeedState::Boundary
            } else {
                AxisFeedState::Within
            };
            (peak[i], state)
        }))
    }
    /// Where the nominal circular move ends: its centre, radius and sweep
    /// place it within the preparation tolerance of the declared end, which
    /// `position(1.)` returns. A line ends at its declared end.
    pub(crate) fn nominal_end(&self) -> Vec3 {
        let end = self.curve.end();
        match self.curve {
            Curve::Line { .. } => end,
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
                let [u, v, _] = plane.axes();
                let (sin, cos) = (self.angle + sweep_rad).sin_cos();
                let mut p = end;
                p[u] = center[u] + self.radius * cos;
                p[v] = center[v] + self.radius * sin;
                p
            }
        }
    }
    /// Fractions in [0, 1] where the nominal curve's coordinate along
    /// `direction` is stationary: with the two ends, every place that
    /// coordinate can be extreme. `direction` is any vector (a row of a
    /// work-to-machine rotation gives a machine axis).
    ///
    /// A line has none. On an arc or helix of radius r, with in-plane axes
    /// u, v and axial axis w, the coordinate at fraction t is
    /// `K + a cos(psi) + d t`, where `psi = angle - phase + t sweep`,
    /// `a = r hypot(direction[u], direction[v])`,
    /// `phase = atan2(direction[v], direction[u])` and
    /// `d = direction[w] (end[w] - start[w])`. Its derivative vanishes where
    /// `sin(psi) = d / (a sweep)`: two angles a turn, or none when the axial
    /// drift outruns the circle. A helix repeats them every turn with a
    /// constant drift, so its extremes are among the first and the last
    /// occurrence of each.
    pub(crate) fn stationary_fractions(&self, direction: Vec3) -> Vec<f64> {
        let (Curve::Arc {
            plane, sweep_rad, ..
        }
        | Curve::Helix {
            plane, sweep_rad, ..
        }) = self.curve
        else {
            return Vec::new();
        };
        let [u, v, w] = plane.axes();
        let slope = self.radius * direction[u].hypot(direction[v]) * sweep_rad;
        let drift = direction[w] * (self.curve.end()[w] - self.curve.start()[w]);
        let sine = drift / slope;
        // No in-plane part (slope 0), or a monotone coordinate. A ratio one
        // rounding step past 1 is the two angles merged into one: keep it.
        if sine.is_nan() || sine.abs() > 1. + 1e-12 {
            return Vec::new();
        }
        let root = sine.clamp(-1., 1.).asin();
        let start = self.angle - direction[v].atan2(direction[u]);
        let span = sweep_rad.abs();
        let mut fractions = Vec::new();
        for target in [root, std::f64::consts::PI - root] {
            let travel = if sweep_rad > 0. {
                (target - start).rem_euclid(TAU)
            } else {
                (start - target).rem_euclid(TAU)
            };
            if travel <= span {
                fractions.push(travel / span);
                let last = travel + TAU * ((span - travel) / TAU).floor();
                if last > travel {
                    fractions.push((last / span).min(1.));
                }
            }
        }
        fractions
    }
    /// Bounds include declared endpoints; callers must add construction/error
    /// guards when using them for collision pruning.
    pub fn bounds(&self) -> Bounds3 {
        self.bounds
    }
    pub fn position(&self, fraction: f64) -> Result<Vec3, String> {
        if !fraction.is_finite() || !(0.0..=1.0).contains(&fraction) {
            return Err("motion fraction must be finite, in [0,1]".into());
        }
        let (start, end) = (self.curve.start(), self.curve.end());
        if fraction == 0. {
            return Ok(start);
        }
        if fraction == 1. {
            return Ok(end);
        }
        match self.curve {
            Curve::Line { .. } => Ok(std::array::from_fn(|i| {
                start[i] + fraction * (end[i] - start[i])
            })),
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
                let (sin, cos) = (self.angle + fraction * sweep_rad).sin_cos();
                let mut p = start;
                p[u] = center[u] + self.radius * cos;
                p[v] = center[v] + self.radius * sin;
                p[w] = start[w] + fraction * (end[w] - start[w]);
                Ok(p)
            }
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MoveKind {
    /// Whole-assembly clearance intent. An optional programmed feed selects a
    /// feed-controlled line traverse; None selects unprogrammed rapid speed.
    Rapid,
    Cut,
    Plunge,
    Lead,
    Link,
    Retract,
    /// Spindle-synchronized tapping (feed = pitch x rpm; a clockwise spindle
    /// taps a right-hand thread, counterclockwise a left-hand one):
    /// a vertical line down, immediately followed by its exact return. A
    /// post writes the pair as one controller tapping cycle.
    Tap,
    /// A rapid (or feed-controlled traverse) straight move with the spindle
    /// stopped at an oriented angle
    /// (after `Event::SpindleOrient`): the boring bar does not rotate. Every
    /// pose lies on the ray from the oriented stop position away from the
    /// cutting edge, so the bar stays inside the bore it just cut (fine
    /// boring, G76). Never cuts.
    Shift,
    /// A spindle-synchronized line (lathe threading, LinuxCNC G33, FANUC
    /// G32): feed = lead x rpm. Cuts like a fed move; the post writes the
    /// lead, not a feed.
    Thread,
    /// Protected probe positioning: a straight fed move of the probe that
    /// stops the machine if the stylus triggers (LinuxCNC G38.3 with a
    /// trigger check, Renishaw O9810). Only inside an `Event::Probe`.
    /// Never removes stock; the whole assembly must stay clear of it.
    ProbePosition,
    /// The guarded measuring move up to `search_mm` before the expected
    /// contact (the first part of one LinuxCNC G38.2 or macro gauging
    /// move): no contact is expected here, so the whole assembly must stay
    /// clear of stock and scene bodies. Never removes stock.
    ProbeMeasure,
    /// The last `search_mm` of the measuring move, ending at the expected
    /// contact: the stylus ball (or the measured cutter on a tool setter)
    /// is expected to touch here, so only the other parts are checked.
    /// Never removes stock.
    ProbeTouch,
    /// The fed move back from a contact to where its measuring move
    /// started. The ball starts touching, so only the other parts are
    /// checked. Never removes stock.
    ProbeReturn,
}

impl MoveKind {
    /// Clearance traverse: rapid, or a programmed traverse feed; never cuts.
    pub fn is_rapid(self) -> bool {
        matches!(self, MoveKind::Rapid | MoveKind::Shift)
    }
    /// A probing move (only inside an `Event::Probe`).
    pub fn is_probe(self) -> bool {
        matches!(
            self,
            MoveKind::ProbePosition
                | MoveKind::ProbeMeasure
                | MoveKind::ProbeTouch
                | MoveKind::ProbeReturn
        )
    }
    /// Whether the move may remove material: every fed move except a
    /// probing move. Rapids, oriented shifts and probing moves never cut,
    /// so no removal certificate may be drawn from them.
    pub fn cuts(self) -> bool {
        !self.is_rapid() && !self.is_probe()
    }
    /// A probing move on which the stylus (or the measured cutter) is
    /// expected to be in contact: only the other assembly parts are checked.
    pub fn expects_contact(self) -> bool {
        matches!(self, MoveKind::ProbeTouch | MoveKind::ProbeReturn)
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Motion {
    pub id: u64,
    pub operation: String,
    pub tool: String,
    pub kind: MoveKind,
    pub curve: Curve,
    pub feed_mm_min: Option<f64>,
}

impl Motion {
    /// Whether this committed motion is `planned` as the stream committed it:
    /// the same motion, or a planned straight-up retract the stream's retract
    /// policy turned into a rapid (same id, operation, tool and path, so the
    /// same swept volume).
    pub fn commits(&self, planned: &Motion) -> bool {
        self == planned
            || (planned.kind == MoveKind::Retract
                && self.kind == MoveKind::Rapid
                && self.feed_mm_min.is_none()
                && self.id == planned.id
                && self.operation == planned.operation
                && self.tool == planned.tool
                && self.curve == planned.curve)
    }
    /// Check interpolation/feed semantics independently of stock or program state.
    /// A fed clearance traverse keeps Rapid intent and never authorizes cutting.
    pub fn validate_feed(&self, max_feed_mm_min: Option<f64>) -> Result<(), String> {
        if self.kind.is_rapid() && !matches!(self.curve, Curve::Line { .. }) {
            return Err("rapid clearance requires a line".into());
        }
        match self.feed_mm_min {
            None if self.kind.is_rapid() => Ok(()),
            Some(feed)
                if feed.is_finite()
                    && feed > 0.
                    && max_feed_mm_min.is_none_or(|max| feed <= max) =>
            {
                Ok(())
            }
            _ => Err("programmed feed must be positive, finite and within machine limits".into()),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SpindleDirection {
    Cw,
    Ccw,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Coolant {
    Off,
    Flood,
    Mist,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Event {
    Motion {
        motion: Motion,
    },
    /// An explicit tip-reference reset, not a connecting cut/rapid. A post must
    /// separately qualify its machine/tool-change route and offset convention.
    ToolChange {
        tool: String,
        number: u32,
        length_register: u32,
        position_after_mm: Vec3,
    },
    Spindle {
        rpm: f64,
        direction: SpindleDirection,
    },
    Coolant {
        mode: Coolant,
    },
    WorkOffset {
        number: u8,
    },
    Stop {
        optional: bool,
    },
    /// Pause in place (G4), e.g. at the bottom of a hole.
    Dwell {
        seconds: f64,
    },
    /// The next `covers` events (motions and a dwell) drill one hole exactly
    /// as the controller's canned cycle would: a post may write them as one
    /// cycle block (G81/G82/G83/G73 with G98) or as the explicit moves. The
    /// moves themselves stay authoritative; a posted cycle is decoded back
    /// into moves and checked against them.
    Cycle {
        cycle: DrillCycle,
        center_mm: [f64; 2],
        /// Final tip depth (Z word).
        z_mm: f64,
        /// R plane.
        r_mm: f64,
        /// Return level (G98: the Z where the cycle starts).
        initial_z_mm: f64,
        feed_mm_min: f64,
        /// Q: peck depth (peck and chip-break cycles).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        peck_mm: Option<f64>,
        /// P: dwell at the bottom (dwell cycle).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        dwell_s: Option<f64>,
        /// Peck cycle: rapid back down to this far above the last peck.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        reentry_mm: Option<f64>,
        /// Chip-break cycle: rapid up this far after each peck.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        lift_mm: Option<f64>,
        /// Fine boring: Q, the shift away from the wall at the bottom.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        shift_mm: Option<f64>,
        covers: u32,
    },
    /// Stop the spindle at its oriented angle (M19), the boring bar's
    /// cutting edge pointing along `edge_angle_deg` (work XY, degrees from
    /// +X; a setup premise the post declares). Only `MoveKind::Shift` moves
    /// follow until the spindle restarts.
    SpindleOrient {
        edge_angle_deg: f64,
    },
    /// Plasma arc or laser beam on at `power_percent` (laser power; a plasma
    /// torch simply fires), or off at 0. A beam tool cuts only while it is on.
    Beam {
        power_percent: f64,
    },
    /// Torch height control (THC) on or off, while the beam is on.
    HeightControl {
        on: bool,
    },
    /// Spindle on at constant surface speed (lathe G96): at radius r = |x|
    /// it turns 1000 x `m_min` / (2 pi r) rpm, never above `max_rpm` (G50 or
    /// G96 D). Spindle cancels it (G97).
    SurfaceSpeed {
        m_min: f64,
        max_rpm: f64,
        direction: SpindleDirection,
    },
    /// Feed moves that follow run at `mm_rev` per spindle revolution (G95;
    /// 0: per minute again, G94). Each such move's `feed_mm_min` is then
    /// `mm_rev` x the highest spindle speed along it (its bound).
    FeedPerRev {
        mm_rev: f64,
    },
    /// Cutter radius compensation (G41 left / G42 right of the programmed
    /// path in the direction of travel, G40 off). The programmed path is the
    /// tool center: the controller offsets it by the register's wear only.
    /// It turns on and off over the next move, a straight line in `plane`
    /// (XY on a mill; XZ on a lathe, G18, where the offset's tip direction
    /// applies).
    CutterCompensation {
        side: CompensationSide,
        #[serde(default = "xy_plane", skip_serializing_if = "is_xy")]
        plane: Plane,
    },
    /// The next `covers` motions are one probing cycle (`crate::probe`):
    /// protected positioning, guarded measuring moves to the expected
    /// contacts and the returns, exactly as `ProbeCycle::legs` lays them
    /// out. A post writes them as the controller's probing blocks (LinuxCNC
    /// G38.x, or Renishaw Inspection Plus macro calls); the moves stay
    /// authoritative, and the posted blocks are decoded back into them.
    /// What the cycle stores on the controller (`cycle.result`) is a premise
    /// for what follows, recorded here, not something camlib can check.
    Probe {
        cycle: crate::probe::ProbeCycle,
        covers: u32,
    },
}

fn xy_plane() -> Plane {
    Plane::Xy
}
fn is_xy(p: &Plane) -> bool {
    *p == Plane::Xy
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CompensationSide {
    Off,
    Left,
    Right,
}

/// Canned drilling cycle types.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DrillCycle {
    /// G81: feed to depth, rapid out.
    Drill,
    /// G82: feed to depth, dwell, rapid out.
    Dwell,
    /// G83: pecks with a rapid return to R after each.
    Peck,
    /// G73: pecks with a short rapid lift after each (chip break).
    ChipBreak,
    /// G85: feed to depth, feed back out to R.
    Bore,
    /// G89: feed to depth, dwell, feed back out to R.
    BoreDwell,
    /// G86: feed to depth, stop the spindle, rapid out to R, restart it.
    BoreStop,
    /// G76: feed to depth, optional dwell, oriented spindle stop, shift Q
    /// away from the wall, rapid out to R, shift back, restart the spindle.
    FineBore,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WorkFrame {
    pub origin_mm: Vec3,
    pub x: Vec3,
    pub y: Vec3,
    pub z: Vec3,
}
impl WorkFrame {
    pub fn identity() -> Self {
        Self {
            origin_mm: [0.; 3],
            x: [1., 0., 0.],
            y: [0., 1., 0.],
            z: [0., 0., 1.],
        }
    }
    pub fn validate(&self) -> Result<(), String> {
        if !point_ok(self.origin_mm) || ![self.x, self.y, self.z].iter().all(|p| point_ok(*p)) {
            return Err("work frame must be finite and bounded".into());
        }
        let dot = |a: Vec3, b: Vec3| a.iter().zip(b).map(|(x, y)| x * y).sum::<f64>();
        let cross = [
            self.x[1] * self.y[2] - self.x[2] * self.y[1],
            self.x[2] * self.y[0] - self.x[0] * self.y[2],
            self.x[0] * self.y[1] - self.x[1] * self.y[0],
        ];
        if [self.x, self.y, self.z]
            .iter()
            .any(|&v| (dot(v, v) - 1.).abs() > 1e-12)
            || dot(self.x, self.y).abs() > 1e-12
            || distance(cross, self.z) > 1e-12
        {
            return Err("work frame axes must be orthonormal and right-handed".into());
        }
        Ok(())
    }
    pub fn to_design(&self, p: Vec3) -> Vec3 {
        std::array::from_fn(|i| {
            self.origin_mm[i] + p[0] * self.x[i] + p[1] * self.y[i] + p[2] * self.z[i]
        })
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Program {
    pub version: u32,
    pub id: String,
    pub revision: String,
    pub units: String,
    pub tolerance_mm: f64,
    pub frame: WorkFrame,
    pub initial_position_mm: Vec3,
    pub tools: Vec<ToolAssembly>,
    pub events: Vec<Event>,
}

/// An engine/post contract, not a claim of a particular controller's support.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Capabilities {
    pub id: String,
    pub planes: Vec<Plane>,
    pub arcs: bool,
    pub helices: bool,
    pub max_sweep_rad: f64,
    pub max_feed_mm_min: f64,
    pub max_spindle_rpm: f64,
    pub max_tool_number: u32,
    pub max_length_register: u32,
    pub max_work_offset: u8,
    pub tool_changes: bool,
    pub flood: bool,
    pub mist: bool,
}

#[derive(Clone, Debug, Serialize)]
pub struct ProgramSummary {
    pub motions: usize,
    pub events: usize,
    pub tools: usize,
    pub rapid_length_mm: f64,
    pub fed_length_mm: f64,
    pub final_position_mm: Vec3,
}

impl Capabilities {
    pub fn validate(&self) -> Result<(), String> {
        if !valid_id(&self.id)
            || !self.max_feed_mm_min.is_finite()
            || self.max_feed_mm_min <= 0.
            || !self.max_spindle_rpm.is_finite()
            || self.max_spindle_rpm <= 0.
            || !self.max_sweep_rad.is_finite()
            || self.max_sweep_rad <= 0.
            || self.max_tool_number == 0
            || self.max_length_register == 0
            || !(1..=9).contains(&self.max_work_offset)
            || (self.helices && !self.arcs)
            || (self.arcs && self.planes.is_empty())
        {
            return Err("invalid controller/engine capabilities".into());
        }
        Ok(())
    }
}

impl Program {
    pub fn validate(&self, capabilities: Option<&Capabilities>) -> Result<ProgramSummary, String> {
        let mut validator = ProgramValidator::new(self, capabilities)?;
        validator.consume(&self.events)?;
        Ok(validator.summary)
    }
}

/// Incremental native validation with no retained motion history. Batches commit
/// atomically; the same state machine validates complete Program values.
/// The least and greatest x over a curve (exact: lines, and arcs and
/// helices through their extreme angles).
pub fn x_extent(curve: &Curve) -> [f64; 2] {
    let (start, end) = (curve.start(), curve.end());
    let (mut lo, mut hi) = (start[0].min(end[0]), start[0].max(end[0]));
    if let Curve::Arc {
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
    } = *curve
    {
        // Angle of the start and the angle where x is greatest: XY from +X
        // toward +Y; XZ (G18) from +Z toward +X. YZ arcs keep x.
        let (a0, top, radius) = match plane {
            Plane::Xy => (
                (start[1] - center[1]).atan2(start[0] - center[0]),
                0.,
                (start[0] - center[0]).hypot(start[1] - center[1]),
            ),
            Plane::Xz => (
                (start[0] - center[0]).atan2(start[2] - center[2]),
                std::f64::consts::FRAC_PI_2,
                (start[0] - center[0]).hypot(start[2] - center[2]),
            ),
            Plane::Yz => return [lo, hi],
        };
        // Include the nominal endpoint as well as the declared endpoint;
        // their permitted gap can otherwise understate CSS RPM near the axis.
        let a1 = a0 + sweep_rad;
        let nominal_x = center[0]
            + radius
                * if plane == Plane::Xy {
                    a1.cos()
                } else {
                    a1.sin()
                };
        lo = lo.min(nominal_x);
        hi = hi.max(nominal_x);
        let tau = std::f64::consts::TAU;
        let passes = |target: f64| {
            if sweep_rad.abs() >= tau {
                return true;
            }
            let d = if sweep_rad >= 0. {
                (target - a0).rem_euclid(tau)
            } else {
                (a0 - target).rem_euclid(tau)
            };
            d <= sweep_rad.abs()
        };
        if passes(top) {
            hi = hi.max(center[0] + radius);
        }
        if passes(top + std::f64::consts::PI) {
            lo = lo.min(center[0] - radius);
        }
    }
    [lo, hi]
}

/// The highest spindle speed along a move at constant surface speed
/// `m_min` capped at `max_rpm`: at its least radius |x|.
pub fn css_rpm_bound(curve: &Curve, m_min: f64, max_rpm: f64) -> f64 {
    let [lo, hi] = x_extent(curve);
    let r = if lo <= 0. && hi >= 0. {
        0.
    } else {
        lo.abs().min(hi.abs())
    };
    if r <= 0. {
        max_rpm
    } else {
        (1000. * m_min / (std::f64::consts::TAU * r)).min(max_rpm)
    }
}

#[derive(Clone)]
pub struct ProgramValidator {
    tools: HashSet<String>,
    /// Constant surface speed in effect (m/min, max rpm), and the feed per
    /// revolution in effect.
    css: Option<(f64, f64)>,
    feed_rev: Option<f64>,
    capabilities: Option<Capabilities>,
    tolerance_mm: f64,
    position: Vec3,
    active_tool: Option<String>,
    rpm: f64,
    last_id: Option<u64>,
    changes: usize,
    active_offset: Option<u8>,
    clockwise: bool,
    /// A tapping motion down (start, bottom, feed) awaiting its return.
    tap: Option<(Vec3, Vec3, f64)>,
    /// Events still covered by the last canned cycle.
    covered: u32,
    /// Cutter compensation in effect, and whether the next motion must be
    /// the straight XY move that turns it on or off.
    compensation: CompensationSide,
    compensation_move: bool,
    /// The plane compensation works in: XY (mills), or XZ (lathes, G18),
    /// set by the straight move that turns it on.
    compensation_plane: Option<Plane>,
    /// Oriented spindle stop in effect: stop position and edge direction.
    oriented: Option<(Vec3, [f64; 2])>,
    /// Beam tools (plasma torches, laser heads), and beam/THC state.
    beams: HashSet<String>,
    beam_on: bool,
    height_control: bool,
    /// Probe tools, and every tool's cutter radius (a probe's ball).
    probes: HashSet<String>,
    radii: std::collections::HashMap<String, f64>,
    /// The moves the running probing cycle still expects, next first.
    probe_legs: std::collections::VecDeque<crate::probe::Leg>,
    /// Work offsets a probing cycle has corrected, and tools whose full
    /// radius a tool setter stored.
    probed_offsets: Vec<u8>,
    radius_set: HashSet<String>,
    pub summary: ProgramSummary,
}
impl ProgramValidator {
    pub fn new(program: &Program, capabilities: Option<&Capabilities>) -> Result<Self, String> {
        if program.version != 1
            || program.units != "mm"
            || !valid_id(&program.id)
            || program.revision.is_empty()
            || !point_ok(program.initial_position_mm)
            || !program.tolerance_mm.is_finite()
            || !(1e-9..=0.1).contains(&program.tolerance_mm)
        {
            return Err("program needs version 1, mm, stable id, revision, bounded initial position and valid tolerance".into());
        }
        program.frame.validate()?;
        if let Some(c) = capabilities {
            c.validate()?;
            // Initial three-axis profile: manual setup/work coordinates must
            // already align their tool axis with machine +Z.
            if distance(program.frame.z, [0., 0., 1.]) > 1e-12 {
                return Err("three-axis profile does not support tilted tool axis".into());
            }
        }
        let mut tools = HashSet::new();
        for t in &program.tools {
            t.prepare()?;
            if !tools.insert(t.id.clone()) {
                return Err("duplicate tool assembly id".into());
            }
        }
        Ok(Self {
            tools,
            css: None,
            feed_rev: None,
            capabilities: capabilities.cloned(),
            tolerance_mm: program.tolerance_mm,
            position: program.initial_position_mm,
            active_tool: None,
            rpm: 0.,
            last_id: None,
            changes: 0,
            active_offset: None,
            clockwise: true,
            tap: None,
            covered: 0,
            compensation: CompensationSide::Off,
            compensation_move: false,
            compensation_plane: None,
            oriented: None,
            beams: program
                .tools
                .iter()
                .filter(|t| matches!(t.cutter.shape, crate::tool::CutterShape::Beam))
                .map(|t| t.id.clone())
                .collect(),
            beam_on: false,
            height_control: false,
            probes: program
                .tools
                .iter()
                .filter(|t| matches!(t.cutter.shape, crate::tool::CutterShape::Probe))
                .map(|t| t.id.clone())
                .collect(),
            radii: program
                .tools
                .iter()
                .map(|t| (t.id.clone(), t.cutter.diameter_mm / 2.))
                .collect(),
            probe_legs: Default::default(),
            probed_offsets: vec![],
            radius_set: HashSet::new(),
            summary: ProgramSummary {
                motions: 0,
                events: 0,
                tools: program.tools.len(),
                rapid_length_mm: 0.,
                fed_length_mm: 0.,
                final_position_mm: program.initial_position_mm,
            },
        })
    }
    /// The feed per revolution in effect (G95), if any.
    pub fn feed_per_rev(&self) -> Option<f64> {
        self.feed_rev
    }
    pub fn consume(&mut self, events: &[Event]) -> Result<(), String> {
        let mut candidate = self.clone();
        candidate.consume_inner(events)?;
        *self = candidate;
        Ok(())
    }
    fn consume_inner(&mut self, events: &[Event]) -> Result<(), String> {
        let mut position = self.position;
        let mut active_tool = self.active_tool.clone();
        let mut rpm = self.rpm;
        let mut last_id = self.last_id;
        let mut changes = self.changes;
        let mut active_offset = self.active_offset;
        let mut clockwise = self.clockwise;
        let mut tap = self.tap;
        let mut covered = self.covered;
        let mut compensation = self.compensation;
        let mut compensation_move = self.compensation_move;
        let mut compensation_plane = self.compensation_plane;
        let mut oriented = self.oriented;
        let mut beam_on = self.beam_on;
        let mut height_control = self.height_control;
        let mut css = self.css;
        let mut feed_rev = self.feed_rev;
        let mut probe_legs = self.probe_legs.clone();
        let mut probed_offsets = self.probed_offsets.clone();
        let mut radius_set = self.radius_set.clone();
        let beam_tool = |t: &Option<String>| t.as_ref().is_some_and(|t| self.beams.contains(t));
        let tools = &self.tools;
        let capabilities = self.capabilities.as_ref();
        let mut summary = self.summary.clone();
        for (local_index, event) in events.iter().enumerate() {
            let index = self.summary.events + local_index;
            let error = |message: &str| format!("event {index}: {message}");
            if tap.is_some()
                && !matches!(event, Event::Motion { motion } if motion.kind == MoveKind::Tap)
            {
                return Err(error("a tapping motion must be followed by its return"));
            }
            if oriented.is_some()
                && !matches!(
                    event,
                    Event::Spindle { .. } | Event::SurfaceSpeed { .. } | Event::ToolChange { .. }
                )
                && !matches!(event, Event::Motion { motion } if motion.kind == MoveKind::Shift)
            {
                return Err(error(
                    "only oriented-spindle shifts until the spindle restarts after an oriented stop",
                ));
            }
            let mut in_probe = false;
            if covered > 0 {
                if let Some(leg) = probe_legs.pop_front() {
                    // A probing cycle covers exactly the moves it lays out.
                    let Event::Motion { motion } = event else {
                        return Err(error("a probing cycle covers only its own moves"));
                    };
                    if motion.kind != leg.kind
                        || !matches!(motion.curve, Curve::Line { .. })
                        || distance(motion.curve.end(), leg.end) > self.tolerance_mm
                        || motion.feed_mm_min != leg.feed_mm_min
                    {
                        return Err(error(&format!(
                            "a probing cycle's next move is a {:?} line to {:?} at feed {:?}, not this {:?} move to {:?} at {:?}",
                            leg.kind,
                            leg.end,
                            leg.feed_mm_min,
                            motion.kind,
                            motion.curve.end(),
                            motion.feed_mm_min
                        )));
                    }
                    in_probe = true;
                } else if !matches!(
                    event,
                    Event::Dwell { .. } | Event::Spindle { .. } | Event::SpindleOrient { .. }
                ) && !matches!(event, Event::Motion { motion } if motion.kind != MoveKind::Tap && !motion.kind.is_probe())
                {
                    return Err(error(
                        "a canned cycle covers only the motions, dwell and spindle stop/restart of its hole",
                    ));
                }
                covered -= 1;
            }
            if let Event::Motion { motion } = event
                && motion.kind.is_probe()
                && !in_probe
            {
                return Err(error(
                    "probing moves belong to a probing cycle (Event::Probe) that covers them",
                ));
            }
            // While compensation is on or changing: only XY-plane motion.
            if (compensation != CompensationSide::Off || compensation_move)
                && !matches!(
                    event,
                    Event::Motion { .. }
                        | Event::CutterCompensation { .. }
                        | Event::HeightControl { .. }
                )
            {
                return Err(error(
                    "only motion in the compensation plane while cutter compensation is on or changing; turn it off first",
                ));
            }
            match event {
                Event::ToolChange {
                    tool,
                    number,
                    length_register,
                    position_after_mm,
                } => {
                    if !tools.contains(tool.as_str())
                        || *number == 0
                        || *length_register == 0
                        || !point_ok(*position_after_mm)
                    {
                        return Err(error("invalid tool assignment or position"));
                    }
                    if rpm != 0. || beam_on {
                        return Err(error("stop the spindle or beam before a tool change"));
                    }
                    if let Some(c) = capabilities
                        && (*number > c.max_tool_number
                            || *length_register > c.max_length_register
                            || changes > 0 && !c.tool_changes)
                    {
                        return Err(error("unsupported tool change/register"));
                    }
                    active_tool = Some(tool.clone());
                    position = *position_after_mm;
                    changes += 1;
                    oriented = None;
                }
                Event::Spindle {
                    rpm: value,
                    direction,
                } => {
                    clockwise = *direction == crate::motion::SpindleDirection::Cw;
                    if !value.is_finite()
                        || *value < 0.
                        || capabilities.is_some_and(|c| *value > c.max_spindle_rpm)
                    {
                        return Err(error("invalid or unsupported spindle speed"));
                    }
                    if *value > 0. && active_tool.is_none() {
                        return Err(error("assign a tool before starting spindle"));
                    }
                    if *value > 0. && beam_tool(&active_tool) {
                        return Err(error(
                            "a beam tool has no spindle; turn the beam on instead",
                        ));
                    }
                    rpm = *value;
                    css = None;
                    if rpm > 0. {
                        oriented = None;
                    }
                }
                Event::SurfaceSpeed {
                    m_min,
                    max_rpm,
                    direction,
                } => {
                    if !(m_min.is_finite() && *m_min > 0. && *m_min <= 1e5)
                        || !(max_rpm.is_finite() && *max_rpm > 0.)
                        || capabilities.is_some_and(|c| *max_rpm > c.max_spindle_rpm)
                    {
                        return Err(error(
                            "constant surface speed needs a positive speed and a maximum rpm the machine reaches",
                        ));
                    }
                    if active_tool.is_none() || beam_tool(&active_tool) {
                        return Err(error(
                            "constant surface speed needs an active (non-beam) tool",
                        ));
                    }
                    clockwise = *direction == SpindleDirection::Cw;
                    css = Some((*m_min, *max_rpm));
                    rpm = *max_rpm;
                    oriented = None;
                }
                Event::FeedPerRev { mm_rev } => {
                    if !(mm_rev.is_finite() && *mm_rev >= 0. && *mm_rev <= 100.) {
                        return Err(error("feed per revolution must be 0..100 mm"));
                    }
                    feed_rev = (*mm_rev > 0.).then_some(*mm_rev);
                }
                Event::Beam { power_percent } => {
                    if !power_percent.is_finite() || !(0. ..=100.).contains(power_percent) {
                        return Err(error("beam power must be 0..100 percent"));
                    }
                    if !beam_tool(&active_tool) {
                        return Err(error(
                            "the beam needs an active beam tool (torch or laser head)",
                        ));
                    }
                    if *power_percent == 0. && height_control {
                        return Err(error("turn torch height control off before the beam"));
                    }
                    beam_on = *power_percent > 0.;
                }
                Event::HeightControl { on } => {
                    if *on && !beam_on {
                        return Err(error("torch height control needs the beam on"));
                    }
                    height_control = *on;
                }
                Event::SpindleOrient { edge_angle_deg } => {
                    if !edge_angle_deg.is_finite() || edge_angle_deg.abs() > 360. {
                        return Err(error("edge_angle_deg must be finite, within +-360 degrees"));
                    }
                    if active_tool.is_none() || oriented.is_some() {
                        return Err(error(
                            "an oriented spindle stop needs an active tool and a running or stopped spindle",
                        ));
                    }
                    let a = edge_angle_deg.to_radians();
                    oriented = Some((position, [a.cos(), a.sin()]));
                    rpm = 0.;
                }
                Event::Coolant { mode } => {
                    if capabilities.is_some_and(|c| {
                        *mode == Coolant::Mist && !c.mist || *mode == Coolant::Flood && !c.flood
                    }) {
                        return Err(error("coolant mode unsupported"));
                    }
                }
                Event::WorkOffset { number } => {
                    if !(1..=9).contains(number)
                        || capabilities.is_some_and(|c| *number > c.max_work_offset)
                    {
                        return Err(error("work offset unsupported"));
                    }
                    if active_offset.is_some_and(|old| old != *number) {
                        return Err(error(
                            "one work frame per program; use a separate setup/program for another work offset",
                        ));
                    }
                    active_offset = Some(*number);
                }
                Event::Stop { .. } => {}
                Event::Dwell { seconds } => {
                    if !seconds.is_finite() || *seconds <= 0. || *seconds > 1e5 {
                        return Err(error("dwell needs a positive finite time in seconds"));
                    }
                }
                Event::CutterCompensation { side, plane } => {
                    if *plane == Plane::Yz {
                        return Err(error("cutter compensation works in XY or XZ"));
                    }
                    if compensation_move
                        || (*side == CompensationSide::Off)
                            == (compensation == CompensationSide::Off)
                    {
                        return Err(error(
                            "cutter compensation must alternate on/off, each with its own straight XY move",
                        ));
                    }
                    if *side != CompensationSide::Off
                        && active_tool.as_ref().is_some_and(|t| radius_set.contains(t))
                    {
                        return Err(error(
                            "a tool setter stored this tool's full radius in its tool table entry; camlib's compensation register holds the wear only",
                        ));
                    }
                    if *side != CompensationSide::Off {
                        compensation_plane = Some(*plane);
                    } else if compensation_plane != Some(*plane) {
                        return Err(error(
                            "cutter compensation turns off in the plane it was turned on in",
                        ));
                    }
                    compensation = *side;
                    compensation_move = true;
                }
                Event::Cycle {
                    cycle,
                    center_mm,
                    z_mm,
                    r_mm,
                    initial_z_mm,
                    feed_mm_min,
                    peck_mm,
                    dwell_s,
                    reentry_mm,
                    lift_mm,
                    shift_mm,
                    covers,
                } => {
                    let positive =
                        |v: &Option<f64>| v.is_some_and(|x| x.is_finite() && x > 0. && x <= 1e5);
                    let shape = match cycle {
                        DrillCycle::Drill => peck_mm.is_none() && dwell_s.is_none(),
                        DrillCycle::Dwell => peck_mm.is_none() && positive(dwell_s),
                        DrillCycle::Peck => {
                            positive(peck_mm) && dwell_s.is_none() && positive(reentry_mm)
                        }
                        DrillCycle::ChipBreak => {
                            positive(peck_mm) && dwell_s.is_none() && positive(lift_mm)
                        }
                        // Boring cycles start at R (initial level = R), which
                        // every controller runs alike.
                        DrillCycle::Bore | DrillCycle::BoreStop => {
                            peck_mm.is_none() && dwell_s.is_none() && r_mm == initial_z_mm
                        }
                        DrillCycle::BoreDwell => {
                            peck_mm.is_none() && positive(dwell_s) && r_mm == initial_z_mm
                        }
                        DrillCycle::FineBore => {
                            peck_mm.is_none()
                                && (dwell_s.is_none() || positive(dwell_s))
                                && positive(shift_mm)
                                && r_mm == initial_z_mm
                        }
                    } && (shift_mm.is_none() || *cycle == DrillCycle::FineBore)
                        && (reentry_mm.is_none() || *cycle == DrillCycle::Peck)
                        && (lift_mm.is_none() || *cycle == DrillCycle::ChipBreak);
                    if !shape
                        || !center_mm
                            .iter()
                            .chain([z_mm, r_mm, initial_z_mm])
                            .all(|v| v.is_finite() && v.abs() <= 1e6)
                        || !(z_mm < r_mm && r_mm <= initial_z_mm)
                        || !(feed_mm_min.is_finite() && *feed_mm_min > 0.)
                        || *covers == 0
                        || *covers > 1_000_000
                    {
                        return Err(error(
                            "invalid canned cycle (needs Z < R <= initial level, positive feed and the words of its type)",
                        ));
                    }
                    if covered > 0 || tap.is_some() {
                        return Err(error("canned cycles cannot nest"));
                    }
                    if position[2] != *initial_z_mm {
                        return Err(error("a canned cycle starts at its return level"));
                    }
                    covered = *covers;
                }
                Event::Probe { cycle, covers } => {
                    if covered > 0 || tap.is_some() {
                        return Err(error("probing cycles cannot nest in other cycles"));
                    }
                    let Some(tool) = active_tool.as_deref() else {
                        return Err(error("a probing cycle needs an active tool"));
                    };
                    cycle.validate().map_err(|e| error(&e))?;
                    let stylus = cycle.feature.stylus();
                    if stylus != self.probes.contains(tool) {
                        return Err(error(if stylus {
                            "a probing cycle needs the active tool to be a probe"
                        } else {
                            "tool setting measures the active cutting tool, not a probe"
                        }));
                    }
                    if stylus && rpm != 0. {
                        return Err(error("stop the spindle before probing"));
                    }
                    if beam_tool(&active_tool) {
                        return Err(error("a beam tool cannot be probed or set"));
                    }
                    match cycle.result {
                        crate::probe::ProbeResult::WorkOffset { number } => {
                            if capabilities.is_some_and(|c| number > c.max_work_offset) {
                                return Err(error("probed work offset unsupported"));
                            }
                            if !probed_offsets.contains(&number) {
                                probed_offsets.push(number);
                            }
                        }
                        crate::probe::ProbeResult::ToolLength { register } => {
                            if active_offset.is_some_and(|o| probed_offsets.contains(&o)) {
                                return Err(error(
                                    "this program already corrected its work offset by probing, which moves the tool setter declared in that offset: set tool lengths before probing the work offset",
                                ));
                            }
                            if capabilities.is_some_and(|c| register > c.max_length_register) {
                                return Err(error("tool length register unsupported"));
                            }
                        }
                        crate::probe::ProbeResult::ToolRadius { register } => {
                            if capabilities.is_some_and(|c| register > c.max_tool_number) {
                                return Err(error("tool table entry unsupported"));
                            }
                            radius_set.insert(tool.to_string());
                        }
                    }
                    let radius = self.radii[tool];
                    let start = cycle.start(radius).map_err(|e| error(&e))?;
                    if distance(start, position) > self.tolerance_mm {
                        return Err(error(&format!(
                            "a probing cycle starts at its start point {start:?}, not at {position:?}"
                        )));
                    }
                    let legs = cycle.legs(radius).map_err(|e| error(&e))?;
                    if legs.len() != *covers as usize {
                        return Err(error(&format!(
                            "this probing cycle has {} moves, not {covers}",
                            legs.len()
                        )));
                    }
                    probe_legs = legs.into();
                    covered = *covers;
                }
                Event::Motion { motion: m } => {
                    let m_error = |message: &str| {
                        format!(
                            "event {index}, operation {}, move {}: {message}",
                            m.operation, m.id
                        )
                    };
                    if !valid_id(&m.operation) || active_tool.as_deref() != Some(m.tool.as_str()) {
                        return Err(m_error("motion must name its operation and active tool"));
                    }
                    if active_offset.is_none() {
                        return Err(m_error("select an explicit work offset before motion"));
                    }
                    if last_id.is_some_and(|id| m.id <= id) {
                        return Err(m_error("motion IDs must be strictly increasing"));
                    }
                    if m.curve.start() != position {
                        return Err(m_error(
                            "discontinuous motion start; declare tool-tip reset with a tool change",
                        ));
                    }
                    if compensation != CompensationSide::Off || compensation_move {
                        let (start, end) = (m.curve.start(), m.curve.end());
                        // The plane: set by the move that turns it on (XY when
                        // it stays at one height, XZ when it stays at one Y).
                        let plane = compensation_plane.unwrap_or(Plane::Xy);
                        let in_plane_curve = matches!(
                            m.curve,
                            Curve::Arc { plane: p, .. } | Curve::Helix { plane: p, .. } if p == plane
                        );
                        // Travel within the plane in every move (in XY a ramp
                        // may also change Z); switching over a straight move
                        // within the plane.
                        let (line, flat) = match plane {
                            Plane::Xy => (
                                matches!(m.curve, Curve::Line { .. })
                                    && (start[0] - end[0]).hypot(start[1] - end[1]) > 0.,
                                start[2] == end[2],
                            ),
                            Plane::Xz => (
                                matches!(m.curve, Curve::Line { .. })
                                    && start[1] == end[1]
                                    && (start[0] - end[0]).hypot(start[2] - end[2]) > 0.,
                                true,
                            ),
                            Plane::Yz => (false, false),
                        };
                        let in_plane = match plane {
                            Plane::Xy => line || in_plane_curve,
                            _ => line || in_plane_curve && start[1] == end[1],
                        };
                        if !in_plane || (compensation_move && !(line && flat)) {
                            return Err(m_error(
                                "with cutter compensation on, every move travels in its plane (XY, or XZ on a lathe; no straight Z moves on a mill); it turns on and off over a straight move in that plane",
                            ));
                        }
                        if compensation == CompensationSide::Off {
                            compensation_plane = None;
                        }
                        compensation_move = false;
                    }
                    let curve = m
                        .curve
                        .prepare(self.tolerance_mm)
                        .map_err(|e| m_error(&e))?;
                    m.validate_feed(capabilities.map(|c| c.max_feed_mm_min))
                        .map_err(|e| m_error(&e))?;
                    if m.kind == MoveKind::Shift {
                        // On the ray from the oriented stop away from the edge.
                        let Some((stop, edge)) = oriented else {
                            return Err(m_error(
                                "an oriented-spindle shift needs an oriented spindle stop first",
                            ));
                        };
                        let on_ray = |p: Vec3| {
                            let d = [p[0] - stop[0], p[1] - stop[1]];
                            let along = -(d[0] * edge[0] + d[1] * edge[1]);
                            let across = (d[0] * edge[1] - d[1] * edge[0]).abs();
                            along >= -self.tolerance_mm && across <= self.tolerance_mm
                        };
                        if !on_ray(m.curve.start()) || !on_ray(m.curve.end()) {
                            return Err(m_error(
                                "an oriented-spindle shift moves only away from the cutting edge and back",
                            ));
                        }
                    }
                    if css.is_some() && matches!(m.kind, MoveKind::Tap | MoveKind::Thread) {
                        return Err(m_error(
                            "threading and tapping need a constant spindle speed (G97), not constant surface speed",
                        ));
                    }
                    if let Some(mm_rev) = feed_rev
                        && !m.kind.is_rapid()
                        && !matches!(m.kind, MoveKind::Tap | MoveKind::Thread)
                    {
                        let top = match css {
                            Some((v, max)) => css_rpm_bound(&m.curve, v, max),
                            None => rpm,
                        };
                        let expected = mm_rev * top;
                        if top <= 0.
                            || m.feed_mm_min
                                .is_none_or(|f| (f - expected).abs() > 1e-9 * expected.max(1.))
                        {
                            return Err(m_error(
                                "with feed per revolution, a feed move needs the spindle running and its feed the per-revolution feed x the highest rpm along it",
                            ));
                        }
                    }
                    if m.kind == MoveKind::Tap {
                        let Curve::Line { start, end } = m.curve else {
                            return Err(m_error("tapping motion must be a line"));
                        };
                        let feed = m.feed_mm_min.unwrap_or(0.);
                        match tap.take() {
                            Some((top, bottom, f)) => {
                                if start != bottom || end != top || feed != f {
                                    return Err(m_error(
                                        "tapping return must reverse its tapping motion exactly",
                                    ));
                                }
                            }
                            None => {
                                if start[0] != end[0] || start[1] != end[1] || end[2] >= start[2] {
                                    return Err(m_error("tapping motion must go straight down"));
                                }
                                // Either direction: the spindle sense is
                                // the thread hand (M3 right, M4 left).
                                if rpm == 0. || feed <= 0. {
                                    return Err(m_error(
                                        "tapping needs the spindle running and a feed",
                                    ));
                                }
                                tap = Some((start, end, feed));
                            }
                        }
                    }
                    if m.kind.is_rapid() {
                        summary.rapid_length_mm += curve.length_mm();
                    } else {
                        if m.kind == MoveKind::Thread && !matches!(m.curve, Curve::Line { .. }) {
                            return Err(m_error("a threading move is a straight line"));
                        }
                        if matches!(
                            m.kind,
                            MoveKind::Cut | MoveKind::Plunge | MoveKind::Lead | MoveKind::Thread
                        ) {
                            if beam_tool(&active_tool) {
                                if !beam_on {
                                    return Err(m_error("cutting motion requires the beam on"));
                                }
                            } else if rpm == 0. {
                                return Err(m_error("cutting motion requires running spindle"));
                            }
                        }
                        summary.fed_length_mm += curve.length_mm();
                    }
                    if let Some(c) = capabilities {
                        match m.curve {
                            Curve::Line { .. } => {}
                            Curve::Arc {
                                plane, sweep_rad, ..
                            }
                            | Curve::Helix {
                                plane, sweep_rad, ..
                            } => {
                                if !c.arcs
                                    || !c.planes.contains(&plane)
                                    || sweep_rad.abs() > c.max_sweep_rad
                                    || matches!(m.curve, Curve::Helix { .. }) && !c.helices
                                {
                                    return Err(m_error(
                                        "circular motion unsupported by profile; explicit bounded lowering required",
                                    ));
                                }
                            }
                        }
                    }
                    position = m.curve.end();
                    last_id = Some(m.id);
                    summary.motions += 1;
                }
            }
        }
        summary.final_position_mm = position;
        summary.events += events.len();
        self.position = position;
        self.active_tool = active_tool;
        self.rpm = rpm;
        self.last_id = last_id;
        self.changes = changes;
        self.active_offset = active_offset;
        self.clockwise = clockwise;
        self.tap = tap;
        self.covered = covered;
        self.compensation = compensation;
        self.compensation_move = compensation_move;
        self.compensation_plane = compensation_plane;
        self.oriented = oriented;
        self.beam_on = beam_on;
        self.height_control = height_control;
        self.css = css;
        self.feed_rev = feed_rev;
        self.probe_legs = probe_legs;
        self.probed_offsets = probed_offsets;
        self.radius_set = radius_set;
        self.summary = summary;
        Ok(())
    }
}
