//! Shared closed offset profiles and source ownership for contour/chamfer.
use crate::{
    Verdict,
    contour::Boundaries,
    planar::{Component, ConstructionOptions, PlanarRegion, PreparedRegion},
    pocket::Stroke,
    stock::CancelSignal,
};
pub(crate) struct ProfileLoop {
    pub strokes: Vec<Stroke>,
    pub component: usize,
    pub hole: Option<usize>,
}
pub(crate) struct Profiles {
    pub loops: Vec<ProfileLoop>,
    pub edges: usize,
    pub arcs: usize,
    pub diagnostics: Vec<String>,
}

pub(crate) struct ProfileSettings {
    pub id: String,
    pub tool: String,
    pub heights: crate::heights::Heights,
    pub feed: f64,
    pub plunge: f64,
    pub tolerance: f64,
    pub lead_in: f64,
    pub lead_out: f64,
    pub spindle: f64,
    pub work_offset: u8,
    pub tool_number: u32,
    pub length_register: u32,
    pub coolant: crate::motion::Coolant,
}
pub(crate) enum ProfileDepths {
    Fixed {
        z: f64,
    },
    Linear {
        top: f64,
        target: f64,
        step: f64,
    },
    Chamfer {
        top: f64,
        reference_height: f64,
        height_per_width: f64,
        width: f64,
        step: f64,
        final_tip: f64,
    },
}
#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase {
    Lift,
    Approach,
    Entry,
    LeadIn,
    Cut,
    LeadOut,
    Retract,
    /// Rapid from the feed height to the retract height (clearance after the
    /// final pass). Zero length, so skipped, with default heights.
    Rise,
    /// Rapid down to the feed height above the entry. Zero length with defaults.
    Descend,
    Done,
}
pub(crate) struct ProfileMotion {
    settings: ProfileSettings,
    spec: crate::stock::StockSpec,
    initial: crate::motion::Vec3,
    position: crate::motion::Vec3,
    loops: Vec<Vec<Stroke>>,
    depths: ProfileDepths,
    passes: usize,
    pass: usize,
    ring: usize,
    edge: usize,
    phase: Phase,
    next_id: u64,
    expected_generation: Option<u64>,
    pub drained: bool,
    pub cuts: usize,
    pub closed_loops: usize,
    /// Cutter compensation side per loop (empty: none), and events due.
    pub sides: Vec<crate::motion::CompensationSide>,
    events: Vec<crate::motion::Event>,
    /// Holding tabs, and the current loop's cut moves (ring, pass, moves).
    pub tabs: Option<TabPlan>,
    cut: Option<(usize, usize, Vec<CutMove>)>,
    /// Machining limits: stretches they block are lifted over.
    pub limits: Option<crate::limits::PreparedLimits>,
    /// Ramp: the first pass of each loop descends from this height to its
    /// depth along the loop (entered at this height).
    pub ramp_from: Option<f64>,
    /// Ramp entry (`RampEntry`, linear depths, one pass): each loop is
    /// entered at the top and descends to the final depth around the loop in
    /// whole laps, at no more than the entry's angle, no lap deeper than the
    /// depth step, at `ramp_feed`; a flat lap at depth then cuts the sloped
    /// floor away. This is FreeCAD CAM's helix ramp on a closed path
    /// (`createHelix` in `Path/Base/Generator/ramp_entry.py`: whole loops,
    /// bounded by the angle and by a pitch per turn): a closed pass needs no
    /// reversal, and the tool never comes straight down into material.
    pub entry_ramp: Option<crate::entry::RampEntry>,
    pub ramp_feed: f64,
}
type CutMove = (crate::motion::Curve, crate::motion::MoveKind);
/// Holding tabs on the tool-center loops: around each center (arc length),
/// the tool stays at `top` over `half` either side, with 45 degree ramps
/// (`ramp`) or straight up and down.
#[derive(Clone, Debug)]
pub(crate) struct TabPlan {
    pub top: f64,
    pub half: f64,
    pub ramp: bool,
    pub centers: Vec<Vec<f64>>,
}
pub(crate) fn stroke_length(s: &Stroke) -> f64 {
    match s.arc {
        None => (s.b[0] - s.a[0]).hypot(s.b[1] - s.a[1]),
        Some((c, sweep)) => (s.a[0] - c[0]).hypot(s.a[1] - c[1]) * sweep.abs(),
    }
}
/// The nearest point of a stroke to p, as (distance, stroke parameter). A
/// point within `eps` of an arc's center is equally near the whole arc: its
/// middle is taken.
fn stroke_nearest(s: &Stroke, p: crate::Point, eps: f64) -> (f64, f64) {
    use std::f64::consts::TAU;
    let t = match s.arc {
        None => {
            let d = [s.b[0] - s.a[0], s.b[1] - s.a[1]];
            let n = d[0] * d[0] + d[1] * d[1];
            if n > 0. {
                (((p[0] - s.a[0]) * d[0] + (p[1] - s.a[1]) * d[1]) / n).clamp(0., 1.)
            } else {
                0.
            }
        }
        Some((c, sweep)) => {
            let v = [p[0] - c[0], p[1] - c[1]];
            let turn = (v[1].atan2(v[0]) - (s.a[1] - c[1]).atan2(s.a[0] - c[0])) * sweep.signum();
            let turn = turn.rem_euclid(TAU);
            let far = |q: crate::Point| (p[0] - q[0]).hypot(p[1] - q[1]);
            if v[0].hypot(v[1]) <= eps {
                0.5
            } else if turn <= sweep.abs() {
                turn / sweep.abs()
            } else if far(s.a) <= far(s.b) {
                0.
            } else {
                1.
            }
        }
    };
    let q = stroke_point(s, t);
    ((p[0] - q[0]).hypot(p[1] - q[1]), t)
}
/// The nearest point of a closed loop to p: (distance, arc length from the
/// loop start, the point).
fn loop_nearest(strokes: &[Stroke], p: crate::Point, eps: f64) -> (f64, f64, crate::Point) {
    let mut best = (f64::INFINITY, 0., p);
    let mut start = 0.;
    for s in strokes {
        let length = stroke_length(s);
        let (d, t) = stroke_nearest(s, p, eps);
        // A point at an arc's center (a convex corner of the part, seen from
        // the tool path) is as near the ends of the neighboring strokes as
        // the arc itself: the arc's middle wins.
        let corner = eps > 0.
            && s.arc
                .is_some_and(|(c, _)| (p[0] - c[0]).hypot(p[1] - c[1]) <= eps);
        if corner {
            return (d, start + t * length, stroke_point(s, t));
        }
        if d < best.0 {
            best = (d, start + t * length, stroke_point(s, t));
        }
        start += length;
    }
    best
}
/// Tab centers (arc length on each tool-center loop) for tabs at given XY
/// points on the part boundary, as picked points place them in established
/// CAM (Fusion 2D Contour "Tab Positioning: At points"; FreeCAD CAM Tag
/// dressup positions).
///
/// `boundaries[k]` is the part boundary that tool-center loop `loops[k]` was
/// offset from. A point belongs to the one boundary within `tolerance` of it
/// and is taken as its nearest point there; the tab is centered where the
/// tool is nearest to that point (the middle of the corner arc for a point
/// on a convex corner). `reach` is the loop length a tab needs either side of
/// its center (half width, tool radius, ramp and a gap): tabs must be that
/// far from the loop start, where the tool enters and leaves, and twice that
/// from each other, as evenly spaced tabs are.
pub(crate) fn tab_centers_at_points(
    boundaries: &[Vec<Stroke>],
    loops: &[Vec<Stroke>],
    points: &[crate::Point],
    tolerance: f64,
    reach: f64,
    width: f64,
) -> Result<Vec<Vec<f64>>, String> {
    // Per loop: (center, the point given).
    let mut placed: Vec<Vec<(f64, crate::Point)>> = vec![vec![]; loops.len()];
    for &p in points {
        let near: Vec<(usize, f64, crate::Point)> = boundaries
            .iter()
            .enumerate()
            .map(|(k, b)| {
                let (d, _, foot) = loop_nearest(b, p, 0.);
                (k, d, foot)
            })
            .collect();
        let on: Vec<&(usize, f64, crate::Point)> =
            near.iter().filter(|n| n.1 <= tolerance).collect();
        match on.as_slice() {
            [] => {
                let (_, d, foot) = near
                    .iter()
                    .min_by(|a, b| a.1.total_cmp(&b.1))
                    .ok_or("tabs need a selected boundary loop")?;
                return Err(format!(
                    "tab position ({}, {}) is not on a selected part boundary: the nearest point on one, ({:.4}, {:.4}), is {d:.4} mm away, more than position_tolerance_mm {tolerance}; give a point on the boundary (or select its loop with `boundaries`)",
                    p[0], p[1], foot[0], foot[1]
                ));
            }
            [(k, _, foot)] => {
                let (_, center, _) = loop_nearest(&loops[*k], *foot, tolerance);
                placed[*k].push((center, p));
            }
            [a, b, ..] => {
                return Err(format!(
                    "tab position ({}, {}) is within position_tolerance_mm {tolerance} of two boundary loops ({:.4} mm from loop {} and {:.4} mm from loop {}): give a point on one of them or a smaller tolerance",
                    p[0], p[1], a.1, a.0, b.1, b.0
                ));
            }
        }
    }
    for (ring, tabs) in loops.iter().zip(&mut placed) {
        if tabs.is_empty() {
            continue;
        }
        tabs.sort_by(|a, b| a.0.total_cmp(&b.0));
        let length: f64 = ring.iter().map(stroke_length).sum();
        let start = ring[0].a;
        for (c, p) in [tabs[0], tabs[tabs.len() - 1]] {
            let room = c.min(length - c);
            if room <= reach {
                return Err(format!(
                    "the tab at ({}, {}) is {room:.3} mm along the tool path from the loop start ({:.4}, {:.4}), where the tool enters and leaves: a tab of width {width} needs more than {reach:.3} mm there (half its width, the tool radius and its ramp) on a loop {length:.3} mm long; move the tab",
                    p[0], p[1], start[0], start[1]
                ));
            }
        }
        for w in tabs.windows(2) {
            let gap = w[1].0 - w[0].0;
            if gap <= 2. * reach {
                return Err(format!(
                    "the tabs at ({}, {}) and ({}, {}) overlap: their centers are {gap:.3} mm apart along the tool path and tabs of width {width} need more than {:.3} mm (the width, the tool diameter and the ramps); move one or make them narrower",
                    w[0].1[0],
                    w[0].1[1],
                    w[1].1[0],
                    w[1].1[1],
                    2. * reach
                ));
            }
        }
    }
    Ok(placed
        .into_iter()
        .map(|tabs| tabs.into_iter().map(|t| t.0).collect())
        .collect())
}
/// The point at parameter t of a stroke, exactly its ends at 0 and 1.
pub(crate) fn stroke_point(s: &Stroke, t: f64) -> crate::Point {
    if t <= 0. {
        return s.a;
    }
    if t >= 1. {
        return s.b;
    }
    match s.arc {
        None => [
            s.a[0] + t * (s.b[0] - s.a[0]),
            s.a[1] + t * (s.b[1] - s.a[1]),
        ],
        Some((c, sweep)) => {
            let r = (s.a[0] - c[0]).hypot(s.a[1] - c[1]);
            let angle = (s.a[1] - c[1]).atan2(s.a[0] - c[0]) + t * sweep;
            [c[0] + r * angle.cos(), c[1] + r * angle.sin()]
        }
    }
}
/// The part of a stroke from t0 to t1, its height going from z0 to z1.
pub(crate) fn sub_curve(s: &Stroke, t0: f64, t1: f64, z0: f64, z1: f64) -> crate::motion::Curve {
    use crate::motion::{Curve, Plane};
    let (a, b) = (stroke_point(s, t0), stroke_point(s, t1));
    let (start, end) = ([a[0], a[1], z0], [b[0], b[1], z1]);
    match s.arc {
        None => Curve::Line { start, end },
        Some((c, sweep)) => {
            let center = [c[0], c[1], z0];
            let sweep_rad = sweep * (t1 - t0);
            if z0 == z1 {
                Curve::Arc {
                    start,
                    end,
                    center,
                    plane: Plane::Xy,
                    sweep_rad,
                }
            } else {
                Curve::Helix {
                    start,
                    end,
                    center,
                    plane: Plane::Xy,
                    sweep_rad,
                }
            }
        }
    }
}
/// A loop's cut moves at depth z, lifted over holding tabs.
pub(crate) fn tabbed_loop(
    strokes: &[Stroke],
    z: f64,
    start_z: f64,
    tabs: Option<(&TabPlan, &[f64])>,
    lifts: &[(f64, f64, f64)],
) -> Result<Vec<CutMove>, String> {
    use crate::motion::{Curve, MoveKind};
    let tabs = tabs.filter(|(p, centers)| z < p.top && !centers.is_empty());
    if tabs.is_none() && lifts.is_empty() && start_z == z {
        return Ok(strokes
            .iter()
            .map(|s| (s.curve(z), MoveKind::Cut))
            .collect());
    }
    let lengths: Vec<f64> = strokes.iter().map(stroke_length).collect();
    let total: f64 = lengths.iter().sum();
    // The base height: z, or a ramp from start_z down to z along the loop.
    let base = |s: f64| {
        if s >= total {
            z
        } else {
            start_z + (z - start_z) * s / total
        }
    };
    // Raised stretches: tabs (with ramps) and lifts over blocked stretches,
    // as (start, end, corners of the height profile); they must not overlap.
    type Raised = (f64, f64, Vec<(f64, f64)>);
    let mut raised: Vec<Raised> = vec![];
    if let Some((plan, centers)) = tabs {
        let ramp = if plan.ramp { plan.top - z } else { 0. };
        for &c in centers {
            let (s0, s1) = (c - plan.half, c + plan.half);
            raised.push((
                s0 - ramp,
                s1 + ramp,
                vec![
                    (s0 - ramp, base(s0 - ramp)),
                    (s0, plan.top),
                    (s1, plan.top),
                    (s1 + ramp, base(s1 + ramp)),
                ],
            ));
        }
    }
    for &(a, b, lift) in lifts {
        if a <= 0. || b >= total {
            return Err("the limits block the start or end of a profile loop (where it enters and leaves): change the loop start, leads or limits".into());
        }
        raised.push((a, b, vec![(a, base(a)), (a, lift), (b, lift), (b, base(b))]));
    }
    raised.sort_by(|x, y| x.0.total_cmp(&y.0));
    if raised.windows(2).any(|w| w[1].0 < w[0].1) {
        return Err(
            "holding tabs overlap a stretch the limits block: move the tabs or change the limits"
                .into(),
        );
    }
    // The height profile along the loop: (arc length, z) corners.
    let mut profile = vec![(0., start_z)];
    for (_, _, corners) in &raised {
        profile.extend(corners.iter().copied());
    }
    profile.push((total, z));
    let mut starts = Vec::with_capacity(strokes.len());
    let mut acc = 0.;
    for l in &lengths {
        starts.push(acc);
        acc += l;
    }
    // Stroke parameter of arc length s (the last stroke owns the end).
    let locate = |s: f64| -> (usize, f64) {
        let k = starts.iter().rposition(|&c| c <= s).unwrap_or(0);
        let t = if lengths[k] > 0. {
            ((s - starts[k]) / lengths[k]).clamp(0., 1.)
        } else {
            0.
        };
        (k, t)
    };
    let mut moves = vec![];
    for w in profile.windows(2) {
        let ((sa, za), (sb, zb)) = (w[0], w[1]);
        if sb <= sa {
            if za != zb {
                let (k, t) = locate(sa);
                let p = stroke_point(&strokes[k], t);
                let kind = if zb < za {
                    MoveKind::Plunge
                } else {
                    MoveKind::Cut
                };
                moves.push((
                    Curve::Line {
                        start: [p[0], p[1], za],
                        end: [p[0], p[1], zb],
                    },
                    kind,
                ));
            }
            continue;
        }
        let height = |s: f64| {
            if s >= sb {
                zb
            } else {
                za + (zb - za) * (s - sa) / (sb - sa)
            }
        };
        for k in 0..strokes.len() {
            let (from, to) = (sa.max(starts[k]), sb.min(starts[k] + lengths[k]));
            if to <= from {
                continue;
            }
            let t0 = (from - starts[k]) / lengths[k];
            let t1 = if to >= starts[k] + lengths[k] {
                1.
            } else {
                (to - starts[k]) / lengths[k]
            };
            let t0 = if from <= starts[k] { 0. } else { t0 };
            moves.push((
                sub_curve(&strokes[k], t0, t1, height(from), height(to)),
                MoveKind::Cut,
            ));
        }
    }
    Ok(moves)
}
pub(crate) fn lead_point(edges: &[Stroke], z: f64, length: f64, end: bool) -> crate::motion::Vec3 {
    let stroke = if end {
        *edges.last().unwrap()
    } else {
        edges[0]
    };
    let mut p = if end {
        stroke.curve(z).end()
    } else {
        stroke.curve(z).start()
    };
    let t = stroke.tangent(end);
    let sign = if end { 1. } else { -1. };
    p[0] += sign * length * t[0];
    p[1] += sign * length * t[1];
    p
}
impl ProfileMotion {
    pub fn new(
        settings: ProfileSettings,
        spec: &crate::stock::StockSpec,
        initial: crate::motion::Vec3,
        loops: Vec<Vec<Stroke>>,
        depths: ProfileDepths,
        passes: usize,
        first_id: u64,
    ) -> Self {
        Self {
            settings,
            spec: spec.clone(),
            initial,
            position: initial,
            loops,
            depths,
            passes,
            pass: 0,
            ring: 0,
            edge: 0,
            phase: Phase::Lift,
            next_id: first_id,
            expected_generation: None,
            drained: false,
            cuts: 0,
            closed_loops: 0,
            sides: vec![],
            events: vec![],
            tabs: None,
            cut: None,
            limits: None,
            ramp_from: None,
            entry_ramp: None,
            ramp_feed: 0.,
        }
    }
    /// The stretches of a loop at depth z the limits block, as arc-length
    /// intervals, and the height to lift over them.
    pub(crate) fn loop_count(&self) -> usize {
        self.loops.len()
    }
    pub(crate) fn operation_id(&self) -> &str {
        &self.settings.id
    }
    pub(crate) fn lifts(&self, ring: usize, z: f64) -> Result<Vec<(f64, f64, f64)>, String> {
        let Some(limits) = &self.limits else {
            return Ok(vec![]);
        };
        let s = &self.settings;
        let mut blocked: Vec<(f64, f64)> = vec![];
        let mut start = 0.;
        for stroke in &self.loops[ring] {
            let length = stroke_length(stroke);
            let allowed = limits.allowed_spans(&stroke.curve(z), true)?;
            let mut t = 0.;
            for span in allowed.iter().chain([&[1., 1.]]) {
                if span[0] > t {
                    let (a, b) = (start + t * length, start + span[0] * length);
                    match blocked.last_mut() {
                        Some(last) if last.1 >= a => last.1 = b,
                        _ => blocked.push((a, b)),
                    }
                }
                t = span[1];
            }
            start += length;
        }
        if blocked.is_empty() {
            return Ok(vec![]);
        }
        if !self.sides.is_empty() {
            return Err("the limits block part of a compensated contour: lifting over them moves straight up and down, which cutter compensation does not allow".into());
        }
        let lift = limits.avoid_top().map_or(s.heights.retract, |top| {
            s.heights.retract.max(top + 16. * s.tolerance)
        });
        if lift > s.heights.clearance {
            return Err(format!(
                "lifting over an avoid area needs Z {lift:.4}, above clearance_z_mm {}",
                s.heights.clearance
            ));
        }
        Ok(blocked.into_iter().map(|(a, b)| (a, b, lift)).collect())
    }
    /// Events due after the motion just handed out (cutter compensation
    /// on before the lead-in, off before the lead-out).
    pub fn take_events(&mut self) -> Vec<crate::motion::Event> {
        std::mem::take(&mut self.events)
    }
    pub fn check_stock(
        &self,
        stock: &crate::stock::Stock,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        if stock.spec() != &self.spec
            || stock.generation() == u64::MAX
            || self
                .expected_generation
                .is_some_and(|g| g != stock.generation())
        {
            return Err("commit preceding profile motion to matching stock".into());
        }
        if cancel.is_cancelled() {
            return Err("profile motion cancelled".into());
        }
        Ok(())
    }
    pub fn prefix_events(&self) -> Vec<crate::motion::Event> {
        self.settings.prefix_events(self.initial)
    }
    pub fn suffix_events(&self) -> Vec<crate::motion::Event> {
        self.settings.suffix_events()
    }
}
impl ProfileSettings {
    pub fn prefix_events(&self, initial: crate::motion::Vec3) -> Vec<crate::motion::Event> {
        use crate::motion::*;
        let s = self;
        vec![
            Event::WorkOffset {
                number: s.work_offset,
            },
            Event::ToolChange {
                tool: s.tool.clone(),
                number: s.tool_number,
                length_register: s.length_register,
                position_after_mm: initial,
            },
            Event::Spindle {
                rpm: s.spindle,
                direction: SpindleDirection::Cw,
            },
            Event::Coolant { mode: s.coolant },
        ]
    }
    pub fn suffix_events(&self) -> Vec<crate::motion::Event> {
        use crate::motion::*;
        vec![
            Event::Coolant { mode: Coolant::Off },
            Event::Spindle {
                rpm: 0.,
                direction: SpindleDirection::Cw,
            },
        ]
    }
}
impl ProfileMotion {
    pub fn next(
        &mut self,
        stock: &crate::stock::Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<crate::motion::Motion>, String> {
        use crate::motion::*;
        loop {
            self.check_stock(stock, cancel)?;
            if self.phase == Phase::Done {
                self.drained = true;
                return Ok(None);
            }
            let z = match self.depths {
                ProfileDepths::Fixed { z } => z,
                ProfileDepths::Linear { top, target, step } => {
                    (top - (self.pass + 1) as f64 * step).max(target)
                }
                ProfileDepths::Chamfer {
                    top,
                    reference_height,
                    height_per_width,
                    width,
                    step,
                    final_tip,
                } => {
                    if self.pass + 1 == self.passes {
                        final_tip
                    } else {
                        top - reference_height
                            - ((self.pass + 1) as f64 * step).min(width) * height_per_width
                    }
                }
            };
            // A ramp pass enters at the height it descends from.
            let entry_z = match self.ramp_from {
                Some(from) if self.pass == 0 => from.max(z),
                _ => z,
            };
            // A ramp entry is one descent from the top to the final depth.
            let (z, entry_z, ramp_step) = match (&self.entry_ramp, &self.depths) {
                (Some(_), ProfileDepths::Linear { top, target, step }) if self.passes == 1 => {
                    (*target, *top, *step)
                }
                (Some(_), _) => {
                    return Err("a ramp entry needs linear depths, planned as one pass".into());
                }
                (None, _) => (z, entry_z, 0.),
            };
            let first = self.loops[self.ring][0].curve(entry_z).start();
            let s = &self.settings;
            let (curve, kind, next) = match self.phase {
                Phase::Lift => (
                    Curve::Line {
                        start: self.position,
                        end: [self.position[0], self.position[1], s.heights.clearance],
                    },
                    MoveKind::Rapid,
                    Phase::Approach,
                ),
                Phase::Approach => {
                    // Travel at the current rapid height (clearance before the
                    // first pass, the retract height between passes).
                    let mut end = lead_point(&self.loops[self.ring], z, s.lead_in, false);
                    end[2] = self.position[2];
                    (
                        Curve::Line {
                            start: self.position,
                            end,
                        },
                        MoveKind::Rapid,
                        Phase::Descend,
                    )
                }
                Phase::Descend => (
                    Curve::Line {
                        start: self.position,
                        end: [self.position[0], self.position[1], s.heights.feed],
                    },
                    MoveKind::Rapid,
                    Phase::Entry,
                ),
                Phase::Entry => (
                    Curve::Line {
                        start: self.position,
                        end: lead_point(&self.loops[self.ring], entry_z, s.lead_in, false),
                    },
                    MoveKind::Plunge,
                    Phase::LeadIn,
                ),
                Phase::LeadIn => (
                    Curve::Line {
                        start: self.position,
                        end: first,
                    },
                    MoveKind::Lead,
                    Phase::Cut,
                ),
                Phase::Cut => {
                    if self
                        .cut
                        .as_ref()
                        .is_none_or(|c| (c.0, c.1) != (self.ring, self.pass))
                    {
                        let tabs = self
                            .tabs
                            .as_ref()
                            .map(|p| (p, p.centers[self.ring].as_slice()));
                        let lifts = self.lifts(self.ring, z)?;
                        let moves = if let Some(ramp) = &self.entry_ramp {
                            if tabs.is_some() || !lifts.is_empty() {
                                return Err(format!(
                                    "operation {}: a ramp entry cannot be combined with {}: the tool comes back down from them steeper than the ramp angle. Remove the entry or the {}",
                                    s.id,
                                    if tabs.is_some() {
                                        "holding tabs"
                                    } else {
                                        "limits that block part of the loop (it is lifted over them)"
                                    },
                                    if tabs.is_some() {
                                        "tabs"
                                    } else {
                                        "blocking limits"
                                    },
                                ));
                            }
                            let ring = &self.loops[self.ring];
                            let length: f64 = ring.iter().map(stroke_length).sum();
                            let depth = entry_z - z;
                            let mut moves = vec![];
                            if depth > 0. {
                                let laps = ramp.laps(length, depth, ramp_step).map_err(|e| {
                                    format!(
                                        "operation {}, loop {}, level Z{z}: {e}",
                                        s.id, self.ring
                                    )
                                })?;
                                for lap in 0..laps {
                                    let from = entry_z - depth * lap as f64 / laps as f64;
                                    let to = if lap + 1 == laps {
                                        z
                                    } else {
                                        entry_z - depth * (lap + 1) as f64 / laps as f64
                                    };
                                    moves.extend(
                                        tabbed_loop(ring, to, from, None, &[])?
                                            .into_iter()
                                            .map(|(curve, _)| (curve, MoveKind::Plunge)),
                                    );
                                }
                            }
                            moves.extend(tabbed_loop(ring, z, z, None, &[])?);
                            moves
                        } else {
                            tabbed_loop(&self.loops[self.ring], z, entry_z, tabs, &lifts)?
                        };
                        self.cut = Some((self.ring, self.pass, moves));
                    }
                    let (curve, kind) = self.cut.as_ref().unwrap().2[self.edge].clone();
                    (curve, kind, Phase::Cut)
                }
                Phase::LeadOut => (
                    Curve::Line {
                        start: self.position,
                        end: lead_point(&self.loops[self.ring], z, s.lead_out, true),
                    },
                    MoveKind::Lead,
                    Phase::Retract,
                ),
                Phase::Retract => (
                    Curve::Line {
                        start: self.position,
                        end: [self.position[0], self.position[1], s.heights.feed],
                    },
                    MoveKind::Retract,
                    Phase::Rise,
                ),
                Phase::Rise => {
                    let last = self.ring + 1 == self.loops.len() && self.pass + 1 == self.passes;
                    let z = if last {
                        s.heights.clearance
                    } else {
                        s.heights.retract
                    };
                    (
                        Curve::Line {
                            start: self.position,
                            end: [self.position[0], self.position[1], z],
                        },
                        MoveKind::Rapid,
                        Phase::Approach,
                    )
                }
                Phase::Done => unreachable!(),
            };
            // Entries, leads, retracts and rapids must respect the limits as
            // they are (the loop's cut moves were already lifted over them).
            if self.phase != Phase::Cut
                && let Some(limits) = &self.limits
                && !matches!(curve, Curve::Line { start, end } if start == end)
                && !limits
                    .allows_curve(&curve, !matches!(kind, MoveKind::Rapid | MoveKind::Link))?
            {
                return Err(format!(
                    "a {kind:?} move from {:?} to {:?} enters an avoid area (or a lead or entry leaves the containment): change leads, heights or limits",
                    curve.start(),
                    curve.end()
                ));
            }
            let motion = if matches!(curve,Curve::Line {start,end}if start==end) {
                None
            } else {
                curve.prepare(s.tolerance)?;
                let m = Motion {
                    id: self.next_id,
                    operation: s.id.clone(),
                    tool: s.tool.clone(),
                    kind,
                    curve,
                    feed_mm_min: if kind == MoveKind::Rapid {
                        None
                    } else {
                        Some(if kind == MoveKind::Plunge {
                            if self.phase == Phase::Cut && self.entry_ramp.is_some() {
                                self.ramp_feed
                            } else {
                                s.plunge
                            }
                        } else {
                            s.feed
                        })
                    },
                };
                self.position = m.curve.end();
                self.next_id += 1;
                self.expected_generation = Some(stock.generation() + 1);
                Some(m)
            };
            let phase = self.phase;
            self.phase = next;
            if phase == Phase::Entry
                && let Some(side) = self.sides.get(self.ring)
            {
                self.events.push(Event::CutterCompensation {
                    side: *side,
                    plane: crate::motion::Plane::Xy,
                });
            }
            if phase == Phase::Cut {
                self.edge += 1;
                self.cuts += 1;
                if self.edge == self.cut.as_ref().map_or(0, |c| c.2.len()) {
                    self.edge = 0;
                    self.phase = Phase::LeadOut;
                    self.closed_loops += 1;
                    if !self.sides.is_empty() {
                        self.events.push(Event::CutterCompensation {
                            side: CompensationSide::Off,
                            plane: crate::motion::Plane::Xy,
                        });
                    }
                }
            }
            if phase == Phase::Rise {
                self.ring += 1;
                if self.ring == self.loops.len() {
                    self.ring = 0;
                    self.pass += 1;
                }
                if self.pass == self.passes {
                    self.phase = Phase::Done;
                }
            }
            if motion.is_some() {
                return Ok(motion);
            }
        }
    }
}
fn single(source: &PlanarRegion, component: Component) -> PlanarRegion {
    PlanarRegion {
        version: 1,
        units: "mm".into(),
        name: source.name.clone(),
        boundary_error_mm: source.boundary_error_mm,
        source: serde_json::Value::Null,
        components: vec![component],
    }
}
pub(crate) fn offset_profiles(
    source: &PreparedRegion,
    distance: f64,
    selection: Boundaries,
    reverse: bool,
    options: ConstructionOptions,
    cancel: &impl CancelSignal,
) -> Result<Profiles, String> {
    let offset = source.offset(-distance)?;
    let center = offset.region.prepare(options)?;
    let definition = source.definition();
    let count_holes = |r: &PlanarRegion| r.components.iter().map(|c| c.holes.len()).sum::<usize>();
    if (selection != Boundaries::Holes
        && definition.components.len() != center.definition().components.len())
        || (selection != Boundaries::Outer
            && count_holes(definition) != count_holes(center.definition()))
    {
        return Err("profile offset loses/splits selected topology; cutter cannot reach selected boundaries".into());
    }
    let mut originals = Vec::new();
    if selection != Boundaries::Outer {
        for (i, c) in definition.components.iter().enumerate() {
            for (j, hole) in c.holes.iter().enumerate() {
                originals.push((
                    i,
                    j,
                    single(
                        definition,
                        Component {
                            outer: hole.clone(),
                            holes: vec![],
                        },
                    )
                    .prepare(options)?,
                ));
            }
        }
    }
    let mut assigned = vec![false; originals.len()];
    let mut outer_assigned = vec![false; definition.components.len()];
    let mut loops = Vec::new();
    let mut edges = 0;
    let mut arcs = 0;
    for c in &center.definition().components {
        if cancel.is_cancelled() {
            return Err("profile ownership cancelled".into());
        }
        if selection != Boundaries::Holes {
            let owners = if definition.components.len() == 1 {
                vec![0]
            } else {
                let prepared = single(center.definition(), c.clone()).prepare(options)?;
                definition
                    .components
                    .iter()
                    .enumerate()
                    .filter_map(|(i, source)| {
                        let p = source.outer.vertices[0];
                        match prepared.query([p[0], p[1]]) {
                            Ok(q) if matches!(q.verdict, Verdict::Allowed) => Some(i),
                            _ => None,
                        }
                    })
                    .collect::<Vec<_>>()
            };
            if owners.len() != 1 || outer_assigned[owners[0]] {
                return Err("profile outer ownership is not a source bijection".into());
            }
            outer_assigned[owners[0]] = true;
            loops.push(ProfileLoop {
                strokes: Stroke::from_boundary(&c.outer),
                component: owners[0],
                hole: None,
            });
        }
        if selection != Boundaries::Outer {
            for hole in &c.holes {
                let p = hole.vertices[0];
                // A part nested in another part's hole has holes inside
                // that hole too: the owner is the innermost (smallest)
                // original hole containing the offset hole.
                let owners = originals
                    .iter()
                    .enumerate()
                    .filter_map(|(i, (_, _, h))| match h.query([p[0], p[1]]) {
                        Ok(q) if matches!(q.verdict, Verdict::Allowed) => Some(i),
                        _ => None,
                    })
                    .min_by(|a, b| {
                        originals[*a]
                            .2
                            .summary()
                            .area_mm2
                            .total_cmp(&originals[*b].2.summary().area_mm2)
                    })
                    .into_iter()
                    .collect::<Vec<_>>();
                if owners.len() != 1 || assigned[owners[0]] {
                    return Err("profile hole offset is not a source bijection".into());
                }
                assigned[owners[0]] = true;
                let (i, j, _) = &originals[owners[0]];
                loops.push(ProfileLoop {
                    strokes: Stroke::from_boundary(hole),
                    component: *i,
                    hole: Some(*j),
                });
            }
        }
    }
    if assigned.iter().any(|a| !*a)
        || (selection != Boundaries::Holes && outer_assigned.iter().any(|a| !*a))
    {
        return Err("profile selected boundary disappeared or split".into());
    }
    for ring in &mut loops {
        if reverse {
            ring.strokes = ring
                .strokes
                .iter()
                .rev()
                .copied()
                .map(Stroke::reversed)
                .collect();
        }
        edges += ring.strokes.len();
        arcs += ring
            .strokes
            .iter()
            .filter(|s| matches!(s.curve(0.), crate::motion::Curve::Arc { .. }))
            .count();
    }
    if loops.is_empty() {
        return Err("profile has no selected loops".into());
    }
    Ok(Profiles {
        loops,
        edges,
        arcs,
        diagnostics: offset.diagnostics,
    })
}

#[cfg(test)]
mod tab_position_tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;
    /// A 10 mm square part and its tool-center loop 2 mm outside (corner
    /// arcs), both counter-clockwise from the bottom left.
    fn square() -> (Vec<Stroke>, Vec<Stroke>) {
        let corners = [[0., 0.], [10., 0.], [10., 10.], [0., 10.]];
        let normals = [[0., -1.], [1., 0.], [0., 1.], [-1., 0.]];
        let part = (0..4)
            .map(|k| Stroke::line(corners[k], corners[(k + 1) % 4]))
            .collect();
        let out = |p: [f64; 2], n: [f64; 2]| [p[0] + 2. * n[0], p[1] + 2. * n[1]];
        let mut path = vec![];
        for k in 0..4 {
            let next = (k + 1) % 4;
            path.push(Stroke::line(
                out(corners[k], normals[k]),
                out(corners[next], normals[k]),
            ));
            path.push(Stroke {
                a: out(corners[next], normals[k]),
                b: out(corners[next], normals[next]),
                arc: Some((corners[next], FRAC_PI_2)),
            });
        }
        (part, path)
    }
    #[test]
    fn points_map_to_the_tool_path_beside_them_and_corners_to_the_arc_middle() {
        let (part, path) = square();
        let arc = 2. * FRAC_PI_2;
        let place = |p| {
            tab_centers_at_points(
                std::slice::from_ref(&part),
                std::slice::from_ref(&path),
                &[p],
                0.01,
                1.,
                1.,
            )
            .unwrap()[0][0]
        };
        // Bottom edge: straight below the point. Right edge: after the first
        // line and corner arc. Off the edge by less than the tolerance: the
        // nearest point of the edge.
        assert!((place([4., 0.]) - 4.).abs() < 1e-12);
        assert!((place([4., 0.008]) - 4.).abs() < 1e-12);
        assert!((place([10., 3.]) - (10. + arc + 3.)).abs() < 1e-12);
        // A convex corner is equally near its whole corner arc: the middle.
        assert!((place([10., 10.]) - (10. + arc + 10. + arc / 2.)).abs() < 1e-12);
        assert!((place([10.004, 10.004]) - (10. + arc + 10. + arc / 2.)).abs() < 1e-12);
    }
    #[test]
    fn a_point_between_two_loops_is_refused() {
        let (part, path) = square();
        let shift = |s: &[Stroke], d: f64| -> Vec<Stroke> {
            s.iter()
                .map(|s| Stroke {
                    a: [s.a[0] + d, s.a[1]],
                    b: [s.b[0] + d, s.b[1]],
                    arc: s.arc.map(|(c, w)| ([c[0] + d, c[1]], w)),
                })
                .collect()
        };
        // A second part 0.01 mm to the right of the first.
        let parts = [part.clone(), shift(&part, 10.01)];
        let paths = [path.clone(), shift(&path, 10.01)];
        let e = tab_centers_at_points(&parts, &paths, &[[10.005, 5.]], 0.01, 1., 1.).unwrap_err();
        assert!(
            e.contains("(10.005, 5) is within position_tolerance_mm 0.01 of two boundary loops"),
            "{e}"
        );
        // With a tolerance that tells them apart it belongs to the nearer.
        let c = tab_centers_at_points(&parts, &paths, &[[10.002, 5.]], 0.004, 1., 1.).unwrap();
        assert!(c[1].is_empty() && (c[0][0] - (10. + 2. * FRAC_PI_2 + 5.)).abs() < 1e-12);
        // Each loop gets the tabs whose points are on it, sorted along it.
        let c = tab_centers_at_points(
            &parts,
            &paths,
            &[[15., 0.], [8., 0.], [3., 0.]],
            0.004,
            1.,
            1.,
        )
        .unwrap();
        assert_eq!(c[0].len(), 2);
        assert!(c[0][0] < c[0][1] && c[1].len() == 1);
    }
}
