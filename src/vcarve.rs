//! V-carve: a V (chamfer) bit carves a region with sharp walls. For each
//! point of the region's outline, the largest disk inside the region that
//! touches it there is found (bisection on the region's signed clearance);
//! the bit's axis goes to that disk's center, deep enough that its cone is as
//! wide as the disk at the top surface. Every point of the region lies in
//! such a disk, so the bit clears the region and reproduces its corners:
//! convex corners run out to the surface, concave corners are swept around.
//! Between outline samples the carve is checked against the outline and the
//! samples refined where it would cross it.
use crate::{
    Point,
    motion::{Coolant, Curve, Event, Motion, MoveKind, Vec3},
    planar::{ConstructionOptions, PlanarRegion},
    pocket::{ResidualStatus, Stroke},
    stock::{CancelSignal, Stock, StockSpec},
    tool::{CutterShape, ToolAssembly, valid_id},
};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VCarveOperation {
    pub id: String,
    pub tool: String,
    /// The region carved (letters, a logo).
    pub region: PlanarRegion,
    /// The surface carved into.
    pub top_z_mm: f64,
    /// Deepest cut allowed; None: as deep as the region needs.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_depth_mm: Option<f64>,
    /// Outline sample spacing.
    pub sample_mm: f64,
    pub clearance_z_mm: f64,
    pub feed_mm_min: f64,
    pub plunge_mm_min: f64,
    pub tolerance_mm: f64,
    pub spindle_rpm: f64,
    pub work_offset: u8,
    pub tool_number: u32,
    pub length_register: u32,
    pub coolant: Coolant,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub retract_z_mm: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub feed_z_mm: Option<f64>,
    /// Containment and avoid areas (clamps, fixtures, features to leave alone).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub limits: Option<crate::limits::Limits>,
}

#[derive(Debug, Serialize)]
pub struct VCarveReport {
    pub strategy: &'static str,
    pub geometric_residual_status: ResidualStatus,
    pub loops: usize,
    pub samples: usize,
    pub refined_samples: usize,
    pub max_depth_mm: f64,
    /// Outline length where the region is narrower than the bit's tip (left
    /// uncarved there).
    pub narrower_than_tip_mm: f64,
    pub completed_moves: usize,
    pub machining_scope: &'static str,
}

pub struct VCarvePlan {
    op: VCarveOperation,
    spec: StockSpec,
    initial: Vec3,
    position: Vec3,
    moves: VecDeque<(Vec3, MoveKind)>,
    next_id: u64,
    expected_generation: Option<u64>,
    drained: bool,
    report: VCarveReport,
}

/// A point of an outline loop: position, inward normal (interior on the left).
fn sample(s: &Stroke, t: f64) -> (Point, Point) {
    let p = crate::profiles::stroke_point(s, t);
    let d = match s.arc {
        None => [s.b[0] - s.a[0], s.b[1] - s.a[1]],
        Some((c, sweep)) => {
            let sign = sweep.signum();
            [-(p[1] - c[1]) * sign, (p[0] - c[0]) * sign]
        }
    };
    let n = metallic::hypot(d[0], d[1]);
    (p, [-d[1] / n, d[0] / n])
}

impl VCarvePlan {
    pub fn prepare(
        mut op: VCarveOperation,
        assembly: &ToolAssembly,
        spec: &StockSpec,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        let tool = assembly.prepare()?;
        let cutter = tool.cutter().definition();
        let CutterShape::Chamfer {
            included_angle_rad,
            tip_diameter_mm,
        } = cutter.shape
        else {
            return Err("v-carve needs a V (chamfer) bit".into());
        };
        let slope = metallic::tan(included_angle_rad / 2.);
        let tip = tip_diameter_mm / 2.;
        let guard = 16. * op.tolerance_mm;
        crate::heights::require(
            "v-carve",
            [
                (
                    valid_id(&op.id) && op.tool == assembly.id,
                    "invalid id or tool".into(),
                ),
                (
                    [
                        op.top_z_mm,
                        op.sample_mm,
                        op.clearance_z_mm,
                        op.feed_mm_min,
                        op.plunge_mm_min,
                    ]
                    .iter()
                    .all(|v| v.is_finite()),
                    "all numeric parameters must be finite".into(),
                ),
                (
                    op.top_z_mm <= spec.max_mm[2] + guard && op.top_z_mm > spec.min_mm[2],
                    format!("top_z_mm {} must be within the stock", op.top_z_mm),
                ),
                (
                    op.sample_mm > guard && op.sample_mm <= 1.,
                    format!("sample_mm {} must be within (0, 1]", op.sample_mm),
                ),
                (
                    (1e-7..=0.01).contains(&op.tolerance_mm),
                    format!("tolerance_mm {} must be within 1e-7..0.01", op.tolerance_mm),
                ),
                (
                    op.max_depth_mm.is_none_or(|d| d.is_finite() && d > guard),
                    "max_depth_mm must be positive".into(),
                ),
                (
                    op.feed_mm_min > 0. && op.plunge_mm_min > 0. && op.spindle_rpm > 0.,
                    "feeds and spindle speed must be positive".into(),
                ),
            ],
        )?;
        if let Some(limits) = op.limits.take().filter(|l| !l.is_empty()) {
            let floor = op.top_z_mm
                - op.max_depth_mm
                    .unwrap_or(f64::INFINITY)
                    .min(cutter.cutting_length_mm - guard);
            op.region = limits.restrict_region(&op.region, floor, op.tolerance_mm)?;
        }
        let heights = crate::heights::Heights::resolve(
            op.clearance_z_mm,
            op.retract_z_mm,
            op.feed_z_mm,
            spec.max_mm[2].max(op.top_z_mm),
            guard,
        )?;
        let options = ConstructionOptions {
            tolerance_mm: op.tolerance_mm,
        };
        let region = op.region.prepare(options)?;
        // Deepest the bit may go: the cutting length, and the asked limit.
        let deepest = op
            .max_depth_mm
            .unwrap_or(f64::INFINITY)
            .min(cutter.cutting_length_mm - guard);
        let reach = tip + deepest * slope;
        let depth_of = |r: f64| ((r - tip) / slope).max(0.);
        // The largest inscribed disk touching outline point p (normal n).
        let check = op.tolerance_mm.max(1e-6);
        let radius_at = |p: Point, n: Point| -> Result<f64, String> {
            let fits = |r: f64| -> Result<bool, String> {
                let c = [p[0] + r * n[0], p[1] + r * n[1]];
                let q = region.query(c)?;
                Ok(q.signed_clearance_mm.unwrap_or(f64::NEG_INFINITY) >= r - check)
            };
            let (mut lo, mut hi) = (0., reach.min(1e4));
            if fits(hi)? {
                return Ok(hi);
            }
            for _ in 0..48 {
                let m = (lo + hi) / 2.;
                if fits(m)? {
                    lo = m;
                } else {
                    hi = m;
                }
            }
            Ok(lo)
        };
        let mut report = VCarveReport {
            strategy: "vcarve",
            geometric_residual_status: ResidualStatus::Pending,
            loops: 0,
            samples: 0,
            refined_samples: 0,
            max_depth_mm: 0.,
            narrower_than_tip_mm: 0.,
            completed_moves: 0,
            machining_scope: "maximal inscribed disks along the outline; cone width at the top surface equals the disk",
        };
        let mut moves: VecDeque<(Vec3, MoveKind)> = VecDeque::new();
        let mut position = initial;
        let mut first = true;
        let push = |moves: &mut VecDeque<(Vec3, MoveKind)>, at: &mut Vec3, end: Vec3, kind| {
            if end != *at {
                moves.push_back((end, kind));
                *at = end;
            }
        };
        for c in &region.definition().components {
            for boundary in std::iter::once(&c.outer).chain(&c.holes) {
                if cancel.is_cancelled() {
                    return Err("v-carve planning cancelled".into());
                }
                let strokes = Stroke::from_boundary(boundary);
                // Outline samples (position, normal), with corner fans.
                let mut outline: Vec<(Point, Point)> = vec![];
                for (k, s) in strokes.iter().enumerate() {
                    let prev = strokes[(k + strokes.len() - 1) % strokes.len()];
                    let (_, n_in) = sample(&prev, 1.);
                    let (p0, n_out) = sample(s, 0.);
                    let turn = metallic::atan2(
                        n_in[0] * n_out[1] - n_in[1] * n_out[0],
                        n_in[0] * n_out[0] + n_in[1] * n_out[1],
                    );
                    if turn > 1e-9 {
                        // Convex corner: the bit runs out to the surface.
                        outline.push((p0, [0., 0.]));
                    } else if turn < -1e-9 {
                        // Concave corner: sweep around it.
                        let steps = (turn.abs() / 5f64.to_radians()).ceil() as usize;
                        let a0 = metallic::atan2(n_in[1], n_in[0]);
                        for j in 0..steps {
                            let a = a0 + turn * j as f64 / steps as f64;
                            outline.push((p0, [metallic::cos(a), metallic::sin(a)]));
                        }
                    }
                    let length = crate::profiles::stroke_length(s);
                    let n = (length / op.sample_mm).ceil().max(1.) as usize;
                    for j in 0..n {
                        outline.push(sample(s, j as f64 / n as f64));
                    }
                }
                // Carve centers; refine between samples whose midpoint carve
                // would cross the outline.
                let mut points: Vec<Vec3> = vec![];
                let center = |p: Point, n: Point| -> Result<(Point, f64), String> {
                    let r = if n == [0., 0.] { 0. } else { radius_at(p, n)? };
                    Ok(([p[0] + r * n[0], p[1] + r * n[1]], r))
                };
                let mut carve = vec![];
                for &(p, n) in &outline {
                    carve.push(center(p, n)?);
                }
                for i in 0..carve.len() {
                    let (c0, r0) = carve[i];
                    let (c1, r1) = carve[(i + 1) % carve.len()];
                    points.push([c0[0], c0[1], op.top_z_mm - depth_of(r0)]);
                    report.max_depth_mm = report.max_depth_mm.max(depth_of(r0));
                    if r0 < tip {
                        report.narrower_than_tip_mm += op.sample_mm;
                    }
                    // The cone along the chord is within the disks swept
                    // between the ends if the middle's disk fits.
                    let mut stack = vec![(c0, r0, c1, r1, 0u32)];
                    let mut extra = vec![];
                    while let Some((a, ra, b, rb, depth)) = stack.pop() {
                        let m = [(a[0] + b[0]) / 2., (a[1] + b[1]) / 2.];
                        let rm = (ra + rb) / 2.;
                        let q = region.query(m)?;
                        if q.signed_clearance_mm.unwrap_or(f64::NEG_INFINITY) >= rm - check
                            || depth >= 12
                        {
                            continue;
                        }
                        // Shrink the middle to the disk that fits there.
                        let fit = q.signed_clearance_mm.unwrap_or(0.).max(0.);
                        extra.push((m, fit));
                        stack.push((m, fit, b, rb, depth + 1));
                        stack.push((a, ra, m, fit, depth + 1));
                        report.refined_samples += 1;
                    }
                    // Order the refinements along the chord.
                    extra.sort_by(|x, y| {
                        let d = |p: Point| metallic::hypot(p[0] - c0[0], p[1] - c0[1]);
                        d(x.0).total_cmp(&d(y.0))
                    });
                    for (m, r) in extra {
                        points.push([m[0], m[1], op.top_z_mm - depth_of(r)]);
                    }
                }
                if points.is_empty() {
                    continue;
                }
                points.push(points[0]);
                report.samples += points.len();
                report.loops += 1;
                // Up to travel height, over the first carve point, down at
                // rapid to the feed height, plunge, carve the loop, leave.
                let travel = if first {
                    heights.clearance
                } else {
                    heights.retract
                };
                first = false;
                let start = points[0];
                let up = [position[0], position[1], travel.max(position[2])];
                push(&mut moves, &mut position, up, MoveKind::Rapid);
                let over = [start[0], start[1], position[2]];
                push(&mut moves, &mut position, over, MoveKind::Rapid);
                push(
                    &mut moves,
                    &mut position,
                    [start[0], start[1], heights.feed],
                    MoveKind::Rapid,
                );
                push(&mut moves, &mut position, start, MoveKind::Plunge);
                for p in &points[1..] {
                    push(&mut moves, &mut position, *p, MoveKind::Cut);
                }
                let leave = [position[0], position[1], heights.feed];
                push(&mut moves, &mut position, leave, MoveKind::Retract);
            }
        }
        if report.loops == 0 {
            return Err("v-carve region has no outline".into());
        }
        if op
            .max_depth_mm
            .is_some_and(|d| report.max_depth_mm >= d - guard)
        {
            return Err(format!(
                "the region is wider than the bit can carve within max_depth_mm {}: its middle needs a flat clearing tool (not supported yet)",
                op.max_depth_mm.unwrap()
            ));
        }
        let end = [position[0], position[1], heights.clearance];
        push(&mut moves, &mut position, end, MoveKind::Rapid);
        crate::resources::check_memory(moves.len() as u128 * 40, "v-carve moves")?;
        Ok(Self {
            op,
            spec: spec.clone(),
            initial,
            position: initial,
            moves,
            next_id: first_id,
            expected_generation: None,
            drained: false,
            report,
        })
    }
    pub fn report(&self) -> &VCarveReport {
        &self.report
    }
    pub fn prefix_events(&self) -> Vec<Event> {
        use crate::motion::SpindleDirection;
        vec![
            Event::WorkOffset {
                number: self.op.work_offset,
            },
            Event::ToolChange {
                tool: self.op.tool.clone(),
                number: self.op.tool_number,
                length_register: self.op.length_register,
                position_after_mm: self.initial,
            },
            Event::Spindle {
                rpm: self.op.spindle_rpm,
                direction: SpindleDirection::Cw,
            },
            Event::Coolant {
                mode: self.op.coolant,
            },
        ]
    }
    pub fn suffix_events(&self) -> Vec<Event> {
        use crate::motion::SpindleDirection;
        vec![
            Event::Coolant { mode: Coolant::Off },
            Event::Spindle {
                rpm: 0.,
                direction: SpindleDirection::Cw,
            },
        ]
    }
    pub fn finish_report(&mut self, cancel: &impl CancelSignal) -> Result<(), String> {
        if cancel.is_cancelled() {
            return Err("v-carve report cancelled".into());
        }
        if !self.drained {
            return Err("drain committed v-carve motions".into());
        }
        self.report.geometric_residual_status = ResidualStatus::Estimated;
        Ok(())
    }
    pub fn next_motion_with_stock(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        if stock.spec() != &self.spec
            || self
                .expected_generation
                .is_some_and(|g| g != stock.generation())
        {
            return Err("commit preceding v-carve motion to matching stock".into());
        }
        if cancel.is_cancelled() {
            return Err("v-carve cancelled".into());
        }
        let Some((end, kind)) = self.moves.pop_front() else {
            self.drained = true;
            return Ok(None);
        };
        let curve = Curve::Line {
            start: self.position,
            end,
        };
        curve.prepare(self.op.tolerance_mm)?;
        let m = Motion {
            id: self.next_id,
            operation: self.op.id.clone(),
            tool: self.op.tool.clone(),
            curve,
            kind,
            feed_mm_min: match kind {
                MoveKind::Rapid => None,
                MoveKind::Plunge => Some(self.op.plunge_mm_min),
                _ => Some(self.op.feed_mm_min),
            },
        };
        self.position = end;
        self.next_id += 1;
        self.expected_generation = Some(stock.generation() + 1);
        self.report.completed_moves += 1;
        Ok(Some(m))
    }
}
