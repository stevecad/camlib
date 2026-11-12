//! Trace (engrave): the tool center follows given paths at depth.
//!
//! The usual Trace/Engrave operation of commercial CAM: no tool radius offset,
//! one or more depth passes from `top_z_mm` down to the tip depth
//! `depth_z_mm`. Paths may be open or closed and are built from lines and
//! arcs. An arc is given by its center and sweep, so its end point follows
//! exactly and no radius mismatch can occur. Linking uses the shared profile
//! motion (clearance, retract and feed heights).
//!
//! With a `side_offset` the tool centre runs beside an open path instead: to
//! the left or right of the direction of travel by the cutter radius plus an
//! allowance, the sideways compensation of commercial CAM (Autodesk Fusion,
//! "Trace milling strategy" and "Side Compensation with Trace reference":
//! "Compensation allows you to offset the toolpath to the left and right of
//! the selected contours", as on 2D Contour). The offset is exact
//! (`planar::chain`) and stops at the normals through the path's ends, as
//! FreeCAD CAM's Profile stops an open-edge profile at the ends of the edge
//! (`Path/Op/Profile.py`, `_makeStop`).
use crate::{
    motion::{Coolant, Motion, Vec3},
    pocket::{ResidualStatus, Stroke},
    profiles::{ProfileDepths, ProfileMotion, ProfileSettings},
    stock::{CancelSignal, Stock, StockSpec},
    tool::{CutterShape, ToolAssembly, valid_id},
};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum TraceSegment {
    Line {
        end_mm: [f64; 2],
    },
    /// Counter-clockwise for a positive sweep (radians), clockwise for negative.
    Arc {
        center_mm: [f64; 2],
        sweep_rad: f64,
    },
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TracePath {
    pub start_mm: [f64; 2],
    pub segments: Vec<TraceSegment>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TraceSide {
    Left,
    Right,
}

/// Keep the cutter on one side of each (open) path: the tool centre runs at
/// the cutter radius plus `radial_allowance_mm` to the `side` of the
/// direction of travel.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TraceSideOffset {
    pub side: TraceSide,
    /// Material left between the cutter and the path (not negative).
    pub radial_allowance_mm: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TraceOperation {
    pub id: String,
    pub tool: String,
    pub paths: Vec<TracePath>,
    /// Material top where the first pass starts (at most the stock top).
    pub top_z_mm: f64,
    /// Final tool tip Z.
    pub depth_z_mm: f64,
    pub stepdown_mm: f64,
    pub clearance_z_mm: f64,
    pub feed_mm_min: f64,
    pub plunge_mm_min: f64,
    pub tolerance_mm: f64,
    pub spindle_rpm: f64,
    pub work_offset: u8,
    pub tool_number: u32,
    pub length_register: u32,
    pub coolant: Coolant,
    /// Rapid travel height between paths and passes (absolute Z). Default: clearance.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub retract_z_mm: Option<f64>,
    /// Rapid descent stops here and plunge feed begins (absolute Z, above the
    /// stock top). Default: retract.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub feed_z_mm: Option<f64>,
    /// Containment and avoid areas (clamps, fixtures, features to leave alone).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub limits: Option<crate::limits::Limits>,
    /// Cutter beside the paths instead of centred on them. Default: centred.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub side_offset: Option<TraceSideOffset>,
}

#[derive(Debug, Serialize)]
pub struct TraceReport {
    pub strategy: &'static str,
    pub geometric_residual_status: ResidualStatus,
    pub paths: usize,
    pub segments: usize,
    pub closed_paths: usize,
    pub depth_passes: usize,
    pub tip_target_z_mm: f64,
    pub cut_segments_emitted: usize,
    pub machining_scope: &'static str,
    /// With a side offset: the side, and the distance from the paths to the
    /// tool centre (cutter radius plus allowance).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub side: Option<TraceSide>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub side_offset_mm: Option<f64>,
}

pub struct TracePlan {
    motion: ProfileMotion,
    report: TraceReport,
}

pub(crate) fn strokes(path: &TracePath, guard: f64) -> Result<(Vec<Stroke>, bool), String> {
    let finite = |p: [f64; 2]| p.iter().all(|v| v.is_finite() && v.abs() <= 1e6);
    if !finite(path.start_mm) {
        return Err("path start must be finite".into());
    }
    if path.segments.is_empty() {
        return Err("a path needs at least one segment".into());
    }
    let mut at = path.start_mm;
    let mut result = Vec::with_capacity(path.segments.len());
    for (i, segment) in path.segments.iter().enumerate() {
        let stroke = match *segment {
            TraceSegment::Line { end_mm } => {
                if !finite(end_mm) || (end_mm[0] - at[0]).hypot(end_mm[1] - at[1]) <= guard {
                    return Err(format!(
                        "segment {i}: line end must be finite and distinct from its start"
                    ));
                }
                Stroke::line(at, end_mm)
            }
            TraceSegment::Arc {
                center_mm,
                sweep_rad,
            } => {
                let radius = (at[0] - center_mm[0]).hypot(at[1] - center_mm[1]);
                if !finite(center_mm)
                    || !sweep_rad.is_finite()
                    || sweep_rad == 0.
                    || sweep_rad.abs() > std::f64::consts::TAU
                    || radius <= guard
                {
                    return Err(format!(
                        "segment {i}: arc needs a finite center away from its start and a sweep within (0, 2*pi] radians"
                    ));
                }
                let angle = (at[1] - center_mm[1]).atan2(at[0] - center_mm[0]) + sweep_rad;
                let end = [
                    center_mm[0] + radius * angle.cos(),
                    center_mm[1] + radius * angle.sin(),
                ];
                Stroke {
                    a: at,
                    b: end,
                    arc: Some((center_mm, sweep_rad)),
                }
            }
        };
        at = stroke.b;
        result.push(stroke);
    }
    let closed = (at[0] - path.start_mm[0]).hypot(at[1] - path.start_mm[1]) <= guard;
    if closed {
        // Close exactly onto the start (within the guard) so the loop has no gap.
        result.last_mut().unwrap().b = path.start_mm;
    }
    Ok((result, closed))
}

impl TracePlan {
    pub fn prepare(
        op: TraceOperation,
        assembly: &ToolAssembly,
        spec: &StockSpec,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        let trace_limits = op.limits.clone().filter(|l| !l.is_empty());
        let trace_tolerance = op.tolerance_mm;
        if cancel.is_cancelled() {
            return Err("trace planning cancelled".into());
        }
        spec.grid()?;
        let tool = assembly.prepare()?;
        let cutter = tool.cutter().definition();
        let guard = 16. * op.tolerance_mm;
        let values = [
            op.top_z_mm,
            op.depth_z_mm,
            op.stepdown_mm,
            op.clearance_z_mm,
            op.feed_mm_min,
            op.plunge_mm_min,
            op.tolerance_mm,
            op.spindle_rpm,
        ];
        crate::heights::require(
            "trace",
            [
                (
                    valid_id(&op.id),
                    format!("id {:?} is not a valid identifier", op.id),
                ),
                (
                    op.tool == assembly.id,
                    format!(
                        "tool {:?} does not match the assembly {:?}",
                        op.tool, assembly.id
                    ),
                ),
                (
                    cutter.center_cutting
                        && matches!(
                            cutter.shape,
                            CutterShape::Flat
                                | CutterShape::Ball
                                | CutterShape::Bull { .. }
                                | CutterShape::Chamfer { .. }
                        ),
                    "trace needs a center-cutting flat, ball, bull or chamfer (engraving) tool"
                        .into(),
                ),
                (
                    values.iter().all(|v| v.is_finite()),
                    "all numeric parameters must be finite".into(),
                ),
                (
                    (1e-7..=0.01).contains(&op.tolerance_mm),
                    format!("tolerance_mm {} must be within 1e-7..0.01", op.tolerance_mm),
                ),
                (!op.paths.is_empty(), "at least one path is needed".into()),
                (
                    op.side_offset.is_none_or(|s| {
                        s.radial_allowance_mm.is_finite()
                            && (0. ..=1e6).contains(&s.radial_allowance_mm)
                    }),
                    "side_offset radial_allowance_mm must be finite and not negative".into(),
                ),
                (
                    op.side_offset.is_none() || matches!(cutter.shape, CutterShape::Flat),
                    "a side offset needs a flat end mill (the offset is its radius); trace other cutters on their centre".into(),
                ),
                (
                    op.top_z_mm <= spec.max_mm[2],
                    format!(
                        "top_z_mm {} must not be above the stock top {}",
                        op.top_z_mm, spec.max_mm[2]
                    ),
                ),
                (
                    op.depth_z_mm < op.top_z_mm && op.depth_z_mm >= spec.min_mm[2],
                    format!(
                        "depth_z_mm {} must be below top_z_mm {} and not below the stock bottom {}",
                        op.depth_z_mm, op.top_z_mm, spec.min_mm[2]
                    ),
                ),
                (
                    spec.max_mm[2] - op.depth_z_mm <= cutter.cutting_length_mm - guard,
                    crate::reach::ReachRefusal::cutting_length(
                        &op.id,
                        &op.tool,
                        spec.max_mm[2],
                        op.depth_z_mm,
                        cutter.cutting_length_mm,
                    )
                    .error(format_args!(
                        "depth {} below the stock top exceeds the cutting length {}",
                        spec.max_mm[2] - op.depth_z_mm,
                        cutter.cutting_length_mm
                    )),
                ),
                (
                    op.stepdown_mm > guard,
                    format!("stepdown_mm {} must be positive", op.stepdown_mm),
                ),
                (
                    op.clearance_z_mm > spec.max_mm[2] + guard && op.clearance_z_mm.abs() <= 1e6,
                    format!(
                        "clearance_z_mm {} must be above the stock top {}",
                        op.clearance_z_mm, spec.max_mm[2]
                    ),
                ),
                (
                    initial.iter().all(|v| v.is_finite() && v.abs() <= 1e6),
                    "start position must be finite".into(),
                ),
                (
                    initial[2] >= op.clearance_z_mm,
                    format!(
                        "start Z {} is below clearance_z_mm {}",
                        initial[2], op.clearance_z_mm
                    ),
                ),
                (
                    op.feed_mm_min > 0. && op.plunge_mm_min > 0. && op.spindle_rpm > 0.,
                    format!(
                        "feed_mm_min {}, plunge_mm_min {} and spindle_rpm {} must be positive",
                        op.feed_mm_min, op.plunge_mm_min, op.spindle_rpm
                    ),
                ),
                (
                    (1..=9).contains(&op.work_offset),
                    format!("work_offset {} must be 1..9", op.work_offset),
                ),
                (
                    op.tool_number > 0 && op.length_register > 0,
                    "tool_number and length_register must be positive".into(),
                ),
                (
                    first_id != 0 && first_id.checked_add(1).is_some(),
                    "motion ID budget exhausted".into(),
                ),
            ],
        )?;
        let heights = crate::heights::Heights::resolve(
            op.clearance_z_mm,
            op.retract_z_mm,
            op.feed_z_mm,
            spec.max_mm[2],
            guard,
        )?;
        let side_offset = op.side_offset.map(|s| {
            (
                s.side,
                assembly.cutter.diameter_mm / 2. + s.radial_allowance_mm,
            )
        });
        let mut loops = Vec::with_capacity(op.paths.len());
        let (mut segments, mut closed_paths) = (0, 0);
        for (i, path) in op.paths.iter().enumerate() {
            if cancel.is_cancelled() {
                return Err("trace planning cancelled".into());
            }
            let (mut path_strokes, closed) = strokes(path, guard)
                .map_err(|e| format!("invalid trace parameters: path {i}, {e}"))?;
            if let Some((side, distance)) = side_offset {
                if closed {
                    return Err(format!(
                        "invalid trace parameters: path {i} is closed; a side offset is for open paths: use Contour for a closed profile, or leave the path open"
                    ));
                }
                let signed = match side {
                    TraceSide::Left => distance,
                    TraceSide::Right => -distance,
                };
                path_strokes = crate::planar::chain::offset_open_chain(
                    &path_strokes,
                    signed,
                    (op.tolerance_mm / 32.).max(1e-9),
                    guard,
                )
                .map_err(|e| format!("invalid trace parameters: path {i}, {e}"))?;
            }
            for stroke in &path_strokes {
                stroke
                    .curve(op.depth_z_mm)
                    .prepare(op.tolerance_mm)
                    .map_err(|e| format!("invalid trace parameters: path {i}: {e}"))?;
            }
            segments += path_strokes.len();
            closed_paths += usize::from(closed);
            loops.push(path_strokes);
        }
        let passes = ((op.top_z_mm - op.depth_z_mm) / op.stepdown_mm).ceil() as usize;
        let report = TraceReport {
            strategy: "trace",
            geometric_residual_status: ResidualStatus::Pending,
            paths: loops.len(),
            segments,
            closed_paths,
            depth_passes: passes,
            tip_target_z_mm: op.depth_z_mm,
            cut_segments_emitted: 0,
            machining_scope: if side_offset.is_some() {
                "tool center beside the given open paths at the cutter radius plus allowance; part protection only through configured checks"
            } else {
                "tool center on the given paths, no radius offset; part protection only through configured checks"
            },
            side: side_offset.map(|s| s.0),
            side_offset_mm: side_offset.map(|s| s.1),
        };
        let settings = ProfileSettings {
            id: op.id,
            tool: op.tool,
            heights,
            feed: op.feed_mm_min,
            plunge: op.plunge_mm_min,
            tolerance: op.tolerance_mm,
            lead_in: 0.,
            lead_out: 0.,
            spindle: op.spindle_rpm,
            work_offset: op.work_offset,
            tool_number: op.tool_number,
            length_register: op.length_register,
            coolant: op.coolant,
        };
        let depths = ProfileDepths::Linear {
            top: op.top_z_mm,
            target: op.depth_z_mm,
            step: op.stepdown_mm,
        };
        let mut motion =
            ProfileMotion::new(settings, spec, initial, loops, depths, passes, first_id);
        if let Some(limits) = trace_limits {
            motion.limits = Some(crate::limits::PreparedLimits::new(
                &limits,
                assembly.cutter.diameter_mm / 2.,
                trace_tolerance,
            )?);
        }
        Ok(Self { motion, report })
    }
    pub fn report(&self) -> &TraceReport {
        &self.report
    }
    pub fn prefix_events(&self) -> Vec<crate::motion::Event> {
        self.motion.prefix_events()
    }
    pub fn suffix_events(&self) -> Vec<crate::motion::Event> {
        self.motion.suffix_events()
    }
    pub fn finish_report(&mut self, cancel: &impl CancelSignal) -> Result<(), String> {
        if cancel.is_cancelled() {
            return Err("trace report cancelled".into());
        }
        if !self.motion.drained {
            return Err("drain committed trace motions first".into());
        }
        self.report.geometric_residual_status = ResidualStatus::Unknown;
        Ok(())
    }
    pub fn next_motion_with_stock(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        let motion = self.motion.next(stock, cancel)?;
        self.report.cut_segments_emitted = self.motion.cuts;
        Ok(motion)
    }
}
