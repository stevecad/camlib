//! Thermal profile cutting (plasma, laser) of sheet parts.
//!
//! Each part (a component of `parts`: an outer boundary with holes) is cut
//! out along its kerf-compensated profile: the beam center runs half a kerf
//! (plus the boundary margin) outside the part. A loop is entered from a
//! pierce point in the scrap over an optional straight lead and a tangent arc
//! lead, cut once around (plus an optional overcut), and left over a tangent
//! arc and straight lead into the scrap again.
//!
//! Order: holes of a part before its outline, and parts that lie inside a
//! hole of another part before that hole, so nothing is cut free before its
//! inner features. Every lead and cut is proven not to touch any part, and
//! every pierce point to keep `pierce_clearance_mm` from every part.
//!
//! With `common_line`, parts exactly one kerf path apart share one cut along
//! their common straight edge (`common_line.rs`).
use crate::{
    contour::{Boundaries, Compensation},
    motion::{CompensationSide, Curve, Event, Motion, MoveKind, Plane, Vec3},
    planar::{Component, ConstructionOptions, PlanarRegion},
    pocket::{ResidualStatus, Stroke},
    stock::{CancelSignal, Stock, StockSpec},
    tool::{Cutter, CutterShape, ToolAssembly, valid_id},
    verification,
};
use serde::{Deserialize, Serialize};
use std::f64::consts::{FRAC_PI_2, PI};

mod common_line;
mod corners;
mod lead_placement;
mod tip_up;
pub use common_line::{CommonLine, CommonLineReport};
pub use corners::{CornerLoops, CornerSlowdown, loop_reach_mm};
pub use lead_placement::{LeadPlacement, LeadPreference};
pub use tip_up::AvoidCutParts;

/// How the torch or head height is controlled.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HeightMode {
    /// The program holds one Z (the cut height); the controller or a fixed
    /// focus handles pierce height and height control (QtPlasmaC, lasers).
    #[default]
    Fixed,
    /// The program moves Z: travel height, down to the pierce height, beam
    /// on, pierce delay, down to the cut height at the plunge feed.
    Program,
}

/// A lead: a straight part of `length_mm` and a tangent arc of `radius_mm`
/// (either may be zero), always on the scrap side.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Lead {
    #[serde(default)]
    pub length_mm: f64,
    #[serde(default)]
    pub radius_mm: f64,
}

/// Small holes: a reduced feed and, by default, height control off
/// (Hypertherm hole-quality guidance: slower speed, THC locked out).
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SmallHoles {
    /// Holes up to this diameter (by their perimeter) count as small.
    /// Default: 1.5 x the sheet thickness (Hypertherm's definition).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_diameter_mm: Option<f64>,
    /// Feed in small holes, percent of `feed_mm_min`.
    pub feed_percent: f64,
    #[serde(default = "yes")]
    pub height_control_off: bool,
}
fn yes() -> bool {
    true
}
fn full_power() -> f64 {
    100.
}
fn boundary_tolerance() -> f64 {
    0.01
}

/// Micro-joints: short uncut bridges that keep a part in the sheet. The
/// beam turns off over each joint and fires again after it (no pierce
/// clearance there: suited to lasers and thin plasma work).
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MicroJoints {
    /// Uncut width along the part edge.
    pub width_mm: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub count: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub spacing_mm: Option<f64>,
    /// Also on holes (default: outlines only).
    #[serde(default)]
    pub holes: bool,
}

/// Which part is cut next. Whatever the choice, a part nested in another
/// part's hole is cut before that part, a part's holes before its outline,
/// and parts joined by common lines as one group: only the order of parts
/// at the same nesting depth is chosen here.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum PartOrder {
    /// The part whose outline starts nearest to where the last loop
    /// started: the shortest travel, and the most heat in one place.
    #[default]
    Nearest,
    /// The parts in the order they are given in `parts`.
    Given,
    /// Spread the heat: the nearest part whose outline starts at least
    /// `min_distance_mm` from where the last loop started, so consecutive
    /// parts are not neighbours; when every remaining part is nearer than
    /// that, the farthest of them. The distance is the shop's (plate
    /// thickness, material and how much it moves); none is assumed.
    Spread { min_distance_mm: f64 },
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ThermalCutOperation {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub limits: Option<crate::limits::Limits>,
    pub id: String,
    /// A beam tool: its cutter diameter is the kerf width.
    pub tool: String,
    /// The parts to keep, each an outer boundary with holes.
    pub parts: PlanarRegion,
    pub feed_mm_min: f64,
    /// Nozzle (or focus) height above the sheet top while cutting.
    pub cut_height_mm: f64,
    #[serde(default)]
    pub height_mode: HeightMode,
    /// Program height mode: pierce and travel heights above the sheet top,
    /// and the feed from the pierce height down to the cut height.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pierce_height_mm: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub travel_height_mm: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub plunge_mm_min: Option<f64>,
    /// Dwell after the beam turns on, before moving.
    #[serde(default)]
    pub pierce_delay_s: f64,
    /// Beam power while cutting (laser power, percent).
    #[serde(default = "full_power")]
    pub power_percent: f64,
    pub lead_in: Lead,
    pub lead_out: Lead,
    /// Keep cutting this far past the loop start before the lead-out; a
    /// negative value starts the lead-out that far before the loop closes
    /// (negative overburn, recommended for holes with an arc lead-in).
    #[serde(default)]
    pub overcut_mm: f64,
    /// Smallest distance from a pierce point to any part, beyond half the
    /// kerf (the pierce crater). Default: the kerf width.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pierce_clearance_mm: Option<f64>,
    /// Torch height control on while cutting each loop (off over leads).
    #[serde(default)]
    pub height_control: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub small_holes: Option<SmallHoles>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub micro_joints: Option<MicroJoints>,
    /// Cut the shared straight edge of two parts placed exactly one kerf
    /// path apart once (see `thermal/common_line.rs`). Off unless set.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub common_line: Option<CommonLine>,
    /// Default: outlines clockwise and holes counter-clockwise (the part on
    /// the right of travel, the good side of a plasma arc with clockwise
    /// swirl). True reverses both.
    #[serde(default)]
    pub reverse: bool,
    /// Which part is cut next (default: the nearest).
    #[serde(default)]
    pub part_order: PartOrder,
    /// Keep rapids clear of parts and slugs already cut free, which can
    /// tip up between the slats (see `thermal/tip_up.rs`). Off unless set.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub avoid_cut_parts: Option<AvoidCutParts>,
    /// Edges that must not carry a lead, and where leads are preferred
    /// (see `thermal/lead_placement.rs`). Off unless set.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lead_placement: Option<LeadPlacement>,
    /// Loops into the scrap at the parts' outside corners, so both edges
    /// are cut straight through the corner (see `thermal/corners.rs`). Off
    /// unless set.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub corner_loops: Option<CornerLoops>,
    /// A reduced feed either side of corners (see `thermal/corners.rs`).
    /// Off unless set.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub corner_slowdown: Option<CornerSlowdown>,
    /// A last cut that frees the usable offcut from the skeleton. Off unless
    /// set.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub crop: Option<CropCut>,
    /// "wear": the same beam-center path posted with G41/G42 so the operator
    /// can trim the kerf at the controller (turned on over the straight
    /// lead-in, off over the straight lead-out).
    #[serde(default)]
    pub compensation: Compensation,
    #[serde(default = "boundary_tolerance")]
    pub boundary_tolerance_mm: f64,
    pub tolerance_mm: f64,
    pub work_offset: u8,
    pub tool_number: u32,
    pub length_register: u32,
}

/// The crop cut: a straight cut (two points) or an L (three), in the
/// sheet's coordinates, made after every part is cut. The beam is pierced at
/// the first point and follows the points at the cut's feed and height, with
/// no leads and no kerf offset: the points are the kerf's centre line.
/// Nothing is assumed about the sheet's edges: the caller places the ends
/// (on the edge, or past it by the run-off the shop uses).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CropCut {
    pub points_mm: Vec<[f64; 2]>,
}

#[derive(Debug, Serialize)]
pub struct ThermalCutReport {
    pub strategy: &'static str,
    pub geometric_residual_status: ResidualStatus,
    pub kerf_mm: f64,
    pub applied_offset_mm: f64,
    pub parts: usize,
    pub loops: usize,
    pub small_holes: usize,
    pub pierces: usize,
    pub micro_joints: usize,
    pub cut_length_mm: f64,
    /// Loop order: (part index, hole index or None for the outline).
    pub order: Vec<(usize, Option<usize>)>,
    pub part_order: PartOrder,
    /// The least distance between the starts of two parts cut one after the
    /// other (absent with fewer than two parts).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub min_part_step_mm: Option<f64>,
    /// Travel between cuts with the beam off, in the sheet's plane.
    pub rapid_length_mm: f64,
    /// Loops started somewhere other than their own start to satisfy
    /// `lead_placement`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub leads_moved: Option<usize>,
    /// Loops cut at corners (`corner_loops`); their length is in
    /// `cut_length_mm`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub corner_loops: Option<usize>,
    /// Stretches cut at the reduced feed (`corner_slowdown`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub corner_slowdowns: Option<usize>,
    /// The crop cut's length (in `cut_length_mm` too; its pierce is one of
    /// `pierces`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub crop_length_mm: Option<f64>,
    /// Every fed move (plunges, leads, cuts) at its programmed feed, in
    /// seconds: nominal, without acceleration or the controller's own
    /// corner and height-control behaviour.
    pub feed_seconds: f64,
    /// The pierce delays the program dwells for, in seconds.
    pub dwell_seconds: f64,
    /// Rapids taken round parts already cut free (`avoid_cut_parts`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rerouted_rapids: Option<usize>,
    pub loops_completed: usize,
    /// The common lines, in cutting order (empty unless `common_line`).
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub common_lines: Vec<CommonLineReport>,
    pub diagnostics: Vec<String>,
}

enum Step {
    Move(Curve, MoveKind, Option<f64>),
    Event(Event),
    /// A loop is complete (for the report).
    Done,
}

pub struct ThermalCutPlan {
    id: String,
    tool: String,
    work_offset: u8,
    tool_number: u32,
    length_register: u32,
    initial: Vec3,
    spec: StockSpec,
    steps: std::collections::VecDeque<Step>,
    events: Vec<Event>,
    next_id: u64,
    expected_generation: Option<u64>,
    drained: bool,
    tolerance: f64,
    report: ThermalCutReport,
}

/// The point at length fraction t of an XY line or arc.
fn point_at(c: &Curve, t: f64) -> Vec3 {
    if t <= 0. {
        return c.start();
    }
    if t >= 1. {
        return c.end();
    }
    match *c {
        Curve::Line { start, end } => std::array::from_fn(|i| start[i] + t * (end[i] - start[i])),
        Curve::Arc {
            start,
            center,
            sweep_rad,
            ..
        }
        | Curve::Helix {
            start,
            center,
            sweep_rad,
            ..
        } => {
            let r = (start[0] - center[0]).hypot(start[1] - center[1]);
            let a = (start[1] - center[1]).atan2(start[0] - center[0]) + t * sweep_rad;
            [center[0] + r * a.cos(), center[1] + r * a.sin(), start[2]]
        }
    }
}
/// The part of an XY line or arc between length fractions t0 and t1.
fn part_of(c: &Curve, t0: f64, t1: f64) -> Curve {
    let (start, end) = (point_at(c, t0), point_at(c, t1));
    match *c {
        Curve::Line { .. } => Curve::Line { start, end },
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
        } => Curve::Arc {
            start,
            end,
            center,
            plane,
            sweep_rad: sweep_rad * (t1 - t0),
        },
    }
}

fn rot_left(v: [f64; 2]) -> [f64; 2] {
    [-v[1], v[0]]
}
fn add(p: [f64; 2], v: [f64; 2], k: f64) -> [f64; 2] {
    [p[0] + k * v[0], p[1] + k * v[1]]
}

/// A flat cylinder tool used only to prove clearance from the parts.
fn probe_tool(diameter: f64, reach: f64) -> ToolAssembly {
    ToolAssembly {
        id: "kerf-probe".into(),
        cutter: Cutter {
            id: "kerf-probe".into(),
            name: String::new(),
            shape: CutterShape::Flat,
            diameter_mm: diameter,
            cutting_length_mm: reach,
            overall_length_mm: reach + 4.,
            shank_diameter_mm: diameter,
            flutes: 1,
            center_cutting: true,
            material: String::new(),
            neck: None,
        },
        stickout_mm: reach + 1.,
        gauge_length_mm: reach + 3.,
        // The probe's holder is the same cylinder: anything above the
        // cutting length is still the kerf (no wider body is claimed).
        holders: vec![crate::tool::HolderSection {
            z_min_mm: reach + 1.,
            z_max_mm: reach + 3.,
            radius_min_mm: diameter / 2.,
            radius_max_mm: diameter / 2.,
        }],
    }
}

impl ThermalCutPlan {
    pub fn prepare(
        op: ThermalCutOperation,
        assembly: &ToolAssembly,
        spec: &StockSpec,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        if cancel.is_cancelled() {
            return Err("thermal cut planning cancelled".into());
        }
        spec.grid()?;
        let tool = assembly.prepare()?;
        let cutter = tool.cutter().definition();
        let kerf = cutter.diameter_mm;
        let top = spec.max_mm[2];
        let guard = 16. * op.tolerance_mm;
        let program = op.height_mode == HeightMode::Program;
        let pierce_h = op.pierce_height_mm.unwrap_or(op.cut_height_mm);
        let travel_h = op.travel_height_mm.unwrap_or(pierce_h);
        let lead_ok = |l: &Lead| {
            l.length_mm.is_finite()
                && l.radius_mm.is_finite()
                && l.length_mm >= 0.
                && l.radius_mm >= 0.
        };
        crate::heights::require(
            "thermal cut",
            [
                (valid_id(&op.id), format!("id {:?} is not a valid identifier", op.id)),
                (
                    op.tool == assembly.id,
                    format!("tool {:?} does not match the assembly {:?}", op.tool, assembly.id),
                ),
                (
                    matches!(cutter.shape, CutterShape::Beam),
                    "thermal cutting needs a beam tool (camlib.tools.Torch or LaserHead): its diameter is the kerf".into(),
                ),
                (
                    (1e-7..=0.01).contains(&op.tolerance_mm)
                        && (0.0001..=0.1).contains(&op.boundary_tolerance_mm)
                        && op.tolerance_mm <= op.boundary_tolerance_mm / 64.,
                    "tolerance_mm must be within 1e-7..0.01 and at most boundary_tolerance_mm (0.0001..0.1) / 64".into(),
                ),
                (
                    op.feed_mm_min.is_finite() && op.feed_mm_min > 0.,
                    format!("feed_mm_min {} must be positive", op.feed_mm_min),
                ),
                (
                    op.cut_height_mm.is_finite() && op.cut_height_mm > guard && op.cut_height_mm <= 1000.,
                    format!("cut_height_mm {} must be positive (the nozzle stays above the sheet)", op.cut_height_mm),
                ),
                (
                    program || (op.pierce_height_mm.is_none() && op.travel_height_mm.is_none() && op.plunge_mm_min.is_none()),
                    "pierce_height_mm, travel_height_mm and plunge_mm_min need height_mode=\"program\" (the controller sets them in fixed mode)".into(),
                ),
                (
                    !program
                        || (pierce_h.is_finite()
                            && travel_h.is_finite()
                            && op.cut_height_mm <= pierce_h
                            && pierce_h <= travel_h
                            && travel_h <= 1000.
                            && op.plunge_mm_min.is_some_and(|f| f.is_finite() && f > 0.)),
                    "program heights need cut_height_mm <= pierce_height_mm <= travel_height_mm and a positive plunge_mm_min".into(),
                ),
                (
                    op.pierce_delay_s.is_finite() && (0. ..=1e3).contains(&op.pierce_delay_s),
                    format!("pierce_delay_s {} must be 0..1000", op.pierce_delay_s),
                ),
                (
                    op.power_percent.is_finite() && op.power_percent > 0. && op.power_percent <= 100.,
                    format!("power_percent {} must be in (0, 100]", op.power_percent),
                ),
                (
                    lead_ok(&op.lead_in) && lead_ok(&op.lead_out) && op.overcut_mm.is_finite(),
                    "lead lengths and radii must not be negative".into(),
                ),
                (
                    op.compensation == Compensation::Computer
                        || (op.lead_in.length_mm > guard && op.lead_out.length_mm > guard),
                    "wear compensation turns G41/G42 on over the straight lead-in and off over the straight lead-out: set both lead length_mm".into(),
                ),
                (
                    op.pierce_clearance_mm.is_none_or(|c| c.is_finite() && c >= 0.),
                    "pierce_clearance_mm must not be negative".into(),
                ),
                (
                    op.small_holes.is_none_or(|s| {
                        s.max_diameter_mm.is_none_or(|d| d.is_finite() && d > 0.)
                            && s.feed_percent.is_finite()
                            && s.feed_percent > 0.
                            && s.feed_percent <= 100.
                    }),
                    "small_holes needs a positive max_diameter_mm and feed_percent in (0, 100]".into(),
                ),
                (
                    op.crop.as_ref().is_none_or(|c| {
                        (2..=3).contains(&c.points_mm.len())
                            && c.points_mm.iter().flatten().all(|v| v.is_finite() && v.abs() <= 1e6)
                            && c.points_mm.windows(2).all(|w| {
                                (w[0][0] - w[1][0]).hypot(w[0][1] - w[1][1]) > kerf
                            })
                    }),
                    "crop needs two points (a straight cut) or three (an L), each more than a kerf from the next".into(),
                ),
                (
                    op.micro_joints.is_none_or(|j| {
                        j.width_mm.is_finite()
                            && j.width_mm > 0.
                            && j.count.is_some() != j.spacing_mm.is_some()
                            && j.count.is_none_or(|n| (1..=1000).contains(&n))
                            && j.spacing_mm.is_none_or(|s| s.is_finite() && s > 0.)
                    }),
                    "micro_joints need a positive width_mm and either a count (1..1000) or a spacing_mm".into(),
                ),
                (
                    op.micro_joints.is_none() || op.compensation == Compensation::Computer,
                    "micro-joints turn the beam off and on inside the loop, which cutter compensation does not allow".into(),
                ),
                (
                    op.common_line.is_none_or(|c| c.min_length_mm.is_finite() && c.min_length_mm > 0.),
                    "common_line min_length_mm must be positive".into(),
                ),
                (
                    op.common_line.is_none() || op.compensation == Compensation::Computer,
                    "a common line has a part on both sides, so the controller cannot trim its kerf: common_line needs compensation=\"computer\"".into(),
                ),
                (
                    op.common_line.is_none() || op.micro_joints.is_none(),
                    "common_line with micro_joints is not supported: use one of them".into(),
                ),
                ((1..=9).contains(&op.work_offset), format!("work_offset {} must be 1..9", op.work_offset)),
                (
                    op.tool_number > 0 && op.length_register > 0,
                    "tool_number and length_register must be positive".into(),
                ),
                (
                    initial.iter().all(|n| n.is_finite() && n.abs() <= 1e6) && first_id > 0,
                    "start position must be finite".into(),
                ),
            ],
        )?;
        let z_cut = top + op.cut_height_mm;
        let (z_pierce, z_travel) = if program {
            (top + pierce_h, top + travel_h)
        } else {
            (z_cut, z_cut)
        };
        let options = ConstructionOptions {
            tolerance_mm: op.tolerance_mm,
        };
        let source = op.parts.prepare(options)?;
        let error = source.summary().boundary_error_mm;
        let margin = op.boundary_tolerance_mm / 2.;
        if 2. * error + margin + guard > op.boundary_tolerance_mm {
            return Err(
                "thermal cut CAD/source uncertainty exceeds boundary tolerance budget".into(),
            );
        }
        let offset = kerf / 2. + error + margin;
        let definition = source.definition().clone();
        // Common lines: parts whose kerf paths coincide are offset one by
        // one (the offset of the whole region cannot represent paths that
        // touch). Without any, the region is offset as a whole as always.
        let split = match op.common_line {
            Some(_) => Some(common_line::part_profiles(
                &definition,
                offset,
                options,
                cancel,
            )?)
            .filter(|p| common_line::any(p, op.tolerance_mm)),
            None => None,
        };
        let shared = split.is_some();
        let prepared = match split {
            Some(p) => p,
            None => crate::profiles::offset_profiles(
                &source,
                offset,
                Boundaries::All,
                false,
                options,
                cancel,
            )
            .map_err(|e| match op.common_line {
                Some(_) => format!(
                    "{e} (common_line: no two parts are exactly {} mm apart, twice the applied offset, along a complete straight edge of both)",
                    2. * offset
                ),
                None => e,
            })?,
        };
        // Nesting depth: how many holes of other parts contain each part.
        let holes: Vec<(usize, crate::planar::PreparedRegion)> = definition
            .components
            .iter()
            .enumerate()
            .flat_map(|(i, c)| c.holes.iter().map(move |h| (i, h)))
            .map(|(i, h)| {
                PlanarRegion {
                    version: 1,
                    units: "mm".into(),
                    source: serde_json::Value::Null,
                    name: "hole".into(),
                    components: vec![Component {
                        outer: h.clone(),
                        holes: vec![],
                    }],
                    boundary_error_mm: definition.boundary_error_mm,
                }
                .prepare(options)
                .map(|r| (i, r))
            })
            .collect::<Result<_, _>>()?;
        let depth = definition
            .components
            .iter()
            .enumerate()
            .map(|(i, c)| {
                let v = c.outer.vertices[0];
                let mut d = 0;
                for (owner, hole) in &holes {
                    if *owner != i
                        && hole.query([v[0], v[1]])?.signed_clearance_mm.unwrap_or(-1.) > 0.
                    {
                        d += 1;
                    }
                }
                Ok(d)
            })
            .collect::<Result<Vec<usize>, String>>()?;
        // Loops: the part on the right of travel unless reversed.
        struct Loop {
            strokes: Vec<Stroke>,
            part: usize,
            hole: Option<usize>,
            length: f64,
        }
        let mut loops = vec![];
        for p in prepared.loops {
            let mut strokes = p.strokes;
            let mut area = 0.;
            for s in &strokes {
                let c = s.curve(0.);
                let mid = c.prepare(op.tolerance_mm)?.position(0.5)?;
                for (a, b) in [(c.start(), mid), (mid, c.end())] {
                    area += a[0] * b[1] - b[0] * a[1];
                }
            }
            // Counter-clockwise (area > 0) puts the inside on the left: the
            // part is inside an outline, outside a hole loop.
            let part_left = (area > 0.) == p.hole.is_none();
            if part_left != op.reverse {
                strokes = strokes.into_iter().rev().map(Stroke::reversed).collect();
            }
            let length = strokes.iter().map(crate::profiles::stroke_length).sum();
            loops.push(Loop {
                strokes,
                part: p.component,
                hole: p.hole,
                length,
            });
        }
        if loops.is_empty() {
            return Err("thermal cut has no part loops".into());
        }
        // Common lines: each group of parts joined by them gets one
        // outline, and each shared edge one open cut.
        let mut common = common_line::Plan::default();
        if let Some(c) = op.common_line.filter(|_| shared) {
            let rings = loops
                .into_iter()
                .map(|l| (l.part, l.hole, l.strokes))
                .collect();
            // Every part's own boundary, for the proof that no kerf path
            // runs inside another part's allowance.
            let boundaries: Vec<(usize, Vec<Stroke>)> = definition
                .components
                .iter()
                .enumerate()
                .flat_map(|(i, c)| {
                    std::iter::once(&c.outer)
                        .chain(&c.holes)
                        .map(move |b| (i, Stroke::from_boundary(b)))
                })
                .collect();
            let (rings, plan) = common_line::plan(
                rings,
                c,
                kerf,
                offset,
                op.tolerance_mm,
                op.reverse,
                &boundaries,
            )?;
            loops = rings
                .into_iter()
                .map(|(part, hole, strokes)| Loop {
                    length: strokes.iter().map(crate::profiles::stroke_length).sum(),
                    strokes,
                    part,
                    hole,
                })
                .collect();
            common = plan;
        }
        // Order: deepest parts first; within a part its holes, then its
        // outline; parts of equal depth as `part_order` says.
        if let PartOrder::Spread { min_distance_mm } = op.part_order
            && !(min_distance_mm.is_finite() && min_distance_mm > 0. && min_distance_mm <= 1e6)
        {
            return Err(format!(
                "thermal cut part_order spread: min_distance_mm {min_distance_mm} must be positive (how far apart the starts of two parts cut one after the other should be)"
            ));
        }
        if op.lead_placement.is_some() && op.common_line.is_some() {
            return Err("thermal cut: lead_placement is not planned together with common_line (a common line's leads are placed by the shared cut); use one of them".into());
        }
        if let Some(c) = &op.corner_loops {
            c.validate()
                .map_err(|e| format!("thermal cut {}: {e}", op.id))?;
            if op.common_line.is_some() {
                return Err(format!(
                    "thermal cut {}: corner_loops is not planned together with common_line (a shared edge has a part on both sides of its corners); use one of them",
                    op.id
                ));
            }
            if op.compensation == Compensation::Wear {
                return Err(format!(
                    "thermal cut {}: corner_loops is not planned together with wear compensation (a controller's cutter compensation is not proven along a path that crosses itself); use compensation in the program",
                    op.id
                ));
            }
        }
        if let Some(c) = &op.corner_slowdown {
            c.validate()
                .map_err(|e| format!("thermal cut {}: {e}", op.id))?;
            if op.common_line.is_some() {
                return Err(format!(
                    "thermal cut {}: corner_slowdown is not planned together with common_line; use one of them",
                    op.id
                ));
            }
        }
        if let Some(avoid) = &op.avoid_cut_parts {
            if !(avoid.clearance_mm.is_finite()
                && avoid.clearance_mm > 0.
                && avoid.clearance_mm <= 1e4)
            {
                return Err(format!(
                    "thermal cut avoid_cut_parts: clearance_mm {} must be positive (the distance rapids keep from parts already cut free, from the beam centre: include the torch body's radius)",
                    avoid.clearance_mm
                ));
            }
            if op.common_line.is_some() {
                return Err("thermal cut: avoid_cut_parts is not planned together with common_line (the travel to a common line's pierce is not routed); use one of them".into());
            }
        }
        let mut order = vec![];
        let mut part_sequence = vec![];
        let mut remaining: Vec<usize> = (0..definition.components.len()).collect();
        let mut at = [initial[0], initial[1]];
        while !remaining.is_empty() {
            let deepest = remaining.iter().map(|&i| depth[i]).max().unwrap();
            let candidates = remaining
                .iter()
                .enumerate()
                .filter(|(_, i)| depth[**i] == deepest)
                .map(|(k, &i)| {
                    let v = definition.components[i].outer.vertices[0];
                    (k, (v[0] - at[0]).hypot(v[1] - at[1]))
                });
            let (k, _) = match op.part_order {
                PartOrder::Nearest => candidates.min_by(|a, b| a.1.total_cmp(&b.1)),
                // `remaining` keeps the given order.
                PartOrder::Given => candidates.min_by_key(|c| c.0),
                PartOrder::Spread { min_distance_mm } => {
                    let all: Vec<(usize, f64)> = candidates.collect();
                    all.iter()
                        .copied()
                        .filter(|c| c.1 >= min_distance_mm || part_sequence.is_empty())
                        .min_by(|a, b| a.1.total_cmp(&b.1))
                        .or_else(|| all.iter().copied().max_by(|a, b| a.1.total_cmp(&b.1)))
                }
            }
            .unwrap();
            let part = remaining.remove(k);
            part_sequence.push(part);
            // Parts joined by common lines are cut as one group.
            let members = common.members(part);
            remaining.retain(|p| !members.contains(p));
            let mut own: Vec<usize> = (0..loops.len())
                .filter(|&l| members.contains(&loops[l].part))
                .collect();
            // Holes nearest first, the outline last.
            let mut ordered = vec![];
            while let Some((k, _)) = own
                .iter()
                .enumerate()
                .filter(|(_, l)| loops[**l].hole.is_some())
                .map(|(k, &l)| {
                    let s = loops[l].strokes[0].a;
                    (k, (s[0] - at[0]).hypot(s[1] - at[1]))
                })
                .min_by(|a, b| a.1.total_cmp(&b.1))
            {
                let l = own.remove(k);
                at = loops[l].strokes[0].a;
                ordered.push(l);
            }
            // Common lines after the holes and before the outline (entries
            // past the loops).
            ordered.extend(common.cuts_of(part).into_iter().map(|c| loops.len() + c));
            ordered.extend(own);
            if let Some(&l) = ordered.last() {
                at = loops[l].strokes[0].a;
            }
            order.extend(ordered);
        }
        // Proofs: the kerf (with the boundary margin) never touches a part,
        // and each pierce crater stays clear of every part.
        let thickness = top - spec.min_mm[2];
        let z_check = spec.min_mm[2];
        // Checked part by part (only the parts a probe can reach).
        let scenes = verification::ComponentScenes::new(
            "thermal-parts",
            &op.id,
            "parts",
            &definition,
            [spec.min_mm[2], top],
            verification::Options {
                tolerance_mm: op.tolerance_mm,
                spatial_resolution_mm: (op.boundary_tolerance_mm / 16.).max(op.tolerance_mm),
                ..Default::default()
            },
        )?;
        let kerf_probe = probe_tool(kerf, thickness + 1.).prepare()?;
        let pierce_clearance = op.pierce_clearance_mm.unwrap_or(kerf);
        let crater_probe = probe_tool(kerf + 2. * pierce_clearance, thickness + 1.).prepare()?;
        let probe = |tool: &crate::tool::PreparedAssembly,
                     curve: Curve,
                     what: &str|
         -> Result<(), String> {
            let motion = Motion {
                id: first_id,
                operation: op.id.clone(),
                tool: "kerf-probe".into(),
                curve,
                kind: MoveKind::Cut,
                feed_mm_min: Some(op.feed_mm_min),
            };
            if let Some(check) = scenes.verify_motion(tool, &motion, cancel)?.unclear {
                return Err(format!(
                    "thermal cut {what} is not proven clear of the parts ({:?}); move the lead or shorten it: {:?}",
                    check.status,
                    check.findings.first()
                ));
            }
            Ok(())
        };
        let at_z = |p: [f64; 2], z: f64| [p[0], p[1], z];
        let mut steps = std::collections::VecDeque::new();
        let mut pos = initial;
        let mut report_order = vec![];
        let mut small = 0;
        let mut micro_joints = 0;
        let mut cut_length = 0.;
        let mut diagnostics = prepared.diagnostics;
        diagnostics.append(&mut common.diagnostics);
        if op.common_line.is_some() && !shared {
            diagnostics.push(
                "common_line: no two parts are exactly one kerf path apart; nothing is shared"
                    .into(),
            );
        }
        let power = op.power_percent;
        let line = |a: Vec3, b: Vec3| Curve::Line { start: a, end: b };
        let push_move = |steps: &mut std::collections::VecDeque<Step>,
                         end: Vec3,
                         pos: &mut Vec3,
                         kind,
                         feed| {
            if *pos != end {
                steps.push_back(Step::Move(line(*pos, end), kind, feed));
                *pos = end;
            }
        };
        if !program && pos[2] != z_cut {
            push_move(
                &mut steps,
                [pos[0], pos[1], z_cut],
                &mut pos,
                MoveKind::Rapid,
                None,
            );
        }
        let mut common_lines: Vec<CommonLineReport> = vec![];
        // Pieces cut free so far, the piece the last loop freed, and per
        // completed loop the piece it freed (for the check of the rapids).
        let mut freed = tip_up::Freed::default();
        let mut leaving: Option<usize> = None;
        let mut freed_by_loop: Vec<Option<usize>> = vec![];
        let mut rerouted = 0;
        let mut leads_moved = 0;
        let (mut corner_loops, mut corner_slowdowns) = (0, 0);
        let lead_rules = op
            .lead_placement
            .as_ref()
            .map(|placement| lead_placement::Prepared::new(placement, offset, guard))
            .transpose()
            .map_err(|e| format!("thermal cut {}: {e}", op.id))?;
        // What each pierce cuts, for the refusals of the kerf checks.
        let mut pierced = vec![];
        for &l in &order {
            if l >= loops.len() {
                let [a, b] = common.cut(l - loops.len()).parts;
                pierced.push(format!("the common line between parts {a} and {b}"));
                let emit = common_line::Emit {
                    probe: &probe,
                    kerf_probe: &kerf_probe,
                    crater_probe: &crater_probe,
                    lead_mm: op.lead_in.length_mm,
                    top,
                    z_check,
                    z_cut,
                    z_pierce,
                    z_travel,
                    program,
                    power_percent: power,
                    pierce_delay_s: op.pierce_delay_s,
                    plunge_mm_min: op.plunge_mm_min,
                    feed_mm_min: op.feed_mm_min,
                    height_control: op.height_control,
                };
                let (cut, line, length) = common.cut(l - loops.len()).steps(&emit, &mut pos)?;
                steps.extend(cut);
                common_lines.push(line);
                cut_length += length;
                continue;
            }
            let lp = &loops[l];
            report_order.push((lp.part, lp.hole));
            pierced.push(match lp.hole {
                Some(h) => format!("hole {h} of part {}", lp.part),
                None => match common.members(lp.part).as_slice() {
                    [_] => format!("the outline of part {}", lp.part),
                    group => format!("the outline of parts {group:?}"),
                },
            });
            let is_small = lp.hole.is_some()
                && op.small_holes.is_some_and(|s| {
                    lp.length / PI + kerf <= s.max_diameter_mm.unwrap_or(1.5 * thickness)
                });
            small += usize::from(is_small);
            let feed = if is_small {
                op.feed_mm_min * op.small_holes.unwrap().feed_percent / 100.
            } else {
                op.feed_mm_min
            };
            let thc =
                op.height_control && !(is_small && op.small_holes.unwrap().height_control_off);
            // Leads inside a hole stay well inside it.
            let limit = if lp.hole.is_some() {
                0.45 * lp.length / (2. * PI)
            } else {
                f64::INFINITY
            };
            let lead = |l: Lead| Lead {
                length_mm: l.length_mm.min(limit),
                radius_mm: l.radius_mm.min(limit),
            };
            let (mut lin, lout) = (lead(op.lead_in), lead(op.lead_out));
            if lin.length_mm < op.lead_in.length_mm || lin.radius_mm < op.lead_in.radius_mm {
                diagnostics.push(format!(
                    "part {} hole {:?}: leads shortened to fit the hole",
                    lp.part, lp.hole
                ));
            }
            let side = if op.reverse { -1. } else { 1. };
            // Scrap side: left of travel unless reversed.
            let scrap = |t: [f64; 2]| {
                let n = rot_left(t);
                [side * n[0], side * n[1]]
            };
            // Where the loop starts: its own start, unless the leads (or the
            // pierce) there are not clear of the parts. Then the middle of
            // its longest straight stroke whose leads are: a loop starting at a
            // corner (a straight-edged hole) sends the tangent lead back
            // past the corner, out of the scrap.
            let mut strokes = lp.strokes.clone();
            let clear_at = |strokes: &[Stroke]| -> Result<(), String> {
                let (start, t) = (strokes[0].a, strokes[0].tangent(false));
                let n = scrap(t);
                let center = add(start, n, lin.radius_mm);
                let a = add(center, t, -lin.radius_mm);
                let pierce = add(a, n, lin.length_mm);
                let low = |p: [f64; 2]| at_z(p, z_check);
                probe(
                    &crater_probe,
                    line(at_z(pierce, top + 1.), low(pierce)),
                    "pierce point",
                )?;
                if lin.length_mm > 0. {
                    probe(&kerf_probe, line(low(pierce), low(a)), "lead")?;
                }
                if lin.radius_mm > 0. {
                    let arc = Curve::Arc {
                        start: low(a),
                        end: low(start),
                        center: low(center),
                        plane: Plane::Xy,
                        sweep_rad: side * FRAC_PI_2,
                    };
                    probe(&kerf_probe, arc, "lead")?;
                }
                // The lead-out from the loop's close (without overcut).
                let last = strokes[strokes.len() - 1];
                let te = last.tangent(true);
                let ne = scrap(te);
                let c_out = add(start, ne, lout.radius_mm);
                let b = add(c_out, te, lout.radius_mm);
                if lout.radius_mm > 0. {
                    let arc = Curve::Arc {
                        start: low(start),
                        end: low(b),
                        center: low(c_out),
                        plane: Plane::Xy,
                        sweep_rad: side * FRAC_PI_2,
                    };
                    probe(&kerf_probe, arc, "lead")?;
                }
                if lout.length_mm > 0. {
                    probe(
                        &kerf_probe,
                        line(low(b), low(add(b, ne, lout.length_mm))),
                        "lead",
                    )?;
                }
                Ok(())
            };
            if let Some(rules) = &lead_rules {
                // The first start, in the preferred order, that is off the
                // kept-off edges and whose leads and pierce are clear; one
                // that is only off them is taken to the checks below (a
                // hole's centre pierce, or the refusal that names the lead).
                let candidates = rules.candidates(&strokes, 1e-6);
                let mut allowed = candidates
                    .iter()
                    .filter(|start| !rules.kept_off(start.point(&strokes)));
                let Some(first) = allowed.next() else {
                    return Err(format!(
                        "thermal cut {}: part {} {}: every place the loop could start is on an edge kept free of leads (lead_placement.keep_off); free part of an edge or reduce keep_off_distance_mm",
                        op.id,
                        lp.part,
                        lp.hole.map_or("outline".into(), |h| format!("hole {h}"))
                    ));
                };
                let mut chosen = first.rotate(&strokes);
                if clear_at(&chosen).is_err() {
                    for start in allowed {
                        let r = start.rotate(&strokes);
                        if clear_at(&r).is_ok() {
                            chosen = r;
                            break;
                        }
                    }
                }
                leads_moved += usize::from(chosen[0].a != strokes[0].a);
                strokes = chosen;
            } else if clear_at(&strokes).is_err() {
                // Straight strokes only: a hole bounded by arcs keeps the
                // center pierce below (small round holes).
                let mut by_length: Vec<usize> = (0..strokes.len())
                    .filter(|&k| strokes[k].arc.is_none())
                    .collect();
                by_length.sort_by(|&i, &j| {
                    crate::profiles::stroke_length(&strokes[j])
                        .total_cmp(&crate::profiles::stroke_length(&strokes[i]))
                        .then(i.cmp(&j))
                });
                let rotated = |k: usize| {
                    let s = strokes[k];
                    let mid = crate::profiles::stroke_point(&s, 0.5);
                    let half = |sweep: f64| sweep / 2.;
                    let mut r = vec![Stroke {
                        a: mid,
                        b: s.b,
                        arc: s.arc.map(|(c, w)| (c, half(w))),
                    }];
                    r.extend_from_slice(&strokes[k + 1..]);
                    r.extend_from_slice(&strokes[..k]);
                    r.push(Stroke {
                        a: s.a,
                        b: mid,
                        arc: s.arc.map(|(c, w)| (c, half(w))),
                    });
                    r
                };
                let found = by_length
                    .iter()
                    .take(16)
                    .map(|&k| rotated(k))
                    .find(|r| clear_at(r).is_ok());
                let fallback = (found.is_none() && lp.hole.is_some())
                    .then(|| by_length.first().map(|&k| rotated(k)))
                    .flatten();
                if let Some(r) = found {
                    strokes = r;
                    diagnostics.push(format!(
                        "part {} hole {:?}: leads moved from the loop's start to the middle of an edge, where they are clear of the parts",
                        lp.part, lp.hole
                    ));
                } else if let Some(r) = fallback {
                    // No edge middle clears the leads: pierce at the hole's
                    // center below, from the middle of its longest edge.
                    strokes = r;
                }
            }
            // Corner loops, once the start is settled (the corner the loop
            // starts at keeps its leads instead).
            let mut in_loop = vec![false; strokes.len()];
            if let Some(c) = &op.corner_loops {
                let (with, flags, count) = corners::with_loops(&strokes, c, offset, side, guard);
                (strokes, in_loop) = (with, flags);
                corner_loops += count;
            }
            let s0 = strokes[0];
            let start = s0.a;
            let t = s0.tangent(false);
            let n = scrap(t);
            let geometry = |l: Lead| {
                let center = add(start, n, l.radius_mm);
                let a = add(center, t, -l.radius_mm);
                (center, a, add(a, n, l.length_mm))
            };
            let crater = |p: [f64; 2]| line(at_z(p, top + 1.), at_z(p, z_check));
            let (mut center, mut a, mut pierce) = geometry(lin);
            let first = probe(&crater_probe, crater(pierce), "pierce point");
            if let Err(error) = first {
                // In a hole: pierce at its center instead (Hypertherm's
                // small-hole practice), with a smaller arc if needed.
                let reach = lp.length / (2. * PI);
                let fallback = lp.hole.and_then(|_| {
                    [0.45, 0.25, 0.1, 0.]
                        .into_iter()
                        .map(|k| {
                            let r = lin.radius_mm.min(k * reach);
                            Lead {
                                radius_mm: r,
                                length_mm: (reach - r).max(0.),
                            }
                        })
                        .find(|l| {
                            probe(&crater_probe, crater(geometry(*l).2), "pierce point").is_ok()
                        })
                });
                let Some(l) = fallback else {
                    return Err(error);
                };
                diagnostics.push(format!(
                    "part {} hole {:?}: pierced at the hole center (lead {:.3} mm, arc radius {:.3} mm)",
                    lp.part, lp.hole, l.length_mm, l.radius_mm
                ));
                lin = l;
                (center, a, pierce) = geometry(l);
            }
            // Travel to the pierce point and pierce: round the pieces
            // already cut free where that is asked for.
            let detour = match &op.avoid_cut_parts {
                Some(avoid) => freed
                    .route([pos[0], pos[1]], pierce, avoid.clearance_mm, leaving)
                    .map_err(|e| format!("thermal cut {}: {e}", op.id))?,
                None => vec![],
            };
            rerouted += usize::from(!detour.is_empty());
            if program {
                push_move(
                    &mut steps,
                    [pos[0], pos[1], z_travel.max(pos[2])],
                    &mut pos,
                    MoveKind::Rapid,
                    None,
                );
            }
            for p in detour {
                let z = pos[2];
                push_move(&mut steps, at_z(p, z), &mut pos, MoveKind::Rapid, None);
            }
            if program {
                push_move(
                    &mut steps,
                    at_z(pierce, pos[2]),
                    &mut pos,
                    MoveKind::Rapid,
                    None,
                );
                push_move(
                    &mut steps,
                    at_z(pierce, z_pierce),
                    &mut pos,
                    MoveKind::Rapid,
                    None,
                );
            } else {
                push_move(
                    &mut steps,
                    at_z(pierce, z_cut),
                    &mut pos,
                    MoveKind::Rapid,
                    None,
                );
            }
            steps.push_back(Step::Event(Event::Beam {
                power_percent: power,
            }));
            if op.pierce_delay_s > 0. {
                steps.push_back(Step::Event(Event::Dwell {
                    seconds: op.pierce_delay_s,
                }));
            }
            push_move(
                &mut steps,
                at_z(pierce, z_cut),
                &mut pos,
                MoveKind::Plunge,
                op.plunge_mm_min,
            );
            let wear = op.compensation == Compensation::Wear;
            let comp_side = if op.reverse {
                CompensationSide::Left
            } else {
                CompensationSide::Right
            };
            // Lead-in: straight, then the tangent arc onto the loop.
            let mut cuts: Vec<(Curve, MoveKind)> = vec![];
            // Per move of `cuts` (absent: none): part of a corner loop, and
            // cut at the corner feed.
            const LOOP: u8 = 1;
            const SLOW: u8 = 2;
            let mut tags: Vec<u8> = vec![];
            if lin.length_mm > 0. {
                cuts.push((line(at_z(pierce, z_cut), at_z(a, z_cut)), MoveKind::Lead));
            }
            if lin.radius_mm > 0. {
                cuts.push((
                    Curve::Arc {
                        start: at_z(a, z_cut),
                        end: at_z(start, z_cut),
                        center: at_z(center, z_cut),
                        plane: Plane::Xy,
                        sweep_rad: side * FRAC_PI_2,
                    },
                    MoveKind::Lead,
                ));
            }
            let lead_in_moves = cuts.len();
            // A negative overcut stops that far before the loop closes.
            let mut short = (-op.overcut_mm).clamp(0., 0.5 * lp.length);
            let mut end = (start, t);
            let mut kept = strokes.len();
            while short > guard && kept > 0 {
                let s = &strokes[kept - 1];
                let length = crate::profiles::stroke_length(s);
                if length > short + guard {
                    break;
                }
                short -= length;
                kept -= 1;
            }
            for (k, s) in strokes[..kept].iter().enumerate() {
                if k + 1 == kept && short > guard {
                    let length = crate::profiles::stroke_length(s);
                    let f = 1. - short / length;
                    cuts.push((
                        crate::profiles::sub_curve(s, 0., f, z_cut, z_cut),
                        MoveKind::Cut,
                    ));
                    let e = crate::profiles::stroke_point(s, f);
                    end = (
                        e,
                        Stroke {
                            a: e,
                            b: s.b,
                            arc: s.arc,
                        }
                        .tangent(false),
                    );
                } else {
                    cuts.push((s.curve(z_cut), MoveKind::Cut));
                    end = (s.b, s.tangent(true));
                }
                tags.resize(cuts.len() - 1, 0);
                tags.push(if in_loop[k] { LOOP } else { 0 });
            }
            // Overcut: on around the loop from its start.
            let mut left = op.overcut_mm.clamp(0., lp.length);
            for s in &strokes {
                if left <= guard {
                    break;
                }
                let length = crate::profiles::stroke_length(s);
                let f = (left / length).min(1.);
                cuts.push((
                    crate::profiles::sub_curve(s, 0., f, z_cut, z_cut),
                    MoveKind::Cut,
                ));
                let e = crate::profiles::stroke_point(s, f);
                let tangent = if f >= 1. {
                    s.tangent(true)
                } else {
                    let probe_s = Stroke {
                        a: e,
                        b: s.b,
                        arc: s.arc,
                    };
                    probe_s.tangent(false)
                };
                end = (e, tangent);
                left -= length * f;
            }
            let mut cut_moves = cuts.len();
            tags.resize(cut_moves, 0);
            // The corner feed: the loop's cut moves split where it begins
            // and ends. A small hole is cut at its own reduced feed.
            if let Some(slow) = op.corner_slowdown.as_ref().filter(|_| !is_small) {
                let curves: Vec<Curve> = cuts[lead_in_moves..]
                    .iter()
                    .map(|(c, _)| c.clone())
                    .collect();
                let (pieces, count) = corners::slow_down(
                    &curves,
                    &strokes,
                    &in_loop,
                    slow,
                    offset,
                    guard,
                    op.tolerance_mm,
                )?;
                corner_slowdowns += count;
                let old = tags.split_off(lead_in_moves);
                cuts.truncate(lead_in_moves);
                for piece in pieces {
                    cuts.push((piece.curve, MoveKind::Cut));
                    tags.push(old[piece.source] | if piece.slowed { SLOW } else { 0 });
                }
                cut_moves = cuts.len();
            }
            // Lead-out: the tangent arc, then straight into the scrap.
            let (e, te) = end;
            let ne = scrap(te);
            let c_out = add(e, ne, lout.radius_mm);
            let b = add(c_out, te, lout.radius_mm);
            let q = add(b, ne, lout.length_mm);
            if lout.radius_mm > 0. {
                cuts.push((
                    Curve::Arc {
                        start: at_z(e, z_cut),
                        end: at_z(b, z_cut),
                        center: at_z(c_out, z_cut),
                        plane: Plane::Xy,
                        sweep_rad: side * FRAC_PI_2,
                    },
                    MoveKind::Lead,
                ));
            }
            if lout.length_mm > 0. {
                cuts.push((line(at_z(b, z_cut), at_z(q, z_cut)), MoveKind::Lead));
            }
            // Micro-joints: gaps along the loop, the beam off over each.
            let mut joints = 0;
            if let Some(j) = op.micro_joints.filter(|j| lp.hole.is_none() || j.holes) {
                let n = j.count.map_or_else(
                    || (lp.length / j.spacing_mm.unwrap()).round().max(1.) as usize,
                    |n| n as usize,
                );
                let half = (j.width_mm + kerf) / 2.;
                if lp.length / (2. * n as f64) <= half + guard {
                    return Err(format!(
                        "{n} micro-joints of width {} do not fit a loop {:.3} mm long",
                        j.width_mm, lp.length
                    ));
                }
                let gaps: Vec<(f64, f64)> = (0..n)
                    .map(|k| {
                        let c = (k as f64 + 0.5) * lp.length / n as f64;
                        (c - half, c + half)
                    })
                    .collect();
                let mut split = vec![];
                let mut split_tags = vec![];
                let mut along = 0.;
                let mut touched = vec![false; gaps.len()];
                for (k, (curve, kind)) in cuts.into_iter().enumerate() {
                    let tag = tags.get(k).copied().unwrap_or(0);
                    // A corner loop lies in the scrap: no joint there, and
                    // its length is not the outline's.
                    if kind != MoveKind::Cut || k >= cut_moves || tag & LOOP != 0 {
                        split.push((curve, kind));
                        split_tags.push(tag);
                        continue;
                    }
                    let length = curve.prepare(op.tolerance_mm)?.length_mm();
                    let (s0, s1) = (along, along + length);
                    along = s1;
                    let mut t = 0.;
                    for (gi, &(g0, g1)) in gaps.iter().enumerate() {
                        if g1 <= s0 || g0 >= s1 {
                            continue;
                        }
                        touched[gi] = true;
                        let a = ((g0 - s0) / length).clamp(0., 1.);
                        let b = ((g1 - s0) / length).clamp(0., 1.);
                        if a > t {
                            split.push((part_of(&curve, t, a), MoveKind::Cut));
                            split_tags.push(tag);
                        }
                        let from = point_at(&curve, a.max(t));
                        let to = point_at(&curve, b);
                        split.push((line(from, to), MoveKind::Rapid));
                        split_tags.push(0);
                        t = b;
                    }
                    if t < 1. {
                        split.push((
                            if t == 0. {
                                curve
                            } else {
                                part_of(&curve, t, 1.)
                            },
                            MoveKind::Cut,
                        ));
                        split_tags.push(tag);
                    }
                }
                joints = touched.iter().filter(|t| **t).count();
                cut_moves = split
                    .iter()
                    .enumerate()
                    .skip(lead_in_moves)
                    .find(|(_, (_, k))| *k == MoveKind::Lead)
                    .map_or(split.len(), |(i, _)| i);
                cuts = split;
                tags = split_tags;
            }
            // A loop's run-on continues the edge before it and its run-back
            // the edge after it: one line each, where the feed is the same.
            if in_loop.iter().any(|f| *f) {
                let mut joined: Vec<(Curve, MoveKind)> = Vec::with_capacity(cuts.len());
                let mut joined_tags: Vec<u8> = Vec::with_capacity(cuts.len());
                let mut removed_before_leads = 0;
                for (k, (curve, kind)) in cuts.into_iter().enumerate() {
                    let tag = tags.get(k).copied().unwrap_or(0);
                    if k > lead_in_moves
                        && k < cut_moves
                        && kind == MoveKind::Cut
                        && let Some((Curve::Line { start, end }, MoveKind::Cut)) = joined.last()
                        && let Curve::Line { start: s2, end: e2 } = &curve
                        && let Some(&last_tag) = joined_tags.last()
                        && (last_tag | tag) & LOOP != 0
                        && (last_tag ^ tag) & SLOW == 0
                        && end == s2
                    {
                        let (u, v) = (
                            [end[0] - start[0], end[1] - start[1]],
                            [e2[0] - s2[0], e2[1] - s2[1]],
                        );
                        let (lu, lv) = (u[0].hypot(u[1]), v[0].hypot(v[1]));
                        if lu > 0.
                            && lv > 0.
                            && (u[0] * v[1] - u[1] * v[0]).abs() <= 1e-12 * lu * lv
                            && u[0] * v[0] + u[1] * v[1] > 0.
                        {
                            let merged = Curve::Line {
                                start: *start,
                                end: *e2,
                            };
                            *joined.last_mut().unwrap() = (merged, MoveKind::Cut);
                            *joined_tags.last_mut().unwrap() = last_tag & tag;
                            removed_before_leads += 1;
                            continue;
                        }
                    }
                    joined.push((curve, kind));
                    joined_tags.push(tag);
                }
                cut_moves -= removed_before_leads;
                (cuts, tags) = (joined, joined_tags);
            }
            let hold = op
                .corner_slowdown
                .is_some_and(|slow| slow.height_control_off);
            let corner_feed = op
                .corner_slowdown
                .map_or(feed, |slow| feed * slow.feed_percent / 100.);
            // Whether height control is on, as the events so far leave it.
            let mut thc_on = false;
            for (k, (curve, kind)) in cuts.into_iter().enumerate() {
                let slowed = tags.get(k).copied().unwrap_or(0) & SLOW != 0;
                if kind == MoveKind::Rapid {
                    // Over a joint: beam off, across, fire again.
                    if thc_on {
                        steps.push_back(Step::Event(Event::HeightControl { on: false }));
                        thc_on = false;
                    }
                    steps.push_back(Step::Event(Event::Beam { power_percent: 0. }));
                    pos = curve.end();
                    steps.push_back(Step::Move(curve, MoveKind::Rapid, None));
                    steps.push_back(Step::Event(Event::Beam {
                        power_percent: power,
                    }));
                    if op.pierce_delay_s > 0. {
                        steps.push_back(Step::Event(Event::Dwell {
                            seconds: op.pierce_delay_s,
                        }));
                    }
                    continue;
                }
                let mut check = curve.clone();
                for p in match &mut check {
                    Curve::Line { start, end } => vec![start, end],
                    Curve::Arc {
                        start, end, center, ..
                    }
                    | Curve::Helix {
                        start, end, center, ..
                    } => {
                        vec![start, end, center]
                    }
                } {
                    p[2] = z_check;
                }
                probe(
                    &kerf_probe,
                    check,
                    if kind == MoveKind::Cut {
                        "kerf path"
                    } else {
                        "lead"
                    },
                )
                .map_err(|e| {
                    if kind == MoveKind::Cut && op.corner_loops.is_some() {
                        format!(
                            "{e}. With corner_loops the loops need room beside the parts: nest for this cut (Nest.for_cut keeps it) or reduce overshoot_mm"
                        )
                    } else {
                        e
                    }
                })?;
                if wear && k == 0 {
                    steps.push_back(Step::Event(Event::CutterCompensation {
                        side: comp_side,
                        plane: crate::motion::Plane::Xy,
                    }));
                }
                // Height control: on over the loop's cut, off over the
                // leads and, where asked, over a corner's reduced feed.
                let wanted = thc && k >= lead_in_moves && k < cut_moves && !(slowed && hold);
                if wanted != thc_on {
                    steps.push_back(Step::Event(Event::HeightControl { on: wanted }));
                    thc_on = wanted;
                }
                if wear
                    && kind == MoveKind::Lead
                    && k >= cut_moves
                    && matches!(curve, Curve::Line { .. })
                {
                    steps.push_back(Step::Event(Event::CutterCompensation {
                        side: CompensationSide::Off,
                        plane: crate::motion::Plane::Xy,
                    }));
                }
                if kind == MoveKind::Cut {
                    cut_length += curve.prepare(op.tolerance_mm)?.length_mm();
                }
                pos = curve.end();
                steps.push_back(Step::Move(
                    curve,
                    kind,
                    Some(if slowed { corner_feed } else { feed }),
                ));
            }
            if thc_on {
                steps.push_back(Step::Event(Event::HeightControl { on: false }));
            }
            steps.push_back(Step::Event(Event::Beam { power_percent: 0. }));
            steps.push_back(Step::Done);
            micro_joints += joints;
            // What this loop cut free: the hole's slug or the part, unless
            // micro-joints hold it.
            leaving = (op.avoid_cut_parts.is_some() && joints == 0).then(|| {
                let c = &definition.components[lp.part];
                freed.add(match lp.hole {
                    Some(h) => &c.holes[h],
                    None => &c.outer,
                })
            });
            freed_by_loop.push(leaving);
            if program {
                push_move(
                    &mut steps,
                    [pos[0], pos[1], z_travel],
                    &mut pos,
                    MoveKind::Rapid,
                    None,
                );
            }
        }
        // The crop cut, last: every part is cut, so nothing it frees can be a
        // part. Its pierce crater and its kerf are proven clear of every
        // part as a lead's are.
        let mut crop_length_mm = None;
        if let Some(crop) = &op.crop {
            let points = &crop.points_mm;
            let low = |p: [f64; 2]| at_z(p, z_check);
            probe(
                &crater_probe,
                line(at_z(points[0], top + 1.), low(points[0])),
                "crop cut's pierce point",
            )?;
            for w in points.windows(2) {
                probe(&kerf_probe, line(low(w[0]), low(w[1])), "crop cut")?;
            }
            let detour = match &op.avoid_cut_parts {
                Some(avoid) => freed
                    .route([pos[0], pos[1]], points[0], avoid.clearance_mm, leaving)
                    .map_err(|e| format!("thermal cut {}: crop cut: {e}", op.id))?,
                None => vec![],
            };
            rerouted += usize::from(!detour.is_empty());
            if program {
                push_move(
                    &mut steps,
                    [pos[0], pos[1], z_travel.max(pos[2])],
                    &mut pos,
                    MoveKind::Rapid,
                    None,
                );
            }
            for p in detour {
                let z = pos[2];
                push_move(&mut steps, at_z(p, z), &mut pos, MoveKind::Rapid, None);
            }
            if program {
                let z = pos[2];
                push_move(
                    &mut steps,
                    at_z(points[0], z),
                    &mut pos,
                    MoveKind::Rapid,
                    None,
                );
                push_move(
                    &mut steps,
                    at_z(points[0], z_pierce),
                    &mut pos,
                    MoveKind::Rapid,
                    None,
                );
            } else {
                push_move(
                    &mut steps,
                    at_z(points[0], z_cut),
                    &mut pos,
                    MoveKind::Rapid,
                    None,
                );
            }
            steps.push_back(Step::Event(Event::Beam {
                power_percent: power,
            }));
            if op.pierce_delay_s > 0. {
                steps.push_back(Step::Event(Event::Dwell {
                    seconds: op.pierce_delay_s,
                }));
            }
            push_move(
                &mut steps,
                at_z(points[0], z_cut),
                &mut pos,
                MoveKind::Plunge,
                op.plunge_mm_min,
            );
            if op.height_control {
                steps.push_back(Step::Event(Event::HeightControl { on: true }));
            }
            let mut length = 0.;
            for w in points.windows(2) {
                length += (w[0][0] - w[1][0]).hypot(w[0][1] - w[1][1]);
                push_move(
                    &mut steps,
                    at_z(w[1], z_cut),
                    &mut pos,
                    MoveKind::Cut,
                    Some(op.feed_mm_min),
                );
            }
            if op.height_control {
                steps.push_back(Step::Event(Event::HeightControl { on: false }));
            }
            steps.push_back(Step::Event(Event::Beam { power_percent: 0. }));
            if program {
                push_move(
                    &mut steps,
                    [pos[0], pos[1], z_travel],
                    &mut pos,
                    MoveKind::Rapid,
                    None,
                );
            }
            cut_length += length;
            crop_length_mm = Some(length);
        }
        if common.len() > 0 {
            common_line::check_kerfs(steps.iter(), &pierced, kerf, op.tolerance_mm)?;
        }
        // Validate the complete outline before emitting any beam-on event.
        // A clipped thermal outline would no longer produce the requested part.
        if let Some(limits) = op.limits.as_ref().filter(|l| !l.is_empty()) {
            let prepared = crate::limits::PreparedLimits::new(limits, kerf / 2., op.tolerance_mm)?;
            for step in &steps {
                if let Step::Move(curve, kind, _) = step {
                    let cutting = !kind.is_rapid();
                    let checked = if cutting {
                        // The beam reaches the sheet bottom even when the nozzle
                        // stays above the top of an avoid area.
                        let lower = |mut p: Vec3| {
                            p[2] = spec.min_mm[2];
                            p
                        };
                        match curve {
                            Curve::Line { start, end } => Curve::Line {
                                start: lower(*start),
                                end: lower(*end),
                            },
                            Curve::Arc {
                                start,
                                end,
                                center,
                                plane,
                                sweep_rad,
                            } => Curve::Arc {
                                start: lower(*start),
                                end: lower(*end),
                                center: lower(*center),
                                plane: *plane,
                                sweep_rad: *sweep_rad,
                            },
                            Curve::Helix { .. } => {
                                return Err("thermal limits: unexpected helical path".into());
                            }
                        }
                    } else {
                        curve.clone()
                    };
                    // A vertical pierce becomes a stationary beam footprint
                    // when projected to the sheet bottom.
                    let allowed = if checked.start() == checked.end()
                        && matches!(checked, Curve::Line { .. })
                    {
                        let p = checked.start();
                        prepared.margin([p[0], p[1]], p[2], cutting)? >= 0.
                    } else {
                        prepared.allows_curve(&checked, cutting)?
                    };
                    if !allowed {
                        return Err(format!(
                            "thermal cut {}: a {kind:?} move reaches an avoid area or leaves containment; change the limits or part placement",
                            op.id
                        ));
                    }
                }
            }
        }
        // Events before the first move are handed out before it.
        let mut events = vec![];
        while let Some(Step::Event(_)) = steps.front() {
            if let Some(Step::Event(e)) = steps.pop_front() {
                events.push(e);
            }
        }
        // The order's rules, checked on the order as it will be cut, apart
        // from how it was chosen: a part's holes before its outline, and no
        // part before one nested deeper (a part in a hole goes first).
        let mut outline_at = vec![usize::MAX; definition.components.len()];
        let mut last_depth = usize::MAX;
        for (k, &(part, hole)) in report_order.iter().enumerate() {
            match hole {
                None => {
                    outline_at[part] = k;
                    if depth[part] > last_depth {
                        return Err(format!(
                            "thermal cut {}: the cut order puts part {part}, nested in another part's hole, after a part outside it; this is a planner defect: report it",
                            op.id
                        ));
                    }
                    last_depth = depth[part];
                }
                Some(h) if outline_at[part] != usize::MAX => {
                    return Err(format!(
                        "thermal cut {}: the cut order puts hole {h} of part {part} after its outline; this is a planner defect: report it",
                        op.id
                    ));
                }
                Some(_) => {}
            }
        }
        // The rapids as they will be run, checked against the pieces cut
        // free before each: apart from the routing that chose them.
        if let Some(avoid) = &op.avoid_cut_parts {
            let (mut count, mut done, mut exempt) = (0, 0, None);
            for step in &steps {
                match step {
                    Step::Done => {
                        exempt = freed_by_loop[done];
                        if let Some(i) = exempt {
                            count = i + 1;
                        }
                        done += 1;
                    }
                    Step::Move(curve, MoveKind::Rapid, _) => {
                        let (a, b) = (curve.start(), curve.end());
                        if a[0] == b[0] && a[1] == b[1] {
                            continue;
                        }
                        if let Some((piece, distance)) = freed.blocked(
                            count,
                            [a[0], a[1]],
                            [b[0], b[1]],
                            avoid.clearance_mm,
                            exempt,
                        ) {
                            return Err(format!(
                                "thermal cut {}: a rapid from ({:.3}, {:.3}) to ({:.3}, {:.3}) passes {distance:.3} mm from freed piece {piece}, nearer than avoid_cut_parts allows; this is a planner defect: report it",
                                op.id, a[0], a[1], b[0], b[1]
                            ));
                        }
                        exempt = None;
                    }
                    _ => {}
                }
            }
            debug_assert_eq!(freed.len(), freed_by_loop.iter().flatten().count());
        }
        let start_of = |part: usize| {
            let v = definition.components[part].outer.vertices[0];
            [v[0], v[1]]
        };
        let min_part_step_mm = part_sequence
            .windows(2)
            .map(|w| {
                let (a, b) = (start_of(w[0]), start_of(w[1]));
                (a[0] - b[0]).hypot(a[1] - b[1])
            })
            .min_by(f64::total_cmp);
        let rapid_length_mm: f64 = steps
            .iter()
            .map(|s| match s {
                Step::Move(curve, MoveKind::Rapid, _) => {
                    let (a, b) = (curve.start(), curve.end());
                    (a[0] - b[0]).hypot(a[1] - b[1])
                }
                _ => 0.,
            })
            .sum();
        let (mut feed_seconds, mut dwell_seconds) = (0., 0.);
        for step in &steps {
            match step {
                Step::Move(curve, kind, Some(feed)) if !kind.is_rapid() => {
                    feed_seconds += curve.prepare(op.tolerance_mm)?.length_mm() / feed * 60.;
                }
                Step::Event(Event::Dwell { seconds }) => dwell_seconds += seconds,
                _ => {}
            }
        }
        let report = ThermalCutReport {
            strategy: "thermal_cut",
            geometric_residual_status: ResidualStatus::Pending,
            kerf_mm: kerf,
            applied_offset_mm: offset,
            parts: definition.components.len(),
            loops: order.len() - common.len(),
            small_holes: small,
            pierces: order.len() + usize::from(op.crop.is_some()),
            micro_joints,
            cut_length_mm: cut_length,
            order: report_order,
            part_order: op.part_order,
            min_part_step_mm,
            rapid_length_mm,
            leads_moved: op.lead_placement.as_ref().map(|_| leads_moved),
            corner_loops: op.corner_loops.map(|_| corner_loops),
            corner_slowdowns: op.corner_slowdown.map(|_| corner_slowdowns),
            crop_length_mm,
            feed_seconds,
            dwell_seconds,
            rerouted_rapids: op.avoid_cut_parts.map(|_| rerouted),
            loops_completed: 0,
            common_lines,
            diagnostics,
        };
        Ok(Self {
            id: op.id,
            tool: op.tool,
            work_offset: op.work_offset,
            tool_number: op.tool_number,
            length_register: op.length_register,
            initial,
            spec: spec.clone(),
            steps,
            events,
            next_id: first_id,
            expected_generation: None,
            drained: false,
            tolerance: op.tolerance_mm,
            report,
        })
    }
    pub fn report(&self) -> &ThermalCutReport {
        &self.report
    }
    pub fn take_events(&mut self) -> Vec<Event> {
        std::mem::take(&mut self.events)
    }
    pub fn prefix_events(&self) -> Vec<Event> {
        vec![
            Event::WorkOffset {
                number: self.work_offset,
            },
            Event::ToolChange {
                tool: self.tool.clone(),
                number: self.tool_number,
                length_register: self.length_register,
                position_after_mm: self.initial,
            },
        ]
    }
    pub fn suffix_events(&self) -> Vec<Event> {
        vec![]
    }
    pub fn finish_report(&mut self, cancel: &impl CancelSignal) -> Result<(), String> {
        if cancel.is_cancelled() {
            return Err("thermal cut report cancelled".into());
        }
        if !self.drained {
            return Err("drain committed thermal cut motions first".into());
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
            return Err("commit preceding thermal cut motion to matching stock".into());
        }
        if cancel.is_cancelled() {
            return Err("thermal cut cancelled".into());
        }
        while let Some(step) = self.steps.pop_front() {
            match step {
                Step::Event(e) => self.events.push(e),
                Step::Done => self.report.loops_completed += 1,
                Step::Move(curve, kind, feed) => {
                    curve.prepare(self.tolerance)?;
                    let m = Motion {
                        id: self.next_id,
                        operation: self.id.clone(),
                        tool: self.tool.clone(),
                        curve,
                        kind,
                        feed_mm_min: if kind.is_rapid() { None } else { feed },
                    };
                    self.next_id += 1;
                    self.expected_generation = Some(stock.generation() + 1);
                    // Events due right after this move stay queued until the
                    // next call hands them out with take_events.
                    while let Some(Step::Event(_) | Step::Done) = self.steps.front() {
                        match self.steps.pop_front() {
                            Some(Step::Event(e)) => self.events.push(e),
                            Some(Step::Done) => self.report.loops_completed += 1,
                            _ => unreachable!(),
                        }
                    }
                    return Ok(Some(m));
                }
            }
        }
        self.drained = true;
        Ok(None)
    }
}
