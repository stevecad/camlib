//! Deburr: break the horizontal edges of a finished part with a chamfer mill,
//! a ball-end mill or a bull-nose mill.
//!
//! Each level is the union of the part's upward-facing planar faces at one
//! height z_e. A vertical-axis tool swept along a horizontal path at tip
//! height z_t removes exactly the points within horizontal distance
//! rho(z - z_t) of the path, rho being the tool's radius at that height above
//! its tip. With the axis on the outward offset of the face boundary by d, a
//! face point at distance s inside the boundary is at least d + s from the
//! path (exactly that on a convex stretch), so the break is the tool's
//! profile seen in the plane of the edge normal, whatever the edge's
//! curvature:
//!
//! - Chamfer mill: d = tip radius + tip clearance, and the break is the
//!   cone's flank: `width_mm` on the face, the chamfer depth down a vertical
//!   wall.
//! - Ball-end or bull-nose mill (corner radius rc, flat radius a = r - rc,
//!   a = 0 for a ball): in that plane the corner is a circle of radius rc
//!   whose centre sits c outward of the wall and c above the face (on the
//!   edge's outward bisector), d = a + c and z_t = z_e + c - rc. With c >= 0
//!   the flat bottom stays outside the wall and the body above the corner
//!   above the face, so the break is the circle's arc: a concave (cove)
//!   break with equal legs L = sqrt(rc^2 - c^2) - c on the face and down a
//!   vertical wall. The size is given either as the leg (`width_mm`:
//!   c = (sqrt(2 rc^2 - L^2) - L) / 2, L <= rc) or as the radius of the fillet
//!   the break must stay inside (`break_radius_mm` rho: the corner circle
//!   tangent to the fillet's, |C - F| = rc + rho with F = (-rho, -rho), so
//!   c = rc / sqrt(2) - rho (1 - 1 / sqrt(2)), rho <= rc (1 + sqrt(2))). The
//!   report gives the legs and the smallest fillet that contains the break.
//!   Equal legs (the centre on the bisector) is established practice: CAM
//!   deburring with a ball's tip breaks edges symmetrically unless an
//!   asymmetric edge is asked for, and keeps the contact off the tip, where
//!   the cutting speed is zero (Mastercam Deburr, "Asymmetric edge-breaking
//!   option", What's New in Mastercam 2026; `tip_clearance_mm` here).
//!
//! Everything else stays untouched, by an exact proof against the part mesh
//! (the whole tool assembly: cone, neck, shank and holder sections, see
//! `flat::Section`). The obstacles at a level are the part above z_e, the part
//! below the break depth, and anything inside the break's height band that is
//! neither the face itself nor a vertical wall standing on its boundary
//! (those two are exactly what the break cuts). A move that cannot be proven
//! clear is not cut: the run ends there, the tool retracts (proven) and
//! re-enters (proven) where cutting can be proven again, and the skipped
//! stretch is reported. Sections are grown radially by the mesh's stated
//! surface error (and each arc's chord sagitta), so the proof holds for the
//! true part within that error.
use crate::{
    flat::{Index, Section, assembly_sections},
    motion::{Coolant, Curve, Event, Motion, MoveKind, SpindleDirection, Vec3},
    planar::{Boolean, ConstructionOptions, PlanarRegion},
    pocket::{ResidualStatus, Stroke},
    stock::{CancelSignal, Stock, StockSpec},
    surface::SurfaceMesh,
    tool::{CutterShape, ToolAssembly, valid_id},
};
use serde::{Deserialize, Serialize};

mod edges;
pub use edges::{DeburrEdge, DeburrEdgeReport};

fn climb() -> String {
    "climb".into()
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DeburrLevel {
    /// The upward-facing planar faces at this height (unioned).
    pub regions: Vec<PlanarRegion>,
    pub z_mm: f64,
    /// Stretches of the faces' boundaries that are already broken (the face
    /// meets a fillet tangentially): reported as such, not as unreachable.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub broken_edges: Vec<DeburrBrokenEdge>,
}

/// A boundary stretch with no sharp edge (a tangent fillet), as an XY
/// polyline within `error_mm` of the edge.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DeburrBrokenEdge {
    pub points: Vec<[f64; 2]>,
    pub error_mm: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DeburrOperation<G = SurfaceMesh> {
    pub id: String,
    pub tool: String,
    /// The finished part (what must not be cut beyond the breaks).
    pub surface: G,
    pub levels: Vec<DeburrLevel>,
    /// Every convex edge of the part (`camlib.cadquery.deburr_edges`): the
    /// report accounts for each, cut or skipped with a reason, and straight
    /// edges between planar faces that no level covers (sloped edges) are
    /// broken with a ball-end mill.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub edges: Vec<DeburrEdge>,
    /// Break width on the face (horizontal leg). With a ball-end or
    /// bull-nose mill, both legs (face and vertical wall). Give this or
    /// `break_radius_mm`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub width_mm: Option<f64>,
    /// Ball-end or bull-nose mill only: the radius of the fillet the break
    /// must stay inside (the largest such break is cut). Give this or
    /// `width_mm`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub break_radius_mm: Option<f64>,
    /// How far out from the end of the tool's tip (a chamfer mill's tip
    /// flat, a bull-nose mill's flat, a ball's centre line) the break's
    /// lower edge is cut (radially), so the flank or corner cuts, not the
    /// point. A chamfer mill is placed by it; for a ball-end or bull-nose
    /// mill it is the least distance the break may reach.
    pub tip_clearance_mm: f64,
    pub clearance_z_mm: f64,
    /// "climb" (default) or "conventional".
    #[serde(default = "climb")]
    pub cutting_sense: String,
    pub feed_mm_min: f64,
    pub plunge_mm_min: f64,
    pub boundary_tolerance_mm: f64,
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

impl<G> DeburrOperation<G> {
    pub(crate) fn map_geometry<H, E>(
        self,
        f: &mut impl FnMut(G) -> Result<H, E>,
    ) -> Result<DeburrOperation<H>, E> {
        Ok(DeburrOperation {
            surface: f(self.surface)?,
            id: self.id,
            tool: self.tool,
            levels: self.levels,
            edges: self.edges,
            width_mm: self.width_mm,
            break_radius_mm: self.break_radius_mm,
            tip_clearance_mm: self.tip_clearance_mm,
            clearance_z_mm: self.clearance_z_mm,
            cutting_sense: self.cutting_sense,
            feed_mm_min: self.feed_mm_min,
            plunge_mm_min: self.plunge_mm_min,
            boundary_tolerance_mm: self.boundary_tolerance_mm,
            tolerance_mm: self.tolerance_mm,
            spindle_rpm: self.spindle_rpm,
            work_offset: self.work_offset,
            tool_number: self.tool_number,
            length_register: self.length_register,
            coolant: self.coolant,
            retract_z_mm: self.retract_z_mm,
            feed_z_mm: self.feed_z_mm,
            limits: self.limits,
        })
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct SkippedSpan {
    /// The level whose boundary this is (None for a listed edge's span).
    pub level: Option<usize>,
    /// The listed edge this span is on (`DeburrEdge::id`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub edge: Option<String>,
    pub from_mm: Vec3,
    pub to_mm: Vec3,
    pub length_mm: f64,
    pub reason: &'static str,
}

#[derive(Debug, Serialize)]
pub struct DeburrReport {
    pub strategy: &'static str,
    pub geometric_residual_status: ResidualStatus,
    pub levels: usize,
    pub loops: usize,
    pub runs: usize,
    pub requested_width_mm: Option<f64>,
    pub requested_break_radius_mm: Option<f64>,
    /// "chamfer" (chamfer mill) or "arc" (ball-end or bull-nose mill: a
    /// concave arc of the tool's corner radius).
    pub break_profile: &'static str,
    /// Break width on the face (horizontal leg).
    pub face_width_mm: f64,
    /// Break depth down a vertical wall below each level.
    pub wall_depth_mm: f64,
    /// The arc break's radius (the tool's corner radius).
    pub break_arc_radius_mm: Option<f64>,
    /// The smallest fillet radius whose fillet contains the arc break on an
    /// edge between the face and a vertical wall (the requested
    /// `break_radius_mm` when one was given).
    pub fillet_radius_mm: Option<f64>,
    /// The tool axis's distance outward from the face boundary.
    pub axis_offset_mm: f64,
    pub cut_length_mm: f64,
    /// Convex edges not broken (not proven clear of the rest of the part).
    pub unreachable_length_mm: f64,
    /// Concave stretches of the faces' boundaries (a wall rises there):
    /// nothing to break.
    pub concave_length_mm: f64,
    pub skipped_length_mm: f64,
    /// Every stretch not cut, with its reason.
    pub skipped: Vec<SkippedSpan>,
    pub obstacle_triangles: usize,
    /// Boundary stretches already broken (a tangent fillet; no sharp edge).
    pub already_broken_length_mm: f64,
    /// Per level: where the tool ran and the steepest drafted wall broken.
    pub level_details: Vec<DeburrLevelReport>,
    /// Every listed convex edge: cut, partly cut or skipped, with reasons.
    pub edges: Vec<DeburrEdgeReport>,
    pub clearance_proof: &'static str,
}

#[derive(Clone, Debug, Serialize)]
pub struct DeburrLevelReport {
    pub z_mm: f64,
    /// The tool axis's distance outward from the face boundary at this level
    /// (larger than `axis_offset_mm` when a drafted wall is broken, so the
    /// break's lower edge on it keeps the tip clearance).
    pub axis_offset_mm: f64,
    pub tip_z_mm: f64,
    /// The steepest drafted wall below an edge at this level that the break
    /// ends on (0: only vertical walls).
    pub max_wall_draft_deg: f64,
}

const CONCAVE: &str = "concave edge: the part rises above the face there (nothing to break)";
const UNPROVEN: &str = "not proven clear of the rest of the part (whole tool assembly)";
const NO_ENTRY: &str = "no proven straight entry or exit from the feed height";
const ALREADY_BROKEN: &str =
    "already broken: the face meets a fillet tangentially there (no sharp edge)";
const DRAFT_STEEP: &str = "the wall below the edge is drafted too steeply for the chamfer mill (at or near its half angle, or the edge is already chamfered or rounded): the cone cannot end the break on it within its radius";
const DRAFT_ARC: &str = "the wall below the edge is drafted: ball-end and bull-nose breaks are proven against vertical walls only (use a chamfer mill)";

/// XY distance from `p` to the segment a-b.
fn segment_distance(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    stroke_distance(p, &Stroke { a, b, arc: None })
}

/// A feature that explains why a stretch near it was not cut.
struct Label {
    segments: Vec<([f64; 2], [f64; 2])>,
    reach: f64,
    reason: &'static str,
}

/// One continuous cut at a level's tip height.
struct Run {
    curves: Vec<Curve>,
}

pub struct DeburrPlan {
    op: DeburrOperation,
    heights: crate::heights::Heights,
    spec: StockSpec,
    initial: Vec3,
    position: Vec3,
    runs: Vec<Run>,
    run: usize,
    curve: usize,
    // 0 lift, 1 travel, 2 descend, 3 plunge, 4 cut, 5 retract, 6 rise, 7 done
    phase: u8,
    next_id: u64,
    expected_generation: Option<u64>,
    center_cutting: bool,
    tip_radius: f64,
    report: DeburrReport,
    /// How the run splits the straight-up lift after each run (#441).
    lift_split: Option<crate::lift::Lifts>,
}

/// Exact half-space clip of a triangle (keep z >= c when `above`), as
/// triangles.
fn clip(t: [Vec3; 3], c: f64, above: bool) -> Vec<[Vec3; 3]> {
    let inside = |p: &Vec3| if above { p[2] >= c } else { p[2] <= c };
    let mut poly: Vec<Vec3> = Vec::with_capacity(4);
    for k in 0..3 {
        let (p, q) = (t[k], t[(k + 1) % 3]);
        if inside(&p) {
            poly.push(p);
        }
        if inside(&p) != inside(&q) {
            let s = (c - p[2]) / (q[2] - p[2]);
            poly.push([p[0] + s * (q[0] - p[0]), p[1] + s * (q[1] - p[1]), c]);
        }
    }
    (1..poly.len().saturating_sub(1))
        .map(|i| [poly[0], poly[i], poly[i + 1]])
        .collect()
}

/// Exact half-space clip of a triangle (keep n . p >= c), as triangles.
fn clip_half(t: [Vec3; 3], n: Vec3, c: f64) -> Vec<[Vec3; 3]> {
    let side = |p: &Vec3| n[0] * p[0] + n[1] * p[1] + n[2] * p[2] - c;
    let mut poly: Vec<Vec3> = Vec::with_capacity(4);
    for k in 0..3 {
        let (p, q) = (t[k], t[(k + 1) % 3]);
        let (sp, sq) = (side(&p), side(&q));
        if sp >= 0. {
            poly.push(p);
        }
        if (sp >= 0.) != (sq >= 0.) {
            let s = sp / (sp - sq);
            poly.push(std::array::from_fn(|i| p[i] + s * (q[i] - p[i])));
        }
    }
    (1..poly.len().saturating_sub(1))
        .map(|i| [poly[0], poly[i], poly[i + 1]])
        .collect()
}

/// XY distance from `p` to the infinite line through a and b.
fn line_distance(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1]];
    let l = d[0].hypot(d[1]);
    if l == 0. {
        return (p[0] - a[0]).hypot(p[1] - a[1]);
    }
    ((p[0] - a[0]) * d[1] - (p[1] - a[1]) * d[0]).abs() / l
}

/// A piece in the vertical plane of the segment a-b, split into the part
/// over the segment (within `slack` past its ends) and the rest.
fn split_under(
    piece: [Vec3; 3],
    a: [f64; 2],
    b: [f64; 2],
    slack: f64,
) -> (Vec<[Vec3; 3]>, Vec<[Vec3; 3]>) {
    let l = (b[0] - a[0]).hypot(b[1] - a[1]);
    let u = [(b[0] - a[0]) / l, (b[1] - a[1]) / l, 0.];
    let (lo, hi) = (
        u[0] * a[0] + u[1] * a[1] - slack,
        u[0] * b[0] + u[1] * b[1] + slack,
    );
    let minus = [-u[0], -u[1], 0.];
    let mut under = vec![];
    for p in clip_half(piece, u, lo) {
        under.extend(clip_half(p, minus, -hi));
    }
    let mut beyond = clip_half(piece, minus, -lo);
    beyond.extend(clip_half(piece, u, hi));
    (under, beyond)
}

/// A triangle's (unnormalised) normal.
fn normal(t: &[Vec3; 3]) -> Vec3 {
    let u: Vec3 = std::array::from_fn(|k| t[1][k] - t[0][k]);
    let v: Vec3 = std::array::from_fn(|k| t[2][k] - t[0][k]);
    [
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    ]
}

/// Horizontal distance from `p` to a stroke (line or arc).
fn stroke_distance(p: [f64; 2], s: &Stroke) -> f64 {
    let segment = |a: [f64; 2], b: [f64; 2]| {
        let d = [b[0] - a[0], b[1] - a[1]];
        let l2 = d[0] * d[0] + d[1] * d[1];
        let t = if l2 > 0. {
            (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / l2).clamp(0., 1.)
        } else {
            0.
        };
        (a[0] + t * d[0] - p[0]).hypot(a[1] + t * d[1] - p[1])
    };
    match s.arc {
        None => segment(s.a, s.b),
        Some((c, sweep)) => {
            let r = (s.a[0] - c[0]).hypot(s.a[1] - c[1]);
            let a0 = (s.a[1] - c[1]).atan2(s.a[0] - c[0]);
            let ap = (p[1] - c[1]).atan2(p[0] - c[0]);
            let rel = if sweep > 0. {
                (ap - a0).rem_euclid(std::f64::consts::TAU)
            } else {
                (a0 - ap).rem_euclid(std::f64::consts::TAU)
            };
            if rel <= sweep.abs() {
                ((p[0] - c[0]).hypot(p[1] - c[1]) - r).abs()
            } else {
                (p[0] - s.a[0])
                    .hypot(p[1] - s.a[1])
                    .min((p[0] - s.b[0]).hypot(p[1] - s.b[1]))
            }
        }
    }
}

fn signed_area(strokes: &[Stroke]) -> f64 {
    // Shoelace over a fine polyline of the loop (arcs sampled; the sign is
    // all that is used).
    let mut points = vec![];
    for s in strokes {
        let n = if s.arc.is_some() { 16 } else { 1 };
        for i in 0..n {
            points.push(crate::profiles::stroke_point(s, i as f64 / n as f64));
        }
    }
    let mut area = 0.;
    for i in 0..points.len() {
        let (p, q) = (points[i], points[(i + 1) % points.len()]);
        area += p[0] * q[1] - q[0] * p[1];
    }
    area / 2.
}

fn reversed(strokes: &[Stroke]) -> Vec<Stroke> {
    strokes
        .iter()
        .rev()
        .map(|s| Stroke {
            a: s.b,
            b: s.a,
            arc: s.arc.map(|(c, sweep)| (c, -sweep)),
        })
        .collect()
}

fn length(curve: &Curve) -> f64 {
    match *curve {
        Curve::Line { start, end } => (end[0] - start[0])
            .hypot(end[1] - start[1])
            .hypot(end[2] - start[2]),
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
        } => (start[0] - center[0]).hypot(start[1] - center[1]) * sweep_rad.abs(),
    }
}

/// Proves curves clear of one level's obstacles for the whole assembly.
struct Prover {
    index: Index,
    sections: Vec<Section>,
    /// Radial growth: the mesh's surface error and the stock grid.
    grow: f64,
    /// Vertical allowance: the mesh's surface error.
    lift: f64,
    /// Arc sub-chord sagitta budget.
    sagitta: f64,
    seen: Vec<u32>,
    stamp: u32,
    /// The whole part, to tell concave stretches from unreachable ones.
    part: Option<Index>,
    part_seen: Vec<u32>,
    part_stamp: u32,
    /// Already-broken edges and drafted walls the tool cannot break, in
    /// the order their reasons apply.
    labels: Vec<Label>,
}

impl Prover {
    /// Why a stretch at `p` (on the tool path) was not cut, beyond being
    /// concave: a nearby label, else not proven.
    fn reason(&self, p: Vec3) -> &'static str {
        let q = [p[0], p[1]];
        self.labels
            .iter()
            .find(|l| {
                l.segments
                    .iter()
                    .any(|(a, b)| segment_distance(q, *a, *b) <= l.reach)
            })
            .map_or(UNPROVEN, |l| l.reason)
    }
    /// Whether the part rises above `z` under the point (a tool centre on a
    /// concave stretch lies inside the wall rising from the face).
    fn inside_part_above(&mut self, p: Vec3, z: f64) -> bool {
        let probe = [Section {
            z0: 0.,
            r0: 1e-6,
            r1: 1e-6,
            rise: 0.,
            arc: 0.,
        }];
        self.part.as_ref().is_some_and(|part| {
            part.max_sections(
                [p[0], p[1]],
                &probe,
                &mut self.part_seen,
                &mut self.part_stamp,
            ) > z
        })
    }
    fn grown(&self, extra: f64) -> Vec<Section> {
        self.sections
            .iter()
            .map(|s| Section {
                z0: s.z0,
                r0: s.r0 + self.grow + extra,
                r1: s.r1 + self.grow + extra,
                rise: s.rise,
                arc: s.arc,
            })
            .collect()
    }
    fn line(&mut self, a: Vec3, b: Vec3, extra: f64) -> bool {
        let sections = self.grown(extra);
        self.index
            .sections_chord_clear(a, b, &sections, self.lift, &mut self.seen, &mut self.stamp)
    }
    fn curve(&mut self, curve: &Curve) -> Result<bool, String> {
        match *curve {
            Curve::Line { start, end } => Ok(self.line(start, end, 0.)),
            Curve::Arc {
                start,
                center,
                sweep_rad,
                ..
            } => {
                // Sub-chords whose sagitta is within budget; each chord is
                // proven with the sections grown by that sagitta, which
                // contain the tool anywhere on its sub-arc.
                let r = (start[0] - center[0]).hypot(start[1] - center[1]);
                let step = 2. * (1. - self.sagitta / r).clamp(-1., 1.).acos();
                let n = (sweep_rad.abs() / step.max(1e-9)).ceil().max(1.);
                if n > 1e6 {
                    return Err("deburr arc needs too many proof chords".into());
                }
                let n = n as usize;
                let s = r * (1. - (sweep_rad.abs() / n as f64 / 2.).cos());
                let a0 = (start[1] - center[1]).atan2(start[0] - center[0]);
                let at = |i: usize| {
                    let t = a0 + sweep_rad * i as f64 / n as f64;
                    [center[0] + r * t.cos(), center[1] + r * t.sin(), start[2]]
                };
                for i in 0..n {
                    if !self.line(at(i), at(i + 1), s) {
                        return Ok(false);
                    }
                }
                Ok(true)
            }
            Curve::Helix { .. } => Ok(false),
        }
    }
}

/// Where the tool runs for a break, and the break it makes on an edge
/// between the face and a vertical wall below it.
struct BreakGeometry {
    /// Axis distance outward from the face boundary.
    offset: f64,
    /// The face's height above the tool tip.
    top: f64,
    /// Leg on the face.
    face: f64,
    /// Leg down a vertical wall.
    wall: f64,
    /// The corner radius of an arc break (None: a chamfer).
    arc: Option<f64>,
    /// The smallest fillet containing an arc break.
    fillet: Option<f64>,
}

impl BreakGeometry {
    fn of(op: &DeburrOperation, tool: &crate::tool::PreparedAssembly) -> Result<Self, String> {
        let cutter = tool.cutter().definition();
        let size = |v: Option<f64>, name: &str| match v {
            Some(v) if !(v.is_finite() && v > 0.) => {
                Err(format!("deburr {name} {v} must be finite and positive"))
            }
            v => Ok(v),
        };
        let (width, radius) = (
            size(op.width_mm, "width_mm")?,
            size(op.break_radius_mm, "break_radius_mm")?,
        );
        if !(op.tip_clearance_mm.is_finite() && op.tip_clearance_mm > 0.) {
            return Err(format!(
                "deburr tip_clearance_mm {} must be finite and positive",
                op.tip_clearance_mm
            ));
        }
        match cutter.shape {
            CutterShape::Chamfer {
                tip_diameter_mm, ..
            } => {
                let (Some(width), None) = (width, radius) else {
                    return Err(
                        "deburr with a chamfer mill needs width_mm (the break width on the face); break_radius_mm applies to ball-end and bull-nose mills"
                            .into(),
                    );
                };
                let d = tip_diameter_mm / 2. + op.tip_clearance_mm;
                let reference = tool
                    .cutter()
                    .tip_height_at_radius(d)
                    .ok_or("deburr tip clearance exceeds the cone radius")?;
                let top = tool
                    .cutter()
                    .tip_height_at_radius(d + width)
                    .ok_or("deburr width exceeds the chamfer mill's cone radius")?;
                Ok(Self {
                    offset: d,
                    top,
                    face: width,
                    wall: top - reference,
                    arc: None,
                    fillet: None,
                })
            }
            CutterShape::Ball | CutterShape::Bull { .. } => {
                let r = cutter.diameter_mm / 2.;
                let rc = match cutter.shape {
                    CutterShape::Bull { corner_radius_mm } => corner_radius_mm,
                    _ => r,
                };
                let half = std::f64::consts::FRAC_1_SQRT_2;
                // The corner circle's centre: c outward of the wall, c above
                // the face (see the module notes).
                let c = match (width, radius) {
                    (Some(leg), None) => {
                        if leg > rc {
                            return Err(format!(
                                "deburr width_mm {leg} exceeds the tool's corner radius {rc}: with equal legs a ball-end or bull-nose mill breaks at most its corner radius"
                            ));
                        }
                        ((2. * rc * rc - leg * leg).sqrt() - leg) / 2.
                    }
                    (None, Some(rho)) => {
                        if rho > rc * (1. + std::f64::consts::SQRT_2) {
                            return Err(format!(
                                "deburr break_radius_mm {rho} exceeds {:.4} (the tool's corner radius {rc} times 1 + sqrt 2): the break would need the corner's centre inside the part",
                                rc * (1. + std::f64::consts::SQRT_2)
                            ));
                        }
                        rc * half - rho * (1. - half)
                    }
                    _ => {
                        return Err(
                            "deburr with a ball-end or bull-nose mill needs exactly one of width_mm (equal legs) and break_radius_mm (the fillet the break stays inside)"
                                .into(),
                        );
                    }
                };
                if c < op.tip_clearance_mm {
                    return Err(format!(
                        "deburr: this break's lower edge is cut {c:.4} mm out from the end of the tool's flat, inside tip_clearance_mm {}: use a smaller break, a larger corner radius or a smaller tip clearance",
                        op.tip_clearance_mm
                    ));
                }
                let leg = (rc * rc - c * c).sqrt() - c;
                // The smallest fillet (centre F = (-rho, -rho)) clear of the
                // corner circle (centre (c, c), radius rc): |C - F| >= rc + rho,
                // and at least each leg.
                let b = 2. * c - rc;
                let disk = -b + (b * b - (2. * c * c - rc * rc)).sqrt();
                let top = rc - c;
                if top > cutter.cutting_length_mm {
                    return Err(format!(
                        "deburr: the break reaches {top:.4} mm above the tool tip, beyond the cutting length {}",
                        cutter.cutting_length_mm
                    ));
                }
                Ok(Self {
                    offset: r - rc + c,
                    top,
                    face: leg,
                    wall: leg,
                    arc: Some(rc),
                    fillet: Some(disk.max(leg)),
                })
            }
            _ => Err(
                "deburr needs a chamfer mill (camlib.tools.ChamferMill), a ball-end mill (BallEndMill) or a bull-nose mill (BullEndMill)"
                    .into(),
            ),
        }
    }
}

impl DeburrPlan {
    pub fn prepare(
        op: DeburrOperation,
        assembly: &ToolAssembly,
        spec: &StockSpec,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        if cancel.is_cancelled() {
            return Err("deburr planning cancelled".into());
        }
        spec.grid()?;
        let actual = assembly.prepare()?;
        let cutter = actual.cutter().definition();
        let tip_radius = match cutter.shape {
            CutterShape::Chamfer {
                tip_diameter_mm, ..
            } => tip_diameter_mm / 2.,
            CutterShape::Bull { corner_radius_mm } => cutter.diameter_mm / 2. - corner_radius_mm,
            _ => 0.,
        };
        let geometry = BreakGeometry::of(&op, &actual)?;
        let (d, top, depth) = (geometry.offset, geometry.top, geometry.wall);
        let guard = 16. * op.tolerance_mm;
        let e = op.surface.surface_error_mm;
        let surface_top = op
            .surface
            .triangles
            .iter()
            .flatten()
            .map(|p| p[2])
            .fold(f64::NEG_INFINITY, f64::max);
        crate::heights::require(
            "deburr",
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
                    [
                        geometry.face,
                        op.tip_clearance_mm,
                        op.clearance_z_mm,
                        op.feed_mm_min,
                        op.plunge_mm_min,
                        op.boundary_tolerance_mm,
                        op.tolerance_mm,
                        op.spindle_rpm,
                    ]
                    .iter()
                    .all(|v| v.is_finite())
                        && initial.iter().all(|v| v.is_finite() && v.abs() <= 1e6),
                    "all numeric parameters must be finite".into(),
                ),
                (
                    (1e-7..=0.01).contains(&op.tolerance_mm),
                    format!("tolerance_mm {} must be within 1e-7..0.01", op.tolerance_mm),
                ),
                (
                    (0.0001..=0.1).contains(&op.boundary_tolerance_mm)
                        && op.tolerance_mm <= op.boundary_tolerance_mm / 64.,
                    format!(
                        "boundary_tolerance_mm {} must be within 0.0001..0.1 and at least 64 x tolerance_mm",
                        op.boundary_tolerance_mm
                    ),
                ),
                (
                    geometry.face > op.boundary_tolerance_mm + guard,
                    format!(
                        "the break width on the face {} must exceed boundary_tolerance_mm",
                        geometry.face
                    ),
                ),
                (
                    op.tip_clearance_mm > guard,
                    format!("tip_clearance_mm {} must be positive", op.tip_clearance_mm),
                ),
                (
                    e.is_finite() && (0. ..=0.1).contains(&e),
                    format!("surface_error_mm {e} must be within 0..0.1"),
                ),
                (
                    (!op.levels.is_empty() || !op.edges.is_empty())
                        && op.levels.iter().all(|l| !l.regions.is_empty()),
                    "deburr needs at least one level (with at least one face) or listed edge"
                        .into(),
                ),
                (
                    op.levels.iter().all(|l| {
                        l.z_mm.is_finite()
                            && l.z_mm <= spec.max_mm[2] + guard
                            && l.z_mm - top >= spec.min_mm[2]
                    }),
                    format!(
                        "every level must be within the stock with room for the tool tip {top:.4} mm below it"
                    ),
                ),
                (
                    surface_top <= spec.max_mm[2] + guard,
                    format!(
                        "the part (top Z{surface_top:.4}) must lie below the stock top {}",
                        spec.max_mm[2]
                    ),
                ),
                (
                    op.clearance_z_mm > spec.max_mm[2] + guard && op.clearance_z_mm.abs() <= 1e6,
                    format!(
                        "clearance_z_mm {} must be above the stock top {}",
                        op.clearance_z_mm, spec.max_mm[2]
                    ),
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
                    "feeds and spindle_rpm must be positive".into(),
                ),
                (
                    matches!(op.cutting_sense.as_str(), "climb" | "conventional"),
                    format!(
                        "cutting_sense {:?} must be \"climb\" or \"conventional\"",
                        op.cutting_sense
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
            ],
        )?;
        let heights = crate::heights::Heights::resolve(
            op.clearance_z_mm,
            op.retract_z_mm,
            op.feed_z_mm,
            spec.max_mm[2],
            guard,
        )?;
        let limits = match op.limits.as_ref().filter(|l| !l.is_empty()) {
            Some(l) => Some(crate::limits::PreparedLimits::new(
                l,
                cutter.diameter_mm / 2.,
                op.tolerance_mm,
            )?),
            None => None,
        };
        let sections = assembly_sections(&actual)?;
        let rise = sections.iter().map(|s| s.rise).fold(0., f64::max);
        let reach = sections.iter().map(|s| s.r1).fold(0., f64::max) + e + 1.;
        let band = guard.max(e);
        let options = ConstructionOptions {
            tolerance_mm: op.tolerance_mm,
        };
        let climb = op.cutting_sense == "climb";
        let mut report = DeburrReport {
            strategy: "deburr_horizontal_edges",
            geometric_residual_status: ResidualStatus::Pending,
            levels: op.levels.len(),
            loops: 0,
            runs: 0,
            requested_width_mm: op.width_mm,
            requested_break_radius_mm: op.break_radius_mm,
            break_profile: if geometry.arc.is_some() {
                "arc"
            } else {
                "chamfer"
            },
            face_width_mm: geometry.face,
            wall_depth_mm: depth,
            break_arc_radius_mm: geometry.arc,
            fillet_radius_mm: geometry.fillet,
            axis_offset_mm: d,
            cut_length_mm: 0.,
            unreachable_length_mm: 0.,
            concave_length_mm: 0.,
            skipped_length_mm: 0.,
            skipped: vec![],
            obstacle_triangles: 0,
            already_broken_length_mm: 0.,
            level_details: vec![],
            edges: vec![],
            clearance_proof: "exact section contact (cone or corner arc, neck, shank, holder) against the part above the level, below the break and any non-face/non-wall geometry in the break band; sections grown by the surface error, two stock cell diagonals and each arc chord's sagitta",
        };
        edges::validate(&op.edges)?;
        let mut runs = vec![];
        // Every level piece as cut (None) or skipped (its reason), and each
        // level's height, boundary and tolerance, for the edge accounting.
        let mut record: Vec<(usize, Curve, Option<&'static str>)> = vec![];
        let mut level_boundaries: Vec<(f64, Vec<Stroke>, f64)> = vec![];
        let min_piece = (op.boundary_tolerance_mm).max(guard);
        for (li, level) in op.levels.iter().enumerate() {
            if cancel.is_cancelled() {
                return Err("deburr planning cancelled".into());
            }
            let z_e = level.z_mm;
            // The face: union of the level's regions.
            let mut face = level.regions[0].prepare(options)?;
            for r in &level.regions[1..] {
                let next = face.boolean_prepared(&r.prepare(options)?, Boolean::Union)?;
                face = next.into_parts().1;
            }
            let error = face.summary().boundary_error_mm;
            let margin = op.boundary_tolerance_mm / 2.;
            if 2. * error + margin + guard > op.boundary_tolerance_mm {
                return Err(format!(
                    "deburr level {li} (Z{z_e}): face boundary uncertainty {error} exceeds the boundary_tolerance_mm budget"
                ));
            }
            let boundary_strokes: Vec<Stroke> = face
                .definition()
                .components
                .iter()
                .flat_map(|c| std::iter::once(&c.outer).chain(&c.holes))
                .flat_map(Stroke::from_boundary)
                .collect();
            // Obstacles: above the level, below the break, and anything in
            // the band that is neither the face nor a wall on its boundary.
            // Radial margin: the mesh's surface error, and the stock grid.
            // The simulation keeps an upper height per cell and certifies a
            // cut only where one sweep covers a whole cell, so a cell next
            // to a wall may stay at the wall's height; it checks a rectangle
            // padded by the cutter radius and half a cell diagonal. Two cell
            // diagonals keep the tool out of that cell.
            let grow = e + 2. * spec.cell_size_mm * std::f64::consts::SQRT_2;
            // The wall is cut down to where the grown tool's end reaches
            // the wall's distance d from the axis.
            let vertical_bottom = match geometry.arc {
                None => z_e - depth - grow * rise,
                Some(_) => z_e - top + sections[0].profile((d - grow).max(0.)).unwrap_or(0.),
            };
            let high = z_e + band;
            let slack = e + op.boundary_tolerance_mm + error + guard;
            let boundary = |p: Vec3| {
                boundary_strokes
                    .iter()
                    .map(|s| stroke_distance([p[0], p[1]], s))
                    .fold(f64::INFINITY, f64::min)
            };
            let on_boundary = |p: Vec3| boundary(p) <= slack;
            level_boundaries.push((z_e, boundary_strokes.clone(), slack.max(band)));
            // A wall hanging off the boundary below the edge (drafted at
            // beta from vertical: each point s = depth tan(beta) out from
            // the boundary; at a convex corner between drafted walls up to
            // sqrt 2 times that). For a non-vertical piece, k is the least
            // slope with s <= depth k + slack at its vertices and centroid
            // (the same slack as a vertical wall). The cone (half angle
            // alpha) at offset X reaches a point only if its distance from
            // the path, at least X - s, is below X + width - depth
            // tan(alpha): only above the depth (width + slack) / (tan(alpha)
            // - k), the proof's grown and lowered tool a little deeper. The
            // piece is excluded down to there and is an obstacle below, so
            // the proof catches anything deeper. Returns (k, the piece's
            // own draft slope, whether it touches the edge).
            let hanging = |piece: &[Vec3; 3]| -> Option<(f64, f64, bool)> {
                let n = normal(piece);
                let (horizontal, up) = (n[0].hypot(n[1]), n[2].abs());
                if horizontal == 0. || up <= 1e-9 * horizontal {
                    return None;
                }
                let mid = std::array::from_fn(|k| (piece[0][k] + piece[1][k] + piece[2][k]) / 3.);
                let (mut k, mut touches) = (0f64, false);
                for p in piece.iter().chain([&mid]) {
                    let (depth, s) = (z_e - p[2], boundary(*p));
                    if depth <= band {
                        // At the edge's height: on the boundary.
                        if s > slack {
                            return None;
                        }
                        touches = true;
                    } else {
                        k = k.max((s - slack).max(0.) / depth);
                    }
                }
                Some((k, up / horizontal, touches))
            };
            // The steepest draft whose break still fits the cone: the axis
            // offset X(s) = d + width s / (tan(alpha) - s) plus the width
            // within the cone's radius r: s <= tan(alpha) K / (width + K),
            // K = r - width - d. None for a ball or bull-nose end.
            let width = geometry.face;
            let tan_alpha = match geometry.arc {
                None => 1. / sections[0].rise,
                Some(_) => 0.,
            };
            let room = (sections[0].r1 - width - d).max(0.);
            let max_slope = tan_alpha * room / (width + room);
            // Drafted walls hanging off the edge (below it, within the
            // vertical wall's band): the axis moves out so the break's
            // lower edge on the steepest keeps the tip clearance.
            let mut steepest = 0f64;
            let mut labels = vec![];
            if !level.broken_edges.is_empty() {
                let mut segments = vec![];
                let mut reach: f64 = 0.;
                for b in &level.broken_edges {
                    if b.points.len() < 2
                        || b.points.iter().flatten().any(|v| !v.is_finite())
                        || !(0. ..=0.1).contains(&b.error_mm)
                    {
                        return Err(format!(
                            "deburr level {li}: a broken edge needs at least two finite points and error_mm within 0..0.1"
                        ));
                    }
                    reach = reach.max(b.error_mm);
                    segments.extend(b.points.windows(2).map(|w| (w[0], w[1])));
                }
                labels.push(Label {
                    segments,
                    reach,
                    reason: ALREADY_BROKEN,
                });
            }
            let mut steep = vec![];
            let (mut drafted, mut draft_slope) = (false, 0f64);
            for t in op.surface.triangles.iter() {
                for piece in clip(*t, high, false)
                    .into_iter()
                    .flat_map(|p| clip(p, vertical_bottom - band, true))
                {
                    if piece.iter().all(|p| (p[2] - z_e).abs() <= band) {
                        continue;
                    }
                    if let Some((k, slope, touches)) = hanging(&piece) {
                        if k <= max_slope {
                            drafted = true;
                            steepest = steepest.max(k);
                            draft_slope = draft_slope.max(slope);
                        } else if touches {
                            steep.extend((0..3).map(|k| {
                                (
                                    [piece[k][0], piece[k][1]],
                                    [piece[(k + 1) % 3][0], piece[(k + 1) % 3][1]],
                                )
                            }));
                        }
                    }
                }
            }
            let d = d + width * steepest / (tan_alpha - steepest).max(f64::MIN_POSITIVE);
            let top = match geometry.arc {
                None => actual.cutter().tip_height_at_radius(d + width).ok_or(format!(
                    "deburr level {li} (Z{z_e}): the axis offset {d:.4} for its drafted walls plus the width exceeds the chamfer mill's cone radius"
                ))?,
                Some(_) => top,
            };
            let tip = z_e - top;
            if tip < spec.min_mm[2] {
                return Err(format!(
                    "deburr level {li} (Z{z_e}): the tool tip {tip:.4} would be below the stock"
                ));
            }
            // The proof also lowers the tool by the surface error (lift),
            // which widens the cone by e tan(alpha) at every height.
            let depth_for =
                |slope: f64| (width + grow + slack + e * tan_alpha) / (tan_alpha - slope);
            let low = if drafted {
                (vertical_bottom - band).min(z_e - depth_for(steepest) - band)
            } else {
                vertical_bottom - band
            };
            // camlib offsets are positive inward (clearance): grow outward.
            let center = face.offset(-(d + error + margin))?.region;
            let mut obstacles = vec![];
            for t in op.surface.triangles.iter() {
                obstacles.extend(clip(*t, high, true));
                obstacles.extend(clip(*t, low, false));
                for piece in clip(*t, high, false)
                    .into_iter()
                    .flat_map(|p| clip(p, low, true))
                {
                    let horizontal = piece.iter().all(|p| (p[2] - z_e).abs() <= band);
                    let n = normal(&piece);
                    let norm = n[0].hypot(n[1]).hypot(n[2]);
                    let vertical = norm > 0. && n[2].abs() <= 1e-9 * norm;
                    let mid =
                        std::array::from_fn(|k| (piece[0][k] + piece[1][k] + piece[2][k]) / 3.);
                    if horizontal || norm == 0. {
                        continue;
                    }
                    // Excluded down to `bottom`: the break ends above it.
                    let bottom =
                        if vertical && piece.iter().all(|p| on_boundary(*p)) && on_boundary(mid) {
                            Some(vertical_bottom - band)
                        } else {
                            hanging(&piece)
                                .filter(|(k, ..)| drafted && *k <= steepest)
                                .map(|(k, ..)| z_e - depth_for(k) - band)
                        };
                    match bottom {
                        // Wholly inside the excluded band.
                        Some(b) if piece.iter().all(|p| p[2] >= b) => {}
                        Some(b) => obstacles.extend(clip(piece, b, false)),
                        // A vertical wall reaching past a straight stretch of
                        // the boundary in its plane (an end wall that also
                        // closes a fillet's or chamfer's profile): the part
                        // under the stretch is the wall, the rest an
                        // obstacle.
                        None if vertical => {
                            let line = boundary_strokes.iter().find(|s| {
                                s.arc.is_none()
                                    && piece
                                        .iter()
                                        .all(|p| line_distance([p[0], p[1]], s.a, s.b) <= slack)
                            });
                            match line {
                                Some(s) => {
                                    let (under, beyond) = split_under(piece, s.a, s.b, slack);
                                    let b = vertical_bottom - band;
                                    for p in under {
                                        obstacles.extend(clip(p, b, false));
                                    }
                                    obstacles.extend(beyond);
                                }
                                None => obstacles.push(piece),
                            }
                        }
                        None => obstacles.push(piece),
                    }
                }
            }
            if !steep.is_empty() {
                labels.push(Label {
                    segments: steep,
                    reach: d + slack,
                    reason: if geometry.arc.is_some() {
                        DRAFT_ARC
                    } else {
                        DRAFT_STEEP
                    },
                });
            }
            for l in &mut labels {
                if l.reason == ALREADY_BROKEN {
                    l.reach += d + slack;
                }
            }
            report.level_details.push(DeburrLevelReport {
                z_mm: z_e,
                axis_offset_mm: d,
                tip_z_mm: tip,
                max_wall_draft_deg: draft_slope.atan().to_degrees(),
            });
            report.obstacle_triangles += obstacles.len();
            let mesh = SurfaceMesh {
                version: 1,
                units: "mm".into(),
                triangles: std::sync::Arc::new(obstacles),
                surface_error_mm: e,
                source: serde_json::json!({ "deburr_obstacles": li }),
            };
            let index = Index::new(&mesh, spec.min_mm[2] - 1e3, reach, cancel)?;
            let part = Index::new(&op.surface, spec.min_mm[2] - 1e3, reach, cancel)?;
            let mut prover = Prover {
                seen: vec![0; index.len()],
                index,
                part_seen: vec![0; part.len()],
                part: Some(part),
                part_stamp: 0,
                sections: sections.clone(),
                grow,
                lift: e,
                sagitta: band,
                stamp: 0,
                labels,
            };
            for component in &center.components {
                for (hole, b) in std::iter::once((false, &component.outer))
                    .chain(component.holes.iter().map(|h| (true, h)))
                {
                    let mut strokes = Stroke::from_boundary(b);
                    if strokes.is_empty() {
                        continue;
                    }
                    // Climb with M3: clockwise around material (outer
                    // boundaries), counter-clockwise inside holes.
                    let ccw = signed_area(&strokes) > 0.;
                    let want_ccw = hole == climb;
                    if ccw != want_ccw {
                        strokes = reversed(&strokes);
                    }
                    report.loops += 1;
                    Self::plan_loop(
                        li,
                        &strokes,
                        tip,
                        heights.feed,
                        z_e - tip,
                        &mut prover,
                        limits.as_ref(),
                        min_piece,
                        &mut runs,
                        &mut report,
                        &mut record,
                        cancel,
                    )?;
                }
            }
        }
        // Every listed convex edge: a horizontal one on a level's boundary
        // is accounted from that level's pieces; a straight one no level
        // covers is broken with a ball-end mill; the rest are skipped with a
        // reason.
        let ball = match cutter.shape {
            CutterShape::Ball => Some(cutter.diameter_mm / 2.),
            _ => None,
        };
        let grow = e + 2. * spec.cell_size_mm * std::f64::consts::SQRT_2;
        for edge in &op.edges {
            if cancel.is_cancelled() {
                return Err("deburr planning cancelled".into());
            }
            let total = edges::polyline_length(&edge.points);
            let level = if edge.kind == "horizontal" {
                level_boundaries.iter().position(|(z, strokes, slack)| {
                    edge.points.iter().all(|p| {
                        (p[2] - z).abs() <= *slack
                            && strokes
                                .iter()
                                .map(|s| stroke_distance([p[0], p[1]], s))
                                .fold(f64::INFINITY, f64::min)
                                <= slack + edge.error_mm
                    })
                })
            } else {
                None
            };
            let skipped = |reason: &'static str| DeburrEdgeReport {
                id: edge.id.clone(),
                kind: edge.kind.clone(),
                length_mm: total,
                status: "skipped",
                cut_length_mm: 0.,
                reasons: vec![reason],
            };
            let entry = if let Some(li) = level {
                edges::account_level(edge, total, li, &record)
            } else if edge.kind == "vertical" {
                skipped(edges::VERTICAL)
            } else if let Some(r) = ball {
                match edges::straight_break(
                    edge,
                    r,
                    op.width_mm,
                    op.break_radius_mm,
                    op.tip_clearance_mm,
                    &op.surface.triangles,
                    e,
                    grow + band,
                    band,
                ) {
                    Err(reason) => skipped(reason),
                    Ok(path) => {
                        let mesh = SurfaceMesh {
                            version: 1,
                            units: "mm".into(),
                            triangles: std::sync::Arc::new(path.obstacles),
                            surface_error_mm: e,
                            source: serde_json::json!({ "deburr_edge": edge.id }),
                        };
                        let index = Index::new(&mesh, spec.min_mm[2] - 1e3, reach, cancel)?;
                        report.obstacle_triangles += index.len();
                        let mut prover = Prover {
                            seen: vec![0; index.len()],
                            index,
                            part: None,
                            part_seen: vec![],
                            part_stamp: 0,
                            sections: sections.clone(),
                            grow,
                            lift: e,
                            sagitta: band,
                            stamp: 0,
                            labels: vec![],
                        };
                        let first = runs.len();
                        let skipped_before = report.skipped.len();
                        Self::plan_edge(
                            &edge.id,
                            path.start,
                            path.end,
                            heights.feed,
                            &mut prover,
                            limits.as_ref(),
                            min_piece,
                            &mut runs,
                            &mut report,
                            cancel,
                        )?;
                        let path_length = length(&Curve::Line {
                            start: path.start,
                            end: path.end,
                        });
                        let cut: f64 = runs[first..]
                            .iter()
                            .flat_map(|r| r.curves.iter())
                            .map(length)
                            .sum();
                        let mut reasons: Vec<&'static str> = vec![];
                        for s in &report.skipped[skipped_before..] {
                            if !reasons.contains(&s.reason) {
                                reasons.push(s.reason);
                            }
                        }
                        // The path is the edge translated: lengths agree.
                        let cut = (cut / path_length * total).min(total);
                        DeburrEdgeReport {
                            id: edge.id.clone(),
                            kind: edge.kind.clone(),
                            length_mm: total,
                            status: edges::status(cut, total, min_piece),
                            cut_length_mm: cut,
                            reasons,
                        }
                    }
                }
            } else {
                skipped(if edge.straight {
                    edges::TOOL
                } else {
                    edges::CURVED
                })
            };
            report.edges.push(entry);
        }
        report.runs = runs.len();
        report.cut_length_mm = runs
            .iter()
            .flat_map(|r: &Run| r.curves.iter())
            .map(length)
            .sum();
        report.skipped_length_mm = report.skipped.iter().map(|s| s.length_mm).sum();
        report.concave_length_mm = report
            .skipped
            .iter()
            .filter(|s| s.reason == CONCAVE)
            .map(|s| s.length_mm)
            .sum();
        report.already_broken_length_mm = report
            .skipped
            .iter()
            .filter(|s| s.reason == ALREADY_BROKEN)
            .map(|s| s.length_mm)
            .sum();
        report.unreachable_length_mm =
            report.skipped_length_mm - report.concave_length_mm - report.already_broken_length_mm;
        if first_id == 0 {
            return Err("deburr motion ids start at 1".into());
        }
        Ok(Self {
            heights,
            spec: spec.clone(),
            initial,
            position: initial,
            runs,
            run: 0,
            curve: 0,
            phase: 0,
            next_id: first_id,
            expected_generation: None,
            center_cutting: cutter.center_cutting,
            tip_radius,
            report,
            lift_split: None,
            op,
        })
    }

    /// Proven pieces of one loop at the tip height, as runs (a closed loop
    /// proven all round is one run; a run crossing the loop's start joins).
    #[allow(clippy::too_many_arguments)]
    fn plan_loop(
        level: usize,
        strokes: &[Stroke],
        tip: f64,
        feed: f64,
        band: f64,
        prover: &mut Prover,
        limits: Option<&crate::limits::PreparedLimits>,
        min_piece: f64,
        runs: &mut Vec<Run>,
        report: &mut DeburrReport,
        record: &mut Vec<(usize, Curve, Option<&'static str>)>,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        let mut pieces: Vec<(Curve, bool)> = vec![];
        for s in strokes {
            let mut stack = vec![(0., 1.)];
            while let Some((t0, t1)) = stack.pop() {
                if cancel.is_cancelled() {
                    return Err("deburr planning cancelled".into());
                }
                let curve = crate::profiles::sub_curve(s, t0, t1, tip, tip);
                let ok = prover.curve(&curve)?
                    && limits.is_none_or(|l| l.allows_curve(&curve, true).unwrap_or(false));
                if ok || length(&curve) <= min_piece {
                    pieces.push((curve, ok));
                } else {
                    let m = (t0 + t1) / 2.;
                    stack.push((m, t1));
                    stack.push((t0, m));
                }
            }
        }
        let mut loop_runs: Vec<Vec<Curve>> = vec![];
        let mut current: Vec<Curve> = vec![];
        for (curve, ok) in pieces.iter().cloned() {
            if ok {
                current.push(curve);
            } else {
                if !current.is_empty() {
                    loop_runs.push(std::mem::take(&mut current));
                }
                let mid = match &curve {
                    Curve::Line { start, end } => std::array::from_fn(|k| (start[k] + end[k]) / 2.),
                    other => other.start(),
                };
                let reason = if prover.inside_part_above(mid, tip + band) {
                    CONCAVE
                } else {
                    prover.reason(mid)
                };
                record.push((level, curve.clone(), Some(reason)));
                match report.skipped.last_mut() {
                    Some(last)
                        if last.level == Some(level)
                            && last.to_mm == curve.start()
                            && last.reason == reason =>
                    {
                        last.to_mm = curve.end();
                        last.length_mm += length(&curve);
                    }
                    _ => report.skipped.push(SkippedSpan {
                        level: Some(level),
                        edge: None,
                        from_mm: curve.start(),
                        to_mm: curve.end(),
                        length_mm: length(&curve),
                        reason,
                    }),
                }
            }
        }
        if !current.is_empty() {
            loop_runs.push(current);
        }
        // A closed loop with a gap: join the last run with the first.
        let gaps = pieces.iter().any(|(_, ok)| !ok);
        if gaps
            && loop_runs.len() > 1
            && pieces.first().is_some_and(|p| p.1)
            && pieces.last().is_some_and(|p| p.1)
        {
            let first = loop_runs.remove(0);
            loop_runs.last_mut().unwrap().extend(first);
        }
        // Entry and exit: straight down from and up to the feed height,
        // proven too; a run shortens from that end until they are.
        for mut curves in loop_runs {
            let skip =
                |curve: &Curve,
                 report: &mut DeburrReport,
                 record: &mut Vec<(usize, Curve, Option<&'static str>)>| {
                    record.push((level, curve.clone(), Some(NO_ENTRY)));
                    report.skipped.push(SkippedSpan {
                        level: Some(level),
                        edge: None,
                        from_mm: curve.start(),
                        to_mm: curve.end(),
                        length_mm: length(curve),
                        reason: NO_ENTRY,
                    })
                };
            while let Some(first) = curves.first() {
                let s = first.start();
                if prover.line([s[0], s[1], feed], s, 0.) {
                    break;
                }
                skip(&curves.remove(0), report, record);
            }
            while let Some(last) = curves.last() {
                let e = last.end();
                if prover.line(e, [e[0], e[1], feed], 0.) {
                    break;
                }
                skip(&curves.pop().unwrap(), report, record);
            }
            if !curves.is_empty() {
                record.extend(curves.iter().map(|c| (level, c.clone(), None)));
                runs.push(Run { curves });
            }
        }
        Ok(())
    }

    /// Proven pieces of a listed edge's straight path, as runs, with proven
    /// straight entries and exits.
    #[allow(clippy::too_many_arguments)]
    fn plan_edge(
        edge: &str,
        start: Vec3,
        end: Vec3,
        feed: f64,
        prover: &mut Prover,
        limits: Option<&crate::limits::PreparedLimits>,
        min_piece: f64,
        runs: &mut Vec<Run>,
        report: &mut DeburrReport,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        let at = |t: f64| -> Vec3 { std::array::from_fn(|k| start[k] + t * (end[k] - start[k])) };
        let mut pieces: Vec<(Curve, bool)> = vec![];
        let mut stack = vec![(0., 1.)];
        while let Some((t0, t1)) = stack.pop() {
            if cancel.is_cancelled() {
                return Err("deburr planning cancelled".into());
            }
            let curve = Curve::Line {
                start: at(t0),
                end: at(t1),
            };
            let ok = prover.line(at(t0), at(t1), 0.)
                && limits.is_none_or(|l| l.allows_curve(&curve, true).unwrap_or(false));
            if ok || length(&curve) <= min_piece {
                pieces.push((curve, ok));
            } else {
                let m = (t0 + t1) / 2.;
                stack.push((m, t1));
                stack.push((t0, m));
            }
        }
        let skip = |curve: &Curve, reason: &'static str, report: &mut DeburrReport| match report
            .skipped
            .last_mut()
        {
            Some(last)
                if last.edge.as_deref() == Some(edge)
                    && last.to_mm == curve.start()
                    && last.reason == reason =>
            {
                last.to_mm = curve.end();
                last.length_mm += length(curve);
            }
            _ => report.skipped.push(SkippedSpan {
                level: None,
                edge: Some(edge.to_string()),
                from_mm: curve.start(),
                to_mm: curve.end(),
                length_mm: length(curve),
                reason,
            }),
        };
        let mut groups: Vec<Vec<Curve>> = vec![];
        let mut current: Vec<Curve> = vec![];
        for (curve, ok) in pieces {
            if ok {
                current.push(curve);
            } else {
                if !current.is_empty() {
                    groups.push(std::mem::take(&mut current));
                }
                skip(&curve, UNPROVEN, report);
            }
        }
        if !current.is_empty() {
            groups.push(current);
        }
        for mut curves in groups {
            while let Some(first) = curves.first() {
                let s = first.start();
                if prover.line([s[0], s[1], feed], s, 0.) {
                    break;
                }
                skip(&curves.remove(0), NO_ENTRY, report);
            }
            while let Some(last) = curves.last() {
                let e = last.end();
                if prover.line(e, [e[0], e[1], feed], 0.) {
                    break;
                }
                skip(&curves.pop().unwrap(), NO_ENTRY, report);
            }
            if !curves.is_empty() {
                runs.push(Run { curves });
            }
        }
        Ok(())
    }

    pub fn report(&self) -> &DeburrReport {
        &self.report
    }
    pub fn prefix_events(&self) -> Vec<Event> {
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
            return Err("deburr report cancelled".into());
        }
        if self.phase != 7 {
            return Err("drain deburr motions before the final report".into());
        }
        self.report.geometric_residual_status = if self.report.skipped.is_empty() {
            ResidualStatus::Estimated
        } else {
            ResidualStatus::Unknown
        };
        Ok(())
    }
    fn emit(&mut self, curve: Curve, kind: MoveKind, stock: &Stock) -> Result<Motion, String> {
        curve.prepare(self.op.tolerance_mm)?;
        let feed = match kind {
            MoveKind::Rapid => None,
            MoveKind::Plunge => Some(self.op.plunge_mm_min),
            _ => Some(self.op.feed_mm_min),
        };
        let m = Motion {
            id: self.next_id,
            operation: self.op.id.clone(),
            tool: self.op.tool.clone(),
            kind,
            curve,
            feed_mm_min: feed,
        };
        self.next_id += 1;
        self.position = m.curve.end();
        self.expected_generation = Some(stock.generation() + 1);
        Ok(m)
    }
    /// Split the straight-up lift after each run where the stock beside the
    /// tool ends (#441, `OperationPlan::split_lifts`).
    pub(crate) fn split_lifts(&mut self, lifts: crate::lift::Lifts) {
        self.lift_split = Some(lifts);
    }
    pub fn next_motion_with_stock(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        if stock.spec() != &self.spec
            || stock.generation() == u64::MAX
            || self
                .expected_generation
                .is_some_and(|g| g != stock.generation())
        {
            return Err("commit the preceding deburr motion to matching stock first".into());
        }
        loop {
            if cancel.is_cancelled() {
                return Err("deburr planning cancelled".into());
            }
            let p = self.position;
            let line = |end: Vec3| Curve::Line { start: p, end };
            let (curve, kind, next) = match self.phase {
                // Up to the travel height (clearance before the first run
                // and after the last, retract between runs).
                0 => {
                    let z = if self.run == 0 || self.run >= self.runs.len() {
                        self.heights.clearance
                    } else {
                        self.heights.retract
                    };
                    (
                        line([p[0], p[1], z.max(p[2])]),
                        MoveKind::Rapid,
                        if self.run >= self.runs.len() { 7 } else { 1 },
                    )
                }
                1 => {
                    let start = self.runs[self.run].curves[0].start();
                    (line([start[0], start[1], p[2]]), MoveKind::Rapid, 2)
                }
                2 => (line([p[0], p[1], self.heights.feed]), MoveKind::Rapid, 3),
                3 => {
                    let start = self.runs[self.run].curves[0].start();
                    if !self.center_cutting {
                        // The tip must enter where the stock is already clear.
                        let query = Motion {
                            id: self.next_id,
                            operation: self.op.id.clone(),
                            tool: self.op.tool.clone(),
                            kind: MoveKind::Plunge,
                            curve: line(start),
                            feed_mm_min: Some(self.op.plunge_mm_min),
                        };
                        let mut work = usize::MAX;
                        if stock.may_remove_material(
                            &query,
                            self.tip_radius.max(16. * self.op.tolerance_mm)
                                + 16. * self.op.tolerance_mm,
                            0.,
                            self.op.tolerance_mm,
                            &mut work,
                            cancel,
                        )? {
                            return Err(format!(
                                "deburr entry at ({:.4}, {:.4}): the chamfer mill does not cut at its center and the stock below its tip is not proven clear",
                                start[0], start[1]
                            ));
                        }
                    }
                    self.curve = 0;
                    (line(start), MoveKind::Plunge, 4)
                }
                4 => {
                    let run = &self.runs[self.run];
                    let curve = run.curves[self.curve].clone();
                    self.curve += 1;
                    let next = if self.curve == run.curves.len() { 5 } else { 4 };
                    (curve, MoveKind::Cut, next)
                }
                5 => {
                    // Fed only through the stock beside the tool, then the
                    // rest of the way as its own retract (#441).
                    let feed = self.heights.feed;
                    let top = crate::lift::feed_to(self.lift_split.as_ref(), stock, p, feed);
                    if top < feed {
                        (line([p[0], p[1], top]), MoveKind::Retract, 5)
                    } else {
                        self.run += 1;
                        (line([p[0], p[1], feed]), MoveKind::Retract, 0)
                    }
                }
                _ => return Ok(None),
            };
            self.phase = next;
            if matches!(curve, Curve::Line { start, end } if start == end) {
                continue;
            }
            let motion = self.emit(curve, kind, stock)?;
            return Ok(Some(motion));
        }
    }
}
