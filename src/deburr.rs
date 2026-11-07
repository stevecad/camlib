//! Deburr: break the horizontal edges of a finished part with a chamfer mill.
//!
//! Each level is the union of the part's upward-facing planar faces at one
//! height z_e. A vertical-axis cone swept along a horizontal path at tip
//! height z_t removes exactly the points within horizontal distance
//! rho(z) = r_tip + (z - z_t) tan(half angle) of the path, so with the axis on
//! the outward offset of the face boundary by d (tip radius + tip clearance)
//! the break is the cone's flank whatever the edge's curvature: `width_mm` on
//! the face, the chamfer depth down a vertical wall.
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

fn climb() -> String {
    "climb".into()
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DeburrLevel {
    /// The upward-facing planar faces at this height (unioned).
    pub regions: Vec<PlanarRegion>,
    pub z_mm: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DeburrOperation<G = SurfaceMesh> {
    pub id: String,
    pub tool: String,
    /// The finished part (what must not be cut beyond the breaks).
    pub surface: G,
    pub levels: Vec<DeburrLevel>,
    /// Break width on the face (horizontal leg).
    pub width_mm: f64,
    /// How far out along the cone from its tip the break's lower edge is
    /// cut (radially), so the flank cuts, not the point.
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
            width_mm: self.width_mm,
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
    pub level: usize,
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
    pub requested_width_mm: f64,
    /// Break depth down a vertical wall below each level.
    pub wall_depth_mm: f64,
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
    pub clearance_proof: &'static str,
}

const CONCAVE: &str = "concave edge: the part rises above the face there (nothing to break)";
const UNPROVEN: &str = "not proven clear of the rest of the part (whole tool assembly)";

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
    part: Index,
    part_seen: Vec<u32>,
    part_stamp: u32,
}

impl Prover {
    /// Whether the part rises above `z` under the point (a tool centre on a
    /// concave stretch lies inside the wall rising from the face).
    fn inside_part_above(&mut self, p: Vec3, z: f64) -> bool {
        let probe = [Section {
            z0: 0.,
            r0: 1e-6,
            r1: 1e-6,
            rise: 0.,
        }];
        self.part.max_sections(
            [p[0], p[1]],
            &probe,
            &mut self.part_seen,
            &mut self.part_stamp,
        ) > z
    }
    fn grown(&self, extra: f64) -> Vec<Section> {
        self.sections
            .iter()
            .map(|s| Section {
                z0: s.z0,
                r0: s.r0 + self.grow + extra,
                r1: s.r1 + self.grow + extra,
                rise: s.rise,
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
        let CutterShape::Chamfer {
            tip_diameter_mm, ..
        } = cutter.shape
        else {
            return Err("deburr needs a conical chamfer mill (camlib.tools.ChamferMill)".into());
        };
        let tip_radius = tip_diameter_mm / 2.;
        let d = tip_radius + op.tip_clearance_mm;
        let reference = actual
            .cutter()
            .tip_height_at_radius(d)
            .ok_or("deburr tip clearance exceeds the cone radius")?;
        let top = actual
            .cutter()
            .tip_height_at_radius(d + op.width_mm)
            .ok_or("deburr width exceeds the chamfer mill's cone radius")?;
        let depth = top - reference;
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
                        op.width_mm,
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
                    op.width_mm > op.boundary_tolerance_mm + guard,
                    format!("width_mm {} must exceed boundary_tolerance_mm", op.width_mm),
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
                    !op.levels.is_empty() && op.levels.iter().all(|l| !l.regions.is_empty()),
                    "deburr needs at least one level with at least one face".into(),
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
            wall_depth_mm: depth,
            cut_length_mm: 0.,
            unreachable_length_mm: 0.,
            concave_length_mm: 0.,
            skipped_length_mm: 0.,
            skipped: vec![],
            obstacle_triangles: 0,
            clearance_proof: "exact section contact (cone, neck, shank, holder) against the part above the level, below the break and any non-face/non-wall geometry in the break band; sections grown by the surface error, two stock cell diagonals and each arc chord's sagitta",
        };
        let mut runs = vec![];
        let min_piece = (op.boundary_tolerance_mm).max(guard);
        for (li, level) in op.levels.iter().enumerate() {
            if cancel.is_cancelled() {
                return Err("deburr planning cancelled".into());
            }
            let z_e = level.z_mm;
            let tip = z_e - top;
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
            // camlib offsets are positive inward (clearance): grow outward.
            let center = face.offset(-(d + error + margin))?.region;
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
            let (low, high) = (z_e - depth - grow * rise - band, z_e + band);
            let on_boundary = |p: Vec3| {
                boundary_strokes
                    .iter()
                    .map(|s| stroke_distance([p[0], p[1]], s))
                    .fold(f64::INFINITY, f64::min)
                    <= e + op.boundary_tolerance_mm + error + guard
            };
            let mut obstacles = vec![];
            for t in op.surface.triangles.iter() {
                obstacles.extend(clip(*t, high, true));
                obstacles.extend(clip(*t, low, false));
                for piece in clip(*t, high, false)
                    .into_iter()
                    .flat_map(|p| clip(p, low, true))
                {
                    let horizontal = piece.iter().all(|p| (p[2] - z_e).abs() <= band);
                    let u = [
                        piece[1][0] - piece[0][0],
                        piece[1][1] - piece[0][1],
                        piece[1][2] - piece[0][2],
                    ];
                    let v = [
                        piece[2][0] - piece[0][0],
                        piece[2][1] - piece[0][1],
                        piece[2][2] - piece[0][2],
                    ];
                    let n = [
                        u[1] * v[2] - u[2] * v[1],
                        u[2] * v[0] - u[0] * v[2],
                        u[0] * v[1] - u[1] * v[0],
                    ];
                    let norm = n[0].hypot(n[1]).hypot(n[2]);
                    let vertical = norm > 0. && n[2].abs() <= 1e-9 * norm;
                    let mid =
                        std::array::from_fn(|k| (piece[0][k] + piece[1][k] + piece[2][k]) / 3.);
                    let wall =
                        vertical && piece.iter().all(|p| on_boundary(*p)) && on_boundary(mid);
                    if !(horizontal || wall || norm == 0.) {
                        obstacles.push(piece);
                    }
                }
            }
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
                part,
                part_stamp: 0,
                sections: sections.clone(),
                grow,
                lift: e,
                sagitta: band,
                stamp: 0,
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
                        cancel,
                    )?;
                }
            }
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
        report.unreachable_length_mm = report.skipped_length_mm - report.concave_length_mm;
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
                    UNPROVEN
                };
                match report.skipped.last_mut() {
                    Some(last)
                        if last.level == level
                            && last.to_mm == curve.start()
                            && last.reason == reason =>
                    {
                        last.to_mm = curve.end();
                        last.length_mm += length(&curve);
                    }
                    _ => report.skipped.push(SkippedSpan {
                        level,
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
            let skip = |curve: &Curve, report: &mut DeburrReport| {
                report.skipped.push(SkippedSpan {
                    level,
                    from_mm: curve.start(),
                    to_mm: curve.end(),
                    length_mm: length(curve),
                    reason: "no proven straight entry or exit from the feed height",
                })
            };
            while let Some(first) = curves.first() {
                let s = first.start();
                if prover.line([s[0], s[1], feed], s, 0.) {
                    break;
                }
                skip(&curves.remove(0), report);
            }
            while let Some(last) = curves.last() {
                let e = last.end();
                if prover.line(e, [e[0], e[1], feed], 0.) {
                    break;
                }
                skip(&curves.pop().unwrap(), report);
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
                    self.run += 1;
                    (line([p[0], p[1], self.heights.feed]), MoveKind::Retract, 0)
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
