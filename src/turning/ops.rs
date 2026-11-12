//! Turning operations planned as checked XZ motion (x = radius).
//!
//! Every move is checked before it is emitted: the insert (a convex polygon
//! containing the nose) swept along a line is the convex hull of its two end
//! poses, and must not reach into the part; the holder must not reach into
//! the current stock; a rapid must not touch the stock at all. Each cutting
//! move then updates the axisymmetric stock.
//!
//! Finishing follows the arcs of its path (the profile's arcs offset by the
//! nose radius, the nose rolling round a sharp corner) with arc moves, which
//! controllers interpolate natively in G18 (G2/G3; LinuxCNC G-code
//! reference, FANUC and Centroid lathe manuals). An arc move is checked in
//! pieces, each inside the triangle of its two ends and the corner of their
//! tangents: the same hulls, checks and stock steps as the chords it
//! replaces (`arc_steps`).
use super::{
    profile::{Envelope, Seg, TurnProfile},
    stock::{TurnChange, TurnSide, TurnStock, TurnStockSpec},
    tool::{Hand, TurningTool},
};
use crate::{
    motion::{CompensationSide, Coolant, Curve, Event, Motion, MoveKind, Plane, SpindleDirection},
    stock::CancelSignal,
    tool::{Cutter, CutterShape, HolderSection, ToolAssembly},
};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TurnToolSetting {
    pub tool: TurningTool,
    /// T word and offset (wear/geometry) number: T0101 = number 1, offset 1.
    pub number: u32,
    pub offset: u32,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum TurnOperation {
    /// Face the stock's end down to `z_mm` in passes of `stepdown_mm`, the
    /// nose cutting from outside the stock across the axis.
    Face {
        id: String,
        tool: String,
        z_mm: f64,
        stepdown_mm: f64,
        feed_mm_rev: f64,
        rpm: f64,
        /// Constant surface speed (G96, m/min) instead of a fixed rpm; `rpm`
        /// is then the spindle limit (G50). Needs feed per revolution.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        surface_speed_m_min: Option<f64>,
    },
    /// Outside roughing in axial passes (G71 type I): each pass at a radius
    /// `depth_of_cut_mm` below the last, from beyond the stock end toward the
    /// chuck until the part (plus `allowance_mm`) is reached, then retract.
    Rough {
        id: String,
        tool: String,
        depth_of_cut_mm: f64,
        allowance_mm: f64,
        feed_mm_rev: f64,
        rpm: f64,
        /// Constant surface speed (G96, m/min) instead of a fixed rpm; `rpm`
        /// is then the spindle limit (G50). Needs feed per revolution.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        surface_speed_m_min: Option<f64>,
        #[serde(default = "retract")]
        retract_mm: f64,
    },
    /// Grooving with a grooving insert: plunges across the groove from
    /// `z_from_mm` to `z_to_mm` (its two side walls) down to
    /// `bottom_radius_mm`, overlapping by `overlap_mm`, each in pecks of
    /// `peck_mm` (if set) with a retract of half a peck after each.
    Groove {
        id: String,
        tool: String,
        z_from_mm: f64,
        z_to_mm: f64,
        bottom_radius_mm: f64,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        peck_mm: Option<f64>,
        #[serde(default = "overlap")]
        overlap_mm: f64,
        feed_mm_rev: f64,
        rpm: f64,
        /// Constant surface speed (G96, m/min) instead of a fixed rpm; `rpm`
        /// is then the spindle limit (G50). Needs feed per revolution.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        surface_speed_m_min: Option<f64>,
    },
    /// Parting off with a grooving insert: its +z side at `z_mm` (the part's
    /// back face), plunged to the axis.
    PartOff {
        id: String,
        tool: String,
        z_mm: f64,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        peck_mm: Option<f64>,
        feed_mm_rev: f64,
        rpm: f64,
        /// Constant surface speed (G96, m/min) instead of a fixed rpm; `rpm`
        /// is then the spindle limit (G50). Needs feed per revolution.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        surface_speed_m_min: Option<f64>,
    },
    /// Outside single-point threading: `passes` spindle-synchronized passes
    /// (constant chip area: depth x sqrt(k / n)) from `z_start_mm` (plus two
    /// leads of run-up) to `z_end_mm`, the insert tip down to the root at
    /// `major_radius_mm - depth_mm`, infeeding along `infeed_angle_deg` (0
    /// radial, 29.5 flank), then `spring_passes` at full depth.
    Thread {
        id: String,
        tool: String,
        z_start_mm: f64,
        z_end_mm: f64,
        major_radius_mm: f64,
        pitch_mm: f64,
        depth_mm: f64,
        passes: u32,
        #[serde(default)]
        infeed_angle_deg: f64,
        #[serde(default)]
        spring_passes: u32,
        rpm: f64,
        /// Constant surface speed (G96, m/min) instead of a fixed rpm; `rpm`
        /// is then the spindle limit (G50). Needs feed per revolution.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        surface_speed_m_min: Option<f64>,
    },
    /// Inside roughing of the part's bore with a boring tool: axial passes
    /// opening the existing hole by `depth_of_cut_mm`, each from the front
    /// to where the bore (less `allowance_mm`) is reached or `z_end_mm`.
    BoreRough {
        id: String,
        tool: String,
        depth_of_cut_mm: f64,
        allowance_mm: f64,
        z_end_mm: f64,
        feed_mm_rev: f64,
        rpm: f64,
        /// Constant surface speed (G96, m/min) instead of a fixed rpm; `rpm`
        /// is then the spindle limit (G50). Needs feed per revolution.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        surface_speed_m_min: Option<f64>,
        #[serde(default = "retract")]
        retract_mm: f64,
    },
    /// Inside finishing along the part's bore from the front face to
    /// `z_end_mm`, the nose center `nose radius + allowance_mm` inside it.
    BoreFinish {
        id: String,
        tool: String,
        #[serde(default)]
        allowance_mm: f64,
        z_end_mm: f64,
        feed_mm_rev: f64,
        rpm: f64,
        /// Constant surface speed (G96, m/min) instead of a fixed rpm; `rpm`
        /// is then the spindle limit (G50). Needs feed per revolution.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        surface_speed_m_min: Option<f64>,
        /// Tool-nose radius compensation: "computer" (the path is already
        /// the nose's), or "wear" (the same path with G41/G42 on, so the
        /// control applies the wear in its offset's nose radius only).
        #[serde(default)]
        compensation: crate::contour::Compensation,
    },
    /// Face grooving with a face grooving tool: axial plunges across the
    /// declared face groove (TurningJob.face_grooves) between the radii, down
    /// to its bottom, overlapping by `overlap_mm`, in pecks if set.
    FaceGroove {
        id: String,
        tool: String,
        inner_radius_mm: f64,
        outer_radius_mm: f64,
        bottom_z_mm: f64,
        #[serde(default)]
        peck_mm: Option<f64>,
        #[serde(default = "overlap")]
        overlap_mm: f64,
        feed_mm_rev: f64,
        rpm: f64,
        /// Constant surface speed (G96, m/min) instead of a fixed rpm; `rpm`
        /// is then the spindle limit (G50). Needs feed per revolution.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        surface_speed_m_min: Option<f64>,
    },
    /// Drilling on the spindle axis with a drill tool: its tip down to
    /// `depth_z_mm`, in pecks of `peck_mm` (back out of the hole after each,
    /// G83-style) when set.
    Drill {
        id: String,
        tool: String,
        depth_z_mm: f64,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        peck_mm: Option<f64>,
        feed_mm_rev: f64,
        rpm: f64,
        /// Constant surface speed (G96, m/min) instead of a fixed rpm; `rpm`
        /// is then the spindle limit (G50). Needs feed per revolution.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        surface_speed_m_min: Option<f64>,
    },
    /// Outside finishing along the profile (front face first) with the nose
    /// center `nose radius + allowance_mm` outside it.
    Finish {
        id: String,
        tool: String,
        #[serde(default)]
        allowance_mm: f64,
        feed_mm_rev: f64,
        rpm: f64,
        /// Constant surface speed (G96, m/min) instead of a fixed rpm; `rpm`
        /// is then the spindle limit (G50). Needs feed per revolution.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        surface_speed_m_min: Option<f64>,
        /// Tool-nose radius compensation: "computer" (the path is already
        /// the nose's), or "wear" (the same path with G41/G42 on, so the
        /// control applies the wear in its offset's nose radius only).
        #[serde(default)]
        compensation: crate::contour::Compensation,
    },
}
fn retract() -> f64 {
    0.5
}
fn overlap() -> f64 {
    0.2
}

/// A lathe setup's turning definition (a stream manifest's `turning`): the
/// part, its bore, the chuck, tools and operations. The stock and the start
/// position are the manifest's own.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TurnSetup {
    pub part: TurnProfile,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bore: Option<TurnProfile>,
    pub chuck_z_mm: f64,
    pub tools: Vec<TurnToolSetting>,
    pub operations: Vec<TurnOperation>,
    pub clearance_mm: f64,
    pub tolerance_mm: f64,
    #[serde(default)]
    pub feed_per_rev: bool,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub face_grooves: Vec<FaceGroove>,
    #[serde(default)]
    pub controlled_point: ControlledPoint,
}

impl TurnSetup {
    /// The posting stand-ins of the setup's tools (a manifest's `tools`).
    pub fn proxy_tools(&self) -> Vec<ToolAssembly> {
        self.tools.iter().map(|t| proxy(&t.tool)).collect()
    }

    /// The planner input for this setup on `stock` from `initial` ([r, z]).
    pub fn job(&self, id: &str, stock: &TurnStockSpec, initial: [f64; 2]) -> TurningJob {
        TurningJob {
            id: id.into(),
            stock: stock.clone(),
            part: self.part.clone(),
            bore: self.bore.clone(),
            chuck_z_mm: self.chuck_z_mm,
            tools: self.tools.clone(),
            operations: self.operations.clone(),
            initial_position_mm: initial,
            clearance_mm: self.clearance_mm,
            tolerance_mm: self.tolerance_mm,
            feed_per_rev: self.feed_per_rev,
            face_grooves: self.face_grooves.clone(),
            controlled_point: self.controlled_point,
        }
    }
}

impl TurnOperation {
    pub fn id(&self) -> &str {
        match self {
            Self::Face { id, .. }
            | Self::Rough { id, .. }
            | Self::Groove { id, .. }
            | Self::PartOff { id, .. }
            | Self::Thread { id, .. }
            | Self::Drill { id, .. }
            | Self::FaceGroove { id, .. }
            | Self::BoreRough { id, .. }
            | Self::BoreFinish { id, .. }
            | Self::Finish { id, .. } => id,
        }
    }
    pub fn tool(&self) -> &str {
        match self {
            Self::Face { tool, .. }
            | Self::Rough { tool, .. }
            | Self::Groove { tool, .. }
            | Self::PartOff { tool, .. }
            | Self::Thread { tool, .. }
            | Self::Drill { tool, .. }
            | Self::FaceGroove { tool, .. }
            | Self::BoreRough { tool, .. }
            | Self::BoreFinish { tool, .. }
            | Self::Finish { tool, .. } => tool,
        }
    }
}

/// The turning planner's input: a setup on its stock from its start.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TurningJob {
    pub id: String,
    pub stock: TurnStockSpec,
    pub part: TurnProfile,
    /// The part's bore: the hole's half-section ([radius, z, bulge]) from
    /// the front face to its deepest point (the drill point's tip on the
    /// axis). Drilling and boring stay inside it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bore: Option<TurnProfile>,
    /// The chuck face: no tool part may reach below this z (plus clearance).
    pub chuck_z_mm: f64,
    pub tools: Vec<TurnToolSetting>,
    pub operations: Vec<TurnOperation>,
    /// Start (and tool change) position: [radius, z].
    pub initial_position_mm: [f64; 2],
    #[serde(default = "clearance")]
    pub clearance_mm: f64,
    #[serde(default = "tolerance")]
    pub tolerance_mm: f64,
    /// Post feeds per spindle revolution (G95) instead of per minute.
    #[serde(default)]
    pub feed_per_rev: bool,
    /// Rectangular grooves in the front face (axial, annular): cut with a
    /// face grooving tool (TurnOperation::FaceGroove).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub face_grooves: Vec<FaceGroove>,
    /// The point the program moves: each tool's imaginary tip (where its
    /// offsets are touched off on a diameter and a face) or its nose center.
    #[serde(default)]
    pub controlled_point: ControlledPoint,
}

/// An annular groove in the part's front face: between `inner_radius_mm`
/// and `outer_radius_mm`, from the face down to `bottom_z_mm`, with square
/// corners (the insert's nose radii stay in its bottom corners).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FaceGroove {
    pub inner_radius_mm: f64,
    pub outer_radius_mm: f64,
    pub bottom_z_mm: f64,
}

/// The point of a turning tool that the program moves (and the start
/// position names).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ControlledPoint {
    /// The imaginary tool tip: the nose center moved one nose radius toward
    /// the diameter (toward the axis outside, away from it inside a bore)
    /// and toward the face the tool cuts toward (-z right hand, +z left
    /// hand; threading inserts: on their center line). Offsets touched off
    /// on a diameter and a face are of this point.
    #[default]
    Tip,
    /// The nose center (offsets of the nose center).
    NoseCenter,
}
fn clearance() -> f64 {
    2.
}
fn tolerance() -> f64 {
    1e-3
}

#[derive(Clone, Debug, Serialize)]
pub struct TurnReport {
    pub id: String,
    pub passes: usize,
    pub moves: usize,
    pub cut_length_mm: f64,
    /// Least clearance of the insert from the part over the operation.
    pub min_part_clearance_mm: f64,
    /// Least clearance of the holder from the stock.
    pub min_holder_clearance_mm: f64,
}

/// The exact stock change of one move: changed cells (ascending index) and
/// rings it certainly removed inside the material (face grooves).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TurnDelta {
    pub motion_id: u64,
    pub changes: Vec<TurnChange>,
    pub rings: Vec<[f64; 4]>,
}

pub struct TurnPlan {
    /// The program events: work offset, tool changes, spindle, motion.
    pub events: Vec<Event>,
    /// Proxy tool assemblies (posting identity only: the turning geometry
    /// above is what the checks use).
    pub tools: Vec<ToolAssembly>,
    pub reports: Vec<TurnReport>,
    pub stock: TurnStock,
}

/// A milling-style assembly standing in for a turning tool in the program
/// header (id, gauge length for posting); not used for geometry.
fn proxy(t: &TurningTool) -> ToolAssembly {
    let d = (2. * t.nose_radius_mm).max(0.01);
    ToolAssembly {
        id: t.id.clone(),
        cutter: Cutter {
            id: format!("{}-insert", t.id),
            name: "turning insert (posting identity)".into(),
            shape: CutterShape::Flat,
            diameter_mm: d,
            cutting_length_mm: 1.,
            overall_length_mm: 4.,
            shank_diameter_mm: d,
            flutes: 1,
            center_cutting: true,
            material: String::new(),
            neck: None,
        },
        stickout_mm: 2.,
        gauge_length_mm: 3.,
        holders: vec![HolderSection {
            z_min_mm: 2.,
            z_max_mm: 3.,
            radius_min_mm: d / 2.,
            radius_max_mm: d / 2.,
        }],
    }
}

fn hull(mut points: Vec<[f64; 2]>) -> Vec<[f64; 2]> {
    points.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
    points.dedup();
    if points.len() < 3 {
        return points;
    }
    let cross = |o: [f64; 2], a: [f64; 2], b: [f64; 2]| {
        (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    };
    let mut lower: Vec<[f64; 2]> = vec![];
    for &p in &points {
        while lower.len() >= 2 && cross(lower[lower.len() - 2], lower[lower.len() - 1], p) <= 0. {
            lower.pop();
        }
        lower.push(p);
    }
    let mut upper: Vec<[f64; 2]> = vec![];
    for &p in points.iter().rev() {
        while upper.len() >= 2 && cross(upper[upper.len() - 2], upper[upper.len() - 1], p) <= 0. {
            upper.pop();
        }
        upper.push(p);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

/// The upper boundary of the stock's possible material as an envelope.
fn stock_envelope(stock: &TurnStock) -> Envelope {
    runs(stock, |c| {
        if c.outer.upper_mm > c.inner.lower_mm {
            c.outer.upper_mm
        } else {
            0.
        }
    })
}

/// The stock's certainly open bore (inner lower bound) over z, for tools
/// working on the axis. Where the stock is gone the bore is unbounded.
fn stock_bore(stock: &TurnStock) -> Envelope {
    runs(stock, |c| {
        if c.outer.upper_mm <= c.inner.lower_mm {
            1e6
        } else {
            c.inner.lower_mm
        }
    })
}

/// A step envelope over the stock's z bins of `radius(cell)`, one flat
/// piece per run of equal radius (points inside a run change no query).
fn runs(stock: &TurnStock, radius: impl Fn(&super::stock::TurnCell) -> f64) -> Envelope {
    let mut points: Vec<[f64; 2]> = vec![];
    let cells = stock.cells();
    let mut i = 0;
    while i < cells.len() {
        let r = radius(&cells[i]);
        let mut j = i;
        while j + 1 < cells.len() && radius(&cells[j + 1]) == r {
            j += 1;
        }
        points.push([stock.bin(i)[0], r]);
        points.push([stock.bin(j)[1], r]);
        i = j + 1;
    }
    Envelope::new(points)
}

struct Planner<'a> {
    job: &'a TurningJob,
    /// Polled before every move: a cancelled plan returns no result.
    cancel: &'a dyn CancelSignal,
    part: Envelope,
    /// The part's bore, if declared (drilling and boring work inside it).
    bore: Option<Envelope>,
    stock: TurnStock,
    events: Vec<Event>,
    /// The stock change of each planned move, in order.
    deltas: Vec<TurnDelta>,
    position: [f64; 2], // (u, v) = (z, r) of the nose center
    next_id: u64,
    guard: f64,
    /// The last cutting move (from, to), for retracing rapids.
    last_cut: Option<([f64; 2], [f64; 2])>,
    /// Where the last threading pass ended (for its pull-out).
    last_thread: Option<[f64; 2]>,
    /// The spindle: fixed rpm, or constant surface speed (m/min, max rpm).
    spindle: Spin,
}

#[derive(Clone, Copy, PartialEq)]
enum Spin {
    Rpm(f64),
    Css(f64, f64),
}

pub(crate) struct ToolGeom {
    id: String,
    /// Convex pieces of the insert's outline on the centre plane, (u, v)
    /// relative to the nose center: a straight move sweeps each piece into
    /// the hull of its two ends (the cutting face's reach).
    insert: Vec<Vec<[f64; 2]>>,
    /// The insert's and the holder's bodies off the centre plane (#103):
    /// convex 3D pieces (u, v, y), y across the plane; every check uses
    /// their images (projection or radial), the 2D outline only the cut.
    pub(crate) insert_body: Vec<Vec<[f64; 3]>>,
    pub(crate) holder_body: Vec<Vec<[f64; 3]>>,
    /// A straight grooving blade's flanks (#170): what each is called and
    /// the body's vertices on it, so a refusal names the flank that reaches
    /// the material.
    pub(crate) flanks: Vec<(String, Vec<[f64; 3]>)>,
    rn: f64,
    /// Grooving blade: the trailing corner's nose center this far along z
    /// (signed by hand) from the leading corner's; zero for one nose.
    blade: f64,
    /// A drill on the axis: its radius and point length.
    drill: Option<(f64, f64)>,
    /// Works inside the part's bore (a drill or a boring tool).
    inner: bool,
    /// The controlled point from the nose center, (u, v).
    pub(crate) tip: [f64; 2],
    /// A face grooving blade (pointing -z; `blade` is then the second
    /// corner's offset along r).
    axial: bool,
    /// The insert's non-cutting sides (an outline insert's segments that do
    /// not cut, a grooving blade's top), each with its outward normal:
    /// material reaching one that leads means the depth of cut exceeds the
    /// cutting edges. With them, the stock removal is the whole region the
    /// insert sweeps.
    back: Option<Vec<super::tool::BackSide>>,
    /// The insert's exact straight sides (relative to the nose center).
    edges: Vec<[[f64; 2]; 2]>,
}

/// How far a flank's images (the one the checks use, and the radial one)
/// reach into material.
type FlankMeasure<'a> = dyn Fn(&[Vec<[f64; 2]>], &[Vec<[f64; 2]>]) -> Result<f64, String> + 'a;

/// The polygon mirrored across the spindle axis.
fn mirrored(polygon: &[[f64; 2]]) -> Vec<[f64; 2]> {
    polygon.iter().map(|p| [p[0], -p[1]]).collect()
}
/// How far a face grooving blade (a convex polygon) reaches outside the
/// declared face grooves where it is below the face: its part below the face
/// must lie in one groove (vertical walls, flat bottom), within `tol`.
fn in_face_groove(
    polygon: &[[f64; 2]],
    face: f64,
    grooves: &[FaceGroove],
    tol: f64,
    radial: Option<&[[f64; 2]]>,
) -> f64 {
    // A polygon clipped to u <= face (convex: its vertices bound it).
    let clip = |polygon: &[[f64; 2]]| {
        let n = polygon.len();
        let mut clipped = vec![];
        for i in 0..n {
            let (p, q) = (polygon[i], polygon[(i + 1) % n]);
            if p[0] <= face {
                clipped.push(p);
            }
            if (p[0] - face) * (q[0] - face) < 0. {
                let t = (face - p[0]) / (q[0] - p[0]);
                clipped.push([face, p[1] + t * (q[1] - p[1])]);
            }
        }
        clipped
    };
    // The inner wall and the bottom against `polygon` (the projection: a
    // body point's radius is at least its |v|); the outer wall against the
    // radial image when given (off the centre plane the body reaches out).
    let near = clip(polygon);
    let far = radial.map_or_else(|| near.clone(), clip);
    grooves
        .iter()
        .map(|g| {
            near.iter()
                .map(|p| (g.inner_radius_mm - p[1]).max(g.bottom_z_mm - p[0]))
                .chain(far.iter().map(|p| p[1] - g.outer_radius_mm))
                .fold(f64::NEG_INFINITY, f64::max)
                - tol
        })
        .fold(f64::INFINITY, f64::min)
}

/// How far a tool reaches outside a bore on either side of the axis.
fn outside(bore: &Envelope, polygon: &[[f64; 2]], tol: f64) -> f64 {
    bore.excess(polygon, tol)
        .max(bore.excess(&mirrored(polygon), tol))
}

pub(crate) fn tool_geometry(
    t: &TurningTool,
    controlled_point: ControlledPoint,
) -> Result<ToolGeom, String> {
    let shape = t.shape()?;
    // Tool frame (r, z) to (u, v) = (z, r); a left-hand tool mirrors z,
    // a boring tool (cutting outward, inside a bore) mirrors r.
    let s = if t.hand == Hand::Left { -1. } else { 1. };
    let boring = t.side == TurnSide::Inner;
    let m = if boring { -1. } else { 1. };
    // A face grooving blade: its depth along +z (u), its width along +r.
    let axial = matches!(t.insert, super::tool::Insert::FaceGroove { .. });
    let map = |p: &[f64; 2]| {
        if axial {
            [p[0], p[1]]
        } else {
            [s * p[1], m * p[0]]
        }
    };
    let mapped = |polygon: &[[f64; 2]]| polygon.iter().map(map).collect::<Vec<_>>();
    let body = super::body::body(t)?;
    let (insert_body, holder_body) = (body.insert, body.holder);
    let mapped3 = |pieces: Vec<Vec<[f64; 3]>>| {
        pieces
            .into_iter()
            .map(|piece| {
                piece
                    .into_iter()
                    .map(|p| {
                        let [u, v] = map(&[p[0], p[1]]);
                        [u, v, p[2]]
                    })
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>()
    };
    let (insert_body, holder_body) = (mapped3(insert_body), mapped3(holder_body));
    let flanks = body
        .flanks
        .into_iter()
        .map(|f| {
            use super::body::BladeFlank;
            // A grooving blade's sides face along z (its reference corner's
            // toward -z, mirrored for a left-hand tool); a face grooving
            // blade's along r (its reference corner's toward the axis).
            let name = match f.flank {
                BladeFlank::Front => "front flank",
                BladeFlank::Corner => "corner flank (between its front and a side)",
                BladeFlank::Side(k) if axial => ["inner side flank", "outer side flank"][k],
                BladeFlank::Side(k) => {
                    if (k == 0) == (s > 0.) {
                        "side flank toward -Z"
                    } else {
                        "side flank toward +Z"
                    }
                }
            };
            let points = mapped3(vec![f.points]).remove(0);
            (
                format!("{name}, {} degrees of clearance", f.clearance_deg),
                points,
            )
        })
        .collect();
    let is_drill = matches!(t.insert, super::tool::Insert::Drill { .. });
    let rn = t.nose_radius_mm;
    let tip = match (controlled_point, &t.insert) {
        (ControlledPoint::NoseCenter, _) | (_, super::tool::Insert::Drill { .. }) => [0., 0.],
        (_, super::tool::Insert::Thread { .. }) => [0., -m * rn],
        // Touched off on the face and the groove's inner diameter.
        (_, super::tool::Insert::FaceGroove { .. }) => [-rn, -rn],
        (_, super::tool::Insert::Outline { .. }) => map(&shape
            .programming_point
            .ok_or("outline insert without a programming point")?),
        _ => [-s * rn, -m * rn],
    };
    let back = shape.back.as_ref().map(|pieces| {
        pieces
            .iter()
            .map(|(e, n)| ([map(&e[0]), map(&e[1])], map(n)))
            .collect()
    });
    let edges = shape
        .edges
        .iter()
        .map(|e| [map(&e[0]), map(&e[1])])
        .collect();
    Ok(ToolGeom {
        id: t.id.clone(),
        back,
        edges,
        insert: super::tool::convex_pieces(&mapped(&shape.insert)),
        insert_body,
        holder_body,
        flanks,
        tip,
        axial,
        inner: boring || is_drill,
        rn: t.nose_radius_mm,
        blade: match t.insert {
            super::tool::Insert::Groove { width_mm, .. } => s * (width_mm - 2. * t.nose_radius_mm),
            super::tool::Insert::FaceGroove { width_mm, .. } => width_mm - 2. * t.nose_radius_mm,
            _ => 0.,
        },
        drill: match t.insert {
            super::tool::Insert::Drill {
                diameter_mm,
                point_angle_deg,
                ..
            } => Some((
                diameter_mm / 2.,
                diameter_mm / 2. / (point_angle_deg / 2.).to_radians().tan(),
            )),
            _ => None,
        },
    })
}

impl Planner<'_> {
    fn geometry(&self, t: &TurningTool) -> Result<ToolGeom, String> {
        tool_geometry(t, self.job.controlled_point)
    }

    /// Check and emit one straight move of the nose center.
    fn emit(
        &mut self,
        tool: &ToolGeom,
        op: &str,
        to: [f64; 2],
        kind: MoveKind,
        feed: Option<f64>,
        report: &mut TurnReport,
    ) -> Result<(), String> {
        self.travel(tool, op, None, to, kind, feed, report)
    }

    /// Check and emit one XZ arc of the nose center (#14): about `center`
    /// through `sweep` (radians from +z toward +r, positive = G3) to `to`.
    /// The program gets one G2/G3 block; the checks run on pieces of the arc
    /// no coarser than the chords it replaces (see `arc_steps`), each
    /// against the stock the pieces before it leave.
    #[allow(clippy::too_many_arguments)]
    fn emit_arc(
        &mut self,
        tool: &ToolGeom,
        op: &str,
        center: [f64; 2],
        sweep: f64,
        to: [f64; 2],
        kind: MoveKind,
        feed: Option<f64>,
        report: &mut TurnReport,
    ) -> Result<(), String> {
        self.travel(tool, op, Some((center, sweep)), to, kind, feed, report)
    }

    /// The move kernel: one line (`arc` None) or one XZ arc of the nose
    /// center, checked piece by piece and emitted as one motion with its
    /// stock change. Nothing is committed unless every piece passes.
    #[allow(clippy::too_many_arguments)]
    fn travel(
        &mut self,
        tool: &ToolGeom,
        op: &str,
        arc: Option<([f64; 2], f64)>,
        to: [f64; 2],
        kind: MoveKind,
        feed: Option<f64>,
        report: &mut TurnReport,
    ) -> Result<(), String> {
        if self.cancel.is_cancelled() {
            return Err(CANCELLED.into());
        }
        let from = self.position;
        if from == to && arc.is_none() {
            return Ok(());
        }
        let xz = |p: [f64; 2]| [p[1], 0., p[0]];
        let steps = match arc {
            None => vec![Step {
                points: vec![from, to],
                curves: vec![Curve::Line {
                    start: xz(from),
                    end: xz(to),
                }],
                sag: 0.,
                slack: 0.,
            }],
            Some((center, sweep)) => {
                // One nose following the arc while cutting: a grooving
                // blade's flat, a drill and a thread are swept as lines only.
                if matches!(kind, MoveKind::Rapid | MoveKind::Thread)
                    || tool.blade != 0.
                    || tool.axial
                    || tool.drill.is_some()
                {
                    return Err(format!(
                        "{op}: a lathe arc is a cutting move of a single-nose insert (not a rapid, a thread, a grooving blade or a drill); write this move as lines"
                    ));
                }
                arc_steps(from, to, center, sweep, self.job.tolerance_mm)
                    .map_err(|e| format!("{op}: {e}"))?
            }
        };
        let at = |shape: &[[f64; 2]], p: [f64; 2]| {
            shape
                .iter()
                .map(|q| [q[0] + p[0], q[1] + p[1]])
                .collect::<Vec<_>>()
        };
        // A convex shape carried along a step stays in the hull of its
        // poses at the step's points (the nose center stays in theirs).
        let carried = |shape: &[[f64; 2]], points: &[[f64; 2]]| {
            hull(points.iter().flat_map(|p| at(shape, *p)).collect())
        };
        let mut after = self.stock.clone();
        let mut touches = false;
        for step in &steps {
            let points = &step.points;
            let swept = |pieces: &[Vec<[f64; 2]>]| {
                pieces
                    .iter()
                    .map(|piece| carried(piece, points))
                    .collect::<Vec<_>>()
            };
            // The bodies' images over the step (each piece swept into the
            // hull of its poses, then imaged): the XZ projection bounds what
            // lies outside the tool's radius (a point's radius is at least
            // its |v|); the radial image (u, √(v² + y²)) of the vertices
            // bounds what lies beyond it (the radius is convex: exact at
            // vertices, never under).
            let image = |pieces: &[Vec<[f64; 3]>], radial: bool| {
                pieces
                    .iter()
                    .map(|piece| {
                        let ends = piece.iter().flat_map(|p| {
                            points.iter().map(move |c| {
                                let v = c[1] + p[1];
                                [c[0] + p[0], if radial { v.hypot(p[2]) } else { v }]
                            })
                        });
                        hull(ends.collect())
                    })
                    .collect::<Vec<_>>()
            };
            let face = swept(&tool.insert);
            let insert_proj = image(&tool.insert_body, false);
            let insert_rad = image(&tool.insert_body, true);
            let holder_proj = image(&tool.holder_body, false);
            let holder_rad = image(&tool.holder_body, true);
            // Drills and boring tools work in a bore: material lies beyond
            // them.
            let (insert, holder) = if tool.inner {
                (insert_rad.clone(), holder_rad.clone())
            } else {
                (insert_proj.clone(), holder_proj.clone())
            };
            // No tool part below the chuck face.
            let chuck = self.job.chuck_z_mm + self.guard;
            if insert.iter().chain(&holder).flatten().any(|p| p[0] < chuck) {
                return Err(format!(
                    "{op}: the tool would reach the chuck (z below {:.3}) moving to r {:.3}, z {:.3}",
                    self.job.chuck_z_mm, to[1], to[0]
                ));
            }
            // Drills and boring tools work inside the part's bore (clear of
            // both walls); every other tool outside the part.
            let face_z = self.job.part.z_range()[1];
            // How far tool pieces reach into the part (outside tools) or out
            // of its bore (drills and boring tools).
            let into_part = |pieces: &[Vec<[f64; 2]>],
                             radial: &[Vec<[f64; 2]>]|
             -> Result<f64, String> {
                let mut worst = f64::NEG_INFINITY;
                for (piece, far) in pieces.iter().zip(radial) {
                    let below_face = piece.iter().any(|p| p[0] < face_z - self.guard);
                    worst = worst.max(if tool.axial && below_face {
                        // A face grooving blade below the face: inside a
                        // declared groove (its inner wall against the
                        // projection, its outer wall against the radial
                        // image).
                        in_face_groove(piece, face_z, &self.job.face_grooves, self.guard, Some(far))
                    } else if tool.inner {
                        let bore = self.bore.as_ref().ok_or_else(|| {
                            format!("{op}: drilling and boring need the part's bore (TurningJob.bore): the hole's half-section from the front face to its deepest point")
                        })?;
                        outside(bore, piece, self.guard)
                    } else {
                        self.part.penetration(piece, self.guard)
                    });
                }
                Ok(worst)
            };
            // Which flank of a grooving blade's body reaches farthest by
            // `measure` (given a piece's image the checks use and its radial
            // image), among those reaching beyond `floor` (refused on their
            // own). A corner shares its ends with the front and a side: it
            // is named only when it reaches farther than both.
            let flank_note = |floor: f64, measure: &FlankMeasure| -> String {
                let mut best = floor;
                let mut named: Vec<&str> = vec![];
                // The front and the sides first, the corners last.
                for corner in [false, true] {
                    for (name, points) in &tool.flanks {
                        if name.starts_with("corner") != corner {
                            continue;
                        }
                        let piece = std::slice::from_ref(points);
                        let far = image(piece, true);
                        let near = if tool.inner {
                            far.clone()
                        } else {
                            image(piece, false)
                        };
                        if far[0].len() < 3 || near[0].len() < 3 {
                            continue;
                        }
                        let Ok(reach) = measure(&near, &far) else {
                            continue;
                        };
                        if reach > best + 1e-9 {
                            best = reach;
                            named = vec![name];
                        } else if reach > floor && reach >= best - 1e-9 && !corner {
                            named.push(name);
                        }
                    }
                }
                if named.is_empty() {
                    String::new()
                } else {
                    format!(
                        "; the body below the cutting face reaches farthest at the blade's {}",
                        named.join(" and its ")
                    )
                }
            };
            let part = into_part(&insert, &insert_rad)?;
            if part > 0. {
                // Name a flank only when the body, not the cutting face
                // itself, is what reaches the part.
                let note = if into_part(&face, &face)? < part - 1e-9 {
                    flank_note(0., &into_part)
                } else {
                    String::new()
                };
                return Err(format!(
                    "{op}: the insert would cut {part:.4} mm into the part moving to r {:.3}, z {:.3}; check the insert's angles against the profile{note}",
                    to[1], to[0]
                ));
            }
            report.min_part_clearance_mm = report.min_part_clearance_mm.min(-part - self.guard);
            // The holder (and bar) against the part and its bore, wherever
            // it is relative to the insert (a face grooving blade's arm
            // stays out of the groove: its holder is checked like an outside
            // tool's).
            let holder_part = if tool.axial {
                holder
                    .iter()
                    .map(|piece| self.part.penetration(piece, self.guard))
                    .fold(f64::NEG_INFINITY, f64::max)
            } else {
                into_part(&holder, &holder_rad)?
            };
            if holder_part > 0. {
                return Err(format!(
                    "{op}: the holder would hit the part ({holder_part:.4} mm) moving to r {:.3}, z {:.3}",
                    to[1], to[0]
                ));
            }
            // The holder trails the insert through the space the insert
            // clears ahead of it: check it against the stock after this
            // step's cut.
            if kind != MoveKind::Rapid {
                // One nose; a grooving blade sweeps both corner noses and
                // its flat bottom (at both ends of the move).
                let line = |a: [f64; 2], b: [f64; 2]| Curve::Line {
                    start: xz(a),
                    end: xz(b),
                };
                let shift = |p: [f64; 2]| [p[0] + tool.blade, p[1]];
                let mut sweeps = step.curves.clone();
                if tool.blade != 0. {
                    sweeps.push(line(shift(from), shift(to)));
                    sweeps.push(line(from, shift(from)));
                    sweeps.push(line(to, shift(to)));
                }
                if tool.axial {
                    // The blade's certain removal: between its nose centers
                    // down to the lower nose, and the noses' full width
                    // above their centers (a ring the radius bounds cannot
                    // hold).
                    let (top, low) = (from[0].max(to[0]), from[0].min(to[0]));
                    let (v0, v1) = (from[1].min(to[1]), from[1].max(to[1]) + tool.blade);
                    after.remove_ring([low - tool.rn, top], [v0, v1]);
                    after.remove_ring([low, top], [v0 - tool.rn, v1 + tool.rn]);
                } else if let Some((radius, cone)) = tool.drill {
                    let changes = after.drill(from[0].min(to[0]), radius, cone);
                    after.apply(&changes)?;
                } else if kind == MoveKind::Thread {
                    // A helical thread leaves its crests: only the solid
                    // bound moves, to the insert tip's radius.
                    let changes = if tool.inner {
                        after.thread_cut_inner(from[0], to[0], to[1].max(from[1]) + tool.rn)
                    } else {
                        after.thread_cut(from[0], to[0], to[1].min(from[1]) - tool.rn)
                    };
                    after.apply(&changes)?;
                } else if let Some(back) = &tool.back {
                    // An insert moving straight meets material only through
                    // the part of its outline facing the motion. Where a
                    // non-cutting side faces forward it may meet stock
                    // first: its sweep must then stay clear of the stock
                    // there may still be. Trailing, it meets only what the
                    // cutting edges have already cut. (Along an arc piece
                    // the motion turns between its end tangents: a side that
                    // faces neither faces no direction between them.)
                    let mut reach = f64::NEG_INFINITY;
                    for (piece, normal) in back {
                        let leads = points.windows(2).any(|w| {
                            normal[0] * (w[1][0] - w[0][0]) + normal[1] * (w[1][1] - w[0][1]) > 0.
                        });
                        if !leads {
                            continue;
                        }
                        let swept = carried(piece, points);
                        reach = reach.max(if tool.inner {
                            outside(&stock_bore(&after), &swept, self.guard)
                        } else {
                            stock_envelope(&after).penetration(&swept, self.guard)
                        });
                    }
                    if reach > 0. {
                        return Err(format!(
                            "{op}: the depth of cut reaches the insert's back, beyond its cutting edges ({reach:.4} mm) moving to r {:.3}, z {:.3}; take less per pass",
                            to[1], to[0]
                        ));
                    }
                    let side = if tool.inner {
                        TurnSide::Inner
                    } else {
                        TurnSide::Outer
                    };
                    // The region the insert sweeps: its nose (or noses)
                    // exactly, and each straight side's parallelogram,
                    // exactly.
                    for c in sweeps {
                        let changes = after.preview_within(
                            &c,
                            tool.rn,
                            side,
                            self.job.tolerance_mm,
                            step.slack,
                        )?;
                        after.apply(&changes)?;
                    }
                    for e in &tool.edges {
                        if step.sag == 0. {
                            let piece = carried(e, points);
                            let changes = after.preview_polygon(&piece, side, 0.);
                            after.apply(&changes)?;
                            continue;
                        }
                        // Along an arc piece a side sweeps a curved band:
                        // material may be gone anywhere in the hull of its
                        // poses (nothing certain: infinite excess), and is
                        // certainly gone inside the band (`certain_band`).
                        let changes =
                            after.preview_polygon(&carried(e, points), side, f64::INFINITY);
                        after.apply(&changes)?;
                        if let Some(band) = certain_band(e, points, step.sag, step.slack) {
                            let changes = after.preview_polygon(&band, side, 0.);
                            after.apply(&changes)?;
                        }
                    }
                } else {
                    let side = if tool.inner {
                        TurnSide::Inner
                    } else {
                        TurnSide::Outer
                    };
                    for c in sweeps {
                        let changes = after.preview_within(
                            &c,
                            tool.rn,
                            side,
                            self.job.tolerance_mm,
                            step.slack,
                        )?;
                        after.apply(&changes)?;
                    }
                }
            }
            let stock = stock_envelope(&after);
            let open = stock_bore(&after);
            let holder_hit = holder
                .iter()
                .map(|piece| {
                    if tool.inner {
                        // The bar or drill body inside the bore the stock
                        // has opened.
                        outside(&open, piece, self.guard)
                    } else {
                        stock.penetration(piece, self.guard)
                    }
                })
                .fold(f64::NEG_INFINITY, f64::max);
            if holder_hit > 0. {
                return Err(format!(
                    "{op}: the holder would hit the stock ({holder_hit:.4} mm) moving to r {:.3}, z {:.3}",
                    to[1], to[0]
                ));
            }
            report.min_holder_clearance_mm =
                report.min_holder_clearance_mm.min(-holder_hit - self.guard);
            // The insert's body below its cutting face against the stock
            // this step leaves (#103): it must reach the stock no farther
            // than the cutting face itself does (the face's own reach is the
            // cut, judged by the rules above). Beyond it, the heel rubs the
            // wall just cut: a too thick or too little cleared insert for
            // this radius or seat.
            let reach = |pieces: &[Vec<[f64; 2]>]| {
                pieces
                    .iter()
                    .map(|piece| {
                        if tool.inner {
                            outside(&open, piece, self.guard)
                        } else {
                            stock.penetration(piece, self.guard)
                        }
                    })
                    .fold(f64::NEG_INFINITY, f64::max)
            };
            let (body_reach, face_reach) = (reach(&insert), reach(&face));
            if kind != MoveKind::Rapid && body_reach > face_reach.max(0.) + self.guard {
                let note = flank_note(face_reach.max(0.) + self.guard, &|near, _| Ok(reach(near)));
                return Err(format!(
                    "{op}: the insert's body below its cutting face would rub the stock ({:.4} mm beyond the cutting face) moving to r {:.3}, z {:.3}; it needs more clearance here (a thinner or more positive insert, or a different seat){note}",
                    body_reach - face_reach.max(0.),
                    to[1],
                    to[0]
                ));
            }
            touches |= insert.iter().any(|piece| {
                if tool.inner {
                    // Inside the bore the stock certainly has open.
                    outside(&open, piece, self.guard) > 0.
                } else {
                    stock.penetration(piece, self.guard) > 0.
                }
            });
        }
        // A rapid straight back along the last cutting move's line (out of a
        // groove or plunge) passes only through space that move's insert
        // swept: certified clear of the stock.
        let retraces = self.last_cut.is_some_and(|(a, b)| {
            let (d, e) = (
                [b[0] - a[0], b[1] - a[1]],
                [to[0] - from[0], to[1] - from[1]],
            );
            from == b
                && (d[0] * e[1] - d[1] * e[0]).abs()
                    <= 1e-12 * (1. + d[0].abs() + d[1].abs()) * (1. + e[0].abs() + e[1].abs())
                && d[0] * e[0] + d[1] * e[1] < 0.
        });
        // Pulling straight out at the end of a threading pass: the insert is
        // in the groove it is cutting at that spindle angle (the crests are
        // elsewhere around), as every threading cycle does.
        let pullout = self.last_thread == Some(from)
            && to[0] == from[0]
            && if tool.inner {
                to[1] < from[1]
            } else {
                to[1] > from[1]
            };
        if kind == MoveKind::Rapid && !retraces && !pullout && touches {
            return Err(format!(
                "{op}: a rapid would touch the stock moving to r {:.3}, z {:.3}",
                to[1], to[0]
            ));
        }
        // Only a straight cut's line can be retraced by a rapid.
        self.last_cut = (kind != MoveKind::Rapid && arc.is_none()).then_some((from, to));
        self.last_thread = (kind == MoveKind::Thread).then_some(to);
        let delta = TurnDelta {
            motion_id: self.next_id,
            changes: if kind == MoveKind::Rapid {
                vec![]
            } else {
                self.stock.changes_to(&after)
            },
            rings: if kind == MoveKind::Rapid {
                vec![]
            } else {
                after.rings()[self.stock.rings().len()..].to_vec()
            },
        };
        if kind != MoveKind::Rapid {
            self.stock = after;
            report.cut_length_mm += match arc {
                None => (to[0] - from[0]).hypot(to[1] - from[1]),
                Some((c, sweep)) => (from[0] - c[0]).hypot(from[1] - c[1]) * sweep.abs(),
            };
        }
        self.deltas.push(delta);
        // `feed` is per revolution: per minute at the highest spindle speed
        // along the move (constant surface speed turns faster inward).
        // The program moves the controlled point (the controller's CSS
        // follows its X): the nose center's path translated by the tip
        // offset, so an arc stays an arc of the same radius and sweep.
        let t = tool.tip;
        let tip = |p: [f64; 2]| [p[1] + t[1], 0., p[0] + t[0]];
        let curve = match arc {
            None => Curve::Line {
                start: tip(from),
                end: tip(to),
            },
            Some((c, sweep)) => Curve::Arc {
                start: tip(from),
                end: tip(to),
                center: tip(c),
                plane: Plane::Xz,
                sweep_rad: sweep,
            },
        };
        let top = match self.spindle {
            Spin::Rpm(r) => r,
            Spin::Css(v, max) => crate::motion::css_rpm_bound(&curve, v, max),
        };
        self.events.push(Event::Motion {
            motion: Motion {
                id: self.next_id,
                operation: op.into(),
                tool: tool.id.clone(),
                curve,
                kind,
                feed_mm_min: feed.map(|f| f * top),
            },
        });
        self.next_id += 1;
        report.moves += 1;
        self.position = to;
        Ok(())
    }
}

/// One checked piece of a move.
struct Step {
    /// The nose center stays in the hull of these: a line's two ends; an arc
    /// piece's two ends and the corner of its end tangents between them
    /// (the first three points).
    points: Vec<[f64; 2]>,
    /// The nose center's path over the piece (the stock's cut).
    curves: Vec<Curve>,
    /// An arc piece's sagitta: how far it bulges from its chord (0: a line).
    sag: f64,
    /// How far the actual path may stray from `curves` (an arc whose end is
    /// not exactly on its circle; 0 for a line).
    slack: f64,
}

/// The pieces an arc of the nose center is checked in: from `from` about
/// `center` through `sweep` (radians from +z toward +r) to `to`.
///
/// Each piece's hull is its two ends and the corner of the tangents at them
/// (the arc lies inside that triangle, so a convex shape carried along the
/// piece stays inside the hull of its three poses). The pieces are those
/// `chain` approximates the same arc with (sagitta at most `tolerance / 4`;
/// more only where that would leave a piece over a quarter turn). `chain`'s
/// chords run from tangent corner to tangent corner through the piece ends,
/// so everything they swept lies in these hulls: a check passes on the arc
/// only where it passed on the chords.
///
/// An end off the start's circle (within `tolerance`: an arc read back from
/// rounded words; a planned one is off by rounding errors only) is what a
/// controller reaches by blending the radius from the start's to the end's
/// (LinuxCNC interp_arc.cc: a spiral; docs/posts.md). Every piece's hull
/// then also holds its triangle at the end's radius (any path between the
/// two circles stays inside), the stock allows for a path that far from the
/// start's circle (`slack`), and the last piece ends on `to`.
fn arc_steps(
    from: [f64; 2],
    to: [f64; 2],
    center: [f64; 2],
    sweep: f64,
    tolerance: f64,
) -> Result<Vec<Step>, String> {
    use std::f64::consts::{FRAC_PI_2, TAU};
    let r = (from[0] - center[0]).hypot(from[1] - center[1]);
    if !(r.is_finite() && r >= 1e-6 && sweep.is_finite() && (1e-12..=TAU).contains(&sweep.abs())) {
        return Err(
            "a lathe arc needs a radius of at least 1e-6 mm and a sweep of at most one turn".into(),
        );
    }
    let a0 = (from[1] - center[1]).atan2(from[0] - center[0]);
    let on = |a: f64, radius: f64| [center[0] + radius * a.cos(), center[1] + radius * a.sin()];
    let nominal = on(a0 + sweep, r);
    let slack = (nominal[0] - to[0]).hypot(nominal[1] - to[1]);
    if !matches!(
        slack.partial_cmp(&tolerance),
        Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
    ) {
        return Err(format!(
            "the arc's end (r {:.4}, z {:.4}) is not on its circle and sweep within the tolerance {tolerance}",
            to[1], to[0]
        ));
    }
    let r_end = (to[0] - center[0]).hypot(to[1] - center[1]);
    // At most a quarter turn per piece: its tangents meet at a corner.
    let n = ((sweep.abs() * (r / (2. * tolerance)).sqrt()).ceil() as usize)
        .max((sweep.abs() / FRAC_PI_2).ceil() as usize)
        .clamp(1, 4096);
    let step = sweep / n as f64;
    let grow = 1. / (step.abs() / 2.).cos();
    let xz = |p: [f64; 2]| [p[1], 0., p[0]];
    Ok((0..n)
        .map(|k| {
            let (a, b) = (a0 + step * k as f64, a0 + step * (k + 1) as f64);
            let p = if k == 0 { from } else { on(a, r) };
            let q = on(b, r);
            let mut points = vec![p, on((a + b) / 2., r * grow), q];
            let mut curves = vec![Curve::Arc {
                start: xz(p),
                end: xz(q),
                center: xz(center),
                plane: Plane::Xz,
                sweep_rad: step,
            }];
            if r_end != r {
                points.extend([on(a, r_end), on((a + b) / 2., r_end * grow), on(b, r_end)]);
            }
            if k + 1 == n && q != to {
                points.push(to);
                curves.push(Curve::Line {
                    start: xz(q),
                    end: xz(to),
                });
            }
            Step {
                points,
                curves,
                sag: r * (1. - (step.abs() / 2.).cos()),
                slack,
            }
        })
        .collect())
}

/// A convex region certainly swept by the straight side `e` of an insert
/// (relative to the nose center) while the nose center follows an arc piece
/// from near `points[0]` to near `points[2]`: a path that starts and ends
/// within `slack` of those and stays within `sag + slack` of their chord.
///
/// The side carried along the chord sweeps a parallelogram; along the path
/// each pose is a chord pose shifted by h·n (n the chord's normal,
/// |h| <= H = sag + slack). With n = α·ê + β·ĉ (ê along the side, ĉ along
/// the chord; α = 1 / (ê·n), β = -α (ê·ĉ)), the path's pose at chord
/// position a is the side at a + β·h along the chord, shifted α·h along
/// itself. a + β·h runs continuously from within slack·(1 + |β|) of the
/// chord's start to within that of its end, so it takes every value τ in
/// between; there the side's point s - α·h is at the parallelogram's point
/// (s, τ), and it is on the side whenever s is at least |α|·H from both of
/// its ends. So the side shortened by H / |ê·n| at each end, carried along
/// the chord shortened by slack·(1 + |β|) at each end, is swept.
fn certain_band(
    e: &[[f64; 2]; 2],
    points: &[[f64; 2]],
    sag: f64,
    slack: f64,
) -> Option<Vec<[f64; 2]>> {
    let (p, q) = (points[0], points[2]);
    let d = [e[1][0] - e[0][0], e[1][1] - e[0][1]];
    let chord = [q[0] - p[0], q[1] - p[1]];
    let (l, c) = (d[0].hypot(d[1]), chord[0].hypot(chord[1]));
    if !(l > 0. && c > 0.) {
        return None;
    }
    let (ue, uc) = ([d[0] / l, d[1] / l], [chord[0] / c, chord[1] / c]);
    let alpha = 1. / (-ue[0] * uc[1] + ue[1] * uc[0]);
    let beta = -alpha * (ue[0] * uc[0] + ue[1] * uc[1]);
    let trim = alpha.abs() * (sag + slack);
    let cut = slack * (1. + beta.abs());
    if !(trim.is_finite() && cut.is_finite() && 2. * trim < l && 2. * cut < c) {
        return None;
    }
    let (a, b) = (
        [e[0][0] + trim * ue[0], e[0][1] + trim * ue[1]],
        [e[1][0] - trim * ue[0], e[1][1] - trim * ue[1]],
    );
    let (p, q) = (
        [p[0] + cut * uc[0], p[1] + cut * uc[1]],
        [q[0] - cut * uc[0], q[1] - cut * uc[1]],
    );
    Some(hull(vec![
        [a[0] + p[0], a[1] + p[1]],
        [b[0] + p[0], b[1] + p[1]],
        [a[0] + q[0], a[1] + q[1]],
        [b[0] + q[0], b[1] + q[1]],
    ]))
}

/// Chain points of an offset path (arcs as chords within `sag`, pushed
/// outward so the chain stays at least the offset away).
fn chain(path: &[Seg], sag: f64) -> Vec<[f64; 2]> {
    let mut points: Vec<[f64; 2]> = vec![];
    for s in path {
        match *s {
            Seg::Line { a, b } => {
                if points.last() != Some(&a) {
                    points.push(a);
                }
                points.push(b);
            }
            Seg::Arc { a, b, c, sweep } => {
                let r = (a[0] - c[0]).hypot(a[1] - c[1]);
                let a0 = (a[1] - c[1]).atan2(a[0] - c[0]);
                let n = ((sweep.abs() * (r / (8. * sag)).sqrt()).ceil() as usize).clamp(1, 4096);
                let step = sweep / n as f64;
                // Corners of the tangent lines: the chords stay outside the
                // arc, so the nose never comes closer than the offset.
                let grow = r / (step.abs() / 2.).cos();
                if points.last() != Some(&a) {
                    points.push(a);
                }
                for k in 0..n {
                    let t = a0 + step * (k as f64 + 0.5);
                    points.push([c[0] + grow * t.cos(), c[1] + grow * t.sin()]);
                }
                points.push(b);
            }
        }
    }
    points
}

/// Whether a finishing path's segment is cut as one native arc (G2/G3,
/// #14) rather than as `chain`'s chords: an arc that `chain` would need
/// more than one step for at the chord sagitta `sag` (a flatter one is a
/// straight move within the tolerance already), followed by a single nose
/// (a grooving blade's flat bottom does not follow an arc).
fn native_arc(tool: &ToolGeom, seg: &Seg, sag: f64) -> bool {
    let Seg::Arc { a, c, sweep, .. } = *seg else {
        return false;
    };
    let r = (a[0] - c[0]).hypot(a[1] - c[1]);
    tool.blade == 0.
        && !tool.axial
        && tool.drill.is_none()
        && r >= 1e-6
        && sweep.abs() <= std::f64::consts::TAU
        && sweep.abs() * (r / (8. * sag)).sqrt() > 1.
}

/// The least coordinate (`axis` 0: z, 1: r) along a segment.
fn seg_low(seg: &Seg, axis: usize) -> f64 {
    let ends = seg.start()[axis].min(seg.end()[axis]);
    let Seg::Arc { a, c, sweep, .. } = *seg else {
        return ends;
    };
    use std::f64::consts::{FRAC_PI_2, PI, TAU};
    // The circle's lowest point along the axis, if the arc passes it.
    let lowest = if axis == 0 { PI } else { -FRAC_PI_2 };
    let a0 = (a[1] - c[1]).atan2(a[0] - c[0]);
    let travel = if sweep > 0. {
        (lowest - a0).rem_euclid(TAU)
    } else {
        (a0 - lowest).rem_euclid(TAU)
    };
    if travel <= sweep.abs() {
        ends.min(c[axis] - (a[0] - c[0]).hypot(a[1] - c[1]))
    } else {
        ends
    }
}

/// The largest z (u) at which the offset chain reaches radius v.
fn first_reach(points: &[[f64; 2]], v: f64) -> Option<f64> {
    let mut best: Option<f64> = None;
    let mut take = |u: f64| best = Some(best.map_or(u, |b: f64| b.max(u)));
    // A pass at a diameter's own level runs along it (strictly above).
    let above = v + 1e-9;
    for w in points.windows(2) {
        let (p, q) = (w[0], w[1]);
        if p[1] > above {
            take(p[0]);
        }
        if q[1] > above {
            take(q[0]);
        }
        if (p[1] - v) * (q[1] - v) < 0. {
            take(p[0] + (v - p[1]) / (q[1] - p[1]) * (q[0] - p[0]));
        }
    }
    best
}

/// Roughing pass levels from `from` toward `to`, each at most `step` from
/// the last, with a pass at every flat diameter of the path between them
/// (over z in `[z_lo, z_hi]`): each diameter is left at its allowance, not
/// up to a depth of cut above it.
fn pass_levels(
    points: &[[f64; 2]],
    [z_lo, z_hi]: [f64; 2],
    from: f64,
    to: f64,
    step: f64,
) -> Vec<f64> {
    let dir = if to < from { -1. } else { 1. };
    let ahead = |a: f64, b: f64| (b - a) * dir > 1e-6; // b strictly past a
    let mut flats: Vec<f64> = points
        .windows(2)
        .filter(|w| {
            (w[0][1] - w[1][1]).abs() < 1e-9
                && (w[0][0] - w[1][0]).abs() > 1e-6
                && w[0][0].max(w[1][0]) > z_lo
                && w[0][0].min(w[1][0]) < z_hi
        })
        .map(|w| w[0][1])
        .filter(|&v| ahead(from, v) && ahead(v, to))
        .collect();
    flats.sort_by(|a, b| ((a - b) * dir).total_cmp(&0.));
    let mut levels = vec![];
    let mut v = from;
    while ahead(v, to) {
        let mut next = v + dir * step;
        if !ahead(next, to) {
            next = to;
        }
        if let Some(&f) = flats.iter().find(|&&f| ahead(v, f) && ahead(f, next)) {
            next = f;
        }
        levels.push(next);
        v = next;
    }
    levels
}

/// A radial plunge of the nose center at `u` from `top` down to `bottom`,
/// in pecks (retracting half a peck after each) when `peck` is set.
#[allow(clippy::too_many_arguments)]
fn plunge(
    p: &mut Planner,
    geom: &ToolGeom,
    id: &str,
    u: f64,
    top: f64,
    bottom: f64,
    peck: Option<f64>,
    feed: Option<f64>,
    report: &mut TurnReport,
) -> Result<(), String> {
    // Toward the axis outside, away from it inside a bore.
    let s = if bottom < top { 1. } else { -1. };
    match peck {
        Some(step) if step > 0. => {
            let mut r = top;
            while s * (r - bottom) > 1e-9 {
                r = if s > 0. {
                    (r - step).max(bottom)
                } else {
                    (r + step).min(bottom)
                };
                p.emit(geom, id, [u, r], MoveKind::Plunge, feed, report)?;
                if r != bottom {
                    let back = r + s * step / 2.;
                    let back = if s > 0. { back.min(top) } else { back.max(top) };
                    p.emit(geom, id, [u, back], MoveKind::Retract, feed, report)?;
                }
            }
            Ok(())
        }
        Some(_) => Err(format!("{id}: peck_mm must be positive")),
        None => p.emit(geom, id, [u, bottom], MoveKind::Plunge, feed, report),
    }
}

/// The error a cancelled plan returns (nothing is returned with it).
pub const CANCELLED: &str = "turning plan cancelled; no result";

/// Plan a turning job. ``cancel`` is polled before every move and operation.
/// A turning plan built one operation at a time (the stream plans each
/// operation when it is reached). Every move is checked as it is planned and
/// carries the exact stock change it makes.
pub struct TurnSession<'a> {
    job: &'a TurningJob,
    planner: Planner<'a>,
    top: f64,
    z_start: f64,
    tol: f64,
    z_hi: f64,
    reports: Vec<TurnReport>,
    active: Option<String>,
    feed_rev: Option<f64>,
    coolant: bool,
}

impl<'a> TurnSession<'a> {
    pub fn new(job: &'a TurningJob, cancel: &'a dyn CancelSignal) -> Result<Self, String> {
        if cancel.is_cancelled() {
            return Err(CANCELLED.into());
        }
        let stock = TurnStock::new(job.stock.clone())?;
        let tol = job.tolerance_mm;
        if !(1e-6..=0.05).contains(&tol) || !(job.clearance_mm.is_finite() && job.clearance_mm > 0.)
        {
            return Err("turning tolerance_mm must be 1e-6..0.05 and clearance_mm positive".into());
        }
        let [z_lo, z_hi] = job.part.z_range();
        if job.chuck_z_mm > z_lo || z_hi > job.stock.z_max_mm || z_lo < job.stock.z_min_mm {
            return Err("the part must lie within the stock and in front of the chuck face".into());
        }
        // Face grooves lie within the flat front face (the profile's points at
        // the face z, from the axis out).
        let face_radius = job
            .part
            .points
            .iter()
            .take_while(|p| p[1] == z_hi)
            .map(|p| p[0])
            .fold(0., f64::max);
        for g in &job.face_grooves {
            if ![g.inner_radius_mm, g.outer_radius_mm, g.bottom_z_mm]
                .iter()
                .all(|v| v.is_finite())
                || g.inner_radius_mm < 0.
                || g.outer_radius_mm <= g.inner_radius_mm
                || g.outer_radius_mm >= face_radius
                || g.bottom_z_mm >= z_hi
                || g.bottom_z_mm <= z_lo
            {
                return Err(format!(
                    "face groove {g:?} must lie within the flat front face (out to r {face_radius}), its bottom behind the face and inside the part"
                ));
            }
        }
        let planner = Planner {
            job,
            cancel,
            part: job.part.envelope(tol / 4.)?,
            bore: job
                .bore
                .as_ref()
                .map(|b| b.envelope(tol / 4.))
                .transpose()?,
            stock,
            events: vec![Event::WorkOffset { number: 1 }],
            deltas: vec![],
            position: [job.initial_position_mm[1], job.initial_position_mm[0]],
            next_id: 1,
            guard: tol,
            last_cut: None,
            last_thread: None,
            spindle: Spin::Rpm(0.),
        };
        Ok(Self {
            job,
            planner,
            top: job.stock.outer_radius_mm,
            z_start: job.stock.z_max_mm + job.clearance_mm,
            tol,
            z_hi,
            reports: vec![],
            active: None,
            feed_rev: None,
            coolant: false,
        })
    }

    /// Plan one operation (in job order), appending its events and moves.
    /// `next_tool`: the next operation's tool (None for the last). Before a
    /// different tool, this operation puts its tool away: spindle and coolant
    /// off, back to the start (turret) position, attributed to this operation.
    pub fn operation(&mut self, op: &TurnOperation, next_tool: Option<&str>) -> Result<(), String> {
        let job = self.job;
        let (top, z_start, tol, z_hi) = (self.top, self.z_start, self.tol, self.z_hi);
        let cancel = self.planner.cancel;
        let mut active = self.active.take();
        let mut feed_rev = self.feed_rev;
        let mut coolant = self.coolant;
        let planner = &mut self.planner;
        let reports = &mut self.reports;
        if cancel.is_cancelled() {
            return Err(CANCELLED.into());
        }
        let (id, tool_id, rpm, css) = match op {
            TurnOperation::Face {
                id,
                tool,
                rpm,
                surface_speed_m_min,
                ..
            }
            | TurnOperation::Rough {
                id,
                tool,
                rpm,
                surface_speed_m_min,
                ..
            }
            | TurnOperation::Groove {
                id,
                tool,
                rpm,
                surface_speed_m_min,
                ..
            }
            | TurnOperation::PartOff {
                id,
                tool,
                rpm,
                surface_speed_m_min,
                ..
            }
            | TurnOperation::Thread {
                id,
                tool,
                rpm,
                surface_speed_m_min,
                ..
            }
            | TurnOperation::Drill {
                id,
                tool,
                rpm,
                surface_speed_m_min,
                ..
            }
            | TurnOperation::FaceGroove {
                id,
                tool,
                rpm,
                surface_speed_m_min,
                ..
            }
            | TurnOperation::BoreRough {
                id,
                tool,
                rpm,
                surface_speed_m_min,
                ..
            }
            | TurnOperation::BoreFinish {
                id,
                tool,
                rpm,
                surface_speed_m_min,
                ..
            }
            | TurnOperation::Finish {
                id,
                tool,
                rpm,
                surface_speed_m_min,
                ..
            } => (id.clone(), tool.clone(), *rpm, *surface_speed_m_min),
        };
        // The op's feed per revolution (threading: its lead, at fixed rpm).
        let op_feed = match op {
            TurnOperation::Face { feed_mm_rev, .. }
            | TurnOperation::Rough { feed_mm_rev, .. }
            | TurnOperation::Groove { feed_mm_rev, .. }
            | TurnOperation::PartOff { feed_mm_rev, .. }
            | TurnOperation::Drill { feed_mm_rev, .. }
            | TurnOperation::FaceGroove { feed_mm_rev, .. }
            | TurnOperation::BoreRough { feed_mm_rev, .. }
            | TurnOperation::BoreFinish { feed_mm_rev, .. }
            | TurnOperation::Finish { feed_mm_rev, .. } => Some(*feed_mm_rev),
            TurnOperation::Thread { .. } => None,
        };
        if let Some(v) = css {
            if !(v.is_finite() && v > 0. && v <= 1e5) {
                return Err(format!("{id}: surface_speed_m_min must be positive"));
            }
            if !job.feed_per_rev {
                return Err(format!(
                    "{id}: constant surface speed needs feeds per revolution (TurningJob.feed_per_rev): a feed per minute would change the chip load with the radius"
                ));
            }
            if matches!(
                op,
                TurnOperation::Thread { .. } | TurnOperation::Drill { .. }
            ) {
                return Err(format!(
                    "{id}: threading and drilling on the axis run at a fixed rpm (no surface_speed_m_min)"
                ));
            }
        }
        let setting = job
            .tools
            .iter()
            .find(|t| t.tool.id == tool_id)
            .ok_or_else(|| format!("{id}: tool {tool_id:?} is not in the job"))?;
        if !(rpm.is_finite() && rpm > 0.) {
            return Err(format!("{id}: rpm must be positive"));
        }
        let geom = planner.geometry(&setting.tool)?;
        let rn = geom.rn;
        if geom.axial != matches!(op, TurnOperation::FaceGroove { .. }) {
            return Err(format!(
                "{id}: face grooving needs a face grooving insert, which only face grooving uses"
            ));
        }
        let mut report = TurnReport {
            id: id.clone(),
            passes: 0,
            moves: 0,
            cut_length_mm: 0.,
            min_part_clearance_mm: f64::INFINITY,
            min_holder_clearance_mm: f64::INFINITY,
        };
        if active.as_deref() != Some(tool_id.as_str()) {
            if active.is_some() {
                return Err(format!(
                    "{id}: the previous tool was not put away (plan each operation with the next one's tool)"
                ));
            }
            planner.events.push(Event::ToolChange {
                tool: tool_id.clone(),
                number: setting.number,
                length_register: setting.offset,
                position_after_mm: [job.initial_position_mm[0], 0., job.initial_position_mm[1]],
            });
            // The new tool's controlled point arrives at the start position.
            planner.position = [
                job.initial_position_mm[1] - geom.tip[0],
                job.initial_position_mm[0] - geom.tip[1],
            ];
            active = Some(tool_id.clone());
        }
        // Spindle, feed mode and coolant only when they change.
        let spin = match css {
            Some(v) => Spin::Css(v, rpm),
            None => Spin::Rpm(rpm),
        };
        if planner.spindle != spin {
            planner.events.push(match spin {
                Spin::Css(m_min, max_rpm) => Event::SurfaceSpeed {
                    m_min,
                    max_rpm,
                    direction: SpindleDirection::Cw,
                },
                Spin::Rpm(rpm) => Event::Spindle {
                    rpm,
                    direction: SpindleDirection::Cw,
                },
            });
            planner.spindle = spin;
        }
        if job.feed_per_rev
            && let Some(f) = op_feed
            && feed_rev != Some(f)
            && f.is_finite()
            && f > 0.
        {
            planner.events.push(Event::FeedPerRev { mm_rev: f });
            feed_rev = Some(f);
        }
        if !coolant {
            planner.events.push(Event::Coolant {
                mode: Coolant::Flood,
            });
            coolant = true;
        }
        let safe_r = top + rn + job.clearance_mm;
        let feed_of = |f: f64| -> Result<Option<f64>, String> {
            if !(f.is_finite() && f > 0.) {
                return Err(format!("{id}: feed_mm_rev must be positive"));
            }
            Ok(Some(f))
        };
        // Approach: out to the safe radius, along to z_start.
        let approach =
            |p: &mut Planner, report: &mut TurnReport, target: [f64; 2]| -> Result<(), String> {
                let pos = p.position;
                p.emit(
                    &geom,
                    &id,
                    [pos[0], pos[1].max(safe_r)],
                    MoveKind::Rapid,
                    None,
                    report,
                )?;
                let pos = p.position;
                p.emit(
                    &geom,
                    &id,
                    [target[0], pos[1]],
                    MoveKind::Rapid,
                    None,
                    report,
                )?;
                p.emit(&geom, &id, target, MoveKind::Rapid, None, report)
            };
        match op {
            TurnOperation::Face {
                z_mm,
                stepdown_mm,
                feed_mm_rev,
                ..
            } => {
                let feed = feed_of(*feed_mm_rev)?;
                if !(stepdown_mm.is_finite() && *stepdown_mm > 0.)
                    || *z_mm >= job.stock.z_max_mm
                    || *z_mm < z_hi - tol
                {
                    return Err(format!(
                        "{id}: face z must be below the stock end and not into the part's front face; stepdown positive"
                    ));
                }
                let mut z = job.stock.z_max_mm;
                while z > *z_mm + tol {
                    z = (z - stepdown_mm).max(*z_mm);
                    let u = z + rn;
                    approach(&mut *planner, &mut report, [u, safe_r])?;
                    planner.emit(&geom, &id, [u, 0.], MoveKind::Cut, feed, &mut report)?;
                    planner.emit(
                        &geom,
                        &id,
                        [u + job.clearance_mm, 0.],
                        MoveKind::Retract,
                        feed,
                        &mut report,
                    )?;
                    report.passes += 1;
                }
            }
            TurnOperation::Rough {
                depth_of_cut_mm,
                allowance_mm,
                feed_mm_rev,
                retract_mm,
                ..
            } => {
                let feed = feed_of(*feed_mm_rev)?;
                if !(depth_of_cut_mm.is_finite()
                    && *depth_of_cut_mm > 0.
                    && allowance_mm.is_finite()
                    && *allowance_mm >= 0.
                    && *retract_mm > 0.)
                {
                    return Err(format!(
                        "{id}: depth_of_cut_mm and retract_mm must be positive, allowance_mm not negative"
                    ));
                }
                let path = job.part.offset(rn + allowance_mm)?;
                let points = chain(&path, tol / 4.);
                let z_end = job.chuck_z_mm + job.clearance_mm;
                let reach = |v: f64| first_reach(&points, v).map_or(z_end, |u| u.max(z_end));
                // The front face's offset is not a diameter to rough to.
                let lowest = points
                    .iter()
                    .filter(|p| p[0] >= z_end && p[0] <= z_hi)
                    .map(|p| p[1])
                    .fold(f64::INFINITY, f64::min)
                    .max(rn);
                let mut first = true;
                for v_pass in
                    pass_levels(&points, [z_end, z_hi], top + rn, lowest, *depth_of_cut_mm)
                {
                    let stop = reach(v_pass);
                    if stop < z_start - tol {
                        if first {
                            approach(&mut *planner, &mut report, [z_start, v_pass])?;
                            first = false;
                        } else {
                            // Beyond the stock end: straight down to the pass.
                            planner.emit(
                                &geom,
                                &id,
                                [z_start, v_pass],
                                MoveKind::Rapid,
                                None,
                                &mut report,
                            )?;
                        }
                        planner.emit(
                            &geom,
                            &id,
                            [stop, v_pass],
                            MoveKind::Cut,
                            feed,
                            &mut report,
                        )?;
                        planner.emit(
                            &geom,
                            &id,
                            [stop + retract_mm, v_pass + retract_mm],
                            MoveKind::Retract,
                            feed,
                            &mut report,
                        )?;
                        let pos = planner.position;
                        planner.emit(
                            &geom,
                            &id,
                            [z_start, pos[1]],
                            MoveKind::Rapid,
                            None,
                            &mut report,
                        )?;
                        report.passes += 1;
                    }
                }
            }
            TurnOperation::Groove {
                z_from_mm,
                z_to_mm,
                bottom_radius_mm,
                peck_mm,
                overlap_mm,
                feed_mm_rev,
                ..
            } => {
                let feed = feed_of(*feed_mm_rev)?;
                let width = geom.blade.abs() + 2. * rn;
                let (z_lo, z_hi) = (z_from_mm.min(*z_to_mm), z_from_mm.max(*z_to_mm));
                if geom.blade == 0.
                    || z_hi - z_lo < width - tol
                    || !(*overlap_mm >= 0. && *overlap_mm < width)
                {
                    return Err(format!(
                        "{id}: grooving needs a grooving insert no wider than the groove ({width:.3} mm) and an overlap below its width"
                    ));
                }
                // Leading-corner nose centers: the first plunge against the
                // -z wall, the last against the +z wall.
                let first = z_lo + rn;
                let last = z_hi - rn - geom.blade.abs();
                // A groove one blade wide within the tolerance: one plunge,
                // centred (at most tol/2 off either wall). Otherwise a
                // rounding-sized span would add a pass a hair from the
                // first, a zero-length move (#101).
                let (first, last, n) = if last - first <= tol {
                    let middle = (first + last) / 2.;
                    (middle, middle, 0)
                } else {
                    (
                        first,
                        last,
                        ((last - first) / (width - overlap_mm)).ceil() as usize,
                    )
                };
                // Inside a bore (an internal groove) the plunges go outward
                // from a radius clear of the open hole, which the tool enters
                // and leaves along the axis.
                let inner = geom.inner;
                let (bottom, clear) = if inner {
                    if geom.drill.is_some() {
                        return Err(format!("{id}: grooving needs a grooving insert"));
                    }
                    // The hole the tool enters through: the smallest certain
                    // inner radius over the bins in front of the groove that
                    // may still hold material. A bin with none (its outer
                    // bound at or below its inner one, e.g. faced off) is
                    // open space and limits nothing (#102).
                    let open = (0..planner.stock.cells().len())
                        .filter(|&i| {
                            let [a, b] = planner.stock.bin(i);
                            let c = planner.stock.cells()[i];
                            b > z_lo
                                && a < job.stock.z_max_mm
                                && c.outer.upper_mm > c.inner.lower_mm
                        })
                        .map(|i| planner.stock.cells()[i].inner.lower_mm)
                        .fold(f64::INFINITY, f64::min);
                    let clear = open - rn - job.clearance_mm.min((open - rn) / 2.);
                    if !clear.is_finite() || clear <= 0. || *bottom_radius_mm <= open {
                        return Err(format!(
                            "{id}: an internal groove needs an open hole in front of it and its bottom outside that hole"
                        ));
                    }
                    (bottom_radius_mm - rn, clear)
                } else {
                    (bottom_radius_mm + rn, safe_r)
                };
                for k in 0..=n {
                    let z = if n == 0 {
                        first
                    } else {
                        first + (last - first) * k as f64 / n as f64
                    };
                    let u = if geom.blade < 0. {
                        z + geom.blade.abs()
                    } else {
                        z
                    };
                    if k == 0 && inner {
                        approach(&mut *planner, &mut report, [z_start, clear])?;
                        planner.emit(&geom, &id, [u, clear], MoveKind::Rapid, None, &mut report)?;
                    } else if k == 0 {
                        approach(&mut *planner, &mut report, [u, clear])?;
                    } else {
                        planner.emit(&geom, &id, [u, clear], MoveKind::Rapid, None, &mut report)?;
                    }
                    plunge(
                        &mut *planner,
                        &geom,
                        &id,
                        u,
                        clear,
                        bottom,
                        *peck_mm,
                        feed,
                        &mut report,
                    )?;
                    planner.emit(&geom, &id, [u, clear], MoveKind::Rapid, None, &mut report)?;
                    report.passes += 1;
                }
                if inner {
                    planner.emit(
                        &geom,
                        &id,
                        [z_start, clear],
                        MoveKind::Rapid,
                        None,
                        &mut report,
                    )?;
                }
            }
            TurnOperation::Thread {
                z_start_mm,
                z_end_mm,
                major_radius_mm,
                pitch_mm,
                depth_mm,
                passes,
                infeed_angle_deg,
                spring_passes,
                ..
            } => {
                let numbers = [
                    *z_start_mm,
                    *z_end_mm,
                    *major_radius_mm,
                    *pitch_mm,
                    *depth_mm,
                    *infeed_angle_deg,
                ];
                if !numbers.iter().all(|v| v.is_finite())
                    || *pitch_mm <= 0.
                    || *depth_mm <= 0.
                    || *depth_mm >= *major_radius_mm
                    || *passes == 0
                    || *passes > 200
                    || z_end_mm >= z_start_mm
                    || !(0. ..=45.).contains(infeed_angle_deg)
                    || !matches!(setting.tool.insert, super::tool::Insert::Thread { .. })
                {
                    return Err(format!(
                        "{id}: threading needs a threading insert, z_end < z_start, a positive pitch, depth below the major radius, 1..200 passes and an infeed angle 0..45"
                    ));
                }
                let internal = geom.inner;
                let root = major_radius_mm - depth_mm;
                let (part, bore) = (planner.part.clone(), planner.bore.clone());
                let (relieved_part, relieved_bore) = thread_relief(
                    op,
                    &setting.tool,
                    internal,
                    &planner.part,
                    planner.bore.as_ref(),
                )?;
                planner.part = relieved_part;
                planner.bore = relieved_bore;
                let feed = Some(*pitch_mm);
                let start = z_start_mm + 2. * pitch_mm + rn;
                // Outside: back out past the crests. Inside: in toward the
                // axis past the crests (the minor radius).
                let out = if internal {
                    (major_radius_mm - depth_mm - rn - job.clearance_mm.min(*depth_mm)).max(rn)
                } else {
                    major_radius_mm + rn + job.clearance_mm
                };
                let total = *passes + *spring_passes;
                for k in 1..=total {
                    let depth = depth_mm * ((k.min(*passes)) as f64 / *passes as f64).sqrt();
                    let shift = depth * infeed_angle_deg.to_radians().tan();
                    let v = if internal {
                        root + depth - rn
                    } else {
                        major_radius_mm - depth + rn
                    };
                    if k == 1 {
                        approach(&mut *planner, &mut report, [start + shift, out])?;
                    } else {
                        planner.emit(
                            &geom,
                            &id,
                            [start + shift, out],
                            MoveKind::Rapid,
                            None,
                            &mut report,
                        )?;
                    }
                    planner.emit(
                        &geom,
                        &id,
                        [start + shift, v],
                        MoveKind::Rapid,
                        None,
                        &mut report,
                    )?;
                    planner.emit(
                        &geom,
                        &id,
                        [z_end_mm + shift, v],
                        MoveKind::Thread,
                        feed,
                        &mut report,
                    )?;
                    planner.emit(
                        &geom,
                        &id,
                        [z_end_mm + shift, out],
                        MoveKind::Rapid,
                        None,
                        &mut report,
                    )?;
                    report.passes += 1;
                }
                if internal {
                    // Out of the bore along the axis, inside the crests.
                    let pos = planner.position;
                    planner.emit(
                        &geom,
                        &id,
                        [start, pos[1]],
                        MoveKind::Rapid,
                        None,
                        &mut report,
                    )?;
                }
                planner.part = part;
                planner.bore = bore;
            }
            TurnOperation::BoreRough {
                depth_of_cut_mm,
                allowance_mm,
                z_end_mm,
                feed_mm_rev,
                retract_mm,
                ..
            } => {
                let feed = feed_of(*feed_mm_rev)?;
                let bore = job.bore.as_ref().ok_or_else(|| {
                    format!("{id}: boring needs the part's bore (TurningJob.bore)")
                })?;
                if !geom.inner
                    || geom.drill.is_some()
                    || !(*depth_of_cut_mm > 0. && *allowance_mm >= 0. && *retract_mm > 0.)
                {
                    return Err(format!(
                        "{id}: inside roughing needs a boring tool (side inner), a positive depth of cut and retract, allowance not negative"
                    ));
                }
                let points = chain(&bore.bore_path(rn + allowance_mm)?, tol / 4.);
                let front = job.part.z_range()[1];
                // The hole already open: the least certain bore over the
                // length to rough.
                let open = (0..planner.stock.cells().len())
                    .filter(|&i| {
                        let [a, b] = planner.stock.bin(i);
                        b > *z_end_mm && a < front
                    })
                    .map(|i| planner.stock.cells()[i].inner.lower_mm)
                    .fold(f64::INFINITY, f64::min);
                if !open.is_finite() || open <= rn {
                    return Err(format!("{id}: no open hole to bore (drill it first)"));
                }
                let target = points
                    .iter()
                    .filter(|p| p[0] >= *z_end_mm && p[0] <= front)
                    .map(|p| p[1])
                    .fold(f64::NEG_INFINITY, f64::max);
                // The first z, moving in from the front, where the bore
                // allows the nose center no further out than v.
                let stop = |v: f64| {
                    let mut best: Option<f64> = None;
                    for w in points.windows(2) {
                        let (p, q) = (w[0], w[1]);
                        for x in [p, q] {
                            if x[1] < v - 1e-9 {
                                best = Some(best.map_or(x[0], |b: f64| b.max(x[0])));
                            }
                        }
                        if (p[1] - v) * (q[1] - v) < 0. {
                            let u = p[0] + (v - p[1]) / (q[1] - p[1]) * (q[0] - p[0]);
                            best = Some(best.map_or(u, |b: f64| b.max(u)));
                        }
                    }
                    best.map_or(*z_end_mm, |u| u.max(*z_end_mm))
                };
                let mut first = true;
                for v in pass_levels(
                    &points,
                    [*z_end_mm, front],
                    open - rn,
                    target,
                    *depth_of_cut_mm,
                ) {
                    let end = stop(v);
                    // Blocked at the bore's mouth: that level is only inside
                    // a groove further in, which axial passes cannot reach.
                    if end >= z_start - tol || end >= points[0][0] - tol {
                        continue;
                    }
                    if first {
                        approach(&mut *planner, &mut report, [z_start, v])?;
                        first = false;
                    } else {
                        planner.emit(
                            &geom,
                            &id,
                            [z_start, v],
                            MoveKind::Rapid,
                            None,
                            &mut report,
                        )?;
                    }
                    planner.emit(&geom, &id, [end, v], MoveKind::Cut, feed, &mut report)?;
                    planner.emit(
                        &geom,
                        &id,
                        [end + retract_mm, v - retract_mm],
                        MoveKind::Retract,
                        feed,
                        &mut report,
                    )?;
                    planner.emit(
                        &geom,
                        &id,
                        [z_start, v - retract_mm],
                        MoveKind::Rapid,
                        None,
                        &mut report,
                    )?;
                    report.passes += 1;
                }
            }
            TurnOperation::BoreFinish {
                allowance_mm,
                z_end_mm,
                feed_mm_rev,
                compensation,
                ..
            } => {
                let feed = feed_of(*feed_mm_rev)?;
                let bore = job.bore.as_ref().ok_or_else(|| {
                    format!("{id}: boring needs the part's bore (TurningJob.bore)")
                })?;
                if !geom.inner || geom.drill.is_some() {
                    return Err(format!(
                        "{id}: inside finishing needs a boring tool (side inner)"
                    ));
                }
                let path = bore.bore_path(rn + allowance_mm)?;
                let first = path[0].start();
                approach(&mut *planner, &mut report, [z_start, first[1]])?;
                // Wear compensation: inside, the tool is on the left of the
                // path cutting toward the chuck (G41).
                let wear = *compensation == crate::contour::Compensation::Wear;
                if wear {
                    planner.events.push(Event::CutterCompensation {
                        side: CompensationSide::Left,
                        plane: crate::motion::Plane::Xz,
                    });
                }
                planner.emit(&geom, &id, first, MoveKind::Plunge, feed, &mut report)?;
                if first[0] < *z_end_mm {
                    return Err(format!("{id}: z_end_mm is in front of the bore"));
                }
                // Along the bore to z_end (the last segment cut short there);
                // out again inside the least radius passed.
                // An arc of the path wholly before z_end is one arc move
                // (G2/G3, #14); any other as its chords.
                let mut low = first[1];
                'path: for seg in &path {
                    let start = seg.start();
                    if planner.position != start {
                        planner.emit(&geom, &id, start, MoveKind::Cut, feed, &mut report)?;
                        low = low.min(start[1]);
                    }
                    if let Seg::Arc { b, c, sweep, .. } = *seg
                        && native_arc(&geom, seg, tol / 4.)
                        && seg_low(seg, 0) > *z_end_mm
                    {
                        planner.emit_arc(
                            &geom,
                            &id,
                            c,
                            sweep,
                            b,
                            MoveKind::Cut,
                            feed,
                            &mut report,
                        )?;
                        low = low.min(seg_low(seg, 1));
                        continue;
                    }
                    let points = chain(std::slice::from_ref(seg), tol / 4.);
                    for w in points.windows(2) {
                        let (a, b) = (w[0], w[1]);
                        let p = if b[0] < *z_end_mm {
                            let t = (a[0] - z_end_mm) / (a[0] - b[0]);
                            [*z_end_mm, a[1] + t * (b[1] - a[1])]
                        } else {
                            b
                        };
                        if p != a {
                            planner.emit(&geom, &id, p, MoveKind::Cut, feed, &mut report)?;
                        }
                        low = low.min(p[1]);
                        if b[0] <= *z_end_mm {
                            break 'path;
                        }
                    }
                }
                let pos = planner.position;
                if wear {
                    planner.events.push(Event::CutterCompensation {
                        side: CompensationSide::Off,
                        plane: crate::motion::Plane::Xz,
                    });
                }
                let back = (low - job.clearance_mm).max(0.);
                // At 45 degrees toward the axis and the front, off the wall
                // and any step or bottom the cut ended at.
                planner.emit(
                    &geom,
                    &id,
                    [pos[0] + (pos[1] - back), back],
                    MoveKind::Retract,
                    feed,
                    &mut report,
                )?;
                planner.emit(
                    &geom,
                    &id,
                    [z_start, back],
                    MoveKind::Rapid,
                    None,
                    &mut report,
                )?;
                report.passes += 1;
            }
            TurnOperation::FaceGroove {
                inner_radius_mm,
                outer_radius_mm,
                bottom_z_mm,
                peck_mm,
                overlap_mm,
                feed_mm_rev,
                ..
            } => {
                let feed = feed_of(*feed_mm_rev)?;
                let groove = FaceGroove {
                    inner_radius_mm: *inner_radius_mm,
                    outer_radius_mm: *outer_radius_mm,
                    bottom_z_mm: *bottom_z_mm,
                };
                if !job.face_grooves.contains(&groove) {
                    return Err(format!(
                        "{id}: declare the groove in TurningJob.face_grooves (the part's own geometry the blade is checked against)"
                    ));
                }
                // A blade curved to follow a groove serves its maker's
                // diameter range only (its body is checked as following it).
                if let Some([lo, hi]) = setting
                    .tool
                    .body
                    .as_ref()
                    .and_then(|b| b.insert.as_ref())
                    .and_then(|b| b.curved_for_diameter_mm)
                    && (2. * inner_radius_mm < lo - tol || 2. * outer_radius_mm > hi + tol)
                {
                    return Err(format!(
                        "{id}: the groove (diameters {:.3}..{:.3} mm) is outside the curved blade's range {lo}..{hi} mm",
                        2. * inner_radius_mm,
                        2. * outer_radius_mm
                    ));
                }
                let width = geom.blade + 2. * rn;
                let span = outer_radius_mm - inner_radius_mm;
                if span < width - tol || !(*overlap_mm >= 0. && *overlap_mm < width) {
                    return Err(format!(
                        "{id}: the face grooving insert ({width:.3} mm) must fit the groove ({span:.3} mm) and overlap below its width"
                    ));
                }
                let face = job.part.z_range()[1];
                // Inner-corner nose centers across the groove, plunging from
                // in front of the face to the bottom.
                let first = inner_radius_mm + rn;
                let last = outer_radius_mm - rn - geom.blade;
                // A groove one blade wide within the tolerance: one plunge,
                // centred (at most tol/2 off either wall). Otherwise a
                // rounding-sized span would add a pass a hair from the
                // first, a zero-length move (#101).
                let (first, last, n) = if last - first <= tol {
                    let middle = (first + last) / 2.;
                    (middle, middle, 0)
                } else {
                    (
                        first,
                        last,
                        ((last - first) / (width - overlap_mm)).ceil() as usize,
                    )
                };
                let bottom = bottom_z_mm + rn;
                let entry = face.max(job.stock.z_max_mm) + rn + job.clearance_mm;
                for k in 0..=n {
                    let v = if n == 0 {
                        first
                    } else {
                        first + (last - first) * k as f64 / n as f64
                    };
                    if k == 0 {
                        approach(&mut *planner, &mut report, [entry, v])?;
                    } else {
                        planner.emit(&geom, &id, [entry, v], MoveKind::Rapid, None, &mut report)?;
                    }
                    match peck_mm {
                        Some(step) if *step > 0. => {
                            let mut u = entry;
                            while u > bottom + 1e-9 {
                                u = (u - step).max(bottom);
                                planner.emit(
                                    &geom,
                                    &id,
                                    [u, v],
                                    MoveKind::Plunge,
                                    feed,
                                    &mut report,
                                )?;
                                if u > bottom {
                                    let back = (u + step / 2.).min(entry);
                                    planner.emit(
                                        &geom,
                                        &id,
                                        [back, v],
                                        MoveKind::Retract,
                                        feed,
                                        &mut report,
                                    )?;
                                }
                            }
                        }
                        Some(_) => return Err(format!("{id}: peck_mm must be positive")),
                        None => planner.emit(
                            &geom,
                            &id,
                            [bottom, v],
                            MoveKind::Plunge,
                            feed,
                            &mut report,
                        )?,
                    }
                    planner.emit(&geom, &id, [entry, v], MoveKind::Rapid, None, &mut report)?;
                    report.passes += 1;
                }
            }
            TurnOperation::Drill {
                depth_z_mm,
                peck_mm,
                feed_mm_rev,
                ..
            } => {
                let feed = feed_of(*feed_mm_rev)?;
                if geom.drill.is_none()
                    || !depth_z_mm.is_finite()
                    || *depth_z_mm >= job.stock.z_max_mm
                {
                    return Err(format!(
                        "{id}: drilling needs a drill tool and a depth below the stock's end"
                    ));
                }
                // On the axis, from in front of the stock, out the same way.
                approach(&mut *planner, &mut report, [z_start, 0.])?;
                match peck_mm {
                    Some(step) if *step > 0. => {
                        let mut z = z_start;
                        let face = job.stock.z_max_mm;
                        while z > depth_z_mm + tol {
                            let to = (z.min(face) - step).max(*depth_z_mm);
                            planner.emit(
                                &geom,
                                &id,
                                [to, 0.],
                                MoveKind::Plunge,
                                feed,
                                &mut report,
                            )?;
                            planner.emit(
                                &geom,
                                &id,
                                [z_start, 0.],
                                MoveKind::Rapid,
                                None,
                                &mut report,
                            )?;
                            if to > depth_z_mm + tol {
                                // Back in to just above the last depth.
                                planner.emit(
                                    &geom,
                                    &id,
                                    [to + job.clearance_mm.min(*step) / 2., 0.],
                                    MoveKind::Rapid,
                                    None,
                                    &mut report,
                                )?;
                            }
                            z = to;
                        }
                    }
                    Some(_) => return Err(format!("{id}: peck_mm must be positive")),
                    None => {
                        planner.emit(
                            &geom,
                            &id,
                            [*depth_z_mm, 0.],
                            MoveKind::Plunge,
                            feed,
                            &mut report,
                        )?;
                        planner.emit(
                            &geom,
                            &id,
                            [z_start, 0.],
                            MoveKind::Rapid,
                            None,
                            &mut report,
                        )?;
                    }
                }
                report.passes += 1;
            }
            TurnOperation::PartOff {
                z_mm,
                peck_mm,
                feed_mm_rev,
                ..
            } => {
                let feed = feed_of(*feed_mm_rev)?;
                if geom.blade == 0. || geom.inner {
                    return Err(format!(
                        "{id}: parting off needs an outside grooving insert"
                    ));
                }
                // The blade's +z side on the part's back face.
                let u = if geom.blade > 0. {
                    z_mm - rn - geom.blade
                } else {
                    z_mm - rn
                };
                approach(&mut *planner, &mut report, [u, safe_r])?;
                plunge(
                    &mut *planner,
                    &geom,
                    &id,
                    u,
                    safe_r,
                    0.,
                    *peck_mm,
                    feed,
                    &mut report,
                )?;
                planner.emit(&geom, &id, [u, safe_r], MoveKind::Rapid, None, &mut report)?;
                report.passes += 1;
            }
            TurnOperation::Finish {
                allowance_mm,
                feed_mm_rev,
                compensation,
                ..
            } => {
                let feed = feed_of(*feed_mm_rev)?;
                let path = job.part.offset(rn + allowance_mm)?;
                let first = path[0].start();
                approach(&mut *planner, &mut report, [z_start, first[1]])?;
                // Wear compensation: on over the straight entry, the tool on
                // the right of the path outside (G42, cutting toward the
                // chuck), off over the retract.
                let wear = *compensation == crate::contour::Compensation::Wear;
                if wear {
                    planner.events.push(Event::CutterCompensation {
                        side: CompensationSide::Right,
                        plane: crate::motion::Plane::Xz,
                    });
                }
                planner.emit(&geom, &id, first, MoveKind::Plunge, feed, &mut report)?;
                // Along the path to the clearance in front of the chuck. An
                // arc of the path (a fillet, a ball end, the nose rolling
                // round a corner) wholly in front of it is one arc move
                // (G2/G3, #14); any other as its chords.
                let limit = job.chuck_z_mm + job.clearance_mm;
                'path: for seg in &path {
                    if let Seg::Arc { a, b, c, sweep } = *seg
                        && native_arc(&geom, seg, tol / 4.)
                        && seg_low(seg, 0) >= limit
                    {
                        planner.emit(&geom, &id, a, MoveKind::Cut, feed, &mut report)?;
                        planner.emit_arc(
                            &geom,
                            &id,
                            c,
                            sweep,
                            b,
                            MoveKind::Cut,
                            feed,
                            &mut report,
                        )?;
                        continue;
                    }
                    for p in chain(std::slice::from_ref(seg), tol / 4.) {
                        if p == planner.position {
                            continue;
                        }
                        if p[0] < limit {
                            break 'path;
                        }
                        planner.emit(&geom, &id, p, MoveKind::Cut, feed, &mut report)?;
                    }
                }
                let pos = planner.position;
                if wear {
                    planner.events.push(Event::CutterCompensation {
                        side: CompensationSide::Off,
                        plane: crate::motion::Plane::Xz,
                    });
                }
                planner.emit(
                    &geom,
                    &id,
                    [pos[0], pos[1] + job.clearance_mm],
                    MoveKind::Retract,
                    feed,
                    &mut report,
                )?;
                report.passes += 1;
            }
        }
        reports.push(report);
        self.active = active;
        self.feed_rev = feed_rev;
        self.coolant = coolant;
        if next_tool.is_some_and(|t| t != op.tool()) {
            self.put_away(op.id())?;
        }
        Ok(())
    }

    /// Spindle and coolant off, the active tool back to the start position
    /// (rapids, outward first), ready for a tool change.
    fn put_away(&mut self, id: &str) -> Result<(), String> {
        let job = self.job;
        let Some(active) = self.active.take() else {
            return Ok(());
        };
        let planner = &mut self.planner;
        planner.events.push(Event::Spindle {
            rpm: 0.,
            direction: SpindleDirection::Cw,
        });
        planner.spindle = Spin::Rpm(0.);
        // Coolant off for the change, on again with the new tool (the change
        // itself stops it on many lathes, e.g. CNC12's).
        if self.coolant {
            planner.events.push(Event::Coolant { mode: Coolant::Off });
            self.coolant = false;
        }
        let old = job
            .tools
            .iter()
            .find(|t| t.tool.id == active)
            .ok_or("active tool missing")?;
        let geom = planner.geometry(&old.tool)?;
        // The old tool's controlled point to the start position.
        let home = [
            job.initial_position_mm[1] - geom.tip[0],
            job.initial_position_mm[0] - geom.tip[1],
        ];
        let mut tmp = self
            .reports
            .last()
            .cloned()
            .ok_or("no operation to attribute")?;
        let safe = [planner.position[0], home[1].max(planner.position[1])];
        planner.emit(&geom, id, safe, MoveKind::Rapid, None, &mut tmp)?;
        planner.emit(&geom, id, home, MoveKind::Rapid, None, &mut tmp)?;
        Ok(())
    }

    /// After the last operation: back to the start position, coolant and
    /// spindle off (attributed to the last operation).
    pub fn finish(&mut self) -> Result<(), String> {
        // Home and stop.
        let last = self
            .reports
            .last()
            .map(|r| r.id.clone())
            .unwrap_or_default();
        let planner = &mut self.planner;
        let job = self.job;
        if let Some(active) = &self.active {
            let t = &job
                .tools
                .iter()
                .find(|t| &t.tool.id == active)
                .unwrap()
                .tool;
            let geom = planner.geometry(t)?;
            let home = [
                job.initial_position_mm[1] - geom.tip[0],
                job.initial_position_mm[0] - geom.tip[1],
            ];
            let mut tmp = self.reports.last().cloned().unwrap();
            let pos = planner.position;
            planner.emit(
                &geom,
                &last,
                [pos[0], home[1].max(pos[1])],
                MoveKind::Rapid,
                None,
                &mut tmp,
            )?;
            planner.emit(&geom, &last, home, MoveKind::Rapid, None, &mut tmp)?;
        }
        planner.events.push(Event::Coolant { mode: Coolant::Off });
        planner.events.push(Event::Spindle {
            rpm: 0.,
            direction: SpindleDirection::Cw,
        });

        Ok(())
    }

    /// Events (motions included) planned since the last call.
    pub fn take_events(&mut self) -> Vec<Event> {
        std::mem::take(&mut self.planner.events)
    }

    /// The stock change of every move planned since the last call, in order.
    pub fn take_deltas(&mut self) -> Vec<TurnDelta> {
        std::mem::take(&mut self.planner.deltas)
    }

    pub fn stock(&self) -> &TurnStock {
        &self.planner.stock
    }

    pub fn reports(&self) -> &[TurnReport] {
        &self.reports
    }

    pub fn tools(&self) -> Vec<ToolAssembly> {
        self.job.tools.iter().map(|t| proxy(&t.tool)).collect()
    }
}

/// The part and bore a thread operation's moves are checked against: the
/// thread is cut into the profile's diameter over its length, and past its
/// end the leading flank reaches depth x tan(half angle) plus the nose
/// radius (the run-out, which must be free of shoulders); inside, its roots
/// open the bore out to the major radius. The planner and the replay of a
/// posted program use this one definition, so they agree on every thread
/// move (#100). Other operations: the part and bore unchanged.
fn thread_relief(
    op: &TurnOperation,
    tool: &TurningTool,
    internal: bool,
    part: &Envelope,
    bore: Option<&Envelope>,
) -> Result<(Envelope, Option<Envelope>), String> {
    let TurnOperation::Thread {
        id,
        z_start_mm,
        z_end_mm,
        major_radius_mm,
        depth_mm,
        ..
    } = op
    else {
        return Ok((part.clone(), bore.cloned()));
    };
    let half = match tool.insert {
        super::tool::Insert::Thread {
            profile_angle_deg, ..
        } => profile_angle_deg / 2.,
        _ => 0.,
    };
    let runout = depth_mm * half.to_radians().tan() + tool.nose_radius_mm;
    if internal {
        let b = bore.ok_or_else(|| {
            format!("{id}: an internal thread needs the part's bore (TurningJob.bore)")
        })?;
        Ok((
            part.clone(),
            Some(b.raised(z_end_mm - runout, *z_start_mm, *major_radius_mm)),
        ))
    } else {
        Ok((
            part.carved(z_end_mm - runout, *z_start_mm, major_radius_mm - depth_mm),
            bore.cloned(),
        ))
    }
}

/// Replays given lathe moves (a posted program read back) on the setup's
/// stock with the planner's own move kernel: each move is checked exactly as
/// a planned one (insert against the part, bore and chuck, holder and rapids
/// against the stock) and yields the stock change it makes.
pub struct TurnReplay {
    job: TurningJob,
    part: Option<Envelope>,
    bore: Option<Envelope>,
    stock: Option<TurnStock>,
    last_cut: Option<([f64; 2], [f64; 2])>,
    last_thread: Option<[f64; 2]>,
}

impl TurnReplay {
    pub fn new(job: TurningJob) -> Result<Self, String> {
        // The same setup validation as planning.
        TurnSession::new(&job, &std::sync::atomic::AtomicBool::new(false))?;
        let tol = job.tolerance_mm;
        Ok(Self {
            part: Some(job.part.envelope(tol / 4.)?),
            bore: job
                .bore
                .as_ref()
                .map(|b| b.envelope(tol / 4.))
                .transpose()?,
            stock: Some(TurnStock::new(job.stock.clone())?),
            job,
            last_cut: None,
            last_thread: None,
        })
    }

    /// Continue from `stock` (the stock at the next move's start).
    pub fn with_stock(mut self, stock: TurnStock) -> Result<Self, String> {
        if stock.spec() != &self.job.stock {
            return Err("replay stock differs from the setup's".into());
        }
        self.stock = Some(stock);
        Ok(self)
    }

    /// Check one move (a line or an arc in the XZ plane, in program
    /// coordinates) and return its stock change; the stock advances only
    /// when it passes.
    pub fn motion(&mut self, motion: &Motion) -> Result<TurnDelta, String> {
        let (start, end, arc) = match motion.curve {
            Curve::Line { start, end } => (start, end, None),
            Curve::Arc {
                start,
                end,
                center,
                plane: Plane::Xz,
                sweep_rad,
            } => (start, end, Some((center, sweep_rad))),
            _ => {
                return Err("lathe moves are replayed as lines and arcs in the XZ plane".into());
            }
        };
        if start[1] != 0. || end[1] != 0. || arc.is_some_and(|(c, _)| c[1] != 0.) {
            return Err("lathe moves lie in the XZ plane (y 0)".into());
        }
        let setting = self
            .job
            .tools
            .iter()
            .find(|t| t.tool.id == motion.tool)
            .ok_or_else(|| {
                format!(
                    "{}: tool {:?} is not in the setup",
                    motion.operation, motion.tool
                )
            })?;
        let never = std::sync::atomic::AtomicBool::new(false);
        let mut planner = Planner {
            job: &self.job,
            cancel: &never,
            part: self.part.take().ok_or("replay state lost")?,
            bore: self.bore.take(),
            stock: self.stock.take().ok_or("replay state lost")?,
            events: vec![],
            deltas: vec![],
            position: [0., 0.],
            next_id: motion.id,
            guard: self.job.tolerance_mm,
            last_cut: self.last_cut,
            last_thread: self.last_thread,
            spindle: Spin::Rpm(0.),
        };
        // A thread operation's moves are checked against the part with its
        // thread cut, exactly as the planner checked them.
        let operation = self
            .job
            .operations
            .iter()
            .find(|o| o.id() == motion.operation);
        let plain = (planner.part.clone(), planner.bore.clone());
        let result = (|| {
            let geom = planner.geometry(&setting.tool)?;
            if let Some(op) = operation {
                let (part, bore) = thread_relief(
                    op,
                    &setting.tool,
                    geom.inner,
                    &planner.part,
                    planner.bore.as_ref(),
                )?;
                planner.part = part;
                planner.bore = bore;
            }
            let t = geom.tip;
            // Program point to nose center: (u, v) = (z, r) less the tip.
            planner.position = [start[2] - t[0], start[0] - t[1]];
            let to = [end[2] - t[0], end[0] - t[1]];
            let feed = (motion.kind != MoveKind::Rapid).then_some(1.);
            let mut report = TurnReport {
                id: motion.operation.clone(),
                passes: 0,
                moves: 0,
                cut_length_mm: 0.,
                min_part_clearance_mm: f64::INFINITY,
                min_holder_clearance_mm: f64::INFINITY,
            };
            // Program point to nose center: a translation, the arc's center
            // with it.
            let arc = arc.map(|(c, sweep)| ([c[2] - t[0], c[0] - t[1]], sweep));
            planner.travel(
                &geom,
                &motion.operation,
                arc,
                to,
                motion.kind,
                feed,
                &mut report,
            )
        })();
        let delta = planner.deltas.pop();
        // The plain part and bore for the next move (the relief belongs to
        // this move's operation only).
        self.part = Some(plain.0);
        self.bore = plain.1;
        self.stock = Some(planner.stock);
        self.last_cut = planner.last_cut;
        self.last_thread = planner.last_thread;
        result?;
        Ok(delta.unwrap_or(TurnDelta {
            motion_id: motion.id,
            changes: vec![],
            rings: vec![],
        }))
    }
}

/// The turned stock compared with the part (a lathe run's material check).
///
/// Gouges are excluded per move, exactly: every planned (and every replayed
/// posted) move's insert is checked against the part and its bore as it is
/// made, so no stock can be removed inside the part beyond the tolerance;
/// the report records that proof. Leftover material is compared per z bin of
/// the stock (its radius bounds against the part's radius range over the
/// bin, outside and in the bore), and over each declared face groove (the
/// rings removed against the groove; its bottom corners keep the blade's
/// nose radii by definition). Behind the part (toward the chuck) is not
/// compared.
pub fn material_report(job: &TurningJob, stock: &TurnStock) -> Result<serde_json::Value, String> {
    use serde_json::json;
    let tol = job.tolerance_mm;
    let part = job.part.envelope(tol / 4.)?;
    let bore = job
        .bore
        .as_ref()
        .map(|b| b.envelope(tol / 4.))
        .transpose()?;
    let [z_lo, z_hi] = job.part.z_range();
    let pi = std::f64::consts::PI;
    let (mut definite, mut possible) = (0usize, 0usize);
    let mut volume = [0f64; 2];
    let mut findings = vec![];
    for (i, c) in stock.cells().iter().enumerate() {
        let [z0, z1] = stock.bin(i);
        if z1 <= z_lo {
            continue;
        }
        let h = z1 - z0;
        let mut check = |side: &str,
                         actual: [f64; 2],
                         target: [f64; 2],
                         definite_excess: bool,
                         possible_excess: bool,
                         excess: [f64; 2]| {
            if !(definite_excess || possible_excess) {
                return;
            }
            if definite_excess {
                definite += 1;
                volume[0] += excess[0];
            } else {
                possible += 1;
            }
            volume[1] += excess[1];
            findings.push(json!({
                "cell_index": i,
                "z_mm": [z0, z1],
                "side": side,
                "actual_radius_mm": actual,
                "target_radius_mm": target,
                "residual": if definite_excess { "definite_excess" } else { "possible_excess" },
            }));
        };
        let actual = [c.outer.lower_mm, c.outer.upper_mm];
        // In front of the face the part has no material.
        let target = if z0 >= z_hi {
            [0., 0.]
        } else {
            part.radius_range(z0.max(z_lo), z1.min(z_hi))
                .unwrap_or([0., 0.])
        };
        let annulus = |outer: f64, inner: f64| pi * (outer.powi(2) - inner.powi(2)).max(0.) * h;
        check(
            "outer",
            actual,
            target,
            c.outer.lower_mm > target[1] + tol && c.outer.lower_mm > c.inner.upper_mm,
            c.outer.upper_mm > target[0] + tol && c.outer.upper_mm > c.inner.lower_mm,
            [
                annulus(c.outer.lower_mm, target[1]),
                annulus(c.outer.upper_mm, target[0]),
            ],
        );
        if let Some(bore) = &bore
            && let Some(hole) = bore.radius_range(z0.max(z_lo), z1.min(z_hi))
            && z0 < z_hi
        {
            let actual = [c.inner.lower_mm, c.inner.upper_mm];
            check(
                "bore",
                actual,
                hole,
                c.inner.upper_mm < hole[0] - tol,
                c.inner.lower_mm < hole[1] - tol,
                [
                    annulus(hole[0], c.inner.upper_mm),
                    annulus(hole[1], c.inner.lower_mm),
                ],
            );
        }
    }
    // Face grooves: the rings certainly removed against each groove.
    let rn = job
        .tools
        .iter()
        .filter(|t| matches!(t.tool.insert, super::tool::Insert::FaceGroove { .. }))
        .map(|t| t.tool.nose_radius_mm)
        .fold(0., f64::max);
    for (k, g) in job.face_grooves.iter().enumerate() {
        let rect = [g.bottom_z_mm, z_hi, g.inner_radius_mm, g.outer_radius_mm];
        let area = (rect[1] - rect[0]) * (rect[3] - rect[2]);
        let covered: f64 = stock
            .rings()
            .iter()
            .map(|r| {
                let dz = r[1].min(rect[1]) - r[0].max(rect[0]);
                let dr = r[3].min(rect[3]) - r[2].max(rect[2]);
                dz.max(0.) * dr.max(0.)
            })
            .sum();
        let left = (area - covered).max(0.);
        // The blade's nose radii stay in the two bottom corners (by the
        // groove's definition); the removed rings are rectangles, which
        // credit neither corner's rn x rn square. And the tolerance along
        // the walls and bottom.
        let allowed = 2. * rn * rn + tol * (2. * (rect[1] - rect[0]) + (rect[3] - rect[2]));
        if left > allowed {
            definite += 1;
            let mean_r = (g.inner_radius_mm + g.outer_radius_mm) / 2.;
            volume[0] += 2. * pi * mean_r * (left - allowed);
            volume[1] += 2. * pi * g.outer_radius_mm * left;
            findings.push(json!({
                "face_groove": k,
                "z_mm": [rect[0], rect[1]],
                "side": "face_groove",
                "uncut_area_mm2": left,
                "allowed_area_mm2": allowed,
                "residual": "definite_excess",
            }));
        }
    }
    let assessment = if definite > 0 {
        "definite_excess"
    } else if possible > 0 {
        "possible_excess"
    } else {
        "within_tolerance"
    };
    Ok(json!({
        "algorithm": "turn-radius-bounds-v1",
        "comparison_tolerance_mm": tol,
        "complete": true,
        "residual": {
            "assessment": assessment,
            "definite_cells": definite,
            "possible_only_cells": possible,
            "volume_mm3": volume,
        },
        "gouge": {
            "assessment": "within_tolerance",
            "definite_cells": 0,
            "possible_only_cells": 0,
            "volume_mm3": [0., 0.],
            "proof": "every move's insert checked against the part and bore as it was made",
        },
        "findings": findings,
    }))
}

/// Plan a turning job: every operation, then home.
pub fn plan(job: &TurningJob, cancel: &dyn CancelSignal) -> Result<TurnPlan, String> {
    let mut session = TurnSession::new(job, cancel)?;
    for (i, op) in job.operations.iter().enumerate() {
        session.operation(op, job.operations.get(i + 1).map(|o| o.tool()))?;
    }
    session.finish()?;
    Ok(TurnPlan {
        tools: session.tools(),
        events: session.planner.events,
        reports: session.reports,
        stock: session.planner.stock,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn plan(job: &TurningJob) -> Result<TurnPlan, String> {
        super::plan(job, &std::sync::atomic::AtomicBool::new(false))
    }

    #[test]
    fn recorded_move_changes_replay_to_the_final_stock() {
        for j in [job(), bore_job()] {
            let never = std::sync::atomic::AtomicBool::new(false);
            let mut session = TurnSession::new(&j, &never).unwrap();
            let mut replay = TurnStock::new(j.stock.clone()).unwrap();
            let mut motions = 0;
            for (i, op) in j.operations.iter().enumerate() {
                session
                    .operation(op, j.operations.get(i + 1).map(|o| o.tool()))
                    .unwrap();
                let events = session.take_events();
                let deltas = session.take_deltas();
                let ids: Vec<u64> = events
                    .iter()
                    .filter_map(|e| match e {
                        Event::Motion { motion } => Some(motion.id),
                        _ => None,
                    })
                    .collect();
                assert_eq!(ids, deltas.iter().map(|d| d.motion_id).collect::<Vec<_>>());
                for d in &deltas {
                    replay.apply_move(&d.changes, &d.rings).unwrap();
                }
                motions += ids.len();
                // Every move of an operation is attributed to it.
                assert!(events.iter().all(|e| match e {
                    Event::Motion { motion } => motion.operation == op.id(),
                    _ => true,
                }));
            }
            session.finish().unwrap();
            for d in session.take_deltas() {
                replay.apply_move(&d.changes, &d.rings).unwrap();
            }
            assert!(motions > 0);
            assert_eq!(replay.cells(), session.stock().cells());
            assert_eq!(replay.rings(), session.stock().rings());
            assert_eq!(replay.cells(), plan(&j).unwrap().stock.cells());
        }
    }

    fn lathe_manifest(j: &TurningJob) -> crate::stream::StreamManifest {
        use crate::stream::{RunIdentity, RunStockSpec, StreamManifest};
        let setup = TurnSetup {
            part: j.part.clone(),
            bore: j.bore.clone(),
            chuck_z_mm: j.chuck_z_mm,
            tools: j.tools.clone(),
            operations: j.operations.clone(),
            clearance_mm: j.clearance_mm,
            tolerance_mm: j.tolerance_mm,
            feed_per_rev: j.feed_per_rev,
            face_grooves: j.face_grooves.clone(),
            controlled_point: j.controlled_point,
        };
        let manifest = StreamManifest {
            initial_stock: None,
            version: 1,
            run: RunIdentity {
                id: "lathe".into(),
                revision: "1".into(),
            },
            stock: RunStockSpec::Turn(j.stock.clone()),
            tools: setup.proxy_tools(),
            initial_position_mm: [j.initial_position_mm[0], 0., j.initial_position_mm[1]],
            sweep: crate::stock::SweepOptions::default(),
            operations: vec![],
            verification: None,
            tool_change: None,
            rapid_retracts: false,
            turning: Some(setup),
        };
        // The manifest's wire form reads back to the same manifest.
        let bytes = manifest.canonical_bytes().unwrap();
        let read: StreamManifest = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(read.canonical_bytes().unwrap(), bytes);
        manifest
    }

    #[test]
    fn a_lathe_run_streams_every_move_with_its_stock_change() {
        use crate::stream::{
            CHECKPOINT_FRAME_LIMIT, DEFAULT_FRAME_LIMIT, StreamProducer, StreamReceiver,
        };
        let kind = |f: &[u8]| u16::from_le_bytes([f[10], f[11]]);
        for j in [job(), bore_job(), groove_job()] {
            let manifest = lathe_manifest(&j);
            let never = std::sync::atomic::AtomicBool::new(false);
            let mut producer = StreamProducer::new(manifest.clone()).unwrap();
            let mut frames = vec![];
            crate::regeneration::run_turning(&mut producer, &never, |f| {
                frames.push(f.to_vec());
                Ok(())
            })
            .unwrap();
            let batch = plan(&j).unwrap();
            let moves = batch
                .events
                .iter()
                .filter(|e| matches!(e, Event::Motion { .. }))
                .count();
            // An independent receiver replays the frames.
            let mut receiver = StreamReceiver::new(manifest.clone()).unwrap();
            let (mut seen, mut checkpoint) = (0, None);
            let mut operations = vec![];
            for f in &frames {
                match kind(f) {
                    10 => {
                        receiver.accept(f, DEFAULT_FRAME_LIMIT).unwrap();
                        seen += 1;
                        if seen == moves / 2 {
                            checkpoint =
                                Some((seen, receiver.checkpoint(CHECKPOINT_FRAME_LIMIT).unwrap()));
                        }
                    }
                    7 => {
                        let control = crate::stream::decode_operation_control(
                            &manifest,
                            f,
                            DEFAULT_FRAME_LIMIT,
                        )
                        .unwrap();
                        if control["stage"] == "begin" {
                            operations.push(control["operation"].as_str().unwrap().to_string());
                        }
                    }
                    4 => receiver.accept_end(f, DEFAULT_FRAME_LIMIT).unwrap(),
                    other => panic!("unexpected frame kind {other}"),
                }
            }
            assert!(receiver.is_finished());
            assert_eq!(seen, moves);
            assert_eq!(
                operations,
                j.operations
                    .iter()
                    .map(|o| o.id().to_string())
                    .collect::<Vec<_>>()
            );
            let streamed = receiver.run_stock().turn().unwrap();
            assert_eq!(streamed.cells(), batch.stock.cells());
            assert_eq!(streamed.rings(), batch.stock.rings());
            // A checkpoint restores mid-run and replays the rest identically.
            let (at, bytes) = checkpoint.unwrap();
            let mut restored =
                StreamReceiver::from_checkpoint(manifest.clone(), &bytes, CHECKPOINT_FRAME_LIMIT)
                    .unwrap();
            assert_eq!(restored.completed(), at as u64);
            for f in frames.iter().filter(|f| kind(f) == 10).skip(at) {
                restored.accept(f, DEFAULT_FRAME_LIMIT).unwrap();
            }
            assert_eq!(
                restored.run_stock().turn().unwrap().cells(),
                batch.stock.cells()
            );
            // A frame applied twice is refused.
            let first = frames.iter().find(|f| kind(f) == 10).unwrap();
            assert!(restored.accept(first, DEFAULT_FRAME_LIMIT).is_err());
            // Cancelled mid-run: an error, and nothing after it.
            let polls = std::cell::Cell::new(0);
            let cancel = || {
                polls.set(polls.get() + 1);
                polls.get() > 40
            };
            let mut producer = StreamProducer::new(manifest).unwrap();
            let mut sent = 0;
            assert!(
                crate::regeneration::run_turning(&mut producer, &cancel, |_| {
                    sent += 1;
                    Ok(())
                })
                .is_err()
            );
            assert!(sent < frames.len());
        }
    }

    /// Replays every planned move (as a posted program read back would be)
    /// and checks the replay accepts it and ends with the planned stock;
    /// then replays the plan again up to its first thread pass and offers
    /// that pass `deeper` mm deeper toward the part, which must be refused.
    fn replay_threads(j: &TurningJob, batch: &TurnPlan, deeper: f64) {
        let mut replay = TurnReplay::new(j.clone()).unwrap();
        let mut stock = TurnStock::new(j.stock.clone()).unwrap();
        for e in &batch.events {
            if let Event::Motion { motion } = e {
                let d = replay.motion(motion).unwrap();
                stock.apply_move(&d.changes, &d.rings).unwrap();
            }
        }
        assert_eq!(stock.cells(), batch.stock.cells());
        let mut replay = TurnReplay::new(j.clone()).unwrap();
        for e in &batch.events {
            if let Event::Motion { motion } = e {
                if motion.kind == MoveKind::Thread {
                    let mut deep = motion.clone();
                    if let Curve::Line { start, end } = &mut deep.curve {
                        start[0] -= deeper;
                        end[0] -= deeper;
                    }
                    assert!(replay.motion(&deep).unwrap_err().contains("into the part"));
                    return;
                }
                replay.motion(motion).unwrap();
            }
        }
        panic!("no thread pass");
    }

    #[test]
    fn replayed_moves_remove_the_planned_stock() {
        for j in [job(), bore_job(), groove_job()] {
            let batch = plan(&j).unwrap();
            let mut replay = TurnReplay::new(j.clone()).unwrap();
            let mut stock = TurnStock::new(j.stock.clone()).unwrap();
            for e in &batch.events {
                if let Event::Motion { motion } = e {
                    let d = replay.motion(motion).unwrap();
                    stock.apply_move(&d.changes, &d.rings).unwrap();
                }
            }
            assert_eq!(stock.cells(), batch.stock.cells());
            assert_eq!(stock.rings(), batch.stock.rings());
        }
        // A move into the part is refused, and the stock is unchanged.
        let j = job();
        let mut replay = TurnReplay::new(j.clone()).unwrap();
        let tool = j.tools[0].tool.id.clone();
        let into = Motion {
            id: 1,
            operation: "finish".into(),
            tool,
            curve: Curve::Line {
                start: [12., 0., 1.],
                end: [5., 0., -15.],
            },
            kind: MoveKind::Cut,
            feed_mm_min: Some(100.),
        };
        assert!(replay.motion(&into).unwrap_err().contains("into the part"));
        assert_eq!(
            replay.stock.as_ref().unwrap().cells(),
            TurnStock::new(j.stock.clone()).unwrap().cells()
        );
    }

    #[test]
    fn material_report_finds_leftover_material_and_proves_no_gouge() {
        let j = job();
        let finished = plan(&j).unwrap();
        let report = material_report(&j, &finished.stock).unwrap();
        assert_eq!(report["gouge"]["definite_cells"], 0);
        assert_eq!(report["complete"], true);
        // Only the sharp inside corner at the shoulder (z -20) keeps material
        // the 0.8 mm nose cannot reach.
        for f in report["findings"].as_array().unwrap() {
            if f["residual"] == "definite_excess" {
                let z = f["z_mm"].as_array().unwrap();
                assert!(
                    z[0].as_f64().unwrap() >= -20. && z[1].as_f64().unwrap() <= -18.9,
                    "{f}"
                );
            }
        }
        // The bare stock: everything beyond the part is leftover material.
        let bare = TurnStock::new(j.stock.clone()).unwrap();
        let report = material_report(&j, &bare).unwrap();
        assert_eq!(report["residual"]["assessment"], "definite_excess");
        let all = report["residual"]["definite_cells"].as_u64().unwrap();
        let rough = j.operations[..2].to_vec();
        let mut roughed = j.clone();
        roughed.operations = rough;
        let report = material_report(&j, &plan(&roughed).unwrap().stock).unwrap();
        let left = report["residual"]["definite_cells"].as_u64().unwrap();
        assert!(0 < left && left < all, "{left} {all}");
    }

    #[test]
    fn cancellation_returns_no_plan() {
        let j = job();
        assert_eq!(super::plan(&j, &|| true).err().as_deref(), Some(CANCELLED));
        // Cancelled mid-plan, after some moves: still no result.
        let polls = std::cell::Cell::new(0);
        let after = || {
            polls.set(polls.get() + 1);
            polls.get() > 20
        };
        assert_eq!(super::plan(&j, &after).err().as_deref(), Some(CANCELLED));
        assert!(polls.get() > 20);
        assert!(plan(&j).is_ok());
    }
    use crate::{
        motion::{Program, ProgramValidator, WorkFrame},
        turning::tool::{Holder, Insert},
    };

    /// A tool with camlib's rectangular holder (`Holder::rectangle`).
    #[allow(clippy::too_many_arguments)]
    fn test_tool(
        id: String,
        insert: Insert,
        nose_radius_mm: f64,
        side: TurnSide,
        hand: Hand,
        width: f64,
        length: f64,
        offset: f64,
    ) -> TurningTool {
        let holder =
            Holder::rectangle(&insert, nose_radius_mm, side, width, length, offset).unwrap();
        let body = Some(test_body(&insert, side));
        TurningTool {
            id,
            insert,
            nose_radius_mm,
            side,
            hand,
            holder,
            body,
        }
    }
    /// The test tools' body, physically plausible for the tests' parts:
    /// outside, a 3 mm positive insert (7 degrees clearance) on a flat seat,
    /// the holder 10 mm below the face to 2 mm above; inside a bore, a
    /// 1.5 mm insert (11 degrees), the bar 1.5 mm below to 1 mm above; a face
    /// grooving blade curved for the tests' 8..14 mm grooves.
    fn test_body(insert: &Insert, side: TurnSide) -> crate::turning::body::ToolBody {
        use crate::turning::body::{InsertBody, ToolBody};
        let edge = match insert {
            Insert::Outline { .. } => 5.,
            Insert::Groove { .. } | Insert::Thread { .. } => 90.,
            Insert::FaceGroove { .. } => 0.,
            Insert::Drill { .. } => 0.,
        };
        let inner = side == TurnSide::Inner;
        ToolBody {
            insert: (!matches!(insert, Insert::Drill { .. })).then_some(InsertBody {
                insert_thickness_mm: if inner { 1.5 } else { 3. },
                insert_clearance_deg: if inner { 11. } else { 7. },
                rake_deg: 0.,
                inclination_deg: 0.,
                edge_deg: edge,
                curved_for_diameter_mm: matches!(insert, Insert::FaceGroove { .. })
                    .then_some([8., 14.]),
                blade_clearance: None,
            }),
            holder_y_mm: if inner { [-1.5, 1.] } else { [-10., 2.] },
            bar_axis_y_mm: 0.,
        }
    }
    fn rebuild_holder(tool: &mut TurningTool, width: f64, length: f64, offset: f64) {
        tool.holder = Holder::rectangle(
            &tool.insert,
            tool.nose_radius_mm,
            tool.side,
            width,
            length,
            offset,
        )
        .unwrap();
    }

    fn job() -> TurningJob {
        let tool = test_tool(
            "cnmg".into(),
            Insert::iso_turning(80., 95., 12., 0.8).unwrap(),
            0.8,
            TurnSide::Outer,
            Hand::Right,
            20.,
            100.,
            0.,
        );
        TurningJob {
            id: "shaft".into(),
            stock: TurnStockSpec {
                version: 1,
                units: "mm".into(),
                z_min_mm: -60.,
                z_max_mm: 2.,
                outer_radius_mm: 20.,
                bore_radius_mm: 0.,
                cell_mm: 0.25,
                max_cells: 10_000,
            },
            part: TurnProfile {
                points: vec![
                    [0., 0., 0.],
                    [10., 0., 0.],
                    [10., -20., 0.],
                    [15., -20., 0.],
                    [15., -40., 0.],
                ],
            },
            bore: None,
            chuck_z_mm: -50.,
            tools: vec![TurnToolSetting {
                tool,
                number: 1,
                offset: 1,
            }],
            operations: vec![
                TurnOperation::Face {
                    id: "face".into(),
                    tool: "cnmg".into(),
                    z_mm: 0.,
                    stepdown_mm: 1.,
                    feed_mm_rev: 0.15,
                    rpm: 1200.,
                    surface_speed_m_min: None,
                },
                TurnOperation::Rough {
                    id: "rough".into(),
                    tool: "cnmg".into(),
                    depth_of_cut_mm: 2.,
                    allowance_mm: 0.3,
                    feed_mm_rev: 0.25,
                    rpm: 1200.,
                    surface_speed_m_min: None,
                    retract_mm: 0.5,
                },
                TurnOperation::Finish {
                    id: "finish".into(),
                    tool: "cnmg".into(),
                    allowance_mm: 0.,
                    feed_mm_rev: 0.1,
                    rpm: 1500.,
                    surface_speed_m_min: None,
                    compensation: Default::default(),
                },
            ],
            initial_position_mm: [30., 20.],
            clearance_mm: 2.,
            tolerance_mm: 1e-3,
            feed_per_rev: false,
            controlled_point: ControlledPoint::Tip,
            face_grooves: vec![],
        }
    }

    #[test]
    fn face_rough_and_finish_a_shaft() {
        let j = job();
        let plan = plan(&j).unwrap();
        assert_eq!(plan.reports.len(), 3);
        assert_eq!(plan.reports[0].passes, 2); // 2 mm of face stock, 1 mm steps
        assert!(plan.reports[1].passes >= 5);
        // The finish leaves the part's size: diameters at r 10 and r 15
        // where the nose ran along them, never below.
        for (i, c) in plan.stock.cells().iter().enumerate() {
            let [z0, z1] = plan.stock.bin(i);
            if z0 >= -18.5 && z1 <= -1.5 {
                assert!(
                    (c.outer.upper_mm - 10.).abs() < 1e-6 && c.outer.lower_mm >= 10. - 1e-6,
                    "{z0} {c:?}"
                );
            }
            if z0 >= -38.5 && z1 <= -21.5 {
                assert!(
                    (c.outer.upper_mm - 15.).abs() < 1e-6 && c.outer.lower_mm >= 15. - 1e-6,
                    "{z0} {c:?}"
                );
            }
            if z0 >= 0. {
                assert_eq!(c.outer.upper_mm, 0., "faced {z0}");
            }
        }
        assert!(
            plan.reports
                .iter()
                .all(|r| r.min_part_clearance_mm >= -1e-3)
        );
        // A valid program for posting: the XZ plane, spindle before cuts.
        let program = Program {
            version: 1,
            id: "shaft".into(),
            revision: "1".into(),
            units: "mm".into(),
            tolerance_mm: 1e-5,
            frame: WorkFrame::identity(),
            initial_position_mm: [30., 0., 20.],
            tools: plan.tools.clone(),
            events: plan.events.clone(),
        };
        ProgramValidator::new(&program, None)
            .unwrap()
            .consume(&program.events)
            .unwrap();
    }

    #[test]
    fn roughing_leaves_the_allowance_on_every_diameter() {
        let mut j = job();
        j.operations.truncate(2); // face, rough
        let plan = plan(&j).unwrap();
        for (i, c) in plan.stock.cells().iter().enumerate() {
            let [z0, z1] = plan.stock.bin(i);
            for (lo, hi, r) in [(-18.5, -1.5, 10.), (-38.5, -21.5, 15.)] {
                if z0 >= lo && z1 <= hi {
                    assert!(
                        c.outer.upper_mm <= r + 0.3 + 1e-6 && c.outer.lower_mm >= r + 0.3 - 1e-6,
                        "{z0} {c:?}"
                    );
                }
            }
        }
    }

    fn groove_job() -> TurningJob {
        let mut j = job();
        j.part.points = vec![
            [0., 0., 0.],
            [10., 0., 0.],
            [10., -20., 0.],
            [15., -20., 0.],
            [15., -25., 0.],
            [12., -25., 0.],
            [12., -28., 0.],
            [15., -28., 0.],
            [15., -40., 0.],
        ];
        j.tools.push(TurnToolSetting {
            // Deeper than the bar's radius, so it can part off.
            tool: test_tool(
                "groove3".into(),
                Insert::Groove {
                    width_mm: 2.,
                    depth_mm: 22.,
                },
                0.2,
                TurnSide::Outer,
                Hand::Right,
                2.,
                60.,
                0.,
            ),
            number: 2,
            offset: 2,
        });
        j.operations.truncate(2); // face, rough
        j.operations.push(TurnOperation::Groove {
            id: "groove".into(),
            tool: "groove3".into(),
            z_from_mm: -25.,
            z_to_mm: -28.,
            bottom_radius_mm: 12.,
            peck_mm: Some(1.),
            overlap_mm: 0.2,
            feed_mm_rev: 0.05,
            rpm: 800.,
            surface_speed_m_min: None,
        });
        j.operations.push(TurnOperation::PartOff {
            id: "part_off".into(),
            tool: "groove3".into(),
            z_mm: -40.,
            peck_mm: None,
            feed_mm_rev: 0.05,
            rpm: 800.,
            surface_speed_m_min: None,
        });
        j
    }

    #[test]
    fn groove_and_part_off() {
        let j = groove_job();
        let plan = plan(&j).unwrap();
        assert_eq!(plan.reports[2].passes, 2); // a 3 mm groove, a 2 mm blade
        for (i, c) in plan.stock.cells().iter().enumerate() {
            let [z0, z1] = plan.stock.bin(i);
            if z0 >= -27.75 && z1 <= -25.25 {
                assert!((c.outer.upper_mm - 12.).abs() < 1e-6, "groove {z0} {c:?}");
            }
            if z0 >= -41.75 && z1 <= -40.25 {
                assert_eq!(c.outer.upper_mm, 0., "parted {z0}");
            }
        }
        // A groove the profile does not have would cut the part.
        let mut bad = j.clone();
        bad.operations[2] = TurnOperation::Groove {
            id: "groove".into(),
            tool: "groove3".into(),
            z_from_mm: -32.,
            z_to_mm: -35.,
            bottom_radius_mm: 12.,
            peck_mm: None,
            overlap_mm: 0.2,
            feed_mm_rev: 0.05,
            rpm: 800.,
            surface_speed_m_min: None,
        };
        assert!(plan_err(&bad).contains("into the part"));
    }

    #[test]
    fn single_point_threading() {
        let mut j = job();
        j.tools.push(TurnToolSetting {
            tool: test_tool(
                "thread60".into(),
                Insert::Thread {
                    profile_angle_deg: 60.,
                    depth_mm: 3.,
                },
                0.1,
                TurnSide::Outer,
                Hand::Right,
                16.,
                60.,
                0.,
            ),
            number: 3,
            offset: 3,
        });
        j.operations.push(TurnOperation::Thread {
            id: "thread".into(),
            tool: "thread60".into(),
            // From the shoulder: passes cannot leave a stretch unthreaded.
            z_start_mm: -20.,
            z_end_mm: -38.,
            major_radius_mm: 15.,
            pitch_mm: 2.,
            depth_mm: 1.227,
            passes: 6,
            infeed_angle_deg: 29.5,
            spring_passes: 1,
            rpm: 600.,
            surface_speed_m_min: None,
        });
        let plan = plan(&j).unwrap();
        assert_eq!(plan.reports[3].passes, 7);
        // The replay of the posted passes checks them against the part with
        // its thread cut, as the planner does (#100): accepted, the same
        // stock; a first pass 1 mm deeper (past the 1.227 mm depth) is not.
        replay_threads(&j, &plan, 1.);
        // Starting a millimetre into the diameter is refused (the passes
        // would cut the stretch before it).
        let mut early = j.clone();
        if let Some(TurnOperation::Thread { z_start_mm, .. }) = early.operations.last_mut() {
            *z_start_mm = -21.;
        }
        assert!(plan_err(&early).contains("into the part"));
        let threads: Vec<&Motion> = plan
            .events
            .iter()
            .filter_map(|e| match e {
                Event::Motion { motion } if motion.kind == MoveKind::Thread => Some(motion),
                _ => None,
            })
            .collect();
        assert_eq!(threads.len(), 7);
        assert!(threads.iter().all(|m| m.feed_mm_min == Some(1200.)));
        // The crests stay (upper bound 15); the solid bound drops to the root.
        for (i, c) in plan.stock.cells().iter().enumerate() {
            let [z0, z1] = plan.stock.bin(i);
            if z0 >= -37. && z1 <= -25. {
                assert!((c.outer.upper_mm - 15.).abs() < 1e-6, "{z0} {c:?}");
                assert!(
                    (c.outer.lower_mm - (15. - 1.227)).abs() < 1e-6,
                    "{z0} {c:?}"
                );
            }
        }
    }

    #[test]
    fn drilling_on_the_axis_inside_the_bore() {
        let mut j = job();
        // A 10 mm hole 20 mm deep with a 118 degree point.
        let cone = 5. / 59f64.to_radians().tan();
        j.bore = Some(TurnProfile {
            points: vec![[5., 0., 0.], [5., -20., 0.], [0., -20. - cone, 0.]],
        });
        j.tools.push(TurnToolSetting {
            tool: test_tool(
                "drill10".into(),
                Insert::Drill {
                    diameter_mm: 10.,
                    point_angle_deg: 118.,
                    flute_length_mm: 40.,
                },
                0.,
                TurnSide::Outer,
                Hand::Right,
                30.,
                60.,
                0.,
            ),
            number: 4,
            offset: 4,
        });
        j.operations.truncate(1); // face
        j.operations.push(TurnOperation::Drill {
            id: "drill".into(),
            tool: "drill10".into(),
            depth_z_mm: -20. - cone,
            peck_mm: Some(6.),
            feed_mm_rev: 0.12,
            rpm: 900.,
            surface_speed_m_min: None,
        });
        let plan = plan(&j).unwrap();
        for (i, c) in plan.stock.cells().iter().enumerate() {
            let [z0, z1] = plan.stock.bin(i);
            if z0 >= -19.5 && z1 <= -0.5 {
                assert!(
                    (c.inner.lower_mm - 5.).abs() < 1e-9 && (c.inner.upper_mm - 5.).abs() < 1e-9,
                    "{z0} {c:?}"
                );
            }
            if z1 <= -20. - cone {
                assert_eq!(c.inner.upper_mm, 0., "{z0}");
            }
        }
        // Deeper than the bore: the point would cut the part.
        let mut deep = j.clone();
        if let Some(TurnOperation::Drill { depth_z_mm, .. }) = deep.operations.last_mut() {
            *depth_z_mm = -26.;
        }
        assert!(plan_err(&deep).contains("into the part"));
        // No bore declared: refused, naming what is missing.
        let mut none = j.clone();
        none.bore = None;
        assert!(plan_err(&none).contains("TurningJob.bore"));
    }

    /// A tube (8 mm bore) to bore out to a stepped hole: r 11 to 20 deep,
    /// r 9 behind it to 35.
    fn bore_job() -> TurningJob {
        let mut j = job();
        j.stock.z_max_mm = 0.;
        j.stock.bore_radius_mm = 8.;
        j.part.points = vec![[0., 0., 0.], [18., 0., 0.], [18., -40., 0.]];
        j.bore = Some(TurnProfile {
            points: vec![
                [11., 0., 0.],
                [11., -20., 0.],
                [9., -20., 0.],
                [9., -35., 0.],
            ],
        });
        j.tools.push(TurnToolSetting {
            tool: // A 10 mm bar, its top 1 mm under the nose center.
test_tool("bar".into(), Insert::iso_turning(55., 95., 6., 0.4).unwrap(), 0.4, TurnSide::Inner, Hand::Right, 10., 80., 1.),
            number: 5,
            offset: 5,
        });
        j.operations = vec![
            TurnOperation::BoreRough {
                id: "bore_rough".into(),
                tool: "bar".into(),
                depth_of_cut_mm: 1.5,
                allowance_mm: 0.2,
                z_end_mm: -34.,
                feed_mm_rev: 0.15,
                rpm: 1000.,
                surface_speed_m_min: None,
                retract_mm: 0.5,
            },
            TurnOperation::BoreFinish {
                id: "bore_finish".into(),
                tool: "bar".into(),
                allowance_mm: 0.,
                z_end_mm: -34.,
                feed_mm_rev: 0.08,
                rpm: 1200.,
                surface_speed_m_min: None,
                compensation: Default::default(),
            },
        ];
        j
    }

    fn inner_radius_is(plan: &TurnPlan, lo: f64, hi: f64, r: f64) {
        for (i, c) in plan.stock.cells().iter().enumerate() {
            let [z0, z1] = plan.stock.bin(i);
            if z0 >= lo && z1 <= hi {
                assert!(
                    (c.inner.lower_mm - r).abs() < 1e-6 && (c.inner.upper_mm - r).abs() < 1e-6,
                    "{z0}: {c:?} not {r}"
                );
            }
        }
    }

    #[test]
    fn boring_roughs_and_finishes_a_stepped_bore() {
        let mut rough = bore_job();
        rough.operations.truncate(1);
        let plan = plan(&rough).unwrap();
        // Every diameter left at its allowance.
        inner_radius_is(&plan, -19., -1., 10.8);
        inner_radius_is(&plan, -33.5, -21., 8.8);
        assert!(plan.reports[0].passes >= 3);
        let plan = self::plan(&bore_job()).unwrap();
        inner_radius_is(&plan, -19., -1., 11.);
        inner_radius_is(&plan, -33.5, -21., 9.);
        assert!(
            plan.reports
                .iter()
                .all(|r| r.min_part_clearance_mm >= -1e-3)
        );
        // Too fat a bar for the hole: its far side hits the bore wall.
        let mut fat = bore_job();
        rebuild_holder(&mut fat.tools.last_mut().unwrap().tool, 17., 80., 1.);
        let err = plan_err(&fat);
        assert!(err.contains("holder"), "{err}");
        // Deeper than the bore: the insert would cut past its bottom.
        let mut deep = bore_job();
        if let Some(TurnOperation::BoreFinish { z_end_mm, .. }) = deep.operations.last_mut() {
            *z_end_mm = -36.;
        }
        assert!(plan_err(&deep).contains("into the part"));
        // A boring tool needs the bore and an open hole.
        let mut none = bore_job();
        none.bore = None;
        assert!(plan_err(&none).contains("TurningJob.bore"));
        let mut solid = bore_job();
        solid.stock.bore_radius_mm = 0.;
        assert!(plan_err(&solid).contains("no open hole"));
    }

    #[test]
    fn internal_threading_opens_the_roots_and_keeps_the_crests() {
        let mut j = bore_job();
        j.stock.bore_radius_mm = 11.;
        j.bore = Some(TurnProfile {
            points: vec![[11., 0., 0.], [11., -30., 0.]],
        });
        j.tools.push(TurnToolSetting {
            tool: test_tool(
                "thread_in".into(),
                Insert::Thread {
                    profile_angle_deg: 60.,
                    depth_mm: 2.,
                },
                0.1,
                TurnSide::Inner,
                Hand::Right,
                10.,
                80.,
                // The bar's top 2 mm under the nose: at the last pass (nose
                // at r 11.9) it stays clear of the 11 mm crests off the
                // centre plane too (#103).
                2.,
            ),
            number: 6,
            offset: 6,
        });
        // M24 x 1.5 internal, 20 mm long.
        j.operations = vec![TurnOperation::Thread {
            id: "thread".into(),
            tool: "thread_in".into(),
            z_start_mm: 0.,
            z_end_mm: -20.,
            major_radius_mm: 12.,
            pitch_mm: 1.5,
            depth_mm: 0.92,
            passes: 6,
            infeed_angle_deg: 29.5,
            spring_passes: 1,
            rpm: 500.,
            surface_speed_m_min: None,
        }];
        let plan = plan(&j).unwrap();
        assert_eq!(plan.reports[0].passes, 7);
        // Inside: deeper means outward, past the major radius (negative x
        // shift toward the axis would be shallower).
        replay_threads(&j, &plan, -1.);
        for (i, c) in plan.stock.cells().iter().enumerate() {
            let [z0, z1] = plan.stock.bin(i);
            if z0 >= -19. && z1 <= -1. {
                assert!((c.inner.lower_mm - 11.).abs() < 1e-6, "crests {z0} {c:?}");
                assert!((c.inner.upper_mm - 12.).abs() < 1e-6, "roots {z0} {c:?}");
            }
        }
        // Every pull-out goes toward the axis.
        let mut last = None;
        for e in &plan.events {
            if let Event::Motion { motion } = e {
                if last == Some(MoveKind::Thread) {
                    let c = &motion.curve;
                    assert!(c.end()[0] < c.start()[0]);
                }
                last = Some(motion.kind);
            }
        }
        // A thread deeper than the run-out allows hits the bore's bottom.
        let mut blind = j.clone();
        blind.bore = Some(TurnProfile {
            points: vec![[11., 0., 0.], [11., -20.2, 0.], [0., -20.2, 0.]],
        });
        assert!(plan_err(&blind).contains("into the part"));
    }

    #[test]
    fn constant_surface_speed_with_feed_per_revolution() {
        let mut j = job();
        j.feed_per_rev = true;
        for op in &mut j.operations {
            if let TurnOperation::Face {
                rpm,
                surface_speed_m_min,
                ..
            }
            | TurnOperation::Rough {
                rpm,
                surface_speed_m_min,
                ..
            }
            | TurnOperation::Finish {
                rpm,
                surface_speed_m_min,
                ..
            } = op
            {
                *rpm = 3000.;
                *surface_speed_m_min = Some(200.);
            }
        }
        let plan = plan(&j).unwrap();
        let program = Program {
            version: 1,
            id: "shaft".into(),
            revision: "1".into(),
            units: "mm".into(),
            tolerance_mm: 1e-5,
            frame: WorkFrame::identity(),
            initial_position_mm: [30., 0., 20.],
            tools: plan.tools.clone(),
            events: plan.events.clone(),
        };
        // The validator recomputes every feed bound from the curve.
        ProgramValidator::new(&program, None)
            .unwrap()
            .consume(&program.events)
            .unwrap();
        let css: Vec<_> = plan
            .events
            .iter()
            .filter(|e| matches!(e, Event::SurfaceSpeed { .. }))
            .collect();
        assert_eq!(css.len(), 1);
        let feeds: Vec<f64> = plan
            .events
            .iter()
            .filter_map(|e| match e {
                Event::FeedPerRev { mm_rev } => Some(*mm_rev),
                _ => None,
            })
            .collect();
        assert_eq!(feeds, vec![0.15, 0.25, 0.1]);
        // Facing reaches the axis: capped at 3000 rpm. A roughing pass at
        // nose radius v: 200 m/min at the nose center's least radius.
        let rpm_at = |r: f64| (200_000. / (std::f64::consts::TAU * r)).min(3000.);
        for e in &plan.events {
            if let Event::Motion { motion } = e
                && motion.kind == MoveKind::Cut
                && motion.operation == "rough"
            {
                let r = motion.curve.start()[0].min(motion.curve.end()[0]);
                let f = motion.feed_mm_min.unwrap();
                assert!((f - 0.25 * rpm_at(r)).abs() < 1e-9, "{r} {f}");
            }
        }
        // A feed that does not follow the spindle is refused.
        let mut events = plan.events.clone();
        for e in &mut events {
            if let Event::Motion { motion } = e
                && motion.kind == MoveKind::Cut
            {
                motion.feed_mm_min = Some(100.);
                break;
            }
        }
        let err = ProgramValidator::new(&program, None)
            .unwrap()
            .consume(&events)
            .err()
            .unwrap();
        assert!(err.contains("per-revolution"), "{err}");
        // CSS needs feeds per revolution.
        let mut per_minute = j.clone();
        per_minute.feed_per_rev = false;
        assert!(plan_err(&per_minute).contains("feed_per_rev"));
    }

    #[test]
    fn x_extent_of_lathe_arcs() {
        use crate::motion::{Plane, x_extent};
        // G18: from +Z toward +X. Center (x 10, z 0), radius 2, start at
        // +Z (x 10, z 2), a half turn through +X: x up to 12.
        let arc = Curve::Arc {
            start: [10., 0., 2.],
            end: [10., 0., -2.],
            center: [10., 0., 0.],
            sweep_rad: std::f64::consts::PI,
            plane: Plane::Xz,
        };
        let [lo, hi] = x_extent(&arc);
        assert!(
            (lo - 10.).abs() < 1e-12 && (hi - 12.).abs() < 1e-12,
            "{lo} {hi}"
        );
        let back = Curve::Arc {
            start: [10., 0., 2.],
            end: [10., 0., -2.],
            center: [10., 0., 0.],
            sweep_rad: -std::f64::consts::PI,
            plane: Plane::Xz,
        };
        let [lo, hi] = x_extent(&back);
        assert!(
            (lo - 8.).abs() < 1e-12 && (hi - 10.).abs() < 1e-12,
            "{lo} {hi}"
        );
    }

    #[test]
    fn internal_grooving_plunges_outward_from_the_open_hole() {
        let mut j = bore_job();
        // A 4 mm wide groove out to r 13 in the r 11 bore, 10 mm in.
        j.bore = Some(TurnProfile {
            points: vec![
                [11., 0., 0.],
                [11., -10., 0.],
                [13., -10., 0.],
                [13., -14., 0.],
                [11., -14., 0.],
                [11., -20., 0.],
                [9., -20., 0.],
                [9., -35., 0.],
            ],
        });
        j.tools.push(TurnToolSetting {
            tool: // A 10 mm bar, its top the blade's depth under the tip.
test_tool("groove_in".into(), Insert::Groove { width_mm: 3.,
                    depth_mm: 4. }, 0.2, TurnSide::Inner, Hand::Right, 10., 80., 4.),
            number: 7,
            offset: 7,
        });
        j.operations.truncate(1); // bore rough
        j.operations.push(TurnOperation::Groove {
            id: "groove".into(),
            tool: "groove_in".into(),
            z_from_mm: -10.,
            z_to_mm: -14.,
            bottom_radius_mm: 13.,
            peck_mm: Some(1.),
            overlap_mm: 0.2,
            feed_mm_rev: 0.04,
            rpm: 600.,
            surface_speed_m_min: None,
        });
        let plan = plan(&j).unwrap();
        assert_eq!(plan.reports[1].passes, 2);
        inner_radius_is(&plan, -13.75, -10.25, 13.);
        // Every plunge goes outward; the tool leaves along the axis.
        for e in &plan.events {
            if let Event::Motion { motion } = e
                && motion.kind == MoveKind::Plunge
                && motion.operation == "groove"
            {
                assert!(motion.curve.end()[0] > motion.curve.start()[0]);
            }
        }
        // A groove deeper than the bore's groove cuts the part.
        let mut deep = j.clone();
        if let Some(TurnOperation::Groove {
            bottom_radius_mm, ..
        }) = deep.operations.last_mut()
        {
            *bottom_radius_mm = 13.5;
        }
        assert!(plan_err(&deep).contains("into the part"));
        // The bar must clear the bore at the groove's bottom.
        let mut thick = j.clone();
        rebuild_holder(&mut thick.tools.last_mut().unwrap().tool, 10., 80., 1.);
        assert!(plan_err(&thick).contains("holder"));
    }

    /// The finish's axial cuts along a diameter, as posted: (x, z start, z end).
    fn finish_cuts(plan: &TurnPlan, op: &str) -> Vec<(f64, f64, f64)> {
        plan.events
            .iter()
            .filter_map(|e| match e {
                Event::Motion { motion }
                    if motion.operation == op
                        && motion.kind == MoveKind::Cut
                        && motion.curve.start()[0] == motion.curve.end()[0]
                        && (motion.curve.start()[2] - motion.curve.end()[2]).abs() > 1. =>
                {
                    Some((
                        motion.curve.start()[0],
                        motion.curve.start()[2],
                        motion.curve.end()[2],
                    ))
                }
                _ => None,
            })
            .collect()
    }

    #[test]
    fn programs_move_the_imaginary_tool_tip() {
        // Touched off on a diameter and a face: the finish along r 10 and
        // r 15 is posted at exactly those radii, starting at the face z.
        let plan = plan(&job()).unwrap();
        let cuts = finish_cuts(&plan, "finish");
        assert!(cuts.iter().any(|c| (c.0 - 10.).abs() < 1e-9), "{cuts:?}");
        assert!(cuts.iter().any(|c| (c.0 - 15.).abs() < 1e-9), "{cuts:?}");
        assert!(cuts.iter().all(|c| c.0 >= 10. - 1e-9));
        // A boring bar's tip is on the bore: r 11 and r 9.
        let plan = self::plan(&bore_job()).unwrap();
        let cuts = finish_cuts(&plan, "bore_finish");
        assert!(cuts.iter().any(|c| (c.0 - 11.).abs() < 1e-9), "{cuts:?}");
        assert!(cuts.iter().any(|c| (c.0 - 9.).abs() < 1e-9), "{cuts:?}");
        // Nose-center programs: the 0.8 mm nose center, 10.8 and 15.8.
        let mut j = job();
        j.controlled_point = ControlledPoint::NoseCenter;
        let plan = self::plan(&j).unwrap();
        let cuts = finish_cuts(&plan, "finish");
        assert!(cuts.iter().any(|c| (c.0 - 10.8).abs() < 1e-9), "{cuts:?}");
        // Both start and end at the start position: the controlled point.
        for plan in [self::plan(&job()).unwrap(), plan] {
            let motions: Vec<&Motion> = plan
                .events
                .iter()
                .filter_map(|e| match e {
                    Event::Motion { motion } => Some(motion),
                    _ => None,
                })
                .collect();
            assert_eq!(motions[0].curve.start(), [30., 0., 20.]);
            assert_eq!(motions.last().unwrap().curve.end(), [30., 0., 20.]);
        }
    }

    /// #170: a straight face grooving blade's outer side reaches out below
    /// the centre plane (√(r² + y²)), so the groove's outer wall needs that
    /// side's clearance, whatever the front's and the inner side's are.
    #[test]
    fn a_face_grooving_blade_is_checked_with_each_sides_clearance() {
        use crate::turning::body::BladeClearance;
        let job_with = |front: f64, sides: [f64; 2]| {
            let mut j = job();
            j.part.points = vec![[0., 0., 0.], [12., 0., 0.], [12., -40., 0.]];
            j.face_grooves = vec![FaceGroove {
                inner_radius_mm: 4.,
                outer_radius_mm: 7.,
                bottom_z_mm: -3.,
            }];
            let mut tool = test_tool(
                "face_groove".into(),
                Insert::FaceGroove {
                    width_mm: 2.,
                    depth_mm: 8.,
                },
                0.2,
                TurnSide::Outer,
                Hand::Right,
                2.,
                40.,
                0.,
            );
            // A straight blade 3 mm thick.
            let body = tool.body.as_mut().unwrap().insert.as_mut().unwrap();
            body.curved_for_diameter_mm = None;
            body.insert_clearance_deg = front.min(sides[0]).min(sides[1]);
            body.blade_clearance = Some(BladeClearance {
                front_deg: front,
                side_deg: sides,
            });
            j.tools.push(TurnToolSetting {
                tool,
                number: 8,
                offset: 8,
            });
            j.operations.truncate(1);
            j.operations.push(TurnOperation::FaceGroove {
                id: "fg".into(),
                tool: "face_groove".into(),
                inner_radius_mm: 4.,
                outer_radius_mm: 7.,
                bottom_z_mm: -3.,
                peck_mm: Some(1.),
                overlap_mm: 0.2,
                feed_mm_rev: 0.03,
                rpm: 600.,
                surface_speed_m_min: None,
            });
            j
        };
        // At the outer wall (r 7) a point 3 mm below the plane is clear of
        // it when its side has receded by 7 - √(7² - 3²) = 0.675 mm: at
        // least 12.7 degrees over 3 mm.
        let needed = ((7. - (49f64 - 9.).sqrt()) / 3.).atan().to_degrees();
        assert!(needed > 12.6 && needed < 12.8);
        // Outer side 15 degrees: planned, with no clearance at the front or
        // the inner side (they recede from their walls as they are).
        assert_eq!(plan(&job_with(0., [0., 15.])).unwrap().reports[1].passes, 2);
        // The same angles with the sides swapped: the outer side rubs.
        let e = plan_err(&job_with(0., [15., 0.]));
        assert!(
            e.contains("the blade's outer side flank, 0 degrees of clearance"),
            "{e}"
        );
        assert!(
            !e.contains("inner side") && !e.contains("front flank"),
            "{e}"
        );
        // A generous front does not help the side.
        let e = plan_err(&job_with(20., [15., 12.]));
        assert!(
            e.contains("the blade's outer side flank, 12 degrees of clearance"),
            "{e}"
        );
        assert!(plan(&job_with(20., [15., 13.])).is_ok());
    }

    #[test]
    fn face_grooving_plunges_axially_inside_the_declared_groove() {
        let mut j = job();
        // A 25 mm diameter shaft face with a ring groove r 4..7, 3 mm deep.
        j.part.points = vec![[0., 0., 0.], [12., 0., 0.], [12., -40., 0.]];
        j.face_grooves = vec![FaceGroove {
            inner_radius_mm: 4.,
            outer_radius_mm: 7.,
            bottom_z_mm: -3.,
        }];
        j.tools.push(TurnToolSetting {
            tool: test_tool(
                "face_groove".into(),
                Insert::FaceGroove {
                    width_mm: 2.,
                    depth_mm: 8.,
                },
                0.2,
                TurnSide::Outer,
                Hand::Right,
                2.,
                40.,
                0.,
            ),
            number: 8,
            offset: 8,
        });
        j.operations.truncate(1); // face
        j.operations.push(TurnOperation::FaceGroove {
            id: "fg".into(),
            tool: "face_groove".into(),
            inner_radius_mm: 4.,
            outer_radius_mm: 7.,
            bottom_z_mm: -3.,
            peck_mm: Some(1.),
            overlap_mm: 0.2,
            feed_mm_rev: 0.03,
            rpm: 600.,
            surface_speed_m_min: None,
        });
        let plan = plan(&j).unwrap();
        assert_eq!(plan.reports[1].passes, 2); // a 3 mm ring, a 2 mm blade
        // The material check finds the groove cut (only its corner radii left).
        let grooves = |stock: &TurnStock| {
            material_report(&j, stock).unwrap()["findings"]
                .as_array()
                .unwrap()
                .iter()
                .filter(|f| f["side"] == "face_groove")
                .count()
        };
        assert_eq!(grooves(&plan.stock), 0);
        // Without the grooving operation, the groove is reported uncut.
        let mut faced = j.clone();
        faced.operations.truncate(1);
        assert_eq!(
            grooves(
                &super::plan(&faced, &std::sync::atomic::AtomicBool::new(false))
                    .unwrap()
                    .stock
            ),
            1
        );
        // The ring is recorded removed: its volume no longer counts.
        let ring = std::f64::consts::PI * (7f64.powi(2) - 4f64.powi(2)) * 3.;
        let rings: f64 = plan
            .stock
            .rings()
            .iter()
            .map(|r| {
                std::f64::consts::PI * (r[3] * r[3] - r[2] * r[2]) * (r[1].min(0.) - r[0]).max(0.)
            })
            .sum();
        assert!(
            rings > 0.95 * ring && rings <= ring + 1e-6,
            "{rings} {ring}"
        );
        // Plunges are axial (constant radius) and go toward -z.
        for e in &plan.events {
            if let Event::Motion { motion } = e
                && motion.operation == "fg"
                && motion.kind == MoveKind::Plunge
            {
                let (a, b) = (motion.curve.start(), motion.curve.end());
                assert!(a[0] == b[0] && b[2] < a[2]);
            }
        }
        // Deeper than declared, or not declared: refused.
        let mut deep = j.clone();
        if let Some(TurnOperation::FaceGroove { bottom_z_mm, .. }) = deep.operations.last_mut() {
            *bottom_z_mm = -4.;
        }
        assert!(plan_err(&deep).contains("face_grooves"));
        let mut wide = j.clone();
        wide.face_grooves[0].outer_radius_mm = 7.5;
        assert!(plan_err(&wide).contains("face_grooves"));
        // A blade wider than the groove.
        let mut narrow = j.clone();
        narrow.face_grooves[0].outer_radius_mm = 5.;
        if let Some(TurnOperation::FaceGroove {
            outer_radius_mm, ..
        }) = narrow.operations.last_mut()
        {
            *outer_radius_mm = 5.;
        }
        assert!(plan_err(&narrow).contains("must fit"));
        // A face grooving insert in another operation is refused.
        let mut misuse = j.clone();
        misuse.operations[0] = TurnOperation::Face {
            id: "face".into(),
            tool: "face_groove".into(),
            z_mm: 0.,
            stepdown_mm: 0.4,
            feed_mm_rev: 0.1,
            rpm: 800.,
            surface_speed_m_min: None,
        };
        assert!(plan_err(&misuse).contains("face grooving"));
    }

    #[test]
    fn facing_deeper_than_the_nose_is_cut_by_the_edges() {
        let mut j = job();
        j.tools[0].tool = test_tool(
            "cnmg".into(),
            Insert::iso_turning(80., 95., 12., 0.4).unwrap(),
            0.4,
            TurnSide::Outer,
            Hand::Right,
            20.,
            100.,
            0.,
        );
        j.stock.z_max_mm = 3.;
        j.operations = vec![TurnOperation::Face {
            id: "face".into(),
            tool: "cnmg".into(),
            z_mm: 0.,
            stepdown_mm: 3.,
            feed_mm_rev: 0.15,
            rpm: 1200.,
            surface_speed_m_min: None,
        }];
        let plan = plan(&j).unwrap();
        assert_eq!(plan.reports[0].passes, 1);
        // The whole 3 mm is gone, not only the nose's 0.8 mm band.
        for (i, c) in plan.stock.cells().iter().enumerate() {
            if plan.stock.bin(i)[0] >= 0. {
                assert_eq!(c.outer.upper_mm, 0., "{i} {c:?}");
            }
        }
        // Deeper than the 12 mm cutting edge: what the edges cannot reach is
        // left, and the holder behind them would hit it.
        let mut deep = j.clone();
        deep.stock.z_max_mm = 15.;
        if let TurnOperation::Face { stepdown_mm, .. } = &mut deep.operations[0] {
            *stepdown_mm = 15.;
        }
        let err = plan_err(&deep);
        assert!(err.contains("holder") || err.contains("back"), "{err}");
    }

    #[test]
    fn wear_compensation_turns_on_over_the_entry_and_off_before_the_retract() {
        let mut j = job();
        if let TurnOperation::Finish { compensation, .. } = &mut j.operations[2] {
            *compensation = crate::contour::Compensation::Wear;
        }
        let plan = plan(&j).unwrap();
        let program = Program {
            version: 1,
            id: "shaft".into(),
            revision: "1".into(),
            units: "mm".into(),
            tolerance_mm: 1e-5,
            frame: WorkFrame::identity(),
            initial_position_mm: [30., 0., 20.],
            tools: plan.tools.clone(),
            events: plan.events.clone(),
        };
        // In-plane (XZ) motion while it is on: the validator accepts it.
        ProgramValidator::new(&program, None)
            .unwrap()
            .consume(&program.events)
            .unwrap();
        let sides: Vec<(usize, CompensationSide)> = plan
            .events
            .iter()
            .enumerate()
            .filter_map(|(i, e)| match e {
                Event::CutterCompensation { side, .. } => Some((i, *side)),
                _ => None,
            })
            .collect();
        assert_eq!(sides.len(), 2);
        assert_eq!(sides[0].1, CompensationSide::Right);
        assert_eq!(sides[1].1, CompensationSide::Off);
        let next = |i: usize| match &plan.events[i + 1] {
            Event::Motion { motion } => motion.kind,
            e => panic!("{e:?}"),
        };
        assert_eq!(next(sides[0].0), MoveKind::Plunge);
        assert_eq!(next(sides[1].0), MoveKind::Retract);
        // Inside: G41.
        let mut b = bore_job();
        if let Some(TurnOperation::BoreFinish { compensation, .. }) = b.operations.last_mut() {
            *compensation = crate::contour::Compensation::Wear;
        }
        let plan = self::plan(&b).unwrap();
        assert!(plan.events.iter().any(|e| matches!(
            e,
            Event::CutterCompensation {
                side: CompensationSide::Left,
                plane: crate::motion::Plane::Xz,
            }
        )));
    }

    fn plan_err(j: &TurningJob) -> String {
        plan(j).err().unwrap()
    }

    #[test]
    fn a_profile_the_insert_cannot_follow_is_refused() {
        let mut j = job();
        // A 60 degree back taper behind a shoulder: the 80 degree insert's
        // trailing edge would cut it.
        j.part.points = vec![
            [0., 0., 0.],
            [15., 0., 0.],
            [15., -10., 0.],
            [10., -13., 0.],
            [10., -40., 0.],
        ];
        j.operations.remove(1);
        j.operations.remove(0);
        let err = plan(&j).err().unwrap();
        assert!(err.contains("into the part"), "{err}");
    }

    /// The shaft job turned with `insert` (nose radius `rn`) on camlib's
    /// rectangular holder.
    fn shaft_with(insert: Insert, rn: f64) -> TurningJob {
        let mut j = job();
        j.tools[0].tool = test_tool(
            "cnmg".into(),
            insert,
            rn,
            TurnSide::Outer,
            Hand::Right,
            20.,
            100.,
            0.,
        );
        j
    }

    #[test]
    fn round_and_trigon_inserts_profile_the_shaft_with_clearance() {
        // A round insert of 8 mm: its 4 mm nose fits the 5 mm shoulder.
        for code in ["RCMT0803M0", "WNMG080408", "CNMG120408"] {
            let (insert, rn) = Insert::iso_code(code, 95.).unwrap();
            // The passes end the clearance short of the chuck (for the nose
            // center): a nose larger than the default 2 mm needs more.
            let mut j = shaft_with(insert, rn);
            j.clearance_mm = rn + 1.;
            let plan = plan(&j).unwrap_or_else(|e| panic!("{code}: {e}"));
            assert!(
                plan.reports
                    .iter()
                    .all(|r| r.min_part_clearance_mm >= -1e-3),
                "{code}"
            );
            // Finished to size along both diameters, away from the inside
            // corner a nose of this radius cannot reach into.
            for (i, c) in plan.stock.cells().iter().enumerate() {
                let [z0, z1] = plan.stock.bin(i);
                if z0 >= -20. + rn + 1. && z1 <= -1.5 {
                    assert!(
                        (c.outer.upper_mm - 10.).abs() < 1e-6 && c.outer.lower_mm >= 10. - 1e-6,
                        "{code} {z0} {c:?}"
                    );
                }
                if z0 >= -38.5 && z1 <= -21.5 {
                    assert!(
                        (c.outer.upper_mm - 15.).abs() < 1e-6 && c.outer.lower_mm >= 15. - 1e-6,
                        "{code} {z0} {c:?}"
                    );
                }
            }
        }
    }

    /// A holder whose head reaches `ahead` mm ahead of the nose (toward -z)
    /// at `above` mm above it (away from the axis), 30 mm long.
    fn head_ahead(ahead: f64, above: f64) -> Holder {
        let (x0, x1, z0, z1) = (above, above + 30., -ahead, 20.);
        Holder {
            outline: crate::turning::tool::Outline {
                start_mm: [x0, z0],
                segments: vec![
                    crate::turning::tool::Segment::Line { to_mm: [x1, z0] },
                    crate::turning::tool::Segment::Line { to_mm: [x1, z1] },
                    crate::turning::tool::Segment::Line { to_mm: [x0, z1] },
                    crate::turning::tool::Segment::Line { to_mm: [x0, z0] },
                ],
            },
            bar: None,
        }
    }

    #[test]
    fn a_holder_head_ahead_of_the_insert_is_caught() {
        // Roughing only, the stock already faced.
        // The stock ends at z -45: no uncut ring between the passes' end
        // (the chuck plus the clearance) and the chuck for a head to meet.
        let rough = |holder: Holder, chuck: f64| {
            let mut j = job();
            j.stock.z_max_mm = 0.;
            j.stock.z_min_mm = -45.;
            j.chuck_z_mm = chuck;
            j.operations
                .retain(|o| matches!(o, TurnOperation::Rough { .. }));
            j.tools[0].tool.holder = holder;
            j
        };
        // 1.5 mm ahead, clear of the stock ahead of each pass (2 mm above
        // the nose center, the passes 2 mm deep): the last pass under r 15
        // runs the head into the shoulder at z -20, below the part's r 15.
        let err = plan_err(&rough(head_ahead(1.5, 2.), -50.));
        assert!(err.contains("the holder would hit the part"), "{err}");
        // High enough for the shoulder, but 6 mm ahead: past the chuck face
        // (z -50) at the end of the outer passes (2 mm before it).
        let err = plan_err(&rough(head_ahead(6., 6.), -50.));
        assert!(err.contains("reach the chuck"), "{err}");
        // The same head 1.5 mm ahead clears both.
        plan(&rough(head_ahead(1.5, 6.), -50.)).unwrap();
    }

    #[test]
    fn a_boring_bar_outline_is_caught_in_a_small_bore() {
        // The boring job's 10 mm bar as a round bar under a small head.
        let bar = |diameter: f64| Holder {
            outline: crate::turning::tool::Outline {
                start_mm: [1., 0.5],
                segments: vec![
                    crate::turning::tool::Segment::Line { to_mm: [6., 0.5] },
                    crate::turning::tool::Segment::Line { to_mm: [6., 6.] },
                    crate::turning::tool::Segment::Line { to_mm: [1., 6.] },
                    crate::turning::tool::Segment::Line { to_mm: [1., 0.5] },
                ],
            },
            bar: Some(crate::turning::tool::Bar {
                diameter_mm: diameter,
                axis_x_mm: 1. + diameter / 2.,
                z_mm: [0., 80.],
            }),
        };
        let mut fits = bore_job();
        fits.tools.last_mut().unwrap().tool.holder = bar(10.);
        let fitted = plan(&fits).unwrap();
        assert!(
            fitted
                .reports
                .iter()
                .all(|r| r.min_holder_clearance_mm > 0.)
        );
        // A 17 mm bar does not fit the bore: its far side hits the wall.
        let mut fat = bore_job();
        fat.tools.last_mut().unwrap().tool.holder = bar(17.);
        let err = plan_err(&fat);
        assert!(err.contains("the holder would hit the part"), "{err}");
    }

    #[test]
    fn a_tool_without_a_holder_or_with_unknown_values_is_refused() {
        let tool = serde_json::to_value(&job().tools[0].tool).unwrap();
        let mut missing = tool.clone();
        missing.as_object_mut().unwrap().remove("holder");
        let err = serde_json::from_value::<TurningTool>(missing)
            .unwrap_err()
            .to_string();
        assert!(err.contains("missing field `holder`"), "{err}");
        let mut old = tool.clone();
        old.as_object_mut()
            .unwrap()
            .insert("holder_width_mm".into(), 20.into());
        assert!(serde_json::from_value::<TurningTool>(old).is_err());
        let mut no_nose = tool;
        no_nose.as_object_mut().unwrap().remove("nose_radius_mm");
        let err = serde_json::from_value::<TurningTool>(no_nose)
            .unwrap_err()
            .to_string();
        assert!(err.contains("missing field `nose_radius_mm`"), "{err}");
    }

    #[test]
    fn iso_wedge_outlines_keep_the_old_geometry() {
        // The wedge camlib has always modelled, exactly: the nose arc
        // circumscribed in 2 degree steps between the two edges' tangent
        // points, the edge ends at the edge length, the straight back.
        let t = test_tool(
            "t".into(),
            Insert::iso_turning(80., 95., 12., 0.8).unwrap(),
            0.8,
            TurnSide::Outer,
            Hand::Right,
            20.,
            100.,
            0.,
        );
        let s = t.shape().unwrap();
        let (major, minor) = (5f64, 85f64);
        let d = |a: f64| [a.to_radians().cos(), a.to_radians().sin()];
        let end = |a: f64, side: f64| {
            let (u, n) = (d(a), d(a + side * 90.));
            [0.8 * n[0] + 12. * u[0], 0.8 * n[1] + 12. * u[1]]
        };
        assert_eq!(s.insert[0], end(minor, 1.));
        assert_eq!(*s.insert.last().unwrap(), end(major, -1.));
        // 50 two-degree steps from 175 to 275 degrees: two tangent points
        // and 50 corners, plus the two edge ends.
        assert_eq!(s.insert.len(), 54);
        assert_eq!(s.edges.len(), 3);
        assert_eq!(s.back.as_ref().unwrap().len(), 1);
        // The holder starts at the insert's reach, 20 mm along z.
        let reach = s.insert.iter().map(|p| p[0]).fold(0., f64::max);
        assert_eq!(s.holder[0], [reach, 0.]);
        assert_eq!(s.holder[2], [reach + 100., 20.]);
    }

    /// A ball end (radius 6), a concave fillet (radius 3) into a shoulder
    /// and a sharp outer corner: (r, z, bulge).
    fn rounded_job() -> TurningJob {
        let b = std::f64::consts::FRAC_PI_8.tan();
        let mut j = job();
        j.part.points = vec![
            [0., 0., b],
            [6., -6., 0.],
            [6., -20., -b],
            [9., -23., 0.],
            [14., -23., 0.],
            [14., -40., 0.],
        ];
        j
    }
    /// The distance from (z, r) to `rounded_job`'s profile, from its
    /// drawing (nothing of the planner's).
    fn rounded_distance(z: f64, r: f64) -> f64 {
        let segment = |a: [f64; 2], b: [f64; 2]| {
            let (d, w) = ([b[0] - a[0], b[1] - a[1]], [z - a[0], r - a[1]]);
            let t = ((w[0] * d[0] + w[1] * d[1]) / (d[0] * d[0] + d[1] * d[1])).clamp(0., 1.);
            (w[0] - t * d[0]).hypot(w[1] - t * d[1])
        };
        // A quarter circle about c from angle a0 to a0 + 90 degrees
        // (angles from +z toward +r); its ends belong to the lines too.
        let quarter = |c: [f64; 2], radius: f64, a0: f64| {
            let a = (r - c[1]).atan2(z - c[0]);
            let within = (a - a0).rem_euclid(std::f64::consts::TAU) <= std::f64::consts::FRAC_PI_2;
            if within {
                ((z - c[0]).hypot(r - c[1]) - radius).abs()
            } else {
                f64::INFINITY
            }
        };
        [
            quarter([-6., 0.], 6., 0.),
            segment([-6., 6.], [-20., 6.]),
            quarter([-20., 9.], 3., std::f64::consts::PI),
            segment([-23., 9.], [-23., 14.]),
            segment([-23., 14.], [-40., 14.]),
            (z + 0.).hypot(r),
        ]
        .into_iter()
        .fold(f64::INFINITY, f64::min)
    }
    /// The finished radius of `rounded_job`'s part at z (the wall at z -23
    /// spans 9..14).
    fn rounded_radius(z: f64) -> f64 {
        if z > 0. {
            0.
        } else if z >= -6. {
            (36. - (z + 6.).powi(2)).sqrt()
        } else if z >= -20. {
            6.
        } else if z > -23. {
            9. - (9. - (z + 20.).powi(2)).sqrt()
        } else {
            14.
        }
    }
    fn arcs_of<'a>(plan: &'a TurnPlan, op: &str) -> Vec<&'a Motion> {
        plan.events
            .iter()
            .filter_map(|e| match e {
                Event::Motion { motion }
                    if motion.operation == op && matches!(motion.curve, Curve::Arc { .. }) =>
                {
                    Some(motion)
                }
                _ => None,
            })
            .collect()
    }
    /// Points along an XZ arc from its own words (G18: angles from +Z
    /// toward +X), as (z, x).
    fn arc_samples(curve: &Curve, n: usize) -> Vec<[f64; 2]> {
        let Curve::Arc {
            start,
            center,
            plane: Plane::Xz,
            sweep_rad,
            ..
        } = *curve
        else {
            panic!("not an XZ arc: {curve:?}")
        };
        let radius = (start[0] - center[0]).hypot(start[2] - center[2]);
        let a0 = (start[0] - center[0]).atan2(start[2] - center[2]);
        (0..=n)
            .map(|k| {
                let a = a0 + sweep_rad * k as f64 / n as f64;
                [center[2] + radius * a.cos(), center[0] + radius * a.sin()]
            })
            .collect()
    }

    #[test]
    fn finishing_follows_a_ball_end_a_fillet_and_a_corner_with_arcs() {
        use std::f64::consts::FRAC_PI_2;
        let mut j = rounded_job();
        // The program moves the nose center: the profile offset by rn.
        j.controlled_point = ControlledPoint::NoseCenter;
        let finished = plan(&j).unwrap();
        let arcs = arcs_of(&finished, "finish");
        // The ball end (rn outside it, G3), the fillet (rn inside it, G2)
        // and the nose rolling round the sharp corner at r 14 (G3).
        let shape: Vec<(f64, f64)> = arcs
            .iter()
            .map(|m| {
                let Curve::Arc {
                    start,
                    center,
                    sweep_rad,
                    ..
                } = m.curve
                else {
                    unreachable!()
                };
                (
                    (start[0] - center[0]).hypot(start[2] - center[2]),
                    sweep_rad,
                )
            })
            .collect();
        assert_eq!(shape.len(), 3, "{shape:?}");
        for (got, want) in shape
            .iter()
            .zip([(6.8, FRAC_PI_2), (2.2, -FRAC_PI_2), (0.8, FRAC_PI_2)])
        {
            assert!(
                (got.0 - want.0).abs() < 1e-9 && (got.1 - want.1).abs() < 1e-9,
                "{shape:?}"
            );
        }
        // Independently of the planner: every arc lies on the profile's
        // offset by the nose radius, end to end.
        for m in &arcs {
            assert_eq!(m.kind, MoveKind::Cut);
            let points = arc_samples(&m.curve, 200);
            let end = m.curve.end();
            let last = points[200];
            assert!((last[0] - end[2]).hypot(last[1] - end[0]) < 1e-9, "{m:?}");
            for p in points {
                let d = rounded_distance(p[0], p[1]);
                assert!((d - 0.8).abs() < 1e-9, "{p:?} is {d} from the profile");
            }
        }
        // Every finishing move joins the next, and the arcs replace the
        // chords: far fewer blocks than the chord chain of the same path.
        let moves: Vec<&Motion> = finished
            .events
            .iter()
            .filter_map(|e| match e {
                Event::Motion { motion } if motion.operation == "finish" => Some(motion),
                _ => None,
            })
            .collect();
        for w in moves.windows(2) {
            assert_eq!(w[0].curve.end(), w[1].curve.start());
        }
        let chords = chain(&j.part.offset(0.8).unwrap(), j.tolerance_mm / 4.).len();
        assert!(chords > 100 && moves.len() < 15, "{} {chords}", moves.len());
        // The stock, against the drawing: each bin's bounds enclose the
        // finished radius within the tolerance (nothing cut below it, all
        // of the allowance above it gone). The tolerance is a distance from
        // the surface: along z too, which matters where the surface is
        // steep (the ball's tip, the fillet's end at the wall).
        let tol = j.tolerance_mm;
        for (i, c) in finished.stock.cells().iter().enumerate() {
            let [za, zb] = finished.stock.bin(i);
            if za < -39. || zb > 0. {
                continue;
            }
            let mut at: Vec<f64> = [za - tol, za, zb, (zb + tol).min(0.)]
                .into_iter()
                .map(rounded_radius)
                .collect();
            if (za - tol..=zb + tol).contains(&-23.) {
                // The wall at z -23, from the fillet's end up.
                at.extend([9., 14.]);
            }
            let lo = at.iter().copied().fold(f64::INFINITY, f64::min);
            let hi = at.iter().copied().fold(f64::NEG_INFINITY, f64::max);
            // Nothing below the surface beyond the move checks' guard.
            assert!(c.outer.lower_mm >= lo - 2. * tol, "{za} {zb} {c:?} {lo}");
            // Removal is certain per piece of a move (one nose position's
            // disc across the whole bin, as for chords): on a curved
            // surface it stays above the surface by up to the disc's rise
            // over half a bin.
            let half = j.stock.cell_mm / 2.;
            let rise = 0.8 - (0.64 - half * half).sqrt();
            assert!(c.outer.upper_mm <= hi + tol + rise, "{za} {zb} {c:?} {hi}");
        }
        let report = material_report(&j, &finished.stock).unwrap();
        assert_eq!(report["gouge"]["assessment"], "within_tolerance");
        assert_eq!(report["residual"]["definite_cells"], 0, "{report}");
        // The posted moves replayed (arcs through the same kernel) leave
        // exactly the planned stock.
        let mut replay = TurnReplay::new(j.clone()).unwrap();
        let mut stock = TurnStock::new(j.stock.clone()).unwrap();
        for e in &finished.events {
            if let Event::Motion { motion } = e {
                let d = replay.motion(motion).unwrap();
                stock.apply_move(&d.changes, &d.rings).unwrap();
            }
        }
        assert_eq!(stock.cells(), finished.stock.cells());
    }

    #[test]
    fn an_arc_of_the_imaginary_tip_is_the_nose_arc_translated() {
        let mut j = rounded_job();
        let tip = plan(&j).unwrap();
        j.controlled_point = ControlledPoint::NoseCenter;
        let nose = plan(&j).unwrap();
        let (tip, nose) = (arcs_of(&tip, "finish"), arcs_of(&nose, "finish"));
        assert_eq!(tip.len(), 3);
        for (t, n) in tip.iter().zip(&nose) {
            let (
                Curve::Arc {
                    start: ts,
                    end: te,
                    center: tc,
                    sweep_rad: tw,
                    ..
                },
                Curve::Arc {
                    start: ns,
                    end: ne,
                    center: nc,
                    sweep_rad: nw,
                    ..
                },
            ) = (&t.curve, &n.curve)
            else {
                unreachable!()
            };
            assert_eq!(tw, nw);
            // One nose radius toward the axis and toward -z.
            for (a, b) in [(ts, ns), (te, ne), (tc, nc)] {
                assert!((a[0] - (b[0] - 0.8)).abs() < 1e-12 && (a[2] - (b[2] - 0.8)).abs() < 1e-12);
                assert_eq!(a[1], 0.);
            }
        }
    }

    #[test]
    fn bore_finishing_rolls_round_a_step_with_an_arc() {
        let plan = plan(&bore_job()).unwrap();
        let arcs = arcs_of(&plan, "bore_finish");
        // The 0.4 mm nose round the step's corner (r 9, z -20); the tip is
        // one nose radius outward and toward -z of the nose center.
        assert_eq!(arcs.len(), 1, "{arcs:?}");
        let Curve::Arc {
            start,
            end,
            center,
            sweep_rad,
            ..
        } = arcs[0].curve
        else {
            unreachable!()
        };
        assert!((center[0] - 9.4).abs() < 1e-9 && (center[2] + 20.4).abs() < 1e-9);
        for p in [start, end] {
            assert!(((p[0] - center[0]).hypot(p[2] - center[2]) - 0.4).abs() < 1e-9);
        }
        // From the step's face round to the r 9 wall: a quarter turn.
        assert!((sweep_rad.abs() - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
        for p in arc_samples(&arcs[0].curve, 50) {
            // Nose center: the tip less its offset; rn from the corner.
            let (z, r) = (p[0] + 0.4, p[1] - 0.4);
            assert!(((z + 20.).hypot(r - 9.) - 0.4).abs() < 1e-9, "{p:?}");
            assert!(r <= 9. + 1e-9 || z >= -20. - 1e-9);
        }
    }

    #[test]
    fn arcs_are_checked_like_the_chords_they_replace() {
        // A lathe arc is one nose cutting: never a rapid.
        let j = rounded_job();
        let mut replay = TurnReplay::new(j.clone()).unwrap();
        let tool = j.tools[0].tool.id.clone();
        let arc = |kind, start: [f64; 3], end: [f64; 3], center: [f64; 3], sweep_rad| Motion {
            id: 1,
            operation: "finish".into(),
            tool: tool.clone(),
            curve: Curve::Arc {
                start,
                end,
                center,
                plane: Plane::Xz,
                sweep_rad,
            },
            kind,
            feed_mm_min: Some(100.),
        };
        let quarter = std::f64::consts::FRAC_PI_2;
        let rapid = arc(
            MoveKind::Rapid,
            [25., 0., 10.],
            [27., 0., 8.],
            [25., 0., 8.],
            quarter,
        );
        assert!(replay.motion(&rapid).unwrap_err().contains("lathe arc"));
        // An arc whose ends are clear of the part but which bulges into it
        // (the tip from just above the r 14 diameter down to r 9.05 at
        // z -30 and back) is refused, and the stock is unchanged.
        let into = arc(
            MoveKind::Cut,
            [14.05, 0., -25.],
            [14.05, 0., -35.],
            [14.05, 0., -30.],
            -2. * quarter,
        );
        assert!(replay.motion(&into).unwrap_err().contains("into the part"));
        // An end off the arc's circle beyond the tolerance is refused.
        let open = arc(
            MoveKind::Cut,
            [25., 0., 10.],
            [27.1, 0., 8.],
            [25., 0., 8.],
            quarter,
        );
        assert!(
            replay
                .motion(&open)
                .unwrap_err()
                .contains("not on its circle")
        );
        assert_eq!(
            replay.stock.as_ref().unwrap().cells(),
            TurnStock::new(j.stock.clone()).unwrap().cells()
        );
    }

    #[test]
    fn the_certain_band_of_a_side_is_swept_by_the_arc() {
        // A straight side carried along an arc piece: every point of the
        // band claimed as certainly swept is on the side at some pose along
        // the arc (found by brute force, not by the band's own algebra).
        let (c, r) = ([1., -2.], 5.);
        let from = [c[0] + r * 0.3f64.cos(), c[1] + r * 0.3f64.sin()];
        let to = [c[0] + r * 0.9f64.cos(), c[1] + r * 0.9f64.sin()];
        // A coarse tolerance: pieces with a visible sagitta.
        let steps = arc_steps(from, to, c, 0.6, 0.2).unwrap();
        assert!(steps.len() >= 2 && steps[0].sag > 0.01, "{}", steps.len());
        let on_side = |e: &[[f64; 2]; 2], x: [f64; 2], a: f64| {
            // The distance from x to the side at the arc's pose at angle a.
            let p = [c[0] + r * a.cos(), c[1] + r * a.sin()];
            let (s, t) = (
                [e[0][0] + p[0], e[0][1] + p[1]],
                [e[1][0] + p[0], e[1][1] + p[1]],
            );
            let d = [t[0] - s[0], t[1] - s[1]];
            let k = (((x[0] - s[0]) * d[0] + (x[1] - s[1]) * d[1]) / (d[0] * d[0] + d[1] * d[1]))
                .clamp(0., 1.);
            (x[0] - s[0] - k * d[0]).hypot(x[1] - s[1] - k * d[1])
        };
        let mut bands = 0;
        for e in [
            [[0.2, 0.1], [1.5, 2.]],
            [[-0.4, 0.3], [-2., 0.5]],
            [[0., 0.5], [0.3, 3.]],
        ] {
            for (k, step) in steps.iter().enumerate() {
                let Some(band) = certain_band(&e, &step.points, step.sag, step.slack) else {
                    continue;
                };
                bands += 1;
                let (a, b) = (
                    0.3 + 0.6 * k as f64 / steps.len() as f64,
                    0.3 + 0.6 * (k + 1) as f64 / steps.len() as f64,
                );
                // The band's corners and points inside it.
                let n = band.len();
                let centre = band.iter().fold([0., 0.], |m, p| {
                    [m[0] + p[0] / n as f64, m[1] + p[1] / n as f64]
                });
                let mut probes = band.clone();
                probes.extend(
                    band.iter()
                        .map(|p| [(p[0] + centre[0]) / 2., (p[1] + centre[1]) / 2.]),
                );
                for x in probes {
                    let nearest = (0..=20_000)
                        .map(|i| on_side(&e, x, a + (b - a) * i as f64 / 20_000.))
                        .fold(f64::INFINITY, f64::min);
                    assert!(nearest < 1e-4, "{e:?} piece {k}: {x:?} is {nearest} off");
                }
                // The side's own end carried along the chord is not swept
                // where the arc bulges away from it: the trim is needed.
                let chord_end = [
                    e[0][0] + (step.points[0][0] + step.points[2][0]) / 2.,
                    e[0][1] + (step.points[0][1] + step.points[2][1]) / 2.,
                ];
                let other_end = [
                    e[1][0] + (step.points[0][0] + step.points[2][0]) / 2.,
                    e[1][1] + (step.points[0][1] + step.points[2][1]) / 2.,
                ];
                let miss = [chord_end, other_end]
                    .into_iter()
                    .map(|x| {
                        (0..=20_000)
                            .map(|i| on_side(&e, x, a + (b - a) * i as f64 / 20_000.))
                            .fold(f64::INFINITY, f64::min)
                    })
                    .fold(0., f64::max);
                assert!(miss > 1e-3, "{e:?} piece {k}: {miss}");
            }
        }
        assert!(bands >= 4, "{bands}");
    }

    #[test]
    fn an_arc_read_back_with_rounded_words_is_checked_between_its_radii() {
        // The fillet of `rounded_job` as a controller reads it from words
        // rounded to 3 decimals in a way that leaves the end 0.8 um off the
        // start's circle: accepted within the tolerance (1 um), with the
        // stock bounds still enclosing the drawn surface.
        let mut j = rounded_job();
        j.controlled_point = ControlledPoint::NoseCenter;
        let finished = plan(&j).unwrap();
        let mut replay = TurnReplay::new(j.clone()).unwrap();
        let mut stock = TurnStock::new(j.stock.clone()).unwrap();
        let fillet = arcs_of(&finished, "finish")[1].id;
        let mut position: Option<[f64; 3]> = None;
        for e in &finished.events {
            let Event::Motion { motion } = e else {
                continue;
            };
            let mut m = motion.clone();
            // Keep the path continuous from a moved end.
            if let Some(p) = position.take() {
                match &mut m.curve {
                    Curve::Line { start, .. } | Curve::Arc { start, .. } => *start = p,
                    Curve::Helix { .. } => unreachable!(),
                }
            }
            if m.id == fillet {
                let Curve::Arc { end, .. } = &mut m.curve else {
                    unreachable!()
                };
                // Toward +z: the end's radius about the centre (r 9, z -20)
                // is 0.8 um less than the start's.
                end[2] += 0.0008;
                position = Some(*end);
            }
            let d = replay.motion(&m).unwrap();
            stock.apply_move(&d.changes, &d.rings).unwrap();
        }
        let tol = j.tolerance_mm;
        let mut differs = false;
        for (i, (c, planned)) in stock.cells().iter().zip(finished.stock.cells()).enumerate() {
            let [za, zb] = stock.bin(i);
            if za < -22.5 || zb > -20. {
                continue;
            }
            differs |= c != planned;
            // The fillet's bins: the bounds still enclose the surface the
            // planned arc leaves, and stay within the moved end's distance
            // of the planned bounds.
            let at = [za, zb].map(rounded_radius);
            assert!(c.outer.lower_mm <= at[0].min(at[1]) + 1e-9, "{za} {c:?}");
            assert!(
                c.outer.lower_mm >= planned.outer.lower_mm - 2. * tol,
                "{za} {c:?}"
            );
            assert!(
                c.outer.upper_mm >= planned.outer.upper_mm - 1e-12,
                "{za} {c:?}"
            );
            assert!(
                c.outer.upper_mm <= planned.outer.upper_mm + 2. * tol,
                "{za} {c:?}"
            );
        }
        assert!(differs);
        // Farther off than the tolerance, the arc is refused.
        let mut replay = TurnReplay::new(j.clone()).unwrap();
        let mut refused = false;
        for e in &finished.events {
            let Event::Motion { motion } = e else {
                continue;
            };
            let mut m = motion.clone();
            if m.id == fillet {
                let Curve::Arc { end, .. } = &mut m.curve else {
                    unreachable!()
                };
                end[2] += 0.002;
                assert!(replay.motion(&m).unwrap_err().contains("not on its circle"));
                refused = true;
                break;
            }
            replay.motion(&m).unwrap();
        }
        assert!(refused);
    }
}
