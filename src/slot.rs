//! Lazy engagement-bounded trochoidal slots along a centreline of lines and
//! arcs (a straight slot is one line).
//!
//! Each depth pass orbits full circles of radius rho = width/2 - r - guard at
//! stations along the centreline; an orbit clears the disk of radius
//! a = rho + r around its station. Links follow the exact rho-offset of the
//! centreline on the orbit's side, so they join the orbits tangentially.
//!
//! - Engagement: after an orbit at C_k all remaining material lies outside
//!   that disk. Up to the next orbit the cutter centre stays within
//!   max_s |C(s) - C_k| + rho of C_k; on a line or an arc of at most a half
//!   turn (|bulge| <= 1) that maximum is the chord to the next station. The
//!   exposed edge angle 2 acos((a^2 - q^2 - r^2) / (2 q r)) grows with q, so a
//!   chord bound bounds the engagement, whatever the curvature.
//! - Coverage: stations sit at every vertex, so each step lies on one piece.
//!   Two disks whose stations are seen from an arc's centre at half-angle
//!   alpha leave their deepest cusp on the bisector at the outer wall:
//!   a - sqrt(a^2 - R^2 sin^2 alpha) + R (1 - cos alpha) <= t (on a line,
//!   R -> infinity: a - sqrt(a^2 - (p/2)^2) <= t). The outside of a bend sets
//!   the step.
//! - Corners: the orbit at the vertex clears a disk that contains both the
//!   incoming and the outgoing link offsets; a straight link between them
//!   stays inside it (no engagement).
//! - Links need the offset defined: an arc bending toward the orbit side needs
//!   a radius above rho (refused otherwise, naming the radius).
use crate::{
    Point,
    motion::{Coolant, Curve, Event, Motion, MoveKind, Plane, SpindleDirection, Vec3},
    pocket::ResidualStatus,
    stock::{CancelSignal, Stock, StockSpec},
    tool::{CutterShape, ToolAssembly, valid_id},
};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::f64::consts::TAU;

/// Stock-query work one slot operation may spend on its stock-clear fast
/// path, shared by every station. A query that finds material near the far
/// end of a piece rescans almost the same area from the next station, so a
/// per-query allowance is unbounded in total; once the shared budget is
/// spent the plan keeps its bounded orbits and queries no more.
const STOCK_QUERY_WORK: usize = 1_000_000;
/// History-clearance work one slot operation may spend proving its passes'
/// straight plunges clear, shared by every pass (each pass's proof revisits
/// the history of the passes before it); out of work a plunge is not proven
/// and the pass takes its helix or is refused.
const PLUNGE_PROOF_WORK: u64 = 1_000_000;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TrochoidalSlotOperation {
    pub id: String,
    pub tool: String,
    /// A straight slot from `start_mm` to `end_mm`, or (instead) `path_mm`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub start_mm: Option<Point>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub end_mm: Option<Point>,
    /// The slot centreline as an open polyline of lines and arcs: vertices
    /// `[x_mm, y_mm, bulge_to_next]`, bulge = tan(signed sweep / 4) within
    /// [-1, 1] (at most a half turn per arc, as for planar boundaries); the
    /// last vertex's bulge is 0.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub path_mm: Option<Vec<[f64; 3]>>,
    pub width_mm: f64,
    pub top_z_mm: f64,
    pub floor_z_mm: f64,
    pub stepdown_mm: f64,
    pub clearance_z_mm: f64,
    pub feed_mm_min: f64,
    pub entry_feed_mm_min: f64,
    pub max_engagement_deg: f64,
    pub boundary_tolerance_mm: f64,
    pub tolerance_mm: f64,
    pub clockwise: bool,
    pub spindle_rpm: f64,
    pub work_offset: u8,
    pub tool_number: u32,
    pub length_register: u32,
    pub coolant: Coolant,
    /// Rapid travel height between passes (absolute Z). Default: clearance.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub retract_z_mm: Option<f64>,
    /// Rapid descent stops here and plunge feed begins; leaving a pass, feed
    /// changes to rapid here (absolute Z, above the stock top). Default: retract.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub feed_z_mm: Option<f64>,
    /// Containment and avoid areas (clamps, fixtures, features to leave alone).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub limits: Option<crate::limits::Limits>,
    /// Feed (mm/min) on the part of each controlled orbit proven to run
    /// inside the disk the previous orbit cleared at the same depth. None:
    /// whole orbits at feed_mm_min (today's output, unchanged).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub air_feed_mm_min: Option<f64>,
    /// How each depth pass enters below the stock: a `HelicalEntry` on the
    /// orbit circle (radius rho, the op's sense, its own angle and feed).
    /// The cutter plunges at the entry feed only down to the highest
    /// stock the entry's footprint (the disk of radius rho + r) can touch,
    /// and helixes from there; no entry declared, a plunge into stock is
    /// refused, naming the pass.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub entry: Option<crate::entry::PocketEntry>,
}

#[derive(Debug, Serialize)]
pub struct SlotReport {
    pub strategy: &'static str,
    pub geometric_residual_status: ResidualStatus,
    pub depth_passes: usize,
    pub target_z_mm: f64,
    pub width_mm: f64,
    pub orbit_radius_mm: f64,
    pub max_engagement_deg: f64,
    pub boundary_underclearance_bound_mm: f64,
    pub bounded_advance_mm: f64,
    pub max_bounded_engagement_deg: f64,
    pub seed_entry_policy: &'static str,
    pub seed_orbits: usize,
    pub controlled_orbits: usize,
    pub stock_clear_advances: usize,
    pub stock_query_work: usize,
    pub stock_query_work_exhausted: bool,
    /// History-proof work spent on straight-plunge proofs, all passes
    /// together (at most `PLUNGE_PROOF_WORK`).
    #[serde(skip_serializing_if = "is_zero")]
    pub plunge_proof_work: usize,
    /// The shared plunge-proof budget ran out: later passes were not proven
    /// clear for a straight plunge (they take their helix or are refused).
    #[serde(skip_serializing_if = "std::ops::Not::not")]
    pub plunge_proof_work_exhausted: bool,
    /// Lines and arcs of the centreline (1 for a straight slot).
    pub path_pieces: usize,
    /// Links across corners, inside the vertex orbit's disk.
    pub corner_links: usize,
    /// Of those, the tangent arcs on the vertex orbit's circle (outer
    /// corners); the rest are straight chords.
    #[serde(skip_serializing_if = "is_zero")]
    pub corner_arcs: usize,
    /// Controlled-orbit arcs fed at `air_feed_mm_min` because they are
    /// proven to run inside the disk the previous orbit cleared at the same
    /// depth.
    #[serde(skip_serializing_if = "is_zero")]
    pub air_fed_arcs: usize,
    /// Total length of air-fed arcs and straight returns.
    #[serde(skip_serializing_if = "is_zero_f64")]
    pub air_fed_mm: f64,
    /// D-shaped (peel) loops: controlled orbits whose proven-air stretch is
    /// cut short by a straight return chord through the previous orbit's
    /// cleared disk, and the path length those chords saved against the arc.
    #[serde(skip_serializing_if = "is_zero")]
    pub return_chords: usize,
    #[serde(skip_serializing_if = "is_zero_f64")]
    pub return_chord_saved_mm: f64,
    /// Depth passes entered on a helix, and its turns in all.
    #[serde(skip_serializing_if = "is_zero")]
    pub helical_entries: usize,
    #[serde(skip_serializing_if = "is_zero")]
    pub helix_turns: usize,
    /// The deepest straight plunge below the stock top, each one proven
    /// to run through air (no stock under the entry's footprint).
    pub max_air_plunge_mm: f64,
}

fn helix(op: &TrochoidalSlotOperation) -> Option<&crate::entry::HelicalEntry> {
    op.entry.as_ref().and_then(|e| e.helix())
}

fn is_zero(n: &usize) -> bool {
    *n == 0
}
fn is_zero_f64(n: &f64) -> bool {
    *n == 0.
}

/// One piece of the slot centreline.
#[derive(Clone, Copy, Debug)]
enum Piece {
    Line {
        a: Point,
        b: Point,
        length: f64,
        dir: Point,
    },
    /// Counter-clockwise for a positive sweep (at most a half turn).
    Arc {
        a: Point,
        b: Point,
        center: Point,
        radius: f64,
        start: f64,
        sweep: f64,
    },
}

fn distance(a: Vec3, b: Vec3) -> f64 {
    metallic::hypot(a[0] - b[0], a[1] - b[1])
}

impl Piece {
    /// Cursor range: mm along a line, radians along an arc.
    fn extent(&self) -> f64 {
        match self {
            Piece::Line { length, .. } => *length,
            Piece::Arc { sweep, .. } => sweep.abs(),
        }
    }
    fn point(&self, along: f64) -> Point {
        match *self {
            Piece::Line {
                a, b, length, dir, ..
            } => {
                if along == length {
                    b
                } else {
                    [a[0] + dir[0] * along, a[1] + dir[1] * along]
                }
            }
            Piece::Arc {
                a,
                b,
                center,
                radius,
                start,
                sweep,
            } => {
                if along == 0. {
                    a
                } else if along == sweep.abs() {
                    b
                } else {
                    let t = start + sweep.signum() * along;
                    let (sin, cos) = metallic::sincos(t);
                    [center[0] + radius * cos, center[1] + radius * sin]
                }
            }
        }
    }
    /// Unit forward tangent.
    fn tangent(&self, along: f64) -> Point {
        match *self {
            Piece::Line { dir, .. } => dir,
            Piece::Arc { start, sweep, .. } => {
                let t = start + sweep.signum() * along;
                let (sin, cos) = metallic::sincos(t);
                [-sweep.signum() * sin, sweep.signum() * cos]
            }
        }
    }
    fn curve(&self, z: f64) -> Curve {
        match *self {
            Piece::Line { a, b, .. } => Curve::Line {
                start: [a[0], a[1], z],
                end: [b[0], b[1], z],
            },
            Piece::Arc {
                a,
                b,
                center,
                sweep,
                ..
            } => Curve::Arc {
                start: [a[0], a[1], z],
                end: [b[0], b[1], z],
                center: [center[0], center[1], z],
                plane: Plane::Xy,
                sweep_rad: sweep,
            },
        }
    }
}

/// The centreline pieces of a slot operation (validated).
fn pieces(op: &TrochoidalSlotOperation, guard: f64) -> Result<Vec<Piece>, String> {
    let vertices: Vec<[f64; 3]> = match (&op.start_mm, &op.end_mm, &op.path_mm) {
        (Some(a), Some(b), None) => vec![[a[0], a[1], 0.], [b[0], b[1], 0.]],
        (None, None, Some(path)) => path.clone(),
        _ => {
            return Err(
                "a trochoidal slot needs either start_mm and end_mm (straight) or path_mm".into(),
            );
        }
    };
    if vertices.len() < 2 || vertices.len() > 1_000_000 {
        return Err("path_mm needs 2..1000000 vertices".into());
    }
    if vertices
        .iter()
        .any(|v| v.iter().any(|x| !x.is_finite()) || v[0].abs() > 1e6 || v[1].abs() > 1e6)
        || vertices.iter().any(|v| v[2].abs() > 1.)
    {
        return Err(
            "path vertices need finite coordinates within ±1e6 mm and bulge within [-1, 1]".into(),
        );
    }
    if vertices.last().unwrap()[2] != 0. {
        return Err("the last path vertex starts no piece: its bulge must be 0".into());
    }
    let mut out = Vec::with_capacity(vertices.len() - 1);
    for (i, w) in vertices.windows(2).enumerate() {
        let (a, b, bulge) = ([w[0][0], w[0][1]], [w[1][0], w[1][1]], w[0][2]);
        let chord = metallic::hypot(b[0] - a[0], b[1] - a[1]);
        if chord <= guard {
            return Err(format!(
                "path piece {i}: its end points coincide (start_mm and end_mm must be different points)"
            ));
        }
        let dir = [(b[0] - a[0]) / chord, (b[1] - a[1]) / chord];
        if bulge == 0. {
            out.push(Piece::Line {
                a,
                b,
                length: chord,
                dir,
            });
            continue;
        }
        let sweep = 4. * metallic::atan(bulge);
        let radius = chord / (2. * metallic::sin(sweep / 2.).abs());
        if radius > 1e6 {
            return Err(format!(
                "path piece {i}: bulge {bulge} is too small for an arc; use a line (bulge 0)"
            ));
        }
        // The centre is on the chord's bisector, left of it for a
        // counter-clockwise arc.
        let h = chord / 2. / metallic::tan(sweep / 2.);
        let center = [
            (a[0] + b[0]) / 2. - dir[1] * h,
            (a[1] + b[1]) / 2. + dir[0] * h,
        ];
        out.push(Piece::Arc {
            a,
            b,
            center,
            radius,
            start: metallic::atan2(a[1] - center[1], a[0] - center[0]),
            sweep,
        });
    }
    Ok(out)
}

/// The slot centreline as motion curves at `z` (for limit checks).
pub fn centerline_curves(op: &TrochoidalSlotOperation, z: f64) -> Result<Vec<Curve>, String> {
    Ok(pieces(op, 16. * op.tolerance_mm)?
        .iter()
        .map(|p| p.curve(z))
        .collect())
}

/// The largest half-angle alpha (seen from the centre of an arc of radius
/// `radius`) between two stations whose disks of radius `a` leave at most `t`
/// uncovered at the outer wall: a - sqrt(a^2 - R^2 sin^2 alpha) +
/// R (1 - cos alpha) <= t. The left side grows with alpha (bisection).
fn arc_coverage_half_angle(radius: f64, a: f64, t: f64) -> f64 {
    let gap = |alpha: f64| {
        let s = radius * metallic::sin(alpha);
        a - (a * a - s * s).max(0.).sqrt() + 2. * radius * metallic::sin(alpha / 2.).powi(2)
    };
    let mut lo = 0f64;
    let mut hi = std::f64::consts::FRAC_PI_2.min(metallic::asin((a / radius).min(1.)));
    if gap(hi) <= t {
        return hi;
    }
    for _ in 0..200 {
        let mid = (lo + hi) / 2.;
        if mid <= lo || mid >= hi {
            break;
        }
        if gap(mid) <= t { lo = mid } else { hi = mid }
    }
    lo
}

/// Fillet radius of a rounded return, as a fraction of the orbit radius.
/// Measured on the trochoidal-slot bench (tangent cycle model) over 0.1 to
/// 0.95: the smallest was fastest with and without an air feed, since the
/// return stays shortest while still tangent throughout.
const RETURN_FILLET: f64 = 0.1;

/// One arc piece of a split controlled orbit: same centre and direction as
/// the whole orbit, same start and end points joined up; `air` marks the
/// proven-air piece fed at `air_feed_mm_min`.
#[derive(Clone, Copy, Debug)]
struct OrbitPiece {
    /// Exact end point; the next piece starts here and the last piece ends
    /// at the orbit start point.
    end: Vec3,
    /// Signed sweep from the piece's start to its end.
    sweep: f64,
    air: bool,
}

/// Stores only a scalar path cursor, not a list of generated orbits/motions.
pub struct SlotPlan {
    op: TrochoidalSlotOperation,
    heights: crate::heights::Heights,
    spec: StockSpec,
    initial: Vec3,
    position: Vec3,
    pieces: Vec<Piece>,
    /// Bounded advance per piece (mm on a line, radians on an arc).
    steps: Vec<f64>,
    /// +1: links run on the left of the centreline (clockwise orbits), -1 right.
    side: f64,
    piece: usize,
    radius: f64,
    cutter_radius: f64,
    guard: f64,
    pass: usize,
    distance: f64,
    // 0=up,1=across,2=plunge,3=seed,4=advance,5=orbit,6=retract,7=done
    phase: u8,
    next_id: u64,
    expected_generation: Option<u64>,
    /// What is left of `STOCK_QUERY_WORK` for the stock-clear fast path.
    query_work_left: usize,
    /// What is left of `PLUNGE_PROOF_WORK`, shared by every pass's plunge proof.
    plunge_work_left: u64,
    /// Centre of the most recent completed orbit at this depth (seed or
    /// controlled): the predecessor the next controlled orbit's proven-air
    /// proof needs. None at a new depth pass and after a stock-clear
    /// advance; corner links keep it (the vertex orbit is the previous
    /// orbit).
    prev_center: Option<Vec3>,
    /// Pending arc pieces of the controlled orbit being emitted (phase 5),
    /// one motion per call like every other phase.
    orbit_pieces: VecDeque<OrbitPiece>,
    /// Motions of the orbit piece being emitted: one cut arc, or the
    /// rounded return of a proven-air piece (marked true).
    pending: VecDeque<(Curve, bool)>,
    /// The helix of the current pass's entry (phase 10): turns left, and
    /// the z each remaining turn ends at is spread evenly down to `z`.
    helix: Option<HelixDescent>,
    report: SlotReport,
    /// How the run splits the straight-up lift after each pass (#441).
    lift_split: Option<crate::lift::Lifts>,
}

#[derive(Clone, Copy, Debug)]
struct HelixDescent {
    top: f64,
    turns: usize,
    done: usize,
}

impl SlotPlan {
    pub fn prepare(
        op: TrochoidalSlotOperation,
        assembly: &ToolAssembly,
        spec: &StockSpec,
        initial: Vec3,
        first_motion_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        if cancel.is_cancelled() {
            return Err("slot planning cancelled".into());
        }
        spec.grid()?;
        let tool = assembly.prepare()?;
        let cutter = tool.cutter().definition();
        let r = cutter.diameter_mm / 2.;
        let values = [
            op.width_mm,
            op.top_z_mm,
            op.floor_z_mm,
            op.stepdown_mm,
            op.clearance_z_mm,
            op.feed_mm_min,
            op.entry_feed_mm_min,
            op.max_engagement_deg,
            op.boundary_tolerance_mm,
            op.tolerance_mm,
            op.spindle_rpm,
        ];
        let guard = 16. * op.tolerance_mm;
        let pieces = pieces(&op, guard)?;
        crate::heights::require(
            "trochoidal slot",
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
                    matches!(cutter.shape, CutterShape::Flat) && cutter.center_cutting,
                    "a trochoidal slot needs a flat, center-cutting end mill".into(),
                ),
                (
                    values.iter().all(|v| v.is_finite())
                        && initial.iter().all(|v| v.is_finite() && v.abs() <= 1e6),
                    "all numeric parameters and points must be finite".into(),
                ),
                (
                    (1e-7..=0.01).contains(&op.tolerance_mm),
                    format!("tolerance_mm {} must be within 1e-7..0.01", op.tolerance_mm),
                ),
                (
                    (1.0..=170.0).contains(&op.max_engagement_deg),
                    format!(
                        "max_engagement_deg {} must be within 1..170",
                        op.max_engagement_deg
                    ),
                ),
                (
                    op.boundary_tolerance_mm > 2. * guard && op.boundary_tolerance_mm <= r / 4.,
                    format!(
                        "boundary_tolerance_mm {} must be positive and at most a quarter of the cutter radius ({})",
                        op.boundary_tolerance_mm,
                        r / 4.
                    ),
                ),
                (
                    op.width_mm > 2. * r + 4. * guard && op.width_mm <= 4. * r,
                    format!(
                        "width_mm {} must be wider than the cutter diameter {} and at most twice it",
                        op.width_mm,
                        2. * r
                    ),
                ),
                (
                    op.stepdown_mm > guard && op.stepdown_mm <= cutter.cutting_length_mm,
                    format!(
                        "stepdown_mm {} must be positive and at most the cutting length {}",
                        op.stepdown_mm, cutter.cutting_length_mm
                    ),
                ),
                (
                    op.floor_z_mm >= spec.min_mm[2] && op.floor_z_mm < op.top_z_mm,
                    format!(
                        "floor_z_mm {} must be below top_z_mm {} and not below the stock bottom {}",
                        op.floor_z_mm, op.top_z_mm, spec.min_mm[2]
                    ),
                ),
                (
                    op.top_z_mm == spec.max_mm[2],
                    format!(
                        "top_z_mm {} must equal the stock top {}",
                        op.top_z_mm, spec.max_mm[2]
                    ),
                ),
                (
                    op.top_z_mm - op.floor_z_mm <= cutter.cutting_length_mm,
                    crate::reach::ReachRefusal::cutting_length(
                        &op.id,
                        &op.tool,
                        op.top_z_mm,
                        op.floor_z_mm,
                        cutter.cutting_length_mm,
                    )
                    .error(format_args!(
                        "slot depth {} exceeds the cutting length {}",
                        op.top_z_mm - op.floor_z_mm,
                        cutter.cutting_length_mm
                    )),
                ),
                (
                    op.clearance_z_mm > op.top_z_mm + guard && op.clearance_z_mm.abs() <= 1e6,
                    format!(
                        "clearance_z_mm {} must be above top_z_mm {}",
                        op.clearance_z_mm, op.top_z_mm
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
                    op.feed_mm_min > 0. && op.entry_feed_mm_min > 0. && op.spindle_rpm > 0.,
                    format!(
                        "feed_mm_min {}, entry_feed_mm_min {} and spindle_rpm {} must be positive",
                        op.feed_mm_min, op.entry_feed_mm_min, op.spindle_rpm
                    ),
                ),
                (
                    op.air_feed_mm_min
                        .is_none_or(|air| air.is_finite() && air >= op.feed_mm_min),
                    format!(
                        "air_feed_mm_min {} must be finite and at least feed_mm_min {}",
                        op.air_feed_mm_min.unwrap_or(f64::NAN),
                        op.feed_mm_min
                    ),
                ),
                (
                    match &op.entry {
                        None | Some(crate::entry::PocketEntry::Helix(_)) => true,
                        Some(_) => false,
                    },
                    "a trochoidal slot enters on a HelicalEntry only (the helix runs on the orbit circle); \
                     StockEdgeEntry and RampEntry are not supported here"
                        .into(),
                ),
                (
                    helix(&op).is_none_or(|h| {
                        h.validate(f64::INFINITY, op.tolerance_mm, op.entry_feed_mm_min)
                            .is_ok()
                            && h.radius_mm.is_none()
                            && h.clockwise.is_none_or(|c| c == op.clockwise)
                    }),
                    "the slot's HelicalEntry needs a valid angle and feed, no radius_mm (it runs on \
                     the orbit circle, width/2 - tool radius) and no clockwise other than the slot's"
                        .into(),
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
        let radius = op.width_mm / 2. - r - guard;
        let a = radius + r;
        let half_phi = op.max_engagement_deg.to_radians() / 2.;
        let (sin, cos) = metallic::sincos(half_phi);
        let engagement_pitch = (a * a - (r * sin).powi(2)).sqrt() - r * cos - radius - guard;
        let t = op.boundary_tolerance_mm - guard;
        let coverage_pitch = 2. * (2. * a * t - t * t).sqrt();
        let pitch = engagement_pitch.min(coverage_pitch) * (1. - 1e-9);
        let passes = ((op.top_z_mm - op.floor_z_mm) / op.stepdown_mm).ceil() as usize;
        if !pitch.is_finite()
            || pitch <= guard
            || passes == 0
            || first_motion_id == 0
            || first_motion_id.checked_add(1).is_none()
        {
            return Err(
                "slot engagement/geometry tolerance or motion budget is too restrictive".into(),
            );
        }
        // Clockwise orbits leave and rejoin their links on the left of the
        // centreline (moving forward there), counter-clockwise on the right.
        let side = if op.clockwise { 1. } else { -1. };
        let mut steps = Vec::with_capacity(pieces.len());
        for (i, piece) in pieces.iter().enumerate() {
            steps.push(match *piece {
                Piece::Line { .. } => pitch,
                Piece::Arc { radius: big, sweep, .. } => {
                    // The links' offset radius on the orbit side.
                    let offset = big - side * sweep.signum() * radius;
                    if offset <= guard {
                        return Err(format!(
                            "slot {}: path piece {i} bends toward the orbit side with radius {big:.4} mm, \
                             not more than the orbit radius {radius:.4} mm (width/2 - tool radius): \
                             the centreline needs a larger radius there, a narrower slot or a larger \
                             tool, or clockwise reversed",
                            op.id
                        ));
                    }
                    // Engagement: the chord to the next station (a half turn
                    // at most) is at most the straight pitch. Coverage: the
                    // outer-wall cusp between the two disks.
                    let chord = metallic::asin((engagement_pitch * (1. - 1e-9) / (2. * big)).min(1.));
                    let cover = arc_coverage_half_angle(big, a, t);
                    let step = 2. * chord.min(cover) * (1. - 1e-9);
                    if !step.is_finite() || step * big <= guard {
                        return Err(format!(
                            "slot {}: path piece {i} (radius {big:.4} mm) leaves no bounded advance; \
                             increase boundary_tolerance_mm or max_engagement_deg",
                            op.id
                        ));
                    }
                    step
                }
            });
        }
        let report = SlotReport {
            strategy: "trochoidal_slot",
            geometric_residual_status: ResidualStatus::Pending,
            depth_passes: passes,
            target_z_mm: op.floor_z_mm,
            width_mm: op.width_mm,
            orbit_radius_mm: radius,
            max_engagement_deg: op.max_engagement_deg,
            boundary_underclearance_bound_mm: op.boundary_tolerance_mm,
            bounded_advance_mm: pitch,
            max_bounded_engagement_deg: 0.,
            seed_entry_policy: if op.entry.is_some() {
                "plunge_through_air_then_declared_helix_on_the_orbit_circle_and_seed_orbit_at_entry_feed; seed engagement is NOT limited"
            } else {
                "plunge_only_through_air_else_refused_and_seed_orbit_at_entry_feed; seed engagement is NOT limited"
            },
            seed_orbits: 0,
            controlled_orbits: 0,
            stock_clear_advances: 0,
            stock_query_work: 0,
            stock_query_work_exhausted: false,
            plunge_proof_work: 0,
            plunge_proof_work_exhausted: false,
            path_pieces: pieces.len(),
            corner_links: 0,
            corner_arcs: 0,
            air_fed_arcs: 0,
            air_fed_mm: 0.,
            return_chords: 0,
            return_chord_saved_mm: 0.,
            helical_entries: 0,
            helix_turns: 0,
            max_air_plunge_mm: 0.,
        };
        let heights = crate::heights::Heights::resolve(
            op.clearance_z_mm,
            op.retract_z_mm,
            op.feed_z_mm,
            spec.max_mm[2],
            guard,
        )?;
        Ok(Self {
            heights,
            op,
            spec: spec.clone(),
            initial,
            position: initial,
            pieces,
            steps,
            side,
            piece: 0,
            radius,
            cutter_radius: r,
            guard,
            pass: 0,
            distance: 0.,
            phase: 0,
            next_id: first_motion_id,
            expected_generation: None,
            lift_split: None,
            query_work_left: STOCK_QUERY_WORK,
            plunge_work_left: PLUNGE_PROOF_WORK,
            prev_center: None,
            orbit_pieces: VecDeque::new(),
            pending: VecDeque::new(),
            helix: None,
            report,
        })
    }
    pub fn report(&self) -> &SlotReport {
        &self.report
    }
    pub fn finish_report(&mut self, cancel: &impl CancelSignal) -> Result<(), String> {
        if cancel.is_cancelled() {
            return Err("slot report cancelled".into());
        }
        if self.phase != 7 {
            return Err("drain slot motions before final report".into());
        }
        self.report.geometric_residual_status = ResidualStatus::Estimated;
        Ok(())
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
    fn center_on(&self, piece: usize, along: f64, z: f64) -> Vec3 {
        let p = self.pieces[piece].point(along);
        [p[0], p[1], z]
    }
    fn center(&self, along: f64, z: f64) -> Vec3 {
        self.center_on(self.piece, along, z)
    }
    /// The rho-offset of the centreline on the orbit side: where an orbit
    /// starts and ends, and the links' path.
    fn orbit_start_on(&self, piece: usize, along: f64, z: f64) -> Vec3 {
        let c = self.center_on(piece, along, z);
        let t = self.pieces[piece].tangent(along);
        let normal = [self.side * -t[1], self.side * t[0]];
        [
            c[0] + normal[0] * self.radius,
            c[1] + normal[1] * self.radius,
            z,
        ]
    }
    fn orbit_start(&self, along: f64, z: f64) -> Vec3 {
        self.orbit_start_on(self.piece, along, z)
    }
    /// The proven-air interval of the controlled orbit at `center`, as
    /// (start, length) in the orbit's own parameter t in [0, TAU) measured
    /// from the orbit start point in the orbit direction. Pure geometry, no
    /// stock query: the previous motion sequence was a full orbit at
    /// `prev_center` at the same z, then the advance link, so the previous
    /// orbit cleared the disk of radius rho + r around `prev_center`. A
    /// cutter centre P = C + rho*u(theta) keeps its whole cutter disk of
    /// radius r strictly inside that cleared disk when |P - C_prev| <=
    /// rho - g. Solving |d + rho*u(theta)| = rho - g with d = C - C_prev
    /// gives cos(theta - theta_back) = c, theta_back = atan2(-d).
    fn air_interval(&self, center: Vec3) -> Option<(f64, f64)> {
        let prev = self.prev_center?;
        let rho = self.radius;
        let g = self.guard;
        let d = [center[0] - prev[0], center[1] - prev[1]];
        let q = metallic::hypot(d[0], d[1]);
        if q <= g {
            return None;
        }
        let c = (q * q + rho * rho - (rho - g) * (rho - g)) / (2. * rho * q);
        if c >= 1. {
            return None;
        }
        let beta = metallic::acos(c) - self.op.tolerance_mm / rho;
        if beta <= 0. {
            return None;
        }
        let sign = if self.op.clockwise { -1. } else { 1. };
        let theta_start =
            metallic::atan2(self.position[1] - center[1], self.position[0] - center[0]);
        let theta_back = metallic::atan2(-d[1], -d[0]);
        // Map the interval [theta_back - beta, theta_back + beta] into the
        // orbit's own parameter t = sign*(theta - theta_start): t runs
        // against theta for clockwise orbits, so the interval starts at the
        // theta_back + beta end there and at the theta_back - beta end for
        // counter-clockwise ones.
        let start = (sign * (theta_back - sign * beta - theta_start)).rem_euclid(TAU);
        Some((if start >= TAU { start - TAU } else { start }, 2. * beta))
    }
    /// The pieces of the controlled orbit at `center`: cut arcs and, where
    /// the orbit is proven to run inside the disk the previous orbit
    /// cleared, arcs at `air_feed_mm_min`. Same centre and direction as
    /// today's single full circle, same start and end point, total sweep
    /// exactly one turn. Without a proven-air interval: one full circle
    /// (today's orbit).
    /// The return across a proven-air piece of the orbit at `center`, from
    /// the current position P to `piece.end` Q (signed sweep s): a D-shaped
    /// loop's straight return with rounded ends. A fillet of radius
    /// rf = RETURN_FILLET * rho, internally tangent to the orbit at P, turns
    /// by s/2 onto the line parallel to the chord PQ; a second fillet turns
    /// by s/2 off it, tangent to the orbit at Q. The path is tangent
    /// throughout and lies between the chord and the arc (the line is
    /// (rho - rf) cos(s/2) + rf from the centre, farther than the chord's
    /// rho cos(s/2)), so inside the convex hull of the arc, inside the disk
    /// |P - C_prev| <= rho - g that contains both: it cuts nothing.
    fn rounded_return(&self, center: Vec3, piece: OrbitPiece) -> Vec<Curve> {
        let (p, q, s) = (self.position, piece.end, piece.sweep);
        let z = p[2];
        let rho = self.radius;
        if rho * s.abs() < 64. * self.guard {
            // Too short to round: the air arc itself (tangent already).
            return vec![Curve::Arc {
                start: p,
                end: q,
                center,
                plane: Plane::Xy,
                sweep_rad: s,
            }];
        }
        let rf = RETURN_FILLET * rho;
        let at = |theta: f64, radius: f64, from: [f64; 2]| {
            let (sin, cos) = metallic::sincos(theta);
            [from[0] + radius * cos, from[1] + radius * sin, z]
        };
        let c = [center[0], center[1]];
        let theta_p = metallic::atan2(p[1] - c[1], p[0] - c[0]);
        let theta_mid = theta_p + s / 2.;
        let theta_q = theta_p + s;
        let f_p = at(theta_p, rho - rf, c);
        let f_q = at(theta_q, rho - rf, c);
        let e_p = at(theta_mid, rf, [f_p[0], f_p[1]]);
        let s_q = at(theta_mid, rf, [f_q[0], f_q[1]]);
        vec![
            Curve::Arc {
                start: p,
                end: e_p,
                center: f_p,
                plane: Plane::Xy,
                sweep_rad: s / 2.,
            },
            Curve::Line {
                start: e_p,
                end: s_q,
            },
            Curve::Arc {
                start: s_q,
                end: q,
                center: f_q,
                plane: Plane::Xy,
                sweep_rad: s / 2.,
            },
        ]
    }
    fn split_orbit(&self, center: Vec3, z: f64) -> VecDeque<OrbitPiece> {
        let sign = if self.op.clockwise { -1. } else { 1. };
        let rho = self.radius;
        // Contiguous t-spans of the orbit parameter with their feed: air on
        // the proven-air interval, cut feed elsewhere.
        let mut spans: Vec<(f64, f64, bool)> = match self.air_interval(center) {
            Some((t1, len)) if t1 + len <= TAU => {
                let mut spans = Vec::new();
                if t1 > 0. {
                    spans.push((0., t1, false));
                }
                spans.push((t1, t1 + len, true));
                if t1 + len < TAU {
                    spans.push((t1 + len, TAU, false));
                }
                spans
            }
            Some((t1, len)) => {
                // The interval wraps through the orbit start point.
                let t2 = t1 + len - TAU;
                let mut spans = Vec::new();
                if t2 > 0. {
                    spans.push((0., t2, true));
                }
                spans.push((t2, t1, false));
                spans.push((t1, TAU, true));
                spans
            }
            None => Vec::new(),
        };
        if spans.is_empty() {
            spans.push((0., TAU, false));
        }
        // Fold pieces shorter than the guard into a neighbour at the cut
        // feed; the air arc is never extended (a folded neighbour runs at
        // the cut feed, so air only ever shrinks).
        while let Some((i, len)) = spans
            .iter()
            .enumerate()
            .map(|(i, &(a, b, _))| (i, rho * (b - a)))
            .min_by(|a, b| a.1.total_cmp(&b.1))
        {
            if spans.len() == 1 || len >= self.guard {
                break;
            }
            let j = if i > 0 && !spans[i - 1].2 {
                i - 1
            } else if i + 1 < spans.len() && !spans[i + 1].2 {
                i + 1
            } else if i > 0 {
                i - 1
            } else {
                i + 1
            };
            let folded = (
                spans[i].0.min(spans[j].0),
                spans[i].1.max(spans[j].1),
                false,
            );
            spans[j] = folded;
            spans.remove(i);
            // A fold can bring two cut-feed spans together.
            let mut merged: Vec<(f64, f64, bool)> = Vec::with_capacity(spans.len());
            for span in spans {
                match merged.last_mut() {
                    Some(last) if last.2 == span.2 => last.1 = span.1,
                    _ => merged.push(span),
                }
            }
            spans = merged;
        }
        debug_assert!(
            spans.len() != 1 || !spans[0].2,
            "a whole orbit is never proven air"
        );
        let theta_start =
            metallic::atan2(self.position[1] - center[1], self.position[0] - center[0]);
        let mut pieces = VecDeque::with_capacity(spans.len());
        let count = spans.len();
        for (k, &(t0, t1, air)) in spans.iter().enumerate() {
            // Exact end points: the last piece ends at the orbit start
            // point, not at a recomputed one.
            let end = if k + 1 == count {
                self.position
            } else {
                let theta = theta_start + sign * t1;
                let (sin, cos) = metallic::sincos(theta);
                [center[0] + rho * cos, center[1] + rho * sin, z]
            };
            pieces.push_back(OrbitPiece {
                end,
                sweep: sign * (t1 - t0),
                air,
            });
        }
        pieces
    }
    fn emit(&mut self, curve: Curve, kind: MoveKind, entry: bool) -> Result<Motion, String> {
        curve.prepare(self.op.tolerance_mm)?;
        let m = Motion {
            id: self.next_id,
            operation: self.op.id.clone(),
            tool: self.op.tool.clone(),
            kind,
            curve,
            feed_mm_min: if kind == MoveKind::Rapid {
                None
            } else {
                Some(if entry {
                    self.op.entry_feed_mm_min
                } else {
                    self.op.feed_mm_min
                })
            },
        };
        self.next_id += 1;
        self.position = m.curve.end();
        Ok(m)
    }
    /// Split the straight-up lift after each pass where the stock beside the
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
            return Err("commit preceding slot motion to matching stock before planning".into());
        }
        loop {
            if cancel.is_cancelled() {
                return Err("slot planning cancelled".into());
            }
            let z = (self.op.top_z_mm - (self.pass + 1) as f64 * self.op.stepdown_mm)
                .max(self.op.floor_z_mm);
            let (curve, kind, entry, next_phase) = match self.phase {
                0 => (
                    Curve::Line {
                        start: self.position,
                        end: [self.position[0], self.position[1], self.op.clearance_z_mm],
                    },
                    MoveKind::Rapid,
                    false,
                    1,
                ),
                // Travel at the current rapid height, then descend to the
                // feed height (phase 9) before plunging.
                1 => (
                    Curve::Line {
                        start: self.position,
                        end: self.orbit_start(0., self.position[2]),
                    },
                    MoveKind::Rapid,
                    false,
                    9,
                ),
                9 => (
                    Curve::Line {
                        start: self.position,
                        end: self.orbit_start(0., self.heights.feed),
                    },
                    MoveKind::Rapid,
                    false,
                    2,
                ),
                // Plunge through air only: down to the highest stock the
                // entry's footprint (the seed orbit's disk, radius rho + r)
                // can touch, then the declared helix (phase 10); without
                // one, stock above the pass is refused. Below the first
                // pass the previous pass's seed orbit at this same centre
                // swept exactly that disk down to its own depth, so the
                // plunge reaches at least that level (the grid's whole
                // cells at the disk's rim would hide it).
                2 => {
                    let c = self.center(0., z);
                    let previous = self.op.top_z_mm - self.pass as f64 * self.op.stepdown_mm;
                    let top = stock
                        .top_under_disk([c[0], c[1]], self.radius + self.cutter_radius)
                        .min(previous);
                    let air = top.max(z);
                    // A straight plunge to the pass depth only where it is
                    // proven to cut nothing: no cell's upper bound under the
                    // disk stands above the pass (a conservative bound), or
                    // the committed cutter sweeps prove the cutter's disk at
                    // the plunge's end clear from the pass depth up (the flat
                    // air-move proof, footprint by footprint from the
                    // recorded sweeps, where a floor cut to exactly this
                    // depth may be touched). A cut to this depth leaves its
                    // cells' upper bounds a sweep's rounding above it, and a
                    // cut elsewhere in the same cell proves nothing here. Any
                    // other stock above the pass, however thin, takes the
                    // helix or is refused. The proofs of all passes share one
                    // work budget: out of it, a pass is not proven.
                    let end = self.orbit_start(0., z);
                    let at_end = Motion {
                        id: self.next_id,
                        operation: self.op.id.clone(),
                        tool: self.op.tool.clone(),
                        kind: MoveKind::Air,
                        curve: Curve::Line { start: end, end },
                        feed_mm_min: Some(self.op.entry_feed_mm_min),
                    };
                    let proven = air <= z || {
                        let before = self.plunge_work_left;
                        let proven = if self.plunge_work_left > 0 {
                            stock.flat_history_clearance(
                                &at_end,
                                self.cutter_radius,
                                &mut self.plunge_work_left,
                                cancel,
                            )
                        } else {
                            Ok(false)
                        };
                        // Counted before a cancelled proof returns: the plan's
                        // budget is spent either way.
                        self.report.plunge_proof_work += (before - self.plunge_work_left) as usize;
                        self.report.plunge_proof_work_exhausted |= self.plunge_work_left == 0;
                        proven?
                    };
                    let helix_top = if proven {
                        z
                    } else if let Some(h) = helix(&self.op) {
                        let turns = h.turns(self.radius, air - z).map_err(|e| {
                            format!("slot {}: depth pass {}: {e}", self.op.id, self.pass + 1)
                        })?;
                        self.helix = Some(HelixDescent {
                            top: air,
                            turns,
                            done: 0,
                        });
                        air
                    } else if self.plunge_work_left == 0 {
                        return Err(format!(
                            "slot {}: depth pass {}: the stock up to Z{:.4} under the orbit's disk at \
                             ({:.4}, {:.4}) is not proven clear of a plunge to Z{:.4}: the plunge proofs' \
                             shared work budget ({PLUNGE_PROOF_WORK} visits) is spent; declare \
                             entry=HelicalEntry(angle) to helix down on the orbit circle, or use fewer, \
                             deeper passes",
                            self.op.id,
                            self.pass + 1,
                            top,
                            c[0],
                            c[1],
                            z
                        ));
                    } else {
                        return Err(format!(
                            "slot {}: depth pass {} would plunge up to {:.4} mm into stock at ({:.4}, {:.4}): \
                             the stock cells under the orbit's disk reach Z{:.4} over the pass at Z{:.4}, \
                             and no committed cut proves the plunge's footprint clear (a cut elsewhere in \
                             a cell does not, nor one to the pass depth that the cells round above it); \
                             declare entry=HelicalEntry(angle) to helix down on the orbit circle, or, if \
                             this stock is already cut to the pass depth, use a finer stock cell_size_mm",
                            self.op.id,
                            self.pass + 1,
                            air - z,
                            c[0],
                            c[1],
                            top,
                            z
                        ));
                    };
                    self.report.max_air_plunge_mm = self
                        .report
                        .max_air_plunge_mm
                        .max(self.op.top_z_mm - helix_top);
                    (
                        Curve::Line {
                            start: self.position,
                            end: self.orbit_start(0., helix_top),
                        },
                        MoveKind::Plunge,
                        true,
                        if self.helix.is_some() { 10 } else { 3 },
                    )
                }
                // One helix turn per motion on the orbit circle, at the
                // entry's feed, ending at the orbit start at the pass depth.
                10 => {
                    let mut d = self.helix.expect("helix planned in phase 2");
                    d.done += 1;
                    let next = if d.done == d.turns {
                        z
                    } else {
                        d.top - (d.top - z) * d.done as f64 / d.turns as f64
                    };
                    let center = self.center(0., self.position[2]);
                    let curve = Curve::Helix {
                        start: self.position,
                        end: [self.position[0], self.position[1], next],
                        center,
                        plane: Plane::Xy,
                        sweep_rad: if self.op.clockwise { -TAU } else { TAU },
                    };
                    let mut motion = self.emit(curve, MoveKind::Plunge, true)?;
                    if let Some(feed) = helix(&self.op).and_then(|h| h.feed_mm_min) {
                        motion.feed_mm_min = Some(feed);
                    }
                    self.report.helix_turns += 1;
                    if d.done == d.turns {
                        self.helix = None;
                        self.report.helical_entries += 1;
                        self.phase = 3;
                    } else {
                        self.helix = Some(d);
                    }
                    self.expected_generation = Some(stock.generation() + 1);
                    return Ok(Some(motion));
                }
                3 => {
                    let center = self.center(self.distance, z);
                    let curve = Curve::Arc {
                        start: self.position,
                        end: self.position,
                        center,
                        plane: Plane::Xy,
                        sweep_rad: if self.op.clockwise { -TAU } else { TAU },
                    };
                    let motion = self.emit(curve, MoveKind::Cut, true)?;
                    self.report.seed_orbits += 1;
                    self.prev_center = Some(center);
                    self.phase = 4;
                    self.expected_generation = Some(stock.generation() + 1);
                    return Ok(Some(motion));
                }
                // A controlled orbit: plan its pieces once (one full circle
                // without a proven-air interval), then emit one per call;
                // each returned motion is committed before the next call.
                5 => {
                    let center = self.center(self.distance, z);
                    if self.pending.is_empty() {
                        if self.orbit_pieces.is_empty() {
                            self.orbit_pieces = self.split_orbit(center, z);
                            self.report.controlled_orbits += 1;
                        }
                        let piece = self.orbit_pieces.pop_front().unwrap();
                        if piece.air {
                            let moves = self.rounded_return(center, piece);
                            // A return too short to round is the air arc
                            // itself: no chord, nothing saved.
                            if moves.iter().any(|c| matches!(c, Curve::Line { .. })) {
                                let length: f64 = moves
                                    .iter()
                                    .map(|c| c.prepare(self.op.tolerance_mm).map(|p| p.length_mm()))
                                    .sum::<Result<f64, String>>()?;
                                self.report.return_chords += 1;
                                self.report.return_chord_saved_mm +=
                                    self.radius * piece.sweep.abs() - length;
                            }
                            self.pending.extend(moves.into_iter().map(|c| (c, true)));
                        } else {
                            self.pending.push_back((
                                Curve::Arc {
                                    start: self.position,
                                    end: piece.end,
                                    center,
                                    plane: Plane::Xy,
                                    sweep_rad: piece.sweep,
                                },
                                false,
                            ));
                        }
                    }
                    let (curve, air) = self.pending.pop_front().unwrap();
                    let last = self.pending.is_empty() && self.orbit_pieces.is_empty();
                    let mut motion = self.emit(curve, MoveKind::Cut, false)?;
                    if air && let Some(feed) = self.op.air_feed_mm_min {
                        motion.kind = MoveKind::Air;
                        motion.feed_mm_min = Some(feed);
                        self.report.air_fed_arcs +=
                            usize::from(matches!(motion.curve, Curve::Arc { .. }));
                        self.report.air_fed_mm +=
                            motion.curve.prepare(self.op.tolerance_mm)?.length_mm();
                    }
                    if last {
                        self.phase = 4;
                        self.prev_center = Some(center);
                    }
                    self.expected_generation = Some(stock.generation() + 1);
                    return Ok(Some(motion));
                }
                4 => {
                    let piece = self.pieces[self.piece];
                    let extent = piece.extent();
                    if self.distance == extent {
                        if self.piece + 1 == self.pieces.len() {
                            self.phase = 6;
                            continue;
                        }
                        // A vertex: its orbit is done. Cross to the next
                        // piece's offset inside that orbit's disk.
                        let vertex = self.center(self.distance, z);
                        let next = self.orbit_start_on(self.piece + 1, 0., z);
                        self.piece += 1;
                        self.distance = 0.;
                        if distance(self.position, next) <= self.op.tolerance_mm {
                            continue;
                        }
                        // The vertex orbit ends at the first offset heading
                        // along the incoming piece. Where the slot turns away
                        // from the link side (an outer corner), the orbit's
                        // own circle, in the orbit's sense, reaches the second
                        // offset heading along the outgoing piece after the
                        // turn angle: a link tangent at both ends that sweeps
                        // nothing the orbit did not. Where it turns toward the
                        // link side, that circle needs 2*pi minus the turn
                        // (a second orbit), so the link stays the straight
                        // chord, inside the same disk.
                        let angle = |p: Vec3| metallic::atan2(p[1] - vertex[1], p[0] - vertex[0]);
                        let (a0, a1) = (angle(self.position), angle(next));
                        let sweep_rad = if self.op.clockwise {
                            -(a0 - a1).rem_euclid(TAU)
                        } else {
                            (a1 - a0).rem_euclid(TAU)
                        };
                        let curve = if sweep_rad.abs() <= std::f64::consts::PI {
                            self.report.corner_arcs += 1;
                            Curve::Arc {
                                start: self.position,
                                end: next,
                                center: vertex,
                                plane: Plane::Xy,
                                sweep_rad,
                            }
                        } else {
                            Curve::Line {
                                start: self.position,
                                end: next,
                            }
                        };
                        let motion = self.emit(curve, MoveKind::Link, false)?;
                        self.report.corner_links += 1;
                        self.expected_generation = Some(stock.generation() + 1);
                        return Ok(Some(motion));
                    }
                    let remaining = extent - self.distance;
                    let mut advance = self.steps[self.piece].min(remaining);
                    let mut known_clear = false;
                    // Stock already clear along the rest of the piece (a
                    // line, or an arc of the centreline): one link.
                    if remaining > advance && !self.report.stock_query_work_exhausted {
                        let rest = match piece {
                            Piece::Line { .. } => Curve::Line {
                                start: self.center(self.distance, z),
                                end: self.center(extent, z),
                            },
                            Piece::Arc { center, sweep, .. } => Curve::Arc {
                                start: self.center(self.distance, z),
                                end: self.center(extent, z),
                                center: [center[0], center[1], z],
                                plane: Plane::Xy,
                                sweep_rad: sweep.signum() * remaining,
                            },
                        };
                        let query = Motion {
                            id: self.next_id,
                            operation: self.op.id.clone(),
                            tool: self.op.tool.clone(),
                            kind: MoveKind::Link,
                            curve: rest,
                            feed_mm_min: Some(self.op.feed_mm_min),
                        };
                        let mut work = self.query_work_left;
                        let before = work;
                        known_clear = !stock.may_remove_material(
                            &query,
                            self.radius + self.cutter_radius,
                            0.,
                            self.op.tolerance_mm,
                            &mut work,
                            cancel,
                        )?;
                        self.query_work_left = work;
                        self.report.stock_query_work += before - work;
                        self.report.stock_query_work_exhausted |= work == 0;
                        if known_clear {
                            advance = remaining;
                            // The stock-clear fast path leaves the previous
                            // orbit's disk behind: the next orbit stays one
                            // full circle.
                            self.prev_center = None;
                        }
                    }
                    // Choose exact endpoint when consuming the remainder; no cumulative overshoot.
                    let next = if advance == remaining {
                        extent
                    } else {
                        self.distance + advance
                    };
                    let end = self.orbit_start(next, z);
                    let (link, chord) = match piece {
                        Piece::Line { .. } => (
                            Curve::Line {
                                start: self.position,
                                end,
                            },
                            advance,
                        ),
                        Piece::Arc { center, sweep, .. } => (
                            Curve::Arc {
                                start: self.position,
                                end,
                                center: [center[0], center[1], z],
                                plane: Plane::Xy,
                                sweep_rad: sweep.signum() * (next - self.distance),
                            },
                            distance(self.center(self.distance, z), self.center(next, z)),
                        ),
                    };
                    let motion = self.emit(link, MoveKind::Link, false)?;
                    if known_clear {
                        self.report.stock_clear_advances += 1;
                    } else {
                        let r = self.cutter_radius;
                        let a = self.radius + r;
                        let q = self.radius + chord + self.guard;
                        let phi = 2.
                            * ((a * a - q * q - r * r) / (2. * q * r))
                                .clamp(-1., 1.)
                                .acos()
                                .to_degrees();
                        self.report.max_bounded_engagement_deg =
                            self.report.max_bounded_engagement_deg.max(phi);
                    }
                    self.distance = next;
                    self.phase = 5;
                    self.expected_generation = Some(stock.generation() + 1);
                    return Ok(Some(motion));
                }
                6 => {
                    // Fed only through the stock beside the tool, then the
                    // rest of the way as its own retract (#441).
                    let top = crate::lift::feed_to(
                        self.lift_split.as_ref(),
                        stock,
                        self.position,
                        self.heights.feed,
                    );
                    (
                        Curve::Line {
                            start: self.position,
                            end: [self.position[0], self.position[1], top],
                        },
                        MoveKind::Retract,
                        false,
                        if top < self.heights.feed { 6 } else { 8 },
                    )
                }
                // Feed height -> retract height; clearance after the last pass.
                8 => {
                    let last = self.pass == self.report.depth_passes;
                    let z = if last {
                        self.heights.clearance
                    } else {
                        self.heights.retract
                    };
                    (
                        Curve::Line {
                            start: self.position,
                            end: [self.position[0], self.position[1], z],
                        },
                        MoveKind::Rapid,
                        false,
                        if last { 7 } else { 1 },
                    )
                }
                _ => return Ok(None),
            };
            if curve.start() == curve.end() {
                self.phase = next_phase;
                continue;
            }
            let motion = self.emit(curve, kind, entry)?;
            if self.phase == 6 && next_phase != 6 {
                self.pass += 1;
                self.piece = 0;
                self.distance = 0.;
                // A new depth pass: no previous orbit at this z.
                self.prev_center = None;
                self.phase = next_phase;
            } else {
                self.phase = next_phase;
            }
            self.expected_generation = Some(stock.generation() + 1);
            return Ok(Some(motion));
        }
    }
}
