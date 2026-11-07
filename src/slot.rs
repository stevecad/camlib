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
use std::f64::consts::TAU;

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
    /// Lines and arcs of the centreline (1 for a straight slot).
    pub path_pieces: usize,
    /// Straight links across corners, inside the vertex orbit's disk.
    pub corner_links: usize,
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
    (a[0] - b[0]).hypot(a[1] - b[1])
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
                    [center[0] + radius * t.cos(), center[1] + radius * t.sin()]
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
                [-sweep.signum() * t.sin(), sweep.signum() * t.cos()]
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
        let chord = (b[0] - a[0]).hypot(b[1] - a[1]);
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
        let sweep = 4. * bulge.atan();
        let radius = chord / (2. * (sweep / 2.).sin().abs());
        if radius > 1e6 {
            return Err(format!(
                "path piece {i}: bulge {bulge} is too small for an arc; use a line (bulge 0)"
            ));
        }
        // The centre is on the chord's bisector, left of it for a
        // counter-clockwise arc.
        let h = chord / 2. / (sweep / 2.).tan();
        let center = [
            (a[0] + b[0]) / 2. - dir[1] * h,
            (a[1] + b[1]) / 2. + dir[0] * h,
        ];
        out.push(Piece::Arc {
            a,
            b,
            center,
            radius,
            start: (a[1] - center[1]).atan2(a[0] - center[0]),
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
        let s = radius * alpha.sin();
        a - (a * a - s * s).max(0.).sqrt() + 2. * radius * (alpha / 2.).sin().powi(2)
    };
    let mut lo = 0f64;
    let mut hi = std::f64::consts::FRAC_PI_2.min((a / radius).min(1.).asin());
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
    report: SlotReport,
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
                    format!(
                        "slot depth {} exceeds the cutting length {}",
                        op.top_z_mm - op.floor_z_mm,
                        cutter.cutting_length_mm
                    ),
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
        let engagement_pitch =
            (a * a - (r * half_phi.sin()).powi(2)).sqrt() - r * half_phi.cos() - radius - guard;
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
                    let chord = (engagement_pitch * (1. - 1e-9) / (2. * big)).min(1.).asin();
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
            seed_entry_policy: "center_cutting_plunge_and_seed_orbit_at_entry_feed; seed engagement is NOT limited",
            seed_orbits: 0,
            controlled_orbits: 0,
            stock_clear_advances: 0,
            stock_query_work: 0,
            stock_query_work_exhausted: false,
            path_pieces: pieces.len(),
            corner_links: 0,
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
                2 => (
                    Curve::Line {
                        start: self.position,
                        end: self.orbit_start(0., z),
                    },
                    MoveKind::Plunge,
                    true,
                    3,
                ),
                3 | 5 => {
                    let seed = self.phase == 3;
                    let curve = Curve::Arc {
                        start: self.position,
                        end: self.position,
                        center: self.center(self.distance, z),
                        plane: Plane::Xy,
                        sweep_rad: if self.op.clockwise { -TAU } else { TAU },
                    };
                    let motion = self.emit(curve, MoveKind::Cut, seed)?;
                    if seed {
                        self.report.seed_orbits += 1;
                    } else {
                        self.report.controlled_orbits += 1;
                    }
                    self.phase = 4;
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
                        let next = self.orbit_start_on(self.piece + 1, 0., z);
                        self.piece += 1;
                        self.distance = 0.;
                        if distance(self.position, next) <= self.op.tolerance_mm {
                            continue;
                        }
                        let motion = self.emit(
                            Curve::Line {
                                start: self.position,
                                end: next,
                            },
                            MoveKind::Link,
                            false,
                        )?;
                        self.report.corner_links += 1;
                        self.expected_generation = Some(stock.generation() + 1);
                        return Ok(Some(motion));
                    }
                    let remaining = extent - self.distance;
                    let mut advance = self.steps[self.piece].min(remaining);
                    let mut known_clear = false;
                    // Stock already clear along the rest of a line: one link.
                    if matches!(piece, Piece::Line { .. })
                        && remaining > advance
                        && !self.report.stock_query_work_exhausted
                    {
                        let query = Motion {
                            id: self.next_id,
                            operation: self.op.id.clone(),
                            tool: self.op.tool.clone(),
                            kind: MoveKind::Link,
                            curve: Curve::Line {
                                start: self.center(self.distance, z),
                                end: self.center(extent, z),
                            },
                            feed_mm_min: Some(self.op.feed_mm_min),
                        };
                        let mut work = usize::MAX;
                        let before = work;
                        known_clear = !stock.may_remove_material(
                            &query,
                            self.radius + self.cutter_radius,
                            0.,
                            self.op.tolerance_mm,
                            &mut work,
                            cancel,
                        )?;
                        self.report.stock_query_work += before - work;
                        self.report.stock_query_work_exhausted |= work == 0;
                        if known_clear {
                            advance = remaining;
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
                6 => (
                    Curve::Line {
                        start: self.position,
                        end: [self.position[0], self.position[1], self.heights.feed],
                    },
                    MoveKind::Retract,
                    false,
                    8,
                ),
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
            if self.phase == 6 {
                self.pass += 1;
                self.piece = 0;
                self.distance = 0.;
                self.phase = next_phase;
            } else {
                self.phase = next_phase;
            }
            self.expected_generation = Some(stock.generation() + 1);
            return Ok(Some(motion));
        }
    }
}
