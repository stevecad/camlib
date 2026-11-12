//! Automatic nesting of sheet parts for thermal cutting (plasma, laser).
//!
//! A nest places copies of parts (each an outline with holes) on a sheet (an
//! outline with remnant holes) and returns their placements and the placed
//! parts as one planar region, ready to be the `parts` of a thermal cut.
//! Nothing about the machine is assumed: the sheet, the kerf, the leads, the
//! spacing between parts, the margin to the sheet's edge, the rotations each
//! part may take and every quantity are declared by the caller.
//!
//! # What a layout guarantees
//!
//! A thermal cut runs the beam center half a kerf plus an allowance (the
//! parts' boundary error and half the boundary tolerance) outside each part,
//! and its leads and pierce craters reach a bounded distance further into
//! the scrap (`lead_reach_mm`, from the declared leads). So everything cut
//! for a part lies within `kerf + allowance + lead reach` of it. A layout
//! keeps
//!
//! - any two parts `2 (kerf + allowance + lead reach) + part_spacing_mm`
//!   apart, so the kerfs, leads and pierce craters of two parts stay
//!   `part_spacing_mm` apart wherever the cut starts its loops (but see the
//!   center pierce under Limits);
//! - every part `kerf + allowance + lead reach + edge_margin_mm` (plus the
//!   sheet's boundary error) from the sheet's outline and holes.
//!
//! Both are proven on the finished layout by the independent check in
//! [`check`], which works on the exact lines and arcs and trusts nothing of
//! the planner. A layout that fails the check is refused, never returned.
//!
//! # Placement
//!
//! Bottom-left fill (Burke, Hellier, Kendall and Whitwell, "A new
//! bottom-left-fill heuristic algorithm for the two-dimensional irregular
//! packing problem", Operations Research 54(3), 2006): parts are placed one
//! at a time, largest area first, each at the lowest free position of the
//! leftmost column that has one. Columns are `resolution_mm` apart, as in
//! the paper; the lowest free position of a column is exact for the
//! planning outlines: the positions where the part would overlap a placed
//! part are a slice of their no-fit polygon (Bennell and Oliveira, "The
//! geometry of nesting problems: a tutorial", European Journal of
//! Operational Research 184(2), 2008), computed here as the union of
//! intervals of the convex pieces of both (the decomposition approach to the
//! no-fit polygon in the same tutorial, with trapezoids as the pieces).
//! Of a part's allowed rotations (and its mirror image, where allowed) the
//! one that ends furthest left is taken. A part falls into a hole of a
//! placed part, or of the sheet's free area, whenever that is the lowest
//! free position.
//!
//! An optional local search (`search_iterations`, `seed`) exchanges two
//! parts of the placement order and keeps the exchange when the nest gets
//! better, as the paper's hill climbing does. It is bounded by iterations,
//! not time, so the same request gives the same nest.
//!
//! # Limits
//!
//! This is a heuristic: it does not find the densest nest and makes no claim
//! about how close it comes. Columns are `resolution_mm` apart, and curved
//! planning outlines stand up to a quarter of that outside the required
//! clearance. Room for leads is reserved all round every part rather than
//! where the cut will put them. In a hole too small for its leads the cut
//! pierces at the hole's center, further from the loop than the reserved
//! reach: a part nested in such a hole keeps the part distance to the hole's
//! edge, but its spacing to that pierce is not part of the nest's proof (the
//! cut proves the pierce and lead clear of the part, or refuses). One sheet
//! is filled; parts that do not fit
//! are refused, or reported when `allow_unplaced` is set, never dropped.
//! Rotations are the declared angles only.
use crate::{
    planar::{Boundary, Component, ConstructionOptions, PlanarRegion},
    stock::CancelSignal,
    thermal::Lead,
};
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

pub mod check;
pub use check::NestCheck;

type P2 = [f64; 2];

/// A part to place: one outline with holes.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NestPart {
    pub region: PlanarRegion,
    /// How many copies to place.
    pub quantity: u32,
    /// The rotations the part may take, degrees counter-clockwise.
    pub rotations_deg: Vec<f64>,
    /// Whether the part may also be placed mirrored (turned over).
    #[serde(default)]
    pub mirror: bool,
}

fn yes() -> bool {
    true
}
fn boundary_tolerance() -> f64 {
    0.01
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NestRequest {
    /// The sheet: one outline, with holes where a remnant is already cut.
    pub sheet: PlanarRegion,
    pub parts: Vec<NestPart>,
    /// The kerf width: the beam tool's diameter in the thermal cut.
    pub kerf_mm: f64,
    /// The leads of the thermal cut that will cut the nest.
    pub lead_in: Lead,
    pub lead_out: Lead,
    /// As in the thermal cut. Default: the kerf width.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pierce_clearance_mm: Option<f64>,
    /// Material left between what is cut for two parts.
    pub part_spacing_mm: f64,
    /// Material left between what is cut for a part and the sheet's edge.
    pub edge_margin_mm: f64,
    /// The column spacing of the placement.
    pub resolution_mm: f64,
    /// Whether a part may be placed in a hole of another part.
    #[serde(default = "yes")]
    pub part_in_part: bool,
    /// Return a partial nest listing the parts that did not fit, instead of
    /// refusing.
    #[serde(default)]
    pub allow_unplaced: bool,
    /// Exchanges of the placement order to try.
    #[serde(default)]
    pub search_iterations: u32,
    #[serde(default)]
    pub seed: u64,
    /// As in the thermal cut that will cut the nest.
    #[serde(default = "boundary_tolerance")]
    pub boundary_tolerance_mm: f64,
    pub tolerance_mm: f64,
}

/// Where one copy of a part lies: a point p of the part moves to
/// `R(rotation_deg) * M(p) + translation_mm`, M mirroring x when `mirrored`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Placement {
    /// Index into the request's parts.
    pub part: usize,
    /// Which copy of the part (0-based).
    pub copy: u32,
    pub rotation_deg: f64,
    pub mirrored: bool,
    pub translation_mm: [f64; 2],
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Unplaced {
    pub part: usize,
    pub quantity: u32,
}

#[derive(Clone, Debug, Serialize)]
pub struct NestResult {
    pub strategy: &'static str,
    /// One per placed copy, in the order of `region`'s components.
    pub placements: Vec<Placement>,
    /// Parts that did not fit (only with `allow_unplaced`).
    pub unplaced: Vec<Unplaced>,
    /// The placed parts: the `parts` of the thermal cut.
    pub region: PlanarRegion,
    pub sheet_area_mm2: f64,
    pub placed_area_mm2: f64,
    /// Placed part area over sheet area.
    pub utilisation: f64,
    /// The box around the placed parts `[min x, min y, max x, max y]`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub used_bounds_mm: Option<[f64; 4]>,
    /// How far leads and pierce craters may reach beyond a kerf's edge.
    pub lead_reach_mm: f64,
    /// The independent check of this layout.
    pub check: NestCheck,
    pub search_iterations: u32,
    pub search_improvements: u32,
    pub diagnostics: Vec<String>,
}

/// The distances a layout must keep.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Clearances {
    lead_reach_mm: f64,
    part_distance_mm: f64,
    sheet_distance_mm: f64,
}

/// Validates the request's numbers and derives the distances to keep.
fn clearances(request: &NestRequest) -> Result<Clearances, String> {
    let lead_ok = |l: &Lead| {
        l.length_mm.is_finite() && l.radius_mm.is_finite() && l.length_mm >= 0. && l.radius_mm >= 0.
    };
    let within = |v: f64, max: f64| v.is_finite() && (0. ..=max).contains(&v);
    crate::heights::require(
        "nest",
        [
            (
                request.kerf_mm.is_finite() && request.kerf_mm > 0. && request.kerf_mm <= 1e3,
                format!("kerf_mm {} must be positive (the beam tool's kerf width)", request.kerf_mm),
            ),
            (
                lead_ok(&request.lead_in) && lead_ok(&request.lead_out),
                "lead lengths and radii must not be negative".into(),
            ),
            (
                request.pierce_clearance_mm.is_none_or(|c| within(c, 1e3)),
                "pierce_clearance_mm must not be negative".into(),
            ),
            (
                within(request.part_spacing_mm, 1e4) && within(request.edge_margin_mm, 1e4),
                "part_spacing_mm and edge_margin_mm must not be negative".into(),
            ),
            (
                request.resolution_mm.is_finite() && (1e-3..=1e4).contains(&request.resolution_mm),
                format!("resolution_mm {} must be within 0.001..10000", request.resolution_mm),
            ),
            (
                (1e-7..=0.01).contains(&request.tolerance_mm)
                    && (0.0001..=0.1).contains(&request.boundary_tolerance_mm),
                "tolerance_mm must be within 1e-7..0.01 and boundary_tolerance_mm within 0.0001..0.1".into(),
            ),
            (!request.parts.is_empty(), "there are no parts to place".into()),
            (
                request.sheet.components.len() == 1,
                format!(
                    "the sheet must be one outline (with holes); it has {} components",
                    request.sheet.components.len()
                ),
            ),
        ],
    )?;
    for (i, part) in request.parts.iter().enumerate() {
        crate::heights::require(
            "nest",
            [
                (
                    part.region.components.len() == 1,
                    format!(
                        "part {i} must be one outline (with holes); it has {} components: declare each as its own part",
                        part.region.components.len()
                    ),
                ),
                (
                    part.quantity > 0,
                    format!("part {i} needs a quantity of at least 1"),
                ),
                (
                    !part.rotations_deg.is_empty()
                        && part
                            .rotations_deg
                            .iter()
                            .all(|a| a.is_finite() && a.abs() <= 360.),
                    format!(
                        "part {i} needs its allowed rotations_deg (each within -360..360; [0] keeps it as drawn)"
                    ),
                ),
            ],
        )?;
    }
    let kerf = request.kerf_mm;
    // The cut's offset beyond half a kerf (thermal.rs: the parts' boundary
    // error plus half the boundary tolerance).
    let error = request
        .parts
        .iter()
        .map(|p| p.region.boundary_error_mm)
        .fold(0., f64::max);
    let allowance = error + request.boundary_tolerance_mm / 2.;
    // A lead starts where its loop starts: the pierce point lies its arc
    // radius along the loop's tangent and radius + length along its normal
    // from there (thermal.rs), and the crater reaches the pierce clearance
    // beyond half a kerf round it. Leads shortened to fit a hole come
    // closer to the loop. The one exception is the cut's center pierce in a
    // hole too small for its leads: it stays inside that hole, where only a
    // part nested in the same hole could meet it, and the cut proves it
    // clear of every part or refuses.
    let reach = |l: &Lead| l.radius_mm.hypot(l.radius_mm + l.length_mm);
    let lead_reach_mm = (reach(&request.lead_in) + request.pierce_clearance_mm.unwrap_or(kerf))
        .max(reach(&request.lead_out));
    let cut = kerf + allowance + lead_reach_mm;
    Ok(Clearances {
        lead_reach_mm,
        part_distance_mm: 2. * cut + request.part_spacing_mm,
        sheet_distance_mm: cut + request.edge_margin_mm + request.sheet.boundary_error_mm,
    })
}

/// (cos, sin) of a rotation; exact for quarter turns.
pub(crate) fn rotation(deg: f64) -> (f64, f64) {
    let quarter = deg.rem_euclid(360.) / 90.;
    if quarter == 0. {
        (1., 0.)
    } else if quarter == 1. {
        (0., 1.)
    } else if quarter == 2. {
        (-1., 0.)
    } else if quarter == 3. {
        (0., -1.)
    } else {
        let a = deg.to_radians();
        (a.cos(), a.sin())
    }
}

/// A boundary vertex of a part, placed.
pub(crate) fn apply(mirror: bool, (c, s): (f64, f64), t: P2, v: [f64; 3]) -> [f64; 3] {
    let (x, bulge) = if mirror { (-v[0], -v[2]) } else { (v[0], v[2]) };
    [c * x - s * v[1] + t[0], s * x + c * v[1] + t[1], bulge]
}

fn sweep(bulge: f64) -> f64 {
    4. * bulge.atan()
}

/// The signed area of a line/arc loop (positive counter-clockwise).
fn loop_area(b: &Boundary) -> f64 {
    let v = &b.vertices;
    let mut area = 0.;
    for i in 0..v.len() {
        let (p, q) = (v[i], v[(i + 1) % v.len()]);
        area += (p[0] * q[1] - q[0] * p[1]) / 2.;
        if p[2] != 0. {
            // The circular segment between the chord and its arc.
            let angle = sweep(p[2]);
            let r = (q[0] - p[0]).hypot(q[1] - p[1]) / (2. * (angle / 2.).sin());
            area += r * r * (angle - angle.sin()) / 2.;
        }
    }
    area
}

fn component_area(c: &Component) -> f64 {
    loop_area(&c.outer).abs() - c.holes.iter().map(|h| loop_area(h).abs()).sum::<f64>()
}

/// A polygon round a loop for planning: with `enclose` it contains the
/// loop's material (arcs bulging out of the material get tangent polylines,
/// arcs bulging into it get chords), without it the material contains the
/// polygon. Either stands at most `deviation` off the arcs. The polygon runs
/// with the material on its left.
fn polygon(b: &Boundary, outer: bool, enclose: bool, deviation: f64) -> Vec<P2> {
    let n = b.vertices.len();
    // Material on the left: outlines counter-clockwise, holes clockwise.
    let forward = (loop_area(b) > 0.) == outer;
    let vertex = |k: usize| -> [f64; 3] {
        if forward {
            b.vertices[k]
        } else {
            // Reversed: edge k runs back along edge n - 2 - k.
            let p = b.vertices[n - 1 - k];
            [p[0], p[1], -b.vertices[(2 * n - 2 - k) % n][2]]
        }
    };
    let mut points = vec![];
    for k in 0..n {
        let (p, q) = (vertex(k), vertex((k + 1) % n));
        points.push([p[0], p[1]]);
        if p[2] == 0. {
            continue;
        }
        let angle = sweep(p[2]);
        let d = [q[0] - p[0], q[1] - p[1]];
        let h = (1. - p[2] * p[2]) / (4. * p[2]);
        let c = [(p[0] + q[0]) / 2. - d[1] * h, (p[1] + q[1]) / 2. + d[0] * h];
        let r = (p[0] - c[0]).hypot(p[1] - c[1]);
        // A tangent polyline over a step of this angle stands
        // r / cos(step / 2) - r outside the arc; chords stay closer.
        let step = (2. * (r / (r + deviation)).acos()).min(PI / 3.);
        let pieces = (angle.abs() / step).ceil().max(1.) as usize;
        let start = (p[1] - c[1]).atan2(p[0] - c[0]);
        let part = angle / pieces as f64;
        // A counter-clockwise arc has its center, so the material, on its
        // left: it bulges out of the material.
        if (angle > 0.) == enclose {
            let radius = r / (part / 2.).cos();
            for i in 0..pieces {
                let a = start + (i as f64 + 0.5) * part;
                points.push([c[0] + radius * a.cos(), c[1] + radius * a.sin()]);
            }
        } else {
            for i in 1..pieces {
                let a = start + i as f64 * part;
                points.push([c[0] + r * a.cos(), c[1] + r * a.sin()]);
            }
        }
    }
    points
}

fn polygons(region: &PlanarRegion, holes: bool, enclose: bool, deviation: f64) -> Vec<Vec<P2>> {
    let mut result = vec![];
    for c in &region.components {
        result.push(polygon(&c.outer, true, enclose, deviation));
        if holes {
            result.extend(
                c.holes
                    .iter()
                    .map(|h| polygon(h, false, enclose, deviation)),
            );
        }
    }
    result
}

fn bounds(loops: &[Vec<P2>]) -> [f64; 4] {
    let mut b = [
        f64::INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NEG_INFINITY,
    ];
    for p in loops.iter().flatten() {
        b = [
            b[0].min(p[0]),
            b[1].min(p[1]),
            b[2].max(p[0]),
            b[3].max(p[1]),
        ];
    }
    b
}

/// A convex piece: between x0 and x1, from a lower to an upper edge.
#[derive(Clone, Copy, Debug)]
struct Trapezoid {
    x0: f64,
    x1: f64,
    /// Lower and upper y at x0 and at x1.
    low: [f64; 2],
    high: [f64; 2],
}
impl Trapezoid {
    /// (lower, upper) y at x.
    fn at(&self, x: f64) -> (f64, f64) {
        if x <= self.x0 {
            (self.low[0], self.high[0])
        } else if x >= self.x1 {
            (self.low[1], self.high[1])
        } else {
            let t = (x - self.x0) / (self.x1 - self.x0);
            (
                self.low[0] + t * (self.low[1] - self.low[0]),
                self.high[0] + t * (self.high[1] - self.high[0]),
            )
        }
    }
    fn moved(&self, x: f64, y: f64) -> Self {
        Self {
            x0: self.x0 + x,
            x1: self.x1 + x,
            low: [self.low[0] + y, self.low[1] + y],
            high: [self.high[0] + y, self.high[1] + y],
        }
    }
}

/// The material of polygons (material on the left of each, counted by
/// winding so that overlapping polygons stay material) as trapezoids between
/// the vertical lines through every vertex; with `frame`, the rest of that
/// box instead.
fn trapezoids(loops: &[Vec<P2>], frame: Option<[f64; 4]>) -> Vec<Trapezoid> {
    let mut xs: Vec<f64> = loops.iter().flatten().map(|p| p[0]).collect();
    if let Some(f) = frame {
        xs.extend([f[0], f[2]]);
    }
    xs.sort_by(f64::total_cmp);
    xs.dedup();
    let edges: Vec<(P2, P2)> = loops
        .iter()
        .flat_map(|l| (0..l.len()).map(move |i| (l[i], l[(i + 1) % l.len()])))
        .filter(|(p, q)| p[0] != q[0])
        .collect();
    let mut result = vec![];
    let mut crossing: Vec<(f64, f64, i32)> = vec![];
    for w in xs.windows(2) {
        let (xa, xb) = (w[0], w[1]);
        crossing.clear();
        for (p, q) in &edges {
            if p[0].min(q[0]) <= xa && p[0].max(q[0]) >= xb {
                let y = |x: f64| {
                    if x == p[0] {
                        p[1]
                    } else if x == q[0] {
                        q[1]
                    } else {
                        p[1] + (x - p[0]) / (q[0] - p[0]) * (q[1] - p[1])
                    }
                };
                // An edge running in +x has the material above it.
                crossing.push((y(xa), y(xb), if q[0] > p[0] { 1 } else { -1 }));
            }
        }
        crossing.sort_by(|a, b| {
            (a.0 + a.1)
                .total_cmp(&(b.0 + b.1))
                .then(b.2.cmp(&a.2))
                .then(a.0.total_cmp(&b.0))
        });
        let mut emit = |low: (f64, f64), high: (f64, f64)| {
            let piece = Trapezoid {
                x0: xa,
                x1: xb,
                low: [low.0.min(high.0), low.1.min(high.1)],
                high: [low.0.max(high.0), low.1.max(high.1)],
            };
            if piece.high[0] > piece.low[0] || piece.high[1] > piece.low[1] {
                result.push(piece);
            }
        };
        let mut winding = 0;
        // The lower edge of the open piece: of material, or of the frame's
        // free space.
        let mut open = frame.map(|f| (f[1], f[1]));
        for &(ya, yb, direction) in &crossing {
            let before = winding > 0;
            winding += direction;
            if before == (winding > 0) {
                continue;
            }
            match (frame.is_some(), winding > 0) {
                (false, true) | (true, false) => open = Some((ya, yb)),
                (false, false) | (true, true) => {
                    if let Some(low) = open.take() {
                        emit(low, (ya, yb));
                    }
                }
            }
        }
        if let (Some(f), Some(low)) = (frame, open) {
            emit(low, (f[3], f[3]));
        }
    }
    result
}

/// Trapezoids by the x they cover, for overlap queries.
struct Index {
    x0: f64,
    width: f64,
    buckets: Vec<Vec<u32>>,
    pieces: Vec<Trapezoid>,
}
impl Index {
    fn new(x0: f64, x1: f64) -> Self {
        let count = 512;
        Self {
            x0,
            width: ((x1 - x0) / count as f64).max(f64::MIN_POSITIVE),
            buckets: vec![vec![]; count],
            pieces: vec![],
        }
    }
    fn bucket(&self, x: f64) -> usize {
        (((x - self.x0) / self.width).max(0.) as usize).min(self.buckets.len() - 1)
    }
    fn add(&mut self, piece: Trapezoid) {
        let id = self.pieces.len() as u32;
        self.pieces.push(piece);
        for b in self.bucket(piece.x0)..=self.bucket(piece.x1) {
            self.buckets[b].push(id);
        }
    }
    /// The y translations at which `moving` (moved by x) would overlap a
    /// piece here: one open interval per pair of convex pieces.
    fn forbidden(
        &self,
        moving: &[Trapezoid],
        x: f64,
        (y_low, y_high): (f64, f64),
        out: &mut Vec<(f64, f64)>,
    ) {
        if self.pieces.is_empty() {
            return;
        }
        for q in moving {
            let (q0, q1) = (q.x0 + x, q.x1 + x);
            let (first, last) = (self.bucket(q0), self.bucket(q1));
            for bucket in first..=last {
                for &id in &self.buckets[bucket] {
                    let p = &self.pieces[id as usize];
                    // Listed once: in the first bucket both cover.
                    if bucket != first.max(self.bucket(p.x0)) {
                        continue;
                    }
                    let (a, b) = (p.x0.max(q0), p.x1.min(q1));
                    if a >= b {
                        continue;
                    }
                    let (pa, pb) = (p.at(a), p.at(b));
                    let (qa, qb) = (q.at(a - x), q.at(b - x));
                    // Overlap at some x' needs p's bottom below q's top and
                    // q's bottom below p's top there; both are linear in x'.
                    let low = (pa.0 - qa.1).min(pb.0 - qb.1);
                    let high = (pa.1 - qa.0).max(pb.1 - qb.0);
                    if low < high && high > y_low && low < y_high {
                        out.push((low, high));
                    }
                }
            }
        }
    }
}

/// A part in one allowed orientation, as planning pieces round the origin.
struct Shape {
    rotation_deg: f64,
    mirrored: bool,
    /// The part with half the part distance round it (against other parts).
    apart: Vec<Trapezoid>,
    /// The part with the sheet distance round it (against the sheet).
    inset: Vec<Trapezoid>,
    inset_bounds: [f64; 4],
}

struct Planner<'a, C: CancelSignal> {
    shapes: Vec<Vec<Shape>>,
    areas: Vec<f64>,
    sheet: Index,
    sheet_bounds: [f64; 4],
    resolution: f64,
    cancel: &'a C,
}

struct Layout {
    /// Per entry of the order: (orientation, x, y) or None when it did not fit.
    placed: Vec<Option<(usize, f64, f64)>>,
    unplaced_area: f64,
    right: f64,
}
impl Layout {
    fn better_than(&self, other: &Layout) -> bool {
        self.unplaced_area
            .total_cmp(&other.unplaced_area)
            .then(self.right.total_cmp(&other.right))
            .is_lt()
    }
}

impl<C: CancelSignal> Planner<'_, C> {
    /// Bottom-left fill of the parts in this order.
    fn fill(&self, order: &[usize]) -> Result<Layout, String> {
        let sb = self.sheet_bounds;
        let mut placed_parts = Index::new(sb[0], sb[2]);
        // Placed parts only add obstacles: a column that had no free
        // position for a shape never gets one, so the next copy of the same
        // shape starts where the last one stopped.
        let mut next_column: Vec<Vec<usize>> =
            self.shapes.iter().map(|s| vec![0; s.len()]).collect();
        let mut intervals = vec![];
        let mut layout = Layout {
            placed: vec![],
            unplaced_area: 0.,
            right: sb[0],
        };
        for &part in order {
            let mut best: Option<(f64, f64, usize, f64, f64)> = None;
            for (o, shape) in self.shapes[part].iter().enumerate() {
                let b = shape.inset_bounds;
                let (x_low, x_high) = (sb[0] - b[0], sb[2] - b[2]);
                let ys = (sb[1] - b[1], sb[3] - b[3]);
                if x_high < x_low || ys.1 < ys.0 {
                    next_column[part][o] = usize::MAX;
                    continue;
                }
                let columns = ((x_high - x_low) / self.resolution).floor() as usize;
                // One more at the far end, where a part can still fit.
                let last = if x_low + columns as f64 * self.resolution < x_high {
                    columns + 1
                } else {
                    columns
                };
                let mut column = next_column[part][o];
                let mut found = None;
                while column <= last {
                    if self.cancel.is_cancelled() {
                        return Err("nesting cancelled".into());
                    }
                    let x = if column > columns {
                        x_high
                    } else {
                        x_low + column as f64 * self.resolution
                    };
                    intervals.clear();
                    self.sheet.forbidden(&shape.inset, x, ys, &mut intervals);
                    placed_parts.forbidden(&shape.apart, x, ys, &mut intervals);
                    intervals.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)));
                    // The lowest y in no open interval.
                    let mut y = ys.0;
                    for &(low, high) in &intervals {
                        if low >= y {
                            break;
                        }
                        y = y.max(high);
                    }
                    if y <= ys.1 {
                        found = Some((x, y));
                        break;
                    }
                    column += 1;
                }
                next_column[part][o] = if found.is_some() { column } else { usize::MAX };
                if let Some((x, y)) = found {
                    let key = (x + b[2], y + b[3]);
                    if best.is_none_or(|(right, top, ..)| {
                        key.0.total_cmp(&right).then(key.1.total_cmp(&top)).is_lt()
                    }) {
                        best = Some((key.0, key.1, o, x, y));
                    }
                }
            }
            match best {
                Some((right, _, o, x, y)) => {
                    for piece in &self.shapes[part][o].apart {
                        placed_parts.add(piece.moved(x, y));
                    }
                    layout.right = layout.right.max(right);
                    layout.placed.push(Some((o, x, y)));
                }
                None => {
                    layout.unplaced_area += self.areas[part];
                    layout.placed.push(None);
                }
            }
        }
        Ok(layout)
    }
}

/// splitmix64 (Steele, Lea and Flood, "Fast splittable pseudorandom number
/// generators", OOPSLA 2014): the same sequence on every platform.
fn next_random(state: &mut u64) -> u64 {
    *state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = *state;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

fn placed_component(source: &Component, p: &Placement) -> Component {
    let turn = rotation(p.rotation_deg);
    let boundary = |b: &Boundary| Boundary {
        vertices: b
            .vertices
            .iter()
            .map(|v| apply(p.mirrored, turn, p.translation_mm, *v))
            .collect(),
    };
    Component {
        outer: boundary(&source.outer),
        holes: source.holes.iter().map(boundary).collect(),
    }
}

/// Checks a layout against a request with the independent check alone: the
/// placements and the placed region may come from anywhere (an edited nest,
/// a manual layout).
pub fn verify(
    request: &NestRequest,
    placements: &[Placement],
    region: &PlanarRegion,
) -> Result<NestCheck, String> {
    let clearances = clearances(request)?;
    let options = ConstructionOptions {
        tolerance_mm: request.tolerance_mm,
    };
    request.sheet.prepare(options)?;
    for part in &request.parts {
        part.region.prepare(options)?;
    }
    check::verify(request, &clearances, placements, region)
}

/// Nests the request's parts on its sheet. See the module documentation.
pub fn nest(request: &NestRequest, cancel: &impl CancelSignal) -> Result<NestResult, String> {
    if cancel.is_cancelled() {
        return Err("nesting cancelled".into());
    }
    let clearances = clearances(request)?;
    let options = ConstructionOptions {
        tolerance_mm: request.tolerance_mm,
    };
    // Planning outlines stand this much beyond the required distances, so
    // the construction's own tolerance cannot bring a layout below them.
    let slack = 8. * request.tolerance_mm;
    let deviation = request.resolution_mm / 4.;
    let sheet = request
        .sheet
        .prepare(options)
        .map_err(|e| format!("nest sheet: {e}"))?;
    let sheet_loops = polygons(sheet.definition(), true, false, deviation);
    let sheet_bounds = bounds(&sheet_loops);
    if (sheet_bounds[2] - sheet_bounds[0]) / request.resolution_mm > 1e6 {
        return Err(format!(
            "nest: resolution_mm {} gives more than a million columns across this sheet; use a coarser resolution",
            request.resolution_mm
        ));
    }
    let mut sheet_index = Index::new(sheet_bounds[0], sheet_bounds[2]);
    for piece in trapezoids(&sheet_loops, Some(sheet_bounds)) {
        sheet_index.add(piece);
    }
    let mut shapes = vec![];
    let mut areas = vec![];
    for (i, part) in request.parts.iter().enumerate() {
        let prepared = part
            .region
            .prepare(options)
            .map_err(|e| format!("nest part {i}: {e}"))?;
        areas.push(component_area(&part.region.components[0]));
        // Negative offsets grow the part (the kerf compensation's offset).
        let grown = |distance: f64, holes: bool| -> Result<Vec<Vec<P2>>, String> {
            let region = prepared
                .offset(-(distance + slack))
                .map_err(|e| format!("nest part {i}: {e}"))?
                .region;
            Ok(polygons(&region, holes, true, deviation))
        };
        let apart = grown(clearances.part_distance_mm / 2., request.part_in_part)?;
        let inset = grown(clearances.sheet_distance_mm, true)?;
        let mut orientations = vec![];
        for mirrored in [false, true] {
            if mirrored && !part.mirror {
                continue;
            }
            for &rotation_deg in &part.rotations_deg {
                let turn = rotation(rotation_deg);
                let turned = |loops: &[Vec<P2>]| -> Vec<Vec<P2>> {
                    loops
                        .iter()
                        .map(|l| {
                            let mut points: Vec<P2> = l
                                .iter()
                                .map(|p| {
                                    let q = apply(mirrored, turn, [0.; 2], [p[0], p[1], 0.]);
                                    [q[0], q[1]]
                                })
                                .collect();
                            // Mirroring turns a loop round: keep the
                            // material on its left.
                            if mirrored {
                                points.reverse();
                            }
                            points
                        })
                        .collect()
                };
                let inset = turned(&inset);
                orientations.push(Shape {
                    rotation_deg,
                    mirrored,
                    apart: trapezoids(&turned(&apart), None),
                    inset_bounds: bounds(&inset),
                    inset: trapezoids(&inset, None),
                });
            }
        }
        shapes.push(orientations);
    }
    let planner = Planner {
        shapes,
        areas,
        sheet: sheet_index,
        sheet_bounds,
        resolution: request.resolution_mm,
        cancel,
    };
    // Largest area first; equal areas in the declared order.
    let mut order: Vec<usize> = request
        .parts
        .iter()
        .enumerate()
        .flat_map(|(i, p)| std::iter::repeat_n(i, p.quantity as usize))
        .collect();
    order.sort_by(|&a, &b| {
        planner.areas[b]
            .total_cmp(&planner.areas[a])
            .then(a.cmp(&b))
    });
    let mut best = planner.fill(&order)?;
    let mut improvements = 0;
    let mut state = request.seed;
    if order.len() > 1 {
        for _ in 0..request.search_iterations {
            let i = (next_random(&mut state) % order.len() as u64) as usize;
            let j = (next_random(&mut state) % order.len() as u64) as usize;
            if order[i] == order[j] {
                continue;
            }
            order.swap(i, j);
            let candidate = planner.fill(&order)?;
            if candidate.better_than(&best) {
                best = candidate;
                improvements += 1;
            } else {
                order.swap(i, j);
            }
        }
    }
    // Copies are numbered in placement order; the result lists parts in the
    // declared order.
    let mut copies = vec![0u32; request.parts.len()];
    let mut placements = vec![];
    for (&part, slot) in order.iter().zip(&best.placed) {
        if let Some((o, x, y)) = *slot {
            let shape = &planner.shapes[part][o];
            placements.push(Placement {
                part,
                copy: copies[part],
                rotation_deg: shape.rotation_deg,
                mirrored: shape.mirrored,
                translation_mm: [x, y],
            });
            copies[part] += 1;
        }
    }
    placements.sort_by_key(|p| (p.part, p.copy));
    let unplaced: Vec<Unplaced> = request
        .parts
        .iter()
        .enumerate()
        .filter(|(i, p)| copies[*i] < p.quantity)
        .map(|(i, p)| Unplaced {
            part: i,
            quantity: p.quantity - copies[i],
        })
        .collect();
    if !unplaced.is_empty() && !request.allow_unplaced {
        let list: Vec<String> = unplaced
            .iter()
            .map(|u| {
                format!(
                    "part {}: {} of {}",
                    u.part, u.quantity, request.parts[u.part].quantity
                )
            })
            .collect();
        return Err(format!(
            "nest: these parts do not fit on the sheet with the declared spacing, margin and rotations ({}); use a larger sheet, fewer parts or more rotations, or set allow_unplaced to get the partial nest with the rest listed",
            list.join(", ")
        ));
    }
    let region = PlanarRegion {
        version: 1,
        units: "mm".into(),
        name: "nest".into(),
        boundary_error_mm: request
            .parts
            .iter()
            .map(|p| p.region.boundary_error_mm)
            .fold(0., f64::max),
        components: placements
            .iter()
            .map(|p| placed_component(&request.parts[p.part].region.components[0], p))
            .collect(),
        source: serde_json::Value::Null,
    };
    let check = check::verify(request, &clearances, &placements, &region).map_err(|e| {
        format!(
            "nest: the planned layout failed its independent check and is not returned ({e}); this is a planner defect: report it, and try another resolution_mm meanwhile"
        )
    })?;
    let mut used: Option<[f64; 4]> = None;
    for c in &region.components {
        let b = check::Loop::new(&c.outer)?.bounds;
        used = Some(used.map_or(b, |u| {
            [
                u[0].min(b[0]),
                u[1].min(b[1]),
                u[2].max(b[2]),
                u[3].max(b[3]),
            ]
        }));
    }
    let sheet_area_mm2 = component_area(&request.sheet.components[0]);
    let placed_area_mm2: f64 = placements.iter().map(|p| planner.areas[p.part]).sum();
    let mut diagnostics = vec![];
    if !unplaced.is_empty() {
        diagnostics.push(format!(
            "{} parts did not fit and are listed as unplaced",
            unplaced.iter().map(|u| u.quantity).sum::<u32>()
        ));
    }
    if check.parts_in_holes > 0 {
        diagnostics.push(format!(
            "{} parts lie in holes of other parts",
            check.parts_in_holes
        ));
    }
    Ok(NestResult {
        strategy: "bottom_left_fill",
        placements,
        unplaced,
        region,
        sheet_area_mm2,
        placed_area_mm2,
        utilisation: placed_area_mm2 / sheet_area_mm2,
        used_bounds_mm: used,
        lead_reach_mm: clearances.lead_reach_mm,
        check,
        search_iterations: request.search_iterations,
        search_improvements: improvements,
        diagnostics,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn area(pieces: &[Trapezoid]) -> f64 {
        pieces
            .iter()
            .map(|t| (t.x1 - t.x0) * (t.high[0] - t.low[0] + t.high[1] - t.low[1]) / 2.)
            .sum()
    }

    #[test]
    fn trapezoids_cover_the_material_and_the_frame_covers_the_rest() {
        // An L shape with a square hole, counter-clockwise and clockwise.
        let outline = vec![
            [0., 0.],
            [10., 0.],
            [10., 4.],
            [4., 4.],
            [4., 10.],
            [0., 10.],
        ];
        let hole = vec![[1., 1.], [1., 3.], [3., 3.], [3., 1.]];
        let loops = [outline, hole];
        assert!((area(&trapezoids(&loops, None)) - 60.).abs() < 1e-12);
        let frame = [0., 0., 10., 10.];
        assert!((area(&trapezoids(&loops, Some(frame))) - 40.).abs() < 1e-12);
    }

    #[test]
    fn planning_polygons_enclose_or_stay_inside_arcs() {
        let disc = Boundary {
            vertices: vec![[10., 0., 1.], [-10., 0., 1.]],
        };
        let radii = |enclose: bool, outer: bool| -> (f64, f64) {
            let p = polygon(&disc, outer, enclose, 0.1);
            // The nearest point of each edge and the farthest vertex.
            let near = (0..p.len())
                .map(|i| {
                    let (a, b) = (p[i], p[(i + 1) % p.len()]);
                    let d = [b[0] - a[0], b[1] - a[1]];
                    (a[0] * d[1] - a[1] * d[0]).abs() / d[0].hypot(d[1])
                })
                .fold(f64::INFINITY, f64::min);
            let far = p.iter().map(|v| v[0].hypot(v[1])).fold(0., f64::max);
            (near, far)
        };
        // A disc of material: enclosed by a polygon outside it.
        let (near, far) = radii(true, true);
        assert!(near >= 10. - 1e-12 && far <= 10.1 + 1e-12, "{near} {far}");
        // As the material's subset, and as a hole to enclose material round
        // it: inside the circle.
        for (enclose, outer) in [(false, true), (true, false)] {
            let (near, far) = radii(enclose, outer);
            assert!(near >= 9.9 - 1e-12 && far <= 10. + 1e-12, "{near} {far}");
        }
        // A hole as a subset of the material round it: outside the circle.
        let (near, far) = radii(false, false);
        assert!(near >= 10. - 1e-12 && far <= 10.1 + 1e-12, "{near} {far}");
    }

    #[test]
    fn quarter_turns_are_exact() {
        assert_eq!(rotation(90.), (0., 1.));
        assert_eq!(rotation(-90.), (0., -1.));
        assert_eq!(rotation(180.), (-1., 0.));
        assert_eq!(rotation(360.), (1., 0.));
        assert_eq!(
            apply(true, rotation(90.), [1., 2.], [3., 4., 0.5]),
            [-3., -1., -0.5]
        );
    }
}
