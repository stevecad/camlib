//! The independent check of a nest.
//!
//! It shares nothing with the planner: no offsets, no polygons, no
//! trapezoids. It reads the placed parts as the line and arc loops that will
//! be cut and computes closed-form distances between them (line-line,
//! line-arc and arc-arc; nothing is sampled), then decides containment by
//! winding numbers. A layout is accepted only when
//!
//! - every placed part is a rigid (or, where declared, mirrored) copy of its
//!   source part, at the reported placement;
//! - every two parts are at least the required part distance apart, and
//!   neither overlaps the other: a part inside another is accepted only when
//!   it is proven to lie in one of that part's holes, and only when the
//!   request allows it;
//! - every part lies in the sheet, at least the required distance from the
//!   sheet's outline and holes.
//!
//! Distances are computed in floating point; a bound on their rounding error
//! is added to every requirement, so a layout that passes has the declared
//! distances.
use super::{Clearances, NestRequest, Placement, apply, rotation};
use crate::planar::{Boundary, Component, ConstructionOptions, PlanarRegion};
use serde::Serialize;
use std::f64::consts::PI;

type P2 = [f64; 2];

/// What the check proved about a layout.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct NestCheck {
    /// Always "proven": a layout that fails is refused, not reported.
    pub status: &'static str,
    /// Required and smallest found distance between two parts (None with
    /// fewer than two parts).
    pub part_distance_mm: f64,
    pub min_part_distance_mm: Option<f64>,
    /// Required and smallest found distance from a part to the sheet's
    /// outline or holes (None without parts).
    pub sheet_distance_mm: f64,
    pub min_sheet_distance_mm: Option<f64>,
    /// The rounding bound added to both requirements.
    pub numerical_guard_mm: f64,
    /// Parts proven to lie inside a hole of another part.
    pub parts_in_holes: usize,
}

#[derive(Clone, Copy, Debug)]
pub(super) enum Edge {
    Line(P2, P2),
    /// A circular arc of at most half a turn from `a` to `b`.
    Arc {
        a: P2,
        b: P2,
        c: P2,
        r: f64,
        ccw: bool,
    },
}

fn sub(a: P2, b: P2) -> P2 {
    [a[0] - b[0], a[1] - b[1]]
}
fn dot(a: P2, b: P2) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
fn cross(a: P2, b: P2) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}
fn dist(a: P2, b: P2) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

/// The edges of a closed boundary (bulge = tan(sweep / 4), at most 1).
pub(super) fn edges(boundary: &Boundary) -> Result<Vec<Edge>, String> {
    let v = &boundary.vertices;
    if v.len() < 2 {
        return Err("a boundary needs at least two vertices".into());
    }
    let mut result = Vec::with_capacity(v.len());
    for i in 0..v.len() {
        let (p, q) = (v[i], v[(i + 1) % v.len()]);
        let (a, b, bulge) = ([p[0], p[1]], [q[0], q[1]], p[2]);
        if !(a.iter().chain(&b).all(|n| n.is_finite()) && bulge.abs() <= 1.) {
            return Err("boundary vertices must be finite, with bulge within [-1, 1]".into());
        }
        let chord = dist(a, b);
        if chord == 0. {
            return Err("a boundary has a zero-length edge".into());
        }
        if bulge == 0. {
            if v.len() == 2 {
                return Err("a two-vertex boundary must be two arcs".into());
            }
            result.push(Edge::Line(a, b));
            continue;
        }
        // The center lies left of the chord for a counter-clockwise arc, at
        // (chord / 2) / tan(sweep / 2) from its middle; tan(sweep / 2) =
        // 2 bulge / (1 - bulge^2).
        let h = (1. - bulge * bulge) / (4. * bulge);
        let d = sub(b, a);
        let c = [(a[0] + b[0]) / 2. - d[1] * h, (a[1] + b[1]) / 2. + d[0] * h];
        result.push(Edge::Arc {
            a,
            b,
            c,
            r: dist(a, c),
            ccw: bulge > 0.,
        });
    }
    Ok(result)
}

/// Whether the point `q` of an arc's circle lies on the arc: an arc of at
/// most half a turn is the part of its circle on one side of its chord.
fn on_arc(a: P2, b: P2, ccw: bool, q: P2) -> bool {
    let s = cross(sub(b, a), sub(q, a));
    if ccw { s <= 0. } else { s >= 0. }
}

fn point_segment(p: P2, a: P2, b: P2) -> f64 {
    let d = sub(b, a);
    let t = (dot(sub(p, a), d) / dot(d, d)).clamp(0., 1.);
    dist(p, [a[0] + t * d[0], a[1] + t * d[1]])
}

fn point_arc(p: P2, a: P2, b: P2, c: P2, r: f64, ccw: bool) -> f64 {
    let v = sub(p, c);
    let n = v[0].hypot(v[1]);
    if n > 0. {
        let q = [c[0] + r * v[0] / n, c[1] + r * v[1] / n];
        if on_arc(a, b, ccw, q) {
            return (n - r).abs();
        }
    }
    dist(p, a).min(dist(p, b))
}

fn orient(a: P2, b: P2, c: P2) -> f64 {
    let k = |p: P2| robust::Coord { x: p[0], y: p[1] };
    robust::orient2d(k(a), k(b), k(c))
}

fn segment_segment(a: P2, b: P2, c: P2, d: P2) -> f64 {
    // A proper crossing, by exact predicates; a touching pair has an
    // endpoint at distance zero below.
    let (o1, o2, o3, o4) = (
        orient(a, b, c),
        orient(a, b, d),
        orient(c, d, a),
        orient(c, d, b),
    );
    if ((o1 > 0.) != (o2 > 0.) && o1 != 0. && o2 != 0.)
        && ((o3 > 0.) != (o4 > 0.) && o3 != 0. && o4 != 0.)
    {
        return 0.;
    }
    point_segment(a, c, d)
        .min(point_segment(b, c, d))
        .min(point_segment(c, a, b))
        .min(point_segment(d, a, b))
}

fn segment_arc(s: P2, e: P2, arc: &Edge) -> f64 {
    let Edge::Arc { a, b, c, r, ccw } = *arc else {
        unreachable!()
    };
    let mut best = point_arc(s, a, b, c, r, ccw)
        .min(point_arc(e, a, b, c, r, ccw))
        .min(point_segment(a, s, e))
        .min(point_segment(b, s, e));
    let d = sub(e, s);
    let f = sub(s, c);
    let (qa, qb, qc) = (dot(d, d), 2. * dot(f, d), dot(f, f) - r * r);
    // Where the segment's line meets the circle.
    let disc = qb * qb - 4. * qa * qc;
    if disc >= 0. {
        for sign in [-1., 1.] {
            let t = (-qb + sign * disc.sqrt()) / (2. * qa);
            let q = [s[0] + t * d[0], s[1] + t * d[1]];
            if (0. ..=1.).contains(&t) && on_arc(a, b, ccw, q) {
                return 0.;
            }
        }
    }
    // The arc point nearest the line: toward the foot of the center.
    let t = -dot(f, d) / qa;
    let foot = [s[0] + t * d[0], s[1] + t * d[1]];
    let n = dist(foot, c);
    if n > 0. {
        let q = [
            c[0] + r * (foot[0] - c[0]) / n,
            c[1] + r * (foot[1] - c[1]) / n,
        ];
        if on_arc(a, b, ccw, q) {
            best = best.min(point_segment(q, s, e));
        }
    }
    best
}

fn arc_arc(first: &Edge, second: &Edge) -> f64 {
    let (
        Edge::Arc {
            a: a1,
            b: b1,
            c: c1,
            r: r1,
            ccw: w1,
        },
        Edge::Arc {
            a: a2,
            b: b2,
            c: c2,
            r: r2,
            ccw: w2,
        },
    ) = (*first, *second)
    else {
        unreachable!()
    };
    let to2 = |p: P2| point_arc(p, a2, b2, c2, r2, w2);
    let to1 = |p: P2| point_arc(p, a1, b1, c1, r1, w1);
    let mut best = to2(a1).min(to2(b1)).min(to1(a2)).min(to1(b2));
    let v = sub(c2, c1);
    let d = v[0].hypot(v[1]);
    if d > 0. {
        let u = [v[0] / d, v[1] / d];
        // An interior closest pair lies on the line through both centers.
        for sign in [-1., 1.] {
            let q1 = [c1[0] + sign * r1 * u[0], c1[1] + sign * r1 * u[1]];
            if on_arc(a1, b1, w1, q1) {
                best = best.min(to2(q1));
            }
            let q2 = [c2[0] + sign * r2 * u[0], c2[1] + sign * r2 * u[1]];
            if on_arc(a2, b2, w2, q2) {
                best = best.min(to1(q2));
            }
        }
        // Where the circles meet.
        if d <= r1 + r2 && d >= (r1 - r2).abs() {
            let along = (d * d + r1 * r1 - r2 * r2) / (2. * d);
            let h = (r1 * r1 - along * along).max(0.).sqrt();
            for sign in [-1., 1.] {
                let q = [
                    c1[0] + along * u[0] - sign * h * u[1],
                    c1[1] + along * u[1] + sign * h * u[0],
                ];
                if on_arc(a1, b1, w1, q) && on_arc(a2, b2, w2, q) {
                    return 0.;
                }
            }
        }
    }
    best
}

/// The smallest distance between two edges, in closed form.
pub(super) fn edge_distance(e: &Edge, f: &Edge) -> f64 {
    match (e, f) {
        (Edge::Line(a, b), Edge::Line(c, d)) => segment_segment(*a, *b, *c, *d),
        (Edge::Line(a, b), arc @ Edge::Arc { .. }) | (arc @ Edge::Arc { .. }, Edge::Line(a, b)) => {
            segment_arc(*a, *b, arc)
        }
        (Edge::Arc { .. }, Edge::Arc { .. }) => arc_arc(e, f),
    }
}

/// A closed loop with its bounding box.
pub(super) struct Loop {
    pub edges: Vec<Edge>,
    pub bounds: [f64; 4],
}

impl Loop {
    pub fn new(boundary: &Boundary) -> Result<Self, String> {
        let edges = edges(boundary)?;
        let mut bounds = [
            f64::INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NEG_INFINITY,
        ];
        let mut add = |p: P2| {
            bounds = [
                bounds[0].min(p[0]),
                bounds[1].min(p[1]),
                bounds[2].max(p[0]),
                bounds[3].max(p[1]),
            ];
        };
        for e in &edges {
            match *e {
                Edge::Line(a, _) => add(a),
                Edge::Arc { a, b, c, r, ccw } => {
                    add(a);
                    // The circle's extreme points that lie on the arc.
                    for q in [
                        [c[0] - r, c[1]],
                        [c[0] + r, c[1]],
                        [c[0], c[1] - r],
                        [c[0], c[1] + r],
                    ] {
                        if on_arc(a, b, ccw, q) {
                            add(q);
                        }
                    }
                }
            }
        }
        Ok(Self { edges, bounds })
    }
    fn first(&self) -> P2 {
        match self.edges[0] {
            Edge::Line(a, _) | Edge::Arc { a, .. } => a,
        }
    }
    /// How often the loop winds around a point that is not on it.
    fn winding(&self, p: P2) -> i32 {
        let mut total = 0.;
        for e in &self.edges {
            let (a, b) = match *e {
                Edge::Line(a, b) | Edge::Arc { a, b, .. } => (a, b),
            };
            let (u, v) = (sub(a, p), sub(b, p));
            let (s, d) = (cross(u, v), dot(u, v));
            let mut angle = s.atan2(d);
            // Seen from between an arc and its chord, the arc turns the
            // other way round the point than its chord does.
            if let Edge::Arc { c, r, ccw, .. } = *e
                && dist(p, c) < r
            {
                if ccw {
                    if s < 0. {
                        angle += 2. * PI;
                    } else if s == 0. && d < 0. {
                        angle = PI;
                    }
                } else if s > 0. {
                    angle -= 2. * PI;
                } else if s == 0. && d < 0. {
                    angle = -PI;
                }
            }
            total += angle;
        }
        (total / (2. * PI)).round() as i32
    }
    fn contains(&self, p: P2) -> bool {
        self.winding(p) != 0
    }
}

/// The gap between two boxes (zero when they overlap): a lower bound of the
/// distance between anything inside them.
fn box_gap(a: &[f64; 4], b: &[f64; 4]) -> f64 {
    let dx = (a[0] - b[2]).max(b[0] - a[2]).max(0.);
    let dy = (a[1] - b[3]).max(b[1] - a[3]).max(0.);
    dx.hypot(dy)
}

/// The smallest distance between two loops, or `floor` when it is proven to
/// be at least `floor` (so far-apart loops cost only a box test).
fn loop_distance(a: &Loop, b: &Loop, floor: f64) -> f64 {
    if box_gap(&a.bounds, &b.bounds) >= floor {
        return floor;
    }
    let mut best = floor;
    for e in &a.edges {
        for f in &b.edges {
            best = best.min(edge_distance(e, f));
        }
    }
    best
}

struct Part {
    outer: Loop,
    holes: Vec<Loop>,
}
impl Part {
    fn new(c: &Component) -> Result<Self, String> {
        Ok(Self {
            outer: Loop::new(&c.outer)?,
            holes: c.holes.iter().map(Loop::new).collect::<Result<_, _>>()?,
        })
    }
    fn loops(&self) -> impl Iterator<Item = &Loop> {
        std::iter::once(&self.outer).chain(&self.holes)
    }
    fn distance(&self, other: &Part, floor: f64) -> f64 {
        let mut best = floor;
        for a in self.loops() {
            for b in other.loops() {
                best = loop_distance(a, b, best);
            }
        }
        best
    }
    /// Which of this part's holes contains the point, if the point is inside
    /// the outline at all: `None` outside the part, `Some(None)` in its
    /// material, `Some(Some(hole))` in a hole.
    fn locate(&self, p: P2) -> Option<Option<usize>> {
        self.outer
            .contains(p)
            .then(|| self.holes.iter().position(|h| h.contains(p)))
    }
}

fn points(c: &Component) -> Vec<P2> {
    std::iter::once(&c.outer)
        .chain(&c.holes)
        .flat_map(|b| b.vertices.iter().map(|v| [v[0], v[1]]))
        .collect()
}

/// Every vertex of a part, as coordinates in the frame of two of its own
/// vertices (the first and `far`): equal coordinates mean a rigid copy,
/// whatever moved it.
fn frame_coordinates(c: &Component, far: usize) -> Vec<P2> {
    let points = points(c);
    let origin = points[0];
    let axis = sub(points[far], origin);
    points
        .iter()
        .map(|p| {
            let v = sub(*p, origin);
            [dot(v, axis), cross(axis, v)]
        })
        .collect()
}

fn magnitude(c: &Component) -> f64 {
    std::iter::once(&c.outer)
        .chain(&c.holes)
        .flat_map(|b| &b.vertices)
        .fold(1., |m: f64, v| m.max(v[0].abs()).max(v[1].abs()))
}

pub(super) fn verify(
    request: &NestRequest,
    clearances: &Clearances,
    placements: &[Placement],
    region: &PlanarRegion,
) -> Result<NestCheck, String> {
    let options = ConstructionOptions {
        tolerance_mm: request.tolerance_mm,
    };
    if region.components.len() != placements.len() {
        return Err(format!(
            "nest check: {} placements for {} placed parts",
            placements.len(),
            region.components.len()
        ));
    }
    // The placed region itself is valid input for the cut.
    region.prepare(options).map_err(|e| {
        format!("nest check: the placed parts are not a valid region for the cut (parts overlap or touch): {e}")
    })?;
    let mut seen = std::collections::BTreeSet::new();
    let mut scale: f64 = 1.;
    for (k, (placement, placed)) in placements.iter().zip(&region.components).enumerate() {
        let part = request.parts.get(placement.part).ok_or_else(|| {
            format!(
                "nest check: placement {k} names part {}, which is not in the request",
                placement.part
            )
        })?;
        if placement.copy >= part.quantity || !seen.insert((placement.part, placement.copy)) {
            return Err(format!(
                "nest check: placement {k} is copy {} of part {}, which is placed twice or beyond its quantity {}",
                placement.copy, placement.part, part.quantity
            ));
        }
        if !part.rotations_deg.contains(&placement.rotation_deg)
            || (placement.mirrored && !part.mirror)
        {
            return Err(format!(
                "nest check: placement {k} turns part {} by {} degrees (mirrored: {}), which the request does not allow",
                placement.part, placement.rotation_deg, placement.mirrored
            ));
        }
        let [source] = &part.region.components[..] else {
            return Err(format!(
                "nest check: part {} is not one outline",
                placement.part
            ));
        };
        let same_shape = source.holes.len() == placed.holes.len()
            && std::iter::once((&source.outer, &placed.outer))
                .chain(source.holes.iter().zip(&placed.holes))
                .all(|(s, p)| {
                    s.vertices.len() == p.vertices.len()
                        && s.vertices
                            .iter()
                            .zip(&p.vertices)
                            .all(|(s, p)| p[2] == if placement.mirrored { -s[2] } else { s[2] })
                });
        if !same_shape {
            return Err(format!(
                "nest check: placement {k} does not have the vertices and arcs of part {}",
                placement.part
            ));
        }
        // A rigid copy, measured in the part's own frame (no transform is
        // trusted): the products are of two lengths, so their tolerance is
        // the rounding of coordinates of this size, squared.
        let m = magnitude(source).max(magnitude(placed));
        scale = scale.max(m);
        let tolerance = 4096. * f64::EPSILON * m * m;
        let corners = points(source);
        let far = (0..corners.len())
            .max_by(|&i, &j| {
                dist(corners[i], corners[0])
                    .total_cmp(&dist(corners[j], corners[0]))
                    .then(j.cmp(&i))
            })
            .unwrap_or(0);
        let (from, to) = (
            frame_coordinates(source, far),
            frame_coordinates(placed, far),
        );
        let side = if placement.mirrored { -1. } else { 1. };
        if from
            .iter()
            .zip(&to)
            .any(|(f, t)| (f[0] - t[0]).abs() > tolerance || (f[1] - side * t[1]).abs() > tolerance)
        {
            return Err(format!(
                "nest check: placement {k} is not a rigid copy of part {}",
                placement.part
            ));
        }
        // And it is where the placement says it is.
        let turn = rotation(placement.rotation_deg);
        let moved = |s: &Boundary, p: &Boundary| {
            s.vertices.iter().zip(&p.vertices).all(|(s, p)| {
                let q = apply(placement.mirrored, turn, placement.translation_mm, *s);
                dist([q[0], q[1]], [p[0], p[1]]) <= 4096. * f64::EPSILON * m
            })
        };
        if !std::iter::once((&source.outer, &placed.outer))
            .chain(source.holes.iter().zip(&placed.holes))
            .all(|(s, p)| moved(s, p))
        {
            return Err(format!(
                "nest check: placement {k} does not match its reported rotation and translation"
            ));
        }
    }
    let [sheet] = &request.sheet.components[..] else {
        return Err("nest check: the sheet is not one outline".into());
    };
    let sheet = Part::new(sheet)?;
    let parts: Vec<Part> = region
        .components
        .iter()
        .map(Part::new)
        .collect::<Result<_, _>>()?;
    for loop_ in sheet
        .loops()
        .chain(parts.iter().flat_map(|p| p.loops()))
        .flat_map(|l| &l.edges)
    {
        if let Edge::Arc { r, .. } = loop_ {
            scale = scale.max(*r);
        }
        let (Edge::Line(a, _) | Edge::Arc { a, .. }) = loop_;
        scale = scale.max(a[0].abs()).max(a[1].abs());
    }
    // Rounding of the closed-form distances at this coordinate size.
    let guard = 4096. * f64::EPSILON * scale;
    let need_parts = clearances.part_distance_mm + guard;
    let need_sheet = clearances.sheet_distance_mm + guard;
    let mut parts_in_holes = 0;
    let mut min_sheet = f64::INFINITY;
    for (k, part) in parts.iter().enumerate() {
        let d = part.distance(&sheet, min_sheet.max(need_sheet));
        if d < need_sheet {
            return Err(format!(
                "nest check: placed part {k} is {d} mm from the sheet's edge; {} mm is required",
                clearances.sheet_distance_mm
            ));
        }
        min_sheet = min_sheet.min(d);
        // No loop touches another, so one point decides each containment.
        if sheet.locate(part.outer.first()) != Some(None) {
            return Err(format!("nest check: placed part {k} is not on the sheet"));
        }
        if sheet
            .holes
            .iter()
            .any(|h| part.locate(h.first()) == Some(None))
        {
            return Err(format!(
                "nest check: placed part {k} covers a hole of the sheet"
            ));
        }
    }
    let mut min_parts = f64::INFINITY;
    for i in 0..parts.len() {
        for j in i + 1..parts.len() {
            let (a, b) = (&parts[i], &parts[j]);
            let d = a.distance(b, min_parts.max(need_parts));
            if d < need_parts {
                return Err(format!(
                    "nest check: placed parts {i} and {j} are {d} mm apart; {} mm is required",
                    clearances.part_distance_mm
                ));
            }
            min_parts = min_parts.min(d);
            for (inner, outer, names) in [(a, b, (i, j)), (b, a, (j, i))] {
                match outer.locate(inner.outer.first()) {
                    None => {}
                    Some(Some(_)) if request.part_in_part => parts_in_holes += 1,
                    Some(Some(_)) => {
                        return Err(format!(
                            "nest check: placed part {} lies in a hole of part {}, which part_in_part=false does not allow",
                            names.0, names.1
                        ));
                    }
                    Some(None) => {
                        return Err(format!(
                            "nest check: placed part {} overlaps part {}",
                            names.0, names.1
                        ));
                    }
                }
            }
        }
    }
    Ok(NestCheck {
        status: "proven",
        part_distance_mm: clearances.part_distance_mm,
        min_part_distance_mm: (parts.len() > 1).then_some(min_parts),
        sheet_distance_mm: clearances.sheet_distance_mm,
        min_sheet_distance_mm: (!parts.is_empty()).then_some(min_sheet),
        numerical_guard_mm: guard,
        parts_in_holes,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Deterministic pseudo-random numbers in [0, 1).
    struct Random(u64);
    impl Random {
        fn next(&mut self) -> f64 {
            self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
            let mut z = self.0;
            z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
            z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
            ((z ^ (z >> 31)) >> 11) as f64 / (1u64 << 53) as f64
        }
        fn edge(&mut self) -> Edge {
            let a = [self.next() * 20., self.next() * 20.];
            let b = [a[0] + self.next() * 16. - 8., a[1] + self.next() * 16. - 8.];
            let bulge = if self.next() < 0.4 {
                0.
            } else {
                self.next() * 2. - 1.
            };
            let boundary = Boundary {
                vertices: vec![[a[0], a[1], bulge], [b[0], b[1], 0.], [30., 30., 0.]],
            };
            edges(&boundary).unwrap()[0]
        }
    }

    fn samples(e: &Edge, n: usize) -> Vec<P2> {
        (0..=n)
            .map(|i| {
                let t = i as f64 / n as f64;
                match *e {
                    Edge::Line(a, b) => [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])],
                    Edge::Arc { a, b, c, r, ccw } => {
                        let start = (a[1] - c[1]).atan2(a[0] - c[0]);
                        let mut sweep = (b[1] - c[1]).atan2(b[0] - c[0]) - start;
                        if ccw && sweep < 0. {
                            sweep += 2. * PI;
                        }
                        if !ccw && sweep > 0. {
                            sweep -= 2. * PI;
                        }
                        let angle = start + t * sweep;
                        [c[0] + r * angle.cos(), c[1] + r * angle.sin()]
                    }
                }
            })
            .collect()
    }

    /// The closed forms agree with a dense sampling of both edges: never
    /// above it, and below it by no more than the sampling step allows.
    #[test]
    fn closed_form_distances_match_dense_sampling() {
        let mut random = Random(7);
        let mut touching = 0;
        for _ in 0..300 {
            let (e, f) = (random.edge(), random.edge());
            let exact = edge_distance(&e, &f);
            let (p, q) = (samples(&e, 400), samples(&f, 400));
            let sampled = p
                .iter()
                .flat_map(|a| q.iter().map(move |b| dist(*a, *b)))
                .fold(f64::INFINITY, f64::min);
            assert!(exact <= sampled + 1e-9, "{e:?} {f:?}: {exact} > {sampled}");
            // Sample spacing is at most 8 pi / 400 on either edge.
            assert!(sampled - exact < 0.1, "{e:?} {f:?}: {exact} << {sampled}");
            touching += usize::from(exact == 0.);
        }
        assert!(touching > 10, "crossing pairs were exercised: {touching}");
    }

    #[test]
    fn winding_numbers_see_arcs_not_chords() {
        // A disc as two half-circle arcs, and a D shape (a chord and an arc).
        let disc = Loop::new(&Boundary {
            vertices: vec![[10., 0., 1.], [-10., 0., 1.]],
        })
        .unwrap();
        assert!(disc.contains([0., 0.]) && disc.contains([0., 9.9]) && disc.contains([0., -9.9]));
        assert!(!disc.contains([0., 10.1]) && !disc.contains([10.1, 0.]));
        let d = Loop::new(&Boundary {
            vertices: vec![[0., -5., 1.], [0., 5., 0.]],
        });
        assert!(
            d.is_err(),
            "a two-vertex loop with a straight edge is refused"
        );
        let cap = Loop::new(&Boundary {
            vertices: vec![[5., 0., 0.4], [-5., 0., 0.], [0., -3., 0.]],
        })
        .unwrap();
        // Between the chord y = 0 and the arc above it.
        assert!(cap.contains([0., 1.]) && cap.contains([0., -1.]));
        assert!(!cap.contains([0., 2.5]) && !cap.contains([6., 0.]));
        assert_eq!(disc.bounds, [-10., -10., 10., 10.]);
    }
}
