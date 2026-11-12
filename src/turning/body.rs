//! Turning tool bodies off the centre plane (#103).
//!
//! A turning tool's outline is its section on the centre plane through the
//! spindle axis. The real insert has thickness below its cutting face, its
//! sides may taper inward by its clearance angle, the holder's seat may tilt
//! it (rake and inclination), and holders and bars have width across the
//! plane. A body point at (x, y) off the axis lies at radius √(x² + y²): in
//! a bore it reaches farther out than the outline shows.
//!
//! Every turning tool declares its body (`ToolBody`). Its convex pieces,
//! in the tool frame (x radial, z axial, y across the centre plane, the
//! cutting face at y = 0 and the insert body below it), are checked as 3D
//! vertex sets:
//! - against material outside the tool's radius (outside turning: the
//!   part, the stock), by their projection on the XZ plane: a point's
//!   radius is at least its |x|, so the projection is conservative;
//! - against material beyond the tool's radius (in a bore), by their radial
//!   image (z, √(x² + y²)) of the vertices: the radius is a convex function,
//!   so every point of a convex piece is at most the image hull's radius at
//!   its z (exact at vertices, never under-reporting).
//!
//! Sources:
//! - ISO 1832:1985 §3.2: the clearance letter is the normal clearance of
//!   the major cutting edge (N 0, A 3, B 5, C 7, D 15, E 20, F 25, G 30,
//!   P 11 degrees; O other, specified separately).
//! - ISO 1832:1985 §3.3, §3.6: the thickness s is "the distance between the
//!   cutting edge of the corner and the opposing supporting surface"; its
//!   symbol is s in mm disregarding decimals (T1, T3 for 1.98, 3.97 mm), so a
//!   code bounds it only to [n, n + 1); the tolerance class adds ±0.025 mm
//!   (A, F, C, H, E, J, K, L, N) or ±0.13 mm (G, M, U).
//! - ISO 5610 (holder figures): the rake and inclination angles are
//!   "recommended values; they can vary according to the application": a
//!   holder's seat angles are its maker's data and are declared.
//!
//! A grooving blade's flanks (#170). ISO 1832 gives an indexable insert one
//! normal clearance, so one angle is the model of an ISO turning insert. A
//! grooving or parting blade is not an ISO 1832 shape: its maker states the
//! clearance below its front edge and below its sides separately, and the
//! two sides may differ:
//! - Kaiser Tool Company, THINBIT "Little Bit" catalogue, pages "O.D./I.D.
//!   Grooving Inserts" and "Parting Inserts": each insert's relief is listed
//!   per face ("BACK CLEARANCE: 2-1/2° both sides", "SIDE CLEARANCE: 0° both
//!   sides"; parting: "SIDE CLEARANCE: 1°"), separately from its front
//!   angle.
//! - US 10,780,505 B2 (Sandvik Intellectual Property AB, "Face grooving tool
//!   body for metal cutting"): the grooving insert's front surface is a
//!   clearance surface, and its two side surfaces carry a first and a second
//!   clearance surface whose angles differ ("the second clearance angle is
//!   larger than the first clearance angle").
//! - Mitsubishi Materials, "Basic threading" course, "Flank clearance
//!   angle": a threading insert's flank clearances are equal on both sides,
//!   so a threading insert keeps the one angle.
//!
//! Solid (one-piece) boring bars (#114) have no insert: `SolidBarHead`
//! declares the ground head, the bar's cylinder cut by its ground faces
//! (see `solid_bar_vertices`).
//!
//! `BladeClearance` declares them: each flank's plane recedes below the
//! face at its own angle; the rounded corners between them (and every side
//! without an angle of its own) recede at the insert's least clearance,
//! `insert_clearance_deg`, so the checked body contains the real one
//! whatever blend the maker grinds there.
use super::tool::{TurningTool, convex_pieces};
use serde::{Deserialize, Serialize};

/// A turning tool's body off the centre plane. Every value is the caller's
/// (or, for thickness and clearance, an ISO 1832 code's worst case).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ToolBody {
    /// The insert's body and seat: required for every insert but a drill
    /// (a drill on the axis is a solid of revolution: its outline is its
    /// exact radial section).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub insert: Option<InsertBody>,
    /// The holder outline's extent across the centre plane, [low, high] mm
    /// (the cutting face at 0, the insert body toward negative).
    pub holder_y_mm: [f64; 2],
    /// A round boring bar's axis offset across the centre plane (mm).
    #[serde(default)]
    pub bar_axis_y_mm: f64,
    /// A solid (one-piece) boring bar's ground head (#114), instead of an
    /// insert body: the tool has no insert and no seat.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub solid_bar: Option<SolidBarHead>,
    /// The body away from the cut from a supplied model (#110): it replaces
    /// the holder outline's body (and bar) in every check (the planner's
    /// tool geometry takes `ModelBody::pieces`; `body` below still gives
    /// the outline's pieces). None: the outline's.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub model: Option<super::model::ModelBody>,
}

/// The insert's body below its cutting face and the seat that tilts it.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InsertBody {
    /// The insert's thickness below its cutting face (mm).
    pub insert_thickness_mm: f64,
    /// The insert's normal clearance (degrees): its sides taper inward below
    /// the face by thickness x tan(clearance). 0 for a negative insert.
    pub insert_clearance_deg: f64,
    /// The seat's orthogonal rake and inclination (degrees, ISO 3002-1),
    /// about `edge_deg`: negative rake tilts the face up away from the edge
    /// (the flank below the edge backs off); positive inclination raises the
    /// edge away from the nose.
    pub rake_deg: f64,
    pub inclination_deg: f64,
    /// The major cutting edge's direction in the tool frame (degrees from +x
    /// toward +z, leaving the nose): the axis the rake turns about.
    pub edge_deg: f64,
    /// A face grooving blade curved to follow the groove (its maker's
    /// diameter range for it, [min, max] mm): off the centre plane it stays
    /// at the radius of its outline, so its body reaches no farther than
    /// the outline. Every face groove it cuts must lie in the range. None:
    /// a straight blade, checked as its straight body.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub curved_for_diameter_mm: Option<[f64; 2]>,
    /// A grooving or face grooving blade's own front and side clearance
    /// (#170). None: `insert_clearance_deg` on every side. With it,
    /// `insert_clearance_deg` must be the least of its angles (it applies
    /// to the corners between the flanks).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub blade_clearance: Option<BladeClearance>,
}

/// A grooving blade's clearance per flank (degrees), from its maker's
/// catalogue: below its front cutting edge, and below each side.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BladeClearance {
    pub front_deg: f64,
    /// [the side at the blade's reference corner, the other side]: for a
    /// grooving blade its leading corner's side (toward -z for a right-hand
    /// tool) then the trailing one; for a face grooving blade its inner
    /// side (toward the axis) then its outer side.
    pub side_deg: [f64; 2],
}

/// A flank of a grooving blade, in its tool frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BladeFlank {
    Front,
    /// 0: the side at the reference corner; 1: the other side.
    Side(usize),
    /// The rounded corners between the front and the sides.
    Corner,
}

/// A flank of a straight grooving blade's body: its clearance and the
/// body's vertices on it (x, z, y), for naming what a refusal is about.
#[derive(Clone, Debug)]
pub struct FlankPiece {
    pub flank: BladeFlank,
    pub clearance_deg: f64,
    pub points: Vec<[f64; 3]>,
}

/// A tool's body as convex 3D pieces: the insert's, the holder's (and
/// bar's), and a straight grooving blade's flanks.
#[derive(Clone, Debug)]
pub struct Body {
    pub insert: Pieces3,
    pub holder: Pieces3,
    pub flanks: Vec<FlankPiece>,
}

impl ToolBody {
    pub fn validate(&self, id: &str, drill: bool) -> Result<(), String> {
        let finite = |v: f64| v.is_finite() && v.abs() <= 1e4;
        if !(finite(self.holder_y_mm[0]) && finite(self.holder_y_mm[1]))
            || self.holder_y_mm[0] >= self.holder_y_mm[1]
            || !finite(self.bar_axis_y_mm)
        {
            return Err(format!(
                "turning tool {id:?}: its body needs the holder's extent across the centre plane from low to high and a finite bar axis offset"
            ));
        }
        if let Some(model) = &self.model {
            model.validate(id)?;
        }
        if let Some(head) = &self.solid_bar {
            // A solid boring bar (#114): the head is the body.
            if drill || self.insert.is_some() {
                return Err(format!(
                    "turning tool {id:?}: a solid boring bar's head (solid_bar) is its whole body below the cutting face: give no insert body (thickness, clearance, seat), and it is not a drill"
                ));
            }
            return head.validate(id);
        }
        match (&self.insert, drill) {
            (Some(_), true) => Err(format!(
                "turning tool {id:?}: a drill's outline is its exact section (a solid of revolution on the axis): give no insert body"
            )),
            (None, false) => Err(missing_body(id)),
            (None, true) => Ok(()),
            (Some(b), false) => {
                if !(finite(b.insert_thickness_mm) && b.insert_thickness_mm > 0.)
                    || !(0. ..60.).contains(&b.insert_clearance_deg)
                    || !(-45. ..=45.).contains(&b.rake_deg)
                    || !(-45. ..=45.).contains(&b.inclination_deg)
                    || !finite(b.edge_deg)
                    || b.curved_for_diameter_mm
                        .is_some_and(|[lo, hi]| !(finite(lo) && finite(hi) && 0. < lo && lo <= hi))
                {
                    return Err(format!(
                        "turning tool {id:?}: its insert body needs a positive thickness, a clearance 0..60 degrees, seat rake and inclination within ±45 degrees and a finite edge direction"
                    ));
                }
                if let Some(c) = &b.blade_clearance {
                    let angles = [c.front_deg, c.side_deg[0], c.side_deg[1]];
                    if angles.iter().any(|a| !(0. ..60.).contains(a)) {
                        return Err(format!(
                            "turning tool {id:?}: its blade's front and side clearance must each be 0..60 degrees"
                        ));
                    }
                    let least = angles.iter().copied().fold(f64::INFINITY, f64::min);
                    if b.insert_clearance_deg != least {
                        return Err(format!(
                            "turning tool {id:?}: insert_clearance_deg {} contradicts its blade's front clearance {} and side clearance {} and {} degrees: it is the insert's least clearance; give {least} (or only insert_clearance_deg, for one angle on every side)",
                            b.insert_clearance_deg, c.front_deg, c.side_deg[0], c.side_deg[1]
                        ));
                    }
                }
                Ok(())
            }
        }
    }
}

/// An insert's thickness bound and clearance from its ISO 1832 designation:
/// the largest thickness its symbol and tolerance class allow, and the
/// normal clearance of its letter.
pub fn iso_insert_body(code: &str) -> Result<(f64, f64), String> {
    let code = code.trim().to_ascii_uppercase();
    let c: Vec<char> = code.chars().collect();
    if c.len() < 10 {
        return Err(format!(
            "insert code {code:?}: give the full ISO 1832 designation"
        ));
    }
    let clearance = match c[1] {
        'N' => 0.,
        'A' => 3.,
        'B' => 5.,
        'C' => 7.,
        'P' => 11.,
        'D' => 15.,
        'E' => 20.,
        'F' => 25.,
        'G' => 30.,
        other => {
            return Err(format!(
                "insert code {code:?}: clearance {other:?} is not a standard ISO 1832 letter (O means a special clearance): declare insert_clearance_deg"
            ));
        }
    };
    let tolerance = match c[2] {
        'A' | 'F' | 'C' | 'H' | 'E' | 'J' | 'K' | 'L' | 'N' => 0.025,
        'G' | 'M' | 'U' => 0.13,
        other => {
            return Err(format!(
                "insert code {code:?}: tolerance class {other:?} is not an ISO 1832 class: declare insert_thickness_mm"
            ));
        }
    };
    let symbol: String = c[6..8].iter().collect();
    // [n, n + 1) mm by the symbol rule; T1/T3 are 1.98 and 3.97 mm.
    let below = match symbol.as_str() {
        "T1" => 2.,
        "T3" => 4.,
        s if s.chars().all(|d| d.is_ascii_digit()) => {
            let n: u32 = s
                .parse()
                .map_err(|_| format!("insert code {code:?}: thickness"))?;
            if n == 0 {
                return Err(format!("insert code {code:?}: thickness symbol 00"));
            }
            f64::from(n) + 1.
        }
        _ => {
            return Err(format!(
                "insert code {code:?}: letters 7-8 must be an ISO 1832 thickness symbol (01, T1, 02, 03, T3, 04, 05, 06, 07, 09, ...)"
            ));
        }
    };
    Ok((below + tolerance, clearance))
}

/// A convex polygon eroded by `d` (each side moved inward by d): the
/// intersection of the inward half-planes, by clipping. None when nothing
/// is left.
#[cfg(test)]
fn erode_convex(polygon: &[[f64; 2]], d: f64) -> Option<Vec<[f64; 2]>> {
    erode_convex_by(polygon, &vec![d; polygon.len()])
}

/// A convex polygon with side i (from vertex i to the next) moved inward
/// by `d[i]`.
fn erode_convex_by(polygon: &[[f64; 2]], d: &[f64]) -> Option<Vec<[f64; 2]>> {
    let n = polygon.len();
    let area2: f64 = (0..n)
        .map(|i| {
            let (a, b) = (polygon[i], polygon[(i + 1) % n]);
            a[0] * b[1] - b[0] * a[1]
        })
        .sum();
    let ccw = area2 > 0.;
    let mut out = polygon.to_vec();
    for i in 0..n {
        let (a, b) = (polygon[i], polygon[(i + 1) % n]);
        let (ex, ez) = (b[0] - a[0], b[1] - a[1]);
        let len = ex.hypot(ez);
        if len == 0. {
            continue;
        }
        // Inward unit normal.
        let nrm = if ccw {
            [-ez / len, ex / len]
        } else {
            [ez / len, -ex / len]
        };
        let offset = nrm[0] * a[0] + nrm[1] * a[1] + d[i];
        let inside = |p: [f64; 2]| nrm[0] * p[0] + nrm[1] * p[1] - offset;
        let mut clipped = vec![];
        for k in 0..out.len() {
            let (p, q) = (out[k], out[(k + 1) % out.len()]);
            let (sp, sq) = (inside(p), inside(q));
            if sp >= 0. {
                clipped.push(p);
            }
            if (sp >= 0.) != (sq >= 0.) {
                let t = sp / (sp - sq);
                clipped.push([p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])]);
            }
        }
        if clipped.len() < 3 {
            return None;
        }
        out = clipped;
    }
    Some(out)
}

/// The sides of a convex polygon as inward half-planes: for side i (from
/// vertex i to the next) its inward unit normal n and offset c (n·p >= c
/// inside); None for a side of no length.
fn half_planes(polygon: &[[f64; 2]]) -> Vec<Option<([f64; 2], f64)>> {
    let n = polygon.len();
    let area2: f64 = (0..n)
        .map(|i| {
            let (a, b) = (polygon[i], polygon[(i + 1) % n]);
            a[0] * b[1] - b[0] * a[1]
        })
        .sum();
    let ccw = area2 > 0.;
    (0..n)
        .map(|i| {
            let (a, b) = (polygon[i], polygon[(i + 1) % n]);
            let (ex, ez) = (b[0] - a[0], b[1] - a[1]);
            let len = ex.hypot(ez);
            (len != 0.).then(|| {
                let nrm = if ccw {
                    [-ez / len, ex / len]
                } else {
                    [ez / len, -ex / len]
                };
                (nrm, nrm[0] * a[0] + nrm[1] * a[1])
            })
        })
        .collect()
}

/// The vertices (x, z, y) of a convex insert body whose side i recedes
/// below the face at `rate[i]` (the tangent of its clearance): the solid
/// {(p, y): -s <= y <= 0, n_i·p >= c_i + rate_i·(-y) for every side}. Its
/// vertices are the face polygon's (y = 0), the bottom polygon's (y = -s)
/// and, where a side runs out between the two (its neighbours meet below
/// the face), the point where it does: without those the hull of face and
/// bottom is thinner than the body. None when nothing is left of the
/// bottom.
///
/// The section at each depth is the convex polygon of the receded sides.
/// Every side has length at the face, and a side of a convex solid is a
/// convex face, present over one range of depths: a side that has run out
/// never returns. So the sides are followed down from the face: between
/// two events each side's ends move in straight lines (where it meets its
/// neighbours), its length changes linearly, and the next event is the
/// first side to reach no length, at the point its neighbours meet from
/// then on. O(n²) for n sides.
fn tapered_vertices(polygon: &[[f64; 2]], rate: &[f64], s: f64) -> Option<Vec<[f64; 3]>> {
    let bottom = erode_convex_by(polygon, &rate.iter().map(|k| s * k).collect::<Vec<_>>())?;
    let mut vertices: Vec<[f64; 3]> = polygon
        .iter()
        .map(|p| [p[0], p[1], 0.])
        .chain(bottom.iter().map(|p| [p[0], p[1], -s]))
        .collect();
    let scale = polygon
        .iter()
        .fold(s, |m, p| m.max(p[0].abs()).max(p[1].abs()));
    let tol = 1e-9 * scale;
    // A side: inward normal, offset, rate, and its direction along the
    // outline.
    struct Side {
        n: [f64; 2],
        c: f64,
        k: f64,
        d: [f64; 2],
    }
    // A side continuing the one before it on the same line (a straight
    // edge running into the first chord of a circumscribed arc) is one
    // side with it; of two rates on one line the larger governs below the
    // face.
    let same_line = |a: &Side, b: &Side| {
        (a.n[0] * b.n[1] - a.n[1] * b.n[0]).abs() < 1e-9
            && a.n[0] * b.n[0] + a.n[1] * b.n[1] > 0.
            && (a.c - b.c).abs() <= tol
    };
    let mut sides: Vec<Side> = vec![];
    let count = polygon.len();
    for (i, (h, k)) in half_planes(polygon).into_iter().zip(rate).enumerate() {
        let Some((n, c)) = h else { continue };
        let (a, b) = (polygon[i], polygon[(i + 1) % count]);
        let len = (b[0] - a[0]).hypot(b[1] - a[1]);
        let side = Side {
            n,
            c,
            k: *k,
            d: [(b[0] - a[0]) / len, (b[1] - a[1]) / len],
        };
        match sides.last_mut() {
            Some(last) if same_line(last, &side) => last.k = last.k.max(side.k),
            _ => sides.push(side),
        }
    }
    if sides.len() > 1 && same_line(&sides[sides.len() - 1], &sides[0]) {
        let last = sides.pop().unwrap();
        sides[0].k = sides[0].k.max(last.k);
    }
    let m = sides.len();
    if m < 3 {
        return Some(vertices);
    }
    // Where sides a and b meet at depth t: p(t) = p0 + p1 t.
    let corner = |a: usize, b: usize| -> Option<([f64; 2], [f64; 2])> {
        let (a, b) = (&sides[a], &sides[b]);
        let det = a.n[0] * b.n[1] - a.n[1] * b.n[0];
        (det.abs() >= 1e-12).then(|| {
            let solve = |r: f64, q: f64| {
                [
                    (r * b.n[1] - a.n[1] * q) / det,
                    (a.n[0] * q - r * b.n[0]) / det,
                ]
            };
            (solve(a.c, b.c), solve(a.k, b.k))
        })
    };
    // The depth (not above `now`) at which side i runs out between its
    // neighbours a and b, and the point where it does.
    let runs_out = |a: usize, i: usize, b: usize, now: f64| -> Option<(f64, [f64; 2])> {
        let ((a0, a1), (b0, b1)) = (corner(a, i)?, corner(i, b)?);
        let d = sides[i].d;
        let l0 = d[0] * (b0[0] - a0[0]) + d[1] * (b0[1] - a0[1]);
        let l1 = d[0] * (b1[0] - a1[0]) + d[1] * (b1[1] - a1[1]);
        (l1 < -1e-12).then(|| {
            let t = (-l0 / l1).max(now);
            (t, [a0[0] + a1[0] * t, a0[1] + a1[1] * t])
        })
    };
    let mut prev: Vec<usize> = (0..m).map(|i| (i + m - 1) % m).collect();
    let mut next: Vec<usize> = (0..m).map(|i| (i + 1) % m).collect();
    let mut event: Vec<Option<(f64, [f64; 2])>> =
        (0..m).map(|i| runs_out(prev[i], i, next[i], 0.)).collect();
    let mut inner: Vec<[f64; 3]> = vec![];
    let mut left = m;
    while left >= 3 {
        let Some((i, (t, p))) = event
            .iter()
            .enumerate()
            .filter_map(|(i, e)| e.map(|e| (i, e)))
            .min_by(|a, b| a.1.0.total_cmp(&b.1.0))
        else {
            break;
        };
        if t >= s * (1. - 1e-12) {
            break;
        }
        let point = [p[0], p[1], -t];
        if t > 0.
            && !inner
                .iter()
                .any(|q| (0..3).all(|k| (q[k] - point[k]).abs() <= tol))
        {
            inner.push(point);
        }
        let (a, b) = (prev[i], next[i]);
        next[a] = b;
        prev[b] = a;
        event[i] = None;
        left -= 1;
        if left >= 3 {
            event[a] = runs_out(prev[a], a, b, t);
            event[b] = runs_out(a, b, next[b], t);
        }
    }
    vertices.extend(inner);
    Some(vertices)
}

/// `tapered_vertices` by another route, for the tests: each pair of side
/// planes meets in a line; the part of it every other side keeps is an edge
/// of the body, and its ends strictly between the face and the bottom are
/// vertices. O(n³), and it repeats a vertex many sides share.
#[cfg(test)]
fn tapered_vertices_by_lines(polygon: &[[f64; 2]], rate: &[f64], s: f64) -> Vec<[f64; 3]> {
    let bottom = erode_convex_by(polygon, &rate.iter().map(|k| s * k).collect::<Vec<_>>()).unwrap();
    let mut vertices: Vec<[f64; 3]> = polygon
        .iter()
        .map(|p| [p[0], p[1], 0.])
        .chain(bottom.iter().map(|p| [p[0], p[1], -s]))
        .collect();
    let planes: Vec<([f64; 2], f64, f64)> = half_planes(polygon)
        .into_iter()
        .zip(rate)
        .filter_map(|(h, k)| h.map(|(n, c)| (n, c, *k)))
        .collect();
    let scale = polygon
        .iter()
        .fold(s, |m, p| m.max(p[0].abs()).max(p[1].abs()));
    let tol = 1e-9 * scale;
    for (i, (ni, ci, ki)) in planes.iter().enumerate() {
        for (nj, cj, kj) in &planes[i + 1..] {
            let det = ni[0] * nj[1] - ni[1] * nj[0];
            if det.abs() < 1e-9 {
                continue;
            }
            let solve =
                |r: f64, q: f64| [(r * nj[1] - ni[1] * q) / det, (ni[0] * q - r * nj[0]) / det];
            let (a, b) = (solve(*ci, *cj), solve(*ki, *kj));
            let (mut lo, mut hi) = (0., s);
            for (nk, ck, kk) in &planes {
                let g0 = nk[0] * a[0] + nk[1] * a[1] - ck + tol;
                let g1 = nk[0] * b[0] + nk[1] * b[1] - kk;
                if g1.abs() <= 1e-12 {
                    if g0 < 0. {
                        hi = f64::NEG_INFINITY;
                    }
                } else if g1 > 0. {
                    lo = f64::max(lo, -g0 / g1);
                } else {
                    hi = f64::min(hi, -g0 / g1);
                }
            }
            if lo > hi {
                continue;
            }
            for t in [lo, hi] {
                if t > 0. && t < s {
                    vertices.push([a[0] + b[0] * t, a[1] + b[1] * t, -t]);
                }
            }
        }
    }
    vertices
}

/// Convex 3D pieces (x, z, y) of a body.
pub type Pieces3 = Vec<Vec<[f64; 3]>>;

/// The tool's body as convex 3D pieces in its frame (x, z, y), the nose
/// center at the origin, for an outside right-hand tool (the caller mirrors
/// as for the outline): the insert's pieces, then the holder's (and bar's).
pub fn body_pieces(tool: &TurningTool) -> Result<(Pieces3, Pieces3), String> {
    let body = body(tool)?;
    Ok((body.insert, body.holder))
}

/// `body_pieces`, with a straight grooving blade's flanks.
pub fn body(tool: &TurningTool) -> Result<Body, String> {
    use super::tool::Insert;
    let tool_body = tool.body.as_ref().ok_or_else(|| missing_body(&tool.id))?;
    let drill = matches!(tool.insert, Insert::Drill { .. });
    tool_body.validate(&tool.id, drill)?;
    if let Some(head) = &tool_body.solid_bar {
        // A solid boring bar (#114): one convex piece, the ground head.
        let vertices = solid_bar_vertices(tool, head)?;
        let shape = tool.shape()?;
        return Ok(Body {
            insert: vec![vertices],
            holder: holder_pieces(tool, &shape, tool_body),
            flanks: vec![],
        });
    }
    let blade = matches!(
        tool.insert,
        Insert::Groove { .. } | Insert::FaceGroove { .. }
    );
    if !blade
        && tool_body
            .insert
            .as_ref()
            .is_some_and(|b| b.blade_clearance.is_some())
    {
        return Err(blade_only(&tool.id));
    }
    let curved = matches!(tool.insert, Insert::FaceGroove { .. })
        && tool_body
            .insert
            .as_ref()
            .is_some_and(|b| b.curved_for_diameter_mm.is_some());
    let shape = tool.shape()?;
    let polygon = &shape.insert;
    let pieces = convex_pieces(polygon);
    let mut insert: Vec<Vec<[f64; 3]>>;
    let Some(body) = tool_body.insert.as_ref().filter(|_| !curved) else {
        // A drill: its outline is its radial section. A curved face
        // grooving blade: it follows the groove's radius off the plane.
        insert = pieces
            .iter()
            .map(|piece| piece.iter().map(|p| [p[0], p[1], 0.]).collect())
            .collect();
        return Ok(Body {
            insert,
            holder: holder_pieces(tool, &shape, tool_body),
            flanks: vec![],
        });
    };
    let s = body.insert_thickness_mm;
    // A blade's sides by their outward direction in its frame: the front
    // toward -x, the top (where the blade goes on into its holder) toward
    // +x, the sides toward -z (the reference corner's) and +z; the rounded
    // corners are every other side.
    let planes = half_planes(polygon);
    let flank_of = |i: usize| -> Option<BladeFlank> {
        let (n, _) = planes[i]?;
        let axis = |v: f64| v > 1. - 1e-12;
        Some(if axis(n[0]) {
            BladeFlank::Front
        } else if axis(-n[0]) {
            return None;
        } else if axis(n[1]) {
            BladeFlank::Side(0)
        } else if axis(-n[1]) {
            BladeFlank::Side(1)
        } else {
            BladeFlank::Corner
        })
    };
    let angle_of = |flank: Option<BladeFlank>| match (&body.blade_clearance, flank) {
        (Some(c), Some(BladeFlank::Front)) => c.front_deg,
        (Some(c), Some(BladeFlank::Side(k))) => c.side_deg[k],
        // The corners (whatever blend is ground between the flanks) and
        // the top: the insert's least clearance.
        _ => body.insert_clearance_deg,
    };
    let mut flanks = vec![];
    insert = if pieces.len() == 1 {
        let no_bottom = || {
            format!(
                "turning tool {:?}: its clearance taper ({} degrees over {s} mm) leaves no insert bottom",
                tool.id, body.insert_clearance_deg
            )
        };
        let rate: Vec<f64> = (0..polygon.len())
            .map(|i| {
                let flank = if blade { flank_of(i) } else { None };
                angle_of(flank).to_radians().tan()
            })
            .collect();
        // Every vertex of the tapering body, those where a side runs out
        // below the face included (a nose arc does wherever the taper
        // exceeds its radius): the hull of the face and the bottom alone is
        // thinner than the body there.
        let vertices = tapered_vertices(polygon, &rate, s).ok_or_else(no_bottom)?;
        if blade {
            // Each flank: the body's vertices on its planes, with a point
            // inside the body (its vertices' mean) so that a flank seen
            // edge-on still has an image with area. The piece lies in the
            // body, so it reaches no farther than the body does, and as far
            // only through a vertex on the flank.
            let count = vertices.len() as f64;
            let middle = vertices.iter().fold([0.; 3], |m, v| {
                [
                    m[0] + v[0] / count,
                    m[1] + v[1] / count,
                    m[2] + v[2] / count,
                ]
            });
            let scale = polygon
                .iter()
                .fold(s, |m, p| m.max(p[0].abs()).max(p[1].abs()));
            let classes: Vec<Option<BladeFlank>> = (0..polygon.len()).map(flank_of).collect();
            for flank in [
                BladeFlank::Front,
                BladeFlank::Side(0),
                BladeFlank::Side(1),
                BladeFlank::Corner,
            ] {
                let on: Vec<[f64; 3]> = vertices
                    .iter()
                    .filter(|v| {
                        (0..polygon.len()).any(|i| {
                            classes[i] == Some(flank)
                                && planes[i].is_some_and(|(n, c)| {
                                    (n[0] * v[0] + n[1] * v[1] - c + rate[i] * v[2]).abs()
                                        <= 1e-7 * scale
                                })
                        })
                    })
                    .copied()
                    .collect();
                if on.len() >= 3 {
                    flanks.push(FlankPiece {
                        flank,
                        clearance_deg: angle_of(Some(flank)),
                        points: on.into_iter().chain([middle]).collect(),
                    });
                }
            }
        }
        vec![vertices]
    } else {
        // A concave outline: its convex pieces extruded straight (a prism
        // contains a tapering body).
        pieces
            .iter()
            .map(|piece| {
                piece
                    .iter()
                    .flat_map(|p| [[p[0], p[1], 0.], [p[0], p[1], -s]])
                    .collect()
            })
            .collect()
    };
    // The seat: rake about the major edge, then inclination about the
    // in-plane direction across it (into the insert), both about the nose.
    let e = [
        body.edge_deg.to_radians().cos(),
        body.edge_deg.to_radians().sin(),
    ];
    let centroid = polygon
        .iter()
        .fold([0., 0.], |c, p| [c[0] + p[0], c[1] + p[1]]);
    let mut q = [-e[1], e[0]];
    if q[0] * centroid[0] + q[1] * centroid[1] < 0. {
        q = [-q[0], -q[1]];
    }
    let lambda = body.inclination_deg.to_radians();
    // ISO 3002-1: tan(normal rake) = tan(orthogonal rake) cos(inclination).
    let gamma_n = (body.rake_deg.to_radians().tan() * lambda.cos()).atan();
    let (theta1, theta2) = (-gamma_n, lambda);
    let seat = |v: &mut [f64; 3]| {
        let (a, b, c) = (v[0] * e[0] + v[1] * e[1], v[0] * q[0] + v[1] * q[1], v[2]);
        let (b, c) = (
            b * theta1.cos() - c * theta1.sin(),
            b * theta1.sin() + c * theta1.cos(),
        );
        let (a, c) = (
            a * theta2.cos() - c * theta2.sin(),
            a * theta2.sin() + c * theta2.cos(),
        );
        *v = [a * e[0] + b * q[0], a * e[1] + b * q[1], c];
    };
    for piece in &mut insert {
        piece.iter_mut().for_each(seat);
    }
    for flank in &mut flanks {
        flank.points.iter_mut().for_each(seat);
    }
    Ok(Body {
        insert,
        holder: holder_pieces(tool, &shape, tool_body),
        flanks,
    })
}

fn holder_pieces(
    tool: &TurningTool,
    shape: &super::tool::ToolShape,
    body: &ToolBody,
) -> Vec<Vec<[f64; 3]>> {
    let [y0, y1] = body.holder_y_mm;
    let mut holder: Vec<Vec<[f64; 3]>> = convex_pieces(&shape.holder)
        .iter()
        .map(|piece| {
            piece
                .iter()
                .flat_map(|p| [[p[0], p[1], y0], [p[0], p[1], y1]])
                .collect()
        })
        .collect();
    if let (Some(bar), Some(section)) = (&tool.holder.bar, &shape.bar) {
        // The round bar (axis parallel to z): a circumscribed 32-gon prism.
        let r = bar.diameter_mm / 2. / (std::f64::consts::PI / 32.).cos();
        let x = (section[0][0] + section[1][0]) / 2.;
        let (z0, z1) = (section[0][1], section[2][1]);
        holder.push(
            (0..32)
                .flat_map(|k| {
                    let t = std::f64::consts::TAU * (k as f64 + 0.5) / 32.;
                    let (px, py) = (x + r * t.cos(), body.bar_axis_y_mm + r * t.sin());
                    [[px, z0, py], [px, z1, py]]
                })
                .collect(),
        );
    }
    holder
}

// Solid (one-piece) boring bars (#114).
//
// A solid carbide boring bar, a brazed tool or a ground HSS bar has no
// insert: its tip is ground into the end of a round bar. Its head below the
// cutting face is the bar's cylinder cut by the ground faces, which narrows
// toward the bottom; a prism of the head's outline (an `InsertBody` of the
// head's depth) keeps its full-width corners at full depth, and is refused
// in the small bores these tools are made for.
//
// The model: the head is
//   {(x, z, y): y <= 0 (the cutting face),
//    (x - ax)² + (y - ay)² <= (d/2)² (the bar's cylinder, axis along z),
//    n_i·(x, z) >= c_i + tan(clearance_i)·(-y) for each side i of the
//    head's outline at the cutting face (the ground face below that side,
//    receding at its clearance angle)},
// an intersection of convex sets. The checks need a convex polytope that
// contains it (the radius √(x² + y²) is convex, so over a polytope it is
// greatest at a vertex; a curved face has no vertices to check): the
// cylinder is replaced by the prism of a regular polygon circumscribed
// about its circle (every side tangent to it, so the prism contains the
// cylinder), and the outline's arcs by their circumscribed polygon as for
// every insert. The polytope's vertices are found by clipping the prism's
// faces by each plane in turn.
//
// Source: KYOCERA, "EZ Bar: Series for Small Internal Machining" catalogue
// (2022). Page 12, "EZ Bar Dimensions ST", tabulates per bar DMIN (min.
// bore dia.), DCON (shank), H, LF, LU, WF, WF2 and RE, e.g. EZBR
// 055050ST-008H: DMIN 5.5, DCON 5.0, H 4.80, LF 58.2, LU 25.0, WF 2.44,
// WF2 0.33, RE 0.08 ±0.015 mm, "Tolerance: Offset ±0.06mm, overall length
// ±0.1mm, edge height +0.06/0mm". Page 8, "Bar Tolerances": Offset (WF),
// Longitudinal Direction (L), Cutting Edge Height (Y) and "Min. Bore Dia."
// ("Same as Shank Dia." for the HP bars, "Different from Shank Dia." for
// the ST bars). The tables give no clearance angle of the ground faces and
// no outline of the head (the drawings show a 5 degree end cutting edge
// angle): both are declared.

/// A solid (one-piece) boring bar's ground head (#114). Every value is the
/// caller's, from the tool's catalogue or drawing.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SolidBarHead {
    /// The diameter of the round the head is ground from (mm): the bar's,
    /// where the head keeps the bar's surface (the catalogue's shank
    /// diameter DCON), or of any cylinder along the bar that contains the
    /// head below its cutting face.
    pub diameter_mm: f64,
    /// That cylinder's axis in the insert outline's frame: its x (the
    /// catalogue's WF, or f, is the distance from the bar's centreline to
    /// the cutting edge) and its offset across the centre plane (the
    /// cutting face at 0: minus the catalogue's cutting edge height above
    /// the bar's centre).
    pub axis_x_mm: f64,
    pub axis_y_mm: f64,
    /// For each segment of the head's outline, the angle (degrees) at which
    /// the ground face below it recedes under the cutting face: the end
    /// clearance below the end cutting edge, the side clearance below the
    /// side cutting edge, at most the lesser of the two below the nose
    /// between them, 0 where the head is not relieved (the cylinder alone
    /// bounds it there). Each angle declares that the whole head lies
    /// behind that plane. An angle not known is declared 0: the body then
    /// contains the real one.
    pub clearance_deg: Vec<f64>,
    /// The catalogue's minimum bore diameter (DMIN): no cut in a smaller
    /// bore, and the head itself must enter it.
    pub min_bore_diameter_mm: f64,
    /// The catalogue's maximum bore depth (usable length LU), from the
    /// head's front end: no move deeper into the stock.
    pub max_bore_depth_mm: f64,
}

/// The step (degrees) of the polygon circumscribed about the head's
/// cylinder: it stands at most d/2 x (1 / cos 1° - 1) = 0.000076 d outside.
const HEAD_STEP_DEG: f64 = 2.;

impl SolidBarHead {
    pub fn validate(&self, id: &str) -> Result<(), String> {
        let positive = |v: f64| v.is_finite() && v > 0. && v <= 1e4;
        if !positive(self.diameter_mm)
            || !(self.axis_x_mm.is_finite() && self.axis_x_mm.abs() <= 1e4)
            || !(self.axis_y_mm.is_finite() && self.axis_y_mm.abs() < self.diameter_mm / 2.)
        {
            return Err(format!(
                "turning tool {id:?}: its solid head needs a positive diameter and its axis (axis_x_mm, and axis_y_mm within the bar's radius of the cutting face: the cutting face must cut the bar)"
            ));
        }
        if self.clearance_deg.iter().any(|a| !(0. ..60.).contains(a)) {
            return Err(format!(
                "turning tool {id:?}: its solid head's clearance angles must each be 0..60 degrees"
            ));
        }
        if !positive(self.min_bore_diameter_mm) || !positive(self.max_bore_depth_mm) {
            return Err(format!(
                "turning tool {id:?}: its solid head needs the catalogue's minimum bore diameter (min_bore_diameter_mm) and maximum bore depth (max_bore_depth_mm), both positive"
            ));
        }
        Ok(())
    }
}

/// A regular `n`-gon circumscribed about the circle of `radius` around
/// `center`: its sides are tangent to the circle (at the angles k x 360/n,
/// so the circle's extremes along both axes are on it when 4 divides n) and
/// its corners lie radius / cos(π/n) out, so it contains the disc.
fn circumscribed_polygon(center: [f64; 2], radius: f64, n: usize) -> Vec<[f64; 2]> {
    let r = radius / (std::f64::consts::PI / n as f64).cos();
    (0..n)
        .map(|k| {
            let t = std::f64::consts::TAU * (k as f64 + 0.5) / n as f64;
            [center[0] + r * t.cos(), center[1] + r * t.sin()]
        })
        .collect()
}

/// Clip a convex polyhedron, given as its faces (each a convex polygon in
/// space), by the half-space n·p <= c (n a unit normal): each face is
/// clipped as a polygon, and the points on the plane close the cut with a
/// new face. A point within `eps` of the plane is on it.
fn clip_polyhedron(faces: &mut Vec<Vec<[f64; 3]>>, n: [f64; 3], c: f64, eps: f64) {
    let dist = |p: &[f64; 3]| n[0] * p[0] + n[1] * p[1] + n[2] * p[2] - c;
    let mut cap: Vec<[f64; 3]> = vec![];
    let mut cut = false;
    for face in faces.iter_mut() {
        let d: Vec<f64> = face.iter().map(dist).collect();
        let m = face.len();
        let mut out = Vec::with_capacity(m + 2);
        for k in 0..m {
            let (p, q) = (face[k], face[(k + 1) % m]);
            let (sp, sq) = (d[k], d[(k + 1) % m]);
            if sp <= eps {
                out.push(p);
                if sp >= -eps {
                    cap.push(p);
                }
            } else {
                cut = true;
            }
            if (sp < -eps && sq > eps) || (sp > eps && sq < -eps) {
                // From the end that is kept, so both faces sharing this
                // edge compute the same point.
                let (a, b, sa, sb) = if sp < 0. {
                    (p, q, sp, sq)
                } else {
                    (q, p, sq, sp)
                };
                let t = sa / (sa - sb);
                let x = [
                    a[0] + t * (b[0] - a[0]),
                    a[1] + t * (b[1] - a[1]),
                    a[2] + t * (b[2] - a[2]),
                ];
                out.push(x);
                cap.push(x);
            }
        }
        *face = out;
    }
    faces.retain(|f| f.len() >= 3);
    if !cut || cap.len() < 3 {
        return;
    }
    // The cut is the convex polygon of the points on the plane: in order
    // around their mean.
    let count = cap.len() as f64;
    let mean = cap.iter().fold([0.; 3], |m, p| {
        [
            m[0] + p[0] / count,
            m[1] + p[1] / count,
            m[2] + p[2] / count,
        ]
    });
    let k = (0..3)
        .min_by(|a, b| n[*a].abs().total_cmp(&n[*b].abs()))
        .unwrap();
    let mut u = [0.; 3];
    u[k] = 1.;
    let along = u[0] * n[0] + u[1] * n[1] + u[2] * n[2];
    let u = [
        u[0] - along * n[0],
        u[1] - along * n[1],
        u[2] - along * n[2],
    ];
    let len = (u[0] * u[0] + u[1] * u[1] + u[2] * u[2]).sqrt();
    let u = [u[0] / len, u[1] / len, u[2] / len];
    let w = [
        n[1] * u[2] - n[2] * u[1],
        n[2] * u[0] - n[0] * u[2],
        n[0] * u[1] - n[1] * u[0],
    ];
    let mut around: Vec<(f64, [f64; 3])> = cap
        .into_iter()
        .map(|p| {
            let v = [p[0] - mean[0], p[1] - mean[1], p[2] - mean[2]];
            let (a, b) = (
                v[0] * u[0] + v[1] * u[1] + v[2] * u[2],
                v[0] * w[0] + v[1] * w[1] + v[2] * w[2],
            );
            (b.atan2(a), p)
        })
        .collect();
    around.sort_by(|a, b| a.0.total_cmp(&b.0));
    let same = |a: &[f64; 3], b: &[f64; 3]| (0..3).all(|k| (a[k] - b[k]).abs() <= eps);
    let mut polygon: Vec<[f64; 3]> = vec![];
    for (_, p) in around {
        if !polygon.last().is_some_and(|q| same(q, &p)) {
            polygon.push(p);
        }
    }
    if polygon.len() > 1 && same(&polygon[0], polygon.last().unwrap()) {
        polygon.pop();
    }
    if polygon.len() >= 3 {
        faces.push(polygon);
    }
}

/// A rounded corner of a solid head's outline with a clearance of its own:
/// the model lets every tangent plane of the corner recede at that angle,
/// each as a whole half-space, while the real corner is ground only within
/// its own sector. The model then contains the real head when the corner
/// lies between two straight sides tangent to it whose own clearance is at
/// least the corner's. A point p of the real head in the corner's sector
/// is within the receded radius of the corner's center, so behind every
/// tangent plane. A point outside the sector is behind the planes of both
/// sides, whose normals are the corner's two end normals and which recede
/// at least as fast as the corner; from p's direction the shorter way
/// round to any direction d in the sector passes one of those end normals
/// e, so d·p <= e·p (the cosine falls with the angle up to a half turn),
/// and p is behind d's plane as it is behind e's. Anything else is
/// refused; a corner declared 0 is the outline's own prism there and
/// always contains the head.
fn rounded_corners_recede_soundly(tool: &TurningTool, head: &SolidBarHead) -> Result<(), String> {
    use super::tool::{Insert, Segment};
    let Insert::Outline { outline, .. } = &tool.insert else {
        return Ok(());
    };
    let segments = &outline.segments;
    let count = segments.len();
    let dir = |deg: f64| [deg.to_radians().cos(), deg.to_radians().sin()];
    let end_of = |k: usize| match &segments[k] {
        Segment::Line { to_mm } => *to_mm,
        Segment::Arc {
            center_mm,
            radius_mm,
            end_deg,
            ..
        } => {
            let d = dir(*end_deg);
            [
                center_mm[0] + radius_mm * d[0],
                center_mm[1] + radius_mm * d[1],
            ]
        }
    };
    for (k, segment) in segments.iter().enumerate() {
        let Segment::Arc {
            radius_mm,
            start_deg,
            end_deg,
            ..
        } = segment
        else {
            continue;
        };
        let angle = head.clearance_deg[k];
        if angle == 0. || *radius_mm == 0. {
            continue;
        }
        // The straight side before the corner (tangent where it starts)
        // and the one after it (tangent where it ends).
        let tangent = |j: usize, at_deg: f64| {
            let Segment::Line { to_mm } = &segments[j] else {
                return false;
            };
            let from = if j == 0 {
                outline.start_mm
            } else {
                end_of(j - 1)
            };
            let (ex, ez) = (to_mm[0] - from[0], to_mm[1] - from[1]);
            let radial = dir(at_deg);
            head.clearance_deg[j] >= angle
                && (ex * radial[0] + ez * radial[1]).abs() <= 1e-6 * ex.hypot(ez)
        };
        if !tangent((k + count - 1) % count, *start_deg) || !tangent((k + 1) % count, *end_deg) {
            return Err(format!(
                "turning tool {:?}: its solid head's clearance below the rounded corner (outline segment {k}: {angle} degrees) must be at most the clearance below each of the two straight sides it joins, tangent to it: camlib checks every tangent plane of the corner as receding at that angle, which contains the real head only then. Give the lesser of the two sides' angles, or 0",
                tool.id
            ));
        }
    }
    Ok(())
}

/// The vertices (x, z, y) of the polytope containing a solid boring bar's
/// head, the nose center at the origin, drawn as an outside right-hand tool
/// (the caller mirrors): the prism of the polygon circumscribed about the
/// bar's circle, below the cutting face, cut by the plane below each side
/// of the head's outline. Refuses a declaration that contradicts itself:
/// an outline off the bar's section, or a minimum bore the head cannot
/// enter.
pub(crate) fn solid_bar_vertices(
    tool: &TurningTool,
    head: &SolidBarHead,
) -> Result<Vec<[f64; 3]>, String> {
    let id = &tool.id;
    head.validate(id)?;
    let outline = super::tool::head_outline(&tool.insert)
        .filter(|_| tool.side == super::stock::TurnSide::Inner)
        .ok_or_else(|| {
            format!(
                "turning tool {id:?}: a solid bar's head (solid_bar) is a boring head: an outline insert (the head's outline at its cutting face) on side inner. Solid grooving, face grooving and threading bars are not modelled as solid heads (#114): declare such a blade as a grooving, face grooving or threading insert with its insert body"
            )
        })?;
    if head.clearance_deg.len() != outline.segments {
        return Err(format!(
            "turning tool {id:?}: its solid head's clearance_deg has {} angles for an outline of {} segments: give one per segment, the angle at which the ground face below that side recedes (0 where the head is not relieved)",
            head.clearance_deg.len(),
            outline.segments
        ));
    }
    if convex_pieces(&outline.points).len() != 1 {
        return Err(format!(
            "turning tool {id:?}: a solid head's outline at its cutting face must be convex: draw the head only (its neck and shank belong to the holder and its bar)"
        ));
    }
    rounded_corners_recede_soundly(tool, head)?;
    let r = head.diameter_mm / 2.;
    let axis = [head.axis_x_mm - outline.nose[0], head.axis_y_mm];
    let scale = outline.points.iter().fold(r.max(axis[0].abs()), |m, p| {
        m.max(p[0].abs()).max(p[1].abs())
    });
    let tol = 1e-9 * scale;
    // The cylinder's section at the cutting face: a strip along z.
    let half = (r * r - axis[1] * axis[1]).sqrt();
    if outline.x_mm[0] < axis[0] - half - tol || outline.x_mm[1] > axis[0] + half + tol {
        return Err(format!(
            "turning tool {id:?}: its head's outline reaches from x {:.4} to {:.4} mm at the cutting face, outside the bar's section there (x {:.4} to {:.4} mm: a {} mm bar, its axis at x {} mm and {} mm across the centre plane): the cutting edge must lie on the bar. Correct axis_x_mm (the catalogue's f or WF is the distance from the bar's centreline to the cutting edge), the diameter or the outline",
            outline.x_mm[0] + outline.nose[0],
            outline.x_mm[1] + outline.nose[0],
            head.axis_x_mm - half,
            head.axis_x_mm + half,
            head.diameter_mm,
            head.axis_x_mm,
            head.axis_y_mm
        ));
    }
    // The prism of the circumscribed polygon, past both ends of the
    // outline (its end faces are cut away by the outline's sides).
    let (z0, z1) = outline
        .points
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
            (lo.min(p[1]), hi.max(p[1]))
        });
    let (z0, z1) = (z0 - scale, z1 + scale);
    let section = circumscribed_polygon(axis, r, (360. / HEAD_STEP_DEG) as usize);
    let count = section.len();
    let mut faces: Vec<Vec<[f64; 3]>> = (0..count)
        .map(|k| {
            let (a, b) = (section[k], section[(k + 1) % count]);
            vec![
                [a[0], z0, a[1]],
                [b[0], z0, b[1]],
                [b[0], z1, b[1]],
                [a[0], z1, a[1]],
            ]
        })
        .collect();
    for z in [z0, z1] {
        faces.push(section.iter().map(|p| [p[0], z, p[1]]).collect());
    }
    // Below the cutting face.
    clip_polyhedron(&mut faces, [0., 0., 1.], 0., tol);
    // Below each side of the outline, the ground face receding at its
    // clearance: n·(x, z) >= c + tan(clearance)·(-y).
    for (i, plane) in half_planes(&outline.points).into_iter().enumerate() {
        let Some((n, c)) = plane else { continue };
        let rate = head.clearance_deg[outline.owners[i]].to_radians().tan();
        let len = (1. + rate * rate).sqrt();
        clip_polyhedron(
            &mut faces,
            [-n[0] / len, -n[1] / len, -rate / len],
            -c / len,
            tol,
        );
    }
    let mut vertices: Vec<[f64; 3]> = faces.into_iter().flatten().collect();
    vertices.sort_by(|a, b| {
        a[0].total_cmp(&b[0])
            .then(a[1].total_cmp(&b[1]))
            .then(a[2].total_cmp(&b[2]))
    });
    vertices.dedup_by(|a, b| (0..3).all(|k| (a[k] - b[k]).abs() <= tol));
    if vertices.len() < 4 {
        return Err(format!(
            "turning tool {id:?}: its solid head's bar and clearance angles leave no body below the cutting face"
        ));
    }
    // The declared minimum bore against the head: with the cutting edge
    // (the outline's point nearest the wall) on a bore of radius R, a body
    // point `a` behind the edge and y below the face lies at radius
    // √((R - a)² + y²), greatest at a vertex. The polygons stand outside
    // the true head by at most `allowance`.
    let edge = outline.x_mm[0];
    let excess = |bore: f64| {
        vertices
            .iter()
            .map(|v| (bore - (v[0] - edge)).hypot(v[2]) - bore)
            .fold(f64::NEG_INFINITY, f64::max)
    };
    let allowance =
        (r + outline.arc_radius_mm) * (1. / (HEAD_STEP_DEG / 2.).to_radians().cos() - 1.) + tol;
    let over = excess(head.min_bore_diameter_mm / 2.);
    if over > allowance {
        // The excess never grows with the bore: the least bore the head
        // enters, by bisection.
        let (mut lo, mut hi) = (head.min_bore_diameter_mm / 2., 1e4);
        let least = if excess(hi) > allowance {
            "it enters no bore".to_string()
        } else {
            for _ in 0..60 {
                let mid = (lo + hi) / 2.;
                if excess(mid) > allowance {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            format!("the least bore it enters is {:.3} mm", 2. * hi)
        };
        return Err(format!(
            "turning tool {id:?}: its declared minimum bore of {} mm contradicts its head: with the cutting edge on that bore's wall the head's body below the cutting face reaches {over:.4} mm beyond the wall ({least}). Correct min_bore_diameter_mm (the catalogue's minimum bore diameter), or the head's diameter, axis and clearance angles",
            head.min_bore_diameter_mm
        ));
    }
    Ok(vertices)
}

/// Dense points of a solid boring bar's true head, for the tests: built
/// from the outline's segments, the true cylinder and the receding faces
/// directly (not from the polytope or its polygons), in the tool's frame
/// with the nose center at the origin. A grid of `step` mm through the
/// body, and points on the cylinder's surface.
#[cfg(test)]
pub(crate) fn solid_head_samples(tool: &TurningTool, step: f64) -> Vec<[f64; 3]> {
    use super::tool::{Insert, Segment};
    let head = tool.body.as_ref().unwrap().solid_bar.as_ref().unwrap();
    let Insert::Outline {
        outline, corner, ..
    } = &tool.insert
    else {
        panic!("an outline insert");
    };
    let Segment::Arc {
        center_mm: nose, ..
    } = outline.segments[*corner]
    else {
        panic!("the corner is an arc");
    };
    let at = |c: [f64; 2], r: f64, deg: f64| {
        [
            c[0] + r * deg.to_radians().cos(),
            c[1] + r * deg.to_radians().sin(),
        ]
    };
    // Each segment's ends.
    let mut from = outline.start_mm;
    let mut ends = vec![];
    for s in &outline.segments {
        let (a, b) = match s {
            Segment::Line { to_mm } => (from, *to_mm),
            Segment::Arc {
                center_mm,
                radius_mm,
                start_deg,
                end_deg,
            } => (
                at(*center_mm, *radius_mm, *start_deg),
                at(*center_mm, *radius_mm, *end_deg),
            ),
        };
        ends.push((a, b));
        from = b;
    }
    let area2: f64 = ends.iter().map(|(a, b)| a[0] * b[1] - b[0] * a[1]).sum();
    let ccw = area2 > 0.;
    let (r, axis) = (head.diameter_mm / 2., [head.axis_x_mm, head.axis_y_mm]);
    // Whether the point p (the outline's frame) at depth t below the face
    // is in the head, the cylinder apart.
    let ground = |p: [f64; 2], t: f64| {
        outline
            .segments
            .iter()
            .zip(&ends)
            .zip(&head.clearance_deg)
            .all(|((s, (a, b)), clearance)| {
                let back = clearance.to_radians().tan() * t;
                match s {
                    Segment::Line { .. } => {
                        let (ex, ez) = (b[0] - a[0], b[1] - a[1]);
                        let len = ex.hypot(ez);
                        let cross = ex * (p[1] - a[1]) - ez * (p[0] - a[0]);
                        (if ccw { cross } else { -cross }) / len >= back
                    }
                    Segment::Arc {
                        center_mm,
                        radius_mm,
                        start_deg,
                        end_deg,
                    } => {
                        // The corner is ground within its own sector only:
                        // there, the head lies within the receded radius.
                        let (dx, dz) = (p[0] - center_mm[0], p[1] - center_mm[1]);
                        let phi = dz.atan2(dx).to_degrees();
                        let (lo, hi) = (start_deg.min(*end_deg), start_deg.max(*end_deg));
                        let within = (-2..=2).any(|k| {
                            let phi = phi + 360. * k as f64;
                            lo <= phi && phi <= hi
                        });
                        !within || dx.hypot(dz) <= radius_mm - back
                    }
                }
            })
    };
    let mut points = vec![];
    // The cutting face's own outline.
    for (s, (a, b)) in outline.segments.iter().zip(&ends) {
        for k in 0..=200 {
            let t = k as f64 / 200.;
            let p = match s {
                Segment::Line { .. } => [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])],
                Segment::Arc {
                    center_mm,
                    radius_mm,
                    start_deg,
                    end_deg,
                } => at(
                    *center_mm,
                    *radius_mm,
                    start_deg + t * (end_deg - start_deg),
                ),
            };
            points.push([p[0] - nose[0], p[1] - nose[1], 0.]);
        }
    }
    let (x0, x1) = (axis[0] - r, axis[0] + r);
    let (z0, z1) = ends
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), (a, _)| {
            (lo.min(a[1]), hi.max(a[1]))
        });
    let (z0, z1) = (z0 - 1., z1 + 1.);
    let count = |a: f64, b: f64| ((b - a) / step).ceil() as usize;
    for k in 0..=count(z0, z1) {
        let z = z0 + (z1 - z0) * k as f64 / count(z0, z1) as f64;
        for i in 0..=count(x0, x1) {
            let x = x0 + (x1 - x0) * i as f64 / count(x0, x1) as f64;
            for j in 0..=count(0., r - axis[1]) {
                let t = (r - axis[1]) * j as f64 / count(0., r - axis[1]) as f64;
                if (x - axis[0]).hypot(-t - axis[1]) <= r && ground([x, z], t) {
                    points.push([x - nose[0], z - nose[1], -t]);
                }
            }
        }
        // On the cylinder's surface.
        for a in 0..1440 {
            let a = std::f64::consts::TAU * a as f64 / 1440.;
            let (x, y) = (axis[0] + r * a.cos(), axis[1] + r * a.sin());
            if y <= 0. && ground([x, z], -y) {
                points.push([x - nose[0], z - nose[1], y]);
            }
        }
    }
    points
}

/// A synthetic solid boring bar for the tests, with the dimension set of
/// a catalogue's Ø5 bar (KYOCERA EZBR 055050ST: DCON 5.0, WF 2.44, DMIN
/// 5.5 mm): a head 6 mm long ground from the Ø5 round, its cutting face on
/// the bar's centre, a 0.1 mm nose, the side cutting edge with 2 degrees of
/// back taper and `side_clearance_deg` below it, the end cutting edge at 5
/// degrees with 8 degrees below it, the nose the lesser of the two, the
/// rest the bar's own surface; the plain bar behind it, in a sleeve.
#[cfg(test)]
pub(crate) fn test_solid_bar(side_clearance_deg: f64, min_bore_diameter_mm: f64) -> TurningTool {
    use super::stock::TurnSide;
    use super::tool::{Bar, Hand, Holder, Insert, Outline, Segment};
    let rn = 0.1;
    let at = |deg: f64| {
        let a = f64::to_radians(deg);
        [rn * a.cos(), rn * a.sin()]
    };
    let (side, end) = (at(178.), at(275.));
    let along = |p: [f64; 2], deg: f64, t: f64| {
        let a = f64::to_radians(deg);
        [p[0] + t * a.cos(), p[1] + t * a.sin()]
    };
    // The side edge up to z = 6 (88 degrees: toward +z, 2 degrees toward
    // +x); the end edge out to x = 4.8 (5 degrees toward +z).
    let top = along(side, 88., (6. - side[1]) / 88f64.to_radians().sin());
    let far = along(end, 5., (4.8 - end[0]) / 5f64.to_radians().cos());
    let line = |to_mm: [f64; 2]| Segment::Line { to_mm };
    let insert = Insert::Outline {
        outline: Outline {
            start_mm: top,
            segments: vec![
                line(side),
                Segment::Arc {
                    center_mm: [0., 0.],
                    radius_mm: rn,
                    start_deg: 178.,
                    end_deg: 275.,
                },
                line(far),
                line([4.8, 6.]),
                line(top),
            ],
        },
        corner: 1,
        cutting: vec![0, 1, 2],
        programming_point_mm: [-rn, -rn],
    };
    let sleeve = |x0: f64, x1: f64, z0: f64, z1: f64| Outline {
        start_mm: [x0, z0],
        segments: vec![
            line([x1, z0]),
            line([x1, z1]),
            line([x0, z1]),
            line([x0, z0]),
        ],
    };
    TurningTool {
        id: "solid".into(),
        insert,
        nose_radius_mm: rn,
        side: TurnSide::Inner,
        hand: Hand::Right,
        holder: Holder {
            outline: sleeve(-5., 10., 30., 60.),
            bar: Some(Bar {
                diameter_mm: 5.,
                axis_x_mm: 2.34,
                z_mm: [6., 30.],
            }),
        },
        body: Some(ToolBody {
            insert: None,
            holder_y_mm: [-8., 8.],
            bar_axis_y_mm: 0.,
            solid_bar: Some(SolidBarHead {
                diameter_mm: 5.,
                // The edge at x -0.1, WF 2.44 from the bar's centreline.
                axis_x_mm: 2.34,
                axis_y_mm: 0.,
                clearance_deg: vec![side_clearance_deg, side_clearance_deg.min(8.), 8., 0., 0.],
                min_bore_diameter_mm,
                max_bore_depth_mm: 6.,
            }),
            model: None,
        }),
    }
}

pub(crate) fn blade_only(id: &str) -> String {
    format!(
        "turning tool {id:?}: a front and side clearance of their own (blade_clearance) are a grooving or face grooving blade's; this insert has one normal clearance (ISO 1832): give insert_clearance_deg only"
    )
}

pub(crate) fn missing_body(id: &str) -> String {
    format!(
        "turning tool {id:?} needs its body (#103): the insert's thickness and clearance (from its ISO code, or declared), the holder's seat rake and inclination (its maker's data; 0 and 0 for a flat seat) and the holder's extent across the centre plane. camlib assumes no tool is flat"
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::turning::stock::TurnSide;
    use crate::turning::tool::{Hand, Holder, Insert};

    #[test]
    fn iso_codes_give_worst_case_thickness_and_clearance() {
        let (s, a) = iso_insert_body("CNMG120408").unwrap();
        assert!((s - 5.13).abs() < 1e-12 && a == 0.);
        let (s, a) = iso_insert_body("CCMT09T304").unwrap();
        assert!((s - 4.13).abs() < 1e-12 && a == 7.);
        let (s, a) = iso_insert_body("DCGT11T302").unwrap();
        assert!((s - 4.13).abs() < 1e-12 && a == 7.);
        let (s, a) = iso_insert_body("TPGN160308").unwrap();
        assert!((s - 4.13).abs() < 1e-12 && a == 11.);
        assert!(
            iso_insert_body("COMT120408")
                .unwrap_err()
                .contains("special clearance")
        );
        assert!(
            iso_insert_body("CNZG120408")
                .unwrap_err()
                .contains("tolerance class")
        );
    }

    #[test]
    fn erosion_moves_every_side_inward() {
        let square = [[0., 0.], [10., 0.], [10., 10.], [0., 10.]];
        let e = erode_convex(&square, 1.).unwrap();
        for p in &e {
            assert!(p.iter().all(|v| (1. - 1e-12..=9. + 1e-12).contains(v)));
        }
        assert!(erode_convex(&square, 6.).is_none());
    }

    fn tool(insert: Insert, rn: f64, side: TurnSide, body: ToolBody) -> TurningTool {
        let holder = Holder::rectangle(&insert, rn, side, 10., 60., 1.).unwrap();
        TurningTool {
            id: "t".into(),
            insert,
            nose_radius_mm: rn,
            side,
            hand: Hand::Right,
            holder,
            body: Some(body),
        }
    }

    fn body(s: f64, alpha: f64, rake: f64, incl: f64, edge: f64) -> ToolBody {
        ToolBody {
            insert: Some(InsertBody {
                insert_thickness_mm: s,
                insert_clearance_deg: alpha,
                rake_deg: rake,
                inclination_deg: incl,
                edge_deg: edge,
                curved_for_diameter_mm: None,
                blade_clearance: None,
            }),
            holder_y_mm: [-10., 0.],
            bar_axis_y_mm: 0.,
            solid_bar: None,
            model: None,
        }
    }

    /// Independent oracle: dense points of the insert body, built directly
    /// from the face polygon (sampled inside), the taper and the rotation,
    /// not from the pieces' vertices.
    fn oracle_points(t: &TurningTool) -> Vec<[f64; 3]> {
        let b = t.body.as_ref().unwrap().insert.as_ref().unwrap();
        let shape = t.shape().unwrap();
        let poly = &shape.insert;
        let (lo, hi) = poly.iter().fold(
            ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]),
            |(lo, hi), p| {
                (
                    [lo[0].min(p[0]), lo[1].min(p[1])],
                    [hi[0].max(p[0]), hi[1].max(p[1])],
                )
            },
        );
        let inside = |p: [f64; 2], d: f64| {
            // Inside the polygon by at least d (convex polygons only here).
            let n = poly.len();
            let area2: f64 = (0..n)
                .map(|i| poly[i][0] * poly[(i + 1) % n][1] - poly[(i + 1) % n][0] * poly[i][1])
                .sum();
            (0..n).all(|i| {
                let (a, c) = (poly[i], poly[(i + 1) % n]);
                let (ex, ez) = (c[0] - a[0], c[1] - a[1]);
                let len = ex.hypot(ez);
                len == 0. || {
                    let cross = ex * (p[1] - a[1]) - ez * (p[0] - a[0]);
                    (if area2 > 0. { cross } else { -cross }) / len >= d
                }
            })
        };
        let e = [b.edge_deg.to_radians().cos(), b.edge_deg.to_radians().sin()];
        let c = poly
            .iter()
            .fold([0., 0.], |c, p| [c[0] + p[0], c[1] + p[1]]);
        let mut q = [-e[1], e[0]];
        if q[0] * c[0] + q[1] * c[1] < 0. {
            q = [-q[0], -q[1]];
        }
        let l = b.inclination_deg.to_radians();
        let g = (b.rake_deg.to_radians().tan() * l.cos()).atan();
        let mut out = vec![];
        for i in 0..=60 {
            for j in 0..=60 {
                let p = [
                    lo[0] + (hi[0] - lo[0]) * i as f64 / 60.,
                    lo[1] + (hi[1] - lo[1]) * j as f64 / 60.,
                ];
                for k in 0..=8 {
                    let y = -b.insert_thickness_mm * k as f64 / 8.;
                    if !inside(p, -y * b.insert_clearance_deg.to_radians().tan()) {
                        continue;
                    }
                    let (a0, b0) = (p[0] * e[0] + p[1] * e[1], p[0] * q[0] + p[1] * q[1]);
                    // Rotate: rake (-g) about e, then inclination l about q.
                    let (b1, c1) = (
                        b0 * (-g).cos() - y * (-g).sin(),
                        b0 * (-g).sin() + y * (-g).cos(),
                    );
                    let (a2, c2) = (a0 * l.cos() - c1 * l.sin(), a0 * l.sin() + c1 * l.cos());
                    out.push([a2 * e[0] + b1 * q[0], a2 * e[1] + b1 * q[1], c2]);
                }
            }
        }
        out
    }

    #[test]
    fn insert_pieces_contain_the_body_and_reach_its_corners() {
        let (insert, rn) = Insert::iso_code("CCMT09T304", 95.).unwrap();
        for (rake, incl) in [(0., 0.), (-6., -6.), (6., 3.), (0., -8.)] {
            let t = tool(
                insert.clone(),
                rn,
                TurnSide::Outer,
                body(3.97, 7., rake, incl, 5.),
            );
            let (pieces, _) = body_pieces(&t).unwrap();
            let points = oracle_points(&t);
            assert!(points.len() > 100);
            // Every oracle point lies in the convex hull of the piece
            // vertices: checked through the radial bound at any offset (the
            // property the checks rely on) and the projection extents.
            for offset in [0., 8., 11.] {
                let hull_max = pieces[0]
                    .iter()
                    .map(|v| (v[0] + offset).hypot(v[2]))
                    .fold(f64::NEG_INFINITY, f64::max);
                let oracle_max = points
                    .iter()
                    .map(|v| (v[0] + offset).hypot(v[2]))
                    .fold(f64::NEG_INFINITY, f64::max);
                assert!(oracle_max <= hull_max + 1e-9, "{rake} {incl} {offset}");
                // Tight: the oracle (sampled) gets within the sampling step.
                assert!(
                    hull_max - oracle_max < 0.25,
                    "{rake} {incl} {offset}: {hull_max} {oracle_max}"
                );
            }
        }
    }

    /// A grooving blade 3 mm wide (0.2 mm corners), 8 mm deep, `s` thick,
    /// with its clearance per flank.
    fn blade(s: f64, front: f64, sides: [f64; 2]) -> TurningTool {
        let mut b = body(s, front.min(sides[0]).min(sides[1]), 0., 0., 90.);
        b.insert.as_mut().unwrap().blade_clearance = Some(BladeClearance {
            front_deg: front,
            side_deg: sides,
        });
        tool(
            Insert::Groove {
                width_mm: 3.,
                depth_mm: 8.,
            },
            0.2,
            TurnSide::Outer,
            b,
        )
    }

    /// Unit directions over the sphere.
    fn directions() -> Vec<[f64; 3]> {
        (0..400)
            .map(|k| {
                let z = 1. - 2. * (k as f64 + 0.5) / 400.;
                let a = k as f64 * 2.399963229728653;
                let r = (1. - z * z).sqrt();
                [r * a.cos(), r * a.sin(), z]
            })
            .collect()
    }
    fn support(points: &[[f64; 3]], n: [f64; 3]) -> f64 {
        points
            .iter()
            .map(|p| p[0] * n[0] + p[1] * n[1] + p[2] * n[2])
            .fold(f64::NEG_INFINITY, f64::max)
    }

    #[test]
    fn a_blades_front_and_sides_recede_at_their_own_clearance() {
        // The blade in its frame: front at x = -0.2, sides at z = -0.2 and
        // z = 2.8, top at x = 8; corner centers (0, 0) and (0, 2.6).
        let (s, front, sides) = (3., 10., [3., 5.]);
        let t = blade(s, front, sides);
        let built = super::body(&t).unwrap();
        let piece = &built.insert[0];
        let tan = |deg: f64| deg.to_radians().tan();
        // Independent oracle: points of the declared body from the flank
        // planes and the corner circles themselves (the corners and the
        // top at the least clearance), not from the polygon.
        let least = 3.;
        let inside = |x: f64, z: f64, depth: f64| {
            let corner = |cz: f64| x.hypot(z - cz) <= 0.2 - depth * tan(least);
            x >= -0.2 + depth * tan(front)
                && x <= 8. - depth * tan(least)
                && z >= -0.2 + depth * tan(sides[0])
                && z <= 2.8 - depth * tan(sides[1])
                && (x >= 0. || (0. ..=2.6).contains(&z) || corner(0.) || corner(2.6))
        };
        let mut oracle = vec![];
        for i in 0..=164 {
            for j in 0..=60 {
                for k in 0..=30 {
                    let (x, z, depth) = (
                        -0.2 + 0.05 * i as f64,
                        -0.2 + 0.05 * j as f64,
                        0.1 * k as f64,
                    );
                    if inside(x, z, depth) {
                        oracle.push([x, z, -depth]);
                    }
                }
            }
        }
        assert!(oracle.len() > 10_000);
        for n in directions() {
            let (hull, truth) = (support(piece, n), support(&oracle, n));
            // Never thinner than the declared body; tight to the sampling.
            assert!(truth <= hull + 1e-9, "{n:?}: {truth} {hull}");
            assert!(hull - truth < 0.12, "{n:?}: {truth} {hull}");
        }
        // The bottom's flanks, exactly where their angles put them.
        let bottom: Vec<_> = piece.iter().filter(|v| v[2] == -s).collect();
        let extent = |axis: usize, sign: f64| {
            bottom
                .iter()
                .map(|v| sign * v[axis])
                .fold(f64::NEG_INFINITY, f64::max)
                * sign
        };
        assert!((extent(0, -1.) - (-0.2 + s * tan(10.))).abs() < 1e-12);
        assert!((extent(1, -1.) - (-0.2 + s * tan(3.))).abs() < 1e-12);
        assert!((extent(1, 1.) - (2.8 - s * tan(5.))).abs() < 1e-12);
        // Each flank is named with its own angle.
        let named: Vec<_> = built
            .flanks
            .iter()
            .map(|f| (f.flank, f.clearance_deg))
            .collect();
        assert_eq!(
            named,
            [
                (BladeFlank::Front, 10.),
                (BladeFlank::Side(0), 3.),
                (BladeFlank::Side(1), 5.),
                (BladeFlank::Corner, 3.)
            ]
        );
        // Between declaring the least angle on every side and the front's
        // on every side: never thicker than the first, never thinner than
        // the second.
        let uniform = |deg: f64| {
            let mut u = t.clone();
            let b = u.body.as_mut().unwrap().insert.as_mut().unwrap();
            b.blade_clearance = None;
            b.insert_clearance_deg = deg;
            body_pieces(&u).unwrap().0.remove(0)
        };
        let (thick, thin) = (uniform(3.), uniform(10.));
        let mut differs = false;
        for n in directions() {
            let h = support(piece, n);
            assert!(h <= support(&thick, n) + 1e-9 && support(&thin, n) <= h + 1e-9);
            differs |= h < support(&thick, n) - 0.1;
        }
        assert!(differs);
    }

    #[test]
    fn a_tapered_body_has_the_vertices_where_a_side_runs_out() {
        // An octagon whose short sides recede slower than their neighbours:
        // they run out below the face, at vertices neither the face nor the
        // bottom has. Oracle: every vertex of the solid by brute force, any
        // three of its planes (sides, face, bottom) that meet in a point
        // every plane keeps.
        let polygon = [
            [0., 1.],
            [0., 4.],
            [1., 5.],
            [6., 5.],
            [7., 4.5],
            [7., 0.5],
            [6.5, 0.],
            [1., 0.],
        ];
        let rate = [0.3, 0.05, 0.2, 0.02, 0.25, 0.02, 0.1, 0.0];
        let s = 4.;
        let got = tapered_vertices(&polygon, &rate, s).unwrap();
        // Planes a·(x, z, depth) >= b.
        let mut planes: Vec<([f64; 3], f64)> = half_planes(&polygon)
            .into_iter()
            .zip(rate)
            .map(|(h, k)| {
                let (n, c) = h.unwrap();
                ([n[0], n[1], -k], c)
            })
            .collect();
        planes.push(([0., 0., 1.], 0.));
        planes.push(([0., 0., -1.], -s));
        let det = |a: [f64; 3], b: [f64; 3], c: [f64; 3]| {
            a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                + a[2] * (b[0] * c[1] - b[1] * c[0])
        };
        let mut truth = vec![];
        for i in 0..planes.len() {
            for j in i + 1..planes.len() {
                for k in j + 1..planes.len() {
                    let (a, b, c) = (planes[i].0, planes[j].0, planes[k].0);
                    let d = det(a, b, c);
                    if d.abs() < 1e-9 {
                        continue;
                    }
                    let r = [planes[i].1, planes[j].1, planes[k].1];
                    let col = |m: usize| {
                        let sub = |v: [f64; 3], w: f64| {
                            let mut v = v;
                            v[m] = w;
                            v
                        };
                        det(sub(a, r[0]), sub(b, r[1]), sub(c, r[2])) / d
                    };
                    let p = [col(0), col(1), col(2)];
                    if planes
                        .iter()
                        .all(|(n, c)| n[0] * p[0] + n[1] * p[1] + n[2] * p[2] >= c - 1e-9)
                    {
                        truth.push([p[0], p[1], -p[2]]);
                    }
                }
            }
        }
        let near = |a: &[f64; 3], b: &[f64; 3]| (0..3).all(|k| (a[k] - b[k]).abs() < 1e-7);
        for v in &truth {
            assert!(got.iter().any(|g| near(g, v)), "missing {v:?}");
        }
        for g in &got {
            assert!(truth.iter().any(|v| near(g, v)), "not a vertex: {g:?}");
        }
        // Some lie strictly between the face and the bottom.
        assert!(truth.iter().any(|v| v[2] < -0.1 && v[2] > -s + 0.1));
    }

    /// The body's vertices followed down from the face against the same
    /// body from every pair of side planes (another construction): the same
    /// solid, for whole ISO inserts and blades with a rate per flank, and
    /// without a vertex repeated for every side that runs out at it.
    #[test]
    fn the_tapered_body_agrees_with_its_construction_from_plane_pairs() {
        let tan = |deg: f64| f64::to_radians(deg).tan();
        let outline = |code: &str| {
            let (insert, rn) = Insert::iso_code(code, 95.).unwrap();
            tool(insert, rn, TurnSide::Outer, body(3., 7., 0., 0., 5.))
                .shape()
                .unwrap()
                .insert
        };
        let blade_outline = blade(3., 10., [3., 5.]).shape().unwrap().insert;
        // A blade's rates by flank, as `body` assigns them.
        let blade_rates = |front: f64, sides: [f64; 2]| -> Vec<f64> {
            let least = front.min(sides[0]).min(sides[1]);
            half_planes(&blade_outline)
                .iter()
                .map(|h| {
                    let n = h.unwrap().0;
                    tan(if n[0] > 1. - 1e-12 {
                        front
                    } else if n[1] > 1. - 1e-12 {
                        sides[0]
                    } else if -n[1] > 1. - 1e-12 {
                        sides[1]
                    } else {
                        least
                    })
                })
                .collect()
        };
        // A case: its name, outline, rate per side and thickness.
        type Case = (String, Vec<[f64; 2]>, Vec<f64>, f64);
        let mut cases: Vec<Case> = vec![];
        for code in [
            "CCMT09T304",
            "VCMT160404",
            "TPGN160304",
            "RCMT10T3M0",
            "CNMG120408",
        ] {
            let p = outline(code);
            for (deg, s) in [(7., 3.97), (11., 4.76), (0., 4.76), (20., 2.)] {
                cases.push((
                    format!("{code} {deg} {s}"),
                    p.clone(),
                    vec![tan(deg); p.len()],
                    s,
                ));
            }
        }
        for (front, sides, s) in [
            (10., [3., 5.], 3.),
            (0., [0., 15.], 3.),
            (20., [15., 12.], 3.),
            (3., [10., 10.], 4.),
            (7., [7., 7.], 2.),
        ] {
            cases.push((
                format!("blade {front} {sides:?} {s}"),
                blade_outline.clone(),
                blade_rates(front, sides),
                s,
            ));
        }
        let directions: Vec<[f64; 3]> = (0..3000)
            .map(|k| {
                let z = 1. - 2. * (k as f64 + 0.5) / 3000.;
                let a = k as f64 * 2.399963229728653;
                let r = (1. - z * z).sqrt();
                [r * a.cos(), r * a.sin(), z]
            })
            .collect();
        let mut inner = 0;
        for (name, polygon, rate, s) in &cases {
            let got = tapered_vertices(polygon, rate, *s).unwrap();
            let oracle = tapered_vertices_by_lines(polygon, rate, *s);
            for n in &directions {
                let (g, o) = (support(&got, *n), support(&oracle, *n));
                assert!((g - o).abs() < 1e-7, "{name}: {n:?}: {g} {o}");
            }
            // At most two vertices for a side that runs out.
            assert!(got.len() <= 4 * polygon.len(), "{name}: {}", got.len());
            inner += got.iter().filter(|v| v[2] < 0. && v[2] > -s).count();
        }
        assert!(inner > 20);
    }

    #[test]
    fn blade_clearance_is_a_blades_and_agrees_with_the_least_clearance() {
        // Not on an ISO insert (one normal clearance, ISO 1832).
        let (insert, rn) = Insert::iso_code("CCMT09T304", 95.).unwrap();
        let mut b = body(3.97, 7., 0., 0., 5.);
        b.insert.as_mut().unwrap().blade_clearance = Some(BladeClearance {
            front_deg: 7.,
            side_deg: [7., 7.],
        });
        let t = tool(insert, rn, TurnSide::Outer, b);
        for e in [t.validate().unwrap_err(), super::body(&t).unwrap_err()] {
            assert!(e.contains("give insert_clearance_deg only"), "{e}");
        }
        // The uniform angle must be the least of the flanks'.
        let mut t = blade(3., 10., [3., 3.]);
        t.body
            .as_mut()
            .unwrap()
            .insert
            .as_mut()
            .unwrap()
            .insert_clearance_deg = 10.;
        let e = t.validate().unwrap_err();
        assert!(e.contains("contradicts") && e.contains("give 3"), "{e}");
        let mut t = blade(3., 10., [3., 3.]);
        t.body
            .as_mut()
            .unwrap()
            .insert
            .as_mut()
            .unwrap()
            .blade_clearance
            .as_mut()
            .unwrap()
            .side_deg[1] = 75.;
        assert!(t.validate().unwrap_err().contains("0..60 degrees"));
        // A tool without it serializes as before, and reads back.
        let plain = body(3., 7., 0., 0., 90.);
        let json = serde_json::to_string(&plain).unwrap();
        assert!(!json.contains("blade_clearance"), "{json}");
        assert_eq!(serde_json::from_str::<ToolBody>(&json).unwrap(), plain);
        let declared = blade(3., 10., [3., 5.]).body.unwrap();
        let json = serde_json::to_string(&declared).unwrap();
        assert!(json.contains(r#""blade_clearance":{"front_deg":10.0,"side_deg":[3.0,5.0]}"#));
        assert_eq!(serde_json::from_str::<ToolBody>(&json).unwrap(), declared);
    }

    /// Where the taper exceeds the nose radius the nose arc runs out below
    /// the face: at depth rn / tan(clearance) every side has receded by rn,
    /// so the nose center itself is a point of the body (its flank's cone
    /// ends there). The hull of the face and the bottom alone passes inside
    /// it.
    #[test]
    fn a_taper_past_the_nose_radius_keeps_the_point_where_the_nose_runs_out() {
        for (code, clearance) in [("CCMT09T304", 7.), ("VCMT160404", 7.), ("TPGN160304", 11.)] {
            let (insert, rn) = Insert::iso_code(code, 95.).unwrap();
            let s = 3.97;
            let depth = rn / f64::to_radians(clearance).tan();
            assert!(depth < s);
            let t = tool(insert, rn, TurnSide::Outer, body(s, clearance, 0., 0., 5.));
            // In the declared body, by the polygon's sides themselves: no
            // side nearer than the taper at that depth.
            let poly = t.shape().unwrap().insert;
            let n = poly.len();
            let nearest = (0..n)
                .filter_map(|i| {
                    let (a, b) = (poly[i], poly[(i + 1) % n]);
                    let (ex, ez) = (b[0] - a[0], b[1] - a[1]);
                    let len = ex.hypot(ez);
                    (len > 0.).then(|| ((ex * -a[1] - ez * -a[0]) / len).abs())
                })
                .fold(f64::INFINITY, f64::min);
            assert!((nearest - rn).abs() < 1e-9, "{code}: {nearest}");
            let point = [0., 0., -depth];
            let (pieces, _) = body_pieces(&t).unwrap();
            let short = (0..20_000)
                .map(|k| {
                    let z = 1. - 2. * (k as f64 + 0.5) / 20_000.;
                    let a = k as f64 * 2.399963229728653;
                    let r = (1. - z * z).sqrt();
                    let dir = [r * a.cos(), r * a.sin(), z];
                    support(&[point], dir) - support(&pieces[0], dir)
                })
                .fold(f64::NEG_INFINITY, f64::max);
            assert!(short <= 1e-9, "{code}: the body misses it by {short} mm");
        }
    }

    #[test]
    fn a_round_bar_is_contained_off_the_plane() {
        use crate::turning::tool::Bar;
        let (insert, rn) = Insert::iso_code("CCMT060204", 95.).unwrap();
        let mut t = tool(insert, rn, TurnSide::Inner, body(2.38, 7., 0., 0., 5.));
        t.holder.bar = Some(Bar {
            diameter_mm: 10.,
            axis_x_mm: 6.,
            z_mm: [0., 60.],
        });
        for axis_y in [0., 3., -2.5] {
            t.body.as_mut().unwrap().bar_axis_y_mm = axis_y;
            let (_, holder) = body_pieces(&t).unwrap();
            let bar = holder.last().unwrap();
            let nose = t.shape().unwrap().bar.unwrap()[0][0] + 5. - 6.;
            for offset in [0., 4., 9.] {
                let hull = bar
                    .iter()
                    .map(|v| (v[0] + offset).hypot(v[2]))
                    .fold(f64::NEG_INFINITY, f64::max);
                // The true cylinder: radius 5 about (axis x, axis y).
                let truth = (0..3600)
                    .map(|k| {
                        let a = std::f64::consts::TAU * k as f64 / 3600.;
                        (6. + nose + 5. * a.cos() + offset).hypot(axis_y + 5. * a.sin())
                    })
                    .fold(f64::NEG_INFINITY, f64::max);
                assert!(
                    truth <= hull + 1e-9 && hull - truth < 0.05,
                    "{axis_y} {offset}: {truth} {hull}"
                );
            }
        }
    }

    #[test]
    fn the_issues_boring_insert_reaches_past_its_outline() {
        // A 4.76 mm thick negative insert, its back corner 10.6 mm out on
        // the face: 4.76 mm below, √(10.6² + 4.76²) = 11.62 mm from the axis.
        let face: f64 = 10.6;
        let r = face.hypot(4.76);
        assert!((r - 11.6196).abs() < 1e-3);
        let (insert, rn) = Insert::iso_code("CNMG120408", 95.).unwrap();
        let t = tool(insert, rn, TurnSide::Inner, body(4.76, 0., 0., 0., 5.));
        let (pieces, _) = body_pieces(&t).unwrap();
        let reach_face = pieces[0]
            .iter()
            .filter(|v| v[2] == 0.)
            .map(|v| v[0])
            .fold(f64::NEG_INFINITY, f64::max);
        let reach_body = pieces[0]
            .iter()
            .map(|v| v[0].hypot(v[2]))
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(reach_body > reach_face + 0.5);
    }

    // Solid (one-piece) boring bars (#114).

    /// The largest radius of `points` (x, z, y) with the cutting edge (at
    /// x -0.1) on the wall of a bore of radius `bore`.
    fn reach_in_bore(points: &[[f64; 3]], bore: f64) -> f64 {
        points
            .iter()
            .map(|p| (bore - (p[0] + 0.1)).hypot(p[2]))
            .fold(f64::NEG_INFINITY, f64::max)
    }

    #[test]
    fn a_circumscribed_polygon_contains_its_circle() {
        for n in [4, 32, 180] {
            let polygon = circumscribed_polygon([1., -2.], 2.5, n);
            // Every side is tangent to the circle: at the circle's radius
            // from the center, never nearer.
            for k in 0..n {
                let (a, b) = (polygon[k], polygon[(k + 1) % n]);
                let (ex, ey) = (b[0] - a[0], b[1] - a[1]);
                let d = (ex * (a[1] + 2.) - ey * (a[0] - 1.)).abs() / ex.hypot(ey);
                assert!((d - 2.5).abs() < 1e-12, "{n} {k}: {d}");
            }
            // The circle's extremes along both axes are on it.
            let x = polygon.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min);
            let y = polygon.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min);
            assert!((x + 1.5).abs() < 1e-12 && (y + 4.5).abs() < 1e-12);
        }
    }

    #[test]
    fn a_solid_head_contains_the_clipped_cylinder_and_no_more_than_its_polygons() {
        for (side, axis_y) in [(20., 0.), (7., 0.), (20., -0.4), (12., 0.3)] {
            let mut t = test_solid_bar(side, 40.);
            let head = t.body.as_mut().unwrap().solid_bar.as_mut().unwrap();
            head.axis_y_mm = axis_y;
            let head = head.clone();
            let pieces = body_pieces(&t).unwrap().0;
            assert_eq!(pieces.len(), 1);
            let vertices = &pieces[0];
            let samples = solid_head_samples(&t, 0.1);
            assert!(samples.len() > 20_000 && vertices.len() > 50);
            // The polytope contains the true head: in every direction it
            // reaches at least as far, and no more than the polygons stand
            // outside (0.0004 mm) plus the samples' spacing.
            let mut slack = 0f64;
            for d in directions() {
                let (hull, truth) = (support(vertices, d), support(&samples, d));
                assert!(
                    truth <= hull + 1e-9,
                    "{side} {axis_y} {d:?}: {truth} {hull}"
                );
                slack = slack.max(hull - truth);
            }
            assert!(slack < 0.2, "{side} {axis_y}: {slack}");
            // Every vertex is in the circumscribed prism, below the face
            // and behind every straight side's ground face: the polytope is
            // no more than the model's polygons.
            let shape = t.shape().unwrap();
            let outline = crate::turning::tool::head_outline(&t.insert).unwrap();
            let planes = half_planes(&shape.insert);
            let r = 2.5 / 1f64.to_radians().cos();
            for v in vertices {
                assert!(v[2] <= 1e-9);
                assert!((v[0] - 2.34).hypot(v[2] - axis_y) <= r + 1e-9, "{v:?}");
                for (i, plane) in planes.iter().enumerate() {
                    let (n, c) = plane.unwrap();
                    let rate = head.clearance_deg[outline.owners[i]].to_radians().tan();
                    assert!(
                        n[0] * v[0] + n[1] * v[1] - c + rate * v[2] >= -1e-8,
                        "{v:?}"
                    );
                }
            }
            // The face itself is the outline: nothing of it is lost.
            let face: Vec<[f64; 3]> = vertices
                .iter()
                .filter(|v| v[2].abs() < 1e-9)
                .copied()
                .collect();
            for k in 0..720 {
                let a = std::f64::consts::TAU * k as f64 / 720.;
                let d = [a.cos(), a.sin(), 0.];
                let outline = shape
                    .insert
                    .iter()
                    .map(|p| p[0] * d[0] + p[1] * d[1])
                    .fold(f64::NEG_INFINITY, f64::max);
                assert!((support(&face, d) - outline).abs() < 1e-8, "{k}");
            }
            // In a bore, with the edge on its wall and anywhere inside it:
            // the true head stays within the vertices' radial bound, at
            // every z (the upper hull of the vertices' radial image).
            for bore in [2.75, 3.25, 6.] {
                for inside in [0., 0.3] {
                    let image = |p: &[f64; 3]| [p[1], (bore - inside - (p[0] + 0.1)).hypot(p[2])];
                    let mut hull: Vec<[f64; 2]> = vertices.iter().map(image).collect();
                    hull.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
                    let mut upper: Vec<[f64; 2]> = vec![];
                    for p in hull {
                        while upper.len() >= 2 {
                            let (a, b) = (upper[upper.len() - 2], upper[upper.len() - 1]);
                            if (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) >= 0. {
                                upper.pop();
                            } else {
                                break;
                            }
                        }
                        upper.push(p);
                    }
                    for s in &samples {
                        let [z, radius] = image(s);
                        let bound = upper
                            .windows(2)
                            .filter(|w| w[0][0] <= z && z <= w[1][0])
                            .map(|w| {
                                if w[1][0] == w[0][0] {
                                    w[0][1].max(w[1][1])
                                } else {
                                    w[0][1]
                                        + (w[1][1] - w[0][1]) * (z - w[0][0]) / (w[1][0] - w[0][0])
                                }
                            })
                            .fold(f64::NEG_INFINITY, f64::max);
                        assert!(radius <= bound + 1e-9, "{bore} {s:?}: {radius} {bound}");
                    }
                }
            }
        }
    }

    #[test]
    fn a_solid_head_enters_the_bore_its_outline_prism_cannot() {
        // The Ø5 bar in its catalogue minimum bore of Ø5.5, the edge on the
        // wall: the ground head stays inside (to within the polygons'
        // 0.0004 mm), and so does the true head.
        let t = test_solid_bar(20., 5.5);
        let head = body_pieces(&t).unwrap().0.remove(0);
        assert!(reach_in_bore(&head, 2.75) <= 2.75 + 4e-4);
        assert!(reach_in_bore(&solid_head_samples(&t, 0.05), 2.75) <= 2.75 + 1e-12);
        // With 7 degrees below its side edge the head itself fouls a Ø6.5
        // bore: the true head, and so the checked one.
        let fouling = test_solid_bar(7., 7.6);
        let truth = reach_in_bore(&solid_head_samples(&fouling, 0.05), 3.25);
        assert!(truth > 3.25 + 0.015, "{truth}");
        assert!(reach_in_bore(&body_pieces(&fouling).unwrap().0[0], 3.25) >= truth);
        // The same head as #103 can describe it: a prism of its outline
        // as deep as the bar reaches below the face (2.5 mm), clearance 0.
        let mut prism = t.clone();
        prism.body = Some(body(2.5, 0., 0., 0., 90.));
        let pieces = body_pieces(&prism).unwrap().0;
        let reach = pieces
            .iter()
            .map(|p| reach_in_bore(p, 2.75))
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(reach > 2.75 + 0.9, "{reach}");
        // The prism contains the head (it is sound, only unusable here).
        let points: Vec<[f64; 3]> = pieces.into_iter().flatten().collect();
        for d in directions() {
            assert!(support(&head, d) <= support(&points, d) + 1e-3);
        }
    }

    #[test]
    fn a_solid_heads_declaration_is_cross_checked() {
        let err = |t: &TurningTool| t.validate().unwrap_err();
        let with = |change: &dyn Fn(&mut SolidBarHead)| {
            let mut t = test_solid_bar(20., 5.5);
            change(t.body.as_mut().unwrap().solid_bar.as_mut().unwrap());
            t
        };
        test_solid_bar(20., 5.5).validate().unwrap();
        // The cutting edge off the bar: WF larger than the bar's radius.
        let e = err(&with(&|h| h.axis_x_mm = 2.6));
        assert!(
            e.contains("outside the bar's section") && e.contains("WF"),
            "{e}"
        );
        // The cutting face must cut the bar.
        let e = err(&with(&|h| h.axis_y_mm = 2.5));
        assert!(e.contains("axis_y_mm"), "{e}");
        // One clearance angle per outline segment.
        let e = err(&with(&|h| h.clearance_deg = vec![20., 8.]));
        assert!(e.contains("2 angles for an outline of 5 segments"), "{e}");
        let e = err(&with(&|h| h.min_bore_diameter_mm = 0.));
        assert!(e.contains("min_bore_diameter_mm"), "{e}");
        // The nose with more clearance than a side it joins: its tangent
        // planes would cut away what the side's flank leaves.
        let e = err(&with(&|h| h.clearance_deg[1] = 9.));
        assert!(
            e.contains("rounded corner (outline segment 1: 9 degrees)"),
            "{e}"
        );
        with(&|h| h.clearance_deg[1] = 0.).validate().unwrap();
        // A minimum bore the head cannot enter: 7 degrees below the side
        // edge of this bar clears a bore's curvature down to Ø7.47 only
        // (the side face leaves the cylinder about 0.91 mm below the face
        // and 0.11 mm behind the edge: (a² + y²) / a).
        let e = err(&test_solid_bar(7., 5.5));
        assert!(
            e.contains("minimum bore of 5.5 mm contradicts its head")
                && e.contains("the least bore it enters is 7.47"),
            "{e}"
        );
        test_solid_bar(7., 7.6).validate().unwrap();
        // 20 degrees: the least bore is under the catalogue's Ø5.5.
        let e = err(&test_solid_bar(20., 5.));
        assert!(e.contains("the least bore it enters is 5.3"), "{e}");
        // A head that is the bar itself (no relief, the edge on the bar's
        // surface at its centre height) enters a bore of the bar's own
        // diameter: "Min. Bore Dia. same as Shank Dia.".
        let mut round = with(&|h| {
            h.axis_x_mm = 2.4;
            h.clearance_deg = vec![0.; 5];
            h.min_bore_diameter_mm = 5.;
        });
        round.validate().unwrap();
        // No relief and the edge inside the bar's surface: no bore.
        round
            .body
            .as_mut()
            .unwrap()
            .solid_bar
            .as_mut()
            .unwrap()
            .axis_x_mm = 2.34;
        assert!(err(&round).contains("contradicts its head"));
        // A boring head only, with no insert body beside it.
        let mut outside = test_solid_bar(20., 5.5);
        outside.side = TurnSide::Outer;
        let e = err(&outside);
        assert!(e.contains("boring head") && e.contains("threading"), "{e}");
        let mut blade = test_solid_bar(20., 5.5);
        blade.insert = Insert::Groove {
            width_mm: 1.,
            depth_mm: 2.,
        };
        assert!(err(&blade).contains("grooving"));
        let mut both = test_solid_bar(20., 5.5);
        both.body.as_mut().unwrap().insert = body(2.5, 0., 0., 0., 90.).insert;
        assert!(err(&both).contains("give no insert body"));
        // Its declaration round-trips, and a tool without one is unchanged.
        let json = serde_json::to_string(&test_solid_bar(20., 5.5)).unwrap();
        assert!(json.contains("\"solid_bar\""));
        assert_eq!(
            serde_json::from_str::<TurningTool>(&json).unwrap(),
            test_solid_bar(20., 5.5)
        );
        let (insert, rn) = Insert::iso_code("CCMT060204", 95.).unwrap();
        let plain = tool(insert, rn, TurnSide::Inner, body(2.38, 7., 0., 0., 5.));
        assert!(!serde_json::to_string(&plain).unwrap().contains("solid_bar"));
    }
}
