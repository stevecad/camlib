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
}
