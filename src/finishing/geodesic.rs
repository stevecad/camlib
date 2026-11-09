//! Geodesic finishing passes (#12), contact mode: passes whose contact
//! paths are a constant stepover apart measured along the part's surface.
//!
//! - The drive surface. The target's triangles are welded into a connected
//!   mesh; the drive surface is its upward-facing triangles (normal Z > 0:
//!   the faces a three-axis ball can touch). Its boundary is explicit: open
//!   edges and edges against walls or downward faces. Passes end there; they
//!   are not rolled over edges. The CAD triangles are used as they are:
//!   the field and the passes are exact on faces of any size or shape.
//! - The field: the exact geodesic distance on that polyhedral surface from
//!   the seed (a point, the boundary, or a curve), by window propagation
//!   (`exact`: Mitchell, Mount and Papadimitriou; Surazhsky et al. 2005;
//!   Bommes and Kobbelt 2007 for curve sources). Creases are crossed on the
//!   surface itself; paths bend only at saddle and boundary vertices. No
//!   cutter contact map and no grid is involved.
//! - The passes: the iso-curves at (offset + k) x stepover, placed exactly:
//!   crossings on edges and inside faces are roots of the field's circles
//!   and lines, and each chord between two is split until it lies within
//!   the path error of the curve. Inside a face the field's maxima are where
//!   three of its circles and lines are equal (solved exactly), so levels
//!   closing inside a face are found too. Fill passes are added until every
//!   point is within half a stepover of a pass, certified with the same
//!   exact maxima of the distance from the passes. Each contact point becomes a cutter position
//!   (ball centre = contact + R x normal); the finishing plan lifts every
//!   position onto the surface with the certified ball contact and
//!   certifies every chord (collision-checked like every pattern).
//! - Verification, independent of the planned field: the achieved contact
//!   of each lifted position is computed afresh and located on the drive
//!   surface; the spacing between neighbouring passes is the exact geodesic
//!   distance from one pass's achieved contact path (its polyline projected
//!   onto the surface, a polygonal source) at the next pass's achieved
//!   contacts. Contacts that land off the drive surface (a wall, the base:
//!   the ball could not reach the intended point) are counted, never
//!   hidden. Nothing is rescaled to fit. The scallop given is a
//!   flat-surface estimate from the spacing (labelled as such); the
//!   material check, not this estimate, decides residual stock.
mod exact;
mod trace;

use super::patterns::GeodesicSeed;
use crate::{
    stock::CancelSignal,
    surface::{PreparedSurface, SurfaceMesh, SurfaceWork},
};
use exact::{Coverage, Distance, FaceQuery, Field, NONE, Piece, Topology, walk};
use serde::Serialize;
use std::{cmp::Ordering, collections::HashMap};

type Vec3 = [f64; 3];

#[derive(Clone, Debug, Serialize)]
pub struct GeodesicReport {
    pub seed: &'static str,
    /// "contact" (the stepover measured along the part's surface) or
    /// "tool_centre" (along the surface the ball's centre moves on).
    pub mode: &'static str,
    pub stepover_mm: f64,
    pub drive_triangles: usize,
    pub levels: usize,
    /// Passes added where the levels left a point farther than half a
    /// stepover from every pass (a peak between levels, a slowly rising
    /// ridge).
    pub cap_passes: usize,
    /// The certified bound on the distance from any point of the drive
    /// surface to the nearest planned pass: half the stepover plus the
    /// path error (planning refuses a surface it cannot cover so).
    pub planned_coverage_mm: f64,
    /// The intrinsic spacing between neighbouring passes' achieved contact
    /// paths, along the drive surface: [min, mean, max] mm.
    pub spacing_mm: [f64; 3],
    pub samples: usize,
    /// Samples farther than the stepover from the previous pass.
    pub over_stepover_samples: usize,
    /// Samples whose achieved contact lies off the drive surface (on a wall
    /// or the base: the ball cannot reach the intended contact there).
    pub off_drive_samples: usize,
    /// Samples with no previous pass within two stepovers on the drive
    /// surface (the previous pass ends, or lies across a boundary).
    pub unmeasured_samples: usize,
    /// Pass vertices the ball cannot touch (the certified drop lifts it
    /// above the intended contact by more than the surface error: a concave
    /// crease narrower than the ball; or its contact leaves the drive
    /// surface): left uncut and counted.
    pub unreachable_samples: usize,
    /// Unreachable runs inside a pass, bridged: the cutter path runs
    /// straight (lifted by the certified drop, riding the crease) from the
    /// exact point where the ball stops reaching the pass to where it
    /// reaches it again, instead of following positions that loop back
    /// (the offset's swallowtail). Unreachable ends of open passes are cut
    /// off at that point.
    pub bridged_spans: usize,
    /// Pass vertices whose achieved contact is off its level by more than
    /// a tenth of the path error (the ball meets a neighbouring facet first,
    /// or rests on both sides of a concave edge).
    pub off_level_samples: usize,
    /// The largest distance (along the surface) between an achieved
    /// contact and its level.
    pub max_level_miss_mm: f64,
    /// Links between consecutive passes kept on the surface (the cutter
    /// stays down) and links that retract.
    pub stay_down_links: usize,
    pub retract_links: usize,
    /// A flat-surface estimate of the scallop at the largest measured
    /// spacing (convex areas leave more, concave less); the material check
    /// is authoritative.
    pub flat_scallop_estimate_mm: f64,
    /// Tool-centre mode: the spacing between neighbouring passes along the
    /// ball-centre surface they were planned on, [min, mean, max] mm
    /// (`spacing_mm` is then the contact spacing it produces).
    pub centre_spacing_mm: Option<[f64; 3]>,
}

fn sub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn dot(a: Vec3, b: Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn norm(a: Vec3) -> f64 {
    dot(a, a).sqrt()
}
fn lerp(a: Vec3, b: Vec3, t: f64) -> Vec3 {
    [0, 1, 2].map(|k| a[k] + t * (b[k] - a[k]))
}

/// A contact path: points on the drive surface with their unit normals.
type ContactPath = Vec<(Vec3, Vec3)>;
/// The passes (XY of the ball centre) and the report.
type Planned = (Vec<Vec<[f64; 2]>>, GeodesicReport);
/// A pass's cutter positions: (band, fill?, XY with how each landed).
type Fitted = (usize, bool, Vec<([f64; 2], Fit)>);
/// A cutter position with its contact.
type Placed = (Vec3, ([f64; 2], Fit));
/// A pass cut into pieces, and the number of bridged runs.
type Pieces = (Vec<Vec<([f64; 2], Fit)>>, usize);
/// Linked cutter paths, and the number of links kept down.
type Linked = (Vec<Vec<[f64; 2]>>, usize);
/// Segments filed by grid cell.
type SegmentGrid = HashMap<(i64, i64), Vec<([f64; 2], [f64; 2])>>;
/// Crossings along a side of a triangle: (parameter, node).
type Side = Vec<(f64, u32)>;

/// Subdivision depth for resolving a face's contour (the face edge over
/// 2^12): ambiguity below that size is below a micrometre on any part.
const MAX_DEPTH: usize = 12;

/// A ball touches the surface where it is within this of it (mm): the
/// verification's contact set.
const TOUCH: f64 = 1e-5;

/// The drive surface: welded, bisected, with per-vertex triangle lists and
/// its edge topology.
struct Drive {
    v: Vec<Vec3>,
    t: Vec<[u32; 3]>,
    /// Unit normal of each triangle.
    n: Vec<Vec3>,
    /// Each triangle's normal at its corners: the angle-weighted normals of
    /// the triangles around that vertex reached without crossing a crease
    /// (an estimate of the smooth surface's normal the mesh samples, to
    /// second order where a facet's normal is first order). An edge is a
    /// crease when it bends more than a smooth surface within the mesh's
    /// surface error e of both facets can: about 4e (1/w1 + 1/w2), w the
    /// facets' widths across it (the circle whose chords sag by e).
    corner_normals: Vec<[Vec3; 3]>,
    surface_error: f64,
    /// The weld tolerance: points this close are one point of the surface.
    weld: f64,
    vt: Vec<Vec<u32>>,
    /// Vertices on the drive surface's boundary.
    boundary: Vec<bool>,
    /// XY buckets of triangles, for locating points.
    cell: f64,
    origin: [f64; 2],
    buckets: HashMap<(i64, i64), Vec<u32>>,
    topo: Topology,
}

impl Drive {
    fn build(mesh: &SurfaceMesh, cancel: &impl CancelSignal) -> Result<Self, String> {
        // Weld: vertices within a binary64-scale tolerance are one, or
        // within a thousandth of the mesh's declared surface error (points
        // that close are one point of the surface it samples; a
        // tessellator's pole can leave a micrometre seam of slivers).
        let scale = mesh
            .triangles
            .iter()
            .flatten()
            .flatten()
            .fold(1f64, |m, v| m.max(v.abs()));
        let tol = (1e-9 * scale).max(1e-3 * mesh.surface_error_mm);
        let key = |p: Vec3| [0, 1, 2].map(|k| (p[k] / tol).round() as i64);
        let mut index: HashMap<[i64; 3], u32> = HashMap::new();
        let mut v: Vec<Vec3> = vec![];
        let mut weld = |p: Vec3, v: &mut Vec<Vec3>| -> u32 {
            let k = key(p);
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        if let Some(&i) = index.get(&[k[0] + dx, k[1] + dy, k[2] + dz])
                            && norm(sub(v[i as usize], p)) <= tol
                        {
                            return i;
                        }
                    }
                }
            }
            let i = v.len() as u32;
            v.push(p);
            index.insert(k, i);
            i
        };
        let mut t: Vec<[u32; 3]> = vec![];
        let mut degenerate: Vec<Vec3> = vec![];
        let mut seen = std::collections::HashSet::new();
        for (k, tri) in mesh.triangles.iter().enumerate() {
            if k % 65536 == 0 && cancel.is_cancelled() {
                return Err("geodesic planning cancelled".into());
            }
            let ids = tri.map(|p| weld(p, &mut v));
            if ids[0] == ids[1] || ids[1] == ids[2] || ids[0] == ids[2] {
                continue;
            }
            // A repeated triangle (a tessellator's fan at a pole) adds
            // nothing. (Slivers stay: removing one whose vertex lies on its
            // opposite edge would crack the mesh there.)
            let mut key = ids;
            key.sort();
            if !seen.insert(key) {
                continue;
            }
            // Upward-facing: reachable by a three-axis ball from above.
            let n = cross(sub(tri[1], tri[0]), sub(tri[2], tri[0]));
            let longest = (0..3)
                .map(|i| norm(sub(tri[(i + 1) % 3], tri[i])))
                .fold(0., f64::max);
            if norm(n) <= tol * longest {
                // Thinner than the weld tolerance: detail below the mesh's
                // own accuracy, with no orientation of its own (its
                // normal's sign is rounding). Geodesic distances across such
                // slivers are not reliable: refused below.
                degenerate.push(tri[0]);
            } else if n[2] > 0. {
                t.push(ids);
            }
        }
        if t.is_empty() {
            return Err("geodesic finishing: the target has no upward-facing surface".into());
        }
        if let Some(p) = degenerate.first() {
            return Err(format!(
                "geodesic finishing: the surface mesh has {} degenerate triangles (thinner than {:.1e} mm, a thousandth of its surface error), the first at {:?}: a tessellator's fan at a sphere's pole, say; repair or re-export the mesh",
                degenerate.len(),
                tol,
                p.map(|x| (x * 1e4).round() / 1e4)
            ));
        }

        // Boundary: drive edges with one drive triangle (open, or against a
        // wall or downward face).
        let mut drive_edges: HashMap<(u32, u32), u32> = HashMap::new();
        for tri in &t {
            for e in 0..3 {
                let (a, b) = (tri[e], tri[(e + 1) % 3]);
                *drive_edges.entry((a.min(b), a.max(b))).or_default() += 1;
            }
        }
        let mut boundary = vec![false; v.len()];
        for (&(a, b), &count) in &drive_edges {
            if count == 1 {
                boundary[a as usize] = true;
                boundary[b as usize] = true;
            }
        }
        let mut drive = Drive {
            v,
            t,
            n: vec![],
            corner_normals: vec![],
            surface_error: mesh.surface_error_mm,
            weld: tol,
            vt: vec![],
            boundary,
            cell: 1.,
            origin: [0., 0.],
            buckets: HashMap::new(),
            topo: Topology::default(),
        };
        drive.index();
        drive.topo = Topology::new(&drive.v, &drive.t, &drive.boundary)?;
        drive.corner_normals = drive.corner_normals();
        Ok(drive)
    }

    fn index(&mut self) {
        self.n = self
            .t
            .iter()
            .map(|tri| {
                let [a, b, c] = tri.map(|i| self.v[i as usize]);
                let nn = cross(sub(b, a), sub(c, a));
                let l = norm(nn);
                [nn[0] / l, nn[1] / l, nn[2] / l]
            })
            .collect();
        self.vt = vec![vec![]; self.v.len()];
        for (i, tri) in self.t.iter().enumerate() {
            for &k in tri {
                self.vt[k as usize].push(i as u32);
            }
        }
        let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
        for p in &self.v {
            for k in 0..2 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
        self.origin = lo;
        self.cell =
            ((hi[0] - lo[0]).max(hi[1] - lo[1]) / (self.t.len() as f64).sqrt().max(1.)).max(1e-6);
        for i in 0..self.t.len() {
            let ps = self.t[i].map(|k| self.v[k as usize]);
            let lo = [0, 1].map(|k| ps.iter().map(|p| p[k]).fold(f64::INFINITY, f64::min));
            let hi = [0, 1].map(|k| ps.iter().map(|p| p[k]).fold(f64::NEG_INFINITY, f64::max));
            let (a, b) = (self.key(lo[0], lo[1]), self.key(hi[0], hi[1]));
            for x in a.0..=b.0 {
                for y in a.1..=b.1 {
                    self.buckets.entry((x, y)).or_default().push(i as u32);
                }
            }
        }
    }

    fn key(&self, x: f64, y: f64) -> (i64, i64) {
        (
            ((x - self.origin[0]) / self.cell).floor() as i64,
            ((y - self.origin[1]) / self.cell).floor() as i64,
        )
    }

    /// Barycentric coordinates of `p`'s projection in triangle `ti` (XY).
    fn bary_xy(&self, ti: u32, p: [f64; 2]) -> Option<[f64; 3]> {
        let [a, b, c] = self.t[ti as usize].map(|k| self.v[k as usize]);
        let det = (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]);
        if det.abs() < 1e-300 {
            return None;
        }
        let l1 = ((p[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (p[1] - a[1])) / det;
        let l2 = ((b[0] - a[0]) * (p[1] - a[1]) - (p[0] - a[0]) * (b[1] - a[1])) / det;
        let l0 = 1. - l1 - l2;
        let e = -1e-12;
        (l0 >= e && l1 >= e && l2 >= e).then_some([l0, l1, l2])
    }

    fn candidates(&self, x: f64, y: f64) -> &[u32] {
        self.buckets.get(&self.key(x, y)).map_or(&[], Vec::as_slice)
    }

    fn point(&self, ti: u32, l: [f64; 3]) -> Vec3 {
        let [a, b, c] = self.t[ti as usize].map(|k| self.v[k as usize]);
        [0, 1, 2].map(|k| l[0] * a[k] + l[1] * b[k] + l[2] * c[k])
    }

    fn corner_normals(&self) -> Vec<[Vec3; 3]> {
        let topo = &self.topo;
        let e = self.surface_error;
        // Smooth edges: those bending less than a surface within e can.
        let smooth: Vec<bool> = (0..topo.edges.len())
            .map(|k| {
                let [f, g] = topo.faces[k];
                if g == NONE {
                    return false;
                }
                let [a, b] = topo.edges[k];
                let (pa, pb) = (self.v[a as usize], self.v[b as usize]);
                let width = |f: u32| {
                    let c = self.v[self.t[f as usize]
                        .iter()
                        .copied()
                        .find(|&x| x != a && x != b)
                        .unwrap() as usize];
                    norm(cross(sub(pb, pa), sub(c, pa))) / norm(sub(pb, pa))
                };
                let (nf, ng) = (self.n[f as usize], self.n[g as usize]);
                let bend = norm(cross(nf, ng)).atan2(dot(nf, ng));
                bend <= 4. * e * (1. / width(f) + 1. / width(g))
            })
            .collect();
        let area_of = |f: u32| {
            let [a, b, c] = self.t[f as usize].map(|k| self.v[k as usize]);
            norm(cross(sub(b, a), sub(c, a)))
        };
        (0..self.t.len() as u32)
            .map(|f| {
                self.t[f as usize].map(|v| {
                    // The triangles around v reached from f across smooth
                    // edges at v.
                    let mut seen = vec![f];
                    let mut stack = vec![f];
                    while let Some(g) = stack.pop() {
                        for &k in &topo.tri_edges[g as usize] {
                            if !topo.edges[k as usize].contains(&v) || !smooth[k as usize] {
                                continue;
                            }
                            let h = topo.other(k, g);
                            if h != NONE && !seen.contains(&h) {
                                seen.push(h);
                                stack.push(h);
                            }
                        }
                    }
                    // At the drive surface's boundary the faces round v lie
                    // on one side only, and their average tilts off the
                    // boundary: there the faces along the boundary at v
                    // give its normal (as for a boundary pass).
                    let on_boundary = |g: &u32| {
                        topo.tri_edges[*g as usize].iter().any(|&k| {
                            topo.faces[k as usize][1] == NONE && topo.edges[k as usize].contains(&v)
                        })
                    };
                    if seen.iter().any(on_boundary) {
                        seen.retain(on_boundary);
                    }
                    // Weighted by area: a needle or cap triangle (a
                    // tessellator's fan at a pole) has an angle near 180
                    // degrees at one corner, which would dominate an
                    // angle-weighted normal there, but next to no area.
                    let mut m = [0.; 3];
                    for &g in &seen {
                        let w = area_of(g);
                        m = axpy(m, w, self.n[g as usize]);
                    }
                    scale(m, 1. / norm(m).max(1e-300))
                })
            })
            .collect()
    }

    /// The smooth normal at a point of triangle `ti`: its corner normals
    /// interpolated (barycentric), unit.
    fn smooth_normal(&self, ti: u32, p: Vec3) -> Vec3 {
        let [a, b, c] = self.t[ti as usize].map(|k| self.v[k as usize]);
        let n = self.n[ti as usize];
        let area = dot(cross(sub(b, a), sub(c, a)), n);
        let l = [
            dot(cross(sub(c, b), sub(p, b)), n) / area,
            dot(cross(sub(a, c), sub(p, c)), n) / area,
            dot(cross(sub(b, a), sub(p, a)), n) / area,
        ];
        let corners = self.corner_normals[ti as usize];
        let mut m = [0.; 3];
        for k in 0..3 {
            for j in 0..3 {
                m[j] += l[k].clamp(0., 1.) * corners[k][j];
            }
        }
        scale(m, 1. / norm(m).max(1e-300))
    }

    /// The drive surface point over `xy` with the largest Z (a seed's
    /// point), as (triangle, barycentric, point).
    fn top_at(&self, xy: [f64; 2]) -> Option<(u32, [f64; 3], Vec3)> {
        let mut best: Option<(u32, [f64; 3], Vec3)> = None;
        for &ti in self.candidates(xy[0], xy[1]) {
            if let Some(l) = self.bary_xy(ti, xy) {
                let p = self.point(ti, l);
                if best.is_none_or(|(_, _, q)| p[2] > q[2]) {
                    best = Some((ti, l, p));
                }
            }
        }
        best
    }

    /// The drive triangle containing the 3D point `p` (its projection in
    /// the triangle, within `tol` of its plane), if any.
    /// The drive triangle `p` lies on, within `tol` (its distance to the
    /// triangle in space, so steep facets, whose plan view is a sliver, are
    /// found as reliably as flat ones), and the barycentric coordinates of
    /// its closest point there; the nearest such triangle.
    fn locate(&self, p: Vec3, tol: f64) -> Option<(u32, [f64; 3])> {
        let mut best: Option<(f64, u32, Vec3)> = None;
        // Every cell the tolerance reaches: a triangle within `tol` may be
        // filed only in a neighbouring cell (its plan box ends at the cell
        // border, or at the mesh's edge with `p` just outside by rounding).
        let (lo, hi) = (
            self.key(p[0] - tol, p[1] - tol),
            self.key(p[0] + tol, p[1] + tol),
        );
        let cells = (lo.0..=hi.0).flat_map(|x| (lo.1..=hi.1).map(move |y| (x, y)));
        for &ti in cells.flat_map(|k| self.buckets.get(&k).map_or(&[][..], Vec::as_slice)) {
            let [a, b, c] = self.t[ti as usize].map(|k| self.v[k as usize]);
            let q = closest_on_triangle(p, a, b, c);
            let d = norm(sub(q, p));
            if d <= tol && best.is_none_or(|x| d < x.0 || (d == x.0 && ti < x.1)) {
                best = Some((d, ti, q));
            }
        }
        best.map(|(_, ti, q)| {
            let [a, b, c] = self.t[ti as usize].map(|k| self.v[k as usize]);
            let (v0, v1, v2) = (sub(b, a), sub(c, a), sub(q, a));
            let (d00, d01, d11, d20, d21) = (
                dot(v0, v0),
                dot(v0, v1),
                dot(v1, v1),
                dot(v2, v0),
                dot(v2, v1),
            );
            let den = d00 * d11 - d01 * d01;
            let (l1, l2) = if den > 0. {
                ((d11 * d20 - d01 * d21) / den, (d00 * d21 - d01 * d20) / den)
            } else {
                (0., 0.)
            };
            (ti, [1. - l1 - l2, l1, l2])
        })
    }

    /// The triangles within distance `r` of `c`, sorted.
    fn within(&self, c: Vec3, r: f64) -> Vec<u32> {
        let (a, b) = (self.key(c[0] - r, c[1] - r), self.key(c[0] + r, c[1] + r));
        let mut out = vec![];
        for x in a.0..=b.0 {
            for y in a.1..=b.1 {
                for &ti in self.buckets.get(&(x, y)).map_or(&[][..], Vec::as_slice) {
                    let [p, q, s] = self.t[ti as usize].map(|k| self.v[k as usize]);
                    if norm(sub(closest_on_triangle(c, p, q, s), c)) <= r {
                        out.push(ti);
                    }
                }
            }
        }
        out.sort();
        out.dedup();
        out
    }

    /// The points of the drive surface within `tol` of the sphere of
    /// radius `r` about `c` (the ball resting there touches nowhere
    /// nearer): per triangle, its closest point to `c`.
    fn touching(&self, c: Vec3, r: f64, tol: f64) -> Vec<(u32, Vec3)> {
        let (a, b) = (self.key(c[0] - r, c[1] - r), self.key(c[0] + r, c[1] + r));
        let mut seen = std::collections::HashSet::new();
        let mut out = vec![];
        for x in a.0..=b.0 {
            for y in a.1..=b.1 {
                for &ti in self.buckets.get(&(x, y)).map_or(&[][..], Vec::as_slice) {
                    if !seen.insert(ti) {
                        continue;
                    }
                    let [p, q, s] = self.t[ti as usize].map(|k| self.v[k as usize]);
                    let near = closest_on_triangle(c, p, q, s);
                    if norm(sub(near, c)) <= r + tol {
                        out.push((ti, near));
                    }
                }
            }
        }
        // One point per contact: triangles sharing an edge or vertex give
        // the same point there.
        out.sort_by(|p, q| p.1.partial_cmp(&q.1).unwrap_or(Ordering::Equal));
        out.dedup_by(|p, q| norm(sub(p.1, q.1)) <= tol);
        out
    }

    /// The parameter interval [lo, hi] within [-r, r] over which m + s u
    /// stays in triangle `ti` (u in its plane).
    fn clip(&self, ti: u32, m: Vec3, u: Vec3, r: f64) -> Option<(f64, f64)> {
        let p = self.t[ti as usize].map(|k| self.v[k as usize]);
        let n = self.n[ti as usize];
        let (mut lo, mut hi) = (-r, r);
        for i in 0..3 {
            let (a, b) = (p[(i + 1) % 3], p[(i + 2) % 3]);
            let e = sub(b, a);
            let lam = |q: Vec3| dot(cross(e, sub(q, a)), n);
            let (l0, l1) = (lam(m), dot(cross(e, u), n));
            if l1.abs() < 1e-300 {
                if l0 < 0. {
                    return None;
                }
                continue;
            }
            let s = -l0 / l1;
            if l1 > 0. {
                lo = lo.max(s);
            } else {
                hi = hi.min(s);
            }
        }
        (hi > lo).then_some((lo, hi))
    }
}

/// The point of triangle (a, b, c) closest to p (Ericson, Real-Time
/// Collision Detection, 5.1.5).
fn closest_on_triangle(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> Vec3 {
    let (ab, ac, ap) = (sub(b, a), sub(c, a), sub(p, a));
    let (d1, d2) = (dot(ab, ap), dot(ac, ap));
    if d1 <= 0. && d2 <= 0. {
        return a;
    }
    let bp = sub(p, b);
    let (d3, d4) = (dot(ab, bp), dot(ac, bp));
    if d3 >= 0. && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0. && d1 >= 0. && d3 <= 0. {
        return axpy(a, d1 / (d1 - d3), ab);
    }
    let cp = sub(p, c);
    let (d5, d6) = (dot(ab, cp), dot(ac, cp));
    if d6 >= 0. && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0. && d2 >= 0. && d6 <= 0. {
        return axpy(a, d2 / (d2 - d6), ac);
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0. && d4 - d3 >= 0. && d5 - d6 >= 0. {
        return axpy(b, (d4 - d3) / ((d4 - d3) + (d5 - d6)), sub(c, b));
    }
    let den = 1. / (va + vb + vc);
    axpy(axpy(a, vb * den, ab), vc * den, ac)
}

/// The longest side of a triangle.
fn diameter(p: [Vec3; 3]) -> f64 {
    (0..3)
        .map(|k| norm(sub(p[(k + 1) % 3], p[k])))
        .fold(0., f64::max)
}

/// Whether `q` (in the plane of the triangle `p`, normal `n`) lies in it.
fn in_triangle(p: [Vec3; 3], n: Vec3, q: Vec3) -> bool {
    let e = -1e-12 * dot(cross(sub(p[1], p[0]), sub(p[2], p[0])), n).abs();
    (0..3).all(|i| dot(cross(sub(p[(i + 1) % 3], p[i]), sub(q, p[i])), n) >= e)
}

fn scale(a: Vec3, k: f64) -> Vec3 {
    [a[0] * k, a[1] * k, a[2] * k]
}
fn axpy(a: Vec3, k: f64, b: Vec3) -> Vec3 {
    [a[0] + k * b[0], a[1] + k * b[1], a[2] + k * b[2]]
}

/// The exact iso-curve of a field at one level, over some faces: crossing
/// nodes on edges (shared by both faces) and inside faces, and the links
/// between them, one per face region.
struct Iso<'a> {
    drive: &'a Drive,
    field: &'a Field<'a>,
    /// Each face's interior peaks (`Field::face_peaks`).
    peaks: &'a [Vec<(Vec3, f64)>],
    level: f64,
    /// How far a link's points may stray from the curve.
    tol: f64,
    nodes: Vec<Vec3>,
    edges: HashMap<u32, Side>,
    /// The node of a crossing at a mesh vertex, shared by every edge there.
    at_vertex: HashMap<u32, u32>,
    /// (from node, to node, face, the curve's points between them).
    links: Vec<(u32, u32, u32, Vec<Vec3>)>,
}

impl<'a> Iso<'a> {
    fn edge(&mut self, e: u32) -> Side {
        if let Some(c) = self.edges.get(&e) {
            return c.clone();
        }
        let [a, b] = self.drive.topo.edges[e as usize];
        let (pa, pb) = (self.drive.v[a as usize], self.drive.v[b as usize]);
        let mut c: Side = vec![];
        for s in self.field.edge_crossings(e, self.level) {
            // A crossing at an end (within rounding) is that vertex: one
            // node for every edge there, so pieces meeting at a vertex
            // chain whichever edges the rounding puts it on.
            let end = if s <= 1e-9 {
                Some((a, 0.))
            } else if s >= 1. - 1e-9 {
                Some((b, 1.))
            } else {
                None
            };
            let entry = match end {
                Some((k, t)) => {
                    let node = match self.at_vertex.get(&k) {
                        Some(&n) => n,
                        None => {
                            self.nodes.push(self.drive.v[k as usize]);
                            let n = self.nodes.len() as u32 - 1;
                            self.at_vertex.insert(k, n);
                            n
                        }
                    };
                    (t, node)
                }
                None => {
                    self.nodes.push(lerp(pa, pb, s));
                    (s, self.nodes.len() as u32 - 1)
                }
            };
            if c.last().is_none_or(|x: &(f64, u32)| x.1 != entry.1) {
                c.push(entry);
            }
        }
        self.edges.insert(e, c.clone());
        c
    }

    fn face(&mut self, f: u32) {
        let tri = self.drive.t[f as usize];
        let p = tri.map(|k| self.drive.v[k as usize]);
        let vals = tri.map(|k| self.field.vertex[k as usize]);
        let signs = vals.map(|d| d >= self.level);
        let sides = [0, 1, 2].map(|k| {
            let e = self.drive.topo.tri_edges[f as usize][k];
            let c = self.edge(e);
            if self.drive.topo.edges[e as usize][0] == tri[k] {
                c
            } else {
                c.iter().rev().map(|&(s, n)| (1. - s, n)).collect()
            }
        });
        let q = FaceQuery::new(self.field, f);
        self.region(&q, f, p, vals, signs, sides, 0);
    }

    /// The contour inside a triangle of face `f` (corner distances `vals`,
    /// corner signs, crossings along its sides). Two crossings are joined;
    /// more, or none with an island inside (an exact maximum above the
    /// level, or a source with the level around it), are resolved by
    /// splitting it into four with exact crossings on the new sides.
    #[allow(clippy::too_many_arguments)]
    fn region(
        &mut self,
        q: &FaceQuery<'a>,
        f: u32,
        p: [Vec3; 3],
        vals: [f64; 3],
        signs: [bool; 3],
        sides: [Side; 3],
        depth: usize,
    ) {
        // A crossing at a corner appears on both sides there: once.
        let mut ring: Vec<u32> = vec![];
        for x in sides.iter().flatten() {
            if ring.last() != Some(&x.1) {
                ring.push(x.1);
            }
        }
        if ring.len() > 1 && ring.first() == ring.last() {
            ring.pop();
        }
        let level = self.level;
        match ring.len() {
            0 => {
                // No crossing on its sides: the level may still close a
                // loop inside it, round an interior peak above the level or
                // a source (the only minima) below it.
                let n = self.drive.n[f as usize];
                let island = depth < MAX_DEPTH
                    && if signs.iter().all(|s| !s) {
                        self.peaks[f as usize]
                            .iter()
                            .any(|&(q, d)| d >= level && in_triangle(p, n, q))
                    } else if signs.iter().all(|s| *s) && level > 0. {
                        self.field.valleys(f).iter().any(|&q| in_triangle(p, n, q))
                    } else {
                        false
                    };
                if island {
                    self.split(q, f, p, vals, signs, sides, depth);
                }
            }
            2 => {
                // One arc between the two crossings, refined onto the
                // curve. Where the refinement cannot reach the curve (the
                // arc turns back on itself: it rounds a corner where the
                // nearest source changes) the triangle is split, down to
                // where any chord across it is within the tolerance.
                let (a, b) = (self.nodes[ring[0] as usize], self.nodes[ring[1] as usize]);
                let mut mid = vec![];
                if self.chord(q, f, a, b, self.tol, 0, &mut mid) || diameter(p) <= self.tol {
                    self.links.push((ring[0], ring[1], f, mid));
                } else {
                    self.split(q, f, p, vals, signs, sides, depth);
                }
            }
            _ if depth < MAX_DEPTH || diameter(p) > self.tol => {
                self.split(q, f, p, vals, signs, sides, depth)
            }
            _ => {
                // Below a micrometre: join each crossing to the next where
                // the side between them is below the level.
                let mut s = signs[0];
                for i in 0..ring.len() {
                    s = !s;
                    if !s {
                        self.links
                            .push((ring[i], ring[(i + 1) % ring.len()], f, vec![]));
                    }
                }
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn split(
        &mut self,
        q: &FaceQuery<'a>,
        f: u32,
        p: [Vec3; 3],
        vals: [f64; 3],
        signs: [bool; 3],
        sides: [Side; 3],
        depth: usize,
    ) {
        let m = [0, 1, 2].map(|k| lerp(p[k], p[(k + 1) % 3], 0.5));
        let dm = m.map(|x| q.at(x));
        let halves = |side: &Side| -> (Side, Side, bool) {
            let a: Side = side
                .iter()
                .filter(|x| x.0 < 0.5)
                .map(|&(s, n)| (2. * s, n))
                .collect();
            let b: Side = side
                .iter()
                .filter(|x| x.0 >= 0.5)
                .map(|&(s, n)| (2. * s - 1., n))
                .collect();
            let odd = a.len() % 2 == 1;
            (a, b, odd)
        };
        let (h0a, h0b, o0) = halves(&sides[0]);
        let (h1a, h1b, o1) = halves(&sides[1]);
        let (h2a, h2b, o2) = halves(&sides[2]);
        // The sign at each midpoint follows from its side's crossings.
        let sm = [signs[0] ^ o0, signs[1] ^ o1, signs[2] ^ o2];
        let mut inner = |a: usize, b: usize| -> Side {
            q.crossings_on(m[a], m[b], self.level, sm[a], sm[b])
                .into_iter()
                .map(|s| {
                    self.nodes.push(lerp(m[a], m[b], s));
                    (s, self.nodes.len() as u32 - 1)
                })
                .collect()
        };
        let (i01, i12, i02) = (inner(0, 1), inner(1, 2), inner(0, 2));
        let rev = |s: &Side| -> Side { s.iter().rev().map(|&(t, n)| (1. - t, n)).collect() };
        let d = depth + 1;
        self.region(
            q,
            f,
            [p[0], m[0], m[2]],
            [vals[0], dm[0], dm[2]],
            [signs[0], sm[0], sm[2]],
            [h0a, i02.clone(), h2b],
            d,
        );
        self.region(
            q,
            f,
            [m[0], p[1], m[1]],
            [dm[0], vals[1], dm[1]],
            [sm[0], signs[1], sm[1]],
            [h0b, h1a, rev(&i01)],
            d,
        );
        self.region(
            q,
            f,
            [m[2], m[1], p[2]],
            [dm[2], dm[1], vals[2]],
            [sm[2], sm[1], signs[2]],
            [rev(&i12), h1b, h2a],
            d,
        );
        self.region(q, f, m, dm, sm, [i01, i12, rev(&i02)], d);
    }

    /// Points of the curve between two of its points in face `f`: the
    /// chord's midpoint is moved onto the curve (the exact crossing on the
    /// chord's normal) until it is within `tol` of it. False if some
    /// chord's normal does not reach the curve inside the face.
    #[allow(clippy::too_many_arguments)]
    fn chord(
        &self,
        q: &FaceQuery<'a>,
        f: u32,
        a: Vec3,
        b: Vec3,
        tol: f64,
        depth: usize,
        out: &mut Vec<Vec3>,
    ) -> bool {
        let d = sub(b, a);
        let l = norm(d);
        if l <= tol {
            return true;
        }
        if depth >= 24 {
            return false;
        }
        let m = lerp(a, b, 0.5);
        let u = scale(cross(self.drive.n[f as usize], d), 1. / l);
        let Some((s0, s1)) = self.drive.clip(f, m, u, l) else {
            return false;
        };
        let (q0, q1) = (axpy(m, s0, u), axpy(m, s1, u));
        let (g0, g1) = (q.at(q0) >= self.level, q.at(q1) >= self.level);
        let Some(z) = q
            .crossings_on(q0, q1, self.level, g0, g1)
            .into_iter()
            .map(|t| s0 + t * (s1 - s0))
            .min_by(|x, y| x.abs().total_cmp(&y.abs()))
        else {
            return false;
        };
        if z.abs() <= tol {
            return true;
        }
        let mq = axpy(m, z, u);
        let ok = self.chord(q, f, a, mq, tol, depth + 1, out);
        out.push(mq);
        ok && self.chord(q, f, mq, b, tol, depth + 1, out)
    }

    /// The links chained into contact paths: open chains (from a node with
    /// one link: the drive boundary, or the end of the faces given) first,
    /// then loops.
    fn paths(&self) -> Vec<ContactPath> {
        let mut adj: HashMap<u32, Vec<usize>> = HashMap::new();
        // A contour along a mesh edge (from vertex to vertex, straight) is
        // linked by both faces there: one segment, once. (Two links between
        // the same nodes through different faces are two arcs of a ring.)
        // None from a node to itself.
        let vertex_of: HashMap<u32, u32> = self.at_vertex.iter().map(|(&v, &n)| (n, v)).collect();
        let along_edge = |a: u32, b: u32| match (vertex_of.get(&a), vertex_of.get(&b)) {
            (Some(&u), Some(&v)) => self.drive.topo.vertex_faces[u as usize]
                .iter()
                .any(|&f| self.drive.t[f as usize].contains(&v)),
            _ => false,
        };
        let mut seen = std::collections::HashSet::new();
        for (i, (a, b, _, mid)) in self.links.iter().enumerate() {
            let (a, b) = (*a, *b);
            if a == b || (mid.is_empty() && along_edge(a, b) && !seen.insert((a.min(b), a.max(b))))
            {
                continue;
            }
            adj.entry(a).or_default().push(i);
            adj.entry(b).or_default().push(i);
        }
        let normal = |node: u32| {
            let mut s = [0.; 3];
            for &i in &adj[&node] {
                let nn = self.drive.n[self.links[i].2 as usize];
                for k in 0..3 {
                    s[k] += nn[k];
                }
            }
            scale(s, 1. / norm(s).max(1e-300))
        };
        let mut used = vec![false; self.links.len()];
        let mut keys: Vec<u32> = adj.keys().copied().collect();
        keys.sort();
        let walk = |start: u32, used: &mut Vec<bool>| {
            let mut path = vec![(self.nodes[start as usize], normal(start))];
            let mut at = start;
            while let Some(&i) = adj[&at].iter().find(|&&i| !used[i]) {
                used[i] = true;
                let (a, b, f, ref mid) = self.links[i];
                let next = if a == at { b } else { a };
                let nf = self.drive.n[f as usize];
                if a == at {
                    path.extend(mid.iter().map(|&q| (q, nf)));
                } else {
                    path.extend(mid.iter().rev().map(|&q| (q, nf)));
                }
                path.push((self.nodes[next as usize], normal(next)));
                at = next;
                if at == start {
                    break;
                }
            }
            path
        };
        let mut out = vec![];
        for &k in &keys {
            if adj[&k].len() == 1 && !used[adj[&k][0]] {
                let c = walk(k, &mut used);
                if c.len() >= 2 {
                    out.push(c);
                }
            }
        }
        for &k in &keys {
            while adj[&k].iter().any(|&i| !used[i]) {
                let c = walk(k, &mut used);
                if c.len() >= 2 {
                    out.push(c);
                }
            }
        }
        out
    }
}

/// The exact iso-curves of `field` at `level` over `faces`, as contact
/// paths (points within `tol` of the curve, with their faces' normals).
fn contour(
    drive: &Drive,
    field: &Field,
    peaks: &[Vec<(Vec3, f64)>],
    level: f64,
    faces: &[u32],
    tol: f64,
) -> Vec<ContactPath> {
    let mut iso = Iso {
        drive,
        field,
        peaks,
        level,
        tol: 0.225 * tol,
        nodes: vec![],
        edges: HashMap::new(),
        at_vertex: HashMap::new(),
        links: vec![],
    };
    let started = std::time::Instant::now();
    for &f in faces {
        iso.face(f);
    }
    trace::line(|| {
        format!(
            "contour {level:.3}: {} faces, {} links, {} nodes in {:?}",
            faces.len(),
            iso.links.len(),
            iso.nodes.len(),
            started.elapsed()
        )
    });
    // Each pass within half the path error of its level (so neighbouring
    // passes are within the path error of the stepover apart): a quarter for
    // the contour, a quarter for the cutter path's smoothing. Chords within
    // 0.225 of it of the curve, then the polyline simplified within 0.025
    // of it of itself (where the curve grazes a long sliver's side within
    // rounding, the exact crossings come micrometres apart; their wiggles
    // are below any tolerance).
    iso.paths()
        .into_iter()
        .map(|path| simplify(&path, 0.025 * tol))
        .collect()
}

/// A polyline simplified (Douglas and Peucker 1973): a subset of its points
/// (so still on the surface, its ends kept, a closed path closed), every
/// dropped point within `eps` of the chord that replaces it.
fn simplify(path: &ContactPath, eps: f64) -> ContactPath {
    let n = path.len();
    if n < 3 {
        return path.clone();
    }
    let mut keep = vec![false; n];
    keep[0] = true;
    keep[n - 1] = true;
    let mut stack = vec![(0usize, n - 1)];
    while let Some((i, j)) = stack.pop() {
        if j <= i + 1 {
            continue;
        }
        let (a, b) = (path[i].0, path[j].0);
        let ab = sub(b, a);
        let ll = dot(ab, ab);
        let (mut far, mut at) = (0f64, i);
        for (k, q) in path.iter().enumerate().take(j).skip(i + 1) {
            let t = if ll > 0. {
                (dot(sub(q.0, a), ab) / ll).clamp(0., 1.)
            } else {
                0.
            };
            let d = norm(sub(axpy(a, t, ab), q.0));
            if d > far {
                (far, at) = (d, k);
            }
        }
        if far > eps {
            keep[at] = true;
            stack.push((i, at));
            stack.push((at, j));
        }
    }
    path.iter()
        .zip(keep)
        .filter(|(_, k)| *k)
        .map(|(q, _)| *q)
        .collect()
}

/// Each face's interior peaks that matter for the levels (offset + k) x
/// stepover: those reaching a level above the face's edges' maximum, or
/// above `top` (the edges' maximum over the surface).
fn level_peaks(
    drive: &Drive,
    field: &Field,
    emax: &[f64],
    stepover: f64,
    offset: f64,
    top: f64,
    cancel: &impl CancelSignal,
) -> Result<Vec<Vec<(Vec3, f64)>>, String> {
    let nf = drive.t.len();
    let workers = std::thread::available_parallelism().map_or(1, usize::from);
    let chunks = (nf / 512).clamp(1, 256);
    Ok(crate::roughing::parallel::ordered(
        chunks,
        workers,
        cancel,
        |c, signal| {
            let mut out = vec![];
            for f in c * nf / chunks..(c + 1) * nf / chunks {
                if f % 1024 == 0 && signal.is_cancelled() {
                    return Err("geodesic planning cancelled".into());
                }
                let lb = drive.topo.tri_edges[f]
                    .iter()
                    .map(|&e| emax[e as usize])
                    .fold(f64::NEG_INFINITY, f64::max);
                let k = ((lb / stepover - offset).floor() + 1.).max(0.);
                let next = (offset + k) * stepover;
                out.push(field.face_peaks_above(f as u32, (next - 1e-9 * (1. + next)).min(top)));
            }
            Ok(out)
        },
        |_| Ok(()),
    )?
    .into_iter()
    .flatten()
    .collect())
}

/// The exact distance from a set of contact paths (polygonal sources).
fn paths_field<'a>(
    drive: &'a Drive,
    paths: &[&ContactPath],
    limit: f64,
    cancel: &impl CancelSignal,
) -> Result<Field<'a>, String> {
    let mut sources = vec![];
    let t_walk = std::time::Instant::now();
    let mut slow = 0;
    for path in paths {
        let mut prev: Option<(u32, Vec3)> = None;
        for &(q, _) in path.iter() {
            let here = drive.locate(q, 1e-6).map(|(f, _)| (f, q));
            match (prev, here) {
                (Some((f, a)), Some(_)) => {
                    let t = std::time::Instant::now();
                    let r = walk(
                        &drive.v,
                        &drive.t,
                        &drive.topo,
                        f,
                        a,
                        [q[0], q[1]],
                        &mut sources,
                    );
                    if t.elapsed().as_millis() > 20 && trace::on() && slow < 5 {
                        slow += 1;
                        trace::line(|| {
                            format!(
                                "slow walk {:?} from face {f} {a:?} to {q:?} ok {}",
                                t.elapsed(),
                                r.is_some()
                            )
                        });
                    }
                }
                (None, Some((f, a))) => sources.push(Piece::Point(f, a)),
                _ => {}
            }
            prev = here;
        }
    }
    if trace::on() {
        trace::line(|| {
            format!(
                "paths_field: walked {} paths into {} pieces in {:?}",
                paths.len(),
                sources.len(),
                t_walk.elapsed()
            )
        });
    }
    Field::compute(&drive.v, &drive.t, &drive.topo, &sources, limit, cancel)
}

/// Fill passes until every point of the drive surface is within half a
/// stepover (plus the chord tolerance) of a pass. The distance F from all
/// passes is exact, and so is its maximum over each face. In each region
/// where F exceeds the target, its worst point c gets a spur: from the
/// nearest point of the passes straight to c (on the surface), spliced into
/// that pass as an excursion. Where the spur cannot run on the surface, the
/// seed field's level F(c)/2 below c instead (c is that far from it along
/// the surface). Repeated until covered; returns the passes added and the
/// certified coverage. Not converging is refused.
#[allow(clippy::too_many_arguments)]
fn fill(
    drive: &Drive,
    field: &Field,
    peaks: &[Vec<(Vec3, f64)>],
    passes: &mut Vec<(usize, bool, f64, ContactPath)>,
    stepover: f64,
    path_error: f64,
    band: &dyn Fn(f64) -> usize,
    cancel: &impl CancelSignal,
) -> Result<(usize, f64), String> {
    let target = stepover / 2. + path_error;
    let topo = &drive.topo;
    let nf = drive.t.len();
    let workers = std::thread::available_parallelism().map_or(1, usize::from);
    let mut added = 0;
    // The distance from the passes, kept to `limit` (farther is a gap all
    // the same); each round adds the field of its new passes alone, the
    // distance being the least of them (nothing computed is redone).
    let limit = 2. * stepover + target;
    let paths: Vec<&ContactPath> = passes.iter().map(|p| &p.3).collect();
    let t_first = std::time::Instant::now();
    let mut near = Coverage {
        fields: vec![paths_field(drive, &paths, limit, cancel)?],
    };
    if trace::on() {
        trace::line(|| {
            format!(
                "fill: field of {} passes in {:?}",
                paths.len(),
                t_first.elapsed()
            )
        });
    }
    // Per face: its maximum and where (exact where above the target, else
    // at most the target). Passes only lower the distance, so a face once
    // within the target stays so: only faces still above are searched.
    let mut maxima: Vec<(f64, Vec3)> = vec![(f64::INFINITY, [0.; 3]); nf];
    let mut open: Vec<usize> = (0..nf).collect();
    let timing = trace::on();
    for round in 0..256 {
        let t0 = std::time::Instant::now();
        let chunks = (open.len() / 64).clamp(1, 4 * workers);
        let found = crate::roughing::parallel::ordered(
            chunks,
            workers,
            cancel,
            |c, signal| {
                let mut out = vec![];
                for &f in &open[c * open.len() / chunks..(c + 1) * open.len() / chunks] {
                    if signal.is_cancelled() {
                        return Err("geodesic planning cancelled".into());
                    }
                    let tf = std::time::Instant::now();
                    let peaks = near.face_peaks_above(f as u32, target);
                    let m = near.face_max(f as u32, &peaks);
                    if timing && tf.elapsed().as_millis() > 200 {
                        let tri = drive.t[f].map(|k| drive.v[k as usize]);
                        trace::line(|| {
                            format!(
                                "   slow face {f}: {:?} functions {} max {:.4} tri {tri:?}",
                                tf.elapsed(),
                                near.regions(f as u32).len(),
                                m.0
                            )
                        });
                    }
                    out.push((m, peaks));
                }
                Ok(out)
            },
            |_| Ok(()),
        )?;
        // Every point above the target found in a face (its maximum and
        // its interior peaks) is a candidate for a fill, not only the
        // maximum: a long ridge across one large face is filled in one
        // round, not one spur per round.
        let mut candidates: Vec<Vec<(f64, Vec3)>> = vec![vec![]; nf];
        for (&f, (m, peaks)) in open.iter().zip(found.into_iter().flatten()) {
            maxima[f] = m;
            candidates[f] = std::iter::once(m)
                .chain(peaks.into_iter().map(|(p, d)| (d, p)))
                .filter(|x| x.0 > target)
                .collect();
        }
        let searched = open.len();
        open.retain(|&f| maxima[f].0 > target);
        if timing {
            trace::line(|| {
                format!(
                    "round {round}: searched {searched} faces in {:?}, open {}, passes {}, fields {}",
                    t0.elapsed(),
                    open.len(),
                    passes.len(),
                    near.fields.len()
                )
            });
            if open.len() <= 4 {
                for &f in &open {
                    let (m, c) = maxima[f];
                    let tri = drive.t[f].map(|k| drive.v[k as usize]);
                    trace::line(|| {
                        format!(
                            "   open face {f}: max {m:.4} at {c:?} at() {:.4} tri {tri:?}",
                            near.at(f as u32, c)
                        )
                    });
                }
            }
        }
        if open.is_empty() {
            // Covered: no face exceeds the target anywhere (its maximum,
            // exact wherever above the target, is at most that). The
            // certificate is the target itself.
            return Ok((added, target));
        }
        // Unreached within the limit: at least that far.
        let worst: Vec<Option<(f64, Vec3)>> = maxima
            .iter()
            .map(|&(m, at)| (m > target).then_some((m.min(limit), at)))
            .collect();
        let t_spurs = std::time::Instant::now();
        // Regions above the target, by shared edges.
        let mut seen = vec![false; nf];
        let mut fresh = vec![];
        for start in 0..nf {
            if seen[start] || worst[start].is_none() {
                continue;
            }
            let mut comp = vec![start];
            seen[start] = true;
            let mut i = 0;
            while i < comp.len() {
                let f = comp[i];
                i += 1;
                for &e in &topo.tri_edges[f] {
                    let g = topo.other(e, f as u32);
                    if g != NONE && !seen[g as usize] && worst[g as usize].is_some() {
                        seen[g as usize] = true;
                        comp.push(g as usize);
                    }
                }
            }
            // Greedily, worst first: a fill through each point not already
            // within reach of one chosen this round.
            let mut order: Vec<(usize, f64, Vec3)> = comp
                .iter()
                .flat_map(|&f| {
                    candidates[f]
                        .iter()
                        .map(move |&(far, c)| (f, far.min(limit), c))
                })
                .collect();
            order.sort_by(|a, b| b.1.total_cmp(&a.1).then(a.0.cmp(&b.0)));
            // A point within the target of a fill already chosen this round
            // is covered by it (its distance to that fill is at most that).
            let mut chosen: Vec<ContactPath> = vec![];
            let near_fill = |c: Vec3, fills: &[ContactPath]| {
                fills.iter().any(|path| {
                    path.windows(2).any(|w| {
                        let (a, b) = (w[0].0, w[1].0);
                        let ab = sub(b, a);
                        let t = (dot(sub(c, a), ab) / dot(ab, ab).max(1e-300)).clamp(0., 1.);
                        norm(sub(axpy(a, t, ab), c)) <= target
                    }) || path.len() == 1 && norm(sub(path[0].0, c)) <= target
                })
            };
            for (f, far, c) in order {
                if near_fill(c, &chosen) {
                    continue;
                }
                let here = field.at(f as u32, c);
                // A spur: from the nearest point of the passes straight
                // (on the surface) to c and back, so c is on a pass. Where
                // F peaks it is on the field's ridge between two passes,
                // whose nearest points are on that ridge too: the spur runs
                // along it (a corner's cleanup).
                let mut nearest: Option<(f64, Vec3)> = None;
                for (_, _, _, path) in passes.iter() {
                    for w in path.windows(2) {
                        let (a, b) = (w[0].0, w[1].0);
                        let ab = sub(b, a);
                        let t = (dot(sub(c, a), ab) / dot(ab, ab).max(1e-300)).clamp(0., 1.);
                        let q = axpy(a, t, ab);
                        let d = norm(sub(q, c));
                        if nearest.is_none_or(|x| d < x.0) {
                            nearest = Some((d, q));
                        }
                    }
                }
                let mut spur = None;
                if let Some((_, q)) = nearest
                    && let Some((g, _)) = drive.locate(q, 1e-6)
                {
                    let mut pieces = vec![];
                    if walk(
                        &drive.v,
                        &drive.t,
                        &drive.topo,
                        g,
                        q,
                        [c[0], c[1]],
                        &mut pieces,
                    )
                    .is_some()
                    {
                        let mut path: ContactPath = vec![];
                        for piece in &pieces {
                            if let Piece::Segment(h, a, b) = *piece {
                                let n = drive.n[h as usize];
                                if path.is_empty() {
                                    path.push((a, n));
                                }
                                path.push((b, n));
                            }
                        }
                        if path.len() >= 2 {
                            spur = Some(path);
                        }
                    }
                }
                if timing {
                    trace::line(|| {
                        format!(
                            "   fill c {c:?} far {far:.4} nearest {:?} spur {:?}",
                            nearest,
                            spur.as_ref()
                                .map(|p| p.iter().map(|q| q.0).collect::<Vec<_>>())
                        )
                    });
                }
                if let Some(path) = spur {
                    chosen.push(path.clone());
                    // Not a level: its contacts are placed at its points.
                    fresh.push((band(here), true, f64::NAN, path));
                    continue;
                }
                // Otherwise the field's level F(c) / 2 below c (c is that
                // far from it along the surface).
                let level = here - far / 2.;
                if level <= 0. {
                    return Err(format!(
                        "geodesic finishing: a point {here:.4} mm from the seed is {far:.4} mm from every pass; this is a defect, please report it with the part"
                    ));
                }
                // Every face within F(c) of c (in space, never farther).
                let faces = drive.within(c, far);
                for path in contour(drive, field, peaks, level, &faces, path_error / 2.) {
                    chosen.push(path.clone());
                    fresh.push((band(level), true, level, path));
                }
            }
        }
        if timing {
            trace::line(|| format!("  spurs built in {:?}", t_spurs.elapsed()));
        }
        if fresh.is_empty() {
            break;
        }
        let new: Vec<&ContactPath> = fresh.iter().map(|p| &p.3).collect();
        let t1 = std::time::Instant::now();
        near.fields.push(paths_field(drive, &new, limit, cancel)?);
        if timing {
            trace::line(|| {
                format!(
                    "  {} new passes, their field in {:?}",
                    new.len(),
                    t1.elapsed()
                )
            });
        }
        added += fresh.len();
        passes.extend(fresh);
    }
    Err(format!(
        "geodesic finishing could not cover the surface within half the stepover ({} mm) after repeated fill passes; this is a defect, please report it with the part",
        stepover / 2.
    ))
}

/// Centre passes (Mastercam's HSM scallop "center passes": an extra pass
/// through the sharp corners of the scallop passes and their collapsed
/// areas). Where the levels meet the field's ridge (its cut locus, where two
/// shortest paths to the seed meet: the medial axis of a boundary's
/// offsets), their corners lie farther apart along the ridge than a
/// stepover, and the fill adds a spur in each gap, one per level: each cut
/// out and back, a full stop and reversal at its tip. Spurs whose tips lie
/// in consecutive bands, each the other's nearest (and within two
/// stepovers, so on the same ridge, its corners at most 60 degrees sharp),
/// are joined tip to tip along the surface into one open pass cut straight
/// through. Not a coverage argument: the fill certifies coverage again
/// afterwards (and adds spurs where these leave a gap). Spurs are the
/// passes from `from` on with a NaN level; chains get level infinity.
/// Returns the number of chains.
fn centre_passes(
    drive: &Drive,
    passes: &mut Vec<(usize, bool, f64, ContactPath)>,
    from: usize,
    stepover: f64,
) -> usize {
    let spurs: Vec<usize> = (from..passes.len())
        .filter(|&i| passes[i].2.is_nan() && passes[i].3.len() >= 2)
        .collect();
    let tip = |i: usize| passes[i].3[passes[i].3.len() - 1].0;
    // The tips joined into chains along the ridge they lie on: the
    // shortest joins first (within two stepovers), each tip joined at most
    // twice and never into a loop (greedy polyline reconstruction).
    let mut pairs = vec![];
    for (x, &i) in spurs.iter().enumerate() {
        for &j in &spurs[x + 1..] {
            let d = norm(sub(tip(i), tip(j)));
            if d <= 2. * stepover {
                pairs.push((d, i, j));
            }
        }
    }
    pairs.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut root: HashMap<usize, usize> = spurs.iter().map(|&i| (i, i)).collect();
    fn find(root: &mut HashMap<usize, usize>, i: usize) -> usize {
        let mut r = i;
        while root[&r] != r {
            r = root[&r];
        }
        root.insert(i, r);
        r
    }
    let mut links: HashMap<usize, Vec<usize>> = HashMap::new();
    for (_, i, j) in pairs {
        if links.get(&i).map_or(0, Vec::len) >= 2 || links.get(&j).map_or(0, Vec::len) >= 2 {
            continue;
        }
        let (ri, rj) = (find(&mut root, i), find(&mut root, j));
        if ri == rj {
            continue;
        }
        root.insert(ri, rj);
        links.entry(i).or_default().push(j);
        links.entry(j).or_default().push(i);
    }
    // Each chain from its outer end (the lower band: its spur leaves the
    // passes there), tip to tip.
    let mut next: HashMap<usize, usize> = HashMap::new();
    let mut has_prev: Vec<usize> = vec![];
    let mut seen: Vec<usize> = vec![];
    let mut ends: Vec<usize> = spurs
        .iter()
        .copied()
        .filter(|i| links.get(i).is_some_and(|l| l.len() == 1))
        .collect();
    ends.sort_by_key(|&i| (passes[i].0, i));
    for &e in &ends {
        if seen.contains(&e) {
            continue;
        }
        let (mut prev, mut at) = (usize::MAX, e);
        seen.push(e);
        while let Some(&j) = links[&at].iter().find(|&&j| j != prev) {
            next.insert(at, j);
            has_prev.push(j);
            seen.push(j);
            (prev, at) = (at, j);
        }
    }
    let mut chains: Vec<ContactPath> = vec![];
    let mut used = vec![];
    for &i in &spurs {
        if has_prev.contains(&i) || !next.contains_key(&i) {
            continue;
        }
        // The first spur from its pass, then tip to tip.
        let mut path = passes[i].3.clone();
        let mut at = i;
        used.push(i);
        while let Some(&j) = next.get(&at) {
            let (a, b) = (tip(at), tip(j));
            let Some((f, _)) = drive.locate(a, 1e-6) else {
                break;
            };
            let mut pieces = vec![];
            if walk(
                &drive.v,
                &drive.t,
                &drive.topo,
                f,
                a,
                [b[0], b[1]],
                &mut pieces,
            )
            .is_none()
            {
                break;
            }
            for piece in &pieces {
                if let Piece::Segment(h, _, q) = *piece {
                    path.push((q, drive.n[h as usize]));
                }
            }
            used.push(j);
            at = j;
        }
        chains.push(path);
    }
    let count = chains.len();
    let band = |i: usize| passes[i].0;
    let first_band = used.iter().map(|&i| band(i)).min().unwrap_or(0);
    let mut keep = vec![];
    for (i, p) in passes.drain(..).enumerate() {
        if !used.contains(&i) {
            keep.push(p);
        }
    }
    *passes = keep;
    for path in chains {
        passes.push((first_band, true, f64::INFINITY, path));
    }
    count
}

/// The drive surface's boundary edges chained into loops (or open chains
/// at a pinch), as contact paths.
fn boundary_loops(drive: &Drive) -> Vec<ContactPath> {
    let topo = &drive.topo;
    let open: Vec<usize> = (0..topo.edges.len())
        .filter(|&e| topo.faces[e][1] == NONE)
        .collect();
    let mut at_vertex: HashMap<u32, Vec<usize>> = HashMap::new();
    for (i, &e) in open.iter().enumerate() {
        for k in topo.edges[e] {
            at_vertex.entry(k).or_default().push(i);
        }
    }
    let normal = |e: usize| drive.n[topo.faces[e][0] as usize];
    let mut used = vec![false; open.len()];
    let mut out = vec![];
    for start in 0..open.len() {
        if used[start] {
            continue;
        }
        used[start] = true;
        let [a, mut at] = topo.edges[open[start]];
        let mut path: ContactPath = vec![
            (drive.v[a as usize], normal(open[start])),
            (drive.v[at as usize], normal(open[start])),
        ];
        while let Some(&i) = at_vertex[&at].iter().find(|&&i| !used[i]) {
            used[i] = true;
            let [p, q] = topo.edges[open[i]];
            at = if p == at { q } else { p };
            path.push((drive.v[at as usize], normal(open[i])));
        }
        out.push(path);
    }
    out
}

/// The planned passes: the seed's exact field, its iso-curves at each
/// level and the caps, as contact paths (level index, cap, level value,
/// path).
struct Plan<'a> {
    name: &'static str,
    field: Field<'a>,
    passes: Vec<(usize, bool, f64, ContactPath)>,
    levels: usize,
    /// Passes added where the levels left a point farther than half a
    /// stepover from every pass (a peak between levels, a slowly rising
    /// ridge).
    caps: usize,
    /// The certified bound on the distance from any point of the drive
    /// surface to the nearest planned pass.
    coverage: f64,
}

fn mark(t: &mut std::time::Instant, what: &str) {
    if trace::on() {
        trace::line(|| format!("phase {what}: {:?}", t.elapsed()));
    }
    *t = std::time::Instant::now();
}

/// A seed on the drive surface: its name, its pieces (the distance's
/// sources), the first level's offset in stepovers, the first level's
/// index, and the passes it is itself (the boundary, a curve).
#[allow(clippy::type_complexity)]
fn seeds(
    drive: &Drive,
    seed: &GeodesicSeed,
) -> Result<
    (
        &'static str,
        Vec<Piece>,
        f64,
        usize,
        Vec<(usize, bool, f64, ContactPath)>,
    ),
    String,
> {
    let mut contact_passes: Vec<(usize, bool, f64, ContactPath)> = vec![];
    let (name, sources, offset, first) = match seed {
        GeodesicSeed::Boundary => {
            let topo = &drive.topo;
            let open: Vec<usize> = (0..topo.edges.len())
                .filter(|&e| topo.faces[e][1] == NONE)
                .collect();
            if open.is_empty() {
                return Err("geodesic boundary seed: the drive surface has no boundary (a closed upward surface); give a point or curve seed".into());
            }
            let pieces: Vec<Piece> = open
                .iter()
                .map(|&e| {
                    let [a, b] = topo.edges[e];
                    Piece::Segment(topo.faces[e][0], drive.v[a as usize], drive.v[b as usize])
                })
                .collect();
            // The boundary itself is the first pass (so every point has a
            // pass no farther than its distance from the seed).
            for path in boundary_loops(drive) {
                contact_passes.push((0, false, 0., path));
            }
            ("boundary", pieces, 0.0, 1)
        }
        GeodesicSeed::Point { xy_mm } => {
            let (ti, _, p) = drive.top_at(*xy_mm).ok_or_else(|| {
                format!("geodesic seed point {xy_mm:?} is not over the drive surface")
            })?;
            // A seed within the weld tolerance of a vertex is that vertex
            // (the source of every face round it: a point a hair off a
            // vertex inside one sliver of a fan would reach the fan's other
            // faces only at grazing angles).
            let p = drive.t[ti as usize]
                .iter()
                .map(|&k| drive.v[k as usize])
                .find(|&q| norm(sub(q, p)) <= drive.weld)
                .unwrap_or(p);
            ("point", vec![Piece::Point(ti, p)], 0.5, 0)
        }
        GeodesicSeed::Curve { points_mm } => {
            if points_mm.len() < 2 {
                return Err("a geodesic seed curve needs at least two points".into());
            }
            let (mut f, _, mut p) = drive.top_at(points_mm[0]).ok_or_else(|| {
                format!(
                    "geodesic seed curve point {:?} is not over the drive surface",
                    points_mm[0]
                )
            })?;
            // The curve projected onto the surface, split per face.
            let mut pieces = vec![];
            for q in &points_mm[1..] {
                (f, p) = walk(&drive.v, &drive.t, &drive.topo, f, p, *q, &mut pieces).ok_or_else(
                    || format!("geodesic seed curve leaves the drive surface before {q:?}"),
                )?;
            }
            let mut path: ContactPath = vec![];
            for piece in &pieces {
                if let Piece::Segment(g, a, b) = *piece {
                    let n = drive.n[g as usize];
                    if path.is_empty() {
                        path.push((a, n));
                    }
                    path.push((b, n));
                }
            }
            contact_passes.push((0, false, 0., path));
            ("curve", pieces, 0.0, 1)
        }
    };
    Ok((name, sources, offset, first, contact_passes))
}

fn plan<'a>(
    drive: &'a Drive,
    seed: &GeodesicSeed,
    stepover: f64,
    path_error: f64,
    cancel: &impl CancelSignal,
) -> Result<Plan<'a>, String> {
    // Seeds on the drive surface. Rings from a point or the boundary start
    // half a stepover out (the seed within half a stepover of the first
    // pass); a curve seed is itself the first pass.
    let mut clock = std::time::Instant::now();
    let (name, sources, offset, first, mut contact_passes) = seeds(drive, seed)?;
    let field = Field::compute(
        &drive.v,
        &drive.t,
        &drive.topo,
        &sources,
        f64::INFINITY,
        cancel,
    )?;
    mark(&mut clock, "seed field");
    if (0..drive.v.len()).any(|k| !field.vertex[k].is_finite() && !drive.vt[k].is_empty()) {
        return Err("geodesic finishing: part of the drive surface is not connected to the seed (a gap, or separate faces); finish it with its own seed".into());
    }
    let value_of = |k: usize| (offset + k as f64) * stepover;
    let band = |x: f64| ((x / stepover - offset).floor().max(0.)) as usize;
    // Per face: Lipschitz bounds on the distance over it (which levels it
    // may meet), and the exact maximum over the surface (edges and interior
    // peaks).
    let topo = &drive.topo;
    let nf = drive.t.len();
    let emax: Vec<f64> = (0..topo.edges.len() as u32)
        .map(|e| field.edge_max(e))
        .collect();
    let (mut lo_f, mut hi_f) = (vec![0.; nf], vec![0.; nf]);
    let mut gmax = 0f64;
    for f in 0..nf {
        let (p, d) = (
            drive.t[f].map(|k| drive.v[k as usize]),
            drive.t[f].map(|k| field.vertex[k as usize]),
        );
        let r = |i: usize| (0..3).map(|j| norm(sub(p[i], p[j]))).fold(0., f64::max);
        lo_f[f] = (0..3)
            .map(|i| d[i] - r(i))
            .fold(f64::NEG_INFINITY, f64::max);
        hi_f[f] = (0..3).map(|i| d[i] + r(i)).fold(f64::INFINITY, f64::min);
        for &e in &topo.tri_edges[f] {
            gmax = gmax.max(emax[e as usize]);
        }
    }
    let workers = std::thread::available_parallelism().map_or(1, usize::from);
    // Interior peaks: the field can rise inside a face above its edges.
    let peaks = level_peaks(drive, &field, &emax, stepover, offset, gmax, cancel)?;
    mark(&mut clock, "emax+peaks");
    for list in &peaks {
        for &(_, d) in list {
            gmax = gmax.max(d);
        }
    }
    let levels = if gmax < value_of(first) {
        0
    } else {
        band(gmax) + 1
    };
    // The faces each level may cross (or hold an island of).
    let mut buckets: Vec<Vec<u32>> = vec![vec![]; levels];
    for f in 0..nf {
        let k0 = ((lo_f[f] / stepover - offset).ceil().max(first as f64)) as usize;
        let k1 = (hi_f[f] / stepover - offset).floor();
        if k1 < 0. {
            continue;
        }
        for bucket in buckets
            .iter_mut()
            .take((k1 as usize + 1).min(levels))
            .skip(k0)
        {
            bucket.push(f as u32);
        }
    }
    let rings = crate::roughing::parallel::ordered(
        levels.saturating_sub(first),
        workers,
        cancel,
        |i, _| {
            let k = first + i;
            Ok(contour(
                drive,
                &field,
                &peaks,
                value_of(k),
                &buckets[k],
                path_error / 2.,
            ))
        },
        |_| Ok(()),
    )?;
    mark(&mut clock, "contours");
    for (i, found) in rings.into_iter().enumerate() {
        for c in found {
            contact_passes.push((first + i, false, value_of(first + i), c));
        }
    }
    // Point and curve seeds: the boundary is cut as one contour pass too
    // (where the levels meet it at an angle, it covers the rim in one pass
    // instead of a fill between each pair of passes). Its level is minus
    // infinity: it is the boundary, not a level of the field (its contacts
    // are placed at its points), and the passes link along it.
    if name != "boundary" {
        for path in boundary_loops(drive) {
            contact_passes.push((levels, true, f64::NEG_INFINITY, path));
        }
    }
    // Coverage: every point within half a stepover (plus the chord
    // tolerance) of a pass, enforced and certified.
    let before = contact_passes.len();
    let (_, mut coverage) = fill(
        drive,
        &field,
        &peaks,
        &mut contact_passes,
        stepover,
        path_error,
        &band,
        cancel,
    )?;
    mark(&mut clock, "fill");
    // Spurs along a ridge chained into centre passes change the passes:
    // certified again, filled where the chains leave a gap, and the new
    // spurs chained in turn, until no spurs chain.
    let mut from = before;
    while centre_passes(drive, &mut contact_passes, from, stepover) > 0 {
        from = contact_passes.len();
        (_, coverage) = fill(
            drive,
            &field,
            &peaks,
            &mut contact_passes,
            stepover,
            path_error,
            &band,
            cancel,
        )?;
    }
    let caps = contact_passes.len() - before;
    contact_passes.sort_by_key(|(k, cap, _, _)| (*k, *cap));
    if contact_passes.is_empty() {
        return Err(format!(
            "geodesic finishing found no pass: stepover_mm {stepover} spans the whole drive surface"
        ));
    }
    Ok(Plan {
        name,
        field,
        passes: contact_passes,
        levels,
        caps,
        coverage,
    })
}

/// Geodesic passes (XY polylines of the ball centre), from the seed
/// outward, and the verification report.
#[allow(clippy::too_many_arguments)]
pub(crate) fn passes(
    mesh: &SurfaceMesh,
    surface: &PreparedSurface,
    stepover: f64,
    contact_radius: f64,
    tool_radius: f64,
    path_error: f64,
    seed: &GeodesicSeed,
    tool_centre: bool,
    slopes: Option<(f64, f64)>,
    work: &mut SurfaceWork,
    cancel: &impl CancelSignal,
) -> Result<Planned, String> {
    // The drive surface: the faces within the slope range (radians from
    // the horizontal), when one is given; steeper faces are left to a
    // waterline or steep-and-shallow pass.
    let limited;
    let mesh = match slopes {
        None => mesh,
        Some((lo, hi)) => {
            let guard = 1e-9;
            let kept: Vec<[Vec3; 3]> = mesh
                .triangles
                .iter()
                .filter(|t| {
                    let n = cross(sub(t[1], t[0]), sub(t[2], t[0]));
                    let l = norm(n);
                    l > 0. && {
                        let slope = (n[2] / l).clamp(-1., 1.).acos();
                        slope >= lo - guard && slope <= hi + guard
                    }
                })
                .copied()
                .collect();
            if kept.is_empty() {
                return Err("geodesic finishing: no face of the target lies within slope_from_deg..slope_to_deg".into());
            }
            limited = SurfaceMesh {
                triangles: std::sync::Arc::new(kept),
                ..mesh.clone()
            };
            &limited
        }
    };
    let drive = Drive::build(mesh, cancel)?;
    if tool_centre {
        return centre_mode(
            &drive,
            mesh,
            surface,
            stepover,
            contact_radius,
            tool_radius,
            path_error,
            seed,
            work,
            cancel,
        );
    }
    let Plan {
        name,
        field,
        passes: contact_passes,
        levels,
        caps,
        coverage,
    } = plan(&drive, seed, stepover, path_error, cancel)?;
    let workers = std::thread::available_parallelism().map_or(1, usize::from);
    // Cutter positions, solved so each achieved contact lies on its level.
    let solver = Solver {
        drive: &drive,
        field: &field,
        surface,
        r: contact_radius,
        tol: mesh.surface_error_mm + 1e-6,
        eps: 0.1 * path_error,
    };
    let solved = crate::roughing::parallel::ordered(
        contact_passes.len(),
        workers,
        cancel,
        |i, signal| {
            let stop = || signal.is_cancelled();
            let mut w = SurfaceWork::default();
            let (_, _, level, path) = &contact_passes[i];
            // The contacts with a vertex repeated for each position of
            // a corner's arc, and the positions.
            let closed = path.len() > 2 && norm(sub(path[0].0, path[path.len() - 1].0)) < 1e-9;
            let mut contacts: ContactPath = Vec::with_capacity(path.len());
            let mut out = Vec::with_capacity(path.len());
            for (i, &(_, nrm)) in path.iter().enumerate() {
                if closed && i + 1 == path.len() {
                    // The closing vertex: the first position again.
                    contacts.push(contacts[0]);
                    out.push(out[0]);
                    break;
                }
                for (cc, fit) in solver.vertex(path, i, *level, &mut w, &stop)? {
                    contacts.push((cc, nrm));
                    out.push(fit);
                }
            }
            let (cut, bridged) = solver.trim(&contacts, &out, *level, &mut w, &stop)?;
            Ok((out, cut, bridged, w))
        },
        |_| Ok(()),
    )?;
    // The cutter path smoothed within a quarter of the path error (the contour
    // takes another quarter).
    let smoothed = |out: Vec<([f64; 2], Fit)>| -> Vec<([f64; 2], Fit)> {
        let xy: Vec<[f64; 2]> = out.iter().map(|p| p.0).collect();
        smooth(&xy, path_error / 4.)
            .into_iter()
            .zip(out)
            .map(|(q, (_, fit))| (q, fit))
            .collect()
    };
    // Every vertex as solved (verified, its unreachable ones counted), and
    // the paths cut (unreachable runs bridged or cut off).
    let mut fits: Vec<Fitted> = vec![];
    let mut cuts: Vec<Fitted> = vec![];
    let mut bridged_spans = 0;
    let mut centre: Vec<Vec<[f64; 2]>> = vec![];
    // The drive surface's boundary passes: links may follow them.
    let mut rims: Vec<Vec<[f64; 2]>> = vec![];
    for ((k, cap, level, _), (out, cut, bridged, w)) in contact_passes.iter().zip(solved) {
        work.nodes += w.nodes;
        work.contact_tests += w.contact_tests;
        work.distance_tests += w.distance_tests;
        bridged_spans += bridged;
        if trace::on() {
            let un: Vec<[f64; 2]> = out
                .iter()
                .filter(|p| p.1 == Fit::Unreachable)
                .map(|p| p.0)
                .collect();
            if !un.is_empty() {
                trace::line(|| {
                    format!(
                        "pass k={k} cap={cap} level={level} unreachable {} of {} bridged {bridged} pieces {} first {:?} last {:?}",
                        un.len(),
                        out.len(),
                        cut.len(),
                        un[0],
                        un[un.len() - 1]
                    )
                });
            }
        }
        // Verified along the cutter path as cut, not only at its vertices:
        // between two reachable vertices the ball follows the chord (the
        // executor drops it along the straight move), and its contact
        // there can differ from both ends (a long straight move along a
        // rim between two corners). Samples at most a quarter stepover
        // apart.
        // A boundary pass is not smoothed: its positions are exact (one
        // face normal along each boundary edge, a true arc round each
        // corner: no facet-to-facet shake to relax), and moving them
        // sideways moves the contact far up a steep face at the edge.
        let rim = *level == f64::NEG_INFINITY || (name == "boundary" && *k == 0 && !*cap);
        let relax = |p: Vec<([f64; 2], Fit)>| if rim { p } else { smoothed(p) };
        let fit = relax(out);
        let mut dense = Vec::with_capacity(fit.len());
        for (i, &(q, f)) in fit.iter().enumerate() {
            if i > 0 {
                let (p, g) = fit[i - 1];
                if f != Fit::Unreachable && g != Fit::Unreachable {
                    let n = ((q[0] - p[0]).hypot(q[1] - p[1]) / (stepover / 4.)).ceil() as usize;
                    for j in 1..n {
                        let t = j as f64 / n as f64;
                        dense.push((
                            [p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])],
                            Fit::Placed,
                        ));
                    }
                }
            }
            dense.push((q, f));
        }
        fits.push((*k, *cap, dense));
        for cut in cut {
            if rim {
                rims.push(cut.iter().map(|p| p.0).collect());
            }
            if *level == f64::INFINITY {
                centre.push(smoothed(cut).into_iter().map(|p| p.0).collect());
            } else {
                cuts.push((*k, *cap, relax(cut)));
            }
        }
    }
    let checked = Checked {
        drive: &drive,
        planned: &field,
        surface,
        stepover,
        contact_radius,
        tool_radius,
    };
    let mut report = checked.verify(&fits, name, levels, caps, coverage, work, cancel)?;
    report.bridged_spans = bridged_spans;
    if trace::on() {
        for (i, (k, cap, c)) in cuts.iter().enumerate() {
            for w in c.windows(2) {
                if (w[1].0[0] - w[0].0[0]).hypot(w[1].0[1] - w[0].0[1]) > 10. {
                    trace::line(|| {
                        format!(
                            "cut {i} k={k} cap={cap} n={} long {:?} -> {:?}",
                            c.len(),
                            w[0].0,
                            w[1].0
                        )
                    });
                }
            }
        }
        for (i, c) in rims.iter().enumerate() {
            trace::line(|| format!("rim {i} n={} {:?}", c.len(), c));
        }
    }
    let (linked, stay_down) = link(
        &drive,
        surface,
        contact_radius,
        stepover,
        &cuts,
        centre,
        &rims,
        work,
        cancel,
    )?;
    if trace::on() {
        for (i, c) in linked.iter().enumerate() {
            for w in c.windows(2) {
                if (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]) > 10. {
                    trace::line(|| {
                        format!("linked {i} n={} long {:?} -> {:?}", c.len(), w[0], w[1])
                    });
                }
            }
        }
    }
    report.stay_down_links = stay_down;
    report.retract_links = linked.len().saturating_sub(1);
    Ok((linked, report))
}

/// Tool-centre mode (#119; Cimatron's "tool center", ModuleWorks'
/// geodesic tool-centre paths): passes a stepover apart along the surface
/// the ball's centre moves on, instead of the part's surface. That surface
/// is the certified drop's ball-centre height over the plane: continuous
/// for a ball (Lipschitz in XY), smooth where the contact map jumps (an
/// internal corner, a wall top), so walls take part as the ball meets
/// them. It is sampled a quarter stepover apart wherever the ball rests on
/// the drive surface, triangulated, and planned on exactly as contact mode
/// plans on the part (exact distance, certified coverage, fills, links);
/// the passes are the cutter path itself. Equal centre spacing is not equal
/// contact spacing (it differs by (rho +- R) / rho with the curvature): both
/// are measured and reported.
#[allow(clippy::too_many_arguments)]
fn centre_mode(
    drive: &Drive,
    mesh: &SurfaceMesh,
    surface: &PreparedSurface,
    stepover: f64,
    r: f64,
    tool_radius: f64,
    path_error: f64,
    seed: &GeodesicSeed,
    work: &mut SurfaceWork,
    cancel: &impl CancelSignal,
) -> Result<Planned, String> {
    let h = stepover / 4.;
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for p in &drive.v {
        for k in 0..2 {
            lo[k] = lo[k].min(p[k] - r);
            hi[k] = hi[k].max(p[k] + r);
        }
    }
    let (nx, ny) = (
        ((hi[0] - lo[0]) / h).ceil() as usize + 1,
        ((hi[1] - lo[1]) / h).ceil() as usize + 1,
    );
    let workers = std::thread::available_parallelism().map_or(1, usize::from);
    // The drive surface's boundary vertices.
    let topo = &drive.topo;
    let mut rim_vertex = vec![false; drive.v.len()];
    for e in 0..topo.edges.len() {
        if topo.faces[e][1] == NONE {
            for k in topo.edges[e] {
                rim_vertex[k as usize] = true;
            }
        }
    }
    // Whether contact point `q` of face `f` lies on the drive surface's
    // boundary.
    let on_rim = |f: u32, q: Vec3| -> bool {
        let tri = drive.t[f as usize];
        (0..3).any(|k| {
            let (a, b) = (drive.v[tri[k] as usize], drive.v[tri[(k + 1) % 3] as usize]);
            let d = sub(b, a);
            let t = (dot(sub(q, a), d) / dot(d, d).max(1e-300)).clamp(0., 1.);
            let near = norm(sub(axpy(a, t, d), q)) <= 1e-9 * (1. + norm(d));
            near && (topo.faces[topo.tri_edges[f as usize][k] as usize][1] == NONE
                || (t <= 0. && rim_vertex[tri[k] as usize])
                || (t >= 1. && rim_vertex[tri[(k + 1) % 3] as usize]))
        })
    };
    // The ball-centre point over `xy`, where the resting ball touches the
    // drive surface as contact mode lets it: anywhere but its boundary
    // (rolling over the edges between its faces), and on the boundary only
    // square onto the boundary face (never rolling off the surface, as
    // contact mode's passes never do).
    let centre_at = |xy: [f64; 2],
                     w: &mut SurfaceWork,
                     stop: &dyn Fn() -> bool|
     -> Result<Option<Vec3>, String> {
        let c = surface.contact(xy, r, w, &stop)?;
        let centre = [xy[0], xy[1], c.center_z];
        let on = !c.on_base()
            && drive.touching(centre, r, TOUCH).iter().any(|&(f, q)| {
                !on_rim(f, q)
                    || dot(scale(sub(centre, q), 1. / r), drive.n[f as usize]) >= 1. - 1e-9
            });
        Ok(on.then_some(centre))
    };
    // Each grid row's ball-centre points.
    let rows = crate::roughing::parallel::ordered(
        ny,
        workers,
        cancel,
        |j, signal| {
            let stop = || signal.is_cancelled();
            let mut w = SurfaceWork::default();
            let mut row = Vec::with_capacity(nx);
            for i in 0..nx {
                row.push(centre_at(
                    [lo[0] + i as f64 * h, lo[1] + j as f64 * h],
                    &mut w,
                    &stop,
                )?);
            }
            Ok((row, w))
        },
        |_| Ok(()),
    )?;
    let mut grid = Vec::with_capacity(ny);
    for (row, w) in rows {
        work.nodes += w.nodes;
        work.contact_tests += w.contact_tests;
        work.distance_tests += w.distance_tests;
        grid.push(row);
    }
    // Each cell's part of the region (marching squares): its corners in
    // it, and on each side running out of it the exact point where it ends
    // (bisection to a tenth of the path error), fanned into triangles. The
    // region's edge, and the passes along it, follow the boundary, not the
    // grid.
    let cells = crate::roughing::parallel::ordered(
        ny.saturating_sub(1),
        workers,
        cancel,
        |j, signal| {
            let stop = || signal.is_cancelled();
            let mut w = SurfaceWork::default();
            let mut tris = vec![];
            for i in 0..nx - 1 {
                let ring = [(i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1)];
                let pts = ring.map(|(x, y)| grid[y][x]);
                if pts.iter().all(Option::is_none) {
                    continue;
                }
                if let [Some(a), Some(b), Some(c), Some(d)] = pts {
                    // Split along the shorter diagonal (in space).
                    if norm(sub(c, a)) <= norm(sub(d, b)) {
                        tris.extend([[a, b, c], [a, c, d]]);
                    } else {
                        tris.extend([[a, b, d], [b, c, d]]);
                    }
                    continue;
                }
                let mut poly = vec![];
                for k in 0..4 {
                    let (p, q) = (pts[k], pts[(k + 1) % 4]);
                    let xy = |(x, y): (usize, usize)| [lo[0] + x as f64 * h, lo[1] + y as f64 * h];
                    let (pa, pb) = (xy(ring[k]), xy(ring[(k + 1) % 4]));
                    if let Some(p) = p {
                        poly.push(p);
                    }
                    if p.is_some() != q.is_some() {
                        // From the corner in the region towards the other.
                        let (inside, outside, mut best) = if let Some(p) = p {
                            (pa, pb, p)
                        } else {
                            (pb, pa, q.unwrap())
                        };
                        let (mut t0, mut t1) = (0., 1.);
                        while (t1 - t0) * h > 0.1 * path_error {
                            let t = (t0 + t1) / 2.;
                            let at = [
                                inside[0] + t * (outside[0] - inside[0]),
                                inside[1] + t * (outside[1] - inside[1]),
                            ];
                            match centre_at(at, &mut w, &stop)? {
                                Some(c) => (t0, best) = (t, c),
                                None => t1 = t,
                            }
                        }
                        if poly.last().is_none_or(|&l| norm(sub(best, l)) > 1e-12) {
                            poly.push(best);
                        }
                    }
                }
                for k in 1..poly.len().saturating_sub(1) {
                    if norm(cross(sub(poly[k], poly[0]), sub(poly[k + 1], poly[0]))) > 1e-12 {
                        tris.push([poly[0], poly[k], poly[k + 1]]);
                    }
                }
            }
            Ok((tris, w))
        },
        |_| Ok(()),
    )?;
    let mut tris = vec![];
    for (t, w) in cells {
        work.nodes += w.nodes;
        work.contact_tests += w.contact_tests;
        work.distance_tests += w.distance_tests;
        tris.extend(t);
    }
    if tris.is_empty() {
        return Err(
            "geodesic tool-centre mode: the ball rests on the drive surface nowhere".into(),
        );
    }
    let centres = SurfaceMesh {
        version: 1,
        units: "mm".into(),
        triangles: std::sync::Arc::new(tris),
        surface_error_mm: mesh.surface_error_mm,
        source: serde_json::json!({}),
    };
    let cdrive = Drive::build(&centres, cancel)?;
    trace::line(|| {
        format!(
            "tool-centre surface: {nx} x {ny} grid, {} triangles sampled, {} in the drive",
            centres.triangles.len(),
            cdrive.t.len()
        )
    });
    let Plan {
        name,
        passes: planned,
        levels,
        caps,
        coverage,
        ..
    } = plan(&cdrive, seed, stepover, path_error, cancel)?;
    // The passes are the cutter path (XY of the ball centre); verified like
    // contact mode's, along the path at most a quarter stepover apart.
    let mut fits: Vec<Fitted> = vec![];
    let mut cuts = vec![];
    let mut centre = vec![];
    let mut rims = vec![];
    for (k, cap, level, path) in &planned {
        let xy: Vec<([f64; 2], Fit)> = path
            .iter()
            .map(|(p, _)| ([p[0], p[1]], Fit::Placed))
            .collect();
        let mut dense = Vec::with_capacity(xy.len());
        for (i, &(q, f)) in xy.iter().enumerate() {
            if i > 0 {
                let p = xy[i - 1].0;
                let n = ((q[0] - p[0]).hypot(q[1] - p[1]) / (stepover / 4.)).ceil() as usize;
                for m in 1..n {
                    let t = m as f64 / n as f64;
                    dense.push((
                        [p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])],
                        Fit::Placed,
                    ));
                }
            }
            dense.push((q, f));
        }
        fits.push((*k, *cap, dense));
        if *level == f64::NEG_INFINITY || (name == "boundary" && *k == 0 && !*cap) {
            rims.push(xy.iter().map(|p| p.0).collect());
        }
        if *level == f64::INFINITY {
            centre.push(xy.iter().map(|p| p.0).collect());
        } else {
            cuts.push((*k, *cap, xy));
        }
    }
    // The contact sets are ordered inner to outer by the seed's distance
    // on the part.
    let (_, sources, ..) = seeds(drive, seed)?;
    let order = Field::compute(
        &drive.v,
        &drive.t,
        &drive.topo,
        &sources,
        f64::INFINITY,
        cancel,
    )?;
    let checked = Checked {
        drive,
        planned: &order,
        surface,
        stepover,
        contact_radius: r,
        tool_radius,
    };
    let mut report = checked.verify(&fits, name, levels, caps, coverage, work, cancel)?;
    report.mode = "tool_centre";
    report.drive_triangles = cdrive.t.len();
    report.centre_spacing_mm = Some(centre_spacing(&cdrive, &planned, stepover, cancel)?);
    let (linked, stay_down) = link(
        drive, surface, r, stepover, &cuts, centre, &rims, work, cancel,
    )?;
    report.stay_down_links = stay_down;
    report.retract_links = linked.len().saturating_sub(1);
    Ok((linked, report))
}

/// The spacing between neighbouring level passes along the surface they
/// lie on (independent of the planned field): the exact distance from each
/// level's passes, at the next level's points. [min, mean, max] mm.
fn centre_spacing(
    drive: &Drive,
    passes: &[(usize, bool, f64, ContactPath)],
    stepover: f64,
    cancel: &impl CancelSignal,
) -> Result<[f64; 3], String> {
    let top = passes
        .iter()
        .filter(|p| !p.1)
        .map(|p| p.0)
        .max()
        .unwrap_or(0);
    let (mut lo, mut sum, mut hi, mut n) = (f64::INFINITY, 0., 0f64, 0usize);
    for level in 1..=top {
        let mut sources = vec![];
        for (_, _, _, path) in passes.iter().filter(|p| !p.1 && p.0 == level - 1) {
            for w in path.windows(2) {
                if let Some((f, _)) = drive.locate(w[0].0, 1e-6) {
                    walk(
                        &drive.v,
                        &drive.t,
                        &drive.topo,
                        f,
                        w[0].0,
                        [w[1].0[0], w[1].0[1]],
                        &mut sources,
                    );
                }
            }
        }
        if sources.is_empty() {
            continue;
        }
        let field = Field::compute(
            &drive.v,
            &drive.t,
            &drive.topo,
            &sources,
            2. * stepover,
            cancel,
        )?;
        for (_, _, _, path) in passes.iter().filter(|p| !p.1 && p.0 == level) {
            for &(q, _) in path {
                if let Some((f, _)) = drive.locate(q, 1e-6) {
                    let d = field.at(f, q);
                    if d.is_finite() && d <= 2. * stepover {
                        (lo, hi, sum, n) = (lo.min(d), hi.max(d), sum + d, n + 1);
                    }
                }
            }
        }
    }
    Ok(if n == 0 {
        [0.; 3]
    } else {
        [lo, sum / n as f64, hi]
    })
}

/// The solved passes as cut, linked on the surface wherever that is
/// proven. A link stays down when it is at most two stepovers long and the
/// ball, dropped along it (at most a quarter stepover apart), touches the
/// drive surface everywhere (so it never crosses a hole, a wall or stock
/// outside the surface being finished); the finishing plan lifts and
/// certifies it like every chord. A fill pass is spliced into the level
/// pass nearest it (from that pass's nearest point, along the fill and
/// back) when both links stay down; consecutive passes are joined the same
/// way; elsewhere passes stay apart (retract, travel, plunge). Returns the
/// passes and the number of links kept down.
/// The closed loop a path ends with (its last point repeated earlier, the
/// loop from there to the end), if the path's last piece is a closed pass.
fn closed_tail(path: &[[f64; 2]]) -> Option<&[[f64; 2]]> {
    let e = path[path.len() - 1];
    let i = (0..path.len().saturating_sub(3))
        .rev()
        .find(|&i| (path[i][0] - e[0]).hypot(path[i][1] - e[1]) < 1e-9)?;
    Some(&path[i..])
}

#[allow(clippy::too_many_arguments)]
fn link(
    drive: &Drive,
    surface: &PreparedSurface,
    r: f64,
    stepover: f64,
    fits: &[Fitted],
    centre: Vec<Vec<[f64; 2]>>,
    rims: &[Vec<[f64; 2]>],
    work: &mut SurfaceWork,
    cancel: &impl CancelSignal,
) -> Result<Linked, String> {
    // A link stays down where the resting ball touches the drive surface
    // (exact closest points within `TOUCH`, as verification decides) at
    // every sample (a quarter stepover apart) along it; links longer than
    // two stepovers retract.
    let stays_on =
        |path: &[[f64; 2]], limit: bool, work: &mut SurfaceWork| -> Result<bool, String> {
            let (a, b) = (path[0], path[path.len() - 1]);
            if limit && (b[0] - a[0]).hypot(b[1] - a[1]) > 2. * stepover {
                return Ok(false);
            }
            for w in path.windows(2) {
                let len = (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]);
                let n = (len / (stepover / 4.)).ceil().max(1.) as usize;
                for i in 0..=n {
                    let t = i as f64 / n as f64;
                    let xy = [
                        w[0][0] + t * (w[1][0] - w[0][0]),
                        w[0][1] + t * (w[1][1] - w[0][1]),
                    ];
                    let c = surface.contact(xy, r, work, cancel)?;
                    if c.on_base()
                        || drive
                            .touching([xy[0], xy[1], c.center_z], r, TOUCH)
                            .is_empty()
                    {
                        return Ok(false);
                    }
                }
            }
            Ok(true)
        };
    let stays = |path: &[[f64; 2]], work: &mut SurfaceWork| stays_on(path, true, work);
    let down_link = |a: [f64; 2], b: [f64; 2], work: &mut SurfaceWork| stays(&[a, b], work);
    // Points on a boundary pass (within an eighth of a stepover): its rim,
    // and the distance along it from its start.
    let lengths: Vec<Vec<f64>> = rims
        .iter()
        .map(|rim| {
            let mut acc = vec![0.];
            for w in rim.windows(2) {
                acc.push(acc[acc.len() - 1] + (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]));
            }
            acc
        })
        .collect();
    let on_rim = |q: [f64; 2]| -> Option<(usize, f64)> {
        let mut best: Option<(f64, usize, f64)> = None;
        for (ri, rim) in rims.iter().enumerate() {
            for (j, w) in rim.windows(2).enumerate() {
                let (u, v) = (
                    [w[1][0] - w[0][0], w[1][1] - w[0][1]],
                    [q[0] - w[0][0], q[1] - w[0][1]],
                );
                let uu = u[0] * u[0] + u[1] * u[1];
                let t = if uu > 0. {
                    ((u[0] * v[0] + u[1] * v[1]) / uu).clamp(0., 1.)
                } else {
                    0.
                };
                let d = (v[0] - t * u[0]).hypot(v[1] - t * u[1]);
                if best.is_none_or(|b| d < b.0) {
                    best = Some((d, ri, lengths[ri][j] + t * uu.sqrt()));
                }
            }
        }
        best.filter(|b| b.0 <= stepover / 8.).map(|b| (b.1, b.2))
    };
    // The way along rim `ri` from `s0` to `s1` (the shorter way round a
    // closed rim): its points.
    let along = |ri: usize, s0: f64, s1: f64| -> (f64, Vec<[f64; 2]>) {
        let (rim, acc) = (&rims[ri], &lengths[ri]);
        let total = acc[acc.len() - 1];
        let closed = rim.len() > 2
            && (rim[0][0] - rim[rim.len() - 1][0]).hypot(rim[0][1] - rim[rim.len() - 1][1]) < 1e-9;
        let at = |x: f64| -> [f64; 2] {
            let j = acc.partition_point(|&a| a <= x).clamp(1, acc.len() - 1);
            let t = ((x - acc[j - 1]) / (acc[j] - acc[j - 1]).max(1e-300)).clamp(0., 1.);
            [
                rim[j - 1][0] + t * (rim[j][0] - rim[j - 1][0]),
                rim[j - 1][1] + t * (rim[j][1] - rim[j - 1][1]),
            ]
        };
        let forward = if s1 >= s0 {
            s1 - s0
        } else if closed {
            total - s0 + s1
        } else {
            f64::INFINITY
        };
        let backward = if s1 <= s0 {
            s0 - s1
        } else if closed {
            s0 + total - s1
        } else {
            f64::INFINITY
        };
        let (len, dir) = if forward <= backward {
            (forward, 1.)
        } else {
            (backward, -1.)
        };
        let n = (len / (stepover / 8.)).ceil().max(1.) as usize;
        let pts = (0..=n)
            .map(|i| {
                let mut x = s0 + dir * len * i as f64 / n as f64;
                if x < 0. {
                    x += total;
                }
                if x > total {
                    x -= total;
                }
                at(x)
            })
            .collect();
        (len, pts)
    };
    // The direction a path leaves its end (or enters its start, reversed):
    // from the point a quarter stepover back.
    let heading = |path: &[[f64; 2]]| -> Option<[f64; 2]> {
        let e = path[path.len() - 1];
        let f = path
            .iter()
            .rev()
            .find(|p| (e[0] - p[0]).hypot(e[1] - p[1]) >= stepover / 4.)?;
        let (x, y) = (e[0] - f[0], e[1] - f[1]);
        let l = x.hypot(y);
        Some([x / l, y / l])
    };
    // A lane change from the end of one pass into the start of the next
    // running the same way:
    // the cubic Hermite curve leaving along the first's heading and joining
    // along the second's (tangent-continuous: no corner for the machine to
    // stop at), its chords an eighth of a stepover.
    let lane_change = |from: &[[f64; 2]], to: &[[f64; 2]]| -> Option<Vec<[f64; 2]>> {
        let ta = heading(from)?;
        let rev: Vec<[f64; 2]> = to.iter().rev().copied().collect();
        let tb = heading(&rev).map(|t| [-t[0], -t[1]])?;
        let (a, b) = (from[from.len() - 1], to[0]);
        let m = (b[0] - a[0]).hypot(b[1] - a[1]);
        // A lane change only: both headings, and the chord, forward (a
        // reversal is no lane change: its loop would swing off the part).
        let ahead = |t: [f64; 2]| t[0] * (b[0] - a[0]) + t[1] * (b[1] - a[1]) > 0.;
        if ta[0] * tb[0] + ta[1] * tb[1] <= 0. || !ahead(ta) || !ahead(tb) {
            return None;
        }
        let n = (m / (stepover / 8.)).ceil().max(4.) as usize;
        Some(
            (0..=n)
                .map(|i| {
                    let t = i as f64 / n as f64;
                    let (h00, h10) = (2. * t.powi(3) - 3. * t * t + 1., t.powi(3) - 2. * t * t + t);
                    let (h01, h11) = (-2. * t.powi(3) + 3. * t * t, t.powi(3) - t * t);
                    [0, 1].map(|k| h00 * a[k] + h10 * m * ta[k] + h01 * b[k] + h11 * m * tb[k])
                })
                .collect(),
        )
    };
    let xy = |f: &Vec<([f64; 2], Fit)>| -> Vec<[f64; 2]> { f.iter().map(|(p, _)| *p).collect() };
    let d2 = |a: [f64; 2], b: [f64; 2]| (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2);
    // Level passes (with their band); fills to splice.
    let mut rings: Vec<(usize, Vec<[f64; 2]>)> = vec![];
    let mut fills: Vec<Vec<[f64; 2]>> = vec![];
    for (k, cap, f) in fits {
        if f.len() < 2 {
            continue;
        }
        if *cap {
            fills.push(xy(f))
        } else {
            rings.push((*k, xy(f)))
        }
    }
    let mut down = 0;
    // Excursions per ring point: (ring, point index) -> fills, in order.
    let mut excursions: HashMap<(usize, usize), Vec<Vec<[f64; 2]>>> = HashMap::new();
    // Centre passes are cut through, each a pass of its own.
    let mut loose: Vec<Vec<[f64; 2]>> = centre;
    for mut fill in fills {
        let (s0, s1) = (fill[0], fill[fill.len() - 1]);
        let best = rings
            .iter()
            .enumerate()
            .flat_map(|(i, ring)| ring.1.iter().enumerate().map(move |(j, &p)| (i, j, p)))
            .map(|(i, j, p)| {
                let (a, b) = (d2(p, s0), d2(p, s1));
                (a.min(b), i, j, b < a)
            })
            .min_by(|a, b| a.0.total_cmp(&b.0));
        let Some((_, i, j, flip)) = best else {
            loose.push(fill);
            continue;
        };
        if flip {
            fill.reverse();
        }
        let p = rings[i].1[j];
        if down_link(p, fill[0], work)? && down_link(fill[fill.len() - 1], p, work)? {
            excursions.entry((i, j)).or_default().push(fill);
            down += 2;
        } else {
            loose.push(fill);
        }
    }
    let mut pool: Vec<(usize, Vec<[f64; 2]>)> = vec![];
    for (i, (k, ring)) in rings.iter().enumerate() {
        let mut path = vec![];
        for (j, &p) in ring.iter().enumerate() {
            path.push(p);
            if let Some(list) = excursions.get(&(i, j)) {
                for fill in list {
                    path.extend(fill.iter().copied());
                    path.push(p);
                }
            }
        }
        pool.push((*k, path));
    }
    pool.extend(loose.into_iter().map(|p| (usize::MAX, p)));
    let is_closed = |p: &[[f64; 2]]| p.len() > 2 && d2(p[0], p[p.len() - 1]) < 1e-18;
    // Where a pass would be entered from `at`, and how far: a closed pass
    // anywhere, an open one at either end.
    let entry = |p: &[[f64; 2]], at: [f64; 2]| -> f64 {
        if is_closed(p) {
            p.iter()
                .map(|&q| d2(q, at))
                .fold(f64::INFINITY, f64::min)
                .sqrt()
        } else {
            d2(p[0], at).min(d2(p[p.len() - 1], at)).sqrt()
        }
    };
    // A pass oriented for entry from `at` (heading `head`): a closed pass
    // counter-clockwise seen from above (one cutting direction
    // throughout), starting twice as far past its nearest point to `at`
    // (along that direction) as that point is from `at`, so the link to it
    // is a lane change at a 1:2 slope rather than a step sideways; an open
    // pass from its nearer end.
    let orient =
        |mut p: Vec<[f64; 2]>, at: Option<[f64; 2]>, head: Option<[f64; 2]>| -> Vec<[f64; 2]> {
            if is_closed(&p) {
                p.pop();
                let area: f64 = (0..p.len())
                    .map(|i| {
                        let (a, b) = (p[i], p[(i + 1) % p.len()]);
                        a[0] * b[1] - b[0] * a[1]
                    })
                    .sum();
                if area < 0. {
                    p.reverse();
                }
                // The ring started at point `q` of its side `j`.
                let start_at = |p: &[[f64; 2]], j: usize, q: [f64; 2]| -> Vec<[f64; 2]> {
                    let n = p.len();
                    let mut out = vec![q];
                    out.extend((1..=n).map(|k| p[(j + k) % n]));
                    if d2(out[out.len() - 1], q) > 1e-18 {
                        out.push(q);
                    }
                    out.retain({
                        let mut prev: Option<[f64; 2]> = None;
                        move |x| {
                            let keep = prev.is_none_or(|y| d2(*x, y) > 1e-18);
                            prev = Some(*x);
                            keep
                        }
                    });
                    out.pop();
                    out
                };
                if let Some(at) = at {
                    let n = p.len();
                    // The exact nearest point (on a side, not only a vertex).
                    let (mut bj, mut bq, mut bd) = (0, p[0], f64::INFINITY);
                    for j in 0..n {
                        let (u, v) = (p[j], p[(j + 1) % n]);
                        let w = [v[0] - u[0], v[1] - u[1]];
                        let ww = w[0] * w[0] + w[1] * w[1];
                        let t = if ww > 0. {
                            (((at[0] - u[0]) * w[0] + (at[1] - u[1]) * w[1]) / ww).clamp(0., 1.)
                        } else {
                            0.
                        };
                        let q = [u[0] + t * w[0], u[1] + t * w[1]];
                        if d2(q, at) < bd {
                            (bj, bq, bd) = (j, q, d2(q, at));
                        }
                    }
                    p = start_at(&p, bj, bq);
                    // For a lane change: twice as far on along the ring as the
                    // nearest point is from `at` (a 1:2 slope), exactly.
                    if let Some(h) = head {
                        let ahead = [p[1][0] - p[0][0], p[1][1] - p[0][1]];
                        if h[0] * ahead[0] + h[1] * ahead[1] > 0. {
                            let mut left = 2. * bd.sqrt();
                            let m = p.len();
                            for j in 0..m {
                                let (u, v) = (p[j], p[(j + 1) % m]);
                                let l = d2(u, v).sqrt();
                                if l >= left {
                                    let t = left / l.max(1e-300);
                                    let q = [u[0] + t * (v[0] - u[0]), u[1] + t * (v[1] - u[1])];
                                    p = start_at(&p, j, q);
                                    break;
                                }
                                left -= l;
                            }
                        }
                    }
                }
                p.push(p[0]);
            } else if let Some(at) = at
                && d2(p[p.len() - 1], at) < d2(p[0], at)
            {
                p.reverse();
            }
            p
        };
    // The link from the end of `from` into pass `p` that stays down: a
    // lane change (into a closed pass two gaps past its nearest point),
    // else straight (into its nearest point), else none. Returns the
    // link's interior points and the pass as entered.
    #[allow(clippy::type_complexity)]
    let join = |from: &[[f64; 2]],
                p: &[[f64; 2]],
                work: &mut SurfaceWork|
     -> Result<Option<(Vec<[f64; 2]>, Vec<[f64; 2]>)>, String> {
        // Only from within two stepovers of the pass (candidates are chosen
        // by their distance from anywhere on the ring just cut, for a
        // re-trace; a direct link must be short itself).
        let at = from[from.len() - 1];
        let near = orient(p.to_vec(), Some(at), None);
        if d2(at, near[0]).sqrt() > 2. * stepover {
            return Ok(None);
        }
        let lane = orient(p.to_vec(), Some(at), heading(from));
        if let Some(curve) = lane_change(from, &lane)
            && stays_on(&curve, false, work)?
        {
            trace::line(|| format!("link: lane change {at:.3?} -> {:.3?}", lane[0]));
            return Ok(Some((curve[1..curve.len() - 1].to_vec(), lane)));
        }
        let down = down_link(at, near[0], work)?;
        if down {
            trace::line(|| format!("link: straight {at:.3?} -> {:.3?}", near[0]));
        }
        Ok(down.then(|| (Vec::new(), near)))
    };
    // Cutting order, from the seed's first pass: the nearest remaining
    // pass that can be joined without lifting (directly, or after running
    // on along the closed pass just cut, over its own path, to the point
    // nearest it); only where none can, the nearest, after a retract.
    let mut out: Vec<Vec<[f64; 2]>> = vec![];
    let Some(first) = (0..pool.len()).min_by_key(|&i| pool[i].0) else {
        return Ok((out, down));
    };
    out.push(orient(pool.remove(first).1, None, None));
    let mut trimmed = false;
    while !pool.is_empty() {
        // Once only fill and centre passes remain (every level pass is cut,
        // with its links): each is cut only where nothing already cut runs
        // within a tenth of a stepover of it. A link along a ridge (corner
        // to corner of rings from a boundary) has cut that stretch already;
        // cutting it again would only rub the finished surface.
        if !trimmed && pool.iter().all(|p| p.0 == usize::MAX) {
            trimmed = true;
            let pieces = uncut(&out, &pool, stepover);
            trace::line(|| {
                format!(
                    "fills: {} trimmed to {} pieces not already cut",
                    pool.len(),
                    pieces.len()
                )
            });
            pool = pieces.into_iter().map(|p| (usize::MAX, p)).collect();
            if pool.is_empty() {
                break;
            }
        }
        let last = out.last().unwrap().clone();
        let at = last[last.len() - 1];
        let tail: Option<Vec<[f64; 2]>> = closed_tail(&last).map(<[_]>::to_vec);
        let reach = |p: &[[f64; 2]]| match &tail {
            Some(t) => t.iter().map(|&q| entry(p, q)).fold(f64::INFINITY, f64::min),
            None => entry(p, at),
        };
        // Level passes before centre and fill passes (cut after the passes
        // they complete); each kind nearest first.
        let mut cands: Vec<(f64, usize)> = pool
            .iter()
            .enumerate()
            .map(|(i, p)| (reach(&p.1), i))
            .collect();
        let extra = |i: usize| pool[i].0 == usize::MAX;
        cands.sort_by(|a, b| {
            extra(a.1)
                .cmp(&extra(b.1))
                .then(a.0.total_cmp(&b.0))
                .then(a.1.cmp(&b.1))
        });
        // Along the part's edge: from a pass ending on a boundary pass, to
        // the open pass whose end is nearest along it (re-cutting the edge
        // between them).
        let mut rim_best: Option<(f64, usize, bool, Vec<[f64; 2]>)> = None;
        if let Some((ri, s0)) = on_rim(at) {
            for (i, (_, p)) in pool.iter().enumerate() {
                if is_closed(p) {
                    continue;
                }
                for (end, rev) in [(p[0], false), (p[p.len() - 1], true)] {
                    if let Some((rj, s1)) = on_rim(end)
                        && rj == ri
                    {
                        // Level passes first, as for every link.
                        let (len, pts) = along(ri, s0, s1);
                        let key = |j: usize, l: f64| (pool[j].0 == usize::MAX, l);
                        if rim_best
                            .as_ref()
                            .is_none_or(|b| key(i, len) < key(b.1, b.0))
                        {
                            rim_best = Some((len, i, rev, pts));
                        }
                    }
                }
            }
        }
        // The edge way's length (to a level pass: re-tracing is compared
        // with it).
        let rim_len = rim_best
            .as_ref()
            .filter(|b| pool[b.1].0 != usize::MAX)
            .map_or(f64::INFINITY, |b| b.0);
        let mut done = false;
        for &(d, i) in &cands {
            if d > 2. * stepover {
                continue;
            }
            if let Some((link, to)) = join(&last, &pool[i].1, work)? {
                let chain = out.last_mut().unwrap();
                chain.extend(link);
                chain.extend(to);
                pool.remove(i);
                down += 1;
                done = true;
                break;
            }
            if let Some(t) = &tail {
                let j = (1..t.len())
                    .min_by(|&a, &b| entry(&pool[i].1, t[a]).total_cmp(&entry(&pool[i].1, t[b])))
                    .unwrap();
                // Re-tracing farther than the way along the edge: take that.
                let retrace: f64 = t[..=j]
                    .windows(2)
                    .map(|w| (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]))
                    .sum();
                if retrace > rim_len {
                    continue;
                }
                let mut from = last.clone();
                from.extend_from_slice(&t[1..=j]);
                if let Some((link, to)) = join(&from, &pool[i].1, work)? {
                    let chain = out.last_mut().unwrap();
                    chain.extend_from_slice(&t[1..=j]);
                    chain.extend(link);
                    chain.extend(to);
                    pool.remove(i);
                    down += 1;
                    done = true;
                    break;
                }
            }
        }
        if !done && let Some((_, i, rev, pts)) = rim_best {
            let mut to = pool[i].1.clone();
            if rev {
                to.reverse();
            }
            let mut path = vec![at];
            path.extend(pts);
            path.push(to[0]);
            if stays_on(&path, false, work)? {
                let chain = out.last_mut().unwrap();
                chain.extend_from_slice(&path[1..path.len() - 1]);
                chain.extend(to);
                pool.remove(i);
                down += 1;
                done = true;
            }
        }
        if !done {
            let i = cands[0].1;
            let next = orient(pool.remove(i).1, Some(at), None);
            trace::line(|| format!("link: retract {at:.3?} -> {:.3?}", next[0]));
            out.push(next);
        }
    }
    // No move shorter than the placement tolerance (a corner arc's last
    // position against the next pass's first, say): a block that short is
    // noise to the controller. Each path keeps its ends.
    let tiny = stepover * 1e-3;
    for path in &mut out {
        let n = path.len();
        let mut kept: Vec<[f64; 2]> = Vec::with_capacity(n);
        for (i, &p) in path.iter().enumerate() {
            match kept.last() {
                Some(&q) if (p[0] - q[0]).hypot(p[1] - q[1]) < tiny => {
                    if i + 1 == n {
                        *kept.last_mut().unwrap() = p;
                    }
                }
                _ => kept.push(p),
            }
        }
        *path = kept;
    }
    Ok((out, down))
}

/// The parts of `extras` not already cut: each sampled an eighth of a
/// stepover apart, and kept in runs of samples farther than a tenth of a
/// stepover from every path in `done` (runs shorter than a quarter
/// stepover are dropped: the cut around them already covers them).
fn uncut(
    done: &[Vec<[f64; 2]>],
    extras: &[(usize, Vec<[f64; 2]>)],
    stepover: f64,
) -> Vec<Vec<[f64; 2]>> {
    let near = stepover / 10.;
    let cell = stepover;
    let key = |p: [f64; 2]| ((p[0] / cell).floor() as i64, (p[1] / cell).floor() as i64);
    let mut grid: SegmentGrid = HashMap::new();
    for path in done {
        for w in path.windows(2) {
            let (a, b) = (
                key([w[0][0].min(w[1][0]), w[0][1].min(w[1][1])]),
                key([w[0][0].max(w[1][0]), w[0][1].max(w[1][1])]),
            );
            for x in a.0..=b.0 {
                for y in a.1..=b.1 {
                    grid.entry((x, y)).or_default().push((w[0], w[1]));
                }
            }
        }
    }
    let covered = |p: [f64; 2]| {
        let k = key(p);
        (k.0 - 1..=k.0 + 1).any(|x| {
            (k.1 - 1..=k.1 + 1).any(|y| {
                grid.get(&(x, y)).is_some_and(|segs| {
                    segs.iter().any(|&(a, b)| {
                        let (u, v) = ([b[0] - a[0], b[1] - a[1]], [p[0] - a[0], p[1] - a[1]]);
                        let uu = u[0] * u[0] + u[1] * u[1];
                        let t = if uu > 0. {
                            ((u[0] * v[0] + u[1] * v[1]) / uu).clamp(0., 1.)
                        } else {
                            0.
                        };
                        (v[0] - t * u[0]).hypot(v[1] - t * u[1]) <= near
                    })
                })
            })
        })
    };
    let mut out = vec![];
    for (_, path) in extras {
        // A run follows the path's own vertices, from the first sample not
        // covered to the last (its ends at sample resolution).
        let mut run: Vec<[f64; 2]> = vec![];
        let mut last: Option<[f64; 2]> = None;
        let mut flush = |run: &mut Vec<[f64; 2]>, last: &mut Option<[f64; 2]>| {
            if let (Some(l), Some(&e)) = (*last, run.last())
                && l != e
            {
                run.push(l);
            }
            *last = None;
            let len: f64 = run
                .windows(2)
                .map(|w| (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]))
                .sum();
            if run.len() >= 2 && len >= stepover / 4. {
                out.push(std::mem::take(run));
            } else {
                run.clear();
            }
        };
        for (i, &q) in path.iter().enumerate() {
            let mut samples = vec![];
            if i > 0 {
                let p = path[i - 1];
                let n = ((q[0] - p[0]).hypot(q[1] - p[1]) / (stepover / 8.)).ceil() as usize;
                for j in 1..n {
                    let t = j as f64 / n as f64;
                    samples.push(([p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])], false));
                }
            }
            samples.push((q, true));
            for (x, vertex) in samples {
                if covered(x) {
                    flush(&mut run, &mut last);
                } else {
                    if run.is_empty() || vertex {
                        run.push(x);
                    }
                    last = Some(x);
                }
            }
        }
        flush(&mut run, &mut last);
    }
    out
}

/// A cutter path relaxed towards least curvature, every point within `tol`
/// of where it was solved (its ends fixed; a closed path stays closed).
/// Each solved position is exact for its own facet's normal, so on a
/// faceted surface consecutive positions step sideways by up to R times the
/// angle between facets; within the tolerance the path need not follow
/// those steps. Each round moves every point halfway to the point of its
/// neighbours' chord at its place along it (by length), clamped to its
/// tolerance disc.
fn smooth(path: &[[f64; 2]], tol: f64) -> Vec<[f64; 2]> {
    let n = path.len();
    if n < 3 || tol <= 0. {
        return path.to_vec();
    }
    let closed = (path[0][0] - path[n - 1][0]).hypot(path[0][1] - path[n - 1][1]) < 1e-12;
    let mut q = path.to_vec();
    let m = if closed { n - 1 } else { n };
    for _ in 0..64 {
        let prev = q.clone();
        for i in 0..m {
            let (a, b) = if closed {
                (prev[(i + m - 1) % m], prev[(i + 1) % m])
            } else if i == 0 || i + 1 == n {
                continue;
            } else {
                (prev[i - 1], prev[i + 1])
            };
            let p = prev[i];
            let (la, lb) = (
                (p[0] - a[0]).hypot(p[1] - a[1]),
                (b[0] - p[0]).hypot(b[1] - p[1]),
            );
            if la + lb <= 0. {
                continue;
            }
            let t = la / (la + lb);
            let target = [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
            let mut x = [(p[0] + target[0]) / 2., (p[1] + target[1]) / 2.];
            let (dx, dy) = (x[0] - path[i][0], x[1] - path[i][1]);
            let d = dx.hypot(dy);
            if d > tol {
                x = [path[i][0] + dx * tol / d, path[i][1] + dy * tol / d];
            }
            q[i] = x;
        }
        if closed {
            q[n - 1] = q[0];
        }
    }
    q
}

/// How a pass vertex's cutter position landed.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Fit {
    /// Its achieved contact is on its level (within a tenth of the path
    /// error).
    Placed,
    /// Its achieved contact is this far (along the field) from its level:
    /// where the ball meets a neighbouring facet first, or rests on both
    /// sides of a concave edge or crease.
    Off(f64),
    /// The ball cannot touch the intended contact there (the drop lifts it
    /// above by more than the surface error: a crease narrower than the
    /// ball, a wall top) or it lands off the drive surface.
    Unreachable,
}

/// Places cutter positions in contact mode. The mesh samples a smooth part:
/// a facet's normal estimates the part's normal to first order (its error,
/// about the facet size over the curvature radius, times R moves the cutter
/// sideways at every facet edge: a shaking path), the vertices' normals
/// interpolated across it to second order. The ball centre is the contact
/// plus R times that smooth normal; the certified drop lifts the ball onto
/// the faceted mesh as it is (no gouge), and the achieved contact's
/// distance from its level is measured and reported.
struct Solver<'a> {
    drive: &'a Drive,
    field: &'a Field<'a>,
    surface: &'a PreparedSurface,
    r: f64,
    tol: f64,
    eps: f64,
}

impl Solver<'_> {
    fn solve(
        &self,
        cc: Vec3,
        level: f64,
        w: &mut SurfaceWork,
        stop: &impl CancelSignal,
    ) -> Result<([f64; 2], Fit), String> {
        let Some((f, _)) = self.drive.locate(cc, 1e-6 + self.tol) else {
            return Ok(([cc[0], cc[1]], Fit::Unreachable));
        };
        self.place(cc, self.drive.smooth_normal(f, cc), level, w, stop)
    }

    /// The cutter position touching `cc` with the surface normal `n`
    /// there.
    fn place(
        &self,
        cc: Vec3,
        n: Vec3,
        level: f64,
        w: &mut SurfaceWork,
        stop: &impl CancelSignal,
    ) -> Result<([f64; 2], Fit), String> {
        let r = self.r;
        let xy = [cc[0] + r * n[0], cc[1] + r * n[1]];
        let c = self.surface.contact(xy, r, w, stop)?;
        let d2 = (xy[0] - cc[0]).powi(2) + (xy[1] - cc[1]).powi(2);
        let touching = cc[2] + (r * r - d2).max(0.).sqrt();
        if c.on_base() || c.center_z > touching + self.tol {
            if trace::on() {
                trace::line(|| {
                    format!(
                        "unreach A cc={cc:?} n={n:?} base={} cz={} touching={touching}",
                        c.on_base(),
                        c.center_z
                    )
                });
            }
            return Ok((xy, Fit::Unreachable));
        }
        let nn = c.contact_normal(xy, r);
        let p = [xy[0] - r * nn[0], xy[1] - r * nn[1], c.center_z - r * nn[2]];
        let Some((g, _)) = self.drive.locate(p, self.tol) else {
            if trace::on() {
                trace::line(|| format!("unreach B cc={cc:?} n={n:?} p={p:?} nn={nn:?}"));
            }
            return Ok((xy, Fit::Unreachable));
        };
        let miss = if !level.is_finite() {
            // A boundary or spur pass: the distance to its own point.
            norm(sub(p, cc))
        } else {
            (self.field.at(g, p) - level).abs()
        };
        Ok((
            xy,
            if miss <= self.eps {
                Fit::Placed
            } else {
                Fit::Off(miss)
            },
        ))
    }

    /// The cutter positions at vertex `i` of a pass, each with its
    /// contact. Where the surface normal jumps at the vertex (a convex
    /// crease or corner: the faces before and after it have different
    /// normals there) the ball rolls round the point: its centre sweeps the
    /// arc about it from one normal to the other (the offset curve's round
    /// corner), in steps whose chords stay within a tenth of the path
    /// error of the arc. Elsewhere one position.
    fn vertex(
        &self,
        path: &ContactPath,
        i: usize,
        level: f64,
        w: &mut SurfaceWork,
        stop: &impl CancelSignal,
    ) -> Result<Vec<Placed>, String> {
        let n = path.len();
        let cc = path[i].0;
        let closed = n > 2 && norm(sub(path[0].0, path[n - 1].0)) < 1e-9;
        let prev = if i > 0 {
            Some(i - 1)
        } else if closed {
            Some(n - 2)
        } else {
            None
        };
        let next = if i + 1 < n {
            Some(i + 1)
        } else if closed {
            Some(1)
        } else {
            None
        };
        // A segment's normal at the vertex: the faces it lies on, averaged
        // (one face; both faces of a crease the segment runs along, whose
        // normal cone it may use throughout).
        let along = |j: usize| -> Option<Vec3> {
            let mid = lerp(cc, path[j].0, 0.5);
            let mut on = self.drive.within(mid, 1e-7);
            // Along the drive surface's boundary the surface is that one
            // face: its own normal (a vertex normal there averages only
            // the faces on one side, tilting it off the boundary face).
            if let [f] = on[..] {
                let topo = &self.drive.topo;
                let [x, y, z] = self.drive.t[f as usize].map(|k| self.drive.v[k as usize]);
                let corners = [x, y, z];
                for k in 0..3 {
                    let e = topo.tri_edges[f as usize][k];
                    if topo.faces[e as usize][1] != NONE {
                        continue;
                    }
                    let (p0, p1) = (corners[k], corners[(k + 1) % 3]);
                    let d = sub(p1, p0);
                    let t = (dot(sub(mid, p0), d) / dot(d, d).max(1e-300)).clamp(0., 1.);
                    if norm(sub(axpy(p0, t, d), mid)) <= 1e-7 {
                        return Some(self.drive.n[f as usize]);
                    }
                }
            }
            if on.is_empty() {
                on.extend(self.drive.locate(mid, 1e-6 + self.tol).map(|x| x.0));
            }
            let mut m = [0.; 3];
            for f in on {
                m = axpy(m, 1., self.drive.smooth_normal(f, cc));
            }
            (norm(m) > 1e-12).then(|| scale(m, 1. / norm(m)))
        };
        if let (Some(a), Some(b)) = (prev, next)
            && let (Some(na), Some(nb)) = (along(a), along(b))
        {
            let angle = dot(na, nb).clamp(-1., 1.).acos();
            let step = 2. * (1. - self.eps / self.r).clamp(-1., 1.).acos();
            // Convex: the next contact lies below the incoming face's
            // tangent plane (a concave crease folds instead; trim).
            if angle > step
                && angle < std::f64::consts::PI - 1e-9
                && dot(na, sub(path[b].0, cc)) < 0.
            {
                let k = (angle / step).ceil() as usize;
                trace::line(|| {
                    format!(
                        "corner arc at {cc:.4?}: {:.2} deg in {k} steps",
                        angle.to_degrees()
                    )
                });
                let mut out = Vec::with_capacity(k + 1);
                for j in 0..=k {
                    // The normals' great-circle arc (slerp).
                    let t = j as f64 / k as f64;
                    let (sa, sb) = (((1. - t) * angle).sin(), (t * angle).sin());
                    let m = scale(axpy(scale(na, sa), sb, nb), 1. / angle.sin());
                    out.push((cc, self.place(cc, m, level, w, stop)?));
                }
                return Ok(out);
            }
            if angle <= step {
                // One normal before and after (on one face, or both
                // segments along the same crease: its cone's bisector).
                let m = axpy(na, 1., nb);
                return Ok(vec![(
                    cc,
                    self.place(cc, scale(m, 1. / norm(m)), level, w, stop)?,
                )]);
            }
        }
        Ok(vec![(cc, self.solve(cc, level, w, stop)?)])
    }

    /// The cutter path of one pass without its unreachable vertices. Where
    /// the ball cannot touch the pass (a concave crease narrower than the
    /// ball), offsetting the contacts by R times the normal folds back on
    /// itself (the offset's swallowtail: across the crease the normal turns
    /// and the positions jump back over the crease). Established practice is
    /// to trim the fold: the exact points where the ball stops and starts
    /// reaching the pass are found by bisection along its contact path
    /// (within a tenth of the path error), and joined straight; the
    /// certified drop lifts the chord onto the part (the ball rides the
    /// crease touching both sides). Such a fold spans a crease narrower
    /// than the ball, so its ends are at most the ball's diameter apart; a
    /// longer gap is no fold (the ball cannot reach a stretch of the pass:
    /// a wall top, a lost corner) and the pass is cut there, never joined
    /// straight across. An unreachable end of an open pass is cut off at
    /// its last reachable point. Returns the pieces and the number of
    /// bridged runs.
    fn trim(
        &self,
        path: &ContactPath,
        out: &[([f64; 2], Fit)],
        level: f64,
        w: &mut SurfaceWork,
        stop: &impl CancelSignal,
    ) -> Result<Pieces, String> {
        let n = out.len();
        let reach = |i: usize| out[i].1 != Fit::Unreachable;
        if (0..n).all(reach) {
            return Ok((vec![out.to_vec()], 0));
        }
        let closed = n > 2 && norm(sub(path[0].0, path[n - 1].0)) < 1e-9;
        // Vertex order: a closed pass from a reachable vertex round to it.
        let order: Vec<usize> = if closed {
            let m = n - 1;
            let Some(s) = (0..m).find(|&i| reach(i)) else {
                return Ok((vec![], 0));
            };
            (0..=m).map(|j| (s + j) % m).collect()
        } else {
            (0..n).collect()
        };
        // The last reachable point from contact a (reachable) towards b.
        let edge = |a: usize, b: usize, w: &mut SurfaceWork| -> Result<([f64; 2], Fit), String> {
            let (pa, pb) = (path[a].0, path[b].0);
            let len = norm(sub(pb, pa));
            let (mut lo, mut hi, mut best) = (0., 1., out[a]);
            while (hi - lo) * len > self.eps {
                let t = (lo + hi) / 2.;
                let q = lerp(pa, pb, t);
                // The chord's point onto the surface.
                let q = match self.drive.locate(q, 1e-6 + self.tol) {
                    Some((f, _)) => {
                        let [x, y, z] = self.drive.t[f as usize].map(|k| self.drive.v[k as usize]);
                        closest_on_triangle(q, x, y, z)
                    }
                    None => q,
                };
                let fit = self.solve(q, level, w, stop)?;
                if fit.1 == Fit::Unreachable {
                    hi = t;
                } else {
                    (lo, best) = (t, fit);
                }
            }
            Ok(best)
        };
        let mut pieces: Vec<Vec<([f64; 2], Fit)>> = vec![vec![]];
        let mut bridged = 0;
        let mut j = 0;
        while j < order.len() {
            let i = order[j];
            if reach(i) {
                pieces.last_mut().unwrap().push(out[i]);
                j += 1;
                continue;
            }
            // An unreachable run order[j..e].
            let mut e = j;
            while e < order.len() && !reach(order[e]) {
                e += 1;
            }
            let before = if j > 0 {
                Some(edge(order[j - 1], i, w)?)
            } else {
                None
            };
            if let Some(b) = before {
                pieces.last_mut().unwrap().push(b);
            }
            if e < order.len() {
                let after = edge(order[e], order[e - 1], w)?;
                match before {
                    Some(b) if (b.0[0] - after.0[0]).hypot(b.0[1] - after.0[1]) <= 2. * self.r => {
                        pieces.last_mut().unwrap().push(after);
                        bridged += 1;
                    }
                    Some(_) => pieces.push(vec![after]),
                    None => pieces.last_mut().unwrap().push(after),
                }
            }
            j = e;
        }
        // A closed pass cut open: its last piece runs on into its first.
        if closed && pieces.len() > 1 {
            let first = pieces.remove(0);
            pieces.last_mut().unwrap().extend(first.into_iter().skip(1));
        }
        pieces.retain(|p| p.len() >= 2);
        Ok((pieces, bridged))
    }
}

struct Checked<'a> {
    drive: &'a Drive,
    /// The planned field: only to order a contact set inner to outer.
    planned: &'a Field<'a>,
    surface: &'a PreparedSurface,
    stepover: f64,
    contact_radius: f64,
    tool_radius: f64,
}

impl Checked<'_> {
    /// Achieved contacts and the intrinsic spacing between neighbouring
    /// passes, independent of the planned field. Each pass vertex's solved
    /// cutter position is dropped afresh with the certified ball contact;
    /// its contact set is every drive-surface point within `TOUCH` of the
    /// resting ball (one point on a facet; two where the ball rests on both
    /// sides of a concave edge). A pass's contacts form two chains, its
    /// inner and outer contacts (the same point where it touches once), as
    /// the polygonal sources of the next level's distance; a vertex's
    /// spacing is the exact distance from its contact set (its nearest
    /// contact) to the previous level's chains. Vertices whose level no
    /// position touches (unreachable) and contacts off the drive surface
    /// are gaps, counted.
    #[allow(clippy::too_many_arguments)]
    fn verify(
        &self,
        fits: &[Fitted],
        seed: &'static str,
        levels: usize,
        caps: usize,
        coverage: f64,
        work: &mut SurfaceWork,
        cancel: &impl CancelSignal,
    ) -> Result<GeodesicReport, String> {
        let (drive, stepover, r) = (self.drive, self.stepover, self.contact_radius);
        let workers = std::thread::available_parallelism().map_or(1, usize::from);
        type Set = Option<Vec<(u32, Vec3)>>;
        let located = crate::roughing::parallel::ordered(
            fits.len(),
            workers,
            cancel,
            |i, signal| {
                let stop = || signal.is_cancelled();
                let mut w = SurfaceWork::default();
                let mut out: Vec<(Set, bool)> = Vec::with_capacity(fits[i].2.len());
                for &(xy, fit) in &fits[i].2 {
                    if fit == Fit::Unreachable {
                        out.push((None, true));
                        continue;
                    }
                    let c = self.surface.contact(xy, r, &mut w, &stop)?;
                    if c.on_base() {
                        out.push((None, false));
                        continue;
                    }
                    let mut set = drive.touching([xy[0], xy[1], c.center_z], r, TOUCH);
                    if set.is_empty() {
                        out.push((None, false));
                        continue;
                    }
                    set.sort_by(|p, q| {
                        self.planned
                            .at(p.0, p.1)
                            .total_cmp(&self.planned.at(q.0, q.1))
                    });
                    out.push((Some(set), false));
                }
                Ok((out, w))
            },
            |_| Ok(()),
        )?;
        let mut contacts: Vec<Vec<Set>> = vec![];
        let (mut unreachable, mut off) = (0usize, 0usize);
        for (c, w) in located {
            work.nodes += w.nodes;
            work.contact_tests += w.contact_tests;
            work.distance_tests += w.distance_tests;
            unreachable += c.iter().filter(|x| x.1).count();
            off += c.iter().filter(|x| !x.1 && x.0.is_none()).count();
            contacts.push(c.into_iter().map(|x| x.0).collect());
        }
        let (mut lo, mut sum, mut hi) = (f64::INFINITY, 0., 0f64);
        let (mut count, mut over, mut unmeasured) = (0usize, 0usize, 0usize);
        let max_level = fits.iter().map(|p| p.0).max().unwrap_or(0);
        let (mut straddled, mut miss) = (0usize, 0f64);
        for (_, _, f) in fits {
            for (_, fit) in f {
                if let Fit::Off(m) = fit {
                    straddled += 1;
                    miss = miss.max(*m);
                }
            }
        }
        let measured = crate::roughing::parallel::ordered(
            max_level,
            workers,
            cancel,
            |i, signal| {
                let level = i + 1;
                // The previous level's contact chains as polygonal sources:
                // each span between consecutive contacts projected onto the
                // surface (split per face), broken at gaps; contacts between
                // the innermost and outermost of a set as points.
                let mut sources: Vec<Piece> = vec![];
                for (_, c) in fits
                    .iter()
                    .zip(&contacts)
                    .filter(|(p, _)| p.0 == level - 1 && !p.1)
                {
                    for side in [0usize, 1] {
                        let mut prev: Option<(u32, Vec3)> = None;
                        for x in c {
                            let here = x.as_ref().map(|set| {
                                if side == 0 {
                                    set[0]
                                } else {
                                    set[set.len() - 1]
                                }
                            });
                            match (prev, here) {
                                (Some((f, a)), Some((_, b))) => {
                                    walk(
                                        &drive.v,
                                        &drive.t,
                                        &drive.topo,
                                        f,
                                        a,
                                        [b[0], b[1]],
                                        &mut sources,
                                    );
                                }
                                (None, Some((f, a))) => sources.push(Piece::Point(f, a)),
                                _ => {}
                            }
                            prev = here;
                        }
                    }
                    for set in c.iter().flatten().filter(|set| set.len() > 2) {
                        for &(f, q) in &set[1..set.len() - 1] {
                            sources.push(Piece::Point(f, q));
                        }
                    }
                }
                let field = Field::compute(
                    &drive.v,
                    &drive.t,
                    &drive.topo,
                    &sources,
                    2. * stepover,
                    signal,
                )?;
                let mut out = vec![];
                for (_, c) in fits
                    .iter()
                    .zip(&contacts)
                    .filter(|(p, _)| p.0 == level && !p.1)
                {
                    for set in c.iter().flatten() {
                        let sv = set
                            .iter()
                            .map(|&(f, q)| field.at(f, q))
                            .fold(f64::INFINITY, f64::min);
                        if trace::on() && sv > 1.003 * stepover && sv <= 2. * stepover {
                            trace::line(|| {
                                format!("wide level {level} spacing {sv:.4} at {:?}", set[0].1)
                            });
                        }
                        if trace::on() && sv < 0.97 * stepover {
                            let q = set[0].1;
                            let near = sources
                                .iter()
                                .map(|pc| match *pc {
                                    Piece::Point(_, a) => (norm(sub(a, q)), a, a),
                                    Piece::Segment(_, a, b) => {
                                        let ab = sub(b, a);
                                        let t = (dot(sub(q, a), ab) / dot(ab, ab).max(1e-300))
                                            .clamp(0., 1.);
                                        (norm(sub(axpy(a, t, ab), q)), a, b)
                                    }
                                })
                                .min_by(|x, y| x.0.total_cmp(&y.0));
                            trace::line(|| {
                                format!(
                                    "narrow level {level} spacing {sv:.4} at {q:?} set {} nearest source {near:?}",
                                    set.len()
                                )
                            });
                        }
                        out.push(sv);
                    }
                }
                Ok(out)
            },
            |_| Ok(()),
        )?;
        for s in measured.into_iter().flatten() {
            if !s.is_finite() || s > 2. * stepover {
                unmeasured += 1;
                continue;
            }
            lo = lo.min(s);
            hi = hi.max(s);
            sum += s;
            count += 1;
            if s > stepover {
                over += 1;
            }
        }
        let spacing = if count == 0 {
            [0.; 3]
        } else {
            [lo, sum / count as f64, hi]
        };
        let tr = self.tool_radius;
        let flat = if hi >= 2. * tr {
            f64::INFINITY
        } else {
            tr - (tr * tr - hi * hi / 4.).sqrt()
        };
        Ok(GeodesicReport {
            seed,
            stepover_mm: stepover,
            drive_triangles: drive.t.len(),
            levels,
            cap_passes: caps,
            planned_coverage_mm: coverage,
            spacing_mm: spacing,
            samples: count,
            over_stepover_samples: over,
            off_drive_samples: off,
            unmeasured_samples: unmeasured,
            unreachable_samples: unreachable,
            bridged_spans: 0,
            off_level_samples: straddled,
            max_level_miss_mm: miss,
            flat_scallop_estimate_mm: flat,
            stay_down_links: 0,
            retract_links: 0,
            mode: "contact",
            centre_spacing_mm: None,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicBool;

    fn peaks_of(drive: &Drive, field: &Field) -> Vec<Vec<(Vec3, f64)>> {
        (0..drive.t.len() as u32)
            .map(|f| field.face_peaks(f))
            .collect()
    }

    pub(super) fn mesh(tris: Vec<[Vec3; 3]>) -> SurfaceMesh {
        SurfaceMesh {
            version: 1,
            units: "mm".into(),
            triangles: std::sync::Arc::new(tris),
            surface_error_mm: 0.,
            source: serde_json::json!({}),
        }
    }

    #[test]
    fn contours_lie_on_the_exact_iso_curves() {
        let never = AtomicBool::new(false);
        // Two coarse triangles of a plane (bisected only to 8 mm): rings
        // from a point are circles, placed exactly whatever the faces.
        let plane = mesh(vec![
            [[0., 0., 1.], [20., 0., 1.], [20., 20., 1.]],
            [[0., 0., 1.], [20., 20., 1.], [0., 20., 1.]],
        ]);
        let drive = Drive::build(&plane, &never).unwrap();
        let (ti, _, p) = drive.top_at([9.3, 10.4]).unwrap();
        let field = Field::compute(
            &drive.v,
            &drive.t,
            &drive.topo,
            &[Piece::Point(ti, p)],
            f64::INFINITY,
            &never,
        )
        .unwrap();
        let faces: Vec<u32> = (0..drive.t.len() as u32).collect();
        let tol = 0.001;
        for r in [0.75, 3., 7.5] {
            let paths = contour(&drive, &field, &peaks_of(&drive, &field), r, &faces, tol);
            assert_eq!(paths.len(), 1, "one closed ring at {r}");
            let ring = &paths[0];
            assert!(norm(sub(ring[0].0, ring[ring.len() - 1].0)) < 1e-12);
            for w in ring.windows(2) {
                assert!((norm(sub(w[0].0, p)) - r).abs() < 1e-9);
                // Every chord within the tolerance of the circle.
                let m = lerp(w[0].0, w[1].0, 0.5);
                assert!(r - norm(sub(m, p)) <= tol + 1e-9, "{r}");
            }
        }
        // A ring cut by the plane's edge: an open arc ending on it.
        let paths = contour(&drive, &field, &peaks_of(&drive, &field), 12., &faces, tol);
        assert!(!paths.is_empty());
        for path in &paths {
            for (q, _) in path {
                assert!((norm(sub(*q, p)) - 12.).abs() < 1e-9);
            }
        }
    }

    /// The Python dome: a height field over 40 x 40 at 0.5 mm with walls.
    fn dome() -> SurfaceMesh {
        height_field(0.5, |x, y| {
            (30f64.powi(2) - (x - 20.).powi(2) - (y - 20.).powi(2))
                .max(0.)
                .sqrt()
                .max(12.5)
                - 12.
        })
    }

    /// A height field over 40 x 40 at `step` with walls down to Z0, as the
    /// Python tests build.
    fn height_field(step: f64, z: impl Fn(f64, f64) -> f64) -> SurfaceMesh {
        let n = (40. / step).round() as usize;
        let at = |i: usize, j: usize| {
            let (x, y) = (i as f64 * step, j as f64 * step);
            [x, y, z(x, y)]
        };
        let mut tris = vec![];
        for j in 0..n {
            for i in 0..n {
                let (a, b, c, d) = (at(i, j), at(i + 1, j), at(i + 1, j + 1), at(i, j + 1));
                tris.push([a, b, c]);
                tris.push([a, c, d]);
            }
        }
        let mut rim = vec![];
        rim.extend((0..n).map(|i| at(i, 0)));
        rim.extend((0..n).map(|j| at(n, j)));
        rim.extend((1..=n).rev().map(|i| at(i, n)));
        rim.extend((1..=n).rev().map(|j| at(0, j)));
        for k in 0..rim.len() {
            let (p, q) = (rim[k], rim[(k + 1) % rim.len()]);
            let (p0, q0) = ([p[0], p[1], 0.], [q[0], q[1], 0.]);
            tris.push([p, q, q0]);
            tris.push([p, q0, p0]);
        }
        mesh(tris)
    }

    #[test]
    fn contours_close_or_end_on_the_boundary() {
        let never = AtomicBool::new(false);
        let drive = Drive::build(&dome(), &never).unwrap();
        let (ti, _, p) = drive.top_at([20., 20.]).unwrap();
        let field = Field::compute(
            &drive.v,
            &drive.t,
            &drive.topo,
            &[Piece::Point(ti, p)],
            f64::INFINITY,
            &never,
        )
        .unwrap();
        let emax: Vec<f64> = (0..drive.topo.edges.len() as u32)
            .map(|e| field.edge_max(e))
            .collect();
        let top = emax.iter().copied().fold(0., f64::max);
        let peaks = level_peaks(&drive, &field, &emax, 1.5, 0.5, top, &never).unwrap();
        let faces: Vec<u32> = (0..drive.t.len() as u32).collect();
        for k in 0..23 {
            let level = (0.5 + k as f64) * 1.5;
            let mut iso = Iso {
                drive: &drive,
                field: &field,
                peaks: &peaks,
                level,
                tol: 0.005,
                nodes: vec![],
                edges: HashMap::new(),
                at_vertex: HashMap::new(),
                links: vec![],
            };
            for &f in &faces {
                iso.face(f);
            }
            let mut degree: HashMap<u32, usize> = HashMap::new();
            for &(a, b, _, _) in &iso.links {
                *degree.entry(a).or_default() += 1;
                *degree.entry(b).or_default() += 1;
            }
            let on_boundary: std::collections::HashSet<u32> = iso
                .edges
                .iter()
                .filter(|(e, _)| drive.topo.faces[**e as usize][1] == NONE)
                .flat_map(|(_, c)| c.iter().map(|x| x.1))
                .collect();
            for (node, d) in degree {
                assert!(
                    d == 2 || (d == 1 && on_boundary.contains(&node)),
                    "level {level}: node {:?} has {d} links",
                    iso.nodes[node as usize]
                );
            }
        }
    }

    #[test]
    fn neighbouring_contours_are_a_stepover_apart() {
        // From a point the distance to the level-k iso-curve at a point of
        // level k + 1 is exactly the stepover; measured from the planned
        // contour (chords within `tol` of it) as a polygonal source.
        let never = AtomicBool::new(false);
        let saddle = |x: f64, y: f64| 10. + ((x - 20.).powi(2) - (y - 20.).powi(2)) / 60.;
        for (surface, levels) in [
            (dome(), vec![3usize, 8, 12]),
            (height_field(0.5, saddle), (0..17).collect()),
            (height_field(0.25, saddle), vec![6, 7, 8]),
        ] {
            let drive = Drive::build(&surface, &never).unwrap();
            let (ti, _, p) = drive.top_at([20., 20.]).unwrap();
            let field = Field::compute(
                &drive.v,
                &drive.t,
                &drive.topo,
                &[Piece::Point(ti, p)],
                f64::INFINITY,
                &never,
            )
            .unwrap();
            let emax: Vec<f64> = (0..drive.topo.edges.len() as u32)
                .map(|e| field.edge_max(e))
                .collect();
            let top = emax.iter().copied().fold(0., f64::max);
            let peaks = level_peaks(&drive, &field, &emax, 1.5, 0.5, top, &never).unwrap();
            let faces: Vec<u32> = (0..drive.t.len() as u32).collect();
            let tol = 0.005;
            let locate = |q: Vec3| drive.locate(q, 1e-6).map(|(f, _)| f).unwrap();
            for k in levels {
                let (a, b) = ((0.5 + k as f64) * 1.5, (1.5 + k as f64) * 1.5);
                let mut sources = vec![];
                for path in contour(&drive, &field, &peaks, a, &faces, tol) {
                    for w in path.windows(2) {
                        let (q0, q1) = (w[0].0, w[1].0);
                        assert!(
                            walk(
                                &drive.v,
                                &drive.t,
                                &drive.topo,
                                locate(q0),
                                q0,
                                [q1[0], q1[1]],
                                &mut sources
                            )
                            .is_some(),
                            "walk {q0:?} -> {q1:?}"
                        );
                    }
                }
                let near =
                    Field::compute(&drive.v, &drive.t, &drive.topo, &sources, 3., &never).unwrap();
                for path in contour(&drive, &field, &peaks, b, &faces, tol) {
                    for (q, _) in path {
                        let s = near.at(locate(q), q);
                        assert!((s - 1.5).abs() <= tol + 1e-6, "level {k}: {q:?} {s}");
                    }
                }
            }
        }
    }

    /// A flat region: the unit square mapped by `map`, n x n cells.
    fn planar(n: usize, map: impl Fn(f64, f64) -> [f64; 2]) -> SurfaceMesh {
        let at = |i: usize, j: usize| {
            let q = map(i as f64 / n as f64, j as f64 / n as f64);
            [q[0], q[1], 2.]
        };
        let mut tris = vec![];
        for j in 0..n {
            for i in 0..n {
                let (a, b, c, d) = (at(i, j), at(i + 1, j), at(i + 1, j + 1), at(i, j + 1));
                tris.push([a, b, c]);
                tris.push([a, c, d]);
            }
        }
        mesh(tris)
    }

    #[test]
    fn every_point_is_within_half_a_stepover_of_a_pass() {
        // On a flat convex region the geodesic distance is Euclidean: dense
        // samples' distance to the planned passes, computed independently,
        // stays within half the stepover plus the chord tolerance (the
        // levels alone leave a peak at a square's centre, a flat ridge along
        // a rectangle and a slowly rising ridge along a tapered trapezoid).
        let never = AtomicBool::new(false);
        let (stepover, tol) = (1.5, 0.005);
        let cases: Vec<(&str, SurfaceMesh, GeodesicSeed)> = vec![
            (
                "square",
                planar(40, |u, v| [40. * u, 40. * v]),
                GeodesicSeed::Boundary,
            ),
            (
                "rectangle",
                planar(40, |u, v| [40. * u, 30. * v]),
                GeodesicSeed::Boundary,
            ),
            (
                "trapezoid",
                planar(40, |u, v| [40. * u, (v - 0.5) * (30. - 6. * u) + 20.]),
                GeodesicSeed::Boundary,
            ),
            (
                "point",
                planar(40, |u, v| [40. * u, 40. * v]),
                GeodesicSeed::Point { xy_mm: [13.3, 9.1] },
            ),
        ];
        for (name, surface, seed) in cases {
            let drive = Drive::build(&surface, &never).unwrap();
            let planned = plan(&drive, &seed, stepover, tol, &never).unwrap();
            // Pass segments, bucketed by stepover-sized cells.
            let cell = stepover;
            let mut grid: SegmentGrid = HashMap::new();
            for (_, _, _, path) in &planned.passes {
                for w in path.windows(2) {
                    let (a, b) = ([w[0].0[0], w[0].0[1]], [w[1].0[0], w[1].0[1]]);
                    let (x0, x1) = (a[0].min(b[0]), a[0].max(b[0]));
                    let (y0, y1) = (a[1].min(b[1]), a[1].max(b[1]));
                    for x in (x0 / cell).floor() as i64..=(x1 / cell).floor() as i64 {
                        for y in (y0 / cell).floor() as i64..=(y1 / cell).floor() as i64 {
                            grid.entry((x, y)).or_default().push((a, b));
                        }
                    }
                }
            }
            let dist = |p: [f64; 2]| {
                let (cx, cy) = ((p[0] / cell).floor() as i64, (p[1] / cell).floor() as i64);
                let mut best = f64::INFINITY;
                for x in cx - 1..=cx + 1 {
                    for y in cy - 1..=cy + 1 {
                        for (a, b) in grid.get(&(x, y)).map_or(&[][..], Vec::as_slice) {
                            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
                            let ll = dx * dx + dy * dy;
                            let t = if ll > 0. {
                                (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / ll).clamp(0., 1.)
                            } else {
                                0.
                            };
                            best = best.min((p[0] - a[0] - t * dx).hypot(p[1] - a[1] - t * dy));
                        }
                    }
                }
                best
            };
            let mut worst = 0f64;
            let n = 400;
            for j in 0..=n {
                for i in 0..=n {
                    let (u, v) = (i as f64 / n as f64, j as f64 / n as f64);
                    // The same maps as the meshes.
                    let p = match name {
                        "rectangle" => [40. * u, 30. * v],
                        "trapezoid" => [40. * u, (v - 0.5) * (30. - 6. * u) + 20.],
                        _ => [40. * u, 40. * v],
                    };
                    worst = worst.max(dist(p));
                }
            }
            let target = stepover / 2. + tol;
            assert!(
                worst <= target + 1e-9,
                "{name}: a point {worst} from every pass"
            );
            assert!(
                planned.coverage <= target + 1e-9,
                "{name}: certified {}",
                planned.coverage
            );
            assert!(
                worst <= planned.coverage + 1e-9,
                "{name}: sampled {worst} above certified {}",
                planned.coverage
            );
        }
    }

    #[test]
    #[ignore]
    fn point_field_external_mesh() {
        // Local investigation: GEO_MESH, GEO_SRC=x,y,z GEO_AT=x,y,z
        let never = AtomicBool::new(false);
        let tris: Vec<[Vec3; 3]> = serde_json::from_str(
            &std::fs::read_to_string(std::env::var("GEO_MESH").unwrap()).unwrap(),
        )
        .unwrap();
        let v3 = |k: &str| -> Vec3 {
            let v: Vec<f64> = std::env::var(k)
                .unwrap()
                .split(',')
                .map(|x| x.parse().unwrap())
                .collect();
            [v[0], v[1], v[2]]
        };
        let mut m = mesh(tris);
        m.surface_error_mm = std::env::var("GEO_ERR").map_or(0., |e| e.parse().unwrap());
        let drive = Drive::build(&m, &never).unwrap();
        let (src, at) = (v3("GEO_SRC"), v3("GEO_AT"));
        let (f, _, src) = drive.top_at([src[0], src[1]]).unwrap();
        let (g, _, at) = drive.top_at([at[0], at[1]]).unwrap();
        let field = Field::compute(
            &drive.v,
            &drive.t,
            &drive.topo,
            &[Piece::Point(f, src)],
            5.,
            &never,
        )
        .unwrap();
        eprintln!(
            "source face {f}, at face {g}: field {:.4}, straight {:.4}",
            field.at(g, at),
            norm(sub(at, src))
        );
        for &h in &drive.within(src, 1e-7) {
            eprintln!("  carrier {h}: n {:?}", drive.n[h as usize]);
        }
    }

    #[test]
    #[ignore]
    fn plan_external_mesh() {
        // Local investigation: GEO_MESH=<json triangles> GEO_SEED=boundary|x,y
        let never = AtomicBool::new(false);
        let tris: Vec<[Vec3; 3]> = serde_json::from_str(
            &std::fs::read_to_string(std::env::var("GEO_MESH").unwrap()).unwrap(),
        )
        .unwrap();
        let seed = match std::env::var("GEO_SEED").unwrap().as_str() {
            "boundary" => GeodesicSeed::Boundary,
            s => {
                let v: Vec<f64> = s.split(',').map(|x| x.parse().unwrap()).collect();
                GeodesicSeed::Point {
                    xy_mm: [v[0], v[1]],
                }
            }
        };
        let t0 = std::time::Instant::now();
        let mut m = mesh(tris);
        m.surface_error_mm = std::env::var("GEO_ERR").map_or(0., |e| e.parse().unwrap());
        let drive = Drive::build(&m, &never).unwrap();
        eprintln!("drive {} triangles in {:?}", drive.t.len(), t0.elapsed());
        let open: Vec<usize> = (0..drive.topo.edges.len())
            .filter(|&e| drive.topo.faces[e][1] == NONE)
            .collect();
        if let GeodesicSeed::Point { xy_mm } = &seed {
            let near = open
                .iter()
                .filter(|&&e| {
                    let [a, b] = drive.topo.edges[e];
                    let m = lerp(drive.v[a as usize], drive.v[b as usize], 0.5);
                    (m[0] - xy_mm[0]).hypot(m[1] - xy_mm[1]) < 3.
                })
                .count();
            eprintln!(
                "open edges {} ({} within 3 mm of the seed)",
                open.len(),
                near
            );
        }
        let planned = plan(&drive, &seed, 1.5, 0.005, &never).unwrap();
        eprintln!(
            "plan: {} passes, {} fills, coverage {:.4} in {:?}",
            planned.passes.len(),
            planned.caps,
            planned.coverage,
            t0.elapsed()
        );
        for y in [12.0, 15.0, 17.0] {
            let row: Vec<String> = (0..9)
                .map(|i| {
                    let x = 10. + 2.5 * i as f64;
                    match drive.locate([x, y, 10.], 1e-3) {
                        Some((f, _)) => {
                            format!("{x:.1}:f{f}={:.3}", planned.field.at(f, [x, y, 10.]))
                        }
                        None => format!("{x:.1}:none"),
                    }
                })
                .collect();
            eprintln!("y={y}: {}", row.join(" "));
        }
        if let GeodesicSeed::Point { xy_mm } = seed {
            for (k, _, level, path) in planned.passes.iter().filter(|p| p.0 == 0) {
                let d: Vec<f64> = path
                    .iter()
                    .map(|(q, _)| (q[0] - xy_mm[0]).hypot(q[1] - xy_mm[1]))
                    .collect();
                eprintln!(
                    "ring k={k} level {level}: {} points, plan distance from seed {:.4}..{:.4}",
                    d.len(),
                    d.iter().copied().fold(f64::INFINITY, f64::min),
                    d.iter().copied().fold(0., f64::max)
                );
            }
        }
        for (k, cap, level, path) in planned.passes.iter() {
            let longest = path
                .windows(2)
                .map(|w| norm(sub(w[1].0, w[0].0)))
                .fold(0., f64::max);
            let len: f64 = path.windows(2).map(|w| norm(sub(w[1].0, w[0].0))).sum();
            let (a, b) = (path[0].0, path[path.len() - 1].0);
            eprintln!(
                "pass k={k} cap={cap} level={level:.3} n={} len={len:.2} longest={longest:.3} {:.2?} -> {:.2?}",
                path.len(),
                a,
                b
            );
        }
    }

    #[test]
    fn contours_on_large_faces_follow_the_curve() {
        // A 40 x 30 mm rectangle of two triangles, from its boundary: each
        // level is a rectangle whose corners lie inside the faces (the
        // diagonal misses them), where the curve turns back on itself
        // between its two crossings on the diagonal. Every vertex and every
        // chord midpoint lies on its level (within the tolerance): no chord
        // cuts a corner.
        let never = AtomicBool::new(false);
        let (a, b, c, d) = ([0., 0., 0.], [40., 0., 0.], [40., 30., 0.], [0., 30., 0.]);
        let drive = Drive::build(&mesh(vec![[a, b, c], [a, c, d]]), &never).unwrap();
        let planned = plan(&drive, &GeodesicSeed::Boundary, 1.5, 0.005, &never).unwrap();
        let exact = |p: Vec3| p[0].min(40. - p[0]).min(p[1]).min(30. - p[1]);
        let mut rings = 0;
        for (_, cap, level, path) in &planned.passes {
            if *cap {
                continue;
            }
            rings += 1;
            for w in path.windows(2) {
                for p in [w[0].0, lerp(w[0].0, w[1].0, 0.5)] {
                    assert!(
                        (exact(p) - level).abs() <= 0.005,
                        "level {level}: {p:?} at {}",
                        exact(p)
                    );
                }
            }
        }
        assert!(rings >= 9);
    }

    #[test]
    fn locate_finds_a_point_just_outside_the_plan_box() {
        // A point on the mesh's edge but outside its plan box by rounding
        // lies within the tolerance of a triangle filed in another cell.
        let never = AtomicBool::new(false);
        let (a, b, c, d) = ([0., 0., 6.], [40., 0., 6.], [40., 30., 10.], [0., 30., 10.]);
        let drive = Drive::build(&mesh(vec![[a, b, c], [a, c, d]]), &never).unwrap();
        assert!(drive.locate([20., -8.9e-16, 6.], 1e-6).is_some());
        assert!(drive.locate([-1e-9, 15., 8.], 1e-6).is_some());
    }

    #[test]
    fn each_level_is_one_unbroken_ring() {
        // A flat square on a 0.5 mm grid from its boundary: every level
        // (a multiple of 1.5 mm) lies on grid lines, through vertices. Each
        // level is one closed ring.
        let never = AtomicBool::new(false);
        let saddle = |x: f64, y: f64| 10. + ((x - 20.).powi(2) - (y - 20.).powi(2)) / 60.;
        for surface in [
            planar(80, |u, v| [40. * u, 40. * v]),
            height_field(0.5, saddle),
        ] {
            let drive = Drive::build(&surface, &never).unwrap();
            let planned = plan(&drive, &GeodesicSeed::Boundary, 1.5, 0.005, &never).unwrap();
            let mut rings: HashMap<usize, Vec<&ContactPath>> = HashMap::new();
            for (k, cap, _, path) in &planned.passes {
                if !cap {
                    rings.entry(*k).or_default().push(path);
                }
            }
            assert!(rings.len() > 10);
            for (k, paths) in &rings {
                assert_eq!(paths.len(), 1, "level {k}: {} pieces", paths.len());
                let p = paths[0];
                assert!(norm(sub(p[0].0, p[p.len() - 1].0)) < 1e-9, "level {k} open");
            }
        }
    }

    #[test]
    fn walls_and_undersides_bound_the_drive_surface() {
        let never = AtomicBool::new(false);
        // A 10 x 10 x 5 box: the top faces up; walls and bottom do not.
        let p = |x: usize, y: usize, z: usize| [[0., 10.][x], [0., 10.][y], [0., 5.][z]];
        let q = |a: Vec3, b: Vec3, c: Vec3, d: Vec3| vec![[a, b, c], [a, c, d]];
        let tris = [
            q(p(0, 0, 1), p(1, 0, 1), p(1, 1, 1), p(0, 1, 1)),
            q(p(0, 0, 0), p(0, 1, 0), p(1, 1, 0), p(1, 0, 0)),
            q(p(0, 0, 0), p(1, 0, 0), p(1, 0, 1), p(0, 0, 1)),
            q(p(0, 1, 0), p(0, 1, 1), p(1, 1, 1), p(1, 1, 0)),
            q(p(0, 0, 0), p(0, 0, 1), p(0, 1, 1), p(0, 1, 0)),
            q(p(1, 0, 0), p(1, 1, 0), p(1, 1, 1), p(1, 0, 1)),
        ]
        .concat();
        let drive = Drive::build(&mesh(tris), &never).unwrap();
        for (k, v) in drive.v.iter().enumerate() {
            if drive.vt[k].is_empty() {
                continue;
            }
            assert_eq!(v[2], 5.);
            let rim = v[0] == 0. || v[0] == 10. || v[1] == 0. || v[1] == 10.;
            assert_eq!(drive.boundary[k], rim, "{v:?}");
        }
    }
}
