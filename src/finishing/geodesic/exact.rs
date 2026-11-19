//! Exact geodesic distance on the drive mesh, from points and polygonal
//! curves on it.
//!
//! - Window propagation: the exact algorithm of Mitchell, Mount and
//!   Papadimitriou (1987), as implemented by Surazhsky, Surazhsky, Kirsanov,
//!   Gortler and Hoppe ("Fast exact and approximate geodesics on meshes",
//!   SIGGRAPH 2005, section 3). Each edge carries windows: intervals over
//!   which the distance is that of one unfolded pseudosource (a point at
//!   distance `sigma` from the source, section 3.1). A window is propagated
//!   across the next face by extending its pencil of rays (3.2); a window on
//!   an edge keeps only the part where it is strictly shorter than the
//!   windows already there (3.3), whichever side they came from; saddle
//!   vertices (total angle above 2 pi) and boundary vertices become
//!   pseudosources (3.2, 3.4). The queue is ordered by each window's least
//!   distance (3.4); the result does not depend on the order.
//! - Polygonal sources: Bommes and Kobbelt ("Accurate computation of
//!   geodesic distance fields for polygonal curves on triangle meshes", VMV
//!   2007, section 4). A source curve is split into segments that each lie
//!   in one triangle; each segment emits linear windows (waves orthogonal to
//!   it, 4.2.1) and each end point circular windows (4.1; a point on an
//!   edge initialises both triangles). Overlaps are resolved the same way
//!   for circular-circular, circular-linear (up to two crossings, a
//!   trisection) and linear-linear windows (4.2.2).
//! - Evaluation: on an edge the distance is the minimum of its windows; at
//!   a point p inside a face it is the minimum over the windows on the
//!   face's edges of min over p' in the window of |p - p'| + D(p') (2005,
//!   section 3.5), over the vertices of |p - v| + D(v), and over sources in
//!   the face. Each of those is a convex function of p (a partial minimum
//!   of a jointly convex function over a convex set), so on any segment
//!   the distance is a minimum of convex functions: its level crossings are
//!   roots of the circles and lines that make it up, kept where they are
//!   the minimum and where the sign of D - level changes. That is how
//!   passes are placed exactly on the iso-curves.
use super::trace;
use super::{Vec3, cross, dot, norm, sub};
use crate::stock::CancelSignal;
use std::{
    cmp::Ordering,
    collections::{BinaryHeap, HashMap},
};

pub(super) const NONE: u32 = u32::MAX;

fn scale(a: Vec3, k: f64) -> Vec3 {
    [a[0] * k, a[1] * k, a[2] * k]
}
fn axpy(a: Vec3, k: f64, b: Vec3) -> Vec3 {
    [a[0] + k * b[0], a[1] + k * b[1], a[2] + k * b[2]]
}

/// Real roots of a x^2 + b x + c = 0 (linear when a vanishes against the
/// other coefficients), appended to `out`.
fn roots(a: f64, b: f64, c: f64, out: &mut Vec<f64>) {
    let s = a.abs().max(b.abs()).max(c.abs());
    if s == 0. || !s.is_finite() {
        return;
    }
    if a.abs() <= 1e-13 * s {
        if b != 0. {
            out.push(-c / b);
        }
        return;
    }
    let disc = b * b - 4. * a * c;
    if disc < 0. {
        return;
    }
    let q = -0.5 * (b + b.signum() * disc.sqrt());
    out.push(q / a);
    if q != 0. {
        out.push(c / q);
    }
}

/// Mesh connectivity for propagation: edges (a < b) with their faces, each
/// face's edges, the in-plane frame of every edge in each of its faces, and
/// the vertices through which shortest paths may bend.
#[derive(Default)]
pub(super) struct Topology {
    pub edges: Vec<[u32; 2]>,
    pub len: Vec<f64>,
    /// Unit vector along each edge, from its first to its second vertex.
    pub xhat: Vec<Vec3>,
    pub faces: Vec<[u32; 2]>,
    /// Face k's edge i joins t[k][i] and t[k][(i + 1) % 3].
    pub tri_edges: Vec<[u32; 3]>,
    /// Face k's in-plane unit normal to its edge i, pointing into the face.
    pub yhat: Vec<[Vec3; 3]>,
    pub vertex_faces: Vec<Vec<u32>>,
    /// Boundary vertices and saddle vertices (total angle above 2 pi):
    /// shortest paths may pass through these (MMP 1987), so they act as
    /// pseudosources.
    pub pseudo: Vec<bool>,
}

impl Topology {
    pub fn new(v: &[Vec3], t: &[[u32; 3]], boundary: &[bool]) -> Result<Self, String> {
        let mut index: HashMap<(u32, u32), u32> = HashMap::with_capacity(t.len() * 2);
        let (mut edges, mut faces) = (vec![], vec![]);
        let mut tri_edges = Vec::with_capacity(t.len());
        for (fi, tri) in t.iter().enumerate() {
            let mut te = [0u32; 3];
            for k in 0..3 {
                let (a, b) = (tri[k], tri[(k + 1) % 3]);
                let key = (a.min(b), a.max(b));
                let id = *index.entry(key).or_insert_with(|| {
                    edges.push([key.0, key.1]);
                    faces.push([NONE, NONE]);
                    (edges.len() - 1) as u32
                });
                let slot: &mut [u32; 2] = &mut faces[id as usize];
                if slot[0] == NONE {
                    slot[0] = fi as u32;
                } else if slot[1] == NONE {
                    slot[1] = fi as u32;
                } else {
                    let (p, q) = (v[key.0 as usize], v[key.1 as usize]);
                    return Err(format!(
                        "geodesic finishing: the drive surface is not a manifold: the edge {p:?} to {q:?} is shared by more than two upward triangles; repair the mesh"
                    ));
                }
                te[k] = id;
            }
            tri_edges.push(te);
        }
        let len: Vec<f64> = edges
            .iter()
            .map(|&[a, b]| norm(sub(v[b as usize], v[a as usize])))
            .collect();
        let xhat: Vec<Vec3> = edges
            .iter()
            .zip(&len)
            .map(|(&[a, b], &l)| scale(sub(v[b as usize], v[a as usize]), 1. / l))
            .collect();
        let yhat = t
            .iter()
            .zip(&tri_edges)
            .map(|(tri, te)| {
                [0, 1, 2].map(|k| {
                    let e = te[k] as usize;
                    let o = v[edges[e][0] as usize];
                    let w = sub(v[tri[(k + 2) % 3] as usize], o);
                    let p = axpy(w, -dot(w, xhat[e]), xhat[e]);
                    scale(p, 1. / norm(p))
                })
            })
            .collect();
        let mut vertex_faces = vec![vec![]; v.len()];
        let mut angle = vec![0f64; v.len()];
        for (fi, tri) in t.iter().enumerate() {
            for k in 0..3 {
                let p = v[tri[k] as usize];
                let (x, y) = (
                    sub(v[tri[(k + 1) % 3] as usize], p),
                    sub(v[tri[(k + 2) % 3] as usize], p),
                );
                angle[tri[k] as usize] += norm(cross(x, y)).atan2(dot(x, y));
                vertex_faces[tri[k] as usize].push(fi as u32);
            }
        }
        let pseudo = (0..v.len())
            .map(|k| boundary[k] || angle[k] > std::f64::consts::TAU + 1e-9)
            .collect();
        Ok(Topology {
            edges,
            len,
            xhat,
            faces,
            tri_edges,
            yhat,
            vertex_faces,
            pseudo,
        })
    }

    pub fn other(&self, e: u32, f: u32) -> u32 {
        let [a, b] = self.faces[e as usize];
        if a == f { b } else { a }
    }

    fn local(&self, f: u32, e: u32) -> usize {
        (0..3)
            .find(|&k| self.tri_edges[f as usize][k] == e)
            .unwrap()
    }
}

/// A source on the drive surface: a point, or a straight segment, in the
/// (closed) face given.
#[derive(Clone, Copy, Debug)]
pub(super) enum Piece {
    Point(u32, Vec3),
    Segment(u32, Vec3, Vec3),
}

/// A window's distance function in its edge's frame: x along the edge from
/// its first vertex, y normal to it, positive away from the source side.
#[derive(Clone, Copy, Debug)]
pub(super) enum Src {
    /// A circular window: an unfolded pseudosource (x, y <= 0) at geodesic
    /// distance `s` from the source.
    Point { x: f64, y: f64, s: f64 },
    /// A linear window: D(x, y) = c + ux x + uy y, uy >= 0.
    Line { ux: f64, uy: f64, c: f64 },
}

impl Src {
    fn edge(&self, x: f64) -> f64 {
        match *self {
            Src::Point { x: sx, y: sy, s } => (x - sx).hypot(sy) + s,
            Src::Line { ux, c, .. } => c + ux * x,
        }
    }
    /// The distance at (x, y >= 0) along the window's pencil, unrestricted.
    fn free(&self, x: f64, y: f64) -> f64 {
        match *self {
            Src::Point { x: sx, y: sy, s } => (x - sx).hypot(y - sy) + s,
            Src::Line { ux, uy, c } => c + ux * x + uy * y,
        }
    }
    /// Where the ray back from (x, y >= 0) crosses the edge.
    fn foot(&self, x: f64, y: f64) -> f64 {
        match *self {
            Src::Point { x: sx, y: sy, .. } => {
                let k = y - sy;
                if k > 0. { x + (sx - x) * y / k } else { x }
            }
            Src::Line { ux, uy, .. } => {
                if uy > 0. {
                    x - ux * y / uy
                } else {
                    x
                }
            }
        }
    }
    fn lowest(&self, b0: f64, b1: f64) -> f64 {
        match *self {
            Src::Point { x, y, s } if (b0..=b1).contains(&x) => y.abs() + s,
            _ => self.edge(b0).min(self.edge(b1)),
        }
    }
}

/// Where two windows' distances along an edge are equal: the roots of a
/// quadratic (2005 section 3.3, 2007 section 4.2.2), unfiltered.
fn ties(p: &Src, q: &Src, out: &mut Vec<f64>) {
    match (*p, *q) {
        (
            Src::Point {
                x: a0,
                y: h0,
                s: s0,
            },
            Src::Point {
                x: a1,
                y: h1,
                s: s1,
            },
        ) => {
            // r0 = r1 + b with b = s1 - s0; r0^2 - r1^2 - b^2 = 2 b r1 is
            // linear in x (k1 x + k0); squared once more.
            let b = s1 - s0;
            let k1 = 2. * (a1 - a0);
            let k0 = a0 * a0 + h0 * h0 - a1 * a1 - h1 * h1 - b * b;
            if b == 0. {
                // Equal offsets (two points of the source): the bisector,
                // linear; squaring it would give a double root that
                // rounding can lose.
                if k1 != 0. {
                    out.push(-k0 / k1);
                }
                return;
            }
            roots(
                k1 * k1 - 4. * b * b,
                2. * k1 * k0 + 8. * b * b * a1,
                k0 * k0 - 4. * b * b * (a1 * a1 + h1 * h1),
                out,
            );
        }
        (Src::Point { x: a, y: h, s }, Src::Line { ux, c, .. })
        | (Src::Line { ux, c, .. }, Src::Point { x: a, y: h, s }) => {
            // r = (c - s) + ux x, squared.
            let n = c - s;
            roots(
                1. - ux * ux,
                -2. * a - 2. * ux * n,
                a * a + h * h - n * n,
                out,
            );
        }
        (Src::Line { ux: u0, c: c0, .. }, Src::Line { ux: u1, c: c1, .. }) => {
            if u0 != u1 {
                out.push((c1 - c0) / (u0 - u1));
            }
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct Win {
    edge: u32,
    /// The face the window propagates into (NONE: evaluation only).
    into: u32,
    b0: f64,
    b1: f64,
    src: Src,
    alive: bool,
    queued: bool,
    /// Its least value along its range (set when final).
    low: f64,
}

impl Win {
    /// min over p' in the window of |p - p'| + D(p'), p = (x, y >= 0).
    fn eval(&self, x: f64, y: f64) -> f64 {
        let f = self.src.foot(x, y);
        if f >= self.b0 && f <= self.b1 {
            self.src.free(x, y)
        } else {
            let q = f.clamp(self.b0, self.b1);
            (x - q).hypot(y) + self.src.edge(q)
        }
    }
}

#[derive(PartialEq)]
struct Event(f64, u32, bool);
impl Eq for Event {}
impl PartialOrd for Event {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for Event {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .0
            .total_cmp(&self.0)
            .then(other.1.cmp(&self.1))
            .then(other.2.cmp(&self.2))
    }
}

/// The exact distance field: windows on every edge reached within `limit`.
pub(super) struct Field<'a> {
    v: &'a [Vec3],
    t: &'a [[u32; 3]],
    topo: &'a Topology,
    wins: Vec<Win>,
    on_edge: Vec<Vec<u32>>,
    /// The distance at each vertex (infinite where not reached).
    pub vertex: Vec<f64>,
    emitted: Vec<f64>,
    in_face: HashMap<u32, Vec<Piece>>,
    /// Per (face, point) of a polyline source inside that face: the
    /// directions of its segments there (the point is nearest only where
    /// it is behind every one of them).
    behind: HashMap<(u32, [u64; 3]), Vec<Vec3>>,
    queue: BinaryHeap<Event>,
    limit: f64,
}

/// Keep crossings of a minimum of convex functions: candidates `c` (the
/// roots of its pieces, in [0, 1]) where the sign of D - level changes.
/// A value on the level counts as above it (symbolic perturbation): a
/// root at an end of the segment, where its end is on the level, is kept
/// there.
fn crossings(mut c: Vec<f64>, s0: bool, s1: bool, sign: impl Fn(f64) -> bool) -> Vec<f64> {
    c.retain(|t| (0. ..=1.).contains(t));
    c.sort_by(f64::total_cmp);
    c.dedup_by(|a, b| (*a - *b).abs() <= 1e-13);
    let mut out = vec![];
    let mut prev = s0;
    for j in 0..c.len() {
        let next = if j + 1 < c.len() {
            sign((c[j] + c[j + 1]) / 2.)
        } else {
            s1
        };
        if next != prev {
            out.push(c[j]);
        }
        prev = next;
    }
    out
}

impl<'a> Field<'a> {
    pub fn compute(
        v: &'a [Vec3],
        t: &'a [[u32; 3]],
        topo: &'a Topology,
        sources: &[Piece],
        limit: f64,
        cancel: &(impl CancelSignal + ?Sized),
    ) -> Result<Self, String> {
        let mut field = Field {
            v,
            t,
            topo,
            wins: vec![],
            on_edge: vec![vec![]; topo.edges.len()],
            vertex: vec![f64::INFINITY; v.len()],
            emitted: vec![f64::INFINITY; v.len()],
            in_face: HashMap::new(),
            behind: HashMap::new(),
            queue: BinaryHeap::new(),
            limit,
        };
        for &piece in sources {
            if let Piece::Segment(f, a, b) = piece {
                let d = sub(b, a);
                let l = norm(d);
                if l > 0. {
                    let d = scale(d, 1. / l);
                    field
                        .behind
                        .entry((f, a.map(f64::to_bits)))
                        .or_default()
                        .push(d);
                    field
                        .behind
                        .entry((f, b.map(f64::to_bits)))
                        .or_default()
                        .push(scale(d, -1.));
                }
            }
        }
        // A polyline's pieces share their ends: each point source once.
        let mut points: std::collections::HashSet<[u64; 3]> = std::collections::HashSet::new();
        let t_ins = std::time::Instant::now();
        for (n, &piece) in sources.iter().enumerate() {
            if trace::on() && n % 2000 == 0 && sources.len() > 1000 {
                trace::line(|| {
                    format!(
                        "  inserting {n}/{}: windows {} at {:?}",
                        sources.len(),
                        field.wins.len(),
                        t_ins.elapsed()
                    )
                });
            }
            let before = field.wins.len();
            let t_one = std::time::Instant::now();
            match piece {
                Piece::Point(f, p) => {
                    if points.insert(p.map(f64::to_bits)) {
                        field.source_point(f, p);
                    }
                }
                Piece::Segment(f, a, b) => {
                    for q in [a, b] {
                        if points.insert(q.map(f64::to_bits)) {
                            field.source_point(f, q);
                        }
                    }
                    field.source_segment(f, a, b);
                }
            }
            if trace::on() && (field.wins.len() - before > 2000 || t_one.elapsed().as_millis() > 5)
            {
                let (kind, f, a) = match piece {
                    Piece::Point(f, a) => ("point", f, a),
                    Piece::Segment(f, a, _) => ("segment", f, a),
                };
                trace::line(|| {
                    format!(
                        "  piece {n} {kind} face {f} at {a:?}: +{} windows in {:?}, carriers {}",
                        field.wins.len() - before,
                        t_one.elapsed(),
                        field.carrier(f, a).len()
                    )
                });
            }
        }
        if trace::on() && sources.len() > 1000 {
            let alive = field.wins.iter().filter(|w| w.alive).count();
            let worst = field.on_edge.iter().map(Vec::len).max().unwrap_or(0);
            trace::line(|| {
                format!(
                    "sources inserted: {} pieces, windows {} alive {alive}, most on one edge {worst}, queue {}",
                    sources.len(),
                    field.wins.len(),
                    field.queue.len()
                )
            });
        }
        let t_prop = std::time::Instant::now();
        let mut steps = 0usize;
        while let Some(Event(key, id, vertex)) = field.queue.pop() {
            steps += 1;
            if steps.is_multiple_of(16384) {
                if trace::on() && steps.is_multiple_of(1 << 20) {
                    let alive = field.wins.iter().filter(|w| w.alive).count();
                    trace::line(|| {
                        format!(
                            "field steps {steps} windows {} alive {alive} queue {} key {key:.6} sources {}",
                            field.wins.len(),
                            field.queue.len(),
                            sources.len()
                        )
                    });
                }
                if cancel.is_cancelled() {
                    return Err("geodesic planning cancelled".into());
                }
                crate::resources::check_memory(
                    field.wins.len() as u128 * std::mem::size_of::<Win>() as u128 * 2,
                    "geodesic distance windows",
                )?;
            }
            if vertex {
                let d = field.vertex[id as usize];
                if d < key || d >= field.emitted[id as usize] - 1e-10 * (1. + d) {
                    continue;
                }
                field.emitted[id as usize] = d;
                field.emit(id, d);
                continue;
            }
            let w = field.wins[id as usize];
            if !w.alive || !w.queued {
                continue;
            }
            field.wins[id as usize].queued = false;
            if w.into != NONE && w.src.lowest(w.b0, w.b1) <= field.limit {
                field.propagate(w);
            }
        }
        trace::line(|| {
            format!(
                "field: {} sources, {} windows over {} edges ({:.1} per edge), {steps} steps in {:?}",
                sources.len(),
                field.wins.len(),
                topo.edges.len(),
                field.wins.len() as f64 / topo.edges.len().max(1) as f64,
                t_prop.elapsed()
            )
        });
        // Every vertex at the least of its edges' windows there.
        for e in 0..topo.edges.len() {
            let [a, b] = topo.edges[e];
            let l = topo.len[e];
            for &id in &field.on_edge[e] {
                let w = field.wins[id as usize];
                if w.alive {
                    let (da, db) = (w.eval(0., 0.), w.eval(l, 0.));
                    field.vertex[a as usize] = field.vertex[a as usize].min(da);
                    field.vertex[b as usize] = field.vertex[b as usize].min(db);
                }
            }
        }
        // Each edge's windows, disjoint and each least on its own range,
        // in order along it.
        for w in &mut field.wins {
            w.low = w.src.lowest(w.b0, w.b1);
        }
        let wins = &field.wins;
        for list in &mut field.on_edge {
            list.retain(|&i| wins[i as usize].alive);
            list.sort_by(|&i, &j| wins[i as usize].b0.total_cmp(&wins[j as usize].b0));
        }
        field.queue = BinaryHeap::new();
        Ok(field)
    }

    fn coords(&self, e: u32, f: u32, p: Vec3) -> (f64, f64) {
        let w = sub(p, self.v[self.topo.edges[e as usize][0] as usize]);
        (
            dot(w, self.topo.xhat[e as usize]),
            dot(w, self.topo.yhat[f as usize][self.topo.local(f, e)]),
        )
    }

    fn third(&self, e: u32, f: u32) -> u32 {
        let [a, b] = self.topo.edges[e as usize];
        *self.t[f as usize]
            .iter()
            .find(|&&k| k != a && k != b)
            .unwrap()
    }

    /// The faces whose closure holds `p` (in face `f`): `f`, both faces of
    /// an edge it lies on, or every face around a vertex it lies at.
    fn carrier(&self, f: u32, p: Vec3) -> Vec<u32> {
        let te = self.topo.tri_edges[f as usize];
        let on: Vec<usize> = (0..3)
            .filter(|&k| self.coords(te[k], f, p).1 <= 1e-9 * self.topo.len[te[k] as usize])
            .collect();
        match on.len() {
            0 => vec![f],
            1 => self.topo.faces[te[on[0]] as usize]
                .into_iter()
                .filter(|&g| g != NONE)
                .collect(),
            _ => {
                // The vertex shared by the first two edges it lies on.
                let [a, b] = self.topo.edges[te[on[0]] as usize];
                let o = self.topo.edges[te[on[1]] as usize];
                let vx = if o.contains(&a) { a } else { b };
                self.topo.vertex_faces[vx as usize].clone()
            }
        }
    }

    fn source_point(&mut self, f: u32, p: Vec3) {
        let carriers = self.carrier(f, p);
        // A vertex of a polyline: nearest only behind each of its segments
        // (ahead of one, the segment's own points are nearer), a wedge
        // bounded by rays from it; its windows cover only the wedge's part
        // of each edge (exact, and slivers or nothing on a smooth curve's
        // chords). Inside a face, its segments there; on an edge between
        // two faces (where a curve crosses from one into the other), its
        // segments in both, the other face's unfolded into this one's
        // plane across the edge (as every window is).
        let key = p.map(f64::to_bits);
        let wedge_in = |g: u32| -> Vec<Vec3> {
            let own = self.behind.get(&(g, key)).cloned().unwrap_or_default();
            match carriers.len() {
                1 => own,
                2 => {
                    let h = if carriers[0] == g {
                        carriers[1]
                    } else {
                        carriers[0]
                    };
                    let Some(&e) = self.topo.tri_edges[g as usize]
                        .iter()
                        .find(|&&e| self.topo.faces[e as usize].contains(&h))
                    else {
                        return vec![];
                    };
                    let xh = self.topo.xhat[e as usize];
                    let (ig, ih) = (self.topo.local(g, e), self.topo.local(h, e));
                    // Into g / into h, perpendicular to the edge in each plane.
                    let (yg, yh) = (
                        self.topo.yhat[g as usize][ig],
                        self.topo.yhat[h as usize][ih],
                    );
                    let mut out = own;
                    for d in self.behind.get(&(h, key)).cloned().unwrap_or_default() {
                        // Across the edge: into h becomes out of g.
                        out.push(axpy(scale(xh, dot(d, xh)), -dot(d, yh), yg));
                    }
                    out
                }
                _ => vec![],
            }
        };
        // At a mesh vertex: the fan's sector (see `vertex_wedge`).
        if carriers.len() > 2
            && let Some(windows) = self.vertex_wedge(f, p, key)
        {
            for &g in &carriers {
                self.in_face.entry(g).or_default().push(Piece::Point(g, p));
            }
            for (g, e, b0, b1) in windows {
                let l = self.topo.len[e as usize];
                let (x, y) = self.coords(e, g, p);
                let on = y <= 1e-9 * l;
                if b1 - b0 <= 1e-12 * l {
                    continue;
                }
                self.add(Win {
                    edge: e,
                    into: if on { NONE } else { self.topo.other(e, g) },
                    b0,
                    b1,
                    src: Src::Point {
                        x,
                        y: (-y).min(0.),
                        s: 0.,
                    },
                    alive: true,
                    low: 0.,
                    queued: !on,
                });
            }
            return;
        }
        let wedges: Vec<(u32, Vec<Vec3>)> = carriers.iter().map(|&g| (g, wedge_in(g))).collect();
        for (g, wedge) in wedges {
            self.in_face.entry(g).or_default().push(Piece::Point(g, p));
            for k in 0..3 {
                let e = self.topo.tri_edges[g as usize][k];
                let l = self.topo.len[e as usize];
                let (x, y) = self.coords(e, g, p);
                let on = y <= 1e-9 * l;
                // The edge's points o + t xhat behind every segment:
                // (o - p) . d + t (xhat . d) <= 0.
                let (mut b0, mut b1) = (0f64, l);
                let o = self.v[self.topo.edges[e as usize][0] as usize];
                for d in &wedge {
                    let (k0, k1) = (dot(sub(o, p), *d), dot(self.topo.xhat[e as usize], *d));
                    if k1 > 0. {
                        b1 = b1.min(-k0 / k1);
                    } else if k1 < 0. {
                        b0 = b0.max(-k0 / k1);
                    } else if k0 > 0. {
                        b1 = -1.;
                    }
                }
                if b1 - b0 <= 1e-12 * l {
                    continue;
                }
                self.add(Win {
                    edge: e,
                    into: if on { NONE } else { self.topo.other(e, g) },
                    b0,
                    b1,
                    src: Src::Point {
                        x,
                        y: (-y).min(0.),
                        s: 0.,
                    },
                    alive: true,
                    low: 0.,
                    queued: !on,
                });
            }
        }
    }

    /// A polyline source at a mesh vertex (point `p` of face `f`): the
    /// windows its point needs, (face, edge, b0, b1), or None to keep them
    /// all (no segment leaves it, or its fan cannot be laid out). Laid
    /// flat by their angles at the vertex, its faces form a fan; the point
    /// is nearest only at directions at least a right angle (through the
    /// fan) from every segment leaving it: within a sector narrower than a
    /// half turn the unfolded straight line exists, and ahead of a segment
    /// its own points are nearer. Along a smooth curve that set is empty;
    /// at a convex corner it is the sector between the perpendiculars.
    fn vertex_wedge(&self, f: u32, p: Vec3, key: [u64; 3]) -> Option<Vec<(u32, u32, f64, f64)>> {
        let topo = self.topo;
        let vx = *self.t[f as usize].iter().min_by(|&&a, &&b| {
            norm(sub(self.v[a as usize], p)).total_cmp(&norm(sub(self.v[b as usize], p)))
        })?;
        let q = self.v[vx as usize];
        let fan_faces = &topo.vertex_faces[vx as usize];
        let at_vx = |g: u32| -> Vec<u32> {
            topo.tri_edges[g as usize]
                .iter()
                .copied()
                .filter(|&e| topo.edges[e as usize].contains(&vx))
                .collect()
        };
        let far_end = |e: u32| -> u32 {
            let [a, b] = topo.edges[e as usize];
            if a == vx { b } else { a }
        };
        let dir = |e: u32| -> Vec3 {
            let d = sub(self.v[far_end(e) as usize], q);
            scale(d, 1. / norm(d))
        };
        // The fan in order: from a boundary edge where there is one.
        let (mut g, mut e_in) = fan_faces
            .iter()
            .find_map(|&g| {
                at_vx(g)
                    .into_iter()
                    .find(|&e| topo.faces[e as usize][1] == NONE)
                    .map(|e| (g, e))
            })
            .or_else(|| fan_faces.first().map(|&g| (g, at_vx(g)[0])))?;
        let start = g;
        let mut fan: Vec<(u32, u32, u32, f64, f64)> = vec![]; // face, in, out, from, angle
        let mut total = 0.;
        let mut closed = false;
        for _ in 0..fan_faces.len() {
            let es = at_vx(g);
            if es.len() != 2 {
                return None;
            }
            let e_out = if es[0] == e_in { es[1] } else { es[0] };
            let th = dot(dir(e_in), dir(e_out)).clamp(-1., 1.).acos();
            fan.push((g, e_in, e_out, total, th));
            total += th;
            let next = topo.other(e_out, g);
            if next == NONE {
                break;
            }
            if next == start {
                closed = true;
                break;
            }
            (g, e_in) = (next, e_out);
        }
        if fan.len() != fan_faces.len() {
            return None;
        }
        // The segments' directions as fan angles.
        let mut phis = vec![];
        for &(g, e_in, e_out, from, th) in &fan {
            for d in self.behind.get(&(g, key)).into_iter().flatten() {
                let (u, o) = (dir(e_in), dir(e_out));
                let w = sub(o, scale(u, dot(o, u)));
                let w = scale(w, 1. / norm(w));
                let a = dot(*d, w).atan2(dot(*d, u)).clamp(0., th);
                phis.push(from + a);
            }
        }
        if phis.is_empty() {
            return None;
        }
        let half = std::f64::consts::FRAC_PI_2;
        let allowed = |phi: f64| {
            phis.iter().all(|&s| {
                let d = (phi - s).abs();
                let d = if closed { d.min(total - d) } else { d };
                d >= half - 1e-12
            })
        };
        // The allowed sub-intervals of [0, total]: breakpoints at s +- a
        // right angle (wrapped on a closed fan), tested at midpoints.
        let mut cuts = vec![0., total];
        for &s in &phis {
            for c in [s - half, s + half] {
                let c = if closed { c.rem_euclid(total) } else { c };
                if (0. ..=total).contains(&c) {
                    cuts.push(c);
                }
            }
        }
        cuts.sort_by(f64::total_cmp);
        cuts.dedup();
        let spans: Vec<(f64, f64)> = cuts
            .windows(2)
            .filter(|w| allowed((w[0] + w[1]) / 2.))
            .map(|w| (w[0], w[1]))
            .collect();
        let mut out = vec![];
        for &(g, e_in, e_out, from, th) in &fan {
            // Edges through the vertex: their whole length, where their
            // direction is allowed.
            if allowed(from) {
                out.push((g, e_in, 0., topo.len[e_in as usize]));
            }
            if allowed(from + th) {
                out.push((g, e_out, 0., topo.len[e_out as usize]));
            }
            // The opposite edge: the rays bounding each allowed span.
            let Some(&e_opp) = topo.tri_edges[g as usize]
                .iter()
                .find(|&&e| e != e_in && e != e_out)
            else {
                continue;
            };
            let (ia, ib) = (far_end(e_in), far_end(e_out));
            let (a3, b3) = (self.v[ia as usize], self.v[ib as usize]);
            let (u, o) = (dir(e_in), dir(e_out));
            let w = sub(o, scale(u, dot(o, u)));
            let w = scale(w, 1. / norm(w));
            let to2 = |x: Vec3| [dot(sub(x, q), u), dot(sub(x, q), w)];
            let (a2, b2) = (to2(a3), to2(b3));
            let l = topo.len[e_opp as usize];
            // Where the ray at fan angle phi meets A -> B, as the edge's
            // parameter from its first vertex.
            let hit = |phi: f64| -> f64 {
                let (c, sn) = ((phi - from).cos(), (phi - from).sin());
                let (ex, ey) = (b2[0] - a2[0], b2[1] - a2[1]);
                let den = c * ey - sn * ex;
                let sp = if den.abs() > 0. {
                    ((a2[1] * c - a2[0] * sn) / -den).clamp(0., 1.)
                } else {
                    0.
                };
                if topo.edges[e_opp as usize][0] == ia {
                    sp * l
                } else {
                    (1. - sp) * l
                }
            };
            for &(s0, s1) in &spans {
                let (lo, hi) = (s0.max(from), s1.min(from + th));
                if hi - lo > 1e-12 {
                    let (x0, x1) = (hit(lo), hit(hi));
                    out.push((g, e_opp, x0.min(x1), x0.max(x1)));
                }
            }
        }
        Some(out)
    }

    /// A segment in face `f` (its ends are point sources too): linear
    /// windows from its interior (Bommes and Kobbelt 2007, figure 6a): on
    /// each edge, the points whose orthogonal foot on the segment lies
    /// within it, on the side the wave leaves the face.
    fn source_segment(&mut self, f: u32, a: Vec3, b: Vec3) {
        self.in_face
            .entry(f)
            .or_default()
            .push(Piece::Segment(f, a, b));
        for k in 0..3 {
            let e = self.topo.tri_edges[f as usize][k];
            let l = self.topo.len[e as usize];
            let (ax, ay) = self.coords(e, f, a);
            let (bx, by) = self.coords(e, f, b);
            // The window frame: y positive away from f.
            let (ay, by) = (-ay.max(0.), -by.max(0.));
            let (dx, dy) = (bx - ax, by - ay);
            let ll = dx * dx + dy * dy;
            if ll <= (1e-12 * l).powi(2) {
                continue;
            }
            let sl = ll.sqrt();
            for (nx, ny) in [(-dy / sl, dx / sl), (dy / sl, -dx / sl)] {
                if ny <= 1e-12 {
                    continue;
                }
                // Foot parameter along the segment for (x, 0): linear in x.
                let (mut lo, mut hi) = (0f64, l);
                let base = -ax * dx - ay * dy;
                if dx.abs() > 1e-15 * sl {
                    let (x0, x1) = (-base / dx, (ll - base) / dx);
                    lo = lo.max(x0.min(x1));
                    hi = hi.min(x0.max(x1));
                } else if !(0. ..=ll).contains(&base) {
                    continue;
                }
                // The side of the segment's line the normal points to.
                if nx > 0. {
                    lo = lo.max(ax + ay * ny / nx);
                } else if nx < 0. {
                    hi = hi.min(ax + ay * ny / nx);
                }
                if hi - lo <= 1e-12 * l {
                    continue;
                }
                self.add(Win {
                    edge: e,
                    into: self.topo.other(e, f),
                    b0: lo,
                    b1: hi,
                    src: Src::Line {
                        ux: nx,
                        uy: ny,
                        c: -(ax * nx + ay * ny),
                    },
                    alive: true,
                    low: 0.,
                    queued: true,
                });
            }
        }
    }

    /// A pseudosource at vertex `vx` (2005 section 3.2): windows from it
    /// over every face around it, at distance `d`.
    fn emit(&mut self, vx: u32, d: f64) {
        let p = self.v[vx as usize];
        for fi in 0..self.topo.vertex_faces[vx as usize].len() {
            let f = self.topo.vertex_faces[vx as usize][fi];
            for k in 0..3 {
                let e = self.topo.tri_edges[f as usize][k];
                let [a, b] = self.topo.edges[e as usize];
                let l = self.topo.len[e as usize];
                // Along the edges at the vertex the distance through it is
                // d + |x - x_v|: every evaluation takes that vertex cone, so
                // no window is needed there.
                if a == vx || b == vx {
                    continue;
                } else {
                    let (x, y) = self.coords(e, f, p);
                    self.add(Win {
                        edge: e,
                        into: self.topo.other(e, f),
                        b0: 0.,
                        b1: l,
                        src: Src::Point {
                            x,
                            y: (-y).min(0.),
                            s: d,
                        },
                        alive: true,
                        low: 0.,
                        queued: true,
                    });
                }
            }
        }
    }

    /// Window propagation across its face (2005 section 3.2; linear windows
    /// with parallel rays, 2007 section 4.2.1).
    fn propagate(&mut self, w: Win) {
        let (e, g) = (w.edge, w.into);
        let [a, b] = self.topo.edges[e as usize];
        let l = self.topo.len[e as usize];
        let c = self.third(e, g);
        let (cx, cy) = self.coords(e, g, self.v[c as usize]);
        if cy <= 0. {
            return;
        }
        // The ray through the opposite vertex splits the window.
        let xc = match w.src {
            Src::Point { x: sx, y: sy, .. } => {
                if sy > -1e-12 * l {
                    return;
                }
                sx + (cx - sx) * (-sy) / (cy - sy)
            }
            Src::Line { ux, uy, .. } => {
                if uy < 1e-12 {
                    return;
                }
                cx - ux * cy / uy
            }
        };
        let pos = |k: u32| {
            if k == a {
                [0., 0.]
            } else if k == b {
                [l, 0.]
            } else {
                [cx, cy]
            }
        };
        let te = self.topo.tri_edges[g as usize];
        for (side, r0, r1) in [(a, w.b0, w.b1.min(xc)), (b, w.b0.max(xc), w.b1)] {
            if r1 - r0 <= 1e-12 * l {
                continue;
            }
            let ne = *te
                .iter()
                .find(|&&x| {
                    let ends = self.topo.edges[x as usize];
                    ends.contains(&side) && ends.contains(&c)
                })
                .unwrap();
            let [p, q] = self.topo.edges[ne as usize];
            let ln = self.topo.len[ne as usize];
            let (o, qq) = (pos(p), pos(q));
            let xh = [(qq[0] - o[0]) / ln, (qq[1] - o[1]) / ln];
            let mut yh = [-xh[1], xh[0]];
            let rest = pos(if side == a { b } else { a });
            if (rest[0] - o[0]) * yh[0] + (rest[1] - o[1]) * yh[1] > 0. {
                yh = [-yh[0], -yh[1]];
            }
            let map = |pt: [f64; 2]| {
                let d = [pt[0] - o[0], pt[1] - o[1]];
                [d[0] * xh[0] + d[1] * xh[1], d[0] * yh[0] + d[1] * yh[1]]
            };
            let (src, image): (Src, [f64; 2]) = match w.src {
                Src::Point { x: sx, y: sy, s } => {
                    let sn = map([sx, sy]);
                    let img = [r0, r1].map(|x| {
                        let xn = map([x, 0.]);
                        let k = xn[1] - sn[1];
                        if k <= 0. {
                            if xn[0] >= sn[0] {
                                f64::INFINITY
                            } else {
                                f64::NEG_INFINITY
                            }
                        } else {
                            sn[0] + (xn[0] - sn[0]) * (-sn[1]) / k
                        }
                    });
                    (
                        Src::Point {
                            x: sn[0],
                            y: sn[1].min(0.),
                            s,
                        },
                        img,
                    )
                }
                Src::Line { ux, uy, c } => {
                    let un = [ux * xh[0] + uy * xh[1], ux * yh[0] + uy * yh[1]];
                    let img = [r0, r1].map(|x| {
                        let xn = map([x, 0.]);
                        if un[1] <= 0. {
                            if un[0] >= 0. {
                                f64::INFINITY
                            } else {
                                f64::NEG_INFINITY
                            }
                        } else {
                            xn[0] + un[0] * (-xn[1]) / un[1]
                        }
                    });
                    (
                        Src::Line {
                            ux: un[0],
                            uy: un[1].max(0.),
                            c: c + ux * o[0] + uy * o[1],
                        },
                        img,
                    )
                }
            };
            let (lo, hi) = (image[0].min(image[1]), image[0].max(image[1]));
            self.add(Win {
                edge: ne,
                into: self.topo.other(ne, g),
                b0: lo.max(0.),
                b1: hi.min(ln),
                src,
                alive: true,
                low: 0.,
                queued: true,
            });
        }
    }

    /// The sub-intervals of [lo, hi] where `p` is strictly shorter than `q`.
    fn better(p: &Src, q: &Src, lo: f64, hi: f64) -> Vec<(f64, f64)> {
        let g = |x: f64| {
            let (dp, dq) = (p.edge(x), q.edge(x));
            if dp < dq - 1e-10 * (1. + dq) {
                -1
            } else if dp > dq + 1e-10 * (1. + dq) {
                1
            } else {
                0
            }
        };
        let mut cuts = vec![];
        ties(p, q, &mut cuts);
        let mut pts = vec![lo];
        pts.extend(cuts.into_iter().filter(|&x| x > lo && x < hi));
        pts.push(hi);
        pts.sort_by(f64::total_cmp);
        // A strict sign change left without a root (rounding near a double
        // root) is found by bisection: the difference is continuous.
        let mut k = 0;
        while k + 1 < pts.len() {
            let (x0, x1) = (pts[k], pts[k + 1]);
            let m = (x0 + x1) / 2.;
            let (s0, sm, s1) = (g(x0), g(m), g(x1));
            let change = |a: i32, b: i32| a * b < 0;
            if x1 > x0 && (change(s0, sm) || change(sm, s1)) {
                let (mut a, mut b) = if change(s0, sm) { (x0, m) } else { (m, x1) };
                let sa = g(a);
                for _ in 0..60 {
                    let c = (a + b) / 2.;
                    if g(c) == sa {
                        a = c;
                    } else {
                        b = c;
                    }
                }
                let c = (a + b) / 2.;
                if c > x0 && c < x1 {
                    pts.insert(k + 1, c);
                    continue;
                }
            }
            k += 1;
        }
        let mut out: Vec<(f64, f64)> = vec![];
        for k in 0..pts.len() - 1 {
            let (x0, x1) = (pts[k], pts[k + 1]);
            if x1 <= x0 {
                continue;
            }
            let m = (x0 + x1) / 2.;
            let (dp, dq) = (p.edge(m), q.edge(m));
            if dp < dq - 1e-10 * (1. + dq) {
                match out.last_mut() {
                    Some(last) if last.1 == x0 => last.1 = x1,
                    _ => out.push((x0, x1)),
                }
            }
        }
        out
    }

    fn touch(&mut self, vx: u32, d: f64) {
        if d < self.vertex[vx as usize] {
            self.vertex[vx as usize] = d;
            if self.topo.pseudo[vx as usize] && d <= self.limit {
                self.queue.push(Event(d, vx, true));
            }
        }
    }

    /// Insert a window, keeping on its edge only where each is strictly
    /// shortest (2005 section 3.3): the new window loses where an existing
    /// one is no longer, and trims existing ones where it is shorter.
    fn add(&mut self, w: Win) {
        let e = w.edge as usize;
        let l = self.topo.len[e];
        let tiny = 1e-11 * l;
        if w.b1 - w.b0 <= tiny || w.src.lowest(w.b0, w.b1) > self.limit {
            return;
        }
        // The edge's live windows are disjoint and kept in order along it:
        // only those overlapping [b0, b1] are compared (a binary search, then
        // the run of them), and the run is replaced by what survives.
        let list = &self.on_edge[e];
        let mut i0 = list.partition_point(|&i| self.wins[i as usize].b0 < w.b0);
        while i0 > 0 && self.wins[list[i0 - 1] as usize].b1 > w.b0 - tiny {
            i0 -= 1;
        }
        let mut i1 = i0;
        while i1 < list.len() && self.wins[list[i1] as usize].b0 < w.b1 + tiny {
            i1 += 1;
        }
        let run: Vec<u32> = list[i0..i1].to_vec();
        let mut survivors: Vec<u32> = Vec::with_capacity(run.len() + 2);
        let mut keep: Vec<(f64, f64)> = vec![(w.b0, w.b1)];
        for id in run {
            let u = self.wins[id as usize];
            let (lo, hi) = (u.b0.max(w.b0), u.b1.min(w.b1));
            if hi - lo <= tiny {
                survivors.push(id);
                continue;
            }
            let better = Self::better(&w.src, &u.src, lo, hi);
            // The new window keeps only `better` within [lo, hi].
            let mut next = vec![];
            for &(k0, k1) in &keep {
                let (o0, o1) = (k0.max(lo), k1.min(hi));
                if o1 <= o0 {
                    next.push((k0, k1));
                    continue;
                }
                if k0 < o0 {
                    next.push((k0, o0));
                }
                for &(g0, g1) in &better {
                    let (c0, c1) = (g0.max(o0), g1.min(o1));
                    if c1 > c0 {
                        next.push((c0, c1));
                    }
                }
                if o1 < k1 {
                    next.push((o1, k1));
                }
            }
            keep = next;
            if better.is_empty() {
                survivors.push(id);
                continue;
            }
            // The existing window loses `better`.
            let mut pieces = vec![];
            let mut at = u.b0;
            for &(g0, g1) in &better {
                if g0 - at > tiny {
                    pieces.push((at, g0));
                }
                at = at.max(g1);
            }
            if u.b1 - at > tiny {
                pieces.push((at, u.b1));
            }
            match pieces.split_first() {
                None => self.wins[id as usize].alive = false,
                Some((&(p0, p1), rest)) => {
                    self.wins[id as usize].b0 = p0;
                    self.wins[id as usize].b1 = p1;
                    survivors.push(id);
                    for &(r0, r1) in rest {
                        let nid = self.wins.len() as u32;
                        self.wins.push(Win {
                            b0: r0,
                            b1: r1,
                            ..u
                        });
                        survivors.push(nid);
                        if u.queued {
                            self.queue.push(Event(u.src.lowest(r0, r1), nid, false));
                        }
                    }
                }
            }
        }
        let [a, b] = self.topo.edges[e];
        for (k0, k1) in keep {
            if k1 - k0 <= tiny {
                continue;
            }
            let id = self.wins.len() as u32;
            let nw = Win {
                b0: k0,
                b1: k1,
                ..w
            };
            self.wins.push(nw);
            survivors.push(id);
            if nw.queued {
                self.queue.push(Event(nw.src.lowest(k0, k1), id, false));
            }
            if k0 <= 1e-9 * l {
                self.touch(a, nw.eval(0., 0.));
            }
            if k1 >= l - 1e-9 * l {
                self.touch(b, nw.eval(l, 0.));
            }
        }
        let wins = &self.wins;
        survivors.sort_by(|&i, &j| wins[i as usize].b0.total_cmp(&wins[j as usize].b0));
        self.on_edge[e].splice(i0..i1, survivors);
    }

    /// The distance at parameter x along edge `e`.
    pub fn edge_at(&self, e: u32, x: f64) -> f64 {
        let [a, b] = self.topo.edges[e as usize];
        let l = self.topo.len[e as usize];
        let mut d = (self.vertex[a as usize] + x).min(self.vertex[b as usize] + l - x);
        // The window covering x is least there (a farther window's value,
        // continued at slope 1 from its end, is no less); in a gap between
        // two, the least of their continuations.
        // Every window reaching x (they end at or after it, just before the
        // first one starting past it; several where pieces meet), the last
        // one before and the first one after.
        let list = &self.on_edge[e as usize];
        let i = list.partition_point(|&id| self.wins[id as usize].b0 <= x);
        if let Some(&id) = list.get(i) {
            d = d.min(self.wins[id as usize].eval(x, 0.));
        }
        for j in (0..i).rev() {
            let w = &self.wins[list[j] as usize];
            d = d.min(w.eval(x, 0.));
            if w.b1 < x {
                break;
            }
        }
        d
    }

    /// The distance at a point `p` of face `f` (its closure).
    pub fn at(&self, f: u32, p: Vec3) -> f64 {
        let mut d = f64::INFINITY;
        for &k in &self.t[f as usize] {
            d = d.min(norm(sub(p, self.v[k as usize])) + self.vertex[k as usize]);
        }
        for &e in &self.topo.tri_edges[f as usize] {
            let (x, y) = self.coords(e, f, p);
            let y = y.max(0.);
            for &id in &self.on_edge[e as usize] {
                let w = &self.wins[id as usize];
                // No path through the window is shorter than the distance
                // to the edge plus the window's least value.
                if w.alive && y + w.low < d {
                    d = d.min(w.eval(x, y));
                }
            }
        }
        if let Some(pieces) = self.in_face.get(&f) {
            for piece in pieces {
                d = d.min(match *piece {
                    Piece::Point(_, q) => norm(sub(p, q)),
                    Piece::Segment(_, a, b) => {
                        let ab = sub(b, a);
                        let s = (dot(sub(p, a), ab) / dot(ab, ab).max(1e-300)).clamp(0., 1.);
                        norm(sub(p, axpy(a, s, ab)))
                    }
                });
            }
        }
        d
    }

    /// The crossings of `level` along edge `e`, as fractions of its length
    /// from its first vertex, exact (roots of its windows).
    pub fn edge_crossings(&self, e: u32, level: f64) -> Vec<f64> {
        let [a, b] = self.topo.edges[e as usize];
        let l = self.topo.len[e as usize];
        let (da, db) = (self.vertex[a as usize], self.vertex[b as usize]);
        let mut c = vec![(level - da) / l, 1. - (level - db) / l];
        for &id in &self.on_edge[e as usize] {
            let w = &self.wins[id as usize];
            if !w.alive {
                continue;
            }
            match w.src {
                Src::Point { x, y, s } if level >= s => {
                    let r2 = (level - s).powi(2) - y * y;
                    if r2 >= 0. {
                        for r in [x - r2.sqrt(), x + r2.sqrt()] {
                            if r >= w.b0 && r <= w.b1 {
                                c.push(r / l);
                            }
                        }
                    }
                }
                Src::Line { ux, c: k, .. } if ux != 0. => {
                    let r = (level - k) / ux;
                    if r >= w.b0 && r <= w.b1 {
                        c.push(r / l);
                    }
                }
                _ => {}
            }
            let (f0, f1) = (w.src.edge(w.b0), w.src.edge(w.b1));
            c.push((w.b0 - (level - f0)) / l);
            c.push((w.b1 + (level - f1)) / l);
        }
        crossings(c, da >= level, db >= level, |t| {
            self.edge_at(e, t * l) >= level
        })
    }

    /// `functions`, each with the region of the face where it is realized
    /// (up to three half-planes n . p >= c; none: everywhere). A window's
    /// function holds only where its rays cross its own interval of the
    /// edge (MMP 1987): the wedge from its unfolded pseudosource through the
    /// interval, or the strip of a linear wave's parallel rays through it;
    /// outside, its formula can lie far below the distance (no path of that
    /// length exists), so it is no competitor there. A line of a segment
    /// source in the face holds where the segment's foot is within it, on
    /// its side. Vertex cones and point sources are path lengths
    /// everywhere. Pieces of a window split by trimming keep their own
    /// regions.
    fn regions(&self, f: u32) -> Vec<(Func, Region)> {
        let mut out = vec![];
        for &k in &self.t[f as usize] {
            if self.topo.pseudo[k as usize] && self.vertex[k as usize].is_finite() {
                out.push((
                    Func::Circle(self.v[k as usize], self.vertex[k as usize]),
                    [None; 3],
                ));
            }
        }
        for (i, &e) in self.topo.tri_edges[f as usize].iter().enumerate() {
            let o = self.v[self.topo.edges[e as usize][0] as usize];
            let (xh, yh) = (self.topo.xhat[e as usize], self.topo.yhat[f as usize][i]);
            let l = self.topo.len[e as usize];
            // A half-plane n . (x, y) >= c of the edge's frame, in 3D.
            let half = |n: [f64; 2], c: f64| -> Option<(Vec3, f64)> {
                let n3 = axpy(scale(xh, n[0]), n[1], yh);
                Some((n3, c + dot(n3, o)))
            };
            for &id in &self.on_edge[e as usize] {
                let w = &self.wins[id as usize];
                if !w.alive || w.into != f {
                    continue;
                }
                out.push(match w.src {
                    Src::Point { x, y, s } => {
                        let func = Func::Circle(axpy(axpy(o, x, xh), y, yh), s);
                        // The wedge between the rays from (x, y) through
                        // the interval's ends (none for a source on the
                        // edge's line).
                        let region = if y < -1e-12 * l {
                            let (a, b) = ([w.b0 - x, -y], [w.b1 - x, -y]);
                            let na = [a[1], -a[0]];
                            let nb = [-b[1], b[0]];
                            [
                                half(na, na[0] * x + na[1] * y),
                                half(nb, nb[0] * x + nb[1] * y),
                                None,
                            ]
                        } else {
                            [None; 3]
                        };
                        (func, region)
                    }
                    Src::Line { ux, uy, c: k } => {
                        let func = Func::Line(o, axpy(scale(xh, ux), uy, yh), k);
                        // The strip of rays back through [b0, b1]: the
                        // foot x - y ux / uy within it.
                        let region = if uy > 1e-12 {
                            let r = ux / uy;
                            [half([1., -r], w.b0), half([-1., r], -w.b1), None]
                        } else {
                            [None; 3]
                        };
                        (func, region)
                    }
                });
            }
        }
        // A polyline vertex in f: where it is behind its segments (at most
        // three of them: a superset where it has more).
        let behind = |q: Vec3| -> Region {
            let mut r: Region = [None; 3];
            if let Some(ds) = self.behind.get(&(f, q.map(f64::to_bits))) {
                for (slot, d) in r.iter_mut().zip(ds) {
                    *slot = Some((scale(*d, -1.), -dot(*d, q)));
                }
            }
            r
        };
        if let Some(pieces) = self.in_face.get(&f) {
            for piece in pieces {
                match *piece {
                    Piece::Point(_, q) => out.push((Func::Circle(q, 0.), behind(q))),
                    Piece::Segment(g, a, b) => {
                        out.push((Func::Circle(a, 0.), behind(a)));
                        out.push((Func::Circle(b, 0.), behind(b)));
                        let ab = sub(b, a);
                        let m = cross(face_normal(self.v, self.t, g), ab);
                        let (lm, lab) = (norm(m), norm(ab));
                        if lm > 0. && lab > 0. {
                            let (m, d) = (scale(m, 1. / lm), scale(ab, 1. / lab));
                            let foot = [Some((d, dot(d, a))), Some((scale(d, -1.), -dot(d, b)))];
                            for side in [m, scale(m, -1.)] {
                                out.push((
                                    Func::Line(a, side, 0.),
                                    [foot[0], foot[1], Some((side, dot(side, a)))],
                                ));
                            }
                        }
                    }
                }
            }
        }
        let mut seen = std::collections::HashSet::new();
        out.retain(|(func, region)| {
            let r = region
                .map(|h| h.map_or([0u64; 4], |(n, c)| [n[0], n[1], n[2], c].map(f64::to_bits)));
            seen.insert((func_key(func), r))
        });
        out
    }

    /// The sources' points in face `f` (where the distance is zero: the
    /// only local minima).
    pub fn valleys(&self, f: u32) -> Vec<Vec3> {
        self.in_face.get(&f).map_or(vec![], |pieces| {
            pieces
                .iter()
                .flat_map(|piece| match *piece {
                    Piece::Point(_, q) => vec![q],
                    Piece::Segment(_, a, b) => vec![a, b, scale(axpy(a, 1., b), 0.5)],
                })
                .collect()
        })
    }
}

/// One face's distance, prepared once for many queries (contour tracing
/// asks for thousands of crossings and values on the same face): its
/// functions with their regions, and its indexed evaluator. The same values
/// as `Field::at` and `Field::crossings_on`.
pub(super) struct FaceQuery<'f> {
    funcs: Vec<(Func, Region)>,
    eval: FaceEval<'f>,
}

impl<'f> FaceQuery<'f> {
    pub(super) fn new(field: &'f Field<'f>, f: u32) -> Self {
        // Pieces of one window split by trimming share its function; keep
        // each function once, with the union of its regions (any of them).
        FaceQuery {
            funcs: field.regions(f),
            eval: FaceEval::new(field, f),
        }
    }

    pub(super) fn at(&self, p: Vec3) -> f64 {
        self.eval.at(p)
    }

    /// `Field::crossings_on`: only the functions realized somewhere on the
    /// segment (its part inside the function's region) can cross the
    /// level there.
    pub(super) fn crossings_on(
        &self,
        p0: Vec3,
        p1: Vec3,
        level: f64,
        s0: bool,
        s1: bool,
    ) -> Vec<f64> {
        let d = sub(p1, p0);
        let dd = dot(d, d);
        let slack = 1e-9 * (1. + dd.sqrt());
        let mut c = vec![];
        for (func, region) in &self.funcs {
            // The segment's parameters inside the region n . p >= c.
            let (mut t0, mut t1) = (0f64, 1f64);
            for &(n, k) in region.iter().flatten() {
                let (a, b) = (dot(n, p0) - k + slack, dot(n, d));
                if b > 0. {
                    t0 = t0.max(-a / b);
                } else if b < 0. {
                    t1 = t1.min(-a / b);
                } else if a < 0. {
                    t1 = -1.;
                }
            }
            if t0 > t1 {
                continue;
            }
            match *func {
                Func::Circle(center, sigma) => {
                    if level >= sigma {
                        let w = sub(p0, center);
                        roots(
                            dd,
                            2. * dot(d, w),
                            dot(w, w) - (level - sigma).powi(2),
                            &mut c,
                        );
                    }
                }
                Func::Line(origin, u, sigma) => {
                    let k = dot(d, u);
                    if k != 0. {
                        c.push((level - sigma - dot(sub(p0, origin), u)) / k);
                    }
                }
            }
        }
        crossings(c, s0, s1, |t| self.eval.at(axpy(p0, t, d)) >= level)
    }
}

/// The triangle `cell` clipped to the half-planes n . p >= c - slack of
/// region `r` (Sutherland and Hodgman 1974): a convex polygon of at most
/// six vertices, and their number (0: they miss each other).
fn clip_cell(
    cell: &[[f64; 2]; 3],
    r: &[Option<([f64; 2], f64)>; 3],
    slack: f64,
) -> ([[f64; 2]; 6], usize) {
    let mut poly = [[0.; 2]; 6];
    poly[..3].copy_from_slice(cell);
    let mut n = 3usize;
    for &(nv, c) in r.iter().flatten() {
        if n == 0 {
            break;
        }
        let mut next = [[0.; 2]; 6];
        let mut m = 0usize;
        let side = |p: [f64; 2]| nv[0] * p[0] + nv[1] * p[1] - (c - slack);
        let mut prev = poly[n - 1];
        let mut dp = side(prev);
        for &cur in &poly[..n] {
            let dc = side(cur);
            if (dp >= 0.) != (dc >= 0.) && m < 6 {
                let t = dp / (dp - dc);
                next[m] = [
                    prev[0] + t * (cur[0] - prev[0]),
                    prev[1] + t * (cur[1] - prev[1]),
                ];
                m += 1;
            }
            if dc >= 0. && m < 6 {
                next[m] = cur;
                m += 1;
            }
            (prev, dp) = (cur, dc);
        }
        poly = next;
        n = m;
    }
    (poly, n)
}

/// The distance from `c` to a convex polygon (zero inside it).
fn dist_to_polygon(c: [f64; 2], poly: &[[f64; 2]]) -> f64 {
    let n = poly.len();
    if n == 1 {
        return (c[0] - poly[0][0]).hypot(c[1] - poly[0][1]);
    }
    // Inside: on the same side of every edge (either orientation).
    let cross =
        |a: [f64; 2], b: [f64; 2]| (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
    let signs: Vec<f64> = (0..n).map(|i| cross(poly[i], poly[(i + 1) % n])).collect();
    if n >= 3 && (signs.iter().all(|&x| x >= 0.) || signs.iter().all(|&x| x <= 0.)) {
        return 0.;
    }
    (0..n)
        .map(|i| {
            let (a, b) = (poly[i], poly[(i + 1) % n]);
            let ab = [b[0] - a[0], b[1] - a[1]];
            let ll = ab[0] * ab[0] + ab[1] * ab[1];
            let t = if ll > 0. {
                (((c[0] - a[0]) * ab[0] + (c[1] - a[1]) * ab[1]) / ll).clamp(0., 1.)
            } else {
                0.
            };
            (c[0] - a[0] - t * ab[0]).hypot(c[1] - a[1] - t * ab[1])
        })
        .fold(f64::INFINITY, f64::min)
}

/// The distance from `p` to a source piece.
fn piece_dist(piece: &Piece, p: Vec3) -> f64 {
    match *piece {
        Piece::Point(_, a) => norm(sub(p, a)),
        Piece::Segment(_, a, b) => {
            let ab = sub(b, a);
            let s = (dot(sub(p, a), ab) / dot(ab, ab).max(1e-300)).clamp(0., 1.);
            norm(sub(p, axpy(a, s, ab)))
        }
    }
}

/// Exact evaluation of one field at many points of one face: its vertex
/// cones; its edges' windows in order of their least value (a window gives
/// at least the distance to its edge plus that, so a scan stops once that
/// exceeds the best found); its sources in the face in a grid over the
/// face's plane, searched outward ring by ring until no farther cell can
/// be nearer.
pub(super) struct FaceEval<'f> {
    field: &'f Field<'f>,
    f: u32,
    /// Each edge's live windows in order along it (disjoint).
    along: Vec<(u32, Vec<u32>)>,
    o: Vec3,
    ex: Vec3,
    ey: Vec3,
    lo: [f64; 2],
    h: f64,
    n: [usize; 2],
    cells: Vec<Vec<u32>>,
    pieces: Vec<Piece>,
}

impl<'f> FaceEval<'f> {
    fn new(field: &'f Field<'f>, f: u32) -> Self {
        let along = field.topo.tri_edges[f as usize]
            .iter()
            .map(|&e| {
                let mut ids: Vec<u32> = field.on_edge[e as usize]
                    .iter()
                    .copied()
                    .filter(|&i| field.wins[i as usize].alive)
                    .collect();
                ids.sort_by(|&a, &b| {
                    field.wins[a as usize]
                        .b0
                        .total_cmp(&field.wins[b as usize].b0)
                });
                (e, ids)
            })
            .collect();
        let tri = field.t[f as usize].map(|k| field.v[k as usize]);
        let o = tri[0];
        let ex = scale(sub(tri[1], o), 1. / norm(sub(tri[1], o)));
        let ey = cross(face_normal(field.v, field.t, f), ex);
        let to2 = |q: Vec3| [dot(sub(q, o), ex), dot(sub(q, o), ey)];
        let pieces: Vec<Piece> = field.in_face.get(&f).cloned().unwrap_or_default();
        let corners = tri.map(to2);
        let lo = [0, 1].map(|k| corners.iter().map(|c| c[k]).fold(f64::INFINITY, f64::min));
        let hi = [0, 1].map(|k| {
            corners
                .iter()
                .map(|c| c[k])
                .fold(f64::NEG_INFINITY, f64::max)
        });
        let (w, ht) = ((hi[0] - lo[0]).max(1e-12), (hi[1] - lo[1]).max(1e-12));
        let h = ((w * ht) / pieces.len().max(1) as f64)
            .sqrt()
            .max(w.max(ht) / 4096.);
        let n = [
            ((w / h).ceil() as usize).max(1),
            ((ht / h).ceil() as usize).max(1),
        ];
        let mut cells = vec![vec![]; n[0] * n[1]];
        let cell = |q: [f64; 2]| {
            [0, 1].map(|k| (((q[k] - lo[k]) / h).floor().max(0.) as usize).min(n[k] - 1))
        };
        for (i, piece) in pieces.iter().enumerate() {
            let (a, b) = match *piece {
                Piece::Point(_, q) => (to2(q), to2(q)),
                Piece::Segment(_, a, b) => (to2(a), to2(b)),
            };
            let (c0, c1) = (
                cell([a[0].min(b[0]), a[1].min(b[1])]),
                cell([a[0].max(b[0]), a[1].max(b[1])]),
            );
            for x in c0[0]..=c1[0] {
                for y in c0[1]..=c1[1] {
                    cells[y * n[0] + x].push(i as u32);
                }
            }
        }
        FaceEval {
            field,
            f,
            along,
            o,
            ex,
            ey,
            lo,
            h,
            n,
            cells,
            pieces,
        }
    }

    /// Lower `best` by `value(piece)` over the pieces whose distance to `p`
    /// is below it (rings of cells outward from p's, until none can be).
    fn near(&self, p: Vec3, best: &mut f64, value: impl Fn(&Piece) -> f64) {
        if self.pieces.is_empty() {
            return;
        }
        let q = [dot(sub(p, self.o), self.ex), dot(sub(p, self.o), self.ey)];
        let c = [0, 1].map(|k| {
            (((q[k] - self.lo[k]) / self.h).floor() as i64).clamp(0, self.n[k] as i64 - 1)
        });
        let outside = [0, 1]
            .map(|k| {
                (self.lo[k] - q[k])
                    .max(q[k] - (self.lo[k] + self.n[k] as f64 * self.h))
                    .max(0.)
            })
            .iter()
            .fold(0., |a: f64, b| a.max(*b));
        let (nx, ny) = (self.n[0] as i64, self.n[1] as i64);
        let reach = nx.max(ny);
        let visit = |x: i64, y: i64, best: &mut f64| {
            for &i in &self.cells[y as usize * self.n[0] + x as usize] {
                // A piece in several cells may be valued twice: harmless.
                *best = best.min(value(&self.pieces[i as usize]));
            }
        };
        for ring in 0..=reach {
            // Cells at ring k are at least (k - 1) h away (plus the point's
            // distance outside the grid's box).
            if (ring as f64 - 1.).max(0.) * self.h + outside >= *best {
                break;
            }
            // The ring's cells that exist: its top and bottom rows whole,
            // its two side columns between.
            let (x0, x1) = ((c[0] - ring).max(0), (c[0] + ring).min(nx - 1));
            for y in [c[1] - ring, c[1] + ring] {
                if (0..ny).contains(&y) {
                    for x in x0..=x1 {
                        visit(x, y, best);
                    }
                }
                if ring == 0 {
                    break;
                }
            }
            if ring > 0 {
                for y in (c[1] - ring + 1).max(0)..=(c[1] + ring - 1).min(ny - 1) {
                    for x in [c[0] - ring, c[0] + ring] {
                        if (0..nx).contains(&x) {
                            visit(x, y, best);
                        }
                    }
                }
            }
            if c[0] - ring <= 0
                && c[0] + ring >= nx - 1
                && c[1] - ring <= 0
                && c[1] + ring >= ny - 1
            {
                break;
            }
        }
    }

    fn at(&self, p: Vec3) -> f64 {
        let field = self.field;
        let mut d = f64::INFINITY;
        for &k in &field.t[self.f as usize] {
            d = d.min(norm(sub(p, field.v[k as usize])) + field.vertex[k as usize]);
        }
        // The sources in the face first: usually the nearest, they bound
        // the windows' scan sooner.
        self.near(p, &mut d, |piece| piece_dist(piece, p));
        // A window gives at least the point's distance to its interval plus
        // its least value: outward from the point's place along the edge,
        // until the gap alone is no nearer.
        for (e, ids) in &self.along {
            let (x, y) = field.coords(*e, self.f, p);
            let y = y.max(0.);
            let i = ids.partition_point(|&id| field.wins[id as usize].b0 <= x);
            for &id in ids[..i].iter().rev() {
                let w = &field.wins[id as usize];
                let gap = (x - w.b1).max(0.).hypot(y);
                if gap >= d {
                    break;
                }
                if gap + w.low < d {
                    d = d.min(w.eval(x, y));
                }
            }
            for &id in &ids[i..] {
                let w = &field.wins[id as usize];
                let gap = (w.b0 - x).max(0.).hypot(y);
                if gap >= d {
                    break;
                }
                if gap + w.low < d {
                    d = d.min(w.eval(x, y));
                }
            }
        }
        d
    }
}

/// A distance over the drive surface that the exact maxima are taken of:
/// one field, or the least of several (`Coverage`). It is the minimum of
/// finitely many convex functions (windows, vertex cones, sources), each
/// valid in its region.
pub(super) trait Distance {
    fn mesh(&self) -> (&[Vec3], &[[u32; 3]], &Topology);
    /// The distance at vertex `k`.
    fn vertex_at(&self, k: u32) -> f64;
    /// The distance at a point `p` of face `f`.
    fn at(&self, f: u32, p: Vec3) -> f64;
    /// `at` over face `f`, indexed for many points (the same values).
    fn face_eval(&self, f: u32) -> Vec<FaceEval<'_>>;
    /// The distance at parameter x along edge `e`.
    fn edge_at(&self, e: u32, x: f64) -> f64;
    /// The windows on edge `e`: (function, range).
    fn edge_windows(&self, e: u32) -> Vec<(Src, f64, f64)>;
    /// The distance at x along edge `e` and the convex piece realizing it
    /// there (a window's function, its continuation past its end, or a
    /// vertex cone, as a function of x along the edge).
    fn edge_piece_at(&self, e: u32, x: f64) -> (f64, Src);
    /// The circles and lines of face `f`, each with its region.
    fn regions(&self, f: u32) -> Vec<(Func, Region)>;

    /// The largest distance along edge `e` and where (its parameter),
    /// exactly. Along the edge the distance is the least of convex
    /// functions over the whole edge: each window's (its function on its
    /// range, continued at slope one beyond it, as `eval` does) and the
    /// two vertex cones. Between consecutive window ends each is a single
    /// convex piece; where one piece is least at both ends of such an
    /// interval the least cannot rise above its ends inside, and where the
    /// least piece changes, it changes where two pieces cross: that
    /// crossing is solved and the halves examined again. The peaks are the
    /// edge's ends, window ends and those crossings.
    fn edge_argmax(&self, e: u32) -> (f64, f64) {
        let (_, _, topo) = self.mesh();
        let l = topo.len[e as usize];
        let mut xs = vec![0., l];
        for (_, b0, b1) in self.edge_windows(e) {
            xs.push(b0.clamp(0., l));
            xs.push(b1.clamp(0., l));
        }
        xs.sort_by(f64::total_cmp);
        xs.dedup();
        let same = |p: &Src, q: &Src| match (*p, *q) {
            (Src::Point { x: a, y: b, s: c }, Src::Point { x: d, y: f, s: g }) => {
                a == d && b == f && c == g
            }
            (Src::Line { ux: a, c: b, .. }, Src::Line { ux: d, c: f, .. }) => a == d && b == f,
            _ => false,
        };
        let mut m = (f64::NEG_INFINITY, 0.);
        let take = |v: f64, x: f64, m: &mut (f64, f64)| {
            if v > m.0 {
                *m = (v, x);
            }
        };
        // Intervals to examine: (a, b, depth).
        let mut stack: Vec<(f64, f64, u32)> = xs.windows(2).map(|w| (w[0], w[1], 0)).collect();
        for &x in &xs {
            take(self.edge_at(e, x), x, &mut m);
        }
        while let Some((a, b, depth)) = stack.pop() {
            let (_, pa) = self.edge_piece_at(e, a);
            let (_, pb) = self.edge_piece_at(e, b);
            if same(&pa, &pb) || depth >= 64 || b - a <= 1e-12 * l {
                continue;
            }
            let mut cuts = vec![];
            ties(&pa, &pb, &mut cuts);
            for x in cuts
                .into_iter()
                .filter(|x| x.is_finite() && *x > a && *x < b)
            {
                take(self.edge_at(e, x), x, &mut m);
                stack.push((a, x, depth + 1));
                stack.push((x, b, depth + 1));
            }
        }
        m
    }

    fn edge_max(&self, e: u32) -> f64 {
        self.edge_argmax(e).0
    }

    /// The local maxima of the distance inside face `f` above its boundary
    /// maximum, exactly. Along the bisector of two of its circles or lines
    /// the minimum of the two rises towards the face's boundary (or is
    /// constant), so an interior maximum is a point where three are equal:
    /// with v their common value, the circles' pairwise differences and the
    /// lines are linear in (x, y, v), leaving one quadratic in v. Each
    /// solution inside the face is kept where `at` confirms the value.
    #[cfg(test)]
    fn face_peaks(&self, f: u32) -> Vec<(Vec3, f64)> {
        self.face_peaks_above(f, f64::NEG_INFINITY)
    }

    /// `face_peaks`, only those above `above` (the search is skipped where
    /// the Lipschitz bound shows none can be).
    fn face_peaks_above(&self, f: u32, above: f64) -> Vec<(Vec3, f64)> {
        let (v, t, topo) = self.mesh();
        let tri = t[f as usize].map(|k| v[k as usize]);
        let vals = t[f as usize].map(|k| self.vertex_at(k));
        let mut lb = vals.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        for &e in &topo.tri_edges[f as usize] {
            lb = lb.max(self.edge_max(e));
        }
        let r = |i: usize| (0..3).map(|j| norm(sub(tri[i], tri[j]))).fold(0., f64::max);
        let ub = (0..3).map(|i| vals[i] + r(i)).fold(f64::INFINITY, f64::min);
        let tol = 1e-9 * (1. + lb.abs());
        let lb = lb.max(above);
        if ub.partial_cmp(&(lb + tol)) != Some(std::cmp::Ordering::Greater) {
            return vec![];
        }
        // The face's plane frame.
        let o = tri[0];
        let ex = scale(sub(tri[1], o), 1. / norm(sub(tri[1], o)));
        let n = face_normal(v, t, f);
        let ey = cross(n, ex);
        let to2 = |q: Vec3| [dot(sub(q, o), ex), dot(sub(q, o), ey)];
        let to3 = |p: [f64; 2]| axpy(axpy(o, p[0], ex), p[1], ey);
        // One entry per function, with every region it holds in (pieces of
        // one wave or circle, split by trimming or along a straight pass).
        type R2 = [Option<([f64; 2], f64)>; 3];
        let mut with: Vec<(F2, Vec<R2>)> = vec![];
        let mut index: HashMap<(u8, [u64; 6]), usize> = HashMap::new();
        for (func, region) in self.regions(f) {
            let f2 = match func {
                Func::Circle(c, s) => F2::Circle(to2(c), s),
                Func::Line(origin, u, s) => {
                    let u2 = [dot(u, ex), dot(u, ey)];
                    let og = to2(origin);
                    F2::Line(u2, s - (og[0] * u2[0] + og[1] * u2[1]))
                }
            };
            // n . p >= c with p = o + x ex + y ey.
            let r2: R2 = region.map(|h| h.map(|(n, c)| ([dot(n, ex), dot(n, ey)], c - dot(n, o))));
            match index.get(&func_key(&func)) {
                Some(&i) => with[i].1.push(r2),
                None => {
                    index.insert(func_key(&func), with.len());
                    with.push((f2, vec![r2]));
                }
            }
        }
        // The search is local: cells of the face (exact corner values),
        // pruned where the Lipschitz bound cannot exceed the threshold; in
        // a cell only the functions whose least value over it is within its
        // bound can be the minimum there, and once few remain their triple
        // points in the cell are solved.
        let evals = self.face_eval(f);
        let at_ = |p: Vec3| evals.iter().map(|e| e.at(p)).fold(f64::INFINITY, f64::min);
        let mut out: Vec<(Vec3, f64)> = vec![];
        // Each cell carries the functions still possible in its parent (a
        // function is never needed in a cell whose parent ruled it out).
        let all: Vec<usize> = (0..with.len()).collect();
        let mut stack = vec![(tri.map(to2), vals, all)];
        let size = (0..3).map(r).fold(0., f64::max);
        while let Some((cell, d, parent)) = stack.pop() {
            let cr = |i: usize| {
                (0..3)
                    .map(|j| {
                        ((cell[i][0] - cell[j][0]).powi(2) + (cell[i][1] - cell[j][1]).powi(2))
                            .sqrt()
                    })
                    .fold(0., f64::max)
            };
            let ub = (0..3).map(|i| d[i] + cr(i)).fold(f64::INFINITY, f64::min);
            if ub.partial_cmp(&(lb + tol)) != Some(std::cmp::Ordering::Greater) {
                continue;
            }
            // A function matters in the cell only where it is realized: its
            // least value over the cell clipped to each of its regions (a
            // region missing the cell contributes nothing).
            let slack = 1e-9 * size;
            let least = |func: &F2, rs: &[R2]| -> f64 {
                let mut best = f64::INFINITY;
                for r in rs {
                    let (poly, m) = clip_cell(&cell, r, slack);
                    if m == 0 {
                        continue;
                    }
                    let poly = &poly[..m];
                    best = best.min(match *func {
                        F2::Circle(c, s) => dist_to_polygon(c, poly) + s,
                        F2::Line(u, k) => poly
                            .iter()
                            .map(|p| k + u[0] * p[0] + u[1] * p[1])
                            .fold(f64::INFINITY, f64::min),
                    });
                }
                best
            };
            let picked: Vec<usize> = parent
                .into_iter()
                .filter(|&i| least(&with[i].0, &with[i].1) <= ub)
                .collect();
            let active: Vec<F2> = picked.iter().map(|&i| with[i].0).collect();
            let holds = |i: usize, p: [f64; 2]| {
                with[picked[i]].1.iter().any(|r| {
                    r.iter()
                        .flatten()
                        .all(|(n, c)| n[0] * p[0] + n[1] * p[1] >= c - slack)
                })
            };
            let diameter = (0..3).map(cr).fold(0., f64::max);
            if active.len() > 12 && diameter > 1e-3 * size {
                // Longest-edge bisection (Rivara): slivers are cut along
                // their length into fat cells, one exact value per split.
                let len2 = |k: usize| {
                    let (a, b) = (cell[k], cell[(k + 1) % 3]);
                    (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)
                };
                let k = (0..3).max_by(|&i, &j| len2(i).total_cmp(&len2(j))).unwrap();
                let (k1, k2) = ((k + 1) % 3, (k + 2) % 3);
                let m = [
                    (cell[k][0] + cell[k1][0]) / 2.,
                    (cell[k][1] + cell[k1][1]) / 2.,
                ];
                let dm = at_(to3(m));
                stack.push(([cell[k], m, cell[k2]], [d[k], dm, d[k2]], picked.clone()));
                stack.push(([m, cell[k1], cell[k2]], [dm, d[k1], d[k2]], picked));
                continue;
            }
            let area = ((cell[1][0] - cell[0][0]) * (cell[2][1] - cell[0][1])
                - (cell[2][0] - cell[0][0]) * (cell[1][1] - cell[0][1]))
                .abs();
            let inside = |p: [f64; 2]| {
                let c = |a: [f64; 2], b: [f64; 2]| {
                    (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
                };
                let e = -1e-12 * area.max(1e-300);
                c(cell[0], cell[1]) >= e && c(cell[1], cell[2]) >= e && c(cell[2], cell[0]) >= e
            };
            for i in 0..active.len() {
                for j in i + 1..active.len() {
                    for k in j + 1..active.len() {
                        for (p, v) in triple(&active[i], &active[j], &active[k]) {
                            // A peak is where three functions realized there tie.
                            if v <= lb + tol
                                || !inside(p)
                                || !holds(i, p)
                                || !holds(j, p)
                                || !holds(k, p)
                            {
                                continue;
                            }
                            let p3 = to3(p);
                            let dv = at_(p3);
                            if (dv - v).abs() <= 1e-9 * (1. + v)
                                && !out
                                    .iter()
                                    .any(|(q, _)| norm(sub(*q, p3)) <= 1e-9 * (1. + v))
                            {
                                out.push((p3, dv));
                            }
                        }
                    }
                }
            }
        }
        out
    }

    /// The exact maximum over face `f` and where: its vertices, its edges'
    /// maxima, its interior peaks.
    fn face_max(&self, f: u32, peaks: &[(Vec3, f64)]) -> (f64, Vec3) {
        let (v, t, topo) = self.mesh();
        let mut best = (f64::NEG_INFINITY, [0.; 3]);
        for &k in &t[f as usize] {
            if self.vertex_at(k) > best.0 {
                best = (self.vertex_at(k), v[k as usize]);
            }
        }
        for &e in &topo.tri_edges[f as usize] {
            let (m, x) = self.edge_argmax(e);
            if m > best.0 {
                let o = v[topo.edges[e as usize][0] as usize];
                best = (m, axpy(o, x, topo.xhat[e as usize]));
            }
        }
        for &(p, d) in peaks {
            if d > best.0 {
                best = (d, p);
            }
        }
        best
    }
}

impl Distance for Field<'_> {
    fn mesh(&self) -> (&[Vec3], &[[u32; 3]], &Topology) {
        (self.v, self.t, self.topo)
    }
    fn vertex_at(&self, k: u32) -> f64 {
        self.vertex[k as usize]
    }
    fn at(&self, f: u32, p: Vec3) -> f64 {
        Field::at(self, f, p)
    }
    fn face_eval(&self, f: u32) -> Vec<FaceEval<'_>> {
        vec![FaceEval::new(self, f)]
    }
    fn edge_at(&self, e: u32, x: f64) -> f64 {
        Field::edge_at(self, e, x)
    }
    fn edge_windows(&self, e: u32) -> Vec<(Src, f64, f64)> {
        self.on_edge[e as usize]
            .iter()
            .map(|&id| &self.wins[id as usize])
            .filter(|w| w.alive)
            .map(|w| (w.src, w.b0, w.b1))
            .collect()
    }
    fn regions(&self, f: u32) -> Vec<(Func, Region)> {
        Field::regions(self, f)
    }
    fn edge_piece_at(&self, e: u32, x: f64) -> (f64, Src) {
        let [a, b] = self.topo.edges[e as usize];
        let l = self.topo.len[e as usize];
        let line = |ux: f64, c: f64| Src::Line { ux, uy: 0., c };
        let (da, db) = (self.vertex[a as usize], self.vertex[b as usize]);
        let mut best = (da + x, line(1., da));
        if db + l - x < best.0 {
            best = (db + l - x, line(-1., db + l));
        }
        // The window's piece at x: its function on its range, else its
        // continuation from the nearer end.
        let piece = |w: &Win| -> (f64, Src) {
            if x < w.b0 {
                let v = w.src.edge(w.b0);
                (v + w.b0 - x, line(-1., v + w.b0))
            } else if x > w.b1 {
                let v = w.src.edge(w.b1);
                (v + x - w.b1, line(1., v - w.b1))
            } else {
                (w.src.edge(x), w.src)
            }
        };
        let list = &self.on_edge[e as usize];
        let i = list.partition_point(|&id| self.wins[id as usize].b0 <= x);
        let mut consider = |w: &Win| {
            let c = piece(w);
            if c.0 < best.0 {
                best = c;
            }
        };
        if let Some(&id) = list.get(i) {
            consider(&self.wins[id as usize]);
        }
        for j in (0..i).rev() {
            let w = &self.wins[list[j] as usize];
            consider(w);
            if w.b1 < x {
                break;
            }
        }
        best
    }
}

/// The distance from the union of several fields' sources: at every point
/// the least of theirs. Coverage grows by adding a field of the new passes
/// alone; nothing already computed changes.
pub(super) struct Coverage<'a> {
    pub fields: Vec<Field<'a>>,
}

impl Distance for Coverage<'_> {
    fn mesh(&self) -> (&[Vec3], &[[u32; 3]], &Topology) {
        self.fields[0].mesh()
    }
    fn vertex_at(&self, k: u32) -> f64 {
        self.fields
            .iter()
            .map(|g| g.vertex[k as usize])
            .fold(f64::INFINITY, f64::min)
    }
    fn at(&self, f: u32, p: Vec3) -> f64 {
        self.fields
            .iter()
            .map(|g| g.at(f, p))
            .fold(f64::INFINITY, f64::min)
    }
    fn face_eval(&self, f: u32) -> Vec<FaceEval<'_>> {
        self.fields.iter().map(|g| FaceEval::new(g, f)).collect()
    }
    fn edge_at(&self, e: u32, x: f64) -> f64 {
        self.fields
            .iter()
            .map(|g| g.edge_at(e, x))
            .fold(f64::INFINITY, f64::min)
    }
    fn edge_windows(&self, e: u32) -> Vec<(Src, f64, f64)> {
        self.fields
            .iter()
            .flat_map(|g| Distance::edge_windows(g, e))
            .collect()
    }
    fn regions(&self, f: u32) -> Vec<(Func, Region)> {
        self.fields.iter().flat_map(|g| g.regions(f)).collect()
    }
    fn edge_piece_at(&self, e: u32, x: f64) -> (f64, Src) {
        self.fields
            .iter()
            .map(|g| g.edge_piece_at(e, x))
            .min_by(|p, q| p.0.total_cmp(&q.0))
            .unwrap()
    }
}

/// Where a function holds: up to three half-planes n . p >= c.
pub(super) type Region = [Option<(Vec3, f64)>; 3];

/// A function's identity: the same circle, or the same wave (direction and
/// value at the origin).
fn func_key(func: &Func) -> (u8, [u64; 6]) {
    match *func {
        Func::Circle(c, s) => (0u8, [c[0], c[1], c[2], s, 0., 0.].map(f64::to_bits)),
        Func::Line(o, u, s) => {
            let k = s - dot(o, u);
            (1u8, [u[0], u[1], u[2], k, 0., 0.].map(f64::to_bits))
        }
    }
}

/// A circle (cone: |p - c| + s) or line (s + (p - o) . u) of a face.
#[derive(Clone, Copy, Debug)]
pub(super) enum Func {
    Circle(Vec3, f64),
    Line(Vec3, Vec3, f64),
}

/// The same in the face's plane: a circle (centre, offset) or a line
/// (direction, value at the origin).
#[derive(Clone, Copy, Debug)]
enum F2 {
    Circle([f64; 2], f64),
    Line([f64; 2], f64),
}

/// The points (x, y) where three of a face's functions take one value v,
/// with v.
fn triple(a: &F2, b: &F2, c: &F2) -> Vec<([f64; 2], f64)> {
    let fs = [a, b, c];
    let circles: Vec<([f64; 2], f64)> = fs
        .iter()
        .filter_map(|f| match f {
            F2::Circle(c, s) => Some((*c, *s)),
            _ => None,
        })
        .collect();
    // Linear rows [x, y, v | rhs].
    let mut rows: Vec<[f64; 4]> = vec![];
    for f in fs {
        if let F2::Line(u, k) = f {
            // k + u . p = v
            rows.push([u[0], u[1], -1., -k]);
        }
    }
    for w in circles.windows(2) {
        let ((ci, si), (cj, sj)) = (w[0], w[1]);
        rows.push([
            2. * (cj[0] - ci[0]),
            2. * (cj[1] - ci[1]),
            2. * (si - sj),
            (cj[0] * cj[0] + cj[1] * cj[1] - sj * sj) - (ci[0] * ci[0] + ci[1] * ci[1] - si * si),
        ]);
    }
    let mut out = vec![];
    if circles.is_empty() {
        // Three lines: a 3 x 3 system.
        let m = [
            [rows[0][0], rows[0][1], rows[0][2]],
            [rows[1][0], rows[1][1], rows[1][2]],
            [rows[2][0], rows[2][1], rows[2][2]],
        ];
        let det = |m: [[f64; 3]; 3]| {
            m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
        };
        let d = det(m);
        if d.abs() < 1e-14 {
            return out;
        }
        let col = |k: usize| {
            let mut mk = m;
            for r in 0..3 {
                mk[r][k] = rows[r][3];
            }
            det(mk) / d
        };
        out.push(([col(0), col(1)], col(2)));
        return out;
    }
    // Two rows: (x, y) = P + Q v.
    let (r0, r1) = (rows[0], rows[1]);
    let det = r0[0] * r1[1] - r0[1] * r1[0];
    let scale = (r0[0].abs() + r0[1].abs()) * (r1[0].abs() + r1[1].abs());
    if det.abs() <= 1e-14 * scale.max(1e-300) {
        return out;
    }
    let solve = |b0: f64, b1: f64| {
        [
            (b0 * r1[1] - r0[1] * b1) / det,
            (r0[0] * b1 - b0 * r1[0]) / det,
        ]
    };
    let p = solve(r0[3], r1[3]);
    let q = solve(-r0[2], -r1[2]);
    let (c1, s1) = circles[0];
    let w = [p[0] - c1[0], p[1] - c1[1]];
    // |P + Q v - c1|^2 = (v - s1)^2
    let mut vs = vec![];
    roots(
        q[0] * q[0] + q[1] * q[1] - 1.,
        2. * (q[0] * w[0] + q[1] * w[1] + s1),
        w[0] * w[0] + w[1] * w[1] - s1 * s1,
        &mut vs,
    );
    for v in vs {
        if circles.iter().all(|&(_, s)| v >= s) {
            out.push(([p[0] + q[0] * v, p[1] + q[1] * v], v));
        }
    }
    out
}

/// The vertical projection onto the mesh of the XY segment from `a` (a
/// point of face `f`) to `b`, split into one piece per face (Bommes and
/// Kobbelt 2007, section 4.2: every source segment lies in one triangle).
/// Returns the face and point reached, or None where the segment leaves
/// the mesh (it crosses a boundary edge) first.
pub(super) fn walk(
    v: &[Vec3],
    t: &[[u32; 3]],
    topo: &Topology,
    f: u32,
    a: Vec3,
    b: [f64; 2],
    out: &mut Vec<Piece>,
) -> Option<(u32, Vec3)> {
    let (mut g, mut p, mut entry) = (f, a, NONE);
    for _ in 0..1_000_000 {
        let tri = t[g as usize].map(|k| v[k as usize]);
        let det = (tri[1][0] - tri[0][0]) * (tri[2][1] - tri[0][1])
            - (tri[2][0] - tri[0][0]) * (tri[1][1] - tri[0][1]);
        let bary = |q: [f64; 2]| {
            let l1 = ((q[0] - tri[0][0]) * (tri[2][1] - tri[0][1])
                - (tri[2][0] - tri[0][0]) * (q[1] - tri[0][1]))
                / det;
            let l2 = ((tri[1][0] - tri[0][0]) * (q[1] - tri[0][1])
                - (q[0] - tri[0][0]) * (tri[1][1] - tri[0][1]))
                / det;
            [1. - l1 - l2, l1, l2]
        };
        let l = bary(b);
        if l.iter().all(|&x| x >= -1e-12) {
            let q = [0, 1, 2].map(|i| l[0] * tri[0][i] + l[1] * tri[1][i] + l[2] * tri[2][i]);
            if norm(sub(q, p)) > 0. {
                out.push(Piece::Segment(g, p, q));
            }
            return Some((g, q));
        }
        // The edge the segment leaves through: the nearest crossing ahead.
        let d = [b[0] - p[0], b[1] - p[1]];
        let mut exit: Option<(f64, usize, f64)> = None;
        for k in 0..3 {
            let e = topo.tri_edges[g as usize][k];
            if e == entry {
                continue;
            }
            let (e0, e1) = (tri[k], tri[(k + 1) % 3]);
            let ed = [e1[0] - e0[0], e1[1] - e0[1]];
            let den = d[0] * ed[1] - d[1] * ed[0];
            if den.abs() < 1e-300 {
                continue;
            }
            let w = [e0[0] - p[0], e0[1] - p[1]];
            let s = (w[0] * ed[1] - w[1] * ed[0]) / den;
            let u = (w[0] * d[1] - w[1] * d[0]) / den;
            if s > 1e-12 && (-1e-9..=1. + 1e-9).contains(&u) && exit.is_none_or(|x| s < x.0) {
                exit = Some((s, k, u.clamp(0., 1.)));
            }
        }
        // Through a vertex: the face around it that the segment enters
        // next (none: it leaves the mesh there).
        let through = |vx: u32, g: u32| -> Option<u32> {
            let q = v[vx as usize];
            let dl = d[0].hypot(d[1]);
            let reach = topo.vertex_faces[vx as usize]
                .iter()
                .map(|&h| {
                    topo.tri_edges[h as usize]
                        .iter()
                        .map(|&e| topo.len[e as usize])
                        .fold(f64::INFINITY, f64::min)
                })
                .fold(f64::INFINITY, f64::min);
            let ahead = [
                q[0] + 1e-7 * reach * d[0] / dl,
                q[1] + 1e-7 * reach * d[1] / dl,
            ];
            // The face that holds the point just ahead best (along an edge
            // it lies on that edge: rounding must not lose it).
            let inside = |h: u32| {
                let w = t[h as usize].map(|k| v[k as usize]);
                let cr = |a: Vec3, b: Vec3| {
                    let l = (b[0] - a[0]).hypot(b[1] - a[1]);
                    ((b[0] - a[0]) * (ahead[1] - a[1]) - (b[1] - a[1]) * (ahead[0] - a[0])) / l
                };
                cr(w[0], w[1]).min(cr(w[1], w[2])).min(cr(w[2], w[0]))
            };
            topo.vertex_faces[vx as usize]
                .iter()
                .copied()
                .filter(|&h| h != g)
                .map(|h| (inside(h), h))
                .filter(|&(m, _)| m >= -1e-9 * reach)
                .max_by(|a, b| a.0.total_cmp(&b.0).then(b.1.cmp(&a.1)))
                .map(|(_, h)| h)
        };
        let Some((_, k, u)) = exit else {
            // No edge ahead: the point is on the face's boundary and the
            // segment leaves at once, through a vertex or across an edge.
            let scale = topo.tri_edges[g as usize]
                .iter()
                .map(|&e| topo.len[e as usize])
                .fold(0., f64::max);
            if let Some(&vx) = t[g as usize]
                .iter()
                .find(|&&k| norm(sub(v[k as usize], p)) <= 1e-9 * scale)
            {
                g = through(vx, g)?;
                (p, entry) = (v[vx as usize], NONE);
                continue;
            }
            let k = (0..3).find(|&k| {
                let (e0, e1) = (tri[k], tri[(k + 1) % 3]);
                let ed = [e1[0] - e0[0], e1[1] - e0[1]];
                let w = [p[0] - e0[0], p[1] - e0[1]];
                // On the edge (XY), with the segment heading out across it.
                (ed[0] * w[1] - ed[1] * w[0]).abs() <= 1e-9 * scale * ed[0].hypot(ed[1])
                    && ed[0] * d[1] - ed[1] * d[0] < 0.
                    && topo.tri_edges[g as usize][k] != entry
            })?;
            let e = topo.tri_edges[g as usize][k];
            let next = topo.other(e, g);
            if next == NONE {
                return None;
            }
            (g, entry) = (next, e);
            continue;
        };
        let e = topo.tri_edges[g as usize][k];
        let (e0, e1) = (tri[k], tri[(k + 1) % 3]);
        if !(1e-9..=1. - 1e-9).contains(&u) {
            let vx = t[g as usize][if u < 0.5 { k } else { (k + 1) % 3 }];
            let x = v[vx as usize];
            if norm(sub(x, p)) > 0. {
                out.push(Piece::Segment(g, p, x));
            }
            g = through(vx, g)?;
            (p, entry) = (x, NONE);
            continue;
        }
        let x = axpy(e0, u, sub(e1, e0));
        if norm(sub(x, p)) > 0. {
            out.push(Piece::Segment(g, p, x));
        }
        let next = topo.other(e, g);
        if next == NONE {
            return None;
        }
        (g, p, entry) = (next, x, e);
    }
    None
}

fn face_normal(v: &[Vec3], t: &[[u32; 3]], f: u32) -> Vec3 {
    let [a, b, c] = t[f as usize].map(|k| v[k as usize]);
    let n = cross(sub(b, a), sub(c, a));
    scale(n, 1. / norm(n))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicBool;

    fn build(v: Vec<Vec3>, t: Vec<[u32; 3]>) -> (Vec<Vec3>, Vec<[u32; 3]>, Topology) {
        let mut count: HashMap<(u32, u32), u32> = HashMap::new();
        for tri in &t {
            for k in 0..3 {
                let (a, b) = (tri[k], tri[(k + 1) % 3]);
                *count.entry((a.min(b), a.max(b))).or_default() += 1;
            }
        }
        let mut boundary = vec![false; v.len()];
        for (&(a, b), &n) in &count {
            if n == 1 {
                boundary[a as usize] = true;
                boundary[b as usize] = true;
            }
        }
        let topo = Topology::new(&v, &t, &boundary).unwrap();
        (v, t, topo)
    }

    /// An n x n grid of the height field z(x, y) over [0, size]^2.
    fn grid(n: usize, size: f64, z: impl Fn(f64, f64) -> f64) -> (Vec<Vec3>, Vec<[u32; 3]>) {
        let mut v = vec![];
        for j in 0..=n {
            for i in 0..=n {
                let (x, y) = (size * i as f64 / n as f64, size * j as f64 / n as f64);
                v.push([x, y, z(x, y)]);
            }
        }
        let id = |i: usize, j: usize| (j * (n + 1) + i) as u32;
        let mut t = vec![];
        for j in 0..n {
            for i in 0..n {
                // Alternate diagonals: vertices of valence 4 and 8.
                if (i + j) % 2 == 0 {
                    t.push([id(i, j), id(i + 1, j), id(i + 1, j + 1)]);
                    t.push([id(i, j), id(i + 1, j + 1), id(i, j + 1)]);
                } else {
                    t.push([id(i, j), id(i + 1, j), id(i, j + 1)]);
                    t.push([id(i + 1, j), id(i + 1, j + 1), id(i, j + 1)]);
                }
            }
        }
        (v, t)
    }

    fn locate(v: &[Vec3], t: &[[u32; 3]], p: [f64; 2]) -> (u32, Vec3) {
        for (k, tri) in t.iter().enumerate() {
            let [a, b, c] = tri.map(|i| v[i as usize]);
            let det = (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]);
            let l1 = ((p[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (p[1] - a[1])) / det;
            let l2 = ((b[0] - a[0]) * (p[1] - a[1]) - (p[0] - a[0]) * (b[1] - a[1])) / det;
            let l0 = 1. - l1 - l2;
            if l0 >= -1e-12 && l1 >= -1e-12 && l2 >= -1e-12 {
                return (
                    k as u32,
                    [0, 1, 2].map(|i| l0 * a[i] + l1 * b[i] + l2 * c[i]),
                );
            }
        }
        panic!("{p:?} off the mesh");
    }

    #[test]
    fn a_point_on_a_plane_is_euclidean_everywhere() {
        let never = AtomicBool::new(false);
        let (v, t) = grid(7, 20., |_, _| 1.);
        let (v, t, topo) = build(v, t);
        for seed in [
            [7.3, 11.1],
            [0., 0.],
            [20. / 7. * 3., 20. / 7. * 2.],
            [5., 0.],
        ] {
            let (f, p) = locate(&v, &t, seed);
            let field = Field::compute(&v, &t, &topo, &[Piece::Point(f, p)], f64::INFINITY, &never)
                .unwrap();
            for k in 0..400 {
                let q = [
                    0.05 * (k % 20) as f64 * 19.9 + 0.01,
                    0.05 * (k / 20) as f64 * 19.9 + 0.02,
                ];
                let (g, q3) = locate(&v, &t, q);
                let d = field.at(g, q3);
                assert!((d - norm(sub(q3, p))).abs() < 1e-9, "{seed:?} {q:?}: {d}");
            }
        }
    }

    #[test]
    fn a_segment_source_is_the_distance_to_the_segment() {
        let never = AtomicBool::new(false);
        let (v, t) = grid(6, 12., |_, _| 0.);
        let (v, t, topo) = build(v, t);
        // A segment crossing several faces, split per face.
        let (a, b) = ([2.3, 3.1, 0.], [9.4, 7.7, 0.]);
        let mut pieces = vec![];
        let (f0, q0) = locate(&v, &t, [a[0], a[1]]);
        walk(&v, &t, &topo, f0, q0, [b[0], b[1]], &mut pieces).unwrap();
        assert!(pieces.len() > 5);
        let field = Field::compute(&v, &t, &topo, &pieces, f64::INFINITY, &never).unwrap();
        let seg = |q: Vec3| {
            let ab = sub(b, a);
            let s = (dot(sub(q, a), ab) / dot(ab, ab)).clamp(0., 1.);
            norm(sub(q, axpy(a, s, ab)))
        };
        for k in 0..400 {
            let q = [
                0.03 * (k % 20) as f64 * 19.9 + 0.01,
                0.03 * (k / 20) as f64 * 19.9 + 0.02,
            ];
            let (g, q3) = locate(&v, &t, q);
            let d = field.at(g, q3);
            assert!((d - seg(q3)).abs() < 1e-9, "{q:?}: {d} {}", seg(q3));
        }
    }

    #[test]
    fn a_walk_passes_through_vertices() {
        let (v, t) = grid(6, 12., |x, y| 0.1 * x + 0.05 * y);
        let (v, t, topo) = build(v, t);
        // Along a grid line and along diagonals: through vertex after
        // vertex, ending where it should and covering the whole segment.
        for (a, b) in [
            ([0.5, 4.], [11.5, 4.]),
            ([0.3, 0.3], [11.7, 11.7]),
            ([11.9, 0.1], [0.1, 11.9]),
        ] {
            let (f, p) = locate(&v, &t, a);
            let mut pieces = vec![];
            let (_, q) = walk(&v, &t, &topo, f, p, b, &mut pieces).expect("reaches the end");
            assert!((q[0] - b[0]).abs() < 1e-9 && (q[1] - b[1]).abs() < 1e-9);
            let length: f64 = pieces
                .iter()
                .map(|piece| match *piece {
                    Piece::Segment(_, x, y) => (x[0] - y[0]).hypot(x[1] - y[1]),
                    Piece::Point(..) => 0.,
                })
                .sum();
            assert!(
                (length - (b[0] - a[0]).hypot(b[1] - a[1])).abs() < 1e-9,
                "{a:?} {b:?}: {length}"
            );
        }
    }

    #[test]
    fn a_segment_never_exceeds_points_along_it() {
        // Cells of 0.5 split along one diagonal on a sphere, as an
        // exported height field is; a short segment from a point on an
        // edge into a face. Points along it are a subset of it, so their
        // field bounds the segment's from above (this caught equal-offset
        // windows whose bisector was lost to a rounded double root).
        let never = AtomicBool::new(false);
        let n = 12;
        let mut v = vec![];
        for j in 0..=n {
            for i in 0..=n {
                let (x, y) = (20. + 0.5 * i as f64, 13. + 0.5 * j as f64);
                v.push([
                    x,
                    y,
                    (900. - (x - 20f64).powi(2) - (y - 20f64).powi(2)).sqrt() - 12.,
                ]);
            }
        }
        let id = |i: usize, j: usize| (j * (n + 1) + i) as u32;
        let mut t = vec![];
        for j in 0..n {
            for i in 0..n {
                t.push([id(i, j), id(i + 1, j), id(i + 1, j + 1)]);
                t.push([id(i, j), id(i + 1, j + 1), id(i, j + 1)]);
            }
        }
        let (v, t, topo) = build(v, t);
        let (_, a) = locate(&v, &t, [23.87721836215997, 16.5]);
        let (f, b) = locate(&v, &t, [23.69322980305756, 16.306770196942438]);
        let field = Field::compute(
            &v,
            &t,
            &topo,
            &[Piece::Segment(f, a, b)],
            f64::INFINITY,
            &never,
        )
        .unwrap();
        let dots: Vec<Piece> = (0..=400)
            .map(|i| Piece::Point(f, axpy(a, i as f64 / 400., sub(b, a))))
            .collect();
        let points = Field::compute(&v, &t, &topo, &dots, f64::INFINITY, &never).unwrap();
        let seg = |q: Vec3| {
            let (g, q3) = locate(&v, &t, [q[0], q[1]]);
            points.at(g, q3)
        };
        let mut worst = (0f64, [0.; 2]);
        for k in 0..2500 {
            let q = [
                20.01 + 0.1196 * (k % 50) as f64,
                13.01 + 0.1196 * (k / 50) as f64,
            ];
            let (g, q3) = locate(&v, &t, q);
            let e = field.at(g, q3) - seg(q3);
            if e > worst.0 {
                worst = (e, q);
            }
        }
        assert!(worst.0 < 1e-9, "{worst:?}");
    }

    #[test]
    fn interior_peaks_are_exact() {
        // A triangle seeded from its own boundary: the distance is zero on
        // every edge and peaks at the incentre, the inradius in (three
        // linear windows equal).
        let never = AtomicBool::new(false);
        let v = vec![[0., 0., 1.], [13., 1., 1.], [4., 9., 1.]];
        let (v, t, topo) = build(v, vec![[0, 1, 2]]);
        let sides: Vec<Piece> = (0..3)
            .map(|k| Piece::Segment(0, v[k], v[(k + 1) % 3]))
            .collect();
        let field = Field::compute(&v, &t, &topo, &sides, f64::INFINITY, &never).unwrap();
        let l = [0, 1, 2].map(|k| norm(sub(v[(k + 1) % 3], v[(k + 2) % 3])));
        let per = l[0] + l[1] + l[2];
        let centre = [0, 1, 2].map(|i| (l[0] * v[0][i] + l[1] * v[1][i] + l[2] * v[2][i]) / per);
        let area = norm(cross(sub(v[1], v[0]), sub(v[2], v[0]))) / 2.;
        let inradius = 2. * area / per;
        let peaks = field.face_peaks(0);
        assert_eq!(peaks.len(), 1, "{peaks:?}");
        assert!(
            norm(sub(peaks[0].0, centre)) < 1e-9 && (peaks[0].1 - inradius).abs() < 1e-9,
            "{peaks:?}"
        );
        let (m, at) = field.face_max(0, &peaks);
        assert!((m - inradius).abs() < 1e-9 && norm(sub(at, centre)) < 1e-9);
    }

    #[test]
    fn paths_bend_around_a_boundary_corner() {
        // An L-shaped plane: from one arm to the other the shortest path
        // turns at the reflex corner (a boundary vertex pseudosource).
        let never = AtomicBool::new(false);
        let (v, t) = grid(8, 8., |_, _| 0.);
        // Remove the square [4, 8] x [4, 8].
        let t: Vec<[u32; 3]> = t
            .into_iter()
            .filter(|tri| {
                let c = tri.iter().fold([0., 0.], |s, &k| {
                    [s[0] + v[k as usize][0] / 3., s[1] + v[k as usize][1] / 3.]
                });
                !(c[0] > 4. && c[1] > 4.)
            })
            .collect();
        let (v, t, topo) = build(v, t);
        let (f, p) = locate(&v, &t, [7.5, 1.]);
        let field =
            Field::compute(&v, &t, &topo, &[Piece::Point(f, p)], f64::INFINITY, &never).unwrap();
        let corner = [4., 4., 0.];
        let (g, q) = locate(&v, &t, [1., 7.5]);
        let truth = norm(sub(p, corner)) + norm(sub(corner, q));
        assert!(
            (field.at(g, q) - truth).abs() < 1e-9,
            "{} {truth}",
            field.at(g, q)
        );
        // Where the straight line stays on the plane, it is the distance.
        let (g, q) = locate(&v, &t, [1., 3.]);
        assert!((field.at(g, q) - norm(sub(p, q))).abs() < 1e-9);
    }

    #[test]
    fn across_a_ridge_the_faces_unfold() {
        let never = AtomicBool::new(false);
        // A roof: slopes of 1 meeting at x = 5.
        let (v, t) = grid(10, 10., |x, _| 5. - (x - 5.).abs());
        let (v, t, topo) = build(v, t);
        let (f, p) = locate(&v, &t, [1., 5.]);
        let field =
            Field::compute(&v, &t, &topo, &[Piece::Point(f, p)], f64::INFINITY, &never).unwrap();
        for y in [5., 7.3, 9.] {
            let (g, q) = locate(&v, &t, [9., y]);
            // Unfolded: 8 sqrt 2 across, (y - 5) along the ridge.
            let truth = (128f64 + (y - 5.).powi(2)).sqrt();
            assert!(
                (field.at(g, q) - truth).abs() < 1e-9,
                "{y}: {}",
                field.at(g, q)
            );
        }
    }

    #[test]
    fn shortest_paths_pass_through_a_saddle_vertex() {
        // A fan of 8 triangles around a vertex whose ring zigzags up and
        // down: its total angle exceeds 2 pi. Between two ring points
        // more than pi apart either way round, the path goes through it.
        let never = AtomicBool::new(false);
        let mut v = vec![[0., 0., 0.]];
        for k in 0..8 {
            let a = std::f64::consts::TAU * k as f64 / 8.;
            v.push([a.cos(), a.sin(), if k % 2 == 0 { 0.6 } else { -0.6 }]);
        }
        let t: Vec<[u32; 3]> = (0..8)
            .map(|k| [0, 1 + k as u32, 1 + ((k + 1) % 8) as u32])
            .collect();
        let (v, t, topo) = build(v, t);
        assert!(topo.pseudo[0], "a saddle");
        let angle_at = |k: usize| {
            let (x, y) = (v[1 + k], v[1 + (k + 1) % 8]);
            norm(cross(x, y)).atan2(dot(x, y))
        };
        let total: f64 = (0..8).map(angle_at).sum();
        assert!(total > std::f64::consts::TAU);
        // Points halfway along the spokes 0 and 4 (opposite).
        let p = scale(v[1], 0.5);
        let q = scale(v[5], 0.5);
        let field =
            Field::compute(&v, &t, &topo, &[Piece::Point(0, p)], f64::INFINITY, &never).unwrap();
        let truth = norm(p) + norm(q);
        assert!(
            (field.at(4, q) - truth).abs() < 1e-9,
            "{} {truth}",
            field.at(4, q)
        );
    }

    #[test]
    fn crossings_are_on_the_level_set() {
        let never = AtomicBool::new(false);
        let (v, t) = grid(5, 10., |x, y| 0.1 * x * y / 10.);
        let (v, t, topo) = build(v, t);
        let (f, p) = locate(&v, &t, [3.3, 4.1]);
        let field =
            Field::compute(&v, &t, &topo, &[Piece::Point(f, p)], f64::INFINITY, &never).unwrap();
        let level = 3.7;
        let mut found = 0;
        for e in 0..topo.edges.len() as u32 {
            for s in field.edge_crossings(e, level) {
                let x = s * topo.len[e as usize];
                assert!((field.edge_at(e, x) - level).abs() < 1e-9);
                found += 1;
            }
        }
        assert!(found > 10);
        // Inside a face, across it.
        for g in 0..t.len() as u32 {
            let [a, b, c] = t[g as usize].map(|k| v[k as usize]);
            let m = scale(axpy(b, 1., c), 0.5);
            let (sa, sm) = (field.at(g, a) >= level, field.at(g, m) >= level);
            for s in FaceQuery::new(&field, g).crossings_on(a, m, level, sa, sm) {
                let q = axpy(a, s, sub(m, a));
                assert!((field.at(g, q) - level).abs() < 1e-9);
            }
        }
    }
}
