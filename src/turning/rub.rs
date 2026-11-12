//! Which parts of a turning tool may be where stock is during a cutting
//! move (#220).
//!
//! The condition a cutting move is held to. Only the insert's cutting face
//! (its nose and edges, on the centre plane) removes material. Every other
//! point of the tool (the insert's body below its face, the holder, the
//! bar) must, at every instant of the move, be clear of the stock as it is
//! at that instant: the stock the move found, less what the cutting face
//! has swept up to that instant. Material the same move removes later is
//! still there when a body point arrives before the face does.
//!
//! In the (z, r) half-plane (u, v) the checks work in, with the tool's
//! pieces as their images there (`body.rs`: the XZ projection for an
//! outside tool, the radial image in a bore):
//!
//! - The stock is a height field over z, the turning stock's own model
//!   (`stock.rs`: a cut lowers the outer radius or raises the bore). Where
//!   the face is or has been at (u, v), no stock is left at that u on the
//!   face's open side (outside turning: at any radius above v; in a bore:
//!   below). So with F the face (convex pieces) and e the open direction,
//!   the space certainly clear at the instant t of a straight move from c0
//!   by D (t in 0..1) is
//!
//!   ```text
//!   clear(t) = c0 + F ⊕ ray(e) ⊕ [0, t]·D
//!   ```
//!
//!   the face's sweep so far, closed toward the open side.
//! - A body point at x at the instant t is admitted when it is in clear(t)
//!   or out of the stock the move found, each within the move checks' guard
//!   (a square of that half-size, as `Envelope::penetration` erodes the
//!   stock by): x in clear(t) ⊕ square, or not in the eroded stock.
//!
//! So what must be out of the stock the move found is, exactly,
//!
//! ```text
//! exposed = ∪ over t of body(t) \ (clear(t) ⊕ square)
//! ```
//!
//! and it has a closed form. In (u, v, t) the body's track is the convex
//! hull of its image at the start (t = 0) and at the end (t = 1); the clear
//! space is convex too (the image of a convex set under a linear map), with
//! one half-space per side normal n of F ⊕ ray(e) ⊕ square and of ±D's
//! perpendicular: n·(x - c0) - t·max(0, n·D) <= support(n). The track less
//! a convex set is the union of its parts beyond each of those half-spaces,
//! each a convex polytope cut by one plane; its shadow on the (u, v) plane
//! is a convex polygon, compared with the stock like any tool piece.
//! Nothing is sampled along the move.
//!
//! Where the track is not rigid (in a bore, a move that changes radius
//! changes every body point's radial image differently) the hull of the
//! two end images is the track of the chords of the points' paths: a
//! point's radius is a convex function of the move's parameter, so its
//! chord is never below it, which is the safe side in a bore.
//!
//! Limits (each on the refusing side):
//! - An arc piece is not parametrised: only the face where it is at that
//!   instant counts as clear (`exposed_fixed`), not what it swept earlier in
//!   the same piece.
//! - A threading pass leaves its crests: only the face where it is counts.
//! - The stock a move finds holds, per z bin, the highest material left in
//!   the bin: where an earlier move ended inside a bin, its sweep there is
//!   not known to the next move.
//! - The face is the polygon the body is built from (arcs circumscribed
//!   within 0.00015 x their radius, `tool.rs`).
use super::ops::hull;

type P2 = [f64; 2];
/// (u, v, t).
type P3 = [f64; 3];
/// Outside when n·x - rate·t > bound.
type HalfSpace = (P2, f64, f64);

/// Rounding noise of the plane tests (mm): a point this near a plane is on
/// it.
const NOISE: f64 = 1e-9;

fn dot(a: P2, b: P2) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

/// The space a convex piece of the cutting face keeps clear, in the tool's
/// frame: the piece, closed toward the stock's open side, grown by the
/// guard square.
pub(crate) struct Shield {
    points: Vec<P2>,
    open: P2,
    guard: f64,
    /// Its sides: n·x <= c.
    planes: Vec<(P2, f64)>,
}

impl Shield {
    /// `face`: a convex polygon; `open`: the unit axis direction in which
    /// no stock lies beyond a cleared point (+v outside, -v in a bore).
    pub(crate) fn new(face: &[P2], open: P2, guard: f64) -> Option<Self> {
        let points = hull(face.to_vec());
        if points.is_empty() {
            return None;
        }
        let mut shield = Shield {
            points,
            open,
            guard,
            planes: vec![],
        };
        // The side normals of a sum of convex sets are those of its terms:
        // the polygon's, and the axes (the ray's sides and the square's).
        let mut normals: Vec<P2> = vec![[1., 0.], [-1., 0.], [0., 1.], [0., -1.]];
        let n = shield.points.len();
        if n >= 2 {
            for i in 0..n {
                let (a, b) = (shield.points[i], shield.points[(i + 1) % n]);
                let d = [b[0] - a[0], b[1] - a[1]];
                let l = d[0].hypot(d[1]);
                // Counter-clockwise: outward is to the right of travel.
                let normal = [d[1] / l, -d[0] / l];
                if l > 0. && normals.iter().all(|m| dot(*m, normal) < 1. - 1e-12) {
                    normals.push(normal);
                }
            }
        }
        shield.planes = normals
            .into_iter()
            .filter_map(|n| shield.support(n).map(|c| (n, c)))
            .collect();
        Some(shield)
    }

    /// How far the shield reaches along the unit direction `n` (None: it
    /// has no end that way).
    fn support(&self, n: P2) -> Option<f64> {
        if dot(n, self.open) > 1e-12 {
            return None;
        }
        let reach = self
            .points
            .iter()
            .map(|p| dot(n, *p))
            .fold(f64::NEG_INFINITY, f64::max);
        Some(reach + self.guard * (n[0].abs() + n[1].abs()))
    }

    /// The clear space's sides over a straight move from `from` by `by`
    /// (t in 0..1). `trailing`: what the face has swept stays clear;
    /// otherwise only where it is.
    fn over_line(&self, from: P2, by: P2, trailing: bool) -> Vec<HalfSpace> {
        let mut out: Vec<HalfSpace> = self
            .planes
            .iter()
            .map(|&(n, c)| {
                let along = dot(n, by);
                let rate = if trailing { along.max(0.) } else { along };
                (n, rate, c + dot(n, from))
            })
            .collect();
        let l = by[0].hypot(by[1]);
        if trailing && l > 0. {
            // The sweep's own sides, along the move.
            for m in [[by[1] / l, -by[0] / l], [-by[1] / l, by[0] / l]] {
                if let Some(c) = self.support(m) {
                    out.push((m, 0., c + dot(m, from)));
                }
            }
        }
        out
    }
}

/// A convex polytope in (u, v, t) by points whose hull it is, with its
/// edges when they are known (None: any two points may span one).
struct Piece {
    points: Vec<P3>,
    edges: Option<Vec<(usize, usize)>>,
}

/// The part of `piece` beyond `plane`: the points beyond it and where an
/// edge crosses it.
fn beyond(piece: &Piece, plane: &HalfSpace) -> Vec<P3> {
    let (n, rate, bound) = *plane;
    let side: Vec<f64> = piece
        .points
        .iter()
        .map(|p| dot(n, [p[0], p[1]]) - rate * p[2] - bound - NOISE)
        .collect();
    if side.iter().all(|s| *s <= 0.) {
        return vec![];
    }
    let mut out: Vec<P3> = piece
        .points
        .iter()
        .zip(&side)
        .filter(|(_, s)| **s > 0.)
        .map(|(p, _)| *p)
        .collect();
    let mut cross = |i: usize, j: usize| {
        if (side[i] > 0.) != (side[j] > 0.) {
            let k = side[i] / (side[i] - side[j]);
            let (p, q) = (piece.points[i], piece.points[j]);
            out.push([
                p[0] + k * (q[0] - p[0]),
                p[1] + k * (q[1] - p[1]),
                p[2] + k * (q[2] - p[2]),
            ]);
        }
    };
    match &piece.edges {
        Some(edges) => edges.iter().for_each(|&(i, j)| cross(i, j)),
        None => {
            for i in 0..piece.points.len() {
                for j in i + 1..piece.points.len() {
                    cross(i, j);
                }
            }
        }
    }
    out
}

/// More points than this in a piece whose edges are not known: the shields
/// not yet taken off it are left on (its exposed part is then larger than
/// the true one, never smaller).
const MOST_POINTS: usize = 256;

/// `start` less every clear space of `clear` (each the intersection of its
/// half-spaces), as the shadows on (u, v) of its convex parts.
fn exposed(start: Piece, clear: &[Vec<HalfSpace>]) -> Vec<Vec<P2>> {
    let mut pieces = vec![start];
    for planes in clear {
        let mut next = vec![];
        for piece in pieces {
            if piece.edges.is_none() && piece.points.len() > MOST_POINTS {
                next.push(piece);
                continue;
            }
            for plane in planes {
                let points = beyond(&piece, plane);
                if !points.is_empty() {
                    next.push(Piece {
                        points,
                        edges: None,
                    });
                }
            }
        }
        pieces = next;
    }
    pieces
        .into_iter()
        .map(|piece| hull(piece.points.iter().map(|p| [p[0], p[1]]).collect()))
        .collect()
}

/// The edges of a convex polygon's ring, its points numbered from `base`.
fn ring(count: usize, base: usize, edges: &mut Vec<(usize, usize)>) {
    if count >= 2 {
        edges.extend((0..count).map(|i| (base + i, base + (i + 1) % count)));
    }
}

/// Everywhere a tool piece is while it is not in clear space, over a
/// straight move from `from` by `by`: convex polygons whose union is that
/// set exactly. `w0`, `w1`: the piece's image (a convex polygon,
/// counter-clockwise as `hull` gives it) at the move's start and end;
/// `shields`: the face's, in the tool's frame; `trailing`: see
/// `Shield::over_line`.
pub(crate) fn exposed_line(
    w0: &[P2],
    w1: &[P2],
    shields: &[Shield],
    from: P2,
    by: P2,
    trailing: bool,
) -> Vec<Vec<P2>> {
    let (k0, k1) = (w0.len(), w1.len());
    let points: Vec<P3> = w0
        .iter()
        .map(|p| [p[0], p[1], 0.])
        .chain(w1.iter().map(|p| [p[0], p[1], 1.]))
        .collect();
    let clear: Vec<Vec<HalfSpace>> = shields
        .iter()
        .map(|s| s.over_line(from, by, trailing))
        .collect();
    // Wholly inside one clear space (the usual case: the insert's body
    // under its face): nothing is exposed.
    let covers = |planes: &Vec<HalfSpace>| {
        planes.iter().all(|&(n, rate, bound)| {
            points
                .iter()
                .all(|p| dot(n, [p[0], p[1]]) - rate * p[2] - bound - NOISE <= 0.)
        })
    };
    if clear.iter().any(covers) {
        return vec![];
    }
    // The track's edges: both polygons' rings and, between them, those of
    // its side faces. A side face lies on a plane touching both polygons
    // along one direction: an edge of one and, of the other, the vertex
    // (or, parallel to it, the edge) farthest that way.
    let mut edges = vec![];
    ring(k0, 0, &mut edges);
    ring(k1, k0, &mut edges);
    if k0 < 3 || k1 < 3 {
        edges.extend((0..k0).flat_map(|i| (0..k1).map(move |j| (i, k0 + j))));
    } else {
        let scale = points
            .iter()
            .fold(1., |m: f64, p| m.max(p[0].abs()).max(p[1].abs()));
        let mut sides = |a: &[P2], a_base: usize, b: &[P2], b_base: usize| {
            for i in 0..a.len() {
                let (p, q) = (a[i], a[(i + 1) % a.len()]);
                let n = [q[1] - p[1], p[0] - q[0]];
                let l = n[0].hypot(n[1]);
                if l == 0. {
                    continue;
                }
                let n = [n[0] / l, n[1] / l];
                let far = b
                    .iter()
                    .map(|x| dot(n, *x))
                    .fold(f64::NEG_INFINITY, f64::max);
                for (j, x) in b.iter().enumerate() {
                    if dot(n, *x) >= far - 1e-9 * scale {
                        edges.push((a_base + i, b_base + j));
                        edges.push((a_base + (i + 1) % a.len(), b_base + j));
                    }
                }
            }
        };
        sides(w0, 0, w1, k0);
        sides(w1, k0, w0, 0);
    }
    exposed(
        Piece {
            points,
            edges: Some(edges),
        },
        &clear,
    )
}

/// The part of a tool piece that is never under the face where the face is
/// at that instant: `body` (a convex polygon in the tool's frame) less the
/// shields, as convex polygons in that frame. For a move that is not
/// parametrised (an arc piece): carried along it, they cover everywhere the
/// piece is while not in the face's own clear space.
pub(crate) fn exposed_fixed(body: &[P2], shields: &[Shield]) -> Vec<Vec<P2>> {
    let body = hull(body.to_vec());
    let mut edges = vec![];
    ring(body.len(), 0, &mut edges);
    let clear: Vec<Vec<HalfSpace>> = shields
        .iter()
        .map(|s| s.planes.iter().map(|&(n, c)| (n, 0., c)).collect())
        .collect();
    exposed(
        Piece {
            points: body.iter().map(|p| [p[0], p[1], 0.]).collect(),
            edges: Some(edges),
        },
        &clear,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A convex polygon's lowest and highest v at u (None outside it).
    fn span(polygon: &[P2], u: f64) -> Option<[f64; 2]> {
        let n = polygon.len();
        let mut out: Option<[f64; 2]> = None;
        for i in 0..n {
            let (p, q) = (polygon[i], polygon[(i + 1) % n]);
            if u < p[0].min(q[0]) || u > p[0].max(q[0]) {
                continue;
            }
            let ends = if p[0] == q[0] {
                [p[1].min(q[1]), p[1].max(q[1])]
            } else {
                let v = p[1] + (u - p[0]) / (q[0] - p[0]) * (q[1] - p[1]);
                [v, v]
            };
            out = Some(out.map_or(ends, |o| [o[0].min(ends[0]), o[1].max(ends[1])]));
        }
        out
    }

    /// Whether x is in face ⊕ ray(open) ⊕ square(guard), from the
    /// definition: some point within the square of x lies on the open side
    /// of the face's boundary at its own u (the boundary is convex, so its
    /// nearest radius over an interval of u is at an end or a vertex).
    fn in_shield(face: &[P2], up: bool, guard: f64, x: P2) -> bool {
        let (lo, hi) = face
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), p| {
                (l.min(p[0]), h.max(p[0]))
            });
        let (a, b) = ((x[0] - guard).max(lo), (x[0] + guard).min(hi));
        if a > b {
            return false;
        }
        [a, b]
            .into_iter()
            .chain(face.iter().map(|p| p[0]).filter(|u| *u > a && *u < b))
            .filter_map(|u| span(face, u))
            .any(|[low, high]| {
                if up {
                    x[1] + guard >= low
                } else {
                    x[1] - guard <= high
                }
            })
    }

    fn inside(polygon: &[P2], x: P2, slack: f64) -> bool {
        let n = polygon.len();
        n >= 3
            && (0..n).all(|i| {
                let (p, q) = (polygon[i], polygon[(i + 1) % n]);
                let d = [q[0] - p[0], q[1] - p[1]];
                let l = d[0].hypot(d[1]);
                l == 0. || (d[0] * (x[1] - p[1]) - d[1] * (x[0] - p[0])) / l >= -slack
            })
    }

    /// Points of a convex polygon: its vertices and a grid of mixes of them.
    fn points_of(polygon: &[P2]) -> Vec<P2> {
        let n = polygon.len();
        let centre = polygon.iter().fold([0., 0.], |c, p| {
            [c[0] + p[0] / n as f64, c[1] + p[1] / n as f64]
        });
        let mut out = vec![];
        for i in 0..n {
            let (p, q) = (polygon[i], polygon[(i + 1) % n]);
            for a in 0..6 {
                let e = [
                    p[0] + (q[0] - p[0]) * a as f64 / 6.,
                    p[1] + (q[1] - p[1]) * a as f64 / 6.,
                ];
                for b in 0..5 {
                    let k = b as f64 / 5.;
                    out.push([e[0] + (centre[0] - e[0]) * k, e[1] + (centre[1] - e[1]) * k]);
                }
            }
        }
        out
    }

    /// A deterministic stream of numbers in 0..1.
    struct Numbers(u64);
    impl Numbers {
        fn next(&mut self) -> f64 {
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (self.0 >> 11) as f64 / (1u64 << 53) as f64
        }
        fn polygon(&mut self, centre: P2, size: f64) -> Vec<P2> {
            let n = 3 + (self.next() * 5.) as usize;
            hull(
                (0..n)
                    .map(|_| {
                        [
                            centre[0] + size * (self.next() - 0.5),
                            centre[1] + size * (self.next() - 0.5),
                        ]
                    })
                    .collect(),
            )
        }
    }

    /// The exposed polygons of a straight move are the set the definition
    /// gives, sampled: every body point at every sampled instant that is
    /// clearly outside the clear space of that instant lies in one of them,
    /// and every vertex of theirs is such a point (within the sampling).
    #[test]
    fn a_straight_moves_exposed_set_is_the_definitions() {
        let mut numbers = Numbers(7);
        let (guard, margin) = (0.01, 2e-3);
        let (mut exposed_points, mut shielded_points) = (0, 0);
        for case in 0..60 {
            let up = case % 2 == 0;
            let trailing = case % 5 != 0;
            let face = numbers.polygon([0., 0.], 3.);
            let centre = [2. * (numbers.next() - 0.5), 2. * (numbers.next() - 0.5)];
            let body = numbers.polygon(centre, 4.);
            if face.len() < 3 || body.len() < 3 {
                continue;
            }
            let from = [10. * numbers.next(), 20. + 5. * numbers.next()];
            let by = [6. * (numbers.next() - 0.5), 6. * (numbers.next() - 0.5)];
            let open = if up { [0., 1.] } else { [0., -1.] };
            let shield = Shield::new(&face, open, guard).unwrap();
            let at = |polygon: &[P2], t: f64| -> Vec<P2> {
                polygon
                    .iter()
                    .map(|p| [p[0] + from[0] + t * by[0], p[1] + from[1] + t * by[1]])
                    .collect()
            };
            let polygons = exposed_line(
                &hull(at(&body, 0.)),
                &hull(at(&body, 1.)),
                std::slice::from_ref(&shield),
                from,
                by,
                trailing,
            );
            // Whether x is clear at the instant t with the guard `g`: under
            // the face at some instant up to t (sampled finer than the
            // margin), or at t alone.
            let steps = 4000;
            let clear = |x: P2, t: f64, g: f64| -> bool {
                let last = (t * steps as f64).floor() as usize;
                let mut instants = vec![t];
                if trailing {
                    instants.extend((0..=last).map(|k| k as f64 / steps as f64));
                }
                instants.into_iter().any(|s| {
                    let local = [x[0] - from[0] - s * by[0], x[1] - from[1] - s * by[1]];
                    in_shield(&face, up, g, local)
                })
            };
            assert!(by[0].hypot(by[1]) / steps as f64 <= margin / 2.);
            let samples = points_of(&body);
            for k in 0..=40 {
                let t = k as f64 / 40.;
                for x in at(&samples, t) {
                    if clear(x, t, guard + margin) {
                        shielded_points += 1;
                        continue;
                    }
                    exposed_points += 1;
                    assert!(
                        polygons.iter().any(|p| inside(p, x, 1e-7)),
                        "case {case}: {x:?} at {t} is exposed and in no polygon"
                    );
                }
            }
            // Tight: each vertex is a body point at some instant (within the
            // sampling of instants) and not clear then with a smaller guard.
            for polygon in &polygons {
                for x in polygon {
                    let found = (0..=400).any(|k| {
                        let t = k as f64 / 400.;
                        inside(&hull(at(&body, t)), *x, by[0].hypot(by[1]) / 400. + 1e-6)
                            && !clear(*x, t, guard - margin)
                    });
                    assert!(
                        found,
                        "case {case}: {x:?} is in a polygon and never exposed"
                    );
                }
            }
        }
        assert!(exposed_points > 5_000 && shielded_points > 5_000);
    }

    /// A body under the face is not exposed; one beside it is, where it is
    /// beside it; behind the face along the move, until the face's sweep
    /// reaches it.
    #[test]
    fn what_trails_the_face_is_exposed_until_the_sweep_reaches_it() {
        // A face 2 wide, open upward, moving 10 along -u from u 0.
        let face = [[-1., 0.], [1., 0.], [1., 1.], [-1., 1.]];
        let shield = [Shield::new(&face, [0., 1.], 0.).unwrap()];
        let square = |u: f64, v: f64| vec![[u, v], [u + 1., v], [u + 1., v + 1.], [u, v + 1.]];
        let run = |body: &[P2], trailing: bool| {
            let end: Vec<P2> = body.iter().map(|p| [p[0] - 10., p[1]]).collect();
            exposed_line(body, &end, &shield, [0., 0.], [-10., 0.], trailing)
        };
        let extent = |polygons: &[Vec<P2>]| {
            polygons
                .iter()
                .flatten()
                .fold([f64::INFINITY, f64::NEG_INFINITY], |e, p| {
                    [e[0].min(p[0]), e[1].max(p[0])]
                })
        };
        // Above the face: never exposed.
        assert!(run(&square(-0.5, 3.), true).is_empty());
        // 3 behind the face's back (u 4..5): exposed from its start until
        // its front (u 4) reaches the face's first place (u 1) ...
        let behind = extent(&run(&square(4., 3.), true));
        assert!(
            (behind[0] - 1.).abs() < 1e-9 && (behind[1] - 5.).abs() < 1e-9,
            "{behind:?}"
        );
        // ... and all along when only the face's own place counts.
        let behind = extent(&run(&square(4., 3.), false));
        assert!(
            (behind[0] + 6.).abs() < 1e-9 && (behind[1] - 5.).abs() < 1e-9,
            "{behind:?}"
        );
        // Ahead of the face: all along.
        let ahead = extent(&run(&square(-4., 3.), true));
        assert!(
            (ahead[0] + 14.).abs() < 1e-9 && (ahead[1] + 3.).abs() < 1e-9,
            "{ahead:?}"
        );
        // Below the face (not its open side): all along, even at its u.
        let below = extent(&run(&square(-0.5, -3.), true));
        assert!(
            (below[0] + 10.5).abs() < 1e-9 && (below[1] - 0.5).abs() < 1e-9,
            "{below:?}"
        );
    }

    /// Without a parametrised move, the body less the face's own place.
    #[test]
    fn the_fixed_exposed_part_is_the_body_less_the_shields() {
        let face = [[-1., 0.], [1., 0.], [1., 1.], [-1., 1.]];
        let shield = [Shield::new(&face, [0., 1.], 0.01).unwrap()];
        // A body 4 wide over the face: its parts beside the face (beyond
        // the guard) are exposed, nothing above the face.
        let body = [[-2., 0.5], [2., 0.5], [2., 2.], [-2., 2.]];
        let parts = exposed_fixed(&body, &shield);
        assert!(!parts.is_empty());
        for part in &parts {
            for p in part {
                assert!(p[0].abs() >= 1.01 - 1e-9, "{p:?}");
            }
        }
        let area: f64 = parts
            .iter()
            .map(|p| {
                (0..p.len())
                    .map(|i| {
                        let (a, b) = (p[i], p[(i + 1) % p.len()]);
                        (a[0] * b[1] - b[0] * a[1]) / 2.
                    })
                    .sum::<f64>()
            })
            .sum();
        assert!((area - 2. * 0.99 * 1.5).abs() < 1e-6, "{area}");
        // A body inside the face's column: nothing.
        let body = [[-0.5, 0.5], [0.5, 0.5], [0.5, 9.], [-0.5, 9.]];
        assert!(exposed_fixed(&body, &shield).is_empty());
    }
}
