//! Whether committed cutter sweeps contain the sweep of a tool section
//! (#200): the shank of a move deeper than the flutes, passing where the
//! layers above have cut.
//!
//! # The claim
//!
//! A committed sweep `i` is a flat cutter of radius `r_i` that followed the
//! XY path `P_i` at the floor `f_i`: the column over `C_i = P_i + D(r_i)`
//! (the Minkowski sum with the closed disk: every point within `r_i` of the
//! path) is empty from `f_i` up, taken as from `f_i + e_i` with the sweep's
//! path error `e_i`. A section of radius `R` whose bottom stays at or above
//! `z` along the XY path `Q` sweeps `W = Q + D(R)`. It meets no stock if
//!
//! ```text
//! W  is inside  U  or outside the stock's XY extent,
//! U = the union of the C_i with f_i + e_i <= z
//! ```
//!
//! that is, if `Q` lies in the erosion of that set by `D(R)`. With `R`
//! equal to the cutter radius, `W` touches the boundary of `U` wherever `Q`
//! runs on the path of an outermost pass: the containment holds with no
//! margin at all, which no test with a positive numeric margin can show.
//! Covering the boundary of `W` would not do either: three cutter disks
//! round a point leave an uncut island between them, so the inside of `W`
//! is proven too.
//!
//! # The proof
//!
//! `W` is split into pieces, each proven by one of two certificates.
//!
//! 1. **Path identity (no margin, exact).** If `q` is a point of `P_i` and
//!    `R <= r_i`, then `D(q, R)` is inside `D(q, r_i)`, which is inside
//!    `C_i`. So a stretch of `Q` that lies on eligible paths of radius at
//!    least `R` is proven with its whole swept disk, by exact predicates on
//!    the declared coordinates and no tolerance: collinearity by
//!    `robust::orient2d`, extent by comparing one coordinate (monotone along
//!    a line). A line may be covered by several collinear sweeps; an arc
//!    only by the identical arc (`ClearedSweep::repeats`). The same declared
//!    geometry is the same nominal curve, so the sweep's path error is
//!    shared, not added twice.
//!
//! 2. **Convex tiles (a margin of the sweeps' own error).** What remains is
//!    cut into convex polygons in the path's own frame: for a line the
//!    rectangle `|u| <= R` across the uncovered stretch, for an arc annular
//!    sectors (each enclosed in a trapezoid: inner chord, outer tangent), and
//!    a full disk at each end of `Q` not proven by (1). Every `x` in `W` has
//!    a nearest point on `Q`: on a stretch proven by (1), in the interior of
//!    an uncovered stretch (then `x` is in its rectangle or sector) or at an
//!    end of `Q` (then in that end's disk); so the pieces cover `W`.
//!    A polygon, clipped to the stock's XY extent, needs ONE sweep holding
//!    it whole: all its vertices within `h_i = r_i - e_i - g` of a line path
//!    (the capsule is convex, so the vertices decide), or inside the annulus
//!    and angular span of an arc path, or its bounding ball inside the
//!    sweep. An unproven polygon is halved, along the path or across it.
//!    The section is grown by its own path error `e_Q` and each sweep is
//!    shrunk by its error `e_i` (the same `path_error` the stock sweep
//!    commits with) and a rounding guard `g`. No other allowance exists:
//!    off the sweeps' own paths, a section whose edge comes within
//!    `e_Q + e_i` of the edge of what they cut is not proven.
//!
//! Tiles follow the path, so a section sliding along a wall a hair inside
//! the cut above costs a few tiles, not the path length over the clearance.
//! A polygon whose centre is outside every sweep ends the proof at once; so
//! do the tile and depth limits. Unproven is `false`, never an error.
use super::{ClearedSweep, Shape, distance, radius_ok, shape_distance};
use crate::{
    Point,
    motion::{Curve, Plane},
    stock::CancelSignal,
};
use std::f64::consts::{FRAC_PI_2, PI, TAU};

/// Polygons examined per query. A provable section needs tens to a few
/// hundred; this bounds one that is not provable and not refuted at once.
const TILES: u32 = 16_384;
const DEPTH: u8 = 60;

/// A tool section along a move: everything within `radius` of the curve's
/// XY path, whose bottom is never below `bottom`. `error` is the move's own
/// path error.
pub(crate) struct Section<'a> {
    pub curve: &'a Curve,
    pub radius: f64,
    pub bottom: f64,
    pub error: f64,
}

#[derive(Clone, Copy)]
enum Tile {
    /// `o + s d + u n` for `s` in `[s0, s1]`, `u` in `[u0, u1]`.
    Rect {
        o: Point,
        d: Point,
        s: [f64; 2],
        u: [f64; 2],
    },
    /// Polar about `c`: angle in `t` (at most a quarter turn), radius in `r`.
    Sector { c: Point, t: [f64; 2], r: [f64; 2] },
}
impl Tile {
    /// A convex polygon holding the tile.
    fn polygon(&self) -> Vec<Point> {
        match *self {
            Tile::Rect { o, d, s, u } => {
                let at = |s: f64, u: f64| [o[0] + s * d[0] - u * d[1], o[1] + s * d[1] + u * d[0]];
                vec![
                    at(s[0], u[0]),
                    at(s[1], u[0]),
                    at(s[1], u[1]),
                    at(s[0], u[1]),
                ]
            }
            Tile::Sector { c, t, r } => {
                // The outer arc lies inside the chord of the circle of
                // radius r1 / cos(half angle), tangent to it at mid-angle;
                // the inner arc lies outside its own chord.
                let outer = r[1] / ((t[1] - t[0]) / 2.).cos();
                let at = |t: f64, r: f64| [c[0] + r * t.cos(), c[1] + r * t.sin()];
                if r[0] == 0. {
                    vec![c, at(t[0], outer), at(t[1], outer)]
                } else {
                    vec![
                        at(t[0], r[0]),
                        at(t[0], outer),
                        at(t[1], outer),
                        at(t[1], r[0]),
                    ]
                }
            }
        }
    }
    fn extent(&self) -> f64 {
        match *self {
            Tile::Rect { s, u, .. } => (s[1] - s[0]).max(u[1] - u[0]),
            Tile::Sector { t, r, .. } => (r[1] * (t[1] - t[0])).max(r[1] - r[0]),
        }
    }
    fn split(&self) -> [Tile; 2] {
        let halves = |v: [f64; 2]| {
            let m = (v[0] + v[1]) / 2.;
            [[v[0], m], [m, v[1]]]
        };
        match *self {
            Tile::Rect { o, d, s, u } => {
                if s[1] - s[0] >= u[1] - u[0] {
                    halves(s).map(|s| Tile::Rect { o, d, s, u })
                } else {
                    halves(u).map(|u| Tile::Rect { o, d, s, u })
                }
            }
            Tile::Sector { c, t, r } => {
                if r[1] * (t[1] - t[0]) >= r[1] - r[0] {
                    halves(t).map(|t| Tile::Sector { c, t, r })
                } else {
                    halves(r).map(|r| Tile::Sector { c, t, r })
                }
            }
        }
    }
}

/// The full disk about `p`, as four quarter sectors.
fn disk(p: Point, radius: f64, tiles: &mut Vec<(Tile, u8)>) {
    for i in 0..4 {
        tiles.push((
            Tile::Sector {
                c: p,
                t: [i as f64 * FRAC_PI_2, (i + 1) as f64 * FRAC_PI_2],
                r: [0., radius],
            },
            0,
        ));
    }
}

/// `polygon` (convex) cut to the closed rectangle `stock`.
fn clip(mut polygon: Vec<Point>, stock: [Point; 2]) -> Vec<Point> {
    for (k, bound, keep_above) in [
        (0, stock[0][0], true),
        (0, stock[1][0], false),
        (1, stock[0][1], true),
        (1, stock[1][1], false),
    ] {
        let inside = |p: &Point| {
            if keep_above {
                p[k] >= bound
            } else {
                p[k] <= bound
            }
        };
        let mut next = Vec::with_capacity(polygon.len() + 2);
        for i in 0..polygon.len() {
            let (a, b) = (polygon[i], polygon[(i + 1) % polygon.len()]);
            if inside(&a) {
                next.push(a);
            }
            if inside(&a) != inside(&b) {
                let t = (bound - a[k]) / (b[k] - a[k]);
                let mut p = [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
                p[k] = bound;
                next.push(p);
            }
        }
        polygon = next;
        if polygon.is_empty() {
            break;
        }
    }
    polygon
}

fn cross(a: Point, b: Point) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

/// The distance from `c` to the convex polygon (zero inside it).
fn polygon_distance(polygon: &[Point], c: Point) -> f64 {
    let n = polygon.len();
    let mut nearest = f64::INFINITY;
    let (mut left, mut right) = (true, true);
    for i in 0..n {
        let (a, b) = (polygon[i], polygon[(i + 1) % n]);
        let e = [b[0] - a[0], b[1] - a[1]];
        let w = [c[0] - a[0], c[1] - a[1]];
        let side = cross(e, w);
        left &= side >= 0.;
        right &= side <= 0.;
        let l = e[0] * e[0] + e[1] * e[1];
        let t = if l > 0. {
            ((w[0] * e[0] + w[1] * e[1]) / l).clamp(0., 1.)
        } else {
            0.
        };
        nearest = nearest.min(distance(c, [a[0] + t * e[0], a[1] + t * e[1]]));
    }
    if n >= 3 && (left || right) {
        0.
    } else {
        nearest
    }
}

/// Whether the sweep holds the whole convex polygon, by `guard` at least
/// inside its radius less its own error.
fn holds(s: &ClearedSweep, polygon: &[Point], guard: f64) -> bool {
    let h = s.radius - s.error - guard;
    if h <= 0. {
        return false;
    }
    let Shape::Arc {
        c,
        radius,
        angle,
        sweep,
        ..
    } = s.shape
    else {
        // A capsule is convex: its vertices decide.
        return polygon.iter().all(|&v| shape_distance(&s.shape, v) <= h);
    };
    // The bounding ball inside the tube: the distance to a set is
    // 1-Lipschitz.
    let n = polygon.len() as f64;
    let mid = [
        polygon.iter().map(|p| p[0]).sum::<f64>() / n,
        polygon.iter().map(|p| p[1]).sum::<f64>() / n,
    ];
    let extent = polygon.iter().map(|&v| distance(mid, v)).fold(0., f64::max);
    if shape_distance(&s.shape, mid) + extent + guard <= h {
        return true;
    }
    // Inside the annulus (the outer disk is convex, so the vertices decide;
    // the inner bound is the polygon's distance from the centre) ...
    let full = sweep.abs() >= TAU;
    if polygon.iter().any(|&v| distance(v, c) > radius + h) {
        return false;
    }
    if full && radius <= h {
        return true;
    }
    if radius <= h || polygon_distance(polygon, c) < radius - h {
        return false;
    }
    if full {
        return true;
    }
    // ... and inside the arc's angular span, where the nearest point of the
    // path is the radial one. A span of at most a half turn is a convex
    // cone (two half-planes); a wider one is tried as three such cones.
    let start = if sweep >= 0. { angle } else { angle + sweep };
    let span = sweep.abs();
    let pad = s.error + guard;
    let within = |from: f64, to: f64| {
        let d0 = [(start + from).cos(), (start + from).sin()];
        let d1 = [(start + to).cos(), (start + to).sin()];
        polygon.iter().all(|&v| {
            let w = [v[0] - c[0], v[1] - c[1]];
            cross(d0, w) >= pad && cross(w, d1) >= pad
        })
    };
    if span <= PI {
        within(0., span)
    } else {
        within(0., span / 2.) || within(span / 2., span) || within(span / 4., 3. * span / 4.)
    }
}

/// Whether the sweeps in `supports` prove the section clear of stock inside
/// the XY rectangle `stock` (module documentation: the claim and its proof).
/// `supports` must hold every committed sweep whose reach meets the
/// section's; others only cost time.
pub(crate) fn contains(
    supports: &[&ClearedSweep],
    section: Section<'_>,
    stock: [Point; 2],
    work: &mut u64,
    cancel: &impl CancelSignal,
) -> Result<bool, String> {
    let Section {
        curve,
        radius,
        bottom,
        error,
    } = section;
    if cancel.is_cancelled() {
        return Err("swept clearance proof cancelled".into());
    }
    if *work == 0 || !radius_ok(radius) || !bottom.is_finite() || !error.is_finite() || error < 0. {
        return Ok(false);
    }
    *work -= 1;
    // As `ClearedSweep::repeats`: a sweep's column is empty from its floor
    // plus its error.
    let eligible: Vec<&ClearedSweep> = supports
        .iter()
        .copied()
        .filter(|s| bottom >= s.floor + s.error)
        .collect();
    if eligible.iter().any(|s| s.repeats(curve, radius, bottom)) {
        return Ok(true);
    }
    let (a3, b3) = (curve.start(), curve.end());
    let (a, b) = ([a3[0], a3[1]], [b3[0], b3[1]]);
    let reach = radius + error;
    let mut scale = reach;
    for v in a.into_iter().chain(b).chain(stock[0]).chain(stock[1]) {
        scale = scale.max(v.abs());
    }
    let mut tiles: Vec<(Tile, u8)> = vec![];
    match *curve {
        Curve::Line { .. } if a == b => {
            if eligible.iter().any(|s| radius <= s.radius && s.on_path(a)) {
                return Ok(true);
            }
            disk(a, reach, &mut tiles);
        }
        Curve::Line { .. } => {
            // One coordinate is strictly monotone along the line: the
            // stretches of Q on collinear sweeps are intervals of it, and
            // comparing them is exact.
            let k = usize::from((b[1] - a[1]).abs() > (b[0] - a[0]).abs());
            let (lo, hi) = (a[k].min(b[k]), a[k].max(b[k]));
            let mut covered: Vec<[f64; 2]> = eligible
                .iter()
                .filter(|s| radius <= s.radius)
                .filter_map(|s| s.collinear_extent(a, b, k))
                .map(|[x, y]| [x.max(lo), y.min(hi)])
                .filter(|[x, y]| x <= y)
                .collect();
            covered.sort_by(|x, y| x[0].total_cmp(&y[0]));
            // Uncovered stretches, and whether each end is a covered point.
            let mut gaps: Vec<([f64; 2], [bool; 2])> = vec![];
            let (mut cursor, mut held) = (lo, false);
            for [x, y] in covered {
                if x > cursor {
                    gaps.push(([cursor, x], [held, true]));
                }
                if y >= cursor {
                    cursor = y;
                    held = true;
                }
            }
            if cursor < hi {
                gaps.push(([cursor, hi], [held, false]));
            }
            if gaps.is_empty() {
                return Ok(true);
            }
            let length = distance(a, b);
            let d = [(b[0] - a[0]) / length, (b[1] - a[1]) / length];
            for (gap, held) in gaps {
                // Path parameters of the gap's ends. An end at a covered
                // point is moved a little into the covered stretch, far
                // more than the rounding of the parameter; an end of Q
                // itself gets its disk.
                let mut t = [0, 1].map(|i| {
                    let t = (gap[i] - a[k]) / (b[k] - a[k]);
                    if held[i] {
                        t
                    } else {
                        disk(if gap[i] == a[k] { a } else { b }, reach, &mut tiles);
                        t.round()
                    }
                });
                if t[0] > t[1] {
                    t.swap(0, 1);
                }
                let pad = 1e-9;
                tiles.push((
                    Tile::Rect {
                        o: a,
                        d,
                        s: [(t[0] - pad).max(0.) * length, (t[1] + pad).min(1.) * length],
                        u: [-reach, reach],
                    },
                    0,
                ));
            }
        }
        Curve::Arc {
            center,
            plane: Plane::Xy,
            sweep_rad,
            ..
        }
        | Curve::Helix {
            center,
            plane: Plane::Xy,
            sweep_rad,
            ..
        } => {
            let c = [center[0], center[1]];
            let rho = distance(a, c);
            if !rho.is_finite() || !sweep_rad.is_finite() {
                return Ok(false);
            }
            scale = scale.max(c[0].abs()).max(c[1].abs()).max(rho + reach);
            let span = sweep_rad.abs().min(TAU);
            let angle = (a[1] - c[1]).atan2(a[0] - c[0]);
            let start = if sweep_rad >= 0. { angle } else { angle - span };
            let pieces = (span / FRAC_PI_2).ceil().max(1.);
            for i in 0..pieces as usize {
                tiles.push((
                    Tile::Sector {
                        c,
                        t: [
                            start + span * i as f64 / pieces,
                            start + span * (i + 1) as f64 / pieces,
                        ],
                        r: [(rho - reach).max(0.), rho + reach],
                    },
                    0,
                ));
            }
            // The declared end is within the path error of the analytic
            // one, and `reach` includes that error.
            disk(a, reach, &mut tiles);
            disk(b, reach, &mut tiles);
        }
        _ => return Ok(false),
    }
    let guard = 4096. * f64::EPSILON * (1. + scale);
    let mut budget = TILES;
    let mut hint = 0;
    while let Some((tile, depth)) = tiles.pop() {
        if cancel.is_cancelled() {
            return Err("swept clearance proof cancelled".into());
        }
        if budget == 0 || *work == 0 {
            return Ok(false);
        }
        budget -= 1;
        *work -= 1;
        let polygon = clip(tile.polygon(), stock);
        if polygon.is_empty() {
            continue;
        }
        let mut lo = [f64::INFINITY; 2];
        let mut hi = [f64::NEG_INFINITY; 2];
        let mut mid = [0.; 2];
        for p in &polygon {
            for k in 0..2 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
                mid[k] += p[k] / polygon.len() as f64;
            }
        }
        // A sweep holding the polygon holds its box within its own.
        let may_hold =
            |s: &ClearedSweep| (0..2).all(|k| lo[k] >= s.bounds[0][k] && hi[k] <= s.bounds[1][k]);
        let n = eligible.len();
        let witness = (0..n).map(|i| (hint + i) % n).find(|&i| {
            let s = eligible[i];
            if !may_hold(s) {
                return false;
            }
            *work = work.saturating_sub(1);
            holds(s, &polygon, guard)
        });
        if let Some(i) = witness {
            hint = i;
            continue;
        }
        // The polygon's centre (inside it, and inside the stock) outside
        // every sweep: no subdivision can prove this section.
        if eligible
            .iter()
            .all(|s| shape_distance(&s.shape, mid) > s.radius + s.error + guard)
        {
            return Ok(false);
        }
        if depth >= DEPTH || tile.extent() <= 8. * guard {
            return Ok(false);
        }
        for half in tile.split() {
            tiles.push((half, depth + 1));
        }
    }
    Ok(true)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::{Motion, MoveKind};

    const STOCK: [Point; 2] = [[0., 0.], [200., 100.]];
    /// The passes in these tests cut at Z5; a section's bottom a little
    /// above that floor is over emptied columns.
    const B: f64 = 5.1;

    fn motion(curve: Curve) -> Motion {
        Motion {
            id: 1,
            operation: "op".into(),
            tool: "t".into(),
            kind: MoveKind::Cut,
            curve,
            feed_mm_min: Some(600.),
        }
    }
    fn line(a: Point, b: Point, z: f64) -> Motion {
        motion(Curve::Line {
            start: [a[0], a[1], z],
            end: [b[0], b[1], z],
        })
    }
    fn arc(a: Point, b: Point, c: Point, sweep: f64, z: f64) -> Motion {
        motion(Curve::Arc {
            start: [a[0], a[1], z],
            end: [b[0], b[1], z],
            center: [c[0], c[1], z],
            plane: Plane::Xy,
            sweep_rad: sweep,
        })
    }
    fn sweeps(cuts: &[(Motion, f64)]) -> Vec<ClearedSweep> {
        cuts.iter()
            .map(|(m, r)| ClearedSweep::from_motion(m, *r, 1e-5).unwrap())
            .collect()
    }
    /// The proof, and the tiles it took.
    fn proof(cuts: &[ClearedSweep], m: &Motion, radius: f64, bottom: f64) -> (bool, u64) {
        let mut work = 1 << 40;
        let proven = contains(
            &cuts.iter().collect::<Vec<_>>(),
            Section {
                curve: &m.curve,
                radius,
                bottom,
                error: crate::verification::path_error(m, 1e-5),
            },
            STOCK,
            &mut work,
            &|| false,
        )
        .unwrap();
        (proven, (1 << 40) - work)
    }
    fn proven(cuts: &[ClearedSweep], m: &Motion, radius: f64, bottom: f64) -> bool {
        proof(cuts, m, radius, bottom).0
    }
    /// Independent of the proof: sample the section's sweep densely and
    /// test each point against every sweep's footprint, by the distance
    /// from the point to the cut's path written out here.
    fn sampled(cuts: &[(Motion, f64)], m: &Motion, radius: f64, bottom: f64, slack: f64) -> bool {
        let at = |m: &Motion, t: f64| -> Point {
            match m.curve {
                Curve::Line { start, end } => [
                    start[0] + t * (end[0] - start[0]),
                    start[1] + t * (end[1] - start[1]),
                ],
                Curve::Arc {
                    start,
                    center,
                    sweep_rad,
                    ..
                } => {
                    let r = (start[0] - center[0]).hypot(start[1] - center[1]);
                    let angle = (start[1] - center[1]).atan2(start[0] - center[0]) + t * sweep_rad;
                    [center[0] + r * angle.cos(), center[1] + r * angle.sin()]
                }
                _ => unreachable!(),
            }
        };
        let away = |cut: &Motion, x: Point| -> f64 {
            let ends = distance(at(cut, 0.), x).min(distance(at(cut, 1.), x));
            match cut.curve {
                Curve::Line { start, end } => {
                    let e = [end[0] - start[0], end[1] - start[1]];
                    let w = [x[0] - start[0], x[1] - start[1]];
                    let t = (w[0] * e[0] + w[1] * e[1]) / (e[0] * e[0] + e[1] * e[1]);
                    if (0. ..=1.).contains(&t) {
                        (w[0] * e[1] - w[1] * e[0]).abs() / e[0].hypot(e[1])
                    } else {
                        ends
                    }
                }
                Curve::Arc {
                    start,
                    center,
                    sweep_rad,
                    ..
                } => {
                    let r = (start[0] - center[0]).hypot(start[1] - center[1]);
                    let from = (start[1] - center[1]).atan2(start[0] - center[0]);
                    let turn = ((x[1] - center[1]).atan2(x[0] - center[0]) - from).rem_euclid(TAU);
                    let on = if sweep_rad >= 0. {
                        turn <= sweep_rad
                    } else {
                        TAU - turn <= -sweep_rad
                    };
                    if on {
                        ((x[0] - center[0]).hypot(x[1] - center[1]) - r).abs()
                    } else {
                        ends
                    }
                }
                _ => unreachable!(),
            }
        };
        (0..=400).all(|i| {
            let q = at(m, i as f64 / 400.);
            (0..=6).all(|ring| {
                (0..72).all(|j| {
                    let angle = TAU * j as f64 / 72.;
                    let rho = radius * ring as f64 / 6.;
                    let x = [q[0] + rho * angle.cos(), q[1] + rho * angle.sin()];
                    (0..2).any(|k| x[k] < STOCK[0][k] || x[k] > STOCK[1][k])
                        || cuts.iter().any(|(cut, r)| {
                            cut.curve.start()[2] <= bottom && away(cut, x) <= r + slack
                        })
                })
            })
        })
    }

    /// The issue's case: a section as wide as the cutter, where the layers
    /// above cut, on a path that repeats none of theirs.
    #[test]
    fn an_equal_radius_section_inside_the_passes_above_is_proven() {
        // Three passes of a 12 mm cutter at Z5, 6 mm apart: a wall pass at
        // y = 40 (stock beyond y = 46 is uncut) and two further out.
        let cuts = [
            (line([20., 40.], [150., 40.], 5.), 6.),
            (line([20., 34.], [150., 34.], 5.), 6.),
            (line([20., 28.], [150., 28.], 5.), 6.),
        ];
        let s = sweeps(&cuts);
        // 2 um, 0.05 mm and 2 mm inside the wall pass (a drafted wall).
        for inside in [0.002, 0.05, 2.] {
            let m = line([30., 40. - inside], [140., 40. - inside], 1.);
            let (ok, tiles) = proof(&s, &m, 6., B);
            assert!(ok, "{inside} mm inside the wall pass");
            assert!(tiles < 2_000, "{tiles} tiles for {inside} mm");
            assert!(sampled(&cuts, &m, 6., B, 1e-9));
        }
        // A hair towards the wall: the section would enter uncut stock.
        for outside in [1e-4, 0.05] {
            let m = line([30., 40. + outside], [140., 40. + outside], 1.);
            assert!(!proven(&s, &m, 6., B), "{outside} mm into the wall");
            assert!(!sampled(&cuts, &m, 6., B, 0.));
        }
        // Oblique across the passes, staying inside them.
        let m = line([40., 30.], [130., 38.], 1.);
        assert!(proven(&s, &m, 6., B));
        assert!(sampled(&cuts, &m, 6., B, 1e-9));
        // The same, running out of the cut at its far end.
        let m = line([40., 30.], [152., 38.], 1.);
        assert!(!proven(&s, &m, 6., B));
        assert!(!sampled(&cuts, &m, 6., B, 0.));
        // Below the floor the passes cut, nothing is proven.
        let m = line([30., 39.], [140., 39.], 1.);
        assert!(proven(&s, &m, 6., B) && !proven(&s, &m, 6., 4.999));
        // Nor at the floor itself: a sweep proves its column empty from
        // its floor plus its error.
        assert!(!proven(&s, &m, 6., 5.) && proven(&s, &m, 6., 5.0001));
        // A section wider than the cutter, 2 um inside the wall pass, is
        // not: it would enter the wall.
        let m = line([30., 39.998], [140., 39.998], 1.);
        assert!(proven(&s, &m, 6., B) && !proven(&s, &m, 6.01, B));
    }

    /// #160 (the scenarios of the proof this one replaces,
    /// `EngagementScene::clear_path`): a section is proven against the
    /// union of the committed sweeps along the move's own path, not against
    /// one sweep it repeats.
    #[test]
    fn a_swept_section_is_proven_by_the_union_of_committed_sweeps() {
        // Two overlapping diagonal passes at Z5, cutter radius 5.
        let a = line([20., 20.], [80., 60.], 5.);
        let b = line([70., 55.], [150., 30.], 5.);
        let one = sweeps(&[(a.clone(), 5.)]);
        let s = sweeps(&[(a.clone(), 5.), (b.clone(), 5.)]);
        // A section a little slimmer than the cutter along part of one pass.
        let along = line([30., 26.666666666666668], [60., 46.666666666666664], 5.);
        assert!(proven(&s, &along, 4.9, B));
        // A path from one pass into the other: the first sweep alone does
        // not hold it, the union does.
        let across = line([74., 56.], [78., 52.5], 5.);
        assert!(!proven(&one, &across, 2., B));
        assert!(proven(&s, &across, 2., B));
        assert!(proven(&s, &across, 2., 7.));
        // Below the floor the sweeps cleared, nothing is proven.
        assert!(!proven(&s, &across, 2., 4.9));
        // Wider than what was cut, or off the cut: refused.
        assert!(!proven(&s, &along, 5.01, B));
        assert!(!proven(&s, &line([30., 30.], [60., 50.], 5.), 5., B));
        // Outside the stock everything is air.
        assert!(proven(&s, &line([-30., -20.], [-10., -20.], 5.), 5., 0.));
        // No work, no proof.
        let all: Vec<_> = s.iter().collect();
        let section = Section {
            curve: &a.curve,
            radius: 5.,
            bottom: B,
            error: 2e-5,
        };
        assert!(!contains(&all, section, STOCK, &mut 0, &|| false).unwrap());
    }

    /// No margin at all: the section runs on the passes' own paths, across
    /// their joints, and is proven by exact path identity.
    #[test]
    fn a_path_on_several_collinear_sweeps_is_proven_without_margin() {
        // One wall pass committed as three moves (0.1 + 0.2 is not 0.3:
        // the joints are whatever the planner produced).
        let y = 0.1 + 0.2;
        let cuts = [
            (line([20., y], [50.5, y], 5.), 6.),
            (line([50.5, y], [90.25, y], 5.), 6.),
            (line([120., y], [90.25, y], 5.), 6.),
        ];
        let s = sweeps(&cuts);
        // Across both joints; a diagonal one too.
        assert!(proven(&s, &line([30., y], [110., y], 1.), 6., B));
        assert!(proven(&s, &line([110., y], [30., y], 1.), 6., B));
        assert!(proven(&s, &line([20., y], [120., y], 3.), 6., B));
        let d = [
            (line([10., 10.], [30., 20.], 5.), 6.),
            (line([30., 20.], [70., 40.], 5.), 6.),
        ];
        assert!(proven(
            &sweeps(&d),
            &line([20., 15.], [60., 35.], 1.),
            6.,
            B
        ));
        // A vertical move (a plunge) at a point of a pass.
        assert!(proven(&s, &line([50.5, y], [50.5, y], 1.), 6., B));
        // Beyond the passes' end, or with a gap between them: not proven.
        assert!(!proven(&s, &line([30., y], [120.001, y], 1.), 6., B));
        assert!(!proven(&s[..1], &line([30., y], [60., y], 1.), 6., B));
        let gap = [cuts[0].clone(), cuts[2].clone()];
        assert!(!proven(
            &sweeps(&gap),
            &line([30., y], [110., y], 1.),
            6.,
            B
        ));
        // One ulp off the line is not on it, and has no margin either.
        let off = f64::from_bits(y.to_bits() + 1);
        assert!(!proven(&s, &line([30., off], [110., off], 1.), 6., B));
        // A wider section, or one below the floor.
        assert!(!proven(&s, &line([30., y], [110., y], 1.), 6.000001, B));
        assert!(!proven(&s, &line([30., y], [110., y], 1.), 6., 4.9));
        // Partly on the pass, then off into a region two other passes cut.
        let more = [
            cuts[0].clone(),
            (line([50.5, y], [50.5, 60.], 5.), 6.),
            (line([56., -3.], [56., 60.], 5.), 6.),
            (line([62., -3.], [62., 60.], 5.), 6.),
        ];
        let turn = line([30., y], [57., y], 1.);
        assert!(proven(&sweeps(&more), &turn, 6., B));
        assert!(sampled(&more, &turn, 6., B, 1e-9));
        assert!(!proven(&sweeps(&more[..2]), &turn, 6., B));
    }

    /// Arcs: the corner of a drafted wall, a little inside the corner arc
    /// of the layer above, with the passes further out.
    #[test]
    fn an_arc_inside_the_arcs_above_is_proven() {
        let corner =
            |c: Point, r: f64, z: f64| arc([c[0] + r, c[1]], [c[0], c[1] + r], c, FRAC_PI_2, z);
        // Corner arcs about (100, 50), 8 and 14 mm, with their leading
        // and trailing lines.
        let cuts = [
            (corner([100., 50.], 8., 5.), 6.),
            (corner([100., 50.], 14., 5.), 6.),
            (line([108., 20.], [108., 50.], 5.), 6.),
            (line([114., 20.], [114., 50.], 5.), 6.),
            (line([100., 58.], [60., 58.], 5.), 6.),
            (line([100., 64.], [60., 64.], 5.), 6.),
        ];
        let s = sweeps(&cuts);
        for inside in [0.02, 0.5] {
            // The part's corner moved out by `inside` on both axes.
            let m = corner([100. + inside, 50. + inside], 8., 1.);
            let (ok, tiles) = proof(&s, &m, 6., B);
            assert!(ok, "{inside} mm");
            assert!(tiles < 4_000, "{tiles} tiles for {inside} mm");
            assert!(sampled(&cuts, &m, 6., B, 1e-9));
        }
        // Towards the corner: into uncut stock.
        let m = corner([100. - 0.02, 50. - 0.02], 8., 1.);
        assert!(!proven(&s, &m, 6., B));
        assert!(!sampled(&cuts, &m, 6., B, 0.));
        // The identical arc is the exact repeat.
        assert!(proven(&s, &corner([100., 50.], 8., 1.), 6., B));
        // A helix down the same circle is not an exact repeat and has no
        // margin; one inside both arcs' cut has.
        let helix = |r: f64| {
            motion(Curve::Helix {
                start: [100. + r, 50., 9.],
                end: [100., 50. + r, 1.],
                center: [100., 50., 9.],
                plane: Plane::Xy,
                sweep_rad: FRAC_PI_2,
            })
        };
        assert!(!proven(&s, &helix(8.), 6., B));
        assert!(proven(&s, &helix(11.), 6., B));
    }

    /// Three cutter disks round a point leave an island between them: the
    /// boundary of a section's disk can be covered while its inside is not.
    #[test]
    fn an_uncut_island_inside_the_section_is_found() {
        let dot = |p: Point| (line(p, [p[0] + 1e-3, p[1]], 5.), 6.);
        let r = 6.6;
        let cuts: Vec<_> = (0..3)
            .map(|i| {
                let a = TAU * i as f64 / 3.;
                dot([100. + r * a.cos(), 50. + r * a.sin()])
            })
            .collect();
        let s = sweeps(&cuts);
        // The centre (100, 50) is 6.6 mm from each: uncut. A 1 mm section
        // there is inside the island; a 4 mm one has its edge covered.
        let plunge = line([100., 50.], [100., 50.], 1.);
        assert!(!proven(&s, &plunge, 1., B));
        assert!(!proven(&s, &plunge, 4., B));
        // With the centre cut too, it is proven.
        let mut all = cuts.clone();
        all.push(dot([100., 50.]));
        assert!(proven(&sweeps(&all), &plunge, 4., B));
    }

    #[test]
    fn outside_the_stock_is_air_and_limits_prove_nothing() {
        let cuts = [(line([-10., 40.], [50., 40.], 5.), 6.)];
        let s = sweeps(&cuts);
        // Wholly outside the stock: nothing to meet.
        assert!(proven(&s, &line([-30., -20.], [-10., -20.], 1.), 6., 0.));
        // Entering along the cut from outside.
        assert!(proven(&s, &line([-30., 39.9], [40., 39.9], 1.), 5.8, B));
        let m = line([10., 39.5], [40., 39.5], 1.);
        let all: Vec<_> = s.iter().collect();
        let section = || Section {
            curve: &m.curve,
            radius: 5.,
            bottom: B,
            error: 2e-5,
        };
        assert!(contains(&all, section(), STOCK, &mut 1_000, &|| false).unwrap());
        assert!(!contains(&all, section(), STOCK, &mut 1, &|| false).unwrap());
        assert!(contains(&all, section(), STOCK, &mut 1_000, &|| true).is_err());
        assert!(!contains(&[], section(), STOCK, &mut 1_000, &|| false).unwrap());
    }
}
