//! Rapids kept off parts that are already cut free.
//!
//! A part or a hole's slug that has been cut free can tilt between the
//! table's slats and stand above the sheet; a torch that then travels across
//! it hits it. With `avoid_cut_parts` every rapid between cuts keeps the
//! declared clearance from every piece cut free earlier, by going round them
//! (Hypertherm, "ProNest collision avoidance": path the traverse around
//! previously cut parts). Where no such path is found the cut is refused,
//! naming the rapid: nothing travels over a freed piece unsaid.
//!
//! What counts as freed: a hole's slug once the hole is cut, a part once its
//! outline is cut; neither when micro-joints hold it. The piece just cut is
//! where the torch stands: the first leg may leave across a slug it stands
//! over, and may pass nearer than the clearance to the part it has just cut
//! round, but never across that part.
//!
//! Pieces are polygons with their arcs as chords; each chord's sagitta is
//! added to the clearance, so the distance kept from the true outline is
//! never less than declared.
use crate::planar::Boundary;
use serde::{Deserialize, Serialize};

type P2 = [f64; 2];

/// Keep rapids `clearance_mm` (measured from the beam centre: include the
/// torch body's radius) from every piece already cut free.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AvoidCutParts {
    pub clearance_mm: f64,
}

struct Piece {
    polygon: Vec<P2>,
    /// The most a chord stands inside its arc.
    sag: f64,
    bounds: [f64; 4],
}

#[derive(Default)]
pub(crate) struct Freed {
    pieces: Vec<Piece>,
}

fn cross(a: P2, b: P2, c: P2) -> f64 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

fn point_segment(p: P2, a: P2, b: P2) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let length2 = dx * dx + dy * dy;
    let t = if length2 == 0. {
        0.
    } else {
        (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length2).clamp(0., 1.)
    };
    (p[0] - a[0] - t * dx).hypot(p[1] - a[1] - t * dy)
}

fn segments_cross(a: P2, b: P2, c: P2, d: P2) -> bool {
    let (d1, d2, d3, d4) = (
        cross(c, d, a),
        cross(c, d, b),
        cross(a, b, c),
        cross(a, b, d),
    );
    (d1 > 0.) != (d2 > 0.) && (d3 > 0.) != (d4 > 0.) && d1 != 0. && d2 != 0. && d3 != 0. && d4 != 0.
}

fn segment_segment(a: P2, b: P2, c: P2, d: P2) -> f64 {
    if segments_cross(a, b, c, d) {
        return 0.;
    }
    point_segment(a, c, d)
        .min(point_segment(b, c, d))
        .min(point_segment(c, a, b))
        .min(point_segment(d, a, b))
}

impl Piece {
    fn new(boundary: &Boundary) -> Self {
        let n = boundary.vertices.len();
        let mut polygon = vec![];
        let mut sag: f64 = 0.;
        for i in 0..n {
            let (a, c) = (boundary.vertices[i], boundary.vertices[(i + 1) % n]);
            polygon.push([a[0], a[1]]);
            let chord = (c[0] - a[0]).hypot(c[1] - a[1]);
            if a[2] == 0. || chord == 0. {
                continue;
            }
            let sweep = 4. * a[2].atan();
            let radius = chord / (2. * (sweep.abs() / 2.).sin());
            let offset = radius * (sweep / 2.).cos() * sweep.signum();
            let normal = [-(c[1] - a[1]) / chord, (c[0] - a[0]) / chord];
            let center = [
                (a[0] + c[0]) / 2. + normal[0] * offset,
                (a[1] + c[1]) / 2. + normal[1] * offset,
            ];
            let start = (a[1] - center[1]).atan2(a[0] - center[0]);
            let steps = (sweep.abs() / (std::f64::consts::PI / 16.)).ceil().max(2.) as usize;
            sag = sag.max(radius * (1. - (sweep.abs() / (2. * steps as f64)).cos()));
            for k in 1..steps {
                let angle = start + sweep * k as f64 / steps as f64;
                polygon.push([
                    center[0] + radius * angle.cos(),
                    center[1] + radius * angle.sin(),
                ]);
            }
        }
        let mut bounds = [
            f64::INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NEG_INFINITY,
        ];
        for p in &polygon {
            bounds = [
                bounds[0].min(p[0]),
                bounds[1].min(p[1]),
                bounds[2].max(p[0]),
                bounds[3].max(p[1]),
            ];
        }
        Self {
            polygon,
            sag,
            bounds,
        }
    }
    fn contains(&self, p: P2) -> bool {
        let mut inside = false;
        let n = self.polygon.len();
        for i in 0..n {
            let (a, b) = (self.polygon[i], self.polygon[(i + 1) % n]);
            if (a[1] > p[1]) != (b[1] > p[1])
                && p[0] < a[0] + (p[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1])
            {
                inside = !inside;
            }
        }
        inside
    }
    /// The distance from a segment to the piece (0 when it touches or
    /// enters it).
    fn distance(&self, a: P2, b: P2) -> f64 {
        if self.contains(a) || self.contains(b) {
            return 0.;
        }
        let n = self.polygon.len();
        (0..n)
            .map(|i| segment_segment(a, b, self.polygon[i], self.polygon[(i + 1) % n]))
            .fold(f64::INFINITY, f64::min)
    }
    fn far_from(&self, a: P2, b: P2, reach: f64) -> bool {
        a[0].min(b[0]) > self.bounds[2] + reach
            || a[0].max(b[0]) < self.bounds[0] - reach
            || a[1].min(b[1]) > self.bounds[3] + reach
            || a[1].max(b[1]) < self.bounds[1] - reach
    }
}

impl Freed {
    /// A piece cut free; returns its index.
    pub fn add(&mut self, boundary: &Boundary) -> usize {
        self.pieces.push(Piece::new(boundary));
        self.pieces.len() - 1
    }
    pub fn len(&self) -> usize {
        self.pieces.len()
    }
    /// The first freed piece (of the first `count`) a rapid from a to b
    /// comes too near, with the distance. `leaving` is the piece just cut,
    /// for a leg that starts where its cut ended.
    pub fn blocked(
        &self,
        count: usize,
        a: P2,
        b: P2,
        clearance: f64,
        leaving: Option<usize>,
    ) -> Option<(usize, f64)> {
        for (i, piece) in self.pieces.iter().take(count).enumerate() {
            let need = clearance + piece.sag;
            if piece.far_from(a, b, need) {
                continue;
            }
            if leaving == Some(i) {
                // Standing over the slug just cut: the way off it is across
                // it. Beside the part just cut: nearer than the clearance,
                // never across it.
                if piece.contains(a) {
                    continue;
                }
                let n = piece.polygon.len();
                let crosses = piece.contains(b)
                    || (0..n).any(|k| {
                        segments_cross(a, b, piece.polygon[k], piece.polygon[(k + 1) % n])
                    });
                if crosses {
                    return Some((i, 0.));
                }
                continue;
            }
            let d = piece.distance(a, b);
            if d < need {
                return Some((i, (d - piece.sag).max(0.)));
            }
        }
        None
    }
    /// Points to travel through from `from` to `to` so that every leg keeps
    /// the clearance (empty: the straight rapid does). Shortest path over
    /// the corners of the boxes round the pieces in the way, each box grown
    /// beyond the clearance; pieces found in the way of a candidate path
    /// join the search.
    pub fn route(
        &self,
        from: P2,
        to: P2,
        clearance: f64,
        leaving: Option<usize>,
    ) -> Result<Vec<P2>, String> {
        let count = self.pieces.len();
        let Some((first, _)) = self.blocked(count, from, to, clearance, leaving) else {
            return Ok(vec![]);
        };
        let refuse = |why: &str| {
            format!(
                "the rapid from ({:.3}, {:.3}) to ({:.3}, {:.3}) cannot keep {clearance} mm from the parts already cut free ({why}). Use a smaller avoid_cut_parts clearance, more space between parts, or another part_order",
                from[0], from[1], to[0], to[1]
            )
        };
        let mut around = vec![first];
        for _ in 0..64 {
            let mut nodes = vec![from, to];
            for &i in &around {
                let piece = &self.pieces[i];
                let grow = clearance + piece.sag + (0.05 * clearance).max(0.01);
                let b = piece.bounds;
                nodes.extend([
                    [b[0] - grow, b[1] - grow],
                    [b[2] + grow, b[1] - grow],
                    [b[2] + grow, b[3] + grow],
                    [b[0] - grow, b[3] + grow],
                ]);
            }
            // Dijkstra from node 0 to node 1; legs are tested as they are
            // relaxed, against every freed piece.
            let n = nodes.len();
            let mut best = vec![f64::INFINITY; n];
            let mut previous = vec![usize::MAX; n];
            let mut done = vec![false; n];
            let mut found_blockers = vec![];
            best[0] = 0.;
            while let Some(u) = (0..n)
                .filter(|&k| !done[k] && best[k].is_finite())
                .min_by(|&a, &b| best[a].total_cmp(&best[b]))
            {
                if u == 1 {
                    break;
                }
                done[u] = true;
                for v in 1..n {
                    if done[v] {
                        continue;
                    }
                    let length = (nodes[u][0] - nodes[v][0]).hypot(nodes[u][1] - nodes[v][1]);
                    if best[u] + length >= best[v] {
                        continue;
                    }
                    let exempt = if u == 0 { leaving } else { None };
                    match self.blocked(count, nodes[u], nodes[v], clearance, exempt) {
                        None => {
                            best[v] = best[u] + length;
                            previous[v] = u;
                        }
                        Some((i, _)) => found_blockers.push(i),
                    }
                }
            }
            if best[1].is_finite() {
                let mut path = vec![];
                let mut k = previous[1];
                while k != 0 {
                    path.push(nodes[k]);
                    k = previous[k];
                }
                path.reverse();
                return Ok(path);
            }
            let before = around.len();
            for i in found_blockers {
                if !around.contains(&i) {
                    around.push(i);
                }
            }
            if around.len() == before {
                return Err(refuse("no way round them was found"));
            }
        }
        Err(refuse("too many parts stand in the way"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square(x: f64, y: f64, side: f64) -> Boundary {
        Boundary {
            vertices: vec![
                [x, y, 0.],
                [x + side, y, 0.],
                [x + side, y + side, 0.],
                [x, y + side, 0.],
            ],
        }
    }

    #[test]
    fn a_rapid_goes_round_a_freed_piece_at_the_clearance_or_is_refused() {
        let mut freed = Freed::default();
        freed.add(&square(40., 0., 20.));
        // Clear of it: straight.
        assert_eq!(
            freed.route([0., 40.], [100., 40.], 5., None).unwrap(),
            Vec::<P2>::new()
        );
        // Across it: round a corner, every leg at least the clearance away.
        let path = freed.route([0., 10.], [100., 10.], 5., None).unwrap();
        assert!(!path.is_empty());
        let mut points = vec![[0., 10.]];
        points.extend(&path);
        points.push([100., 10.]);
        for leg in points.windows(2) {
            assert!(
                freed.blocked(1, leg[0], leg[1], 5., None).is_none(),
                "{leg:?}"
            );
        }
        let length: f64 = points
            .windows(2)
            .map(|w| (w[0][0] - w[1][0]).hypot(w[0][1] - w[1][1]))
            .sum();
        assert!(length > 100. && length < 140., "{length}");
        // Leaving a slug the torch stands over is allowed; crossing the part
        // just cut round is not.
        assert!(
            freed
                .blocked(1, [50., 10.], [100., 10.], 5., Some(0))
                .is_none()
        );
        assert!(
            freed
                .blocked(1, [38., 10.], [100., 10.], 5., Some(0))
                .is_some()
        );
        assert!(
            freed
                .blocked(1, [38., 10.], [38., 60.], 5., Some(0))
                .is_none()
        );
        // Walled in: refused, naming the rapid.
        let mut walled = Freed::default();
        for (x, y) in [(-30., -10.), (10., -10.), (-10., -30.), (-10., 10.)] {
            walled.add(&square(x, y, 20.));
        }
        let error = walled.route([0., 0.], [100., 100.], 12., None).unwrap_err();
        assert!(
            error.contains("from (0.000, 0.000) to (100.000, 100.000)")
                && error.contains("part_order"),
            "{error}"
        );
    }

    #[test]
    fn an_arc_piece_keeps_the_clearance_from_the_true_circle() {
        let mut freed = Freed::default();
        freed.add(&Boundary {
            vertices: vec![[30., 0., 1.], [-30., 0., 1.]],
        });
        // 35.1 mm from the centre is 5.1 from the circle: clear at 5. 34.9 is not.
        assert!(
            freed
                .blocked(
                    1,
                    [-100., 35.1 + freed.pieces[0].sag],
                    [100., 35.1 + freed.pieces[0].sag],
                    5.,
                    None
                )
                .is_none()
        );
        assert!(
            freed
                .blocked(1, [-100., 34.9], [100., 34.9], 5., None)
                .is_some()
        );
    }
}
