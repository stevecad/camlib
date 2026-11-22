//! `compensation="control"`: the controller applies the whole kerf.
//!
//! The program carries the part side of the beam path (the beam path offset
//! back by half the declared kerf) and G41/G42 with that half kerf; the
//! controller offsets the written path onto the beam path (WinCNC Users
//! Guide, Controller 3.1, revision 3.1.04, pp. 138-139, "G41 Cutter
//! Compensation Left - G41 O#"; ShopSabre's own plasma post, Autodesk post
//! library "shopsabre plasma", writes `G41 O<kerf/2>`).
//!
//! What the controller does at a corner is planned for, not assumed away.
//! Where the beam path turns toward the part it is an arc of the kerf
//! offset's radius about a drawn outside corner, and its part side is an arc
//! of a few micrometres that no program can write: the written path has the
//! corner there. A controller joins its offset elements outside such a
//! corner either at their intersection (WinCNC: "calculate a new
//! intersection point", Versions Information List 3.1.01a) or with an arc
//! about the written corner. So the beam path planned here takes the
//! intersection join: the two edges' kerf paths extended to where they meet,
//! r / sin(theta / 2) from the written corner (theta the corner's angle);
//! and the arc join, which lies inside it, is returned too so that both are
//! proven clear of every part.
//!
//! Refused: a loop the controller cannot form from its part side, where the
//! kerf path of a feature narrower than the kerf bridges it (two outside
//! corners meet, or a corner meets a sharp turn), where the beam path turns
//! sharply toward the part, where the part side of an element is trimmed
//! away, and where the beam path crosses itself.
use crate::{Point, pocket::Stroke};
use std::f64::consts::{PI, TAU};

/// A corner of the beam path the controller joins: the intersection join
/// (the two edges extended to `tip`, `extensions` the added pieces) and the
/// arc join it could take instead.
pub(super) struct Join {
    pub(super) tip: Point,
    pub(super) reach_mm: f64,
    pub(super) extensions: Vec<Stroke>,
    pub(super) arc_join: Stroke,
}

fn sub(a: Point, b: Point) -> Point {
    [a[0] - b[0], a[1] - b[1]]
}
fn add(a: Point, v: Point, k: f64) -> Point {
    [a[0] + k * v[0], a[1] + k * v[1]]
}
fn dot(a: Point, b: Point) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
fn cross(a: Point, b: Point) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}
fn len(a: Point) -> f64 {
    a[0].hypot(a[1])
}
fn left(v: Point) -> Point {
    [-v[1], v[0]]
}
fn angle_of(c: Point, p: Point) -> f64 {
    (p[1] - c[1]).atan2(p[0] - c[0])
}
fn wrap(a: f64) -> f64 {
    let r = a.rem_euclid(TAU);
    if r > PI { r - TAU } else { r }
}
fn radius(s: &Stroke) -> Option<f64> {
    s.arc.map(|(c, _)| len(sub(s.a, c)))
}

/// A stroke's carrier: its line, or its circle.
#[derive(Clone, Copy)]
enum Carrier {
    Line(Point, Point),
    Circle(Point, f64),
}
fn carrier(s: &Stroke) -> Carrier {
    match s.arc {
        None => {
            let d = sub(s.b, s.a);
            let l = len(d);
            Carrier::Line(s.a, [d[0] / l, d[1] / l])
        }
        Some((c, _)) => Carrier::Circle(c, len(sub(s.a, c))),
    }
}
/// The carrier moved by `k` along the stroke's left normal.
fn moved(s: &Stroke, k: f64) -> Option<Carrier> {
    match s.arc {
        None => {
            let t = s.tangent(false);
            Some(Carrier::Line(add(s.a, left(t), k), t))
        }
        Some((c, w)) => {
            // The left normal points to the center on a counter-clockwise
            // arc.
            let r = len(sub(s.a, c)) - w.signum() * k;
            (r > 1e-9).then_some(Carrier::Circle(c, r))
        }
    }
}
fn meet(a: Carrier, b: Carrier) -> Vec<Point> {
    match (a, b) {
        (Carrier::Line(p, d), Carrier::Line(q, e)) => {
            let den = cross(d, e);
            if den.abs() < 1e-14 {
                return vec![];
            }
            vec![add(p, d, cross(sub(q, p), e) / den)]
        }
        (Carrier::Line(p, d), Carrier::Circle(c, r))
        | (Carrier::Circle(c, r), Carrier::Line(p, d)) => {
            let foot = add(p, d, dot(sub(c, p), d));
            let h = len(sub(foot, c));
            if h > r {
                return vec![];
            }
            let w = (r * r - h * h).sqrt();
            vec![add(foot, d, -w), add(foot, d, w)]
        }
        (Carrier::Circle(c1, r1), Carrier::Circle(c2, r2)) => {
            let d = len(sub(c2, c1));
            if d < 1e-12 || d > r1 + r2 || d < (r1 - r2).abs() {
                return vec![];
            }
            let a = (r1 * r1 - r2 * r2 + d * d) / (2. * d);
            let h = (r1 * r1 - a * a).max(0.).sqrt();
            let u = [(c2[0] - c1[0]) / d, (c2[1] - c1[1]) / d];
            let m = add(c1, u, a);
            vec![add(m, left(u), h), add(m, left(u), -h)]
        }
    }
}
fn nearest(points: Vec<Point>, to: Point) -> Option<Point> {
    points
        .into_iter()
        .min_by(|p, q| len(sub(*p, to)).total_cmp(&len(sub(*q, to))))
}
/// The stroke with its end moved to `b` along its carrier (`end`) or its
/// start moved to `a`; None when that leaves no length in its direction.
fn moved_end(s: &Stroke, p: Point, end: bool) -> Option<Stroke> {
    let (a, b) = if end { (s.a, p) } else { (p, s.b) };
    match s.arc {
        None => (dot(sub(b, a), sub(s.b, s.a)) > 0.).then_some(Stroke { a, b, arc: None }),
        Some((c, w)) => {
            let sweep = w + if end {
                wrap(angle_of(c, b) - angle_of(c, s.b))
            } else {
                -wrap(angle_of(c, a) - angle_of(c, s.a))
            };
            (sweep * w > 0. && sweep.abs() < TAU).then_some(Stroke {
                a,
                b,
                arc: Some((c, sweep)),
            })
        }
    }
}
fn turn(a: &Stroke, b: &Stroke) -> f64 {
    let (i, o) = (a.tangent(true), b.tangent(false));
    cross(i, o).atan2(dot(i, o))
}

/// The beam path of one closed loop for the controller to cut: each round
/// outside corner (an arc of radius at most `offset` turning toward the
/// part) replaced by its edges extended to their intersection. `scrap` is
/// +1 with the scrap on the left of travel, -1 on the right; `half` is half
/// the declared kerf (the controller's amount). Returns the loop and its
/// joins.
pub(super) fn beam_path(
    strokes: &[Stroke],
    scrap: f64,
    half: f64,
    offset: f64,
    guard: f64,
    tolerance: f64,
) -> Result<(Vec<Stroke>, Vec<Join>), String> {
    let n = strokes.len();
    let at = |p: Point| format!("({:.3}, {:.3})", p[0], p[1]);
    let corner = |s: &Stroke| {
        s.arc.is_some_and(|(_, w)| w * scrap < 0.) && radius(s).is_some_and(|r| r <= offset + guard)
    };
    // Every junction: tangent, or a sharp turn away from the part.
    let tangent_turn = 1e-6;
    for k in 0..n {
        let (a, b) = (&strokes[k], &strokes[(k + 1) % n]);
        let t = turn(a, b);
        if t * scrap < -tangent_turn {
            return Err(format!(
                "the kerf path turns sharply toward the part at {}: the controller cannot form its offset there; use compensation=\"computer\"",
                at(a.b)
            ));
        }
        if t.abs() > tangent_turn && (corner(a) || corner(b)) {
            return Err(format!(
                "the kerf path bridges a gap narrower than the kerf at {} (an outside corner's arc meets a sharp turn): the controller cannot form its offset from the part side; widen the gap or use compensation=\"computer\"",
                at(a.b)
            ));
        }
        if corner(a) && corner(b) {
            return Err(format!(
                "the kerf path bridges a gap narrower than the kerf at {} (two outside corners' arcs meet): the controller cannot form its offset from the part side; widen the gap or use compensation=\"computer\"",
                at(a.b)
            ));
        }
    }
    if strokes.iter().all(corner) {
        return Err("a part smaller than the kerf: the controller cannot form its offset".into());
    }
    let mut out: Vec<Stroke> = strokes.to_vec();
    let mut joins = vec![];
    // Each corner arc, from the last, so indices below stay valid.
    let mut k = n;
    while k > 0 {
        k -= 1;
        if !corner(&out[k]) {
            continue;
        }
        let m = out.len();
        let (pi, ni) = ((k + m - 1) % m, (k + 1) % m);
        let (prev, arc, next) = (out[pi], out[k], out[ni]);
        let (c, w) = arc.arc.unwrap();
        let mid = add(
            c,
            [
                (angle_of(c, arc.a) + w / 2.).cos(),
                (angle_of(c, arc.a) + w / 2.).sin(),
            ],
            radius(&arc).unwrap(),
        );
        let tip = nearest(meet(carrier(&prev), carrier(&next)), mid).ok_or_else(|| {
            format!(
                "the edges either side of the corner at {} do not meet when extended: the controller cannot join its offset there",
                at(mid)
            )
        })?;
        let (Some(p2), Some(n2)) = (moved_end(&prev, tip, true), moved_end(&next, tip, false))
        else {
            return Err(format!(
                "the edges either side of the corner at {} meet behind it: the controller cannot join its offset there",
                at(mid)
            ));
        };
        // The written corner: where the two edges' part sides meet.
        let (Some(pw), Some(nw)) = (moved(&prev, -scrap * half), moved(&next, -scrap * half))
        else {
            return Err(format!(
                "the corner at {} is tighter than the kerf",
                at(mid)
            ));
        };
        let written = nearest(meet(pw, nw), tip)
            .ok_or_else(|| format!("the part side of the corner at {} does not meet", at(mid)))?;
        let (ta, tb) = (prev.tangent(true), next.tangent(false));
        let (na, nb) = (
            [scrap * -ta[1], scrap * ta[0]],
            [scrap * -tb[1], scrap * tb[0]],
        );
        // The same turn as the corner's arc, about the written corner.
        let arc_join = Stroke {
            a: add(written, na, half),
            b: add(written, nb, half),
            arc: Some((written, w)),
        };
        let extensions = vec![
            Stroke {
                a: prev.b,
                ..moved_end(&prev, tip, true).unwrap()
            },
            Stroke {
                b: next.a,
                ..moved_end(&next, tip, false).unwrap()
            },
        ]
        .into_iter()
        .map(|s| match s.arc {
            None => Stroke::line(s.a, s.b),
            Some((c, _)) => {
                let sweep = wrap(angle_of(c, s.b) - angle_of(c, s.a));
                Stroke {
                    arc: Some((c, sweep)),
                    ..s
                }
            }
        })
        .filter(|s| crate::profiles::stroke_length(s) > guard)
        .collect();
        joins.push(Join {
            tip,
            reach_mm: len(sub(tip, written)),
            extensions,
            arc_join,
        });
        out[pi] = p2;
        out[ni] = n2;
        out.remove(k);
    }
    joins.reverse();
    // A loop that started at an outside corner now starts at the tip of
    // its join; it starts in the middle of the edge after it instead, so
    // that the leads meet a straight edge whose part side is the drawn one
    // (and no sliver of a move is written between them and the corner).
    if corner(&strokes[0]) || corner(&strokes[n - 1]) {
        let s = out[0];
        let mid = crate::profiles::stroke_point(&s, 0.5);
        let half = |w: f64| w / 2.;
        let (first, second) = (
            Stroke {
                a: s.a,
                b: mid,
                arc: s.arc.map(|(c, w)| (c, half(w))),
            },
            Stroke {
                a: mid,
                b: s.b,
                arc: s.arc.map(|(c, w)| (c, half(w))),
            },
        );
        out[0] = second;
        out.push(first);
    }
    // The part side of every element must keep its direction: trimmed at
    // the turns toward the part (the corners just joined), extended at the
    // turns away from it.
    let m = out.len();
    let written_corner = |a: &Stroke, b: &Stroke| -> Result<Point, String> {
        if turn(a, b).abs() <= tangent_turn {
            return Ok(add(
                a.b,
                [scrap * a.tangent(true)[1], -scrap * a.tangent(true)[0]],
                half,
            ));
        }
        let (Some(pa), Some(pb)) = (moved(a, -scrap * half), moved(b, -scrap * half)) else {
            return Err(format!(
                "an arc at {} is tighter than half the kerf",
                at(a.b)
            ));
        };
        nearest(meet(pa, pb), a.b).ok_or_else(|| {
            format!(
                "the part side does not meet at {}: the controller cannot form its offset there",
                at(a.b)
            )
        })
    };
    let corners = (0..m)
        .map(|k| written_corner(&out[k], &out[(k + 1) % m]))
        .collect::<Result<Vec<_>, _>>()?;
    let mut written = Vec::with_capacity(m);
    for k in 0..m {
        let s = &out[k];
        let (a, b) = (corners[(k + m - 1) % m], corners[k]);
        let arc = s.arc.map(|(c, w)| {
            let sweep = w + wrap(angle_of(c, b) - angle_of(c, s.b))
                - wrap(angle_of(c, a) - angle_of(c, s.a));
            (c, sweep)
        });
        let keeps = match (s.arc, arc) {
            (Some((_, w)), Some((_, sweep))) => sweep * w > 0. && sweep.abs() < TAU,
            _ => dot(sub(b, a), sub(s.b, s.a)) > 0.,
        };
        if !keeps {
            return Err(format!(
                "the part side of the kerf path's element from {} to {} is trimmed away by its corners (a feature narrower than the kerf): the controller cannot form its offset; use compensation=\"computer\"",
                at(s.a),
                at(s.b)
            ));
        }
        written.push(Stroke { a, b, arc });
    }
    validate_loop(&written, tolerance, "written part-side loop")?;
    validate_loop(&out, tolerance, "beam loop")?;
    // Arcs of at most a quarter turn: the controller's offset of a written
    // arc ends where the offsets of its neighbours meet it, so its sweep
    // moves by a hair of the written words' rounding; a half turn would
    // then be more than a controller that takes half a turn per block
    // (WinCNC) is written or decoded with.
    let mut split = Vec::with_capacity(out.len());
    for s in out {
        match s.arc {
            Some((c, w)) if w.abs() > PI / 2. + 1e-12 => {
                let pieces = (w.abs() / (PI / 2.)).ceil() as usize;
                let r = radius(&s).unwrap();
                let a0 = angle_of(c, s.a);
                let mut a = s.a;
                for k in 1..=pieces {
                    let b = if k == pieces {
                        s.b
                    } else {
                        let t = a0 + w * k as f64 / pieces as f64;
                        [c[0] + r * t.cos(), c[1] + r * t.sin()]
                    };
                    split.push(Stroke {
                        a,
                        b,
                        arc: Some((c, w / pieces as f64)),
                    });
                    a = b;
                }
            }
            _ => split.push(s),
        }
    }
    Ok((split, joins))
}

/// Reuse the same simple-boundary checks as an input region: exact
/// polygon predicates and continuous line/arc intersections, including
/// overlapping carriers and a second contact between consecutive edges.
fn validate_loop(strokes: &[Stroke], tolerance: f64, name: &str) -> Result<(), String> {
    use crate::planar::{Boundary, Component, ConstructionOptions, PlanarRegion};
    let mut vertices = Vec::with_capacity(strokes.len());
    for s in strokes {
        let sweep = s.arc.map_or(0., |(_, w)| w);
        // Boundary represents circular arcs of at most 180 degrees.
        let count = (sweep.abs() / PI).ceil().max(1.) as usize;
        let bulge = (sweep / (4. * count as f64)).tan();
        for k in 0..count {
            let p = if k == 0 {
                s.a
            } else {
                crate::profiles::stroke_point(s, k as f64 / count as f64)
            };
            vertices.push([p[0], p[1], bulge]);
        }
    }
    PlanarRegion {
        version: 1, units: "mm".into(), name: name.into(), boundary_error_mm: 0.,
        components: vec![Component { outer: Boundary { vertices }, holes: vec![] }],
        source: serde_json::Value::Null,
    }
    .prepare(ConstructionOptions { tolerance_mm: tolerance })
    .map_err(|reason| {
        format!("the controller's {name} cannot form a simple boundary: {reason}; use compensation=\"computer\"")
    })?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn consecutive_edges_may_only_meet_at_their_join() {
        // A semicircle from (0,0) to (2,0) through (1,-1). Its
        // next edge crosses it a second time at (0.2,-0.6).
        let arc = Stroke {
            a: [0., 0.],
            b: [2., 0.],
            arc: Some(([1., 0.], PI)),
        };
        let loop_edges = |corner| {
            [
                arc,
                Stroke::line(arc.b, corner),
                Stroke::line(corner, arc.a),
            ]
        };
        assert!(validate_loop(&loop_edges([-1., -1.]), 1e-5, "beam loop").is_err());
        assert!(validate_loop(&loop_edges([1., 1.]), 1e-5, "beam loop").is_ok());
    }
}
