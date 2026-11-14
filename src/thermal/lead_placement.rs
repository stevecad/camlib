//! Where a loop may start, and so where its leads go.
//!
//! A lead leaves a mark where it meets the cut: a small divot or a nub on
//! the part's edge. On an edge that is seen, sealed or welded that matters,
//! so nesting software lets the programmer keep leads off chosen edges and
//! put them at a corner or on a chosen side instead (the "lead position"
//! and "avoid edge" settings of commercial nesting packages). Here the
//! caller gives the stretches of outline to keep off as paths, a distance to
//! keep from them, and a preference; the planner starts a loop at the first
//! candidate, in the preferred order, that is off those stretches and whose
//! leads and pierce are proven clear of the parts. A loop with no start off
//! them is refused by name.
use crate::{
    pocket::Stroke,
    profiles::{stroke_length, stroke_point},
    trace::TracePath,
};
use serde::{Deserialize, Serialize};

type P2 = [f64; 2];

/// Which start is taken first among those that are allowed and clear.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum LeadPreference {
    /// The loop's own start as drawn, then the middle of its longest
    /// straight edges (what the cut does without this option).
    #[default]
    AsDrawn,
    /// A corner of the loop, the sharpest first.
    Corner,
    /// The start lying furthest in this direction (the side of the part
    /// facing it).
    Toward { direction: P2 },
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LeadPlacement {
    /// Stretches of the parts' outlines that must not carry a lead, in the
    /// sheet's coordinates (lines and arcs along the drawn edges).
    #[serde(default)]
    pub keep_off: Vec<TracePath>,
    /// How far along the outline a loop's start stays from those stretches,
    /// measured from the drawn edge. No default.
    pub keep_off_distance_mm: f64,
    #[serde(default)]
    pub prefer: LeadPreference,
}

/// The distance from `p` to a stroke.
fn stroke_distance(s: &Stroke, p: P2) -> f64 {
    match s.arc {
        None => {
            let (d, v) = (
                [s.b[0] - s.a[0], s.b[1] - s.a[1]],
                [p[0] - s.a[0], p[1] - s.a[1]],
            );
            let length2 = d[0] * d[0] + d[1] * d[1];
            let t = if length2 > 0. {
                ((v[0] * d[0] + v[1] * d[1]) / length2).clamp(0., 1.)
            } else {
                0.
            };
            (v[0] - t * d[0]).hypot(v[1] - t * d[1])
        }
        Some((c, sweep)) => {
            let radius = (s.a[0] - c[0]).hypot(s.a[1] - c[1]);
            let start = (s.a[1] - c[1]).atan2(s.a[0] - c[0]);
            let at = (p[1] - c[1]).atan2(p[0] - c[0]);
            // The angle from the arc's start to p, along the arc's direction.
            let along = if sweep >= 0. {
                (at - start).rem_euclid(std::f64::consts::TAU)
            } else {
                (start - at).rem_euclid(std::f64::consts::TAU)
            };
            if along <= sweep.abs() {
                ((p[0] - c[0]).hypot(p[1] - c[1]) - radius).abs()
            } else {
                let to = |q: P2| (p[0] - q[0]).hypot(p[1] - q[1]);
                to(s.a).min(to(s.b))
            }
        }
    }
}

pub(crate) struct Prepared {
    keep_off: Vec<Stroke>,
    /// A start nearer than this to a kept-off stretch is on it: the declared
    /// distance and the kerf path's offset from the drawn edge.
    reach: f64,
    /// The kerf path's offset from the drawn edge: a drawn outside corner is
    /// an arc of this radius on the kerf path.
    offset: f64,
    prefer: LeadPreference,
}

impl Prepared {
    pub(crate) fn new(placement: &LeadPlacement, offset: f64, guard: f64) -> Result<Self, String> {
        if !(placement.keep_off_distance_mm.is_finite()
            && (0. ..=1e6).contains(&placement.keep_off_distance_mm))
        {
            return Err(format!(
                "lead_placement: keep_off_distance_mm {} must be zero or more (how far a loop's start stays from the edges kept free of leads)",
                placement.keep_off_distance_mm
            ));
        }
        if let LeadPreference::Toward { direction } = placement.prefer {
            let length = direction[0].hypot(direction[1]);
            if !(length.is_finite() && length > 0.) {
                return Err(
                    "lead_placement: the preferred direction must be a non-zero vector".into(),
                );
            }
        }
        let mut keep_off = vec![];
        for (i, path) in placement.keep_off.iter().enumerate() {
            let (strokes, _) = crate::trace::strokes(path, guard)
                .map_err(|e| format!("lead_placement: keep_off path {i}: {e}"))?;
            keep_off.extend(strokes);
        }
        Ok(Self {
            keep_off,
            reach: placement.keep_off_distance_mm + offset.abs(),
            offset: offset.abs(),
            prefer: placement.prefer,
        })
    }

    /// Whether a loop starting at `p` (a point of the kerf path) would put
    /// its leads on a kept-off stretch.
    pub(crate) fn kept_off(&self, p: P2) -> bool {
        self.keep_off
            .iter()
            .any(|s| stroke_distance(s, p) <= self.reach)
    }

    /// The loop's possible starts in the preferred order: its stroke ends
    /// (the loop's own start first) and the middles of its straight
    /// strokes. `tolerance` decides what is a corner: a join whose tangents
    /// differ by more than it.
    pub(crate) fn candidates(&self, strokes: &[Stroke], tolerance: f64) -> Vec<Start> {
        let n = strokes.len();
        // How far the direction turns where stroke k begins (0: tangent).
        let turn = |k: usize| {
            let (i, o) = (
                strokes[(k + n - 1) % n].tangent(true),
                strokes[k].tangent(false),
            );
            (i[0] * o[1] - i[1] * o[0])
                .atan2(i[0] * o[0] + i[1] * o[1])
                .abs()
        };
        let mut middles: Vec<usize> = (0..n).filter(|&k| strokes[k].arc.is_none()).collect();
        middles.sort_by(|&i, &j| {
            stroke_length(&strokes[j])
                .total_cmp(&stroke_length(&strokes[i]))
                .then(i.cmp(&j))
        });
        let middles = middles.into_iter().map(Start::Middle);
        match self.prefer {
            LeadPreference::AsDrawn => std::iter::once(Start::End(0))
                .chain(middles)
                .chain((1..n).map(Start::End))
                .collect(),
            LeadPreference::Corner => {
                // A corner of the kerf path: a join where the direction
                // turns, or the arc the offset puts round a drawn outside
                // corner (its middle, where a lead meets it on the corner's
                // bisector).
                let turns: Vec<f64> = (0..n).map(turn).collect();
                let round = |k: usize| {
                    strokes[k].arc.is_some_and(|(c, _)| {
                        (strokes[k].a[0] - c[0]).hypot(strokes[k].a[1] - c[1])
                            <= self.offset + tolerance
                    })
                };
                let mut corners: Vec<(f64, Start)> = (0..n)
                    .filter(|&k| turns[k] > tolerance)
                    .map(|k| (turns[k], Start::End(k)))
                    .chain((0..n).filter(|&k| round(k)).map(|k| {
                        (
                            strokes[k].arc.map_or(0., |(_, w)| w.abs()),
                            Start::Middle(k),
                        )
                    }))
                    .collect();
                // Stable: equal corners keep the loop's order.
                corners.sort_by(|a, b| b.0.total_cmp(&a.0));
                corners
                    .into_iter()
                    .map(|(_, start)| start)
                    .chain(middles)
                    .chain((0..n).filter(|&k| turns[k] <= tolerance).map(Start::End))
                    .collect()
            }
            LeadPreference::Toward { direction } => {
                let mut all: Vec<Start> = std::iter::once(Start::End(0))
                    .chain(middles)
                    .chain((1..n).map(Start::End))
                    .collect();
                let along = |start: &Start| {
                    let p = start.point(strokes);
                    p[0] * direction[0] + p[1] * direction[1]
                };
                // Stable: equal starts keep the as-drawn order.
                all.sort_by(|a, b| along(b).total_cmp(&along(a)));
                all
            }
        }
    }
}

/// A place a loop can start: where stroke k begins, or its middle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum Start {
    End(usize),
    Middle(usize),
}

impl Start {
    pub(crate) fn point(&self, strokes: &[Stroke]) -> P2 {
        match *self {
            Start::End(k) => strokes[k].a,
            Start::Middle(k) => stroke_point(&strokes[k], 0.5),
        }
    }

    /// The same loop, starting here.
    pub(crate) fn rotate(&self, strokes: &[Stroke]) -> Vec<Stroke> {
        match *self {
            Start::End(k) => {
                let mut r = strokes[k..].to_vec();
                r.extend_from_slice(&strokes[..k]);
                r
            }
            Start::Middle(k) => {
                let s = strokes[k];
                let mid = stroke_point(&s, 0.5);
                let mut r = vec![Stroke {
                    a: mid,
                    b: s.b,
                    arc: s.arc.map(|(c, w)| (c, w / 2.)),
                }];
                r.extend_from_slice(&strokes[k + 1..]);
                r.extend_from_slice(&strokes[..k]);
                r.push(Stroke {
                    a: s.a,
                    b: mid,
                    arc: s.arc.map(|(c, w)| (c, w / 2.)),
                });
                r
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square() -> Vec<Stroke> {
        let p = [[0., 0.], [10., 0.], [10., 6.], [0., 6.]];
        (0..4).map(|i| Stroke::line(p[i], p[(i + 1) % 4])).collect()
    }
    fn prepared(prefer: LeadPreference) -> Prepared {
        Prepared {
            keep_off: vec![Stroke::line([0., 0.], [10., 0.])],
            reach: 1.,
            offset: 0.5,
            prefer,
        }
    }

    #[test]
    fn distances_to_lines_and_arcs() {
        let line = Stroke::line([0., 0.], [10., 0.]);
        assert_eq!(stroke_distance(&line, [5., 3.]), 3.);
        assert_eq!(stroke_distance(&line, [13., 4.]), 5.);
        // A quarter arc of radius 5 about the origin from (5, 0) to (0, 5).
        let arc = Stroke {
            a: [5., 0.],
            b: [0., 5.],
            arc: Some(([0., 0.], std::f64::consts::FRAC_PI_2)),
        };
        assert!((stroke_distance(&arc, [3., 3.]) - (5. - 18f64.sqrt())).abs() < 1e-12);
        // Beyond its end: to the end point, not to the circle.
        assert!((stroke_distance(&arc, [-3., 5.]) - 3.).abs() < 1e-12);
        // The same arc run clockwise from (0, 5).
        let back = Stroke {
            a: [0., 5.],
            b: [5., 0.],
            arc: Some(([0., 0.], -std::f64::consts::FRAC_PI_2)),
        };
        assert!((stroke_distance(&back, [3., 3.]) - (5. - 18f64.sqrt())).abs() < 1e-12);
        assert!((stroke_distance(&back, [5., -2.]) - 2.).abs() < 1e-12);
    }

    #[test]
    fn candidates_follow_the_preference_and_keep_the_loop() {
        let strokes = square();
        let starts = |prefer| -> Vec<P2> {
            let all = prepared(prefer).candidates(&strokes, 1e-6);
            for start in &all {
                // Every candidate is the same closed loop, from its point.
                let r = start.rotate(&strokes);
                let length: f64 = r.iter().map(stroke_length).sum();
                assert!((length - 32.).abs() < 1e-12);
                assert_eq!(r[0].a, r[r.len() - 1].b);
                assert_eq!(r[0].a, start.point(&strokes));
            }
            all.iter().map(|start| start.point(&strokes)).collect()
        };
        // As drawn: the start, the long edges' middles, the short ones',
        // then the other corners.
        assert_eq!(
            starts(LeadPreference::AsDrawn),
            [
                [0., 0.],
                [5., 0.],
                [5., 6.],
                [10., 3.],
                [0., 3.],
                [10., 0.],
                [10., 6.],
                [0., 6.]
            ]
        );
        // Corners first.
        assert_eq!(
            starts(LeadPreference::Corner)[..4],
            [[0., 0.], [10., 0.], [10., 6.], [0., 6.]]
        );
        // Toward +y: the top edge's points first.
        let up = starts(LeadPreference::Toward {
            direction: [0., 1.],
        });
        assert!(up[..3].iter().all(|p| p[1] == 6.));
        assert!(up[5..].iter().all(|p| p[1] == 0.));
        // A square's kerf path, half a millimetre out: its corners are
        // quarter arcs, and a lead at a corner starts in the arc's middle.
        let h = std::f64::consts::FRAC_PI_2;
        let rounded = vec![
            Stroke::line([0., -0.5], [10., -0.5]),
            Stroke {
                a: [10., -0.5],
                b: [10.5, 0.],
                arc: Some(([10., 0.], h)),
            },
            Stroke::line([10.5, 0.], [10.5, 6.]),
            Stroke {
                a: [10.5, 6.],
                b: [10., 6.5],
                arc: Some(([10., 6.], h)),
            },
            Stroke::line([10., 6.5], [0., 6.5]),
            Stroke {
                a: [0., 6.5],
                b: [-0.5, 6.],
                arc: Some(([0., 6.], h)),
            },
            Stroke::line([-0.5, 6.], [-0.5, 0.]),
            Stroke {
                a: [-0.5, 0.],
                b: [0., -0.5],
                arc: Some(([0., 0.], h)),
            },
        ];
        let first = prepared(LeadPreference::Corner).candidates(&rounded, 1e-6);
        assert_eq!(
            first[..4],
            [
                Start::Middle(1),
                Start::Middle(3),
                Start::Middle(5),
                Start::Middle(7)
            ]
        );
        let p = first[0].point(&rounded);
        let d = 0.5 * std::f64::consts::FRAC_1_SQRT_2;
        assert!((p[0] - (10. + d)).abs() < 1e-12 && (p[1] + d).abs() < 1e-12);
        // The bottom edge is kept off, a millimetre round it.
        let p = prepared(LeadPreference::AsDrawn);
        assert!(p.kept_off([5., 0.]) && p.kept_off([10., 1.]) && !p.kept_off([10., 3.]));
    }
}
