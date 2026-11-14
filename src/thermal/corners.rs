//! Corner treatment for a thermal cut: loops and slow-downs.
//!
//! A plasma arc lags behind the torch, more at the bottom of the plate than
//! at the top, and a machine slows down to turn a sharp corner. Both round
//! or undercut a part's outside corners. Nesting software offers two
//! remedies, and both are here as declared options:
//!
//! - A corner loop: the cut runs straight on past the corner into the scrap,
//!   turns round in a loop there and comes back onto the next edge, so both
//!   edges are cut straight through the corner at full speed and the corner
//!   is left sharp.
//! - A corner slow-down: the feed drops over a declared distance either side
//!   of a corner so the arc catches up, with torch height control held off
//!   meanwhile (the arc voltage rises as the torch slows, and a height
//!   control that follows it would dive).
//!
//! The kerf path runs at the kerf's offset from the drawn edge, so a drawn
//! outside corner is an arc of that radius on it and a drawn inside corner is
//! a sharp join.
use crate::{motion::Curve, pocket::Stroke};
use serde::{Deserialize, Serialize};
use std::f64::consts::{FRAC_PI_2, PI, TAU};

type P2 = [f64; 2];

fn yes() -> bool {
    true
}

/// Loops at the parts' outside corners. Every value is the caller's.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CornerLoops {
    /// How far the cut runs on past the corner before it turns, and how far
    /// before the corner it comes back onto the next edge.
    pub overshoot_mm: f64,
    /// Corners whose direction changes by at least this much get a loop.
    pub min_angle_deg: f64,
    /// Corners whose direction changes by more than this get none (the loop
    /// of a very sharp corner starts far from the part).
    pub max_angle_deg: f64,
}

/// A slower feed around corners. Every value is the caller's.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CornerSlowdown {
    /// How far before and after a corner the feed is reduced.
    pub distance_mm: f64,
    /// The feed there, as a percentage of the cut's feed.
    pub feed_percent: f64,
    /// Corners whose direction changes by at least this much are slowed.
    pub min_angle_deg: f64,
    /// Hold torch height control off over the slowed stretch.
    #[serde(default = "yes")]
    pub height_control_off: bool,
}

impl CornerLoops {
    pub(crate) fn validate(&self) -> Result<(), String> {
        if !(self.overshoot_mm.is_finite() && self.overshoot_mm > 0. && self.overshoot_mm <= 1e6) {
            return Err(format!(
                "corner_loops: overshoot_mm {} must be positive (how far the cut runs past a corner)",
                self.overshoot_mm
            ));
        }
        if !(self.min_angle_deg.is_finite()
            && self.max_angle_deg.is_finite()
            && self.min_angle_deg > 0.
            && self.min_angle_deg <= self.max_angle_deg
            && self.max_angle_deg < 180.)
        {
            return Err(format!(
                "corner_loops: min_angle_deg {} and max_angle_deg {} must satisfy 0 < min <= max < 180 (the change of direction at the corners that get a loop)",
                self.min_angle_deg, self.max_angle_deg
            ));
        }
        Ok(())
    }
}

impl CornerSlowdown {
    pub(crate) fn validate(&self) -> Result<(), String> {
        if !(self.distance_mm.is_finite() && self.distance_mm > 0. && self.distance_mm <= 1e6) {
            return Err(format!(
                "corner_slowdown: distance_mm {} must be positive (how far either side of a corner the feed is reduced)",
                self.distance_mm
            ));
        }
        if !(self.feed_percent.is_finite() && self.feed_percent > 0. && self.feed_percent < 100.) {
            return Err(format!(
                "corner_slowdown: feed_percent {} must be above 0 and below 100",
                self.feed_percent
            ));
        }
        if !(self.min_angle_deg.is_finite() && self.min_angle_deg > 0. && self.min_angle_deg < 180.)
        {
            return Err(format!(
                "corner_slowdown: min_angle_deg {} must be above 0 and below 180",
                self.min_angle_deg
            ));
        }
        Ok(())
    }
}

fn cross(a: P2, b: P2) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}
fn dot(a: P2, b: P2) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
fn along(p: P2, v: P2, k: f64) -> P2 {
    [p[0] + k * v[0], p[1] + k * v[1]]
}
fn radius(s: &Stroke) -> Option<f64> {
    s.arc.map(|(c, _)| (s.a[0] - c[0]).hypot(s.a[1] - c[1]))
}

/// The loop's strokes with a loop at every outside corner the option names,
/// a flag per stroke (true: part of a loop, in the scrap), and the number
/// of loops.
///
/// `side` is +1 with the scrap on the left of travel and -1 with it on the
/// right; `offset` is the kerf path's distance from the drawn edge. A
/// corner is an arc of at most that radius (a drawn outside corner) or a
/// sharp join, between two straight strokes, that turns towards the part.
/// The corner where the loop starts and ends is left alone: the leads are
/// there.
///
/// At a corner whose direction changes by `t`, with `v` the point where the
/// two straight edges of the kerf path meet: the cut runs on to `v + d`
/// along the incoming edge, turns away from the part through `2 pi - t` on
/// a circle of radius `d / tan(t / 2)` (tangent to both edges), and meets
/// the outgoing edge at `v - d`. The circle is written as arcs of a quarter
/// turn each from its start, and what is left.
pub(crate) fn with_loops(
    strokes: &[Stroke],
    loops: &CornerLoops,
    offset: f64,
    side: f64,
    guard: f64,
) -> (Vec<Stroke>, Vec<bool>, usize) {
    let n = strokes.len();
    let (min, max) = (
        loops.min_angle_deg.to_radians(),
        loops.max_angle_deg.to_radians(),
    );
    let d = loops.overshoot_mm;
    let straight = |k: usize| strokes[k].arc.is_none();
    let mut out: Vec<Stroke> = Vec::with_capacity(n + 8);
    let mut flags: Vec<bool> = Vec::with_capacity(n + 8);
    let mut count = 0;
    // The loop for a corner at `v`, entered along `t_in` and left along
    // `t_out`: the run-on, the arcs, the run-back.
    let mut make = |from: P2, v: P2, t_in: P2, t_out: P2, to: P2| -> Option<Vec<Stroke>> {
        let turn = cross(t_in, t_out).atan2(dot(t_in, t_out));
        // Towards the part: away from the scrap side.
        if turn * side >= 0. || turn.abs() < min - 1e-12 || turn.abs() > max + 1e-12 {
            return None;
        }
        let theta = turn.abs();
        let r = d / (theta / 2.).tan();
        let away = -turn.signum();
        let p1 = along(v, t_in, d);
        let p2 = along(v, t_out, -d);
        let center = along(p1, [-t_in[1], t_in[0]], away * r);
        let sweep = away * (TAU - theta);
        let u = [p1[0] - center[0], p1[1] - center[1]];
        let at = |angle: f64| {
            let (ca, sa) = (angle.cos(), angle.sin());
            [
                center[0] + ca * u[0] - sa * u[1],
                center[1] + sa * u[0] + ca * u[1],
            ]
        };
        count += 1;
        let mut parts = vec![Stroke::line(from, p1)];
        // Quarter turns from the loop's start, then what is left: a square
        // corner's loop is three quarter arcs between points a controller's
        // rounded words can state exactly.
        let quarters = (sweep.abs() / FRAC_PI_2 - 1e-9).ceil().max(1.) as usize;
        let mut a = p1;
        for q in 1..=quarters {
            let (angle, b) = if q == quarters {
                (sweep, p2)
            } else {
                let angle = away * q as f64 * FRAC_PI_2;
                (angle, at(angle))
            };
            let piece = angle - away * (q - 1) as f64 * FRAC_PI_2;
            parts.push(Stroke {
                a,
                b,
                arc: Some((center, piece)),
            });
            a = b;
        }
        parts.push(Stroke::line(p2, to));
        Some(parts)
    };
    let mut k = 0;
    while k < n {
        let s = strokes[k];
        // A sharp join before stroke k, between two straight strokes.
        if k > 0 && straight(k - 1) && straight(k) {
            let (t_in, t_out) = (strokes[k - 1].tangent(true), s.tangent(false));
            if let Some(parts) = make(s.a, s.a, t_in, t_out, s.a) {
                flags.extend(std::iter::repeat_n(true, parts.len()));
                out.extend(parts);
            }
        }
        // The arc the offset puts round a drawn outside corner.
        let round = k > 0
            && k + 1 < n
            && straight(k - 1)
            && straight(k + 1)
            && radius(&s).is_some_and(|r| r <= offset + guard)
            && s.arc.is_some_and(|(_, w)| w.abs() < PI - 1e-9);
        if round {
            let (t_in, t_out) = (s.tangent(false), s.tangent(true));
            let theta = s.arc.map_or(0., |(_, w)| w.abs());
            let v = along(s.a, t_in, radius(&s).unwrap_or(0.) * (theta / 2.).tan());
            if let Some(parts) = make(s.a, v, t_in, t_out, s.b) {
                flags.extend(std::iter::repeat_n(true, parts.len()));
                out.extend(parts);
                k += 1;
                continue;
            }
        }
        out.push(s);
        flags.push(false);
        k += 1;
    }
    (out, flags, count)
}

/// How far a corner loop's kerf reaches beyond the edge of the kerf of the
/// cut it belongs to, at most: the nest keeps this much room. The loop's
/// far side is `d / tan(t / 4)` from the point where the kerf path's edges
/// meet, the most at the smallest angle; that point is `offset / cos(t / 2)`
/// from the drawn corner, the most at the largest.
pub fn loop_reach_mm(loops: &CornerLoops, offset: f64) -> f64 {
    let (min, max) = (
        loops.min_angle_deg.to_radians(),
        loops.max_angle_deg.to_radians(),
    );
    loops.overshoot_mm / (min / 4.).tan() + offset * (1. / (max / 2.).cos() - 1.)
}

/// A piece of a cut move between two changes of feed.
pub(crate) struct Piece {
    /// The move it is a piece of (its index among those given).
    pub source: usize,
    pub curve: Curve,
    /// Cut at the corner feed.
    pub slowed: bool,
}

/// A loop's cut moves, in order, split where the feed changes, and the
/// number of slowed stretches.
///
/// `strokes` is the closed loop the moves follow from its start (`in_loop`:
/// its corner-loop strokes, which are no corners); the moves may stop short
/// of closing it or run on past its start (the overcut). A corner is a join
/// where the direction changes by at least the declared angle, or an arc of
/// at most the kerf path's `offset` in radius that turns by at least it (the
/// arc the offset puts round a drawn outside corner). The feed is reduced
/// from `distance_mm` before a corner to `distance_mm` after it, along the
/// loop and across its start; stretches that meet are one.
pub(crate) fn slow_down(
    cuts: &[Curve],
    strokes: &[Stroke],
    in_loop: &[bool],
    slow: &CornerSlowdown,
    offset: f64,
    guard: f64,
    tolerance: f64,
) -> Result<(Vec<Piece>, usize), String> {
    let min = slow.min_angle_deg.to_radians() - 1e-12;
    let n = strokes.len();
    let mut begins = Vec::with_capacity(n);
    let mut period = 0.;
    for s in strokes {
        begins.push(period);
        period += crate::profiles::stroke_length(s);
    }
    // Corners as stretches of the loop, from its start.
    let mut corners: Vec<(f64, f64)> = vec![];
    for (k, s) in strokes.iter().enumerate() {
        let before = (k + n - 1) % n;
        if !(in_loop[k] && in_loop[before]) {
            let (i, o) = (strokes[before].tangent(true), s.tangent(false));
            if cross(i, o).atan2(dot(i, o)).abs() >= min {
                corners.push((begins[k], begins[k]));
            }
        }
        if !in_loop[k]
            && radius(s).is_some_and(|r| r <= offset + guard)
            && s.arc.is_some_and(|(_, w)| w.abs() >= min)
        {
            corners.push((begins[k], begins[k] + crate::profiles::stroke_length(s)));
        }
    }
    // Widened by the distance, cut at the loop's start, merged.
    let mut spans: Vec<(f64, f64)> = vec![];
    if corners
        .iter()
        .map(|(a, b)| b - a + 2. * slow.distance_mm)
        .sum::<f64>()
        >= period
    {
        spans.push((0., period));
    } else {
        for (a, b) in corners {
            let (a, b) = (a - slow.distance_mm, b + slow.distance_mm);
            if a < 0. {
                spans.push((a + period, period));
            }
            if b > period {
                spans.push((0., b - period));
            }
            spans.push((a.max(0.), b.min(period)));
        }
    }
    spans.sort_by(|x, y| x.0.total_cmp(&y.0));
    let mut merged: Vec<(f64, f64)> = vec![];
    for (a, b) in spans {
        match merged.last_mut() {
            Some(last) if a <= last.1 + guard => last.1 = last.1.max(b),
            _ => merged.push((a, b)),
        }
    }
    // One stretch across the loop's start counts once.
    let across =
        merged.len() > 1 && merged[0].0 <= guard && merged[merged.len() - 1].1 >= period - guard;
    let count = merged.len() - usize::from(across);
    let slowed_at = |along: f64| {
        let s = along.rem_euclid(period);
        merged.iter().any(|&(a, b)| s >= a && s <= b)
    };
    let mut pieces = vec![];
    let mut s0 = 0.;
    for (k, c) in cuts.iter().enumerate() {
        let length = c.prepare(tolerance)?.length_mm();
        let s1 = s0 + length;
        // Where the feed changes inside this move (not within a guard of
        // its ends: no sliver moves). The moves may pass the loop's start.
        let mut marks = vec![];
        for turn in [0., period] {
            for &(a, b) in &merged {
                for m in [a + turn, b + turn] {
                    if m > s0 + guard && m < s1 - guard {
                        marks.push((m - s0) / length);
                    }
                }
            }
        }
        marks.sort_by(f64::total_cmp);
        marks.dedup_by(|x, y| (*x - *y).abs() * length <= guard);
        marks.insert(0, 0.);
        marks.push(1.);
        for w in marks.windows(2) {
            let piece = if w[0] == 0. && w[1] == 1. {
                c.clone()
            } else {
                super::part_of(c, w[0], w[1])
            };
            pieces.push(Piece {
                source: k,
                curve: piece,
                slowed: slowed_at(s0 + 0.5 * (w[0] + w[1]) * length),
            });
        }
        s0 = s1;
    }
    Ok((pieces, count))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square(reverse: bool) -> Vec<Stroke> {
        // A 20 mm square part cut clockwise from the middle of its bottom
        // edge (the part on the right of travel), with no kerf offset.
        let p = [
            [10., 0.],
            [0., 0.],
            [0., 20.],
            [20., 20.],
            [20., 0.],
            [10., 0.],
        ];
        let s: Vec<Stroke> = p.windows(2).map(|w| Stroke::line(w[0], w[1])).collect();
        if reverse {
            s.into_iter().rev().map(Stroke::reversed).collect()
        } else {
            s
        }
    }

    #[test]
    fn a_loop_leaves_and_returns_along_the_edges_and_lies_in_the_scrap() {
        let loops = CornerLoops {
            overshoot_mm: 3.,
            min_angle_deg: 45.,
            max_angle_deg: 135.,
        };
        for (reverse, side) in [(false, 1.), (true, -1.)] {
            let (strokes, flags, count) = with_loops(&square(reverse), &loops, 0., side, 1e-6);
            // Four corners; the start is on an edge.
            assert_eq!(count, 4);
            assert_eq!(strokes.len(), 5 + 20);
            for (k, s) in strokes.iter().enumerate() {
                // A closed chain, tangent everywhere but at the part's corners.
                let next = strokes[(k + 1) % strokes.len()];
                assert!((s.b[0] - next.a[0]).hypot(s.b[1] - next.a[1]) < 1e-9);
                if !flags[k] {
                    continue;
                }
                // Loop strokes stay outside the part.
                let m = crate::profiles::stroke_point(s, 0.5);
                let inside = m[0] > 1e-9 && m[0] < 20. - 1e-9 && m[1] > 1e-9 && m[1] < 20. - 1e-9;
                assert!(!inside, "{m:?}");
                if let Some((c, w)) = s.arc {
                    // 270 degrees in three quarter arcs of radius 3 for a
                    // square corner.
                    assert!((w.abs() - FRAC_PI_2).abs() < 1e-9);
                    assert!(((s.a[0] - c[0]).hypot(s.a[1] - c[1]) - 3.).abs() < 1e-9);
                }
            }
            // The cut runs straight through each corner: the stroke into a
            // loop and the stroke out of it are tangent to the edges.
            for k in 1..strokes.len() {
                if flags[k] && !flags[k - 1] {
                    let (i, o) = (strokes[k - 1].tangent(true), strokes[k].tangent(false));
                    assert!(cross(i, o).abs() < 1e-9 && dot(i, o) > 0.);
                }
            }
        }
        // The scrap on the other side: the same corners turn away from the
        // part (a hole's corners), and get no loop.
        let (strokes, _, count) = with_loops(&square(false), &loops, 0., -1., 1e-6);
        assert_eq!((strokes.len(), count), (5, 0));
        // Outside the declared angles: none.
        let narrow = CornerLoops {
            min_angle_deg: 100.,
            ..loops
        };
        assert_eq!(with_loops(&square(false), &narrow, 0., 1., 1e-6).2, 0);
        // The reach the nest keeps covers a square corner's loop: 3 / tan(22.5 deg).
        let exact = CornerLoops {
            min_angle_deg: 90.,
            max_angle_deg: 90.,
            ..loops
        };
        assert!((loop_reach_mm(&exact, 0.) - 3. * (1. + 2f64.sqrt())).abs() < 1e-9);
    }

    #[test]
    fn the_offsets_corner_arc_is_replaced_by_a_loop_through_the_edges_meeting_point() {
        // A part below and to the right of its corner (0, 0), the kerf path
        // 1 mm off it: up x = -1, right round the corner, along y = 1.
        let strokes = vec![
            Stroke::line([-1., -10.], [-1., 0.]),
            Stroke {
                a: [-1., 0.],
                b: [0., 1.],
                arc: Some(([0., 0.], -PI / 2.)),
            },
            Stroke::line([0., 1.], [10., 1.]),
        ];
        let loops = CornerLoops {
            overshoot_mm: 2.,
            min_angle_deg: 45.,
            max_angle_deg: 135.,
        };
        let (out, flags, count) = with_loops(&strokes, &loops, 1., 1., 1e-6);
        assert_eq!(count, 1);
        assert_eq!(flags, [false, true, true, true, true, true, false]);
        // On past the meeting point (-1, 1) by 2, back onto the top edge 2
        // before it.
        assert!((out[1].b[0] + 1.).abs() < 1e-9 && (out[1].b[1] - 3.).abs() < 1e-9);
        assert!((out[5].a[0] + 3.).abs() < 1e-9 && (out[5].a[1] - 1.).abs() < 1e-9);
        assert_eq!(out[5].b, [0., 1.]);
        // The loop turns left (away from the part), 270 degrees in all.
        let sweep: f64 = out[2..5].iter().map(|s| s.arc.unwrap().1).sum();
        assert!((sweep - 1.5 * PI).abs() < 1e-9);
    }
}
