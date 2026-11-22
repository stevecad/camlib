//! A controller's cutter compensation by a whole amount, replayed from the
//! written path: where the tool center runs when the program carries the
//! part side and G41/G42 with the offset (`ThermalCut(compensation="control")`).
//!
//! This is the decoders' model of the controller. It reads only the written
//! curves, the side and the amount; it shares no code with the planner (which
//! builds the beam path from the part) or with the post (which writes the
//! part side from the beam path).
//!
//! The model, as WinCNC documents it (Users Guide, Controller 3.1, revision
//! 3.1.04, pp. 138-139: "G41 Cutter Compensation Left - G41 O# ... Starts to
//! the left on the first move. O# is the compensation amount"; p. 72,
//! `cutcomp=T0`: "does not insert entry move"; Versions Information List
//! 3.1.01a: arcs are offset and "a new intersection point" calculated):
//!
//! - Each written line or arc is offset by the amount to the side (a line
//!   moves along its normal, an arc keeps its center and changes its
//!   radius).
//! - Consecutive offset elements are joined at their intersection: trimmed
//!   where they overlap (inside a turn), extended where they part (outside
//!   it). Outside a turn a controller may instead join with an arc about the
//!   written corner; that join is reported (`arc_joins`) with the reach of
//!   the intersection beyond it, so both can be bounded. The intersection
//!   join sweeps everything the arc join does there, and further into the
//!   side the tool is on.
//! - The entry move (the first written move, compensation turning on) is a
//!   straight line from the uncompensated position to the offset start of
//!   the next element; the cancel move (the last, compensation turning off)
//!   a straight line from the offset end of the element before it to its
//!   written end. Both must run tangent into and out of the compensated
//!   path: at a corner, controllers differ on where a start-up block ends.
//!
//! Near-tangent junctions (rounded words leave consecutive elements a hair
//! off tangent) are joined at the offsets' intersection where it is local,
//! else straight across the gap between the offset ends; the gap is
//! reported (`max_gap_mm`) and refused beyond a quarter of the tolerance.
//! The same quarter bounds how far the entry and cancel moves may turn into
//! the path: amount x sin(turn), the most two controllers' start-up (or
//! cancel) positions can differ there. Rounded words leave such a hair of
//! a turn (at four decimals of an inch, about 1.4 um on a 1.5 mm lead with
//! a 0.8 mm amount); it is the one part of the controller's path this model
//! does not decide. It is reported (`residual_mm`), and the post engine adds
//! it to each replayed move's checked deviation, which must stay within the
//! output tolerance.
use crate::motion::{CompensationSide, Curve, Plane, Vec3};
use serde::Serialize;
use std::f64::consts::{PI, TAU};

/// The replayed tool-center path: one curve per written curve (entry,
/// compensated elements, cancel), and the corners where a controller could
/// join with an arc instead.
#[derive(Clone, Debug, Serialize)]
pub struct Replay {
    pub path: Vec<Curve>,
    pub arc_joins: Vec<ArcJoin>,
    /// The widest gap between two offset ends joined straight across.
    pub max_gap_mm: f64,
    /// How far the controller's path may be from `path` where this model
    /// does not decide it: those gaps, and amount x sin(turn) where the
    /// entry or cancel move turns into the path. A caller checking `path`
    /// against a plan within a tolerance adds it.
    pub residual_mm: f64,
}

/// A corner outside the turn: the arc join about the written corner, and
/// how far the intersection join reaches from that corner.
#[derive(Clone, Debug, Serialize)]
pub struct ArcJoin {
    /// Index of the written element the corner ends.
    pub after: usize,
    pub corner_mm: [f64; 2],
    pub arc: Curve,
    pub intersection_mm: [f64; 2],
    pub reach_mm: f64,
}

#[derive(Clone, Copy, Debug)]
enum Primitive {
    Line {
        point: [f64; 2],
        direction: [f64; 2],
    },
    Circle {
        center: [f64; 2],
        radius: f64,
    },
}

fn xy(p: Vec3) -> [f64; 2] {
    [p[0], p[1]]
}
fn sub(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] - b[0], a[1] - b[1]]
}
fn add(a: [f64; 2], b: [f64; 2], k: f64) -> [f64; 2] {
    [a[0] + k * b[0], a[1] + k * b[1]]
}
fn dot(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
fn cross(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}
fn norm(a: [f64; 2]) -> f64 {
    a[0].hypot(a[1])
}
fn unit(a: [f64; 2]) -> Option<[f64; 2]> {
    let n = norm(a);
    (n > 0. && n.is_finite()).then(|| [a[0] / n, a[1] / n])
}
fn left(v: [f64; 2]) -> [f64; 2] {
    [-v[1], v[0]]
}
/// Angle in (-pi, pi].
fn wrap(a: f64) -> f64 {
    let r = a.rem_euclid(TAU);
    if r > PI { r - TAU } else { r }
}

/// The unit tangent of a written XY curve at its start (`end` false) or
/// end, in the direction of travel.
fn tangent(c: &Curve, end: bool) -> Result<[f64; 2], String> {
    match *c {
        Curve::Line { start, end: e } => {
            unit(sub(xy(e), xy(start))).ok_or_else(|| "a written move of zero length".into())
        }
        Curve::Arc {
            start,
            end: e,
            center,
            sweep_rad,
            ..
        } => {
            let p = if end { e } else { start };
            let r = unit(sub(xy(p), xy(center))).ok_or("an arc of zero radius")?;
            let t = left(r);
            Ok(if sweep_rad > 0. { t } else { [-t[0], -t[1]] })
        }
        Curve::Helix { .. } => Err("a helix under cutter compensation".into()),
    }
}

/// Unit normal on the compensation side at the start or end of a curve.
fn side_normal(c: &Curve, end: bool, s: f64) -> Result<[f64; 2], String> {
    let t = tangent(c, end)?;
    let n = left(t);
    Ok([s * n[0], s * n[1]])
}

fn offset_primitive(c: &Curve, s: f64, r: f64, index: usize) -> Result<Primitive, String> {
    match *c {
        Curve::Line { start, .. } => {
            let d = tangent(c, false)?;
            let n = left(d);
            Ok(Primitive::Line {
                point: add(xy(start), n, s * r),
                direction: d,
            })
        }
        Curve::Arc {
            start,
            center,
            sweep_rad,
            ..
        } => {
            let rho = norm(sub(xy(start), xy(center)));
            // Left of travel points to the center on a counter-clockwise
            // arc: an offset to the left shrinks it.
            let radius = rho - s * sweep_rad.signum() * r;
            if radius.is_nan() || radius <= 1e-6 {
                return Err(format!(
                    "written element {index} is an arc of radius {rho:.4} mm that an offset of {r:.4} mm turns inside out: the controller cannot offset it"
                ));
            }
            Ok(Primitive::Circle {
                center: xy(center),
                radius,
            })
        }
        Curve::Helix { .. } => Err("a helix under cutter compensation".into()),
    }
}

/// The point of a primitive nearest to `p`.
fn project(p: &Primitive, q: [f64; 2]) -> [f64; 2] {
    match *p {
        Primitive::Line { point, direction } => {
            add(point, direction, dot(sub(q, point), direction))
        }
        Primitive::Circle { center, radius } => match unit(sub(q, center)) {
            Some(u) => add(center, u, radius),
            None => add(center, [1., 0.], radius),
        },
    }
}

fn intersections(a: &Primitive, b: &Primitive) -> Vec<[f64; 2]> {
    match (*a, *b) {
        (
            Primitive::Line {
                point: p,
                direction: d,
            },
            Primitive::Line {
                point: q,
                direction: e,
            },
        ) => {
            let den = cross(d, e);
            if den.abs() < 1e-15 {
                return vec![];
            }
            let t = cross(sub(q, p), e) / den;
            vec![add(p, d, t)]
        }
        (Primitive::Line { point, direction }, Primitive::Circle { center, radius })
        | (Primitive::Circle { center, radius }, Primitive::Line { point, direction }) => {
            let foot = add(point, direction, dot(sub(center, point), direction));
            let h = norm(sub(foot, center));
            if h > radius {
                return vec![];
            }
            let w = (radius * radius - h * h).max(0.).sqrt();
            vec![add(foot, direction, -w), add(foot, direction, w)]
        }
        (
            Primitive::Circle {
                center: c1,
                radius: r1,
            },
            Primitive::Circle {
                center: c2,
                radius: r2,
            },
        ) => {
            let d = norm(sub(c2, c1));
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

/// The offset element `index` from `from` to `to` on its primitive, in the
/// written element's direction; refused when the offset trims it away.
fn element(
    c: &Curve,
    primitive: &Primitive,
    from: [f64; 2],
    to: [f64; 2],
    z: f64,
    index: usize,
) -> Result<Curve, String> {
    let at = |p: [f64; 2]| [p[0], p[1], z];
    match (c, *primitive) {
        (&Curve::Line { .. }, Primitive::Line { direction, .. }) => {
            if dot(sub(to, from), direction) <= 0. {
                return Err(format!(
                    "the controller's offset of written element {index} has no length left (it is shorter than its corners' trims): a feature narrower than the kerf, or an offset that crosses itself"
                ));
            }
            Ok(Curve::Line {
                start: at(from),
                end: at(to),
            })
        }
        (
            &Curve::Arc {
                start,
                end,
                sweep_rad,
                ..
            },
            Primitive::Circle { center, .. },
        ) => {
            let angle = |p: [f64; 2]| (p[1] - center[1]).atan2(p[0] - center[0]);
            let a0 = angle(xy(start));
            let a1 = angle(xy(end));
            let sweep = sweep_rad + wrap(angle(to) - a1) - wrap(angle(from) - a0);
            if sweep * sweep_rad <= 0. || sweep.abs() >= TAU {
                return Err(format!(
                    "the controller's offset of written arc {index} has no sweep left (its corners' trims meet): a feature narrower than the kerf, or an offset that crosses itself"
                ));
            }
            Ok(Curve::Arc {
                start: at(from),
                end: at(to),
                center: at(center),
                plane: Plane::Xy,
                sweep_rad: sweep,
            })
        }
        _ => unreachable!("primitive follows its curve"),
    }
}

/// Replays the controller's offset of `written` (the entry move, the
/// compensated elements, the cancel move) by `amount_mm` to `side`.
/// `tolerance_mm` bounds the near-tangent gaps joined straight across.
pub fn replay(
    written: &[Curve],
    side: CompensationSide,
    amount_mm: f64,
    tolerance_mm: f64,
) -> Result<Replay, String> {
    let s = match side {
        CompensationSide::Left => 1.,
        CompensationSide::Right => -1.,
        CompensationSide::Off => return Err("no compensation side to replay".into()),
    };
    let r = amount_mm;
    if !(r.is_finite() && r > 0.) || !(tolerance_mm.is_finite() && tolerance_mm > 0.) {
        return Err("the compensation amount and the tolerance must be positive".into());
    }
    let n = written.len();
    if n < 3 {
        return Err(
            "controller compensation needs an entry move, at least one compensated move and a cancel move"
                .into(),
        );
    }
    let z = written[0].start()[2];
    for (k, c) in written.iter().enumerate() {
        if c.start()[2] != z || c.end()[2] != z {
            return Err(format!(
                "written element {k} leaves the compensation plane: only XY moves at one height are offset"
            ));
        }
        if matches!(c, Curve::Helix { .. }) {
            return Err("a helix under cutter compensation".into());
        }
        if k > 0 && xy(written[k - 1].end()) != xy(c.start()) {
            return Err(format!(
                "written element {k} does not start where {} ends",
                k - 1
            ));
        }
    }
    for (k, which) in [(0, "entry"), (n - 1, "cancel")] {
        if !matches!(written[k], Curve::Line { .. }) {
            return Err(format!(
                "the {which} move of controller compensation must be a straight line (the controller refuses it on an arc)"
            ));
        }
    }
    let gap_limit = tolerance_mm / 4.;
    let elements = &written[1..n - 1];
    let primitives = elements
        .iter()
        .enumerate()
        .map(|(k, c)| offset_primitive(c, s, r, k + 1))
        .collect::<Result<Vec<_>, _>>()?;
    // The entry and cancel moves run tangent into and out of the path.
    let mut residual: f64 = 0.;
    for (a, b, which) in [
        (&written[0], &elements[0], "entry"),
        (&elements[elements.len() - 1], &written[n - 1], "cancel"),
    ] {
        let (ta, tb) = (tangent(a, true)?, tangent(b, false)?);
        residual = residual.max(r * cross(ta, tb).abs());
        if r * cross(ta, tb).abs() > gap_limit || dot(ta, tb) <= 0. {
            return Err(format!(
                "the {which} move turns {:.3} degrees into the compensated path: where a controller ends its start-up or cancel at a corner differs between controllers; run it tangent (a lead arc)",
                cross(ta, tb).atan2(dot(ta, tb)).abs().to_degrees()
            ));
        }
    }
    let mut joints: Vec<[f64; 2]> = Vec::with_capacity(elements.len() + 1);
    joints.push(add(
        xy(elements[0].start()),
        side_normal(&elements[0], false, s)?,
        r,
    ));
    let mut arc_joins = vec![];
    let mut max_gap: f64 = 0.;
    for k in 0..elements.len() - 1 {
        let (a, b) = (&elements[k], &elements[k + 1]);
        let corner = xy(a.end());
        let (na, nb) = (side_normal(a, true, s)?, side_normal(b, false, s)?);
        let (pa, pb) = (add(corner, na, r), add(corner, nb, r));
        let (ta, tb) = (tangent(a, true)?, tangent(b, false)?);
        let turn = cross(ta, tb).atan2(dot(ta, tb));
        if turn.abs() > PI - 1e-6 {
            return Err(format!(
                "the written path reverses at ({:.4}, {:.4}): the controller cannot offset it",
                corner[0], corner[1]
            ));
        }
        let expected = match unit([na[0] + nb[0], na[1] + nb[1]]) {
            Some(m) => add(corner, m, r / (turn / 2.).cos()),
            None => pa,
        };
        let window = 4. * r * (turn / 2.).tan().abs() + gap_limit;
        let found = intersections(&primitives[k], &primitives[k + 1])
            .into_iter()
            .filter(|p| norm(sub(*p, expected)) <= window)
            .min_by(|p, q| norm(sub(*p, expected)).total_cmp(&norm(sub(*q, expected))));
        let joint = match found {
            Some(p) => p,
            None => {
                let gap = norm(sub(pa, pb));
                if gap > gap_limit {
                    return Err(format!(
                        "the controller's offset elements do not meet at ({:.4}, {:.4}) (written elements {} and {}): it cannot form the offset there",
                        corner[0],
                        corner[1],
                        k + 1,
                        k + 2
                    ));
                }
                max_gap = max_gap.max(gap);
                project(&primitives[k + 1], expected)
            }
        };
        // Outside the turn (the offset side away from it): a controller
        // may join with an arc about the written corner instead.
        if s * turn < 0. && r * turn.abs() > gap_limit {
            arc_joins.push(ArcJoin {
                after: k + 1,
                corner_mm: corner,
                arc: Curve::Arc {
                    start: [pa[0], pa[1], z],
                    end: [pb[0], pb[1], z],
                    center: [corner[0], corner[1], z],
                    plane: Plane::Xy,
                    sweep_rad: turn,
                },
                intersection_mm: joint,
                reach_mm: norm(sub(joint, corner)),
            });
        }
        joints.push(joint);
    }
    let last = &elements[elements.len() - 1];
    joints.push(add(xy(last.end()), side_normal(last, true, s)?, r));
    let mut path = Vec::with_capacity(n);
    path.push(Curve::Line {
        start: written[0].start(),
        end: [joints[0][0], joints[0][1], z],
    });
    for (k, c) in elements.iter().enumerate() {
        path.push(element(
            c,
            &primitives[k],
            joints[k],
            joints[k + 1],
            z,
            k + 1,
        )?);
    }
    let end = joints[joints.len() - 1];
    path.push(Curve::Line {
        start: [end[0], end[1], z],
        end: written[n - 1].end(),
    });
    Ok(Replay {
        path,
        arc_joins,
        max_gap_mm: max_gap,
        residual_mm: residual.max(max_gap),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line(a: [f64; 2], b: [f64; 2]) -> Curve {
        Curve::Line {
            start: [a[0], a[1], 0.],
            end: [b[0], b[1], 0.],
        }
    }

    #[test]
    fn a_square_offset_outside_meets_at_its_corners_and_reports_the_arc_joins() {
        // Clockwise round a 10 mm square from the middle of its left side,
        // the tool on the left (outside): entry and cancel along the side.
        let w = vec![
            line([0., 2.], [0., 5.]),
            line([0., 5.], [0., 10.]),
            line([0., 10.], [10., 10.]),
            line([10., 10.], [10., 0.]),
            line([10., 0.], [0., 0.]),
            line([0., 0.], [0., 4.]),
            line([0., 4.], [0., 6.]),
        ];
        let replay = replay(&w, CompensationSide::Left, 1., 0.01).unwrap();
        let ends: Vec<[f64; 3]> = replay.path.iter().map(|c| c.end()).collect();
        assert_eq!(ends[0], [-1., 5., 0.]);
        for (got, want) in ends[1..5]
            .iter()
            .zip([[-1., 11.], [11., 11.], [11., -1.], [-1., -1.]])
        {
            assert!(
                (got[0] - want[0]).abs() < 1e-12 && (got[1] - want[1]).abs() < 1e-12,
                "{got:?}"
            );
        }
        assert_eq!(ends[6], [0., 6., 0.]);
        assert_eq!(replay.arc_joins.len(), 4);
        for j in &replay.arc_joins {
            assert!((j.reach_mm - 2f64.sqrt()).abs() < 1e-12);
        }
    }

    #[test]
    fn inside_corners_trim_and_arcs_keep_their_centers() {
        // A hole: counter-clockwise, tool on the left (inside).
        let w = vec![
            line([5., 5.], [5., 8.]),
            line([5., 8.], [5., 10.]),
            Curve::Arc {
                start: [5., 10., 0.],
                end: [5., 0., 0.],
                center: [5., 5., 0.],
                plane: Plane::Xy,
                sweep_rad: PI,
            },
            line([5., 0.], [5., 2.]),
            line([5., 2.], [6., 2.]),
        ];
        // The entry runs up along +y into an arc starting at its top going
        // counter-clockwise: tangent is -x there, not +y.
        assert!(replay(&w, CompensationSide::Left, 1., 0.01).is_err());
        let w = vec![
            line([8., 10.], [5., 10.]),
            Curve::Arc {
                start: [5., 10., 0.],
                end: [5., 0., 0.],
                center: [5., 5., 0.],
                plane: Plane::Xy,
                sweep_rad: PI,
            },
            line([5., 0.], [7., 0.]),
        ];
        let replay = replay(&w, CompensationSide::Left, 1., 0.01).unwrap();
        let Curve::Arc { center, start, .. } = replay.path[1] else {
            panic!()
        };
        assert_eq!(center, [5., 5., 0.]);
        assert!((norm(sub(xy(start), xy(center))) - 4.).abs() < 1e-12);
        assert!(replay.arc_joins.is_empty());
    }

    #[test]
    fn an_offset_that_trims_an_element_away_is_refused() {
        // A 1 mm notch step with a 2 mm offset inside the turns.
        let w = vec![
            line([0., -3.], [0., 0.]),
            line([0., 0.], [0., 10.]),
            line([0., 10.], [1., 10.]),
            line([1., 10.], [1., 0.]),
            line([1., 0.], [1., -3.]),
        ];
        let e = replay(&w, CompensationSide::Right, 2., 0.01).unwrap_err();
        assert!(e.contains("no length left"), "{e}");
    }
}
