//! The shapes of a thermal cut's leads, and where each lies.
//!
//! Cutting CAM offers a lead as a straight line of a length at an angle to
//! the edge, as a tangent arc of a radius and a sweep, and as the two
//! together, the line tangent to the arc (Aerotech CADFusion 3.00.000 help,
//! "Lead Moves": "A lead move can be a line or it can be a circular arc",
//! the line with its angle and length, the arc with the "full angle that the
//! lead move sweeps through" and its radius; Autodesk Fusion, "2D Contour
//! reference", Linking tab: "Lead-In Sweep Angle: Specifies the sweep of the
//! lead-in arc", "Linear Lead-In Distance" for the straight move the
//! controller turns radius compensation on over, the lead-out alike; eCam
//! "Generic cutting module": tangent arc, perpendicular, slant line and
//! "Extension", which puts the leads at a corner). A lead-in runs from the
//! pierce point over its straight part and then its arc onto the cut; the
//! lead-out leaves over its arc, then its straight part, so one `Lead`
//! describes either, read from the cut outward.
//!
//! - `Lead { length_mm, radius_mm }`: the straight part square to the edge,
//!   then a quarter-turn arc tangent to the cut (camlib's lead since the
//!   first thermal cut; unchanged, so its programs are too).
//! - `sweep_deg`: the same with an arc of that sweep (`Lead.arc`, and
//!   `Lead.line_arc` / `Lead.arc_line` with a straight part tangent to it).
//! - `angle_deg`: a straight lead alone, at that angle to the edge's tangent
//!   (`Lead.line`): 90 is square to it, 0 in line with it, along the edge's
//!   extension. An in-line lead at a corner of the kerf path (the arc the
//!   offset puts round a drawn outside corner) runs along the next edge's
//!   extension past the corner; an in-line lead-out there runs on along the
//!   previous edge's extension, and the corner's arc is not cut: the two
//!   straight kerfs cross beyond it (see `corner`).
use super::{Lead, add};
use crate::pocket::Stroke;
use std::f64::consts::FRAC_PI_2;

type P2 = [f64; 2];

/// cos and sin of `deg` degrees, exact at 0 and 90.
fn turn(deg: f64) -> (f64, f64) {
    if deg == 90. {
        (0., 1.)
    } else if deg == 0. {
        (1., 0.)
    } else {
        let a = deg.to_radians();
        (a.cos(), a.sin())
    }
}

/// A lead-in: from the pierce point (`pierce`, before any controller offset)
/// along `dir` to `a`, then the arc about `center` onto the cut (when the
/// lead has one).
#[derive(Clone, Copy, Debug)]
pub(crate) struct In {
    pub pierce: P2,
    pub a: P2,
    pub center: P2,
    /// The straight part's direction of travel (towards the cut).
    pub dir: P2,
}

/// A lead-out: the arc from the cut about `center` to `b` (when the lead has
/// one), then straight along `dir` to `q`.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Out {
    pub b: P2,
    pub center: P2,
    pub q: P2,
    /// The straight part's direction of travel (away from the cut).
    pub dir: P2,
}

impl Lead {
    /// The lead camlib had before shapes: square straight part, quarter arc.
    pub(crate) fn plain(&self) -> bool {
        self.sweep_deg.is_none() && self.angle_deg.is_none()
    }

    /// A straight lead alone, in line with the edge (`Lead.line(l, 0)`).
    pub(crate) fn in_line(&self) -> bool {
        self.angle_deg == Some(0.)
    }

    /// The arc's sweep in radians (unsigned), when the lead has an arc.
    pub(crate) fn sweep_rad(&self) -> f64 {
        match self.sweep_deg {
            None | Some(90.) => FRAC_PI_2,
            Some(deg) => deg.to_radians(),
        }
    }

    fn sweep_turn(&self) -> (f64, f64) {
        turn(self.sweep_deg.unwrap_or(90.))
    }

    /// What the shape needs, refused by name: `which` is "lead_in" or
    /// "lead_out".
    pub(crate) fn validate(&self, which: &str) -> Result<(), String> {
        if !(self.length_mm.is_finite()
            && self.radius_mm.is_finite()
            && self.length_mm >= 0.
            && self.radius_mm >= 0.)
        {
            return Err("lead lengths and radii must not be negative".into());
        }
        if let Some(angle) = self.angle_deg {
            if !(angle.is_finite() && (0. ..=90.).contains(&angle)) {
                return Err(format!(
                    "{which} Lead.line angle_deg {angle} must be 0 (in line with the edge) to 90 (square to it)"
                ));
            }
            if self.radius_mm != 0. || self.sweep_deg.is_some() {
                return Err(format!(
                    "{which}: a Lead.line is straight (angle_deg): it has no radius_mm or sweep_deg; use Lead.line_arc for a straight part and an arc"
                ));
            }
            if self.length_mm <= 0. {
                return Err(format!("{which} Lead.line needs a positive length_mm"));
            }
        }
        if let Some(sweep) = self.sweep_deg
            && !(sweep.is_finite() && sweep > 0. && sweep <= 180. && self.radius_mm > 0.)
        {
            return Err(format!(
                "{which}: an arc lead's sweep_deg {sweep} must be more than 0 and at most 180 (a half turn: beyond it the arc turns back towards the cut), with a positive radius_mm"
            ));
        }
        Ok(())
    }

    /// The farthest any point of the lead lies from where it meets the cut.
    pub(crate) fn reach_mm(&self) -> f64 {
        if self.plain() {
            // The pierce, r along the tangent and r + l along the normal.
            return self.radius_mm.hypot(self.radius_mm + self.length_mm);
        }
        // The farthest point of an arc of at most half a turn is its far
        // end (the chord grows with the sweep up to a half turn); of a
        // straight part, one of its ends.
        let (c, s) = self.sweep_turn();
        let (r, l) = (self.radius_mm, self.length_mm);
        match self.angle_deg {
            Some(_) => l,
            None => {
                let a = [-r * s, r * (1. - c)];
                let far = [a[0] - l * c, a[1] + l * s];
                far[0].hypot(far[1]).max(a[0].hypot(a[1]))
            }
        }
    }

    /// How far the lead reaches back along the cut and out from it, from
    /// where it meets the cut: the largest |t| and the largest n of its
    /// points, the tangent t along the cut and n into the scrap (a lead-in;
    /// a lead-out is its mirror image).
    pub(crate) fn extent(&self) -> [f64; 2] {
        let (r, l) = (self.radius_mm, self.length_mm);
        if let Some(angle) = self.angle_deg {
            let (c, s) = turn(angle);
            return [l * c, l * s];
        }
        let (c, s) = self.sweep_turn();
        let sweep = self.sweep_rad();
        // The arc from the cut: t = -r sin(u), n = r (1 - cos(u)), u to
        // the sweep; |t| is largest at a quarter turn, n at the end.
        let t_arc = if sweep >= FRAC_PI_2 { r } else { r * s };
        let a = [-r * s, r * (1. - c)];
        let far = [a[0] - l * c, a[1] + l * s];
        [t_arc.max(far[0].abs()), a[1].max(far[1])]
    }

    /// The pierce point, in the frame (t, n) of the place it meets the cut.
    pub(crate) fn pierce_tn(&self) -> [f64; 2] {
        let (r, l) = (self.radius_mm, self.length_mm);
        if let Some(angle) = self.angle_deg {
            let (c, s) = turn(angle);
            return [-l * c, l * s];
        }
        let (c, s) = self.sweep_turn();
        [-r * s - l * c, r * (1. - c) + l * s]
    }
}

/// The lead-in meeting the cut at `start`, the cut running along `t`, `n`
/// the scrap side.
pub(crate) fn lead_in(start: P2, t: P2, n: P2, l: &Lead) -> In {
    if let Some(angle) = l.angle_deg {
        let (c, s) = turn(angle);
        let dir = [t[0] * c - n[0] * s, t[1] * c - n[1] * s];
        return In {
            pierce: add(start, dir, -l.length_mm),
            a: start,
            center: start,
            dir,
        };
    }
    let r = l.radius_mm;
    let (c, s) = l.sweep_turn();
    let center = add(start, n, r);
    // Back along the arc by its sweep: a quarter turn puts the arc's start
    // r back along the tangent (written so, it is the same arithmetic as
    // the plain lead's).
    let a = if c == 0. {
        add(center, t, -r * s)
    } else {
        add(add(center, n, -r * c), t, -r * s)
    };
    let dir = if c == 0. {
        [-n[0] * s, -n[1] * s]
    } else {
        [t[0] * c - n[0] * s, t[1] * c - n[1] * s]
    };
    In {
        pierce: add(a, dir, -l.length_mm),
        a,
        center,
        dir,
    }
}

/// The lead-out leaving the cut at `end`, the cut running along `t`, `n` the
/// scrap side.
pub(crate) fn lead_out(end: P2, t: P2, n: P2, l: &Lead) -> Out {
    if let Some(angle) = l.angle_deg {
        let (c, s) = turn(angle);
        let dir = [t[0] * c + n[0] * s, t[1] * c + n[1] * s];
        return Out {
            b: end,
            center: end,
            q: add(end, dir, l.length_mm),
            dir,
        };
    }
    let r = l.radius_mm;
    let (c, s) = l.sweep_turn();
    let center = add(end, n, r);
    let b = if c == 0. {
        add(center, t, r * s)
    } else {
        add(add(center, n, -r * c), t, r * s)
    };
    let dir = if c == 0. {
        [n[0] * s, n[1] * s]
    } else {
        [t[0] * c + n[0] * s, t[1] * c + n[1] * s]
    };
    Out {
        b,
        center,
        q: add(b, dir, l.length_mm),
        dir,
    }
}

/// An arc of the kerf path round a drawn outside corner: its radius is the
/// path's offset from the drawn edge.
pub(crate) fn is_round(s: &Stroke, offset: f64, tolerance: f64) -> bool {
    s.arc
        .is_some_and(|(c, _)| ((s.a[0] - c[0]).hypot(s.a[1] - c[1]) - offset).abs() <= tolerance)
}

/// Whether the loop ends with the arc round a drawn outside corner between
/// two straight edges (the loop's last stroke and its first).
pub(crate) fn ends_at_corner(strokes: &[Stroke], offset: f64, tolerance: f64) -> bool {
    let n = strokes.len();
    n >= 3
        && is_round(&strokes[n - 1], offset, tolerance)
        && strokes[n - 1]
            .arc
            .is_some_and(|(_, w)| w.abs() < std::f64::consts::PI - 1e-9)
        && strokes[n - 2].arc.is_none()
        && strokes[0].arc.is_none()
}

/// An in-line lead-in at a corner: the loop restarted where the corner's
/// arc ends, that arc whole at its end, so the lead comes in along the next
/// edge's extension past the corner. None where the loop does not start on
/// such an arc (or at its end) between two straight edges.
pub(crate) fn corner(strokes: &[Stroke], offset: f64, tolerance: f64) -> Option<Vec<Stroke>> {
    let r = corner_start(strokes, offset, tolerance)?;
    ends_at_corner(&r, offset, tolerance).then_some(r)
}

fn corner_start(strokes: &[Stroke], offset: f64, tolerance: f64) -> Option<Vec<Stroke>> {
    let n = strokes.len();
    if n < 3 {
        return None;
    }
    let round = |s: &Stroke| is_round(s, offset, tolerance);
    let same = |a: &Stroke, b: &Stroke| match (a.arc, b.arc) {
        (Some((c, w)), Some((d, v))) => {
            (c[0] - d[0]).hypot(c[1] - d[1]) <= tolerance && w.signum() == v.signum()
        }
        _ => false,
    };
    let (first, last) = (&strokes[0], &strokes[n - 1]);
    if round(first) {
        // On the arc (at its start or inside it): the arc's piece before
        // the start, if any, is the loop's last stroke; join the two.
        let mut r: Vec<Stroke> = strokes[1..].to_vec();
        if same(first, last) && round(last) {
            let before = r.pop().unwrap();
            let (c, w) = before.arc.unwrap();
            r.push(Stroke {
                a: before.a,
                b: first.b,
                arc: Some((c, w + first.arc.unwrap().1)),
            });
        } else {
            r.push(*first);
        }
        return Some(r);
    }
    round(last).then(|| strokes.to_vec())
}

/// Where the two edges' kerf lines meet at a corner whose arc runs from `a`
/// (tangent `ta`) to `b` (tangent `tb`), and how far that is from `b` and
/// from `a`. None when the edges are near parallel there.
pub(crate) fn crossing(a: P2, ta: P2, b: P2, tb: P2) -> Option<(P2, f64, f64)> {
    let d = super::cross(ta, tb);
    if d.abs() <= 1e-9 {
        return None;
    }
    let w = [b[0] - a[0], b[1] - a[1]];
    // a + u ta = b - v tb, so u ta + v tb = b - a.
    let u = super::cross(w, tb) / d;
    let v = super::cross(ta, w) / d;
    let x = add(a, ta, u);
    (u > 0. && v > 0.).then_some((x, v, u))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lead(length_mm: f64, radius_mm: f64) -> Lead {
        Lead {
            length_mm,
            radius_mm,
            sweep_deg: None,
            angle_deg: None,
        }
    }

    #[test]
    fn the_plain_lead_is_the_quarter_line_arc() {
        // From (0, 0) along +x, scrap on +y: the arc's start r back and r
        // out, the pierce l further out, square to the edge.
        let l = lead(2., 3.);
        let i = lead_in([0., 0.], [1., 0.], [0., 1.], &l);
        assert_eq!(i.center, [0., 3.]);
        assert_eq!(i.a, [-3., 3.]);
        assert_eq!(i.pierce, [-3., 5.]);
        let o = lead_out([0., 0.], [1., 0.], [0., 1.], &l);
        assert_eq!((o.b, o.q), ([3., 3.], [3., 5.]));
        // An explicit quarter turn is the same lead.
        let q = Lead {
            sweep_deg: Some(90.),
            ..l
        };
        let j = lead_in([0., 0.], [1., 0.], [0., 1.], &q);
        assert_eq!((j.a, j.pierce), (i.a, i.pierce));
        assert_eq!(l.extent(), [3., 5.]);
        assert_eq!(l.pierce_tn(), [-3., 5.]);
    }

    #[test]
    fn shapes_lie_where_their_parameters_say() {
        let (t, n) = ([1., 0.], [0., 1.]);
        // A line at 30 degrees: its pierce 5 back along the edge's tangent
        // at that angle.
        let line = Lead {
            length_mm: 5.,
            radius_mm: 0.,
            sweep_deg: None,
            angle_deg: Some(30.),
        };
        let i = lead_in([0., 0.], t, n, &line);
        let (c, s) = (30f64.to_radians().cos(), 30f64.to_radians().sin());
        assert!((i.pierce[0] + 5. * c).abs() < 1e-12 && (i.pierce[1] - 5. * s).abs() < 1e-12);
        let o = lead_out([0., 0.], t, n, &line);
        assert!((o.q[0] - 5. * c).abs() < 1e-12 && (o.q[1] - 5. * s).abs() < 1e-12);
        // In line: straight back along the edge.
        let inline = Lead {
            angle_deg: Some(0.),
            ..line
        };
        assert_eq!(lead_in([0., 0.], t, n, &inline).pierce, [-5., 0.]);
        assert_eq!(lead_out([0., 0.], t, n, &inline).q, [5., 0.]);
        assert_eq!(inline.reach_mm(), 5.);
        // A half-turn arc of radius 2 with a straight part of 1 tangent to
        // it: the arc starts 4 out, square above the start, and the line
        // runs on from there along the edge's direction.
        let half = Lead {
            length_mm: 1.,
            radius_mm: 2.,
            sweep_deg: Some(180.),
            angle_deg: None,
        };
        let i = lead_in([0., 0.], t, n, &half);
        assert!((i.a[0]).abs() < 1e-12 && (i.a[1] - 4.).abs() < 1e-12);
        assert!((i.pierce[0] - 1.).abs() < 1e-12 && (i.pierce[1] - 4.).abs() < 1e-12);
        assert!((half.extent()[0] - 2.).abs() < 1e-12 && (half.extent()[1] - 4.).abs() < 1e-12);
        assert!((half.reach_mm() - 17f64.sqrt()).abs() < 1e-12);
        // An arc of 45 degrees without a straight part.
        let arc = Lead {
            length_mm: 0.,
            radius_mm: 4.,
            sweep_deg: Some(45.),
            angle_deg: None,
        };
        let i = lead_in([0., 0.], t, n, &arc);
        let r = 4.;
        let h = std::f64::consts::FRAC_PI_4;
        assert!((i.pierce[0] + r * h.sin()).abs() < 1e-12);
        assert!((i.pierce[1] - r * (1. - h.cos())).abs() < 1e-12);
        assert!((i.a[0] - i.pierce[0]).abs() < 1e-15);
    }

    #[test]
    fn shapes_refuse_what_they_cannot_be() {
        let ok = |l: Lead| l.validate("lead_in");
        let line = |angle: f64| Lead {
            length_mm: 3.,
            radius_mm: 0.,
            sweep_deg: None,
            angle_deg: Some(angle),
        };
        assert!(ok(line(0.)).is_ok() && ok(line(90.)).is_ok());
        assert!(ok(line(91.)).unwrap_err().contains("angle_deg"));
        assert!(ok(line(-1.)).unwrap_err().contains("angle_deg"));
        assert!(
            ok(Lead {
                radius_mm: 1.,
                ..line(30.)
            })
            .unwrap_err()
            .contains("straight")
        );
        assert!(
            ok(Lead {
                length_mm: 0.,
                ..line(30.)
            })
            .unwrap_err()
            .contains("positive length_mm")
        );
        let arc = |sweep: f64, r: f64| Lead {
            length_mm: 0.,
            radius_mm: r,
            sweep_deg: Some(sweep),
            angle_deg: None,
        };
        assert!(ok(arc(180., 2.)).is_ok() && ok(arc(10., 2.)).is_ok());
        for bad in [arc(0., 2.), arc(181., 2.), arc(90., 0.), arc(f64::NAN, 2.)] {
            assert!(ok(bad).unwrap_err().contains("sweep_deg"), "{bad:?}");
        }
    }

    #[test]
    fn an_in_line_start_at_a_corner_moves_to_the_end_of_its_arc() {
        let h = FRAC_PI_2;
        // A 10 x 6 rectangle's kerf path half a millimetre out, started in
        // the middle of the arc round its corner (0, 0).
        let mid = [
            -0.5 * std::f64::consts::FRAC_1_SQRT_2,
            -0.5 * std::f64::consts::FRAC_1_SQRT_2,
        ];
        let path = vec![
            Stroke {
                a: mid,
                b: [0., -0.5],
                arc: Some(([0., 0.], h / 2.)),
            },
            Stroke::line([0., -0.5], [10., -0.5]),
            Stroke {
                a: [10., -0.5],
                b: [10.5, 0.],
                arc: Some(([10., 0.], h)),
            },
            Stroke::line([10.5, 0.], [10.5, 6.]),
            Stroke {
                a: [-0.5, 0.],
                b: mid,
                arc: Some(([0., 0.], h / 2.)),
            },
        ];
        let r = corner(&path, 0.5, 1e-9).unwrap();
        assert_eq!(r[0].a, [0., -0.5]);
        let last = r[r.len() - 1];
        assert_eq!((last.a, last.b), ([-0.5, 0.], [0., -0.5]));
        assert!((last.arc.unwrap().1 - h).abs() < 1e-15);
        // Already at the arc's end: as it is. On a straight edge: none.
        assert_eq!(corner(&r, 0.5, 1e-9).unwrap().len(), r.len());
        let edge = [path[1], path[2], path[3]];
        assert!(corner(&edge, 0.5, 1e-9).is_none());
        // Where the two edges' kerf lines cross: the kerf corner (-0.5, -0.5),
        // half a millimetre from either end of the arc.
        let (x, from_b, from_a) =
            crossing(last.a, last.tangent(false), last.b, last.tangent(true)).unwrap();
        assert!((x[0] + 0.5).abs() < 1e-12 && (x[1] + 0.5).abs() < 1e-12);
        assert!((from_a - 0.5).abs() < 1e-12 && (from_b - 0.5).abs() < 1e-12);
    }
}
