//! A second chucking (#418): the jaws on the part itself, and the stretch
//! of the profile one chucking turns.
//!
//! A turned part is normally made in two chuckings: one end is turned, the
//! part is taken out, turned end for end and gripped on a diameter the first
//! chucking finished, and the other end is turned. Each chucking's program
//! holds only the portion of the profile it turns, as the contour between
//! the blocks P and Q of a stock removal cycle does (FANUC Series 0i-TD
//! Operator's Manual, lathe system, B-64304EN-1, "Stock removal in turning
//! (G71)" and "Finishing cycle (G70)").
//!
//! The chuck is what it was before: everything behind the chuck (jaw) face
//! `chuck_z_mm` is closed to every part of every tool. That half-space
//! contains the entry-inhibition area a lathe control checks tool tips
//! against, the jaws' outline set by their length and depth and the chuck's
//! position (the same manual, "Chuck and tail stock barriers"), whatever
//! the jaws' own dimensions are. What a second chucking adds is that the
//! face may sit along the part: a declared `ChuckGrip` says the jaws hold
//! the work over a length behind that face on a stated radius. Nothing
//! behind the face is machined; the grip is checked against the stock, so a
//! chuck face set by mistake onto a shoulder, a taper or thin air is refused
//! rather than planned around.
//!
//! camlib does not judge whether a grip holds against the cutting forces
//! (jaw type, pressure, friction and the cut decide that; it knows none).
use super::{
    ops::{TurnOperation, TurningJob},
    profile::{Envelope, Seg},
    stock::TurnStock,
};
use serde::{Deserialize, Serialize};

/// An overlap along z shorter than this is rounding, not material.
const SLIVER: f64 = 1e-9;

/// The jaws on the work (a lathe setup's `grip`): they hold it from
/// `chuck_z_mm - length_mm` to the chuck face `chuck_z_mm`, closed on
/// `radius_mm`. With a grip the chuck face may sit along the part.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ChuckGrip {
    /// How far behind the chuck face the jaws hold the work.
    pub length_mm: f64,
    /// The radius the jaws close on: the stock's own over that length.
    pub radius_mm: f64,
}

impl ChuckGrip {
    /// The z range the jaws hold: [back, chuck face].
    pub fn span(&self, chuck_z_mm: f64) -> [f64; 2] {
        [chuck_z_mm - self.length_mm, chuck_z_mm]
    }

    /// The grip against the setup's stock and part: the stock is the
    /// cylinder the jaws close on over the whole held length (within the
    /// setup's tolerance; in the bin its own end lies in, no larger), the
    /// part is not larger than it there, and some of the part lies in
    /// front of the chuck clearance.
    pub(super) fn check(
        &self,
        job: &TurningJob,
        part: &Envelope,
        stock: &TurnStock,
    ) -> Result<(), String> {
        let tol = job.tolerance_mm;
        let radius = self.radius_mm;
        if !(self.length_mm.is_finite() && self.length_mm > 0. && radius.is_finite() && radius > 0.)
        {
            return Err("the grip's length_mm and radius_mm must be positive".into());
        }
        let [back, face] = self.span(job.chuck_z_mm);
        let [z_lo, z_hi] = job.part.z_range();
        if z_hi <= face + job.clearance_mm {
            return Err(format!(
                "no part of the profile lies in front of the chuck clearance (z {}: chuck_z_mm {face} plus clearance_mm {}); the part's front face is at z {z_hi}",
                face + job.clearance_mm,
                job.clearance_mm
            ));
        }
        if back < job.stock.z_min_mm - tol {
            return Err(format!(
                "the grip holds z {back} to z {face} (length_mm {} behind the chuck face), but the stock ends at z {}: the jaws would close on nothing there; shorten the grip or correct the stock",
                self.length_mm, job.stock.z_min_mm
            ));
        }
        for (i, c) in stock.cells().iter().enumerate() {
            let [z0, z1] = stock.bin(i);
            if z1.min(face) - z0.max(back) <= SLIVER {
                continue;
            }
            // The bin the stock's chuck-side end lies in: its end face is
            // somewhere in it, so its radius is only known not to exceed
            // the cylinder's (a faced end leaves exactly that: the bounds
            // hold for every z of the bin, the face's plane included). The
            // jaws may overhang the end of the work there.
            let holds = |c: &super::stock::TurnCell| c.outer.upper_mm > c.inner.lower_mm;
            let end = i == 0 || !holds(&stock.cells()[i - 1]);
            let found = if !holds(c) {
                "no material".to_string()
            } else if (c.outer.upper_mm - radius).abs() > tol
                || (!end && (c.outer.lower_mm - radius).abs() > tol)
            {
                format!(
                    "a radius between {} and {}",
                    c.outer.lower_mm, c.outer.upper_mm
                )
            } else {
                continue;
            };
            return Err(format!(
                "the grip closes on radius {radius} from z {back} to z {face}, but the stock between z {z0} and z {z1} is {found}: jaws hold a parallel diameter. Declare the grip where the stock is that cylinder (its length_mm and radius_mm), or correct the stock or the chuck face"
            ));
        }
        if let Some([_, high]) = part.radius_range(back.max(z_lo), face.min(z_hi))
            && high > radius + tol
        {
            return Err(format!(
                "the part reaches radius {high} between z {back} and z {face}, more than the radius {radius} the grip closes on there: the jaws would be inside the part"
            ));
        }
        Ok(())
    }
}

/// With a declared grip: an operation whose own target lies behind the
/// chuck face asks to cut where the jaws hold the work. Refused by name,
/// before any move. (Roughing and finishing have no such target: they stop
/// at the clearance in front of the face, and their `z_to_mm` is refused by
/// `window`.)
pub(super) fn in_front_of_jaws(op: &TurnOperation, job: &TurningJob) -> Result<(), String> {
    let Some(grip) = &job.grip else {
        return Ok(());
    };
    let (what, z) = match op {
        TurnOperation::Face { z_mm, .. } => ("its z_mm", *z_mm),
        TurnOperation::Groove {
            z_from_mm, z_to_mm, ..
        } => ("the groove's far wall", z_from_mm.min(*z_to_mm)),
        TurnOperation::PartOff { z_mm, .. } => ("its z_mm", *z_mm),
        TurnOperation::Thread {
            z_start_mm,
            z_end_mm,
            ..
        } => ("the thread's far end", z_start_mm.min(*z_end_mm)),
        TurnOperation::BoreRough { z_end_mm, .. } | TurnOperation::BoreFinish { z_end_mm, .. } => {
            ("its z_end_mm", *z_end_mm)
        }
        TurnOperation::FaceGroove { bottom_z_mm, .. } => ("its bottom_z_mm", *bottom_z_mm),
        TurnOperation::Drill { depth_z_mm, .. } => ("its depth_z_mm", *depth_z_mm),
        TurnOperation::Rough { .. } | TurnOperation::Finish { .. } => return Ok(()),
    };
    if z < job.chuck_z_mm {
        return Err(format!(
            "{}: {what} (z {z}) lies behind the chuck face (z {}), in the {} mm the jaws hold (the setup's grip): this chucking cannot machine there. Machine it with the part held on its other end, or correct chuck_z_mm",
            op.id(),
            job.chuck_z_mm,
            grip.length_mm
        ));
    }
    Ok(())
}

/// The stretch of z a roughing or finishing operation's nose centre works
/// in.
pub(super) struct Window {
    /// Its chuck-side end: the clearance in front of the chuck face, or
    /// `z_to_mm` where that is in front of it.
    pub lo: f64,
    /// Its front end (`z_from_mm`); None: from in front of the stock.
    pub hi: Option<f64>,
    /// Declared bounds or a grip: the path is cut exactly at the window's
    /// ends and the rest of the profile is reported as left. (Without
    /// either, an operation plans exactly as it did before bounds existed.)
    pub bounded: bool,
    /// The part's z ranges outside the window, ascending: left for another
    /// operation or chucking.
    pub left: Vec<[f64; 2]>,
}

pub(super) fn window(
    id: &str,
    job: &TurningJob,
    z_from: Option<f64>,
    z_to: Option<f64>,
) -> Result<Window, String> {
    let tol = job.tolerance_mm;
    let limit = job.chuck_z_mm + job.clearance_mm;
    let [z_lo, z_hi] = job.part.z_range();
    if [z_from, z_to].iter().flatten().any(|z| !z.is_finite()) {
        return Err(format!("{id}: z_from_mm and z_to_mm must be finite"));
    }
    if let Some(to) = z_to {
        if to < limit - tol {
            let held = if job.grip.is_some() {
                ", where the jaws hold the work (the setup's grip)"
            } else {
                ""
            };
            return Err(format!(
                "{id}: z_to_mm {to} lies behind the clearance in front of the chuck face (z {limit}: chuck_z_mm {} plus clearance_mm {}){held}: this chucking cannot turn there. End the operation at or in front of z {limit}, and turn the rest with the part held on its other end",
                job.chuck_z_mm, job.clearance_mm
            ));
        }
        if to >= z_hi {
            return Err(format!(
                "{id}: z_to_mm {to} must be behind the part's front face (z {z_hi}): nothing of the profile lies in front of it"
            ));
        }
    }
    let lo = z_to.map_or(limit, |z| z.max(limit));
    if let Some(from) = z_from
        && from <= lo.max(z_lo)
    {
        return Err(format!(
            "{id}: z_from_mm {from} must be in front of (greater than) where the operation ends (z {}: z_to_mm, the chuck clearance or the part's chuck-side end)",
            lo.max(z_lo)
        ));
    }
    let bounded = z_from.is_some() || z_to.is_some() || job.grip.is_some();
    let mut left = vec![];
    if bounded {
        if lo > z_lo + tol {
            left.push([z_lo, lo]);
        }
        if let Some(hi) = z_from
            && hi < z_hi - tol
        {
            left.push([hi, z_hi]);
        }
    }
    Ok(Window {
        lo,
        hi: z_from,
        bounded,
        left,
    })
}

/// The point of the segment p-q at u = `at` (which lies between their u).
fn cut(p: [f64; 2], q: [f64; 2], at: f64) -> [f64; 2] {
    [at, p[1] + (at - p[0]) / (q[0] - p[0]) * (q[1] - p[1])]
}

/// The pieces of a polyline (points (u, v) = (z, r)) with lo <= u <= hi,
/// cut exactly at the two ends.
pub(super) fn clip(points: &[[f64; 2]], lo: f64, hi: f64) -> Vec<Vec<[f64; 2]>> {
    let mut pieces: Vec<Vec<[f64; 2]>> = vec![];
    for w in points.windows(2) {
        let (p, q) = (w[0], w[1]);
        if p[0].max(q[0]) < lo || p[0].min(q[0]) > hi {
            continue;
        }
        let inside = |x: [f64; 2], other: [f64; 2]| {
            if x[0] > hi {
                cut(x, other, hi)
            } else if x[0] < lo {
                cut(x, other, lo)
            } else {
                x
            }
        };
        let (a, b) = (inside(p, q), inside(q, p));
        match pieces.last_mut() {
            Some(piece) if piece.last() == Some(&a) => piece.push(b),
            _ => pieces.push(vec![a, b]),
        }
    }
    pieces
}

/// One move of a finishing path: a line to a point, or an arc (about `c`
/// through `sweep`) from where the path is to `b`.
pub(super) enum Piece {
    To([f64; 2]),
    Arc {
        c: [f64; 2],
        sweep: f64,
        b: [f64; 2],
    },
}

/// The finishing path inside a bounded window: where it starts and the
/// moves that follow it from there, cut exactly at the window's ends.
///
/// `chords`: a segment's chain points (an arc's are the corners of its
/// tangents, so their hull holds the arc); `native`: whether an arc is cut
/// as one arc move. An arc wholly inside the window stays an arc; one the
/// window cuts is followed as its chords.
pub(super) fn follow(
    path: &[Seg],
    window: &Window,
    chords: impl Fn(&Seg) -> Vec<[f64; 2]>,
    native: impl Fn(&Seg) -> bool,
) -> Result<([f64; 2], Vec<Piece>), String> {
    let (lo, hi) = (window.lo, window.hi.unwrap_or(f64::INFINITY));
    let mut start: Option<[f64; 2]> = None;
    let mut moves = vec![];
    'path: for seg in path {
        let points = chords(seg);
        if let Seg::Arc { a, b, c, sweep } = *seg
            && native(seg)
            && points.iter().all(|p| p[0] >= lo && p[0] <= hi)
        {
            start.get_or_insert(a);
            moves.push(Piece::To(a));
            moves.push(Piece::Arc { c, sweep, b });
            continue;
        }
        for w in points.windows(2) {
            let (mut p, q) = (w[0], w[1]);
            if start.is_none() {
                // Not yet inside the front bound.
                if p[0] > hi {
                    if q[0] > hi {
                        continue;
                    }
                    p = cut(p, q, hi);
                }
                start = Some(p);
            } else if q[0] > hi + SLIVER {
                return Err(format!(
                    "the path turns back to z {} in front of z_from_mm {hi} after entering there: give a z_from_mm in front of that feature",
                    q[0]
                ));
            }
            if q[0] < lo {
                if p[0] > lo {
                    moves.push(Piece::To(cut(p, q, lo)));
                }
                break 'path;
            }
            moves.push(Piece::To(q));
        }
    }
    match start {
        Some(start) if start[0] >= lo && !moves.is_empty() => Ok((start, moves)),
        _ => Err(format!(
            "no part of the nose-centre path lies between z {lo} and z {hi} (z_to_mm or the chuck clearance, and z_from_mm)"
        )),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_polyline_is_clipped_exactly_at_both_ends() {
        // Down a diameter, up a shoulder, along the next diameter.
        let chain = [[0., 10.], [-20., 10.], [-20., 15.], [-40., 15.]];
        let pieces = clip(&chain, -30., -10.);
        assert_eq!(
            pieces,
            vec![vec![[-10., 10.], [-20., 10.], [-20., 15.], [-30., 15.]]]
        );
        // A taper is cut at the bound's own radius.
        let taper = [[0., 10.], [-10., 20.]];
        assert_eq!(
            clip(&taper, -5., f64::INFINITY),
            vec![vec![[0., 10.], [-5., 15.]]]
        );
        // Wholly outside: nothing; a path that leaves and returns: two pieces.
        assert!(clip(&chain, 5., 9.).is_empty());
        let back = [
            [0., 5.],
            [-10., 5.],
            [-10., 8.],
            [-4., 8.],
            [-4., 12.],
            [-12., 12.],
        ];
        assert_eq!(clip(&back, -6., 1.).len(), 2);
    }

    #[test]
    fn a_finishing_path_is_followed_between_its_bounds() {
        let line = |a: [f64; 2], b: [f64; 2]| Seg::Line { a, b };
        let path = [
            line([0.8, 0.], [0.8, 10.8]),
            line([0.8, 10.8], [-19.2, 10.8]),
            line([-19.2, 10.8], [-19.2, 15.8]),
            line([-19.2, 15.8], [-40., 15.8]),
        ];
        let chords = |s: &Seg| vec![s.start(), s.end()];
        let ends = |lo: f64, hi: Option<f64>| {
            let window = Window {
                lo,
                hi,
                bounded: true,
                left: vec![],
            };
            let (start, moves) = follow(&path, &window, chords, |_| false).unwrap();
            let last = match moves.last().unwrap() {
                Piece::To(p) => *p,
                Piece::Arc { b, .. } => *b,
            };
            (start, last, moves.len())
        };
        // To the bound, cut on the last diameter where it crosses.
        assert_eq!(ends(-30., None), ([0.8, 0.], [-30., 15.8], 4));
        // From a front bound on the first diameter.
        assert_eq!(ends(-30., Some(-5.)), ([-5., 10.8], [-30., 15.8], 3));
        // A window nothing of the path is in.
        let nothing = Window {
            lo: 5.,
            hi: Some(6.),
            bounded: true,
            left: vec![],
        };
        // (The path starts in front of it only by its face offset.)
        assert!(follow(&path[1..], &nothing, chords, |_| false).is_err());
    }
}
