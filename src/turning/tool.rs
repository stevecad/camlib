//! Turning tools: inserts and holders as outlines in the tool's XZ frame.
//!
//! Tool frame (x, z): x points away from the spindle axis for an outside
//! tool (toward it for a boring tool, by the mirror in `TurnSide`), z along
//! the spindle axis; drawn as an outside, right-hand tool (cutting toward
//! -z, toward the chuck). A left-hand tool is the mirror image in z.
//!
//! A turning insert is an [`Outline`] (lines and circular arcs) with its
//! cutting corner (an arc: its center is the nose center, its radius the
//! nose radius), the segments that cut, and the programming point. The
//! holder is an outline in the same frame, anywhere relative to the insert,
//! with an optional round boring bar. Nothing has a default: the ISO
//! constructors ([`Insert::iso_turning`], [`Insert::iso_code`],
//! [`Holder::rectangle`]) build outlines from values the caller states.
//! Grooving, face grooving, threading and drilling inserts keep their
//! parametric kinds (their operations plan from width, profile angle or
//! diameter); their holders are outlines too.
use super::stock::TurnSide;
use robust::{Coord, orient2d};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Hand {
    /// Cuts toward -z (toward the chuck): the usual turning direction.
    Right,
    /// Cuts toward +z.
    Left,
}

/// One piece of an outline, from the end of the previous one.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Segment {
    /// A straight line to `to_mm` (x, z).
    Line { to_mm: [f64; 2] },
    /// A circular arc around `center_mm` of `radius_mm` (0: a sharp corner)
    /// from the angle `start_deg` to `end_deg` (degrees from +x toward +z;
    /// increasing: counterclockwise in (x, z)), at most a full turn. It
    /// starts where the previous segment ends.
    Arc {
        center_mm: [f64; 2],
        radius_mm: f64,
        start_deg: f64,
        end_deg: f64,
    },
}

/// A closed outline: `start_mm`, then segments; the last ends at the start.
/// It must not cross itself.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Outline {
    pub start_mm: [f64; 2],
    pub segments: Vec<Segment>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Insert {
    /// A turning insert of any shape: its outline, the index of the arc
    /// segment that is the cutting corner (the nose), the indices of the
    /// segments that may cut (the corner and the edges next to it; every
    /// other segment must stay clear of the stock), and the programming
    /// point (the imaginary tool tip offsets are touched off with, x, z).
    Outline {
        outline: Outline,
        corner: usize,
        cutting: Vec<usize>,
        programming_point_mm: [f64; 2],
    },
    /// A grooving or parting insert of `width_mm`, `depth_mm` deep, with the
    /// nose radius at both corners. The reference point is its leading
    /// corner's nose center (the corner toward -z for a right-hand tool).
    Groove { width_mm: f64, depth_mm: f64 },
    /// A face grooving insert: a grooving blade pointing along -z into the
    /// front face, `width_mm` across in r, reaching `depth_mm` deep, with the
    /// nose radius at both corners. The reference point is its inner
    /// corner's nose center (the corner toward the axis).
    FaceGroove { width_mm: f64, depth_mm: f64 },
    /// A threading insert with `profile_angle_deg` (60 metric/UN), reaching
    /// `depth_mm` beyond its nose.
    Thread {
        profile_angle_deg: f64,
        depth_mm: f64,
    },
    /// A drill on the spindle axis (twist or insert drill): `diameter_mm`,
    /// `point_angle_deg` (118, 140; 180 for a flat bottom) and the length of
    /// its flutes. Its reference point is the tip on the axis.
    Drill {
        diameter_mm: f64,
        point_angle_deg: f64,
        flute_length_mm: f64,
    },
}

/// A round boring bar: `diameter_mm`, its axis at `axis_x_mm` in the tool
/// frame (parallel to z), from `z_mm[0]` to `z_mm[1]`. Its section through
/// its axis bounds its reach from the spindle axis on both sides.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Bar {
    pub diameter_mm: f64,
    pub axis_x_mm: f64,
    pub z_mm: [f64; 2],
}

/// The holder (or boring bar) in the tool frame of its insert.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Holder {
    pub outline: Outline,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bar: Option<Bar>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TurningTool {
    pub id: String,
    pub insert: Insert,
    /// The nose radius: the cutting corner's radius for an outline insert
    /// (they must agree), the corner radius of a grooving blade, the tip
    /// radius of a threading insert.
    pub nose_radius_mm: f64,
    pub side: TurnSide,
    pub hand: Hand,
    pub holder: Holder,
    /// The body off the centre plane (#103): required (validate refuses a
    /// tool without it).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub body: Option<super::body::ToolBody>,
}

/// A non-cutting side of an insert (its two ends) and its outward normal.
pub type BackSide = ([[f64; 2]; 2], [f64; 2]);

/// The tool's geometry in its frame, the nose center at the origin, for an
/// outside, right-hand tool; the caller mirrors z for a left-hand tool and
/// x for a boring tool.
#[derive(Clone, Debug)]
pub struct ToolShape {
    /// A polygon containing the insert (convex arcs circumscribed, concave
    /// arcs by their chords).
    pub insert: Vec<[f64; 2]>,
    /// A polygon containing the holder, and the bar's section if any.
    pub holder: Vec<[f64; 2]>,
    pub bar: Option<Vec<[f64; 2]>>,
    /// The insert's straight sides, exact (its outline is these and its
    /// arcs): what sweeps the material besides the nose.
    pub edges: Vec<[[f64; 2]; 2]>,
    /// Sides of the insert that must not meet the stock when they lead,
    /// each with its outward normal: an outline insert's non-cutting
    /// segments, a grooving blade's top.
    pub back: Option<Vec<BackSide>>,
    /// An outline insert's programming point (from the nose center).
    pub programming_point: Option<[f64; 2]>,
}

fn dir(angle_deg: f64) -> [f64; 2] {
    // Angle measured from +x toward +z.
    let a = angle_deg.to_radians();
    [a.cos(), a.sin()]
}
fn at(center: [f64; 2], radius: f64, angle_deg: f64) -> [f64; 2] {
    let d = dir(angle_deg);
    [center[0] + radius * d[0], center[1] + radius * d[1]]
}
fn finite(p: [f64; 2]) -> bool {
    p.iter().all(|v| v.is_finite() && v.abs() <= 1e4)
}
fn orient(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    orient2d(
        Coord { x: a[0], y: a[1] },
        Coord { x: b[0], y: b[1] },
        Coord { x: c[0], y: c[1] },
    )
}

/// Points of a polygon containing the arc from `from` to `to` degrees
/// around `center` (in steps of at most 2 degrees: within 0.00015 x the
/// radius of it): the tangent points at both ends and, between them, the
/// corners where the tangent lines at successive sample angles meet.
fn circumscribed(center: [f64; 2], radius: f64, from: f64, to: f64) -> Vec<[f64; 2]> {
    let n = (((to - from).abs() / 2.).ceil() as usize).max(1);
    let step = (to - from) / n as f64;
    let grow = radius / (step.abs().to_radians() / 2.).cos();
    let mut points = vec![at(center, radius, from)];
    points.extend((0..n).map(|k| at(center, grow, from + step * (k as f64 + 0.5))));
    points.push(at(center, radius, to));
    points
}

/// The polygon of an outline: vertices, and for each the segment its edge
/// to the next vertex belongs to; with the outline's orientation.
struct Polygon {
    points: Vec<[f64; 2]>,
    owners: Vec<usize>,
    ccw: bool,
}

impl Outline {
    fn ends(&self) -> Vec<([f64; 2], [f64; 2])> {
        let mut from = self.start_mm;
        self.segments
            .iter()
            .map(|s| {
                let start = match s {
                    Segment::Line { .. } => from,
                    Segment::Arc {
                        center_mm,
                        radius_mm,
                        start_deg,
                        ..
                    } => at(*center_mm, *radius_mm, *start_deg),
                };
                let end = match s {
                    Segment::Line { to_mm } => *to_mm,
                    Segment::Arc {
                        center_mm,
                        radius_mm,
                        end_deg,
                        ..
                    } => at(*center_mm, *radius_mm, *end_deg),
                };
                let pair = (from, start);
                from = end;
                let _ = pair;
                (start, end)
            })
            .collect()
    }

    /// Twice the signed area, arcs included (counterclockwise positive).
    fn signed_area2(&self) -> f64 {
        let mut area = 0.;
        let mut from = self.start_mm;
        for (s, (start, end)) in self.segments.iter().zip(self.ends()) {
            area += from[0] * start[1] - start[0] * from[1];
            area += start[0] * end[1] - end[0] * start[1];
            if let Segment::Arc {
                radius_mm,
                start_deg,
                end_deg,
                ..
            } = s
            {
                let t = (end_deg - start_deg).to_radians();
                area += radius_mm * radius_mm * (t - t.sin());
            }
            from = end;
        }
        area
    }

    /// Check the outline: finite, continuous within `gap` mm, closed, arcs
    /// of at most a full turn, enclosing area, not crossing itself.
    pub fn validate(&self, what: &str) -> Result<(), String> {
        if self.segments.is_empty() || self.segments.len() > 10_000 || !finite(self.start_mm) {
            return Err(format!(
                "{what}: an outline needs a finite start and 1..10000 segments"
            ));
        }
        let scale = 1e-9 * (1. + self.start_mm[0].abs().max(self.start_mm[1].abs()));
        let gap = 1e-6_f64.max(scale);
        let mut from = self.start_mm;
        for (i, (s, (start, end))) in self.segments.iter().zip(self.ends()).enumerate() {
            match s {
                Segment::Line { to_mm } => {
                    if !finite(*to_mm) {
                        return Err(format!("{what}: segment {i}: the line's end is not finite"));
                    }
                }
                Segment::Arc {
                    center_mm,
                    radius_mm,
                    start_deg,
                    end_deg,
                } => {
                    let sweep = end_deg - start_deg;
                    if !finite(*center_mm)
                        || !(radius_mm.is_finite() && (0. ..=1e4).contains(radius_mm))
                        || !start_deg.is_finite()
                        || !end_deg.is_finite()
                        || sweep == 0.
                        || sweep.abs() > 360.
                    {
                        return Err(format!(
                            "{what}: segment {i}: an arc needs a finite center, a radius 0..10000 mm and a sweep of up to a full turn"
                        ));
                    }
                    if (start[0] - from[0]).hypot(start[1] - from[1]) > gap {
                        return Err(format!(
                            "{what}: segment {i}: the arc starts at ({:.6}, {:.6}), not where the outline is ({:.6}, {:.6})",
                            start[0], start[1], from[0], from[1]
                        ));
                    }
                }
            }
            from = end;
        }
        if (from[0] - self.start_mm[0]).hypot(from[1] - self.start_mm[1]) > gap {
            return Err(format!(
                "{what}: the outline must end where it starts ({:.6}, {:.6}), not at ({:.6}, {:.6})",
                self.start_mm[0], self.start_mm[1], from[0], from[1]
            ));
        }
        if self.signed_area2().abs() <= 1e-12 {
            return Err(format!("{what}: the outline encloses no area"));
        }
        let polygon = self.polygon();
        let p = &polygon.points;
        let n = p.len();
        if n < 3 {
            return Err(format!("{what}: the outline encloses no area"));
        }
        // No two non-adjacent edges of the (containing) polygon meet.
        let meets = |a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]| {
            let (o1, o2) = (orient(a, b, c), orient(a, b, d));
            let (o3, o4) = (orient(c, d, a), orient(c, d, b));
            let within = |p: [f64; 2], q: [f64; 2], r: [f64; 2]| {
                r[0] >= p[0].min(q[0])
                    && r[0] <= p[0].max(q[0])
                    && r[1] >= p[1].min(q[1])
                    && r[1] <= p[1].max(q[1])
            };
            (o1 * o2 < 0. && o3 * o4 < 0.)
                || (o1 == 0. && within(a, b, c))
                || (o2 == 0. && within(a, b, d))
                || (o3 == 0. && within(c, d, a))
                || (o4 == 0. && within(c, d, b))
        };
        for i in 0..n {
            for j in i + 2..n {
                if i == 0 && j == n - 1 {
                    continue;
                }
                if meets(p[i], p[(i + 1) % n], p[j], p[(j + 1) % n]) {
                    return Err(format!(
                        "{what}: the outline crosses itself near ({:.4}, {:.4})",
                        p[j][0], p[j][1]
                    ));
                }
            }
        }
        Ok(())
    }

    /// A polygon containing the outline's region: lines exactly, arcs that
    /// bulge outward circumscribed, arcs that bulge inward by their chord.
    fn polygon(&self) -> Polygon {
        let ccw = self.signed_area2() > 0.;
        let mut points = vec![self.start_mm];
        let mut owners = vec![];
        // A point within rounding of the last (an arc starting where a line
        // ends, each computed its own way) is the same vertex.
        let mut push = |p: [f64; 2], owner: usize, points: &mut Vec<[f64; 2]>| {
            let last = *points.last().unwrap();
            let near =
                (p[0] - last[0]).hypot(p[1] - last[1]) <= 1e-12 * (1. + p[0].abs().max(p[1].abs()));
            if !near {
                owners.push(owner);
                points.push(p);
            }
        };
        for (k, s) in self.segments.iter().enumerate() {
            match s {
                Segment::Line { to_mm } => push(*to_mm, k, &mut points),
                Segment::Arc {
                    center_mm,
                    radius_mm,
                    start_deg,
                    end_deg,
                } => {
                    let outward = (end_deg > start_deg) == ccw;
                    if outward {
                        for p in circumscribed(*center_mm, *radius_mm, *start_deg, *end_deg) {
                            push(p, k, &mut points);
                        }
                    } else {
                        push(at(*center_mm, *radius_mm, *start_deg), k, &mut points);
                        push(at(*center_mm, *radius_mm, *end_deg), k, &mut points);
                    }
                }
            }
        }
        // Closed: the last vertex is the start again (or within the gap).
        if points.len() > 1 {
            points.pop();
        }
        let last = self.segments.len() - 1;
        owners.truncate(points.len().saturating_sub(1));
        owners.push(last);
        Polygon {
            points,
            owners,
            ccw,
        }
    }

    fn translated(&self, by: [f64; 2]) -> Outline {
        let t = |p: [f64; 2]| [p[0] + by[0], p[1] + by[1]];
        Outline {
            start_mm: t(self.start_mm),
            segments: self
                .segments
                .iter()
                .map(|s| match s {
                    Segment::Line { to_mm } => Segment::Line { to_mm: t(*to_mm) },
                    Segment::Arc {
                        center_mm,
                        radius_mm,
                        start_deg,
                        end_deg,
                    } => Segment::Arc {
                        center_mm: t(*center_mm),
                        radius_mm: *radius_mm,
                        start_deg: *start_deg,
                        end_deg: *end_deg,
                    },
                })
                .collect(),
        }
    }

    /// The least distance from `p` to the outline other than segment `skip`.
    fn distance_except(&self, p: [f64; 2], skip: usize) -> f64 {
        let mut best = f64::INFINITY;
        let mut from = self.start_mm;
        for (k, (s, (start, end))) in self.segments.iter().zip(self.ends()).enumerate() {
            if k != skip {
                let d = match s {
                    Segment::Line { .. } => point_segment(p, start, end),
                    Segment::Arc {
                        center_mm,
                        radius_mm,
                        start_deg,
                        end_deg,
                    } => {
                        let a = (p[1] - center_mm[1])
                            .atan2(p[0] - center_mm[0])
                            .to_degrees();
                        let (lo, hi) = (start_deg.min(*end_deg), start_deg.max(*end_deg));
                        let on = (0..3).any(|t| {
                            let a = a + 360. * (t as f64 - 1.);
                            (lo..=hi).contains(&a)
                        });
                        let radial =
                            ((p[0] - center_mm[0]).hypot(p[1] - center_mm[1]) - radius_mm).abs();
                        let ends = (p[0] - start[0])
                            .hypot(p[1] - start[1])
                            .min((p[0] - end[0]).hypot(p[1] - end[1]));
                        if on { radial.min(ends) } else { ends }
                    }
                };
                best = best.min(d);
                // The straight join into an arc that starts off the previous
                // end (within the continuity gap) is part of the boundary.
                best = best.min(point_segment(p, from, start));
            }
            from = end;
        }
        best
    }

    /// Whether `p` is inside the outline (by its containing polygon's
    /// winding, which agrees with the outline away from its arcs).
    fn contains(&self, p: [f64; 2]) -> bool {
        let poly = self.polygon();
        let n = poly.points.len();
        let mut winding = 0i32;
        for i in 0..n {
            let (a, b) = (poly.points[i], poly.points[(i + 1) % n]);
            if a[1] <= p[1] {
                if b[1] > p[1] && orient(a, b, p) > 0. {
                    winding += 1;
                }
            } else if b[1] <= p[1] && orient(a, b, p) < 0. {
                winding -= 1;
            }
        }
        winding != 0
    }
}

fn point_segment(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1]];
    let l2 = d[0] * d[0] + d[1] * d[1];
    let t = if l2 > 0. {
        (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / l2).clamp(0., 1.)
    } else {
        0.
    };
    (p[0] - a[0] - t * d[0]).hypot(p[1] - a[1] - t * d[1])
}

/// The ISO 1832 insert shapes the code constructor knows: shape letter,
/// corner angles in order around the insert (the first is the cutting
/// corner) and how the cutting edge length follows from the inscribed
/// circle diameter.
struct IsoShape {
    corners: &'static [f64],
    /// Side length per inscribed circle diameter.
    side_per_ic: f64,
}

impl Insert {
    /// An ISO-style turning insert as the wedge camlib has always modelled:
    /// a nose of `nose_radius_mm` between two edges of `edge_length_mm` at
    /// the `included_angle_deg` (80 C, 55 D, 35 V, 90 S, 60 T) on a holder
    /// with `approach_angle_deg` (the angle between the major cutting edge
    /// and the feed, e.g. 93 or 95), closed straight across the back. The
    /// programming point is the imaginary tip (one nose radius toward the
    /// axis and toward -z from the nose center). All three edges and the
    /// nose cut; the back must stay clear of the stock.
    pub fn iso_turning(
        included_angle_deg: f64,
        approach_angle_deg: f64,
        edge_length_mm: f64,
        nose_radius_mm: f64,
    ) -> Result<Insert, String> {
        let (e, k, l, rn) = (
            included_angle_deg,
            approach_angle_deg,
            edge_length_mm,
            nose_radius_mm,
        );
        if !(5. ..=120.).contains(&e)
            || !(30. ..=120.).contains(&k)
            || !(l.is_finite() && l > 0. && l <= 1e4)
            || !(rn.is_finite() && (0. ..=50.).contains(&rn))
        {
            return Err("turning insert: included angle 5..120, approach angle 30..120 degrees, positive edge length, nose radius 0..50 mm".into());
        }
        if k + e >= 180. {
            return Err("turning insert: approach plus included angle must leave a trailing clearance (< 180 degrees)".into());
        }
        if l <= rn {
            return Err("turning insert edge must be longer than the nose radius".into());
        }
        // Feed toward -z (angle -90). The major edge leaves the nose at the
        // approach angle from the feed direction; the minor (trailing) edge
        // the included angle further round. The nose faces the cut between
        // them.
        let major = k - 90.;
        let minor = major + e;
        // The far end of an edge leaving the nose along `angle`, tangent to
        // it on the `side` (+1: the tangent point at angle + 90), reaching
        // `length` along the edge.
        let end = |angle: f64, side: f64, length: f64| {
            let d = dir(angle);
            let t = dir(angle + side * 90.);
            [rn * t[0] + length * d[0], rn * t[1] + length * d[1]]
        };
        let (from, to) = (minor + 90., major + 270.);
        let first = end(minor, 1., l);
        Ok(Insert::Outline {
            outline: Outline {
                start_mm: first,
                segments: vec![
                    Segment::Line {
                        to_mm: at([0., 0.], rn, from),
                    },
                    Segment::Arc {
                        center_mm: [0., 0.],
                        radius_mm: rn,
                        start_deg: from,
                        end_deg: to,
                    },
                    Segment::Line {
                        to_mm: end(major, -1., l),
                    },
                    Segment::Line { to_mm: first },
                ],
            },
            corner: 1,
            cutting: vec![0, 1, 2],
            programming_point_mm: [-rn, -rn],
        })
    }

    /// An insert from its ISO 1832 designation (e.g. "CNMG120408") on a
    /// holder of `approach_angle_deg` (ISO 5608 tool style: e.g. L 95, J 93),
    /// for an outside, right-hand tool frame. The whole insert is outlined:
    /// every corner rounded with the code's nose radius, the cutting corner
    /// at the nose and the two edges next to it cutting; the rest must stay
    /// clear of the stock. Returns the insert and its nose radius.
    ///
    /// Shapes (letter 1): C 80, D 55, V 35 degree rhombic; S square; T
    /// triangle; W 80 degree hexagon (negative inserts); R round. Size
    /// (letters 5-6): the cutting edge length in whole mm, which names one
    /// inscribed circle diameter per shape (round: the diameter). Nose
    /// radius (letters 9-10): tenths of a mm (00 sharp, M0 round).
    /// Clearance, tolerance, type and thickness do not change the outline
    /// in the XZ plane and are not used.
    pub fn iso_code(code: &str, approach_angle_deg: f64) -> Result<(Insert, f64), String> {
        let code = code.trim().to_ascii_uppercase();
        let chars: Vec<char> = code.chars().collect();
        if chars.len() < 10 || !chars.iter().all(|c| c.is_ascii_alphanumeric()) {
            return Err(format!(
                "insert code {code:?}: give the ISO 1832 designation with shape, clearance, tolerance, type, size, thickness and nose radius (e.g. CNMG120408)"
            ));
        }
        let size: String = chars[4..6].iter().collect();
        let size: u32 = size
            .parse()
            .map_err(|_| format!("insert code {code:?}: letters 5-6 must be the size in mm"))?;
        let corner: String = chars[8..10].iter().collect();
        let letter = chars[0];
        if letter == 'R' {
            // Round: the size is the diameter; the whole edge cuts.
            let d = f64::from(size);
            if let Some(ic) = iso::round_diameter(size) {
                let r = ic / 2.;
                return Ok((
                    Insert::Outline {
                        outline: Outline {
                            start_mm: at([0., 0.], r, 0.),
                            segments: vec![Segment::Arc {
                                center_mm: [0., 0.],
                                radius_mm: r,
                                start_deg: 0.,
                                end_deg: 360.,
                            }],
                        },
                        corner: 0,
                        cutting: vec![0],
                        programming_point_mm: [-r, -r],
                    },
                    r,
                ));
            }
            return Err(format!(
                "insert code {code:?}: size {d} does not name one round insert diameter camlib knows ({}; 12 and 25 each name two); give an Outline insert instead",
                iso::ROUND
                    .iter()
                    .map(|(c, _)| c.to_string())
                    .collect::<Vec<_>>()
                    .join(", ")
            ));
        }
        let shape = iso::shape(letter).ok_or_else(|| {
            format!(
                "insert code {code:?}: shape {letter:?} is not one camlib outlines from its code (C, D, V, S, T, W, R); give an Outline insert instead"
            )
        })?;
        let rn = iso::nose_radius(&corner).ok_or_else(|| {
            format!(
                "insert code {code:?}: letters 9-10 must be a two-digit nose radius code (00, 01, 02, 04, 08, 10, 12, 16, 20, 24, 32)"
            )
        })?;
        if letter == 'W' && chars[1] != 'N' {
            return Err(format!(
                "insert code {code:?}: W size codes differ for positive inserts; camlib reads negative (N) W inserts only; give an Outline insert instead"
            ));
        }
        let ic = iso::inscribed_circle(letter, size).ok_or_else(|| {
            format!(
                "insert code {code:?}: size {size:02} is not an ISO 1832 size of shape {letter} camlib knows; give an Outline insert instead"
            )
        })?;
        let insert = Insert::polygon_insert(
            shape.corners,
            shape.side_per_ic * ic,
            rn,
            approach_angle_deg,
        )?;
        Ok((insert, rn))
    }

    /// A convex insert with equal sides of `side_mm` and the corner angles
    /// `corners` (counterclockwise from the cutting corner), each rounded
    /// with `nose_radius_mm`, its cutting corner at the nose and its major
    /// edge at `approach_angle_deg` from the feed (toward -z).
    fn polygon_insert(
        corners: &[f64],
        side_mm: f64,
        rn: f64,
        approach_angle_deg: f64,
    ) -> Result<Insert, String> {
        let e = corners[0];
        let k = approach_angle_deg;
        if !(30. ..=120.).contains(&k) || k + e >= 180. {
            return Err(format!(
                "insert: approach angle must be 30..120 degrees and leave a trailing clearance with the {e} degree corner (approach plus corner angle under 180)"
            ));
        }
        // Sharp corners, counterclockwise, the cutting corner at the origin:
        // the major edge leaves it at k - 90 degrees.
        let n = corners.len();
        let mut sharp = vec![[0., 0.]];
        let mut heading = k - 90.;
        for c in &corners[1..] {
            let p = *sharp.last().unwrap();
            let d = dir(heading);
            sharp.push([p[0] + side_mm * d[0], p[1] + side_mm * d[1]]);
            heading += 180. - c;
        }
        // Each corner: tangent points along its two edges, the arc between.
        let mut arcs = vec![];
        let mut incoming = k - 90. + e - 180.; // the edge into corner 0
        for (i, c) in corners.iter().enumerate() {
            let half = (c / 2.).to_radians();
            if rn / half.tan() >= side_mm / 2. {
                return Err("insert: the nose radius is too large for its edge length".into());
            }
            let outgoing = incoming + 180. - c;
            // Inward bisector: from the outgoing edge, half the corner angle
            // toward the incoming edge's reverse.
            let bisector = outgoing + c / 2.;
            let center = at(sharp[i], rn / half.sin(), bisector);
            arcs.push((center, incoming - 90., outgoing - 90.));
            incoming = outgoing;
        }
        // Start on the edge into the cutting corner at its far tangent
        // point (the end of the last corner's arc).
        let (c_last, _, end_last) = arcs[n - 1];
        let start = at(c_last, rn, end_last);
        let mut segments = vec![];
        for (center, a0, a1) in &arcs {
            segments.push(Segment::Line {
                to_mm: at(*center, rn, *a0),
            });
            segments.push(Segment::Arc {
                center_mm: *center,
                radius_mm: rn,
                start_deg: *a0,
                end_deg: *a1,
            });
        }
        // Nose center at the origin.
        let nose = arcs[0].0;
        let outline = Outline {
            start_mm: start,
            segments,
        }
        .translated([-nose[0], -nose[1]]);
        Ok(Insert::Outline {
            outline,
            corner: 1,
            cutting: vec![0, 1, 2],
            programming_point_mm: [-rn, -rn],
        })
    }
}

/// ISO 1832 and ISO 5608 code tables. Sources (see docs/operations.md):
/// shapes and corner angles, ISO 1832:1991 section 4.1 table 1; sizes and
/// corner radii, Sumitomo Electric Hardmetal general catalog, section B
/// "Indexable Insert Identification Code", tables 5 and 7 (p. B3), which
/// reproduce ISO 1832's designations; holder styles, ISO 5608:1995 section
/// 4.3 table 3.
mod iso {
    use super::IsoShape;
    /// Inscribed circle diameters (mm) of the metric size codes per shape
    /// (Sumitomo table 5; W: negative inserts only, whose codes differ from
    /// positive ones).
    const IC: &[(char, &[(u32, f64)])] = &[
        (
            'C',
            &[
                (6, 6.35),
                (8, 7.94),
                (9, 9.525),
                (12, 12.7),
                (16, 15.875),
                (19, 19.05),
                (25, 25.4),
            ],
        ),
        (
            'D',
            &[(7, 6.35), (9, 7.94), (11, 9.525), (15, 12.7), (19, 15.875)],
        ),
        (
            'V',
            &[(8, 4.76), (9, 5.56), (11, 6.35), (16, 9.525), (22, 12.7)],
        ),
        (
            'S',
            &[
                (6, 6.35),
                (7, 7.94),
                (9, 9.525),
                (12, 12.7),
                (15, 15.875),
                (19, 19.05),
                (25, 25.4),
                (31, 31.75),
            ],
        ),
        (
            'T',
            &[
                (6, 3.97),
                (8, 4.76),
                (9, 5.56),
                (11, 6.35),
                (13, 7.94),
                (16, 9.525),
                (22, 12.7),
                (27, 15.875),
                (33, 19.05),
            ],
        ),
        (
            'W',
            &[
                (3, 5.56),
                (4, 6.35),
                (5, 7.94),
                (6, 9.525),
                (8, 12.7),
                (10, 15.875),
            ],
        ),
    ];
    /// Round insert diameters (mm) whose size code names one diameter (12
    /// and 25 each name two: 12.0/12.70 and 25.0/25.40).
    pub const ROUND: &[(u32, f64)] = &[
        (8, 8.),
        (10, 10.),
        (15, 15.875),
        (16, 16.),
        (19, 19.05),
        (20, 20.),
        (24, 24.),
        (32, 32.),
    ];
    /// Sumitomo's tabulated cutting edge lengths (mm), for the tests.
    #[cfg(test)]
    pub const EDGE: &[(char, u32, f64)] = &[
        ('C', 12, 12.9),
        ('C', 9, 9.7),
        ('D', 15, 15.5),
        ('D', 11, 11.6),
        ('V', 16, 16.6),
        ('V', 22, 22.1),
        ('S', 12, 12.7),
        ('T', 16, 16.5),
        ('T', 22, 22.0),
        ('W', 8, 8.7),
        ('W', 6, 6.5),
    ];
    pub fn inscribed_circle(letter: char, size: u32) -> Option<f64> {
        IC.iter()
            .find(|(l, _)| *l == letter)?
            .1
            .iter()
            .find(|(s, _)| *s == size)
            .map(|(_, d)| *d)
    }
    pub fn round_diameter(size: u32) -> Option<f64> {
        ROUND.iter().find(|(s, _)| *s == size).map(|(_, d)| *d)
    }
    /// Corner radius codes (Sumitomo table 7), two-digit metric codes.
    pub fn nose_radius(code: &str) -> Option<f64> {
        Some(match code {
            "00" => 0.,
            "01" => 0.1,
            "02" => 0.2,
            "04" => 0.4,
            "08" => 0.8,
            "10" => 1.0,
            "12" => 1.2,
            "16" => 1.6,
            "20" => 2.0,
            "24" => 2.4,
            "32" => 3.2,
            _ => return None,
        })
    }
    /// Shapes with sizes in the table: corner angles (ISO 1832:1991 table 1)
    /// and the side length per inscribed circle diameter, from the geometry
    /// of each shape (it reproduces the tabulated edge lengths; see the
    /// tests). Rhombus of corner e: side = IC / sin e. Square: side = IC.
    /// Equilateral triangle: side = IC x sqrt 3. W (80 degree hexagon, six
    /// equal sides, corners 80 and 160): side = IC / 2 x (cot 40 + cot 80).
    pub fn shape(letter: char) -> Option<IsoShape> {
        let rhombus = |corners: &'static [f64]| IsoShape {
            corners,
            side_per_ic: 1. / corners[0].to_radians().sin(),
        };
        Some(match letter {
            'C' => rhombus(&[80., 100., 80., 100.]),
            'D' => rhombus(&[55., 125., 55., 125.]),
            'V' => rhombus(&[35., 145., 35., 145.]),
            'S' => IsoShape {
                corners: &[90., 90., 90., 90.],
                side_per_ic: 1.,
            },
            'T' => IsoShape {
                corners: &[60., 60., 60.],
                side_per_ic: 3f64.sqrt(),
            },
            'W' => IsoShape {
                corners: &[80., 160., 80., 160., 80., 160.],
                side_per_ic: 0.5 * (1. / 40f64.to_radians().tan() + 1. / 80f64.to_radians().tan()),
            },
            _ => return None,
        })
    }
    /// The cutting edge angle (kappa r) of an ISO 5608 tool style letter
    /// (ISO 5608:1995 table 3).
    pub fn holder_style(letter: char) -> Option<f64> {
        Some(match letter {
            'A' | 'C' | 'F' | 'G' => 90.,
            'B' | 'K' | 'R' => 75.,
            'D' | 'S' => 45.,
            'E' | 'T' | 'W' => 60.,
            'H' => 107.5,
            'J' | 'U' => 93.,
            'L' => 95.,
            'M' => 50.,
            'N' => 63.,
            'P' => 117.5,
            'V' => 72.5,
            'Y' => 85.,
            _ => return None,
        })
    }
}

/// The cutting edge angle (approach angle, kappa r) of an ISO 5608 holder
/// designation (e.g. "PCLNR2525M12": style L, 95 degrees), from its third
/// letter.
pub fn iso_holder_approach_deg(code: &str) -> Result<f64, String> {
    let code = code.trim().to_ascii_uppercase();
    code.chars()
        .nth(2)
        .and_then(iso::holder_style)
        .ok_or_else(|| {
            format!(
                "holder code {code:?}: its third letter must be an ISO 5608 tool style (A, B, C, D, E, F, G, H, J, K, L, M, N, P, R, S, T, U, V, W, Y)"
            )
        })
}

impl Holder {
    /// A rectangular holder as camlib has always modelled it, for `insert`
    /// with `nose_radius_mm` on `side`: outside tools a shank `width_mm`
    /// along z and `length_mm` away from the cut, starting `offset_mm`
    /// behind the insert's reach (turning: from the nose center toward +z;
    /// grooving and threading: across the blade); a boring tool (inner) an
    /// axial bar `width_mm` across, its top `offset_mm` under the nose
    /// center, `length_mm` out of the bore; a drill its chuck, `width_mm`
    /// across, `offset_mm` behind the flutes, `length_mm` long.
    pub fn rectangle(
        insert: &Insert,
        nose_radius_mm: f64,
        side: TurnSide,
        width_mm: f64,
        length_mm: f64,
        offset_mm: f64,
    ) -> Result<Holder, String> {
        let positive = |v: f64| v.is_finite() && v > 0. && v <= 1e4;
        if !positive(width_mm)
            || !positive(length_mm)
            || !(offset_mm.is_finite() && (0. ..=1e4).contains(&offset_mm))
        {
            return Err("holder: positive width and length, and an offset 0..10000 mm".into());
        }
        let rect = |x0: f64, x1: f64, z0: f64, z1: f64| Holder {
            outline: Outline {
                start_mm: [x0, z0],
                segments: vec![
                    Segment::Line { to_mm: [x1, z0] },
                    Segment::Line { to_mm: [x1, z1] },
                    Segment::Line { to_mm: [x0, z1] },
                    Segment::Line { to_mm: [x0, z0] },
                ],
            },
            bar: None,
        };
        if side == TurnSide::Inner && !matches!(insert, Insert::Drill { .. }) {
            return Ok(rect(offset_mm, offset_mm + width_mm, 0., length_mm));
        }
        let rn = nose_radius_mm;
        Ok(match insert {
            Insert::Outline { .. } => {
                let shape = insert_shape(insert, rn)?;
                let reach = shape.insert.iter().map(|p| p[0]).fold(0., f64::max);
                let r0 = reach + offset_mm;
                rect(r0, r0 + length_mm, 0., width_mm)
            }
            Insert::Groove {
                width_mm: w,
                depth_mm,
            }
            | Insert::FaceGroove {
                width_mm: w,
                depth_mm,
            } => {
                let r0 = depth_mm + offset_mm;
                rect(r0, r0 + length_mm, -rn, w - 2. * rn + rn)
            }
            Insert::Thread { depth_mm, .. } => {
                let r0 = depth_mm + offset_mm;
                rect(r0, r0 + length_mm, -width_mm / 2., width_mm / 2.)
            }
            Insert::Drill {
                flute_length_mm, ..
            } => {
                let z0 = flute_length_mm + offset_mm;
                rect(0., width_mm / 2., z0, z0 + length_mm)
            }
        })
    }
}

/// The insert's polygon, edges and back in the frame of `insert` with its
/// nose center at the origin.
struct InsertShape {
    insert: Vec<[f64; 2]>,
    edges: Vec<[[f64; 2]; 2]>,
    back: Option<Vec<BackSide>>,
    programming_point: Option<[f64; 2]>,
    /// Where the insert's own frame has its nose center (outline inserts).
    nose: [f64; 2],
}

fn outward(a: [f64; 2], b: [f64; 2], ccw: bool) -> [f64; 2] {
    let d = [b[0] - a[0], b[1] - a[1]];
    if ccw { [d[1], -d[0]] } else { [-d[1], d[0]] }
}

fn insert_shape(insert: &Insert, rn: f64) -> Result<InsertShape, String> {
    let end = |angle: f64, side: f64, length: f64| {
        let d = dir(angle);
        let t = dir(angle + side * 90.);
        [rn * t[0] + length * d[0], rn * t[1] + length * d[1]]
    };
    let arc = |from: f64, to: f64| circumscribed([0., 0.], rn, from, to);
    Ok(match insert {
        Insert::Outline {
            outline,
            corner,
            cutting,
            programming_point_mm,
        } => {
            let Some(Segment::Arc { center_mm, .. }) = outline.segments.get(*corner) else {
                return Err("outline insert: the corner must be an arc segment".into());
            };
            let nose = *center_mm;
            let local = outline.translated([-nose[0], -nose[1]]);
            let polygon = local.polygon();
            // Exact straight sides.
            let mut edges = vec![];
            let mut from = local.start_mm;
            for (s, (start, end)) in local.segments.iter().zip(local.ends()) {
                if let Segment::Line { to_mm } = s {
                    edges.push([from, *to_mm]);
                } else if (from[0] - start[0]).hypot(from[1] - start[1])
                    > 1e-12 * (1. + start[0].abs().max(start[1].abs()))
                {
                    // An arc starting off the previous end (within the
                    // outline's continuity gap): the straight join.
                    edges.push([from, start]);
                }
                from = end;
            }
            let n = polygon.points.len();
            let back = (0..n)
                .filter(|i| !cutting.contains(&polygon.owners[*i]))
                .map(|i| {
                    let (a, b) = (polygon.points[i], polygon.points[(i + 1) % n]);
                    ([a, b], outward(a, b, polygon.ccw))
                })
                .collect();
            InsertShape {
                insert: polygon.points,
                edges,
                back: Some(back),
                programming_point: Some([
                    programming_point_mm[0] - nose[0],
                    programming_point_mm[1] - nose[1],
                ]),
                nose,
            }
        }
        insert_kind @ (Insert::Groove { width_mm, depth_mm }
        | Insert::FaceGroove { width_mm, depth_mm }) => {
            // Nose centers at z = 0 (leading corner) and z = w; the blade
            // reaches depth_mm toward +r.
            let w = width_mm - 2. * rn;
            let mut insert = vec![[*depth_mm, w + rn], [*depth_mm, -rn]];
            insert.extend(arc(270., 180.));
            insert.extend(arc(180., 90.).into_iter().map(|p| [p[0], p[1] + w]));
            // Top, both sides and the flat between the corner noses.
            let edges = vec![
                [[*depth_mm, w + rn], [*depth_mm, -rn]],
                [[*depth_mm, -rn], [0., -rn]],
                [[-rn, 0.], [-rn, w]],
                [[0., w + rn], [*depth_mm, w + rn]],
            ];
            // A grooving blade's top, at its depth (outward: +x), must stay
            // clear of the stock; a face grooving blade is checked against
            // its declared groove instead.
            let top = [insert[0], insert[1]];
            InsertShape {
                edges,
                back: matches!(insert_kind, Insert::Groove { .. }).then(|| vec![(top, [1., 0.])]),
                insert,
                programming_point: None,
                nose: [0., 0.],
            }
        }
        Insert::Thread {
            profile_angle_deg,
            depth_mm,
        } => {
            // A V pointing toward the axis: flanks at +-half the profile
            // angle from +r.
            let half = profile_angle_deg / 2.;
            let length = (depth_mm + rn) / half.to_radians().cos();
            let mut insert = vec![end(half, 1., length)];
            insert.extend(arc(half + 90., 270. - half));
            insert.push(end(-half, -1., length));
            InsertShape {
                insert,
                edges: vec![],
                back: None,
                programming_point: None,
                nose: [0., 0.],
            }
        }
        Insert::Drill {
            diameter_mm,
            point_angle_deg,
            flute_length_mm,
        } => {
            // The half on the +r side of the axis (the drill is symmetric):
            // tip on the axis, the point's cone to the full diameter, the
            // flutes.
            let r = diameter_mm / 2.;
            let cone = r / (point_angle_deg / 2.).to_radians().tan();
            InsertShape {
                insert: vec![
                    [0., 0.],
                    [r, cone],
                    [r, *flute_length_mm],
                    [0., *flute_length_mm],
                ],
                edges: vec![],
                back: None,
                programming_point: None,
                nose: [0., 0.],
            }
        }
    })
}

impl TurningTool {
    pub fn validate(&self) -> Result<(), String> {
        let positive = |v: f64| v.is_finite() && v > 0. && v <= 1e4;
        if !crate::tool::valid_id(&self.id)
            || !(self.nose_radius_mm.is_finite()
                && self.nose_radius_mm >= 0.
                && self.nose_radius_mm <= 50.)
        {
            return Err(format!(
                "turning tool {:?}: needs an id and a nose radius 0..50 mm",
                self.id
            ));
        }
        let id = &self.id;
        match &self.insert {
            Insert::Outline {
                outline,
                corner,
                cutting,
                programming_point_mm,
            } => {
                outline.validate(&format!("tool {id}: insert"))?;
                let Some(Segment::Arc {
                    center_mm,
                    radius_mm,
                    start_deg,
                    end_deg,
                }) = outline.segments.get(*corner)
                else {
                    return Err(format!(
                        "tool {id}: the insert's corner must be the index of an arc segment (its nose)"
                    ));
                };
                if (radius_mm - self.nose_radius_mm).abs() > 1e-9 * (1. + radius_mm) {
                    return Err(format!(
                        "tool {id}: the nose radius {} mm differs from the insert's corner radius {radius_mm} mm",
                        self.nose_radius_mm
                    ));
                }
                if (end_deg > start_deg) != (outline.signed_area2() > 0.) {
                    return Err(format!(
                        "tool {id}: the insert's corner must bulge outward (a convex arc)"
                    ));
                }
                if cutting.is_empty()
                    || !cutting.contains(corner)
                    || cutting.iter().any(|c| *c >= outline.segments.len())
                {
                    return Err(format!(
                        "tool {id}: the insert's cutting segments must be indices of its outline and include the corner"
                    ));
                }
                if !finite(*programming_point_mm) {
                    return Err(format!(
                        "tool {id}: the insert's programming point must be finite"
                    ));
                }
                // The nose disc lies in the insert: its sweep is material the
                // insert certainly removes.
                if *radius_mm > 0. {
                    if !outline.contains(*center_mm) {
                        return Err(format!(
                            "tool {id}: the insert's corner center lies outside its outline"
                        ));
                    }
                    let room = outline.distance_except(*center_mm, *corner);
                    if room < radius_mm - 1e-9 * (1. + radius_mm) {
                        return Err(format!(
                            "tool {id}: the insert's nose circle (radius {radius_mm} mm) leaves its outline ({room:.6} mm from the nose center)"
                        ));
                    }
                }
            }
            Insert::Groove { width_mm, depth_mm } | Insert::FaceGroove { width_mm, depth_mm } => {
                if !positive(*width_mm)
                    || !positive(*depth_mm)
                    || *width_mm < 2. * self.nose_radius_mm
                {
                    return Err(format!(
                        "tool {id}: a grooving insert needs a positive width (at least two nose radii) and depth"
                    ));
                }
            }
            Insert::Thread {
                profile_angle_deg,
                depth_mm,
            } => {
                if !(20. ..=90.).contains(profile_angle_deg) || !positive(*depth_mm) {
                    return Err(format!(
                        "tool {id}: a threading insert needs a profile angle 20..90 degrees and a positive depth"
                    ));
                }
            }
            Insert::Drill {
                diameter_mm,
                point_angle_deg,
                flute_length_mm,
            } => {
                if !positive(*diameter_mm)
                    || !(60. ..=180.).contains(point_angle_deg)
                    || !(flute_length_mm.is_finite() && *flute_length_mm > diameter_mm / 2.)
                {
                    return Err(format!(
                        "tool {id}: a drill needs a positive diameter, a point angle 60..180 degrees and flutes longer than the point"
                    ));
                }
            }
        }
        self.holder
            .outline
            .validate(&format!("tool {id}: holder"))?;
        // The body off the centre plane (#103): every tool declares it.
        self.body
            .as_ref()
            .ok_or_else(|| super::body::missing_body(id))?
            .validate(id, matches!(self.insert, Insert::Drill { .. }))?;
        if !matches!(
            self.insert,
            Insert::Groove { .. } | Insert::FaceGroove { .. }
        ) && self
            .body
            .as_ref()
            .and_then(|b| b.insert.as_ref())
            .is_some_and(|b| b.blade_clearance.is_some())
        {
            return Err(super::body::blade_only(id));
        }
        if let Some(bar) = &self.holder.bar
            && (!positive(bar.diameter_mm)
                || !(bar.axis_x_mm.is_finite() && bar.axis_x_mm.abs() <= 1e4)
                || !finite(bar.z_mm)
                || bar.z_mm[1] <= bar.z_mm[0])
        {
            return Err(format!(
                "tool {id}: a boring bar needs a positive diameter, a finite axis and z_mm from low to high"
            ));
        }
        // A solid boring bar's ground head (#114): its declaration against
        // its outline, and its minimum bore against its geometry.
        if let Some(head) = self.body.as_ref().and_then(|b| b.solid_bar.as_ref()) {
            super::body::solid_bar_vertices(self, head)?;
        }
        Ok(())
    }

    /// The tool's shape in its frame (nose center at the origin), for an
    /// outside, right-hand tool; the caller mirrors z for a left-hand tool
    /// and x for a boring tool.
    pub fn shape(&self) -> Result<ToolShape, String> {
        self.validate()?;
        let s = insert_shape(&self.insert, self.nose_radius_mm)?;
        let nose = s.nose;
        let holder = self
            .holder
            .outline
            .translated([-nose[0], -nose[1]])
            .polygon()
            .points;
        let bar = self.holder.bar.as_ref().map(|b| {
            let (x0, x1) = (
                b.axis_x_mm - b.diameter_mm / 2. - nose[0],
                b.axis_x_mm + b.diameter_mm / 2. - nose[0],
            );
            let (z0, z1) = (b.z_mm[0] - nose[1], b.z_mm[1] - nose[1]);
            vec![[x0, z0], [x1, z0], [x1, z1], [x0, z1]]
        });
        Ok(ToolShape {
            insert: s.insert,
            holder,
            bar,
            edges: s.edges,
            back: s.back,
            programming_point: s.programming_point,
        })
    }
}

/// A solid boring bar head's outline at its cutting face (#114), the nose
/// center at the origin: the containing polygon (as `ToolShape::insert`),
/// for each of its sides the outline segment it belongs to, the outline's
/// exact extent in x (lines by their ends, arcs by their ends and their
/// extreme points) and its largest arc radius.
pub(crate) struct HeadOutline {
    pub points: Vec<[f64; 2]>,
    pub owners: Vec<usize>,
    pub segments: usize,
    pub x_mm: [f64; 2],
    pub arc_radius_mm: f64,
    /// The nose center in the insert's own frame.
    pub nose: [f64; 2],
}

/// The head outline of an `Insert::Outline` (None for the other kinds, or
/// when its corner is not an arc). The caller validates the tool.
pub(crate) fn head_outline(insert: &Insert) -> Option<HeadOutline> {
    let Insert::Outline {
        outline, corner, ..
    } = insert
    else {
        return None;
    };
    let Some(Segment::Arc { center_mm, .. }) = outline.segments.get(*corner) else {
        return None;
    };
    let nose = *center_mm;
    let local = outline.translated([-nose[0], -nose[1]]);
    let polygon = local.polygon();
    let (mut lo, mut hi, mut arc) = (f64::INFINITY, f64::NEG_INFINITY, 0f64);
    let mut reach = |x: f64| {
        lo = lo.min(x);
        hi = hi.max(x);
    };
    reach(local.start_mm[0]);
    for (s, (start, end)) in local.segments.iter().zip(local.ends()) {
        reach(start[0]);
        reach(end[0]);
        if let Segment::Arc {
            center_mm,
            radius_mm,
            start_deg,
            end_deg,
        } = s
        {
            arc = arc.max(*radius_mm);
            // Its extremes in x: at 0 and 180 degrees (any turn), where the
            // sweep passes them.
            let (a, b) = (start_deg.min(*end_deg), start_deg.max(*end_deg));
            for (base, sign) in [(0., 1.), (180., -1.)] {
                let first = ((a - base) / 360.).ceil();
                if base + 360. * first <= b {
                    reach(center_mm[0] + sign * radius_mm);
                }
            }
        }
    }
    Some(HeadOutline {
        points: polygon.points,
        owners: polygon.owners,
        segments: outline.segments.len(),
        x_mm: [lo, hi],
        arc_radius_mm: arc,
        nose,
    })
}

/// Convex pieces whose union is the simple polygon `p` (itself, if convex;
/// otherwise its triangles, by ear clipping with exact orientation tests).
pub(crate) fn convex_pieces(p: &[[f64; 2]]) -> Vec<Vec<[f64; 2]>> {
    let n = p.len();
    if n < 4 {
        return vec![p.to_vec()];
    }
    // A turn within rounding of straight (a tangent edge running into a
    // circumscribed arc) counts as straight: the polygon is then convex up
    // to that rounding, and its hull, which contains it, is its piece.
    let turns: Vec<f64> = (0..n)
        .map(|i| {
            let (a, b, c) = (p[i], p[(i + 1) % n], p[(i + 2) % n]);
            let o = orient(a, b, c);
            let scale = (b[0] - a[0]).hypot(b[1] - a[1]) * (c[0] - b[0]).hypot(c[1] - b[1]);
            if o.abs() <= 1e-9 * scale { 0. } else { o }
        })
        .collect();
    if turns.iter().all(|t| *t >= 0.) || turns.iter().all(|t| *t <= 0.) {
        return vec![p.to_vec()];
    }
    let area: f64 = (0..n)
        .map(|i| p[i][0] * p[(i + 1) % n][1] - p[(i + 1) % n][0] * p[i][1])
        .sum();
    let sign = area.signum();
    let mut index: Vec<usize> = (0..n).collect();
    let mut pieces = vec![];
    let mut guard = 0;
    while index.len() > 3 && guard < 4 * n * n {
        guard += 1;
        let m = index.len();
        let mut clipped = false;
        for i in 0..m {
            let (a, b, c) = (
                p[index[(i + m - 1) % m]],
                p[index[i]],
                p[index[(i + 1) % m]],
            );
            if orient(a, b, c) * sign <= 0. {
                continue;
            }
            let ear = index.iter().all(|&j| {
                let q = p[j];
                q == a
                    || q == b
                    || q == c
                    || !(orient(a, b, q) * sign >= 0.
                        && orient(b, c, q) * sign >= 0.
                        && orient(c, a, q) * sign >= 0.)
            });
            if ear {
                pieces.push(vec![a, b, c]);
                index.remove(i);
                clipped = true;
                break;
            }
        }
        if !clipped {
            // Collinear leftovers (no positive ear): drop a straight vertex.
            if let Some(i) = (0..m).find(|&i| {
                orient(
                    p[index[(i + m - 1) % m]],
                    p[index[i]],
                    p[index[(i + 1) % m]],
                ) == 0.
            }) {
                index.remove(i);
            } else {
                break;
            }
        }
    }
    if index.len() == 3 {
        pieces.push(index.iter().map(|&i| p[i]).collect());
    }
    pieces
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tool(insert: Insert, rn: f64) -> TurningTool {
        let holder = Holder::rectangle(&insert, rn, TurnSide::Outer, 20., 100., 0.).unwrap();
        TurningTool {
            id: "t".into(),
            insert,
            nose_radius_mm: rn,
            side: TurnSide::Outer,
            hand: Hand::Right,
            holder,
            body: Some(super::super::body::ToolBody {
                insert: Some(super::super::body::InsertBody {
                    insert_thickness_mm: 3.,
                    insert_clearance_deg: 7.,
                    rake_deg: 0.,
                    inclination_deg: 0.,
                    edge_deg: 5.,
                    curved_for_diameter_mm: None,
                    blade_clearance: None,
                }),
                holder_y_mm: [-10., 2.],
                bar_axis_y_mm: 0.,
                solid_bar: None,
            }),
        }
    }
    fn convex(p: &[[f64; 2]]) -> bool {
        convex_pieces(p).len() == 1
    }

    #[test]
    fn inserts_are_convex_and_contain_their_nose() {
        for insert in [
            Insert::iso_turning(80., 95., 12., 0.4).unwrap(),
            Insert::iso_turning(35., 93., 16., 0.4).unwrap(),
            Insert::Groove {
                width_mm: 3.,
                depth_mm: 15.,
            },
            Insert::Thread {
                profile_angle_deg: 60.,
                depth_mm: 3.,
            },
        ] {
            let s = tool(insert.clone(), 0.4).shape().unwrap();
            assert!(convex(&s.insert), "{insert:?} {:?}", s.insert);
            // The nose point toward the axis (-r) is on or inside the polygon.
            let lowest = s.insert.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min);
            assert!(lowest <= -0.4 + 1e-9, "{insert:?}");
            assert!(s.holder.iter().all(|p| p[0] >= 0.));
        }
    }

    #[test]
    fn iso_codes_give_the_whole_insert() {
        // CNMG120408: 80 degree rhombus, IC 12.7, nose 0.8.
        let (insert, rn) = Insert::iso_code("CNMG120408", 95.).unwrap();
        assert_eq!(rn, 0.8);
        let t = tool(insert, rn);
        t.validate().unwrap();
        let s = t.shape().unwrap();
        assert!(convex(&s.insert));
        // Four sides of 12.7 / sin 80 = 12.896 mm, less the rounded corners.
        let side = 12.7 / 80f64.to_radians().sin();
        assert!(s.edges.iter().all(|e| {
            let l = (e[1][0] - e[0][0]).hypot(e[1][1] - e[0][1]);
            l < side && l > side - 2.
        }));
        assert_eq!(s.edges.len(), 4);
        // The back corners stay clear of the cut: 5 non-cutting pieces at
        // least (three corners and two sides).
        assert!(s.back.as_ref().unwrap().len() > 5);
        // Round and trigon.
        let (round, r) = Insert::iso_code("RCMT10T3M0", 95.).unwrap();
        assert_eq!(r, 5.);
        tool(round, r).validate().unwrap();
        let (trigon, r) = Insert::iso_code("WNMG080408", 95.).unwrap();
        let s = tool(trigon, r).shape().unwrap();
        assert_eq!(s.edges.len(), 6);
        // A shape it does not outline is refused with what to do instead.
        let e = Insert::iso_code("KNUX160405", 95.).unwrap_err();
        assert!(e.contains("give an Outline insert"), "{e}");
        let e = Insert::iso_code("RCMT12T3M0", 95.).unwrap_err();
        assert!(e.contains("each name two"), "{e}");
        assert!(e.contains("give an Outline insert"), "{e}");
    }

    #[test]
    fn iso_sizes_reproduce_the_tabulated_edge_lengths() {
        for (letter, size, edge) in iso::EDGE {
            let ic = iso::inscribed_circle(*letter, *size).unwrap();
            let side = iso::shape(*letter).unwrap().side_per_ic * ic;
            assert!(
                (side - edge).abs() < 0.05,
                "{letter}{size}: {side} vs {edge}"
            );
        }
        assert_eq!(iso_holder_approach_deg("PCLNR2525M12").unwrap(), 95.);
        assert_eq!(iso_holder_approach_deg("PDJNR2525M15").unwrap(), 93.);
        assert!(iso_holder_approach_deg("PCXNR").is_err());
    }

    #[test]
    fn outlines_are_checked() {
        let (insert, rn) = Insert::iso_code("CNMG120408", 95.).unwrap();
        let mut t = tool(insert, rn);
        t.nose_radius_mm = 0.4;
        assert!(
            t.validate()
                .unwrap_err()
                .contains("differs from the insert's corner radius")
        );
        // Open, crossing and empty outlines.
        let bad = |segments: Vec<Segment>| Outline {
            start_mm: [0., 0.],
            segments,
        };
        let open = bad(vec![
            Segment::Line { to_mm: [1., 0.] },
            Segment::Line { to_mm: [1., 1.] },
        ]);
        assert!(
            open.validate("x")
                .unwrap_err()
                .contains("must end where it starts")
        );
        let crossing = bad(vec![
            Segment::Line { to_mm: [4., 0.] },
            Segment::Line { to_mm: [4., 3.] },
            Segment::Line { to_mm: [1., -1.] },
            Segment::Line { to_mm: [0., 3.] },
            Segment::Line { to_mm: [0., 0.] },
        ]);
        assert!(
            crossing
                .validate("x")
                .unwrap_err()
                .contains("crosses itself")
        );
        let flat = bad(vec![
            Segment::Line { to_mm: [1., 0.] },
            Segment::Line { to_mm: [0., 0.] },
        ]);
        assert!(flat.validate("x").unwrap_err().contains("no area"));
    }

    #[test]
    fn non_convex_polygons_split_into_convex_pieces() {
        // An L: three squares.
        let l = [[0., 0.], [2., 0.], [2., 1.], [1., 1.], [1., 2.], [0., 2.]];
        let pieces = convex_pieces(&l);
        let area: f64 = pieces
            .iter()
            .map(|t| {
                let n = t.len();
                (0..n)
                    .map(|i| t[i][0] * t[(i + 1) % n][1] - t[(i + 1) % n][0] * t[i][1])
                    .sum::<f64>()
                    .abs()
                    / 2.
            })
            .sum();
        assert!((area - 3.).abs() < 1e-12);
        assert!(pieces.iter().all(|p| convex(p)));
    }
}
