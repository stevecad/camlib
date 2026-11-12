//! Common-line cutting: two parts whose facing straight edges are one kerf
//! apart share a single cut along that edge.
//!
//! Practice followed:
//! - A common line is one cut that separates two parts placed a kerf apart
//!   ("a Kerf value of 1 must have a margin between parts set to at least 1";
//!   "works best along straight edges": Autodesk Fabrication CAMduct, "Common
//!   Line Cutting"; Hypertherm ProNest, "Common Line Cutting": "two or more
//!   parts share the cut path").
//! - The common cut carries no kerf side: the software offsets the parts
//!   from it and the controller cannot trim it (Kinetic PrimeCut, "Common
//!   Line and Common Crop Cutting": "The common line cuts are created as
//!   kerf none ... The outside cuts are kept kerf left as normal"). So wear
//!   compensation is refused, and one of the two parts is on the poorer side
//!   of a plasma arc ("it is impossible to preserve the kerf side").
//! - Inner cuts come first: "Holes in the parts are always cut before the
//!   common lines" (PrimeCut), and the outline of the group last, as for any
//!   part: every part stays held by the sheet until its last edge.
//! - The pierce stays away from both parts, as every pierce does (the
//!   operation's pierce clearance): it lies in the scrap on the extension of
//!   the line.
//!
//! Here "a kerf apart" is exact: the kerf-compensated paths of the two
//! parts (the beam center `applied_offset_mm` outside each) coincide along
//! the whole shared edge, to the operation's geometric `tolerance_mm`. That
//! is a spacing of twice the applied offset: the kerf width plus the
//! boundary allowance every thermal cut leaves on a part. The single cut is
//! therefore the ordinary compensated path of both parts, and the ordinary
//! kerf proof (against every part) covers both sides of it.
//!
//! Supported: a shared edge that is a complete straight edge of both parts,
//! ending at outside corners where the neighbouring edges of the two parts
//! are in line (parts flush at both ends: pairs and rows). The outline of
//! the group then runs straight across each end of the common line, as one
//! move (no vertex there: nothing needs one).
//! Anything else whose kerf paths coincide is refused, never cut twice.
//!
//! What is proven, compared with a job without common lines:
//! - Ordinary job: the region is offset as a whole, which refuses any two
//!   parts whose kerf paths touch or cross; every lead and kerf path is
//!   proven clear of every part, every pierce to keep its clearance.
//! - Job with common lines: parts are offset one by one, and the same is
//!   proven by exact stroke-to-stroke distances (`separate`): outside the
//!   strokes each declared common line replaces, no two kerf paths of
//!   different parts touch or cross, and no kerf path runs inside another
//!   part's applied offset. The proofs against the parts are the same ones,
//!   run on the moves actually cut. In addition (`check_kerfs`), because
//!   kerfs of one group meet: every pierce lies in uncut sheet and no lead
//!   runs within a kerf width of a kerf cut before it. The only place a
//!   beam meets an earlier kerf is the group's outline (a cut, not a lead)
//!   passing the ends of its common lines: it closes each common line at
//!   the far end and crosses its lead-in kerf at the near end, which is
//!   what joins the cuts (PrimeCut advises against cuts crossing gaps for
//!   flame cutting only).
use super::Step;
use crate::{
    Point,
    contour::Boundaries,
    motion::{Curve, Event, MoveKind, Vec3},
    planar::{Component, ConstructionOptions, PlanarRegion},
    pocket::Stroke,
    profiles::{Profiles, offset_profiles},
    stock::CancelSignal,
    tool::PreparedAssembly,
};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};

/// Opt-in common-line cutting.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CommonLine {
    /// The shortest shared edge to cut as a common line. A shorter one is
    /// refused (the parts cannot be cut separately at that spacing).
    pub min_length_mm: f64,
}

/// One common line in the report.
#[derive(Clone, Debug, Serialize)]
pub struct CommonLineReport {
    /// The two parts it separates.
    pub parts: [usize; 2],
    /// The beam-center cut, in cutting order, between the group's outline
    /// at either end.
    pub start_mm: Point,
    pub end_mm: Point,
    /// The length of the shared edge.
    pub shared_length_mm: f64,
}

/// A loop as planned: (part, hole, strokes in cutting direction).
pub(super) type Ring = (usize, Option<usize>, Vec<Stroke>);

pub(super) struct Cut {
    pub parts: [usize; 2],
    /// Between the group's outline at either end.
    pub ends: [Point; 2],
    pub shared_length: f64,
}

#[derive(Default)]
pub(super) struct Plan {
    /// The parts of each group (a set of parts joined by common lines).
    groups: Vec<Vec<usize>>,
    /// (group, cut)
    cuts: Vec<(usize, Cut)>,
    pub diagnostics: Vec<String>,
}

fn sub(a: Point, b: Point) -> Point {
    [a[0] - b[0], a[1] - b[1]]
}
fn dot(a: Point, b: Point) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
fn cross(a: Point, b: Point) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}
fn dist(a: Point, b: Point) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}
fn unit(s: &Stroke) -> Option<(Point, f64)> {
    let v = sub(s.b, s.a);
    let n = v[0].hypot(v[1]);
    (n > 0.).then(|| ([v[0] / n, v[1] / n], n))
}

/// Each part offset on its own: parts a kerf apart have kerf paths that
/// touch, which the offset of the whole region cannot represent.
pub(super) fn part_profiles(
    definition: &PlanarRegion,
    offset: f64,
    options: ConstructionOptions,
    cancel: &impl CancelSignal,
) -> Result<Profiles, String> {
    let mut all = Profiles {
        loops: vec![],
        edges: 0,
        arcs: 0,
        diagnostics: vec![],
    };
    for (i, c) in definition.components.iter().enumerate() {
        let one = PlanarRegion {
            version: 1,
            units: "mm".into(),
            source: serde_json::Value::Null,
            name: definition.name.clone(),
            components: vec![Component {
                outer: c.outer.clone(),
                holes: c.holes.clone(),
            }],
            boundary_error_mm: definition.boundary_error_mm,
        }
        .prepare(options)?;
        let p = offset_profiles(&one, offset, Boundaries::All, false, options, cancel)
            .map_err(|e| format!("part {i}: {e}"))?;
        all.edges += p.edges;
        all.arcs += p.arcs;
        all.diagnostics.extend(p.diagnostics);
        all.loops.extend(p.loops.into_iter().map(|mut l| {
            l.component = i;
            l
        }));
    }
    Ok(all)
}

/// Two strokes of different parts' kerf paths that run along each other.
struct Hit {
    loops: [usize; 2],
    strokes: [usize; 2],
    straight: bool,
    /// The two strokes have the same ends.
    full: bool,
    opposite: bool,
    length: f64,
}

fn bounds(strokes: &[Stroke]) -> [f64; 4] {
    let mut b = [f64::INFINITY, f64::INFINITY, -f64::INFINITY, -f64::INFINITY];
    for s in strokes {
        let r = s.arc.map_or(0., |(c, _)| dist(s.a, c));
        for p in [s.a, s.b, s.arc.map_or(s.a, |(c, _)| c)] {
            b[0] = b[0].min(p[0] - r);
            b[1] = b[1].min(p[1] - r);
            b[2] = b[2].max(p[0] + r);
            b[3] = b[3].max(p[1] + r);
        }
    }
    b
}

/// The fraction of an arc stroke at which `p` (on its circle) lies, if it
/// lies on the arc.
fn on_arc(s: &Stroke, p: Point, tol: f64) -> Option<f64> {
    let (c, sweep) = s.arc?;
    let r = dist(s.a, c);
    if (dist(p, c) - r).abs() > tol || r <= tol {
        return None;
    }
    let a0 = (s.a[1] - c[1]).atan2(s.a[0] - c[0]);
    let a = (p[1] - c[1]).atan2(p[0] - c[0]);
    let turn = std::f64::consts::TAU;
    let mut d = (a - a0) * sweep.signum();
    d -= (d / turn).floor() * turn;
    let slack = tol / r;
    if d > turn - slack {
        d = 0.;
    }
    (d <= sweep.abs() + slack).then(|| (d / sweep.abs()).min(1.))
}

/// Every pair of strokes, of loops of different parts, that coincide over
/// more than the tolerance.
fn hits(rings: &[Ring], tol: f64) -> Vec<Hit> {
    let boxes: Vec<[f64; 4]> = rings.iter().map(|r| bounds(&r.2)).collect();
    let mut found = vec![];
    for la in 0..rings.len() {
        for lb in la + 1..rings.len() {
            let (a, b) = (boxes[la], boxes[lb]);
            if rings[la].0 == rings[lb].0
                || a[0] > b[2] + tol
                || b[0] > a[2] + tol
                || a[1] > b[3] + tol
                || b[1] > a[3] + tol
            {
                continue;
            }
            for (i, p) in rings[la].2.iter().enumerate() {
                for (j, q) in rings[lb].2.iter().enumerate() {
                    let hit = match (p.arc, q.arc) {
                        (None, None) => {
                            let (Some((u, length)), Some(_)) = (unit(p), unit(q)) else {
                                continue;
                            };
                            let (da, db) = (sub(q.a, p.a), sub(q.b, p.a));
                            if cross(u, da).abs() > tol || cross(u, db).abs() > tol {
                                continue;
                            }
                            let (ta, tb) = (dot(u, da), dot(u, db));
                            let over = ta.max(tb).min(length) - ta.min(tb).max(0.);
                            if over <= tol {
                                continue;
                            }
                            let opposite = tb < ta;
                            let (first, last) = if opposite { (q.b, q.a) } else { (q.a, q.b) };
                            Hit {
                                loops: [la, lb],
                                strokes: [i, j],
                                straight: true,
                                full: dist(first, p.a) <= tol && dist(last, p.b) <= tol,
                                opposite,
                                length: over,
                            }
                        }
                        (Some((cp, _)), Some((cq, _))) => {
                            let r = dist(p.a, cp);
                            if dist(cp, cq) > tol || (dist(q.a, cq) - r).abs() > tol {
                                continue;
                            }
                            // Two arcs of one circle overlap when an end of
                            // one is inside the other, or they are the same.
                            let inside = |s: &Stroke, x: Point| {
                                on_arc(s, x, tol).is_some_and(|t| {
                                    let slack = tol / (r * s.arc.unwrap().1.abs());
                                    t > slack && t < 1. - slack
                                })
                            };
                            let mid = |s: &Stroke| crate::profiles::stroke_point(s, 0.5);
                            if !(inside(p, q.a)
                                || inside(p, q.b)
                                || inside(q, p.a)
                                || inside(q, p.b)
                                || inside(p, mid(q))
                                || inside(q, mid(p)))
                            {
                                continue;
                            }
                            Hit {
                                loops: [la, lb],
                                strokes: [i, j],
                                straight: false,
                                full: false,
                                opposite: false,
                                length: 0.,
                            }
                        }
                        _ => continue,
                    };
                    found.push(hit);
                }
            }
        }
    }
    found
}

/// Whether any kerf paths of different parts coincide (so the parts have to
/// be planned one by one).
pub(super) fn any(profiles: &Profiles, tol: f64) -> bool {
    let rings: Vec<Ring> = profiles
        .loops
        .iter()
        .map(|l| (l.component, l.hole, l.strokes.clone()))
        .collect();
    !hits(&rings, tol).is_empty()
}

/// What the offset of the whole region proves of an ordinary job, proven
/// here for a job with common lines (whose parts are offset one by one):
/// the kerf paths of different parts neither touch nor cross, and none runs
/// inside another part's allowance. By exact distances, between every pair
/// of parts:
/// - every stroke of one part's kerf paths keeps the applied offset from
///   the other part's boundary (so each part keeps its half kerf and
///   allowance from every cut, as from its own);
/// - every two strokes of their kerf paths are apart, except the strokes
///   one declared common line replaces on its two parts (the shared stroke
///   and the corner arc at each end of it), which coincide by definition.
fn separate(
    rings: &[Ring],
    boundaries: &[(usize, Vec<Stroke>)],
    removed: &HashMap<(usize, usize), usize>,
    offset: f64,
    tol: f64,
) -> Result<(), String> {
    let grown = |b: [f64; 4], by: f64| [b[0] - by, b[1] - by, b[2] + by, b[3] + by];
    let disjoint =
        |a: [f64; 4], b: [f64; 4]| a[0] > b[2] || b[0] > a[2] || a[1] > b[3] || b[1] > a[3];
    let boxes: Vec<[f64; 4]> = rings.iter().map(|r| bounds(&r.2)).collect();
    let fix = "outside a common line the kerf paths of different parts must stay apart; move the parts apart";
    for (la, a) in rings.iter().enumerate() {
        for (part, edge) in boundaries {
            if *part == a.0 || disjoint(grown(boxes[la], offset + tol), bounds(edge)) {
                continue;
            }
            for p in &a.2 {
                for q in edge {
                    let d = apart(p, q);
                    if d < offset - tol {
                        return Err(format!(
                            "thermal cut common line: the kerf path of part {} runs {d:.6} mm from part {part}, inside the applied offset {offset} mm; {fix}",
                            a.0
                        ));
                    }
                }
            }
        }
        for (lb, b) in rings.iter().enumerate().skip(la + 1) {
            if a.0 == b.0 || disjoint(grown(boxes[la], tol), boxes[lb]) {
                continue;
            }
            for (i, p) in a.2.iter().enumerate() {
                for (j, q) in b.2.iter().enumerate() {
                    let shared = removed
                        .get(&(la, i))
                        .zip(removed.get(&(lb, j)))
                        .is_some_and(|(x, y)| x == y);
                    if !shared && apart(p, q) <= tol {
                        return Err(format!(
                            "thermal cut common line: the kerf paths of parts {} and {} touch or cross near ({:.3}, {:.3}); {fix}",
                            a.0, b.0, q.a[0], q.a[1]
                        ));
                    }
                }
            }
        }
    }
    Ok(())
}

/// Plans the common lines of oriented loops: returns the loops to cut (the
/// outlines of each group replaced by the group's one outline) and the
/// common cuts.
pub(super) fn plan(
    rings: Vec<Ring>,
    options: CommonLine,
    kerf: f64,
    offset: f64,
    tol: f64,
    reverse: bool,
    boundaries: &[(usize, Vec<Stroke>)],
) -> Result<(Vec<Ring>, Plan), String> {
    let spacing = 2. * offset;
    let fix = format!(
        "a common line needs the two parts exactly {spacing} mm apart (twice the applied offset: the kerf plus the boundary allowance) along a complete straight edge of both; move the parts to that spacing or further apart"
    );
    let mut diagnostics = vec![];
    // Facing straight edges that are near, but not at, the common-line
    // spacing: reported, or refused where one kerf path lies inside the
    // other part's allowance.
    let side = if reverse { -1. } else { 1. };
    for (la, a) in rings.iter().enumerate() {
        for b in rings.iter().skip(la + 1) {
            if a.0 == b.0 || a.1.is_some() || b.1.is_some() {
                continue;
            }
            for p in a.2.iter().filter(|s| s.arc.is_none()) {
                let Some((u, length)) = unit(p) else { continue };
                // The scrap is on the left of travel unless reversed.
                let n = [-side * u[1], side * u[0]];
                for q in b.2.iter().filter(|s| s.arc.is_none()) {
                    let (da, db) = (sub(q.a, p.a), sub(q.b, p.a));
                    let (ta, tb) = (dot(u, da), dot(u, db));
                    let d = dot(n, da);
                    if (d - dot(n, db)).abs() > tol
                        || tb >= ta
                        || ta.min(length) - tb.max(0.) <= tol
                        || d.abs() <= tol
                    {
                        continue;
                    }
                    let gap = spacing + d;
                    if d < 0. && d > -spacing {
                        return Err(format!(
                            "thermal cut common line: parts {} and {} face each other {gap:.6} mm apart, closer than one kerf path allows; {fix}",
                            a.0, b.0
                        ));
                    }
                    if d > 0. && d < kerf {
                        diagnostics.push(format!(
                            "parts {} and {}: facing edges {gap:.6} mm apart are not a common line (exactly {spacing} mm); cut separately, their kerfs overlap",
                            a.0, b.0
                        ));
                    }
                }
            }
        }
    }
    // Coincident kerf paths: each is a common line or refused.
    // The strokes (loop, index) a common line replaces: the shared stroke
    // and the corner arc at either end of it, on both parts; each with the
    // common line it belongs to.
    let mut removed: HashMap<(usize, usize), usize> = HashMap::new();
    // After a stroke (loop, index) the outline runs straight on into the
    // stroke (loop, index) of the other part.
    let mut jump: HashMap<(usize, usize), (usize, usize)> = HashMap::new();
    let mut targets: HashSet<(usize, usize)> = HashSet::new();
    let mut cuts: Vec<Cut> = vec![];
    let parts = rings.iter().map(|r| r.0).max().map_or(0, |m| m + 1);
    let mut group: Vec<usize> = (0..parts).collect();
    fn root(group: &mut [usize], mut i: usize) -> usize {
        while group[i] != i {
            group[i] = group[group[i]];
            i = group[i];
        }
        i
    }
    for h in hits(&rings, tol) {
        let [la, lb] = h.loops;
        let (pa, pb) = (rings[la].0, rings[lb].0);
        let between = format!("thermal cut common line: parts {pa} and {pb}");
        if rings[la].1.is_some() || rings[lb].1.is_some() {
            return Err(format!(
                "{between}: the kerf path of a hole coincides with another part's; a common line with a hole wall is not supported, move the part at least {} mm from the hole wall",
                spacing + kerf
            ));
        }
        if !h.straight {
            return Err(format!(
                "{between}: their kerf paths coincide along an arc; only straight edges can be common lines, separate the parts"
            ));
        }
        if !h.opposite {
            return Err(format!("{between}: the parts overlap"));
        }
        if !h.full {
            return Err(format!(
                "{between}: their facing edges share only {:.6} mm, not a complete edge of both; {fix}",
                h.length
            ));
        }
        if h.length < options.min_length_mm {
            return Err(format!(
                "{between}: the shared edge is {:.6} mm long, shorter than common_line min_length_mm {}; lower it or separate the parts",
                h.length, options.min_length_mm
            ));
        }
        let [i, j] = h.strokes;
        let (sa, sb) = (&rings[la].2, &rings[lb].2);
        let at = |n: usize, i: usize, k: isize| (i as isize + k).rem_euclid(n as isize) as usize;
        // The offset of a sharp outside corner: an arc of the offset's
        // radius turning the way the loop does.
        let corner = |s: &Stroke| {
            s.arc
                .is_some_and(|(c, w)| (dist(s.a, c) - offset).abs() <= tol && w * side < 0.)
        };
        let shape = |s: &[Stroke], i: usize| {
            s.len() >= 6
                && corner(&s[at(s.len(), i, -1)])
                && corner(&s[at(s.len(), i, 1)])
                && s[at(s.len(), i, -2)].arc.is_none()
                && s[at(s.len(), i, 2)].arc.is_none()
        };
        if !shape(sa, i) || !shape(sb, j) {
            return Err(format!(
                "{between}: the shared edge must end at outside corners with a straight edge beyond each (not an inside corner, a fillet or an arc); separate the parts"
            ));
        }
        let (u, _) = unit(&sa[i]).unwrap();
        // At each end the outline arrives on one part's edge and leaves on
        // the other's: the two must be in line.
        let mut ends = [[0.; 2]; 2];
        for (end, (from, to)) in [
            ((la, at(sa.len(), i, -2)), (lb, at(sb.len(), j, 2))),
            ((lb, at(sb.len(), j, -2)), (la, at(sa.len(), i, 2))),
        ]
        .into_iter()
        .enumerate()
        {
            let (s, t) = (rings[from.0].2[from.1], rings[to.0].2[to.1]);
            let flush = unit(&s).zip(unit(&t)).is_some_and(|((v, _), (w, _))| {
                cross(v, sub(t.a, s.a)).abs() <= tol
                    && cross(v, sub(t.b, s.a)).abs() <= tol
                    && dot(v, w) > 0.
                    && dot(v, sub(t.a, s.b)) > 0.
                    && cross(v, u).abs() > 1e-9
            });
            if !flush {
                return Err(format!(
                    "{between}: the parts are not flush at an end of the shared edge (the edges next to it must be in line); align or separate the parts"
                ));
            }
            // Where that line crosses the common line.
            let (v, _) = unit(&s).unwrap();
            let k = cross(sub(s.b, sa[i].a), v) / cross(u, v);
            let junction = [sa[i].a[0] + k * u[0], sa[i].a[1] + k * u[1]];
            ends[end] = junction;
            if jump.insert(from, to).is_some() || !targets.insert(to) {
                return Err(format!(
                    "{between}: common lines that meet at a corner (a grid of parts) are not supported; keep common lines to pairs and rows"
                ));
            }
        }
        for key in [-1, 0, 1]
            .into_iter()
            .flat_map(|k| [(la, at(sa.len(), i, k)), (lb, at(sb.len(), j, k))])
        {
            if removed.insert(key, cuts.len()).is_some() {
                return Err(format!(
                    "{between}: common lines that meet at a corner (a grid of parts) are not supported; keep common lines to pairs and rows"
                ));
            }
        }
        let (ra, rb) = (root(&mut group, pa), root(&mut group, pb));
        group[ra.max(rb)] = ra.min(rb);
        cuts.push(Cut {
            parts: [pa, pb],
            ends,
            shared_length: h.length,
        });
    }
    if jump
        .iter()
        .any(|(k, v)| removed.contains_key(k) || removed.contains_key(v))
    {
        return Err(
            "thermal cut common line: common lines that meet at a corner (a grid of parts) are not supported; keep common lines to pairs and rows".into(),
        );
    }
    separate(&rings, boundaries, &removed, offset, tol)?;
    // The groups and the one outline of each.
    let roots: Vec<usize> = (0..parts).map(|p| root(&mut group, p)).collect();
    let mut groups: Vec<Vec<usize>> = vec![];
    let mut group_of: HashMap<usize, usize> = HashMap::new();
    for c in &cuts {
        let r = roots[c.parts[0]];
        if let std::collections::hash_map::Entry::Vacant(e) = group_of.entry(r) {
            e.insert(groups.len());
            groups.push((0..parts).filter(|p| roots[*p] == r).collect());
        }
    }
    let outline_of = |part: usize| {
        rings
            .iter()
            .position(|r| r.0 == part && r.1.is_none())
            .ok_or("thermal cut common line: a part has no outline loop")
    };
    let mut outlines: HashMap<usize, Vec<Stroke>> = HashMap::new();
    for members in &groups {
        let mut all = vec![];
        for &m in members {
            let l = outline_of(m)?;
            all.extend((0..rings[l].2.len()).map(|s| (l, s)));
        }
        let kept = all.iter().filter(|k| !removed.contains_key(k)).count();
        let start = *all
            .iter()
            .find(|k| !removed.contains_key(k) && !targets.contains(k))
            .ok_or("thermal cut common line: the group's outline has no free edge to start on")?;
        let single = "thermal cut common line: the parts joined by common lines do not have one outline (they enclose scrap); separate them";
        let mut strokes = vec![];
        let mut seen = 0;
        let mut cur = start;
        loop {
            let mut s = rings[cur.0].2[cur.1];
            // Straight on across each end of a common line: the two edges
            // are in line, so they are one move (no vertex at the junction).
            loop {
                if removed.contains_key(&cur) || seen > kept {
                    return Err(single.into());
                }
                seen += 1;
                let Some(&next) = jump.get(&cur) else { break };
                cur = next;
                s.b = rings[cur.0].2[cur.1].b;
            }
            strokes.push(s);
            cur = (cur.0, (cur.1 + 1) % rings[cur.0].2.len());
            if cur == start {
                break;
            }
        }
        if seen != kept {
            return Err(single.into());
        }
        outlines.insert(members[0], strokes);
    }
    let mut out = vec![];
    for (part, hole, strokes) in rings {
        if hole.is_some() {
            out.push((part, hole, strokes));
        } else if let Some(g) = group_of.get(&roots[part]) {
            // Only the group's first part carries its outline.
            if let Some(strokes) = outlines.remove(&part).filter(|_| groups[*g][0] == part) {
                out.push((part, None, strokes));
            }
        } else {
            out.push((part, None, strokes));
        }
    }
    let cuts = cuts
        .into_iter()
        .map(|c| (group_of[&roots[c.parts[0]]], c))
        .collect();
    Ok((
        out,
        Plan {
            groups,
            cuts,
            diagnostics,
        },
    ))
}

impl Plan {
    /// The parts cut together with `part` (itself when it has no common
    /// line), the outline's owner first.
    pub(super) fn members(&self, part: usize) -> Vec<usize> {
        self.groups
            .iter()
            .find(|g| g.contains(&part))
            .cloned()
            .unwrap_or_else(|| vec![part])
    }
    /// The common cuts of the group of `part`.
    pub(super) fn cuts_of(&self, part: usize) -> Vec<usize> {
        let Some(g) = self.groups.iter().position(|g| g.contains(&part)) else {
            return vec![];
        };
        (0..self.cuts.len())
            .filter(|&c| self.cuts[c].0 == g)
            .collect()
    }
    pub(super) fn cut(&self, index: usize) -> &Cut {
        &self.cuts[index].1
    }
    pub(super) fn len(&self) -> usize {
        self.cuts.len()
    }
}

/// Whether a point of an arc stroke's circle lies on the arc (the ends
/// included; a point missed beside an end is covered by that end, which is
/// always a candidate of its own).
fn within(s: &Stroke, p: Point) -> bool {
    on_arc(s, p, 1e-9).is_some()
}

/// The exact distance from a point to a stroke.
fn to_stroke(p: Point, s: &Stroke) -> f64 {
    let ends = dist(p, s.a).min(dist(p, s.b));
    match s.arc {
        None => {
            let v = sub(s.b, s.a);
            let n = dot(v, v);
            if n > 0. {
                let t = (dot(sub(p, s.a), v) / n).clamp(0., 1.);
                dist(p, [s.a[0] + t * v[0], s.a[1] + t * v[1]])
            } else {
                ends
            }
        }
        Some((c, _)) => {
            let (r, d) = (dist(s.a, c), dist(p, c));
            // The nearest point of the circle, when it is on the arc.
            if d > 0.
                && within(
                    s,
                    [c[0] + (p[0] - c[0]) * r / d, c[1] + (p[1] - c[1]) * r / d],
                )
            {
                (d - r).abs()
            } else {
                ends
            }
        }
    }
}

/// The exact smallest distance between two strokes (lines or arcs): zero
/// where they meet, otherwise the least over the points where a smallest
/// distance can lie: the ends of either, and the points where the joining
/// line is square to both (the foot of an arc's center on a line; the
/// points of two arcs on the line through their centers).
pub(super) fn apart(p: &Stroke, q: &Stroke) -> f64 {
    let mut least = to_stroke(p.a, q)
        .min(to_stroke(p.b, q))
        .min(to_stroke(q.a, p))
        .min(to_stroke(q.b, p));
    match (p.arc, q.arc) {
        (None, None) => {
            let (v, w) = (sub(p.b, p.a), sub(q.b, q.a));
            let k = cross(v, w);
            if k != 0. {
                let d = sub(q.a, p.a);
                let (t, u) = (cross(d, w) / k, cross(d, v) / k);
                if (0. ..=1.).contains(&t) && (0. ..=1.).contains(&u) {
                    return 0.;
                }
            }
        }
        (None, Some(_)) | (Some(_), None) => {
            let (line, arc) = if p.arc.is_none() { (p, q) } else { (q, p) };
            let (c, _) = arc.arc.unwrap();
            let r = dist(arc.a, c);
            let v = sub(line.b, line.a);
            let n = dot(v, v);
            if n > 0. {
                let at = |t: f64| [line.a[0] + t * v[0], line.a[1] + t * v[1]];
                // Where the line meets the circle.
                let foot = dot(sub(c, line.a), v) / n;
                let h2 = r * r - dist(at(foot), c).powi(2);
                if h2 >= 0. {
                    let h = (h2 / n).sqrt();
                    for t in [foot - h, foot + h] {
                        if (0. ..=1.).contains(&t) && within(arc, at(t)) {
                            return 0.;
                        }
                    }
                }
                if (0. ..=1.).contains(&foot) {
                    least = least.min(to_stroke(at(foot), arc));
                }
            }
        }
        (Some((c1, _)), Some((c2, _))) => {
            let (r1, r2) = (dist(p.a, c1), dist(q.a, c2));
            let d = dist(c1, c2);
            if d > 0. {
                let u = [(c2[0] - c1[0]) / d, (c2[1] - c1[1]) / d];
                // Where the circles meet.
                let x = (d * d + r1 * r1 - r2 * r2) / (2. * d);
                let h2 = r1 * r1 - x * x;
                if h2 >= 0. {
                    let h = h2.sqrt();
                    for k in [-h, h] {
                        let m = [c1[0] + x * u[0] - k * u[1], c1[1] + x * u[1] + k * u[0]];
                        if within(p, m) && within(q, m) {
                            return 0.;
                        }
                    }
                }
                for k in [-1., 1.] {
                    let on_p = [c1[0] + k * r1 * u[0], c1[1] + k * r1 * u[1]];
                    if within(p, on_p) {
                        least = least.min(to_stroke(on_p, q));
                    }
                    let on_q = [c2[0] + k * r2 * u[0], c2[1] + k * r2 * u[1]];
                    if within(q, on_q) {
                        least = least.min(to_stroke(on_q, p));
                    }
                }
            }
        }
    }
    least
}

fn flat(c: &Curve) -> Stroke {
    let (a, b) = (c.start(), c.end());
    Stroke {
        a: [a[0], a[1]],
        b: [b[0], b[1]],
        arc: match *c {
            Curve::Line { .. } => None,
            Curve::Arc {
                center, sweep_rad, ..
            }
            | Curve::Helix {
                center, sweep_rad, ..
            } => Some(([center[0], center[1]], sweep_rad)),
        },
    }
}

/// With common lines the kerfs of one group meet, so two more things are
/// proven of every pierce and lead, against every kerf (cut or lead) burnt
/// under an earlier pierce, by exact distances:
/// - each pierce lies in uncut sheet, at least a kerf from the center of
///   every earlier kerf (a pierce over a kerf has no material to start in);
/// - each lead-in and lead-out keeps its whole kerf out of every earlier
///   kerf: their centers are at least a kerf apart, so the beam never runs
///   along or across an open slot on a lead.
///
/// `labels` names what each pierce cuts, in order.
pub(super) fn check_kerfs<'a>(
    steps: impl Iterator<Item = &'a Step>,
    labels: &[String],
    kerf: f64,
    tol: f64,
) -> Result<(), String> {
    let mut earlier: Vec<Stroke> = vec![];
    let mut current: Vec<Stroke> = vec![];
    let mut at: Option<Point> = None;
    let mut on = false;
    let mut pierces = 0;
    let mut cutting = false;
    let name = |pierces: usize| {
        labels
            .get(pierces.wrapping_sub(1))
            .map_or("a loop", String::as_str)
    };
    for step in steps {
        match step {
            Step::Move(curve, kind, _) => {
                let end = curve.end();
                at = Some([end[0], end[1]]);
                if !on || !matches!(kind, MoveKind::Cut | MoveKind::Lead) {
                    continue;
                }
                let s = flat(curve);
                if *kind == MoveKind::Cut {
                    cutting = true;
                } else if let Some(d) = earlier
                    .iter()
                    .map(|e| apart(&s, e))
                    .find(|d| *d < kerf - tol)
                {
                    return Err(format!(
                        "thermal cut common line: the lead-{} of {} from ({:.3}, {:.3}) to ({:.3}, {:.3}) runs {d:.3} mm from the center of a kerf cut before it (it needs the kerf width, {kerf} mm, so it does not cross an open slot); change that lead's length_mm or radius_mm, or space the parts so the lead has uncut sheet",
                        if cutting { "out" } else { "in" },
                        name(pierces),
                        s.a[0],
                        s.a[1],
                        s.b[0],
                        s.b[1]
                    ));
                }
                current.push(s);
            }
            Step::Event(Event::Beam { power_percent }) => {
                let fire = *power_percent > 0.;
                if fire && !on {
                    pierces += 1;
                    cutting = false;
                    if let Some(p) = at
                        && earlier.iter().any(|e| to_stroke(p, e) < kerf - tol)
                    {
                        return Err(format!(
                            "thermal cut common line: the pierce of {} at ({:.3}, {:.3}) lies on a kerf cut before it; change the lead lengths so every pierce is in uncut sheet",
                            name(pierces),
                            p[0],
                            p[1]
                        ));
                    }
                }
                if !fire {
                    earlier.append(&mut current);
                }
                on = fire;
            }
            _ => {}
        }
    }
    Ok(())
}

/// What a common cut needs from the operation.
pub(super) struct Emit<'a> {
    pub probe: &'a dyn Fn(&PreparedAssembly, Curve, &str) -> Result<(), String>,
    pub kerf_probe: &'a PreparedAssembly,
    pub crater_probe: &'a PreparedAssembly,
    /// The straight lead-in, along the extension of the line.
    pub lead_mm: f64,
    pub top: f64,
    pub z_check: f64,
    pub z_cut: f64,
    pub z_pierce: f64,
    pub z_travel: f64,
    pub program: bool,
    pub power_percent: f64,
    pub pierce_delay_s: f64,
    pub plunge_mm_min: Option<f64>,
    pub feed_mm_min: f64,
    pub height_control: bool,
}

impl Cut {
    /// One pierce in the scrap on the extension of the line, a straight
    /// lead-in along it, and the cut from the group's outline at one end to
    /// the outline at the other. It ends there without a lead-out: the
    /// outline, cut afterwards, passes through both ends.
    pub(super) fn steps(
        &self,
        e: &Emit,
        pos: &mut Vec3,
    ) -> Result<(Vec<Step>, CommonLineReport, f64), String> {
        let at = |p: Point, z: f64| [p[0], p[1], z];
        let line = |a: Vec3, b: Vec3| Curve::Line { start: a, end: b };
        let near = |p: Point| dist(p, [pos[0], pos[1]]);
        let mut order = self.ends;
        if near(order[1]) < near(order[0]) {
            order.swap(0, 1);
        }
        let mut chosen = None;
        let mut refusal = String::new();
        for (start, end) in [(order[0], order[1]), (order[1], order[0])] {
            let length = dist(start, end);
            let u = [(end[0] - start[0]) / length, (end[1] - start[1]) / length];
            let pierce = [start[0] - e.lead_mm * u[0], start[1] - e.lead_mm * u[1]];
            let checked = (e.probe)(
                e.crater_probe,
                line(at(pierce, e.top + 1.), at(pierce, e.z_check)),
                "common-line pierce point",
            )
            .and_then(|()| {
                if e.lead_mm > 0. {
                    (e.probe)(
                        e.kerf_probe,
                        line(at(pierce, e.z_check), at(start, e.z_check)),
                        "common-line lead",
                    )?;
                }
                (e.probe)(
                    e.kerf_probe,
                    line(at(start, e.z_check), at(end, e.z_check)),
                    "common-line kerf path",
                )
            });
            match checked {
                Ok(()) => {
                    chosen = Some((start, end, pierce, length));
                    break;
                }
                Err(error) if refusal.is_empty() => refusal = error,
                Err(_) => {}
            }
        }
        let Some((start, end, pierce, length)) = chosen else {
            return Err(format!(
                "common line between parts {} and {}: no pierce on the extension of the line, lead_in length_mm {} from either end, is clear of the parts; lengthen lead_in length_mm (the pierce needs pierce_clearance_mm beyond half a kerf from both parts): {refusal}",
                self.parts[0], self.parts[1], e.lead_mm
            ));
        };
        let mut steps = vec![];
        fn go(steps: &mut Vec<Step>, pos: &mut Vec3, to: Vec3, kind: MoveKind, feed: Option<f64>) {
            if *pos != to {
                steps.push(Step::Move(
                    Curve::Line {
                        start: *pos,
                        end: to,
                    },
                    kind,
                    feed,
                ));
                *pos = to;
            }
        }
        let feed = Some(e.feed_mm_min);
        if e.program {
            let up = [pos[0], pos[1], e.z_travel.max(pos[2])];
            go(&mut steps, pos, up, MoveKind::Rapid, None);
            let over = at(pierce, pos[2]);
            go(&mut steps, pos, over, MoveKind::Rapid, None);
            go(
                &mut steps,
                pos,
                at(pierce, e.z_pierce),
                MoveKind::Rapid,
                None,
            );
        } else {
            go(&mut steps, pos, at(pierce, e.z_cut), MoveKind::Rapid, None);
        }
        steps.push(Step::Event(Event::Beam {
            power_percent: e.power_percent,
        }));
        if e.pierce_delay_s > 0. {
            steps.push(Step::Event(Event::Dwell {
                seconds: e.pierce_delay_s,
            }));
        }
        let down = at(pierce, e.z_cut);
        go(&mut steps, pos, down, MoveKind::Plunge, e.plunge_mm_min);
        go(&mut steps, pos, at(start, e.z_cut), MoveKind::Lead, feed);
        if e.height_control {
            steps.push(Step::Event(Event::HeightControl { on: true }));
        }
        go(&mut steps, pos, at(end, e.z_cut), MoveKind::Cut, feed);
        if e.height_control {
            steps.push(Step::Event(Event::HeightControl { on: false }));
        }
        steps.push(Step::Event(Event::Beam { power_percent: 0. }));
        if e.program {
            let up = [pos[0], pos[1], e.z_travel];
            go(&mut steps, pos, up, MoveKind::Rapid, None);
        }
        Ok((
            steps,
            CommonLineReport {
                parts: self.parts,
                start_mm: start,
                end_mm: end,
                shared_length_mm: self.shared_length,
            },
            length,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line(a: Point, b: Point) -> Stroke {
        Stroke::line(a, b)
    }
    fn arc(c: Point, r: f64, from: f64, sweep: f64) -> Stroke {
        let at = |a: f64| [c[0] + r * a.cos(), c[1] + r * a.sin()];
        Stroke {
            a: at(from),
            b: at(from + sweep),
            arc: Some((c, sweep)),
        }
    }
    /// The distance by brute force: the least over points `step` apart on
    /// both (never below the true distance, at most `step` above it).
    fn sampled(p: &Stroke, q: &Stroke, n: usize) -> f64 {
        let points = |s: &Stroke| -> Vec<Point> {
            (0..=n)
                .map(|k| crate::profiles::stroke_point(s, k as f64 / n as f64))
                .collect()
        };
        let (a, b) = (points(p), points(q));
        a.iter()
            .flat_map(|x| b.iter().map(move |y| dist(*x, *y)))
            .fold(f64::INFINITY, f64::min)
    }

    #[test]
    fn stroke_distances_are_exact() {
        let near = |a: f64, b: f64| assert!((a - b).abs() < 1e-12, "{a} != {b}");
        // Lines: crossing, parallel, end to side.
        near(
            apart(&line([0., 0.], [4., 4.]), &line([0., 4.], [4., 0.])),
            0.,
        );
        near(
            apart(&line([0., 0.], [4., 0.]), &line([1., 2.], [9., 2.])),
            2.,
        );
        near(
            apart(&line([0., 0.], [4., 0.]), &line([6., -3.], [6., 3.])),
            2.,
        );
        // A line passing an arc: nearest at the foot of the center.
        let quarter = arc([0., 0.], 5., 0., std::f64::consts::FRAC_PI_2);
        near(
            apart(&line([6., 6.], [-2., 14.]), &quarter),
            72f64.sqrt() - 5.,
        );
        // The same line cannot reach the other half of the circle.
        let far = arc(
            [0., 0.],
            5.,
            std::f64::consts::PI,
            std::f64::consts::FRAC_PI_2,
        );
        near(apart(&line([6., 6.], [-2., 14.]), &far), 17. / 2f64.sqrt());
        near(apart(&line([-9., 3.], [9., 3.]), &quarter), 0.);
        // A chord inside the circle, between the arc's ends.
        near(apart(&line([1., 1.], [2., 2.]), &quarter), 5. - 8f64.sqrt());
        // Arcs: concentric over a common span, concentric apart, crossing,
        // and facing each other along the line of centers.
        let inner = arc([0., 0.], 3., 0.5, 0.5);
        near(apart(&quarter, &inner), 2.);
        let behind = arc([0., 0.], 3., 3., 0.5);
        near(apart(&quarter, &behind), dist(quarter.b, behind.a));
        near(apart(&quarter, &arc([6., 0.], 5., 2., 1.)), 0.);
        near(apart(&quarter, &arc([12., 0.], 5., 2.5, 1.2)), 2.);
        near(
            apart(&arc([0., 0.], 5., -0.5, 1.), &arc([12., 0.], 5., 2.5, 1.2)),
            2.,
        );
    }

    #[test]
    fn stroke_distances_agree_with_brute_force() {
        // A fixed pseudo-random set of lines and arcs.
        let mut state = 0x2545F4914F6CDD1Du64;
        let mut next = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state >> 11) as f64 / (1u64 << 53) as f64
        };
        let mut strokes = vec![];
        for k in 0..40 {
            let p = [20. * next() - 10., 20. * next() - 10.];
            strokes.push(if k % 2 == 0 {
                line(p, [20. * next() - 10., 20. * next() - 10.])
            } else {
                arc(
                    p,
                    0.5 + 6. * next(),
                    6.2 * next(),
                    (0.2 + 5.8 * next()) * if k % 4 == 1 { 1. } else { -1. },
                )
            });
        }
        let n = 400;
        for (i, p) in strokes.iter().enumerate() {
            for q in &strokes[i + 1..] {
                let (exact, brute) = (apart(p, q), sampled(p, q, n));
                // Neighbouring samples are at most this far apart.
                let step = [p, q]
                    .iter()
                    .map(|s| crate::profiles::stroke_length(s) / n as f64)
                    .sum::<f64>();
                assert!(exact <= brute + 1e-9, "{p:?} {q:?}: {exact} > {brute}");
                assert!(
                    exact >= brute - step,
                    "{p:?} {q:?}: {exact} < {brute} - {step}"
                );
                assert_eq!(exact.to_bits(), apart(q, p).to_bits(), "{p:?} {q:?}");
            }
        }
    }
}
