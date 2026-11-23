//! Conservative simplification before erosion: hulls of almost-convex linear
//! holes, then, on every loop, shallow convex arcs taken as their chords and
//! local removal of tiny convex corners of the region along its straight
//! runs. Both only shrink machining space (protected material
//! grows), never the reverse. Retained vertices are exact input vertices; each
//! replaced chain has a reported Hausdorff bound. This is not input repair.
use super::*;
use robust::{Coord, orient2d};

fn turn(a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> f64 {
    orient2d(
        Coord { x: a[0], y: a[1] },
        Coord { x: b[0], y: b[1] },
        Coord { x: c[0], y: c[1] },
    )
}

fn envelope(boundary: &Boundary, budget: f64) -> Option<(Boundary, f64)> {
    let v = &boundary.vertices;
    if v.len() < 4 || v.iter().any(|p| p[2] != 0.) {
        return None;
    }
    let mut sorted: Vec<_> = (0..v.len()).collect();
    sorted.sort_unstable_by(|&i, &j| {
        v[i][0]
            .total_cmp(&v[j][0])
            .then(v[i][1].total_cmp(&v[j][1]))
    });
    let mut hull = Vec::new();
    for &i in &sorted {
        while hull.len() >= 2 && turn(v[hull[hull.len() - 2]], v[hull[hull.len() - 1]], v[i]) <= 0.
        {
            hull.pop();
        }
        hull.push(i);
    }
    let lower = hull.len();
    for &i in sorted.iter().rev().skip(1) {
        while hull.len() > lower
            && turn(v[hull[hull.len() - 2]], v[hull[hull.len() - 1]], v[i]) <= 0.
        {
            hull.pop();
        }
        hull.push(i);
    }
    hull.pop();
    if hull.len() < 3 || hull.len() == v.len() {
        return None;
    }
    // Hull vertices must occur in the same circular order as the authored loop.
    // Both orientations are supported; sorting indices retains the input one.
    let mut order = hull.clone();
    order.sort_unstable();
    let start = hull.iter().position(|&i| i == order[0])?;
    let forward = (0..hull.len()).all(|i| hull[(start + i) % hull.len()] == order[i]);
    let reverse = (0..hull.len()).all(|i| hull[(start + hull.len() - i) % hull.len()] == order[i]);
    if !forward && !reverse {
        return None;
    }
    let mut error = 0f64;
    let scale = v
        .iter()
        .flat_map(|p| &p[..2])
        .map(|x| x.abs())
        .fold(1., f64::max);
    let guard = 512. * f64::EPSILON * scale;
    for k in 0..order.len() {
        let (i, j) = (order[k], order[(k + 1) % order.len()]);
        if (i + 1) % v.len() == j {
            continue;
        }
        let a = [v[i][0], v[i][1]];
        let b = [v[j][0], v[j][1]];
        let mut q = (i + 1) % v.len();
        while q != j {
            // Distance to a convex segment is convex along each original line.
            // Endpoints therefore enclose every line, not only samples. The
            // connected chain joins A to B: its projection covers AB, giving
            // the reverse distance bound too.
            error = error.max(
                crate::geometry::point_segment_distance_sq([v[q][0], v[q][1]], a, b).sqrt() + guard,
            );
            if error > budget {
                return None;
            }
            q = (q + 1) % v.len();
        }
    }
    Some((
        Boundary {
            vertices: order.into_iter().map(|i| v[i]).collect(),
        },
        error,
    ))
}

/// Largest boundary displacement allowed when conservatively simplifying a
/// region before erosion (mm). Covers the sub-micron zigzag of sliced
/// triangle meshes, on which the offset kernel otherwise mis-joins slices;
/// well inside the 0.01 mm boundary budgets. Removal only ever shrinks the
/// machinable region, the bound actually used is added to the boundary error,
/// and every planned path is still checked against the ORIGINAL boundary.
pub(super) const EROSION_SIMPLIFY_BUDGET_MM: f64 = 0.002;

/// Local conservative simplification of a straight-line chain of a loop whose
/// region interior is on the LEFT (outer CCW, holes CW). A vertex is removed
/// only when it is a strictly convex corner of the region (removal shrinks the
/// region) and every original vertex replaced by the new chord lies within
/// `budget` of it. Distance to a segment is convex along each original line,
/// so original vertices bound the whole replaced chain (and, the chain being
/// connected from chord end to chord end, the reverse direction too): the
/// returned value is a Hausdorff bound. Arc segments are never touched.
fn shrink_simplify(boundary: &Boundary, budget: f64) -> Option<(Boundary, f64)> {
    let v = &boundary.vertices;
    let n = v.len();
    if n < 4 {
        return None;
    }
    let scale = v
        .iter()
        .flat_map(|p| &p[..2])
        .map(|x| x.abs())
        .fold(1., f64::max);
    let guard = 512. * f64::EPSILON * scale;
    let mut keep: Vec<usize> = (0..n).collect();
    let mut error = 0f64;
    loop {
        let mut changed = false;
        let mut i = 0;
        while i < keep.len() && keep.len() > 3 {
            let m = keep.len();
            let (ia, ib, ic) = (keep[(i + m - 1) % m], keep[i], keep[(i + 1) % m]);
            if turn(v[ia], v[ib], v[ic]) > 0. {
                let (a, c) = ([v[ia][0], v[ia][1]], [v[ic][0], v[ic][1]]);
                let mut bound = 0f64;
                let mut lines = v[ia][2] == 0.;
                let mut q = (ia + 1) % n;
                while lines && q != ic {
                    lines = v[q][2] == 0.;
                    bound = bound.max(
                        crate::geometry::point_segment_distance_sq([v[q][0], v[q][1]], a, c).sqrt()
                            + guard,
                    );
                    q = (q + 1) % n;
                }
                if lines && bound <= budget {
                    error = error.max(bound);
                    keep.remove(i);
                    changed = true;
                    continue;
                }
            }
            i += 1;
        }
        if !changed {
            break;
        }
    }
    (keep.len() < n).then(|| {
        (
            Boundary {
                vertices: keep.into_iter().map(|i| v[i]).collect(),
            },
            error,
        )
    })
}

/// A loop whose region interior is on the LEFT (outer CCW, holes CW) with
/// every left-turning minor arc (bulge in `(0, 1]`, so the region is convex
/// along it) whose sagitta is within `budget` replaced by its chord. The
/// chord lies inside the region: only the circular segment between arc and
/// chord leaves it, so the region only shrinks, by at most the sagitta (the
/// arc's Hausdorff distance from its chord, `bulge * chord / 2`). Endpoints
/// stay the exact input vertices; right-turning arcs, which would grow the
/// region, are never touched. A shadow grown with round joins carries such an
/// arc of a few microns at every facet corner (#393); with them the offset
/// kernel's input is no longer linear, so `shrink_simplify` cannot merge the
/// near-collinear facets on which it mis-joins slices.
fn chord_shallow_arcs(boundary: &Boundary, budget: f64) -> Option<(Boundary, f64, usize)> {
    let v = &boundary.vertices;
    let n = v.len();
    let scale = v
        .iter()
        .flat_map(|p| &p[..2])
        .map(|x| x.abs())
        .fold(1., f64::max);
    let guard = 512. * f64::EPSILON * scale;
    let mut out = boundary.clone();
    let (mut error, mut chorded) = (0f64, 0usize);
    for i in 0..n {
        let bulge = v[i][2];
        if !(bulge > 0. && bulge <= 1.) {
            continue;
        }
        let next = v[(i + 1) % n];
        let sagitta = bulge * (next[0] - v[i][0]).hypot(next[1] - v[i][1]) / 2. + guard;
        if sagitta <= budget {
            out.vertices[i][2] = 0.;
            error = error.max(sagitta);
            chorded += 1;
        }
    }
    (chorded > 0).then_some((out, error, chorded))
}

impl PreparedRegion {
    /// Cover small notches in a protective shadow before expanding it.
    /// Every hull contains its original outer ring, and the holes stay fixed.
    pub(crate) fn protective_outer_envelope(&self) -> Option<PreparedRegion> {
        let mut definition = self.definition.clone();
        let mut error = 0f64;
        for component in &mut definition.components {
            if let Some((outer, bound)) = envelope(&component.outer, EROSION_SIMPLIFY_BUDGET_MM) {
                component.outer = outer;
                error = error.max(bound);
            }
        }
        if error == 0. {
            return None;
        }
        definition.boundary_error_mm += error;
        // A proposed hull can close a gap to another component. Use it
        // only if the full result passes ordinary topology validation.
        definition.prepare(self.options).ok()
    }

    /// Only positive offsets dilate holes. Dilation is monotone and preserves
    /// this Hausdorff bound. Other boundaries and the original source stay exact.
    pub(super) fn erosion_envelope(&self) -> Option<(PreparedRegion, f64, usize)> {
        let mut replacements = Vec::new();
        let mut error = 0f64;
        let mut removed = 0;
        for (ci, c) in self.definition.components.iter().enumerate() {
            for (hi, hole) in c.holes.iter().enumerate() {
                if let Some((new, bound)) = envelope(hole, self.options.tolerance_mm / 8.) {
                    removed += hole.vertices.len() - new.vertices.len();
                    error = error.max(bound);
                    replacements.push((ci, hi, new));
                }
            }
        }
        let mut definition = self.definition.clone();
        for (ci, hi, new) in replacements {
            definition.components[ci].holes[hi] = new;
        }
        // Displacements of successive steps on the same loop add (triangle
        // inequality for Hausdorff distance): hull bound + local bound, the
        // local bound being the chorded arcs' plus the chain simplification's,
        // within the one budget.
        let mut local = 0f64;
        for c in &mut definition.components {
            for b in std::iter::once(&mut c.outer).chain(c.holes.iter_mut()) {
                let mut arcs = 0f64;
                if let Some((new, bound, chorded)) =
                    chord_shallow_arcs(b, EROSION_SIMPLIFY_BUDGET_MM / 2.)
                {
                    removed += chorded;
                    arcs = bound;
                    *b = new;
                }
                let mut chain = 0f64;
                if let Some((new, bound)) = shrink_simplify(b, EROSION_SIMPLIFY_BUDGET_MM - arcs) {
                    removed += b.vertices.len() - new.vertices.len();
                    chain = bound;
                    *b = new;
                }
                local = local.max(arcs + chain);
            }
        }
        error += local;
        if removed == 0 {
            return None;
        }
        definition.boundary_error_mm += error;
        // An expanded hole may approach another boundary. Ordinary topology
        // validation remains mandatory; use the original kernel input on refusal.
        definition
            .prepare(self.options)
            .ok()
            .map(|p| (p, error, removed))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn boundary(shift: f64, depth: f64) -> Boundary {
        Boundary {
            vertices: [
                [0., 0., 0.],
                [5., depth, 0.],
                [10., 0., 0.],
                [10., 10., 0.],
                [0., 10., 0.],
            ]
            .map(|p| [p[0] + shift, p[1] + shift, p[2]])
            .to_vec(),
        }
    }

    #[test]
    fn only_shallow_convex_arcs_become_chords_and_the_region_only_shrinks() {
        // A 10 mm square (CCW), its corners rounded by quarter arcs of
        // 1e-4 mm radius (sagitta 2.9e-5 mm), the left side bulging out by a
        // shallow arc (sagitta 5e-4 mm) and the bottom side bulging in by one,
        // a right-turning arc whose chord would grow the region.
        let (q, k) = (1e-4, std::f64::consts::FRAC_PI_8.tan());
        let b = Boundary {
            vertices: vec![
                [q, 0., -1e-4],
                [10. - q, 0., k],
                [10., q, 0.],
                [10., 10. - q, k],
                [10. - q, 10., 0.],
                [q, 10., k],
                [0., 10. - q, 1e-4],
                [0., q, k],
            ],
        };
        let (chorded, error, count) = chord_shallow_arcs(&b, 0.001).unwrap();
        assert_eq!(count, 5);
        for (old, new) in b.vertices.iter().zip(&chorded.vertices) {
            assert_eq!(old[..2], new[..2], "endpoints stay exact");
        }
        let bulges: Vec<f64> = chorded.vertices.iter().map(|v| v[2]).collect();
        assert_eq!(bulges, [-1e-4, 0., 0., 0., 0., 0., 0., 0.]);
        let left = 1e-4 * (10. - 2. * q) / 2.;
        // The sagitta plus the rounding guard (512 ulps at the coordinates).
        assert!(error >= left && error <= left + 1e-11, "{error}");
        // The chord is inside: the region shrinks, by no more than `error`
        // anywhere (checked on the prepared regions' own queries).
        let region = |b: &Boundary| {
            PlanarRegion {
                version: 1,
                units: "mm".into(),
                name: "arcs".into(),
                boundary_error_mm: 0.,
                components: vec![Component {
                    outer: b.clone(),
                    holes: vec![],
                }],
                source: serde_json::Value::Null,
            }
            .prepare(ConstructionOptions::default())
            .unwrap()
        };
        let (before, after) = (region(&b), region(&chorded));
        assert!(after.summary().area_mm2 < before.summary().area_mm2);
        // Erosions take it: the arcs no longer keep the region from the
        // simplification that runs before them.
        let (_, bound, removed) = before.erosion_envelope().unwrap();
        assert!(removed >= 5 && bound >= error && bound <= EROSION_SIMPLIFY_BUDGET_MM);
        for i in 0..=40 {
            for j in 0..=40 {
                let p = [-0.1 + i as f64 * 0.255, -0.1 + j as f64 * 0.255];
                let a = after.query(p).unwrap().signed_clearance_mm.unwrap();
                let o = before.query(p).unwrap().signed_clearance_mm.unwrap();
                // Within the queries' own rounding (1e-10).
                assert!(
                    a <= o + 1e-10 && o - a <= error + 1e-10,
                    "{p:?}: {o} -> {a}"
                );
            }
        }
        // Nothing to chord: a circle's half arcs, or a budget below every sagitta.
        let circle = PlanarRegion::circle("c", [0., 0.], 1.).unwrap();
        assert!(chord_shallow_arcs(&circle.components[0].outer, 0.001).is_none());
        assert!(chord_shallow_arcs(&b, 1e-9).is_none());
    }

    #[test]
    fn protective_envelope_fills_only_bounded_notches_and_reports_the_error() {
        let options = ConstructionOptions::default();
        for depth in [0.0001, 0.01] {
            let mut source = PlanarRegion::circle("shadow", [0., 0.], 1.).unwrap();
            source.components[0].outer = boundary(0., depth);
            let shadow = source.prepare(options).unwrap();
            let result = shadow.protective_outer_envelope();
            if depth < EROSION_SIMPLIFY_BUDGET_MM {
                let covered = result.unwrap();
                assert!((covered.summary().area_mm2 - 100.).abs() < 1e-10);
                assert!(covered.definition().boundary_error_mm >= depth);
                assert_eq!(shadow.definition().components[0].outer.vertices.len(), 5);
            } else {
                assert!(result.is_none());
            }
        }
    }

    #[test]
    fn envelope_is_bounded_and_preserves_retained_vertices_in_both_orientations() {
        for shift in [0., 100_000.] {
            for reverse in [false, true] {
                let mut b = boundary(shift, 1e-7);
                if reverse {
                    b.vertices.reverse();
                }
                let (h, e) = envelope(&b, 1e-6).unwrap();
                assert_eq!(h.vertices.len(), 4);
                assert!(
                    e >= (b.vertices[if reverse { 3 } else { 1 }][1] - shift).abs() && e <= 1e-6
                );
                assert!(h.vertices.iter().all(|p| b.vertices.contains(p)));
            }
        }
        assert!(envelope(&boundary(0., 1.), 1e-6).is_none());
        let mut arc = boundary(0., 1e-7);
        arc.vertices[0][2] = 0.1;
        assert!(envelope(&arc, 1e-6).is_none());
    }
    #[test]
    fn only_holes_expand_and_original_source_is_unchanged() {
        let mut r = PlanarRegion::circle("envelope", [5., 5.], 20.).unwrap();
        r.components[0].holes.push(boundary(0., 1e-7));
        r.components.extend(
            PlanarRegion::circle("island", [5., 5.], 1.)
                .unwrap()
                .components,
        );
        let p = r.prepare(ConstructionOptions::default()).unwrap();
        let before = p.definition.clone();
        let (q, error, removed) = p.erosion_envelope().unwrap();
        assert_eq!(removed, 1);
        assert!(error > 0. && error <= 1e-6);
        assert_eq!(p.definition, before);
        assert_eq!(q.definition.components[0].outer, before.components[0].outer);
        assert_eq!(q.definition.components[1], before.components[1]);
        assert_eq!(
            q.definition.boundary_error_mm,
            before.boundary_error_mm + error
        );
        // Independent linear predicates confirm that new protected material
        // contains the original, while an ordinary concavity is never erased.
        for x in 1..10 {
            for y in 1..10 {
                assert_eq!(
                    p.query([x as f64, y as f64]).unwrap().verdict,
                    q.query([x as f64, y as f64]).unwrap().verdict
                );
            }
        }
    }

    #[test]
    fn faceted_hole_offset_matches_independent_convex_dilation_area() {
        let n = 96;
        let points: Vec<_> = (0..n)
            .map(|i| {
                let a = std::f64::consts::TAU * i as f64 / n as f64;
                [10. * a.cos(), 10. * a.sin()]
            })
            .collect();
        let mut vertices = Vec::new();
        let mut area = 0.;
        let mut perimeter = 0.;
        for i in 0..n {
            let (a, b) = (points[i], points[(i + 1) % n]);
            vertices.push([a[0], a[1], 0.]);
            // A near-collinear, very short inward turn on each facet, as can
            // arise from independent intersections with a triangulated solid.
            let p = [a[0] * 0.996 + b[0] * 0.004, a[1] * 0.996 + b[1] * 0.004];
            let d = p[0].hypot(p[1]);
            vertices.push([p[0] * (1. - 1e-8 / d), p[1] * (1. - 1e-8 / d), 0.]);
            area += (a[0] * b[1] - a[1] * b[0]) / 2.;
            perimeter += (a[0] - b[0]).hypot(a[1] - b[1]);
        }
        let source = PlanarRegion {
            version: 1,
            units: "mm".into(),
            name: "faceted".into(),
            source: serde_json::Value::Null,
            boundary_error_mm: 0.,
            components: vec![Component {
                outer: Boundary {
                    vertices: vec![
                        [-20., -20., 0.],
                        [20., -20., 0.],
                        [20., 20., 0.],
                        [-20., 20., 0.],
                    ],
                },
                holes: vec![Boundary { vertices }],
            }],
        };
        let p = source.prepare(ConstructionOptions::default()).unwrap();
        let old = p.definition.clone();
        let offset = p.offset_prepared(1.5).unwrap();
        // Steiner's area formula for a convex polygon dilated by a disk;
        // independently accounts for all straight facets and round joins.
        let expected =
            37f64.powi(2) - (area + 1.5 * perimeter + std::f64::consts::PI * 1.5f64.powi(2));
        assert!((offset.region.summary().area_mm2 - expected).abs() < 1e-6);
        assert!(offset.report.normalized_vertices >= n);
        assert!(offset.report.normalization_error_mm > 0.);
        assert!(offset.region.summary().arc_segments >= n);
        assert_eq!(p.definition, old);
        assert_eq!(
            offset.region.definition.boundary_error_mm,
            p.definition.boundary_error_mm
                + p.options.tolerance_mm
                + offset.report.normalization_error_mm
        );
    }
}
