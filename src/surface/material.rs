//! Whole-rectangle upper-envelope bounds, independent of cutter paths.
use super::*;
use std::collections::BTreeMap;

type EdgeKey = ([u64; 2], [u64; 2]);

fn key(p: Vec3) -> [u64; 2] {
    [p[0], p[1]].map(|x| if x == 0. { 0 } else { x.to_bits() })
}
fn point(p: [u64; 2]) -> Vec3 {
    [f64::from_bits(p[0]), f64::from_bits(p[1]), 0.]
}
fn edge_meets_box(a: Vec3, b: Vec3, r: [f64; 4]) -> bool {
    if a[0].max(b[0]) < r[0]
        || a[0].min(b[0]) > r[2]
        || a[1].max(b[1]) < r[1]
        || a[1].min(b[1]) > r[3]
    {
        return false;
    }
    let corners = [
        [r[0], r[1], 0.],
        [r[2], r[1], 0.],
        [r[2], r[3], 0.],
        [r[0], r[3], 0.],
    ];
    // The segment's supporting line misses this convex rectangle iff all
    // corners lie strictly on one side. Combined with the AABB this is a
    // conservative intersection test (false positives only).
    let signs = corners.map(|p| orientation(a, b, p, 2));
    !(signs.iter().all(|s| *s > 0.) || signs.iter().all(|s| *s < 0.))
}
/// Whether the XY projection of facet `p` (exact orientation `area` of its
/// projection) meets the closed box `r`, given that their bounding boxes
/// overlap. Exact predicates: false only when one of the facet's edge lines
/// strictly separates the box (with the box axes, the complete separating-axis
/// test for two convex polygons). A degenerate (vertical) facet projects to
/// the segments of its edges.
fn projection_meets_box(p: [Vec3; 3], area: f64, r: [f64; 4]) -> bool {
    if area == 0. {
        return (0..3).any(|j| edge_meets_box(p[j], p[(j + 1) % 3], r));
    }
    let corners = [
        [r[0], r[1], 0.],
        [r[2], r[1], 0.],
        [r[2], r[3], 0.],
        [r[0], r[3], 0.],
    ];
    (0..3).all(|j| {
        let (a, b) = (p[j], p[(j + 1) % 3]);
        // The facet lies on the `area` side of each of its edge lines.
        !corners.iter().all(|&c| {
            let side = orientation(a, b, c, 2);
            side != 0. && (side > 0.) != (area > 0.)
        })
    })
}
fn charge(work: &mut u64, limit: u64, amount: u64) -> bool {
    if amount > limit.saturating_sub(*work) {
        return false;
    }
    *work += amount;
    true
}

impl PreparedSurface {
    pub(crate) fn material_bounds(
        &self,
        xy: [f64; 4],
        tolerance: f64,
        work: &mut u64,
        limit: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Option<[f64; 2]>, String> {
        let r = [
            xy[0] - tolerance,
            xy[1] - tolerance,
            xy[2] + tolerance,
            xy[3] + tolerance,
        ];
        let mut stack = vec![0];
        let mut candidates = Vec::new();
        let mut upper = self.floor;
        while let Some(i) = stack.pop() {
            if cancel.is_cancelled() {
                return Err("surface material check cancelled".into());
            }
            if !charge(work, limit, 1) {
                return Ok(None);
            }
            let node = &self.nodes[i];
            if node.max[0] < r[0] || node.min[0] > r[2] || node.max[1] < r[1] || node.min[1] > r[3]
            {
                continue;
            }
            if let Some(children) = node.children {
                stack.extend(children);
                continue;
            }
            for i in node.range.clone() {
                if !charge(work, limit, 1) {
                    return Ok(None);
                }
                let t = &self.triangles[i];
                if t.max[0] < r[0] || t.min[0] > r[2] || t.max[1] < r[1] || t.min[1] > r[3] {
                    continue;
                }
                let area = orientation(t.p[0], t.p[1], t.p[2], 2);
                let mut lo = t.min[2];
                let mut hi = t.max[2];
                if area != 0. {
                    let d = sub(t.p[1], t.p[0]);
                    let e = sub(t.p[2], t.p[0]);
                    let gx = (d[2] * e[1] - e[2] * d[1]) / area;
                    let gy = (d[0] * e[2] - e[0] * d[2]) / area;
                    let scale = 1.
                        + t.p.iter().flatten().map(|x| x.abs()).sum::<f64>()
                        + r.iter().map(|x| x.abs()).sum::<f64>();
                    let condition =
                        (d[0].abs() * e[1].abs() + e[0].abs() * d[1].abs()) / area.abs();
                    let guard = (tolerance + 512. * f64::EPSILON * scale * (1. + condition))
                        * (1. + gx.abs() + gy.abs());
                    let z = [[r[0], r[1]], [r[0], r[3]], [r[2], r[1]], [r[2], r[3]]]
                        .map(|p| t.p[0][2] + gx * (p[0] - t.p[0][0]) + gy * (p[1] - t.p[0][1]));
                    if guard.is_finite() && z.iter().all(|v| v.is_finite()) {
                        lo = lo.max(z.iter().copied().fold(f64::INFINITY, f64::min) - guard);
                        hi = hi.min(z.iter().copied().fold(f64::NEG_INFINITY, f64::max) + guard);
                    }
                }
                if lo > hi {
                    continue;
                } // affine plane excludes this AABB candidate
                // Only a facet whose projection meets the box bounds the
                // surface over it. A facet beside the box whose bounding box
                // overlaps it (a top face around a through hole) does not:
                // counting it raised the bound over the hole to the top face.
                if projection_meets_box(t.p, area, r) {
                    upper = upper.max(hi);
                }
                if area != 0. {
                    candidates.push((lo, i));
                }
            }
        }
        candidates.sort_unstable_by(|a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)));
        let mut edges: BTreeMap<EdgeKey, u8> = BTreeMap::new();
        let center = [(r[0] + r[2]) * 0.5, (r[1] + r[3]) * 0.5, 0.];
        let mut covered_center = false;
        for (lower, i) in candidates {
            if cancel.is_cancelled() {
                return Err("surface material check cancelled".into());
            }
            if !charge(work, limit, 3) {
                return Ok(None);
            }
            let t = &self.triangles[i];
            covered_center |= inside(t.p, center, 2);
            for j in 0..3 {
                let (a, b) = (key(t.p[j]), key(t.p[(j + 1) % 3]));
                let (a, b) = if a < b { (a, b) } else { (b, a) };
                let side = orientation(point(a), point(b), t.p[(j + 2) % 3], 2);
                *edges.entry((a, b)).or_default() |= if side > 0. { 1 } else { 2 };
            }
            if !covered_center {
                continue;
            }
            let mut boundary = false;
            for (&(a, b), &sides) in &edges {
                if !charge(work, limit, 1) {
                    return Ok(None);
                }
                if sides != 3 && edge_meets_box(point(a), point(b), r) {
                    boundary = true;
                    break;
                }
            }
            // Every boundary of this union is among the uncancelled edges.
            // Shared edges cancel only with triangles on BOTH sides, using
            // exact endpoint identity and robust orientation; no small gaps or
            // projected facets are rounded away. A connected box with one
            // covered point and no possible boundary crossing is fully covered.
            if !boundary {
                return Ok(Some([self.floor.max(lower), upper]));
            }
        }
        Ok(Some([self.floor, upper]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicBool;

    /// A 20 x 20 plate, top Z10, base Z0, with a diamond hole around (10, 10)
    /// (corners 4 mm from its centre) whose floor is `hole_floor` (0: through).
    /// The bounding box of the top facet (0,0)-(20,0)-(14,10) overlaps the
    /// hole, while the facet itself ends at the hole's edge.
    fn plate(hole_floor: f64) -> PreparedSurface {
        let o = [[0., 0.], [20., 0.], [20., 20.], [0., 20.]];
        let i = [[10., 6.], [14., 10.], [10., 14.], [6., 10.]];
        let at = |p: [f64; 2], z: f64| [p[0], p[1], z];
        let mut triangles = vec![];
        for k in 0..4 {
            let n = (k + 1) % 4;
            triangles.push([at(o[k], 10.), at(o[n], 10.), at(i[n], 10.)]);
            triangles.push([at(o[k], 10.), at(i[n], 10.), at(i[k], 10.)]);
            // Hole walls (vertical), down to the hole floor.
            triangles.push([at(i[k], 10.), at(i[n], 10.), at(i[n], hole_floor)]);
            triangles.push([at(i[k], 10.), at(i[n], hole_floor), at(i[k], hole_floor)]);
        }
        if hole_floor > 0. {
            triangles.push([
                at(i[0], hole_floor),
                at(i[1], hole_floor),
                at(i[2], hole_floor),
            ]);
            triangles.push([
                at(i[0], hole_floor),
                at(i[2], hole_floor),
                at(i[3], hole_floor),
            ]);
        }
        SurfaceMesh {
            version: 1,
            units: "mm".into(),
            triangles: triangles.into(),
            surface_error_mm: 0.,
            source: serde_json::Value::Null,
        }
        .prepare(0., 1e-5, &AtomicBool::new(false))
        .unwrap()
    }
    fn bounds(surface: &PreparedSurface, xy: [f64; 4]) -> [f64; 2] {
        surface
            .material_bounds(xy, 1e-5, &mut 0, u64::MAX, &AtomicBool::new(false))
            .unwrap()
            .unwrap()
    }

    #[test]
    fn a_cell_inside_a_through_hole_is_bounded_by_the_base_not_the_top_face() {
        let through = plate(0.);
        // Wholly inside the hole: no facet of the part above it.
        assert_eq!(bounds(&through, [10.5, 9., 11.5, 9.8]), [0., 0.]);
        assert_eq!(bounds(&through, [9., 9., 11., 11.]), [0., 0.]);
        // Touching or straddling a hole wall: the wall and top face count.
        assert_eq!(bounds(&through, [12., 9., 13., 10.])[1], 10.);
        assert_eq!(bounds(&through, [12.5, 8., 13.5, 9.])[1], 10.);
        // On the plate: the top face.
        assert_eq!(bounds(&through, [1., 1., 2., 2.])[1], 10.);
    }

    #[test]
    fn a_blind_hole_keeps_its_floor_as_the_bound() {
        let blind = plate(5.);
        let inside = bounds(&blind, [10.5, 9., 11.5, 9.8]);
        assert!(inside[1] >= 5., "{inside:?}");
        assert!(inside[1] < 10., "{inside:?}");
    }

    #[test]
    fn the_projection_test_never_separates_a_touching_facet() {
        let t = [[0., 0., 1.], [4., 0., 1.], [0., 4., 1.]];
        let area = orientation(t[0], t[1], t[2], 2);
        // Box touching the hypotenuse at one point, beyond it, and across.
        assert!(projection_meets_box(t, area, [2., 2., 3., 3.]));
        assert!(!projection_meets_box(t, area, [2.0001, 2.0001, 3., 3.]));
        assert!(projection_meets_box(t, area, [1., 1., 3., 3.]));
        // Either winding.
        let r = [t[0], t[2], t[1]];
        assert!(projection_meets_box(r, -area, [2., 2., 3., 3.]));
        assert!(!projection_meets_box(r, -area, [2.0001, 2.0001, 3., 3.]));
        // A vertical facet projects to a segment.
        let wall = [[0., 0., 0.], [4., 4., 0.], [4., 4., 5.]];
        assert!(projection_meets_box(wall, 0., [1.5, 1.5, 2.5, 2.5]));
        assert!(!projection_meets_box(wall, 0., [2.5, 0., 3.5, 1.]));
    }
}
