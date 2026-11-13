//! The part shadow is closed by the mesh's declared surface error (#241).
//!
//! A mesh whose neighbouring faces were tessellated separately has hairline
//! seams along their shared edges. Projected, they are slivers and
//! near-touching boundaries the planar kernel rightly refuses as ambiguous.
//! A seam below the mesh's own declared `surface_error_mm` says nothing about
//! the part, so the shadow is closed by that error: gaps narrower than twice
//! the error are bridged, and nothing is removed.
//!
//! Checked with geometry that uses neither the planar kernel nor Clipper:
//! exact orientation predicates on the facets and on the shadow's own
//! vertices (every point of every facet inside the shadow, not only facet
//! vertices, to the kernel's shortest edge), and point-to-triangle distances
//! for how far the shadow grew.
use camlib::{
    planar::{ConstructionOptions, PlanarRegion},
    roughing::PreparedRoughingTarget,
    silhouette::silhouette_region,
    surface::SurfaceMesh,
};
use robust::{Coord, orient2d};
use std::f64::consts::TAU;
use std::sync::atomic::AtomicBool;

type P2 = [f64; 2];
type P3 = [f64; 3];

fn mesh(triangles: Vec<[P3; 3]>, error: f64) -> SurfaceMesh {
    SurfaceMesh {
        version: 1,
        units: "mm".into(),
        triangles: triangles.into(),
        surface_error_mm: error,
        source: serde_json::json!({"model":"shadow closing test part"}),
    }
}

/// A drafted boss in two faces that share the circle of radius 6 at Z4 and
/// were tessellated on their own: the wall (radius 10 at Z0 to 6 at Z4) with
/// `n` chords of that circle, the crown (radius 6 at Z4 to the apex at Z6)
/// with `m` chords, turned by `twist` of a turn. Every vertex is on the true
/// circle, so the faces differ by at most the larger chord's sagitta,
/// `6 * (1 - cos(pi / min(n, m)))`: 0.91 um for 180 and 240. With a twist
/// of a few 1e-8 of a turn the two faces' vertices are nanometres apart
/// where they should be one: the slivers between them have an edge below
/// the planar kernel's shortest, which is what it refuses.
fn two_faces(n: usize, m: usize, twist: f64) -> Vec<[P3; 3]> {
    let at = |r: f64, turn: f64, z: f64| [r * (TAU * turn).cos(), r * (TAU * turn).sin(), z];
    let mut triangles = vec![];
    for i in 0..n {
        let (a, b) = (i as f64 / n as f64, (i + 1) as f64 / n as f64);
        triangles.push([at(10., a, 0.), at(10., b, 0.), at(6., b, 4.)]);
        triangles.push([at(10., a, 0.), at(6., b, 4.), at(6., a, 4.)]);
    }
    for i in 0..m {
        let (a, b) = (i as f64 / m as f64 + twist, (i + 1) as f64 / m as f64 + twist);
        triangles.push([at(6., a, 4.), at(6., b, 4.), [0., 0., 6.]]);
    }
    triangles
}

fn orient(a: P2, b: P2, c: P2) -> f64 {
    orient2d(
        Coord { x: a[0], y: a[1] },
        Coord { x: b[0], y: b[1] },
        Coord { x: c[0], y: c[1] },
    )
}
fn rings(region: &PlanarRegion) -> Vec<Vec<P2>> {
    region
        .components
        .iter()
        .flat_map(|c| std::iter::once(&c.outer).chain(&c.holes))
        .map(|b| {
            b.vertices
                .iter()
                .map(|v| {
                    assert_eq!(v[2], 0., "a shadow has straight edges only");
                    [v[0], v[1]]
                })
                .collect()
        })
        .collect()
}
fn on_segment(p: P2, a: P2, b: P2) -> bool {
    orient(a, b, p) == 0.
        && p[0] >= a[0].min(b[0])
        && p[0] <= a[0].max(b[0])
        && p[1] >= a[1].min(b[1])
        && p[1] <= a[1].max(b[1])
}
/// Inside or on the boundary: even-odd crossings with exact orientations.
fn covers(rings: &[Vec<P2>], p: P2) -> bool {
    let mut inside = false;
    for ring in rings {
        for i in 0..ring.len() {
            let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
            if on_segment(p, a, b) {
                return true;
            }
            if (a[1] > p[1]) != (b[1] > p[1]) {
                let side = orient(a, b, p);
                if (side > 0.) == (b[1] > a[1]) {
                    inside = !inside;
                }
            }
        }
    }
    inside
}
fn distance_to_segment(p: P2, a: P2, b: P2) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1]];
    let l2 = d[0] * d[0] + d[1] * d[1];
    let s = if l2 > 0. {
        (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / l2).clamp(0., 1.)
    } else {
        0.
    };
    (p[0] - a[0] - s * d[0]).hypot(p[1] - a[1] - s * d[1])
}
/// The planar kernel takes no edge shorter than this (1e-6 mm, or a quarter
/// of the tolerance) and merges the ends of one that a construction leaves:
/// a region's vertices are exact to that, and a corner may be cut by it.
const KERNEL: f64 = 2.5e-6;
/// What absorbing a short edge may add beyond the closing distance: a cut
/// corner is put back when it is within four times the absorbed length
/// (eight times the tolerance) of the edge's ends.
const ABSORBED: f64 = 4. * 8e-5 + 1e-5;
/// How deep the part of the triangle outside the region is: 0 when every
/// point of the triangle is in the region (its corners are, no boundary edge
/// crosses one of its edges and no boundary vertex is strictly inside it).
/// Otherwise the part left out is bounded by pieces of the triangle's edges
/// and of the region's boundary, and is no deeper than the farthest of its
/// corners from the other of the two: a corner of the triangle outside the
/// region (its distance to the region's boundary), or a boundary vertex
/// inside the triangle (its distance to the triangle's edges).
fn left_out(rings: &[Vec<P2>], t: [P2; 3]) -> f64 {
    let mut depth = 0f64;
    for p in t {
        if !covers(rings, p) {
            let nearest = rings
                .iter()
                .flat_map(|ring| {
                    (0..ring.len()).map(move |i| (ring[i], ring[(i + 1) % ring.len()]))
                })
                .map(|(a, b)| distance_to_segment(p, a, b))
                .fold(f64::INFINITY, f64::min);
            depth = depth.max(nearest);
        }
    }
    let turn = orient(t[0], t[1], t[2]);
    for ring in rings {
        for &p in ring {
            if turn != 0. && (0..3).all(|k| orient(t[k], t[(k + 1) % 3], p) * turn > 0.) {
                let nearest = (0..3)
                    .map(|k| distance_to_segment(p, t[k], t[(k + 1) % 3]))
                    .fold(f64::INFINITY, f64::min);
                depth = depth.max(nearest);
            }
        }
    }
    depth
}
fn distance_to_triangle(p: P2, t: [P2; 3]) -> f64 {
    let turn = orient(t[0], t[1], t[2]);
    if turn != 0. && (0..3).all(|k| orient(t[k], t[(k + 1) % 3], p) * turn >= 0.) {
        return 0.;
    }
    (0..3)
        .map(|k| distance_to_segment(p, t[k], t[(k + 1) % 3]))
        .fold(f64::INFINITY, f64::min)
}
fn plan(t: &[P3; 3]) -> [P2; 3] {
    [[t[0][0], t[0][1]], [t[1][0], t[1][1]], [t[2][0], t[2][1]]]
}

#[test]
fn sub_error_seams_between_two_faces_are_closed_for_every_consumer() {
    let cancel = AtomicBool::new(false);
    // The three ways a seam was refused (issue #241): a sliver with an edge
    // the kernel cannot remove, one whose removal would sweep another
    // boundary, and boundaries that touch.
    for (n, m, twist, message) in [
        (180, 240, 2e-8, "short edge cannot be removed"),
        (240, 180, 5e-9, "short-edge collapse would sweep another boundary"),
        (180, 180, 1e-7, "short"),
    ] {
        seams_close(n, m, twist, message, &cancel);
    }
}

fn seams_close(n: usize, m: usize, twist: f64, message: &str, cancel: &AtomicBool) {
    let triangles = two_faces(n, m, twist);
    // An exact mesh (error 0) claims its seams are real: still refused, and
    // the refusal says what to declare.
    let exact = PreparedRoughingTarget::prepare(mesh(triangles.clone(), 0.), 1e-5, cancel).unwrap();
    let refused = exact
        .shadow(2., 0., &mut 0, cancel)
        .err()
        .expect("a zero-error mesh with seams is refused");
    assert!(refused.contains(message), "{n} {m}: {refused}");
    assert!(refused.contains("not watertight"), "{refused}");
    assert!(
        refused.contains("surface error of at least half the widest seam"),
        "{refused}"
    );
    // The same mesh declaring 2 um (its seams are 0.91 um): every level
    // builds, with and without a cutter (roughing, waterline, silhouette).
    let error = 0.002;
    let target =
        PreparedRoughingTarget::prepare(mesh(triangles.clone(), error), 1e-5, cancel).unwrap();
    // Below Z4 the two faces overlap and the exact union is refused (above);
    // there the closing is applied and reported. Where the exact union is a
    // valid region it is used as it is, with no closing.
    let mut closed_levels = 0;
    for z in [-1., 0.5, 2., 3.9, 3.999, 4., 4.001, 4.5, 5.9] {
        for radius in [0., 0.5, 3.] {
            let shadow = target
                .shadow_for_cutter(z, 0., radius, &mut 0, cancel)
                .unwrap_or_else(|e| panic!("{n} {m}: Z{z} radius {radius}: {e}"));
            if let Some(closing) = shadow.report.seam_closing {
                closed_levels += 1;
                assert_eq!(closing.distance_mm, error);
                // Lunes between two chord sets of one circle, and the rounding.
                assert!((0. ..0.05).contains(&closing.added_area_mm2), "{closing:?}");
                assert_eq!(shadow.report.filled_holes, 0, "the seams are closed, not holes");
            }
            let region = shadow.region.prepare(ConstructionOptions::default()).unwrap();
            assert_eq!(
                (region.summary().components, region.summary().holes),
                (1, 0),
                "Z{z} radius {radius}"
            );
            let level = shadow.report.protected_from_z_mm;
            let rings = rings(&shadow.region);
            let above: Vec<_> = triangles
                .iter()
                .filter(|t| t.iter().all(|p| p[2] >= level))
                .collect();
            for t in &above {
                let out = left_out(&rings, plan(t));
                assert!(out <= KERNEL, "Z{z}: {t:?} is {out} outside");
            }
            // Nothing is protected farther than the closing distance from a
            // facet at or above the level (the closing of X lies within the
            // distance of X), but for the kernel's rounding.
            let partly: Vec<_> = triangles
                .iter()
                .filter(|t| t.iter().any(|p| p[2] >= level))
                .collect();
            for ring in &rings {
                for &p in ring {
                    let d = partly
                        .iter()
                        .map(|t| distance_to_triangle(p, plan(t)))
                        .fold(f64::INFINITY, f64::min);
                    assert!(
                        d <= error + shadow.report.projection_enclosure_mm + ABSORBED,
                        "Z{z}: {p:?} is {d} from the part"
                    );
                }
            }
        }
        let silhouette = silhouette_region(
            &mesh(triangles.clone(), error),
            z,
            0.01,
            ConstructionOptions::default(),
            cancel,
        )
        .unwrap_or_else(|e| panic!("silhouette at Z{z}: {e}"));
        assert_eq!((silhouette.report.components, silhouette.report.holes), (1, 0));
    }
    assert!(closed_levels >= 3, "{n} {m}: closing reported at {closed_levels} levels");
}

/// xorshift64*: the tests need repeatable shapes, not statistics.
struct Random(u64);
impl Random {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        (self.0.wrapping_mul(0x2545F4914F6CDD1D) >> 11) as f64 / (1u64 << 53) as f64
    }
}

#[test]
fn the_closed_shadow_contains_every_point_of_every_facet_over_random_sharp_meshes() {
    // Overlapping facets with sharp corners (half angles from 1 degree): the
    // case a round-joined dilation loses the tips of (PR #255's review
    // measured up to 27 um over such meshes). No simplification, no cutter.
    let cancel = AtomicBool::new(false);
    let (mut built, mut closed) = (0, 0);
    let mut worst = 0f64;
    for (seed, error) in (1u64..=60).map(|s| (s, [0.001, 0.0272, 0.1][s as usize % 3])) {
        let mut random = Random(seed.wrapping_mul(0x9E3779B97F4A7C15));
        let mut triangles: Vec<[P3; 3]> = vec![];
        for _ in 0..6 + (seed % 7) as usize {
            let tip = [20. * random.next(), 20. * random.next()];
            let heading = TAU * random.next();
            let half = (1. + 29. * random.next().powi(3)).to_radians();
            let length = 4. + 16. * random.next();
            let side = |sign: f64| {
                let a = heading + sign * half;
                [tip[0] + length * a.cos(), tip[1] + length * a.sin(), 1.]
            };
            triangles.push([[tip[0], tip[1], 1.], side(-1.), side(1.)]);
        }
        // A seam as between two tessellations of one face: two plates 0.4 um
        // apart, capped at both ends, so the exact union has a sliver hole
        // and the closing (which joins it) is what builds this shadow.
        let (x, y) = (40. + 10. * random.next(), 10. * random.next());
        let plate = |x0: f64, x1: f64, y0: f64, y1: f64| {
            [
                [[x0, y0, 1.], [x1, y0, 1.], [x1, y1, 1.]],
                [[x0, y0, 1.], [x1, y1, 1.], [x0, y1, 1.]],
            ]
        };
        triangles.extend(plate(x, x + 5., y, y + 10.));
        triangles.extend(plate(x + 5.0004, x + 10., y, y + 10.));
        triangles.extend(plate(x, x + 10., y + 10., y + 11.));
        triangles.extend(plate(x, x + 10., y - 1., y));
        let target =
            PreparedRoughingTarget::prepare(mesh(triangles.clone(), error), 1e-5, &cancel).unwrap();
        // Facets that happen to touch within the tolerance but farther apart
        // than the closing bridges are a refusal, as before; few seeds.
        let Ok(shadow) = target.shadow(0.5, 0., &mut 0, &cancel) else {
            continue;
        };
        built += 1;
        closed += usize::from(shadow.report.seam_closing.is_some());
        assert!(shadow.report.simplified.is_none());
        let rings = rings(&shadow.region);
        for t in &triangles {
            let out = left_out(&rings, plan(t));
            worst = worst.max(out);
            assert!(out <= KERNEL, "seed {seed}, error {error}: {t:?} is {out} outside");
        }
        for ring in &rings {
            for &p in ring {
                let d = triangles
                    .iter()
                    .map(|t| distance_to_triangle(p, plan(t)))
                    .fold(f64::INFINITY, f64::min);
                assert!(
                    d <= error + ABSORBED,
                    "seed {seed}: {p:?} is {d} from the part"
                );
            }
        }
    }
    assert!(built >= 50, "only {built} of 60 random meshes built");
    assert!(closed >= 30, "the closing built only {closed} of {built} shadows");
    println!(
        "{built} of 60 built, {closed} by closing; farthest facet point outside its shadow: {worst} mm"
    );
}

#[test]
fn a_sharp_tip_is_kept_and_a_gap_wider_than_twice_the_error_stays_open() {
    let cancel = AtomicBool::new(false);
    // Two-facet wedges of 2, 5, 10 and 22.5 degree half angle, tip at the
    // origin (the review's table: 0.38 um to 15 um lost by round/round).
    for half in [2f64, 5., 10., 22.5] {
        for error in [0.0272, 0.1] {
            let h = half.to_radians();
            let far = |a: f64| [30. * a.cos(), 30. * a.sin(), 1.];
            let triangles = vec![
                [[0., 0., 1.], far(-h), far(0.)],
                [[0., 0., 1.], far(0.), far(h)],
            ];
            let target =
                PreparedRoughingTarget::prepare(mesh(triangles.clone(), error), 1e-5, &cancel)
                    .unwrap();
            let shadow = target.shadow(0.5, 0., &mut 0, &cancel).unwrap();
            let rings = rings(&shadow.region);
            for t in &triangles {
                let out = left_out(&rings, plan(t));
                assert!(
                    out <= KERNEL,
                    "half angle {half}, error {error}: the tip is {out} outside"
                );
            }
            // And the tip is not blunted outwards either: the shadow's
            // nearest vertex to it is within the kernel's rounding.
            let nearest = rings[0]
                .iter()
                .map(|p| p[0].hypot(p[1]))
                .fold(f64::INFINITY, f64::min);
            assert!(nearest < 1e-5, "half angle {half}: tip vertex {nearest} away");
        }
    }
    // Two plates 0.05 apart: one shadow at an error of 0.03 (the gap is
    // within twice the error, and the closing that joins them is reported),
    // two at 0.02 (a real 50 um slot: nothing is joined, so the exact union
    // is used and no closing is reported).
    let plate = |x0: f64, x1: f64| {
        vec![
            [[x0, 0., 1.], [x1, 0., 1.], [x1, 10., 1.]],
            [[x0, 0., 1.], [x1, 10., 1.], [x0, 10., 1.]],
        ]
    };
    let mut triangles = plate(0., 5.);
    triangles.extend(plate(5.05, 10.));
    for (error, components) in [(0.03, 1), (0.02, 2)] {
        let target =
            PreparedRoughingTarget::prepare(mesh(triangles.clone(), error), 1e-5, &cancel).unwrap();
        let shadow = target.shadow(0.5, 0., &mut 0, &cancel).unwrap();
        assert_eq!(shadow.region.components.len(), components, "error {error}");
        match shadow.report.seam_closing {
            // Joined: the 0.05 x 10 slot is what the closing added.
            Some(closing) => {
                assert_eq!(components, 1);
                assert!((closing.added_area_mm2 - 0.5).abs() < 0.01, "{closing:?}");
            }
            // Nothing to join: the exact union, with no closing applied.
            None => assert_eq!(components, 2),
        }
    }
}
