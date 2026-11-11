//! The silhouette of a surface target at a depth (#173), checked against
//! geometry that does not use the planar kernel: an even-odd point test with
//! edge distances on the returned vertices, points sampled on the mesh, and
//! the part's analytic outline at each depth.
use camlib::{
    contour::{ContourOperation, ContourPlan},
    motion::MoveKind,
    planar::{ConstructionOptions, PlanarRegion},
    pocket::Operation,
    silhouette::silhouette_region,
    stock::{Stock, SweepOptions},
    stream::StreamManifest,
    surface::SurfaceMesh,
};
use std::sync::atomic::AtomicBool;

type P3 = [f64; 3];
fn quad(a: P3, b: P3, c: P3, d: P3) -> Vec<[P3; 3]> {
    vec![[a, b, c], [a, c, d]]
}
fn rect_at(r: [f64; 4], z: f64) -> [P3; 4] {
    [
        [r[0], r[1], z],
        [r[2], r[1], z],
        [r[2], r[3], z],
        [r[0], r[3], z],
    ]
}
/// The faces between two rectangles at two heights (walls, vertical or drafted).
fn band(low: [P3; 4], high: [P3; 4]) -> Vec<[P3; 3]> {
    (0..4)
        .flat_map(|i| quad(low[i], low[(i + 1) % 4], high[(i + 1) % 4], high[i]))
        .collect()
}
const BASE: [f64; 4] = [4., 6., 36., 34.];
const BASE_TOP: f64 = 3.;
const FOOT: [f64; 4] = [10., 12., 30., 28.];
const CROWN: [f64; 4] = [14., 16., 26., 24.];
const CROWN_Z: f64 = 9.;
/// A stepped and drafted part: a base slab with vertical walls up to Z3 and
/// on it a boss whose walls draft inward from Z3 to its top at Z9.
fn part(error: f64) -> SurfaceMesh {
    let mut triangles = band(rect_at(BASE, 0.), rect_at(BASE, BASE_TOP));
    let [a, b, c, d] = rect_at(BASE, BASE_TOP);
    triangles.extend(quad(a, b, c, d));
    triangles.extend(band(rect_at(FOOT, BASE_TOP), rect_at(CROWN, CROWN_Z)));
    let [a, b, c, d] = rect_at(CROWN, CROWN_Z);
    triangles.extend(quad(a, b, c, d));
    SurfaceMesh {
        version: 1,
        units: "mm".into(),
        triangles: triangles.into(),
        surface_error_mm: error,
        source: serde_json::json!({"model":"stepped drafted boss"}),
    }
}
/// The part's exact shadow at or above `z`: one rectangle.
fn outline(z: f64) -> Option<[f64; 4]> {
    if z <= BASE_TOP {
        Some(BASE)
    } else if z <= CROWN_Z {
        let t = (z - BASE_TOP) / (CROWN_Z - BASE_TOP);
        Some(std::array::from_fn(|i| {
            FOOT[i] + t * (CROWN[i] - FOOT[i])
        }))
    } else {
        None
    }
}
/// Signed distance to a rectangle: negative inside.
fn rect_distance(r: [f64; 4], p: [f64; 2]) -> f64 {
    let dx = (r[0] - p[0]).max(p[0] - r[2]);
    let dy = (r[1] - p[1]).max(p[1] - r[3]);
    if dx > 0. && dy > 0. {
        dx.hypot(dy)
    } else {
        dx.max(dy)
    }
}
/// How far `p` is outside the region (0 inside): even-odd crossings and
/// distances to the boundary segments, on the region's own vertices.
fn outside_by(region: &PlanarRegion, p: [f64; 2]) -> f64 {
    let mut inside = false;
    let mut nearest = f64::INFINITY;
    for c in &region.components {
        for ring in std::iter::once(&c.outer).chain(&c.holes) {
            let v = &ring.vertices;
            for (a, b) in v.iter().zip(v.iter().cycle().skip(1)) {
                assert_eq!(a[2], 0., "a silhouette has straight edges only");
                if (a[1] > p[1]) != (b[1] > p[1])
                    && p[0] < a[0] + (p[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0])
                {
                    inside = !inside;
                }
                let d = [b[0] - a[0], b[1] - a[1]];
                let l2 = d[0] * d[0] + d[1] * d[1];
                let t = if l2 > 0. {
                    (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / l2).clamp(0., 1.)
                } else {
                    0.
                };
                nearest = nearest.min((p[0] - a[0] - t * d[0]).hypot(p[1] - a[1] - t * d[1]));
            }
        }
    }
    if inside { 0. } else { nearest }
}
/// Points on every facet (corners, edges and interior).
fn samples(mesh: &SurfaceMesh) -> Vec<P3> {
    const N: usize = 12;
    let mut points = vec![];
    for t in mesh.triangles.iter() {
        for i in 0..=N {
            for j in 0..=N - i {
                let (u, v) = (i as f64 / N as f64, j as f64 / N as f64);
                points.push(std::array::from_fn(|k| {
                    t[0][k] + u * (t[1][k] - t[0][k]) + v * (t[2][k] - t[0][k])
                }));
            }
        }
    }
    points
}
fn never() -> AtomicBool {
    AtomicBool::new(false)
}

#[test]
fn the_silhouette_holds_everything_at_or_above_each_depth_and_no_more() {
    for error in [0., 0.002] {
        let mesh = part(error);
        let points = samples(&mesh);
        for z in [-1., 0., 2.999, 3., 3.001, 4.5, 6., 8.999, 9.] {
            let s = silhouette_region(&mesh, z, 0.01, ConstructionOptions::default(), &never())
                .unwrap();
            let bound = s.report.boundary_error_mm;
            assert_eq!(bound, s.region.boundary_error_mm);
            assert!(bound >= error && bound < error + 0.001, "{bound}");
            assert_eq!(s.report.source_error_mm, error);
            assert_eq!(s.region.name, "silhouette");
            assert_eq!(s.region.source["adapter"], "surface_silhouette_v1");
            assert_eq!(s.region.source["z_mm"], z);
            assert_eq!((s.report.components, s.report.holes), (1, 0));
            // Every mesh point at or above the depth is in the region. (The
            // intended surface is within the declared error of one of them,
            // and that error is part of the stated bound.)
            let mut checked = 0;
            for p in points.iter().filter(|p| p[2] >= z) {
                let out = outside_by(&s.region, [p[0], p[1]]);
                assert!(out <= bound - error, "{p:?} is {out} outside at Z{z}");
                checked += 1;
            }
            assert!(checked > 100);
            // And it is no bigger than the exact shadow of the level it
            // states, grown by the bound.
            // (A depth within the error of a horizontal face has that face.)
            assert!(z - s.report.protected_from_z_mm <= bound);
            let exact = outline(s.report.protected_from_z_mm).unwrap();
            for c in &s.region.components {
                assert!(c.holes.is_empty());
                for v in &c.outer.vertices {
                    assert!(rect_distance(exact, [v[0], v[1]]) <= bound, "{v:?} at Z{z}");
                }
            }
            let ideal = (exact[2] - exact[0]) * (exact[3] - exact[1]);
            assert!((s.report.area_mm2 - ideal).abs() < 0.2, "area at Z{z}");
        }
    }
}

#[test]
fn an_overhang_above_the_depth_is_in_the_silhouette_though_no_section_meets_it() {
    // A post from Z0 to Z10 carrying a wider cap from Z8 to Z10.
    let post = [18., 18., 22., 22.];
    let cap = [12., 14., 28., 26.];
    let mut triangles = band(rect_at(post, 0.), rect_at(post, 8.));
    triangles.extend(band(rect_at(cap, 8.), rect_at(cap, 10.)));
    for (r, z) in [(cap, 8.), (cap, 10.)] {
        let [a, b, c, d] = rect_at(r, z);
        triangles.extend(quad(a, b, c, d));
    }
    let mesh = SurfaceMesh {
        version: 1,
        units: "mm".into(),
        triangles: triangles.into(),
        surface_error_mm: 0.,
        source: serde_json::Value::Null,
    };
    let s = silhouette_region(&mesh, 4., 0.01, ConstructionOptions::default(), &never()).unwrap();
    // The section at Z4 is the 4 mm post; the silhouette is the cap.
    for p in [[12.5, 14.5], [27.5, 25.5], [20., 20.]] {
        assert_eq!(outside_by(&s.region, p), 0.);
    }
    assert!(outside_by(&s.region, [11.9, 20.]) > 0.09);
    assert!((s.report.area_mm2 - 16. * 12.).abs() < 0.1);
}

#[test]
fn what_cannot_be_stated_within_the_requested_error_is_refused() {
    let options = ConstructionOptions::default();
    let refused = |mesh: &SurfaceMesh, z: f64, max: f64| {
        silhouette_region(mesh, z, max, options, &never())
            .err()
            .unwrap()
    };
    // The declared mesh error is part of the bound.
    let e = refused(&part(0.02), 5., 0.01);
    assert!(
        e.contains("exceeds max_error_mm 0.01") && e.contains("surface_error_mm 0.02"),
        "{e}"
    );
    assert!(silhouette_region(&part(0.02), 5., 0.025, options, &never()).is_ok());
    // Nothing at or above the depth.
    let e = refused(&part(0.), 9.5, 0.01);
    assert!(e.contains("nothing at or above"), "{e}");
    for (z, max) in [(f64::NAN, 0.01), (5., 0.), (5., f64::NAN), (5., -1.)] {
        refused(&part(0.), z, max);
    }
    assert!(silhouette_region(&part(0.), 5., 0.01, options, &AtomicBool::new(true)).is_err());
    let mut bad = part(0.);
    bad.surface_error_mm = 0.5;
    assert!(refused(&bad, 5., 1.).contains("surface_error_mm"));
}

#[test]
fn contours_cut_from_each_depths_silhouette_stay_beside_the_part() {
    let manifest: StreamManifest =
        serde_json::from_str(include_str!("../fixtures/operations/contour.json")).unwrap();
    let Operation::Contour(template) = manifest.operations[0].clone() else {
        panic!("fixture")
    };
    let spec = manifest.stock.height().unwrap();
    let tool = manifest.tools[0].prepare().unwrap();
    let radius = 2.;
    let mesh = part(0.001);
    let cancel = never();
    let mut stock = Stock::new(spec.clone()).unwrap();
    let mut first_id = 1;
    let mut position = manifest.initial_position_mm;
    let mut cuts = 0;
    // Z3.01: beside the boss, just above the base's top face at Z3.
    for floor in [7., 5., 3.01, 0.] {
        let s = silhouette_region(&mesh, floor, 0.0025, ConstructionOptions::default(), &cancel)
            .unwrap();
        let op = ContourOperation {
            id: format!("contour-z{floor}"),
            region: s.region,
            floor_z_mm: floor,
            stepdown_mm: 4.,
            tabs: None,
            ..template.clone()
        };
        let mut plan =
            ContourPlan::prepare(op, &manifest.tools[0], spec, position, first_id, &cancel)
                .unwrap();
        assert_eq!(plan.report().loops, 1);
        while let Some(m) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
            first_id = m.id + 1;
            position = m.curve.end();
            if m.kind == MoveKind::Cut {
                let curve = m.curve.prepare(1e-5).unwrap();
                for i in 0..=32 {
                    let p = curve.position(i as f64 / 32.).unwrap();
                    // The cutter stands over its tip: it must clear the
                    // part's exact outline at the tip's height (the part
                    // only narrows upward), by the cutter radius.
                    let clear = rect_distance(outline(p[2]).unwrap(), [p[0], p[1]]) - radius;
                    assert!(clear >= 0., "tip {p:?} is {clear} into the part");
                    // And it follows that depth's outline, not a wider one.
                    if (p[2] - floor).abs() < 1e-9 {
                        assert!(clear < 0.02, "tip {p:?} is {clear} from the part");
                        cuts += 1;
                    }
                }
            }
            let d = stock
                .preview_sweep(&tool, &m, 1., SweepOptions::default(), &cancel)
                .unwrap();
            stock.apply_delta(&d).unwrap();
        }
    }
    assert!(cuts > 100);
    // One region for every depth, as Contour took before: cut to the floor
    // with the silhouette of the boss's crown, it gouges the boss below it.
    let crown = silhouette_region(&mesh, 8.5, 0.0025, ConstructionOptions::default(), &cancel)
        .unwrap()
        .region;
    let foot = outline(3.).unwrap();
    assert!(
        crown.components[0]
            .outer
            .vertices
            .iter()
            .all(|v| rect_distance(foot, [v[0], v[1]]) < -3.)
    );
}
