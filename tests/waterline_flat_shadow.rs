//! The part shadow a flat or bull-nose waterline level is offset from (#203).
//!
//! Waterline used to build that shadow itself and dropped a clipped facet the
//! planar kernel refused: a facet with a vertex on the level (or within the
//! kernel's shortest edge of it) and the rest above vanished whole, and the
//! level's contour then ran through the part. It now uses the roughing
//! shadow (`PreparedRoughingTarget::shadow_for_cutter`), which encloses
//! degenerate projections and refuses what it cannot take.
//!
//! Checked against geometry that uses neither the planar kernel nor the
//! planner: points sampled on the mesh, an even-odd point test on the
//! shadow's own vertices, and the cutter's profile at each sampled tool
//! position.
use camlib::{
    motion::{Curve, MoveKind},
    planar::PlanarRegion,
    pocket::Operation,
    roughing::PreparedRoughingTarget,
    stock::Stock,
    stream::StreamManifest,
    surface::SurfaceMesh,
    waterline::WaterlinePlan,
};
use std::sync::atomic::AtomicBool;

type P3 = [f64; 3];
const TOLERANCE: f64 = 1e-5;
const SLAB_TOP: f64 = 2.;
/// The boss's footprint on the slab.
const FOOT: [[f64; 2]; 4] = [[10., 8.], [30., 8.], [30., 22.], [10., 22.]];

fn quad(a: P3, b: P3, c: P3, d: P3) -> Vec<[P3; 3]> {
    vec![[a, b, c], [a, c, d]]
}
fn mesh(triangles: Vec<[P3; 3]>) -> SurfaceMesh {
    SurfaceMesh {
        version: 1,
        units: "mm".into(),
        triangles: triangles.into(),
        surface_error_mm: 0.,
        source: serde_json::json!({"model":"waterline shadow test part"}),
    }
}
fn slab() -> Vec<[P3; 3]> {
    quad(
        [0., 0., SLAB_TOP],
        [40., 0., SLAB_TOP],
        [40., 30., SLAB_TOP],
        [0., 30., SLAB_TOP],
    )
}
/// Vertical walls from the slab up to `top` over a rectangle.
fn walls(r: [[f64; 2]; 4], top: impl Fn(f64, f64) -> f64) -> Vec<[P3; 3]> {
    (0..4)
        .flat_map(|i| {
            let (a, b) = (r[i], r[(i + 1) % 4]);
            quad(
                [a[0], a[1], SLAB_TOP],
                [b[0], b[1], SLAB_TOP],
                [b[0], b[1], top(b[0], b[1])],
                [a[0], a[1], top(a[0], a[1])],
            )
        })
        .collect()
}
/// The tilted top of the boss: its corners stand at Z6, 8, 11.5 and 9.5, its
/// middle (20, 15) at 8.75.
fn tilt(x: f64, y: f64) -> f64 {
    6. + 0.1 * (x - 10.) + 0.25 * (y - 8.)
}
const MIDDLE: f64 = 8.75;
/// A slab with a boss on it: vertical walls (every facet edge-on in plan)
/// and a top following `top`, two facets, or with `centre` a fan of four
/// around a vertex at (20, 15, centre).
fn boss(top: impl Fn(f64, f64) -> f64 + Copy, centre: Option<f64>) -> SurfaceMesh {
    let mut t = slab();
    t.extend(walls(FOOT, top));
    let p = |i: usize| [FOOT[i][0], FOOT[i][1], top(FOOT[i][0], FOOT[i][1])];
    match centre {
        None => t.extend(quad(p(0), p(1), p(2), p(3))),
        Some(z) => t.extend((0..4).map(|i| [p(i), p((i + 1) % 4), [20., 15., z]])),
    }
    mesh(t)
}
/// A fin `thickness` thick standing on the slab along y = 15: vertical
/// walls, and a top that is a sliver.
fn fin(thickness: f64) -> SurfaceMesh {
    let r = [
        [10., 15.],
        [30., 15.],
        [30., 15. + thickness],
        [10., 15. + thickness],
    ];
    let mut t = slab();
    t.extend(walls(r, |_, _| 8.));
    t.extend(quad(
        [10., 15., 8.],
        [30., 15., 8.],
        [30., 15. + thickness, 8.],
        [10., 15. + thickness, 8.],
    ));
    mesh(t)
}
/// A single vertical sheet on the slab: nothing but edge-on facets.
fn sheet() -> SurfaceMesh {
    let mut t = slab();
    t.extend(quad(
        [10., 15., SLAB_TOP],
        [30., 15., SLAB_TOP],
        [30., 15., 8.],
        [10., 15., 8.],
    ));
    mesh(t)
}
/// A ramp rising along x whose face holds a strip of needle facets `width`
/// wide.
fn needles(width: f64) -> SurfaceMesh {
    let z = |x: f64| SLAB_TOP + 0.5 * (x - 10.);
    let ys = [8., 15., 15. + width, 22.];
    let mut t = slab();
    for k in 0..3 {
        t.extend(quad(
            [10., ys[k], z(10.)],
            [30., ys[k], z(30.)],
            [30., ys[k + 1], z(30.)],
            [10., ys[k + 1], z(10.)],
        ));
    }
    mesh(t)
}
/// Points on every facet (corners, edges and interior).
fn samples(mesh: &SurfaceMesh) -> Vec<P3> {
    const N: usize = 16;
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
/// How far `p` is outside the region (0 inside): even-odd crossings and
/// distances to the boundary segments, on the region's own vertices.
fn outside_by(region: &PlanarRegion, p: [f64; 2]) -> f64 {
    let mut inside = false;
    let mut nearest = f64::INFINITY;
    for c in &region.components {
        for ring in std::iter::once(&c.outer).chain(&c.holes) {
            let v = &ring.vertices;
            for (a, b) in v.iter().zip(v.iter().cycle().skip(1)) {
                assert_eq!(a[2], 0., "a shadow has straight edges only");
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
fn never() -> AtomicBool {
    AtomicBool::new(false)
}
/// The level the shadow of a cutter tip at `tip` is taken at (the tip less
/// the numerical reserve; these meshes declare no error).
fn level_of(tip: f64) -> f64 {
    PreparedRoughingTarget::prepare(boss(tilt, None), TOLERANCE, &never())
        .unwrap()
        .shadow(tip, 0., &mut 0, &never())
        .unwrap()
        .report
        .protected_from_z_mm
}

/// The shadow at `tip` holds every sampled mesh point at or above its level
/// within its stated bound, and nothing farther than `reach` from such a
/// point.
fn assert_shadow_holds_the_part(name: &str, mesh: &SurfaceMesh, tip: f64, level: Option<f64>) {
    let cancel = never();
    let target = PreparedRoughingTarget::prepare(mesh.clone(), TOLERANCE, &cancel).unwrap();
    let shadow = target.shadow(tip, 0., &mut 0, &cancel).unwrap();
    if let Some(level) = level {
        assert_eq!(
            shadow.report.protected_from_z_mm, level,
            "{name}: the case needs this level exactly"
        );
    }
    let level = shadow.report.protected_from_z_mm;
    let bound = shadow.region.boundary_error_mm;
    assert!(bound <= 100. * TOLERANCE, "{name}: bound {bound}");
    let above: Vec<P3> = samples(mesh)
        .into_iter()
        .filter(|p| p[2] >= level)
        .collect();
    assert!(!above.is_empty(), "{name}: nothing above the level");
    for p in &above {
        let d = outside_by(&shadow.region, [p[0], p[1]]);
        assert!(
            d <= bound,
            "{name}: mesh point {p:?} at or above Z{level} is {d} mm outside the shadow (bound {bound})"
        );
    }
    // No more than the part: every shadow vertex is beside a sampled point
    // at or above the level (the samples are at most 1.6 mm apart).
    for c in &shadow.region.components {
        for v in std::iter::once(&c.outer)
            .chain(&c.holes)
            .flat_map(|b| &b.vertices)
        {
            let nearest = above
                .iter()
                .map(|p| (p[0] - v[0]).hypot(p[1] - v[1]))
                .fold(f64::INFINITY, f64::min);
            assert!(
                nearest <= 1.6,
                "{name}: shadow vertex {v:?} is {nearest} mm from the part above Z{level}"
            );
        }
    }
}

#[test]
fn the_shadow_holds_edge_on_and_sliver_facets_at_the_level() {
    // Vertical walls only at the level: the top facets cover their footprint.
    assert_shadow_holds_the_part("boss, vertical walls", &boss(|_, _| 8., None), 5., None);
    // A tilted top cut by the level, and levels through its corners.
    for tip in [7., 9., 11.] {
        assert_shadow_holds_the_part("tilted boss", &boss(tilt, None), tip, None);
    }
    // A fan vertex exactly on the level, and a hair above and below it: the
    // facets beside it clip to polygons with an edge shorter than the planar
    // kernel takes (1e-6 mm), or to a duplicated vertex.
    let level = level_of(MIDDLE);
    for offset in [0., 1e-9, -1e-9, 1e-8, -1e-8, 1e-7, -1e-7, 1e-6, -1e-6, 1e-5] {
        assert_shadow_holds_the_part(
            &format!("fan vertex {offset:e} mm from the level"),
            &boss(tilt, Some(level + offset)),
            MIDDLE,
            Some(level),
        );
    }
    // A corner of the two-facet top exactly on the level.
    let low = level_of(8.);
    let lifted = |x: f64, y: f64| tilt(x, y) + low - 8.;
    assert_shadow_holds_the_part("corner on the level", &boss(lifted, None), 8., Some(low));
    // Fins down to a sliver top below the kernel's shortest edge, and a
    // sheet with no thickness at all: every facet above the level is edge-on
    // or a sliver.
    for thickness in [1e-3, 1e-5, 1e-6, 1e-7] {
        assert_shadow_holds_the_part(
            &format!("fin {thickness:e} mm thick"),
            &fin(thickness),
            5.,
            None,
        );
    }
    assert_shadow_holds_the_part("sheet", &sheet(), 5., None);
    // Needle facets across the level on a sloped face.
    for width in [1e-4, 1e-6, 1e-7] {
        assert_shadow_holds_the_part(
            &format!("needle strip {width:e} mm wide"),
            &needles(width),
            7.,
            None,
        );
    }
}

fn manifest(shape: serde_json::Value) -> StreamManifest {
    let mut m: serde_json::Value =
        serde_json::from_str(include_str!("../fixtures/operations/waterline.json")).unwrap();
    m["tools"][0]["cutter"]["shape"] = shape;
    m["stock"]["max_mm"] = serde_json::json!([40, 30, 12]);
    m["stock"]["cell_size_mm"] = serde_json::json!(0.5);
    m["initial_position_mm"] = serde_json::json!([0, 0, 15]);
    serde_json::from_value(m).unwrap()
}
/// Points along a level's cut (lines and plan-view arcs).
fn along(curve: &Curve) -> Vec<P3> {
    const STEP: f64 = 0.2;
    match *curve {
        Curve::Line { start, end } => {
            let n = ((end[0] - start[0]).hypot(end[1] - start[1]) / STEP).ceil() as usize + 1;
            (0..=n)
                .map(|i| {
                    let t = i as f64 / n as f64;
                    std::array::from_fn(|k| start[k] + t * (end[k] - start[k]))
                })
                .collect()
        }
        Curve::Arc {
            start,
            end,
            center,
            sweep_rad,
            ..
        } => {
            assert!(
                start[2] == end[2] && start[2] == center[2],
                "a level's arcs lie in its plane"
            );
            let r = (start[0] - center[0]).hypot(start[1] - center[1]);
            let from = (start[1] - center[1]).atan2(start[0] - center[0]);
            let n = (r * sweep_rad.abs() / STEP).ceil() as usize + 1;
            let points: Vec<P3> = (0..=n)
                .map(|i| {
                    let a = from + sweep_rad * i as f64 / n as f64;
                    [center[0] + r * a.cos(), center[1] + r * a.sin(), start[2]]
                })
                .collect();
            let last = points[n];
            assert!((last[0] - end[0]).hypot(last[1] - end[1]) < 1e-6);
            points
        }
        Curve::Helix { .. } => panic!("a flat level has no helix"),
    }
}
/// Runs a flat or bull-nose waterline over `mesh` and returns how deep the
/// cutter's profile reaches into the sampled mesh at worst (negative: the
/// smallest clearance), with the number of cut moves.
///
/// The cutter (radius 2, corner radius `corner`) with its tip at Z t reaches,
/// at a height u above the tip, out to 2 - corner + sqrt(corner^2 -
/// (corner - u)^2) for u < corner and to 2 above that.
fn worst_gouge(mesh: &SurfaceMesh, corner: f64, top: f64) -> (f64, usize) {
    let cancel = never();
    let m = manifest(if corner > 0. {
        serde_json::json!({"kind":"bull","corner_radius_mm":corner})
    } else {
        serde_json::json!({"kind":"flat"})
    });
    let Operation::Waterline(mut op) = m.operations[0].clone() else {
        panic!("fixture")
    };
    op.surface = mesh.clone();
    op.top_z_mm = top;
    op.bottom_z_mm = top - 2.;
    op.stepdown_mm = 1.;
    op.clearance_z_mm = 15.;
    let spec = m.stock.height().unwrap();
    let mut plan =
        WaterlinePlan::prepare(op, &m.tools[0], spec, m.initial_position_mm, 1, &cancel).unwrap();
    let tool = m.tools[0].prepare().unwrap();
    let mut stock = Stock::new(spec.clone()).unwrap();
    let points = samples(mesh);
    let (mut worst, mut cuts) = (f64::NEG_INFINITY, 0);
    while let Some(motion) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        if motion.kind == MoveKind::Cut {
            cuts += 1;
            for c in along(&motion.curve) {
                for p in &points {
                    let u = p[2] - c[2];
                    if u <= 0. {
                        continue;
                    }
                    let reach = if u < corner {
                        2. - corner + (corner * corner - (corner - u).powi(2)).sqrt()
                    } else {
                        2.
                    };
                    worst = worst.max(reach - (p[0] - c[0]).hypot(p[1] - c[1]));
                }
            }
        }
        let delta = stock
            .preview_sweep(&tool, &motion, 1., m.sweep, &cancel)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
    }
    plan.finish_report(&cancel).unwrap();
    let error = plan.report().shadow_error_mm.unwrap();
    assert!(error > 0. && error <= 100. * TOLERANCE, "{error}");
    (worst, cuts)
}

#[test]
fn flat_and_bull_levels_stay_beside_a_part_with_a_vertex_on_the_level() {
    // The fixture's margin: the first level's tip is at `top`, and the
    // shadow under it is taken this far below (less a numerical reserve).
    let margin = 0.005;
    for corner in [0., 0.5] {
        // The fan vertex where the first level's shadow is taken, a hair off
        // it, and where the former waterline shadow was taken (at the tip
        // less (2 + margin) - 2, which dropped the facets beside it).
        let top = MIDDLE + 0.25;
        let now = PreparedRoughingTarget::prepare(boss(tilt, None), TOLERANCE, &never())
            .unwrap()
            .shadow(top, margin, &mut 0, &never())
            .unwrap()
            .report
            .protected_from_z_mm;
        let former = top - ((2. + margin) - 2.);
        for (name, centre) in [
            ("on the level", now),
            ("1e-8 above the level", now + 1e-8),
            ("1e-8 below the level", now - 1e-8),
            ("on the former level", former),
            ("1e-8 above the former level", former + 1e-8),
            ("1e-8 below the former level", former - 1e-8),
        ] {
            let (worst, cuts) = worst_gouge(&boss(tilt, Some(centre)), corner, top);
            assert!(cuts > 8, "corner {corner}, vertex {name}: {cuts} cut moves");
            assert!(
                worst <= 0.,
                "corner {corner}, vertex {name}: the cutter reaches {worst} mm into the part"
            );
            // And it finishes the part: the closest it comes is within the
            // margin and the shadow's own error.
            assert!(
                worst >= -0.02,
                "corner {corner}, vertex {name}: the cutter stays {} mm off the part",
                -worst
            );
        }
    }
}
