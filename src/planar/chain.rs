//! Exact one-sided offset of an open line/arc chain (sideways compensation of
//! an open contour).
//!
//! The offset is constructed by the same kernel as region offsets
//! (cavalier_contours 0.9.0, open-polyline parallel offset): each line moves
//! parallel, each arc keeps its centre with its radius changed by the offset,
//! convex joints are closed by an arc about the joint, concave joints are
//! trimmed at the intersection of the neighbours, and every span closer to
//! the chain than the offset is removed. The offset starts and ends on the
//! normals through the chain's end points: nothing wraps around the ends.
//!
//! The kernel result is not trusted. It is accepted only when it is one
//! continuous path from the start normal to the end normal, lies at the
//! offset distance from the chain (measured here, analytically), and touches
//! every segment of the chain. Anything else is refused: a segment the cutter
//! cannot reach is never dropped silently.
use crate::{Point, pocket::Stroke};
use cavalier_contours::polyline::{
    PlineOffsetOptions, PlineSource, PlineSourceMut, PlineVertex, Polyline,
};
use std::f64::consts::PI;

fn arc_radius(s: &Stroke) -> Option<(Point, f64, f64)> {
    s.arc
        .map(|(c, sweep)| (c, (s.a[0] - c[0]).hypot(s.a[1] - c[1]), sweep))
}

/// Point of the stroke at parameter `t` in 0..=1 (by arc angle for arcs).
fn point_at(s: &Stroke, t: f64) -> Point {
    match arc_radius(s) {
        None => [
            s.a[0] + t * (s.b[0] - s.a[0]),
            s.a[1] + t * (s.b[1] - s.a[1]),
        ],
        Some((c, r, sweep)) => {
            let angle = (s.a[1] - c[1]).atan2(s.a[0] - c[0]) + t * sweep;
            [c[0] + r * angle.cos(), c[1] + r * angle.sin()]
        }
    }
}

/// Distance from `p` to the stroke and the parameter (0..=1) of its nearest point.
fn closest(s: &Stroke, p: Point) -> (f64, f64) {
    let t = match arc_radius(s) {
        None => {
            let d = [s.b[0] - s.a[0], s.b[1] - s.a[1]];
            (((p[0] - s.a[0]) * d[0] + (p[1] - s.a[1]) * d[1]) / (d[0] * d[0] + d[1] * d[1]))
                .clamp(0., 1.)
        }
        Some((c, _, sweep)) => {
            let start = (s.a[1] - c[1]).atan2(s.a[0] - c[0]);
            // Angle travelled from the start in the arc's direction, in 0..2*pi.
            let travelled =
                (((p[1] - c[1]).atan2(p[0] - c[0]) - start) * sweep.signum()).rem_euclid(2. * PI);
            if travelled <= sweep.abs() {
                travelled / sweep.abs()
            } else {
                // Outside the sweep: the nearer end point.
                let end = |q: Point| (p[0] - q[0]).hypot(p[1] - q[1]);
                if end(s.a) <= end(s.b) { 0. } else { 1. }
            }
        }
    };
    let q = point_at(s, t);
    ((p[0] - q[0]).hypot(p[1] - q[1]), t)
}

fn left_normal(t: Point) -> Point {
    [-t[1], t[0]]
}

/// Offset the open chain by `offset_mm`: positive to the left of the
/// direction of travel, negative to the right. `epsilon_mm` is the
/// construction resolution, `guard_mm` the acceptance band of the checks.
pub(crate) fn offset_open_chain(
    chain: &[Stroke],
    offset_mm: f64,
    epsilon_mm: f64,
    guard_mm: f64,
) -> Result<Vec<Stroke>, String> {
    let distance = offset_mm.abs();
    let side = if offset_mm > 0. { "left" } else { "right" };
    if chain.is_empty() || !offset_mm.is_finite() || distance <= guard_mm || distance > 1e6 {
        return Err("a side offset needs a chain and a finite offset above the tolerance".into());
    }
    // An arc curving towards the offset side keeps its centre and loses the
    // offset from its radius; at or below zero there is no offset arc.
    for (i, s) in chain.iter().enumerate() {
        if let Some((_, radius, sweep)) = arc_radius(s)
            && radius - offset_mm * sweep.signum() <= guard_mm
        {
            return Err(format!(
                "segment {i}: the arc of radius {radius} mm curves towards the {side} side and is not larger than the offset {distance} mm (tool radius plus allowance), so the cutter cannot follow it; use a smaller tool or allowance, or a larger arc"
            ));
        }
    }
    let origin = chain[0].a;
    let mut source = Polyline::new();
    for s in chain {
        match arc_radius(s) {
            None => {
                source.add(s.a[0] - origin[0], s.a[1] - origin[1], 0.);
            }
            Some((_, _, sweep)) => {
                // A bulge describes at most a half circle: split longer arcs.
                let pieces = if sweep.abs() > PI { 2 } else { 1 };
                let bulge = (sweep / pieces as f64 / 4.).tan();
                for k in 0..pieces {
                    let p = point_at(s, k as f64 / pieces as f64);
                    source.add(p[0] - origin[0], p[1] - origin[1], bulge);
                }
            }
        }
    }
    let end = chain[chain.len() - 1].b;
    source.add(end[0] - origin[0], end[1] - origin[1], 0.);
    let options = PlineOffsetOptions {
        handle_self_intersects: true,
        pos_equal_eps: epsilon_mm,
        offset_dist_eps: 4. * epsilon_mm,
        ..Default::default()
    };
    let mut results: Vec<Polyline<f64>> = source.parallel_offset_opt(offset_mm, &options);
    let unreachable = "use a smaller tool or allowance, or split the path there";
    if results.len() != 1 || results[0].is_closed() || results[0].vertex_count() < 2 {
        return Err(format!(
            "the path offset {distance} mm to the {side} is not one continuous path ({} pieces): the chain folds back on itself or passes closer to itself than the offset; {unreachable}",
            results.len()
        ));
    }
    let result = results.pop().unwrap();
    let vertices: Vec<PlineVertex<f64>> =
        (0..result.vertex_count()).map(|i| result.at(i)).collect();
    let mut strokes = Vec::with_capacity(vertices.len() - 1);
    for pair in vertices.windows(2) {
        let (v, w) = (pair[0], pair[1]);
        let a = [v.x + origin[0], v.y + origin[1]];
        let b = [w.x + origin[0], w.y + origin[1]];
        let mut stroke = Stroke::line(a, b);
        if v.bulge != 0. {
            // Centre of a bulge arc: on the chord's bisector, as for regions
            // (`Stroke::from_boundary`).
            let factor = (1. - v.bulge * v.bulge) / (4. * v.bulge);
            stroke.arc = Some((
                [
                    (a[0] + b[0]) / 2. - factor * (b[1] - a[1]),
                    (a[1] + b[1]) / 2. + factor * (b[0] - a[0]),
                ],
                4. * v.bulge.atan(),
            ));
        }
        strokes.push(stroke);
    }
    // The path runs from the start normal to the end normal.
    for (name, at, tangent, got) in [
        ("start", chain[0].a, chain[0].tangent(false), strokes[0].a),
        (
            "end",
            end,
            chain[chain.len() - 1].tangent(true),
            strokes[strokes.len() - 1].b,
        ),
    ] {
        let n = left_normal(tangent);
        let want = [at[0] + offset_mm * n[0], at[1] + offset_mm * n[1]];
        if (got[0] - want[0]).hypot(got[1] - want[1]) > guard_mm {
            return Err(format!(
                "the path offset {distance} mm to the {side} does not reach the {name} of the chain: another part of the chain is closer than the offset there; {unreachable}"
            ));
        }
    }
    // Every part of the path is at the offset distance from the chain, and
    // every chain segment is touched somewhere along its length.
    let mut reached = vec![false; chain.len()];
    for stroke in &strokes {
        for k in 0..=16 {
            let t = k as f64 / 16.;
            let p = point_at(stroke, t);
            let mut nearest = f64::INFINITY;
            for (i, s) in chain.iter().enumerate() {
                let (d, at) = closest(s, p);
                nearest = nearest.min(d);
                if t == 0.5 && (d - distance).abs() <= guard_mm && at > 0. && at < 1. {
                    reached[i] = true;
                }
            }
            if (nearest - distance).abs() > guard_mm {
                return Err(format!(
                    "the path offset to the {side} failed its check: a point of it is {nearest} mm from the chain, not {distance} mm; report this path"
                ));
            }
        }
    }
    if let Some(i) = reached.iter().position(|r| !r) {
        return Err(format!(
            "segment {i} is not reached by the path offset {distance} mm to the {side}: the cutter does not fit there (a corner, notch or fold tighter than the offset); {unreachable}"
        ));
    }
    Ok(strokes)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line(a: Point, b: Point) -> Stroke {
        Stroke::line(a, b)
    }

    #[test]
    fn convex_joint_is_rounded_and_concave_joint_trimmed() {
        // An L: east, then north. Left is the inside of the corner.
        let chain = [line([0., 0.], [10., 0.]), line([10., 0.], [10., 10.])];
        let inside = offset_open_chain(&chain, 2., 1e-7, 1e-5).unwrap();
        assert_eq!(inside.len(), 2);
        assert!(inside.iter().all(|s| s.arc.is_none()));
        let corner = inside[0].b;
        assert!((corner[0] - 8.).abs() < 1e-9 && (corner[1] - 2.).abs() < 1e-9);
        let outside = offset_open_chain(&chain, -2., 1e-7, 1e-5).unwrap();
        assert_eq!(outside.len(), 3);
        let (center, sweep) = outside[1].arc.expect("a join arc about the corner");
        assert!((center[0] - 10.).abs() < 1e-9 && center[1].abs() < 1e-9);
        assert!((sweep - PI / 2.).abs() < 1e-9, "{sweep}");
        assert_eq!(outside[0].a, [0., -2.]);
        let last = outside[2].b;
        assert!((last[0] - 12.).abs() < 1e-9 && (last[1] - 10.).abs() < 1e-9);
    }

    #[test]
    fn unreachable_geometry_is_refused() {
        // A concave arc no larger than the offset.
        let arc = Stroke {
            a: [0., 0.],
            b: [2., 2.],
            arc: Some(([0., 2.], PI / 2.)),
        };
        let chain = [line([-10., 0.], [0., 0.]), arc, line([2., 2.], [2., 12.])];
        let error = offset_open_chain(&chain, 3., 1e-7, 1e-5).unwrap_err();
        assert!(error.contains("segment 1: the arc of radius 2"), "{error}");
        assert!(offset_open_chain(&chain, 1., 1e-7, 1e-5).is_ok());
        assert!(offset_open_chain(&chain, -3., 1e-7, 1e-5).is_ok());
        // A notch narrower than the cutter: its floor cannot be reached.
        let notch = [
            line([0., 0.], [10., 0.]),
            line([10., 0.], [10., 5.]),
            line([10., 5.], [13., 5.]),
            line([13., 5.], [13., 0.]),
            line([13., 0.], [23., 0.]),
        ];
        let error = offset_open_chain(&notch, -2., 1e-7, 1e-5).unwrap_err();
        assert!(error.contains("not reached"), "{error}");
        assert!(offset_open_chain(&notch, 2., 1e-7, 1e-5).is_ok());
        // A chain folding back on itself.
        let fold = [line([0., 0.], [10., 0.]), line([10., 0.], [0., 1.])];
        assert!(offset_open_chain(&fold, 2., 1e-7, 1e-5).is_err());
    }
}
