//! Pencil finishing paths: the concave creases of a surface where a ball of
//! the tool radius touches two faces at once (inside corners and valleys).
//!
//! On a sample grid of drop-cutter contacts (inflated ball), a grid edge
//! may cross a crease when the contact normals at its ends differ by more
//! than `min_angle`. The crossing is located by bisection on which end's
//! normal the contact matches. It is a crease when, across it, one contact
//! point lies above the other side's tangent plane (an inside corner or a
//! valley, also beside a wall where the ball-center height jumps); around a
//! convex edge both contacts stay on the edge, so it is not. Crossings are
//! linked cell by cell (marching squares) into polylines. The polylines only choose where to cut: every chord is lifted
//! onto the surface and certified like any other finishing pass.
use crate::{
    stock::CancelSignal,
    surface::{PreparedSurface, SurfaceWork},
};
use std::collections::HashMap;

struct Sample {
    base: bool,
    h: f64,
    n: [f64; 3],
    /// Contact point on the inflated surface.
    p: [f64; 3],
}

fn sample(
    surface: &PreparedSurface,
    xy: [f64; 2],
    radius: f64,
    work: &mut SurfaceWork,
    cancel: &impl CancelSignal,
) -> Result<Sample, String> {
    let c = surface.contact(xy, radius, work, cancel)?;
    let n = c.contact_normal(xy, radius);
    Ok(Sample {
        base: c.on_base(),
        h: c.center_z,
        n,
        p: [
            xy[0] - radius * n[0],
            xy[1] - radius * n[1],
            c.center_z - radius * n[2],
        ],
    })
}

fn angle(a: [f64; 3], b: [f64; 3]) -> f64 {
    (a[0] * b[0] + a[1] * b[1] + a[2] * b[2])
        .clamp(-1., 1.)
        .acos()
}

/// Crease polylines (XY) over `lo..hi`, sampled `spacing` apart.
#[allow(clippy::too_many_arguments)]
pub(crate) fn creases(
    surface: &PreparedSurface,
    lo: [f64; 2],
    hi: [f64; 2],
    spacing: f64,
    radius: f64,
    min_angle: f64,
    locate: f64,
    work: &mut SurfaceWork,
    cancel: &impl CancelSignal,
) -> Result<Vec<Vec<[f64; 2]>>, String> {
    // Chords between crease points sag at most spacing^2 / (8 radius)
    // toward a corner; the points are held off cliffs by that and `locate`.
    let margin = locate + spacing * spacing / (8. * radius);
    let nx = ((hi[0] - lo[0]) / spacing).ceil().max(1.) as usize + 1;
    let ny = ((hi[1] - lo[1]) / spacing).ceil().max(1.) as usize + 1;
    crate::resources::check_memory((nx * ny) as u128 * 64, "pencil sample grid")?;
    let at = |i: usize, j: usize| {
        [
            lo[0] + (hi[0] - lo[0]) * i as f64 / (nx - 1) as f64,
            lo[1] + (hi[1] - lo[1]) * j as f64 / (ny - 1) as f64,
        ]
    };
    let mut grid = Vec::with_capacity(nx * ny);
    for j in 0..ny {
        for i in 0..nx {
            grid.push(sample(surface, at(i, j), radius, work, cancel)?);
        }
    }
    // Crossing point on each grid edge, keyed (cell index, horizontal?).
    let mut crossings: HashMap<(usize, bool), [f64; 2]> = HashMap::new();
    let mut edge = |a: [f64; 2],
                    b: [f64; 2],
                    sa: &Sample,
                    sb: &Sample,
                    key: (usize, bool),
                    work: &mut SurfaceWork|
     -> Result<(), String> {
        if angle(sa.n, sb.n) < min_angle {
            return Ok(());
        }
        let (mut p, mut q) = (a, b);
        let copy = |s: &Sample| Sample {
            base: s.base,
            h: s.h,
            n: s.n,
            p: s.p,
        };
        let (mut sp, mut sq) = (copy(sa), copy(sb));
        while (q[0] - p[0]).hypot(q[1] - p[1]) > locate {
            let x = [(p[0] + q[0]) / 2., (p[1] + q[1]) / 2.];
            let s = sample(surface, x, radius, work, cancel)?;
            if angle(s.n, sa.n) <= angle(s.n, sb.n) {
                (p, sp) = (x, s);
            } else {
                (q, sq) = (x, s);
            }
        }
        let rise = |from: &Sample, to: &Sample| {
            (0..3)
                .map(|k| (to.p[k] - from.p[k]) * from.n[k])
                .sum::<f64>()
        };
        // The machining base is a limit, not part of the part.
        if sp.base
            || sq.base
            || angle(sp.n, sq.n) < min_angle
            || rise(&sp, &sq).max(rise(&sq, &sp)) <= locate
        {
            return Ok(()); // convex edge or smooth turn: no crease here
        }
        // Beside a wall the ball-center height jumps: keep the point on the
        // low side, pushed off the wall (along the high side's horizontal
        // normal) by `margin`, so chords between crease points (sagitta at
        // most the margin) stay clear of the corner.
        let point = if (sp.h - sq.h).abs() > 16. * locate {
            let (low, high) = if sp.h < sq.h { (p, &sq) } else { (q, &sp) };
            let len = high.n[0].hypot(high.n[1]);
            if len > 0. {
                [
                    low[0] + margin * high.n[0] / len,
                    low[1] + margin * high.n[1] / len,
                ]
            } else {
                low
            }
        } else {
            [(p[0] + q[0]) / 2., (p[1] + q[1]) / 2.]
        };
        crossings.insert(key, point);
        Ok(())
    };
    for j in 0..ny {
        for i in 0..nx {
            if cancel.is_cancelled() {
                return Err("pencil detection cancelled".into());
            }
            let k = j * nx + i;
            if i + 1 < nx {
                edge(
                    at(i, j),
                    at(i + 1, j),
                    &grid[k],
                    &grid[k + 1],
                    (k, true),
                    work,
                )?;
            }
            if j + 1 < ny {
                edge(
                    at(i, j),
                    at(i, j + 1),
                    &grid[k],
                    &grid[k + nx],
                    (k, false),
                    work,
                )?;
            }
        }
    }
    Ok(link(&crossings, nx, ny))
}

/// Link grid-edge crossings, keyed (node index, horizontal edge?), cell by
/// cell (marching squares) into polylines: open chains first, then loops.
pub(super) fn link(
    crossings: &HashMap<(usize, bool), [f64; 2]>,
    nx: usize,
    ny: usize,
) -> Vec<Vec<[f64; 2]>> {
    // Marching squares: link the crossings of each cell. Only cells
    // touching a crossing can link any, visited in ascending order (the
    // same order as a scan of every cell, so the same chains).
    let mut links: HashMap<(usize, bool), Vec<(usize, bool)>> = HashMap::new();
    let mut cells: Vec<usize> = crossings
        .keys()
        .flat_map(|&(k, horizontal)| {
            // A horizontal edge k..k+1 borders cells k and k - nx; a
            // vertical edge k..k+nx borders cells k and k - 1.
            let other = if horizontal {
                k.checked_sub(nx)
            } else {
                (k % nx > 0).then(|| k - 1)
            };
            [Some(k), other]
        })
        .flatten()
        .filter(|&c| c % nx < nx - 1 && c / nx < ny - 1)
        .collect();
    cells.sort_unstable();
    cells.dedup();
    for k in cells {
        {
            let edges = [(k, true), (k + nx, true), (k, false), (k + 1, false)];
            let present: Vec<_> = edges
                .into_iter()
                .filter(|e| crossings.contains_key(e))
                .collect();
            let pairs: Vec<((usize, bool), (usize, bool))> = match present.len() {
                2 => vec![(present[0], present[1])],
                3 => {
                    // Junction: link the closest pair.
                    let mut best = (f64::INFINITY, 0, 1);
                    for a in 0..3 {
                        for b in a + 1..3 {
                            let (p, q) = (crossings[&present[a]], crossings[&present[b]]);
                            let d = (p[0] - q[0]).hypot(p[1] - q[1]);
                            if d < best.0 {
                                best = (d, a, b);
                            }
                        }
                    }
                    vec![(present[best.1], present[best.2])]
                }
                4 => {
                    let p = |e| crossings[&e];
                    let d = |a: [f64; 2], b: [f64; 2]| (a[0] - b[0]).hypot(a[1] - b[1]);
                    let (e0, e1, e2, e3) = (present[0], present[1], present[2], present[3]);
                    let options = [
                        ((e0, e1), (e2, e3)),
                        ((e0, e2), (e1, e3)),
                        ((e0, e3), (e1, e2)),
                    ];
                    let best = options
                        .into_iter()
                        .min_by(|a, b| {
                            let la = d(p(a.0.0), p(a.0.1)) + d(p(a.1.0), p(a.1.1));
                            let lb = d(p(b.0.0), p(b.0.1)) + d(p(b.1.0), p(b.1.1));
                            la.total_cmp(&lb)
                        })
                        .unwrap();
                    vec![best.0, best.1]
                }
                _ => vec![],
            };
            for (a, b) in pairs {
                links.entry(a).or_default().push(b);
                links.entry(b).or_default().push(a);
            }
        }
    }
    // Chain the links into polylines (open chains first, then loops).
    let mut used: std::collections::HashSet<((usize, bool), (usize, bool))> =
        std::collections::HashSet::new();
    let mut keys: Vec<_> = links.keys().copied().collect();
    keys.sort();
    let mut chains = vec![];
    type Edge = (usize, bool);
    let walk = |start: Edge,
                used: &mut std::collections::HashSet<(Edge, Edge)>,
                chains: &mut Vec<Vec<[f64; 2]>>| {
        let mut chain = vec![crossings[&start]];
        let mut at = start;
        loop {
            let next = links[&at]
                .iter()
                .copied()
                .find(|n| !used.contains(&(at, *n)));
            let Some(n) = next else { break };
            used.insert((at, n));
            used.insert((n, at));
            chain.push(crossings[&n]);
            at = n;
            if at == start {
                break;
            }
        }
        if chain.len() >= 2 {
            chains.push(chain);
        }
    };
    for &k in &keys {
        if links[&k].len() == 1 && !used.contains(&(k, links[&k][0])) {
            walk(k, &mut used, &mut chains);
        }
    }
    for &k in &keys {
        while links[&k].iter().any(|n| !used.contains(&(k, *n))) {
            walk(k, &mut used, &mut chains);
        }
    }
    chains
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::surface::SurfaceMesh;
    use std::sync::atomic::AtomicBool;

    /// An 8 x 8 x 3 boss standing on the base plane Z 4.
    fn boss() -> SurfaceMesh {
        let (x0, x1, y0, y1, z0, z1) = (8., 16., 4., 12., 4., 7.);
        let q = |a: [f64; 3], b: [f64; 3], c: [f64; 3], d: [f64; 3]| [[a, b, c], [a, c, d]];
        let mut t = vec![];
        // The floor around the boss is part of the part (the base is lower).
        t.extend(q(
            [-4., -4., z0],
            [28., -4., z0],
            [28., 20., z0],
            [-4., 20., z0],
        ));
        t.extend(q([x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]));
        t.extend(q([x0, y0, z0], [x1, y0, z0], [x1, y0, z1], [x0, y0, z1]));
        t.extend(q([x1, y0, z0], [x1, y1, z0], [x1, y1, z1], [x1, y0, z1]));
        t.extend(q([x1, y1, z0], [x0, y1, z0], [x0, y1, z1], [x1, y1, z1]));
        t.extend(q([x0, y1, z0], [x0, y0, z0], [x0, y0, z1], [x0, y1, z1]));
        SurfaceMesh {
            version: 1,
            units: "mm".into(),
            triangles: std::sync::Arc::new(t),
            surface_error_mm: 0.,
            source: serde_json::Value::Null,
        }
    }

    #[test]
    fn crease_around_a_boss_is_one_closed_loop() {
        let cancel = AtomicBool::new(false);
        let surface = boss().prepare(0., 1e-5, &cancel).unwrap();
        let mut work = SurfaceWork::default();
        let chains = creases(
            &surface,
            [0., 0.],
            [24., 16.],
            0.5,
            2.,
            20f64.to_radians(),
            0.001,
            &mut work,
            &cancel,
        )
        .unwrap();
        let lengths: Vec<f64> = chains
            .iter()
            .map(|c| {
                c.windows(2)
                    .map(|w| (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]))
                    .sum()
            })
            .collect();
        assert_eq!(chains.len(), 1, "{lengths:?}");
        assert!(
            (lengths[0] - (32. + std::f64::consts::TAU * 2.)).abs() < 0.5,
            "{lengths:?}"
        );
    }

    #[test]
    fn a_convex_ridge_is_not_a_crease_but_a_valley_is() {
        let cancel = AtomicBool::new(false);
        let mesh = |up: f64| SurfaceMesh {
            version: 1,
            units: "mm".into(),
            // Two 20 degree slopes meeting at x = 12: a ridge (up) or a valley.
            triangles: std::sync::Arc::new(vec![
                [[0., 0., 5.], [12., 0., 5. + up], [12., 16., 5. + up]],
                [[0., 0., 5.], [12., 16., 5. + up], [0., 16., 5.]],
                [[12., 0., 5. + up], [24., 0., 5.], [24., 16., 5.]],
                [[12., 0., 5. + up], [24., 16., 5.], [12., 16., 5. + up]],
            ]),
            surface_error_mm: 0.,
            source: serde_json::Value::Null,
        };
        let rise = 12. * 20f64.to_radians().tan();
        let count = |up: f64| {
            let surface = mesh(up).prepare(0., 1e-5, &cancel).unwrap();
            let mut work = SurfaceWork::default();
            creases(
                &surface,
                [1., 1.],
                [23., 15.],
                0.5,
                2.,
                20f64.to_radians(),
                0.001,
                &mut work,
                &cancel,
            )
            .unwrap()
        };
        assert!(count(rise).is_empty(), "ridge");
        let valley = count(-rise);
        assert_eq!(valley.len(), 1);
        assert!(
            valley[0].iter().all(|p| (p[0] - 12.).abs() < 0.01),
            "{valley:?}"
        );
    }
}
