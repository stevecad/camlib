//! Scallop (constant stepover) finishing passes: successive offsets of the
//! machining boundary measured along the surface, so the stepover stays about
//! constant on steep and shallow areas alike.
//!
//! On a sample grid, the distance from the grid border is computed over the
//! ball-center surface (Dijkstra with 8 neighbours; each step weighs its 3D
//! length, so slopes count fully). Passes are its contours at multiples of
//! the stepover (marching squares with linear interpolation). Grid
//! metrication makes the stepover approximate (within about 8%); the passes
//! only choose where to cut: every chord is lifted onto the surface and
//! certified like any other finishing pass.
//! The opt-in field measures the boundary distance on the triangulated
//! ball-centre samples. It keeps the same contour interpolation and lifting.
use crate::{
    stock::CancelSignal,
    surface::{PreparedSurface, SurfaceWork},
};
use std::{cmp::Ordering, collections::BinaryHeap, collections::HashMap};

#[derive(PartialEq)]
struct Node(f64, usize);
impl Eq for Node {}
impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        other.0.total_cmp(&self.0).then(other.1.cmp(&self.1))
    }
}

/// Scallop passes (XY polylines), outermost first.
#[allow(clippy::too_many_arguments)]
pub(crate) fn passes(
    surface: &PreparedSurface,
    lo: [f64; 2],
    hi: [f64; 2],
    spacing: f64,
    stepover: f64,
    radius: f64,
    intrinsic: bool,
    work: &mut SurfaceWork,
    cancel: &impl CancelSignal,
) -> Result<Vec<Vec<[f64; 2]>>, String> {
    let nx = ((hi[0] - lo[0]) / spacing).ceil().max(1.) as usize + 1;
    let ny = ((hi[1] - lo[1]) / spacing).ceil().max(1.) as usize + 1;
    crate::resources::check_memory(nx as u128 * ny as u128 * 48, "scallop distance grid")?;
    let at = |i: usize, j: usize| {
        [
            lo[0] + (hi[0] - lo[0]) * i as f64 / (nx - 1) as f64,
            lo[1] + (hi[1] - lo[1]) * j as f64 / (ny - 1) as f64,
        ]
    };
    let mut h = Vec::with_capacity(nx * ny);
    for j in 0..ny {
        if cancel.is_cancelled() {
            return Err("scallop planning cancelled".into());
        }
        for i in 0..nx {
            h.push(surface.contact(at(i, j), radius, work, cancel)?.center_z);
        }
    }
    let (dx, dy) = (
        (hi[0] - lo[0]) / (nx - 1) as f64,
        (hi[1] - lo[1]) / (ny - 1) as f64,
    );
    let dist = if intrinsic {
        intrinsic_distances(&h, nx, ny, dx, dy, cancel)?
    } else {
        grid_distances(&h, nx, ny, dx, dy, cancel)?
    };
    let max = dist.iter().copied().fold(0., f64::max);
    let levels = (max / stepover).floor() as usize;
    crate::resources::check_memory(levels as u128 * (nx + ny) as u128 * 32, "scallop contours")?;
    let mut out = vec![];
    for level in 1..=levels {
        if cancel.is_cancelled() {
            return Err("scallop planning cancelled".into());
        }
        let value = level as f64 * stepover;
        let mut crossings: HashMap<(usize, bool), [f64; 2]> = HashMap::new();
        for j in 0..ny {
            for i in 0..nx {
                let k = j * nx + i;
                let mut edge = |n: usize, b: [f64; 2], key: (usize, bool)| {
                    let (u, v) = (dist[k], dist[n]);
                    if (u < value) != (v < value) {
                        let t = (value - u) / (v - u);
                        let a = at(i, j);
                        crossings.insert(key, [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]);
                    }
                };
                if i + 1 < nx {
                    edge(k + 1, at(i + 1, j), (k, true));
                }
                if j + 1 < ny {
                    edge(k + nx, at(i, j + 1), (k, false));
                }
            }
        }
        out.extend(super::pencil::link(&crossings, nx, ny));
    }
    Ok(out)
}

fn on_boundary(i: usize, j: usize, nx: usize, ny: usize) -> bool {
    i == 0 || j == 0 || i + 1 == nx || j + 1 == ny
}

fn grid_distances(
    h: &[f64],
    nx: usize,
    ny: usize,
    dx: f64,
    dy: f64,
    cancel: &impl CancelSignal,
) -> Result<Vec<f64>, String> {
    let mut dist = vec![f64::INFINITY; nx * ny];
    let mut heap = BinaryHeap::new();
    for j in 0..ny {
        for i in 0..nx {
            if on_boundary(i, j, nx, ny) {
                dist[j * nx + i] = 0.;
                heap.push(Node(0., j * nx + i));
            }
        }
    }
    let mut popped = 0usize;
    while let Some(Node(d, k)) = heap.pop() {
        popped += 1;
        if popped.is_multiple_of(65536) && cancel.is_cancelled() {
            return Err("scallop planning cancelled".into());
        }
        if d > dist[k] {
            continue;
        }
        let (i, j) = ((k % nx) as isize, (k / nx) as isize);
        for (di, dj) in [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
            (-1, -1),
            (1, -1),
            (-1, 1),
            (1, 1),
        ] {
            let (a, b) = (i + di, j + dj);
            if a < 0 || b < 0 || a >= nx as isize || b >= ny as isize {
                continue;
            }
            let n = b as usize * nx + a as usize;
            let step =
                ((di as f64 * dx).powi(2) + (dj as f64 * dy).powi(2) + (h[n] - h[k]).powi(2))
                    .sqrt();
            if d + step < dist[n] {
                dist[n] = d + step;
                heap.push(Node(d + step, n));
            }
        }
    }
    Ok(dist)
}

fn intrinsic_distances(
    h: &[f64],
    nx: usize,
    ny: usize,
    dx: f64,
    dy: f64,
    cancel: &impl CancelSignal,
) -> Result<Vec<f64>, String> {
    // The exact field holds windows as well as the mesh. Leave room for
    // connectivity and boundary sources before the field's own work checks.
    crate::resources::check_memory(h.len() as u128 * 1024, "scallop tool-centre mesh")?;
    if h.len() > (u32::MAX as usize) / 6 {
        return Err("scallop tool-centre mesh has too many vertices".into());
    }
    let mut vertices = Vec::with_capacity(h.len());
    let mut boundary = Vec::with_capacity(h.len());
    let mut triangles = Vec::with_capacity(2 * (nx - 1) * (ny - 1));
    for j in 0..ny {
        if cancel.is_cancelled() {
            return Err("scallop planning cancelled".into());
        }
        for i in 0..nx {
            let k = j * nx + i;
            vertices.push([i as f64 * dx, j as f64 * dy, h[k]]);
            boundary.push(on_boundary(i, j, nx, ny));
            if i + 1 < nx && j + 1 < ny {
                let [a, b, c, d] = [k, k + 1, k + nx + 1, k + nx];
                let [a_id, b_id, c_id, d_id] = [a, b, c, d].map(|n| n as u32);
                if (h[a] - h[c]).abs() <= (h[b] - h[d]).abs() {
                    triangles.extend([[a_id, b_id, c_id], [a_id, c_id, d_id]]);
                } else {
                    triangles.extend([[a_id, b_id, d_id], [b_id, c_id, d_id]]);
                }
            }
        }
    }
    super::geodesic::sampled_boundary_distances(&vertices, &triangles, &boundary, cancel)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sampled_intrinsic_distance_finds_the_perpendicular_to_an_oblique_edge() {
        // On this plane the shortest path to x=0 also moves in y.
        // No sequence of the eight grid directions follows that direction.
        let (nx, ny) = (21, 31);
        let (sx, sy) = (0.9_f64, 0.8_f64);
        let heights: Vec<_> = (0..ny)
            .flat_map(|j| (0..nx).map(move |i| sx * i as f64 + sy * j as f64))
            .collect();
        let cancel = || false;
        let index = 15 * nx + 7;
        let exact = 7. * (1. + sx * sx / (1. + sy * sy)).sqrt();
        let grid = grid_distances(&heights, nx, ny, 1., 1., &cancel).unwrap();
        let sampled = intrinsic_distances(&heights, nx, ny, 1., 1., &cancel).unwrap();
        assert!((sampled[index] - exact).abs() < 1e-8, "{}", sampled[index]);
        assert!(grid[index] - exact > 0.25, "{}", grid[index]);
    }
}
