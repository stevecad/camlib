//! Continuous translation of closed convex solids. Face normals and edge
//! cross products are the separating axes (Eberly, sections 4 and 6):
//! https://www.geometrictools.com/Documentation/MethodOfSeparatingAxes.pdf
//! Projection enclosures prove an entire interval clear. Point evaluations
//! supply collision witnesses only; they never prove a motion clear.
use crate::{motion::Vec3, stock::CancelSignal, verification::Status};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub(crate) fn add(a: Vec3, b: Vec3) -> Vec3 {
    std::array::from_fn(|k| a[k] + b[k])
}
pub(crate) fn sub(a: Vec3, b: Vec3) -> Vec3 {
    std::array::from_fn(|k| a[k] - b[k])
}
pub(crate) fn mul(a: Vec3, s: f64) -> Vec3 {
    a.map(|v| v * s)
}
pub(crate) fn dot(a: Vec3, b: Vec3) -> f64 {
    (0..3).map(|k| a[k] * b[k]).sum()
}
fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn norm(a: Vec3) -> f64 {
    a[0].hypot(a[1]).hypot(a[2])
}
pub(crate) fn point(p: Vec3) -> bool {
    p.into_iter().all(|v| v.is_finite() && v.abs() <= 1e6)
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Solid {
    Box {
        min_mm: Vec3,
        max_mm: Vec3,
    },
    /// A closed, outward-wound convex mesh. Concave fixtures are a union of
    /// caller-supplied convex solids, never a silently substituted hull.
    ConvexMesh {
        vertices_mm: Vec<Vec3>,
        triangles: Vec<[usize; 3]>,
    },
}

pub(crate) struct Budget<'a, C: CancelSignal> {
    pub remaining: u64,
    pub cancel: &'a C,
}
impl<C: CancelSignal> Budget<'_, C> {
    pub fn tick(&mut self) -> Result<(), String> {
        if self.cancel.is_cancelled() {
            return Err("machine crash check cancelled".into());
        }
        self.remaining = self
            .remaining
            .checked_sub(1)
            .ok_or("machine crash check work budget exceeded; clearance is unproven")?;
        Ok(())
    }
}

#[derive(Clone)]
pub(crate) struct Shape {
    pub vertices: Vec<Vec3>,
    normals: Vec<Vec3>,
    edges: Vec<Vec3>,
}
fn unique_axis(axes: &mut Vec<Vec3>, seen: &mut BTreeSet<[u64; 3]>, mut n: Vec3) {
    let length = norm(n);
    if length == 0. {
        return;
    }
    n = mul(n, 1. / length);
    if n.iter().find(|v| **v != 0.).is_some_and(|v| *v < 0.) {
        n = mul(n, -1.);
    }
    n = n.map(|v| if v == 0. { 0. } else { v });
    if seen.insert(n.map(f64::to_bits)) {
        axes.push(n);
    }
}
impl Solid {
    pub(crate) fn prepare<C: CancelSignal>(
        &self,
        tolerance: f64,
        work: &mut Budget<'_, C>,
    ) -> Result<Shape, String> {
        let (vertices, triangles) = match self {
            Self::Box {
                min_mm: a,
                max_mm: b,
            } => {
                if !point(*a) || !point(*b) || (0..3).any(|k| b[k] - a[k] <= tolerance) {
                    return Err(
                        "machine box needs finite ordered bounds larger than the check tolerance"
                            .into(),
                    );
                }
                let v = (0..8)
                    .map(|i| std::array::from_fn(|k| if i & (1 << k) == 0 { a[k] } else { b[k] }))
                    .collect();
                (
                    v,
                    vec![
                        [0, 2, 3],
                        [0, 3, 1],
                        [4, 5, 7],
                        [4, 7, 6],
                        [0, 1, 5],
                        [0, 5, 4],
                        [2, 6, 7],
                        [2, 7, 3],
                        [0, 4, 6],
                        [0, 6, 2],
                        [1, 3, 7],
                        [1, 7, 5],
                    ],
                )
            }
            Self::ConvexMesh {
                vertices_mm,
                triangles,
            } => (vertices_mm.clone(), triangles.clone()),
        };
        if vertices.len() < 4 || vertices.iter().any(|p| !point(*p)) || triangles.len() < 4 {
            return Err("machine mesh needs a finite closed three-dimensional convex solid".into());
        }
        crate::resources::check_memory(
            (vertices.len() as u128 * 8 + triangles.len() as u128 * 16) * 8,
            "machine convex solid",
        )?;
        let mut edge_counts = BTreeMap::new();
        let mut referenced = BTreeSet::new();
        let (mut normals, mut edges, mut normal_keys, mut edge_keys) =
            (vec![], vec![], BTreeSet::new(), BTreeSet::new());
        let mut volume = false;
        for face in triangles {
            work.tick()?;
            if face.iter().any(|i| *i >= vertices.len()) {
                return Err("machine mesh triangle index out of range".into());
            }
            let [a, b, c] = face.map(|i| vertices[i]);
            let n = cross(sub(b, a), sub(c, a));
            if norm(n) <= tolerance * tolerance {
                return Err("machine mesh has a degenerate or sub-resolution triangle".into());
            }
            let coord = |p: Vec3| robust::Coord3D {
                x: p[0],
                y: p[1],
                z: p[2],
            };
            for &p in &vertices {
                work.tick()?;
                let side = robust::orient3d(coord(a), coord(b), coord(c), coord(p));
                if side < 0. {
                    return Err("machine mesh must be convex with outward-wound faces; supply a convex decomposition".into());
                }
                volume |= side > 0.;
            }
            unique_axis(&mut normals, &mut normal_keys, n);
            for k in 0..3 {
                let (i, j) = (face[k], face[(k + 1) % 3]);
                if i == j {
                    return Err("machine mesh has a repeated face vertex".into());
                }
                referenced.insert(i);
                let key = (i.min(j), i.max(j));
                let entry = edge_counts.entry(key).or_insert((0_u32, 0_i32));
                entry.0 += 1;
                entry.1 += if i < j { 1 } else { -1 };
                unique_axis(&mut edges, &mut edge_keys, sub(vertices[j], vertices[i]));
            }
        }
        if !volume
            || referenced.len() != vertices.len()
            || edge_counts.values().any(|v| *v != (2, 0))
        {
            return Err(
                "machine mesh must be a closed consistently wound volume with every vertex used"
                    .into(),
            );
        }
        Ok(Shape {
            vertices,
            normals,
            edges,
        })
    }
}
impl Shape {
    pub(crate) fn contains(
        &self,
        other: &Shape,
        shift: Vec3,
        error: [Vec3; 2],
        tolerance: f64,
    ) -> bool {
        self.normals.iter().all(|&n| {
            let outer = self.project(n);
            let inner = other.project(n);
            let u = Pair::uncertainty(n, error);
            let s = dot(n, shift);
            let guard =
                tolerance + 4096. * f64::EPSILON * (1. + s.abs() + outer[0].abs() + outer[1].abs());
            inner[0] + s + u[0] >= outer[0] + guard && inner[1] + s + u[1] <= outer[1] - guard
        })
    }
    /// Internal enclosing boxes may be degenerate (empty stock faces).
    pub(crate) fn aabb(a: Vec3, b: Vec3) -> Self {
        Self {
            vertices: (0..8)
                .map(|i| std::array::from_fn(|k| if i & (1 << k) == 0 { a[k] } else { b[k] }))
                .collect(),
            normals: vec![[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
            edges: vec![[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
        }
    }
    /// Circumscribed regular prism. Every circular section is inside it;
    /// the excess radius is bounded by the declared resolution.
    pub(crate) fn cylinder<C: CancelSignal>(
        r: f64,
        z: [f64; 2],
        resolution: f64,
        work: &mut Budget<'_, C>,
    ) -> Result<Self, String> {
        let n = (std::f64::consts::PI / (r / (r + resolution)).acos())
            .ceil()
            .max(4.) as usize;
        crate::resources::check_memory(n as u128 * 256, "machine tool enclosure")?;
        if n as u64 > work.remaining {
            return Err("machine tool enclosure exceeds work budget".into());
        }
        let r = r / (std::f64::consts::PI / n as f64).cos();
        let mut vertices = vec![];
        let mut normals = vec![[0., 0., 1.]];
        let mut edges = vec![[0., 0., 1.]];
        for i in 0..n {
            work.tick()?;
            let angle = std::f64::consts::TAU * i as f64 / n as f64;
            for h in z {
                vertices.push([r * angle.cos(), r * angle.sin(), h]);
            }
            let angle = angle + std::f64::consts::PI / n as f64;
            normals.push([angle.cos(), angle.sin(), 0.]);
            edges.push([-angle.sin(), angle.cos(), 0.]);
        }
        Ok(Self {
            vertices,
            normals,
            edges,
        })
    }
    pub(crate) fn transformed(
        &self,
        f: impl Fn(Vec3) -> Vec3,
        direction: impl Fn(Vec3) -> Vec3,
    ) -> Self {
        // WorkFrame validation permits small orthogonality residuals. Use the
        // actual inverse transpose for face normals, not an assumed rotation.
        let [x, y, z] = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]].map(&direction);
        let determinant = dot(x, cross(y, z));
        let normal = |n: Vec3| {
            mul(
                add(
                    add(mul(cross(y, z), n[0]), mul(cross(z, x), n[1])),
                    mul(cross(x, y), n[2]),
                ),
                1. / determinant,
            )
        };
        Self {
            vertices: self.vertices.iter().copied().map(f).collect(),
            normals: self.normals.iter().copied().map(normal).collect(),
            edges: self.edges.iter().copied().map(direction).collect(),
        }
    }
    fn project(&self, n: Vec3) -> [f64; 2] {
        self.vertices
            .iter()
            .fold([f64::INFINITY, f64::NEG_INFINITY], |b, p| {
                let s = dot(*p, n);
                [b[0].min(s), b[1].max(s)]
            })
    }
}

pub(crate) struct Pair {
    axes: Vec<(Vec3, [f64; 2], [f64; 2])>,
    reliable: bool,
    scale: f64,
}
#[derive(Debug, Clone, Copy)]
pub(crate) struct Contact {
    pub status: Status,
    pub fraction: f64,
    pub margin_mm: f64,
}
impl Pair {
    pub(crate) fn new<C: CancelSignal>(
        a: &Shape,
        b: &Shape,
        work: &mut Budget<'_, C>,
    ) -> Result<Self, String> {
        crate::resources::check_memory(
            (a.edges.len() as u128 * b.edges.len() as u128
                + a.normals.len() as u128
                + b.normals.len() as u128)
                * 192,
            "machine separating axes",
        )?;
        let (mut axes, mut seen) = (vec![], BTreeSet::new());
        for &n in a.normals.iter().chain(&b.normals) {
            work.tick()?;
            unique_axis(&mut axes, &mut seen, n);
        }
        let mut reliable = true;
        for &e in &a.edges {
            for &f in &b.edges {
                work.tick()?;
                let n = cross(e, f);
                let l = norm(n);
                if l > 0. && l < 1024. * f64::EPSILON {
                    reliable = false;
                }
                unique_axis(&mut axes, &mut seen, n);
            }
        }
        let scale = a
            .vertices
            .iter()
            .chain(&b.vertices)
            .flatten()
            .map(|v| v.abs())
            .fold(1., f64::max);
        Ok(Self {
            axes: axes
                .into_iter()
                .map(|n| (n, a.project(n), b.project(n)))
                .collect(),
            reliable,
            scale,
        })
    }
    fn guard(&self, tolerance: f64, shift: Vec3) -> f64 {
        tolerance
            + 4096.
                * f64::EPSILON
                * (1. + self.scale + shift.into_iter().map(f64::abs).fold(1., f64::max))
    }
    fn uncertainty(n: Vec3, error: [Vec3; 2]) -> [f64; 2] {
        std::array::from_fn(|s| {
            (0..3)
                .map(|k| n[k] * error[if n[k] >= 0. { s } else { 1 - s }][k])
                .sum()
        })
    }
    fn witness<C: CancelSignal>(
        &self,
        shift: Vec3,
        error: [Vec3; 2],
        tol: f64,
        work: &mut Budget<'_, C>,
    ) -> Result<(Status, f64), String> {
        let mut margin = f64::INFINITY;
        let guard = self.guard(tol, shift);
        for &(n, a, b) in &self.axes {
            work.tick()?;
            let u = Self::uncertainty(n, error);
            let s = dot(n, shift);
            if b[1] + s + u[1] < a[0] - guard || b[0] + s + u[0] > a[1] + guard {
                return Ok((Status::Clear, 0.));
            }
            margin = margin
                .min(b[1] + s + u[0] - a[0])
                .min(a[1] - b[0] - s - u[1]);
        }
        Ok((
            if self.reliable && margin > guard {
                Status::Violation
            } else {
                Status::Uncertain
            },
            (margin - guard).max(0.),
        ))
    }
    pub(crate) fn linear<C: CancelSignal>(
        &self,
        from: Vec3,
        to: Vec3,
        error: [Vec3; 2],
        tol: f64,
        work: &mut Budget<'_, C>,
    ) -> Result<Contact, String> {
        let velocity = sub(to, from);
        let guard = self.guard(tol, from).max(self.guard(tol, to));
        let (mut enter, mut exit) = (0_f64, 1_f64);
        for &(n, a, b) in &self.axes {
            work.tick()?;
            let u = Self::uncertainty(n, error);
            let range = [a[0] - b[1] - u[1] - guard, a[1] - b[0] - u[0] + guard];
            let (s, v) = (dot(n, from), dot(n, velocity));
            if v == 0. {
                if s < range[0] || s > range[1] {
                    return Ok(Contact {
                        status: Status::Clear,
                        fraction: 0.,
                        margin_mm: 0.,
                    });
                }
            } else {
                let t0 = (range[0] - s) / v;
                let t1 = (range[1] - s) / v;
                enter = enter.max(t0.min(t1).next_down());
                exit = exit.min(t0.max(t1).next_up());
                if enter > exit {
                    return Ok(Contact {
                        status: Status::Clear,
                        fraction: 0.,
                        margin_mm: 0.,
                    });
                }
            }
        }
        let t = (enter + exit) / 2.;
        let (status, margin) = self.witness(add(from, mul(velocity, t)), error, tol, work)?;
        Ok(Contact {
            status: if status == Status::Clear {
                Status::Uncertain
            } else {
                status
            },
            fraction: t,
            margin_mm: margin,
        })
    }
    /// `speed` bounds the complete relative trajectory's length over unit
    /// parameter time, including all transformed axis coefficients.
    pub(crate) fn curved<C: CancelSignal>(
        &self,
        pose: impl Fn(f64) -> Result<Vec3, String>,
        speed: f64,
        error: [Vec3; 2],
        tol: f64,
        resolution: f64,
        work: &mut Budget<'_, C>,
    ) -> Result<Contact, String> {
        let mut stack = vec![(0., 1.)];
        let mut possible = None;
        while let Some((a, b)) = stack.pop() {
            work.tick()?;
            let mid = (a + b) / 2.;
            let shift = pose(mid)?;
            let radius = speed * (b - a) / 2.;
            let broad = [sub(error[0], [radius; 3]), add(error[1], [radius; 3])];
            if self.witness(shift, broad, tol, work)?.0 == Status::Clear {
                continue;
            }
            let (status, margin) = self.witness(shift, error, tol, work)?;
            if status == Status::Violation {
                return Ok(Contact {
                    status,
                    fraction: mid,
                    margin_mm: margin,
                });
            }
            if speed * (b - a) <= resolution || mid == a || mid == b {
                possible.get_or_insert(Contact {
                    status: Status::Uncertain,
                    fraction: mid,
                    margin_mm: 0.,
                });
            } else {
                stack.push((mid, b));
                stack.push((a, mid));
            }
        }
        Ok(possible.unwrap_or(Contact {
            status: Status::Clear,
            fraction: 0.,
            margin_mm: 0.,
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn translated_boxes_check_entire_paths_and_offset_ranges() {
        let mut work = Budget {
            remaining: 100000,
            cancel: &|| false,
        };
        let a = Solid::Box {
            min_mm: [-1.; 3],
            max_mm: [1.; 3],
        }
        .prepare(1e-6, &mut work)
        .unwrap();
        let pair = Pair::new(&a, &a, &mut work).unwrap();
        let contact = pair
            .linear([-5., 0., 0.], [5., 0., 0.], [[0.; 3]; 2], 1e-6, &mut work)
            .unwrap();
        assert_eq!(contact.status, Status::Violation);
        assert!(contact.fraction > 0. && contact.fraction < 1.);
        assert_eq!(
            pair.linear([-5., 3., 0.], [5., 3., 0.], [[0.; 3]; 2], 1e-6, &mut work)
                .unwrap()
                .status,
            Status::Clear
        );
        assert_eq!(
            pair.linear([-5., 2., 0.], [5., 2., 0.], [[0.; 3]; 2], 1e-6, &mut work)
                .unwrap()
                .status,
            Status::Uncertain
        );
        assert_ne!(
            pair.linear(
                [-5., 3., 0.],
                [5., 3., 0.],
                [[-2.; 3], [2.; 3]],
                1e-6,
                &mut work
            )
            .unwrap()
            .status,
            Status::Clear
        );
    }
}
