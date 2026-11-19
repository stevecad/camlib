//! Analytic ball contact and continuous clearance for downward-extruded facets.
//! Surface uncertainty is independent of display geometry: docs/finishing.md.
use crate::{Point, motion::Vec3, stock::CancelSignal};
use robust::{Coord, orient2d};
use serde::{Deserialize, Serialize};
mod contact_bounds;
mod convex;
mod material;
pub mod spline_bounds;
pub(crate) use contact_bounds::ContactEnclosure;

#[derive(Clone, Debug)]
pub struct SurfaceMesh {
    pub version: u32,
    pub units: String,
    pub triangles: std::sync::Arc<Vec<[Vec3; 3]>>,
    pub surface_error_mm: f64,
    pub source: serde_json::Value,
}
// Facets are shared independently of source/error metadata. Copies are immutable
// until an author explicitly uses Arc::make_mut; indexed wire coordinates keep bits.
#[derive(Clone, Debug)]
pub(crate) struct SurfaceGeometry {
    pub triangles: std::sync::Arc<Vec<[Vec3; 3]>>,
}
fn coordinate_fields<S: serde::ser::SerializeStruct>(
    triangles: &[[Vec3; 3]],
    state: &mut S,
) -> Result<(), S::Error> {
    if triangles.len() >= 1024 {
        let mut lookup = std::collections::HashMap::new();
        let mut vertices = Vec::new();
        let mut indices = Vec::with_capacity(triangles.len());
        for triangle in triangles {
            let row: [usize; 3] = std::array::from_fn(|i| {
                let point = triangle[i];
                *lookup.entry(point.map(f64::to_bits)).or_insert_with(|| {
                    let index = vertices.len();
                    vertices.push(point);
                    index
                })
            });
            indices.push(row);
        }
        state.serialize_field("vertices", &vertices)?;
        state.serialize_field("indices", &indices)?;
    } else {
        state.serialize_field("triangles", triangles)?;
    }
    Ok(())
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Coordinates {
    triangles: Option<Vec<[Vec3; 3]>>,
    vertices: Option<Vec<Vec3>>,
    indices: Option<Vec<[usize; 3]>>,
}
impl Coordinates {
    fn resolve<E: serde::de::Error>(self) -> Result<SurfaceGeometry, E> {
        let triangles = match (self.triangles, self.vertices, self.indices) {
            (Some(triangles), None, None) => triangles,
            (None, Some(vertices), Some(indices)) => indices
                .into_iter()
                .map(|row| {
                    let mut points = [[0.; 3]; 3];
                    for i in 0..3 {
                        points[i] = *vertices
                            .get(row[i])
                            .ok_or_else(|| E::custom("surface vertex index out of range"))?;
                    }
                    Ok(points)
                })
                .collect::<Result<Vec<_>, E>>()?,
            _ => {
                return Err(E::custom(
                    "surface requires triangles or vertices and indices",
                ));
            }
        };
        if triangles.is_empty() {
            return Err(E::custom("surface needs at least one triangle"));
        }
        Ok(SurfaceGeometry {
            triangles: triangles.into(),
        })
    }
}
impl Serialize for SurfaceGeometry {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        use serde::ser::SerializeStruct;
        let mut state = serializer.serialize_struct(
            "SurfaceGeometry",
            if self.triangles.len() >= 1024 { 2 } else { 1 },
        )?;
        coordinate_fields(&self.triangles, &mut state)?;
        state.end()
    }
}
impl<'de> Deserialize<'de> for SurfaceGeometry {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        Coordinates::deserialize(deserializer)?.resolve()
    }
}
impl Serialize for SurfaceMesh {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        use serde::ser::SerializeStruct;
        let mut state = serializer.serialize_struct(
            "SurfaceMesh",
            if self.triangles.len() >= 1024 { 6 } else { 5 },
        )?;
        state.serialize_field("version", &self.version)?;
        state.serialize_field("units", &self.units)?;
        coordinate_fields(&self.triangles, &mut state)?;
        state.serialize_field("surface_error_mm", &self.surface_error_mm)?;
        state.serialize_field("source", &self.source)?;
        state.end()
    }
}
impl<'de> Deserialize<'de> for SurfaceMesh {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        #[derive(Deserialize)]
        #[serde(deny_unknown_fields)]
        struct Wire {
            version: u32,
            units: String,
            triangles: Option<Vec<[Vec3; 3]>>,
            vertices: Option<Vec<Vec3>>,
            indices: Option<Vec<[usize; 3]>>,
            surface_error_mm: f64,
            source: serde_json::Value,
        }
        let wire = Wire::deserialize(deserializer)?;
        let geometry = Coordinates {
            triangles: wire.triangles,
            vertices: wire.vertices,
            indices: wire.indices,
        }
        .resolve::<D::Error>()?;
        Ok(Self {
            version: wire.version,
            units: wire.units,
            triangles: geometry.triangles,
            surface_error_mm: wire.surface_error_mm,
            source: wire.source,
        })
    }
}
#[derive(Clone, Copy, Default, Debug, Serialize)]
pub struct SurfaceWork {
    pub nodes: usize,
    pub contact_tests: usize,
    pub distance_tests: usize,
}
impl SurfaceWork {
    /// Work counters are reported, not capped: callers bound run time by
    /// cancellation (or an explicit budget), never by a fixed constant.
    fn check(&self, cancel: &impl CancelSignal) -> Result<(), String> {
        if cancel.is_cancelled() {
            Err("surface planning cancelled".into())
        } else {
            Ok(())
        }
    }
}
fn sub(a: Vec3, b: Vec3) -> Vec3 {
    std::array::from_fn(|i| a[i] - b[i])
}
fn add(a: Vec3, b: Vec3) -> Vec3 {
    std::array::from_fn(|i| a[i] + b[i])
}
fn mul(a: Vec3, s: f64) -> Vec3 {
    a.map(|x| x * s)
}
fn dot(a: Vec3, b: Vec3) -> f64 {
    (a[0] * b[0] + a[1] * b[1]) + a[2] * b[2]
}
fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn orientation(a: Vec3, b: Vec3, c: Vec3, omit: usize) -> f64 {
    let u = (omit + 1) % 3;
    let v = (omit + 2) % 3;
    orient2d(
        Coord { x: a[u], y: a[v] },
        Coord { x: b[u], y: b[v] },
        Coord { x: c[u], y: c[v] },
    )
}
fn inside(t: [Vec3; 3], p: Vec3, omit: usize) -> bool {
    let s = [
        orientation(t[0], t[1], p, omit),
        orientation(t[1], t[2], p, omit),
        orientation(t[2], t[0], p, omit),
    ];
    s.iter().all(|x| *x >= 0.) || s.iter().all(|x| *x <= 0.)
}
#[derive(Clone)]
struct Triangle {
    p: [Vec3; 3],
    normal: Vec3,
    min: Vec3,
    max: Vec3,
}
/// A halfspace containing all downward prisms in a source group. Any normal
/// with nonnegative Z is valid: take its maximum over ALL top vertices. The
/// lower vertices cannot project higher. This is not a fitted/sample plane.
#[derive(Clone)]
struct PlaneBound {
    normal: Vec3,
    origin: Vec3,
    offset: f64,
    length_upper: f64,
    source_scale: f64,
}
impl PlaneBound {
    fn new(normal: Vec3, points: impl Iterator<Item = Vec3>, origin: Vec3) -> Option<Self> {
        let length = dot(normal, normal).sqrt();
        if length == 0. || !length.is_finite() || normal[2] < 0. {
            return None;
        }
        let normal = mul(normal, 1. / length);
        let mut offset = f64::NEG_INFINITY;
        let mut source_scale = origin.iter().map(|x| x.abs()).sum::<f64>();
        for point in points {
            offset = offset.max(dot(sub(point, origin), normal));
            source_scale = source_scale.max(point.iter().map(|x| x.abs()).sum());
        }
        Some(Self {
            normal,
            origin,
            offset,
            length_upper: dot(normal, normal).sqrt() * (1. + 64. * f64::EPSILON),
            source_scale,
        })
    }
    fn separates(&self, triangle: [Vec3; 3], radius: f64) -> bool {
        let minimum = triangle
            .iter()
            .map(|p| dot(sub(*p, self.origin), self.normal))
            .fold(f64::INFINITY, f64::min);
        let scale = 1.
            + self.source_scale
            + radius
            + triangle.iter().flatten().map(|x| x.abs()).sum::<f64>();
        // The affine projection attains its minimum on the filled triangle at
        // a vertex. Cauchy-Schwarz bounds the projection of its WHOLE radius
        // offset. Keep numerical uncertainty on the rejecting side, falling
        // back to the original distance tests near contact.
        minimum - self.offset > radius * self.length_upper + 512. * f64::EPSILON * scale
    }
}
impl Triangle {
    fn new(p: [Vec3; 3]) -> Self {
        let mut n = cross(sub(p[1], p[0]), sub(p[2], p[0]));
        if n[2] < 0. {
            n = mul(n, -1.);
        }
        let len = dot(n, n).sqrt();
        if len > 0. {
            n = mul(n, 1. / len);
        }
        Self {
            p,
            normal: n,
            min: std::array::from_fn(|i| p[0][i].min(p[1][i]).min(p[2][i])),
            max: std::array::from_fn(|i| p[0][i].max(p[1][i]).max(p[2][i])),
        }
    }
    fn separates_triangle(&self, query: [Vec3; 3], radius: f64, floor: f64) -> bool {
        let center = mul(add(add(query[0], query[1]), query[2]), 1. / 3.);
        let mut source = mul(add(add(self.p[0], self.p[1]), self.p[2]), 1. / 3.);
        source[2] = source[2].min(center[2]).max(floor);
        // Direction toward the finite prism, including its horizontal extent.
        // Its centroid chooses ONLY a direction: MAX over all source vertices
        // in PlaneBound is still the authoritative enclosure of the prism.
        PlaneBound::new(sub(center, source), self.p.into_iter(), self.p[0])
            .is_some_and(|plane| plane.separates(query, radius))
    }
    fn face_contact(&self, xy: [f64; 2], radius: f64) -> Option<f64> {
        if self.normal[2] <= 1e-12 {
            return None;
        }
        let q = [
            xy[0] - radius * self.normal[0],
            xy[1] - radius * self.normal[1],
            0.,
        ];
        if !inside(self.p, q, 2) {
            return None;
        }
        let area = orientation(self.p[0], self.p[1], self.p[2], 2);
        let w0 = orientation(self.p[1], self.p[2], q, 2) / area;
        let w1 = orientation(self.p[2], self.p[0], q, 2) / area;
        let z = w0 * self.p[0][2] + w1 * self.p[1][2] + (1. - w0 - w1) * self.p[2][2];
        Some(z + radius * self.normal[2])
    }
    fn raise_contact(&self, index: usize, xy: [f64; 2], radius: f64, result: &mut Contact) {
        let t = self;
        if let Some(z) = t.face_contact(xy, radius)
            && z > result.center_z
        {
            *result = Contact {
                center_z: z,
                support: Support::Face {
                    index,
                    point: [
                        xy[0] - radius * t.normal[0],
                        xy[1] - radius * t.normal[1],
                        z - radius * t.normal[2],
                    ],
                },
            };
        }
        for k in 0..3 {
            let a = t.p[k];
            let b = t.p[(k + 1) % 3];
            let dx = b[0] - a[0];
            let dy = b[1] - a[1];
            let dz = b[2] - a[2];
            let l2 = dx * dx + dy * dy;
            let t = if l2 > 0. {
                let t0 = ((xy[0] - a[0]) * dx + (xy[1] - a[1]) * dy) / l2;
                // Cross-product perpendicular distance avoids cancellation.
                let perpendicular = ((xy[0] - a[0]) * dy - (xy[1] - a[1]) * dx).powi(2) / l2;
                if perpendicular > radius * radius {
                    continue;
                }
                (t0 + dz * (radius * radius - perpendicular).max(0.).sqrt()
                    / (l2 * (l2 + dz * dz)).sqrt())
                .clamp(0., 1.)
            } else if dz > 0. {
                1.
            } else {
                0.
            };
            let q = add(a, mul(sub(b, a), t));
            let d2 = (xy[0] - q[0]).powi(2) + (xy[1] - q[1]).powi(2);
            if d2 <= radius * radius {
                let z = q[2] + (radius * radius - d2).max(0.).sqrt();
                if z > result.center_z {
                    *result = Contact {
                        center_z: z,
                        support: Support::Point(q),
                    };
                }
            }
        }
    }
    fn height_at(&self, p: Vec3) -> Option<f64> {
        let area = orientation(self.p[0], self.p[1], self.p[2], 2);
        if area == 0. || !inside(self.p, p, 2) {
            return None;
        }
        let a = orientation(self.p[1], self.p[2], p, 2) / area;
        let b = orientation(self.p[2], self.p[0], p, 2) / area;
        Some(a * self.p[0][2] + b * self.p[1][2] + (1. - a - b) * self.p[2][2])
    }
}
#[derive(Clone, Copy, Debug)]
pub enum DistanceWitness {
    Base,
    Prism(usize),
}
#[derive(Clone, Copy, Debug)]
pub struct PointDistance {
    pub distance_mm: f64,
    pub witness: DistanceWitness,
}
/// A ball's contact with the solid at a fixed level (`level_contact`).
#[derive(Clone, Copy, Debug)]
pub struct LevelContact {
    /// Distance from the ball's center to the solid.
    pub distance_mm: f64,
    /// Unit normal from the nearest point of the solid to the center.
    pub normal: Vec3,
    /// Least and greatest z component of the contact normals of every prism
    /// within the tie distance of the nearest.
    pub cos_axis: [f64; 2],
}
#[derive(Clone, Copy)]
pub enum Support {
    Base,
    Face { index: usize, point: Vec3 },
    Point(Vec3),
}
#[derive(Clone, Copy)]
pub struct Contact {
    pub center_z: f64,
    support: Support,
}
impl Contact {
    /// Angle (radians) between the tool axis and the ball's contact normal:
    /// 0 on flat areas (and the base), pi/2 against a vertical wall. This is
    /// the usual CAM contact (slope) angle.
    /// The contact is on the machining base, not on the part.
    pub fn on_base(&self) -> bool {
        matches!(self.support, Support::Base)
    }
    /// Unit contact normal (from the contact point to the ball center).
    pub fn contact_normal(&self, xy: [f64; 2], radius: f64) -> [f64; 3] {
        match self.support {
            Support::Base => [0., 0., 1.],
            Support::Face { point, .. } | Support::Point(point) => [
                (xy[0] - point[0]) / radius,
                (xy[1] - point[1]) / radius,
                (self.center_z - point[2]) / radius,
            ],
        }
    }
    pub fn contact_angle(&self, radius: f64) -> f64 {
        match self.support {
            Support::Base => 0.,
            Support::Face { point, .. } | Support::Point(point) => {
                ((self.center_z - point[2]) / radius).clamp(-1., 1.).acos()
            }
        }
    }
}
#[derive(Clone)]
struct Node {
    min: Vec3,
    max: Vec3,
    range: std::ops::Range<usize>,
    children: Option<[usize; 2]>,
    contact_plane: Option<PlaneBound>,
    horizontal_top: Option<f64>,
}
// Preparation work is charged before each facet/node visit, projected vertex,
// sort comparison/swap, and reserved triangle/node record. A sort needs no
// auxiliary allocation. The same counter can continue into material_bounds.
fn preparation_charge(
    work: &mut u64,
    limit: u64,
    amount: u64,
    cancel: &impl CancelSignal,
) -> Result<(), String> {
    if cancel.is_cancelled() {
        return Err("surface preparation cancelled".into());
    }
    if amount > limit.saturating_sub(*work) {
        return Err("surface preparation work limit; no prepared surface committed".into());
    }
    *work += amount;
    Ok(())
}
fn preparation_node_capacity(triangles: usize) -> Result<usize, String> {
    // Balanced halves at a level have floor(n/width) or ceil(n/width) facets.
    // Count full levels, then only the remaining nine-facet nodes split.
    // This O(log n) metadata calculation visits no facets and allocates nothing.
    let overflow = || "surface preparation node capacity overflow".to_string();
    let mut width = 1usize;
    while triangles / width > 8 {
        width = width.checked_mul(2).ok_or_else(overflow)?;
    }
    let extra = if triangles / width == 8 {
        2 * (triangles % width)
    } else {
        0
    };
    width
        .checked_mul(2)
        .and_then(|n| n.checked_sub(1))
        .and_then(|n| n.checked_add(extra))
        .ok_or_else(overflow)
}
fn preparation_bytes(triangles: usize, nodes: usize) -> u128 {
    triangles as u128 * std::mem::size_of::<Triangle>() as u128
        + nodes as u128 * std::mem::size_of::<Node>() as u128
}
fn preparation_memory(triangles: usize, nodes: usize) -> Result<(), String> {
    crate::resources::check_memory(preparation_bytes(triangles, nodes), "surface preparation")
}
fn surface_order(a: &Triangle, b: &Triangle, axis: usize) -> std::cmp::Ordering {
    (a.min[axis] + a.max[axis]).total_cmp(&(b.min[axis] + b.max[axis]))
}
fn sift_preparation_heap<T>(
    values: &mut [T],
    mut root: usize,
    end: usize,
    compare: &impl Fn(&T, &T) -> std::cmp::Ordering,
    work: &mut u64,
    limit: u64,
    cancel: &impl CancelSignal,
) -> Result<(), String> {
    while root < end / 2 {
        let mut child = root * 2 + 1;
        if child + 1 < end {
            preparation_charge(work, limit, 1, cancel)?;
            if compare(&values[child], &values[child + 1]).is_lt() {
                child += 1;
            }
        }
        preparation_charge(work, limit, 1, cancel)?;
        if !compare(&values[root], &values[child]).is_lt() {
            break;
        }
        preparation_charge(work, limit, 1, cancel)?;
        values.swap(root, child);
        root = child;
    }
    Ok(())
}
fn sort_preparation<T>(
    values: &mut [T],
    compare: impl Fn(&T, &T) -> std::cmp::Ordering,
    work: &mut u64,
    limit: u64,
    cancel: &impl CancelSignal,
    legacy_sort: bool,
) -> Result<(), String> {
    if legacy_sort {
        // Preserve the ordinary prepare API's exact existing unstable-sort tie
        // order. Polling never changes the comparator's consistent total order.
        let mut failure = None;
        values.sort_unstable_by(|a, b| {
            if failure.is_none() {
                failure = preparation_charge(work, limit, 1, cancel).err();
            }
            compare(a, b)
        });
        return failure.map_or(Ok(()), Err);
    }
    // In-place heapsort permits refusal BEFORE every comparison and swap.
    // O(n log n) time, O(1) sort scratch; cancellation never changes ordering.
    for root in (0..values.len() / 2).rev() {
        sift_preparation_heap(values, root, values.len(), &compare, work, limit, cancel)?;
    }
    for end in (1..values.len()).rev() {
        preparation_charge(work, limit, 1, cancel)?;
        values.swap(0, end);
        sift_preparation_heap(values, 0, end, &compare, work, limit, cancel)?;
    }
    Ok(())
}
pub struct PreparedSurface {
    triangles: Vec<Triangle>,
    nodes: Vec<Node>,
    floor: f64,
    convex_solid: std::sync::OnceLock<bool>,
}
impl SurfaceMesh {
    pub(crate) fn validate_input(&self, floor: f64, tolerance: f64) -> Result<(), String> {
        if self.version != 1 {
            return Err(format!(
                "surface mesh version {} is unsupported; expected 1",
                self.version
            ));
        }
        if self.units != "mm" {
            return Err(format!(
                "surface mesh units {:?} are unsupported; expected mm",
                self.units
            ));
        }
        if !self.surface_error_mm.is_finite() || !(0. ..=0.1).contains(&self.surface_error_mm) {
            return Err(format!(
                "surface_error_mm {} must be finite and in 0..=0.1 mm; supply a mesh within that error limit",
                self.surface_error_mm
            ));
        }
        if self.triangles.is_empty() {
            return Err("surface mesh triangle count 0; supply at least one triangle".into());
        }
        if !floor.is_finite() {
            return Err(format!("surface base plane {floor} must be finite"));
        }
        if !tolerance.is_finite() || tolerance <= 0. {
            return Err(format!(
                "surface tolerance {tolerance} must be finite and positive"
            ));
        }
        Ok(())
    }

    /// Shared native facet validation, including callers using another index.
    pub(crate) fn validate_triangle(
        p: [Vec3; 3],
        floor: f64,
        tolerance: f64,
    ) -> Result<(), String> {
        if p.iter().flatten().any(|x| !x.is_finite() || x.abs() > 1e6)
            || p.iter().any(|q| q[2] < floor)
        {
            return Err("surface vertices must be finite, bounded and above the base plane".into());
        }
        let area = cross(sub(p[1], p[0]), sub(p[2], p[0]));
        if dot(area, area).sqrt() <= tolerance * tolerance {
            return Err("degenerate surface triangle".into());
        }
        Ok(())
    }

    /// Clip facets at an explicit machining base. The existing infinite base
    /// plane dominates every discarded point at/below it, so ball contact and
    /// continuous clearance describe the union of retained prisms and that base.
    /// Never silently discard a positive-area sub-resolution facet.
    pub fn above_floor(
        &self,
        floor: f64,
        tolerance: f64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        self.validate_input(floor, tolerance)?;
        let mut triangles = Vec::new();
        for &triangle in self.triangles.iter() {
            if cancel.is_cancelled() {
                return Err("surface floor clipping cancelled".into());
            }
            if triangle
                .iter()
                .flatten()
                .any(|p| !p.is_finite() || p.abs() > 1e6)
            {
                return Err("surface floor clipping needs finite bounded vertices".into());
            }
            let area = cross(sub(triangle[1], triangle[0]), sub(triangle[2], triangle[0]));
            if dot(area, area).sqrt() <= tolerance * tolerance {
                return Err("degenerate source surface triangle".into());
            }
            let mut polygon = Vec::with_capacity(4);
            let mut push = |point| {
                if polygon.last() != Some(&point) {
                    polygon.push(point);
                }
            };
            for i in 0..3 {
                let a = triangle[i];
                let b = triangle[(i + 1) % 3];
                if a[2] >= floor {
                    push(a);
                }
                if (a[2] < floor && b[2] > floor) || (a[2] > floor && b[2] < floor) {
                    let fraction = (floor - a[2]) / (b[2] - a[2]);
                    push([
                        a[0] + fraction * (b[0] - a[0]),
                        a[1] + fraction * (b[1] - a[1]),
                        floor,
                    ]);
                }
            }
            if polygon.len() > 1 && polygon.first() == polygon.last() {
                polygon.pop();
            }
            for i in 1..polygon.len().saturating_sub(1) {
                let p = [polygon[0], polygon[i], polygon[i + 1]];
                // Points on the exact base are already represented by it.
                if p.iter().all(|p| p[2] == floor) {
                    continue;
                }
                let area = cross(sub(p[1], p[0]), sub(p[2], p[0]));
                if dot(area, area).sqrt() <= tolerance * tolerance {
                    return Err("surface floor clipping produced a sub-resolution facet".into());
                }
                triangles.push(p);
            }
        }
        if triangles.is_empty() {
            return Err("surface has no area above the machining bottom".into());
        }
        Ok(Self {
            version: self.version,
            units: self.units.clone(),
            triangles: triangles.into(),
            surface_error_mm: self.surface_error_mm,
            source: self.source.clone(),
        })
    }

    pub fn prepare(
        &self,
        floor: f64,
        tolerance: f64,
        cancel: &impl CancelSignal,
    ) -> Result<PreparedSurface, String> {
        self.prepare_with_budget(floor, tolerance, &mut 0, u64::MAX, cancel, true)
    }

    /// Prepare with one caller-owned work counter, suitable for continuing into
    /// geometry queries. Exhaustion returns no prepared surface. Work units are
    /// reserved triangle/node records, facet/node visits, projected vertices,
    /// and sort comparisons/swaps; every charge precedes its work/allocation.
    pub fn prepare_bounded(
        &self,
        floor: f64,
        tolerance: f64,
        work: &mut u64,
        max_work: u64,
        cancel: &impl CancelSignal,
    ) -> Result<PreparedSurface, String> {
        self.prepare_with_budget(floor, tolerance, work, max_work, cancel, false)
    }
    fn prepare_with_budget(
        &self,
        floor: f64,
        tolerance: f64,
        work: &mut u64,
        max_work: u64,
        cancel: &impl CancelSignal,
        legacy_sort: bool,
    ) -> Result<PreparedSurface, String> {
        self.validate_input(floor, tolerance)?;
        let count = self.triangles.len();
        let node_capacity = preparation_node_capacity(count)?;
        // Reserve allocation records and the complete facet validation/conversion
        // pass before either large allocation or the first facet examination.
        preparation_charge(
            work,
            max_work,
            (count as u64)
                .saturating_mul(2)
                .saturating_add(node_capacity as u64),
            cancel,
        )?;
        preparation_memory(count, node_capacity)?;
        let mut triangles = Vec::with_capacity(count);
        let nodes = Vec::with_capacity(node_capacity);
        for &p in self.triangles.iter() {
            if cancel.is_cancelled() {
                return Err("surface preparation cancelled".into());
            }
            Self::validate_triangle(p, floor, tolerance)?;
            triangles.push(Triangle::new(p));
        }
        let mut result = PreparedSurface {
            triangles,
            nodes,
            floor,
            convex_solid: std::sync::OnceLock::new(),
        };
        result.build(0..count, work, max_work, cancel, legacy_sort)?;
        Ok(result)
    }
}
impl PreparedSurface {
    pub(crate) fn allocation_bytes(&self) -> u128 {
        preparation_bytes(self.triangles.capacity(), self.nodes.capacity())
    }
    pub fn prism_count(&self) -> usize {
        self.triangles.len()
    }
    /// The bounding-volume hierarchy's nodes.
    pub(crate) fn node_count(&self) -> usize {
        self.nodes.len()
    }
    /// Exact edge extrema of one convex prism's ball-offset slice. The slice
    /// support lies on a prism-edge cross section or its analytic stationary
    /// height, not on a sampled surface. See docs/waterline-support-plan.md.
    pub fn slice_support(
        &self,
        index: usize,
        height: f64,
        radius: f64,
        normal: Point,
        work: &mut SurfaceWork,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Point>, String> {
        work.check(cancel)?;
        if !height.is_finite()
            || height.abs() > 1e6
            || !radius.is_finite()
            || radius <= 0.
            || radius > 1e6
            || height <= self.floor + radius
            || normal.iter().any(|x| !x.is_finite())
            || (normal[0].hypot(normal[1]) - 1.).abs() > 1e-9
        {
            return Err("invalid convex-slice height/radius/unit normal".into());
        }
        let triangle = self
            .triangles
            .get(index)
            .ok_or("invalid convex-slice prism index")?;
        if triangle.max[2] < height - radius {
            return Ok(None);
        }
        let p = triangle.p;
        let v = [
            p[0],
            p[1],
            p[2],
            [p[0][0], p[0][1], self.floor],
            [p[1][0], p[1][1], self.floor],
            [p[2][0], p[2][1], self.floor],
        ];
        let mut best = f64::NEG_INFINITY;
        let mut result = None;
        for (i, j) in [
            (0, 1),
            (1, 2),
            (2, 0),
            (3, 4),
            (4, 5),
            (5, 3),
            (0, 3),
            (1, 4),
            (2, 5),
        ] {
            work.contact_tests += 1;
            let a = v[i];
            let b = v[j];
            let d = sub(b, a);
            let mut evaluate = |t: f64| {
                let q = add(a, mul(d, t.clamp(0., 1.)));
                let gap = height - q[2];
                let rounding = 16. * f64::EPSILON * height.abs().max(radius).max(1.);
                if gap < -rounding || gap > radius + rounding {
                    return;
                }
                let gap = gap.clamp(0., radius);
                let horizontal = ((radius - gap) * (radius + gap)).max(0.).sqrt();
                let point = [q[0] + horizontal * normal[0], q[1] + horizontal * normal[1]];
                let value = point[0] * normal[0] + point[1] * normal[1];
                if value > best {
                    best = value;
                    result = Some(point);
                }
            };
            if d[2] == 0. {
                evaluate(0.);
                evaluate(1.);
                continue;
            }
            let low = a[2].min(b[2]).max(height - radius);
            let high = a[2].max(b[2]).min(height);
            if low > high {
                continue;
            }
            evaluate((low - a[2]) / d[2]);
            evaluate((high - a[2]) / d[2]);
            let slope = (normal[0] * d[0] + normal[1] * d[1]) / d[2];
            if slope < 0. {
                let ratio = if slope.is_infinite() {
                    -1.
                } else {
                    slope / slope.hypot(1.)
                };
                let z = (height + radius * ratio).clamp(low, high);
                evaluate((z - a[2]) / d[2]);
            }
        }
        work.check(cancel)?;
        Ok(result)
    }
    fn prism_distance_sq(&self, p: Vec3, index: usize, work: &mut SurfaceWork) -> f64 {
        let t = &self.triangles[index];
        if p[2] >= self.floor && t.height_at(p).is_some_and(|z| p[2] <= z) {
            return 0.;
        }
        work.distance_tests += 7;
        let mut result = point_triangle_sq(p, t.p);
        for k in 0..3 {
            result = result.min(point_wall_sq(
                p,
                t.p[k],
                t.p[(k + 1) % 3],
                self.floor,
                result,
            ));
        }
        result
    }
    pub fn witness_distance(
        &self,
        p: Vec3,
        witness: DistanceWitness,
        work: &mut SurfaceWork,
    ) -> f64 {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("surface.witness_distance");
        match witness {
            DistanceWitness::Base => (p[2] - self.floor).max(0.),
            DistanceWitness::Prism(i) => self.prism_distance_sq(p, i, work).sqrt(),
        }
    }
    /// Distance to the retained SOLID, including zero for interior points.
    pub fn point_distance(
        &self,
        p: Vec3,
        work: &mut SurfaceWork,
        cancel: &impl CancelSignal,
    ) -> Result<PointDistance, String> {
        Ok(self.distance(p, None, work, cancel)?.unwrap())
    }
    /// A fixed-level offset ignores the base (checked separately) and prisms
    /// wholly below H-radius: neither can contribute to this level's boundary.
    pub fn level_distance(
        &self,
        p: Vec3,
        radius: f64,
        work: &mut SurfaceWork,
        cancel: &impl CancelSignal,
    ) -> Result<Option<PointDistance>, String> {
        if !radius.is_finite() || radius <= 0. || p[2] <= self.floor + radius {
            return Err("waterline center must be above the offset base".into());
        }
        self.distance(p, Some(p[2] - radius), work, cancel)
    }
    /// The contact of a ball centered at `p` on a fixed level with the
    /// retained solid: the nearest point of the solid and the unit normal
    /// from it to `p`, as `level_distance` finds it (the base and prisms
    /// wholly below `p[2] - radius` cannot touch a ball of that radius there).
    /// `cos_axis` is the range of the normal's z component over every prism
    /// within `tie` of the nearest: a ball resting against a wall and a floor
    /// at once touches both. A side of a prism gives exactly 0 (a vertical
    /// wall), a facet its slope, an edge the angle the ball rolls over it at.
    /// `None` when nothing is within reach of the level.
    pub fn level_contact(
        &self,
        p: Vec3,
        radius: f64,
        tie: f64,
        work: &mut SurfaceWork,
        cancel: &impl CancelSignal,
    ) -> Result<Option<LevelContact>, String> {
        let Some(nearest) = self.level_distance(p, radius, work, cancel)? else {
            return Ok(None);
        };
        let DistanceWitness::Prism(index) = nearest.witness else {
            return Ok(None);
        };
        let distance = nearest.distance_mm;
        let normal_of = |q: Vec3, d: f64| -> Vec3 {
            if d > 0. {
                [(p[0] - q[0]) / d, (p[1] - q[1]) / d, (p[2] - q[2]) / d]
            } else {
                // Inside the solid: no contact normal; the axis stands in.
                [0., 0., 1.]
            }
        };
        let (d2, q) = self.prism_closest(p, index, work);
        let normal = normal_of(q, d2.sqrt());
        let mut cos_axis = [normal[2], normal[2]];
        let min_top = p[2] - radius;
        let limit = distance + tie;
        let limit_sq = limit * limit;
        let lower = |min: Vec3, max: Vec3| {
            (0..3)
                .map(|k| {
                    ((if k == 2 { self.floor } else { min[k] }) - p[k])
                        .max(p[k] - max[k])
                        .max(0.)
                        .powi(2)
                })
                .sum::<f64>()
        };
        let margin = 4096. * f64::EPSILON * (1. + limit_sq);
        let mut pending = vec![0usize];
        while let Some(i) = pending.pop() {
            work.nodes += 1;
            work.check(cancel)?;
            let node = &self.nodes[i];
            if node.max[2] < min_top || lower(node.min, node.max) > limit_sq + margin {
                continue;
            }
            if let Some([a, b]) = node.children {
                pending.push(a);
                pending.push(b);
                continue;
            }
            for ti in node.range.clone() {
                let t = &self.triangles[ti];
                if ti == index || t.max[2] < min_top || lower(t.min, t.max) > limit_sq + margin {
                    continue;
                }
                let (d2, q) = self.prism_closest(p, ti, work);
                if d2 <= limit_sq {
                    let z = normal_of(q, d2.sqrt())[2];
                    cos_axis = [cos_axis[0].min(z), cos_axis[1].max(z)];
                }
            }
        }
        Ok(Some(LevelContact {
            distance_mm: distance,
            normal,
            cos_axis,
        }))
    }
    /// Squared distance from `p` to prism `index` and its nearest point
    /// (`p` itself inside the prism).
    fn prism_closest(&self, p: Vec3, index: usize, work: &mut SurfaceWork) -> (f64, Vec3) {
        let t = &self.triangles[index];
        if p[2] >= self.floor && t.height_at(p).is_some_and(|z| p[2] <= z) {
            return (0., p);
        }
        work.distance_tests += 7;
        let mut best = closest_on_triangle(p, t.p);
        for k in 0..3 {
            let (a, b) = (t.p[k], t.p[(k + 1) % 3]);
            let (af, bf) = ([a[0], a[1], self.floor], [b[0], b[1], self.floor]);
            // The wall below edge a-b: two triangles in one vertical plane.
            for wall in [[a, b, bf], [a, bf, af]] {
                let candidate = closest_on_triangle(p, wall);
                if candidate.0 < best.0 {
                    best = candidate;
                }
            }
        }
        best
    }
    fn distance(
        &self,
        p: Vec3,
        min_top: Option<f64>,
        work: &mut SurfaceWork,
        cancel: &impl CancelSignal,
    ) -> Result<Option<PointDistance>, String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("surface.nearest_distance");
        if p.iter().any(|x| !x.is_finite() || x.abs() > 1e6) {
            return Err("invalid surface distance point".into());
        }
        let mut best = if min_top.is_none() {
            (p[2] - self.floor).max(0.).powi(2)
        } else {
            f64::INFINITY
        };
        let mut witness = DistanceWitness::Base;
        // Preserve the previous fixed-DFS witness on EXACT distance ties:
        // leaves were visited in descending preorder-node id, then ascending
        // triangle index. The base preceded every prism. No fuzzy tie changes.
        let mut priority = (0, 0);
        let lower = |node: &Node| {
            if min_top.is_some_and(|h| node.max[2] < h) {
                return f64::INFINITY;
            }
            (0..3)
                .map(|k| {
                    ((if k == 2 { self.floor } else { node.min[k] }) - p[k])
                        .max(p[k] - node.max[k])
                        .max(0.)
                        .powi(2)
                })
                .sum::<f64>()
        };
        // Depth-first, nearer child first, pruned by the best distance so
        // far. Every node that could hold an equal distance is still visited
        // (pruning is strict), so the explicit tie priority below decides.
        let mut pending: Vec<(f64, usize)> = Vec::with_capacity(64);
        work.nodes += 1;
        work.check(cancel)?;
        pending.push((lower(&self.nodes[0]), 0));
        while let Some((bound, i)) = pending.pop() {
            work.check(cancel)?;
            if bound > best || bound == f64::INFINITY {
                continue;
            }
            let node = &self.nodes[i];
            if let Some([a, b]) = node.children {
                work.nodes += 2;
                work.check(cancel)?;
                let (la, lb) = (lower(&self.nodes[a]), lower(&self.nodes[b]));
                if la <= lb {
                    pending.push((lb, b));
                    pending.push((la, a));
                } else {
                    pending.push((la, a));
                    pending.push((lb, b));
                }
                continue;
            }
            for ti in node.range.clone() {
                let t = &self.triangles[ti];
                if min_top.is_some_and(|h| t.max[2] < h) {
                    continue;
                }
                // The prism lies in its triangle's box extended down to the
                // floor: farther than the best already, it cannot win or tie.
                let box_sq = (0..3)
                    .map(|k| {
                        ((if k == 2 { self.floor } else { t.min[k] }) - p[k])
                            .max(p[k] - t.max[k])
                            .max(0.)
                            .powi(2)
                    })
                    .sum::<f64>();
                // Margin: the bound and the distance round differently.
                if box_sq > best + 4096. * f64::EPSILON * (1. + best) {
                    continue;
                }
                let distance = self.prism_distance_sq(p, ti, work);
                let candidate_priority = (usize::MAX - i, ti);
                if distance < best || (distance == best && candidate_priority < priority) {
                    best = distance;
                    witness = DistanceWitness::Prism(ti);
                    priority = candidate_priority;
                }
                // A zero-distance prism may tie another containing prism;
                // continue until all possible ties have the old DFS priority.
            }
        }
        work.check(cancel)?;
        Ok(best.is_finite().then_some(PointDistance {
            distance_mm: best.sqrt(),
            witness,
        }))
    }
    fn build(
        &mut self,
        range: std::ops::Range<usize>,
        work: &mut u64,
        max_work: u64,
        cancel: &impl CancelSignal,
        legacy_sort: bool,
    ) -> Result<usize, String> {
        preparation_charge(work, max_work, 1, cancel)?;
        let (mut min, mut max) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        let mut normal = [0.; 3];
        let mut horizontal_top: Option<f64> = None;
        for t in &self.triangles[range.clone()] {
            preparation_charge(work, max_work, 1, cancel)?;
            for i in 0..3 {
                min[i] = min[i].min(t.min[i]);
                max[i] = max[i].max(t.max[i]);
            }
            normal = add(normal, t.normal);
            if t.p[0][2] == t.p[1][2]
                && t.p[1][2] == t.p[2][2]
                && horizontal_top.is_none_or(|z| !t.p[0][2].total_cmp(&z).is_lt())
            {
                horizontal_top = Some(t.p[0][2]);
            }
        }
        // MAX projection over every vertex establishes the enclosing halfspace.
        // A partial plane is discarded together with the entire preparation.
        let mut failure = None;
        let points = self.triangles[range.clone()]
            .iter()
            .flat_map(|t| t.p)
            .map_while(|p| match preparation_charge(work, max_work, 1, cancel) {
                Ok(()) => Some(p),
                Err(error) => {
                    failure = Some(error);
                    None
                }
            });
        let contact_plane = PlaneBound::new(normal, points, self.triangles[range.start].p[0]);
        if let Some(error) = failure {
            return Err(error);
        }
        let id = self.nodes.len();
        self.nodes.push(Node {
            min,
            max,
            range: range.clone(),
            children: None,
            contact_plane,
            horizontal_top,
        });
        if range.len() > 8 {
            let axis = (0..3)
                .max_by(|&a, &b| (max[a] - min[a]).total_cmp(&(max[b] - min[b])))
                .unwrap();
            sort_preparation(
                &mut self.triangles[range.clone()],
                |a, b| surface_order(a, b, axis),
                work,
                max_work,
                cancel,
                legacy_sort,
            )?;
            let mid = range.start + range.len() / 2;
            let left = self.build(range.start..mid, work, max_work, cancel, legacy_sort)?;
            let right = self.build(mid..range.end, work, max_work, cancel, legacy_sort)?;
            self.nodes[id].children = Some([left, right]);
        }
        Ok(id)
    }
    pub fn max_z(&self) -> f64 {
        self.nodes[0].max[2]
    }
    /// Continuous filled-triangle clearance supports topology-preserving
    /// contour shortcuts, not merely checking the new chord's endpoints.
    pub fn triangle_clear(
        &self,
        t: [Vec3; 3],
        radius: f64,
        work: &mut SurfaceWork,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("surface.filled_triangle_clear");
        if !radius.is_finite() || radius <= 0. || t.iter().flatten().any(|v| !v.is_finite()) {
            return Err("invalid surface triangle clearance query".into());
        }
        if t.iter().any(|p| p[2] < self.floor + radius) {
            return Ok(false);
        }
        // Preserve the first-point coordinate validation previously performed
        // by the redundant global membership query.
        if t[0].iter().any(|v| v.abs() > 1e6) {
            return Err("invalid surface distance point".into());
        }
        let low: Vec3 = std::array::from_fn(|i| t[0][i].min(t[1][i]).min(t[2][i]));
        let high: Vec3 = std::array::from_fn(|i| t[0][i].max(t[1][i]).max(t[2][i]));
        let mut stack = vec![0];
        while let Some(i) = stack.pop() {
            work.nodes += 1;
            work.check(cancel)?;
            let n = &self.nodes[i];
            // These boxes contain the WHOLE filled query triangle and source
            // prisms, so the existing distance lower bound applies unchanged.
            if segment_box_separates(low, high, n.min, n.max, self.floor, radius) {
                continue;
            }
            if n.contact_plane
                .as_ref()
                .is_some_and(|plane| plane.separates(t, radius))
            {
                continue;
            }
            if let Some(children) = n.children {
                stack.extend(children);
                continue;
            }
            for index in n.range.clone() {
                let source_triangle = &self.triangles[index];
                work.distance_tests += 1;
                work.check(cancel)?;
                if segment_box_separates(
                    low,
                    high,
                    source_triangle.min,
                    source_triangle.max,
                    self.floor,
                    radius,
                ) {
                    continue;
                }
                work.distance_tests += 1;
                work.check(cancel)?;
                if source_triangle.separates_triangle(t, radius, self.floor) {
                    continue;
                }
                // Membership is checked against EVERY candidate filled prism.
                // Then complete triangle/facet distances cover crossing its
                // boundary. Neither a boundary-only nor endpoint-only proof.
                if t[0][2] >= self.floor
                    && source_triangle
                        .height_at(t[0])
                        .is_some_and(|z| t[0][2] <= z)
                {
                    return Ok(false);
                }
                let source = source_triangle.p;
                let clear = |facet: [Vec3; 3], work: &mut SurfaceWork| {
                    work.distance_tests += 6;
                    let normal = cross(sub(facet[1], facet[0]), sub(facet[2], facet[0]));
                    if dot(normal, normal) <= 1e-30 {
                        return true;
                    }
                    (0..3).all(|j| {
                        segment_triangle_sq(t[j], t[(j + 1) % 3], facet) >= radius * radius
                            && segment_triangle_sq(facet[j], facet[(j + 1) % 3], t)
                                >= radius * radius
                    })
                };
                if !clear(source, work) {
                    return Ok(false);
                }
                for j in 0..3 {
                    let a = source[j];
                    let b = source[(j + 1) % 3];
                    let af = [a[0], a[1], self.floor];
                    let bf = [b[0], b[1], self.floor];
                    if !clear([a, b, bf], work) || !clear([a, bf, af], work) {
                        return Ok(false);
                    }
                }
            }
        }
        work.check(cancel)?;
        Ok(true)
    }
    /// Build an optional contact-only index. A facet whose highest possible
    /// ball center is below the independently enforced minimum cannot affect
    /// max(contact, minimum). Retain original facet coordinates and base plane.
    /// This index must not be used for raw target or gouge classification.
    pub(crate) fn contact_window(
        &self,
        minimum: f64,
        maximum_radius: f64,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Self>, String> {
        let keep = |t: &Triangle| {
            let guard =
                4096. * f64::EPSILON * (1. + minimum.abs() + maximum_radius + t.max[2].abs());
            t.max[2] + maximum_radius + guard >= minimum
        };
        let (mut count, mut highest) = (0usize, 0usize);
        for (index, t) in self.triangles.iter().enumerate() {
            if cancel.is_cancelled() {
                return Err("surface contact preparation cancelled".into());
            }
            count += usize::from(keep(t));
            if !t.max[2].total_cmp(&self.triangles[highest].max[2]).is_lt() {
                highest = index;
            }
        }
        if count == self.triangles.len() {
            return Ok(None);
        }
        let capacity = count.max(1);
        let node_capacity = preparation_node_capacity(capacity)?;
        preparation_memory(capacity, node_capacity)?;
        let mut triangles = Vec::with_capacity(capacity);
        if count == 0 {
            // Harmless below-floor facet for the normal nonempty BVH contract.
            triangles.push(self.triangles[highest].clone());
        } else {
            for t in &self.triangles {
                if cancel.is_cancelled() {
                    return Err("surface contact preparation cancelled".into());
                }
                if keep(t) {
                    triangles.push(t.clone());
                }
            }
        }
        let mut surface = Self {
            triangles,
            nodes: Vec::with_capacity(node_capacity),
            floor: self.floor,
            convex_solid: std::sync::OnceLock::new(),
        };
        surface.build(0..surface.triangles.len(), &mut 0, u64::MAX, cancel, true)?;
        Ok(Some(surface))
    }
    pub fn contact(
        &self,
        xy: [f64; 2],
        radius: f64,
        work: &mut SurfaceWork,
        cancel: &impl CancelSignal,
    ) -> Result<Contact, String> {
        let mut result = Contact {
            center_z: self.floor + radius,
            support: Support::Base,
        };
        let mut stack = vec![0];
        while let Some(i) = stack.pop() {
            work.nodes += 1;
            work.check(cancel)?;
            let n = &self.nodes[i];
            let d2 = (0..2)
                .map(|k| (n.min[k] - xy[k]).max(0.).max(xy[k] - n.max[k]).powi(2))
                .sum::<f64>();
            let upper = n.max[2] + (radius * radius - d2).max(0.).sqrt();
            let rounding =
                512. * f64::EPSILON * (1. + radius + n.max[2].abs() + xy[0].abs() + xy[1].abs());
            if d2 > radius * radius || upper + rounding < result.center_z {
                continue;
            }
            if n.contact_plane
                .as_ref()
                .is_some_and(|plane| plane.separates([[xy[0], xy[1], result.center_z]; 3], radius))
            {
                continue;
            }
            if let Some([left, right]) = n.children {
                let bound = |index: usize| {
                    let child = &self.nodes[index];
                    let d2: f64 = (0..2)
                        .map(|k| {
                            (child.min[k] - xy[k])
                                .max(0.)
                                .max(xy[k] - child.max[k])
                                .powi(2)
                        })
                        .sum();
                    if d2 > radius * radius {
                        f64::NEG_INFINITY
                    } else {
                        child.max[2] + (radius * radius - d2).max(0.).sqrt()
                    }
                };
                if bound(left) > bound(right) {
                    stack.extend([right, left]);
                } else {
                    stack.extend([left, right]);
                }
                continue;
            }
            for ti in n.range.clone() {
                work.contact_tests += 1;
                self.triangles[ti].raise_contact(ti, xy, radius, &mut result);
            }
        }
        Ok(result)
    }
    /// A union of exact horizontal face intervals can support a long flat cut
    /// across tessellation seams. Uncovered gaps use a spherical-cap bound,
    /// never an assumed filled polygon or endpoint-only coverage claim.
    pub fn horizontal_excess_bound(
        &self,
        a: Vec3,
        b: Vec3,
        radius: f64,
        max_error: f64,
        work: &mut SurfaceWork,
        cancel: &impl CancelSignal,
    ) -> Result<Option<f64>, String> {
        if (a[2] - b[2]).abs() > max_error {
            return Ok(None);
        }
        let length = (a[0] - b[0]).hypot(a[1] - b[1]);
        if length == 0. {
            return Ok(None);
        }
        let scale = 1. + a.iter().chain(&b).map(|v| v.abs()).sum::<f64>() + radius;
        let guard = 512. * f64::EPSILON * scale;
        let threshold = a[2].max(b[2]) - radius - max_error;
        let point = |t: f64, height: f64| {
            if t == 0. {
                [a[0], a[1], height]
            } else if t == 1. {
                [b[0], b[1], height]
            } else {
                [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), height]
            }
        };
        let mut intervals = vec![];
        let mut height = f64::INFINITY;
        let mut stack = vec![0];
        while let Some(index) = stack.pop() {
            work.nodes += 1;
            work.check(cancel)?;
            let node = &self.nodes[index];
            if node.horizontal_top.is_none_or(|z| z < threshold)
                || (0..2).any(|k| a[k].min(b[k]) > node.max[k] || a[k].max(b[k]) < node.min[k])
            {
                continue;
            }
            if let Some(children) = node.children {
                stack.extend(children);
                continue;
            }
            for index in node.range.clone() {
                work.contact_tests += 1;
                let triangle = &self.triangles[index];
                let p = triangle.p;
                if p[0][2] != p[1][2] || p[1][2] != p[2][2] || p[0][2] < threshold {
                    continue;
                }
                let sign = orientation(p[0], p[1], p[2], 2).signum();
                if sign == 0. {
                    continue;
                }
                let mut lo: f64 = 0.;
                let mut hi: f64 = 1.;
                for k in 0..3 {
                    let d0 = sign * orientation(p[k], p[(k + 1) % 3], a, 2);
                    let d1 = sign * orientation(p[k], p[(k + 1) % 3], b, 2);
                    if d0 < 0. && d1 < 0. {
                        lo = 1.;
                        hi = 0.;
                        break;
                    }
                    if d0 < 0. {
                        lo = lo.max(d0 / (d0 - d1));
                    }
                    if d1 < 0. {
                        hi = hi.min(d0 / (d0 - d1));
                    }
                }
                if lo >= hi {
                    continue;
                }
                // The calculated intersections are proposals. Move inward and
                // validate BOTH actual endpoints with the robust predicate.
                // Convexity then certifies the full interval between them.
                let mut inset = guard / length;
                let mut accepted = None;
                for _ in 0..8 {
                    let left = if lo > 0. { lo + inset } else { lo };
                    let right = if hi < 1. { hi - inset } else { hi };
                    if left >= right {
                        break;
                    }
                    if inside(p, point(left, p[0][2]), 2) && inside(p, point(right, p[0][2]), 2) {
                        accepted = Some((left, right));
                        break;
                    }
                    inset *= 2.;
                }
                if let Some(interval) = accepted {
                    intervals.push(interval);
                    height = height.min(p[0][2]);
                }
            }
        }
        if intervals.is_empty() {
            return Ok(None);
        }
        intervals.sort_unstable_by(|a, b| a.0.total_cmp(&b.0));
        let mut through: f64 = 0.;
        let mut gap: f64 = 0.;
        for (lo, hi) in intervals {
            gap = gap.max((lo - through).max(0.));
            through = through.max(hi);
        }
        gap = gap.max(1. - through);
        // Every gap point is within this distance of an endpoint already
        // certified on a source facet. Include interpolation roundoff too.
        let distance = gap * length + 2. * guard;
        if distance >= radius {
            return Ok(None);
        }
        let lower =
            (self.floor + radius).max(height + ((radius - distance) * (radius + distance)).sqrt());
        let excess = (a[2].max(b[2]) - lower).max(0.) + guard;
        work.check(cancel)?;
        Ok((excess <= max_error).then_some(excess))
    }
    /// Supporting graph bounds the entire segment, not a midpoint-error sample.
    pub fn excess_bound(&self, a: Vec3, b: Vec3, contact: Contact, radius: f64) -> Option<f64> {
        let support = if let Support::Face { index, point } = contact.support {
            let face = &self.triangles[index];
            if let (Some(za), Some(zb)) = (
                face.face_contact([a[0], a[1]], radius),
                face.face_contact([b[0], b[1]], radius),
            ) {
                return Some((a[2] - za).max(b[2] - zb).max(0.));
            }
            // A segment can straddle a coplanar triangulation seam at an
            // irrational coordinate: subdivision alone need never hit it.
            // The actual contact point remains a valid supporting sphere.
            Support::Point(point)
        } else {
            contact.support
        };
        let lower = |p: Vec3| match support {
            Support::Base => Some(self.floor + radius),
            Support::Face { .. } => unreachable!(),
            Support::Point(q) => {
                let d2 = (p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2);
                if d2 <= radius * radius {
                    Some((self.floor + radius).max(q[2] + (radius * radius - d2).max(0.).sqrt()))
                } else if self.floor + radius >= q[2] {
                    Some(self.floor + radius)
                } else {
                    None
                }
            }
        };
        let mut error = (a[2] - lower(a)?).max(b[2] - lower(b)?).max(0.);
        if let Support::Point(q) = support {
            let base = self.floor + radius;
            if base >= q[2] && base < q[2] + radius {
                // max(base, spherical cap) is not globally concave. Partition
                // at its exact base/cap switches; on each interval, linear
                // path height minus this lower graph is convex or affine, so
                // its maximum is attained at an interval endpoint. Merely
                // clamping the old endpoint formula to the base is unsound.
                let dx = b[0] - a[0];
                let dy = b[1] - a[1];
                let l2 = dx * dx + dy * dy;
                if l2 > 0. {
                    let active2 = radius * radius - (base - q[2]).powi(2);
                    let perpendicular = ((q[0] - a[0]) * dy - (q[1] - a[1]) * dx).powi(2) / l2;
                    if perpendicular <= active2 {
                        let middle = ((q[0] - a[0]) * dx + (q[1] - a[1]) * dy) / l2;
                        let half = ((active2 - perpendicular).max(0.) / l2).sqrt();
                        for t in [middle - half, middle + half] {
                            if (0. ..=1.).contains(&t) {
                                error = error.max(a[2] + t * (b[2] - a[2]) - base);
                            }
                        }
                    }
                }
                error += 64.
                    * f64::EPSILON
                    * (1. + radius + a.iter().chain(&b).chain(&q).map(|v| v.abs()).sum::<f64>());
            }
        }
        Some(error)
    }
    /// The target includes each triangle's downward solid, not just its skin.
    pub fn segment_clear(
        &self,
        a: Vec3,
        b: Vec3,
        radius: f64,
        work: &mut SurfaceWork,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        if !radius.is_finite()
            || radius <= 0.
            || a.iter().chain(&b).any(|v| !v.is_finite() || v.abs() > 1e6)
        {
            return Err("invalid surface segment clearance query".into());
        }
        if a[2].min(b[2]) < self.floor + radius {
            return Ok(false);
        }
        // Each candidate prism below checks the initial point for containment.
        // Its complete boundary checks then cover the connected segment. This
        // avoids a separate global nearest-distance traversal for every query.
        let low: Vec3 = std::array::from_fn(|i| a[i].min(b[i]));
        let high: Vec3 = std::array::from_fn(|i| a[i].max(b[i]));
        let mut stack = vec![0];
        while let Some(i) = stack.pop() {
            work.nodes += 1;
            work.check(cancel)?;
            let n = &self.nodes[i];
            if segment_box_separates(low, high, n.min, n.max, self.floor, radius) {
                continue;
            }
            if n.contact_plane
                .as_ref()
                .is_some_and(|plane| plane.separates([a, b, mul(add(a, b), 0.5)], radius))
            {
                continue;
            }
            if let Some(children) = n.children {
                stack.extend(children);
                continue;
            }
            for ti in n.range.clone() {
                let source = &self.triangles[ti];
                work.distance_tests += 1;
                if segment_box_separates(low, high, source.min, source.max, self.floor, radius) {
                    continue;
                }
                work.distance_tests += 1;
                let midpoint = mul(add(a, b), 0.5);
                if source.separates_triangle([a, b, midpoint], radius, self.floor) {
                    continue;
                }
                if a[2] >= self.floor && source.height_at(a).is_some_and(|z| a[2] <= z) {
                    return Ok(false);
                }
                let t = source.p;
                work.distance_tests += 1;
                if segment_triangle_sq(a, b, t) < radius * radius {
                    return Ok(false);
                }
                for k in 0..3 {
                    let c = t[k];
                    let d = t[(k + 1) % 3];
                    let cf = [c[0], c[1], self.floor];
                    let df = [d[0], d[1], self.floor];
                    for side in [[c, d, df], [c, df, cf]] {
                        work.distance_tests += 1;
                        let n = cross(sub(side[1], side[0]), sub(side[2], side[0]));
                        if dot(n, n) > 1e-30 && segment_triangle_sq(a, b, side) < radius * radius {
                            return Ok(false);
                        }
                    }
                }
            }
        }
        work.check(cancel)?;
        Ok(true)
    }
}
/// Both complete objects are contained in these boxes. Their separation is a
/// lower bound on segment/prism distance, including the downward sides. Keep
/// rounding uncertainty on the fallback side; this never substitutes samples
/// for a continuous clearance query.
fn segment_box_separates(
    low: Vec3,
    high: Vec3,
    source_min: Vec3,
    source_max: Vec3,
    floor: f64,
    radius: f64,
) -> bool {
    let mut distance_sq = 0.;
    let mut scale = 1. + radius + floor.abs();
    for k in 0..3 {
        let lower = if k == 2 { floor } else { source_min[k] };
        let gap = (low[k] - source_max[k]).max(lower - high[k]).max(0.);
        distance_sq += gap * gap;
        scale += low[k].abs() + high[k].abs() + lower.abs() + source_max[k].abs();
    }
    let guarded_radius = radius + 512. * f64::EPSILON * scale;
    distance_sq > guarded_radius * guarded_radius
}
fn point_segment_sq(p: Vec3, a: Vec3, b: Vec3) -> f64 {
    let d = sub(b, a);
    let l2 = dot(d, d);
    let t = if l2 > 0. {
        (dot(sub(p, a), d) / l2).clamp(0., 1.)
    } else {
        0.
    };
    let v = sub(p, add(a, mul(d, t)));
    dot(v, v)
}
/// Squared distance from `p` to segment a-b and its nearest point.
fn closest_on_segment(p: Vec3, a: Vec3, b: Vec3) -> (f64, Vec3) {
    let d = sub(b, a);
    let l2 = dot(d, d);
    let t = if l2 > 0. {
        (dot(sub(p, a), d) / l2).clamp(0., 1.)
    } else {
        0.
    };
    let q = add(a, mul(d, t));
    let v = sub(p, q);
    (dot(v, v), q)
}
/// Squared distance from `p` to triangle `t` and its nearest point (a
/// degenerate triangle is its edges).
fn closest_on_triangle(p: Vec3, t: [Vec3; 3]) -> (f64, Vec3) {
    let n = cross(sub(t[1], t[0]), sub(t[2], t[0]));
    let nn = dot(n, n);
    if nn > 0. {
        let distance = dot(n, sub(p, t[0]));
        let q = sub(p, mul(n, distance / nn));
        let omit = if n[0].abs() >= n[1].abs() && n[0].abs() >= n[2].abs() {
            0
        } else if n[1].abs() >= n[2].abs() {
            1
        } else {
            2
        };
        if inside_fast(t, q, omit) {
            return (distance * distance / nn, q);
        }
    }
    let mut best = closest_on_segment(p, t[0], t[1]);
    for (a, b) in [(t[1], t[2]), (t[2], t[0])] {
        let candidate = closest_on_segment(p, a, b);
        if candidate.0 < best.0 {
            best = candidate;
        }
    }
    best
}
fn point_triangle_sq(p: Vec3, t: [Vec3; 3]) -> f64 {
    let n = cross(sub(t[1], t[0]), sub(t[2], t[0]));
    let nn = dot(n, n);
    if nn > 0. {
        let distance = dot(n, sub(p, t[0]));
        let q = sub(p, mul(n, distance / nn));
        let omit = if n[0].abs() >= n[1].abs() && n[0].abs() >= n[2].abs() {
            0
        } else if n[1].abs() >= n[2].abs() {
            1
        } else {
            2
        };
        // Floating-point containment suffices: distance is continuous across
        // the triangle boundary, so a borderline classification changes the
        // result only by rounding (the exact predicate stays in `height_at`).
        if inside_fast(t, q, omit) {
            return distance * distance / nn;
        }
    }
    point_segment_sq(p, t[0], t[1])
        .min(point_segment_sq(p, t[1], t[2]))
        .min(point_segment_sq(p, t[2], t[0]))
}
fn inside_fast(t: [Vec3; 3], p: Vec3, omit: usize) -> bool {
    let (u, v) = ((omit + 1) % 3, (omit + 2) % 3);
    let side = |a: Vec3, b: Vec3| (b[u] - a[u]) * (p[v] - a[v]) - (b[v] - a[v]) * (p[u] - a[u]);
    let s = [side(t[0], t[1]), side(t[1], t[2]), side(t[2], t[0])];
    s.iter().all(|x| *x >= 0.) || s.iter().all(|x| *x <= 0.)
}
/// 2D point/segment squared distance.
fn point_segment_sq_2d(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1]];
    let l2 = d[0] * d[0] + d[1] * d[1];
    let t = if l2 > 0. {
        (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / l2).clamp(0., 1.)
    } else {
        0.
    };
    let (x, y) = (p[0] - a[0] - d[0] * t, p[1] - a[1] - d[1] * t);
    x * x + y * y
}
/// Squared distance from `p` to the vertical wall below top edge a-b, i.e.
/// the union of triangles [a, b, b_floor] and [a, b_floor, a_floor]. Both lie
/// in one vertical plane, so the 3D distance splits exactly into the
/// horizontal offset from that plane plus a 2D distance within it.
/// `known` is a distance already found: a wall whose plane alone is farther
/// returns it unchanged (the caller takes the minimum).
fn point_wall_sq(p: Vec3, a: Vec3, b: Vec3, floor: f64, known: f64) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let length = dx.hypot(dy);
    if length == 0. {
        // Degenerate wall: both triangles collapse to vertical segments.
        let af = [a[0], a[1], floor];
        let bf = [b[0], b[1], floor];
        return point_triangle_sq(p, [a, b, bf]).min(point_triangle_sq(p, [a, bf, af]));
    }
    let (ux, uy) = (dx / length, dy / length);
    let (rx, ry) = (p[0] - a[0], p[1] - a[1]);
    let along = rx * ux + ry * uy;
    let across = ry * ux - rx * uy;
    if across * across >= known {
        return known;
    }
    let q = [along, p[2]];
    across * across + point_trapezoid_sq_2d(q, length, a[2], b[2], floor)
}
/// 2D squared distance to the wall trapezoid (0, floor), (length, floor),
/// (length, zb), (0, za) with za, zb >= floor: the union of the wall's two
/// triangles. Zero inside; otherwise the nearest of its edges (only the top
/// edge when the point lies above it within the wall's span).
fn point_trapezoid_sq_2d(q: [f64; 2], length: f64, za: f64, zb: f64, floor: f64) -> f64 {
    let (top_a, top_b) = ([0., za], [length, zb]);
    if (0. ..=length).contains(&q[0]) && q[1] >= floor {
        let top = za + (zb - za) * (q[0] / length);
        if q[1] <= top {
            return 0.;
        }
        return point_segment_sq_2d(q, top_a, top_b);
    }
    point_segment_sq_2d(q, top_a, top_b)
        .min(point_segment_sq_2d(q, [0., floor], top_a))
        .min(point_segment_sq_2d(q, [length, floor], top_b))
        .min(point_segment_sq_2d(q, [0., floor], [length, floor]))
}
fn segments_sq(p: Vec3, q: Vec3, a: Vec3, b: Vec3) -> f64 {
    let u = sub(q, p);
    let v = sub(b, a);
    let w = sub(p, a);
    let mut result = point_segment_sq(p, a, b)
        .min(point_segment_sq(q, a, b))
        .min(point_segment_sq(a, p, q))
        .min(point_segment_sq(b, p, q));
    // Cross products retain near-parallel angles lost by uu*vv-uv*uv.
    let n = cross(u, v);
    let denominator = dot(n, n);
    if denominator > 0. {
        let s = dot(cross(mul(w, -1.), v), n) / denominator;
        let t = dot(cross(mul(w, -1.), u), n) / denominator;
        if (0. ..=1.).contains(&s) && (0. ..=1.).contains(&t) {
            let d = sub(add(p, mul(u, s)), add(a, mul(v, t)));
            result = result.min(dot(d, d));
        }
    }
    result
}
/// Independent continuous segment/facet distance; no motion discretization.
pub fn segment_triangle_sq(a: Vec3, b: Vec3, t: [Vec3; 3]) -> f64 {
    let n = cross(sub(t[1], t[0]), sub(t[2], t[0]));
    let da = dot(n, sub(a, t[0]));
    let db = dot(n, sub(b, t[0]));
    if da != db && ((da <= 0. && db >= 0.) || (db <= 0. && da >= 0.)) {
        let q = add(a, mul(sub(b, a), da / (da - db)));
        let omit = (0..3)
            .max_by(|&i, &j| n[i].abs().total_cmp(&n[j].abs()))
            .unwrap();
        if inside(t, q, omit) {
            return 0.;
        }
    }
    let mut d = point_triangle_sq(a, t).min(point_triangle_sq(b, t));
    for i in 0..3 {
        d = d.min(segments_sq(a, b, t[i], t[(i + 1) % 3]));
    }
    d
}

#[cfg(test)]
mod clearance_tests;

#[cfg(test)]
mod wire_tests;

#[cfg(test)]
mod fast_distance_tests {
    use super::*;
    fn reference_point_triangle_sq(p: Vec3, t: [Vec3; 3]) -> f64 {
        let n = cross(sub(t[1], t[0]), sub(t[2], t[0]));
        let nn = dot(n, n);
        if nn > 0. {
            let distance = dot(n, sub(p, t[0]));
            let q = sub(p, mul(n, distance / nn));
            let omit = (0..3)
                .max_by(|&i, &j| n[i].abs().total_cmp(&n[j].abs()))
                .unwrap();
            if inside(t, q, omit) {
                return distance * distance / nn;
            }
        }
        (0..3)
            .map(|i| point_segment_sq(p, t[i], t[(i + 1) % 3]))
            .fold(f64::INFINITY, f64::min)
    }
    #[test]
    fn fast_prism_kernels_match_the_exact_reference_formulation() {
        let mut seed = 0x9e37_79b9_7f4a_7c15u64;
        let mut rnd = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        let mut worst = 0f64;
        for case in 0..1_000_000 {
            let scale = [1e-3, 1., 50.][case % 3];
            let mut v = || {
                [
                    (rnd() - 0.5) * scale,
                    (rnd() - 0.5) * scale,
                    (rnd() - 0.2) * scale,
                ]
            };
            let (a, b, c, p) = (v(), v(), v(), v());
            let floor = -0.3 * scale;
            // occasional degenerate and axis-aligned inputs
            let b = if case % 97 == 0 {
                [a[0], a[1], b[2]]
            } else {
                b
            };
            let reference = reference_point_triangle_sq(p, [a, b, c]);
            let fast = point_triangle_sq(p, [a, b, c]);
            let (af, bf) = ([a[0], a[1], floor], [b[0], b[1], floor]);
            let wall_ref = reference_point_triangle_sq(p, [a, b, bf])
                .min(reference_point_triangle_sq(p, [a, bf, af]));
            let wall = point_wall_sq(p, a, b, floor, f64::INFINITY);
            for (x, y) in [(reference, fast), (wall_ref, wall)] {
                let tolerance = 1e-12 * scale * scale + 1e-9 * x.abs();
                worst = worst.max((x - y).abs() / (scale * scale));
                assert!((x - y).abs() <= tolerance, "case {case}: {x} vs {y}");
            }
        }
        assert!(worst < 1e-10, "worst relative difference {worst}");
    }
}

#[cfg(test)]
mod preparation_tests {
    use super::*;
    use std::cell::Cell;

    fn mesh(count: usize) -> SurfaceMesh {
        SurfaceMesh {
            version: 1,
            units: "mm".into(),
            triangles: vec![[[0., 0., 1.], [1., 0., 1.], [0., 1., 1.]]; count].into(),
            surface_error_mm: 0.,
            source: serde_json::Value::Null,
        }
    }

    #[test]
    fn node_capacity_covers_exactly_the_balanced_tree_without_reallocation() {
        for count in 1..=257 {
            let prepared = mesh(count).prepare(0., 1e-6, &|| false).unwrap();
            assert_eq!(
                prepared.nodes.capacity(),
                prepared.nodes.len(),
                "{count} facets"
            );
        }
    }

    #[test]
    fn bvh_storage_is_guarded_even_when_all_triangle_records_fit() {
        let count = 128;
        let mesh = mesh(count);
        let polls = Cell::new(0);
        let triangle_bytes = count * std::mem::size_of::<Triangle>();
        let error = crate::resources::with_memory_budget(triangle_bytes as u64, || {
            mesh.prepare_bounded(0., 1e-6, &mut 0, 1_000_000, &|| {
                polls.set(polls.get() + 1);
                false
            })
            .err()
            .unwrap()
        });
        assert!(error.contains("surface preparation needs"), "{error}");
        assert_eq!(polls.get(), 1, "examined facets before guarding BVH memory");
    }
}
