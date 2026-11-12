//! Tool and holder bodies from supplied models (#110).
//!
//! Some tool bodies are described by no code or parameter set: grooving
//! blade carriers, modified boring bars, holders with clamps or coolant
//! blocks that stick out. Their maker's 3D model is then the body to check.
//! The engine takes that model as a mesh (`SurfaceMesh`, the input type of
//! parts) with its declared `surface_error_mm`; it reads no CAD file (STEP
//! comes through `camlib.cadquery`, which tessellates to a stated error).
//!
//! What the model is. `ModelBody::body` is the tool away from the cut: the
//! holder, bar, shim, clamp, coolant block: everything but the insert. It
//! replaces the holder outline's body in every check and is judged by the
//! holder's rules (clear of the part, its bore, the chuck, and the stock
//! left after each cut). The insert stays the declared one (ISO code or
//! outline, with its thickness, clearance and seat): its exact outline is
//! what cuts, and a mesh known only to `surface_error_mm` cannot certify a
//! contact at the cutting edge more closely than that error. A model that
//! contains the insert's own material in `body` therefore meets the stock at
//! the edge and is refused there; give the insert as the separate
//! `InsertCheck::Model` mesh.
//!
//! Placement is declared, never fitted (`Placement`): the model point that
//! is the nose centre, and the model directions of the lathe's +X (radial,
//! away from the spindle axis on the tool's side) and +Z, as the tool is
//! mounted. Y completes the right-handed frame (Y = Z x X): the cutting
//! face toward +Y.
//!
//! The cross-check (`InsertCheck::Model`). A model placed wrongly is checked
//! in the wrong place, so where the model has the insert, its cutting corner
//! must agree with the declared insert's body, measured the same way on
//! both (support functions h(d) = max d·p over the points within
//! `corner_region_mm` of the nose centre in X and Z, in directions d on the
//! declared insert's cutting face, which its seat tilts, and raised 45
//! degrees from it, where only the face's rim counts: the comparison does
//! not depend on the insert's thickness or clearance below the face):
//! - the nose radius, from three supports across the nose arc: for a circle
//!   of centre c and radius r, h(θ) = c·d(θ) + r, so
//!   r = (h(θ0 - α) + h(θ0 + α) - 2 cos α h(θ0)) / (2 (1 - cos α)),
//!   whatever c is (and whatever the rim's height: raised by φ its support
//!   is cos φ h + sin φ z, and two elevations give h);
//! - the direction of each straight edge at the corner, from the model's
//!   reach across the edge at two stations along it;
//! - the position of the corner (the controlled point, and the centre
//!   height), from the supports in every direction the declared corner is
//!   outermost in, along the face and raised from it.
//!
//! Each has its own declared tolerance; a model outside one is refused
//! before any move, as misplaced or as a different insert. Nothing is
//! aligned or corrected. A model without the insert cannot be compared and
//! says so (`InsertCheck::NotModelled`, with the caller's reason).
//!
//! How the body is checked. The turning checks work on convex pieces
//! (`body.rs`): a piece's XZ projection against material outside the tool's
//! radius, and the radial image (z, √(x² + y²)) of its vertices against
//! material beyond it. A vendor model is not convex, and its hull would
//! fill every relief and neck. It is cut into slices across Z of the
//! declared `axial_resolution_mm`; each slice of the surface is enclosed in
//! one convex prism, and the prisms are the pieces. The argument, for a
//! slice S of the surface between z0 and z1:
//!
//! - Every triangle is convex, so its part inside the slice is a convex
//!   polygon whose vertices are the triangle's corners inside and the
//!   points where its sides cross z0 and z1; S is the union of those
//!   polygons, and lies in the hull of their vertices.
//! - Inside a bore (material beyond the tool: r > bore(z)). With the nose
//!   at radius c, a body point (v, y) of the slice is at radius
//!   √((c + v)² + y²), a convex function of (v, y). Its maximum over S is
//!   its maximum over the hull H of the slice's (v, y) points, attained at
//!   a vertex of H. The prism [z0, z1] x H contains S, and at every z of
//!   the slice it reaches exactly the largest radius S reaches anywhere in
//!   the slice, for every c: nothing is lost across the section (a groove
//!   along the bar, a flat) because only the largest radius can meet the
//!   wall; the body is over-estimated only along Z, by less than one slice.
//! - Outside (material nearer the axis: r < part(z)). A body point's radius
//!   √(x² + y²) is at least its x, so the projection on XZ is never
//!   farther from the axis than the body. Between two vertices the radial
//!   image does dip below their chord (the radius is convex along a
//!   segment: its least value is the segment's distance from the axis), but
//!   never below the projection, which is what is checked: the bound needs
//!   no sagitta term. The prism is the hull of the slice's (z, x) points,
//!   over the slice's extent in y: its lower side is the lower hull of the
//!   projection, exact wherever the body's underside is convex within one
//!   slice.
//! - The surface is enough. From a point inside the solid, the straight line
//!   away from the axis at its height (or along X, either way) leaves the
//!   solid through its surface at the same z: at every z the solid's
//!   largest radius and its least x are those of its surface.
//! - The mesh's error: every point of the real surface is within
//!   `surface_error_mm` of the mesh (its declaration), so within the pieces
//!   grown by that much. Each prism is grown by a cube of that half-size
//!   (which contains the ball) in z, x and y.
//!
//! The pieces are convex and contain the body, so the move checks (swept
//! hulls, projection and radial image) apply to them as to every other
//! piece. Consecutive slices of one section (a straight bar) are one prism.
//! The work per move depends on the number of prism vertices (the facets
//! round each distinct section), not on the triangles along the body; a
//! model is reduced once and its pieces kept (`REDUCED`).
use super::body::Pieces3;
use crate::surface::SurfaceMesh;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

type V3 = [f64; 3];
type P2 = [f64; 2];

/// Where a model sits in the tool's frame, as mounted on the lathe.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Placement {
    /// The model point that is the insert's nose centre on its cutting face
    /// (a drill: its tip on the axis). The theoretical tip a vendor frame
    /// may give is one nose radius from it along X and along Z.
    pub nose_center_mm: V3,
    /// The model direction of the lathe's +X (radial, away from the spindle
    /// axis on the tool's side) and of its +Z: unit and perpendicular.
    pub x_axis: V3,
    pub z_axis: V3,
}

/// How the model's placement is cross-checked against the declared insert.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum InsertCheck {
    /// The model's insert (same placement as the body): its cutting corner
    /// must agree with the declared insert's body within these tolerances.
    Model {
        insert: SurfaceMesh,
        /// The corner is compared within this distance of the nose centre
        /// along X and Z (at least two nose radii).
        corner_region_mm: f64,
        position_tolerance_mm: f64,
        nose_radius_tolerance_mm: f64,
        edge_angle_tolerance_deg: f64,
    },
    /// The model has no insert: its placement cannot be compared with the
    /// declared insert and rests on the caller's `reason` (the source of the
    /// placement, e.g. the maker's cutting-point frame).
    NotModelled { reason: String },
}

/// A tool's body from a supplied model.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ModelBody {
    /// The tool away from the cut (holder, bar, shim, clamp, coolant
    /// block), without the insert: a closed surface, every point of the
    /// real one within its `surface_error_mm`.
    pub body: SurfaceMesh,
    pub placement: Placement,
    /// The width of the slices across Z the body is checked in: the body is
    /// over-estimated along Z by less than this.
    pub axial_resolution_mm: f64,
    pub insert_check: InsertCheck,
}

fn same_mesh(a: &SurfaceMesh, b: &SurfaceMesh) -> bool {
    a.version == b.version
        && a.units == b.units
        && a.surface_error_mm == b.surface_error_mm
        && a.source == b.source
        && (std::sync::Arc::ptr_eq(&a.triangles, &b.triangles) || a.triangles == b.triangles)
}

impl PartialEq for InsertCheck {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (
                Self::Model {
                    insert: a,
                    corner_region_mm: a0,
                    position_tolerance_mm: a1,
                    nose_radius_tolerance_mm: a2,
                    edge_angle_tolerance_deg: a3,
                },
                Self::Model {
                    insert: b,
                    corner_region_mm: b0,
                    position_tolerance_mm: b1,
                    nose_radius_tolerance_mm: b2,
                    edge_angle_tolerance_deg: b3,
                },
            ) => a0 == b0 && a1 == b1 && a2 == b2 && a3 == b3 && same_mesh(a, b),
            (Self::NotModelled { reason: a }, Self::NotModelled { reason: b }) => a == b,
            _ => false,
        }
    }
}

impl PartialEq for ModelBody {
    fn eq(&self, other: &Self) -> bool {
        self.placement == other.placement
            && self.axial_resolution_mm == other.axial_resolution_mm
            && self.insert_check == other.insert_check
            && same_mesh(&self.body, &other.body)
    }
}

fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

impl Placement {
    pub fn validate(&self, what: &str) -> Result<(), String> {
        let finite = |p: V3| p.iter().all(|v| v.is_finite() && v.abs() <= 1e6);
        let unit = |p: V3| (dot(p, p) - 1.).abs() <= 1e-9;
        if !(finite(self.nose_center_mm)
            && finite(self.x_axis)
            && finite(self.z_axis)
            && unit(self.x_axis)
            && unit(self.z_axis)
            && dot(self.x_axis, self.z_axis).abs() <= 1e-9)
        {
            return Err(format!(
                "{what}: its model's placement needs the model point of the nose centre and the model directions of the lathe's +X and +Z as unit, perpendicular vectors (within 1e-9); camlib assumes no placement"
            ));
        }
        Ok(())
    }
    /// A model point in the checks' frame (u, v, y) = (Z, X, Y) from the
    /// nose centre, Y = Z x X.
    fn place(&self, p: V3) -> V3 {
        let d = [
            p[0] - self.nose_center_mm[0],
            p[1] - self.nose_center_mm[1],
            p[2] - self.nose_center_mm[2],
        ];
        [
            dot(d, self.z_axis),
            dot(d, self.x_axis),
            dot(d, cross(self.z_axis, self.x_axis)),
        ]
    }
}

/// The convex hull of 2D points, counterclockwise (Andrew's monotone chain).
fn hull2(mut points: Vec<P2>) -> Vec<P2> {
    points.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
    points.dedup();
    if points.len() < 3 {
        return points;
    }
    let turn = |o: P2, a: P2, b: P2| (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
    let mut lower: Vec<P2> = vec![];
    for &p in &points {
        while lower.len() >= 2 && turn(lower[lower.len() - 2], lower[lower.len() - 1], p) <= 0. {
            lower.pop();
        }
        lower.push(p);
    }
    let mut upper: Vec<P2> = vec![];
    for &p in points.iter().rev() {
        while upper.len() >= 2 && turn(upper[upper.len() - 2], upper[upper.len() - 1], p) <= 0. {
            upper.pop();
        }
        upper.push(p);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

/// The part of a convex polygon where `f` <= 0 (`f` linear).
fn clip(polygon: &[V3], f: impl Fn(V3) -> f64) -> Vec<V3> {
    let n = polygon.len();
    let mut out = Vec::with_capacity(n + 2);
    for i in 0..n {
        let (p, q) = (polygon[i], polygon[(i + 1) % n]);
        let (fp, fq) = (f(p), f(q));
        if fp <= 0. {
            out.push(p);
        }
        if (fp < 0. && fq > 0.) || (fp > 0. && fq < 0.) {
            let t = fp / (fp - fq);
            out.push([
                p[0] + t * (q[0] - p[0]),
                p[1] + t * (q[1] - p[1]),
                p[2] + t * (q[2] - p[2]),
            ]);
        }
    }
    out
}

/// How far `p` is outside a convex counterclockwise polygon (a segment or a
/// point when it has fewer than three vertices): at most its distance.
fn outside(p: P2, polygon: &[P2]) -> f64 {
    let segment = |a: P2, b: P2| {
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let l2 = dx * dx + dy * dy;
        let t = if l2 == 0. {
            0.
        } else {
            (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2).clamp(0., 1.)
        };
        (p[0] - a[0] - t * dx).hypot(p[1] - a[1] - t * dy)
    };
    match polygon.len() {
        0 => f64::INFINITY,
        1 => segment(polygon[0], polygon[0]),
        2 => segment(polygon[0], polygon[1]),
        n => (0..n)
            .map(|i| {
                let (a, b) = (polygon[i], polygon[(i + 1) % n]);
                let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
                // Positive on the right of a counterclockwise side.
                (dy * (p[0] - a[0]) - dx * (p[1] - a[1])) / dx.hypot(dy)
            })
            .fold(f64::NEG_INFINITY, f64::max),
    }
}

/// Whether `p` is certainly within `reach` of a convex counterclockwise
/// polygon (of the lines of the triangle of its fan from vertex 0 that `p`
/// falls in: a sufficient test, in O(log n); a point it does not accept may
/// still be that near).
fn near(p: P2, polygon: &[P2], reach: f64) -> bool {
    let n = polygon.len();
    if n < 3 {
        return outside(p, polygon) <= reach;
    }
    let turn = |a: P2, b: P2| (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]);
    // Positive on the right of the side from a to b.
    let off = |a: P2, b: P2| -turn(a, b) / (b[0] - a[0]).hypot(b[1] - a[1]);
    let (mut lo, mut hi) = (1, n - 1);
    while hi - lo > 1 {
        let mid = (lo + hi) / 2;
        if turn(polygon[0], polygon[mid]) >= 0. {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    off(polygon[lo], polygon[lo + 1]) <= reach
        && off(polygon[0], polygon[1]) <= reach
        && off(polygon[n - 1], polygon[0]) <= reach
}

/// A mesh's triangles placed in the checks' frame (u, v, y), validated.
fn placed(mesh: &SurfaceMesh, placement: &Placement, what: &str) -> Result<Vec<[V3; 3]>, String> {
    mesh.validate_input(0., 1.)
        .map_err(|e| format!("{what}: {e}"))?;
    crate::resources::check_memory(mesh.triangles.len() as u128 * 400, "tool model mesh")?;
    let mut out = Vec::with_capacity(mesh.triangles.len());
    for t in mesh.triangles.iter() {
        if t.iter().flatten().any(|v| !v.is_finite()) {
            return Err(format!("{what}: a vertex is not finite"));
        }
        let t = t.map(|p| placement.place(p));
        if t.iter().flatten().any(|v| v.abs() > 1e4) {
            return Err(format!(
                "{what}: a vertex lies over 10 m from the nose centre; check the model's units (mm) and its placement"
            ));
        }
        out.push(t);
    }
    Ok(out)
}

/// Refuse a surface that is not closed. An exactly shared edge is run once
/// each way by its two faces. A tessellation of separate CAD faces need not
/// share vertices along their common curve: both sides are then within the
/// mesh's error of that curve, so every unmatched edge must have, within
/// twice the error all along it, an unmatched edge of another triangle
/// running the other way. An edge with no face on its other side (a
/// missing face, a shell) has none.
fn check_closed(triangles: &[[V3; 3]], error: f64, what: &str) -> Result<(), String> {
    let key = |p: V3| p.map(|x| (x + 0.).to_bits());
    let mut count: HashMap<([u64; 3], [u64; 3]), i32> = HashMap::new();
    for t in triangles {
        for i in 0..3 {
            let (a, b) = (key(t[i]), key(t[(i + 1) % 3]));
            if a != b {
                *count.entry((a, b)).or_default() += 1;
            }
        }
    }
    let mut open: Vec<(V3, V3, usize)> = vec![];
    for (index, t) in triangles.iter().enumerate() {
        for i in 0..3 {
            let (a, b) = (t[i], t[(i + 1) % 3]);
            let (ka, kb) = (key(a), key(b));
            if ka != kb && count.get(&(ka, kb)) != count.get(&(kb, ka)) {
                open.push((a, b, index));
            }
        }
    }
    if open.is_empty() {
        return Ok(());
    }
    let refuse = |a: V3, b: V3, n: usize| {
        format!(
            "{what} is not a closed surface: {n} edge(s) have no face on their other side within twice its surface_error_mm ({error}), e.g. the edge from (Z {:.4}, X {:.4}, Y {:.4}) to (Z {:.4}, X {:.4}, Y {:.4}) from the nose centre; a tool body is a closed solid: repair or export the model again (camlib does not close it)",
            a[0], a[1], a[2], b[0], b[1], b[2]
        )
    };
    if error == 0. {
        return Err(refuse(open[0].0, open[0].1, open.len()));
    }
    let reach = 2. * error;
    let (lo, hi) = open.iter().fold(
        ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]),
        |(lo, hi), (a, b, _)| {
            (
                std::array::from_fn(|k| lo[k].min(a[k]).min(b[k])),
                std::array::from_fn(|k| hi[k].max(a[k]).max(b[k])),
            )
        },
    );
    let extent = (0..3).map(|k| hi[k] - lo[k]).fold(0., f64::max);
    let cell = (4. * reach).max(extent / 128.);
    let index =
        |p: V3| -> [i64; 3] { std::array::from_fn(|k| ((p[k] - lo[k]) / cell).floor() as i64) };
    let mut grid: HashMap<[i64; 3], Vec<usize>> = HashMap::new();
    let mut long: Vec<usize> = vec![];
    for (i, (a, b, _)) in open.iter().enumerate() {
        let low = index(std::array::from_fn(|k| a[k].min(b[k]) - reach));
        let high = index(std::array::from_fn(|k| a[k].max(b[k]) + reach));
        let cells: i64 = (0..3).map(|k| high[k] - low[k] + 1).product();
        if cells > 512 {
            long.push(i);
            continue;
        }
        for x in low[0]..=high[0] {
            for y in low[1]..=high[1] {
                for z in low[2]..=high[2] {
                    grid.entry([x, y, z]).or_default().push(i);
                }
            }
        }
    }
    let mut unmatched = 0;
    let mut first = None;
    for (a, b, triangle) in &open {
        let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let mated = [0., 0.25, 0.5, 0.75, 1.].iter().all(|t| {
            let p = [a[0] + t * d[0], a[1] + t * d[1], a[2] + t * d[2]];
            grid.get(&index(p))
                .into_iter()
                .flatten()
                .chain(&long)
                .any(|&j| {
                    let (c, e, other) = open[j];
                    let f = [e[0] - c[0], e[1] - c[1], e[2] - c[2]];
                    if other == *triangle || dot(d, f) >= 0. {
                        return false;
                    }
                    let w = [p[0] - c[0], p[1] - c[1], p[2] - c[2]];
                    let s = (dot(w, f) / dot(f, f)).clamp(0., 1.);
                    let g = [w[0] - s * f[0], w[1] - s * f[1], w[2] - s * f[2]];
                    dot(g, g).sqrt() <= reach + 1e-12
                })
        });
        if !mated {
            unmatched += 1;
            first.get_or_insert((*a, *b));
        }
    }
    match first {
        None => Ok(()),
        Some((a, b)) => Err(refuse(a, b, unmatched)),
    }
}

/// The pieces of a model last reduced: what they are a function of (the
/// mesh by its SHA-256, and by identity while this entry keeps it alive).
struct Reduced {
    digest: [u8; 32],
    triangles: std::sync::Arc<Vec<[V3; 3]>>,
    values: (Placement, [f64; 2], bool, u32, String),
    pieces: std::sync::Arc<Pieces3>,
}
static REDUCED: std::sync::Mutex<std::collections::VecDeque<Reduced>> =
    std::sync::Mutex::new(std::collections::VecDeque::new());

impl ModelBody {
    /// The declared values (cheap; the meshes are checked when the pieces
    /// are built).
    pub fn validate(&self, id: &str) -> Result<(), String> {
        let what = format!("turning tool {id:?}");
        self.placement.validate(&what)?;
        if !(self.axial_resolution_mm.is_finite()
            && (1e-3..=1e3).contains(&self.axial_resolution_mm))
        {
            return Err(format!(
                "{what}: its model needs axial_resolution_mm (0.001..1000): the width of the slices across Z its body is checked in (the body is over-estimated along Z by less than one slice)"
            ));
        }
        match &self.insert_check {
            InsertCheck::NotModelled { reason } => {
                if reason.trim().is_empty() {
                    return Err(format!(
                        "{what}: a model without its insert cannot be cross-checked against the declared insert: state the reason its placement is right (the source of the nose centre and axes), or give the model's insert"
                    ));
                }
            }
            InsertCheck::Model {
                corner_region_mm,
                position_tolerance_mm,
                nose_radius_tolerance_mm,
                edge_angle_tolerance_deg,
                ..
            } => {
                let positive = |v: f64, max: f64| v.is_finite() && v > 0. && v <= max;
                if !(positive(*corner_region_mm, 1e3)
                    && positive(*position_tolerance_mm, 1e3)
                    && positive(*nose_radius_tolerance_mm, 1e3)
                    && positive(*edge_angle_tolerance_deg, 45.))
                {
                    return Err(format!(
                        "{what}: its model's insert check needs corner_region_mm, position_tolerance_mm, nose_radius_tolerance_mm (each positive) and edge_angle_tolerance_deg (0..45]: camlib has no default tolerance for a model's agreement with its insert"
                    ));
                }
            }
        }
        Ok(())
    }

    /// The body as convex pieces (u, v, y) = (Z, X, Y) from the nose
    /// centre, containing the real body (see the module's argument):
    /// prisms over each slice's section hull for a tool working in a bore
    /// (`inner`), over each slice's projection hull otherwise.
    pub fn pieces(&self, id: &str, inner: bool) -> Result<Pieces3, String> {
        self.validate(id)?;
        let what = format!("turning tool {id:?}: its model's body mesh");
        // The pieces depend only on the mesh, its error, the placement, the
        // resolution and the side: a run asks for them at every operation
        // and tool change, so the last few are kept.
        let values = (
            self.placement.clone(),
            [self.axial_resolution_mm, self.body.surface_error_mm],
            inner,
            self.body.version,
            self.body.units.clone(),
        );
        let lock = || REDUCED.lock().unwrap_or_else(|e| e.into_inner());
        let same = |r: &Reduced| {
            std::sync::Arc::ptr_eq(&r.triangles, &self.body.triangles) && r.values == values
        };
        if let Some(found) = lock().iter().find(|r| same(r)) {
            return Ok(found.pieces.as_ref().clone());
        }
        let digest = self.digest(inner);
        let known = lock()
            .iter()
            .find(|r| r.digest == digest)
            .map(|r| r.pieces.clone());
        let pieces = match known {
            Some(pieces) => pieces,
            None => {
                let triangles = placed(&self.body, &self.placement, &what)?;
                check_closed(&triangles, self.body.surface_error_mm, &what)?;
                std::sync::Arc::new(
                    slices(
                        &triangles,
                        self.axial_resolution_mm,
                        self.body.surface_error_mm,
                        inner,
                    )
                    .map_err(|e| format!("{what}: {e}"))?,
                )
            }
        };
        let mut cache = lock();
        if !cache.iter().any(same) {
            if cache.len() == 16 {
                cache.pop_front();
            }
            cache.push_back(Reduced {
                digest,
                triangles: self.body.triangles.clone(),
                values,
                pieces: pieces.clone(),
            });
        }
        Ok(pieces.as_ref().clone())
    }

    /// What the pieces are a function of (SHA-256).
    fn digest(&self, inner: bool) -> [u8; 32] {
        use sha2::{Digest, Sha256};
        let mut hash = Sha256::new();
        hash.update(b"camlib turning model pieces v1");
        hash.update([inner as u8]);
        hash.update(self.body.version.to_le_bytes());
        hash.update(self.body.units.as_bytes());
        let numbers = [self.axial_resolution_mm, self.body.surface_error_mm]
            .into_iter()
            .chain(self.placement.nose_center_mm)
            .chain(self.placement.x_axis)
            .chain(self.placement.z_axis);
        for v in numbers {
            hash.update(v.to_bits().to_le_bytes());
        }
        let mut block = Vec::with_capacity(72 * 1024);
        for chunk in self.body.triangles.chunks(1024) {
            block.clear();
            for v in chunk.iter().flatten().flatten() {
                block.extend_from_slice(&v.to_bits().to_le_bytes());
            }
            hash.update(&block);
        }
        hash.finalize().into()
    }

    /// Cross-check the model's insert against the declared insert's body
    /// (`insert_body`: its convex pieces in (u, v, y)); see the module doc.
    pub fn check_insert(
        &self,
        id: &str,
        insert_body: &[Vec<V3>],
        nose_radius_mm: f64,
        drill: bool,
    ) -> Result<(), String> {
        self.validate(id)?;
        let InsertCheck::Model {
            insert,
            corner_region_mm: region,
            position_tolerance_mm,
            nose_radius_tolerance_mm,
            edge_angle_tolerance_deg,
        } = &self.insert_check
        else {
            return Ok(());
        };
        let tool = format!("turning tool {id:?}");
        if drill {
            return Err(format!(
                "{tool}: a drill on the axis has no insert corner to compare a model with; give its model's body with the reason its placement is right"
            ));
        }
        let region = *region;
        if region < 2. * nose_radius_mm {
            return Err(format!(
                "{tool}: corner_region_mm {region} must be at least two nose radii ({}): the corner is compared with the edges beside it",
                2. * nose_radius_mm
            ));
        }
        let what = format!("{tool}: its model's insert mesh");
        let triangles = placed(insert, &self.placement, &what)?;
        check_closed(&triangles, insert.surface_error_mm, &what)?;
        // The model within the corner region (a column along y).
        let mut corner: Vec<Vec<V3>> = vec![];
        for t in &triangles {
            let mut polygon = t.to_vec();
            for (axis, sign) in [(0, 1.), (0, -1.), (1, 1.), (1, -1.)] {
                polygon = clip(&polygon, |p| sign * p[axis] - region);
                if polygon.is_empty() {
                    break;
                }
            }
            if !polygon.is_empty() {
                corner.push(polygon);
            }
        }
        let misplaced = "the model is misplaced (check the placement's nose centre and axes) or is not this insert; camlib does not fit a model to its insert";
        if corner.is_empty() {
            return Err(format!(
                "{what} has nothing within corner_region_mm ({region}) of the declared nose centre: {misplaced}"
            ));
        }
        let declared: Vec<V3> = insert_body.iter().flatten().copied().collect();
        let scale = declared
            .iter()
            .flatten()
            .fold(region, |m, v| m.max(v.abs()));
        let flat = 1e-9 * scale;
        let inside =
            |p: &V3| p[0].abs() < region * (1. - 1e-9) && p[1].abs() < region * (1. - 1e-9);
        // The declared insert's cutting face: the plane through the nose
        // centre and two more of its body's vertices that has the whole
        // body on one side, facing +y (the seat tilts it; of a sharp
        // corner's planes, the one facing +y most).
        let near_corner: Vec<V3> = declared.iter().filter(|p| inside(p)).copied().collect();
        let mut normal: Option<V3> = None;
        for (i, a) in near_corner.iter().enumerate() {
            for b in &near_corner[i + 1..] {
                let n = cross(*a, *b);
                let length = dot(n, n).sqrt();
                if length < 1e-6 * scale * scale {
                    continue;
                }
                let n = n.map(|v| v / length * if n[2] < 0. { -1. } else { 1. });
                if normal.is_some_and(|m| m[2] >= n[2])
                    || declared.iter().any(|p| dot(*p, n) > flat)
                {
                    continue;
                }
                normal = Some(n);
            }
        }
        let Some(normal) = normal.filter(|n| n[2] > 0.5) else {
            return Err(format!(
                "{tool}: the declared insert's cutting face is not found at its nose centre within corner_region_mm ({region}); the model cannot be compared"
            ));
        };
        // Directions on the face (azimuth from the face's line nearest +Z)
        // and raised from it toward its normal.
        let e1 = {
            let along = [
                1. - normal[0] * normal[0],
                -normal[0] * normal[1],
                -normal[0] * normal[2],
            ];
            let length = dot(along, along).sqrt();
            along.map(|v| v / length)
        };
        let e2 = cross(normal, e1);
        let on_face = |a: f64, b: f64| -> V3 { std::array::from_fn(|k| a * e1[k] + b * e2[k]) };
        let direction = |azimuth: usize, elevation: f64| -> V3 {
            let a = (azimuth as f64).to_radians();
            let d = on_face(a.cos() * elevation.cos(), a.sin() * elevation.cos());
            std::array::from_fn(|k| d[k] + elevation.sin() * normal[k])
        };
        // The declared body's support in direction d, where it is attained
        // only inside the corner region (so the region does not cut it),
        // with the greatest distance of its attaining vertices from the
        // nose centre; and the model's.
        let declared_support = |d: V3| -> Option<(f64, f64)> {
            let h = declared
                .iter()
                .map(|p| dot(*p, d))
                .fold(f64::NEG_INFINITY, f64::max);
            let mut far: f64 = 0.;
            for p in declared.iter().filter(|p| dot(**p, d) >= h - flat) {
                if !inside(p) {
                    return None;
                }
                far = far.max(dot(*p, *p).sqrt());
            }
            Some((h, far))
        };
        let model_support = |d: V3| {
            corner
                .iter()
                .flatten()
                .map(|p| dot(*p, d))
                .fold(f64::NEG_INFINITY, f64::max)
        };
        // Every whole degree round the face: along it, and raised 45
        // degrees from it. Raised, the support is the face's rim alone
        // (the body recedes below the face), cos 45 times the outline's
        // support on the face: (declared, its reach from the nose centre,
        // the model's).
        let raise = std::f64::consts::FRAC_PI_4;
        let supports = |elevation: f64| -> Vec<Option<(f64, f64, f64)>> {
            (0..360)
                .map(|k| {
                    let d = direction(k, elevation);
                    declared_support(d).map(|(h, far)| (h, far, model_support(d)))
                })
                .collect()
        };
        let (along, raised) = (supports(0.), supports(raise));
        if raised.iter().flatten().count() < 3 {
            return Err(format!(
                "{tool}: corner_region_mm ({region}) holds no cutting corner of the declared insert to compare the model with; give a region that holds the nose and the start of its edges"
            ));
        }
        // 1. The nose radius: the longest run of directions whose declared
        // support lies on the nose (one nose radius from its centre).
        let on_nose = |k: usize| {
            raised[k % 360].is_some_and(|(_, far, _)| {
                (far - nose_radius_mm).abs() <= 1e-3 * nose_radius_mm + flat
            })
        };
        let mut best = (0, 0);
        for start in 0..360 {
            if on_nose(start) && !on_nose(start + 359) {
                let len = (0..360).take_while(|k| on_nose(start + k)).count();
                if len > best.1 {
                    best = (start, len);
                }
            }
        }
        if best.1 == 0 && (0..360).all(on_nose) {
            best = (0, 359);
        }
        // Under 20 degrees of nose the three supports are too close to
        // measure a radius (no insert's corner is that flat).
        if best.1 < 21 {
            return Err(format!(
                "{tool}: the declared insert's nose (radius {nose_radius_mm}) is not found within corner_region_mm ({region}) of its centre over at least 20 degrees; the model cannot be compared"
            ));
        }
        // The three supports: the middle of the nose and 0.7 of the way to
        // each of its ends (clear of the edges, whose supports a model
        // turned a little would take over).
        let middle = best.0 + (best.1 - 1) / 2;
        let half = (best.1 - 1) * 7 / 20;
        // Raised by φ, a rim's support is cos φ (c·d + r) + sin φ z for a
        // rim at height z above the face: two elevations give c·d + r
        // whatever z is, so a model off in height is not read as a model
        // of another radius.
        let lower = raise / 2.;
        let at = [middle - half + 360, middle, middle + half];
        let mut outline_support = [[0.; 3]; 2];
        for (i, k) in at.into_iter().enumerate() {
            let d = direction(k % 360, lower);
            let (Some((high, _, model_high)), Some((low, _))) =
                (raised[k % 360], declared_support(d))
            else {
                return Err(format!(
                    "{tool}: the declared insert's nose is cut by corner_region_mm ({region}); give a larger region"
                ));
            };
            let solve = |high: f64, low: f64| {
                (high * lower.sin() - low * raise.sin())
                    / (raise.cos() * lower.sin() - lower.cos() * raise.sin())
            };
            outline_support[0][i] = solve(high, low);
            outline_support[1][i] = solve(model_high, model_support(d));
        }
        let radius = |h: [f64; 3]| {
            let alpha = (half as f64).to_radians();
            (h[0] + h[2] - 2. * alpha.cos() * h[1]) / (2. * (1. - alpha.cos()))
        };
        let (declared_radius, model_radius) =
            (radius(outline_support[0]), radius(outline_support[1]));
        if (model_radius - declared_radius).abs() > *nose_radius_tolerance_mm {
            return Err(format!(
                "{tool}: its model's nose radius measures {model_radius:.4} mm, the declared insert's {declared_radius:.4} mm (nose_radius_tolerance_mm {nose_radius_tolerance_mm}): the model is a different insert, or misplaced; camlib does not fit a model to its insert"
            ));
        }
        // 2. The straight edges at the corner: the sides of the declared
        // cutting face's outline that are long (not a chord of the nose)
        // and start in the region. In the face's own coordinates.
        let face = |p: V3| -> P2 { [dot(p, e1), dot(p, e2)] };
        let outline = hull2(
            declared
                .iter()
                .filter(|p| dot(**p, normal) >= -flat)
                .map(|p| face(*p))
                .collect(),
        );
        let n = outline.len();
        let mut missing: Option<String> = None;
        for i in 0..n {
            let (a, b) = (outline[i], outline[(i + 1) % n]);
            let length = (b[0] - a[0]).hypot(b[1] - a[1]);
            let within = |p: P2| inside(&on_face(p[0], p[1]));
            if n < 3 || length < region / 2. || !(within(a) || within(b)) {
                continue;
            }
            // The side's start, direction and outward normal (of a
            // counterclockwise outline), in the checks' frame.
            let t = [(b[0] - a[0]) / length, (b[1] - a[1]) / length];
            let (start, t, out) = (
                on_face(a[0], a[1]),
                on_face(t[0], t[1]),
                on_face(t[1], -t[0]),
            );
            // The part of the side inside the region (Liang-Barsky).
            let (mut s0, mut s1) = (0., length);
            for k in 0..2 {
                if t[k] != 0. {
                    let (x, y) = ((-region - start[k]) / t[k], (region - start[k]) / t[k]);
                    s0 = f64::max(s0, x.min(y));
                    s1 = f64::min(s1, x.max(y));
                }
            }
            let span = s1 - s0;
            if span < region / 4. {
                continue;
            }
            let from = |p: V3| -> V3 { std::array::from_fn(|k| p[k] - start[k]) };
            let mut reach = [0.; 2];
            for (station, fraction) in [0.25, 0.75].into_iter().enumerate() {
                let (at, width) = (s0 + fraction * span, 0.05 * span);
                let along = |p: V3| dot(from(p), t);
                let mut h = f64::NEG_INFINITY;
                for polygon in &corner {
                    let strip = clip(&clip(polygon, |p| along(p) - at - width), |p| {
                        at - width - along(p)
                    });
                    for p in strip {
                        h = h.max(dot(from(p), out));
                    }
                }
                reach[station] = h;
            }
            if reach.contains(&f64::NEG_INFINITY) {
                // Nothing to measure a direction from: the position check
                // says how far off the model is; if it does not, this does.
                missing.get_or_insert(format!(
                    "{what} has no material along the declared insert's edge toward (X {:.3}, Z {:.3}) from the nose: {misplaced}",
                    t[1], t[0]
                ));
                continue;
            }
            let angle = (reach[1] - reach[0]).atan2(0.5 * span).to_degrees();
            if angle.abs() > *edge_angle_tolerance_deg {
                return Err(format!(
                    "{tool}: its model's edge runs {angle:.3} degrees off the declared insert's edge along (X {:.3}, Z {:.3}) (edge_angle_tolerance_deg {edge_angle_tolerance_deg}): the model is rotated in its placement, or is a different insert; camlib does not fit a model to its insert",
                    t[1], t[0]
                ));
            }
        }
        // 3. The corner's position: along the face and raised from it (the
        // centre height).
        let mut worst: Option<(f64, V3)> = None;
        for (elevation, entries) in [(0., &along), (raise, &raised)] {
            for (k, entry) in entries.iter().enumerate() {
                if let Some((h, _, model)) = entry {
                    let off = model - h;
                    if worst.is_none_or(|(w, _)| off.abs() > w.abs()) {
                        worst = Some((off, direction(k, elevation)));
                    }
                }
            }
        }
        if let Some((off, d)) = worst
            && off.abs() > *position_tolerance_mm
        {
            return Err(format!(
                "{tool}: its model's cutting corner is {:.4} mm {} the declared insert's toward (X {:.3}, Y {:.3}, Z {:.3}) (position_tolerance_mm {position_tolerance_mm}): {misplaced}",
                off.abs(),
                if off > 0. { "beyond" } else { "short of" },
                d[1],
                d[2],
                d[0]
            ));
        }
        missing.map_or(Ok(()), Err)
    }
}

/// The slices of a surface across u, each as the points of its triangles'
/// parts inside it (see the module's argument): `visit(k, [lo, hi], points)`.
fn for_slices(
    triangles: &[[V3; 3]],
    width: f64,
    mut visit: impl FnMut(usize, &[V3]),
) -> Result<(f64, f64, usize), String> {
    let (lo, hi) = triangles
        .iter()
        .flatten()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), p| {
            (l.min(p[0]), h.max(p[0]))
        });
    let count = ((hi - lo) / width).ceil().max(1.);
    if count > 1e5 {
        return Err(format!(
            "{count} slices of axial_resolution_mm {width} along its {:.1} mm: give a coarser resolution",
            hi - lo
        ));
    }
    let count = count as usize;
    crate::resources::check_memory(count as u128 * 4096 * 16, "tool model slices")?;
    let slot = |u: f64| (((u - lo) / width).floor().max(0.) as usize).min(count - 1);
    for t in triangles {
        let (a, b) = t
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), p| {
                (l.min(p[0]), h.max(p[0]))
            });
        let (first, last) = (slot(a), slot(b));
        if first == last {
            visit(first, &t[..]);
            continue;
        }
        for k in first..=last {
            let [low, high] = slice_range(lo, hi, width, count, k);
            let part = clip(&clip(t, |p| low - p[0]), |p| p[0] - high);
            if !part.is_empty() {
                visit(k, &part);
            }
        }
    }
    Ok((lo, hi, count))
}

fn slice_range(lo: f64, hi: f64, width: f64, count: usize, k: usize) -> [f64; 2] {
    [
        lo + k as f64 * width,
        if k + 1 == count {
            hi
        } else {
            lo + (k + 1) as f64 * width
        },
    ]
}

/// The convex pieces of a placed surface (see `ModelBody::pieces`).
fn slices(triangles: &[[V3; 3]], width: f64, error: f64, inner: bool) -> Result<Pieces3, String> {
    // Per slice: its points in the plane that matters ((v, y) inside a
    // bore, (u, v) outside), kept as their hull once they are many, and its
    // extent in y.
    let mut points: Vec<Vec<P2>> = vec![];
    let mut extent: Vec<[f64; 2]> = vec![];
    let (lo, hi, count) = for_slices(triangles, width, |k, part| {
        if points.len() <= k {
            points.resize(k + 1, vec![]);
            extent.resize(k + 1, [f64::INFINITY, f64::NEG_INFINITY]);
        }
        for p in part {
            points[k].push(if inner { [p[1], p[2]] } else { [p[0], p[1]] });
            extent[k] = [extent[k][0].min(p[2]), extent[k][1].max(p[2])];
        }
        if points[k].len() > 4096 {
            points[k] = hull2(std::mem::take(&mut points[k]));
        }
    })?;
    points.resize(count, vec![]);
    extent.resize(count, [f64::INFINITY, f64::NEG_INFINITY]);
    let hulls: Vec<Vec<P2>> = points.into_iter().map(hull2).collect();
    // Grown by the mesh's error: a square of that half-size at each vertex.
    let grown = |hull: &[P2]| {
        if error == 0. {
            return hull.to_vec();
        }
        hull2(
            hull.iter()
                .flat_map(|p| {
                    [
                        [p[0] - error, p[1] - error],
                        [p[0] + error, p[1] - error],
                        [p[0] + error, p[1] + error],
                        [p[0] - error, p[1] + error],
                    ]
                })
                .collect(),
        )
    };
    let mut pieces: Pieces3 = vec![];
    if !inner {
        for (k, hull) in hulls.iter().enumerate() {
            if hull.is_empty() {
                continue;
            }
            let [y0, y1] = [extent[k][0] - error, extent[k][1] + error];
            pieces.push(
                grown(hull)
                    .into_iter()
                    .flat_map(|p| [[p[0], p[1], y0], [p[0], p[1], y1]])
                    .collect(),
            );
        }
        return Ok(pieces);
    }
    // Inside a bore: consecutive slices of one section are one prism. Each
    // joins the run while its section is the run's first within `same`
    // (rounding only: the run's hull is then every member's, to `same`).
    let scale = triangles
        .iter()
        .flatten()
        .flatten()
        .fold(1., |m: f64, v| m.max(v.abs()));
    let same = 1e-9 * scale;
    let alike = |a: &[P2], b: &[P2]| {
        a.iter().all(|p| near(*p, b, same)) && b.iter().all(|p| near(*p, a, same))
    };
    let mut k = 0;
    while k < count {
        if hulls[k].is_empty() {
            k += 1;
            continue;
        }
        let mut end = k + 1;
        let mut all = hulls[k].clone();
        while end < count && !hulls[end].is_empty() && alike(&hulls[k], &hulls[end]) {
            all.extend(&hulls[end]);
            end += 1;
        }
        let section = grown(&hull2(all));
        let (u0, u1) = (
            slice_range(lo, hi, width, count, k)[0] - error,
            slice_range(lo, hi, width, count, end - 1)[1] + error,
        );
        pieces.push(
            section
                .into_iter()
                .flat_map(|p| [[u0, p[0], p[1]], [u1, p[0], p[1]]])
                .collect(),
        );
        k = end;
    }
    Ok(pieces)
}

/// The envelope of a body turning about an axis (a milling holder in its
/// spindle), from its mesh: for each slice of `axial_resolution_mm` along
/// the axis between `z_range_mm` (distances from `origin_mm` along `axis`),
/// the largest radius the surface reaches in it, grown by the mesh's error
/// (the slice also takes the surface within that error of it along the
/// axis). Returned as (z0, z1, radius) cylinders, consecutive equal radii
/// joined.
///
/// The solid a body sweeps when it turns about the axis is the union of the
/// circles through its points. Each triangle's part in a slice is a convex
/// polygon and the radius √(x² + y²) is convex, so the largest radius in
/// the slice is at a vertex of those polygons: the cylinder of that radius
/// over the slice contains everything the slice's surface sweeps. The
/// surface is enough: a point inside the body lies, at its height, nearer
/// the axis than the surface point the line outward from it leaves by.
/// The model above the range's end (a taper inside the spindle) is not part
/// of the envelope. Refused: a slice with no
/// surface (the model does not span the range), a model reaching below the
/// range's start (it would be cut off), an open mesh.
pub fn revolved_envelope(
    mesh: &SurfaceMesh,
    origin_mm: V3,
    axis: V3,
    z_range_mm: [f64; 2],
    axial_resolution_mm: f64,
) -> Result<Vec<[f64; 3]>, String> {
    let what = "holder model mesh";
    let finite = |p: V3| p.iter().all(|v| v.is_finite() && v.abs() <= 1e6);
    if !(finite(origin_mm) && finite(axis) && (dot(axis, axis) - 1.).abs() <= 1e-9) {
        return Err(format!(
            "{what}: its placement needs the model point of the tool tip on the axis and the model direction of the axis (from the tip toward the spindle) as a unit vector; camlib assumes no placement"
        ));
    }
    let [z0, z1] = z_range_mm;
    if !(z0.is_finite() && z1.is_finite() && z0 < z1 && z0.abs() <= 1e4 && z1.abs() <= 1e4) {
        return Err(format!(
            "{what}: z_range_mm must run from the holder's start (the stickout) up to the gauge plane"
        ));
    }
    let width = axial_resolution_mm;
    if !(width.is_finite() && (1e-3..=1e3).contains(&width)) {
        return Err(format!(
            "{what}: axial_resolution_mm (0.001..1000) is the height of the cylinders the holder is checked as"
        ));
    }
    // Any frame about the axis: radii do not depend on it.
    let seed = if axis[0].abs() < 0.9 {
        [1., 0., 0.]
    } else {
        [0., 1., 0.]
    };
    let x = cross(cross(axis, seed), axis);
    let norm = dot(x, x).sqrt();
    let placement = Placement {
        nose_center_mm: origin_mm,
        x_axis: x.map(|v| v / norm),
        z_axis: axis,
    };
    let triangles = placed(mesh, &placement, what)?;
    let error = mesh.surface_error_mm;
    check_closed(&triangles, error, what)?;
    let low = triangles
        .iter()
        .flatten()
        .fold(f64::INFINITY, |m, p| m.min(p[0]));
    if low - error < z0 - 1e-9 {
        return Err(format!(
            "{what}: the model reaches {:.4} mm along the axis from the tip, below the range's start {z0} (its error {error} included): the holder starts lower than declared; start the range (the tool's stickout) at or below that",
            low - error
        ));
    }
    let count = ((z1 - z0) / width).ceil().max(1.);
    if count > 1e5 {
        return Err(format!(
            "{what}: {count} slices of axial_resolution_mm {width}: give a coarser resolution"
        ));
    }
    let count = count as usize;
    let range = |k: usize| slice_range(z0, z1, width, count, k);
    let mut radius = vec![f64::NEG_INFINITY; count];
    for t in &triangles {
        let (a, b) = t
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), p| {
                (l.min(p[0]), h.max(p[0]))
            });
        // The slices within the mesh's error of the triangle along the axis.
        let slot = |u: f64| ((u - z0) / width).floor();
        let (first, last) = (
            slot(a - error).max(0.),
            slot(b + error).min(count as f64 - 1.),
        );
        if first > last {
            continue;
        }
        for (k, r) in radius
            .iter_mut()
            .enumerate()
            .take(last as usize + 1)
            .skip(first as usize)
        {
            let [low, high] = range(k);
            let part = clip(&clip(t, |p| low - error - p[0]), |p| p[0] - high - error);
            for p in part {
                *r = r.max(p[1].hypot(p[2]));
            }
        }
    }
    let mut out: Vec<[f64; 3]> = vec![];
    for (k, r) in radius.into_iter().enumerate() {
        let [low, high] = range(k);
        if r == f64::NEG_INFINITY {
            return Err(format!(
                "{what}: the model has no surface between {low:.4} and {high:.4} mm along the axis from the tip: it does not span z_range_mm (a holder's envelope must reach from the stickout to the gauge plane); check the placement and the range"
            ));
        }
        let r = r + error;
        match out.last_mut() {
            Some(last) if last[2] == r => last[1] = high,
            _ => out.push([low, high, r]),
        }
    }
    Ok(out)
}

/// Synthetic closed meshes for the tests, in the checks' frame (u, v, y);
/// `to_model` gives the model coordinates of `identity()`'s placement.
#[cfg(test)]
pub(crate) mod testing {
    use super::*;

    pub fn identity() -> Placement {
        Placement {
            nose_center_mm: [0., 0., 0.],
            x_axis: [1., 0., 0.],
            z_axis: [0., 0., 1.],
        }
    }
    /// (u, v, y) = (Z, X, Y) as model (x, y, z) under `identity()`.
    pub fn to_model(p: V3) -> V3 {
        [p[1], p[2], p[0]]
    }
    pub fn surface(triangles: Vec<[V3; 3]>, error: f64) -> SurfaceMesh {
        SurfaceMesh {
            version: 1,
            units: "mm".into(),
            triangles: std::sync::Arc::new(
                triangles.into_iter().map(|t| t.map(to_model)).collect(),
            ),
            surface_error_mm: error,
            source: serde_json::Value::Null,
        }
    }
    /// A closed hexahedron: corners 0..4 one end, 4..8 the other (each end
    /// in order round it).
    pub fn hexa(c: [V3; 8]) -> Vec<[V3; 3]> {
        [
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [1, 2, 6],
            [1, 6, 5],
            [2, 3, 7],
            [2, 7, 6],
            [3, 0, 4],
            [3, 4, 7],
        ]
        .iter()
        .map(|t| t.map(|i| c[i]))
        .collect()
    }
    /// A hexahedron from the rectangle (v, y) `a` at u0 to `b` at u1 (each
    /// [v_lo, v_hi, y_lo, y_hi]).
    pub fn loft(u0: f64, a: [f64; 4], u1: f64, b: [f64; 4]) -> Vec<[V3; 3]> {
        let end = |u: f64, r: [f64; 4]| {
            [
                [u, r[0], r[2]],
                [u, r[1], r[2]],
                [u, r[1], r[3]],
                [u, r[0], r[3]],
            ]
        };
        let (p, q) = (end(u0, a), end(u1, b));
        hexa([p[0], p[1], p[2], p[3], q[0], q[1], q[2], q[3]])
    }
    pub fn box_mesh(lo: V3, hi: V3) -> Vec<[V3; 3]> {
        let r = [lo[1], hi[1], lo[2], hi[2]];
        loft(lo[0], r, hi[0], r)
    }
    /// A prism over a polygon in (u, v) from y0 to y1.
    pub fn prism(polygon: &[P2], y0: f64, y1: f64) -> Vec<[V3; 3]> {
        let n = polygon.len();
        let at = |i: usize, y: f64| [polygon[i % n][0], polygon[i % n][1], y];
        let mut out = vec![];
        for i in 1..n - 1 {
            out.push([at(0, y1), at(i, y1), at(i + 1, y1)]);
            out.push([at(0, y0), at(i + 1, y0), at(i, y0)]);
        }
        for i in 0..n {
            out.push([at(i, y0), at(i + 1, y0), at(i + 1, y1)]);
            out.push([at(i, y0), at(i + 1, y1), at(i, y1)]);
        }
        out
    }
    /// A body of revolution about the line (v, y) = `axis` parallel to u,
    /// through the rings (u, radius), `around` facets round; its ends
    /// closed with `caps` facets (`around` for a mesh with shared vertices).
    pub fn revolved(axis: P2, rings: &[P2], around: usize, caps: usize) -> Vec<[V3; 3]> {
        let at = |ring: P2, k: usize, n: usize| {
            let a = std::f64::consts::TAU * (k % n) as f64 / n as f64;
            [
                ring[0],
                axis[0] + ring[1] * a.cos(),
                axis[1] + ring[1] * a.sin(),
            ]
        };
        let mut out = vec![];
        for w in rings.windows(2) {
            for k in 0..around {
                let (a, b) = (at(w[0], k, around), at(w[0], k + 1, around));
                let (c, d) = (at(w[1], k, around), at(w[1], k + 1, around));
                out.push([a, b, d]);
                out.push([a, d, c]);
            }
        }
        let (first, last) = (rings[0], rings[rings.len() - 1]);
        for k in 0..caps {
            out.push([
                [first[0], axis[0], axis[1]],
                at(first, k + 1, caps),
                at(first, k, caps),
            ]);
            out.push([
                [last[0], axis[0], axis[1]],
                at(last, k, caps),
                at(last, k + 1, caps),
            ]);
        }
        out
    }
    pub fn model(triangles: Vec<[V3; 3]>, error: f64, resolution: f64) -> ModelBody {
        ModelBody {
            body: surface(triangles, error),
            placement: identity(),
            axial_resolution_mm: resolution,
            insert_check: InsertCheck::NotModelled {
                reason: "synthetic test body".into(),
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::testing::*;
    use super::*;

    /// Points of every triangle (a barycentric grid), in (u, v, y).
    fn samples(triangles: &[[V3; 3]], n: usize) -> Vec<V3> {
        let mut out = vec![];
        for t in triangles {
            for i in 0..=n {
                for j in 0..=n - i {
                    let (a, b) = (i as f64 / n as f64, j as f64 / n as f64);
                    let c = 1. - a - b;
                    out.push(std::array::from_fn(|k| {
                        a * t[0][k] + b * t[1][k] + c * t[2][k]
                    }));
                }
            }
        }
        out
    }

    /// A body no hull describes: a bar off the centre plane, a block far
    /// along it, a cone between, a wedge under the head.
    fn awkward() -> Vec<[V3; 3]> {
        let mut t = revolved([-6., 0.7], &[[0.5, 5.], [80., 5.]], 24, 24);
        t.extend(box_mesh([45., -20., -9.], [60., 9., 6.]));
        t.extend(revolved(
            [-6., 0.7],
            &[[20., 5.], [27.3, 8.5], [31., 6.]],
            24,
            24,
        ));
        t.extend(loft(-1.5, [-4., -1., -3., 0.], 3.25, [-9., 2.5, -6., 1.]));
        t
    }

    #[test]
    fn the_pieces_contain_the_sampled_body_in_both_images() {
        let triangles = awkward();
        for (error, width) in [(0., 0.5), (0.02, 0.37), (0., 7.)] {
            let model = model(triangles.clone(), error, width);
            // Points of the surface, pushed out by the mesh's error in a
            // direction that varies from point to point.
            let points: Vec<V3> = samples(&triangles, 6)
                .into_iter()
                .enumerate()
                .map(|(i, p)| {
                    let (a, b) = (i as f64 * 0.7, i as f64 * 1.3);
                    [
                        p[0] + error * a.cos() * b.cos(),
                        p[1] + error * a.sin() * b.cos(),
                        p[2] + error * b.sin(),
                    ]
                })
                .collect();
            // Inside a bore: at the point's z, some piece reaches at least
            // its radius, wherever the nose is.
            let inner = model.pieces("t", true).unwrap();
            let ranges: Vec<(f64, f64)> = inner
                .iter()
                .map(|piece| {
                    piece
                        .iter()
                        .fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), p| {
                            (l.min(p[0]), h.max(p[0]))
                        })
                })
                .collect();
            for c in [0., 3., 8.6, 25., -4.] {
                for p in &points {
                    let radius = (c + p[1]).hypot(p[2]);
                    let reach = inner
                        .iter()
                        .zip(&ranges)
                        // (A sample's z is rounded: 1e-9 either way.)
                        .filter(|(_, (lo, hi))| *lo - 1e-9 <= p[0] && p[0] <= *hi + 1e-9)
                        .flat_map(|(piece, _)| piece.iter().map(|q| (c + q[1]).hypot(q[2])))
                        .fold(f64::NEG_INFINITY, f64::max);
                    assert!(reach >= radius - 1e-9, "{error} {width} {c} {p:?}: {reach}");
                }
            }
            // Outside: the point's projection lies in some piece's.
            let outer = model.pieces("t", false).unwrap();
            let images: Vec<Vec<P2>> = outer
                .iter()
                .map(|piece| hull2(piece.iter().map(|p| [p[0], p[1]]).collect()))
                .collect();
            for p in &points {
                let out = images
                    .iter()
                    .map(|image| outside([p[0], p[1]], image))
                    .fold(f64::INFINITY, f64::min);
                assert!(out <= 1e-9, "{error} {width} {p:?}: {out}");
            }
        }
    }

    /// Where a dense model's reduction spends its time (run with
    /// --ignored --nocapture).
    #[test]
    #[ignore = "measurement"]
    fn dense_reduction_phases() {
        let mut rings: Vec<P2> = (0..=40)
            .map(|k| [0.5 + 0.25 * k as f64, 4. + 0.025 * k as f64])
            .collect();
        rings.extend((1..=100).map(|k| [10.5 + 0.69 * k as f64, 5.]));
        let triangles = revolved([-6., 0.], &rings, 512, 512);
        let mesh = surface(triangles, 1e-4);
        let clock = std::time::Instant::now();
        let placed = placed(&mesh, &identity(), "t").unwrap();
        let a = clock.elapsed();
        check_closed(&placed, 1e-4, "t").unwrap();
        let b = clock.elapsed();
        let inner = slices(&placed, 0.25, 1e-4, true).unwrap();
        let c = clock.elapsed();
        let outer = slices(&placed, 0.25, 1e-4, false).unwrap();
        let d = clock.elapsed();
        let count = |p: &Pieces3| (p.len(), p.iter().map(Vec::len).sum::<usize>());
        println!(
            "{} triangles: placed {a:?}, closed {:?}, inner slices {:?} {:?}, outer slices {:?} {:?}",
            placed.len(),
            b - a,
            c - b,
            count(&inner),
            d - c,
            count(&outer)
        );
    }

    #[test]
    fn a_neck_and_a_relief_stay_open() {
        // The bar between its tip and the block: the pieces there reach the
        // bar's radius, not the block's (a hull of the whole body would).
        let model = model(awkward(), 0., 0.5);
        let reach = |pieces: &Pieces3, u: f64, c: f64| {
            pieces
                .iter()
                .filter(|piece| piece.iter().any(|p| p[0] <= u) && piece.iter().any(|p| p[0] >= u))
                .flat_map(|piece| piece.iter().map(|q| (c + q[1]).hypot(q[2])))
                .fold(f64::NEG_INFINITY, f64::max)
        };
        let inner = model.pieces("t", true).unwrap();
        // The bar alone (axis at v -6, y 0.7, radius 5; nose at 8.6).
        let bar = (8.6f64 - 6.).hypot(0.7) + 5.;
        for u in [5.2, 12.7, 40.1, 70.3] {
            // (A 24-sided bar: its corner nearest that direction.)
            let found = reach(&inner, u, 8.6);
            assert!(found <= bar + 1e-9 && found >= bar - 1e-3, "{u} {found}");
        }
        // The block: its far corner.
        let block = (8.6f64 + 9.).hypot(9.);
        assert!((reach(&inner, 50.2, 8.6) - block).abs() < 1e-9);
        // The straight bar is one prism, not a slice each.
        assert!(inner.len() < 60, "{}", inner.len());
        // Outside: the underside follows the body (the bar's lower side at
        // v -11, the block's at -20), exact where it is flat.
        let outer = model.pieces("t", false).unwrap();
        let lowest = |u: f64| {
            outer
                .iter()
                .filter(|piece| piece.iter().any(|p| p[0] <= u) && piece.iter().any(|p| p[0] >= u))
                .flat_map(|piece| piece.iter().map(|q| q[1]))
                .fold(f64::INFINITY, f64::min)
        };
        assert!((lowest(40.2) + 11.).abs() < 1e-9);
        assert!((lowest(50.2) + 20.).abs() < 1e-9);
        assert!((lowest(70.2) + 11.).abs() < 1e-9);
    }

    #[test]
    fn the_mesh_error_is_grown_onto_the_body() {
        let exact = model(box_mesh([0., 1., -2.], [10., 4., 3.]), 0., 2.5);
        let grown = model(box_mesh([0., 1., -2.], [10., 4., 3.]), 0.05, 2.5);
        for inner in [true, false] {
            let bounds = |m: &ModelBody| {
                m.pieces("t", inner).unwrap().iter().flatten().fold(
                    ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]),
                    |(lo, hi), p| {
                        (
                            std::array::from_fn(|k| lo[k].min(p[k])),
                            std::array::from_fn(|k| hi[k].max(p[k])),
                        )
                    },
                )
            };
            let ((a, b), (c, d)) = (bounds(&exact), bounds(&grown));
            assert_eq!((a, b), ([0., 1., -2.], [10., 4., 3.]));
            for k in 0..3 {
                assert!((a[k] - c[k] - 0.05).abs() < 1e-12 && (d[k] - b[k] - 0.05).abs() < 1e-12);
            }
        }
    }

    #[test]
    fn an_open_or_unplaced_model_is_refused() {
        // A box without one face.
        let mut open = box_mesh([0., 1., -2.], [10., 4., 3.]);
        open.truncate(10);
        for error in [0., 0.01] {
            let err = model(open.clone(), error, 1.)
                .pieces("t", true)
                .unwrap_err();
            assert!(err.contains("not a closed surface"), "{err}");
        }
        // Faces tessellated separately (caps of 17 facets on a side of 24):
        // closed within the mesh's error, refused as exact.
        let cracked = revolved([0., 0.], &[[0., 5.], [20., 5.]], 24, 17);
        let sag = 5. * (1. - (std::f64::consts::PI / 17.).cos());
        model(cracked.clone(), sag, 1.).pieces("t", true).unwrap();
        let err = model(cracked, 0., 1.).pieces("t", true).unwrap_err();
        assert!(err.contains("not a closed surface"), "{err}");
        // No placement, resolution or reason is assumed.
        let good = || model(box_mesh([0., 1., -2.], [10., 4., 3.]), 0., 1.);
        let mut m = good();
        m.placement.x_axis = [1., 0.1, 0.];
        assert!(m.validate("t").unwrap_err().contains("placement"));
        let mut m = good();
        m.placement.z_axis = [1., 0., 0.];
        assert!(m.validate("t").unwrap_err().contains("placement"));
        let mut m = good();
        m.axial_resolution_mm = 0.;
        assert!(m.validate("t").unwrap_err().contains("axial_resolution_mm"));
        let mut m = good();
        m.insert_check = InsertCheck::NotModelled { reason: " ".into() };
        assert!(m.validate("t").unwrap_err().contains("reason"));
        let mut m = good();
        m.body.surface_error_mm = 0.5;
        assert!(
            m.pieces("t", true)
                .unwrap_err()
                .contains("surface_error_mm")
        );
        let mut m = good();
        m.body.units = "inch".into();
        assert!(m.pieces("t", true).unwrap_err().contains("units"));
    }

    #[test]
    fn a_placement_turns_and_moves_the_model() {
        // The model drawn with its bar along its own +x, its nose at
        // (100, 50, 20), radial toward its -y.
        let placement = Placement {
            nose_center_mm: [100., 50., 20.],
            x_axis: [0., -1., 0.],
            z_axis: [1., 0., 0.],
        };
        // Y = Z x X = (1,0,0) x (0,-1,0) = (0,0,-1).
        assert_eq!(placement.place([103., 48., 19.]), [3., 2., 1.]);
        let lo = [2., 1., -1.];
        let hi = [9., 4., 2.];
        let triangles = box_mesh(lo, hi)
            .into_iter()
            .map(|t| t.map(|p| [100. + p[0], 50. - p[1], 20. - p[2]]))
            .collect();
        let mut m = model(vec![], 0., 1.);
        m.body.triangles = std::sync::Arc::new(triangles);
        m.placement = placement;
        let pieces = m.pieces("t", true).unwrap();
        let (a, b) = pieces.iter().flatten().fold(
            ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]),
            |(a, b), p| {
                (
                    std::array::from_fn(|k| a[k].min(p[k])),
                    std::array::from_fn(|k| b[k].max(p[k])),
                )
            },
        );
        for k in 0..3 {
            assert!((a[k] - lo[k]).abs() < 1e-12 && (b[k] - hi[k]).abs() < 1e-12);
        }
    }

    #[test]
    fn a_turning_holder_envelope_bounds_every_sampled_point() {
        // An off-axis block on a shank, about the model's z axis from the
        // tip at z = 0 (model coordinates: x, y across, z along).
        let mut triangles = revolved([0., 0.], &[[30., 16.], [80., 16.]], 32, 32);
        triangles.extend(box_mesh([40., -5., -30.], [55., 12., 10.]));
        let mesh = surface(triangles.clone(), 0.01);
        let sections = revolved_envelope(&mesh, [0.; 3], [0., 0., 1.], [29.5, 75.], 2.).unwrap();
        assert!(sections.first().unwrap()[0] == 29.5 && sections.last().unwrap()[1] == 75.);
        assert!(sections.windows(2).all(|w| w[0][1] == w[1][0]));
        for p in samples(&triangles, 5) {
            if p[0] > 75. {
                continue;
            }
            let r = p[1].hypot(p[2]);
            let bound = sections
                .iter()
                .filter(|s| s[0] <= p[0] && p[0] <= s[1])
                .map(|s| s[2])
                .fold(f64::NEG_INFINITY, f64::max);
            assert!(bound >= r + 0.01 - 1e-9, "{p:?} {r} {bound}");
        }
        // Away from the block the envelope is the shank's (and the error).
        assert!((sections[0][2] - 16.01).abs() < 1e-9);
        // Over the block: its far corner.
        let over = sections
            .iter()
            .find(|s| s[0] <= 47. && 47. <= s[1])
            .unwrap();
        assert!(
            (over[2] - (30f64.hypot(12.) + 0.01)).abs() < 1e-9,
            "{over:?}"
        );
        // The model must span the range and start where declared.
        let err = revolved_envelope(&mesh, [0.; 3], [0., 0., 1.], [29.5, 90.], 2.).unwrap_err();
        assert!(err.contains("no surface"), "{err}");
        // The real surface may lie the mesh's error below the mesh: a
        // range starting at the mesh's own start does not hold it.
        for start in [35., 30.] {
            let err =
                revolved_envelope(&mesh, [0.; 3], [0., 0., 1.], [start, 75.], 2.).unwrap_err();
            assert!(err.contains("below the range's start"), "{err}");
        }
        let err = revolved_envelope(&mesh, [0.; 3], [0., 0., 2.], [30., 75.], 2.).unwrap_err();
        assert!(err.contains("unit vector"), "{err}");
    }
}
