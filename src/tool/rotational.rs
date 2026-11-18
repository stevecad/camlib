//! Assembled milling profiles of revolution, including annular cutting regions.
//!
//! Coordinates are radius from the spindle axis and height above the tool tip,
//! in mm. These are **mounted** meridian profiles, not unmounted insert outlines.
//! A tilted insert's nominal corner circle need not generate a circular meridian.
//! See `docs/indexable-geometry.md` for the manufacturer references, numerical
//! contract and the boundary between this geometry and motion verification.
//!
//! Each section returns both an inner (certain) and outer (possible) set. Basic
//! binary64 operations are rounded outward individually; circles are intersected
//! analytically, never tessellated. The bounds concern the declared geometry,
//! not manufacturing tolerances, tooth marks, deflection or a swept motion.
use crate::interval::Bounds as B;
use serde::{Deserialize, Serialize};

/// A closed interval in mm. Section results are sorted and disjoint.
pub type Interval = [f64; 2];

/// Inner and outer sets enclosing the exact section of the declared profile.
/// An empty `certain` set does not imply clearance: inspect `possible`.
#[derive(Clone, Debug, Default, PartialEq, Serialize)]
pub struct SectionBounds {
    pub certain: Vec<Interval>,
    pub possible: Vec<Interval>,
}

/// A solid meridian primitive, revolved about the spindle axis. Unions of
/// primitives retain their holes; taking a convex hull is not equivalent.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum ProfileRegion {
    /// Inner and outer radii interpolate linearly between the two Z planes.
    /// Equal inner/outer radii at one end describe a triangular section.
    AnnularFrustum {
        z_mm: Interval,
        inner_radius_mm: Interval,
        outer_radius_mm: Interval,
    },
    /// The disk (r-cr)^2 + (z-cz)^2 <= radius^2, intersected with `z_mm`.
    /// Its revolution is a toroidal region, optionally clipped axially.
    /// `cr >= radius` keeps the meridian on the nonnegative-radius side.
    /// The radius is a real mounted-profile radius, never a programming radius.
    CircularRing {
        center_rz_mm: [f64; 2],
        radius_mm: f64,
        z_mm: Interval,
    },
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RotationalProfile {
    pub regions: Vec<ProfileRegion>,
}

/// Inserts sharing one mounted rotational cutting profile. Angular tooth
/// positions do not affect this envelope; count is retained as assembly data.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MountedInsertGroup {
    pub id: String,
    pub count: u32,
    pub cutting: RotationalProfile,
}

/// Geometry only. The noncutting envelope includes the cutter body and any
/// insert backs, fasteners or other noncutting regions relevant to clearance.
/// Cutting and noncutting sets are queried separately even where they overlap.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct IndexableGeometry {
    pub inserts: Vec<MountedInsertGroup>,
    pub noncutting: RotationalProfile,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MillingFamily {
    Face,
    HighFeed,
}

/// Operating limits of the assembled body/insert combination. Zero ramp angle
/// forbids cutting descent. None for hole diameters forbids helical entry;
/// Linear cutting ramps require an exit/reversal model and are refused.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct IndexableMilling {
    pub family: MillingFamily,
    pub geometry: IndexableGeometry,
    pub max_depth_of_cut_mm: f64,
    pub max_ramp_angle_deg: f64,
    pub helical_hole_diameter_mm: Option<Interval>,
}

impl IndexableMilling {
    pub(crate) fn validate(&self) -> Result<PreparedIndexableGeometry, String> {
        if !super::positive(self.max_depth_of_cut_mm)
            || !self.max_ramp_angle_deg.is_finite()
            || !(0. ..90.).contains(&self.max_ramp_angle_deg)
            || self
                .helical_hole_diameter_mm
                .is_some_and(|d| !d.iter().all(|&v| super::positive(v)) || d[0] > d[1])
        {
            return Err("indexable milling needs a positive maximum depth of cut, a ramp angle in [0,90), ordered positive helical hole diameters when declared".into());
        }
        let prepared = self.geometry.prepare()?;
        for region in &self.geometry.noncutting.regions {
            if !matches!(
                region,
                ProfileRegion::AnnularFrustum {
                    inner_radius_mm: [0., 0.],
                    ..
                }
            ) {
                return Err("indexable noncutting body must be declared as filled cylindrical/conical envelope sections (annular_frustum with zero inner radii)".into());
            }
        }
        Ok(prepared)
    }
}

pub(crate) fn check_cutting_descent(
    tool: &super::PreparedAssembly,
    motion: &crate::motion::Motion,
) -> Result<(), String> {
    use crate::motion::{Curve, Plane};
    let limits = tool
        .cutter()
        .definition()
        .shape
        .indexable()
        .ok_or("missing indexable limits")?;
    let (a, b) = (motion.curve.start(), motion.curve.end());
    if !matches!(
        motion.curve,
        Curve::Line { .. }
            | Curve::Arc {
                plane: Plane::Xy,
                ..
            }
            | Curve::Helix {
                plane: Plane::Xy,
                ..
            }
    ) {
        return Err("indexable cutting motion supports lines and XY arcs/helices only".into());
    }
    if b[2] >= a[2] {
        return Ok(());
    }
    let horizontal = match motion.curve {
        Curve::Line { .. } => (b[0] - a[0]).hypot(b[1] - a[1]),
        Curve::Helix {
            center,
            plane: Plane::Xy,
            sweep_rad,
            ..
        } => {
            let radius = (a[0] - center[0]).hypot(a[1] - center[1]);
            // A shaped cutter does not bore one cylindrical diameter: its
            // tip may cut a narrower hole than its largest section. Check
            // the enclosing diameter RANGE, so the maximum tool diameter
            // cannot falsely satisfy the manufacturer's minimum hole size.
            let minimum_cutting_radius = tool
                .cutter()
                .indexable()
                .unwrap()
                .cutting()
                .minimum_outer_radius_bound();
            let hole = [
                2. * (radius + minimum_cutting_radius),
                2. * radius + tool.cutter().definition().diameter_mm,
            ];
            if !limits
                .helical_hole_diameter_mm
                .is_some_and(|d| d[0] <= hole[0] && hole[1] <= d[1])
            {
                return Err(format!(
                    "indexable helical entry has hole diameter bounds {hole:?} mm outside the declared helical_hole_diameter_mm; change the entry diameter or use an exterior entry"
                ));
            }
            radius * sweep_rad.abs()
        }
        _ => {
            return Err(
                "indexable cutting descent requires a declared XY helix or an exterior entry"
                    .into(),
            );
        }
    };
    if horizontal == 0. {
        return Err("indexable cutter cannot plunge into stock at its noncutting center; use an exterior entry or a declared helical entry".into());
    }
    let angle = (a[2] - b[2]).atan2(horizontal).to_degrees();
    if angle > limits.max_ramp_angle_deg {
        return Err(format!(
            "indexable entry angle {angle} degrees exceeds max_ramp_angle_deg {}; reduce entry pitch",
            limits.max_ramp_angle_deg
        ));
    }
    if matches!(motion.curve, Curve::Line { .. }) {
        return Err("indexable linear cutting ramps require a verified reversal/exit transition; use a declared helical entry or exterior entry".into());
    }
    Ok(())
}

#[derive(Clone, Debug)]
pub struct PreparedProfile {
    definition: RotationalProfile,
}

#[derive(Clone, Debug)]
pub struct PreparedIndexableGeometry {
    definition: IndexableGeometry,
    cutting: PreparedProfile,
    noncutting: PreparedProfile,
}

fn dimension(x: f64) -> bool {
    x.is_finite() && (0.0..=1e6).contains(&x)
}

fn height_range(z: Interval) -> bool {
    z.iter().all(|&x| dimension(x)) && z[0] < z[1]
}

impl ProfileRegion {
    /// Enclosing radius and height ranges, rounded outward.
    pub(crate) fn bounds(&self) -> [Interval; 2] {
        match *self {
            Self::AnnularFrustum {
                z_mm,
                inner_radius_mm: a,
                outer_radius_mm: b,
            } => [[a[0].min(a[1]), b[0].max(b[1])], z_mm],
            Self::CircularRing {
                center_rz_mm: [r, z],
                radius_mm: q,
                z_mm,
            } => {
                let bottom = B::point(z).sub(B::point(q));
                let top = B::point(z).add(B::point(q));
                [
                    [
                        B::point(r).sub(B::point(q)).lo.max(0.),
                        B::point(r).add(B::point(q)).hi,
                    ],
                    [z_mm[0].max(bottom.lo), z_mm[1].min(top.hi)],
                ]
            }
        }
    }

    /// Radial occupancy for a height window. Certain intervals are present at
    /// EVERY height; possible intervals cover occupancy at ANY height.
    pub(crate) fn radial_band(&self, heights: Interval) -> SectionBounds {
        let bounds = self.bounds()[1];
        let clip = [heights[0].max(bounds[0]), heights[1].min(bounds[1])];
        if clip[0] > clip[1] {
            return SectionBounds::default();
        }
        match *self {
            Self::AnnularFrustum {
                z_mm,
                inner_radius_mm: inner,
                outer_radius_mm: outer,
            } => {
                let a = fraction(clip[0], z_mm);
                let b = fraction(clip[1], z_mm);
                let (ia, ib, oa, ob) = (
                    lerp(inner, a),
                    lerp(inner, b),
                    lerp(outer, a),
                    lerp(outer, b),
                );
                let mut result = SectionBounds::default();
                push_interval(
                    &mut result.possible,
                    [ia.lo.min(ib.lo), oa.hi.max(ob.hi)],
                    [0., 1e6],
                );
                if heights[0] >= z_mm[0] && heights[1] <= z_mm[1] {
                    push_interval(
                        &mut result.certain,
                        [ia.hi.max(ib.hi), oa.lo.min(ob.lo)],
                        [0., 1e6],
                    );
                }
                result
            }
            Self::CircularRing {
                center_rz_mm: [r, z],
                radius_mm: q,
                z_mm,
            } => {
                let near = z.clamp(clip[0], clip[1]);
                let mut result = circle_section(r, B::point(near).sub(B::point(z)), q, [0., 1e6]);
                result.certain.clear();
                if heights[0] >= z_mm[0] && heights[1] <= z_mm[1] {
                    let d2 = far_squared(heights, z);
                    result.certain = circle_squared_section(r, d2, q, [0., 1e6]).certain;
                }
                result
            }
        }
    }

    /// Axial occupancy that reaches ANY radius in a band (`all=false`) or
    /// contains EVERY radius in it (`all=true`). Both numerical sets bound
    /// that specified quantifier; unlike radial_band this is not a mixed pair.
    pub(crate) fn axial_band(&self, radii: Interval, all: bool) -> SectionBounds {
        match *self {
            Self::AnnularFrustum {
                z_mm,
                inner_radius_mm,
                outer_radius_mm,
            } => frustum_axial(z_mm, inner_radius_mm, outer_radius_mm, radii, all),
            Self::CircularRing {
                center_rz_mm: [r, z],
                radius_mm: q,
                z_mm,
            } => {
                let squared = if all {
                    far_squared(radii, r)
                } else {
                    B::point(r.clamp(radii[0], radii[1]))
                        .sub(B::point(r))
                        .square()
                };
                circle_squared_section(z, squared, q, z_mm)
            }
        }
    }
    fn validate(&self) -> Result<(), String> {
        match *self {
            Self::AnnularFrustum {
                z_mm: z,
                inner_radius_mm: inner,
                outer_radius_mm: outer,
            } => {
                if !height_range(z)
                    || !inner.iter().chain(&outer).all(|&r| dimension(r))
                    || inner[0] > outer[0]
                    || inner[1] > outer[1]
                    || (inner[0] == outer[0] && inner[1] == outer[1])
                {
                    return Err("annular frustum needs increasing nonnegative Z, nonnegative inner <= outer radii at each end, and positive meridian area; dimensions must be finite and <= 1e6 mm".into());
                }
            }
            Self::CircularRing {
                center_rz_mm: [r, z],
                radius_mm: radius,
                z_mm: clip,
            } => {
                if !height_range(clip)
                    || !dimension(r)
                    || !z.is_finite()
                    || z.abs() > 1e6
                    || !super::positive(radius)
                    || r < radius
                    || B::point(r).add(B::point(radius)).hi > 1e6
                {
                    return Err("circular ring needs a positive finite radius, center radius >= ring radius, increasing nonnegative Z clip, and finite dimensions within 1e6 mm".into());
                }
                let bottom = B::point(z).sub(B::point(radius));
                let top = B::point(z).add(B::point(radius));
                if clip[0] >= top.lo || clip[1] <= bottom.hi {
                    return Err("circular ring Z clip must intersect the disk with positive area; move the clip away from a tangent if the intersection is unresolved at binary64 precision".into());
                }
            }
        }
        Ok(())
    }

    fn radial(&self, z: f64) -> SectionBounds {
        match *self {
            Self::AnnularFrustum {
                z_mm: height,
                inner_radius_mm: inner,
                outer_radius_mm: outer,
            } => {
                if z < height[0] || z > height[1] {
                    return SectionBounds::default();
                }
                let t = fraction(z, height);
                let inner = lerp(inner, t);
                let outer = lerp(outer, t);
                section(inner, outer, [0., 1e6], true)
            }
            Self::CircularRing {
                center_rz_mm: [r, cz],
                radius_mm: radius,
                z_mm: clip,
            } => {
                if z < clip[0] || z > clip[1] {
                    return SectionBounds::default();
                }
                circle_section(r, B::point(z).sub(B::point(cz)), radius, [0., 1e6])
            }
        }
    }

    fn axial(&self, r: f64) -> SectionBounds {
        match *self {
            Self::AnnularFrustum {
                z_mm: z,
                inner_radius_mm: inner,
                outer_radius_mm: outer,
            } => frustum_axial(z, inner, outer, [r, r], true),
            Self::CircularRing {
                center_rz_mm: [cr, z],
                radius_mm: radius,
                z_mm: clip,
            } => circle_section(z, B::point(r).sub(B::point(cr)), radius, clip),
        }
    }
}

impl RotationalProfile {
    pub fn prepare(&self) -> Result<PreparedProfile, String> {
        if self.regions.is_empty() {
            return Err(
                "rotational profile needs at least one region; absent geometry is not clearance"
                    .into(),
            );
        }
        for (i, region) in self.regions.iter().enumerate() {
            region
                .validate()
                .map_err(|e| format!("profile region {i}: {e}"))?;
        }
        Ok(PreparedProfile {
            definition: self.clone(),
        })
    }
}

impl PreparedProfile {
    /// Lower bound on the outer radius at every occupied profile height.
    /// A union's outer radius is at least that of any present primitive.
    /// A clipped circular meridian's outer branch is always >= its center R.
    pub(crate) fn minimum_outer_radius_bound(&self) -> f64 {
        self.definition
            .regions
            .iter()
            .map(|region| match *region {
                ProfileRegion::AnnularFrustum {
                    outer_radius_mm, ..
                } => outer_radius_mm[0].min(outer_radius_mm[1]),
                ProfileRegion::CircularRing { center_rz_mm, .. } => center_rz_mm[0],
            })
            .fold(f64::INFINITY, f64::min)
    }
    pub(crate) fn floor_radius(&self) -> Option<f64> {
        let mut radius: f64 = 0.;
        for region in &self.definition.regions {
            if let ProfileRegion::CircularRing {
                center_rz_mm: [r, z],
                radius_mm,
                z_mm,
            } = *region
                && z == radius_mm
                && z_mm[0] == 0.
            {
                // Exact equality of authored binary64 values establishes the
                // bottom tangent; interval subtraction otherwise straddles it.
                radius = radius.max(r);
            }
            for band in region.radial(0.).certain {
                radius = radius.max(band[1]);
            }
        }
        (radius > 0.).then_some(radius)
    }
    pub fn definition(&self) -> &RotationalProfile {
        &self.definition
    }

    pub(crate) fn bounds(&self) -> [Interval; 2] {
        self.definition
            .regions
            .iter()
            .map(ProfileRegion::bounds)
            .fold([[f64::INFINITY, f64::NEG_INFINITY]; 2], |a, b| {
                std::array::from_fn(|k| [a[k][0].min(b[k][0]), a[k][1].max(b[k][1])])
            })
    }

    pub(crate) fn axial_band(&self, radii: Interval, all: bool) -> SectionBounds {
        union(
            self.definition
                .regions
                .iter()
                .map(|r| r.axial_band(radii, all)),
        )
    }

    /// Per-region bands: keep the correlation between the inner and outer
    /// radius when sweeping a ring. Unioning their projections first is unsafe.
    pub(crate) fn radial_bands(
        &self,
        heights: Interval,
    ) -> impl Iterator<Item = SectionBounds> + '_ {
        self.definition
            .regions
            .iter()
            .map(move |r| r.radial_band(heights))
    }

    /// Closed radial intervals occupied at this tip-relative height.
    pub fn radial_intervals_at(&self, z_mm: f64) -> Result<SectionBounds, String> {
        if !z_mm.is_finite() {
            return Err("profile height query must be finite".into());
        }
        Ok(union(
            self.definition.regions.iter().map(|p| p.radial(z_mm)),
        ))
    }

    /// Closed axial intervals occupied at this nonnegative radius.
    pub fn axial_intervals_at(&self, radius_mm: f64) -> Result<SectionBounds, String> {
        if !radius_mm.is_finite() || radius_mm < 0. {
            return Err("profile radius query must be finite and nonnegative".into());
        }
        // All authored radii are <= 1e6; avoid overflowing distant queries.
        if radius_mm > 1e6 {
            return Ok(SectionBounds::default());
        }
        Ok(union(
            self.definition.regions.iter().map(|p| p.axial(radius_mm)),
        ))
    }

    /// Bounds on the axial intervals cut in an entire XY cell by one chord.
    /// `error` encloses path and coordinate error, `half` the cell half-widths.
    ///
    /// For every cell point, distance to a continuous path takes every value
    /// between its minimum and maximum. The common distance interval below is
    /// therefore reached by EVERY point (possibly at different times). At a
    /// climbing chord we intersect the axial occupancy over all tip heights.
    /// Fixed-pose certificates additionally handle a stationary or short chord.
    pub(crate) fn sweep_column(
        &self,
        a: [f64; 3],
        b: [f64; 3],
        center: [f64; 2],
        half: [f64; 2],
        error: f64,
    ) -> SectionBounds {
        let dx = b[0] - a[0];
        let dy = b[1] - a[1];
        let length2 = dx * dx + dy * dy;
        let t = if length2 == 0. {
            0.
        } else {
            (((center[0] - a[0]) * dx + (center[1] - a[1]) * dy) / length2).clamp(0., 1.)
        };
        // Motion and stock coordinates are bounded by 1e6 mm. Use explicit
        // squares and sqrt here: a platform hypot differing by one ulp can
        // turn a repeat of a cut into a new height change in a boundary cell.
        let norm = |x: f64, y: f64| (x * x + y * y).sqrt();
        let distance = |t: f64| norm(center[0] - a[0] - t * dx, center[1] - a[1] - t * dy);
        let near = distance(t);
        let far = distance(0.).max(distance(1.));
        let pad = norm(half[0], half[1]) + error;
        let zlo = a[2].min(b[2]);
        let zhi = a[2].max(b[2]);
        let outer = self.axial_band([(near - pad).max(0.), far + pad], false);
        let mut result = SectionBounds {
            certain: vec![],
            possible: outer
                .possible
                .iter()
                .map(|h| [zlo + h[0] - error, zhi + h[1] + error])
                .collect(),
        };
        let common = [near + pad, far - pad];
        if common[0] <= common[1] {
            // Each primitive is convex in (radius,height): within the
            // common radial strip, a connected top-to-bottom profile meets
            // the path's left-to-right crossing. Contract BEFORE unioning
            // primitives. Merging axial projections first would lose radial
            // correlation and could bridge two separate insert groups while
            // a climbing path passes above one and below the other.
            for region in &self.definition.regions {
                for h in region.axial_band(common, false).certain {
                    let span = [zhi + h[0] + error, zlo + h[1] - error];
                    if span[0] <= span[1] {
                        result.certain.push(span);
                    }
                }
            }
        }
        for s in [0., t, 1.] {
            let d = distance(s);
            let z = a[2] + s * (b[2] - a[2]);
            for h in self.axial_band([(d - pad).max(0.), d + pad], true).certain {
                let span = if length2 == 0. {
                    [zlo + h[0] + error, zhi + h[1] - error]
                } else {
                    [z + h[0] + error, z + h[1] - error]
                };
                if span[0] <= span[1] {
                    result.certain.push(span);
                }
            }
        }
        union(std::iter::once(result))
    }
}

impl IndexableGeometry {
    pub fn prepare(&self) -> Result<PreparedIndexableGeometry, String> {
        if self.inserts.is_empty() {
            return Err("indexable geometry needs at least one mounted insert group".into());
        }
        let mut ids = std::collections::HashSet::new();
        let mut regions = Vec::new();
        for insert in &self.inserts {
            if !super::valid_id(&insert.id) || !ids.insert(&insert.id) || insert.count == 0 {
                return Err(
                    "mounted insert groups need unique stable ids and positive counts".into(),
                );
            }
            insert
                .cutting
                .prepare()
                .map_err(|e| format!("insert {}: {e}", insert.id))?;
            regions.extend(insert.cutting.regions.iter().cloned());
        }
        let noncutting = self
            .noncutting
            .prepare()
            .map_err(|e| format!("noncutting body: {e}"))?;
        Ok(PreparedIndexableGeometry {
            definition: self.clone(),
            cutting: PreparedProfile {
                definition: RotationalProfile { regions },
            },
            noncutting,
        })
    }
}

impl PreparedIndexableGeometry {
    pub fn definition(&self) -> &IndexableGeometry {
        &self.definition
    }
    pub fn cutting(&self) -> &PreparedProfile {
        &self.cutting
    }
    pub fn noncutting(&self) -> &PreparedProfile {
        &self.noncutting
    }
}

fn fraction(x: f64, range: Interval) -> B {
    if x == range[0] {
        return B::point(0.);
    }
    if x == range[1] {
        return B::point(1.);
    }
    let f = B::point(x)
        .sub(B::point(range[0]))
        .div(B::point(range[1]).sub(B::point(range[0])));
    B {
        lo: f.lo.max(0.),
        hi: f.hi.min(1.),
    }
}

fn lerp(ends: Interval, t: B) -> B {
    if ends[0] == ends[1] {
        return B::point(ends[0]);
    }
    if t.lo == 0. && t.hi == 0. {
        return B::point(ends[0]);
    }
    if t.lo == 1. && t.hi == 1. {
        return B::point(ends[1]);
    }
    let value = B::point(ends[0]).add(B::point(ends[1]).sub(B::point(ends[0])).mul(t));
    // Called only on t in [0,1]; the exact interpolation lies between its ends.
    B {
        lo: value.lo.max(ends[0].min(ends[1])),
        hi: value.hi.min(ends[0].max(ends[1])),
    }
}

fn circle_section(center: f64, delta: B, radius: f64, clip: Interval) -> SectionBounds {
    circle_squared_section(center, delta.square(), radius, clip)
}

fn far_squared(range: Interval, center: f64) -> B {
    let a = B::point(range[0]).sub(B::point(center)).square();
    let b = B::point(range[1]).sub(B::point(center)).square();
    B {
        lo: a.lo.max(b.lo),
        hi: a.hi.max(b.hi),
    }
}

fn circle_squared_section(
    center: f64,
    delta_squared: B,
    radius: f64,
    clip: Interval,
) -> SectionBounds {
    let squared = B::point(radius).square().sub(delta_squared);
    if squared.hi < 0. {
        return SectionBounds::default();
    }
    let half = B {
        lo: squared.lo.max(0.).sqrt().next_down().max(0.),
        hi: squared.hi.sqrt().next_up(),
    };
    section(
        B::point(center).sub(half),
        B::point(center).add(half),
        clip,
        squared.lo >= 0.,
    )
}

fn section(low: B, high: B, clip: Interval, certain_exists: bool) -> SectionBounds {
    let mut out = SectionBounds::default();
    if certain_exists {
        push_interval(&mut out.certain, [low.hi, high.lo], clip);
    }
    push_interval(&mut out.possible, [low.lo, high.hi], clip);
    out
}

fn push_interval(out: &mut Vec<Interval>, span: Interval, clip: Interval) {
    let span = [span[0].max(clip[0]), span[1].min(clip[1])];
    if span[0] <= span[1] {
        out.push(span);
    }
}

fn union(sections: impl Iterator<Item = SectionBounds>) -> SectionBounds {
    let mut result = SectionBounds::default();
    for section in sections {
        result.certain.extend(section.certain);
        result.possible.extend(section.possible);
    }
    for spans in [&mut result.certain, &mut result.possible] {
        spans.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
        let mut kept = 0;
        for i in 0..spans.len() {
            let span = spans[i];
            if kept > 0 && span[0] <= spans[kept - 1][1] {
                spans[kept - 1][1] = spans[kept - 1][1].max(span[1]);
            } else {
                spans[kept] = span;
                kept += 1;
            }
        }
        spans.truncate(kept);
    }
    result
}

#[cfg(test)]
mod tests;

fn frustum_axial(
    z: Interval,
    inner: Interval,
    outer: Interval,
    radii: Interval,
    all: bool,
) -> SectionBounds {
    if radii[1] < inner[0].min(inner[1]) || radii[0] > outer[0].max(outer[1]) {
        return SectionBounds::default();
    }
    // In t=(z-z0)/(z1-z0), solve inner(t) <= r <= outer(t).
    // Each linear inequality clips one end of the unit interval.
    let mut certain: Interval = [0., 1.];
    let mut possible: Interval = [0., 1.];
    for (ends, is_inner) in [(inner, true), (outer, false)] {
        if ends[0] == ends[1] {
            let limit = if is_inner == all { radii[0] } else { radii[1] };
            if (is_inner && limit < ends[0]) || (!is_inner && limit > ends[0]) {
                return SectionBounds::default();
            }
            continue;
        }
        let limit = if is_inner == all { radii[0] } else { radii[1] };
        let root = B::point(limit)
            .sub(B::point(ends[0]))
            .div(B::point(ends[1]).sub(B::point(ends[0])));
        let upper = (ends[1] > ends[0]) == is_inner;
        if upper {
            certain[1] = certain[1].min(root.lo);
            possible[1] = possible[1].min(root.hi);
        } else {
            certain[0] = certain[0].max(root.hi);
            possible[0] = possible[0].max(root.lo);
        }
    }
    let mut out = SectionBounds::default();
    if certain[0] <= certain[1] {
        push_interval(
            &mut out.certain,
            [
                lerp(z, B::point(certain[0])).hi,
                lerp(z, B::point(certain[1])).lo,
            ],
            z,
        );
    }
    if possible[0] <= possible[1] {
        push_interval(
            &mut out.possible,
            [
                lerp(z, B::point(possible[0])).lo,
                lerp(z, B::point(possible[1])).hi,
            ],
            z,
        );
    }
    out
}
