//! Flat-cutter column protection and stock-minus-retained-body layer geometry.
//! See docs/retained-body-roughing.md; shadows are not tip-plane cross-sections.
use crate::{
    Point,
    adaptive::{AdaptivePathStrategy, AdaptivePlan, AdaptivePocketOperation, CuttingSense},
    adaptive_front::FrontReport,
    motion::{Coolant, Event, Motion, Vec3},
    planar::{
        Boolean, Boundary, Component, ConstructionOptions, FilledHoles, PlanarRegion,
        PreparedRegion,
    },
    pocket::{
        HelicalEntry, PocketEntry, PocketOperation, PocketPlan, RasterRouting, ResidualStatus,
    },
    stock::{CancelSignal, Stock, StockSpec},
    surface::SurfaceMesh,
    tool::ToolAssembly,
};
use robust::{Coord, orient2d};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{collections::VecDeque, sync::OnceLock};

mod engagement_front;
pub(crate) mod parallel;
use engagement_front::EngagementFrontPlan;
pub use engagement_front::EngagementFrontReport;

#[derive(Clone, Debug, Serialize)]
pub struct ShadowReport {
    pub tip_z_mm: f64,
    pub protected_from_z_mm: f64,
    pub axial_allowance_mm: f64,
    pub source_error_mm: f64,
    pub numerical_z_reserve_mm: f64,
    pub projected_triangles: usize,
    pub enclosed_degenerate_projections: usize,
    pub projection_enclosure_mm: f64,
    /// Holes of the projected union too small to hold the cutter, filled
    /// (protected): `PreparedRoughingTarget::shadow_for_cutter`.
    pub filled_holes: usize,
    pub filled_hole_area_mm2: f64,
    /// The shadow simplified within a tolerance and grown by it (None: the
    /// exact shadow).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub simplified: Option<SimplifiedShadow>,
    pub input_vertices: usize,
    pub output_vertices: usize,
    pub boolean_operations: usize,
    pub boundary_error_mm: f64,
}
pub struct RoughingShadow {
    pub region: PlanarRegion,
    pub report: ShadowReport,
}
pub struct PreparedRoughingTarget {
    mesh: SurfaceMesh,
    tolerance: f64,
    numerical: f64,
    source_sha256: String,
}
fn check(cancel: &impl CancelSignal) -> Result<(), String> {
    if cancel.is_cancelled() {
        Err("retained-body roughing cancelled".into())
    } else {
        Ok(())
    }
}
fn committed_top(stock: &Stock, cancel: &impl CancelSignal) -> Result<f64, String> {
    let mut top = f64::NEG_INFINITY;
    for (i, cell) in stock.cells().iter().enumerate() {
        if i.is_multiple_of(1024) {
            check(cancel)?;
        }
        top = top.max(cell.upper_mm);
    }
    check(cancel)?;
    Ok(top)
}
fn no_part_depth_check(value: &bool) -> bool {
    !value
}
fn orientation(a: Point, b: Point, c: Point) -> f64 {
    orient2d(
        Coord { x: a[0], y: a[1] },
        Coord { x: b[0], y: b[1] },
        Coord { x: c[0], y: c[1] },
    )
}
fn hull(mut points: Vec<Point>) -> Vec<Point> {
    points.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
    points.dedup();
    if points.len() < 3 {
        return points;
    }
    fn half(points: impl Iterator<Item = Point>) -> Vec<Point> {
        let mut result: Vec<Point> = vec![];
        for p in points {
            while result.len() > 1
                && orientation(result[result.len() - 2], result[result.len() - 1], p) <= 0.
            {
                result.pop();
            }
            result.push(p);
        }
        result
    }
    let mut a = half(points.iter().copied());
    let mut b = half(points.iter().rev().copied());
    a.pop();
    b.pop();
    a.extend(b);
    a
}
fn clipped(triangle: [Vec3; 3], z: f64) -> Vec<Point> {
    let mut points = Vec::with_capacity(4);
    for i in 0..3 {
        let a = triangle[i];
        let b = triangle[(i + 1) % 3];
        if a[2] >= z {
            points.push([a[0], a[1]]);
        }
        if (a[2] >= z) != (b[2] >= z) {
            let t = ((z - a[2]) / (b[2] - a[2])).clamp(0., 1.);
            points.push([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]);
        }
    }
    hull(points)
}
/// Step-up levels between a full stepdown layer at `z` and the layer (or
/// material top) `above` it: evenly spaced, at most `stepup` apart, ascending,
/// so each climbs back up through the stair the full stepdown left on sloped
/// and stepped walls. Established practice: the "step up" of rest-aware
/// adaptive roughing (for example Mastercam Dynamic OptiRough, Cut
/// Parameters, "Step up"); ported from CAMBuddy's MIT `plan_stepup_levels`.
pub(crate) fn stepup_layers(z: f64, above: f64, stepup: f64, guard: f64) -> Vec<f64> {
    let depth = above - z;
    // Also refuses NaN: callers validate finite inputs first.
    if depth.is_nan() || depth <= stepup.max(guard) {
        return vec![];
    }
    let count = (depth / stepup).ceil() as usize;
    let pitch = depth / count as f64;
    (1..count).map(|j| z + j as f64 * pitch).collect()
}

/// The region a step-up layer at `region`'s depth may still find material in:
/// next to the part, within the layer `below`'s shadow grown by three times
/// `reach` (cutter radius plus radial allowance).
///
/// After the full layer `below`, every point farther than `reach = r + a`
/// from its shadow is a valid cutter centre there and so was cut down to
/// `below`. Material left at this height is therefore within `reach` of that
/// shadow, and a cutter centre that reaches it lies within `2r + a`; keeping
/// that centre `reach` inside the band needs the band to extend `3r + 2a`
/// (plus the shadow's own construction error) beyond the shadow. The band is
/// intersected with this layer's own region, so it never adds area the full
/// layer would not cut.
fn stepup_band(
    shadow: &RoughingShadow,
    reach: f64,
    options: ConstructionOptions,
) -> Result<PlanarRegion, String> {
    // `shadow` is the parent layer's shadow, built the same (simplified) way
    // as every layer's. It contains the exact shadow, so the band only grows,
    // and its boundary is no finer than the layer region it is intersected
    // with (an exact fine-mesh shadow left sliver components that failed
    // topology validation).
    // 3 * reach = 3r + 3a covers the 3r + 2a needed, with the allowance spare.
    let grow = 3. * reach + shadow.report.boundary_error_mm + options.tolerance_mm;
    // Square-join growth contains the round growth and keeps straight edges:
    // a round offset's arcs came back from the intersection as thousands of
    // tiny segments, whose erosion envelope then let the cutter-centre
    // region creep inside the clearance the pocket proves.
    Ok(shadow
        .region
        .prepare(options)?
        .inflate_square(grow, 0.)?
        .0
        .region)
}

fn count_vertices(region: &PlanarRegion) -> usize {
    region
        .components
        .iter()
        .flat_map(|c| std::iter::once(&c.outer).chain(&c.holes))
        .map(|b| b.vertices.len())
        .sum()
}
#[derive(Clone, Debug, Serialize)]
pub struct SimplifiedShadow {
    pub tolerance_mm: f64,
    /// Boundary vertices of the exact shadow (the report's output_vertices
    /// are the simplified region's).
    pub exact_vertices: usize,
    /// Why the exact shadow was kept: not at least twice as simple (a coarse
    /// or planar shadow), or a simplified ring would cross itself.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
}

/// Douglas-Peucker on a closed ring: every removed vertex lies within
/// `tolerance` of the chord that replaces it (the chord's segment, not its
/// line), so the whole removed chain lies in the segment's tolerance band.
fn simplify_ring(ring: &[Point], tolerance: f64) -> Vec<Point> {
    let n = ring.len();
    if n <= 3 {
        return ring.to_vec();
    }
    let distance = |p: Point, a: Point, b: Point| {
        let d = [b[0] - a[0], b[1] - a[1]];
        let l2 = d[0] * d[0] + d[1] * d[1];
        let t = if l2 > 0. {
            (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / l2).clamp(0., 1.)
        } else {
            0.
        };
        (p[0] - a[0] - t * d[0]).hypot(p[1] - a[1] - t * d[1])
    };
    // Split at the vertex farthest from the first: two open chains.
    let far = (1..n)
        .max_by(|&i, &j| {
            let (a, b) = (ring[0], [ring[i], ring[j]]);
            (b[0][0] - a[0])
                .hypot(b[0][1] - a[1])
                .total_cmp(&(b[1][0] - a[0]).hypot(b[1][1] - a[1]))
        })
        .unwrap();
    let mut keep = vec![false; n];
    keep[0] = true;
    keep[far] = true;
    let mut stack = vec![(0usize, far), (far, n)];
    while let Some((a, b)) = stack.pop() {
        let pb = ring[b % n];
        let (mut worst, mut at) = (0., None);
        for i in a + 1..b {
            let d = distance(ring[i], ring[a], pb);
            if d > worst {
                (worst, at) = (d, Some(i));
            }
        }
        if let Some(i) = at
            && worst > tolerance
        {
            keep[i] = true;
            stack.push((a, i));
            stack.push((i, b));
        }
    }
    (0..n).filter(|&i| keep[i]).map(|i| ring[i]).collect()
}

/// A region containing `region` whose rings are simplified within
/// `tolerance` and grown by it: every point of the exact region is in the
/// simplified one or within `tolerance` of a simplified edge (the removed
/// chain lies in that edge's band), and the square-join growth contains the
/// growth by a disc of that radius (`PreparedRegion::inflate_square`): the
/// result contains the exact region, with straight edges only. Holes too
/// small for the cutter are filled as for the exact shadow.
fn simplify_shadow(
    region: &PlanarRegion,
    tolerance: f64,
    construction: f64,
    fill_below_mm2: f64,
    options: ConstructionOptions,
) -> Result<PlanarRegion, String> {
    let ring = |b: &Boundary| -> Vec<Point> {
        simplify_ring(
            &b.vertices.iter().map(|v| [v[0], v[1]]).collect::<Vec<_>>(),
            tolerance,
        )
    };
    let boundary = |r: Vec<Point>| Boundary {
        vertices: r.into_iter().map(|p| [p[0], p[1], 0.]).collect(),
    };
    let mut components = vec![];
    for c in &region.components {
        if c.outer
            .vertices
            .iter()
            .chain(c.holes.iter().flat_map(|h| &h.vertices))
            .any(|v| v[2] != 0.)
        {
            return Err("the shadow has arcs".into());
        }
        components.push(Component {
            outer: boundary(ring(&c.outer)),
            holes: c
                .holes
                .iter()
                .map(&ring)
                .filter(|h| h.len() >= 3)
                .map(boundary)
                .collect(),
        });
    }
    let simplified = PlanarRegion {
        version: 1,
        units: "mm".into(),
        name: "retained-column-shadow".into(),
        components,
        boundary_error_mm: region.boundary_error_mm,
        source: serde_json::Value::Null,
    }
    .prepare_generated(options)?;
    let (grown, _) = simplified.inflate_square(tolerance + 4. * construction, fill_below_mm2)?;
    Ok(grown.region)
}

impl PreparedRoughingTarget {
    pub fn prepare(
        mesh: SurfaceMesh,
        tolerance_mm: f64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        check(cancel)?;
        let floor = mesh
            .triangles
            .iter()
            .flatten()
            .map(|p| p[2])
            .fold(f64::INFINITY, f64::min);
        // Reuse authoritative surface validation (finite geometry, real 3D
        // triangles, version/units/error/size). Build once, not once per layer.
        mesh.prepare(floor, tolerance_mm, cancel)?;
        let scale = mesh
            .triangles
            .iter()
            .flatten()
            .flatten()
            .fold(1f64, |a, b| a.max(b.abs()));
        let numerical = 32. * tolerance_mm + 512. * f64::EPSILON * (1. + scale);
        let source_sha256 = format!(
            "{:x}",
            Sha256::digest(serde_json::to_vec(&mesh.source).map_err(|e| e.to_string())?)
        );
        Ok(Self {
            mesh,
            tolerance: tolerance_mm,
            numerical,
            source_sha256,
        })
    }
    pub fn mesh(&self) -> &SurfaceMesh {
        &self.mesh
    }
    /// Final floor guard is separate from radial/axial stock allowances.
    pub fn floor_guard_mm(&self) -> f64 {
        2. * (self.numerical + self.mesh.surface_error_mm)
    }
    /// Protect every retained point above the lowest cutter tip, including all
    /// higher geometry. Degenerate projections are enclosed, never discarded.
    pub fn shadow(
        &self,
        tip_z_mm: f64,
        axial_allowance_mm: f64,
        work: &mut usize,
        cancel: &impl CancelSignal,
    ) -> Result<RoughingShadow, String> {
        self.shadow_for_cutter(tip_z_mm, axial_allowance_mm, 0., work, cancel)
    }
    /// `shadow`, for a cutter of radius `cutter_radius_mm`: holes of the
    /// projected union with less area than a disk of half that radius are
    /// filled (protected too). No cutter disk fits in such a hole, so the
    /// layer's tool-centre region is the same; filling only adds protection.
    /// It settles uncovered slivers between facets that the union returns
    /// touching another boundary within the tolerance (a mesh whose faces
    /// were tessellated separately leaves them along shared edges), which
    /// would otherwise make the layer's topology ambiguous.
    pub fn shadow_for_cutter(
        &self,
        tip_z_mm: f64,
        axial_allowance_mm: f64,
        cutter_radius_mm: f64,
        work: &mut usize,
        cancel: &impl CancelSignal,
    ) -> Result<RoughingShadow, String> {
        self.shadow_simplified(
            tip_z_mm,
            axial_allowance_mm,
            cutter_radius_mm,
            0.,
            work,
            cancel,
        )
    }
    /// `shadow_for_cutter`, then simplified within `simplify_mm` and grown by
    /// it (`simplify_shadow`): a region containing the exact shadow whose
    /// boundary has no more vertices than that distance needs, whatever the
    /// mesh's facet size. 0: the exact shadow.
    pub fn shadow_simplified(
        &self,
        tip_z_mm: f64,
        axial_allowance_mm: f64,
        cutter_radius_mm: f64,
        simplify_mm: f64,
        work: &mut usize,
        cancel: &impl CancelSignal,
    ) -> Result<RoughingShadow, String> {
        check(cancel)?;
        if !simplify_mm.is_finite() || !(0. ..=1e3).contains(&simplify_mm) {
            return Err("invalid roughing contour tolerance".into());
        }
        if !tip_z_mm.is_finite()
            || tip_z_mm.abs() > 1e6
            || !axial_allowance_mm.is_finite()
            || !(0. ..=1e6).contains(&axial_allowance_mm)
            || !cutter_radius_mm.is_finite()
            || !(0. ..=1e6).contains(&cutter_radius_mm)
        {
            return Err("invalid roughing shadow level/axial allowance/cutter radius".into());
        }
        let fill_below_mm2 = std::f64::consts::PI * (cutter_radius_mm / 2.).powi(2);
        let level = tip_z_mm - axial_allowance_mm - self.mesh.surface_error_mm - self.numerical;
        let options = ConstructionOptions {
            tolerance_mm: self.tolerance,
        };
        let mut regions: Vec<PreparedRegion> = vec![];
        let mut vertices = 0;
        let mut projected = 0;
        let mut degenerate = 0;
        for &triangle in self.mesh.triangles.iter() {
            check(cancel)?;
            *work = work
                .checked_add(1)
                .ok_or("roughing clipping work overflow")?;
            let mut points = clipped(triangle, level);
            if points.is_empty() {
                continue;
            }
            let area = if points.len() >= 3 {
                (1..points.len() - 1)
                    .map(|i| orientation(points[0], points[i], points[i + 1]))
                    .sum::<f64>()
            } else {
                0.
            };
            if area <= (64. * self.tolerance).powi(2) {
                // A small Minkowski box preserves even vertical walls/points.
                // This is explicit overprotection, not a mesh or point sample.
                let pad = 32. * self.tolerance;
                points = hull(
                    points
                        .iter()
                        .flat_map(|p| {
                            [-pad, pad].into_iter().flat_map(move |x| {
                                [-pad, pad].into_iter().map(move |y| [p[0] + x, p[1] + y])
                            })
                        })
                        .collect(),
                );
                degenerate += 1;
            }
            vertices += points.len();
            let region = PlanarRegion {
                version: 1,
                units: "mm".into(),
                name: "retained-column-shadow".into(),
                components: vec![Component {
                    outer: Boundary {
                        vertices: points.into_iter().map(|p| [p[0], p[1], 0.]).collect(),
                    },
                    holes: vec![],
                }],
                boundary_error_mm: self.mesh.surface_error_mm
                    + self.numerical
                    + if area <= (64. * self.tolerance).powi(2) {
                        32. * self.tolerance * 2f64.sqrt()
                    } else {
                        0.
                    },
                source: serde_json::Value::Null,
            }
            .prepare_generated(options)?;
            regions.push(region);
            projected += 1;
        }
        check(cancel)?;
        let unions = usize::from(regions.len() > 1);
        let mut filled = FilledHoles::default();
        let mut region = if regions.len() > 1 {
            let (union, holes) = PreparedRegion::union_linear_filling(&regions, fill_below_mm2)
                .map_err(|e| {
                    format!(
                        "{e}. The target mesh's facets above Z{level} project to boundaries \
                         that touch within the construction tolerance {} mm. This usually means \
                         the mesh is not watertight (faces tessellated separately, so neighbouring \
                         faces do not share their edge vertices): export the target with shared \
                         edge vertices",
                        self.tolerance
                    )
                })?;
            filled = holes;
            union.region
        } else if let Some(p) = regions.pop() {
            p.definition().clone()
        } else {
            PlanarRegion {
                version: 1,
                units: "mm".into(),
                name: "retained-column-shadow".into(),
                components: vec![],
                boundary_error_mm: self.mesh.surface_error_mm + self.numerical,
                source: serde_json::Value::Null,
            }
        };
        let exact_vertices = count_vertices(&region);
        let mut simplified = None;
        if simplify_mm > 0. && !region.components.is_empty() {
            match simplify_shadow(
                &region,
                simplify_mm,
                self.tolerance,
                fill_below_mm2,
                options,
            )
            .and_then(|r| {
                // Only a much simpler region is worth its extra stock: a
                // shadow whose facets are already coarse (a planar or coarse
                // mesh) keeps its exact boundary and toolpath.
                let n = count_vertices(&r);
                if 2 * n <= exact_vertices {
                    Ok(r)
                } else {
                    Err(format!(
                        "not much simpler ({n} vertices against the exact {exact_vertices}; simplified only from half as many)"
                    ))
                }
            }) {
                Ok(r) => {
                    simplified = Some(SimplifiedShadow {
                        tolerance_mm: simplify_mm,
                        exact_vertices,
                        reason: None,
                    });
                    region = r;
                }
                Err(e) => {
                    simplified = Some(SimplifiedShadow {
                        tolerance_mm: simplify_mm,
                        exact_vertices,
                        reason: Some(format!("kept the exact shadow: {e}")),
                    })
                }
            }
        }
        region.source = serde_json::json!({"adapter":"flat_column_shadow_v1","target_source_sha256":self.source_sha256});
        check(cancel)?;
        let report = ShadowReport {
            tip_z_mm,
            protected_from_z_mm: level,
            axial_allowance_mm,
            source_error_mm: self.mesh.surface_error_mm,
            numerical_z_reserve_mm: self.numerical,
            projected_triangles: projected,
            enclosed_degenerate_projections: degenerate,
            filled_holes: filled.holes,
            filled_hole_area_mm2: filled.area_mm2,
            simplified,
            projection_enclosure_mm: if degenerate > 0 {
                32. * self.tolerance * 2f64.sqrt()
            } else {
                0.
            },
            input_vertices: vertices,
            output_vertices: count_vertices(&region),
            boolean_operations: unions,
            boundary_error_mm: region.boundary_error_mm,
        };
        Ok(RoughingShadow { region, report })
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum RoughingClearing {
    Offset {
        stepover_mm: f64,
        #[serde(default = "default_coverage_margin")]
        coverage_margin_mm: f64,
    },
    Raster {
        stepover_mm: f64,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        routing: Option<RasterRouting>,
    },
    Adaptive {
        #[serde(default, skip_serializing_if = "AdaptivePathStrategy::is_default")]
        path_strategy: AdaptivePathStrategy,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        cutting_sense: Option<CuttingSense>,
        max_radial_engagement_mm: f64,
        orbit_radius_mm: Option<f64>,
        min_orbit_radius_mm: Option<f64>,
        local_orbit_tolerance_mm: Option<f64>,
        boundary_tolerance_mm: f64,
    },
}
fn default_coverage_margin() -> f64 {
    0.01
}
fn automatic_entry() -> PocketEntry {
    PocketEntry::StockEdgeOrHelix(HelicalEntry::default())
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RoughingOperation<G = SurfaceMesh> {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rest_tolerance_mm: Option<f64>,
    #[serde(default, skip_serializing_if = "no_part_depth_check")]
    pub omit_empty_cuts: bool,
    #[serde(default, skip_serializing_if = "no_part_depth_check")]
    pub require_part_at_first_depth: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub clearing_tolerance_mm: Option<f64>,
    /// Each layer's protected shadow is simplified within this distance and
    /// grown by it, so it still contains the exact shadow: the layer's moves
    /// no longer follow every facet of a fine mesh. None: a quarter of the
    /// radial allowance; 0: off. Adds at most twice this to the stock left.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub contour_tolerance_mm: Option<f64>,
    /// engagement_front only. A descent into a column that the committed
    /// sweeps prove clear for the whole tool assembly is rapid down to this
    /// far above the proven height, and only the rest is fed; without it
    /// every entry feeds down from `feed_z_mm` through material already
    /// cut. None: rapid descent stops at `feed_z_mm`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rapid_descent_clearance_mm: Option<f64>,
    /// engagement_front only. Each helical entry starts this far above the
    /// material its bore meets (proven by the committed sweeps or the stock
    /// cell bounds, and never above `feed_z_mm`), so the cutter is already
    /// ramping when it reaches stock. None: the helix starts at that
    /// material top.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ramp_clearance_mm: Option<f64>,
    /// engagement_front only. Stay-down links at depth feed at this rate
    /// (mm/min) wherever the committed stock's whole-cell bounds prove the
    /// cutter's footprint clear of stock; stretches that may touch stock keep
    /// `feed_mm_min`. None: every link feeds at `feed_mm_min`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub link_feed_mm_min: Option<f64>,
    /// engagement_front only, with `link_feed_mm_min`. A link stretch that
    /// may touch stock feeds faster by radial chip thinning: from an upper
    /// bound on its radial engagement (whole-cell stock bounds), at the feed
    /// that keeps the chip no thicker than `feed_mm_min` at the clearing's
    /// radial engagement, capped at `link_feed_mm_min`.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub link_chip_thinning: bool,
    pub id: String,
    pub tool: String,
    pub target: G,
    pub clearing: RoughingClearing,
    pub top_z_mm: f64,
    pub floor_z_mm: f64,
    pub stepdown_mm: f64,
    /// Step-up levels: after each full stepdown layer, extra layers climb back
    /// up through the remaining depth at most this far apart, each cut only in
    /// a band next to the part. See `stepup_layers`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stepup_mm: Option<f64>,
    pub clearance_z_mm: f64,
    pub feed_mm_min: f64,
    pub plunge_mm_min: f64,
    pub radial_allowance_mm: f64,
    pub bottom_allowance_mm: f64,
    pub tolerance_mm: f64,
    pub spindle_rpm: f64,
    pub work_offset: u8,
    pub tool_number: u32,
    pub length_register: u32,
    pub coolant: Coolant,
    pub clockwise: bool,
    #[serde(default = "automatic_entry")]
    pub entry: PocketEntry,
    /// See `PocketOperation::retract_z_mm`. Raster, offset and engagement_front clearing.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub retract_z_mm: Option<f64>,
    /// See `PocketOperation::feed_z_mm`. Raster, offset and engagement_front clearing.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub feed_z_mm: Option<f64>,
    /// Containment and avoid areas (clamps, fixtures, features to leave alone).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub limits: Option<crate::limits::Limits>,
}
impl<G> RoughingOperation<G> {
    pub(crate) fn map_geometry<H, E>(
        self,
        f: &mut impl FnMut(G) -> Result<H, E>,
    ) -> Result<RoughingOperation<H>, E> {
        Ok(RoughingOperation {
            limits: self.limits,
            rest_tolerance_mm: self.rest_tolerance_mm,
            omit_empty_cuts: self.omit_empty_cuts,
            require_part_at_first_depth: self.require_part_at_first_depth,
            clearing_tolerance_mm: self.clearing_tolerance_mm,
            contour_tolerance_mm: self.contour_tolerance_mm,
            rapid_descent_clearance_mm: self.rapid_descent_clearance_mm,
            ramp_clearance_mm: self.ramp_clearance_mm,
            link_feed_mm_min: self.link_feed_mm_min,
            link_chip_thinning: self.link_chip_thinning,
            id: self.id,
            tool: self.tool,
            target: f(self.target)?,
            clearing: self.clearing,
            top_z_mm: self.top_z_mm,
            floor_z_mm: self.floor_z_mm,
            stepdown_mm: self.stepdown_mm,
            stepup_mm: self.stepup_mm,
            clearance_z_mm: self.clearance_z_mm,
            feed_mm_min: self.feed_mm_min,
            plunge_mm_min: self.plunge_mm_min,
            radial_allowance_mm: self.radial_allowance_mm,
            bottom_allowance_mm: self.bottom_allowance_mm,
            tolerance_mm: self.tolerance_mm,
            spindle_rpm: self.spindle_rpm,
            work_offset: self.work_offset,
            tool_number: self.tool_number,
            length_register: self.length_register,
            coolant: self.coolant,
            clockwise: self.clockwise,
            entry: self.entry,
            retract_z_mm: self.retract_z_mm,
            feed_z_mm: self.feed_z_mm,
        })
    }
}

#[derive(Clone, Serialize)]
pub struct RoughingLayerReport {
    pub tip_z_mm: f64,
    /// For a step-up layer: the full stepdown layer below it whose band it cuts.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub step_up_of_z_mm: Option<f64>,
    pub shadow: ShadowReport,
    pub clearing: serde_json::Value,
}
#[derive(Clone, Serialize)]
pub struct MaterialTopCheck {
    pub tolerance_mm: f64,
    pub checked_cells: usize,
    pub maximum_upper_z_mm: Option<f64>,
    pub generation: Option<u64>,
}
#[derive(Clone, Serialize)]
pub struct HeightSchedule {
    pub authored_top_z_mm: f64,
    pub entry_top_z_mm: f64,
    pub skipped_air_depths: usize,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stock_top_check: Option<MaterialTopCheck>,
}
#[derive(Clone, Serialize)]
pub struct RoughingNoOp {
    pub reason: &'static str,
    pub checked_cells: usize,
    pub maximum_upper_z_mm: f64,
    pub cut_floor_z_mm: f64,
    pub generation: u64,
}
#[derive(Clone, Serialize)]
pub struct RoughingReport {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rest_tolerance_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub no_op: Option<RoughingNoOp>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub maximum_construction_error_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub height_schedule: Option<HeightSchedule>,
    pub strategy: &'static str,
    pub geometric_residual_status: ResidualStatus,
    pub clearing: &'static str,
    pub source_error_mm: f64,
    pub requested_floor_z_mm: f64,
    pub target_z_mm: f64,
    pub floor_geometry_guard_mm: f64,
    pub radial_allowance_mm: f64,
    pub bottom_allowance_mm: f64,
    pub max_radial_engagement_mm: Option<f64>,
    pub max_engagement_deg: Option<f64>,
    pub layers: Vec<RoughingLayerReport>,
    pub completed_layers: usize,
    pub active_layer: usize,
    pub prepared_guides: usize,
    pub prepared_region_vertices: usize,
    pub clipping_work: usize,
    pub motions: usize,
    pub stock_edge_entries: usize,
    pub helical_entries: usize,
    pub diagnostics: Vec<String>,
}
enum LayerPlan {
    Pocket(Box<PocketPlan>),
    Adaptive(Box<AdaptivePlan>),
    Engagement(Box<EngagementFrontPlan>),
}
impl LayerPlan {
    fn construction_error_mm(&self) -> f64 {
        match self {
            Self::Pocket(p) => p.construction_error_mm(),
            Self::Adaptive(p) => p.construction_error_mm(),
            Self::Engagement(p) => p.construction_error_mm(),
        }
    }
    fn activate(&mut self, p: Vec3, id: u64, spec: &StockSpec) -> Result<(), String> {
        match self {
            Self::Pocket(x) => x.activate_layer(p, id, spec),
            Self::Adaptive(x) => x.activate_layer(p, id, spec),
            Self::Engagement(x) => x.activate_layer(p, id, spec),
        }
    }
    fn next(&mut self, s: &Stock, c: &impl CancelSignal) -> Result<Option<Motion>, String> {
        match self {
            Self::Pocket(x) => x.next_motion_with_stock(s, c),
            Self::Adaptive(x) => x.next_motion_with_stock(s, c),
            Self::Engagement(x) => x.next_motion_with_stock(s, c),
        }
    }
    fn report(&self) -> serde_json::Value {
        let mut value = match self {
            Self::Pocket(x) => serde_json::to_value(x.report()).unwrap(),
            Self::Adaptive(x) => serde_json::to_value(x.report()).unwrap(),
            Self::Engagement(x) => serde_json::to_value(x.report()).unwrap(),
        };
        // Layer contours remain separately queryable through the shadow API.
        // Do not put every residual polygon into one bounded control frame.
        value.as_object_mut().unwrap().remove("geometric_residual");
        value
    }
    fn front_progress(&self) -> Option<FrontProgress> {
        if let Self::Adaptive(plan) = self {
            let r = plan.report();
            return r.front.as_ref().map(|front| FrontProgress {
                front: front.clone(),
                max_bounded_engagement_deg: r.max_bounded_engagement_deg,
                motions: r.motions,
                seed_orbits: r.seed_orbits,
                helical_entries: r.helical_entries,
            });
        }
        None
    }
    fn entries(&self) -> (usize, usize) {
        match self {
            Self::Pocket(x) => (x.report().stock_edge_entries, x.report().helical_entries),
            Self::Adaptive(x) => (x.report().stock_edge_entries, x.report().helical_entries),
            Self::Engagement(x) => (x.report().stock_edge_entries, x.report().helical_entries),
        }
    }
    fn prefix(&self) -> Vec<Event> {
        match self {
            Self::Pocket(x) => x.prefix_events(),
            Self::Adaptive(x) => x.prefix_events(),
            Self::Engagement(x) => x.prefix_events(),
        }
    }
    fn suffix(&self) -> Vec<Event> {
        match self {
            Self::Pocket(x) => x.suffix_events(),
            Self::Adaptive(x) => x.suffix_events(),
            Self::Engagement(x) => x.suffix_events(),
        }
    }
    fn finish(&mut self, c: &impl CancelSignal) -> Result<(), String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("roughing.layer.finish");
        match self {
            Self::Pocket(x) => {
                x.finish_report(c)?;
                Ok(())
            }
            Self::Adaptive(x) => x.finish_report(c),
            Self::Engagement(x) => x.finish_report(c),
        }
    }
}
// Snapshot only at the existing successful-motion reporting boundary. Reading
// the live layer from report() would expose changes from a rejected/cancelled
// planner attempt that the previous public report intentionally did not publish.
struct FrontProgress {
    front: FrontReport,
    max_bounded_engagement_deg: f64,
    motions: usize,
    seed_orbits: usize,
    helical_entries: usize,
}
impl FrontProgress {
    fn apply(&self, report: &mut serde_json::Value) {
        report["front"] = serde_json::to_value(&self.front).unwrap();
        report["max_bounded_engagement_deg"] = self.max_bounded_engagement_deg.into();
        report["motions"] = self.motions.into();
        report["seed_orbits"] = self.seed_orbits.into();
        report["helical_entries"] = self.helical_entries.into();
    }
}
pub struct RoughingPlan {
    depth_parts: Vec<(crate::verification::ToolPart, f64, f64)>,
    no_op_stock_identity: Option<u64>,
    part_depth_limit: Option<f64>,
    depth_policy_checked: bool,
    height_guard_mm: f64,
    plans: VecDeque<LayerPlan>,
    activated: bool,
    spec: StockSpec,
    position: Vec3,
    next_id: u64,
    generation: Option<u64>,
    prefix: Vec<Event>,
    suffix: Vec<Event>,
    report: RoughingReport,
    front_progress: Option<FrontProgress>,
    materialized_report: OnceLock<RoughingReport>,
}
impl RoughingPlan {
    pub fn prepare(
        op: RoughingOperation,
        tool: &ToolAssembly,
        spec: &StockSpec,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        Self::prepare_common(op, tool, spec, None, initial, first_id, cancel)
    }
    pub fn prepare_with_stock(
        op: RoughingOperation,
        tool: &ToolAssembly,
        stock: &Stock,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        Self::prepare_common(
            op,
            tool,
            stock.spec(),
            Some(stock),
            initial,
            first_id,
            cancel,
        )
    }
    fn prepare_common(
        op: RoughingOperation,
        tool: &ToolAssembly,
        spec: &StockSpec,
        stock: Option<&Stock>,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        check(cancel)?;
        spec.grid()?;
        let prepared_tool = tool.prepare()?;
        let r = prepared_tool.cutter().definition().diameter_mm / 2.;
        // The shadow simplification: a quarter of the radial allowance unless
        // stated, at most the allowance (it adds at most twice this).
        let contour = op
            .contour_tolerance_mm
            .unwrap_or(op.radial_allowance_mm / 4.);
        if !(contour.is_finite() && contour >= 0. && contour <= op.radial_allowance_mm.max(0.)) {
            return Err(format!(
                "roughing contour_tolerance_mm {contour} must be within 0..radial_allowance_mm ({})",
                op.radial_allowance_mm
            ));
        }
        if op.rest_tolerance_mm.is_some()
            && matches!(op.clearing, RoughingClearing::Adaptive { .. })
        {
            return Err(
                "Adaptive rest filtering is not implemented; use RasterClearing or OffsetClearing"
                    .into(),
            );
        }
        let target = PreparedRoughingTarget::prepare(op.target.clone(), op.tolerance_mm, cancel)?;
        let guard = target.floor_guard_mm();
        let floor = op.floor_z_mm + op.bottom_allowance_mm + guard;
        if op
            .clearing_tolerance_mm
            .is_some_and(|b| !b.is_finite() || !(1e-5..=1.).contains(&b) || guard > b)
        {
            return Err(
                "roughing clearing_tolerance_mm cannot enclose source/floor construction error"
                    .into(),
            );
        }
        let numeric_guard = 16. * op.tolerance_mm;
        crate::heights::require(
            "roughing",
            [
                (
                    [
                        op.top_z_mm,
                        op.floor_z_mm,
                        op.stepdown_mm,
                        op.bottom_allowance_mm,
                        op.radial_allowance_mm,
                    ]
                    .iter()
                    .all(|x| x.is_finite())
                        && op.top_z_mm.abs() <= 1e6,
                    "all numeric parameters must be finite".into(),
                ),
                (
                    op.clearance_z_mm > op.top_z_mm.max(spec.max_mm[2]) + numeric_guard,
                    format!(
                        "clearance_z_mm {} must be above top_z_mm {} and the stock top {}",
                        op.clearance_z_mm, op.top_z_mm, spec.max_mm[2]
                    ),
                ),
                (
                    op.floor_z_mm >= spec.min_mm[2],
                    format!(
                        "floor_z_mm {} must not be below the stock bottom {}",
                        op.floor_z_mm, spec.min_mm[2]
                    ),
                ),
                (
                    floor < op.top_z_mm && floor < spec.max_mm[2],
                    format!(
                        "floor_z_mm {} plus bottom allowance and construction guard ({floor}) must be below top_z_mm {} and the stock top {}",
                        op.floor_z_mm, op.top_z_mm, spec.max_mm[2]
                    ),
                ),
                (
                    op.stepdown_mm >= numeric_guard,
                    format!("stepdown_mm {} must be positive", op.stepdown_mm),
                ),
                (
                    op.stepup_mm
                        .is_none_or(|s| s.is_finite() && s >= numeric_guard && s < op.stepdown_mm),
                    format!(
                        "stepup_mm {:?} must be positive and smaller than stepdown_mm {}",
                        op.stepup_mm, op.stepdown_mm
                    ),
                ),
                (
                    op.bottom_allowance_mm >= 0. && op.radial_allowance_mm >= 0.,
                    "allowances must not be negative".into(),
                ),
                (
                    first_id != 0 && first_id.checked_add(1).is_some(),
                    "motion ID budget exhausted".into(),
                ),
            ],
        )?;
        for p in target.mesh.triangles.iter().flatten() {
            if (0..3).any(|i| {
                p[i] < spec.min_mm[i] - target.mesh.surface_error_mm - guard
                    || p[i] > spec.max_mm[i] + target.mesh.surface_error_mm + guard
            }) {
                return Err("retained roughing target exceeds declared stock envelope".into());
            }
        }
        let count = ((op.top_z_mm - floor) / op.stepdown_mm).ceil() as usize;
        if count == 0 {
            return Err("roughing needs at least one layer".into());
        }
        // The envelope is a preparation premise, not an assumed stock update.
        // next_motion checks committed whole-cell upper bounds before activation.
        let mut material_spec = spec.clone();
        material_spec.max_mm[2] = (op.top_z_mm + 16. * op.tolerance_mm).min(spec.max_mm[2]);
        let actual_top = stock.map(|s| committed_top(s, cancel)).transpose()?;
        if let Some(top) = actual_top {
            if top > material_spec.max_mm[2] {
                return Err(format!(
                    "roughing top_mm {} requires committed stock upper bounds <={}; current upper {top} remains high or uncertain",
                    op.top_z_mm, material_spec.max_mm[2]
                ));
            }
            material_spec.max_mm[2] = top;
        }
        let lowered = material_spec.max_mm[2] < spec.max_mm[2];
        let mut height_schedule =
            (op.top_z_mm != spec.max_mm[2] || lowered).then(|| HeightSchedule {
                authored_top_z_mm: op.top_z_mm,
                entry_top_z_mm: material_spec.max_mm[2],
                skipped_air_depths: 0,
                stock_top_check: lowered.then_some(MaterialTopCheck {
                    tolerance_mm: 16. * op.tolerance_mm,
                    checked_cells: stock.map_or(0, |s| s.cells().len()),
                    maximum_upper_z_mm: actual_top,
                    generation: stock.map(Stock::generation),
                }),
            });
        let part_depth_limit = op.require_part_at_first_depth.then(|| {
            target
                .mesh
                .triangles
                .iter()
                .flatten()
                .map(|p| p[2])
                .fold(f64::NEG_INFINITY, f64::max)
                + target.mesh.surface_error_mm
                + 32. * op.tolerance_mm
        });
        let (clearing, engagement, width, orbit) = match op.clearing {
            RoughingClearing::Offset { .. } => ("offset", None, None, 0.),
            RoughingClearing::Raster { .. } => ("raster", None, None, 0.),
            RoughingClearing::Adaptive {
                max_radial_engagement_mm: w,
                orbit_radius_mm,
                path_strategy,
                ..
            } => {
                if !w.is_finite() || w <= 0. || w > r {
                    return Err("adaptive roughing radial engagement must be positive and at most cutter radius".into());
                }
                // Width is an upper limit. A stricter native angle still
                // honors it, including widths whose angle would exceed170deg.
                let phi = if path_strategy == AdaptivePathStrategy::EngagementFront {
                    // The contact angle of a straight pass of width w, less
                    // the cutter's own trail: what engagement_front proves
                    // every stroke against.
                    (1. - w / r).acos().to_degrees()
                } else {
                    (2. * (1. - w / r).acos()).to_degrees().min(170.)
                };
                if !(1. ..=170.).contains(&phi) {
                    return Err("adaptive roughing engagement is outside native 1..170 degree circumference bound".into());
                }
                (
                    "adaptive",
                    Some(phi),
                    Some(w),
                    orbit_radius_mm.unwrap_or(r / 2.),
                )
            }
        };
        let margin = 4. * (r + op.radial_allowance_mm + orbit.max(r)) + guard;
        let bounds = [
            [spec.min_mm[0] - margin, spec.min_mm[1] - margin],
            [spec.max_mm[0] + margin, spec.max_mm[1] + margin],
        ];
        let stock_region = PlanarRegion {
            version: 1,
            units: "mm".into(),
            name: op.id.clone(),
            boundary_error_mm: 0.,
            source: serde_json::json!({"stock":spec}),
            components: vec![Component {
                outer: Boundary {
                    vertices: vec![
                        [bounds[0][0], bounds[0][1], 0.],
                        [bounds[1][0], bounds[0][1], 0.],
                        [bounds[1][0], bounds[1][1], 0.],
                        [bounds[0][0], bounds[1][1], 0.],
                    ],
                },
                holes: vec![],
            }],
        };
        let options = ConstructionOptions {
            tolerance_mm: op.tolerance_mm,
        };
        let expanded = stock_region.prepare(options)?;
        // A no-op still validates the selected tool/setup/entry/strategy. Use
        // the same scalar policies as actual layers, without inventing a layer.
        let step = match op.clearing {
            RoughingClearing::Offset { stepover_mm, .. }
            | RoughingClearing::Raster { stepover_mm, .. } => stepover_mm,
            RoughingClearing::Adaptive { .. } => r,
        };
        let template = PocketOperation {
            max_stay_down_mm: None,
            limits: op.limits.clone(),
            id: op.id.clone(),
            tool: op.tool.clone(),
            region: stock_region,
            top_z_mm: op.top_z_mm,
            floor_z_mm: floor,
            stepdown_mm: op.stepdown_mm,
            stepover_mm: step,
            feed_mm_min: op.feed_mm_min,
            plunge_mm_min: op.plunge_mm_min,
            clearance_z_mm: op.clearance_z_mm,
            radial_allowance_mm: op.radial_allowance_mm,
            bottom_allowance_mm: 0.,
            tolerance_mm: op.tolerance_mm,
            spindle_rpm: op.spindle_rpm,
            work_offset: op.work_offset,
            tool_number: op.tool_number,
            length_register: op.length_register,
            coolant: op.coolant,
            rest_tolerance_mm: op.rest_tolerance_mm,
            boundary_clockwise: Some(op.clockwise),
            entry: Some(op.entry.clone()),
            retract_z_mm: op.retract_z_mm,
            feed_z_mm: op.feed_z_mm,
        };
        template.validate_authoring(tool, prepared_tool.cutter().definition(), initial, r)?;
        for (name, value) in [
            ("rapid_descent_clearance_mm", op.rapid_descent_clearance_mm),
            ("ramp_clearance_mm", op.ramp_clearance_mm),
            ("link_feed_mm_min", op.link_feed_mm_min),
        ] {
            let Some(d) = value else { continue };
            if !matches!(
                op.clearing,
                RoughingClearing::Adaptive {
                    path_strategy: AdaptivePathStrategy::EngagementFront,
                    ..
                }
            ) {
                return Err(format!("{name} applies to engagement_front clearing only"));
            }
            let max = if name == "link_feed_mm_min" { 1e6 } else { 1e3 };
            if !d.is_finite() || d <= 0. || d > max {
                return Err(format!("roughing {name} {d} must be positive and finite"));
            }
        }
        if op.link_chip_thinning && op.link_feed_mm_min.is_none() {
            return Err("link_chip_thinning needs link_feed_mm_min: the thinned link feed is capped at the declared link feed".into());
        }
        match op.clearing {
            RoughingClearing::Offset {
                coverage_margin_mm, ..
            } => crate::pocket::validate_offset_margin(r, coverage_margin_mm)?,
            RoughingClearing::Adaptive {
                path_strategy: AdaptivePathStrategy::EngagementFront,
                orbit_radius_mm,
                min_orbit_radius_mm,
                local_orbit_tolerance_mm,
                ..
            } => {
                if orbit_radius_mm.is_some()
                    || min_orbit_radius_mm.is_some()
                    || local_orbit_tolerance_mm.is_some()
                {
                    return Err("engagement_front does not use orbits; leave orbit_radius_mm, min_orbit_radius_mm and local_orbit_tolerance_mm unset".into());
                }
                if op.omit_empty_cuts {
                    return Err("engagement_front plans every pass from the committed stock; omit_empty_cuts does not apply".into());
                }
            }
            RoughingClearing::Adaptive {
                path_strategy: AdaptivePathStrategy::DiskFront,
                ..
            } => {
                // #167. disk_front covers a region it takes to be solid with
                // orbits and never reads the stock. Roughing's region is the
                // stock footprint grown by a margin, so that the stock's edge
                // is no wall: disk_front then orbits through all the air
                // beside the stock (75 m of 137 m on a 130 x 90 mm stock).
                return Err(
                    "Roughing does not clear with path_strategy disk_front (also what a program \
                     saved without path_strategy asks for): it treats the whole region as \
                     material and never reads the stock, so it orbits through the air beside \
                     the stock. Use path_strategy engagement_front, AdaptiveClearing's default; \
                     disk_front remains AdaptivePocket's strategy"
                        .into(),
                );
            }
            RoughingClearing::Adaptive {
                path_strategy,
                cutting_sense,
                orbit_radius_mm,
                min_orbit_radius_mm,
                local_orbit_tolerance_mm,
                boundary_tolerance_mm,
                ..
            } => {
                AdaptivePocketOperation {
                    pocket: template,
                    path_strategy,
                    cutting_sense,
                    orbit_radius_mm,
                    min_orbit_radius_mm,
                    local_orbit_tolerance_mm,
                    boundary_tolerance_mm,
                    max_engagement_deg: engagement.unwrap(),
                    clockwise: op.clockwise,
                }
                .validate_parameters(r)?;
            }
            RoughingClearing::Raster { .. } => (),
        }
        let mut plans = VecDeque::new();
        let mut reports = vec![];
        let (mut work, mut vertices, mut guides) = (0usize, 0usize, 0usize);
        let mut maximum_error = guard;
        let mut depths = vec![];
        // The deepest full layer cut so far: everything above it outside its
        // shadow is already clear when a later layer (or step-up) starts.
        let mut last_full: Option<f64> = None;
        for i in 0..count {
            check(cancel)?;
            let z = (op.top_z_mm - (i + 1) as f64 * op.stepdown_mm).max(floor);
            if z >= material_spec.max_mm[2]
                || (lowered && z + 16. * op.tolerance_mm >= material_spec.max_mm[2])
            {
                height_schedule.as_mut().unwrap().skipped_air_depths += 1;
                continue;
            }
            if depths.is_empty()
                && stock.is_some()
                && part_depth_limit.is_some_and(|limit| z > limit)
            {
                return Err(format!(
                    "roughing top_mm: first material depth Z{z} is above retained part; face the stock first"
                ));
            }
            depths.push((z, None, last_full));
            if let Some(stepup) = op.stepup_mm {
                // Climb back up through this layer's full depth, below the
                // previous layer (or the material top) and never in air.
                let above = (op.top_z_mm - i as f64 * op.stepdown_mm).min(material_spec.max_mm[2]);
                for up in stepup_layers(z, above, stepup, numeric_guard) {
                    if up + 16. * op.tolerance_mm < material_spec.max_mm[2] {
                        depths.push((up, Some(z), last_full));
                    }
                }
            }
            last_full = Some(z);
        }
        // Offset/guide construction can dominate preflight even for a simple
        // faceted solid. Independent depth layers already have deterministic
        // ordered collection and shared cancellation; use that path for a
        // multi-layer job as well as for expensive individual mesh shadows.
        let workers = if depths.len() >= 4 || target.mesh.triangles.len() >= 4096 {
            std::thread::available_parallelism()
                .map_or(1, usize::from)
                .min(16)
        } else {
            1
        };
        let prepared = parallel::ordered(
            depths.len(),
            workers,
            cancel,
            |i, signal| {
                let cancel = || signal.is_cancelled();
                let (z, step_up_of, cleared_above) = depths[i];
                // Earlier layers clear the space above this one. Keep the
                // original entry plane while the flutes span it; deeper
                // layers admit only their actual increment below the last
                // full layer above them (a step-up's band cut clears less),
                // with a reserve for that sweep's conservative height bound.
                let mut layer_spec = material_spec.clone();
                if material_spec.max_mm[2] - z > tool.cutter.cutting_length_mm {
                    layer_spec.max_mm[2] = match cleared_above {
                        None => material_spec.max_mm[2],
                        Some(above) => above + 16. * op.tolerance_mm,
                    };
                }
                let cut_depth = layer_spec.max_mm[2] - z;
                if cut_depth > tool.cutter.cutting_length_mm {
                    return Err(crate::reach::ReachRefusal::cutting_length(&op.id, &tool.id, layer_spec.max_mm[2], z, tool.cutter.cutting_length_mm).error(format_args!(
                        "roughing layer {i} at Z{z}: actual cut depth {cut_depth} mm exceeds cutting length {} mm; use a smaller stepdown or longer flutes",
                        tool.cutter.cutting_length_mm
                    )));
                }
                let mut work = 0;
                let shadow = target
                    .shadow_simplified(z, op.bottom_allowance_mm, r, contour, &mut work, &cancel)
                    .map_err(|e| format!("roughing shadow at Z{z}: {e}"))?;
                let band = match step_up_of {
                    Some(below) => Some(
                        target
                            .shadow_simplified(
                                below,
                                op.bottom_allowance_mm,
                                r,
                                contour,
                                &mut work,
                                &cancel,
                            )
                            .and_then(|parent| {
                                stepup_band(&parent, r + op.radial_allowance_mm, options)
                            })
                            .map_err(|e| format!("roughing step-up band at Z{z}: {e}"))?,
                    ),
                    None => None,
                };
                let plan_layer = |shadow: RoughingShadow| -> Result<_, String> {
                    let mut region = expanded
                        .boolean(&shadow.region.prepare(options)?, Boolean::Difference)?
                        .region;
                    if let Some(band) = &band {
                        region = region
                            .prepare(options)?
                            .boolean(&band.prepare(options)?, Boolean::Intersection)?
                            .region;
                    }
                    let layer_vertices = count_vertices(&region);
                    let step = match op.clearing {
                        RoughingClearing::Raster { stepover_mm, .. }
                        | RoughingClearing::Offset { stepover_mm, .. } => stepover_mm,
                        RoughingClearing::Adaptive { .. } => r,
                    };
                    let pocket = PocketOperation {
                        max_stay_down_mm: None,
                        limits: op.limits.clone(),
                        id: op.id.clone(),
                        tool: op.tool.clone(),
                        region,
                        top_z_mm: layer_spec.max_mm[2],
                        floor_z_mm: z,
                        stepdown_mm: layer_spec.max_mm[2] - z,
                        stepover_mm: step,
                        feed_mm_min: op.feed_mm_min,
                        plunge_mm_min: op.plunge_mm_min,
                        clearance_z_mm: op.clearance_z_mm,
                        radial_allowance_mm: op.radial_allowance_mm,
                        bottom_allowance_mm: 0.,
                        tolerance_mm: op.tolerance_mm,
                        spindle_rpm: op.spindle_rpm,
                        work_offset: op.work_offset,
                        tool_number: op.tool_number,
                        length_register: op.length_register,
                        coolant: op.coolant,
                        rest_tolerance_mm: op.rest_tolerance_mm,
                        boundary_clockwise: Some(op.clockwise),
                        entry: Some(op.entry.clone()),
                        retract_z_mm: op.retract_z_mm,
                        feed_z_mm: op.feed_z_mm,
                    };
                    let mut plan = match op.clearing {
                        RoughingClearing::Offset {
                            coverage_margin_mm, ..
                        } => LayerPlan::Pocket(Box::new(PocketPlan::prepare_offset(
                            pocket,
                            tool,
                            &layer_spec,
                            initial,
                            1,
                            coverage_margin_mm,
                            &cancel,
                        )?)),
                        RoughingClearing::Raster { routing, .. } => {
                            let p = PocketPlan::prepare(
                                pocket,
                                tool,
                                &layer_spec,
                                initial,
                                1,
                                &cancel,
                            )?;
                            LayerPlan::Pocket(Box::new(if let Some(policy) = routing {
                                p.with_routing(policy, &cancel)?
                            } else {
                                p
                            }))
                        }
                        RoughingClearing::Adaptive {
                            path_strategy: AdaptivePathStrategy::EngagementFront,
                            max_radial_engagement_mm,
                            cutting_sense,
                            ..
                        } => LayerPlan::Engagement(Box::new(
                            EngagementFrontPlan::prepare(
                                pocket,
                                tool,
                                &layer_spec,
                                initial,
                                max_radial_engagement_mm,
                                cutting_sense.unwrap_or(CuttingSense::Climb) == CuttingSense::Climb,
                                &cancel,
                            )?
                            .with_rapid_descent(op.rapid_descent_clearance_mm)
                            .with_ramp_clearance(op.ramp_clearance_mm)
                            .with_link_feed(op.link_feed_mm_min, op.link_chip_thinning),
                        )),
                        RoughingClearing::Adaptive {
                            path_strategy,
                            cutting_sense,
                            orbit_radius_mm,
                            min_orbit_radius_mm,
                            local_orbit_tolerance_mm,
                            boundary_tolerance_mm,
                            ..
                        } => LayerPlan::Adaptive(Box::new(AdaptivePlan::prepare(
                            AdaptivePocketOperation {
                                pocket,
                                path_strategy,
                                cutting_sense,
                                orbit_radius_mm,
                                min_orbit_radius_mm,
                                local_orbit_tolerance_mm,
                                boundary_tolerance_mm,
                                max_engagement_deg: engagement.unwrap(),
                                clockwise: op.clockwise,
                            },
                            tool,
                            &layer_spec,
                            initial,
                            1,
                            &cancel,
                        )?)),
                    };
                    if op.omit_empty_cuts {
                        match &mut plan {
                            LayerPlan::Pocket(p) => p.enable_empty_cuts(),
                            LayerPlan::Adaptive(p) => p.enable_empty_cuts(),
                            LayerPlan::Engagement(_) => (),
                        }
                    }
                    let layer_error = plan.construction_error_mm();
                    if op
                        .clearing_tolerance_mm
                        .is_some_and(|budget| layer_error > budget)
                    {
                        return Err(format!(
                            "roughing clearing_tolerance_mm cannot enclose layer construction error {layer_error} at Z{z}"
                        ));
                    }
                    let layer_guides = match &plan {
                        LayerPlan::Pocket(p) => p.report().xy_strokes,
                        LayerPlan::Adaptive(p) => {
                            p.report().guide_strokes + p.report().navigation_links
                        }
                        LayerPlan::Engagement(_) => 0,
                    };
                    let report = RoughingLayerReport {
                        tip_z_mm: z,
                        step_up_of_z_mm: step_up_of,
                        shadow: shadow.report,
                        clearing: plan.report(),
                    };
                    Ok((plan, report, layer_vertices, layer_guides, layer_error))
                };
                // The simplified shadow only grows the protected area, so it
                // can narrow a gap the cutter still fits until no entry
                // reaches it (#123): that layer keeps the exact shadow.
                let simplified = shadow
                    .report
                    .simplified
                    .as_ref()
                    .is_some_and(|s| s.reason.is_none());
                let (plan, report, layer_vertices, layer_guides, layer_error) = match plan_layer(
                    shadow,
                ) {
                    Err(e) if simplified && e.contains(crate::entry::NO_ENTRY) => {
                        let exact = target
                            .shadow_simplified(z, op.bottom_allowance_mm, r, 0., &mut work, &cancel)
                            .map_err(|e| format!("roughing shadow at Z{z}: {e}"))?;
                        let exact_vertices = exact.report.output_vertices;
                        let (plan, mut report, v, g, error) = plan_layer(exact)?;
                        report.shadow.simplified = Some(SimplifiedShadow {
                            tolerance_mm: contour,
                            exact_vertices,
                            reason: Some(format!(
                                "kept the exact shadow: the simplified one left a region no entry reaches ({e})"
                            )),
                        });
                        (plan, report, v, g, error)
                    }
                    result => result?,
                };
                Ok((
                    plan,
                    report,
                    layer_vertices,
                    layer_guides,
                    work,
                    layer_error,
                ))
            },
            |(_, _, layer_vertices, layer_guides, layer_work, _)| {
                vertices += layer_vertices;
                guides += layer_guides;
                work += layer_work;
                // Prepared layers are retained until drained: check the
                // accumulated geometry against available memory.
                crate::resources::check_memory(
                    (vertices as u128 * 3 + guides as u128 * 8) * 8,
                    "prepared roughing layers",
                )
            },
        )?;
        // One clearance scene for all layers: each layer extends it with the
        // sweeps committed since, instead of rebuilding from the whole job.
        let entry_scene = crate::pocket::EntrySceneCache::default();
        for (mut plan, report, _, _, _, error) in prepared {
            maximum_error = maximum_error.max(error);
            if let LayerPlan::Pocket(p) = &mut plan {
                p.share_entry_scene(&entry_scene);
            }
            plans.push_back(plan);
            reports.push(report);
        }
        let no_op = if plans.is_empty() {
            let stock = stock.ok_or("empty roughing requires current committed stock")?;
            let top = actual_top.ok_or("empty roughing requires a committed stock height proof")?;
            if top > floor {
                return Err("roughing has no material depth below the committed envelope within construction resolution; remaining upper stock is above the cut floor".into());
            }
            Some(RoughingNoOp {
                reason: "committed_stock_below_cut_floor",
                checked_cells: stock.cells().len(),
                maximum_upper_z_mm: top,
                cut_floor_z_mm: floor,
                generation: stock.generation(),
            })
        } else {
            None
        };
        let no_op_stock_identity = no_op.as_ref().map(|_| stock.unwrap().identity());
        let prefix = plans.front().map_or_else(Vec::new, LayerPlan::prefix);
        let suffix = plans.front().map_or_else(Vec::new, LayerPlan::suffix);

        let report=RoughingReport{rest_tolerance_mm:op.rest_tolerance_mm,no_op,maximum_construction_error_mm:op.clearing_tolerance_mm.map(|_|maximum_error),height_schedule,strategy:"roughing",geometric_residual_status:ResidualStatus::Pending,clearing,source_error_mm:target.mesh.surface_error_mm,
            requested_floor_z_mm:op.floor_z_mm,target_z_mm:floor,floor_geometry_guard_mm:guard,
            radial_allowance_mm:op.radial_allowance_mm,bottom_allowance_mm:op.bottom_allowance_mm,
            max_radial_engagement_mm:width,max_engagement_deg:engagement,layers:reports,completed_layers:0,active_layer:0,
            prepared_guides:guides,prepared_region_vertices:vertices,clipping_work:work,motions:0,stock_edge_entries:0,helical_entries:0,
            diagnostics:vec!["Flat-cutter column shadows protect the downward closure of all original facets above each tip; source/Boolean engineering errors included".into(),
                "Per-layer nominal residual areas are estimates; actual remaining stock and independent part/assembly/fixture checks are separate".into()]};
        check(cancel)?;
        Ok(Self {
            depth_parts: if material_spec.max_mm[2] - floor > tool.cutter.cutting_length_mm {
                crate::verification::parts(&prepared_tool)?
                    .into_iter()
                    .filter(|p| p.kind != crate::verification::ToolPart::Cutter)
                    .map(|p| (p.kind, p.min, p.radius.at(p.min).max(p.radius.at(p.max))))
                    .collect()
            } else {
                vec![]
            },
            no_op_stock_identity,
            part_depth_limit,
            depth_policy_checked: stock.is_some() || !op.require_part_at_first_depth,
            height_guard_mm: 16. * op.tolerance_mm,
            plans,
            activated: false,
            spec: spec.clone(),
            position: initial,
            next_id: first_id,
            generation: stock.map(Stock::generation),
            prefix,
            suffix,
            report,
            front_progress: None,
            materialized_report: OnceLock::new(),
        })
    }
    pub fn report(&self) -> &RoughingReport {
        let Some(progress) = &self.front_progress else {
            return &self.report;
        };
        // Ordinary Rust borrows keep a returned snapshot valid until the next
        // mutable planner call. Repeated readers share one materialization;
        // generation without report consumers allocates no per-motion JSON.
        self.materialized_report.get_or_init(|| {
            let mut report = self.report.clone();
            progress.apply(&mut report.layers[report.active_layer].clearing);
            report
        })
    }
    pub fn prefix_events(&self) -> Vec<Event> {
        self.prefix.clone()
    }
    pub fn suffix_events(&self) -> Vec<Event> {
        self.suffix.clone()
    }
    pub fn next_motion_with_stock(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        check(cancel)?;
        if self
            .no_op_stock_identity
            .is_some_and(|id| id != stock.identity())
            || stock.spec() != &self.spec
            || stock.generation() == u64::MAX
            || self.generation.is_some_and(|g| g != stock.generation())
        {
            return Err(
                "commit preceding roughing motion to matching stock before planning".into(),
            );
        }
        if !self.depth_policy_checked {
            let top = committed_top(stock, cancel)?;
            if let Some(first) = self
                .report
                .layers
                .iter()
                .find(|l| l.tip_z_mm + self.height_guard_mm < top)
                && self
                    .part_depth_limit
                    .is_some_and(|limit| first.tip_z_mm > limit)
            {
                return Err("roughing top_mm: first material depth is above retained part; face the stock first".into());
            }
            self.depth_policy_checked = true;
        }
        if let Some(schedule) = &mut self.report.height_schedule
            && let Some(proof) = &mut schedule.stock_top_check
            && proof.generation.is_none()
        {
            let mut maximum = f64::NEG_INFINITY;
            for (i, cell) in stock.cells().iter().enumerate() {
                if i.is_multiple_of(1024) {
                    check(cancel)?;
                }
                if cell.upper_mm > schedule.entry_top_z_mm {
                    return Err(format!(
                        "roughing authored top {} requires committed stock upper bounds <={}; cell {i} upper {} remains high or uncertain",
                        schedule.authored_top_z_mm, schedule.entry_top_z_mm, cell.upper_mm
                    ));
                }
                maximum = maximum.max(cell.upper_mm);
            }
            check(cancel)?;
            proof.checked_cells = stock.cells().len();
            proof.maximum_upper_z_mm = Some(maximum);
            proof.generation = Some(stock.generation());
            self.generation = Some(stock.generation());
            self.materialized_report.take();
        }
        loop {
            check(cancel)?;
            let Some(plan) = self.plans.front_mut() else {
                return Ok(None);
            };
            if !self.activated {
                plan.activate(self.position, self.next_id, &self.spec)?;
                self.activated = true;
            }
            let old = plan.entries();
            if let Some(m) = plan.next(stock, cancel).map_err(|error| {
                let report = plan.report();
                format!(
                    "{error}; roughing layer {} at Z{}, front={}",
                    self.report.active_layer,
                    self.report.layers[self.report.active_layer].tip_z_mm,
                    report.get("front").unwrap_or(&serde_json::Value::Null)
                )
            })? {
                let bounds = m.curve.prepare(self.height_guard_mm / 16.)?.bounds();
                let mut clearance_work = u64::MAX;
                for &(part, height, radius) in &self.depth_parts {
                    let pad = radius + self.height_guard_mm;
                    let rect = [
                        [bounds.min[0] - pad, bounds.min[1] - pad],
                        [bounds.max[0] + pad, bounds.max[1] + pad],
                    ];
                    if !stock.bounds_clear(
                        rect,
                        bounds.min[2] + height - self.height_guard_mm,
                        &mut clearance_work,
                        cancel,
                    )? && !stock.repeated_clearance(
                        &m,
                        radius,
                        height,
                        &mut clearance_work,
                        cancel,
                    )? {
                        return Err(format!(
                            "roughing layer {} at Z{}: {:?} clearance is not proven, {} mm above the tip; to keep it above the original stock, tip Z must exceed {} mm; remove obstructing stock first or use a longer tool assembly",
                            self.report.active_layer,
                            self.report.layers[self.report.active_layer].tip_z_mm,
                            part,
                            height,
                            stock.spec().max_mm[2] - height + self.height_guard_mm
                        ));
                    }
                }
                let new = plan.entries();
                self.report.stock_edge_entries += new.0 - old.0;
                self.report.helical_entries += new.1 - old.1;
                self.position = m.curve.end();
                self.next_id = m.id + 1;
                self.generation = Some(stock.generation() + 1);
                self.report.motions += 1;
                self.front_progress = plan.front_progress();
                self.materialized_report.take();
                return Ok(Some(m));
            }
            plan.finish(cancel)?;
            self.report.layers[self.report.active_layer].clearing = plan.report();
            self.front_progress = None;
            self.materialized_report.take();
            self.report.completed_layers += 1;
            self.report.active_layer += 1;
            let memory = match self.plans.pop_front() {
                Some(LayerPlan::Engagement(mut done)) => done.take_memory(),
                _ => None,
            };
            if let (Some(memory), Some(LayerPlan::Engagement(next))) =
                (memory, self.plans.front_mut())
            {
                next.give_memory(memory);
            }
            self.activated = false;
        }
    }
    pub fn finish_report(&mut self, cancel: &impl CancelSignal) -> Result<(), String> {
        check(cancel)?;
        if self.plans.is_empty() {
            self.materialized_report.take();
            self.report.geometric_residual_status = if !self.report.layers.is_empty()
                && self
                    .report
                    .layers
                    .iter()
                    .all(|l| l.clearing["geometric_residual_status"] == "estimated")
            {
                ResidualStatus::Estimated
            } else {
                ResidualStatus::Unknown
            };
            Ok(())
        } else {
            Err("roughing report requires all layers to finish".into())
        }
    }
}

#[cfg(test)]
mod simplify_tests {
    use super::*;
    use crate::planar::{Boundary, Component, ConstructionOptions, PlanarRegion};

    fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Boundary {
        Boundary {
            vertices: vec![[x0, y0, 0.], [x1, y0, 0.], [x1, y1, 0.], [x0, y1, 0.]],
        }
    }

    #[test]
    fn a_ring_simplification_would_collapse_is_refused_not_dropped() {
        // A shadow island narrower than twice the tolerance collapses to a
        // segment under Douglas-Peucker. Dropping it would lose protected
        // area: the simplification is refused instead, so the layer keeps
        // its exact shadow (and reports why).
        let region = PlanarRegion {
            version: 1,
            units: "mm".into(),
            name: "shadow".into(),
            components: vec![
                Component {
                    outer: rect(0., 0., 10., 10.),
                    holes: vec![],
                },
                Component {
                    outer: rect(20., 0., 30., 0.05),
                    holes: vec![],
                },
            ],
            boundary_error_mm: 0.,
            source: serde_json::Value::Null,
        };
        let options = ConstructionOptions { tolerance_mm: 1e-5 };
        assert!(simplify_shadow(&region, 0.075, 1e-5, 0., options).is_err());
        // Wide enough to keep its ring: simplified, and the island stays
        // protected.
        let mut wide = region.clone();
        wide.components[1].outer = rect(20., 0., 30., 0.2);
        let grown = simplify_shadow(&wide, 0.075, 1e-5, 0., options)
            .unwrap()
            .prepare(options)
            .unwrap();
        for p in [[20., 0.], [30., 0.2], [25., 0.1], [0., 0.], [10., 10.]] {
            assert!(grown.query(p).unwrap().signed_clearance_mm.unwrap() > 0.07);
        }
    }

    #[test]
    fn step_up_band_of_a_round_shadow_stays_straight_sided_and_unnormalized() {
        // A finely faceted cone: its shadow is round. A round offset of it
        // came back from the band intersection as thousands of tiny
        // segments; eroding that by the cutter needed a normalization
        // envelope whose error the pocket's clearance proof then refused
        // (Core/test-block step-up layers). The band must stay coarse.
        let n = 512;
        let apex = [0., 0., 10.];
        let rim = |k: usize| {
            let t = std::f64::consts::TAU * k as f64 / n as f64;
            [10. * t.cos(), 10. * t.sin(), 0.]
        };
        let mesh = SurfaceMesh {
            version: 1,
            units: "mm".into(),
            surface_error_mm: 0.,
            source: serde_json::Value::Null,
            triangles: (0..n)
                .map(|k| [rim(k), rim(k + 1), apex])
                .collect::<Vec<_>>()
                .into(),
        };
        let cancel = || false;
        let target = PreparedRoughingTarget::prepare(mesh, 1e-5, &cancel).unwrap();
        let options = ConstructionOptions { tolerance_mm: 1e-5 };
        let (r, allowance, contour) = (3., 0.4, 0.075);
        let reach = r + allowance;
        // The layer at Z6 around the cone, inside a 60 mm square of stock.
        let shadow = target
            .shadow_simplified(6., 0., r, contour, &mut 0, &cancel)
            .unwrap();
        let square = PlanarRegion {
            version: 1,
            units: "mm".into(),
            name: "stock".into(),
            components: vec![Component {
                outer: rect(-30., -30., 30., 30.),
                holes: vec![],
            }],
            boundary_error_mm: 0.,
            source: serde_json::Value::Null,
        };
        let layer = square
            .prepare(options)
            .unwrap()
            .boolean(
                &shadow.region.prepare(options).unwrap(),
                Boolean::Difference,
            )
            .unwrap()
            .region;
        // As the layer loop does: the parent (Z4) shadow's band, intersected
        // with the step-up layer's region.
        let below = target
            .shadow_simplified(4., 0., r, contour, &mut 0, &cancel)
            .unwrap();
        let band = layer
            .prepare(options)
            .unwrap()
            .boolean(
                &stepup_band(&below, reach, options)
                    .unwrap()
                    .prepare(options)
                    .unwrap(),
                Boolean::Intersection,
            )
            .unwrap()
            .region;
        let vertices: Vec<_> = band
            .components
            .iter()
            .flat_map(|c| std::iter::once(&c.outer).chain(&c.holes))
            .flat_map(|b| &b.vertices)
            .collect();
        assert!(vertices.iter().all(|v| v[2] == 0.), "band has arcs");
        assert!(vertices.len() < 400, "band has {} vertices", vertices.len());
        // It still reaches 3r + 2a beyond the shadow at Z4 (radius 6 here).
        let prepared = band.prepare(options).unwrap();
        for k in 0..64 {
            let t = std::f64::consts::TAU * k as f64 / 64.;
            let d = 6. + 3. * r + 2. * allowance - 0.01;
            assert!(
                prepared
                    .query([d * t.cos(), d * t.sin()])
                    .unwrap()
                    .signed_clearance_mm
                    .unwrap()
                    > 0.,
                "band misses {t}"
            );
        }
        // The cutter-centre construction needs no normalization envelope.
        let centres = prepared.cutter_region(r, allowance).unwrap();
        assert_eq!(
            centres.normalization_error_mm, 0.,
            "{:?}",
            centres.diagnostics
        );
    }
}
