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

pub(crate) mod parallel;

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
fn count_vertices(region: &PlanarRegion) -> usize {
    region
        .components
        .iter()
        .flat_map(|c| std::iter::once(&c.outer).chain(&c.holes))
        .map(|b| b.vertices.len())
        .sum()
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
        check(cancel)?;
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
    pub id: String,
    pub tool: String,
    pub target: G,
    pub clearing: RoughingClearing,
    pub top_z_mm: f64,
    pub floor_z_mm: f64,
    pub stepdown_mm: f64,
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
    /// See `PocketOperation::retract_z_mm`. Raster/offset clearing only.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub retract_z_mm: Option<f64>,
    /// See `PocketOperation::feed_z_mm`. Raster/offset clearing only.
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
            id: self.id,
            tool: self.tool,
            target: f(self.target)?,
            clearing: self.clearing,
            top_z_mm: self.top_z_mm,
            floor_z_mm: self.floor_z_mm,
            stepdown_mm: self.stepdown_mm,
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
}
impl LayerPlan {
    fn construction_error_mm(&self) -> f64 {
        match self {
            Self::Pocket(p) => p.construction_error_mm(),
            Self::Adaptive(p) => p.construction_error_mm(),
        }
    }
    fn activate(&mut self, p: Vec3, id: u64, spec: &StockSpec) -> Result<(), String> {
        match self {
            Self::Pocket(x) => x.activate_layer(p, id, spec),
            Self::Adaptive(x) => x.activate_layer(p, id, spec),
        }
    }
    fn next(&mut self, s: &Stock, c: &impl CancelSignal) -> Result<Option<Motion>, String> {
        match self {
            Self::Pocket(x) => x.next_motion_with_stock(s, c),
            Self::Adaptive(x) => x.next_motion_with_stock(s, c),
        }
    }
    fn report(&self) -> serde_json::Value {
        let mut value = match self {
            Self::Pocket(x) => serde_json::to_value(x.report()).unwrap(),
            Self::Adaptive(x) => serde_json::to_value(x.report()).unwrap(),
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
        }
    }
    fn prefix(&self) -> Vec<Event> {
        match self {
            Self::Pocket(x) => x.prefix_events(),
            Self::Adaptive(x) => x.prefix_events(),
        }
    }
    fn suffix(&self) -> Vec<Event> {
        match self {
            Self::Pocket(x) => x.suffix_events(),
            Self::Adaptive(x) => x.suffix_events(),
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
                ..
            } => {
                if !w.is_finite() || w <= 0. || w > r {
                    return Err("adaptive roughing radial engagement must be positive and at most cutter radius".into());
                }
                // Width is an upper limit. A stricter native angle still
                // honors it, including widths whose angle would exceed170deg.
                let phi = (2. * (1. - w / r).acos()).to_degrees().min(170.);
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
        match op.clearing {
            RoughingClearing::Offset {
                coverage_margin_mm, ..
            } => crate::pocket::validate_offset_margin(r, coverage_margin_mm)?,
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
            depths.push(z);
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
                let z = depths[i];
                let mut work = 0;
                let shadow = target
                    .shadow_for_cutter(z, op.bottom_allowance_mm, r, &mut work, &cancel)
                    .map_err(|e| format!("roughing shadow at Z{z}: {e}"))?;
                let region = expanded
                    .boolean(&shadow.region.prepare(options)?, Boolean::Difference)?
                    .region;
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
                    top_z_mm: material_spec.max_mm[2],
                    floor_z_mm: z,
                    stepdown_mm: material_spec.max_mm[2] - z,
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
                        &material_spec,
                        initial,
                        1,
                        coverage_margin_mm,
                        &cancel,
                    )?)),
                    RoughingClearing::Raster { routing, .. } => {
                        let p =
                            PocketPlan::prepare(pocket, tool, &material_spec, initial, 1, &cancel)?;
                        LayerPlan::Pocket(Box::new(if let Some(policy) = routing {
                            p.with_routing(policy, &cancel)?
                        } else {
                            p
                        }))
                    }
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
                        &material_spec,
                        initial,
                        1,
                        &cancel,
                    )?)),
                };
                if op.omit_empty_cuts {
                    match &mut plan {
                        LayerPlan::Pocket(p) => p.enable_empty_cuts(),
                        LayerPlan::Adaptive(p) => p.enable_empty_cuts(),
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
                };
                let report = RoughingLayerReport {
                    tip_z_mm: z,
                    shadow: shadow.report,
                    clearing: plan.report(),
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
            self.plans.pop_front();
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
