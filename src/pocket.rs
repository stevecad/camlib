//! Baseline prismatic pocket clearing. XY geometry is prepared once; depth passes
//! and attributed M1 motion are pulled lazily. Geometry fit is not M5 verification.
pub use crate::entry::{HelicalEntry, PocketEntry};
mod air_entry;
mod offset;
mod routing;
use crate::{
    Point,
    entry::{EntryFitOptions, PendingMove, PocketEntries, entry_line, fit_entries},
    motion::{Coolant, Curve, Event, Motion, MoveKind, Plane, SpindleDirection, Vec3},
    planar::{Boolean, Boundary, ConstructionOptions, PlanarRegion, PreparedRegion},
    stock::{CancelSignal, Stock, StockSpec},
    tool::{CutterShape, ToolAssembly, valid_id},
};
pub use offset::OffsetReport;
pub(crate) use offset::validate_margin as validate_offset_margin;
use robust::{Coord, orient2d};
pub use routing::{RasterRouting, RoutingReport};
use serde::{Deserialize, Serialize};
use std::{
    collections::VecDeque,
    f64::consts::{FRAC_PI_2, PI, TAU},
    sync::Arc,
};

#[derive(Clone, Copy)]
enum GuideMode {
    Raster(Option<f64>),
    Offset { coverage_margin_mm: f64 },
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Operation<G = crate::surface::SurfaceMesh> {
    Roughing(crate::roughing::RoughingOperation<G>),
    Pocket(PocketOperation),
    AdaptivePocket(crate::adaptive::AdaptivePocketOperation),
    TrochoidalSlot(crate::slot::TrochoidalSlotOperation),
    Facing(crate::facing::FacingOperation),
    Contour(crate::contour::ContourOperation),
    Drilling(crate::holes::DrillingOperation),
    Bore(crate::holes::BoreOperation),
    Chamfer(crate::chamfer::ChamferOperation),
    Deburr(crate::deburr::DeburrOperation<G>),
    Parallel(crate::finishing::ParallelOperation<G>),
    Waterline(crate::waterline::WaterlineOperation<G>),
    Trace(crate::trace::TraceOperation),
    ThreadMill(crate::holes::ThreadMillOperation),
    Tapping(crate::holes::TappingOperation),
    Reaming(crate::holes::ReamingOperation),
    Flat(crate::flat::FlatOperation<G>),
    VCarve(crate::vcarve::VCarveOperation),
    ThermalCut(crate::thermal::ThermalCutOperation),
}
impl<G> Operation<G> {
    /// Hole operations whose retract feed is a declared process parameter
    /// (feeding out of a reamed or bored hole, a drill's retract feed): their
    /// retracts keep that feed and never become rapids.
    pub fn declares_retract_feed(&self) -> bool {
        matches!(
            self,
            Self::Drilling(_) | Self::Bore(_) | Self::Reaming(_) | Self::Tapping(_)
        )
    }
    pub(crate) fn map_geometry<H, E>(
        self,
        f: &mut impl FnMut(G) -> Result<H, E>,
    ) -> Result<Operation<H>, E> {
        Ok(match self {
            Self::Roughing(op) => Operation::Roughing(op.map_geometry(f)?),
            Self::Pocket(op) => Operation::Pocket(op),
            Self::AdaptivePocket(op) => Operation::AdaptivePocket(op),
            Self::TrochoidalSlot(op) => Operation::TrochoidalSlot(op),
            Self::Facing(op) => Operation::Facing(op),
            Self::Contour(op) => Operation::Contour(op),
            Self::Drilling(op) => Operation::Drilling(op),
            Self::Bore(op) => Operation::Bore(op),
            Self::Chamfer(op) => Operation::Chamfer(op),
            Self::Deburr(op) => Operation::Deburr(op.map_geometry(f)?),
            Self::Parallel(op) => Operation::Parallel(op.map_geometry(f)?),
            Self::Waterline(op) => Operation::Waterline(op.map_geometry(f)?),
            Self::Trace(op) => Operation::Trace(op),
            Self::ThreadMill(op) => Operation::ThreadMill(op),
            Self::Tapping(op) => Operation::Tapping(op),
            Self::Reaming(op) => Operation::Reaming(op),
            Self::Flat(op) => Operation::Flat(op.map_geometry(f)?),
            Self::VCarve(op) => Operation::VCarve(op),
            Self::ThermalCut(op) => Operation::ThermalCut(op),
        })
    }
}

impl Operation {
    pub fn id(&self) -> &str {
        match self {
            Self::Roughing(p) => &p.id,
            Self::Pocket(p) => &p.id,
            Self::AdaptivePocket(p) => &p.pocket.id,
            Self::TrochoidalSlot(p) => &p.id,
            Self::Facing(p) => &p.id,
            Self::Contour(p) => &p.id,
            Self::Drilling(p) => &p.id,
            Self::Bore(p) => &p.id,
            Self::Chamfer(p) => &p.id,
            Self::Deburr(p) => &p.id,
            Self::Parallel(p) => &p.id,
            Self::Waterline(p) => &p.id,
            Self::Trace(p) => &p.id,
            Self::ThreadMill(p) => &p.id,
            Self::Tapping(p) => &p.id,
            Self::Reaming(p) => &p.id,
            Self::Flat(p) => &p.id,
            Self::VCarve(p) => &p.id,
            Self::ThermalCut(p) => &p.id,
        }
    }
    pub fn tool(&self) -> &str {
        match self {
            Self::Roughing(p) => &p.tool,
            Self::Pocket(p) => &p.tool,
            Self::AdaptivePocket(p) => &p.pocket.tool,
            Self::TrochoidalSlot(p) => &p.tool,
            Self::Facing(p) => &p.tool,
            Self::Contour(p) => &p.tool,
            Self::Drilling(p) => &p.tool,
            Self::Bore(p) => &p.tool,
            Self::Chamfer(p) => &p.tool,
            Self::Deburr(p) => &p.tool,
            Self::Parallel(p) => &p.tool,
            Self::Waterline(p) => &p.tool,
            Self::Trace(p) => &p.tool,
            Self::ThreadMill(p) => &p.tool,
            Self::Tapping(p) => &p.tool,
            Self::Reaming(p) => &p.tool,
            Self::Flat(p) => &p.tool,
            Self::VCarve(p) => &p.tool,
            Self::ThermalCut(p) => &p.tool,
        }
    }
    /// The operation's machining limits, if any.
    pub fn limits(&self) -> Option<&crate::limits::Limits> {
        match self {
            Self::Roughing(p) => p.limits.as_ref(),
            Self::Pocket(p) => p.limits.as_ref(),
            Self::AdaptivePocket(p) => p.pocket.limits.as_ref(),
            Self::TrochoidalSlot(p) => p.limits.as_ref(),
            Self::Facing(p) => p.limits.as_ref(),
            Self::Contour(p) => p.limits.as_ref(),
            Self::Drilling(p) => p.limits.as_ref(),
            Self::Bore(p) => p.limits.as_ref(),
            Self::Chamfer(p) => p.limits.as_ref(),
            Self::Deburr(p) => p.limits.as_ref(),
            Self::Parallel(p) => p.limits.as_ref(),
            Self::Waterline(p) => p.limits.as_ref(),
            Self::Trace(p) => p.limits.as_ref(),
            Self::ThreadMill(p) => p.limits.as_ref(),
            Self::Tapping(p) => p.limits.as_ref(),
            Self::Reaming(p) => p.limits.as_ref(),
            Self::Flat(p) => p.limits.as_ref(),
            Self::VCarve(p) => p.limits.as_ref(),
            Self::ThermalCut(p) => p.limits.as_ref(),
        }
        .filter(|l| !l.is_empty())
    }
    pub fn work_offset(&self) -> u8 {
        match self {
            Self::Roughing(p) => p.work_offset,
            Self::Pocket(p) => p.work_offset,
            Self::AdaptivePocket(p) => p.pocket.work_offset,
            Self::TrochoidalSlot(p) => p.work_offset,
            Self::Facing(p) => p.work_offset,
            Self::Contour(p) => p.work_offset,
            Self::Drilling(p) => p.work_offset,
            Self::Bore(p) => p.work_offset,
            Self::Chamfer(p) => p.work_offset,
            Self::Deburr(p) => p.work_offset,
            Self::Parallel(p) => p.work_offset,
            Self::Waterline(p) => p.work_offset,
            Self::Trace(p) => p.work_offset,
            Self::ThreadMill(p) => p.work_offset,
            Self::Tapping(p) => p.work_offset,
            Self::Reaming(p) => p.work_offset,
            Self::Flat(p) => p.work_offset,
            Self::VCarve(p) => p.work_offset,
            Self::ThermalCut(p) => p.work_offset,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PocketOperation {
    pub id: String,
    pub tool: String,
    pub region: PlanarRegion,
    pub top_z_mm: f64,
    pub floor_z_mm: f64,
    pub stepdown_mm: f64,
    pub stepover_mm: f64,
    pub feed_mm_min: f64,
    pub plunge_mm_min: f64,
    pub clearance_z_mm: f64,
    pub radial_allowance_mm: f64,
    pub bottom_allowance_mm: f64,
    pub tolerance_mm: f64,
    pub spindle_rpm: f64,
    pub work_offset: u8,
    pub tool_number: u32,
    pub length_register: u32,
    pub coolant: Coolant,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rest_tolerance_mm: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub boundary_clockwise: Option<bool>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub entry: Option<PocketEntry>,
    /// Height for travel between passes (absolute Z). Default: clearance.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub retract_z_mm: Option<f64>,
    /// Height where rapid approach changes to plunge feed, and where leaving
    /// the cut changes from feed to rapid (absolute Z, above the stock top).
    /// Default: the retract height.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub feed_z_mm: Option<f64>,
    /// Longest link kept at cutting depth between separate regions (beyond
    /// ordinary stepovers; always proven through cleared stock). None: stay
    /// down when that is faster than retracting to the feed height and
    /// plunging again; 0: always retract.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_stay_down_mm: Option<f64>,
    /// Containment and avoid areas (clamps, fixtures, features to leave alone).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub limits: Option<crate::limits::Limits>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ResidualStatus {
    Pending,
    Estimated,
    Unknown,
}

impl PocketOperation {
    pub fn retract_z(&self) -> f64 {
        self.retract_z_mm.unwrap_or(self.clearance_z_mm)
    }
    pub fn feed_z(&self) -> f64 {
        self.feed_z_mm.unwrap_or_else(|| self.retract_z())
    }
    /// Authoring/setup checks independent of whether material remains to cut.
    /// Physical depth/reach and region construction are checked by the planner.
    pub(crate) fn validate_authoring(
        &self,
        assembly: &ToolAssembly,
        cutter: &crate::tool::Cutter,
        initial: Vec3,
        radius: f64,
    ) -> Result<(), String> {
        if !valid_id(&self.id)
            || self.tool != assembly.id
            || !matches!(cutter.shape, CutterShape::Flat)
            || !cutter.center_cutting
        {
            return Err("baseline pocket requires stable operation/tool IDs and a flat center-cutting end mill".into());
        }
        if self
            .max_stay_down_mm
            .is_some_and(|m| !(m.is_finite() && (0. ..=1e6).contains(&m)))
        {
            return Err(format!(
                "invalid pocket parameters: max_stay_down_mm {:?} must be a non-negative length",
                self.max_stay_down_mm
            ));
        }
        let values = [
            self.top_z_mm,
            self.floor_z_mm,
            self.stepdown_mm,
            self.stepover_mm,
            self.feed_mm_min,
            self.plunge_mm_min,
            self.clearance_z_mm,
            self.radial_allowance_mm,
            self.bottom_allowance_mm,
            self.tolerance_mm,
            self.spindle_rpm,
        ];
        let guard = 16. * self.tolerance_mm;
        // First failing rule, named precisely, so authors can fix their input.
        let checks: [(bool, String); 20] = [
            (
                values.iter().all(|v| v.is_finite()),
                "all numeric parameters must be finite".into(),
            ),
            (
                self.rest_tolerance_mm
                    .is_none_or(|v| v.is_finite() && (0.0..=0.1).contains(&v)),
                format!(
                    "rest_tolerance_mm {:?} must be in [0, 0.1]",
                    self.rest_tolerance_mm
                ),
            ),
            (
                (1e-7..=0.01).contains(&self.tolerance_mm),
                format!("tolerance_mm {} must be in [1e-7, 0.01]", self.tolerance_mm),
            ),
            (
                self.stepdown_mm >= guard,
                format!("stepdown_mm {} must be positive", self.stepdown_mm),
            ),
            (
                self.stepover_mm >= guard,
                format!("stepover_mm {} must be positive", self.stepover_mm),
            ),
            (
                self.stepover_mm <= radius,
                format!(
                    "stepover_mm {} must not exceed the tool radius {radius}",
                    self.stepover_mm
                ),
            ),
            (
                self.feed_mm_min > 0.,
                format!("feed_mm_min {} must be positive", self.feed_mm_min),
            ),
            (
                self.plunge_mm_min > 0.,
                format!("plunge_mm_min {} must be positive", self.plunge_mm_min),
            ),
            (
                self.spindle_rpm > 0.,
                format!("spindle_rpm {} must be positive", self.spindle_rpm),
            ),
            (
                self.radial_allowance_mm >= 0. && self.bottom_allowance_mm >= 0.,
                "allowances must not be negative".into(),
            ),
            (
                self.clearance_z_mm > self.top_z_mm + guard && self.clearance_z_mm.abs() <= 1e6,
                format!(
                    "clearance_z_mm {} must be above top_z_mm {}",
                    self.clearance_z_mm, self.top_z_mm
                ),
            ),
            (
                self.retract_z_mm.is_none_or(f64::is_finite)
                    && self.feed_z_mm.is_none_or(f64::is_finite),
                "retract_z_mm/feed_z_mm must be finite".into(),
            ),
            (
                self.feed_z() > self.top_z_mm + guard,
                format!(
                    "feed_z_mm {} must be above top_z_mm {}",
                    self.feed_z(),
                    self.top_z_mm
                ),
            ),
            (
                self.feed_z() <= self.retract_z(),
                format!(
                    "feed_z_mm {} must not be above retract_z_mm {}",
                    self.feed_z(),
                    self.retract_z()
                ),
            ),
            (
                self.retract_z() <= self.clearance_z_mm,
                format!(
                    "retract_z_mm {} must not be above clearance_z_mm {}",
                    self.retract_z(),
                    self.clearance_z_mm
                ),
            ),
            (
                initial.iter().all(|v| v.is_finite() && v.abs() <= 1e6),
                "start position must be finite".into(),
            ),
            (
                initial[2] >= self.clearance_z_mm,
                format!(
                    "start Z {} is below clearance_z_mm {}",
                    initial[2], self.clearance_z_mm
                ),
            ),
            (
                (1..=9).contains(&self.work_offset),
                format!("work_offset {} must be 1..9", self.work_offset),
            ),
            (
                self.tool_number != 0,
                "tool_number must be at least 1".into(),
            ),
            (
                self.length_register != 0,
                "length_register must be at least 1".into(),
            ),
        ];
        if let Some((_, why)) = checks.into_iter().find(|(ok, _)| !ok) {
            return Err(format!("invalid pocket parameters: {why}"));
        }
        if let Some(entry) = &self.entry {
            entry.helix().validate(
                cutter.diameter_mm / 2.,
                self.tolerance_mm,
                self.plunge_mm_min,
            )?;
        }
        Ok(())
    }
}

#[derive(Debug, Serialize)]
pub struct PocketReport {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stock_top: Option<StockTopReport>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub empty_cuts: Option<crate::empty_cut::EmptyCutReport>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub offset: Option<OffsetReport>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub routing: Option<RoutingReport>,
    pub stock_edge_entries: usize,
    pub helical_entries: usize,
    pub entry_turns: usize,
    pub entry_navigation_moves: usize,
    #[serde(skip_serializing_if = "air_entry::none")]
    pub cleared_entry_descents: usize,
    pub entry_geometry_work: usize,
    pub entry_radii_mm: Vec<f64>,
    pub max_entry_turns_per_descent: usize,
    pub depth_passes: usize,
    pub target_z_mm: f64,
    pub xy_strokes: usize,
    pub raster_strokes: usize,
    pub boundary_strokes: usize,
    pub center_components: usize,
    pub applied_clearance_mm: f64,
    pub geometric_residual_status: ResidualStatus,
    pub geometric_residual_area_mm2: Option<f64>,
    pub geometric_residual: Option<PlanarRegion>,
    pub diagnostics: Vec<String>,
    pub rest_tolerance_mm: Option<f64>,
    pub rest_candidates: u64,
    pub rest_skipped_strokes: u64,
    pub rest_query_work: usize,
    pub rest_query_work_exhausted: bool,
    pub rest_from_generation: Option<u64>,
    pub rest_through_generation: Option<u64>,
    pub rest_merged_pieces: u64,
    pub rest_long_links: u64,
}

#[derive(Clone, Debug, Serialize)]
pub struct StockTopReport {
    pub authored_top_z_mm: f64,
    pub maximum_upper_z_mm: f64,
    pub checked_cells: usize,
    pub generation: u64,
}

/// A lower authored top is valid when preceding operations actually removed the
/// material. The original stock grid remains unchanged; this is a planning bound.
pub(crate) struct StockTop {
    ceiling: f64,
    identity: u64,
    pub(crate) report: StockTopReport,
}
impl StockTop {
    pub(crate) fn prepare(
        stock: &Stock,
        top: f64,
        tolerance: f64,
        cancel: &impl CancelSignal,
    ) -> Result<(StockSpec, Option<Self>), String> {
        let mut spec = stock.spec().clone();
        if top >= spec.max_mm[2] || !top.is_finite() {
            return Ok((spec, None));
        }
        if !tolerance.is_finite() || !(1e-7..=0.01).contains(&tolerance) {
            return Err("pocket tolerance_mm must be finite in [1e-7, 0.01]".into());
        }
        let ceiling = top + 16. * tolerance;
        let maximum = Self::scan(stock, top, ceiling, cancel)?;
        spec.max_mm[2] = top;
        stock.request_sweep_receipts();
        Ok((
            spec,
            Some(Self {
                ceiling,
                identity: stock.identity(),
                report: StockTopReport {
                    authored_top_z_mm: top,
                    maximum_upper_z_mm: maximum,
                    checked_cells: stock.cells().len(),
                    generation: stock.generation(),
                },
            }),
        ))
    }
    fn scan(
        stock: &Stock,
        top: f64,
        ceiling: f64,
        cancel: &impl CancelSignal,
    ) -> Result<f64, String> {
        let mut maximum = f64::NEG_INFINITY;
        for (i, cell) in stock.cells().iter().enumerate() {
            if i.is_multiple_of(1024) && cancel.is_cancelled() {
                return Err("pocket stock-top inspection cancelled".into());
            }
            if cell.upper_mm > ceiling {
                return Err(format!(
                    "pocket top_z_mm {top} is below remaining stock at cell {i} (upper {} mm); face or clear that stock first, or raise the operation top",
                    cell.upper_mm
                ));
            }
            maximum = maximum.max(cell.upper_mm);
        }
        if cancel.is_cancelled() {
            return Err("pocket stock-top inspection cancelled".into());
        }
        Ok(maximum)
    }
    pub(crate) fn validate(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        if stock.identity() != self.identity {
            // Native monotone commits preserve this height bound. Foreign,
            // restored or undone state is assessed afresh instead of trusted.
            if !stock
                .last_sweep_receipt()
                .is_some_and(|r| r.base_identity == self.identity)
            {
                Self::scan(stock, self.report.authored_top_z_mm, self.ceiling, cancel)?;
            }
            self.identity = stock.identity();
        }
        stock.request_sweep_receipts();
        Ok(())
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct Stroke {
    pub(crate) a: Point,
    pub(crate) b: Point,
    pub(crate) arc: Option<(Point, f64)>,
}
impl Stroke {
    pub(crate) fn line(a: Point, b: Point) -> Self {
        Self { a, b, arc: None }
    }
    pub(crate) fn curve(self, z: f64) -> Curve {
        let start = [self.a[0], self.a[1], z];
        let end = [self.b[0], self.b[1], z];
        match self.arc {
            None => Curve::Line { start, end },
            Some((c, sweep_rad)) => Curve::Arc {
                start,
                end,
                center: [c[0], c[1], z],
                plane: Plane::Xy,
                sweep_rad,
            },
        }
    }
    pub(crate) fn from_boundary(b: &Boundary) -> Vec<Self> {
        (0..b.vertices.len())
            .map(|i| {
                let a = b.vertices[i];
                let end = b.vertices[(i + 1) % b.vertices.len()];
                let mut s = Self {
                    a: [a[0], a[1]],
                    b: [end[0], end[1]],
                    arc: None,
                };
                if a[2] != 0. {
                    let factor = (1. - a[2] * a[2]) / (4. * a[2]);
                    s.arc = Some((
                        [
                            (a[0] + end[0]) / 2. - factor * (end[1] - a[1]),
                            (a[1] + end[1]) / 2. + factor * (end[0] - a[0]),
                        ],
                        4. * a[2].atan(),
                    ));
                }
                s
            })
            .collect()
    }
    pub(crate) fn reversed(self) -> Self {
        Self {
            a: self.b,
            b: self.a,
            arc: self.arc.map(|(c, s)| (c, -s)),
        }
    }
    pub(crate) fn tangent(self, end: bool) -> Point {
        let p = if end { self.b } else { self.a };
        let v = if let Some((c, sweep)) = self.arc {
            let sign = sweep.signum();
            [-(p[1] - c[1]) * sign, (p[0] - c[0]) * sign]
        } else {
            [self.b[0] - self.a[0], self.b[1] - self.a[1]]
        };
        let n = v[0].hypot(v[1]);
        [v[0] / n, v[1] / n]
    }
}

/// A conservative broad phase only. Arc envelopes include the complete circle
/// and both authored endpoints; exact source distances still decide nearby fits.
#[derive(Clone, Copy)]
struct StrokeBounds {
    min: Point,
    max: Point,
}
impl StrokeBounds {
    fn new(s: Stroke) -> Self {
        let mut min = [s.a[0].min(s.b[0]), s.a[1].min(s.b[1])];
        let mut max = [s.a[0].max(s.b[0]), s.a[1].max(s.b[1])];
        if let Some((center, _)) = s.arc {
            let radius = norm(s.a, center);
            for k in 0..2 {
                min[k] = min[k].min(center[k] - radius);
                max[k] = max[k].max(center[k] + radius);
            }
        }
        let scale = min.into_iter().chain(max).map(f64::abs).fold(1., f64::max);
        let guard = 4096. * f64::EPSILON * scale;
        Self {
            min: min.map(|x| x - guard),
            max: max.map(|x| x + guard),
        }
    }
    /// L-infinity separation is a lower bound on Euclidean separation. The
    /// expanded boxes and downward rounding keep this strictly a rejection aid
    /// for distant boundaries, not an approximation of the cutter envelope.
    fn separation(self, other: Self) -> f64 {
        (0..2)
            .map(|k| (self.min[k] - other.max[k]).max(other.min[k] - self.max[k]))
            .fold(0., f64::max)
            .next_down()
            .max(0.)
    }
}

/// Remove only exactly collinear forward line subdivisions after validating
/// the original source. No chord tolerance, arc lowering or source mutation.
fn simplify_boundary(b: &mut Boundary) {
    let flat = |a: [f64; 3], b: [f64; 3], c: [f64; 3]| {
        a[2] == 0.
            && b[2] == 0.
            && orient2d(
                Coord { x: a[0], y: a[1] },
                Coord { x: b[0], y: b[1] },
                Coord { x: c[0], y: c[1] },
            ) == 0.
            && (b[0] - a[0]) * (c[0] - b[0]) + (b[1] - a[1]) * (c[1] - b[1]) >= 0.
    };
    let mut vertices = Vec::with_capacity(b.vertices.len());
    for &v in &b.vertices {
        while vertices.len() >= 2
            && flat(
                vertices[vertices.len() - 2],
                vertices[vertices.len() - 1],
                v,
            )
        {
            vertices.pop();
        }
        vertices.push(v);
    }
    loop {
        let n = vertices.len();
        if n <= 2 {
            break;
        }
        if flat(vertices[n - 1], vertices[0], vertices[1]) {
            vertices.remove(0);
        } else if flat(vertices[n - 2], vertices[n - 1], vertices[0]) {
            vertices.pop();
        } else {
            break;
        }
    }
    b.vertices = vertices;
}

fn norm(a: Point, b: Point) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}
fn point_segment(p: Point, a: Point, b: Point) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1]];
    let t = if a == b {
        0.
    } else {
        ((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / (d[0] * d[0] + d[1] * d[1])
    }
    .clamp(0., 1.);
    norm(p, [a[0] + t * d[0], a[1] + t * d[1]])
}
fn on_arc(p: Point, s: Stroke) -> bool {
    let (c, sweep) = s.arc.unwrap();
    let a = (s.a[1] - c[1]).atan2(s.a[0] - c[0]);
    let b = (p[1] - c[1]).atan2(p[0] - c[0]);
    let travel = if sweep > 0. {
        (b - a).rem_euclid(TAU)
    } else {
        (a - b).rem_euclid(TAU)
    };
    travel <= sweep.abs() + 1e-12 || TAU - travel <= 1e-12
}
fn point_arc(p: Point, s: Stroke) -> f64 {
    let (c, _) = s.arc.unwrap();
    let d = norm(p, c);
    let r = norm(s.a, c);
    let mut result = norm(p, s.a).min(norm(p, s.b));
    if d > 0. && on_arc(p, s) {
        result = result.min((d - r).abs());
    }
    result
}
pub(crate) fn stroke_distance(a: Stroke, b: Stroke) -> f64 {
    if a.arc.is_none() {
        return segment_distance(a.a, a.b, b);
    }
    if b.arc.is_none() {
        return segment_distance(b.a, b.b, a);
    }
    let (ca, _) = a.arc.unwrap();
    let (cb, _) = b.arc.unwrap();
    let ra = norm(a.a, ca);
    let rb = norm(b.a, cb);
    let d = norm(ca, cb);
    let mut best = point_arc(a.a, b)
        .min(point_arc(a.b, b))
        .min(point_arc(b.a, a))
        .min(point_arc(b.b, a));
    if d == 0. {
        return best;
    }
    let u = [(cb[0] - ca[0]) / d, (cb[1] - ca[1]) / d];
    // Nonintersecting interior extrema lie on the line through both centers.
    for sa in [-1., 1.] {
        for sb in [-1., 1.] {
            let p = [ca[0] + sa * ra * u[0], ca[1] + sa * ra * u[1]];
            let q = [cb[0] + sb * rb * u[0], cb[1] + sb * rb * u[1]];
            if on_arc(p, a) && on_arc(q, b) {
                best = best.min(norm(p, q));
            }
        }
    }
    if d >= (ra - rb).abs() && d <= ra + rb {
        let along = ((ra - rb) * (ra + rb) + d * d) / (2. * d);
        let high = ((ra - along) * (ra + along)).max(0.).sqrt();
        for sign in [-1., 1.] {
            let p = [
                ca[0] + along * u[0] - sign * high * u[1],
                ca[1] + along * u[1] + sign * high * u[0],
            ];
            if on_arc(p, a) && on_arc(p, b) {
                return 0.;
            }
        }
    }
    best
}
fn stroke_point(s: Stroke, fraction: f64) -> Point {
    if let Some((c, sweep)) = s.arc {
        let angle = (s.a[1] - c[1]).atan2(s.a[0] - c[0]) + fraction * sweep;
        let r = norm(s.a, c);
        [c[0] + r * angle.cos(), c[1] + r * angle.sin()]
    } else {
        [
            s.a[0] + fraction * (s.b[0] - s.a[0]),
            s.a[1] + fraction * (s.b[1] - s.a[1]),
        ]
    }
}
fn farthest_point(s: Stroke, p: Point) -> f64 {
    let mut maximum = norm(s.a, p).max(norm(s.b, p));
    if let Some((c, _)) = s.arc {
        let d = norm(c, p);
        if d == 0. {
            return norm(s.a, c);
        }
        let r = norm(s.a, c);
        let opposite = [c[0] + r * (c[0] - p[0]) / d, c[1] + r * (c[1] - p[1]) / d];
        if on_arc(opposite, s) {
            maximum = maximum.max(d + r);
        }
    }
    maximum
}
fn maximum_boundary_distance(s: Stroke, edge: Stroke) -> f64 {
    if let Some((center, boundary_sweep)) = edge.arc {
        // Actual boundary points are valid witnesses for every guide point.
        let mut upper = farthest_point(s, edge.a).min(farthest_point(s, edge.b));
        let middle = stroke_point(s, 0.5);
        let distance = norm(middle, center);
        if distance > 0. && on_arc(middle, edge) {
            let radius = norm(edge.a, center);
            let witness = [
                center[0] + radius * (middle[0] - center[0]) / distance,
                center[1] + radius * (middle[1] - center[1]) / distance,
            ];
            // This bounds the entire guide and converges under subdivision
            // even when guide and boundary arcs are not concentric.
            upper = upper.min(farthest_point(s, witness));
        }
        if let Some((guide_center, sweep)) = s.arc
            && guide_center == center
            && sweep.abs() <= PI
            && boundary_sweep.abs() <= PI
            && on_arc(s.a, edge)
            && on_arc(s.b, edge)
            && on_arc(middle, edge)
        {
            upper = upper.min((norm(s.a, center) - norm(edge.a, center)).abs());
        }
        upper
    } else {
        // Distance to a segment is convex. The chord's endpoint maximum,
        // plus the whole arc's sagitta, bounds every point of this guide.
        let sagitta = s.arc.map_or(0., |(c, sweep)| {
            2. * norm(s.a, c) * (sweep / 4.).sin().powi(2)
        });
        point_segment(s.a, edge.a, edge.b).max(point_segment(s.b, edge.a, edge.b)) + sagitta
    }
}
/// Continuous minimum distance between a straight link and a boundary segment.
/// All endpoint, intersection and interior stationary candidates are considered.
/// A separate numerical guard and sign query make this a conservative engineering
/// fit test, not interval-certified physical collision verification.
fn segment_distance(a: Point, b: Point, s: Stroke) -> f64 {
    let mut result = point_segment(s.a, a, b).min(point_segment(s.b, a, b));
    let d = [b[0] - a[0], b[1] - a[1]];
    let length = norm(a, b);
    if let Some((c, _)) = s.arc {
        result = result.min(point_arc(a, s)).min(point_arc(b, s));
        if length == 0. {
            return result;
        }
        let unit = [d[0] / length, d[1] / length];
        let along = (c[0] - a[0]) * unit[0] + (c[1] - a[1]) * unit[1];
        let nearest = [a[0] + along * unit[0], a[1] + along * unit[1]];
        let separation = norm(nearest, c);
        let radius = norm(s.a, c);
        if separation <= radius {
            let offset = ((radius - separation) * (radius + separation)).sqrt();
            for t in [along - offset, along + offset] {
                let p = [a[0] + t * unit[0], a[1] + t * unit[1]];
                if (0.0..=length).contains(&t) && on_arc(p, s) {
                    return 0.;
                }
            }
        }
        for sign in [-1., 1.] {
            let p = [
                c[0] - sign * radius * unit[1],
                c[1] + sign * radius * unit[0],
            ];
            if on_arc(p, s) {
                result = result.min(point_segment(p, a, b));
            }
        }
    } else {
        result = result
            .min(point_segment(a, s.a, s.b))
            .min(point_segment(b, s.a, s.b));
        let e = [s.b[0] - s.a[0], s.b[1] - s.a[1]];
        let det = d[0] * e[1] - d[1] * e[0];
        if det != 0. {
            let delta = [s.a[0] - a[0], s.a[1] - a[1]];
            let t = (delta[0] * e[1] - delta[1] * e[0]) / det;
            let u = (delta[0] * d[1] - delta[1] * d[0]) / det;
            if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u) {
                return 0.;
            }
        }
    }
    result
}

/// A Y-monotone line/circle piece with half-open scan crossings. Tangent double
/// roots are kept (zero-width paired intervals); deduplicating them breaks parity.
struct Crossing {
    a: Point,
    b: Point,
    circle: Option<(Point, f64, f64)>,
}
impl Crossing {
    fn split(s: Stroke) -> Vec<Self> {
        let Some((center, sweep)) = s.arc else {
            return vec![Self {
                a: s.a,
                b: s.b,
                circle: None,
            }];
        };
        let radius = (s.a[0] - center[0]).hypot(s.a[1] - center[1]);
        let angle = (s.a[1] - center[1]).atan2(s.a[0] - center[0]);
        let mut cuts = vec![(0., s.a), (1., s.b)];
        for k in -4..=4 {
            let target = FRAC_PI_2 + k as f64 * PI;
            let f = (target - angle) / sweep;
            if f > 0. && f < 1. {
                cuts.push((
                    f,
                    [
                        center[0],
                        center[1] + if k % 2 == 0 { radius } else { -radius },
                    ],
                ));
            }
        }
        cuts.sort_by(|a, b| a.0.total_cmp(&b.0));
        cuts.windows(2)
            .map(|p| {
                let midpoint = angle + (p[0].0 + p[1].0) * sweep / 2.;
                Self {
                    a: p[0].1,
                    b: p[1].1,
                    circle: Some((center, radius, midpoint.cos().signum())),
                }
            })
            .collect()
    }
    fn x_at(&self, y: f64) -> Option<f64> {
        if y < self.a[1].min(self.b[1]) || y >= self.a[1].max(self.b[1]) {
            return None;
        }
        Some(match self.circle {
            None => self.a[0] + (y - self.a[1]) / (self.b[1] - self.a[1]) * (self.b[0] - self.a[0]),
            Some((c, r, sign)) => {
                let d = (y - c[1]).abs().min(r);
                c[0] + sign * ((r - d) * (r + d)).sqrt()
            }
        })
    }
}

/// An actually successful whole-curve source check, scoped to the immutable
/// source geometry/error/allowance of one prepared pocket. Retaining the Arc
/// allocation prevents address reuse from impersonating a former owner.
pub(crate) struct GuideTubeApproval {
    owner: Arc<()>,
    curve: Curve,
    radius: f64,
    tolerance: f64,
}
impl GuideTubeApproval {
    pub(crate) fn matches(
        &self,
        source: &PocketPlan,
        curve: &Curve,
        radius: f64,
        tolerance: f64,
    ) -> bool {
        Arc::ptr_eq(&self.owner, &source.source_owner)
            && &self.curve == curve
            && self.radius == radius
            && self.tolerance == tolerance
            && source.operation.tolerance_mm == tolerance
    }
}
/// Committed-clearance scene shared by the plans that stream into one stock
/// (for example every layer of a roughing operation). It validates itself
/// against the stock's column history at each use, so sharing is always safe.
pub(crate) type EntrySceneCache = Arc<
    std::sync::Mutex<
        Option<(
            crate::engagement::EngagementScene,
            crate::stock::ClearanceCursor,
        )>,
    >,
>;

pub struct PocketPlan {
    operation: PocketOperation,
    source: PreparedRegion,
    source_owner: Arc<()>,
    centers: PreparedRegion,
    radius: f64,
    source_edges: Vec<Stroke>,
    source_bounds: Vec<StrokeBounds>,
    source_magnitude: f64,
    strokes: Vec<Stroke>,
    rest_group_ends: Vec<usize>,
    report: PocketReport,
    initial: Vec3,
    position: Vec3,
    pass: usize,
    stroke: usize,
    next_id: u64,
    pending: VecDeque<PendingMove>,
    entries: Option<PocketEntries>,
    finished: bool,
    stock_spec: StockSpec,
    expected_stock_generation: Option<u64>,
    rest_tool: Option<ToolAssembly>,
    air_entry_parts: Vec<(f64, f64)>,
    /// Committed-clearance scene for air entries, extended incrementally from
    /// the stock's column history (see `ClearanceCursor`).
    entry_scene: EntrySceneCache,
    rest_stock_identity: Option<u64>,
    rest_pending: Option<Motion>,
    /// End of the previous entry helix turn; a helix starting there continues
    /// the same descent rather than beginning a new helical entry.
    entry_helix_end: Option<crate::motion::Vec3>,
    stock_top: Option<StockTop>,
}

impl PocketPlan {
    pub fn prepare_with_stock(
        op: PocketOperation,
        assembly: &ToolAssembly,
        stock: &Stock,
        initial: Vec3,
        first_motion_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        let (envelope, bound) = StockTop::prepare(stock, op.top_z_mm, op.tolerance_mm, cancel)?;
        let mut plan = Self::prepare(op, assembly, &envelope, initial, first_motion_id, cancel)?;
        if let Some(bound) = bound {
            plan.activate_layer(initial, first_motion_id, stock.spec())?;
            plan.report.stock_top = Some(bound.report.clone());
            plan.stock_top = Some(bound);
            plan.expected_stock_generation = Some(stock.generation());
        }
        Ok(plan)
    }
    pub(crate) fn enable_empty_cuts(&mut self) {
        self.report.empty_cuts = Some(Default::default());
        self.report.diagnostics.push("Nominal guide coverage includes omitted guides with whole-sweep empty-stock proofs; actual remaining material is checked separately".into());
    }
    pub fn prepare(
        op: PocketOperation,
        assembly: &ToolAssembly,
        stock: &StockSpec,
        initial: Vec3,
        first_motion_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        Self::prepare_guides(op, assembly, stock, initial, first_motion_id, None, cancel)
    }
    pub(crate) fn prepare_guides(
        op: PocketOperation,
        assembly: &ToolAssembly,
        stock: &StockSpec,
        initial: Vec3,
        first_motion_id: u64,
        guide_radius: Option<f64>,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        Self::prepare_core(
            op,
            assembly,
            stock,
            initial,
            first_motion_id,
            GuideMode::Raster(guide_radius),
            cancel,
        )
    }
    pub(crate) fn prepare_offset(
        op: PocketOperation,
        assembly: &ToolAssembly,
        stock: &StockSpec,
        initial: Vec3,
        first_motion_id: u64,
        coverage_margin_mm: f64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        Self::prepare_core(
            op,
            assembly,
            stock,
            initial,
            first_motion_id,
            GuideMode::Offset { coverage_margin_mm },
            cancel,
        )
    }
    fn prepare_core(
        op: PocketOperation,
        assembly: &ToolAssembly,
        stock: &StockSpec,
        initial: Vec3,
        first_motion_id: u64,
        mode: GuideMode,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        if cancel.is_cancelled() {
            return Err("pocket planning cancelled".into());
        }
        // Limits restrict the region itself: the planner then routes around
        // avoid areas and stays inside the containment.
        let mut op = op;
        if let Some(limits) = op.limits.take().filter(|l| !l.is_empty()) {
            op.region = limits.restrict_region(&op.region, op.floor_z_mm, op.tolerance_mm)?;
        }
        stock.grid()?;
        let prepared_tool = assembly.prepare()?;
        let cutter = prepared_tool.cutter().definition();
        let actual_radius = cutter.diameter_mm / 2.;
        let guide_radius = match mode {
            GuideMode::Raster(radius) => radius,
            GuideMode::Offset { .. } => None,
        };
        let radius = guide_radius.unwrap_or(actual_radius);
        if !radius.is_finite() || radius < actual_radius || radius > 2. * actual_radius {
            return Err("invalid adaptive guide radius".into());
        }
        op.validate_authoring(assembly, cutter, initial, radius)?;
        let target = op.floor_z_mm + op.bottom_allowance_mm;
        let reach: [(bool, String); 5] = [
            (
                op.stepdown_mm <= cutter.cutting_length_mm,
                format!(
                    "stepdown_mm {} exceeds the cutting length {}",
                    op.stepdown_mm, cutter.cutting_length_mm
                ),
            ),
            (
                op.floor_z_mm >= stock.min_mm[2],
                format!(
                    "floor_z_mm {} is below the stock bottom {}",
                    op.floor_z_mm, stock.min_mm[2]
                ),
            ),
            (
                target < op.top_z_mm,
                format!(
                    "floor_z_mm {} plus bottom allowance must be below top_z_mm {}",
                    op.floor_z_mm, op.top_z_mm
                ),
            ),
            (
                op.top_z_mm == stock.max_mm[2],
                format!(
                    "top_z_mm {} must equal the stock top {}",
                    op.top_z_mm, stock.max_mm[2]
                ),
            ),
            (
                op.top_z_mm - target <= cutter.cutting_length_mm,
                format!(
                    "pocket depth {} exceeds the cutting length {}",
                    op.top_z_mm - target,
                    cutter.cutting_length_mm
                ),
            ),
        ];
        if let Some((_, why)) = reach.into_iter().find(|(ok, _)| !ok) {
            return Err(format!("invalid pocket parameters: {why}"));
        }
        let passes = ((op.top_z_mm - target) / op.stepdown_mm).ceil() as usize;
        if passes == 0 {
            return Err("pocket exceeds bounded depth passes".into());
        }
        let options = ConstructionOptions {
            tolerance_mm: op.tolerance_mm,
        };
        let original = op.region.prepare(options)?;
        let original_segments = original.summary().segments;
        let mut simplified = original.definition().clone();
        for c in &mut simplified.components {
            simplify_boundary(&mut c.outer);
            for hole in &mut c.holes {
                simplify_boundary(hole);
            }
        }
        let source = simplified.prepare(options)?;
        let simplified_segments = source.summary().segments;
        let construction =
            source.cutter_region(radius, op.radial_allowance_mm + 16. * op.tolerance_mm)?;
        let center = construction.region.prepare(options)?;
        if cancel.is_cancelled() {
            return Err("pocket planning cancelled".into());
        }
        let mut boundaries = Vec::new();
        for c in &center.definition().components {
            for b in std::iter::once(&c.outer).chain(&c.holes) {
                let mut segments = Stroke::from_boundary(b);
                // Prepared filled regions normalize outer loops CCW and holes
                // CW. Reverse both so material-side boundary finishing changes
                // direction while the interior raster remains bidirectional.
                if op.boundary_clockwise == Some(true) {
                    segments = segments.into_iter().rev().map(Stroke::reversed).collect();
                }
                boundaries.extend(segments);
            }
        }
        let (strokes, raster_strokes, offset_report) =
            if let GuideMode::Offset { coverage_margin_mm } = mode {
                let (strokes, report) =
                    offset::generate(&center, boundaries, &op, radius, coverage_margin_mm, cancel)?;
                (strokes, 0, Some(report))
            } else {
                // Circular extrema matter: vertex coordinates alone are not full bounds.
                let (mut ymin, mut ymax) = (f64::INFINITY, f64::NEG_INFINITY);
                let mut crossings = Vec::new();
                for &s in &boundaries {
                    let bounds = s.curve(target).prepare(op.tolerance_mm)?.bounds();
                    ymin = ymin.min(bounds.min[1]);
                    ymax = ymax.max(bounds.max[1]);
                    crossings.extend(Crossing::split(s));
                }
                let rows = if boundaries.is_empty() {
                    0
                } else {
                    ((ymax - ymin) / op.stepover_mm).ceil() as usize
                };
                let mut strokes = Vec::new();
                let mut xs = Vec::with_capacity(crossings.len());
                for row in 0..rows {
                    if cancel.is_cancelled() {
                        return Err("pocket planning cancelled".into());
                    }
                    let y = ymin + (row as f64 + 0.5) * (ymax - ymin) / rows as f64;
                    xs.clear();
                    xs.extend(crossings.iter().filter_map(|c| c.x_at(y)));
                    xs.sort_by(f64::total_cmp);
                    if xs.len() % 2 != 0 {
                        return Err("ambiguous pocket scan intersection parity".into());
                    }
                    let start = strokes.len();
                    for pair in xs.as_chunks::<2>().0 {
                        let width = pair[1] - pair[0];
                        if width <= 8. * op.tolerance_mm {
                            continue;
                        }
                        let trim = (4. * op.tolerance_mm).min(width / 4.);
                        let (a, b) = ([pair[0] + trim, y], [pair[1] - trim, y]);
                        strokes.push(if row % 2 == 0 {
                            Stroke { a, b, arc: None }
                        } else {
                            Stroke {
                                a: b,
                                b: a,
                                arc: None,
                            }
                        });
                    }
                    if row % 2 == 1 {
                        strokes[start..].reverse();
                    }
                    if (row + 1) % 4096 == 0 {
                        // Projected total from rows so far (x2 for Vec growth).
                        let projected = strokes.len() as u128 * rows as u128 / (row as u128 + 1);
                        crate::resources::check_memory(
                            projected * 2 * std::mem::size_of::<Stroke>() as u128,
                            "pocket raster strokes",
                        )?;
                    }
                }
                let raster_strokes = strokes.len();
                strokes.extend(boundaries);
                (strokes, raster_strokes, None)
            };
        let mut diagnostics = construction.diagnostics;
        if let Some(clockwise) = op.boundary_clockwise {
            diagnostics.push(if offset_report.is_some() {
                format!("offset outer loops clockwise={clockwise}, holes opposite; whole guides scheduled through admitted routes before any stock filtering")
            } else {
                format!("boundary finish outer loops clockwise={clockwise}, holes opposite; interior raster remains bidirectional")
            });
        }
        if simplified_segments < original_segments {
            diagnostics.push(format!("collapsed exact collinear subdivisions: {original_segments} -> {simplified_segments} source segments; original manifest/source retained"));
        }
        diagnostics.push("geometric residual includes inaccessible corners/channels and numerical guards; actual remaining stock is a separate sweep/verification result".into());
        diagnostics.push(if op.entry.as_ref().is_some_and(|e|e.policy()!=crate::entry::EntryPolicy::Helix) {
            "entry uses whole-source admitted disks/routes; air descent requires the entire seed disk outside original stock; enclosed helix requires explicit policy; assembly/fixture/machine verification is separate".into()
        } else if op.entry.is_some() {
            "helical entry uses whole-source admitted circle/routes; assembly, fixture and machine verification is separate".into()
        } else { "vertical entry requires a center-cutting flat tool; shank, holder, fixture and machine checks remain unverified".into() });
        if strokes.is_empty() {
            diagnostics
                .push("no cutter-center region remains: operation emits no cutting motion".into());
        }
        let report = PocketReport {
            stock_top: None,
            empty_cuts: None,
            offset: offset_report,
            routing: None,
            stock_edge_entries: 0,
            helical_entries: 0,
            entry_turns: 0,
            entry_navigation_moves: 0,
            cleared_entry_descents: 0,
            entry_geometry_work: 0,
            entry_radii_mm: vec![],
            max_entry_turns_per_descent: 0,
            rest_tolerance_mm: op.rest_tolerance_mm,
            rest_candidates: 0,
            rest_skipped_strokes: 0,
            rest_query_work: 0,
            rest_query_work_exhausted: false,
            rest_from_generation: None,
            rest_through_generation: None,
            rest_merged_pieces: 0,
            rest_long_links: 0,
            depth_passes: passes,
            target_z_mm: target,
            xy_strokes: strokes.len(),
            raster_strokes,
            boundary_strokes: strokes.len() - raster_strokes,
            center_components: construction.after_components,
            applied_clearance_mm: construction.applied_offset_mm.unwrap(),
            geometric_residual_status: ResidualStatus::Pending,
            geometric_residual_area_mm2: None,
            geometric_residual: None,
            diagnostics,
        };
        let source_edges: Vec<_> = source
            .definition()
            .components
            .iter()
            .flat_map(|c| std::iter::once(&c.outer).chain(&c.holes))
            .flat_map(Stroke::from_boundary)
            .collect();
        let source_bounds = source_edges
            .iter()
            .copied()
            .map(StrokeBounds::new)
            .collect();
        let source_magnitude = source_edges
            .iter()
            .flat_map(|e| [e.a[0], e.a[1], e.b[0], e.b[1]])
            .map(f64::abs)
            .fold(1., f64::max);
        let mut result = Self {
            source_bounds,
            source_magnitude,
            rest_tool: (op.rest_tolerance_mm.is_some() || op.entry.is_some())
                .then(|| assembly.clone()),
            entry_scene: EntrySceneCache::default(),
            air_entry_parts: if op.entry.is_some() {
                crate::verification::parts(&prepared_tool)?
                    .iter()
                    .map(|p| (p.radius.at(p.min).max(p.radius.at(p.max)), p.min))
                    .collect()
            } else {
                vec![]
            },
            rest_stock_identity: None,
            rest_pending: None,
            entry_helix_end: None,
            operation: op,
            source,
            source_owner: Arc::new(()),
            centers: center,
            radius,
            source_edges,
            strokes,
            rest_group_ends: vec![],
            report,
            initial,
            position: initial,
            pass: 0,
            stroke: 0,
            next_id: first_motion_id,
            pending: VecDeque::with_capacity(4),
            entries: None,
            finished: false,
            stock_spec: stock.clone(),
            expected_stock_generation: None,
            stock_top: None,
        };
        // Independent proof of the constructed path: every planned cutter-centre
        // stroke must keep radius + allowance (+ source error and guards) from
        // the ORIGINAL boundary. This does not trust the offset construction;
        // a construction defect becomes an error, never a gouge.
        for &s in &result.strokes {
            if cancel.is_cancelled() {
                return Err("pocket planning cancelled".into());
            }
            if !result.guide_tube_fits(s, result.radius) {
                return Err(format!(
                    "refusing toolpath: planned cutter path ({:.4}, {:.4}) -> ({:.4}, {:.4}) does not keep the required {:.4} mm clearance from the original boundary (internal offset construction error)",
                    s.a[0],
                    s.a[1],
                    s.b[0],
                    s.b[1],
                    result.radius + result.operation.radial_allowance_mm
                ));
            }
        }
        if guide_radius.is_none() && result.operation.entry.is_some() && !result.strokes.is_empty()
        {
            let entries = result.prepare_entries(&result.strokes, cancel)?;
            result.report.entry_geometry_work = entries.work;
            result.report.entry_radii_mm = entries.locations.iter().map(|e| e.radius).collect();
            result.report.max_entry_turns_per_descent = entries
                .locations
                .iter()
                .map(|e| e.max_turns)
                .max()
                .unwrap_or(0);
            result.entries = Some(entries);
        }
        if matches!(mode, GuideMode::Offset { .. }) {
            result = result.schedule_guides(RasterRouting::Economical, cancel)?;
        }
        if result.operation.rest_tolerance_mm.is_some() {
            result.subdivide_rest_guides(cancel)?;
        }
        first_motion_id
            .checked_add(1)
            .ok_or("pocket motion IDs overflow")?;
        Ok(result)
    }
    /// Preserve scheduled whole-guide order and sense. Stock queries may omit
    /// pieces, but cannot reorient arcs or reorder pieces across guide groups.
    fn subdivide_rest_guides(&mut self, cancel: &impl CancelSignal) -> Result<(), String> {
        let originals = std::mem::take(&mut self.strokes);
        let mut expanded_raster = 0;
        for (index, s) in originals.iter().copied().enumerate() {
            if cancel.is_cancelled() {
                return Err("rest preparation cancelled".into());
            }
            let path = s.curve(0.).prepare(self.operation.tolerance_mm)?;
            let count = (path.length_mm() / self.radius).ceil().max(1.) as usize;
            let end = self.strokes.len() + count;
            self.rest_group_ends.extend(std::iter::repeat_n(end, count));
            for i in 0..count {
                let a = path.position(i as f64 / count as f64)?;
                let b = path.position((i + 1) as f64 / count as f64)?;
                self.strokes.push(Stroke {
                    a: [a[0], a[1]],
                    b: [b[0], b[1]],
                    arc: s.arc.map(|(c, sweep)| (c, sweep / count as f64)),
                });
            }
            if index < self.report.raster_strokes {
                expanded_raster = self.strokes.len();
            }
        }
        if let Some(entries) = &mut self.entries {
            entries.subdivide(
                &originals,
                &self.strokes,
                &self.rest_group_ends,
                self.operation.tolerance_mm,
                cancel,
            )?;
            self.report.entry_geometry_work = entries.work;
        }
        self.report.xy_strokes = self.strokes.len();
        self.report.raster_strokes = expanded_raster;
        self.report.boundary_strokes = self.strokes.len() - expanded_raster;
        if let Some(offset) = &mut self.report.offset {
            offset.coverage_scope = "whole centre-domain coverage by nominal analytic line/arc guides before rest filtering; original boundary sweep covers exterior radius band; omitted pieces require whole-sweep committed-stock clearance; engineering bounds, not emitted-path coverage";
        }
        self.report.diagnostics.push("Rest keeps scheduled analytic guide order/sense; omitted pieces require whole-sweep committed-stock clearance; uncertain pieces are retained".into());
        Ok(())
    }
    pub(crate) fn operation_tolerance(&self) -> f64 {
        self.operation.tolerance_mm
    }
    pub(crate) fn construction_error_mm(&self) -> f64 {
        let boundary_error = self
            .report
            .offset
            .as_ref()
            .map_or(self.centers.definition().boundary_error_mm, |r| {
                r.construction_error_mm
            });
        (self.report.applied_clearance_mm - self.radius - self.operation.radial_allowance_mm)
            .max(0.)
            + boundary_error
    }
    pub(crate) fn entry_centers(
        &self,
        swept_radius: f64,
        cancel: &impl CancelSignal,
    ) -> Result<Vec<Point>, String> {
        let region = self
            .source
            .cutter_region(
                swept_radius,
                self.operation.radial_allowance_mm + 16. * self.operation.tolerance_mm,
            )?
            .region;
        let mut candidates = Vec::new();
        for component in &region.components {
            for boundary in std::iter::once(&component.outer).chain(&component.holes) {
                for edge in Stroke::from_boundary(boundary) {
                    if cancel.is_cancelled() {
                        return Err("helical entry candidate preparation cancelled".into());
                    }
                    candidates.push(edge.a);
                    candidates.push(stroke_point(edge, 0.5));
                }
            }
        }
        Ok(candidates)
    }
    fn prepare_entries(
        &self,
        strokes: &[Stroke],
        cancel: &impl CancelSignal,
    ) -> Result<PocketEntries, String> {
        let mut work = 0;
        for &s in strokes {
            if cancel.is_cancelled() {
                return Err("helical pocket admission cancelled".into());
            }
            work += self.link_work_per_query();
            if !self.guide_tube_fits(s, self.radius) {
                return Err(
                    "helical pocket navigation stroke lacks original-source clearance".into(),
                );
            }
        }
        let mut graph = crate::navigation::graph(
            self,
            strokes.iter().map(|s| (*s, 0., 0.)).collect(),
            crate::navigation::GraphOptions {
                bridge_radius: 0.,
                cutter_radius: self.radius,
                coverage: 0.,
                local_sizing: false,
                starting_work: work,
            },
            cancel,
        )?;
        crate::entry::compact_entry_tours(&mut graph.tours, &mut graph.work, cancel)?;
        let fit = fit_entries(
            self,
            &graph.tours,
            self.operation.entry.as_ref().unwrap().helix(),
            EntryFitOptions {
                cutter_radius: self.radius,
                minimum_orbit: 0.,
                preferred_cap: self.radius,
                max_depth: self.operation.top_z_mm - self.report.target_z_mm,
                starting_work: graph.work,
                policy: self.operation.entry.as_ref().unwrap().policy(),
            },
            cancel,
        )?;
        PocketEntries::new(graph.tours, fit, self.operation.tolerance_mm)
    }
    fn helical_moves(
        &self,
        target: Point,
        z: f64,
        stock: Option<&Stock>,
        cancel: &impl CancelSignal,
    ) -> Result<Vec<PendingMove>, String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("pocket.route");
        let entries = self
            .entries
            .as_ref()
            .ok_or("missing helical entry geometry")?;
        let h = self.operation.entry.as_ref().unwrap().helix();
        let feed = Some(h.feed_mm_min.unwrap_or(self.operation.plunge_mm_min));
        let component = entries.component(target)?;
        let current = [self.position[0], self.position[1]];
        let mut result = vec![];
        if self.position[2] == z && entries.component(current).ok() == Some(component) {
            result.extend(
                entries
                    .route(current, target, z, cancel)?
                    .into_iter()
                    .map(|c| (c, MoveKind::Link, feed)),
            );
        } else {
            let entry = &entries.locations[component];
            let c = entry.center;
            let (entry_top, opposite) = if entry.stock_edge {
                (self.operation.top_z_mm, false)
            } else {
                self.cleared_entry_start(c, entry.radius, z, stock, cancel)?
            };
            // Leave at feed to the feed height, rapid to the retract height,
            // travel, then rapid down to the feed height. With default heights
            // (feed = retract = clearance) the extra lines have zero length.
            let feed_z = self.operation.feed_z().max(self.position[2]);
            let retract_z = self.operation.retract_z().max(feed_z);
            let safe = [self.position[0], self.position[1], feed_z];
            let up = [self.position[0], self.position[1], retract_z];
            let x = c[0]
                + if opposite {
                    -entry.radius
                } else {
                    entry.radius
                };
            let over = [x, c[1], retract_z];
            let p = [x, c[1], self.operation.feed_z()];
            entry_line(&mut result, self.position, safe, MoveKind::Retract, None);
            entry_line(&mut result, safe, up, MoveKind::Rapid, None);
            entry_line(&mut result, up, over, MoveKind::Rapid, None);
            entry_line(&mut result, over, p, MoveKind::Rapid, None);
            let bottom = [p[0], p[1], z];
            if entry.stock_edge {
                entry_line(&mut result, p, bottom, MoveKind::Plunge, feed);
            } else {
                let top = [p[0], p[1], entry_top];
                entry_line(
                    &mut result,
                    p,
                    top,
                    if entry_top < self.operation.top_z_mm {
                        MoveKind::Link
                    } else {
                        MoveKind::Plunge
                    },
                    feed,
                );
                result.extend(
                    h.curves(
                        c,
                        entry.radius,
                        entry_top,
                        z,
                        self.operation.boundary_clockwise.unwrap_or(true),
                        opposite,
                    )?
                    .into_iter()
                    .map(|c| (c, MoveKind::Lead, feed)),
                );
            }
            let center = [c[0], c[1], z];
            entry_line(&mut result, bottom, center, MoveKind::Link, feed);
            let root = entries.root(component);
            let root3 = [root[0], root[1], z];
            entry_line(&mut result, center, root3, MoveKind::Link, feed);
            result.extend(
                entries
                    .route(root, target, z, cancel)?
                    .into_iter()
                    .map(|c| (c, MoveKind::Link, feed)),
            );
        }
        for (curve, _, _) in &result {
            if cancel.is_cancelled() {
                return Err("helical pocket route cancelled".into());
            }
            curve.prepare(self.operation.tolerance_mm)?;
        }
        Ok(result)
    }
    pub fn report(&self) -> &PocketReport {
        &self.report
    }
    pub(crate) fn outside_stock_disk(&self, center: Point, radius: f64) -> bool {
        let distance = (0..2)
            .map(|i| {
                (self.stock_spec.min_mm[i] - center[i])
                    .max(center[i] - self.stock_spec.max_mm[i])
                    .max(0.)
                    .powi(2)
            })
            .sum::<f64>()
            .sqrt();
        let scale = center[0].abs().max(center[1].abs()).max(radius);
        distance > radius + 16. * self.operation.tolerance_mm + 256. * f64::EPSILON * (1. + scale)
    }
    /// Activate an untouched prepared layer; geometry/entry admission is reused.
    pub(crate) fn activate_layer(
        &mut self,
        initial: Vec3,
        first_id: u64,
        stock: &StockSpec,
    ) -> Result<(), String> {
        // The owner admits a lower material envelope before binding the
        // original stock/delta identity here.
        let mut envelope = stock.clone();
        envelope.max_mm[2] = self.stock_spec.max_mm[2];
        if self.pass != 0
            || self.stroke != 0
            || !self.pending.is_empty()
            || self.finished
            || self.expected_stock_generation.is_some()
            || self.position != self.initial
            || first_id == 0
            || initial.iter().any(|x| !x.is_finite() || x.abs() > 1e6)
            || initial[2] < self.operation.clearance_z_mm
            || envelope != self.stock_spec
            || stock.max_mm[2] < self.stock_spec.max_mm[2]
        {
            return Err("cannot activate a used pocket layer or invalid start".into());
        }
        self.initial = initial;
        self.position = initial;
        self.next_id = first_id;
        self.stock_spec = stock.clone();
        Ok(())
    }
    pub(crate) fn guide_strokes(&self) -> &[Stroke] {
        &self.strokes
    }
    /// Offset for a physical cutter axis, retaining the source, allowance and
    /// construction reserves. Adaptive guide radii additionally contain an
    /// orbit radius; that spacing does not constrain a direct contour axis.
    pub(crate) fn cutter_axis_clearance(&self, cutter_radius: f64) -> f64 {
        cutter_radius + (self.report.applied_clearance_mm - self.radius)
    }
    /// Excluded linear loops of the selected ORIGINAL source component. These
    /// are a proposal footprint only, never a substitute for whole-tube fit.
    pub(crate) fn front_obstacles(&self, seed: Point) -> Result<Vec<Point>, String> {
        for component in &self.source.definition().components {
            let selected = PlanarRegion {
                components: vec![component.clone()],
                ..self.source.definition().clone()
            }
            .prepare(ConstructionOptions {
                tolerance_mm: self.operation.tolerance_mm,
            })?;
            let q = selected.query(seed)?;
            if !q
                .signed_clearance_mm
                .is_some_and(|d| d >= -2. * q.uncertainty_mm)
            {
                continue;
            }
            let count: usize = component.holes.iter().map(|h| h.vertices.len()).sum();
            if count > 4096
                || component
                    .holes
                    .iter()
                    .any(|h| h.vertices.iter().any(|p| p[2] != 0.))
            {
                return Ok(vec![]);
            }
            return Ok(component
                .holes
                .iter()
                .flat_map(|h| h.vertices.iter().map(|p| [p[0], p[1]]))
                .collect());
        }
        Ok(vec![])
    }
    /// Same nominal morphological reachable domain as guide_residual, scoped
    /// to the centre component owning a real admitted entry. No path authority.
    pub(crate) fn front_component_region(
        &self,
        seed: Point,
        inflation: f64,
    ) -> Result<(PreparedRegion, PreparedRegion), String> {
        let options = ConstructionOptions {
            tolerance_mm: self.operation.tolerance_mm,
        };
        let mut selected = None;
        let mut best = f64::NEG_INFINITY;
        for component in &self.centers.definition().components {
            let region = PlanarRegion {
                components: vec![component.clone()],
                ..self.centers.definition().clone()
            }
            .prepare(options)?;
            let q = region.query(seed)?;
            if let Some(signed) = q.signed_clearance_mm
                && signed >= -2. * q.uncertainty_mm
                && signed > best
            {
                best = signed;
                selected = Some(region);
            }
        }
        let centres = selected.ok_or("stock-front entry has no matching centre component")?;
        let domain = centres.offset_prepared(-inflation)?;
        Ok((centres, domain.into_parts().1))
    }
    /// Corridor proposals only: the caller must fit the complete tube against
    /// the original source and prove the entry disk clear in committed stock.
    pub(crate) fn front_channel_rails(
        &self,
        seed: Point,
        material: [Point; 2],
        margin: f64,
    ) -> Result<Vec<Stroke>, String> {
        let mut rails: Vec<Stroke> = Vec::new();
        let tolerance = self.operation.tolerance_mm;
        for component in &self.source.definition().components {
            if component.holes.len() < 2 {
                continue;
            }
            let selected = PlanarRegion {
                components: vec![component.clone()],
                ..self.source.definition().clone()
            }
            .prepare(ConstructionOptions {
                tolerance_mm: tolerance,
            })?;
            let q = selected.query(seed)?;
            if !q
                .signed_clearance_mm
                .is_some_and(|d| d >= -2. * q.uncertainty_mm)
            {
                continue;
            }
            let edges: Vec<_> = component
                .holes
                .iter()
                .enumerate()
                .flat_map(|(i, h)| {
                    Stroke::from_boundary(h)
                        .into_iter()
                        .filter(|s| s.arc.is_none())
                        .map(move |s| (i, s))
                })
                .collect();
            if edges.len() > 256 {
                return Ok(rails);
            }
            let corners = [
                material[0],
                [material[0][0], material[1][1]],
                material[1],
                [material[1][0], material[0][1]],
            ];
            for (i, (hole, a)) in edges.iter().enumerate() {
                let length = (a.b[0] - a.a[0]).hypot(a.b[1] - a.a[1]);
                if length <= 32. * tolerance {
                    continue;
                }
                let t = [(a.b[0] - a.a[0]) / length, (a.b[1] - a.a[1]) / length];
                let n = [t[1], -t[0]];
                let dot = |p: Point, v: Point| p[0] * v[0] + p[1] * v[1];
                for (other, b) in &edges[i + 1..] {
                    if hole == other {
                        continue;
                    }
                    let bd = [b.b[0] - b.a[0], b.b[1] - b.a[1]];
                    let bl = bd[0].hypot(bd[1]);
                    if bl <= 32. * tolerance
                        || (bd[0] * t[1] - bd[1] * t[0]).abs() > 4096. * f64::EPSILON * bl
                    {
                        continue;
                    }
                    let aa = dot(a.a, t);
                    let ab = dot(a.b, t);
                    let ba = dot(b.a, t);
                    let bb = dot(b.b, t);
                    if aa.max(ab).min(ba.max(bb)) - aa.min(ab).max(ba.min(bb)) <= 32. * tolerance {
                        continue;
                    }
                    let av = dot(a.a, n);
                    let bv = dot(b.a, n);
                    if (av - bv).abs() <= 2. * margin {
                        continue;
                    }
                    let v = (av + bv) / 2.;
                    let lo = corners
                        .iter()
                        .map(|&p| dot(p, t))
                        .fold(f64::INFINITY, f64::min)
                        - margin;
                    let hi = corners
                        .iter()
                        .map(|&p| dot(p, t))
                        .fold(f64::NEG_INFINITY, f64::max)
                        + margin;
                    let point = |u: f64| [u * t[0] + v * n[0], u * t[1] + v * n[1]];
                    let rail = Stroke::line(point(lo), point(hi));
                    let near =
                        |a: Point, b: Point| (a[0] - b[0]).hypot(a[1] - b[1]) <= 64. * tolerance;
                    if !rails.iter().any(|old| {
                        (near(old.a, rail.a) && near(old.b, rail.b))
                            || (near(old.a, rail.b) && near(old.b, rail.a))
                    }) {
                        rails.push(rail);
                    }
                    if rails.len() >= 64 {
                        return Ok(rails);
                    }
                }
            }
            break;
        }
        Ok(rails)
    }
    pub(crate) fn guide_cover_region(&self, inflation: f64) -> Result<PlanarRegion, String> {
        Ok(self.centers.offset(-inflation)?.region)
    }
    /// Clip straight guides against a conservatively eroded covered domain.
    /// Transform to the line's local XY frame and reuse analytic line/circle
    /// scan intersections; parity ambiguity retains the whole original guide.
    pub(crate) fn guides_outside(
        &self,
        cover: &PlanarRegion,
        cancel: &impl CancelSignal,
    ) -> Result<Vec<Stroke>, String> {
        let edges: Vec<_> = cover
            .components
            .iter()
            .flat_map(|c| std::iter::once(&c.outer).chain(&c.holes))
            .flat_map(Stroke::from_boundary)
            .collect();
        let mut result = vec![];
        let guard = 8. * self.operation.tolerance_mm;
        for &s in &self.strokes {
            if cancel.is_cancelled() {
                return Err("adaptive fine-guide clipping cancelled".into());
            }
            if s.arc.is_some() {
                result.push(s);
                continue;
            }
            let length = norm(s.a, s.b);
            let u = [(s.b[0] - s.a[0]) / length, (s.b[1] - s.a[1]) / length];
            let local = |p: Point| {
                [
                    (p[0] - s.a[0]) * u[0] + (p[1] - s.a[1]) * u[1],
                    -(p[0] - s.a[0]) * u[1] + (p[1] - s.a[1]) * u[0],
                ]
            };
            let mut xs: Vec<_> = edges
                .iter()
                .flat_map(|e| {
                    Crossing::split(Stroke {
                        a: local(e.a),
                        b: local(e.b),
                        arc: e.arc.map(|(c, sweep)| (local(c), sweep)),
                    })
                })
                .filter_map(|e| e.x_at(0.))
                .collect();
            xs.sort_by(f64::total_cmp);
            if xs.len() % 2 != 0 {
                result.push(s);
                continue;
            }
            let mut cursor = 0f64;
            let world = |d: f64| [s.a[0] + d * u[0], s.a[1] + d * u[1]];
            for pair in xs.as_chunks::<2>().0 {
                let lo = (pair[0] + guard).clamp(0., length);
                let hi = (pair[1] - guard).clamp(0., length);
                if hi <= lo || hi <= cursor {
                    continue;
                }
                if lo > cursor {
                    result.push(Stroke::line(world(cursor), world(lo)));
                }
                cursor = cursor.max(hi);
            }
            if cursor < length {
                result.push(Stroke::line(world(cursor), s.b));
            }
        }
        Ok(result)
    }
    pub(crate) fn source_component(
        &self,
        p: Point,
        work: &mut usize,
        limit: usize,
        cancel: &impl CancelSignal,
    ) -> Result<Option<usize>, String> {
        self.source.interior_component(p, work, limit, cancel)
    }
    pub(crate) fn link_work_per_query(&self) -> usize {
        1 + 2 * self.source_edges.len()
    }
    /// Original-region whole-guide tube radius interval. The lower bound
    /// admits an entire line/arc; the upper bound limits possible local growth
    /// and avoids splitting constant-clearance boundary guides needlessly.
    pub(crate) fn guide_radius_bounds(&self, stroke: Stroke) -> Option<(f64, f64)> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("source.guide_bounds");
        let q = self.source.query(stroke_point(stroke, 0.5)).ok()?;
        let signed = q.signed_clearance_mm?;
        if signed <= 0. {
            return None;
        }
        let magnitude = [stroke.a[0], stroke.a[1], stroke.b[0], stroke.b[1]]
            .into_iter()
            .map(f64::abs)
            .fold(self.source_magnitude, f64::max);
        let numeric = 1024. * f64::EPSILON * magnitude;
        let reserve = self.operation.radial_allowance_mm
            + q.uncertainty_mm
            + 4. * self.operation.tolerance_mm;
        let mut lower = signed;
        let mut upper = f64::INFINITY;
        for &edge in &self.source_edges {
            lower = lower.min(stroke_distance(stroke, edge));
            upper = upper.min(maximum_boundary_distance(stroke, edge));
        }
        Some((
            (lower - reserve - numeric).max(0.),
            (upper - reserve + numeric + 8. * self.operation.tolerance_mm).max(0.),
        ))
    }
    /// Boolean version of the original whole-guide lower-radius condition.
    /// Every boundary must pass; a midpoint alone never admits the tube. Keep
    /// subtraction order identical to guide_radius_bounds at guarded limits.
    pub(crate) fn approve_guide_tube(
        &self,
        curve: &Curve,
        radius: f64,
    ) -> Option<GuideTubeApproval> {
        let (a, b) = (curve.start(), curve.end());
        let arc = match curve {
            Curve::Line { .. } => None,
            Curve::Arc {
                center,
                plane: Plane::Xy,
                sweep_rad,
                ..
            } => Some(([center[0], center[1]], *sweep_rad)),
            _ => return None,
        };
        let stroke = Stroke {
            a: [a[0], a[1]],
            b: [b[0], b[1]],
            arc,
        };
        self.guide_tube_fits(stroke, radius)
            .then(|| GuideTubeApproval {
                owner: Arc::clone(&self.source_owner),
                curve: curve.clone(),
                radius,
                tolerance: self.operation.tolerance_mm,
            })
    }
    pub(crate) fn guide_tube_fits(&self, stroke: Stroke, required_radius: f64) -> bool {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("source.guide_admission");
        if !required_radius.is_finite() || required_radius <= 0. {
            return false;
        }
        let magnitude = [stroke.a[0], stroke.a[1], stroke.b[0], stroke.b[1]]
            .into_iter()
            .map(f64::abs)
            .fold(self.source_magnitude, f64::max);
        let numeric = 1024. * f64::EPSILON * magnitude;
        let reserve = self.operation.radial_allowance_mm
            + (self.source.definition().boundary_error_mm + self.operation.tolerance_mm)
            + 4. * self.operation.tolerance_mm;
        let fits = |distance: f64| (distance - reserve - numeric).max(0.) >= required_radius;
        let bounds = StrokeBounds::new(stroke);
        // Test the complete boundary first: a rejected candidate needs no sign
        // query. Passing this broad phase never bypasses the inside/clearance test.
        self.source_edges
            .iter()
            .zip(&self.source_bounds)
            .all(|(&edge, &edge_bounds)| {
                fits(bounds.separation(edge_bounds)) || fits(stroke_distance(stroke, edge))
            })
            && self
                .source
                .query(stroke_point(stroke, 0.5))
                .ok()
                .and_then(|q| q.signed_clearance_mm)
                .is_some_and(fits)
    }
    pub(crate) fn guide_residual(
        &self,
        radius: f64,
    ) -> Result<crate::planar::PreparedConstruction, String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("guide.residual");
        let inflated = self.centers.offset_prepared(-radius)?;
        let desired = self
            .source
            .offset_prepared(self.operation.radial_allowance_mm)?;
        desired
            .region()
            .boolean_prepared(inflated.region(), Boolean::Difference)
    }
    /// Deferred diagnostic work. Actual material removal can stream first;
    /// a pending or unknown residual must never be interpreted as all clear.
    pub fn finish_report(&mut self, cancel: &impl CancelSignal) -> Result<&PocketReport, String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("pocket.finish");
        if !self.finished || !self.pending.is_empty() {
            return Err("drain pocket motions before final report".into());
        }
        if self.report.geometric_residual_status != ResidualStatus::Pending {
            return Ok(&self.report);
        }
        if cancel.is_cancelled() {
            return Err("pocket report cancelled".into());
        }
        let residual = (|| {
            let inflated = self.centers.offset_prepared(-self.radius)?;
            let desired = self
                .source
                .offset_prepared(self.operation.radial_allowance_mm)?;
            desired
                .region()
                .boolean_prepared(inflated.region(), Boolean::Difference)
        })();
        if cancel.is_cancelled() {
            return Err("pocket report cancelled".into());
        }
        match residual {
            Ok(residual) => {
                let (residual, prepared) = residual.into_parts();
                self.report.geometric_residual_area_mm2 = Some(prepared.summary().area_mm2);
                self.report.geometric_residual = Some(residual.region);
                self.report.geometric_residual_status = ResidualStatus::Estimated;
                self.report.diagnostics.extend(residual.diagnostics);
            }
            Err(error) => {
                self.report.geometric_residual_status = ResidualStatus::Unknown;
                self.report.diagnostics.push(format!("geometric residual is unknown because its boundary is ambiguous at the requested tolerance: {error}; do not interpret this as cleared material"));
            }
        }
        Ok(&self.report)
    }
    pub fn prefix_events(&self) -> Vec<Event> {
        let p = &self.operation;
        vec![
            Event::WorkOffset {
                number: p.work_offset,
            },
            Event::ToolChange {
                tool: p.tool.clone(),
                number: p.tool_number,
                length_register: p.length_register,
                position_after_mm: self.initial,
            },
            Event::Spindle {
                rpm: p.spindle_rpm,
                direction: SpindleDirection::Cw,
            },
            Event::Coolant { mode: p.coolant },
        ]
    }
    pub fn suffix_events(&self) -> Vec<Event> {
        vec![
            Event::Coolant { mode: Coolant::Off },
            Event::Spindle {
                rpm: 0.,
                direction: SpindleDirection::Cw,
            },
        ]
    }
    pub fn position(&self) -> Vec3 {
        self.position
    }
    /// The sign at the midpoint plus minimum separation from every boundary
    /// establishes whole-segment fit. No endpoint-only or animation sampling.
    pub(crate) fn link_geometry_fits(&self, a: Point, b: Point) -> bool {
        self.link_geometry_fits_radius(a, b, self.radius)
    }
    pub(crate) fn link_geometry_fits_radius(&self, a: Point, b: Point, radius: f64) -> bool {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("source.link_bounds");
        let p = &self.operation;
        let Ok(q) = self.source.query([(a[0] + b[0]) / 2., (a[1] + b[1]) / 2.]) else {
            return false;
        };
        let required = radius + p.radial_allowance_mm + q.uncertainty_mm + 4. * p.tolerance_mm;
        if q.signed_clearance_mm.is_none_or(|d| d < required) {
            return false;
        }
        // Bound arithmetic at the declared coordinate scale, in addition to the
        // source/construction uncertainty. Near-boundary uncertainty retracts.
        let guard = 1024. * f64::EPSILON * self.source_magnitude;
        let bounds = StrokeBounds::new(Stroke::line(a, b));
        self.source_edges
            .iter()
            .zip(&self.source_bounds)
            .all(|(&s, &edge_bounds)| {
                bounds.separation(edge_bounds) - guard >= required
                    || segment_distance(a, b, s) - guard >= required
            })
    }
    /// Leave the cut at feed up to the feed height (above all stock), then
    /// rapid to `to_z`. Returns the point reached.
    fn queue_lift(&mut self, from: Vec3, to_z: f64) -> Vec3 {
        let feed_z = self.operation.feed_z().max(from[2]);
        let safe = [from[0], from[1], feed_z];
        let up = [from[0], from[1], to_z.max(feed_z)];
        self.queue_line(from, safe, MoveKind::Retract);
        self.queue_line(safe, up, MoveKind::Rapid);
        up
    }
    /// Lift to the retract height, travel there, rapid down to the feed
    /// height, then plunge at plunge feed. With default heights (retract =
    /// feed = clearance) this is the original retract/rapid/plunge sequence.
    fn queue_relink(&mut self, start: Vec3) {
        let up = self.queue_lift(self.position, self.operation.retract_z());
        let across = [start[0], start[1], up[2]];
        let above = [start[0], start[1], self.operation.feed_z().max(start[2])];
        self.queue_line(up, across, MoveKind::Rapid);
        self.queue_line(across, above, MoveKind::Rapid);
        self.queue_line(above, start, MoveKind::Plunge);
    }
    fn queue_line(&mut self, from: Vec3, to: Vec3, kind: MoveKind) {
        if from != to {
            self.pending.push_back((
                Curve::Line {
                    start: from,
                    end: to,
                },
                kind,
                None,
            ));
        }
    }
    /// The caller drains one motion before requesting the next. Default entry
    /// retains at most six pending moves (four with default heights); helical entry bounds each pending
    /// route to MAX_ENTRY_ROUTE, irrespective of the number of depth passes.
    pub fn next_motion(&mut self, cancel: &impl CancelSignal) -> Result<Option<Motion>, String> {
        if self.operation.rest_tolerance_mm.is_some()
            || self.report.empty_cuts.is_some()
            || self.stock_top.is_some()
        {
            return Err("stock-aware pocket requires committed stock feedback".into());
        }
        self.next_with_stock(None, cancel)
    }
    pub fn next_motion_with_stock(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        if let Some(bound) = &mut self.stock_top {
            bound.validate(stock, cancel)?;
        }
        if self.operation.rest_tolerance_mm.is_some()
            || self.operation.entry.is_some()
            || self.report.empty_cuts.is_some()
            || self.stock_top.is_some()
        {
            if stock.spec() != &self.stock_spec
                || stock.generation() == u64::MAX
                || self
                    .expected_stock_generation
                    .is_some_and(|g| g != stock.generation())
            {
                return Err(
                    "commit the preceding motion to matching stock before rest planning".into(),
                );
            }
            if self.operation.rest_tolerance_mm.is_some() {
                self.report
                    .rest_from_generation
                    .get_or_insert(stock.generation());
                self.report.rest_through_generation = Some(stock.generation());
            }
        }
        if let Some(tool) = &self.rest_tool {
            // Skipped pieces and queued links assume every preceding emitted
            // move was fully committed to this stock with this cutter. A mere
            // generation increment also describes partial/unrelated sweeps.
            if let Some(base) = self.rest_stock_identity {
                if let Some(pending) = &self.rest_pending {
                    let valid = stock.last_sweep_receipt().is_some_and(|r| {
                        r.base_identity == base
                            && r.motion.commits(pending)
                            && &r.tool == tool
                            && r.fraction == 1.
                    });
                    if !valid {
                        return Err("commit the complete preceding rest motion with the matching cutter and stock before continuing".into());
                    }
                } else if stock.identity() != base {
                    return Err("rest planning stock changed during an interrupted query".into());
                }
            }
            stock.request_sweep_receipts();
            self.rest_stock_identity.get_or_insert(stock.identity());
        }
        let result = self.next_with_stock(Some(stock), cancel)?;
        if self.rest_tool.is_some()
            && let Some(motion) = &result
        {
            self.rest_stock_identity = Some(stock.identity());
            self.rest_pending = Some(motion.clone());
        }
        if (self.operation.rest_tolerance_mm.is_some()
            || self.operation.entry.is_some()
            || self.report.empty_cuts.is_some()
            || self.stock_top.is_some())
            && result.is_some()
        {
            self.expected_stock_generation = Some(stock.generation() + 1);
        }
        Ok(result)
    }
    fn advance_stroke(&mut self) {
        self.advance_to(self.stroke + 1);
    }
    fn advance_to(&mut self, next: usize) {
        self.stroke = next;
        if self.stroke == self.strokes.len() {
            self.stroke = 0;
            self.pass += 1;
        }
    }
    fn rest_has_material(
        &mut self,
        stock: &Stock,
        s: Stroke,
        z: f64,
        tolerance: f64,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        self.report.rest_candidates += 1;
        let candidate = Motion {
            id: self.next_id,
            operation: self.operation.id.clone(),
            tool: self.operation.tool.clone(),
            kind: MoveKind::Cut,
            curve: s.curve(z),
            feed_mm_min: Some(self.operation.feed_mm_min),
        };
        self.material_on_motion(stock, &candidate, tolerance, cancel)
    }
    fn material_on_motion(
        &mut self,
        stock: &Stock,
        motion: &Motion,
        tolerance: f64,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        let mut work = usize::MAX;
        let before = work;
        let material = stock.may_remove_material(
            motion,
            self.radius,
            tolerance,
            self.operation.tolerance_mm,
            &mut work,
            cancel,
        )?;
        self.report.rest_query_work += before - work;
        self.report.rest_query_work_exhausted |= work == 0;
        Ok(material)
    }
    fn next_with_stock(
        &mut self,
        stock: Option<&Stock>,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        if cancel.is_cancelled() {
            return Err("pocket generation cancelled; prefix remains partial".into());
        }
        if self.finished && self.pending.is_empty() {
            return Ok(None);
        }
        while self.pending.is_empty() && !self.finished {
            if cancel.is_cancelled() {
                return Err("rest generation cancelled; prefix remains partial".into());
            }
            if self.pass == self.report.depth_passes || self.strokes.is_empty() {
                self.queue_lift(self.position, self.operation.clearance_z_mm);
                self.finished = true;
            } else {
                let z = (self.operation.top_z_mm
                    - (self.pass + 1) as f64 * self.operation.stepdown_mm)
                    .max(self.report.target_z_mm);
                let mut s = self.strokes[self.stroke];
                let mut next = self.stroke + 1;
                if let Some(proof) = &mut self.report.empty_cuts {
                    let candidate = Motion {
                        id: self.next_id,
                        operation: self.operation.id.clone(),
                        tool: self.operation.tool.clone(),
                        kind: MoveKind::Cut,
                        curve: s.curve(z),
                        feed_mm_min: Some(self.operation.feed_mm_min),
                    };
                    if proof.may_omit(
                        stock.ok_or("missing empty-cut stock")?,
                        &candidate,
                        self.radius,
                        self.operation.tolerance_mm,
                        cancel,
                    )? {
                        // Only virtual guide progress changes. All links and
                        // entries below start at the actual last emitted pose.
                        self.advance_stroke();
                        continue;
                    }
                }
                if let Some(tolerance) = self.operation.rest_tolerance_mm {
                    let stock = stock.ok_or("missing rest stock")?;
                    let material = self.rest_has_material(stock, s, z, tolerance, cancel)?;
                    if !material {
                        self.report.rest_skipped_strokes += 1;
                        self.advance_stroke();
                        continue;
                    }
                    let group_end = self.rest_group_ends[self.stroke];
                    let mut end = next;
                    while end < group_end {
                        if cancel.is_cancelled() {
                            return Err("rest lookahead cancelled; cursor not advanced".into());
                        }
                        if self.report.rest_query_work_exhausted {
                            self.report.rest_candidates += (group_end - end) as u64;
                            end = group_end;
                            break;
                        }
                        if !self.rest_has_material(
                            stock,
                            self.strokes[end],
                            z,
                            tolerance,
                            cancel,
                        )? {
                            break;
                        }
                        end += 1;
                    }
                    if end > next {
                        s.b = self.strokes[end - 1].b;
                        s.arc = s
                            .arc
                            .map(|(c, sweep)| (c, sweep * (end - self.stroke) as f64));
                        s.curve(z).prepare(self.operation.tolerance_mm)?;
                        self.report.rest_merged_pieces += (end - self.stroke - 1) as u64;
                    }
                    // The next queried clear piece stays clear under subsequent
                    // monotone stock cuts, so consume it without querying twice.
                    next = end;
                    if end < group_end {
                        next += 1;
                        self.report.rest_skipped_strokes += 1;
                    }
                }
                let start = [s.a[0], s.a[1], z];
                let a = [self.position[0], self.position[1]];
                let short = norm(a, s.a) <= 4. * self.operation.stepover_mm;
                let fit = self.position[2] == z
                    && (short
                        || self.operation.rest_tolerance_mm.is_some()
                        || self.report.empty_cuts.is_some())
                    && self.link_geometry_fits(a, s.a);
                let mut link = fit && short;
                // Automatic: a sufficient feed-only saving. Retracting to the
                // feed height and plunging back takes at least this long
                // (above the feed height the tool moves at rapid).
                let economical = match self.operation.max_stay_down_mm {
                    Some(limit) => norm(a, s.a) <= limit,
                    None => {
                        norm(a, s.a)
                            <= (self.operation.feed_z() - z)
                                * (1. + self.operation.feed_mm_min / self.operation.plunge_mm_min)
                    }
                };
                if fit
                    && !short
                    && economical
                    && (self.operation.rest_tolerance_mm.is_some()
                        || self.report.empty_cuts.is_some())
                {
                    let motion = Motion {
                        id: self.next_id,
                        operation: self.operation.id.clone(),
                        tool: self.operation.tool.clone(),
                        kind: MoveKind::Link,
                        curve: Curve::Line {
                            start: self.position,
                            end: start,
                        },
                        feed_mm_min: Some(self.operation.feed_mm_min),
                    };
                    if let Some(tolerance) = self.operation.rest_tolerance_mm {
                        link = !self.material_on_motion(
                            stock.ok_or("missing rest stock")?,
                            &motion,
                            tolerance,
                            cancel,
                        )?;
                        if link {
                            self.report.rest_long_links += 1;
                        }
                    } else if let Some(proof) = &mut self.report.empty_cuts {
                        link = proof.may_link(
                            stock.ok_or("missing empty-link stock")?,
                            &motion,
                            self.radius,
                            self.operation.tolerance_mm,
                            cancel,
                        )?;
                    }
                }
                if link {
                    self.queue_line(self.position, start, MoveKind::Link);
                } else if self.entries.is_some() {
                    let mut moves = self.helical_moves(s.a, z, stock, cancel)?;
                    if let Some(proof) = &mut self.report.empty_cuts {
                        for (curve, kind, feed) in &mut moves {
                            if *kind != MoveKind::Link
                                || curve.start()[2] != curve.end()[2]
                                || feed.is_none_or(|f| f >= self.operation.feed_mm_min)
                            {
                                continue;
                            }
                            let candidate = Motion {
                                id: self.next_id,
                                operation: self.operation.id.clone(),
                                tool: self.operation.tool.clone(),
                                kind: MoveKind::Link,
                                curve: curve.clone(),
                                feed_mm_min: Some(self.operation.feed_mm_min),
                            };
                            if proof.may_link(
                                stock.ok_or("missing empty-route stock")?,
                                &candidate,
                                self.radius,
                                self.operation.tolerance_mm,
                                cancel,
                            )? {
                                // These graph edges are already source-admitted
                                // navigation. Empty travel uses the authored
                                // linking/cutting feed; actual entry stays at
                                // its independently authored entry feed.
                                *feed = None;
                            }
                        }
                    }
                    self.pending.extend(moves);
                } else {
                    self.queue_relink(start);
                }
                self.pending.push_back((s.curve(z), MoveKind::Cut, None));
                self.advance_to(next);
            }
        }
        let Some((curve, kind, entry_feed)) = self.pending.pop_front() else {
            return Ok(None);
        };
        curve.prepare(self.operation.tolerance_mm)?;
        let motion = Motion {
            id: self.next_id,
            operation: self.operation.id.clone(),
            tool: self.operation.tool.clone(),
            feed_mm_min: if kind == MoveKind::Rapid {
                None
            } else {
                Some(entry_feed.unwrap_or(if kind == MoveKind::Plunge {
                    self.operation.plunge_mm_min
                } else {
                    self.operation.feed_mm_min
                }))
            },
            kind,
            curve,
        };
        if entry_feed.is_some() {
            if let Curve::Helix {
                sweep_rad,
                start,
                end,
                ..
            } = &motion.curve
            {
                if self.entry_helix_end != Some(*start) {
                    self.report.helical_entries += 1;
                }
                self.entry_helix_end = Some(*end);
                self.report.entry_turns += (sweep_rad.abs() / TAU).round() as usize;
            } else if kind == MoveKind::Plunge && motion.curve.end()[2] < self.operation.top_z_mm {
                self.report.stock_edge_entries += 1;
            } else if kind == MoveKind::Link && motion.curve.start()[2] > motion.curve.end()[2] {
                self.report.cleared_entry_descents += 1;
            } else if kind == MoveKind::Link {
                self.report.entry_navigation_moves += 1;
            }
        }
        self.next_id = self
            .next_id
            .checked_add(1)
            .ok_or("pocket motion IDs overflow")?;
        self.position = motion.curve.end();
        Ok(Some(motion))
    }
}

#[cfg(test)]
#[path = "pocket/admission_tests.rs"]
mod admission_tests;

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn continuous_link_distance_considers_arc_intersections_and_stationary_points() {
        let upper = Stroke {
            a: [-3., 0.],
            b: [3., 0.],
            arc: Some(([0., 0.], -PI)),
        };
        let close = |a: f64, b: f64| assert!((a - b).abs() < 1e-10, "{a} vs {b}");
        close(segment_distance([-4., 4.], [4., 4.], upper), 1.);
        close(segment_distance([-4., 2.], [4., 2.], upper), 0.);
        close(segment_distance([-4., -2.], [4., -2.], upper), 2.);
        close(segment_distance([4., -4.], [4., 4.], upper), 1.);
        let edge = Stroke {
            a: [0., -1.],
            b: [0., 1.],
            arc: None,
        };
        close(segment_distance([-1., 0.], [1., 0.], edge), 0.);
        close(segment_distance([1., -4.], [1., 4.], edge), 1.);
    }
    #[test]
    fn whole_arc_pair_and_radius_upper_bounds_cover_interior_extrema() {
        let upper = Stroke {
            a: [-3., 0.],
            b: [3., 0.],
            arc: Some(([0., 0.], -PI)),
        };
        let lower = Stroke {
            a: [2., 8.],
            b: [-2., 8.],
            arc: Some(([0., 8.], -PI)),
        };
        assert!((stroke_distance(upper, lower) - 3.).abs() < 1e-10);
        let crossing = Stroke {
            a: [2., 4.],
            b: [-2., 4.],
            arc: Some(([0., 4.], -PI)),
        };
        assert_eq!(stroke_distance(upper, crossing), 0.);
        let concentric = Stroke {
            a: [-5., 0.],
            b: [5., 0.],
            arc: Some(([0., 0.], -PI)),
        };
        assert!((stroke_distance(upper, concentric) - 2.).abs() < 1e-10);
        assert!((maximum_boundary_distance(upper, concentric) - 2.).abs() < 1e-10);
        let wall = Stroke::line([-10., 5.], [10., 5.]);
        let flat = Stroke::line([-3., 0.], [3., 0.]);
        assert_eq!(stroke_distance(flat, wall), 5.);
        assert_eq!(maximum_boundary_distance(flat, wall), 5.);
        assert!((stroke_distance(upper, wall) - 2.).abs() < 1e-10);
        assert!(maximum_boundary_distance(upper, wall) >= 5.);
        let near_top = Stroke::line([-0.001, 4.], [0.001, 4.]);
        assert!(maximum_boundary_distance(near_top, concentric) < 1.000001);
        assert!(maximum_boundary_distance(near_top, concentric) >= 1.);
    }
    #[test]
    fn half_open_circle_scan_keeps_tangent_double_roots() {
        let circle = [
            Stroke {
                a: [-3., 0.],
                b: [3., 0.],
                arc: Some(([0., 0.], PI)),
            },
            Stroke {
                a: [3., 0.],
                b: [-3., 0.],
                arc: Some(([0., 0.], PI)),
            },
        ];
        let pieces: Vec<_> = circle.into_iter().flat_map(Crossing::split).collect();
        let mut lower: Vec<_> = pieces.iter().filter_map(|p| p.x_at(-3.)).collect();
        lower.sort_by(f64::total_cmp);
        assert_eq!(lower, vec![0., 0.]);
        let mut middle: Vec<_> = pieces.iter().filter_map(|p| p.x_at(0.)).collect();
        middle.sort_by(f64::total_cmp);
        assert_eq!(middle, vec![-3., 3.]);
        assert!(pieces.iter().all(|p| p.x_at(3.).is_none()));
    }
}
