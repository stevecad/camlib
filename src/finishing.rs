//! Adaptive, continuously checked parallel finishing of faceted upper surfaces.
use crate::{
    facing::TraverseAxis,
    motion::{Coolant, Curve, Event, Motion, MoveKind, SpindleDirection, Vec3},
    pocket::ResidualStatus,
    stock::{CancelSignal, Stock, StockSpec},
    surface::{PreparedSurface, SurfaceMesh, SurfaceWork},
    tool::{CutterShape, ToolAssembly, valid_id},
};
use serde::{Deserialize, Serialize};

mod patterns;
mod pencil;
mod scallop;
pub use patterns::FinishPattern;
/// One XY polyline of a scan pattern.
type Pass = Vec<[f64; 2]>;

/// Whole-stock upper-height premise for a finishing cutter after roughing.
/// This reports actual conservative stock, not the nominal CAD surface height.
#[derive(Clone, Serialize)]
pub struct FinishingStockTop {
    pub original_top_z_mm: f64,
    pub maximum_upper_z_mm: f64,
    pub checked_cells: usize,
    pub generation: u64,
}

pub(crate) struct StockCeiling {
    identity: u64,
    pub(crate) report: FinishingStockTop,
}
impl StockCeiling {
    fn maximum(stock: &Stock, cancel: &impl CancelSignal) -> Result<f64, String> {
        let mut maximum = stock.spec().min_mm[2];
        for (i, cell) in stock.cells().iter().enumerate() {
            if i.is_multiple_of(1024) && cancel.is_cancelled() {
                return Err("finishing stock-height inspection cancelled".into());
            }
            maximum = maximum.max(cell.upper_mm);
        }
        if cancel.is_cancelled() {
            return Err("finishing stock-height inspection cancelled".into());
        }
        Ok(maximum)
    }
    pub(crate) fn prepare(
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<(StockSpec, Self), String> {
        let maximum = Self::maximum(stock, cancel)?;
        let mut envelope = stock.spec().clone();
        envelope.max_mm[2] = maximum;
        stock.request_sweep_receipts();
        Ok((
            envelope,
            Self {
                identity: stock.identity(),
                report: FinishingStockTop {
                    original_top_z_mm: stock.spec().max_mm[2],
                    maximum_upper_z_mm: maximum,
                    checked_cells: stock.cells().len(),
                    generation: stock.generation(),
                },
            },
        ))
    }
    pub(crate) fn validate(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        if cancel.is_cancelled() {
            return Err("finishing stock-height inspection cancelled".into());
        }
        if stock.identity() != self.identity {
            // Only a native monotone successor inherits this ceiling. Restored,
            // foreign or undone stock must be inspected, even at equal generation.
            if !stock
                .last_sweep_receipt()
                .is_some_and(|r| r.base_identity == self.identity)
                && Self::maximum(stock, cancel)? > self.report.maximum_upper_z_mm
            {
                return Err("finishing stock exceeds the admitted remaining-stock height; prepare again from current stock".into());
            }
            self.identity = stock.identity();
        }
        stock.request_sweep_receipts();
        Ok(())
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ParallelOperation<G = SurfaceMesh> {
    pub id: String,
    pub tool: String,
    pub surface: G,
    pub floor_z_mm: f64,
    /// Row spacing (raster) or ring pitch (spiral). Not used by radial or
    /// project, which require 0.
    #[serde(default)]
    pub stepover_mm: f64,
    pub clearance_z_mm: f64,
    /// Raster row direction.
    #[serde(default = "default_axis")]
    pub axis: TraverseAxis,
    /// Raster: start the first row from the high end. Others require false.
    #[serde(default)]
    pub reverse: bool,
    #[serde(default, skip_serializing_if = "FinishPattern::is_raster")]
    pub pattern: FinishPattern,
    /// Contact (slope) angle limits, degrees from the tool axis: cut only
    /// where the surface slope at the ball contact lies in this range
    /// (0 flat, 90 vertical). Default 0..90, everywhere.
    #[serde(default, skip_serializing_if = "is_zero")]
    pub slope_from_deg: f64,
    #[serde(default = "ninety", skip_serializing_if = "is_ninety")]
    pub slope_to_deg: f64,
    /// Rest finishing: cut only where the current stock still stands more
    /// than this above the target (plus stock to leave), within the tool's
    /// reach. Needs the stock of the preceding operations.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rest_threshold_mm: Option<f64>,
    pub stock_to_leave_mm: f64,
    pub path_error_mm: f64,
    pub safety_margin_mm: f64,
    pub entry_air_gap_mm: f64,
    pub feed_mm_min: f64,
    pub plunge_mm_min: f64,
    pub tolerance_mm: f64,
    pub spindle_rpm: f64,
    pub work_offset: u8,
    pub tool_number: u32,
    pub length_register: u32,
    pub coolant: Coolant,
    /// Rapid travel height between rows (absolute Z). Default: clearance.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub retract_z_mm: Option<f64>,
    /// Rapid descent stops here (or at the row's air entry if higher) and
    /// plunge feed begins; leaving a row, feed changes to rapid here (absolute
    /// Z, above the stock top). Default: retract.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub feed_z_mm: Option<f64>,
    /// Containment and avoid areas (clamps, fixtures, features to leave alone).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub limits: Option<crate::limits::Limits>,
}
fn default_axis() -> TraverseAxis {
    TraverseAxis::X
}
pub(crate) fn ninety() -> f64 {
    90.
}
/// Stock cells standing above the target by more than a threshold.
struct RestMap {
    nx: usize,
    ny: usize,
    min: [f64; 2],
    step: [f64; 2],
    flags: Vec<bool>,
    /// Committed stock lower bounds per cell: stock certainly present.
    lower: Vec<f64>,
}
impl RestMap {
    /// Uses the stock's lower bounds (stock certainly present; the upper
    /// bounds are conservative by up to about a cell on slopes) and the
    /// highest the surface may rise within the cell, so a flagged cell
    /// certainly holds stock above the target by more than the threshold
    /// (thinner rest, within the stock resolution, is not seen).
    fn new(
        target: &crate::flat::Index,
        stock: &Stock,
        allowance: f64,
        threshold: f64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        let [nx, ny] = stock.dimensions();
        let step = stock.cell_widths_mm();
        let reach = step[0].hypot(step[1]) / 2.;
        let cells = stock.cells();
        let workers = std::thread::available_parallelism().map_or(1, usize::from);
        let rows = crate::roughing::parallel::ordered(
            ny,
            workers,
            cancel,
            |j, signal| {
                if signal.is_cancelled() {
                    return Err("rest finishing cancelled".into());
                }
                let (mut seen, mut stamp) = (vec![0u32; target.len()], 0u32);
                let mut row = Vec::with_capacity(nx);
                for i in 0..nx {
                    let q = stock.cell_center(j * nx + i).unwrap();
                    // Exact highest target point over the cell's disk.
                    let top = target.max_height(q, q, reach, &mut seen, &mut stamp);
                    row.push(cells[j * nx + i].lower_mm - (top + allowance) > threshold);
                }
                Ok(row)
            },
            |_| Ok(()),
        )?;
        let flags = rows.into_iter().flatten().collect();
        Ok(Self {
            nx,
            ny,
            min: [stock.spec().min_mm[0], stock.spec().min_mm[1]],
            step,
            flags,
            lower: cells.iter().map(|c| c.lower_mm).collect(),
        })
    }
    /// The ball (center `p`, height `center_z`) reaches more than
    /// `threshold` into the stock of some flagged cell: its lowest point over
    /// the cell (at the cell's nearest point) is that far below the stock.
    fn engaged(&self, p: [f64; 2], center_z: f64, radius: f64, threshold: f64) -> bool {
        let half = self.step[0].hypot(self.step[1]) / 2.;
        let range = |k: usize, n: usize| {
            let lo = ((p[k] - radius - half - self.min[k]) / self.step[k])
                .floor()
                .max(0.) as usize;
            let hi = ((p[k] + radius + half - self.min[k]) / self.step[k]).floor();
            if hi < 0. {
                return 0..0;
            }
            lo.min(n)..(hi as usize + 1).min(n)
        };
        for j in range(1, self.ny) {
            for i in range(0, self.nx) {
                let k = j * self.nx + i;
                if !self.flags[k] {
                    continue;
                }
                let c = [
                    self.min[0] + (i as f64 + 0.5) * self.step[0],
                    self.min[1] + (j as f64 + 0.5) * self.step[1],
                ];
                let near = ((c[0] - p[0]).hypot(c[1] - p[1]) - half).max(0.);
                if near <= radius
                    && self.lower[k] - (center_z - (radius * radius - near * near).sqrt())
                        > threshold
                {
                    return true;
                }
            }
        }
        false
    }
}
pub(crate) fn is_zero(v: &f64) -> bool {
    *v == 0.
}
pub(crate) fn is_ninety(v: &f64) -> bool {
    *v == 90.
}
/// Validate contact angle limits; Some((from, to)) in radians when limited.
pub(crate) fn slope_limits(what: &str, from: f64, to: f64) -> Result<Option<(f64, f64)>, String> {
    crate::heights::require(
        what,
        [(
            from.is_finite()
                && to.is_finite()
                && (0. ..90.).contains(&from)
                && from < to
                && to <= 90.,
            format!(
                "slope_from_deg {from} and slope_to_deg {to} must satisfy 0 <= from < to <= 90"
            ),
        )],
    )?;
    Ok((from > 0. || to < 90.).then(|| (from.to_radians(), to.to_radians())))
}
/// Split XY passes into the runs whose samples (at most `spacing` apart)
/// satisfy `keep`; each run is cut as its own pass.
pub(crate) fn filter_runs(
    passes: Vec<Vec<[f64; 2]>>,
    spacing: f64,
    mut keep: impl FnMut([f64; 2]) -> Result<bool, String>,
) -> Result<Vec<Vec<[f64; 2]>>, String> {
    let mut samples = 0f64;
    for pass in &passes {
        for w in pass.windows(2) {
            samples += ((w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]) / spacing).ceil() + 1.;
        }
    }
    if !samples.is_finite() || samples > u32::MAX as f64 {
        return Err("pass limits need too many samples".into());
    }
    crate::resources::check_memory(samples as u128 * 16, "limited passes")?;
    let mut out = vec![];
    for pass in passes {
        let mut run: Vec<[f64; 2]> = vec![];
        let mut visit = |p: [f64; 2], out: &mut Vec<Vec<[f64; 2]>>| -> Result<(), String> {
            if keep(p)? {
                if run.last() != Some(&p) {
                    run.push(p);
                }
            } else if run.len() >= 2 {
                out.push(std::mem::take(&mut run));
            } else {
                run.clear();
            }
            Ok(())
        };
        for w in pass.windows(2) {
            let (a, b) = (w[0], w[1]);
            let n = ((b[0] - a[0]).hypot(b[1] - a[1]) / spacing).ceil().max(1.) as usize;
            for k in 0..n {
                let t = k as f64 / n as f64;
                visit(
                    [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])],
                    &mut out,
                )?;
            }
        }
        if let Some(&last) = pass.last() {
            visit(last, &mut out)?;
        }
        if run.len() >= 2 {
            out.push(run);
        }
    }
    Ok(out)
}
impl<G> ParallelOperation<G> {
    pub(crate) fn map_geometry<H, E>(
        self,
        f: &mut impl FnMut(G) -> Result<H, E>,
    ) -> Result<ParallelOperation<H>, E> {
        Ok(ParallelOperation {
            limits: self.limits,
            id: self.id,
            tool: self.tool,
            surface: f(self.surface)?,
            floor_z_mm: self.floor_z_mm,
            stepover_mm: self.stepover_mm,
            clearance_z_mm: self.clearance_z_mm,
            axis: self.axis,
            reverse: self.reverse,
            pattern: self.pattern,
            slope_from_deg: self.slope_from_deg,
            slope_to_deg: self.slope_to_deg,
            rest_threshold_mm: self.rest_threshold_mm,
            stock_to_leave_mm: self.stock_to_leave_mm,
            path_error_mm: self.path_error_mm,
            safety_margin_mm: self.safety_margin_mm,
            entry_air_gap_mm: self.entry_air_gap_mm,
            feed_mm_min: self.feed_mm_min,
            plunge_mm_min: self.plunge_mm_min,
            tolerance_mm: self.tolerance_mm,
            spindle_rpm: self.spindle_rpm,
            work_offset: self.work_offset,
            tool_number: self.tool_number,
            length_register: self.length_register,
            coolant: self.coolant,
            retract_z_mm: self.retract_z_mm,
            feed_z_mm: self.feed_z_mm,
        })
    }
}

#[derive(Serialize)]
pub struct ParallelReport {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stock_top: Option<FinishingStockTop>,
    pub strategy: &'static str,
    pub geometric_residual_status: ResidualStatus,
    pub triangles: usize,
    pub rows: usize,
    pub row_pitch_mm: f64,
    pub rows_completed: usize,
    pub cut_segments: usize,
    pub adaptive_splits: usize,
    pub compacted_vertices: usize,
    pub reused_segments: usize,
    pub vertical_transitions: usize,
    pub transition_moves: usize,
    pub max_transition_height_mm: f64,
    pub path_error_mm: f64,
    pub safety_margin_mm: f64,
    pub stock_to_leave_mm: f64,
    pub surface_error_mm: f64,
    /// Ball end mill: the certified bound; None for a flat end mill.
    pub max_certified_path_excess_mm: Option<f64>,
    /// Flat end mill: the largest sampled chord excess.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_sampled_path_excess_mm: Option<f64>,
    pub flat_reference_scallop_mm: f64,
    pub work: SurfaceWork,
    pub clearance_proof: &'static str,
    pub surface_completion: &'static str,
}
#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase {
    Lift,
    Approach,
    Entry,
    Lead,
    Cut,
    Exit,
    Retract,
    /// Leaving-feed height -> retract height (clearance after the last row).
    Rise,
    /// Travel height -> feed height (or the air entry if higher).
    Descend,
    Done,
}
#[derive(Clone, Copy)]
struct Segment {
    a: Vec3,
    b: Vec3,
    depth: usize,
}
/// A flat or bull-nose end mill over the part, its reference point at the
/// tip: exact contacts with the triangles (the flat-area index).
struct FlatContact {
    index: crate::flat::Index,
    /// Contact cutter (radius and corner plus the offset) and the offset lift.
    radius: f64,
    corner: f64,
    lift: f64,
    /// The no-gouge cutter and lift (without stock to leave).
    guard_radius: f64,
    guard_corner: f64,
    guard_lift: f64,
    /// A flat end mill (no corner): moving contacts are exact capsule maxima.
    flat: bool,
    scratch: std::cell::RefCell<(Vec<u32>, u32)>,
}
impl FlatContact {
    fn max(&self, a: [f64; 2], b: [f64; 2], r: f64) -> f64 {
        let mut s = self.scratch.borrow_mut();
        let (seen, stamp) = &mut *s;
        self.index.max_height(a, b, r, seen, stamp)
    }
    fn bull(&self, xy: [f64; 2], r: f64, rc: f64) -> f64 {
        let mut s = self.scratch.borrow_mut();
        let (seen, stamp) = &mut *s;
        self.index.max_bull(xy, r, rc, seen, stamp)
    }
    /// Tip height resting on the offset part at `xy`.
    fn tip(&self, xy: [f64; 2]) -> f64 {
        if self.flat {
            self.max(xy, xy, self.radius) + self.lift
        } else {
            self.bull(xy, self.radius, self.corner) + self.lift
        }
    }
    /// Proven: the cutter moving straight from tip `a` to tip `b` stays
    /// above the part (per triangle, the exact maximum of the concave
    /// contact-minus-tip difference along the chord).
    fn clear(&self, a: Vec3, b: Vec3) -> bool {
        let mut s = self.scratch.borrow_mut();
        let (seen, stamp) = &mut *s;
        let corner = if self.flat { 0. } else { self.guard_corner };
        self.index.bull_chord_clear(
            a,
            b,
            self.guard_radius,
            corner,
            self.guard_lift,
            seen,
            stamp,
        )
    }
    /// How far the chord rises above the offset surface, sampled at seven
    /// interior points (not a certificate; the gouge check is).
    fn sampled_excess(&self, a: Vec3, b: Vec3) -> f64 {
        (1..8)
            .map(|k| {
                let t = k as f64 / 8.;
                let p = [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
                a[2] + t * (b[2] - a[2]) - self.tip(p)
            })
            .fold(0., f64::max)
    }
}
pub struct ParallelPlan {
    /// Flat end mill contact (None: ball, through `surface`).
    flat: Option<FlatContact>,
    /// Reference point to tip (the ball radius; 0 for a flat end mill).
    tip_offset: f64,
    op: ParallelOperation,
    heights: crate::heights::Heights,
    spec: StockSpec,
    surface: PreparedSurface,
    initial: Vec3,
    position: Vec3,
    radius: f64,
    inflated: f64,
    protected: f64,
    axis: usize,
    low: f64,
    high: f64,
    row: usize,
    start: Vec3,
    end: Vec3,
    air_start: Vec3,
    air_end: Vec3,
    stack: Vec<Segment>,
    /// Current pass, lifted onto the surface; `cursor` is the next chord.
    vertices: Vec<Vec3>,
    cursor: usize,
    /// XY polylines of a non-raster pattern (entered by a vertical plunge).
    passes: Vec<Pass>,
    transition: Vec<(Vec3, MoveKind)>,
    prepared_segment: Option<(Segment, Option<f64>)>,
    phase: Phase,
    next_id: u64,
    expected_generation: Option<u64>,
    drained: bool,
    report: ParallelReport,
    stock_ceiling: Option<StockCeiling>,
    /// When the remaining stock stands higher above the floor than the
    /// cutting length: the tool parts above the flutes (neck, shank, holder
    /// sections) as (lowest height above the tip, largest radius), each
    /// checked against the stock under it on every feed move (the depth the
    /// pass actually cuts, not the stock top over the floor).
    depth_parts: Option<Vec<(f64, f64)>>,
}
impl ParallelPlan {
    pub fn prepare_with_stock(
        op: ParallelOperation,
        tool: &ToolAssembly,
        stock: &Stock,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        if stock.spec().max_mm[2] - op.floor_z_mm
            < tool.cutter.cutting_length_mm - 16. * op.tolerance_mm
        {
            return Self::prepare_inner(
                op,
                tool,
                stock.spec(),
                Some(stock),
                initial,
                first_id,
                cancel,
            );
        }
        let (envelope, ceiling) = StockCeiling::prepare(stock, cancel)?;
        // Stock standing higher above the floor than the cutting length need
        // not be cut that deep: check each move's actual depth instead.
        let per_move = envelope.max_mm[2] - op.floor_z_mm
            >= tool.cutter.cutting_length_mm - 16. * op.tolerance_mm;
        let mut plan = Self::prepare_inner_depth(
            op,
            tool,
            &envelope,
            Some(stock),
            initial,
            first_id,
            per_move,
            cancel,
        )?;
        if per_move {
            let prepared = tool.prepare()?;
            plan.depth_parts = Some(
                crate::verification::parts(&prepared)?
                    .into_iter()
                    .filter(|p| p.kind != crate::verification::ToolPart::Cutter)
                    .map(|p| (p.min, p.radius.at(p.min).max(p.radius.at(p.max))))
                    .collect(),
            );
        }
        plan.spec = stock.spec().clone();
        plan.expected_generation = Some(stock.generation());
        plan.report.stock_top = Some(ceiling.report.clone());
        plan.report.clearance_proof = if plan.depth_parts.is_some() {
            "continuous segment distance to downward-extruded triangles and base; inflated analytic ball contact; neck/shank/holder above the committed stock under them on every feed move"
        } else {
            "continuous segment distance to downward-extruded triangles and base; inflated analytic ball contact; shank/holder above admitted committed stock ceiling"
        };
        plan.stock_ceiling = Some(ceiling);
        Ok(plan)
    }
    pub fn prepare(
        op: ParallelOperation,
        tool: &ToolAssembly,
        spec: &StockSpec,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        Self::prepare_inner(op, tool, spec, None, initial, first_id, cancel)
    }
    fn prepare_inner(
        op: ParallelOperation,
        tool: &ToolAssembly,
        spec: &StockSpec,
        stock: Option<&Stock>,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        Self::prepare_inner_depth(op, tool, spec, stock, initial, first_id, false, cancel)
    }
    /// `per_move_depth`: the depth below the stock top is checked on every
    /// move against the committed stock (`depth_parts`), not here.
    #[allow(clippy::too_many_arguments)]
    fn prepare_inner_depth(
        op: ParallelOperation,
        tool: &ToolAssembly,
        spec: &StockSpec,
        stock: Option<&Stock>,
        initial: Vec3,
        first_id: u64,
        per_move_depth: bool,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        if cancel.is_cancelled() {
            return Err("parallel planning cancelled".into());
        }
        spec.grid()?;
        let prepared = tool.prepare()?;
        let cutter = prepared.cutter().definition();
        let radius = cutter.diameter_mm / 2.;
        let guard = 16. * op.tolerance_mm;
        // Kernel coordinates at a coincident stock face can differ by a few
        // floating-point ulps. Entry reach already includes the larger guard;
        // this admission allowance does not change the protected target.
        let coordinate_guard = 128.
            * f64::EPSILON
            * spec
                .min_mm
                .iter()
                .chain(&spec.max_mm)
                .map(|x| x.abs())
                .fold(1., f64::max);
        let values = [
            op.floor_z_mm,
            op.stepover_mm,
            op.clearance_z_mm,
            op.stock_to_leave_mm,
            op.path_error_mm,
            op.safety_margin_mm,
            op.entry_air_gap_mm,
            op.feed_mm_min,
            op.plunge_mm_min,
            op.tolerance_mm,
            op.spindle_rpm,
        ];
        crate::heights::require(
            "parallel finishing",
            [
                (
                    valid_id(&op.id),
                    format!("id {:?} is not a valid identifier", op.id),
                ),
                (
                    op.tool == tool.id,
                    format!(
                        "tool {:?} does not match the assembly {:?}",
                        op.tool, tool.id
                    ),
                ),
                (
                    matches!(
                        cutter.shape,
                        CutterShape::Ball | CutterShape::Flat | CutterShape::Bull { .. }
                    ) && cutter.center_cutting,
                    "this operation needs a center-cutting ball, flat or bull-nose end mill".into(),
                ),
                (
                    !matches!(cutter.shape, CutterShape::Flat | CutterShape::Bull { .. })
                        || (op.slope_from_deg == 0.
                            && op.slope_to_deg == 90.
                            && op.rest_threshold_mm.is_none()
                            && !matches!(
                                op.pattern,
                                FinishPattern::Pencil { .. } | FinishPattern::Scallop
                            )),
                    "slope limits, rest finishing, pencil and scallop passes need a ball end mill (a flat end mill has no single contact normal)".into(),
                ),
                (
                    values.iter().all(|x| x.is_finite()),
                    "all numeric parameters must be finite".into(),
                ),
                (
                    (1e-7..=0.001).contains(&op.tolerance_mm),
                    format!(
                        "tolerance_mm {} must be within 1e-7..0.001",
                        op.tolerance_mm
                    ),
                ),
                (
                    (0.0001..=0.1).contains(&op.path_error_mm) && op.path_error_mm > guard,
                    format!(
                        "path_error_mm {} must be within 0.0001..0.1",
                        op.path_error_mm
                    ),
                ),
                (
                    (0.001..=0.1).contains(&op.safety_margin_mm)
                        && op.safety_margin_mm > 4. * guard,
                    format!(
                        "safety_margin_mm {} must be within 0.001..0.1",
                        op.safety_margin_mm
                    ),
                ),
                (
                    (0. ..=radius).contains(&op.stock_to_leave_mm),
                    format!(
                        "stock_to_leave_mm {} must be between 0 and the cutter radius {radius}",
                        op.stock_to_leave_mm
                    ),
                ),
                (
                    op.floor_z_mm >= spec.min_mm[2] && op.floor_z_mm < spec.max_mm[2],
                    format!(
                        "floor_z_mm {} must be within the stock Z range {}..{}",
                        op.floor_z_mm, spec.min_mm[2], spec.max_mm[2]
                    ),
                ),
                (
                    per_move_depth || spec.max_mm[2] - op.floor_z_mm < cutter.cutting_length_mm - guard,
                    format!(
                        "depth {} below the stock top exceeds the cutting length {}",
                        spec.max_mm[2] - op.floor_z_mm,
                        cutter.cutting_length_mm
                    ),
                ),
                if matches!(
                    op.pattern,
                    FinishPattern::Raster
                        | FinishPattern::Spiral { .. }
                        | FinishPattern::Scallop
                        | FinishPattern::Morph { .. }
                ) {
                    (
                        op.stepover_mm > guard && op.stepover_mm <= 2. * (radius - guard),
                        format!(
                            "stepover_mm {} must be positive and less than the cutter diameter {}",
                            op.stepover_mm,
                            2. * radius
                        ),
                    )
                } else {
                    (
                        op.stepover_mm == 0.,
                        format!(
                            "stepover_mm {} is not used by the {} pattern; leave it 0",
                            op.stepover_mm,
                            op.pattern.name()
                        ),
                    )
                },
                (
                    op.pattern.is_raster() || !op.reverse,
                    format!(
                        "reverse applies to the raster pattern only; the {} pattern has its own direction",
                        op.pattern.name()
                    ),
                ),
                (
                    op.entry_air_gap_mm >= guard,
                    format!("entry_air_gap_mm {} must be positive", op.entry_air_gap_mm),
                ),
                (
                    op.clearance_z_mm
                        > spec.max_mm[2]
                            + op.surface.surface_error_mm
                            + op.stock_to_leave_mm
                            + op.safety_margin_mm
                        && op.clearance_z_mm.abs() <= 1e6,
                    format!(
                        "clearance_z_mm {} must be above the stock top {} (plus surface error, stock to leave and margin)",
                        op.clearance_z_mm, spec.max_mm[2]
                    ),
                ),
                (
                    initial.iter().all(|v| v.is_finite() && v.abs() <= 1e6),
                    "start position must be finite".into(),
                ),
                (
                    initial[2] >= op.clearance_z_mm,
                    format!(
                        "start Z {} is below clearance_z_mm {}",
                        initial[2], op.clearance_z_mm
                    ),
                ),
                (
                    op.feed_mm_min > 0. && op.plunge_mm_min > 0. && op.spindle_rpm > 0.,
                    format!(
                        "feed_mm_min {}, plunge_mm_min {} and spindle_rpm {} must be positive",
                        op.feed_mm_min, op.plunge_mm_min, op.spindle_rpm
                    ),
                ),
                (
                    (1..=9).contains(&op.work_offset),
                    format!("work_offset {} must be 1..9", op.work_offset),
                ),
                (
                    op.tool_number > 0 && op.length_register > 0,
                    "tool_number and length_register must be positive".into(),
                ),
                (
                    first_id != 0 && first_id.checked_add(1).is_some(),
                    "motion ID budget exhausted".into(),
                ),
            ],
        )?;
        let clipped = if op
            .surface
            .triangles
            .iter()
            .flatten()
            .any(|p| p[2] < op.floor_z_mm)
        {
            Some(
                op.surface
                    .above_floor(op.floor_z_mm, op.tolerance_mm, cancel)?,
            )
        } else {
            None
        };
        let mesh = clipped.as_ref().unwrap_or(&op.surface);
        let surface = mesh.prepare(op.floor_z_mm, op.tolerance_mm, cancel)?;
        if surface.max_z() > spec.max_mm[2] + coordinate_guard {
            return Err("surface target exceeds initial stock top".into());
        }
        let limits = slope_limits("parallel finishing", op.slope_from_deg, op.slope_to_deg)?;
        let (mut passes, spoke_pitch) = Self::passes(&op, spec, radius)?;
        let axis = if op.axis == TraverseAxis::X { 0 } else { 1 };
        let cross = 1 - axis;
        // Scan the rounded rectangle expanded by the actual ball radius so
        // sloped edge contacts can lie outside the model's XY footprint.
        let span = spec.max_mm[cross] - spec.min_mm[cross] + 2. * radius;
        let (rows, pitch) = match &op.pattern {
            FinishPattern::Raster => {
                let intervals = (span / op.stepover_mm).ceil() as usize;
                if intervals == 0 {
                    return Err("parallel finishing needs at least one row".into());
                }
                (intervals + 1, span / intervals as f64)
            }
            FinishPattern::Spiral { .. } => (passes.len(), op.stepover_mm),
            FinishPattern::Radial { .. } => (passes.len(), spoke_pitch),
            FinishPattern::Project { .. } | FinishPattern::Pencil { .. } => (passes.len(), 0.),
            FinishPattern::Morph { .. } => (passes.len(), op.stepover_mm),
            FinishPattern::Scallop => (passes.len(), op.stepover_mm),
        };
        let inflated =
            radius + op.stock_to_leave_mm + op.surface.surface_error_mm + op.safety_margin_mm;
        let mut slope_work = SurfaceWork::default();
        let corner = match cutter.shape {
            CutterShape::Flat => Some(0.),
            CutterShape::Bull { corner_radius_mm } => Some(corner_radius_mm),
            _ => None,
        };
        let flat = if let Some(corner) = corner {
            let index = crate::flat::Index::new(mesh, op.floor_z_mm, radius, cancel)?;
            // The offset cutter (grown by the offset in every direction)
            // has its radius and corner grown, and its tip lowered, by it.
            let offset = inflated - radius;
            let guard_offset = op.surface.surface_error_mm + 2. * guard;
            let grow = |c: f64, by: f64| if c == 0. { 0. } else { c + by };
            Some(FlatContact {
                scratch: std::cell::RefCell::new((vec![0; index.len()], 0)),
                index,
                radius: inflated,
                corner: grow(corner, offset),
                lift: offset,
                guard_radius: radius + guard_offset,
                guard_corner: grow(corner, guard_offset),
                guard_lift: guard_offset,
                flat: corner == 0.,
            })
        } else {
            None
        };
        if matches!(op.pattern, FinishPattern::Scallop) {
            let spacing = (op.stepover_mm / 4.)
                .min(radius / 4.)
                .min((8. * radius * op.path_error_mm).sqrt());
            passes = scallop::passes(
                &surface,
                [spec.min_mm[0] - radius, spec.min_mm[1] - radius],
                [spec.max_mm[0] + radius, spec.max_mm[1] + radius],
                spacing,
                op.stepover_mm,
                inflated,
                &mut slope_work,
                cancel,
            )?;
        }
        if let FinishPattern::Pencil {
            min_angle_deg,
            sample_mm,
        } = op.pattern
        {
            // Default: a quarter radius, and fine enough that chords sag
            // no more than the path error.
            let spacing =
                sample_mm.unwrap_or((radius / 4.).min((8. * radius * op.path_error_mm).sqrt()));
            crate::heights::require(
                "pencil",
                [
                    (
                        min_angle_deg.is_finite() && (1. ..=90.).contains(&min_angle_deg),
                        format!("min_angle_deg {min_angle_deg} must be within 1..90"),
                    ),
                    (
                        spacing.is_finite() && spacing > guard && spacing <= radius,
                        format!(
                            "sample_mm {spacing} must be positive and at most the tool radius {radius}"
                        ),
                    ),
                ],
            )?;
            let lo = [spec.min_mm[0] - radius, spec.min_mm[1] - radius];
            let hi = [spec.max_mm[0] + radius, spec.max_mm[1] + radius];
            passes = pencil::creases(
                &surface,
                lo,
                hi,
                spacing,
                inflated,
                min_angle_deg.to_radians(),
                op.path_error_mm,
                &mut slope_work,
                cancel,
            )?
            .into_iter()
            // Shorter than the tool radius: a corner point, not a crease.
            .filter(|c| {
                c.windows(2)
                    .map(|w| (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]))
                    .sum::<f64>()
                    >= radius
            })
            .collect();
        }
        let rest = match op.rest_threshold_mm {
            None => None,
            Some(threshold) => {
                crate::heights::require(
                    "rest finishing",
                    [
                        (
                            threshold.is_finite() && threshold > guard && threshold <= 10.,
                            format!("rest_threshold_mm {threshold} must be positive"),
                        ),
                        (
                            stock.is_some(),
                            "rest finishing needs the stock left by the preceding operations"
                                .into(),
                        ),
                    ],
                )?;
                let target = crate::flat::Index::new(mesh, op.floor_z_mm, radius, cancel)?;
                Some(RestMap::new(
                    &target,
                    stock.unwrap(),
                    op.stock_to_leave_mm + op.surface.surface_error_mm,
                    threshold,
                    cancel,
                )?)
            }
        };
        let region_limits = op
            .limits
            .as_ref()
            .filter(|l| !l.is_empty())
            .map(|l| crate::limits::PreparedLimits::new(l, radius, op.tolerance_mm))
            .transpose()?;
        let rows = if limits.is_some() || rest.is_some() || region_limits.is_some() {
            if op.pattern.is_raster() {
                passes = (0..rows)
                    .map(|row| Self::raster_row(&op, spec, radius, axis, rows, pitch, row))
                    .collect();
            }
            let spacing = if op.stepover_mm > 0. {
                (op.stepover_mm / 2.).min(radius / 4.)
            } else {
                radius / 4.
            };
            let guard_angle = 1e-9;
            passes = filter_runs(passes, spacing, |p| {
                if let Some(f) = &flat {
                    // Flat end mill: only the limits apply (checked above).
                    return region_limits.as_ref().map_or(Ok(true), |l| {
                        l.margin(p, f.tip(p) - spacing, true)
                            .map(|m| m >= spacing / 2.)
                    });
                }
                let here = surface.contact(p, inflated, &mut slope_work, cancel)?;
                if let (Some(map), Some(threshold)) = (&rest, op.rest_threshold_mm)
                    && !map.engaged(p, here.center_z, radius, threshold)
                {
                    return Ok(false);
                }
                if let Some((lo, hi)) = limits {
                    let angle = here.contact_angle(inflated);
                    if angle < lo - guard_angle || angle > hi + guard_angle {
                        return Ok(false);
                    }
                }
                // The cutter disk here, and to the next sample (half a
                // spacing either way), is inside the limits; its tip may dip
                // up to a spacing lower between samples.
                if let Some(l) = &region_limits
                    && l.margin(p, here.center_z - radius - spacing, true)? < spacing / 2.
                {
                    return Ok(false);
                }
                Ok(true)
            })?;
            if passes.is_empty() && rest.is_some() {
                return Err(
                    "rest finishing found no stock above the target by more than the threshold"
                        .into(),
                );
            }
            passes.len()
        } else if matches!(
            op.pattern,
            FinishPattern::Pencil { .. } | FinishPattern::Scallop
        ) {
            if passes.is_empty() {
                return Err(match op.pattern {
                    FinishPattern::Pencil { .. } => {
                        "pencil finishing found no concave crease for this tool".into()
                    }
                    _ => format!(
                        "scallop finishing found no pass: stepover_mm {} spans the whole region",
                        op.stepover_mm
                    ),
                });
            }
            passes.len()
        } else {
            rows
        };
        if rows == 0 {
            return Err(format!(
                "the {} pattern has no pass over the stock",
                op.pattern.name()
            ));
        }
        let protected = inflated - op.safety_margin_mm + guard;
        let low = spec.min_mm[axis] - inflated - op.entry_air_gap_mm - guard;
        let high = spec.max_mm[axis] + inflated + op.entry_air_gap_mm + guard;
        // Include all protected geometry in entry reach, not just stock bounds.
        if op.surface.triangles.iter().flatten().any(|p| {
            p[0] < spec.min_mm[0] - coordinate_guard
                || p[0] > spec.max_mm[0] + coordinate_guard
                || p[1] < spec.min_mm[1] - coordinate_guard
                || p[1] > spec.max_mm[1] + coordinate_guard
        }) || low.abs() > 1e6
            || high.abs() > 1e6
        {
            return Err("parallel target must lie within initial stock XY".into());
        }
        let report = ParallelReport {
            stock_top: None,
            strategy: op.pattern.name(),
            geometric_residual_status: ResidualStatus::Pending,
            triangles: mesh.triangles.len(),
            rows,
            row_pitch_mm: pitch,
            rows_completed: 0,
            cut_segments: 0,
            adaptive_splits: 0,
            compacted_vertices: 0,
            reused_segments: 0,
            vertical_transitions: 0,
            transition_moves: 0,
            max_transition_height_mm: 0.,
            path_error_mm: op.path_error_mm,
            safety_margin_mm: op.safety_margin_mm,
            stock_to_leave_mm: op.stock_to_leave_mm,
            surface_error_mm: op.surface.surface_error_mm,
            max_certified_path_excess_mm: matches!(cutter.shape, CutterShape::Ball).then_some(0.),
            max_sampled_path_excess_mm: (!matches!(cutter.shape, CutterShape::Ball)).then_some(0.),
            flat_reference_scallop_mm: radius
                - (radius * radius - (pitch / 2.).powi(2)).max(0.).sqrt(),
            work: slope_work,
            clearance_proof: "continuous segment distance to downward-extruded triangles and base; inflated analytic ball contact; shank/holder above initial stock",
            surface_completion: "not_certified; path excess is bounded, whole-target scallop/unreachable regions require independent stock checks",
        };
        let heights = crate::heights::Heights::resolve(
            op.clearance_z_mm,
            op.retract_z_mm,
            op.feed_z_mm,
            spec.max_mm[2],
            16. * op.tolerance_mm,
        )?;
        let tip_offset = if flat.is_some() { 0. } else { radius };
        Ok(Self {
            flat,
            tip_offset,
            heights,
            initial,
            position: initial,
            radius,
            inflated,
            protected,
            axis,
            low,
            high,
            row: 0,
            start: initial,
            end: initial,
            air_start: initial,
            air_end: initial,
            stack: vec![],
            vertices: vec![],
            cursor: 0,
            passes,
            transition: vec![],
            prepared_segment: None,
            phase: Phase::Lift,
            next_id: first_id,
            expected_generation: None,
            drained: false,
            op,
            spec: spec.clone(),
            surface,
            report,
            stock_ceiling: None,
            depth_parts: None,
        })
    }
    pub fn report(&self) -> &ParallelReport {
        &self.report
    }
    pub fn prefix_events(&self) -> Vec<Event> {
        vec![
            Event::WorkOffset {
                number: self.op.work_offset,
            },
            Event::ToolChange {
                tool: self.op.tool.clone(),
                number: self.op.tool_number,
                length_register: self.op.length_register,
                position_after_mm: self.initial,
            },
            Event::Spindle {
                rpm: self.op.spindle_rpm,
                direction: SpindleDirection::Cw,
            },
            Event::Coolant {
                mode: self.op.coolant,
            },
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
    pub fn finish_report(&mut self, cancel: &impl CancelSignal) -> Result<(), String> {
        if cancel.is_cancelled() {
            return Err("parallel report cancelled".into());
        }
        if !self.drained {
            return Err("drain committed parallel motions before final report".into());
        }
        self.report.geometric_residual_status = ResidualStatus::Unknown;
        Ok(())
    }
    /// XY passes of a non-raster pattern, clipped to the stock grown by the
    /// cutter radius (beyond it the cutter cannot touch stock).
    fn passes(
        op: &ParallelOperation,
        spec: &StockSpec,
        radius: f64,
    ) -> Result<(Vec<Pass>, f64), String> {
        let lo = [spec.min_mm[0] - radius, spec.min_mm[1] - radius];
        let hi = [spec.max_mm[0] + radius, spec.max_mm[1] + radius];
        let sagitta = op.path_error_mm;
        let guard = 16. * op.tolerance_mm;
        let finite = |v: &[f64]| v.iter().all(|x| x.is_finite() && x.abs() <= 1e6);
        let radii = |center: [f64; 2], inner: f64, outer: Option<f64>| {
            let outer = outer.unwrap_or_else(|| patterns::reach(center, lo, hi));
            crate::heights::require(
                op.pattern.name(),
                [
                    (finite(&center), "center_mm must be finite".into()),
                    (
                        inner.is_finite() && inner >= 0.,
                        format!("inner_radius_mm {inner} must be 0 or more"),
                    ),
                    (
                        outer.is_finite() && outer <= 1e6 && outer > inner + guard,
                        format!(
                            "outer_radius_mm {outer} must be finite and greater than inner_radius_mm {inner}"
                        ),
                    ),
                ],
            )?;
            Ok::<_, String>(outer)
        };
        let mut spoke_pitch = 0.;
        let passes = match &op.pattern {
            // Pencil passes need the prepared surface: found in prepare.
            FinishPattern::Raster | FinishPattern::Pencil { .. } | FinishPattern::Scallop => vec![],
            &FinishPattern::Spiral {
                center_mm,
                inner_radius_mm,
                outer_radius_mm,
                outward,
                clockwise,
            } => {
                let outer = radii(center_mm, inner_radius_mm, outer_radius_mm)?;
                let points = patterns::spiral(
                    center_mm,
                    inner_radius_mm,
                    outer,
                    op.stepover_mm,
                    outward,
                    clockwise,
                    sagitta,
                )?;
                patterns::clip(&points, lo, hi)
            }
            &FinishPattern::Radial {
                center_mm,
                angular_step_deg,
                inner_radius_mm,
                outer_radius_mm,
                start_angle_deg,
                alternate,
            } => {
                let outer = radii(center_mm, inner_radius_mm, outer_radius_mm)?;
                crate::heights::require(
                    "radial",
                    [
                        (
                            angular_step_deg > 0. && angular_step_deg <= 180.,
                            format!("angular_step_deg {angular_step_deg} must be within (0, 180]"),
                        ),
                        (
                            start_angle_deg.is_finite(),
                            "start_angle_deg must be finite".into(),
                        ),
                    ],
                )?;
                let spokes = (360. / angular_step_deg).ceil();
                crate::resources::check_memory(spokes as u128 * 64, "radial spokes")?;
                // Widest spacing, at the outer end of the spokes.
                spoke_pitch = outer * (360. / spokes).to_radians();
                patterns::radial(
                    center_mm,
                    inner_radius_mm,
                    outer,
                    angular_step_deg.to_radians(),
                    start_angle_deg.to_radians(),
                    alternate,
                )
                .iter()
                .flat_map(|spoke| patterns::clip(spoke, lo, hi))
                .collect()
            }
            FinishPattern::Morph { from, to } => {
                let a = patterns::flatten(from, 0, sagitta, guard)
                    .map_err(|e| format!("invalid morph parameters: {e}"))?;
                let b = patterns::flatten(to, 1, sagitta, guard)
                    .map_err(|e| format!("invalid morph parameters: {e}"))?;
                let spacing = (op.stepover_mm / 2.).min(radius / 4.).max(16. * guard);
                patterns::morph(&a, &b, op.stepover_mm, spacing)
                    .iter()
                    .flat_map(|pass| patterns::clip(pass, lo, hi))
                    .collect()
            }
            FinishPattern::Project { paths } => {
                if paths.is_empty() {
                    return Err("invalid project parameters: needs at least one path".into());
                }
                let mut passes = Vec::with_capacity(paths.len());
                for (i, path) in paths.iter().enumerate() {
                    let points = patterns::flatten(path, i, sagitta, guard)
                        .map_err(|e| format!("invalid project parameters: {e}"))?;
                    if let Some(p) = points.iter().find(|p| {
                        (0..2)
                            .any(|k| p[k] < spec.min_mm[k] - guard || p[k] > spec.max_mm[k] + guard)
                    }) {
                        return Err(format!(
                            "invalid project parameters: path {i} point {p:?} is outside the stock XY"
                        ));
                    }
                    passes.push(points);
                }
                passes
            }
        };
        Ok((passes, spoke_pitch))
    }
    /// XY extent of raster row `row` (as setup_row), for slope-limited passes.
    fn raster_row(
        op: &ParallelOperation,
        spec: &StockSpec,
        radius: f64,
        axis: usize,
        rows: usize,
        pitch: f64,
        row: usize,
    ) -> Vec<[f64; 2]> {
        let cross = 1 - axis;
        let reverse = op.reverse ^ (row % 2 == 1);
        let c = if row + 1 == rows {
            spec.max_mm[cross] + radius
        } else {
            spec.min_mm[cross] - radius + row as f64 * pitch
        };
        let outside = (spec.min_mm[cross] - c).max(c - spec.max_mm[cross]).max(0.);
        let extension = (radius * radius - outside * outside).max(0.).sqrt();
        let (lo, hi) = (spec.min_mm[axis] - extension, spec.max_mm[axis] + extension);
        let point = |along: f64| {
            let mut p = [0.; 2];
            p[axis] = along;
            p[cross] = c;
            p
        };
        if reverse {
            vec![point(hi), point(lo)]
        } else {
            vec![point(lo), point(hi)]
        }
    }
    fn setup_row(&mut self, cancel: &impl CancelSignal) -> Result<(), String> {
        if !self.passes.is_empty() {
            return self.setup_pass(cancel);
        }
        let mut a = [0.; 3];
        let mut b = [0.; 3];
        let cross = 1 - self.axis;
        let reverse = self.op.reverse ^ (self.row % 2 == 1);
        a[cross] = if self.row + 1 == self.report.rows {
            self.spec.max_mm[cross] + self.radius
        } else {
            self.spec.min_mm[cross] - self.radius + self.row as f64 * self.report.row_pitch_mm
        };
        b[cross] = a[cross];
        let outside = (self.spec.min_mm[cross] - a[cross])
            .max(a[cross] - self.spec.max_mm[cross])
            .max(0.);
        let extension = (self.radius * self.radius - outside * outside)
            .max(0.)
            .sqrt();
        a[self.axis] = if reverse {
            self.spec.max_mm[self.axis] + extension
        } else {
            self.spec.min_mm[self.axis] - extension
        };
        b[self.axis] = if reverse {
            self.spec.min_mm[self.axis] - extension
        } else {
            self.spec.max_mm[self.axis] + extension
        };
        let mut work = self.report.work;
        a[2] = self.contact_z([a[0], a[1]], &mut work, cancel)?;
        b[2] = self.contact_z([b[0], b[1]], &mut work, cancel)?;
        let mut air_start = a;
        let mut air_end = b;
        air_start[self.axis] = if reverse { self.high } else { self.low };
        air_end[self.axis] = if reverse { self.low } else { self.high };
        if !self.clear(air_start, a, &mut work, cancel)?
            || !self.clear(b, air_end, &mut work, cancel)?
        {
            return Err("parallel exterior entry/exit is not continuously clear".into());
        }
        self.air_start = air_start;
        self.air_end = air_end;
        self.start = a;
        self.end = b;
        self.vertices = vec![a, b];
        self.cursor = 0;
        self.stack.clear();
        self.prepared_segment = None;
        self.report.work = work;
        Ok(())
    }
    /// A pattern pass is entered and left vertically. Above a contact the
    /// cutter only moves away from the protected solid, which is extruded
    /// downward, so those vertical moves need no further clearance proof.
    fn setup_pass(&mut self, cancel: &impl CancelSignal) -> Result<(), String> {
        let mut work = self.report.work;
        let pass = &self.passes[self.row];
        let mut vertices = Vec::with_capacity(pass.len());
        for p in pass {
            let z = self.contact_z(*p, &mut work, cancel)?;
            vertices.push([p[0], p[1], z]);
        }
        let (a, b) = (vertices[0], *vertices.last().unwrap());
        self.air_start = a;
        self.air_end = b;
        self.start = a;
        self.end = b;
        self.vertices = vertices;
        self.cursor = 0;
        self.stack.clear();
        self.prepared_segment = None;
        self.report.work = work;
        Ok(())
    }
    /// The tool's reference height resting on the offset part at `xy`.
    fn contact_z(
        &self,
        xy: [f64; 2],
        work: &mut SurfaceWork,
        cancel: &impl CancelSignal,
    ) -> Result<f64, String> {
        match &self.flat {
            Some(f) => Ok(f.tip(xy)),
            None => Ok(self
                .surface
                .contact(xy, self.inflated, work, cancel)?
                .center_z),
        }
    }
    /// Proven: the move between reference points `a` and `b` does not gouge.
    fn clear(
        &self,
        a: Vec3,
        b: Vec3,
        work: &mut SurfaceWork,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        match &self.flat {
            Some(f) => {
                if cancel.is_cancelled() {
                    return Err("parallel planning cancelled".into());
                }
                Ok(f.clear(a, b))
            }
            None => self
                .surface
                .segment_clear(a, b, self.protected, work, cancel),
        }
    }
    /// The reference height at the chord's middle, and the chord's excess
    /// over the offset surface (certified for a ball, sampled for a flat).
    fn mid_excess(
        &self,
        a: Vec3,
        b: Vec3,
        work: &mut SurfaceWork,
        cancel: &impl CancelSignal,
    ) -> Result<(f64, Option<f64>), String> {
        let xy = [(a[0] + b[0]) / 2., (a[1] + b[1]) / 2.];
        if let Some(f) = &self.flat {
            return Ok((f.tip(xy), Some(f.sampled_excess(a, b))));
        }
        let contact = self.surface.contact(xy, self.inflated, work, cancel)?;
        Ok((
            contact.center_z,
            self.path_excess(a, b, contact, work, cancel)?,
        ))
    }
    fn path_excess(
        &self,
        a: Vec3,
        b: Vec3,
        contact: crate::surface::Contact,
        work: &mut SurfaceWork,
        cancel: &impl CancelSignal,
    ) -> Result<Option<f64>, String> {
        let original = self.surface.excess_bound(a, b, contact, self.inflated);
        if original.is_some_and(|e| e <= self.op.path_error_mm) {
            return Ok(original);
        }
        Ok(self
            .surface
            .horizontal_excess_bound(a, b, self.inflated, self.op.path_error_mm, work, cancel)?
            .or(original))
    }
    fn transition_corner(s: Segment) -> Vec3 {
        if s.b[2] > s.a[2] {
            [s.a[0], s.a[1], s.b[2]]
        } else {
            [s.b[0], s.b[1], s.a[2]]
        }
    }
    fn next_cut(&mut self, cancel: &impl CancelSignal) -> Result<Option<(Vec3, MoveKind)>, String> {
        if cancel.is_cancelled() {
            return Err("parallel planning cancelled".into());
        }
        if let Some(next) = self.transition.pop() {
            return Ok(Some(next));
        }
        // Transactional local stack/work: cancellation cannot consume a segment.
        let mut stack = self.stack.clone();
        let mut cursor = self.cursor;
        let mut work = self.report.work;
        let mut splits = 0;
        let reused = self.prepared_segment.is_some();
        let first = if let Some(prepared) = self.prepared_segment {
            Some(prepared)
        } else {
            self.next_segment(&mut stack, &mut cursor, &mut work, &mut splits, cancel)?
        };
        let Some((first, error)) = first else {
            return Ok(None);
        };
        let Some(mut error) = error else {
            // A vertical offset-graph cliff needs an entry/exit, not a false
            // finishing-error certificate. Both complete legs were checked
            // against the original protected solid in next_segment.
            if cancel.is_cancelled() {
                return Err("parallel planning cancelled".into());
            }
            let mut corner = Self::transition_corner(first);
            let mut end = first.b;
            let ascending = end[2] > first.a[2];
            corner[2] -= self.tip_offset;
            end[2] -= self.tip_offset;
            self.transition.push((
                end,
                if ascending {
                    MoveKind::Lead
                } else {
                    MoveKind::Plunge
                },
            ));
            self.stack = stack;
            self.cursor = cursor;
            self.prepared_segment = None;
            self.report.reused_segments += usize::from(reused);
            self.report.work = work;
            self.report.adaptive_splits += splits;
            self.report.vertical_transitions += 1;
            self.report.max_transition_height_mm = self
                .report
                .max_transition_height_mm
                .max((first.b[2] - first.a[2]).abs());
            return Ok(Some((
                corner,
                if ascending {
                    MoveKind::Retract
                } else {
                    MoveKind::Lead
                },
            )));
        };
        let mut end = first.b;
        let mut compacted = 0;
        let mut prepared = None;
        // Greedy bounded lookahead removes subdivision artifacts only after
        // proving the replacement against the original protected solid and
        // original contact graph. No endpoint-only path simplification.
        for _ in 0..16 {
            let mut candidate = stack.clone();
            let mut candidate_cursor = cursor;
            let Some((next, next_error)) = self.next_segment(
                &mut candidate,
                &mut candidate_cursor,
                &mut work,
                &mut splits,
                cancel,
            )?
            else {
                break;
            };
            // The next segment and its complete clearance/error certificate
            // are independent of a proposed merge. Retain them if merging
            // fails, with the corresponding remaining stack. Commit only after
            // the final cancellation check, just like emitted geometry.
            prepared = Some((next, next_error));
            stack = candidate;
            cursor = candidate_cursor;
            if next_error.is_none() {
                break;
            }
            let (_, bound) = self.mid_excess(first.a, next.b, &mut work, cancel)?;
            let Some(bound) = bound.filter(|b| *b <= self.op.path_error_mm) else {
                break;
            };
            if !self.clear(first.a, next.b, &mut work, cancel)? {
                break;
            }
            prepared = None;
            end = next.b;
            error = error.max(bound);
            compacted += 1;
        }
        if cancel.is_cancelled() {
            return Err("parallel planning cancelled".into());
        }
        self.stack = stack;
        self.cursor = cursor;
        self.prepared_segment = prepared;
        self.report.reused_segments += usize::from(reused);
        self.report.work = work;
        self.report.adaptive_splits += splits;
        self.report.compacted_vertices += compacted;
        if self.flat.is_some() {
            self.report.max_sampled_path_excess_mm = Some(
                self.report
                    .max_sampled_path_excess_mm
                    .unwrap_or(0.)
                    .max(error),
            );
        } else {
            self.report.max_certified_path_excess_mm = Some(
                self.report
                    .max_certified_path_excess_mm
                    .unwrap_or(0.)
                    .max(error),
            );
        }
        end[2] -= self.tip_offset;
        Ok(Some((end, MoveKind::Cut)))
    }
    fn next_segment(
        &self,
        stack: &mut Vec<Segment>,
        cursor: &mut usize,
        work: &mut SurfaceWork,
        splits: &mut usize,
        cancel: &impl CancelSignal,
    ) -> Result<Option<(Segment, Option<f64>)>, String> {
        loop {
            let s = match stack.pop() {
                Some(s) => s,
                None if *cursor + 1 < self.vertices.len() => {
                    *cursor += 1;
                    Segment {
                        a: self.vertices[*cursor - 1],
                        b: self.vertices[*cursor],
                        depth: 0,
                    }
                }
                None => return Ok(None),
            };
            let xy = [(s.a[0] + s.b[0]) / 2., (s.a[1] + s.b[1]) / 2.];
            let (mid_z, excess) = self.mid_excess(s.a, s.b, work, cancel)?;
            if let Some(error) = excess.filter(|e| *e <= self.op.path_error_mm)
                && self.clear(s.a, s.b, work, cancel)?
            {
                return Ok(Some((s, Some(error))));
            }
            if s.depth >= 48
                || (s.a[0] - s.b[0]).hypot(s.a[1] - s.b[1]) <= 32. * self.op.tolerance_mm
            {
                let corner = Self::transition_corner(s);
                if (s.b[2] - s.a[2]).abs() > self.op.path_error_mm
                    && self.clear(s.a, corner, work, cancel)?
                    && self.clear(corner, s.b, work, cancel)?
                {
                    return Ok(Some((s, None)));
                }
                let clear = self.clear(s.a, s.b, work, cancel)?;
                return Err(format!(
                    "parallel segment cannot meet continuous clearance/path error within subdivision limit: a={:?}, b={:?}, excess={excess:?}, clear={clear}, midpoint_z={mid_z}",
                    s.a, s.b
                ));
            }
            let mid = [xy[0], xy[1], mid_z];
            let depth = s.depth + 1;
            stack.push(Segment {
                a: mid,
                b: s.b,
                depth,
            });
            stack.push(Segment {
                a: s.a,
                b: mid,
                depth,
            });
            *splits += 1;
        }
    }
    fn emit(&mut self, end: Vec3, kind: MoveKind) -> Result<Option<Motion>, String> {
        // Shorter than a motion line can be (1e-9 mm): already there. Two
        // passes whose end and start coincide but for rounding (Project
        // paths, flow rows) link without a move.
        let d = (0..3)
            .map(|k| (end[k] - self.position[k]).powi(2))
            .sum::<f64>()
            .sqrt();
        if end == self.position || d < 1e-9 {
            return Ok(None);
        }
        if self.next_id == u64::MAX {
            return Err("parallel motion id space exhausted".into());
        }
        let curve = Curve::Line {
            start: self.position,
            end,
        };
        curve.prepare(self.op.tolerance_mm)?;
        let motion = Motion {
            id: self.next_id,
            operation: self.op.id.clone(),
            tool: self.op.tool.clone(),
            kind,
            curve,
            feed_mm_min: if kind == MoveKind::Rapid {
                None
            } else {
                Some(if kind == MoveKind::Plunge {
                    self.op.plunge_mm_min
                } else {
                    self.op.feed_mm_min
                })
            },
        };
        self.position = end;
        self.next_id += 1;
        Ok(Some(motion))
    }
    pub fn next_motion_with_stock(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        if stock.spec() != &self.spec
            || stock.generation() == u64::MAX
            || self
                .expected_generation
                .is_some_and(|g| g != stock.generation())
        {
            return Err(
                "commit preceding parallel motion to matching stock before planning".into(),
            );
        }
        if let Some(ceiling) = &mut self.stock_ceiling {
            ceiling.validate(stock, cancel)?;
        }
        loop {
            if cancel.is_cancelled() {
                return Err("parallel planning cancelled".into());
            }
            let result = match self.phase {
                Phase::Lift => {
                    let result = self.emit(
                        [self.position[0], self.position[1], self.heights.clearance],
                        MoveKind::Rapid,
                    )?;
                    self.phase = Phase::Approach;
                    result
                }
                Phase::Approach => {
                    self.setup_row(cancel)?;
                    // Travel at the current rapid height.
                    let result = self.emit(
                        [self.air_start[0], self.air_start[1], self.position[2]],
                        MoveKind::Rapid,
                    )?;
                    self.phase = Phase::Descend;
                    result
                }
                Phase::Descend => {
                    // Rapid only down to the feed height (above the stock top),
                    // or to the air entry if that is higher.
                    let entry = self.air_start[2] - self.tip_offset;
                    let result = self.emit(
                        [
                            self.air_start[0],
                            self.air_start[1],
                            self.heights.feed.max(entry),
                        ],
                        MoveKind::Rapid,
                    )?;
                    self.phase = Phase::Entry;
                    result
                }
                Phase::Entry => {
                    let result = self.emit(
                        [
                            self.air_start[0],
                            self.air_start[1],
                            self.air_start[2] - self.tip_offset,
                        ],
                        MoveKind::Plunge,
                    )?;
                    self.phase = Phase::Lead;
                    result
                }
                Phase::Lead => {
                    let result = self.emit(
                        [
                            self.start[0],
                            self.start[1],
                            self.start[2] - self.tip_offset,
                        ],
                        MoveKind::Lead,
                    )?;
                    self.phase = Phase::Cut;
                    result
                }
                Phase::Cut => match self.next_cut(cancel)? {
                    Some((end, kind)) => {
                        let r = self.emit(end, kind)?;
                        if r.is_some() {
                            if kind == MoveKind::Cut {
                                self.report.cut_segments += 1;
                            } else {
                                self.report.transition_moves += 1;
                            }
                        }
                        r
                    }
                    None => {
                        self.report.rows_completed += 1;
                        self.phase = Phase::Exit;
                        None
                    }
                },
                Phase::Exit => {
                    let result = self.emit(
                        [
                            self.air_end[0],
                            self.air_end[1],
                            self.air_end[2] - self.tip_offset,
                        ],
                        MoveKind::Lead,
                    )?;
                    self.phase = Phase::Retract;
                    result
                }
                Phase::Retract => {
                    // Feed only up to the feed height (never downward).
                    let r = self.emit(
                        [
                            self.position[0],
                            self.position[1],
                            self.heights.feed.max(self.position[2]),
                        ],
                        MoveKind::Retract,
                    )?;
                    self.phase = Phase::Rise;
                    r
                }
                Phase::Rise => {
                    self.row += 1;
                    let last = self.row == self.report.rows;
                    let z = if last {
                        self.heights.clearance
                    } else {
                        self.heights.retract
                    };
                    let r = self.emit(
                        [self.position[0], self.position[1], z.max(self.position[2])],
                        MoveKind::Rapid,
                    )?;
                    self.phase = if last { Phase::Done } else { Phase::Approach };
                    r
                }
                Phase::Done => {
                    self.drained = true;
                    return Ok(None);
                }
            };
            if let (Some(motion), Some(parts)) = (&result, &self.depth_parts)
                && motion.kind != MoveKind::Rapid
            {
                // The parts above the flutes stay above the committed stock
                // under them: the pass cuts no deeper than the flutes reach.
                let bounds = motion.curve.prepare(self.op.tolerance_mm)?.bounds();
                let tip = bounds.min[2];
                let guard = 16. * self.op.tolerance_mm;
                let mut work = u64::MAX;
                for &(height, radius) in parts {
                    let pad = radius + guard;
                    let rect = [
                        [bounds.min[0] - pad, bounds.min[1] - pad],
                        [bounds.max[0] + pad, bounds.max[1] + pad],
                    ];
                    if !stock.bounds_clear(rect, tip + height - guard, &mut work, cancel)? {
                        let at = motion.curve.end();
                        let flutes = parts.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
                        return Err(format!(
                            "{}: the pass at X{:.3} Y{:.3} Z{:.3} cuts deeper than the cutting length {flutes}: stock stands above the tool's non-cutting part {height:.3} mm above the tip; remove that stock first (roughing) or use a tool with longer flutes",
                            self.op.id, at[0], at[1], at[2]
                        ));
                    }
                }
            }
            if result.is_some() {
                self.expected_generation = Some(stock.generation() + 1);
                return Ok(result);
            }
        }
    }
}
