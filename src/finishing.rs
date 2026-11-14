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

mod geodesic;
mod patterns;
mod pencil;
mod scallop;
pub use patterns::{FinishPattern, GeodesicSeed};
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
    /// Raster: cut every row in the same direction (from the low end of
    /// `axis`, or the high end with `reverse`) instead of alternating. The
    /// tool returns to the start side at the retract height, a rapid link
    /// like any other, and never cuts on the way back. Others require false.
    /// Established practice for a consistent climb or conventional cut:
    /// Autodesk Fusion's Parallel strategy ("To create only climb cuts or
    /// only conventional cuts, set the Direction parameter", Fusion help,
    /// "Generate a Parallel toolpath") and FreeCAD CAM's 3D Surface
    /// operation (cut pattern `Line` against `ZigZag`, Path/Op/Surface.py).
    /// Issue #175.
    #[serde(default, skip_serializing_if = "is_false")]
    pub one_way: bool,
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
    /// How a pass is entered where live stock stands above its entry: a
    /// ramp down the pass line (`RampEntry`: declared angle, feed and
    /// shortest reversal). None declares that no stock stands above the
    /// entries; an entry that has stock above it is then refused (see
    /// `ParallelPlan::plunge_or_ramp`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub entry: Option<crate::entry::PocketEntry>,
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
    /// The real flat/ball/bull profile at its planned tip height may reach
    /// more than `threshold` into a flagged cell. The cell's enclosing disk
    /// deliberately retains uncertain strokes; selection is not a removal
    /// certificate, and the full stock/material replay still decides completion.
    fn engaged(&self, p: [f64; 2], tip_z: f64, radius: f64, corner: f64, threshold: f64) -> bool {
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
                if near <= radius {
                    let arc = (near - (radius - corner)).max(0.);
                    let bottom = tip_z + corner - (corner * corner - arc * arc).max(0.).sqrt();
                    if self.lower[k] - bottom > threshold {
                        return true;
                    }
                }
            }
        }
        false
    }
}
fn is_false(v: &bool) -> bool {
    !*v
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
    /// Raster row `row` is cut from the high end of the axis to the low end.
    /// Zigzag alternates from the first row's direction; one-way keeps it.
    fn row_reversed(&self, row: usize) -> bool {
        self.reverse ^ (!self.one_way && row % 2 == 1)
    }
    pub(crate) fn map_geometry<H, E>(
        self,
        f: &mut impl FnMut(G) -> Result<H, E>,
    ) -> Result<ParallelOperation<H>, E> {
        Ok(ParallelOperation {
            limits: self.limits,
            entry: self.entry,
            id: self.id,
            tool: self.tool,
            surface: f(self.surface)?,
            floor_z_mm: self.floor_z_mm,
            stepover_mm: self.stepover_mm,
            clearance_z_mm: self.clearance_z_mm,
            axis: self.axis,
            reverse: self.reverse,
            one_way: self.one_way,
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
    /// Raster rows all cut in the same direction (reported when requested).
    #[serde(skip_serializing_if = "is_false")]
    pub one_way: bool,
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
    /// Passes entered by the declared ramp because the live stock stood
    /// above the pass's entry height.
    #[serde(skip_serializing_if = "no_entries")]
    pub ramped_entries: usize,
    /// Passes entered by a plunge in air back along their first segment and
    /// a horizontal lead, because live stock stood above the air entry.
    #[serde(skip_serializing_if = "no_entries")]
    pub side_entries: usize,
    /// The most stock an entry plunge was let through where the stock
    /// cells cannot tell stock from the part: a cell whose upper bound is no
    /// higher than the part's highest point within it (stock finished to
    /// size) may stand above the cutter's face by up to the part's height
    /// range across that cell.
    pub max_admitted_plunge_mm: f64,
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
    /// Geodesic passes: the measured spacing and scallop (#12).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub geodesic: Option<geodesic::GeodesicReport>,
}
fn no_entries(n: &usize) -> bool {
    *n == 0
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase {
    Lift,
    Approach,
    Entry,
    /// Zigzag down along the row through stock above the entry height.
    Ramp,
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
    /// Ramp points (tip) down to the pass's air entry, at the entry feed.
    ramp: std::collections::VecDeque<Vec3>,
    /// Where the entry plunge stops (tip): the entry, or a ramp's start.
    entry_target: f64,
    /// The operation's machining limits (side entries are checked on them).
    limits: Option<crate::limits::PreparedLimits>,
    /// The part, for its highest point within a stock cell (entry checks).
    part: crate::flat::Index,
    part_scratch: std::cell::RefCell<(Vec<u32>, u32)>,
    prepared_segment: Option<(Segment, Option<f64>)>,
    phase: Phase,
    next_id: u64,
    expected_generation: Option<u64>,
    drained: bool,
    report: ParallelReport,
    stock_ceiling: Option<StockCeiling>,
    /// When the remaining stock stands higher above the floor than the
    /// cutting length: the tool parts above the flutes (neck, shank, holder
    /// sections), each checked against the stock under it on every feed
    /// move (the depth the pass actually cuts, not the stock top over the
    /// floor; issues #88, #161).
    move_depth: Option<crate::reach::MoveDepth>,
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
            plan.move_depth = Some(crate::reach::MoveDepth::new(
                &plan.op.id,
                tool,
                plan.op.tolerance_mm,
            )?);
        }
        plan.spec = stock.spec().clone();
        plan.expected_generation = Some(stock.generation());
        plan.report.stock_top = Some(ceiling.report.clone());
        plan.report.clearance_proof = if plan.move_depth.is_some() {
            "continuous segment clearance against downward-extruded triangles and base with the declared cutter profile; neck/shank/holder above the committed stock under them on every feed move"
        } else {
            "continuous segment clearance against downward-extruded triangles and base with the declared cutter profile; shank/holder above admitted committed stock ceiling"
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
    /// move against the committed stock (`move_depth`), not here.
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
                            && !matches!(
                                op.pattern,
                                FinishPattern::Pencil { .. }
                                    | FinishPattern::Scallop
                                    | FinishPattern::Geodesic { .. }
                            )),
                    "slope limits, pencil, scallop and geodesic passes need a ball end mill (a flat end mill has no single contact normal)".into(),
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
                    crate::reach::ReachRefusal::cutting_length(&op.id, &op.tool, spec.max_mm[2], op.floor_z_mm, cutter.cutting_length_mm).error(format_args!(
                        "depth {} below the stock top exceeds the cutting length {}",
                        spec.max_mm[2] - op.floor_z_mm,
                        cutter.cutting_length_mm
                    )),
                ),
                if matches!(
                    op.pattern,
                    FinishPattern::Raster
                        | FinishPattern::Spiral { .. }
                        | FinishPattern::Scallop
                        | FinishPattern::Geodesic { .. }
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
                    op.pattern.is_raster() || !op.one_way,
                    format!(
                        "one_way applies to the raster pattern only; the {} pattern has its own direction",
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
        if let Some(entry) = &op.entry {
            let Some(ramp) = entry.ramp() else {
                return Err(
                    "parallel finishing: only a RampEntry enters a pass through stock (a helix has no place on a pass line); use entry=RampEntry(angle_deg, ...) or declare none"
                        .into(),
                );
            };
            ramp.validate(op.tolerance_mm, op.plunge_mm_min)?;
        }
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
        let mut limits = slope_limits("parallel finishing", op.slope_from_deg, op.slope_to_deg)?;
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
            FinishPattern::Scallop | FinishPattern::Geodesic { .. } => {
                (passes.len(), op.stepover_mm)
            }
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
        let mut geodesic_report = None;
        if let FinishPattern::Geodesic { seed, tool_centre } = &op.pattern {
            // Geodesic plans inside its slope range (the drive surface is
            // the faces within it): its passes are not cut afterwards.
            let (found, report) = geodesic::passes(
                mesh,
                &surface,
                op.stepover_mm,
                inflated,
                radius + op.stock_to_leave_mm,
                op.path_error_mm,
                seed,
                *tool_centre,
                limits.take(),
                &mut slope_work,
                cancel,
            )?;
            passes = found;
            geodesic_report = Some(report);
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
                    let tip = f.tip(p);
                    if let (Some(map), Some(threshold)) = (&rest, op.rest_threshold_mm)
                        && !map.engaged(p, tip, radius, corner.unwrap(), threshold)
                    {
                        return Ok(false);
                    }
                    return region_limits.as_ref().map_or(Ok(true), |l| {
                        l.margin(p, tip - spacing, true).map(|m| m >= spacing / 2.)
                    });
                }
                let here = surface.contact(p, inflated, &mut slope_work, cancel)?;
                if let (Some(map), Some(threshold)) = (&rest, op.rest_threshold_mm)
                    && !map.engaged(p, here.center_z - radius, radius, radius, threshold)
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
                    "rest finishing selected no resolvable strokes above the threshold; completion remains unproven"
                        .into(),
                );
            }
            passes.len()
        } else if matches!(
            op.pattern,
            FinishPattern::Pencil { .. } | FinishPattern::Scallop | FinishPattern::Geodesic { .. }
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
            one_way: op.one_way,
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
            ramped_entries: 0,
            side_entries: 0,
            max_admitted_plunge_mm: 0.,
            path_error_mm: op.path_error_mm,
            safety_margin_mm: op.safety_margin_mm,
            stock_to_leave_mm: op.stock_to_leave_mm,
            surface_error_mm: op.surface.surface_error_mm,
            max_certified_path_excess_mm: matches!(cutter.shape, CutterShape::Ball).then_some(0.),
            max_sampled_path_excess_mm: (!matches!(cutter.shape, CutterShape::Ball)).then_some(0.),
            flat_reference_scallop_mm: radius
                - (radius * radius - (pitch / 2.).powi(2)).max(0.).sqrt(),
            work: slope_work,
            clearance_proof: "continuous segment clearance against downward-extruded triangles and base with the declared cutter profile; shank/holder above initial stock",
            surface_completion: "not_certified; path excess is bounded, whole-target scallop/unreachable regions require independent stock checks",
            geodesic: geodesic_report,
        };
        let heights = crate::heights::Heights::resolve(
            op.clearance_z_mm,
            op.retract_z_mm,
            op.feed_z_mm,
            spec.max_mm[2],
            16. * op.tolerance_mm,
        )?;
        let tip_offset = if flat.is_some() { 0. } else { radius };
        let part = crate::flat::Index::new(mesh, op.floor_z_mm, radius, cancel)?;
        let part_scratch = std::cell::RefCell::new((vec![0u32; part.len()], 0u32));
        Ok(Self {
            part,
            part_scratch,
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
            ramp: Default::default(),
            entry_target: 0.,
            limits: op
                .limits
                .as_ref()
                .filter(|l| !l.is_empty())
                .map(|l| crate::limits::PreparedLimits::new(l, radius, op.tolerance_mm))
                .transpose()?,
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
            move_depth: None,
        })
    }
    pub fn report(&self) -> &ParallelReport {
        &self.report
    }
    /// Planning progress (#198): finishing passes (raster rows, or the
    /// pattern's passes) done of those planned.
    pub fn progress(&self) -> crate::progress::Planned {
        crate::progress::Planned::Passes {
            done: self.report.rows_completed,
            total: self.report.rows,
        }
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
            FinishPattern::Raster
            | FinishPattern::Pencil { .. }
            | FinishPattern::Scallop
            | FinishPattern::Geodesic { .. } => vec![],
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
        let reverse = op.row_reversed(row);
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
        let reverse = self.op.row_reversed(self.row);
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
        // The vertices a merge drops: each must lie on the merged chord in
        // XY (within the tolerance), so merging removes only collinear
        // points (subdivision midpoints, straight runs) and never moves the
        // planned path sideways. The height certificate below bounds only
        // the vertical error, so on a flat face it would accept any chord.
        let mut dropped: Vec<Vec3> = vec![];
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
            dropped.push(end);
            if dropped
                .iter()
                .any(|q| xy_off_chord(*q, first.a, next.b) > self.op.tolerance_mm)
            {
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
    /// How the pass is entered, decided before the cutter travels to it.
    /// A straight plunge where the live stock under the cutter's lower face
    /// (each cell at its highest point and its point nearest the axis)
    /// stands no more than a tolerance above it at the air entry. Otherwise
    /// the pass first looks back along its own first segment, up to four cutter
    /// radii, for a point the plunge reaches with no more than that, which
    /// the machining limits allow and from which a horizontal lead at the
    /// entry height to the pass is proven clear of the part: the cutter then
    /// plunges there, in air, and leads in (`side_entries`). Failing that,
    /// the declared `RampEntry` ramps down the pass's own path (a raster
    /// row's line, or a pattern pass's polyline, as far as a ramp from the
    /// feed height would reach), from the stock's top over that path, never
    /// below the part's contact along it (z = max(planar ramp, contact)),
    /// every move proven clear of the part (`ramped_entries`). No entry
    /// declared, a one-way pass (a zigzag would cut back along the row),
    /// stock above the feed height, a ramp that does not fit and one that is
    /// not clear are refused, naming the pass: the cutter is never plunged
    /// into stock. Sets `air_start`, `entry_target` and `ramp`.
    fn plan_entry(
        &mut self,
        stock: &Stock,
        work: &mut SurfaceWork,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        self.ramp.clear();
        let e = self.air_start;
        let entry = e[2] - self.tip_offset;
        self.entry_target = entry;
        let a = [e[0], e[1]];
        let (excess, admitted) = self.entry_excess(stock, a, entry);
        if excess <= self.op.tolerance_mm {
            self.report.max_admitted_plunge_mm = self.report.max_admitted_plunge_mm.max(admitted);
            return Ok(());
        }
        // The path a lead or ramp follows: a raster row's line (air entry to
        // air end), or a pattern pass's own polyline.
        let line: Vec<[f64; 2]> = if self.passes.is_empty() {
            vec![a, [self.air_end[0], self.air_end[1]]]
        } else {
            self.vertices.iter().map(|v| [v[0], v[1]]).collect()
        };
        if let Some((c, admitted)) = self.side_entry(stock, &line, entry, work, cancel)? {
            self.air_start = [c[0], c[1], e[2]];
            self.report.max_admitted_plunge_mm = self.report.max_admitted_plunge_mm.max(admitted);
            self.report.side_entries += 1;
            return Ok(());
        }
        let reach = self
            .op
            .entry
            .as_ref()
            .and_then(|e| e.ramp())
            .map_or(0., |r| {
                (self.heights.feed - entry).max(0.) / r.angle_rad.tan()
            });
        // The path, up to the length a ramp from the feed height would need.
        let mut segments = vec![];
        let mut length = 0.;
        for w in line.windows(2) {
            let (p, q) = (w[0], w[1]);
            let d = (q[0] - p[0]).hypot(q[1] - p[1]);
            if d <= 0. {
                continue;
            }
            let f = if length + d > reach {
                ((reach - length) / d).max(0.)
            } else {
                1.
            };
            let q = [p[0] + f * (q[0] - p[0]), p[1] + f * (q[1] - p[1])];
            segments.push((p, q));
            length += f * d;
            if f < 1. {
                break;
            }
        }
        let chain: Vec<_> = segments
            .iter()
            .map(|&(p, q)| crate::pocket::Stroke::line(p, q))
            .collect();
        // The stock's top over the ramp's own footprint: the stretch of the
        // path a ramp from that height needs (the whole path when it must
        // reverse), raised until it no longer grows.
        let over = |span: f64| -> Result<f64, String> {
            let mut top = f64::MIN;
            let mut walked = 0.;
            for &(p, q) in &segments {
                let d = (q[0] - p[0]).hypot(q[1] - p[1]);
                let f = if walked + d > span {
                    ((span - walked) / d).clamp(0., 1.)
                } else {
                    1.
                };
                let q = [p[0] + f * (q[0] - p[0]), p[1] + f * (q[1] - p[1])];
                top = top.max(crate::flat::segment_stock_top(
                    stock,
                    p,
                    q,
                    self.radius,
                    cancel,
                )?);
                walked += f * d;
                if f < 1. {
                    break;
                }
            }
            Ok(top)
        };
        let mut top = entry + excess;
        if let Some(ramp) = self.op.entry.as_ref().and_then(|e| e.ramp())
            && !segments.is_empty()
        {
            let tan = ramp.angle_rad.tan();
            let mut settled = false;
            for _ in 0..32 {
                let span = ((top - entry) / tan).min(length);
                let next = over(span)?.max(entry + excess);
                if next <= top {
                    settled = true;
                    break;
                }
                top = next;
            }
            if !settled {
                top = top.max(over(length)?);
            }
        }
        let b = line.last().copied().unwrap_or(a);
        let which = format!(
            "parallel finishing {}: the pass entered at X{:.3} Y{:.3} Z{entry:.3} (its path runs towards X{:.3} Y{:.3}) has live stock {excess:.3} mm above the cutter there and up to Z{top:.3} along the path",
            self.op.id, a[0], a[1], b[0], b[1]
        );
        let Some(ramp) = self.op.entry.as_ref().and_then(|e| e.ramp()) else {
            return Err(format!(
                "{which}, and the operation declares no entry: the cutter is not plunged into stock. Declare entry=RampEntry(angle_deg, ...) for the tool, rough or rest-rough this area first, or use finer stock cells"
            ));
        };
        if self.op.one_way {
            return Err(format!(
                "{which}: a one-way pass cannot ramp in (a zigzag would cut back along the row). Rough or rest-rough this area first, or cut the rows zigzag"
            ));
        }
        if length <= 16. * self.op.tolerance_mm {
            return Err(format!(
                "{which}: the path is shorter than 16 tolerances, too short to ramp on. Rough or rest-rough this area first"
            ));
        }
        if top > self.heights.feed {
            return Err(format!(
                "{which}, above the feed height Z{:.3}: raise feed_z_mm/retract_z_mm or rough this area first",
                self.heights.feed
            ));
        }
        let curves = ramp.zigzag(&chain, top, entry).map_err(|why| {
            format!(
                "{which}: {why}. Use a steeper declared angle, a longer pass, or rough this area first"
            )
        })?;
        // Each planar leg, sampled finely enough to follow the part, kept at
        // or above the part's contact along it.
        let [wx, wy] = stock.cell_widths_mm();
        let step = (0.5 * wx.min(wy))
            .min(self.radius / 4.)
            .max(16. * self.op.tolerance_mm);
        let first = curves.first().map_or(a, |c| {
            let q = c.start();
            [q[0], q[1]]
        });
        // (tip point, its planar ramp height): the planar legs sampled finely
        // enough to follow the part, each kept at or above the part's
        // contact; a move that is not proven clear (the part bulges between
        // samples, e.g. over a convex edge) is halved, the midpoint lifted
        // onto the contact, down to a depth.
        let mut samples: Vec<(Vec3, f64)> = vec![([first[0], first[1], top], top)];
        for c in &curves {
            let (p, q) = (c.start(), c.end());
            let run = (q[0] - p[0]).hypot(q[1] - p[1]);
            let n = (run / step).ceil().max(1.);
            if n > 1e5 || samples.len() > 1_000_000 {
                return Err(format!(
                    "{which}: the ramp needs too many moves to follow the part; use a steeper declared angle or rough this area first"
                ));
            }
            for k in 1..=n as usize {
                if k % 256 == 0 && cancel.is_cancelled() {
                    return Err("parallel planning cancelled".into());
                }
                let f = k as f64 / n;
                let xy = [p[0] + f * (q[0] - p[0]), p[1] + f * (q[1] - p[1])];
                let planar = p[2] + f * (q[2] - p[2]);
                let contact = self.contact_z(xy, work, cancel)? - self.tip_offset;
                samples.push(([xy[0], xy[1], planar.max(contact)], planar));
            }
        }
        let mut points = Vec::with_capacity(samples.len());
        let mut previous = ([a[0], a[1], top], top);
        for next in samples {
            // Stack of pending moves from `previous`, each with its depth.
            let mut pending = vec![(next, 0u32)];
            while let Some((end, depth)) = pending.pop() {
                let from = [
                    previous.0[0],
                    previous.0[1],
                    previous.0[2] + self.tip_offset,
                ];
                let to = [end.0[0], end.0[1], end.0[2] + self.tip_offset];
                if self.clear(from, to, work, cancel)? {
                    points.push(end.0);
                    previous = end;
                    continue;
                }
                let run = (end.0[0] - previous.0[0]).hypot(end.0[1] - previous.0[1]);
                if depth >= 24 || run <= 16. * self.op.tolerance_mm || points.len() > 1_000_000 {
                    return Err(format!(
                        "{which}: the declared ramp down the pass's path is not clear of the part (a move reaches X{:.3} Y{:.3} Z{:.3}). Rough or rest-rough this area first",
                        end.0[0], end.0[1], end.0[2]
                    ));
                }
                let xy = [
                    0.5 * (previous.0[0] + end.0[0]),
                    0.5 * (previous.0[1] + end.0[1]),
                ];
                let planar = 0.5 * (previous.1 + end.1);
                let contact = self.contact_z(xy, work, cancel)? - self.tip_offset;
                let mid = ([xy[0], xy[1], planar.max(contact)], planar);
                pending.push((end, depth + 1));
                pending.push((mid, depth + 1));
            }
        }
        self.entry_target = top;
        self.ramp = points.into();
        self.report.ramped_entries += 1;
        Ok(())
    }
    /// `plunge_excess` at `xy` for a plunge to `tip`; a cell counts as at size
    /// where its upper bound is no higher than the part's highest point
    /// within the cell (its circumscribed disk) plus the stock to leave and
    /// a tolerance.
    fn entry_excess(&self, stock: &Stock, xy: [f64; 2], tip: f64) -> (f64, f64) {
        let [wx, wy] = stock.cell_widths_mm();
        let half = 0.5 * wx.hypot(wy);
        let mut scratch = self.part_scratch.borrow_mut();
        let (seen, stamp) = &mut *scratch;
        plunge_excess(
            stock,
            xy,
            tip,
            self.radius,
            self.tip_offset,
            |_, c, upper| {
                let top = self.part.max_height(c, c, half, seen, stamp);
                upper <= top + self.op.stock_to_leave_mm + self.op.tolerance_mm
            },
        )
    }
    /// A point back along the pass's first segment (away from its direction
    /// of travel), or straight away from the stock's box when the entry lies
    /// beside it, up to four cutter radii from the air entry, where a plunge to
    /// the entry height has at most a tolerance of live stock above the
    /// cutter's face, the machining limits allow the plunge and the lead,
    /// and the horizontal lead at the entry height to the air entry is
    /// proven clear of the part. None when no such point exists.
    fn side_entry(
        &self,
        stock: &Stock,
        line: &[[f64; 2]],
        entry: f64,
        work: &mut SurfaceWork,
        cancel: &impl CancelSignal,
    ) -> Result<Option<([f64; 2], f64)>, String> {
        let a = line[0];
        let Some(&b) = line.iter().find(|q| **q != a) else {
            return Ok(None);
        };
        let d = (b[0] - a[0]).hypot(b[1] - a[1]);
        let mut directions = vec![[(a[0] - b[0]) / d, (a[1] - b[1]) / d]];
        // Beside the stock's box: also straight away from it.
        let near = [
            a[0].clamp(self.spec.min_mm[0], self.spec.max_mm[0]),
            a[1].clamp(self.spec.min_mm[1], self.spec.max_mm[1]),
        ];
        let out = (a[0] - near[0]).hypot(a[1] - near[1]);
        if out > 0. {
            directions.push([(a[0] - near[0]) / out, (a[1] - near[1]) / out]);
        }
        let [wx, wy] = stock.cell_widths_mm();
        let step = (0.5 * wx.min(wy))
            .min(self.radius / 4.)
            .max(16. * self.op.tolerance_mm);
        let limit = 4. * self.radius;
        let n = (limit / step).ceil().min(4096.) as usize;
        let reference = self.air_start[2];
        for back in directions {
            for k in 1..=n {
                if cancel.is_cancelled() {
                    return Err("parallel planning cancelled".into());
                }
                let t = (k as f64 * step).min(limit);
                let c = [a[0] + t * back[0], a[1] + t * back[1]];
                let (excess, admitted) = self.entry_excess(stock, c, entry);
                if excess > self.op.tolerance_mm {
                    continue;
                }
                if let Some(l) = &self.limits {
                    let plunge = Curve::Line {
                        start: [c[0], c[1], self.heights.feed.max(entry)],
                        end: [c[0], c[1], entry],
                    };
                    let lead = Curve::Line {
                        start: [c[0], c[1], entry],
                        end: [a[0], a[1], entry],
                    };
                    if !l.allows_curve(&plunge, true)? || !l.allows_curve(&lead, true)? {
                        continue;
                    }
                }
                if self.clear(
                    [c[0], c[1], reference],
                    [a[0], a[1], reference],
                    work,
                    cancel,
                )? {
                    return Ok(Some((c, admitted)));
                }
            }
        }
        Ok(None)
    }
    fn emit(&mut self, end: Vec3, kind: MoveKind) -> Result<Option<Motion>, String> {
        self.emit_at(end, kind, None)
    }
    /// `emit`, with a plunge feed other than the operation's (a ramp's).
    fn emit_at(
        &mut self,
        end: Vec3,
        kind: MoveKind,
        plunge_feed: Option<f64>,
    ) -> Result<Option<Motion>, String> {
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
                    plunge_feed.unwrap_or(self.op.plunge_mm_min)
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
                    let mut work = self.report.work;
                    self.plan_entry(stock, &mut work, cancel)?;
                    self.report.work = work;
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
                    let xy = [self.air_start[0], self.air_start[1]];
                    self.phase = if self.ramp.is_empty() {
                        Phase::Lead
                    } else {
                        Phase::Ramp
                    };
                    // Down through air to the stock's top (a ramp's start),
                    // or onto the entry.
                    self.emit(
                        [xy[0], xy[1], self.entry_target.min(self.position[2])],
                        MoveKind::Plunge,
                    )?
                }
                Phase::Ramp => match self.ramp.pop_front() {
                    Some(end) => {
                        let feed = self
                            .op
                            .entry
                            .as_ref()
                            .and_then(|e| e.feed_mm_min())
                            .unwrap_or(self.op.plunge_mm_min);
                        self.emit_at(end, MoveKind::Plunge, Some(feed))?
                    }
                    None => {
                        self.phase = Phase::Lead;
                        None
                    }
                },
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
            if let (Some(motion), Some(depth)) = (&result, &self.move_depth) {
                // The parts above the flutes stay above the committed stock
                // under them: the pass cuts no deeper than the flutes reach.
                depth.check(motion, stock, cancel)?;
            }
            if result.is_some() {
                self.expected_generation = Some(stock.generation() + 1);
                return Ok(result);
            }
        }
    }
}

/// The XY distance of `q` from the chord `a`-`b`.
fn xy_off_chord(q: Vec3, a: Vec3, b: Vec3) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let ll = dx * dx + dy * dy;
    let t = if ll > 0. {
        (((q[0] - a[0]) * dx + (q[1] - a[1]) * dy) / ll).clamp(0., 1.)
    } else {
        0.
    };
    (q[0] - a[0] - t * dx).hypot(q[1] - a[1] - t * dy)
}

/// How far a vertical plunge to `tip` at `xy` would cut into live stock: the
/// most any cell (at its nearest point to the axis) stands above the
/// cutter's lower face there (a ball of `ball` radius, or flat when 0).
/// A cell whose upper bound is no higher than `at_size` says the part
/// itself stands there (stock finished to size: the bound's own resolution,
/// a whole cell's height range of the part) is not counted; the most such a
/// cell stands above the face is returned as the admitted excess.
fn plunge_excess(
    stock: &Stock,
    xy: [f64; 2],
    tip: f64,
    r: f64,
    ball: f64,
    mut at_size: impl FnMut(usize, [f64; 2], f64) -> bool,
) -> (f64, f64) {
    let spec = stock.spec();
    let [nx, ny] = stock.dimensions();
    let [wx, wy] = stock.cell_widths_mm();
    let reach = r;
    let range = |c: f64, lo: f64, w: f64, n: usize| {
        let a = ((c - reach - lo) / w).floor().max(0.) as usize;
        let b = (((c + reach - lo) / w).ceil().max(0.) as usize).min(n);
        a..b
    };
    let mut excess = 0f64;
    let mut admitted = 0f64;
    for j in range(xy[1], spec.min_mm[1], wy, ny) {
        for i in range(xy[0], spec.min_mm[0], wx, nx) {
            let k = j * nx + i;
            let Some(c) = stock.cell_center(k) else {
                continue;
            };
            // `upper_mm` bounds the whole cell, so test the cell's nearest
            // point to the axis, not its centre: stock anywhere in a cell
            // the footprint overlaps may stand there, and the cutter's
            // lower face is lowest at that nearest point.
            let gx = ((c[0] - xy[0]).abs() - wx / 2.).max(0.);
            let gy = ((c[1] - xy[1]).abs() - wy / 2.).max(0.);
            let rho = gx.hypot(gy);
            if rho > r {
                continue;
            }
            let face = if ball > 0. && rho < ball {
                tip + ball - (ball * ball - rho * rho).sqrt()
            } else {
                tip + ball
            };
            let upper = stock.cells()[k].upper_mm;
            if upper - face <= 0. {
                continue;
            }
            if at_size(k, c, upper) {
                admitted = admitted.max(upper - face);
            } else {
                excess = excess.max(upper - face);
            }
        }
    }
    (excess, admitted)
}
