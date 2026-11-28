//! Lazy whole-stock planar facing. Coverage and exterior entry/link contract:
//! docs/operations.md. Nominal coverage is separate from stock/model uncertainty.
use crate::{
    motion::{Coolant, Curve, Event, Motion, MoveKind, SpindleDirection, Vec3},
    pocket::ResidualStatus,
    stock::{CancelSignal, Stock, StockSpec},
    tool::{CutterShape, ToolAssembly, valid_id},
};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

mod around;
pub use around::AroundLimitsReport;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TraverseAxis {
    X,
    Y,
    Auto,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FacingMode {
    Zigzag,
    OneWay,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FacingOperation {
    pub id: String,
    pub tool: String,
    pub floor_z_mm: f64,
    pub stepdown_mm: f64,
    pub stepover_mm: f64,
    pub clearance_z_mm: f64,
    pub feed_mm_min: f64,
    pub plunge_mm_min: f64,
    pub axis: TraverseAxis,
    /// Nominal rapid rate for comparing complete X and Y routes in `Auto`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub auto_rapid_mm_min: Option<f64>,
    pub mode: FacingMode,
    pub reverse: bool,
    pub entry_air_gap_mm: f64,
    pub edge_overlap_mm: Option<f64>,
    pub tolerance_mm: f64,
    pub spindle_rpm: f64,
    pub work_offset: u8,
    pub tool_number: u32,
    pub length_register: u32,
    pub coolant: Coolant,
    /// Rapid travel height between rows (absolute Z). Default: clearance.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub retract_z_mm: Option<f64>,
    /// Rapid descent stops here and plunge feed begins; leaving a row, feed
    /// changes to rapid here (absolute Z, above the stock top). Default: retract.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub feed_z_mm: Option<f64>,
    /// Containment and avoid areas (clamps, fixtures, features to leave alone).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub limits: Option<crate::limits::Limits>,
}
#[derive(Clone, Debug, Serialize)]
pub struct FacingReport {
    pub strategy: &'static str,
    pub geometric_residual_status: ResidualStatus,
    pub depth_passes: usize,
    pub rows_per_pass: usize,
    pub row_pitch_mm: f64,
    pub target_z_mm: f64,
    pub stock_xy_min_mm: [f64; 2],
    pub stock_xy_max_mm: [f64; 2],
    pub center_xy_min_mm: [f64; 2],
    pub center_xy_max_mm: [f64; 2],
    pub edge_overlap_mm: f64,
    pub entry_air_gap_mm: f64,
    pub axis: TraverseAxis,
    /// Estimated seconds for X and Y, respectively, in opt-in mode.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub auto_axis_seconds: Option<[Option<f64>; 2]>,
    /// Refusal reason for each unavailable axis, in X, Y order.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub auto_axis_refusals: Option<[Option<String>; 2]>,
    pub mode: FacingMode,
    pub cut_rows_emitted: usize,
    pub side_links_emitted: usize,
    pub clearance_returns: usize,
    pub nominal_uncovered_area_mm2: f64,
    pub entry_policy: &'static str,
    /// Facing round limits that reach the passes (#422): what was planned.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub around_limits: Option<AroundLimitsReport>,
}
#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase {
    Lift,
    Approach,
    Entry,
    Cut,
    SideLink,
    Retract,
    /// Feed height -> retract height (clearance after the last row).
    Rise,
    /// Travel height -> feed height above the next row start.
    Descend,
    Done,
}
#[derive(Clone)]
pub struct FacingPlan {
    op: FacingOperation,
    heights: crate::heights::Heights,
    spec: StockSpec,
    initial: Vec3,
    position: Vec3,
    axis: usize,
    low: f64,
    high: f64,
    first_row: f64,
    last_row: f64,
    /// The radius the cutter cuts the floor to.
    floor_radius: f64,
    /// The planned route round limits that reach the passes (#422), streamed
    /// in place of the plain rows.
    around: Option<VecDeque<around::Step>>,
    pass: usize,
    row: usize,
    phase: Phase,
    drained: bool,
    next_id: u64,
    expected_generation: Option<u64>,
    report: FacingReport,
}
const PLANNING_CANCELLED: &str = "facing planning cancelled";

impl FacingPlan {
    pub fn prepare(
        op: FacingOperation,
        assembly: &ToolAssembly,
        spec: &StockSpec,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        if cancel.is_cancelled() {
            return Err(PLANNING_CANCELLED.into());
        }
        if let Some(limits) = op.limits.clone().filter(|l| !l.is_empty()) {
            return Self::prepare_limited(op, limits, assembly, spec, initial, first_id, cancel);
        }
        if op.axis == TraverseAxis::Auto {
            if op.limits.as_ref().is_some_and(|limits| !limits.is_empty()) {
                return Err("automatic facing axis requires whole rectangular stock without in-plane limits; choose an explicit axis".into());
            }
            let rapid = op
                .auto_rapid_mm_min
                .filter(|v| v.is_finite() && *v > 0.)
                .ok_or("automatic facing axis requires a positive finite auto_rapid_mm_min")?;
            let candidate = |axis| {
                let mut concrete = op.clone();
                concrete.axis = axis;
                concrete.auto_rapid_mm_min = None;
                let plan = Self::prepare(concrete, assembly, spec, initial, first_id, cancel)?;
                let seconds = plan.clone().route_seconds(rapid, cancel)?;
                Ok::<_, String>((plan, seconds))
            };
            let x = candidate(TraverseAxis::X);
            if cancel.is_cancelled() || matches!(&x, Err(reason) if reason == PLANNING_CANCELLED) {
                return Err(PLANNING_CANCELLED.into());
            }
            let y = candidate(TraverseAxis::Y);
            if cancel.is_cancelled() || matches!(&y, Err(reason) if reason == PLANNING_CANCELLED) {
                return Err(PLANNING_CANCELLED.into());
            }
            let (mut selected, seconds, refusals) = match (x, y) {
                (Ok((x, tx)), Ok((y, ty))) => (
                    if ty + 1e-9 < tx { y } else { x },
                    [Some(tx), Some(ty)],
                    None,
                ),
                (Ok((x, tx)), Err(reason)) => (x, [Some(tx), None], Some([None, Some(reason)])),
                (Err(reason), Ok((y, ty))) => (y, [None, Some(ty)], Some([Some(reason), None])),
                (Err(x), Err(y)) if x == y => return Err(x),
                (Err(x), Err(y)) => {
                    return Err(format!("automatic facing axis refused: X: {x}; Y: {y}"));
                }
            };
            selected.report.auto_axis_seconds = Some(seconds);
            selected.report.auto_axis_refusals = refusals;
            return Ok(selected);
        }
        if op.auto_rapid_mm_min.is_some() {
            return Err("auto_rapid_mm_min requires axis = auto".into());
        }
        spec.grid()?;
        let prepared = assembly.prepare()?;
        let cutter = prepared.cutter().definition();
        let r = cutter.diameter_mm / 2.;
        let floor_r = if let Some(g) = prepared.cutter().indexable() {
            g.cutting()
                .floor_radius()
                .ok_or("facing: mounted inserts must reach the declared tip plane Z=0")?
        } else {
            r
        };
        if let Some(m) = cutter.shape.indexable()
            && op.stepdown_mm > m.max_depth_of_cut_mm
        {
            return Err(format!(
                "facing stepdown_mm exceeds max_depth_of_cut_mm {}; reduce stepdown",
                m.max_depth_of_cut_mm
            ));
        }
        let guard = 16. * op.tolerance_mm;
        let overlap = op.edge_overlap_mm.unwrap_or(cutter.diameter_mm * 0.1);
        let numbers = [
            op.floor_z_mm,
            op.stepdown_mm,
            op.stepover_mm,
            op.clearance_z_mm,
            op.feed_mm_min,
            op.plunge_mm_min,
            op.entry_air_gap_mm,
            overlap,
            op.tolerance_mm,
            op.spindle_rpm,
        ];
        crate::heights::require(
            "facing",
            [
                (
                    valid_id(&op.id),
                    format!("id {:?} is not a valid identifier", op.id),
                ),
                (
                    op.tool == assembly.id,
                    format!(
                        "tool {:?} does not match the assembly {:?}",
                        op.tool, assembly.id
                    ),
                ),
                (
                    matches!(
                        cutter.shape,
                        CutterShape::Flat | CutterShape::Indexable { .. }
                    ),
                    "facing needs a flat end mill or a declared indexable milling cutter".into(),
                ),
                (
                    numbers.iter().all(|n| n.is_finite()),
                    "all numeric parameters must be finite".into(),
                ),
                (
                    (1e-7..=0.01).contains(&op.tolerance_mm),
                    format!("tolerance_mm {} must be within 1e-7..0.01", op.tolerance_mm),
                ),
                (
                    op.floor_z_mm >= spec.min_mm[2] && op.floor_z_mm < spec.max_mm[2],
                    format!(
                        "floor_z_mm {} must be within the stock Z range {}..{}",
                        op.floor_z_mm, spec.min_mm[2], spec.max_mm[2]
                    ),
                ),
                (
                    cutter.shape.indexable().is_some()
                        || spec.max_mm[2] - op.floor_z_mm <= cutter.cutting_length_mm - guard,
                    crate::reach::ReachRefusal::cutting_length(
                        &op.id,
                        &op.tool,
                        spec.max_mm[2],
                        op.floor_z_mm,
                        cutter.cutting_length_mm,
                    )
                    .error(format_args!(
                        "facing depth {} below the stock top exceeds the cutting length {}",
                        spec.max_mm[2] - op.floor_z_mm,
                        cutter.cutting_length_mm
                    )),
                ),
                (
                    op.stepdown_mm > guard && op.stepdown_mm <= cutter.cutting_length_mm,
                    format!(
                        "stepdown_mm {} must be positive and at most the cutting length {}",
                        op.stepdown_mm, cutter.cutting_length_mm
                    ),
                ),
                (
                    op.stepover_mm > guard && op.stepover_mm <= 2. * (floor_r - guard),
                    format!(
                        "stepover_mm {} must be positive and less than the floor-cutting diameter {}",
                        op.stepover_mm,
                        2. * floor_r
                    ),
                ),
                (
                    (0. ..=floor_r - guard).contains(&overlap),
                    format!(
                        "edge_overlap_mm {overlap} must be between 0 and the cutter radius {r}"
                    ),
                ),
                (
                    op.entry_air_gap_mm >= guard,
                    format!("entry_air_gap_mm {} must be positive", op.entry_air_gap_mm),
                ),
                (
                    op.feed_mm_min > 0. && op.plunge_mm_min > 0. && op.spindle_rpm > 0.,
                    format!(
                        "feed_mm_min {}, plunge_mm_min {} and spindle_rpm {} must be positive",
                        op.feed_mm_min, op.plunge_mm_min, op.spindle_rpm
                    ),
                ),
                (
                    op.clearance_z_mm > spec.max_mm[2] + guard && op.clearance_z_mm.abs() <= 1e6,
                    format!(
                        "clearance_z_mm {} must be above the stock top {}",
                        op.clearance_z_mm, spec.max_mm[2]
                    ),
                ),
                (
                    initial.iter().all(|n| n.is_finite() && n.abs() <= 1e6),
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
                    (1..=9).contains(&op.work_offset),
                    format!("work_offset {} must be 1..9", op.work_offset),
                ),
                (
                    op.tool_number > 0 && op.length_register > 0,
                    "tool_number and length_register must be positive".into(),
                ),
            ],
        )?;
        let axis = if op.axis == TraverseAxis::X { 0 } else { 1 };
        let cross = 1 - axis;
        let low = spec.min_mm[axis] - r - op.entry_air_gap_mm - guard;
        let high = spec.max_mm[axis] + r + op.entry_air_gap_mm + guard;
        let first_row = spec.min_mm[cross] + floor_r - overlap - guard;
        let last_row = spec.max_mm[cross] - floor_r + overlap + guard;
        let (first_row, last_row, rows, pitch) = if first_row >= last_row {
            let midpoint = (spec.min_mm[cross] + spec.max_mm[cross]) / 2.;
            (midpoint, midpoint, 1, 0.)
        } else {
            let span = last_row - first_row;
            let intervals = (span / op.stepover_mm).ceil() as usize;
            (
                first_row,
                last_row,
                intervals.saturating_add(1),
                span / intervals as f64,
            )
        };
        let passes = ((spec.max_mm[2] - op.floor_z_mm) / op.stepdown_mm).ceil() as usize;
        if passes == 0
            || first_id == 0
            || first_id.checked_add(1).is_none()
            || [low, high, first_row, last_row]
                .iter()
                .any(|n| !n.is_finite() || n.abs() > 1e6)
        {
            return Err("facing exceeds bounded rows/passes/IDs or coordinate overtravel".into());
        }
        let mut xymin = [0.; 2];
        let mut xymax = [0.; 2];
        xymin[axis] = low;
        xymin[cross] = first_row;
        xymax[axis] = high;
        xymax[cross] = last_row;
        let report = FacingReport {
            strategy: "facing",
            geometric_residual_status: ResidualStatus::Pending,
            depth_passes: passes,
            rows_per_pass: rows,
            row_pitch_mm: pitch,
            target_z_mm: op.floor_z_mm,
            stock_xy_min_mm: [spec.min_mm[0], spec.min_mm[1]],
            stock_xy_max_mm: [spec.max_mm[0], spec.max_mm[1]],
            center_xy_min_mm: xymin,
            center_xy_max_mm: xymax,
            edge_overlap_mm: overlap,
            entry_air_gap_mm: op.entry_air_gap_mm,
            axis: op.axis,
            auto_axis_seconds: None,
            auto_axis_refusals: None,
            mode: op.mode,
            cut_rows_emitted: 0,
            side_links_emitted: 0,
            clearance_returns: 0,
            nominal_uncovered_area_mm2: 0.,
            entry_policy: "vertical entries and zigzag links outside initial stock; one-way returns at clearance",
            around_limits: None,
        };
        let heights = crate::heights::Heights::resolve(
            op.clearance_z_mm,
            op.retract_z_mm,
            op.feed_z_mm,
            spec.max_mm[2],
            guard,
        )?;
        Ok(Self {
            heights,
            op,
            spec: spec.clone(),
            initial,
            position: initial,
            axis,
            low,
            high,
            first_row,
            last_row,
            floor_radius: floor_r,
            around: None,
            pass: 0,
            row: 0,
            phase: Phase::Lift,
            drained: false,
            next_id: first_id,
            expected_generation: None,
            report,
        })
    }
    /// Facing with limits (#411, #422). Limits that cannot touch anything the
    /// facing sweeps are left out and the plain facing runs unchanged:
    /// - an avoid area constrains only a tip below its top, and every facing
    ///   move keeps its tip at or above `floor_z_mm`, so an area whose top is
    ///   at or below the floor never constrains it;
    /// - when every move of the plain facing is proven inside what remains
    ///   (the proof region operations and the run's link guard use), the
    ///   limits do not reach it either.
    ///
    /// Otherwise the plain facing's rows are planned round the limits
    /// (`around`): stopped where the limits stop the cutter, linked at depth
    /// where a proven link exists, and finished by a pass along each limit's
    /// edge, every entry a vertical descent with the whole cutter
    /// `entry_air_gap_mm` clear of the stock. Both explicit axes use zigzag
    /// links. One-way facing can leave stretches without an entry and is
    /// refused by name where the limits reach the cut.
    fn prepare_limited(
        op: FacingOperation,
        limits: crate::limits::Limits,
        assembly: &ToolAssembly,
        spec: &StockSpec,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        limits.validate()?;
        let reaching = crate::limits::Limits {
            contain: limits.contain.clone(),
            avoid: limits
                .avoid
                .iter()
                .filter(|a| a.top_z_mm > op.floor_z_mm)
                .cloned()
                .collect(),
        };
        let mut native = op.clone();
        native.limits = None;
        let mut plan = Self::prepare(native, assembly, spec, initial, first_id, cancel)?;
        let r = assembly.cutter.diameter_mm / 2.;
        if reaching.is_empty() || plan.proven_within(&reaching, r, cancel)? {
            return Ok(plan);
        }
        if op.axis == TraverseAxis::Auto || op.auto_rapid_mm_min.is_some() {
            return Err("automatic facing axis is unavailable with in-plane limits; choose an explicit axis without auto_rapid_mm_min".into());
        }
        if op.mode == FacingMode::OneWay {
            return Err(format!(
                "facing {}: mode \"one_way\" is unavailable with limits that reach the passes: a stretch closed at both ends needs an at-depth link from another row; use mode \"zigzag\"",
                op.id
            ));
        }
        let route = around::route(&plan, &reaching, r, plan.floor_radius, cancel)?;
        plan.around = Some(route.steps);
        plan.report.strategy = "facing_round_limits";
        plan.report.entry_policy = "rows stopped at the limits and linked at depth where proven; vertical entries outside initial stock; one climb pass along each limit's edge";
        plan.report.nominal_uncovered_area_mm2 = route.report.unreachable_area_mm2;
        plan.report.around_limits = Some(route.report);
        Ok(plan)
    }
    /// The world coordinate across the rows at row `row`.
    fn row_across(&self, row: usize) -> f64 {
        if row + 1 == self.report.rows_per_pass {
            self.last_row
        } else {
            self.first_row + row as f64 * self.report.row_pitch_mm
        }
    }
    pub fn report(&self) -> &FacingReport {
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
            return Err("facing report cancelled".into());
        }
        if !self.drained {
            return Err("drain committed facing motions before final report".into());
        }
        self.report.geometric_residual_status = ResidualStatus::Estimated;
        Ok(())
    }
    fn row_point(&self, row: usize, z: f64, end: bool) -> Vec3 {
        let reverse = self.op.reverse ^ (self.op.mode == FacingMode::Zigzag && row % 2 == 1);
        let mut p = [0., 0., z];
        p[self.axis] = if reverse ^ end { self.high } else { self.low };
        p[1 - self.axis] = self.row_across(row);
        p
    }
    fn emit(&mut self, end: Vec3, kind: MoveKind) -> Result<Option<Motion>, String> {
        if end == self.position {
            return Ok(None);
        }
        let next_id = self
            .next_id
            .checked_add(1)
            .ok_or("facing motion IDs overflow")?;
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
        self.next_id = next_id;
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
            return Err("commit preceding facing motion to matching stock before planning".into());
        }
        let motion = self.next_motion(cancel)?;
        if motion.is_some() {
            self.expected_generation = Some(stock.generation() + 1);
        }
        Ok(motion)
    }

    /// Whether every move of this plan is proven allowed by `limits` (#411),
    /// by the same proof region operations and the run's link guard use: the
    /// whole cutter disk (radius `radius`) at each sampled position, each
    /// sample clearing by at least half its spacing. Containment applies to
    /// every feed move; avoid areas to every move whose tip is below their
    /// top. A move that cannot be proven (including one too long to prove at
    /// the minimum sample spacing) answers false; nothing is assumed.
    pub(crate) fn proven_within(
        &self,
        limits: &crate::limits::Limits,
        radius: f64,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        let prepared = crate::limits::PreparedLimits::new(limits, radius, self.op.tolerance_mm)?;
        let mut plan = self.clone();
        while let Some(motion) = plan.next_motion(cancel)? {
            let cutting = motion.kind != MoveKind::Rapid;
            if !prepared
                .allows_curve(&motion.curve, cutting)
                .unwrap_or(false)
            {
                return Ok(false);
            }
        }
        Ok(true)
    }

    fn route_seconds(mut self, rapid: f64, cancel: &impl CancelSignal) -> Result<f64, String> {
        let mut seconds = 0_f64;
        while let Some(motion) = self.next_motion(cancel)? {
            let (a, b) = (motion.curve.start(), motion.curve.end());
            let length = metallic::hypot(metallic::hypot(b[0] - a[0], b[1] - a[1]), b[2] - a[2]);
            seconds += 60. * length / motion.feed_mm_min.unwrap_or(rapid);
        }
        if seconds.is_finite() {
            Ok(seconds)
        } else {
            Err("automatic facing axis timing is not finite; check the rates".into())
        }
    }

    fn next_motion(&mut self, cancel: &impl CancelSignal) -> Result<Option<Motion>, String> {
        if let Some(steps) = &mut self.around {
            if cancel.is_cancelled() {
                return Err(PLANNING_CANCELLED.into());
            }
            let Some((curve, kind, count)) = steps.front().cloned() else {
                self.drained = true;
                return Ok(None);
            };
            if curve.start() != self.position {
                return Err("facing: internal planning error: a planned move does not start at the tool position".into());
            }
            let next_id = self
                .next_id
                .checked_add(1)
                .ok_or("facing motion IDs overflow")?;
            steps.pop_front();
            match count {
                around::Count::None => {}
                around::Count::Row => self.report.cut_rows_emitted += 1,
                around::Count::StayDown => self.report.side_links_emitted += 1,
                around::Count::Return => self.report.clearance_returns += 1,
            }
            let motion = Motion {
                id: self.next_id,
                operation: self.op.id.clone(),
                tool: self.op.tool.clone(),
                kind,
                feed_mm_min: match kind {
                    MoveKind::Rapid => None,
                    MoveKind::Plunge => Some(self.op.plunge_mm_min),
                    _ => Some(self.op.feed_mm_min),
                },
                curve,
            };
            self.position = motion.curve.end();
            self.next_id = next_id;
            return Ok(Some(motion));
        }
        loop {
            if cancel.is_cancelled() {
                return Err(PLANNING_CANCELLED.into());
            }
            let z = (self.spec.max_mm[2] - (self.pass + 1) as f64 * self.op.stepdown_mm)
                .max(self.op.floor_z_mm);
            let result = match self.phase {
                Phase::Lift => {
                    let m = self.emit(
                        [self.position[0], self.position[1], self.heights.clearance],
                        MoveKind::Rapid,
                    )?;
                    self.phase = Phase::Approach;
                    m
                }
                Phase::Approach => {
                    // Travel at the current rapid height (clearance first,
                    // then the retract height).
                    let m = self.emit(
                        self.row_point(self.row, self.position[2], false),
                        MoveKind::Rapid,
                    )?;
                    self.phase = Phase::Descend;
                    m
                }
                Phase::Descend => {
                    let m = self.emit(
                        [self.position[0], self.position[1], self.heights.feed],
                        MoveKind::Rapid,
                    )?;
                    self.phase = Phase::Entry;
                    m
                }
                Phase::Entry => {
                    let m = self.emit(self.row_point(self.row, z, false), MoveKind::Plunge)?;
                    self.phase = Phase::Cut;
                    m
                }
                Phase::Cut => {
                    let m = self.emit(self.row_point(self.row, z, true), MoveKind::Cut)?;
                    self.report.cut_rows_emitted += 1;
                    self.phase = if self.op.mode == FacingMode::Zigzag
                        && self.row + 1 < self.report.rows_per_pass
                    {
                        Phase::SideLink
                    } else {
                        Phase::Retract
                    };
                    m
                }
                Phase::SideLink => {
                    let m = self.emit(self.row_point(self.row + 1, z, false), MoveKind::Link)?;
                    self.report.side_links_emitted += 1;
                    self.row += 1;
                    self.phase = Phase::Cut;
                    m
                }
                Phase::Retract => {
                    let m = self.emit(
                        [self.position[0], self.position[1], self.heights.feed],
                        MoveKind::Retract,
                    )?;
                    self.report.clearance_returns += 1;
                    self.phase = Phase::Rise;
                    m
                }
                Phase::Rise => {
                    self.row += 1;
                    if self.row == self.report.rows_per_pass {
                        self.row = 0;
                        self.pass += 1;
                    }
                    let last = self.pass == self.report.depth_passes;
                    let z = if last {
                        self.heights.clearance
                    } else {
                        self.heights.retract
                    };
                    let m = self.emit([self.position[0], self.position[1], z], MoveKind::Rapid)?;
                    self.phase = if last { Phase::Done } else { Phase::Approach };
                    m
                }
                Phase::Done => {
                    self.drained = true;
                    return Ok(None);
                }
            };
            if result.is_some() {
                return Ok(result);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn auto_axis_requires_rapid_rate_and_breaks_symmetric_ties_to_x() {
        let mut manifest: crate::stream::StreamManifest =
            serde_json::from_str(include_str!("../fixtures/operations/facing.json")).unwrap();
        let crate::pocket::Operation::Facing(mut op) = manifest.operations.remove(0) else {
            panic!("facing fixture")
        };
        let tool = &manifest.tools[0];
        let mut spec = manifest.stock.height().unwrap().clone();
        spec.max_mm[1] = spec.max_mm[0];
        op.axis = TraverseAxis::Auto;
        let prepare = |op| FacingPlan::prepare(op, tool, &spec, [0., 0., 12.], 1, &|| false);
        assert!(
            prepare(op.clone())
                .err()
                .unwrap()
                .contains("auto_rapid_mm_min")
        );
        op.auto_rapid_mm_min = Some(f64::NAN);
        assert!(prepare(op.clone()).is_err());
        op.auto_rapid_mm_min = Some(6000.);
        let plan = prepare(op).unwrap();
        assert_eq!(plan.report().axis, TraverseAxis::X);
        let [Some(x), Some(y)] = plan.report().auto_axis_seconds.unwrap() else {
            panic!("two available axes")
        };
        assert!((x - y).abs() < 1e-9);
    }

    #[test]
    fn auto_axis_selects_x_at_the_one_nanosecond_boundary() {
        let mut manifest: crate::stream::StreamManifest =
            serde_json::from_str(include_str!("../fixtures/operations/facing.json")).unwrap();
        let crate::pocket::Operation::Facing(mut op) = manifest.operations.remove(0) else {
            panic!("facing fixture")
        };
        let mut spec = manifest.stock.height().unwrap().clone();
        spec.max_mm[0] = 24.;
        op.axis = TraverseAxis::Auto;
        op.auto_rapid_mm_min = Some(6000.);
        op.floor_z_mm = 9.;
        // On this square the initial diagonal rapid decides the near tie.
        let initial = [0.058823529411764705, 0.05882345679234425, 12.];
        let plan =
            FacingPlan::prepare(op, &manifest.tools[0], &spec, initial, 1, &|| false).unwrap();
        assert_eq!(plan.report().axis, TraverseAxis::X);
    }

    #[test]
    fn auto_facing_streams_the_explicit_route_when_an_axis_is_dense_or_out_of_range() {
        let m: crate::stream::StreamManifest =
            serde_json::from_str(include_str!("../fixtures/operations/facing.json")).unwrap();
        let crate::pocket::Operation::Facing(base) = &m.operations[0] else {
            panic!("facing fixture")
        };
        let tool = &m.tools[0];
        for (min, max, pitch, preferred) in [
            ([0., 0.], [1000., 10.], 0.01, TraverseAxis::X),
            (
                [999960., 0.],
                [999999., 24.],
                base.stepover_mm,
                TraverseAxis::Y,
            ),
        ] {
            let mut spec = m.stock.height().unwrap().clone();
            spec.min_mm[..2].copy_from_slice(&min);
            spec.max_mm[..2].copy_from_slice(&max);
            let prepare =
                |op| FacingPlan::prepare(op, tool, &spec, m.initial_position_mm, 1, &|| false);
            let mut op = base.clone();
            op.stepover_mm = pitch;
            op.axis = TraverseAxis::Auto;
            op.auto_rapid_mm_min = Some(6000.);
            let mut auto = prepare(op.clone()).unwrap();
            assert_eq!(auto.report().axis, preferred);
            let scores = auto.report().auto_axis_seconds.unwrap();
            if preferred == TraverseAxis::X {
                assert_eq!(auto.report().rows_per_pass, 362);
                assert!(scores[0].unwrap() < scores[1].unwrap());
                assert!(auto.report().auto_axis_refusals.is_none());
            } else {
                assert_eq!(auto.report().rows_per_pass, 7);
                assert!(scores[0].is_none());
                let [Some(x), None] = auto.report().auto_axis_refusals.as_ref().unwrap() else {
                    panic!("only X should be refused");
                };
                assert!(x.contains("coordinate overtravel"), "{x}");
            }
            op.axis = preferred;
            op.auto_rapid_mm_min = None;
            let mut explicit = prepare(op).unwrap();
            loop {
                let a = auto.next_motion(&|| false).unwrap();
                let b = explicit.next_motion(&|| false).unwrap();
                assert_eq!(
                    serde_json::to_value(&a).unwrap(),
                    serde_json::to_value(b).unwrap()
                );
                if a.is_none() {
                    break;
                }
            }
        }

        // Cancellation during the longer candidate is propagated even though X qualified.
        let mut spec = m.stock.height().unwrap().clone();
        spec.max_mm[..2].copy_from_slice(&[1000., 10.]);
        let mut op = base.clone();
        op.stepover_mm = 0.01;
        op.axis = TraverseAxis::Auto;
        op.auto_rapid_mm_min = Some(6000.);
        let polls = std::cell::Cell::new(0);
        let cancel = || {
            polls.set(polls.get() + 1);
            polls.get() >= 10_000
        };
        let error = FacingPlan::prepare(op, tool, &spec, m.initial_position_mm, 1, &cancel)
            .err()
            .unwrap();
        assert_eq!(error, PLANNING_CANCELLED);
    }

    #[test]
    fn exhausted_motion_ids_refuse_without_advancing_position() {
        let manifest: crate::stream::StreamManifest =
            serde_json::from_str(include_str!("../fixtures/operations/facing.json")).unwrap();
        let crate::pocket::Operation::Facing(operation) = manifest.operations[0].clone() else {
            panic!("facing fixture")
        };
        let assembly = &manifest.tools[0];
        let mut plan = FacingPlan::prepare(
            operation,
            assembly,
            manifest.stock.height().unwrap(),
            manifest.initial_position_mm,
            u64::MAX - 1,
            &|| false,
        )
        .unwrap();
        let mut stock = Stock::new(manifest.stock.height().unwrap().clone()).unwrap();
        let motion = plan
            .next_motion_with_stock(&stock, &|| false)
            .unwrap()
            .unwrap();
        assert_eq!(motion.id, u64::MAX - 1);
        let delta = stock
            .preview_sweep(
                &assembly.prepare().unwrap(),
                &motion,
                1.,
                manifest.sweep,
                &|| false,
            )
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        let position = plan.position;

        let error = plan.next_motion_with_stock(&stock, &|| false).unwrap_err();
        assert!(error.contains("motion IDs overflow"), "{error}");
        assert_eq!(plan.position, position);
        assert_eq!(plan.next_id, u64::MAX);
    }
}
