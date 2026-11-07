//! Explicit drilling/peck and full-interior bore milling. No canned-cycle dialect
//! assumptions. Depth/coverage/reentry contract is in docs/holes.md.
use crate::{
    Point,
    motion::{Coolant, Curve, DrillCycle, Event, Motion, MoveKind, Plane, SpindleDirection, Vec3},
    pocket::ResidualStatus,
    stock::{CancelSignal, Stock, StockSpec},
    tool::{CutterShape, ToolAssembly, valid_id},
};
use serde::{Deserialize, Serialize};
use std::f64::consts::TAU;

mod ream;
mod tap;
mod thread;
pub use ream::{ReamingOperation, ReamingPlan, ReamingReport};
pub use tap::{Hand, TappingOperation, TappingPlan, TappingReport};
pub use thread::{ThreadMillOperation, ThreadMillPlan, ThreadMillReport};
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum DepthReference {
    Tip,
    FullDiameter,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum PeckMode {
    FullRetract,
    ChipBreak,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DrillingOperation {
    pub id: String,
    pub tool: String,
    pub centers_mm: Vec<Point>,
    pub depth_z_mm: f64,
    pub depth_reference: DepthReference,
    pub peck_mm: Option<f64>,
    pub peck_mode: PeckMode,
    pub retract_z_mm: Option<f64>,
    pub chip_break_lift_mm: f64,
    pub reentry_clearance_mm: f64,
    pub allow_breakthrough_mm: f64,
    pub clearance_z_mm: f64,
    pub feed_mm_min: f64,
    pub retract_feed_mm_min: f64,
    pub tolerance_mm: f64,
    pub spindle_rpm: f64,
    pub work_offset: u8,
    pub tool_number: u32,
    pub length_register: u32,
    pub coolant: Coolant,
    /// Dwell at the full depth of each hole (seconds), as the G82 cycle:
    /// spot drilling, counterboring.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dwell_s: Option<f64>,
    /// Move exactly as the controller's canned cycle (G81, G82 with dwell_s,
    /// G83 full-retract pecks, G73 chip-break pecks; G98 return): pecks are
    /// measured from the R plane and retracts are rapid. Each hole is
    /// announced by a cycle event, so a post can write one cycle block.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub canned_cycle: bool,
    /// Containment and avoid areas (clamps, fixtures, features to leave alone).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub limits: Option<crate::limits::Limits>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BoreOperation {
    pub id: String,
    pub tool: String,
    pub centers_mm: Vec<Point>,
    pub diameter_mm: f64,
    pub floor_z_mm: f64,
    pub stepdown_mm: f64,
    pub radial_stepover_mm: Option<f64>,
    pub radial_allowance_mm: f64,
    pub bottom_allowance_mm: f64,
    pub boundary_tolerance_mm: f64,
    pub clockwise: bool,
    pub entry_feed_mm_min: f64,
    pub clearance_z_mm: f64,
    pub feed_mm_min: f64,
    pub retract_feed_mm_min: f64,
    pub tolerance_mm: f64,
    pub spindle_rpm: f64,
    pub work_offset: u8,
    pub tool_number: u32,
    pub length_register: u32,
    pub coolant: Coolant,
    /// Rapid travel height between holes (absolute Z). Default: clearance.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub retract_z_mm: Option<f64>,
    /// Rapid descent stops here and entry feed begins; leaving a hole, feed
    /// changes to rapid here (absolute Z, above the stock top). Default: retract.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub feed_z_mm: Option<f64>,
    /// Containment and avoid areas (clamps, fixtures, features to leave alone).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub limits: Option<crate::limits::Limits>,
}
#[derive(Debug, Serialize)]
pub struct DrillingReport {
    pub strategy: &'static str,
    pub geometric_residual_status: ResidualStatus,
    pub holes: usize,
    pub depth_reference: DepthReference,
    pub tip_target_z_mm: f64,
    pub full_diameter_z_mm: f64,
    pub point_height_mm: f64,
    pub full_diameter_reached: bool,
    pub breakthrough_mm: f64,
    pub pecks_per_hole: usize,
    pub peck_mode: PeckMode,
    pub retract_plane_z_mm: f64,
    pub completed_holes: usize,
    pub cut_pecks: usize,
    pub peck_returns: usize,
    pub reentry_policy: &'static str,
}
#[derive(Debug, Serialize)]
pub struct BoreReport {
    pub strategy: &'static str,
    pub geometric_residual_status: ResidualStatus,
    pub holes: usize,
    pub target_z_mm: f64,
    pub depth_passes: usize,
    pub rings_per_layer: usize,
    pub requested_diameter_mm: f64,
    pub nominal_machined_diameter_mm: f64,
    pub path_safety_margin_mm: f64,
    pub radial_stepover_mm: f64,
    pub core_cleared_nominal: bool,
    pub completed_holes: usize,
    pub helical_turns: usize,
    pub level_circles: usize,
    pub coverage_policy: &'static str,
}
// Common immutable input points plus a scalar streaming cursor; no motion history.
struct Core {
    id: String,
    tool: String,
    centers: Vec<Point>,
    clearance: f64,
    feed: f64,
    retract_feed: f64,
    tol: f64,
    spindle: f64,
    work_offset: u8,
    tool_number: u32,
    length_register: u32,
    coolant: Coolant,
    spec: StockSpec,
    initial: Vec3,
    position: Vec3,
    next_id: u64,
    expected_generation: Option<u64>,
    drained: bool,
}
macro_rules! core {
    ($op:ident,$spec:expr,$initial:expr,$id:expr) => {
        Core {
            id: $op.id.clone(),
            tool: $op.tool.clone(),
            centers: std::mem::take(&mut $op.centers_mm),
            clearance: $op.clearance_z_mm,
            feed: $op.feed_mm_min,
            retract_feed: $op.retract_feed_mm_min,
            tol: $op.tolerance_mm,
            spindle: $op.spindle_rpm,
            work_offset: $op.work_offset,
            tool_number: $op.tool_number,
            length_register: $op.length_register,
            coolant: $op.coolant,
            spec: $spec.clone(),
            initial: $initial,
            position: $initial,
            next_id: $id,
            expected_generation: None,
            drained: false,
        }
    };
}
impl Core {
    fn validate(
        &self,
        assembly: &ToolAssembly,
        radius: f64,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        if cancel.is_cancelled() {
            return Err("hole planning cancelled".into());
        }
        self.spec.grid()?;
        let guard = 16. * self.tol;
        let outside = self.centers.iter().position(|p| {
            p.iter().enumerate().any(|(i, x)| {
                !x.is_finite()
                    || x.abs() > 1e6
                    || *x - radius < self.spec.min_mm[i] + guard
                    || *x + radius > self.spec.max_mm[i] - guard
            })
        });
        crate::heights::require(
            "hole",
            [
                (
                    valid_id(&self.id),
                    format!("id {:?} is not a valid identifier", self.id),
                ),
                (
                    self.tool == assembly.id,
                    format!(
                        "tool {:?} does not match the assembly {:?}",
                        self.tool, assembly.id
                    ),
                ),
                (
                    !self.centers.is_empty(),
                    "at least one hole center is needed".into(),
                ),
                (
                    [
                        self.clearance,
                        self.feed,
                        self.retract_feed,
                        self.tol,
                        self.spindle,
                    ]
                    .iter()
                    .all(|x| x.is_finite()),
                    "all numeric parameters must be finite".into(),
                ),
                (
                    (1e-7..=0.01).contains(&self.tol),
                    format!("tolerance_mm {} must be within 1e-7..0.01", self.tol),
                ),
                (
                    self.feed > 0. && self.retract_feed > 0. && self.spindle > 0.,
                    format!(
                        "feeds ({}, retract {}) and spindle_rpm {} must be positive",
                        self.feed, self.retract_feed, self.spindle
                    ),
                ),
                (
                    self.clearance > self.spec.max_mm[2] + guard && self.clearance.abs() <= 1e6,
                    format!(
                        "clearance_z_mm {} must be above the stock top {}",
                        self.clearance, self.spec.max_mm[2]
                    ),
                ),
                (
                    self.initial.iter().all(|x| x.is_finite() && x.abs() <= 1e6),
                    "start position must be finite".into(),
                ),
                (
                    self.initial[2] >= self.clearance,
                    format!(
                        "start Z {} is below clearance_z_mm {}",
                        self.initial[2], self.clearance
                    ),
                ),
                (
                    (1..=9).contains(&self.work_offset),
                    format!("work_offset {} must be 1..9", self.work_offset),
                ),
                (
                    self.tool_number > 0 && self.length_register > 0,
                    "tool_number and length_register must be positive".into(),
                ),
                (
                    self.next_id != 0 && self.next_id.checked_add(1).is_some(),
                    "motion ID budget exhausted".into(),
                ),
                (
                    outside.is_none(),
                    format!(
                        "hole center {} {:?} (with the cutter radius {radius}) is not inside the stock",
                        outside.unwrap_or(0),
                        outside.map(|i| self.centers[i])
                    ),
                ),
            ],
        )?;
        Ok(())
    }
    fn check(&self, stock: &Stock, cancel: &impl CancelSignal) -> Result<(), String> {
        if stock.spec() != &self.spec
            || stock.generation() == u64::MAX
            || self
                .expected_generation
                .is_some_and(|g| g != stock.generation())
        {
            return Err("commit preceding hole motion to matching stock".into());
        }
        if cancel.is_cancelled() {
            return Err("hole planning cancelled".into());
        }
        Ok(())
    }
    fn prefix(&self) -> Vec<Event> {
        vec![
            Event::WorkOffset {
                number: self.work_offset,
            },
            Event::ToolChange {
                tool: self.tool.clone(),
                number: self.tool_number,
                length_register: self.length_register,
                position_after_mm: self.initial,
            },
            Event::Spindle {
                rpm: self.spindle,
                direction: SpindleDirection::Cw,
            },
            Event::Coolant { mode: self.coolant },
        ]
    }
    fn suffix(&self) -> Vec<Event> {
        vec![
            Event::Coolant { mode: Coolant::Off },
            Event::Spindle {
                rpm: 0.,
                direction: SpindleDirection::Cw,
            },
        ]
    }
    fn emit(
        &mut self,
        curve: Curve,
        kind: MoveKind,
        feed: f64,
        stock: &Stock,
    ) -> Result<Option<Motion>, String> {
        if matches!(curve,Curve::Line {start,end} if start==end) {
            return Ok(None);
        }
        curve.prepare(self.tol)?;
        let m = Motion {
            id: self.next_id,
            operation: self.id.clone(),
            tool: self.tool.clone(),
            curve,
            kind,
            feed_mm_min: if kind.is_rapid() { None } else { Some(feed) },
        };
        self.position = m.curve.end();
        self.next_id += 1;
        self.expected_generation = Some(stock.generation() + 1);
        Ok(Some(m))
    }
    fn line(&self, end: Vec3) -> Curve {
        Curve::Line {
            start: self.position,
            end,
        }
    }
}
#[derive(Clone, Copy, PartialEq, Eq)]
enum DrillPhase {
    Lift,
    Traverse,
    Approach,
    Cut,
    PeckReturn,
    Reenter,
    Finish,
    Done,
}
pub struct DrillingPlan {
    core: Core,
    op: DrillingOperation,
    phase: DrillPhase,
    hole: usize,
    peck: usize,
    peck_size: f64,
    last_depth: f64,
    report: DrillingReport,
    events: Vec<Event>,
    /// Canned-cycle mode: the rest of the current hole (end, kind, dwell after).
    queue: std::collections::VecDeque<(Vec3, MoveKind, bool)>,
}
impl DrillingPlan {
    /// Events due after the last motion handed out (a dwell at full depth).
    pub fn take_events(&mut self) -> Vec<Event> {
        std::mem::take(&mut self.events)
    }
    pub fn prepare(
        mut op: DrillingOperation,
        assembly: &ToolAssembly,
        spec: &StockSpec,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        let tool = assembly.prepare()?;
        let cutter = tool.cutter();
        let def = cutter.definition();
        let radius = def.diameter_mm / 2.;
        let point_height = cutter
            .tip_height_at_radius(radius)
            .ok_or("missing drill point profile")?;
        let target = op.depth_z_mm
            - if op.depth_reference == DepthReference::FullDiameter {
                point_height
            } else {
                0.
            };
        let peck = op.peck_mm.unwrap_or(spec.max_mm[2] - target);
        let plane = op.retract_z_mm.unwrap_or(spec.max_mm[2] + 1.);
        let guard = 16. * op.tolerance_mm;
        crate::heights::require(
            "drilling",
            [
                (
                    def.center_cutting
                        && matches!(def.shape, CutterShape::Drill { .. } | CutterShape::Flat),
                    "drilling needs a drill or a flat, center-cutting end mill".into(),
                ),
                (
                    [
                        op.depth_z_mm,
                        target,
                        peck,
                        plane,
                        op.chip_break_lift_mm,
                        op.reentry_clearance_mm,
                        op.allow_breakthrough_mm,
                    ]
                    .iter()
                    .all(|x| x.is_finite()),
                    "all numeric parameters must be finite".into(),
                ),
                (
                    target < spec.max_mm[2] && op.depth_z_mm < spec.max_mm[2],
                    format!(
                        "depth_z_mm {} (tool tip Z{target}) must be below the stock top {}",
                        op.depth_z_mm, spec.max_mm[2]
                    ),
                ),
                (
                    op.allow_breakthrough_mm >= 0.,
                    format!(
                        "allow_breakthrough_mm {} must not be negative",
                        op.allow_breakthrough_mm
                    ),
                ),
                (
                    op.dwell_s
                        .is_none_or(|s| s.is_finite() && s > 0. && s <= 1e5),
                    format!(
                        "dwell_s {:?} must be a positive time in seconds (at most 100000)",
                        op.dwell_s
                    ),
                ),
                (
                    target >= spec.min_mm[2] - op.allow_breakthrough_mm && target.abs() <= 1e6,
                    format!(
                        "tool tip Z{target} goes through the stock bottom {} by more than allow_breakthrough_mm {}",
                        spec.min_mm[2], op.allow_breakthrough_mm
                    ),
                ),
                (
                    spec.max_mm[2] - target <= def.cutting_length_mm - guard,
                    format!(
                        "hole depth {} below the stock top exceeds the cutting length {}",
                        spec.max_mm[2] - target,
                        def.cutting_length_mm
                    ),
                ),
                (peck > guard, format!("peck_mm {peck} must be positive")),
                (
                    plane > spec.max_mm[2] + guard && plane <= op.clearance_z_mm,
                    format!(
                        "retract_z_mm (R plane) {plane} must be above the stock top {} and not above clearance_z_mm {}",
                        spec.max_mm[2], op.clearance_z_mm
                    ),
                ),
                (
                    op.chip_break_lift_mm > guard && op.reentry_clearance_mm > guard,
                    format!(
                        "chip_break_lift_mm {} and reentry_clearance_mm {} must be positive",
                        op.chip_break_lift_mm, op.reentry_clearance_mm
                    ),
                ),
                (
                    !op.canned_cycle || op.peck_mm.is_none() || op.dwell_s.is_none(),
                    "canned pecking cycles (G83/G73) have no dwell: set dwell_s or peck_mm, not both".into(),
                ),
                (
                    !op.canned_cycle
                        || op.peck_mm.is_none()
                        || match op.peck_mode {
                            PeckMode::FullRetract => op.reentry_clearance_mm < peck,
                            PeckMode::ChipBreak => op.chip_break_lift_mm < peck,
                        },
                    format!(
                        "canned pecking: the controller's return amount (reentry_clearance_mm {} or chip_break_lift_mm {}) must be less than peck_mm {peck}",
                        op.reentry_clearance_mm, op.chip_break_lift_mm
                    ),
                ),
            ],
        )?;
        let core = core!(op, spec, initial, first_id);
        core.validate(assembly, radius, cancel)?;
        let pecks = ((spec.max_mm[2] - target) / peck).ceil() as usize;
        if pecks == 0 {
            return Err("drilling needs at least one peck".into());
        }
        let report = DrillingReport {
            strategy: "drilling",
            geometric_residual_status: ResidualStatus::Pending,
            holes: core.centers.len(),
            depth_reference: op.depth_reference,
            tip_target_z_mm: target,
            full_diameter_z_mm: target + point_height,
            point_height_mm: point_height,
            full_diameter_reached: target + point_height <= spec.max_mm[2],
            breakthrough_mm: (spec.min_mm[2] - target).max(0.),
            pecks_per_hole: pecks,
            peck_mode: op.peck_mode,
            retract_plane_z_mm: plane,
            completed_holes: 0,
            cut_pecks: 0,
            peck_returns: 0,
            reentry_policy: if op.canned_cycle {
                "canned cycle: pecks from the R plane, rapid retract and re-entry as the controller's G83/G73; XY rapids only at clearance"
            } else {
                "fed same-axis reentry; XY rapids only at clearance; nominal wall tangency is not a strict clearance certificate"
            },
        };
        let canned = op.canned_cycle;
        let mut plan = Self {
            core,
            op,
            phase: DrillPhase::Lift,
            hole: 0,
            peck: 0,
            peck_size: peck,
            last_depth: spec.max_mm[2],
            events: vec![],
            queue: Default::default(),
            report,
        };
        if canned {
            plan.report.pecks_per_hole = plan
                .canned_moves([0.; 2])
                .0
                .iter()
                .filter(|m| m.1 == MoveKind::Plunge)
                .count();
            // The cycle starts at the clearance level: the first hole's event
            // is due before any motion when no lift is needed.
            if plan.core.position[2] == plan.core.clearance {
                plan.phase = DrillPhase::Traverse;
                plan.start_hole();
            }
        }
        Ok(plan)
    }
    /// The canned-cycle moves of one hole from the clearance level, exactly
    /// as the controller runs them (LinuxCNC interp_cycles.cc, FANUC and
    /// Centroid G81/G82/G83/G73 with G98), and the cycle type.
    fn canned_moves(&self, c: Point) -> (Vec<(Vec3, MoveKind, bool)>, DrillCycle) {
        let (z0, r, z) = (
            self.core.clearance,
            self.report.retract_plane_z_mm,
            self.report.tip_target_z_mm,
        );
        let at = |h: f64| [c[0], c[1], h];
        let mut moves = vec![
            (at(z0), MoveKind::Rapid, false),
            (at(r), MoveKind::Rapid, false),
        ];
        let cycle = match (self.op.peck_mm, self.op.peck_mode) {
            (None, _) if self.op.dwell_s.is_some() => DrillCycle::Dwell,
            (None, _) => DrillCycle::Drill,
            (Some(_), PeckMode::FullRetract) => DrillCycle::Peck,
            (Some(_), PeckMode::ChipBreak) => DrillCycle::ChipBreak,
        };
        if let Some(q) = self.op.peck_mm {
            let mut depth = r - q;
            while depth > z {
                moves.push((at(depth), MoveKind::Plunge, false));
                if cycle == DrillCycle::Peck {
                    moves.push((at(r), MoveKind::Rapid, false));
                    moves.push((
                        at(depth + self.op.reentry_clearance_mm),
                        MoveKind::Rapid,
                        false,
                    ));
                } else {
                    moves.push((
                        at(depth + self.op.chip_break_lift_mm),
                        MoveKind::Rapid,
                        false,
                    ));
                }
                depth -= q;
            }
        }
        moves.push((at(z), MoveKind::Plunge, self.op.dwell_s.is_some()));
        moves.push((at(z0), MoveKind::Rapid, false));
        (moves, cycle)
    }
    /// Queue the current hole's canned-cycle moves and announce the cycle.
    fn start_hole(&mut self) {
        let c = self.core.centers[self.hole];
        let (moves, cycle) = self.canned_moves(c);
        let mut position = self.core.position;
        self.queue.clear();
        for m in moves {
            if m.0 != position {
                position = m.0;
                self.queue.push_back(m);
            }
        }
        let covers = self.queue.len() + usize::from(self.op.dwell_s.is_some());
        self.events.push(Event::Cycle {
            cycle,
            center_mm: c,
            z_mm: self.report.tip_target_z_mm,
            r_mm: self.report.retract_plane_z_mm,
            initial_z_mm: self.core.position[2],
            feed_mm_min: self.core.feed,
            peck_mm: self.op.peck_mm,
            dwell_s: self.op.dwell_s,
            reentry_mm: (cycle == DrillCycle::Peck).then_some(self.op.reentry_clearance_mm),
            lift_mm: (cycle == DrillCycle::ChipBreak).then_some(self.op.chip_break_lift_mm),
            shift_mm: None,
            covers: covers as u32,
        });
    }
    fn next_canned(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        self.core.check(stock, cancel)?;
        if self.phase == DrillPhase::Done {
            self.core.drained = true;
            return Ok(None);
        }
        if self.phase == DrillPhase::Lift {
            let p = self.core.position;
            let m = self.core.emit(
                self.core.line([p[0], p[1], self.core.clearance]),
                MoveKind::Rapid,
                self.core.feed,
                stock,
            )?;
            self.phase = DrillPhase::Traverse;
            self.start_hole();
            return Ok(m);
        }
        let (end, kind, dwell) = self.queue.pop_front().ok_or("canned cycle queue empty")?;
        let rising = end[2] > self.core.position[2];
        let m = self
            .core
            .emit(self.core.line(end), kind, self.core.feed, stock)?;
        if kind == MoveKind::Plunge {
            self.report.cut_pecks += 1;
        } else if rising && !self.queue.is_empty() {
            // Every rise but the final retract leaves a peck.
            self.report.peck_returns += 1;
        }
        if dwell && let Some(seconds) = self.op.dwell_s {
            self.events.push(Event::Dwell { seconds });
        }
        if self.queue.is_empty() {
            self.report.completed_holes += 1;
            self.hole += 1;
            if self.hole == self.core.centers.len() {
                self.phase = DrillPhase::Done;
            } else {
                self.start_hole();
            }
        }
        Ok(m)
    }
    pub fn report(&self) -> &DrillingReport {
        &self.report
    }
    pub fn prefix_events(&self) -> Vec<Event> {
        self.core.prefix()
    }
    pub fn suffix_events(&self) -> Vec<Event> {
        self.core.suffix()
    }
    pub fn finish_report(&mut self, cancel: &impl CancelSignal) -> Result<(), String> {
        if cancel.is_cancelled() {
            return Err("drilling report cancelled".into());
        }
        if !self.core.drained {
            return Err("drain committed drilling motions".into());
        }
        self.report.geometric_residual_status = ResidualStatus::Estimated;
        Ok(())
    }
    pub fn next_motion_with_stock(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        if self.op.canned_cycle {
            return self.next_canned(stock, cancel);
        }
        loop {
            self.core.check(stock, cancel)?;
            if self.phase == DrillPhase::Done {
                self.core.drained = true;
                return Ok(None);
            }
            let c = self.core.centers[self.hole];
            let top = self.core.spec.max_mm[2];
            let (end, kind, feed, next) = match self.phase {
                DrillPhase::Lift => (
                    [
                        self.core.position[0],
                        self.core.position[1],
                        self.core.clearance,
                    ],
                    MoveKind::Rapid,
                    self.core.feed,
                    DrillPhase::Traverse,
                ),
                DrillPhase::Traverse => (
                    [c[0], c[1], self.core.clearance],
                    MoveKind::Rapid,
                    self.core.feed,
                    DrillPhase::Approach,
                ),
                DrillPhase::Approach => (
                    [c[0], c[1], self.report.retract_plane_z_mm],
                    MoveKind::Rapid,
                    self.core.feed,
                    DrillPhase::Cut,
                ),
                DrillPhase::Cut => (
                    [
                        c[0],
                        c[1],
                        (top - (self.peck + 1) as f64 * self.peck_size)
                            .max(self.report.tip_target_z_mm),
                    ],
                    MoveKind::Plunge,
                    self.core.feed,
                    DrillPhase::PeckReturn,
                ),
                DrillPhase::PeckReturn => (
                    [
                        c[0],
                        c[1],
                        if self.op.peck_mode == PeckMode::FullRetract {
                            self.report.retract_plane_z_mm
                        } else {
                            (self.last_depth + self.op.chip_break_lift_mm)
                                .min(self.report.retract_plane_z_mm)
                        },
                    ],
                    MoveKind::Retract,
                    self.core.retract_feed,
                    DrillPhase::Reenter,
                ),
                DrillPhase::Reenter => (
                    [
                        c[0],
                        c[1],
                        self.core.position[2].min(self.last_depth + self.op.reentry_clearance_mm),
                    ],
                    MoveKind::Link,
                    self.core.retract_feed,
                    DrillPhase::Cut,
                ),
                DrillPhase::Finish => (
                    [c[0], c[1], self.core.clearance],
                    MoveKind::Retract,
                    self.core.retract_feed,
                    DrillPhase::Traverse,
                ),
                DrillPhase::Done => unreachable!(),
            };
            let phase = self.phase;
            let m = self.core.emit(self.core.line(end), kind, feed, stock)?;
            self.phase = next;
            if phase == DrillPhase::Cut {
                self.peck += 1;
                self.last_depth = end[2];
                self.report.cut_pecks += 1;
                if self.peck == self.report.pecks_per_hole {
                    self.phase = DrillPhase::Finish;
                    if let Some(seconds) = self.op.dwell_s {
                        self.events.push(Event::Dwell { seconds });
                    }
                }
            }
            if phase == DrillPhase::PeckReturn {
                self.report.peck_returns += 1;
            }
            if phase == DrillPhase::Finish {
                self.hole += 1;
                self.peck = 0;
                self.last_depth = top;
                self.report.completed_holes += 1;
                if self.hole == self.core.centers.len() {
                    self.phase = DrillPhase::Done;
                }
            }
            if m.is_some() {
                return Ok(m);
            }
        }
    }
}
#[derive(Clone, Copy, PartialEq, Eq)]
enum BorePhase {
    Lift,
    Traverse,
    Entry,
    Seed,
    Helix,
    Circle,
    Widen,
    Center,
    Finish,
    /// Feed height -> retract height (clearance after the last hole).
    Rise,
    /// Travel height -> feed height above the next hole.
    Descend,
    Done,
}
pub struct BorePlan {
    core: Core,
    heights: crate::heights::Heights,
    op: BoreOperation,
    phase: BorePhase,
    hole: usize,
    layer: usize,
    ring: usize,
    inner: f64,
    outer: f64,
    report: BoreReport,
}
impl BorePlan {
    pub fn prepare(
        mut op: BoreOperation,
        assembly: &ToolAssembly,
        spec: &StockSpec,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        let tool = assembly.prepare()?;
        let def = tool.cutter().definition();
        let r = def.diameter_mm / 2.;
        let margin = op.boundary_tolerance_mm / 2.;
        let outer = op.diameter_mm / 2. - op.radial_allowance_mm - r - margin;
        let step = op.radial_stepover_mm.unwrap_or(r);
        let target = op.floor_z_mm + op.bottom_allowance_mm;
        let guard = 16. * op.tolerance_mm;
        crate::heights::require(
            "bore",
            [
                (
                    def.center_cutting && matches!(def.shape, CutterShape::Flat),
                    "bore milling needs a flat, center-cutting end mill".into(),
                ),
                (
                    [
                        op.diameter_mm,
                        margin,
                        outer,
                        step,
                        target,
                        op.floor_z_mm,
                        op.radial_allowance_mm,
                        op.bottom_allowance_mm,
                        op.stepdown_mm,
                        op.entry_feed_mm_min,
                    ]
                    .iter()
                    .all(|x| x.is_finite()),
                    "all numeric parameters must be finite".into(),
                ),
                (
                    (0.0001..=0.1).contains(&op.boundary_tolerance_mm)
                        && op.tolerance_mm <= op.boundary_tolerance_mm / 64.,
                    format!(
                        "boundary_tolerance_mm {} must be within 0.0001..0.1 and at least 64 x tolerance_mm {}",
                        op.boundary_tolerance_mm, op.tolerance_mm
                    ),
                ),
                (
                    outer > guard,
                    format!(
                        "diameter_mm {} leaves no room for the tool path: it must exceed the cutter diameter {} plus allowances and margin",
                        op.diameter_mm, def.diameter_mm
                    ),
                ),
                (
                    step > guard && step <= r,
                    format!(
                        "radial_stepover_mm {step} must be positive and at most the cutter radius {r}"
                    ),
                ),
                (
                    op.radial_allowance_mm >= 0. && op.bottom_allowance_mm >= 0.,
                    "allowances must not be negative".into(),
                ),
                (
                    op.floor_z_mm >= spec.min_mm[2] && target < spec.max_mm[2],
                    format!(
                        "floor_z_mm {} plus bottom allowance must be within the stock Z range {}..{}",
                        op.floor_z_mm, spec.min_mm[2], spec.max_mm[2]
                    ),
                ),
                (
                    spec.max_mm[2] - target <= def.cutting_length_mm - guard,
                    format!(
                        "bore depth {} below the stock top exceeds the cutting length {}",
                        spec.max_mm[2] - target,
                        def.cutting_length_mm
                    ),
                ),
                (
                    op.stepdown_mm > guard && op.stepdown_mm <= def.cutting_length_mm,
                    format!(
                        "stepdown_mm {} must be positive and at most the cutting length {}",
                        op.stepdown_mm, def.cutting_length_mm
                    ),
                ),
                (
                    op.entry_feed_mm_min > 0.,
                    format!(
                        "entry_feed_mm_min {} must be positive",
                        op.entry_feed_mm_min
                    ),
                ),
            ],
        )?;
        let core = core!(op, spec, initial, first_id);
        core.validate(assembly, op.diameter_mm / 2., cancel)?;
        let layers = ((spec.max_mm[2] - target) / op.stepdown_mm).ceil() as usize;
        let rings = (outer / step).ceil() as usize;
        if layers == 0 || rings == 0 {
            return Err("bore needs at least one layer and ring".into());
        }
        let report = BoreReport {
            strategy: "bore",
            geometric_residual_status: ResidualStatus::Pending,
            holes: core.centers.len(),
            target_z_mm: target,
            depth_passes: layers,
            rings_per_layer: rings,
            requested_diameter_mm: op.diameter_mm,
            nominal_machined_diameter_mm: 2. * (outer + r),
            path_safety_margin_mm: margin,
            radial_stepover_mm: step,
            core_cleared_nominal: true,
            completed_holes: 0,
            helical_turns: 0,
            level_circles: 0,
            coverage_policy: "seed orbit <= cutter radius; overlapping radial bands and full floor circles; no pre-drilled core assumed",
        };
        let heights = crate::heights::Heights::resolve(
            op.clearance_z_mm,
            op.retract_z_mm,
            op.feed_z_mm,
            spec.max_mm[2],
            16. * op.tolerance_mm,
        )?;
        Ok(Self {
            heights,
            core,
            op,
            phase: BorePhase::Lift,
            hole: 0,
            layer: 0,
            ring: 0,
            inner: outer.min(step),
            outer,
            report,
        })
    }
    pub fn report(&self) -> &BoreReport {
        &self.report
    }
    pub fn prefix_events(&self) -> Vec<Event> {
        self.core.prefix()
    }
    pub fn suffix_events(&self) -> Vec<Event> {
        self.core.suffix()
    }
    pub fn finish_report(&mut self, cancel: &impl CancelSignal) -> Result<(), String> {
        if cancel.is_cancelled() {
            return Err("bore report cancelled".into());
        }
        if !self.core.drained {
            return Err("drain committed bore motions".into());
        }
        self.report.geometric_residual_status = ResidualStatus::Estimated;
        Ok(())
    }
    pub fn next_motion_with_stock(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        loop {
            self.core.check(stock, cancel)?;
            if self.phase == BorePhase::Done {
                self.core.drained = true;
                return Ok(None);
            }
            let c = self.core.centers[self.hole];
            let top = self.core.spec.max_mm[2];
            let previous = top - self.layer as f64 * self.op.stepdown_mm;
            let z =
                (top - (self.layer + 1) as f64 * self.op.stepdown_mm).max(self.report.target_z_mm);
            let sweep = if self.op.clockwise { -TAU } else { TAU };
            let (curve, kind, feed, next) = match self.phase {
                BorePhase::Lift => (
                    self.core.line([
                        self.core.position[0],
                        self.core.position[1],
                        self.core.clearance,
                    ]),
                    MoveKind::Rapid,
                    self.core.feed,
                    BorePhase::Traverse,
                ),
                BorePhase::Traverse => (
                    self.core.line([c[0], c[1], self.core.position[2]]),
                    MoveKind::Rapid,
                    self.core.feed,
                    BorePhase::Descend,
                ),
                BorePhase::Descend => (
                    self.core.line([c[0], c[1], self.heights.feed]),
                    MoveKind::Rapid,
                    self.core.feed,
                    BorePhase::Entry,
                ),
                BorePhase::Entry => (
                    self.core.line([c[0], c[1], top]),
                    MoveKind::Plunge,
                    self.op.entry_feed_mm_min,
                    BorePhase::Seed,
                ),
                BorePhase::Seed => (
                    self.core.line([c[0] + self.inner, c[1], previous]),
                    MoveKind::Link,
                    self.core.feed,
                    BorePhase::Helix,
                ),
                BorePhase::Helix => (
                    Curve::Helix {
                        start: self.core.position,
                        end: [c[0] + self.inner, c[1], z],
                        center: [c[0], c[1], previous],
                        plane: Plane::Xy,
                        sweep_rad: sweep,
                    },
                    MoveKind::Lead,
                    self.core.feed,
                    BorePhase::Circle,
                ),
                BorePhase::Circle => (
                    Curve::Arc {
                        start: self.core.position,
                        end: self.core.position,
                        center: [c[0], c[1], z],
                        plane: Plane::Xy,
                        sweep_rad: sweep,
                    },
                    MoveKind::Cut,
                    self.core.feed,
                    BorePhase::Widen,
                ),
                BorePhase::Widen => (
                    self.core.line([
                        c[0] + ((self.ring + 2) as f64 * self.report.radial_stepover_mm)
                            .min(self.outer),
                        c[1],
                        z,
                    ]),
                    MoveKind::Cut,
                    self.core.feed,
                    BorePhase::Circle,
                ),
                BorePhase::Center => (
                    self.core.line([c[0], c[1], z]),
                    MoveKind::Link,
                    self.core.feed,
                    BorePhase::Seed,
                ),
                BorePhase::Finish => (
                    self.core.line([c[0], c[1], self.heights.feed]),
                    MoveKind::Retract,
                    self.core.retract_feed,
                    BorePhase::Rise,
                ),
                BorePhase::Rise => {
                    let last = self.hole + 1 == self.core.centers.len();
                    let z = if last {
                        self.heights.clearance
                    } else {
                        self.heights.retract
                    };
                    (
                        self.core.line([c[0], c[1], z]),
                        MoveKind::Rapid,
                        self.core.feed,
                        BorePhase::Traverse,
                    )
                }
                BorePhase::Done => unreachable!(),
            };
            let phase = self.phase;
            let m = self.core.emit(curve, kind, feed, stock)?;
            self.phase = next;
            if phase == BorePhase::Helix {
                self.report.helical_turns += 1;
            }
            if phase == BorePhase::Circle {
                self.report.level_circles += 1;
                if self.ring + 1 == self.report.rings_per_layer {
                    self.phase = BorePhase::Center;
                }
            }
            if phase == BorePhase::Widen {
                self.ring += 1;
            }
            if phase == BorePhase::Center {
                self.layer += 1;
                self.ring = 0;
                if self.layer == self.report.depth_passes {
                    self.phase = BorePhase::Finish;
                }
            }
            if phase == BorePhase::Rise {
                self.hole += 1;
                self.layer = 0;
                self.ring = 0;
                self.report.completed_holes += 1;
                if self.hole == self.core.centers.len() {
                    self.phase = BorePhase::Done;
                }
            }
            if m.is_some() {
                return Ok(m);
            }
        }
    }
}
