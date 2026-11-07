//! Rigid tapping of drilled holes. Each hole is one tapping pair of motions,
//! down from the R plane to the depth and back, at the synchronized feed
//! pitch x spindle speed. A post writes the pair as one controller cycle
//! (G33.1 on LinuxCNC, G84 ... G80 where declared) and its decoder reads the
//! cycle back into the same pair.
//!
//! A right-hand tap turns clockwise (M3) into the hole, a left-hand tap
//! counterclockwise (M4); the controller reverses the spindle for the return
//! (LinuxCNC G33.1 reverses whatever direction is running; FANUC G84 / G74,
//! B-63844EN/02 13.1.2 and 13.2). The current stock must show
//! each hole drilled below the tap depth plus the bottom clearance before
//! the tap goes down. The tap is modeled by its drill (minor) diameter, so
//! the stock keeps the material of the thread flanks: conservative.
use super::{Core, ResidualStatus, thread::column_open};
use crate::{
    Point,
    motion::{Coolant, Event, Motion, MoveKind, SpindleDirection, Vec3},
    stock::{CancelSignal, Stock, StockSpec},
    tool::{CutterShape, ToolAssembly},
};
use serde::{Deserialize, Serialize};

/// Thread hand. Right unless designated left: a thread designation without
/// "LH" is right-hand (ISO 965-1, ASME B1.1).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Hand {
    #[default]
    Right,
    Left,
}
impl Hand {
    fn is_right(&self) -> bool {
        *self == Hand::Right
    }
    /// The spindle direction that drives the tap into the hole.
    pub fn spindle(self) -> SpindleDirection {
        match self {
            Hand::Right => SpindleDirection::Cw,
            Hand::Left => SpindleDirection::Ccw,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TappingOperation {
    pub id: String,
    pub tool: String,
    pub centers_mm: Vec<Point>,
    pub pitch_mm: f64,
    /// The tap's thread hand: the spindle turns clockwise (right) or
    /// counterclockwise (left) into the hole.
    #[serde(default, skip_serializing_if = "Hand::is_right")]
    pub hand: Hand,
    /// Tap tip depth (absolute Z).
    pub depth_z_mm: f64,
    /// Space the hole must have below the tap tip (controller overrun,
    /// chips). Default: one pitch.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bottom_clearance_mm: Option<f64>,
    pub clearance_z_mm: f64,
    pub spindle_rpm: f64,
    pub tolerance_mm: f64,
    pub work_offset: u8,
    pub tool_number: u32,
    pub length_register: u32,
    pub coolant: Coolant,
    /// Rapid travel height between holes. Default: clearance.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub retract_z_mm: Option<f64>,
    /// The R plane: where each tapping cycle starts and ends (above the
    /// stock top). Default: retract.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub feed_z_mm: Option<f64>,
    /// Containment and avoid areas (clamps, fixtures, features to leave alone).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub limits: Option<crate::limits::Limits>,
}

#[derive(Debug, Serialize)]
pub struct TappingReport {
    pub strategy: &'static str,
    pub geometric_residual_status: ResidualStatus,
    pub holes: usize,
    pub hand: Hand,
    pub synchronized_feed_mm_min: f64,
    pub r_plane_z_mm: f64,
    pub depth_z_mm: f64,
    pub bottom_clearance_mm: f64,
    pub proven_open_radius_mm: f64,
    pub completed_holes: usize,
    pub stock_model: &'static str,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase {
    Lift,
    Traverse,
    Descend,
    Down,
    Up,
    Rise,
    Done,
}

pub struct TappingPlan {
    core: Core,
    heights: crate::heights::Heights,
    op: TappingOperation,
    phase: Phase,
    hole: usize,
    report: TappingReport,
}

impl TappingPlan {
    pub fn prepare(
        mut op: TappingOperation,
        assembly: &ToolAssembly,
        spec: &StockSpec,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        let tool = assembly.prepare()?;
        let def = tool.cutter().definition();
        let r = def.diameter_mm / 2.;
        let guard = 16. * op.tolerance_mm;
        let clearance = op.bottom_clearance_mm.unwrap_or(op.pitch_mm);
        let feed = op.pitch_mm * op.spindle_rpm;
        crate::heights::require(
            "tapping",
            [
                (
                    matches!(def.shape, CutterShape::Flat),
                    "tapping needs a tap modeled as a flat cutter of its drill (minor) diameter (camlib.tools.Tap)".into(),
                ),
                (
                    [op.pitch_mm, op.depth_z_mm, clearance]
                        .iter()
                        .all(|x| x.is_finite() && x.abs() <= 1e6),
                    "all numeric parameters must be finite".into(),
                ),
                (
                    op.pitch_mm > guard,
                    format!("pitch_mm {} must be positive", op.pitch_mm),
                ),
                (
                    clearance >= 0.,
                    format!("bottom_clearance_mm {clearance} must not be negative"),
                ),
                (
                    op.depth_z_mm - clearance >= spec.min_mm[2] && op.depth_z_mm < spec.max_mm[2],
                    format!(
                        "depth_z_mm {} (less bottom clearance {clearance}) must be within the stock Z range {}..{}",
                        op.depth_z_mm, spec.min_mm[2], spec.max_mm[2]
                    ),
                ),
                (
                    spec.max_mm[2] - op.depth_z_mm <= def.cutting_length_mm - guard,
                    format!(
                        "tap depth {} below the stock top exceeds the thread length {}",
                        spec.max_mm[2] - op.depth_z_mm,
                        def.cutting_length_mm
                    ),
                ),
            ],
        )?;
        let core = Core {
            id: op.id.clone(),
            tool: op.tool.clone(),
            centers: std::mem::take(&mut op.centers_mm),
            clearance: op.clearance_z_mm,
            feed,
            retract_feed: feed,
            tol: op.tolerance_mm,
            spindle: op.spindle_rpm,
            work_offset: op.work_offset,
            tool_number: op.tool_number,
            length_register: op.length_register,
            coolant: op.coolant,
            spec: spec.clone(),
            initial,
            position: initial,
            next_id: first_id,
            expected_generation: None,
            drained: false,
        };
        core.validate(assembly, r, cancel)?;
        let heights = crate::heights::Heights::resolve(
            op.clearance_z_mm,
            op.retract_z_mm,
            op.feed_z_mm,
            spec.max_mm[2],
            guard,
        )?;
        // The hole is proven open to within one stock cell of its wall.
        let cell = spec.cell_size_mm * std::f64::consts::SQRT_2;
        let report = TappingReport {
            strategy: "tapping",
            geometric_residual_status: ResidualStatus::Pending,
            holes: core.centers.len(),
            hand: op.hand,
            synchronized_feed_mm_min: feed,
            r_plane_z_mm: heights.feed,
            depth_z_mm: op.depth_z_mm,
            bottom_clearance_mm: clearance,
            proven_open_radius_mm: (r - cell).max(r / 2.),
            completed_holes: 0,
            stock_model: "tap as its drill (minor) diameter: thread flanks stay in the stock (conservative)",
        };
        Ok(Self {
            core,
            heights,
            op,
            phase: Phase::Lift,
            hole: 0,
            report,
        })
    }
    pub fn report(&self) -> &TappingReport {
        &self.report
    }
    pub fn prefix_events(&self) -> Vec<Event> {
        let mut events = self.core.prefix();
        for event in &mut events {
            if let Event::Spindle { direction, .. } = event {
                *direction = self.op.hand.spindle();
            }
        }
        events
    }
    pub fn suffix_events(&self) -> Vec<Event> {
        self.core.suffix()
    }
    pub fn finish_report(&mut self, cancel: &impl CancelSignal) -> Result<(), String> {
        if cancel.is_cancelled() {
            return Err("tapping report cancelled".into());
        }
        if !self.core.drained {
            return Err("drain committed tapping motions".into());
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
            if self.phase == Phase::Done {
                self.core.drained = true;
                return Ok(None);
            }
            let p = self.core.position;
            let c = self.core.centers[self.hole];
            let r_plane = self.heights.feed;
            let (curve, kind, next) = match self.phase {
                Phase::Lift => (
                    self.core.line([p[0], p[1], self.heights.clearance]),
                    MoveKind::Rapid,
                    Phase::Traverse,
                ),
                Phase::Traverse => (
                    self.core.line([c[0], c[1], p[2]]),
                    MoveKind::Rapid,
                    Phase::Descend,
                ),
                Phase::Descend => {
                    let low = self.op.depth_z_mm - self.report.bottom_clearance_mm;
                    let radius = self.report.proven_open_radius_mm;
                    if !column_open(stock, c, radius, low) {
                        return Err(format!(
                            "tapping hole {}: the stock is not proven open at ({:.4}, {:.4}) within radius {radius:.4} down to Z {low:.4}; drill the hole deeper than the tap depth plus bottom clearance first (or use a finer stock cell size)",
                            self.hole, c[0], c[1]
                        ));
                    }
                    (
                        self.core.line([c[0], c[1], r_plane]),
                        MoveKind::Rapid,
                        Phase::Down,
                    )
                }
                Phase::Down => (
                    self.core.line([c[0], c[1], self.op.depth_z_mm]),
                    MoveKind::Tap,
                    Phase::Up,
                ),
                Phase::Up => (
                    self.core.line([c[0], c[1], r_plane]),
                    MoveKind::Tap,
                    Phase::Rise,
                ),
                Phase::Rise => {
                    self.hole += 1;
                    self.report.completed_holes += 1;
                    let last = self.hole == self.core.centers.len();
                    let z = if last {
                        self.heights.clearance
                    } else {
                        self.heights.retract
                    };
                    (
                        self.core.line([p[0], p[1], z.max(p[2])]),
                        MoveKind::Rapid,
                        if last { Phase::Done } else { Phase::Traverse },
                    )
                }
                Phase::Done => unreachable!(),
            };
            self.phase = next;
            let feed = self.core.feed;
            if let Some(motion) = self.core.emit(curve, kind, feed, stock)? {
                return Ok(Some(motion));
            }
        }
    }
}
