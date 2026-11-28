//! Reaming and boring of existing holes, as the usual G85 cycle: feed down
//! from the R plane to the depth, feed back out (G89 with a dwell at depth;
//! G86 stops the spindle at depth and rapids out). Reamers and boring bars do
//! not cut at their center: the current stock must show each pilot hole open
//! below the depth before the tool goes down. With `canned_cycle` each hole
//! runs from the R plane as the controller's cycle, announced by a cycle event.
//!
//! Fine boring (G76) stops the spindle at an oriented angle at depth, shifts
//! the bar away from the wall, rapids out to R, shifts back and restarts the
//! spindle. The bar is modeled with a neck of the bar diameter; the shift plus
//! the bar radius must fit inside the bore radius (proven again by the
//! stock-contact check of every shift).
use super::{Core, ResidualStatus, thread::column_open};
use crate::{
    Point,
    motion::{Coolant, DrillCycle, Event, Motion, MoveKind, SpindleDirection, Vec3},
    stock::{CancelSignal, Stock, StockSpec},
    tool::{CutterShape, ToolAssembly},
};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReamingOperation {
    pub id: String,
    pub tool: String,
    pub centers_mm: Vec<Point>,
    /// The existing (pilot) hole diameter, at most the tool diameter.
    pub pilot_diameter_mm: f64,
    /// Tool tip depth (absolute Z).
    pub depth_z_mm: f64,
    pub clearance_z_mm: f64,
    pub feed_mm_min: f64,
    /// Feed out of the hole (as G85; a reamer usually leaves at the same feed).
    pub retract_feed_mm_min: f64,
    pub spindle_rpm: f64,
    pub tolerance_mm: f64,
    pub work_offset: u8,
    pub tool_number: u32,
    pub length_register: u32,
    pub coolant: Coolant,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub retract_z_mm: Option<f64>,
    /// The R plane where feed starts and ends. Default: retract.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub feed_z_mm: Option<f64>,
    /// Dwell at depth (seconds), as G89.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dwell_s: Option<f64>,
    /// Stop the spindle at depth and leave at rapid, as G86 (a boring bar
    /// leaves a light line where the tool stops; feed out marks the bore).
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub spindle_stop: bool,
    /// Run each hole from the R plane as the controller's G85/G89/G86 cycle;
    /// the R plane is then also the height between holes.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub canned_cycle: bool,
    /// Containment and avoid areas (clamps, fixtures, features to leave alone).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub limits: Option<crate::limits::Limits>,
    /// Fine boring (G76): oriented stop and shift at depth, rapid out.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fine_bore: Option<FineBore>,
}

/// Oriented spindle stop at the bottom of each bore, then a shift away from
/// the wall before the rapid out, so the stopped tool leaves no mark.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FineBore {
    /// Q: shift away from the wall at the bottom.
    pub shift_mm: f64,
    /// Direction the cutting edge points after the oriented stop (work XY,
    /// degrees from +X). The shift goes the opposite way. A setup premise:
    /// it must match the machine's orientation and G76 shift direction
    /// (FANUC parameter 5101 RD1/RD2, Centroid parameter 136), which the
    /// post declares.
    pub edge_angle_deg: f64,
}

#[derive(Debug, Serialize)]
pub struct ReamingReport {
    pub strategy: &'static str,
    pub geometric_residual_status: ResidualStatus,
    pub holes: usize,
    pub r_plane_z_mm: f64,
    pub depth_z_mm: f64,
    pub proven_open_radius_mm: f64,
    pub completed_holes: usize,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase {
    Lift,
    Traverse,
    Descend,
    In,
    Out,
    ShiftUp,
    ShiftBack,
    Rise,
    Done,
}

/// One planned move: end, kind, feed, events due after it, and whether the
/// pilot hole must be proven open first.
type Step = (Vec3, MoveKind, f64, Vec<Event>, bool);

pub struct ReamingPlan {
    core: Core,
    heights: crate::heights::Heights,
    op: ReamingOperation,
    phase: Phase,
    hole: usize,
    report: ReamingReport,
    events: Vec<Event>,
    /// Canned mode: the moves still to make (lead-in, a hole, or the exit).
    queue: std::collections::VecDeque<Step>,
}

impl ReamingPlan {
    pub fn prepare(
        mut op: ReamingOperation,
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
        let reach = def.cutting_length_mm + def.neck.as_ref().map_or(0., |n| n.length_mm);
        crate::heights::require(
            "reaming",
            [
                (
                    matches!(def.shape, CutterShape::Flat),
                    "reaming needs a reamer or boring bar modeled as a flat cutter of its diameter"
                        .into(),
                ),
                (
                    [op.pilot_diameter_mm, op.depth_z_mm]
                        .iter()
                        .all(|x| x.is_finite() && x.abs() <= 1e6),
                    "all numeric parameters must be finite".into(),
                ),
                (
                    op.pilot_diameter_mm > guard && op.pilot_diameter_mm <= def.diameter_mm,
                    format!(
                        "pilot_diameter_mm {} must be positive and at most the tool diameter {}",
                        op.pilot_diameter_mm, def.diameter_mm
                    ),
                ),
                (
                    op.depth_z_mm >= spec.min_mm[2] && op.depth_z_mm < spec.max_mm[2],
                    format!(
                        "depth_z_mm {} must be within the stock Z range {}..{}",
                        op.depth_z_mm, spec.min_mm[2], spec.max_mm[2]
                    ),
                ),
                (
                    // A neck (boring bar) follows the cutter down the bore it
                    // cuts; stock contact checks it on every move.
                    spec.max_mm[2] - op.depth_z_mm <= reach - guard,
                    format!(
                        "depth {} below the stock top exceeds the reach {reach} (cutting length plus neck)",
                        spec.max_mm[2] - op.depth_z_mm,
                    ),
                ),
                (
                    // Numeric safety only: positive and finite. The control's
                    // dwell range is its dialect's declaration
                    // (GCodeDialect.dwell_max_s), checked when the dwell is
                    // written.
                    op.dwell_s.is_none_or(|s| s.is_finite() && s > 0.),
                    format!(
                        "{}: dwell_s {:?} must be a positive finite time in seconds",
                        op.id, op.dwell_s
                    ),
                ),
                (
                    !(op.canned_cycle && op.spindle_stop && op.dwell_s.is_some()),
                    "the canned stop-boring cycle (G86) has no dwell on every controller: drop dwell_s or spindle_stop".into(),
                ),
                (
                    !(op.fine_bore.is_some() && op.spindle_stop),
                    "fine_bore already stops the spindle (oriented); drop spindle_stop".into(),
                ),
            ],
        )?;
        if let Some(f) = op.fine_bore {
            let bar = def.neck.as_ref().map(|n| n.diameter_mm / 2.);
            let room = bar.map(|b| r - b - guard);
            if !(f.shift_mm.is_finite()
                && f.edge_angle_deg.is_finite()
                && f.edge_angle_deg.abs() <= 360.
                && f.shift_mm >= guard
                && room.is_some_and(|room| f.shift_mm <= room))
            {
                return Err(match bar {
                    None => "fine_bore needs the boring bar modeled with a neck of the bar diameter (camlib.BoringBar), so the shift can be proven to stay in the bore".into(),
                    Some(b) => format!(
                        "fine_bore shift_mm {} must be at least {guard} and at most {} (bore radius {r} - bar radius {b} - guard {guard}); edge_angle_deg {} must be within +-360",
                        f.shift_mm, r - b - guard, f.edge_angle_deg
                    ),
                });
            }
        }
        let core = Core {
            id: op.id.clone(),
            tool: op.tool.clone(),
            centers: std::mem::take(&mut op.centers_mm),
            clearance: op.clearance_z_mm,
            feed: op.feed_mm_min,
            retract_feed: op.retract_feed_mm_min,
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
        let pilot = op.pilot_diameter_mm / 2.;
        let cell = spec.cell_size_mm * std::f64::consts::SQRT_2;
        let report = ReamingReport {
            strategy: "reaming",
            geometric_residual_status: ResidualStatus::Pending,
            holes: core.centers.len(),
            r_plane_z_mm: heights.feed,
            depth_z_mm: op.depth_z_mm,
            proven_open_radius_mm: (pilot - cell).max(pilot / 2.),
            completed_holes: 0,
        };
        if op.canned_cycle && heights.feed != heights.retract {
            return Err(format!(
                "reaming: canned cycles start and end each hole at the R plane; set feed_z_mm {} equal to retract_z_mm {} (or leave it unset)",
                heights.feed, heights.retract
            ));
        }
        let mut plan = Self {
            core,
            heights,
            op,
            phase: Phase::Lift,
            hole: 0,
            report,
            events: vec![],
            queue: Default::default(),
        };
        if plan.op.canned_cycle {
            // Lead-in: up to clearance, over the first hole, down to R.
            let (p, c, r) = (plan.core.position, plan.core.centers[0], plan.heights.feed);
            let clearance = plan.heights.clearance;
            let mut at = p;
            for end in [
                [p[0], p[1], clearance],
                [c[0], c[1], clearance],
                [c[0], c[1], r],
            ] {
                if end != at {
                    plan.queue
                        .push_back((end, MoveKind::Rapid, plan.core.feed, vec![], false));
                    at = end;
                }
            }
            // Traverse: lead-in still queued; In: holes.
            plan.phase = Phase::Traverse;
            if plan.queue.is_empty() {
                plan.phase = Phase::In;
                plan.start_hole();
            }
        }
        Ok(plan)
    }
    /// Events due after the motion just handed out.
    pub fn take_events(&mut self) -> Vec<Event> {
        std::mem::take(&mut self.events)
    }
    fn cycle(&self) -> DrillCycle {
        match (self.op.spindle_stop, self.op.dwell_s) {
            _ if self.op.fine_bore.is_some() => DrillCycle::FineBore,
            (true, _) => DrillCycle::BoreStop,
            (false, Some(_)) => DrillCycle::BoreDwell,
            (false, None) => DrillCycle::Bore,
        }
    }
    /// Events after the move to depth, and after the move out.
    fn bottom_events(&self) -> (Vec<Event>, Vec<Event>) {
        let mut bottom = vec![];
        if let Some(seconds) = self.op.dwell_s {
            bottom.push(Event::Dwell { seconds });
        }
        let mut out = vec![];
        if let Some(f) = self.op.fine_bore {
            bottom.push(Event::SpindleOrient {
                edge_angle_deg: f.edge_angle_deg,
            });
            out.push(Event::Spindle {
                rpm: self.core.spindle,
                direction: SpindleDirection::Cw,
            });
        }
        if self.op.spindle_stop {
            bottom.push(Event::Spindle {
                rpm: 0.,
                direction: SpindleDirection::Cw,
            });
            out.push(Event::Spindle {
                rpm: self.core.spindle,
                direction: SpindleDirection::Cw,
            });
        }
        (bottom, out)
    }
    /// Fine boring: where the bar axis shifts to, away from the edge.
    fn shifted(&self, c: crate::Point) -> Option<crate::Point> {
        self.op.fine_bore.map(|f| {
            let a = f.edge_angle_deg.to_radians();
            [c[0] - f.shift_mm * a.cos(), c[1] - f.shift_mm * a.sin()]
        })
    }
    /// Queue the current hole's cycle from the R plane and announce it.
    fn start_hole(&mut self) {
        let c = self.core.centers[self.hole];
        let r = self.heights.feed;
        let (bottom, out) = self.bottom_events();
        let p = self.core.position;
        let mut steps: Vec<Step> = vec![];
        if [p[0], p[1]] != c {
            steps.push((
                [c[0], c[1], r],
                MoveKind::Rapid,
                self.core.feed,
                vec![],
                false,
            ));
        }
        steps.push((
            [c[0], c[1], self.op.depth_z_mm],
            MoveKind::Plunge,
            self.core.feed,
            bottom,
            true,
        ));
        if let Some(shifted) = self.shifted(c) {
            let feed = self.core.feed;
            steps.push((
                [shifted[0], shifted[1], self.op.depth_z_mm],
                MoveKind::Shift,
                feed,
                vec![],
                false,
            ));
            steps.push((
                [shifted[0], shifted[1], r],
                MoveKind::Shift,
                feed,
                vec![],
                false,
            ));
            steps.push(([c[0], c[1], r], MoveKind::Shift, feed, out, false));
        } else if self.op.spindle_stop {
            steps.push(([c[0], c[1], r], MoveKind::Rapid, self.core.feed, out, false));
        } else {
            steps.push((
                [c[0], c[1], r],
                MoveKind::Retract,
                self.core.feed,
                out,
                false,
            ));
        }
        let covers = steps.len() + steps.iter().map(|s| s.3.len()).sum::<usize>();
        self.events.push(Event::Cycle {
            cycle: self.cycle(),
            center_mm: c,
            z_mm: self.op.depth_z_mm,
            r_mm: r,
            initial_z_mm: r,
            return_to_r: false,
            feed_mm_min: self.core.feed,
            peck_mm: None,
            dwell_s: self.op.dwell_s.filter(|_| !self.op.spindle_stop),
            reentry_mm: None,
            lift_mm: None,
            shift_mm: self.op.fine_bore.map(|f| f.shift_mm),
            covers: covers as u32,
        });
        self.queue.extend(steps);
    }
    fn next_canned(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        loop {
            self.core.check(stock, cancel)?;
            let Some((end, kind, feed, after, check)) = self.queue.pop_front() else {
                self.core.drained = true;
                return Ok(None);
            };
            if check {
                self.check_open(stock)?;
            }
            let m = self.core.emit(self.core.line(end), kind, feed, stock)?;
            self.events.extend(after);
            self.advance();
            if m.is_some() {
                return Ok(m);
            }
        }
    }
    fn advance(&mut self) {
        if self.queue.is_empty() && self.phase != Phase::Done {
            if self.phase == Phase::Traverse {
                // Lead-in done: the first hole starts.
                self.phase = Phase::In;
                self.start_hole();
            } else {
                self.hole += 1;
                self.report.completed_holes += 1;
                if self.hole < self.core.centers.len() {
                    self.start_hole();
                } else {
                    self.phase = Phase::Done;
                    let p = self.core.position;
                    self.queue.push_back((
                        [p[0], p[1], self.heights.clearance],
                        MoveKind::Rapid,
                        self.core.feed,
                        vec![],
                        false,
                    ));
                }
            }
        }
    }
    fn check_open(&self, stock: &Stock) -> Result<(), String> {
        let c = self.core.centers[self.hole];
        let radius = self.report.proven_open_radius_mm;
        if !column_open(stock, c, radius, self.op.depth_z_mm) {
            return Err(format!(
                "reaming hole {}: the stock is not proven open at ({:.4}, {:.4}) within radius {radius:.4} down to Z {:.4}; drill the pilot hole first (or use a finer stock cell size)",
                self.hole, c[0], c[1], self.op.depth_z_mm
            ));
        }
        Ok(())
    }
    pub fn report(&self) -> &ReamingReport {
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
            return Err("reaming report cancelled".into());
        }
        if !self.core.drained {
            return Err("drain committed reaming motions".into());
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
            if self.phase == Phase::Done {
                self.core.drained = true;
                return Ok(None);
            }
            let p = self.core.position;
            let c = self.core.centers[self.hole];
            let r_plane = self.heights.feed;
            let (curve, kind, feed, next) = match self.phase {
                Phase::Lift => (
                    self.core.line([p[0], p[1], self.heights.clearance]),
                    MoveKind::Rapid,
                    self.core.feed,
                    Phase::Traverse,
                ),
                Phase::Traverse => (
                    self.core.line([c[0], c[1], p[2]]),
                    MoveKind::Rapid,
                    self.core.feed,
                    Phase::Descend,
                ),
                Phase::Descend => {
                    self.check_open(stock)?;
                    (
                        self.core.line([c[0], c[1], r_plane]),
                        MoveKind::Rapid,
                        self.core.feed,
                        Phase::In,
                    )
                }
                Phase::In => (
                    self.core.line([c[0], c[1], self.op.depth_z_mm]),
                    MoveKind::Plunge,
                    self.core.feed,
                    Phase::Out,
                ),
                Phase::Out if self.op.fine_bore.is_some() => {
                    let s = self.shifted(c).expect("fine bore");
                    (
                        self.core.line([s[0], s[1], self.op.depth_z_mm]),
                        MoveKind::Shift,
                        self.core.feed,
                        Phase::ShiftUp,
                    )
                }
                Phase::ShiftUp => (
                    self.core.line([p[0], p[1], r_plane]),
                    MoveKind::Shift,
                    self.core.feed,
                    Phase::ShiftBack,
                ),
                Phase::ShiftBack => (
                    self.core.line([c[0], c[1], r_plane]),
                    MoveKind::Shift,
                    self.core.feed,
                    Phase::Rise,
                ),
                Phase::Out if self.op.spindle_stop => (
                    self.core.line([c[0], c[1], r_plane]),
                    MoveKind::Rapid,
                    self.core.feed,
                    Phase::Rise,
                ),
                Phase::Out => (
                    self.core.line([c[0], c[1], r_plane]),
                    MoveKind::Retract,
                    self.core.retract_feed,
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
                        self.core.feed,
                        if last { Phase::Done } else { Phase::Traverse },
                    )
                }
                Phase::Done => unreachable!(),
            };
            let phase = self.phase;
            self.phase = next;
            if let Some(motion) = self.core.emit(curve, kind, feed, stock)? {
                let (bottom, out) = self.bottom_events();
                match phase {
                    Phase::In => self.events.extend(bottom),
                    Phase::Out if self.op.fine_bore.is_none() => self.events.extend(out),
                    Phase::ShiftBack => self.events.extend(out),
                    _ => {}
                }
                return Ok(Some(motion));
            }
        }
    }
}
