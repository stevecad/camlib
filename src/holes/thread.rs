//! Helical thread milling with a single-profile thread mill, internal or
//! external, as in commercial CAM: tangent arc lead-in from the hole center
//! (internal) or from outside the boss (external), one helical turn per
//! pitch, tangent arc lead-out, optional equal radial passes and spring
//! passes.
//!
//! Direction follows the usual rules for a clockwise spindle: a right-hand
//! thread advances +Z while moving counter-clockwise. Climb milling moves
//! counter-clockwise inside a hole and clockwise around a boss, so an
//! internal right-hand thread is cut bottom-up and an external one top-down.
//!
//! Before each descent the current stock must prove the descent column open
//! down to the start depth (a drilled hole, or cleared space beside a boss):
//! a thread mill cannot plunge into material.
use super::{Core, ResidualStatus};
use crate::{
    Point,
    motion::{Coolant, Curve, Event, Motion, MoveKind, Plane, Vec3},
    stock::{CancelSignal, Stock, StockSpec},
    tool::{CutterShape, ToolAssembly},
};
use serde::{Deserialize, Serialize};
use std::f64::consts::{PI, TAU};

fn yes() -> bool {
    true
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ThreadMillOperation {
    pub id: String,
    pub tool: String,
    pub centers_mm: Vec<Point>,
    /// Internal (in a hole) or external (around a boss).
    pub internal: bool,
    /// Final diameter reached by the tooth tip: the major diameter of an
    /// internal thread, the minor diameter of an external one.
    pub diameter_mm: f64,
    /// Existing wall: the drilled hole (internal) or the boss (external).
    pub start_diameter_mm: f64,
    pub pitch_mm: f64,
    #[serde(default = "yes")]
    pub right_hand: bool,
    #[serde(default = "yes")]
    pub climb: bool,
    /// Tool tip Z range of the helix.
    pub top_z_mm: f64,
    pub bottom_z_mm: f64,
    /// Equal radial steps from the start to the final diameter.
    #[serde(default = "one")]
    pub radial_passes: u32,
    /// Extra passes at the final diameter.
    #[serde(default)]
    pub spring_passes: u32,
    pub clearance_z_mm: f64,
    /// Programmed feed at the tool center.
    pub feed_mm_min: f64,
    pub plunge_mm_min: f64,
    pub tolerance_mm: f64,
    pub spindle_rpm: f64,
    pub work_offset: u8,
    pub tool_number: u32,
    pub length_register: u32,
    pub coolant: Coolant,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub retract_z_mm: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub feed_z_mm: Option<f64>,
    /// Containment and avoid areas (clamps, fixtures, features to leave alone).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub limits: Option<crate::limits::Limits>,
}
fn one() -> u32 {
    1
}

#[derive(Debug, Serialize)]
pub struct ThreadMillReport {
    pub strategy: &'static str,
    pub geometric_residual_status: ResidualStatus,
    pub holes: usize,
    pub internal: bool,
    pub passes_per_hole: usize,
    pub path_radii_mm: Vec<f64>,
    pub turns_per_pass: f64,
    pub rotation: &'static str,
    pub z_direction: &'static str,
    pub completed_holes: usize,
    pub helical_turns: usize,
    pub open_columns_proven: usize,
    pub stock_model: &'static str,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase {
    Lift,
    Traverse,
    Descend,
    Plunge,
    LeadIn,
    Helix,
    LeadOut,
    Retract,
    Rise,
    Done,
}

pub struct ThreadMillPlan {
    core: Core,
    heights: crate::heights::Heights,
    op: ThreadMillOperation,
    radius: f64,
    /// Lead arc radius for external threads (the radial thread depth).
    lead: f64,
    /// +1 counter-clockwise, -1 clockwise.
    rotation: f64,
    start_z: f64,
    end_z: f64,
    phase: Phase,
    hole: usize,
    pass: usize,
    turn: usize,
    turns: usize,
    report: ThreadMillReport,
    /// How the run splits the straight-up lift out of each pass (#441).
    lift_split: Option<crate::lift::Lifts>,
}

/// Every stock cell touching the disk is at or below `z`.
pub(super) fn column_open(stock: &Stock, center: Point, radius: f64, z: f64) -> bool {
    let spec = stock.spec();
    let [nx, ny] = stock.dimensions();
    let [wx, wy] = stock.cell_widths_mm();
    let range = |k: usize, w: f64, n: usize| {
        let lo = ((center[k] - radius - spec.min_mm[k]) / w).floor().max(0.) as usize;
        let hi = ((center[k] + radius - spec.min_mm[k]) / w).floor().max(0.) as usize;
        lo.min(n)..(hi + 1).min(n)
    };
    let cells = stock.cells();
    for iy in range(1, wy, ny) {
        for ix in range(0, wx, nx) {
            let x0 = spec.min_mm[0] + ix as f64 * wx;
            let y0 = spec.min_mm[1] + iy as f64 * wy;
            let dx = (x0 - center[0]).max(0.).max(center[0] - x0 - wx);
            let dy = (y0 - center[1]).max(0.).max(center[1] - y0 - wy);
            if dx * dx + dy * dy < radius * radius && cells[iy * nx + ix].upper_mm > z {
                return false;
            }
        }
    }
    true
}

impl ThreadMillPlan {
    pub fn prepare(
        mut op: ThreadMillOperation,
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
        let final_radius = if op.internal {
            op.diameter_mm / 2. - r
        } else {
            op.diameter_mm / 2. + r
        };
        let start_radius = if op.internal {
            op.start_diameter_mm / 2. - r
        } else {
            op.start_diameter_mm / 2. + r
        };
        // Neck beside the thread crests: inside a hole the neck (on the final
        // path circle) must stay inside the start hole; around a boss it must
        // stay outside the boss.
        let neck = def
            .neck
            .as_ref()
            .map_or(def.shank_diameter_mm, |n| n.diameter_mm);
        let clear_room = if op.internal {
            op.start_diameter_mm / 2. - final_radius
        } else {
            final_radius - op.start_diameter_mm / 2.
        };
        let neck_fits = neck / 2. < clear_room - guard;
        let values = [
            op.diameter_mm,
            op.start_diameter_mm,
            op.pitch_mm,
            op.top_z_mm,
            op.bottom_z_mm,
            op.plunge_mm_min,
        ];
        crate::heights::require(
            "thread milling",
            [
                (
                    matches!(def.shape, CutterShape::Flat),
                    "thread milling needs a thread mill modeled as a flat cutter (tooth diameter and height, neck)".into(),
                ),
                (
                    values.iter().all(|x| x.is_finite() && x.abs() <= 1e6),
                    "all numeric parameters must be finite".into(),
                ),
                (
                    op.pitch_mm > guard,
                    format!("pitch_mm {} must be positive", op.pitch_mm),
                ),
                (
                    if op.internal {
                        op.start_diameter_mm < op.diameter_mm - guard
                    } else {
                        op.start_diameter_mm > op.diameter_mm + guard
                    },
                    format!(
                        "start_diameter_mm {} must be {} than diameter_mm {} (the existing {} before threading)",
                        op.start_diameter_mm,
                        if op.internal { "smaller" } else { "larger" },
                        op.diameter_mm,
                        if op.internal { "hole" } else { "boss" }
                    ),
                ),
                (
                    !op.internal || start_radius > guard,
                    format!(
                        "the cutter diameter {} does not fit in the start hole diameter {}",
                        def.diameter_mm, op.start_diameter_mm
                    ),
                ),
                (
                    op.internal || op.diameter_mm > guard,
                    format!("diameter_mm {} must be positive", op.diameter_mm),
                ),
                (
                    op.top_z_mm > op.bottom_z_mm + guard,
                    format!(
                        "top_z_mm {} must be above bottom_z_mm {}",
                        op.top_z_mm, op.bottom_z_mm
                    ),
                ),
                (
                    op.bottom_z_mm >= spec.min_mm[2] && op.top_z_mm < op.clearance_z_mm,
                    format!(
                        "bottom_z_mm {} must be within the stock and top_z_mm {} below clearance_z_mm {}",
                        op.bottom_z_mm, op.top_z_mm, op.clearance_z_mm
                    ),
                ),
                (
                    spec.max_mm[2] - op.bottom_z_mm <= reach - guard,
                    format!(
                        "depth {} below the stock top exceeds the tooth and neck reach {reach}",
                        spec.max_mm[2] - op.bottom_z_mm
                    ),
                ),
                (
                    neck_fits,
                    format!(
                        "the neck diameter {neck} must clear the thread crests: at most {} for this thread and tooth",
                        2. * clear_room
                    ),
                ),
                (
                    op.radial_passes >= 1,
                    "radial_passes must be at least 1".into(),
                ),
                (
                    op.plunge_mm_min > 0.,
                    format!("plunge_mm_min {} must be positive", op.plunge_mm_min),
                ),
            ],
        )?;
        // Outside a boss the lead arc starts 2 x lead beyond the thread circle;
        // a lead of the full radial depth keeps the descent clear of the boss.
        let lead = (start_radius - final_radius).abs();
        // The hole or boss must lie in the stock; an external approach
        // outside the stock is in air.
        let extent = if op.internal {
            op.diameter_mm / 2.
        } else {
            op.start_diameter_mm / 2.
        };
        let core = Core {
            id: op.id.clone(),
            tool: op.tool.clone(),
            centers: std::mem::take(&mut op.centers_mm),
            clearance: op.clearance_z_mm,
            feed: op.feed_mm_min,
            retract_feed: op.feed_mm_min,
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
        core.validate(assembly, extent, cancel)?;
        // Climb: counter-clockwise inside, clockwise outside (clockwise spindle).
        let rotation = if op.internal == op.climb { 1. } else { -1. };
        let handed = if op.right_hand { 1. } else { -1. };
        let up = rotation * handed > 0.;
        let (start_z, end_z) = if up {
            (op.bottom_z_mm, op.top_z_mm)
        } else {
            (op.top_z_mm, op.bottom_z_mm)
        };
        let steps = op.radial_passes as usize;
        let mut radii: Vec<f64> = (1..=steps)
            .map(|k| start_radius + (final_radius - start_radius) * k as f64 / steps as f64)
            .collect();
        radii.extend(std::iter::repeat_n(final_radius, op.spring_passes as usize));
        let turns_exact = (op.top_z_mm - op.bottom_z_mm) / op.pitch_mm;
        crate::resources::check_memory(radii.len() as u128 * 8, "thread passes")?;
        let report = ThreadMillReport {
            strategy: "thread_mill",
            geometric_residual_status: ResidualStatus::Pending,
            holes: core.centers.len(),
            internal: op.internal,
            passes_per_hole: radii.len(),
            path_radii_mm: radii,
            turns_per_pass: turns_exact,
            rotation: if rotation > 0. { "ccw" } else { "cw" },
            z_direction: if up { "up" } else { "down" },
            completed_holes: 0,
            helical_turns: 0,
            open_columns_proven: 0,
            stock_model: "tooth as a flat disk (with neck): the stock shows the swept tooth envelope, not the thread form",
        };
        let heights = crate::heights::Heights::resolve(
            op.clearance_z_mm,
            op.retract_z_mm,
            op.feed_z_mm,
            spec.max_mm[2],
            guard,
        )?;
        Ok(Self {
            core,
            heights,
            radius: r,
            lead,
            rotation,
            start_z,
            end_z,
            phase: Phase::Lift,
            hole: 0,
            pass: 0,
            turn: 0,
            turns: (turns_exact - 1e-9).ceil().max(1.) as usize,
            report,
            op,
            lift_split: None,
        })
    }
    pub fn report(&self) -> &ThreadMillReport {
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
            return Err("thread milling report cancelled".into());
        }
        if !self.core.drained {
            return Err("drain committed thread milling motions".into());
        }
        self.report.geometric_residual_status = ResidualStatus::Estimated;
        Ok(())
    }
    fn path_radius(&self) -> f64 {
        self.report.path_radii_mm[self.pass]
    }
    /// Point at `angle` on a circle about the hole center.
    fn at(&self, radius: f64, angle: f64, z: f64) -> Vec3 {
        let c = self.core.centers[self.hole];
        [c[0] + radius * angle.cos(), c[1] + radius * angle.sin(), z]
    }
    fn end_angle(&self) -> f64 {
        self.rotation * TAU * self.report.turns_per_pass
    }
    /// Where the tool goes down (and comes back up): the hole center, or
    /// outside the boss beyond the lead arc.
    fn entry(&self) -> Vec3 {
        if self.op.internal {
            let c = self.core.centers[self.hole];
            [c[0], c[1], self.start_z]
        } else {
            self.at(self.path_radius() + 2. * self.lead, 0., self.start_z)
        }
    }
    fn exit(&self) -> Vec3 {
        if self.op.internal {
            let c = self.core.centers[self.hole];
            [c[0], c[1], self.end_z]
        } else {
            self.at(
                self.path_radius() + 2. * self.lead,
                self.end_angle(),
                self.end_z,
            )
        }
    }
    /// Semicircle tangent to the thread circle at `on`, from or to `off`.
    /// Inside a hole it turns with the helix; outside a boss against it.
    fn lead_arc(&self, from: Vec3, to: Vec3, z: f64) -> Curve {
        let sweep = if self.op.internal {
            self.rotation * PI
        } else {
            -self.rotation * PI
        };
        Curve::Arc {
            start: from,
            end: to,
            center: [(from[0] + to[0]) / 2., (from[1] + to[1]) / 2., z],
            plane: Plane::Xy,
            sweep_rad: sweep,
        }
    }
    fn prove_open(&mut self, stock: &Stock, point: Vec3, low: f64) -> Result<(), String> {
        let disk = self.radius + 2. * self.op.tolerance_mm;
        if !column_open(stock, [point[0], point[1]], disk, low) {
            return Err(format!(
                "thread milling {} {}: the stock is not proven open for the cutter at ({:.4}, {:.4}) down to Z {:.4}; {} first (or use a finer stock cell size)",
                if self.op.internal { "hole" } else { "boss" },
                self.hole,
                point[0],
                point[1],
                low,
                if self.op.internal {
                    "drill the hole"
                } else {
                    "clear around the boss"
                }
            ));
        }
        self.report.open_columns_proven += 1;
        Ok(())
    }
    /// Split the straight-up lift out of each pass where the stock beside
    /// the tool ends (#441, `OperationPlan::split_lifts`).
    pub(crate) fn split_lifts(&mut self, lifts: crate::lift::Lifts) {
        self.lift_split = Some(lifts);
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
            let position = self.core.position;
            let (curve, kind, feed, next) = match self.phase {
                Phase::Lift => (
                    self.core
                        .line([position[0], position[1], self.heights.clearance]),
                    MoveKind::Rapid,
                    self.core.feed,
                    Phase::Traverse,
                ),
                Phase::Traverse => {
                    let e = self.entry();
                    (
                        self.core.line([e[0], e[1], position[2]]),
                        MoveKind::Rapid,
                        self.core.feed,
                        Phase::Descend,
                    )
                }
                Phase::Descend => {
                    let e = self.entry();
                    let low = self.start_z.min(self.end_z);
                    self.prove_open(stock, e, low)?;
                    if !self.op.internal {
                        let x = self.exit();
                        self.prove_open(stock, x, low)?;
                    }
                    (
                        self.core.line([e[0], e[1], self.heights.feed.max(e[2])]),
                        MoveKind::Rapid,
                        self.core.feed,
                        Phase::Plunge,
                    )
                }
                Phase::Plunge => (
                    self.core.line(self.entry()),
                    MoveKind::Plunge,
                    self.op.plunge_mm_min,
                    Phase::LeadIn,
                ),
                Phase::LeadIn => {
                    let on = self.at(self.path_radius(), 0., self.start_z);
                    self.turn = 0;
                    (
                        self.lead_arc(position, on, self.start_z),
                        MoveKind::Lead,
                        self.core.feed,
                        Phase::Helix,
                    )
                }
                Phase::Helix => {
                    let total = self.report.turns_per_pass;
                    let done = self.turn as f64;
                    let part = (total - done).min(1.);
                    let angle = self.rotation * TAU * (done + part);
                    let z = if self.turn + 1 == self.turns {
                        self.end_z
                    } else {
                        let dz = (self.end_z - self.start_z).signum() * self.op.pitch_mm;
                        self.start_z + dz * (done + part)
                    };
                    let end = self.at(self.path_radius(), angle, z);
                    self.turn += 1;
                    self.report.helical_turns += 1;
                    (
                        Curve::Helix {
                            start: position,
                            end,
                            center: [
                                self.core.centers[self.hole][0],
                                self.core.centers[self.hole][1],
                                position[2],
                            ],
                            plane: Plane::Xy,
                            sweep_rad: self.rotation * TAU * part,
                        },
                        MoveKind::Cut,
                        self.core.feed,
                        if self.turn == self.turns {
                            Phase::LeadOut
                        } else {
                            Phase::Helix
                        },
                    )
                }
                Phase::LeadOut => (
                    self.lead_arc(position, self.exit(), self.end_z),
                    MoveKind::Lead,
                    self.core.feed,
                    Phase::Retract,
                ),
                Phase::Retract => {
                    // Fed only through the stock beside the tool, then the
                    // rest of the way as its own retract (#441).
                    let feed = self.heights.feed.max(position[2]);
                    let top = crate::lift::feed_to(self.lift_split.as_ref(), stock, position, feed);
                    (
                        self.core.line([position[0], position[1], top]),
                        MoveKind::Retract,
                        self.core.retract_feed,
                        if top < feed {
                            Phase::Retract
                        } else {
                            Phase::Rise
                        },
                    )
                }
                Phase::Rise => {
                    self.pass += 1;
                    let mut last = false;
                    if self.pass == self.report.path_radii_mm.len() {
                        self.pass = 0;
                        self.hole += 1;
                        self.report.completed_holes += 1;
                        last = self.hole == self.core.centers.len();
                    }
                    let z = if last {
                        self.heights.clearance
                    } else {
                        self.heights.retract
                    };
                    (
                        self.core
                            .line([position[0], position[1], z.max(position[2])]),
                        MoveKind::Rapid,
                        self.core.feed,
                        if last { Phase::Done } else { Phase::Traverse },
                    )
                }
                Phase::Done => unreachable!(),
            };
            self.phase = next;
            if let Some(motion) = self.core.emit(curve, kind, feed, stock)? {
                return Ok(Some(motion));
            }
        }
    }
}
