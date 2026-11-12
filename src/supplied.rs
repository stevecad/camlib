//! A toolpath supplied by the caller (#157): its moves are run exactly as
//! given, through the same stream as a planned operation's.
//!
//! Exchanging a toolpath as cutter-location data, apart from the system
//! that planned it, is standard practice: CLDATA (ISO 4343) and APT CL
//! files carry a toolpath from the system that made it to the one that
//! checks, simulates and posts it. This operation is that hand-over with
//! camlib's own typed motion (`motion::Curve`, `motion::MoveKind`) in the
//! setup's work frame instead of a text format.
//!
//! Nothing in the supplied moves is changed: no re-linking, no smoothing, no
//! feed substitution. A move that cannot be run exactly is refused with its
//! index (0-based, into `motions`). The rules, in order:
//!
//! * a move is a rapid, cut, plunge, lead, link or retract. The other kinds
//!   belong to cycles (tapping, oriented shifts, threading, probing) whose
//!   events only their own operations write;
//! * a rapid is a line (`Motion::validate_feed`); every other kind states a
//!   positive finite feed. A rapid may state one (a fed traverse) or none;
//! * its geometry is a valid curve within `tolerance_mm`
//!   (`Curve::prepare`: finite coordinates, a line of nonzero length, an arc
//!   or helix whose end lies on its circle within the tolerance);
//! * each move starts exactly where the one before it ends (the same
//!   coordinates, bit for bit). That is the stream's own continuity rule;
//!   camlib does not bridge or snap a gap, however small;
//! * the first move starts, and the last move ends, at or above
//!   `clearance_z_mm`, which is above the stock top: the convention of every
//!   planned operation (`heights`: an operation starts and ends at
//!   clearance). The toolpath brings its own approach and retract;
//! * with machining limits, every move stays out of the avoid areas below
//!   their tops and every cutting move inside the containment.
//!
//! camlib adds only the travel to the first supplied point, as planned
//! operations travel to theirs: from where the operation starts (at or
//! above clearance), a vertical rapid to the first point's height, then a
//! horizontal rapid to it. Both are ordinary moves of this operation,
//! checked like the rest; neither is written when it has no length.
//!
//! What a planner would have guaranteed is not assumed here:
//! * the run must check stock contact (`StreamManifest::validate`): nothing
//!   else stands behind a supplied rapid, link, shank or holder;
//! * a cutter that is not center cutting descends only through air: a fed
//!   descending move is refused unless the stock proves nothing above its
//!   lowest point under the whole move (camlib has no ramp-angle data);
//! * spindle speed, coolant, work offset, tool number and length register
//!   are declared like any operation's, and written the same way.
use crate::{
    limits::PreparedLimits,
    motion::{Coolant, Curve, Event, Motion, MoveKind, SpindleDirection, Vec3},
    pocket::ResidualStatus,
    stock::{CancelSignal, Stock, StockSpec},
    tool::{CutterShape, ToolAssembly, valid_id},
};
use serde::{Deserialize, Serialize};

/// One supplied move: its role, its path and its feed.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuppliedMove {
    pub kind: MoveKind,
    pub curve: Curve,
    /// mm/min. None only on a rapid (unprogrammed rapid speed).
    pub feed_mm_min: Option<f64>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuppliedToolpathOperation {
    pub id: String,
    pub tool: String,
    /// The moves, in order, in the setup's work frame.
    pub motions: Vec<SuppliedMove>,
    /// The toolpath starts and ends at or above this height (absolute Z,
    /// above the stock top); the operation starts at or above it too.
    pub clearance_z_mm: f64,
    pub tolerance_mm: f64,
    pub spindle_rpm: f64,
    pub work_offset: u8,
    pub tool_number: u32,
    pub length_register: u32,
    pub coolant: Coolant,
    /// Containment and avoid areas (clamps, fixtures, features to leave alone).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub limits: Option<crate::limits::Limits>,
}

#[derive(Debug, Serialize)]
pub struct SuppliedToolpathReport {
    pub strategy: &'static str,
    pub geometric_residual_status: ResidualStatus,
    pub supplied_moves: usize,
    /// Travel moves camlib added before the first supplied move (0 to 2).
    pub approach_moves: usize,
    /// The motion id of supplied move 0; move `i` has this id plus `i`.
    pub first_supplied_motion_id: u64,
    pub moves_emitted: usize,
    pub machining_scope: &'static str,
}

pub struct SuppliedToolpathPlan {
    op: SuppliedToolpathOperation,
    approach: Vec<Curve>,
    first_id: u64,
    next: usize,
    initial: Vec3,
    /// Not center cutting: the cutter radius, to prove descents in air.
    air_descent_radius: Option<f64>,
    drained: bool,
    report: SuppliedToolpathReport,
}

/// Shortest line a motion can be (`Curve::prepare`).
const MIN_LINE_MM: f64 = 1e-9;

fn kind_name(kind: MoveKind) -> String {
    serde_json::to_value(kind)
        .ok()
        .and_then(|v| v.as_str().map(str::to_string))
        .unwrap_or_else(|| format!("{kind:?}"))
}

impl SuppliedToolpathPlan {
    pub fn prepare(
        op: SuppliedToolpathOperation,
        assembly: &ToolAssembly,
        spec: &StockSpec,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        if cancel.is_cancelled() {
            return Err("supplied toolpath preparation cancelled".into());
        }
        spec.grid()?;
        let tool = assembly.prepare()?;
        let cutter = tool.cutter().definition();
        let guard = 16. * op.tolerance_mm;
        let top = spec.max_mm[2];
        crate::heights::require(
            "supplied toolpath",
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
                    !matches!(cutter.shape, CutterShape::Beam),
                    "a supplied toolpath is three-axis milling: a plasma or laser beam needs its pierce, lead and beam events, which only ThermalCut plans".into(),
                ),
                (
                    [op.clearance_z_mm, op.tolerance_mm, op.spindle_rpm]
                        .iter()
                        .all(|v| v.is_finite()),
                    "all numeric parameters must be finite".into(),
                ),
                (
                    (1e-7..=0.01).contains(&op.tolerance_mm),
                    format!("tolerance_mm {} must be within 1e-7..0.01", op.tolerance_mm),
                ),
                (
                    !op.motions.is_empty(),
                    "at least one move is needed".into(),
                ),
                (
                    op.clearance_z_mm > top + guard && op.clearance_z_mm.abs() <= 1e6,
                    format!(
                        "clearance_z_mm {} must be above the stock top {top}",
                        op.clearance_z_mm
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
                    op.spindle_rpm > 0.,
                    format!("spindle_rpm {} must be positive", op.spindle_rpm),
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
        let refuse = |i: usize, message: String| -> String {
            format!(
                "supplied toolpath {}: move {i} of {}: {message}",
                op.id,
                op.motions.len()
            )
        };
        let limits = op
            .limits
            .as_ref()
            .filter(|l| !l.is_empty())
            .map(|l| PreparedLimits::new(l, cutter.diameter_mm / 2., op.tolerance_mm))
            .transpose()?;
        let mut previous: Option<Vec3> = None;
        for (i, m) in op.motions.iter().enumerate() {
            if i % 4096 == 0 && cancel.is_cancelled() {
                return Err("supplied toolpath preparation cancelled".into());
            }
            if !matches!(
                m.kind,
                MoveKind::Rapid
                    | MoveKind::Cut
                    | MoveKind::Plunge
                    | MoveKind::Lead
                    | MoveKind::Link
                    | MoveKind::Retract
            ) {
                return Err(refuse(
                    i,
                    format!(
                        "kind {:?} belongs to a cycle (tapping, oriented shift, threading or probing) that only its own operation writes; a supplied move is a rapid, cut, plunge, lead, link or retract",
                        kind_name(m.kind)
                    ),
                ));
            }
            if m.kind == MoveKind::Rapid && !matches!(m.curve, Curve::Line { .. }) {
                return Err(refuse(
                    i,
                    "a rapid must be a line; give an arc or helix a cutting kind and a feed".into(),
                ));
            }
            match m.feed_mm_min {
                None if m.kind == MoveKind::Rapid => {}
                None => {
                    return Err(refuse(
                        i,
                        format!(
                            "a {} move needs a feed (feed_mm_min); camlib supplies none",
                            kind_name(m.kind)
                        ),
                    ));
                }
                Some(feed) if feed.is_finite() && feed > 0. => {}
                Some(feed) => {
                    return Err(refuse(
                        i,
                        format!("feed_mm_min {feed} must be positive and finite"),
                    ));
                }
            }
            m.curve.prepare(op.tolerance_mm).map_err(|e| {
                refuse(
                    i,
                    format!("{e} (checked within tolerance_mm {})", op.tolerance_mm),
                )
            })?;
            let start = m.curve.start();
            if let Some(end) = previous
                && end != start
            {
                return Err(refuse(
                    i,
                    format!(
                        "starts at {start:?}, but move {} ends at {end:?}: consecutive moves must share that point exactly (camlib does not bridge a gap)",
                        i - 1
                    ),
                ));
            }
            previous = Some(m.curve.end());
            if let Some(limits) = &limits
                && !limits.allows_curve(&m.curve, m.kind.cuts())?
            {
                return Err(refuse(
                    i,
                    if m.kind.cuts() {
                        "reaches into an avoid area below its top or out of the containment; change the move or the limits".into()
                    } else {
                        "passes through an avoid area below its top; change the move or the limits"
                            .into()
                    },
                ));
            }
        }
        let first = op.motions[0].curve.start();
        let last = op.motions[op.motions.len() - 1].curve.end();
        if first[2] < op.clearance_z_mm {
            return Err(refuse(
                0,
                format!(
                    "starts at Z {}, below clearance_z_mm {}: a supplied toolpath brings its own approach from at or above the clearance height",
                    first[2], op.clearance_z_mm
                ),
            ));
        }
        if last[2] < op.clearance_z_mm {
            return Err(refuse(
                op.motions.len() - 1,
                format!(
                    "ends at Z {}, below clearance_z_mm {}: a supplied toolpath ends with its own retract to at or above the clearance height",
                    last[2], op.clearance_z_mm
                ),
            ));
        }
        // The standard travel to the first point: down (or up) to its
        // height where the operation starts, then across at that height.
        // Both heights are at or above clearance.
        let corner = [initial[0], initial[1], first[2]];
        let mut approach = vec![];
        for (start, end) in [(initial, corner), (corner, first)] {
            if start == end {
                continue;
            }
            let length = (end[0] - start[0])
                .hypot(end[1] - start[1])
                .hypot(end[2] - start[2]);
            if length < MIN_LINE_MM {
                return Err(refuse(
                    0,
                    format!(
                        "starts {length:e} mm from where the operation starts ({initial:?}): too short a travel move to write; start exactly there, or further away"
                    ),
                ));
            }
            approach.push(Curve::Line { start, end });
        }
        let total = (approach.len() as u64).checked_add(op.motions.len() as u64);
        if first_id == 0 || total.and_then(|n| first_id.checked_add(n)).is_none() {
            return Err("invalid supplied toolpath parameters: motion ID budget exhausted".into());
        }
        let report = SuppliedToolpathReport {
            strategy: "supplied_toolpath",
            geometric_residual_status: ResidualStatus::Pending,
            supplied_moves: op.motions.len(),
            approach_moves: approach.len(),
            first_supplied_motion_id: first_id + approach.len() as u64,
            moves_emitted: 0,
            machining_scope: "the caller's moves, unchanged, after camlib's travel to the first; no planner guarantee stands behind them: the part, stock, fixtures and machine are protected only through the configured checks",
        };
        Ok(Self {
            air_descent_radius: (!cutter.center_cutting).then_some(cutter.diameter_mm / 2.),
            op,
            approach,
            first_id,
            next: 0,
            initial,
            drained: false,
            report,
        })
    }
    pub fn report(&self) -> &SuppliedToolpathReport {
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
            return Err("supplied toolpath report cancelled".into());
        }
        if !self.drained {
            return Err("drain committed supplied toolpath motions first".into());
        }
        self.report.geometric_residual_status = ResidualStatus::Unknown;
        Ok(())
    }
    pub fn next_motion_with_stock(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        if cancel.is_cancelled() {
            return Err("supplied toolpath cancelled".into());
        }
        let id = self.first_id + self.next as u64;
        let (kind, curve, feed_mm_min) = if let Some(curve) = self.approach.get(self.next) {
            (MoveKind::Rapid, curve.clone(), None)
        } else if let Some(m) = self.op.motions.get(self.next - self.approach.len()) {
            if let Some(radius) = self.air_descent_radius
                && m.kind.cuts()
                && m.curve.end()[2] < m.curve.start()[2]
            {
                let i = self.next - self.approach.len();
                let bounds = m.curve.prepare(self.op.tolerance_mm)?.bounds();
                let pad = radius + 16. * self.op.tolerance_mm;
                let mut work = u64::MAX;
                if !stock.bounds_clear(
                    [
                        [bounds.min[0] - pad, bounds.min[1] - pad],
                        [bounds.max[0] + pad, bounds.max[1] + pad],
                    ],
                    bounds.min[2],
                    &mut work,
                    cancel,
                )? {
                    return Err(format!(
                        "supplied toolpath {}: move {i} of {}: descends to Z {} where stock may remain above that height, with tool {} whose cutter is not center cutting; camlib has no ramp-angle data for it: descend through air (outside the stock or where it is already cut), or use a center-cutting cutter",
                        self.op.id,
                        self.op.motions.len(),
                        bounds.min[2],
                        self.op.tool
                    ));
                }
            }
            (m.kind, m.curve.clone(), m.feed_mm_min)
        } else {
            self.drained = true;
            return Ok(None);
        };
        self.next += 1;
        self.report.moves_emitted = self.next;
        Ok(Some(Motion {
            id,
            operation: self.op.id.clone(),
            tool: self.op.tool.clone(),
            kind,
            curve,
            feed_mm_min,
        }))
    }
}
