//! Shared dispatch for native operations; motion/stock streaming stays common.
use crate::{
    adaptive::{AdaptivePlan, AdaptiveReport},
    chamfer::{ChamferPlan, ChamferReport},
    contour::{ContourPlan, ContourReport},
    deburr::{DeburrPlan, DeburrReport},
    facing::{FacingPlan, FacingReport},
    finishing::{ParallelPlan, ParallelReport},
    flat::{FlatPlan, FlatReport},
    holes::{
        BorePlan, BoreReport, DrillingPlan, DrillingReport, ReamingPlan, ReamingReport,
        TappingPlan, TappingReport, ThreadMillPlan, ThreadMillReport,
    },
    motion::{Event, Motion, Vec3},
    pocket::{Operation, PocketPlan, PocketReport},
    roughing::{RoughingPlan, RoughingReport},
    slot::{SlotPlan, SlotReport},
    stock::{CancelSignal, Stock, StockSpec},
    tool::ToolAssembly,
    trace::{TracePlan, TraceReport},
    undercut::{UndercutPlan, UndercutReport},
    waterline::{WaterlinePlan, WaterlineReport},
};
use serde::Serialize;

#[derive(Serialize)]
#[serde(untagged)]
pub enum OperationReport<'a> {
    Roughing(&'a RoughingReport),
    Adaptive(&'a AdaptiveReport),
    AdaptiveFront(&'a crate::pocket_front::PocketFrontReport),
    Pocket(&'a PocketReport),
    Slot(&'a SlotReport),
    Facing(&'a FacingReport),
    Contour(&'a ContourReport),
    Drilling(&'a DrillingReport),
    Bore(&'a BoreReport),
    Chamfer(&'a ChamferReport),
    Deburr(&'a DeburrReport),
    Parallel(&'a ParallelReport),
    Waterline(&'a WaterlineReport),
    Trace(&'a TraceReport),
    Undercut(&'a UndercutReport),
    ThreadMill(&'a ThreadMillReport),
    Tapping(&'a TappingReport),
    Reaming(&'a ReamingReport),
    Flat(&'a FlatReport),
    VCarve(&'a crate::vcarve::VCarveReport),
    ThermalCut(&'a crate::thermal::ThermalCutReport),
    Probe(&'a crate::probe::ProbeReport),
}
pub enum OperationPlan {
    Roughing(Box<RoughingPlan>),
    Adaptive(Box<AdaptivePlan>),
    /// An adaptive pocket cleared by the engagement front (#190).
    AdaptiveFront(Box<crate::pocket_front::PocketFrontPlan>),
    Pocket(Box<PocketPlan>),
    Slot(Box<SlotPlan>),
    Facing(Box<FacingPlan>),
    Contour(Box<ContourPlan>),
    Drilling(Box<DrillingPlan>),
    Bore(Box<BorePlan>),
    Chamfer(Box<ChamferPlan>),
    Deburr(Box<DeburrPlan>),
    Parallel(Box<ParallelPlan>),
    Waterline(Box<WaterlinePlan>),
    Trace(Box<TracePlan>),
    Undercut(Box<UndercutPlan>),
    ThreadMill(Box<ThreadMillPlan>),
    Tapping(Box<TappingPlan>),
    Reaming(Box<ReamingPlan>),
    Flat(Box<FlatPlan>),
    VCarve(Box<crate::vcarve::VCarvePlan>),
    ThermalCut(Box<crate::thermal::ThermalCutPlan>),
    Probe(Box<crate::probe::ProbePlan>),
}
/// Facing with limits clears the stock rectangle (grown so the tool still runs
/// past the edges) as a raster pocket restricted by the limits, entering from
/// the stock edge where it can.
fn facing_limits(
    op: Operation,
    tool: &ToolAssembly,
    spec: &StockSpec,
) -> Result<Operation, String> {
    use crate::planar::{Boundary, Component, PlanarRegion};
    let Operation::Facing(f) = op else {
        return Ok(op);
    };
    let Some(limits) = f.limits.clone().filter(|l| !l.is_empty()) else {
        return Ok(Operation::Facing(f));
    };
    let r = tool.cutter.diameter_mm / 2.;
    if f.stepover_mm > r {
        return Err(format!(
            "facing {}: with limits, facing is planned as a pocket around them, whose stepover_mm {} must be at most the tool radius {r}",
            f.id, f.stepover_mm
        ));
    }
    let grow = r + f.edge_overlap_mm.unwrap_or(tool.cutter.diameter_mm * 0.1);
    let (lo, hi) = (spec.min_mm, spec.max_mm);
    let corners = [
        [lo[0] - grow, lo[1] - grow, 0.],
        [hi[0] + grow, lo[1] - grow, 0.],
        [hi[0] + grow, hi[1] + grow, 0.],
        [lo[0] - grow, hi[1] + grow, 0.],
    ];
    let region = PlanarRegion {
        version: 1,
        units: "mm".into(),
        name: format!("{} stock face", f.id),
        boundary_error_mm: 0.,
        source: serde_json::Value::Null,
        components: vec![Component {
            outer: Boundary {
                vertices: corners.to_vec(),
            },
            holes: vec![],
        }],
    };
    let entry = serde_json::from_value(serde_json::json!({"kind": "stock_edge_or_helix"}))
        .map_err(|e| format!("facing entry: {e}"))?;
    Ok(Operation::Pocket(crate::pocket::PocketOperation {
        max_stay_down_mm: None,
        id: f.id,
        tool: f.tool,
        region,
        top_z_mm: spec.max_mm[2],
        floor_z_mm: f.floor_z_mm,
        stepdown_mm: f.stepdown_mm,
        stepover_mm: f.stepover_mm,
        feed_mm_min: f.feed_mm_min,
        plunge_mm_min: f.plunge_mm_min,
        clearance_z_mm: f.clearance_z_mm,
        radial_allowance_mm: 0.,
        bottom_allowance_mm: 0.,
        tolerance_mm: f.tolerance_mm,
        spindle_rpm: f.spindle_rpm,
        work_offset: f.work_offset,
        tool_number: f.tool_number,
        length_register: f.length_register,
        coolant: f.coolant,
        rest_tolerance_mm: None,
        boundary_clockwise: None,
        entry: Some(entry),
        retract_z_mm: f.retract_z_mm,
        feed_z_mm: f.feed_z_mm,
        limits: Some(limits),
    }))
}
/// Holes take no part of their footprint into an avoid area or out of the
/// containment: such a hole is refused, never skipped.
fn hole_limits(op: Operation, tool: &ToolAssembly) -> Result<Operation, String> {
    use crate::limits::check_holes;
    let r = tool.cutter.diameter_mm / 2.;
    Ok(match op {
        Operation::Drilling(mut p) => {
            check_holes(
                p.limits.take(),
                &p.centers_mm,
                r,
                p.depth_z_mm,
                p.tolerance_mm,
                "drilling",
            )?;
            Operation::Drilling(p)
        }
        Operation::Bore(mut p) => {
            let radius = p.diameter_mm / 2.;
            check_holes(
                p.limits.take(),
                &p.centers_mm,
                radius,
                p.floor_z_mm,
                p.tolerance_mm,
                "bore",
            )?;
            Operation::Bore(p)
        }
        Operation::Reaming(mut p) => {
            check_holes(
                p.limits.take(),
                &p.centers_mm,
                r,
                p.depth_z_mm,
                p.tolerance_mm,
                "reaming",
            )?;
            Operation::Reaming(p)
        }
        Operation::Tapping(mut p) => {
            check_holes(
                p.limits.take(),
                &p.centers_mm,
                r,
                p.depth_z_mm,
                p.tolerance_mm,
                "tapping",
            )?;
            Operation::Tapping(p)
        }
        Operation::ThreadMill(mut p) => {
            let radius = p.diameter_mm / 2. + if p.internal { 0. } else { 2. * r };
            check_holes(
                p.limits.take(),
                &p.centers_mm,
                radius,
                p.bottom_z_mm,
                p.tolerance_mm,
                "thread milling",
            )?;
            Operation::ThreadMill(p)
        }
        Operation::TrochoidalSlot(mut p) => {
            // The whole slot (its width around the centerline) at full depth.
            if let Some(limits) = p.limits.take().filter(|l| !l.is_empty()) {
                let prepared =
                    crate::limits::PreparedLimits::new(&limits, p.width_mm / 2., p.tolerance_mm)?;
                for curve in crate::slot::centerline_curves(&p, p.floor_z_mm)? {
                    if !prepared.allows_curve(&curve, true)? {
                        return Err(format!(
                            "slot {}: the slot reaches into an avoid area or out of the containment",
                            p.id
                        ));
                    }
                }
            }
            Operation::TrochoidalSlot(p)
        }
        other => other,
    })
}
/// A probe never cuts: only probing operations use it.
fn probe_only_probes(op: &Operation, tool: &ToolAssembly) -> Result<(), String> {
    if matches!(tool.cutter.shape, crate::tool::CutterShape::Probe)
        && !matches!(op, Operation::Probe(_))
    {
        return Err(format!(
            "operation {}: tool {} is a probe, which never cuts; only probing operations use it",
            op.id(),
            tool.id
        ));
    }
    Ok(())
}
impl OperationPlan {
    pub fn prepare_with_stock(
        op: Operation,
        tool: &ToolAssembly,
        stock: &Stock,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        let op = facing_limits(op, tool, stock.spec())?;
        probe_only_probes(&op, tool)?;
        match op {
            Operation::Parallel(p) => Ok(Self::Parallel(Box::new(
                ParallelPlan::prepare_with_stock(p, tool, stock, initial, first_id, cancel)?,
            ))),
            Operation::Waterline(p) => Ok(Self::Waterline(Box::new(
                WaterlinePlan::prepare_with_stock(p, tool, stock, initial, first_id, cancel)?,
            ))),
            Operation::Pocket(p) => Ok(Self::Pocket(Box::new(PocketPlan::prepare_with_stock(
                p, tool, stock, initial, first_id, cancel,
            )?))),
            Operation::AdaptivePocket(p)
                if p.path_strategy == crate::adaptive::AdaptivePathStrategy::EngagementFront =>
            {
                Ok(Self::AdaptiveFront(Box::new(
                    crate::pocket_front::PocketFrontPlan::prepare_with_stock(
                        p, tool, stock, initial, first_id, cancel,
                    )?,
                )))
            }
            Operation::AdaptivePocket(p) => Ok(Self::Adaptive(Box::new(
                AdaptivePlan::prepare_with_stock(p, tool, stock, initial, first_id, cancel)?,
            ))),
            Operation::Roughing(p) => Ok(Self::Roughing(Box::new(
                RoughingPlan::prepare_with_stock(p, tool, stock, initial, first_id, cancel)?,
            ))),
            other => Self::prepare(other, tool, stock.spec(), initial, first_id, cancel),
        }
    }
    pub fn prepare(
        op: Operation,
        tool: &ToolAssembly,
        stock: &StockSpec,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        let op = facing_limits(hole_limits(op, tool)?, tool, stock)?;
        probe_only_probes(&op, tool)?;
        match op {
            Operation::Probe(p) => Ok(Self::Probe(Box::new(crate::probe::ProbePlan::prepare(
                p, tool, stock, initial, first_id, cancel,
            )?))),
            Operation::Roughing(p) => Ok(Self::Roughing(Box::new(RoughingPlan::prepare(
                p, tool, stock, initial, first_id, cancel,
            )?))),
            Operation::AdaptivePocket(p)
                if p.path_strategy == crate::adaptive::AdaptivePathStrategy::EngagementFront =>
            {
                Ok(Self::AdaptiveFront(Box::new(
                    crate::pocket_front::PocketFrontPlan::prepare(
                        p, tool, stock, initial, first_id, cancel,
                    )?,
                )))
            }
            Operation::AdaptivePocket(p) => Ok(Self::Adaptive(Box::new(AdaptivePlan::prepare(
                p, tool, stock, initial, first_id, cancel,
            )?))),
            Operation::Waterline(p) => Ok(Self::Waterline(Box::new(WaterlinePlan::prepare(
                p, tool, stock, initial, first_id, cancel,
            )?))),
            Operation::Parallel(p) => Ok(Self::Parallel(Box::new(ParallelPlan::prepare(
                p, tool, stock, initial, first_id, cancel,
            )?))),
            Operation::Chamfer(p) => Ok(Self::Chamfer(Box::new(ChamferPlan::prepare(
                p, tool, stock, initial, first_id, cancel,
            )?))),
            Operation::Drilling(p) => Ok(Self::Drilling(Box::new(DrillingPlan::prepare(
                p, tool, stock, initial, first_id, cancel,
            )?))),
            Operation::Bore(p) => Ok(Self::Bore(Box::new(BorePlan::prepare(
                p, tool, stock, initial, first_id, cancel,
            )?))),
            Operation::Contour(p) => Ok(Self::Contour(Box::new(ContourPlan::prepare(
                p, tool, stock, initial, first_id, cancel,
            )?))),
            Operation::Undercut(p) => Ok(Self::Undercut(Box::new(UndercutPlan::prepare(
                p, tool, stock, initial, first_id, cancel,
            )?))),
            Operation::Trace(p) => Ok(Self::Trace(Box::new(TracePlan::prepare(
                p, tool, stock, initial, first_id, cancel,
            )?))),
            Operation::ThreadMill(p) => Ok(Self::ThreadMill(Box::new(ThreadMillPlan::prepare(
                p, tool, stock, initial, first_id, cancel,
            )?))),
            Operation::Reaming(p) => Ok(Self::Reaming(Box::new(ReamingPlan::prepare(
                p, tool, stock, initial, first_id, cancel,
            )?))),
            Operation::Flat(p) => Ok(Self::Flat(Box::new(FlatPlan::prepare(
                p, tool, stock, initial, first_id, cancel,
            )?))),
            Operation::Deburr(p) => Ok(Self::Deburr(Box::new(DeburrPlan::prepare(
                p, tool, stock, initial, first_id, cancel,
            )?))),
            Operation::ThermalCut(p) => Ok(Self::ThermalCut(Box::new(
                crate::thermal::ThermalCutPlan::prepare(p, tool, stock, initial, first_id, cancel)?,
            ))),
            Operation::VCarve(p) => Ok(Self::VCarve(Box::new(crate::vcarve::VCarvePlan::prepare(
                p, tool, stock, initial, first_id, cancel,
            )?))),
            Operation::Tapping(p) => Ok(Self::Tapping(Box::new(TappingPlan::prepare(
                p, tool, stock, initial, first_id, cancel,
            )?))),
            Operation::Facing(p) => Ok(Self::Facing(Box::new(FacingPlan::prepare(
                p, tool, stock, initial, first_id, cancel,
            )?))),
            Operation::Pocket(p) => Ok(Self::Pocket(Box::new(PocketPlan::prepare(
                p, tool, stock, initial, first_id, cancel,
            )?))),
            Operation::TrochoidalSlot(p) => Ok(Self::Slot(Box::new(SlotPlan::prepare(
                p, tool, stock, initial, first_id, cancel,
            )?))),
        }
    }
    pub fn report(&self) -> OperationReport<'_> {
        match self {
            Self::Roughing(p) => OperationReport::Roughing(p.report()),
            Self::Adaptive(p) => OperationReport::Adaptive(p.report()),
            Self::AdaptiveFront(p) => OperationReport::AdaptiveFront(p.report()),
            Self::Pocket(p) => OperationReport::Pocket(p.report()),
            Self::Slot(p) => OperationReport::Slot(p.report()),
            Self::Facing(p) => OperationReport::Facing(p.report()),
            Self::Contour(p) => OperationReport::Contour(p.report()),
            Self::Drilling(p) => OperationReport::Drilling(p.report()),
            Self::Bore(p) => OperationReport::Bore(p.report()),
            Self::Chamfer(p) => OperationReport::Chamfer(p.report()),
            Self::Deburr(p) => OperationReport::Deburr(p.report()),
            Self::Parallel(p) => OperationReport::Parallel(p.report()),
            Self::Waterline(p) => OperationReport::Waterline(p.report()),
            Self::Trace(p) => OperationReport::Trace(p.report()),
            Self::Undercut(p) => OperationReport::Undercut(p.report()),
            Self::ThreadMill(p) => OperationReport::ThreadMill(p.report()),
            Self::Tapping(p) => OperationReport::Tapping(p.report()),
            Self::Reaming(p) => OperationReport::Reaming(p.report()),
            Self::Flat(p) => OperationReport::Flat(p.report()),
            Self::VCarve(p) => OperationReport::VCarve(p.report()),
            Self::ThermalCut(p) => OperationReport::ThermalCut(p.report()),
            Self::Probe(p) => OperationReport::Probe(p.report()),
        }
    }
    pub fn prefix_events(&self) -> Vec<Event> {
        match self {
            Self::Roughing(p) => p.prefix_events(),
            Self::Adaptive(p) => p.prefix_events(),
            Self::AdaptiveFront(p) => p.prefix_events(),
            Self::Pocket(p) => p.prefix_events(),
            Self::Slot(p) => p.prefix_events(),
            Self::Facing(p) => p.prefix_events(),
            Self::Contour(p) => p.prefix_events(),
            Self::Drilling(p) => p.prefix_events(),
            Self::Bore(p) => p.prefix_events(),
            Self::Chamfer(p) => p.prefix_events(),
            Self::Deburr(p) => p.prefix_events(),
            Self::Parallel(p) => p.prefix_events(),
            Self::Waterline(p) => p.prefix_events(),
            Self::Trace(p) => p.prefix_events(),
            Self::Undercut(p) => p.prefix_events(),
            Self::ThreadMill(p) => p.prefix_events(),
            Self::Tapping(p) => p.prefix_events(),
            Self::Reaming(p) => p.prefix_events(),
            Self::Flat(p) => p.prefix_events(),
            Self::VCarve(p) => p.prefix_events(),
            Self::ThermalCut(p) => p.prefix_events(),
            Self::Probe(p) => p.prefix_events(),
        }
    }
    pub fn suffix_events(&self) -> Vec<Event> {
        match self {
            Self::Roughing(p) => p.suffix_events(),
            Self::Adaptive(p) => p.suffix_events(),
            Self::AdaptiveFront(p) => p.suffix_events(),
            Self::Pocket(p) => p.suffix_events(),
            Self::Slot(p) => p.suffix_events(),
            Self::Facing(p) => p.suffix_events(),
            Self::Contour(p) => p.suffix_events(),
            Self::Drilling(p) => p.suffix_events(),
            Self::Bore(p) => p.suffix_events(),
            Self::Chamfer(p) => p.suffix_events(),
            Self::Deburr(p) => p.suffix_events(),
            Self::Parallel(p) => p.suffix_events(),
            Self::Waterline(p) => p.suffix_events(),
            Self::Trace(p) => p.suffix_events(),
            Self::Undercut(p) => p.suffix_events(),
            Self::ThreadMill(p) => p.suffix_events(),
            Self::Tapping(p) => p.suffix_events(),
            Self::Reaming(p) => p.suffix_events(),
            Self::Flat(p) => p.suffix_events(),
            Self::VCarve(p) => p.suffix_events(),
            Self::ThermalCut(p) => p.suffix_events(),
            Self::Probe(p) => p.suffix_events(),
        }
    }
    pub fn next_motion_with_stock(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        match self {
            Self::Roughing(p) => p.next_motion_with_stock(stock, cancel),
            Self::Adaptive(p) => p.next_motion_with_stock(stock, cancel),
            Self::AdaptiveFront(p) => p.next_motion_with_stock(stock, cancel),
            Self::Pocket(p) => p.next_motion_with_stock(stock, cancel),
            Self::Slot(p) => p.next_motion_with_stock(stock, cancel),
            Self::Facing(p) => p.next_motion_with_stock(stock, cancel),
            Self::Contour(p) => p.next_motion_with_stock(stock, cancel),
            Self::Drilling(p) => p.next_motion_with_stock(stock, cancel),
            Self::Bore(p) => p.next_motion_with_stock(stock, cancel),
            Self::Chamfer(p) => p.next_motion_with_stock(stock, cancel),
            Self::Deburr(p) => p.next_motion_with_stock(stock, cancel),
            Self::Parallel(p) => p.next_motion_with_stock(stock, cancel),
            Self::Waterline(p) => p.next_motion_with_stock(stock, cancel),
            Self::Trace(p) => p.next_motion_with_stock(stock, cancel),
            Self::Undercut(p) => p.next_motion_with_stock(stock, cancel),
            Self::ThreadMill(p) => p.next_motion_with_stock(stock, cancel),
            Self::Tapping(p) => p.next_motion_with_stock(stock, cancel),
            Self::Reaming(p) => p.next_motion_with_stock(stock, cancel),
            Self::Flat(p) => p.next_motion_with_stock(stock, cancel),
            Self::VCarve(p) => p.next_motion_with_stock(stock, cancel),
            Self::ThermalCut(p) => p.next_motion_with_stock(stock, cancel),
            Self::Probe(p) => p.next_motion_with_stock(stock, cancel),
        }
    }
    /// Events due after the motion just handed out (e.g. a dwell at the
    /// bottom of a hole). Streams send them before the next motion.
    pub fn take_events(&mut self) -> Vec<Event> {
        match self {
            Self::Drilling(p) => p.take_events(),
            Self::Reaming(p) => p.take_events(),
            Self::Contour(p) => p.take_events(),
            Self::ThermalCut(p) => p.take_events(),
            Self::Probe(p) => p.take_events(),
            _ => vec![],
        }
    }
    pub fn finish_report(&mut self, cancel: &impl CancelSignal) -> Result<(), String> {
        match self {
            Self::Roughing(p) => p.finish_report(cancel),
            Self::Adaptive(p) => p.finish_report(cancel),
            Self::AdaptiveFront(p) => p.finish_report(cancel),
            Self::Pocket(p) => {
                p.finish_report(cancel)?;
                Ok(())
            }
            Self::Slot(p) => p.finish_report(cancel),
            Self::Facing(p) => p.finish_report(cancel),
            Self::Contour(p) => p.finish_report(cancel),
            Self::Drilling(p) => p.finish_report(cancel),
            Self::Bore(p) => p.finish_report(cancel),
            Self::Chamfer(p) => p.finish_report(cancel),
            Self::Deburr(p) => p.finish_report(cancel),
            Self::Parallel(p) => p.finish_report(cancel),
            Self::Waterline(p) => p.finish_report(cancel),
            Self::Trace(p) => p.finish_report(cancel),
            Self::Undercut(p) => p.finish_report(cancel),
            Self::ThreadMill(p) => p.finish_report(cancel),
            Self::Tapping(p) => p.finish_report(cancel),
            Self::Reaming(p) => p.finish_report(cancel),
            Self::Flat(p) => p.finish_report(cancel),
            Self::VCarve(p) => p.finish_report(cancel),
            Self::ThermalCut(p) => p.finish_report(cancel),
            Self::Probe(p) => p.finish_report(cancel),
        }
    }
}
