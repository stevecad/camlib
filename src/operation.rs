//! Shared dispatch for native operations; motion/stock streaming stays common.
use crate::{
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
    ThermalMark(&'a crate::thermal_mark::ThermalMarkReport),
    Probe(&'a crate::probe::ProbeReport),
    SuppliedToolpath(&'a crate::supplied::SuppliedToolpathReport),
}
pub enum OperationPlan {
    Roughing(Box<RoughingPlan>),
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
    ThermalMark(Box<crate::thermal_mark::ThermalMarkPlan>),
    Probe(Box<crate::probe::ProbePlan>),
    /// The caller's own moves, run unchanged (#157).
    SuppliedToolpath(Box<crate::supplied::SuppliedToolpathPlan>),
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
    if tool.cutter.shape.indexable().is_some()
        && !matches!(
            op,
            Operation::Facing(_)
                | Operation::Pocket(_)
                | Operation::Roughing(_)
                | Operation::Contour(_)
                | Operation::SuppliedToolpath(_)
        )
    {
        return Err(format!(
            "operation {}: indexable milling supports Facing, Pocket, raster/offset Roughing, exterior-entry Contour and checked supplied toolpaths; this strategy does not support the insert profile",
            op.id()
        ));
    }
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
        probe_only_probes(&op, tool)?;
        match op {
            Operation::Parallel(p) => Ok(Self::Parallel(Box::new(
                ParallelPlan::prepare_with_stock(p, tool, stock, initial, first_id, cancel)?,
            ))),
            Operation::Waterline(p) => Ok(Self::Waterline(Box::new(
                WaterlinePlan::prepare_with_stock(p, tool, stock, initial, first_id, cancel)?,
            ))),
            Operation::Pocket(p) => {
                let plan =
                    PocketPlan::prepare_with_stock(p, tool, stock, initial, first_id, cancel)?;
                plan.refuse_no_cutter_position()?;
                Ok(Self::Pocket(Box::new(plan)))
            }
            Operation::AdaptivePocket(p) => Ok(Self::AdaptiveFront(Box::new(
                crate::pocket_front::PocketFrontPlan::prepare_with_stock(
                    p, tool, stock, initial, first_id, cancel,
                )?,
            ))),
            Operation::Roughing(p) => Ok(Self::Roughing(Box::new(
                RoughingPlan::prepare_with_stock(p, tool, stock, initial, first_id, cancel)?,
            ))),
            // A mark is planned against the stock: its parts must not be cut yet.
            Operation::ThermalMark(p) => Ok(Self::ThermalMark(Box::new(
                crate::thermal_mark::ThermalMarkPlan::prepare_with_stock(
                    p, tool, stock, initial, first_id, cancel,
                )?,
            ))),
            other => Self::prepare(other, tool, stock.spec(), initial, first_id, cancel),
        }
    }
    /// Resolve declared drilling dependencies from the complete source run.
    pub(crate) fn prepare_in_run(
        manifest: &crate::stream::StreamManifest,
        index: usize,
        stock: &Stock,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        let mut op = manifest
            .operations
            .get(index)
            .ok_or("operation index missing")?
            .clone();
        let report = crate::holes::preparation_report(manifest, index, cancel)?;
        let tool = manifest
            .tools
            .iter()
            .find(|t| t.id == op.tool())
            .ok_or("operation tool missing")?;
        if let Operation::Drilling(d) = &mut op {
            d.preparation = None;
        }
        let mut plan = Self::prepare_with_stock(op, tool, stock, initial, first_id, cancel)?;
        if let Self::Drilling(d) = &mut plan {
            d.set_preparation_report(report);
        }
        Ok(plan)
    }
    pub fn prepare(
        op: Operation,
        tool: &ToolAssembly,
        stock: &StockSpec,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        let op = hole_limits(op, tool)?;
        probe_only_probes(&op, tool)?;
        match op {
            Operation::Probe(p) => Ok(Self::Probe(Box::new(crate::probe::ProbePlan::prepare(
                p, tool, stock, initial, first_id, cancel,
            )?))),
            Operation::SuppliedToolpath(p) => Ok(Self::SuppliedToolpath(Box::new(
                crate::supplied::SuppliedToolpathPlan::prepare(
                    p, tool, stock, initial, first_id, cancel,
                )?,
            ))),
            Operation::Roughing(p) => Ok(Self::Roughing(Box::new(RoughingPlan::prepare(
                p, tool, stock, initial, first_id, cancel,
            )?))),
            Operation::AdaptivePocket(p) => Ok(Self::AdaptiveFront(Box::new(
                crate::pocket_front::PocketFrontPlan::prepare(
                    p, tool, stock, initial, first_id, cancel,
                )?,
            ))),
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
            Operation::ThermalMark(p) => Err(format!(
                "thermal mark {}: marking is planned against the stock (its parts must not be cut yet); use OperationPlan::prepare_with_stock",
                p.id
            )),
            Operation::VCarve(p) => Ok(Self::VCarve(Box::new(crate::vcarve::VCarvePlan::prepare(
                p, tool, stock, initial, first_id, cancel,
            )?))),
            Operation::Tapping(p) => Ok(Self::Tapping(Box::new(TappingPlan::prepare(
                p, tool, stock, initial, first_id, cancel,
            )?))),
            Operation::Facing(p) => Ok(Self::Facing(Box::new(FacingPlan::prepare(
                p, tool, stock, initial, first_id, cancel,
            )?))),
            Operation::Pocket(p) => {
                let plan = PocketPlan::prepare(p, tool, stock, initial, first_id, cancel)?;
                plan.refuse_no_cutter_position()?;
                Ok(Self::Pocket(Box::new(plan)))
            }
            Operation::TrochoidalSlot(p) => Ok(Self::Slot(Box::new(SlotPlan::prepare(
                p, tool, stock, initial, first_id, cancel,
            )?))),
        }
    }
    pub fn report(&self) -> OperationReport<'_> {
        match self {
            Self::Roughing(p) => OperationReport::Roughing(p.report()),
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
            Self::ThermalMark(p) => OperationReport::ThermalMark(p.report()),
            Self::Probe(p) => OperationReport::Probe(p.report()),
            Self::SuppliedToolpath(p) => OperationReport::SuppliedToolpath(p.report()),
        }
    }
    /// Planning progress (#198): where the planner stands in its own
    /// counts, after `prepare` or after any motion. `None` for an operation
    /// with nothing honest to count (never an estimate).
    pub fn progress(&self) -> Option<crate::progress::Planned> {
        match self {
            Self::Roughing(p) => Some(p.progress()),
            Self::AdaptiveFront(p) => Some(p.progress()),
            Self::Pocket(p) => Some(p.progress()),
            Self::Contour(p) => Some(p.progress()),
            Self::Chamfer(p) => Some(p.progress()),
            Self::Parallel(p) => Some(p.progress()),
            Self::Waterline(p) => Some(p.progress()),
            _ => None,
        }
    }
    pub fn prefix_events(&self) -> Vec<Event> {
        match self {
            Self::Roughing(p) => p.prefix_events(),
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
            Self::ThermalMark(p) => p.prefix_events(),
            Self::Probe(p) => p.prefix_events(),
            Self::SuppliedToolpath(p) => p.prefix_events(),
        }
    }
    pub fn suffix_events(&self) -> Vec<Event> {
        match self {
            Self::Roughing(p) => p.suffix_events(),
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
            Self::ThermalMark(p) => p.suffix_events(),
            Self::Probe(p) => p.suffix_events(),
            Self::SuppliedToolpath(p) => p.suffix_events(),
        }
    }
    pub fn next_motion_with_stock(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        match self {
            Self::Roughing(p) => p.next_motion_with_stock(stock, cancel),
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
            Self::ThermalMark(p) => p.next_motion_with_stock(stock, cancel),
            Self::Probe(p) => p.next_motion_with_stock(stock, cancel),
            Self::SuppliedToolpath(p) => p.next_motion_with_stock(stock, cancel),
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
            Self::ThermalMark(p) => p.take_events(),
            Self::Probe(p) => p.take_events(),
            _ => vec![],
        }
    }
    pub fn finish_report(&mut self, cancel: &impl CancelSignal) -> Result<(), String> {
        match self {
            Self::Roughing(p) => p.finish_report(cancel),
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
            Self::ThermalMark(p) => p.finish_report(cancel),
            Self::Probe(p) => p.finish_report(cancel),
            Self::SuppliedToolpath(p) => p.finish_report(cancel),
        }
    }
}
