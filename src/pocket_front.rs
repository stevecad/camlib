//! An adaptive pocket cleared by the engagement front (issue #190): the
//! stock-driven planner `Roughing` uses (`roughing::engagement_front`), one
//! depth level at a time, inside the pocket's region.
//!
//! The region's boundary is a wall: cutter centres stay the cutter radius
//! and the radial allowance inside it, exactly certified stroke by stroke.
//! Material is what the committed stock holds, so nothing is cut twice: a
//! level starts with a helix into uncut material (or straight down through
//! stock an earlier operation removed, where the committed cuts prove it
//! air), and every cutting stroke is proven within the contact angle of the
//! declared width, acos(1 - w/r).
use crate::{
    adaptive::{AdaptivePathStrategy, AdaptivePocketOperation, CuttingSense},
    entry::PocketEntry,
    motion::{Event, Motion, Vec3},
    pocket::ResidualStatus,
    roughing::EngagementFrontPlan,
    stock::{CancelSignal, Stock, StockSpec},
    tool::ToolAssembly,
};
use serde::Serialize;
use std::collections::VecDeque;

#[derive(Clone, Debug, Serialize)]
pub struct PocketFrontReport {
    pub strategy: &'static str,
    pub clearing: &'static str,
    pub target_z_mm: f64,
    /// The radial width (the operation's `stepover_mm`, narrowed by a
    /// `max_engagement_deg` below its own angle) and the contact angle of a
    /// straight pass that wide, acos(1 - w/r): the limit every cutting
    /// stroke is proven against.
    pub max_radial_engagement_mm: f64,
    pub max_engagement_deg: f64,
    /// Each depth level's tip height and its `EngagementFrontReport`.
    pub layers: Vec<PocketFrontLayer>,
    pub completed_layers: usize,
    pub active_layer: usize,
    pub motions: usize,
    pub stock_edge_entries: usize,
    pub helical_entries: usize,
    pub geometric_residual_status: ResidualStatus,
    pub diagnostics: Vec<String>,
}
#[derive(Clone, Debug, Serialize)]
pub struct PocketFrontLayer {
    pub tip_z_mm: f64,
    pub clearing: serde_json::Value,
}

pub struct PocketFrontPlan {
    plans: VecDeque<EngagementFrontPlan>,
    prefix: Vec<Event>,
    suffix: Vec<Event>,
    activated: bool,
    position: Vec3,
    next_id: u64,
    generation: Option<u64>,
    spec: StockSpec,
    report: PocketFrontReport,
}

impl PocketFrontPlan {
    pub fn prepare(
        op: AdaptivePocketOperation,
        tool: &ToolAssembly,
        spec: &StockSpec,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        debug_assert_eq!(op.path_strategy, AdaptivePathStrategy::EngagementFront);
        let pocket = &op.pocket;
        if op.orbit_radius_mm.is_some()
            || op.min_orbit_radius_mm.is_some()
            || op.local_orbit_tolerance_mm.is_some()
        {
            return Err("engagement_front does not use orbits; leave orbit_radius_mm, min_orbit_radius_mm and local_orbit_tolerance_mm unset".into());
        }
        if pocket.max_stay_down_mm.is_some() || pocket.rest_tolerance_mm.is_some() {
            return Err("engagement_front links through its own proven-clear moves and plans from the committed stock; leave max_stay_down_mm and rest_tolerance_mm unset".into());
        }
        // Entry and link options, by Roughing's rules (one validator).
        let motion = op.front_motion_options();
        motion.validate(&format!("adaptive pocket {}", pocket.id), None)?;
        let r = tool.prepare()?.cutter().definition().diameter_mm / 2.;
        // The radial width is the operation's stepover: how far each pass
        // steps into the material.
        let stepover = pocket.stepover_mm;
        if !op.max_engagement_deg.is_finite() || !(1. ..=170.).contains(&op.max_engagement_deg) {
            return Err(format!(
                "adaptive pocket {}: max_engagement_deg must be within 1..170",
                pocket.id
            ));
        }
        // A contact angle limit below the stepover's own narrows the width.
        let width = if op.max_engagement_deg < 90. {
            stepover.min(r * (1. - op.max_engagement_deg.to_radians().cos()))
        } else {
            stepover
        };
        if !stepover.is_finite() || stepover <= 0. || stepover > r {
            return Err(format!(
                "adaptive pocket {}: stepover_mm {stepover} is the radial engagement of each pass; it must be positive and at most the cutter radius {r} mm",
                pocket.id
            ));
        }
        let floor = pocket.floor_z_mm + pocket.bottom_allowance_mm;
        if !(pocket.stepdown_mm.is_finite() && pocket.stepdown_mm > 0.) || floor >= pocket.top_z_mm
        {
            return Err(format!(
                "adaptive pocket {}: needs a positive stepdown and a floor below its top",
                pocket.id
            ));
        }
        // A pocket is enclosed: a helical entry policy means a helix where
        // there is material and a straight descent only through stock the
        // committed cuts prove gone, which is what the planner does.
        // With no entry given, the helix is HelicalEntry's own default: an
        // adaptive pocket has no plunge into uncut material.
        if let Some(entry) = &pocket.entry {
            // The planner proves the engagement of every stroke from an entry
            // made in air, in cleared stock or on a helix; a ramp along the
            // first pass is a full-width slot, outside that proof.
            entry.refuse_ramp(
                &format!("adaptive pocket {}", pocket.id),
                "adaptive clearing enters from cleared stock or on a helix so that every pass keeps its proven engagement",
            )?;
        }
        let entry = Some(match pocket.entry.clone() {
            Some(PocketEntry::Helix(h)) => PocketEntry::StockEdgeOrHelix(h),
            Some(other) => other,
            None => PocketEntry::StockEdgeOrHelix(crate::entry::HelicalEntry::default()),
        });
        let climb = op.cutting_sense.unwrap_or(CuttingSense::Climb) == CuttingSense::Climb;
        let guard = 16. * pocket.tolerance_mm;
        let levels = ((pocket.top_z_mm - floor) / pocket.stepdown_mm - 1e-9)
            .ceil()
            .max(1.) as usize;
        let mut plans = VecDeque::new();
        let mut layers = vec![];
        let mut above: Option<f64> = None;
        for level in 0..levels {
            if cancel.is_cancelled() {
                return Err("adaptive pocket preparation cancelled".into());
            }
            let z = (pocket.top_z_mm - (level + 1) as f64 * pocket.stepdown_mm).max(floor);
            // Earlier levels clear the space above this one. Keep the stock
            // top while the flutes span it; deeper levels admit only their
            // own increment below the level above (as roughing layers do).
            let mut layer_spec = spec.clone();
            if spec.max_mm[2] - z > tool.cutter.cutting_length_mm
                && let Some(above) = above
            {
                layer_spec.max_mm[2] = above + guard;
            }
            let cut_depth = layer_spec.max_mm[2] - z;
            if cut_depth > tool.cutter.cutting_length_mm {
                return Err(crate::reach::ReachRefusal::cutting_length(
                    &pocket.id,
                    &tool.id,
                    layer_spec.max_mm[2],
                    z,
                    tool.cutter.cutting_length_mm,
                )
                .error(format_args!(
                    "adaptive pocket {} level {level} at Z{z}: cut depth {cut_depth} mm exceeds cutting length {} mm; use a smaller stepdown or longer flutes",
                    pocket.id, tool.cutter.cutting_length_mm
                )));
            }
            let mut layer = pocket.clone();
            layer.top_z_mm = layer_spec.max_mm[2];
            layer.floor_z_mm = z;
            layer.stepdown_mm = layer_spec.max_mm[2] - z;
            layer.bottom_allowance_mm = 0.;
            layer.entry = entry.clone();
            let plan = EngagementFrontPlan::prepare(
                layer,
                tool,
                &layer_spec,
                initial,
                width,
                climb,
                cancel,
            )
            .map_err(|e| format!("adaptive pocket level {level} at Z{z}: {e}"))?
            // A pocket's walls are finished surfaces.
            .with_wall_pass(true)
            // Each level proves its own rapid descents and fast links from
            // the stock committed when it plans them, as a roughing layer
            // does: the level's operation is built the same way (its top the
            // stock top, or just above the level before it).
            .with_motion_options(motion);
            layers.push(PocketFrontLayer {
                tip_z_mm: z,
                clearing: serde_json::to_value(plan.report()).unwrap(),
            });
            plans.push_back(plan);
            above = Some(z);
        }
        let prefix = plans.front().map_or_else(Vec::new, |p| p.prefix_events());
        let suffix = plans.front().map_or_else(Vec::new, |p| p.suffix_events());
        Ok(Self {
            plans,
            prefix,
            suffix,
            activated: false,
            position: initial,
            next_id: first_id,
            generation: None,
            spec: spec.clone(),
            report: PocketFrontReport {
                strategy: "adaptive_pocket",
                clearing: "engagement_front",
                target_z_mm: floor,
                max_radial_engagement_mm: width,
                max_engagement_deg: (1. - width / r).acos().to_degrees(),
                layers,
                completed_layers: 0,
                active_layer: 0,
                motions: 0,
                stock_edge_entries: 0,
                helical_entries: 0,
                geometric_residual_status: ResidualStatus::Pending,
                diagnostics: vec![
                    "Each depth level follows the boundary between cut and uncut stock inside the pocket region; every cutting stroke is proven within max_engagement_deg = acos(1 - stepover/r) against the committed cleared sweeps".into(),
                ],
            },
        })
    }
    pub fn prepare_with_stock(
        op: AdaptivePocketOperation,
        tool: &ToolAssembly,
        stock: &Stock,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        Self::prepare(op, tool, stock.spec(), initial, first_id, cancel)
    }
    pub fn report(&self) -> &PocketFrontReport {
        &self.report
    }
    pub fn prefix_events(&self) -> Vec<Event> {
        self.prefix.clone()
    }
    pub fn suffix_events(&self) -> Vec<Event> {
        self.suffix.clone()
    }
    pub fn next_motion_with_stock(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        if cancel.is_cancelled() {
            return Err("adaptive pocket planning cancelled".into());
        }
        if stock.spec() != &self.spec
            || stock.generation() == u64::MAX
            || self.generation.is_some_and(|g| g != stock.generation())
        {
            return Err(
                "commit preceding adaptive pocket motion to matching stock before planning".into(),
            );
        }
        loop {
            let Some(plan) = self.plans.front_mut() else {
                return Ok(None);
            };
            if !self.activated {
                plan.activate_layer(self.position, self.next_id, &self.spec)?;
                self.activated = true;
            }
            let layer = self.report.active_layer;
            let before = (
                plan.report().stock_edge_entries,
                plan.report().helical_entries,
            );
            if let Some(m) = plan.next_motion_with_stock(stock, cancel).map_err(|e| {
                format!(
                    "{e}; adaptive pocket level {layer} at Z{}",
                    self.report.layers[layer].tip_z_mm
                )
            })? {
                self.report.stock_edge_entries += plan.report().stock_edge_entries - before.0;
                self.report.helical_entries += plan.report().helical_entries - before.1;
                self.position = m.curve.end();
                self.next_id = m.id + 1;
                self.generation = Some(stock.generation() + 1);
                self.report.motions += 1;
                return Ok(Some(m));
            }
            plan.finish_report(cancel)?;
            self.report.layers[layer].clearing = serde_json::to_value(plan.report()).unwrap();
            self.report.completed_layers += 1;
            self.report.active_layer += 1;
            let memory = self
                .plans
                .pop_front()
                .and_then(|mut done| done.take_memory());
            if let (Some(memory), Some(next)) = (memory, self.plans.front_mut()) {
                next.give_memory(memory);
            }
            self.activated = false;
        }
    }
    pub fn finish_report(&mut self, cancel: &impl CancelSignal) -> Result<(), String> {
        if cancel.is_cancelled() {
            return Err("adaptive pocket report cancelled".into());
        }
        if self.plans.is_empty() {
            self.report.geometric_residual_status = ResidualStatus::Estimated;
        }
        Ok(())
    }
}
