//! Closed retained-part profiles. Full continuous source admission precedes
//! emission; cutting a contour band is not a claim that all stock was cleared.
use crate::{
    motion::{CompensationSide, Coolant, Curve, Event, Motion, MoveKind, Vec3},
    planar::{ConstructionOptions, PlanarRegion},
    pocket::ResidualStatus,
    stock::{CancelSignal, Stock, StockSpec},
    tool::{CutterShape, ToolAssembly, valid_id},
    verification,
};
use serde::{Deserialize, Serialize};
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Boundaries {
    Outer,
    Holes,
    All,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ContourOperation {
    pub id: String,
    pub tool: String,
    pub region: PlanarRegion,
    pub boundaries: Boundaries,
    pub floor_z_mm: f64,
    pub stepdown_mm: f64,
    pub clearance_z_mm: f64,
    pub feed_mm_min: f64,
    pub plunge_mm_min: f64,
    pub radial_allowance_mm: f64,
    pub bottom_allowance_mm: f64,
    pub boundary_tolerance_mm: f64,
    pub tolerance_mm: f64,
    pub reverse: bool,
    pub lead_in_mm: f64,
    pub lead_out_mm: f64,
    pub spindle_rpm: f64,
    pub work_offset: u8,
    pub tool_number: u32,
    pub length_register: u32,
    pub coolant: Coolant,
    /// Rapid travel height between passes (absolute Z). Default: clearance.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub retract_z_mm: Option<f64>,
    /// Rapid descent stops here and plunge feed begins; leaving a pass, feed
    /// changes to rapid here (absolute Z, above the stock top). Default: retract.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub feed_z_mm: Option<f64>,
    /// "computer" (default): the path is the tool center, no G41/G42.
    /// "wear": the same tool-center path, posted with G41/G42 so the
    /// controller adds the tool's wear register (turned on over the lead-in,
    /// off over the lead-out).
    #[serde(default, skip_serializing_if = "Compensation::is_computer")]
    pub compensation: Compensation,
    /// Holding tabs left on the part (through-cuts).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tabs: Option<Tabs>,
    /// Containment and avoid areas (clamps, fixtures, features to leave alone).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub limits: Option<crate::limits::Limits>,
    /// `RampEntry`: each loop is cut in one continuous descent around it,
    /// from the stock top to the floor, at no more than the ramp angle and
    /// no more than `stepdown_mm` per lap, then once more flat at the floor.
    /// None: a pass per level, each a straight plunge at the lead-in point
    /// (the default). Only a ramp is accepted: a profile has no pocket to
    /// fit a helix in.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub entry: Option<crate::entry::PocketEntry>,
}
/// Holding tabs: `count` per loop (or one per `spacing_mm` of loop length),
/// or one at each of `positions_mm`; `width_mm` wide along the part edge,
/// `height_mm` above the floor.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Tabs {
    pub width_mm: f64,
    pub height_mm: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub count: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub spacing_mm: Option<f64>,
    /// Tab centres as XY points on the selected part boundaries, in the
    /// region's coordinates, instead of `count`/`spacing_mm` (as "Tab
    /// Positioning: At points" in Fusion 2D Contour and the tag positions of
    /// the FreeCAD CAM Tag dressup). Each point must lie within
    /// `position_tolerance_mm` of exactly one selected boundary loop; that
    /// loop gets the tab. A loop with no point gets no tab.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub positions_mm: Option<Vec<[f64; 2]>>,
    /// How far a position may be from the part boundary and still be taken
    /// as the nearest point on it. Required with `positions_mm`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub position_tolerance_mm: Option<f64>,
    pub shape: TabShape,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TabShape {
    /// 45 degree ramps up and down.
    Triangle,
    /// Straight up and over.
    Rectangle,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Compensation {
    #[default]
    Computer,
    Wear,
}
impl Compensation {
    fn is_computer(&self) -> bool {
        *self == Compensation::Computer
    }
}
#[derive(Debug, Serialize)]
pub struct ContourReport {
    pub strategy: &'static str,
    pub geometric_residual_status: ResidualStatus,
    pub target_z_mm: f64,
    pub depth_passes: usize,
    pub loops: usize,
    pub xy_edges: usize,
    pub applied_clearance_mm: f64,
    pub path_safety_margin_mm: f64,
    pub boundary_tolerance_mm: f64,
    pub source_error_mm: f64,
    pub preflight_nodes: u64,
    pub preflight_boundary_tests: u64,
    pub cut_edges_emitted: usize,
    pub loops_completed: usize,
    pub retained_analytic_arcs: usize,
    pub machining_scope: &'static str,
    pub wall_completion: &'static str,
    pub diagnostics: Vec<String>,
}
pub struct ContourPlan {
    motion: crate::profiles::ProfileMotion,
    report: ContourReport,
}
use crate::profiles::{ProfileDepths, ProfileMotion, ProfileSettings, lead_point};
impl ContourPlan {
    pub fn prepare(
        op: ContourOperation,
        assembly: &ToolAssembly,
        spec: &StockSpec,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        if cancel.is_cancelled() {
            return Err("contour planning cancelled".into());
        }
        spec.grid()?;
        let tool = assembly.prepare()?;
        let cutter = tool.cutter().definition();
        let target = op.floor_z_mm + op.bottom_allowance_mm;
        let values = [
            target,
            op.floor_z_mm,
            op.bottom_allowance_mm,
            op.stepdown_mm,
            op.clearance_z_mm,
            op.feed_mm_min,
            op.plunge_mm_min,
            op.radial_allowance_mm,
            op.boundary_tolerance_mm,
            op.tolerance_mm,
            op.lead_in_mm,
            op.lead_out_mm,
            op.spindle_rpm,
        ];
        let guard = 16. * op.tolerance_mm;
        crate::heights::require(
            "contour",
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
                    matches!(cutter.shape, CutterShape::Flat) && cutter.center_cutting,
                    "contour needs a flat, center-cutting end mill".into(),
                ),
                (
                    values.iter().all(|n| n.is_finite()),
                    "all numeric parameters must be finite".into(),
                ),
                (
                    (1e-7..=0.01).contains(&op.tolerance_mm),
                    format!("tolerance_mm {} must be within 1e-7..0.01", op.tolerance_mm),
                ),
                (
                    (0.0001..=0.1).contains(&op.boundary_tolerance_mm),
                    format!(
                        "boundary_tolerance_mm {} must be within 0.0001..0.1",
                        op.boundary_tolerance_mm
                    ),
                ),
                (
                    op.tolerance_mm <= op.boundary_tolerance_mm / 64.,
                    format!(
                        "tolerance_mm {} must be at most boundary_tolerance_mm / 64 ({})",
                        op.tolerance_mm,
                        op.boundary_tolerance_mm / 64.
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
                    spec.max_mm[2] - target <= cutter.cutting_length_mm - guard,
                    crate::reach::ReachRefusal::cutting_length(&op.id, &op.tool, spec.max_mm[2], target, cutter.cutting_length_mm).error(format_args!(
                        "contour depth {} below the stock top exceeds the cutting length {}",
                        spec.max_mm[2] - target,
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
                    op.feed_mm_min > 0. && op.plunge_mm_min > 0. && op.spindle_rpm > 0.,
                    format!(
                        "feed_mm_min {}, plunge_mm_min {} and spindle_rpm {} must be positive",
                        op.feed_mm_min, op.plunge_mm_min, op.spindle_rpm
                    ),
                ),
                (
                    op.lead_in_mm >= 0. && op.lead_out_mm >= 0.,
                    "lead_in_mm and lead_out_mm must not be negative".into(),
                ),
                (
                    op.compensation == Compensation::Computer
                        || (op.lead_in_mm > guard && op.lead_out_mm > guard),
                    "wear compensation turns G41/G42 on over the lead-in and off over the lead-out: set lead_in_mm and lead_out_mm (longer than the largest wear)".into(),
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
        let ramp = match &op.entry {
            None => None,
            Some(crate::entry::PocketEntry::Ramp(ramp)) => {
                ramp.validate(op.tolerance_mm, op.plunge_mm_min)
                    .map_err(|e| format!("contour {}: {e}", op.id))?;
                if op.tabs.is_some() {
                    return Err(format!(
                        "contour {}: a ramp entry cannot be combined with holding tabs: the tool comes back down from a tab steeper than the ramp angle. Remove the entry or the tabs",
                        op.id
                    ));
                }
                Some(ramp.clone())
            }
            Some(_) => {
                return Err(format!(
                    "contour {}: the entry must be a RampEntry (or None for a plunge): a profile has no pocket to fit a helical or stock-edge entry in",
                    op.id
                ));
            }
        };
        let options = ConstructionOptions {
            tolerance_mm: op.tolerance_mm,
        };
        let source = op.region.prepare(options)?;
        let error = source.summary().boundary_error_mm;
        let margin = op.boundary_tolerance_mm / 2.;
        if 2. * error + margin + 16. * op.tolerance_mm > op.boundary_tolerance_mm {
            return Err("contour CAD/source uncertainty exceeds boundary tolerance budget".into());
        }
        let clearance = cutter.diameter_mm / 2. + op.radial_allowance_mm + error + margin;
        let prepared = crate::profiles::offset_profiles(
            &source,
            clearance,
            op.boundaries,
            op.reverse,
            options,
            cancel,
        )?;
        let edges = prepared.edges;
        let arcs = prepared.arcs;
        // G41 when the part is on the right of the travel direction, G42
        // when it is on the left (the part is inside an outer loop, outside
        // a hole's loop).
        let sides = prepared
            .loops
            .iter()
            .map(|p| {
                let mut area = 0.;
                let mut points = vec![];
                for s in &p.strokes {
                    let c = s.curve(target);
                    let prepared = c.prepare(op.tolerance_mm)?;
                    points.push(c.start());
                    points.push(prepared.position(0.5)?);
                }
                for (a, b) in points.iter().zip(points.iter().cycle().skip(1)) {
                    area += a[0] * b[1] - b[0] * a[1];
                }
                let part_left = (area > 0.) == p.hole.is_none();
                Ok(if part_left {
                    CompensationSide::Right
                } else {
                    CompensationSide::Left
                })
            })
            .collect::<Result<Vec<_>, String>>()?;
        let owners = prepared
            .loops
            .iter()
            .map(|p| (p.component, p.hole))
            .collect::<Vec<_>>();
        let loops = prepared
            .loops
            .into_iter()
            .map(|p| p.strokes)
            .collect::<Vec<_>>();
        if let Some(ramp) = &ramp {
            // Refused before any motion.
            for (index, ring) in loops.iter().enumerate() {
                let length: f64 = ring.iter().map(crate::profiles::stroke_length).sum();
                ramp.laps(length, spec.max_mm[2] - target, op.stepdown_mm)
                    .map_err(|e| format!("contour {}, loop {index}: {e}", op.id))?;
            }
        }
        let passes = ((spec.max_mm[2] - target) / op.stepdown_mm).ceil() as usize;
        // A ramp is one continuous descent per loop, not a pass per level.
        let passes = if ramp.is_some() {
            passes.min(1)
        } else {
            passes
        };
        if loops.is_empty() || passes == 0 || first_id == 0 || first_id.checked_add(1).is_none() {
            return Err(
                "contour has no selected loops or exceeds bounded geometry/passes/motion IDs"
                    .into(),
            );
        }
        // Checked part by part (only the parts the tool can reach).
        let scene = verification::ComponentScenes::new(
            "contour-source",
            &op.id,
            "retained-part",
            source.definition(),
            [spec.min_mm[2], spec.max_mm[2]],
            verification::Options {
                tolerance_mm: op.tolerance_mm,
                spatial_resolution_mm: (op.boundary_tolerance_mm / 16.).max(op.tolerance_mm),
                ..Default::default()
            },
        )?;
        let mut nodes = 0;
        let mut tests = 0;
        for ring in &loops {
            let start = ring[0].curve(target).start();
            let finish = ring.last().unwrap().curve(target).end();
            let curves = ring.iter().map(|s| s.curve(target)).chain([
                Curve::Line {
                    start: lead_point(ring, target, op.lead_in_mm, false),
                    end: start,
                },
                Curve::Line {
                    start: finish,
                    end: lead_point(ring, target, op.lead_out_mm, true),
                },
            ]);
            for curve in curves {
                if matches!(curve,Curve::Line{start,end} if start==end) {
                    continue;
                }
                let motion = Motion {
                    id: first_id,
                    operation: op.id.clone(),
                    tool: op.tool.clone(),
                    curve,
                    kind: MoveKind::Cut,
                    feed_mm_min: Some(op.feed_mm_min),
                };
                let checked = scene.verify_motion(&tool, &motion, cancel)?;
                nodes += checked.nodes;
                tests += checked.boundary_tests;
                if let Some(check) = checked.unclear {
                    return Err(format!(
                        "contour protected-part continuous admission {:?}; change cutter/lead/geometry or tolerance: {:?}",
                        check.status,
                        check.findings.first()
                    ));
                }
            }
        }
        let mut report = ContourReport {
            strategy: "contour",
            geometric_residual_status: ResidualStatus::Pending,
            target_z_mm: target,
            depth_passes: passes,
            loops: loops.len(),
            xy_edges: edges,
            applied_clearance_mm: clearance,
            path_safety_margin_mm: margin,
            boundary_tolerance_mm: op.boundary_tolerance_mm,
            source_error_mm: error,
            preflight_nodes: nodes,
            preflight_boundary_tests: tests,
            cut_edges_emitted: 0,
            loops_completed: 0,
            retained_analytic_arcs: arcs,
            machining_scope: "selected profile band; outside stock is not cleared",
            wall_completion: "not_certified; concave/unreachable residual stock needs independent material checks",
            diagnostics: prepared.diagnostics,
        };
        let heights = crate::heights::Heights::resolve(
            op.clearance_z_mm,
            op.retract_z_mm,
            op.feed_z_mm,
            spec.max_mm[2],
            16. * op.tolerance_mm,
        )?;
        let compensation = op.compensation;
        let limits = op.limits.clone().filter(|l| !l.is_empty());
        let tabs = op
            .tabs
            .as_ref()
            .map(|t| -> Result<crate::profiles::TabPlan, String> {
                if t.positions_mm.is_some() && (t.count.is_some() || t.spacing_mm.is_some()) {
                    return Err("tabs are placed either at positions_mm or evenly by count or spacing_mm, not both: give only one of them".into());
                }
                if t.positions_mm.is_none() && t.position_tolerance_mm.is_some() {
                    return Err("tabs position_tolerance_mm applies only to positions_mm: give positions_mm or remove it".into());
                }
                if !(t.width_mm.is_finite() && t.width_mm > 0. && t.height_mm.is_finite() && t.height_mm > 0.)
                    || (t.positions_mm.is_none() && t.count.is_some() == t.spacing_mm.is_some())
                    || t.count.is_some_and(|n| n == 0 || n > 1000)
                    || t.spacing_mm.is_some_and(|s| !(s.is_finite() && s > 0.))
                {
                    return Err("tabs need a positive width and height, and either a count (1..1000) or a spacing".into());
                }
                if t.shape == TabShape::Rectangle && compensation == Compensation::Wear {
                    return Err("rectangular tabs move straight up and down, which cutter compensation does not allow: use triangular tabs".into());
                }
                let top = target + t.height_mm;
                let ramp = if t.shape == TabShape::Triangle { (top - target).max(0.) } else { 0. };
                let half = t.width_mm / 2. + cutter.diameter_mm / 2.;
                if let Some(points) = &t.positions_mm {
                    let tolerance = t.position_tolerance_mm.ok_or(
                        "tabs at positions_mm need position_tolerance_mm: how far a point may be from the part boundary and still be taken as on it",
                    )?;
                    if !(tolerance.is_finite() && tolerance > 0.) {
                        return Err(format!("tabs position_tolerance_mm {tolerance} must be positive"));
                    }
                    if points.is_empty() || points.len() > 1000 {
                        return Err(format!("tabs positions_mm needs 1..1000 points, not {}", points.len()));
                    }
                    if let Some(p) = points.iter().find(|p| !p.iter().all(|n| n.is_finite() && n.abs() <= 1e6)) {
                        return Err(format!("tab position ({}, {}) is not a finite point", p[0], p[1]));
                    }
                    // The part boundary each tool-center loop was offset from.
                    let definition = source.definition();
                    let boundaries = owners
                        .iter()
                        .map(|&(component, hole)| {
                            let c = &definition.components[component];
                            crate::pocket::Stroke::from_boundary(hole.map_or(&c.outer, |h| &c.holes[h]))
                        })
                        .collect::<Vec<_>>();
                    let centers = crate::profiles::tab_centers_at_points(
                        &boundaries,
                        &loops,
                        points,
                        tolerance,
                        half + ramp + guard,
                        t.width_mm,
                    )?;
                    for (k, (&(component, hole), c)) in owners.iter().zip(&centers).enumerate() {
                        if c.is_empty() {
                            let name = hole.map_or_else(
                                || format!("the outer boundary of component {component}"),
                                |h| format!("hole {h} of component {component}"),
                            );
                            report.diagnostics.push(format!(
                                "tabs: loop {k} ({name}) has no tab position and is cut without tabs"
                            ));
                        }
                    }
                    return Ok(crate::profiles::TabPlan { top, half, ramp: t.shape == TabShape::Triangle, centers });
                }
                let centers = loops
                    .iter()
                    .map(|ring| {
                        let length: f64 = ring.iter().map(crate::profiles::stroke_length).sum();
                        let n = t.count.map_or_else(
                            || (length / t.spacing_mm.unwrap()).round().max(1.) as usize,
                            |n| n as usize,
                        );
                        // Room for each tab, its ramps and a gap, away from
                        // the loop start (the leads).
                        if length / (2. * n as f64) <= half + ramp + guard {
                            return Err(format!(
                                "{n} tabs of width {} (plus the tool diameter and ramps) do not fit a loop {length:.3} mm long",
                                t.width_mm
                            ));
                        }
                        Ok((0..n).map(|k| (k as f64 + 0.5) * length / n as f64).collect())
                    })
                    .collect::<Result<Vec<Vec<f64>>, String>>()?;
                Ok(crate::profiles::TabPlan { top, half, ramp: t.shape == TabShape::Triangle, centers })
            })
            .transpose()?;
        let motion_tolerance = op.tolerance_mm;
        let settings = ProfileSettings {
            id: op.id,
            tool: op.tool,
            heights,
            feed: op.feed_mm_min,
            plunge: op.plunge_mm_min,
            tolerance: op.tolerance_mm,
            lead_in: op.lead_in_mm,
            lead_out: op.lead_out_mm,
            spindle: op.spindle_rpm,
            work_offset: op.work_offset,
            tool_number: op.tool_number,
            length_register: op.length_register,
            coolant: op.coolant,
        };
        let depths = ProfileDepths::Linear {
            top: spec.max_mm[2],
            target,
            step: op.stepdown_mm,
        };
        let mut motion =
            ProfileMotion::new(settings, spec, initial, loops, depths, passes, first_id);
        if compensation == Compensation::Wear {
            motion.sides = sides;
        }
        motion.tabs = tabs;
        let ramped = ramp.is_some();
        if let Some(ramp) = ramp {
            motion.ramp_feed = ramp.feed_mm_min.unwrap_or(op.plunge_mm_min);
            motion.entry_ramp = Some(ramp);
        }
        if let Some(limits) = limits {
            motion.limits = Some(crate::limits::PreparedLimits::new(
                &limits,
                cutter.diameter_mm / 2.,
                motion_tolerance,
            )?);
            if ramped {
                // Refused before any motion: a stretch the limits block is
                // lifted over, and the tool comes straight back down.
                for ring in 0..motion.loop_count() {
                    if !motion.lifts(ring, target)?.is_empty() {
                        return Err(format!(
                            "contour {}: a ramp entry cannot be combined with limits that block part of loop {ring}: the tool is lifted over the blocked stretch and comes straight back down into material, steeper than the ramp angle. Remove the entry or change the limits",
                            motion.operation_id()
                        ));
                    }
                }
            }
        }
        Ok(Self { motion, report })
    }

    pub fn report(&self) -> &ContourReport {
        &self.report
    }
    /// Planning progress (#198): depth passes done of those planned.
    pub fn progress(&self) -> crate::progress::Planned {
        self.motion.progress()
    }
    pub fn take_events(&mut self) -> Vec<Event> {
        self.motion.take_events()
    }
    pub fn prefix_events(&self) -> Vec<Event> {
        self.motion.prefix_events()
    }
    pub fn suffix_events(&self) -> Vec<Event> {
        self.motion.suffix_events()
    }
    pub fn finish_report(&mut self, cancel: &impl CancelSignal) -> Result<(), String> {
        if cancel.is_cancelled() {
            return Err("contour report cancelled".into());
        }
        if !self.motion.drained {
            return Err("drain committed contour motions first".into());
        }
        self.report.geometric_residual_status = ResidualStatus::Unknown;
        Ok(())
    }
    pub fn next_motion_with_stock(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        let motion = self.motion.next(stock, cancel)?;
        self.report.cut_edges_emitted = self.motion.cuts;
        self.report.loops_completed = self.motion.closed_loops;
        Ok(motion)
    }
}
