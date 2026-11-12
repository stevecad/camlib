//! Top chamfers certified against height-dependent retained sections, not an
//! unchamfered prism. Proof/entry/accuracy scope: docs/chamfer.md.
use crate::{
    contour::Boundaries,
    motion::{Curve, Event, Motion, MoveKind, Vec3, WorkFrame},
    planar::{Boundary, Component, ConstructionOptions, PlanarRegion},
    pocket::{ResidualStatus, Stroke},
    profiles::{self, ProfileDepths, ProfileMotion, ProfileSettings},
    stock::{CancelSignal, Stock, StockSpec},
    tool::{CutterShape, ToolAssembly, valid_id},
    verification,
};
use serde::{Deserialize, Serialize};
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ChamferOperation {
    pub id: String,
    pub tool: String,
    pub region: PlanarRegion,
    pub top_z_mm: f64,
    pub width_mm: f64,
    pub clearance_z_mm: f64,
    pub boundaries: Boundaries,
    pub tip_clearance_mm: f64,
    pub width_step_mm: Option<f64>,
    pub boundary_tolerance_mm: f64,
    pub tolerance_mm: f64,
    pub reverse: bool,
    pub lead_in_mm: f64,
    pub lead_out_mm: f64,
    pub feed_mm_min: f64,
    pub plunge_mm_min: f64,
    pub spindle_rpm: f64,
    pub work_offset: u8,
    pub tool_number: u32,
    pub length_register: u32,
    pub coolant: crate::motion::Coolant,
    /// Rapid travel height between passes (absolute Z). Default: clearance.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub retract_z_mm: Option<f64>,
    /// Rapid descent stops here and plunge feed begins; leaving a pass, feed
    /// changes to rapid here (absolute Z, above the stock top). Default: retract.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub feed_z_mm: Option<f64>,
    /// Containment and avoid areas (clamps, fixtures, features to leave alone).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub limits: Option<crate::limits::Limits>,
}
#[derive(Debug, Serialize)]
pub struct ChamferReport {
    pub strategy: &'static str,
    pub geometric_residual_status: ResidualStatus,
    pub requested_width_mm: f64,
    pub axial_bevel_depth_mm: f64,
    pub tool_tip_target_z_mm: f64,
    pub top_z_mm: f64,
    pub width_passes: usize,
    pub loops: usize,
    pub xy_edges: usize,
    pub retained_analytic_arcs: usize,
    pub top_target_area_mm2: f64,
    pub reference_axis_radius_mm: f64,
    pub applied_axis_offset_mm: f64,
    pub source_error_mm: f64,
    pub path_safety_margin_mm: f64,
    pub preflight_nodes: u64,
    pub preflight_boundary_tests: u64,
    pub entry_requires_cleared_tip: bool,
    pub entry_checked_from_generation: Option<u64>,
    pub entry_query_work: usize,
    pub cut_edges_emitted: usize,
    pub loops_completed: usize,
    pub target_proof: &'static str,
    pub edge_completion: &'static str,
}
pub struct ChamferPlan {
    op: ChamferOperation,
    motion: ProfileMotion,
    entries: Vec<Vec3>,
    tip_radius: f64,
    entries_checked: bool,
    report: ChamferReport,
}
fn region(source: &PlanarRegion, components: Vec<Component>) -> PlanarRegion {
    PlanarRegion {
        version: 1,
        units: "mm".into(),
        name: source.name.clone(),
        boundary_error_mm: source.boundary_error_mm,
        source: serde_json::Value::Null,
        components,
    }
}
fn single_outer(source: &PlanarRegion, b: &Boundary) -> PlanarRegion {
    region(
        source,
        vec![Component {
            outer: b.clone(),
            holes: vec![],
        }],
    )
}
fn inset_boundary(
    source: &PlanarRegion,
    b: &Boundary,
    distance: f64,
    options: ConstructionOptions,
) -> Result<Boundary, String> {
    let result = single_outer(source, b).prepare(options)?.offset(distance)?;
    if result.region.components.len() != 1 || !result.region.components[0].holes.is_empty() {
        return Err("chamfer width erases/splits a selected top boundary".into());
    }
    Ok(result.region.components[0].outer.clone())
}
fn target_top(
    source: &PlanarRegion,
    width: f64,
    selection: Boundaries,
    options: ConstructionOptions,
) -> Result<PlanarRegion, String> {
    let mut components = Vec::new();
    for c in &source.components {
        let outer = if selection != Boundaries::Holes {
            inset_boundary(source, &c.outer, width, options)?
        } else {
            c.outer.clone()
        };
        let holes = c
            .holes
            .iter()
            .map(|h| {
                if selection != Boundaries::Outer {
                    inset_boundary(source, h, -width, options)
                } else {
                    Ok(h.clone())
                }
            })
            .collect::<Result<Vec<_>, String>>()?;
        components.push(Component { outer, holes });
    }
    let mut target = region(source, components);
    target.boundary_error_mm += options.tolerance_mm;
    Ok(target.prepare(options)?.definition().clone())
}
fn scene(
    footprint: PlanarRegion,
    id: &str,
    spec: &StockSpec,
    top: f64,
    tol: f64,
    resolution: f64,
) -> Result<verification::PreparedScene, String> {
    verification::Scene {
        version: 1,
        units: "mm".into(),
        id: id.into(),
        revision: "chamfer-target-v1".into(),
        work_to_machine: WorkFrame::identity(),
        machine: None,
        bodies: vec![verification::Prism {
            id: id.into(),
            role: verification::BodyRole::ProtectedTarget,
            representation: verification::Representation::SolidPrism,
            footprint,
            z_min_mm: spec.min_mm[2],
            z_max_mm: top,
            z_error_mm: 0.,
        }],
    }
    .prepare(verification::Options {
        tolerance_mm: tol,
        spatial_resolution_mm: resolution,
        ..Default::default()
    })
}
impl ChamferPlan {
    pub fn prepare(
        op: ChamferOperation,
        assembly: &ToolAssembly,
        spec: &StockSpec,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        if cancel.is_cancelled() {
            return Err("chamfer planning cancelled".into());
        }
        spec.grid()?;
        let actual = assembly.prepare()?;
        let cutter = actual.cutter().definition();
        let CutterShape::Chamfer {
            tip_diameter_mm, ..
        } = cutter.shape
        else {
            return Err("chamfer operation needs a conical chamfer cutter".into());
        };
        let width_step = op.width_step_mm.unwrap_or(op.width_mm);
        let margin = op.boundary_tolerance_mm / 2.;
        let d = tip_diameter_mm / 2. + op.tip_clearance_mm;
        let reference_height = actual
            .cutter()
            .tip_height_at_radius(d)
            .ok_or("chamfer tip clearance exceeds cone radius")?;
        let top_height = actual
            .cutter()
            .tip_height_at_radius(d + op.width_mm)
            .ok_or("chamfer width exceeds conical cutter radius")?;
        let depth = top_height - reference_height;
        let tip = op.top_z_mm - top_height;
        let values = [
            op.top_z_mm,
            op.width_mm,
            op.tip_clearance_mm,
            width_step,
            op.boundary_tolerance_mm,
            op.tolerance_mm,
            op.clearance_z_mm,
            op.feed_mm_min,
            op.plunge_mm_min,
            op.spindle_rpm,
            op.lead_in_mm,
            op.lead_out_mm,
            depth,
            tip,
        ];
        let guard = 16. * op.tolerance_mm;
        crate::heights::require(
            "chamfer",
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
                    op.width_mm > margin + guard,
                    format!(
                        "width_mm {} must exceed the boundary margin {margin}",
                        op.width_mm
                    ),
                ),
                (
                    width_step > guard,
                    format!("width_step_mm {width_step} must be positive"),
                ),
                (
                    op.tip_clearance_mm > guard,
                    format!("tip_clearance_mm {} must be positive", op.tip_clearance_mm),
                ),
                (
                    op.top_z_mm <= spec.max_mm[2],
                    format!(
                        "top_z_mm {} must not be above the stock top {}",
                        op.top_z_mm, spec.max_mm[2]
                    ),
                ),
                (
                    op.top_z_mm - depth > spec.min_mm[2] && tip >= spec.min_mm[2],
                    format!(
                        "chamfer depth {depth} (tool tip at Z{tip}) reaches below the stock bottom {}",
                        spec.min_mm[2]
                    ),
                ),
                (
                    spec.max_mm[2] - tip <= cutter.cutting_length_mm - guard,
                    crate::reach::ReachRefusal::cutting_length(
                        &op.id,
                        &op.tool,
                        spec.max_mm[2],
                        tip,
                        cutter.cutting_length_mm,
                    )
                    .error(format_args!(
                        "tool tip Z{tip} is deeper below the stock top than the cutting length {}",
                        cutter.cutting_length_mm
                    )),
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
                    (1..=9).contains(&op.work_offset),
                    format!("work_offset {} must be 1..9", op.work_offset),
                ),
                (
                    op.tool_number > 0 && op.length_register > 0,
                    "tool_number and length_register must be positive".into(),
                ),
            ],
        )?;
        let options = ConstructionOptions {
            tolerance_mm: op.tolerance_mm,
        };
        let source = op.region.prepare(options)?;
        let error = source.summary().boundary_error_mm;
        if 2. * error + margin + 16. * op.tolerance_mm > op.boundary_tolerance_mm
            || op.width_mm <= margin + 2. * error + 16. * op.tolerance_mm
        {
            return Err("chamfer source uncertainty exceeds width/accuracy budget".into());
        }
        let top_target = target_top(source.definition(), op.width_mm, op.boundaries, options)?;
        let top_area = top_target.prepare(options)?.summary().area_mm2;
        let applied = d + error + margin;
        let prepared = profiles::offset_profiles(
            &source,
            applied,
            op.boundaries,
            op.reverse,
            options,
            cancel,
        )?;
        let passes = (op.width_mm / width_step).ceil() as usize;
        if passes == 0 || first_id == 0 || first_id.checked_add(1).is_none() {
            return Err("chamfer exceeds bounded width passes/motions/IDs".into());
        }
        let mut proxy = assembly.clone();
        proxy.cutter.shape = CutterShape::Flat;
        proxy.cutter.diameter_mm = 2. * d;
        proxy.cutter.neck = None;
        let proxy = proxy.prepare()?;
        let mut min = [spec.min_mm[0], spec.min_mm[1]];
        let mut max = [spec.max_mm[0], spec.max_mm[1]];
        for c in &source.definition().components {
            for b in std::iter::once(&c.outer).chain(&c.holes) {
                for s in Stroke::from_boundary(b) {
                    let bounds = s.curve(op.top_z_mm).prepare(op.tolerance_mm)?.bounds();
                    for i in 0..2 {
                        min[i] = min[i].min(bounds.min[i]);
                        max[i] = max[i].max(bounds.max[i]);
                    }
                }
            }
        }
        let pad = cutter.diameter_mm + op.width_mm + applied + 1.;
        let bbox = Boundary {
            vertices: vec![
                [min[0] - pad, min[1] - pad, 0.],
                [max[0] + pad, min[1] - pad, 0.],
                [max[0] + pad, max[1] + pad, 0.],
                [min[0] - pad, max[1] + pad, 0.],
            ],
        };
        let resolution = (op.boundary_tolerance_mm / 16.).max(op.tolerance_mm);
        let mut nodes = 0;
        let mut tests = 0;
        let mut entries = Vec::new();
        // The neighbor parts, checked part by part (each loop skips its own).
        let neighbors = verification::ComponentScenes::new(
            "neighbor-parts",
            "chamfer-target-v1",
            "neighbor-parts",
            source.definition(),
            [spec.min_mm[2], op.top_z_mm],
            verification::Options {
                tolerance_mm: op.tolerance_mm,
                spatial_resolution_mm: resolution,
                ..Default::default()
            },
        )?;
        for ring in &prepared.loops {
            let own = &source.definition().components[ring.component];
            let reference = if let Some(h) = ring.hole {
                region(
                    source.definition(),
                    vec![Component {
                        outer: bbox.clone(),
                        holes: vec![own.holes[h].clone()],
                    }],
                )
            } else {
                single_outer(source.definition(), &own.outer)
            };
            let tube_scene = scene(
                reference,
                "axis-reference",
                spec,
                op.top_z_mm,
                op.tolerance_mm,
                resolution,
            )?;
            let first = ring.strokes[0].curve(tip).start();
            let finish = ring.strokes.last().unwrap().curve(tip).end();
            let entry = profiles::lead_point(&ring.strokes, tip, op.lead_in_mm, false);
            entries.push(entry);
            let curves = ring.strokes.iter().map(|s| s.curve(tip)).chain([
                Curve::Line {
                    start: entry,
                    end: first,
                },
                Curve::Line {
                    start: finish,
                    end: profiles::lead_point(&ring.strokes, tip, op.lead_out_mm, true),
                },
            ]);
            for curve in curves {
                if matches!(curve,Curve::Line {start,end}if start==end) {
                    continue;
                }
                let motion = Motion {
                    id: first_id,
                    operation: op.id.clone(),
                    tool: op.tool.clone(),
                    kind: MoveKind::Cut,
                    curve,
                    feed_mm_min: Some(op.feed_mm_min),
                };
                let report = tube_scene.verify_motion(&proxy, &motion, cancel)?;
                nodes += report.stats.nodes;
                tests += report.stats.boundary_tests;
                if report.status != verification::Status::Clear {
                    return Err(format!(
                        "chamfer beveled-target admission {:?}; axis tube/neighbor part is not proven clear",
                        report.status
                    ));
                }
                let checked = neighbors.verify_motion_except(
                    &actual,
                    &motion,
                    Some(ring.component),
                    cancel,
                )?;
                nodes += checked.nodes;
                tests += checked.boundary_tests;
                if let Some(report) = checked.unclear {
                    return Err(format!(
                        "chamfer beveled-target admission {:?}; axis tube/neighbor part is not proven clear",
                        report.status
                    ));
                }
            }
        }
        let report = ChamferReport {
            strategy: "chamfer",
            geometric_residual_status: ResidualStatus::Pending,
            requested_width_mm: op.width_mm,
            axial_bevel_depth_mm: depth,
            tool_tip_target_z_mm: tip,
            top_z_mm: op.top_z_mm,
            width_passes: passes,
            loops: prepared.loops.len(),
            xy_edges: prepared.edges,
            retained_analytic_arcs: prepared.arcs,
            top_target_area_mm2: top_area,
            reference_axis_radius_mm: d,
            applied_axis_offset_mm: applied,
            source_error_mm: error,
            path_safety_margin_mm: margin,
            preflight_nodes: nodes,
            preflight_boundary_tests: tests,
            entry_requires_cleared_tip: !cutter.center_cutting,
            entry_checked_from_generation: None,
            entry_query_work: 0,
            cut_edges_emitted: 0,
            loops_completed: 0,
            target_proof: "continuous flat proxy axis tube + analytic cone/eroded-reference sections; real assembly checks other original components",
            edge_completion: "not_certified; concave/inaccessible corner residual requires independent material checks",
        };
        let heights = crate::heights::Heights::resolve(
            op.clearance_z_mm,
            op.retract_z_mm,
            op.feed_z_mm,
            spec.max_mm[2].max(op.top_z_mm),
            16. * op.tolerance_mm,
        )?;
        let settings = ProfileSettings {
            id: op.id.clone(),
            tool: op.tool.clone(),
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
        let depths = ProfileDepths::Chamfer {
            top: op.top_z_mm,
            reference_height,
            height_per_width: depth / op.width_mm,
            width: op.width_mm,
            step: width_step,
            final_tip: tip,
        };
        let mut motion = ProfileMotion::new(
            settings,
            spec,
            initial,
            prepared.loops.into_iter().map(|r| r.strokes).collect(),
            depths,
            passes,
            first_id,
        );
        if let Some(limits) = op.limits.as_ref().filter(|l| !l.is_empty()) {
            motion.limits = Some(crate::limits::PreparedLimits::new(
                limits,
                cutter.diameter_mm / 2.,
                op.tolerance_mm,
            )?);
        }
        Ok(Self {
            op,
            motion,
            entries,
            tip_radius: tip_diameter_mm / 2.,
            entries_checked: false,
            report,
        })
    }
    pub fn report(&self) -> &ChamferReport {
        &self.report
    }
    /// Planning progress (#198): depth passes done of those planned.
    pub fn progress(&self) -> crate::progress::Planned {
        self.motion.progress()
    }
    pub fn prefix_events(&self) -> Vec<Event> {
        self.motion.prefix_events()
    }
    pub fn suffix_events(&self) -> Vec<Event> {
        self.motion.suffix_events()
    }
    pub fn finish_report(&mut self, cancel: &impl CancelSignal) -> Result<(), String> {
        if cancel.is_cancelled() {
            return Err("chamfer report cancelled".into());
        }
        if !self.motion.drained {
            return Err("drain committed chamfer motions".into());
        }
        self.report.geometric_residual_status = ResidualStatus::Unknown;
        Ok(())
    }
    pub fn next_motion_with_stock(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        self.motion.check_stock(stock, cancel)?;
        if !self.entries_checked {
            if self.report.entry_requires_cleared_tip {
                for &entry in &self.entries {
                    let mut end = entry;
                    end[0] += if entry[0] >= 0. {
                        -16. * self.op.tolerance_mm
                    } else {
                        16. * self.op.tolerance_mm
                    };
                    let query = Motion {
                        id: 1,
                        operation: self.op.id.clone(),
                        tool: self.op.tool.clone(),
                        kind: MoveKind::Cut,
                        curve: Curve::Line { start: entry, end },
                        feed_mm_min: Some(self.op.plunge_mm_min),
                    };
                    let mut work = usize::MAX;
                    let before = work;
                    let material = stock.may_remove_material(
                        &query,
                        self.tip_radius.max(16. * self.op.tolerance_mm)
                            + 16. * self.op.tolerance_mm,
                        0.,
                        self.op.tolerance_mm,
                        &mut work,
                        cancel,
                    )?;
                    self.report.entry_query_work += before - work;
                    if material {
                        return Err("non-center-cutting chamfer needs a proven cleared tip entry; stock is material/uncertain or query budget exhausted".into());
                    }
                }
            }
            self.entries_checked = true;
            self.report.entry_checked_from_generation = Some(stock.generation());
        }
        let m = self.motion.next(stock, cancel)?;
        self.report.cut_edges_emitted = self.motion.cuts;
        self.report.loops_completed = self.motion.closed_loops;
        Ok(m)
    }
}
