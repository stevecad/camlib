//! Thermal marking (plasma, laser): lines traced on the sheet surface with
//! the beam in the machine's marking process, without cutting through
//! (part numbers, bend lines, layout marks).
//!
//! The beam centre follows the given paths (lines and arcs, as `Trace`)
//! with no kerf compensation, at the marking height, feed and power the
//! caller declares for the machine and material: camlib assumes none of
//! them. Each path is one beam-on: travel to its start, the marking beam on
//! (`Event::BeamMark`), the path as `MoveKind::Mark` moves, the beam off.
//! Paths run in the order given.
//!
//! What sets a mark's depth and width is the machine's: "You can vary the
//! depth and width of marks by varying torch speed, output current, and
//! torch-to-workpiece height" (Hypertherm Powermax45 SYNC Operator Manual,
//! "Marking process guidelines", which marks at 9 A to 19 A). A plasma
//! controller sets the marking current in its own process or material
//! table; a laser's is its power word.
//!
//! Marking runs before cutting, while each part is still held by the
//! sheet and has not moved (issue #18; Hypertherm ProNest sequences
//! scribing with "a simple checkbox controlling whether scribing is
//! processed first", its "Punch/Scribe first" setting: "Scribing with
//! ProNest", hypertherm.com). So:
//!
//! * a mark's whole width lies inside one part of `parts`, proven with the
//!   limits' containment proof (`limits::PreparedLimits`), unless
//!   `allow_off_part` asks for marks on the scrap (remnant labels); the
//!   report says which;
//! * a part with a mark must not have been cut yet: read from the stock,
//!   no cell at or inside its outline may be (possibly) cut through;
//! * a marking move removes no stock, and the stock model refuses one over
//!   sheet that is (possibly) cut through or beyond the sheet
//!   (`Stock::preview_sweep`), in the planned run and in the replay of the
//!   posted program alike.
use crate::{
    limits::{Limits, PreparedLimits},
    motion::{Curve, Event, Motion, MoveKind, Vec3},
    planar::{ConstructionOptions, PlanarRegion, PreparedRegion},
    pocket::ResidualStatus,
    stock::{CancelSignal, Stock, StockSpec},
    thermal::HeightMode,
    tool::{CutterShape, ToolAssembly, valid_id},
    trace::TracePath,
};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ThermalMarkOperation {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub limits: Option<crate::limits::Limits>,
    pub id: String,
    /// A beam tool (the torch or laser head that also cuts).
    pub tool: String,
    /// The parts of the sheet (each an outer boundary with holes), as given
    /// to the cut: every mark lies inside one of them.
    pub parts: PlanarRegion,
    /// The lines to mark: the beam centre follows them.
    pub paths: Vec<TracePath>,
    pub feed_mm_min: f64,
    /// Nozzle (or focus) height above the sheet top while marking.
    pub mark_height_mm: f64,
    #[serde(default)]
    pub height_mode: HeightMode,
    /// Program height mode: the height above the sheet top for travel
    /// between paths.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub travel_height_mm: Option<f64>,
    /// Marking power (laser power, percent; a plasma torch marks at the
    /// current its controller's marking process sets).
    pub power_percent: f64,
    /// The width of the marked line on this material, declared by the
    /// caller: that width must stay inside the part.
    pub mark_width_mm: f64,
    /// Smallest distance kept between a mark's edge and its part's edges.
    #[serde(default)]
    pub edge_clearance_mm: f64,
    /// Marks may lie on the scrap or cross parts (remnant labels, layout
    /// lines): the containment proof is skipped, and reported as skipped.
    #[serde(default)]
    pub allow_off_part: bool,
    pub tolerance_mm: f64,
    pub work_offset: u8,
    pub tool_number: u32,
    pub length_register: u32,
}

#[derive(Debug, Serialize)]
pub struct ThermalMarkReport {
    pub strategy: &'static str,
    pub geometric_residual_status: ResidualStatus,
    pub mark_width_mm: f64,
    pub paths: usize,
    pub segments: usize,
    pub closed_paths: usize,
    pub mark_length_mm: f64,
    /// Per path: the part (component of `parts`) its start lies in.
    pub parts: Vec<Option<usize>>,
    /// "proven": every mark's width lies inside its part. "not_checked":
    /// `allow_off_part` was set.
    pub containment: &'static str,
    pub paths_completed: usize,
    pub diagnostics: Vec<String>,
}

enum Step {
    Move(Curve, MoveKind),
    Event(Event),
    /// A path is complete (for the report).
    Done,
}

pub struct ThermalMarkPlan {
    id: String,
    tool: String,
    work_offset: u8,
    tool_number: u32,
    length_register: u32,
    initial: Vec3,
    spec: StockSpec,
    feed: f64,
    steps: VecDeque<Step>,
    events: Vec<Event>,
    next_id: u64,
    expected_generation: Option<u64>,
    drained: bool,
    tolerance: f64,
    report: ThermalMarkReport,
}

/// One part of `parts` as its own region.
fn part_region(
    definition: &PlanarRegion,
    index: usize,
    options: ConstructionOptions,
) -> Result<PreparedRegion, String> {
    PlanarRegion {
        version: 1,
        units: "mm".into(),
        source: serde_json::Value::Null,
        name: format!("part-{index}"),
        components: vec![definition.components[index].clone()],
        boundary_error_mm: definition.boundary_error_mm,
    }
    .prepare(options)
}

/// The largest distance a cut keeps between a part's outline and the near
/// edge of its kerf: half of `ThermalCutOperation::boundary_tolerance_mm`,
/// which is at most 0.1 mm.
const MAX_CUT_MARGIN_MM: f64 = 0.05;

/// Refuse when `part` has been cut: a stock cell that is (possibly) cut
/// through and lies inside the part or within `reach` outside it. A kerf
/// along the outline (or a hole) of the part always leaves such a cell,
/// whatever its width and however fine the grid: the kerf's near edge lies
/// at most `MAX_CUT_MARGIN_MM` (and the boundary error) outside the
/// outline, every cell whose footprint meets the kerf is marked possibly
/// cut, and `reach` adds half a cell diagonal to that distance, so the
/// band of outward distances it covers is wider than the spacing of the
/// cell centres.
fn part_uncut(
    stock: &Stock,
    region: &PreparedRegion,
    index: usize,
    id: &str,
    cancel: &impl CancelSignal,
) -> Result<(), String> {
    let spec = stock.spec();
    let (nx, ny, _) = spec.grid()?;
    let step = [
        (spec.max_mm[0] - spec.min_mm[0]) / nx as f64,
        (spec.max_mm[1] - spec.min_mm[1]) / ny as f64,
    ];
    let cell_radius = step[0].hypot(step[1]) / 2.;
    let Some(bounds) = region.bounds() else {
        return Ok(());
    };
    let reach = cell_radius
        + MAX_CUT_MARGIN_MM
        + region.definition().boundary_error_mm
        + region.summary().numerical_guard_mm;
    let range = |k: usize, n: usize| {
        let lo = ((bounds[k] - reach - spec.min_mm[k]) / step[k]).floor();
        let hi = ((bounds[k + 2] + reach - spec.min_mm[k]) / step[k]).floor();
        (lo.max(0.) as usize, hi.min(n as f64 - 1.))
    };
    let ((x0, x1), (y0, y1)) = (range(0, nx), range(1, ny));
    if x1 < 0. || y1 < 0. {
        return Ok(());
    }
    let (x1, y1) = (x1 as usize, y1 as usize);
    let cells = stock.cells();
    let bottom = spec.min_mm[2];
    for j in y0..=y1 {
        if cancel.is_cancelled() {
            return Err("thermal mark planning cancelled".into());
        }
        for i in x0..=x1 {
            let cell = j * nx + i;
            if cells[cell].lower_mm > bottom {
                continue;
            }
            let p = stock.cell_center(cell).ok_or("stock cell out of range")?;
            let clearance = region
                .query(p)?
                .signed_clearance_mm
                .ok_or("thermal mark: empty part")?;
            if clearance >= -reach {
                return Err(format!(
                    "thermal mark {id}: part {index} is already cut (the sheet is cut through near ({:.3}, {:.3})); marking runs before cutting, while the part is held by the sheet: put this operation before the cut",
                    p[0], p[1]
                ));
            }
        }
    }
    Ok(())
}

impl ThermalMarkPlan {
    pub fn prepare_with_stock(
        op: ThermalMarkOperation,
        assembly: &ToolAssembly,
        stock: &Stock,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        if cancel.is_cancelled() {
            return Err("thermal mark planning cancelled".into());
        }
        let spec = stock.spec();
        spec.grid()?;
        let tool = assembly.prepare()?;
        let cutter = tool.cutter().definition();
        let top = spec.max_mm[2];
        let guard = 16. * op.tolerance_mm;
        let program = op.height_mode == HeightMode::Program;
        let travel_h = op.travel_height_mm.unwrap_or(op.mark_height_mm);
        crate::heights::require(
            "thermal mark",
            [
                (valid_id(&op.id), format!("id {:?} is not a valid identifier", op.id)),
                (
                    op.tool == assembly.id,
                    format!("tool {:?} does not match the assembly {:?}", op.tool, assembly.id),
                ),
                (
                    matches!(cutter.shape, CutterShape::Beam),
                    "thermal marking needs a beam tool (camlib.tools.Torch or LaserHead)".into(),
                ),
                (
                    (1e-7..=0.01).contains(&op.tolerance_mm),
                    "tolerance_mm must be within 1e-7..0.01".into(),
                ),
                (
                    op.feed_mm_min.is_finite() && op.feed_mm_min > 0.,
                    format!("feed_mm_min {} must be positive", op.feed_mm_min),
                ),
                (
                    op.mark_height_mm.is_finite() && op.mark_height_mm > guard && op.mark_height_mm <= 1000.,
                    format!("mark_height_mm {} must be positive (the nozzle stays above the sheet)", op.mark_height_mm),
                ),
                (
                    program || op.travel_height_mm.is_none(),
                    "travel_height_mm needs height_mode=\"program\" (the controller owns Z in fixed mode)".into(),
                ),
                (
                    !program
                        || (op.travel_height_mm.is_some()
                            && travel_h.is_finite()
                            && op.mark_height_mm <= travel_h
                            && travel_h <= 1000.),
                    "height_mode=\"program\" needs travel_height_mm, at least mark_height_mm".into(),
                ),
                (
                    op.power_percent.is_finite() && op.power_percent > 0. && op.power_percent <= 100.,
                    format!("power_percent {} must be in (0, 100]", op.power_percent),
                ),
                (
                    op.mark_width_mm.is_finite() && op.mark_width_mm > 0. && op.mark_width_mm <= 100.,
                    format!("mark_width_mm {} must be the positive width of the marked line", op.mark_width_mm),
                ),
                (
                    op.edge_clearance_mm.is_finite() && (0. ..=1000.).contains(&op.edge_clearance_mm),
                    "edge_clearance_mm must not be negative".into(),
                ),
                (!op.paths.is_empty(), "paths: give at least one path to mark".into()),
                (
                    op.allow_off_part || !op.parts.components.is_empty(),
                    "parts: give the parts the marks belong to (or allow_off_part for marks on the scrap)".into(),
                ),
                ((1..=9).contains(&op.work_offset), format!("work_offset {} must be 1..9", op.work_offset)),
                (
                    op.tool_number > 0 && op.length_register > 0,
                    "tool_number and length_register must be positive".into(),
                ),
                (
                    initial.iter().all(|n| n.is_finite() && n.abs() <= 1e6) && first_id > 0,
                    "start position must be finite".into(),
                ),
            ],
        )?;
        let z_mark = top + op.mark_height_mm;
        let z_travel = if program { top + travel_h } else { z_mark };
        let options = ConstructionOptions {
            tolerance_mm: op.tolerance_mm,
        };
        // The parts, each as its own region (validated together first, so
        // overlapping or malformed parts are refused as the cut refuses them).
        let parts = op.parts.components.len();
        let definition = if parts == 0 {
            op.parts.clone()
        } else {
            op.parts.prepare(options)?.definition().clone()
        };
        let regions: Vec<PreparedRegion> = (0..parts)
            .map(|index| part_region(&definition, index, options))
            .collect::<Result<_, _>>()?;
        let need = op.mark_width_mm / 2. + op.edge_clearance_mm;
        let mut steps = VecDeque::new();
        let mut pos = initial;
        let push_move = |steps: &mut VecDeque<Step>, end: Vec3, pos: &mut Vec3| {
            if *pos != end {
                steps.push_back(Step::Move(
                    Curve::Line { start: *pos, end },
                    MoveKind::Rapid,
                ));
                *pos = end;
            }
        };
        if !program && pos[2] != z_mark {
            push_move(&mut steps, [pos[0], pos[1], z_mark], &mut pos);
        }
        let (mut segments, mut closed_paths, mut length) = (0, 0, 0.);
        let mut owners = vec![];
        let mut checked: Vec<usize> = vec![];
        for (k, path) in op.paths.iter().enumerate() {
            if cancel.is_cancelled() {
                return Err("thermal mark planning cancelled".into());
            }
            let (strokes, closed) = crate::trace::strokes(path, guard)
                .map_err(|e| format!("thermal mark {}: path {k}: {e}", op.id))?;
            segments += strokes.len();
            closed_paths += usize::from(closed);
            // The part this path starts in.
            let mut owner = None;
            for (index, region) in regions.iter().enumerate() {
                let clearance = region
                    .query(path.start_mm)?
                    .signed_clearance_mm
                    .ok_or("thermal mark: empty part")?;
                if clearance > 0. {
                    owner = Some(index);
                    break;
                }
            }
            if !op.allow_off_part {
                let Some(index) = owner else {
                    return Err(format!(
                        "thermal mark {}: path {k} starts at ({:.3}, {:.3}), outside every part (on the scrap); move it onto its part, or set allow_off_part to mark the scrap",
                        op.id, path.start_mm[0], path.start_mm[1]
                    ));
                };
                // Proof: the mark's width (and the edge clearance) stays
                // inside the part along every move.
                let inside = PreparedLimits::new(
                    &Limits {
                        contain: Some(regions[index].definition().clone()),
                        avoid: vec![],
                    },
                    need,
                    op.tolerance_mm,
                )?;
                for (n, stroke) in strokes.iter().enumerate() {
                    if !inside.allows_curve(&stroke.curve(z_mark), true)? {
                        return Err(format!(
                            "thermal mark {}: path {k} segment {n} is not proven inside part {index} with its mark width {} mm and edge clearance {} mm (it reaches the part's edge, a hole or the scrap); move the mark, or set allow_off_part to mark outside the part",
                            op.id, op.mark_width_mm, op.edge_clearance_mm
                        ));
                    }
                }
            }
            if let Some(index) = owner
                && !checked.contains(&index)
            {
                checked.push(index);
            }
            owners.push(owner);
            let start = [path.start_mm[0], path.start_mm[1]];
            if program {
                push_move(&mut steps, [pos[0], pos[1], z_travel.max(pos[2])], &mut pos);
                push_move(&mut steps, [start[0], start[1], pos[2]], &mut pos);
            }
            push_move(&mut steps, [start[0], start[1], z_mark], &mut pos);
            steps.push_back(Step::Event(Event::BeamMark {
                power_percent: op.power_percent,
            }));
            for stroke in &strokes {
                let curve = stroke.curve(z_mark);
                length += curve.prepare(op.tolerance_mm)?.length_mm();
                pos = curve.end();
                steps.push_back(Step::Move(curve, MoveKind::Mark));
            }
            steps.push_back(Step::Event(Event::Beam { power_percent: 0. }));
            steps.push_back(Step::Done);
            if program {
                push_move(&mut steps, [pos[0], pos[1], z_travel], &mut pos);
            }
        }
        // With marks allowed off their parts, a path may cross any part.
        if op.allow_off_part {
            checked = (0..parts).collect();
        }
        // Order: no marked part has been cut (read from the stock).
        for &index in &checked {
            part_uncut(stock, &regions[index], index, &op.id, cancel)?;
        }
        // Machining limits, as the cut honours them: travel stays out of
        // the avoid areas, and a mark (the beam meets the sheet top) stays
        // out of them and inside the containment.
        if let Some(limits) = op.limits.as_ref().filter(|l| !l.is_empty()) {
            let prepared = PreparedLimits::new(limits, op.mark_width_mm / 2., op.tolerance_mm)?;
            for step in &steps {
                let Step::Move(curve, kind) = step else {
                    continue;
                };
                let marking = *kind == MoveKind::Mark;
                let lower = |mut p: Vec3| {
                    p[2] = top;
                    p
                };
                let checked = match curve {
                    _ if !marking => curve.clone(),
                    Curve::Line { start, end } => Curve::Line {
                        start: lower(*start),
                        end: lower(*end),
                    },
                    Curve::Arc {
                        start,
                        end,
                        center,
                        plane,
                        sweep_rad,
                    } => Curve::Arc {
                        start: lower(*start),
                        end: lower(*end),
                        center: lower(*center),
                        plane: *plane,
                        sweep_rad: *sweep_rad,
                    },
                    Curve::Helix { .. } => {
                        return Err("thermal mark limits: unexpected helical path".into());
                    }
                };
                // A vertical rapid is one point in XY.
                let allowed = if matches!(checked, Curve::Line { start, end } if start[0] == end[0] && start[1] == end[1])
                {
                    let (a, b) = (checked.start(), checked.end());
                    prepared.margin([a[0], a[1]], a[2].min(b[2]), marking)? >= 0.
                } else {
                    prepared.allows_curve(&checked, marking)?
                };
                if !allowed {
                    return Err(format!(
                        "thermal mark {}: a {kind:?} move reaches an avoid area or leaves containment; change the limits or the marks",
                        op.id
                    ));
                }
            }
        }
        let mut events = vec![];
        while let Some(Step::Event(_)) = steps.front() {
            if let Some(Step::Event(e)) = steps.pop_front() {
                events.push(e);
            }
        }
        let report = ThermalMarkReport {
            strategy: "thermal_mark",
            geometric_residual_status: ResidualStatus::Pending,
            mark_width_mm: op.mark_width_mm,
            paths: op.paths.len(),
            segments,
            closed_paths,
            mark_length_mm: length,
            parts: owners,
            containment: if op.allow_off_part {
                "not_checked"
            } else {
                "proven"
            },
            paths_completed: 0,
            diagnostics: vec![],
        };
        Ok(Self {
            id: op.id,
            tool: op.tool,
            work_offset: op.work_offset,
            tool_number: op.tool_number,
            length_register: op.length_register,
            initial,
            spec: spec.clone(),
            feed: op.feed_mm_min,
            steps,
            events,
            next_id: first_id,
            expected_generation: None,
            drained: false,
            tolerance: op.tolerance_mm,
            report,
        })
    }
    pub fn report(&self) -> &ThermalMarkReport {
        &self.report
    }
    pub fn take_events(&mut self) -> Vec<Event> {
        std::mem::take(&mut self.events)
    }
    pub fn prefix_events(&self) -> Vec<Event> {
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
        ]
    }
    pub fn suffix_events(&self) -> Vec<Event> {
        vec![]
    }
    pub fn finish_report(&mut self, cancel: &impl CancelSignal) -> Result<(), String> {
        if cancel.is_cancelled() {
            return Err("thermal mark report cancelled".into());
        }
        if !self.drained {
            return Err("drain committed thermal mark motions first".into());
        }
        // A mark removes nothing: there is no residual to estimate.
        self.report.geometric_residual_status = ResidualStatus::Estimated;
        Ok(())
    }
    pub fn next_motion_with_stock(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        if stock.spec() != &self.spec
            || self
                .expected_generation
                .is_some_and(|g| g != stock.generation())
        {
            return Err("commit preceding thermal mark motion to matching stock".into());
        }
        if cancel.is_cancelled() {
            return Err("thermal mark cancelled".into());
        }
        while let Some(step) = self.steps.pop_front() {
            match step {
                Step::Event(e) => self.events.push(e),
                Step::Done => self.report.paths_completed += 1,
                Step::Move(curve, kind) => {
                    curve.prepare(self.tolerance)?;
                    let m = Motion {
                        id: self.next_id,
                        operation: self.id.clone(),
                        tool: self.tool.clone(),
                        curve,
                        kind,
                        feed_mm_min: (!kind.is_rapid()).then_some(self.feed),
                    };
                    self.next_id += 1;
                    self.expected_generation = Some(stock.generation() + 1);
                    // Events due right after this move stay queued until the
                    // next call hands them out with take_events.
                    while let Some(Step::Event(_) | Step::Done) = self.steps.front() {
                        match self.steps.pop_front() {
                            Some(Step::Event(e)) => self.events.push(e),
                            Some(Step::Done) => self.report.paths_completed += 1,
                            _ => unreachable!(),
                        }
                    }
                    return Ok(Some(m));
                }
            }
        }
        self.drained = true;
        Ok(None)
    }
}
