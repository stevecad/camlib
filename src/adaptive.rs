//! Region-based, stock-aware disk-front clearing. See docs/adaptive-pocket.md.
use crate::{
    Point,
    adaptive_front::FrontDriver,
    entry::{EntryFitOptions, EntryLocation, fit_entries},
    motion::{Curve, Event, Motion, MoveKind, Plane, Vec3},
    navigation::{GraphOptions, Guide, graph},
    planar::{ConstructionOptions, PlanarRegion},
    pocket::{PocketOperation, PocketPlan, ResidualStatus, Stroke},
    stock::{CancelSignal, Stock, StockSpec},
    tool::ToolAssembly,
};
use serde::{Deserialize, Serialize};
use std::{
    collections::HashMap,
    f64::consts::{PI, TAU},
};

pub use crate::adaptive_front::FrontReport;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AdaptivePathStrategy {
    #[default]
    DiskFront,
    StockFront,
    /// Roughing only: grid-steered constant-engagement front following.
    EngagementFront,
}
impl AdaptivePathStrategy {
    pub(crate) fn is_default(&self) -> bool {
        *self == Self::DiskFront
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CuttingSense {
    Climb,
    Conventional,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(try_from = "serde_json::Value", into = "serde_json::Value")]
pub struct AdaptivePocketOperation {
    pub pocket: PocketOperation,
    pub path_strategy: AdaptivePathStrategy,
    pub cutting_sense: Option<CuttingSense>,
    pub orbit_radius_mm: Option<f64>,
    pub min_orbit_radius_mm: Option<f64>,
    pub local_orbit_tolerance_mm: Option<f64>,
    pub max_engagement_deg: f64,
    pub boundary_tolerance_mm: f64,
    pub clockwise: bool,
    /// engagement_front only: see `RoughingOperation::rapid_descent_clearance_mm`.
    /// The proof is per depth level: a descent into a level below one
    /// already cut is rapid down to this far above what the committed sweeps
    /// prove clear for the whole tool. None: rapid descent stops at `feed_z_mm`.
    pub rapid_descent_clearance_mm: Option<f64>,
    /// engagement_front only: see `RoughingOperation::ramp_clearance_mm`.
    pub ramp_clearance_mm: Option<f64>,
    /// engagement_front only: see `RoughingOperation::link_feed_mm_min`.
    /// None: every stay-down link feeds at `feed_mm_min`.
    pub link_feed_mm_min: Option<f64>,
    /// engagement_front only, with `link_feed_mm_min`: see
    /// `RoughingOperation::link_chip_thinning`.
    pub link_chip_thinning: bool,
}
// Keep the public operation wire flat, like Pocket. Explicitly consume strategy
// fields before Pocket's strict unknown-field validator; serde flatten combined
// with deny_unknown_fields would weaken or break that validation.
impl AdaptivePocketOperation {
    pub(crate) fn front_motion_options(&self) -> crate::roughing::FrontMotionOptions {
        crate::roughing::FrontMotionOptions {
            rapid_descent_clearance_mm: self.rapid_descent_clearance_mm,
            ramp_clearance_mm: self.ramp_clearance_mm,
            link_feed_mm_min: self.link_feed_mm_min,
            link_chip_thinning: self.link_chip_thinning,
        }
    }
    /// The orbit planners' parameters (disk_front, stock_front).
    pub(crate) fn validate_parameters(&self, r: f64) -> Result<(f64, f64, f64), String> {
        if self.path_strategy != AdaptivePathStrategy::EngagementFront {
            self.front_motion_options().validate(
                "adaptive pocket",
                Some("; leave it unset or use path_strategy engagement_front (disk_front and stock_front relink by lifted re-entries)"),
            )?;
        }
        if self.pocket.max_stay_down_mm.is_some() {
            return Err("max_stay_down_mm applies to pocket and rest pocket links; adaptive clearing relinks through its own proven-clear re-entries".into());
        }
        if self.path_strategy == AdaptivePathStrategy::DiskFront && self.cutting_sense.is_some() {
            return Err("cutting_sense requires stock_front; disk_front defines only the orbit direction (clockwise)".into());
        }
        let guard = 16. * self.pocket.tolerance_mm;
        let radius = self.orbit_radius_mm.unwrap_or(r / 2.);
        let a = r + radius;
        let min_radius = self.min_orbit_radius_mm.unwrap_or(radius);
        if let Some(t) = self.local_orbit_tolerance_mm
            && (!t.is_finite() || t <= 8. * guard || t > radius || min_radius >= radius)
        {
            return Err("local orbit tolerance requires variable radii and finite resolution above numerical guard and within max radius".into());
        }
        if !min_radius.is_finite()
            || min_radius <= 4. * guard
            || min_radius > radius
            || !radius.is_finite()
            || radius <= 4. * guard
            || radius > r
            || !self.max_engagement_deg.is_finite()
            || !(1. ..=170.).contains(&self.max_engagement_deg)
            || !self.boundary_tolerance_mm.is_finite()
            || self.boundary_tolerance_mm <= 2. * guard
            || self.boundary_tolerance_mm > r / 4.
            || self.pocket.rest_tolerance_mm.is_some()
        {
            return Err("invalid adaptive orbit/engagement/boundary budget; rest height tolerance is not an engagement proof".into());
        }
        let half = self.max_engagement_deg.to_radians() / 2.;
        let bound =
            (a * a - r * r * half.sin().powi(2)).sqrt() - r * half.cos() - radius - 2. * guard;
        if bound <= guard {
            return Err("adaptive advance below numerical resolution".into());
        }
        Ok((radius, min_radius, bound))
    }
}

impl TryFrom<serde_json::Value> for AdaptivePocketOperation {
    type Error = String;
    fn try_from(value: serde_json::Value) -> Result<Self, String> {
        let serde_json::Value::Object(mut fields) = value else {
            return Err("adaptive operation must be an object".into());
        };
        let path_strategy = serde_json::from_value(
            fields
                .remove("path_strategy")
                .unwrap_or(serde_json::json!("disk_front")),
        )
        .map_err(|e| e.to_string())?;
        let cutting_sense = serde_json::from_value(
            fields
                .remove("cutting_sense")
                .unwrap_or(serde_json::Value::Null),
        )
        .map_err(|e| e.to_string())?;
        let orbit_radius_mm = serde_json::from_value(
            fields
                .remove("orbit_radius_mm")
                .unwrap_or(serde_json::Value::Null),
        )
        .map_err(|e| e.to_string())?;
        let min_orbit_radius_mm = serde_json::from_value(
            fields
                .remove("min_orbit_radius_mm")
                .unwrap_or(serde_json::Value::Null),
        )
        .map_err(|e| e.to_string())?;
        let max_engagement_deg = serde_json::from_value(
            fields
                .remove("max_engagement_deg")
                .ok_or("missing max_engagement_deg")?,
        )
        .map_err(|e| e.to_string())?;
        let local_orbit_tolerance_mm = serde_json::from_value(
            fields
                .remove("local_orbit_tolerance_mm")
                .unwrap_or(serde_json::Value::Null),
        )
        .map_err(|e| e.to_string())?;
        let boundary_tolerance_mm = serde_json::from_value(
            fields
                .remove("boundary_tolerance_mm")
                .ok_or("missing boundary_tolerance_mm")?,
        )
        .map_err(|e| e.to_string())?;
        let clockwise =
            serde_json::from_value(fields.remove("clockwise").ok_or("missing clockwise")?)
                .map_err(|e| e.to_string())?;
        let mut optional = |name: &str| -> Result<Option<f64>, String> {
            serde_json::from_value(fields.remove(name).unwrap_or(serde_json::Value::Null))
                .map_err(|e| format!("{name}: {e}"))
        };
        let rapid_descent_clearance_mm = optional("rapid_descent_clearance_mm")?;
        let ramp_clearance_mm = optional("ramp_clearance_mm")?;
        let link_feed_mm_min = optional("link_feed_mm_min")?;
        let link_chip_thinning = serde_json::from_value(
            fields
                .remove("link_chip_thinning")
                .unwrap_or(serde_json::Value::Bool(false)),
        )
        .map_err(|e| format!("link_chip_thinning: {e}"))?;
        let pocket =
            serde_json::from_value(serde_json::Value::Object(fields)).map_err(|e| e.to_string())?;
        Ok(Self {
            pocket,
            path_strategy,
            cutting_sense,
            orbit_radius_mm,
            min_orbit_radius_mm,
            local_orbit_tolerance_mm,
            max_engagement_deg,
            boundary_tolerance_mm,
            clockwise,
            rapid_descent_clearance_mm,
            ramp_clearance_mm,
            link_feed_mm_min,
            link_chip_thinning,
        })
    }
}
impl From<AdaptivePocketOperation> for serde_json::Value {
    fn from(op: AdaptivePocketOperation) -> Self {
        let mut value = serde_json::to_value(op.pocket).expect("Pocket serialization");
        let fields = value.as_object_mut().expect("Pocket object");
        if !op.path_strategy.is_default() {
            fields.insert(
                "path_strategy".into(),
                serde_json::to_value(op.path_strategy).unwrap(),
            );
        }
        if let Some(sense) = op.cutting_sense {
            fields.insert("cutting_sense".into(), serde_json::to_value(sense).unwrap());
        }
        if let Some(r) = op.orbit_radius_mm {
            fields.insert("orbit_radius_mm".into(), r.into());
        }
        if let Some(r) = op.min_orbit_radius_mm {
            fields.insert("min_orbit_radius_mm".into(), r.into());
        }
        if let Some(t) = op.local_orbit_tolerance_mm {
            fields.insert("local_orbit_tolerance_mm".into(), t.into());
        }
        fields.insert("max_engagement_deg".into(), op.max_engagement_deg.into());
        fields.insert(
            "boundary_tolerance_mm".into(),
            op.boundary_tolerance_mm.into(),
        );
        fields.insert("clockwise".into(), op.clockwise.into());
        // Absent when unset: an operation without them serializes as before.
        for (name, value) in [
            ("rapid_descent_clearance_mm", op.rapid_descent_clearance_mm),
            ("ramp_clearance_mm", op.ramp_clearance_mm),
            ("link_feed_mm_min", op.link_feed_mm_min),
        ] {
            if let Some(v) = value {
                fields.insert(name.into(), v.into());
            }
        }
        if op.link_chip_thinning {
            fields.insert("link_chip_thinning".into(), true.into());
        }
        value
    }
}
#[derive(Serialize)]
pub struct AdaptiveReport {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stock_top: Option<crate::pocket::StockTopReport>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub front: Option<FrontReport>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub empty_cuts: Option<crate::empty_cut::EmptyCutReport>,
    pub stock_edge_entries: usize,
    pub helical_entries: usize,
    pub entry_turns: usize,
    pub entry_geometry_work: usize,
    pub entry_navigation_links: usize,
    pub entry_radii_mm: Vec<f64>,
    pub max_entry_turns_per_descent: usize,
    pub strategy: &'static str,
    /// Areas the cutter fits in that no orbit does
    /// (`crate::entry::UnenteredArea`): an `AdaptivePocket` is refused with
    /// the first; a roughing layer lists them here. Their material is in
    /// the residual region below.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub unentered_areas: Vec<crate::entry::UnenteredArea>,
    pub geometric_residual_status: ResidualStatus,
    pub geometric_residual: Option<PlanarRegion>,
    pub geometric_residual_area_mm2: Option<f64>,
    pub target_z_mm: f64,
    pub depth_passes: usize,
    pub center_components: usize,
    pub navigation_components: usize,
    pub guide_strokes: usize,
    pub navigation_links: usize,
    pub navigation_work: usize,
    pub orbit_radius_mm: f64,
    pub swept_disk_radius_mm: f64,
    pub min_orbit_radius_mm: f64,
    pub min_active_radius_mm: f64,
    pub min_cutting_orbit_radius_mm: Option<f64>,
    pub radius_changes: usize,
    pub radius_growth_orbits: usize,
    pub fine_guide_strokes: usize,
    pub local_radius_splits: usize,
    pub local_radius_work: usize,
    pub locally_sized_guides: usize,
    pub uncertain_local_guides: usize,
    pub distinct_guide_radii: usize,
    pub guide_orbit_radii_mm: Vec<f64>,
    pub guide_orbit_radii_truncated: bool,
    pub omitted_endpoint_orbits: usize,
    pub omitted_seed_orbits: usize,
    pub stock_clear_radius_changes: usize,
    pub committed_tube_advances: usize,
    pub max_engagement_deg: f64,
    pub max_bounded_engagement_deg: f64,
    pub bounded_advance_mm: f64,
    pub boundary_underclearance_budget_mm: f64,
    pub seed_entry_policy: &'static str,
    pub seed_orbits: usize,
    pub controlled_orbits: usize,
    pub bounded_advances: usize,
    pub stock_clear_advances: usize,
    pub turn_arcs: usize,
    pub chord_fallbacks: usize,
    pub stock_query_work: usize,
    pub stock_query_work_exhausted: bool,
    pub motions: usize,
    pub diagnostics: Vec<String>,
}
fn distance(a: Point, b: Point) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}
fn point(s: Stroke, f: f64) -> Point {
    if f == 0. {
        return s.a;
    }
    if f == 1. {
        return s.b;
    }
    if let Some((c, sweep)) = s.arc {
        let a = (s.a[1] - c[1]).atan2(s.a[0] - c[0]) + f * sweep;
        let r = distance(s.a, c);
        [c[0] + r * a.cos(), c[1] + r * a.sin()]
    } else {
        [
            s.a[0] + f * (s.b[0] - s.a[0]),
            s.a[1] + f * (s.b[1] - s.a[1]),
        ]
    }
}
fn tangent(s: Stroke, f: f64) -> Point {
    if let Some((c, sweep)) = s.arc {
        let p = point(s, f);
        let r = distance(p, c);
        let sign = sweep.signum();
        [-(p[1] - c[1]) * sign / r, (p[0] - c[0]) * sign / r]
    } else {
        s.tangent(false)
    }
}
fn length(s: Stroke) -> f64 {
    s.arc
        .map_or_else(|| distance(s.a, s.b), |(c, a)| distance(s.a, c) * a.abs())
}
#[derive(Default)]
struct LocalSizing {
    originals: Vec<(Stroke, f64, f64)>,
    splits: usize,
    work: usize,
    resized: usize,
    uncertain: usize,
}
fn size_guides(
    guides: &PocketPlan,
    originals: Vec<(Stroke, f64, f64)>,
    cutter: f64,
    maximum: f64,
    tolerance: f64,
    cancel: &impl CancelSignal,
) -> Result<LocalSizing, String> {
    let mut out = LocalSizing::default();
    let mut stack: Vec<_> = originals.into_iter().rev().map(|g| (g, 0usize)).collect();
    while let Some(((s, base, coverage), depth)) = stack.pop() {
        if cancel.is_cancelled() {
            return Err("adaptive local sizing cancelled".into());
        }
        out.work += guides.link_work_per_query();
        let Some((lower, upper)) = guides.guide_radius_bounds(s) else {
            out.originals.push((s, base, coverage));
            out.uncertain += 1;
            continue;
        };
        let radius = (lower - cutter).min(maximum).max(base);
        let upper = (upper - cutter).min(maximum).max(radius);
        if upper - radius > tolerance {
            if depth >= 32 || length(s) <= tolerance / 64. {
                return Err("adaptive local sizing subdivision limit exceeded".into());
            }
            let mid = point(s, 0.5);
            let arc = s.arc.map(|(c, a)| (c, a / 2.));
            stack.push((
                (
                    Stroke {
                        a: mid,
                        b: s.b,
                        arc,
                    },
                    base,
                    coverage,
                ),
                depth + 1,
            ));
            stack.push((
                (
                    Stroke {
                        a: s.a,
                        b: mid,
                        arc,
                    },
                    base,
                    coverage,
                ),
                depth + 1,
            ));
            out.splits += 1;
        } else {
            out.resized += usize::from(radius > base + tolerance / 64.);
            out.originals.push((s, radius, coverage));
        }
    }
    Ok(out)
}
struct Advance {
    curve: Curve,
    fraction: f64,
    center: Point,
    normal: Point,
    clear: bool,
    committed_tube: bool,
    angle: f64,
    chord: bool,
}
#[derive(Clone, Copy, PartialEq)]
enum Phase {
    Up,
    Across,
    Plunge,
    EntryTop,
    EntryHelix,
    Seed,
    Turn,
    Advance,
    Orbit,
    RadiusOrbit,
    Front,
    Retract,
    /// Rapid from the feed height to the retract height (clearance at the end).
    Rise,
    /// Rapid from the travel height down to the feed height above an entry.
    Descend,
    Done,
}
pub struct AdaptivePlan {
    heights: crate::heights::Heights,
    /// The entry phase after the descent to the feed height.
    entry: Phase,
    stock_top: Option<crate::pocket::StockTop>,
    front: Option<FrontDriver>,
    op: AdaptivePocketOperation,
    guides: PocketPlan,
    tours: Vec<Vec<Guide>>,
    helical: Option<Vec<EntryLocation>>,
    spec: StockSpec,
    position: Vec3,
    center: Point,
    normal: Point,
    r: f64,
    guard: f64,
    active_radius: f64,
    disk_radius: f64,
    /// (emitted, total) one-turn helices of the current helical entry.
    entry_turn: Option<(usize, usize)>,
    cleared_edges: HashMap<usize, f64>,
    pass: usize,
    component: usize,
    guide: usize,
    fraction: f64,
    phase: Phase,
    advance: Option<Advance>,
    next_id: u64,
    generation: Option<u64>,
    report: AdaptiveReport,
    /// Whether the cutter fits anywhere in the region (its own centre
    /// region, not the orbits').
    cutter_fits: bool,
}
impl AdaptivePlan {
    pub fn prepare_with_stock(
        op: AdaptivePocketOperation,
        tool: &ToolAssembly,
        stock: &Stock,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        let scope = crate::pocket::ReachScope {
            operation: &op.pocket.id,
            tool,
            region: &op.pocket.region,
            lowest_tip_z: op.pocket.floor_z_mm + op.pocket.bottom_allowance_mm,
            clearance_z: op.pocket.clearance_z_mm,
            // Adaptive paths may approach the boundary within this tolerance.
            boundary_slack_mm: op.boundary_tolerance_mm,
        };
        let (envelope, bound) = crate::pocket::StockTop::prepare(
            stock,
            op.pocket.top_z_mm,
            op.pocket.tolerance_mm,
            &scope,
            cancel,
        )?;
        let mut plan = Self::prepare(op, tool, &envelope, initial, first_id, cancel)?;
        if let Some(bound) = bound {
            plan.activate_layer(initial, first_id, stock.spec())?;
            plan.report.stock_top = Some(bound.report.clone());
            plan.stock_top = Some(bound);
            plan.generation = Some(stock.generation());
        }
        Ok(plan)
    }
    pub(crate) fn enable_empty_cuts(&mut self) {
        self.report.empty_cuts = Some(Default::default());
        self.report.diagnostics.push("Omitted non-seed full orbits require a whole empty support disk; no unexecuted in-stock disk is considered cleared".into());
    }
    pub(crate) fn construction_error_mm(&self) -> f64 {
        self.guides.construction_error_mm()
    }
    pub(crate) fn activate_layer(
        &mut self,
        initial: Vec3,
        first_id: u64,
        stock: &StockSpec,
    ) -> Result<(), String> {
        let mut envelope = stock.clone();
        envelope.max_mm[2] = self.spec.max_mm[2];
        if self.report.motions != 0
            || self.generation.is_some()
            || self.phase != Phase::Up
            || first_id == 0
            || initial.iter().any(|x| !x.is_finite() || x.abs() > 1e6)
            || initial[2] < self.op.pocket.clearance_z_mm
            || envelope != self.spec
            || stock.max_mm[2] < self.spec.max_mm[2]
        {
            return Err("cannot activate a used adaptive layer or invalid start".into());
        }
        self.guides.activate_layer(initial, first_id, stock)?;
        self.position = initial;
        self.next_id = first_id;
        self.spec = stock.clone();
        Ok(())
    }
    pub fn prepare(
        op: AdaptivePocketOperation,
        tool: &ToolAssembly,
        spec: &StockSpec,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        if op.path_strategy == AdaptivePathStrategy::EngagementFront {
            return Err("an engagement_front adaptive pocket is planned by PocketFrontPlan (OperationPlan::prepare)".into());
        }
        if let Some(entry) = &op.pocket.entry {
            // disk_front and stock_front plan every pass from a fitted seed
            // circle, which a helix opens; a ramp has no seed circle. The
            // engagement front ramps into enclosed material (#219).
            entry.refuse_ramp(
                &format!("adaptive pocket {}", op.pocket.id),
                "the disk_front and stock_front strategies start every pass from a fitted seed circle that a helix opens; path_strategy engagement_front ramps into enclosed material",
            )?;
        }
        let prepared = tool.prepare()?;
        let r = prepared.cutter().definition().diameter_mm / 2.;
        let (radius, min_radius, bound) = op.validate_parameters(r)?;
        let guard = 16. * op.pocket.tolerance_mm;
        let a = r + radius;
        let coarse = PocketPlan::prepare_guides(
            op.pocket.clone(),
            tool,
            spec,
            initial,
            first_id,
            Some(a),
            cancel,
        )?;
        let variable = min_radius < radius;
        let coarse_coverage = if variable {
            op.boundary_tolerance_mm / 2.
        } else {
            op.boundary_tolerance_mm
        };
        let mut originals: Vec<_> = coarse
            .guide_strokes()
            .iter()
            .map(|s| (*s, radius, coarse_coverage))
            .collect();
        let (guides, fine_count) = if variable {
            let fine = PocketPlan::prepare_guides(
                op.pocket.clone(),
                tool,
                spec,
                initial,
                first_id,
                Some(r + min_radius),
                cancel,
            )?;
            let cover = coarse.guide_cover_region(
                radius - min_radius + op.boundary_tolerance_mm - coarse_coverage - 2. * guard,
            )?;
            let options = ConstructionOptions {
                tolerance_mm: op.pocket.tolerance_mm,
            };
            let cover = cover
                .prepare(options)?
                .offset(cover.boundary_error_mm + 4. * op.pocket.tolerance_mm)?
                .region;
            let retained = fine.guides_outside(&cover, cancel)?;
            let count = retained.len();
            originals.extend(
                retained
                    .into_iter()
                    .map(|s| (s, min_radius, op.boundary_tolerance_mm)),
            );
            (fine, count)
        } else {
            (coarse, 0)
        };
        let sizing = if let Some(t) = op.local_orbit_tolerance_mm {
            size_guides(&guides, originals, r, radius, t, cancel)?
        } else {
            LocalSizing {
                originals,
                ..LocalSizing::default()
            }
        };
        let guide_count = sizing.originals.len();
        let mut graph = graph(
            &guides,
            sizing.originals,
            GraphOptions {
                bridge_radius: min_radius,
                cutter_radius: r,
                coverage: op.boundary_tolerance_mm,
                local_sizing: op.local_orbit_tolerance_mm.is_some(),
                starting_work: 0,
            },
            cancel,
        )?;
        let mut helical = None;
        let mut entry_work = 0;
        let mut entry_links = 0;
        if let Some(entry) = &op.pocket.entry {
            let fit = fit_entries(
                &guides,
                &graph.tours,
                entry.helix().ok_or("missing helical entry")?,
                EntryFitOptions {
                    cutter_radius: r,
                    minimum_orbit: min_radius,
                    preferred_cap: radius,
                    max_depth: op.pocket.top_z_mm - guides.report().target_z_mm,
                    starting_work: graph.work,
                    policy: entry.policy(),
                },
                cancel,
            )?;
            entry_work = fit.work - graph.work;
            let mut edge = graph
                .tours
                .iter()
                .flatten()
                .map(|g| g.edge)
                .max()
                .unwrap_or(0)
                + 1;
            for (tour, location) in graph.tours.iter_mut().zip(&fit.locations) {
                tour.rotate_left(location.attachment);
                let target = tour[0].stroke.a;
                if location.center != target {
                    tour.insert(
                        0,
                        Guide {
                            stroke: Stroke::line(location.center, target),
                            radius: min_radius,
                            coverage: op.boundary_tolerance_mm,
                            edge,
                        },
                    );
                    edge += 1;
                    entry_links += 1;
                }
            }
            helical = Some(fit.locations);
        }
        let mut targets: Vec<_> = graph.tours.iter().flatten().map(|g| g.radius).collect();
        targets.sort_by(f64::total_cmp);
        targets.dedup();
        let distinct_guide_radii = targets.len();
        let guide_orbit_radii_truncated = targets.len() > 64;
        let guide_orbit_radii_mm = if guide_orbit_radii_truncated {
            (0..64)
                .map(|i| targets[i * (targets.len() - 1) / 63])
                .collect()
        } else {
            targets
        };
        // Areas the cutter fits in that no orbit does (#250): components of
        // the cutter's own centre region (as `PocketPlan` constructs it)
        // that hold no guide of any tour.
        let options = ConstructionOptions {
            tolerance_mm: op.pocket.tolerance_mm,
        };
        let cutter = guides
            .source_region()
            .cutter_region(r, op.pocket.radial_allowance_mm + guard)?
            .region;
        let cutter_fits = !cutter.components.is_empty();
        let mut unentered_areas = vec![];
        for component in &cutter.components {
            if cancel.is_cancelled() {
                return Err("adaptive pocket planning cancelled".into());
            }
            let mut one = cutter.clone();
            one.components = vec![component.clone()];
            let one = one.prepare(options)?;
            let mut entered = false;
            for tour in &graph.tours {
                // A tour is connected: one of its points decides.
                let Some(first) = tour.first() else { continue };
                if one
                    .query(first.stroke.a)?
                    .signed_clearance_mm
                    .is_some_and(|d| d >= 0.)
                {
                    entered = true;
                    break;
                }
            }
            if entered {
                continue;
            }
            let Some(at) = component.outer.vertices.first() else {
                continue;
            };
            let (mut low, mut high) = ([at[0], at[1]], [at[0], at[1]]);
            for v in &component.outer.vertices {
                low = [low[0].min(v[0]), low[1].min(v[1])];
                high = [high[0].max(v[0]), high[1].max(v[1])];
            }
            unentered_areas.push(crate::entry::UnenteredArea {
                reason: "no_orbit_fits",
                at_mm: [at[0], at[1]],
                min_mm: low,
                max_mm: high,
                level_z_mm: guides.report().target_z_mm,
                cutter_radius_mm: r,
                radii_tried_mm: if min_radius < radius {
                    vec![radius, min_radius]
                } else {
                    vec![radius]
                },
                needed_mm: min_radius,
                available_mm: None,
            });
        }
        let mut report=AdaptiveReport {
            unentered_areas,
            stock_top:None,
            front:None,
            stock_edge_entries:0,helical_entries:0,entry_turns:0,entry_geometry_work:entry_work,entry_navigation_links:entry_links,
            entry_radii_mm:helical.as_ref().map_or_else(Vec::new,|locations|locations.iter().map(|e|e.radius).collect()),
            max_entry_turns_per_descent:helical.as_ref().and_then(|locations|locations.iter().map(|e|e.max_turns).max()).unwrap_or(0),
            strategy:"adaptive_pocket",geometric_residual_status:ResidualStatus::Pending,
            geometric_residual:None,geometric_residual_area_mm2:None,
            target_z_mm:guides.report().target_z_mm,depth_passes:guides.report().depth_passes,
            center_components:guides.report().center_components,navigation_components:graph.tours.len(),
            guide_strokes:guide_count,navigation_links:graph.links+entry_links,navigation_work:graph.work,
            orbit_radius_mm:radius,swept_disk_radius_mm:a,max_engagement_deg:op.max_engagement_deg,
            min_orbit_radius_mm:min_radius,min_active_radius_mm:radius,min_cutting_orbit_radius_mm:None,
            radius_changes:0,radius_growth_orbits:0,fine_guide_strokes:fine_count,omitted_endpoint_orbits:0,omitted_seed_orbits:0,stock_clear_radius_changes:0,committed_tube_advances:0,
            local_radius_splits:sizing.splits,local_radius_work:sizing.work,locally_sized_guides:sizing.resized,uncertain_local_guides:sizing.uncertain,
            distinct_guide_radii,guide_orbit_radii_mm,guide_orbit_radii_truncated,
            max_bounded_engagement_deg:0.,bounded_advance_mm:bound,
            boundary_underclearance_budget_mm:op.boundary_tolerance_mm,
            seed_entry_policy:if op.pocket.entry.as_ref().is_some_and(|e|e.policy()!=crate::entry::EntryPolicy::Helix) {"whole-source admitted seed disks completely outside original stock permit air descent; any enclosed helix and full floor seed are explicit startup exceptions; subsequent approach/growth/clearing retains bounded engagement"} else if op.pocket.entry.is_some() {"descending helix and first full floor orbit are explicit startup exceptions; all subsequent approach/growth/clearing retains bounded engagement"} else {"plunge and first full orbit per connected guide component/depth are explicit engagement exceptions at plunge feed"},
            seed_orbits:0,controlled_orbits:0,bounded_advances:0,stock_clear_advances:0,turn_arcs:0,chord_fallbacks:0,
            stock_query_work:0,stock_query_work_exhausted:false,motions:0,empty_cuts:None,
            diagnostics:vec!["Orbit sweep radius exceeds actual tool radius; narrow features and corner rest remain explicit nominal residual, actual remaining stock is separately verified".into()],
        };
        let phase = if graph.tours.is_empty() {
            Phase::Done
        } else {
            Phase::Up
        };
        let front =
            (op.path_strategy == AdaptivePathStrategy::StockFront).then(|| FrontDriver::new(tool));
        if let Some(front) = &front {
            report.front = Some(front.report.clone());
            report.diagnostics.push("Development stock_front: committed-sweep coverage and proven-clear re-entry; broad geometry, full dynamics and performance qualification remain open; unresolved steering/coverage stops with an error".into());
        }
        let heights = crate::heights::Heights::resolve(
            op.pocket.clearance_z_mm,
            op.pocket.retract_z_mm,
            op.pocket.feed_z_mm,
            spec.max_mm[2].max(op.pocket.top_z_mm),
            guard,
        )?;
        Ok(Self {
            heights,
            entry: Phase::Plunge,
            stock_top: None,
            front,
            op,
            guides,
            tours: graph.tours,
            helical,
            spec: spec.clone(),
            position: initial,
            center: [0., 0.],
            normal: [0., 0.],
            r,
            guard,
            active_radius: radius,
            disk_radius: 0.,
            entry_turn: None,
            cleared_edges: HashMap::new(),
            pass: 0,
            component: 0,
            guide: 0,
            fraction: 0.,
            phase,
            advance: None,
            next_id: first_id,
            generation: None,
            report,
            cutter_fits,
        })
    }
    pub fn report(&self) -> &AdaptiveReport {
        &self.report
    }
    /// The rule of `crate::entry::UnenteredArea` for an `AdaptivePocket`
    /// planned with orbits (#250): the orbit that opens each area is this
    /// strategy's entry, so an area the cutter fits in and no orbit does
    /// refuses the pocket, as does a region the cutter fits nowhere in.
    /// (A roughing layer lists such areas in its report instead.) Parts of
    /// an area an orbit does fit in that are too narrow for one are that
    /// area's reported residual, as before.
    pub(crate) fn refuse_unentered(&self) -> Result<(), String> {
        let pocket = &self.op.pocket;
        let name = format!("adaptive pocket {}", pocket.id);
        if !self.cutter_fits {
            return Err(crate::pocket::cutter_fits_nowhere(
                &name,
                self.r,
                pocket.radial_allowance_mm,
            ));
        }
        match self.report.unentered_areas.first() {
            Some(area) => Err(area.message(&name)),
            None => Ok(()),
        }
    }
    /// For roughing: why this layer, having made no motion, left material
    /// it was asked to cut (its first area no orbit fits in).
    pub(crate) fn refusal(&self) -> Option<String> {
        if self.report.motions > 0 {
            return None;
        }
        Some(
            self.report
                .unentered_areas
                .first()?
                .message(&self.op.pocket.id),
        )
    }
    /// Planning progress (#198): depth passes done of those planned.
    pub fn progress(&self) -> crate::progress::Planned {
        crate::progress::Planned::Layers {
            done: self.pass,
            total: self.report.depth_passes,
        }
    }
    pub fn prefix_events(&self) -> Vec<Event> {
        self.guides.prefix_events()
    }
    pub fn suffix_events(&self) -> Vec<Event> {
        self.guides.suffix_events()
    }
    pub fn finish_report(&mut self, cancel: &impl CancelSignal) -> Result<(), String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("adaptive.finish");
        if cancel.is_cancelled() {
            return Err("adaptive report cancelled".into());
        }
        if self.phase != Phase::Done {
            return Err("drain adaptive motions before report".into());
        }
        if self.report.geometric_residual_status != ResidualStatus::Pending {
            return Ok(());
        }
        let residual = self.guides.guide_residual(
            self.r + self.report.min_orbit_radius_mm - self.op.boundary_tolerance_mm,
        );
        if cancel.is_cancelled() {
            return Err("adaptive report cancelled".into());
        }
        match residual {
            Ok(residual) => {
                let (residual, prepared) = residual.into_parts();
                self.report.geometric_residual_area_mm2 = Some(prepared.summary().area_mm2);
                self.report.geometric_residual = Some(residual.region);
                self.report.geometric_residual_status = ResidualStatus::Estimated;
            }
            Err(e) => {
                self.report.geometric_residual_status = ResidualStatus::Unknown;
                self.report.diagnostics.push(e);
            }
        }
        Ok(())
    }
    fn sign(&self) -> f64 {
        if self.op.clockwise { -1. } else { 1. }
    }
    fn normal_for(&self, d: Point) -> Point {
        [self.sign() * d[1], -self.sign() * d[0]]
    }
    fn z(&self) -> f64 {
        (self.op.pocket.top_z_mm - (self.pass + 1) as f64 * self.op.pocket.stepdown_mm)
            .max(self.report.target_z_mm)
    }
    fn tip(&self, p: Point, n: Point, z: f64) -> Vec3 {
        [
            p[0] + self.active_radius * n[0],
            p[1] + self.active_radius * n[1],
            z,
        ]
    }
    fn emit(
        &mut self,
        curve: Curve,
        kind: MoveKind,
        entry: bool,
        stock: &Stock,
    ) -> Result<Option<Motion>, String> {
        curve.prepare(self.op.pocket.tolerance_mm)?;
        let motion = Motion {
            id: self.next_id,
            operation: self.op.pocket.id.clone(),
            tool: self.op.pocket.tool.clone(),
            kind,
            curve,
            feed_mm_min: if kind == MoveKind::Rapid {
                None
            } else {
                Some(if entry {
                    self.op
                        .pocket
                        .entry
                        .as_ref()
                        .and_then(|e| e.feed_mm_min())
                        .unwrap_or(self.op.pocket.plunge_mm_min)
                } else {
                    self.op.pocket.feed_mm_min
                })
            },
        };
        if let Some(front) = &mut self.front {
            front.emitted(stock, &motion);
        }
        self.position = motion.curve.end();
        self.next_id += 1;
        self.report.motions += 1;
        self.generation = Some(stock.generation() + 1);
        Ok(Some(motion))
    }
    fn stock_clears_disk(
        &mut self,
        radius: f64,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        let mut work = usize::MAX;
        if work == 0 {
            return Ok(false);
        }
        let start = [
            self.center[0] + radius * self.normal[0],
            self.center[1] + radius * self.normal[1],
            self.z(),
        ];
        let query = Motion {
            id: self.next_id,
            operation: self.op.pocket.id.clone(),
            tool: self.op.pocket.tool.clone(),
            kind: MoveKind::Link,
            curve: Curve::Arc {
                start,
                end: start,
                center: [self.center[0], self.center[1], self.z()],
                plane: Plane::Xy,
                sweep_rad: self.sign() * TAU,
            },
            feed_mm_min: Some(self.op.pocket.feed_mm_min),
        };
        let before = work;
        let clear = !stock.may_remove_material(
            &query,
            self.r,
            0.,
            self.op.pocket.tolerance_mm,
            &mut work,
            cancel,
        )?;
        self.report.stock_query_work += before - work;
        self.report.stock_query_work_exhausted |= work == 0;
        Ok(clear)
    }
    fn adjust_radius(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        let guide = self.tours[self.component][self.guide];
        let mut target = guide.radius;
        if let Some(covered) = self.cleared_edges.get(&guide.edge) {
            let smaller = covered - self.r - 2. * self.guard;
            let parallel = guide.stroke.arc.is_none_or(|(c, sweep)| {
                distance(guide.stroke.a, c) + smaller * self.sign() * sweep.signum() > self.guard
            });
            if smaller > 4. * self.guard && parallel {
                // A cleared return never needs a larger offset or a new orbit.
                target = target.min(smaller).min(self.active_radius);
            }
        }
        if (target - self.active_radius).abs() <= self.op.pocket.tolerance_mm {
            return Ok(None);
        }
        let stock_clear = target > self.active_radius
            && target + self.r > self.disk_radius
            && self.stock_clears_disk(target, stock, cancel)?;
        let next =
            if target < self.active_radius || target + self.r <= self.disk_radius || stock_clear {
                target
            } else {
                let half = self.op.max_engagement_deg.to_radians() / 2.;
                let allowed = (self.disk_radius * self.disk_radius
                    - self.r * self.r * half.sin().powi(2))
                .max(0.)
                .sqrt()
                    - self.r * half.cos()
                    - 2. * self.guard;
                if allowed <= self.active_radius + self.guard {
                    return Err("adaptive radius growth below resolution".into());
                }
                target.min(allowed)
            };
        let grow = next + self.r > self.disk_radius && !stock_clear;
        let delta = next - self.active_radius;
        let end = [
            self.position[0] + delta * self.normal[0],
            self.position[1] + delta * self.normal[1],
            self.z(),
        ];
        let result = self.emit(
            Curve::Line {
                start: self.position,
                end,
            },
            MoveKind::Link,
            true,
            stock,
        )?;
        self.active_radius = next;
        if stock_clear {
            self.disk_radius = self.r + next;
            self.report.stock_clear_radius_changes += 1;
        }
        self.report.radius_changes += 1;
        self.report.min_active_radius_mm = self.report.min_active_radius_mm.min(next);
        if grow {
            let q = next + self.guard;
            let angle = 2.
                * ((self.disk_radius * self.disk_radius - q * q - self.r * self.r)
                    / (2. * q * self.r))
                    .clamp(-1., 1.)
                    .acos()
                    .to_degrees();
            self.report.max_bounded_engagement_deg =
                self.report.max_bounded_engagement_deg.max(angle);
            self.phase = Phase::RadiusOrbit;
        }
        Ok(result)
    }
    fn make_advance(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Advance, String> {
        let guide = self.tours[self.component][self.guide];
        let s = guide.stroke;
        let total = length(s);
        let remaining = (1. - self.fraction) * total;
        let half_budget = (guide.coverage - self.guard) / 2.;
        let a = self.r + self.active_radius;
        let half = self.op.max_engagement_deg.to_radians() / 2.;
        let bound = (self.disk_radius * self.disk_radius - self.r * self.r * half.sin().powi(2))
            .max(0.)
            .sqrt()
            - self.r * half.cos()
            - self.active_radius
            - 2. * self.guard;
        let mut step = bound.min(2. * (2. * a * half_budget - half_budget * half_budget).sqrt());
        if let Some((c, _)) = s.arc {
            let rho = distance(s.a, c);
            step = step
                .min(2. * rho * (1. - (half_budget / rho).min(1.)).acos())
                .min(PI * rho);
        }
        let mut committed_tube = self
            .cleared_edges
            .get(&guide.edge)
            .is_some_and(|radius| *radius >= a + self.guard);
        let mut clear = committed_tube;
        let mut query_work = usize::MAX;
        // Local clearance subdivision creates guides shorter than a bounded
        // cutting advance. They still need an air query, otherwise precleared
        // stock acquires redundant cutting orbits at every short piece.
        let query_short = self.op.local_orbit_tolerance_mm.is_some();
        if !clear && (remaining > step || query_short) && query_work > 0 {
            let query = Motion {
                id: self.next_id,
                operation: self.op.pocket.id.clone(),
                tool: self.op.pocket.tool.clone(),
                kind: MoveKind::Link,
                curve: match s.arc {
                    Some((c, sweep)) => Curve::Arc {
                        start: [self.center[0], self.center[1], self.z()],
                        end: [s.b[0], s.b[1], self.z()],
                        center: [c[0], c[1], self.z()],
                        plane: Plane::Xy,
                        sweep_rad: sweep * (1. - self.fraction),
                    },
                    None => Curve::Line {
                        start: [self.center[0], self.center[1], self.z()],
                        end: [s.b[0], s.b[1], self.z()],
                    },
                },
                feed_mm_min: Some(self.op.pocket.feed_mm_min),
            };
            let before = query_work;
            clear = !stock.may_remove_material(
                &query,
                a,
                0.,
                self.op.pocket.tolerance_mm,
                &mut query_work,
                cancel,
            )?;
            self.report.stock_query_work += before - query_work;
            self.report.stock_query_work_exhausted |= query_work == 0;
        }
        let bounded_step = step.min(remaining);
        step = if clear { remaining } else { bounded_step };
        loop {
            if step <= self.guard && step < remaining {
                return Err("adaptive advance below resolution".into());
            }
            let f = if step >= remaining {
                1.
            } else {
                self.fraction + step / total
            };
            let center = point(s, f);
            let mut n0 = self.normal_for(tangent(s, self.fraction));
            let mut n1 = self.normal_for(tangent(s, f));
            let mut chord = false;
            let curve = if let Some((c, sweep)) = s.arc {
                let rho = distance(s.a, c);
                let offset = rho + self.active_radius * self.sign() * sweep.signum();
                if offset > self.guard {
                    Curve::Arc {
                        start: self.tip(self.center, n0, self.z()),
                        end: self.tip(center, n1, self.z()),
                        center: [c[0], c[1], self.z()],
                        plane: Plane::Xy,
                        sweep_rad: sweep * (f - self.fraction),
                    }
                } else {
                    if !self
                        .guides
                        .link_geometry_fits_radius(self.center, center, a)
                    {
                        // The clear certificate covers the original guide tube,
                        // not this shortened chord. Restore both limits as well
                        // as revoking clear before retrying.
                        step = (step / 2.).min(bounded_step);
                        clear = false;
                        committed_tube = false;
                        continue;
                    }
                    if clear {
                        // An arc-tube certificate does not certify its chord tube.
                        clear = false;
                        committed_tube = false;
                        step = step.min(bounded_step);
                        continue;
                    }
                    let d = [
                        (center[0] - self.center[0]) / distance(self.center, center),
                        (center[1] - self.center[1]) / distance(self.center, center),
                    ];
                    n0 = self.normal_for(d);
                    n1 = n0;
                    chord = true;
                    Curve::Line {
                        start: self.tip(self.center, n0, self.z()),
                        end: self.tip(center, n1, self.z()),
                    }
                }
            } else {
                Curve::Line {
                    start: self.tip(self.center, n0, self.z()),
                    end: self.tip(center, n1, self.z()),
                }
            };
            let q = self.active_radius + step + self.guard;
            let angle = 2.
                * ((self.disk_radius * self.disk_radius - q * q - self.r * self.r)
                    / (2. * q * self.r))
                    .clamp(-1., 1.)
                    .acos()
                    .to_degrees();
            return Ok(Advance {
                curve,
                fraction: f,
                center,
                normal: n1,
                clear,
                committed_tube,
                angle,
                chord,
            });
        }
    }
    pub fn next_motion_with_stock(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        let result = self.next_planned(stock, cancel)?;
        if let (Some(bound), Some(motion)) = (&self.stock_top, &result) {
            bound.admit(motion)?;
        }
        Ok(result)
    }
    fn next_planned(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        if let Some(bound) = &mut self.stock_top {
            bound.validate(stock, cancel)?;
        }
        if stock.spec() != &self.spec
            || stock.generation() == u64::MAX
            || self.generation.is_some_and(|g| g != stock.generation())
        {
            return Err(
                "commit preceding adaptive motion to matching stock before planning".into(),
            );
        }
        if let Some(front) = &mut self.front {
            front.synchronize(stock, self.r, self.op.pocket.tolerance_mm, cancel)?;
            self.report.max_bounded_engagement_deg = self
                .report
                .max_bounded_engagement_deg
                .max(front.report.max_bounded_engagement_deg);
            self.report.front = Some(front.report.clone());
        }
        loop {
            if cancel.is_cancelled() {
                return Err("adaptive planning cancelled".into());
            }
            match self.phase {
                Phase::Front => {
                    let prototype = Motion {
                        id: self.next_id,
                        operation: self.op.pocket.id.clone(),
                        tool: self.op.pocket.tool.clone(),
                        kind: MoveKind::Cut,
                        curve: Curve::Line {
                            start: self.position,
                            end: [
                                self.position[0] + self.r,
                                self.position[1],
                                self.position[2],
                            ],
                        },
                        feed_mm_min: Some(self.op.pocket.feed_mm_min),
                    };
                    let floor = self.z();
                    let front = self.front.as_mut().ok_or("missing stock-front driver")?;
                    front.begin_component(
                        &self.guides,
                        self.center,
                        (self.pass, self.component),
                        floor,
                        self.r,
                        self.report.min_orbit_radius_mm,
                        self.r + self.report.min_orbit_radius_mm - self.op.boundary_tolerance_mm,
                        self.op.pocket.tolerance_mm,
                        self.op.boundary_tolerance_mm,
                    )?;
                    let result = front.next(
                        &self.guides,
                        stock,
                        self.position,
                        self.r,
                        self.report.min_orbit_radius_mm,
                        self.op.max_engagement_deg,
                        self.op.cutting_sense.unwrap_or(CuttingSense::Climb) == CuttingSense::Climb,
                        prototype,
                        self.op.pocket.tolerance_mm,
                        crate::adaptive_front::LinkHeights {
                            retract: self.heights.retract,
                            feed: self.heights.feed,
                        },
                        self.op.pocket.plunge_mm_min,
                        cancel,
                    );
                    self.report.max_bounded_engagement_deg = self
                        .report
                        .max_bounded_engagement_deg
                        .max(front.report.max_bounded_engagement_deg);
                    self.report.front = Some(front.report.clone());
                    let Some(motion) = result? else {
                        self.phase = Phase::Retract;
                        continue;
                    };
                    return self.emit(
                        motion.curve,
                        motion.kind,
                        motion.kind == MoveKind::Plunge,
                        stock,
                    );
                }
                Phase::Done => return Ok(None),
                Phase::Up => {
                    let end = [
                        self.position[0],
                        self.position[1],
                        self.op.pocket.clearance_z_mm,
                    ];
                    self.phase = Phase::Across;
                    if end != self.position {
                        return self.emit(
                            Curve::Line {
                                start: self.position,
                                end,
                            },
                            MoveKind::Rapid,
                            false,
                            stock,
                        );
                    }
                }
                Phase::Across => {
                    let first = self.tours[self.component][0];
                    let s = first.stroke;
                    self.active_radius = self
                        .helical
                        .as_ref()
                        .map_or(first.radius, |locations| locations[self.component].radius);
                    self.disk_radius = 0.;
                    self.report.min_active_radius_mm =
                        self.report.min_active_radius_mm.min(self.active_radius);
                    self.center = s.a;
                    self.normal = self.normal_for(tangent(s, 0.));
                    // Travel at the current height: clearance first, then
                    // the retract height between regions.
                    let end = self.tip(self.center, self.normal, self.position[2]);
                    self.phase = Phase::Descend;
                    self.entry = if self
                        .helical
                        .as_ref()
                        .is_some_and(|locations| !locations[self.component].stock_edge)
                    {
                        Phase::EntryTop
                    } else {
                        Phase::Plunge
                    };
                    if end != self.position {
                        return self.emit(
                            Curve::Line {
                                start: self.position,
                                end,
                            },
                            MoveKind::Rapid,
                            false,
                            stock,
                        );
                    }
                }
                Phase::Descend => {
                    self.phase = self.entry;
                    let end = self.tip(self.center, self.normal, self.heights.feed);
                    if end[2] < self.position[2] {
                        return self.emit(
                            Curve::Line {
                                start: self.position,
                                end,
                            },
                            MoveKind::Rapid,
                            false,
                            stock,
                        );
                    }
                }
                Phase::Plunge => {
                    let end = self.tip(self.center, self.normal, self.z());
                    self.phase = Phase::Seed;
                    let result = self.emit(
                        Curve::Line {
                            start: self.position,
                            end,
                        },
                        MoveKind::Plunge,
                        true,
                        stock,
                    )?;
                    if self
                        .helical
                        .as_ref()
                        .is_some_and(|locations| locations[self.component].stock_edge)
                    {
                        self.report.stock_edge_entries += 1;
                    }
                    return Ok(result);
                }
                Phase::EntryTop => {
                    let end = self.tip(self.center, self.normal, self.op.pocket.top_z_mm);
                    self.phase = Phase::EntryHelix;
                    return self.emit(
                        Curve::Line {
                            start: self.position,
                            end,
                        },
                        MoveKind::Plunge,
                        true,
                        stock,
                    );
                }
                Phase::EntryHelix => {
                    let h = self
                        .op
                        .pocket
                        .entry
                        .as_ref()
                        .and_then(|e| e.helix())
                        .ok_or("missing helical entry")?;
                    let turns = h.turns(self.active_radius, self.op.pocket.top_z_mm - self.z())?;
                    let default_clockwise = if self.front.is_some() {
                        self.op.cutting_sense.unwrap_or(CuttingSense::Climb)
                            == CuttingSense::Conventional
                    } else {
                        self.op.clockwise
                    };
                    let sweep = if h.clockwise.unwrap_or(default_clockwise) {
                        -TAU
                    } else {
                        TAU
                    };
                    // One motion per full turn: every controller can represent
                    // a <=one-turn helix as a single G2/G3 block.
                    let (done, total) = self.entry_turn.unwrap_or((0, turns));
                    let top = self.op.pocket.top_z_mm;
                    let bottom = self.z();
                    let end_z = if done + 1 == total {
                        bottom
                    } else {
                        top - (top - bottom) * (done + 1) as f64 / total as f64
                    };
                    let curve = Curve::Helix {
                        start: self.position,
                        end: self.tip(self.center, self.normal, end_z),
                        center: [self.center[0], self.center[1], self.position[2]],
                        plane: Plane::Xy,
                        sweep_rad: sweep,
                    };
                    let result = self.emit(curve, MoveKind::Lead, true, stock)?;
                    if done + 1 < total {
                        self.entry_turn = Some((done + 1, total));
                        return Ok(result);
                    }
                    self.entry_turn = None;
                    self.report.helical_entries += 1;
                    self.report.entry_turns += total;
                    self.phase = Phase::Seed;
                    return Ok(result);
                }
                Phase::Seed | Phase::Orbit | Phase::RadiusOrbit => {
                    let seed = self.phase == Phase::Seed;
                    if seed
                        && self.front.is_none()
                        && self.stock_clears_disk(self.active_radius, stock, cancel)?
                    {
                        self.disk_radius = self.r + self.active_radius;
                        self.report.omitted_seed_orbits += 1;
                        self.phase = Phase::Turn;
                        continue;
                    }
                    let growth = self.phase == Phase::RadiusOrbit;
                    let z = self.z();
                    let sign = self.sign();
                    if !seed && let Some(proof) = &mut self.report.empty_cuts {
                        // radius<=r is enforced at admission: the full circle
                        // swept by the cutter covers the entire support disk.
                        // Query canonical active radius, not a sampled annulus.
                        let start = [
                            self.center[0] + self.active_radius * self.normal[0],
                            self.center[1] + self.active_radius * self.normal[1],
                            z,
                        ];
                        let support = Motion {
                            id: self.next_id,
                            operation: self.op.pocket.id.clone(),
                            tool: self.op.pocket.tool.clone(),
                            kind: MoveKind::Cut,
                            curve: Curve::Arc {
                                start,
                                end: start,
                                center: [self.center[0], self.center[1], z],
                                plane: Plane::Xy,
                                sweep_rad: sign * TAU,
                            },
                            feed_mm_min: Some(self.op.pocket.feed_mm_min),
                        };
                        if proof.may_omit(
                            stock,
                            &support,
                            self.r,
                            self.op.pocket.tolerance_mm,
                            cancel,
                        )? {
                            self.disk_radius = self.r + self.active_radius;
                            self.phase = Phase::Turn;
                            // Closed orbit ends at its start: actual pose/IDs/
                            // generation do not advance for the omitted move.
                            continue;
                        }
                    }
                    let curve = Curve::Arc {
                        start: self.position,
                        end: self.position,
                        center: [self.center[0], self.center[1], self.z()],
                        plane: Plane::Xy,
                        sweep_rad: if seed && self.front.is_some() {
                            if self.op.cutting_sense.unwrap_or(CuttingSense::Climb)
                                == CuttingSense::Climb
                            {
                                TAU
                            } else {
                                -TAU
                            }
                        } else if seed {
                            self.op
                                .pocket
                                .entry
                                .as_ref()
                                .and_then(|e| e.helix()?.clockwise)
                                .map_or(self.sign(), |clockwise| if clockwise { -1. } else { 1. })
                                * TAU
                        } else {
                            self.sign() * TAU
                        },
                    };
                    let result = self.emit(curve, MoveKind::Cut, seed, stock)?;
                    if seed {
                        self.report.seed_orbits += 1;
                    } else {
                        self.report.controlled_orbits += 1;
                    }
                    self.disk_radius = self.r + self.active_radius;
                    self.report.min_cutting_orbit_radius_mm = Some(
                        self.report
                            .min_cutting_orbit_radius_mm
                            .map_or(self.active_radius, |old| old.min(self.active_radius)),
                    );
                    if growth {
                        self.report.radius_growth_orbits += 1;
                    }
                    self.phase = if seed && self.front.is_some() {
                        Phase::Front
                    } else {
                        Phase::Turn
                    };
                    return Ok(result);
                }
                Phase::Turn => {
                    while self.fraction == 1. {
                        if self.report.min_orbit_radius_mm < self.report.orbit_radius_mm {
                            let guide = self.tours[self.component][self.guide];
                            let radius = self.r + self.active_radius - guide.coverage - self.guard;
                            self.cleared_edges
                                .entry(guide.edge)
                                .and_modify(|old| *old = old.max(radius))
                                .or_insert(radius);
                        }
                        self.guide += 1;
                        self.fraction = 0.;
                    }
                    if self.guide == self.tours[self.component].len() {
                        self.phase = Phase::Retract;
                        continue;
                    }
                    if self.advance.is_none() {
                        if let Some(motion) = self.adjust_radius(stock, cancel)? {
                            return Ok(Some(motion));
                        }
                        self.advance = Some(self.make_advance(stock, cancel)?);
                    }
                    let end = self.advance.as_ref().unwrap().curve.start();
                    self.phase = Phase::Advance;
                    if distance([self.position[0], self.position[1]], [end[0], end[1]])
                        > self.op.pocket.tolerance_mm
                    {
                        let old = [
                            self.position[0] - self.center[0],
                            self.position[1] - self.center[1],
                        ];
                        let next = [end[0] - self.center[0], end[1] - self.center[1]];
                        let angle = (old[0] * next[1] - old[1] * next[0])
                            .atan2(old[0] * next[0] + old[1] * next[1]);
                        let sweep = if self.sign() > 0. {
                            angle.rem_euclid(TAU)
                        } else {
                            -(-angle).rem_euclid(TAU)
                        };
                        let result = self.emit(
                            Curve::Arc {
                                start: self.position,
                                end,
                                center: [self.center[0], self.center[1], self.z()],
                                plane: Plane::Xy,
                                sweep_rad: sweep,
                            },
                            MoveKind::Link,
                            false,
                            stock,
                        )?;
                        self.report.turn_arcs += 1;
                        return Ok(result);
                    }
                }
                Phase::Advance => {
                    let next = self.advance.take().unwrap();
                    let mut curve = next.curve;
                    match &mut curve {
                        Curve::Line { start, .. } | Curve::Arc { start, .. } => {
                            *start = self.position
                        }
                        _ => unreachable!(),
                    }
                    let result = self.emit(curve, MoveKind::Link, false, stock)?;
                    if next.clear {
                        if next.committed_tube {
                            self.report.committed_tube_advances += 1;
                        } else {
                            self.report.stock_clear_advances += 1;
                        }
                        self.report.omitted_endpoint_orbits += 1;
                    } else {
                        self.report.bounded_advances += 1;
                        self.report.max_bounded_engagement_deg =
                            self.report.max_bounded_engagement_deg.max(next.angle);
                    }
                    self.report.chord_fallbacks += usize::from(next.chord);
                    self.fraction = next.fraction;
                    self.center = next.center;
                    self.normal = next.normal;
                    self.phase = if next.clear {
                        Phase::Turn
                    } else {
                        Phase::Orbit
                    };
                    if next.clear {
                        // A complete committed guide also proved both endpoint
                        // disks before its tube was contracted for scallops.
                        // Retain that larger endpoint witness on a cached return.
                        self.disk_radius = if next.committed_tube {
                            let guide = self.tours[self.component][self.guide];
                            self.cleared_edges[&guide.edge] + guide.coverage + self.guard
                        } else {
                            self.r + self.active_radius
                        };
                    }
                    return Ok(result);
                }
                Phase::Retract => {
                    // Leave the cut at feed up to the feed height.
                    let end = [self.position[0], self.position[1], self.heights.feed];
                    self.component += 1;
                    if self.component == self.tours.len() {
                        self.component = 0;
                        self.pass += 1;
                        self.cleared_edges.clear();
                    }
                    self.guide = 0;
                    self.fraction = 0.;
                    self.phase = Phase::Rise;
                    if end[2] > self.position[2] {
                        return self.emit(
                            Curve::Line {
                                start: self.position,
                                end,
                            },
                            MoveKind::Retract,
                            false,
                            stock,
                        );
                    }
                }
                Phase::Rise => {
                    // Rapid to the retract height, or clearance at the end.
                    let done = self.pass == self.report.depth_passes;
                    let z = if done {
                        self.heights.clearance
                    } else {
                        self.heights.retract
                    };
                    let end = [self.position[0], self.position[1], z.max(self.position[2])];
                    self.phase = if done { Phase::Done } else { Phase::Across };
                    if end != self.position {
                        return self.emit(
                            Curve::Line {
                                start: self.position,
                                end,
                            },
                            MoveKind::Rapid,
                            false,
                            stock,
                        );
                    }
                }
            }
        }
    }
}
