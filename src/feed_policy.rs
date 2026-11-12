//! Explicit engagement/load caps, reusing committed native stock.
//!
//! Angular contact is not enough to prove chip thinning or spindle load.
//! Only proved outside-original-stock air may use a declared higher cap.
//! Caps and unresolved behavior
//! are caller declarations, not inferred cutting data. A final machine schedule
//! and independent posted replay are still required.
use crate::{
    engagement::{
        ClearedSweep, EngagementOptions, EngagementScene, EngagementStatus, UnresolvedReason,
    },
    motion::{Curve, Event, MoveKind, Plane, Program},
    stock::{CancelSignal, InitialStock, StockSpec, SweepOptions},
    stream::{DEFAULT_FRAME_LIMIT, RunIdentity, StreamManifest, StreamProducer},
    tool::CutterShape,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FeedBand {
    pub at_least_angle_deg: f64,
    pub max_feed_mm_min: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum UnresolvedPolicy {
    Refuse,
    Preserve,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FeedPolicy {
    pub bands: Vec<FeedBand>,
    pub unresolved: UnresolvedPolicy,
    #[serde(default)]
    pub options: EngagementOptions,
    #[serde(default)]
    pub air_feed_mm_min: Option<f64>,
    #[serde(default)]
    pub tool_limits: Vec<ToolLoadLimit>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ToolLoadLimit {
    pub tool: String,
    pub max_axial_mm: f64,
    pub max_radial_mm: f64,
    pub max_cut_feed_mm_min: f64,
}
#[derive(Clone, Debug, Serialize)]
pub struct Observation {
    pub event_index: usize,
    pub motion_id: u64,
    pub stock_generation: u64,
    pub status: String,
    pub upper_angle_deg: Option<f64>,
    pub original_feed_mm_min: Option<f64>,
    pub requested_feed_mm_min: Option<f64>,
    pub shape: CutterShape,
    pub radial_upper_mm: Option<f64>,
    pub axial_upper_mm: Option<f64>,
    pub outside_original_stock: bool,
    pub reason: Option<String>,
}
#[derive(Clone, Debug, Serialize)]
pub struct FeedReport {
    pub identity: String,
    pub revision: String,
    pub observations: Vec<Observation>,
    pub scope: String,
}

pub fn apply(
    program: &Program,
    stock: StockSpec,
    initial_stock: Option<InitialStock>,
    sweep: SweepOptions,
    policy: &FeedPolicy,
    cancel: &impl CancelSignal,
) -> Result<FeedReport, String> {
    program.validate(None)?;
    if program
        .tools
        .iter()
        .any(|t| t.cutter.shape == CutterShape::Beam)
    {
        return Err("engagement feed policy supports milling, not beam cutters".into());
    }
    stock.grid()?;
    sweep.validate()?;
    policy.options.validate()?;
    if policy
        .air_feed_mm_min
        .is_some_and(|v| !v.is_finite() || v <= 0.)
        || policy.tool_limits.iter().enumerate().any(|(i, l)| {
            !program.tools.iter().any(|t| t.id == l.tool)
                || policy.tool_limits[..i].iter().any(|old| old.tool == l.tool)
                || [l.max_axial_mm, l.max_radial_mm, l.max_cut_feed_mm_min]
                    .iter()
                    .any(|v| !v.is_finite() || *v <= 0.)
        })
    {
        return Err(
            "feed policy needs positive finite air/tool limits and distinct known tool IDs".into(),
        );
    }
    if policy.bands.is_empty()
        || policy.bands.iter().any(|b| {
            !b.at_least_angle_deg.is_finite()
                || !(0. ..=360.).contains(&b.at_least_angle_deg)
                || !b.max_feed_mm_min.is_finite()
                || b.max_feed_mm_min <= 0.
        })
        || policy.bands.windows(2).any(|b| {
            b[0].at_least_angle_deg >= b[1].at_least_angle_deg
                || b[0].max_feed_mm_min < b[1].max_feed_mm_min
        })
    {
        return Err("feed bands need increasing contact-angle thresholds and positive nonincreasing feed caps".into());
    }
    if program.events.iter().any(|e| {
        matches!(
            e,
            Event::SurfaceSpeed { .. }
                | Event::FeedPerRev { .. }
                | Event::Beam { .. }
                | Event::BeamMark { .. }
                | Event::HeightControl { .. }
        ) || matches!(e, Event::Motion {motion} if motion.kind == MoveKind::Thread)
    }) {
        return Err("engagement feed policy supports milling with per-minute feeds, not turning or thermal controls".into());
    }
    let bytes = serde_json::to_vec(&(program, &stock, &initial_stock, sweep, policy))
        .map_err(|e| e.to_string())?;
    let identity = format!("{:x}", Sha256::digest(bytes));
    let revision = format!("engagement@{identity}");
    let mut replay = StreamProducer::new(StreamManifest {
        version: 2,
        run: RunIdentity {
            id: program.id.clone(),
            revision: revision.clone(),
        },
        stock: stock.clone().into(),
        initial_stock,
        tools: program.tools.clone(),
        initial_position_mm: program.initial_position_mm,
        sweep,
        operations: vec![],
        verification: None,
        tool_change: None,
        rapid_retracts: false,
        turning: None,
    })?;
    // Initial irregular stock is enclosed by its StockSpec box here. Missing
    // material can only overstate engagement. Supports are added exclusively
    // after their matching native stock transaction has committed.
    let mut scene = EngagementScene::new(stock, vec![])?;
    let mut observations = Vec::new();
    let mut cycle_until = None;
    for (index, event) in program.events.iter().enumerate() {
        if cancel.is_cancelled() {
            return Err("engagement feed analysis cancelled".into());
        }
        if let Event::Cycle { covers, .. } = event {
            cycle_until = Some(index + *covers as usize);
            continue; // metadata; the authoritative covered motions follow
        }
        if let Event::Motion { motion } = event {
            let tool = program
                .tools
                .iter()
                .find(|t| t.id == motion.tool)
                .ok_or("unknown feed-policy tool")?;
            let horizontal = motion.curve.start()[2] == motion.curve.end()[2]
                && matches!(
                    motion.curve,
                    Curve::Line { .. }
                        | Curve::Arc {
                            plane: Plane::Xy,
                            ..
                        }
                );
            let supported = tool.cutter.shape == CutterShape::Flat && horizontal;
            let general = !policy.tool_limits.is_empty() && tool.cutter.shape != CutterShape::Beam;
            let synchronized =
                cycle_until.is_some_and(|end| index <= end) || motion.kind == MoveKind::Tap;
            let mut observation = Observation {
                event_index: index,
                motion_id: motion.id,
                stock_generation: replay.stock()?.generation(),
                status: "unchanged_role".into(),
                upper_angle_deg: None,
                original_feed_mm_min: motion.feed_mm_min,
                requested_feed_mm_min: motion.feed_mm_min,
                shape: tool.cutter.shape.clone(),
                radial_upper_mm: None,
                axial_upper_mm: None,
                outside_original_stock: false,
                reason: None,
            };
            if synchronized {
                observation.status = "unchanged_cycle".into();
            } else if motion.kind == MoveKind::Cut || general && motion.kind == MoveKind::Plunge {
                let radius = tool.cutter.diameter_mm / 2.;
                let air = replay.stock()?.sweep_misses_stock_envelope(
                    motion,
                    radius,
                    sweep.tolerance_mm,
                    cancel,
                )?;
                observation.outside_original_stock = air;
                let limits = policy.tool_limits.iter().find(|l| l.tool == tool.id);
                let mut load_ok = true;
                if general || air && policy.air_feed_mm_min.is_some() {
                    let bounds = motion.curve.prepare(sweep.tolerance_mm)?.bounds();
                    let state = replay.stock()?;
                    let spec = state.spec();
                    let [nx, ny] = state.dimensions();
                    let step = state.cell_widths_mm();
                    let pad = radius + crate::verification::path_error(motion, sweep.tolerance_mm);
                    let range = |k: usize, n: usize| {
                        let lo = ((bounds.min[k] - pad - spec.min_mm[k]) / step[k]).floor();
                        let hi = ((bounds.max[k] + pad - spec.min_mm[k]) / step[k]).floor();
                        if hi < 0. || lo >= n as f64 {
                            0..0
                        } else {
                            lo.max(0.) as usize..((hi as usize).min(n - 1) + 1)
                        }
                    };
                    let xs = range(0, nx);
                    let ys = range(1, ny);
                    if !air && xs.len().saturating_mul(ys.len()) as u64 > policy.options.max_work {
                        load_ok = false;
                        observation.reason = Some("load_bound_work_limit".into());
                    } else {
                        let mut top = spec.min_mm[2];
                        if !air {
                            for y in ys {
                                for x in xs.clone() {
                                    if cancel.is_cancelled() {
                                        return Err("engagement load analysis cancelled".into());
                                    }
                                    top = top.max(state.cells()[y * nx + x].upper_mm);
                                }
                            }
                        }
                        let axial = if air {
                            0.
                        } else {
                            (top - bounds.min[2]
                                + crate::verification::path_error(motion, sweep.tolerance_mm))
                            .max(0.)
                            .min(tool.cutter.cutting_length_mm)
                        };
                        let radial = if air { 0. } else { tool.cutter.diameter_mm };
                        observation.axial_upper_mm = Some(axial);
                        observation.radial_upper_mm = Some(radial);
                        observation.upper_angle_deg = Some(if air { 0. } else { 360. });
                        if !policy.tool_limits.is_empty()
                            && limits
                                .is_none_or(|l| axial > l.max_axial_mm || radial > l.max_radial_mm)
                        {
                            load_ok = false;
                            observation.reason =
                                Some("approved_load_limit_missing_or_exceeded".into());
                        }
                    }
                }
                if !load_ok {
                    observation.status = "unresolved".into();
                    if matches!(policy.unresolved, UnresolvedPolicy::Refuse) {
                        let detail = if observation.reason.as_deref()
                            == Some("load_bound_work_limit")
                        {
                            format!(
                                "load analysis exceeded the declared work budget of {}; increase the analysis budget or simplify the requested sweep",
                                policy.options.max_work
                            )
                        } else if let Some(l) = limits {
                            let exceeded: Vec<_> = [
                                ("axial", observation.axial_upper_mm, l.max_axial_mm),
                                ("radial", observation.radial_upper_mm, l.max_radial_mm),
                            ]
                            .into_iter()
                            .filter_map(|(axis, upper, limit)| {
                                upper.filter(|v| *v > limit).map(|upper| {
                                    format!("{axis} load bound {upper} mm exceeds approved maximum {limit} mm")
                                })
                            })
                            .collect();
                            format!(
                                "{}; revise the operation or supply reviewed tool load limits; reducing feed alone does not reduce these geometric bounds",
                                exceeded.join("; ")
                            )
                        } else {
                            "approved tool load limits are missing; declare ToolLoadLimit with reviewed maximum axial depth (mm), radial width (mm) and cutting feed (mm/min) for this tool".into()
                        };
                        return Err(format!(
                            "motion {} (tool '{}'): {detail} [{}]",
                            motion.id,
                            tool.id,
                            observation.reason.as_deref().unwrap_or("unresolved_load")
                        ));
                    }
                } else if air && policy.air_feed_mm_min.is_some() {
                    observation.status = "proven_air".into();
                    observation.requested_feed_mm_min = Some(
                        policy
                            .air_feed_mm_min
                            .unwrap()
                            .min(limits.map_or(f64::INFINITY, |l| l.max_cut_feed_mm_min)),
                    );
                } else if !supported && !general {
                    observation.status = "unsupported".into();
                    if matches!(policy.unresolved, UnresolvedPolicy::Refuse) {
                        return Err(format!(
                            "motion {}: engagement policy needs a flat cutter and horizontal line/XY arc",
                            motion.id
                        ));
                    }
                } else if !supported {
                    observation.status = "bounded_load".into();
                    let mut feed = motion.feed_mm_min.ok_or("cut has no feed")?;
                    for band in &policy.bands {
                        if observation.upper_angle_deg.unwrap_or(360.) >= band.at_least_angle_deg {
                            feed = feed.min(band.max_feed_mm_min);
                        }
                    }
                    observation.requested_feed_mm_min =
                        Some(feed.min(limits.unwrap().max_cut_feed_mm_min));
                } else {
                    let report = scene.check(
                        motion,
                        tool.cutter.diameter_mm / 2.,
                        // Bound the contact, rather than requiring it to be
                        // below a feed-band threshold. A full-slot bound of
                        // 360 degrees is useful slowdown advice.
                        360.,
                        policy.options,
                        cancel,
                    )?;
                    if matches!(report.status, EngagementStatus::Unresolved) {
                        observation.status = "unresolved".into();
                        if matches!(policy.unresolved, UnresolvedPolicy::Refuse) {
                            let detail = match report.reason {
                                Some(UnresolvedReason::Work) => format!(
                                    "work/node budget exhausted (max_work={}, max_nodes={}); increase the declared analysis budget",
                                    policy.options.max_work, policy.options.max_nodes
                                ),
                                Some(UnresolvedReason::Resolution) => format!(
                                    "contact resolution {} mm reached; refine the declared analysis resolution",
                                    policy.options.resolution_mm
                                ),
                                Some(UnresolvedReason::Depth) => format!(
                                    "subdivision depth limit {} reached; increase the declared analysis depth",
                                    policy.options.max_depth
                                ),
                                None => "contact bound remains unproven; review the declared analysis limits".into(),
                            };
                            return Err(format!(
                                "motion {} (tool '{}'): engagement bound unresolved: {detail}",
                                motion.id, tool.id
                            ));
                        }
                    } else {
                        observation.status = "bounded".into();
                        observation.upper_angle_deg = Some(report.upper_angle_deg);
                        let mut feed = motion.feed_mm_min.ok_or("cut has no feed")?;
                        for band in &policy.bands {
                            if report.upper_angle_deg >= band.at_least_angle_deg {
                                feed = feed.min(band.max_feed_mm_min);
                            }
                        }
                        observation.requested_feed_mm_min = Some(feed);
                        if let Some(l) = limits {
                            observation.requested_feed_mm_min =
                                Some(feed.min(l.max_cut_feed_mm_min));
                        }
                    }
                }
            }
            // Replay the original geometry/feed. Declared feed revisions
            // cannot alter geometric removal; the final program is rechecked.
            replay.push_motion(motion.clone(), DEFAULT_FRAME_LIMIT, cancel)?;
            if supported && !motion.kind.is_rapid() {
                scene.append(ClearedSweep::from_motion(
                    motion,
                    tool.cutter.diameter_mm / 2.,
                    sweep.tolerance_mm,
                )?)?;
            }
            observations.push(observation);
        } else {
            replay.push_control(event.clone(), DEFAULT_FRAME_LIMIT, cancel)?;
        }
    }
    Ok(FeedReport {identity, revision, observations,
        scope: "Conditional angular bounds and conservative whole-curve axial/radial bounds from committed stock. Explicit air increases require analytic separation from original stock. No chip-thinning, force, power or torque guarantee; machine scheduling and independent posted verification remain required.".into()})
}
