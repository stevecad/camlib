use super::geometry::{Contact, Pair, mul};
use super::*;
use crate::{
    motion::{Curve, Event, Motion, PreparedCurve},
    stream::{RunStock, StreamManifest, StreamProducer},
    tool::PreparedAssembly,
};

const LIMIT: usize = 64 * 1024 * 1024;

struct Path {
    from: Vec3,
    to: Vec3,
    curve: Option<PreparedCurve>,
    /// The decoded move in work coordinates (a line too), for the travel
    /// rule; `None` for a route between declared machine-axis positions.
    work: Option<PreparedCurve>,
    frame: WorkFrame,
    gauge: Vec3,
    speed: f64,
    dogleg: bool,
    absolute: bool,
    error: Vec3,
    /// For a decoded move, the machine axes still at a declared machine-axis
    /// position at its start, and those still there for the rest of it
    /// (`travel::curve_calibrated_exact`); the travel rule holds them
    /// exactly, without the calibration interval.
    exact_start: [Option<f64>; 3],
    exact_rest: [Option<f64>; 3],
}
impl Path {
    fn absolute(from: Vec3, to: Vec3, dogleg: bool) -> Self {
        Self {
            from,
            to,
            curve: None,
            work: None,
            frame: WorkFrame::identity(),
            gauge: [0.; 3],
            speed: sub(to, from).into_iter().map(f64::abs).sum(),
            dogleg,
            absolute: true,
            error: [0.; 3],
            exact_start: [None; 3],
            exact_rest: [None; 3],
        }
    }
    fn position(&self, t: f64) -> Result<Vec3, String> {
        if let Some(c) = &self.curve {
            Ok(add(transform(&self.frame, c.position(t)?), self.gauge))
        } else {
            Ok(add(self.from, mul(sub(self.to, self.from), t)))
        }
    }
}
#[derive(Clone, Default)]
struct Attribution {
    motion: Option<u64>,
    event: Option<usize>,
    route: Option<String>,
}

/// An owned, fail-closed replay of actual decoded events. No imported stock
/// deltas are trusted: the existing native stock kernel recomputes every cut.
/// Any error poisons the session, so an incomplete prefix cannot be released.
pub struct Session {
    prepared: Prepared,
    replay: StreamProducer,
    tools: BTreeMap<String, PreparedAssembly>,
    tool_shapes: BTreeMap<String, Vec<Shape>>,
    /// Each tool's lowest point along the tool axis, in its own frame (tip
    /// at the origin), before the gauge offset (`Session::floor_rise`).
    tool_low_z: BTreeMap<String, f64>,
    /// The stock replay's vertical guard on a floor it cuts: a cell cut flat
    /// at Z keeps the upper bound Z + this (`SweepOptions.tolerance_mm`).
    floor_guard_mm: f64,
    active: Option<String>,
    axes: Vec3,
    /// Each machine axis still at the declared machine-axis position that
    /// last set it (the start pose, a route waypoint), because no posted
    /// block has commanded it in work coordinates since: `None` once one may
    /// have (see `Session::chunk`).
    exact: [Option<f64>; 3],
    /// The bound post's axis word addresses (`Dialect.axes`) and whether
    /// parentheses are comments, to find which axes a posted block commands.
    /// `None` when the definition does not state them: then every block is
    /// taken to command every axis.
    axis_letters: Option<[u8; 3]>,
    parentheses_comments: bool,
    report: Report,
    failed: bool,
    finished: bool,
    last_event: Option<usize>,
}
impl Session {
    pub fn new(
        model: Model,
        mut manifest: StreamManifest,
        definition: &serde_json::Value,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        let prepared = model.prepare(cancel)?;
        if (model.process == Process::Turning) != manifest.turning.is_some() {
            return Err("machine process differs from posted stock/setup".into());
        }
        let machine = definition
            .get("machine")
            .ok_or("machine check needs bound posting definition")?;
        let frame: WorkFrame = serde_json::from_value(
            machine
                .get("frame")
                .cloned()
                .ok_or("posting frame missing")?,
        )
        .map_err(|e| e.to_string())?;
        // Program.frame identifies the CAD/setup frame. NC and replay stock
        // are local to that setup; work_to_machine is the physical placement.
        frame.validate()?;
        let posted_frame: WorkFrame = serde_json::from_value(
            machine
                .get("work_to_machine")
                .cloned()
                .ok_or("posting machine frame missing")?,
        )
        .map_err(|e| e.to_string())?;
        if posted_frame != model.work_to_machine
            || machine.get("work_offset").and_then(|v| v.as_u64()) != Some(model.work_offset as u64)
        {
            return Err(
                "machine crash model work offset/frame differs from the bound post definition"
                    .into(),
            );
        }

        let table = machine
            .get("tools")
            .and_then(|v| v.as_array())
            .ok_or("posting tool table missing")?;
        if model.tools.len() != manifest.tools.len()
            || model
                .tools
                .iter()
                .any(|t| !manifest.tools.iter().any(|u| u.id == t.tool))
        {
            return Err("machine calibrations must match exactly the posted assemblies".into());
        }
        let mut work = Budget {
            remaining: model.options.max_axis_tests,
            cancel,
        };
        let mut tools = BTreeMap::new();
        let mut tool_shapes = BTreeMap::new();
        let mut tool_low_z = BTreeMap::new();
        for t in &manifest.tools {
            let c = model
                .tools
                .iter()
                .find(|c| c.tool == t.id)
                .ok_or("tool calibration missing")?;
            if !table.iter().any(|v| {
                v.get("tool").and_then(|v| v.as_str()) == Some(&t.id)
                    && v.get("length_register").and_then(|v| v.as_u64())
                        == Some(c.length_register as u64)
            }) {
                return Err(format!(
                    "tool {} calibration register differs from post",
                    t.id
                ));
            }
            let p = t.prepare()?;
            let mut shapes = vec![];
            if matches!(t.cutter.shape, crate::tool::CutterShape::Beam) {
                return Err("thermal machinery needs its own physical process model; this checker covers milling and turning".into());
            }
            if !c.body_solids.is_empty() {
                for solid in &c.body_solids {
                    shapes.push(solid.prepare(model.options.tolerance_mm, &mut work)?);
                }
            } else if let Some(turning) = &manifest.turning {
                let setting = turning
                    .tools
                    .iter()
                    .find(|s| s.tool.id == t.id)
                    .ok_or("turning tool missing")?;
                let geometry =
                    crate::turning::ops::tool_geometry(&setting.tool, turning.controlled_point)?;
                let special = setting
                    .tool
                    .body
                    .as_ref()
                    .and_then(|b| b.insert.as_ref())
                    .filter(|b| b.curved_for_diameter_mm.is_some());
                for (i, pieces) in [geometry.insert_body, geometry.holder_body]
                    .into_iter()
                    .enumerate()
                {
                    for piece in pieces {
                        let mut lo = [f64::INFINITY; 3];
                        let mut hi = [f64::NEG_INFINITY; 3];
                        for v in piece {
                            let v = [v[1] - geometry.tip[1], v[2], v[0] - geometry.tip[0]];
                            for k in 0..3 {
                                lo[k] = lo[k].min(v[k]);
                                hi[k] = hi[k].max(v[k]);
                            }
                        }
                        if i == 0 {
                            if let crate::turning::tool::Insert::Drill { diameter_mm, .. } =
                                setting.tool.insert
                            {
                                lo[0] = -diameter_mm / 2.;
                                hi[0] = diameter_mm / 2.;
                                lo[1] = -diameter_mm / 2.;
                                hi[1] = diameter_mm / 2.;
                            }
                            if special.is_some() {
                                return Err(format!(
                                    "curved face-grooving tool {} needs complete ToolCalibration.body_solids: its axisymmetric stock envelope cannot locate the physical body in 3D",
                                    t.id
                                ));
                            }
                        }
                        shapes.push(Shape::aabb(lo, hi));
                    }
                }
            } else {
                if c.gauge_offset_min_mm[0] != 0.
                    || c.gauge_offset_max_mm[0] != 0.
                    || c.gauge_offset_min_mm[1] != 0.
                    || c.gauge_offset_max_mm[1] != 0.
                    || t.gauge_length_mm < c.gauge_offset_min_mm[2]
                    || t.gauge_length_mm > c.gauge_offset_max_mm[2]
                {
                    return Err(format!(
                        "milling tool {} needs axial gauge bounds containing its assembly gauge length",
                        t.id
                    ));
                }
                for part in crate::verification::parts(&p)? {
                    let r = part.radius.at(part.min).max(part.radius.at(part.max));
                    shapes.push(Shape::cylinder(
                        r,
                        [part.min, part.max],
                        model.options.spatial_resolution_mm,
                        &mut work,
                    )?);
                }
            }
            tool_low_z.insert(
                t.id.clone(),
                shapes
                    .iter()
                    .flat_map(|s| s.vertices.iter().map(|v| v[2]))
                    .fold(f64::INFINITY, f64::min),
            );
            let g = gauge(c);
            let shapes = shapes
                .into_iter()
                .map(|s| s.transformed(|v| sub(v, g), |v| v))
                .collect();
            tools.insert(t.id.clone(), p);
            tool_shapes.insert(t.id.clone(), shapes);
        }
        manifest.verification = None;
        let floor_guard_mm = manifest.sweep.tolerance_mm;
        // This replay consumes checked events, not the manifest's planner operations.
        let mut replay = StreamProducer::new(manifest)?;
        if model.process == Process::Milling {
            replay.enable_contact_queries(cancel)?;
        }
        let report=Report{algorithm:"declared-machine-continuous-v1",model_sha256:prepared.digest.clone(),status:Status::Clear,complete:false,decoded_motions:0,tool_changes:0,findings:vec![],allowed_contacts:model.allowed_contacts.clone(),scope:vec![
            "Declared upright three-axis mill or XZ lathe, nominal exact-stop decoded paths; common work calibration and correctly matched tool/register gauge ranges".into(),
            "Convex body unions translate continuously. Tool solids and lathe stock use conservative outer enclosures; unresolved enclosure contact refuses export".into(),
            "Indexing is covered only by caller-certified complete swept envelopes; rotary kinematics (#2), overhang stock (#1), actual setup mistakes and servo/blending deviations are not inferred".into(),
            "G28/G30/G53 and controller homing blocks are rejected by the independent decoder; controller-internal tool-change/end motion must be fully declared in the bound routes".into(),
        ]};
        let dialect = machine.get("dialect");
        let axis_letters = dialect
            .and_then(|d| d.get("axes"))
            .and_then(|a| a.as_array())
            .and_then(|a| {
                let letters: Vec<u8> = a
                    .iter()
                    .filter_map(|v| v.as_str())
                    .filter(|v| v.len() == 1 && v.as_bytes()[0].is_ascii_uppercase())
                    .map(|v| v.as_bytes()[0])
                    .collect();
                <[u8; 3]>::try_from(letters).ok()
            })
            .filter(|l| l[0] != l[1] && l[1] != l[2] && l[0] != l[2]);
        let parentheses_comments = dialect
            .and_then(|d| d.get("parentheses_comments"))
            .and_then(|v| v.as_bool())
            .unwrap_or(false);
        let mut s = Self {
            axes: model.initial_axes_mm,
            exact: model.initial_axes_mm.map(Some),
            axis_letters,
            parentheses_comments,
            active: model.initial_tool.clone(),
            prepared,
            replay,
            tools,
            tool_shapes,
            tool_low_z,
            floor_guard_mm,
            report,
            failed: true,
            finished: false,
            last_event: None,
        };
        if let Some(t) = &s.active {
            let expected = add(
                transform(&model.work_to_machine, s.replay.position()),
                gauge(s.calibration(t)?),
            );
            if (0..3).any(|k| (s.axes[k] - expected[k]).abs() > model.options.tolerance_mm) {
                return Err(
                    "initial machine axes do not match decoded initial tip and calibrated tool"
                        .into(),
                );
            }
        }
        let path = Path::absolute(s.axes, s.axes, false);
        s.check_path(
            &path,
            Attribution {
                route: Some("initial_pose".into()),
                ..Default::default()
            },
            false,
            cancel,
        )?;
        s.failed = false;
        Ok(s)
    }
    fn calibration(&self, id: &str) -> Result<&ToolCalibration, String> {
        self.prepared
            .model
            .tools
            .iter()
            .find(|t| t.tool == id)
            .ok_or_else(|| format!("undeclared machine tool {id}"))
    }
    fn bodies(&self) -> Result<Vec<Placed>, String> {
        let state = self.active.as_deref().unwrap_or("");
        let mut bodies = vec![];
        for (id, variants) in &self.prepared.bodies {
            let variant = if variants.len() == 1
                && self
                    .prepared
                    .model
                    .bodies
                    .iter()
                    .find(|b| &b.id == id)
                    .unwrap()
                    .indexed_solids
                    .is_empty()
            {
                &variants[0]
            } else {
                variants
                    .iter()
                    .find(|b| b.id == state)
                    .ok_or("indexed body state missing")?
            };
            let mut b = variant.clone();
            b.id = id.clone();
            bodies.push(b);
        }
        if let Some(id) = &self.active {
            bodies.push(Placed {
                id: "@tool".into(),
                shapes: self
                    .tool_shapes
                    .get(id)
                    .ok_or("machine tool shapes missing")?
                    .clone(),
                coefficients: self.prepared.model.coefficients(Mount::Head),
                workpiece: false,
                tip: true,
                enclosure_only: true,
            });
        }
        Ok(bodies)
    }
    fn allowed(&self, a: &str, b: &str) -> bool {
        let mut p = [a.to_string(), b.to_string()];
        p.sort();
        self.prepared.allowed.contains(&p)
    }
    fn finding(
        &mut self,
        status: Status,
        kind: &str,
        bodies: [String; 2],
        fraction: f64,
        detail: String,
        at: &Attribution,
    ) -> Result<(), String> {
        if status == Status::Clear {
            return Ok(());
        }
        if self.report.findings.len() >= self.prepared.model.options.max_findings {
            return Err("machine finding cap reached; program check incomplete".into());
        }
        if status == Status::Violation || self.report.status == Status::Clear {
            self.report.status = status;
        }
        self.report.findings.push(Finding {
            status,
            kind: kind.into(),
            bodies,
            motion_id: at.motion,
            event_index: at.event,
            route: at.route.clone(),
            fraction,
            detail,
        });
        Ok(())
    }
    fn error(&self, a: &Placed, b: &Placed, path: &Path) -> Result<[Vec3; 2], String> {
        let diff = sub(b.coefficients, a.coefficients);
        let w = f64::from(b.workpiece) - f64::from(a.workpiece);
        let tip = f64::from(b.tip) - f64::from(a.tip);
        let cw = if path.absolute {
            [w; 3]
        } else {
            add(diff, [w; 3])
        };
        let cg = if path.absolute {
            [-tip; 3]
        } else {
            sub(diff, [tip; 3])
        };
        let mut e = interval_scaled(
            cw,
            [
                self.prepared.model.work_error_min_mm,
                self.prepared.model.work_error_max_mm,
            ],
        );
        if let Some(t) = &self.active {
            e = interval_add(e, interval_scaled(cg, gauge_error(self.calibration(t)?)));
        }
        let p = std::array::from_fn(|k| diff[k].abs() * path.error[k]);
        Ok([sub(e[0], p), add(e[1], p)])
    }
    fn contact(
        &self,
        a: &Placed,
        b: &Placed,
        sa: &Shape,
        sb: &Shape,
        path: &Path,
        work: &mut Budget<'_, impl CancelSignal>,
    ) -> Result<Contact, String> {
        let pair = Pair::new(sa, sb, work)?;
        let c = sub(b.coefficients, a.coefficients);
        let map = |q: Vec3| std::array::from_fn(|k| q[k] * c[k]);
        let e = self.error(a, b, path)?;
        let o = &self.prepared.model.options;
        if path.dogleg {
            let lo = std::array::from_fn(|k| (c[k] * path.from[k]).min(c[k] * path.to[k]));
            let hi = std::array::from_fn(|k| (c[k] * path.from[k]).max(c[k] * path.to[k]));
            let mid = mul(add(lo, hi), 0.5);
            let e = [add(e[0], sub(lo, mid)), add(e[1], sub(hi, mid))];
            let mut hit = pair.linear(mid, mid, e, o.tolerance_mm, work)?;
            if hit.status != Status::Clear {
                hit.status = Status::Uncertain;
            }
            Ok(hit)
        } else if path.curve.is_none() {
            pair.linear(map(path.from), map(path.to), e, o.tolerance_mm, work)
        } else {
            pair.curved(
                |t| Ok(map(path.position(t)?)),
                path.speed,
                e,
                o.tolerance_mm,
                o.spatial_resolution_mm,
                work,
            )
        }
    }
    /// Machine-axis travel of one path, by the one travel rule
    /// (`crate::travel`, docs/python.md "The travel rule"): the axis travel
    /// is closed, a declared position is exact up to the evaluation bound,
    /// and no tolerance enters the decision.
    ///
    /// A decoded move's axis position is its transformed position plus the
    /// tool's expected gauge offset, known to the declared calibration
    /// interval (work frame and gauge offset): inside only if the whole
    /// interval is inside, a violation only if the whole interval is beyond
    /// a limit, uncertain otherwise. An axis no posted block has commanded
    /// since a declared machine-axis position is still there exactly, with
    /// no interval (`Path::exact_start`, `Session::chunk`). A route between declared machine-axis
    /// positions has no interval, so it is inside or a violation. A decoded
    /// arc's nominal circle is known to its endpoint gap either side, as in
    /// the posted-output check.
    ///
    /// Every axis is extreme at an end of the path or where an arc's
    /// coordinate is stationary (an independent-axis rapid moves each axis
    /// monotonically between its ends), so those points decide the path.
    fn travel(
        &mut self,
        path: &Path,
        at: &Attribution,
        work: &mut Budget<'_, impl CancelSignal>,
    ) -> Result<(), String> {
        use crate::travel::{ArcGap, Calibration, Travel};
        let m = &self.prepared.model;
        let travel = Travel::tip(
            std::array::from_fn(|k| m.axes[k].min_mm),
            std::array::from_fn(|k| m.axes[k].max_mm),
        );
        work.tick()?;
        let found = if let Some(curve) = &path.work {
            let mut e = [m.work_error_min_mm, m.work_error_max_mm];
            if let Some(t) = &self.active {
                e = interval_add(e, gauge_error(self.calibration(t)?));
            }
            let calibration = Calibration {
                offset_mm: path.gauge,
                low_mm: e[0],
                high_mm: e[1],
            };
            // An axis no block has commanded since a declared machine-axis
            // position is still there exactly: the interval is between
            // commanded work coordinates and the axes, so it enters an axis
            // only with a command (`travel::curve_calibrated_exact`).
            crate::travel::curve_calibrated_exact(
                &path.frame,
                &travel,
                &calibration,
                curve,
                ArcGap::Executed,
                path.exact_start,
                path.exact_rest,
            )?
        } else {
            // Declared machine-axis positions: exact, no calibration.
            [(0., path.from), (1., path.to)]
                .into_iter()
                .find_map(|(fraction, axes)| {
                    crate::travel::axes_outside(&travel, axes).map(|mut found| {
                        found.fraction = fraction;
                        found
                    })
                })
        };
        self.travel_finding(found, at)
    }
    /// The machine axes the active tool's position commands to the decoded
    /// work position without moving (a block whose words equal it): as a
    /// decoded move's end, the interval on every axis not still `exact`.
    fn travel_point(
        &mut self,
        at: &Attribution,
        work: &mut Budget<'_, impl CancelSignal>,
    ) -> Result<(), String> {
        use crate::travel::{Calibration, Travel};
        let Some(t) = self.active.clone() else {
            return Ok(());
        };
        let m = &self.prepared.model;
        let travel = Travel::tip(
            std::array::from_fn(|k| m.axes[k].min_mm),
            std::array::from_fn(|k| m.axes[k].max_mm),
        );
        work.tick()?;
        let c = self.calibration(&t)?;
        let e = interval_add([m.work_error_min_mm, m.work_error_max_mm], gauge_error(c));
        let calibration = Calibration {
            offset_mm: gauge(c),
            low_mm: e[0],
            high_mm: e[1],
        };
        let found = crate::travel::point_calibrated_exact(
            &m.work_to_machine,
            &travel,
            &calibration,
            self.replay.position(),
            self.exact,
        );
        self.travel_finding(found, at)
    }
    fn travel_finding(
        &mut self,
        found: Option<crate::travel::Outside>,
        at: &Attribution,
    ) -> Result<(), String> {
        let m = &self.prepared.model;
        let Some(found) = found else {
            return Ok(());
        };
        let k = found.axis;
        let name = ["X", "Y", "Z"][k];
        let (min, max) = (m.axes[k].min_mm, m.axes[k].max_mm);
        let side = if found.low { "minimum" } else { "maximum" };
        let [low, high] = found.interval_mm;
        let gap = found.nominal_gap_mm.map_or(String::new(), |gap| {
            format!(
                "; a point of a decoded arc's nominal circle, which the executed path may differ from by its endpoint gap {gap:.3e} mm"
            )
        });
        let (status, detail) = if found.certain {
            (
                Status::Violation,
                format!(
                    "axis {name} at {:.6} mm (calibration interval {low:.6} to {high:.6} mm) exceeds declared [{min}, {max}] machine travel: the whole interval is beyond the {side} {:.6} mm, by at least {:.3e} mm{gap}",
                    found.machine_mm[k], found.limit_mm, found.beyond_mm
                ),
            )
        } else {
            (
                Status::Uncertain,
                if low == high {
                    // No calibration interval: only a decoded arc's gap.
                    format!(
                        "axis {name} at {low:.6} mm is not shown inside the {side} {:.6} mm of declared [{min}, {max}] machine travel{gap}",
                        found.limit_mm
                    )
                } else {
                    format!(
                        "axis {name} calibration interval {low:.6} to {high:.6} mm (expected position {:.6} mm) straddles the {side} {:.6} mm of declared [{min}, {max}] machine travel: not shown inside{gap}",
                        found.machine_mm[k], found.limit_mm
                    )
                },
            )
        };
        self.finding(
            status,
            "machine_travel",
            [
                format!("axis_{}", ["x", "y", "z"][k]),
                "travel_limit".into(),
            ],
            found.fraction,
            detail,
            at,
        )
    }
    fn check_path(
        &mut self,
        path: &Path,
        at: Attribution,
        cutting: bool,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        let mut work = Budget {
            remaining: self.prepared.model.options.max_axis_tests,
            cancel,
        };
        self.travel(path, &at, &mut work)?;
        let bodies = self.bodies()?;
        self.check_bodies(&bodies, path, &at, cutting, &mut work)
    }
    fn check_bodies(
        &mut self,
        bodies: &[Placed],
        path: &Path,
        at: &Attribution,
        cutting: bool,
        work: &mut Budget<'_, impl CancelSignal>,
    ) -> Result<(), String> {
        for (i, a) in bodies.iter().enumerate() {
            for b in &bodies[i + 1..] {
                if self.allowed(&a.id, &b.id) {
                    continue;
                }
                let mut result = None;
                for sa in &a.shapes {
                    for sb in &b.shapes {
                        let mut hit = self.contact(a, b, sa, sb, path, work)?;
                        if hit.status == Status::Violation && (a.enclosure_only || b.enclosure_only)
                        {
                            hit.status = Status::Uncertain;
                        }
                        if hit.status != Status::Clear
                            && result.is_none_or(|r: Contact| r.status != Status::Violation)
                        {
                            result = Some(hit);
                        }
                    }
                }
                if let Some(hit) = result {
                    self.finding(
                        hit.status,
                        "machine_contact",
                        [a.id.clone(), b.id.clone()],
                        hit.fraction,
                        format!(
                            "continuous body overlap; guaranteed penetration bound {} mm{}",
                            hit.margin_mm,
                            if a.enclosure_only || b.enclosure_only {
                                " (outer tool/index enclosure: physical contact unproven)"
                            } else {
                                ""
                            }
                        ),
                        at,
                    )?;
                }
            }
            let native_tool_stock = a.id == "@tool"
                && cutting
                && self
                    .active
                    .as_ref()
                    .is_some_and(|t| self.calibration(t).is_ok_and(|t| t.body_solids.is_empty()));
            if !self.allowed(&a.id, "@stock") && !native_tool_stock {
                self.check_stock(a, path, at, work)?;
            }
        }
        Ok(())
    }
    /// For a decoded move that only lifts the tool along the tool axis (#412):
    /// the work Z of the tool's lowest point where the move starts, plus the
    /// stock replay's own vertical guard on a floor it cut (a cell cut flat
    /// at Z keeps the bound Z + `SweepOptions.tolerance_mm`). A height-field
    /// cell whose upper bound is at or below it is the floor the tool already
    /// stood on: the move touches it only within the calibration interval
    /// and that guard, as the move that brought the tool there did, so it is
    /// decided clear.
    ///
    /// Why: each cell bounds its material by the column from the stock bottom
    /// up to its upper bound. The move translates the tool rigidly by +Z
    /// only (work and machine XY are unchanged, and the work frame's Z is the
    /// machine Z exactly), and the calibration interval is the same box at
    /// every instant, so every pose the tool may take during the move is a
    /// pose it may take at the start, raised. The column's top therefore
    /// never comes nearer the tool than at the start, where it is no higher
    /// than the guard above the tool's lowest point: any overlap is the one
    /// the tool had where the move began, a band at its bottom no deeper than
    /// the interval and the guard, which is the end of the move that brought
    /// it there and was checked (or cut) as that move. Everything else is
    /// checked as before: a cell above that (material beside or above the
    /// tool's start), any XY change, an arc, a route between declared
    /// machine positions, a rotated work frame or a lathe.
    fn floor_rise(&self, body: &Placed, path: &Path) -> Option<f64> {
        let f = &self.prepared.model.work_to_machine;
        if !body.tip
            || path.curve.is_some()
            || f.x[2] != 0.
            || f.y[2] != 0.
            || f.z != [0., 0., 1.]
            || path.from[0] != path.to[0]
            || path.from[1] != path.to[1]
        {
            return None;
        }
        let Curve::Line { start, end } = *path.work.as_ref()?.curve() else {
            return None;
        };
        if start[0] != end[0]
            || start[1] != end[1]
            || end[2].partial_cmp(&start[2]) != Some(std::cmp::Ordering::Greater)
        {
            return None;
        }
        let low = *self.tool_low_z.get(self.active.as_deref()?)?;
        // Never above the exact sum (a tip-low tool adds nothing).
        let z = if low == 0. {
            start[2]
        } else {
            (start[2] + low).next_down()
        };
        let z = z + self.floor_guard_mm;
        z.is_finite().then_some(z)
    }
    fn check_stock(
        &mut self,
        body: &Placed,
        path: &Path,
        at: &Attribution,
        work: &mut Budget<'_, impl CancelSignal>,
    ) -> Result<(), String> {
        let m = &self.prepared.model;
        let stock_body = |shape: Shape| Placed {
            id: "@stock".into(),
            shapes: vec![shape.transformed(
                |p| transform(&m.work_to_machine, p),
                |p| vector(&m.work_to_machine, p),
            )],
            coefficients: m.coefficients(Mount::Workpiece),
            workpiece: true,
            tip: false,
            enclosure_only: true,
        };
        let mut cells = vec![];
        match self.replay.run_stock() {
            // A tri-dexel run's height view is a valid height-field bound of
            // its material: its cells enclose the stock as a height field's do.
            RunStock::Height(_) | RunStock::Dexel(_) => {
                let stock = self.replay.run_stock().height()?;
                crate::resources::check_memory(
                    stock.cells().len() as u128 * 96,
                    "machine stock enclosures",
                )?;
                let spec = stock.spec();
                let broad = stock_body(Shape::aabb(spec.min_mm, spec.max_mm));
                if body
                    .shapes
                    .iter()
                    .map(|sa| self.contact(body, &broad, sa, &broad.shapes[0], path, work))
                    .collect::<Result<Vec<_>, _>>()?
                    .iter()
                    .all(|c| c.status == Status::Clear)
                {
                    return Ok(());
                }
                let (nx, _, _) = spec.grid()?;
                let widths = stock.cell_widths_mm();
                let floor = self.floor_rise(body, path);
                for (i, h) in stock.cells().iter().enumerate() {
                    work.tick()?;
                    if h.upper_mm <= spec.min_mm[2] || floor.is_some_and(|z| h.upper_mm <= z) {
                        continue;
                    }
                    let lo = [
                        spec.min_mm[0] + (i % nx) as f64 * widths[0],
                        spec.min_mm[1] + (i / nx) as f64 * widths[1],
                        spec.min_mm[2],
                    ];
                    let hi = [
                        (lo[0] + widths[0]).min(spec.max_mm[0]),
                        (lo[1] + widths[1]).min(spec.max_mm[1]),
                        h.upper_mm,
                    ];
                    cells.push((lo, hi, Some(h.lower_mm)));
                }
            }
            RunStock::Turn(stock) => {
                for (i, c) in stock.cells().iter().enumerate() {
                    work.tick()?;
                    let [z0, z1] = stock.bin(i);
                    let r = c.outer.upper_mm;
                    if r > c.inner.lower_mm {
                        cells.push(([-r, -r, z0], [r, r, z1], None));
                    }
                }
            }
        }
        let mut result = None;
        for (lo, hi, lower) in cells {
            let b = stock_body(Shape::aabb(lo, hi));
            for shape in &body.shapes {
                let mut hit = self.contact(body, &b, shape, &b.shapes[0], path, work)?;
                if hit.status == Status::Clear {
                    continue;
                }
                hit.status = Status::Uncertain;
                if !body.enclosure_only
                    && let Some(z) = lower
                    && z > lo[2]
                {
                    let b = stock_body(Shape::aabb(lo, [hi[0], hi[1], z]));
                    let inner = self.contact(body, &b, shape, &b.shapes[0], path, work)?;
                    if inner.status == Status::Violation {
                        hit = inner;
                    }
                }
                if result.is_none_or(|r: Contact| r.status != Status::Violation) {
                    result = Some(hit);
                }
            }
            if result.is_some_and(|c| c.status == Status::Violation) {
                break;
            }
        }
        if let Some(hit) = result {
            self.finding(hit.status,"machine_stock_contact",[body.id.clone(),"@stock".into()],hit.fraction,"body swept into evolving stock; outer bounds that overlap are refused even when exact physical contact is unresolved".into(),at)?;
        }
        Ok(())
    }
    /// One decoded event, without the posted blocks it came from. Which axes
    /// the blocks commanded is then unknown: every axis is taken to carry
    /// the calibration interval from here on, at both ends of every move
    /// (`chunk` holds axes no block commanded exactly).
    pub fn event(
        &mut self,
        event: Event,
        event_index: usize,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        if !(self.failed || self.finished) {
            self.exact = [None; 3];
        }
        self.dispatch(event, event_index, [true; 3], cancel)
    }
    /// The work axes a posted block may command: those whose word address
    /// appears in it outside a comment. Every decoder reads a block only
    /// before its first `;` and, where the dialect makes them comments,
    /// outside parentheses (`Decoder::read_full`, the Haas and Carvera
    /// readers, `post_decode`), and an axis moves to a work position only by
    /// its own word or by a decoded move along it. An address anywhere else
    /// (in a keyword, an unrelated word) only counts an axis as commanded,
    /// which is never less safe. Unbalanced parentheses, which no decoder
    /// accepts, or an undeclared dialect count every axis.
    fn axis_words(&self, text: &str) -> [bool; 3] {
        let Some(letters) = self.axis_letters else {
            return [!text.trim().is_empty(); 3];
        };
        let body = text.split(';').next().unwrap_or("");
        let mut depth = 0usize;
        let mut words = [false; 3];
        for b in body.bytes() {
            match b {
                b'(' if self.parentheses_comments => depth += 1,
                b')' if self.parentheses_comments => {
                    if depth == 0 {
                        return [true; 3];
                    }
                    depth -= 1;
                }
                _ if depth == 0 => {
                    let b = b.to_ascii_uppercase();
                    for k in 0..3 {
                        words[k] |= b == letters[k];
                    }
                }
                _ => {}
            }
        }
        if depth != 0 { [true; 3] } else { words }
    }
    /// The work axes a posted block may command: none for a block the
    /// independent decoder read as setting an offset or a mode without
    /// moving any axis (`Line::holds_axes`: its axis letters are parameters,
    /// such as the shift of `G52 X0 Y0 Z0`), else `axis_words`. Only the
    /// decoder's own reading holds a block; any other block that names an
    /// axis commands it.
    fn line_words(&self, line: &crate::post::Line) -> [bool; 3] {
        if line.holds_axes {
            [false; 3]
        } else {
            self.axis_words(&line.text)
        }
    }
    /// The machine axes a command of the `work` axes may move: those whose
    /// row of the rigid work-to-machine frame depends on one of them.
    fn affected(&self, work: [bool; 3]) -> [bool; 3] {
        let f = &self.prepared.model.work_to_machine;
        std::array::from_fn(|k| {
            let row = [f.x[k], f.y[k], f.z[k]];
            (0..3).any(|j| work[j] && row[j] != 0.)
        })
    }
    /// Blocks that may command the `work` axes: the machine axes they may
    /// move are no longer at a declared position. Without a decoded move the
    /// machine still goes to the commanded work position, the decoded one:
    /// with `check`, an axis that leaves its declared position there is held
    /// to travel with its interval (a following move's start would hold the
    /// same point, so a block before a move in its own record skips it).
    fn commanded(
        &mut self,
        work: [bool; 3],
        check: bool,
        at: &Attribution,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        let moved = self.affected(work);
        let mut left = false;
        for (exact, moved) in self.exact.iter_mut().zip(moved) {
            if moved && exact.is_some() {
                *exact = None;
                left = true;
            }
        }
        if left && check {
            let mut budget = Budget {
                remaining: self.prepared.model.options.max_axis_tests,
                cancel,
            };
            if let Err(e) = self.travel_point(at, &mut budget) {
                self.failed = true;
                return Err(e);
            }
        }
        Ok(())
    }
    /// One retained posted chunk: its decoded records in order, with the
    /// posted blocks they came from (`Chunk.lines`, by `line_interval`).
    ///
    /// The calibration interval is an unknown constant between commanded
    /// work coordinates and machine axes. A machine axis last set by a
    /// declared machine-axis position (the start pose, a route waypoint)
    /// that no block has since commanded is at that position exactly; a
    /// block that writes an axis word commands that axis to a work position,
    /// even one equal to the decoded current position (that block decodes
    /// to no move, yet the machine goes to the commanded position, which the
    /// interval places), so it is held with the interval from then on. A
    /// block the decoder read as moving no axis (`line_words`: `G52 X0 Y0
    /// Z0` and the like) writes no axis word.
    ///
    /// For a decoded move: its start is exact on every axis still exact
    /// before its record's blocks (except the last one, which may be the
    /// move's own block, its words reaching only the end) and its control
    /// effects; the rest of it carries the interval on every axis its
    /// record's blocks name or the move changes in work coordinates (an arc
    /// or helix always its plane's two axes), mapped to machine axes through
    /// the frame. A record whose lines are not all in `lines` counts every
    /// axis. A block between records, or a record without a decoded move,
    /// commands the axes it names at the decoded position, which is then
    /// held to travel with the interval on them.
    pub fn chunk(
        &mut self,
        lines: &[crate::post::Line],
        checked: &[crate::post::Checked],
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        if self.failed || self.finished {
            return Err("machine session is failed or finished".into());
        }
        let mut next = 0;
        for check in checked {
            let at = Attribution {
                event: Some(check.event_index),
                ..Default::default()
            };
            let [start, end] = check.line_interval;
            let (before, all) = if check.cycle_event_index.is_some() || end < start {
                ([false; 3], [false; 3])
            } else {
                while next < lines.len() && lines[next].number < start {
                    let words = self.line_words(&lines[next]);
                    let between = Attribution {
                        event: self.last_event,
                        ..Default::default()
                    };
                    self.commanded(words, true, &between, cancel)?;
                    next += 1;
                }
                let mut own = vec![];
                let mut complete = true;
                while next < lines.len() && lines[next].number <= end {
                    complete &= lines[next].number == start + own.len();
                    own.push(self.line_words(&lines[next]));
                    next += 1;
                }
                if !complete || own.len() != end - start + 1 {
                    ([true; 3], [true; 3])
                } else {
                    let union = |words: &[[bool; 3]]| {
                        words
                            .iter()
                            .fold([false; 3], |a, w| std::array::from_fn(|k| a[k] | w[k]))
                    };
                    (union(&own[..own.len() - 1]), union(&own))
                }
            };
            for event in check.control_effects.clone().unwrap_or_default() {
                self.dispatch(event, check.event_index, all, cancel)?;
            }
            match &check.motion {
                Some(motion) => {
                    self.commanded(before, false, &at, cancel)?;
                    self.dispatch(
                        Event::Motion {
                            motion: motion.clone(),
                        },
                        check.event_index,
                        all,
                        cancel,
                    )?
                }
                None => self.commanded(all, true, &at, cancel)?,
            }
        }
        while next < lines.len() {
            let words = self.line_words(&lines[next]);
            let between = Attribution {
                event: self.last_event,
                ..Default::default()
            };
            self.commanded(words, true, &between, cancel)?;
            next += 1;
        }
        Ok(())
    }
    /// `commanded`: the work axes the event's posted blocks may command
    /// (for a move, besides those it changes).
    fn dispatch(
        &mut self,
        event: Event,
        event_index: usize,
        commanded: [bool; 3],
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        if self.failed || self.finished {
            return Err("machine session is failed or finished".into());
        }
        self.failed = true;
        // One source cycle can have several checked decoded motions; order is nondecreasing.
        if self.last_event.is_some_and(|i| event_index < i) {
            return Err("machine decoded events are out of order".into());
        }
        self.last_event = Some(event_index);
        let at = Attribution {
            event: Some(event_index),
            ..Default::default()
        };
        match event {
            Event::Motion { motion } => self.motion(motion, at, commanded, cancel)?,
            Event::ToolChange {
                tool,
                length_register,
                position_after_mm,
                ..
            } => {
                if self.calibration(&tool)?.length_register != length_register {
                    return Err(
                        "decoded tool length register differs from machine calibration".into(),
                    );
                }
                self.change_tool(&tool, position_after_mm, at, cancel)?;
            }
            Event::WorkOffset { number } => {
                if number != self.prepared.model.work_offset {
                    return Err("machine crash checking requires a separate model/setup for each work offset".into());
                }
            }
            Event::CutterCompensation { side, .. }
                if side != crate::motion::CompensationSide::Off =>
            {
                return Err("machine clearance under controller cutter compensation (wear, or the controller's whole kerf) is unproven; use computer compensation with declared geometry or explicitly accept uncertainty".into());
            }
            Event::Beam { .. }
            | Event::BeamStart { .. }
            | Event::BeamMark { .. }
            | Event::HeightControl { .. } => {
                return Err(
                    "machine crash model covers milling and turning, not thermal machinery".into(),
                );
            }
            _ => {}
        }
        self.failed = false;
        Ok(())
    }
    fn motion(
        &mut self,
        motion: Motion,
        mut at: Attribution,
        commanded: [bool; 3],
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        // The work axes this move commands: those its blocks name, those it
        // changes, and an arc's or helix's plane axes (its extremes are on
        // commanded coordinates even where an end coincides).
        let (start, end) = (motion.curve.start(), motion.curve.end());
        let mut command: [bool; 3] = std::array::from_fn(|j| commanded[j] || start[j] != end[j]);
        if let Curve::Arc { plane, .. } | Curve::Helix { plane, .. } = &motion.curve {
            let [u, v, _] = plane.axes();
            command[u] = true;
            command[v] = true;
        }
        let moved = self.affected(command);
        let exact_start = self.exact;
        let exact_rest: [Option<f64>; 3] =
            std::array::from_fn(|k| exact_start[k].filter(|_| !moved[k]));
        if self.active.as_deref() != Some(&motion.tool) {
            return Err("decoded motion has no matching active machine tool".into());
        }
        let m = &self.prepared.model;
        let g = gauge(self.calibration(&motion.tool)?);
        let curve = motion.curve.prepare(m.options.tolerance_mm)?;
        let from = add(transform(&m.work_to_machine, motion.curve.start()), g);
        let to = add(transform(&m.work_to_machine, motion.curve.end()), g);
        if (0..3).any(|k| (self.axes[k] - from[k]).abs() > m.options.tolerance_mm) {
            return Err(
                "decoded motion is not contiguous with machine axes after its declared route"
                    .into(),
            );
        }
        at.motion = Some(motion.id);
        let dogleg = motion.kind.is_rapid()
            && motion.feed_mm_min.is_none()
            && m.rapid_mode == RapidMode::AxisIndependent;
        if dogleg && !matches!(motion.curve, Curve::Line { .. }) {
            return Err("independent-axis rapid must decode as a line endpoint command".into());
        }
        let error = crate::verification::path_error(&motion, m.options.tolerance_mm);
        let b = curve.bounds();
        let coordinate_error = std::array::from_fn(|k| {
            // A stationary commanded coordinate is exact, including a lathe's
            // fixed Y. Endpoint/circle tolerance cannot invent motion on it.
            let row = [
                m.work_to_machine.x[k],
                m.work_to_machine.y[k],
                m.work_to_machine.z[k],
            ];
            if (0..3).all(|j| row[j] == 0. || b.min[j] == b.max[j]) {
                0.
            } else {
                error
            }
        });
        let path = Path {
            from,
            to,
            speed: curve.length_mm()
                * m.work_to_machine
                    .x
                    .iter()
                    .chain(&m.work_to_machine.y)
                    .chain(&m.work_to_machine.z)
                    .map(|v| v * v)
                    .sum::<f64>()
                    .sqrt()
                    .next_up()
                * 1.00000001,
            work: Some(curve.clone()),
            curve: if matches!(motion.curve, Curve::Line { .. }) {
                None
            } else {
                Some(curve)
            },
            frame: m.work_to_machine.clone(),
            gauge: g,
            dogleg,
            absolute: false,
            error: coordinate_error,
            exact_start,
            exact_rest,
        };
        // The native rotational assembly/stock checker handles exact cutter
        // engagement rules. A dogleg rapid instead checks its full XYZ envelope.
        let milling = m.process == Process::Milling;
        if milling && !dogleg {
            let check = crate::stock_contact::check(
                self.replay.stock()?,
                &self.tools[&motion.tool],
                &motion,
                m.options.stock_contact,
                cancel,
            )?;
            for f in check.findings {
                self.finding(
                    f.finding.status,
                    "tool_stock_contact",
                    ["@tool".into(), "@stock".into()],
                    0.,
                    serde_json::to_string(&f).map_err(|e| e.to_string())?,
                    &at,
                )?;
            }
            if !check.fully_examined {
                self.finding(
                    Status::Uncertain,
                    "tool_stock_incomplete",
                    ["@tool".into(), "@stock".into()],
                    0.,
                    "native continuous stock check incomplete".into(),
                    &at,
                )?;
            }
        }
        // Skip tool/stock only where the exact milling or turning kernel checks it.
        self.check_path(&path, at, !dogleg, cancel)?;
        if milling {
            self.replay.push_checked_motion(motion, LIMIT, cancel)?;
        } else {
            self.replay.push_turn_replayed(motion, LIMIT, cancel)?;
        }
        self.axes = to;
        self.exact = exact_rest;
        self.report.decoded_motions += 1;
        Ok(())
    }
    fn waypoints(
        &mut self,
        points: &[[Option<f64>; 3]],
        at: &Attribution,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        for p in points {
            let to = std::array::from_fn(|k| p[k].unwrap_or(self.axes[k]));
            let path = Path::absolute(
                self.axes,
                to,
                self.prepared.model.rapid_mode == RapidMode::AxisIndependent,
            );
            self.check_path(&path, at.clone(), false, cancel)?;
            self.axes = to;
            // A declared waypoint puts an axis at that machine position; an
            // axis it retains stays as it was.
            for (exact, declared) in self.exact.iter_mut().zip(p) {
                if declared.is_some() {
                    *exact = *declared;
                }
            }
        }
        Ok(())
    }
    fn change_tool(
        &mut self,
        tool: &str,
        tip: Vec3,
        mut at: Attribution,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        let r = self
            .prepared
            .model
            .tool_change_routes
            .get(tool)
            .ok_or("decoded tool has no declared complete route")?
            .clone();
        at.route = Some(format!("tool_change:{tool}:before_index"));
        self.waypoints(&r.before_index_mm, &at, cancel)?;
        at.route = Some(format!("tool_change:{tool}:index"));
        let mut work = Budget {
            remaining: self.prepared.model.options.max_axis_tests,
            cancel,
        };
        let mut bodies = self.bodies()?;
        bodies.retain(|b| !r.replaces.contains(&b.id));
        let envelope = Placed {
            id: "@index".into(),
            shapes: r
                .index_envelopes
                .iter()
                .map(|s| s.prepare(self.prepared.model.options.tolerance_mm, &mut work))
                .collect::<Result<Vec<_>, _>>()?,
            coefficients: self.prepared.model.coefficients(Mount::Head),
            workpiece: false,
            tip: false,
            enclosure_only: true,
        };
        // Reject an envelope that contradicts even an endpoint assembly. A
        // complete sweep between endpoints remains an explicit caller premise.
        let old = self.bodies()?;
        let saved = self.active.clone();
        self.active = Some(tool.into());
        let new = self.bodies();
        self.active = saved.clone();
        for (state, bodies) in [(saved, old), (Some(tool.into()), new?)] {
            for b in bodies {
                if !r.replaces.contains(&b.id) {
                    continue;
                }
                let error = if b.tip {
                    let e = gauge_error(
                        self.calibration(state.as_deref().ok_or("index endpoint tool missing")?)?,
                    );
                    [mul(e[1], -1.), mul(e[0], -1.)]
                } else {
                    [[0.; 3]; 2]
                };
                for shape in &b.shapes {
                    if !envelope.shapes.iter().any(|s| {
                        s.contains(
                            shape,
                            [0.; 3],
                            error,
                            self.prepared.model.options.tolerance_mm,
                        )
                    }) {
                        return Err(format!(
                            "index envelope for {tool} does not enclose endpoint body {} with calibration margins",
                            b.id
                        ));
                    }
                }
            }
        }
        // Envelope shares only contacts explicitly allowed for every body it replaces.
        let mut retained = vec![];
        for b in &bodies {
            if !r.replaces.iter().all(|id| self.allowed(id, &b.id)) {
                retained.push(b.clone());
            }
        }
        retained.push(envelope);
        let path = Path::absolute(self.axes, self.axes, false);
        self.check_bodies(&retained, &path, &at, false, &mut work)?;
        self.active = Some(tool.into());
        at.route = Some(format!("tool_change:{tool}:after_index"));
        self.waypoints(&r.after_index_mm, &at, cancel)?;
        let expected = add(
            transform(&self.prepared.model.work_to_machine, tip),
            gauge(self.calibration(tool)?),
        );
        if (0..3)
            .any(|k| (self.axes[k] - expected[k]).abs() > self.prepared.model.options.tolerance_mm)
        {
            return Err(format!(
                "declared route for {tool} does not end at decoded post-change tip/gauge position"
            ));
        }
        self.replay.change_tool(tool)?;
        if (0..3).any(|k| {
            (self.replay.position()[k] - tip[k]).abs() > self.prepared.model.options.tolerance_mm
        }) {
            return Err("decoded change differs from bound stock-replay tool-change pose".into());
        }
        let path = Path::absolute(self.axes, self.axes, false);
        self.check_path(&path, at, false, cancel)?;
        self.report.tool_changes += 1;
        Ok(())
    }
    /// Keep definite findings when a later budget, cancellation or route
    /// failure leaves the remainder unproven. Uncertainty never downgrades a
    /// collision already established in an earlier decoded prefix.
    pub fn incomplete(&self, reason: String) -> Report {
        let mut report = self.report.clone();
        report.complete = false;
        if report.status == Status::Clear {
            report.status = Status::Uncertain;
        }
        report.findings.push(Finding {
            status: Status::Uncertain,
            kind: "machine_check_incomplete".into(),
            bodies: ["machine".into(), "program".into()],
            motion_id: None,
            event_index: self.last_event,
            route: None,
            fraction: 0.,
            detail: reason,
        });
        report
    }
    pub fn finish(
        &mut self,
        expected_motions: u64,
        cancel: &impl CancelSignal,
    ) -> Result<&Report, String> {
        if self.failed || self.finished {
            return Err("machine session is failed or finished".into());
        }
        self.failed = true;
        if expected_motions != self.report.decoded_motions {
            return Err("machine check did not cover every decoded posted motion".into());
        }
        let route = self.prepared.model.program_end_route_mm.clone();
        self.waypoints(
            &route,
            &Attribution {
                route: Some("program_end".into()),
                ..Default::default()
            },
            cancel,
        )?;
        self.report.complete = true;
        self.finished = true;
        self.failed = false;
        Ok(&self.report)
    }
}
