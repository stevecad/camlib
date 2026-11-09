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
    frame: WorkFrame,
    gauge: Vec3,
    speed: f64,
    dogleg: bool,
    absolute: bool,
    error: Vec3,
}
impl Path {
    fn absolute(from: Vec3, to: Vec3, dogleg: bool) -> Self {
        Self {
            from,
            to,
            curve: None,
            frame: WorkFrame::identity(),
            gauge: [0.; 3],
            speed: sub(to, from).into_iter().map(f64::abs).sum(),
            dogleg,
            absolute: true,
            error: [0.; 3],
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
    active: Option<String>,
    axes: Vec3,
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
            let g = gauge(c);
            let shapes = shapes
                .into_iter()
                .map(|s| s.transformed(|v| sub(v, g), |v| v))
                .collect();
            tools.insert(t.id.clone(), p);
            tool_shapes.insert(t.id.clone(), shapes);
        }
        manifest.verification = None;
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
        let mut s = Self {
            axes: model.initial_axes_mm,
            active: model.initial_tool.clone(),
            prepared,
            replay,
            tools,
            tool_shapes,
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
    fn travel(
        &mut self,
        path: &Path,
        at: &Attribution,
        work: &mut Budget<'_, impl CancelSignal>,
    ) -> Result<(), String> {
        let m = &self.prepared.model;
        let mut e = if path.absolute {
            [[0.; 3]; 2]
        } else {
            [m.work_error_min_mm, m.work_error_max_mm]
        };
        if !path.absolute
            && let Some(t) = &self.active
        {
            e = interval_add(e, gauge_error(self.calibration(t)?));
        }
        e = [sub(e[0], path.error), add(e[1], path.error)];
        let mut bad = vec![];
        for k in 0..3 {
            let axis = &m.axes[k];
            let tol = m.options.tolerance_mm;
            let speed = if path.from[k] == path.to[k] && path.error[k] == 0. {
                0.
            } else {
                path.speed
            };
            let mut stack = vec![(0., 1.)];
            let mut outcome = None;
            while let Some((a, b)) = stack.pop() {
                work.tick()?;
                let t = (a + b) / 2.;
                let p = path.position(t)?[k];
                let (lo, hi) = if path.curve.is_none() {
                    (
                        path.from[k].min(path.to[k]) + e[0][k],
                        path.from[k].max(path.to[k]) + e[1][k],
                    )
                } else {
                    (
                        p + e[0][k] - speed * (b - a) / 2.,
                        p + e[1][k] + speed * (b - a) / 2.,
                    )
                };
                if lo >= axis.min_mm && hi <= axis.max_mm {
                    continue;
                }
                let witness = [a, t, b]
                    .into_iter()
                    .map(|t| path.position(t).map(|p| (t, p[k])))
                    .collect::<Result<Vec<_>, _>>()?
                    .into_iter()
                    .find(|(_, p)| {
                        p + e[1][k] < axis.min_mm - tol || p + e[0][k] > axis.max_mm + tol
                    });
                if let Some((t, p)) = witness {
                    outcome = Some((
                        Status::Violation,
                        t,
                        format!(
                            "axis {} at {p} mm exceeds declared [{}, {}] machine travel",
                            ["X", "Y", "Z"][k],
                            axis.min_mm,
                            axis.max_mm
                        ),
                    ));
                    break;
                }
                if path.curve.is_none()
                    || speed * (b - a) <= m.options.spatial_resolution_mm
                    || t == a
                    || t == b
                {
                    outcome = Some((
                        Status::Uncertain,
                        t,
                        format!(
                            "axis {} calibration/path enclosure overlaps travel boundary",
                            ["X", "Y", "Z"][k]
                        ),
                    ));
                    break;
                }
                stack.push((t, b));
                stack.push((a, t));
            }
            if let Some(v) = outcome {
                bad.push((k, v));
            }
        }
        for (k, (status, t, detail)) in bad {
            self.finding(
                status,
                "machine_travel",
                [
                    format!("axis_{}", ["x", "y", "z"][k]),
                    "travel_limit".into(),
                ],
                t,
                detail,
                at,
            )?;
        }
        Ok(())
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
            RunStock::Height(stock) => {
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
                for (i, h) in stock.cells().iter().enumerate() {
                    work.tick()?;
                    if h.upper_mm <= spec.min_mm[2] {
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
    pub fn event(
        &mut self,
        event: Event,
        event_index: usize,
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
            Event::Motion { motion } => self.motion(motion, at, cancel)?,
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
                return Err("machine clearance under controller wear compensation is unproven; use computer compensation with declared geometry or explicitly accept uncertainty".into());
            }
            Event::Beam { .. } | Event::HeightControl { .. } => {
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
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
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
