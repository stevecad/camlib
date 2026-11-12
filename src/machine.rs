//! Declared machine crash checking of decoded programs (#116).
//!
//! Physical dimensions, calibration bounds, axis ownership and controller
//! routes are supplied by the caller. This module never controls a machine.
//! Convex solids translate continuously; rotating index mechanisms require
//! caller-declared complete swept envelopes, not sampled rotary poses (#2).
mod geometry;
mod session;
pub use session::Session;

use crate::{
    motion::{Vec3, WorkFrame},
    stock::CancelSignal,
    verification::Status,
};
pub use geometry::Solid;
use geometry::{Budget, Shape, add, point, sub};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Process {
    Milling,
    Turning,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Drive {
    Head,
    Table,
    Fixed,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Mount {
    Fixed,
    Head,
    Table,
    Workpiece,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum RapidMode {
    Linear,
    AxisIndependent,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Axis {
    pub drive: Drive,
    pub min_mm: f64,
    pub max_mm: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Body {
    pub id: String,
    pub component: String,
    pub mount: Mount,
    pub solids: Vec<Solid>,
    /// All shapes for an indexed body, keyed by selected tool ("" is empty).
    /// Empty means the geometry does not change at tool selection.
    pub indexed_solids: BTreeMap<String, Vec<Solid>>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ToolCalibration {
    pub tool: String,
    pub length_register: u32,
    /// Complete physical tool-body enclosure relative to the controlled point.
    /// Empty uses the native assembly. Required for curved face-grooving tools
    /// whose axisymmetric stock envelope does not locate their body in 3D.
    pub body_solids: Vec<Solid>,
    /// Expected correctly calibrated gauge-to-controlled-point offset. The
    /// controller's register and physical tool are assumed to agree within
    /// this declared range; actual setup mistakes cannot be measured here.
    pub gauge_offset_min_mm: Vec3,
    pub gauge_offset_max_mm: Vec3,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AllowedContact {
    pub bodies: [String; 2],
    pub reason: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ToolRoute {
    pub reference: String,
    /// Absolute machine-axis waypoints. None explicitly retains that axis.
    pub before_index_mm: Vec<[Option<f64>; 3]>,
    /// Complete enclosure of all changing bodies and both tool assemblies
    /// throughout indexing, in the carrier's zero-axis frame.
    pub index_envelopes: Vec<Solid>,
    pub replaces: Vec<String>,
    pub after_index_mm: Vec<[Option<f64>; 3]>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Options {
    pub tolerance_mm: f64,
    pub spatial_resolution_mm: f64,
    pub max_axis_tests: u64,
    pub max_findings: usize,
    pub stock_contact: crate::stock_contact::Options,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Model {
    pub version: u32,
    pub units: String,
    pub id: String,
    pub revision: String,
    pub process: Process,
    pub axes: [Axis; 3],
    pub work_offset: u8,
    pub work_to_machine: WorkFrame,
    /// Calibration uncertainty of the common work/fixture/stock frame.
    pub work_error_min_mm: Vec3,
    pub work_error_max_mm: Vec3,
    pub initial_axes_mm: Vec3,
    pub initial_tool: Option<String>,
    pub rapid_mode: RapidMode,
    pub controller_reference: String,
    pub tools: Vec<ToolCalibration>,
    pub bodies: Vec<Body>,
    /// Explicit absence, with a reason, for components the machine lacks.
    pub absent_components: BTreeMap<String, String>,
    pub allowed_contacts: Vec<AllowedContact>,
    pub tool_change_routes: BTreeMap<String, ToolRoute>,
    pub program_end_route_mm: Vec<[Option<f64>; 3]>,
    pub options: Options,
}

#[derive(Clone, Debug, Serialize)]
pub struct Finding {
    pub status: Status,
    pub kind: String,
    pub bodies: [String; 2],
    pub motion_id: Option<u64>,
    pub event_index: Option<usize>,
    pub route: Option<String>,
    pub fraction: f64,
    pub detail: String,
}
#[derive(Clone, Debug, Serialize)]
pub struct Report {
    pub algorithm: &'static str,
    pub model_sha256: String,
    pub status: Status,
    pub complete: bool,
    pub decoded_motions: u64,
    pub tool_changes: u64,
    pub findings: Vec<Finding>,
    pub allowed_contacts: Vec<AllowedContact>,
    pub scope: Vec<String>,
}

fn range_ok(a: Vec3, b: Vec3) -> bool {
    point(a) && point(b) && (0..3).all(|k| a[k] <= 0. && b[k] >= 0.)
}
fn text(s: &str) -> bool {
    !s.trim().is_empty() && s.len() <= 4096
}
fn gauge(t: &ToolCalibration) -> Vec3 {
    std::array::from_fn(|k| (t.gauge_offset_min_mm[k] + t.gauge_offset_max_mm[k]) / 2.)
}
fn gauge_error(t: &ToolCalibration) -> [Vec3; 2] {
    let g = gauge(t);
    [sub(t.gauge_offset_min_mm, g), sub(t.gauge_offset_max_mm, g)]
}
fn interval_scaled(v: Vec3, r: [Vec3; 2]) -> [Vec3; 2] {
    std::array::from_fn(|s| {
        std::array::from_fn(|k| v[k] * r[if v[k] >= 0. { s } else { 1 - s }][k])
    })
}
fn interval_add(a: [Vec3; 2], b: [Vec3; 2]) -> [Vec3; 2] {
    [add(a[0], b[0]), add(a[1], b[1])]
}
fn vector(frame: &WorkFrame, p: Vec3) -> Vec3 {
    std::array::from_fn(|k| frame.x[k] * p[0] + frame.y[k] * p[1] + frame.z[k] * p[2])
}
fn transform(frame: &WorkFrame, p: Vec3) -> Vec3 {
    add(vector(frame, p), frame.origin_mm)
}

#[derive(Clone)]
struct Placed {
    id: String,
    shapes: Vec<Shape>,
    coefficients: Vec3,
    workpiece: bool,
    tip: bool,
    enclosure_only: bool,
}
pub struct Prepared {
    model: Model,
    bodies: BTreeMap<String, Vec<Placed>>,
    allowed: BTreeSet<[String; 2]>,
    digest: String,
}
impl Model {
    pub fn prepare(&self, cancel: &impl CancelSignal) -> Result<Prepared, String> {
        let o = &self.options;
        if self.version != 1
            || self.units != "mm"
            || !crate::tool::valid_id(&self.id)
            || !text(&self.revision)
            || !(1..=9).contains(&self.work_offset)
            || !text(&self.controller_reference)
            || !o.tolerance_mm.is_finite()
            || !(1e-7..=0.1).contains(&o.tolerance_mm)
            || !o.spatial_resolution_mm.is_finite()
            || !(o.tolerance_mm..=1.).contains(&o.spatial_resolution_mm)
            || !(1..=100_000_000).contains(&o.max_axis_tests)
            || !(1..=1024).contains(&o.max_findings)
        {
            return Err("machine model needs version 1, mm, identities, work offset, controller reference and finite positive checking budgets".into());
        }
        self.work_to_machine.validate()?;
        if self.work_to_machine.z != [0., 0., 1.]
            || self.process == Process::Turning
                && (self.work_to_machine.x != [1., 0., 0.]
                    || self.work_to_machine.y != [0., 1., 0.])
        {
            return Err("machine model supports upright three-axis milling and XZ turning; tilted/rotary kinematics require #2/#5".into());
        }
        if !range_ok(self.work_error_min_mm, self.work_error_max_mm) || !point(self.initial_axes_mm)
        {
            return Err("machine calibration error bounds must be finite and enclose zero".into());
        }
        for (k, a) in self.axes.iter().enumerate() {
            if !a.min_mm.is_finite()
                || !a.max_mm.is_finite()
                || a.min_mm.abs() > 1e6
                || a.max_mm.abs() > 1e6
                || a.min_mm > a.max_mm
                || (a.drive == Drive::Fixed) != (a.min_mm == a.max_mm)
            {
                return Err(format!(
                    "machine axis {} needs ordered travel and an initial position inside it; fixed axes have equal limits",
                    ["X", "Y", "Z"][k]
                ));
            }
        }
        // The start position by the travel rule (`crate::travel`): a declared
        // machine-axis position, compared exactly; on a limit is inside.
        if let Some(found) = crate::travel::axes_outside(
            &crate::travel::Travel::tip(
                std::array::from_fn(|k| self.axes[k].min_mm),
                std::array::from_fn(|k| self.axes[k].max_mm),
            ),
            self.initial_axes_mm,
        ) {
            return Err(format!(
                "machine axis {} needs ordered travel and an initial position inside it; fixed axes have equal limits",
                ["X", "Y", "Z"][found.axis]
            ));
        }
        o.stock_contact.validate()?;
        let mut tool_ids = BTreeSet::new();
        for t in &self.tools {
            if self.process == Process::Milling && !t.body_solids.is_empty() {
                return Err("milling body_solids must be empty: physical assemblies come from ToolAssembly; model spindle accessories as machine bodies".into());
            }
            if !crate::tool::valid_id(&t.tool)
                || !tool_ids.insert(t.tool.clone())
                || t.length_register == 0
                || !point(t.gauge_offset_min_mm)
                || !point(t.gauge_offset_max_mm)
                || (0..3).any(|k| t.gauge_offset_min_mm[k] > t.gauge_offset_max_mm[k])
            {
                return Err("machine tool calibrations need unique tool IDs, registers and finite ordered gauge bounds".into());
            }
            if !self.tool_change_routes.contains_key(&t.tool) {
                return Err(format!(
                    "machine tool {} needs a complete declared tool-change route",
                    t.tool
                ));
            }
        }
        if self.tools.is_empty()
            || self
                .initial_tool
                .as_ref()
                .is_some_and(|t| !tool_ids.contains(t))
            || self
                .tool_change_routes
                .keys()
                .any(|t| !tool_ids.contains(t))
        {
            return Err("machine model has missing or foreign tool calibration/route".into());
        }
        let components: &[&str] = match self.process {
            Process::Milling => &["spindle_head", "column", "table", "enclosure"],
            Process::Turning => &[
                "turret",
                "other_tools",
                "chuck",
                "tailstock",
                "quill",
                "steady_rest",
                "bed",
                "enclosure",
            ],
        };
        let present: BTreeSet<_> = self.bodies.iter().map(|b| b.component.as_str()).collect();
        for c in components {
            if present.contains(c) == self.absent_components.contains_key(*c) {
                return Err(format!(
                    "machine component {c} must be modelled or explicitly declared absent, once"
                ));
            }
        }
        if self
            .absent_components
            .iter()
            .any(|(k, v)| !components.contains(&k.as_str()) || !text(v))
        {
            return Err("unknown or unexplained absent machine component".into());
        }
        if self.process == Process::Milling && !present.contains("spindle_head")
            || self.process == Process::Turning && !present.contains("chuck")
        {
            return Err("machine must include its spindle/head or lathe chuck geometry".into());
        }
        let mut ids = BTreeSet::new();
        let mut work = Budget {
            remaining: o.max_axis_tests,
            cancel,
        };
        let mut bodies = BTreeMap::new();
        for b in &self.bodies {
            if !crate::tool::valid_id(&b.id)
                || b.id.starts_with('@')
                || !ids.insert(b.id.clone())
                || b.solids.is_empty() && b.indexed_solids.is_empty()
            {
                return Err("machine bodies need unique IDs and nonempty solid geometry".into());
            }
            if !components.contains(&b.component.as_str())
                && !["fixture", "protected_target", "auxiliary", "tool_carrier"]
                    .contains(&b.component.as_str())
            {
                return Err(format!("unknown machine body component {}", b.component));
            }
            if !b.indexed_solids.is_empty()
                && tool_ids
                    .iter()
                    .chain(std::iter::once(&String::new()))
                    .any(|id| !b.indexed_solids.contains_key(id))
            {
                return Err(format!(
                    "indexed body {} needs every tool state and the empty state",
                    b.id
                ));
            }
            let mut variants = vec![];
            let states: Vec<_> = if b.indexed_solids.is_empty() {
                vec![(String::new(), &b.solids)]
            } else {
                b.indexed_solids
                    .iter()
                    .map(|(k, v)| (k.clone(), v))
                    .collect()
            };
            for (state, extra) in states {
                let mut shapes = vec![];
                let parts: Vec<_> = if b.indexed_solids.is_empty() {
                    extra.iter().collect()
                } else {
                    b.solids.iter().chain(extra).collect()
                };
                if parts.is_empty() {
                    return Err(format!(
                        "indexed machine body {} has an empty geometry state",
                        b.id
                    ));
                }
                for s in parts {
                    let shape = s.prepare(o.tolerance_mm, &mut work)?;
                    shapes.push(if b.mount == Mount::Workpiece {
                        shape.transformed(
                            |p| transform(&self.work_to_machine, p),
                            |p| vector(&self.work_to_machine, p),
                        )
                    } else {
                        shape
                    });
                }
                variants.push(Placed {
                    id: state,
                    shapes,
                    coefficients: self.coefficients(b.mount),
                    workpiece: b.mount == Mount::Workpiece,
                    tip: false,
                    enclosure_only: false,
                });
            }
            bodies.insert(b.id.clone(), variants);
        }
        ids.insert("@tool".into());
        ids.insert("@stock".into());
        let mut allowed = BTreeSet::new();
        for a in &self.allowed_contacts {
            let mut pair = a.bodies.clone();
            pair.sort();
            if pair[0] == pair[1]
                || pair.iter().any(|id| !ids.contains(id))
                || !text(&a.reason)
                || pair == ["@stock".to_string(), "@tool".to_string()]
                || !allowed.insert(pair)
            {
                return Err("allowed contacts must name distinct existing bodies with a reason; tool/stock contact is checked by the cutting kernel".into());
            }
        }
        for (id, r) in &self.tool_change_routes {
            if !text(&r.reference)
                || r.index_envelopes.is_empty()
                || !r.replaces.iter().any(|id| id == "@tool")
                || r.replaces
                    .iter()
                    .any(|id| !ids.contains(id) || id == "@stock")
                || r.replaces.iter().collect::<BTreeSet<_>>().len() != r.replaces.len()
            {
                return Err(format!(
                    "tool-change route {id} needs a reference, complete index envelopes and explicit replaced bodies including @tool"
                ));
            }
            for s in &r.index_envelopes {
                s.prepare(o.tolerance_mm, &mut work)?;
            }
            for p in r.before_index_mm.iter().chain(&r.after_index_mm) {
                validate_waypoint(p)?;
            }
            for b in &self.bodies {
                if r.replaces.contains(&b.id) && b.mount != Mount::Head {
                    return Err(format!("route {id} may only replace head-mounted bodies"));
                }
                if !b.indexed_solids.is_empty() && !r.replaces.contains(&b.id) {
                    return Err(format!("route {id} must enclose indexed body {}", b.id));
                }
            }
        }
        for p in &self.program_end_route_mm {
            validate_waypoint(p)?;
        }
        Ok(Prepared {
            model: self.clone(),
            bodies,
            allowed,
            digest: crate::verification::identity(self)?,
        })
    }
    fn coefficients(&self, mount: Mount) -> Vec3 {
        std::array::from_fn(|k| match mount {
            Mount::Fixed => 0.,
            Mount::Head => {
                if self.axes[k].drive == Drive::Table {
                    0.
                } else {
                    1.
                }
            }
            Mount::Table | Mount::Workpiece => {
                if self.axes[k].drive == Drive::Table {
                    -1.
                } else {
                    0.
                }
            }
        })
    }
}
fn validate_waypoint(p: &[Option<f64>; 3]) -> Result<(), String> {
    if p.iter().flatten().any(|v| !v.is_finite() || v.abs() > 1e6) || p.iter().all(Option::is_none)
    {
        Err(
            "machine route waypoint needs finite axis positions and at least one commanded axis"
                .into(),
        )
    } else {
        Ok(())
    }
}
