//! Independent continuous prism/assembly checks. No planner-fit or stock-sweep
//! calls. Clear intervals require separation bounds, never just sampled endpoints.
use crate::tool::ToolAssembly;
use crate::{
    motion::{Motion, PreparedCurve, Vec3, WorkFrame},
    planar::{Boundary, ConstructionOptions, PlanarRegion, PreparedRegion},
    stock::CancelSignal,
    tool::{CutterShape, PreparedAssembly, valid_id},
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::HashSet;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BodyRole {
    Fixture,
    ProtectedTarget,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Representation {
    #[default]
    SolidPrism,
    OuterEnvelope,
}
impl Representation {
    fn is_solid(value: &Self) -> bool {
        *value == Self::SolidPrism
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Prism {
    pub id: String,
    pub role: BodyRole,
    pub footprint: PlanarRegion,
    pub z_min_mm: f64,
    pub z_max_mm: f64,
    pub z_error_mm: f64,
    #[serde(default, skip_serializing_if = "Representation::is_solid")]
    pub representation: Representation,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MachineLimits {
    /// Travel of one tool's tip (machine coordinates): `min_tip_mm` and
    /// `max_tip_mm` together, or neither. It is the travel of a tool of one
    /// gauge length, so with tools of different lengths declare the gauge
    /// line's travel (`min_gauge_mm`/`max_gauge_mm`) instead. Exactly one of
    /// the two boxes is declared.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub min_tip_mm: Option<Vec3>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_tip_mm: Option<Vec3>,
    pub max_feed_mm_min: f64,
    /// Optional limit on nominal curve normal acceleration at declared feed.
    /// No junction, tangential/axis acceleration or controller timing model.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_normal_accel_mm_s2: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_axis_feed_mm_min: Option<Vec3>,
    /// Travel of the spindle gauge line (machine coordinates): the machine's
    /// own axis travel. Controllers hold soft limits on the machine position,
    /// whatever tool length offset is active (LinuxCNC INI `[JOINT_n]`/
    /// `[AXIS_L]` `MIN_LIMIT`/`MAX_LIMIT`; FANUC stored stroke check 1,
    /// parameters 1320/1321, which the builder sets to the maximum stroke
    /// and which checks one fixed point whatever the tool length: FANUC
    /// 16i/18i-MB Operator's Manual B-63534EN/02, section 6.3, p. 855-856), and the
    /// length offset moves the controlled point from the gauge line to the
    /// tip (G43). So each tool's tip travels that box lowered along machine Z
    /// by the tool's gauge length
    /// (`ToolAssembly::gauge_length_mm`), so every tool is checked against
    /// its own tip travel (#176). Needs the work Z axis along machine Z (an
    /// upright spindle) and tools held in the spindle (not a beam).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub min_gauge_mm: Option<Vec3>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_gauge_mm: Option<Vec3>,
}

/// The travel of one tool's tip in machine coordinates: the declared tip box,
/// or the gauge-line box lowered by the tool's gauge length.
struct TipTravel<'a> {
    travel: crate::travel::Travel,
    /// `Some` when derived from the gauge line: the tool and its gauge length.
    gauge: Option<(&'a str, f64)>,
}

impl MachineLimits {
    /// The declared travel box: `(min, max, on the gauge line)`.
    fn travel(&self) -> Result<(Vec3, Vec3, bool), String> {
        match (
            self.min_tip_mm,
            self.max_tip_mm,
            self.min_gauge_mm,
            self.max_gauge_mm,
        ) {
            (Some(min), Some(max), None, None) => Ok((min, max, false)),
            (None, None, Some(min), Some(max)) => Ok((min, max, true)),
            _ => Err("machine travel is declared once: either the tip box of one tool length (min_tip_mm and max_tip_mm) or the spindle gauge line's box (min_gauge_mm and max_gauge_mm), which gives every tool its own tip travel from its gauge length".into()),
        }
    }
    fn tip_travel<'a>(&self, tool: &'a ToolAssembly) -> Result<TipTravel<'a>, String> {
        let (min, max, gauge_line) = self.travel()?;
        if !gauge_line {
            return Ok(TipTravel {
                travel: crate::travel::Travel::tip(min, max),
                gauge: None,
            });
        }
        if tool.cutter.shape == CutterShape::Beam {
            return Err(format!(
                "gauge-line machine travel gives a spindle tool's tip travel from its gauge length; beam tool {} has none: declare the head's tip travel (min_tip_mm and max_tip_mm)",
                tool.id
            ));
        }
        let length = tool.gauge_length_mm;
        Ok(TipTravel {
            travel: crate::travel::Travel::gauge_line(min, max, length),
            gauge: Some((&tool.id, length)),
        })
    }
}

impl TipTravel<'_> {
    fn violation(&self, found: &crate::travel::Outside) -> String {
        let Some((tool, length)) = self.gauge else {
            return "Tip exceeds configured travel after rigid work-to-machine transform".into();
        };
        let k = found.axis;
        format!(
            "Tool {tool} (gauge length {length} mm) tip at machine {axis} {at:.6} mm is {side} its travel limit {limit:.6} mm: the gauge line's {axis} limit{offset}",
            axis = ["X", "Y", "Z"][k],
            at = found.machine_mm[k],
            limit = found.limit_mm,
            side = if found.low { "below" } else { "above" },
            offset = if k == 2 {
                " less the tool's gauge length"
            } else {
                ""
            },
        )
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Scene {
    pub version: u32,
    pub units: String,
    pub id: String,
    pub revision: String,
    pub work_to_machine: WorkFrame,
    pub bodies: Vec<Prism>,
    pub machine: Option<MachineLimits>,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Options {
    pub tolerance_mm: f64,
    /// Explicit machining allowance at protected prism tops, for fed cutter
    /// profiles only. None retains strict contact detection everywhere.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub target_floor_tolerance_mm: Option<f64>,
    pub spatial_resolution_mm: f64,
    pub max_nodes: u64,
    pub max_boundary_tests: u64,
    pub max_findings: usize,
}
impl Default for Options {
    fn default() -> Self {
        Self {
            tolerance_mm: 1e-5,
            target_floor_tolerance_mm: None,
            spatial_resolution_mm: 0.001,
            max_nodes: 100_000,
            max_boundary_tests: 10_000_000,
            max_findings: 1024,
        }
    }
}
impl Options {
    pub(crate) fn validate(self) -> Result<(), String> {
        if !self.tolerance_mm.is_finite()
            || !(1e-7..=0.1).contains(&self.tolerance_mm)
            || self
                .target_floor_tolerance_mm
                .is_some_and(|v| !v.is_finite() || !(1e-7..=1.).contains(&v))
            || !self.spatial_resolution_mm.is_finite()
            || !(self.tolerance_mm..=1.).contains(&self.spatial_resolution_mm)
            || !(1..=1_000_000).contains(&self.max_nodes)
            || !(1..=1_000_000_000).contains(&self.max_boundary_tests)
            || !(1..=4096).contains(&self.max_findings)
        {
            return Err("invalid bounded verification resolution/work/finding options".into());
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Status {
    Clear,
    Violation,
    Uncertain,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ToolPart {
    Cutter,
    Neck,
    Shank,
    Holder,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FindingKind {
    FixtureContact,
    Gouge,
    MachineTravel,
    FeedLimit,
    NormalAccelerationLimit,
    AxisFeedLimit,
    StockContact,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Witness {
    pub fraction: f64,
    pub tip_mm: Vec3,
    pub section_height_mm: Option<f64>,
    /// A strict overlap/out-of-travel margin after engineering uncertainty.
    pub margin_lower_mm: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Finding {
    pub status: Status,
    pub kind: FindingKind,
    pub object_id: String,
    pub part: Option<ToolPart>,
    pub holder_section: Option<usize>,
    /// Search enclosure, NOT an assertion that every point in it collides.
    pub fraction_interval: [f64; 2],
    pub witness: Option<Witness>,
    pub detail: String,
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Stats {
    pub nodes: u64,
    pub boundary_tests: u64,
    pub pruned_intervals: u64,
    pub max_stack_nodes: usize,
    pub pairs_checked: usize,
    pub unexamined_pairs: usize,
    pub budget_exhausted: bool,
    pub findings_truncated: bool,
}
#[derive(Debug, Serialize, Deserialize)]
pub struct Coverage {
    pub configured_prism_bodies: usize,
    #[serde(default, skip_serializing_if = "is_zero")]
    pub enclosure_only_bodies: usize,
    pub machine_tip_travel_examined: bool,
    pub declared_fed_rate_checked: bool,
    #[serde(default, skip_serializing_if = "is_false")]
    pub nominal_normal_accel_checked: bool,
    #[serde(default, skip_serializing_if = "is_false")]
    pub nominal_axis_feed_checked: bool,
    pub evolving_stock_contact: bool,
    pub remaining_material: bool,
    pub controller_execution: bool,
}
fn is_zero(value: &usize) -> bool {
    *value == 0
}
fn is_false(value: &bool) -> bool {
    !*value
}
#[derive(Debug, Serialize, Deserialize)]
pub struct Report {
    pub scene_id: String,
    pub revision: String,
    pub scene_sha256: String,
    pub tool_sha256: String,
    pub motion_sha256: String,
    pub motion_id: u64,
    pub operation: String,
    pub tool: String,
    pub status: Status,
    pub coverage: Coverage,
    pub findings: Vec<Finding>,
    pub stats: Stats,
    pub numerical_policy: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub target_floor_tolerance_mm: Option<f64>,
}

struct Body {
    definition: Prism,
    region: PreparedRegion,
    segments: u64,
    guard: f64,
    outer_circle: Option<([f64; 2], f64)>,
    hole_disks: Vec<([f64; 2], f64)>,
}
/// A protected region checked part by part: one prepared scene per
/// component (a solid prism of it), each with conservative XY bounds. A
/// motion is checked against every component its tool can reach; the
/// clearance from the region is the least over its components, and a
/// component beyond the tool's reach cannot touch it, so the result is that
/// of the whole region at once, without the work of the distant parts.
pub struct ComponentScenes {
    scenes: Vec<([[f64; 2]; 2], PreparedScene)>,
    slack_mm: f64,
}
/// The result over the reached components: the first one not proven clear
/// (if any) and the work spent.
pub struct ComponentCheck {
    pub unclear: Option<Report>,
    pub nodes: u64,
    pub boundary_tests: u64,
}
impl ComponentScenes {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        id: &str,
        revision: &str,
        body_id: &str,
        region: &crate::planar::PlanarRegion,
        z_mm: [f64; 2],
        options: Options,
    ) -> Result<Self, String> {
        let scenes = region
            .components
            .iter()
            .map(|c| {
                let scene = Scene {
                    version: 1,
                    units: "mm".into(),
                    id: id.into(),
                    revision: revision.into(),
                    work_to_machine: WorkFrame::identity(),
                    machine: None,
                    bodies: vec![Prism {
                        id: body_id.into(),
                        role: BodyRole::ProtectedTarget,
                        representation: Representation::SolidPrism,
                        footprint: crate::planar::PlanarRegion {
                            version: region.version,
                            units: region.units.clone(),
                            name: region.name.clone(),
                            boundary_error_mm: region.boundary_error_mm,
                            components: vec![c.clone()],
                            source: region.source.clone(),
                        },
                        z_min_mm: z_mm[0],
                        z_max_mm: z_mm[1],
                        z_error_mm: 0.,
                    }],
                }
                .prepare(options)?;
                Ok((boundary_bounds(&c.outer), scene))
            })
            .collect::<Result<_, String>>()?;
        Ok(Self {
            scenes,
            // Boundary error, tolerances and a millimetre of slack.
            slack_mm: region.boundary_error_mm + options.tolerance_mm + 1.,
        })
    }
    pub fn verify_motion(
        &self,
        tool: &PreparedAssembly,
        motion: &Motion,
        cancel: &impl CancelSignal,
    ) -> Result<ComponentCheck, String> {
        self.verify_motion_except(tool, motion, None, cancel)
    }
    /// `verify_motion` against every component but `skip` (by its index
    /// in the region).
    pub fn verify_motion_except(
        &self,
        tool: &PreparedAssembly,
        motion: &Motion,
        skip: Option<usize>,
        cancel: &impl CancelSignal,
    ) -> Result<ComponentCheck, String> {
        let t = tool.definition();
        let c = &t.cutter;
        let widest = t
            .holders
            .iter()
            .map(|h| h.radius_min_mm.max(h.radius_max_mm))
            .chain([c.diameter_mm / 2., c.shank_diameter_mm / 2.])
            .chain(c.neck.as_ref().map(|n| n.diameter_mm / 2.))
            .fold(0., f64::max);
        let reach = widest + self.slack_mm;
        let bounds = motion.curve.prepare(1e-6)?.bounds();
        let mut out = ComponentCheck {
            unclear: None,
            nodes: 0,
            boundary_tests: 0,
        };
        for (i, (b, scene)) in self.scenes.iter().enumerate() {
            if Some(i) == skip
                || (0..2)
                    .any(|k| b[0][k] > bounds.max[k] + reach || b[1][k] < bounds.min[k] - reach)
            {
                continue;
            }
            let check = scene.verify_motion(tool, motion, cancel)?;
            out.nodes += check.stats.nodes;
            out.boundary_tests += check.stats.boundary_tests;
            if check.status != Status::Clear {
                out.unclear = Some(check);
                break;
            }
        }
        Ok(out)
    }
}

/// Bounds of a closed boundary: its vertices, and for each arc the whole
/// circle it lies on (conservative).
pub fn boundary_bounds(b: &crate::planar::Boundary) -> [[f64; 2]; 2] {
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    let n = b.vertices.len();
    for i in 0..n {
        let [x, y, bulge] = b.vertices[i];
        let [qx, qy, _] = b.vertices[(i + 1) % n];
        let mut grow = |p: [f64; 2], r: f64| {
            for k in 0..2 {
                lo[k] = lo[k].min(p[k] - r);
                hi[k] = hi[k].max(p[k] + r);
            }
        };
        grow([x, y], 0.);
        if bulge != 0. {
            let chord = (qx - x).hypot(qy - y);
            let f = (1. - bulge * bulge) / (4. * bulge);
            let c = [(x + qx) / 2. - f * (qy - y), (y + qy) / 2. + f * (qx - x)];
            let r = chord * (1. + bulge * bulge) / (4. * bulge.abs());
            grow(c, r * (1. + 1e-12) + 1e-9);
        }
    }
    [lo, hi]
}

pub struct PreparedScene {
    definition: Scene,
    bodies: Vec<Body>,
    options: Options,
    digest: String,
}
/// Identity of one motion: SHA-256 of its exact values in a fixed binary
/// layout (domain-separated). Cheaper than the canonical JSON identity and
/// computed the same way wherever a motion is bound to a check record.
pub(crate) fn motion_identity(m: &crate::motion::Motion) -> String {
    use crate::motion::{Curve, MoveKind, Plane};
    let mut h = Sha256::new();
    h.update(b"camlib-motion-identity-v1\0");
    h.update(m.id.to_le_bytes());
    for text in [&m.operation, &m.tool] {
        h.update((text.len() as u32).to_le_bytes());
        h.update(text.as_bytes());
    }
    h.update([match m.kind {
        MoveKind::Rapid => 0u8,
        MoveKind::Cut => 1,
        MoveKind::Plunge => 2,
        MoveKind::Lead => 3,
        MoveKind::Link => 4,
        MoveKind::Retract => 5,
        MoveKind::Tap => 6,
        MoveKind::Shift => 7,
        MoveKind::Thread => 8,
        MoveKind::ProbePosition => 9,
        MoveKind::ProbeMeasure => 10,
        MoveKind::ProbeTouch => 11,
        MoveKind::ProbeReturn => 12,
        MoveKind::Mark => 13,
    }]);
    match m.feed_mm_min {
        Some(f) => {
            h.update([1u8]);
            h.update(f.to_le_bytes());
        }
        None => h.update([0u8]),
    }
    let point = |h: &mut Sha256, p: crate::motion::Vec3| {
        for v in p {
            h.update(v.to_le_bytes());
        }
    };
    match m.curve {
        Curve::Line { start, end } => {
            h.update([0u8]);
            point(&mut h, start);
            point(&mut h, end);
        }
        Curve::Arc {
            start,
            end,
            center,
            plane,
            sweep_rad,
        }
        | Curve::Helix {
            start,
            end,
            center,
            plane,
            sweep_rad,
        } => {
            h.update([if matches!(m.curve, Curve::Arc { .. }) {
                1u8
            } else {
                2
            }]);
            point(&mut h, start);
            point(&mut h, end);
            point(&mut h, center);
            h.update([match plane {
                Plane::Xy => 0u8,
                Plane::Xz => 1,
                Plane::Yz => 2,
            }]);
            h.update(sweep_rad.to_le_bytes());
        }
    }
    format!("{:x}", h.finalize())
}
pub(crate) fn identity(value: &impl Serialize) -> Result<String, String> {
    // Stream canonical bytes into SHA rather than duplicating large CAD source
    // provenance in a temporary serialized scene/tool buffer.
    struct HashWriter(Sha256);
    impl std::io::Write for HashWriter {
        fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
            self.0.update(bytes);
            Ok(bytes.len())
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }
    let mut writer = HashWriter(Sha256::new());
    serde_json::to_writer(&mut writer, value).map_err(|e| format!("verification identity: {e}"))?;
    Ok(format!("{:x}", writer.0.finalize()))
}
impl Scene {
    pub fn prepare(&self, options: Options) -> Result<PreparedScene, String> {
        options.validate()?;
        self.work_to_machine.validate()?;
        if self.version != 1
            || self.units != "mm"
            || !valid_id(&self.id)
            || self.revision.is_empty()
            || self.revision.len() > 512
            || (self.bodies.is_empty() && self.machine.is_none())
        {
            return Err("verification scene needs version 1/mm, identity/revision and configured bounded checks".into());
        }
        if let Some(m) = &self.machine {
            let (min, max, gauge_line) = m.travel()?;
            if !point_ok(min) || !point_ok(max) || (0..3).any(|k| min[k] >= max[k]) {
                return Err(
                    "invalid machine tip travel, declared feed, axis feed or normal acceleration limit"
                        .into(),
                );
            }
            // The tip is below the gauge line along the spindle axis, machine
            // Z on an upright machine; a tilted work frame would put the tool
            // axis elsewhere, which this model does not describe.
            if gauge_line && self.work_to_machine.z != [0., 0., 1.] {
                return Err("gauge-line machine travel needs the work Z axis along machine Z (work_to_machine.z = [0, 0, 1]): the tip is the gauge line lowered by the gauge length along the spindle axis".into());
            }
        }
        if let Some(m) = &self.machine
            && (!m.max_feed_mm_min.is_finite()
                || !(1e-6..=1e7).contains(&m.max_feed_mm_min)
                || m.max_axis_feed_mm_min.is_some_and(|limits| {
                    limits
                        .iter()
                        .any(|v| !v.is_finite() || !(1e-6..=1e7).contains(v))
                })
                || m.max_normal_accel_mm_s2
                    .is_some_and(|a| !a.is_finite() || !(1e-6..=1e9).contains(&a)))
        {
            return Err(
                "invalid machine tip travel, declared feed, axis feed or normal acceleration limit"
                    .into(),
            );
        }
        let mut ids = HashSet::new();
        let mut bodies = Vec::with_capacity(self.bodies.len());
        let mut total = 0;
        for body in &self.bodies {
            if !valid_id(&body.id)
                || !ids.insert(body.id.as_str())
                || !body.z_min_mm.is_finite()
                || !body.z_max_mm.is_finite()
                || body.z_min_mm.abs() > 1e6
                || body.z_max_mm.abs() > 1e6
                || body.z_max_mm <= body.z_min_mm
                || !body.z_error_mm.is_finite()
                || !(0.0..=1.).contains(&body.z_error_mm)
                || body.footprint.components.is_empty()
            {
                return Err(
                    "prism needs unique ID, filled footprint, finite Z extent and explicit error"
                        .into(),
                );
            }
            let region = body.footprint.prepare(ConstructionOptions {
                tolerance_mm: options.tolerance_mm,
            })?;
            if body.role == BodyRole::ProtectedTarget
                && options
                    .target_floor_tolerance_mm
                    .is_some_and(|v| v >= body.z_max_mm - body.z_min_mm)
            {
                return Err(
                    "target floor tolerance must be smaller than the protected prism height".into(),
                );
            }
            let summary = region.summary();
            total += summary.segments;
            if total > 65_536 {
                return Err("verification scene exceeds 65536 boundary segments".into());
            }
            let outer_circle = enclosing_circle(region.definition());
            // Another component may be a filled island inside a hole.
            let hole_disks = if region.definition().components.len() == 1 {
                region.definition().components[0]
                    .holes
                    .iter()
                    .filter_map(|b| boundary_circle(b, true))
                    .collect()
            } else {
                Vec::new()
            };
            bodies.push(Body {
                definition: body.clone(),
                region,
                segments: summary.segments as u64,
                guard: summary.boundary_error_mm
                    + summary.numerical_guard_mm
                    + body.z_error_mm
                    + options.tolerance_mm,
                outer_circle,
                hole_disks,
            });
        }
        Ok(PreparedScene {
            definition: self.clone(),
            bodies,
            options,
            digest: identity(&(self, options))?,
        })
    }
}

fn point_ok(p: Vec3) -> bool {
    p.iter().all(|v| v.is_finite() && v.abs() <= 1e6)
}
/// Two same-direction minor semicircle-like arcs lie inside their shared
/// diameter circle, even when CAD bulges differ slightly from±1. Use only an
/// OUTER enclosure for separation; witnesses still query the original footprint.
fn enclosing_circle(region: &PlanarRegion) -> Option<([f64; 2], f64)> {
    if region.components.len() != 1 {
        return None;
    }
    boundary_circle(&region.components[0].outer, false)
}
fn boundary_circle(boundary: &Boundary, inner: bool) -> Option<([f64; 2], f64)> {
    let points = &boundary.vertices;
    if points.len() != 2
        || points
            .iter()
            .any(|p| p[2].abs() < 0.999_999 || p[2].abs() > 1.)
        || points[0][2].signum() != points[1][2].signum()
    {
        return None;
    }
    let (a, b) = (points[0], points[1]);
    Some((
        [a[0] + (b[0] - a[0]) / 2., a[1] + (b[1] - a[1]) / 2.],
        (b[0] - a[0]).hypot(b[1] - a[1]) / 2.
            * if inner {
                a[2].abs().min(b[2].abs())
            } else {
                1.
            },
    ))
}
/// Complete-circle enclosure is conservative for partial arcs and XY helices.
fn circle_axis_max_enclosure(
    motion: &Motion,
    path: &PreparedCurve,
    span: [f64; 2],
    c: [f64; 2],
) -> Option<f64> {
    use crate::motion::{Curve, Plane};
    let distance = |p: Vec3| (p[0] - c[0]).hypot(p[1] - c[1]);
    match motion.curve {
        Curve::Line { .. } => {
            Some(distance(path.position(span[0]).ok()?).max(distance(path.position(span[1]).ok()?)))
        }
        Curve::Arc {
            start,
            center,
            plane: Plane::Xy,
            ..
        }
        | Curve::Helix {
            start,
            center,
            plane: Plane::Xy,
            ..
        } => Some(distance(center) + (start[0] - center[0]).hypot(start[1] - center[1])),
        _ => None,
    }
}

/// Prepared whole-prefix axis geometry. Callers include endpoint/numeric error;
/// widened angular membership alone is not an exact upper-distance certificate.
pub(crate) enum AxisSpan {
    Line {
        start: [f64; 2],
        unit: [f64; 2],
        length: f64,
    },
    Circle {
        start: [f64; 2],
        end: [f64; 2],
        center: [f64; 2],
        radius: f64,
        angle: f64,
        sweep: f64,
        width: f64,
        angular_guard: f64,
    },
}
impl AxisSpan {
    pub(crate) fn new(motion: &Motion, path: &PreparedCurve, span: [f64; 2]) -> Option<Self> {
        use crate::motion::{Curve, Plane};
        let [a, b] = span;
        let p = path.position(a).ok()?;
        let q = path.position(b).ok()?;
        match motion.curve {
            Curve::Line { .. } => {
                let dx = q[0] - p[0];
                let dy = q[1] - p[1];
                let length = dx.hypot(dy);
                let unit = if length == 0. {
                    [0., 0.]
                } else {
                    [dx / length, dy / length]
                };
                Some(Self::Line {
                    start: [p[0], p[1]],
                    unit,
                    length,
                })
            }
            Curve::Arc {
                start,
                center,
                plane: Plane::Xy,
                sweep_rad,
                ..
            }
            | Curve::Helix {
                start,
                center,
                plane: Plane::Xy,
                sweep_rad,
                ..
            } => {
                let radius = (start[0] - center[0]).hypot(start[1] - center[1]);
                let angle = (start[1] - center[1]).atan2(start[0] - center[0]) + sweep_rad * a;
                Some(Self::Circle {
                    start: [p[0], p[1]],
                    end: [q[0], q[1]],
                    center: [center[0], center[1]],
                    radius,
                    angle,
                    sweep: sweep_rad,
                    width: sweep_rad.abs() * (b - a),
                    angular_guard: 256. * f64::EPSILON * (1. + angle.abs() + sweep_rad.abs()),
                })
            }
            _ => None,
        }
    }
    pub(crate) fn distance(&self, c: [f64; 2]) -> f64 {
        let distance = |p: [f64; 2]| (p[0] - c[0]).hypot(p[1] - c[1]);
        match *self {
            Self::Line {
                start,
                unit,
                length,
            } => {
                let t =
                    ((c[0] - start[0]) * unit[0] + (c[1] - start[1]) * unit[1]).clamp(0., length);
                distance([start[0] + t * unit[0], start[1] + t * unit[1]])
            }
            Self::Circle {
                start,
                end,
                center,
                radius,
                angle,
                sweep,
                width,
                angular_guard,
            } => {
                let d = distance(center);
                if d == 0. || width >= std::f64::consts::TAU {
                    return (radius - d).abs();
                }
                let target = (c[1] - center[1]).atan2(c[0] - center[0]);
                let travel = if sweep > 0. {
                    (target - angle).rem_euclid(std::f64::consts::TAU)
                } else {
                    (angle - target).rem_euclid(std::f64::consts::TAU)
                };
                if travel <= width + angular_guard
                    || std::f64::consts::TAU - travel <= angular_guard
                {
                    (radius - d).abs()
                } else {
                    distance(start).min(distance(end))
                }
            }
        }
    }
}
/// Minimum axis distance to a point over an entire line/XY circle/helix interval.
/// Endpoint/phase candidates are analytic; caller includes endpoint/numeric error.
pub(crate) fn circle_axis_distance(
    motion: &Motion,
    path: &PreparedCurve,
    span: [f64; 2],
    c: [f64; 2],
) -> Option<f64> {
    AxisSpan::new(motion, path, span).map(|axis| axis.distance(c))
}
#[derive(Clone, Copy)]
pub(crate) enum Radius {
    Constant(f64),
    Ball(f64),
    Bull(f64, f64),
    Cone {
        radius: f64,
        tip: f64,
        slope: f64,
    },
    Linear {
        z: f64,
        radius: f64,
        slope: f64,
    },
    /// A whole sphere of this radius from the tip (a lollipop cutter).
    Sphere(f64),
}
impl Radius {
    /// The largest radius over heights [lo, hi]: at an end, or at a
    /// sphere's equator when the interval holds it.
    pub(crate) fn max_over(self, lo: f64, hi: f64) -> f64 {
        match self {
            Self::Sphere(r) if lo <= r && r <= hi => r,
            _ => self.at(lo).max(self.at(hi)),
        }
    }
    pub(crate) fn at(self, z: f64) -> f64 {
        match self {
            Self::Constant(r) => r,
            Self::Ball(r) => {
                let z = z.min(r);
                (z * (2. * r - z)).max(0.).sqrt()
            }
            Self::Sphere(r) => {
                let z = z.clamp(0., 2. * r);
                (z * (2. * r - z)).max(0.).sqrt()
            }
            Self::Bull(r, c) => {
                let z = z.min(c);
                r - c + (z * (2. * c - z)).max(0.).sqrt()
            }
            Self::Cone { radius, tip, slope } => (tip + z * slope).min(radius),
            Self::Linear {
                z: a,
                radius,
                slope,
            } => radius + (z - a) * slope,
        }
    }
}
pub(crate) struct Part {
    pub(crate) kind: ToolPart,
    pub(crate) holder: Option<usize>,
    pub(crate) min: f64,
    pub(crate) max: f64,
    pub(crate) radius: Radius,
}
pub(crate) fn parts(tool: &PreparedAssembly) -> Result<Vec<Part>, String> {
    let t = tool.definition();
    if t.holders.len() > 1024 {
        return Err("verification tool exceeds 1024 holder sections".into());
    }
    let c = &t.cutter;
    let r = c.diameter_mm / 2.;
    let radius = match c.shape {
        CutterShape::Flat | CutterShape::Beam => Radius::Constant(r),
        CutterShape::Ball | CutterShape::Probe => Radius::Ball(r),
        CutterShape::Lollipop => Radius::Sphere(r),
        CutterShape::Dovetail { flank_angle_rad } => Radius::Linear {
            z: 0.,
            radius: r,
            slope: -1. / flank_angle_rad.tan(),
        },
        CutterShape::Bull { corner_radius_mm } => Radius::Bull(r, corner_radius_mm),
        CutterShape::Drill { point_angle_rad } => Radius::Cone {
            radius: r,
            tip: 0.,
            slope: (point_angle_rad / 2.).tan(),
        },
        CutterShape::Chamfer {
            included_angle_rad,
            tip_diameter_mm,
        } => Radius::Cone {
            radius: r,
            tip: tip_diameter_mm / 2.,
            slope: (included_angle_rad / 2.).tan(),
        },
    };
    let mut result = vec![Part {
        kind: ToolPart::Cutter,
        holder: None,
        min: 0.,
        max: c.cutting_length_mm,
        radius,
    }];
    let shoulder = c.cutting_length_mm + c.neck.as_ref().map_or(0., |n| n.length_mm);
    if let Some(n) = &c.neck {
        result.push(Part {
            kind: ToolPart::Neck,
            holder: None,
            min: c.cutting_length_mm,
            max: shoulder,
            radius: Radius::Constant(n.diameter_mm / 2.),
        });
    }
    if shoulder < t.stickout_mm {
        result.push(Part {
            kind: ToolPart::Shank,
            holder: None,
            min: shoulder,
            max: t.stickout_mm,
            radius: Radius::Constant(c.shank_diameter_mm / 2.),
        });
    }
    for (i, h) in t.holders.iter().enumerate() {
        result.push(Part {
            kind: ToolPart::Holder,
            holder: Some(i),
            min: h.z_min_mm,
            max: h.z_max_mm,
            radius: Radius::Linear {
                z: h.z_min_mm,
                radius: h.radius_min_mm,
                slope: (h.radius_max_mm - h.radius_min_mm) / (h.z_max_mm - h.z_min_mm),
            },
        });
    }
    Ok(result)
}

struct Work<'a, C> {
    stats: Stats,
    options: Options,
    cancel: &'a C,
}
impl<C: CancelSignal> Work<'_, C> {
    fn charge(&mut self, boundary: u64) -> Result<bool, String> {
        if self.cancel.is_cancelled() {
            return Err("verification cancelled; no clear result published".into());
        }
        if self.stats.nodes == self.options.max_nodes
            || self.stats.boundary_tests + boundary > self.options.max_boundary_tests
        {
            self.stats.budget_exhausted = true;
            return Ok(false);
        }
        self.stats.nodes += 1;
        self.stats.boundary_tests += boundary;
        Ok(true)
    }
}

pub(crate) fn path_error(motion: &Motion, tolerance: f64) -> f64 {
    // The contract permits a circular declared endpoint within tolerance of its
    // analytic circle. Cover that endpoint jump, plus binary64 coordinate guards.
    let mismatch = if matches!(motion.curve, crate::motion::Curve::Line { .. }) {
        0.
    } else {
        tolerance
    };
    let scale = motion
        .curve
        .start()
        .into_iter()
        .chain(motion.curve.end())
        .map(f64::abs)
        .fold(1., f64::max);
    let angular = match motion.curve {
        crate::motion::Curve::Arc {
            start,
            center,
            plane,
            sweep_rad,
            ..
        }
        | crate::motion::Curve::Helix {
            start,
            center,
            plane,
            sweep_rad,
            ..
        } => {
            let [u, v, _] = plane.axes();
            // AxisSpan widens angular membership by
            // 256*EPS*(1 + |start_angle+sweep*a| + |sweep|).
            // For a in [0,1], this bound covers that full lateral displacement.
            (start[u] - center[u]).hypot(start[v] - center[v])
                * (1. + std::f64::consts::PI + 2. * sweep_rad.abs())
        }
        _ => 0.,
    };
    mismatch + tolerance + 256. * f64::EPSILON * (scale + angular)
}

fn check_body<C: CancelSignal>(
    body: &Body,
    part: &Part,
    path: &PreparedCurve,
    motion: &Motion,
    error: f64,
    work: &mut Work<'_, C>,
) -> Result<Option<Finding>, String> {
    let b = &body.definition;
    let guard = body.guard + error;
    // A machining tolerance is distinct from numeric uncertainty. Only the
    // cutter on a fed move may enter this explicitly declared top cap. Side
    // boundaries, bottom, fixtures and every noncutting tool part stay strict.
    let top = if b.role == BodyRole::ProtectedTarget
        && part.kind == ToolPart::Cutter
        && motion.kind != crate::motion::MoveKind::Rapid
        && !motion.kind.is_probe()
    {
        b.z_max_mm - work.options.target_floor_tolerance_mm.unwrap_or(0.)
    } else {
        b.z_max_mm
    };
    let bounds = path.bounds();
    let mut stack = vec![(0., 1., 0u8)];
    let mut uncertain = None;
    while let Some((a, z, depth)) = stack.pop() {
        work.stats.max_stack_nodes = work.stats.max_stack_nodes.max(stack.len() + 1);
        if !work.charge(body.segments)? {
            uncertain.get_or_insert([a, z]);
            break;
        }
        // Independent whole-curve extrema avoid speed-based subdivision for a
        // flat floor or a descent ending above the protected interior.
        if bounds.min[2] + part.min - error > top + body.guard
            || bounds.max[2] + part.max + error < b.z_min_mm - body.guard
        {
            work.stats.pruned_intervals += 1;
            continue;
        }
        let t = (a + z) / 2.;
        let p = path.position(t)?;
        let reach = path.length_mm() * (z - a) / 2. + error;
        let lo = part.min.max(b.z_min_mm - p[2] - reach - body.guard);
        let hi = part.max.min(top - p[2] + reach + body.guard);
        if lo > hi {
            work.stats.pruned_intervals += 1;
            continue;
        }
        let radius = part.radius.max_over(lo, hi);
        if let Some((center, body_radius)) = body.outer_circle
            && let Some(distance) = circle_axis_distance(motion, path, [a, z], center)
            && distance > body_radius + radius + guard
        {
            work.stats.pruned_intervals += 1;
            continue;
        }
        if body.hole_disks.iter().any(|&(center, hole_radius)| {
            circle_axis_max_enclosure(motion, path, [a, z], center)
                .is_some_and(|distance| distance + radius + guard < hole_radius)
        }) {
            work.stats.pruned_intervals += 1;
            continue;
        }
        let query = body.region.query([p[0], p[1]])?;
        let d = query.signed_clearance_mm.ok_or("empty verification body")?;
        if d + radius + reach + body.guard < 0. {
            work.stats.pruned_intervals += 1;
            continue;
        }
        // Strict interior witness. It detects a collision, never proves clear.
        let low = part.min.max(b.z_min_mm - p[2]);
        let high = part.max.min(top - p[2]);
        if high - low > 2. * guard {
            let inset = (2. * guard).min((high - low) / 2.);
            let height = [low + inset, (low + high) / 2., high - inset]
                .into_iter()
                .max_by(|a, b| part.radius.at(*a).total_cmp(&part.radius.at(*b)))
                .unwrap();
            let inner_radius = part
                .radius
                .at(height - guard)
                .min(part.radius.at(height + guard));
            let margin = (d + inner_radius).min((height - low).min(high - height)) - guard;
            if margin > 0. {
                if b.representation == Representation::OuterEnvelope {
                    return Ok(Some(Finding {
                        status:Status::Uncertain,
                        kind:if b.role==BodyRole::Fixture {FindingKind::FixtureContact}else{FindingKind::Gouge},
                        object_id:b.id.clone(),part:Some(part.kind),holder_section:part.holder,
                        fraction_interval:[a,z],witness:None,
                        detail:"Tool overlaps a CAD outer envelope; source-solid contact is not established".into(),
                    }));
                }
                return Ok(Some(Finding {
                    status: Status::Violation,
                    kind: if b.role == BodyRole::Fixture {
                        FindingKind::FixtureContact
                    } else {
                        FindingKind::Gouge
                    },
                    object_id: b.id.clone(),
                    part: Some(part.kind),
                    holder_section: part.holder,
                    fraction_interval: [a, z],
                    witness: Some(Witness {
                        fraction: t,
                        tip_mm: p,
                        section_height_mm: Some(height),
                        margin_lower_mm: margin,
                    }),
                    detail: "Strict interior prism/profile overlap at localized witness".into(),
                }));
            }
        }
        if depth == 48 || path.length_mm() * (z - a) <= work.options.spatial_resolution_mm {
            uncertain.get_or_insert([a, z]);
        } else {
            stack.push((t, z, depth + 1));
            stack.push((a, t, depth + 1));
        }
    }
    Ok(uncertain.map(|span| Finding {
        status: Status::Uncertain,
        kind: if b.role == BodyRole::Fixture {
            FindingKind::FixtureContact
        } else {
            FindingKind::Gouge
        },
        object_id: b.id.clone(),
        part: Some(part.kind),
        holder_section: part.holder,
        fraction_interval: span,
        witness: None,
        detail: if work.stats.budget_exhausted {
            "Work budget exhausted; interval remains unchecked"
        } else {
            "Contact/uncertainty remains within resolution; cannot establish separation"
        }
        .into(),
    }))
}

/// Machine travel of one move, decided in closed form (#166, #212) by the
/// one travel rule (`crate::travel`, #246), which the posted-output check
/// applies to the decoded moves too: the travel box is closed, a declared end
/// is exact (only the binary64 evaluation of the work-to-machine transform is
/// allowed for) and a point of a planned arc's nominal circle keeps the arc's
/// own endpoint gap (none when the declared end is on the circle). The motion
/// tolerance is not a travel allowance.
///
/// Every point tested is a point of the move, so one outside is a violation
/// with that point as the witness (the earliest along the move), and a move
/// with none is inside. Nothing is left to subdivide: a move that starts on,
/// touches or runs along a limit is decided like any other.
fn check_travel(
    scene: &Scene,
    limits: &TipTravel<'_>,
    path: &PreparedCurve,
) -> Result<Option<Finding>, String> {
    let found = crate::travel::curve_outside(
        &scene.work_to_machine,
        &limits.travel,
        path,
        crate::travel::ArcGap::Planned,
    )?;
    Ok(found.map(|found| Finding {
        status: Status::Violation,
        kind: FindingKind::MachineTravel,
        object_id: "machine".into(),
        part: None,
        holder_section: None,
        fraction_interval: [found.fraction, found.fraction],
        witness: Some(Witness {
            fraction: found.fraction,
            tip_mm: found.point_mm,
            section_height_mm: None,
            margin_lower_mm: found.beyond_mm - found.guard_mm,
        }),
        detail: limits.violation(&found),
    }))
}

impl PreparedScene {
    pub fn digest(&self) -> &str {
        &self.digest
    }
    pub fn definition(&self) -> &Scene {
        &self.definition
    }
    pub fn verify_motion(
        &self,
        tool: &PreparedAssembly,
        motion: &Motion,
        cancel: &impl CancelSignal,
    ) -> Result<Report, String> {
        let tool_sha256 = identity(tool.definition())?;
        let motion_sha256 = motion_identity(motion);
        self.verify_motion_digested(tool, motion, &tool_sha256, &motion_sha256, cancel)
    }
    /// `verify_motion` with the tool and motion identities already computed
    /// (a stream computes each once instead of once per check).
    pub(crate) fn verify_motion_digested(
        &self,
        tool: &PreparedAssembly,
        motion: &Motion,
        tool_sha256: &str,
        motion_sha256: &str,
        cancel: &impl CancelSignal,
    ) -> Result<Report, String> {
        if motion.tool != tool.definition().id
            || !valid_id(&motion.operation)
            || motion.validate_feed(None).is_err()
        {
            return Err(
                "verification motion needs matching tool, operation and valid rapid/feed semantics"
                    .into(),
            );
        }
        let path = motion.curve.prepare(self.options.tolerance_mm)?;
        let error = path_error(motion, self.options.tolerance_mm);
        let travel = match &self.definition.machine {
            Some(m) => Some(m.tip_travel(tool.definition())?),
            None => None,
        };
        // Where a probing move expects its contact, the stylus (or the
        // measured cutter) may touch whatever it measures; the other parts
        // stay checked.
        let parts = parts(tool)?
            .into_iter()
            .filter(|p| p.kind != ToolPart::Cutter || !motion.kind.expects_contact())
            .collect::<Vec<_>>();
        let mut work = Work {
            stats: Stats::default(),
            options: self.options,
            cancel,
        };
        // Poll even when all geometry checks are trivially empty.
        if cancel.is_cancelled() {
            return Err("verification cancelled; no clear result published".into());
        }
        let mut findings = Vec::new();
        if let (Some(limits), Some(feed)) = (&self.definition.machine, motion.feed_mm_min)
            && feed > limits.max_feed_mm_min
        {
            findings.push(Finding {
                status: Status::Violation,
                kind: FindingKind::FeedLimit,
                object_id: "machine".into(),
                part: None,
                holder_section: None,
                fraction_interval: [0., 1.],
                witness: None,
                detail:
                    "Declared fed-motion rate exceeds configured limit (no controller timing model)"
                        .into(),
            });
        }
        let mut normal_examined = false;
        if let (Some(limit), Some(feed)) = (
            self.definition
                .machine
                .as_ref()
                .and_then(|m| m.max_normal_accel_mm_s2),
            motion.feed_mm_min,
        ) {
            normal_examined = true;
            let (acceleration, state) = path.nominal_normal_acceleration_limit(feed, limit)?;
            let status = match state {
                crate::motion::NormalAccelerationState::Exceeded => Some(Status::Violation),
                crate::motion::NormalAccelerationState::Boundary => Some(Status::Uncertain),
                crate::motion::NormalAccelerationState::Within => None,
            };
            if let Some(status) = status {
                if findings.len() < self.options.max_findings {
                    findings.push(Finding {
                        status, kind: FindingKind::NormalAccelerationLimit,
                        object_id: "machine".into(), part: None, holder_section: None,
                        fraction_interval: [0., 1.], witness: None,
                        detail: format!("Nominal analytic curve normal acceleration {acceleration:.9} mm/s^2 at declared feed versus configured {limit:.9}; excludes junctions, tangential/axis acceleration, endpoint correction and controller timing"),
                    });
                } else {
                    work.stats.findings_truncated = true;
                }
            }
        }
        let mut axis_examined = false;
        if let (Some(limits), Some(feed)) = (
            self.definition
                .machine
                .as_ref()
                .and_then(|m| m.max_axis_feed_mm_min),
            motion.feed_mm_min,
        ) {
            axis_examined = true;
            for (axis, (peak, state)) in path
                .nominal_axis_feed_limit(feed, &self.definition.work_to_machine, limits)?
                .into_iter()
                .enumerate()
            {
                let status = match state {
                    crate::motion::AxisFeedState::Exceeded => Some(Status::Violation),
                    crate::motion::AxisFeedState::Boundary => Some(Status::Uncertain),
                    crate::motion::AxisFeedState::Within => None,
                };
                if let Some(status) = status {
                    if findings.len() < self.options.max_findings {
                        findings.push(Finding {
                            status, kind: FindingKind::AxisFeedLimit,
                            object_id: "machine".into(), part: None, holder_section: None,
                            fraction_interval: [0., 1.], witness: None,
                            detail: format!("Nominal analytic machine axis {} peak feed {peak:.9} mm/min versus configured {:.9}; excludes junctions, acceleration, jerk, endpoint correction and controller timing", ["X","Y","Z"][axis], limits[axis]),
                        });
                    } else {
                        work.stats.findings_truncated = true;
                    }
                }
            }
        }
        'bodies: for body in &self.bodies {
            for part in &parts {
                if findings.len() == self.options.max_findings {
                    work.stats.findings_truncated = true;
                    break 'bodies;
                }
                let finding = check_body(body, part, &path, motion, error, &mut work)?;
                work.stats.pairs_checked += 1;
                if let Some(f) = finding {
                    findings.push(f);
                }
                if work.stats.budget_exhausted {
                    break 'bodies;
                }
            }
        }
        let mut travel_examined = false;
        if let Some(travel) = &travel
            && !work.stats.budget_exhausted
            && findings.len() < self.options.max_findings
        {
            travel_examined = true;
            if let Some(f) = check_travel(&self.definition, travel, &path)? {
                findings.push(f);
            }
            work.stats.pairs_checked += 1;
        }
        let total =
            self.bodies.len() * parts.len() + usize::from(self.definition.machine.is_some());
        work.stats.unexamined_pairs = total - work.stats.pairs_checked;
        let status = if findings.iter().any(|f| f.status == Status::Violation) {
            Status::Violation
        } else if work.stats.unexamined_pairs > 0
            || work.stats.budget_exhausted
            || findings.iter().any(|f| f.status == Status::Uncertain)
        {
            Status::Uncertain
        } else {
            Status::Clear
        };
        Ok(Report {
            scene_id: self.definition.id.clone(),
            revision: self.definition.revision.clone(),
            scene_sha256: self.digest.clone(),
            tool_sha256: tool_sha256.to_owned(),
            motion_sha256: motion_sha256.to_owned(),
            motion_id: motion.id,
            operation: motion.operation.clone(),
            tool: motion.tool.clone(),
            status,
            coverage: Coverage {
                configured_prism_bodies: self.bodies.len(),
                enclosure_only_bodies:self.bodies.iter().filter(|b|b.definition.representation==Representation::OuterEnvelope).count(),
                machine_tip_travel_examined: travel_examined,
                declared_fed_rate_checked: self.definition.machine.is_some()
                    && motion.feed_mm_min.is_some(),
                nominal_normal_accel_checked: normal_examined,
                nominal_axis_feed_checked: axis_examined,
                evolving_stock_contact: false,
                remaining_material: false,
                controller_execution: false,
            },
            findings,
            stats: work.stats,
            numerical_policy: "Binary64 Lipschitz enclosures with explicit geometry/path guards; engineering bounds, not certified interval arithmetic or global machine safety".into(),
            target_floor_tolerance_mm: self.options.target_floor_tolerance_mm,
        })
    }
}
