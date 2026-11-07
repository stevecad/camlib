//! Optional manifest-bound, cursor-attributed partial verification. Reports are
//! passive metadata: accepting one never changes stock or the playback cursor.
use crate::{
    material,
    motion::Motion,
    pocket::Operation,
    stock::{CancelSignal, Stock},
    stream::{DEFAULT_FRAME_LIMIT, RunIdentity, StreamManifest},
    tool::PreparedAssembly,
    verification,
};
use serde::{Deserialize, Serialize};
use std::{collections::HashMap, sync::Arc};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SceneCheck {
    pub scene: verification::Scene,
    #[serde(default)]
    pub options: verification::Options,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MaterialCheck<G = crate::surface::SurfaceMesh> {
    /// None means end of run; otherwise inspect stock after this operation.
    pub after_operation: Option<String>,
    pub target: material::Definition<G>,
    #[serde(default)]
    pub options: material::Options,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
#[serde(bound(deserialize = "G: Deserialize<'de>"))]
pub struct Config<G = crate::surface::SurfaceMesh> {
    pub scene: Option<SceneCheck>,
    #[serde(default)]
    pub material: Vec<MaterialCheck<G>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stock_contact: Option<crate::stock_contact::Options>,
}
impl<G> Config<G> {
    pub(crate) fn map_geometry<H, E>(
        self,
        f: &mut impl FnMut(G) -> Result<H, E>,
    ) -> Result<Config<H>, E> {
        Ok(Config {
            scene: self.scene,
            stock_contact: self.stock_contact,
            material: self
                .material
                .into_iter()
                .map(|m| {
                    Ok(MaterialCheck {
                        after_operation: m.after_operation,
                        target: m.target.map_geometry(f)?,
                        options: m.options,
                    })
                })
                .collect::<Result<_, E>>()?,
        })
    }
}
impl Config {
    pub fn validate(&self, operations: &[Operation]) -> Result<(), String> {
        if !self.motion_checks() && self.material.is_empty() {
            return Err("verification needs configured checks".into());
        }
        if let Some(scene) = &self.scene {
            scene.options.validate()?;
        }
        if let Some(options) = self.stock_contact {
            options.validate()?;
        }
        for (i, check) in self.material.iter().enumerate() {
            check.options.validate()?;
            if check.after_operation.as_ref().is_some_and(|id| {
                !crate::tool::valid_id(id)
                    || !operations.is_empty() && !operations.iter().any(|op| op.id() == id)
            }) || self.material[..i]
                .iter()
                .any(|old| old.after_operation == check.after_operation)
            {
                return Err(
                    "material stage must be unique and name a manifest operation or end of run"
                        .into(),
                );
            }
        }
        Ok(())
    }
    fn motion_checks(&self) -> bool {
        self.scene.is_some() || self.stock_contact.is_some()
    }
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Progress {
    pub records: u64,
    /// A resumed producer makes no claim about checks before this cursor.
    pub from_completed: u64,
    pub through_completed: u64,
    pub clear: u64,
    pub violation: u64,
    pub uncertain: u64,
    pub material_reports: u64,
}
impl Progress {
    fn motion(&mut self, completed: u64, status: verification::Status) {
        self.records += 1;
        self.through_completed = completed;
        match status {
            verification::Status::Clear => self.clear += 1,
            verification::Status::Violation => self.violation += 1,
            verification::Status::Uncertain => self.uncertain += 1,
        }
    }
}
#[derive(Debug, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Check {
    Motion {
        report: Option<Box<verification::Report>>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        stock_contact: Option<Box<crate::stock_contact::Report>>,
    },
    MaterialStarted {
        after_operation: Option<String>,
        target_id: String,
        target_sha256: String,
    },
    Material {
        after_operation: Option<String>,
        report: Box<material::Report>,
        /// The run's per-move gouge proof against a surface target (#72):
        /// the possible-only gouge cells of `report` it decides.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        sweep_proof: Option<material::SweepProofReport>,
    },
}
/// The internally tagged form, decoded without buffering: records are
/// written with "kind" first, so the variant is known before its fields,
/// which are then decoded straight into place (the derived decoder would
/// buffer every report into an intermediate tree first). Unknown, repeated
/// and missing required fields are refused as by the derived decoder.
impl<'de> Deserialize<'de> for Check {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        struct Fields;
        impl<'de> serde::de::Visitor<'de> for Fields {
            type Value = Check;
            fn expecting(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
                f.write_str("a check record with its kind first")
            }
            fn visit_map<A: serde::de::MapAccess<'de>>(
                self,
                mut map: A,
            ) -> Result<Check, A::Error> {
                use serde::de::Error;
                let key: Option<std::borrow::Cow<'de, str>> = map.next_key()?;
                let Some(key) = key else {
                    return Err(A::Error::missing_field("kind"));
                };
                if key != "kind" {
                    // Another writer's key order: buffer the record, then
                    // decode it exactly as when the kind comes first.
                    let mut record = serde_json::Map::new();
                    record.insert(key.into_owned(), map.next_value()?);
                    while let Some((k, v)) = map.next_entry::<String, serde_json::Value>()? {
                        if record.insert(k.clone(), v).is_some() {
                            return Err(A::Error::custom(format!("duplicate check field `{k}`")));
                        }
                    }
                    let kind = record
                        .remove("kind")
                        .ok_or_else(|| A::Error::missing_field("kind"))?;
                    // Kind first (serde_json maps sort their keys).
                    let mut text = format!(
                        "{{\"kind\":{}",
                        serde_json::to_string(&kind).map_err(A::Error::custom)?
                    );
                    for (k, v) in record {
                        text.push(',');
                        text.push_str(&serde_json::to_string(&k).map_err(A::Error::custom)?);
                        text.push(':');
                        text.push_str(&serde_json::to_string(&v).map_err(A::Error::custom)?);
                    }
                    text.push('}');
                    return serde_json::from_str::<Check>(&text).map_err(A::Error::custom);
                }
                let kind: std::borrow::Cow<'de, str> = map.next_value()?;
                let mut seen = std::collections::HashSet::new();
                let mut once = |name: &str| {
                    if seen.insert(name.to_string()) {
                        Ok(())
                    } else {
                        Err(A::Error::custom(format!("duplicate check field `{name}`")))
                    }
                };
                match kind.as_ref() {
                    "motion" => {
                        let (mut report, mut stock_contact) = (None, None);
                        while let Some(key) = map.next_key::<std::borrow::Cow<'de, str>>()? {
                            once(&key)?;
                            match key.as_ref() {
                                "report" => report = map.next_value()?,
                                "stock_contact" => stock_contact = map.next_value()?,
                                other => {
                                    return Err(A::Error::unknown_field(
                                        other,
                                        &["kind", "report", "stock_contact"],
                                    ));
                                }
                            }
                        }
                        Ok(Check::Motion {
                            report,
                            stock_contact,
                        })
                    }
                    "material_started" => {
                        let (mut after, mut target_id, mut target_sha256) = (None, None, None);
                        while let Some(key) = map.next_key::<std::borrow::Cow<'de, str>>()? {
                            once(&key)?;
                            match key.as_ref() {
                                "after_operation" => after = map.next_value()?,
                                "target_id" => target_id = Some(map.next_value()?),
                                "target_sha256" => target_sha256 = Some(map.next_value()?),
                                other => {
                                    return Err(A::Error::unknown_field(
                                        other,
                                        &["kind", "after_operation", "target_id", "target_sha256"],
                                    ));
                                }
                            }
                        }
                        Ok(Check::MaterialStarted {
                            after_operation: after,
                            target_id: target_id
                                .ok_or_else(|| A::Error::missing_field("target_id"))?,
                            target_sha256: target_sha256
                                .ok_or_else(|| A::Error::missing_field("target_sha256"))?,
                        })
                    }
                    "material" => {
                        let (mut after, mut report, mut proof) = (None, None, None);
                        while let Some(key) = map.next_key::<std::borrow::Cow<'de, str>>()? {
                            once(&key)?;
                            match key.as_ref() {
                                "after_operation" => after = map.next_value()?,
                                "report" => report = Some(map.next_value()?),
                                "sweep_proof" => proof = map.next_value()?,
                                other => {
                                    return Err(A::Error::unknown_field(
                                        other,
                                        &["kind", "after_operation", "report", "sweep_proof"],
                                    ));
                                }
                            }
                        }
                        Ok(Check::Material {
                            after_operation: after,
                            report: report.ok_or_else(|| A::Error::missing_field("report"))?,
                            sweep_proof: proof,
                        })
                    }
                    other => Err(A::Error::unknown_variant(
                        other,
                        &["motion", "material_started", "material"],
                    )),
                }
            }
        }
        deserializer.deserialize_map(Fields)
    }
}
#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Record {
    pub completed: u64,
    pub progress: Progress,
    pub check: Check,
}
impl Check {
    pub fn motion_status(&self) -> Option<verification::Status> {
        let Self::Motion {
            report,
            stock_contact,
        } = self
        else {
            return None;
        };
        let mut statuses = report
            .as_ref()
            .map(|r| r.status)
            .into_iter()
            .chain(stock_contact.as_ref().map(|r| r.status));
        let initial = statuses.next()?;
        Some(statuses.fold(initial, |a, b| match (a, b) {
            (verification::Status::Violation, _) | (_, verification::Status::Violation) => {
                verification::Status::Violation
            }
            (verification::Status::Uncertain, _) | (_, verification::Status::Uncertain) => {
                verification::Status::Uncertain
            }
            _ => verification::Status::Clear,
        }))
    }
}

/// Cached metadata identities only; no stock allocation or geometry preparation.
pub struct Binding {
    config: Config,
    run: RunIdentity,
    stock: crate::stock::StockSpec,
    manifest_sha256: String,
    pub(crate) identity: Vec<u8>,
    scene_sha256: Option<String>,
    options_sha256: Option<String>,
    tool_sha256: HashMap<String, String>,
    target_sha256: Vec<String>,
}
impl Binding {
    pub fn new(manifest: &StreamManifest) -> Result<Self, String> {
        let config = manifest
            .verification
            .clone()
            .ok_or("run has no verification configuration")?;
        config.validate(&manifest.operations)?;
        Ok(Self {
            options_sha256: config
                .stock_contact
                .as_ref()
                .map(verification::identity)
                .transpose()?,
            scene_sha256: config
                .scene
                .as_ref()
                .map(|c| verification::identity(&(&c.scene, c.options)))
                .transpose()?,
            tool_sha256: manifest
                .tools
                .iter()
                .map(|t| Ok((t.id.clone(), verification::identity(t)?)))
                .collect::<Result<_, String>>()?,
            target_sha256: config
                .material
                .iter()
                .map(|c| verification::identity(&(&c.target, c.options)))
                .collect::<Result<_, _>>()?,
            config,
            run: manifest.run.clone(),
            stock: manifest.stock.height()?.clone(),
            manifest_sha256: format!("{:x}", sha2::Sha256::digest(manifest.canonical_bytes()?)),
            identity: crate::stream::identity_bytes(manifest)?,
        })
    }
    fn material_index(&self, after_operation: &Option<String>) -> Result<usize, String> {
        self.config
            .material
            .iter()
            .position(|c| c.after_operation == *after_operation)
            .ok_or_else(|| "unconfigured material verification stage".into())
    }
    fn validate_stock_contact(
        &self,
        r: &crate::stock_contact::Report,
        m: &Motion,
        motion_sha256: &str,
        completed: u64,
    ) -> Result<(), String> {
        use verification::{FindingKind, Status, ToolPart};
        let options = self
            .config
            .stock_contact
            .ok_or("unconfigured stock contact")?;
        let options_sha256 = self
            .options_sha256
            .as_deref()
            .ok_or("unconfigured stock contact")?;
        let (nx, ny, _) = self.stock.grid()?;
        if r.algorithm != "continuous_stock_contact_v1"
            || r.stock != self.stock
            || r.pre_generation != completed - 1
            || r.options_sha256 != options_sha256
            || self.tool_sha256.get(&m.tool) != Some(&r.tool_sha256)
            || r.motion_sha256 != motion_sha256
            || r.motion_id != m.id
            || r.operation != m.operation
            || r.tool != m.tool
            || r.scope
                != match m.kind {
                    crate::motion::MoveKind::Rapid => "rapid_assembly",
                    crate::motion::MoveKind::Shift => "oriented_shift",
                    _ => "fed_noncutting",
                }
            || r.findings.len() > options.max_findings
            || r.stats.nodes > options.max_nodes
            || r.stats.cell_reads > options.max_cell_reads
            || r.fully_examined != (r.stats.unexamined_parts == 0 && !r.stats.work_exhausted)
            || r.findings.iter().any(|f| {
                f.finding.kind != FindingKind::StockContact
                    || f.finding.object_id != "stock"
                    || f.cell_range[0] >= f.cell_range[2]
                    || f.cell_range[1] >= f.cell_range[3]
                    || f.cell_range[2] > nx
                    || f.cell_range[3] > ny
                    || f.finding.fraction_interval[0] < 0.
                    || f.finding.fraction_interval[1] > 1.
                    || f.finding.fraction_interval[0] > f.finding.fraction_interval[1]
                    || f.finding.status == Status::Clear
                    || f.solid_lower_mm < self.stock.min_mm[2]
                    || f.possible_upper_mm > self.stock.max_mm[2]
                    || f.solid_lower_mm > f.possible_upper_mm
                    || (m.kind != crate::motion::MoveKind::Rapid
                        && f.finding.part == Some(ToolPart::Cutter))
                    || (f.finding.status == Status::Violation
                        && (f.finding.witness.is_none() || f.contact_point_mm.is_none()))
                    || f.finding.witness.as_ref().is_some_and(|w| {
                        w.margin_lower_mm <= 0.
                            || w.fraction < f.finding.fraction_interval[0]
                            || w.fraction > f.finding.fraction_interval[1]
                    })
                    || f.contact_point_mm
                        .is_some_and(|p| p[2] <= self.stock.min_mm[2] || p[2] >= f.solid_lower_mm)
            })
            || (r.status == Status::Clear && (!r.findings.is_empty() || !r.fully_examined))
            || (r.status == Status::Violation
                && !r
                    .findings
                    .iter()
                    .any(|f| f.finding.status == Status::Violation))
        {
            return Err("stock-contact attribution, source, cursor or scope mismatch".into());
        }
        Ok(())
    }
    /// Validate attribution, source identities and scope without claiming to
    /// reexecute the checker. Useful for retained random-access report pages.
    pub fn decode(
        &self,
        bytes: &[u8],
        completed: u64,
        motion: Option<&Motion>,
    ) -> Result<Record, String> {
        let record =
            crate::stream::decode_verification(&self.identity, bytes, DEFAULT_FRAME_LIMIT)?;
        let p = &record.progress;
        let count = p
            .clear
            .checked_add(p.violation)
            .and_then(|n| n.checked_add(p.uncertain));
        if record.completed != completed
            || p.from_completed > p.through_completed
            || p.through_completed > completed
            || count != Some(p.through_completed - p.from_completed)
            || p.material_reports > self.config.material.len() as u64
            || p.records == 0
            || p.records > i64::MAX as u64
            || (!self.config.motion_checks() && p.through_completed != p.from_completed)
        {
            return Err("verification cursor/progress mismatch".into());
        }
        match &record.check {
            Check::Motion {
                report,
                stock_contact,
            } => {
                let m = motion.ok_or("motion verification needs the committed source motion")?;
                if !self.config.motion_checks()
                    || report.is_some() != self.config.scene.is_some()
                    || stock_contact.is_some() != self.config.stock_contact.is_some()
                    || completed == 0
                    || p.through_completed != completed
                {
                    return Err("unconfigured/missing motion checker or cursor mismatch".into());
                }
                // One canonical identity per motion, shared by both checks.
                let motion_sha256 = verification::motion_identity(m);
                if let (Some(r), Some(c)) = (report.as_ref(), self.config.scene.as_ref())
                    && (completed == 0
                        || p.through_completed != completed
                        || r.motion_id != m.id
                        || r.operation != m.operation
                        || r.tool != m.tool
                        || r.motion_sha256 != motion_sha256
                        || self.tool_sha256.get(&m.tool) != Some(&r.tool_sha256)
                        || self.scene_sha256.as_ref() != Some(&r.scene_sha256)
                        || r.scene_id != c.scene.id
                        || r.revision != c.scene.revision
                        || r.findings.len() > c.options.max_findings
                        || r.coverage.configured_prism_bodies != c.scene.bodies.len()
                        || r.coverage.enclosure_only_bodies
                            != c.scene
                                .bodies
                                .iter()
                                .filter(|b| {
                                    b.representation == verification::Representation::OuterEnvelope
                                })
                                .count()
                        || r.coverage.evolving_stock_contact
                        || r.coverage.remaining_material
                        || r.coverage.controller_execution
                        || (r.coverage.machine_tip_travel_examined && c.scene.machine.is_none())
                        || (c.scene.machine.is_some()
                            && !r.coverage.machine_tip_travel_examined
                            && r.stats.unexamined_pairs == 0)
                        || r.coverage.declared_fed_rate_checked
                            != (c.scene.machine.is_some() && m.feed_mm_min.is_some())
                        || r.coverage.nominal_normal_accel_checked
                            != (c
                                .scene
                                .machine
                                .as_ref()
                                .and_then(|l| l.max_normal_accel_mm_s2)
                                .is_some()
                                && m.feed_mm_min.is_some())
                        || r.coverage.nominal_axis_feed_checked
                            != (c
                                .scene
                                .machine
                                .as_ref()
                                .and_then(|l| l.max_axis_feed_mm_min)
                                .is_some()
                                && m.feed_mm_min.is_some())
                        || r.findings.iter().any(|f| {
                            f.fraction_interval[0] < 0.
                                || f.fraction_interval[1] > 1.
                                || f.fraction_interval[0] > f.fraction_interval[1]
                                || f.witness.as_ref().is_some_and(|w| {
                                    w.fraction < f.fraction_interval[0]
                                        || w.fraction > f.fraction_interval[1]
                                        || w.margin_lower_mm <= 0.
                                })
                                || (matches!(
                                    f.kind,
                                    verification::FindingKind::FixtureContact
                                        | verification::FindingKind::Gouge
                                ) && c.scene.bodies.iter().any(|b| {
                                    b.id == f.object_id
                                        && b.representation
                                            == verification::Representation::OuterEnvelope
                                }) && (f.status == verification::Status::Violation
                                    || f.witness.is_some()))
                        })
                        || (r.status == verification::Status::Clear
                            && (!r.findings.is_empty()
                                || r.stats.unexamined_pairs != 0
                                || r.stats.budget_exhausted)))
                {
                    return Err(
                        "motion verification attribution, source or coverage mismatch".into(),
                    );
                }
                if let Some(r) = stock_contact {
                    self.validate_stock_contact(r, m, &motion_sha256, completed)?;
                }
            }
            Check::MaterialStarted {
                after_operation,
                target_id,
                target_sha256,
            } => {
                let i = self.material_index(after_operation)?;
                if *target_id != self.config.material[i].target.id()
                    || *target_sha256 != self.target_sha256[i]
                {
                    return Err("material start target identity mismatch".into());
                }
            }
            Check::Material {
                after_operation,
                report: r,
                sweep_proof,
            } => {
                let i = self.material_index(after_operation)?;
                let c = &self.config.material[i];
                let binding = r
                    .context
                    .run
                    .as_ref()
                    .ok_or("material report lacks run binding")?;
                let (nx, ny, _) = self.stock.grid()?;
                let finishing_valid = match (c.target.finishing_scope(), r.finishing.as_ref()) {
                    (None, None) => true,
                    (Some(scope), Some(f)) => {
                        let material::Definition::Surface(source) = &c.target else {
                            return Err("finishing report needs a surface target".into());
                        };
                        let inner = !source.inner_patches.is_empty()
                            || source.contact_witnesses.is_some()
                            || source.surface.surface_error_mm == 0.
                            || matches!(
                                source.source_enclosure,
                                material::SurfaceEnclosure::Inscribed
                            );
                        f.valid_binding(scope, c.options, nx * ny)
                            && f.inner_contact_available == inner
                    }
                    _ => false,
                };
                if !finishing_valid
                    || r.algorithm != c.target.algorithm()
                    || r.target_id != c.target.id()
                    || r.target_revision != c.target.revision()
                    || r.target_sha256 != self.target_sha256[i]
                    || binding.run != self.run
                    || binding.manifest_sha256 != self.manifest_sha256
                    || verification::identity(&r.context.stock)?
                        != verification::identity(&self.stock)?
                    || r.context.generation != completed
                    || r.context.fraction != 0.
                    || r.comparison_tolerance_mm != c.options.comparison_tolerance_mm
                    || r.findings.len() > c.options.max_findings
                    || r.stats.validated_cells != nx * ny
                    || r.stats
                        .classified_cells
                        .checked_add(r.stats.unexamined_cells)
                        != Some(nx * ny)
                    || r.complete
                        != (r.stats.unexamined_cells == 0
                            && r.finishing.as_ref().is_none_or(|f| f.complete))
                    || r.stock_sha256.len() != 64
                    || !r.stock_sha256.bytes().all(|b| b.is_ascii_hexdigit())
                    || [&r.residual, &r.gouge].iter().any(|s| {
                        s.volume_mm3[0] < 0.
                            || s.volume_mm3[1] < s.volume_mm3[0]
                            || s.definite_cells
                                .checked_add(s.possible_only_cells)
                                .is_none_or(|n| n > r.stats.classified_cells)
                            || (s.assessment == material::Assessment::WithinTolerance
                                && (s.definite_cells != 0
                                    || s.possible_only_cells != 0
                                    || r.stats.unexamined_cells != 0))
                    })
                    || r.findings.iter().any(|f| f.cell_index as usize >= nx * ny)
                    // A proof decides at most the report's possible-only
                    // gouge cells, at the configured tolerance.
                    || sweep_proof.as_ref().is_some_and(|p| {
                        p.decided_cells > r.gouge.possible_only_cells
                            || p.tolerance_mm != c.options.comparison_tolerance_mm
                    })
                {
                    return Err("material verification source, cursor or coverage mismatch".into());
                }
            }
        }
        Ok(record)
    }
}
use sha2::Digest;

#[derive(Clone)]
pub struct Tracker {
    pub binding: Arc<Binding>,
    pub progress: Progress,
    pending: Option<(usize, u64)>,
    done: Vec<usize>,
}
impl Tracker {
    pub fn new(binding: Arc<Binding>, completed: u64) -> Self {
        Self {
            binding,
            progress: Progress {
                from_completed: completed,
                through_completed: completed,
                ..Progress::default()
            },
            pending: None,
            done: Vec::new(),
        }
    }
    /// Transactional metadata receipt; failed records preserve coverage state.
    pub fn accept(
        &mut self,
        bytes: &[u8],
        completed: u64,
        motion: Option<&Motion>,
    ) -> Result<Record, String> {
        let r = self.binding.decode(bytes, completed, motion)?;
        let mut expected = self.progress.clone();
        let mut pending = self.pending;
        let mut finished = None;
        match &r.check {
            Check::Motion { .. } => {
                if pending.is_some()
                    || completed
                        != expected
                            .through_completed
                            .checked_add(1)
                            .ok_or("verification cursor exhausted")?
                {
                    return Err(
                        "missing/repeated verification or material check still active".into(),
                    );
                }
                expected.motion(completed, r.check.motion_status().unwrap());
            }
            Check::MaterialStarted {
                after_operation, ..
            } => {
                let i = self.binding.material_index(after_operation)?;
                if pending.is_some()
                    || self.done.contains(&i)
                    || self.binding.config.motion_checks()
                        && expected.through_completed != completed
                {
                    return Err("repeated material stage or unchecked motion prefix".into());
                }
                expected.records += 1;
                pending = Some((i, completed));
            }
            Check::Material {
                after_operation, ..
            } => {
                let i = self.binding.material_index(after_operation)?;
                if pending != Some((i, completed)) {
                    return Err("material report lacks matching started stage/cursor".into());
                }
                expected.records += 1;
                expected.material_reports += 1;
                pending = None;
                finished = Some(i);
            }
        }
        if r.progress != expected {
            return Err("missing/repeated/out-of-order verification progress".into());
        }
        self.progress = expected;
        self.pending = pending;
        if let Some(i) = finished {
            self.done.push(i);
        }
        Ok(r)
    }
    pub fn finish(&self, completed: u64) -> Result<(), String> {
        if self.pending.is_some()
            || self.done.len() != self.binding.config.material.len()
            || self.binding.config.motion_checks() && self.progress.through_completed != completed
        {
            return Err("verification incomplete at run end".into());
        }
        Ok(())
    }
}

pub(crate) struct Runtime {
    scene: Option<verification::PreparedScene>,
    targets: Vec<material::PreparedMaterial>,
    /// Per surface target: every motion proven against it (#72).
    proofs: Vec<Option<material::SweepProof>>,
    /// The stream's curve tolerance (validated every motion already).
    sweep_tolerance_mm: f64,
    pub tracker: Tracker,
}
impl Runtime {
    pub fn ready_for_motion(&self) -> Result<(), String> {
        if self.tracker.pending.is_some() {
            return Err("material check still active; stock cannot advance".into());
        }
        Ok(())
    }
    pub fn prepare(manifest: &StreamManifest, completed: u64) -> Result<Self, String> {
        let binding = Arc::new(Binding::new(manifest)?);
        let never = std::sync::atomic::AtomicBool::new(false);
        let proofs = binding
            .config
            .material
            .iter()
            .map(|c| match &c.target {
                // The motions are proven only from the raw stock on: a
                // resumed stream decides no cell (history incomplete).
                material::Definition::Surface(t) => {
                    material::SweepProof::new(t, c.options, &binding.stock, completed == 0, &never)
                        .map(Some)
                }
                material::Definition::Prismatic(t) => material::SweepProof::new_prismatic(
                    t,
                    c.options,
                    &binding.stock,
                    completed == 0,
                )
                .map(Some),
            })
            .collect::<Result<_, String>>()?;
        Ok(Self {
            scene: binding
                .config
                .scene
                .as_ref()
                .map(|c| c.scene.prepare(c.options))
                .transpose()?,
            targets: binding
                .config
                .material
                .iter()
                .map(|c| c.target.prepare(c.options))
                .collect::<Result<_, _>>()?,
            proofs,
            sweep_tolerance_mm: manifest.sweep.tolerance_mm,
            tracker: Tracker::new(binding, completed),
        })
    }
    pub fn motion(
        &mut self,
        stock: &Stock,
        tool: &PreparedAssembly,
        motion: &Motion,
        completed: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Vec<u8>>, String> {
        let tolerance = self.sweep_tolerance_mm;
        for proof in self.proofs.iter_mut().flatten() {
            proof.motion(tool, motion, tolerance, cancel)?;
        }
        if !self.tracker.binding.config.motion_checks() {
            return Ok(None);
        }
        // Run-constant identities come from the binding; the motion identity
        // is computed once and shared by both reports.
        let binding = &self.tracker.binding;
        let tool_sha256 = binding
            .tool_sha256
            .get(&motion.tool)
            .ok_or("motion tool is not declared in the run")?;
        let motion_sha256 = verification::motion_identity(motion);
        let report = self
            .scene
            .as_ref()
            .map(|scene| {
                scene.verify_motion_digested(tool, motion, tool_sha256, &motion_sha256, cancel)
            })
            .transpose()?
            .map(Box::new);
        let stock_contact = binding
            .config
            .stock_contact
            .map(|options| {
                let digests = crate::stock_contact::Digests {
                    options: binding.options_sha256.clone().unwrap_or_default(),
                    tool: tool_sha256.clone(),
                    motion: motion_sha256.clone(),
                };
                crate::stock_contact::check_digested(stock, tool, motion, options, &digests, cancel)
            })
            .transpose()?
            .map(Box::new);
        let check = Check::Motion {
            report,
            stock_contact,
        };
        let mut progress = self.tracker.progress.clone();
        progress.motion(completed, check.motion_status().unwrap());
        self.encode(&Record {
            completed,
            progress,
            check,
        })
        .map(Some)
    }
    fn encode(&self, record: &Record) -> Result<Vec<u8>, String> {
        crate::stream::encode_verification(
            &self.tracker.binding.identity,
            record,
            DEFAULT_FRAME_LIMIT,
        )
    }
    pub fn start_material(
        &mut self,
        after_operation: Option<&str>,
        completed: u64,
    ) -> Result<Option<Vec<u8>>, String> {
        let Some(i) = self
            .tracker
            .binding
            .config
            .material
            .iter()
            .position(|c| c.after_operation.as_deref() == after_operation)
        else {
            return Ok(None);
        };
        let c = &self.tracker.binding.config.material[i];
        let mut progress = self.tracker.progress.clone();
        progress.records += 1;
        let bytes = self.encode(&Record {
            completed,
            progress,
            check: Check::MaterialStarted {
                after_operation: c.after_operation.clone(),
                target_id: c.target.id().to_owned(),
                target_sha256: self.targets[i].digest().into(),
            },
        })?;
        self.tracker.accept(&bytes, completed, None)?;
        Ok(Some(bytes))
    }
    pub fn material(
        &mut self,
        stock: &crate::stock::Stock,
        completed: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Vec<u8>, String> {
        let (i, cursor) = self.tracker.pending.ok_or("no material check active")?;
        if cursor != completed {
            return Err("stock advanced during material check".into());
        }
        let b = &self.tracker.binding;
        let context = material::Context {
            stock: b.stock.clone(),
            generation: completed,
            fraction: 0.,
            run: Some(material::RunBinding {
                run: b.run.clone(),
                manifest_sha256: b.manifest_sha256.clone(),
            }),
        };
        let (report, sweep_proof) = match (&self.targets[i], &self.proofs[i]) {
            (material::PreparedMaterial::Surface(t), Some(proof)) => {
                let (report, proof) = t.check_proven(stock, &context, proof, cancel)?;
                (report, Some(proof))
            }
            (material::PreparedMaterial::Prismatic(t), Some(proof)) => {
                let (report, proof) = t.check_proven(stock, &context, proof, cancel)?;
                (report, Some(proof))
            }
            (target, _) => (target.check(stock, &context, cancel)?, None),
        };
        let mut progress = self.tracker.progress.clone();
        progress.records += 1;
        progress.material_reports += 1;
        let bytes = self.encode(&Record {
            completed,
            progress,
            check: Check::Material {
                after_operation: b.config.material[i].after_operation.clone(),
                report: Box::new(report),
                sweep_proof,
            },
        })?;
        self.tracker.accept(&bytes, completed, None)?;
        Ok(bytes)
    }
}

#[cfg(test)]
mod check_decoding_tests {
    use super::Check;

    #[test]
    fn check_records_decode_in_any_key_order_and_refuse_bad_fields() {
        let first = r#"{"kind":"material_started","after_operation":null,"target_id":"t","target_sha256":"ab"}"#;
        let sorted = r#"{"after_operation":null,"kind":"material_started","target_id":"t","target_sha256":"ab"}"#;
        for text in [first, sorted] {
            match serde_json::from_str::<Check>(text).unwrap() {
                Check::MaterialStarted {
                    after_operation,
                    target_id,
                    target_sha256,
                } => {
                    assert_eq!(
                        (after_operation, target_id.as_str(), target_sha256.as_str()),
                        (None, "t", "ab")
                    );
                }
                other => panic!("{other:?}"),
            }
        }
        let motion: Check = serde_json::from_str(r#"{"kind":"motion","report":null}"#).unwrap();
        assert!(matches!(
            motion,
            Check::Motion {
                report: None,
                stock_contact: None
            }
        ));
        for bad in [
            r#"{"kind":"motion","report":null,"report":null}"#,
            r#"{"kind":"motion","report":null,"extra":1}"#,
            r#"{"kind":"material_started","target_id":"t"}"#,
            r#"{"kind":"nonsense"}"#,
            r#"{"report":null}"#,
        ] {
            assert!(serde_json::from_str::<Check>(bad).is_err(), "{bad}");
        }
    }
}
