//! Post a setup to checked G-code from JSON, without Python (issue #120).
//!
//! In: the setup's stream manifest (what `cam-sim-stream` reads), its
//! program id and work frame, the machine and controller dialect as JSON
//! (`camlib.post.Machine.to_dict()`), optional lowering and the finding kinds
//! the caller accepts. Out: the program, released only when every finding is
//! clear or accepted, and the findings as JSON.
//!
//! The run is the one `Job.post_nc` makes with the default Python post:
//! the operations are planned and streamed (`regeneration::run_uncached`),
//! their events posted by the default post (`gcode`), every written block
//! decoded independently and compared with its source event (a mismatch is
//! an error), the decoded motions replayed on the stock with the setup's
//! checks (scene, stock contact, limits, material), the decoded program
//! checked against the declared machine model (#116) when the machine has
//! one, and the same export gate applied. Lathe setups and feed scheduling
//! are refused (not yet supported here).
//!
//! No filesystem, process or thread is used, so a WebAssembly build can call
//! [`run`].
use crate::gcode::{Engine, LoweringOptions, Machine};
use crate::motion::{Coolant, Event, Motion, Program, Vec3, WorkFrame};
use crate::post_intent::Output;
use crate::stock::CancelSignal;
use crate::stream::{
    DEFAULT_FRAME_LIMIT, GaugePose, InPlacePose, RunIdentity, StreamManifest, StreamProducer,
    TablePose, ToolChangePose,
};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, VecDeque};

/// The JSON input.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Input {
    /// The setup's stream manifest, as `cam-sim-stream` reads it.
    pub manifest: StreamManifest,
    /// The program id (the setup id).
    pub program_id: String,
    /// The setup's work frame; it must equal the machine's `frame`.
    pub frame: WorkFrame,
    /// `camlib.post.Machine.to_dict()`.
    pub machine: Value,
    #[serde(default)]
    pub lowering: Option<LoweringOptions>,
    /// Finding kinds accepted for release: uncertain, violation, unchecked_machine.
    #[serde(default)]
    pub accept: Vec<String>,
    /// The posting run id (default: derived from the input).
    #[serde(default)]
    pub run_id: Option<String>,
}

/// One gated result, as `PostedRun.export` reports it.
#[derive(Clone, Serialize)]
pub struct Finding {
    pub completed: Option<u64>,
    pub status: String,
    pub findings: Vec<Value>,
}

/// The JSON output.
#[derive(Serialize)]
pub struct Report {
    /// Whether the program is released (every finding clear or accepted).
    pub released: bool,
    /// The program text, only when released.
    pub program: Option<String>,
    /// Why the program was not released.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub refused: Option<Refusal>,
    pub lines: usize,
    pub motions: u64,
    pub findings: Vec<Finding>,
    pub accepted: Vec<String>,
    /// Checks the setup did not configure (not run: no claim either way).
    pub unchecked: Vec<String>,
    /// The machine model check (#116), or its unchecked_machine report.
    pub machine: Value,
    pub posting: Value,
}
#[derive(Serialize)]
pub struct Refusal {
    pub statuses: Vec<String>,
    pub message: String,
}

const ACCEPTABLE: [&str; 3] = ["uncertain", "violation", "unchecked_machine"];

/// Post the input JSON; returns the report JSON. Errors (invalid input, a
/// planning failure, a block that does not decode to its source) are
/// returned as `Err`; check findings are not errors, they gate release.
pub fn run(input: &str, cancel: &impl CancelSignal) -> Result<String, String> {
    let parsed: Input = serde_json::from_str(input).map_err(|e| format!("post job input: {e}"))?;
    let run_id = parsed.run_id.clone().unwrap_or_else(|| {
        let digest = format!("{:x}", Sha256::digest(input.as_bytes()));
        format!("post-{}", &digest[..32])
    });
    let report = post(parsed, run_id, cancel)?;
    serde_json::to_string(&report).map_err(|e| e.to_string())
}

/// An item of the source event stream (`posted.OperationEvents`).
enum Item {
    Event(Event),
    /// An operation ended (segmented runs only).
    Boundary(String),
}
enum Update {
    Operation {
        operation: String,
        stage: String,
        events: Vec<Event>,
    },
    Motion(Motion),
}

pub fn post(input: Input, run_id: String, cancel: &impl CancelSignal) -> Result<Report, String> {
    let kinds: Vec<String> = input.accept.clone();
    if let Some(k) = kinds.iter().find(|k| !ACCEPTABLE.contains(&k.as_str())) {
        return Err(format!("accept takes {ACCEPTABLE:?}, not {k:?}"));
    }
    let machine = Machine::from_json(input.machine)?;
    let manifest = input.manifest;
    if manifest.turning.is_some() {
        return Err(
            "lathe setups are not supported by the Rust post yet: post them with the Python post"
                .into(),
        );
    }
    if manifest.version != 1 || manifest.operations.is_empty() {
        return Err("the post job needs a version 1 setup manifest with its operations".into());
    }
    check_tool_change_pose(&machine, &manifest)?;
    let header = Program {
        version: 1,
        id: input.program_id,
        revision: manifest.run.revision.clone(),
        units: "mm".into(),
        tolerance_mm: manifest.sweep.tolerance_mm,
        frame: input.frame,
        initial_position_mm: manifest.initial_position_mm,
        tools: manifest.tools.clone(),
        events: vec![],
    };
    // Material stages after an operation: the posted replay checks them
    // after that operation's posted moves (a segmented source).
    let stages: Vec<String> = manifest
        .verification
        .as_ref()
        .map(|v| {
            v.material
                .iter()
                .filter_map(|m| m.after_operation.clone())
                .collect()
        })
        .unwrap_or_default();
    if stages
        .iter()
        .any(|s| !manifest.operations.iter().any(|op| op.id() == s))
    {
        return Err("unknown source operation material stage".into());
    }
    let items = source_events(&manifest, !stages.is_empty(), cancel)?;

    let (mut engine, first) = Engine::begin(
        &header,
        machine.clone(),
        input.lowering,
        run_id.clone(),
        cancel,
    )?;
    let identity = serde_json::to_value(&first.chunk.identity).map_err(|e| e.to_string())?;
    let decoded_initial = engine.position_mm();
    let mut chunks: Vec<Output> = vec![first];
    // (number of chunks posted before it, operation) of each material stage.
    let mut material_after: Vec<(usize, String)> = vec![];
    let mut segment = vec![];
    for item in items {
        match item {
            Item::Event(e) => segment.push(e),
            Item::Boundary(op) => {
                engine.consume_events(std::mem::take(&mut segment), cancel, |o| {
                    chunks.push(o);
                    Ok(())
                })?;
                if stages.contains(&op) {
                    material_after.push((chunks.len(), op));
                }
            }
        }
    }
    engine.consume_events(segment, cancel, |o| {
        chunks.push(o);
        Ok(())
    })?;
    chunks.push(engine.finish(cancel)?);
    // Chunks of events written in a later block (a compensation change
    // carried by its move) have no lines and nothing to replay.
    let journaled = |o: &Output| {
        !(o.chunk.lines.is_empty()
            && !o.chunk.checked.is_empty()
            && o.chunk
                .checked
                .iter()
                .all(|c| c.motion.is_none() && c.line_interval[0] > c.line_interval[1]))
    };

    // The posted program replayed on the stock with the setup's checks.
    let replay = StreamManifest {
        version: 1,
        run: RunIdentity {
            id: run_id.clone(),
            revision: header.revision.clone(),
        },
        stock: manifest.stock.clone(),
        // The declared blank the plan started from (#133): the replay
        // must start from the same material.
        initial_stock: manifest.initial_stock.clone(),
        tools: header.tools.clone(),
        initial_position_mm: decoded_initial,
        sweep: manifest.sweep,
        operations: vec![],
        verification: manifest.verification.clone(),
        tool_change: replay_tool_change(&machine, &header)?,
        rapid_retracts: true,
        turning: None,
    };
    let identity_bytes = crate::stream::identity_bytes(&replay)?;
    let mut producer = StreamProducer::new(replay.clone())?;
    let mut records: Vec<Value> = vec![];
    let mut record = |bytes: &[u8]| -> Result<(), String> {
        let r = crate::stream::decode_verification(&identity_bytes, bytes, DEFAULT_FRAME_LIMIT)?;
        records.push(serde_json::to_value(&r).map_err(|e| e.to_string())?);
        Ok(())
    };
    let mut motions = 0u64;
    let mut stage = material_after.iter().peekable();
    for (index, chunk) in chunks.iter().enumerate() {
        while let Some((at, op)) = stage.peek() {
            if *at != index {
                break;
            }
            let started = producer
                .start_material_verification(Some(op.as_str()))?
                .ok_or("material stage is not configured")?;
            record(&started)?;
            record(&producer.finish_material_verification(cancel)?)?;
            stage.next();
        }
        if !journaled(chunk) {
            continue;
        }
        for check in &chunk.chunk.checked {
            if let Some(motion) = &check.motion {
                motions += 1;
                let (_, verification) =
                    producer.push_checked_motion(motion.clone(), DEFAULT_FRAME_LIMIT, cancel)?;
                if let Some(bytes) = verification {
                    record(&bytes)?;
                }
            }
        }
    }
    if let Some(started) = producer.start_material_verification(None)? {
        record(&started)?;
        record(&producer.finish_material_verification(cancel)?)?;
    }
    producer.finish(DEFAULT_FRAME_LIMIT)?;

    // Findings, as PostedRun.export gathers them.
    let mut findings = vec![];
    for r in &records {
        let check = &r["check"];
        let kind = check["kind"].as_str().unwrap_or_default();
        let (status, detail) = if kind == "material" {
            let report = &check["report"];
            let proof = check.get("sweep_proof").filter(|p| !p.is_null());
            let detail = match proof {
                None => report.clone(),
                Some(p) => {
                    let mut d = report.clone();
                    d["sweep_proof"] = p.clone();
                    d
                }
            };
            (Some(material_status(report, proof)), vec![detail])
        } else if kind == "motion" {
            (motion_status(check), motion_findings(check))
        } else {
            (None, vec![])
        };
        if let Some(status) = status.filter(|s| s != "clear") {
            findings.push(Finding {
                completed: r["completed"].as_u64(),
                status,
                findings: detail,
            });
        }
    }
    let machine_report = check_machine(
        &machine,
        &replay,
        &engine.definition,
        &chunks,
        &journaled,
        motions,
        cancel,
    )?;
    let machine_status = machine_report["status"]
        .as_str()
        .unwrap_or_default()
        .to_string();
    if machine_status != "clear" {
        let mut groups: Vec<(String, Vec<Value>)> = vec![];
        for f in machine_report["findings"].as_array().into_iter().flatten() {
            let status = f["status"].as_str().unwrap_or(&machine_status).to_string();
            match groups.iter_mut().find(|(s, _)| *s == status) {
                Some((_, list)) => list.push(f.clone()),
                None => groups.push((status, vec![f.clone()])),
            }
        }
        for (status, detail) in groups {
            findings.push(Finding {
                completed: None,
                status,
                findings: detail,
            });
        }
    }
    let lines: usize = chunks
        .iter()
        .filter(|c| journaled(c))
        .map(|c| c.chunk.lines.len())
        .sum();
    let mut unchecked = unchecked_checks(&manifest);
    if machine_status == "unchecked_machine" {
        unchecked.push("machine".into());
    }
    let posting = json!({
        "run_id": run_id,
        "identity": identity,
        "definition": engine.definition,
        "arc_output": engine.arc_output,
        "checked_motions": motions,
        "chunks": chunks.iter().filter(|c| journaled(c)).count(),
    });
    let refused: Vec<&Finding> = findings
        .iter()
        .filter(|f| !kinds.contains(&f.status))
        .collect();
    if !refused.is_empty() {
        let mut statuses: Vec<String> = refused.iter().map(|f| f.status.clone()).collect();
        statuses.sort();
        statuses.dedup();
        let first = refused[0];
        let message = format!(
            "export refused: {} move(s) with unaccepted {} verification results (first at move {}: {}); fix the job, or accept {} to release it anyway",
            refused.len(),
            statuses.join("/"),
            first.completed.map_or("None".into(), |c| c.to_string()),
            describe(first),
            statuses.join(", ")
        );
        // As `ExportRefused.findings`: the unaccepted results.
        let findings = refused.into_iter().cloned().collect();
        return Ok(Report {
            released: false,
            program: None,
            refused: Some(Refusal { statuses, message }),
            lines,
            motions,
            findings,
            accepted: vec![],
            unchecked,
            machine: machine_report,
            posting,
        });
    }
    let released: Vec<&String> = findings.iter().map(|f| &f.status).collect();
    let mut accepted: Vec<String> = kinds
        .iter()
        .filter(|k| released.contains(k))
        .cloned()
        .collect();
    accepted.sort();
    accepted.dedup();
    let findings: Vec<Finding> = findings
        .into_iter()
        .filter(|f| f.status != "unchecked_machine")
        .collect();
    let mut text = String::new();
    for chunk in chunks.iter().filter(|c| journaled(c)) {
        for line in &chunk.chunk.lines {
            text.push_str(&line.text);
            text.push('\n');
        }
    }
    Ok(Report {
        released: true,
        program: Some(text),
        refused: None,
        lines,
        motions,
        findings,
        accepted,
        unchecked,
        machine: machine_report,
        posting,
    })
}

/// Plan and stream the setup's operations and return its events as
/// `posted.OperationEvents(reuse_active_tool=True)` yields them.
fn source_events(
    manifest: &StreamManifest,
    segmented: bool,
    cancel: &impl CancelSignal,
) -> Result<Vec<Item>, String> {
    let identity = crate::stream::identity_bytes(manifest)?;
    let mut producer = StreamProducer::new(manifest.clone())?;
    let mut updates: Vec<Update> = vec![];
    crate::regeneration::run_uncached(&mut producer, cancel, |bytes| {
        match crate::stream::frame_kind(bytes)? {
            crate::stream::CHUNK_KIND => {
                updates.push(Update::Motion(crate::stream::decode_chunk_motion(
                    manifest,
                    &identity,
                    bytes,
                    DEFAULT_FRAME_LIMIT,
                )?))
            }
            crate::stream::OPERATION_KIND => {
                let control =
                    crate::stream::decode_operation_control(manifest, bytes, DEFAULT_FRAME_LIMIT)?;
                updates.push(Update::Operation {
                    operation: control["operation"]
                        .as_str()
                        .unwrap_or_default()
                        .to_string(),
                    stage: control["stage"].as_str().unwrap_or_default().to_string(),
                    events: serde_json::from_value(control["events"].clone())
                        .map_err(|e| e.to_string())?,
                });
            }
            _ => {}
        }
        Ok(())
    })?;
    Ok(operation_events(
        updates,
        manifest.initial_position_mm,
        segmented,
    ))
}

type Modal = (
    u8,
    (String, u32, u32),
    (u64, crate::motion::SpindleDirection),
    Coolant,
);

fn is_stop(events: &[Event]) -> bool {
    let mut kinds: Vec<&str> = events
        .iter()
        .map(|e| match e {
            Event::Coolant { .. } => "coolant",
            Event::Spindle { .. } => "spindle",
            _ => "other",
        })
        .collect();
    kinds.sort();
    kinds == ["coolant", "spindle"]
        && events.iter().all(|e| match e {
            Event::Coolant { mode } => *mode == Coolant::Off,
            Event::Spindle { rpm, .. } => *rpm == 0.,
            _ => false,
        })
}
/// The modal state and declared pose of a standard operation start, or None.
fn begin_modal(events: &[Event]) -> Option<(Modal, Vec3)> {
    if events.len() != 4 {
        return None;
    }
    let (mut offset, mut tool, mut spindle, mut coolant) = (None, None, None, None);
    for e in events {
        match e {
            Event::WorkOffset { number } if offset.is_none() => offset = Some(*number),
            Event::ToolChange {
                tool: t,
                number,
                length_register,
                position_after_mm,
            } if tool.is_none() => {
                tool = Some(((t.clone(), *number, *length_register), *position_after_mm))
            }
            Event::Spindle { rpm, direction } if spindle.is_none() => {
                spindle = Some((*rpm, *direction))
            }
            Event::Coolant { mode } if coolant.is_none() => coolant = Some(*mode),
            _ => return None,
        }
    }
    let ((assignment, pose), (rpm, direction)) = (tool?, spindle?);
    if rpm == 0. {
        return None;
    }
    Some((
        (offset?, assignment, (rpm.to_bits(), direction), coolant?),
        pose,
    ))
}

/// `OperationEvents.__next__` over a finished run's updates.
fn operation_events(updates: Vec<Update>, initial: Vec3, segmented: bool) -> Vec<Item> {
    let mut out = vec![];
    let mut active: Option<(String, u32, u32)> = None;
    let mut position = initial;
    let mut held: Option<Vec<Event>> = None;
    let mut modal: Option<Modal> = None;
    let mut controls: VecDeque<Event> = VecDeque::new();
    let mut pending_boundary: Option<String> = None;
    let mut updates = updates.into_iter();
    loop {
        while let Some(event) = controls.pop_front() {
            if let Event::ToolChange {
                tool,
                number,
                length_register,
                position_after_mm,
            } = &event
            {
                // Omit only an identical assignment that keeps the pose.
                let assignment = (tool.clone(), *number, *length_register);
                let same = active.as_ref() == Some(&assignment) && *position_after_mm == position;
                active = Some(assignment);
                position = *position_after_mm;
                if same {
                    continue;
                }
            }
            out.push(Item::Event(event));
        }
        if segmented && let Some(h) = held.take() {
            controls = h.into();
            continue;
        }
        if let Some(b) = pending_boundary.take()
            && segmented
        {
            out.push(Item::Boundary(b));
        }
        let Some(update) = updates.next() else {
            match held.take() {
                None => return out,
                Some(h) => {
                    controls = h.into();
                    continue;
                }
            }
        };
        match update {
            Update::Operation {
                operation,
                stage,
                mut events,
            } => {
                if stage == "end" && held.is_none() && is_stop(&events) {
                    held = Some(std::mem::take(&mut events));
                } else if stage == "begin" {
                    // All-or-nothing modal continuation after a held stop.
                    let begin = begin_modal(&events);
                    let h = held.take();
                    let m = begin.as_ref().map(|b| b.0.clone());
                    if h.is_some()
                        && m.is_some()
                        && m == modal
                        && m.as_ref().map(|m| &m.1) == active.as_ref()
                        && begin.as_ref().map(|b| b.1) == Some(position)
                    {
                        events = vec![];
                    } else {
                        modal = m;
                        let mut all = h.unwrap_or_default();
                        all.extend(events);
                        events = all;
                    }
                }
                controls = events.into();
                if stage == "end" {
                    pending_boundary = Some(operation);
                }
            }
            Update::Motion(motion) => {
                position = motion.curve.end();
                out.push(Item::Event(Event::Motion { motion }));
            }
        }
    }
}

/// `posted.check_tool_change_pose`: the setup's tool-change poses must be
/// where the machine's tool change leaves each tool.
fn check_tool_change_pose(machine: &Machine, manifest: &StreamManifest) -> Result<(), String> {
    let mode = machine.tool_change_position_mode.as_str();
    let change = manifest.tool_change.as_ref();
    let gauges: BTreeMap<&str, f64> = manifest
        .tools
        .iter()
        .map(|t| (t.id.as_str(), t.gauge_length_mm))
        .collect();
    let mut used: Vec<&str> = vec![];
    for op in &manifest.operations {
        if !used.contains(&op.tool()) {
            used.push(op.tool());
        }
    }
    let initial = manifest.initial_position_mm;
    let tolerance = machine.tolerance_mm;
    type Tip = [Option<f64>; 3];
    let planned = |tool: &str, first: bool| -> Option<Tip> {
        match change {
            None => first.then_some(initial.map(Some)),
            Some(ToolChangePose::Table(t)) => t.tips_mm.get(tool).map(|p| p.map(Some)),
            // Each later pose depends on where the previous tool stops: the
            // posted output check compares every actual change.
            Some(ToolChangePose::InPlace(_)) => first.then_some(initial.map(Some)),
            Some(ToolChangePose::Gauge(g)) => {
                let xy = match g.xy_mm {
                    Some(xy) => [Some(xy[0]), Some(xy[1])],
                    None if first => [Some(initial[0]), Some(initial[1])],
                    None => [None, None],
                };
                Some([
                    xy[0],
                    xy[1],
                    Some(g.gauge_z_mm - gauges.get(tool).copied().unwrap_or(f64::NAN)),
                ])
            }
        }
    };
    let describe = || match change {
        None => "Setup.tool_change is None: the tip stays where it was".to_string(),
        Some(ToolChangePose::Table(_)) => "Setup.tool_change is a ToolChangeTable".to_string(),
        Some(ToolChangePose::InPlace(_)) => {
            "Setup.tool_change is ToolChangeInPlace: the spindle stays where it was".to_string()
        }
        Some(ToolChangePose::Gauge(g)) => format!(
            "Setup.tool_change is ToolChangePosition(gauge_z_mm={}, xy_mm={:?})",
            g.gauge_z_mm, g.xy_mm
        ),
    };
    if matches!(change, Some(ToolChangePose::InPlace(_))) != (mode == "preserve_spindle") {
        return Err(if mode == "preserve_spindle" {
            format!(
                "the machine changes tools where the spindle stands (tool_change_position_mode=\"preserve_spindle\"), but {}: plan with Setup(tool_change=ToolChangeInPlace())",
                describe()
            )
        } else {
            format!(
                "{}, but the machine's tool_change_position_mode is \"{mode}\": declare tool_change_position_mode=\"preserve_spindle\" if its tool change does not move the spindle, or plan with the machine's tool-change pose",
                describe()
            )
        });
    }
    let (tips, source, fix): (BTreeMap<String, Tip>, String, String) = match mode {
        // Checked above: the setup plans its changes in place too.
        "preserve_spindle" => return Ok(()),
        "preserve_tip" => {
            if change.is_some() {
                return Err(format!(
                    "the machine keeps the tip where it was at a tool change (tool_change_position_mode=\"preserve_tip\"), but {}: plan with Setup(tool_change=None)",
                    describe()
                ));
            }
            return Ok(());
        }
        "z_home" => {
            let gauge_z = machine.tool_change_home_z_mm.unwrap_or(f64::NAN)
                - machine.work_to_machine.origin_mm[2];
            let fix =
                format!("plan with Setup(tool_change=ToolChangePosition(gauge_z_mm={gauge_z}))");
            if let Some(c) = change
                && match c {
                    ToolChangePose::Table(_) => true,
                    ToolChangePose::Gauge(g) => g.xy_mm.is_some(),
                    // Refused above for every machine but preserve_spindle.
                    ToolChangePose::InPlace(_) => true,
                }
            {
                return Err(format!(
                    "M6 sends the spindle to Z home and keeps XY (tool_change_position_mode=\"z_home\"), but {}: {fix}",
                    describe()
                ));
            }
            let tips = used
                .iter()
                .map(|t| {
                    (
                        t.to_string(),
                        [
                            None,
                            None,
                            Some(gauge_z - gauges.get(t).copied().unwrap_or(f64::NAN)),
                        ],
                    )
                })
                .collect();
            (tips, format!("Z home: gauge line at work Z {gauge_z}"), fix)
        }
        _ => {
            let tips: BTreeMap<String, Tip> = machine
                .tools
                .iter()
                .filter(|s| gauges.contains_key(s.tool.as_str()))
                .map(|s| (s.tool.clone(), s.position_after_mm.map(Some)))
                .collect();
            let missing: Vec<&&str> = used.iter().filter(|t| !tips.contains_key(**t)).collect();
            if !missing.is_empty() {
                return Err(format!(
                    "the machine tool table has no entry for tool(s) {missing:?}: add a ToolSetting for each"
                ));
            }
            (
                tips,
                "its tool table position_after_mm".into(),
                "plan with Setup(tool_change=ToolChangeTable.from_machine(machine))".into(),
            )
        }
    };
    for (k, tool) in used.iter().enumerate() {
        let Some(plan) = planned(tool, k == 0) else {
            continue;
        };
        let tip = tips[*tool];
        let agrees = plan.iter().zip(tip).all(|(a, b)| {
            a.is_none() || b.is_none() || (a.unwrap() - b.unwrap()).abs() <= tolerance
        });
        if !agrees {
            return Err(format!(
                "tool {tool}: the setup starts it at {plan:?} after its tool change ({}), but the machine leaves it at {tip:?} (tool_change_position_mode=\"{mode}\", {source}); {fix}",
                describe()
            ));
        }
    }
    Ok(())
}

/// Where the replay puts each tool after a decoded tool change: the
/// machine's tool-change behaviour (as `PostedRun` declares it).
fn replay_tool_change(
    machine: &Machine,
    header: &Program,
) -> Result<Option<ToolChangePose>, String> {
    Ok(match machine.tool_change_position_mode.as_str() {
        "z_home" => Some(ToolChangePose::Gauge(GaugePose {
            gauge_z_mm: machine.tool_change_home_z_mm.unwrap_or(f64::NAN)
                - machine.work_to_machine.origin_mm[2],
            xy_mm: None,
        })),
        "table" => {
            let mut tips = BTreeMap::new();
            for t in &header.tools {
                let s = machine
                    .tools
                    .iter()
                    .find(|s| s.tool == t.id)
                    .ok_or_else(|| {
                        format!(
                            "the machine's tool table has no entry for tool(s) [{:?}]",
                            t.id
                        )
                    })?;
                tips.insert(t.id.clone(), s.position_after_mm);
            }
            Some(ToolChangePose::Table(TablePose { tips_mm: tips }))
        }
        "preserve_spindle" => Some(ToolChangePose::InPlace(InPlacePose { in_place: true })),
        _ => None,
    })
}

/// `PostedRun.check_machine`.
fn check_machine(
    machine: &Machine,
    replay: &StreamManifest,
    definition: &Value,
    chunks: &[Output],
    journaled: &impl Fn(&Output) -> bool,
    motions: u64,
    cancel: &impl CancelSignal,
) -> Result<Value, String> {
    let Some(model) = &machine.crash_model else {
        return Ok(json!({
            "status": "unchecked_machine",
            "complete": false,
            "findings": [{
                "kind": "unchecked_machine",
                "bodies": ["machine", "program"],
                "detail": "no declared machine crash model; supply Machine.crash_model or explicitly accept unchecked_machine",
            }],
        }));
    };
    let model: crate::machine::Model =
        serde_json::from_value(model.clone()).map_err(|e| format!("machine model: {e}"))?;
    // Configuration errors are actionable errors, never clear results.
    let mut session = crate::machine::Session::new(model, replay.clone(), definition, cancel)?;
    let result = (|| {
        for chunk in chunks.iter().filter(|c| journaled(c)) {
            for check in &chunk.chunk.checked {
                for event in check.control_effects.clone().unwrap_or_default() {
                    session.event(event, check.event_index, cancel)?;
                }
                if let Some(motion) = &check.motion {
                    session.event(
                        Event::Motion {
                            motion: motion.clone(),
                        },
                        check.event_index,
                        cancel,
                    )?;
                }
            }
        }
        session.finish(motions, cancel).cloned()
    })();
    let report = match result {
        Ok(report) => report,
        Err(error) => session.incomplete(error),
    };
    serde_json::to_value(&report).map_err(|e| e.to_string())
}

/// `VerificationUpdate.status` of a motion record.
fn motion_status(check: &Value) -> Option<String> {
    let statuses: Vec<&str> = ["report", "stock_contact"]
        .iter()
        .filter_map(|k| check.get(*k).filter(|r| !r.is_null()))
        .filter_map(|r| r["status"].as_str())
        .collect();
    if statuses.is_empty() {
        return None;
    }
    Some(
        if statuses.contains(&"violation") {
            "violation"
        } else if statuses.contains(&"uncertain") {
            "uncertain"
        } else {
            "clear"
        }
        .into(),
    )
}
/// `VerificationUpdate.findings` of a motion record.
fn motion_findings(check: &Value) -> Vec<Value> {
    let mut out = vec![];
    if let Some(report) = check.get("report").filter(|r| !r.is_null()) {
        for f in report["findings"].as_array().into_iter().flatten() {
            let mut f = f.clone();
            f["source"] = json!("scene");
            out.push(f);
        }
    }
    if let Some(report) = check.get("stock_contact").filter(|r| !r.is_null()) {
        for f in report["findings"].as_array().into_iter().flatten() {
            let mut g = f["finding"].clone();
            g["source"] = json!("stock");
            g["cell_range"] = f["cell_range"].clone();
            g["contact_point_mm"] = f["contact_point_mm"].clone();
            // The obstruction for that part over the move (#129); null when
            // the record has none, as `finding.get(...)` in Python.
            for key in [
                "obstruction_top_mm",
                "lowest_tip_z_mm",
                "max_clear_radius_mm",
            ] {
                g[key] = f.get(key).cloned().unwrap_or(Value::Null);
            }
            out.push(g);
        }
    }
    out
}
/// `export.material_status`.
fn material_status(report: &Value, proof: Option<&Value>) -> String {
    let gouge = &report["gouge"];
    if gouge["definite_cells"].as_u64().unwrap_or(0) != 0 {
        return "violation".into();
    }
    if report["complete"] != json!(true) {
        return "uncertain".into();
    }
    let possible = gouge["possible_only_cells"].as_u64().unwrap_or(0);
    if possible != 0 && proof.is_none_or(|p| p["decided_cells"].as_u64() != Some(possible)) {
        return "uncertain".into();
    }
    "clear".into()
}
/// `export.unchecked`: the geometric checks the setup did not configure.
fn unchecked_checks(manifest: &StreamManifest) -> Vec<String> {
    let config = manifest
        .verification
        .as_ref()
        .map(|v| serde_json::to_value(v).unwrap_or(Value::Null))
        .unwrap_or(Value::Null);
    let truthy = |v: &Value| match v {
        Value::Null => false,
        Value::Bool(b) => *b,
        Value::Array(a) => !a.is_empty(),
        Value::Object(o) => !o.is_empty(),
        _ => true,
    };
    ["scene", "stock_contact", "material"]
        .into_iter()
        .filter(|k| !config.get(*k).is_some_and(truthy))
        .map(String::from)
        .collect()
}
/// `export.describe`: one line naming what a gated result found.
fn describe(result: &Finding) -> String {
    let Some(detail) = result.findings.first() else {
        return format!("{} result without a localized finding", result.status);
    };
    if detail.get("gouge").is_some() && detail.get("residual").is_some() {
        let target = detail
            .get("target_id")
            .and_then(Value::as_str)
            .map_or("the turned part".to_string(), |t| format!("target {t}"));
        return format!(
            "material gouge {} on {target}",
            detail["gouge"]["assessment"].as_str().unwrap_or_default()
        );
    }
    if let Some(bodies) = detail.get("bodies").and_then(Value::as_array) {
        let names: Vec<&str> = bodies.iter().filter_map(Value::as_str).collect();
        return format!(
            "{} {}: {}",
            detail
                .get("kind")
                .and_then(Value::as_str)
                .unwrap_or("machine check"),
            names.join(" / "),
            detail
                .get("detail")
                .and_then(Value::as_str)
                .unwrap_or_default()
        );
    }
    let d = detail.get("finding").unwrap_or(detail);
    format!(
        "{} {}",
        d.get("kind").and_then(Value::as_str).unwrap_or("finding"),
        d.get("object_id")
            .and_then(Value::as_str)
            .unwrap_or("(no object)")
    )
}
