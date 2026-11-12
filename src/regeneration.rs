//! Owned operation artifacts: typed motion/deltas, freshly bound output frames.
//! See docs/python.md; no verification findings or old frames cached.
use crate::{
    motion::{Event, Motion, Vec3},
    operation::OperationPlan,
    pocket::Operation,
    progress::{Cadence, Meter},
    stock::{CancelSignal, StockDelta},
    stream::{DEFAULT_FRAME_LIMIT, OperationControl, StreamProducer, encode_operation_control},
};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    cell::OnceCell,
    collections::HashMap,
    fs::{self, File},
    io::Write,
    path::PathBuf,
    time::{SystemTime, UNIX_EPOCH},
};

fn entry_limit() -> u64 {
    64 * 1024 * 1024
}
fn total_limit() -> u64 {
    1024 * 1024 * 1024
}
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct CacheOptions {
    pub directory: PathBuf,
    #[serde(default = "entry_limit")]
    pub max_entry_bytes: u64,
    #[serde(default = "total_limit")]
    pub max_total_bytes: u64,
    #[serde(default)]
    pub dependencies: HashMap<String, String>,
}
#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Header {
    version: u8,
    key: String,
    operation: String,
    generated_revision: String,
    base_generation: u64,
    events: Vec<Event>,
    report: Value,
    /// Events due after begin, before the first motion (a canned cycle).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    start_events: Vec<Event>,
}
#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Footer {
    motions: u64,
    events: Vec<Event>,
    report: Value,
    end_stock: String,
    end_position: Vec3,
}
#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Record {
    motion: Motion,
    delta: StockDelta,
    /// Events due after this motion (e.g. a dwell at the bottom of a hole).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    events: Vec<Event>,
}
/// Constructed only from a hash/key/attribution-checked native owned artifact.
/// This is not a public wire-delta entry point or a serialized native token.
pub(crate) struct VerifiedCacheMotion {
    motion: Motion,
    delta: StockDelta,
}
impl VerifiedCacheMotion {
    pub(crate) fn into_parts(self) -> (Motion, StockDelta) {
        (self.motion, self.delta)
    }
}
#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Index {
    version: u8,
    key: String,
    sha256: String,
    bytes: u64,
}
fn hex(bytes: impl AsRef<[u8]>) -> String {
    format!("{:x}", Sha256::digest(bytes.as_ref()))
}
/// Full-byte SHA-256, exactly the original portable identity algorithm.
fn executable_identity(bytes: &[u8]) -> String {
    hex(bytes)
}

fn digest_valid(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase())
}
/// Only provenance dictionaries are excluded; actual geometry/error/parameters
/// remain authoritative. Python nodes separately bind full CAD dependencies.
pub fn semantic_operation(operation: &Operation) -> Result<Value, String> {
    fn strip(v: &mut Value) {
        if let Value::Object(fields) = v {
            if fields.contains_key("components") || fields.contains_key("triangles") {
                fields.remove("source");
            }
            for value in fields.values_mut() {
                strip(value);
            }
        } else if let Value::Array(values) = v {
            for value in values {
                strip(value);
            }
        }
    }
    let mut value = serde_json::to_value(operation).map_err(|e| e.to_string())?;
    strip(&mut value);
    Ok(value)
}

fn stock_digest(producer: &StreamProducer) -> Result<String, String> {
    let mut h = Sha256::new();
    for bounds in producer.stock()?.cells() {
        h.update(bounds.lower_mm.to_le_bytes());
        h.update(bounds.upper_mm.to_le_bytes());
    }
    Ok(format!("{:x}", h.finalize()))
}
fn clearance_dependency(stock: &crate::stock::Stock, operation: &Operation) -> Option<[u8; 32]> {
    use crate::{adaptive::AdaptivePathStrategy::StockFront, roughing::RoughingClearing};
    let sensitive = match operation {
        Operation::AdaptivePocket(op) => op.path_strategy == StockFront,
        Operation::Roughing(op) => matches!(
            op.clearing,
            RoughingClearing::Adaptive {
                path_strategy: StockFront,
                ..
            }
        ),
        _ => false,
    };
    sensitive.then(|| stock.committed_clearance_fingerprint())
}
struct Entry {
    header: Header,
    footer: Footer,
    bytes: Vec<u8>,
    start: usize,
    end: usize,
}
impl Entry {
    fn verified_motion(
        &self,
        mut record: Record,
        op: &Operation,
        id: u64,
    ) -> Result<VerifiedCacheMotion, String> {
        if self.header.operation != op.id()
            || record.motion.operation != op.id()
            || record.motion.tool != op.tool()
            || record.delta.operation != op.id()
            || record.delta.fraction != 1.
            || record.delta.motion_id != record.motion.id
        {
            return Err("cached motion attribution/fraction mismatch".into());
        }
        record.motion.id = id;
        Ok(VerifiedCacheMotion {
            motion: record.motion,
            delta: record.delta,
        })
    }
    fn records(&self) -> impl Iterator<Item = Result<Record, String>> + '_ {
        self.bytes[self.start..self.end]
            .split(|b| *b == b'\n')
            .filter(|s| !s.is_empty())
            .map(|line| {
                serde_json::from_slice(line)
                    .map_err(|e| format!("invalid regeneration record: {e}"))
            })
    }
}
struct Store {
    options: CacheOptions,
    directory: PathBuf,
    kernel: OnceCell<String>,
}
impl Store {
    fn new(options: CacheOptions) -> Result<Self, String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("regeneration.store");
        if options.max_entry_bytes < 1024
            || options.max_entry_bytes > 64 * 1024 * 1024
            || options.max_total_bytes < options.max_entry_bytes
            || options.max_total_bytes > 16 * 1024 * 1024 * 1024
            || options.dependencies.values().any(|s| !digest_valid(s))
        {
            return Err("invalid bounded regeneration cache options/dependency digest".into());
        }
        let directory = options.directory.join("camlib-regeneration-v1");
        fs::create_dir_all(&directory).map_err(|e| e.to_string())?;
        Ok(Self {
            options,
            directory,
            kernel: OnceCell::new(),
        })
    }
    fn kernel(&self) -> Result<&str, String> {
        if self.kernel.get().is_none() {
            #[cfg(feature = "profile-planning")]
            let _profile = crate::planning_profile::span("regeneration.kernel_identity");
            let bytes = fs::read(std::env::current_exe().map_err(|e| e.to_string())?)
                .map_err(|e| e.to_string())?;
            let _ = self.kernel.set(executable_identity(&bytes));
        }
        Ok(self.kernel.get().unwrap())
    }
    fn key_inputs(&self, producer: &StreamProducer, op: &Operation) -> Result<Value, String> {
        let m = producer.manifest();
        let tool = m
            .tools
            .iter()
            .find(|t| t.id == op.tool())
            .ok_or("operation tool missing")?;
        let mut inputs = json!({"version":1,"operation":semantic_operation(op)?,
            "tool":tool,"stock":m.stock,"before_stock":stock_digest(producer)?,"pose":producer.position(),
            "sweep":m.sweep,"native_clearance":clearance_dependency(producer.stock()?,op),
            "dependencies":self.options.dependencies.get(op.id())});
        if m.rapid_retracts {
            inputs["rapid_retracts"] = true.into();
        }
        Ok(inputs)
    }
    fn key_from_inputs(&self, mut inputs: Value) -> Result<String, String> {
        inputs["kernel"] = self.kernel()?.into();
        Ok(hex(serde_json::to_vec(&inputs).map_err(|e| e.to_string())?))
    }
    fn key(&self, producer: &StreamProducer, op: &Operation) -> Result<String, String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("regeneration.key");
        self.key_from_inputs(self.key_inputs(producer, op)?)
    }
    fn has_input_hint(&self, hint: &str) -> bool {
        self.directory.join(format!(".{hint}.input-hint")).is_file()
    }
    fn mark_input_hint(&self, hint: &str) {
        // Advisory negative lookup only. Presence NEVER authorizes a hit;
        // reading an artifact still requires the full executable SHA-256 key.
        let _ = File::create_new(self.directory.join(format!(".{hint}.input-hint")));
    }
    fn load(&self, key: &str) -> Result<Option<Entry>, String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("regeneration.load");
        let path = self.directory.join(format!("{key}.index.json"));
        let bytes = match fs::read(path) {
            Ok(b) => b,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
            Err(e) => return Err(e.to_string()),
        };
        if bytes.len() > 1024 {
            return Err("regeneration cache index exceeds limit".into());
        }
        let index: Index = serde_json::from_slice(&bytes)
            .map_err(|e| format!("invalid regeneration index: {e}"))?;
        if index.version != 1 || index.key != key || !digest_valid(&index.sha256) {
            return Err("regeneration index identity mismatch".into());
        }
        if index.bytes > self.options.max_entry_bytes {
            return Ok(None);
        }
        let path = self.directory.join(format!("{key}.{}.jsonl", index.sha256));
        let file = match File::open(&path) {
            Ok(f) => f,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
            Err(e) => return Err(e.to_string()),
        };
        if file.metadata().map_err(|e| e.to_string())?.len() != index.bytes {
            return Err("regeneration cache length mismatch".into());
        }
        let mut bytes = Vec::new();
        use std::io::Read;
        file.take(self.options.max_entry_bytes + 1)
            .read_to_end(&mut bytes)
            .map_err(|e| e.to_string())?;
        if bytes.len() as u64 != index.bytes || hex(&bytes) != index.sha256 {
            return Err("regeneration cache checksum mismatch".into());
        }
        let first = bytes
            .iter()
            .position(|b| *b == b'\n')
            .ok_or("cache header missing")?;
        if bytes.last() != Some(&b'\n') {
            return Err("regeneration cache incomplete".into());
        }
        let last = bytes[..bytes.len() - 1]
            .iter()
            .rposition(|b| *b == b'\n')
            .ok_or("cache footer missing")?;
        let header: Header = serde_json::from_slice(&bytes[..first]).map_err(|e| e.to_string())?;
        let footer: Footer =
            serde_json::from_slice(&bytes[last + 1..bytes.len() - 1]).map_err(|e| e.to_string())?;
        if header.version != 1
            || header.key != key
            || footer.motions > 1_000_000
            || footer.motions
                != bytes[first + 1..last + 1]
                    .split(|b| *b == b'\n')
                    .filter(|s| !s.is_empty())
                    .count() as u64
            || !matches!(
                footer
                    .report
                    .get("geometric_residual_status")
                    .and_then(Value::as_str),
                Some("estimated" | "unknown")
            )
        {
            return Err("regeneration entry identity/count/completion mismatch".into());
        }
        Ok(Some(Entry {
            header,
            footer,
            bytes,
            start: first + 1,
            end: last + 1,
        }))
    }
    fn writer(&self, key: &str, header: &Header) -> Result<Option<Writer>, String> {
        let used = fs::read_dir(&self.directory)
            .map_err(|e| e.to_string())?
            .try_fold(0u64, |sum, e| {
                let e = e.map_err(|e| e.to_string())?;
                Ok::<_, String>(sum.saturating_add(e.metadata().map_err(|e| e.to_string())?.len()))
            })?;
        let available = self
            .options
            .max_total_bytes
            .saturating_sub(used)
            .min(self.options.max_entry_bytes);
        if available < 1024 {
            return Ok(None);
        }
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map_err(|e| e.to_string())?
            .as_nanos();
        let path = self
            .directory
            .join(format!(".{key}.{}.{nonce}.tmp", std::process::id()));
        let mut writer = Writer {
            file: Some(File::create_new(&path).map_err(|e| e.to_string())?),
            path,
            directory: self.directory.clone(),
            key: key.into(),
            written: 0,
            limit: available,
            hasher: Sha256::new(),
        };
        if !writer.append(header)? {
            return Ok(None);
        }
        Ok(Some(writer))
    }
}
struct Writer {
    file: Option<File>,
    path: PathBuf,
    directory: PathBuf,
    key: String,
    written: u64,
    limit: u64,
    hasher: Sha256,
}
impl Writer {
    fn append(&mut self, value: &impl Serialize) -> Result<bool, String> {
        let mut bytes = serde_json::to_vec(value).map_err(|e| e.to_string())?;
        bytes.push(b'\n');
        self.append_encoded(&bytes)
    }
    // Private owned records were serialized only after native checked commits.
    fn append_encoded(&mut self, bytes: &[u8]) -> Result<bool, String> {
        if self.written.saturating_add(bytes.len() as u64) > self.limit {
            return Ok(false);
        }
        self.file
            .as_mut()
            .ok_or("cache writer closed")?
            .write_all(bytes)
            .map_err(|e| e.to_string())?;
        self.written += bytes.len() as u64;
        self.hasher.update(bytes);
        Ok(true)
    }
    fn finish(mut self, footer: &Footer) -> Result<bool, String> {
        if !self.append(footer)? {
            return Ok(false);
        }
        self.file
            .take()
            .unwrap()
            .sync_all()
            .map_err(|e| e.to_string())?;
        let digest = format!("{:x}", self.hasher.clone().finalize());
        let artifact = self
            .directory
            .join(format!("{}.{}.jsonl", self.key, digest));
        fs::rename(&self.path, artifact).map_err(|e| e.to_string())?;
        let index = Index {
            version: 1,
            key: self.key.clone(),
            sha256: digest,
            bytes: self.written,
        };
        let staging = self.path.with_extension("index.tmp");
        fs::write(
            &staging,
            serde_json::to_vec(&index).map_err(|e| e.to_string())?,
        )
        .map_err(|e| e.to_string())?;
        fs::rename(
            staging,
            self.directory.join(format!("{}.index.json", self.key)),
        )
        .map_err(|e| e.to_string())?;
        Ok(true)
    }
}
impl Drop for Writer {
    fn drop(&mut self) {
        let _ = fs::remove_file(&self.path);
    }
}
const MAX_PREFIX_RECORDS: usize = 32;
const MAX_PREFIX_BYTES: usize = 4 * 1024 * 1024;
struct Recording<'a> {
    store: &'a Store,
    header: Header,
    pending_key: Option<Value>,
    writer: Option<Writer>,
    prefix: Vec<Vec<u8>>,
    prefix_bytes: usize,
    removed_lower_mm3: f64,
    cell_area: f64,
    ready: bool,
    input_hint: Option<String>,
}
impl<'a> Recording<'a> {
    fn new(
        store: &'a Store,
        header: Header,
        pending_key: Option<Value>,
        cell_area: f64,
        input_hint: Option<String>,
    ) -> Result<Self, String> {
        let writer = if pending_key.is_none() {
            store.writer(&header.key, &header)?
        } else {
            None
        };
        Ok(Self {
            store,
            header,
            pending_key,
            writer,
            prefix: vec![],
            prefix_bytes: 0,
            removed_lower_mm3: 0.,
            cell_area,
            ready: false,
            input_hint,
        })
    }
    fn needs_record(&self) -> bool {
        self.pending_key.is_some() || self.writer.is_some()
    }
    fn append(&mut self, record: &Record) -> Result<Option<Vec<u8>>, String> {
        let mut bytes = serde_json::to_vec(record).map_err(|e| e.to_string())?;
        bytes.push(b'\n');
        if self.pending_key.is_none() {
            if let Some(writer) = &mut self.writer
                && !writer.append_encoded(&bytes)?
            {
                self.writer = None;
            }
            return Ok(None);
        }
        self.removed_lower_mm3 += record
            .delta
            .changes()
            .iter()
            .map(|c| (c.before.upper_mm - c.after.upper_mm).max(0.) * self.cell_area)
            .sum::<f64>();
        self.ready |= self.removed_lower_mm3 > 0.001;
        let limit = MAX_PREFIX_BYTES.min(self.store.options.max_entry_bytes as usize);
        if bytes.len() > limit.saturating_sub(self.prefix_bytes) {
            self.ready = true;
            // At most one ordinary in-flight record, exactly as an immediate
            // writer; retained prefix never exceeds its explicit byte/count cap.
            return Ok(Some(bytes));
        }
        self.prefix_bytes += bytes.len();
        self.prefix.push(bytes);
        self.ready |= self.prefix.len() >= MAX_PREFIX_RECORDS;
        Ok(None)
    }
    fn activate(&mut self) -> Result<(), String> {
        if let Some(inputs) = self.pending_key.take() {
            self.header.key = self.store.key_from_inputs(inputs)?;
            self.writer = self.store.writer(&self.header.key, &self.header)?;
            for bytes in self.prefix.drain(..) {
                if let Some(writer) = &mut self.writer
                    && !writer.append_encoded(&bytes)?
                {
                    self.writer = None;
                }
            }
            self.prefix_bytes = 0;
        }
        Ok(())
    }
    fn after_publish(&mut self, overflow: Option<Vec<u8>>) -> Result<(), String> {
        if self.ready {
            self.activate()?;
        }
        if let Some(bytes) = overflow
            && let Some(writer) = &mut self.writer
            && !writer.append_encoded(&bytes)?
        {
            self.writer = None;
        }
        Ok(())
    }
    fn finish(mut self, footer: &Footer) -> Result<(bool, String), String> {
        self.activate()?;
        let saved = self
            .writer
            .take()
            .map(|w| w.finish(footer))
            .transpose()?
            .unwrap_or(false);
        if saved && let Some(hint) = &self.input_hint {
            self.store.mark_input_hint(hint);
        }
        Ok((saved, self.header.key))
    }
}

fn report_sources(
    report: &mut Value,
    op: &Operation,
    old_base: u64,
    new_base: u64,
) -> Result<(), String> {
    let definition = serde_json::to_value(op).map_err(|e| e.to_string())?;
    let source = definition
        .get("region")
        .or_else(|| definition.get("surface"))
        .and_then(|v| v.get("source"))
        .cloned()
        .unwrap_or(Value::Null);
    if let Some(fields) = report.as_object_mut() {
        for name in ["rest_from_generation", "rest_through_generation"] {
            if let Some(g) = fields.get(name).and_then(Value::as_u64) {
                fields.insert(
                    name.into(),
                    json!(
                        g.checked_sub(old_base)
                            .and_then(|v| new_base.checked_add(v))
                            .ok_or("cached report generation overflow")?
                    ),
                );
            }
        }
    }
    if let Some(proof) = report
        .get_mut("height_schedule")
        .and_then(|v| v.get_mut("stock_top_check"))
        .and_then(Value::as_object_mut)
        && let Some(g) = proof.get("generation").and_then(Value::as_u64)
    {
        let rebound = g
            .checked_sub(old_base)
            .and_then(|n| new_base.checked_add(n))
            .ok_or("cached stock-top proof generation overflow")?;
        proof.insert("generation".into(), json!(rebound));
    }
    for field in ["no_op", "stock_top"] {
        if let Some(proof) = report.get_mut(field).and_then(Value::as_object_mut)
            && let Some(g) = proof.get("generation").and_then(Value::as_u64)
        {
            let rebound = g
                .checked_sub(old_base)
                .and_then(|n| new_base.checked_add(n))
                .ok_or("cached stock report generation overflow")?;
            proof.insert("generation".into(), json!(rebound));
        }
    }
    fn update(value: &mut Value, source: &Value) {
        match value {
            Value::Object(fields) => {
                if fields.contains_key("components") && fields.contains_key("source") {
                    fields.insert("source".into(), source.clone());
                }
                for (key, value) in fields {
                    if key != "source" {
                        update(value, source);
                    }
                }
            }
            Value::Array(values) => {
                for value in values {
                    update(value, source);
                }
            }
            _ => {}
        }
    }
    update(report, &source);
    Ok(())
}

fn control(
    producer: &StreamProducer,
    op: &Operation,
    stage: &str,
    events: Vec<Event>,
    mut report: Value,
    stats: &Value,
    send: &mut impl FnMut(&[u8]) -> Result<(), String>,
) -> Result<(), String> {
    report
        .as_object_mut()
        .ok_or("operation report is not an object")?
        .insert("regeneration".into(), stats.clone());
    send(&encode_operation_control(
        producer.manifest(),
        &OperationControl {
            operation: op.id(),
            completed: producer.completed(),
            stage,
            events,
            report: Some(report),
        },
        DEFAULT_FRAME_LIMIT,
    )?)
}
/// Mid-operation events (after the motion just sent).
fn send_events(
    producer: &StreamProducer,
    op: &Operation,
    events: Vec<Event>,
    send: &mut impl FnMut(&[u8]) -> Result<(), String>,
) -> Result<(), String> {
    send(&encode_operation_control(
        producer.manifest(),
        &OperationControl::<Value> {
            operation: op.id(),
            completed: producer.completed(),
            stage: "events",
            events,
            report: None,
        },
        DEFAULT_FRAME_LIMIT,
    )?)
}
/// Plan and stream every operation of a version 1 run without a
/// regeneration cache (as `cam-sim-stream` does when no cache is given),
/// then the final material check and the end frame.
pub fn run_uncached(
    producer: &mut StreamProducer,
    cancel: &impl CancelSignal,
    mut send: impl FnMut(&[u8]) -> Result<(), String>,
) -> Result<(), String> {
    uncached(producer, cancel, &Meter::plain(&mut send))
}
/// Plan and stream a run as `run`, `run_uncached` or `run_turning` do (a
/// lathe run by its turning setup, otherwise with the cache when one is
/// given), with planning progress frames (#198, `crate::progress`) between
/// the run's own. Without those frames the output is the run's.
pub fn run_with_progress(
    producer: &mut StreamProducer,
    cache: Option<CacheOptions>,
    cancel: &impl CancelSignal,
    progress: Cadence,
    mut send: impl FnMut(&[u8]) -> Result<(), String> + Send,
) -> Result<(), String> {
    let lathe = producer.manifest().turning.as_ref();
    let operations = lathe.map_or(producer.manifest().operations.len(), |t| t.operations.len());
    let lathe = lathe.is_some();
    let meter = Meter::new(producer.manifest(), operations, progress, &mut send)?;
    meter.run(|| match cache {
        _ if lathe => turning(producer, cancel, &meter),
        Some(cache) => cached(producer, cache, cancel, &meter),
        None => uncached(producer, cancel, &meter),
    })
}
/// The run bodies are generic over the caller's cancel signal only, which
/// the planners get unchanged; the output and the progress are the `Meter`.
fn uncached<C: CancelSignal>(
    producer: &mut StreamProducer,
    cancel: &C,
    meter: &Meter<'_>,
) -> Result<(), String> {
    producer.writable()?;
    if producer.completed() != 0
        || producer.manifest().operations.is_empty()
        || producer.manifest().turning.is_some()
    {
        return Err(
            "operations require an unstarted writable run with manifest-bound operations".into(),
        );
    }
    let manifest = producer.manifest().clone();
    let send = |bytes: &[u8]| meter.send(bytes);
    let mut active: Option<&str> = None;
    for (index, operation) in manifest.operations.iter().enumerate() {
        meter.begin(operation.id(), index)?;
        let changed = active != Some(operation.tool());
        if changed {
            producer.change_tool(operation.tool())?;
            active = Some(operation.tool());
        }
        let tool = manifest
            .tools
            .iter()
            .find(|t| t.id == operation.tool())
            .ok_or("operation tool missing")?;
        let mut plan = OperationPlan::prepare_with_stock(
            operation.clone(),
            tool,
            producer.stock().unwrap(),
            producer.position(),
            producer
                .completed()
                .checked_add(1)
                .ok_or("motion ID overflow")?,
            cancel,
        )
        .map_err(|e| {
            if changed {
                producer.explain_tool_change(e)
            } else {
                e
            }
        })?;
        meter.report(plan.progress())?;
        send(&encode_operation_control(
            &manifest,
            &OperationControl {
                operation: operation.id(),
                completed: producer.completed(),
                stage: "begin",
                events: plan.prefix_events(),
                report: Some(plan.report()),
            },
            DEFAULT_FRAME_LIMIT,
        )?)?;
        // Events due before the first motion (a canned cycle).
        let events = plan.take_events();
        if !events.is_empty() {
            send(&encode_operation_control(
                &manifest,
                &OperationControl::<crate::operation::OperationReport> {
                    operation: operation.id(),
                    completed: producer.completed(),
                    stage: "events",
                    events,
                    report: None,
                },
                DEFAULT_FRAME_LIMIT,
            )?)?;
        }
        let guard =
            crate::limits::LinkGuard::new(operation.limits(), tool, manifest.sweep.tolerance_mm)?;
        while let Some(motion) = plan.next_motion_with_stock(producer.stock().unwrap(), cancel)? {
            let motion = manifest.retract_policy(
                motion,
                producer.stock().unwrap(),
                operation.declares_retract_feed(),
            );
            if let Some(g) = &guard {
                g.check(&motion)?;
            }
            let (chunk, check) =
                producer.push_checked_motion(motion, DEFAULT_FRAME_LIMIT, cancel)?;
            send(&chunk)?;
            if let Some(check) = check {
                send(&check)?;
            }
            let events = plan.take_events();
            if !events.is_empty() {
                send(&encode_operation_control(
                    &manifest,
                    &OperationControl::<crate::operation::OperationReport> {
                        operation: operation.id(),
                        completed: producer.completed(),
                        stage: "events",
                        events,
                        report: None,
                    },
                    DEFAULT_FRAME_LIMIT,
                )?)?;
            }
            meter.report(plan.progress())?;
        }
        plan.finish_report(cancel)?;
        meter.end()?;
        send(&encode_operation_control(
            &manifest,
            &OperationControl {
                operation: operation.id(),
                completed: producer.completed(),
                stage: "end",
                events: plan.suffix_events(),
                report: Some(plan.report()),
            },
            DEFAULT_FRAME_LIMIT,
        )?)?;
        while let Some(start) = producer.start_material_verification(Some(operation.id()))? {
            send(&start)?;
            send(&producer.finish_material_verification(cancel)?)?;
        }
    }

    while let Some(start) = producer.start_material_verification(None)? {
        send(&start)?;
        send(&producer.finish_material_verification(cancel)?)?;
    }
    send(&producer.finish(DEFAULT_FRAME_LIMIT)?)?;
    Ok(())
}
/// Generate or reuse complete operations, always publishing current-identity
/// frames and freshly evaluated verification/material records.
pub fn run(
    producer: &mut StreamProducer,
    options: CacheOptions,
    cancel: &impl CancelSignal,
    mut send: impl FnMut(&[u8]) -> Result<(), String>,
) -> Result<(), String> {
    cached(producer, options, cancel, &Meter::plain(&mut send))
}
fn cached<C: CancelSignal>(
    producer: &mut StreamProducer,
    options: CacheOptions,
    cancel: &C,
    meter: &Meter<'_>,
) -> Result<(), String> {
    producer.writable()?;
    if producer.manifest().turning.is_some() {
        return turning(producer, cancel, meter);
    }
    if producer.manifest().stock.dexel().is_some() {
        return Err("tri-dexel runs have no regeneration cache (it holds height-field stock changes); plan the run without a cache".into());
    }
    if producer.completed() != 0
        || producer.manifest().version != 1
        || producer.manifest().operations.is_empty()
    {
        return Err("regeneration requires an unstarted version 1 operation run".into());
    }
    if options.dependencies.keys().any(|id| {
        !producer
            .manifest()
            .operations
            .iter()
            .any(|op| op.id() == id)
    }) {
        return Err("regeneration dependency references an unknown operation".into());
    }
    let store = Store::new(options)?;
    // A missing advisory input hint may conservatively miss an older cache.
    // It cannot authorize replay; present hints still use full kernel identity.
    let mut first_operation = true;
    let operations = producer.manifest().operations.clone();
    let mut send = |bytes: &[u8]| meter.send(bytes);
    let mut active: Option<String> = None;
    for (index, op) in operations.into_iter().enumerate() {
        if cancel.is_cancelled() {
            return Err("job regeneration cancelled".into());
        }
        meter.begin(op.id(), index)?;
        // Before the cache key: the operation starts from the tool-change pose.
        let changed = active.as_deref() != Some(op.tool());
        if changed {
            producer.change_tool(op.tool())?;
            active = Some(op.tool().to_string());
        }
        let (pending_key, input_hint, key) = if first_operation {
            first_operation = false;
            let inputs = store.key_inputs(producer, &op)?;
            let hint = hex(serde_json::to_vec(&inputs).map_err(|e| e.to_string())?);
            if store.has_input_hint(&hint) {
                (None, Some(hint), Some(store.key_from_inputs(inputs)?))
            } else {
                (Some(inputs), Some(hint), None)
            }
        } else {
            (None, None, Some(store.key(producer, &op)?))
        };
        let entry = key.as_ref().map(|k| store.load(k)).transpose()?.flatten();
        let base = producer.completed();
        if let Some(entry) = entry {
            if entry.header.operation != op.id() {
                return Err("cached operation attribution mismatch".into());
            }
            let mut stats = json!({"cache_hit":true,"key":key,"planned_motions":0,"computed_sweeps":0,"validated_cached_columns":0,"cached_reach_cell_visits":0,"reused_motions":entry.footer.motions,"reused_deltas":entry.footer.motions,"generated_revision":entry.header.generated_revision,"saved":true});
            let mut begin = entry.header.report.clone();
            report_sources(&mut begin, &op, entry.header.base_generation, base)?;
            control(
                producer,
                &op,
                "begin",
                entry.header.events.clone(),
                begin,
                &stats,
                &mut send,
            )?;
            if !entry.header.start_events.is_empty() {
                send_events(producer, &op, entry.header.start_events.clone(), &mut send)?;
            }
            for record in entry.records() {
                // A reused operation is replayed, not planned: its progress
                // updates carry no fraction.
                if cancel.is_cancelled() {
                    return Err("job regeneration cancelled".into());
                }
                let id = producer
                    .completed()
                    .checked_add(1)
                    .ok_or("motion ID overflow")?;
                let mut record = record?;
                let events = std::mem::take(&mut record.events);
                let record = entry.verified_motion(record, &op, id)?;
                let (frame, check, validated) =
                    producer.push_verified_cache_motion(record, DEFAULT_FRAME_LIMIT, cancel)?;
                stats["validated_cached_columns"] =
                    (stats["validated_cached_columns"].as_u64().unwrap() + validated.columns)
                        .into();
                stats["cached_reach_cell_visits"] =
                    (stats["cached_reach_cell_visits"].as_u64().unwrap()
                        + validated.reach_cell_visits)
                        .into();
                send(&frame)?;
                if let Some(check) = check {
                    send(&check)?;
                }
                if !events.is_empty() {
                    send_events(producer, &op, events, &mut send)?;
                }
            }
            if producer.position() != entry.footer.end_position
                || stock_digest(producer)? != entry.footer.end_stock
            {
                return Err("cached operation final stock/position mismatch".into());
            }
            meter.end()?;
            let mut report = entry.footer.report.clone();
            report_sources(&mut report, &op, entry.header.base_generation, base)?;
            control(
                producer,
                &op,
                "end",
                entry.footer.events.clone(),
                report,
                &stats,
                &mut send,
            )?;
        } else {
            let tool = producer
                .manifest()
                .tools
                .iter()
                .find(|t| t.id == op.tool())
                .ok_or("operation tool missing")?
                .clone();
            let mut plan = {
                #[cfg(feature = "profile-planning")]
                let _profile = crate::planning_profile::span("regeneration.plan.prepare");
                OperationPlan::prepare_with_stock(
                    op.clone(),
                    &tool,
                    producer.stock()?,
                    producer.position(),
                    base.checked_add(1).ok_or("motion ID overflow")?,
                    cancel,
                )
                .map_err(|e| {
                    if changed {
                        producer.explain_tool_change(e)
                    } else {
                        e
                    }
                })?
            };
            meter.report(plan.progress())?;
            let begin = serde_json::to_value(plan.report()).map_err(|e| e.to_string())?;
            let header = Header {
                version: 1,
                key: key.clone().unwrap_or_default(),
                operation: op.id().into(),
                generated_revision: producer.manifest().run.revision.clone(),
                base_generation: base,
                events: plan.prefix_events(),
                report: begin.clone(),
                start_events: plan.take_events(),
            };
            let step = producer.stock()?.cell_widths_mm();
            let mut recording =
                Recording::new(&store, header, pending_key, step[0] * step[1], input_hint)?;
            let mut count = 0u64;
            let mut stats = json!({"cache_hit":false,"key":key,"planned_motions":0,"computed_sweeps":0,"reused_motions":0,"reused_deltas":0,"generated_revision":producer.manifest().run.revision,"saved":false});
            control(
                producer,
                &op,
                "begin",
                recording.header.events.clone(),
                begin,
                &stats,
                &mut send,
            )?;
            if !recording.header.start_events.is_empty() {
                send_events(
                    producer,
                    &op,
                    recording.header.start_events.clone(),
                    &mut send,
                )?;
            }
            let guard = crate::limits::LinkGuard::new(
                op.limits(),
                &tool,
                producer.manifest().sweep.tolerance_mm,
            )?;
            while let Some(motion) = {
                #[cfg(feature = "profile-planning")]
                let _profile = crate::planning_profile::span("regeneration.plan.next");
                plan.next_motion_with_stock(producer.stock()?, cancel)?
            } {
                let motion = producer.manifest().retract_policy(
                    motion,
                    producer.stock()?,
                    op.declares_retract_feed(),
                );
                if let Some(g) = &guard {
                    g.check(&motion)?;
                }
                let events = plan.take_events();
                if recording.needs_record() {
                    let (frame, check, delta) = producer.push_recorded_motion(
                        motion.clone(),
                        None,
                        DEFAULT_FRAME_LIMIT,
                        cancel,
                    )?;
                    let overflow = recording.append(&Record {
                        motion,
                        delta,
                        events: events.clone(),
                    })?;
                    send(&frame)?;
                    if let Some(check) = check {
                        send(&check)?;
                    }
                    if !events.is_empty() {
                        send_events(producer, &op, events, &mut send)?;
                    }
                    // First stock AND its fresh check are available before
                    // optional cache identity/file work on an empty-cache run.
                    recording.after_publish(overflow)?;
                } else {
                    let (frame, check) =
                        producer.push_checked_motion(motion, DEFAULT_FRAME_LIMIT, cancel)?;
                    send(&frame)?;
                    if let Some(check) = check {
                        send(&check)?;
                    }
                    if !events.is_empty() {
                        send_events(producer, &op, events, &mut send)?;
                    }
                }
                count += 1;
                meter.report(plan.progress())?;
            }
            plan.finish_report(cancel)?;
            meter.end()?;
            let report = serde_json::to_value(plan.report()).map_err(|e| e.to_string())?;
            let footer = Footer {
                motions: count,
                events: plan.suffix_events(),
                report: report.clone(),
                end_stock: stock_digest(producer)?,
                end_position: producer.position(),
            };
            let (saved, final_key) = recording.finish(&footer)?;
            stats["key"] = final_key.into();
            stats["planned_motions"] = json!(count);
            stats["computed_sweeps"] = json!(count);
            stats["saved"] = json!(saved);
            control(
                producer,
                &op,
                "end",
                footer.events,
                report,
                &stats,
                &mut send,
            )?;
        }
        while let Some(start) = producer.start_material_verification(Some(op.id()))? {
            send(&start)?;
            send(&producer.finish_material_verification(cancel)?)?;
        }
    }
    while let Some(start) = producer.start_material_verification(None)? {
        send(&start)?;
        send(&producer.finish_material_verification(cancel)?)?;
    }
    send(&producer.finish(DEFAULT_FRAME_LIMIT)?)?;
    Ok(())
}

/// A lathe run: each turning operation is planned when it is reached (every
/// move checked against the part, bore, chuck and stock as it is planned)
/// and streamed move by move with the exact stock change it makes.
pub fn run_turning(
    producer: &mut StreamProducer,
    cancel: &impl CancelSignal,
    mut send: impl FnMut(&[u8]) -> Result<(), String>,
) -> Result<(), String> {
    turning(producer, cancel, &Meter::plain(&mut send))
}
fn turning<C: CancelSignal>(
    producer: &mut StreamProducer,
    cancel: &C,
    meter: &Meter<'_>,
) -> Result<(), String> {
    if producer.completed() != 0 {
        return Err("a lathe run starts unstarted".into());
    }
    let manifest = producer.manifest().clone();
    let turning = manifest
        .turning
        .as_ref()
        .ok_or("lathe run needs its turning setup")?;
    let spec = manifest
        .stock
        .turn()
        .ok_or("lathe run needs turning stock")?;
    let start = manifest.initial_position_mm;
    let job = turning.job(&manifest.run.id, spec, [start[0], start[2]]);
    let count = job.operations.len();
    // A turning operation is planned whole before its first move, with
    // nothing to count: its progress updates carry no fraction.
    let mut send = |bytes: &[u8]| meter.send(bytes);
    let signal = || cancel.is_cancelled();
    let mut session = crate::turning::ops::TurnSession::new(&job, &signal)?;
    for (index, op) in job.operations.iter().enumerate() {
        if cancel.is_cancelled() {
            return Err("lathe run cancelled".into());
        }
        meter.begin(op.id(), index)?;
        session.operation(op, job.operations.get(index + 1).map(|o| o.tool()))?;
        if index + 1 == count {
            session.finish()?;
        }
        let events = session.take_events();
        let mut deltas = session.take_deltas().into_iter();
        let mut report =
            serde_json::to_value(&session.reports()[index]).map_err(|e| e.to_string())?;
        // Turning leaves exactly the planned profile plus allowances; the
        // residual is not estimated separately.
        report["geometric_residual_status"] = json!("unknown");
        let motions = events
            .iter()
            .filter(|e| matches!(e, Event::Motion { .. }))
            .count();
        let stats = json!({"cache_hit": false, "planned_motions": motions, "saved": false});
        let mut pending = vec![];
        let mut begun = false;
        for event in events {
            let Event::Motion { motion } = event else {
                pending.push(event);
                continue;
            };
            if !begun {
                control_turn(
                    producer,
                    op.id(),
                    "begin",
                    std::mem::take(&mut pending),
                    report.clone(),
                    &stats,
                    &mut send,
                )?;
                begun = true;
            } else if !pending.is_empty() {
                control_turn(
                    producer,
                    op.id(),
                    "events",
                    std::mem::take(&mut pending),
                    Value::Null,
                    &stats,
                    &mut send,
                )?;
            }
            let delta = deltas
                .next()
                .ok_or("turning move without its stock change")?;
            send(&producer.push_turn_motion(motion, delta, DEFAULT_FRAME_LIMIT, cancel)?)?;
        }
        if deltas.next().is_some() {
            return Err("turning stock change without its move".into());
        }
        if !begun {
            control_turn(
                producer,
                op.id(),
                "begin",
                vec![],
                report.clone(),
                &stats,
                &mut send,
            )?;
        }
        meter.end()?;
        // The stock after this operation against the part.
        report["material"] = crate::turning::ops::material_report(&job, session.stock())?;
        control_turn(producer, op.id(), "end", pending, report, &stats, &mut send)?;
    }
    send(&producer.finish(DEFAULT_FRAME_LIMIT)?)?;
    Ok(())
}
fn control_turn(
    producer: &StreamProducer,
    operation: &str,
    stage: &str,
    events: Vec<Event>,
    mut report: Value,
    stats: &Value,
    send: &mut impl FnMut(&[u8]) -> Result<(), String>,
) -> Result<(), String> {
    if let Some(object) = report.as_object_mut() {
        object.insert("regeneration".into(), stats.clone());
    }
    send(&encode_operation_control(
        producer.manifest(),
        &OperationControl::<Value> {
            operation,
            completed: producer.completed(),
            stage,
            events,
            report: (!report.is_null()).then_some(report),
        },
        DEFAULT_FRAME_LIMIT,
    )?)
}

#[cfg(test)]
mod clearance_key_tests {
    use super::*;
    use crate::{
        adaptive::AdaptivePathStrategy,
        motion::{Curve, MoveKind},
        stock::{Stock, SweepOptions},
        stream::StreamManifest,
    };

    #[test]
    fn equal_cell_images_do_not_alias_distinct_native_clearance_premises() {
        let manifest: StreamManifest =
            serde_json::from_str(include_str!("../fixtures/operations/adaptive-pocket.json"))
                .unwrap();
        let prepared = manifest.tools[0].prepare().unwrap();
        let make = |shift: f64, id, operation: &str| {
            let mut stock = Stock::new(manifest.stock.height().unwrap().clone()).unwrap();
            let motion = Motion {
                id,
                operation: operation.into(),
                tool: manifest.tools[0].id.clone(),
                kind: MoveKind::Cut,
                feed_mm_min: Some(600.),
                curve: Curve::Line {
                    start: [8., 13. + shift, 5.],
                    end: [18., 13. + shift, 5.],
                },
            };
            let delta = stock
                .preview_sweep(&prepared, &motion, 1., SweepOptions::default(), &|| false)
                .unwrap();
            stock.apply_delta(&delta).unwrap();
            stock
        };
        let a = make(0., 1, "a");
        let b = make(0.0001, 1, "b");
        let renamed = make(0., 99, "renamed");
        assert_eq!(
            a.snapshot(),
            b.snapshot(),
            "the height image alone cannot distinguish these sweeps"
        );
        let mut operation = manifest.operations[0].clone();
        let Operation::AdaptivePocket(adaptive) = &mut operation else {
            panic!("fixture");
        };
        adaptive.path_strategy = AdaptivePathStrategy::StockFront;
        assert_ne!(
            clearance_dependency(&a, &operation),
            clearance_dependency(&b, &operation)
        );
        assert_eq!(
            clearance_dependency(&a, &operation),
            clearance_dependency(&renamed, &operation),
            "runtime identities/IDs/attribution cannot poison geometric reuse"
        );
        assert_eq!(
            clearance_dependency(&a, &operation),
            clearance_dependency(&a.clone(), &operation)
        );
        let restored = Stock::from_snapshot(a.snapshot()).unwrap();
        assert_ne!(
            clearance_dependency(&a, &operation),
            clearance_dependency(&restored, &operation)
        );
        let Operation::AdaptivePocket(adaptive) = &mut operation else {
            unreachable!();
        };
        adaptive.path_strategy = AdaptivePathStrategy::DiskFront;
        let raster = Operation::Pocket(adaptive.pocket.clone());
        assert_eq!(clearance_dependency(&a, &operation), None);
        assert_eq!(clearance_dependency(&a, &raster), None);
        let rough: StreamManifest =
            serde_json::from_str(include_str!("../fixtures/regressions/stock-front.json")).unwrap();
        assert!(
            clearance_dependency(&a, &rough.operations[0]).is_some(),
            "nested stock-front roughing also consumes the premise"
        );
    }
}

#[cfg(test)]
mod executable_identity_tests {
    use super::*;
    #[test]
    fn kernel_identity_matches_independent_oracles_and_all_actual_executable_bytes() {
        let oracle: Value =
            serde_json::from_str(include_str!("../fixtures/integrity/sha256-identity.json"))
                .unwrap();
        for case in oracle["cases"].as_array().unwrap() {
            let n = case["length"].as_u64().unwrap() as usize;
            let bytes: Vec<u8> = (0..n).map(|i| (i % 251) as u8).collect();
            assert_eq!(executable_identity(&bytes), case["sha256"]);
            assert_eq!(executable_identity(&bytes), hex(&bytes));
        }
        let bytes = fs::read(std::env::current_exe().unwrap()).unwrap();
        assert_eq!(executable_identity(&bytes), hex(&bytes));
        let mut changed = bytes;
        let last = changed.len() - 1;
        changed[last] ^= 1;
        assert_ne!(
            executable_identity(&changed),
            hex(fs::read(std::env::current_exe().unwrap()).unwrap())
        );
    }
}
