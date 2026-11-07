//! Versioned bounded binary motion/stock chunks. Production and receipt commit
//! atomically; run/revision, sequence, continuity and old stock values are checked.
//! Disk retention, playback clock and Python/viewer adapters build on this layer.
use crate::{
    motion::{Curve, Event, Motion, MoveKind, Plane, Vec3},
    operation::OperationReport,
    pocket::Operation,
    stock::{
        CancelSignal, CellChange, HeightBounds, Stock, StockDelta, StockSnapshot, StockSpec,
        SweepOptions, SweepStats,
    },
    tool::{PreparedAssembly, ToolAssembly, valid_id},
    turning::{
        ops::{TurnDelta, TurnSetup},
        stock::{RadiusBounds, TurnCell, TurnChange, TurnStock, TurnStockSpec},
    },
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::{HashMap, HashSet},
    io::Read,
    sync::Arc,
};

mod controls;
mod geometry;
pub use controls::{ControlCursor, ControlState, ToolSelection};

const HEADER: usize = 20;
/// Largest frame the binary format can express: the payload length field is
/// a u32. This is the only frame-size invariant; callers may pass smaller
/// explicit limits.
pub const MAX_FRAME_BYTES: usize = HEADER.saturating_add(u32::MAX as usize);
pub const DEFAULT_FRAME_LIMIT: usize = MAX_FRAME_BYTES;
pub const CHECKPOINT_FRAME_LIMIT: usize = MAX_FRAME_BYTES;
/// A canonical manifest is carried in one frame.
pub const MANIFEST_LIMIT: usize = u32::MAX as usize;
const MAGIC: &[u8; 8] = b"CAMLSTRM";
const VERSION: u16 = 1;
const CHUNK: u16 = 1;
const CHECKPOINT: u16 = 2;
const MANIFEST: u16 = 3;
const END: u16 = 4;
const PREVIEW: u16 = 5;
const RESTORED: u16 = 6;
const OPERATION: u16 = 7;
const VERIFICATION: u16 = 8;
const CONTROL: u16 = 9;
/// A committed lathe move: its motion and its turning stock change.
const TURN_CHUNK: u16 = 10;
const CONTROL_JSON_LIMIT: usize = 4096;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RunIdentity {
    pub id: String,
    pub revision: String,
}
/// The stock of a run: a height field (milling, thermal cutting) or
/// axisymmetric bar stock (turning).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum RunStockSpec {
    Height(StockSpec),
    Turn(TurnStockSpec),
}
impl From<StockSpec> for RunStockSpec {
    fn from(spec: StockSpec) -> Self {
        Self::Height(spec)
    }
}
impl RunStockSpec {
    pub fn height(&self) -> Result<&StockSpec, String> {
        match self {
            Self::Height(spec) => Ok(spec),
            Self::Turn(_) => {
                Err("this needs a height-field stock; the run has turning stock".into())
            }
        }
    }
    pub fn height_mut(&mut self) -> Result<&mut StockSpec, String> {
        match self {
            Self::Height(spec) => Ok(spec),
            Self::Turn(_) => {
                Err("this needs a height-field stock; the run has turning stock".into())
            }
        }
    }
    pub fn turn(&self) -> Option<&TurnStockSpec> {
        match self {
            Self::Turn(spec) => Some(spec),
            Self::Height(_) => None,
        }
    }
}
/// A run's committed stock.
#[derive(Clone, Debug)]
pub enum RunStock {
    Height(Box<Stock>),
    Turn(TurnStock),
}
impl RunStock {
    pub fn height(&self) -> Result<&Stock, String> {
        match self {
            Self::Height(stock) => Ok(stock),
            Self::Turn(_) => {
                Err("this needs a height-field stock; the run has turning stock".into())
            }
        }
    }
    fn height_mut(&mut self) -> Result<&mut Stock, String> {
        match self {
            Self::Height(stock) => Ok(stock),
            Self::Turn(_) => {
                Err("this needs a height-field stock; the run has turning stock".into())
            }
        }
    }
    pub fn turn(&self) -> Result<&TurnStock, String> {
        match self {
            Self::Turn(stock) => Ok(stock),
            Self::Height(_) => Err("this needs turning stock; the run has a height field".into()),
        }
    }
}
#[derive(Clone, Debug)]
pub struct StreamManifest {
    pub version: u32,
    pub run: RunIdentity,
    pub stock: RunStockSpec,
    pub tools: Vec<ToolAssembly>,
    pub initial_position_mm: Vec3,
    pub sweep: SweepOptions,
    pub operations: Vec<Operation>,
    pub verification: Option<crate::stream_verification::Config>,
    /// Where a tool change leaves the tool (operation runs). None: the tip
    /// stays where the previous operation ended.
    pub tool_change: Option<ToolChangePose>,
    /// Operation runs: leave cuts straight up at rapid instead of at feed.
    pub rapid_retracts: bool,
    /// A lathe run's turning setup (with turning stock); None otherwise.
    pub turning: Option<TurnSetup>,
}
/// The machine's tool-change pose, in work coordinates.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum ToolChangePose {
    /// The spindle gauge line at `gauge_z_mm` (e.g. Z home, where M6 sends
    /// it), over `xy_mm` or over the position where the change was called.
    Gauge(GaugePose),
    /// Each tool's tip after its change, declared per tool (a tool table:
    /// posts with `tool_change_position_mode="table"`).
    Table(TablePose),
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GaugePose {
    pub gauge_z_mm: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub xy_mm: Option<[f64; 2]>,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TablePose {
    /// Tool assembly id -> tip position after the change to it.
    pub tips_mm: std::collections::BTreeMap<String, Vec3>,
}
impl StreamManifest {
    /// With `rapid_retracts`, a plan's straight-up retract becomes a rapid,
    /// only when it leaves from exactly where the last committed motion left
    /// the same tool: that pose is empty and, in a height field, so is
    /// everything above it (`Stock::departs_clear`). Otherwise it stays at
    /// feed. The stock-contact check, when enabled, verifies it again.
    /// `declared_feed`: the operation declares its retract feed (hole
    /// cycles feeding out); those retracts are kept as planned.
    pub fn retract_policy(
        &self,
        mut motion: Motion,
        stock: &crate::stock::Stock,
        declared_feed: bool,
    ) -> Motion {
        if self.rapid_retracts
            && !declared_feed
            && motion.kind == MoveKind::Retract
            && matches!(motion.curve, Curve::Line { start, end }
                if start[0] == end[0] && start[1] == end[1] && end[2] > start[2]
                    && stock.departs_clear(&motion.tool, start))
        {
            motion.kind = MoveKind::Rapid;
            motion.feed_mm_min = None;
        }
        motion
    }
    /// The tip position after changing to `tool` at `position`, or None when
    /// the manifest declares no tool-change pose (the tip stays put).
    pub fn tool_change_tip(&self, tool: &str, position: Vec3) -> Result<Option<Vec3>, String> {
        if self.turning.is_some() {
            // A lathe indexes the turret at the start position: every tool's
            // controlled point arrives there.
            if !self.tools.iter().any(|t| t.id == tool) {
                return Err("tool change to an undeclared tool".into());
            }
            return Ok(Some(self.initial_position_mm));
        }
        let Some(pose) = &self.tool_change else {
            return Ok(None);
        };
        let t = self
            .tools
            .iter()
            .find(|t| t.id == tool)
            .ok_or("tool change to an undeclared tool")?;
        match pose {
            ToolChangePose::Gauge(g) => {
                let xy = g.xy_mm.unwrap_or([position[0], position[1]]);
                Ok(Some([xy[0], xy[1], g.gauge_z_mm - t.gauge_length_mm]))
            }
            ToolChangePose::Table(table) => {
                table.tips_mm.get(tool).copied().map(Some).ok_or_else(|| {
                    format!(
                        "the tool-change table declares no position after a change to tool {tool}"
                    )
                })
            }
        }
    }
    pub fn validate(&self) -> Result<(), String> {
        if ![1, 2].contains(&self.version)
            || !valid_id(&self.run.id)
            || self.run.revision.is_empty()
            || self.run.revision.len() > 512
            || !point_valid(self.initial_position_mm)
            || self.tools.is_empty()
        {
            return Err(
                "invalid stream version/run/revision, initial position or tool count".into(),
            );
        }
        match &self.stock {
            RunStockSpec::Height(spec) => {
                spec.grid()?;
                if self.turning.is_some() {
                    return Err("a turning setup needs turning stock".into());
                }
            }
            RunStockSpec::Turn(spec) => {
                spec.validate()?;
                let turning = self
                    .turning
                    .as_ref()
                    .ok_or("turning stock needs the run's turning setup")?;
                if self.version != 1
                    || !self.operations.is_empty()
                    || self.verification.is_some()
                    || self.tool_change.is_some()
                    || self.initial_position_mm[1] != 0.
                {
                    return Err("a lathe run is an operation run (version 1) of its turning operations, starting in the XZ plane, without milling operations, milling checks or a tool-change pose".into());
                }
                if self.tools != turning.proxy_tools() {
                    return Err(
                        "a lathe run's tools are its turning tools' posting stand-ins".into(),
                    );
                }
                let mut ids = HashSet::new();
                for op in &turning.operations {
                    if !valid_id(op.id())
                        || !ids.insert(op.id())
                        || !turning.tools.iter().any(|t| t.tool.id == op.tool())
                    {
                        return Err(
                            "invalid/duplicate turning operation ID or unknown operation tool"
                                .into(),
                        );
                    }
                }
                if turning.operations.is_empty() {
                    return Err("a lathe run needs at least one turning operation".into());
                }
            }
        }
        self.sweep.validate()?;
        let finite = |v: &f64| v.is_finite() && v.abs() <= 1e6;
        if let Some(pose) = &self.tool_change
            && (self.version != 1
                || match pose {
                    ToolChangePose::Gauge(g) => {
                        !finite(&g.gauge_z_mm) || g.xy_mm.is_some_and(|p| !p.iter().all(finite))
                    }
                    ToolChangePose::Table(t) => {
                        t.tips_mm.is_empty()
                            || t.tips_mm.iter().any(|(tool, tip)| {
                                !tip.iter().all(finite) || !self.tools.iter().any(|a| &a.id == tool)
                            })
                    }
                })
        {
            return Err(
                "tool change pose needs finite work coordinates, for declared tools (operation runs only)".into(),
            );
        }
        if self.version == 2 && !self.operations.is_empty() {
            return Err(
                "version 2 typed control input cannot use passive operation metadata".into(),
            );
        }
        let mut ids = HashSet::new();
        for t in &self.tools {
            if t.cutter.name.len() > 1024 || t.cutter.material.len() > 1024 {
                return Err("stream tool metadata exceeds bounded manifest limits".into());
            }
            t.prepare()?;
            if !ids.insert(&t.id) {
                return Err("duplicate stream tool assembly".into());
            }
        }
        let mut operations = HashSet::new();
        let mut work_offset = None;
        for op in &self.operations {
            if work_offset.is_some_and(|old| old != op.work_offset()) {
                return Err("one work frame/offset per operation run".into());
            }
            work_offset = Some(op.work_offset());
            if !valid_id(op.id())
                || !self.tools.iter().any(|t| t.id == op.tool())
                || !operations.insert(op.id())
            {
                return Err("invalid/duplicate operation ID or unknown operation tool".into());
            }
        }
        if let Some(config) = &self.verification {
            config.validate(&self.operations)?;
        }
        self.canonical_bytes()?;
        Ok(())
    }
    /// Exact manifest bytes whose SHA-256 is bound into every chunk/checkpoint.
    /// Adapters can retain these bytes rather than reserializing float values.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, String> {
        let bytes = serde_json::to_vec(self).map_err(|e| e.to_string())?;
        if bytes.len() > MANIFEST_LIMIT {
            return Err("stream manifest exceeds the u32 frame length field".into());
        }
        Ok(bytes)
    }
    pub fn digest(&self) -> Result<[u8; 32], String> {
        Ok(Sha256::digest(self.canonical_bytes()?).into())
    }
}

#[derive(Debug)]
pub struct MotionChunk {
    pub sequence: u64,
    pub from_complete: u64,
    pub to_complete: u64,
    pub control_cursor: Option<ControlCursor>,
    pub motion: Motion,
    pub delta: StockDelta,
}

pub type RecordedMotionFrames = (Vec<u8>, Option<Vec<u8>>, StockDelta);
type PreparedMotionFrames = (
    Vec<u8>,
    Option<Vec<u8>>,
    Option<StockDelta>,
    crate::stock::ReplayClearanceStats,
);
pub(crate) type VerifiedCacheFrames =
    (Vec<u8>, Option<Vec<u8>>, crate::stock::ReplayClearanceStats);

#[derive(Clone, Debug, PartialEq)]
pub struct ControlChunk {
    pub completed: u64,
    pub before: ControlCursor,
    pub position_before: Vec3,
    pub event: Event,
}

struct ControlDefinition {
    manifest: StreamManifest,
    identity: Vec<u8>,
}

/// Immutable control metadata for external stock-buffer consumers. No grid,
/// stock index or verifier is allocated; clones share the prepared definition.
#[derive(Clone)]
pub struct ControlContext {
    definition: Arc<ControlDefinition>,
    state: ControlState,
}
impl ControlContext {
    pub fn new(manifest: StreamManifest) -> Result<Self, String> {
        manifest.validate()?;
        if manifest.version != 2 {
            return Err("control context requires manifest version 2".into());
        }
        let identity = identity_bytes(&manifest)?;
        Ok(Self {
            definition: Arc::new(ControlDefinition { manifest, identity }),
            state: ControlState::default(),
        })
    }
    pub fn state(&self) -> &ControlState {
        &self.state
    }
    pub fn state_bytes(&self) -> Result<Vec<u8>, String> {
        let bytes = serde_json::to_vec(&self.state).map_err(|e| e.to_string())?;
        if bytes.len() > CONTROL_JSON_LIMIT {
            return Err("control state exceeds bounded size".into());
        }
        Ok(bytes)
    }
    pub fn restored(&self, bytes: &[u8], completed: u64, position: Vec3) -> Result<Self, String> {
        if bytes.len() > CONTROL_JSON_LIMIT {
            return Err("control state exceeds bounded size".into());
        }
        let state: ControlState = serde_json::from_slice(bytes).map_err(|e| e.to_string())?;
        state.validate(&self.definition.manifest, completed, position)?;
        Ok(Self {
            definition: Arc::clone(&self.definition),
            state,
        })
    }
    /// Caller owns its motion/finished cursor. This prepares an immutable control
    /// candidate; it cannot change that cursor or any stock buffer.
    pub fn accepted(
        &self,
        bytes: &[u8],
        completed: u64,
        position: Vec3,
        limit: usize,
    ) -> Result<(Self, Vec3, Event), String> {
        let chunk = decode_control_prepared(
            &self.definition.manifest,
            &self.definition.identity,
            bytes,
            limit,
        )?;
        let (state, after) = prepare_control_state(
            &self.definition.manifest,
            &self.state,
            completed,
            position,
            &chunk,
        )?;
        Ok((
            Self {
                definition: Arc::clone(&self.definition),
                state,
            },
            after,
            chunk.event,
        ))
    }
    pub fn check_tool(&self, tool: &str, rapid: bool) -> Result<(), String> {
        self.state.check_tool(tool, rapid)
    }
    pub fn cursor_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(40);
        put_control_cursor(&mut bytes, Some(self.state.cursor));
        bytes
    }
}
fn prepare_control_state(
    manifest: &StreamManifest,
    state: &ControlState,
    completed: u64,
    position: Vec3,
    chunk: &ControlChunk,
) -> Result<(ControlState, Vec3), String> {
    if chunk.completed != completed
        || chunk.before != state.cursor
        || chunk.position_before != position
    {
        return Err("control frame cursor, prefix or previous position mismatch".into());
    }
    state.transition(&chunk.event, manifest, completed, position)
}

struct CheckpointFields {
    sequence: u64,
    position: Vec3,
    has_last: u8,
    last: u64,
    generation: u64,
    count: usize,
    controls: Option<ControlState>,
}
impl CheckpointFields {
    fn read(body: &mut Decoder<'_>, manifest: &StreamManifest) -> Result<Self, String> {
        Ok(Self {
            sequence: body.u64()?,
            position: body.vec3()?,
            has_last: body.u8()?,
            last: body.u64()?,
            generation: body.u64()?,
            count: body.u32()? as usize,
            controls: if manifest.version == 2 {
                Some(body.control_json()?)
            } else {
                None
            },
        })
    }
    fn validate(&self, manifest: &StreamManifest, cell_bytes: usize) -> Result<(), String> {
        // Height cells: 16 bytes. Turning cells: 32 bytes, then a u32 ring
        // count and 32 bytes per ring.
        let shape = match &manifest.stock {
            RunStockSpec::Height(spec) => {
                let (nx, ny, _) = spec.grid()?;
                self.count == nx * ny && cell_bytes == self.count * 16
            }
            RunStockSpec::Turn(spec) => {
                let rest = cell_bytes.checked_sub(self.count * 32 + 4);
                self.count == spec.validate()? && rest.is_some_and(|r| r % 32 == 0)
            }
        };
        if self.generation != self.sequence
            || !point_valid(self.position)
            || self.has_last > 1
            || (self.has_last == 0) != (self.sequence == 0)
            || !shape
            || self.sequence == 0 && self.last != 0
            || self.sequence == 0
                && manifest.version == 1
                && self.position != manifest.initial_position_mm
        {
            return Err("invalid stock checkpoint cursor/shape/position".into());
        }
        if let Some(controls) = &self.controls {
            controls.validate(manifest, self.sequence, self.position)?;
        }
        Ok(())
    }
}

/// Pull-based native producer: no growing motion/chunk queue. Callers add bounded
/// queues or blocking writes without requesting another chunk while full.
pub struct StreamProducer {
    state: StreamReceiver,
    tools: HashMap<String, PreparedAssembly>,
    checks: Option<crate::stream_verification::Runtime>,
    read_only: bool,
    /// A lathe run given its moves (a posted program read back): the
    /// planner's move kernel computes and checks each move's stock change.
    turn_replay: Option<crate::turning::ops::TurnReplay>,
}
#[derive(Clone)]
pub struct StreamReceiver {
    manifest: StreamManifest,
    identity: Vec<u8>,
    stock: RunStock,
    sequence: u64,
    position: Vec3,
    last_motion_id: Option<u64>,
    finished: bool,
    controls: Option<ControlState>,
    /// The last committed motion's tool (None after a restore).
    last_tool: Option<String>,
}
impl StreamProducer {
    pub fn new(manifest: StreamManifest) -> Result<Self, String> {
        Self::from_state(StreamReceiver::new(manifest)?, false)
    }
    /// Disposable reconstruction/preview worker: tools and stock only. It cannot
    /// generate new motion or verification, so it needs neither verifier/index.
    pub fn new_query(manifest: StreamManifest) -> Result<Self, String> {
        Self::from_state(StreamReceiver::new(manifest)?, true)
    }
    pub fn from_checkpoint(
        manifest: StreamManifest,
        bytes: &[u8],
        limit: usize,
    ) -> Result<Self, String> {
        Self::from_state(
            StreamReceiver::from_checkpoint(manifest, bytes, limit)?,
            false,
        )
    }
    pub fn from_checkpoint_reader(
        manifest: StreamManifest,
        reader: &mut impl Read,
        limit: usize,
    ) -> Result<Self, String> {
        Self::from_state(
            StreamReceiver::from_checkpoint_reader(manifest, reader, limit)?,
            false,
        )
    }
    pub fn query_from_checkpoint_reader(
        manifest: StreamManifest,
        reader: &mut impl Read,
        limit: usize,
    ) -> Result<Self, String> {
        Self::from_state(
            StreamReceiver::from_checkpoint_reader(manifest, reader, limit)?,
            true,
        )
    }
    pub(crate) fn writable(&self) -> Result<(), String> {
        if self.read_only {
            Err("read-only query worker cannot generate or finish a run".into())
        } else {
            Ok(())
        }
    }
    fn from_state(mut state: StreamReceiver, read_only: bool) -> Result<Self, String> {
        if !read_only
            && state
                .manifest
                .verification
                .as_ref()
                .is_some_and(|c| c.stock_contact.is_some())
        {
            state
                .stock
                .height_mut()?
                .enable_contact_queries(&std::sync::atomic::AtomicBool::new(false))?;
        }
        let tools = state
            .manifest
            .tools
            .iter()
            .map(|t| Ok((t.id.clone(), t.prepare()?)))
            .collect::<Result<_, String>>()?;
        let checks = if read_only {
            None
        } else {
            state
                .manifest
                .verification
                .as_ref()
                .map(|_| {
                    crate::stream_verification::Runtime::prepare(&state.manifest, state.sequence)
                })
                .transpose()?
        };
        Ok(Self {
            state,
            tools,
            checks,
            read_only,
            turn_replay: None,
        })
    }
    /// The committed height-field stock (milling and thermal runs).
    pub fn stock(&self) -> Result<&Stock, String> {
        self.state.stock.height()
    }
    pub fn run_stock(&self) -> &RunStock {
        &self.state.stock
    }
    pub fn completed(&self) -> u64 {
        self.state.sequence
    }
    pub fn position(&self) -> Vec3 {
        self.state.position
    }
    pub fn controls(&self) -> Option<&ControlState> {
        self.state.controls.as_ref()
    }
    /// A declared state/pose transfer, never a stock sweep or physical ATC path.
    pub fn push_control(
        &mut self,
        event: Event,
        limit: usize,
        cancel: &impl CancelSignal,
    ) -> Result<Vec<u8>, String> {
        self.writable()?;
        if cancel.is_cancelled() {
            return Err("control transfer cancelled; no prefix committed".into());
        }
        let chunk = ControlChunk {
            completed: self.state.sequence,
            before: self
                .state
                .control_cursor()
                .ok_or("controls require manifest version 2")?,
            position_before: self.state.position,
            event,
        };
        let (next, position) = self.state.prepare_control(&chunk)?;
        let bytes = encode_control_prepared(&self.state.identity, &chunk, limit)?;
        if cancel.is_cancelled() {
            return Err("control transfer cancelled; no prefix committed".into());
        }
        self.state.controls = Some(next);
        self.state.position = position;
        Ok(bytes)
    }
    pub fn manifest(&self) -> &StreamManifest {
        &self.state.manifest
    }
    pub fn checkpoint(&self, limit: usize) -> Result<Vec<u8>, String> {
        self.state.checkpoint(limit)
    }
    /// Synchronize a disposable query cache from an already committed source
    /// chunk. This reuses receiver CRC/identity/sequence/old-value checks and does
    /// not plan or sweep a new motion. The CLI permits it only in query workers.
    pub fn accept_cached_chunk(&mut self, bytes: &[u8], limit: usize) -> Result<(), String> {
        if check_header(bytes, limit)?.0 == CONTROL {
            self.state.accept_control(bytes, limit).map(|_| ())
        } else {
            self.state.accept(bytes, limit).map(|_| ())
        }
    }
    pub fn restored_cursor(&self, limit: usize) -> Result<Vec<u8>, String> {
        let mut body = self.state.identity.clone();
        put_u64(&mut body, self.state.sequence);
        put_vec3(&mut body, self.state.position);
        put_control_cursor(&mut body, self.state.control_cursor());
        frame(RESTORED, body, limit)
    }
    pub fn restore_checkpoint(&mut self, bytes: &[u8], limit: usize) -> Result<(), String> {
        self.state.restore_checkpoint(bytes, limit)?;
        if !self.read_only
            && self
                .state
                .manifest
                .verification
                .as_ref()
                .is_some_and(|c| c.stock_contact.is_some())
        {
            self.state
                .stock
                .height_mut()?
                .enable_contact_queries(&std::sync::atomic::AtomicBool::new(false))?;
        }
        if let Some(checks) = &mut self.checks {
            checks.tracker = crate::stream_verification::Tracker::new(
                checks.tracker.binding.clone(),
                self.state.sequence,
            );
        }
        Ok(())
    }
    /// Noncommitting prefix query. Consumers overlay it on the move-start stock;
    /// subsequent committed chunks still use the unchanged producer cursor.
    pub fn preview_motion(
        &self,
        motion: &Motion,
        fraction: f64,
        limit: usize,
        cancel: &impl CancelSignal,
    ) -> Result<Vec<u8>, String> {
        let delta = self.state.preview_partial(motion, fraction, cancel)?;
        let position = motion
            .curve
            .prepare(self.state.manifest.sweep.tolerance_mm)?
            .position(fraction)?;
        let mut body = self.state.identity.clone();
        put_u64(&mut body, self.state.sequence);
        put_control_cursor(&mut body, self.state.control_cursor());
        put_motion(&mut body, motion)?;
        put_f64(&mut body, fraction);
        put_vec3(&mut body, position);
        put_delta_cells(&mut body, &delta, limit)?;
        frame(PREVIEW, body, limit)
    }
    pub fn finish(&mut self, limit: usize) -> Result<Vec<u8>, String> {
        self.writable()?;
        if self.state.finished {
            return Err("stream already finished".into());
        }
        if let Some(checks) = &self.checks {
            checks.tracker.finish(self.state.sequence)?;
        }
        let mut body = self.state.identity.clone();
        put_u64(&mut body, self.state.sequence);
        put_vec3(&mut body, self.state.position);
        put_control_cursor(&mut body, self.state.control_cursor());
        let bytes = frame(END, body, limit)?;
        self.state.finished = true;
        Ok(bytes)
    }
    pub fn push_motion(
        &mut self,
        motion: Motion,
        limit: usize,
        cancel: &impl CancelSignal,
    ) -> Result<Vec<u8>, String> {
        if self.checks.is_some() {
            return Err("configured verification requires push_checked_motion".into());
        }
        self.push_unchecked_motion(motion, limit, cancel)
    }
    /// A tool change in an operation run: the tip moves to the manifest's
    /// tool-change pose (a declared machine premise, not a planned path; the
    /// consumer recomputes it). Returns the position after the change.
    pub fn change_tool(&mut self, tool: &str) -> Result<Vec3, String> {
        self.writable()?;
        if let Some(tip) = self
            .state
            .manifest
            .tool_change_tip(tool, self.state.position)?
        {
            if !point_valid(tip) {
                return Err("tool change pose out of range".into());
            }
            self.state.position = tip;
        }
        Ok(self.state.position)
    }
    /// Both frames are prepared before stock commits. Publish the chunk first,
    /// then its optional passive check record before producing another motion.
    pub fn push_checked_motion(
        &mut self,
        motion: Motion,
        limit: usize,
        cancel: &impl CancelSignal,
    ) -> Result<(Vec<u8>, Option<Vec<u8>>), String> {
        if self.checks.is_none() {
            return self
                .push_unchecked_motion(motion, limit, cancel)
                .map(|b| (b, None));
        }
        let (bytes, check, _, _) =
            self.push_prepared_motion(motion, None, false, false, limit, cancel)?;
        Ok((bytes, check))
    }
    /// Owned regeneration artifacts contain typed deltas, never previous-run
    /// frames. All verification is freshly computed before a rebased commit.
    pub fn push_recorded_motion(
        &mut self,
        motion: Motion,
        cached: Option<StockDelta>,
        limit: usize,
        cancel: &impl CancelSignal,
    ) -> Result<RecordedMotionFrames, String> {
        let (bytes, check, delta, _) =
            self.push_prepared_motion(motion, cached, true, false, limit, cancel)?;
        Ok((bytes, check, delta.ok_or("recorded delta missing")?))
    }
    pub fn push_cached_motion(
        &mut self,
        motion: Motion,
        delta: StockDelta,
        limit: usize,
        cancel: &impl CancelSignal,
    ) -> Result<(Vec<u8>, Option<Vec<u8>>), String> {
        let (bytes, check, _, _) =
            self.push_prepared_motion(motion, Some(delta), false, false, limit, cancel)?;
        Ok((bytes, check))
    }
    pub(crate) fn push_verified_cache_motion(
        &mut self,
        record: crate::regeneration::VerifiedCacheMotion,
        limit: usize,
        cancel: &impl CancelSignal,
    ) -> Result<VerifiedCacheFrames, String> {
        let (motion, delta) = record.into_parts();
        let (bytes, check, _, validation) =
            self.push_prepared_motion(motion, Some(delta), false, true, limit, cancel)?;
        Ok((bytes, check, validation))
    }
    fn push_prepared_motion(
        &mut self,
        motion: Motion,
        cached: Option<StockDelta>,
        record: bool,
        verified_cache: bool,
        limit: usize,
        cancel: &impl CancelSignal,
    ) -> Result<PreparedMotionFrames, String> {
        self.writable()?;
        self.state.check_motion(&motion)?;
        let completed = self
            .state
            .sequence
            .checked_add(1)
            .ok_or("stream cursor exhausted")?;
        let report = {
            #[cfg(feature = "profile-planning")]
            let _profile = crate::planning_profile::span("stream.verification");
            if let Some(checks) = &mut self.checks {
                checks.ready_for_motion()?;
                checks.motion(
                    self.state.stock.height()?,
                    self.tools.get(&motion.tool).ok_or("stream tool missing")?,
                    &motion,
                    completed,
                    cancel,
                )?
            } else {
                None
            }
        };
        if let Some(bytes) = &report {
            check_size(bytes.len() - HEADER, limit)?;
        }
        let next_tracker = {
            #[cfg(feature = "profile-planning")]
            let _profile = crate::planning_profile::span("stream.report_binding");
            if let (Some(checks), Some(bytes)) = (&self.checks, &report) {
                let mut tracker = checks.tracker.clone();
                tracker.accept(bytes, completed, Some(&motion))?;
                Some(tracker)
            } else {
                None
            }
        };
        let mut delta = {
            #[cfg(feature = "profile-planning")]
            let _profile = crate::planning_profile::span("stream.delta.prepare");
            match cached {
                Some(delta) => delta,
                None => self.state.stock.height()?.preview_sweep(
                    self.tools.get(&motion.tool).ok_or("stream tool missing")?,
                    &motion,
                    1.,
                    self.state.manifest.sweep,
                    cancel,
                )?,
            }
        };
        if delta.spec != *self.state.manifest.stock.height()?
            || delta.fraction != 1.
            || delta.operation != motion.operation
        {
            return Err("regeneration delta specification/fraction/operation mismatch".into());
        }
        delta.base_generation = self.state.sequence;
        delta.motion_id = motion.id;
        let validation = {
            #[cfg(feature = "profile-planning")]
            let _profile = crate::planning_profile::span("stream.cached_delta_validation");
            if verified_cache {
                self.state.stock.height()?.certify_verified_replay(
                    &mut delta,
                    self.tools.get(&motion.tool).ok_or("stream tool missing")?,
                    &motion,
                    self.state.manifest.sweep,
                    cancel,
                )?
            } else {
                crate::stock::ReplayClearanceStats::default()
            }
        };
        let chunk = MotionChunk {
            sequence: self.state.sequence,
            from_complete: self.state.sequence,
            to_complete: completed,
            control_cursor: self.state.control_cursor(),
            motion,
            delta,
        };
        let bytes = {
            #[cfg(feature = "profile-planning")]
            let _profile = crate::planning_profile::span("stream.encode");
            encode_chunk_prepared(&self.state.manifest, &self.state.identity, &chunk, limit)?
        };
        if cancel.is_cancelled() {
            return Err("stream production cancelled; no prefix committed".into());
        }
        let saved = record.then(|| chunk.delta.clone());
        {
            #[cfg(feature = "profile-planning")]
            let _profile = crate::planning_profile::span("stream.commit");
            self.state.commit_chunk(chunk)?;
        }
        if let Some(tracker) = next_tracker {
            self.checks.as_mut().unwrap().tracker = tracker;
        }
        Ok((bytes, report, saved, validation))
    }
    pub fn start_material_verification(
        &mut self,
        after_operation: Option<&str>,
    ) -> Result<Option<Vec<u8>>, String> {
        self.writable()?;
        if self.state.finished {
            return Err("stream already finished".into());
        }
        self.checks
            .as_mut()
            .map(|c| c.start_material(after_operation, self.state.sequence))
            .transpose()
            .map(Option::flatten)
    }
    pub fn finish_material_verification(
        &mut self,
        cancel: &impl CancelSignal,
    ) -> Result<Vec<u8>, String> {
        self.writable()?;
        self.checks
            .as_mut()
            .ok_or("run has no verification configuration")?
            .material(self.state.stock.height()?, self.state.sequence, cancel)
    }
    /// Commit one planned lathe move with the stock change its planner
    /// computed (and checked). Returns the committed frame.
    pub fn push_turn_motion(
        &mut self,
        motion: Motion,
        delta: TurnDelta,
        limit: usize,
        cancel: &impl CancelSignal,
    ) -> Result<Vec<u8>, String> {
        self.writable()?;
        let chunk = TurnMotionChunk {
            sequence: self.state.sequence,
            motion,
            delta,
        };
        let bytes = encode_turn_chunk(&self.state.manifest, &self.state.identity, &chunk, limit)?;
        if cancel.is_cancelled() {
            return Err("stream production cancelled; no prefix committed".into());
        }
        self.state.commit_turn_chunk(chunk)?;
        Ok(bytes)
    }
    /// Commit one given lathe move (an unstarted lathe run fed its moves):
    /// its stock change and checks come from the turning planner's kernel.
    pub fn push_turn_replayed(
        &mut self,
        motion: Motion,
        limit: usize,
        cancel: &impl CancelSignal,
    ) -> Result<Vec<u8>, String> {
        self.writable()?;
        if self.turn_replay.is_none() {
            let m = &self.state.manifest;
            let turning = m
                .turning
                .as_ref()
                .ok_or("given lathe moves need a lathe run")?;
            let spec = m
                .stock
                .turn()
                .ok_or("given lathe moves need turning stock")?;
            if self.state.sequence != 0 {
                return Err("a lathe replay starts with an unstarted run".into());
            }
            let p = m.initial_position_mm;
            self.turn_replay = Some(crate::turning::ops::TurnReplay::new(turning.job(
                &m.run.id,
                spec,
                [p[0], p[2]],
            ))?);
        }
        self.state.check_motion(&motion)?;
        let delta = self
            .turn_replay
            .as_mut()
            .ok_or("lathe replay missing")?
            .motion(&motion)?;
        self.push_turn_motion(motion, delta, limit, cancel)
    }
    fn push_unchecked_motion(
        &mut self,
        motion: Motion,
        limit: usize,
        cancel: &impl CancelSignal,
    ) -> Result<Vec<u8>, String> {
        self.writable()?;
        self.state.check_motion(&motion)?;
        let tool = self.tools.get(&motion.tool).ok_or("stream tool missing")?;
        let delta = self.state.stock.height()?.preview_sweep(
            tool,
            &motion,
            1.,
            self.state.manifest.sweep,
            cancel,
        )?;
        let chunk = MotionChunk {
            sequence: self.state.sequence,
            from_complete: self.state.sequence,
            to_complete: self
                .state
                .sequence
                .checked_add(1)
                .ok_or("stream cursor exhausted")?,
            control_cursor: self.state.control_cursor(),
            motion,
            delta,
        };
        let bytes =
            encode_chunk_prepared(&self.state.manifest, &self.state.identity, &chunk, limit)?;
        if cancel.is_cancelled() {
            return Err("stream production cancelled; no prefix committed".into());
        }
        self.state.commit_chunk(chunk)?;
        Ok(bytes)
    }
}
impl StreamReceiver {
    pub fn new(manifest: StreamManifest) -> Result<Self, String> {
        manifest.validate()?;
        let stock = match &manifest.stock {
            RunStockSpec::Height(spec) => RunStock::Height(Box::new(Stock::new(spec.clone())?)),
            RunStockSpec::Turn(spec) => RunStock::Turn(TurnStock::new(spec.clone())?),
        };
        let identity = identity_bytes(&manifest)?;
        Ok(Self {
            position: manifest.initial_position_mm,
            controls: (manifest.version == 2).then(ControlState::default),
            identity,
            manifest,
            stock,
            sequence: 0,
            last_motion_id: None,
            finished: false,
            last_tool: None,
        })
    }
    /// The committed height-field stock (milling and thermal runs).
    pub fn stock(&self) -> Result<&Stock, String> {
        self.stock.height()
    }
    pub fn run_stock(&self) -> &RunStock {
        &self.stock
    }
    pub fn completed(&self) -> u64 {
        self.sequence
    }
    pub fn is_finished(&self) -> bool {
        self.finished
    }
    pub fn position(&self) -> Vec3 {
        self.position
    }
    pub fn controls(&self) -> Option<&ControlState> {
        self.controls.as_ref()
    }
    fn control_cursor(&self) -> Option<ControlCursor> {
        self.controls.as_ref().map(|s| s.cursor)
    }
    fn prepare_control(&self, chunk: &ControlChunk) -> Result<(ControlState, Vec3), String> {
        if self.finished {
            return Err("control frame cursor, prefix or previous position mismatch".into());
        }
        prepare_control_state(
            &self.manifest,
            self.controls
                .as_ref()
                .ok_or("controls require manifest version 2")?,
            self.sequence,
            self.position,
            chunk,
        )
    }
    pub fn accept_control(&mut self, bytes: &[u8], limit: usize) -> Result<Event, String> {
        let chunk = decode_control_prepared(&self.manifest, &self.identity, bytes, limit)?;
        let (next, position) = self.prepare_control(&chunk)?;
        self.controls = Some(next);
        self.position = position;
        Ok(chunk.event)
    }
    pub fn manifest(&self) -> &StreamManifest {
        &self.manifest
    }
    pub fn accept(&mut self, frame: &[u8], limit: usize) -> Result<Motion, String> {
        if self.manifest.stock.turn().is_some() {
            let chunk = decode_turn_chunk(&self.manifest, &self.identity, frame, limit)?;
            return self.commit_turn_chunk(chunk);
        }
        let chunk = decode_chunk_prepared(&self.manifest, &self.identity, frame, limit)?;
        self.commit_chunk(chunk)
    }
    fn commit_turn_chunk(&mut self, chunk: TurnMotionChunk) -> Result<Motion, String> {
        if chunk.sequence != self.sequence || chunk.delta.motion_id != chunk.motion.id {
            return Err("turning chunk missing, repeated, out of order or misattributed".into());
        }
        let to_complete = self
            .sequence
            .checked_add(1)
            .ok_or("stream cursor exhausted")?;
        self.check_motion(&chunk.motion)?;
        if chunk.motion.kind == MoveKind::Rapid
            && (!chunk.delta.changes.is_empty() || !chunk.delta.rings.is_empty())
        {
            return Err("a rapid removes no turning stock".into());
        }
        let RunStock::Turn(stock) = &mut self.stock else {
            return Err("turning chunk on a height-field run".into());
        };
        stock.apply_move(&chunk.delta.changes, &chunk.delta.rings)?;
        self.sequence = to_complete;
        self.position = chunk.motion.curve.end();
        self.last_motion_id = Some(chunk.motion.id);
        self.last_tool = Some(chunk.motion.tool.clone());
        Ok(chunk.motion)
    }
    pub fn accept_end(&mut self, bytes: &[u8], limit: usize) -> Result<(), String> {
        let mut body = payload(bytes, END, limit)?;
        read_identity(&mut body, &self.manifest)?;
        let sequence = body.u64()?;
        let position = body.vec3()?;
        let control_cursor = body.control_cursor(&self.manifest)?;
        if self.finished
            || sequence != self.sequence
            || position != self.position
            || control_cursor != self.control_cursor()
            || body.left() != 0
        {
            return Err(
                "end-of-stream cursor mismatch, incomplete prefix or repeated finish".into(),
            );
        }
        self.finished = true;
        Ok(())
    }
    fn check_motion(&self, motion: &Motion) -> Result<(), String> {
        if self.finished
            || !valid_id(&motion.operation)
            || !self.manifest.tools.iter().any(|t| t.id == motion.tool)
            || (!self.manifest.operations.is_empty()
                && !self
                    .manifest
                    .operations
                    .iter()
                    .any(|op| op.id() == motion.operation && op.tool() == motion.tool))
            || self.manifest.turning.as_ref().is_some_and(|t| {
                !t.operations
                    .iter()
                    .any(|op| op.id() == motion.operation && op.tool() == motion.tool)
            })
            || (motion.curve.start() != self.position && !self.tool_change_start(motion)?)
            || self.last_motion_id.is_some_and(|id| motion.id <= id)
        {
            return Err(
                "stream motion has invalid attribution, continuity or non-increasing ID".into(),
            );
        }
        motion.curve.prepare(self.manifest.sweep.tolerance_mm)?;
        motion.validate_feed(None)?;
        if let Some(controls) = &self.controls {
            controls.check_motion(motion)?;
        }
        Ok(())
    }
    /// The first motion with a new tool may start at the manifest's declared
    /// tool-change pose instead of the previous end.
    fn tool_change_start(&self, motion: &Motion) -> Result<bool, String> {
        Ok(self.last_tool.as_deref() != Some(motion.tool.as_str())
            && self
                .manifest
                .tool_change_tip(&motion.tool, self.position)?
                .is_some_and(|tip| tip == motion.curve.start()))
    }
    fn commit_chunk(&mut self, chunk: MotionChunk) -> Result<Motion, String> {
        check_chunk_header(&self.manifest, &chunk)?;
        if chunk.sequence != self.sequence || chunk.control_cursor != self.control_cursor() {
            return Err("stream chunk missing, repeated or out of order".into());
        }
        self.check_motion(&chunk.motion)?;
        self.stock.height_mut()?.apply_delta(&chunk.delta)?;
        self.sequence = chunk.to_complete;
        self.position = chunk.motion.curve.end();
        self.last_motion_id = Some(chunk.motion.id);
        self.last_tool = Some(chunk.motion.tool.clone());
        Ok(chunk.motion)
    }
    pub fn checkpoint(&self, limit: usize) -> Result<Vec<u8>, String> {
        let mut body = self.identity.clone();
        put_u64(&mut body, self.sequence);
        put_vec3(&mut body, self.position);
        put_u8(&mut body, u8::from(self.last_motion_id.is_some()));
        put_u64(&mut body, self.last_motion_id.unwrap_or(0));
        match &self.stock {
            RunStock::Height(stock) => {
                put_u64(&mut body, stock.generation());
                put_u32(&mut body, stock.cells().len() as u32);
                if let Some(controls) = &self.controls {
                    put_control_json(&mut body, controls)?;
                }
                check_size(body.len() + stock.cells().len() * 16, limit)?;
                body.reserve(stock.cells().len() * 16);
                for h in stock.cells() {
                    put_height(&mut body, *h);
                }
            }
            RunStock::Turn(stock) => {
                // One stock generation per committed move, as in a height field.
                put_u64(&mut body, self.sequence);
                put_u32(&mut body, stock.cells().len() as u32);
                let bytes = stock.cells().len() * 32 + 4 + stock.rings().len() * 32;
                check_size(body.len() + bytes, limit)?;
                body.reserve(bytes);
                for c in stock.cells() {
                    put_turn_cell(&mut body, *c);
                }
                put_u32(&mut body, stock.rings().len() as u32);
                for r in stock.rings() {
                    for v in r {
                        put_f64(&mut body, *v);
                    }
                }
            }
        }
        frame(CHECKPOINT, body, limit)
    }
    pub fn restore_checkpoint(&mut self, bytes: &[u8], limit: usize) -> Result<(), String> {
        *self = Self::from_checkpoint(self.manifest.clone(), bytes, limit)?;
        Ok(())
    }
    /// Load retained stock directly; no unused initial grid allocation.
    pub fn from_checkpoint(
        manifest: StreamManifest,
        bytes: &[u8],
        limit: usize,
    ) -> Result<Self, String> {
        manifest.validate()?;
        let mut body = payload(bytes, CHECKPOINT, limit)?;
        read_identity(&mut body, &manifest)?;
        let fields = CheckpointFields::read(&mut body, &manifest)?;
        fields.validate(&manifest, body.left())?;
        if manifest.stock.turn().is_some() {
            return Self::from_checkpoint_turn(manifest, fields, &mut body);
        }
        let mut cells = Vec::with_capacity(fields.count);
        for _ in 0..fields.count {
            cells.push(body.height()?);
        }
        Self::from_checkpoint_cells(manifest, fields, cells)
    }
    fn from_checkpoint_turn(
        manifest: StreamManifest,
        fields: CheckpointFields,
        body: &mut Decoder<'_>,
    ) -> Result<Self, String> {
        let spec = manifest
            .stock
            .turn()
            .ok_or("turning checkpoint needs turning stock")?;
        let mut cells = Vec::with_capacity(fields.count);
        for _ in 0..fields.count {
            cells.push(body.turn_cell()?);
        }
        let count = body.u32()? as usize;
        if body.left() != count * 32 {
            return Err("invalid turning checkpoint ring count/length".into());
        }
        let mut rings = Vec::with_capacity(count);
        for _ in 0..count {
            rings.push([body.f64()?, body.f64()?, body.f64()?, body.f64()?]);
        }
        let stock = TurnStock::from_parts(spec.clone(), cells, rings)?;
        Ok(Self {
            identity: identity_bytes(&manifest)?,
            manifest,
            stock: RunStock::Turn(stock),
            sequence: fields.sequence,
            position: fields.position,
            last_motion_id: (fields.has_last == 1).then_some(fields.last),
            finished: false,
            controls: fields.controls,
            last_tool: None,
        })
    }
    /// A single retained file, parsed into final cell storage in bounded blocks.
    /// No second, full checkpoint payload Vec is allocated.
    pub fn from_checkpoint_reader(
        manifest: StreamManifest,
        reader: &mut impl Read,
        limit: usize,
    ) -> Result<Self, String> {
        manifest.validate()?;
        if manifest.stock.turn().is_some() {
            // Turning stock is small (one cell per z bin): read the whole
            // frame (bounded by its header) and decode it.
            let bytes = read_frame(reader, limit)?.ok_or("empty checkpoint file")?;
            if reader.read(&mut [0u8; 1]).map_err(|e| e.to_string())? != 0 {
                return Err("trailing checkpoint file data".into());
            }
            return Self::from_checkpoint(manifest, &bytes, limit);
        }
        let mut header = [0u8; HEADER];
        reader
            .read_exact(&mut header)
            .map_err(|e| format!("truncated checkpoint header: {e}"))?;
        let (kind, length) = check_header(&header, limit)?;
        if kind != CHECKPOINT {
            return Err("expected checkpoint file kind".into());
        }
        let expected = u32::from_le_bytes(header[16..20].try_into().unwrap());
        let mut prefix = Vec::new();
        for maximum in [128, 512] {
            read_prefix(reader, &mut prefix, 2, length)?;
            let n = prefix.len();
            let count = u16::from_le_bytes(prefix[n - 2..].try_into().unwrap()) as usize;
            if count > maximum {
                return Err("checkpoint identity exceeds byte limit".into());
            }
            read_prefix(reader, &mut prefix, count, length)?;
        }
        read_prefix(reader, &mut prefix, 32 + 8 + 24 + 1 + 8 + 8 + 4, length)?;
        if manifest.version == 2 {
            read_prefix(reader, &mut prefix, 4, length)?;
            let n = prefix.len();
            let count = u32::from_le_bytes(prefix[n - 4..].try_into().unwrap()) as usize;
            if count > CONTROL_JSON_LIMIT {
                return Err("control checkpoint state exceeds bounded size".into());
            }
            read_prefix(reader, &mut prefix, count, length)?;
        }
        let mut meta = Decoder {
            bytes: &prefix,
            offset: 0,
        };
        read_identity(&mut meta, &manifest)?;
        let fields = CheckpointFields::read(&mut meta, &manifest)?;
        fields.validate(&manifest, length - prefix.len())?;
        let mut crc = crc32fast::Hasher::new();
        crc.update(&header[..16]);
        crc.update(&prefix);
        let mut cells = Vec::with_capacity(fields.count);
        let mut block = [0u8; 65536];
        let mut remaining = fields.count * 16;
        while remaining > 0 {
            let count = remaining.min(block.len());
            reader
                .read_exact(&mut block[..count])
                .map_err(|e| format!("truncated checkpoint cells: {e}"))?;
            crc.update(&block[..count]);
            for pair in block[..count].as_chunks::<16>().0 {
                cells.push(HeightBounds {
                    lower_mm: f64::from_le_bytes(pair[..8].try_into().unwrap()),
                    upper_mm: f64::from_le_bytes(pair[8..].try_into().unwrap()),
                });
            }
            remaining -= count;
        }
        if crc.finalize() != expected {
            return Err("checkpoint file checksum mismatch".into());
        }
        if reader.read(&mut [0u8; 1]).map_err(|e| e.to_string())? != 0 {
            return Err("trailing checkpoint file data".into());
        }
        Self::from_checkpoint_cells(manifest, fields, cells)
    }
    fn from_checkpoint_cells(
        manifest: StreamManifest,
        fields: CheckpointFields,
        cells: Vec<HeightBounds>,
    ) -> Result<Self, String> {
        let identity = identity_bytes(&manifest)?;
        let stock = Stock::from_snapshot(StockSnapshot {
            spec: manifest.stock.height()?.clone(),
            generation: fields.generation,
            cells,
        })?;
        Ok(Self {
            manifest,
            identity,
            stock: RunStock::Height(Box::new(stock)),
            sequence: fields.sequence,
            position: fields.position,
            last_motion_id: (fields.has_last == 1).then_some(fields.last),
            finished: false,
            controls: fields.controls,
            last_tool: None,
        })
    }
    /// A lathe move's first `fraction` from the stock at its start (the
    /// committed receiver is unchanged): the turning planner's move kernel
    /// on the shortened move, with its checks. A rapid removes nothing.
    pub fn preview_turn(&self, next: &Motion, fraction: f64) -> Result<TurnStock, String> {
        self.check_motion(next)?;
        if !(0. ..=1.).contains(&fraction) {
            return Err("partial move fraction must be in [0, 1]".into());
        }
        let stock = self.stock.turn()?;
        if next.kind == MoveKind::Rapid || fraction == 0. {
            return Ok(stock.clone());
        }
        let Curve::Line { start, end } = next.curve else {
            return Err("lathe moves are lines in the XZ plane".into());
        };
        let m = &self.manifest;
        let turning = m
            .turning
            .as_ref()
            .ok_or("lathe preview needs the turning setup")?;
        let spec = m.stock.turn().ok_or("lathe preview needs turning stock")?;
        let p = m.initial_position_mm;
        let mut replay =
            crate::turning::ops::TurnReplay::new(turning.job(&m.run.id, spec, [p[0], p[2]]))?
                .with_stock(stock.clone())?;
        let mut partial = next.clone();
        let at = |k: usize| start[k] + fraction * (end[k] - start[k]);
        partial.curve = Curve::Line {
            start,
            end: [at(0), at(1), at(2)],
        };
        let delta = replay.motion(&partial)?;
        let mut shown = stock.clone();
        shown.apply_move(&delta.changes, &delta.rings)?;
        Ok(shown)
    }
    /// Reconstruct this cursor's next move prefix from move-start stock. The
    /// committed receiver remains unchanged while the UI displays partial stock.
    pub fn preview_partial(
        &self,
        next: &Motion,
        fraction: f64,
        cancel: &impl CancelSignal,
    ) -> Result<StockDelta, String> {
        self.check_motion(next)?;
        let tool = self
            .manifest
            .tools
            .iter()
            .find(|t| t.id == next.tool)
            .ok_or("partial tool missing")?
            .prepare()?;
        self.stock
            .height()?
            .preview_sweep(&tool, next, fraction, self.manifest.sweep, cancel)
    }
}

fn check_chunk_header(manifest: &StreamManifest, chunk: &MotionChunk) -> Result<(), String> {
    if chunk.from_complete != chunk.sequence
        || chunk.sequence.checked_add(1) != Some(chunk.to_complete)
        || chunk.delta.base_generation != chunk.from_complete
        || manifest
            .stock
            .height()
            .is_ok_and(|s| chunk.delta.spec != *s)
        || manifest.stock.turn().is_some()
        || chunk.delta.fraction != 1.
        || chunk.delta.motion_id != chunk.motion.id
        || chunk.delta.operation != chunk.motion.operation
        || (manifest.version == 2) != chunk.control_cursor.is_some()
    {
        return Err("chunk cursor, stock generation or motion attribution mismatch".into());
    }
    Ok(())
}
/// A committed lathe move: the motion and its turning stock change.
#[derive(Debug)]
pub struct TurnMotionChunk {
    pub sequence: u64,
    pub motion: Motion,
    pub delta: TurnDelta,
}
fn encode_turn_chunk(
    manifest: &StreamManifest,
    identity: &[u8],
    chunk: &TurnMotionChunk,
    limit: usize,
) -> Result<Vec<u8>, String> {
    let spec = manifest
        .stock
        .turn()
        .ok_or("turning chunk needs turning stock")?;
    if chunk.delta.changes.len() > spec.validate()? || chunk.delta.motion_id != chunk.motion.id {
        return Err("turning chunk exceeds the stock cell count or is misattributed".into());
    }
    let mut body = identity.to_vec();
    let to = chunk
        .sequence
        .checked_add(1)
        .ok_or("stream cursor exhausted")?;
    for value in [chunk.sequence, chunk.sequence, to] {
        put_u64(&mut body, value);
    }
    put_motion(&mut body, &chunk.motion)?;
    let bytes = 4 + chunk.delta.changes.len() * 68 + 4 + chunk.delta.rings.len() * 32;
    check_size(body.len() + bytes, limit)?;
    body.reserve(bytes);
    put_u32(&mut body, chunk.delta.changes.len() as u32);
    for c in &chunk.delta.changes {
        put_u32(
            &mut body,
            u32::try_from(c.index).map_err(|_| "turning cell index exceeds u32")?,
        );
        put_turn_cell(&mut body, c.before);
        put_turn_cell(&mut body, c.after);
    }
    put_u32(&mut body, chunk.delta.rings.len() as u32);
    for r in &chunk.delta.rings {
        for v in r {
            put_f64(&mut body, *v);
        }
    }
    frame(TURN_CHUNK, body, limit)
}
fn decode_turn_chunk(
    manifest: &StreamManifest,
    identity: &[u8],
    bytes: &[u8],
    limit: usize,
) -> Result<TurnMotionChunk, String> {
    let spec = manifest
        .stock
        .turn()
        .ok_or("turning chunk needs turning stock")?;
    let mut body = payload(bytes, TURN_CHUNK, limit)?;
    if body.take(identity.len())? != identity {
        return Err("stale/foreign stream run or input revision".into());
    }
    let sequence = body.u64()?;
    if body.u64()? != sequence || sequence.checked_add(1) != Some(body.u64()?) {
        return Err("turning chunk cursor mismatch".into());
    }
    let motion = body.motion()?;
    let count = body.u32()? as usize;
    if count > spec.validate()? || body.left() < count * 68 + 4 {
        return Err("invalid turning chunk cell count/length".into());
    }
    let mut changes = Vec::with_capacity(count);
    for _ in 0..count {
        changes.push(TurnChange {
            index: body.u32()? as usize,
            before: body.turn_cell()?,
            after: body.turn_cell()?,
        });
    }
    let rings = body.u32()? as usize;
    if body.left() != rings * 32 {
        return Err("invalid turning chunk ring count/length".into());
    }
    let rings = (0..rings)
        .map(|_| Ok([body.f64()?, body.f64()?, body.f64()?, body.f64()?]))
        .collect::<Result<_, String>>()?;
    Ok(TurnMotionChunk {
        sequence,
        delta: TurnDelta {
            motion_id: motion.id,
            changes,
            rings,
        },
        motion,
    })
}
/// Decode a committed lathe move frame (motion and stock change).
pub fn decode_turn_chunk_frame(
    manifest: &StreamManifest,
    bytes: &[u8],
    limit: usize,
) -> Result<TurnMotionChunk, String> {
    decode_turn_chunk(manifest, &identity_bytes(manifest)?, bytes, limit)
}
pub fn encode_chunk(
    manifest: &StreamManifest,
    chunk: &MotionChunk,
    limit: usize,
) -> Result<Vec<u8>, String> {
    manifest.validate()?;
    encode_chunk_prepared(manifest, &identity_bytes(manifest)?, chunk, limit)
}
fn encode_chunk_prepared(
    manifest: &StreamManifest,
    identity: &[u8],
    chunk: &MotionChunk,
    limit: usize,
) -> Result<Vec<u8>, String> {
    check_chunk_header(manifest, chunk)?;
    let (nx, ny, _) = manifest.stock.height()?.grid()?;
    if chunk.delta.changes.len() > nx * ny {
        return Err("chunk exceeds stock cell count".into());
    }
    let mut body = identity.to_vec();
    for value in [chunk.sequence, chunk.from_complete, chunk.to_complete] {
        put_u64(&mut body, value);
    }
    put_control_cursor(&mut body, chunk.control_cursor);
    put_motion(&mut body, &chunk.motion)?;
    put_delta_cells(&mut body, &chunk.delta, limit)?;
    frame(CHUNK, body, limit)
}
fn put_delta_cells(body: &mut Vec<u8>, delta: &StockDelta, limit: usize) -> Result<(), String> {
    put_u64(body, delta.base_generation);
    put_u32(body, delta.changes.len() as u32);
    let cells = delta
        .changes
        .len()
        .checked_mul(36)
        .ok_or("chunk size overflow")?;
    check_size(body.len() + cells, limit)?;
    body.reserve(cells);
    for c in &delta.changes {
        put_u32(body, c.index);
        put_height(body, c.before);
        put_height(body, c.after);
    }
    Ok(())
}
pub fn decode_chunk(
    manifest: &StreamManifest,
    bytes: &[u8],
    limit: usize,
) -> Result<MotionChunk, String> {
    manifest.validate()?;
    decode_chunk_prepared(manifest, &identity_bytes(manifest)?, bytes, limit)
}
/// Decode only the motion of a committed chunk frame, using the same frame,
/// identity, cursor and control-prefix validation as `decode_chunk`. Cell
/// changes are not parsed (callers that need them use `decode_chunk`).
pub fn decode_chunk_motion(
    manifest: &StreamManifest,
    identity: &[u8],
    bytes: &[u8],
    limit: usize,
) -> Result<Motion, String> {
    let mut body = payload(bytes, CHUNK, limit)?;
    if body.take(identity.len())? != identity {
        return Err("stale/foreign stream run or input revision".into());
    }
    let (_, _, _) = (body.u64()?, body.u64()?, body.u64()?);
    body.control_cursor(manifest)?;
    body.motion()
}
fn decode_chunk_prepared(
    manifest: &StreamManifest,
    identity: &[u8],
    bytes: &[u8],
    limit: usize,
) -> Result<MotionChunk, String> {
    let mut body = payload(bytes, CHUNK, limit)?;
    if body.take(identity.len())? != identity {
        return Err("stale/foreign stream run or input revision".into());
    }
    let sequence = body.u64()?;
    let from_complete = body.u64()?;
    let to_complete = body.u64()?;
    let control_cursor = body.control_cursor(manifest)?;
    let motion = body.motion()?;
    let base_generation = body.u64()?;
    let count = body.u32()? as usize;
    let (nx, ny, _) = manifest.stock.height()?.grid()?;
    if count > nx * ny || body.left() != count * 36 {
        return Err("invalid chunk cell count/length".into());
    }
    let mut changes = Vec::with_capacity(count);
    for _ in 0..count {
        changes.push(CellChange {
            index: body.u32()?,
            before: body.height()?,
            after: body.height()?,
        });
    }
    let delta = StockDelta {
        spec: manifest.stock.height()?.clone(),
        base_generation,
        motion_id: motion.id,
        operation: motion.operation.clone(),
        fraction: 1.,
        stats: SweepStats {
            changed_cells: count,
            delta_bytes: count * std::mem::size_of::<CellChange>(),
            ..Default::default()
        },
        changes,
        receipt: None,
        native_identity: None,
        cleared_column: None,
        axial_column: None,
        departure: None,
        shift: false,
    };
    let chunk = MotionChunk {
        sequence,
        from_complete,
        to_complete,
        control_cursor,
        motion,
        delta,
    };
    check_chunk_header(manifest, &chunk)?;
    Ok(chunk)
}

pub fn encode_control(
    manifest: &StreamManifest,
    chunk: &ControlChunk,
    limit: usize,
) -> Result<Vec<u8>, String> {
    manifest.validate()?;
    if manifest.version != 2 {
        return Err("controls require manifest version 2".into());
    }
    encode_control_prepared(&identity_bytes(manifest)?, chunk, limit)
}
fn encode_control_prepared(
    identity: &[u8],
    chunk: &ControlChunk,
    limit: usize,
) -> Result<Vec<u8>, String> {
    if !point_valid(chunk.position_before) || matches!(chunk.event, Event::Motion { .. }) {
        return Err("control frame requires bounded prior pose and nonmotion event".into());
    }
    let mut body = identity.to_vec();
    put_u64(&mut body, chunk.completed);
    put_control_cursor(&mut body, Some(chunk.before));
    put_vec3(&mut body, chunk.position_before);
    put_control_json(&mut body, &chunk.event)?;
    frame(CONTROL, body, limit)
}
pub fn decode_control(
    manifest: &StreamManifest,
    bytes: &[u8],
    limit: usize,
) -> Result<ControlChunk, String> {
    manifest.validate()?;
    decode_control_prepared(manifest, &identity_bytes(manifest)?, bytes, limit)
}
fn decode_control_prepared(
    manifest: &StreamManifest,
    identity: &[u8],
    bytes: &[u8],
    limit: usize,
) -> Result<ControlChunk, String> {
    if manifest.version != 2 {
        return Err("controls require manifest version 2".into());
    }
    let mut body = payload(bytes, CONTROL, limit)?;
    if body.take(identity.len())? != identity {
        return Err("stale/foreign stream run or input revision".into());
    }
    let chunk = ControlChunk {
        completed: body.u64()?,
        before: body.control_cursor(manifest)?.unwrap(),
        position_before: body.vec3()?,
        event: body.control_json()?,
    };
    if body.left() != 0 || matches!(chunk.event, Event::Motion { .. }) {
        return Err("control frame has extra bytes or a motion event".into());
    }
    Ok(chunk)
}

/// Control records bracket streamed motions with complete M1 setup/shutdown
/// events. They share the exact run/manifest binding and committed cursor.
#[derive(Serialize)]
pub struct OperationControl<'a, R = OperationReport<'a>> {
    pub operation: &'a str,
    pub completed: u64,
    pub stage: &'a str,
    pub events: Vec<Event>,
    pub report: Option<R>,
}
pub fn encode_operation_control<R: Serialize>(
    manifest: &StreamManifest,
    control: &OperationControl<'_, R>,
    limit: usize,
) -> Result<Vec<u8>, String> {
    let mut body = identity_bytes(manifest)?;
    body.extend(serde_json::to_vec(control).map_err(|e| e.to_string())?);
    frame(OPERATION, body, limit)
}
pub fn decode_operation_control(
    manifest: &StreamManifest,
    bytes: &[u8],
    limit: usize,
) -> Result<serde_json::Value, String> {
    let mut body = payload(bytes, OPERATION, limit)?;
    read_identity(&mut body, manifest)?;
    serde_json::from_slice(body.take(body.left())?).map_err(|e| e.to_string())
}

pub(crate) fn encode_verification(
    identity: &[u8],
    record: &crate::stream_verification::Record,
    limit: usize,
) -> Result<Vec<u8>, String> {
    let mut body = identity.to_vec();
    serde_json::to_writer(&mut body, record).map_err(|e| e.to_string())?;
    frame(VERIFICATION, body, limit)
}
pub(crate) fn decode_verification(
    identity: &[u8],
    bytes: &[u8],
    limit: usize,
) -> Result<crate::stream_verification::Record, String> {
    let mut body = payload(bytes, VERIFICATION, limit)?;
    if body.take(identity.len())? != identity {
        return Err("stale/foreign verification run, revision or manifest".into());
    }
    serde_json::from_slice(body.take(body.left())?).map_err(|e| e.to_string())
}

/// Setup metadata appears once; dense stock and motion frames remain binary.
pub fn encode_manifest(manifest: &StreamManifest, limit: usize) -> Result<Vec<u8>, String> {
    manifest.validate()?;
    frame(MANIFEST, manifest.canonical_bytes()?, limit)
}
pub fn decode_manifest(bytes: &[u8], limit: usize) -> Result<StreamManifest, String> {
    let body = payload(bytes, MANIFEST, limit)?;
    let manifest: StreamManifest = serde_json::from_slice(body.bytes).map_err(|e| e.to_string())?;
    manifest.validate()?;
    if manifest.canonical_bytes() != Ok(body.bytes.to_vec()) {
        return Err("manifest frame must contain canonical setup bytes".into());
    }
    Ok(manifest)
}

/// Read exactly one bounded frame from a pipe/file without buffering later
/// frames. None is clean EOF; truncated input is an error, never completion.
pub fn read_frame(reader: &mut impl Read, limit: usize) -> Result<Option<Vec<u8>>, String> {
    let mut header = [0u8; HEADER];
    match reader.read_exact(&mut header[..1]) {
        Ok(()) => {}
        Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => return Ok(None),
        Err(e) => return Err(e.to_string()),
    }
    reader
        .read_exact(&mut header[1..])
        .map_err(|e| format!("truncated frame header: {e}"))?;
    let (_, len) = check_header(&header, limit)?;
    // Grow with the bytes that actually arrive: a corrupt or hostile header
    // claiming a huge length must not allocate that length up front.
    let mut output = header.to_vec();
    let read = reader
        .take(len as u64)
        .read_to_end(&mut output)
        .map_err(|e| format!("truncated frame payload: {e}"))?;
    if read != len {
        return Err(format!("truncated frame payload: {read} of {len} bytes"));
    }
    // Validate checksum before the caller treats a read as a committed frame.
    validate_crc(&output)?;
    Ok(Some(output))
}

fn point_valid(p: Vec3) -> bool {
    p.iter().all(|v| v.is_finite() && v.abs() <= 1e6)
}
fn check_size(body: usize, limit: usize) -> Result<(), String> {
    if !(HEADER..=MAX_FRAME_BYTES).contains(&limit) || body > limit - HEADER {
        return Err("binary frame exceeds the caller's limit or the u32 frame length field".into());
    }
    Ok(())
}
fn frame(kind: u16, body: Vec<u8>, limit: usize) -> Result<Vec<u8>, String> {
    check_size(body.len(), limit)?;
    let mut out = Vec::with_capacity(HEADER + body.len());
    out.extend_from_slice(MAGIC);
    out.extend_from_slice(&VERSION.to_le_bytes());
    out.extend_from_slice(&kind.to_le_bytes());
    put_u32(&mut out, body.len() as u32);
    put_u32(&mut out, 0);
    out.extend_from_slice(&body);
    let crc = crc32_parts(&out[..16], &out[HEADER..]);
    out[16..20].copy_from_slice(&crc.to_le_bytes());
    Ok(out)
}
fn check_header(header: &[u8], limit: usize) -> Result<(u16, usize), String> {
    if header.len() < HEADER
        || &header[..8] != MAGIC
        || u16::from_le_bytes(header[8..10].try_into().unwrap()) != VERSION
    {
        return Err("invalid binary stream magic/version or short header".into());
    }
    let kind = u16::from_le_bytes(header[10..12].try_into().unwrap());
    let len = u32::from_le_bytes(header[12..16].try_into().unwrap()) as usize;
    if ![
        CHUNK,
        CHECKPOINT,
        MANIFEST,
        END,
        PREVIEW,
        RESTORED,
        OPERATION,
        VERIFICATION,
        CONTROL,
        TURN_CHUNK,
    ]
    .contains(&kind)
    {
        return Err("unsupported binary frame kind".into());
    }
    check_size(len, limit)?;
    Ok((kind, len))
}
fn validate_crc(frame: &[u8]) -> Result<(), String> {
    let expected = u32::from_le_bytes(frame[16..20].try_into().unwrap());
    if crc32_parts(&frame[..16], &frame[HEADER..]) != expected {
        return Err("binary frame checksum mismatch".into());
    }
    Ok(())
}
fn payload(bytes: &[u8], kind: u16, limit: usize) -> Result<Decoder<'_>, String> {
    let (found, len) = check_header(bytes, limit)?;
    if found != kind || bytes.len() != HEADER + len {
        return Err("binary frame kind or exact payload length mismatch".into());
    }
    validate_crc(bytes)?;
    Ok(Decoder {
        bytes: &bytes[HEADER..],
        offset: 0,
    })
}
/// Standard CRC-32 (IEEE, reflected 0xEDB88320; identical to zlib.crc32).
/// `crc32fast` uses PCLMULQDQ/SIMD where available.
fn crc32_parts(a: &[u8], b: &[u8]) -> u32 {
    let mut hasher = crc32fast::Hasher::new();
    hasher.update(a);
    hasher.update(b);
    hasher.finalize()
}
fn read_prefix(
    reader: &mut impl Read,
    prefix: &mut Vec<u8>,
    count: usize,
    total: usize,
) -> Result<(), String> {
    if count > total.saturating_sub(prefix.len()) {
        return Err("checkpoint payload too short for metadata".into());
    }
    let before = prefix.len();
    prefix.resize(before + count, 0);
    reader
        .read_exact(&mut prefix[before..])
        .map_err(|e| format!("truncated checkpoint metadata: {e}"))
}
fn put_u8(out: &mut Vec<u8>, v: u8) {
    out.push(v)
}
fn put_u32(out: &mut Vec<u8>, v: u32) {
    out.extend_from_slice(&v.to_le_bytes());
}
fn put_u64(out: &mut Vec<u8>, v: u64) {
    out.extend_from_slice(&v.to_le_bytes());
}
fn put_f64(out: &mut Vec<u8>, v: f64) {
    out.extend_from_slice(&v.to_le_bytes());
}
fn put_vec3(out: &mut Vec<u8>, v: Vec3) {
    for n in v {
        put_f64(out, n);
    }
}
fn put_control_cursor(out: &mut Vec<u8>, cursor: Option<ControlCursor>) {
    if let Some(cursor) = cursor {
        put_u64(out, cursor.count);
        out.extend_from_slice(&cursor.chain_sha256);
    }
}
fn put_control_json(out: &mut Vec<u8>, value: &impl Serialize) -> Result<(), String> {
    let raw = serde_json::to_vec(value).map_err(|e| e.to_string())?;
    if raw.len() > CONTROL_JSON_LIMIT {
        return Err("control JSON exceeds bounded size".into());
    }
    put_u32(out, raw.len() as u32);
    out.extend_from_slice(&raw);
    Ok(())
}
fn put_turn_cell(out: &mut Vec<u8>, c: TurnCell) {
    for v in [
        c.outer.lower_mm,
        c.outer.upper_mm,
        c.inner.lower_mm,
        c.inner.upper_mm,
    ] {
        put_f64(out, v);
    }
}
fn put_height(out: &mut Vec<u8>, v: HeightBounds) {
    put_f64(out, v.lower_mm);
    put_f64(out, v.upper_mm);
}
fn put_string(out: &mut Vec<u8>, v: &str) {
    out.extend_from_slice(&(v.len() as u16).to_le_bytes());
    out.extend_from_slice(v.as_bytes());
}
pub fn identity_bytes(manifest: &StreamManifest) -> Result<Vec<u8>, String> {
    let run = &manifest.run;
    let mut bytes = Vec::new();
    put_string(&mut bytes, &run.id);
    put_string(&mut bytes, &run.revision);
    bytes.extend_from_slice(&manifest.digest()?);
    Ok(bytes)
}
fn read_identity(body: &mut Decoder<'_>, manifest: &StreamManifest) -> Result<(), String> {
    let expected = &manifest.run;
    let id = body.string(128)?;
    let revision = body.string(512)?;
    let digest = body.take(32)?;
    if id != expected.id || revision != expected.revision || digest != manifest.digest()? {
        return Err("stale/foreign stream run or input revision".into());
    }
    Ok(())
}
fn put_motion(out: &mut Vec<u8>, m: &Motion) -> Result<(), String> {
    if !valid_id(&m.operation) || !valid_id(&m.tool) {
        return Err("invalid wire motion identifiers".into());
    }
    put_u64(out, m.id);
    put_string(out, &m.operation);
    put_string(out, &m.tool);
    put_u8(
        out,
        match m.kind {
            MoveKind::Rapid => 0,
            MoveKind::Cut => 1,
            MoveKind::Plunge => 2,
            MoveKind::Lead => 3,
            MoveKind::Link => 4,
            MoveKind::Retract => 5,
            MoveKind::Tap => 6,
            MoveKind::Shift => 7,
            MoveKind::Thread => 8,
        },
    );
    put_u8(out, u8::from(m.feed_mm_min.is_some()));
    put_f64(out, m.feed_mm_min.unwrap_or(0.));
    put_u8(
        out,
        match m.curve {
            Curve::Line { .. } => 0,
            Curve::Arc { .. } => 1,
            Curve::Helix { .. } => 2,
        },
    );
    put_vec3(out, m.curve.start());
    put_vec3(out, m.curve.end());
    if let Curve::Arc {
        center,
        plane,
        sweep_rad,
        ..
    }
    | Curve::Helix {
        center,
        plane,
        sweep_rad,
        ..
    } = m.curve
    {
        put_vec3(out, center);
        put_u8(
            out,
            match plane {
                Plane::Xy => 0,
                Plane::Xz => 1,
                Plane::Yz => 2,
            },
        );
        put_f64(out, sweep_rad);
    }
    Ok(())
}
struct Decoder<'a> {
    bytes: &'a [u8],
    offset: usize,
}
impl<'a> Decoder<'a> {
    fn left(&self) -> usize {
        self.bytes.len() - self.offset
    }
    fn take(&mut self, n: usize) -> Result<&'a [u8], String> {
        if n > self.left() {
            return Err("truncated binary field".into());
        }
        let part = &self.bytes[self.offset..self.offset + n];
        self.offset += n;
        Ok(part)
    }
    fn u8(&mut self) -> Result<u8, String> {
        Ok(self.take(1)?[0])
    }
    fn u32(&mut self) -> Result<u32, String> {
        Ok(u32::from_le_bytes(self.take(4)?.try_into().unwrap()))
    }
    fn u64(&mut self) -> Result<u64, String> {
        Ok(u64::from_le_bytes(self.take(8)?.try_into().unwrap()))
    }
    fn f64(&mut self) -> Result<f64, String> {
        Ok(f64::from_le_bytes(self.take(8)?.try_into().unwrap()))
    }
    fn vec3(&mut self) -> Result<Vec3, String> {
        Ok([self.f64()?, self.f64()?, self.f64()?])
    }
    fn control_cursor(
        &mut self,
        manifest: &StreamManifest,
    ) -> Result<Option<ControlCursor>, String> {
        if manifest.version == 2 {
            Ok(Some(ControlCursor {
                count: self.u64()?,
                chain_sha256: self.take(32)?.try_into().unwrap(),
            }))
        } else {
            Ok(None)
        }
    }
    fn control_json<T: serde::de::DeserializeOwned>(&mut self) -> Result<T, String> {
        let size = self.u32()? as usize;
        if size > CONTROL_JSON_LIMIT {
            return Err("control JSON exceeds bounded size".into());
        }
        serde_json::from_slice(self.take(size)?).map_err(|e| e.to_string())
    }
    fn turn_cell(&mut self) -> Result<TurnCell, String> {
        let mut bounds = || -> Result<RadiusBounds, String> {
            Ok(RadiusBounds {
                lower_mm: self.f64()?,
                upper_mm: self.f64()?,
            })
        };
        let outer = bounds()?;
        let inner = bounds()?;
        Ok(TurnCell { outer, inner })
    }
    fn height(&mut self) -> Result<HeightBounds, String> {
        Ok(HeightBounds {
            lower_mm: self.f64()?,
            upper_mm: self.f64()?,
        })
    }
    fn string(&mut self, limit: usize) -> Result<String, String> {
        let len = u16::from_le_bytes(self.take(2)?.try_into().unwrap()) as usize;
        if len > limit {
            return Err("binary string length exceeds limit".into());
        }
        String::from_utf8(self.take(len)?.to_vec()).map_err(|e| e.to_string())
    }
    fn motion(&mut self) -> Result<Motion, String> {
        let id = self.u64()?;
        let operation = self.string(128)?;
        let tool = self.string(128)?;
        let kind = match self.u8()? {
            0 => MoveKind::Rapid,
            1 => MoveKind::Cut,
            2 => MoveKind::Plunge,
            3 => MoveKind::Lead,
            4 => MoveKind::Link,
            5 => MoveKind::Retract,
            6 => MoveKind::Tap,
            7 => MoveKind::Shift,
            8 => MoveKind::Thread,
            _ => return Err("invalid binary motion kind".into()),
        };
        let feed_flag = self.u8()?;
        let feed = self.f64()?;
        if feed_flag > 1 || feed_flag == 0 && feed != 0. {
            return Err("invalid binary feed flag/value".into());
        }
        let feed_mm_min = (feed_flag == 1).then_some(feed);
        let curve_kind = self.u8()?;
        let start = self.vec3()?;
        let end = self.vec3()?;
        let curve = match curve_kind {
            0 => Curve::Line { start, end },
            1 | 2 => {
                let center = self.vec3()?;
                let plane = match self.u8()? {
                    0 => Plane::Xy,
                    1 => Plane::Xz,
                    2 => Plane::Yz,
                    _ => return Err("invalid binary curve plane".into()),
                };
                let sweep_rad = self.f64()?;
                if curve_kind == 1 {
                    Curve::Arc {
                        start,
                        end,
                        center,
                        plane,
                        sweep_rad,
                    }
                } else {
                    Curve::Helix {
                        start,
                        end,
                        center,
                        plane,
                        sweep_rad,
                    }
                }
            }
            _ => return Err("invalid binary curve kind".into()),
        };
        Ok(Motion {
            id,
            operation,
            tool,
            kind,
            curve,
            feed_mm_min,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn public_cached_motion_does_not_acquire_verified_cache_column_authority() {
        let mut manifest: StreamManifest =
            serde_json::from_str(include_str!("../fixtures/operations/adaptive-pocket.json"))
                .unwrap();
        manifest.operations.clear();
        manifest.initial_position_mm = [8., 13., 5.];
        let motion = Motion {
            id: 1,
            operation: "wire".into(),
            tool: manifest.tools[0].id.clone(),
            kind: MoveKind::Cut,
            feed_mm_min: Some(600.),
            curve: Curve::Line {
                start: [8., 13., 5.],
                end: [18., 13., 5.],
            },
        };
        let mut wire = StreamProducer::new(manifest.clone()).unwrap();
        let original = wire
            .stock()
            .unwrap()
            .preview_sweep(
                &manifest.tools[0].prepare().unwrap(),
                &motion,
                1.,
                manifest.sweep,
                &|| false,
            )
            .unwrap();
        let delta: StockDelta =
            serde_json::from_str(&serde_json::to_string(&original).unwrap()).unwrap();
        wire.push_cached_motion(motion.clone(), delta, DEFAULT_FRAME_LIMIT, &|| false)
            .unwrap();
        assert!(
            wire.stock()
                .unwrap()
                .committed_clearance(1e-5, &|| false)
                .unwrap()
                .is_empty()
        );
        let mut native = StreamProducer::new(manifest).unwrap();
        native
            .push_motion(motion, DEFAULT_FRAME_LIMIT, &|| false)
            .unwrap();
        assert_eq!(
            native
                .stock()
                .unwrap()
                .committed_clearance(1e-5, &|| false)
                .unwrap()
                .len(),
            1
        );
        assert_eq!(
            wire.stock().unwrap().snapshot(),
            native.stock().unwrap().snapshot()
        );
    }

    #[test]
    fn crc_matches_standard_check_vector() {
        assert_eq!(super::crc32_parts(b"1234", b"56789"), 0xcbf43926);
        // Bitwise reference on varied lengths/splits (frames are split at 16).
        let reference = |data: &[u8]| {
            let mut crc = !0u32;
            for &byte in data {
                crc ^= byte as u32;
                for _ in 0..8 {
                    crc = if crc & 1 != 0 {
                        0xedb8_8320 ^ (crc >> 1)
                    } else {
                        crc >> 1
                    };
                }
            }
            !crc
        };
        let data: Vec<u8> = (0..70_000u32)
            .map(|i| (i.wrapping_mul(2_654_435_761) >> 13) as u8)
            .collect();
        for n in [0, 1, 15, 16, 17, 63, 64, 65, 4096, 70_000] {
            for split in [0, n / 3, n.min(16)] {
                assert_eq!(
                    super::crc32_parts(&data[..split], &data[split..n]),
                    reference(&data[..n])
                );
            }
        }
    }
}
