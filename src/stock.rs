//! Bounded top-down stock. Each cell encloses remaining height over its entire
//! footprint; continuous sweeps produce transactional sparse before/after deltas.
//! Engineering bounds, not interval-certified collision verification.
use crate::{
    Point,
    engagement::ClearedSweep,
    motion::{Curve, Motion, MoveKind, Plane, Vec3},
    tool::{CutterShape, PreparedAssembly, valid_id},
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::sync::{
    Arc, OnceLock,
    atomic::{AtomicBool, AtomicU64, Ordering},
};

static NEXT_STOCK_IDENTITY: AtomicU64 = AtomicU64::new(1);
fn new_identity() -> u64 {
    NEXT_STOCK_IDENTITY.fetch_add(1, Ordering::Relaxed)
}

/// Runtime authority only: deliberately absent from snapshots and wire deltas.
#[derive(Clone, Debug)]
pub(crate) struct SweepReceipt {
    pub base_identity: u64,
    pub motion: Motion,
    pub tool: crate::tool::ToolAssembly,
    pub fraction: f64,
    pub tolerance_mm: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct ReplayClearanceStats {
    pub columns: u64,
    pub reach_cell_visits: u64,
}

/// Stock cell indices are u32 in every delta/checkpoint frame: the grid-size
/// invariant. Memory is governed by the caller's explicit StockSpec.max_cells.
const MAX_CELLS: usize = u32::MAX as usize;

/// Polled at bounded work intervals. Atomics support cross-thread cancellation;
/// callbacks also allow schedulers/tests to stop deterministically.
pub trait CancelSignal {
    fn is_cancelled(&self) -> bool;
}
impl CancelSignal for AtomicBool {
    fn is_cancelled(&self) -> bool {
        self.load(Ordering::Relaxed)
    }
}
impl<F: Fn() -> bool> CancelSignal for F {
    fn is_cancelled(&self) -> bool {
        self()
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StockSpec {
    pub version: u32,
    pub units: String,
    pub min_mm: Vec3,
    pub max_mm: Vec3,
    pub cell_size_mm: f64,
    pub max_cells: usize,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HeightBounds {
    pub lower_mm: f64,
    pub upper_mm: f64,
}

/// Caller-declared whole-cell bounds of a top-down blank in setup coordinates.
/// Source identity binds the physical premise; this is not a restored snapshot
/// and does not claim that an imported mesh or a weld was measured by camlib.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InitialStock {
    pub source_id: String,
    pub revision: String,
    pub source_sha256: String,
    /// Recorded placement only: bounds below are already in setup coordinates.
    pub source_to_setup: crate::motion::WorkFrame,
    /// Source/placement error already enclosed by every whole-cell bound.
    pub uncertainty_mm: f64,
    pub representation: String,
    pub bounds: Vec<HeightBounds>,
}
impl InitialStock {
    const INVALID: &str = "initial stock needs source identity, whole_cell_top_down representation and one valid lower/upper height pair per cell";

    pub(crate) fn validate_metadata(&self) -> Result<(), String> {
        self.source_to_setup.validate()?;
        if !crate::tool::valid_id(&self.source_id)
            || self.revision.is_empty()
            || self.revision.len() > 512
            || self.source_sha256.len() != 64
            || !self.source_sha256.bytes().all(|b| b.is_ascii_hexdigit())
            || !self.uncertainty_mm.is_finite()
            || !(0. ..=1e6).contains(&self.uncertainty_mm)
            || self.representation != "whole_cell_top_down"
        {
            return Err(Self::INVALID.into());
        }
        Ok(())
    }

    pub fn validate(&self, spec: &StockSpec) -> Result<(), String> {
        let (nx, ny, _) = spec.grid()?;
        self.validate_metadata()?;
        if self.bounds.len() != nx * ny
            || self
                .bounds
                .iter()
                .any(|&height| !height_valid(height, spec))
        {
            return Err(Self::INVALID.into());
        }
        Ok(())
    }

    pub fn volume(&self, spec: &StockSpec) -> Result<[f64; 2], String> {
        self.validate(spec)?;
        let (_, _, step) = spec.grid()?;
        let area = step[0] * step[1];
        Ok([
            self.bounds
                .iter()
                .map(|h| (h.lower_mm - spec.min_mm[2]) * area)
                .sum(),
            self.bounds
                .iter()
                .map(|h| (h.upper_mm - spec.min_mm[2]) * area)
                .sum(),
        ])
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StockSnapshot {
    pub spec: StockSpec,
    pub generation: u64,
    pub cells: Vec<HeightBounds>,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CellChange {
    pub index: u32,
    pub before: HeightBounds,
    pub after: HeightBounds,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StockDelta {
    pub(crate) spec: StockSpec,
    pub(crate) base_generation: u64,
    pub(crate) motion_id: u64,
    pub(crate) operation: String,
    pub(crate) fraction: f64,
    pub(crate) changes: Vec<CellChange>,
    pub(crate) stats: SweepStats,
    #[serde(skip)]
    pub(crate) receipt: Option<Arc<SweepReceipt>>,
    // A wire delta can reproduce heights, never native geometric authority.
    #[serde(skip)]
    pub(crate) native_identity: Option<u64>,
    #[serde(skip)]
    pub(crate) cleared_column: Option<CommittedColumn>,
    #[serde(skip)]
    pub(crate) axial_column: Option<AxialColumn>,
    /// The tool and end pose of this (fully swept) motion: native authority.
    #[serde(skip)]
    pub(crate) departure: Option<(String, Vec3)>,
    /// This motion is an oriented-spindle shift (keeps the bore record).
    #[serde(skip)]
    pub(crate) shift: bool,
}
/// A committed straight-down feed plunge of a tool without a neck (or of
/// any tool whose plunge starts with its tip above the stock, so its cutting
/// part swept the whole column down from outside the stock): every
/// pose of that tool's cutting part on the same axis with its tip at or above
/// `tip_z` is empty space. The swept cutter removed the material it occupied,
/// stock only loses material, and while no undercut has been cut the stock is
/// a height field, so the space above an empty pose is empty too. Native
/// authority only (never deserialized).
#[derive(Clone, Debug)]
pub(crate) struct AxialColumn {
    tool: String,
    xy: [f64; 2],
    tip_z: f64,
}
impl AxialColumn {
    fn from_motion(tool: &PreparedAssembly, motion: &Motion, stock_top: f64) -> Option<Self> {
        let Curve::Line { start, end } = motion.curve else {
            return None;
        };
        (motion.kind.cuts()
            && start[0] == end[0]
            && start[1] == end[1]
            && end[2] < start[2]
            && (tool.cutter().definition().neck.is_none() || start[2] >= stock_top))
            .then(|| Self {
                tool: motion.tool.clone(),
                xy: [end[0], end[1]],
                tip_z: end[2],
            })
    }
}
/// Bound on retained axial certificates; beyond it new ones are dropped
/// (conservative: fewer proofs, never a false one).
const MAX_AXIAL_COLUMNS: usize = 1 << 20;
impl StockDelta {
    fn record_change(
        &mut self,
        index: usize,
        before: HeightBounds,
        after: HeightBounds,
        limit: usize,
    ) -> Result<(), String> {
        if before != after {
            if self.changes.len() >= limit {
                return Err("sweep changed-cell budget exceeded".into());
            }
            self.changes.push(CellChange {
                index: index as u32,
                before,
                after,
            });
        }
        Ok(())
    }

    pub fn changes(&self) -> &[CellChange] {
        &self.changes
    }
    pub fn base_generation(&self) -> u64 {
        self.base_generation
    }
    /// Native preview work counters. Decoded wire deltas retain only changed-cell
    /// count/bytes; execution-work counters are not part of stock wire semantics.
    pub fn stats(&self) -> &SweepStats {
        &self.stats
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SweepOptions {
    pub tolerance_mm: f64,
    pub chord_error_mm: f64,
    pub max_tip_step_mm: f64,
    pub max_segments: usize,
    /// Maximum affected work rectangle. Segmented kernels also allocate a
    /// height scratch buffer of this size; analytic kernels emit deltas directly.
    pub max_scratch_cells: usize,
    pub max_changed_cells: usize,
    pub max_cell_visits: u64,
}
impl Default for SweepOptions {
    fn default() -> Self {
        Self {
            tolerance_mm: 1e-5,
            chord_error_mm: 0.002,
            max_tip_step_mm: 0.1,
            max_segments: 100_000,
            // A sweep cannot touch more cells than the grid holds; the
            // caller's StockSpec.max_cells is the binding size limit.
            max_scratch_cells: MAX_CELLS,
            max_changed_cells: MAX_CELLS,
            max_cell_visits: 100_000_000,
        }
    }
}
impl SweepOptions {
    pub(crate) fn validate(&self) -> Result<(), String> {
        if !self.tolerance_mm.is_finite()
            || !(1e-7..=0.1).contains(&self.tolerance_mm)
            || !self.chord_error_mm.is_finite()
            || !(1e-7..=0.1).contains(&self.chord_error_mm)
            || !self.max_tip_step_mm.is_finite()
            || !(1e-5..=10.).contains(&self.max_tip_step_mm)
            || self.max_segments == 0
            || self.max_segments > 1_000_000
            || self.max_scratch_cells == 0
            || self.max_scratch_cells > MAX_CELLS
            || self.max_changed_cells == 0
            || self.max_changed_cells > MAX_CELLS
            || self.max_cell_visits == 0
            || self.max_cell_visits > 1_000_000_000
        {
            return Err("invalid sweep tolerance or bounded work/memory limits".into());
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SweepStats {
    #[serde(default, skip_serializing_if = "no_reach_visits")]
    pub reach_cell_visits: u64,
    /// Configured prefix resolution/admission count, also for analytic kernels.
    pub segments: usize,
    #[serde(default)]
    pub materialized_segments: usize,
    #[serde(default)]
    pub analytic_path_queries: u64,
    pub visited_cells: u64,
    /// Actually materialized height scratch; zero for direct analytic sweeps.
    pub scratch_cells: usize,
    pub changed_cells: usize,
    pub scratch_bytes: usize,
    pub delta_bytes: usize,
    pub max_path_error_mm: f64,
    pub numerical_guard_mm: f64,
    /// Cells where the cutting part passed under material that stays above it
    /// (an undercut such as a thread groove). Their top height is unchanged.
    #[serde(default, skip_serializing_if = "no_reach_visits")]
    pub undercut_cells: u64,
    /// Cells this move only partly covers whose upper bound still dropped:
    /// every sub-square proved inside this move or a committed cleared pass
    /// at or below its height (`Stock::union_covered`).
    #[serde(default, skip_serializing_if = "no_reach_visits")]
    pub union_cells: u64,
}
fn no_reach_visits(n: &u64) -> bool {
    *n == 0
}
/// A necked tool (thread mill, undercut tool) may run its cutting part under
/// committed material, between its cutting length and its shoulder. Only the
/// neck can then rub that material: a cell beyond the neck keeps its top
/// height (material stays above the cut); a cell within it is refused, as is
/// any material above the shoulder, as for every covered cell without a neck.
struct Undercut {
    cutting: f64,
    shoulder: f64,
    neck: f64,
    guard: f64,
}
impl Undercut {
    fn new(tool: &crate::tool::Cutter, guard: f64) -> Option<Self> {
        tool.neck.as_ref().map(|n| Self {
            cutting: tool.cutting_length_mm,
            shoulder: tool.cutting_length_mm + n.length_mm,
            neck: n.diameter_mm / 2.,
            guard,
        })
    }
    /// False: the cutting part spans the material top here (ordinary
    /// removal). True: undercut, top height unchanged.
    fn classify(
        &self,
        committed: HeightBounds,
        tip: f64,
        distance: f64,
        cell_radius: f64,
        path_error: f64,
    ) -> Result<bool, String> {
        if committed.upper_mm <= tip + self.cutting - self.guard {
            return Ok(false);
        }
        if committed.upper_mm > tip + self.shoulder - self.guard {
            return Err(REACH.into());
        }
        let near = (distance - cell_radius - path_error).max(0.);
        if near <= self.neck {
            let center_clearance = distance - path_error - self.neck;
            let margin = center_clearance - cell_radius;
            let remedy = if center_clearance > 0. {
                // The distance bound is 1-Lipschitz; a square cell has
                // circumradius cell_size/sqrt(2). This is a LOCAL threshold
                // at this cell centre, not a promise about a refined grid.
                format!(
                    "at this cell center, square cell_size_mm must be < {:.9} mm; rerun with finer stock and recheck all cells (new centers and stock bounds may differ)",
                    center_clearance * std::f64::consts::SQRT_2
                )
            } else {
                "no positive cell-size threshold at this cell center; recheck stock bounds, use a narrower neck, or clear more stock".into()
            };
            return Err(format!(
                "the tool neck may touch material beside an undercut (thread groove): neck radius {:.9} mm, stock cell half-diagonal {:.9} mm, path error {:.9} mm, clearance lower bound {:.9} mm; {remedy}",
                self.neck, cell_radius, path_error, margin
            ));
        }
        Ok(true)
    }
}
const UNDERCUT_STOCK: &str = "lollipop and dovetail cutters cut above their widest section, under material a height field cannot hold; run them on tri-dexel stock (DexelStockSpec)";
const REACH: &str = "cutting length cannot span committed stock upper bound at lowest tip; unsupported non-heightfield removal/reach";
/// Whether every point of the cell centred at `center` lies farther than
/// `touch` from the move's axis (distance is 1-Lipschitz; `error` covers the
/// path's numerical error).
fn beyond(
    axis: &crate::verification::AxisSpan,
    center: Option<[f64; 2]>,
    cell_radius: f64,
    error: f64,
    touch: f64,
) -> bool {
    center.is_some_and(|p| axis.distance(p) - cell_radius - error > touch)
}

#[derive(Clone, Copy, Debug, Serialize)]
pub struct StockVolume {
    pub remaining_lower_mm3: f64,
    pub remaining_upper_mm3: f64,
    pub removed_lower_mm3: f64,
    pub removed_upper_mm3: f64,
}

pub(crate) const MAX_COMMITTED_COLUMNS: usize = 262_144;
/// What the union rule reads besides the cells, as a producer checkpoint
/// keeps it (`Stock::clearance_state`).
#[derive(Clone, Debug)]
pub(crate) struct ClearanceState {
    pub unproven: bool,
    pub operation: Option<String>,
    pub union_from: usize,
    /// Each column's tolerance and sweep, oldest first.
    pub columns: Vec<(f64, ClearedSweep)>,
}
/// Quadtree depth for a partly covered cell (`Stock::union_covered`):
/// 0.5 mm cells split down to 1/32, about 0.016 mm sub-squares.
const UNION_DEPTH: u32 = 5;
/// Index visits and sub-squares per partly covered cell before it is left
/// unproven (its upper bound unchanged).
const UNION_WORK: u64 = 4096;
/// Most committed columns one partly covered cell is tested against.
const UNION_CANDIDATES: usize = 64;

#[derive(Clone, Debug)]
pub(crate) struct CommittedColumn {
    sweep: ClearedSweep,
    tolerance_mm: f64,
}

// Persistent binary forest: clones share immutable trees, append takes O(log N),
// and drop depth is at most18 rather than one recursive link per tool move.
// Digests are computed lazily on first request (same formula and bytes as an
// eager computation): planning never reads them, so it no longer pays for
// several SHA-256 compressions per tool move.
#[derive(Debug)]
enum ColumnNode {
    Leaf {
        column: CommittedColumn,
        digest: OnceLock<[u8; 32]>,
    },
    Branch {
        size: usize,
        digest: OnceLock<[u8; 32]>,
        children: [Arc<ColumnNode>; 2],
        /// Union of the descendants' query extents (a search index only:
        /// never a clearance premise by itself).
        reach: ColumnReach,
    },
}
/// What a subtree's sweeps can witness: their outer boxes, lowest floor,
/// largest containable ball and finest tolerance. Pruning only.
#[derive(Clone, Copy, Debug)]
struct ColumnReach {
    bounds: [Point; 2],
    min_floor: f64,
    /// The lowest height any sweep below proves its column empty from (its
    /// floor plus its path error): what `history_clearance` prunes on.
    min_clear_from: f64,
    max_extent: f64,
    min_tolerance: f64,
}
impl ColumnReach {
    fn of(column: &CommittedColumn) -> Self {
        Self {
            bounds: column.sweep.bounds(),
            min_floor: column.sweep.floor(),
            min_clear_from: column.sweep.clear_from(),
            max_extent: column.sweep.max_witness_extent(),
            min_tolerance: column.tolerance_mm,
        }
    }
    fn union(a: Self, b: Self) -> Self {
        Self {
            bounds: [
                std::array::from_fn(|k| a.bounds[0][k].min(b.bounds[0][k])),
                std::array::from_fn(|k| a.bounds[1][k].max(b.bounds[1][k])),
            ],
            min_floor: a.min_floor.min(b.min_floor),
            min_clear_from: a.min_clear_from.min(b.min_clear_from),
            max_extent: a.max_extent.max(b.max_extent),
            min_tolerance: a.min_tolerance.min(b.min_tolerance),
        }
    }
}
impl ColumnNode {
    fn reach(&self) -> ColumnReach {
        match self {
            Self::Leaf { column, .. } => ColumnReach::of(column),
            Self::Branch { reach, .. } => *reach,
        }
    }
    fn size(&self) -> usize {
        match self {
            Self::Leaf { .. } => 1,
            Self::Branch { size, .. } => *size,
        }
    }
    fn digest(&self) -> &[u8; 32] {
        match self {
            Self::Leaf { column, digest } => digest.get_or_init(|| {
                let mut hash = Sha256::new();
                hash.update(b"camlib-native-column-leaf-v1");
                hash.update(column.tolerance_mm.to_le_bytes());
                hash.update(column.sweep.fingerprint());
                hash.finalize().into()
            }),
            Self::Branch {
                digest, children, ..
            } => digest.get_or_init(|| {
                let mut hash = Sha256::new();
                hash.update(b"camlib-native-column-branch-v1");
                hash.update(children[0].digest());
                hash.update(children[1].digest());
                hash.finalize().into()
            }),
        }
    }
}

#[derive(Clone, Debug, Default)]
struct CommittedColumns {
    roots: Vec<Arc<ColumnNode>>,
    count: usize,
    /// The operation of the last committed native move, and the history
    /// position where its columns start: the union rule (`union_covered`)
    /// combines only one operation's own passes, so an operation's stock
    /// changes depend on its starting cells and its own moves alone (as a
    /// regeneration cache keyed by the starting cells assumes).
    operation: Option<String>,
    union_from: usize,
    /// The largest tolerance any column was committed with.
    tolerance_mm: f64,
}
impl CommittedColumns {
    fn enter(&mut self, operation: &str) {
        if self.operation.as_deref() != Some(operation) {
            self.operation = Some(operation.to_string());
            self.union_from = self.count;
        }
    }
    /// Where `operation`'s own columns start (`count`: none yet).
    fn own_from(&self, operation: &str) -> usize {
        if self.operation.as_deref() == Some(operation) {
            self.union_from
        } else {
            self.count
        }
    }
    fn append(&mut self, column: CommittedColumn) {
        if self.count == MAX_COMMITTED_COLUMNS {
            // Retaining an older valid prefix is conservative. Never evict a
            // premise by pretending the discarded future geometry was clear.
            return;
        }
        let tolerance_mm = column.tolerance_mm;
        let mut node = Arc::new(ColumnNode::Leaf {
            column,
            digest: OnceLock::new(),
        });
        while self
            .roots
            .last()
            .is_some_and(|old| old.size() == node.size())
        {
            let left = self.roots.pop().unwrap();
            node = Arc::new(ColumnNode::Branch {
                size: node.size() * 2,
                digest: OnceLock::new(),
                reach: ColumnReach::union(left.reach(), node.reach()),
                children: [left, node],
            });
        }
        self.roots.push(node);
        self.count += 1;
        self.tolerance_mm = self.tolerance_mm.max(tolerance_mm);
    }

    fn collect(
        &self,
        tolerance_mm: f64,
        cancel: &impl CancelSignal,
    ) -> Result<Vec<ClearedSweep>, String> {
        self.collect_from(0, tolerance_mm, cancel)
    }

    /// Sweeps of columns `first..count` (history order), skipping whole
    /// subtrees before `first`: O(log N + returned).
    fn collect_from(
        &self,
        first: usize,
        tolerance_mm: f64,
        cancel: &impl CancelSignal,
    ) -> Result<Vec<ClearedSweep>, String> {
        let mut result = Vec::with_capacity(self.count.saturating_sub(first));
        let mut stack = Vec::with_capacity(self.roots.len() + 64);
        let mut start = self.count;
        for root in self.roots.iter().rev() {
            start -= root.size();
            stack.push((root.as_ref(), start));
        }
        while let Some((node, start)) = stack.pop() {
            cancelled(cancel)?;
            if start + node.size() <= first {
                continue;
            }
            match node {
                ColumnNode::Leaf { column, .. } if column.tolerance_mm <= tolerance_mm => {
                    result.push(column.sweep.clone());
                }
                ColumnNode::Leaf { .. } => {}
                ColumnNode::Branch { children, .. } => {
                    let half = children[0].size();
                    stack.push((&children[1], start + half));
                    stack.push((&children[0], start));
                }
            }
        }
        cancelled(cancel)?;
        Ok(result)
    }

    /// Columns at history positions `first..` that may clear part of the XY
    /// box `area` from height `z` up at `tolerance_mm` or finer (footprint
    /// boxes meet it), into `out`. `false` when `work` (index visits) or the
    /// candidate cap runs out: unproven, never clear.
    fn overlapping<'a>(
        &'a self,
        first: usize,
        area: [Point; 2],
        z: f64,
        tolerance_mm: f64,
        work: &mut u64,
        out: &mut Vec<&'a CommittedColumn>,
    ) -> bool {
        let mut stack = Vec::with_capacity(self.roots.len() + 64);
        let mut start = 0;
        for root in &self.roots {
            stack.push((root.as_ref(), start));
            start += root.size();
        }
        while let Some((node, start)) = stack.pop() {
            if start + node.size() <= first {
                continue;
            }
            if *work == 0 {
                return false;
            }
            *work -= 1;
            let reach = node.reach();
            if reach.min_floor > z
                || reach.min_tolerance > tolerance_mm
                || (0..2)
                    .any(|k| reach.bounds[1][k] < area[0][k] || reach.bounds[0][k] > area[1][k])
            {
                continue;
            }
            match node {
                ColumnNode::Leaf { column, .. } => {
                    if out.len() == UNION_CANDIDATES {
                        return false;
                    }
                    out.push(column);
                }
                ColumnNode::Branch { children, .. } => {
                    let half = children[0].size();
                    stack.push((&children[0], start));
                    stack.push((&children[1], start + half));
                }
            }
        }
        true
    }

    /// The leaf at history position `index`: O(log N).
    fn leaf(&self, index: usize) -> Option<&Arc<ColumnNode>> {
        let mut start = 0;
        for root in &self.roots {
            if index < start + root.size() {
                let (mut node, mut offset) = (root, index - start);
                while let ColumnNode::Branch { children, .. } = node.as_ref() {
                    let half = children[0].size();
                    (node, offset) = if offset < half {
                        (&children[0], offset)
                    } else {
                        (&children[1], offset - half)
                    };
                }
                return Some(node);
            }
            start += root.size();
        }
        None
    }
}

/// A consumer's position in a stock's committed-column history, so a cached
/// query structure can append only new sweeps instead of rebuilding from the
/// whole history at every query.
///
/// Validity is proved, not assumed: the cursor holds the last leaf it consumed.
/// Native commits only append, so the history still extends the consumed
/// prefix exactly when the leaf at that position is the same allocation.
/// Undo, wire/foreign deltas and diverged forks replace or differ in that
/// leaf, and the consumer must rebuild. Holding the `Arc` keeps the allocation
/// alive, so a pointer match can never be a reused address.
#[derive(Default)]
pub(crate) struct ClearanceCursor {
    seen: usize,
    last: Option<Arc<ColumnNode>>,
    tolerance_mm: f64,
}

#[derive(Clone, Debug)]
pub struct Stock {
    spec: StockSpec,
    nx: usize,
    ny: usize,
    step: [f64; 2],
    generation: u64,
    cells: Vec<HeightBounds>,
    contact_index: Option<crate::stock_contact::HeightIndex>,
    initial_full_stock: bool,
    initial_volume_mm3: Option<[f64; 2]>,
    identity: u64,
    receipt_requested: Arc<AtomicBool>,
    last_receipt: Option<Arc<SweepReceipt>>,
    committed_columns: CommittedColumns,
    /// Lowest certified plunge tip per (tool, axis position).
    axial_columns: std::collections::HashMap<(String, [u64; 2]), f64>,
    /// An undercut or a foreign transaction may have left material above
    /// empty space: no axial certificate applies any more.
    heightfield_unproven: bool,
    /// Where the last committed motion left its tool (that pose is empty).
    departure: Option<(String, Vec3)>,
    /// During oriented-spindle shifts: the tool and the bottom pose of the
    /// plunge they started from (the bore axis).
    bore: Option<(String, Vec3)>,
}

impl StockSpec {
    pub(crate) fn grid(&self) -> Result<(usize, usize, [f64; 2]), String> {
        if self.version != 1
            || self.units != "mm"
            || !self
                .min_mm
                .iter()
                .chain(&self.max_mm)
                .all(|n| n.is_finite() && n.abs() <= 1e6)
            || !self.cell_size_mm.is_finite()
            || !(1e-5..=1e6).contains(&self.cell_size_mm)
            || self.max_cells == 0
            || self.max_cells > MAX_CELLS
            || (0..3).any(|k| self.max_mm[k] - self.min_mm[k] < 1e-6)
        {
            return Err(
                "stock needs version 1/mm, bounded positive extents, spacing and cell budget"
                    .into(),
            );
        }
        let dims: [f64; 2] = std::array::from_fn(|k| {
            ((self.max_mm[k] - self.min_mm[k]) / self.cell_size_mm)
                .ceil()
                .max(1.)
        });
        if dims.iter().any(|n| *n > self.max_cells as f64)
            || dims[0] * dims[1] > self.max_cells as f64
        {
            return Err(format!(
                "stock grid of {} x {} cells exceeds max_cells {} (hard ceiling {MAX_CELLS}); increase cell_size_mm or max_cells",
                dims[0], dims[1], self.max_cells
            ));
        }
        let [nx, ny] = dims.map(|n| n as usize);
        let step = std::array::from_fn(|k| (self.max_mm[k] - self.min_mm[k]) / dims[k]);
        let magnitude = self
            .min_mm
            .iter()
            .chain(&self.max_mm)
            .map(|n| n.abs())
            .fold(1., f64::max);
        if step
            .iter()
            .any(|d| *d < 1e-6_f64.max(128. * f64::EPSILON * magnitude))
        {
            return Err("stock cells are below supported numerical resolution".into());
        }
        Ok((nx, ny, step))
    }
}

impl Stock {
    pub fn new(spec: StockSpec) -> Result<Self, String> {
        let (nx, ny, step) = spec.grid()?;
        crate::resources::check_memory(
            (nx * ny) as u128 * std::mem::size_of::<HeightBounds>() as u128,
            "stock grid",
        )?;
        let cells = vec![
            HeightBounds {
                lower_mm: spec.max_mm[2],
                upper_mm: spec.max_mm[2]
            };
            nx * ny
        ];
        Ok(Self {
            spec,
            nx,
            ny,
            step,
            generation: 0,
            cells,
            contact_index: None,
            initial_full_stock: true,
            initial_volume_mm3: None,
            identity: new_identity(),
            receipt_requested: Arc::new(AtomicBool::new(false)),
            last_receipt: None,
            committed_columns: CommittedColumns::default(),
            axial_columns: Default::default(),
            heightfield_unproven: false,
            departure: None,
            bore: None,
        })
    }
    /// A run's starting stock given cell by cell (a mesh stock converted to
    /// the height field, `crate::stock_mesh`): the run's initial state, with
    /// its history (none) known, unlike a snapshot.
    pub(crate) fn from_initial_cells(
        spec: StockSpec,
        cells: Vec<HeightBounds>,
    ) -> Result<Self, String> {
        let mut stock = Self::new(spec)?;
        if cells.len() != stock.cells.len() || cells.iter().any(|&h| !height_valid(h, &stock.spec))
        {
            return Err("initial stock has invalid height bounds or grid length".into());
        }
        let top = stock.spec.max_mm[2];
        stock.initial_full_stock = cells.iter().all(|h| h.lower_mm == top);
        stock.cells = cells;
        Ok(stock)
    }
    pub fn spec(&self) -> &StockSpec {
        &self.spec
    }

    pub fn from_initial(spec: StockSpec, initial: &InitialStock) -> Result<Self, String> {
        initial.validate(&spec)?;
        let mut stock = Self::new(spec)?;
        stock.cells.clone_from(&initial.bounds);
        stock.initial_full_stock = false;
        stock.initial_volume_mm3 = Some(initial.volume(&stock.spec)?);
        Ok(stock)
    }

    pub(crate) fn initial_volume(&mut self, initial: &InitialStock) -> Result<(), String> {
        self.initial_volume_mm3 = Some(initial.volume(&self.spec)?);
        Ok(())
    }
    pub fn dimensions(&self) -> [usize; 2] {
        [self.nx, self.ny]
    }
    pub fn cell_widths_mm(&self) -> [f64; 2] {
        self.step
    }
    pub fn cells(&self) -> &[HeightBounds] {
        &self.cells
    }
    pub(crate) fn request_sweep_receipts(&self) {
        self.receipt_requested.store(true, Ordering::Relaxed);
    }
    pub(crate) fn identity(&self) -> u64 {
        self.identity
    }
    pub(crate) fn last_sweep_receipt(&self) -> Option<&SweepReceipt> {
        self.last_receipt.as_deref()
    }
    pub(crate) fn committed_clearance(
        &self,
        tolerance_mm: f64,
        cancel: &impl CancelSignal,
    ) -> Result<Vec<ClearedSweep>, String> {
        self.committed_columns.collect(tolerance_mm, cancel)
    }
    /// The committed cleared columns, oldest first, with their tolerances,
    /// and whether the stock may no longer be a height field: the state the
    /// union rule (`union_covered`) reads, kept in producer checkpoints so a
    /// resumed run lowers the same cells as one that never stopped.
    pub(crate) fn clearance_state(&self) -> ClearanceState {
        let mut out = Vec::with_capacity(self.committed_columns.count);
        for i in 0..self.committed_columns.count {
            if let Some(ColumnNode::Leaf { column, .. }) =
                self.committed_columns.leaf(i).map(AsRef::as_ref)
            {
                out.push((column.tolerance_mm, column.sweep.clone()));
            }
        }
        ClearanceState {
            unproven: self.heightfield_unproven,
            operation: self.committed_columns.operation.clone(),
            union_from: self.committed_columns.union_from,
            columns: out,
        }
    }
    /// Restore `clearance_state` from a producer checkpoint bound to exactly
    /// these cells (the caller checks the binding).
    pub(crate) fn restore_clearance(&mut self, state: ClearanceState) -> Result<(), String> {
        let ClearanceState {
            unproven,
            operation,
            union_from,
            columns,
        } = state;
        if columns.len() > MAX_COMMITTED_COLUMNS
            || union_from > columns.len()
            || operation.as_deref().is_some_and(|o| !valid_id(o))
            || columns
                .iter()
                .any(|(t, _)| !t.is_finite() || !(1e-7..=0.1).contains(t))
        {
            return Err("invalid checkpoint cleared columns".into());
        }
        self.committed_columns = CommittedColumns::default();
        for (tolerance_mm, sweep) in columns {
            self.committed_columns.append(CommittedColumn {
                sweep,
                tolerance_mm,
            });
        }
        self.committed_columns.operation = operation;
        self.committed_columns.union_from = union_from;
        self.heightfield_unproven = unproven;
        Ok(())
    }
    /// Whether the committed cutter sweeps prove a tool section clear along
    /// `motion`: everything within `radius` of its XY path, from `height`
    /// above its lowest tip up, is inside columns those sweeps emptied or
    /// outside the stock (`engagement::cover`: the claim and its proof).
    /// One sweep the move repeats, several it runs along, or the union of
    /// the passes it runs between: a shank as wide as the cutter is clear
    /// where the layers above cut (#200). The planner, the stock sweep's
    /// cutting-length check and stock-contact verification all ask this one
    /// question, so they agree.
    ///
    /// Native committed columns only: decoded deltas or undercut stock cannot
    /// manufacture this proof. Stock only loses material after a certificate.
    /// Unproven, or out of `work`, is `false`.
    pub(crate) fn history_clearance(
        &self,
        motion: &Motion,
        radius: f64,
        height: f64,
        work: &mut u64,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        cancelled(cancel)?;
        if self.heightfield_unproven || self.committed_columns.count == 0 {
            return Ok(false);
        }
        let (a, b) = (motion.curve.start(), motion.curve.end());
        let bottom = a[2].min(b[2]) + height;
        // The move's own path error, at the tolerance the history was
        // committed with (and this move will be).
        let error = crate::verification::path_error(motion, self.committed_columns.tolerance_mm);
        // A box holding the section's sweep: an arc within its circle.
        let (mut lo, mut hi) = (
            [a[0].min(b[0]), a[1].min(b[1])],
            [a[0].max(b[0]), a[1].max(b[1])],
        );
        if let Curve::Arc { center, .. } | Curve::Helix { center, .. } = motion.curve {
            let r = (a[0] - center[0]).hypot(a[1] - center[1]);
            for k in 0..2 {
                lo[k] = lo[k].min(center[k] - r);
                hi[k] = hi[k].max(center[k] + r);
            }
        }
        let pad = radius + 2. * error;
        if !(pad.is_finite() && lo.iter().chain(&hi).all(|v| v.is_finite())) {
            return Ok(false);
        }
        // Every sweep whose reach meets that box and whose floor is low
        // enough, newest first.
        let mut sweeps = vec![];
        let mut stack: Vec<&ColumnNode> = self
            .committed_columns
            .roots
            .iter()
            .map(AsRef::as_ref)
            .collect();
        while let Some(node) = stack.pop() {
            cancelled(cancel)?;
            if *work == 0 {
                return Ok(false);
            }
            *work -= 1;
            let ColumnReach {
                bounds,
                min_clear_from,
                ..
            } = node.reach();
            if min_clear_from > bottom
                || (0..2).any(|k| bounds[0][k] > hi[k] + pad || bounds[1][k] < lo[k] - pad)
            {
                continue;
            }
            match node {
                ColumnNode::Leaf { column, .. } => sweeps.push(&column.sweep),
                ColumnNode::Branch { children, .. } => {
                    stack.push(&children[0]);
                    stack.push(&children[1]);
                }
            }
        }
        crate::engagement::cover::contains(
            &sweeps,
            crate::engagement::cover::Section {
                curve: &motion.curve,
                radius,
                bottom,
                error,
            },
            [
                [self.spec.min_mm[0], self.spec.min_mm[1]],
                [self.spec.max_mm[0], self.spec.max_mm[1]],
            ],
            work,
            cancel,
        )
    }
    /// Sweeps committed since `cursor`, and whether the consumer must rebuild
    /// (`true`: the returned sweeps are the whole history, not an extension).
    pub(crate) fn committed_clearance_since(
        &self,
        cursor: &mut ClearanceCursor,
        tolerance_mm: f64,
        cancel: &impl CancelSignal,
    ) -> Result<(bool, Vec<ClearedSweep>), String> {
        let columns = &self.committed_columns;
        let extends = cursor.tolerance_mm == tolerance_mm
            && cursor.seen <= columns.count
            && match &cursor.last {
                None => cursor.seen == 0,
                Some(last) => cursor
                    .seen
                    .checked_sub(1)
                    .and_then(|i| columns.leaf(i))
                    .is_some_and(|leaf| Arc::ptr_eq(leaf, last)),
            };
        let first = if extends { cursor.seen } else { 0 };
        let sweeps = columns.collect_from(first, tolerance_mm, cancel)?;
        *cursor = ClearanceCursor {
            seen: columns.count,
            last: columns
                .count
                .checked_sub(1)
                .and_then(|i| columns.leaf(i))
                .cloned(),
            tolerance_mm,
        };
        Ok((!extends, sweeps))
    }
    /// Whether `tool`'s cutting part is in certified empty space at every
    /// tip height from `tip_z` up on the vertical axis through `xy`.
    /// Whether `tool` leaves `start` straight up from where the last committed
    /// motion left it: that pose is empty space (the swept cutter removed what
    /// it occupied), and in a height field everything above empty space is
    /// empty, so a vertical move up keeps the cutting part clear.
    /// Whether the stock may no longer be a height field (an undercut, a
    /// restored snapshot or a foreign transaction): no history-based proof.
    pub(crate) fn heightfield_unproven(&self) -> bool {
        self.heightfield_unproven
    }
    pub fn departs_clear(&self, tool: &str, start: Vec3) -> bool {
        !self.heightfield_unproven
            && self
                .departure
                .as_ref()
                .is_some_and(|(t, p)| t == tool && *p == start)
    }
    /// The bore an oriented-spindle shift of `tool` starting at `start` is
    /// in: the axis and the lowest certified tip of the plunge the shifts
    /// started from (the first shift starts at that plunge's end).
    pub(crate) fn shift_bore(&self, tool: &str, start: Vec3) -> Option<([f64; 2], f64)> {
        if self.heightfield_unproven {
            return None;
        }
        let axis = match (&self.bore, &self.departure) {
            (Some((t, p)), _) if t == tool => *p,
            (None, Some((t, p))) if t == tool && *p == start => *p,
            _ => return None,
        };
        let xy = [axis[0], axis[1]];
        self.axial_columns
            .get(&(tool.to_string(), xy.map(f64::to_bits)))
            .map(|z| (xy, *z))
    }
    pub(crate) fn axial_clear(&self, tool: &str, xy: [f64; 2], tip_z: f64) -> bool {
        !self.heightfield_unproven
            && self
                .axial_columns
                .get(&(tool.to_string(), xy.map(f64::to_bits)))
                .is_some_and(|z| tip_z >= *z)
    }
    pub(crate) fn committed_clearance_fingerprint(&self) -> [u8; 32] {
        let mut hash = Sha256::new();
        hash.update(b"camlib-native-column-forest-v1");
        hash.update((self.committed_columns.count as u64).to_le_bytes());
        for root in &self.committed_columns.roots {
            hash.update((root.size() as u64).to_le_bytes());
            hash.update(root.digest());
        }
        hash.finalize().into()
    }
    pub fn generation(&self) -> u64 {
        self.generation
    }
    /// Prove a complete cutter sweep misses the ORIGINAL stock envelope.
    /// This uses analytic curve extents plus the shared engineering path-error
    /// guard. It says nothing about holders, fixtures or machine clearance.
    pub fn sweep_misses_stock_envelope(
        &self,
        motion: &Motion,
        radius: f64,
        tolerance_mm: f64,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        cancelled(cancel)?;
        if !radius.is_finite()
            || radius <= 0.
            || !tolerance_mm.is_finite()
            || !(1e-7..=0.01).contains(&tolerance_mm)
        {
            return Err(
                "sweep envelope query needs positive radius and supported tolerance".into(),
            );
        }
        let bounds = motion.curve.prepare(tolerance_mm)?.bounds();
        let guard = crate::verification::path_error(motion, tolerance_mm)
            + 512. * f64::EPSILON * (1. + radius);
        let pad = radius + guard;
        Ok((0..2).any(|k| {
            bounds.max[k] + pad < self.spec.min_mm[k] || bounds.min[k] - pad > self.spec.max_mm[k]
        }) || bounds.min[2] - guard > self.spec.max_mm[2])
    }
    /// Whole closed rectangle upper-height certificate for front coverage.
    /// Include both neighboring cells at grid boundaries via a numeric pad.
    pub(crate) fn bounds_clear(
        &self,
        bounds: [[f64; 2]; 2],
        z: f64,
        work: &mut u64,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("coverage.stock_box");
        let scale = bounds
            .into_iter()
            .flatten()
            .map(f64::abs)
            .fold(1., f64::max);
        let guard = 1024. * f64::EPSILON * (1. + scale);
        let Some(rect) = self.rect(bounds[0].map(|v| v - guard), bounds[1].map(|v| v + guard))
        else {
            return Ok(true);
        };
        for y in rect.y0..=rect.y1 {
            if cancel.is_cancelled() {
                return Err("stock-front stock coverage cancelled".into());
            }
            for x in rect.x0..=rect.x1 {
                if *work == 0 {
                    return Err("stock-front stock coverage work limit; coverage unproven".into());
                }
                *work -= 1;
                if self.cells[y * self.nx + x].upper_mm > z {
                    return Ok(false);
                }
            }
        }
        Ok(true)
    }
    /// Conservative material query for a flat horizontal cutter stroke. Whole
    /// cells and analytic full-path distance are used, never endpoint samples.
    /// Work exhaustion retains the cut; false proves no upper stock above the
    /// explicit height budget in an enclosing full cutter footprint.
    pub fn may_remove_material(
        &self,
        motion: &Motion,
        radius: f64,
        rest_tolerance: f64,
        tolerance_mm: f64,
        remaining_work: &mut usize,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        if !radius.is_finite()
            || radius <= 0.
            || !rest_tolerance.is_finite()
            || !(0.0..=0.1).contains(&rest_tolerance)
            || !tolerance_mm.is_finite()
            || !(1e-7..=0.01).contains(&tolerance_mm)
            || !matches!(
                motion.curve,
                Curve::Line { .. }
                    | Curve::Arc {
                        plane: crate::motion::Plane::Xy,
                        ..
                    }
            )
            || motion.curve.start()[2] != motion.curve.end()[2]
        {
            return Err(
                "rest query requires a flat horizontal line/XY arc and explicit height budget"
                    .into(),
            );
        }
        cancelled(cancel)?;
        let path = motion.curve.prepare(tolerance_mm)?;
        let bounds = path.bounds();
        let guard = crate::verification::path_error(motion, tolerance_mm)
            + 512. * f64::EPSILON * (1. + radius);
        let pad = radius + guard;
        if *remaining_work == 0 {
            return Ok(true);
        }
        *remaining_work -= 1;
        let Some(rect) = self.rect(
            [bounds.min[0] - pad, bounds.min[1] - pad],
            [bounds.max[0] + pad, bounds.max[1] + pad],
        ) else {
            return Ok(false);
        };
        let cell_radius = self.step[0].hypot(self.step[1]) / 2.;
        for y in rect.y0..=rect.y1 {
            cancelled(cancel)?;
            for x in rect.x0..=rect.x1 {
                if *remaining_work == 0 {
                    return Ok(true);
                }
                *remaining_work -= 1;
                let index = y * self.nx + x;
                if self.cells[index].upper_mm <= bounds.min[2] + rest_tolerance {
                    continue;
                }
                let center = self.cell_center(index).unwrap();
                if crate::verification::circle_axis_distance(motion, &path, [0., 1.], center)
                    .is_none_or(|distance| distance <= radius + cell_radius + guard)
                {
                    return Ok(true);
                }
            }
        }
        Ok(false)
    }
    pub fn enable_contact_queries(
        &mut self,
        cancel: &impl CancelSignal,
    ) -> Result<crate::stock_contact::IndexStats, String> {
        if cancel.is_cancelled() {
            return Err("stock contact index preparation cancelled".into());
        }
        if self.contact_index.is_none() {
            let prepared = crate::stock_contact::HeightIndex::new(
                &self.spec,
                &self.cells,
                self.initial_full_stock,
                cancel,
            )?;
            self.contact_index = Some(prepared);
        }
        Ok(self.contact_index.as_ref().unwrap().stats)
    }
    pub fn contact_index_stats(&self) -> Option<crate::stock_contact::IndexStats> {
        self.contact_index.as_ref().map(|i| i.stats)
    }
    pub(crate) fn contact_index(&self) -> Option<&crate::stock_contact::HeightIndex> {
        self.contact_index.as_ref()
    }
    /// The highest upper bound of the stock over the cells a disk of radius
    /// `r` at `xy` can touch (whole cells: an overestimate, never an
    /// underestimate); the stock's bottom where it touches none.
    pub fn top_under_disk(&self, xy: [f64; 2], r: f64) -> f64 {
        let [wx, wy] = self.step;
        let reach = r + 0.5 * wx.hypot(wy);
        let range = |c: f64, lo: f64, w: f64, n: usize| {
            let a = ((c - reach - lo) / w).floor().max(0.) as usize;
            let b = (((c + reach - lo) / w).ceil().max(0.) as usize).min(n);
            a..b
        };
        let mut top = self.spec.min_mm[2];
        for j in range(xy[1], self.spec.min_mm[1], wy, self.ny) {
            for i in range(xy[0], self.spec.min_mm[0], wx, self.nx) {
                let k = j * self.nx + i;
                let c = self.cell_center(k).unwrap_or(xy);
                if (c[0] - xy[0]).hypot(c[1] - xy[1]) <= reach {
                    top = top.max(self.cells[k].upper_mm);
                }
            }
        }
        top
    }
    pub fn cell_center(&self, index: usize) -> Option<[f64; 2]> {
        (index < self.cells.len()).then(|| {
            [
                self.spec.min_mm[0] + (index % self.nx) as f64 * self.step[0] + self.step[0] / 2.,
                self.spec.min_mm[1] + (index / self.nx) as f64 * self.step[1] + self.step[1] / 2.,
            ]
        })
    }
    pub fn snapshot(&self) -> StockSnapshot {
        StockSnapshot {
            spec: self.spec.clone(),
            generation: self.generation,
            cells: self.cells.clone(),
        }
    }
    pub fn from_snapshot(snapshot: StockSnapshot) -> Result<Self, String> {
        let (nx, ny, step) = snapshot.spec.grid()?;
        if snapshot.cells.len() != nx * ny
            || snapshot
                .cells
                .iter()
                .any(|&h| !height_valid(h, &snapshot.spec))
        {
            return Err("snapshot has invalid height bounds or grid length".into());
        }
        Ok(Self {
            spec: snapshot.spec,
            nx,
            ny,
            step,
            generation: snapshot.generation,
            cells: snapshot.cells,
            contact_index: None,
            initial_full_stock: false,
            initial_volume_mm3: None,
            identity: new_identity(),
            receipt_requested: Arc::new(AtomicBool::new(false)),
            last_receipt: None,
            committed_columns: CommittedColumns::default(),
            axial_columns: Default::default(),
            // A snapshot does not carry its history: no height-field proof.
            heightfield_unproven: true,
            departure: None,
            bore: None,
        })
    }
    pub fn volume(&self) -> StockVolume {
        let area = self.step[0] * self.step[1];
        let base = self.spec.min_mm[2];
        let lower = self
            .cells
            .iter()
            .map(|c| (c.lower_mm - base) * area)
            .sum::<f64>();
        let upper = self
            .cells
            .iter()
            .map(|c| (c.upper_mm - base) * area)
            .sum::<f64>();
        let total = (self.spec.max_mm[2] - base) * area * self.cells.len() as f64;
        let initial = self.initial_volume_mm3.unwrap_or([total, total]);
        StockVolume {
            remaining_lower_mm3: lower,
            remaining_upper_mm3: upper,
            removed_lower_mm3: (initial[0] - upper).max(0.),
            removed_upper_mm3: (initial[1] - lower).max(0.),
        }
    }

    /// Plan [0,fraction] from the stock state before this move. Does not mutate.
    /// The caller has checked an owned native artifact, its current-input key,
    /// and full typed motion attribution. Re-certify only analytic whole columns;
    /// never infer them from decoded cell heights. No height sweep is computed.
    pub(crate) fn certify_verified_replay(
        &self,
        delta: &mut StockDelta,
        tool: &PreparedAssembly,
        motion: &Motion,
        options: SweepOptions,
        cancel: &impl CancelSignal,
    ) -> Result<ReplayClearanceStats, String> {
        cancelled(cancel)?;
        options.validate()?;
        if delta.spec != self.spec
            || delta.base_generation != self.generation
            || delta.operation != motion.operation
            || delta.motion_id != motion.id
            || delta.fraction != 1.
            || motion.tool != tool.definition().id
        {
            return Err("verified replay stock/motion/tool/fraction mismatch".into());
        }
        let path = motion.curve.prepare(options.tolerance_mm)?;
        motion.validate_feed(None)?;
        let mut stats = ReplayClearanceStats::default();
        let mut column = None;
        let mut undercut = false;
        if motion.kind.cuts()
            && matches!(tool.cutter().definition().shape, CutterShape::Flat)
            && motion.curve.start()[2] == motion.curve.end()[2]
            && matches!(
                motion.curve,
                Curve::Line { .. }
                    | Curve::Arc {
                        plane: Plane::Xy,
                        ..
                    }
            )
        {
            let bounds = path.bounds();
            let scale = self
                .spec
                .min_mm
                .iter()
                .chain(&self.spec.max_mm)
                .chain(&bounds.min)
                .chain(&bounds.max)
                .map(|v| v.abs())
                .fold(1., f64::max);
            let guard = options.tolerance_mm.max(512. * f64::EPSILON * scale);
            let radius = tool.cutter().definition().diameter_mm / 2.;
            let raw_error = crate::verification::path_error(motion, options.tolerance_mm) + guard;
            let mut error = (raw_error / guard).ceil() * guard;
            if error < raw_error {
                error += guard;
            }
            // Same enclosing rectangle/reach conditions as native preview.
            // Full original-stock reach is O(1); a shorter tool needs bounded
            // whole-cell upper bounds over the ENTIRE affected rectangle.
            let pad = radius
                + self.step[0].hypot(self.step[1]) / 2.
                + error.max(options.chord_error_mm + 2. * guard);
            if let Some(rect) = self.rect(
                [bounds.min[0] - pad, bounds.min[1] - pad],
                [bounds.max[0] + pad, bounds.max[1] + pad],
            ) {
                if rect.count() > options.max_scratch_cells {
                    return Err("verified replay clearance rectangle budget exceeded".into());
                }
                if bounds.min[2] + tool.cutter().definition().cutting_length_mm
                    < self.spec.max_mm[2] + guard
                    && bounds.min[2] < self.spec.max_mm[2]
                {
                    let reachable =
                        bounds.min[2] + tool.cutter().definition().cutting_length_mm - guard;
                    let mut buried = false;
                    let whole = crate::verification::AxisSpan::new(motion, &path, [0., 1.]);
                    let cell_radius = self.step[0].hypot(self.step[1]) / 2.;
                    let touch = radius.max(
                        tool.cutter()
                            .definition()
                            .neck
                            .as_ref()
                            .map_or(0., |n| n.diameter_mm / 2.),
                    );
                    'reach: for y in rect.y0..=rect.y1 {
                        cancelled(cancel)?;
                        for x in rect.x0..=rect.x1 {
                            if stats.reach_cell_visits.is_multiple_of(1024) {
                                cancelled(cancel)?;
                            }
                            stats.reach_cell_visits += 1;
                            if stats.reach_cell_visits > options.max_cell_visits {
                                return Err(
                                    "verified replay clearance reach budget exceeded".into()
                                );
                            }
                            // Native preview certified any undercut and made
                            // no column for it; match that, never claim one.
                            // Like native preview, cells beyond the cutter
                            // and its neck are not judged for reach.
                            let upper = self.cells[y * self.nx + x].upper_mm;
                            if upper > reachable
                                && !whole.as_ref().is_some_and(|a| {
                                    beyond(
                                        a,
                                        self.cell_center(y * self.nx + x),
                                        cell_radius,
                                        error,
                                        touch,
                                    )
                                })
                            {
                                match Undercut::new(tool.cutter().definition(), guard) {
                                    Some(u) if upper <= bounds.min[2] + u.shoulder - guard => {
                                        buried = true
                                    }
                                    _ => {
                                        let mut work = options
                                            .max_cell_visits
                                            .saturating_sub(stats.reach_cell_visits);
                                        let before = work;
                                        let clear = self.history_clearance(
                                            motion,
                                            radius,
                                            tool.cutter().definition().cutting_length_mm,
                                            &mut work,
                                            cancel,
                                        )?;
                                        stats.reach_cell_visits += before - work;
                                        if clear {
                                            break 'reach;
                                        }
                                        return Err("verified replay cutting length cannot span committed stock upper bound".into());
                                    }
                                }
                            }
                        }
                    }
                    undercut = buried;
                }
                if !undercut {
                    column = ClearedSweep::from_motion(motion, radius, options.tolerance_mm)
                        .ok()
                        .map(|sweep| CommittedColumn {
                            sweep,
                            tolerance_mm: options.tolerance_mm,
                        });
                }
            }
        }
        cancelled(cancel)?;
        stats.columns = u64::from(column.is_some());
        // Runtime binding only, after certificate checks. State and history
        // still change only when the complete delta passes commit validation.
        delta.native_identity = Some(self.identity);
        delta.cleared_column = column;
        delta.axial_column = AxialColumn::from_motion(tool, motion, self.spec.max_mm[2]);
        delta.departure =
            (motion.kind.cuts() && !undercut).then(|| (motion.tool.clone(), motion.curve.end()));
        delta.shift = motion.kind == MoveKind::Shift;
        delta.receipt = None;
        Ok(stats)
    }

    pub fn preview_sweep(
        &self,
        tool: &PreparedAssembly,
        motion: &Motion,
        fraction: f64,
        options: SweepOptions,
        cancel: &impl CancelSignal,
    ) -> Result<StockDelta, String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("stock.preview");
        cancelled(cancel)?;
        options.validate()?;
        if !fraction.is_finite()
            || !(0.0..=1.0).contains(&fraction)
            || !valid_id(&motion.operation)
            || motion.tool != tool.definition().id
            || self.generation == u64::MAX
        {
            return Err(
                "invalid sweep prefix, motion attribution or exhausted stock generation".into(),
            );
        }
        let prepared = motion.curve.prepare(options.tolerance_mm)?;
        motion.validate_feed(None)?;
        let mut delta = StockDelta {
            spec: self.spec.clone(),
            base_generation: self.generation,
            motion_id: motion.id,
            operation: motion.operation.clone(),
            fraction,
            changes: vec![],
            stats: SweepStats::default(),
            native_identity: Some(self.identity),
            cleared_column: None,
            axial_column: None,
            departure: None,
            shift: motion.kind == MoveKind::Shift,
            receipt: self.receipt_requested.load(Ordering::Relaxed).then(|| {
                Arc::new(SweepReceipt {
                    base_identity: self.identity,
                    motion: motion.clone(),
                    tool: tool.definition().clone(),
                    fraction,
                    tolerance_mm: options.tolerance_mm,
                })
            }),
        };
        // Rapids, oriented shifts, probing moves and a beam's marking moves
        // never remove stock.
        if fraction == 0. || !motion.kind.cuts() {
            if motion.kind == MoveKind::Mark && fraction > 0. {
                self.mark_on_sheet(tool, motion, fraction, options, cancel)?;
            }
            return Ok(delta);
        }
        if tool.cutter().definition().shape.undercuts() {
            return Err(UNDERCUT_STOCK.into());
        }
        if matches!(tool.cutter().definition().shape, CutterShape::Beam) {
            self.beam_sweep(&mut delta, tool, motion, fraction, options, cancel)?;
            return Ok(delta);
        }
        let bounds = prepared.bounds();
        let magnitude = self
            .spec
            .min_mm
            .iter()
            .chain(&self.spec.max_mm)
            .chain(&bounds.min)
            .chain(&bounds.max)
            .map(|v| v.abs())
            .fold(1., f64::max);
        let guard = options.tolerance_mm.max(512. * f64::EPSILON * magnitude);
        let radius = tool.cutter().definition().diameter_mm / 2.;
        let cell_radius = self.step[0].hypot(self.step[1]) / 2.;
        let analytic = matches!(tool.cutter().definition().shape, CutterShape::Flat)
            && motion.curve.start()[2] == motion.curve.end()[2]
            && matches!(
                motion.curve,
                Curve::Line { .. }
                    | Curve::Arc {
                        plane: Plane::Xy,
                        ..
                    }
            );
        let raw_error = crate::verification::path_error(motion, options.tolerance_mm) + guard;
        // Round UP to the numerical-guard grid. Sub-guard coordinate roundoff
        // must not perturb every serialized height after a simple translation.
        let mut analytic_error = (raw_error / guard).ceil() * guard;
        if analytic_error < raw_error {
            analytic_error += guard;
        }
        let pad = radius
            + cell_radius
            + if analytic {
                analytic_error.max(options.chord_error_mm + 2. * guard)
            } else {
                options.chord_error_mm + 2. * guard
            };
        let Some(rect) = self.rect(
            [bounds.min[0] - pad, bounds.min[1] - pad],
            [bounds.max[0] + pad, bounds.max[1] + pad],
        ) else {
            return Ok(delta);
        };
        if rect.count() > options.max_scratch_cells {
            return Err(
                "sweep scratch-cell budget exceeded; partition motion or adjust explicit budget"
                    .into(),
            );
        }
        let mut reach_cell_visits = 0u64;
        let undercut = Undercut::new(tool.cutter().definition(), guard);
        // Reach is judged where the cutter can touch: cells provably beyond
        // its cross-section (and its neck) over the whole move are left to
        // the assembly checks (issue #129), not to its bounding rectangle.
        let whole = crate::verification::AxisSpan::new(motion, &prepared, [0., 1.]);
        let touch = radius.max(undercut.as_ref().map_or(0., |u| u.neck));
        // A straight plunge from above the stock: the cutting part sweeps the
        // whole column down to the tip, so the neck follows it through empty
        // space and nothing is left above the cut (no undercut).
        let from_above = undercut.is_some()
            && AxialColumn::from_motion(tool, motion, self.spec.max_mm[2]).is_some()
            && motion.curve.start()[2] >= self.spec.max_mm[2];
        let mut buried = false;
        if bounds.min[2] + tool.cutter().definition().cutting_length_mm
            < self.spec.max_mm[2] + guard
            && bounds.min[2] < self.spec.max_mm[2]
        {
            // Whole-cell upper bounds enclose committed material everywhere in
            // the affected rectangle. A previous operation may have removed
            // the original stock top; never infer that from tip samples/history.
            let reachable = bounds.min[2] + tool.cutter().definition().cutting_length_mm - guard;
            'reach: for j in rect.y0..=rect.y1 {
                cancelled(cancel)?;
                for i in rect.x0..=rect.x1 {
                    if reach_cell_visits.is_multiple_of(1024) {
                        cancelled(cancel)?;
                    }
                    reach_cell_visits += 1;
                    if reach_cell_visits > options.max_cell_visits {
                        return Err(
                            "sweep cell-visit budget exceeded during reach admission".into()
                        );
                    }
                    let upper = self.cells[j * self.nx + i].upper_mm;
                    if upper > reachable
                        && !whole.as_ref().is_some_and(|a| {
                            beyond(
                                a,
                                self.cell_center(j * self.nx + i),
                                cell_radius,
                                analytic_error,
                                touch,
                            )
                        })
                    {
                        // A neck may pass beside material between the cutting
                        // length and the shoulder; decided per cell below.
                        match &undercut {
                            Some(u) if upper <= bounds.min[2] + u.shoulder - guard => {
                                buried |= !from_above
                            }
                            _ => {
                                let mut work =
                                    options.max_cell_visits.saturating_sub(reach_cell_visits);
                                let before = work;
                                let clear = self.history_clearance(
                                    motion,
                                    radius,
                                    tool.cutter().definition().cutting_length_mm,
                                    &mut work,
                                    cancel,
                                )?;
                                reach_cell_visits += before - work;
                                if clear {
                                    break 'reach;
                                }
                                return Err(REACH.into());
                            }
                        }
                    }
                }
            }
        }
        let mut stats = SweepStats {
            reach_cell_visits,
            numerical_guard_mm: guard,
            ..Default::default()
        };
        if analytic {
            let (count, _) = segment_layout(&motion.curve, options)?;
            stats.segments = ((count as f64 * fraction).ceil() as usize).min(count);
            stats.max_path_error_mm = analytic_error;
            let axis = crate::verification::AxisSpan::new(motion, &prepared, [0., fraction])
                .ok_or("unsupported analytic stock axis")?;
            let z = motion.curve.start()[2];
            // The admitted motion has EXACTLY constant tip Z. Circular endpoint
            // and angular errors are lateral; keep the original vertical guard.
            let lower = (z - guard).clamp(self.spec.min_mm[2], self.spec.max_mm[2]);
            let upper = (z + guard).clamp(self.spec.min_mm[2], self.spec.max_mm[2]);
            for j in rect.y0..=rect.y1 {
                cancelled(cancel)?;
                for i in rect.x0..=rect.x1 {
                    if stats.visited_cells.is_multiple_of(1024) {
                        cancelled(cancel)?;
                    }
                    stats.visited_cells += 1;
                    if stats.visited_cells + stats.reach_cell_visits > options.max_cell_visits {
                        return Err("sweep cell-visit budget exceeded".into());
                    }
                    let index = j * self.nx + i;
                    let before = self.cells[index];
                    let mut h = before;
                    if h.lower_mm <= lower && h.upper_mm <= upper {
                        continue;
                    }
                    let p = self.cell_center(j * self.nx + i).unwrap();
                    let distance = axis.distance(p);
                    stats.analytic_path_queries += 1;
                    let mut under = false;
                    if distance - cell_radius - analytic_error > touch {
                        // Untouched by the cutter and its neck: unchanged.
                        continue;
                    }
                    if buried {
                        let near = (distance - cell_radius - analytic_error).max(0.);
                        let u = undercut.as_ref().expect("buried needs a neck");
                        under = u.classify(before, z, distance, cell_radius, analytic_error)?;
                        if under && near <= radius {
                            stats.undercut_cells += 1;
                            if before.lower_mm >= z + u.cutting {
                                continue;
                            }
                        }
                    }
                    // Distance to a closed path is 1-Lipschitz. These bounds
                    // apply to every point in the cell, over the whole prefix.
                    if distance - cell_radius - analytic_error <= radius {
                        h.lower_mm = h.lower_mm.min(lower);
                    }
                    if !under && distance + cell_radius + analytic_error <= radius {
                        h.upper_mm = h.upper_mm.min(upper);
                    } else if !buried
                        && h.upper_mm > upper
                        && distance - cell_radius - analytic_error < radius
                        && self.union_covered(
                            &motion.operation,
                            index,
                            &axis,
                            radius,
                            analytic_error,
                            z,
                            options.tolerance_mm,
                        )
                    {
                        stats.union_cells += 1;
                        h.upper_mm = h.upper_mm.min(upper);
                    }
                    delta.record_change(index, before, h, options.max_changed_cells)?;
                }
            }
        } else {
            let mut scratch = Vec::with_capacity(rect.count());
            for j in rect.y0..=rect.y1 {
                cancelled(cancel)?;
                scratch
                    .extend_from_slice(&self.cells[j * self.nx + rect.x0..=j * self.nx + rect.x1]);
            }
            stats.scratch_cells = scratch.len();
            stats.scratch_bytes = scratch.len() * std::mem::size_of::<HeightBounds>();
            let mut segments =
                segments(&motion.curve, &prepared, fraction, options, guard, cancel)?;
            stats.segments = segments.len();
            // A move along Z alone: every segment covers the same cells with
            // the same distance, and the bounds below are minima over the
            // segments' lowest points, so one segment from the first point
            // to the last gives exactly the same result.
            // (Not for a buried necked tool: each segment is classified
            // at its own tip height.)
            if let Curve::Line { start, end } = motion.curve
                && !buried
                && start[0] == end[0]
                && start[1] == end[1]
                && segments.len() > 1
            {
                let (first, last) = (segments[0].a, segments[segments.len() - 1].b);
                let error = segments[0].error;
                segments = vec![Segment {
                    a: first,
                    b: last,
                    error,
                }];
            }
            stats.materialized_segments = segments.len();
            for s in &segments {
                cancelled(cancel)?;
                stats.max_path_error_mm = stats.max_path_error_mm.max(s.error);
                let expand = radius + cell_radius + s.error;
                let Some(r) = self.rect(
                    [s.a[0].min(s.b[0]) - expand, s.a[1].min(s.b[1]) - expand],
                    [s.a[0].max(s.b[0]) + expand, s.a[1].max(s.b[1]) + expand],
                ) else {
                    continue;
                };
                let dx = s.b[0] - s.a[0];
                let dy = s.b[1] - s.a[1];
                let length2 = dx * dx + dy * dy;
                for j in r.y0..=r.y1 {
                    for i in r.x0..=r.x1 {
                        if stats.visited_cells.is_multiple_of(1024) {
                            cancelled(cancel)?;
                        }
                        stats.visited_cells += 1;
                        if stats.visited_cells + stats.reach_cell_visits > options.max_cell_visits {
                            return Err("sweep cell-visit budget exceeded".into());
                        }
                        let p = self.cell_center(j * self.nx + i).unwrap();
                        let t = if length2 == 0. {
                            if s.a[2] < s.b[2] { 0. } else { 1. }
                        } else {
                            (((p[0] - s.a[0]) * dx + (p[1] - s.a[1]) * dy) / length2).clamp(0., 1.)
                        };
                        let d = (p[0] - (s.a[0] + t * dx)).hypot(p[1] - (s.a[1] + t * dy));
                        let near = (d - cell_radius - s.error).max(0.);
                        if near > touch {
                            continue;
                        }
                        let tip = s.a[2].min(s.b[2]) - s.error;
                        let under = match (&undercut, buried) {
                            (Some(u), true) => u.classify(
                                self.cells[j * self.nx + i],
                                tip,
                                d,
                                cell_radius,
                                s.error,
                            )?,
                            _ => false,
                        };
                        if near > radius {
                            continue;
                        }
                        if under {
                            stats.undercut_cells += 1;
                            if self.cells[j * self.nx + i].lower_mm
                                >= tip + tool.cutter().definition().cutting_length_mm
                            {
                                continue;
                            }
                        }
                        let h = &mut scratch[(j - rect.y0) * rect.width() + i - rect.x0];
                        let bottom = tool.cutter().tip_height_at_radius(near).unwrap();
                        h.lower_mm = h.lower_mm.min(
                            (s.a[2].min(s.b[2]) + bottom - s.error)
                                .clamp(self.spec.min_mm[2], self.spec.max_mm[2]),
                        );
                        let far = d + cell_radius + s.error;
                        if under {
                            continue;
                        }
                        if let Some(bottom) = tool.cutter().tip_height_at_radius(far) {
                            let z = s.a[2] + t * (s.b[2] - s.a[2]) + bottom + s.error;
                            h.upper_mm = h
                                .upper_mm
                                .min(z.clamp(self.spec.min_mm[2], self.spec.max_mm[2]));
                        }
                    }
                }
            }
            for j in rect.y0..=rect.y1 {
                cancelled(cancel)?;
                for i in rect.x0..=rect.x1 {
                    let index = j * self.nx + i;
                    let after = scratch[(j - rect.y0) * rect.width() + i - rect.x0];
                    let before = self.cells[index];
                    delta.record_change(index, before, after, options.max_changed_cells)?;
                }
            }
        }
        cancelled(cancel)?;
        stats.changed_cells = delta.changes.len();
        stats.delta_bytes = delta.changes.len() * std::mem::size_of::<CellChange>();
        delta.stats = stats;
        if fraction == 1. && !buried {
            delta.axial_column = AxialColumn::from_motion(tool, motion, self.spec.max_mm[2]);
            delta.departure = Some((motion.tool.clone(), motion.curve.end()));
        }
        // An undercut leaves material above the swept area: no cleared column.
        if analytic && fraction == 1. && !buried {
            // A stationary line has no supported XY sweep premise. It remains
            // a valid stock transaction, without adding geometric authority.
            delta.cleared_column = ClearedSweep::from_motion(motion, radius, options.tolerance_mm)
                .ok()
                .map(|sweep| CommittedColumn {
                    sweep,
                    tolerance_mm: options.tolerance_mm,
                });
        }
        Ok(delta)
    }

    /// Whether the whole footprint of cell `index` is cleared from height `z`
    /// up by this flat constant-height move (`axis`, `radius`, `error`)
    /// together with committed cleared passes of the same operation at or
    /// below `z`.
    ///
    /// A cell that no single move covers (two passes that each clear part of
    /// it) otherwise keeps its upper bound at the old height, which shows as
    /// thin "needles" of stock that is not there. The cell is split into
    /// sub-squares (a quadtree, down to [`UNION_DEPTH`] levels); each must lie
    /// entirely, with its half-diagonal and every path error, inside this
    /// move's swept disk or inside one committed column's cleared footprint
    /// (the same certificate as the engagement scene's `clear_box`). Any
    /// sub-square left over at the depth or work limit keeps the old bound:
    /// the result is never lower than what the swept disks prove.
    #[allow(clippy::too_many_arguments)]
    fn union_covered(
        &self,
        operation: &str,
        index: usize,
        axis: &crate::verification::AxisSpan,
        radius: f64,
        error: f64,
        z: f64,
        tolerance_mm: f64,
    ) -> bool {
        let first = self.committed_columns.own_from(operation);
        if self.heightfield_unproven || first == self.committed_columns.count {
            return false;
        }
        let Some(c) = self.cell_center(index) else {
            return false;
        };
        let half = [self.step[0] / 2., self.step[1] / 2.];
        let (lo, hi) = (
            [c[0] - half[0], c[1] - half[1]],
            [c[0] + half[0], c[1] + half[1]],
        );
        // Candidate witnesses: this operation's columns at or below `z`
        // whose footprint boxes meet the cell (one index walk per cell).
        let mut work = UNION_WORK;
        let mut candidates = Vec::new();
        if !self.committed_columns.overlapping(
            first,
            [lo, hi],
            z,
            tolerance_mm,
            &mut work,
            &mut candidates,
        ) || candidates.is_empty()
        {
            return false;
        }
        let inside = |p: Point, extent: f64| {
            axis.distance(p) + extent + error <= radius
                || candidates
                    .iter()
                    .any(|w| w.sweep.contains_ball(p, extent, z))
        };
        // Quick refusal: a corner or the centre outside every footprint
        // cannot be proven, whatever the split (a point is a zero ball).
        if [lo, hi, [lo[0], hi[1]], [hi[0], lo[1]], c]
            .into_iter()
            .any(|p| !inside(p, 0.))
        {
            return false;
        }
        // Depth-first quadtree: at most 3 pending siblings per level.
        let mut boxes = [([0.; 2], [0.; 2], 0u32); 3 * UNION_DEPTH as usize + 1];
        boxes[0] = (lo, hi, 0);
        let mut pending = 1;
        while pending > 0 {
            pending -= 1;
            let (lo, hi, depth) = boxes[pending];
            if work == 0 {
                return false;
            }
            work -= 1;
            let p: Point = [(lo[0] + hi[0]) / 2., (lo[1] + hi[1]) / 2.];
            let extent = (hi[0] - lo[0]).hypot(hi[1] - lo[1]) / 2.;
            if inside(p, extent) {
                continue;
            }
            // No single footprint holds this square: split it, so its parts
            // may each lie in a different one, down to the depth limit.
            if depth == UNION_DEPTH || !inside(p, 0.) {
                return false;
            }
            for child in [
                (lo, p),
                ([p[0], lo[1]], [hi[0], p[1]]),
                ([lo[0], p[1]], [p[0], hi[1]]),
                (p, hi),
            ] {
                boxes[pending] = (child.0, child.1, depth + 1);
                pending += 1;
            }
        }
        true
    }

    /// A height-view transaction for a tri-dexel run (`crate::dexel`): the
    /// cell changes are the projection of the dexel sweep's Z rays, and the
    /// native records are the ones this height field's own sweep would add
    /// for the same move, under the same condition (`records`: a fully swept
    /// cutting move that passed under no material, so the space above every
    /// pose of its cutting part is empty). `undercut_cells` > 0 marks a move
    /// whose cutting part ran under material, as `preview_sweep` does.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn derived_delta(
        &self,
        tool: &PreparedAssembly,
        motion: &Motion,
        fraction: f64,
        changes: Vec<CellChange>,
        records: bool,
        analytic: bool,
        undercut_cells: u64,
        tolerance_mm: f64,
    ) -> StockDelta {
        let stats = SweepStats {
            changed_cells: changes.len(),
            delta_bytes: changes.len() * std::mem::size_of::<CellChange>(),
            undercut_cells,
            ..Default::default()
        };
        let mut delta = StockDelta {
            spec: self.spec.clone(),
            base_generation: self.generation,
            motion_id: motion.id,
            operation: motion.operation.clone(),
            fraction,
            changes,
            stats,
            native_identity: Some(self.identity),
            cleared_column: None,
            axial_column: None,
            departure: None,
            shift: motion.kind == MoveKind::Shift,
            receipt: self.receipt_requested.load(Ordering::Relaxed).then(|| {
                // The dexel kernel swept exactly this motion prefix with this
                // tool from this view's stock state.
                Arc::new(SweepReceipt {
                    base_identity: self.identity,
                    motion: motion.clone(),
                    tool: tool.definition().clone(),
                    fraction,
                    tolerance_mm,
                })
            }),
        };
        if records && fraction == 1. && !motion.kind.is_rapid() {
            delta.axial_column = AxialColumn::from_motion(tool, motion, self.spec.max_mm[2]);
            delta.departure = Some((motion.tool.clone(), motion.curve.end()));
            if analytic {
                let radius = tool.cutter().definition().diameter_mm / 2.;
                delta.cleared_column = ClearedSweep::from_motion(motion, radius, tolerance_mm)
                    .ok()
                    .map(|sweep| CommittedColumn {
                        sweep,
                        tolerance_mm,
                    });
            }
        }
        delta
    }

    /// A height-only transaction (no native records) for received tri-dexel
    /// changes: the projection of their Z rays.
    pub(crate) fn wire_delta(
        &self,
        motion: &Motion,
        fraction: f64,
        changes: Vec<CellChange>,
    ) -> StockDelta {
        StockDelta {
            spec: self.spec.clone(),
            base_generation: self.generation,
            motion_id: motion.id,
            operation: motion.operation.clone(),
            fraction,
            stats: SweepStats {
                changed_cells: changes.len(),
                ..Default::default()
            },
            changes,
            native_identity: None,
            cleared_column: None,
            axial_column: None,
            departure: None,
            shift: false,
            receipt: None,
        }
    }

    pub fn apply_delta(&mut self, delta: &StockDelta) -> Result<(), String> {
        self.commit(delta, false)
    }
    pub fn undo_delta(&mut self, delta: &StockDelta) -> Result<(), String> {
        self.commit(delta, true)
    }
    fn commit(&mut self, delta: &StockDelta, reverse: bool) -> Result<(), String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("stock.commit");
        let next = delta
            .base_generation
            .checked_add(1)
            .ok_or("stock generation overflow")?;
        let expected = if reverse { next } else { delta.base_generation };
        if self.spec != delta.spec || self.generation != expected {
            return Err("delta stock specification/generation mismatch".into());
        }
        let mut last = None;
        for c in &delta.changes {
            let index = c.index as usize;
            if last.is_some_and(|i| index <= i)
                || index >= self.cells.len()
                || !height_valid(c.before, &self.spec)
                || !height_valid(c.after, &self.spec)
                || c.after.lower_mm > c.before.lower_mm
                || c.after.upper_mm > c.before.upper_mm
                || self.cells[index] != if reverse { c.after } else { c.before }
            {
                return Err(
                    "delta invalid index/order, old state or non-monotone height bounds".into(),
                );
            }
            last = Some(index);
        }
        // Validation is complete before the first mutation.
        if !reverse && delta.native_identity == Some(self.identity) {
            self.committed_columns.enter(&delta.operation);
            if let Some(column) = &delta.cleared_column {
                self.committed_columns.append(column.clone());
            }
            if delta.stats.undercut_cells > 0 {
                self.heightfield_unproven = true;
            }
            self.bore = if delta.shift {
                match (&self.bore, &self.departure) {
                    (Some(b), _) => Some(b.clone()),
                    (None, d) => d.clone(),
                }
            } else {
                None
            };
            self.departure = delta.departure.clone();
            if let Some(a) = &delta.axial_column
                && self.axial_columns.len() < MAX_AXIAL_COLUMNS
            {
                let z = self
                    .axial_columns
                    .entry((a.tool.clone(), a.xy.map(f64::to_bits)))
                    .or_insert(a.tip_z);
                *z = z.min(a.tip_z);
            }
        } else {
            // Undo can restore material. Wire/foreign transactions cannot
            // preserve the native prefix binding, even if their heights agree.
            self.committed_columns = CommittedColumns::default();
            self.axial_columns.clear();
            self.departure = None;
            self.bore = None;
            if !reverse {
                self.heightfield_unproven = true;
            }
        }
        for c in &delta.changes {
            self.cells[c.index as usize] = if reverse { c.before } else { c.after };
        }
        if let Some(index) = &mut self.contact_index {
            index.update(&self.cells, self.nx, &delta.changes);
        }
        if !delta.changes.is_empty() {
            self.initial_full_stock = false;
        }
        self.last_receipt = if reverse {
            None
        } else {
            delta
                .receipt
                .as_ref()
                .filter(|r| r.base_identity == self.identity)
                .cloned()
        };
        self.identity = new_identity();
        self.generation = if reverse { delta.base_generation } else { next };
        Ok(())
    }
    /// A plasma or laser beam cuts through the whole sheet wherever its kerf
    /// passes, whatever the nozzle height: every cell entirely inside the
    /// kerf is cut through; one it only partly covers may be cut somewhere
    /// (lower bound to the stock bottom, upper bound kept).
    fn beam_sweep(
        &self,
        delta: &mut StockDelta,
        tool: &PreparedAssembly,
        motion: &Motion,
        fraction: f64,
        options: SweepOptions,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        let r = tool.cutter().definition().diameter_mm / 2.;
        let distance = beam_distance(motion, fraction)?;
        let bounds = motion.curve.prepare(options.tolerance_mm)?.bounds();
        let cell_radius = self.step[0].hypot(self.step[1]) / 2.;
        let pad = r + cell_radius;
        let Some(rect) = self.rect(
            [bounds.min[0] - pad, bounds.min[1] - pad],
            [bounds.max[0] + pad, bounds.max[1] + pad],
        ) else {
            return Ok(());
        };
        if rect.count() > options.max_scratch_cells {
            return Err("beam sweep rectangle budget exceeded".into());
        }
        let bottom = self.spec.min_mm[2];
        let guard = options.tolerance_mm;
        for j in rect.y0..=rect.y1 {
            cancelled(cancel)?;
            for i in rect.x0..=rect.x1 {
                let index = j * self.nx + i;
                let p = self.cell_center(index).unwrap();
                let d = distance(p);
                let before = self.cells[index];
                let mut after = before;
                if d + cell_radius + guard <= r {
                    after = HeightBounds {
                        lower_mm: bottom,
                        upper_mm: bottom,
                    };
                } else if d - cell_radius <= r + guard {
                    after.lower_mm = bottom;
                }
                if after != before {
                    delta.record_change(index, before, after, options.max_changed_cells)?;
                }
            }
        }
        delta.stats.changed_cells = delta.changes.len();
        delta.stats.delta_bytes = delta.changes.len() * std::mem::size_of::<CellChange>();
        Ok(())
    }
    /// A beam's marking move (`MoveKind::Mark`) removes nothing, and the
    /// sheet under it must still be whole: the beam's width (the tool's
    /// kerf) stays on the sheet, over cells no beam has cut through or may
    /// have (a mark is made before the cuts around it, on sheet that has
    /// not moved; nothing is marked across a kerf or beyond the sheet edge).
    fn mark_on_sheet(
        &self,
        tool: &PreparedAssembly,
        motion: &Motion,
        fraction: f64,
        options: SweepOptions,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        if !matches!(tool.cutter().definition().shape, CutterShape::Beam) {
            return Err(
                "a marking move needs a beam tool (camlib.tools.Torch or LaserHead)".into(),
            );
        }
        let refuse = |p: [f64; 2], why: &str| {
            format!(
                "marking move {} of operation {} {why} near ({:.3}, {:.3}): a mark is made on whole sheet, before the cuts around it (run the marking operation before the cutting operation, and keep its paths on the sheet)",
                motion.id, motion.operation, p[0], p[1]
            )
        };
        let r = tool.cutter().definition().diameter_mm / 2.;
        let distance = beam_distance(motion, fraction)?;
        let bounds = motion.curve.prepare(options.tolerance_mm)?.bounds();
        let guard = options.tolerance_mm;
        for k in 0..2 {
            if bounds.min[k] - r < self.spec.min_mm[k] - guard
                || bounds.max[k] + r > self.spec.max_mm[k] + guard
            {
                let s = motion.curve.start();
                return Err(refuse([s[0], s[1]], "leaves the sheet"));
            }
        }
        let cell_radius = self.step[0].hypot(self.step[1]) / 2.;
        let pad = r + cell_radius;
        let Some(rect) = self.rect(
            [bounds.min[0] - pad, bounds.min[1] - pad],
            [bounds.max[0] + pad, bounds.max[1] + pad],
        ) else {
            return Ok(());
        };
        if rect.count() > options.max_scratch_cells {
            return Err("marking move rectangle budget exceeded".into());
        }
        let bottom = self.spec.min_mm[2];
        for j in rect.y0..=rect.y1 {
            cancelled(cancel)?;
            for i in rect.x0..=rect.x1 {
                let index = j * self.nx + i;
                let p = self.cell_center(index).unwrap();
                if distance(p) - cell_radius <= r + guard && self.cells[index].lower_mm <= bottom {
                    return Err(refuse(p, "passes over sheet that is or may be cut through"));
                }
            }
        }
        Ok(())
    }
    fn rect(&self, min: [f64; 2], max: [f64; 2]) -> Option<Rect> {
        if (0..2).any(|k| max[k] < self.spec.min_mm[k] || min[k] > self.spec.max_mm[k]) {
            return None;
        }
        let dims = [self.nx, self.ny];
        let index = |p: [f64; 2]| -> [usize; 2] {
            std::array::from_fn(|k| {
                ((p[k] - self.spec.min_mm[k]) / self.step[k])
                    .floor()
                    .max(0.)
                    .min((dims[k] - 1) as f64) as usize
            })
        };
        let [x0, y0] = index(min);
        let [x1, y1] = index(max);
        Some(Rect { x0, x1, y0, y1 })
    }
}

/// The XY distance from a point to the first `fraction` of a beam move's
/// path (a line or an XY arc).
type PathDistance = Box<dyn Fn([f64; 2]) -> f64>;
fn beam_distance(motion: &Motion, fraction: f64) -> Result<PathDistance, String> {
    let start = motion.curve.start();
    Ok(match motion.curve {
        Curve::Line { end, .. } => {
            let b = [
                start[0] + fraction * (end[0] - start[0]),
                start[1] + fraction * (end[1] - start[1]),
            ];
            let a = [start[0], start[1]];
            {
                Box::new(move |p: [f64; 2]| {
                    let d = [b[0] - a[0], b[1] - a[1]];
                    let l = d[0] * d[0] + d[1] * d[1];
                    let t = if l > 0. {
                        (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / l).clamp(0., 1.)
                    } else {
                        0.
                    };
                    (p[0] - a[0] - t * d[0]).hypot(p[1] - a[1] - t * d[1])
                })
            }
        }
        Curve::Arc {
            center,
            sweep_rad,
            plane: Plane::Xy,
            ..
        }
        | Curve::Helix {
            center,
            sweep_rad,
            plane: Plane::Xy,
            ..
        } => {
            let c = [center[0], center[1]];
            let radius = (start[0] - c[0]).hypot(start[1] - c[1]);
            let a0 = (start[1] - c[1]).atan2(start[0] - c[0]);
            let sweep = sweep_rad * fraction;
            let end = [
                c[0] + radius * (a0 + sweep).cos(),
                c[1] + radius * (a0 + sweep).sin(),
            ];
            let a = [start[0], start[1]];
            {
                Box::new(move |p: [f64; 2]| {
                    let angle = (p[1] - c[1]).atan2(p[0] - c[0]) - a0;
                    let within = if sweep >= 0. {
                        angle.rem_euclid(std::f64::consts::TAU) <= sweep
                    } else {
                        (-angle).rem_euclid(std::f64::consts::TAU) <= -sweep
                    };
                    if within || sweep.abs() >= std::f64::consts::TAU {
                        ((p[0] - c[0]).hypot(p[1] - c[1]) - radius).abs()
                    } else {
                        (p[0] - a[0])
                            .hypot(p[1] - a[1])
                            .min((p[0] - end[0]).hypot(p[1] - end[1]))
                    }
                })
            }
        }
        _ => return Err("a beam cuts with straight moves and XY arcs only".into()),
    })
}
fn height_valid(h: HeightBounds, spec: &StockSpec) -> bool {
    h.lower_mm.is_finite()
        && h.upper_mm.is_finite()
        && h.lower_mm >= spec.min_mm[2]
        && h.upper_mm <= spec.max_mm[2]
        && h.lower_mm <= h.upper_mm
}
pub(crate) fn cancelled(cancel: &impl CancelSignal) -> Result<(), String> {
    if cancel.is_cancelled() {
        Err("stock sweep cancelled; no changes committed".into())
    } else {
        Ok(())
    }
}
struct Rect {
    x0: usize,
    x1: usize,
    y0: usize,
    y1: usize,
}
impl Rect {
    fn width(&self) -> usize {
        self.x1 - self.x0 + 1
    }
    fn count(&self) -> usize {
        self.width() * (self.y1 - self.y0 + 1)
    }
}
pub(crate) struct Segment {
    pub(crate) a: Vec3,
    pub(crate) b: Vec3,
    pub(crate) error: f64,
}
fn segment_layout(curve: &Curve, options: SweepOptions) -> Result<(usize, f64), String> {
    let layout = match *curve {
        Curve::Line { start, end } => (
            ((end[2] - start[2]).abs() / options.max_tip_step_mm)
                .ceil()
                .max(1.) as usize,
            0.,
        ),
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
            let [u, v, w] = plane.axes();
            let radius = (start[u] - center[u]).hypot(start[v] - center[v]);
            let max_angle = (4.
                * (options.chord_error_mm / (2. * radius))
                    .min(0.5)
                    .sqrt()
                    .asin())
            .min(std::f64::consts::FRAC_PI_2);
            let z_travel = if w == 2 {
                (end[2] - start[2]).abs()
            } else {
                radius * sweep_rad.abs()
            };
            let n = (sweep_rad.abs() / max_angle)
                .ceil()
                .max((z_travel / options.max_tip_step_mm).ceil())
                .max(1.) as usize;
            (n, options.chord_error_mm + options.tolerance_mm)
        }
    };
    if layout.0 > options.max_segments {
        return Err("sweep segment budget exceeded".into());
    }
    Ok(layout)
}
pub(crate) fn segments(
    curve: &Curve,
    prepared: &crate::motion::PreparedCurve,
    fraction: f64,
    options: SweepOptions,
    guard: f64,
    cancel: &impl CancelSignal,
) -> Result<Vec<Segment>, String> {
    let (count, arc_error) = segment_layout(curve, options)?;
    let mut output = Vec::with_capacity((count as f64 * fraction).ceil() as usize);
    for i in 0..count {
        if i.is_multiple_of(1024) {
            cancelled(cancel)?;
        }
        let from = i as f64 / count as f64;
        if from >= fraction {
            break;
        }
        let to = ((i + 1) as f64 / count as f64).min(fraction);
        output.push(Segment {
            a: prepared.position(from)?,
            b: prepared.position(to)?,
            error: arc_error + guard,
        });
    }
    Ok(output)
}

#[cfg(test)]
mod undercut_diagnostic_tests {
    use super::*;

    #[test]
    fn neck_diagnostic_distinguishes_grid_uncertainty_from_center_obstruction() {
        let neck = Undercut {
            cutting: 0.5,
            shoulder: 10.,
            neck: 1.,
            guard: 1e-5,
        };
        let material = HeightBounds {
            lower_mm: 5.,
            upper_mm: 5.,
        };
        let radius = 0.26 / std::f64::consts::SQRT_2;
        let error = neck.classify(material, 0., 1.1, radius, 0.002).unwrap_err();
        assert!(
            error.contains("clearance lower bound -0.085847763 mm"),
            "{error}"
        );
        assert!(
            error.contains("cell_size_mm must be < 0.138592929 mm"),
            "{error}"
        );
        assert!(error.contains("new centers and stock bounds may differ"));
        assert!(
            neck.classify(material, 0., 1.1, 0.05 / std::f64::consts::SQRT_2, 0.002)
                .unwrap()
        );
        for distance in [1., 0.9] {
            let error = neck
                .classify(material, 0., distance, radius, 0.)
                .unwrap_err();
            assert!(error.contains("no positive cell-size threshold"), "{error}");
            assert!(!error.contains("cell_size_mm must"));
        }
        // Ordinary removal and material above the shoulder retain their rules.
        assert!(!neck.classify(material, 5., 0., radius, 0.).unwrap());
        assert_eq!(
            neck.classify(material, -10., 5., radius, 0.).unwrap_err(),
            REACH
        );
    }
}

#[cfg(test)]
pub(crate) mod front_coverage_tests {
    use super::*;
    use crate::engagement::EngagementScene;

    #[test]
    fn repeated_path_reach_requires_native_history_and_exact_footprint_containment() {
        let (mut stock, mut tool, motion) = native_fixture();
        let options = SweepOptions::default();
        let delta = stock
            .preview_sweep(&tool.prepare().unwrap(), &motion, 1., options, &|| false)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        tool.cutter.cutting_length_mm = 2.;
        let mut deeper = motion.clone();
        deeper.id += 1;
        deeper.curve = Curve::Line {
            start: [8., 13., 4.],
            end: [18., 13., 4.],
        };
        let check = |s: &Stock, m: &Motion, r, h| {
            s.history_clearance(m, r, h, &mut 1000, &|| false).unwrap()
        };
        assert!(check(&stock, &deeper, 2., 2.));
        assert!(!check(&stock, &deeper, 2.001, 2.));
        assert!(!check(&stock, &deeper, 2., 0.));
        assert!(
            !stock
                .history_clearance(&deeper, 2., 2., &mut 0, &|| false)
                .unwrap()
        );
        assert!(
            stock
                .history_clearance(&deeper, 2., 2., &mut 1000, &|| true)
                .is_err()
        );
        let shifted = Motion {
            curve: Curve::Line {
                start: [8., 13.001, 4.],
                end: [18., 13.001, 4.],
            },
            ..deeper.clone()
        };
        assert!(!check(&stock, &shifted, 2., 2.));
        assert!(
            stock
                .preview_sweep(&tool.prepare().unwrap(), &deeper, 1., options, &|| false)
                .is_ok()
        );
        // Wire transactions cannot create native empty-column authority.
        let (mut decoded_stock, _, _) = native_fixture();
        let decoded: StockDelta =
            serde_json::from_slice(&serde_json::to_vec(&delta).unwrap()).unwrap();
        decoded_stock.apply_delta(&decoded).unwrap();
        assert!(!check(&decoded_stock, &deeper, 2., 2.));
    }

    fn native_fixture() -> (Stock, crate::tool::ToolAssembly, Motion) {
        let manifest: crate::stream::StreamManifest =
            serde_json::from_str(include_str!("../fixtures/operations/adaptive-pocket.json"))
                .unwrap();
        let tool = manifest.tools[0].clone();
        let motion = Motion {
            id: 1,
            operation: "upstream".into(),
            tool: tool.id.clone(),
            kind: MoveKind::Cut,
            feed_mm_min: Some(600.),
            curve: Curve::Line {
                start: [8., 13., 5.],
                end: [18., 13., 5.],
            },
        };
        (
            Stock::new(manifest.stock.height().unwrap().clone()).unwrap(),
            tool,
            motion,
        )
    }

    /// Apply flat cutting lines (x0, x1, y, z) of the fixture's 4 mm end mill
    /// in order; returns the stock and the sum of `union_cells`.
    pub(crate) fn passes(lines: &[(f64, f64, f64, f64)]) -> (Stock, u64) {
        let (mut stock, tool, motion) = native_fixture();
        let tool = tool.prepare().unwrap();
        let mut union = 0;
        for (n, &(x0, x1, y, z)) in lines.iter().enumerate() {
            let mut m = motion.clone();
            m.id = n as u64 + 1;
            m.curve = Curve::Line {
                start: [x0, y, z],
                end: [x1, y, z],
            };
            let delta = stock
                .preview_sweep(&tool, &m, 1., SweepOptions::default(), &|| false)
                .unwrap();
            union += delta.stats().union_cells;
            stock.apply_delta(&delta).unwrap();
        }
        (stock, union)
    }
    #[test]
    fn restored_columns_lower_the_same_cells_as_an_uninterrupted_run() {
        // Two passes, the state kept and put back onto a stock rebuilt from
        // the cells alone, then a third pass: the same cells as when the
        // run never stopped. Without the columns the rebuilt stock proves
        // nothing by union.
        let lines = [(4., 22., 13., 5.), (4., 22., 16.8, 5.), (4., 22., 20.6, 5.)];
        let (whole, lowered) = passes(&lines);
        assert!(lowered > 0);
        let (first, _) = passes(&lines[..2]);
        let third = |mut stock: Stock| {
            let (_, tool, motion) = native_fixture();
            let tool = tool.prepare().unwrap();
            let mut m = motion.clone();
            let (x0, x1, y, z) = lines[2];
            m.curve = Curve::Line {
                start: [x0, y, z],
                end: [x1, y, z],
            };
            let d = stock
                .preview_sweep(&tool, &m, 1., SweepOptions::default(), &|| false)
                .unwrap();
            stock.apply_delta(&d).unwrap();
            stock
        };
        let rebuilt = || {
            Stock::from_snapshot(StockSnapshot {
                spec: first.spec().clone(),
                generation: first.generation(),
                cells: first.cells().to_vec(),
            })
            .unwrap()
        };
        let mut resumed = rebuilt();
        resumed.restore_clearance(first.clearance_state()).unwrap();
        assert_eq!(third(resumed).cells(), whole.cells());
        let bare = third(rebuilt());
        assert_ne!(bare.cells(), whole.cells());
        for (a, b) in bare.cells().iter().zip(whole.cells()) {
            assert!(a.upper_mm >= b.upper_mm && a.lower_mm == b.lower_mm);
        }
        // A state that does not describe a history is refused.
        let mut bad = first.clearance_state();
        bad.union_from = bad.columns.len() + 1;
        assert!(rebuilt().restore_clearance(bad).is_err());
        let mut bad = first.clearance_state();
        bad.columns[0].0 = 1.;
        assert!(rebuilt().restore_clearance(bad).is_err());
        let mut bad = first.clearance_state();
        bad.operation = Some("not an id".into());
        assert!(rebuilt().restore_clearance(bad).is_err());
    }

    fn upper_at(stock: &Stock, p: [f64; 2]) -> f64 {
        let [nx, _] = stock.dimensions();
        let [w, h] = stock.cell_widths_mm();
        let index = (p[1] / h) as usize * nx + (p[0] / w) as usize;
        stock.cells()[index].upper_mm
    }

    #[test]
    fn overlapping_passes_lower_cells_no_single_pass_covers() {
        // Two passes 3.8 mm apart with a 4 mm cutter overlap by 0.2 mm, less
        // than a 0.4 mm cell. The cells across the seam (y 14.8..15) are each
        // only partly cut by either pass: before the union rule they kept the
        // stock top (10) and showed as needles.
        let (stock, union) = passes(&[(4., 22., 13., 5.), (4., 22., 16.8, 5.)]);
        assert!(union > 0, "seam cells must need the union rule");
        let [nx, _] = stock.dimensions();
        let r = 2.;
        let mut dropped = 0;
        for (i, c) in stock.cells().iter().enumerate() {
            let p = stock.cell_center(i).unwrap();
            if (8.0..=18.0).contains(&p[0]) && (13.0..=16.8).contains(&p[1]) {
                assert!(c.upper_mm < 5.01, "cell {i} at {p:?} kept {}", c.upper_mm);
            }
            // Independent check: any cell at the cut height lies, sampled
            // on a 0.01 mm grid, entirely inside one of the two swept disks.
            if c.upper_mm < 5.01 {
                dropped += 1;
                let [w, h] = stock.cell_widths_mm();
                let lo = [p[0] - w / 2., p[1] - h / 2.];
                for a in 0..=40 {
                    for b in 0..=40 {
                        let q = [lo[0] + w * a as f64 / 40., lo[1] + h * b as f64 / 40.];
                        let inside = [13., 16.8].iter().any(|&y| {
                            let x = q[0].clamp(4., 22.);
                            (q[0] - x).hypot(q[1] - y) <= r
                        });
                        assert!(inside, "cell {i} lowered but {q:?} is uncut");
                    }
                }
            }
        }
        assert!(dropped > 0 && nx > 0);
    }

    #[test]
    fn hairline_between_passes_keeps_its_stock() {
        // 4.04 mm apart: a 0.04 mm wall of real material stands between the
        // two swept disks. The cells holding it must keep the stock top.
        let (stock, _) = passes(&[(4., 22., 13., 5.), (4., 22., 17.04, 5.)]);
        for x in [9., 12.3, 15.7] {
            assert_eq!(upper_at(&stock, [x, 15.02]), 10., "x {x}");
        }
        // Cells fully inside either pass still drop.
        assert!(upper_at(&stock, [12., 13.]) < 5.01);
        assert!(upper_at(&stock, [12., 17.]) < 5.01);
    }

    #[test]
    fn union_uses_only_passes_at_or_below_the_new_height() {
        // A higher pass (z 7) cannot prove a cell cleared down to 5.
        let (stock, _) = passes(&[(4., 22., 13., 7.), (4., 22., 16.8, 5.)]);
        let seam = upper_at(&stock, [12., 15.]);
        assert!(seam > 5.01, "seam lowered to {seam} by a pass at z 7");
        // A deeper pass proves the seam cleared down to the higher one (7).
        let (stock, union) = passes(&[(4., 22., 13., 5.), (4., 22., 16.8, 7.)]);
        assert!(union > 0);
        let seam = upper_at(&stock, [12., 15.]);
        assert!(seam > 6.99 && seam < 7.01, "seam {seam}");
    }

    #[test]
    fn union_combines_only_the_same_operations_passes() {
        // The same two passes, the second in another operation: its stock
        // change must not depend on the first operation's history.
        let (mut stock, tool, motion) = native_fixture();
        let tool = tool.prepare().unwrap();
        let options = SweepOptions::default();
        let mut a = motion.clone();
        a.curve = Curve::Line {
            start: [4., 13., 5.],
            end: [22., 13., 5.],
        };
        let first = stock
            .preview_sweep(&tool, &a, 1., options, &|| false)
            .unwrap();
        stock.apply_delta(&first).unwrap();
        let mut b = motion.clone();
        b.id = 2;
        b.operation = "next".into();
        b.curve = Curve::Line {
            start: [4., 16.8, 5.],
            end: [22., 16.8, 5.],
        };
        let other = stock
            .preview_sweep(&tool, &b, 1., options, &|| false)
            .unwrap();
        assert_eq!(other.stats().union_cells, 0);
        b.operation = a.operation.clone();
        let same = stock
            .preview_sweep(&tool, &b, 1., options, &|| false)
            .unwrap();
        assert!(same.stats().union_cells > 0);
    }

    #[test]
    fn union_needs_native_history() {
        // Undo drops the committed passes: a later pass cannot use them.
        let (mut stock, tool, motion) = native_fixture();
        let tool = tool.prepare().unwrap();
        let options = SweepOptions::default();
        let mut a = motion.clone();
        a.curve = Curve::Line {
            start: [4., 13., 5.],
            end: [22., 13., 5.],
        };
        let first = stock
            .preview_sweep(&tool, &a, 1., options, &|| false)
            .unwrap();
        stock.apply_delta(&first).unwrap();
        let mut probe = stock.clone();
        probe.undo_delta(&first).unwrap();
        probe.apply_delta(&first).unwrap();
        let mut b = motion.clone();
        b.id = 2;
        b.curve = Curve::Line {
            start: [4., 16.8, 5.],
            end: [22., 16.8, 5.],
        };
        let native = stock
            .preview_sweep(&tool, &b, 1., options, &|| false)
            .unwrap();
        assert!(native.stats().union_cells > 0);
        let replayed = probe
            .preview_sweep(&tool, &b, 1., options, &|| false)
            .unwrap();
        assert_eq!(replayed.stats().union_cells, 0);
    }

    fn known_box(stock: &Stock, bounds: [[f64; 2]; 2], floor: f64) -> bool {
        EngagementScene::new(
            stock.spec().clone(),
            stock.committed_clearance(1e-5, &|| false).unwrap(),
        )
        .unwrap()
        .clear_box(bounds, floor, &mut 10000, &|| false)
        .unwrap()
    }

    #[test]
    fn native_columns_require_commits_and_keep_clone_wire_undo_and_floor_isolation() {
        let (mut stock, tool, motion) = native_fixture();
        let initial = stock.snapshot();
        let cut = stock
            .preview_sweep(
                &tool.prepare().unwrap(),
                &motion,
                1.,
                SweepOptions::default(),
                &|| false,
            )
            .unwrap();
        let bounds = [[12.5, 12.5], [13.5, 13.5]];
        assert!(!known_box(&stock, bounds, 5.), "preview is not a commit");
        let mut branch = stock.clone();
        let mut foreign = Stock::new(stock.spec().clone()).unwrap();
        foreign.apply_delta(&cut).unwrap();
        assert!(
            !known_box(&foreign, bounds, 5.),
            "matching heights/generation are not native identity"
        );
        stock.apply_delta(&cut).unwrap();
        branch.apply_delta(&cut).unwrap();
        assert!(known_box(&stock, bounds, 5.));
        assert!(known_box(&branch, bounds, 5.));
        assert!(!known_box(&stock, bounds, 4.999));
        assert_eq!(stock.snapshot(), foreign.snapshot());
        assert!(
            !stock
                .bounds_clear(bounds, 5., &mut 1000, &|| false)
                .unwrap(),
            "native support does not relax the height upper's guard"
        );
        assert!(
            stock
                .committed_clearance(1e-6, &|| false)
                .unwrap()
                .is_empty()
        );
        assert!(stock.committed_clearance(1e-5, &|| true).is_err());
        let mut fork = stock.clone();
        assert!(Arc::ptr_eq(
            &stock.committed_columns.roots[0],
            &fork.committed_columns.roots[0]
        ));
        let mut lower = motion.clone();
        lower.id = 2;
        lower.curve = Curve::Line {
            start: [8., 20., 4.],
            end: [18., 20., 4.],
        };
        let delta = fork
            .preview_sweep(
                &tool.prepare().unwrap(),
                &lower,
                1.,
                SweepOptions::default(),
                &|| false,
            )
            .unwrap();
        fork.apply_delta(&delta).unwrap();
        assert!(known_box(&fork, [[12.5, 19.5], [13.5, 20.5]], 4.));
        assert!(!known_box(&stock, [[12.5, 19.5], [13.5, 20.5]], 4.));
        let before = stock.snapshot();
        assert!(stock.apply_delta(&cut).is_err());
        assert_eq!(stock.snapshot(), before);
        assert!(
            known_box(&stock, bounds, 5.),
            "rejected delta cannot invalidate authority"
        );
        let restored = Stock::from_snapshot(stock.snapshot()).unwrap();
        assert!(!known_box(&restored, bounds, 5.));
        branch.undo_delta(&cut).unwrap();
        assert_eq!(branch.snapshot(), initial);
        assert!(!known_box(&branch, bounds, 5.));

        let mut rapid = motion.clone();
        rapid.kind = MoveKind::Rapid;
        rapid.feed_mm_min = None;
        rapid.curve = Curve::Line {
            start: [0., 0., 12.],
            end: [1., 0., 12.],
        };
        let delta = stock
            .preview_sweep(
                &tool.prepare().unwrap(),
                &rapid,
                1.,
                SweepOptions::default(),
                &|| false,
            )
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        assert!(
            known_box(&stock, bounds, 5.),
            "a genuine native rapid preserves prior columns"
        );
        let delta = stock
            .preview_sweep(
                &tool.prepare().unwrap(),
                &rapid,
                1.,
                SweepOptions::default(),
                &|| false,
            )
            .unwrap();
        let wire = serde_json::to_string(&delta).unwrap();
        assert!(!wire.contains("native_identity") && !wire.contains("cleared_column"));
        let decoded: StockDelta = serde_json::from_str(&wire).unwrap();
        stock.apply_delta(&decoded).unwrap();
        assert!(
            !known_box(&stock, bounds, 5.),
            "wire replay drops geometric authority"
        );
    }

    #[test]
    fn native_columns_keep_actual_radius_and_refuse_partial_or_nonflat_profiles() {
        let (mut stock, mut tool, mut motion) = native_fixture();
        tool.cutter.diameter_mm = 8.;
        tool.cutter.shank_diameter_mm = 8.;
        let prepared = tool.prepare().unwrap();
        let delta = stock
            .preview_sweep(&prepared, &motion, 1., SweepOptions::default(), &|| false)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        assert!(
            known_box(&stock, [[12.9, 16.1], [13.1, 16.3]], 5.),
            "retain actual D8 radius, not next cutter's D4"
        );
        motion.curve = Curve::Line {
            start: [8., 22., 5.],
            end: [18., 22., 5.],
        };
        let delta = stock
            .preview_sweep(&prepared, &motion, 0.5, SweepOptions::default(), &|| false)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        assert!(!known_box(&stock, [[14., 22.], [15., 22.1]], 5.));
        tool.cutter.shape = CutterShape::Ball;
        let delta = stock
            .preview_sweep(
                &tool.prepare().unwrap(),
                &motion,
                1.,
                SweepOptions::default(),
                &|| false,
            )
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        assert!(!known_box(&stock, [[14., 22.], [15., 22.1]], 5.));
        motion.curve = Curve::Line {
            start: [8., 22., 6.],
            end: [18., 22., 5.],
        };
        let delta = stock
            .preview_sweep(&prepared, &motion, 1., SweepOptions::default(), &|| false)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        assert!(!known_box(&stock, [[14., 22.], [15., 22.1]], 5.));
        assert!(
            known_box(&stock, [[12.9, 16.1], [13.1, 16.3]], 5.),
            "native removals preserve an older valid column"
        );
    }

    #[test]
    fn verified_replay_recovers_whole_line_and_arc_columns_without_height_sweeps() {
        let (base, tool, line) = native_fixture();
        let arc = Motion {
            curve: Curve::Arc {
                start: [16., 13., 5.],
                end: [16., 13., 5.],
                center: [13., 13., 5.],
                plane: Plane::Xy,
                sweep_rad: std::f64::consts::TAU,
            },
            ..line.clone()
        };
        for motion in [line.clone(), arc] {
            let mut cold = Stock::new(base.spec().clone()).unwrap();
            let native = cold
                .preview_sweep(
                    &tool.prepare().unwrap(),
                    &motion,
                    1.,
                    SweepOptions::default(),
                    &|| false,
                )
                .unwrap();
            let raw = serde_json::to_string(&native).unwrap();
            let mut cached: StockDelta = serde_json::from_str(&raw).unwrap();
            let mut replay = Stock::new(base.spec().clone()).unwrap();
            let checked = replay
                .certify_verified_replay(
                    &mut cached,
                    &tool.prepare().unwrap(),
                    &motion,
                    SweepOptions::default(),
                    &|| false,
                )
                .unwrap();
            assert_eq!(checked.columns, 1);
            assert_eq!(
                checked.reach_cell_visits, 0,
                "whole original-stock axial reach is constant work"
            );
            let mut invalid = cached.clone();
            invalid.changes[0].before.lower_mm = 9.;
            let before = replay.snapshot();
            let key = replay.committed_clearance_fingerprint();
            assert!(replay.apply_delta(&invalid).is_err());
            assert_eq!(replay.snapshot(), before);
            assert_eq!(
                replay.committed_clearance_fingerprint(),
                key,
                "a certificate cannot commit an invalid cached height delta"
            );
            cold.apply_delta(&native).unwrap();
            replay.apply_delta(&cached).unwrap();
            assert_eq!(cold.snapshot(), replay.snapshot());
            assert_eq!(
                cold.committed_clearance_fingerprint(),
                replay.committed_clearance_fingerprint()
            );
            let mut wire = Stock::new(base.spec().clone()).unwrap();
            let decoded: StockDelta = serde_json::from_str(&raw).unwrap();
            wire.apply_delta(&decoded).unwrap();
            assert!(
                wire.committed_clearance(1e-5, &|| false)
                    .unwrap()
                    .is_empty(),
                "plain decoded data cannot obtain the private certificate"
            );
            assert!(
                cached.receipt.is_none(),
                "cached geometry is not an original preview receipt"
            );
            if matches!(motion.curve, Curve::Arc { .. }) {
                assert!(
                    !known_box(&replay, [[12.9, 12.9], [13.1, 13.1]], 5.),
                    "annular hole survives replay"
                );
                assert!(!known_box(&replay, [[15.9, 12.9], [16.1, 13.1]], 4.99));
            }
            let mut rapid = motion.clone();
            rapid.id += 1;
            rapid.kind = MoveKind::Rapid;
            rapid.feed_mm_min = None;
            rapid.curve = Curve::Line {
                start: [0., 0., 12.],
                end: [1., 0., 12.],
            };
            let d = cold
                .preview_sweep(
                    &tool.prepare().unwrap(),
                    &rapid,
                    1.,
                    SweepOptions::default(),
                    &|| false,
                )
                .unwrap();
            let mut d: StockDelta =
                serde_json::from_str(&serde_json::to_string(&d).unwrap()).unwrap();
            let before = replay.committed_clearance_fingerprint();
            let checked = replay
                .certify_verified_replay(
                    &mut d,
                    &tool.prepare().unwrap(),
                    &rapid,
                    SweepOptions::default(),
                    &|| false,
                )
                .unwrap();
            assert_eq!(checked.columns, 0);
            replay.apply_delta(&d).unwrap();
            assert_eq!(before, replay.committed_clearance_fingerprint());
        }
    }

    #[test]
    fn verified_replay_short_reach_requires_whole_stock_upper_bounds_and_budgets() {
        let (base, mut tool, motion) = native_fixture();
        tool.cutter.cutting_length_mm = 2.;
        let prepared = tool.prepare().unwrap();
        let mut snapshot = base.snapshot();
        snapshot.cells.fill(HeightBounds {
            lower_mm: 6.,
            upper_mm: 6.,
        });
        let mut cold = Stock::from_snapshot(snapshot.clone()).unwrap();
        let delta = cold
            .preview_sweep(&prepared, &motion, 1., SweepOptions::default(), &|| false)
            .unwrap();
        let raw = serde_json::to_string(&delta).unwrap();
        let mut replay = Stock::from_snapshot(snapshot.clone()).unwrap();
        let mut cached: StockDelta = serde_json::from_str(&raw).unwrap();
        let stats = replay
            .certify_verified_replay(
                &mut cached,
                &prepared,
                &motion,
                SweepOptions::default(),
                &|| false,
            )
            .unwrap();
        assert_eq!(stats.columns, 1);
        assert!(stats.reach_cell_visits > 0);
        replay.apply_delta(&cached).unwrap();
        cold.apply_delta(&delta).unwrap();
        assert_eq!(cold.snapshot(), replay.snapshot());
        assert_eq!(
            cold.committed_clearance_fingerprint(),
            replay.committed_clearance_fingerprint()
        );

        let source = Stock::from_snapshot(snapshot.clone()).unwrap();
        for mode in 0..3 {
            let mut cached: StockDelta = serde_json::from_str(&raw).unwrap();
            let options = if mode == 0 {
                SweepOptions {
                    max_cell_visits: 1,
                    ..Default::default()
                }
            } else {
                SweepOptions::default()
            };
            let before = source.snapshot();
            let polls = std::cell::Cell::new(0usize);
            let cancel = || {
                polls.set(polls.get() + 1);
                mode == 1 && polls.get() >= 4
            };
            if mode == 2 {
                cached.fraction = 0.5;
            }
            assert!(
                source
                    .certify_verified_replay(&mut cached, &prepared, &motion, options, &cancel)
                    .is_err()
            );
            assert_eq!(source.snapshot(), before);
            assert!(cached.native_identity.is_none() && cached.cleared_column.is_none());
        }
        let [nx, _] = base.dimensions();
        snapshot.cells[32 * nx + 32].upper_mm = 10.; // Potential tall material in the whole affected rectangle.
        let unknown = Stock::from_snapshot(snapshot).unwrap();
        let mut cached: StockDelta = serde_json::from_str(&raw).unwrap();
        assert!(
            unknown
                .certify_verified_replay(
                    &mut cached,
                    &prepared,
                    &motion,
                    SweepOptions::default(),
                    &|| false
                )
                .unwrap_err()
                .contains("cutting length")
        );
        assert!(cached.native_identity.is_none());
    }

    #[test]
    fn native_column_digest_tracks_order_and_resets_without_runtime_identity() {
        let (stock, tool, motion) = native_fixture();
        let prepared = tool.prepare().unwrap();
        let mut a = motion.clone();
        a.curve = Curve::Line {
            start: [8., 20., 5.],
            end: [18., 20., 5.],
        };
        let mut ab = stock.clone();
        let mut ba = stock.clone();
        for (target, order) in [(&mut ab, [&motion, &a]), (&mut ba, [&a, &motion])] {
            for (i, original) in order.into_iter().enumerate() {
                let mut m = original.clone();
                m.id = i as u64 + 1;
                let delta = target
                    .preview_sweep(&prepared, &m, 1., SweepOptions::default(), &|| false)
                    .unwrap();
                target.apply_delta(&delta).unwrap();
            }
        }
        assert_eq!(ab.snapshot(), ba.snapshot());
        assert_ne!(
            ab.committed_clearance_fingerprint(),
            ba.committed_clearance_fingerprint(),
            "recent-history order affects steering"
        );
        assert_eq!(
            ab.committed_clearance_fingerprint(),
            ab.clone().committed_clearance_fingerprint()
        );
        let empty = Stock::from_snapshot(ab.snapshot()).unwrap();
        assert_eq!(
            empty.committed_clearance_fingerprint(),
            stock.committed_clearance_fingerprint()
        );
    }

    #[test]
    fn native_arc_columns_preserve_annular_holes_and_partial_sweep_direction() {
        let (mut stock, tool, mut motion) = native_fixture();
        motion.curve = Curve::Arc {
            start: [16., 13., 5.],
            end: [16., 13., 5.],
            center: [13., 13., 5.],
            plane: Plane::Xy,
            sweep_rad: std::f64::consts::TAU,
        };
        let delta = stock
            .preview_sweep(
                &tool.prepare().unwrap(),
                &motion,
                1.,
                SweepOptions::default(),
                &|| false,
            )
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        assert!(known_box(&stock, [[15.9, 12.9], [16.1, 13.1]], 5.));
        assert!(
            !known_box(&stock, [[12.9, 12.9], [13.1, 13.1]], 5.),
            "the inner annular hole is still material"
        );
        assert!(!known_box(&stock, [[15.9, 12.9], [16.1, 13.1]], 4.99));
        let mut partial = Stock::new(stock.spec().clone()).unwrap();
        motion.curve = Curve::Arc {
            start: [16., 13., 5.],
            end: [10., 13., 5.],
            center: [13., 13., 5.],
            plane: Plane::Xy,
            sweep_rad: std::f64::consts::PI,
        };
        let delta = partial
            .preview_sweep(
                &tool.prepare().unwrap(),
                &motion,
                1.,
                SweepOptions::default(),
                &|| false,
            )
            .unwrap();
        partial.apply_delta(&delta).unwrap();
        assert!(known_box(&partial, [[12.9, 15.9], [13.1, 16.1]], 5.));
        assert!(
            !known_box(&partial, [[12.9, 9.9], [13.1, 10.1]], 5.),
            "partial arc does not clear its opposite half"
        );
    }

    #[test]
    fn persistent_column_cap_is_conservative_and_clone_drop_is_bounded() {
        let (stock, _, mut motion) = native_fixture();
        let column = CommittedColumn {
            sweep: ClearedSweep::from_motion(&motion, 2., 1e-5).unwrap(),
            tolerance_mm: 1e-5,
        };
        let mut history = CommittedColumns::default();
        for _ in 0..MAX_COMMITTED_COLUMNS {
            history.append(column.clone());
        }
        let shared = history.clone();
        assert!(Arc::ptr_eq(&history.roots[0], &shared.roots[0]));
        motion.curve = Curve::Line {
            start: [8., 22., 5.],
            end: [18., 22., 5.],
        };
        history.append(CommittedColumn {
            sweep: ClearedSweep::from_motion(&motion, 2., 1e-5).unwrap(),
            tolerance_mm: 1e-5,
        });
        let supports = history.collect(1e-5, &|| false).unwrap();
        assert_eq!(supports.len(), MAX_COMMITTED_COLUMNS);
        let scene = EngagementScene::new(stock.spec().clone(), supports).unwrap();
        assert!(
            scene
                .clear_box([[12.5, 12.5], [13.5, 13.5]], 5., &mut 1000, &|| false)
                .unwrap()
        );
        assert!(
            !scene
                .clear_box([[10., 21.5], [16., 22.5]], 5., &mut 1000, &|| false)
                .unwrap()
        );
        drop(history);
        drop(shared); // A quarter-million-node linked list would overflow here.
    }

    #[test]
    fn incremental_clearance_matches_full_history_and_rebuilds_on_reset_or_fork() {
        let (mut stock, _, mut motion) = native_fixture();
        let column = |motion: &mut Motion, i: usize, tolerance_mm: f64| {
            let y = 1. + (i % 997) as f64 * 0.02;
            motion.curve = Curve::Line {
                start: [8., y, 5.],
                end: [18., y + (i / 997) as f64 * 0.001, 5.],
            };
            CommittedColumn {
                sweep: ClearedSweep::from_motion(motion, 2., 1e-5).unwrap(),
                tolerance_mm,
            }
        };
        let prints =
            |v: &[ClearedSweep]| v.iter().map(ClearedSweep::fingerprint).collect::<Vec<_>>();
        let mut cursor = ClearanceCursor::default();
        let mut seen = vec![];
        let mut i = 0;
        // Uneven batch sizes cross every forest shape; mixed tolerances are
        // filtered the same way as a full collect.
        for batch in [0, 1, 1, 2, 3, 5, 8, 13, 100, 1, 255, 513, 7] {
            for _ in 0..batch {
                let tolerance = if i % 3 == 0 { 2e-5 } else { 1e-5 };
                stock
                    .committed_columns
                    .append(column(&mut motion, i, tolerance));
                i += 1;
            }
            let (rebuild, new) = stock
                .committed_clearance_since(&mut cursor, 1e-5, &|| false)
                .unwrap();
            assert!(!rebuild || seen.is_empty());
            seen.extend(new);
            let full = stock.committed_clearance(1e-5, &|| false).unwrap();
            assert_eq!(prints(&seen), prints(&full));
            for k in 0..stock.committed_columns.count {
                let ColumnNode::Leaf { column, .. } =
                    stock.committed_columns.leaf(k).unwrap().as_ref()
                else {
                    panic!("leaf lookup returned a branch")
                };
                let all = stock.committed_columns.collect(1., &|| false).unwrap();
                assert_eq!(column.sweep.fingerprint(), all[k].fingerprint());
                if k > 40 {
                    break;
                }
            }
        }
        // A different tolerance is a different filter: rebuild.
        let (rebuild, _) = stock
            .committed_clearance_since(&mut cursor, 2e-5, &|| false)
            .unwrap();
        assert!(rebuild);
        let (rebuild, _) = stock
            .committed_clearance_since(&mut cursor, 1e-5, &|| false)
            .unwrap();
        assert!(rebuild);
        // A fork that diverged before the consumed prefix ends must rebuild;
        // one that shares the whole consumed prefix extends it.
        let mut fork = stock.clone();
        let mut shared = cursor_clone(&cursor);
        fork.committed_columns
            .append(column(&mut motion, 5000, 1e-5));
        let (rebuild, new) = fork
            .committed_clearance_since(&mut shared, 1e-5, &|| false)
            .unwrap();
        assert!(!rebuild);
        assert_eq!(new.len(), 1);
        let mut diverged = stock.clone();
        diverged.committed_columns = CommittedColumns::default();
        for k in 0..stock.committed_columns.count {
            diverged
                .committed_columns
                .append(column(&mut motion, k, 1e-5));
        }
        let (rebuild, all) = diverged
            .committed_clearance_since(&mut cursor, 1e-5, &|| false)
            .unwrap();
        assert!(
            rebuild,
            "equal length and geometry are not the same history"
        );
        assert_eq!(all.len(), stock.committed_columns.count);
        // Undo/foreign deltas reset the history: rebuild from what remains.
        stock.committed_columns = CommittedColumns::default();
        let (rebuild, all) = stock
            .committed_clearance_since(&mut cursor, 1e-5, &|| false)
            .unwrap();
        assert!(rebuild && all.is_empty());
    }

    fn cursor_clone(cursor: &ClearanceCursor) -> ClearanceCursor {
        ClearanceCursor {
            seen: cursor.seen,
            last: cursor.last.clone(),
            tolerance_mm: cursor.tolerance_mm,
        }
    }

    #[test]
    fn closed_box_coverage_uses_upper_bounds_and_includes_touching_neighbor_cells() {
        let spec = StockSpec {
            version: 1,
            units: "mm".into(),
            min_mm: [0., 0., 0.],
            max_mm: [2., 1., 10.],
            cell_size_mm: 1.,
            max_cells: 10,
        };
        let mut snapshot = Stock::new(spec).unwrap().snapshot();
        snapshot.cells[0] = HeightBounds {
            lower_mm: 5.,
            upper_mm: 10.,
        };
        snapshot.cells[1] = HeightBounds {
            lower_mm: 5.,
            upper_mm: 5.,
        };
        let stock = Stock::from_snapshot(snapshot).unwrap();
        let mut work = 100;
        assert!(
            stock
                .bounds_clear([[1.01, 0.1], [1.99, 0.9]], 5., &mut work, &|| false)
                .unwrap()
        );
        assert!(
            !stock
                .bounds_clear([[1., 0.1], [1.99, 0.9]], 5., &mut work, &|| false)
                .unwrap()
        );
        assert!(
            !stock
                .bounds_clear([[0.1, 0.1], [0.9, 0.9]], 5., &mut work, &|| false)
                .unwrap()
        );
        assert!(
            stock
                .bounds_clear([[1.01, 0.1], [1.99, 0.9]], 5., &mut 0, &|| false)
                .is_err()
        );
        assert!(
            stock
                .bounds_clear([[1.01, 0.1], [1.99, 0.9]], 5., &mut work, &|| true)
                .is_err()
        );
    }
}
