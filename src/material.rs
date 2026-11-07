//! Compare bounded height stock against an independently authored prismatic
//! target. Whole-footprint enclosures, never center-only residual verdicts.
use crate::{
    buffers::HeightRead,
    planar::{ConstructionOptions, PlanarRegion, PreparedRegion},
    stock::{CancelSignal, Stock, StockSpec},
    stream::RunIdentity,
    tool::valid_id,
    verification::identity,
};
use serde::{Deserialize, Serialize};
mod contact_points;
mod inner_patches;
pub use contact_points::ContactWitnesses;
pub use inner_patches::InnerPlanarPatch;
mod finishing;
mod surface_target;
mod sweep_proof;
pub use finishing::{
    AccessFinding, AccessUncertainty, FinishingReport, FinishingScope, FinishingUncertainty,
};
use sha2::{Digest, Sha256};
pub use surface_target::{
    Definition, PreparedMaterial, PreparedSurfaceTarget, SurfaceEnclosure, SurfaceTarget,
};
pub(crate) use sweep_proof::SweepProof;
pub use sweep_proof::SweepProofReport;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Target {
    pub version: u32,
    pub units: String,
    pub id: String,
    pub revision: String,
    pub machining_region: PlanarRegion,
    pub floor_z_mm: f64,
    pub floor_error_mm: f64,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Options {
    pub geometry_tolerance_mm: f64,
    pub comparison_tolerance_mm: f64,
    pub max_tile_nodes: u64,
    pub max_boundary_tests: u64,
    pub max_classified_cells: usize,
    pub max_findings: usize,
}
impl Default for Options {
    fn default() -> Self {
        Self {
            geometry_tolerance_mm: 1e-5,
            comparison_tolerance_mm: 0.001,
            max_tile_nodes: 1_000_000,
            max_boundary_tests: 50_000_000,
            max_classified_cells: u32::MAX as usize,
            max_findings: 1024,
        }
    }
}
impl Options {
    pub(crate) fn validate(self) -> Result<(), String> {
        if !self.geometry_tolerance_mm.is_finite()
            || !(1e-7..=0.01).contains(&self.geometry_tolerance_mm)
            || !self.comparison_tolerance_mm.is_finite()
            || !(1e-7..=1.).contains(&self.comparison_tolerance_mm)
            || !(1..=10_000_000).contains(&self.max_tile_nodes)
            || !(1..=1_000_000_000).contains(&self.max_boundary_tests)
            || !(1..=u32::MAX as usize).contains(&self.max_classified_cells)
            || !(1..=4096).contains(&self.max_findings)
        {
            return Err("invalid bounded material verification options".into());
        }
        Ok(())
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RunBinding {
    pub run: RunIdentity,
    pub manifest_sha256: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Context {
    pub stock: StockSpec,
    pub generation: u64,
    pub fraction: f64,
    pub run: Option<RunBinding>,
}
impl Context {
    pub fn dimensions(&self) -> Result<[usize; 2], String> {
        self.stock.grid().map(|(nx, ny, _)| [nx, ny])
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Assessment {
    WithinTolerance,
    DefiniteExcess,
    PossibleExcess,
    Incomplete,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FootprintClass {
    Inside,
    Outside,
    Mixed,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Finding {
    pub cell_index: u32,
    pub xy_mm: [f64; 4],
    pub target_class: FootprintClass,
    pub actual_height_mm: [f64; 2],
    pub target_height_mm: [f64; 2],
    pub residual_height_mm: [f64; 2],
    pub gouge_height_mm: [f64; 2],
    pub residual: Assessment,
    pub gouge: Assessment,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Summary {
    pub assessment: Assessment,
    pub definite_cells: usize,
    pub possible_only_cells: usize,
    /// Raw physical excess volume, not a tolerance-subtracted volume.
    pub volume_mm3: [f64; 2],
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Stats {
    pub validated_cells: usize,
    pub classified_cells: usize,
    pub unexamined_cells: usize,
    pub tile_nodes: u64,
    pub geometry_queries: u64,
    pub estimated_boundary_tests: u64,
    pub uniform_tiles: usize,
    pub mixed_cells: usize,
    pub max_stack_nodes: usize,
    pub work_exhausted: bool,
    pub findings_truncated: bool,
}
#[derive(Debug, Serialize, Deserialize)]
pub struct Report {
    pub algorithm: String,
    pub target_id: String,
    pub target_revision: String,
    pub target_sha256: String,
    pub stock_sha256: String,
    pub context: Context,
    pub comparison_tolerance_mm: f64,
    pub complete: bool,
    pub residual: Summary,
    pub gouge: Summary,
    pub findings: Vec<Finding>,
    pub stats: Stats,
    pub numerical_policy: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub finishing: Option<FinishingReport>,
}

pub struct PreparedTarget {
    definition: Target,
    region: PreparedRegion,
    options: Options,
    guard_xy: f64,
    segments: u64,
    digest: String,
}
impl Target {
    pub fn prepare(&self, options: Options) -> Result<PreparedTarget, String> {
        options.validate()?;
        if self.version != 1
            || self.units != "mm"
            || !valid_id(&self.id)
            || self.revision.is_empty()
            || self.revision.len() > 512
            || !self.floor_z_mm.is_finite()
            || self.floor_z_mm.abs() > 1e6
            || !self.floor_error_mm.is_finite()
            || !(0.0..=1.).contains(&self.floor_error_mm)
        {
            return Err(
                "material target needs version 1/mm, identity/revision and finite floor/error"
                    .into(),
            );
        }
        let region = self.machining_region.prepare(ConstructionOptions {
            tolerance_mm: options.geometry_tolerance_mm,
        })?;
        let summary = region.summary();
        if summary.segments > 65_536 {
            return Err("material target exceeds 65536 boundary segments".into());
        }
        Ok(PreparedTarget {
            definition: self.clone(),
            region,
            options,
            guard_xy: summary.boundary_error_mm
                + summary.numerical_guard_mm
                + options.geometry_tolerance_mm,
            segments: summary.segments as u64,
            digest: identity(&(self, options))?,
        })
    }
}

impl HeightRead for Stock {
    fn scalar_len(&self) -> usize {
        self.cells().len() * 2
    }
    fn read(&self, i: usize) -> f64 {
        let h = self.cells()[i / 2];
        if i.is_multiple_of(2) {
            h.lower_mm
        } else {
            h.upper_mm
        }
    }
}
#[derive(Default)]
struct Sum {
    value: f64,
    error: f64,
    terms: usize,
}
impl Sum {
    fn add(&mut self, value: f64) {
        let corrected = value - self.error;
        let next = self.value + corrected;
        self.error = (next - self.value) - corrected;
        self.value = next;
        self.terms += 1;
    }
    fn bound(&self, upper: bool) -> f64 {
        let pad = 64. * f64::EPSILON * (self.terms + 1) as f64 * self.value.abs();
        if upper {
            (self.value + pad).next_up()
        } else {
            (self.value - pad).max(0.).next_down().max(0.)
        }
    }
}
#[derive(Default)]
struct Accumulator {
    lower: Sum,
    upper: Sum,
    definite: usize,
    possible: usize,
}
impl Accumulator {
    fn add(&mut self, height: [f64; 2], area: f64, tolerance: f64) -> Assessment {
        self.lower.add(height[0] * area * (1. - 32. * f64::EPSILON));
        self.upper.add(height[1] * area * (1. + 32. * f64::EPSILON));
        let class = assess(height, tolerance);
        match class {
            Assessment::DefiniteExcess => self.definite += 1,
            Assessment::PossibleExcess => self.possible += 1,
            _ => {}
        }
        class
    }
    fn finish(mut self, unexamined: usize, unknown_volume: f64) -> Summary {
        if unexamined > 0 {
            self.upper.add(unknown_volume);
        }
        Summary {
            assessment: if unexamined > 0 {
                Assessment::Incomplete
            } else if self.definite > 0 {
                Assessment::DefiniteExcess
            } else if self.possible > 0 {
                Assessment::PossibleExcess
            } else {
                Assessment::WithinTolerance
            },
            definite_cells: self.definite,
            possible_only_cells: self.possible,
            volume_mm3: [self.lower.bound(false), self.upper.bound(true)],
        }
    }
}
fn assess(height: [f64; 2], tolerance: f64) -> Assessment {
    if height[0] > tolerance {
        Assessment::DefiniteExcess
    } else if height[1] > tolerance {
        Assessment::PossibleExcess
    } else {
        Assessment::WithinTolerance
    }
}
fn cancelled(cancel: &impl CancelSignal) -> Result<(), String> {
    if cancel.is_cancelled() {
        Err("material verification cancelled; no report published".into())
    } else {
        Ok(())
    }
}

impl PreparedTarget {
    pub fn digest(&self) -> &str {
        &self.digest
    }
    pub fn definition(&self) -> &Target {
        &self.definition
    }
    pub fn check_stock(&self, stock: &Stock, cancel: &impl CancelSignal) -> Result<Report, String> {
        let context = Context {
            stock: stock.spec().clone(),
            generation: stock.generation(),
            fraction: 0.,
            run: None,
        };
        self.check(stock, &context, cancel)
    }
    pub fn check(
        &self,
        heights: &impl HeightRead,
        context: &Context,
        cancel: &impl CancelSignal,
    ) -> Result<Report, String> {
        check_material(self, heights, context, None, cancel).map(|(r, _)| r)
    }
}

trait MaterialGeometry {
    fn finishing_target(&self) -> Option<&PreparedSurfaceTarget> {
        None
    }
    fn options(&self) -> Options;
    fn floor(&self) -> f64;
    fn accepts_stock_z(&self, stock: &StockSpec) -> bool {
        (stock.min_mm[2]..=stock.max_mm[2]).contains(&self.floor())
    }
    fn id(&self) -> &str;
    fn revision(&self) -> &str;
    fn digest(&self) -> &str;
    fn algorithm(&self) -> &'static str;
    fn policy(&self) -> &'static str;
    fn bounds(
        &self,
        xy: [f64; 4],
        leaf: bool,
        spec: &StockSpec,
        arithmetic: f64,
        stats: &mut Stats,
        cancel: &impl CancelSignal,
    ) -> Result<Option<(FootprintClass, [f64; 2])>, String>;
}

impl MaterialGeometry for PreparedTarget {
    fn options(&self) -> Options {
        self.options
    }
    fn floor(&self) -> f64 {
        self.definition.floor_z_mm
    }
    fn id(&self) -> &str {
        &self.definition.id
    }
    fn revision(&self) -> &str {
        &self.definition.revision
    }
    fn digest(&self) -> &str {
        &self.digest
    }
    fn algorithm(&self) -> &'static str {
        "prismatic_material_v1"
    }
    fn policy(&self) -> &'static str {
        "Whole-cell stock premise; independent prismatic target footprint enclosures with source/error guards and compensated volume sums. Binary64 engineering bounds, not certified stock provenance or continuous tool-contact verification."
    }
    fn bounds(
        &self,
        xy: [f64; 4],
        _leaf: bool,
        spec: &StockSpec,
        arithmetic: f64,
        stats: &mut Stats,
        _cancel: &impl CancelSignal,
    ) -> Result<Option<(FootprintClass, [f64; 2])>, String> {
        if stats.estimated_boundary_tests + self.segments > self.options.max_boundary_tests {
            return Ok(None);
        }
        stats.estimated_boundary_tests += self.segments;
        let center = [(xy[0] + xy[2]) * 0.5, (xy[1] + xy[3]) * 0.5];
        let radius = (xy[2] - xy[0]).hypot(xy[3] - xy[1]) * 0.5;
        let class = if self.segments == 0 {
            FootprintClass::Outside
        } else {
            stats.geometry_queries += 1;
            let d = self
                .region
                .query(center)?
                .signed_clearance_mm
                .ok_or("empty material boundary")?;
            if d > radius + self.guard_xy + arithmetic {
                FootprintClass::Inside
            } else if d < -radius - self.guard_xy - arithmetic {
                FootprintClass::Outside
            } else {
                FootprintClass::Mixed
            }
        };
        let inside = [
            (self.floor() - self.definition.floor_error_mm).max(spec.min_mm[2]),
            (self.floor() + self.definition.floor_error_mm).min(spec.max_mm[2]),
        ];
        Ok(Some((
            class,
            match class {
                FootprintClass::Inside => inside,
                FootprintClass::Outside => [spec.max_mm[2]; 2],
                FootprintClass::Mixed => [inside[0], spec.max_mm[2]],
            },
        )))
    }
}

fn check_material(
    target: &impl MaterialGeometry,
    heights: &impl HeightRead,
    context: &Context,
    proof: Option<&SweepProof>,
    cancel: &impl CancelSignal,
) -> Result<(Report, usize), String> {
    let options = target.options();
    cancelled(cancel)?;
    let spec = &context.stock;
    let (nx, ny, step) = spec.grid()?;
    let cells = nx * ny;
    if heights.scalar_len() != cells * 2
        || !context.fraction.is_finite()
        || !(0.0..=1.).contains(&context.fraction)
        || !target.accepts_stock_z(spec)
    {
        return Err(
            "material stock buffer/cursor/floor does not match supported stock grid".into(),
        );
    }
    if let Some(run) = &context.run
        && (!valid_id(&run.run.id)
            || run.run.revision.is_empty()
            || run.run.revision.len() > 512
            || run.manifest_sha256.len() != 64
            || !run
                .manifest_sha256
                .bytes()
                .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b)))
    {
        return Err("invalid material run/manifest identity".into());
    }
    let mut hash = Sha256::new();
    hash.update(b"camlib-material-stock-v1");
    hash.update(identity(context)?.as_bytes());
    // Validate/hash ALL input even when subsequent classification work is capped.
    let mut block = [0u8; 16 * 1024];
    for start in (0..cells).step_by(1024) {
        cancelled(cancel)?;
        let count = (cells - start).min(1024);
        for j in 0..count {
            let i = start + j;
            let lo = heights.read(2 * i);
            let hi = heights.read(2 * i + 1);
            if !lo.is_finite()
                || !hi.is_finite()
                || lo < spec.min_mm[2]
                || lo > hi
                || hi > spec.max_mm[2]
            {
                return Err("invalid material stock height bounds".into());
            }
            block[16 * j..16 * j + 8].copy_from_slice(&lo.to_le_bytes());
            block[16 * j + 8..16 * j + 16].copy_from_slice(&hi.to_le_bytes());
        }
        // Same exact byte sequence, bounded16KiB scratch and1024-cell cancel
        // polling. Avoid millions of tiny digest-buffer updates.
        hash.update(&block[..count * 16]);
    }
    let stock_sha256 = format!("{:x}", hash.finalize());
    let mut stats = Stats {
        validated_cells: cells,
        ..Stats::default()
    };
    let mut findings = Vec::new();
    let mut finishing = target
        .finishing_target()
        .map(|t| finishing::FinishingAccumulator::new(t, spec))
        .transpose()?;
    let (mut residual, mut gouge) = (Accumulator::default(), Accumulator::default());
    let mut decided = 0;
    let area = step[0] * step[1];
    let magnitude = spec
        .min_mm
        .into_iter()
        .chain(spec.max_mm)
        .map(f64::abs)
        .fold(1., f64::max);
    let arithmetic = 256. * f64::EPSILON * magnitude;
    let mut stack = vec![[0, 0, nx, ny]];
    'tiles: while let Some([x0, y0, x1, y1]) = stack.pop() {
        cancelled(cancel)?;
        stats.max_stack_nodes = stats.max_stack_nodes.max(stack.len() + 1);
        if stats.tile_nodes == options.max_tile_nodes
            || stats.classified_cells == options.max_classified_cells
        {
            stats.work_exhausted = true;
            break;
        }
        stats.tile_nodes += 1;
        let xy = [
            spec.min_mm[0] + x0 as f64 * step[0],
            spec.min_mm[1] + y0 as f64 * step[1],
            spec.min_mm[0] + x1 as f64 * step[0],
            spec.min_mm[1] + y1 as f64 * step[1],
        ];
        let Some((class, desired)) = target.bounds(
            xy,
            x1 - x0 == 1 && y1 - y0 == 1,
            spec,
            arithmetic,
            &mut stats,
            cancel,
        )?
        else {
            stats.work_exhausted = true;
            break;
        };
        if class == FootprintClass::Mixed && (x1 - x0 > 1 || y1 - y0 > 1) {
            let xm = x0 + (x1 - x0) / 2;
            let ym = y0 + (y1 - y0) / 2;
            let xs = if xm == x0 {
                vec![(x0, x1)]
            } else {
                vec![(x0, xm), (xm, x1)]
            };
            let ys = if ym == y0 {
                vec![(y0, y1)]
            } else {
                vec![(y0, ym), (ym, y1)]
            };
            // Fixed <=4 children, depth bounded by validated integer grid.
            for &(a, b) in ys.iter().rev() {
                for &(c, d) in xs.iter().rev() {
                    stack.push([c, a, d, b]);
                }
            }
            continue;
        }
        if class != FootprintClass::Mixed {
            stats.uniform_tiles += 1;
        }
        for y in y0..y1 {
            for x in x0..x1 {
                if stats.classified_cells.is_multiple_of(1024) {
                    cancelled(cancel)?;
                }
                if stats.classified_cells == options.max_classified_cells {
                    stats.work_exhausted = true;
                    break 'tiles;
                }
                let i = y * nx + x;
                let actual = [heights.read(2 * i), heights.read(2 * i + 1)];
                if let Some(scope) = &mut finishing {
                    let cell_xy = [
                        spec.min_mm[0] + x as f64 * step[0],
                        spec.min_mm[1] + y as f64 * step[1],
                        spec.min_mm[0] + (x + 1) as f64 * step[0],
                        spec.min_mm[1] + (y + 1) as f64 * step[1],
                    ];
                    scope.add(i, cell_xy, actual, desired, cancel)?;
                }
                let r = [
                    (actual[0] - desired[1] - arithmetic).max(0.),
                    (actual[1] - desired[0] + arithmetic).max(0.),
                ];
                let g = [
                    (desired[0] - actual[1] - arithmetic).max(0.),
                    (desired[1] - actual[0] + arithmetic).max(0.),
                ];
                let rc = residual.add(r, area, options.comparison_tolerance_mm);
                let gc = gouge.add(g, area, options.comparison_tolerance_mm);
                // The report stays the cell comparison (reproducible from
                // the stock alone); the per-move proof counts, beside it,
                // the possible-only gouge cells it rules out (no swept
                // point deeper than the tolerance into the part, no
                // unproven motion on the cell). Definite cells never.
                if gc == Assessment::PossibleExcess && proof.is_some_and(|p| p.decides(i)) {
                    decided += 1;
                }
                stats.classified_cells += 1;
                if class == FootprintClass::Mixed {
                    stats.mixed_cells += 1;
                }
                if rc != Assessment::WithinTolerance || gc != Assessment::WithinTolerance {
                    if findings.len() < options.max_findings {
                        findings.push(Finding {
                            cell_index: i as u32,
                            xy_mm: [
                                spec.min_mm[0] + x as f64 * step[0],
                                spec.min_mm[1] + y as f64 * step[1],
                                spec.min_mm[0] + (x + 1) as f64 * step[0],
                                spec.min_mm[1] + (y + 1) as f64 * step[1],
                            ],
                            target_class: class,
                            actual_height_mm: actual,
                            target_height_mm: desired,
                            residual_height_mm: r,
                            gouge_height_mm: g,
                            residual: rc,
                            gouge: gc,
                        });
                    } else {
                        stats.findings_truncated = true;
                    }
                }
            }
        }
    }
    stats.unexamined_cells = cells - stats.classified_cells;
    let unknown = (spec.max_mm[2] - spec.min_mm[2])
        * area
        * stats.unexamined_cells as f64
        * (1. + 32. * f64::EPSILON);
    let finishing = finishing.map(finishing::FinishingAccumulator::finish);
    let report = Report {
        algorithm: target.algorithm().into(),
        target_id: target.id().into(),
        target_revision: target.revision().into(),
        target_sha256: target.digest().into(),
        stock_sha256,
        context: context.clone(),
        comparison_tolerance_mm: options.comparison_tolerance_mm,
        complete: stats.unexamined_cells == 0 && finishing.as_ref().is_none_or(|f| f.complete),
        residual: residual.finish(stats.unexamined_cells, unknown),
        gouge: gouge.finish(stats.unexamined_cells, unknown),
        findings,
        stats,
        numerical_policy: target.policy().into(),
        finishing,
    };
    cancelled(cancel)?;
    Ok((report, decided))
}
