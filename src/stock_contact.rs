//! Independent continuous contact against validated whole-cell height stock.
//! A stock-owned hierarchy supplies enclosures, never sampled-sweep clearance.
use crate::{
    motion::{Bounds3, Curve, Motion, MoveKind, PreparedCurve, Vec3},
    stock::{CancelSignal, CellChange, HeightBounds, Stock, StockSpec},
    tool::{CutterShape, PreparedAssembly, valid_id},
    verification::{self, Finding, FindingKind, Part, Status, ToolPart, Witness},
};
use serde::{Deserialize, Serialize};

const TILE: usize = 16;
const NONE: usize = usize::MAX;
#[derive(Clone, Debug)]
struct Node {
    rect: [usize; 4], // exclusive cell rectangle
    children: [usize; 2],
    parent: usize,
    lower: f64,
    upper: f64,
}
#[derive(Clone, Copy, Debug, Default, Serialize)]
pub struct IndexStats {
    pub nodes: usize,
    pub bytes: usize,
    pub scanned_cells: usize,
    pub changed_tiles: usize,
    /// Nodes recomputed by the last commit (zero for an unchanged delta).
    pub updated_nodes: usize,
}
#[derive(Clone, Debug)]
pub(crate) struct HeightIndex {
    nodes: Vec<Node>,
    leaves: Vec<usize>,
    dirty: Vec<bool>,
    tx: usize,
    pub stats: IndexStats,
}
impl HeightIndex {
    pub(crate) fn new(
        spec: &StockSpec,
        cells: &[HeightBounds],
        uniform_full: bool,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        let (nx, ny, _) = spec.grid()?;
        let tx = nx.div_ceil(TILE);
        let ty = ny.div_ceil(TILE);
        let mut result = Self {
            nodes: Vec::with_capacity(tx * ty * 2 - 1),
            leaves: vec![0; tx * ty],
            dirty: Vec::new(),
            tx,
            stats: IndexStats::default(),
        };
        result.build([0, 0, tx, ty], NONE, [nx, ny], spec.max_mm[2], cancel)?;
        if !uniform_full {
            for i in 0..result.nodes.len() {
                if result.nodes[i].children[0] == NONE {
                    if cancel.is_cancelled() {
                        return Err("stock contact index preparation cancelled".into());
                    }
                    result.refresh_leaf(i, cells, nx);
                    result.stats.scanned_cells += (result.nodes[i].rect[2]
                        - result.nodes[i].rect[0])
                        * (result.nodes[i].rect[3] - result.nodes[i].rect[1]);
                }
            }
            result.recombine();
        }
        if cancel.is_cancelled() {
            return Err("stock contact index preparation cancelled".into());
        }
        result.dirty = vec![false; result.nodes.len()];
        result.stats.nodes = result.nodes.len();
        result.stats.bytes = result.nodes.capacity() * std::mem::size_of::<Node>()
            + result.leaves.capacity() * std::mem::size_of::<usize>()
            + result.dirty.capacity();
        Ok(result)
    }
    fn build(
        &mut self,
        tile: [usize; 4],
        parent: usize,
        dims: [usize; 2],
        top: f64,
        cancel: &impl CancelSignal,
    ) -> Result<usize, String> {
        if cancel.is_cancelled() {
            return Err("stock contact index preparation cancelled".into());
        }
        let id = self.nodes.len();
        self.nodes.push(Node {
            rect: [
                tile[0] * TILE,
                tile[1] * TILE,
                (tile[2] * TILE).min(dims[0]),
                (tile[3] * TILE).min(dims[1]),
            ],
            children: [NONE; 2],
            parent,
            lower: top,
            upper: top,
        });
        let w = tile[2] - tile[0];
        let h = tile[3] - tile[1];
        if w == 1 && h == 1 {
            self.leaves[tile[1] * self.tx + tile[0]] = id;
        } else {
            let axis = usize::from(h > w);
            let middle = tile[axis] + (tile[axis + 2] - tile[axis]) / 2;
            let mut a = tile;
            let mut b = tile;
            a[axis + 2] = middle;
            b[axis] = middle;
            self.nodes[id].children = [
                self.build(a, id, dims, top, cancel)?,
                self.build(b, id, dims, top, cancel)?,
            ];
        }
        Ok(id)
    }
    fn refresh_leaf(&mut self, id: usize, cells: &[HeightBounds], nx: usize) {
        let [x0, y0, x1, y1] = self.nodes[id].rect;
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        for y in y0..y1 {
            for h in &cells[y * nx + x0..y * nx + x1] {
                lo = lo.min(h.lower_mm);
                hi = hi.max(h.upper_mm);
            }
        }
        self.nodes[id].lower = lo;
        self.nodes[id].upper = hi;
    }
    fn combine(&mut self, id: usize) {
        let [a, b] = self.nodes[id].children;
        self.nodes[id].lower = self.nodes[a].lower.min(self.nodes[b].lower);
        self.nodes[id].upper = self.nodes[a].upper.max(self.nodes[b].upper);
    }
    fn recombine(&mut self) {
        for i in (0..self.nodes.len()).rev() {
            if self.nodes[i].children[0] != NONE {
                self.combine(i);
            }
        }
    }
    pub(crate) fn update(&mut self, cells: &[HeightBounds], nx: usize, changes: &[CellChange]) {
        self.stats.scanned_cells = 0;
        self.stats.changed_tiles = 0;
        self.stats.updated_nodes = 0;
        for c in changes {
            let i = c.index as usize;
            let mut id = self.leaves[(i / nx / TILE) * self.tx + (i % nx / TILE)];
            // Mark the ancestor closure once. A shared dirty ancestor already
            // implies that all its ancestors are dirty as well.
            while id != NONE && !self.dirty[id] {
                self.dirty[id] = true;
                id = self.nodes[id].parent;
            }
        }
        self.refresh_dirty(0, cells, nx);
    }
    fn refresh_dirty(&mut self, id: usize, cells: &[HeightBounds], nx: usize) {
        if !self.dirty[id] {
            return;
        }
        let [a, b] = self.nodes[id].children;
        if a == NONE {
            self.refresh_leaf(id, cells, nx);
            let r = self.nodes[id].rect;
            self.stats.scanned_cells += (r[2] - r[0]) * (r[3] - r[1]);
            self.stats.changed_tiles += 1;
        } else {
            self.refresh_dirty(a, cells, nx);
            self.refresh_dirty(b, cells, nx);
            self.combine(id);
        }
        self.stats.updated_nodes += 1;
        self.dirty[id] = false;
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Options {
    pub tolerance_mm: f64,
    pub spatial_resolution_mm: f64,
    pub max_nodes: u64,
    pub max_cell_reads: u64,
    pub max_findings: usize,
}
impl Default for Options {
    fn default() -> Self {
        Self {
            tolerance_mm: 1e-5,
            spatial_resolution_mm: 0.001,
            max_nodes: 200_000,
            max_cell_reads: 2_000_000,
            max_findings: 128,
        }
    }
}
impl Options {
    pub(crate) fn validate(self) -> Result<(), String> {
        if !self.tolerance_mm.is_finite()
            || !(1e-7..=0.1).contains(&self.tolerance_mm)
            || !self.spatial_resolution_mm.is_finite()
            || !(self.tolerance_mm..=1.).contains(&self.spatial_resolution_mm)
            || !(1..=2_000_000).contains(&self.max_nodes)
            || !(1..=100_000_000).contains(&self.max_cell_reads)
            || !(1..=1024).contains(&self.max_findings)
        {
            return Err("invalid bounded stock contact options".into());
        }
        Ok(())
    }
}
#[derive(Debug, Default, Serialize, Deserialize)]
pub struct Stats {
    pub nodes: u64,
    pub cell_reads: u64,
    pub pruned_nodes: u64,
    pub removal_certificates: u64,
    pub parts_examined: usize,
    pub unexamined_parts: usize,
    pub max_stack_nodes: usize,
    pub work_exhausted: bool,
    pub findings_truncated: bool,
}
#[derive(Debug, Serialize, Deserialize)]
pub struct ContactFinding {
    pub finding: Finding,
    pub cell_range: [usize; 4],
    pub solid_lower_mm: f64,
    pub possible_upper_mm: f64,
    pub contact_point_mm: Option<Vec3>,
}
#[derive(Debug, Serialize, Deserialize)]
pub struct Report {
    pub algorithm: String,
    pub stock: StockSpec,
    pub pre_generation: u64,
    pub options_sha256: String,
    pub tool_sha256: String,
    pub motion_sha256: String,
    pub motion_id: u64,
    pub operation: String,
    pub tool: String,
    pub scope: String,
    pub status: Status,
    pub fully_examined: bool,
    pub findings: Vec<ContactFinding>,
    pub stats: Stats,
    pub numerical_policy: String,
}

#[derive(Clone, Copy)]
struct Task {
    rect: [usize; 4],
    node: usize,
    span: [f64; 2],
    depth: u8,
}
struct Search<'a, C> {
    stock: &'a Stock,
    index: &'a HeightIndex,
    tool: &'a PreparedAssembly,
    motion: &'a Motion,
    path: PreparedCurve,
    guard: f64,
    options: Options,
    stats: Stats,
    cancel: &'a C,
}
fn rect_bounds(stock: &Stock, r: [usize; 4]) -> [f64; 4] {
    let min = stock.spec().min_mm;
    let max = stock.spec().max_mm;
    let [dx, dy] = stock.cell_widths_mm();
    [
        min[0] + r[0] as f64 * dx,
        min[1] + r[1] as f64 * dy,
        (min[0] + r[2] as f64 * dx).min(max[0]),
        (min[1] + r[3] as f64 * dy).min(max[1]),
    ]
}
fn signed_rect(p: [f64; 2], r: [f64; 4]) -> f64 {
    let dx = (r[0] - p[0]).max(p[0] - r[2]).max(0.);
    let dy = (r[1] - p[1]).max(p[1] - r[3]).max(0.);
    if dx > 0. || dy > 0. {
        -dx.hypot(dy)
    } else {
        (p[0] - r[0])
            .min(r[2] - p[0])
            .min(p[1] - r[1])
            .min(r[3] - p[1])
    }
}
impl<C: CancelSignal> Search<'_, C> {
    fn charge(&mut self) -> Result<bool, String> {
        if self.cancel.is_cancelled() {
            return Err("stock contact cancelled; no report published".into());
        }
        if self.stats.nodes >= self.options.max_nodes {
            self.stats.work_exhausted = true;
            return Ok(false);
        }
        self.stats.nodes += 1;
        Ok(true)
    }
    fn heights(&mut self, task: Task) -> Option<[f64; 2]> {
        if task.node != NONE {
            let n = &self.index.nodes[task.node];
            return Some([n.lower, n.upper]);
        }
        let [x0, y0, x1, y1] = task.rect;
        let count = ((x1 - x0) * (y1 - y0)) as u64;
        if count > self.options.max_cell_reads - self.stats.cell_reads {
            self.stats.work_exhausted = true;
            return None;
        }
        self.stats.cell_reads += count;
        let nx = self.stock.dimensions()[0];
        let mut result = [f64::INFINITY, f64::NEG_INFINITY];
        for y in y0..y1 {
            for h in &self.stock.cells()[y * nx + x0..y * nx + x1] {
                result[0] = result[0].min(h.lower_mm);
                result[1] = result[1].max(h.upper_mm);
            }
        }
        Some(result)
    }
    fn bounds(&self, span: [f64; 2]) -> Result<Bounds3, String> {
        let p = self.path.position((span[0] + span[1]) / 2.)?;
        let reach = self.path.length_mm() * (span[1] - span[0]) / 2.;
        let global = self.path.bounds();
        let mut result = Bounds3 {
            min: std::array::from_fn(|k| (p[k] - reach).max(global.min[k])),
            max: std::array::from_fn(|k| (p[k] + reach).min(global.max[k])),
        };
        if matches!(self.motion.curve, Curve::Line { .. })
            || matches!(
                self.motion.curve,
                Curve::Arc {
                    plane: crate::motion::Plane::Xy,
                    ..
                } | Curve::Helix {
                    plane: crate::motion::Plane::Xy,
                    ..
                }
            )
        {
            let a = self.path.position(span[0])?;
            let b = self.path.position(span[1])?;
            result.min[2] = a[2].min(b[2]);
            result.max[2] = a[2].max(b[2]);
            if matches!(self.motion.curve, Curve::Line { .. }) {
                for k in 0..2 {
                    result.min[k] = a[k].min(b[k]);
                    result.max[k] = a[k].max(b[k]);
                }
            }
        }
        Ok(result)
    }
    fn axis_distance(&self, span: [f64; 2], p: [f64; 2]) -> Result<f64, String> {
        if let Some(d) = verification::circle_axis_distance(self.motion, &self.path, span, p) {
            return Ok(d);
        }
        let mid = self.path.position((span[0] + span[1]) / 2.)?;
        let bounds = self.bounds(span)?;
        let box_distance = ((bounds.min[0] - p[0]).max(p[0] - bounds.max[0]).max(0.))
            .hypot((bounds.min[1] - p[1]).max(p[1] - bounds.max[1]).max(0.));
        Ok(((p[0] - mid[0]).hypot(p[1] - mid[1])
            - self.path.length_mm() * (span[1] - span[0]) / 2.)
            .max(box_distance))
    }
    fn bottom_at_radius(&self, r: f64) -> Option<f64> {
        let c = &self.tool.definition().cutter;
        let radius = c.diameter_mm / 2.;
        if r > radius {
            return None;
        }
        Some(match c.shape {
            CutterShape::Flat | CutterShape::Beam => 0.,
            CutterShape::Ball => radius - (radius * radius - r * r).max(0.).sqrt(),
            CutterShape::Bull {
                corner_radius_mm: k,
            } => {
                let d = (r - (radius - k)).max(0.);
                k - (k * k - d * d).max(0.).sqrt()
            }
            CutterShape::Drill { point_angle_rad: a } => r / (a / 2.).tan(),
            CutterShape::Chamfer {
                included_angle_rad: a,
                tip_diameter_mm: d,
            } => (r - d / 2.).max(0.) / (a / 2.).tan(),
        })
    }
    /// A fixed earlier pose must cover EVERY point in a tile and reach its old
    /// top. This is an enclosure certificate, not a sampled current-sweep test.
    fn earlier_upper(&mut self, rect: [f64; 4], top: f64, before: f64) -> Result<f64, String> {
        if before == 0.
            || self.motion.kind.is_rapid()
            || !self.tool.definition().cutter.center_cutting
        {
            return Ok(top);
        }
        let mut candidates = [0.; 8];
        candidates[..5].copy_from_slice(&[0., before / 4., before / 2., before * 0.75, before]);
        let center = [(rect[0] + rect[2]) / 2., (rect[1] + rect[3]) / 2.];
        if let Curve::Line { start, end } = &self.motion.curve {
            let dx = end[0] - start[0];
            let dy = end[1] - start[1];
            let l = dx * dx + dy * dy;
            if l > 0. {
                candidates[5] = (((center[0] - start[0]) * dx + (center[1] - start[1]) * dy) / l)
                    .clamp(0., before);
            }
            let dz = end[2] - start[2];
            if dz != 0. {
                candidates[6] = ((top - self.tool.definition().cutter.cutting_length_mm
                    + 4. * self.guard
                    - start[2])
                    / dz)
                    .clamp(0., before);
                candidates[7] =
                    ((top - self.tool.definition().cutter.cutting_length_mm / 2. - start[2]) / dz)
                        .clamp(0., before);
            }
        }
        let mut upper = top;
        // Compose only in time order: a later pose can reach the remaining top
        // certified by earlier poses, even if it no longer reaches original top.
        candidates.sort_by(f64::total_cmp);
        for t in candidates {
            let p = self.path.position(t)?;
            if p[2] + self.tool.definition().cutter.cutting_length_mm < upper + 2. * self.guard {
                continue;
            }
            let far = (p[0] - rect[0])
                .abs()
                .max((p[0] - rect[2]).abs())
                .hypot((p[1] - rect[1]).abs().max((p[1] - rect[3]).abs()))
                + 2. * self.guard;
            if let Some(z) = self.bottom_at_radius(far) {
                upper = upper.min(p[2] + z + 2. * self.guard);
                self.stats.removal_certificates += 1;
            }
        }
        Ok(upper.max(self.stock.spec().min_mm[2]))
    }
    fn uncut_ball(&self, point: Vec3, radius: f64, t: f64) -> Result<bool, String> {
        if self.motion.kind.is_rapid() || t == 0. {
            return Ok(true);
        }
        let bounds = self.bounds([0., t])?;
        let c = &self.tool.definition().cutter;
        Ok(
            point[2] - radius > bounds.max[2] + c.cutting_length_mm + self.guard
                || point[2] + radius < bounds.min[2] - self.guard
                || self.axis_distance([0., t], [point[0], point[1]])?
                    > c.diameter_mm / 2. + radius + self.guard,
        )
    }
    fn witness(
        &self,
        part: &Part,
        task: Task,
        rect: [f64; 4],
        lower: f64,
    ) -> Result<Option<(Witness, Vec3)>, String> {
        let t = (task.span[0] + task.span[1]) / 2.;
        let p = self.path.position(t)?;
        let low = part.min.max(self.stock.spec().min_mm[2] - p[2]);
        let high = part.max.min(lower - p[2]);
        if high - low <= 8. * self.guard
            || rect[2] - rect[0] <= 8. * self.guard
            || rect[3] - rect[1] <= 8. * self.guard
        {
            return Ok(None);
        }
        for height in [
            low + 4. * self.guard,
            (low + high) / 2.,
            high - 4. * self.guard,
        ] {
            let r = part
                .radius
                .at(height - self.guard)
                .min(part.radius.at(height + self.guard));
            let clamp = |x: f64, k: usize| {
                x.clamp(rect[k] + 4. * self.guard, rect[k + 2] - 4. * self.guard)
            };
            let near = [clamp(p[0], 0), clamp(p[1], 1)];
            let points = [
                near,
                [clamp(p[0] + r * 0.8, 0), near[1]],
                [clamp(p[0] - r * 0.8, 0), near[1]],
                [near[0], clamp(p[1] + r * 0.8, 1)],
                [near[0], clamp(p[1] - r * 0.8, 1)],
            ];
            for xy in points {
                let margin = (r - (xy[0] - p[0]).hypot(xy[1] - p[1]))
                    .min(signed_rect(xy, rect))
                    .min(height - low)
                    .min(high - height)
                    - self.guard;
                if margin > self.guard {
                    let point = [xy[0], xy[1], p[2] + height];
                    let radius = margin / 2.;
                    if self.uncut_ball(point, radius, t)? {
                        return Ok(Some((
                            Witness {
                                fraction: t,
                                tip_mm: p,
                                section_height_mm: Some(height),
                                margin_lower_mm: radius,
                            },
                            point,
                        )));
                    }
                }
            }
        }
        Ok(None)
    }
    fn finding(
        &self,
        part: &Part,
        task: Task,
        h: [f64; 2],
        witness: Option<(Witness, Vec3)>,
    ) -> ContactFinding {
        let violation = witness.is_some();
        let (witness, point) = witness.map_or((None, None), |(w, p)| (Some(w), Some(p)));
        ContactFinding {finding:Finding {status:if violation {Status::Violation}else{Status::Uncertain},kind:FindingKind::StockContact,object_id:"stock".into(),part:Some(part.kind),holder_section:part.holder,fraction_interval:task.span,witness,
            detail:if violation {"Strict lower-solid stock/profile overlap; witness neighborhood is outside all earlier cutting"} else {"Stock contact/evolution unresolved within configured bounds, resolution or work cap"}.into()},cell_range:task.rect,solid_lower_mm:h[0],possible_upper_mm:h[1],contact_point_mm:point}
    }
    fn part(&mut self, part: &Part) -> Result<Option<ContactFinding>, String> {
        let root = &self.index.nodes[0];
        let mut stack = vec![Task {
            rect: root.rect,
            node: 0,
            span: [0., 1.],
            depth: 0,
        }];
        let mut uncertain = None;
        let mut history_checked = false;
        while let Some(task) = stack.pop() {
            self.stats.max_stack_nodes = self.stats.max_stack_nodes.max(stack.len() + 1);
            if !self.charge()? {
                uncertain.get_or_insert(self.finding(
                    part,
                    task,
                    [self.stock.spec().min_mm[2], self.stock.spec().max_mm[2]],
                    None,
                ));
                break;
            }
            let Some(h) = self.heights(task) else {
                uncertain.get_or_insert(self.finding(
                    part,
                    task,
                    [self.stock.spec().min_mm[2], self.stock.spec().max_mm[2]],
                    None,
                ));
                break;
            };
            if h[1] <= self.stock.spec().min_mm[2] {
                self.stats.pruned_nodes += 1;
                continue;
            }
            let bounds = self.bounds(task.span)?;
            let rect = rect_bounds(self.stock, task.rect);
            let low = part
                .min
                .max(self.stock.spec().min_mm[2] - bounds.max[2] - self.guard);
            let high = part.max.min(h[1] - bounds.min[2] + self.guard);
            if low > high {
                self.stats.pruned_nodes += 1;
                continue;
            }
            let radius = part.radius.at(low).max(part.radius.at(high));
            let dx = (rect[0] - bounds.max[0])
                .max(bounds.min[0] - rect[2])
                .max(0.);
            let dy = (rect[1] - bounds.max[1])
                .max(bounds.min[1] - rect[3])
                .max(0.);
            if dx.hypot(dy) > radius + 2. * self.guard {
                self.stats.pruned_nodes += 1;
                continue;
            }
            let center = [(rect[0] + rect[2]) / 2., (rect[1] + rect[3]) / 2.];
            let extent = (rect[2] - rect[0]).hypot(rect[3] - rect[1]) / 2.;
            if self.axis_distance(task.span, center)? > extent + radius + 2. * self.guard {
                self.stats.pruned_nodes += 1;
                continue;
            }
            if !self.motion.kind.is_rapid() {
                let upper = self.earlier_upper(rect, h[1], task.span[0])?;
                if bounds.min[2] + part.min > upper + 2. * self.guard {
                    self.stats.pruned_nodes += 1;
                    continue;
                }
            }
            if let Some(w) = self.witness(part, task, rect, h[0])? {
                return Ok(Some(self.finding(part, task, h, Some(w))));
            }
            let children = if task.node != NONE {
                self.index.nodes[task.node].children
            } else {
                [NONE; 2]
            };
            if children[0] != NONE {
                for child in children.into_iter().rev() {
                    stack.push(Task {
                        node: child,
                        rect: self.index.nodes[child].rect,
                        ..task
                    });
                }
            } else if task.rect[2] - task.rect[0] > 1 || task.rect[3] - task.rect[1] > 1 {
                let axis = usize::from(task.rect[3] - task.rect[1] > task.rect[2] - task.rect[0]);
                let m = task.rect[axis] + (task.rect[axis + 2] - task.rect[axis]) / 2;
                let mut a = task;
                let mut b = task;
                a.node = NONE;
                b.node = NONE;
                a.rect[axis + 2] = m;
                b.rect[axis] = m;
                stack.push(b);
                stack.push(a);
            } else if !history_checked && part.kind != ToolPart::Cutter {
                // A whole-cell proof has priority. Only an unresolved leaf
                // needs the additional native-history proof, charged to node
                // work rather than stealing the height-cell read budget.
                history_checked = true;
                let radius = part.radius.at(part.min).max(part.radius.at(part.max));
                let mut work = self.options.max_nodes.saturating_sub(self.stats.nodes);
                let before = work;
                let clear = self.stock.repeated_clearance(
                    self.motion,
                    radius,
                    part.min,
                    &mut work,
                    self.cancel,
                )?;
                self.stats.nodes += before - work;
                if clear {
                    self.stats.removal_certificates += 1;
                    return Ok(None);
                }
                stack.push(task);
            } else if task.depth >= 48
                || self.path.length_mm() * (task.span[1] - task.span[0])
                    <= self.options.spatial_resolution_mm
            {
                uncertain.get_or_insert(self.finding(part, task, h, None));
            } else {
                let t = (task.span[0] + task.span[1]) / 2.;
                stack.push(Task {
                    span: [t, task.span[1]],
                    depth: task.depth + 1,
                    ..task
                });
                stack.push(Task {
                    span: [task.span[0], t],
                    depth: task.depth + 1,
                    ..task
                });
            }
        }
        Ok(uncertain)
    }
}

pub fn check(
    stock: &Stock,
    tool: &PreparedAssembly,
    motion: &Motion,
    options: Options,
    cancel: &impl CancelSignal,
) -> Result<Report, String> {
    let digests = Digests {
        options: verification::identity(&options)?,
        tool: verification::identity(tool.definition())?,
        motion: verification::motion_identity(motion),
    };
    check_digested(stock, tool, motion, options, &digests, cancel)
}

/// Canonical identities bound into a report; a stream computes the options and
/// tool identities once per run and the motion identity once per motion.
pub(crate) struct Digests {
    pub options: String,
    pub tool: String,
    pub motion: String,
}

pub(crate) fn check_digested(
    stock: &Stock,
    tool: &PreparedAssembly,
    motion: &Motion,
    options: Options,
    digests: &Digests,
    cancel: &impl CancelSignal,
) -> Result<Report, String> {
    options.validate()?;
    if motion.tool != tool.definition().id
        || !valid_id(&motion.operation)
        || motion.validate_feed(None).is_err()
    {
        return Err("invalid stock-contact motion attribution/feed/rapid".into());
    }
    let index = stock
        .contact_index()
        .ok_or("enable stock contact queries before checking")?;
    let path = motion.curve.prepare(options.tolerance_mm)?;
    let guard = verification::path_error(motion, options.tolerance_mm);
    let parts = verification::parts(tool)?;
    let selected = parts
        .iter()
        .filter(|p| motion.kind.is_rapid() || p.kind != ToolPart::Cutter)
        .collect::<Vec<_>>();
    let mut s = Search {
        stock,
        index,
        tool,
        motion,
        path,
        guard,
        options,
        stats: Stats::default(),
        cancel,
    };
    // A vertical move above the deepest committed plunge of this tool on the
    // same axis keeps its cutting part in certified empty space (a canned
    // cycle's rapid retracts and re-entries).
    let axial = match motion.curve {
        Curve::Line { start, end } if start[0] == end[0] && start[1] == end[1] => {
            stock.axial_clear(&motion.tool, [start[0], start[1]], start[2].min(end[2]))
                || (end[2] > start[2] && stock.departs_clear(&motion.tool, start))
        }
        _ => false,
    };
    // An oriented-spindle shift inside the bore its plunge cut: the
    // stopped bar lies within its bar (neck) diameter except the cutting
    // edge, which points back toward the bore axis (the motion validator
    // keeps every shift on the ray away from the edge). A part of radius r
    // at distance s from the axis stays inside the certified empty column
    // of the cutting radius R when r + s <= R; the edge reaches R - s.
    let bore = (motion.kind == MoveKind::Shift)
        .then(|| {
            let (start, end) = (motion.curve.start(), motion.curve.end());
            let (axis, tip) = stock.shift_bore(&motion.tool, start)?;
            let offset = |p: Vec3| (p[0] - axis[0]).hypot(p[1] - axis[1]);
            (start[2].min(end[2]) >= tip).then(|| offset(start).max(offset(end)))
        })
        .flatten();
    let cutter = &tool.definition().cutter;
    let bar = cutter
        .neck
        .as_ref()
        .map_or(cutter.diameter_mm, |n| n.diameter_mm)
        / 2.;
    let mut findings = Vec::new();
    for part in &selected {
        if findings.len() == options.max_findings {
            s.stats.findings_truncated = true;
            break;
        }
        if axial && part.kind == ToolPart::Cutter {
            s.stats.removal_certificates += 1;
            s.stats.parts_examined += 1;
            continue;
        }
        if let Some(offset) = bore {
            let radius = if part.kind == ToolPart::Cutter {
                bar
            } else {
                part.radius.at(part.min).max(part.radius.at(part.max))
            };
            if radius + offset <= cutter.diameter_mm / 2. - s.guard {
                s.stats.removal_certificates += 1;
                s.stats.parts_examined += 1;
                continue;
            }
        }
        if let Some(f) = s.part(part)? {
            findings.push(f);
        }
        s.stats.parts_examined += 1;
        if s.stats.work_exhausted {
            break;
        }
    }
    if cancel.is_cancelled() {
        return Err("stock contact cancelled; no report published".into());
    }
    s.stats.unexamined_parts = selected.len() - s.stats.parts_examined;
    let status = if findings
        .iter()
        .any(|f| f.finding.status == Status::Violation)
    {
        Status::Violation
    } else if !findings.is_empty() || s.stats.unexamined_parts > 0 || s.stats.work_exhausted {
        Status::Uncertain
    } else {
        Status::Clear
    };
    Ok(Report {algorithm:"continuous_stock_contact_v1".into(),stock:stock.spec().clone(),pre_generation:stock.generation(),options_sha256:digests.options.clone(),
        tool_sha256:digests.tool.clone(),motion_sha256:digests.motion.clone(),motion_id:motion.id,operation:motion.operation.clone(),tool:motion.tool.clone(),
        scope:match motion.kind {MoveKind::Rapid=>"rapid_assembly",MoveKind::Shift=>"oriented_shift",_=>"fed_noncutting"}.into(),status,fully_examined:s.stats.unexamined_parts==0&&!s.stats.work_exhausted,
        findings,stats:s.stats,numerical_policy:"Validated whole-cell stock premise; continuous interval/profile enclosures and prior full-footprint cutting certificates. Strict lower-solid witnesses require earlier-cut separation. Binary64 engineering guards, not interval certification or machine qualification.".into()})
}

#[cfg(test)]
mod index_update_tests {
    use super::*;

    #[test]
    fn sparse_updates_and_undo_match_full_rebuild_for_every_node() {
        // Partial edge tiles and a nonsquare tree exercise both split axes.
        let nx = 513;
        let ny = 385;
        let spec = StockSpec {
            version: 1,
            units: "mm".into(),
            min_mm: [0., 0., 0.],
            max_mm: [nx as f64, ny as f64, 10.],
            cell_size_mm: 1.,
            max_cells: nx * ny,
        };
        let top = HeightBounds {
            lower_mm: 10.,
            upper_mm: 10.,
        };
        let mut cells = vec![top; nx * ny];
        let mut index = HeightIndex::new(&spec, &cells, true, &|| false).unwrap();
        for positions in [
            vec![0],
            (0..16).map(|i| (nx * 17) + 32 + i).collect(),
            vec![nx - 1, nx * ny - 1, nx * (ny - 1), nx * (ny / 2) + nx / 2],
            (0..nx * ny).step_by(251).collect(),
        ] {
            let mut changes = Vec::new();
            for i in positions {
                let after = HeightBounds {
                    lower_mm: 2.,
                    upper_mm: 3. + (i % 7) as f64,
                };
                changes.push(CellChange {
                    index: i as u32,
                    before: cells[i],
                    after,
                });
                cells[i] = after;
            }
            for reverse in [false, true] {
                if reverse {
                    for c in &changes {
                        cells[c.index as usize] = c.before;
                    }
                }
                index.update(&cells, nx, &changes);
                let rebuilt = HeightIndex::new(&spec, &cells, false, &|| false).unwrap();
                for (a, b) in index.nodes.iter().zip(&rebuilt.nodes) {
                    assert_eq!((a.lower, a.upper), (b.lower, b.upper));
                }
                assert!(index.dirty.iter().all(|&d| !d));
                // Single-cell edits touch just the leaf and its ancestor path.
                if changes.len() == 1 {
                    assert_eq!(index.stats.changed_tiles, 1);
                    assert!(index.stats.updated_nodes <= 12);
                    assert!(index.stats.updated_nodes * 100 < index.nodes.len());
                    assert_eq!(index.stats.scanned_cells, TILE * TILE);
                }
            }
        }
        index.update(&cells, nx, &[]);
        assert_eq!(index.stats.updated_nodes, 0);
        assert_eq!(index.stats.scanned_cells, 0);
    }
}
