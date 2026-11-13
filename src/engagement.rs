//! Conditional, continuous flat-cutter engagement bounds from known empty sweeps.
//! Supports must describe the caller's committed cleared columns. This module
//! does not itself establish stock revision, source/fixture or machine clearance.
use crate::{
    Point,
    motion::{Curve, Motion, Plane, PreparedCurve},
    stock::{CancelSignal, StockSpec},
};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::f64::consts::{PI, TAU};

mod angular;
pub(crate) mod cover;

#[derive(Clone, Debug)]
enum Shape {
    Line {
        a: Point,
        b: Point,
        unit: Point,
        length: f64,
    },
    Arc {
        a: Point,
        b: Point,
        c: Point,
        radius: f64,
        angle: f64,
        sweep: f64,
    },
}
/// f64 words in an encoded [`ClearedSweep`] (`ClearedSweep::encode`).
pub(crate) const SWEEP_WORDS: usize = 16;
/// A geometric premise, NOT authority to mark arbitrary material cleared.
/// Only add a sweep after its matching flat-cutter stock transaction committed.
/// Its column must be clear from the declared floor through original stock top.
#[derive(Clone, Debug)]
pub struct ClearedSweep {
    shape: Shape,
    sector: Option<angular::ArcSector>,
    floor: f64,
    radius: f64,
    error: f64,
    bounds: [Point; 2],
}
fn flat(curve: &Curve) -> bool {
    curve.start()[2] == curve.end()[2]
        && matches!(
            curve,
            Curve::Line { .. }
                | Curve::Arc {
                    plane: Plane::Xy,
                    ..
                }
        )
}
/// Whether `r` is exactly on the segment `p`-`q`.
fn on_segment(p: Point, q: Point, r: Point) -> bool {
    robust::orient2d(
        robust::Coord { x: p[0], y: p[1] },
        robust::Coord { x: q[0], y: q[1] },
        robust::Coord { x: r[0], y: r[1] },
    ) == 0.
        && (0..2).all(|k| r[k] >= p[k].min(q[k]) && r[k] <= p[k].max(q[k]))
}
fn radius_ok(r: f64) -> bool {
    r.is_finite() && (1e-6..=1e6).contains(&r)
}
impl ClearedSweep {
    /// Exact reuse of a committed horizontal cutter footprint above its floor.
    /// Identical XY geometry shares the same path uncertainty, so no sampled
    /// or tolerance-based containment is inferred. Like an axial certificate,
    /// this includes an equal-radius body following the same cleared path.
    pub(crate) fn repeats(&self, curve: &Curve, radius: f64, bottom: f64) -> bool {
        if radius > self.radius || bottom < self.floor + self.error {
            return false;
        }
        let a = curve.start();
        let b = curve.end();
        let (a, b) = ([a[0], a[1]], [b[0], b[1]]);
        match (&self.shape, curve) {
            (Shape::Line { a: p, b: q, .. }, Curve::Line { .. }) => {
                on_segment(*p, *q, a) && on_segment(*p, *q, b)
            }
            (
                Shape::Arc {
                    a: p,
                    b: q,
                    c,
                    sweep,
                    ..
                },
                Curve::Arc {
                    center,
                    plane: Plane::Xy,
                    sweep_rad,
                    ..
                },
            ) => a == *p && b == *q && [center[0], center[1]] == *c && sweep_rad == sweep,
            _ => false,
        }
    }
    /// Whether `p` is exactly a point of this sweep's path: on a line's
    /// segment, or a declared end of an arc.
    fn on_path(&self, p: Point) -> bool {
        match self.shape {
            Shape::Line { a, b, .. } => on_segment(a, b, p),
            Shape::Arc { a, b, .. } => p == a || p == b,
        }
    }
    /// For a line sweep exactly collinear with the segment `a`-`b` (which
    /// is not a point): the sweep's extent in coordinate `k`, along which
    /// that segment is not constant. A point of the common line is on the
    /// sweep's path exactly when its coordinate `k` is in that extent.
    fn collinear_extent(&self, a: Point, b: Point, k: usize) -> Option<[f64; 2]> {
        let Shape::Line { a: p, b: q, .. } = self.shape else {
            return None;
        };
        let coord = |v: Point| robust::Coord { x: v[0], y: v[1] };
        (robust::orient2d(coord(p), coord(q), coord(a)) == 0.
            && robust::orient2d(coord(p), coord(q), coord(b)) == 0.)
            .then(|| [p[k].min(q[k]), p[k].max(q[k])])
    }
    /// The height from which this sweep's column is proven empty.
    pub(crate) fn clear_from(&self) -> f64 {
        self.floor + self.error
    }

    /// Stable geometry only; runtime identities and reissued motion IDs are
    /// not part of a clearance premise.
    pub(crate) fn fingerprint(&self) -> [u8; 32] {
        let mut hash = Sha256::new();
        hash.update(b"camlib-cleared-sweep-v1");
        for value in [
            self.floor,
            self.radius,
            self.error,
            self.bounds[0][0],
            self.bounds[0][1],
            self.bounds[1][0],
            self.bounds[1][1],
        ] {
            hash.update(value.to_le_bytes());
        }
        match &self.shape {
            Shape::Line { a, b, unit, length } => {
                hash.update([0]);
                for value in a.iter().chain(b).chain(unit).chain(std::iter::once(length)) {
                    hash.update(value.to_le_bytes());
                }
            }
            Shape::Arc {
                a,
                b,
                c,
                radius,
                angle,
                sweep,
            } => {
                hash.update([1]);
                for value in a.iter().chain(b).chain(c).chain([radius, angle, sweep]) {
                    hash.update(value.to_le_bytes());
                }
            }
        }
        hash.finalize().into()
    }

    /// Outer XY box of the cleared footprint (radius and path error included).
    pub(crate) fn bounds(&self) -> [Point; 2] {
        self.bounds
    }
    pub(crate) fn floor(&self) -> f64 {
        self.floor
    }
    /// Largest ball this sweep can ever contain (its witness extent).
    pub(crate) fn max_witness_extent(&self) -> f64 {
        match self.shape {
            Shape::Arc { radius, sweep, .. } if sweep.abs() >= TAU && radius <= self.radius => {
                self.radius + radius
            }
            _ => self.radius,
        }
    }
    /// The whole closed XY ball (`p`, `extent`) lies in this sweep's cleared
    /// column at every height from `z` up: the same certificate as the
    /// engagement scene's box coverage (`clear_box`).
    pub(crate) fn contains_ball(&self, p: Point, extent: f64, z: f64) -> bool {
        let numeric = 4096. * f64::EPSILON * (1. + p[0].abs() + p[1].abs() + extent);
        ball_in_support(self, p, extent, z, numeric)
    }

    /// Exact words of this sweep for a producer checkpoint (`decode` rebuilds
    /// the identical sweep, fingerprint included): tag (0 line, 1 arc), then
    /// [`SWEEP_WORDS`] f64 values.
    pub(crate) fn encode(&self) -> (u8, [f64; SWEEP_WORDS]) {
        let mut w = [0.; SWEEP_WORDS];
        w[..7].copy_from_slice(&[
            self.floor,
            self.radius,
            self.error,
            self.bounds[0][0],
            self.bounds[0][1],
            self.bounds[1][0],
            self.bounds[1][1],
        ]);
        let tag = match self.shape {
            Shape::Line { a, b, unit, length } => {
                w[7..14].copy_from_slice(&[a[0], a[1], b[0], b[1], unit[0], unit[1], length]);
                0
            }
            Shape::Arc {
                a,
                b,
                c,
                radius,
                angle,
                sweep,
            } => {
                w[7..16]
                    .copy_from_slice(&[a[0], a[1], b[0], b[1], c[0], c[1], radius, angle, sweep]);
                1
            }
        };
        (tag, w)
    }
    /// Rebuild a sweep from `encode`'s words, refusing any that no real
    /// motion could have produced: `tolerance_mm` is the tolerance the column
    /// was committed with (the same endpoint check `Curve::prepare` makes).
    pub(crate) fn decode(
        tag: u8,
        w: [f64; SWEEP_WORDS],
        tolerance_mm: f64,
    ) -> Result<Self, String> {
        let bad = || "invalid checkpoint cleared sweep".to_string();
        if w.iter().any(|v| !v.is_finite()) || !radius_ok(w[1]) || w[2] < 0. {
            return Err(bad());
        }
        let bounds = [[w[3], w[4]], [w[5], w[6]]];
        let inside = |p: [f64; 2]| (0..2).all(|i| bounds[0][i] <= p[i] && p[i] <= bounds[1][i]);
        let near = |p: [f64; 2], q: [f64; 2]| distance(p, q) <= tolerance_mm;
        let shape = match tag {
            0 => {
                let (a, b, unit, length) = ([w[7], w[8]], [w[9], w[10]], [w[11], w[12]], w[13]);
                // A line has XY length, its unit direction runs from a to b,
                // and its footprint box holds both ends.
                let d = [b[0] - a[0], b[1] - a[1]];
                if length <= 0.
                    || (d[0].hypot(d[1]) - length).abs() > 1e-9 * (1. + length)
                    || (unit[0] * length - d[0]).hypot(unit[1] * length - d[1])
                        > 1e-9 * (1. + length)
                    || !inside(a)
                    || !inside(b)
                {
                    return Err(bad());
                }
                Shape::Line { a, b, unit, length }
            }
            1 => {
                let (a, b, c) = ([w[7], w[8]], [w[9], w[10]], [w[11], w[12]]);
                let (radius, angle, sweep) = (w[13], w[14], w[15]);
                // An arc has a radius, a start on its circle at `angle`, a
                // nonzero sweep, and an end where the sweep leads (within the
                // tolerance the motion was accepted with).
                let end = [
                    c[0] + radius * (angle + sweep).cos(),
                    c[1] + radius * (angle + sweep).sin(),
                ];
                if radius < 1e-6
                    || sweep == 0.
                    || (distance(a, c) - radius).abs() > 1e-9 * (1. + radius)
                    || (angle.cos() * radius - (a[0] - c[0])).abs() > 1e-9 * (1. + radius)
                    || (angle.sin() * radius - (a[1] - c[1])).abs() > 1e-9 * (1. + radius)
                    || !near(end, b)
                    || !inside(a)
                    || !inside(b)
                {
                    return Err(bad());
                }
                Shape::Arc {
                    a,
                    b,
                    c,
                    radius,
                    angle,
                    sweep,
                }
            }
            _ => return Err(bad()),
        };
        let sector = match shape {
            Shape::Arc { angle, sweep, .. } if sweep.abs() < TAU => {
                Some(angular::ArcSector::new(angle, sweep))
            }
            _ => None,
        };
        Ok(Self {
            shape,
            sector,
            floor: w[0],
            radius: w[1],
            error: w[2],
            bounds: [[w[3], w[4]], [w[5], w[6]]],
        })
    }

    pub fn from_motion(motion: &Motion, radius: f64, tolerance_mm: f64) -> Result<Self, String> {
        if !flat(&motion.curve) || !radius_ok(radius) {
            return Err(
                "engagement support requires a flat horizontal line/XY arc and finite radius"
                    .into(),
            );
        }
        let path = motion.curve.prepare(tolerance_mm)?;
        let a3 = motion.curve.start();
        let b3 = motion.curve.end();
        let (a, b) = ([a3[0], a3[1]], [b3[0], b3[1]]);
        let shape = match motion.curve {
            Curve::Line { .. } => {
                let d = [b[0] - a[0], b[1] - a[1]];
                let length = d[0].hypot(d[1]);
                if length == 0. {
                    return Err("engagement support has zero XY length".into());
                }
                Shape::Line {
                    a,
                    b,
                    unit: [d[0] / length, d[1] / length],
                    length,
                }
            }
            Curve::Arc {
                center, sweep_rad, ..
            } => {
                let c = [center[0], center[1]];
                Shape::Arc {
                    a,
                    b,
                    c,
                    radius: distance(a, c),
                    angle: (a[1] - c[1]).atan2(a[0] - c[0]),
                    sweep: sweep_rad,
                }
            }
            _ => unreachable!(),
        };
        let error = crate::verification::path_error(motion, tolerance_mm);
        let bounds = path.bounds();
        let sector = match shape {
            Shape::Arc { angle, sweep, .. } if sweep.abs() < TAU => {
                Some(angular::ArcSector::new(angle, sweep))
            }
            _ => None,
        };
        Ok(Self {
            shape,
            sector,
            floor: a3[2],
            radius,
            error,
            bounds: [
                [
                    bounds.min[0] - radius - error,
                    bounds.min[1] - radius - error,
                ],
                [
                    bounds.max[0] + radius + error,
                    bounds.max[1] + radius + error,
                ],
            ],
        })
    }
}
#[derive(Clone, Copy, Debug, Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EngagementOptions {
    pub tolerance_mm: f64,
    pub resolution_mm: f64,
    pub max_depth: u8,
    pub max_nodes: u64,
    /// Shared cap for broad-phase index visits and primitive tests.
    pub max_work: u64,
}
impl EngagementOptions {
    pub fn validate(self) -> Result<(), String> {
        if !self.tolerance_mm.is_finite()
            || !(1e-7..=0.01).contains(&self.tolerance_mm)
            || !self.resolution_mm.is_finite()
            || !(self.tolerance_mm..=1.).contains(&self.resolution_mm)
            || !(1..=32).contains(&self.max_depth)
            || self.max_nodes > 1_000_000
            || self.max_work > 100_000_000
        {
            return Err("invalid flat-cutter engagement query/options".into());
        }
        Ok(())
    }
}
impl Default for EngagementOptions {
    fn default() -> Self {
        Self {
            tolerance_mm: 1e-5,
            resolution_mm: 0.001,
            max_depth: 24,
            max_nodes: 100_000,
            max_work: 20_000_000,
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum EngagementStatus {
    Bounded,
    Unresolved,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum UnresolvedReason {
    Work,
    Resolution,
    Depth,
}
#[derive(Clone, Debug, Serialize)]
pub struct EngagementReport {
    pub status: EngagementStatus,
    pub upper_angle_deg: f64,
    pub nodes: u64,
    pub support_tests: u64,
    pub index_visits: u64,
    pub maximum_depth: u8,
    pub reason: Option<UnresolvedReason>,
}
/// Immutable geometric scene. The future stock-front planner must additionally
/// bind this premise set to its stock specification/revision/committed prefix.
pub struct EngagementScene {
    stock: StockSpec,
    rectangular_shells: Vec<(f64, [Point; 2])>,
    supports: Vec<ClearedSweep>,
    index: Vec<IndexNode>,
    roots: Vec<usize>,
}
pub(crate) struct SceneMark {
    supports: usize,
    index: usize,
    roots: Vec<usize>,
}
struct IndexNode {
    bounds: [Point; 2],
    /// Lowest removal floor in this entire subtree. Higher-only history can
    /// never supply an empty-column premise for a deeper pass.
    min_floor: f64,
    // Upper bound on the extent accepted by ANY single-ball witness below.
    // This is a rejection bound only, never a union/clearance certificate.
    max_witness_extent: f64,
    children: Option<[usize; 2]>,
    support: Option<usize>,
    size: usize,
}
// At most2^18 leaves: the binary forest has at most18 roots and depth18.
// Forty pending nodes bound every depth-first traversal without heap allocation.
// Keep the original newest-first order, including budget/cancellation semantics.
struct IndexStack {
    nodes: [usize; 40],
    len: usize,
}
impl IndexStack {
    fn new(roots: &[usize]) -> Self {
        let mut result = Self {
            nodes: [0; 40],
            len: roots.len(),
        };
        result.nodes[..roots.len()].copy_from_slice(roots);
        result
    }
    fn pop(&mut self) -> Option<usize> {
        if self.len == 0 {
            return None;
        }
        self.len -= 1;
        Some(self.nodes[self.len])
    }
    fn extend(&mut self, children: [usize; 2]) {
        self.nodes[self.len..self.len + 2].copy_from_slice(&children);
        self.len += 2;
    }
}

#[derive(Clone, Copy)]
enum Metric {
    Total,
    /// Contact on the leading side only: the trailing side is the cutter's
    /// own swept trail (see `trail_sector`).
    Leading,
    WrongSide {
        material_on_right: bool,
    },
    Front {
        material_on_right: bool,
    },
}

impl EngagementScene {
    pub fn new(stock: StockSpec, supports: Vec<ClearedSweep>) -> Result<Self, String> {
        Self::new_with_cancel(stock, supports, &|| false)
    }
    pub fn new_with_cancel(
        stock: StockSpec,
        supports: Vec<ClearedSweep>,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        stock.grid()?;
        if supports.len() > 262_144 {
            return Err("engagement support count exceeds 262144".into());
        }
        let mut scene = Self {
            stock,
            rectangular_shells: vec![],
            supports: vec![],
            index: vec![],
            roots: vec![],
        };
        for support in supports {
            if cancel.is_cancelled() {
                return Err("engagement scene cancelled".into());
            }
            scene.append(support)?;
        }
        if cancel.is_cancelled() {
            return Err("engagement scene cancelled".into());
        }
        Ok(scene)
    }
    /// The scene's current extent, to return to with `restore`: supports are
    /// only ever appended, and index nodes are immutable once written.
    pub(crate) fn mark(&self) -> SceneMark {
        SceneMark {
            supports: self.supports.len(),
            index: self.index.len(),
            roots: self.roots.clone(),
        }
    }
    /// Drop every support appended since `mark` (premises that were only
    /// planned, never committed).
    pub(crate) fn restore(&mut self, mark: SceneMark) {
        self.supports.truncate(mark.supports);
        self.index.truncate(mark.index);
        self.roots = mark.roots;
    }
    pub(crate) fn stock_spec(&self) -> &StockSpec {
        &self.stock
    }
    /// Conditional geometric premise, just like a ClearedSweep. The planner
    /// must bind a completed shell recipe to actual full native stock commits.
    /// Original StockSpec and its physical height cells are never changed.
    pub(crate) fn append_rectangular_shell(
        &mut self,
        bounds: [Point; 2],
        floor: f64,
    ) -> Result<(), String> {
        let old = self.material_bounds(floor);
        if !floor.is_finite()
            || bounds.iter().flatten().any(|v| !v.is_finite())
            || (0..2).any(|k| {
                bounds[0][k] >= bounds[1][k] || bounds[0][k] < old[0][k] || bounds[1][k] > old[1][k]
            })
        {
            return Err("invalid or non-nested committed rectangular shell".into());
        }
        if let Some((_, previous)) = self
            .rectangular_shells
            .iter_mut()
            .find(|(z, _)| *z == floor)
        {
            *previous = bounds;
        } else {
            if self.rectangular_shells.len() >= 4096 {
                return Err("rectangular shell floor limit exceeded".into());
            }
            self.rectangular_shells.push((floor, bounds));
        }
        Ok(())
    }
    pub(crate) fn material_bounds(&self, z: f64) -> [Point; 2] {
        let mut result = [
            [self.stock.min_mm[0], self.stock.min_mm[1]],
            [self.stock.max_mm[0], self.stock.max_mm[1]],
        ];
        for (floor, bounds) in &self.rectangular_shells {
            if *floor <= z {
                for k in 0..2 {
                    result[0][k] = result[0][k].max(bounds[0][k]);
                    result[1][k] = result[1][k].min(bounds[1][k]);
                }
            }
        }
        result
    }
    /// Append a validated conditional premise in O(log N) bounded work. A
    /// binary forest keeps balanced immutable AABB subtrees without rebuilding
    /// the whole swept history at each committed front advance.
    pub fn append(&mut self, support: ClearedSweep) -> Result<(), String> {
        if self.supports.len() == 262_144 {
            return Err("engagement support count exceeds 262144".into());
        }
        let id = self.supports.len();
        let mut node = self.index.len();
        let max_witness_extent = support.max_witness_extent();
        self.index.push(IndexNode {
            bounds: support.bounds,
            min_floor: support.floor,
            max_witness_extent,
            children: None,
            support: Some(id),
            size: 1,
        });
        self.supports.push(support);
        while self
            .roots
            .last()
            .is_some_and(|i| self.index[*i].size == self.index[node].size)
        {
            let left = self.roots.pop().unwrap();
            let a = &self.index[left];
            let b = &self.index[node];
            let bounds = [
                std::array::from_fn(|k| a.bounds[0][k].min(b.bounds[0][k])),
                std::array::from_fn(|k| a.bounds[1][k].max(b.bounds[1][k])),
            ];
            let size = a.size + b.size;
            let min_floor = a.min_floor.min(b.min_floor);
            let max_witness_extent = a.max_witness_extent.max(b.max_witness_extent);
            let parent = self.index.len();
            self.index.push(IndexNode {
                bounds,
                min_floor,
                max_witness_extent,
                children: Some([left, node]),
                support: None,
                size,
            });
            node = parent;
        }
        self.roots.push(node);
        Ok(())
    }
    pub fn check(
        &self,
        motion: &Motion,
        radius: f64,
        max_angle_deg: f64,
        options: EngagementOptions,
        cancel: &impl CancelSignal,
    ) -> Result<EngagementReport, String> {
        self.check_metric(
            motion,
            radius,
            max_angle_deg,
            options,
            Metric::Total,
            cancel,
        )
    }
    /// The cutter's engagement while it advances along `motion`: an upper
    /// bound on the contact angle that excludes the cutter's own trail.
    ///
    /// `check` bounds contact with everything the committed supports have
    /// not cleared, which counts material this same motion has just removed
    /// behind the cutter: in a straight pass of radial width w it reports
    /// 2 acos(1 - w/r), twice the engagement. Here the trailing sector is
    /// known clear for every pose after the motion's start (`trail_sector`),
    /// so the bound is the engagement itself, acos(1 - w/r) in that pass.
    /// The start pose itself is the previous motion's end pose and is covered
    /// by that motion's bound.
    pub fn check_leading(
        &self,
        motion: &Motion,
        radius: f64,
        max_angle_deg: f64,
        options: EngagementOptions,
        cancel: &impl CancelSignal,
    ) -> Result<EngagementReport, String> {
        self.check_metric(
            motion,
            radius,
            max_angle_deg,
            options,
            Metric::Leading,
            cancel,
        )
    }
    /// Prove the opposite half of the cutter circumference empty relative to
    /// instantaneous feed over the WHOLE motion. CW spindle climb has material
    /// on the right. This is separate from the total-contact-angle bound.
    pub fn check_material_side(
        &self,
        motion: &Motion,
        radius: f64,
        material_on_right: bool,
        options: EngagementOptions,
        cancel: &impl CancelSignal,
    ) -> Result<EngagementReport, String> {
        self.check_metric(
            motion,
            radius,
            0.,
            options,
            Metric::WrongSide { material_on_right },
            cancel,
        )
    }
    /// Joint continuous front admission. One interval tree shares all geometric
    /// angular unions; success requires BOTH total contact and the entire
    /// instantaneous forbidden half to be bounded throughout the whole motion.
    pub(crate) fn check_front(
        &self,
        motion: &Motion,
        radius: f64,
        max_angle_deg: f64,
        material_on_right: bool,
        options: EngagementOptions,
        cancel: &impl CancelSignal,
    ) -> Result<EngagementReport, String> {
        self.check_metric(
            motion,
            radius,
            max_angle_deg,
            options,
            Metric::Front { material_on_right },
            cancel,
        )
    }
    /// Heuristic for candidate ranking ONLY. Point queries are not sweep,
    /// stock, productivity or remaining-material certificates.
    pub(crate) fn point_near_cleared_sweep(
        &self,
        p: Point,
        z: f64,
        ranking_margin: f64,
        remaining_work: &mut u64,
        cancel: &impl CancelSignal,
    ) -> Result<Option<bool>, String> {
        if cancel.is_cancelled() {
            return Err("stock-front ranking cancelled".into());
        }
        let material = self.material_bounds(z);
        if (0..2).any(|k| p[k] < material[0][k] || p[k] > material[1][k]) {
            return Ok(Some(true));
        }
        let mut stack = IndexStack::new(&self.roots);
        while let Some(id) = stack.pop() {
            if cancel.is_cancelled() {
                return Err("stock-front ranking cancelled".into());
            }
            if *remaining_work == 0 {
                return Ok(None);
            }
            *remaining_work -= 1;
            let node = &self.index[id];
            if node.min_floor > z {
                continue;
            }
            if (0..2).any(|k| {
                p[k] < node.bounds[0][k] - ranking_margin
                    || p[k] > node.bounds[1][k] + ranking_margin
            }) {
                continue;
            }
            if let Some(children) = node.children {
                stack.extend(children);
                continue;
            }
            let s = &self.supports[node.support.unwrap()];
            if s.floor > z {
                continue;
            }
            let reach = s.radius + s.error + ranking_margin;
            if !certainly_farther(&s.shape, p, reach) && shape_distance(&s.shape, p) <= reach {
                return Ok(Some(true));
            }
        }
        Ok(Some(false))
    }
    /// Candidate poses near the recent cleared front. Geometric ranking ONLY:
    /// a point on/near a committed axis is not a padded empty disk certificate.
    /// The caller must still prove source, full disk, engagement and side.
    pub(crate) fn reentry_positions(
        &self,
        goal: Point,
        floor: f64,
        inset: f64,
        material_on_right: bool,
        work: &mut u64,
        cancel: &impl CancelSignal,
    ) -> Result<Vec<(Point, f64)>, String> {
        if cancel.is_cancelled() {
            return Err("stock-front history search cancelled".into());
        }
        let mut positions = Vec::new();
        for support in self.supports.iter().rev().take(128) {
            if cancel.is_cancelled() {
                return Err("stock-front history search cancelled".into());
            }
            if *work == 0 {
                return Err("stock-front history search work limit; coverage unproven".into());
            }
            *work -= 1;
            if support.floor > floor {
                continue;
            }
            let poses = match support.shape {
                Shape::Line {
                    a, unit, length, ..
                } => {
                    let t = dot([goal[0] - a[0], goal[1] - a[1]], unit).clamp(0., length);
                    [t, length / 2.].map(|t| {
                        (
                            [a[0] + t * unit[0], a[1] + t * unit[1]],
                            unit[1].atan2(unit[0]),
                        )
                    })
                }
                Shape::Arc {
                    a,
                    b,
                    c,
                    radius,
                    angle,
                    sweep,
                } => {
                    let theta = (goal[1] - c[1]).atan2(goal[0] - c[0]);
                    let travel = if sweep > 0. {
                        angular::wrap_tau(theta - angle)
                    } else {
                        angular::wrap_tau(angle - theta)
                    };
                    let closest = if travel <= sweep.abs() {
                        travel * sweep.signum()
                    } else if distance(goal, a) < distance(goal, b) {
                        0.
                    } else {
                        sweep
                    };
                    [closest, sweep / 2.].map(|travel| {
                        let theta = angle + travel;
                        (
                            [c[0] + radius * theta.cos(), c[1] + radius * theta.sin()],
                            theta + sweep.signum() * PI / 2.,
                        )
                    })
                }
            };
            let side = if material_on_right { 1. } else { -1. };
            for (p, heading) in poses {
                // Nudge toward the cleared side. Exact path points alone may
                // only fit the original radius, not a guarded descent disk.
                positions.push((
                    [
                        p[0] - side * inset * heading.sin(),
                        p[1] + side * inset * heading.cos(),
                    ],
                    heading,
                ));
            }
        }
        Ok(positions)
    }
    /// Whole closed XY box covered by committed empty columns or outside
    /// original stock. A centre-distance plus half-diagonal bound encloses ALL
    /// points. Reuses the swept-history index; no endpoint/point-sample proof.
    pub(crate) fn clear_box(
        &self,
        bounds: [Point; 2],
        z: f64,
        work: &mut u64,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("coverage.scene_box");
        let Some((p, extent)) = self.box_ball(bounds, z, cancel)? else {
            return Ok(true);
        };
        self.clear_ball_witness(p, extent, z, work, cancel)
    }
    /// A remembered index is only a search hint. Every use rechecks the
    /// current support geometry, floor and complete query ball; even a hint
    /// from another scene cannot manufacture a clearance premise.
    pub(crate) fn clear_box_with_hint(
        &self,
        bounds: [Point; 2],
        z: f64,
        hint: &mut Option<usize>,
        work: &mut u64,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("coverage.scene_box");
        let Some((p, extent)) = self.box_ball(bounds, z, cancel)? else {
            return Ok(true);
        };
        let numeric = 4096. * f64::EPSILON * (1. + p[0].abs() + p[1].abs() + extent);
        if let Some(support) = hint.and_then(|i| self.supports.get(i)) {
            if *work == 0 {
                return Err("stock-front coverage work limit; coverage unproven".into());
            }
            *work -= 1;
            if ball_in_support(support, p, extent, z, numeric) {
                return Ok(true);
            }
        }
        if let Some(index) = self.find_ball_witness(p, extent, z, work, cancel)? {
            *hint = Some(index);
            Ok(true)
        } else {
            Ok(false)
        }
    }
    fn box_ball(
        &self,
        bounds: [Point; 2],
        z: f64,
        cancel: &impl CancelSignal,
    ) -> Result<Option<(Point, f64)>, String> {
        if cancel.is_cancelled() {
            return Err("stock-front coverage cancelled".into());
        }
        if z >= self.stock.max_mm[2] {
            return Ok(None);
        }
        let material = self.material_bounds(z);
        let lo: Point = std::array::from_fn(|k| bounds[0][k].max(material[0][k]));
        let hi: Point = std::array::from_fn(|k| bounds[1][k].min(material[1][k]));
        if (0..2).any(|k| lo[k] > hi[k]) {
            return Ok(None);
        }
        Ok(Some((
            std::array::from_fn(|k| (lo[k] + hi[k]) / 2.),
            distance(lo, hi) / 2.,
        )))
    }
    fn clear_ball_witness(
        &self,
        p: Point,
        extent: f64,
        z: f64,
        work: &mut u64,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        Ok(self
            .find_ball_witness(p, extent, z, work, cancel)?
            .is_some())
    }
    fn find_ball_witness(
        &self,
        p: Point,
        extent: f64,
        z: f64,
        work: &mut u64,
        cancel: &impl CancelSignal,
    ) -> Result<Option<usize>, String> {
        let numeric = 4096. * f64::EPSILON * (1. + p[0].abs() + p[1].abs() + extent);
        let mut stack = IndexStack::new(&self.roots);
        while let Some(id) = stack.pop() {
            if cancel.is_cancelled() {
                return Err("stock-front coverage cancelled".into());
            }
            if *work == 0 {
                return Err("stock-front coverage work limit; coverage unproven".into());
            }
            *work -= 1;
            let node = &self.index[id];
            if node.min_floor > z || extent > node.max_witness_extent + numeric {
                continue;
            }
            if (0..2).any(|k| p[k] < node.bounds[0][k] || p[k] > node.bounds[1][k]) {
                continue;
            }
            // A single sweep can enclose this ball only if its outer box
            // contains the entire ball. Parent boxes enclose every descendant.
            // Padding only retains uncertain candidates; it never proves clear.
            if (0..2).any(|k| {
                p[k] - extent + numeric < node.bounds[0][k]
                    || p[k] + extent - numeric > node.bounds[1][k]
            }) {
                continue;
            }
            if let Some(children) = node.children {
                stack.extend(children);
                continue;
            }
            let index = node.support.unwrap();
            if ball_in_support(&self.supports[index], p, extent, z, numeric) {
                return Ok(Some(index));
            }
        }
        Ok(None)
    }
    /// Full disk containment for a noncutting re-entry. Uncertain boundaries
    /// or insufficient work reject re-entry; no peripheral-angle shortcut.
    pub(crate) fn clear_disk(
        &self,
        p: Point,
        r: f64,
        z: f64,
        tolerance: f64,
        work: &mut u64,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("engagement.descent");
        if self.clear_ball_witness(p, r, z, work, cancel)? {
            return Ok(true);
        }
        if self.clear_clipped_disk_witness(p, r, z, work, cancel)? {
            return Ok(true);
        }
        if !self.disk_may_be_clear(p, r, z, work, cancel)? {
            return Ok(false);
        }
        let mut stack = vec![([[p[0] - r, p[1] - r], [p[0] + r, p[1] + r]], 0u8)];
        while let Some((b, depth)) = stack.pop() {
            if cancel.is_cancelled() {
                return Err("stock-front re-entry cancelled".into());
            }
            if *work == 0 {
                return Err("stock-front re-entry work limit; coverage unproven".into());
            }
            *work -= 1;
            let nearest: Point = std::array::from_fn(|k| p[k].clamp(b[0][k], b[1][k]));
            if distance(nearest, p) > r {
                continue;
            }
            if self.clear_box(b, z, work, cancel)? {
                continue;
            }
            let mid: Point = std::array::from_fn(|k| (b[0][k] + b[1][k]) / 2.);
            if depth >= 24 || distance(b[0], b[1]) <= 32. * tolerance {
                return Ok(false);
            }
            for q in 0..4 {
                stack.push((
                    [
                        std::array::from_fn(|k| if q & (1 << k) == 0 { b[0][k] } else { mid[k] }),
                        std::array::from_fn(|k| if q & (1 << k) == 0 { mid[k] } else { b[1][k] }),
                    ],
                    depth + 1,
                ));
            }
        }
        Ok(true)
    }
    /// Whole horizontal capsule containment in committed empty columns.
    /// Tiles outside a Lipschitz enclosure of the axis are discarded; every
    /// retained tile must have a full cleared-box witness. Endpoint disks are
    /// rejection/fast proofs only, never a substitute for the intervening tube.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn clear_line(
        &self,
        a: Point,
        b: Point,
        radius: f64,
        z: f64,
        tolerance: f64,
        work: &mut u64,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        for p in [a, b] {
            if !self.clear_disk(p, radius, z, tolerance, work, cancel)? {
                return Ok(false);
            }
        }
        let d = [b[0] - a[0], b[1] - a[1]];
        let length = d[0].hypot(d[1]);
        if length == 0. {
            return Ok(true);
        }
        let unit = d.map(|v| v / length);
        let lo = std::array::from_fn(|k| a[k].min(b[k]) - radius);
        let hi = std::array::from_fn(|k| a[k].max(b[k]) + radius);
        let mut stack = vec![([lo, hi], 0u8)];
        while let Some((bounds, depth)) = stack.pop() {
            if cancel.is_cancelled() {
                return Err("stock-front clear link cancelled".into());
            }
            if *work == 0 {
                return Err("stock-front clear link work limit; clearance unproven".into());
            }
            *work -= 1;
            let mid: Point = std::array::from_fn(|k| (bounds[0][k] + bounds[1][k]) / 2.);
            let extent = distance(bounds[0], bounds[1]) / 2.;
            let t = dot([mid[0] - a[0], mid[1] - a[1]], unit).clamp(0., length);
            let nearest = [a[0] + t * unit[0], a[1] + t * unit[1]];
            let numeric = 4096.
                * f64::EPSILON
                * (1. + a[0].abs() + a[1].abs() + b[0].abs() + b[1].abs() + radius);
            if distance(mid, nearest) > radius + extent + numeric {
                continue;
            }
            if self.clear_box(bounds, z, work, cancel)? {
                continue;
            }
            if depth >= 24 || extent <= 16. * tolerance {
                return Ok(false);
            }
            for q in 0..4 {
                stack.push((
                    [
                        std::array::from_fn(|k| {
                            if q & (1 << k) == 0 {
                                bounds[0][k]
                            } else {
                                mid[k]
                            }
                        }),
                        std::array::from_fn(|k| {
                            if q & (1 << k) == 0 {
                                mid[k]
                            } else {
                                bounds[1][k]
                            }
                        }),
                    ],
                    depth + 1,
                ));
            }
        }
        Ok(true)
    }
    /// Cheap rejection only. A positive result does not prove disk clearance;
    /// callers must still use clear_disk before admitting a descent.
    pub(crate) fn disk_may_be_clear(
        &self,
        p: Point,
        r: f64,
        z: f64,
        work: &mut u64,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        // Necessary condition only: a whole disk cannot be certified from
        // these premises if even one INTERIOR point is outside their expanded
        // union. Passing these probes proves nothing; the whole-region proof
        // in clear_disk remains mandatory. This avoids subdividing unsuitable
        // descent disks until their numerical boundary budget is exhausted.
        let margin = 4096. * f64::EPSILON * (1. + p[0].abs() + p[1].abs() + r);
        for i in 0..=16 {
            let q = if i == 16 {
                p
            } else {
                let angle = TAU * i as f64 / 16.;
                [p[0] + 0.99 * r * angle.cos(), p[1] + 0.99 * r * angle.sin()]
            };
            match self.point_near_cleared_sweep(q, z, margin, work, cancel)? {
                Some(false) => return Ok(false),
                Some(true) => {}
                None => {
                    return Err(
                        "stock-front re-entry point rejection work limit; coverage unproven".into(),
                    );
                }
            }
        }
        Ok(true)
    }
    /// Convex containment of Disk intersect original StockXY. Its boundary is
    /// circle arcs and rectangle segments. Distance to a capsule axis (or a
    /// filled-disk centre) is convex, so analytic boundary extrema enclose ALL
    /// interior points. This shortcut never applies to a nonconvex arc tube.
    fn clear_clipped_disk_witness(
        &self,
        p: Point,
        r: f64,
        z: f64,
        work: &mut u64,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        if cancel.is_cancelled() {
            return Err("stock-front disk containment cancelled".into());
        }
        if *work == 0 {
            return Err("stock-front disk containment work limit; coverage unproven".into());
        }
        *work -= 1;
        let stock = self.material_bounds(z);
        if (0..2).any(|k| stock[0][k] > stock[1][k]) {
            return Ok(true);
        }
        let nearest: Point = std::array::from_fn(|k| p[k].clamp(stock[0][k], stock[1][k]));
        let outside_guard = 4096.
            * f64::EPSILON
            * (1. + p[0].abs() + p[1].abs() + nearest[0].abs() + nearest[1].abs() + r);
        if distance(p, nearest) - outside_guard > r {
            return Ok(true);
        }
        let mut stack = IndexStack::new(&self.roots);
        while let Some(id) = stack.pop() {
            if cancel.is_cancelled() {
                return Err("stock-front disk containment cancelled".into());
            }
            if *work == 0 {
                return Err("stock-front disk containment work limit; coverage unproven".into());
            }
            *work -= 1;
            let node = &self.index[id];
            // nearest belongs to the query intersection: a covering support
            // must contain it. Outside bounds may reject a witness, not clear.
            if node.min_floor > z
                || (0..2).any(|k| nearest[k] < node.bounds[0][k] || nearest[k] > node.bounds[1][k])
            {
                continue;
            }
            if let Some(children) = node.children {
                stack.extend(children);
                continue;
            }
            let s = &self.supports[node.support.unwrap()];
            if s.floor > z {
                continue;
            }
            let scale = 1. + p[0].abs() + p[1].abs() + r;
            let h = s.radius - s.error - 4096. * f64::EPSILON * scale;
            let (a, b, unit, length, limit) = match s.shape {
                Shape::Line { a, b, unit, length } => (a, b, unit, length, h),
                Shape::Arc {
                    c, radius, sweep, ..
                } if sweep.abs() >= TAU && radius <= h => (c, c, [1., 0.], 0., radius + h),
                _ => continue,
            };
            if let Some(maximum) = clipped_disk_segment_max(p, r, stock, a, b, unit, length)
                && maximum <= limit
            {
                return Ok(true);
            }
        }
        Ok(false)
    }
    fn check_metric(
        &self,
        motion: &Motion,
        radius: f64,
        max_angle_deg: f64,
        options: EngagementOptions,
        metric: Metric,
        cancel: &impl CancelSignal,
    ) -> Result<EngagementReport, String> {
        if matches!(metric, Metric::Front { .. }) {
            self.check_metric_mode::<true, true>(
                motion,
                radius,
                max_angle_deg,
                options,
                metric,
                cancel,
            )
        } else {
            // Public angle inspection retains its full-history report; this
            // optimization answers only the planner's joint admission question.
            self.check_metric_mode::<false, true>(
                motion,
                radius,
                max_angle_deg,
                options,
                metric,
                cancel,
            )
        }
    }
    fn check_metric_mode<const EARLY_UNION: bool, const REFUTE: bool>(
        &self,
        motion: &Motion,
        radius: f64,
        max_angle_deg: f64,
        options: EngagementOptions,
        metric: Metric,
        cancel: &impl CancelSignal,
    ) -> Result<EngagementReport, String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span(match metric {
            Metric::Total => "engagement.total",
            Metric::Leading => "engagement.leading",
            Metric::WrongSide { .. } => "engagement.side",
            Metric::Front { .. } => "engagement.front",
        });
        if cancel.is_cancelled() {
            return Err("engagement query cancelled".into());
        }
        options.validate()?;
        if !flat(&motion.curve)
            || !radius_ok(radius)
            || !max_angle_deg.is_finite()
            || !(0. ..=360.).contains(&max_angle_deg)
        {
            return Err("invalid flat-cutter engagement query/options".into());
        }
        let path = motion.curve.prepare(options.tolerance_mm)?;
        let error = crate::verification::path_error(motion, options.tolerance_mm);
        let b = path.bounds();
        let mut report = EngagementReport {
            status: EngagementStatus::Bounded,
            upper_angle_deg: 0.,
            nodes: 0,
            support_tests: 0,
            index_visits: 0,
            maximum_depth: 0,
            reason: None,
        };
        let material = self.material_bounds(motion.curve.start()[2]);
        if b.min[2] - error > self.stock.max_mm[2]
            || (0..2).any(|k| {
                material[0][k] > material[1][k]
                    || b.max[k] + radius + error < material[0][k]
                    || b.min[k] - radius - error > material[1][k]
            })
        {
            return Ok(report);
        }
        // Only a root certificate may be reused. It covers every position of
        // this one motion; pose rejection and subdivisions cannot populate it.
        let mut whole_side_proved = false;
        let mut stack = vec![(0., 1., 0u8)];
        while let Some((a, b, depth)) = stack.pop() {
            if cancel.is_cancelled() {
                return Err("engagement query cancelled".into());
            }
            if report.nodes >= options.max_nodes {
                return Ok(unresolved(report, UnresolvedReason::Work));
            }
            report.nodes += 1;
            report.maximum_depth = report.maximum_depth.max(depth);
            let span = Span {
                motion,
                path: &path,
                a,
                b,
                error,
            };
            let Some(upper) = self.angular_bound::<EARLY_UNION, REFUTE>(
                &span,
                radius,
                &mut report,
                options.max_work,
                metric,
                max_angle_deg,
                &mut whole_side_proved,
                cancel,
            )?
            else {
                return Ok(unresolved(report, UnresolvedReason::Work));
            };
            let deg = upper.to_degrees().min(360.);
            // A joint side failure is encoded as TAU. An authored 360deg
            // contact limit must never turn that forbidden-half failure into
            // a successful one-sided front proof.
            let side_unproved = matches!(metric, Metric::Front { .. }) && upper >= TAU;
            if deg <= max_angle_deg && !side_unproved {
                report.upper_angle_deg = report.upper_angle_deg.max(deg);
                continue;
            }
            if depth == 0 {
                // Rejection only. If the root cannot be admitted, avoid
                // subdividing a long prefix before discovering an unproved
                // pose later on. A conservative point bound can refuse the
                // candidate; passing ALL poses never admits any part of it.
                // The original whole-interval tree below remains mandatory.
                for f in [1., 0.5, 0.] {
                    if cancel.is_cancelled() {
                        return Err("engagement query cancelled".into());
                    }
                    if report.nodes >= options.max_nodes {
                        return Ok(unresolved(report, UnresolvedReason::Work));
                    }
                    report.nodes += 1;
                    let pose = Span {
                        motion,
                        path: &path,
                        a: f,
                        b: f,
                        error,
                    };
                    let Some(upper) = self.angular_bound::<EARLY_UNION, REFUTE>(
                        &pose,
                        radius,
                        &mut report,
                        options.max_work,
                        metric,
                        max_angle_deg,
                        &mut whole_side_proved,
                        cancel,
                    )?
                    else {
                        return Ok(unresolved(report, UnresolvedReason::Work));
                    };
                    if upper.to_degrees().min(360.) > max_angle_deg
                        || (matches!(metric, Metric::Front { .. }) && upper >= TAU)
                    {
                        return Ok(unresolved(report, UnresolvedReason::Resolution));
                    }
                }
            }
            if depth >= options.max_depth {
                return Ok(unresolved(report, UnresolvedReason::Depth));
            }
            if path.length_mm() * (b - a) <= options.resolution_mm {
                return Ok(unresolved(report, UnresolvedReason::Resolution));
            }
            let mid = (a + b) / 2.;
            stack.push((mid, b, depth + 1));
            stack.push((a, mid, depth + 1));
        }
        Ok(report)
    }
    #[allow(clippy::too_many_arguments)]
    fn angular_bound<const EARLY_UNION: bool, const REFUTE: bool>(
        &self,
        span: &Span<'_>,
        r: f64,
        report: &mut EngagementReport,
        limit: u64,
        metric: Metric,
        max_angle_deg: f64,
        whole_side_proved: &mut bool,
        cancel: &impl CancelSignal,
    ) -> Result<Option<f64>, String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("engagement.angular");
        let p3 = span.path.position((span.a + span.b) / 2.)?;
        let p = [p3[0], p3[1]];
        let scale = span
            .motion
            .curve
            .start()
            .into_iter()
            .chain(span.motion.curve.end())
            .map(f64::abs)
            .fold(r.max(1.), f64::max);
        let numeric = 2048. * f64::EPSILON * (1. + scale + r);
        let pad = (8. * (numeric / r).sqrt()).min(PI);
        let forbidden = forbidden_sector(span, pad, metric);
        let mut intervals = vec![];
        if matches!(metric, Metric::Leading)
            && let Some([lo, hi]) = trail_sector(span, r, pad)
        {
            if hi <= TAU {
                intervals.push([lo, hi]);
            } else {
                intervals.push([lo, TAU]);
                intervals.push([0., hi - TAU]);
            }
        }
        let x = span.projection([1., 0.]);
        let y = span.projection([0., 1.]);
        let material = self.material_bounds(p3[2]);
        let outside = [
            [r, 0., material[0][0] - x[1] - span.error - numeric],
            [-r, 0., x[0] - material[1][0] - span.error - numeric],
            [0., r, material[0][1] - y[1] - span.error - numeric],
            [0., -r, y[0] - material[1][1] - span.error - numeric],
        ];
        for h in outside {
            // Guard each known-empty halfplane independently, then union.
            // Padding a joint event partition cut artificial holes at roots
            // of OTHER planes, even when this plane clears the whole circle.
            if let Some(clear) = angular::halfplanes([h], pad) {
                clear.append_to(&mut intervals);
                continue;
            }
            let mut events = vec![0., TAU];
            line_events(h, &mut events);
            partition(
                events,
                pad,
                |angle| h[0] * angle.cos() + h[1] * angle.sin() < h[2],
                &mut intervals,
            );
        }
        let delta = span.path.length_mm() * (span.b - span.a) / 2. + span.error;
        let mut contact_upper = TAU;
        let mut side_proved = *whole_side_proved;
        let mut normalized = 0;
        let mut interval_terms = 0;
        let mut incremental = EARLY_UNION;
        let x_bounds = span.projection([1., 0.]);
        let y_bounds = span.projection([0., 1.]);
        let bounds = [
            [x_bounds[0] - r - numeric, y_bounds[0] - r - numeric],
            [x_bounds[1] + r + numeric, y_bounds[1] + r + numeric],
        ];
        if let Metric::WrongSide { material_on_right } | Metric::Front { material_on_right } =
            metric
            && REFUTE
            && !side_proved
        {
            let saved = (report.support_tests, report.index_visits);
            match self.refute_side(
                span,
                r,
                p3[2],
                material_on_right,
                material,
                bounds,
                numeric,
                report,
                limit,
                cancel,
            )? {
                Refutation::Refuted => return Ok(Some(TAU)),
                Refutation::Work => return Ok(None),
                Refutation::Covered => (report.support_tests, report.index_visits) = saved,
            }
        }
        let mut stack = IndexStack::new(&self.roots);
        while let Some(node) = stack.pop() {
            if cancel.is_cancelled() {
                return Err("engagement query cancelled".into());
            }
            if report.support_tests + report.index_visits >= limit {
                return Ok(None);
            }
            // Every interval already encloses clear angles for this WHOLE
            // motion span. Their union can prove admission before older
            // supports are visited; more cleared geometry cannot undo it.
            // Complex fragmented sets use the original one-sort fallback.
            if incremental && intervals.len() > 128 {
                incremental = false;
            }
            if incremental
                && merge_new_intervals(&mut intervals, &mut normalized, &mut interval_terms)
            {
                let (side, upper) = interval_metric_bound(
                    &intervals,
                    forbidden,
                    side_proved,
                    metric,
                    contact_upper,
                    interval_terms,
                );
                if let Some(bound) = sufficient_metric_bound(metric, side, upper, max_angle_deg) {
                    return Ok(Some(bound));
                }
            }
            report.index_visits += 1;
            let node = &self.index[node];
            if node.min_floor > p3[2] {
                continue;
            }
            if (0..2).any(|k| bounds[1][k] < node.bounds[0][k] || bounds[0][k] > node.bounds[1][k])
            {
                continue;
            }
            if let Some(children) = node.children {
                stack.extend(children);
                continue;
            }
            let support = &self.supports[node.support.unwrap()];
            if support.floor > p3[2] {
                continue;
            }
            if report.support_tests + report.index_visits >= limit {
                return Ok(None);
            }
            report.support_tests += 1;
            let h = support.radius - support.error - span.error - numeric;
            if h <= 0. {
                continue;
            }
            match support.shape {
                Shape::Line { a, b, unit, length } => {
                    #[cfg(feature = "profile-planning")]
                    let _profile = crate::planning_profile::span("engagement.line_support");
                    if span.a == 0. && span.b == 1. {
                        let right = match metric {
                            Metric::Total | Metric::Leading => None,
                            Metric::WrongSide { material_on_right }
                            | Metric::Front { material_on_right } => Some(material_on_right),
                        };
                        if let Some((side, upper)) =
                            whole_line_strip_bound(span, r, &support.shape, h, right, numeric, pad)
                        {
                            contact_upper = contact_upper.min(upper);
                            side_proved |= side;
                            *whole_side_proved |= side;
                            if let Some(upper) = sufficient_metric_bound(
                                metric,
                                side_proved,
                                contact_upper,
                                max_angle_deg,
                            ) {
                                return Ok(Some(upper));
                            }
                        }
                    }
                    let normal = [-unit[1], unit[0]];
                    let t = span.projection(unit);
                    let n = span.projection(normal);
                    let ta = dot(a, unit);
                    let na = dot(a, normal);
                    let constraints = [
                        [r * unit[0], r * unit[1], length - (t[1] - ta)],
                        [-r * unit[0], -r * unit[1], t[0] - ta],
                        [r * normal[0], r * normal[1], h - (n[1] - na)],
                        [-r * normal[0], -r * normal[1], h + n[0] - na],
                    ];
                    if let Some(clear) = angular::halfplanes(constraints, pad) {
                        clear.append_to(&mut intervals);
                    } else {
                        let mut events = vec![0., TAU];
                        for c in constraints {
                            line_events(c, &mut events);
                        }
                        partition(
                            events,
                            pad,
                            |angle| {
                                constraints
                                    .iter()
                                    .all(|c| c[0] * angle.cos() + c[1] * angle.sin() < c[2])
                            },
                            &mut intervals,
                        );
                    }
                    for c in [a, b] {
                        disk_intervals(p, r, c, h - delta, pad, &mut intervals);
                    }
                }
                Shape::Arc {
                    a,
                    b,
                    c,
                    radius,
                    angle,
                    sweep,
                } => {
                    #[cfg(feature = "profile-planning")]
                    let _profile = crate::planning_profile::span("engagement.arc_support");
                    if sweep.abs() >= TAU && radius <= h {
                        let disk = radius + h;
                        if disk >= r {
                            let q = span.farthest(c) + span.error + numeric;
                            let upper = if q + r <= disk {
                                // This single committed filled sweep encloses
                                // the ENTIRE cutter at every axis position in
                                // the span. Other history cannot invalidate
                                // that proof; all cutting-side metrics are zero.
                                // Annular holes never enter this branch.
                                return Ok(Some(0.));
                            } else if q == 0. {
                                TAU
                            } else {
                                2. * ((disk * disk - q * q - r * r) / (2. * q * r))
                                    .clamp(-1., 1.)
                                    .acos()
                            };
                            contact_upper = contact_upper.min(upper);
                            if let Metric::WrongSide { material_on_right }
                            | Metric::Front { material_on_right } = metric
                                && !side_proved
                                && span.a == 0.
                                && span.b == 1.
                                && whole_side_disk_bound(
                                    span,
                                    r,
                                    c,
                                    material_on_right,
                                    numeric,
                                    pad,
                                )
                                .is_some_and(|bound| bound <= disk)
                            {
                                // Every instantaneous forbidden half fits this
                                // committed filled disk. Total contact remains
                                // a separate bound for joint front admission.
                                side_proved = true;
                                *whole_side_proved = true;
                                if matches!(metric, Metric::WrongSide { .. }) {
                                    return Ok(Some(0.));
                                }
                            }
                            // A single committed whole-span witness is enough.
                            // Later cleared history can only reduce contact;
                            // no need to build its angular unions for a tighter
                            // report once the authored bound is already proved.
                            if let Some(upper) = sufficient_metric_bound(
                                metric,
                                side_proved,
                                contact_upper,
                                max_angle_deg,
                            ) {
                                return Ok(Some(upper));
                            }
                        }
                    }
                    let h = h - delta;
                    if h <= 0. {
                        continue;
                    }
                    if sweep.abs() >= TAU && radius <= h {
                        disk_intervals(p, r, c, radius + h, pad, &mut intervals);
                        continue;
                    }
                    if let Some(clear) = angular::arc_tube(
                        p,
                        r,
                        angular::ArcTube {
                            start: a,
                            end: b,
                            center: c,
                            radius,
                            angle,
                            sweep,
                            clearance: h,
                            shrink: (numeric / radius).min(0.1),
                        },
                        pad,
                        support.sector.as_ref(),
                    ) {
                        clear.append_to(&mut intervals);
                        continue;
                    }
                    let mut events = vec![0., TAU];
                    circle_events(p, r, c, radius + h, &mut events);
                    if radius > h {
                        circle_events(p, r, c, radius - h, &mut events);
                    }
                    if sweep.abs() < TAU {
                        for endpoint in [a, b] {
                            circle_events(p, r, endpoint, h, &mut events);
                        }
                        // Infinite ray lines include all possible angular-sector
                        // transitions; opposite-ray roots only overpartition.
                        let shrink = (numeric / radius).min(0.1);
                        for theta in [
                            angle + sweep.signum() * shrink,
                            angle + sweep - sweep.signum() * shrink,
                        ] {
                            let normal = [-theta.sin(), theta.cos()];
                            line_events(
                                [
                                    r * normal[0],
                                    r * normal[1],
                                    dot([c[0] - p[0], c[1] - p[1]], normal),
                                ],
                                &mut events,
                            );
                        }
                    }
                    partition(
                        events,
                        pad,
                        |theta| {
                            let q = [p[0] + r * theta.cos(), p[1] + r * theta.sin()];
                            if sweep.abs() < TAU && (distance(q, a) < h || distance(q, b) < h) {
                                return true;
                            }
                            let d = [q[0] - c[0], q[1] - c[1]];
                            let travel = if sweep > 0. {
                                (d[1].atan2(d[0]) - angle).rem_euclid(TAU)
                            } else {
                                (angle - d[1].atan2(d[0])).rem_euclid(TAU)
                            };
                            let shrink = (numeric / radius).min(0.1);
                            (sweep.abs() >= TAU
                                || (travel > shrink && travel < sweep.abs() - shrink))
                                && (d[0].hypot(d[1]) - radius).abs() < h
                        },
                        &mut intervals,
                    );
                }
            }
        }
        if EARLY_UNION {
            merge_new_intervals(&mut intervals, &mut normalized, &mut interval_terms);
        } else {
            intervals.sort_unstable_by(|a, b| a[0].total_cmp(&b[0]));
            interval_terms = intervals.len();
        }
        Ok(Some(
            interval_metric_bound(
                &intervals,
                forbidden,
                side_proved,
                metric,
                contact_upper,
                interval_terms,
            )
            .1,
        ))
    }
}

enum Refutation {
    /// The query cannot be admitted: its bound exceeds every limit (side
    /// unproved, or uncleared contact over the authored angle).
    Refuted,
    /// The work limit came first, with the refutation still standing.
    Work,
    /// No conclusion: run the full query.
    Covered,
}

impl EngagementScene {
    /// Walk the supports exactly as `angular_bound`'s main loop does: same
    /// order, culling and work counters. `visit(node)` sees each counted
    /// support and returns false to stop. Ok(None): the work limit, at the
    /// point the main loop would stop; Ok(Some(true)): every support seen.
    #[allow(clippy::too_many_arguments)]
    fn walk_supports(
        &self,
        z: f64,
        bounds: [Point; 2],
        report: &mut EngagementReport,
        limit: u64,
        cancel: &impl CancelSignal,
        mut visit: impl FnMut(usize) -> bool,
    ) -> Result<Option<bool>, String> {
        let mut stack = IndexStack::new(&self.roots);
        while let Some(id) = stack.pop() {
            if cancel.is_cancelled() {
                return Err("engagement query cancelled".into());
            }
            if report.support_tests + report.index_visits >= limit {
                return Ok(None);
            }
            report.index_visits += 1;
            let node = &self.index[id];
            if node.min_floor > z {
                continue;
            }
            if (0..2).any(|k| bounds[1][k] < node.bounds[0][k] || bounds[0][k] > node.bounds[1][k])
            {
                continue;
            }
            if let Some(children) = node.children {
                stack.extend(children);
                continue;
            }
            if self.supports[node.support.unwrap()].floor > z {
                continue;
            }
            if report.support_tests + report.index_visits >= limit {
                return Ok(None);
            }
            report.support_tests += 1;
            if !visit(id) {
                return Ok(Some(false));
            }
        }
        Ok(Some(true))
    }

    /// Whether the support at index node `id` may reach within `extra` of q:
    /// its axis within radius + error + numeric + extra (exact up to
    /// rounding far below the numeric reserve).
    fn reaches(&self, id: usize, q: Point, extra: f64, numeric: f64) -> bool {
        let node = &self.index[id];
        let support = &self.supports[node.support.unwrap()];
        let margin = extra + numeric;
        (0..2).all(|k| q[k] >= node.bounds[0][k] - margin && q[k] <= node.bounds[1][k] + margin)
            && within_reach(support, q, support.radius + support.error + numeric + extra)
    }

    /// Exact refutation of the side proof, without building angular sets.
    ///
    /// A witness is a point of the cutter circle, at a centre of the span,
    /// strictly inside that centre's own forbidden half (so also inside the
    /// span's forbidden sector), inside the material bounds and farther than
    /// radius + error (+ the numeric reserve) from every committed sweep at
    /// or below z. Every clear set of `angular_bound` holds only angles whose
    /// cutter point lies, for every centre of the span, within the sweep
    /// radius less its error, the span error and the numeric reserve of one
    /// sweep (arc and disk sets shrink by the half span, line strips range
    /// over the span's projections, all are shrunk by the angular pad) or
    /// outside the material bounds. So the witness angle is in none of them
    /// and the union cannot cover the forbidden sector. The single-sweep
    /// side shortcuts (whole_line_strip_bound, whole_side_disk_bound) claim
    /// every centre's whole forbidden half inside one sweep, the witness
    /// point included, so none of them can hold either.
    ///
    /// The walk is the main loop's (`walk_supports`), so a refuted query
    /// leaves the report as the full query would, and reaching the limit
    /// with a witness alive is the full query's own work stop (no proof can
    /// precede it). If every witness is covered the caller restores the
    /// counters and runs the full query.
    #[allow(clippy::too_many_arguments)]
    fn refute_side(
        &self,
        span: &Span<'_>,
        r: f64,
        z: f64,
        material_on_right: bool,
        material: [Point; 2],
        bounds: [Point; 2],
        numeric: f64,
        report: &mut EngagementReport,
        limit: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Refutation, String> {
        // Fractions of a quarter turn from the forbidden direction, denser
        // towards the half's edges, where uncleared gaps usually remain.
        const FRACTIONS: [f64; 23] = [
            0., -0.111, 0.111, -0.222, 0.222, -0.333, 0.333, -0.444, 0.444, -0.556, 0.556, -0.667,
            0.667, -0.778, 0.778, -0.889, 0.889, -0.95, 0.95, -0.99, 0.99, -0.999, 0.999,
        ];
        let mut witnesses = [[0.; 2]; 3 * FRACTIONS.len()];
        let mut alive = 0;
        for f in [0.5, 0., 1.] {
            let m = span.a + f * (span.b - span.a);
            let c = span.path.position(m)?;
            // This centre's heading, as forbidden_sector takes it.
            let heading = match span.motion.curve {
                Curve::Line { start, end } => (end[1] - start[1]).atan2(end[0] - start[0]),
                Curve::Arc {
                    start,
                    center,
                    sweep_rad,
                    ..
                } => {
                    (start[1] - center[1]).atan2(start[0] - center[0])
                        + m * sweep_rad
                        + sweep_rad.signum() * PI / 2.
                }
                _ => return Ok(Refutation::Covered),
            };
            let centre = heading + if material_on_right { PI / 2. } else { -PI / 2. };
            for fraction in FRACTIONS {
                // Strictly inside the half (|fraction| < 1).
                let t = centre + PI / 2. * fraction;
                let q = [c[0] + r * t.cos(), c[1] + r * t.sin()];
                if (0..2).all(|k| q[k] >= material[0][k] && q[k] <= material[1][k]) {
                    witnesses[alive] = q;
                    alive += 1;
                }
            }
        }
        if alive == 0 {
            return Ok(Refutation::Covered);
        }
        // One live witness at a time: when a support covers it, the next
        // witness must also escape every support already seen. Refuted
        // exactly when some witness escapes them all.
        let mut seen = Vec::with_capacity(256);
        let mut current = 0;
        let walked = self.walk_supports(z, bounds, report, limit, cancel, |id| {
            seen.push(id);
            while self.reaches(id, witnesses[current], 0., numeric) {
                current += 1;
                loop {
                    if current == alive {
                        return false;
                    }
                    if !seen
                        .iter()
                        .any(|&old| self.reaches(old, witnesses[current], 0., numeric))
                    {
                        break;
                    }
                    current += 1;
                }
            }
            true
        })?;
        Ok(match walked {
            None => Refutation::Work,
            Some(true) => Refutation::Refuted,
            Some(false) => Refutation::Covered,
        })
    }
}

/// Whether q may lie within `reach` of the support's axis: exactly
/// shape_distance <= reach up to rounding (far below the numeric reserve in
/// `reach`), without atan2. Off the arc's band every arc point, endpoints
/// included, is farther than reach; in the band the sector decides between
/// the band distance and the nearer endpoint.
#[inline(always)]
fn within_reach(support: &ClearedSweep, q: Point, reach: f64) -> bool {
    match support.shape {
        Shape::Line { .. } => shape_distance(&support.shape, q) <= reach,
        Shape::Arc {
            a,
            b,
            c,
            radius,
            sweep,
            ..
        } => {
            let v = [q[0] - c[0], q[1] - c[1]];
            if ((v[0] * v[0] + v[1] * v[1]).sqrt() - radius).abs() > reach {
                return false;
            }
            sweep.abs() >= TAU
                || support
                    .sector
                    .as_ref()
                    .is_none_or(|sector| sector.contains(v, sweep.abs()))
                || distance(q, a).min(distance(q, b)) <= reach
        }
    }
}

/// Whole-ball inclusion in one actual depth-tagged committed sweep.
fn ball_in_support(s: &ClearedSweep, p: Point, extent: f64, z: f64, numeric: f64) -> bool {
    if s.floor > z {
        return false;
    }
    let h = s.radius - s.error - numeric;
    if let Shape::Arc {
        c, radius, sweep, ..
    } = s.shape
        && sweep.abs() >= TAU
        && radius <= h
        && distance(p, c) + extent <= radius + h
    {
        return true;
    }
    !certainly_farther(&s.shape, p, h - extent) && shape_distance(&s.shape, p) + extent <= h
}

/// True only when shape_distance(shape, p) certainly exceeds `limit` by far
/// more than any rounding in the callers' exact comparisons, so they can
/// skip the exact (atan2) distance with the same decision. An arc's band
/// distance |p - c| - radius bounds the distance to every arc point,
/// endpoints included. Lines are always left to the exact test.
#[inline(always)]
fn certainly_farther(shape: &Shape, p: Point, limit: f64) -> bool {
    let Shape::Arc { c, radius, .. } = *shape else {
        return false;
    };
    let v = [p[0] - c[0], p[1] - c[1]];
    let centre = (v[0] * v[0] + v[1] * v[1]).sqrt();
    let slack = 1e-9 * (1. + centre + radius + limit.abs());
    (centre - radius).abs() > limit + slack
}

/// Merge only existing known-empty intervals. Keep the original number of
/// arithmetic terms even after compression so the roundoff reserve never shrinks.
fn merge_new_intervals(
    intervals: &mut Vec<[f64; 2]>,
    normalized: &mut usize,
    terms: &mut usize,
) -> bool {
    #[cfg(feature = "profile-planning")]
    let _profile = crate::planning_profile::span("engagement.merge");
    if intervals.len() == *normalized {
        return false;
    }
    *terms += intervals.len() - *normalized;
    intervals.sort_unstable_by(|a, b| a[0].total_cmp(&b[0]));
    let mut written = 0;
    for read in 0..intervals.len() {
        let value = intervals[read];
        if written > 0 && value[0] <= intervals[written - 1][1] {
            intervals[written - 1][1] = intervals[written - 1][1].max(value[1]);
        } else {
            intervals[written] = value;
            written += 1;
        }
    }
    intervals.truncate(written);
    *normalized = written;
    true
}

/// This world-angle sector depends on the query span, not the growing union.
/// Compute it once while retaining the same whole-span turn and angular reserve.
fn forbidden_sector(span: &Span<'_>, pad: f64, metric: Metric) -> Option<[f64; 2]> {
    let material_on_right = match metric {
        Metric::WrongSide { material_on_right } | Metric::Front { material_on_right } => {
            material_on_right
        }
        Metric::Total | Metric::Leading => return None,
    };
    let (heading, turn) = span_heading(span);
    // The union of instantaneous forbidden halves. Existing world-angle
    // clear intervals hold for every centre in this whole span.
    let half = (PI / 2. + turn + pad).min(PI);
    let centre = heading + if material_on_right { PI / 2. } else { -PI / 2. };
    let lo = (centre - half).rem_euclid(TAU);
    let hi = lo + 2. * half;
    Some(if half == PI { [0., TAU] } else { [lo, hi] })
}

/// World angles of the cutter circumference that lie in the cutter's own
/// swept trail for every pose of `span` after the motion's start.
///
/// A circumference point at angle t from the feed direction, at centre c, is
/// p = c + r (cos t, sin t). The centre a small arc length d earlier is
/// c - d u + O(d^2) for the unit heading u, so the squared distance from p to
/// it is r^2 + 2 r d cos t + O(d^2). For cos t < 0 that is below r^2: the
/// point is strictly inside the cutter disc at that earlier pose, and the
/// cutter has already removed it at this same depth. The open trailing half
/// is therefore clear at every pose with any trail behind it, on lines and
/// arcs alike.
///
/// Over a span the heading varies by `turn` either side of its middle, so
/// only the intersection of the trailing halves is used, less the angular
/// reserve `pad` and the angle the path's declared position error can turn
/// the heading over one cutter radius.
fn trail_sector(span: &Span<'_>, r: f64, pad: f64) -> Option<[f64; 2]> {
    let (heading, turn) = span_heading(span);
    let reserve = turn + pad + (span.error / r).min(1.).asin();
    let half = PI / 2. - reserve;
    if half <= 0. {
        return None;
    }
    let lo = (heading + PI - half).rem_euclid(TAU);
    Some([lo, lo + 2. * half])
}

/// Feed heading at the middle of the span and the most it turns either side.
fn span_heading(span: &Span<'_>) -> (f64, f64) {
    let mid = (span.a + span.b) / 2.;
    match span.motion.curve {
        Curve::Line { start, end } => ((end[1] - start[1]).atan2(end[0] - start[0]), 0.),
        Curve::Arc {
            start,
            center,
            sweep_rad,
            ..
        } => (
            (start[1] - center[1]).atan2(start[0] - center[0])
                + mid * sweep_rad
                + sweep_rad.signum() * PI / 2.,
            sweep_rad.abs() * (span.b - span.a) / 2.,
        ),
        _ => unreachable!(),
    }
}

fn interval_metric_bound(
    intervals: &[[f64; 2]],
    forbidden: Option<[f64; 2]>,
    mut side_proved: bool,
    metric: Metric,
    contact_upper: f64,
    terms: usize,
) -> (bool, f64) {
    #[cfg(feature = "profile-planning")]
    let _profile = crate::planning_profile::span("engagement.metric");
    if !side_proved && let Some([lo, hi]) = forbidden {
        let covered = |a: f64, b: f64| {
            let mut cursor = a;
            for interval in intervals {
                if interval[0] > cursor {
                    break;
                }
                cursor = cursor.max(interval[1]);
                if cursor >= b {
                    return true;
                }
            }
            false
        };
        let clear = if hi <= TAU {
            covered(lo, hi)
        } else {
            covered(lo, TAU) && covered(0., hi - TAU)
        };
        side_proved = clear;
    }
    if matches!(metric, Metric::WrongSide { .. }) {
        return (side_proved, if side_proved { 0. } else { TAU });
    }
    if matches!(metric, Metric::Front { .. }) && !side_proved {
        return (false, TAU);
    }
    let mut clear = 0.;
    let mut end = 0.;
    for i in intervals {
        if i[1] > end {
            clear += (i[1] - end.max(i[0])).max(0.);
            end = i[1];
        }
    }
    let round = 8192. * f64::EPSILON * (1. + terms as f64) * TAU;
    (
        side_proved,
        (TAU - clear + round).clamp(0., TAU).min(contact_upper),
    )
}
/// A completed sufficient certificate may stop the history query. Uncertainty
/// never enters here as side=true; TAU still encodes an unproved joint side.
fn sufficient_metric_bound(
    metric: Metric,
    side: bool,
    upper: f64,
    max_angle_deg: f64,
) -> Option<f64> {
    match metric {
        Metric::WrongSide { .. } if side => Some(0.),
        Metric::Total | Metric::Leading if upper.to_degrees().min(360.) <= max_angle_deg => {
            Some(upper)
        }
        Metric::Front { .. }
            if side && upper < TAU && upper.to_degrees().min(360.) <= max_angle_deg =>
        {
            Some(upper)
        }
        _ => None,
    }
}

/// Whole LINE cutter-half enclosure in the finite inner rectangle of a
/// committed line capsule, with a separate total-contact bound. Linear
/// projection extrema and analytic half-circle extrema cover every axis
/// position and every cutter angle; no point samples or infinite-strip premise.
fn whole_line_strip_bound(
    span: &Span<'_>,
    radius: f64,
    shape: &Shape,
    h: f64,
    right: Option<bool>,
    numeric: f64,
    pad: f64,
) -> Option<(bool, f64)> {
    let Curve::Line { start, end } = span.motion.curve else {
        return None;
    };
    let Shape::Line { a, b, unit, length } = *shape else {
        return None;
    };
    let scale = start
        .into_iter()
        .chain(end)
        .chain(a)
        .chain(b)
        .map(f64::abs)
        .fold(radius.max(length).max(1.), f64::max);
    let arithmetic = 8192. * f64::EPSILON * (1. + scale + radius);
    let direction = [end[0] - start[0], end[1] - start[1]];
    let distance = direction[0].hypot(direction[1]);
    if distance <= arithmetic {
        return None;
    }
    // Orientation becomes uncertain for very short, far-translated lines.
    // Widen all projected half-circle extrema; never choose a narrow sign
    // branch when its dot product could cross zero under this error.
    let angular = arithmetic / distance + 8192. * f64::EPSILON;
    if angular >= 0.01 {
        return None;
    }
    let tangent = direction.map(|v| v / distance);
    let normal = [-unit[1], unit[0]];
    let guard = span.error + numeric + arithmetic + radius * (pad + 4. * angular);
    let project = |axis| {
        let x = dot([start[0] - a[0], start[1] - a[1]], axis);
        let y = dot([end[0] - a[0], end[1] - a[1]], axis);
        [x.min(y), x.max(y)]
    };
    let t = project(unit);
    let n = project(normal);
    let side = right.is_some_and(|right| {
        let forbidden = if right {
            [-tangent[1], tangent[0]]
        } else {
            [tangent[1], -tangent[0]]
        };
        let half = |axis| {
            let facing = dot(axis, forbidden);
            let endpoint = (dot(axis, tangent).abs() + angular).min(1.);
            [
                if facing <= angular { -1. } else { -endpoint },
                if facing >= -angular { 1. } else { endpoint },
            ]
        };
        let along = half(unit);
        let across = half(normal);
        t[0] + radius * along[0] > guard
            && t[1] + radius * along[1] < length - guard
            && n[0] + radius * across[0] > -h + guard
            && n[1] + radius * across[1] < h - guard
    });
    // For total contact the ENTIRE circumference must fit the finite strip's
    // longitudinal extent. The two normal halfplanes then bound the sum of
    // possibly contacting angular caps throughout the complete line.
    let upper = if t[0] - radius > guard && t[1] + radius < length - guard {
        let h = h - guard;
        (2. * ((h - n[1]) / radius).clamp(-1., 1.).acos()
            + 2. * ((h + n[0]) / radius).clamp(-1., 1.).acos()
            + 8192. * f64::EPSILON * TAU)
            .min(TAU)
    } else {
        TAU
    };
    Some((side, upper))
}

/// Sufficient WHOLE-motion enclosure of the forbidden cutter half in a filled
/// disk. No endpoint sampling inference: a line uses convexity, and an inward
/// arc uses a bound valid for ALL axis angles, including multiple turns.
fn whole_side_disk_bound(
    span: &Span<'_>,
    radius: f64,
    centre: Point,
    right: bool,
    numeric: f64,
    pad: f64,
) -> Option<f64> {
    let mut scale = span
        .motion
        .curve
        .start()
        .into_iter()
        .chain(span.motion.curve.end())
        .chain(centre)
        .map(f64::abs)
        .fold(radius.max(1.), f64::max);
    if let Curve::Arc { center, .. } = span.motion.curve {
        scale = center.into_iter().map(f64::abs).fold(scale, f64::max);
    }
    let arithmetic = 8192. * f64::EPSILON * (1. + scale + radius);
    // Keep the existing angular reserve: widening a unit half-circle by pad
    // moves any point by <=pad, hence <=radius*pad on the cutter.
    let guard = span.error + numeric + arithmetic + radius * pad;
    let bound = match span.motion.curve {
        Curve::Line { start, end } => {
            let direction = [end[0] - start[0], end[1] - start[1]];
            let length = direction[0].hypot(direction[1]);
            if length <= arithmetic {
                return None;
            }
            let t = direction.map(|v| v / length);
            let n = if right { [-t[1], t[0]] } else { [t[1], -t[0]] };
            let mut far: f64 = 0.;
            for f in [span.a, span.b] {
                let p = span.path.position(f).ok()?;
                let q = [p[0] - centre[0], p[1] - centre[1]];
                let d = q[0].hypot(q[1]);
                let distance = if dot(q, n) >= -arithmetic {
                    d + radius
                } else {
                    (d * d + radius * radius + 2. * radius * dot(q, t).abs()).sqrt()
                };
                far = far.max(distance);
            }
            // Max_v |p+r*v-c| for a fixed forbidden half is convex in p;
            // endpoint maxima therefore enclose EVERY point of this line.
            far
        }
        Curve::Arc {
            start,
            center,
            sweep_rad,
            ..
        } => {
            if sweep_rad == 0. || (sweep_rad > 0.) != right {
                return None;
            }
            let axis = (start[0] - center[0]).hypot(start[1] - center[1]);
            let d = (center[0] - centre[0]).hypot(center[1] - centre[1]);
            if d + guard >= axis {
                return None;
            }
            // p-c=d+R*u, forbidden v.u<=0. Since R>|d|, the farthest
            // permitted v lies at +/-tangent: |p+r*v-c|^2 is at most
            // (R+|d|)^2+r^2+2*r*|d| for EVERY u, no arc sampling.
            ((axis + d).powi(2) + radius * radius + 2. * radius * d).sqrt()
        }
        _ => return None,
    };
    (bound + guard).is_finite().then_some(bound + guard)
}

fn unresolved(mut r: EngagementReport, reason: UnresolvedReason) -> EngagementReport {
    r.status = EngagementStatus::Unresolved;
    r.upper_angle_deg = 360.;
    r.reason = Some(reason);
    r
}
fn dot(a: Point, b: Point) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
fn distance(a: Point, b: Point) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}
struct Span<'a> {
    motion: &'a Motion,
    path: &'a PreparedCurve,
    a: f64,
    b: f64,
    error: f64,
}
impl Span<'_> {
    fn projection(&self, axis: Point) -> [f64; 2] {
        let p = self.path.position(self.a).unwrap();
        let q = self.path.position(self.b).unwrap();
        let (a, b) = (dot([p[0], p[1]], axis), dot([q[0], q[1]], axis));
        let mut range = [a.min(b), a.max(b)];
        if let Curve::Arc {
            start,
            center,
            sweep_rad,
            ..
        } = self.motion.curve
        {
            let radius = (start[0] - center[0]).hypot(start[1] - center[1]);
            let phase = (start[1] - center[1]).atan2(start[0] - center[0]);
            let lo = phase + sweep_rad * self.a;
            let hi = phase + sweep_rad * self.b;
            let direction = axis[1].atan2(axis[0]);
            for (angle, value) in [
                (direction, dot([center[0], center[1]], axis) + radius),
                (direction + PI, dot([center[0], center[1]], axis) - radius),
            ] {
                if contains_angle(lo, hi, angle) {
                    range[0] = range[0].min(value);
                    range[1] = range[1].max(value);
                }
            }
        }
        [range[0] - self.error, range[1] + self.error]
    }
    fn farthest(&self, c: Point) -> f64 {
        let p = self.path.position(self.a).unwrap();
        let q = self.path.position(self.b).unwrap();
        let mut max = distance([p[0], p[1]], c).max(distance([q[0], q[1]], c));
        if let Curve::Arc {
            start,
            center,
            sweep_rad,
            ..
        } = self.motion.curve
        {
            let phase = (start[1] - center[1]).atan2(start[0] - center[0]);
            let angle = (center[1] - c[1]).atan2(center[0] - c[0]);
            if contains_angle(
                phase + sweep_rad * self.a,
                phase + sweep_rad * self.b,
                angle,
            ) {
                max = max.max(
                    distance([center[0], center[1]], c)
                        + (start[0] - center[0]).hypot(start[1] - center[1]),
                );
            }
        }
        max
    }
}
fn contains_angle(a: f64, b: f64, angle: f64) -> bool {
    ((a.min(b) - angle) / TAU).ceil() <= ((a.max(b) - angle) / TAU).floor()
}
/// Fixed-capacity phase list for `clipped_disk_segment_max`.
struct Phases {
    values: [f64; 20],
    len: usize,
}
impl Default for Phases {
    fn default() -> Self {
        Self {
            values: [0.; 20],
            len: 0,
        }
    }
}
impl Phases {
    fn as_slice(&self) -> &[f64] {
        &self.values[..self.len]
    }
}
impl Extend<f64> for Phases {
    fn extend<I: IntoIterator<Item = f64>>(&mut self, iter: I) {
        for value in iter {
            // Capacity is the proven maximum; exceeding it is a logic error.
            self.values[self.len] = value;
            self.len += 1;
        }
    }
}
fn line_events(h: [f64; 3], out: &mut impl Extend<f64>) {
    let amplitude = h[0].hypot(h[1]);
    if amplitude == 0. {
        return;
    }
    let k = h[2] / amplitude;
    if !(-1. ..=1.).contains(&k) {
        return;
    }
    let phase = h[1].atan2(h[0]);
    let half = k.acos();
    out.extend([
        (phase - half).rem_euclid(TAU),
        (phase + half).rem_euclid(TAU),
    ]);
}
fn circle_events(p: Point, r: f64, c: Point, radius: f64, out: &mut impl Extend<f64>) {
    let d = [c[0] - p[0], c[1] - p[1]];
    let length = d[0].hypot(d[1]);
    if length == 0. || radius <= 0. {
        return;
    }
    let k = (length * length + r * r - radius * radius) / (2. * length * r);
    if !(-1. ..=1.).contains(&k) {
        return;
    }
    let phase = d[1].atan2(d[0]);
    let half = k.acos();
    out.extend([
        (phase - half).rem_euclid(TAU),
        (phase + half).rem_euclid(TAU),
    ]);
}
fn partition(
    mut events: Vec<f64>,
    pad: f64,
    inside: impl Fn(f64) -> bool,
    out: &mut Vec<[f64; 2]>,
) {
    events.sort_unstable_by(f64::total_cmp);
    events.dedup();
    for pair in events.windows(2) {
        let a = pair[0];
        let b = pair[1];
        let mid = (a + b) / 2.;
        if mid > a && mid < b && inside(mid) {
            let lo = a + if a == 0. { 0. } else { pad };
            let hi = b - if b == TAU { 0. } else { pad };
            if lo < hi {
                out.push([lo, hi]);
            }
        }
    }
}
fn disk_intervals(p: Point, r: f64, c: Point, radius: f64, pad: f64, out: &mut Vec<[f64; 2]>) {
    if radius <= 0. {
        return;
    }
    if let Some(clear) = angular::disk(p, r, c, radius, pad) {
        clear.append_to(out);
        return;
    }
    let mut events = vec![0., TAU];
    circle_events(p, r, c, radius, &mut events);
    partition(
        events,
        pad,
        |theta| distance([p[0] + r * theta.cos(), p[1] + r * theta.sin()], c) < radius,
        out,
    );
}

/// Full boundary extrema for the convex set Disk(p,r) intersect rectangle.
/// A circle arc's distance to a segment is piecewise point-distance or absolute
/// normal projection. Include their stationary phases, projection switches and
/// rectangle intersections. Straight rectangle edges attain a convex maximum
/// at endpoints (corners or those same circle intersections).
fn clipped_disk_segment_max(
    p: Point,
    r: f64,
    stock: [Point; 2],
    a: Point,
    b: Point,
    unit: Point,
    length: f64,
) -> Option<f64> {
    let scale = p
        .into_iter()
        .chain(a)
        .chain(b)
        .chain(stock.into_iter().flatten())
        .map(f64::abs)
        .fold(r.max(1.), f64::max);
    // Expand the clip rectangle and reserve for ill-conditioned tangencies,
    // trig/event arithmetic. Engineering bound, not interval arithmetic.
    let reserve = 8. * f64::EPSILON.sqrt() * (1. + scale);
    let clip = [stock[0].map(|v| v - reserve), stock[1].map(|v| v + reserve)];
    let normal = [-unit[1], unit[0]];
    // At most 20 phases (4 fixed, 4 endpoint, 2x2 end-cap, 4x2 clip events):
    // a stack buffer, since this runs once per candidate support.
    let mut phases = Phases::default();
    phases.extend([
        0.,
        TAU,
        normal[1].atan2(normal[0]),
        normal[1].atan2(normal[0]) + PI,
    ]);
    for end in [a, b] {
        let theta = (p[1] - end[1]).atan2(p[0] - end[0]);
        phases.extend([theta, theta + PI]);
    }
    for t in [0., length] {
        line_events(
            [
                r * unit[0],
                r * unit[1],
                t - dot([p[0] - a[0], p[1] - a[1]], unit),
            ],
            &mut phases,
        );
    }
    for axis in 0..2 {
        for side in &clip {
            let mut event = [0., 0., side[axis] - p[axis]];
            event[axis] = r;
            line_events(event, &mut phases);
        }
    }
    let mut maximum: Option<f64> = None;
    let axis_distance = |q: Point| {
        let t = dot([q[0] - a[0], q[1] - a[1]], unit).clamp(0., length);
        distance(q, [a[0] + t * unit[0], a[1] + t * unit[1]])
    };
    let mut include = |q: Point| {
        let d = axis_distance(q);
        maximum = Some(maximum.map_or(d, |old| old.max(d)));
    };
    for &theta in phases.as_slice() {
        let q = [p[0] + r * theta.cos(), p[1] + r * theta.sin()];
        if (0..2).all(|k| q[k] >= clip[0][k] - reserve && q[k] <= clip[1][k] + reserve) {
            include(q);
        }
    }
    for corner in 0..4 {
        let q = std::array::from_fn(|k| clip[(corner >> k) & 1][k]);
        if distance(q, p) <= r + reserve {
            include(q);
        }
    }
    maximum.map(|v| v + 4. * reserve)
}

fn shape_distance(shape: &Shape, p: Point) -> f64 {
    match *shape {
        Shape::Line {
            a, unit, length, ..
        } => {
            let t = dot([p[0] - a[0], p[1] - a[1]], unit).clamp(0., length);
            distance(p, [a[0] + t * unit[0], a[1] + t * unit[1]])
        }
        Shape::Arc {
            a,
            b,
            c,
            radius,
            angle,
            sweep,
        } => {
            let v = [p[0] - c[0], p[1] - c[1]];
            let travel = if sweep > 0. {
                angular::wrap_tau(v[1].atan2(v[0]) - angle)
            } else {
                angular::wrap_tau(angle - v[1].atan2(v[0]))
            };
            let end = distance(p, a).min(distance(p, b));
            if sweep.abs() >= TAU || travel <= sweep.abs() {
                end.min((v[0].hypot(v[1]) - radius).abs())
            } else {
                end
            }
        }
    }
}

#[cfg(test)]
mod leading_tests {
    use super::*;
    use crate::motion::{Curve, Motion, MoveKind};

    fn motion(curve: Curve) -> Motion {
        Motion {
            id: 1,
            operation: "op".into(),
            tool: "t".into(),
            kind: MoveKind::Cut,
            curve,
            feed_mm_min: Some(600.),
        }
    }
    fn line(a: [f64; 2], b: [f64; 2]) -> Motion {
        motion(Curve::Line {
            start: [a[0], a[1], 5.],
            end: [b[0], b[1], 5.],
        })
    }
    fn stock() -> StockSpec {
        StockSpec {
            version: 1,
            units: "mm".into(),
            min_mm: [0., 0., 0.],
            max_mm: [200., 100., 10.],
            cell_size_mm: 1.,
            max_cells: 100_000,
        }
    }
    /// The smallest limit (to 0.05 degrees) at which `check` proves `m`.
    fn least(check: impl Fn(f64) -> EngagementReport) -> f64 {
        let (mut lo, mut hi) = (0f64, 360f64);
        while hi - lo > 0.05 {
            let mid = (lo + hi) / 2.;
            if matches!(check(mid).status, EngagementStatus::Bounded) {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        hi
    }

    /// #167. A straight pass of radial width w beside an earlier pass: the
    /// engagement is acos(1 - w/r). The total contact bound counts the wall
    /// behind the cutter's widest point too and gives twice that.
    #[test]
    fn a_straight_pass_engages_acos_one_minus_w_over_r() {
        let options = EngagementOptions::default();
        for (r, w) in [(5., 3.), (5., 0.5), (1., 0.25), (3., 3.)] {
            // The earlier pass runs along y = 50; this one is w further in.
            let earlier = line([-20., 50.], [220., 50.]);
            let scene = EngagementScene::new(
                stock(),
                vec![ClearedSweep::from_motion(&earlier, r, 1e-5).unwrap()],
            )
            .unwrap();
            for (a, b) in [
                ([20., 50. + w], [180., 50. + w]),
                ([180., 50. - w], [20., 50. - w]),
            ] {
                let pass = line(a, b);
                let expected = (1. - w / r).acos().to_degrees();
                let leading = least(|max| {
                    scene
                        .check_leading(&pass, r, max, options, &|| false)
                        .unwrap()
                });
                assert!(
                    (leading - expected).abs() < 0.1,
                    "r {r} w {w}: leading {leading} against {expected}"
                );
                let total = least(|max| scene.check(&pass, r, max, options, &|| false).unwrap());
                assert!(
                    (total - 2. * expected).abs() < 0.1,
                    "r {r} w {w}: total {total} against {}",
                    2. * expected
                );
            }
        }
    }

    /// The trail is only this motion's own: material ahead and on both sides
    /// still counts, and a slot is half the circumference.
    #[test]
    fn leading_contact_counts_everything_ahead() {
        let options = EngagementOptions::default();
        let scene = EngagementScene::new(stock(), vec![]).unwrap();
        let slot = line([50., 50.], [150., 50.]);
        let bound = least(|max| {
            scene
                .check_leading(&slot, 5., max, options, &|| false)
                .unwrap()
        });
        assert!((bound - 180.).abs() < 0.1, "{bound}");
        // In air beside the stock nothing is engaged.
        let air = line([50., -6.], [150., -6.]);
        assert_eq!(
            scene
                .check_leading(&air, 5., 0., options, &|| false)
                .unwrap()
                .status,
            EngagementStatus::Bounded
        );
    }

    /// The trail credit agrees with the same bound taken the slow way: the
    /// motion cut into short pieces, each checked for total contact with the
    /// pieces before it committed. A piece of length s leaves at most
    /// 2 asin(s / 2r) of its own trail uncredited.
    #[test]
    fn leading_contact_agrees_with_committing_the_motion_piece_by_piece() {
        let options = EngagementOptions::default();
        let (r, w) = (5., 2.);
        let earlier = motion(Curve::Arc {
            start: [130., 50., 5.],
            end: [70., 50., 5.],
            center: [100., 50., 5.],
            plane: Plane::Xy,
            sweep_rad: PI,
        });
        // An arc pass concentric with the earlier one, w further out, and
        // one w further in: the contact differs on the two sides of a curve.
        for radius in [30. + w, 30. - w] {
            let pass = Curve::Arc {
                start: [100. + radius, 50., 5.],
                end: [100. - radius, 50., 5.],
                center: [100., 50., 5.],
                plane: Plane::Xy,
                sweep_rad: PI,
            };
            let whole = motion(pass);
            let scene = EngagementScene::new(
                stock(),
                vec![ClearedSweep::from_motion(&earlier, r, 1e-5).unwrap()],
            )
            .unwrap();
            let leading = least(|max| {
                scene
                    .check_leading(&whole, r, max, options, &|| false)
                    .unwrap()
            });
            let pieces = 400;
            let slack = 2. * (PI * radius / pieces as f64 / (2. * r)).asin().to_degrees();
            let mut stepped = EngagementScene::new(
                stock(),
                vec![ClearedSweep::from_motion(&earlier, r, 1e-5).unwrap()],
            )
            .unwrap();
            let mut worst = 0f64;
            for i in 0..pieces {
                let at = |k: usize| {
                    let a = PI * k as f64 / pieces as f64;
                    [100. + radius * a.cos(), 50. + radius * a.sin(), 5.]
                };
                let piece = motion(Curve::Arc {
                    start: if i == 0 {
                        [100. + radius, 50., 5.]
                    } else {
                        at(i)
                    },
                    end: if i + 1 == pieces {
                        [100. - radius, 50., 5.]
                    } else {
                        at(i + 1)
                    },
                    center: [100., 50., 5.],
                    plane: Plane::Xy,
                    sweep_rad: PI / pieces as f64,
                });
                // The first pieces have no trail of their own yet.
                if i > 20 {
                    worst = worst.max(least(|max| {
                        stepped.check(&piece, r, max, options, &|| false).unwrap()
                    }));
                }
                stepped
                    .append(ClearedSweep::from_motion(&piece, r, 1e-5).unwrap())
                    .unwrap();
            }
            assert!(
                leading <= worst + 0.1 && leading >= worst - slack - 0.1,
                "radius {radius}: leading {leading}, piece by piece {worst} (slack {slack})"
            );
        }
    }

    #[test]
    fn a_scene_returns_to_its_mark() {
        let options = EngagementOptions::default();
        let mut scene = EngagementScene::new(stock(), vec![]).unwrap();
        let pass = line([20., 53.], [180., 53.]);
        let before = least(|max| {
            scene
                .check_leading(&pass, 5., max, options, &|| false)
                .unwrap()
        });
        for n in [1, 2, 5, 9] {
            let mark = scene.mark();
            for i in 0..n {
                let y = 50. - i as f64;
                scene
                    .append(
                        ClearedSweep::from_motion(&line([-20., y], [220., y]), 5., 1e-5).unwrap(),
                    )
                    .unwrap();
            }
            let with = least(|max| {
                scene
                    .check_leading(&pass, 5., max, options, &|| false)
                    .unwrap()
            });
            assert!(with < before - 60., "{with} {before}");
            scene.restore(mark);
            let after = least(|max| {
                scene
                    .check_leading(&pass, 5., max, options, &|| false)
                    .unwrap()
            });
            assert_eq!(after, before);
        }
    }
}

#[cfg(test)]
mod full_column_tests {
    #[test]
    fn rectangular_shells_are_floor_tagged_and_preserve_original_stock() {
        let spec = crate::stock::StockSpec {
            version: 1,
            units: "mm".into(),
            min_mm: [0., 0., 0.],
            max_mm: [20., 20., 10.],
            cell_size_mm: 1.,
            max_cells: 1000,
        };
        let mut s = super::EngagementScene::new(spec.clone(), vec![]).unwrap();
        s.append_rectangular_shell([[0.5, 0.5], [19.5, 19.5]], 5.)
            .unwrap();
        assert_eq!(s.material_bounds(4.), [[0., 0.], [20., 20.]]);
        assert_eq!(s.material_bounds(5.), [[0.5, 0.5], [19.5, 19.5]]);
        assert_eq!(s.stock, spec);
        assert!(
            s.clear_box([[0.1, 9.], [0.4, 11.]], 5., &mut 10000, &|| false)
                .unwrap()
        );
        assert!(
            !s.clear_box([[0.1, 9.], [0.4, 11.]], 4., &mut 10000, &|| false)
                .unwrap()
        );
        assert!(
            s.clear_disk([0.25, 10.], 0.1, 5., 1e-5, &mut 10000, &|| false)
                .unwrap()
        );
        assert!(
            !s.clear_disk([0.25, 10.], 0.1, 4., 1e-5, &mut 10000, &|| false)
                .unwrap()
        );
        assert!(
            s.clear_disk([0.25, 10.], 0.1, 5., 1e-5, &mut 0, &|| false)
                .is_err()
        );
        assert!(
            s.clear_box([[0.1, 9.], [0.4, 11.]], 5., &mut 10000, &|| true)
                .is_err()
        );
        assert!(
            s.append_rectangular_shell([[0.4, 0.5], [19.5, 19.5]], 5.)
                .is_err()
        );
        let mut m = crate::motion::Motion {
            id: 1,
            operation: "shell".into(),
            tool: "t".into(),
            kind: crate::motion::MoveKind::Cut,
            curve: crate::motion::Curve::Line {
                start: [0.25, 9., 5.],
                end: [0.25, 11., 5.],
            },
            feed_mm_min: Some(600.),
        };
        assert_eq!(
            s.check_front(
                &m,
                0.1,
                90.,
                true,
                super::EngagementOptions::default(),
                &|| false
            )
            .unwrap()
            .upper_angle_deg,
            0.
        );
        m.curve = crate::motion::Curve::Line {
            start: [0.25, 9., 4.],
            end: [0.25, 11., 4.],
        };
        assert_eq!(
            s.check_front(
                &m,
                0.1,
                90.,
                true,
                super::EngagementOptions::default(),
                &|| false
            )
            .unwrap()
            .status,
            super::EngagementStatus::Unresolved
        );
    }
    use super::*;
    use crate::motion::MoveKind;
    #[test]
    fn clear_line_requires_the_whole_capsule_at_the_requested_floor() {
        let joined = scene(&[([-1., 0.], 0.5, 5.), ([1., 0.], 0.5, 5.)], 1.);
        assert!(
            joined
                .clear_line([-1., 0.], [1., 0.], 0.7, 5., 1e-5, &mut 100000, &|| false)
                .unwrap()
        );
        assert!(
            !joined
                .clear_line([-1., 0.], [1., 0.], 0.7, 4.999, 1e-5, &mut 100000, &|| {
                    false
                })
                .unwrap()
        );
        let separated = scene(&[([-2., 0.], 0.5, 5.), ([2., 0.], 0.5, 5.)], 1.);
        for p in [[-2., 0.], [2., 0.]] {
            assert!(
                separated
                    .clear_disk(p, 0.7, 5., 1e-5, &mut 100000, &|| false)
                    .unwrap()
            );
        }
        assert!(
            !separated
                .clear_line([-2., 0.], [2., 0.], 0.7, 5., 1e-5, &mut 100000, &|| false)
                .unwrap()
        );
        // The ends lie in a genuinely cleared annulus, but the connecting
        // diameter crosses retained material in its centre.
        let annulus = scene(&[([0., 0.], 3., 5.)], 1.);
        assert!(
            !annulus
                .clear_line([-3., 0.], [3., 0.], 0.5, 5., 1e-5, &mut 100000, &|| false)
                .unwrap()
        );
        assert!(
            joined
                .clear_line([-1., 0.], [1., 0.], 0.7, 5., 1e-5, &mut 0, &|| false)
                .is_err()
        );
        assert!(
            joined
                .clear_line([-1., 0.], [1., 0.], 0.7, 5., 1e-5, &mut 100000, &|| true)
                .unwrap_err()
                .contains("cancelled")
        );
    }
    fn scene(disks: &[(Point, f64, f64)], r: f64) -> EngagementScene {
        let stock = StockSpec {
            version: 1,
            units: "mm".into(),
            min_mm: [-20., -20., 0.],
            max_mm: [20., 20., 10.],
            cell_size_mm: 1.,
            max_cells: 10000,
        };
        let supports = disks
            .iter()
            .map(|&(c, orbit, z)| {
                let p = [c[0] + orbit, c[1], z];
                ClearedSweep::from_motion(
                    &Motion {
                        id: 1,
                        operation: "test".into(),
                        tool: "t".into(),
                        kind: MoveKind::Cut,
                        curve: Curve::Arc {
                            start: p,
                            end: p,
                            center: [c[0], c[1], z],
                            plane: Plane::Xy,
                            sweep_rad: TAU,
                        },
                        feed_mm_min: Some(600.),
                    },
                    r,
                    1e-5,
                )
                .unwrap()
            })
            .collect();
        EngagementScene::new(stock, supports).unwrap()
    }
    #[test]
    fn coverage_hint_rechecks_full_geometry_depth_foreign_scene_budget_and_cancel() {
        let history = scene(&[([0., 0.], 0.5, 5.)], 1.);
        let query = [[-0.1, -0.1], [0.1, 0.1]];
        let mut hint = None;
        assert!(
            history
                .clear_box_with_hint(query, 5., &mut hint, &mut 100, &|| false)
                .unwrap()
        );
        assert_eq!(hint, Some(0));
        // Same witness, independently checked on a neighboring complete box.
        let nearby = [[0.2, 0.2], [0.4, 0.4]];
        let mut work = 1;
        assert!(
            history
                .clear_box_with_hint(nearby, 5., &mut hint, &mut work, &|| false)
                .unwrap()
        );
        assert_eq!(work, 0);
        for (bounds, depth) in [
            (query, 4.999),
            ([[2., 2.], [2.1, 2.1]], 5.),
            ([[-2., -2.], [2., 2.]], 5.),
        ] {
            assert!(
                !history
                    .clear_box_with_hint(bounds, depth, &mut hint, &mut 100, &|| false)
                    .unwrap()
            );
        }
        // Index zero in a different scene refers to different material removal.
        let foreign = scene(&[([6., 0.], 0.5, 5.)], 1.);
        assert!(
            !foreign
                .clear_box_with_hint(query, 5., &mut hint, &mut 100, &|| false)
                .unwrap()
        );
        let empty = scene(&[], 1.);
        assert!(
            !empty
                .clear_box_with_hint(query, 5., &mut hint, &mut 100, &|| false)
                .unwrap()
        );
        // A full annular sweep still leaves the entire central disk untouched.
        let annulus = scene(&[([0., 0.], 3., 5.)], 1.);
        assert!(
            !annulus
                .clear_box_with_hint(query, 5., &mut hint, &mut 100, &|| false)
                .unwrap()
        );
        for cancelled in [false, true] {
            let mut work = if cancelled { 100 } else { 0 };
            let before = hint;
            assert!(
                history
                    .clear_box_with_hint(query, 5., &mut hint, &mut work, &|| cancelled)
                    .is_err()
            );
            assert_eq!(hint, before);
            assert_eq!(work, if cancelled { 100 } else { 0 });
        }
        // Analytic filled-disk containment is independent of the lookup path.
        for i in 0..80 {
            let p = [i as f64 / 20. - 2., 0.3];
            let bounds = [[p[0] - 0.04, p[1] - 0.04], [p[0] + 0.04, p[1] + 0.04]];
            let result = history
                .clear_box_with_hint(bounds, 5., &mut hint, &mut 100, &|| false)
                .unwrap();
            assert_eq!(
                result,
                history.clear_box(bounds, 5., &mut 100, &|| false).unwrap()
            );
            if result {
                assert!(p[0].hypot(p[1]) + 0.04 * 2f64.sqrt() < 1.5);
            }
        }
    }
    #[test]
    fn reentry_point_rejection_cannot_certify_unsampled_interior() {
        let mut disks: Vec<_> = (0..16)
            .map(|i| {
                let a = TAU * i as f64 / 16.;
                ([0.99 * a.cos(), 0.99 * a.sin()], 0.01, 5.)
            })
            .collect();
        disks.push(([0., 0.], 0.01, 5.));
        let history = scene(&disks, 0.03);
        assert!(
            history
                .disk_may_be_clear([0., 0.], 1., 5., &mut 10000, &|| false)
                .unwrap()
        );
        assert!(
            !history
                .clear_disk([0., 0.], 1., 5., 1e-5, &mut 100000, &|| false)
                .unwrap()
        );
        assert!(
            history
                .disk_may_be_clear([0., 0.], 1., 5., &mut 0, &|| false)
                .is_err()
        );
        assert!(
            history
                .disk_may_be_clear([0., 0.], 1., 5., &mut 10000, &|| true)
                .is_err()
        );
    }
    #[test]
    fn incremental_union_combines_distinct_sweeps_over_whole_lines_and_turning_arcs() {
        let disks: Vec<_> = (0..195)
            .map(|i| {
                let a = TAU * (i % 3) as f64 / 3.;
                ([a.cos(), a.sin()], 0.4, 5.)
            })
            .collect();
        let history = scene(&disks, 1.);
        for curve in [
            Curve::Line {
                start: [-0.01, 0., 5.],
                end: [0.01, 0., 5.],
            },
            Curve::Arc {
                start: [0., 0., 5.],
                end: [0.2 * 0.05_f64.cos() - 0.2, 0.2 * 0.05_f64.sin(), 5.],
                center: [-0.2, 0., 5.],
                plane: Plane::Xy,
                sweep_rad: 0.05,
            },
        ] {
            let query = Motion {
                id: 2,
                operation: "combined-clearance".into(),
                tool: "t".into(),
                kind: MoveKind::Cut,
                curve,
                feed_mm_min: Some(600.),
            };
            for right in [false, true] {
                let metric = Metric::Front {
                    material_on_right: right,
                };
                let current = history
                    .check_metric_mode::<true, true>(
                        &query,
                        1.,
                        1.,
                        EngagementOptions::default(),
                        metric,
                        &|| false,
                    )
                    .unwrap();
                let reference = history
                    .check_metric_mode::<false, true>(
                        &query,
                        1.,
                        1.,
                        EngagementOptions::default(),
                        metric,
                        &|| false,
                    )
                    .unwrap();
                assert_eq!(current.status, EngagementStatus::Bounded, "{current:?}");
                assert_eq!(reference.status, EngagementStatus::Bounded);
                assert!(
                    current.support_tests * 8 < reference.support_tests,
                    "{current:?} vs {reference:?}"
                );
                assert!(current.upper_angle_deg <= 1.);
                // No single component disk encloses this whole circumference.
                for disk in &disks[..3] {
                    assert_eq!(
                        scene(&[*disk], 1.)
                            .check_front(
                                &query,
                                1.,
                                1.,
                                right,
                                EngagementOptions::default(),
                                &|| false
                            )
                            .unwrap()
                            .status,
                        EngagementStatus::Unresolved
                    );
                }
            }
            assert!(
                history
                    .check_front(&query, 1., 1., true, EngagementOptions::default(), &|| true)
                    .is_err()
            );
            assert_eq!(
                history
                    .check_front(
                        &query,
                        1.,
                        1.,
                        true,
                        EngagementOptions {
                            max_work: 0,
                            ..Default::default()
                        },
                        &|| false
                    )
                    .unwrap()
                    .status,
                EngagementStatus::Unresolved
            );
        }
    }

    /// The side refutation changes no result: random line and arc histories,
    /// line and arc queries, both sides, both side metrics, with and without
    /// the early union, and small work limits. Reports (status, bound, work
    /// counters, depth, reason) equal the full angular proof's.
    #[test]
    fn side_refutation_matches_the_full_angular_proof() {
        let spec = crate::stock::StockSpec {
            version: 1,
            units: "mm".into(),
            min_mm: [-10., -10., 0.],
            max_mm: [10., 10., 10.],
            cell_size_mm: 0.5,
            max_cells: 100_000,
        };
        let mut state = 0x2545_f491_4f6c_dd1d_u64;
        let mut next = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state >> 11) as f64 / (1u64 << 53) as f64
        };
        let mut refuted = 0;
        let mut compared = 0;
        for _ in 0..60 {
            let mut history = vec![];
            let mut at = [next() * 8. - 4., next() * 8. - 4.];
            for i in 0..(5 + (next() * 40.) as usize) {
                let floor = if next() < 0.15 { 6. } else { 5. };
                let curve = if next() < 0.5 {
                    let end = [
                        (at[0] + next() * 4. - 2.).clamp(-9., 9.),
                        (at[1] + next() * 4. - 2.).clamp(-9., 9.),
                    ];
                    Curve::Line {
                        start: [at[0], at[1], floor],
                        end: [end[0], end[1], floor],
                    }
                } else {
                    let radius = 0.3 + next() * 2.;
                    let phase = next() * TAU;
                    let center = [at[0] - radius * phase.cos(), at[1] - radius * phase.sin()];
                    let sweep = (next() * 2. - 1.) * if next() < 0.2 { TAU } else { 3. };
                    let end = phase + sweep;
                    Curve::Arc {
                        start: [at[0], at[1], floor],
                        end: [
                            center[0] + radius * end.cos(),
                            center[1] + radius * end.sin(),
                            floor,
                        ],
                        center: [center[0], center[1], floor],
                        plane: Plane::Xy,
                        sweep_rad: sweep,
                    }
                };
                let end = curve.end();
                at = [end[0].clamp(-9., 9.), end[1].clamp(-9., 9.)];
                let motion = Motion {
                    id: i as u64 + 1,
                    operation: "history".into(),
                    tool: "t".into(),
                    kind: MoveKind::Cut,
                    curve,
                    feed_mm_min: Some(600.),
                };
                if let Ok(sweep) = ClearedSweep::from_motion(&motion, 0.5 + next(), 1e-4) {
                    history.push(sweep);
                }
            }
            let scene = EngagementScene::new(spec.clone(), history).unwrap();
            for _ in 0..20 {
                let start = [next() * 12. - 6., next() * 12. - 6.];
                let curve = if next() < 0.5 {
                    let a = next() * TAU;
                    let l = 0.05 + next() * 2.;
                    Curve::Line {
                        start: [start[0], start[1], 5.],
                        end: [start[0] + l * a.cos(), start[1] + l * a.sin(), 5.],
                    }
                } else {
                    let radius = 0.2 + next() * 2.;
                    let phase = next() * TAU;
                    let center = [
                        start[0] - radius * phase.cos(),
                        start[1] - radius * phase.sin(),
                    ];
                    let sweep = (next() * 2. - 1.) * 2.;
                    Curve::Arc {
                        start: [start[0], start[1], 5.],
                        end: [
                            center[0] + radius * (phase + sweep).cos(),
                            center[1] + radius * (phase + sweep).sin(),
                            5.,
                        ],
                        center: [center[0], center[1], 5.],
                        plane: Plane::Xy,
                        sweep_rad: sweep,
                    }
                };
                let query = Motion {
                    id: 9,
                    operation: "query".into(),
                    tool: "t".into(),
                    kind: MoveKind::Cut,
                    curve,
                    feed_mm_min: Some(600.),
                };
                let radius = 0.4 + next();
                let angle = next() * 200.;
                let options = EngagementOptions {
                    max_work: if next() < 0.2 {
                        20 + (next() * 300.) as u64
                    } else {
                        EngagementOptions::default().max_work
                    },
                    ..Default::default()
                };
                for right in [false, true] {
                    for metric in [
                        Metric::Front {
                            material_on_right: right,
                        },
                        Metric::WrongSide {
                            material_on_right: right,
                        },
                    ] {
                        let pairs = [
                            (
                                scene.check_metric_mode::<true, true>(
                                    &query,
                                    radius,
                                    angle,
                                    options,
                                    metric,
                                    &|| false,
                                ),
                                scene.check_metric_mode::<true, false>(
                                    &query,
                                    radius,
                                    angle,
                                    options,
                                    metric,
                                    &|| false,
                                ),
                            ),
                            (
                                scene.check_metric_mode::<false, true>(
                                    &query,
                                    radius,
                                    angle,
                                    options,
                                    metric,
                                    &|| false,
                                ),
                                scene.check_metric_mode::<false, false>(
                                    &query,
                                    radius,
                                    angle,
                                    options,
                                    metric,
                                    &|| false,
                                ),
                            ),
                        ];
                        for (fast, full) in pairs {
                            let (fast, full) = (
                                fast.map_err(|e| e.to_string()),
                                full.map_err(|e| e.to_string()),
                            );
                            compared += 1;
                            match (&fast, &full) {
                                (Ok(a), Ok(b)) => {
                                    assert_eq!(
                                        format!("{a:?}"),
                                        format!("{b:?}"),
                                        "{query:?} r={radius} angle={angle}"
                                    );
                                    if a.support_tests > 0 && a.upper_angle_deg >= 360. {
                                        refuted += 1;
                                    }
                                }
                                _ => assert_eq!(format!("{fast:?}"), format!("{full:?}")),
                            }
                        }
                    }
                }
            }
        }
        assert!(compared > 4000, "{compared}");
        assert!(refuted > 100, "few rejections exercised: {refuted}");
    }

    #[test]
    fn incremental_union_keeps_real_gaps_and_wrong_depths_unresolved() {
        for floor in [5., 6.] {
            let disks: Vec<_> = (0..195)
                .map(|i| {
                    let a = TAU * (i % 3) as f64 / 3.;
                    ([1.5 * a.cos(), 1.5 * a.sin()], 0.1, floor)
                })
                .collect();
            let history = scene(&disks, 1.);
            let query = Motion {
                id: 2,
                operation: "uncleared-gaps".into(),
                tool: "t".into(),
                kind: MoveKind::Cut,
                curve: Curve::Line {
                    start: [-0.01, 0., 5.],
                    end: [0.01, 0., 5.],
                },
                feed_mm_min: Some(600.),
            };
            for right in [false, true] {
                for early in [false, true] {
                    let metric = Metric::Front {
                        material_on_right: right,
                    };
                    let checked = if early {
                        history.check_metric_mode::<true, true>(
                            &query,
                            1.,
                            360.,
                            EngagementOptions::default(),
                            metric,
                            &|| false,
                        )
                    } else {
                        history.check_metric_mode::<false, true>(
                            &query,
                            1.,
                            360.,
                            EngagementOptions::default(),
                            metric,
                            &|| false,
                        )
                    }
                    .unwrap();
                    assert_eq!(checked.status, EngagementStatus::Unresolved, "{checked:?}");
                }
            }
        }
    }

    #[test]
    fn fragmented_union_fallback_preserves_tiny_uncleared_gaps() {
        let disks: Vec<_> = (0..200)
            .map(|i| {
                let a = TAU * i as f64 / 200.;
                ([a.cos(), a.sin()], 0.001, 5.)
            })
            .collect();
        let history = scene(&disks, 0.01);
        let query = Motion {
            id: 2,
            operation: "fragmented".into(),
            tool: "t".into(),
            kind: MoveKind::Cut,
            curve: Curve::Line {
                start: [-0.001, 0., 5.],
                end: [0.001, 0., 5.],
            },
            feed_mm_min: Some(600.),
        };
        for right in [false, true] {
            let metric = Metric::Front {
                material_on_right: right,
            };
            let current = history
                .check_metric_mode::<true, true>(
                    &query,
                    1.,
                    360.,
                    EngagementOptions::default(),
                    metric,
                    &|| false,
                )
                .unwrap();
            let reference = history
                .check_metric_mode::<false, true>(
                    &query,
                    1.,
                    360.,
                    EngagementOptions::default(),
                    metric,
                    &|| false,
                )
                .unwrap();
            assert_eq!(current.status, EngagementStatus::Unresolved);
            assert_eq!(reference.status, current.status);
            assert_eq!(reference.support_tests, current.support_tests);
        }
    }
    #[test]
    fn finite_line_witness_proves_contact_and_side_without_scanning_older_history() {
        for shift in [[0., 0.], [10000., -20000.]] {
            for angle in [0_f64, 0.73] {
                let point = |x: f64, y: f64| {
                    [
                        shift[0] + x * angle.cos() - y * angle.sin(),
                        shift[1] + x * angle.sin() + y * angle.cos(),
                        5.,
                    ]
                };
                let motion = |id, start, end| Motion {
                    id,
                    operation: "finite-strip".into(),
                    tool: "t".into(),
                    kind: MoveKind::Cut,
                    curve: Curve::Line { start, end },
                    feed_mm_min: Some(600.),
                };
                let old = motion(1, point(-9., 0.), point(9., 0.));
                let support = ClearedSweep::from_motion(&old, 1., 1e-5).unwrap();
                let mut history = vec![support.clone(); 64];
                // The final size-one root is visited first. Other history is
                // real, intersecting and would require additional index work.
                history.push(support);
                let scene = EngagementScene::new(
                    StockSpec {
                        version: 1,
                        units: "mm".into(),
                        min_mm: [shift[0] - 20., shift[1] - 20., 0.],
                        max_mm: [shift[0] + 20., shift[1] + 20., 10.],
                        cell_size_mm: 1.,
                        max_cells: 10000,
                    },
                    history,
                )
                .unwrap();
                for right in [true, false] {
                    let sign = if right { 1. } else { -1. };
                    let query = motion(2, point(-5. * sign, -0.2), point(5. * sign, -0.2));
                    let options = EngagementOptions {
                        max_work: 2,
                        max_nodes: 1,
                        ..Default::default()
                    };
                    let total = scene.check(&query, 1., 90., options, &|| false).unwrap();
                    let side = scene
                        .check_material_side(&query, 1., right, options, &|| false)
                        .unwrap();
                    let joint = scene
                        .check_front(&query, 1., 90., right, options, &|| false)
                        .unwrap();
                    for bound in [&total, &side, &joint] {
                        assert_eq!(bound.status, EngagementStatus::Bounded, "{bound:?}");
                        assert_eq!(bound.nodes, 1);
                        assert_eq!(bound.support_tests, 1);
                        assert_eq!(bound.index_visits, 1);
                    }
                    assert_eq!(side.upper_angle_deg, 0.);
                    let exact = 2. * 0.8f64.acos().to_degrees();
                    assert!(joint.upper_angle_deg >= exact && joint.upper_angle_deg < 90.);
                    assert!(total.upper_angle_deg >= exact && total.upper_angle_deg < 90.);
                    // A stricter peak limit must still fail despite the side
                    // witness. A previous success cannot carry to the wrong
                    // material side, another floor, or a zero work query.
                    for (limit, side, curve, work) in [
                        (60., right, query.curve.clone(), 200000),
                        (360., !right, query.curve.clone(), 200000),
                        (90., right, query.curve.clone(), 0),
                        (
                            360.,
                            right,
                            Curve::Line {
                                start: [query.curve.start()[0], query.curve.start()[1], 4.],
                                end: [query.curve.end()[0], query.curve.end()[1], 4.],
                            },
                            200000,
                        ),
                    ] {
                        let refused = scene
                            .check_front(
                                &Motion {
                                    curve,
                                    ..query.clone()
                                },
                                1.,
                                limit,
                                side,
                                EngagementOptions {
                                    max_work: work,
                                    ..Default::default()
                                },
                                &|| false,
                            )
                            .unwrap();
                        assert_eq!(refused.status, EngagementStatus::Unresolved);
                    }
                    // Corroboration only: exact projection inequalities above
                    // are the proof. Check the finite capsule and actual side.
                    let path = query.curve.prepare(1e-5).unwrap();
                    for i in 0..=64 {
                        let p = path.position(i as f64 / 64.).unwrap();
                        for j in 0..=64 {
                            let theta = angle + PI * j as f64 / 64.;
                            let q = [p[0] + theta.cos(), p[1] + theta.sin()];
                            assert!(shape_distance(&scene.supports[64].shape, q) < 0.999);
                        }
                    }
                    assert!(
                        scene
                            .check_front(&query, 1., 90., right, options, &|| true)
                            .is_err()
                    );
                }
                // The rectangle is finite. Extending beyond a segment end is
                // not justified by an otherwise identical infinite strip.
                let end_query = motion(3, point(8., -0.2), point(10., -0.2));
                assert_eq!(
                    scene
                        .check_front(
                            &end_query,
                            1.,
                            360.,
                            true,
                            EngagementOptions::default(),
                            &|| false,
                        )
                        .unwrap()
                        .status,
                    EngagementStatus::Unresolved
                );
                let polls = std::cell::Cell::new(0);
                let cancelled = scene
                    .check_front(
                        &end_query,
                        1.,
                        90.,
                        true,
                        EngagementOptions::default(),
                        &|| {
                            polls.set(polls.get() + 1);
                            polls.get() > 3
                        },
                    )
                    .unwrap_err();
                assert!(cancelled.contains("cancelled"));
                assert!(polls.get() > 3);
            }
        }
    }

    #[test]
    fn sufficient_filled_disk_contact_stops_only_after_both_front_bounds_pass() {
        let base = scene(&[([0., 0.], 0.5, 5.)], 1.);
        let history =
            EngagementScene::new(base.stock.clone(), vec![base.supports[0].clone(); 65]).unwrap();
        let query = Motion {
            id: 2,
            operation: "early-contact".into(),
            tool: "t".into(),
            kind: MoveKind::Cut,
            feed_mm_min: Some(600.),
            curve: Curve::Arc {
                start: [0.6, 0., 5.],
                end: [0.6, 0., 5.],
                center: [0., 0., 5.],
                plane: Plane::Xy,
                sweep_rad: TAU,
            },
        };
        let options = EngagementOptions {
            max_work: 2,
            max_nodes: 1,
            ..Default::default()
        };
        let bounded = history
            .check_front(&query, 1., 100., true, options, &|| false)
            .unwrap();
        assert_eq!(bounded.status, EngagementStatus::Bounded);
        assert_eq!(bounded.support_tests + bounded.index_visits, 2);
        assert!(bounded.upper_angle_deg > 1. && bounded.upper_angle_deg < 100.);
        for (angle, right) in [(1., true), (360., false)] {
            assert_eq!(
                history
                    .check_front(&query, 1., angle, right, options, &|| false)
                    .unwrap()
                    .status,
                EngagementStatus::Unresolved
            );
        }
    }

    #[test]
    fn filled_disk_proves_whole_forbidden_halves_without_claiming_all_contact_clear() {
        for shift in [[0., 0.], [10000., -20000.]] {
            let source = scene(&[(shift, 0.5, 5.)], 1.);
            let filled = EngagementScene::new(
                StockSpec {
                    version: 1,
                    units: "mm".into(),
                    min_mm: [shift[0] - 20., shift[1] - 20., 0.],
                    max_mm: [shift[0] + 20., shift[1] + 20., 10.],
                    cell_size_mm: 1.,
                    max_cells: 10000,
                },
                source.supports.clone(),
            )
            .unwrap();
            for angle in [0_f64, 0.73] {
                let point = |x: f64, y: f64| {
                    [
                        shift[0] + x * angle.cos() - y * angle.sin(),
                        shift[1] + x * angle.sin() + y * angle.cos(),
                        5.,
                    ]
                };
                for right in [true, false] {
                    let sign = if right { 1. } else { -1. };
                    let curves = [
                        Curve::Line {
                            start: point(0.6, -sign * 0.2),
                            end: point(0.6, sign * 0.2),
                        },
                        Curve::Arc {
                            start: point(0.6, 0.),
                            end: point(0.6, 0.),
                            center: point(0.2, 0.),
                            plane: Plane::Xy,
                            sweep_rad: sign * 2. * TAU,
                        },
                    ];
                    for (kind, curve) in curves.into_iter().enumerate() {
                        let query = Motion {
                            id: 2,
                            operation: "half".into(),
                            tool: "t".into(),
                            kind: MoveKind::Cut,
                            curve,
                            feed_mm_min: Some(600.),
                        };
                        let options = EngagementOptions {
                            max_work: 2,
                            ..Default::default()
                        };
                        let side = filled
                            .check_material_side(&query, 1., right, options, &|| false)
                            .unwrap();
                        assert_eq!(side.status, EngagementStatus::Bounded, "{query:?}");
                        assert_eq!(side.upper_angle_deg, 0.);
                        assert_eq!(side.support_tests + side.index_visits, 2);
                        let total = filled.check(&query, 1., 100., options, &|| false).unwrap();
                        assert_eq!(total.status, EngagementStatus::Bounded);
                        assert!(total.upper_angle_deg > 1., "whole cutter is NOT clear");
                        let joint = filled
                            .check_front(&query, 1., 100., right, options, &|| false)
                            .unwrap();
                        assert_eq!(joint.status, EngagementStatus::Bounded);
                        assert!(joint.upper_angle_deg > 1. && joint.upper_angle_deg <= 100.);
                        assert_eq!(
                            filled
                                .check_front(
                                    &query,
                                    1.,
                                    360.,
                                    !right,
                                    EngagementOptions::default(),
                                    &|| false
                                )
                                .unwrap()
                                .status,
                            EngagementStatus::Unresolved,
                            "wrong side must remain unproved even at360deg"
                        );
                        assert_eq!(
                            filled
                                .check_front(
                                    &query,
                                    1.,
                                    100.,
                                    right,
                                    EngagementOptions {
                                        max_work: 0,
                                        ..options
                                    },
                                    &|| false
                                )
                                .unwrap()
                                .status,
                            EngagementStatus::Unresolved
                        );
                        assert!(
                            filled
                                .check_front(&query, 1., 100., right, options, &|| true)
                                .is_err()
                        );
                        // CORROBORATION only. Whole proof is convexity for the
                        // line, and the all-angle inequality for the arc.
                        let path = query.curve.prepare(1e-5).unwrap();
                        for i in 0..=64 {
                            let f = i as f64 / 64.;
                            let p = path.position(f).unwrap();
                            let inward = if kind == 0 {
                                angle + PI
                            } else {
                                angle + sign * 2. * TAU * f + PI
                            };
                            for j in 0..=64 {
                                let a = inward - PI / 2. + PI * j as f64 / 64.;
                                let q = [p[0] + a.cos(), p[1] + a.sin()];
                                assert!((q[0] - shift[0]).hypot(q[1] - shift[1]) < 1.5 - 0.1);
                            }
                        }
                    }
                }
            }
        }
        let annular = scene(&[([0., 0.], 2., 5.)], 1.);
        let p = [0.6, 0., 5.];
        let query = Motion {
            id: 2,
            operation: "hole".into(),
            tool: "t".into(),
            kind: MoveKind::Cut,
            feed_mm_min: Some(600.),
            curve: Curve::Arc {
                start: p,
                end: p,
                center: [0.2, 0., 5.],
                plane: Plane::Xy,
                sweep_rad: TAU,
            },
        };
        assert_eq!(
            annular
                .check_material_side(&query, 1., true, EngagementOptions::default(), &|| false)
                .unwrap()
                .status,
            EngagementStatus::Unresolved,
            "real inner hole is NOT a filled disk"
        );
    }

    #[test]
    fn whole_side_certificate_keeps_total_subdivision_and_query_isolation() {
        // Each committed filled disk clears a different part of the circle.
        // No individual disk encloses the full cutter sweep. The centre disk
        // proves the inward half for all angles, while total contact needs the
        // union of the others and genuine whole-interval subdivision.
        let history = scene(
            &[
                ([1., 0.], 0.5, 5.),
                ([-1., 0.], 0.5, 5.),
                ([0., 1.], 0.5, 5.),
                ([0., -1.], 0.5, 5.),
                ([0., 0.], 0.5, 5.),
            ],
            1.,
        );
        for right in [true, false] {
            let query = Motion {
                id: 2,
                operation: "union".into(),
                tool: "t".into(),
                kind: MoveKind::Cut,
                feed_mm_min: Some(600.),
                curve: Curve::Arc {
                    start: [0.6, 0., 5.],
                    end: [0.6, 0., 5.],
                    center: [0., 0., 5.],
                    plane: Plane::Xy,
                    sweep_rad: if right { TAU } else { -TAU },
                },
            };
            let root_only = EngagementOptions {
                max_nodes: 1,
                ..Default::default()
            };
            let incomplete = history
                .check_front(&query, 1., 1., right, root_only, &|| false)
                .unwrap();
            assert_eq!(incomplete.status, EngagementStatus::Unresolved);
            let complete = history
                .check_front(&query, 1., 1., right, EngagementOptions::default(), &|| {
                    false
                })
                .unwrap();
            assert_eq!(complete.status, EngagementStatus::Bounded);
            assert!(complete.maximum_depth > 0 && complete.nodes > 4);
            assert!(complete.upper_angle_deg <= 1.);
            let mut deeper = query.clone();
            if let Curve::Arc {
                start, end, center, ..
            } = &mut deeper.curve
            {
                start[2] = 4.;
                end[2] = 4.;
                center[2] = 4.;
            }
            assert_eq!(
                history
                    .check_front(
                        &deeper,
                        1.,
                        360.,
                        right,
                        EngagementOptions::default(),
                        &|| false,
                    )
                    .unwrap()
                    .status,
                EngagementStatus::Unresolved,
                "a previous query's certificate cannot clear a deeper floor"
            );
        }
    }

    #[test]
    fn whole_boxes_and_disks_keep_inner_holes_floors_and_guard_margins() {
        let filled = scene(&[([0., 0.], 0.5, 5.)], 1.);
        let mut work = 2_000_000;
        assert!(
            filled
                .clear_box([[-1., -1.], [1., 1.]], 5., &mut work, &|| false)
                .unwrap()
        );
        assert!(
            filled
                .clear_disk([0., 0.], 1.49, 5., 1e-5, &mut work, &|| false)
                .unwrap()
        );
        assert!(
            !filled
                .clear_disk([0., 0.], 1.5, 5., 1e-5, &mut work, &|| false)
                .unwrap()
        );
        assert!(
            !filled
                .clear_box([[-0.1, -0.1], [0.1, 0.1]], 4., &mut work, &|| false)
                .unwrap()
        );
        let annulus = scene(&[([0., 0.], 2., 5.)], 0.5);
        assert!(
            !annulus
                .clear_box([[-0.1, -0.1], [0.1, 0.1]], 5., &mut work, &|| false)
                .unwrap()
        );
        assert!(
            !annulus
                .clear_disk([0., 0.], 0.1, 5., 1e-5, &mut work, &|| false)
                .unwrap()
        );
        assert!(
            annulus
                .clear_box([[1.9, -0.1], [2.1, 0.1]], 5., &mut work, &|| false)
                .unwrap()
        );
    }
    #[test]
    fn disk_containment_uses_union_subdivision_and_never_budget_or_cancel_as_clear() {
        let union = scene(&[([-0.5, 0.], 0.2, 5.), ([0.5, 0.], 0.2, 5.)], 1.);
        let mut work = 2_000_000;
        assert!(
            !union
                .clear_ball_witness([0., 0.], 0.8, 5., &mut work, &|| false)
                .unwrap()
        );
        assert!(
            union
                .clear_disk([0., 0.], 0.8, 5., 1e-5, &mut work, &|| false)
                .unwrap()
        );
        assert!(
            union
                .clear_disk([0., 0.], 0.8, 5., 1e-5, &mut 0, &|| false)
                .is_err()
        );
        assert!(
            union
                .clear_disk([0., 0.], 0.8, 5., 1e-5, &mut work, &|| true)
                .is_err()
        );
    }
    #[test]
    fn higher_only_history_is_pruned_as_a_subtree_without_clearing_deeper_material() {
        let mut history = scene(&vec![([0., 0.], 0.5, 9.); 4096], 1.);
        let bounds = [[-0.1, -0.1], [0.1, 0.1]];
        let mut work = 4;
        assert!(!history.clear_box(bounds, 8., &mut work, &|| false).unwrap());
        assert_eq!(
            work, 3,
            "one entire high-floor subtree should cost one visit"
        );
        let mut work = 4;
        assert_eq!(
            history
                .point_near_cleared_sweep([0., 0.], 8., 0., &mut work, &|| false)
                .unwrap(),
            Some(false)
        );
        assert_eq!(work, 3);
        let query = Motion {
            id: 1,
            operation: "deep".into(),
            tool: "t".into(),
            kind: MoveKind::Cut,
            curve: Curve::Line {
                start: [-0.1, 0., 8.],
                end: [0.1, 0., 8.],
            },
            feed_mm_min: Some(600.),
        };
        let options = EngagementOptions {
            max_work: 32,
            max_nodes: 1,
            ..Default::default()
        };
        let unknown = history.check(&query, 0.5, 90., options, &|| false).unwrap();
        assert_eq!(unknown.status, EngagementStatus::Unresolved);
        assert_eq!(unknown.support_tests, 0);
        assert_eq!(unknown.index_visits, 1);
        // A mixed subtree must retain its actual deep sweep. Appending the next
        // shallow leaf merges it; pruning uses MINIMUM floor, never maximum.
        let lower = scene(&[([0., 0.], 0.5, 8.), ([0., 0.], 0.5, 9.)], 1.);
        for support in lower.supports {
            history.append(support).unwrap();
        }
        assert!(history.clear_box(bounds, 8., &mut 16, &|| false).unwrap());
        assert_eq!(
            history
                .point_near_cleared_sweep([0., 0.], 8., 0., &mut 16, &|| false)
                .unwrap(),
            Some(true)
        );
        let clear = history
            .check(
                &query,
                0.5,
                90.,
                EngagementOptions {
                    max_work: 32,
                    ..Default::default()
                },
                &|| false,
            )
            .unwrap();
        assert_eq!(clear.status, EngagementStatus::Bounded);
        assert!(clear.index_visits < 16);
        assert!(clear.support_tests > 0);
        assert_eq!(
            history
                .check_material_side(
                    &query,
                    0.5,
                    true,
                    EngagementOptions {
                        max_work: 32,
                        ..Default::default()
                    },
                    &|| false
                )
                .unwrap()
                .status,
            EngagementStatus::Bounded
        );
        assert!(!history.clear_box(bounds, 7., &mut 16, &|| false).unwrap());
    }
    #[test]
    fn clipped_disk_capsule_proof_covers_stock_edge_without_filling_arc_holes() {
        let mut edge = scene(&[], 1.);
        assert!(
            edge.clear_disk([-25., 0.], 0.1, 5., 1e-5, &mut 0, &|| false)
                .is_err()
        );
        assert!(
            edge.clear_disk([-25., 0.], 0.1, 5., 1e-5, &mut 1, &|| true)
                .is_err()
        );
        assert!(
            edge.clear_disk([-25., 0.], 0.1, 5., 1e-5, &mut 1, &|| false)
                .unwrap()
        );
        let line = Motion {
            id: 1,
            operation: "edge".into(),
            tool: "t".into(),
            kind: MoveKind::Cut,
            curve: Curve::Line {
                start: [-20.8, 0., 5.],
                end: [-20.8, 10., 5.],
            },
            feed_mm_min: Some(600.),
        };
        edge.append(ClearedSweep::from_motion(&line, 1., 1e-5).unwrap())
            .unwrap();
        assert!(
            !edge
                .clear_ball_witness([-20.81, 5.], 1.00064, 5., &mut 100, &|| false)
                .unwrap()
        );
        let mut work = 64;
        assert!(
            edge.clear_disk([-20.81, 5.], 1.00064, 5., 1e-5, &mut work, &|| false)
                .unwrap()
        );
        assert!(
            work > 50,
            "analytic whole clipped region should avoid subdivision"
        );
        assert!(
            !edge
                .clear_disk([-20.79, 5.], 1.00064, 5., 1e-5, &mut 200000, &|| false)
                .unwrap()
        );
        assert!(
            !edge
                .clear_disk([-20.81, 10.9], 1.00064, 5., 1e-5, &mut 200000, &|| false)
                .unwrap()
        );
        assert!(
            !edge
                .clear_disk([-20.81, 5.], 1.00064, 4., 1e-5, &mut 200000, &|| false)
                .unwrap()
        );
        assert!(
            edge.clear_disk([-20.81, 5.], 1.00064, 5., 1e-5, &mut 0, &|| false)
                .is_err()
        );
        assert!(
            edge.clear_disk([-20.81, 5.], 1.00064, 5., 1e-5, &mut 200000, &|| true)
                .is_err()
        );
        let annulus = scene(&[([0., 0.], 2., 5.)], 0.5);
        assert!(
            !annulus
                .clear_disk([0., 0.], 0.1, 5., 1e-5, &mut 200000, &|| false)
                .unwrap()
        );
        // Dense independent corroboration of the max enclosure, including
        // clipped corners, short segments and off-centre axis projections.
        for (p, r, rect, a, b) in [
            (
                [-0.81, 0.],
                1.00064,
                [[0., -1.], [1., 1.]],
                [-0.8, -2.],
                [-0.8, 2.],
            ),
            (
                [0.2, 0.3],
                0.7,
                [[-0.1, -0.2], [0.5, 0.4]],
                [-0.4, 0.2],
                [0.6, -0.3],
            ),
            (
                [0., 0.],
                2.,
                [[-0.2, -0.3], [0.2, 0.3]],
                [0.8, 0.1],
                [0.8, 0.1],
            ),
        ] {
            let length = distance(a, b);
            let unit = if length == 0. {
                [1., 0.]
            } else {
                [(b[0] - a[0]) / length, (b[1] - a[1]) / length]
            };
            let maximum = clipped_disk_segment_max(p, r, rect, a, b, unit, length).unwrap();
            for ix in 0..=80 {
                for iy in 0..=80 {
                    let q: Point = std::array::from_fn(|k| {
                        rect[0][k] + (rect[1][k] - rect[0][k]) * [ix, iy][k] as f64 / 80.
                    });
                    if distance(p, q) > r {
                        continue;
                    }
                    let t = dot([q[0] - a[0], q[1] - a[1]], unit).clamp(0., length);
                    assert!(distance(q, [a[0] + t * unit[0], a[1] + t * unit[1]]) <= maximum);
                }
            }
        }
    }
    #[test]
    fn joint_front_proof_requires_whole_contact_and_whole_forbidden_side() {
        let filled = scene(&[([0., 0.], 0.5, 5.)], 1.);
        for radius in [0.58, 0.7] {
            for direction in [1., -1.] {
                for right in [false, true] {
                    let p = [radius, 0., 5.];
                    let query = Motion {
                        id: 1,
                        operation: "front".into(),
                        tool: "t".into(),
                        kind: MoveKind::Cut,
                        feed_mm_min: Some(600.),
                        curve: Curve::Arc {
                            start: p,
                            end: p,
                            center: [0., 0., 5.],
                            plane: Plane::Xy,
                            sweep_rad: direction * TAU,
                        },
                    };
                    let options = EngagementOptions::default();
                    let total = filled.check(&query, 1., 83., options, &|| false).unwrap();
                    let side = filled
                        .check_material_side(&query, 1., right, options, &|| false)
                        .unwrap();
                    let joint = filled
                        .check_front(&query, 1., 83., right, options, &|| false)
                        .unwrap();
                    assert_eq!(
                        joint.status == EngagementStatus::Bounded,
                        total.status == EngagementStatus::Bounded
                            && side.status == EngagementStatus::Bounded,
                        "radius={radius},direction={direction},right={right}: {joint:?}"
                    );
                    if joint.status == EngagementStatus::Bounded {
                        assert!(joint.upper_angle_deg <= 83.);
                    }
                    let unlimited = filled
                        .check_front(&query, 1., 360., right, options, &|| false)
                        .unwrap();
                    assert_eq!(
                        unlimited.status == EngagementStatus::Bounded,
                        side.status == EngagementStatus::Bounded
                    );
                    let budget = filled
                        .check_front(
                            &query,
                            1.,
                            83.,
                            right,
                            EngagementOptions {
                                max_work: 1,
                                ..options
                            },
                            &|| false,
                        )
                        .unwrap();
                    assert_eq!(budget.status, EngagementStatus::Unresolved);
                    assert!(
                        filled
                            .check_front(&query, 1., 83., right, options, &|| true)
                            .is_err()
                    );
                }
            }
        }
    }
    #[test]
    fn fixed_history_stack_preserves_order_at_the_full_supported_capacity() {
        for roots in [vec![18], (0..18).rev().collect()] {
            let mut reference = roots.clone();
            let mut stack = IndexStack::new(&roots);
            let mut visited = 0;
            while let Some(depth) = reference.pop() {
                assert_eq!(stack.pop(), Some(depth));
                if depth > 0 {
                    reference.extend([depth - 1; 2]);
                    stack.extend([depth - 1; 2]);
                }
                visited += 1;
            }
            assert!(visited > 524_000);
            assert!(stack.pop().is_none());
        }
    }

    #[test]
    fn whole_ball_history_pruning_preserves_exhaustive_witnesses_and_bounded_work() {
        let repeated = scene(&vec![([0., 0.], 0.5, 5.); 4096], 1.);
        let mut work = 1;
        assert!(
            !repeated
                .clear_ball_witness([0., 0.], 2., 5., &mut work, &|| false)
                .unwrap()
        );
        assert_eq!(
            work, 0,
            "whole impossible history is rejected by its one root box"
        );
        assert!(
            repeated
                .clear_ball_witness([0., 0.], 1.49, 5., &mut 16, &|| false)
                .unwrap()
        );
        assert!(
            repeated
                .clear_ball_witness([0., 0.], 1.49, 5., &mut 0, &|| false)
                .is_err()
        );
        assert!(
            repeated
                .clear_ball_witness([0., 0.], 1.49, 5., &mut 100, &|| true)
                .is_err()
        );
        let diagonal = ClearedSweep::from_motion(
            &Motion {
                id: 1,
                operation: "diagonal".into(),
                tool: "t".into(),
                kind: MoveKind::Cut,
                feed_mm_min: Some(100.),
                curve: Curve::Line {
                    start: [-10., -10., 5.],
                    end: [10., 10., 5.],
                },
            },
            1.,
            1e-5,
        )
        .unwrap();
        let mut narrow = scene(&[], 1.);
        for _ in 0..4096 {
            narrow.append(diagonal.clone()).unwrap();
        }
        assert!(
            !narrow
                .clear_ball_witness([0., 0.], 2., 5., &mut 1, &|| false)
                .unwrap()
        );
        // A later larger filled disk must widen every merged ancestor bound.
        for support in scene(&[([0., 0.], 1., 5.)], 2.).supports {
            narrow.append(support).unwrap();
        }
        assert!(
            narrow
                .clear_ball_witness([0., 0.], 2.5, 5., &mut 100, &|| false)
                .unwrap()
        );
        for shift in [[0., 0.], [100_000., -200_000.]] {
            let mut history = scene(&[], 1.);
            for i in 0..96 {
                let p = [
                    shift[0] + (i % 8) as f64 - 4.,
                    shift[1] + (i / 8) as f64 - 2.,
                ];
                let z = if i % 3 == 0 { 8. } else { 5. };
                let start = [p[0] + 0.5, p[1], z];
                let curve = match i % 3 {
                    0 => Curve::Line {
                        start,
                        end: [p[0] - 1., p[1] + 0.3, z],
                    },
                    1 => Curve::Arc {
                        start,
                        end: start,
                        center: [p[0], p[1], z],
                        plane: Plane::Xy,
                        sweep_rad: TAU,
                    },
                    _ => Curve::Arc {
                        start,
                        end: [p[0], p[1] - 0.5, z],
                        center: [p[0], p[1], z],
                        plane: Plane::Xy,
                        sweep_rad: -PI / 2.,
                    },
                };
                history
                    .append(
                        ClearedSweep::from_motion(
                            &Motion {
                                id: i,
                                operation: "history".into(),
                                tool: "t".into(),
                                kind: MoveKind::Cut,
                                feed_mm_min: Some(100.),
                                curve,
                            },
                            1.,
                            1e-5,
                        )
                        .unwrap(),
                    )
                    .unwrap();
            }
            for (k, offset) in shift.into_iter().enumerate() {
                history.stock.min_mm[k] += offset;
                history.stock.max_mm[k] += offset;
            }
            for i in 0..256 {
                let p = [
                    shift[0] + (i % 16) as f64 * 0.5 - 4.,
                    shift[1] + (i / 16) as f64 * 0.4 - 3.,
                ];
                for extent in [0., 0.1, 0.6, 1.49, 1.5, 2.] {
                    for z in [4., 5., 8.] {
                        // Original exhaustive witness predicate, WITHOUT any
                        // index/box pruning. Shared distances isolate traversal
                        // equivalence; the concentric disk cases above are analytic.
                        let numeric =
                            4096. * f64::EPSILON * (1. + p[0].abs() + p[1].abs() + extent);
                        let expected = history.supports.iter().any(|s| {
                            if s.floor > z {
                                return false;
                            }
                            let h = s.radius - s.error - numeric;
                            if let Shape::Arc {
                                c, radius, sweep, ..
                            } = s.shape
                                && sweep.abs() >= TAU
                                && radius <= h
                                && distance(p, c) + extent <= radius + h
                            {
                                return true;
                            }
                            shape_distance(&s.shape, p) + extent <= h
                        });
                        assert_eq!(
                            history
                                .clear_ball_witness(p, extent, z, &mut 10000, &|| false)
                                .unwrap(),
                            expected,
                            "{p:?}/{extent}/{z}"
                        );
                    }
                }
            }
        }
    }
}
