//! Engagement-front clearing of one roughing layer (`path_strategy =
//! "engagement_front"`, issues #26/#27).
//!
//! Established practice: constant-engagement ("adaptive", "dynamic",
//! "trochoidal-free high-efficiency") roughing keeps the cutter's radial
//! engagement near a target width by steering along the boundary between cut
//! and uncut stock, entering from air or already-cleared stock wherever it
//! can and helixing only into enclosed material. This module is a clean-room
//! implementation of that idea; it shares no code with any other CAM kernel.
//!
//! The steering model is a fine height grid seeded from the committed stock
//! (whole-cell upper bounds, so it never sees less material than there is)
//! and this layer's own planned sweeps. It is an *estimate* that only sizes
//! the engagement; it is not a safety proof. Safety comes from:
//! - every cutting/link stroke is certified exactly against the layer region
//!   (cutter radius + radial allowance + construction uncertainty) and a
//!   stroke that fails is refused with an error, never adjusted;
//! - every vertical air entry is proven clear by the committed-sweep scene
//!   (the same proof pocket air entries use); a start that cannot be proven
//!   is not used;
//! - the stream verifier still checks the run against stock, part and machine.
use crate::{
    Point,
    engagement::{ClearedSweep, EngagementOptions, EngagementScene, EngagementStatus},
    entry::{EntryPolicy, PocketEntry},
    heights::Heights,
    motion::{Curve, Event, Motion, MoveKind, Vec3},
    planar::Boundary,
    pocket::{PocketOperation, PocketPlan, ResidualStatus, Stroke},
    stock::{CancelSignal, ClearanceCursor, Stock, StockSpec},
    tool::ToolAssembly,
};
use serde::Serialize;
use std::{
    collections::VecDeque,
    f64::consts::{PI, TAU},
};

/// Fine-grid geometry shared by every layer of one roughing operation.
#[derive(Clone, Copy, PartialEq, Debug)]
struct Grid {
    x0: f64,
    y0: f64,
    c: f64,
    nx: usize,
    ny: usize,
}
impl Grid {
    fn idx(&self, i: usize, j: usize) -> usize {
        j * self.nx + i
    }
    fn center(&self, k: usize) -> Point {
        [
            self.x0 + ((k % self.nx) as f64 + 0.5) * self.c,
            self.y0 + ((k / self.nx) as f64 + 0.5) * self.c,
        ]
    }
    fn cell(&self, p: Point) -> Option<(usize, usize)> {
        let i = floor((p[0] - self.x0) / self.c);
        let j = floor((p[1] - self.y0) / self.c);
        (i >= 0. && j >= 0. && i < self.nx as f64 && j < self.ny as f64)
            .then_some((i as usize, j as usize))
    }
    /// The up to eight cells around cell `k`.
    fn for_neighbours(&self, k: usize, mut f: impl FnMut(usize)) {
        let (i, j) = (k % self.nx, k / self.nx);
        for jj in j.saturating_sub(1)..=(j + 1).min(self.ny - 1) {
            for ii in i.saturating_sub(1)..=(i + 1).min(self.nx - 1) {
                if (ii, jj) != (i, j) {
                    f(self.idx(ii, jj));
                }
            }
        }
    }
    /// The box of cells `for_disk(p, rad)` looks at: lowest and highest
    /// column and row (empty when a low exceeds its high).
    fn disk_box(&self, p: Point, rad: f64) -> Option<([usize; 2], [usize; 2])> {
        let lo = [
            ceil((p[0] - rad - self.x0) / self.c - 0.5).max(0.) as usize,
            ceil((p[1] - rad - self.y0) / self.c - 0.5).max(0.) as usize,
        ];
        let hi = [
            floor((p[0] + rad - self.x0) / self.c - 0.5).min(self.nx as f64 - 1.),
            floor((p[1] + rad - self.y0) / self.c - 0.5).min(self.ny as f64 - 1.),
        ];
        if hi[0] < 0. || hi[1] < 0. {
            return None;
        }
        Some((lo, [hi[0] as usize, hi[1] as usize]))
    }
    /// Whether a cell centre `dx`, `dy` from a point is within `rad` of it,
    /// as `dx.hypot(dy) <= rad` decides. The squared distance settles all
    /// but the cells within `DISTANCE_SLACK` of the rim (`sure` and `rim`
    /// from `rim_bounds`); only those need the root.
    #[inline]
    fn within(dx: f64, dy: f64, rad: f64, sure: f64, rim: f64) -> bool {
        let d2 = dx * dx + dy * dy;
        d2 <= sure || (d2 <= rim && dx.hypot(dy) <= rad)
    }
    /// Squared distances surely within `rad`, and the largest that can be.
    /// Nothing is within a negative radius.
    fn rim_bounds(rad: f64) -> (f64, f64) {
        if rad < 0. {
            return (-1., -1.);
        }
        (
            rad * rad * (1. - DISTANCE_SLACK),
            rad * rad * (1. + DISTANCE_SLACK),
        )
    }
    /// The cells `for_disk` visits, in its order, with their centres and
    /// without their distances: `Layer::cut`'s definition.
    #[cfg(test)]
    fn for_disk_cells(&self, p: Point, rad: f64, mut f: impl FnMut(usize, Point)) {
        let Some((lo, hi)) = self.disk_box(p, rad) else {
            return;
        };
        let (sure, rim) = Self::rim_bounds(rad);
        for j in lo[1]..=hi[1] {
            let cy = self.y0 + (j as f64 + 0.5) * self.c;
            for i in lo[0]..=hi[0] {
                let cx = self.x0 + (i as f64 + 0.5) * self.c;
                if Self::within(cx - p[0], cy - p[1], rad, sure, rim) {
                    f(self.idx(i, j), [cx, cy]);
                }
            }
        }
    }
    /// Cells whose centres lie within `rad` of `p`, row by row, with their
    /// distances: the definition the faster searches are tested against.
    #[cfg(test)]
    fn for_disk(&self, p: Point, rad: f64, mut f: impl FnMut(usize, Point, f64)) {
        let Some((lo, hi)) = self.disk_box(p, rad) else {
            return;
        };
        for j in lo[1]..=hi[1] {
            let cy = self.y0 + (j as f64 + 0.5) * self.c;
            for i in lo[0]..=hi[0] {
                let cx = self.x0 + (i as f64 + 0.5) * self.c;
                let d = (cx - p[0]).hypot(cy - p[1]);
                if d <= rad {
                    f(self.idx(i, j), [cx, cy], d);
                }
            }
        }
    }
    /// The cells from `lo` to `hi` along one axis (origin `o`) whose centres
    /// can lie within `half` of the coordinate `at`: a whole cell wider at
    /// each end than the arithmetic says, so rounding never loses one.
    fn span(&self, o: f64, at: f64, half: f64, lo: usize, hi: usize) -> Option<(usize, usize)> {
        let a = (floor((at - half - o) / self.c - 0.5) - 1.).max(lo as f64);
        let b = (ceil((at + half - o) / self.c - 0.5) + 1.).min(hi as f64);
        (a <= b).then_some((a as usize, b as usize))
    }
}

/// `x.floor()` and `x.ceil()`, to the bit, without the library call (this
/// build targets processors without a rounding instruction): through the
/// whole number below 2^52 in size, where the conversion is exact; the
/// library's own for anything else.
#[inline]
fn floor(x: f64) -> f64 {
    if x.abs() < 4e15 {
        let t = x as i64 as f64;
        if t > x {
            t - 1.
        } else if t == 0. {
            // (The sign of a zero is kept: `x` is 0, -0 or in (0, 1).)
            x * 0.
        } else {
            t
        }
    } else {
        x.floor()
    }
}
#[inline]
fn ceil(x: f64) -> f64 {
    -floor(-x)
}

/// Relative slack on a squared distance when cells are ruled out without
/// their exact distance (`Layer::nearest_set`): far above the rounding of
/// `dx * dx + dy * dy` and of `hypot` (a few parts in 1e16), far below the
/// spacing of distinct cell distances.
const DISTANCE_SLACK: f64 = 1e-6;

/// The highest set bit of `row` at or below `i`, not below `lo`.
fn bit_at_or_below(row: &[u64], i: usize, lo: usize) -> Option<usize> {
    let mut w = i / 64;
    let mut word = row[w] & (u64::MAX >> (63 - i % 64));
    loop {
        if word != 0 {
            let b = w * 64 + 63 - word.leading_zeros() as usize;
            return (b >= lo).then_some(b);
        }
        if w == 0 || w * 64 <= lo {
            return None;
        }
        w -= 1;
        word = row[w];
    }
}
/// The lowest set bit of `row` at or above `i`, not above `hi`.
fn bit_at_or_above(row: &[u64], i: usize, hi: usize) -> Option<usize> {
    let mut w = i / 64;
    if i > hi || w >= row.len() {
        return None;
    }
    let mut word = row[w] & (u64::MAX << (i % 64));
    loop {
        if word != 0 {
            let b = w * 64 + word.trailing_zeros() as usize;
            return (b <= hi).then_some(b);
        }
        w += 1;
        if w >= row.len() || w * 64 > hi {
            return None;
        }
        word = row[w];
    }
}

/// The largest whole squared distance in cells, `D`, whose distance in mm
/// as `edt` reports it, `sqrt(D) * c`, is below `t` (at most `t` when
/// `inclusive`); `None` when not even 0 is. So "a set cell within `t`" can
/// be asked of whole numbers and answers exactly as a comparison of `edt`'s
/// output with `t` does. (`t` is a few cutter radii: far below the 1e9
/// `edt` reports where there is no set cell at all.)
fn cells2_within(c: f64, t: f64, inclusive: bool) -> Option<usize> {
    cells2_where(c, t, |d| if inclusive { d <= t } else { d < t })
}

/// `cells2_within` for any test of a distance that holds up to some
/// distance and not beyond (`about` mm, roughly): the largest whole squared
/// distance in cells whose distance in mm, as `edt` reports it, passes.
fn cells2_where(c: f64, about: f64, test: impl Fn(f64) -> bool) -> Option<usize> {
    let t = about;
    let near = |d2: usize| test((d2 as f64).sqrt() * c);
    if !near(0) {
        return None;
    }
    let mut d2 = ((t / c) * (t / c)).floor().max(0.) as usize;
    while !near(d2) {
        d2 -= 1;
    }
    while near(d2 + 1) {
        d2 += 1;
    }
    Some(d2)
}

/// The cells of a window of the grid (columns `i[0]..=i[1]`, rows
/// `j[0]..=j[1]`) that have a set cell of `bits` within a whole squared
/// distance (`Near::new`): the set cells grown by a disk, row by row with
/// word operations. It answers what a comparison of the window's distance
/// transform (`edt`) with a distance answers, without computing a distance
/// for every cell (#202, #192). Set cells outside the window are not seen,
/// as `edt` of the window does not see them.
struct Near {
    /// Rows of `words` words; the first holds column `64 * first_word`.
    grown: Vec<u64>,
    words: usize,
    first_word: usize,
    first_row: usize,
}
impl Near {
    fn new(bits: &[u64], stride: usize, i: [usize; 2], j: [usize; 2], cells2: usize) -> Self {
        let (w0, w1) = (i[0] / 64, i[1] / 64);
        let (words, rows) = (w1 - w0 + 1, j[1] - j[0] + 1);
        let radius = cells2.isqrt();
        // The rows each half-width of the disk belongs to.
        let mut rows_of_width = vec![vec![]; radius + 1];
        for dy in 0..=radius {
            rows_of_width[(cells2 - dy * dy).isqrt()].push(dy);
        }
        let mut grown = vec![0u64; words * rows];
        let mut cur = vec![0u64; words];
        for s in 0..rows {
            let from = (j[0] + s) * stride + w0;
            cur.copy_from_slice(&bits[from..from + words]);
            cur[0] &= u64::MAX << (i[0] % 64);
            cur[words - 1] &= u64::MAX >> (63 - i[1] % 64);
            if cur.iter().all(|&w| w == 0) {
                continue;
            }
            for (width, dys) in rows_of_width.iter().enumerate() {
                // `cur` is this row's set cells grown `width` cells each way.
                for &dy in dys {
                    let targets = [Some(s + dy), (dy > 0).then(|| s.checked_sub(dy)).flatten()];
                    for t in targets.into_iter().flatten().filter(|&t| t < rows) {
                        for (o, c) in grown[t * words..(t + 1) * words].iter_mut().zip(&cur) {
                            *o |= c;
                        }
                    }
                }
                if width < radius {
                    let mut below = 0u64;
                    for w in 0..words {
                        let old = cur[w];
                        let above = if w + 1 < words { cur[w + 1] } else { 0 };
                        cur[w] = old | old << 1 | below >> 63 | old >> 1 | above << 63;
                        below = old;
                    }
                }
            }
        }
        Self {
            grown,
            words,
            first_word: w0,
            first_row: j[0],
        }
    }
    /// Whether cell (`i`, `j`) of the window is near a set cell.
    fn at(&self, i: usize, j: usize) -> bool {
        let w = (j - self.first_row) * self.words + i / 64 - self.first_word;
        self.grown[w] >> (i % 64) & 1 == 1
    }
}

/// The cells waiting in a link's search (`link_search`), by their distance
/// from the start in 1/1024 of a cell.
trait LinkQueue {
    fn push(&mut self, key: u64, cell: usize);
    /// The waiting cell of the smallest distance; of equals, the highest
    /// cell index.
    fn pop(&mut self) -> Option<(u64, usize)>;
}
/// The definition: a heap of every waiting cell.
#[cfg(test)]
impl LinkQueue for std::collections::BinaryHeap<(std::cmp::Reverse<u64>, usize)> {
    fn push(&mut self, key: u64, cell: usize) {
        std::collections::BinaryHeap::push(self, (std::cmp::Reverse(key), cell));
    }
    fn pop(&mut self) -> Option<(u64, usize)> {
        std::collections::BinaryHeap::pop(self).map(|(key, cell)| (key.0, cell))
    }
}
/// The same order from bands one cell of distance wide. A cell is pushed
/// with a distance at least one whole cell beyond that of the cell being
/// expanded, so into a later band than the one being taken from: when a
/// band's turn comes it is complete, and sorted once it gives its cells in
/// the heap's order.
#[derive(Default)]
struct LinkBuckets {
    bands: Vec<Vec<(u64, usize)>>,
    /// The band being taken from, sorted so that its end is next; `next`
    /// is the band after it.
    current: Vec<(u64, usize)>,
    next: usize,
}
impl LinkQueue for LinkBuckets {
    fn push(&mut self, key: u64, cell: usize) {
        let band = (key >> 10) as usize;
        debug_assert!(band >= self.next || self.next == 0);
        if band >= self.bands.len() {
            self.bands.resize_with(band + 1, Vec::new);
        }
        self.bands[band].push((key, cell));
    }
    fn pop(&mut self) -> Option<(u64, usize)> {
        while self.current.is_empty() {
            if self.next >= self.bands.len() {
                return None;
            }
            self.current = std::mem::take(&mut self.bands[self.next]);
            self.next += 1;
            self.current
                .sort_unstable_by(|a, b| b.0.cmp(&a.0).then(a.1.cmp(&b.1)));
        }
        self.current.pop()
    }
}

/// The search of a routed link over a window of `size` cells (columns,
/// rows): shortest eight-neighbour distances from `start` in cells, until
/// `goal` is taken from the queue or the distance passes `limit`.
/// `enter(column, row, d)` says whether a cell may be stepped on from a
/// cell at distance `d`. Returns each cell's distance and the cell it was
/// reached from, and the goal's distance in the queue if the search ended
/// by taking the goal from it.
///
/// A cell's distance in the queue is in 1/1024 of a cell, rounded down, and
/// its neighbours' distances are counted on from that rounded value. So a
/// cell is queued at least 1024 (a straight step) or 1448 (a diagonal one:
/// 1024 times the square root of 2, rounded down) beyond the cell it was
/// reached from: `LINK_DIAGONAL` names the latter.
fn link_search(
    mut queue: impl LinkQueue,
    size: [usize; 2],
    start: usize,
    goal: usize,
    limit: f64,
    enter: impl Fn(usize, usize, f64) -> bool,
) -> (Vec<f64>, Vec<usize>, Option<f64>) {
    let n = size[0] * size[1];
    let mut dist = vec![f64::INFINITY; n];
    let mut prev = vec![usize::MAX; n];
    let mut at_goal = None;
    dist[start] = 0.;
    queue.push(0, start);
    while let Some((d, k)) = queue.pop() {
        let d = d as f64 / 1024.;
        if d > dist[k] + 1e-9 {
            continue;
        }
        if k == goal || d > limit {
            at_goal = (k == goal).then_some(d);
            break;
        }
        let (i, j) = ((k % size[0]) as i64, (k / size[0]) as i64);
        for (di, dj, step) in [
            (1, 0, 1.),
            (-1, 0, 1.),
            (0, 1, 1.),
            (0, -1, 1.),
            (1, 1, std::f64::consts::SQRT_2),
            (1, -1, std::f64::consts::SQRT_2),
            (-1, 1, std::f64::consts::SQRT_2),
            (-1, -1, std::f64::consts::SQRT_2),
        ] {
            let (ni, nj) = (i + di, j + dj);
            if ni < 0 || nj < 0 || ni >= size[0] as i64 || nj >= size[1] as i64 {
                continue;
            }
            let (ni, nj) = (ni as usize, nj as usize);
            let nk = nj * size[0] + ni;
            // A cell reached no sooner than before is left alone: asked
            // first, since most neighbours are (the answer is the same in
            // either order).
            let nd = d + step;
            if nd >= dist[nk] || nd.is_nan() || !enter(ni, nj, d) {
                continue;
            }
            dist[nk] = nd;
            prev[nk] = k;
            queue.push((nd * 1024.) as u64, nk);
        }
    }
    (dist, prev, at_goal)
}

/// What a diagonal step adds at least to a cell's distance in a link
/// search's queue (`link_search`), in 1/1024 of a cell.
const LINK_DIAGONAL: u64 = 1448;

/// The shortest eight-neighbour path between two cells `dx` and `dy` apart
/// with nothing in the way, counted as `link_search` counts at least: in
/// 1/1024 of a cell.
fn link_distance(dx: usize, dy: usize) -> u64 {
    let (long, short) = (dx.max(dy) as u64, dx.min(dy) as u64);
    1024 * (long - short) + LINK_DIAGONAL * short
}

/// The set cells of `pass` (rows of `words` words) that a path from `start`
/// (column, row) over set cells reaches within `bound`, in 1/1024 of a
/// cell: a step to one of the eight neighbours counted 1024 or
/// `LINK_DIAGONAL`, as `link_search` counts at least. Dijkstra's algorithm
/// in whole numbers, stopped at `bound`.
fn cells_within(pass: &[u64], words: usize, start: (usize, usize), bound: u64) -> Vec<u64> {
    let (cols, rows) = (words * 64, pass.len() / words);
    let set = |i: usize, j: usize| pass[j * words + i / 64] >> (i % 64) & 1 == 1;
    let mut dist = vec![u64::MAX; cols * rows];
    let mut within = vec![0u64; pass.len()];
    let mut queue = LinkBuckets::default();
    let first = start.1 * cols + start.0;
    dist[first] = 0;
    queue.push(0, first);
    while let Some((d, k)) = queue.pop() {
        if d > dist[k] {
            continue;
        }
        if d > bound {
            break;
        }
        let (i, j) = (k % cols, k / cols);
        within[j * words + i / 64] |= 1 << (i % 64);
        for (di, dj, step) in [
            (1i64, 0i64, 1024),
            (-1, 0, 1024),
            (0, 1, 1024),
            (0, -1, 1024),
            (1, 1, LINK_DIAGONAL),
            (1, -1, LINK_DIAGONAL),
            (-1, 1, LINK_DIAGONAL),
            (-1, -1, LINK_DIAGONAL),
        ] {
            let (ni, nj) = (i as i64 + di, j as i64 + dj);
            if ni < 0 || nj < 0 || ni >= cols as i64 || nj >= rows as i64 {
                continue;
            }
            let (ni, nj) = (ni as usize, nj as usize);
            let nk = nj * cols + ni;
            let nd = d + step;
            if nd < dist[nk] && set(ni, nj) {
                dist[nk] = nd;
                queue.push(nd, nk);
            }
        }
    }
    within
}

/// Cells a stay-down link from one start can possibly reach, for one state
/// of the layer's material (`Layer::link_reach`).
struct LinkReach {
    version: u64,
    start: usize,
    max_len: u64,
    /// The box of cells this was worked out in: columns and rows.
    i: [usize; 2],
    j: [usize; 2],
    /// Rows `j[0]..` of `words` words from word `i[0] / 64` of the grid's
    /// rows.
    reach: Vec<u64>,
    words: usize,
}
impl LinkReach {
    /// Whether a search in the window of columns `i` and rows `j` is
    /// covered by this box, so that what this says of its goal holds.
    fn covers(&self, i: [usize; 2], j: [usize; 2]) -> bool {
        self.i[0] <= i[0] && i[1] <= self.i[1] && self.j[0] <= j[0] && j[1] <= self.j[1]
    }
    /// Whether cell (`i`, `j`) of the box is out of the link's reach.
    fn excludes(&self, i: usize, j: usize) -> bool {
        let w = (j - self.j[0]) * self.words + i / 64 - self.i[0] / 64;
        self.reach[w] >> (i % 64) & 1 == 0
    }
}

/// One bit per fine cell, in rows of whole words: where the layer has
/// material (`Layer::material`), and where that material is reachable.
/// Kept current by everything that changes a cleared height (`Layer::cut`,
/// `Layer::undo`), so a search for material reads set bits instead of
/// testing every cell of a disk (#202).
#[derive(Default)]
struct MaterialBits {
    /// Words per row.
    stride: usize,
    any: Vec<u64>,
    reach: Vec<u64>,
    /// The layer's allowed centres (they do not change).
    allowed: Vec<u64>,
    /// Cells this operation has not cleared down to the layer yet (cleared
    /// height above it), with or without material: the cells a cut changes.
    uncut: Vec<u64>,
    /// Counts the changes of `any`: what was derived from the bits at one
    /// count holds as long as the count stands.
    version: u64,
}
impl MaterialBits {
    fn assign(&mut self, g: &Grid, k: usize, material: bool, reachable: bool) {
        let (w, bit) = (
            (k / g.nx) * self.stride + (k % g.nx) / 64,
            1u64 << (k % g.nx % 64),
        );
        self.assign_at(w, bit, material, reachable);
    }
    /// `assign` for the cell at bit `bit` of word `w`.
    fn assign_at(&mut self, w: usize, bit: u64, material: bool, reachable: bool) {
        self.version += 1;
        if material {
            self.any[w] |= bit;
        } else {
            self.any[w] &= !bit;
        }
        if material && reachable {
            self.reach[w] |= bit;
        } else {
            self.reach[w] &= !bit;
        }
    }
}

/// Heights this operation's own committed layers cleared on the fine grid,
/// handed from one layer to the next. Valid only for the stock generation it
/// was produced against.
pub(crate) struct FrontMemory {
    grid: Grid,
    cleared: Vec<f64>,
    generation: u64,
}

fn is_zero(n: &usize) -> bool {
    *n == 0
}
fn is_zero_f64(n: &f64) -> bool {
    *n == 0.
}
/// Longest link stretch proven at once for the link feed (mm).
const LINK_PIECE: f64 = 2.;

#[derive(Clone, Debug, Serialize)]
pub struct EngagementFrontReport {
    pub strategy: &'static str,
    pub target_z_mm: f64,
    pub target_radial_engagement_mm: f64,
    /// The declared width as a contact angle, acos(1 - w/r): the limit every
    /// cutting and stay-down link stroke is proven against.
    pub engagement_limit_deg: f64,
    /// Largest contact angle bound the exact proof returned for a stroke of
    /// this layer (at most the limit), and the radial width of a straight
    /// pass with that contact, r (1 - cos).
    pub proven_max_engagement_deg: f64,
    pub proven_max_engagement_mm: f64,
    /// Strokes proven within the limit, and passes planned again with a
    /// smaller steering width because a stroke of theirs was not.
    pub engagement_proved_strokes: usize,
    pub engagement_replans: usize,
    /// Strokes in confined material (a pocket barely wider than the cutter)
    /// that could not be proven within the limit at half the declared
    /// width. Like the helical entry they follow, they are cut as entry
    /// moves at the entry feed, and only when the entry policy allows a
    /// helix. Their length, and the largest contact bound proven for one.
    #[serde(skip_serializing_if = "is_zero")]
    pub confined_strokes: usize,
    #[serde(skip_serializing_if = "is_zero_f64")]
    pub confined_stroke_mm: f64,
    #[serde(skip_serializing_if = "is_zero_f64")]
    pub confined_max_engagement_deg: f64,
    /// Largest radial width the steering grid predicted for a planned step.
    /// A grid estimate (see `engagement_grid_mm`); the proven bound is
    /// `proven_max_engagement_deg`.
    pub max_estimated_engagement_mm: f64,
    pub engagement_grid_mm: f64,
    pub passes: usize,
    pub stay_down_links: usize,
    /// Stay-down links routed around uncut stock through cleared area.
    pub routed_links: usize,
    pub lifts: usize,
    pub stock_edge_entries: usize,
    pub helical_entries: usize,
    pub entry_turns: usize,
    /// Ramp entries (`RampEntry`) into enclosed material: descents along a
    /// straight path, each followed by one flat pass along it at depth. The
    /// legs made, the length of full-width slot they opened (entry motion
    /// at the entry feed, like a helix: not bounded by the engagement
    /// limit) and the steepest leg in degrees (never above the declared
    /// angle).
    #[serde(skip_serializing_if = "is_zero")]
    pub ramp_entries: usize,
    #[serde(skip_serializing_if = "is_zero")]
    pub ramp_legs: usize,
    #[serde(skip_serializing_if = "is_zero_f64")]
    pub ramp_slot_mm: f64,
    #[serde(skip_serializing_if = "is_zero_f64")]
    pub max_ramp_angle_deg: f64,
    /// With a `RampEntry`: material left beside a ramp's slot, in a pocket
    /// too narrow for a pass to start from cleared stock, entered sideways
    /// from that slot at depth instead of by another ramp through the air
    /// the first one left.
    #[serde(skip_serializing_if = "is_zero")]
    pub slot_side_entries: usize,
    /// Entries the helix search chose whose whole bore was proven air (by
    /// the committed sweeps or the stock cell bounds): a straight plunge.
    pub air_entries: usize,
    /// Vertical entries made by plunging a few stock cells back from the
    /// material (where the cell bounds proved the rapid could come lower)
    /// and feeding across at depth.
    pub offset_descents: usize,
    /// Rapids between passes that travelled below `retract_z_mm`, at
    /// `travel_clearance_mm` above the whole-cell stock bounds under every
    /// tool section over the rectangle their ends span.
    pub lowered_travels: usize,
    /// Passes that cleared a remnant too thin for a full step (a sliver
    /// left between passes) from beside it at depth, instead of a helix.
    pub remnant_passes: usize,
    /// Link stretches run at `link_feed_mm_min` (proven clear of stock), and
    /// their total length.
    #[serde(skip_serializing_if = "is_zero")]
    pub fast_links: usize,
    #[serde(skip_serializing_if = "is_zero_f64")]
    pub fast_link_mm: f64,
    /// Link stretches fed between the cutting and link feeds by radial chip
    /// thinning (`link_chip_thinning`), and their total length.
    #[serde(skip_serializing_if = "is_zero")]
    pub thinned_links: usize,
    #[serde(skip_serializing_if = "is_zero_f64")]
    pub thinned_link_mm: f64,
    pub unproven_air_starts: usize,
    /// Wall passes (see `EngagementFrontPlan::with_wall_pass` and
    /// `with_wall_runs`): loops, or runs of them, cut along the region's
    /// walls after the level is cleared, and those left uncut because no
    /// way onto them was found or their engagement was not proven where the
    /// entry policy allows no entry motion.
    #[serde(skip_serializing_if = "is_zero")]
    pub wall_passes: usize,
    #[serde(skip_serializing_if = "is_zero")]
    pub wall_passes_skipped: usize,
    pub grid_refreshes: usize,
    /// Why passes ended: material exhausted ahead, the far side would
    /// overload, the front thinned out, or the region boundary blocked it.
    pub pass_ends_exhausted: usize,
    pub pass_ends_overload: usize,
    pub pass_ends_thin: usize,
    pub pass_ends_boundary: usize,
    pub certified_strokes: usize,
    pub arcs: usize,
    /// Fitted arcs that bowed out of the region between their points and
    /// were cut as their points' certified polyline instead.
    #[serde(skip_serializing_if = "is_zero")]
    pub refitted_arcs: usize,
    pub motions: usize,
    pub stock_top_carried: bool,
    pub geometric_residual_status: ResidualStatus,
    /// All the stock left in the level's region at this level, by the
    /// steering grid and the stock's whole-cell bounds: what the level was
    /// asked to clear and did not, whether or not the planner has a
    /// position there (#250). It is the sum of the three parts below.
    pub geometric_residual_area_mm2: Option<f64>,
    /// The part within reach of the planner's own positions: slivers too
    /// thin to cut, and the areas no entry reaches (`unentered_areas`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub residual_reachable_area_mm2: Option<f64>,
    /// The part the cutter reaches in the region but the planner's
    /// positions do not: they keep `grid_clearance` off the walls, so this
    /// is the skin along the walls (which a pocket's wall pass takes) and
    /// any area tighter than the planner resolves.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub residual_unresolved_area_mm2: Option<f64>,
    /// The part beyond the cutter's reach: corners, the radial allowance,
    /// parts of the region narrower than the cutter.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub residual_unreachable_area_mm2: Option<f64>,
    /// Areas with material that no entry reaches, found when the level
    /// starts (`crate::entry::UnenteredArea`). An `AdaptivePocket` is
    /// refused with the first; a roughing layer lists them here.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub unentered_areas: Vec<crate::entry::UnenteredArea>,
    pub diagnostics: Vec<String>,
}

impl Default for EngagementFrontReport {
    fn default() -> Self {
        Self {
            strategy: "engagement_front",
            target_z_mm: 0.,
            target_radial_engagement_mm: 0.,
            engagement_limit_deg: 0.,
            proven_max_engagement_deg: 0.,
            proven_max_engagement_mm: 0.,
            engagement_proved_strokes: 0,
            engagement_replans: 0,
            confined_strokes: 0,
            confined_stroke_mm: 0.,
            confined_max_engagement_deg: 0.,
            max_estimated_engagement_mm: 0.,
            engagement_grid_mm: 0.,
            passes: 0,
            stay_down_links: 0,
            routed_links: 0,
            lifts: 0,
            stock_edge_entries: 0,
            helical_entries: 0,
            entry_turns: 0,
            ramp_entries: 0,
            ramp_legs: 0,
            ramp_slot_mm: 0.,
            max_ramp_angle_deg: 0.,
            slot_side_entries: 0,
            air_entries: 0,
            offset_descents: 0,
            lowered_travels: 0,
            remnant_passes: 0,
            fast_links: 0,
            fast_link_mm: 0.,
            thinned_links: 0,
            thinned_link_mm: 0.,
            unproven_air_starts: 0,
            wall_passes: 0,
            wall_passes_skipped: 0,
            grid_refreshes: 0,
            pass_ends_exhausted: 0,
            pass_ends_overload: 0,
            pass_ends_thin: 0,
            pass_ends_boundary: 0,
            certified_strokes: 0,
            arcs: 0,
            refitted_arcs: 0,
            motions: 0,
            stock_top_carried: false,
            geometric_residual_status: ResidualStatus::Pending,
            geometric_residual_area_mm2: None,
            residual_reachable_area_mm2: None,
            residual_unresolved_area_mm2: None,
            residual_unreachable_area_mm2: None,
            unentered_areas: vec![],
            diagnostics: vec![],
        }
    }
}

/// The allowed cells that had material when `Layer::most_open` was first
/// asked, by room (most first; of equals the lower cell index first), and
/// how far down that order the cells are known to be cut or dead.
struct OpenOrder {
    /// The clearance the rooms are counted beyond (its bits).
    need: u64,
    cells: Vec<(f64, usize)>,
    next: usize,
}
impl OpenOrder {
    fn order(a: &(f64, usize), b: &(f64, usize)) -> std::cmp::Ordering {
        b.0.total_cmp(&a.0).then(a.1.cmp(&b.1))
    }
}

struct Layer {
    g: Grid,
    /// Top of possible material per fine cell.
    top: Vec<f64>,
    /// This operation's cleared height per fine cell.
    cleared: Vec<f64>,
    /// Cell centres inside the level's region.
    inside: Vec<bool>,
    allowed: Vec<bool>,
    /// Material the cutter can reach from an allowed centre.
    reachable: Vec<bool>,
    /// Distance from the cell centre to the outside of the region (mm):
    /// `room()`, worked out from `outside` when first asked for (the rare
    /// entries into enclosed material ask; a pass does not).
    room: std::cell::OnceCell<Vec<f32>>,
    /// Cells outside the region.
    outside: Vec<bool>,
    dead: Vec<bool>,
    candidates: Vec<usize>,
    /// Starts whose pass found the material beside them too thin to cut.
    thin: Vec<Point>,
    z: f64,
    material_guard: f64,
    /// Cleared heights overwritten since the journal was last emptied, so a
    /// pass whose engagement was not proven can be planned again.
    journal: Vec<(usize, f64)>,
    /// `material` and `reachable` as bits (see `sync_bits`).
    bits: MaterialBits,
    /// See `most_open`.
    open: std::cell::RefCell<Option<OpenOrder>>,
}
impl Layer {
    fn room(&self) -> &[f32] {
        self.room.get_or_init(|| {
            edt(&self.g, &self.outside)
                .into_iter()
                .map(|d| d as f32)
                .collect()
        })
    }
    /// The allowed cell with material that is not dead and has the most
    /// room beyond `need` (the first of equals, by cell index), and that
    /// room: where an entry into enclosed material is tried.
    ///
    /// Asked once for every entry or start that is refused, which on a
    /// large part is thousands of times a layer (#202: one layer of a
    /// 430 mm part spent seven minutes testing every cell of the grid each
    /// time). The allowed cells with material are put in order once, most
    /// room first (`OpenOrder`), and the answer is the first of them that
    /// still has material and is not dead: cells only leave (cut, or marked
    /// dead) except where `reopened` is told of one coming back.
    fn most_open(&self, need: f64) -> Option<(f64, usize)> {
        let room = self.room();
        let mut open = self.open.borrow_mut();
        if open.as_ref().is_none_or(|o| o.need != need.to_bits()) {
            let stride = self.bits.stride;
            let mut cells = vec![];
            for (w, (any, allowed)) in self.bits.any.iter().zip(&self.bits.allowed).enumerate() {
                let mut word = any & allowed;
                while word != 0 {
                    let i = (w % stride) * 64 + word.trailing_zeros() as usize;
                    word &= word - 1;
                    let k = (w / stride) * self.g.nx + i;
                    cells.push((room[k] as f64 - need, k));
                }
            }
            cells.sort_unstable_by(OpenOrder::order);
            *open = Some(OpenOrder {
                need: need.to_bits(),
                cells,
                next: 0,
            });
        }
        let order = open.as_mut().unwrap();
        while order
            .cells
            .get(order.next)
            .is_some_and(|&(_, k)| self.dead[k] || !self.material(k))
        {
            order.next += 1;
        }
        let best = order.cells.get(order.next).copied();
        #[cfg(test)]
        {
            shortcut_taken(3);
            let mut scan: Option<(f64, usize)> = None;
            for k in 0..self.top.len() {
                if self.allowed[k] && !self.dead[k] && self.material(k) {
                    let room = room[k] as f64 - need;
                    if scan.is_none_or(|(b, _)| room > b) {
                        scan = Some((room, k));
                    }
                }
            }
            assert_eq!(
                best.map(|(room, k)| (room.to_bits(), k)),
                scan.map(|(room, k)| (room.to_bits(), k))
            );
        }
        best
    }
    /// Cell `k` may have material and be alive again (an undone cut, a
    /// revived start): `most_open` looks at it again.
    fn reopened(&self, k: usize) {
        let mut open = self.open.borrow_mut();
        let Some(order) = open.as_mut() else {
            return;
        };
        if !self.allowed[k] || !self.material(k) {
            return;
        }
        let key = (self.room()[k] as f64 - f64::from_bits(order.need), k);
        match order
            .cells
            .binary_search_by(|cell| OpenOrder::order(cell, &key))
        {
            Ok(at) => order.next = order.next.min(at),
            // Not among the cells put in order (it had no material then).
            Err(_) => *open = None,
        }
    }
    /// Rebuild the material bits from the heights. Needed once, when the
    /// layer is made: `cut` and `undo` keep them current after that.
    fn sync_bits(&mut self) {
        *self.open.borrow_mut() = None;
        let g = self.g;
        let stride = g.nx.div_ceil(64);
        self.bits = MaterialBits {
            stride,
            any: vec![0; stride * g.ny],
            reach: vec![0; stride * g.ny],
            allowed: vec![0; stride * g.ny],
            uncut: vec![0; stride * g.ny],
            version: self.bits.version + 1,
        };
        for k in 0..g.nx * g.ny {
            if self.material(k) {
                self.bits.assign(&g, k, true, self.reachable[k]);
            }
            let (w, bit) = ((k / g.nx) * stride + (k % g.nx) / 64, 1 << (k % g.nx % 64));
            if self.allowed[k] {
                self.bits.allowed[w] |= bit;
            }
            if self.cleared[k] > self.z {
                self.bits.uncut[w] |= bit;
            }
        }
    }
    fn undo(&mut self, to: usize) {
        while self.journal.len() > to {
            let (k, old) = self.journal.pop().unwrap();
            self.cleared[k] = old;
            let material = self.material(k);
            self.bits.assign(&self.g, k, material, self.reachable[k]);
            let (w, bit) = (
                (k / self.g.nx) * self.bits.stride + (k % self.g.nx) / 64,
                1 << (k % self.g.nx % 64),
            );
            if old > self.z {
                self.bits.uncut[w] |= bit;
            } else {
                self.bits.uncut[w] &= !bit;
            }
            if material {
                self.reopened(k);
            }
        }
    }
    fn material(&self, k: usize) -> bool {
        self.top[k].min(self.cleared[k]) > self.z + self.material_guard
    }
    fn allowed_at(&self, p: Point) -> bool {
        self.g
            .cell(p)
            .is_some_and(|(i, j)| self.allowed[self.g.idx(i, j)])
    }
    /// Mark the disk cleared; returns how many material cells it cleared.
    ///
    /// Only the cells of the disk not yet cleared to the layer change, and
    /// a pass's next step finds few: they are read from the `uncut` bits,
    /// in the order a scan of the disk meets them (the journal's order).
    fn cut(&mut self, p: Point, rad: f64) -> usize {
        let (z, floor) = (self.z, self.z + self.material_guard);
        let g = self.g;
        #[cfg(test)]
        let scan = {
            let mut cells = vec![];
            g.for_disk_cells(p, rad, |k, _| {
                if self.cleared[k] > z {
                    cells.push(k);
                }
            });
            cells
        };
        let Some((lo, hi)) = g.disk_box(p, rad) else {
            return 0;
        };
        if lo[0] > hi[0] {
            return 0;
        }
        let (sure, rim) = Grid::rim_bounds(rad);
        let stride = self.bits.stride;
        let mark = self.journal.len();
        let mut removed = 0;
        for j in lo[1]..=hi[1] {
            let dy = g.y0 + (j as f64 + 0.5) * g.c - p[1];
            let h2 = rim - dy * dy;
            if h2 < 0. {
                continue;
            }
            let Some((i0, i1)) = g.span(g.x0, p[0], h2.sqrt(), lo[0], hi[0]) else {
                continue;
            };
            let mut from = i0;
            while let Some(i) =
                bit_at_or_above(&self.bits.uncut[j * stride..(j + 1) * stride], from, i1)
            {
                from = i + 1;
                let x = g.x0 + (i as f64 + 0.5) * g.c;
                if !Grid::within(x - p[0], dy, rad, sure, rim) {
                    continue;
                }
                let k = g.idx(i, j);
                removed += usize::from(self.top[k].min(self.cleared[k]) > floor);
                self.journal.push((k, self.cleared[k]));
                self.cleared[k] = z;
                let (w, bit) = (j * stride + i / 64, 1 << (i % 64));
                self.bits.uncut[w] &= !bit;
                self.bits
                    .assign_at(w, bit, self.top[k].min(z) > floor, self.reachable[k]);
            }
        }
        #[cfg(test)]
        assert!(
            self.journal[mark..].iter().map(|(k, _)| k).eq(scan.iter()),
            "cut at {p:?} of {rad}"
        );
        let _ = mark;
        removed
    }
    /// Every cell a routed link from cell `start` can come to within its
    /// length limit (`routed_link`, `max_len` mm) in any search window
    /// inside the box of columns `i` and rows `j`, and possibly more: a
    /// goal outside this set is one the search does not reach.
    ///
    /// The search steps between cells that are allowed and have no material
    /// within `tight` mm, in a window around start and goal that closes a
    /// ring at its border because it does not see the material beyond; and
    /// within `relaxed` cells of path from the start it may also step on
    /// other cells. A cell it steps on, away from the start, is therefore
    /// allowed and has no material of the box within `tight` (the window
    /// lies in the box, and its ring is wider than `tight`). So every path
    /// the search follows is a path over such cells, the square of
    /// `relaxed` cells around the start counted with them.
    ///
    /// The search's distance of a cell, in 1/1024 of a cell, is at least
    /// the length of its path counted 1024 for a straight step and
    /// `LINK_DIAGONAL` for a diagonal one (`link_search`), and it returns a
    /// link only when the goal's distance is at most `max_len / c` cells. A
    /// goal whose shortest path over those cells, counted that way
    /// (`cells_within`), is longer than that is therefore not returned: its
    /// distance exceeds the limit, or it was never reached.
    fn link_reach(
        &self,
        start: (usize, usize),
        tight: f64,
        relaxed: usize,
        i: [usize; 2],
        j: [usize; 2],
        max_len: f64,
    ) -> LinkReach {
        let g = &self.g;
        let stride = self.bits.stride;
        let near =
            cells2_within(g.c, tight, false).map(|d2| Near::new(&self.bits.any, stride, i, j, d2));
        let (w0, words, rows) = (i[0] / 64, i[1] / 64 - i[0] / 64 + 1, j[1] - j[0] + 1);
        let mut pass = vec![0u64; words * rows];
        for row in 0..rows {
            for w in 0..words {
                let allowed = self.bits.allowed[(j[0] + row) * stride + w0 + w];
                let blocked = near.as_ref().map_or(0, |n| n.grown[row * words + w]);
                pass[row * words + w] = allowed & !blocked;
            }
        }
        for row in start.1.saturating_sub(relaxed).max(j[0])..=(start.1 + relaxed).min(j[1]) {
            for col in start.0.saturating_sub(relaxed).max(i[0])..=(start.0 + relaxed).min(i[1]) {
                pass[(row - j[0]) * words + col / 64 - w0] |= 1 << (col % 64);
            }
        }
        LinkReach {
            version: self.bits.version,
            start: g.idx(start.0, start.1),
            max_len: max_len.to_bits(),
            i,
            j,
            // (A whole number at or above the limit in 1/1024 of a cell.)
            reach: cells_within(
                &pass,
                words,
                (start.0 - w0 * 64, start.1 - j[0]),
                (max_len / g.c * 1024. * (1. + 1e-9)).ceil() as u64 + 1,
            ),
            words,
        }
    }
    /// What a `for_disk(p, rad)` scan keeping the first cell of the smallest
    /// distance finds among the cells whose bit is set in `bits`: that
    /// cell, its centre and its distance. The same cell, to the bit:
    ///
    /// 1. Rows outward from `p`, the set bit nearest `p`'s column on each
    ///    side, give an upper bound on the smallest squared distance.
    /// 2. Every set cell of the scan's own box within that bound (and
    ///    `DISTANCE_SLACK`) is then measured exactly as the scan measures
    ///    it (`hypot`, `d <= rad`), in the scan's order, and the first of
    ///    the smallest kept. A cell left out is farther than a measured
    ///    one by more than the slack, so the scan would not have kept it;
    ///    cells at equal distance are all measured, so the scan's choice
    ///    between them stands.
    fn nearest_set(&self, bits: &[u64], p: Point, rad: f64) -> Option<(usize, Point, f64)> {
        let g = &self.g;
        let (lo, hi) = g.disk_box(p, rad)?;
        if lo[0] > hi[0] || lo[1] > hi[1] {
            return None;
        }
        let stride = self.bits.stride;
        let row = |j: usize| &bits[j * stride..(j + 1) * stride];
        let cx = |i: usize| g.x0 + (i as f64 + 0.5) * g.c;
        let cy = |j: usize| g.y0 + (j as f64 + 0.5) * g.c;
        let mut bound = rad * rad * (1. + DISTANCE_SLACK);
        let column = ((p[0] - g.x0) / g.c - 0.5).round();
        let look = |j: usize, bound: &mut f64| {
            let dy = cy(j) - p[1];
            let h2 = *bound - dy * dy;
            if h2 < 0. {
                return;
            }
            let Some((i0, i1)) = g.span(g.x0, p[0], h2.sqrt(), lo[0], hi[0]) else {
                return;
            };
            let ic = column.clamp(i0 as f64, i1 as f64) as usize;
            let sides = [
                bit_at_or_below(row(j), ic, i0),
                bit_at_or_above(row(j), ic, i1),
            ];
            for i in sides.into_iter().flatten() {
                let dx = cx(i) - p[0];
                *bound = bound.min(dx * dx + dy * dy);
            }
        };
        let middle = ((p[1] - g.y0) / g.c - 0.5)
            .round()
            .clamp(lo[1] as f64, hi[1] as f64) as usize;
        let (mut up, mut down) = (middle, middle);
        let (mut more_up, mut more_down) = (true, true);
        while more_up || more_down {
            if more_up {
                let dy = cy(up) - p[1];
                if dy > 0. && dy * dy > bound {
                    more_up = false;
                } else {
                    look(up, &mut bound);
                    up += 1;
                    more_up = up <= hi[1];
                }
            }
            if more_down {
                if down == lo[1] {
                    more_down = false;
                } else {
                    down -= 1;
                    let dy = cy(down) - p[1];
                    if dy < 0. && dy * dy > bound {
                        more_down = false;
                    } else {
                        look(down, &mut bound);
                    }
                }
            }
        }
        let within = bound * (1. + DISTANCE_SLACK);
        let (j0, j1) = g.span(g.y0, p[1], within.sqrt(), lo[1], hi[1])?;
        let mut best: Option<(usize, Point, f64)> = None;
        for j in j0..=j1 {
            let y = cy(j);
            let h2 = within - (y - p[1]) * (y - p[1]);
            if h2 < 0. {
                continue;
            }
            let Some((i0, i1)) = g.span(g.x0, p[0], h2.sqrt(), lo[0], hi[0]) else {
                continue;
            };
            let mut from = i0;
            while let Some(i) = bit_at_or_above(row(j), from, i1) {
                let x = cx(i);
                let d = (x - p[0]).hypot(y - p[1]);
                if d <= rad && best.is_none_or(|(_, _, b)| d < b) {
                    best = Some((g.idx(i, j), [x, y], d));
                }
                from = i + 1;
            }
        }
        best
    }
    /// Whether a `for_disk(p, rad)` scan meets a cell whose bit is set in
    /// `bits` at a distance of at most `limit`: each set cell that can be
    /// that near is measured exactly as the scan measures it.
    fn any_set_within(&self, bits: &[u64], p: Point, rad: f64, limit: f64) -> bool {
        let g = &self.g;
        let Some((lo, hi)) = g.disk_box(p, rad) else {
            return false;
        };
        if lo[0] > hi[0] || lo[1] > hi[1] {
            return false;
        }
        let stride = self.bits.stride;
        let within = limit * limit * (1. + DISTANCE_SLACK);
        let Some((j0, j1)) = g.span(g.y0, p[1], within.sqrt(), lo[1], hi[1]) else {
            return false;
        };
        for j in j0..=j1 {
            let y = g.y0 + (j as f64 + 0.5) * g.c;
            let h2 = within - (y - p[1]) * (y - p[1]);
            if h2 < 0. {
                continue;
            }
            let Some((i0, i1)) = g.span(g.x0, p[0], h2.sqrt(), lo[0], hi[0]) else {
                continue;
            };
            let row = &bits[j * stride..(j + 1) * stride];
            let mut from = i0;
            while let Some(i) = bit_at_or_above(row, from, i1) {
                let x = g.x0 + (i as f64 + 0.5) * g.c;
                let d = (x - p[0]).hypot(y - p[1]);
                if d <= rad && d <= limit {
                    return true;
                }
                from = i + 1;
            }
        }
        false
    }
    /// Material cell centres a step of up to `reach` from `p` can meet.
    /// Only reachable material counts: stock no allowed centre can cut (the
    /// part and its allowance beyond the region) lies just past the cutter
    /// along every wall, and counting it made a pass ride the wall through
    /// cleared air on material it could never remove.
    ///
    /// Read from the material bits: the cells a scan of the disk
    /// (`gather_scan`) collects, in the same order.
    fn gather(&self, p: Point, reach: f64, out: &mut Vec<Point>) {
        out.clear();
        let g = &self.g;
        let Some((lo, hi)) = g.disk_box(p, reach) else {
            return;
        };
        if lo[0] > hi[0] {
            return;
        }
        let (sure, rim) = Grid::rim_bounds(reach);
        let stride = self.bits.stride;
        for j in lo[1]..=hi[1] {
            let y = g.y0 + (j as f64 + 0.5) * g.c;
            let dy = y - p[1];
            let h2 = rim - dy * dy;
            if h2 < 0. {
                continue;
            }
            let Some((i0, i1)) = g.span(g.x0, p[0], h2.sqrt(), lo[0], hi[0]) else {
                continue;
            };
            let row = &self.bits.reach[j * stride..(j + 1) * stride];
            let mut from = i0;
            while let Some(i) = bit_at_or_above(row, from, i1) {
                let x = g.x0 + (i as f64 + 0.5) * g.c;
                if Grid::within(x - p[0], dy, reach, sure, rim) {
                    out.push([x, y]);
                }
                from = i + 1;
            }
        }
        #[cfg(test)]
        {
            let mut scan = vec![];
            self.gather_scan(p, reach, &mut scan);
            let exact =
                |v: &[Point]| -> Vec<[u64; 2]> { v.iter().map(|q| q.map(f64::to_bits)).collect() };
            assert_eq!(exact(out), exact(&scan), "gather at {p:?} within {reach}");
        }
    }
    /// `gather` as a scan of every cell of the disk.
    #[cfg(test)]
    fn gather_scan(&self, p: Point, reach: f64, out: &mut Vec<Point>) {
        out.clear();
        self.g.for_disk(p, reach, |k, q, _| {
            if self.reachable[k] && self.material(k) {
                out.push(q);
            }
        });
    }
}

/// A cached start candidate (cell `k`, at index `at` of `Layer::candidates`)
/// and its squared distance from the cutter. Ordered by that, then by the
/// place in the list.
struct NearStart {
    d2: f64,
    at: usize,
    k: usize,
}
impl Ord for NearStart {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.d2.total_cmp(&other.d2).then(self.at.cmp(&other.at))
    }
}
impl PartialOrd for NearStart {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl PartialEq for NearStart {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other).is_eq()
    }
}
impl Eq for NearStart {}

/// The cached start candidates of a layer by distance from `pos`, measured
/// once. `take` hands out what a scan of the whole list for the nearest
/// would (`nearest_start_scan`: the smallest distance, and of equal ones
/// the first in the list), and removes it from the list the same way, so
/// the list stays in the same order.
///
/// The queue is ordered by squared distance, which costs no square root
/// per candidate. The scan compares distances (`hypot`), and two
/// candidates whose squares differ in the last places can have the same
/// distance: so the candidates whose squares are within `DISTANCE_SLACK` of
/// the smallest are measured as the scan measures them, and the scan's
/// choice among them is taken. A candidate farther than that is farther by
/// more than rounding and is not the scan's choice.
struct StartQueue {
    pos: Point,
    heap: std::collections::BinaryHeap<std::cmp::Reverse<NearStart>>,
}
impl StartQueue {
    fn offset(layer: &Layer, pos: Point, k: usize) -> Point {
        let q = layer.g.center(k);
        [q[0] - pos[0], q[1] - pos[1]]
    }
    fn near(layer: &Layer, pos: Point, at: usize) -> std::cmp::Reverse<NearStart> {
        let k = layer.candidates[at];
        let v = Self::offset(layer, pos, k);
        std::cmp::Reverse(NearStart {
            d2: v[0] * v[0] + v[1] * v[1],
            at,
            k,
        })
    }
    fn new(layer: &Layer, pos: Point) -> Self {
        let all: Vec<_> = (0..layer.candidates.len())
            .map(|at| Self::near(layer, pos, at))
            .collect();
        Self {
            pos,
            heap: all.into(),
        }
    }
    /// Remove the nearest candidate from the layer's list and return it.
    fn take(&mut self, layer: &mut Layer) -> Option<usize> {
        // The list fills a removed slot with its last entry. That entry is
        // queued again under its new slot (below); its old queue entry
        // names a slot past the list's end from then on (the list only
        // shrinks here) and is skipped.
        let current = |layer: &Layer, n: &NearStart| layer.candidates.get(n.at) == Some(&n.k);
        let mut best: Option<(f64, NearStart)> = None;
        let mut others = vec![];
        let mut within = f64::INFINITY;
        while let Some(std::cmp::Reverse(n)) = self.heap.peek() {
            if n.d2 > within {
                break;
            }
            let std::cmp::Reverse(n) = self.heap.pop().unwrap();
            if !current(layer, &n) {
                continue;
            }
            if best.is_none() {
                within = n.d2 * (1. + DISTANCE_SLACK);
            }
            let v = Self::offset(layer, self.pos, n.k);
            let d = v[0].hypot(v[1]);
            let nearer = match &best {
                Some((b, held)) => d.total_cmp(b).then(n.at.cmp(&held.at)).is_lt(),
                None => true,
            };
            if nearer {
                others.extend(best.replace((d, n)).map(|(_, n)| n));
            } else {
                others.push(n);
            }
        }
        self.heap.extend(others.into_iter().map(std::cmp::Reverse));
        let best = best.map(|(_, n)| (n.at, n.k));
        #[cfg(test)]
        assert_eq!(best, nearest_start_scan(layer, self.pos));
        let (at, k) = best?;
        layer.candidates.swap_remove(at);
        if at < layer.candidates.len() {
            self.heap.push(Self::near(layer, self.pos, at));
        }
        Some(k)
    }
}

/// The nearest cached start candidate to `pos` by a scan of the whole list
/// (index in the list, cell): the definition the queue in `plan_pass` is
/// tested against.
#[cfg(test)]
fn nearest_start_scan(layer: &Layer, pos: Point) -> Option<(usize, usize)> {
    layer
        .candidates
        .iter()
        .enumerate()
        .map(|(i, &k)| {
            let q = layer.g.center(k);
            (i, k, (q[0] - pos[0]).hypot(q[1] - pos[1]))
        })
        .min_by(|a, b| a.2.total_cmp(&b.2))
        .map(|(i, k, _)| (i, k))
}

/// Smoothed material area (mm²) inside the cutter disk centred at `p`.
///
/// A cell counts 1 when its centre is half a cell or more inside the disk,
/// 0 when half a cell or more outside, and in between by its distance.
/// Only the cells in between need that distance (a square root): the
/// squared distance tells the others apart (`smoothing_band`), and they add
/// exactly 1 or nothing, as the formula gives for them (`area_plain`).
#[cfg(test)]
fn area(cells: &[Point], p: Point, r: f64, c: f64) -> f64 {
    let (full, none) = smoothing_band(r, c, 0.5);
    let mut n = 0.;
    for q in cells {
        let (dx, dy) = (q[0] - p[0], q[1] - p[1]);
        let d2 = dx * dx + dy * dy;
        if d2 >= none {
            continue;
        }
        if d2 <= full {
            n += 1.;
            continue;
        }
        n += ((r - dx.hypot(dy)) / c + 0.5).clamp(0., 1.);
    }
    #[cfg(test)]
    assert_eq!((n * c * c).to_bits(), area_plain(cells, p, r, c).to_bits());
    n * c * c
}

/// `x + 1.` taken `n` times, for `x >= 0`: the same value, to the bit.
///
/// Between two powers of two (the upper one included) the numbers `x + j`
/// for whole `j` are all representable when `x` is (they are multiples of
/// `x`'s last place, which is at most 1 below 2^53), so those additions are
/// exact and any number of them is one exact addition. Only an addition
/// that passes a power of two rounds, and it is made on its own here as it
/// would be in turn.
fn add_ones(mut x: f64, mut n: usize) -> f64 {
    while n > 0 {
        if !(1. ..9e15).contains(&x) {
            x += 1.;
            n -= 1;
            continue;
        }
        // The next power of two above `x`, and the whole steps up to it.
        let top = f64::from_bits(((x.to_bits() >> 52) + 1) << 52);
        let room = (top - x).floor();
        if room >= 1. {
            let steps = if room < n as f64 { room as usize } else { n };
            x += steps as f64;
            n -= steps;
        } else {
            x += 1.;
            n -= 1;
        }
    }
    x
}

/// One entry of the cells a step's load sums read (`step_cells`).
#[derive(Clone, Copy, Debug)]
enum LoadCell {
    /// So many cells in a row that lie inside the cutter by half a cell or
    /// more wherever the step goes: each counts 1.
    Full(usize),
    /// A cell centre whose count depends on where the step goes.
    At(Point),
}

/// The material cells gathered around `p` (`Layer::gather`) as the load
/// sums of a step of length `ds` from `p` read them: in the same order,
/// every run of cells that are half a cell or more inside the cutter at
/// any point `ds` from `p` as one count, and the cells that are a cell and
/// a half or more outside it at every such point left out.
///
/// The steering tries some 26 directions for each step and sums over these
/// cells for each. A cell within r - c/2 - ds of `p` is within r - c/2 of
/// every centre tried (the triangle inequality, with `DISTANCE_SLACK` to
/// spare), so it counts exactly 1 in the area and nothing in the ring each
/// time: its distance need not be taken 26 times. Likewise a cell beyond
/// r + 3c/2 + ds counts nothing. The sums are built in the same order with
/// the same terms (`add_ones` for a run of ones), so they are the same to
/// the bit (#192).
fn step_cells(cells: &[Point], p: Point, ds: f64, r: f64, c: f64, out: &mut Vec<LoadCell>) {
    out.clear();
    let (inner, outer) = (r - 0.5 * c - ds, r + 1.5 * c + ds);
    let full = if inner >= c {
        inner * inner * (1. - DISTANCE_SLACK)
    } else {
        -1.
    };
    let none = outer * outer * (1. + DISTANCE_SLACK);
    for q in cells {
        let (dx, dy) = (q[0] - p[0], q[1] - p[1]);
        let d2 = dx * dx + dy * dy;
        if d2 >= none {
            continue;
        }
        if d2 <= full {
            if let Some(LoadCell::Full(n)) = out.last_mut() {
                *n += 1;
            } else {
                out.push(LoadCell::Full(1));
            }
        } else {
            out.push(LoadCell::At(*q));
        }
    }
}

/// `area` over a step's cells (`step_cells`).
fn area_of(cells: &[LoadCell], p: Point, r: f64, c: f64) -> f64 {
    let (full, none) = smoothing_band(r, c, 0.5);
    let mut n = 0.;
    for cell in cells {
        let q = match *cell {
            LoadCell::Full(k) => {
                n = add_ones(n, k);
                continue;
            }
            LoadCell::At(q) => q,
        };
        let (dx, dy) = (q[0] - p[0], q[1] - p[1]);
        let d2 = dx * dx + dy * dy;
        if d2 >= none {
            continue;
        }
        if d2 <= full {
            n += 1.;
            continue;
        }
        n += ((r - dx.hypot(dy)) / c + 0.5).clamp(0., 1.);
    }
    n * c * c
}

/// `step_load` over a step's cells (`step_cells`).
fn step_load_of(cells: &[LoadCell], p: Point, u: Point, r: f64, c: f64) -> (f64, f64) {
    let (full, none) = smoothing_band(r, c, 1.5);
    let (mut inside, mut ring) = (0., 0.);
    for cell in cells {
        let q = match *cell {
            LoadCell::Full(k) => {
                inside = add_ones(inside, k);
                continue;
            }
            LoadCell::At(q) => q,
        };
        let v = [q[0] - p[0], q[1] - p[1]];
        let d2 = v[0] * v[0] + v[1] * v[1];
        if d2 >= none {
            continue;
        }
        if d2 <= full {
            inside += 1.;
            continue;
        }
        let d = v[0].hypot(v[1]);
        let within = ((r - d) / c + 0.5).clamp(0., 1.);
        inside += within;
        if v[0] * u[0] + v[1] * u[1] >= 0. {
            ring += ((r + c - d) / c + 0.5).clamp(0., 1.) - within;
        }
    }
    (inside * c * c, ring * c / (r + c / 2.))
}

/// Smoothed count of the cells inside the cutter disk at `p` that were not
/// inside it at `prev` (farther than `r` from there): what a link step from
/// `prev` to `p` newly meets. Roots only where the answer depends on them,
/// as in `area` (`fresh_cells_plain` is the formula).
fn fresh_cells(cells: &[Point], prev: Point, p: Point, r: f64, c: f64) -> f64 {
    let (full, none) = smoothing_band(r, c, 0.5);
    let (sure, rim) = Grid::rim_bounds(r);
    let mut new = 0.;
    for q in cells {
        let (dx, dy) = (q[0] - p[0], q[1] - p[1]);
        let d2 = dx * dx + dy * dy;
        if d2 >= none || Grid::within(q[0] - prev[0], q[1] - prev[1], r, sure, rim) {
            continue;
        }
        new += if d2 <= full {
            1.
        } else {
            ((r - dx.hypot(dy)) / c + 0.5).clamp(0., 1.)
        };
    }
    #[cfg(test)]
    assert_eq!(
        f64::to_bits(new),
        fresh_cells_plain(cells, prev, p, r, c).to_bits()
    );
    new
}

/// `fresh_cells` by its formula, every cell measured.
#[cfg(test)]
fn fresh_cells_plain(cells: &[Point], prev: Point, p: Point, r: f64, c: f64) -> f64 {
    let mut new = 0.;
    for q in cells {
        if (q[0] - prev[0]).hypot(q[1] - prev[1]) > r {
            new += ((r - (q[0] - p[0]).hypot(q[1] - p[1])) / c + 0.5).clamp(0., 1.);
        }
    }
    new
}

/// Squared distances from a disk's centre at which a cell centre is surely
/// half a cell or more inside its radius `r` (so `((r - d) / c + 0.5)`
/// clamps to 1), and surely `outside` cells or more beyond it (so the
/// smoothed count out to there clamps to 0). `DISTANCE_SLACK` keeps both
/// clear of rounding; the cells between are measured.
fn smoothing_band(r: f64, c: f64, outside: f64) -> (f64, f64) {
    let (inner, outer) = (r - 0.5 * c, r + outside * c);
    // (Only for a disk of some cells: the slack is relative to its radius.)
    let full = if inner >= c {
        inner * inner * (1. - DISTANCE_SLACK)
    } else {
        -1.
    };
    (full, outer * outer * (1. + DISTANCE_SLACK))
}

/// `area` by its formula, every cell measured.
#[cfg(test)]
fn area_plain(cells: &[Point], p: Point, r: f64, c: f64) -> f64 {
    let mut n = 0.;
    for q in cells {
        let d = (q[0] - p[0]).hypot(q[1] - p[1]);
        n += ((r - d) / c + 0.5).clamp(0., 1.);
    }
    n * c * c
}

/// One sweep over the cells for a step to `p` heading along `u`: the
/// smoothed material area (mm2) inside the cutter disk, as `area`, and the
/// estimated contact angle (rad): the material in the ring of one cell just
/// outside the cutter's leading half, as arc length over its radius. (Behind
/// the widest point the ring holds the wall the pass leaves beside its own
/// trail, which the cutter no longer touches.) The exact bound is proven
/// afterwards (`EngagementFrontPlan::prove`).
///
/// As in `area`, only the cells whose count depends on their distance are
/// measured: those from half a cell inside the cutter to a cell and a half
/// outside it. Nearer cells add 1 to the area and nothing to the ring;
/// farther cells add nothing to either (`step_load_plain` is the formula).
#[cfg(test)]
fn step_load(cells: &[Point], p: Point, u: Point, r: f64, c: f64) -> (f64, f64) {
    let (full, none) = smoothing_band(r, c, 1.5);
    let (mut inside, mut ring) = (0., 0.);
    for q in cells {
        let v = [q[0] - p[0], q[1] - p[1]];
        let d2 = v[0] * v[0] + v[1] * v[1];
        if d2 >= none {
            continue;
        }
        if d2 <= full {
            inside += 1.;
            continue;
        }
        let d = v[0].hypot(v[1]);
        let within = ((r - d) / c + 0.5).clamp(0., 1.);
        inside += within;
        if v[0] * u[0] + v[1] * u[1] >= 0. {
            ring += ((r + c - d) / c + 0.5).clamp(0., 1.) - within;
        }
    }
    let load = (inside * c * c, ring * c / (r + c / 2.));
    #[cfg(test)]
    {
        let plain = step_load_plain(cells, p, u, r, c);
        assert_eq!(
            (load.0.to_bits(), load.1.to_bits()),
            (plain.0.to_bits(), plain.1.to_bits())
        );
    }
    load
}

/// `step_load` by its formula, every cell measured.
#[cfg(test)]
fn step_load_plain(cells: &[Point], p: Point, u: Point, r: f64, c: f64) -> (f64, f64) {
    let (mut inside, mut ring) = (0., 0.);
    for q in cells {
        let v = [q[0] - p[0], q[1] - p[1]];
        let d = v[0].hypot(v[1]);
        let within = ((r - d) / c + 0.5).clamp(0., 1.);
        inside += within;
        if v[0] * u[0] + v[1] * u[1] >= 0. {
            ring += ((r + c - d) / c + 0.5).clamp(0., 1.) - within;
        }
    }
    (inside * c * c, ring * c / (r + c / 2.))
}

/// Exact squared Euclidean distance transform (Felzenszwalb & Huttenlocher,
/// "Distance Transforms of Sampled Functions", 2012): distance in mm from each
/// cell centre to the nearest `true` cell centre (1e9 when none).
fn edt(g: &Grid, mask: &[bool]) -> Vec<f64> {
    const INF: f64 = 1e20;
    fn pass(f: &[f64], out: &mut [f64], v: &mut [usize], z: &mut [f64]) {
        let n = f.len();
        let mut k = 0usize;
        v[0] = 0;
        z[0] = f64::NEG_INFINITY;
        z[1] = f64::INFINITY;
        for q in 1..n {
            loop {
                let p = v[k];
                let s = ((f[q] + (q * q) as f64) - (f[p] + (p * p) as f64))
                    / (2. * q as f64 - 2. * p as f64);
                if s <= z[k] {
                    if k == 0 {
                        v[0] = q;
                        z[0] = f64::NEG_INFINITY;
                        z[1] = f64::INFINITY;
                        break;
                    }
                    k -= 1;
                } else {
                    k += 1;
                    v[k] = q;
                    z[k] = s;
                    z[k + 1] = f64::INFINITY;
                    break;
                }
            }
        }
        k = 0;
        for (q, o) in out.iter_mut().enumerate() {
            while z[k + 1] < q as f64 {
                k += 1;
            }
            let d = q as f64 - v[k] as f64;
            *o = d * d + f[v[k]];
        }
    }
    let (nx, ny) = (g.nx, g.ny);
    let mut f: Vec<f64> = mask.iter().map(|&m| if m { 0. } else { INF }).collect();
    let n = nx.max(ny);
    let (mut v, mut zz) = (vec![0usize; n], vec![0f64; n + 1]);
    let (mut col, mut out) = (vec![0f64; ny], vec![0f64; ny]);
    for i in 0..nx {
        for j in 0..ny {
            col[j] = f[j * nx + i];
        }
        pass(&col, &mut out, &mut v, &mut zz);
        for j in 0..ny {
            f[j * nx + i] = out[j];
        }
    }
    let mut row = vec![0f64; nx];
    let mut out = vec![0f64; nx];
    for j in 0..ny {
        row.copy_from_slice(&f[j * nx..(j + 1) * nx]);
        pass(&row, &mut out, &mut v, &mut zz);
        f[j * nx..(j + 1) * nx].copy_from_slice(&out);
    }
    f.iter()
        .map(|&d| if d >= INF / 2. { 1e9 } else { d.sqrt() * g.c })
        .collect()
}

/// Flatten a region boundary to straight segments with sagitta <= `sag`.
fn flatten(b: &Boundary, sag: f64, out: &mut Vec<(Point, Point)>) {
    for s in Stroke::from_boundary(b) {
        let Some((c, sweep)) = s.arc else {
            out.push((s.a, s.b));
            continue;
        };
        let rho = (s.a[0] - c[0]).hypot(s.a[1] - c[1]);
        let step = 2. * (1. - sag / rho).clamp(-1., 1.).acos();
        let n = ((sweep.abs() / step.max(1e-6)).ceil() as usize).clamp(1, 1 << 16);
        let a0 = (s.a[1] - c[1]).atan2(s.a[0] - c[0]);
        let mut prev = s.a;
        for t in 1..=n {
            let p = if t == n {
                s.b
            } else {
                let a = a0 + sweep * t as f64 / n as f64;
                [c[0] + rho * a.cos(), c[1] + rho * a.sin()]
            };
            out.push((prev, p));
            prev = p;
        }
    }
}

/// Greedy line/arc fit of a polyline: each output stroke passes within `tol`
/// of every point it replaces.
fn fit_strokes(pts: &[Point], tol: f64, max_radius: f64) -> Vec<Stroke> {
    fn one(q: &[Point], tol: f64, max_radius: f64) -> Option<Option<(Point, f64)>> {
        let (a, b) = (q[0], *q.last().unwrap());
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let l = dx.hypot(dy);
        if l < 1e-9 {
            return None;
        }
        if q.iter().all(|p| {
            let along = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l;
            ((p[0] - a[0]) * dy - (p[1] - a[1]) * dx).abs() / l <= tol
                && along >= -tol
                && along <= l + tol
        }) {
            return Some(None);
        }
        if q.len() < 3 {
            return None;
        }
        let m = q[q.len() / 2];
        let d = 2. * (a[0] * (m[1] - b[1]) + m[0] * (b[1] - a[1]) + b[0] * (a[1] - m[1]));
        if d.abs() < 1e-12 {
            return None;
        }
        let (a2, m2, b2) = (
            a[0] * a[0] + a[1] * a[1],
            m[0] * m[0] + m[1] * m[1],
            b[0] * b[0] + b[1] * b[1],
        );
        let c = [
            (a2 * (m[1] - b[1]) + m2 * (b[1] - a[1]) + b2 * (a[1] - m[1])) / d,
            (a2 * (b[0] - m[0]) + m2 * (a[0] - b[0]) + b2 * (m[0] - a[0])) / d,
        ];
        let rho = (a[0] - c[0]).hypot(a[1] - c[1]);
        if rho > max_radius
            || q.iter()
                .any(|p| ((p[0] - c[0]).hypot(p[1] - c[1]) - rho).abs() > tol)
        {
            return None;
        }
        let mut sweep = 0.;
        for w in q.windows(2) {
            let mut da =
                (w[1][1] - c[1]).atan2(w[1][0] - c[0]) - (w[0][1] - c[1]).atan2(w[0][0] - c[0]);
            if da > PI {
                da -= TAU;
            }
            if da < -PI {
                da += TAU;
            }
            // A fitted arc must turn monotonically through its points.
            if sweep != 0. && da != 0. && da.signum() != f64::signum(sweep) {
                return None;
            }
            sweep += da;
        }
        (sweep.abs() <= 0.9 * PI && sweep != 0.).then_some(Some((c, sweep)))
    }
    let mut out = vec![];
    let mut i = 0;
    while i + 1 < pts.len() {
        let mut best = (i + 1, None);
        let mut j = i + 1;
        while j < pts.len() {
            match one(&pts[i..=j], tol, max_radius) {
                Some(f) => {
                    best = (j, f);
                    j += 1;
                }
                None if j == i + 1 => {
                    // Coincident points: skip.
                    best = (j, None);
                    break;
                }
                None => break,
            }
        }
        let (j, arc) = best;
        if pts[i] != pts[j] {
            out.push(Stroke {
                a: pts[i],
                b: pts[j],
                arc,
            });
        }
        i = j;
    }
    out
}

/// `fit_strokes`, with every fitted arc that `fits` rejects replaced by the
/// polyline of the points it stood for. The fit only checks the points: an
/// arc through points far apart on a tight turn bows out between them and can
/// come closer to the region than the points do, while the points' own lines
/// stay as close as the points. Returns the strokes and how many arcs were
/// replaced; the caller still certifies every stroke.
fn fit_certifiable(
    pts: &[Point],
    tol: f64,
    max_radius: f64,
    fits: impl Fn(Stroke) -> bool,
) -> (Vec<Stroke>, usize) {
    let mut out = vec![];
    let mut refitted = 0;
    let mut at = 0;
    for s in fit_strokes(pts, tol, max_radius) {
        // Strokes run between consecutive input points, in order.
        let from = at + pts[at..].iter().position(|&p| p == s.a).unwrap_or(0);
        at = from + pts[from..].iter().position(|&p| p == s.b).unwrap_or(0);
        if s.arc.is_some() && !fits(s) {
            refitted += 1;
            out.extend(
                pts[from..=at]
                    .windows(2)
                    .filter(|w| w[0] != w[1])
                    .map(|w| Stroke::line(w[0], w[1])),
            );
        } else {
            out.push(s);
        }
    }
    (out, refitted)
}
/// Whether the committed sweeps of `scene` prove that a cutter of radius `r`
/// moved along `s` at `z` meets no material thicker than the entry skin
/// (`crate::entry::skin`): the whole tube of the stroke, of the radius less
/// the skin, lies in cleared columns (`EngagementScene::clear_line`, a
/// proof of clearance; the contact-angle bound of `check_leading` is an
/// upper bound that stays well above zero in cleared space beside earlier
/// sweeps, so it cannot say a stroke is in air).
///
/// An arc is covered by chords: its tube lies in the chords' tubes of the
/// radius plus the arc's sagitta, and the chords are short enough that the
/// sagitta is at most half the skin. A stroke the proof's work limit does
/// not settle is not proven clear.
fn wall_piece_is_clear(
    scene: &EngagementScene,
    s: Stroke,
    z: f64,
    r: f64,
    tolerance: f64,
    cancel: &impl CancelSignal,
) -> Result<bool, String> {
    let skin = crate::entry::skin(tolerance);
    let (chords, sagitta) = match s.arc {
        None => (1, 0.),
        Some((centre, sweep)) => {
            let radius = (s.a[0] - centre[0]).hypot(s.a[1] - centre[1]);
            let most = 2. * (1. - (skin / 2. / radius).min(1.)).acos();
            let chords = ((sweep.abs() / most).ceil() as usize).max(1);
            (
                chords,
                radius * (1. - (sweep.abs() / chords as f64 / 2.).cos()),
            )
        }
    };
    for i in 0..chords {
        if cancel.is_cancelled() {
            return Err("engagement_front planning cancelled".into());
        }
        let chord = stroke_part(s, i as f64 / chords as f64, (i + 1) as f64 / chords as f64);
        let mut work = WALL_PIECE_WORK;
        match scene.clear_line(
            chord.a,
            chord.b,
            r - skin + sagitta,
            z,
            tolerance,
            &mut work,
            cancel,
        ) {
            Ok(true) => (),
            Ok(false) => return Ok(false),
            Err(_) if !cancel.is_cancelled() => return Ok(false),
            Err(e) => return Err(e),
        }
    }
    Ok(true)
}
/// Work one chord of a wall piece may spend proving itself clear.
const WALL_PIECE_WORK: u64 = 200_000;
/// The length of a stroke.
fn stroke_length(s: Stroke) -> f64 {
    match s.arc {
        Some((centre, sweep)) => (s.a[0] - centre[0]).hypot(s.a[1] - centre[1]) * sweep.abs(),
        None => (s.b[0] - s.a[0]).hypot(s.b[1] - s.a[1]),
    }
}
/// The part of a stroke from the fraction `f0` of its length to `f1`. The
/// stroke's own ends are kept exactly, and a point between is the same
/// whichever part it ends, so consecutive parts connect.
fn stroke_part(s: Stroke, f0: f64, f1: f64) -> Stroke {
    let at = |f: f64| -> Point {
        if f <= 0. {
            return s.a;
        }
        if f >= 1. {
            return s.b;
        }
        match s.arc {
            Some((centre, sweep)) => {
                let radius = (s.a[0] - centre[0]).hypot(s.a[1] - centre[1]);
                let angle = (s.a[1] - centre[1]).atan2(s.a[0] - centre[0]) + sweep * f;
                [
                    centre[0] + radius * angle.cos(),
                    centre[1] + radius * angle.sin(),
                ]
            }
            None => [
                s.a[0] + (s.b[0] - s.a[0]) * f,
                s.a[1] + (s.b[1] - s.a[1]) * f,
            ],
        }
    };
    Stroke {
        a: at(f0),
        b: at(f1),
        arc: s
            .arc
            .map(|(centre, sweep)| (centre, sweep * (f1.min(1.) - f0.max(0.)))),
    }
}

/// Line/arc fit tolerance as a fraction of the steering grid cell.
const FIT_FRACTION: f64 = 0.25;
/// Cells of width the steering gives away (see `steering_width`).
/// Cells of width the steering gives away (see `steering_width`): half a
/// cell for material next to a wall that the grid does not count
/// (`Layer::reachable`), and the stroke fit tolerance.
const STEER_MARGIN_CELLS: f64 = 0.5 + FIT_FRACTION;
/// Steps of a pass followed before its strokes are proven: an unproven
/// stroke then costs at most this much planning again, not the whole pass.
const PROOF_CHUNK: usize = 384;
/// How much narrower a pass is steered after a stroke of it was not proven.
const RETRY_SCALE: f64 = 0.95;
struct PassMark {
    pending: usize,
    journal: usize,
    report: EngagementFrontReport,
}

type Pending = (Curve, MoveKind, Option<f64>);

/// Whether two curves are the same to the bit (so every computation on
/// them gives the same result).
fn same_curve(a: &Curve, b: &Curve) -> bool {
    let same = |p: &Vec3, q: &Vec3| (0..3).all(|k| p[k].to_bits() == q[k].to_bits());
    match (a, b) {
        (Curve::Line { start, end }, Curve::Line { start: s, end: e }) => {
            same(start, s) && same(end, e)
        }
        (
            Curve::Arc {
                start,
                end,
                center,
                plane,
                sweep_rad,
            },
            Curve::Arc {
                start: s,
                end: e,
                center: c,
                plane: p,
                sweep_rad: w,
            },
        )
        | (
            Curve::Helix {
                start,
                end,
                center,
                plane,
                sweep_rad,
            },
            Curve::Helix {
                start: s,
                end: e,
                center: c,
                plane: p,
                sweep_rad: w,
            },
        ) => {
            same(start, s)
                && same(end, e)
                && same(center, c)
                && plane == p
                && sweep_rad.to_bits() == w.to_bits()
        }
        _ => false,
    }
}

#[cfg(test)]
thread_local! {
    /// How often this thread's planning took each shortcut that a test
    /// build compares with its definition: a recorded proof (0), a goal
    /// out of a link's reach (1), a link search settled in its corridor
    /// (2), the most open cell read from the bits (3). The whole-plan tests
    /// check that none of them goes untested.
    static SHORTCUTS: std::cell::Cell<[usize; 4]> = const { std::cell::Cell::new([0; 4]) };
}
#[cfg(test)]
fn shortcut_taken(which: usize) {
    SHORTCUTS.with(|taken| {
        let mut counts = taken.get();
        counts[which] += 1;
        taken.set(counts);
    });
}

/// The exact proof's status for a stroke and its bound in degrees.
type Proved = (EngagementStatus, f64);

/// What the exact proof found for the strokes queued from `from` on, in
/// order: each stroke as it was queued, and the proof's status and bound
/// for it once it has been proven (`None`: only ever a premise).
///
/// A stroke's proof depends on the committed sweeps and on the strokes
/// queued before it since `from`, which are its premises, and on nothing
/// else. A pass with an unproven stroke is planned again from that stroke,
/// and the strokes before it are queued again as they were: while the
/// committed sweeps are the same and the queue from `from` repeats what is
/// recorded here stroke for stroke, to the bit, a stroke's proof is the
/// recorded one and is not computed again (#192). The record ends at the
/// first stroke that differs.
#[derive(Default)]
struct ProofMemo {
    from: usize,
    strokes: Vec<(Curve, MoveKind, Option<Proved>)>,
}

pub(crate) struct EngagementFrontPlan {
    guides: PocketPlan,
    op: PocketOperation,
    entry: PocketEntry,
    heights: Heights,
    /// See `RoughingOperation::rapid_descent_clearance_mm`.
    rapid_descent: Option<f64>,
    /// See `RoughingOperation::travel_clearance_mm`.
    travel_clearance: Option<f64>,
    /// The bounding box and top of each avoid area of the operation's
    /// limits (`None` when one cannot be measured), for `travel_height`.
    #[allow(clippy::type_complexity)]
    avoid_boxes: std::sync::OnceLock<Option<Vec<([f64; 4], f64)>>>,
    /// See `RoughingOperation::ramp_clearance_mm`.
    ramp_clearance: Option<f64>,
    /// See `RoughingOperation::link_feed_mm_min`.
    link_feed: Option<f64>,
    /// See `RoughingOperation::link_chip_thinning`.
    chip_thinning: bool,
    r: f64,
    width: f64,
    climb: bool,
    z: f64,
    spec: StockSpec,
    layer: Option<Layer>,
    memory: Option<FrontMemory>,
    /// The stock generation of the memory this layer was built from, when
    /// it carried one: a layer that commits no motion hands that memory on
    /// unchanged (see `take_memory`).
    carried_generation: Option<u64>,
    pending: VecDeque<Pending>,
    position: Vec3,
    next_id: u64,
    generation: Option<u64>,
    finished: bool,
    report: EngagementFrontReport,
    scratch: Vec<Point>,
    scratch_load: Vec<LoadCell>,
    /// Committed cleared sweeps, for the exact engagement proof of each pass.
    proof: Option<(EngagementScene, ClearanceCursor)>,
    /// Passes may start on a remnant too thin for a full step (see
    /// `follow`): set only to clear material a helix would otherwise bore
    /// onto, until the next pass is planned.
    remnant: bool,
    /// Finish each wall with one pass along it (`with_wall_pass`), and
    /// whether this level's have been planned.
    wall_pass: bool,
    /// Cut along the walls only where material stands beside them
    /// (`with_wall_runs`).
    wall_runs: bool,
    walls_done: bool,
    /// Reachable material area of this level (left, at its start) as of the
    /// last candidate refresh: planning progress (#198), read by nothing
    /// that plans.
    area_mm2: Option<(f64, f64)>,
    /// The paths of this level's ramp entries: slots cut to depth.
    ramp_slots: Vec<(Point, Point)>,
    /// Proofs of the strokes queued for the pass being planned.
    proof_memo: ProofMemo,
    /// Where links from the current position can reach (`routed_link`),
    /// and the start and material state of the last link search that did
    /// not reach its goal.
    link_reach: std::cell::RefCell<Option<LinkReach>>,
    link_failed: std::cell::Cell<Option<(u64, usize, u64)>>,
    /// Refuse the level when it starts with an area no entry reaches
    /// (`with_unentered_refused`), instead of listing it in the report.
    refuse_unentered: bool,
}

/// The entry and link options of an engagement_front clearing, as `Roughing`
/// and `AdaptivePocket` both declare them (see the fields of
/// `RoughingOperation`). None of them changes a cutting pass: they set
/// where a descent may be rapid (and so where it comes down), where a helix
/// starts, and the feed of stay-down links.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct FrontMotionOptions {
    pub rapid_descent_clearance_mm: Option<f64>,
    pub travel_clearance_mm: Option<f64>,
    pub ramp_clearance_mm: Option<f64>,
    pub link_feed_mm_min: Option<f64>,
    pub link_chip_thinning: bool,
}
impl FrontMotionOptions {
    /// `what` names the operation in messages ("roughing"). `other` is set
    /// when the operation's strategy is not engagement_front: every option
    /// is then refused by name, and `other` says what to do about it.
    pub(crate) fn validate(&self, what: &str, other: Option<&str>) -> Result<(), String> {
        for (name, value) in [
            (
                "rapid_descent_clearance_mm",
                self.rapid_descent_clearance_mm,
            ),
            ("travel_clearance_mm", self.travel_clearance_mm),
            ("ramp_clearance_mm", self.ramp_clearance_mm),
            ("link_feed_mm_min", self.link_feed_mm_min),
        ] {
            let Some(d) = value else { continue };
            if let Some(other) = other {
                return Err(format!(
                    "{name} applies to engagement_front clearing only{other}"
                ));
            }
            let max = if name == "link_feed_mm_min" { 1e6 } else { 1e3 };
            if !d.is_finite() || d <= 0. || d > max {
                return Err(format!("{what} {name} {d} must be positive and finite"));
            }
        }
        if let (true, Some(other)) = (self.link_chip_thinning, other) {
            return Err(format!(
                "link_chip_thinning applies to engagement_front clearing only{other}"
            ));
        }
        if self.link_chip_thinning && self.link_feed_mm_min.is_none() {
            return Err("link_chip_thinning needs link_feed_mm_min: the thinned link feed is capped at the declared link feed".into());
        }
        Ok(())
    }
}

impl EngagementFrontPlan {
    /// Validated options (`FrontMotionOptions::validate`).
    pub(crate) fn with_motion_options(self, options: FrontMotionOptions) -> Self {
        self.with_rapid_descent(options.rapid_descent_clearance_mm)
            .with_travel_clearance(options.travel_clearance_mm)
            .with_ramp_clearance(options.ramp_clearance_mm)
            .with_link_feed(options.link_feed_mm_min, options.link_chip_thinning)
    }
    pub(crate) fn prepare(
        op: PocketOperation,
        tool: &ToolAssembly,
        spec: &StockSpec,
        initial: Vec3,
        width: f64,
        climb: bool,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        let entry = op
            .entry
            .clone()
            .ok_or("engagement_front needs an entry policy")?;
        if entry.policy() == EntryPolicy::Helix {
            return Err("engagement_front enters from air or cleared stock and helixes only into enclosed material; use StockEdgeEntry (stock_edge_or_helix or stock_edge), not HelicalEntry".into());
        }
        let r = tool.prepare()?.cutter().definition().diameter_mm / 2.;
        if !width.is_finite() || width <= 0. || width > r {
            return Err(
                "engagement_front radial engagement must be positive and at most the cutter radius"
                    .into(),
            );
        }
        let guard = 16. * op.tolerance_mm;
        let heights = Heights::resolve(
            op.clearance_z_mm,
            op.retract_z_mm,
            op.feed_z_mm,
            spec.max_mm[2].max(op.top_z_mm),
            guard,
        )?;
        let guides = if entry.ramp().is_some() {
            // No pass of the guide plan is cut, so none needs a ramp.
            PocketPlan::prepare_region_guides(op.clone(), tool, spec, initial, 1, cancel)?
        } else {
            PocketPlan::prepare(op.clone(), tool, spec, initial, 1, cancel)?
        };
        let z = guides.report().target_z_mm;
        let ramped = entry.ramp().is_some();
        let mut report = EngagementFrontReport {
            strategy: "engagement_front",
            target_z_mm: z,
            target_radial_engagement_mm: width,
            engagement_limit_deg: (1. - width / r).acos().to_degrees(),
            geometric_residual_status: ResidualStatus::Pending,
            diagnostics: if ramped {
                vec![
                    "Engagement is steered on a fine grid seeded from committed whole-cell stock bounds, then every cutting and stay-down link stroke is proven exactly against the committed cleared sweeps: its contact angle, less the cutter's own trail, is at most engagement_limit_deg = acos(1 - w/r). A pass with an unproven stroke is planned again narrower; ramp entries cut a full slot at the entry feed and are not bounded by w".into(),
                    "Every cutting, link and ramp stroke is certified exactly against the layer region; air entries are proven by committed sweeps; enclosed material is entered by a ramp along a straight path, from the material top the committed stock proves".into(),
                ]
            } else {
                vec![
                    "Engagement is steered on a fine grid seeded from committed whole-cell stock bounds, then every cutting and stay-down link stroke is proven exactly against the committed cleared sweeps: its contact angle, less the cutter's own trail, is at most engagement_limit_deg = acos(1 - w/r). A pass with an unproven stroke is planned again narrower; helical entries cut a full slot and are not bounded by w".into(),
                    "Every cutting and link stroke is certified exactly against the layer region; air entries are proven by committed sweeps; enclosed material is entered by helix".into(),
                ]
            },
            ..Default::default()
        };
        if guides.guide_strokes().is_empty() {
            // As `PocketPlan` says it: the cutter fits nowhere in this
            // level's region, and all its stock is the level's residual.
            report
                .diagnostics
                .push("no cutter-center region remains: this level emits no cutting motion".into());
        }
        Ok(Self {
            guides,
            op,
            entry,
            heights,
            rapid_descent: None,
            travel_clearance: None,
            avoid_boxes: std::sync::OnceLock::new(),
            ramp_clearance: None,
            link_feed: None,
            chip_thinning: false,
            r,
            width,
            climb,
            z,
            spec: spec.clone(),
            layer: None,
            memory: None,
            carried_generation: None,
            pending: VecDeque::new(),
            position: initial,
            next_id: 1,
            generation: None,
            finished: false,
            report,
            scratch: vec![],
            scratch_load: vec![],
            proof: None,
            remnant: false,
            wall_pass: false,
            wall_runs: false,
            walls_done: false,
            area_mm2: None,
            ramp_slots: vec![],
            proof_memo: ProofMemo::default(),
            link_reach: std::cell::RefCell::new(None),
            link_failed: std::cell::Cell::new(None),
            refuse_unentered: false,
        })
    }
    /// See `RoughingOperation::ramp_clearance_mm` (validated there).
    pub(crate) fn with_ramp_clearance(mut self, clearance: Option<f64>) -> Self {
        self.ramp_clearance = clearance;
        self
    }
    /// See `RoughingOperation::link_feed_mm_min` (validated there).
    pub(crate) fn with_link_feed(mut self, feed: Option<f64>, chip_thinning: bool) -> Self {
        self.link_feed = feed;
        self.chip_thinning = chip_thinning;
        self
    }
    /// After the level is cleared, cut once along every wall of the region:
    /// the cutter centre on the boundary's offset by the cutter radius and
    /// the radial allowance. The front leaves a thin, uneven skin on the
    /// walls (it stops where the material ahead is thin, and its steering
    /// grid keeps centres about two cells off the exact offset), and a
    /// pocket's walls are finished surfaces. Every boundary of a pocket is
    /// a wall; a roughing layer has `with_wall_runs` instead.
    pub(crate) fn with_wall_pass(mut self, on: bool) -> Self {
        self.wall_pass = on;
        self
    }
    /// After the level is cleared, cut along its walls wherever material
    /// still stands beside them, and nowhere else (`plan_wall_runs`).
    ///
    /// A roughing layer's region is the stock's surroundings less the
    /// part's shadow: most of its boundary is air, and a wall is often an
    /// open run that ends where the stock does. The front ends its passes
    /// at the wall a stepover apart, so between two pass ends a cusp
    /// stands on the wall, well beyond the radial allowance asked for
    /// (0.16 to 0.30 mm along a straight wall where 0.1 mm was asked, with
    /// a 6 mm cutter at 0.6 mm engagement; a step-up found one and nibbled
    /// its top). One pass along the wall leaves the allowance and no more,
    /// which is what the operation after roughing is told to expect.
    pub(crate) fn with_wall_runs(mut self, on: bool) -> Self {
        self.wall_runs = on;
        self
    }
    /// See `RoughingOperation::rapid_descent_clearance_mm` (validated there).
    pub(crate) fn with_rapid_descent(mut self, clearance: Option<f64>) -> Self {
        self.rapid_descent = clearance;
        self
    }
    /// See `RoughingOperation::travel_clearance_mm` (validated there).
    pub(crate) fn with_travel_clearance(mut self, clearance: Option<f64>) -> Self {
        self.travel_clearance = clearance;
        self
    }
    pub(crate) fn construction_error_mm(&self) -> f64 {
        self.guides.construction_error_mm()
    }
    /// This level's reachable material area (left, at its start) as of the
    /// last candidate refresh; `None` before the first.
    pub(crate) fn area_mm2(&self) -> Option<(f64, f64)> {
        self.area_mm2
    }
    pub(crate) fn activate_layer(
        &mut self,
        initial: Vec3,
        first_id: u64,
        stock: &StockSpec,
    ) -> Result<(), String> {
        if self.report.motions != 0 || self.generation.is_some() {
            return Err("cannot activate a used engagement_front layer".into());
        }
        self.guides.activate_layer(initial, first_id, stock)?;
        self.position = initial;
        self.next_id = first_id;
        self.spec = stock.clone();
        Ok(())
    }
    pub(crate) fn give_memory(&mut self, memory: FrontMemory) {
        self.memory = Some(memory);
    }
    /// The cleared-height memory to hand to the next layer. A layer that
    /// committed motions hands its own, valid for the stock generation after
    /// its last motion. A layer that committed none changed no stock, so the
    /// memory it was given (used or not) is handed on with its original
    /// generation; the next layer still accepts it only if the stock is at
    /// that generation. Dropping it here made the layer after every empty
    /// step-up steer from whole-cell stock bounds, which see material along
    /// walls that earlier layers had cut, and run passes in air.
    pub(crate) fn take_memory(&mut self) -> Option<FrontMemory> {
        if self.generation.is_none() {
            if let Some(memory) = self.memory.take() {
                return Some(memory);
            }
            let layer = self.layer.take()?;
            return Some(FrontMemory {
                grid: layer.g,
                cleared: layer.cleared,
                generation: self.carried_generation?,
            });
        }
        let layer = self.layer.take()?;
        Some(FrontMemory {
            grid: layer.g,
            cleared: layer.cleared,
            generation: self.generation?,
        })
    }
    pub(crate) fn report(&self) -> &EngagementFrontReport {
        &self.report
    }
    pub(crate) fn prefix_events(&self) -> Vec<Event> {
        self.guides.prefix_events()
    }
    pub(crate) fn suffix_events(&self) -> Vec<Event> {
        self.guides.suffix_events()
    }
    pub(crate) fn finish_report(&mut self, cancel: &impl CancelSignal) -> Result<(), String> {
        if cancel.is_cancelled() {
            return Err("engagement_front report cancelled".into());
        }
        if !self.finished || !self.pending.is_empty() {
            return Err("drain engagement_front motions before report".into());
        }
        if self.report.geometric_residual_status == ResidualStatus::Pending {
            match &self.layer {
                Some(layer) => {
                    // Every cell of the region that still holds stock above
                    // the level, from the region and the stock themselves,
                    // not from where the planner could stand (#250); split
                    // by why it is left. A tool centre fits where the room
                    // to the walls is the cutter radius and the allowance;
                    // the cutter reaches its radius from there.
                    let g = layer.g;
                    let fits: Vec<bool> = (0..layer.top.len())
                        .map(|k| {
                            layer.inside[k]
                                && layer.room()[k] as f64 >= self.r + self.op.radial_allowance_mm
                        })
                        .collect();
                    let to_fit = edt(&g, &fits);
                    let mut left = [0usize; 3];
                    for k in 0..layer.top.len() {
                        if !layer.material(k) {
                            continue;
                        }
                        if layer.reachable[k] {
                            left[0] += 1;
                        } else if layer.inside[k] {
                            left[if to_fit[k] <= self.r { 1 } else { 2 }] += 1;
                        }
                    }
                    let area = |cells: usize| cells as f64 * g.c * g.c;
                    self.report.residual_reachable_area_mm2 = Some(area(left[0]));
                    self.report.residual_unresolved_area_mm2 = Some(area(left[1]));
                    self.report.residual_unreachable_area_mm2 = Some(area(left[2]));
                    self.report.geometric_residual_area_mm2 =
                        Some(area(left[0] + left[1] + left[2]));
                    self.report.geometric_residual_status = ResidualStatus::Estimated;
                }
                None => self.report.geometric_residual_status = ResidualStatus::Unknown,
            }
        }
        Ok(())
    }

    fn grid_for(&self, stock: &Stock) -> Result<Grid, String> {
        let step = stock.cell_widths_mm();
        // Steering resolution: fine enough to resolve the target width.
        let want = (self.r / 20.).min(self.width / 5.);
        let c = step[0] / (step[0] / want).ceil();
        let margin = 3. * self.r + self.op.radial_allowance_mm + 24. * c;
        let x0 = self.spec.min_mm[0] - (margin / c).ceil() * c;
        let y0 = self.spec.min_mm[1] - (margin / c).ceil() * c;
        let nx = ((self.spec.max_mm[0] + margin - x0) / c).ceil() as usize;
        let ny = ((self.spec.max_mm[1] + margin - y0) / c).ceil() as usize;
        crate::resources::check_memory((nx * ny) as u128 * 32, "engagement_front steering grid")?;
        if nx * ny > 64_000_000 {
            return Err(format!(
                "engagement_front steering grid of {nx} x {ny} cells exceeds 64M; use a larger tool or split the stock"
            ));
        }
        Ok(Grid { x0, y0, c, nx, ny })
    }

    /// Clearance a cutter centre needs on the grid: the exact certification
    /// requirement plus grid, chord and fit error.
    fn grid_clearance(&self, c: f64) -> f64 {
        let region = self.guides.source_region();
        let uncertainty = region.definition().boundary_error_mm + self.op.tolerance_mm;
        self.r
            + self.op.radial_allowance_mm
            + uncertainty
            + 4. * self.op.tolerance_mm
            + 1.5 * c
            + c / 8.
            + FIT_FRACTION * c
    }

    fn build_layer(&mut self, stock: &Stock, cancel: &impl CancelSignal) -> Result<(), String> {
        let g = self.grid_for(stock)?;
        let n = g.nx * g.ny;
        // Region interior at cell centres (even-odd over flattened edges).
        let mut edges = vec![];
        for comp in &self.guides.source_region().definition().components {
            flatten(&comp.outer, g.c / 8., &mut edges);
            for h in &comp.holes {
                flatten(h, g.c / 8., &mut edges);
            }
        }
        let mut inside = vec![false; n];
        let mut xs = vec![];
        for j in 0..g.ny {
            if j % 256 == 0 && cancel.is_cancelled() {
                return Err("engagement_front planning cancelled".into());
            }
            let y = g.y0 + (j as f64 + 0.5) * g.c;
            xs.clear();
            for &(a, b) in &edges {
                if (a[1] > y) != (b[1] > y) {
                    xs.push(a[0] + (y - a[1]) * (b[0] - a[0]) / (b[1] - a[1]));
                }
            }
            xs.sort_by(f64::total_cmp);
            for pair in xs.as_chunks::<2>().0 {
                let i0 = ((pair[0] - g.x0) / g.c - 0.5).ceil().max(0.) as usize;
                let i1 = ((pair[1] - g.x0) / g.c - 0.5).floor().min(g.nx as f64 - 1.);
                if i1 < 0. {
                    continue;
                }
                for i in i0..=i1 as usize {
                    inside[g.idx(i, j)] = true;
                }
            }
        }
        // A boundary feature narrower than a cell (a hairline island, a
        // needle of the target's shadow) can fall between cell centres and
        // be missed by the centre test above, letting the grid allow centres
        // the exact certification then rejects. An edge-touched cell whose
        // centre is inside but which has no outside-centre neighbour holds
        // such an unseen feature: it counts as outside, so every boundary
        // point lies in an outside cell or next to an outside centre (as
        // along an ordinary boundary, which the clearance margin covers).
        let centre_inside = inside.clone();
        let seen = |i: usize, j: usize| {
            (i.saturating_sub(1)..=(i + 1).min(g.nx - 1)).any(|a| {
                (j.saturating_sub(1)..=(j + 1).min(g.ny - 1)).any(|b| !centre_inside[g.idx(a, b)])
            })
        };
        for &(a, b) in &edges {
            let len = (b[0] - a[0]).hypot(b[1] - a[1]);
            let pieces = ((2. * len / g.c).ceil() as usize).clamp(1, 1 << 24);
            for t in 0..pieces {
                let f0 = t as f64 / pieces as f64;
                let f1 = (t + 1) as f64 / pieces as f64;
                let p = [a[0] + (b[0] - a[0]) * f0, a[1] + (b[1] - a[1]) * f0];
                let q = [a[0] + (b[0] - a[0]) * f1, a[1] + (b[1] - a[1]) * f1];
                let cell = |v: f64, o: f64, n: usize| {
                    ((v - o) / g.c).floor().clamp(0., n as f64 - 1.) as usize
                };
                for j in cell(p[1].min(q[1]), g.y0, g.ny)..=cell(p[1].max(q[1]), g.y0, g.ny) {
                    for i in cell(p[0].min(q[0]), g.x0, g.nx)..=cell(p[0].max(q[0]), g.x0, g.nx) {
                        if centre_inside[g.idx(i, j)] && !seen(i, j) {
                            inside[g.idx(i, j)] = false;
                        }
                    }
                }
            }
        }
        // The grid border counts as outside.
        for i in 0..g.nx {
            inside[g.idx(i, 0)] = false;
            inside[g.idx(i, g.ny - 1)] = false;
        }
        for j in 0..g.ny {
            inside[g.idx(0, j)] = false;
            inside[g.idx(g.nx - 1, j)] = false;
        }
        let outside: Vec<bool> = inside.iter().map(|b| !b).collect();
        let need = self.grid_clearance(g.c);
        // Allowed centres: at least `need` from the outside, the distance
        // taken as the layer keeps it (`Layer::room`, single precision).
        // Reachable cells: within the cutter's reach of an allowed centre.
        // Both are asked of the cells grown by a disk (`Near`) instead of
        // two distance transforms of the grid; the distances themselves
        // are worked out only if an entry needs them.
        let stride = g.nx.div_ceil(64);
        let pack = |cells: &[bool]| {
            let mut bits = vec![0u64; stride * g.ny];
            for (k, _) in cells.iter().enumerate().filter(|(_, set)| **set) {
                bits[(k / g.nx) * stride + (k % g.nx) / 64] |= 1 << (k % g.nx % 64);
            }
            bits
        };
        let whole = ([0, g.nx - 1], [0, g.ny - 1]);
        let grown = |cells: &[bool], cells2: Option<usize>| -> Vec<bool> {
            match cells2 {
                Some(d2) => {
                    let near = Near::new(&pack(cells), stride, whole.0, whole.1, d2);
                    (0..n).map(|k| near.at(k % g.nx, k / g.nx)).collect()
                }
                None => vec![false; n],
            }
        };
        let too_near = cells2_where(g.c, need, |d| ((d as f32) as f64) < need);
        let allowed: Vec<bool> = grown(&outside, too_near).iter().map(|b| !b).collect();
        let reach_limit = self.r - 0.5 * g.c;
        let reachable = grown(&allowed, cells2_within(g.c, reach_limit, true));
        #[cfg(test)]
        {
            let room: Vec<f32> = edt(&g, &outside).into_iter().map(|d| d as f32).collect();
            let by_distance: Vec<bool> = room.iter().map(|&d| d as f64 >= need).collect();
            assert!(allowed == by_distance, "allowed centres");
            let by_distance: Vec<bool> = edt(&g, &allowed)
                .iter()
                .map(|&d| d <= reach_limit)
                .collect();
            assert!(reachable == by_distance, "reachable cells");
        }
        // Stock: whole-cell upper bounds, so no material is ever missed.
        let [snx, sny] = stock.dimensions();
        let step = stock.cell_widths_mm();
        let cells = stock.cells();
        let smin = stock.spec().min_mm;
        let top: Vec<f64> = (0..n)
            .map(|k| {
                let p = g.center(k);
                let i = ((p[0] - smin[0]) / step[0]).floor();
                let j = ((p[1] - smin[1]) / step[1]).floor();
                if i < 0. || j < 0. || i >= snx as f64 || j >= sny as f64 {
                    f64::NEG_INFINITY
                } else {
                    cells[j as usize * snx + i as usize].upper_mm
                }
            })
            .collect();
        let cleared = match self.memory.take() {
            Some(m) if m.grid == g && Some(m.generation) == Some(stock.generation()) => {
                self.report.stock_top_carried = true;
                self.carried_generation = Some(m.generation);
                m.cleared
            }
            Some(_) => {
                self.report.diagnostics.push("previous layer's steering memory did not match the committed stock; steering from whole-cell stock bounds only".into());
                vec![f64::INFINITY; n]
            }
            None => vec![f64::INFINITY; n],
        };
        self.report.engagement_grid_mm = g.c;
        self.layer = Some(Layer {
            g,
            top,
            cleared,
            inside,
            allowed,
            reachable,
            room: std::cell::OnceCell::new(),
            outside,
            dead: vec![false; n],
            candidates: vec![],
            thin: vec![],
            z: self.z,
            material_guard: 16. * self.op.tolerance_mm,
            journal: vec![],
            bits: MaterialBits::default(),
            open: Default::default(),
        });
        self.layer.as_mut().unwrap().sync_bits();
        // What was known of another layer's links is void.
        *self.link_reach.borrow_mut() = None;
        Ok(())
    }

    /// Recompute air/cleared start candidates: allowed centres whose cutter
    /// disk is clear of material while reachable material is within a few
    /// grid cells. Returns the reachable material area left.
    ///
    /// The two distances are asked of the material bits grown by a disk
    /// (`Near`) rather than of two distance transforms of the whole grid;
    /// `refresh_scan` is the definition by distance transforms.
    fn refresh(&mut self) -> f64 {
        let r = self.r;
        let layer = self.layer.as_mut().unwrap();
        let g = layer.g;
        let stride = layer.bits.stride;
        let cells: usize = layer
            .bits
            .reach
            .iter()
            .map(|w| w.count_ones() as usize)
            .sum();
        let left = cells as f64 * g.c * g.c;
        let whole = ([0, g.nx - 1], [0, g.ny - 1]);
        // Material within r + c blocks a start; reachable material within
        // r + 3 c makes one.
        let near = |bits: &[u64], t: f64| {
            cells2_within(g.c, t, true).map(|d2| Near::new(bits, stride, whole.0, whole.1, d2))
        };
        let blocked = near(&layer.bits.any, r + g.c);
        let beside = near(&layer.bits.reach, r + 3. * g.c);
        let mut candidates = vec![];
        if let Some(beside) = &beside {
            for j in 0..g.ny {
                for w in 0..stride {
                    let mut word = beside.grown[j * stride + w];
                    if let Some(blocked) = &blocked {
                        word &= !blocked.grown[j * stride + w];
                    }
                    while word != 0 {
                        let i = w * 64 + word.trailing_zeros() as usize;
                        word &= word - 1;
                        if i < g.nx {
                            let k = g.idx(i, j);
                            if layer.allowed[k] && !layer.dead[k] {
                                candidates.push(k);
                            }
                        }
                    }
                }
            }
        }
        #[cfg(test)]
        {
            let (area, scan) = Self::refresh_scan(layer, r);
            assert_eq!((left.to_bits(), &candidates), (area.to_bits(), &scan));
        }
        layer.candidates = candidates;
        self.report.grid_refreshes += 1;
        left
    }

    /// `refresh` by two distance transforms of the whole grid: the area
    /// left and the candidates.
    #[cfg(test)]
    fn refresh_scan(layer: &Layer, r: f64) -> (f64, Vec<usize>) {
        let g = layer.g;
        let n = g.nx * g.ny;
        let mat: Vec<bool> = (0..n).map(|k| layer.material(k)).collect();
        let reach: Vec<bool> = (0..n).map(|k| mat[k] && layer.reachable[k]).collect();
        let left = reach.iter().filter(|&&b| b).count() as f64 * g.c * g.c;
        let dmat = edt(&g, &mat);
        let dk = edt(&g, &reach);
        let candidates = (0..n)
            .filter(|&k| {
                layer.allowed[k] && !layer.dead[k] && dmat[k] > r + g.c && dk[k] <= r + 3. * g.c
            })
            .collect();
        (left, candidates)
    }

    /// Validate candidate `k` locally against the current grid: returns the
    /// unit direction towards the nearest reachable material.
    ///
    /// Read from the layer's material bits (#202); `candidate_direction_scan`
    /// is the plain scan of the disk it replaces and must agree with it to
    /// the bit, since the answer decides where a pass starts.
    fn candidate_direction(&self, k: usize) -> Option<Point> {
        let layer = self.layer.as_ref().unwrap();
        let found = (|| {
            if !layer.allowed[k] || layer.dead[k] {
                return None;
            }
            let p = layer.g.center(k);
            let c = layer.g.c;
            let rad = self.r + 3. * c;
            if layer.any_set_within(&layer.bits.any, p, rad, self.r + c) {
                return None;
            }
            let (_, q, d) = layer.nearest_set(&layer.bits.reach, p, rad)?;
            Some([(q[0] - p[0]) / d.max(1e-12), (q[1] - p[1]) / d.max(1e-12)])
        })();
        #[cfg(test)]
        assert_eq!(
            found.map(|u| u.map(f64::to_bits)),
            self.candidate_direction_scan(k)
                .map(|u| u.map(f64::to_bits)),
            "candidate {k}"
        );
        found
    }

    /// `candidate_direction` as a scan of every cell of the disk: the
    /// definition the fast search is tested against.
    #[cfg(test)]
    fn candidate_direction_scan(&self, k: usize) -> Option<Point> {
        let layer = self.layer.as_ref().unwrap();
        if !layer.allowed[k] || layer.dead[k] {
            return None;
        }
        let p = layer.g.center(k);
        let c = layer.g.c;
        let mut blocked = false;
        let mut nearest: Option<(f64, Point)> = None;
        layer.g.for_disk(p, self.r + 3. * c, |q, qc, d| {
            if layer.material(q) {
                if d <= self.r + c {
                    blocked = true;
                }
                if layer.reachable[q] && nearest.is_none_or(|(b, _)| d < b) {
                    nearest = Some((d, qc));
                }
            }
        });
        if blocked {
            return None;
        }
        let (d, q) = nearest?;
        Some([(q[0] - p[0]) / d.max(1e-12), (q[1] - p[1]) / d.max(1e-12)])
    }

    /// Nearest reachable material to `p` within `reach`: its distance and
    /// the unit direction to it. Read from the material bits, like
    /// `candidate_direction`, and tested against `nearest_material_scan`.
    fn nearest_material(&self, p: Point, reach: f64) -> Option<(f64, Point)> {
        let layer = self.layer.as_ref().unwrap();
        let found = layer
            .nearest_set(&layer.bits.reach, p, reach)
            .map(|(_, q, d)| {
                (
                    d,
                    [(q[0] - p[0]) / d.max(1e-12), (q[1] - p[1]) / d.max(1e-12)],
                )
            });
        #[cfg(test)]
        assert_eq!(
            found.map(|(d, u)| (d.to_bits(), u.map(f64::to_bits))),
            self.nearest_material_scan(p, reach)
                .map(|(d, u)| (d.to_bits(), u.map(f64::to_bits))),
            "nearest material to {p:?} within {reach}"
        );
        found
    }

    /// `nearest_material` as a scan of every cell of the disk.
    #[cfg(test)]
    fn nearest_material_scan(&self, p: Point, reach: f64) -> Option<(f64, Point)> {
        let layer = self.layer.as_ref().unwrap();
        let mut nearest: Option<(f64, Point)> = None;
        layer.g.for_disk(p, reach, |q, qc, d| {
            if layer.reachable[q] && layer.material(q) && nearest.is_none_or(|(b, _)| d < b) {
                nearest = Some((d, qc));
            }
        });
        let (d, q) = nearest?;
        Some((
            d,
            [(q[0] - p[0]) / d.max(1e-12), (q[1] - p[1]) / d.max(1e-12)],
        ))
    }

    /// The start a pass beside candidate `k` should take: the upstream end
    /// of the material it will cut.
    ///
    /// The cutting sense fixes the direction a pass travels along a front
    /// (material on one side). A start part-way along a strip of material
    /// leaves the part behind it for later passes, which then each start
    /// at the nearest point, just behind the last one, and cut only a
    /// cutter's width before running into cleared stock: a strip beside a
    /// wall was nibbled by a hundred short passes (#167). Walking back
    /// along the front, against the travel direction, until the material
    /// ends finds the start from which one pass cuts the whole strip. A
    /// front that closes on itself has no end: the candidate stands.
    fn upstream(&self, k: usize, dir: Point) -> (usize, Point) {
        let layer = self.layer.as_ref().unwrap();
        let g = layer.g;
        let (r, c) = (self.r, g.c);
        let side = if self.climb { 1. } else { -1. };
        let standoff = r + 2. * c;
        let step = 0.5 * r;
        let origin = g.center(k);
        let mut best = (k, dir);
        let mut p = origin;
        let mut along = dir;
        let limit = 2 * (g.nx + g.ny);
        for i in 0..limit {
            // Step back against the travel direction, then stand off the
            // nearest material again.
            let heading = [-side * along[1], side * along[0]];
            let back = [p[0] - step * heading[0], p[1] - step * heading[1]];
            let Some((d, to)) = self.nearest_material(back, standoff + step) else {
                break;
            };
            let q = [
                back[0] + (d - standoff) * to[0],
                back[1] + (d - standoff) * to[1],
            ];
            let Some((ci, cj)) = g.cell(q) else { break };
            let cell = g.idx(ci, cj);
            let Some(found) = self.candidate_direction(cell) else {
                break;
            };
            if i > 2 && (q[0] - origin[0]).hypot(q[1] - origin[1]) < step {
                // Back where it began: a closed front.
                return (k, dir);
            }
            best = (cell, found);
            p = g.center(cell);
            along = found;
        }
        best
    }

    fn mark_dead(&mut self, p: Point, cells: i64) {
        let layer = self.layer.as_mut().unwrap();
        let Some((i, j)) = layer.g.cell(p) else {
            return;
        };
        for dj in -cells..=cells {
            for di in -cells..=cells {
                let (ii, jj) = (i as i64 + di, j as i64 + dj);
                if ii >= 0 && jj >= 0 && (ii as usize) < layer.g.nx && (jj as usize) < layer.g.ny {
                    let k = layer.g.idx(ii as usize, jj as usize);
                    layer.dead[k] = true;
                }
            }
        }
    }

    /// Follow the material front from `start`. Returns the centre polyline
    /// (first point = start) and whether any material was cut.
    fn follow(
        &mut self,
        start: Point,
        heading: f64,
        approach: bool,
        narrow: f64,
    ) -> (Vec<Point>, bool, bool) {
        let r = self.r;
        // Steering width: the declared width less what the grid cannot
        // resolve (see `steering_width`), narrowed by `narrow` when an
        // earlier attempt at this pass was not proven within the limit.
        let w = narrow * self.steering_width();
        let limit = (1. - w / r).acos();
        let side = if self.climb { 1. } else { -1. };
        let remnant = self.remnant;
        let layer = self.layer.as_mut().unwrap();
        let c = layer.g.c;
        let ds = (0.15 * r).min(0.6 * w).max(2. * c);
        let target = w * ds;
        let fmax = 0.6 * PI;
        let mut cells = std::mem::take(&mut self.scratch);
        let mut near = std::mem::take(&mut self.scratch_load);
        let mut pts = vec![start];
        let mut p = start;
        let mut th = heading;
        let mut engaged = !approach;
        let mut approached = 0.;
        let mut cut_any = false;
        let mut idle = 0;
        let mut max_eng: f64 = 0.;
        let mut end = 0;
        for _ in 0..2_000_000 {
            if pts.len() > PROOF_CHUNK {
                // Enough to prove at once; the pass continues from here.
                end = 4;
                break;
            }
            layer.gather(p, r + ds + 2. * c, &mut cells);
            // Every sum of this step is taken at a point `ds` from `p`.
            step_cells(&cells, p, ds, r, c, &mut near);
            let f = |phi: f64, layer: &Layer| {
                let q = [p[0] + ds * (th + phi).cos(), p[1] + ds * (th + phi).sin()];
                if !layer.allowed_at(q) {
                    return f64::INFINITY;
                }
                let a = area_of(&near, q, r, c);
                #[cfg(test)]
                assert_eq!(a.to_bits(), area_plain(&cells, q, r, c).to_bits());
                a
            };
            if !engaged {
                let a0 = f(-side * PI / 2., layer);
                if a0 == 0. && approached < 0.4 * r {
                    p = [
                        p[0] + ds * (th - side * PI / 2.).cos(),
                        p[1] + ds * (th - side * PI / 2.).sin(),
                    ];
                    approached += ds;
                    pts.push(p);
                    continue;
                }
                engaged = true;
            }
            // Material side is -side*fmax; the far side is +side*fmax.
            let into = f(-side * fmax, layer);
            let away = f(side * fmax, layer);
            // Material too thin for a full step ends the pass. A remnant
            // pass that has cut nothing yet still takes its steps: a sliver
            // thinner than a quarter step (left between passes, e.g. along
            // the stock face) is cleared from here, at depth beside cleared
            // stock, rather than bored onto by a helix. A few such steps
            // that clear nothing end it as before.
            if (!remnant || cut_any || idle >= 4)
                && into.min(1e9) < 0.25 * target
                && away < 0.25 * target
            {
                let ahead = f(0., layer);
                if ahead.is_infinite() || ahead < 0.25 * target {
                    end = 0;
                    break;
                }
            }
            if away > 1.3 * target {
                end = 1;
                break;
            }
            // A step's load: the area it removes against the target, or its
            // estimated contact angle against the limit, whichever is
            // nearer its bound. Area alone under-reads the contact where
            // the path turns into the material.
            let load = |phi: f64, layer: &Layer| {
                let u = [(th + phi).cos(), (th + phi).sin()];
                let q = [p[0] + ds * u[0], p[1] + ds * u[1]];
                if !layer.allowed_at(q) {
                    return f64::INFINITY;
                }
                let (a, contact) = step_load_of(&near, q, u, r, c);
                #[cfg(test)]
                {
                    let plain = step_load_plain(&cells, q, u, r, c);
                    assert_eq!(
                        (a.to_bits(), contact.to_bits()),
                        (plain.0.to_bits(), plain.1.to_bits())
                    );
                }
                a.max(target * contact / limit)
            };
            // Turn from the far side towards the material until a step
            // would reach the target load, then bisect that bracket. The
            // load is not monotone in the turn, so scan before bisecting.
            const SCAN: usize = 16;
            let mut prev = (fmax, away);
            let mut best = prev;
            let mut bracket = None;
            for i in 1..=SCAN {
                let u = fmax - 2. * fmax * i as f64 / SCAN as f64;
                let a = load(side * u, layer);
                if a > target {
                    bracket = Some((prev.0, u));
                    break;
                }
                if a > best.1 {
                    best = (u, a);
                }
                prev = (u, a);
            }
            let phi = match bracket {
                Some((mut ok, mut over)) => {
                    for _ in 0..10 {
                        let mid = 0.5 * (ok + over);
                        if load(side * mid, layer) > target {
                            over = mid;
                        } else {
                            ok = mid;
                        }
                    }
                    side * ok
                }
                None => side * best.0,
            };
            let a = f(phi, layer);
            if !a.is_finite() || a < 0.02 * target {
                end = if a.is_finite() { 2 } else { 3 };
                break;
            }
            th += phi;
            p = [p[0] + ds * th.cos(), p[1] + ds * th.sin()];
            max_eng = max_eng.max(a / ds);
            // Progress is material this plan marks cleared, not material the
            // step merely meets: cells just beyond the cutter's reach along a
            // boundary count in the step's area but are never cleared, and a
            // pass that only meets them would be planned again forever.
            cut_any |= layer.cut(p, r) > 0;
            idle += usize::from(!cut_any);
            pts.push(p);
        }
        self.scratch = cells;
        self.scratch_load = near;
        if cut_any {
            match end {
                0 => self.report.pass_ends_exhausted += 1,
                1 => self.report.pass_ends_overload += 1,
                2 => self.report.pass_ends_thin += 1,
                3 => self.report.pass_ends_boundary += 1,
                _ => {}
            }
            self.report.max_estimated_engagement_mm =
                self.report.max_estimated_engagement_mm.max(max_eng);
        } else {
            pts.truncate(1);
        }
        (pts, cut_any, end == 4)
    }

    /// A stay-down link from `a` to `b` through already cleared stock: the
    /// shortest 8-connected grid route whose cutter centres keep material at
    /// least `r - w/2` away, simplified, then re-checked step by step with
    /// [`Self::link_samples`] and certified exactly against the region.
    /// Returns (waypoints after `a`, samples to cut, widest step).
    fn routed_link(
        &self,
        a: Point,
        b: Point,
        max_len: f64,
    ) -> Option<(Vec<Point>, Vec<Point>, f64)> {
        let layer = self.layer.as_ref().unwrap();
        let g = layer.g;
        let (r, c) = (self.r, g.c);
        let straight = (b[0] - a[0]).hypot(b[1] - a[1]);
        if straight > max_len {
            return None;
        }
        let margin = 2. * r + 0.5 * (max_len - straight).min(straight) + 4. * c;
        let (ia, ja) = g.cell(a)?;
        let (ib, jb) = g.cell(b)?;
        let m = (margin / c).ceil() as usize;
        let i0 = ia.min(ib).saturating_sub(m);
        let j0 = ja.min(jb).saturating_sub(m);
        let i1 = (ia.max(ib) + m).min(g.nx - 1);
        let j1 = (ja.max(jb) + m).min(g.ny - 1);
        // A goal that no link from this start reaches within the length
        // limit (`Layer::link_reach`, worked out when a second search from
        // the same start over the same material fails to reach its goal:
        // starts are tried one after another from where the cutter is).
        // The search below would not return it; a test build runs it and
        // checks that.
        let key = (layer.bits.version, g.idx(ia, ja), max_len.to_bits());
        let known_reach = |known: &LinkReach| (known.version, known.start, known.max_len) == key;
        let out_of_reach = self.link_reach.borrow().as_ref().is_some_and(|known| {
            known_reach(known) && known.covers([i0, i1], [j0, j1]) && known.excludes(ib, jb)
        });
        #[cfg(not(test))]
        if out_of_reach {
            return None;
        }
        #[cfg(test)]
        if out_of_reach {
            shortcut_taken(1);
        }
        let w = Grid {
            x0: g.x0 + i0 as f64 * c,
            y0: g.y0 + j0 as f64 * c,
            c,
            nx: i1 - i0 + 1,
            ny: j1 - j0 + 1,
        };
        // Material of the window nearer than a distance, from the material
        // bits (`Near`): what `dmat[k] < t` says of the window's distance
        // transform `dmat`, which used to be computed for every link.
        let nearer = |t: f64| {
            cells2_within(c, t, false)
                .map(|d2| Near::new(&layer.bits.any, layer.bits.stride, [i0, i1], [j0, j1], d2))
        };
        let (tight, loose) = (nearer(r - 0.5 * self.width), nearer(r - self.width));
        // (By window column and row: the search below knows them.)
        let clear = |near: &Option<Near>, i: usize, j: usize| {
            !near.as_ref().is_some_and(|n| n.at(i0 + i, j0 + j))
        };
        // Material beyond the window is unseen: its border ring is closed.
        let ring = ((r + c) / c).ceil() as usize + 1;
        let passable = |i: usize, j: usize| {
            let border = (i < ring && i0 > 0)
                || (j < ring && j0 > 0)
                || (i + ring >= w.nx && i1 + 1 < g.nx)
                || (j + ring >= w.ny && j1 + 1 < g.ny);
            !border && layer.allowed[g.idx(i0 + i, j0 + j)] && clear(&tight, i, j)
        };
        #[cfg(test)]
        {
            let mat: Vec<bool> = (0..w.nx * w.ny)
                .map(|k| layer.material(g.idx(i0 + k % w.nx, j0 + k / w.nx)))
                .collect();
            let dmat = edt(&w, &mat);
            for (k, &d) in dmat.iter().enumerate() {
                let (i, j) = (k % w.nx, k / w.nx);
                assert_eq!(clear(&tight, i, j), d >= r - 0.5 * self.width);
                assert_eq!(clear(&loose, i, j), d >= r - self.width);
            }
        }
        let start = w.idx(ia - i0, ja - j0);
        let goal = w.idx(ib - i0, jb - j0);
        if !passable(ib - i0, jb - j0) {
            return None;
        }
        // Dijkstra on the window (the start may still touch the front).
        let limit = max_len / c;
        let enter = |i: usize, j: usize, d: f64| {
            passable(i, j) || (d < 2. * self.width / c && clear(&loose, i, j))
        };
        // First in a corridor around the straight way: the cells from which
        // start and goal are together within `corridor` (in 1/1024 of a
        // cell, by `link_distance`). If that search ends by taking the goal
        // from the queue at a distance within the corridor's and the
        // link's limits, it is the whole window's search as far as the
        // link goes: the same distance of the goal and the same cells
        // behind it.
        //
        // Why: call a queued cell light when its distance in the queue plus
        // its `link_distance` to the goal is within `corridor`, and heavy
        // otherwise. A cell is queued at least `link_distance` beyond any
        // cell it descends from, so (1) a cell outside the corridor is
        // heavy, as is everything reached through a heavy cell; (2) of two
        // distances for one cell, the smaller is light if the larger is,
        // so a heavy distance never replaces a light one nor keeps one
        // out. The queue hands cells out in one fixed order whatever else
        // it holds (`LinkQueue`), so the light cells are taken in the same
        // order, find the same distances and are reached from the same
        // cells in both searches; they differ in heavy cells only. A goal
        // taken at a light distance within the link's limit ends both
        // searches there: an earlier end would be a goal taken at a
        // smaller, also light, distance, or a distance beyond the limit
        // taken before a smaller one. Otherwise the whole window is
        // searched.
        let size = [w.nx, w.ny];
        let (ga, gb) = (start % w.nx, start / w.nx);
        let (ha, hb) = (goal % w.nx, goal / w.nx);
        let corridor = link_distance(ga.abs_diff(ha), gb.abs_diff(hb)) * 5 / 4 + 12 * 1024;
        let inside = |i: usize, j: usize| {
            link_distance(i.abs_diff(ga), j.abs_diff(gb))
                + link_distance(i.abs_diff(ha), j.abs_diff(hb))
                <= corridor
        };
        let narrow = link_search(
            LinkBuckets::default(),
            size,
            start,
            goal,
            limit,
            |i, j, d| inside(i, j) && enter(i, j, d),
        );
        let settled = narrow
            .2
            .is_some_and(|d| d <= limit && d * 1024. <= corridor as f64);
        #[cfg(test)]
        if settled {
            shortcut_taken(2);
        }
        let (dist, prev, _) = if settled {
            narrow
        } else {
            link_search(LinkBuckets::default(), size, start, goal, limit, enter)
        };
        #[cfg(test)]
        {
            // The definition: the whole window, from a heap.
            let heap = std::collections::BinaryHeap::new();
            let (d, p, _) = link_search(heap, size, start, goal, limit, enter);
            assert_eq!(dist[goal].to_bits(), d[goal].to_bits());
            if dist[goal].is_finite() {
                let (mut a, mut b) = (goal, goal);
                while a != start {
                    (a, b) = (prev[a], p[b]);
                    assert_eq!(a, b, "the cells behind the goal");
                }
            }
            if !settled {
                assert!(prev == p);
            }
        }
        #[cfg(test)]
        assert!(!out_of_reach || !dist[goal].is_finite() || dist[goal] * c > max_len);
        if !dist[goal].is_finite() || dist[goal] * c > max_len {
            // The second such search from this start in a row: the goals
            // tried next are asked of the reach first. Its box holds this
            // window and those of goals some way around this one.
            if self.link_failed.replace(Some(key)) == Some(key) {
                let mut known = self.link_reach.borrow_mut();
                if !known
                    .as_ref()
                    .is_some_and(|k| known_reach(k) && k.covers([i0, i1], [j0, j1]))
                {
                    const AROUND: usize = 160;
                    *known = Some(layer.link_reach(
                        (ia, ja),
                        r - 0.5 * self.width,
                        (2. * self.width / c).ceil() as usize + 2,
                        [i0.saturating_sub(AROUND), (i1 + AROUND).min(g.nx - 1)],
                        [j0.saturating_sub(AROUND), (j1 + AROUND).min(g.ny - 1)],
                        max_len,
                    ));
                }
            }
        }
        if !dist[goal].is_finite() || dist[goal] * c > max_len {
            return None;
        }
        let mut chain = vec![b];
        let mut k = prev[goal];
        while k != start && k != usize::MAX {
            chain.push(w.center(k));
            k = prev[k];
        }
        chain.push(a);
        chain.reverse();
        // Simplify greedily: the longest straight jump each step may take
        // while the bounded-engagement check and region certificate hold.
        let mut waypoints = vec![];
        let mut samples = vec![];
        let mut widest: f64 = 0.;
        let mut at = 0;
        while at + 1 < chain.len() {
            let mut best = None;
            let mut hi = chain.len() - 1;
            loop {
                if let Some((s, wd)) = self.link_samples(chain[at], chain[hi])
                    && self
                        .guides
                        .stroke_geometry_fits_radius(Stroke::line(chain[at], chain[hi]), r)
                {
                    best = Some((hi, s, wd));
                    break;
                }
                if hi == at + 1 {
                    break;
                }
                hi = at + 1 + (hi - at - 1) / 2;
            }
            let (next, s, wd) = best?;
            samples.extend(s);
            widest = widest.max(wd);
            waypoints.push(chain[next]);
            at = next;
        }
        Some((waypoints, samples, widest))
    }

    /// Front step length: short against the cutter and the target width.
    fn step_mm(&self, c: f64) -> f64 {
        (0.15 * self.r).min(0.6 * self.width).max(2. * c)
    }
    /// Whether a straight stay-down link keeps the engagement bounded: every
    /// short step may remove at most the target width times its length
    /// (plus one grid cell of slack). Returns the samples to cut.
    fn link_samples(&self, a: Point, b: Point) -> Option<(Vec<Point>, f64)> {
        let layer = self.layer.as_ref().unwrap();
        let c = layer.g.c;
        let r = self.r;
        let len = (b[0] - a[0]).hypot(b[1] - a[1]);
        let n = (len / self.step_mm(c)).ceil().max(1.) as usize;
        let h = len / n as f64;
        let mut cells = vec![];
        let mut samples = Vec::with_capacity(n + 1);
        let mut prev = a;
        let mut widest: f64 = 0.;
        samples.push(a);
        for t in 1..=n {
            let f = t as f64 / n as f64;
            let p = [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f];
            if !layer.allowed_at(p) {
                return None;
            }
            layer.gather(p, r + c, &mut cells);
            let mut new = fresh_cells(&cells, prev, p, r, c);
            new *= c * c;
            if new > self.width * h + 0.5 * c * c {
                return None;
            }
            widest = widest.max(new / h);
            samples.push(p);
            prev = p;
        }
        Some((samples, widest))
    }

    fn certify(&mut self, s: Stroke) -> Result<(), String> {
        if !self.guides.stroke_geometry_fits_radius(s, self.r) {
            return Err(format!(
                "engagement_front planned a stroke from {:?} to {:?} that the exact region check rejects; this is a planner defect, please report it",
                s.a, s.b
            ));
        }
        self.report.certified_strokes += 1;
        Ok(())
    }

    fn queue(&mut self, curve: Curve, kind: MoveKind, feed: Option<f64>) {
        if curve.start() != curve.end() || matches!(curve, Curve::Helix { .. } | Curve::Arc { .. })
        {
            self.pending.push_back((curve, kind, feed));
        }
    }
    fn line(&mut self, a: Vec3, b: Vec3, kind: MoveKind) {
        self.queue(Curve::Line { start: a, end: b }, kind, None);
    }
    /// A stay-down link at depth. With a link feed, the stretches whose
    /// whole cutter footprint the committed stock's whole-cell upper bounds
    /// prove clear of stock (`Stock::may_remove_material`, the bounds the
    /// stream's stock checks read) run at the link feed; every other stretch
    /// keeps the cutting feed, and the fast stretch ends before the cutter
    /// could reach possible stock. Stretches are at most `LINK_PIECE` long
    /// so a link that only grazes stock near one end is fast elsewhere.
    fn link(
        &mut self,
        stock: &Stock,
        a: Vec3,
        b: Vec3,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        let Some(fast) = self.link_feed else {
            self.line(a, b, MoveKind::Link);
            return Ok(());
        };
        let length = (b[0] - a[0]).hypot(b[1] - a[1]);
        if length == 0. {
            return Ok(());
        }
        let n = (length / LINK_PIECE).ceil().max(1.) as usize;
        let at = |i: usize| -> Vec3 {
            let t = i as f64 / n as f64;
            [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2]]
        };
        let mut runs: Vec<(Vec3, Vec3, Option<f64>)> = vec![];
        // Cells this layer cut keep an upper bound a path error above its
        // depth: material no thicker than the layer's material guard (16
        // tolerances, as `Layer::material`) counts as none.
        let skin = (16. * self.op.tolerance_mm).min(0.1);
        for i in 0..n {
            let (p, q) = (at(i), if i + 1 == n { b } else { at(i + 1) });
            let probe = Motion {
                id: 0,
                operation: self.op.id.clone(),
                tool: self.op.tool.clone(),
                kind: MoveKind::Link,
                curve: Curve::Line { start: p, end: q },
                feed_mm_min: Some(fast),
            };
            let mut work = 1usize << 20;
            let air = self
                .guides
                .cleared_link([p[0], p[1]], [q[0], q[1]], p[2], stock, cancel)?
                || !stock.may_remove_material(
                    &probe,
                    self.r,
                    skin,
                    self.op.tolerance_mm,
                    &mut work,
                    cancel,
                )?;
            let feed = if air {
                Some(fast)
            } else if self.chip_thinning {
                self.thinned_feed(stock, p, q, skin, fast)
            } else {
                None
            };
            match runs.last_mut() {
                Some(run) if run.2 == feed => run.1 = q,
                _ => runs.push((p, q, feed)),
            }
        }
        for (p, q, feed) in runs {
            let length = (q[0] - p[0]).hypot(q[1] - p[1]);
            if feed == Some(fast) {
                self.report.fast_links += 1;
                self.report.fast_link_mm += length;
            } else if feed.is_some() {
                self.report.thinned_links += 1;
                self.report.thinned_link_mm += length;
            }
            self.queue(Curve::Line { start: p, end: q }, MoveKind::Link, feed);
        }
        Ok(())
    }

    /// Radial chip thinning on a link stretch `p`-`q` that may touch stock:
    /// the feed at which no chip is thicker than at the cutting feed and the
    /// clearing's radial engagement, capped at `cap`. None (the cutting feed)
    /// when that feed is not above the cutting feed.
    ///
    /// On a straight move a tooth at angle t from the feed direction cuts a
    /// chip fz * cos(t) (Sandvik Coromant, "radial chip thinning": for a
    /// radial engagement ae < D/2 the maximum is fz * 2 * sqrt(ae/D -
    /// (ae/D)^2)), and it meets material only where material lies at its
    /// lateral offset r * sin(t) from the line of motion. So if no material
    /// the cutter can reach on the stretch lies within lateral offset s of
    /// the line, no chip is thicker than fz * sqrt(1 - (s/r)^2), the chip of
    /// a side cut with ae = r - s. This holds at the stretch's ends too, and
    /// for corner-radius cutters, whose smaller radius near the tip makes the
    /// tooth angle for the same offset only more oblique. Material is bounded
    /// by the stock's whole-cell upper bounds: a cell whose bound is above
    /// the link depth plus `skin` may hold material anywhere in its extent.
    fn thinned_feed(&self, stock: &Stock, p: Vec3, q: Vec3, skin: f64, cap: f64) -> Option<f64> {
        let (r, w) = (self.r, self.width);
        let d_tool = 2. * r;
        if w >= r {
            return None;
        }
        let [nx, ny] = stock.dimensions();
        let [wx, wy] = stock.cell_widths_mm();
        let min = stock.spec().min_mm;
        let cells = stock.cells();
        let half = wx.hypot(wy) / 2.;
        let (dx, dy) = (q[0] - p[0], q[1] - p[1]);
        let length2 = dx * dx + dy * dy;
        if length2 == 0. {
            return None;
        }
        let length = length2.sqrt();
        let (nxv, nyv) = (-dy / length, dx / length);
        let reach = r + half;
        let lo = |v: f64, o: f64, c: f64| ((v - reach - o) / c).floor().max(0.) as usize;
        let hi = |v: f64, o: f64, c: f64, n: usize| {
            (((v + reach - o) / c).floor() as i64).min(n as i64 - 1)
        };
        let (i1, j1) = (
            hi(p[0].max(q[0]), min[0], wx, nx),
            hi(p[1].max(q[1]), min[1], wy, ny),
        );
        // Smallest lateral offset of possible material the cutter can reach.
        let mut offset = r;
        if i1 >= 0 && j1 >= 0 {
            for j in lo(p[1].min(q[1]), min[1], wy)..=j1 as usize {
                for i in lo(p[0].min(q[0]), min[0], wx)..=i1 as usize {
                    if cells[j * nx + i].upper_mm <= p[2] + skin {
                        continue;
                    }
                    let c = [
                        min[0] + (i as f64 + 0.5) * wx,
                        min[1] + (j as f64 + 0.5) * wy,
                    ];
                    let t = (((c[0] - p[0]) * dx + (c[1] - p[1]) * dy) / length2).clamp(0., 1.);
                    if (c[0] - p[0] - t * dx).hypot(c[1] - p[1] - t * dy) >= reach {
                        continue;
                    }
                    let lateral = ((c[0] - p[0]) * nxv + (c[1] - p[1]) * nyv).abs() - half;
                    offset = offset.min(lateral.max(0.));
                }
            }
        }
        // Chip factor 2 sqrt(x - x^2), x = ae/D.
        let factor = |a: f64| {
            let x = a / d_tool;
            2. * (x - x * x).max(0.).sqrt()
        };
        let ae = r - offset;
        if ae >= w {
            return None;
        }
        let reference = self.op.feed_mm_min;
        let thin = factor(ae);
        let feed = if thin <= 0. {
            cap
        } else {
            (reference * factor(w) / thin).min(cap)
        };
        // Whole 10 mm/min steps, rounded down, keep blocks few and never
        // exceed the computed feed.
        let feed = (feed / 10.).floor() * 10.;
        (feed > reference).then_some(feed)
    }

    /// Lift at feed to the feed height, then rapid to `to_z`; returns the pose.
    fn lift(&mut self, from: Vec3, to_z: f64) -> Vec3 {
        let feed_z = self.heights.feed.max(from[2]);
        let safe = [from[0], from[1], feed_z];
        let up = [from[0], from[1], to_z.max(feed_z)];
        self.line(from, safe, MoveKind::Retract);
        self.line(safe, up, MoveKind::Rapid);
        up
    }
    /// Rapid to above `to`, descending to the feed height, or to the
    /// rapid-descent clearance above `proven` when that is lower: a height
    /// down to which the committed sweeps prove the whole tool's descent at
    /// `to` clear, raised to the whole-cell stock bounds under the tool so
    /// the stream's stock-contact check certifies the rapid too. Returns the
    /// pose.
    /// A height `cleared_entry_start` returned, when it is a proof: it
    /// returns the operation top unexamined when nothing is committed yet.
    fn proven(&self, height: f64) -> Option<f64> {
        (height < self.op.top_z_mm - crate::entry::skin(self.op.tolerance_mm)).then_some(height)
    }
    /// Rapid from `from` up to the retract height, across to `to`, and down
    /// to the feed height (or the proven rapid-descent stop below it), never
    /// below `at_least`: the caller's next motion starts there, so the rapid
    /// ends at or above it and every endpoint connects. With
    /// `travel_clearance_mm`, the rapid across runs lower when the
    /// whole-cell stock bounds allow: see `travel_height`.
    fn travel(
        &mut self,
        stock: &Stock,
        from: Vec3,
        to: Point,
        proven: Option<f64>,
        at_least: f64,
    ) -> Vec3 {
        let height = self.travel_height(stock, from, to, at_least);
        let up = if height < self.heights.feed {
            // Below the feed height: straight up, then across.
            let up = [from[0], from[1], height];
            self.line(from, up, MoveKind::Retract);
            self.report.lowered_travels += 1;
            up
        } else if (from[2] - height).abs() < 1e-12 || from[2] >= self.heights.feed {
            let up = [from[0], from[1], from[2].max(height)];
            self.line(from, up, MoveKind::Rapid);
            up
        } else {
            self.lift(from, height)
        };
        if height < self.heights.retract && height >= self.heights.feed {
            self.report.lowered_travels += 1;
        }
        let across = [to[0], to[1], up[2]];
        self.line(up, across, MoveKind::Rapid);
        let stop = match (self.rapid_descent, proven) {
            (Some(d), Some(h)) => self.heights.feed.min(h + d),
            _ => self.heights.feed,
        }
        .max(at_least)
        .min(up[2]);
        let above = [to[0], to[1], stop];
        self.line(across, above, MoveKind::Rapid);
        above
    }

    /// The height of the rapid across from `from` to above `to`: the retract
    /// height, or with `travel_clearance_mm` that far above the highest
    /// whole-cell stock upper bound any tool section can meet anywhere in
    /// the rectangle spanned by the two ends (so either axis-first dogleg is
    /// clear too), when that is lower. Never below `from` or `at_least`.
    /// Stock only loses material, so bounds of the committed stock stay
    /// upper bounds while queued cuts are still to be committed.
    fn travel_height(&self, stock: &Stock, from: Vec3, to: Point, at_least: f64) -> f64 {
        let retract = self.heights.retract;
        let Some(clearance) = self.travel_clearance else {
            return retract;
        };
        // The rectangle every route between the ends lies in, grown by the
        // widest tool section: what the tool can be over on the way.
        let reach = self.guides.widest_part_radius() + 32. * self.op.tolerance_mm;
        let (lo, hi) = (
            [from[0].min(to[0]) - reach, from[1].min(to[1]) - reach],
            [from[0].max(to[0]) + reach, from[1].max(to[1]) + reach],
        );
        // A controller may run a rapid as a straight line, as either
        // axis-first dogleg, or with each axis at its own rate (a diagonal,
        // then the rest along one axis): anywhere in the rectangle of its
        // two ends, which is what the stock bound and the obstacles below
        // are taken over. Both ends are positions the layer's region allows
        // the cutter at; beside the stock, where no stock bound says
        // anything, the rectangle between two such positions is taken to be
        // as free as they are (the premise the entries beside the stock
        // already rest on). A containment region withdraws that premise.
        // Declared obstacles. A containment region is kept by staying at the
        // retract height (a dogleg may leave it where the straight route
        // does not); an avoid area the rectangle meets is crossed above its
        // top by the same clearance. Areas that cannot be measured keep the
        // retract height too.
        let mut height = self.guides.cell_route_height(stock, [from[0], from[1]], to);
        if !height.is_finite() {
            return retract;
        }
        if let Some(limits) = &self.op.limits {
            if limits.contain.is_some() {
                return retract;
            }
            let boxes = self.avoid_boxes.get_or_init(|| {
                limits
                    .avoid
                    .iter()
                    .map(|area| {
                        let region = area
                            .region
                            .prepare(crate::planar::ConstructionOptions {
                                tolerance_mm: self.op.tolerance_mm,
                            })
                            .ok()?;
                        Some((region.bounds()?, area.top_z_mm))
                    })
                    .collect::<Option<Vec<_>>>()
            });
            let Some(boxes) = boxes else {
                return retract;
            };
            for &([x0, y0, x1, y1], top) in boxes {
                if x0 <= hi[0] && x1 >= lo[0] && y0 <= hi[1] && y1 >= lo[1] {
                    height = height.max(top);
                }
            }
        }
        (height + clearance).max(from[2]).max(at_least).min(retract)
    }

    /// A vertical entry at `q` sits beside the material it is about to cut,
    /// so the whole-cell stock bounds under the tool there still hold that
    /// material's height and the rapid descent must stop above it (the
    /// stream's stock-contact check reads those same bounds). When a point a
    /// few stock cells back from the material (opposite `toward`) is proven
    /// clear lower down, by the committed sweeps and the cell bounds both,
    /// descend there instead and feed across to `q` at depth: the plunge
    /// through cleared air becomes a rapid. The link is part-certified and
    /// sampled against the engagement limit like a stay-down link. Returns
    /// the descent point, its proven height and the link samples.
    #[allow(clippy::type_complexity)]
    fn offset_descent(
        &self,
        stock: &Stock,
        q: Point,
        toward: Point,
        proven: Option<f64>,
        cancel: &impl CancelSignal,
    ) -> Result<Option<(Point, Option<f64>, Vec<Point>)>, String> {
        let (Some(d), Some(h)) = (self.rapid_descent, proven) else {
            return Ok(None);
        };
        let z = self.z;
        let stop = self.heights.feed.min(h + d);
        // Worth a link only when it saves real feed travel.
        if stop - (z + d) <= 1. {
            return Ok(None);
        }
        let [wx, wy] = stock.cell_widths_mm();
        let w = wx.max(wy);
        let guard = crate::entry::skin(self.op.tolerance_mm);
        for n in [2., 4., 8.] {
            let back = (n * w).min(self.r);
            let p = [q[0] - back * toward[0], q[1] - back * toward[1]];
            if !self.layer.as_ref().unwrap().allowed_at(p)
                || !self
                    .guides
                    .stroke_geometry_fits_radius(Stroke::line(p, q), self.r)
            {
                continue;
            }
            let (height, _) = self
                .guides
                .cleared_entry_start(p, 0., z, Some(stock), cancel)?;
            if height - z > guard {
                continue;
            }
            let Some(at) = self
                .proven(height)
                .map(|h| h.max(self.guides.cell_entry_height(stock, p)))
            else {
                continue;
            };
            if self.heights.feed.min(at + d) > stop - 1. {
                continue;
            }
            let Some((samples, _)) = self.link_samples(p, q) else {
                continue;
            };
            return Ok(Some((p, Some(at), samples)));
        }
        Ok(None)
    }

    fn emit_cuts(&mut self, pts: &[Point]) -> Result<(), String> {
        let c = self.layer.as_ref().unwrap().g.c;
        let z = self.z;
        let guides = &self.guides;
        let r = self.r;
        let (strokes, refitted) = fit_certifiable(pts, FIT_FRACTION * c, 1000. * r, |s| {
            guides.stroke_geometry_fits_radius(s, r)
        });
        self.report.refitted_arcs += refitted;
        for s in strokes {
            self.certify(s)?;
            if s.arc.is_some() {
                self.report.arcs += 1;
            }
            self.queue(s.curve(z), MoveKind::Cut, None);
        }
        Ok(())
    }

    /// Plan the next pass (link or entry plus cut) into `pending`. Returns
    /// false when the layer is complete.
    fn plan_pass(&mut self, stock: &Stock, cancel: &impl CancelSignal) -> Result<bool, String> {
        let r = self.r;
        let z = self.z;
        let min_area = 0.02 * r * r;
        let keep_down = 6. * r;
        let mut refreshed = false;
        loop {
            if cancel.is_cancelled() {
                return Err("engagement_front planning cancelled".into());
            }
            let pos = [self.position[0], self.position[1]];
            // Nearest valid start among the cached candidates.
            let mut found = None;
            // The candidates by distance, measured once: a stale candidate
            // is dropped and the next nearest taken from the queue, where
            // every drop used to scan the whole list again (#192).
            let mut queue = StartQueue::new(self.layer.as_ref().unwrap(), pos);
            loop {
                // A fine grid has many stale candidates to drop in a row.
                if cancel.is_cancelled() {
                    return Err("engagement_front planning cancelled".into());
                }
                let Some(k) = queue.take(self.layer.as_mut().unwrap()) else {
                    break;
                };
                if let Some(dir) = self.candidate_direction(k) {
                    found = Some((k, dir));
                    break;
                }
            }
            let Some((k, dir)) = found else {
                if refreshed {
                    if self.revive_thin_starts() {
                        self.remnant = true;
                        continue;
                    }
                    self.remnant = false;
                    return self.helix_pass(stock, cancel);
                }
                let left = self.refresh();
                // Planning progress (#198): what is left against the most
                // this level has had (a refresh may find more reachable).
                let start = self.area_mm2.map_or(left, |(_, start)| start.max(left));
                self.area_mm2 = Some((left, start));
                refreshed = true;
                if left < min_area {
                    return Ok(false);
                }
                continue;
            };
            // Start where the material this pass will cut begins (see
            // `upstream`), not merely at the nearest point beside it.
            let (k, dir) = self.upstream(k, dir);
            let q = self.layer.as_ref().unwrap().g.center(k);
            let here = self.position;
            let at_layer = (here[2] - z).abs() < 1e-9;
            let len = (q[0] - here[0]).hypot(q[1] - here[1]);
            let from = [here[0], here[1]];
            let link = if !at_layer {
                None
            } else if len <= keep_down
                && let Some((samples, widest)) = self.link_samples(from, q)
                && self
                    .guides
                    .stroke_geometry_fits_radius(Stroke::line(from, q), r)
            {
                Some((vec![q], samples, widest))
            } else {
                self.routed_link(from, q, 3. * keep_down)
            };
            let mut proven = None;
            let mut descent = None;
            if link.is_none() {
                // A vertical entry needs the committed sweeps (or air outside
                // the original stock) to prove the whole column clear.
                let (height, _) = self
                    .guides
                    .cleared_entry_start(q, 0., z, Some(stock), cancel)?;
                if height - z > crate::entry::skin(self.op.tolerance_mm) {
                    self.report.unproven_air_starts += 1;
                    self.mark_dead(q, 1);
                    continue;
                }
                proven = self
                    .proven(height)
                    .map(|h| h.max(self.guides.cell_entry_height(stock, q)));
                descent = self.offset_descent(stock, q, dir, proven, cancel)?;
            }
            let side = if self.climb { 1. } else { -1. };
            let heading = (side * dir[0]).atan2(-side * dir[1]);
            // The link or entry is queued first, so the pass's cuts are
            // proven with it already cleared; all of it is undone if the
            // pass turns out to cut nothing.
            self.layer.as_mut().unwrap().journal.clear();
            let base = self.checkpoint();
            if let Some((waypoints, samples, widest)) = link {
                let layer = self.layer.as_mut().unwrap();
                for p in samples {
                    layer.cut(p, r);
                }
                self.report.max_estimated_engagement_mm =
                    self.report.max_estimated_engagement_mm.max(widest);
                self.report.stay_down_links += 1;
                if waypoints.len() > 1 {
                    self.report.routed_links += 1;
                }
                let mut at = here;
                for w in waypoints {
                    self.report.certified_strokes += 1;
                    let next = [w[0], w[1], z];
                    self.link(stock, at, next, cancel)?;
                    at = next;
                }
            } else if let Some((p, proven_p, samples)) = descent {
                // Plunge beside the entry, where the stock bounds let the
                // rapid come down, then feed across to it at depth.
                let above = self.travel(stock, here, p, proven_p, z);
                self.line(above, [p[0], p[1], z], MoveKind::Plunge);
                let layer = self.layer.as_mut().unwrap();
                for s in samples {
                    layer.cut(s, r);
                }
                self.report.certified_strokes += 1;
                self.link(stock, [p[0], p[1], z], [q[0], q[1], z], cancel)?;
                self.report.lifts += 1;
                self.report.stock_edge_entries += 1;
                self.report.offset_descents += 1;
            } else {
                let above = self.travel(stock, here, q, proven, z);
                self.line(above, [q[0], q[1], z], MoveKind::Plunge);
                self.report.lifts += 1;
                self.report.stock_edge_entries += 1;
            }
            let Some(pts) = self.proven_pass(stock, q, heading, true, base.pending, cancel)? else {
                self.rollback(&base);
                if !self.remnant {
                    self.layer.as_mut().unwrap().thin.push(q);
                }
                self.mark_dead(q, 3);
                continue;
            };
            if std::mem::take(&mut self.remnant) {
                self.report.remnant_passes += 1;
            }
            let end = *pts.last().unwrap();
            self.position = [end[0], end[1], z];
            self.report.passes += 1;
            return Ok(true);
        }
    }

    /// Everything a planned pass changes, to undo it.
    fn checkpoint(&mut self) -> PassMark {
        PassMark {
            pending: self.pending.len(),
            journal: self.layer.as_ref().unwrap().journal.len(),
            report: self.report.clone(),
        }
    }
    fn rollback(&mut self, mark: &PassMark) {
        self.pending.truncate(mark.pending);
        self.layer.as_mut().unwrap().undo(mark.journal);
        let replans = self.report.engagement_replans;
        self.report = mark.report.clone();
        self.report.engagement_replans = replans;
    }

    /// The declared width less what the steering grid cannot resolve: the
    /// fitted strokes may lie `FIT_FRACTION` of a cell from the planned
    /// points, and a cell's material is known to half a cell.
    fn steering_width(&self) -> f64 {
        let c = self.layer.as_ref().unwrap().g.c;
        (self.width - STEER_MARGIN_CELLS * c).max(0.5 * self.width)
    }

    /// Follow the front from `start`, emit its cuts and prove every feed
    /// stroke queued since `from` (the link or entry before them included)
    /// within the engagement limit. A pass with an unproven stroke is
    /// planned again with a narrower steering width; one that cannot be
    /// proven at half the width stops the run. Returns the pass's points,
    /// or `None` when it cuts nothing (nothing of it is then queued).
    fn proven_pass(
        &mut self,
        stock: &Stock,
        start: Point,
        heading: f64,
        approach: bool,
        from: usize,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Vec<Point>>, String> {
        let (mut start, mut heading, mut approach) = (start, heading, approach);
        let mut narrow = 1.;
        let mut kept: Vec<Point> = vec![];
        // Strokes queued before this index are proven already: premises only.
        let mut proven = from;
        loop {
            let mark = self.checkpoint();
            let (pts, cut, more) = self.follow(start, heading, approach, narrow);
            if !cut {
                self.rollback(&mark);
                return Ok((!kept.is_empty()).then_some(kept));
            }
            self.emit_cuts(&pts)?;
            let proved = self.prove(stock, from, proven, false, cancel)?;
            let Some((at, detail)) = proved else {
                let (n, first) = (pts.len(), kept.is_empty());
                let (before, last) = (pts[n - 2], pts[n - 1]);
                kept.extend(pts.into_iter().skip(usize::from(!first)));
                if !more {
                    return Ok(Some(kept));
                }
                proven = self.pending.len();
                // The next stretch of the same pass, a step wider again
                // where an earlier stretch had to be narrowed.
                heading = (last[1] - before[1]).atan2(last[0] - before[0]);
                start = last;
                approach = false;
                narrow = (narrow / RETRY_SCALE).min(1.);
                continue;
            };
            if narrow * RETRY_SCALE < 0.5 {
                // Confined material: no step width keeps a cutter pinned
                // between walls within the limit. It is entry motion, as the
                // helix that opened it is, where the entry policy allows one.
                if self.entry.policy() == EntryPolicy::StockEdge {
                    return Err(format!(
                        "engagement_front: {detail}, even steered at half the declared width: the \
                         material there is confined (a pocket barely wider than the cutter). \
                         StockEdgeEntry(helix=None) forbids cutting it as entry motion; allow a \
                         helical entry, use a smaller tool, or exclude the area"
                    ));
                }
                self.prove(stock, from, proven, true, cancel)?;
                kept.extend(pts.into_iter().skip(usize::from(!kept.is_empty())));
                return Ok(Some(kept));
            }
            // Keep the part of the pass before the unproven stroke (its
            // strokes are proven) and steer narrower from there on.
            narrow *= RETRY_SCALE;
            self.rollback(&mark);
            self.report.engagement_replans += 1;
            let split = pts.iter().position(|p| *p == at).unwrap_or(0);
            if split >= 2 {
                let prefix = &pts[..=split];
                let layer = self.layer.as_mut().unwrap();
                for p in &prefix[1..] {
                    layer.cut(*p, self.r);
                }
                self.emit_cuts(prefix)?;
                let before = prefix[split - 1];
                heading = (at[1] - before[1]).atan2(at[0] - before[0]);
                start = at;
                approach = false;
                kept.extend(prefix.iter().copied().skip(usize::from(!kept.is_empty())));
            }
        }
    }

    /// Prove the contact angle of every cutting and stay-down link stroke
    /// queued since `from` against the committed cleared sweeps and the
    /// strokes queued before it. The proof is exact
    /// (`EngagementScene::check_leading`) and shares nothing with the
    /// steering grid. Returns where the first unproven stroke starts and
    /// what was not proven, or `None`. With
    /// `confined`, an unproven stroke becomes an entry move at the entry
    /// feed (see `EngagementFrontReport::confined_strokes`) and nothing is
    /// returned. Strokes before `unproven_from` were proven by an earlier
    /// call and are premises only.
    fn prove(
        &mut self,
        stock: &Stock,
        from: usize,
        unproven_from: usize,
        confined: bool,
        cancel: &impl CancelSignal,
    ) -> Result<Option<(Point, String)>, String> {
        let tolerance = self.op.tolerance_mm;
        let (mut scene, mut cursor) = match self.proof.take() {
            Some(held) => held,
            None => (
                EngagementScene::new_with_cancel(stock.spec().clone(), vec![], cancel)?,
                ClearanceCursor::default(),
            ),
        };
        let (rebuild, sweeps) = stock.committed_clearance_since(&mut cursor, tolerance, cancel)?;
        // Recorded proofs stand only for the same committed sweeps and the
        // same first queued stroke.
        if rebuild || !sweeps.is_empty() || self.proof_memo.from != from {
            self.proof_memo = ProofMemo {
                from,
                strokes: vec![],
            };
        }
        // Whether the queue has repeated the record so far.
        let mut recorded = true;
        if rebuild {
            scene = EngagementScene::new_with_cancel(stock.spec().clone(), sweeps, cancel)?;
        } else {
            for sweep in sweeps {
                scene.append(sweep)?;
            }
        }
        let mark = scene.mark();
        let limit = self.report.engagement_limit_deg;
        let mut failed = None;
        let (mut proved, mut upper) = (0, 0f64);
        let entry_feed = self.entry.feed_mm_min().unwrap_or(self.op.plunge_mm_min);
        // `halved[i]`: stroke `from + i` continues the straight stroke before
        // it, which the proof halved.
        let mut halved = vec![false; self.pending.len().saturating_sub(from)];
        let mut next = from;
        while next < self.pending.len() {
            let index = next;
            next += 1;
            let (curve, kind) = (self.pending[index].0.clone(), self.pending[index].1);
            // This stroke's recorded proof, if the queue still repeats the
            // record; otherwise the record ends here and is written anew.
            let slot = index - from;
            let known = match self.proof_memo.strokes.get(slot) {
                Some((c, k, result)) if recorded && same_curve(c, &curve) && *k == kind => *result,
                _ => {
                    recorded = false;
                    if !confined {
                        self.proof_memo.strokes.truncate(slot);
                        self.proof_memo.strokes.push((curve.clone(), kind, None));
                    }
                    None
                }
            };
            let flat = curve.start()[2] == curve.end()[2]
                && matches!(
                    curve,
                    Curve::Line { .. }
                        | Curve::Arc {
                            plane: crate::motion::Plane::Xy,
                            ..
                        }
                );
            if !flat || kind.is_rapid() {
                continue;
            }
            let motion = Motion {
                id: 0,
                operation: self.op.id.clone(),
                tool: self.op.tool.clone(),
                feed_mm_min: Some(self.op.feed_mm_min),
                kind,
                curve: curve.clone(),
            };
            if index >= unproven_from && matches!(kind, MoveKind::Cut | MoveKind::Link) {
                let check = || {
                    scene
                        .check_leading(&motion, self.r, limit, EngagementOptions::default(), cancel)
                        .map(|report| (report.status, report.upper_angle_deg))
                };
                let (status, upper_angle_deg) = match known {
                    Some(result) => {
                        #[cfg(test)]
                        {
                            shortcut_taken(0);
                            let again = check()?;
                            assert_eq!(
                                (result.0, result.1.to_bits()),
                                (again.0, again.1.to_bits()),
                                "recorded proof of {curve:?}"
                            );
                        }
                        result
                    }
                    None => {
                        let result = check()?;
                        if !confined {
                            self.proof_memo.strokes[slot].2 = Some(result);
                        }
                        result
                    }
                };
                struct Proof {
                    status: EngagementStatus,
                    upper_angle_deg: f64,
                }
                let report = Proof {
                    status,
                    upper_angle_deg,
                };
                // An unproven straight stroke is halved until the unproven
                // part is no longer than an eighth of the cutter radius, so only the
                // stretch that is not proven becomes entry motion.
                if matches!(report.status, EngagementStatus::Unresolved)
                    && confined
                    && let Curve::Line { start, end } = curve
                    && (end[0] - start[0]).hypot(end[1] - start[1]) > self.r / 8.
                {
                    let middle = [(start[0] + end[0]) / 2., (start[1] + end[1]) / 2., start[2]];
                    let feed = self.pending[index].2;
                    self.pending[index].0 = Curve::Line { start, end: middle };
                    self.pending
                        .insert(index + 1, (Curve::Line { start: middle, end }, kind, feed));
                    halved.insert(index + 1 - from, true);
                    next = index;
                    // The queue no longer repeats the record.
                    recorded = false;
                    continue;
                }
                if matches!(report.status, EngagementStatus::Unresolved) && confined {
                    // The bound an entry move is reported with: what a
                    // half-circle limit proves, or the whole circle.
                    let bound = scene.check_leading(
                        &motion,
                        self.r,
                        180.,
                        EngagementOptions::default(),
                        cancel,
                    )?;
                    self.pending[index].1 = MoveKind::Lead;
                    self.pending[index].2 = Some(entry_feed);
                    self.report.confined_strokes += 1;
                    self.report.confined_stroke_mm += curve.prepare(tolerance)?.length_mm();
                    self.report.confined_max_engagement_deg = self
                        .report
                        .confined_max_engagement_deg
                        .max(match bound.status {
                            EngagementStatus::Bounded => bound.upper_angle_deg,
                            _ => 360.,
                        });
                } else if matches!(report.status, EngagementStatus::Unresolved) {
                    let (a, b) = (curve.start(), curve.end());
                    failed = Some((
                        [a[0], a[1]],
                        format!(
                            "the contact angle of the stroke from ({:.3}, {:.3}) to ({:.3}, {:.3}) at Z{:.3} \
                         is not proven within {limit:.2} degrees (max_radial_engagement_mm {})",
                            a[0], a[1], b[0], b[1], a[2], self.width
                        ),
                    ));
                    break;
                }
                if matches!(report.status, EngagementStatus::Bounded) {
                    proved += 1;
                    upper = upper.max(report.upper_angle_deg);
                }
            }
            scene.append(ClearedSweep::from_motion(&motion, self.r, tolerance)?)?;
        }
        // Halves that ended up the same kind of move are one block again.
        for index in (from + 1..self.pending.len()).rev() {
            let (before, after) = (&self.pending[index - 1], &self.pending[index]);
            if halved[index - from] && before.1 == after.1 && before.2 == after.2 {
                let end = after.0.end();
                if let Curve::Line { start, .. } = before.0 {
                    self.pending[index - 1].0 = Curve::Line { start, end };
                    self.pending.remove(index);
                    if self.pending[index - 1].1 == MoveKind::Lead {
                        self.report.confined_strokes -= 1;
                    }
                }
            }
        }
        if confined {
            // Confined strokes were changed in the queue; nothing recorded
            // for this pass is used again.
            self.proof_memo.strokes.clear();
        }
        // Planned strokes are premises only until the stock commits them.
        scene.restore(mark);
        self.proof = Some((scene, cursor));
        if failed.is_none() {
            self.report.engagement_proved_strokes += proved;
            if upper > self.report.proven_max_engagement_deg {
                self.report.proven_max_engagement_deg = upper;
                self.report.proven_max_engagement_mm = self.r * (1. - upper.to_radians().cos());
            }
        }
        Ok(failed)
    }

    /// One pass along every wall loop of the region (`with_wall_pass`).
    ///
    /// The loops are the pocket guides' own boundary strokes: the exact
    /// offset of the region by the cutter radius and the radial allowance.
    /// Each is entered on a quarter-arc lead-in from a few grid cells
    /// inside it, reached as a pass's start is (`wall_entry`). Every stroke
    /// is certified against the region and proven within the engagement
    /// limit like any other; in an inside corner, where the skin is as
    /// thick as the corner is deep, an unproven stroke is entry motion (see
    /// `confined_strokes`).
    fn plan_walls(&mut self, stock: &Stock, cancel: &impl CancelSignal) -> Result<(), String> {
        for wall in self.wall_loops() {
            if cancel.is_cancelled() {
                return Err("engagement_front planning cancelled".into());
            }
            self.wall_loop(&wall, stock, cancel)?;
        }
        Ok(())
    }

    /// The closed loops of the guides' boundary strokes: the exact offset
    /// of the region by the cutter radius and the radial allowance.
    fn wall_loops(&self) -> Vec<Vec<Stroke>> {
        let first = self.guides.report().raster_strokes;
        let mut loops: Vec<Vec<Stroke>> = vec![];
        for &s in &self.guides.guide_strokes()[first..] {
            match loops.last_mut() {
                Some(l) if l.last().is_some_and(|p| p.b == s.a) => l.push(s),
                _ => loops.push(vec![s]),
            }
        }
        loops.retain(|wall| wall.first().map(|s| s.a) == wall.last().map(|s| s.b));
        loops
    }

    /// One pass round a whole wall loop (see `plan_walls`).
    fn wall_loop(
        &mut self,
        wall: &[Stroke],
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        let z = self.z;
        {
            // A start: the middle of a wall stroke (longest first: a corner's
            // start would sit against the next wall's skin), with a
            // proven-clear point a few cells inside it.
            let middle = |s: Stroke| -> Point {
                match s.arc {
                    Some((centre, sweep)) => {
                        let radius = (s.a[0] - centre[0]).hypot(s.a[1] - centre[1]);
                        let angle = (s.a[1] - centre[1]).atan2(s.a[0] - centre[0]) + sweep / 2.;
                        [
                            centre[0] + radius * angle.cos(),
                            centre[1] + radius * angle.sin(),
                        ]
                    }
                    None => [(s.a[0] + s.b[0]) / 2., (s.a[1] + s.b[1]) / 2.],
                }
            };
            let halves = |s: Stroke| -> (Stroke, Stroke) {
                let m = middle(s);
                let arc = s.arc.map(|(centre, sweep)| (centre, sweep / 2.));
                (Stroke { a: s.a, b: m, arc }, Stroke { a: m, b: s.b, arc })
            };
            let span = |s: &Stroke| match s.arc {
                Some((centre, sweep)) => {
                    (s.a[0] - centre[0]).hypot(s.a[1] - centre[1]) * sweep.abs()
                }
                None => (s.b[0] - s.a[0]).hypot(s.b[1] - s.a[1]),
            };
            let mut order: Vec<usize> = (0..wall.len()).collect();
            order.sort_by(|a, b| span(&wall[*b]).total_cmp(&span(&wall[*a])).then(a.cmp(b)));
            self.layer.as_mut().unwrap().journal.clear();
            let mark = self.checkpoint();
            let from = self.pending.len();
            let mut entry = None;
            for k in order {
                let (first, second) = halves(wall[k]);
                if self.wall_entry(second, stock, cancel)? {
                    entry = Some((k, first, second));
                    break;
                }
            }
            let Some((k, first, second)) = entry else {
                self.report.wall_passes_skipped += 1;
                return Ok(());
            };
            let start = second.a;
            let round = std::iter::once(second)
                .chain((1..wall.len()).map(|i| wall[(k + i) % wall.len()]))
                .chain(std::iter::once(first));
            for s in round {
                self.certify(s)?;
                if s.arc.is_some() {
                    self.report.arcs += 1;
                }
                self.queue(s.curve(z), MoveKind::Cut, None);
            }
            if self.prove(stock, from, from, false, cancel)?.is_some() {
                if self.entry.policy() == EntryPolicy::StockEdge {
                    self.rollback(&mark);
                    self.report.wall_passes_skipped += 1;
                    return Ok(());
                }
                self.prove(stock, from, from, true, cancel)?;
            }
            self.position = [start[0], start[1], z];
            self.report.wall_passes += 1;
        }
        Ok(())
    }

    /// Queue the way onto a wall pass that starts with `onto`: to the start
    /// of a quarter-arc lead-in tangent to it, as a contour pass is entered
    /// (the skin comes into the cut gradually and stays ahead of the
    /// cutter), then the arc. Whether a way was found; nothing is queued
    /// otherwise.
    ///
    /// The arc starts a few grid cells off the wall, at a centre the front
    /// itself may stand on, so it is reached as a pass's start is
    /// (`wall_approach`).
    fn wall_entry(
        &mut self,
        onto: Stroke,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        let lead = 4. * self.layer.as_ref().unwrap().g.c;
        let t = onto.tangent(false);
        for side in [1., -1.] {
            let centre = [
                onto.a[0] - side * lead * t[1],
                onto.a[1] + side * lead * t[0],
            ];
            let q = [centre[0] - lead * t[0], centre[1] - lead * t[1]];
            let arc = Stroke {
                a: q,
                b: onto.a,
                arc: Some((centre, side * std::f64::consts::FRAC_PI_2)),
            };
            if !self.guides.stroke_geometry_fits_radius(arc, self.r) {
                continue;
            }
            // From the arc's centre to the wall.
            let toward = [side * t[1], -side * t[0]];
            if self.wall_approach(stock, q, toward, cancel)? {
                self.certify(arc)?;
                self.report.arcs += 1;
                self.queue(arc.curve(self.z), MoveKind::Cut, None);
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// Queue the way from where the cutter is to `q` at this level, as a
    /// pass's start is reached (`plan_pass`): a stay-down link through
    /// cleared stock when the cutter is at the level and one is found;
    /// otherwise a vertical entry, if the committed sweeps prove the whole
    /// column at `q` clear, coming down a few stock cells back from the
    /// wall (opposite `toward`) where the rapid can come lower
    /// (`offset_descent`). Whether a way was found.
    fn wall_approach(
        &mut self,
        stock: &Stock,
        q: Point,
        toward: Point,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        let (z, r) = (self.z, self.r);
        let here = self.position;
        let from = [here[0], here[1]];
        let keep_down = 6. * r;
        let link = if (here[2] - z).abs() >= 1e-9 {
            None
        } else if (q[0] - from[0]).hypot(q[1] - from[1]) <= keep_down
            && let Some((samples, widest)) = self.link_samples(from, q)
            && self
                .guides
                .stroke_geometry_fits_radius(Stroke::line(from, q), r)
        {
            Some((vec![q], samples, widest))
        } else {
            self.routed_link(from, q, 3. * keep_down)
        };
        if let Some((waypoints, samples, widest)) = link {
            let layer = self.layer.as_mut().unwrap();
            for p in samples {
                layer.cut(p, r);
            }
            self.report.max_estimated_engagement_mm =
                self.report.max_estimated_engagement_mm.max(widest);
            self.report.stay_down_links += 1;
            if waypoints.len() > 1 {
                self.report.routed_links += 1;
            }
            let mut at = here;
            for w in waypoints {
                self.report.certified_strokes += 1;
                let next = [w[0], w[1], z];
                self.link(stock, at, next, cancel)?;
                at = next;
            }
            return Ok(true);
        }
        let (height, _) = self
            .guides
            .cleared_entry_start(q, 0., z, Some(stock), cancel)?;
        if height - z > crate::entry::skin(self.op.tolerance_mm) {
            return Ok(false);
        }
        let proven = self
            .proven(height)
            .map(|h| h.max(self.guides.cell_entry_height(stock, q)));
        if let Some((p, proven_p, samples)) =
            self.offset_descent(stock, q, toward, proven, cancel)?
        {
            let above = self.travel(stock, here, p, proven_p, z);
            self.line(above, [p[0], p[1], z], MoveKind::Plunge);
            let layer = self.layer.as_mut().unwrap();
            for s in samples {
                layer.cut(s, r);
            }
            self.report.certified_strokes += 1;
            self.link(stock, [p[0], p[1], z], [q[0], q[1], z], cancel)?;
            self.report.offset_descents += 1;
        } else {
            let above = self.travel(stock, here, q, proven, z);
            self.line(above, [q[0], q[1], z], MoveKind::Plunge);
        }
        self.report.lifts += 1;
        Ok(true)
    }

    /// The committed cleared sweeps as the scene the proofs are made in,
    /// brought up to the stock's last commit (as `prove` brings it).
    fn committed_scene(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<(EngagementScene, ClearanceCursor), String> {
        let (mut scene, mut cursor) = match self.proof.take() {
            Some(held) => held,
            None => (
                EngagementScene::new_with_cancel(stock.spec().clone(), vec![], cancel)?,
                ClearanceCursor::default(),
            ),
        };
        let (rebuild, sweeps) =
            stock.committed_clearance_since(&mut cursor, self.op.tolerance_mm, cancel)?;
        if rebuild || !sweeps.is_empty() {
            // Recorded proofs stood for the sweeps before these.
            self.proof_memo = ProofMemo::default();
        }
        if rebuild {
            scene = EngagementScene::new_with_cancel(stock.spec().clone(), sweeps, cancel)?;
        } else {
            for sweep in sweeps {
                scene.append(sweep)?;
            }
        }
        Ok((scene, cursor))
    }

    /// Cut along the walls where material stands beside them
    /// (`with_wall_runs`), after the level's passes are committed.
    ///
    /// The wall loops are those of `plan_walls`. Each is taken in pieces no
    /// longer than the cutter is wide, and a piece the committed sweeps
    /// prove clear cuts nothing (`wall_piece_is_clear`). The other pieces
    /// are joined into runs. Two runs are joined across the air between
    /// them when feeding through it takes no longer than the feed-rate
    /// descent of a new entry would (feed x (feed height - Z) / plunge
    /// feed); a loop with no longer gap than that is cut whole, as a
    /// pocket's is.
    ///
    /// A run is entered like a whole loop, on a quarter-arc lead-in reached
    /// as a pass's start is (`wall_entry`); where no arc fits or is
    /// reached it starts on the loop, in the air before the run, at a point
    /// whose column the committed cuts prove clear. Runs are taken nearest
    /// first. Every stroke is certified and proven like any other (an
    /// unproven one is entry motion where the entry policy allows, as in
    /// `plan_walls`).
    fn plan_wall_runs(&mut self, stock: &Stock, cancel: &impl CancelSignal) -> Result<(), String> {
        enum Wall {
            Whole(Vec<Stroke>),
            Run {
                before: Stroke,
                strokes: Vec<Stroke>,
            },
        }
        let z = self.z;
        let longest = 2. * self.r;
        let bridge = self.op.feed_mm_min * (self.heights.feed - z).max(0.) / self.op.plunge_mm_min;
        let loops = self.wall_loops();
        if loops.is_empty() {
            return Ok(());
        }
        let (scene, cursor) = self.committed_scene(stock, cancel)?;
        let mut walls: Vec<Wall> = vec![];
        let mut failed = None;
        'loops: for wall in loops {
            let pieces: Vec<Stroke> = wall
                .iter()
                .flat_map(|&s| {
                    let n = ((stroke_length(s) / longest).ceil() as usize).max(1);
                    (0..n).map(move |i| {
                        stroke_part(s, i as f64 / n as f64, (i + 1) as f64 / n as f64)
                    })
                })
                .filter(|s| stroke_length(*s) > 0.)
                .collect();
            let mut cuts = Vec::with_capacity(pieces.len());
            for s in &pieces {
                if cancel.is_cancelled() {
                    failed = Some("engagement_front planning cancelled".to_string());
                    break 'loops;
                }
                match wall_piece_is_clear(&scene, *s, z, self.r, self.op.tolerance_mm, cancel) {
                    Ok(clear) => cuts.push(!clear),
                    Err(e) => {
                        failed = Some(e);
                        break 'loops;
                    }
                }
            }
            let n = pieces.len();
            if !cuts.contains(&true) {
                continue;
            }
            // Air gaps: (first piece, pieces, length), round the loop.
            let mut gaps: Vec<(usize, usize, f64)> = vec![];
            for i in 0..n {
                if !cuts[i] && cuts[(i + n - 1) % n] {
                    let mut count = 0;
                    let mut length = 0.;
                    while !cuts[(i + count) % n] {
                        length += stroke_length(pieces[(i + count) % n]);
                        count += 1;
                    }
                    gaps.push((i, count, length));
                }
            }
            gaps.retain(|gap| gap.2 > bridge);
            if gaps.is_empty() {
                walls.push(Wall::Whole(wall));
                continue;
            }
            for (g, &(first, count, _)) in gaps.iter().enumerate() {
                // The run after this gap ends where the next gap starts.
                let start = (first + count) % n;
                let end = gaps[(g + 1) % gaps.len()].0;
                let length = (end + n - start) % n;
                walls.push(Wall::Run {
                    before: pieces[(start + n - 1) % n],
                    strokes: (0..length).map(|i| pieces[(start + i) % n]).collect(),
                });
            }
        }
        self.proof = Some((scene, cursor));
        if let Some(e) = failed {
            return Err(e);
        }
        while !walls.is_empty() {
            if cancel.is_cancelled() {
                return Err("engagement_front planning cancelled".into());
            }
            let here = self.position;
            let distance = |w: &Wall| {
                let p = match w {
                    Wall::Whole(strokes) => strokes[0].a,
                    Wall::Run { strokes, .. } => strokes[0].a,
                };
                (p[0] - here[0]).hypot(p[1] - here[1])
            };
            let mut nearest = 0;
            for i in 1..walls.len() {
                if distance(&walls[i]) < distance(&walls[nearest]) {
                    nearest = i;
                }
            }
            let (before, strokes) = match walls.remove(nearest) {
                Wall::Whole(wall) => {
                    self.wall_loop(&wall, stock, cancel)?;
                    continue;
                }
                Wall::Run { before, strokes } => (before, strokes),
            };
            self.layer.as_mut().unwrap().journal.clear();
            let mark = self.checkpoint();
            let from = self.pending.len();
            let mut entered = self.wall_entry(strokes[0], stock, cancel)?;
            if !entered {
                // No lead-in arc fits or is reached: start on the loop, in
                // the air before the run, at the nearest point whose column
                // is proven clear.
                for back in [0.25, 0.5, 0.75, 1.] {
                    let lead = stroke_part(before, 1. - back, 1.);
                    let q = lead.a;
                    let (height, _) =
                        self.guides
                            .cleared_entry_start(q, 0., z, Some(stock), cancel)?;
                    if height - z > crate::entry::skin(self.op.tolerance_mm) {
                        continue;
                    }
                    let proven = self
                        .proven(height)
                        .map(|h| h.max(self.guides.cell_entry_height(stock, q)));
                    let above = self.travel(stock, here, q, proven, z);
                    self.line(above, [q[0], q[1], z], MoveKind::Plunge);
                    self.report.lifts += 1;
                    self.certify(lead)?;
                    if lead.arc.is_some() {
                        self.report.arcs += 1;
                    }
                    self.queue(lead.curve(z), MoveKind::Cut, None);
                    entered = true;
                    break;
                }
            }
            if !entered {
                self.report.wall_passes_skipped += 1;
                continue;
            }
            for &s in &strokes {
                self.certify(s)?;
                if s.arc.is_some() {
                    self.report.arcs += 1;
                }
                self.queue(s.curve(z), MoveKind::Cut, None);
            }
            if self.prove(stock, from, from, false, cancel)?.is_some() {
                if self.entry.policy() == EntryPolicy::StockEdge {
                    self.rollback(&mark);
                    self.report.wall_passes_skipped += 1;
                    continue;
                }
                self.prove(stock, from, from, true, cancel)?;
            }
            let end = strokes.last().unwrap().b;
            self.position = [end[0], end[1], z];
            self.report.wall_passes += 1;
        }
        Ok(())
    }

    /// Before a helix bores onto the material `helix_pass` would pick: if
    /// passes started beside it found it too thin to cut, it is a remnant
    /// reachable at depth, not enclosed. Offer those starts again (once
    /// each) as remnant passes. Returns whether any were offered.
    fn revive_thin_starts(&mut self) -> bool {
        let c = self.layer.as_ref().unwrap().g.c;
        let need = self.grid_clearance(c);
        let reach = self.r + 4. * c;
        let layer = self.layer.as_mut().unwrap();
        if layer.thin.is_empty() {
            return false;
        }
        let best = layer.most_open(need);
        let Some((_, k)) = best else {
            return false;
        };
        let q = layer.g.center(k);
        let g = layer.g;
        let mut revived = false;
        let thin = std::mem::take(&mut layer.thin);
        for t in thin {
            if (t[0] - q[0]).hypot(t[1] - q[1]) > reach {
                layer.thin.push(t);
                continue;
            }
            let Some((i, j)) = g.cell(t) else { continue };
            for jj in j.saturating_sub(3)..=(j + 3).min(g.ny - 1) {
                for ii in i.saturating_sub(3)..=(i + 3).min(g.nx - 1) {
                    layer.dead[g.idx(ii, jj)] = false;
                    layer.reopened(g.idx(ii, jj));
                }
            }
            layer.candidates.push(g.idx(i, j));
            revived = true;
        }
        revived
    }

    /// The longest straight path through `q`, at most `want` long, that
    /// keeps the cutter inside the region: its two ends. Directions are
    /// tried a degree apart on the steering grid (the first of the longest
    /// wins), and the path is then certified exactly against the region,
    /// shortened a little at both ends if the grid was optimistic. None:
    /// no straight path through `q` at all.
    fn ramp_line(&self, q: Point, want: f64) -> Option<(Point, Point)> {
        let layer = self.layer.as_ref().unwrap();
        let c = layer.g.c;
        if !layer.allowed_at(q) || !(want.is_finite() && want > 0.) {
            return None;
        }
        // Free travel from `q` along `u`, by the grid's allowed centres.
        let reach = |u: Point| {
            let mut d = 0.;
            while d < want {
                let next = (d + c).min(want);
                if !layer.allowed_at([q[0] + next * u[0], q[1] + next * u[1]]) {
                    break;
                }
                d = next;
            }
            d
        };
        const DIRECTIONS: usize = 180;
        let mut found: Vec<(f64, f64, f64, Point)> = (0..DIRECTIONS)
            .map(|i| {
                let angle = PI * i as f64 / DIRECTIONS as f64;
                let u = [angle.cos(), angle.sin()];
                let (ahead, behind) = (reach(u), reach([-u[0], -u[1]]));
                ((ahead + behind).min(want), ahead, behind, u)
            })
            .collect();
        // Longest first; equal lengths keep their order (stable sort).
        found.sort_by(|a, b| b.0.total_cmp(&a.0));
        for (length, ahead, behind, u) in found.into_iter().take(8) {
            // `length` of the free travel, as evenly about `q` as it allows.
            let back = behind.min((length / 2.).max(length - ahead));
            let fore = length - back;
            for shrink in 0..8 {
                let trim = 2. * c * shrink as f64;
                let (back, fore) = ((back - trim).max(0.), (fore - trim).max(0.));
                if back + fore <= 4. * c {
                    break;
                }
                let a = [q[0] - back * u[0], q[1] - back * u[1]];
                let b = [q[0] + fore * u[0], q[1] + fore * u[1]];
                if self
                    .guides
                    .stroke_geometry_fits_radius(Stroke::line(a, b), self.r)
                {
                    return Some((a, b));
                }
            }
        }
        None
    }

    /// Enclosed material with a `RampEntry` (#219): ramp down along a
    /// straight path through the most open reachable point, then cut that
    /// path once flat at depth and follow the front from the slot it leaves.
    ///
    /// Established practice: adaptive clearing enters enclosed material by
    /// a helix or, where the tool or the pocket asks for it, by a zig-zag
    /// ramp (Autodesk Fusion, 2D Adaptive Clearing reference,
    /// help.autodesk.com/cloudhelp/ENU/Fusion-CAM/files/
    /// GUIDA73542E9-ED9C-4BD9-A87D-3A0ECA8BEB41.htm: "Ramp Type: Specifies
    /// how the cutter moves down for each depth cut", with "Zig-Zag" beside
    /// "Helix", and "Maximum Ramp Stepdown ... allows the tool load to be
    /// constrained when doing full-width cuts during ramping"). A linear
    /// ramp is a full-width slot (Sandvik Coromant, "Ramping: two axis
    /// linear and circular": "Circular ramping is always preferred to
    /// linear ramping (full slotting)"), so it is never chosen where an
    /// entry from air or cleared stock exists, and it is entry motion like
    /// the helix: at the entry feed, not bounded by the engagement limit,
    /// and reported (`ramp_entries`, `ramp_slot_mm`).
    ///
    /// What is proven:
    /// - the path keeps the cutter inside the region (`certify`, exact);
    /// - the ramp is `RampEntry::zigzag`, never steeper than the declared
    ///   angle, one leg where the path is long enough for it and otherwise
    ///   back and forth over the whole path (which needs `min_length_mm`);
    /// - it starts at the lowest height at which the committed stock proves
    ///   the whole tool clear along the whole path (the committed sweeps or
    ///   the whole-cell bounds), so the straight descent to it is through
    ///   air and the ramp starts no higher than the highest material under
    ///   it; a path proven air down to the level needs no ramp at all
    ///   (`air_entries`);
    /// - the path is no longer than one leg needs, so the slot is as short
    ///   as the angle allows;
    /// - the tool's sections above the flutes are clear of stock along the
    ///   path at depth (`clear_above_flutes`: `Stock::history_clearance`);
    /// - every cutting stroke that follows is proven within the engagement
    ///   limit with the flat pass as its premise (`proven_pass`).
    ///
    /// A place where no ramp fits is refused by name, not left uncut.
    ///
    /// In a pocket barely wider than the cutter the pass after the ramp
    /// cannot always take everything, and what it leaves lies beside the
    /// slot. That is not ramped into again, through the air the slot left:
    /// `slot_side_entry` cuts it from the slot at depth.
    fn ramp_pass(
        &mut self,
        ramp: &crate::entry::RampEntry,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        let (r, z) = (self.r, self.z);
        if cancel.is_cancelled() {
            return Err("engagement_front planning cancelled".into());
        }
        let layer = self.layer.as_ref().unwrap();
        let c = layer.g.c;
        let need = self.grid_clearance(c);
        let best = layer.most_open(need);
        let Some((_, k)) = best else {
            return Ok(false);
        };
        let q = layer.g.center(k);
        let tolerance = self.op.tolerance_mm;
        let skin = crate::entry::skin(tolerance);
        if self.slot_side_entry(q, stock, cancel)? {
            return Ok(true);
        }
        let refuse = |why: String| {
            format!(
                "engagement_front: no ramp entry into the enclosed material at ({:.3}, {:.3}) for the level at Z{z}: {why}. Use a helical entry (StockEdgeEntry with a helix, or HelicalEntry on an adaptive pocket), a smaller cutter, or exclude the area",
                q[0], q[1]
            )
        };
        // The ramp starts no higher than this, whatever the stock holds.
        let begin_at = |top: f64| {
            (top + self.ramp_clearance.unwrap_or(0.))
                .min(self.heights.feed)
                .max(top)
        };
        let tan = ramp.angle_rad.tan();
        let highest = begin_at(self.op.top_z_mm.max(z));
        let (mut a, mut b) = self
            .ramp_line(q, (highest - z).max(skin) / tan)
            .ok_or_else(|| refuse("the cutter has no straight path there".into()))?;
        // The material top along the path, and the path no longer than one
        // leg from there needs: a shorter path has a lower top or the same.
        let mut top;
        let mut round = 0;
        loop {
            let path = [(a, b)];
            top = self
                .guides
                .cleared_path_start(&[Stroke::line(a, b)], z, stock, cancel)?
                .min(self.guides.cell_path_height(stock, &path, 0.))
                .max(z);
            let length = (b[0] - a[0]).hypot(b[1] - a[1]);
            let needed = (begin_at(top) - z) / tan;
            if top - z <= skin || needed >= length {
                break;
            }
            // Keep `needed` of it, as evenly about `q` as it allows.
            let u = [(b[0] - a[0]) / length, (b[1] - a[1]) / length];
            let (behind, ahead) = (
                (q[0] - a[0]).hypot(q[1] - a[1]),
                (b[0] - q[0]).hypot(b[1] - q[1]),
            );
            let back = behind.min((needed / 2.).max(needed - ahead));
            let fore = needed - back;
            a = [q[0] - back * u[0], q[1] - back * u[1]];
            b = [q[0] + fore * u[0], q[1] + fore * u[1]];
            round += 1;
            if round == 4 {
                // Shortened to one leg from a top measured on a longer path
                // through the same point, which is no lower than this
                // path's own: the ramp starts no lower than it must.
                break;
            }
        }
        let air = top - z <= skin;
        let begin = begin_at(top);
        let chain = [Stroke::line(b, a)];
        let total = (b[0] - a[0]).hypot(b[1] - a[1]);
        // The ramp ends at `b` at depth; the flat pass then runs to `a`.
        let (curves, legs) = if air {
            (vec![], 0)
        } else {
            let (legs, _) = ramp.legs(total, begin - z).map_err(refuse)?;
            (ramp.zigzag(&chain, begin, z).map_err(refuse)?, legs)
        };
        self.certify(chain[0])?;
        // Below the start the flutes cut; the sections above them must be
        // clear of what stands beside the path (the levels above cut it).
        if !air && !self.guides.clear_above_flutes(a, b, z, stock, cancel)? {
            return Err(refuse(
                "the tool above its flutes (neck, shank or holder) is not proven clear of the stock along the ramp's path".into(),
            ));
        }
        let start = curves.first().map_or([a[0], a[1], z], Curve::start);
        let here = self.position;
        // The flat pass is a premise for the cuts that follow it.
        let from = self.pending.len();
        self.layer.as_mut().unwrap().journal.clear();
        let proven = self
            .proven(top)
            .map(|h| h.max(self.guides.cell_entry_height(stock, [start[0], start[1]])));
        let above = self.travel(stock, here, [start[0], start[1]], proven, start[2]);
        if above[2] > start[2] {
            self.line(above, start, MoveKind::Plunge);
        } else if above[2] < start[2] {
            return Err(format!(
                "engagement_front ramp start Z{:.4} is above the retract height \
                 Z{:.4}; raise retract_z_mm",
                start[2], above[2]
            ));
        }
        if air {
            self.report.air_entries += 1;
        } else {
            let feed = Some(ramp.feed_mm_min.unwrap_or(self.op.plunge_mm_min));
            let mut travel = 0.;
            for curve in curves {
                travel += curve.prepare(tolerance)?.length_mm();
                self.queue(curve, MoveKind::Lead, feed);
            }
            let drop = begin - z;
            let angle = (drop / (travel * travel - drop * drop).max(0.).sqrt())
                .atan()
                .to_degrees();
            self.queue(
                Curve::Line {
                    start: [b[0], b[1], z],
                    end: [a[0], a[1], z],
                },
                MoveKind::Lead,
                feed,
            );
            self.report.ramp_entries += 1;
            self.report.ramp_legs += legs;
            self.report.ramp_slot_mm += total;
            self.report.max_ramp_angle_deg = self.report.max_ramp_angle_deg.max(angle);
        }
        self.ramp_slots.push((a, b));
        // The slot on the steering grid.
        let steps = (total / c).ceil().max(1.) as usize;
        let layer = self.layer.as_mut().unwrap();
        for i in 0..=steps {
            let t = i as f64 / steps as f64;
            layer.cut([b[0] + (a[0] - b[0]) * t, b[1] + (a[1] - b[1]) * t], r);
        }
        self.report.lifts += 1;
        self.position = [a[0], a[1], z];
        // Follow the front from the end of the slot: back along it, turning
        // into one of its walls (the far side of the first step is the slot
        // itself; 0.6 pi is `follow`'s widest turn).
        let side = if self.climb { 1. } else { -1. };
        let heading = (b[1] - a[1]).atan2(b[0] - a[0]) - side * 0.6 * PI;
        if let Some(pts) = self.proven_pass(stock, a, heading, false, from, cancel)? {
            let end = *pts.last().unwrap();
            self.position = [end[0], end[1], z];
        }
        self.report.passes += 1;
        Ok(true)
    }

    /// Material at `q` beside a slot one of this level's ramps cut: come
    /// down in the slot and cut straight across to it at depth, then follow
    /// the front from there. Returns whether that was planned.
    ///
    /// What is proven: the committed cuts still show the whole tool clear
    /// along the whole slot at depth (`cleared_path_start`), so the descent
    /// at its nearest point to `q` is through air; the stroke across keeps
    /// the cutter inside the region (`certify`) and is no longer than the
    /// cutter radius and four grid cells, so it cuts only what lies beside
    /// the slot; its contact angle is proven within the engagement limit
    /// like any cutting stroke, and where that fails it is entry motion at
    /// the entry feed, counted as a confined stroke.
    fn slot_side_entry(
        &mut self,
        q: Point,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        let (r, z) = (self.r, self.z);
        let c = self.layer.as_ref().unwrap().g.c;
        let skin = crate::entry::skin(self.op.tolerance_mm);
        for index in 0..self.ramp_slots.len() {
            let (a, b) = self.ramp_slots[index];
            let d = [b[0] - a[0], b[1] - a[1]];
            let l2 = d[0] * d[0] + d[1] * d[1];
            let t = if l2 > 0. {
                (((q[0] - a[0]) * d[0] + (q[1] - a[1]) * d[1]) / l2).clamp(0., 1.)
            } else {
                0.
            };
            let p = [a[0] + t * d[0], a[1] + t * d[1]];
            let across = (q[0] - p[0]).hypot(q[1] - p[1]);
            if across <= 0.
                || across > r + 4. * c
                || !self
                    .guides
                    .stroke_geometry_fits_radius(Stroke::line(p, q), r)
            {
                continue;
            }
            let height = self
                .guides
                .cleared_path_start(&[Stroke::line(a, b)], z, stock, cancel)?;
            if height - z > skin {
                continue;
            }
            let here = self.position;
            let from = self.pending.len();
            self.layer.as_mut().unwrap().journal.clear();
            let proven = self
                .proven(height)
                .map(|h| h.max(self.guides.cell_entry_height(stock, p)));
            let above = self.travel(stock, here, p, proven, z);
            self.line(above, [p[0], p[1], z], MoveKind::Plunge);
            self.certify(Stroke::line(p, q))?;
            self.queue(
                Curve::Line {
                    start: [p[0], p[1], z],
                    end: [q[0], q[1], z],
                },
                MoveKind::Cut,
                None,
            );
            let steps = (across / c).ceil().max(1.) as usize;
            let layer = self.layer.as_mut().unwrap();
            for i in 0..=steps {
                let f = i as f64 / steps as f64;
                layer.cut([p[0] + (q[0] - p[0]) * f, p[1] + (q[1] - p[1]) * f], r);
            }
            if self.prove(stock, from, from, false, cancel)?.is_some() {
                self.prove(stock, from, from, true, cancel)?;
            }
            self.report.lifts += 1;
            self.report.stock_edge_entries += 1;
            self.report.slot_side_entries += 1;
            self.position = [q[0], q[1], z];
            let heading = (q[1] - p[1]).atan2(q[0] - p[0]);
            if let Some(pts) = self.proven_pass(stock, q, heading, false, from, cancel)? {
                let end = *pts.last().unwrap();
                self.position = [end[0], end[1], z];
            }
            self.report.passes += 1;
            return Ok(true);
        }
        Ok(false)
    }

    /// A pocket operation is asked to clear its whole region: an area of
    /// it no entry reaches refuses the level when it starts, before any of
    /// its motions (`crate::entry::UnenteredArea` states the rule). Without
    /// this (a roughing layer) such areas are listed in the report.
    pub(crate) fn with_unentered_refused(mut self, on: bool) -> Self {
        self.refuse_unentered = on;
        self
    }
    /// Whether the cutter fits anywhere in the level's region (the exact
    /// cutter-centre region of the guide plan).
    pub(crate) fn cutter_fits(&self) -> bool {
        self.guides.report().center_components > 0
    }
    /// For an operation that lists unentered areas: why this level, having
    /// made no motion, left material it was asked to cut. The caller
    /// refuses the operation only if none of its levels makes a motion.
    pub(crate) fn refusal(&self) -> Option<String> {
        if self.report.motions > 0 {
            return None;
        }
        Some(self.report.unentered_areas.first()?.message(&self.op.id))
    }
    /// At the start of the level: list the areas no entry reaches, and
    /// refuse with the first where the operation asks for that.
    fn note_unentered(&mut self) -> Result<(), String> {
        self.report.unentered_areas = self.unentered_areas();
        match self.report.unentered_areas.first() {
            Some(area) if self.refuse_unentered => Err(area.message(&self.op.id)),
            _ => Ok(()),
        }
    }

    /// The areas of this level with material that no entry reaches (#250),
    /// from the layer as it stands (called when the level starts).
    ///
    /// Enclosed material no helix fits in. An area is a connected set of
    /// cutter positions (allowed centres). It has an entry from air or
    /// cleared stock where one of them has the cutter's whole disk clear of
    /// material (`refresh` finds its starts by the same test), and
    /// otherwise a helix at its most open position on material, by
    /// `helix_pass`'s own fit. With neither, nothing brings the cutter
    /// down there. (A `RampEntry` is planned and refused by `ramp_pass`.)
    ///
    /// Material where the planner has no position at all: see
    /// `unresolved_areas`.
    ///
    /// Established practice: adaptive clearing enters a pocket by a helix
    /// no smaller than the tool allows, by a ramp, or through an opening
    /// made first (Autodesk Fusion, 2D Adaptive Clearing reference,
    /// help.autodesk.com/cloudhelp/ENU/Fusion-CAM/files/
    /// GUIDA73542E9-ED9C-4BD9-A87D-3A0ECA8BEB41.htm: "Ramp Type: Specifies
    /// how the cutter moves down for each depth cut"; "Minimum Ramp
    /// Diameter: The smallest Helix Ramp Diameter that is acceptable. ...
    /// Smaller diameters can reduce the chip evacuation, create jerking
    /// machine motion and can cause tool breakage"; "Predrill Positions:
    /// Select points where holes have been drilled to provide clearance
    /// for the cutter to enter the material"). Where none of them is
    /// possible that area cannot be cut as asked, and leaving it without a
    /// word would be a silent fallback.
    fn unentered_areas(&self) -> Vec<crate::entry::UnenteredArea> {
        let mut found = self.unresolved_areas();
        let Some(helix) = self.entry.helix() else {
            return found;
        };
        let layer = self.layer.as_ref().unwrap();
        let g = layer.g;
        let n = g.nx * g.ny;
        let need = self.grid_clearance(g.c);
        let material: Vec<bool> = (0..n).map(|k| layer.material(k)).collect();
        let clear = edt(&g, &material);
        let radii = helix.radii(self.r, self.r / 2., 0.);
        let mut seen = vec![false; n];
        let mut open = vec![];
        for start in 0..n {
            if !layer.allowed[start] || seen[start] {
                continue;
            }
            // One area: whether it has a way in, its most open position on
            // material, and its extent.
            let mut way_in = false;
            let mut best: Option<(f64, usize)> = None;
            let (mut low, mut high) = (g.center(start), g.center(start));
            seen[start] = true;
            open.push(start);
            while let Some(k) = open.pop() {
                let p = g.center(k);
                low = [low[0].min(p[0]), low[1].min(p[1])];
                high = [high[0].max(p[0]), high[1].max(p[1])];
                way_in |= clear[k] > self.r + g.c || layer.cleared[k] <= layer.z;
                if material[k] {
                    let room = layer.room()[k] as f64 - need;
                    if best.is_none_or(|(b, _)| room > b) {
                        best = Some((room, k));
                    }
                }
                g.for_neighbours(k, |m| {
                    if layer.allowed[m] && !seen[m] {
                        seen[m] = true;
                        open.push(m);
                    }
                });
            }
            let Some((room, k)) = best else { continue };
            if way_in || radii.iter().any(|&h| h <= room - g.c && h > 4. * g.c) {
                continue;
            }
            found.push(crate::entry::UnenteredArea {
                reason: if self.entry.policy() == EntryPolicy::StockEdge {
                    "no_helix_allowed"
                } else {
                    "no_helix_fits"
                },
                at_mm: g.center(k),
                min_mm: low,
                max_mm: high,
                level_z_mm: self.z,
                cutter_radius_mm: self.r,
                radii_tried_mm: radii.clone(),
                needed_mm: 4. * g.c,
                available_mm: Some(room - g.c),
            });
        }
        found
    }

    /// Areas with material where the cutter fits the region exactly but
    /// this planner has no position at all (#250): the region leaves the
    /// cutter less play there than the steering grid resolves
    /// (`grid_clearance`), as in a slot a few tenths wider than the cutter.
    ///
    /// Each is found from a point of a guide stroke: a cutter-centre path
    /// of the exact region (`PocketPlan`'s, each proven to keep the cutter
    /// off the walls). There is material at it that no allowed centre
    /// reaches, and the area of near-fitting centres around it holds no
    /// allowed centre: it is not a corner or a neck of an area the planner
    /// does work in (what it leaves there is beside its cuts, and counted
    /// in `residual_unresolved_area_mm2`), but an area it cannot work in
    /// at all.
    fn unresolved_areas(&self) -> Vec<crate::entry::UnenteredArea> {
        let mut found = vec![];
        let Some(layer) = self.layer.as_ref() else {
            return found;
        };
        let g = layer.g;
        // Centres the cutter fits at, to within the grid's resolution.
        let fits = self.r + self.op.radial_allowance_mm - 2. * g.c;
        // Per cell: 1 in an area with an allowed centre, 2 in one without.
        let mut area = vec![0u8; g.nx * g.ny];
        for s in self.guides.guide_strokes() {
            let steps = (crate::profiles::stroke_length(s) / g.c).ceil().max(1.) as usize;
            for i in 0..=steps {
                let t = i as f64 / steps as f64;
                let p = match s.arc {
                    None => [
                        s.a[0] + t * (s.b[0] - s.a[0]),
                        s.a[1] + t * (s.b[1] - s.a[1]),
                    ],
                    Some((c, sweep)) => {
                        let angle = (s.a[1] - c[1]).atan2(s.a[0] - c[0]) + t * sweep;
                        let radius = (s.a[0] - c[0]).hypot(s.a[1] - c[1]);
                        [c[0] + radius * angle.cos(), c[1] + radius * angle.sin()]
                    }
                };
                let Some((ci, cj)) = g.cell(p) else { continue };
                let k = g.idx(ci, cj);
                if !layer.material(k) || layer.reachable[k] || area[k] != 0 {
                    continue;
                }
                let mut seen = vec![k];
                let mut open = vec![k];
                let mut resolved = false;
                area[k] = 2;
                while let Some(k) = open.pop() {
                    g.for_neighbours(k, |m| {
                        if area[m] == 1 || layer.allowed[m] {
                            resolved = true;
                        } else if area[m] == 0 && layer.room()[m] as f64 >= fits {
                            area[m] = 2;
                            seen.push(m);
                            open.push(m);
                        }
                    });
                }
                if resolved {
                    for m in seen {
                        area[m] = 1;
                    }
                    continue;
                }
                let (mut low, mut high) = (p, p);
                for m in seen {
                    let q = g.center(m);
                    low = [low[0].min(q[0]), low[1].min(q[1])];
                    high = [high[0].max(q[0]), high[1].max(q[1])];
                }
                found.push(crate::entry::UnenteredArea {
                    reason: "tighter_than_planner_resolves",
                    at_mm: p,
                    min_mm: low,
                    max_mm: high,
                    level_z_mm: self.z,
                    cutter_radius_mm: self.r,
                    radii_tried_mm: vec![],
                    needed_mm: self.grid_clearance(g.c) - self.r - self.op.radial_allowance_mm,
                    available_mm: None,
                });
            }
        }
        found
    }

    /// Enclosed material: helix down at the most open reachable point.
    fn helix_pass(&mut self, stock: &Stock, cancel: &impl CancelSignal) -> Result<bool, String> {
        if let Some(ramp) = self.entry.ramp().cloned() {
            return self.ramp_pass(&ramp, stock, cancel);
        }
        let r = self.r;
        let z = self.z;
        loop {
            if cancel.is_cancelled() {
                return Err("engagement_front planning cancelled".into());
            }
            let layer = self.layer.as_ref().unwrap();
            let c = layer.g.c;
            let need = self.grid_clearance(c);
            let best = layer.most_open(need);
            let Some((room, k)) = best else {
                return Ok(false);
            };
            let q = layer.g.center(k);
            let helix = self
                .entry
                .helix()
                .ok_or("engagement_front: missing helical entry")?
                .clone();
            let radii = helix.radii(r, r / 2., 0.);
            let Some(rh) = radii.into_iter().find(|&h| h <= room - c && h > 4. * c) else {
                // No helix fits at the most open point left, so at none of
                // it: it stays, counted in the level's residual area. An
                // area nothing enters at all was listed when the level
                // started (`unentered_areas`).
                self.mark_dead(q, 1);
                continue;
            };
            if self.entry.policy() == EntryPolicy::StockEdge {
                return Err(format!(
                    "engagement_front: enclosed material near ({:.3}, {:.3}) at Z{z} needs a helical entry; StockEdgeEntry(helix=None) forbids it",
                    q[0], q[1]
                ));
            }
            for half in [0., PI] {
                let a = [q[0] + rh * half.cos(), q[1] + rh * half.sin()];
                let b = [q[0] - rh * half.cos(), q[1] - rh * half.sin()];
                self.certify(Stroke {
                    a,
                    b,
                    arc: Some((q, PI)),
                })?;
            }
            let (swept, opposite) =
                self.guides
                    .cleared_entry_start(q, rh, z, Some(stock), cancel)?;
            // The helix ramps only through material: above the committed
            // sweeps' proof, or above every whole-cell stock bound the tool
            // can meet anywhere on the bore, it is air. Descend straight
            // down to there (a rapid to the proven height, then the feed
            // plunge) and ramp from there; a bore that is air down to the
            // layer needs no helix at all.
            let top = swept
                .min(self.guides.cell_sweep_height(stock, q, rh))
                .max(z);
            // Within the skin a proven entry start may stop short of the
            // level (`entry::skin`): a helix through less would be a few
            // micrometres tall.
            let air = top <= z + crate::entry::skin(self.op.tolerance_mm);
            // The ramp starts `ramp_clearance` above the material it found
            // (never above the feed height), so the helix is already turning
            // when the cutter reaches stock.
            let begin = (top + self.ramp_clearance.unwrap_or(0.))
                .min(self.heights.feed)
                .max(top);
            let curves = if air {
                vec![]
            } else {
                helix.curves(q, rh, begin, z, !self.climb, opposite)?
            };
            let start = curves.first().map_or_else(
                || [q[0] + if opposite { -rh } else { rh }, q[1], z],
                Curve::start,
            );
            let here = self.position;
            // The helix's flat floor circle is a premise for the cuts that
            // follow it.
            let from = self.pending.len();
            self.layer.as_mut().unwrap().journal.clear();
            let proven = self
                .proven(top)
                .map(|h| h.max(self.guides.cell_entry_height(stock, [start[0], start[1]])));
            // A rapid-descent stop below the helix start (rapid clearance
            // under the ramp clearance) ends at the start instead: higher
            // than the proven stop, so no less safe, and continuous.
            let above = self.travel(stock, here, [start[0], start[1]], proven, start[2]);
            if above[2] > start[2] {
                self.line(above, start, MoveKind::Plunge);
            } else if above[2] < start[2] {
                return Err(format!(
                    "engagement_front helix start Z{:.4} is above the retract height \
                     Z{:.4}; raise retract_z_mm",
                    start[2], above[2]
                ));
            }
            if air {
                self.report.air_entries += 1;
            } else {
                let feed = Some(helix.feed_mm_min.unwrap_or(self.op.plunge_mm_min));
                self.report.entry_turns += curves.len() - 1;
                for curve in curves {
                    self.queue(curve, MoveKind::Lead, feed);
                }
                self.report.helical_entries += 1;
            }
            self.layer.as_mut().unwrap().cut(q, r + rh);
            self.report.lifts += 1;
            self.position = [start[0], start[1], z];
            // Continue clearing from the helix: follow the new front.
            let side = if self.climb { 1. } else { -1. };
            let p = [start[0], start[1]];
            let radial = [(p[0] - q[0]) / rh, (p[1] - q[1]) / rh];
            let heading = (side * radial[0]).atan2(-side * radial[1]);
            if let Some(pts) = self.proven_pass(stock, p, heading, false, from, cancel)? {
                let end = *pts.last().unwrap();
                self.position = [end[0], end[1], z];
            }
            self.report.passes += 1;
            return Ok(true);
        }
    }

    pub(crate) fn next_motion_with_stock(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        if stock.spec() != &self.spec
            || stock.generation() == u64::MAX
            || self.generation.is_some_and(|g| g != stock.generation())
        {
            return Err(
                "commit preceding engagement_front motion to matching stock before planning".into(),
            );
        }
        while self.pending.is_empty() && !self.finished {
            if self.layer.is_none() {
                self.build_layer(stock, cancel)?;
                self.note_unentered()?;
            }
            if !self.plan_pass(stock, cancel)? {
                if (self.wall_pass || self.wall_runs) && !self.walls_done {
                    self.walls_done = true;
                    if self.wall_runs {
                        self.plan_wall_runs(stock, cancel)?;
                    } else {
                        self.plan_walls(stock, cancel)?;
                    }
                    continue;
                }
                let here = self.position;
                self.lift(here, self.op.clearance_z_mm);
                self.finished = true;
            }
        }
        let Some((curve, kind, entry_feed)) = self.pending.pop_front() else {
            return Ok(None);
        };
        curve.prepare(self.op.tolerance_mm)?;
        let motion = Motion {
            id: self.next_id,
            operation: self.op.id.clone(),
            tool: self.op.tool.clone(),
            feed_mm_min: match kind {
                MoveKind::Rapid => None,
                MoveKind::Plunge => Some(self.op.plunge_mm_min),
                _ => Some(entry_feed.unwrap_or(self.op.feed_mm_min)),
            },
            kind,
            curve,
        };
        self.position = motion.curve.end();
        self.next_id += 1;
        self.report.motions += 1;
        self.generation = Some(stock.generation() + 1);
        Ok(Some(motion))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn edt_matches_brute_force() {
        let g = Grid {
            x0: 0.,
            y0: 0.,
            c: 0.5,
            nx: 9,
            ny: 7,
        };
        let mask: Vec<bool> = (0..63).map(|k| k == 10 || k == 50).collect();
        let d = edt(&g, &mask);
        for (k, got) in d.iter().enumerate() {
            let p = g.center(k);
            let want = [10, 50]
                .iter()
                .map(|&m| {
                    let q = g.center(m);
                    (p[0] - q[0]).hypot(p[1] - q[1])
                })
                .fold(f64::INFINITY, f64::min);
            assert!((got - want).abs() < 1e-9, "{k}: {got} vs {want}");
        }
    }

    #[test]
    fn fitted_strokes_stay_within_tolerance_of_every_point() {
        let pts: Vec<Point> = (0..60)
            .map(|i| {
                let a = i as f64 * 0.05;
                if i < 30 {
                    [3. * a.cos(), 3. * a.sin()]
                } else {
                    [3. * 1.45f64.cos() + (a - 1.45), 3. * 1.45f64.sin()]
                }
            })
            .collect();
        let strokes = fit_strokes(&pts, 0.005, 1000.);
        assert!(strokes.len() < 10, "{}", strokes.len());
        assert!(strokes.iter().any(|s| s.arc.is_some()));
        for p in &pts {
            let near = strokes.iter().any(|s| match s.arc {
                None => {
                    let (d, l) = (
                        [s.b[0] - s.a[0], s.b[1] - s.a[1]],
                        (s.b[0] - s.a[0]).hypot(s.b[1] - s.a[1]),
                    );
                    let t =
                        (((p[0] - s.a[0]) * d[0] + (p[1] - s.a[1]) * d[1]) / (l * l)).clamp(0., 1.);
                    (s.a[0] + t * d[0] - p[0]).hypot(s.a[1] + t * d[1] - p[1]) <= 0.005 + 1e-9
                }
                Some((c, _)) => {
                    ((p[0] - c[0]).hypot(p[1] - c[1]) - (s.a[0] - c[0]).hypot(s.a[1] - c[1])).abs()
                        <= 0.005 + 1e-9
                }
            });
            assert!(near, "{p:?}");
        }
    }

    /// Three points far apart on a tight circle (a front step of 0.95 mm
    /// around a 0.86 mm turn, Core Front roughing at Z41.692): the arc
    /// through them bows 0.14 mm out from the polyline between them. With a
    /// check that refuses it (here: leaving the polyline by more than the
    /// tolerance), the points' own lines are cut instead.
    #[test]
    fn fitted_arcs_stay_within_tolerance_between_points() {
        let (rho, tol) = (0.8644863398607524, 0.0625);
        let pts: Vec<Point> = [0f64, 1.165, 2.33]
            .iter()
            .map(|a| [rho * a.cos(), rho * a.sin()])
            .collect();
        let polyline = |p: Point| {
            pts.windows(2)
                .map(|w| {
                    let d = [w[1][0] - w[0][0], w[1][1] - w[0][1]];
                    let t = (((p[0] - w[0][0]) * d[0] + (p[1] - w[0][1]) * d[1])
                        / (d[0] * d[0] + d[1] * d[1]))
                        .clamp(0., 1.);
                    (w[0][0] + t * d[0] - p[0]).hypot(w[0][1] + t * d[1] - p[1])
                })
                .fold(f64::INFINITY, f64::min)
        };
        let sample = |s: Stroke, f: f64| match s.arc {
            None => [
                s.a[0] + (s.b[0] - s.a[0]) * f,
                s.a[1] + (s.b[1] - s.a[1]) * f,
            ],
            Some((c, sweep)) => {
                let r = (s.a[0] - c[0]).hypot(s.a[1] - c[1]);
                let a = (s.a[1] - c[1]).atan2(s.a[0] - c[0]) + sweep * f;
                [c[0] + r * a.cos(), c[1] + r * a.sin()]
            }
        };
        let fits = |s: Stroke| (0..=64).all(|k| polyline(sample(s, k as f64 / 64.)) <= tol);
        let fitted = fit_strokes(&pts, tol, 1000.);
        assert!(
            fitted.iter().any(|&s| !fits(s)),
            "the plain fit no longer bows out here: {fitted:?}"
        );
        let (strokes, refitted) = fit_certifiable(&pts, tol, 1000., fits);
        assert_eq!(refitted, 1);
        assert_eq!(strokes.len(), 2);
        for s in strokes {
            for k in 0..=64 {
                let p = sample(s, k as f64 / 64.);
                assert!(
                    polyline(p) <= tol + 1e-9,
                    "{s:?} leaves the polyline at {p:?}"
                );
            }
        }
    }

    /// A pass makes progress only by clearing material in the plan's own
    /// model. Material just beyond the cutter's reach (here a ring of cells
    /// between r and r + c/2 from the centre, which a step's area counts in
    /// part) is never cleared, so cutting the same disk again reports none:
    /// before, a pass that only met such cells counted as progress and Core
    /// Front's layer at Z10.436 replanned the same six passes forever.
    #[test]
    fn a_repeated_cut_clears_nothing() {
        let g = Grid {
            x0: 0.,
            y0: 0.,
            c: 0.25,
            nx: 64,
            ny: 64,
        };
        let n = g.nx * g.ny;
        let mut layer = Layer {
            g,
            top: vec![10.; n],
            cleared: vec![f64::INFINITY; n],
            inside: vec![true; n],
            allowed: vec![true; n],
            reachable: vec![true; n],
            room: std::cell::OnceCell::from(vec![100.; n]),
            outside: vec![],
            dead: vec![false; n],
            candidates: vec![],
            thin: vec![],
            z: 0.,
            material_guard: 1e-4,
            journal: vec![],
            bits: MaterialBits::default(),
            open: Default::default(),
        };
        layer.sync_bits();
        let (p, r) = ([8., 8.], 3.);
        let before = (0..n).filter(|&k| layer.material(k)).count();
        let first = layer.cut(p, r);
        assert!(first > 0);
        assert_eq!(
            before - first,
            (0..n).filter(|&k| layer.material(k)).count()
        );
        assert_eq!(layer.cut(p, r), 0);
        // Cells already below the layer (no material) are not progress.
        layer.top = vec![-1.; n];
        layer.cleared = vec![f64::INFINITY; n];
        layer.sync_bits();
        assert_eq!(layer.cut(p, r), 0);
    }

    #[test]
    fn a_step_along_a_wall_sees_no_unreachable_material() {
        // Allowed centres end at the row centred on Y4.875; reachable cells
        // (within r - c/2 of an allowed centre, as `build_layer` marks them)
        // end at Y7.75, and the part (stock no centre can cut) stands beyond.
        // Once the reachable band is cleared, a step along the wall meets
        // nothing, though the part's first cell row (Y7.875) lies within the
        // smoothing band of a cutter centred at Y4.9.
        let g = Grid {
            x0: 0.,
            y0: 0.,
            c: 0.25,
            nx: 64,
            ny: 64,
        };
        let n = g.nx * g.ny;
        let r = 3.;
        let y = |k: usize| g.center(k)[1];
        let reachable: Vec<bool> = (0..n)
            .map(|k| y(k) <= 4.875 + r - 0.5 * g.c + 1e-9)
            .collect();
        let mut layer = Layer {
            g,
            top: vec![10.; n],
            cleared: (0..n)
                .map(|k| if reachable[k] { 0. } else { f64::INFINITY })
                .collect(),
            inside: vec![true; n],
            allowed: (0..n).map(|k| y(k) <= 5.).collect(),
            reachable,
            room: std::cell::OnceCell::from(vec![100.; n]),
            outside: vec![],
            dead: vec![false; n],
            candidates: vec![],
            thin: vec![],
            z: 0.,
            material_guard: 1e-4,
            journal: vec![],
            bits: MaterialBits::default(),
            open: Default::default(),
        };
        layer.sync_bits();
        let (p, q) = ([8., 4.9], [8.9, 4.9]);
        assert!(layer.allowed_at(p) && layer.allowed_at(q));
        // The part's material is there, inside the cutter's smoothing band.
        assert!((0..n).any(|k| layer.material(k) && area(&[g.center(k)], q, r, g.c) > 0.));
        let mut cells = vec![];
        layer.gather(p, r + g.c, &mut cells);
        assert_eq!(area(&cells, q, r, g.c), 0.);
    }

    /// A layer whose material and reachable cells are given (top 10 over a
    /// level at 0), every centre allowed.
    fn layer_of(
        g: Grid,
        material: impl Fn(usize) -> bool,
        reachable: impl Fn(usize) -> bool,
    ) -> Layer {
        let n = g.nx * g.ny;
        let mut layer = Layer {
            g,
            top: (0..n)
                .map(|k| if material(k) { 10. } else { -1. })
                .collect(),
            cleared: vec![f64::INFINITY; n],
            inside: vec![true; n],
            allowed: vec![true; n],
            reachable: (0..n).map(reachable).collect(),
            room: std::cell::OnceCell::from(vec![100.; n]),
            outside: vec![],
            dead: vec![false; n],
            candidates: vec![],
            thin: vec![],
            z: 0.,
            material_guard: 1e-4,
            journal: vec![],
            bits: MaterialBits::default(),
            open: Default::default(),
        };
        layer.sync_bits();
        layer
    }

    /// Deterministic pseudo-random numbers in 0..1 (SplitMix64).
    struct Random(u64);
    impl Random {
        fn next(&mut self) -> f64 {
            self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
            let mut z = self.0;
            z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
            z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
            ((z ^ (z >> 31)) >> 11) as f64 / (1u64 << 53) as f64
        }
        fn below(&mut self, n: usize) -> usize {
            ((self.next() * n as f64) as usize).min(n - 1)
        }
    }

    type Found = Option<(usize, [u64; 2], u64)>;
    fn exact(found: Option<(usize, Point, f64)>) -> Found {
        found.map(|(k, q, d)| (k, q.map(f64::to_bits), d.to_bits()))
    }
    /// The definition: scan the disk, keep the first cell of the smallest
    /// distance among those `keep` accepts.
    fn nearest_scan(layer: &Layer, keep: impl Fn(usize) -> bool, p: Point, rad: f64) -> Found {
        let mut nearest: Option<(usize, Point, f64)> = None;
        layer.g.for_disk(p, rad, |k, q, d| {
            if keep(k) && nearest.is_none_or(|(_, _, b)| d < b) {
                nearest = Some((k, q, d));
            }
        });
        exact(nearest)
    }
    fn bits_are_current(layer: &Layer) {
        let g = layer.g;
        for k in 0..g.nx * g.ny {
            let (w, bit) = (
                (k / g.nx) * layer.bits.stride + (k % g.nx) / 64,
                k % g.nx % 64,
            );
            let material = layer.material(k);
            assert_eq!(layer.bits.any[w] >> bit & 1 == 1, material, "cell {k}");
            assert_eq!(
                layer.bits.uncut[w] >> bit & 1 == 1,
                layer.cleared[k] > layer.z,
                "cell {k}"
            );
            assert_eq!(
                layer.bits.reach[w] >> bit & 1 == 1,
                material && layer.reachable[k],
                "cell {k}"
            );
        }
    }

    /// #202: the searches that read the material bits return exactly what
    /// a scan of the disk returns: the same cell, centre and distance to
    /// the bit, with the scan's choice between cells at equal distance
    /// (queries from cell centres make many), at the edge of the disk and
    /// of the grid, and after cuts and their undoing.
    #[test]
    fn bit_searches_match_a_scan_of_the_disk() {
        let mut random = Random(202);
        let grids = [
            Grid {
                x0: -17.3,
                y0: 4.1,
                c: 0.3,
                nx: 150,
                ny: 97,
            },
            Grid {
                x0: 0.,
                y0: 0.,
                c: 0.25,
                nx: 64,
                ny: 130,
            },
            Grid {
                x0: 1000.7,
                y0: -2000.2,
                c: 0.07,
                nx: 201,
                ny: 77,
            },
        ];
        let (mut compared, mut found, mut tied) = (0, 0, 0);
        for g in grids {
            let n = g.nx * g.ny;
            let r = 20. * g.c;
            let centre = g.center(g.idx(g.nx / 2, g.ny / 2));
            let noise: Vec<f64> = (0..n).map(|_| random.next()).collect();
            let off = |k: usize| {
                let q = g.center(k);
                (q[0] - centre[0]).hypot(q[1] - centre[1])
            };
            type Pattern<'a> = Box<dyn Fn(usize) -> bool + 'a>;
            let patterns: Vec<Pattern> = vec![
                Box::new(|_| false),
                Box::new(|k| noise[k] < 0.004),
                Box::new(|k| noise[k] < 0.5),
                // A straight front along a column, and along a row.
                Box::new(|k| k % g.nx > g.nx / 2 + 21),
                Box::new(|k| k / g.nx < g.ny / 2 - 23),
                // A ring about a cell centre: many cells at equal distance.
                Box::new(|k| off(k) >= r + g.c && off(k) <= r + 2. * g.c),
                Box::new(|k| off(k) > 1.5 * r),
                // One cell exactly a disk radius away along a row.
                Box::new(|k| k == g.idx(g.nx / 2 + 23, g.ny / 2)),
                Box::new(|_| true),
            ];
            for material in &patterns {
                let mut layer = layer_of(g, material, |k| noise[k] > 0.2);
                bits_are_current(&layer);
                for round in 0..3 {
                    for _ in 0..150 {
                        let k = match random.below(3) {
                            0 => g.idx(g.nx / 2, g.ny / 2),
                            _ => random.below(n),
                        };
                        let mut p = g.center(k);
                        if random.below(2) == 0 {
                            p = [
                                p[0] + (random.next() - 0.5) * 3. * g.c,
                                p[1] + (random.next() - 0.5) * 3. * g.c,
                            ];
                        }
                        if random.below(20) == 0 {
                            // Off the grid.
                            p[0] = g.x0 - random.next() * 2. * r;
                        }
                        let rad = match random.below(4) {
                            0 => r + 3. * g.c,
                            1 => 1.5 * r + 2. * g.c,
                            2 => 23. * g.c,
                            _ => random.next() * 2. * r,
                        };
                        let limit = if random.below(2) == 0 {
                            r + g.c
                        } else {
                            random.next() * rad
                        };
                        for reach in [false, true] {
                            let bits = if reach {
                                &layer.bits.reach
                            } else {
                                &layer.bits.any
                            };
                            let keep =
                                |q: usize| layer.material(q) && (!reach || layer.reachable[q]);
                            let want = nearest_scan(&layer, keep, p, rad);
                            assert_eq!(
                                exact(layer.nearest_set(bits, p, rad)),
                                want,
                                "nearest to {p:?} within {rad} on {g:?}"
                            );
                            let mut near = false;
                            let mut ties = 0;
                            layer.g.for_disk(p, rad, |q, _, d| {
                                near |= keep(q) && d <= limit;
                                ties += usize::from(
                                    keep(q) && want.is_some_and(|w| w.2 == d.to_bits()),
                                );
                            });
                            assert_eq!(
                                layer.any_set_within(bits, p, rad, limit),
                                near,
                                "within {limit} of {p:?} in {rad} on {g:?}"
                            );
                            // The disk's cells without their distances.
                            let (mut cells, mut scan) = (vec![], vec![]);
                            layer.g.for_disk_cells(p, rad, |q, c| cells.push((q, c)));
                            layer.g.for_disk(p, rad, |q, c, _| scan.push((q, c)));
                            assert_eq!(cells, scan, "disk at {p:?} of {rad} on {g:?}");
                            // (`gather` checks itself against its scan.)
                            layer.gather(p, rad, &mut vec![]);
                            compared += 1;
                            found += usize::from(want.is_some());
                            tied += usize::from(ties > 1);
                        }
                    }
                    // Cut a few disks, then (last round) undo some of them.
                    let mark = layer.journal.len();
                    for i in 0..4 {
                        let p = g.center(random.below(n));
                        layer.cut(p, r * (0.5 + random.next()));
                        if round == 1 && i == 1 {
                            layer.undo((mark + layer.journal.len()) / 2);
                        }
                    }
                    bits_are_current(&layer);
                }
            }
        }
        // The cases meant to be covered were: answers, and equal distances.
        assert!(
            compared > 20000 && found > compared / 3 && tied > 500,
            "{compared} {found} {tied}"
        );
    }

    /// #192: the queue of start candidates hands them out in the order a
    /// scan of the whole list for the nearest does, candidates at equal
    /// distance and repeated candidates included, and leaves the list in
    /// the same order.
    #[test]
    fn start_queue_takes_candidates_in_the_order_of_a_scan() {
        let g = Grid {
            x0: -3.,
            y0: 2.,
            c: 0.3,
            nx: 40,
            ny: 30,
        };
        let mut random = Random(192);
        for pos in [g.center(g.idx(20, 15)), [1.234, 5.678], [-40., 90.]] {
            let mut layer = layer_of(g, |_| false, |_| true);
            // A ring about the position (equal distances), scattered cells
            // and some of them twice (`revive_thin_starts` may repeat one).
            layer.candidates = (0..g.nx * g.ny)
                .filter(|&k| k % 7 == 3 || (k % g.nx).abs_diff(20) + (k / g.nx).abs_diff(15) == 6)
                .collect();
            for _ in 0..40 {
                let k = layer.candidates[random.below(layer.candidates.len())];
                layer.candidates.push(k);
            }
            let mut list = layer.candidates.clone();
            let mut queue = StartQueue::new(&layer, pos);
            let mut taken = 0;
            loop {
                // The scan, as `plan_pass` did it.
                let want = list
                    .iter()
                    .enumerate()
                    .map(|(i, &k)| {
                        let q = g.center(k);
                        (i, k, (q[0] - pos[0]).hypot(q[1] - pos[1]))
                    })
                    .min_by(|a, b| a.2.total_cmp(&b.2));
                let got = queue.take(&mut layer);
                assert_eq!(got, want.map(|(_, k, _)| k));
                let Some((i, _, _)) = want else { break };
                list.swap_remove(i);
                assert_eq!(layer.candidates, list);
                taken += 1;
            }
            assert!(taken > 200 && layer.candidates.is_empty());
        }
    }

    /// Plan a fixture's first operation against its committed stock, up to
    /// `limit` motions. In a test build every search of the planner checks
    /// itself against the scan it replaced (`candidate_direction`,
    /// `nearest_material`, `StartQueue::take`), so a run is a comparison
    /// of the two on every decision it makes.
    fn plan_checked(text: &str, limit: usize) -> (usize, serde_json::Value) {
        let m: crate::stream::StreamManifest = serde_json::from_str(text).unwrap();
        m.validate().unwrap();
        let tool = m.tools[0].prepare().unwrap();
        let mut stock = Stock::new(m.stock.height().unwrap().clone()).unwrap();
        let mut plan = crate::operation::OperationPlan::prepare_with_stock(
            m.operations[0].clone(),
            &m.tools[0],
            &stock,
            m.initial_position_mm,
            1,
            &|| false,
        )
        .unwrap();
        let mut motions = 0;
        while motions < limit
            && let Some(motion) = plan.next_motion_with_stock(&stock, &|| false).unwrap()
        {
            let delta = stock
                .preview_sweep(&tool, &motion, 1., m.sweep, &|| false)
                .unwrap();
            stock.apply_delta(&delta).unwrap();
            motions += 1;
        }
        (motions, serde_json::to_value(plan.report()).unwrap())
    }

    /// The twelve triangles of a box.
    fn box_triangles(lo: [f64; 3], hi: [f64; 3]) -> Vec<[[f64; 3]; 3]> {
        let v: Vec<[f64; 3]> = (0..8)
            .map(|n| {
                [
                    if n & 1 == 0 { lo[0] } else { hi[0] },
                    if n & 2 == 0 { lo[1] } else { hi[1] },
                    if n & 4 == 0 { lo[2] } else { hi[2] },
                ]
            })
            .collect();
        [
            [0, 2, 3, 1],
            [4, 5, 7, 6],
            [0, 1, 5, 4],
            [2, 6, 7, 3],
            [0, 4, 6, 2],
            [1, 3, 7, 5],
        ]
        .iter()
        .flat_map(|q| [[v[q[0]], v[q[1]], v[q[2]]], [v[q[0]], v[q[2]], v[q[3]]]])
        .collect()
    }

    /// A plate with two bosses and a frame, in the open block's stock:
    /// links routed around the bosses, goals out of a link's reach, passes
    /// planned again, candidate refreshes.
    #[test]
    fn plate_with_bosses_plan_matches_the_scans_at_every_decision() {
        let mut m: serde_json::Value = serde_json::from_str(include_str!(
            "../../fixtures/operations/roughing-open-block.json"
        ))
        .unwrap();
        let mut triangles = box_triangles([0., 0., 0.], [100., 60., 14.]);
        for (lo, hi) in [
            ([8., 8.], [30., 24.]),
            ([40., 6.], [52., 54.]),
            ([62., 30.], [96., 36.]),
            ([62., 50.], [96., 56.]),
            ([62., 30.], [68., 56.]),
            ([90., 30.], [96., 56.]),
        ] {
            triangles.extend(box_triangles([lo[0], lo[1], 0.], [hi[0], hi[1], 21.]));
        }
        let op = &mut m["operations"][0];
        op["target"]["triangles"] = serde_json::to_value(triangles).unwrap();
        op["entry"]["kind"] = "stock_edge_or_helix".into();
        let (motions, report) = plan_checked(&m.to_string(), 1500);
        let clearing = &report["layers"][0]["clearing"];
        // Every shortcut was taken, and so compared with its definition.
        let taken = SHORTCUTS.with(|taken| taken.get());
        assert!(taken.iter().all(|&n| n >= 3), "{taken:?}");
        let count = |name: &str| clearing[name].as_u64().unwrap();
        assert!(count("passes") >= 20, "{clearing}");
        assert!(count("routed_links") >= 1, "{clearing}");
        assert!(count("grid_refreshes") >= 2, "{clearing}");
        assert!(
            motions > 300 && count("engagement_replans") >= 1,
            "{clearing}"
        );
    }

    #[test]
    fn open_block_plan_matches_the_scans_at_every_decision() {
        let (motions, report) = plan_checked(
            include_str!("../../fixtures/operations/roughing-open-block.json"),
            usize::MAX,
        );
        let clearing = &report["layers"][0]["clearing"];
        assert_eq!(clearing["strategy"], "engagement_front");
        assert!(motions > 100 && clearing["passes"].as_u64().unwrap() >= 5);
    }

    #[test]
    fn frustum_plan_matches_the_scans_at_every_decision() {
        // The first passes of each of the first layers of the benchmark's
        // `roughing-adaptive` part (the whole run is the benchmark's).
        let (motions, _) = plan_checked(
            include_str!("../../fixtures/operations/roughing-adaptive.json"),
            600,
        );
        assert_eq!(motions, 600);
    }

    /// #202: growing the set cells by a disk answers "is a set cell within
    /// this distance" exactly as the distance transform compared with the
    /// distance does: on whole grids and on windows of them (set cells
    /// outside a window unseen), for distances that fall exactly on cell
    /// distances (3, 4, 5 cells) and between them, either comparison.
    #[test]
    fn grown_bits_match_the_distance_transform() {
        let mut random = Random(7);
        let mut compared = 0;
        for (nx, ny, c) in [
            (150, 97, 0.3),
            (64, 130, 0.25),
            (201, 40, 0.07),
            (70, 70, 1. / 6.),
        ] {
            let g = Grid {
                x0: -3.3,
                y0: 9.1,
                c,
                nx,
                ny,
            };
            let n = nx * ny;
            for density in [0., 0.0005, 0.01, 0.3] {
                // The reachable bits are the set cells here.
                let set: Vec<bool> = (0..n).map(|_| random.next() < density).collect();
                let layer = layer_of(g, |_| true, |k| set[k]);
                for _ in 0..6 {
                    let (i0, j0) = (random.below(nx), random.below(ny));
                    let (i1, j1) = (i0 + random.below(nx - i0), j0 + random.below(ny - j0));
                    let (i, j) = if random.below(3) == 0 {
                        ([0, nx - 1], [0, ny - 1])
                    } else {
                        ([i0, i1], [j0, j1])
                    };
                    let w = Grid {
                        x0: g.x0 + i[0] as f64 * c,
                        y0: g.y0 + j[0] as f64 * c,
                        c,
                        nx: i[1] - i[0] + 1,
                        ny: j[1] - j[0] + 1,
                    };
                    let mask: Vec<bool> = (0..w.nx * w.ny)
                        .map(|k| set[g.idx(i[0] + k % w.nx, j[0] + k / w.nx)])
                        .collect();
                    let distance = edt(&w, &mask);
                    for t in [
                        0.,
                        c,
                        3. * c,
                        5. * c,
                        2f64.sqrt() * c,
                        (0.2 + 12. * random.next()) * c,
                        20. * c + c,
                        20. * c - 0.6,
                    ] {
                        for inclusive in [false, true] {
                            let near = cells2_within(c, t, inclusive).map(|d2| {
                                Near::new(&layer.bits.reach, layer.bits.stride, i, j, d2)
                            });
                            for (k, &d) in distance.iter().enumerate() {
                                let want = if inclusive { d <= t } else { d < t };
                                let got = near
                                    .as_ref()
                                    .is_some_and(|n| n.at(i[0] + k % w.nx, j[0] + k / w.nx));
                                assert_eq!(got, want, "cell {k} of {i:?} {j:?}, within {t}");
                            }
                            compared += 1;
                        }
                    }
                }
            }
        }
        assert!(compared > 1000);
    }

    /// #192: the load sums take a square root only for the cells whose
    /// count depends on it, and return what the formula over every cell
    /// returns, to the bit (each checks itself against the formula in a
    /// test build): cells exactly half a cell inside and outside the
    /// cutter and a cell and a half outside it included.
    #[test]
    fn load_sums_match_their_formulas() {
        let mut random = Random(26);
        for (r, c) in [(1., 0.05), (6., 1. / 6.), (5., 0.25), (0.3, 0.25)] {
            let g = Grid {
                x0: -2. * r,
                y0: -2. * r,
                c,
                nx: (4. * r / c) as usize + 1,
                ny: (4. * r / c) as usize + 1,
            };
            let mut cells: Vec<Point> = (0..g.nx * g.ny).map(|k| g.center(k)).collect();
            for _ in 0..200 {
                let centre = cells[random.below(g.nx * g.ny)];
                let p = if random.below(2) == 0 {
                    centre
                } else {
                    [random.next() - 0.5, random.next() - 0.5]
                };
                // Cells on the edges of the smoothing bands, exactly.
                for edge in [r - 0.5 * c, r + 0.5 * c, r + 1.5 * c, r] {
                    cells.push([p[0] + edge, p[1]]);
                    cells.push([p[0], p[1] - edge]);
                    cells.push([p[0] + 0.6 * edge, p[1] + 0.8 * edge]);
                }
                let a = std::f64::consts::TAU * random.next();
                let prev = [p[0] - 0.3 * r * a.cos(), p[1] - 0.3 * r * a.sin()];
                assert!(area(&cells, p, r, c) > 0.);
                assert!(step_load(&cells, p, [a.cos(), a.sin()], r, c).0 > 0.);
                assert!(fresh_cells(&cells, prev, p, r, c) > 0.);
                cells.truncate(g.nx * g.ny);
            }
        }
    }

    /// The bounded shortest-path set holds exactly the cells whose shortest
    /// path over set cells is within the bound (distances by relaxing every
    /// cell until nothing changes).
    #[test]
    fn cells_within_matches_relaxed_distances() {
        let mut random = Random(11);
        for (cols, rows, density, bound) in [
            (64, 9, 0.6, 7300),
            (128, 40, 0.7, 31700),
            (192, 30, 0.8, 55100),
            (64, 3, 0.9, 12200),
        ] {
            let words = cols / 64;
            for _ in 0..12 {
                let mut pass = vec![0u64; words * rows];
                for j in 0..rows {
                    for i in 0..cols {
                        if random.next() < density {
                            pass[j * words + i / 64] |= 1 << (i % 64);
                        }
                    }
                }
                let start = (random.below(cols), random.below(rows));
                pass[start.1 * words + start.0 / 64] |= 1 << (start.0 % 64);
                let set = |bits: &[u64], i: usize, j: usize| {
                    bits[j * words + i / 64] >> (i % 64) & 1 == 1
                };
                let mut dist = vec![u64::MAX / 2; cols * rows];
                dist[start.1 * cols + start.0] = 0;
                loop {
                    let mut changed = false;
                    for j in 0..rows {
                        for i in 0..cols {
                            if !set(&pass, i, j) {
                                continue;
                            }
                            for dj in -1i64..=1 {
                                for di in -1i64..=1 {
                                    let (a, b) = (i as i64 + di, j as i64 + dj);
                                    if (di, dj) == (0, 0)
                                        || a < 0
                                        || b < 0
                                        || a >= cols as i64
                                        || b >= rows as i64
                                    {
                                        continue;
                                    }
                                    let step = if di != 0 && dj != 0 {
                                        LINK_DIAGONAL
                                    } else {
                                        1024
                                    };
                                    let d = dist[b as usize * cols + a as usize] + step;
                                    if d < dist[j * cols + i] {
                                        dist[j * cols + i] = d;
                                        changed = true;
                                    }
                                }
                            }
                        }
                    }
                    if !changed {
                        break;
                    }
                }
                let within = cells_within(&pass, words, start, bound);
                let mut inside = 0;
                for j in 0..rows {
                    for i in 0..cols {
                        let d = dist[j * cols + i];
                        assert_eq!(set(&within, i, j), d <= bound, "{i} {j}: {d}");
                        inside += usize::from(d <= bound);
                    }
                }
                assert!(inside >= 1);
                // With nothing in the way, the distance is `link_distance`.
                let open = vec![u64::MAX; words * rows];
                let within = cells_within(&open, words, start, bound);
                for j in 0..rows {
                    for i in 0..cols {
                        let d = link_distance(i.abs_diff(start.0), j.abs_diff(start.1));
                        assert_eq!(set(&within, i, j), d <= bound);
                    }
                }
            }
        }
    }

    /// A run of ones added at once is the ones added in turn, to the bit:
    /// from sums with a fraction just under and over powers of two too.
    #[test]
    fn add_ones_matches_adding_one_at_a_time() {
        let mut random = Random(5);
        for round in 0..4000 {
            let x = match round % 5 {
                0 => 0.,
                1 => random.next(),
                2 => (1u64 << random.below(13)) as f64 - random.next() * 3.,
                3 => random.next() * 5000.,
                _ => (random.below(4096) as f64) + random.next() * 1e-9,
            }
            .max(0.);
            let n = random.below(3000);
            let mut want = x;
            for _ in 0..n {
                want += 1.;
            }
            assert_eq!(add_ones(x, n).to_bits(), want.to_bits(), "{x} + {n}");
        }
    }

    /// The sums over a step's cells are the formulas' sums over all the
    /// gathered cells, to the bit, for every centre a step length from the
    /// point the cells were prepared for.
    #[test]
    fn step_sums_match_their_formulas() {
        let mut random = Random(27);
        for (r, c, ds) in [
            (1., 0.05, 0.15),
            (6., 1. / 6., 0.72),
            (6., 0.25, 0.9),
            (5., 0.25, 0.75),
        ] {
            let g = Grid {
                x0: -2. * r,
                y0: -2. * r,
                c,
                nx: (4. * r / c) as usize + 1,
                ny: (4. * r / c) as usize + 1,
            };
            let all: Vec<Point> = (0..g.nx * g.ny).map(|k| g.center(k)).collect();
            let (mut runs, mut near) = (0, vec![]);
            for _ in 0..60 {
                let p = [random.next() - 0.5, random.next() - 0.5];
                // Material on one side of a line through the cutter.
                let (a, cut) = (std::f64::consts::TAU * random.next(), random.next() * r);
                let cells: Vec<Point> = all
                    .iter()
                    .copied()
                    .filter(|q| (q[0] - p[0]) * a.cos() + (q[1] - p[1]) * a.sin() > cut - r)
                    .filter(|q| (q[0] - p[0]).hypot(q[1] - p[1]) <= r + ds + 2. * c)
                    .collect();
                step_cells(&cells, p, ds, r, c, &mut near);
                runs += near
                    .iter()
                    .filter(|cell| matches!(cell, LoadCell::Full(n) if *n > 1))
                    .count();
                for _ in 0..26 {
                    let t = std::f64::consts::TAU * random.next();
                    let (u, q) = (
                        [t.cos(), t.sin()],
                        [p[0] + ds * t.cos(), p[1] + ds * t.sin()],
                    );
                    assert_eq!(
                        area_of(&near, q, r, c).to_bits(),
                        area_plain(&cells, q, r, c).to_bits()
                    );
                    let (got, want) = (
                        step_load_of(&near, q, u, r, c),
                        step_load_plain(&cells, q, u, r, c),
                    );
                    assert_eq!(
                        (got.0.to_bits(), got.1.to_bits()),
                        (want.0.to_bits(), want.1.to_bits())
                    );
                }
            }
            assert!(runs > 100, "{runs}");
        }
    }

    #[test]
    fn floor_and_ceil_match_the_library() {
        let mut random = Random(3);
        let mut values = vec![
            0.,
            -0.,
            0.5,
            -0.5,
            1.,
            -1.,
            4e15,
            -4e15,
            1e300,
            f64::INFINITY,
        ];
        for _ in 0..20000 {
            let scale = [1., 1e3, 1e9, 3.9e15, 1e-9][random.below(5)];
            let x = (random.next() - 0.5) * 2. * scale;
            values.extend([x, x.round(), x.round() + 0.5, x.round() - 1e-9]);
        }
        for x in values {
            assert_eq!(floor(x).to_bits(), x.floor().to_bits(), "floor {x}");
            assert_eq!(ceil(x).to_bits(), x.ceil().to_bits(), "ceil {x}");
        }
        assert!(floor(f64::NAN).is_nan() && ceil(f64::NAN).is_nan());
    }
}
