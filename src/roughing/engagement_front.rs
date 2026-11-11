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
        let i = ((p[0] - self.x0) / self.c).floor();
        let j = ((p[1] - self.y0) / self.c).floor();
        (i >= 0. && j >= 0. && i < self.nx as f64 && j < self.ny as f64)
            .then_some((i as usize, j as usize))
    }
    /// Cells whose centres lie within `rad` of `p`.
    fn for_disk(&self, p: Point, rad: f64, mut f: impl FnMut(usize, Point, f64)) {
        let lo = [
            ((p[0] - rad - self.x0) / self.c - 0.5).ceil().max(0.) as usize,
            ((p[1] - rad - self.y0) / self.c - 0.5).ceil().max(0.) as usize,
        ];
        let hi = [
            (((p[0] + rad - self.x0) / self.c - 0.5).floor()).min(self.nx as f64 - 1.),
            (((p[1] + rad - self.y0) / self.c - 0.5).floor()).min(self.ny as f64 - 1.),
        ];
        if hi[0] < 0. || hi[1] < 0. {
            return;
        }
        for j in lo[1]..=hi[1] as usize {
            let cy = self.y0 + (j as f64 + 0.5) * self.c;
            for i in lo[0]..=hi[0] as usize {
                let cx = self.x0 + (i as f64 + 0.5) * self.c;
                let d = (cx - p[0]).hypot(cy - p[1]);
                if d <= rad {
                    f(self.idx(i, j), [cx, cy], d);
                }
            }
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
    /// Entries the helix search chose whose whole bore was proven air (by
    /// the committed sweeps or the stock cell bounds): a straight plunge.
    pub air_entries: usize,
    /// Vertical entries made by plunging a few stock cells back from the
    /// material (where the cell bounds proved the rapid could come lower)
    /// and feeding across at depth.
    pub offset_descents: usize,
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
    /// Wall passes (see `EngagementFrontPlan::with_wall_pass`): closed loops
    /// cut along the region's walls after the level is cleared, and loops
    /// left uncut because no start beside them was proven clear.
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
    pub geometric_residual_area_mm2: Option<f64>,
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
            air_entries: 0,
            offset_descents: 0,
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
            diagnostics: vec![],
        }
    }
}

struct Layer {
    g: Grid,
    /// Top of possible material per fine cell.
    top: Vec<f64>,
    /// This operation's cleared height per fine cell.
    cleared: Vec<f64>,
    allowed: Vec<bool>,
    /// Material the cutter can reach from an allowed centre.
    reachable: Vec<bool>,
    /// Distance from the cell centre to the outside of the region (mm).
    room: Vec<f32>,
    dead: Vec<bool>,
    candidates: Vec<usize>,
    /// Starts whose pass found the material beside them too thin to cut.
    thin: Vec<Point>,
    z: f64,
    material_guard: f64,
    /// Cleared heights overwritten since the journal was last emptied, so a
    /// pass whose engagement was not proven can be planned again.
    journal: Vec<(usize, f64)>,
}
impl Layer {
    fn undo(&mut self, to: usize) {
        while self.journal.len() > to {
            let (k, old) = self.journal.pop().unwrap();
            self.cleared[k] = old;
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
    fn cut(&mut self, p: Point, rad: f64) -> usize {
        let (z, floor) = (self.z, self.z + self.material_guard);
        let (cleared, top, journal) = (&mut self.cleared, &self.top, &mut self.journal);
        let mut removed = 0;
        self.g.for_disk(p, rad, |k, _, _| {
            if cleared[k] > z {
                removed += usize::from(top[k].min(cleared[k]) > floor);
                journal.push((k, cleared[k]));
                cleared[k] = z;
            }
        });
        removed
    }
    /// Material cell centres a step of up to `reach` from `p` can meet.
    /// Only reachable material counts: stock no allowed centre can cut (the
    /// part and its allowance beyond the region) lies just past the cutter
    /// along every wall, and counting it made a pass ride the wall through
    /// cleared air on material it could never remove.
    fn gather(&self, p: Point, reach: f64, out: &mut Vec<Point>) {
        out.clear();
        self.g.for_disk(p, reach, |k, q, _| {
            if self.reachable[k] && self.material(k) {
                out.push(q);
            }
        });
    }
}

/// Smoothed material area (mm²) inside the cutter disk centred at `p`.
fn area(cells: &[Point], p: Point, r: f64, c: f64) -> f64 {
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
fn step_load(cells: &[Point], p: Point, u: Point, r: f64, c: f64) -> (f64, f64) {
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

pub(crate) struct EngagementFrontPlan {
    guides: PocketPlan,
    op: PocketOperation,
    entry: PocketEntry,
    heights: Heights,
    /// See `RoughingOperation::rapid_descent_clearance_mm`.
    rapid_descent: Option<f64>,
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
    pending: VecDeque<Pending>,
    position: Vec3,
    next_id: u64,
    generation: Option<u64>,
    finished: bool,
    report: EngagementFrontReport,
    scratch: Vec<Point>,
    /// Committed cleared sweeps, for the exact engagement proof of each pass.
    proof: Option<(EngagementScene, ClearanceCursor)>,
    /// Passes may start on a remnant too thin for a full step (see
    /// `follow`): set only to clear material a helix would otherwise bore
    /// onto, until the next pass is planned.
    remnant: bool,
    /// Finish each wall with one pass along it (`with_wall_pass`), and
    /// whether this level's have been planned.
    wall_pass: bool,
    walls_done: bool,
}

impl EngagementFrontPlan {
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
        let guides = PocketPlan::prepare(op.clone(), tool, spec, initial, 1, cancel)?;
        let z = guides.report().target_z_mm;
        let report = EngagementFrontReport {
            strategy: "engagement_front",
            target_z_mm: z,
            target_radial_engagement_mm: width,
            engagement_limit_deg: (1. - width / r).acos().to_degrees(),
            geometric_residual_status: ResidualStatus::Pending,
            diagnostics: vec![
                "Engagement is steered on a fine grid seeded from committed whole-cell stock bounds, then every cutting and stay-down link stroke is proven exactly against the committed cleared sweeps: its contact angle, less the cutter's own trail, is at most engagement_limit_deg = acos(1 - w/r). A pass with an unproven stroke is planned again narrower; helical entries cut a full slot and are not bounded by w".into(),
                "Every cutting and link stroke is certified exactly against the layer region; air entries are proven by committed sweeps; enclosed material is entered by helix".into(),
            ],
            ..Default::default()
        };
        Ok(Self {
            guides,
            op,
            entry,
            heights,
            rapid_descent: None,
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
            pending: VecDeque::new(),
            position: initial,
            next_id: 1,
            generation: None,
            finished: false,
            report,
            scratch: vec![],
            proof: None,
            remnant: false,
            wall_pass: false,
            walls_done: false,
        })
    }
    /// See `RoughingOperation::rapid_descent_clearance_mm` (validated there).
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
    /// grid keeps centres about two cells off the exact offset); a pocket's
    /// walls are finished surfaces, a roughing layer's are not.
    pub(crate) fn with_wall_pass(mut self, on: bool) -> Self {
        self.wall_pass = on;
        self
    }
    pub(crate) fn with_rapid_descent(mut self, clearance: Option<f64>) -> Self {
        self.rapid_descent = clearance;
        self
    }
    pub(crate) fn construction_error_mm(&self) -> f64 {
        self.guides.construction_error_mm()
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
    pub(crate) fn take_memory(&mut self) -> Option<FrontMemory> {
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
                    let c2 = layer.g.c * layer.g.c;
                    let left = (0..layer.top.len())
                        .filter(|&k| layer.reachable[k] && layer.material(k))
                        .count() as f64
                        * c2;
                    self.report.geometric_residual_area_mm2 = Some(left);
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
        let room: Vec<f32> = edt(&g, &outside).into_iter().map(|d| d as f32).collect();
        let need = self.grid_clearance(g.c);
        let allowed: Vec<bool> = room.iter().map(|&d| d as f64 >= need).collect();
        let dallowed = edt(&g, &allowed);
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
                m.cleared
            }
            Some(_) => {
                self.report.diagnostics.push("previous layer's steering memory did not match the committed stock; steering from whole-cell stock bounds only".into());
                vec![f64::INFINITY; n]
            }
            None => vec![f64::INFINITY; n],
        };
        let reach_limit = self.r - 0.5 * g.c;
        let reachable = dallowed.iter().map(|&d| d <= reach_limit).collect();
        self.report.engagement_grid_mm = g.c;
        self.layer = Some(Layer {
            g,
            top,
            cleared,
            allowed,
            reachable,
            room,
            dead: vec![false; n],
            candidates: vec![],
            thin: vec![],
            z: self.z,
            material_guard: 16. * self.op.tolerance_mm,
            journal: vec![],
        });
        Ok(())
    }

    /// Recompute air/cleared start candidates: allowed centres whose cutter
    /// disk is clear of material while reachable material is within a few
    /// grid cells. Returns the reachable material area left.
    fn refresh(&mut self) -> f64 {
        let r = self.r;
        let layer = self.layer.as_mut().unwrap();
        let g = layer.g;
        let n = g.nx * g.ny;
        let mat: Vec<bool> = (0..n).map(|k| layer.material(k)).collect();
        let reach: Vec<bool> = (0..n).map(|k| mat[k] && layer.reachable[k]).collect();
        let left = reach.iter().filter(|&&b| b).count() as f64 * g.c * g.c;
        let dmat = edt(&g, &mat);
        let dk = edt(&g, &reach);
        layer.candidates = (0..n)
            .filter(|&k| {
                layer.allowed[k] && !layer.dead[k] && dmat[k] > r + g.c && dk[k] <= r + 3. * g.c
            })
            .collect();
        self.report.grid_refreshes += 1;
        left
    }

    /// Validate candidate `k` locally against the current grid: returns the
    /// unit direction towards the nearest reachable material.
    fn candidate_direction(&self, k: usize) -> Option<Point> {
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
    /// the unit direction to it.
    fn nearest_material(&self, p: Point, reach: f64) -> Option<(f64, Point)> {
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
            let f = |phi: f64, layer: &Layer| {
                let q = [p[0] + ds * (th + phi).cos(), p[1] + ds * (th + phi).sin()];
                if !layer.allowed_at(q) {
                    return f64::INFINITY;
                }
                area(&cells, q, r, c)
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
                let (a, contact) = step_load(&cells, q, u, r, c);
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
        let w = Grid {
            x0: g.x0 + i0 as f64 * c,
            y0: g.y0 + j0 as f64 * c,
            c,
            nx: i1 - i0 + 1,
            ny: j1 - j0 + 1,
        };
        let n = w.nx * w.ny;
        let global = |k: usize| g.idx(i0 + k % w.nx, j0 + k / w.nx);
        let mat: Vec<bool> = (0..n).map(|k| layer.material(global(k))).collect();
        let dmat = edt(&w, &mat);
        // Material beyond the window is unseen: its border ring is closed.
        let ring = ((r + c) / c).ceil() as usize + 1;
        let passable: Vec<bool> = (0..n)
            .map(|k| {
                let (i, j) = (k % w.nx, k / w.nx);
                let border = (i < ring && i0 > 0)
                    || (j < ring && j0 > 0)
                    || (i + ring >= w.nx && i1 + 1 < g.nx)
                    || (j + ring >= w.ny && j1 + 1 < g.ny);
                !border && layer.allowed[global(k)] && dmat[k] >= r - 0.5 * self.width
            })
            .collect();
        let start = w.idx(ia - i0, ja - j0);
        let goal = w.idx(ib - i0, jb - j0);
        if !passable[goal] {
            return None;
        }
        // Dijkstra on the window (the start may still touch the front).
        let mut dist = vec![f64::INFINITY; n];
        let mut prev = vec![usize::MAX; n];
        let mut heap = std::collections::BinaryHeap::new();
        dist[start] = 0.;
        heap.push((std::cmp::Reverse(0u64), start));
        let limit = max_len / c;
        while let Some((std::cmp::Reverse(d), k)) = heap.pop() {
            let d = d as f64 / 1024.;
            if d > dist[k] + 1e-9 {
                continue;
            }
            if k == goal || d > limit {
                break;
            }
            let (i, j) = ((k % w.nx) as i64, (k / w.nx) as i64);
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
                if ni < 0 || nj < 0 || ni >= w.nx as i64 || nj >= w.ny as i64 {
                    continue;
                }
                let nk = w.idx(ni as usize, nj as usize);
                let near_start = d < 2. * self.width / c;
                if !passable[nk] && !(near_start && dmat[nk] >= r - self.width) {
                    continue;
                }
                let nd = d + step;
                if nd < dist[nk] {
                    dist[nk] = nd;
                    prev[nk] = k;
                    heap.push((std::cmp::Reverse((nd * 1024.) as u64), nk));
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
            let mut new = 0.;
            for q in &cells {
                if (q[0] - prev[0]).hypot(q[1] - prev[1]) > r {
                    new += ((r - (q[0] - p[0]).hypot(q[1] - p[1])) / c + 0.5).clamp(0., 1.);
                }
            }
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
        (height < self.op.top_z_mm - 0.01_f64.max(64. * self.op.tolerance_mm)).then_some(height)
    }
    /// Rapid from `from` up to the retract height, across to `to`, and down
    /// to the feed height (or the proven rapid-descent stop below it), never
    /// below `at_least`: the caller's next motion starts there, so the rapid
    /// ends at or above it and every endpoint connects.
    fn travel(&mut self, from: Vec3, to: Point, proven: Option<f64>, at_least: f64) -> Vec3 {
        let up = if (from[2] - self.heights.retract).abs() < 1e-12 || from[2] >= self.heights.feed {
            let up = [from[0], from[1], from[2].max(self.heights.retract)];
            self.line(from, up, MoveKind::Rapid);
            up
        } else {
            self.lift(from, self.heights.retract)
        };
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
        let guard = 0.01_f64.max(64. * self.op.tolerance_mm);
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
            loop {
                let layer = self.layer.as_ref().unwrap();
                let best = layer
                    .candidates
                    .iter()
                    .enumerate()
                    .map(|(i, &k)| {
                        let q = layer.g.center(k);
                        (i, k, (q[0] - pos[0]).hypot(q[1] - pos[1]))
                    })
                    .min_by(|a, b| a.2.total_cmp(&b.2));
                let Some((i, k, _)) = best else { break };
                self.layer.as_mut().unwrap().candidates.swap_remove(i);
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
                if height - z > 0.01_f64.max(64. * self.op.tolerance_mm) {
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
                let above = self.travel(here, p, proven_p, z);
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
                let above = self.travel(here, q, proven, z);
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
        let entry_feed = self
            .entry
            .helix()
            .feed_mm_min
            .unwrap_or(self.op.plunge_mm_min);
        // `halved[i]`: stroke `from + i` continues the straight stroke before
        // it, which the proof halved.
        let mut halved = vec![false; self.pending.len().saturating_sub(from)];
        let mut next = from;
        while next < self.pending.len() {
            let index = next;
            next += 1;
            let (curve, kind) = (self.pending[index].0.clone(), self.pending[index].1);
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
                let report = scene.check_leading(
                    &motion,
                    self.r,
                    limit,
                    EngagementOptions::default(),
                    cancel,
                )?;
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
    /// Each is entered from stock this level has cleared: a point a few grid
    /// cells inside the loop whose whole column the committed cuts prove
    /// clear, then a straight lead onto the loop. Every stroke is certified
    /// against the region and proven within the engagement limit like any
    /// other; in an inside corner, where the skin is as thick as the corner
    /// is deep, an unproven stroke is entry motion (see `confined_strokes`).
    fn plan_walls(&mut self, stock: &Stock, cancel: &impl CancelSignal) -> Result<(), String> {
        let z = self.z;
        let c = self.layer.as_ref().unwrap().g.c;
        let first = self.guides.report().raster_strokes;
        let strokes: Vec<Stroke> = self.guides.guide_strokes()[first..].to_vec();
        let mut loops: Vec<Vec<Stroke>> = vec![];
        for s in strokes {
            match loops.last_mut() {
                Some(l) if l.last().is_some_and(|p| p.b == s.a) => l.push(s),
                _ => loops.push(vec![s]),
            }
        }
        for wall in loops {
            if cancel.is_cancelled() {
                return Err("engagement_front planning cancelled".into());
            }
            if wall.first().map(|s| s.a) != wall.last().map(|s| s.b) {
                continue;
            }
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
            let mut entry = None;
            'search: for k in order {
                let (first, second) = halves(wall[k]);
                let t = second.tangent(false);
                for side in [1., -1.] {
                    // A quarter-arc lead-in tangent to the wall, as a contour
                    // pass is entered: the skin comes into the cut gradually
                    // and stays ahead of the cutter.
                    let lead = 4. * c;
                    let centre = [
                        second.a[0] - side * lead * t[1],
                        second.a[1] + side * lead * t[0],
                    ];
                    let q = [centre[0] - lead * t[0], centre[1] - lead * t[1]];
                    let arc = Stroke {
                        a: q,
                        b: second.a,
                        arc: Some((centre, side * std::f64::consts::FRAC_PI_2)),
                    };
                    if !self.guides.stroke_geometry_fits_radius(arc, self.r) {
                        continue;
                    }
                    let (height, _) =
                        self.guides
                            .cleared_entry_start(q, 0., z, Some(stock), cancel)?;
                    if height - z <= 0.01_f64.max(64. * self.op.tolerance_mm) {
                        entry = Some((k, arc, height, first, second));
                        break 'search;
                    }
                }
            }
            let Some((k, lead_in, height, first, second)) = entry else {
                self.report.wall_passes_skipped += 1;
                continue;
            };
            let q = lead_in.a;
            let from = self.pending.len();
            let here = self.position;
            let proven = self
                .proven(height)
                .map(|h| h.max(self.guides.cell_entry_height(stock, q)));
            let above = self.travel(here, q, proven, z);
            self.line(above, [q[0], q[1], z], MoveKind::Plunge);
            let start = second.a;
            self.certify(lead_in)?;
            self.report.arcs += 1;
            self.queue(lead_in.curve(z), MoveKind::Cut, None);
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
                    self.pending.truncate(from);
                    self.report.wall_passes_skipped += 1;
                    continue;
                }
                self.prove(stock, from, from, true, cancel)?;
            }
            self.position = [start[0], start[1], z];
            self.report.lifts += 1;
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
        let mut best: Option<(f64, usize)> = None;
        for k in 0..layer.top.len() {
            if layer.allowed[k] && !layer.dead[k] && layer.material(k) {
                let room = layer.room[k] as f64 - need;
                if best.is_none_or(|(b, _)| room > b) {
                    best = Some((room, k));
                }
            }
        }
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
                }
            }
            layer.candidates.push(g.idx(i, j));
            revived = true;
        }
        revived
    }

    /// Enclosed material: helix down at the most open reachable point.
    fn helix_pass(&mut self, stock: &Stock, cancel: &impl CancelSignal) -> Result<bool, String> {
        let r = self.r;
        let z = self.z;
        loop {
            if cancel.is_cancelled() {
                return Err("engagement_front planning cancelled".into());
            }
            let layer = self.layer.as_ref().unwrap();
            let c = layer.g.c;
            let need = self.grid_clearance(c);
            let mut best: Option<(f64, usize)> = None;
            for k in 0..layer.top.len() {
                if layer.allowed[k] && !layer.dead[k] && layer.material(k) {
                    let room = layer.room[k] as f64 - need;
                    if best.is_none_or(|(b, _)| room > b) {
                        best = Some((room, k));
                    }
                }
            }
            let Some((room, k)) = best else {
                return Ok(false);
            };
            let q = layer.g.center(k);
            let helix = self.entry.helix().clone();
            let radii = helix.radii(r, r / 2., 0.);
            let Some(rh) = radii.into_iter().find(|&h| h <= room - c && h > 4. * c) else {
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
            let air = top <= z + 64. * self.op.tolerance_mm;
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
            let above = self.travel(here, [start[0], start[1]], proven, start[2]);
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
            }
            if !self.plan_pass(stock, cancel)? {
                if self.wall_pass && !self.walls_done {
                    self.walls_done = true;
                    self.plan_walls(stock, cancel)?;
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
            allowed: vec![true; n],
            reachable: vec![true; n],
            room: vec![100.; n],
            dead: vec![false; n],
            candidates: vec![],
            thin: vec![],
            z: 0.,
            material_guard: 1e-4,
            journal: vec![],
        };
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
        let layer = Layer {
            g,
            top: vec![10.; n],
            cleared: (0..n)
                .map(|k| if reachable[k] { 0. } else { f64::INFINITY })
                .collect(),
            allowed: (0..n).map(|k| y(k) <= 5.).collect(),
            reachable,
            room: vec![100.; n],
            dead: vec![false; n],
            candidates: vec![],
            thin: vec![],
            z: 0.,
            material_guard: 1e-4,
            journal: vec![],
        };
        let (p, q) = ([8., 4.9], [8.9, 4.9]);
        assert!(layer.allowed_at(p) && layer.allowed_at(q));
        // The part's material is there, inside the cutter's smoothing band.
        assert!((0..n).any(|k| layer.material(k) && area(&[g.center(k)], q, r, g.c) > 0.));
        let mut cells = vec![];
        layer.gather(p, r + g.c, &mut cells);
        assert_eq!(area(&cells, q, r, g.c), 0.);
    }
}
