//! Conservative removal of a swept tool assembly from tri-dexel stock, for a
//! fixed tool axis along a signed stock axis.
//!
//! A move is split into straight chords exactly as the height field splits it
//! (`stock::segments`: a straight line is one chord unless it climbs; arcs
//! and helices within the chord error). Every pose of the true path lies
//! within the chord's error `e` of a pose on the chord. In the tool's frame
//! (local z = tool axis, the tip at h = 0), for each chord and ray:
//!
//! - **Rays along the tool axis** (columns): with `d` the distance from the
//!   column's centre line to the chord, `r` its cross-section's
//!   half-diagonal, the cutting part may touch the column's cross-section
//!   only at heights `h` where its radius reaches `d - r - e`, and certainly
//!   covers it where its radius reaches `d + r + e` at one pose of the chord.
//!   This is the height-field kernel's bound per cell (`Stock::preview_sweep`),
//!   kept as an interval instead of "everything above".
//! - **Rays across the tool axis**: at the ray's height window (its
//!   cross-section's extent along the axis, widened by the chord's climb and
//!   `e`), the tool's slice is a disk whose radius lies between the profile's
//!   minimum and maximum over that window. The swept disk is a stadium around
//!   the chord, so the ray's interval is the stadium's extent across the
//!   ray's cross-section (outer: the stadium of the largest radius plus `e`
//!   meets the cross-section; inner: the stadium of the smallest radius less
//!   `e` contains both edges of the cross-section, hence all of it, the
//!   stadium being convex).
//!
//! Then certain material loses every outer interval and possible material
//! every inner one. The neck, shank and holder never cut: a move whose
//! non-cutting sweep may meet possible material (before the move) is
//! refused, unless one grid proves that sweep empty (or the height view's
//! native proof that committed cutter sweeps hold it,
//! `Stock::history_clearance`).
use super::{DexelDelta, DexelGrid, DexelStock, RayChange, Spans, intersect, normalize, subtract};
use crate::{
    motion::{Curve, Motion, Plane, Vec3},
    stock::{CancelSignal, SweepOptions, cancelled, segments},
    tool::{CutterShape, PreparedAssembly, valid_id},
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// A fixed tool axis along a signed stock axis: the direction from the tool
/// tip toward the spindle. Runs use `PosZ` (three-axis milling); the other
/// axes are for indexed machining once orientations can be declared (#2, #5).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Axis {
    PosX,
    NegX,
    PosY,
    NegY,
    PosZ,
    NegZ,
}
impl Axis {
    /// A signed permutation onto the tool frame: local[i] = sign[i] *
    /// world[perm[i]], local z the tool axis.
    fn frame(self) -> ([usize; 3], [f64; 3]) {
        match self {
            Self::PosZ => ([0, 1, 2], [1., 1., 1.]),
            Self::NegZ => ([0, 1, 2], [1., -1., -1.]),
            Self::PosX => ([1, 2, 0], [1., 1., 1.]),
            Self::NegX => ([1, 2, 0], [1., -1., -1.]),
            Self::PosY => ([2, 0, 1], [1., 1., 1.]),
            Self::NegY => ([2, 0, 1], [1., -1., -1.]),
        }
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DexelSweepStats {
    pub segments: usize,
    pub visited_rays: u64,
    pub changed_rays: usize,
    /// Columns where possible material stands above the cutting part's top
    /// (an undercut: the cutting part passed under it).
    pub undercut_columns: u64,
    /// Rays checked against the neck, shank and holder in detail.
    pub clearance_rays: u64,
    pub max_path_error_mm: f64,
}

/// One linear piece of the non-cutting envelope, tip-relative.
#[derive(Clone, Copy)]
struct Piece {
    h: [f64; 2],
    r: [f64; 2],
}
impl Piece {
    fn radius_max(&self, h0: f64, h1: f64) -> Option<f64> {
        let (lo, hi) = (h0.max(self.h[0]), h1.min(self.h[1]));
        if lo > hi {
            return None;
        }
        let at = |h: f64| {
            let span = self.h[1] - self.h[0];
            let t = if span > 0. {
                ((h - self.h[0]) / span).clamp(0., 1.)
            } else {
                0.
            };
            self.r[0] + t * (self.r[1] - self.r[0])
        };
        Some(at(lo).max(at(hi)))
    }
}

/// The assembly's rotational envelope in the tool frame: a cutting part whose
/// radius widens from the tip to its widest section and may then narrow
/// toward the spindle (unimodal: every `CutterShape` but `Beam`; lollipop and
/// dovetail cutters narrow), then non-cutting pieces (neck, shank, holder
/// sections).
struct Profile<'a> {
    tool: &'a PreparedAssembly,
    radius: f64,
    cutting: f64,
    pieces: Vec<Piece>,
    reach: f64,
}
impl<'a> Profile<'a> {
    fn new(tool: &'a PreparedAssembly) -> Result<Self, String> {
        let def = tool.definition();
        let c = &def.cutter;
        if matches!(c.shape, CutterShape::Beam) {
            return Err(
                "dexel stock models milling cutters; plasma and laser runs use height-field stock"
                    .into(),
            );
        }
        let mut pieces = vec![];
        let mut h = c.cutting_length_mm;
        if let Some(n) = &c.neck {
            pieces.push(Piece {
                h: [h, h + n.length_mm],
                r: [n.diameter_mm / 2.; 2],
            });
            h += n.length_mm;
        }
        pieces.push(Piece {
            h: [h, def.stickout_mm],
            r: [c.shank_diameter_mm / 2.; 2],
        });
        for s in &def.holders {
            pieces.push(Piece {
                h: [s.z_min_mm, s.z_max_mm],
                r: [s.radius_min_mm, s.radius_max_mm],
            });
        }
        let reach = pieces.iter().map(|p| p.r[0].max(p.r[1])).fold(0., f64::max);
        Ok(Self {
            tool,
            radius: c.diameter_mm / 2.,
            cutting: c.cutting_length_mm,
            pieces,
            reach,
        })
    }
    /// Heights of the cutting part whose radius reaches `rho`.
    fn covered(&self, rho: f64) -> Option<[f64; 2]> {
        self.tool.cutter().heights_at_radius(rho.max(0.))
    }
    fn cut_radius(&self, h: f64) -> f64 {
        self.tool
            .cutter()
            .radius_at(h.clamp(0., self.cutting))
            .map_or(0., |s| s.radius_mm)
    }
    /// Largest cutting radius over heights [h0, h1]: at the widest section
    /// when the window holds it, else at the window's nearer end (unimodal).
    fn cut_max(&self, h0: f64, h1: f64) -> Option<f64> {
        let (lo, hi) = (h0.max(0.), h1.min(self.cutting));
        (lo <= hi).then(|| self.cut_radius(self.tool.cutter().widest_height().clamp(lo, hi)))
    }
    /// Smallest cutting radius over [h0, h1], when that window lies inside
    /// the cutting part: at one of its ends (unimodal).
    fn cut_min(&self, h0: f64, h1: f64) -> Option<f64> {
        (h0 >= 0. && h1 <= self.cutting).then(|| self.cut_radius(h0).min(self.cut_radius(h1)))
    }
}

/// Extent along x of a disk of radius `rho` around `c` within y ∈ [y0, y1].
fn disk_strip(c: [f64; 2], rho: f64, y0: f64, y1: f64) -> Option<[f64; 2]> {
    let dy = if c[1] < y0 {
        y0 - c[1]
    } else if c[1] > y1 {
        c[1] - y1
    } else {
        0.
    };
    (dy <= rho).then(|| {
        let half = ((rho - dy) * (rho + dy)).max(0.).sqrt();
        [c[0] - half, c[0] + half]
    })
}
/// Extent along x of a convex polygon within y ∈ [y0, y1].
fn polygon_strip(poly: [[f64; 2]; 4], y0: f64, y1: f64) -> Option<[f64; 2]> {
    let mut pts: Vec<[f64; 2]> = poly.to_vec();
    for (bound, above) in [(y0, true), (y1, false)] {
        let inside = |p: [f64; 2]| if above { p[1] >= bound } else { p[1] <= bound };
        let mut out = Vec::with_capacity(pts.len() + 2);
        for i in 0..pts.len() {
            let (p, q) = (pts[i], pts[(i + 1) % pts.len()]);
            if inside(p) {
                out.push(p);
            }
            if inside(p) != inside(q) {
                let t = (bound - p[1]) / (q[1] - p[1]);
                out.push([p[0] + t * (q[0] - p[0]), bound]);
            }
        }
        pts = out;
        if pts.is_empty() {
            return None;
        }
    }
    let lo = pts.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min);
    let hi = pts.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max);
    Some([lo, hi])
}
/// Extent along x of the stadium {p : dist(p, [a, b]) <= rho} within
/// y ∈ [y0, y1]. The stadium is convex (two end disks and the rectangle
/// between them), so its slice is one interval.
fn stadium_strip(a: [f64; 2], b: [f64; 2], rho: f64, y0: f64, y1: f64) -> Option<[f64; 2]> {
    let mut out: Option<[f64; 2]> = None;
    let mut add = |e: Option<[f64; 2]>| {
        if let Some(e) = e {
            out = Some(out.map_or(e, |o| [o[0].min(e[0]), o[1].max(e[1])]));
        }
    };
    add(disk_strip(a, rho, y0, y1));
    add(disk_strip(b, rho, y0, y1));
    let d = [b[0] - a[0], b[1] - a[1]];
    let length = d[0].hypot(d[1]);
    if length > 0. {
        let n = [-d[1] / length * rho, d[0] / length * rho];
        add(polygon_strip(
            [
                [a[0] + n[0], a[1] + n[1]],
                [b[0] + n[0], b[1] + n[1]],
                [b[0] - n[0], b[1] - n[1]],
                [a[0] - n[0], a[1] - n[1]],
            ],
            y0,
            y1,
        ));
    }
    out
}
/// The world interval of a local one (a free function: the closures borrow
/// the kernel mutably).
fn k_interval(k: &Kernel, i: usize, s: [f64; 2]) -> [f64; 2] {
    k.local_interval(i, s)
}
/// Whether material may lie within `s` along a ray: a quick test against the
/// span of its possible intervals, before any geometry.
fn spans_meet(p: &[[f64; 2]], s: [f64; 2]) -> bool {
    match (p.first(), p.last()) {
        (Some(first), Some(last)) => first[0] < s[1] && s[0] < last[1],
        _ => false,
    }
}
/// Distance from `c` to the segment [a, b], and the closest parameter.
fn segment_distance(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> (f64, f64) {
    let d = [b[0] - a[0], b[1] - a[1]];
    let l = d[0] * d[0] + d[1] * d[1];
    let t = if l > 0. {
        (((c[0] - a[0]) * d[0] + (c[1] - a[1]) * d[1]) / l).clamp(0., 1.)
    } else {
        0.
    };
    (
        (c[0] - (a[0] + t * d[0])).hypot(c[1] - (a[1] + t * d[1])),
        t,
    )
}

/// What a sweep collects per ray: outer (may touch) and inner (certainly
/// covers) intervals, in world coordinates along the ray.
#[derive(Default)]
struct Hits {
    outer: Vec<[f64; 2]>,
    inner: Vec<[f64; 2]>,
}

struct Kernel<'a> {
    stock: &'a DexelStock,
    perm: [usize; 3],
    sign: [f64; 3],
    guard: f64,
    visits: u64,
    max_visits: u64,
}
/// A ray as the kernel sees it: its grid and index, its centre on the local
/// cross axes (the ray's own local axis entry is unused) and half extents.
struct LocalRay {
    axis: usize,
    index: usize,
    center: Vec3,
    half: Vec3,
}
impl Kernel<'_> {
    fn local(&self, p: Vec3) -> Vec3 {
        std::array::from_fn(|i| self.sign[i] * p[self.perm[i]])
    }
    /// The local axis of world axis `a`.
    fn local_axis(&self, a: usize) -> usize {
        (0..3).find(|i| self.perm[*i] == a).unwrap()
    }
    /// World interval along world axis perm[i] of a local interval on i.
    fn world(&self, i: usize, s: [f64; 2]) -> [f64; 2] {
        if self.sign[i] > 0. { s } else { [-s[1], -s[0]] }
    }
    fn local_interval(&self, i: usize, s: [f64; 2]) -> [f64; 2] {
        self.world(i, s)
    }
    /// Visit the rays of world grid `a` whose cross-sections meet the local
    /// box `ranges` (one [lo, hi] per local axis; the ray axis' entry is
    /// ignored).
    fn rays(
        &mut self,
        a: usize,
        ranges: [[f64; 2]; 3],
        cancel: &impl CancelSignal,
        mut f: impl FnMut(&mut Self, LocalRay) -> Result<(), String>,
    ) -> Result<(), String> {
        let la = self.local_axis(a);
        let spec = &self.stock.spec;
        let grid = self.stock.grid;
        let [u, v] = DexelGrid::cross(a);
        let mut index_range = [[0usize; 2]; 3];
        for i in (0..3).filter(|i| *i != la) {
            let w = self.perm[i];
            let [lo, hi] = self.world(i, ranges[i]);
            if hi < spec.min_mm[w] || lo > spec.max_mm[w] || lo > hi {
                return Ok(());
            }
            let cell = |x: f64| {
                ((x - spec.min_mm[w]) / grid.step_mm[w])
                    .floor()
                    .clamp(0., (grid.cells[w] - 1) as f64) as usize
            };
            index_range[w] = [cell(lo), cell(hi)];
        }
        for iv in index_range[v][0]..=index_range[v][1] {
            cancelled(cancel)?;
            for iu in index_range[u][0]..=index_range[u][1] {
                self.visits += 1;
                if self.visits.is_multiple_of(1024) {
                    cancelled(cancel)?;
                }
                if self.visits > self.max_visits {
                    return Err("dexel sweep ray-visit budget exceeded".into());
                }
                let mut center = [0.; 3];
                let mut half = [0.; 3];
                for (w, iw) in [(u, iu), (v, iv)] {
                    let i = self.local_axis(w);
                    let c = spec.min_mm[w] + (iw as f64 + 0.5) * grid.step_mm[w];
                    center[i] = self.sign[i] * c;
                    half[i] = grid.step_mm[w] / 2.;
                }
                f(
                    self,
                    LocalRay {
                        axis: a,
                        index: iv * grid.cells[u] + iu,
                        center,
                        half,
                    },
                )?;
            }
        }
        Ok(())
    }
    fn possible(&self, ray: &LocalRay) -> &[[f64; 2]] {
        self.stock.rays[ray.axis][ray.index].possible.as_slice()
    }
}

/// A chord in the tool frame.
struct Chord {
    a: Vec3,
    b: Vec3,
    e: f64,
}

impl DexelStock {
    /// The stock change of `fraction` of `motion` swept by `tool` along +Z
    /// (three-axis milling). Nothing changes until the delta is applied.
    pub fn preview_sweep(
        &self,
        tool: &PreparedAssembly,
        motion: &Motion,
        fraction: f64,
        options: SweepOptions,
        cancel: &impl CancelSignal,
    ) -> Result<DexelDelta, String> {
        self.preview_sweep_axis(tool, motion, fraction, Axis::PosZ, options, cancel)
    }

    /// As `preview_sweep`, with the tool axis along `axis`.
    pub fn preview_sweep_axis(
        &self,
        tool: &PreparedAssembly,
        motion: &Motion,
        fraction: f64,
        axis: Axis,
        options: SweepOptions,
        cancel: &impl CancelSignal,
    ) -> Result<DexelDelta, String> {
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
        let profile = Profile::new(tool)?;
        let mut stats = DexelSweepStats::default();
        let empty = |stats: DexelSweepStats| DexelDelta {
            base_generation: self.generation,
            motion_id: motion.id,
            operation: motion.operation.clone(),
            fraction,
            changes: vec![],
            view: self.view.derived_delta(
                tool,
                motion,
                fraction,
                vec![],
                false,
                false,
                0,
                options.tolerance_mm,
            ),
            stats,
        };
        if fraction == 0. || motion.kind.is_rapid() {
            return Ok(empty(stats));
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
        let (perm, sign) = axis.frame();
        let mut kernel = Kernel {
            stock: self,
            perm,
            sign,
            guard,
            visits: 0,
            max_visits: options.max_cell_visits,
        };
        let chords: Vec<Chord> = if let Curve::Line { start, end } = motion.curve {
            // A line climbs only along the tool axis: chords at most
            // `max_tip_step_mm` apart along it (the height field's layout
            // for +Z, in the tool's frame for every axis).
            let (a, b) = (kernel.local(start), kernel.local(end));
            let count = ((b[2] - a[2]).abs() / options.max_tip_step_mm)
                .ceil()
                .max(1.) as usize;
            if count > options.max_segments {
                return Err("sweep segment budget exceeded".into());
            }
            let mut chords = vec![];
            for i in 0..count {
                let from = i as f64 / count as f64;
                if from >= fraction {
                    break;
                }
                let to = ((i + 1) as f64 / count as f64).min(fraction);
                chords.push(Chord {
                    a: kernel.local(prepared.position(from)?),
                    b: kernel.local(prepared.position(to)?),
                    e: guard,
                });
            }
            chords
        } else {
            segments(&motion.curve, &prepared, fraction, options, guard, cancel)?
                .into_iter()
                .map(|s| Chord {
                    a: kernel.local(s.a),
                    b: kernel.local(s.b),
                    e: s.error,
                })
                .collect()
        };
        stats.segments = chords.len();
        stats.max_path_error_mm = chords.iter().map(|c| c.e).fold(0., f64::max);
        // The lowest tip of the whole motion in the tool frame (exact, from
        // the curve's bounds), as the height field's reach admission uses.
        let lb = [kernel.local(bounds.min), kernel.local(bounds.max)];
        let z_min = lb[0][2].min(lb[1][2]);
        let reach_height = z_min + profile.cutting - guard;
        let parallel = perm[2];
        // Columns (rays along the tool axis) with possible material above
        // the cutting part's top at the lowest tip: undercut, or the
        // non-cutting part may meet material.
        let pad = profile.reach.max(profile.radius) + stats.max_path_error_mm;
        let local_top = |k: &Kernel, ray: &LocalRay| -> Option<f64> {
            let p = k.possible(ray);
            if sign[2] > 0. {
                p.last().map(|s| s[1])
            } else {
                p.first().map(|s| -s[0])
            }
        };
        let mut high = 0u64;
        let mut undercut = 0u64;
        for c in &chords {
            let ranges = [
                [c.a[0].min(c.b[0]) - pad, c.a[0].max(c.b[0]) + pad],
                [c.a[1].min(c.b[1]) - pad, c.a[1].max(c.b[1]) + pad],
                [0., 0.],
            ];
            kernel.rays(parallel, ranges, cancel, |k, ray| {
                if local_top(k, &ray).is_some_and(|t| t > reach_height) {
                    high += 1;
                    let r = ray.half[0].hypot(ray.half[1]);
                    let (d, _) = segment_distance(
                        [c.a[0], c.a[1]],
                        [c.b[0], c.b[1]],
                        [ray.center[0], ray.center[1]],
                    );
                    if d - r - c.e <= profile.radius {
                        undercut += 1;
                    }
                }
                Ok(())
            })?;
        }
        stats.undercut_columns = undercut;
        if high > 0 {
            self.prove_clearance(&mut kernel, &profile, &chords, motion, &mut stats, cancel)?;
        }
        // Cutting removal.
        let mut hits: [BTreeMap<usize, Hits>; 3] = Default::default();
        let analytic = axis == Axis::PosZ
            && matches!(tool.cutter().definition().shape, CutterShape::Flat)
            && motion.curve.start()[2] == motion.curve.end()[2]
            && matches!(
                motion.curve,
                Curve::Line { .. }
                    | Curve::Arc {
                        plane: Plane::Xy,
                        ..
                    }
            );
        if analytic {
            self.columns_analytic(
                &mut kernel,
                &profile,
                motion,
                &prepared,
                fraction,
                options,
                &mut hits[2],
                cancel,
            )?;
        }
        for c in &chords {
            for (a, h) in hits.iter_mut().enumerate() {
                if a == parallel {
                    if !analytic {
                        Self::columns(&mut kernel, &profile, c, a, h, cancel)?;
                    }
                } else {
                    let local = kernel.local_axis(a);
                    Self::across(&mut kernel, &profile, c, a, local, h, cancel)?;
                }
            }
        }
        stats.visited_rays = kernel.visits;
        let mut changes = vec![];
        for (a, map) in hits.into_iter().enumerate() {
            for (index, h) in map {
                cancelled(cancel)?;
                let before = &self.rays[a][index];
                let outer = normalize(h.outer);
                let inner = normalize(h.inner);
                let possible = subtract(before.possible.as_slice(), &inner);
                let certain = subtract(&subtract(before.certain.as_slice(), &outer), &inner);
                let certain = intersect(&certain, &possible);
                if possible.as_slice() == before.possible.as_slice()
                    && certain.as_slice() == before.certain.as_slice()
                {
                    continue;
                }
                if changes.len() >= options.max_changed_cells {
                    return Err("sweep changed-ray budget exceeded".into());
                }
                changes.push(RayChange {
                    axis: a as u8,
                    index: index as u32,
                    before: before.clone(),
                    after: super::Ray {
                        possible: Spans::from_vec(possible),
                        certain: Spans::from_vec(certain),
                    },
                });
            }
        }
        stats.changed_rays = changes.len();
        let (view_changes, _) = self.validate(&changes)?;
        // A cutter that narrows toward the spindle leaves material above its
        // widest section in its footprint: never a height-field move.
        let buried =
            undercut > 0 || axis != Axis::PosZ || tool.cutter().definition().shape.undercuts();
        let view = self.view.derived_delta(
            tool,
            motion,
            fraction,
            view_changes,
            !buried,
            analytic,
            u64::from(buried).max(undercut),
            options.tolerance_mm,
        );
        cancelled(cancel)?;
        Ok(DexelDelta {
            base_generation: self.generation,
            motion_id: motion.id,
            operation: motion.operation.clone(),
            fraction,
            changes,
            view,
            stats,
        })
    }

    /// The neck, shank and holder must not meet possible material: proven
    /// when one grid shows their sweep empty, or by the height view's native
    /// proof that the same path was cleared before.
    fn prove_clearance(
        &self,
        kernel: &mut Kernel,
        profile: &Profile,
        chords: &[Chord],
        motion: &Motion,
        stats: &mut DexelSweepStats,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        let before = kernel.visits;
        let clear = Self::grid_clear(kernel, &profile.pieces, chords, cancel)?;
        stats.clearance_rays = kernel.visits - before;
        if clear {
            return Ok(());
        }
        if kernel.perm == [0, 1, 2] && kernel.sign[2] > 0. {
            let mut work = kernel.max_visits.saturating_sub(kernel.visits);
            if self.view.history_clearance(
                motion,
                profile.radius,
                profile.cutting,
                &mut work,
                cancel,
            )? {
                return Ok(());
            }
        }
        Err(format!(
            "the tool's neck, shank or holder may touch stock material in motion {} ({}): no dexel grid proves the non-cutting part of the assembly clear of possible material; clear the material first, retract above it, or use a tool whose cutting length reaches it",
            motion.id, motion.operation
        ))
    }

    /// Whether one grid proves the sweep of the envelope `pieces` (tip
    /// relative) along the chords clear of possible material.
    fn grid_clear(
        kernel: &mut Kernel,
        pieces: &[Piece],
        chords: &[Chord],
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        let reach = pieces.iter().map(|p| p.r[0].max(p.r[1])).fold(0., f64::max);
        let (bottom, top) = (
            pieces.iter().map(|p| p.h[0]).fold(f64::INFINITY, f64::min),
            pieces
                .iter()
                .map(|p| p.h[1])
                .fold(f64::NEG_INFINITY, f64::max),
        );
        for a in 0..3 {
            let mut hit = false;
            let la = kernel.local_axis(a);
            for c in chords {
                let pad = reach + c.e;
                let (z_lo, z_hi) = (c.a[2].min(c.b[2]), c.a[2].max(c.b[2]));
                let mut ranges = [
                    [c.a[0].min(c.b[0]) - pad, c.a[0].max(c.b[0]) + pad],
                    [c.a[1].min(c.b[1]) - pad, c.a[1].max(c.b[1]) + pad],
                    [z_lo + bottom - c.e, z_hi + top + c.e],
                ];
                ranges[la] = [0., 0.];
                kernel.rays(a, ranges, cancel, |k, ray| {
                    if hit {
                        return Ok(());
                    }
                    let p = k.possible(&ray);
                    if p.is_empty() {
                        return Ok(());
                    }
                    for s in envelope_outer(k, pieces, c, &ray, la) {
                        if super::overlaps(p, &[s]) {
                            hit = true;
                        }
                    }
                    Ok(())
                })?;
                if hit {
                    break;
                }
            }
            if !hit {
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// Whether the rays prove the whole assembly (cutting part as its
    /// enclosing cylinder, neck, shank and holder) clear of possible material
    /// along `fraction` of `motion` (tool axis +Z): a rapid or link that
    /// touches no stock. `false` means not proven, never a contact.
    pub fn proves_clear(
        &self,
        tool: &PreparedAssembly,
        motion: &Motion,
        options: SweepOptions,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        options.validate()?;
        let prepared = motion.curve.prepare(options.tolerance_mm)?;
        let profile = Profile::new(tool)?;
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
        let (perm, sign) = Axis::PosZ.frame();
        let mut kernel = Kernel {
            stock: self,
            perm,
            sign,
            guard,
            visits: 0,
            max_visits: options.max_cell_visits,
        };
        let chords: Vec<Chord> = segments(&motion.curve, &prepared, 1., options, guard, cancel)?
            .into_iter()
            .map(|s| Chord {
                a: s.a,
                b: s.b,
                e: s.error,
            })
            .collect();
        let mut pieces = vec![Piece {
            h: [0., profile.cutting],
            r: [profile.radius; 2],
        }];
        pieces.extend_from_slice(&profile.pieces);
        Self::grid_clear(&mut kernel, &pieces, &chords, cancel)
    }

    /// Columns for a flat end mill at constant height along a line or an XY
    /// arc: the exact distance to the whole path, as the height field's
    /// analytic kernel uses.
    #[allow(clippy::too_many_arguments)]
    fn columns_analytic(
        &self,
        kernel: &mut Kernel,
        profile: &Profile,
        motion: &Motion,
        prepared: &crate::motion::PreparedCurve,
        fraction: f64,
        options: SweepOptions,
        hits: &mut BTreeMap<usize, Hits>,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        let guard = kernel.guard;
        let raw = crate::verification::path_error(motion, options.tolerance_mm) + guard;
        let mut error = (raw / guard).ceil() * guard;
        if error < raw {
            error += guard;
        }
        let axis = crate::verification::AxisSpan::new(motion, prepared, [0., fraction])
            .ok_or("unsupported analytic stock axis")?;
        let b = prepared.bounds();
        let z = motion.curve.start()[2];
        let pad = profile.radius + error;
        let ranges = [
            [b.min[0] - pad, b.max[0] + pad],
            [b.min[1] - pad, b.max[1] + pad],
            [0., 0.],
        ];
        let reach = k_interval(kernel, 2, [z - guard, z + profile.cutting + guard]);
        kernel.rays(2, ranges, cancel, |k, ray| {
            if !spans_meet(k.possible(&ray), reach) {
                return Ok(());
            }
            let r = ray.half[0].hypot(ray.half[1]);
            let d = axis.distance([ray.center[0], ray.center[1]]);
            if d - r - error > profile.radius {
                return Ok(());
            }
            let outer = [z - guard, z + profile.cutting + guard];
            if !super::overlaps(k.possible(&ray), &[outer]) {
                return Ok(());
            }
            let h = hits.entry(ray.index).or_default();
            h.outer.push(outer);
            if d + r + error <= profile.radius {
                h.inner.push([z + guard, z + profile.cutting - guard]);
            }
            Ok(())
        })
    }

    /// Rays along the tool axis.
    fn columns(
        kernel: &mut Kernel,
        profile: &Profile,
        c: &Chord,
        a: usize,
        hits: &mut BTreeMap<usize, Hits>,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        let pad = profile.radius + c.e;
        let ranges = [
            [c.a[0].min(c.b[0]) - pad, c.a[0].max(c.b[0]) + pad],
            [c.a[1].min(c.b[1]) - pad, c.a[1].max(c.b[1]) + pad],
            [0., 0.],
        ];
        let (z_lo, z_hi) = (c.a[2].min(c.b[2]), c.a[2].max(c.b[2]));
        let (a2, b2) = ([c.a[0], c.a[1]], [c.b[0], c.b[1]]);
        let vertical = a2 == b2;
        // A cheap superset of every outer interval along these columns.
        let reach = k_interval(kernel, 2, [z_lo - c.e, z_hi + profile.cutting + c.e]);
        kernel.rays(a, ranges, cancel, |k, ray| {
            if !spans_meet(k.possible(&ray), reach) {
                return Ok(());
            }
            let r = ray.half[0].hypot(ray.half[1]);
            let centre = [ray.center[0], ray.center[1]];
            let (d, t) = segment_distance(a2, b2, centre);
            let Some(h) = profile.covered(d - r - c.e) else {
                return Ok(());
            };
            let outer = k.local_interval(2, [z_lo + h[0] - c.e, z_hi + h[1] + c.e]);
            if !super::overlaps(k.possible(&ray), &[outer]) {
                return Ok(());
            }
            let mut inner = vec![];
            if vertical {
                if let Some(h) = profile.covered(d + r + c.e) {
                    inner.push([z_lo + h[0] + c.e, z_hi + h[1] - c.e]);
                }
            } else {
                for s in [t, 0., 1.] {
                    let p = [
                        c.a[0] + s * (c.b[0] - c.a[0]),
                        c.a[1] + s * (c.b[1] - c.a[1]),
                    ];
                    let z = c.a[2] + s * (c.b[2] - c.a[2]);
                    let d = (centre[0] - p[0]).hypot(centre[1] - p[1]);
                    if let Some(h) = profile.covered(d + r + c.e) {
                        inner.push([z + h[0] + c.e, z + h[1] - c.e]);
                    }
                }
            }
            let entry = hits.entry(ray.index).or_default();
            entry.outer.push(outer);
            entry
                .inner
                .extend(inner.into_iter().map(|s| k.local_interval(2, s)));
            Ok(())
        })
    }

    /// Rays across the tool axis (along local axis `la`, 0 or 1).
    fn across(
        kernel: &mut Kernel,
        profile: &Profile,
        c: &Chord,
        a: usize,
        la: usize,
        hits: &mut BTreeMap<usize, Hits>,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        let o = 1 - la;
        let (z_lo, z_hi) = (c.a[2].min(c.b[2]), c.a[2].max(c.b[2]));
        let pad = profile.radius + c.e;
        let mut ranges = [[0.; 2]; 3];
        ranges[o] = [c.a[o].min(c.b[o]) - pad, c.a[o].max(c.b[o]) + pad];
        ranges[2] = [z_lo - c.e, z_hi + profile.cutting + c.e];
        let (a2, b2) = ([c.a[la], c.a[o]], [c.b[la], c.b[o]]);
        let guard = kernel.guard;
        // A cheap superset of every outer interval along these rays.
        let reach = k_interval(
            kernel,
            la,
            [
                a2[0].min(b2[0]) - pad - guard,
                a2[0].max(b2[0]) + pad + guard,
            ],
        );
        kernel.rays(a, ranges, cancel, |k, ray| {
            if !spans_meet(k.possible(&ray), reach) {
                return Ok(());
            }
            let (yc, hy) = (ray.center[o], ray.half[o]);
            let (zc, hz) = (ray.center[2], ray.half[2]);
            let w = [zc - hz - z_hi - c.e, zc + hz - z_lo + c.e];
            let Some(rmax) = profile.cut_max(w[0], w[1]) else {
                return Ok(());
            };
            let Some(outer) = stadium_strip(a2, b2, rmax + c.e, yc - hy, yc + hy) else {
                return Ok(());
            };
            let outer = k.local_interval(la, [outer[0] - guard, outer[1] + guard]);
            if !super::overlaps(k.possible(&ray), &[outer]) {
                return Ok(());
            }
            let mut inner = None;
            if let Some(rmin) = profile.cut_min(w[0], w[1]) {
                let rho = rmin - c.e;
                if rho > 0.
                    && let (Some(lo), Some(hi)) = (
                        stadium_strip(a2, b2, rho, yc - hy, yc - hy),
                        stadium_strip(a2, b2, rho, yc + hy, yc + hy),
                    )
                {
                    let s = [lo[0].max(hi[0]) + guard, lo[1].min(hi[1]) - guard];
                    if s[0] < s[1] {
                        inner = Some(k.local_interval(la, s));
                    }
                }
            }
            let entry = hits.entry(ray.index).or_default();
            entry.outer.push(outer);
            entry.inner.extend(inner);
            Ok(())
        })
    }
}

/// Where an envelope (tip-relative pieces) may meet a ray's cross-section
/// over a chord, in world coordinates along the ray.
fn envelope_outer(
    k: &Kernel,
    pieces: &[Piece],
    c: &Chord,
    ray: &LocalRay,
    la: usize,
) -> Vec<[f64; 2]> {
    let (z_lo, z_hi) = (c.a[2].min(c.b[2]), c.a[2].max(c.b[2]));
    let mut out = vec![];
    if la == 2 {
        let r = ray.half[0].hypot(ray.half[1]);
        let (d, _) = segment_distance(
            [c.a[0], c.a[1]],
            [c.b[0], c.b[1]],
            [ray.center[0], ray.center[1]],
        );
        let rho = d - r - c.e;
        for p in pieces {
            if p.r[0].max(p.r[1]) >= rho {
                out.push(k.local_interval(2, [z_lo + p.h[0] - c.e, z_hi + p.h[1] + c.e]));
            }
        }
    } else {
        let o = 1 - la;
        let (yc, hy) = (ray.center[o], ray.half[o]);
        let (zc, hz) = (ray.center[2], ray.half[2]);
        let w = [zc - hz - z_hi - c.e, zc + hz - z_lo + c.e];
        if let Some(rmax) = pieces
            .iter()
            .filter_map(|p| p.radius_max(w[0], w[1]))
            .reduce(f64::max)
            && let Some(s) = stadium_strip(
                [c.a[la], c.a[o]],
                [c.b[la], c.b[o]],
                rmax + c.e,
                yc - hy,
                yc + hy,
            )
        {
            out.push(k.local_interval(la, [s[0] - k.guard, s[1] + k.guard]));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stadium_slices() {
        // A disk of radius 1 at the origin (degenerate chord).
        let s = stadium_strip([0., 0.], [0., 0.], 1., -0.1, 0.1).unwrap();
        assert!((s[0] + 1.).abs() < 1e-12 && (s[1] - 1.).abs() < 1e-12);
        let s = stadium_strip([0., 0.], [0., 0.], 1., 0.6, 0.6).unwrap();
        assert!((s[1] - 0.8).abs() < 1e-12);
        assert!(stadium_strip([0., 0.], [0., 0.], 1., 1.1, 2.).is_none());
        // A chord along x: the slice spans both end caps.
        let s = stadium_strip([0., 0.], [5., 0.], 1., 0., 0.).unwrap();
        assert!((s[0] + 1.).abs() < 1e-12 && (s[1] - 6.).abs() < 1e-12);
        // A chord along y crossed by a strip: the rectangle's width.
        let s = stadium_strip([0., -5.], [0., 5.], 1., 2., 3.).unwrap();
        assert!((s[0] + 1.).abs() < 1e-12 && (s[1] - 1.).abs() < 1e-12);
        // A diagonal chord: points on the slice are within rho, its ends exactly.
        let (a, b, rho) = ([0., 0.], [4., 3.], 0.5);
        let s = stadium_strip(a, b, rho, 1., 1.).unwrap();
        for x in [s[0], s[1]] {
            let (d, _) = segment_distance(a, b, [x, 1.]);
            assert!((d - rho).abs() < 1e-12, "{d}");
        }
    }
}
