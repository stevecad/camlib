//! Facing round machining limits (#422): the plain facing's rows, each stopped
//! where the limits stop the cutter, linked at depth wherever a proven link
//! exists, then one climb pass along each limit's edge.
//!
//! This is direction-parallel (zigzag) area milling as established practice
//! describes it: the area is swept by parallel passes clipped to it, the pass
//! elements are linked so the tool retracts as seldom as possible, and a pass
//! along the boundary removes the scallops the clipped passes leave against it
//! (M. Held, "On the Computational Geometry of Pocket Machining", LNCS 500,
//! 1991, zigzag milling with a final contour pass; S. C. Park and B. K. Choi,
//! "Tool-path planning for direction-parallel area milling", Computer-Aided
//! Design 32, 2000, linking the pass elements to minimise retractions). The
//! boundary pass keeps the material on its right, which with a clockwise
//! spindle is climb (down) milling, the direction recommended for finishing
//! passes (Sandvik Coromant, Training Handbook, milling: "Climb milling").
//!
//! What is planned, per depth (each depth with the avoid areas whose tops
//! are above it, since an area only limits a tip below its top):
//! - **Cutter centres.** Every position from which the cutter can reach the
//!   stock (the stock grown by as much as the plain rows overrun it), less
//!   every avoid area grown by the cutter radius and the clearance region
//!   operations keep (`limits::REGION_CLEARANCE`), inside the containment
//!   shrunk by the same.
//! - **Rows.** The rows of the plain facing (same Y, same ends outside the
//!   stock), each cut into the stretches inside the cutter centres. A stretch
//!   along which the whole cutter stays clear of the stock cuts nothing and is
//!   left out: that is decided by the distance from the stretch to the stock
//!   rectangle (at least the cutter radius), never by a length threshold.
//! - **Links.** After a stretch the tool stays down for a stretch of a
//!   neighbouring row when a step across the rows joins them inside the cutter
//!   centres: it goes back along the row just cut to a point from which the
//!   step clears every limit, steps across, and takes the next row from its
//!   nearer end. A stretch that can also be entered from air is linked so only
//!   while the travel this adds to a plain zigzag link is no longer than the
//!   fed part of a retract and re-entry, the rule `PocketPlan` links by
//!   (retracting to the feed height and plunging back takes at least that
//!   long). Otherwise the tool lifts straight up.
//! - **Entries.** Every descent is vertical with the whole cutter
//!   `entry_air_gap_mm` clear of the stock, as in the plain facing. A stretch
//!   closed off at both ends is reached at depth from a neighbouring row; one
//!   nothing reaches is refused, never plunged into.
//! - **Edges.** After the rows, each run of the cutter-centre boundary that
//!   comes from a limit (not from the facing's own rectangle) and brings the
//!   cutter over the stock is followed once in its own direction. A row
//!   stretch that runs from air to such a run is cut on the way into it,
//!   rather than with an entry of its own. What then remains is what no
//!   allowed cutter position reaches, reported as `unreachable_area_mm2`.
//!
//! Nothing here is trusted for safety: every move is put through
//! `PreparedLimits::allows_curve`, the proof the run's link guard uses, before
//! it is planned, and a move that cannot be proven is not made.
use super::FacingPlan;
use crate::{
    Point,
    limits::{AvoidArea, Limits, PreparedLimits, REGION_CLEARANCE},
    motion::{Curve, MoveKind, Vec3},
    planar::{Boolean, Boundary, Component, ConstructionOptions, PlanarRegion, PreparedRegion},
    pocket::{Crossing, Stroke},
    stock::CancelSignal,
};
use serde::Serialize;
use std::collections::VecDeque;

/// What facing round limits planned, counted over every depth pass.
#[derive(Clone, Debug, Default, Serialize)]
pub struct AroundLimitsReport {
    /// Row stretches cut.
    pub row_stretches: usize,
    /// Row stretches left out because the whole cutter stays clear of the
    /// stock along them (they would cut nothing).
    pub air_stretches_omitted: usize,
    /// Passes along the limits' edges.
    pub edge_runs: usize,
    /// Runs along the limits' edges left out because the whole cutter stays
    /// clear of the stock along them.
    pub air_edge_runs_omitted: usize,
    /// Vertical entries from above the stock.
    pub entries: usize,
    /// Links made at depth instead of a retract and re-entry.
    pub stay_down_links: usize,
    /// Stock top area that no allowed cutter position reaches at the floor:
    /// what the limits leave standing.
    pub unreachable_area_mm2: f64,
}

/// Which of the facing report's counters a move adds to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Count {
    None,
    Row,
    StayDown,
    Return,
}
pub(super) type Step = (Curve, MoveKind, Count);

pub(super) struct Route {
    pub steps: VecDeque<Step>,
    pub report: AroundLimitsReport,
}

/// A point on a run: the piece it lies on and the point.
#[derive(Clone, Copy)]
struct Anchor {
    run: usize,
    piece: usize,
    at: Point,
}

/// One stretch of a row: its ends in X, whether a vertical entry is allowed
/// at each (the whole cutter `entry_air_gap_mm` clear of the stock), and for
/// an end at a limit the run it stops at.
struct Stretch {
    row: usize,
    y: f64,
    x: [f64; 2],
    air: [bool; 2],
    anchor: [Option<Anchor>; 2],
    done: bool,
}
impl Stretch {
    fn enclosed(&self) -> bool {
        !self.air[0] && !self.air[1]
    }
    /// A stretch that leads from air into a run: its end in air and where it
    /// stops at the run.
    fn lead(&self) -> Option<(usize, Anchor)> {
        match (self.air, self.anchor) {
            ([true, false], [_, Some(a)]) => Some((0, a)),
            ([false, true], [Some(a), _]) => Some((1, a)),
            _ => None,
        }
    }
}

/// A run of boundary pieces that come from a limit, in the boundary's own
/// direction (cutter centres on the left).
struct Run {
    pieces: Vec<Stroke>,
    closed: bool,
    /// Whether a vertical entry is allowed at its start and end (open runs).
    air: [bool; 2],
    done: bool,
}

/// The cutter centres, stretches and runs at one set of active limits.
struct Layout {
    centres: PreparedRegion,
    crossings: Vec<Crossing>,
    stretches: Vec<Stretch>,
    runs: Vec<Run>,
    air_stretches_omitted: usize,
    air_edge_runs_omitted: usize,
}

fn rectangle(name: String, lo: Point, hi: Point) -> PlanarRegion {
    PlanarRegion {
        version: 1,
        units: "mm".into(),
        name,
        boundary_error_mm: 0.,
        source: serde_json::Value::Null,
        components: vec![Component {
            outer: Boundary {
                vertices: vec![
                    [lo[0], lo[1], 0.],
                    [hi[0], lo[1], 0.],
                    [hi[0], hi[1], 0.],
                    [lo[0], hi[1], 0.],
                ],
            },
            holes: vec![],
        }],
    }
}

/// The two parts of `s` at the point `at` on it.
fn split(s: Stroke, at: Point) -> (Stroke, Stroke) {
    let Some((c, sweep)) = s.arc else {
        return (Stroke::line(s.a, at), Stroke::line(at, s.b));
    };
    let angle = |p: Point| metallic::atan2(p[1] - c[1], p[0] - c[0]);
    let mut d = angle(at) - angle(s.a);
    let turn = std::f64::consts::TAU;
    if sweep > 0. && d < 0. {
        d += turn;
    } else if sweep < 0. && d > 0. {
        d -= turn;
    }
    let part = d.clamp(sweep.min(0.), sweep.max(0.));
    (
        Stroke {
            a: s.a,
            b: at,
            arc: Some((c, part)),
        },
        Stroke {
            a: at,
            b: s.b,
            arc: Some((c, sweep - part)),
        },
    )
}

fn stroke_length(s: &Stroke) -> f64 {
    match s.arc {
        None => metallic::hypot(s.b[0] - s.a[0], s.b[1] - s.a[1]),
        Some((c, sweep)) => metallic::hypot(s.a[0] - c[0], s.a[1] - c[1]) * sweep.abs(),
    }
}

/// Distance from the segment `a`-`b` to the closed rectangle `lo`-`hi`.
fn segment_to_rectangle(a: Point, b: Point, lo: Point, hi: Point) -> f64 {
    let point = |p: Point| {
        metallic::hypot(
            (lo[0] - p[0]).max(p[0] - hi[0]).max(0.),
            (lo[1] - p[1]).max(p[1] - hi[1]).max(0.),
        )
    };
    let to_segment = |p: Point| {
        let d = [b[0] - a[0], b[1] - a[1]];
        let n = d[0] * d[0] + d[1] * d[1];
        let t = if n > 0. {
            (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / n).clamp(0., 1.)
        } else {
            0.
        };
        metallic::hypot(a[0] + t * d[0] - p[0], a[1] + t * d[1] - p[1])
    };
    // Crossing the rectangle: an edge of it separates the endpoints, or an
    // endpoint is inside.
    if point(a) == 0. || point(b) == 0. {
        return 0.;
    }
    let corners = [lo, [hi[0], lo[1]], hi, [lo[0], hi[1]]];
    let side = |p: Point, q: Point, r: Point| {
        (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    };
    for k in 0..4 {
        let (p, q) = (corners[k], corners[(k + 1) % 4]);
        if side(a, b, p) * side(a, b, q) <= 0. && side(p, q, a) * side(p, q, b) <= 0. {
            return 0.;
        }
    }
    corners
        .into_iter()
        .map(to_segment)
        .chain([point(a), point(b)])
        .fold(f64::INFINITY, f64::min)
}

struct Geometry<'a> {
    plan: &'a FacingPlan,
    radius: f64,
    stock_lo: Point,
    stock_hi: Point,
}
impl Geometry<'_> {
    /// A vertical entry is allowed here: the whole cutter at least
    /// `entry_air_gap_mm` clear of the stock, as the plain facing enters.
    fn air(&self, p: Point) -> bool {
        segment_to_rectangle(p, p, self.stock_lo, self.stock_hi)
            >= self.radius + self.plan.op.entry_air_gap_mm
    }
    /// The whole cutter stays clear of the stock along the segment: a move
    /// along it removes nothing.
    fn clear(&self, a: Point, b: Point) -> bool {
        segment_to_rectangle(a, b, self.stock_lo, self.stock_hi) >= self.radius
    }
}

/// The cutter centres, rows and runs with the limits `limits` active.
fn layout(
    geometry: &Geometry,
    limits: &Limits,
    cancel: &impl CancelSignal,
) -> Result<Layout, String> {
    let plan = geometry.plan;
    let op = &plan.op;
    let radius = geometry.radius;
    let tol = op.tolerance_mm;
    let options = ConstructionOptions { tolerance_mm: tol };
    let (low, high) = (plan.low, plan.high);
    let rows = plan.report.rows_per_pass;
    // Every cutter position from which the cutter can reach the stock: the
    // stock grown, across the rows as along them, by as much as the rows
    // overrun it (the radius, the air gap and the plain facing's guard), so
    // a run along a limit's edge goes as far as the cutter can cut beside
    // it and ends in air. The rows lie inside it.
    let overrun = high - geometry.stock_hi[0];
    let (bottom, top) = (
        geometry.stock_lo[1] - overrun,
        geometry.stock_hi[1] + overrun,
    );
    // How far cutter centres keep from a limit: the radius, the clearance
    // `Limits::restrict_region` keeps for region operations, and a margin
    // for this construction's own error.
    let keep = |region: &PlanarRegion| {
        radius + 2. * (region.boundary_error_mm + tol) + REGION_CLEARANCE.max(32. * tol) + 16. * tol
    };
    let mut centres = rectangle(
        format!("{} cutter centres", op.id),
        [low, bottom],
        [high, top],
    )
    .prepare(options)?;
    if let Some(contain) = &limits.contain {
        let inner = contain
            .prepare(options)?
            .offset_prepared(keep(contain))?
            .into_parts()
            .1;
        centres = centres
            .boolean_prepared(&inner, Boolean::Intersection)?
            .into_parts()
            .1;
    }
    for area in &limits.avoid {
        if cancel.is_cancelled() {
            return Err(super::PLANNING_CANCELLED.into());
        }
        let zone = area
            .region
            .prepare(options)?
            .offset_prepared(-keep(&area.region))?
            .into_parts()
            .1;
        centres = centres
            .boolean_prepared(&zone, Boolean::Difference)?
            .into_parts()
            .1;
    }
    let loops: Vec<Vec<Stroke>> = centres
        .definition()
        .components
        .iter()
        .flat_map(|c| std::iter::once(&c.outer).chain(&c.holes))
        .map(Stroke::from_boundary)
        .collect();
    // A piece of the facing's own rectangle (not from a limit).
    let edge = 4. * tol;
    let own = |s: &Stroke| {
        let on = |k: usize, v: f64| (s.a[k] - v).abs() <= edge && (s.b[k] - v).abs() <= edge;
        s.arc.is_none() && (on(0, low) || on(0, high) || on(1, bottom) || on(1, top))
    };
    // Conservative: the cutter may come over the stock somewhere along the
    // piece (its bounding box grown by the radius meets the stock).
    let (stock_lo, stock_hi) = (geometry.stock_lo, geometry.stock_hi);
    let over_stock = |s: &Stroke| {
        let (lo, hi) = match s.arc {
            None => (
                [s.a[0].min(s.b[0]), s.a[1].min(s.b[1])],
                [s.a[0].max(s.b[0]), s.a[1].max(s.b[1])],
            ),
            Some((c, _)) => {
                let rho = metallic::hypot(s.a[0] - c[0], s.a[1] - c[1]);
                ([c[0] - rho, c[1] - rho], [c[0] + rho, c[1] + rho])
            }
        };
        (0..2).all(|k| lo[k] - radius < stock_hi[k] && hi[k] + radius > stock_lo[k])
    };

    // Runs: maximal runs of pieces that are not the facing's own rectangle.
    // `place[l][p]` is the run and position in it of piece p of loop l.
    let mut runs = vec![];
    let mut air_edge_runs_omitted = 0;
    let mut place: Vec<Vec<Option<(usize, usize)>>> =
        loops.iter().map(|l| vec![None; l.len()]).collect();
    for (l, pieces) in loops.iter().enumerate() {
        let n = pieces.len();
        let mut spans: Vec<(usize, usize, bool)> = vec![];
        match (0..n).find(|&i| own(&pieces[i])) {
            None => spans.push((0, n, true)),
            Some(first) => {
                let mut i = 0;
                while i < n {
                    let at = (first + i) % n;
                    if own(&pieces[at]) {
                        i += 1;
                        continue;
                    }
                    let mut count = 0;
                    while i + count < n && !own(&pieces[(first + i + count) % n]) {
                        count += 1;
                    }
                    spans.push((at, count, false));
                    i += count;
                }
            }
        }
        for (start, count, closed) in spans {
            let index: Vec<usize> = (0..count).map(|i| (start + i) % n).collect();
            let run: Vec<Stroke> = index.iter().map(|&i| pieces[i]).collect();
            if !run.iter().any(over_stock) {
                air_edge_runs_omitted += 1;
                continue;
            }
            for (k, &i) in index.iter().enumerate() {
                place[l][i] = Some((runs.len(), k));
            }
            let air = if closed {
                [false, false]
            } else {
                [geometry.air(run[0].a), geometry.air(run[count - 1].b)]
            };
            runs.push(Run {
                pieces: run,
                closed,
                air,
                done: false,
            });
        }
    }

    // Row stretches: each row's crossings with the cutter-centre boundary.
    let mut crossings = vec![];
    let mut sources = vec![];
    for (l, pieces) in loops.iter().enumerate() {
        for (p, &s) in pieces.iter().enumerate() {
            for c in Crossing::split(s) {
                crossings.push(c);
                sources.push((l, p, own(&s)));
            }
        }
    }
    let mut stretches = vec![];
    let mut air_stretches_omitted = 0;
    let mut xs: Vec<(f64, usize)> = vec![];
    for row in 0..rows {
        if cancel.is_cancelled() {
            return Err(super::PLANNING_CANCELLED.into());
        }
        let y = plan.row_y(row);
        xs.clear();
        xs.extend(
            crossings
                .iter()
                .enumerate()
                .filter_map(|(i, c)| c.x_at(y).map(|x| (x, i))),
        );
        xs.sort_by(|a, b| a.0.total_cmp(&b.0));
        if !xs.len().is_multiple_of(2) {
            return Err(format!(
                "facing {}: ambiguous row intersection with the limits at Y{y}",
                op.id
            ));
        }
        for pair in xs.as_chunks::<2>().0 {
            let width = pair[1].0 - pair[0].0;
            if width <= 8. * tol {
                // Touching the boundary (a tangent): nothing between the
                // crossings to move along.
                continue;
            }
            // An end at the facing's own rectangle is the plain facing's
            // row end exactly; an end at a limit is the crossing itself, on
            // the cutter-centre boundary that the run along that limit
            // follows (it keeps the limits' full clearance).
            let mut x = [0.; 2];
            let mut air = [false; 2];
            let mut anchor = [None; 2];
            for k in 0..2 {
                let (cx, i) = pair[k];
                let (l, p, is_own) = sources[i];
                if is_own {
                    x[k] = if (cx - low).abs() < (cx - high).abs() {
                        low
                    } else {
                        high
                    };
                    air[k] = true;
                } else {
                    x[k] = cx;
                    air[k] = geometry.air([x[k], y]);
                    anchor[k] = place[l][p].map(|(run, piece)| Anchor {
                        run,
                        piece,
                        at: [cx, y],
                    });
                }
            }
            if geometry.clear([x[0], y], [x[1], y]) {
                air_stretches_omitted += 1;
                continue;
            }
            stretches.push(Stretch {
                row,
                y,
                x,
                air,
                anchor,
                done: false,
            });
        }
    }
    Ok(Layout {
        centres,
        crossings,
        stretches,
        runs,
        air_stretches_omitted,
        air_edge_runs_omitted,
    })
}

impl Layout {
    /// The X positions from which a straight step across the rows from `y0`
    /// to `y1` within `[lo, hi]` meets no boundary of the cutter centres,
    /// as open intervals (each kept `margin` from the boundary).
    fn steps(&self, y0: f64, y1: f64, lo: f64, hi: f64, margin: f64) -> Vec<[f64; 2]> {
        let (b0, b1) = (y0.min(y1), y0.max(y1));
        let mut shadows: Vec<[f64; 2]> = self
            .crossings
            .iter()
            .filter_map(|c| c.x_range(b0, b1))
            .map(|[a, b]| [a - margin, b + margin])
            .filter(|s| s[1] > lo && s[0] < hi)
            .collect();
        shadows.sort_by(|a, b| a[0].total_cmp(&b[0]));
        let mut free = vec![];
        let mut from = lo;
        for s in shadows {
            if s[0] > from {
                free.push([from, s[0].min(hi)]);
            }
            from = from.max(s[1]);
            if from >= hi {
                break;
            }
        }
        if from < hi {
            free.push([from, hi]);
        }
        free.retain(|f| f[1] > f[0]);
        free
    }
}

/// The point of the intervals `free` nearest the interval `[a, b]`, and
/// among those inside it the one nearest `prefer`.
fn nearest(free: &[[f64; 2]], a: f64, b: f64, prefer: f64) -> Option<f64> {
    let outside = |x: f64| (a - x).max(x - b).max(0.);
    free.iter()
        .map(|f| prefer.clamp(f[0].max(a).min(f[1]), f[1].min(b).max(f[0])))
        .min_by(|p, q| {
            outside(*p)
                .total_cmp(&outside(*q))
                .then((p - prefer).abs().total_cmp(&(q - prefer).abs()))
        })
}

struct Builder<'a, C: CancelSignal> {
    plan: &'a FacingPlan,
    geometry: Geometry<'a>,
    limits: PreparedLimits,
    cancel: &'a C,
    position: Vec3,
    steps: VecDeque<Step>,
    report: AroundLimitsReport,
}
/// The tool's place at depth after a pass element.
#[derive(Clone, Copy)]
enum At {
    /// Above the stock, or not at the end of an element a link starts from.
    Free,
    /// At end `end` of stretch `stretch`.
    Stretch { stretch: usize, end: usize },
}
impl<C: CancelSignal> Builder<'_, C> {
    fn poll(&self) -> Result<(), String> {
        if self.cancel.is_cancelled() {
            return Err(super::PLANNING_CANCELLED.into());
        }
        Ok(())
    }
    fn proven(&self, curve: &Curve, kind: MoveKind) -> Result<bool, String> {
        self.limits.allows_curve(curve, kind != MoveKind::Rapid)
    }
    /// Plan one move, proven inside the limits first.
    fn push(&mut self, curve: Curve, kind: MoveKind, count: Count) -> Result<(), String> {
        self.poll()?;
        let (start, end) = (curve.start(), curve.end());
        if start != self.position {
            return Err(format!(
                "facing {}: internal planning error: a move starts at {start:?}, not at the tool position {:?}",
                self.plan.op.id, self.position
            ));
        }
        match &curve {
            Curve::Line { .. } if start == end => return Ok(()),
            Curve::Arc { sweep_rad, .. } if *sweep_rad == 0. || start == end => {
                // A piece of no length: an arc split at its own end.
                return Ok(());
            }
            _ => {}
        }
        if !self.proven(&curve, kind)? {
            return Err(if kind == MoveKind::Rapid {
                format!(
                    "facing {}: the rapid from {start:?} to {end:?} passes through an avoid area below its top; raise the clearance/retract heights above it",
                    self.plan.op.id
                )
            } else {
                format!(
                    "facing {}: internal planning error: the {kind:?} move from {start:?} to {end:?} is not proven inside the limits",
                    self.plan.op.id
                )
            });
        }
        curve.prepare(self.plan.op.tolerance_mm)?;
        self.position = end;
        self.steps.push_back((curve, kind, count));
        Ok(())
    }
    fn line(&mut self, end: Vec3, kind: MoveKind, count: Count) -> Result<(), String> {
        self.push(
            Curve::Line {
                start: self.position,
                end,
            },
            kind,
            count,
        )
    }
    /// A fed move at depth to `end`: a link where the whole cutter stays
    /// clear of the stock or over a row already cut (`cut_over`), a cut
    /// otherwise.
    fn feed_to(&mut self, end: Point, cut_over: bool, count: Count) -> Result<(), String> {
        let a = [self.position[0], self.position[1]];
        let kind = if cut_over || self.geometry.clear(a, end) {
            MoveKind::Link
        } else {
            MoveKind::Cut
        };
        self.line([end[0], end[1], self.position[2]], kind, count)
    }
    /// Straight up: at feed to the feed height, then rapid to the retract
    /// height (the clearance height after the last move).
    fn lift(&mut self, last: bool) -> Result<(), String> {
        let h = self.plan.heights;
        let [x, y, z] = self.position;
        if z < h.feed {
            self.line([x, y, h.feed], MoveKind::Retract, Count::Return)?;
        }
        let top = if last { h.clearance } else { h.retract };
        if self.position[2] < top {
            self.line([x, y, top], MoveKind::Rapid, Count::None)?;
        }
        Ok(())
    }
    /// A vertical entry at `xy`, where the whole cutter is clear of the
    /// stock by the air gap.
    fn enter(&mut self, xy: Point, z: f64) -> Result<(), String> {
        if !self.geometry.air(xy) {
            return Err(format!(
                "facing {}: internal planning error: an entry at ({:.4}, {:.4}) is not clear of the stock",
                self.plan.op.id, xy[0], xy[1]
            ));
        }
        let h = self.plan.heights;
        if self.position[2] < h.feed {
            self.lift(false)?;
        }
        let travel = self.position[2];
        self.line([xy[0], xy[1], travel], MoveKind::Rapid, Count::None)?;
        self.line([xy[0], xy[1], h.feed], MoveKind::Rapid, Count::None)?;
        self.line([xy[0], xy[1], z], MoveKind::Plunge, Count::None)?;
        self.report.entries += 1;
        Ok(())
    }
    /// Cut stretch `t` from its end `start` to the other; returns that end.
    fn cut_stretch(&mut self, layout: &mut Layout, t: usize, start: usize) -> Result<At, String> {
        let s = &layout.stretches[t];
        let end = 1 - start;
        let (x, y) = (s.x[end], s.y);
        let z = self.position[2];
        self.line([x, y, z], MoveKind::Cut, Count::Row)?;
        layout.stretches[t].done = true;
        self.report.row_stretches += 1;
        Ok(At::Stretch { stretch: t, end })
    }
    /// The step from stretch `c` (the tool at its end at `x`) to stretch
    /// `t`: the X to step at and the travel it adds to a plain zigzag link
    /// (back along `c`, then along `t` to its nearer end `s`).
    fn step(
        &self,
        layout: &Layout,
        c: usize,
        x: f64,
        t: usize,
        z: f64,
    ) -> Result<Option<(f64, usize, f64)>, String> {
        let (from, to) = (&layout.stretches[c], &layout.stretches[t]);
        let s = usize::from((to.x[1] - x).abs() < (to.x[0] - x).abs());
        let start = to.x[s];
        let (lo, hi) = (from.x[0].max(to.x[0]), from.x[1].min(to.x[1]));
        if lo > hi {
            return Ok(None);
        }
        let trim = 4. * self.plan.op.tolerance_mm;
        let mut free = layout.steps(from.y, to.y, lo, hi, trim);
        // The stretches' own ends are where the plain facing's side link
        // runs (outside the stock) or where they stop at a limit; both are
        // candidate step positions in their own right.
        for e in [x, start] {
            if (lo..=hi).contains(&e) {
                free.push([e, e]);
            }
        }
        let (a, b) = (x.min(start), x.max(start));
        while let Some(at) = nearest(&free, a, b, x) {
            self.poll()?;
            let proven = self.proven(
                &Curve::Line {
                    start: [at, from.y, z],
                    end: [at, to.y, z],
                },
                MoveKind::Link,
            )?;
            if proven {
                return Ok(Some((at, s, (x - at).abs() + (at - start).abs())));
            }
            // Not proven there (a numerical edge case of the shadows): no
            // step anywhere in that interval is tried again.
            free.retain(|f| !(f[0] <= at && at <= f[1]));
        }
        Ok(None)
    }
    /// Take the step to stretch `t` found by `step`, then cut it.
    fn link_and_cut(
        &mut self,
        layout: &mut Layout,
        at: f64,
        t: usize,
        s: usize,
    ) -> Result<At, String> {
        let (y, row, start) = {
            let to = &layout.stretches[t];
            (self.position[1], to.y, to.x[s])
        };
        // Back along the row just cut (already cut), across, then along the
        // next row to its nearer end.
        self.feed_to([at, y], true, Count::None)?;
        self.feed_to([at, row], false, Count::StayDown)?;
        self.feed_to([start, row], false, Count::None)?;
        self.report.stay_down_links += 1;
        self.cut_stretch(layout, t, s)
    }
    /// The stretch to link to at depth from end `e` of stretch `c`: one of a
    /// neighbouring row that a proven step reaches, those closed off at both
    /// ends first (nothing else reaches them), then the least added travel;
    /// one that could be entered from air only within `economical`. Returns
    /// (stretch, step X, its start end).
    fn continuation(
        &self,
        layout: &Layout,
        c: usize,
        e: usize,
        z: f64,
        economical: f64,
    ) -> Result<Option<(usize, f64, usize)>, String> {
        let x = layout.stretches[c].x[e];
        let row = layout.stretches[c].row;
        let mut best: Option<(u8, f64, usize, f64, usize)> = None;
        for t in 0..layout.stretches.len() {
            let to = &layout.stretches[t];
            if t == c || to.done || to.row.abs_diff(row) != 1 {
                continue;
            }
            // Stretches closed off at both ends first (nothing else reaches
            // them), then those a chain can go on from, then those leading
            // into a run (where a chain stops until the runs are cut).
            let class = if to.enclosed() {
                0
            } else if to.lead().is_none() {
                1
            } else {
                2
            };
            let Some((step, s, extra)) = self.step(layout, c, x, t, z)? else {
                continue;
            };
            if class > 0 && extra > economical {
                continue;
            }
            if best.is_none_or(|(k, cost, ..)| class < k || (class == k && extra < cost)) {
                best = Some((class, extra, t, step, s));
            }
        }
        Ok(best.map(|(_, _, t, step, s)| (t, step, s)))
    }
    /// The row stretches of one depth. A stretch leading from air into a run
    /// that no link continues from is left for the edge phase, which cuts it
    /// on the way into that run instead of entering it on its own. Returns
    /// where the tool is.
    fn rows(&mut self, layout: &mut Layout, z: f64, economical: f64) -> Result<At, String> {
        let mut at = At::Free;
        let mut first = true;
        loop {
            self.poll()?;
            if let At::Stretch { stretch: c, end: e } = at
                && let Some((t, step, s)) = self.continuation(layout, c, e, z, economical)?
            {
                at = self.link_and_cut(layout, step, t, s)?;
                continue;
            }
            // A new chain: the first stretch with an entry from air (the
            // lowest row from its low end, as the plain facing starts), then
            // the one whose entry is nearest.
            let here = [self.position[0], self.position[1]];
            let mut chain: Option<(f64, usize, usize)> = None;
            for (t, s) in layout.stretches.iter().enumerate() {
                if s.done || s.enclosed() {
                    continue;
                }
                if let Some((end, _)) = s.lead()
                    && self
                        .continuation(layout, t, 1 - end, z, economical)?
                        .is_none()
                {
                    continue;
                }
                for end in 0..2 {
                    if !s.air[end] {
                        continue;
                    }
                    let d = if first {
                        (s.row * 2 + end) as f64
                    } else {
                        metallic::hypot(s.x[end] - here[0], s.y - here[1])
                    };
                    if chain.is_none_or(|b| d < b.0) {
                        chain = Some((d, t, end));
                    }
                }
            }
            if let Some((_, t, end)) = chain {
                first = false;
                let s = &layout.stretches[t];
                self.enter([s.x[end], s.y], z)?;
                at = self.cut_stretch(layout, t, end)?;
                continue;
            }
            // Only stretches closed off at both ends, and those leading into
            // a run, are left: each closed one is reached along a
            // neighbouring row already cut, entered from its end in air.
            let Some(t) = layout
                .stretches
                .iter()
                .position(|s| !s.done && s.enclosed())
            else {
                return Ok(at);
            };
            let mut way = None;
            'search: for (t, to) in layout.stretches.iter().enumerate() {
                if to.done || !to.enclosed() {
                    continue;
                }
                for (c, from) in layout.stretches.iter().enumerate() {
                    if !from.done || from.enclosed() || from.row.abs_diff(to.row) != 1 {
                        continue;
                    }
                    for end in 0..2 {
                        if !from.air[end] {
                            continue;
                        }
                        if let Some((step, s, _)) = self.step(layout, c, from.x[end], t, z)? {
                            way = Some((c, end, t, step, s));
                            break 'search;
                        }
                    }
                }
            }
            let Some((c, end, t, step, s)) = way else {
                let s = &layout.stretches[t];
                return Err(format!(
                    "facing {}: no entry from outside the stock reaches the row stretch X{:.3}..{:.3} at Y{:.3}: the limits close it off at both ends, and no row beside it leads to it. Free one end of it (move or shrink the avoid areas, or widen the containment), or machine that area with a Pocket and its own entry",
                    self.plan.op.id, s.x[0], s.x[1], s.y
                ));
            };
            let from = &layout.stretches[c];
            self.enter([from.x[end], from.y], z)?;
            at = self.link_and_cut(layout, step, t, s)?;
        }
    }
    /// Cut run `r` entered at `anchor` (the tool at depth beside it): back
    /// along it to its start, then the whole run in its own direction; a
    /// closed run once round from the anchor. Returns where it ends.
    fn cut_run(&mut self, layout: &mut Layout, anchor: Anchor) -> Result<Point, String> {
        let z = self.position[2];
        let pieces = layout.runs[anchor.run].pieces.clone();
        let closed = layout.runs[anchor.run].closed;
        self.feed_to(anchor.at, true, Count::None)?;
        let cut = |b: &mut Self, s: Stroke| b.push(s.curve(z), MoveKind::Cut, Count::None);
        let (head, tail) = split(pieces[anchor.piece], anchor.at);
        if closed {
            cut(self, tail)?;
            for &s in pieces[anchor.piece + 1..]
                .iter()
                .chain(&pieces[..anchor.piece])
            {
                cut(self, s)?;
            }
            cut(self, head)?;
        } else {
            cut(self, head.reversed())?;
            for s in pieces[..anchor.piece].iter().rev() {
                cut(self, s.reversed())?;
            }
            for &s in &pieces {
                cut(self, s)?;
            }
        }
        layout.runs[anchor.run].done = true;
        self.report.edge_runs += 1;
        Ok([self.position[0], self.position[1]])
    }
    /// The fed travel back along a run from `anchor` to its start (none for
    /// a closed run, which is cut once round from the anchor).
    fn backtrack(layout: &Layout, anchor: Anchor) -> f64 {
        let run = &layout.runs[anchor.run];
        if run.closed {
            return 0.;
        }
        let (head, _) = split(run.pieces[anchor.piece], anchor.at);
        run.pieces[..anchor.piece]
            .iter()
            .map(stroke_length)
            .sum::<f64>()
            + stroke_length(&head)
    }
    /// A stretch still to cut that leads into a run that is cut, or into
    /// one it may be cut on the way into: going back along the run from
    /// where it arrives takes no longer than a retract and re-entry.
    fn ready(layout: &Layout, s: &Stretch, economical: f64) -> Option<(usize, Anchor)> {
        s.lead().filter(|(_, a)| {
            !s.done && (layout.runs[a.run].done || Self::backtrack(layout, *a) <= economical)
        })
    }
    /// A link at depth from `from` to `to` through air (the whole cutter
    /// clear of the stock), proven inside the limits, no longer than
    /// `budget`: its length.
    fn air_link(&self, from: Point, to: Point, z: f64, budget: f64) -> Result<Option<f64>, String> {
        let length = metallic::hypot(to[0] - from[0], to[1] - from[1]);
        if length > budget || !self.geometry.clear(from, to) {
            return Ok(None);
        }
        let curve = Curve::Line {
            start: [from[0], from[1], z],
            end: [to[0], to[1], z],
        };
        Ok(self.proven(&curve, MoveKind::Link)?.then_some(length))
    }
    /// The nearest edge-phase element an air link from `from` (in air at
    /// depth `z`) reaches within `budget`: a stretch leading into a run, or
    /// the start of a run.
    fn through_air(
        &self,
        layout: &Layout,
        from: Point,
        z: f64,
        budget: f64,
        economical: f64,
    ) -> Result<Option<Target>, String> {
        if !self.geometry.air(from) {
            return Ok(None);
        }
        let mut best: Option<(f64, Target)> = None;
        for (t, s) in layout.stretches.iter().enumerate() {
            if let Some((end, _)) = Self::ready(layout, s, economical)
                && let Some(length) = self.air_link(from, [s.x[end], s.y], z, budget)?
                && best.as_ref().is_none_or(|b| length < b.0)
            {
                best = Some((length, Target::Lead(t, end)));
            }
        }
        for (r, run) in layout.runs.iter().enumerate() {
            if !run.done
                && !run.closed
                && run.air[0]
                && let Some(length) = self.air_link(from, run.pieces[0].a, z, budget)?
                && best.as_ref().is_none_or(|b| length < b.0)
            {
                best = Some((length, Target::Start(r)));
            }
        }
        Ok(best.map(|b| b.1))
    }
    /// The runs along the limits' edges of one depth, and the stretches the
    /// rows left that lead from air into a run. `at` is where the rows left
    /// the tool.
    fn edges(
        &mut self,
        layout: &mut Layout,
        z: f64,
        mut at: At,
        economical: f64,
    ) -> Result<(), String> {
        loop {
            self.poll()?;
            // At the end of a stretch that stops at a run still to cut: into
            // the run at depth.
            if let At::Stretch { stretch, end } = at
                && let Some(anchor) = layout.stretches[stretch].anchor[end]
                && !layout.runs[anchor.run].done
            {
                self.cut_run(layout, anchor)?;
                self.report.stay_down_links += 1;
                at = At::Free;
                continue;
            }
            // At the end of a stretch whose run is cut: back along it (cut)
            // to its end in air, when an air link goes on from there and the
            // two together are shorter than a retract and re-entry.
            if let At::Stretch { stretch, end } = at {
                let s = &layout.stretches[stretch];
                let (back, length) = ([s.x[1 - end], s.y], s.x[1] - s.x[0]);
                at = At::Free;
                if !s.air[end]
                    && s.air[1 - end]
                    && length <= economical
                    && self
                        .through_air(layout, back, z, economical - length, economical)?
                        .is_some()
                {
                    self.feed_to(back, true, Count::None)?;
                }
                continue;
            }
            // In air at depth: through air to a stretch leading into a run,
            // or to the start of a run, when that is shorter than a retract
            // and re-entry.
            let here = [self.position[0], self.position[1]];
            let best = if self.position[2] == z {
                self.through_air(layout, here, z, economical, economical)?
            } else {
                None
            };
            if let Some(target) = best {
                self.report.stay_down_links += 1;
                at = self.reach(layout, target, z, false)?;
                continue;
            }
            // A new entry from air, nearest first: for each run still to
            // cut, a stretch leading into it (cut on the way in) unless
            // going back along the run from where it arrives would take
            // longer than a retract and re-entry and the run starts in air;
            // else the run's start; else along a stretch already cut; else the
            // run's end. Then each leading stretch left whose run is cut.
            let distance = |p: Point| metallic::hypot(p[0] - here[0], p[1] - here[1]);
            let mut entry: Option<(f64, Target)> = None;
            let mut consider = |p: Point, target: Target| {
                let d = distance(p);
                if entry.as_ref().is_none_or(|b| d < b.0) {
                    entry = Some((d, target));
                }
            };
            for (r, run) in layout.runs.iter().enumerate() {
                if run.done {
                    continue;
                }
                let mut lead: Option<(f64, usize, usize)> = None;
                for (t, s) in layout.stretches.iter().enumerate() {
                    if let Some((end, anchor)) = s.lead().filter(|(_, a)| a.run == r && !s.done) {
                        let extra = Self::backtrack(layout, anchor);
                        if lead.is_none_or(|l| extra < l.0) {
                            lead = Some((extra, t, end));
                        }
                    }
                }
                let start = (!run.closed && run.air[0]).then_some(run.pieces[0].a);
                if let Some((extra, t, end)) = lead
                    && (extra <= economical || start.is_none())
                {
                    let s = &layout.stretches[t];
                    consider([s.x[end], s.y], Target::Lead(t, end));
                    continue;
                }
                if let Some(a) = start {
                    consider(a, Target::Start(r));
                    continue;
                }
                let mut along: Option<(f64, usize, usize)> = None;
                for (t, s) in layout.stretches.iter().enumerate() {
                    for end in 0..2 {
                        if let Some(anchor) = s.anchor[end].filter(|a| a.run == r)
                            && s.air[1 - end]
                        {
                            let extra = s.x[1] - s.x[0] + Self::backtrack(layout, anchor);
                            if along.is_none_or(|w| extra < w.0) {
                                along = Some((extra, t, 1 - end));
                            }
                        }
                    }
                }
                if let Some((_, t, end)) = along {
                    let s = &layout.stretches[t];
                    consider([s.x[end], s.y], Target::Along(t, end));
                    continue;
                }
                if !run.closed && run.air[1] {
                    consider(run.pieces[run.pieces.len() - 1].b, Target::End(r));
                    continue;
                }
                let a = run.pieces[0].a;
                return Err(format!(
                    "facing {}: no entry from outside the stock reaches the edge of the limits at ({:.3}, {:.3}): no row of the facing runs from outside the stock up to it. Free a row's end there (move or shrink the avoid areas, or widen the containment), or machine that area with a Pocket and its own entry",
                    self.plan.op.id, a[0], a[1]
                ));
            }
            for (t, s) in layout.stretches.iter().enumerate() {
                if let Some((end, _)) = s.lead().filter(|(_, a)| !s.done && layout.runs[a.run].done)
                {
                    consider([s.x[end], s.y], Target::Lead(t, end));
                }
            }
            let Some((_, target)) = entry else {
                return Ok(());
            };
            at = self.reach(layout, target, z, true)?;
        }
    }
    /// Go to `target` (entering from air when `enter`, else already at depth
    /// in air) and cut it: a leading stretch, or a run from its start or
    /// end, or along a stretch to the run it stops at.
    fn reach(
        &mut self,
        layout: &mut Layout,
        target: Target,
        z: f64,
        enter: bool,
    ) -> Result<At, String> {
        let point = match target {
            Target::Lead(t, end) | Target::Along(t, end) => {
                let s = &layout.stretches[t];
                [s.x[end], s.y]
            }
            Target::Start(r) => layout.runs[r].pieces[0].a,
            Target::End(r) => layout.runs[r].pieces[layout.runs[r].pieces.len() - 1].b,
        };
        if enter {
            self.enter(point, z)?;
        } else {
            self.feed_to(point, true, Count::None)?;
        }
        Ok(match target {
            Target::Lead(t, end) => self.cut_stretch(layout, t, end)?,
            Target::Start(run) => {
                self.cut_run(
                    layout,
                    Anchor {
                        run,
                        piece: 0,
                        at: point,
                    },
                )?;
                At::Free
            }
            Target::End(run) => {
                let piece = layout.runs[run].pieces.len() - 1;
                self.cut_run(
                    layout,
                    Anchor {
                        run,
                        piece,
                        at: point,
                    },
                )?;
                At::Free
            }
            Target::Along(t, end) => {
                if layout.stretches[t].done {
                    let s = &layout.stretches[t];
                    let (x, y) = (s.x[1 - end], s.y);
                    self.feed_to([x, y], true, Count::None)?;
                    At::Stretch {
                        stretch: t,
                        end: 1 - end,
                    }
                } else {
                    self.cut_stretch(layout, t, end)?
                }
            }
        })
    }
}

/// What the edge phase moves to next.
#[derive(Clone, Copy)]
enum Target {
    /// Stretch `t` from its end in air `end`, into the run it stops at.
    Lead(usize, usize),
    /// The start of run `r`, in air.
    Start(usize),
    /// The end of run `r`, in air (the only end that is).
    End(usize),
    /// Along stretch `t` from its end in air `end` to the run it stops at.
    Along(usize, usize),
}

/// The complete route of `plan` (a prepared plain facing: its rows, heights
/// and overrun) round `limits`, every move proven inside them. `radius` is the
/// cutter's outer radius (what must keep out of the limits) and
/// `floor_radius` the radius it cuts the floor to (what it reaches).
pub(super) fn route(
    plan: &FacingPlan,
    limits: &Limits,
    radius: f64,
    floor_radius: f64,
    cancel: &impl CancelSignal,
) -> Result<Route, String> {
    let op = &plan.op;
    let tol = op.tolerance_mm;
    let spec = &plan.spec;
    let geometry = Geometry {
        plan,
        radius,
        stock_lo: [spec.min_mm[0], spec.min_mm[1]],
        stock_hi: [spec.max_mm[0], spec.max_mm[1]],
    };
    let h = plan.heights;
    let mut builder = Builder {
        plan,
        geometry,
        limits: PreparedLimits::new(limits, radius, tol)?,
        cancel,
        position: plan.initial,
        steps: VecDeque::new(),
        report: AroundLimitsReport::default(),
    };
    let [x, y, _] = plan.initial;
    builder.line([x, y, h.clearance], MoveKind::Rapid, Count::None)?;
    // One layout per set of avoid areas whose tops are above the depth.
    let mut layouts: Vec<(Vec<bool>, Layout)> = vec![];
    let passes = plan.report.depth_passes;
    let mut floor = None;
    for pass in 0..passes {
        let z = (spec.max_mm[2] - (pass + 1) as f64 * op.stepdown_mm).max(op.floor_z_mm);
        let active: Vec<bool> = limits.avoid.iter().map(|a| a.top_z_mm > z).collect();
        let index = match layouts.iter().position(|(k, _)| *k == active) {
            Some(i) => i,
            None => {
                let at_depth = Limits {
                    contain: limits.contain.clone(),
                    avoid: limits
                        .avoid
                        .iter()
                        .zip(&active)
                        .filter(|(_, a)| **a)
                        .map(|(a, _)| AvoidArea::clone(a))
                        .collect(),
                };
                let layout = layout(&builder.geometry, &at_depth, cancel)?;
                if layout.stretches.is_empty() {
                    return Err(format!(
                        "facing {}: the limits leave no row of the facing to cut at Z{z} (the avoid areas or the containment cover the stock top)",
                        op.id
                    ));
                }
                layouts.push((active, layout));
                layouts.len() - 1
            }
        };
        let layout = &mut layouts[index].1;
        for s in &mut layout.stretches {
            s.done = false;
        }
        for r in &mut layout.runs {
            r.done = false;
        }
        // Retracting to the feed height and plunging back takes at least as
        // long as this much fed travel (`PocketPlan`'s stay-down rule).
        let economical = (h.feed - z) * (1. + op.feed_mm_min / op.plunge_mm_min);
        let at = builder.rows(layout, z, economical)?;
        builder.edges(layout, z, at, economical)?;
        builder.report.air_stretches_omitted += layout.air_stretches_omitted;
        builder.report.air_edge_runs_omitted += layout.air_edge_runs_omitted;
        builder.lift(pass + 1 == passes)?;
        floor = Some(index);
    }
    // What no allowed cutter position reaches at the floor.
    let centres = &layouts[floor.ok_or("facing: no depth pass")?].1.centres;
    let options = ConstructionOptions { tolerance_mm: tol };
    let reach = centres.offset_prepared(-floor_radius)?.into_parts().1;
    builder.report.unreachable_area_mm2 = rectangle(
        format!("{} stock top", op.id),
        builder.geometry.stock_lo,
        builder.geometry.stock_hi,
    )
    .prepare(options)?
    .boolean_prepared(&reach, Boolean::Difference)?
    .region()
    .summary()
    .area_mm2
    .max(0.);
    Ok(Route {
        steps: builder.steps,
        report: builder.report,
    })
}
