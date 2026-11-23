//! Conservative adaptive waterlines with continuous solid/error certificates.
//! Geometry contract, including inside distance and contour error: docs/waterline.md.
use crate::{
    Point,
    finishing::{FinishingStockTop, StockCeiling},
    motion::{Coolant, Curve, Motion, MoveKind, Vec3},
    pocket::{ResidualStatus, Stroke},
    profiles::{ProfileDepths, ProfileMotion, ProfileSettings, stroke_length},
    roughing::PreparedRoughingTarget,
    stock::{CancelSignal, Stock, StockSpec},
    surface::{PreparedSurface, SurfaceMesh, SurfaceWork},
    tool::{CutterShape, ToolAssembly, valid_id},
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, VecDeque};
mod support;
type Vertex = [u32; 2];
type GridEdge = [Vertex; 2];
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ContourMethod {
    #[default]
    QuadTree,
    Support,
}
#[derive(Clone, Copy)]
pub struct ContourOptions {
    pub method: ContourMethod,
    pub center_z_mm: f64,
    pub radius_mm: f64,
    pub protected_radius_mm: f64,
    pub normal_error_mm: f64,
    pub tolerance_mm: f64,
    pub xy_min_mm: Point,
    pub xy_max_mm: Point,
    pub simplify: bool,
}
#[derive(Clone, Debug, Serialize)]
pub struct LevelReport {
    pub method: ContourMethod,
    pub support_queries: usize,
    pub support_vertices: usize,
    pub certificate_tests: usize,
    pub certificate_nodes: usize,
    pub boolean_operations: usize,
    pub support_error_mm: f64,
    pub boolean_error_mm: f64,
    pub tip_z_mm: f64,
    pub grid_cells: usize,
    pub finest_cell_mm: f64,
    pub raw_edges: usize,
    pub cut_edges: usize,
    pub loops: usize,
    pub removed_vertices: usize,
    pub max_normal_excess_mm: f64,
    pub work: SurfaceWork,
}
pub struct WaterlineContours {
    pub loops: Vec<Vec<Point>>,
    pub report: LevelReport,
}
#[cfg(test)]
fn total(work: SurfaceWork) -> usize {
    work.nodes + work.contact_tests + work.distance_tests
}
fn sum(a: SurfaceWork, b: SurfaceWork) -> SurfaceWork {
    SurfaceWork {
        nodes: a.nodes + b.nodes,
        contact_tests: a.contact_tests + b.contact_tests,
        distance_tests: a.distance_tests + b.distance_tests,
    }
}
fn cancelled(cancel: &impl CancelSignal) -> Result<(), String> {
    if cancel.is_cancelled() {
        Err("waterline planning cancelled".into())
    } else {
        Ok(())
    }
}
fn events(map: &mut BTreeMap<u32, Vec<(u32, i32)>>, fixed: u32, a: u32, b: u32, sign: i32) {
    let entries = map.entry(fixed).or_default();
    entries.push((a, sign));
    entries.push((b, -sign));
}
fn boundaries(
    map: BTreeMap<u32, Vec<(u32, i32)>>,
    horizontal: bool,
    edges: &mut Vec<GridEdge>,
    cancel: &impl CancelSignal,
) -> Result<(), String> {
    for (fixed, mut entries) in map {
        cancelled(cancel)?;
        entries.sort_unstable();
        let mut i = 0;
        let mut count = 0i32;
        let mut start = 0;
        while i < entries.len() {
            let x = entries[i].0;
            let mut delta = 0;
            while i < entries.len() && entries[i].0 == x {
                delta += entries[i].1;
                i += 1;
            }
            if delta == 0 {
                continue;
            }
            let next = count + delta;
            if next.abs() > 1 {
                return Err("waterline occupancy edge overlap".into());
            }
            if count != 0 && x > start {
                let a = if horizontal {
                    [start, fixed]
                } else {
                    [fixed, start]
                };
                let b = if horizontal { [x, fixed] } else { [fixed, x] };
                edges.push(if count > 0 { [a, b] } else { [b, a] });
            }
            start = x;
            count = next;
        }
        if count != 0 {
            return Err("waterline occupancy boundary is not closed".into());
        }
    }
    Ok(())
}
fn turn(incoming: GridEdge, outgoing: GridEdge) -> i32 {
    let a: [i64; 2] =
        std::array::from_fn(|i| (incoming[1][i] as i64 - incoming[0][i] as i64).signum());
    let b: [i64; 2] =
        std::array::from_fn(|i| (outgoing[1][i] as i64 - outgoing[0][i] as i64).signum());
    let cross = a[0] * b[1] - a[1] * b[0];
    let dot = a[0] * b[0] + a[1] * b[1];
    if cross > 0 {
        0
    } else if dot > 0 {
        1
    } else if cross < 0 {
        2
    } else {
        3
    }
}
fn trace(edges: &[GridEdge], cancel: &impl CancelSignal) -> Result<Vec<Vec<Vertex>>, String> {
    let mut outgoing: BTreeMap<Vertex, Vec<usize>> = BTreeMap::new();
    for (i, e) in edges.iter().enumerate() {
        outgoing.entry(e[0]).or_default().push(i);
    }
    if outgoing.values().any(|v| v.len() > 2) {
        return Err("waterline ambiguous boundary vertex".into());
    }
    let mut seen = vec![false; edges.len()];
    let mut loops = vec![];
    for first in 0..edges.len() {
        if seen[first] {
            continue;
        }
        let start = edges[first][0];
        let mut i = first;
        let mut ring = vec![];
        loop {
            cancelled(cancel)?;
            if seen[i] {
                return Err("waterline repeated boundary edge".into());
            }
            seen[i] = true;
            ring.push(edges[i][0]);
            let end = edges[i][1];
            if end == start {
                break;
            }
            i = *outgoing
                .get(&end)
                .ok_or("waterline boundary gap")?
                .iter()
                .filter(|j| !seen[**j])
                .min_by_key(|j| turn(edges[i], edges[**j]))
                .ok_or("waterline boundary cannot close")?;
        }
        let area = ring
            .iter()
            .zip(ring.iter().cycle().skip(1))
            .take(ring.len())
            .map(|(a, b)| a[0] as i128 * b[1] as i128 - a[1] as i128 * b[0] as i128)
            .sum::<i128>();
        if ring.len() < 3 || area == 0 {
            return Err("degenerate waterline boundary".into());
        }
        loops.push(ring);
    }
    Ok(loops)
}
fn simplify_ring(
    surface: &PreparedSurface,
    points: &[Point],
    o: ContourOptions,
    work: &mut SurfaceWork,
    cancel: &impl CancelSignal,
) -> Result<(Vec<Point>, usize, f64), String> {
    let n = points.len();
    let mut previous: Vec<_> = (0..n).map(|i| (i + n - 1) % n).collect();
    let mut next: Vec<_> = (0..n).map(|i| (i + 1) % n).collect();
    let mut alive = vec![true; n];
    let mut queue: VecDeque<_> = (0..n).collect();
    let mut remaining = n;
    let mut excess = 0f64;
    let point = |i: usize| [points[i][0], points[i][1], o.center_z_mm];
    while let Some(i) = queue.pop_front() {
        cancelled(cancel)?;
        if !alive[i] || remaining <= 3 {
            continue;
        }
        let a = previous[i];
        let b = next[i];
        if points[a] == points[b] {
            continue;
        }
        let pa = point(a);
        let pb = point(b);
        let mid = [(pa[0] + pb[0]) / 2., (pa[1] + pb[1]) / 2., o.center_z_mm];
        let witness = surface
            .level_distance(mid, o.radius_mm, work, cancel)?
            .ok_or("waterline shortcut lost its source support")?
            .witness;
        let upper = surface
            .witness_distance(pa, witness, work)
            .max(surface.witness_distance(pb, witness, work));
        if upper > o.radius_mm + o.normal_error_mm - 16. * o.tolerance_mm {
            continue;
        }
        // The filled ear must be clear. A chord alone could discard an island.
        if !surface.triangle_clear([pa, point(i), pb], o.protected_radius_mm, work, cancel)? {
            continue;
        }
        excess = excess.max((upper - o.radius_mm).max(0.));
        alive[i] = false;
        remaining -= 1;
        next[a] = b;
        previous[b] = a;
        queue.push_back(a);
        queue.push_back(b);
    }
    let first = alive.iter().position(|x| *x).unwrap();
    let mut result = vec![];
    let mut i = first;
    loop {
        result.push(points[i]);
        i = next[i];
        if i == first {
            break;
        }
    }
    Ok((result, n - remaining, excess))
}
/// The contact (slope) angles, in radians from the tool axis, of the ball at
/// a contour sample: least and greatest.
///
/// A waterline's tool is on its level, so its contact is the nearest point
/// of the part to the ball's center there: 90 degrees against a vertical
/// wall, a facet's slope on a facet, the roll-over angle on an edge. (A ball
/// dropped onto the part at the sample's XY rests somewhere else: at a
/// wall's foot it lands on the floor, and the wall would be classified
/// flat.) A sample lies within `band` of the offset surface, not on it, so
/// it is first moved onto the surface along the contact normal; the range
/// then covers every contact within the tolerance of the nearest, which is
/// more than one where the ball touches a wall and a floor at once.
fn level_contact_angles(
    surface: &PreparedSurface,
    sample: [f64; 3],
    radius: f64,
    band: f64,
    tolerance: f64,
    work: &mut SurfaceWork,
    cancel: &impl CancelSignal,
) -> Result<(f64, f64), String> {
    let tie = 64. * tolerance;
    let mut center = sample;
    let mut range = None;
    for _ in 0..8 {
        let Some(contact) = surface.level_contact(center, radius, tie, work, cancel)? else {
            break;
        };
        range = Some(contact.cos_axis);
        let off = radius - contact.distance_mm;
        let horizontal = metallic::hypot(contact.normal[0], contact.normal[1]);
        // On the surface, or resting on a flat (no horizontal move changes
        // the distance there).
        if off.abs() <= tie || horizontal < 1e-6 {
            break;
        }
        let step = off / horizontal;
        let next = [
            center[0] + step * contact.normal[0] / horizontal,
            center[1] + step * contact.normal[1] / horizontal,
            center[2],
        ];
        // Never further from the sample than a sample can be from the surface.
        if metallic::hypot(next[0] - sample[0], next[1] - sample[1]) > band + tie {
            break;
        }
        center = next;
    }
    // Nothing of the part within reach of the level: as on the base.
    let [low, high] = range.unwrap_or([1., 1.]);
    Ok((
        metallic::acos(high.clamp(-1., 1.)),
        metallic::acos(low.clamp(-1., 1.)),
    ))
}
pub fn prepare_contours(
    surface: &PreparedSurface,
    o: ContourOptions,
    cancel: &impl CancelSignal,
) -> Result<WaterlineContours, String> {
    prepare_contours_with(surface, o, 1, cancel)
}
enum Cell {
    Skip,
    Occupy,
    Split,
}
/// As `prepare_contours`, the quad tree using up to `workers` threads: the
/// top of the tree is expanded serially until there are enough independent
/// subtrees, which are then finished in parallel. Every cell gets the same
/// decision as in a serial run, so the contours are identical.
pub fn prepare_contours_with(
    surface: &PreparedSurface,
    o: ContourOptions,
    workers: usize,
    cancel: &impl CancelSignal,
) -> Result<WaterlineContours, String> {
    cancelled(cancel)?;
    let guard = 16. * o.tolerance_mm;
    if [
        o.center_z_mm,
        o.radius_mm,
        o.protected_radius_mm,
        o.normal_error_mm,
        o.tolerance_mm,
    ]
    .iter()
    .any(|x| !x.is_finite())
        || !(1e-7..=0.001).contains(&o.tolerance_mm)
        || o.normal_error_mm <= 4. * guard
        || o.normal_error_mm > 0.1
        || o.protected_radius_mm <= 0.
        || o.radius_mm <= o.protected_radius_mm
        || o.xy_min_mm
            .iter()
            .chain(o.xy_max_mm.iter())
            .any(|v| !v.is_finite())
        || (0..2).any(|k| o.xy_min_mm[k] >= o.xy_max_mm[k])
    {
        return Err("invalid waterline contour options".into());
    }
    // A level must be above the infinite offset base, even if all facets cull.
    let _ = surface.level_distance(
        [o.xy_min_mm[0], o.xy_min_mm[1], o.center_z_mm],
        o.radius_mm,
        &mut SurfaceWork::default(),
        cancel,
    )?;
    if o.method == ContourMethod::Support {
        return support::prepare(surface, o, cancel);
    }
    let span = (o.xy_max_mm[0] - o.xy_min_mm[0]).max(o.xy_max_mm[1] - o.xy_min_mm[1])
        + 2. * (o.radius_mm + o.normal_error_mm + 4. * guard);
    let target = (o.normal_error_mm - 2. * guard) / 2f64.sqrt();
    let depth = (span / target).log2().ceil().max(0.) as u32;
    if depth > 24 {
        // Grid coordinates are u32 and the finest cell must stay well above
        // f64 resolution: an invariant, not a work budget.
        return Err(format!(
            "waterline contour_error_mm {} is too fine for a {span:.1} mm wide part (grid depth {depth} > 24)",
            o.normal_error_mm
        ));
    }
    let grid = 1u32 << depth;
    let cell = span / grid as f64;
    let origin: Point = std::array::from_fn(|k| (o.xy_min_mm[k] + o.xy_max_mm[k] - span) / 2.);
    if (0..2).any(|k| origin[k].abs() > 1e6 || (origin[k] + span).abs() > 1e6) {
        return Err("waterline grid coordinate overtravel".into());
    }
    let classify = |x: u32,
                    y: u32,
                    size: u32,
                    scan: &mut SurfaceWork,
                    cancel: &dyn CancelSignal|
     -> Result<Cell, String> {
        let p = [
            origin[0] + (x as f64 + size as f64 / 2.) * cell,
            origin[1] + (y as f64 + size as f64 / 2.) * cell,
            o.center_z_mm,
        ];
        let stop = || cancel.is_cancelled();
        let contact = surface.level_distance(p, o.radius_mm, scan, &stop)?;
        let distance = contact.map_or(f64::INFINITY, |d| d.distance_mm);
        let diagonal = size as f64 * cell / 2f64.sqrt();
        if distance > o.radius_mm + diagonal + guard {
            return Ok(Cell::Skip);
        }
        // Near a horizontal plateau, a Lipschitz ball alone would refine its
        // entire area. Distance to one convex prism is convex: four supported
        // corners certify the whole rectangle inside, including its interior.
        let inside_prism = if distance < o.radius_mm - guard && size > 1 {
            let witness = contact.unwrap().witness;
            [(x, y), (x + size, y), (x, y + size), (x + size, y + size)]
                .into_iter()
                .all(|(cx, cy)| {
                    surface.witness_distance(
                        [
                            origin[0] + cx as f64 * cell,
                            origin[1] + cy as f64 * cell,
                            o.center_z_mm,
                        ],
                        witness,
                        scan,
                    ) < o.radius_mm - guard
                })
        } else {
            false
        };
        Ok(
            if distance < o.radius_mm - diagonal - guard || inside_prism || size == 1 {
                Cell::Occupy
            } else {
                Cell::Split
            },
        )
    };
    let children = |x: u32, y: u32, size: u32| {
        let half = size / 2;
        [
            (x, y, half),
            (x + half, y, half),
            (x, y + half, half),
            (x + half, y + half, half),
        ]
    };
    let mut scan = SurfaceWork::default();
    let mut occupied = vec![];
    let mut cells = 0;
    // Serial top of the tree (all of it with one worker).
    let target = if workers > 1 {
        16 * workers
    } else {
        usize::MAX
    };
    let mut queue = std::collections::VecDeque::from([(0u32, 0u32, grid)]);
    let mut frontier = vec![];
    while let Some((x, y, size)) = queue.pop_front() {
        cancelled(cancel)?;
        if workers > 1 && queue.len() + 1 >= target {
            frontier.push((x, y, size));
            frontier.extend(queue.drain(..));
            break;
        }
        cells += 1;
        match classify(x, y, size, &mut scan, &|| cancel.is_cancelled())? {
            Cell::Skip => {}
            Cell::Occupy => occupied.push((x, y, size)),
            Cell::Split => queue.extend(children(x, y, size)),
        }
    }
    // Independent subtrees, finished in parallel.
    let subtrees = crate::roughing::parallel::ordered(
        frontier.len(),
        workers,
        cancel,
        |i, signal| {
            let mut scan = SurfaceWork::default();
            let (mut occupied, mut cells) = (vec![], 0usize);
            let mut stack = vec![frontier[i]];
            while let Some((x, y, size)) = stack.pop() {
                if signal.is_cancelled() {
                    return Err("waterline contour preparation cancelled".into());
                }
                cells += 1;
                match classify(x, y, size, &mut scan, signal)? {
                    Cell::Skip => {}
                    Cell::Occupy => occupied.push((x, y, size)),
                    Cell::Split => stack.extend(children(x, y, size)),
                }
            }
            Ok((occupied, cells, scan))
        },
        |_| Ok(()),
    )?;
    for (more, count, work) in subtrees {
        occupied.extend(more);
        cells += count;
        scan = sum(scan, work);
    }
    let mut horizontal = BTreeMap::new();
    let mut vertical = BTreeMap::new();
    for (x, y, size) in occupied {
        cancelled(cancel)?;
        events(&mut horizontal, y, x, x + size, 1);
        events(&mut horizontal, y + size, x, x + size, -1);
        events(&mut vertical, x + size, y, y + size, 1);
        events(&mut vertical, x, y, y + size, -1);
    }
    let mut edges = vec![];
    boundaries(horizontal, true, &mut edges, cancel)?;
    boundaries(vertical, false, &mut edges, cancel)?;
    let rings = trace(&edges, cancel)?;
    let loops = rings
        .into_iter()
        .map(|ring| {
            ring.iter()
                .map(|p| {
                    [
                        origin[0] + p[0] as f64 * cell,
                        origin[1] + p[1] as f64 * cell,
                    ]
                })
                .collect()
        })
        .collect::<Vec<Vec<Point>>>();
    let max_excess = if loops.is_empty() {
        0.
    } else {
        2f64.sqrt() * cell + guard
    };
    let report = LevelReport {
        method: ContourMethod::QuadTree,
        support_queries: 0,
        support_vertices: 0,
        certificate_tests: 0,
        certificate_nodes: 0,
        boolean_operations: 0,
        support_error_mm: 0.,
        boolean_error_mm: 0.,
        tip_z_mm: 0.,
        grid_cells: cells,
        finest_cell_mm: cell,
        raw_edges: edges.len(),
        cut_edges: 0,
        loops: 0,
        removed_vertices: 0,
        max_normal_excess_mm: max_excess,
        work: scan,
    };
    finish_contours(surface, loops, o, report, cancel)
}
fn finish_contours(
    surface: &PreparedSurface,
    rings: Vec<Vec<Point>>,
    o: ContourOptions,
    mut report: LevelReport,
    cancel: &impl CancelSignal,
) -> Result<WaterlineContours, String> {
    let mut loops = vec![];
    let mut simplify_work = SurfaceWork::default();
    let mut check = SurfaceWork::default();
    let mut removed = 0;
    let mut max_excess = report.max_normal_excess_mm;
    for points in rings {
        let points = if o.simplify {
            #[cfg(feature = "profile-planning")]
            let _profile = crate::planning_profile::span("waterline.simplify");
            let (p, n, e) = simplify_ring(surface, &points, o, &mut simplify_work, cancel)?;
            removed += n;
            max_excess = max_excess.max(e);
            p
        } else {
            points
        };
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("waterline.final_admission");
        for (a, b) in points
            .iter()
            .zip(points.iter().cycle().skip(1))
            .take(points.len())
        {
            if !surface.segment_clear(
                [a[0], a[1], o.center_z_mm],
                [b[0], b[1], o.center_z_mm],
                o.protected_radius_mm,
                &mut check,
                cancel,
            )? {
                return Err("waterline continuous contour admission is not clear".into());
            }
        }
        loops.push(points);
    }
    report.work = sum(sum(report.work, simplify_work), check);
    report.cut_edges = loops.iter().map(Vec::len).sum();
    report.loops = loops.len();
    report.removed_vertices = removed;
    report.max_normal_excess_mm = max_excess;
    Ok(WaterlineContours { loops, report })
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WaterlineOperation<G = SurfaceMesh> {
    #[serde(default)]
    pub contour_method: ContourMethod,
    pub id: String,
    pub tool: String,
    pub surface: G,
    pub floor_z_mm: f64,
    pub top_z_mm: f64,
    pub bottom_z_mm: f64,
    pub stepdown_mm: f64,
    pub clearance_z_mm: f64,
    pub contour_error_mm: f64,
    pub safety_margin_mm: f64,
    pub stock_to_leave_mm: f64,
    pub reverse: bool,
    pub simplify: bool,
    pub tolerance_mm: f64,
    pub feed_mm_min: f64,
    pub plunge_mm_min: f64,
    pub spindle_rpm: f64,
    pub work_offset: u8,
    pub tool_number: u32,
    pub length_register: u32,
    pub coolant: Coolant,
    /// Contact (slope) angle limits, degrees from the tool axis: keep only
    /// the parts of each contour whose ball contact slope lies in this range
    /// (steep areas: from e.g. 30 to 90). Default 0..90, whole contours.
    #[serde(default, skip_serializing_if = "crate::finishing::is_zero")]
    pub slope_from_deg: f64,
    #[serde(
        default = "crate::finishing::ninety",
        skip_serializing_if = "crate::finishing::is_ninety"
    )]
    pub slope_to_deg: f64,
    /// Rapid travel height between contours of a level (absolute Z).
    /// Default: clearance. Each level still ends at clearance.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub retract_z_mm: Option<f64>,
    /// Rapid descent stops here and plunge feed begins (absolute Z, above the
    /// stock top). Default: retract.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub feed_z_mm: Option<f64>,
    /// Containment and avoid areas (clamps, fixtures, features to leave alone).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub limits: Option<crate::limits::Limits>,
    /// Descend continuously along each level's contour from the level above
    /// (the lowest level is then cut once more flat). Ball end mills: every
    /// level of an island is already reached by a ramp along its own contour
    /// from the level above (#46), so this changes nothing there; flat and
    /// bull-nose end mills keep this behaviour.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub ramp: bool,
    /// Ball end mills (#46): the ramp a level is reached by, along its own
    /// contour - from the level above it, or from the height the stock is
    /// clear at when the tool comes down to an island - descends at no more
    /// than this angle, at its feed (default the plunge feed), taking as many
    /// laps (or passes along a run) as the angle needs. Only `RampEntry`
    /// (kind "ramp"). None: 3 degrees at the declared plunge feed.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub entry: Option<crate::entry::PocketEntry>,
}
impl<G> WaterlineOperation<G> {
    pub(crate) fn map_geometry<H, E>(
        self,
        f: &mut impl FnMut(G) -> Result<H, E>,
    ) -> Result<WaterlineOperation<H>, E> {
        Ok(WaterlineOperation {
            ramp: self.ramp,
            entry: self.entry,
            limits: self.limits,
            contour_method: self.contour_method,
            id: self.id,
            tool: self.tool,
            surface: f(self.surface)?,
            floor_z_mm: self.floor_z_mm,
            top_z_mm: self.top_z_mm,
            bottom_z_mm: self.bottom_z_mm,
            stepdown_mm: self.stepdown_mm,
            clearance_z_mm: self.clearance_z_mm,
            contour_error_mm: self.contour_error_mm,
            safety_margin_mm: self.safety_margin_mm,
            stock_to_leave_mm: self.stock_to_leave_mm,
            reverse: self.reverse,
            simplify: self.simplify,
            tolerance_mm: self.tolerance_mm,
            feed_mm_min: self.feed_mm_min,
            plunge_mm_min: self.plunge_mm_min,
            spindle_rpm: self.spindle_rpm,
            work_offset: self.work_offset,
            tool_number: self.tool_number,
            length_register: self.length_register,
            coolant: self.coolant,
            slope_from_deg: self.slope_from_deg,
            slope_to_deg: self.slope_to_deg,
            retract_z_mm: self.retract_z_mm,
            feed_z_mm: self.feed_z_mm,
        })
    }
}

impl WaterlineOperation {
    fn settings(&self) -> ProfileSettings {
        ProfileSettings {
            id: self.id.clone(),
            tool: self.tool.clone(),
            // Validated by Heights::resolve when the plan is prepared.
            heights: crate::heights::Heights {
                clearance: self.clearance_z_mm,
                retract: self.retract_z_mm.unwrap_or(self.clearance_z_mm),
                feed: self
                    .feed_z_mm
                    .unwrap_or(self.retract_z_mm.unwrap_or(self.clearance_z_mm)),
            },
            feed: self.feed_mm_min,
            plunge: self.plunge_mm_min,
            tolerance: self.tolerance_mm,
            lead_in: 0.,
            lead_out: 0.,
            spindle: self.spindle_rpm,
            work_offset: self.work_offset,
            tool_number: self.tool_number,
            length_register: self.length_register,
            coolant: self.coolant,
        }
    }
}
#[derive(Serialize)]
pub struct WaterlineReport {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stock_top: Option<FinishingStockTop>,
    pub contour_method: ContourMethod,
    pub certificate_tests: usize,
    pub certificate_nodes: usize,
    pub strategy: &'static str,
    pub geometric_residual_status: ResidualStatus,
    pub levels: usize,
    pub levels_completed: usize,
    pub empty_levels: usize,
    /// Ball end mills: in-between levels on shallow slopes (#46) that cut
    /// something. Skipped when none did.
    #[serde(skip_serializing_if = "is_zero")]
    pub scallop_levels: usize,
    /// Ball end mills: the scallop bound the in-between levels keep: the
    /// scallop of a vertical wall at this stepdown, `R -
    /// sqrt(R^2 - stepdown_mm^2/4)` (R the ball radius; the radius itself
    /// when stepdown^2/4 >= R^2), plus the path tolerance `contour_error_mm`,
    /// capped at the ball radius — a level is added only where it lowers the
    /// scallop by more than the tolerance. None for flat and bull-nose
    /// cutters.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub scallop_mm: Option<f64>,
    /// Ball end mills: below this contact angle (degrees from the tool axis)
    /// the scallop bound is NOT claimed:
    /// `asin(min(1, stepdown_mm / (2^K * D)))`, with K the deepest sub-level
    /// and `D = 2 * sqrt(R^2 - (R - scallop_mm)^2)` the widest spacing along
    /// the surface the bound allows. None for flat and bull-nose cutters.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub scallop_from_deg: Option<f64>,
    /// Loops whose start moved to a clear side entry found along the loop
    /// itself (#46): no point beside the given start had the stock at or
    /// below the plunge height within reach, but a sample along the loop
    /// did, and the loop rotates so it starts there. Absent when none moved.
    #[serde(skip_serializing_if = "is_zero")]
    pub moved_entries: usize,
    /// Loops entered on a ramp along the loop itself (#46): no
    /// point beside the loop within reach had the stock at or below the
    /// plunge height and the part clear, so the tool plunges at the loop
    /// start to the lowest height the stock under the ball there is clear
    /// at (at most the feed height) and descends along the loop to the
    /// level. Rough or rest-rough where these gather; absent when every
    /// loop has a proven side entry.
    #[serde(skip_serializing_if = "is_zero")]
    pub helical_entries: usize,
    /// Ball end mills (#46): the islands (stacks of contours, one per
    /// connected wall region, a merge starting a new one) cut top to bottom
    /// one at a time.
    #[serde(skip_serializing_if = "is_zero")]
    pub stacks: usize,
    /// Stacks whose top-to-bottom order a proof refused (the shank or holder
    /// would meet stock a neighbour still carries): they were finished last,
    /// level by level.
    #[serde(skip_serializing_if = "is_zero")]
    pub stacks_level_ordered: usize,
    /// Levels reached from the level above without a retract: a short link
    /// at the upper level to the lower contour, then a ramp down along it,
    /// then the lower contour cut in full over the ramp.
    #[serde(skip_serializing_if = "is_zero")]
    pub ramp_links: usize,
    /// Links between levels not taken (the lower loop entered after a
    /// retract instead), by the check that refused them: the stock the link
    /// would meet, the link's part clearance, the ramp's part clearance.
    #[serde(skip_serializing_if = "is_zero")]
    pub links_refused_stock: usize,
    #[serde(skip_serializing_if = "is_zero")]
    pub links_refused_link_clearance: usize,
    #[serde(skip_serializing_if = "is_zero")]
    pub links_refused_ramp_clearance: usize,
    /// Links not taken because the open run below is too short for the ramp
    /// down along it and back (#392): shorter than the declared
    /// `min_length_mm`, the shortest span the tool may reverse on, or than
    /// the angle allows within 1000 passes. The run is entered after a
    /// retract.
    #[serde(skip_serializing_if = "is_zero")]
    pub links_refused_short_run: usize,
    /// Open runs entered after a retract whose entry ramp runs along the
    /// level's whole contour from the run's start and back, because the run
    /// itself is too short for it (#392); also counted in
    /// `helical_entries`.
    #[serde(skip_serializing_if = "is_zero")]
    pub contour_ramp_entries: usize,
    /// Entry ramps and level links that meet stock more than three
    /// stepdowns deep where they reach the level (#46) - for a link, beyond
    /// the level gap it descends: unroughed stock, such as a raw blank or a
    /// gap the rough tool could not reach, which the waterline cuts in full
    /// depth from the side. Rough or rest-rough first; absent when none did.
    #[serde(skip_serializing_if = "is_zero")]
    pub deep_stock_entries: usize,
    pub cut_segments: usize,
    pub loops_completed: usize,
    pub requested_bottom_z_mm: f64,
    pub last_level_z_mm: Option<f64>,
    pub contour_error_mm: f64,
    pub safety_margin_mm: f64,
    pub surface_error_mm: f64,
    pub stock_to_leave_mm: f64,
    pub work: SurfaceWork,
    pub level_reports: Vec<LevelReport>,
    pub contour_proof: &'static str,
    pub surface_completion: &'static str,
    /// How the depth below the remaining stock was admitted when that stock
    /// stands higher above `bottom_z_mm` than the cutting length (absent
    /// otherwise: every level is then within the flutes).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reach_proof: Option<&'static str>,
    /// Flat and bull-nose end mills: the largest stated error
    /// (`boundary_error_mm`) of the part shadows the levels so far were
    /// offset from. Each level is offset by what this adds to the mesh's
    /// declared error, on top of the cutter radius, stock to leave and margin.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub shadow_error_mm: Option<f64>,
}
pub struct WaterlinePlan {
    /// Flat or bull-nose end mill: its levels are offsets of the part's
    /// shadow; the corner radius (grown by the offset) of a bull nose.
    flat_target: Option<PreparedRoughingTarget>,
    flat_corner: f64,
    op: WaterlineOperation,
    spec: StockSpec,
    surface: PreparedSurface,
    radius: f64,
    inflated: f64,
    protected: f64,
    initial: Vec3,
    position: Vec3,
    next_id: u64,
    expected_generation: Option<u64>,
    /// Every level in cutting order: main levels (depth 0) at today's
    /// heights, then the in-between scallop levels inserted between them
    /// (#46). Flat and bull-nose cutters have main levels only.
    schedule: Vec<LevelSlot>,
    /// Position in `schedule`.
    slot: usize,
    profile: Option<ProfileMotion>,
    finished_loops: usize,
    drained: bool,
    report: WaterlineReport,
    stock_ceiling: Option<StockCeiling>,
    /// When the remaining stock stands higher above `bottom_z_mm` than the
    /// cutting length: every feed move is checked against the committed
    /// stock under the neck, shank and holder sections (issue #161).
    move_depth: Option<crate::reach::MoveDepth>,
    /// Contours of upcoming levels, computed in parallel ahead of use.
    /// Levels depend only on the surface and their Z, never on stock.
    prefetched: std::collections::VecDeque<WaterlineContours>,
    limits: Option<(f64, f64)>,
    /// Machining limits (containment, avoid areas).
    region_limits: Option<crate::limits::PreparedLimits>,
    /// The ramp (#46): its angle is the steepest a ramp along a contour
    /// descends (`loop_ramp`, `run_ramp`), its feed the ramp's feed. Without a declared
    /// entry, 3 degrees at the declared plunge feed.
    ramp_entry: crate::entry::RampEntry,
    /// Ball end mills (#46): every level's pieces grouped into stacks, and
    /// the moves of the piece being cut. Built on the first motion.
    emission: Option<Emission>,
}
fn is_zero(n: &usize) -> bool {
    *n == 0
}
/// Per loop the entry: a point beside the loop the tool plunges at (None
/// for a loop entered on a helix along itself, from the height in the
/// second vector), and how many loop starts moved and how many loops
/// entered on a helix.
type Entries = (Vec<Option<[f64; 2]>>, Vec<Option<f64>>, usize, usize);
/// A loop whose last stroke ends where its first starts: a contour ring.
/// Runs the slope or region limits cut out of a contour are open, and only
/// a closed loop can move its start.
fn loop_is_closed(edges: &[Stroke]) -> bool {
    let a = edges[0].curve(0.).start();
    let b = edges[edges.len() - 1].curve(0.).end();
    (a[0] - b[0]).hypot(a[1] - b[1]) <= 1e-9
}
/// The point and travel tangent at arc length `s` along a loop; the stroke
/// containing `s` decides the tangent (waterline loops are polylines).
fn loop_point_tangent(edges: &[Stroke], s: f64) -> ([f64; 2], [f64; 2]) {
    let mut acc = 0.;
    for e in edges {
        let l = stroke_length(e);
        if l > 0. && acc + l >= s {
            let c = e.curve(0.);
            let (a, b) = (c.start(), c.end());
            let t = (s - acc) / l;
            return (
                [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])],
                e.tangent(false),
            );
        }
        acc += l;
    }
    let e = edges[edges.len() - 1];
    let c = e.curve(0.).end();
    ([c[0], c[1]], e.tangent(true))
}
/// Move a closed loop's start to arc length `s`: the order and direction
/// stay, the stroke containing `s` is split there (#46).
fn rotate_loop(edges: &mut Vec<Stroke>, s: f64) {
    let mut acc = 0.;
    for k in 0..edges.len() {
        let l = stroke_length(&edges[k]);
        if l > 0. && acc + l >= s {
            let c = edges[k].curve(0.);
            let (a, b) = (c.start(), c.end());
            let t = (s - acc) / l;
            let q = [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
            // A split within 1e-9 of a vertex is the vertex itself (#392):
            // the stroke not split off is dropped below, so the loop must
            // start and end exactly there, or a second rotation (the flat
            // lap after a ramp) leaves a gap of a rounding error mid-loop
            // that the stream refuses.
            let q = if (q[0] - b[0]).hypot(q[1] - b[1]) <= 1e-9 {
                [b[0], b[1]]
            } else if (a[0] - q[0]).hypot(a[1] - q[1]) <= 1e-9 {
                [a[0], a[1]]
            } else {
                q
            };
            let mut rotated: Vec<Stroke> = vec![];
            if (q[0] - b[0]).hypot(q[1] - b[1]) > 1e-9 {
                rotated.push(Stroke::line(q, [b[0], b[1]]));
            }
            rotated.extend(edges[k + 1..].iter().copied());
            rotated.extend(edges[..k].iter().copied());
            if (a[0] - q[0]).hypot(a[1] - q[1]) > 1e-9 {
                rotated.push(Stroke::line([a[0], a[1]], q));
            }
            *edges = rotated;
            return;
        }
        acc += l;
    }
}
/// How far out (in cutter radii) a loop's side entry is searched.
const SIDE_ENTRY_REACH_RADII: f64 = 4.;

/// One level of the cutting schedule: a main level (`depth` 0) at today's
/// height, or an in-between scallop level of depth k between two main levels
/// (#46). Cutting order is descending z.
struct LevelSlot {
    z: f64,
    depth: usize,
    /// Ramp mode: the main level a main level descends from (None for the
    /// first main level and for sub-levels, which are cut flat).
    ramp_from: Option<f64>,
    /// Ramp mode: the lowest main level is cut once more flat.
    last_main: bool,
    /// Sub-levels only: a sample is kept while the sine of its greatest
    /// contact angle stays under this — the parent level spacing of the
    /// level's gap over the widest allowed spacing `D`, minus the 1e-9 tie
    /// (#46). Infinity on main levels, which keep every sample.
    keep_below: f64,
}

/// One piece a ball waterline level cuts (#46): a closed loop or an open run
/// at tip height `z`, and the closed contour (`ring`) it was cut from.
struct Piece {
    z: f64,
    ring: usize,
    closed: bool,
    strokes: Vec<Stroke>,
}
/// One island's (or hole's) pieces, top to bottom (#46), cut after its
/// parents (the stacks a merged contour joins).
struct Stack {
    pieces: Vec<usize>,
    parents: Vec<usize>,
    next: usize,
    deferred: bool,
}
impl Stack {
    fn done(&self) -> bool {
        self.next == self.pieces.len()
    }
}
/// A move waiting to be handed out: curve, kind and feed (None: rapid).
type Queued = (Curve, MoveKind, Option<f64>);
/// A piece's moves, and (ramp link, moved entries, helical entries, links
/// refused by [the stock met, the link's part clearance, the ramp's part
/// clearance, an open run too short for its ramp], whether the entry or
/// link met stock deeper than a stepdown, open runs entered by a ramp along
/// their contour).
type Job = (Vec<Queued>, (bool, usize, usize, [usize; 4], bool, usize));
/// Ball end mills (#46): the pieces of every level in stacks, the stack
/// being cut, the piece just cut (the tool stands at its end), whether the
/// emitter has fallen back to level order, and the moves of the piece being
/// cut.
struct Emission {
    pieces: Vec<Piece>,
    /// Every level's whole contours, cut direction (`Piece::ring` indexes
    /// them): an open run too short for a ramp along itself is entered by a
    /// ramp along its contour (#392).
    rings: Vec<Vec<[f64; 2]>>,
    stacks: Vec<Stack>,
    current: Option<usize>,
    last: Option<usize>,
    level_order: bool,
    queue: VecDeque<Queued>,
    finished: bool,
}
/// Groups the pieces into stacks (#46): top-down over the levels, a contour
/// joins the stack whose contour at that stack's latest level it overlaps
/// (islands grow downward, holes shrink); a contour overlapping none starts
/// a stack, and one overlapping several starts a new stack cut after them
/// (a merge). Pieces keep their level order within a stack.
fn build_stacks(rings: &[Vec<[f64; 2]>], ring_slots: &[usize], pieces: &[Piece]) -> Vec<Stack> {
    let boxes: Vec<[f64; 4]> = rings.iter().map(|r| ring_box(r)).collect();
    let mut has_piece = vec![false; rings.len()];
    for p in pieces {
        has_piece[p.ring] = true;
    }
    let mut stacks: Vec<Stack> = vec![];
    let mut frontier: Vec<Vec<usize>> = vec![];
    let mut ring_stack = vec![usize::MAX; rings.len()];
    let mut i = 0;
    while i < rings.len() {
        let mut j = i;
        while j < rings.len() && ring_slots[j] == ring_slots[i] {
            j += 1;
        }
        let mut joined: Vec<(usize, usize)> = vec![];
        let mut merged: Vec<usize> = vec![];
        for r in (i..j).filter(|&r| has_piece[r]) {
            let hits: Vec<usize> = (0..stacks.len())
                .filter(|&s| {
                    frontier[s]
                        .iter()
                        .any(|&f| rings_overlap(&rings[f], &boxes[f], &rings[r], &boxes[r]))
                })
                .collect();
            let s = if hits.len() == 1 {
                hits[0]
            } else {
                stacks.push(Stack {
                    pieces: vec![],
                    parents: hits.clone(),
                    next: 0,
                    deferred: false,
                });
                frontier.push(vec![]);
                merged.extend(hits);
                stacks.len() - 1
            };
            ring_stack[r] = s;
            joined.push((s, r));
        }
        let mut touched: Vec<usize> = joined.iter().map(|&(s, _)| s).collect();
        touched.sort_unstable();
        touched.dedup();
        for &s in &touched {
            frontier[s] = joined
                .iter()
                .filter(|&&(t, _)| t == s)
                .map(|&(_, r)| r)
                .collect();
        }
        for s in merged {
            if !touched.contains(&s) {
                frontier[s].clear();
            }
        }
        i = j;
    }
    for (k, p) in pieces.iter().enumerate() {
        stacks[ring_stack[p.ring]].pieces.push(k);
    }
    stacks
}
fn ring_box(ring: &[[f64; 2]]) -> [f64; 4] {
    ring.iter().fold(
        [
            f64::INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NEG_INFINITY,
        ],
        |b, p| {
            [
                b[0].min(p[0]),
                b[1].min(p[1]),
                b[2].max(p[0]),
                b[3].max(p[1]),
            ]
        },
    )
}
/// Even-odd point-in-polygon test of a closed ring.
fn point_in_ring(p: [f64; 2], ring: &[[f64; 2]]) -> bool {
    let mut inside = false;
    for (a, b) in ring
        .iter()
        .zip(ring.iter().cycle().skip(1))
        .take(ring.len())
    {
        if (a[1] > p[1]) != (b[1] > p[1])
            && p[0] < a[0] + (p[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1])
        {
            inside = !inside;
        }
    }
    inside
}
fn segments_cross(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> bool {
    let cross = |o: [f64; 2], p: [f64; 2], q: [f64; 2]| {
        (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0])
    };
    let (d1, d2) = (cross(c, d, a), cross(c, d, b));
    let (d3, d4) = (cross(a, b, c), cross(a, b, d));
    (d1 > 0.) != (d2 > 0.) && (d3 > 0.) != (d4 > 0.)
}
/// Whether two closed contours' regions overlap: one contains a vertex of
/// the other, or their edges cross.
fn rings_overlap(a: &[[f64; 2]], ba: &[f64; 4], b: &[[f64; 2]], bb: &[f64; 4]) -> bool {
    if ba[0] > bb[2] || bb[0] > ba[2] || ba[1] > bb[3] || bb[1] > ba[3] {
        return false;
    }
    if point_in_ring(a[0], b) || point_in_ring(b[0], a) {
        return true;
    }
    a.iter()
        .zip(a.iter().cycle().skip(1))
        .take(a.len())
        .any(|(p, q)| {
            b.iter()
                .zip(b.iter().cycle().skip(1))
                .take(b.len())
                .any(|(r, s)| segments_cross(*p, *q, *r, *s))
        })
}
/// The arc length and point of a loop (lines) nearest to `p`.
fn nearest_on_loop(edges: &[Stroke], p: [f64; 2]) -> (f64, [f64; 2]) {
    let mut best = (f64::INFINITY, 0., edges[0].a);
    let mut acc = 0.;
    for e in edges {
        let l = stroke_length(e);
        let (a, b) = (e.a, e.b);
        let t = if l > 0. {
            (((p[0] - a[0]) * (b[0] - a[0]) + (p[1] - a[1]) * (b[1] - a[1])) / (l * l))
                .clamp(0., 1.)
        } else {
            0.
        };
        let q = [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
        let d = (q[0] - p[0]).hypot(q[1] - p[1]);
        if d < best.0 {
            best = (d, acc + t * l, q);
        }
        acc += l;
    }
    (best.1, best.2)
}
/// A ramp along a loop from its start, `length` long, descending linearly
/// from `from` to `to` (#46); it ends exactly at `to`.
fn ramp_curves(edges: &[Stroke], from: f64, to: f64, length: f64) -> Vec<Curve> {
    let mut out = vec![];
    let mut acc = 0.;
    for e in edges {
        let l = stroke_length(e);
        if l <= 0. {
            continue;
        }
        let s1 = (acc + l).min(length);
        let t = (s1 - acc) / l;
        let mut q = [
            e.a[0] + t * (e.b[0] - e.a[0]),
            e.a[1] + t * (e.b[1] - e.a[1]),
        ];
        // Snapped like `rotate_loop` snaps its split: the flat lap that
        // follows starts exactly where the ramp ends.
        if (q[0] - e.b[0]).hypot(q[1] - e.b[1]) <= 1e-9 {
            q = e.b;
        }
        let z0 = from + (to - from) * acc / length;
        let z1 = from + (to - from) * s1 / length;
        out.push(Curve::Line {
            start: [e.a[0], e.a[1], z0],
            end: [q[0], q[1], z1],
        });
        acc += l;
        if acc >= length {
            break;
        }
    }
    if let Some(Curve::Line { end, .. }) = out.last_mut() {
        end[2] = to;
    }
    out
}
/// The longest gap (in ball radii) an in-between level may leave between
/// the fragments it keeps of a contour for the contour to be cut whole (#46).
const BRIDGE_RADII: f64 = 4.;
/// How many stepdowns of stock an entry may meet where it reaches its level,
/// or a link beyond the level gap it descends, before it counts as entering
/// unroughed stock (#46): a rough in steps up to three times the finishing
/// stepdown leaves that much (plus its allowance) on a slope.
const DEEP_STEPDOWNS: f64 = 3.;
/// Whether the runs an in-between level keeps of a closed contour leave only
/// gaps along it shorter than `max_gap` (#46), measured along the contour
/// from the end of each run to the start of the next.
fn runs_bridge(ring: &[[f64; 2]], runs: &[Vec<[f64; 2]>], max_gap: f64) -> bool {
    let strokes: Vec<Stroke> = ring
        .iter()
        .zip(ring.iter().cycle().skip(1))
        .take(ring.len())
        .map(|(a, b)| Stroke::line(*a, *b))
        .collect();
    let total: f64 = strokes.iter().map(stroke_length).sum();
    let mut spans: Vec<(f64, f64)> = runs
        .iter()
        .filter(|r| !r.is_empty())
        .map(|r| {
            (
                nearest_on_loop(&strokes, r[0]).0,
                nearest_on_loop(&strokes, r[r.len() - 1]).0,
            )
        })
        .collect();
    if total <= 0. || spans.is_empty() {
        return false;
    }
    spans.sort_by(|a, b| a.0.total_cmp(&b.0));
    (0..spans.len()).all(|i| {
        let next = spans[(i + 1) % spans.len()].0;
        (next - spans[i].1).rem_euclid(total) < max_gap
    })
}
/// The fragments an in-between level keeps of one run the slope or region
/// limits allow (#46), cut as one piece when every gap between them along
/// the run is shorter than `max_gap`: the stretch of `run` from the first
/// fragment's start to the last one's end. The gaps lie inside the limits
/// (only the scallop rule left them out), so cutting them stays within
/// what the operation may cut. None when the fragments do not bridge.
/// The arc length along `edges` (an open polyline) of the point nearest `p`
/// at or after arc length `from`.
fn position_from(edges: &[Stroke], p: [f64; 2], from: f64) -> f64 {
    let mut best = (f64::INFINITY, from);
    let mut acc = 0.;
    for e in edges {
        let l = stroke_length(e);
        if acc + l >= from - 1e-9 {
            let (a, b) = (e.a, e.b);
            let lo = if l > 0. {
                ((from - acc) / l).clamp(0., 1.)
            } else {
                0.
            };
            let t = if l > 0. {
                (((p[0] - a[0]) * (b[0] - a[0]) + (p[1] - a[1]) * (b[1] - a[1])) / (l * l))
                    .clamp(lo, 1.)
            } else {
                0.
            };
            let q = [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
            let d = (q[0] - p[0]).hypot(q[1] - p[1]);
            if d < best.0 - 1e-12 {
                best = (d, acc + t * l);
            }
        }
        acc += l;
    }
    best.1
}
fn bridge_open(run: &[[f64; 2]], kept: &[Vec<[f64; 2]>], max_gap: f64) -> Option<Vec<[f64; 2]>> {
    if kept.len() < 2 {
        return None;
    }
    let strokes: Vec<Stroke> = run.windows(2).map(|w| Stroke::line(w[0], w[1])).collect();
    // The fragments lie along the run in order: each point is placed at or
    // after the previous one, so a run that closes on itself (its last point
    // its first) does not map a fragment ending at the seam back to its start.
    let mut last = 0.;
    let spans: Vec<(f64, f64)> = kept
        .iter()
        .map(|k| {
            let a = position_from(&strokes, k[0], last);
            last = position_from(&strokes, k[k.len() - 1], a);
            (a, last)
        })
        .collect();
    if !spans
        .windows(2)
        .all(|w| w[1].0 >= w[0].1 && w[1].0 - w[0].1 < max_gap)
    {
        return None;
    }
    let (from, to) = (spans[0].0, spans[spans.len() - 1].1);
    let mut out = vec![kept[0][0]];
    let mut acc = 0.;
    for (e, w) in strokes.iter().zip(run.windows(2)) {
        acc += stroke_length(e);
        if acc > from + 1e-9 && acc < to - 1e-9 {
            out.push(w[1]);
        }
    }
    let last = kept[kept.len() - 1][kept[kept.len() - 1].len() - 1];
    if out.last() != Some(&last) {
        out.push(last);
    }
    Some(out)
}
/// How near a vertex of a loop a split (where a ramp ends, where a link
/// meets the loop, where a moved start lies) is moved onto it (#46): a
/// sliver shorter than this rounds to nothing in a program posted at three
/// decimals (0.0005 mm), and nothing so short is worth a move.
const SNAP_MM: f64 = 0.005;
/// An arc length along a loop moved onto a vertex when within `SNAP_MM` of
/// one: the ramp then ends, and the flat lap after it starts, exactly at
/// that vertex, never leaving a sliver no post can express.
/// The descent onto an open run from `from` to `to` (#46): an open run ends
/// away from its start, so along its first `length` to half the depth and
/// back over the same stretch to the level, ending at its start (the flat
/// pass then cuts over all of it).
fn out_and_back(edges: &[Stroke], from: f64, to: f64, length: f64) -> Vec<Curve> {
    let mid = (from + to) / 2.;
    // A stretch too short to post descends over the whole run instead.
    let length = if length > SNAP_MM {
        length
    } else {
        edges.iter().map(stroke_length).sum()
    };
    let out = ramp_curves(edges, from, mid, length);
    // The same points back: a point `s` along the run is at `from - (from
    // - mid) s / length` going out and at `to + (mid - to) s / length`
    // coming back.
    let back_z = |z: f64| to + (mid - to) * (from - z) / (from - mid);
    let back: Vec<Curve> = out
        .iter()
        .rev()
        .map(|c| {
            let (a, b) = (c.start(), c.end());
            Curve::Line {
                start: [b[0], b[1], back_z(b[2])],
                end: [a[0], a[1], back_z(a[2])],
            }
        })
        .collect();
    let mut curves = out;
    curves.extend(back);
    if let Some(Curve::Line { end, .. }) = curves.last_mut() {
        end[2] = to;
    }
    curves
}
fn snap_to_vertex(edges: &[Stroke], length: f64) -> f64 {
    let mut acc = 0.;
    for e in edges {
        if (length - acc).abs() <= SNAP_MM {
            return acc;
        }
        acc += stroke_length(e);
    }
    if (length - acc).abs() <= SNAP_MM {
        acc
    } else {
        length
    }
}
/// Where to split a ramp `length` long along `edges` (#46): never shorter
/// than `length` (a shorter ramp would be steeper than the angle), moved up
/// onto the next vertex when that lies within `SNAP_MM` or when the vertex
/// before lies within it (no sliver either side); at most the whole length.
/// Positive for any positive `length`.
fn ramp_split(edges: &[Stroke], length: f64) -> f64 {
    let mut acc = 0.;
    for e in edges {
        let next = acc + stroke_length(e);
        if next >= length {
            return if next - length <= SNAP_MM || length - acc < SNAP_MM {
                next
            } else {
                length
            };
        }
        acc = next;
    }
    acc
}
/// A closed loop cut flat at `z` once round, from arc length `s` (where a
/// ramp along it ended) back to `s`: the lap passes over the ramp again.
fn flat_lap(edges: &[Stroke], s: f64, z: f64) -> Vec<Curve> {
    let total: f64 = edges.iter().map(stroke_length).sum();
    let mut lap = edges.to_vec();
    if s > 1e-9 && s < total - 1e-9 {
        rotate_loop(&mut lap, s);
    }
    lap.iter().map(|e| e.curve(z)).collect()
}
/// Bounds cutter penetration on a horizontal link. Where the finishing ball
/// reaches at least as deep at the same XY, the finishing cut itself witnesses
/// the load bound, even if the stock height there is uncertain.
struct BallLink {
    start: Vec3,
    end: [f64; 2],
    unit: [f64; 2],
    length: f64,
    radius: f64,
    lap: Option<Vec3>,
    limit: f64,
}

impl BallLink {
    fn cell_depth(&self, c: [f64; 2], half: [f64; 2], height: f64, guard: f64, depth: u8) -> f64 {
        let [hx, hy] = half;
        let [ux, uy] = self.unit;
        let [ax, ay, z] = self.start;
        let [bx, by] = self.end;
        let r = self.radius;
        let (x, y) = (c[0] - ax, c[1] - ay);
        let projection = ux.mul_add(x, uy * y);
        let span = hx * ux.abs() + hy * uy.abs() + guard;
        if projection + span <= 0. {
            return 0.;
        } // wholly behind the starting pose
        let t = projection.clamp(0., self.length);
        let d = ((x - t * ux).hypot(y - t * uy) - hx.hypot(hy))
            .max((uy * x - ux * y).abs() - hx * uy.abs() - hy * ux.abs())
            .max((ax.min(bx) - c[0]).max(c[0] - ax.max(bx)) - hx)
            .max((ay.min(by) - c[1]).max(c[1] - ay.max(by)) - hy);
        let d = (d - guard).max(0.);
        if d > r {
            return 0.;
        }
        let ball_height = |d: f64| r - ((r - d) * (r + d)).sqrt();
        let bottom = z + ball_height(d) - guard;
        let far = (x.abs() + hx).hypot(y.abs() + hy) + guard;
        let top = if far < r {
            height.min(z + ball_height(far) + guard)
        } else {
            height
        };
        let upper = (top - bottom).max(0.);
        if upper <= self.limit {
            return upper;
        }
        if let Some([px, py, pz]) = self.lap {
            // Beyond the segment end its cap and the arrival disk have the
            // same XY axis. Otherwise compare the worst heights on the box.
            if [px, py] == self.end && projection - span >= self.length && pz <= z {
                return 0.;
            }
            let far = ((c[0] - px).abs() + hx).hypot((c[1] - py).abs() + hy) + guard;
            if far < r && pz + ball_height(far) + guard <= bottom {
                return 0.;
            }
        }
        // Refine an undecided cell, never its stock bounds. Exhaustion
        // returns the conservative upper bound and refuses the link.
        if depth == 0 {
            return upper;
        }
        let sub = [hx / 2., hy / 2.];
        let mut worst = 0f64;
        for x in [-sub[0], sub[0]] {
            for y in [-sub[1], sub[1]] {
                let cut = self.cell_depth([c[0] + x, c[1] + y], sub, height, guard, depth - 1);
                if cut > self.limit {
                    return upper;
                }
                worst = worst.max(cut);
            }
        }
        worst
    }
}

impl WaterlinePlan {
    pub fn prepare_with_stock(
        op: WaterlineOperation,
        tool: &ToolAssembly,
        stock: &Stock,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        if stock.spec().max_mm[2] - op.bottom_z_mm
            < tool.cutter.cutting_length_mm - 16. * op.tolerance_mm
        {
            return Self::prepare(op, tool, stock.spec(), initial, first_id, cancel);
        }
        let (envelope, ceiling) = StockCeiling::prepare(stock, cancel)?;
        // Stock standing higher above the lowest level than the cutting
        // length need not be cut that deep: levels run top-down, so the
        // levels above leave the finished wall beside the tool. Check the
        // depth each feed move actually cuts instead (as parallel, #88).
        let per_move = envelope.max_mm[2] - op.bottom_z_mm
            >= tool.cutter.cutting_length_mm - 16. * op.tolerance_mm;
        let move_depth = per_move
            .then(|| crate::reach::MoveDepth::new(&op.id, tool, op.tolerance_mm))
            .transpose()?;
        let mut plan =
            Self::prepare_depth(op, tool, &envelope, initial, first_id, per_move, cancel)?;
        if move_depth.is_some() {
            plan.report.reach_proof = Some(
                "neck/shank/holder above the committed stock under them on every feed move, at the move's lowest tip",
            );
        }
        plan.move_depth = move_depth;
        plan.spec = stock.spec().clone();
        plan.expected_generation = Some(stock.generation());
        plan.report.stock_top = Some(ceiling.report.clone());
        plan.stock_ceiling = Some(ceiling);
        Ok(plan)
    }
    pub fn prepare(
        op: WaterlineOperation,
        tool: &ToolAssembly,
        spec: &StockSpec,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        Self::prepare_depth(op, tool, spec, initial, first_id, false, cancel)
    }
    /// `per_move_depth`: the depth below the stock top is checked on every
    /// feed move against the committed stock (`move_depth`), not here.
    fn prepare_depth(
        op: WaterlineOperation,
        tool: &ToolAssembly,
        spec: &StockSpec,
        initial: Vec3,
        first_id: u64,
        per_move_depth: bool,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        cancelled(cancel)?;
        spec.grid()?;
        let prepared = tool.prepare()?;
        let cutter = prepared.cutter().definition();
        let radius = cutter.diameter_mm / 2.;
        let guard = 16. * op.tolerance_mm;
        let inflated =
            radius + op.stock_to_leave_mm + op.surface.surface_error_mm + op.safety_margin_mm;
        let protected = inflated - op.safety_margin_mm + guard;
        let values = [
            op.floor_z_mm,
            op.top_z_mm,
            op.bottom_z_mm,
            op.stepdown_mm,
            op.clearance_z_mm,
            op.contour_error_mm,
            op.safety_margin_mm,
            op.stock_to_leave_mm,
            op.tolerance_mm,
            op.feed_mm_min,
            op.plunge_mm_min,
            op.spindle_rpm,
        ];
        let outside = op
            .surface
            .triangles
            .iter()
            .flatten()
            .any(|p| (0..2).any(|k| p[k] < spec.min_mm[k] || p[k] > spec.max_mm[k]));
        let limits =
            crate::finishing::slope_limits("waterline", op.slope_from_deg, op.slope_to_deg)?;
        let region_limits = op
            .limits
            .as_ref()
            .filter(|l| !l.is_empty())
            .map(|l| crate::limits::PreparedLimits::new(l, radius, op.tolerance_mm))
            .transpose()?;
        let ramp_entry = match &op.entry {
            // No declared entry: ramp at 3 degrees at the declared plunge feed (#46);
            // an along-contour descent uses the same entry-feed policy as
            // an explicit RampEntry with no feed override.
            None => crate::entry::RampEntry {
                angle_rad: 3f64.to_radians(),
                min_length_mm: None,
                feed_mm_min: None,
            },
            Some(crate::entry::PocketEntry::Ramp(r)) => {
                r.validate(op.tolerance_mm, op.plunge_mm_min)?;
                r.clone()
            }
            Some(_) => {
                return Err(format!(
                    "waterline operation {}: only a ramp entry (RampEntry) is accepted: a ball waterline reaches every level by a ramp or helix along its own contour",
                    op.id
                ));
            }
        };
        crate::heights::require(
            "waterline",
            [
                (
                    valid_id(&op.id),
                    format!("id {:?} is not a valid identifier", op.id),
                ),
                (
                    op.tool == tool.id,
                    format!(
                        "tool {:?} does not match the assembly {:?}",
                        op.tool, tool.id
                    ),
                ),
                (
                    matches!(
                        cutter.shape,
                        CutterShape::Ball | CutterShape::Flat | CutterShape::Bull { .. }
                    ) && cutter.center_cutting,
                    "this operation needs a center-cutting ball, flat or bull-nose end mill".into(),
                ),
                (
                    !matches!(cutter.shape, CutterShape::Flat | CutterShape::Bull { .. })
                        || (op.slope_from_deg == 0. && op.slope_to_deg == 90.),
                    "slope limits need a ball end mill (a flat end mill has no single contact normal)"
                        .into(),
                ),
                (
                    values.iter().all(|v| v.is_finite()),
                    "all numeric parameters must be finite".into(),
                ),
                (
                    (1e-7..=0.001).contains(&op.tolerance_mm),
                    format!(
                        "tolerance_mm {} must be within 1e-7..0.001",
                        op.tolerance_mm
                    ),
                ),
                (
                    (0.001..=0.1).contains(&op.contour_error_mm)
                        && op.contour_error_mm > 4. * guard,
                    format!(
                        "contour_error_mm {} must be within 0.001..0.1",
                        op.contour_error_mm
                    ),
                ),
                (
                    (0.001..=0.1).contains(&op.safety_margin_mm)
                        && op.safety_margin_mm > 4. * guard,
                    format!(
                        "safety_margin_mm {} must be within 0.001..0.1",
                        op.safety_margin_mm
                    ),
                ),
                (
                    (0. ..=radius).contains(&op.stock_to_leave_mm),
                    format!(
                        "stock_to_leave_mm {} must be between 0 and the cutter radius {radius}",
                        op.stock_to_leave_mm
                    ),
                ),
                (
                    op.floor_z_mm >= spec.min_mm[2],
                    format!(
                        "floor_z_mm {} must not be below the stock bottom {}",
                        op.floor_z_mm, spec.min_mm[2]
                    ),
                ),
                (
                    op.bottom_z_mm >= op.floor_z_mm + inflated - radius + guard
                        && op.bottom_z_mm <= op.top_z_mm,
                    format!(
                        "bottom_z_mm {} must be between floor_z_mm {} (plus stock to leave and margin) and top_z_mm {}",
                        op.bottom_z_mm, op.floor_z_mm, op.top_z_mm
                    ),
                ),
                (
                    op.stepdown_mm > guard,
                    format!("stepdown_mm {} must be positive", op.stepdown_mm),
                ),
                (
                    op.top_z_mm < op.clearance_z_mm - guard
                        && op.clearance_z_mm > spec.max_mm[2] + inflated - radius + guard
                        && op.clearance_z_mm.abs() <= 1e6,
                    format!(
                        "clearance_z_mm {} must be above top_z_mm {} and the stock top {} (plus stock to leave and margin)",
                        op.clearance_z_mm, op.top_z_mm, spec.max_mm[2]
                    ),
                ),
                (
                    per_move_depth
                        || spec.max_mm[2] - op.bottom_z_mm < cutter.cutting_length_mm - guard,
                    crate::reach::ReachRefusal::cutting_length(&op.id, &op.tool, spec.max_mm[2], op.bottom_z_mm, cutter.cutting_length_mm).error(format_args!(
                        "depth {} below the stock top exceeds the cutting length {}",
                        spec.max_mm[2] - op.bottom_z_mm,
                        cutter.cutting_length_mm
                    )),
                ),
                (
                    initial.iter().all(|v| v.is_finite() && v.abs() <= 1e6),
                    "start position must be finite".into(),
                ),
                (
                    initial[2] >= op.clearance_z_mm,
                    format!(
                        "start Z {} is below clearance_z_mm {}",
                        initial[2], op.clearance_z_mm
                    ),
                ),
                (
                    op.feed_mm_min > 0. && op.plunge_mm_min > 0. && op.spindle_rpm > 0.,
                    format!(
                        "feed_mm_min {}, plunge_mm_min {} and spindle_rpm {} must be positive",
                        op.feed_mm_min, op.plunge_mm_min, op.spindle_rpm
                    ),
                ),
                (
                    (1..=9).contains(&op.work_offset),
                    format!("work_offset {} must be 1..9", op.work_offset),
                ),
                (
                    op.tool_number > 0 && op.length_register > 0,
                    "tool_number and length_register must be positive".into(),
                ),
                (first_id != 0, "motion ID budget exhausted".into()),
                (
                    !outside,
                    "the surface extends outside the stock in X or Y".into(),
                ),
            ],
        )?;
        crate::heights::Heights::resolve(
            op.clearance_z_mm,
            op.retract_z_mm,
            op.feed_z_mm,
            spec.max_mm[2] + inflated - radius,
            guard,
        )?;
        let levels = ((op.top_z_mm - op.bottom_z_mm) / op.stepdown_mm).ceil() as usize + 1;
        // Main level tip heights, highest first: today's levels, unchanged.
        let main_z: Vec<f64> = (0..levels)
            .map(|i| {
                if i + 1 == levels {
                    op.bottom_z_mm
                } else {
                    (op.top_z_mm - i as f64 * op.stepdown_mm).max(op.bottom_z_mm)
                }
            })
            .collect();
        let mut schedule: Vec<LevelSlot> = (0..levels)
            .map(|i| LevelSlot {
                z: main_z[i],
                depth: 0,
                ramp_from: (i > 0).then(|| main_z[i - 1]),
                last_main: i + 1 == levels,
                keep_below: f64::INFINITY,
            })
            .collect();
        // In-between levels on shallow slopes (#46), ball end mills only. On
        // a slope at contact angle t the passes of levels stepdown apart are
        // stepdown / sin(t) apart along the surface, so fillets, domes and
        // shallow slopes get a large scallop. The bound is the scallop of a
        // vertical wall at this stepdown plus the path tolerance: a level is
        // added only where it lowers the scallop by more than
        // contour_error_mm. The widest spacing along the surface that keeps
        // the bound is D; between two main levels, sub-levels of depth k
        // (halving) keep only the parts whose parent spacing along the
        // surface was still too wide (`sin(t) < P / D - 1e-9`, P the parent
        // spacing of that gap), which brings the spacing under D and the
        // scallop under scallop_mm.
        let (scallop_mm, scallop_from_deg) = if matches!(cutter.shape, CutterShape::Ball) {
            let quarter = op.stepdown_mm * op.stepdown_mm / 4.;
            let wall = if quarter >= radius * radius {
                radius
            } else {
                radius - (radius * radius - quarter).sqrt()
            };
            let scallop = (wall + op.contour_error_mm).min(radius);
            // The widest spacing along the surface that still leaves the
            // bound (`scallop` is at most the radius, so the square root is
            // real): the flat-surface ball scallop at spacing D is
            // R - sqrt(R^2 - D^2/4) = scallop.
            let d = 2. * (radius * radius - (radius - scallop) * (radius - scallop)).sqrt();
            // Deepest useful sub-level: a level spacing smaller than the
            // scallop itself buys nothing. When the bound reaches the ball
            // radius (a stepdown of the ball diameter or more) no sub-level
            // can lower it: none are added, so K = 0.
            let mut k_max = 0usize;
            if scallop < radius {
                while op.stepdown_mm / 2f64.powi(k_max as i32 + 1) >= scallop {
                    k_max += 1;
                }
            }
            // A gap gets depth-k sub-levels only if some upward-facing
            // triangle shallow enough to need them (0 < sin(t) <
            // (za - zb) / 2^(k-1) / D - 1e-9 for that gap, with the same
            // 1e-9 tie as the keep test; exactly horizontal ones skipped)
            // can be contacted at a level of the gap: a ball contacting a
            // triangle at tip z has z in the triangle's [zmin - R, zmax].
            // Conservative: a kept sample's contact is on such a triangle,
            // so no gap that could keep samples is skipped.
            let gaps = levels - 1;
            if k_max > 0 && gaps > 0 {
                let mut eligible = vec![false; gaps * k_max];
                for tri in op.surface.triangles.iter() {
                    let (a, b, c) = (tri[0], tri[1], tri[2]);
                    let e1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
                    let e2 = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
                    let n = [
                        e1[1] * e2[2] - e1[2] * e2[1],
                        e1[2] * e2[0] - e1[0] * e2[2],
                        e1[0] * e2[1] - e1[1] * e2[0],
                    ];
                    let len = n[0].hypot(n[1]).hypot(n[2]);
                    if len <= 1e-12 || n[2] <= 0. {
                        // Degenerate, or facing down.
                        continue;
                    }
                    let sin_t = (n[0] * n[0] + n[1] * n[1]).sqrt() / len;
                    if sin_t <= 1e-9 {
                        // Exactly horizontal.
                        continue;
                    }
                    let zmin = a[2].min(b[2]).min(c[2]);
                    let zmax = a[2].max(b[2]).max(c[2]);
                    // Overlapping gaps: main_z[i] > zmin - R and
                    // main_z[i + 1] < zmax (main_z descending).
                    let start = main_z.partition_point(|&z| z >= zmax).saturating_sub(1);
                    let end = main_z.partition_point(|&z| z > zmin - radius).min(gaps);
                    if start >= end {
                        continue;
                    }
                    for k in 1..=k_max {
                        for gap in start..end {
                            let parent = (main_z[gap] - main_z[gap + 1]) / 2f64.powi(k as i32 - 1);
                            if sin_t < parent / d - 1e-9 {
                                eligible[(k - 1) * gaps + gap] = true;
                            }
                        }
                    }
                }
                // Under a slope limit a sample is kept only where its
                // greatest contact angle tg is at least slope_from_deg (less
                // the keep test's 1e-9 tie). Contact angles lie in 0..=90
                // degrees: the part is a downward-extruded surface, so the
                // point of it nearest a ball centre is never above the centre
                // (`level_contact`'s normals point up). sin is increasing
                // there, so sin(tg) >= sin(slope_from). A depth-k level whose
                // bound sin(tg) < keep_below is at most that keeps no sample
                // anywhere: its contours would be computed for nothing
                // (#398). The 1e-12 covers the rounding of sin. Exact: the
                // levels left out are the ones the keep test empties.
                let slope_floor = limits.map(|(lo, _)| (lo - 1e-9).max(0.).sin() - 1e-12);
                for gap in 0..gaps {
                    let (za, zb) = (main_z[gap], main_z[gap + 1]);
                    for k in 1..=k_max {
                        if !eligible[(k - 1) * gaps + gap] {
                            continue;
                        }
                        let step = (za - zb) / 2f64.powi(k as i32);
                        // A depth-k level keeps a sample only where the
                        // parent spacing along the surface was too wide:
                        // sin(tg) < P / D - 1e-9, P the parent spacing of
                        // this gap (#46).
                        let keep_below = (za - zb) / 2f64.powi(k as i32 - 1) / d - 1e-9;
                        if slope_floor.is_some_and(|floor| keep_below <= floor) {
                            continue;
                        }
                        let mut j = 1;
                        while za - j as f64 * step > zb {
                            schedule.push(LevelSlot {
                                z: za - j as f64 * step,
                                depth: k,
                                ramp_from: None,
                                last_main: false,
                                keep_below,
                            });
                            j += 2;
                        }
                    }
                }
                schedule.sort_by(|a, b| b.z.total_cmp(&a.z));
            }
            let from = (op.stepdown_mm / (2f64.powi(k_max as i32) * d))
                .min(1.)
                .asin()
                .to_degrees();
            (Some(scallop), Some(from))
        } else {
            (None, None)
        };
        let surface = op.surface.prepare(op.floor_z_mm, op.tolerance_mm, cancel)?;
        let flat_target = matches!(cutter.shape, CutterShape::Flat | CutterShape::Bull { .. })
            .then(|| PreparedRoughingTarget::prepare(op.surface.clone(), op.tolerance_mm, cancel))
            .transpose()?;
        let flat_corner = match cutter.shape {
            CutterShape::Bull { corner_radius_mm } => {
                corner_radius_mm
                    + op.stock_to_leave_mm
                    + op.surface.surface_error_mm
                    + op.safety_margin_mm
            }
            _ => 0.,
        };
        if surface.max_z() > spec.max_mm[2] {
            return Err("waterline target exceeds initial stock top".into());
        }
        let report = WaterlineReport {
            stock_top: None,
            contour_method: op.contour_method,
            certificate_tests: 0,
            certificate_nodes: 0,
            strategy: "waterline",
            geometric_residual_status: ResidualStatus::Pending,
            levels,
            levels_completed: 0,
            empty_levels: 0,
            scallop_levels: 0,
            scallop_mm,
            scallop_from_deg,
            moved_entries: 0,
            helical_entries: 0,
            stacks: 0,
            stacks_level_ordered: 0,
            ramp_links: 0,
            links_refused_stock: 0,
            links_refused_link_clearance: 0,
            links_refused_ramp_clearance: 0,
            links_refused_short_run: 0,
            contour_ramp_entries: 0,
            deep_stock_entries: 0,
            cut_segments: 0,
            loops_completed: 0,
            requested_bottom_z_mm: op.bottom_z_mm,
            last_level_z_mm: None,
            contour_error_mm: op.contour_error_mm,
            safety_margin_mm: op.safety_margin_mm,
            surface_error_mm: op.surface.surface_error_mm,
            stock_to_leave_mm: op.stock_to_leave_mm,
            work: SurfaceWork::default(),
            level_reports: vec![],
            contour_proof: match op.contour_method {
                ContourMethod::QuadTree => {
                    "inside-aware solid distance + Lipschitz conservative quadtree; filled-ear continuous homotopy and normal-error certificates; whole-segment admission"
                }
                ContourMethod::Support => {
                    "analytic convex prism slice support + whole-polygon outer/inner error bounds + M2 union engineering budget; filled-ear homotopy and final original-solid admission"
                }
            },
            surface_completion: "not_certified; waterline bands do not clear shallow/unreachable regions; independent remaining-stock checks required",
            reach_proof: None,
            shadow_error_mm: None,
        };
        Ok(Self {
            op,
            spec: spec.clone(),
            surface,
            radius,
            inflated,
            protected,
            initial,
            position: initial,
            next_id: first_id,
            expected_generation: None,
            schedule,
            slot: 0,
            profile: None,
            finished_loops: 0,
            drained: false,
            report,
            stock_ceiling: None,
            move_depth: None,
            prefetched: std::collections::VecDeque::new(),
            limits,
            region_limits,
            flat_target,
            flat_corner,
            ramp_entry,
            emission: None,
        })
    }
    fn contour_options(&self, z: f64) -> ContourOptions {
        ContourOptions {
            method: self.op.contour_method,
            center_z_mm: z + self.radius,
            radius_mm: self.inflated,
            protected_radius_mm: self.protected,
            normal_error_mm: self.op.contour_error_mm,
            tolerance_mm: self.op.tolerance_mm,
            xy_min_mm: [self.spec.min_mm[0], self.spec.min_mm[1]],
            xy_max_mm: [self.spec.max_mm[0], self.spec.max_mm[1]],
            simplify: self.op.simplify,
        }
    }
    /// Contours of the current level. Upcoming levels are computed in an
    /// ordered parallel batch (one per core, twice over) and consumed in order;
    /// each is identical to a serial computation.
    fn next_contours(&mut self, cancel: &impl CancelSignal) -> Result<WaterlineContours, String> {
        if let Some(contours) = self.prefetched.pop_front() {
            return Ok(contours);
        }
        let workers = std::thread::available_parallelism().map_or(1, usize::from);

        // Quad-tree levels: up to one level per core at a time, each level
        // using the remaining cores for its own subtrees; support levels:
        // a batch of two per core, one thread each.
        let quad = self.op.contour_method == ContourMethod::QuadTree;
        let batch = if quad { workers } else { 2 * workers };
        let end = (self.slot + batch).min(self.schedule.len());
        let inner = if quad {
            (workers / (end - self.slot).max(1)).max(1)
        } else {
            1
        };
        let jobs: Vec<_> = (self.slot..end)
            .map(|i| {
                let slot = &self.schedule[i];
                (slot.z, self.contour_options(slot.z))
            })
            .collect();
        let surface = &self.surface;
        let values = crate::roughing::parallel::ordered(
            jobs.len(),
            workers,
            cancel,
            |i, signal| {
                let stop = || signal.is_cancelled();
                let (z, o) = jobs[i];
                let mut contours = prepare_contours_with(surface, o, inner, &stop)?;
                contours.report.tip_z_mm = z;
                Ok(contours)
            },
            |_| Ok(()),
        )?;
        self.prefetched.extend(values);
        self.prefetched
            .pop_front()
            .ok_or_else(|| "waterline level batch was empty".into())
    }
    pub fn report(&self) -> &WaterlineReport {
        &self.report
    }
    /// Planning progress (#198): Z levels done of those planned.
    pub fn progress(&self) -> crate::progress::Planned {
        crate::progress::Planned::Layers {
            done: self.report.levels_completed,
            total: self.report.levels,
        }
    }
    pub fn prefix_events(&self) -> Vec<crate::motion::Event> {
        self.op.settings().prefix_events(self.initial)
    }
    pub fn suffix_events(&self) -> Vec<crate::motion::Event> {
        self.op.settings().suffix_events()
    }
    pub fn finish_report(&mut self, cancel: &impl CancelSignal) -> Result<(), String> {
        cancelled(cancel)?;
        if !self.drained {
            return Err("drain committed waterline motions before final report".into());
        }
        self.report.geometric_residual_status = ResidualStatus::Unknown;
        Ok(())
    }
    pub fn next_motion_with_stock(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        if stock.spec() != &self.spec
            || stock.generation() == u64::MAX
            || self
                .expected_generation
                .is_some_and(|g| g != stock.generation())
        {
            return Err("commit preceding waterline motion to matching stock".into());
        }
        if let Some(ceiling) = &mut self.stock_ceiling {
            ceiling.validate(stock, cancel)?;
        }
        if self.flat_target.is_none() {
            return self.next_ball_motion(stock, cancel);
        }
        loop {
            cancelled(cancel)?;
            if self.next_id == u64::MAX {
                return Err("waterline motion id space exhausted".into());
            }
            if let Some(profile) = &mut self.profile {
                if let Some(motion) = profile.next(stock, cancel)? {
                    if let Some(depth) = &self.move_depth {
                        depth.check(&motion, stock, cancel)?;
                    }
                    self.position = motion.curve.end();
                    self.next_id = motion.id + 1;
                    self.expected_generation = Some(stock.generation() + 1);
                    self.report.cut_segments += usize::from(motion.kind == MoveKind::Cut);
                    self.report.loops_completed = self.finished_loops + profile.closed_loops;
                    return Ok(Some(motion));
                }
                self.finished_loops += profile.closed_loops;
                self.profile = None;
                // A completed main level counts as today; a completed
                // in-between level (it was not empty, so it cut) counts in
                // scallop_levels (#46).
                if self.schedule[self.slot].depth == 0 {
                    self.report.levels_completed += 1;
                } else {
                    self.report.scallop_levels += 1;
                }
                self.slot += 1;
            }
            if self.slot == self.schedule.len() {
                self.drained = true;
                return Ok(None);
            }
            if let Some(target) = &self.flat_target {
                let z = self.schedule[self.slot].z;
                let (mut loops, error) = flat_level_loops(
                    target,
                    z,
                    self.inflated,
                    self.flat_corner,
                    self.op.stock_to_leave_mm + self.op.safety_margin_mm,
                    self.op.tolerance_mm,
                    cancel,
                )?;
                self.report.shadow_error_mm =
                    Some(self.report.shadow_error_mm.unwrap_or(0.).max(error));
                // Region boundaries run counter-clockwise outside and
                // clockwise around holes: climb milling runs against them.
                if !self.op.reverse {
                    for ring in &mut loops {
                        *ring = ring.iter().rev().map(|s| s.reversed()).collect();
                    }
                }
                self.report.last_level_z_mm = Some(z);
                if loops.is_empty() {
                    self.slot += 1;
                    self.report.empty_levels += 1;
                    self.report.levels_completed += 1;
                    continue;
                }
                let (ramp_from, passes) = self.ramp_setup(self.slot);
                let mut profile = ProfileMotion::new(
                    self.op.settings(),
                    &self.spec,
                    self.position,
                    loops,
                    ProfileDepths::Fixed { z },
                    passes,
                    self.next_id,
                );
                profile.ramp_from = ramp_from;
                if let Some(limits) = self.op.limits.as_ref().filter(|l| !l.is_empty()) {
                    profile.limits = Some(crate::limits::PreparedLimits::new(
                        limits,
                        self.radius,
                        self.op.tolerance_mm,
                    )?);
                }
                self.profile = Some(profile);
                continue;
            }
            unreachable!("ball end mills are planned by next_ball_motion");
        }
    }
    /// Ball end mills (#46): every level's contours as the pieces the level
    /// cuts - closed loops, or the runs the in-between levels, slope and
    /// region limits keep - each tied to the closed contour it was cut from,
    /// grouped into stacks (`build_stacks`). Fills the per-level report
    /// fields as the levels are contoured.
    fn plan_pieces(&mut self, cancel: &impl CancelSignal) -> Result<Emission, String> {
        let mut rings: Vec<Vec<[f64; 2]>> = vec![];
        let mut ring_slots: Vec<usize> = vec![];
        let mut pieces: Vec<Piece> = vec![];
        while self.slot < self.schedule.len() {
            cancelled(cancel)?;
            let depth = self.schedule[self.slot].depth;
            let keep_below = self.schedule[self.slot].keep_below;
            let contours = self.next_contours(cancel)?;
            let z = contours.report.tip_z_mm;
            let work = sum(self.report.work, contours.report.work);
            let certificates = self
                .report
                .certificate_tests
                .saturating_add(contours.report.certificate_tests);
            let certificate_nodes = self
                .report
                .certificate_nodes
                .saturating_add(contours.report.certificate_nodes);
            let mut slope_work = SurfaceWork::default();
            let mut loops: Vec<(usize, Vec<Stroke>)> = vec![];
            for mut ring in contours.loops {
                if self.op.reverse {
                    ring.reverse();
                }
                let owner = rings.len();
                rings.push(ring.clone());
                ring_slots.push(self.slot);
                if depth == 0 && self.limits.is_none() && self.region_limits.is_none() {
                    loops.push((
                        owner,
                        ring.iter()
                            .zip(ring.iter().cycle().skip(1))
                            .take(ring.len())
                            .map(|(a, b)| Stroke::line(*a, *b))
                            .collect(),
                    ));
                    continue;
                };
                let mut closed = ring.clone();
                closed.push(ring[0]);
                let spacing = self.radius / 4.;
                let (surface, inflated, slope, region) = (
                    &self.surface,
                    self.inflated,
                    self.limits,
                    &self.region_limits,
                );
                // An in-between level keeps only the parts whose parent
                // spacing along the surface was too wide: the greatest
                // contact angle at the sample must satisfy
                // sin(tg) < keep_below — the parent spacing of the level's
                // gap over the widest allowed spacing D, minus the 1e-9 tie
                // (#46); the tie keeps an exactly vertical wall (sin = 1)
                // from ever getting a level.
                let center_z = z + self.radius;
                let band = (self.inflated - self.protected).max(0.) + self.op.contour_error_mm;
                let tolerance = self.op.tolerance_mm;
                // `scallop`: also the in-between level's own rule; without
                // it, only what the slope and region limits allow.
                let mut keep = |p: [f64; 2], scallop: bool| -> Result<bool, String> {
                    if slope.is_some() || scallop && keep_below.is_finite() {
                        let (least, greatest) = level_contact_angles(
                            surface,
                            [p[0], p[1], center_z],
                            inflated,
                            band,
                            tolerance,
                            &mut slope_work,
                            cancel,
                        )?;
                        if scallop && greatest.sin() >= keep_below {
                            return Ok(false);
                        }
                        if let Some((lo, hi)) = slope
                            && (greatest < lo - 1e-9 || least > hi + 1e-9)
                        {
                            return Ok(false);
                        }
                    }
                    // The cutter disk at this sample (and half a spacing
                    // either way) inside the limits at this level.
                    if let Some(l) = region
                        && l.margin(p, z, true)? < spacing / 2.
                    {
                        return Ok(false);
                    }
                    Ok(true)
                };
                let join = |mut runs: Vec<Vec<[f64; 2]>>| {
                    if runs.len() >= 2
                        && runs[0].first() == closed.first()
                        && runs.last().unwrap().last() == closed.last()
                    {
                        // A run through the ring's start: join its two halves.
                        let first = runs.remove(0);
                        runs.last_mut().unwrap().extend(first.into_iter().skip(1));
                    }
                    runs
                };
                let limited = depth > 0 && (slope.is_some() || region.is_some());
                let runs = if limited {
                    // An in-between level under limits (#46): within each
                    // run the limits allow, the fragments the scallop rule
                    // keeps are cut as one piece across short gaps.
                    let mut runs = vec![];
                    for run in join(crate::finishing::filter_runs(
                        vec![closed.clone()],
                        spacing,
                        |p| keep(p, false),
                    )?) {
                        let kept =
                            crate::finishing::filter_runs(vec![run.clone()], spacing, |p| {
                                keep(p, true)
                            })?;
                        match bridge_open(&run, &kept, BRIDGE_RADII * self.radius) {
                            Some(piece) => runs.push(piece),
                            None => runs.extend(kept),
                        }
                    }
                    runs
                } else {
                    crate::finishing::filter_runs(vec![closed.clone()], spacing, |p| keep(p, true))?
                };
                let whole = runs.len() == 1
                    && runs[0].first() == closed.first()
                    && runs[0].last() == closed.last();
                // An in-between level that keeps the contour in fragments
                // with only short gaps between them cuts it whole (#46):
                // one closed loop the level above links to, not a retract
                // and an entry per fragment. Never where slope or region
                // limits cut the contour: those parts must stay uncut.
                let bridged = !whole
                    && depth > 0
                    && slope.is_none()
                    && region.is_none()
                    && runs_bridge(&ring, &runs, BRIDGE_RADII * self.radius);
                if whole || bridged {
                    // Entirely within the limits: keep the original contour.
                    loops.push((
                        owner,
                        ring.iter()
                            .zip(ring.iter().cycle().skip(1))
                            .take(ring.len())
                            .map(|(a, b)| Stroke::line(*a, *b))
                            .collect(),
                    ));
                    continue;
                }
                let runs = if limited { runs } else { join(runs) };
                loops.extend(runs.into_iter().map(|run| {
                    (
                        owner,
                        run.windows(2)
                            .map(|w| Stroke::line(w[0], w[1]))
                            .collect::<Vec<_>>(),
                    )
                }));
            }
            let work = sum(work, slope_work);
            cancelled(cancel)?;
            self.report.work = work;
            self.report.certificate_tests = certificates;
            self.report.certificate_nodes = certificate_nodes;
            // These keep their meaning for main levels only (#46); an
            // in-between level still adds its contour work and
            // certificates above.
            if depth == 0 {
                self.report.last_level_z_mm = Some(z);
                self.report.level_reports.push(contours.report);
            }
            if loops.is_empty() {
                if depth == 0 {
                    self.report.empty_levels += 1;
                    self.report.levels_completed += 1;
                }
            } else if depth == 0 {
                self.report.levels_completed += 1;
            } else {
                self.report.scallop_levels += 1;
            }
            pieces.extend(loops.into_iter().map(|(ring, strokes)| Piece {
                z,
                ring,
                closed: loop_is_closed(&strokes),
                strokes,
            }));
            self.slot += 1;
        }
        let stacks = build_stacks(&rings, &ring_slots, &pieces);
        Ok(Emission {
            pieces,
            rings,
            stacks,
            current: None,
            last: None,
            level_order: false,
            queue: std::collections::VecDeque::new(),
            finished: false,
        })
    }
    /// Per loop an entry that never plunges into stock (#46): a point beside
    /// the loop where the ball plunges before a horizontal lead onto it - the
    /// stock the roughing left against the wall is then cut sideways, not by
    /// the ball's tip - or, when no point beside the loop qualifies, a helix
    /// along the loop itself. A point is taken only where the lead (at the
    /// level's ball centre height) is clear of the part (higher at the same
    /// XY is clear too: the part is a downward-extruded surface), where the
    /// live stock under the ball (the upper bound of every cell its disk can
    /// touch) is at or below the height it plunges to, within the tolerance,
    /// and where the plunge point and the lead are inside the machining
    /// limits.
    ///
    /// The search runs first at the loop's given start, then - a closed loop
    /// only - at samples along the whole loop, `max(r/2, 1)` mm apart,
    /// outward in both directions, nearest first; the first sample with a
    /// point moves the loop's start there (order and direction stay, only
    /// the start moves), so loops that enter cleanly today are unchanged.
    /// Open loops (runs the slope or region limits cut out of a contour)
    /// keep their start: a moved entry elsewhere on them would need a long
    /// lead no check covers. A loop with no qualifying point beside it
    /// enters on a ramp along itself: the lowest height the stock under the
    /// ball at the loop start is clear at (at most the feed height the rapid
    /// descent stops at), the plunge there in air, then a ramp along the
    /// loop down to the level, then the loop flat - never a vertical plunge
    /// into stock. Every ramp move carries the same part-clearance proof as
    /// the loop's own moves (`path_clear`); where neither entry exists the
    /// operation is refused, naming the level and the start.
    ///
    /// Side-entry points are tried from `max(r/2, 1)` mm out, in steps of
    /// that, up to `SIDE_ENTRY_REACH_RADII` radii, on both sides of the loop
    /// tangent (the side farther from the part first).
    fn entries(
        &self,
        stock: &Stock,
        loops: &mut [Vec<Stroke>],
        z: f64,
        plunge_z: f64,
        cancel: &impl CancelSignal,
    ) -> Result<Entries, String> {
        let step = (self.radius / 2.).max(1.);
        let reach = SIDE_ENTRY_REACH_RADII * self.radius;
        let center = z + self.radius;
        let mut work = SurfaceWork::default();
        let mut points = Vec::with_capacity(loops.len());
        let mut helix = Vec::with_capacity(loops.len());
        let (mut moved, mut entered) = (0, 0);
        for edges in loops.iter_mut() {
            let start = edges[0].curve(z).start();
            let mut found = self.entry_at(
                stock,
                [start[0], start[1]],
                edges[0].tangent(false),
                center,
                plunge_z,
                step,
                reach,
                &mut work,
                cancel,
            )?;
            // A closed loop with no clear entry at its start tries samples
            // along the whole loop, nearest first, and moves its start to
            // the first sample with one (#46).
            if found.is_none() && loop_is_closed(edges) {
                let total: f64 = edges.iter().map(stroke_length).sum();
                let mut tried: Vec<f64> = vec![];
                let mut d = step;
                while found.is_none() && d < total - 1e-9 {
                    for &s in &[d, total - d] {
                        // The candidate start is snapped first, so the lead
                        // the checks below prove is the lead emitted.
                        let s = snap_to_vertex(edges, s);
                        if s <= 1e-9
                            || s >= total - 1e-9
                            || tried.iter().any(|&t| (t - s).abs() <= 1e-9)
                        {
                            continue;
                        }
                        tried.push(s);
                        let (q, t) = loop_point_tangent(edges, s);
                        found = self.entry_at(
                            stock, q, t, center, plunge_z, step, reach, &mut work, cancel,
                        )?;
                        if found.is_some() {
                            rotate_loop(edges, s);
                            break;
                        }
                    }
                    d += step;
                }
                moved += usize::from(found.is_some());
            }
            if let Some(p) = found {
                points.push(Some(p));
                helix.push(None);
                continue;
            }
            // No clear side entry anywhere on the loop: enter on a ramp
            // along the loop itself (#46). The lowest height the stock under
            // the ball at the loop start is clear at, at most the feed
            // height; the plunge there is in air and the ramp descends from
            // it to the level.
            let top = stock.top_under_disk([start[0], start[1]], self.radius);
            let feed = self.op.settings().heights.feed;
            if top > feed + self.op.tolerance_mm {
                return Err(format!(
                    "waterline operation {}: no entry for the loop at level Z{z} starting at ({:.4}, {:.4}): the stock under the ball there stands at Z{top:.4}, above the feed height Z{feed:.4} the entry plunge comes down from, and no point beside the loop within {reach:.3} mm has the stock at or below the plunge height; rough or rest-rough there first",
                    self.op.id, start[0], start[1]
                ));
            }
            let zc = top.max(z).min(feed);
            entered += 1;
            points.push(None);
            helix.push(Some(zc));
        }
        Ok((points, helix, moved, entered))
    }
    /// The side-entry search at one point of a loop: beside it, from `step`
    /// mm out in steps of `step` up to `reach`, on both sides of the tangent
    /// `t` (the side farther from the part first), the point farthest from
    /// the part at the nearest working distance where the lead to it (at the
    /// level's ball centre height) is clear of the part, the live stock
    /// under the ball is at or below the plunge height within the tolerance
    /// and the limits allow the plunge and the lead.
    #[allow(clippy::too_many_arguments)]
    fn entry_at(
        &self,
        stock: &Stock,
        start: [f64; 2],
        t: [f64; 2],
        center: f64,
        plunge_z: f64,
        step: f64,
        reach: f64,
        work: &mut SurfaceWork,
        cancel: &impl CancelSignal,
    ) -> Result<Option<[f64; 2]>, String> {
        let mut k = 1.;
        while k * step <= reach + 1e-9 {
            let length = k * step;
            let mut best: Option<([f64; 2], f64)> = None;
            for sign in [1., -1.] {
                let p = [
                    start[0] - sign * length * t[1],
                    start[1] + sign * length * t[0],
                ];
                if !self.surface.segment_clear(
                    [p[0], p[1], center],
                    [start[0], start[1], center],
                    self.protected,
                    work,
                    cancel,
                )? {
                    continue;
                }
                let top = stock.top_under_disk(p, self.radius);
                if top > plunge_z + self.op.tolerance_mm {
                    continue;
                }
                // The lead onto the loop is pure air: it meets no stock above
                // the level (#46), so the tool never comes in from the side
                // into a full cut.
                if self.stock_met(
                    stock,
                    [p[0], p[1], plunge_z],
                    start,
                    None,
                    self.op.tolerance_mm,
                    cancel,
                )? > self.op.tolerance_mm
                {
                    continue;
                }
                if let Some(limits) = &self.region_limits {
                    let lead = crate::motion::Curve::Line {
                        start: [p[0], p[1], plunge_z],
                        end: [start[0], start[1], plunge_z],
                    };
                    if !limits.allows(p, plunge_z)? || !limits.allows_curve(&lead, true)? {
                        continue;
                    }
                }
                let d = self
                    .surface
                    .point_distance([p[0], p[1], center], work, cancel)?
                    .distance_mm;
                if best.is_none_or(|(_, b)| d > b) {
                    best = Some((p, d));
                }
            }
            if let Some((p, _)) = best {
                return Ok(Some(p));
            }
            k += 1.;
        }
        Ok(None)
    }
    /// Ball end mills (#46): the next motion. On the first call every level is
    /// contoured and its pieces grouped into stacks (`plan_pieces`); then one
    /// job per piece is built against the live stock right before it is cut
    /// (`job`) and its moves handed out one by one.
    fn next_ball_motion(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        if self.emission.is_none() {
            let emission = self.plan_pieces(cancel)?;
            self.report.stacks = emission.stacks.len();
            self.emission = Some(emission);
        }
        loop {
            cancelled(cancel)?;
            if self.next_id == u64::MAX {
                return Err("waterline motion id space exhausted".into());
            }
            let emission = self.emission.as_mut().unwrap();
            if let Some((curve, kind, feed)) = emission.queue.pop_front() {
                if let Some(motion) = self.emit(curve, kind, feed, stock, cancel)? {
                    return Ok(Some(motion));
                }
                continue;
            }
            if emission.finished {
                self.drained = true;
                return Ok(None);
            }
            match self.next_job(stock, cancel)? {
                Some(moves) => self.emission.as_mut().unwrap().queue.extend(moves),
                None => {
                    let cut = self.emission.as_ref().unwrap().last.is_some();
                    let moves = if cut { self.leave(true) } else { vec![] };
                    let emission = self.emission.as_mut().unwrap();
                    emission.queue.extend(moves);
                    emission.finished = true;
                }
            }
        }
    }
    /// One queued move as a motion (none for a zero-length line), checked for
    /// the neck, shank and holder reach like every waterline feed move.
    fn emit(
        &mut self,
        curve: Curve,
        kind: MoveKind,
        feed: Option<f64>,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        if matches!(curve, Curve::Line { start, end } if start == end) {
            return Ok(None);
        }
        curve.prepare(self.op.tolerance_mm)?;
        let motion = Motion {
            id: self.next_id,
            operation: self.op.id.clone(),
            tool: self.op.tool.clone(),
            kind,
            curve,
            feed_mm_min: feed,
        };
        if let Some(depth) = &self.move_depth {
            depth.check(&motion, stock, cancel)?;
        }
        self.position = motion.curve.end();
        self.next_id = motion.id + 1;
        self.expected_generation = Some(stock.generation() + 1);
        self.report.cut_segments += usize::from(motion.kind == MoveKind::Cut);
        Ok(Some(motion))
    }
    /// The next piece's moves (#46), or None when every piece is cut. A
    /// stack is cut top to bottom; the next stack is the nearest one whose
    /// parents are done. If a job's moves would fail the reach check against
    /// the live stock (the neck, shank or holder would meet stock a
    /// neighbour still carries), its stack is deferred - the job is not cut -
    /// and the deferred stacks are finished last, level by level: the
    /// highest next piece whose job passes the check (the nearest of equal
    /// height first); when none passes, the highest is cut and its failing
    /// move refused as before.
    fn next_job(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Vec<Queued>>, String> {
        loop {
            let reordered = self.nearest_at_level();
            let candidates = self.pick();
            if candidates.is_empty() {
                return Ok(None);
            }
            let level_order = candidates[0].2;
            let mut chosen = None;
            for &(s, k, _) in &candidates {
                let job = self.job(stock, s, k, cancel)?;
                if self.reach_refused(&job.0, stock, cancel) {
                    if !level_order {
                        let emission = self.emission.as_mut().unwrap();
                        // The refused piece goes back to its place: the
                        // level-order fallback then sees the stack's pieces
                        // top-down, the higher ones (which may clear what
                        // the refused one would meet) first.
                        if let Some((rs, next, best)) = reordered
                            && rs == s
                        {
                            emission.stacks[s].pieces[next..=next + best].rotate_left(1);
                        }
                        emission.stacks[s].deferred = true;
                        emission.current = None;
                        self.report.stacks_level_ordered += 1;
                        break;
                    }
                    continue;
                }
                chosen = Some((s, k, job));
                break;
            }
            if chosen.is_none() && level_order {
                let (s, k, _) = candidates[0];
                chosen = Some((s, k, self.job(stock, s, k, cancel)?));
            }
            let Some((s, k, (moves, counts))) = chosen else {
                continue;
            };
            self.report.ramp_links += usize::from(counts.0);
            self.report.moved_entries += counts.1;
            self.report.helical_entries += counts.2;
            self.report.links_refused_stock += counts.3[0];
            self.report.links_refused_link_clearance += counts.3[1];
            self.report.links_refused_ramp_clearance += counts.3[2];
            self.report.links_refused_short_run += counts.3[3];
            self.report.deep_stock_entries += usize::from(counts.4);
            self.report.contour_ramp_entries += counts.5;
            self.report.loops_completed += 1;
            let emission = self.emission.as_mut().unwrap();
            emission.stacks[s].next += 1;
            emission.current = Some(s);
            emission.last = Some(k);
            emission.level_order = level_order;
            return Ok(Some(moves));
        }
    }
    /// In the stack being cut, the next piece is the nearest one that is
    /// ready (#46): a piece may go before the pieces ahead of it in the
    /// stack's top-down order when none of those it skips lies within two
    /// cutter radii of it (bounding boxes) and none of them is a closed loop.
    /// The runs slope or region limits cut a contour into - separate walls
    /// of one ring - are then cut wall by wall, top to bottom, each wall's
    /// levels linked, instead of switching walls at every level. Order only:
    /// every move keeps its own proofs on the live stock. Returns the
    /// rotation made (stack, first index, how far the piece moved), so a
    /// refused piece can go back to its place.
    fn nearest_at_level(&mut self) -> Option<(usize, usize, usize)> {
        let position = [self.position[0], self.position[1]];
        let margin = 2. * self.radius;
        let e = self.emission.as_mut()?;
        let s = e.current.filter(|_| !e.level_order)?;
        let stack = &e.stacks[s];
        if stack.done() || stack.deferred {
            return None;
        }
        let bbox = |k: usize| {
            let mut b = [
                f64::INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::NEG_INFINITY,
            ];
            for st in &e.pieces[k].strokes {
                for p in [st.a, st.b] {
                    b = [
                        b[0].min(p[0]),
                        b[1].min(p[1]),
                        b[2].max(p[0]),
                        b[3].max(p[1]),
                    ];
                }
            }
            b
        };
        let apart = |a: [f64; 4], b: [f64; 4]| {
            a[0] > b[2] + margin
                || b[0] > a[2] + margin
                || a[1] > b[3] + margin
                || b[1] > a[3] + margin
        };
        let distance = |k: usize| {
            let piece = &e.pieces[k];
            let q = if piece.closed {
                nearest_on_loop(&piece.strokes, position).1
            } else {
                piece.strokes[0].a
            };
            (q[0] - position[0]).hypot(q[1] - position[1])
        };
        let rest = &stack.pieces[stack.next..];
        let boxes: Vec<[f64; 4]> = rest.iter().map(|&k| bbox(k)).collect();
        let mut best = (distance(rest[0]), 0);
        for j in 1..rest.len() {
            if e.pieces[rest[j - 1]].closed || e.pieces[rest[j]].closed {
                break;
            }
            if !(0..j).all(|i| apart(boxes[i], boxes[j])) {
                continue;
            }
            let d = distance(rest[j]);
            if d < best.0 {
                best = (d, j);
            }
        }
        let next = stack.next;
        e.stacks[s].pieces[next..=next + best.1].rotate_right(1);
        (best.1 > 0).then_some((s, next, best.1))
    }
    /// Whether any feed move of a job would fail the reach check against the
    /// live stock (always false when the check does not run).
    fn reach_refused(&self, moves: &[Queued], stock: &Stock, cancel: &impl CancelSignal) -> bool {
        let Some(depth) = &self.move_depth else {
            return false;
        };
        moves.iter().any(|(curve, kind, feed)| {
            feed.is_some()
                && depth
                    .check(
                        &Motion {
                            id: 0,
                            operation: self.op.id.clone(),
                            tool: self.op.tool.clone(),
                            kind: *kind,
                            curve: curve.clone(),
                            feed_mm_min: *feed,
                        },
                        stock,
                        cancel,
                    )
                    .is_err()
        })
    }
    /// The stacks and pieces to try next, and whether they are cut in level
    /// order (deferred stacks). The current stack continues while it has
    /// pieces; otherwise the nearest ready stack (its parents done); then the
    /// deferred stacks, highest next piece first (the nearest of equal
    /// height first).
    fn pick(&self) -> Vec<(usize, usize, bool)> {
        let Some(e) = self.emission.as_ref() else {
            return vec![];
        };
        let ready = |s: usize| {
            !e.stacks[s].done() && e.stacks[s].parents.iter().all(|&p| e.stacks[p].done())
        };
        let next = |s: usize| e.stacks[s].pieces[e.stacks[s].next];
        if let Some(s) = e.current
            && !e.level_order
            && !e.stacks[s].deferred
            && ready(s)
        {
            return vec![(s, next(s), false)];
        }
        let distance = |s: usize| {
            let a = e.pieces[next(s)].strokes[0].a;
            (a[0] - self.position[0]).hypot(a[1] - self.position[1])
        };
        if let Some(s) = (0..e.stacks.len())
            .filter(|&s| ready(s) && !e.stacks[s].deferred)
            .min_by(|&a, &b| distance(a).total_cmp(&distance(b)))
        {
            return vec![(s, next(s), false)];
        }
        let mut order: Vec<usize> = (0..e.stacks.len()).filter(|&s| ready(s)).collect();
        if order.is_empty() {
            order = (0..e.stacks.len())
                .filter(|&s| !e.stacks[s].done())
                .collect();
        }
        order.sort_by(|&a, &b| {
            e.pieces[next(b)]
                .z
                .total_cmp(&e.pieces[next(a)].z)
                .then(distance(a).total_cmp(&distance(b)))
        });
        order.into_iter().map(|s| (s, next(s), true)).collect()
    }
    /// The moves that cut piece `k` of stack `s` (#46) from where the tool
    /// is, and (ramp link, moved entries, helical entries).
    ///
    /// Ramp link: when the tool has just finished the piece before this one
    /// in the same stack and this piece is a closed loop below it, a short
    /// link at the upper level to the nearest point P of this loop, a ramp
    /// down along the loop from P (`loop_ramp`), then the whole loop
    /// flat from where the ramp ends - the flat lap passes over the ramp
    /// again. An open run (at the same level or below) is linked to its
    /// start, descended along itself and back (`out_and_back`), then cut;
    /// from an open run a link the part blocks goes back along that run
    /// first. Taken only when the link, the ramp and the piece are proven
    /// clear of the part and the link meets no more stock (measured on the
    /// stock grid, `stock_met`) than the level gap the ramp descends or the
    /// piece's own cut where the link arrives.
    ///
    /// Otherwise an entry, after a retract: a side entry (plunge beside the
    /// loop, a horizontal lead onto it) only where the lead is pure air;
    /// else the plunge at the loop start stops in air at the height the
    /// stock under the ball there is clear at, and a ramp descends along the
    /// loop to the level (an open run: along it and back, ending at its
    /// start), then the piece flat. Never a vertical plunge into stock.
    fn job(
        &self,
        stock: &Stock,
        s: usize,
        k: usize,
        cancel: &impl CancelSignal,
    ) -> Result<Job, String> {
        let e = self.emission.as_ref().unwrap();
        let piece = &e.pieces[k];
        let z = piece.z;
        let mut work = SurfaceWork::default();
        let cut_feed = Some(self.op.feed_mm_min);
        let ramp_feed = Some(self.ramp_entry.feed_mm_min.unwrap_or(self.op.plunge_mm_min));
        let stack = &e.stacks[s];
        let mut refused = [0; 4];
        let above = (stack.next > 0)
            .then(|| stack.pieces[stack.next - 1])
            .filter(|&q| e.last == Some(q) && e.current == Some(s))
            .filter(|&q| z < e.pieces[q].z - 1e-9 || !piece.closed && z <= e.pieces[q].z + 1e-9);
        // A link down onto an open run descends along the run and back, a
        // reversal; on a run shorter than the declared `min_length_mm`, or
        // too short for the angle within `MAX_RAMP_LEGS` passes, no such
        // ramp exists (#392). That is a link the run cannot take, like one a
        // proof refuses: the run is entered after a retract instead.
        let above = match above {
            Some(q)
                if !piece.closed
                    && z < e.pieces[q].z - 1e-9
                    && !self.run_ramp_fits(&piece.strokes, e.pieces[q].z - z) =>
            {
                refused = [0, 0, 0, 1];
                None
            }
            above => above,
        };
        if let Some(q) = above {
            let za = e.pieces[q].z;
            let mut strokes = piece.strokes.clone();
            let total: f64 = strokes.iter().map(stroke_length).sum();
            // A closed loop is entered at its nearest point P and ramped
            // along it (`loop_ramp`); an open run at its start, along itself
            // and back (`run_ramp`; no ramp at the same level).
            let (ramp, flat_from) = if piece.closed {
                let (sp, _) = nearest_on_loop(&strokes, [self.position[0], self.position[1]]);
                let sp = snap_to_vertex(&strokes, sp);
                if sp > 1e-9 && sp < total - 1e-9 {
                    rotate_loop(&mut strokes, sp);
                }
                self.loop_ramp(&strokes, za, z)?
            } else if za > z + 1e-9 {
                (self.run_ramp(&strokes, za, z)?, 0.)
            } else {
                (vec![], 0.)
            };
            let mut flat: Vec<Curve> = if piece.closed {
                flat_lap(&strokes, flat_from, z)
            } else {
                strokes.iter().map(|e| e.curve(z)).collect()
            };
            let mut ramp = ramp;
            // P is the (rotated) piece's own first point, so the ramp - or,
            // without one, the flat pass - starts exactly where the link ends.
            let p = strokes[0].a;
            // A link too short to post (the two pieces meet within the
            // snapping distance) is left out: the ramp or the flat pass
            // starts exactly where the tool stands. That changes the first
            // ramp or cut move, which is then proven again below.
            let snapped_flat = ramp.is_empty();
            let link: Vec<Curve> =
                if (p[0] - self.position[0]).hypot(p[1] - self.position[1]) > SNAP_MM {
                    vec![Curve::Line {
                        start: self.position,
                        end: [p[0], p[1], za],
                    }]
                } else {
                    if let Some(Curve::Line { start, .. }) = ramp.first_mut().or(flat.first_mut()) {
                        *start = self.position;
                    }
                    vec![]
                };
            let snapped_flat = snapped_flat && link.is_empty();
            let mut link = link;
            // Every link segment is proven against the part and the
            // machining limits (a straight link can cross an avoid area
            // between two runs the limits keep).
            let mut link_clear = self.route_clear(&link, &mut work, cancel)?;
            // From an open run, a straight link can cut across the part
            // (runs of one wall all run the same way, so the next one starts
            // at the far end): back along the run just cut, at its level, to
            // its point nearest P, then straight to P (#46).
            if !link_clear && !e.pieces[q].closed && !link.is_empty() {
                let back: Vec<Stroke> = e.pieces[q]
                    .strokes
                    .iter()
                    .rev()
                    .map(|e| e.reversed())
                    .collect();
                let (sq, _) = nearest_on_loop(&back, [p[0], p[1]]);
                let sq = snap_to_vertex(&back, sq);
                let mut path = if sq > 1e-9 {
                    ramp_curves(&back, za, za, sq)
                } else {
                    vec![]
                };
                if let Some(Curve::Line { start, .. }) = path.first_mut() {
                    *start = self.position;
                }
                let q_end = path.last().map_or(self.position, |c| c.end());
                if (p[0] - q_end[0]).hypot(p[1] - q_end[1]) > SNAP_MM {
                    path.push(Curve::Line {
                        start: q_end,
                        end: [p[0], p[1], za],
                    });
                } else if let Some(Curve::Line { end, .. }) = path.last_mut() {
                    *end = [p[0], p[1], za];
                }
                if !path.is_empty() && self.route_clear(&path, &mut work, cancel)? {
                    link = path;
                    link_clear = true;
                }
            }
            let ramp_clear = link_clear
                && self.route_clear(&ramp, &mut work, cancel)?
                && (!snapped_flat || self.route_clear(&flat[..1], &mut work, cancel)?);
            // The link may meet no more stock than the level gap it ramps
            // down, or than the lower level's own cut meets where the link
            // arrives (the stock the finishing lap removes there anyway):
            // never a deeper cut from the side than the waterline takes.
            let (met, bound) = if ramp_clear {
                let bound = (za - z).max(self.lap_depth(stock, p, z, cancel)?);
                let mut met: f64 = 0.;
                for c in &link {
                    let (a, b) = (c.start(), c.end());
                    met = met.max(self.stock_met(
                        stock,
                        a,
                        [b[0], b[1]],
                        Some([p[0], p[1], z]),
                        bound + self.op.tolerance_mm,
                        cancel,
                    )?);
                }
                (met, bound)
            } else {
                (0., 0.)
            };
            refused = if !link_clear {
                [0, 1, 0, 0]
            } else if !ramp_clear {
                [0, 0, 1, 0]
            } else {
                [1, 0, 0, 0]
            };
            if link_clear && ramp_clear && met <= bound + self.op.tolerance_mm {
                let end = ramp.last().map_or(p, |c| {
                    let q = c.end();
                    [q[0], q[1]]
                });
                // Beyond the level gap the link and ramp descend (the
                // level above cut down to it): roughing in steps up to three
                // stepdowns, plus its allowance, is expected; more is
                // unroughed stock.
                let deep = met.max(self.stock_under_ball(stock, end, z, &mut work, cancel)?)
                    > za - z + DEEP_STEPDOWNS * self.op.stepdown_mm + self.op.tolerance_mm;
                let mut moves: Vec<Queued> = link
                    .into_iter()
                    .map(|c| (c, MoveKind::Lead, cut_feed))
                    .collect();
                moves.extend(ramp.into_iter().map(|c| (c, MoveKind::Plunge, ramp_feed)));
                moves.extend(flat.into_iter().map(|c| (c, MoveKind::Cut, cut_feed)));
                return Ok((moves, (true, 0, 0, [0; 4], deep, 0)));
            }
        }
        let heights = self.op.settings().heights;
        let mut moves = self.leave(false);
        if e.last.is_none() {
            moves = vec![(
                Curve::Line {
                    start: self.position,
                    end: [self.position[0], self.position[1], heights.clearance],
                },
                MoveKind::Rapid,
                None,
            )];
        }
        let travel = moves.last().map_or(self.position, |m| m.0.end());
        let mut strokes = piece.strokes.clone();
        let (points, helix, moved, helical) =
            self.entries(stock, std::slice::from_mut(&mut strokes), z, z, cancel)?;
        let start = strokes[0].curve(z).start();
        let entry = points[0].unwrap_or([start[0], start[1]]);
        let entry_z = helix[0].unwrap_or(z);
        let approach = [
            (
                Curve::Line {
                    start: travel,
                    end: [entry[0], entry[1], travel[2]],
                },
                MoveKind::Rapid,
                None,
            ),
            (
                Curve::Line {
                    start: [entry[0], entry[1], travel[2]],
                    end: [entry[0], entry[1], heights.feed],
                },
                MoveKind::Rapid,
                None,
            ),
        ];
        moves.extend(approach);
        let plunge = Curve::Line {
            start: [entry[0], entry[1], heights.feed],
            end: [entry[0], entry[1], entry_z],
        };
        let mut descent: Vec<Curve> = vec![];
        let mut flat_from = 0.;
        let mut contour_ramps = 0;
        if points[0].is_some() {
            descent.push(Curve::Line {
                start: [entry[0], entry[1], z],
                end: start,
            });
        } else if entry_z > z && piece.closed {
            (descent, flat_from) = self.loop_ramp(&strokes, entry_z, z)?;
        } else if entry_z > z && self.run_ramp_fits(&strokes, entry_z - z) {
            descent = self.run_ramp(&strokes, entry_z, z)?;
        } else if entry_z > z {
            descent = self.contour_ramp(&e.rings[piece.ring], &strokes, entry_z, z)?;
            contour_ramps = 1;
        }
        // The final proof of the moves emitted: the plunge, and the side
        // lead or the ramp, against the part and the limits.
        if !self.path_clear(std::slice::from_ref(&plunge), &mut work, cancel)?
            || !self.route_clear(&descent, &mut work, cancel)?
        {
            return Err(format!(
                "waterline operation {}: the entry to the loop at level Z{z} starting at ({:.4}, {:.4}) is not clear of the part or the machining limits; rough or rest-rough there first, or change the limits",
                self.op.id, start[0], start[1]
            ));
        }
        // Where the entry reaches the level: the side lead's end or the
        // ramp's.
        let end = descent.last().map_or(start, |c| c.end());
        let deep = self.stock_under_ball(stock, [end[0], end[1]], z, &mut work, cancel)?
            > DEEP_STEPDOWNS * self.op.stepdown_mm + self.op.tolerance_mm;
        moves.push((plunge, MoveKind::Plunge, Some(self.op.plunge_mm_min)));
        if points[0].is_some() {
            moves.extend(descent.into_iter().map(|c| (c, MoveKind::Lead, cut_feed)));
        } else {
            moves.extend(
                descent
                    .into_iter()
                    .map(|c| (c, MoveKind::Plunge, ramp_feed)),
            );
        }
        let flat = if piece.closed {
            flat_lap(&strokes, flat_from, z)
        } else {
            strokes.iter().map(|e| e.curve(z)).collect()
        };
        moves.extend(flat.into_iter().map(|c| (c, MoveKind::Cut, cut_feed)));
        Ok((moves, (false, moved, helical, refused, deep, contour_ramps)))
    }
    /// The moves off the piece just cut: up at the feed to the feed height,
    /// then a rapid to the retract height (`last`: the clearance height).
    fn leave(&self, last: bool) -> Vec<Queued> {
        let h = self.op.settings().heights;
        let p = self.position;
        let top = if last { h.clearance } else { h.retract };
        vec![
            (
                Curve::Line {
                    start: p,
                    end: [p[0], p[1], h.feed.max(p[2])],
                },
                MoveKind::Retract,
                Some(self.op.feed_mm_min),
            ),
            (
                Curve::Line {
                    start: [p[0], p[1], h.feed.max(p[2])],
                    end: [p[0], p[1], top.max(h.feed).max(p[2])],
                },
                MoveKind::Rapid,
                None,
            ),
        ]
    }
    /// A ramp descending `from` -> `to` along a closed loop from its start
    /// (#46), never steeper than the ramp angle: within one lap when that is
    /// long enough (split where `ramp_split` allows), else whole laps each
    /// descending an equal share, ending at the start. Returns the ramp and
    /// where along the loop it ends (the flat lap starts there and passes
    /// over the whole ramp). Refused by name past `MAX_RAMP_LEGS` laps.
    fn loop_ramp(&self, edges: &[Stroke], from: f64, to: f64) -> Result<(Vec<Curve>, f64), String> {
        let total: f64 = edges.iter().map(stroke_length).sum();
        let length = (from - to) / self.ramp_entry.angle_rad.tan();
        if length <= total {
            let s = ramp_split(edges, length);
            return Ok((ramp_curves(edges, from, to, s), s));
        }
        let laps = self.ramp_passes(length, total, from - to)?;
        let step = (from - to) / laps as f64;
        let mut curves = vec![];
        for i in 0..laps {
            curves.extend(ramp_curves(
                edges,
                from - i as f64 * step,
                from - (i + 1) as f64 * step,
                total,
            ));
        }
        Ok((curves, 0.))
    }
    /// A ramp descending `from` -> `to` onto an open run, ending at its start
    /// (#46), never steeper than the ramp angle: out along the run and back
    /// over the same stretch, as many times as the angle needs.
    fn run_ramp(&self, edges: &[Stroke], from: f64, to: f64) -> Result<Vec<Curve>, String> {
        let total: f64 = edges.iter().map(stroke_length).sum();
        // Each leg descends half of one out-and-back's share.
        let minimum = self.ramp_entry.min_length_mm.unwrap_or(0.);
        // The callers never ask for this ramp on such a run (#392): a link
        // enters it after a retract instead, and the entry ramps along the
        // run's contour (`contour_ramp`). Kept as the guard of the ramp
        // itself.
        if self.too_short_to_reverse(edges) {
            return Err(format!(
                "waterline operation {}: open ramp span {total:.4} mm is shorter than min_length_mm {minimum:.4} mm for a reversal",
                self.op.id
            ));
        }
        let leg = ((from - to) / 2. / self.ramp_entry.angle_rad.tan()).max(minimum);
        if leg <= total {
            return Ok(out_and_back(edges, from, to, ramp_split(edges, leg)));
        }
        let passes = self.ramp_passes(leg, total, from - to)?;
        let step = (from - to) / passes as f64;
        let mut curves = vec![];
        for i in 0..passes {
            curves.extend(out_and_back(
                edges,
                from - i as f64 * step,
                from - (i + 1) as f64 * step,
                total,
            ));
        }
        Ok(curves)
    }
    /// The entry ramp of an open run too short for a ramp along itself
    /// (`run_ramp_fits`), after a retract (#392): out along the level's
    /// whole contour `ring` from the run's start, in the cut direction
    /// (through the run and on along the contour), and back to the start,
    /// exactly as `run_ramp` does on a run, then the run is cut. This is the
    /// ramp along "the path that comes after the plunge" of FreeCAD's ramp
    /// dressup (see `RampEntry`): the run is one stretch of that contour,
    /// and the slope or region limits that cut it out choose what is
    /// finished, not where the tool may travel above the level. Every point
    /// of the ramp lies over the contour at or above its level, and the
    /// caller proves it clear of the part and inside the machining limits
    /// (`route_clear`) like every other entry. Refused by name where the
    /// contour, too, is shorter than `min_length_mm` or than the angle
    /// allows within `MAX_RAMP_LEGS` passes.
    fn contour_ramp(
        &self,
        ring: &[[f64; 2]],
        run: &[Stroke],
        from: f64,
        to: f64,
    ) -> Result<Vec<Curve>, String> {
        let start = run[0].a;
        let mut edges: Vec<Stroke> = ring
            .iter()
            .zip(ring.iter().cycle().skip(1))
            .take(ring.len())
            .map(|(a, b)| Stroke::line(*a, *b))
            .collect();
        let total: f64 = edges.iter().map(stroke_length).sum();
        let run_length: f64 = run.iter().map(stroke_length).sum();
        let (s, q) = nearest_on_loop(&edges, start);
        // The run's points are samples of this contour (`filter_runs`).
        if (q[0] - start[0]).hypot(q[1] - start[1]) > 1e-6 {
            return Err(format!(
                "waterline operation {}: the open run at level Z{to} starting at ({:.4}, {:.4}) does not lie on its contour; this is a planner defect, please report it",
                self.op.id, start[0], start[1]
            ));
        }
        if s > 1e-9 && s < total - 1e-9 {
            rotate_loop(&mut edges, s);
        }
        // The ramp starts and ends exactly where the run starts.
        edges[0].a = start;
        if !self.run_ramp_fits(&edges, from - to) {
            return Err(format!(
                "waterline operation {}: no entry for the open run at level Z{to} starting at ({:.4}, {:.4}): the run ({run_length:.4} mm) and its whole contour ({total:.4} mm) are too short for a ramp along them and back descending {:.4} mm at {:.3} degrees with min_length_mm {:?} (the shortest span the tool may reverse on); no point beside the run has the stock at or below the level, and it is not plunged into; rough or rest-rough there down to the level, or declare the steeper angle or smaller min_length_mm the tool allows",
                self.op.id,
                start[0],
                start[1],
                from - to,
                self.ramp_entry.angle_rad.to_degrees(),
                self.ramp_entry.min_length_mm,
            ));
        }
        self.run_ramp(&edges, from, to)
    }
    /// Whether an open run is shorter than the declared `min_length_mm`, the
    /// shortest span the tool may reverse a ramp on (#392): a ramp along it
    /// and back (`run_ramp`) does not exist there.
    fn too_short_to_reverse(&self, edges: &[Stroke]) -> bool {
        self.ramp_entry
            .min_length_mm
            .is_some_and(|minimum| minimum > edges.iter().map(stroke_length).sum::<f64>())
    }
    /// Whether `run_ramp` exists on `edges` for a descent of `drop` (#392):
    /// the run is at least `min_length_mm` long and the angle needs no more
    /// than `MAX_RAMP_LEGS` passes along it - exactly the cases `run_ramp`
    /// does not refuse.
    fn run_ramp_fits(&self, edges: &[Stroke], drop: f64) -> bool {
        if self.too_short_to_reverse(edges) {
            return false;
        }
        let total: f64 = edges.iter().map(stroke_length).sum();
        let minimum = self.ramp_entry.min_length_mm.unwrap_or(0.);
        let leg = (drop / 2. / self.ramp_entry.angle_rad.tan()).max(minimum);
        leg <= total || self.ramp_passes(leg, total, drop).is_ok()
    }
    /// How many passes of `total` a ramp `length` long needs, refused by
    /// name past `MAX_RAMP_LEGS` (#46).
    fn ramp_passes(&self, length: f64, total: f64, drop: f64) -> Result<usize, String> {
        let passes = (length / total).ceil();
        if !passes.is_finite() || passes > crate::entry::MAX_RAMP_LEGS || total <= 1e-9 {
            return Err(format!(
                "waterline operation {}: descending {drop:.4} mm at the ramp angle of {:.3} degrees along a contour {total:.4} mm long needs more than {} passes; declare a steeper RampEntry the tool allows",
                self.op.id,
                self.ramp_entry.angle_rad.to_degrees(),
                crate::entry::MAX_RAMP_LEGS
            ));
        }
        Ok(passes as usize)
    }
    /// The part-clearance proof of entry, link and ramp moves (#46): the
    /// ball centre along each line keeps the distance from the part that the
    /// loop's own moves keep (`protected`).
    fn path_clear(
        &self,
        curves: &[Curve],
        work: &mut SurfaceWork,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        for c in curves {
            let (a, b) = (c.start(), c.end());
            if !self.surface.segment_clear(
                [a[0], a[1], a[2] + self.radius],
                [b[0], b[1], b[2] + self.radius],
                self.protected,
                work,
                cancel,
            )? {
                return Ok(false);
            }
        }
        Ok(true)
    }
    /// Whether every one of `curves` is proven inside the machining limits
    /// (#46): cutting moves stay out of the avoid areas and inside the
    /// containment, like the loops the limits keep (always true without
    /// limits). Leads are not checked by the run's `LinkGuard`, so a link or
    /// ramp the emitter builds is checked here.
    fn allowed(&self, curves: &[Curve]) -> Result<bool, String> {
        let Some(limits) = &self.region_limits else {
            return Ok(true);
        };
        for c in curves {
            if !limits.allows_curve(c, true)? {
                return Ok(false);
            }
        }
        Ok(true)
    }
    /// Part clearance (`path_clear`) and the machining limits (`allowed`).
    fn route_clear(
        &self,
        curves: &[Curve],
        work: &mut SurfaceWork,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        Ok(self.path_clear(curves, work, cancel)? && self.allowed(curves)?)
    }
    /// How deep the ball standing with its tip at `z` over `p` meets stock that
    /// is not the part (#46): what a cut at that point removes, on the stock
    /// grid. This centre-sampled estimate is used only for the rough-first warning.
    fn stock_under_ball(
        &self,
        stock: &Stock,
        p: [f64; 2],
        z: f64,
        work: &mut SurfaceWork,
        cancel: &impl CancelSignal,
    ) -> Result<f64, String> {
        let r = self.radius;
        let spec = stock.spec();
        let [nx, ny] = stock.dimensions();
        let [wx, wy] = stock.cell_widths_mm();
        let half = wx.hypot(wy) / 2.;
        let cells = stock.cells();
        let index = |v: f64, k: usize, w: f64, n: usize| {
            (((v - spec.min_mm[k]) / w).floor().max(0.) as usize).min(n)
        };
        let mut worst: f64 = 0.;
        for j in index(p[1] - r, 1, wy, ny)..index(p[1] + r, 1, wy, ny).saturating_add(1).min(ny) {
            for i in
                index(p[0] - r, 0, wx, nx)..index(p[0] + r, 0, wx, nx).saturating_add(1).min(nx)
            {
                let cell = j * nx + i;
                let c = stock.cell_center(cell).unwrap();
                let d = (c[0] - p[0]).hypot(c[1] - p[1]);
                if d >= r {
                    continue;
                }
                let excess = cells[cell].upper_mm - (z + r - (r * r - d * d).sqrt());
                if excess <= worst || excess <= self.op.tolerance_mm {
                    continue;
                }
                cancelled(cancel)?;
                let part = self.surface.contact(c, half, work, cancel)?.center_z;
                if cells[cell].upper_mm <= part + self.op.tolerance_mm {
                    continue;
                }
                worst = excess;
            }
        }
        Ok(worst)
    }
    /// A lower bound on stock the finishing lap encounters at its arrival
    /// point. Only the guaranteed occupied height can authorize a deeper
    /// link. The point nearest the ball axis in each cell is a witness.
    fn lap_depth(
        &self,
        stock: &Stock,
        p: [f64; 2],
        z: f64,
        cancel: &impl CancelSignal,
    ) -> Result<f64, String> {
        let r = self.radius;
        let widths = stock.cell_widths_mm();
        let mut depth = 0f64;
        self.visit_ball_cells(stock, p, p, cancel, |center, heights, guard| {
            let offset: [f64; 2] = std::array::from_fn(|k| {
                ((center[k] - p[k]).abs() - widths[k] / 2.).max(0.) + guard
            });
            let d = offset[0].hypot(offset[1]) + guard;
            if d < r {
                let bottom = z + r - ((r - d) * (r + d)).sqrt();
                depth = depth.max(heights.lower_mm - bottom - guard);
            }
        })?;
        Ok(depth)
    }

    fn visit_ball_cells(
        &self,
        stock: &Stock,
        a: [f64; 2],
        b: [f64; 2],
        cancel: &impl CancelSignal,
        mut visit: impl FnMut([f64; 2], crate::stock::HeightBounds, f64),
    ) -> Result<(), String> {
        let spec = stock.spec();
        let dims = stock.dimensions();
        let widths = stock.cell_widths_mm();
        let scale = a
            .into_iter()
            .chain(b)
            .chain(spec.min_mm)
            .chain(spec.max_mm)
            .map(f64::abs)
            .fold(self.radius.max(1.), f64::max);
        let guard = 128. * f64::EPSILON * scale;
        let range = |k: usize| {
            let index =
                |v: f64| (((v - spec.min_mm[k]) / widths[k]).floor().max(0.) as usize).min(dims[k]);
            index(a[k].min(b[k]) - self.radius - guard)
                ..index(a[k].max(b[k]) + self.radius + guard)
                    .saturating_add(1)
                    .min(dims[k])
        };
        for row in range(1) {
            cancelled(cancel)?;
            for col in range(0) {
                let cell = row * dims[0] + col;
                visit(stock.cell_center(cell).unwrap(), stock.cells()[cell], guard);
            }
        }
        Ok(())
    }

    /// Upper bound over the entire swept disk and every closed stock cell.
    /// Projection on any unit axis gives a lower bound on the distance of
    /// two convex sets. Use the two grid axes and the segment normal; at
    /// the ends also enclose the cell in its circumdisk. In particular a
    /// high corner cannot disappear because its cell centre misses the ball.
    fn stock_met(
        &self,
        stock: &Stock,
        a: Vec3,
        b: [f64; 2],
        lap: Option<Vec3>,
        limit: f64,
        cancel: &impl CancelSignal,
    ) -> Result<f64, String> {
        let dx = b[0] - a[0];
        let dy = b[1] - a[1];
        let length = dx.hypot(dy);
        if length == 0. {
            return Ok(0.);
        }
        let link = BallLink {
            start: a,
            end: b,
            unit: [dx / length, dy / length],
            length,
            radius: self.radius,
            lap,
            limit,
        };
        let half = stock.cell_widths_mm().map(|w| w / 2.);
        let mut worst = 0f64;
        self.visit_ball_cells(stock, [a[0], a[1]], b, cancel, |center, height, guard| {
            if worst <= limit && height.upper_mm > a[2] + worst - guard {
                worst = worst.max(link.cell_depth(center, half, height.upper_mm, guard, 5));
            }
        })?;
        Ok(worst)
    }
    /// Ramp: a main level descends from the one above (none for the first
    /// main level); the lowest main level is cut once more flat. In-between
    /// levels are cut flat at their own z, one pass (#46).
    fn ramp_setup(&self, slot: usize) -> (Option<f64>, usize) {
        if !self.op.ramp {
            return (None, 1);
        }
        let slot = &self.schedule[slot];
        if slot.depth > 0 {
            return (None, 1);
        }
        (slot.ramp_from, if slot.last_main { 2 } else { 1 })
    }
}

/// Tool-center loops of a flat or bull-nose end mill at tip height `z`, and
/// the largest stated error of the shadows they were offset from.
///
/// Flat: the boundary of the part's shadow at or above `z - allowance`
/// (`allowance`: stock to leave plus safety margin, the vertical clearance
/// under the tip) grown by `radius` (the cutter radius plus the same clearance
/// and the mesh's declared error). Bull nose (corner `corner`): a part point u
/// above the tip is reached within R - rc + sqrt(rc^2 - (rc - u)^2); the
/// corner's height is split into bands, each band's shadow grown by the
/// largest reach in it, and the regions joined: conservative (never closer
/// than allowed) by at most a few hundredths of the corner radius.
///
/// The shadow is the one 3D roughing protects its layers with
/// (`PreparedRoughingTarget::shadow_for_cutter`, also `silhouette_region`):
/// its level is lowered by the mesh's declared error and a numerical reserve,
/// facets seen edge-on are enclosed, never dropped, and a facet the planar
/// kernel cannot take is a refusal. Each shadow is grown by what its
/// `boundary_error_mm` states beyond the mesh's declared error (already in
/// `radius`) as well, as `Contour` adds a region's error to its clearance.
/// Holes of a shadow too small for a disk of half its reach are filled: its
/// growth by the whole reach closes them anyway. The shadow is closed by the
/// mesh's declared error (seams below it are bridged, see
/// `PreparedRoughingTarget::shadow`): growing by a reach of at least that
/// error gives the same region as from the exact shadow, and `radius`
/// includes the error. Only a bull-nose band near the tip, whose reach is
/// smaller than the error, can stand up to the error farther out of a sharp
/// concave corner than the exact shadow would put it.
#[allow(clippy::too_many_arguments)]
pub(crate) fn flat_level_loops(
    target: &PreparedRoughingTarget,
    z: f64,
    radius: f64,
    corner: f64,
    allowance: f64,
    tolerance_mm: f64,
    cancel: &impl CancelSignal,
) -> Result<(Vec<Vec<Stroke>>, f64), String> {
    use crate::planar::{Boolean, ConstructionOptions};
    let options = ConstructionOptions { tolerance_mm };
    let source_error = target.mesh().surface_error_mm;
    let mut region: Option<crate::planar::PreparedRegion> = None;
    let mut error = 0f64;
    let mut add = |tip: f64, reach: f64| -> Result<(), String> {
        cancelled(cancel)?;
        let shadow = target
            .shadow_for_cutter(tip, allowance, reach, &mut 0, cancel)
            .map_err(|e| format!("waterline flat-cutter shadow at Z{tip}: {e}"))?
            .region;
        if shadow.components.is_empty() {
            return Ok(());
        }
        error = error.max(shadow.boundary_error_mm);
        let reach = reach + (shadow.boundary_error_mm - source_error).max(0.);
        let grown = shadow
            .prepare(options)
            .map_err(|e| format!("waterline flat-cutter shadow at Z{tip}: {e}"))?
            .offset_prepared(-reach)?
            .into_parts()
            .1;
        region = Some(match region.take() {
            None => grown,
            Some(r) => r.boolean_prepared(&grown, Boolean::Union)?.into_parts().1,
        });
        Ok(())
    };
    if corner <= 0. {
        add(z, radius)?;
    } else {
        // Bands thin near the tip (where the reach changes fastest).
        const BANDS: usize = 24;
        let reach =
            |u: f64| radius - corner + (corner * corner - (corner - u).powi(2)).max(0.).sqrt();
        for k in 0..BANDS {
            let u0 = corner * (k as f64 / BANDS as f64).powi(3);
            let u1 = corner * ((k + 1) as f64 / BANDS as f64).powi(3);
            add(z + u0, reach(u1))?;
        }
        add(z + corner, radius)?;
    }
    let Some(region) = region else {
        return Ok((vec![], error));
    };
    let mut loops = vec![];
    for c in &region.definition().components {
        for b in std::iter::once(&c.outer).chain(&c.holes) {
            loops.push(Stroke::from_boundary(b));
        }
    }
    Ok((loops, error))
}

#[cfg(test)]
mod bridge_tests {
    use super::*;
    /// A 10 x 10 square contour and the runs kept of it, as arc-length spans.
    fn kept(spans: &[(f64, f64)]) -> (Vec<[f64; 2]>, Vec<Vec<[f64; 2]>>) {
        let ring = vec![[0., 0.], [10., 0.], [10., 10.], [0., 10.]];
        let strokes: Vec<Stroke> = ring
            .iter()
            .zip(ring.iter().cycle().skip(1))
            .take(4)
            .map(|(a, b)| Stroke::line(*a, *b))
            .collect();
        let at = |s: f64| loop_point_tangent(&strokes, s).0;
        let runs = spans.iter().map(|&(a, b)| vec![at(a), at(b)]).collect();
        (ring, runs)
    }
    #[test]
    fn in_between_fragments_with_short_gaps_are_cut_whole() {
        // Gaps of 2, 3 and 2 mm (the last across the contour's start) with a
        // 4 mm limit: cut whole. One gap of 6 mm: kept as fragments (#46).
        let (ring, runs) = kept(&[(1., 10.), (12., 25.), (28., 39.)]);
        assert!(runs_bridge(&ring, &runs, 4.));
        let (ring, runs) = kept(&[(1., 10.), (16., 39.)]);
        assert!(!runs_bridge(&ring, &runs, 4.));
        // A single fragment leaving one short gap: whole; a long one: not.
        let (ring, runs) = kept(&[(2., 39.)]);
        assert!(runs_bridge(&ring, &runs, 4.));
        let (ring, runs) = kept(&[(2., 30.)]);
        assert!(!runs_bridge(&ring, &runs, 4.));
    }
    #[test]
    fn bridged_fragments_follow_a_run_that_closes_at_its_seam() {
        // A region limit admits the whole 10 x 10 square, so the run the
        // limits allow closes on itself; the scallop rule keeps two
        // fragments 2 mm apart, the second ending at the seam. The bridged
        // piece follows the square to the seam, never the diagonal back to
        // the run's start (#46).
        let run = vec![[0., 0.], [10., 0.], [10., 10.], [0., 10.], [0., 0.]];
        let kept = vec![
            vec![[10., 2.], [10., 10.], [0., 10.]],
            vec![[0., 8.], [0., 0.]],
        ];
        assert_eq!(
            bridge_open(&run, &kept, 4.),
            Some(vec![[10., 2.], [10., 10.], [0., 10.], [0., 0.]])
        );
    }
}
#[cfg(test)]
mod entry_tests {
    use super::*;
    use std::sync::atomic::AtomicBool;
    #[test]
    fn a_loop_rotated_twice_near_a_vertex_stays_continuous() {
        // #392: a split 5e-10 mm past a vertex is the vertex; the loop then
        // starts and ends exactly there. Rotated again (the flat lap after a
        // ramp), it used to carry the rounding gap mid-loop, which the
        // stream refuses as a break in continuity.
        let square = [[0., 0.], [1., 0.], [1., 1.], [0., 1.]];
        for s in [1. + 5e-10, 2. - 5e-10] {
            let mut edges: Vec<Stroke> = (0..4)
                .map(|i| Stroke::line(square[i], square[(i + 1) % 4]))
                .collect();
            rotate_loop(&mut edges, s);
            rotate_loop(&mut edges, 2.5);
            for w in edges.windows(2) {
                assert_eq!(w[0].b, w[1].a, "{edges:?}");
            }
            assert_eq!(edges[edges.len() - 1].b, edges[0].a, "{edges:?}");
        }
    }
    #[test]
    fn an_open_run_too_short_to_reverse_on_ramps_along_its_contour() {
        // #392: a 0.4 mm run of a 10 x 10 contour, min_length_mm 1. Along
        // the run itself there is no ramp; along the contour from the run's
        // start and back there is, ending exactly where the run starts.
        let manifest: crate::stream::StreamManifest =
            serde_json::from_str(include_str!("../fixtures/operations/waterline.json")).unwrap();
        let crate::pocket::Operation::Waterline(op) = &manifest.operations[0] else {
            panic!("fixture");
        };
        let mut op = op.clone();
        op.entry = Some(crate::entry::PocketEntry::Ramp(crate::entry::RampEntry {
            angle_rad: 3f64.to_radians(),
            min_length_mm: Some(1.),
            feed_mm_min: None,
        }));
        let plan = WaterlinePlan::prepare(
            op,
            &manifest.tools[0],
            manifest.stock.height().unwrap(),
            manifest.initial_position_mm,
            1,
            &AtomicBool::new(false),
        )
        .unwrap();
        let ring = [[0., 0.], [10., 0.], [10., 10.], [0., 10.]];
        let run = [Stroke::line([3., 0.], [3.4, 0.])];
        assert!(!plan.run_ramp_fits(&run, 0.5));
        assert!(plan.run_ramp(&run, 1., 0.5).is_err());
        let ramp = plan.contour_ramp(&ring, &run, 1., 0.5).unwrap();
        assert_eq!(ramp[0].start(), [3., 0., 1.]);
        assert_eq!(ramp.last().unwrap().end(), [3., 0., 0.5]);
        for w in ramp.windows(2) {
            assert_eq!(w[0].end(), w[1].start());
        }
        // Out along the run and on along the contour: 0.5 / (2 tan 3 deg)
        // = 4.77 mm, then back.
        let far = ramp.iter().map(|c| c.end()[0]).fold(0., f64::max);
        assert!(
            (far - (3. + 0.25 / 3f64.to_radians().tan())).abs() < 1e-9,
            "{far}"
        );
        // A contour shorter than min_length_mm is refused by name.
        let tiny = [[0., 0.], [0.2, 0.], [0.2, 0.2], [0., 0.2]];
        let short = [Stroke::line([0.05, 0.], [0.1, 0.])];
        let error = plan.contour_ramp(&tiny, &short, 1., 0.5).unwrap_err();
        assert!(
            error.contains("min_length_mm") && error.contains("rough or rest-rough"),
            "{error}"
        );
    }
    #[test]
    fn accepted_link_cells_respect_the_stock_limit_or_the_finishing_ball() {
        // Independent section of the swept sphere on a horizontal segment.
        // Include cells behind and beyond its ends, at the rim, and on
        // both sides of the small gap between endpoint footprints.
        let link = BallLink {
            start: [0., 0., 2.],
            end: [0.8, 0.],
            unit: [1., 0.],
            length: 0.8,
            radius: 1.,
            lap: Some([0.8, 0., 1.8]),
            limit: 0.2,
        };
        let bottom = |z: f64, d: f64| {
            if d <= 1. {
                z + 1. - (1. - d * d).sqrt()
            } else {
                f64::INFINITY
            }
        };
        let mut decisions = [0; 2];
        for ix in -6..11 {
            for iy in -6..7 {
                let center = [0.2 * f64::from(ix), 0.2 * f64::from(iy)];
                let accepted = link.cell_depth(center, [0.1, 0.1], 4., 1e-12, 5) <= link.limit;
                decisions[usize::from(accepted)] += 1;
                if !accepted {
                    continue;
                }
                for u in 0..=8 {
                    for v in 0..=8 {
                        let x = center[0] - 0.1 + f64::from(u) * 0.025;
                        let y = center[1] - 0.1 + f64::from(v) * 0.025;
                        let stock = 4f64.min(bottom(2., x.hypot(y)));
                        let cut = stock - bottom(2., (x - x.clamp(0., 0.8)).hypot(y));
                        let lap = stock - bottom(1.8, (x - 0.8).hypot(y));
                        assert!(cut <= 0.2f64.max(lap) + 1e-12, "at {x},{y}: {cut} vs {lap}");
                    }
                }
            }
        }
        assert!(decisions.into_iter().all(|count| count > 0));
    }

    #[test]
    fn open_ramp_honors_the_declared_reversal_span() {
        let manifest: crate::stream::StreamManifest =
            serde_json::from_str(include_str!("../fixtures/operations/waterline.json")).unwrap();
        let crate::pocket::Operation::Waterline(op) = &manifest.operations[0] else {
            panic!("fixture");
        };
        let mut op = op.clone();
        op.entry = Some(crate::entry::PocketEntry::Ramp(crate::entry::RampEntry {
            angle_rad: 10f64.to_radians(),
            min_length_mm: Some(4.),
            feed_mm_min: None,
        }));
        let plan = WaterlinePlan::prepare(
            op,
            &manifest.tools[0],
            manifest.stock.height().unwrap(),
            manifest.initial_position_mm,
            1,
            &AtomicBool::new(false),
        )
        .unwrap();
        let line = |x| [Stroke::line([0., 0.], [x, 0.])];
        let ramp = plan.run_ramp(&line(6.), 1., 0.99).unwrap();
        assert!(ramp.iter().map(|curve| curve.end()[0]).fold(0., f64::max) >= 4.);
        assert!(
            plan.run_ramp(&line(3.), 1., 0.99)
                .unwrap_err()
                .contains("min_length_mm")
        );
    }

    #[test]
    fn slope_limits_leave_out_in_between_levels_that_keep_nothing() {
        // #398: the roof of tests/waterline.rs (20.6 degree planes, ball
        // R2, stepdown 0.4) wants in-between levels down to depth 2: on its
        // planes sin(t) = 0.352 is below the depth-2 bound 0.355. Under
        // slope_from_deg 25 a sample is kept only where sin(tg) >= sin(25) =
        // 0.423, and a depth-2 level keeps only sin(tg) < 0.355: it keeps
        // nothing, so it is not planned (its contours are the work). Depth 1
        // (bound 0.709) stays. The toolpath is the same either way; the
        // roof's integration tests pin it.
        let m: crate::stream::StreamManifest =
            serde_json::from_str(include_str!("../fixtures/operations/waterline.json")).unwrap();
        let crate::pocket::Operation::Waterline(op) = &m.operations[0] else {
            panic!("fixture");
        };
        let mut op = op.clone();
        let (a, b, c, d, e, f) = (
            [4., 4., 4.],
            [12., 4., 7.],
            [20., 4., 4.],
            [4., 12., 4.],
            [12., 12., 7.],
            [20., 12., 4.],
        );
        op.surface = crate::surface::SurfaceMesh {
            version: 1,
            units: "mm".into(),
            surface_error_mm: 0.,
            source: serde_json::Value::Null,
            triangles: vec![[a, b, e], [a, e, d], [b, c, f], [b, f, e]].into(),
        };
        op.top_z_mm = 6.8;
        op.bottom_z_mm = 4.4;
        op.stepdown_mm = 0.4;
        let spec = m.stock.height().unwrap().clone();
        let plan = |slope_from_deg: f64| {
            let mut op = op.clone();
            op.slope_from_deg = slope_from_deg;
            WaterlinePlan::prepare(
                op,
                &m.tools[0],
                &spec,
                m.initial_position_mm,
                1,
                &AtomicBool::new(false),
            )
            .unwrap()
        };
        let depths = |p: &WaterlinePlan| {
            let mut count = [0usize; 8];
            for slot in &p.schedule {
                count[slot.depth.min(7)] += 1;
            }
            count
        };
        let unlimited = plan(0.);
        let limited = plan(25.);
        let (all, kept) = (depths(&unlimited), depths(&limited));
        // Main levels Z6.8 .. Z4.4 either way.
        assert_eq!((all[0], kept[0]), (7, 7));
        // Without a slope limit the planes need depth 2 (and depth 1).
        assert!(all[1] > 0 && all[2] > 0, "{all:?}");
        // Under the limit depth 1 stays whole and nothing deeper is planned.
        assert_eq!(kept[1], all[1], "{kept:?}");
        assert_eq!(kept[2..].iter().sum::<usize>(), 0, "{kept:?}");
        // Every planned in-between level can keep a sample.
        let floor = (25f64.to_radians() - 1e-9).sin();
        for slot in limited.schedule.iter().filter(|s| s.depth > 0) {
            assert!(slot.keep_below > floor, "{}", slot.keep_below);
        }
    }
    fn stock_with_wall_and_isolated_cell(isolated_lower: f64) -> (WaterlinePlan, Stock) {
        let m: crate::stream::StreamManifest =
            serde_json::from_str(include_str!("../fixtures/operations/waterline.json")).unwrap();
        let mut op = match &m.operations[0] {
            crate::pocket::Operation::Waterline(p) => p.clone(),
            _ => panic!("fixture"),
        };
        let (a, b, c, d) = ([8., 5., 6.], [16., 5., 6.], [16., 11., 6.], [8., 11., 6.]);
        op.surface = crate::surface::SurfaceMesh {
            version: 1,
            units: "mm".into(),
            surface_error_mm: 0.,
            source: serde_json::Value::Null,
            triangles: vec![[a, b, c], [a, c, d]].into(),
        };
        let cancel = AtomicBool::new(false);
        let spec = m.stock.height().unwrap().clone();
        let plan =
            WaterlinePlan::prepare(op, &m.tools[0], &spec, m.initial_position_mm, 1, &cancel)
                .unwrap();
        let probe = Stock::new(spec.clone()).unwrap();
        let [nx, ny] = probe.dimensions();
        let [wx, wy] = probe.cell_widths_mm();
        let mut bounds = Vec::with_capacity(nx * ny);
        for j in 0..ny {
            for i in 0..nx {
                let (cx, cy) = ((i as f64 + 0.5) * wx, (j as f64 + 0.5) * wy);
                let wall = (15.2..15.6).contains(&cx);
                let cell = (cx - 20.2).abs() < 1e-9 && (cy - 8.2).abs() < 1e-9;
                let top = if wall || cell { 6. } else { 2. };
                bounds.push(crate::stock::HeightBounds {
                    lower_mm: if cell { isolated_lower } else { top },
                    upper_mm: top,
                });
            }
        }
        let initial = crate::stock::InitialStock {
            source_id: "entry".into(),
            revision: "1".into(),
            source_sha256: "0".repeat(64),
            source_to_setup: crate::motion::WorkFrame::identity(),
            uncertainty_mm: 0.,
            representation: "whole_cell_top_down".into(),
            bounds,
        };
        (plan, Stock::from_initial(spec, &initial).unwrap())
    }

    #[test]
    fn stock_at_a_cell_edge_blocks_the_lead_and_uncertain_stock_cannot_authorize_it() {
        let cancel = AtomicBool::new(false);
        let (plan, stock) = stock_with_wall_and_isolated_cell(6.);
        // The edge of the elevated cell at (20.2, 8.2) reaches X20.
        // Its centre misses this lead by 2.1 mm; its left edge is only
        // 1.9 mm from the centreline of the 2 mm ball. At the closest
        // point the ball is below Z5 and the cell is solid up to Z6.
        let rim = plan
            .stock_met(&stock, [18.1, 4., 3.], [18.1, 10.], None, 1., &cancel)
            .unwrap();
        assert!(
            rim >= 1.,
            "the swept ball meets at least 1 mm of stock at the cell edge, got {rim}"
        );
        let (_, uncertain) = stock_with_wall_and_isolated_cell(2.);
        // Both blanks have the same possible top. Only the first guarantees
        // that the lower lap will actually cut anything at this location.
        assert!(plan.lap_depth(&stock, [20.2, 8.2], 3., &cancel).unwrap() > 2.99);
        assert_eq!(
            plan.lap_depth(&uncertain, [20.2, 8.2], 3., &cancel)
                .unwrap(),
            0.
        );
    }

    #[test]
    fn a_moved_side_entry_proves_the_lead_it_emits() {
        // The start moves away from the wall. Check the lead all the way to
        // the snapped vertex against the elevated cell beside the contour.
        let cancel = AtomicBool::new(false);
        let (plan, stock) = stock_with_wall_and_isolated_cell(6.);
        let (s0, v, w, x) = ([17.199, 8.2], [18.202, 8.2], [18.202, 12.2], [17.199, 12.2]);
        let mut loops = vec![vec![
            Stroke::line(s0, v),
            Stroke::line(v, w),
            Stroke::line(w, x),
            Stroke::line(x, s0),
        ]];
        let z = 3.;
        let (points, _, moved, _) = plan.entries(&stock, &mut loops, z, z, &cancel).unwrap();
        let p = points[0].expect("a side entry at a moved start");
        assert_eq!(moved, 1, "the start moved");
        let start = loops[0][0].a;
        let met = plan
            .stock_met(
                &stock,
                [p[0], p[1], z],
                start,
                None,
                plan.op.tolerance_mm,
                &cancel,
            )
            .unwrap();
        assert!(
            met <= plan.op.tolerance_mm,
            "the lead from {p:?} onto the moved start {start:?} meets {met} mm of stock"
        );
    }
}
