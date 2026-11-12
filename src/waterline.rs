//! Conservative adaptive waterlines with continuous solid/error certificates.
//! Geometry contract, including inside distance and contour error: docs/waterline.md.
use crate::{
    Point,
    finishing::{FinishingStockTop, StockCeiling},
    motion::{Coolant, Motion, MoveKind, Vec3},
    pocket::{ResidualStatus, Stroke},
    profiles::{ProfileDepths, ProfileMotion, ProfileSettings},
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
        let horizontal = contact.normal[0].hypot(contact.normal[1]);
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
        if (next[0] - sample[0]).hypot(next[1] - sample[1]) > band + tie {
            break;
        }
        center = next;
    }
    // Nothing of the part within reach of the level: as on the base.
    let [low, high] = range.unwrap_or([1., 1.]);
    Ok((high.clamp(-1., 1.).acos(), low.clamp(-1., 1.).acos()))
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
    /// (the lowest level is then cut once more flat).
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub ramp: bool,
}
impl<G> WaterlineOperation<G> {
    pub(crate) fn map_geometry<H, E>(
        self,
        f: &mut impl FnMut(G) -> Result<H, E>,
    ) -> Result<WaterlineOperation<H>, E> {
        Ok(WaterlineOperation {
            ramp: self.ramp,
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
    level: usize,
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
            level: 0,
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
        })
    }
    fn level_options(&self, level: usize) -> (f64, ContourOptions) {
        let z = if level + 1 == self.report.levels {
            self.op.bottom_z_mm
        } else {
            (self.op.top_z_mm - level as f64 * self.op.stepdown_mm).max(self.op.bottom_z_mm)
        };
        (
            z,
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
            },
        )
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
        let end = (self.level + batch).min(self.report.levels);
        let inner = if quad {
            (workers / (end - self.level).max(1)).max(1)
        } else {
            1
        };
        let jobs: Vec<_> = (self.level..end).map(|l| self.level_options(l)).collect();
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
                self.level += 1;
                self.report.levels_completed += 1;
            }
            if self.level == self.report.levels {
                self.drained = true;
                return Ok(None);
            }
            if let Some(target) = &self.flat_target {
                let (z, _) = self.level_options(self.level);
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
                    self.level += 1;
                    self.report.empty_levels += 1;
                    self.report.levels_completed += 1;
                    continue;
                }
                let (ramp_from, passes) = self.ramp_setup();
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
            let mut loops: Vec<Vec<Stroke>> = vec![];
            for mut ring in contours.loops {
                if self.op.reverse {
                    ring.reverse();
                }
                if self.limits.is_none() && self.region_limits.is_none() {
                    loops.push(
                        ring.iter()
                            .zip(ring.iter().cycle().skip(1))
                            .take(ring.len())
                            .map(|(a, b)| Stroke::line(*a, *b))
                            .collect(),
                    );
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
                // The ball's center on this level, and how far a contour
                // sample may lie from the offset surface (inside it down to
                // the protected radius, outside it by the contour error).
                let center_z = z + self.radius;
                let band = (self.inflated - self.protected).max(0.) + self.op.contour_error_mm;
                let tolerance = self.op.tolerance_mm;
                let mut runs = crate::finishing::filter_runs(vec![closed.clone()], spacing, |p| {
                    if let Some((lo, hi)) = slope {
                        let (least, greatest) = level_contact_angles(
                            surface,
                            [p[0], p[1], center_z],
                            inflated,
                            band,
                            tolerance,
                            &mut slope_work,
                            cancel,
                        )?;
                        if greatest < lo - 1e-9 || least > hi + 1e-9 {
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
                })?;
                let whole = runs.len() == 1
                    && runs[0].first() == closed.first()
                    && runs[0].last() == closed.last();
                if whole {
                    // Entirely within the limits: keep the original contour.
                    loops.push(
                        ring.iter()
                            .zip(ring.iter().cycle().skip(1))
                            .take(ring.len())
                            .map(|(a, b)| Stroke::line(*a, *b))
                            .collect(),
                    );
                    continue;
                }
                if runs.len() >= 2
                    && runs[0].first() == closed.first()
                    && runs.last().unwrap().last() == closed.last()
                {
                    // A run through the ring's start: join its two halves.
                    let first = runs.remove(0);
                    runs.last_mut().unwrap().extend(first.into_iter().skip(1));
                }
                loops.extend(runs.into_iter().map(|run| {
                    run.windows(2)
                        .map(|w| Stroke::line(w[0], w[1]))
                        .collect::<Vec<_>>()
                }));
            }
            let work = sum(work, slope_work);
            let empty = loops.is_empty();
            cancelled(cancel)?;
            self.report.work = work;
            self.report.certificate_tests = certificates;
            self.report.certificate_nodes = certificate_nodes;
            self.report.last_level_z_mm = Some(z);
            self.report.level_reports.push(contours.report);
            if empty {
                self.level += 1;
                self.report.empty_levels += 1;
                self.report.levels_completed += 1;
                continue;
            }
            let (ramp_from, passes) = self.ramp_setup();
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
            self.profile = Some(profile);
        }
    }
    /// Ramp: the current level descends from the one above (none for the
    /// first level); the lowest level is cut once more flat.
    fn ramp_setup(&self) -> (Option<f64>, usize) {
        if !self.op.ramp {
            return (None, 1);
        }
        let from = (self.level > 0).then(|| self.level_options(self.level - 1).0);
        let last = self.level + 1 == self.report.levels;
        (from, if last { 2 } else { 1 })
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
