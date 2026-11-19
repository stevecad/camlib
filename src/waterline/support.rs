//! Construct convex slice enclosures, then union through the existing M2 kernel.
use super::*;
use crate::planar::{
    Boolean, Boundary, Component, ConstructionOptions, PlanarRegion, PreparedRegion,
};
use robust::{Coord, orient2d};
fn dot(a: Point, b: Point) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
fn orientation(a: Point, b: Point, c: Point) -> f64 {
    orient2d(
        Coord { x: a[0], y: a[1] },
        Coord { x: b[0], y: b[1] },
        Coord { x: c[0], y: c[1] },
    )
}
fn hull(mut points: Vec<Point>) -> Vec<Point> {
    points.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
    points.dedup();
    if points.len() < 3 {
        return points;
    }
    fn chain(iter: impl Iterator<Item = Point>) -> Vec<Point> {
        let mut result: Vec<Point> = vec![];
        for p in iter {
            while result.len() >= 2
                && orientation(result[result.len() - 2], result[result.len() - 1], p) <= 0.
            {
                result.pop();
            }
            result.push(p);
        }
        result
    }
    let mut a = chain(points.clone().into_iter());
    let mut b = chain(points.into_iter().rev());
    a.pop();
    b.pop();
    a.extend(b);
    a
}
fn segment_witness(p: Point, a: Point, b: Point) -> Point {
    let d = [b[0] - a[0], b[1] - a[1]];
    let l2 = dot(d, d);
    let t = if l2 > 0. {
        (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / l2).clamp(0., 1.)
    } else {
        0.
    };
    [a[0] + t * d[0], a[1] + t * d[1]]
}
struct EdgeNode {
    min: Point,
    max: Point,
    start: usize,
    end: usize,
    children: Option<[usize; 2]>,
}
impl EdgeNode {
    fn distance2(&self, p: Point) -> f64 {
        (0..2)
            .map(|k| (self.min[k] - p[k]).max(p[k] - self.max[k]).max(0.).powi(2))
            .sum()
    }
}
// Every returned witness belongs to Q. Even for an interior query, distance to
// its boundary overestimates distance to Q, so the whole-polygon proof remains
// conservative. No uncounted linear inside scan is needed for each P vertex.
struct InnerHull {
    points: Vec<Point>,
    nodes: Vec<EdgeNode>,
}
impl InnerHull {
    fn new(points: Vec<Point>) -> Self {
        let edges = if points.len() <= 2 { 1 } else { points.len() };
        let mut result = Self {
            points,
            nodes: vec![],
        };
        result.build(0, edges);
        result
    }
    fn build(&mut self, start: usize, end: usize) -> usize {
        let mut min = [f64::INFINITY; 2];
        let mut max = [f64::NEG_INFINITY; 2];
        for i in start..end {
            for p in [self.points[i], self.points[(i + 1) % self.points.len()]] {
                for k in 0..2 {
                    min[k] = min[k].min(p[k]);
                    max[k] = max[k].max(p[k]);
                }
            }
        }
        let index = self.nodes.len();
        self.nodes.push(EdgeNode {
            min,
            max,
            start,
            end,
            children: None,
        });
        if end - start > 4 {
            let mid = start + (end - start) / 2;
            let a = self.build(start, mid);
            let b = self.build(mid, end);
            self.nodes[index].children = Some([a, b]);
        }
        index
    }
    fn nearest(&self, p: Point, tests: &mut usize, nodes: &mut usize) -> Result<Point, String> {
        let mut result = self.points[0];
        let mut best = f64::INFINITY;
        let mut stack = vec![0];
        while let Some(index) = stack.pop() {
            *nodes += 1;
            let node = &self.nodes[index];
            if node.distance2(p) > best {
                continue;
            }
            if let Some([a, b]) = node.children {
                if self.nodes[a].distance2(p) <= self.nodes[b].distance2(p) {
                    stack.extend([b, a]);
                } else {
                    stack.extend([a, b]);
                }
            } else {
                for i in node.start..node.end {
                    *tests += 1;
                    let q = segment_witness(
                        p,
                        self.points[i],
                        self.points[(i + 1) % self.points.len()],
                    );
                    let d = (p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2);
                    if d < best {
                        best = d;
                        result = q;
                    }
                }
            }
        }
        Ok(result)
    }
}
fn clip(poly: &[Point], normal: Point, limit: f64) -> Vec<Point> {
    let mut result = vec![];
    for (a, b) in poly
        .iter()
        .zip(poly.iter().cycle().skip(1))
        .take(poly.len())
    {
        let da = dot(*a, normal) - limit;
        let db = dot(*b, normal) - limit;
        if da <= 0. {
            result.push(*a);
        }
        if (da <= 0.) != (db <= 0.) {
            let t = (da / (da - db)).clamp(0., 1.);
            result.push([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]);
        }
    }
    result.dedup();
    if result.len() > 1 && result.first() == result.last() {
        result.pop();
    }
    result
}
// Removing a near-duplicate convex vertex contracts by at most its edge length.
// Track that contraction explicitly; outward support padding reserves it.
fn clean(poly: &mut Vec<Point>, tol: f64) -> Result<f64, String> {
    let threshold = 1e-6_f64.max(tol / 4.) * 1.001;
    let mut error = 0.;
    loop {
        let short = (0..poly.len()).find_map(|i| {
            let j = (i + 1) % poly.len();
            let d = (poly[i][0] - poly[j][0]).hypot(poly[i][1] - poly[j][1]);
            (d < threshold).then_some((j, d))
        });
        let Some((j, d)) = short else { break };
        if poly.len() <= 3 || error + d > 2. * tol {
            return Err("support polygon has edges below Boolean resolution".into());
        }
        error += d;
        poly.remove(j);
    }
    Ok(error)
}
struct Polygon {
    points: Vec<Point>,
    error: f64,
}
struct Builder {
    work: SurfaceWork,
    queries: usize,
    tests: usize,
    nodes: usize,
    vertices: usize,
    workers: usize,
}
impl Builder {
    fn query(
        &mut self,
        surface: &PreparedSurface,
        index: Option<usize>,
        o: ContourOptions,
        n: Point,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Point>, String> {
        self.queries += 1;
        match index {
            Some(index) => {
                surface.slice_support(index, o.center_z_mm, o.radius_mm, n, &mut self.work, cancel)
            }
            None => {
                surface.convex_slice_support(o.center_z_mm, o.radius_mm, n, &mut self.work, cancel)
            }
        }
    }
    fn query_many(
        &mut self,
        surface: &PreparedSurface,
        index: Option<usize>,
        o: ContourOptions,
        normals: &[Point],
        cancel: &impl CancelSignal,
    ) -> Result<Vec<Option<Point>>, String> {
        if index.is_some() || self.workers <= 1 || normals.len() < 4 {
            return normals
                .iter()
                .map(|&n| self.query(surface, index, o, n, cancel))
                .collect();
        }
        let mut combined = self.work;
        // Establish the immutable proof before workers start so its cost is
        // charged once deterministically, including on the first query batch.
        if !surface.convex_solid(&mut combined, cancel)? {
            return Err("global support requires a certified convex solid".into());
        }
        let values = crate::roughing::parallel::ordered(
            normals.len(),
            self.workers,
            cancel,
            |i, signal| {
                let stop = || signal.is_cancelled();
                let mut work = SurfaceWork::default();
                let point = surface.convex_slice_support(
                    o.center_z_mm,
                    o.radius_mm,
                    normals[i],
                    &mut work,
                    &stop,
                )?;
                Ok((point, work))
            },
            |(_, work)| {
                combined = sum(combined, *work);
                Ok(())
            },
        )?;
        cancelled(cancel)?;
        self.work = combined;
        self.queries += normals.len();
        Ok(values.into_iter().map(|(point, _)| point).collect())
    }
    fn polygon(
        &mut self,
        surface: &PreparedSurface,
        index: Option<usize>,
        o: ContourOptions,
        depth: usize,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Polygon>, String> {
        let padding = (depth + 8) as f64 * o.tolerance_mm;
        let budget = o.normal_error_mm - padding;
        let normals = [[1., 0.], [0., 1.], [-1., 0.], [0., -1.]];
        let Some(first) = self.query(surface, index, o, normals[0], cancel)? else {
            return Ok(None);
        };
        let mut inner = vec![first];
        let mut directions = vec![normals[0]];
        for n in &normals[1..] {
            inner.push(
                self.query(surface, index, o, *n, cancel)?
                    .ok_or("inconsistent empty convex slice")?,
            );
            directions.push(*n);
        }
        let h: Vec<_> = inner
            .iter()
            .zip(normals)
            .map(|(p, n)| dot(*p, n) + padding)
            .collect();
        let mut outer = vec![[-h[2], -h[3]], [h[0], -h[3]], [h[0], h[1]], [-h[2], h[1]]];
        loop {
            cancelled(cancel)?;
            let q = InnerHull::new(hull(inner.clone()));
            let mut worst = 0f64;
            let mut refinements = vec![];
            for p in &outer {
                let close = q.nearest(*p, &mut self.tests, &mut self.nodes)?;
                let distance = (p[0] - close[0]).hypot(p[1] - close[1]);
                worst = worst.max(distance);
                if distance > budget {
                    refinements.push([(p[0] - close[0]) / distance, (p[1] - close[1]) / distance]);
                }
            }
            if worst <= budget {
                let contraction = clean(&mut outer, o.tolerance_mm)?;
                self.vertices += outer.len();
                return Ok(Some(Polygon {
                    points: outer,
                    error: worst + contraction + 4. * o.tolerance_mm,
                }));
            }
            // All directions come from the same conservative P/Q pair.
            // Each exact support halfplane contains K independently, so they
            // may be added together. Certify EVERY vertex of the resulting P
            // against the rebuilt Q next round before accepting anything.
            // Refining just one vertex redoes that whole certificate and hull
            // construction for every direction on every source facet.
            let before = directions.len();
            let mut selected = Vec::new();
            for witness in refinements {
                if directions
                    .iter()
                    .any(|n| (n[0] - witness[0]).hypot(n[1] - witness[1]) < 1e-12)
                {
                    continue;
                }
                if directions.len() >= 4096 {
                    return Err("support polygon direction limit exceeded".into());
                }
                directions.push(witness);
                selected.push(witness);
            }
            let values = self.query_many(surface, index, o, &selected, cancel)?;
            for (witness, value) in selected.into_iter().zip(values) {
                let p = value.ok_or("convex slice support vanished during refinement")?;
                outer = clip(&outer, witness, dot(p, witness) + padding);
                if outer.len() < 3 {
                    return Err("support halfplanes collapsed a nonempty slice".into());
                }
                inner.push(p);
            }
            if directions.len() == before {
                return Err("support polygon refinement stalled at numerical resolution".into());
            }
        }
    }
}
pub(super) fn prepare(
    surface: &PreparedSurface,
    o: ContourOptions,
    cancel: &impl CancelSignal,
) -> Result<WaterlineContours, String> {
    let depth = (surface.prism_count() as f64).log2().ceil() as usize;
    let mut builder = Builder {
        work: SurfaceWork::default(),
        queries: 0,
        tests: 0,
        nodes: 0,
        vertices: 0,
        workers: if surface.prism_count() >= 4096 {
            std::thread::available_parallelism()
                .map_or(1, usize::from)
                .min(16)
        } else {
            1
        },
    };
    let options = ConstructionOptions {
        tolerance_mm: o.tolerance_mm,
    };
    let mut regions: Vec<PreparedRegion> = vec![];
    let mut input_error = 0f64;
    #[cfg(feature = "profile-planning")]
    let polygon_profile = crate::planning_profile::span("waterline.support_polygons");
    // For a certified convex downward solid, the offset slice is convex too.
    // Global support witnesses belong to that slice; their convex hull is an
    // inner bound, so the SAME complete outer/inner polygon certificate applies.
    // The ordinary per-prism construction/union remains the general fallback.
    let convex = surface.convex_solid(&mut builder.work, cancel)?;
    for index in 0..if convex { 1 } else { surface.prism_count() } {
        cancelled(cancel)?;
        let Some(p) = builder.polygon(surface, (!convex).then_some(index), o, depth, cancel)?
        else {
            continue;
        };
        input_error = input_error.max(p.error);
        let region = PlanarRegion {
            version: 1,
            units: "mm".into(),
            name: "waterline-support".into(),
            boundary_error_mm: p.error,
            source: serde_json::Value::Null,
            components: vec![Component {
                outer: Boundary {
                    vertices: p.points.into_iter().map(|p| [p[0], p[1], 0.]).collect(),
                },
                holes: vec![],
            }],
        };
        regions.push(region.prepare(options)?);
    }
    #[cfg(feature = "profile-planning")]
    drop(polygon_profile);
    let mut unions = 0;
    #[cfg(feature = "profile-planning")]
    let union_profile = crate::planning_profile::span("waterline.unions");
    // Balanced union bounds construction error by tree depth, not prism count.
    while regions.len() > 1 {
        let mut next = vec![];
        let mut iter = regions.into_iter();
        while let Some(a) = iter.next() {
            cancelled(cancel)?;
            if let Some(b) = iter.next() {
                let result = a.boolean(&b, Boolean::Union)?.region.prepare(options)?;
                next.push(result);
                unions += 1;
            } else {
                next.push(a);
            }
        }
        regions = next;
    }
    #[cfg(feature = "profile-planning")]
    drop(union_profile);
    let mut rings = vec![];
    let mut error = 0.;
    if let Some(region) = regions.pop() {
        error = region.definition().boundary_error_mm;
        if error > o.normal_error_mm {
            return Err("support/Boolean approximation exceeds normal contour budget".into());
        }
        for component in &region.definition().components {
            for boundary in std::iter::once(&component.outer).chain(&component.holes) {
                if boundary.vertices.iter().any(|v| v[2] != 0.) {
                    return Err("support Boolean unexpectedly produced a curved edge".into());
                }
                rings.push(
                    boundary
                        .vertices
                        .iter()
                        .map(|p| [p[0], p[1]])
                        .collect::<Vec<_>>(),
                );
            }
        }
    }
    let report = LevelReport {
        method: ContourMethod::Support,
        support_queries: builder.queries,
        support_vertices: builder.vertices,
        certificate_tests: builder.tests,
        certificate_nodes: builder.nodes,
        boolean_operations: unions,
        support_error_mm: input_error,
        boolean_error_mm: (error - input_error).max(0.),
        tip_z_mm: 0.,
        grid_cells: 0,
        finest_cell_mm: 0.,
        raw_edges: rings.iter().map(Vec::len).sum(),
        cut_edges: 0,
        loops: 0,
        removed_vertices: 0,
        max_normal_excess_mm: error,
        work: builder.work,
    };
    finish_contours(surface, rings, o, report, cancel)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn builder(workers: usize) -> Builder {
        Builder {
            work: SurfaceWork::default(),
            queries: 0,
            tests: 0,
            nodes: 0,
            vertices: 0,
            workers,
        }
    }
    fn hemisphere() -> PreparedSurface {
        let saved: serde_json::Value =
            serde_json::from_str(include_str!("../../fixtures/operations/hemisphere.json"))
                .unwrap();
        let mesh: SurfaceMesh =
            serde_json::from_value(saved["operations"][0]["surface"].clone()).unwrap();
        mesh.prepare(2., 1e-5, &|| false).unwrap()
    }
    fn settings() -> ContourOptions {
        ContourOptions {
            method: ContourMethod::Support,
            center_z_mm: 6.,
            radius_mm: 2.01,
            protected_radius_mm: 2.005,
            normal_error_mm: 0.01,
            tolerance_mm: 1e-5,
            xy_min_mm: [0., 0.],
            xy_max_mm: [16., 16.],
            simplify: true,
        }
    }
    #[test]
    fn parallel_global_refinement_matches_serial_points_error_and_total_work() {
        let serial_surface = hemisphere();
        let parallel_surface = hemisphere();
        let (mut serial, mut parallel) = (builder(1), builder(4));
        let a = serial
            .polygon(&serial_surface, None, settings(), 12, &|| false)
            .unwrap()
            .unwrap();
        let b = parallel
            .polygon(&parallel_surface, None, settings(), 12, &|| false)
            .unwrap()
            .unwrap();
        assert_eq!(a.points, b.points);
        assert_eq!(a.error, b.error);
        assert_eq!(serial.queries, parallel.queries);
        assert_eq!(serial.tests, parallel.tests);
        assert_eq!(serial.nodes, parallel.nodes);
        assert_eq!(
            serde_json::to_value(serial.work).unwrap(),
            serde_json::to_value(parallel.work).unwrap()
        );
    }
    #[test]
    fn parallel_global_queries_refuse_aggregate_work_and_cancel_without_partial_commit() {
        use std::cell::Cell;
        let surface = hemisphere();
        assert!(
            surface
                .convex_solid(&mut SurfaceWork::default(), &|| false)
                .unwrap()
        );
        let normals: Vec<_> = (0..32)
            .map(|i| {
                let t = i as f64 * std::f64::consts::TAU / 32.;
                [t.cos(), t.sin()]
            })
            .collect();
        // Work is counted, never capped: a large prior count still completes
        // and the aggregate keeps growing.
        let mut limited = builder(4);
        limited.work.nodes = 19_999_990;
        assert!(
            limited
                .query_many(&surface, None, settings(), &normals, &|| false)
                .is_ok()
        );
        assert!(limited.work.contact_tests + limited.work.distance_tests > 0);
        let mut cancelled_builder = builder(4);
        let polls = Cell::new(0);
        let cancel = || {
            polls.set(polls.get() + 1);
            polls.get() >= 4
        };
        assert!(
            cancelled_builder
                .query_many(&surface, None, settings(), &normals, &cancel)
                .unwrap_err()
                .contains("cancelled")
        );
        assert_eq!(total(cancelled_builder.work) + cancelled_builder.queries, 0);
        let actual = cancelled_builder
            .query_many(&surface, None, settings(), &normals, &|| false)
            .unwrap();
        let expected = builder(1)
            .query_many(&surface, None, settings(), &normals, &|| false)
            .unwrap();
        assert_eq!(actual, expected);
    }

    // Independent linear projection oracle; includes inside queries because a
    // boundary witness there must remain conservative rather than return zero.
    fn linear_distance(p: Point, ring: &[Point]) -> f64 {
        (0..ring.len())
            .map(|i| {
                let a = ring[i];
                let b = ring[(i + 1) % ring.len()];
                let length = (b[0] - a[0]).hypot(b[1] - a[1]);
                if length == 0. {
                    return (p[0] - a[0]).hypot(p[1] - a[1]);
                }
                let u = [(b[0] - a[0]) / length, (b[1] - a[1]) / length];
                let along = ((p[0] - a[0]) * u[0] + (p[1] - a[1]) * u[1]).clamp(0., length);
                (p[0] - a[0] - along * u[0]).hypot(p[1] - a[1] - along * u[1])
            })
            .fold(f64::INFINITY, f64::min)
    }

    #[test]
    fn indexed_boundary_witness_matches_linear_oracle() {
        let mut rings = vec![
            vec![[2., 3.]],
            vec![[0., 0.], [3., 1.]],
            vec![[0., 0.], [3., 0.], [1., 2.]],
            vec![[0., 0.], [4., 0.], [4., 2.], [0., 2.]],
        ];
        for n in [32, 128] {
            for (scale, shift) in [([4., 3.], [0., 0.]), ([10., 0.001], [10000., -3000.])] {
                rings.push(
                    (0..n)
                        .map(|i| {
                            let a = i as f64 * std::f64::consts::TAU / n as f64;
                            [shift[0] + scale[0] * a.cos(), shift[1] + scale[1] * a.sin()]
                        })
                        .collect(),
                );
            }
        }
        for ring in rings {
            let index = InnerHull::new(ring.clone());
            for offset in [[0., 0.], [1e-9, -1e-9], [0.3, 0.2], [2., 4.], [100., -50.]] {
                for base in &ring {
                    let p = [base[0] + offset[0], base[1] + offset[1]];
                    let q = index.nearest(p, &mut 0, &mut 0).unwrap();
                    let expected = linear_distance(p, &ring);
                    assert!(((p[0] - q[0]).hypot(p[1] - q[1]) - expected).abs() < 1e-10);
                }
            }
        }
    }

    #[test]
    fn index_reduces_edge_work_and_preserves_limit() {
        let points = (0..128)
            .map(|i| {
                let a = i as f64 * std::f64::consts::TAU / 128.;
                [a.cos(), a.sin()]
            })
            .collect();
        let index = InnerHull::new(points);
        let (mut edges, mut nodes) = (0, 0);
        for i in 0..128 {
            let a = i as f64 * std::f64::consts::TAU / 128.;
            index
                .nearest([2. * a.cos(), 2. * a.sin()], &mut edges, &mut nodes)
                .unwrap();
        }
        assert!(edges < 128 * 128 / 4, "{edges}");
        assert!(edges + nodes < 128 * 128);
        // Work is counted, never capped.
        assert!(index.nearest([2., 0.], &mut 20_000_000, &mut 0).is_ok());
    }
}
