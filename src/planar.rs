//! Constructed line/arc regions. Offsets, simple-loop booleans, and guarded
//! sparse-component differences retain arcs. General topology falls back to
//! bounded chord/quantization approximation.
//! Reported error budgets are engineering bounds, not certified interval proofs.
use crate::{Fixture, Point, Verdict, geometry::Region};
use cavalier_contours::{
    core::math::Vector2,
    polyline::*,
    shape_algorithms::{Shape, ShapeOffsetOptions},
};
use clipper2_rust::{ClipType, Clipper64, FillRule, Paths64, Point64};
use serde::{Deserialize, Serialize};
mod components;
mod constructed;
mod difference;
mod envelope;

/// Each vertex is [x_mm, y_mm, bulge_to_next], bulge=tan(signed_sweep/4).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Boundary {
    pub vertices: Vec<[f64; 3]>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Component {
    pub outer: Boundary,
    #[serde(default)]
    pub holes: Vec<Boundary>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanarRegion {
    pub version: u32,
    pub units: String,
    pub name: String,
    pub boundary_error_mm: f64,
    pub components: Vec<Component>,
    #[serde(default)]
    pub source: serde_json::Value,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ConstructionOptions {
    pub tolerance_mm: f64,
}
impl Default for ConstructionOptions {
    fn default() -> Self {
        Self { tolerance_mm: 1e-5 }
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Boolean {
    Union,
    Intersection,
    Difference,
    Xor,
}

#[derive(Debug, Serialize)]
pub struct RegionSummary {
    pub components: usize,
    pub holes: usize,
    pub segments: usize,
    pub arc_segments: usize,
    pub area_mm2: f64,
    pub numerical_guard_mm: f64,
    pub boundary_error_mm: f64,
}

#[derive(Debug, Serialize)]
pub struct RegionQuery {
    pub verdict: Verdict,
    pub signed_clearance_mm: Option<f64>,
    pub uncertainty_mm: f64,
}

#[derive(Debug, Serialize)]
pub struct Construction {
    pub region: PlanarRegion,
    pub backend: String,
    pub tolerance_mm: f64,
    pub before_components: usize,
    pub after_components: usize,
    pub before_holes: usize,
    pub after_holes: usize,
    pub applied_offset_mm: Option<f64>,
    pub requested_clearance_mm: Option<f64>,
    pub chord_error_mm: f64,
    pub quantization_grid_mm: f64,
    pub normalization_error_mm: f64,
    pub normalized_vertices: usize,
    pub operand_sources: Vec<serde_json::Value>,
    pub diagnostics: Vec<String>,
}

/// A kernel result retaining the exact validated region used for its report.
/// Shared access is immutable; consuming parts produces independent owned values.
/// Raw authored geometry still enters through `PlanarRegion::prepare`.
pub struct PreparedConstruction {
    report: Construction,
    region: PreparedRegion,
}
impl PreparedConstruction {
    pub fn report(&self) -> &Construction {
        &self.report
    }
    pub fn region(&self) -> &PreparedRegion {
        &self.region
    }
    pub fn into_parts(self) -> (Construction, PreparedRegion) {
        (self.report, self.region)
    }
}

pub struct PreparedRegion {
    definition: PlanarRegion,
    shape: Shape<f64>,
    origin: Point,
    options: ConstructionOptions,
    epsilon: f64,
    linear_components: Option<Vec<Region>>,
    boundary_owners: Vec<usize>,
}

impl PlanarRegion {
    /// Internal generated geometry only: preserve the cleanup displacement as
    /// source uncertainty before validating. Caller input still uses `prepare`.
    pub(crate) fn prepare_generated(
        mut self,
        options: ConstructionOptions,
    ) -> Result<PreparedRegion, String> {
        let (error, _) = constructed::normalize(&mut self.components, options.tolerance_mm)?;
        self.boundary_error_mm += error;
        self.prepare(options)
    }
    pub fn from_fixture(f: &Fixture) -> Self {
        let boundary = |points: &[Point]| Boundary {
            vertices: points.iter().map(|p| [p[0], p[1], 0.]).collect(),
        };
        Self {
            version: f.version,
            units: f.units.clone(),
            name: f.name.clone(),
            boundary_error_mm: f.boundary_error_mm,
            components: vec![Component {
                outer: boundary(&f.outer),
                holes: f.holes.iter().map(|h| boundary(h)).collect(),
            }],
            source: f.source.clone(),
        }
    }
    pub fn circle(name: &str, center: Point, radius: f64) -> Result<Self, String> {
        if !radius.is_finite()
            || !(1e-6..=1e6).contains(&radius)
            || !center.iter().all(|v| v.is_finite())
        {
            return Err("circle needs finite center and positive radius in [1e-6,1e6] mm".into());
        }
        Ok(Self {
            version: 1,
            units: "mm".into(),
            name: name.into(),
            boundary_error_mm: 0.,
            source: serde_json::Value::Null,
            components: vec![Component {
                outer: Boundary {
                    vertices: vec![
                        [center[0] - radius, center[1], 1.],
                        [center[0] + radius, center[1], 1.],
                    ],
                },
                holes: vec![],
            }],
        })
    }
    pub fn prepare(&self, options: ConstructionOptions) -> Result<PreparedRegion, String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("planar.prepare");
        if self.version != 1
            || self.units != "mm"
            || !self.boundary_error_mm.is_finite()
            || !(0.0..=1.0).contains(&self.boundary_error_mm)
        {
            return Err(
                "planar region needs version 1, mm and finite boundary_error_mm in [0,1]".into(),
            );
        }
        if !options.tolerance_mm.is_finite() || !(1e-7..=0.01).contains(&options.tolerance_mm) {
            return Err("construction tolerance must be finite, in [1e-7,0.01] mm".into());
        }
        let mut min = [f64::INFINITY; 2];
        let mut max = [f64::NEG_INFINITY; 2];
        for b in self
            .components
            .iter()
            .flat_map(|c| std::iter::once(&c.outer).chain(&c.holes))
        {
            for v in &b.vertices {
                if !v.iter().all(|n| n.is_finite())
                    || v[0].abs() > 1e6
                    || v[1].abs() > 1e6
                    || v[2].abs() > 1.
                {
                    return Err("boundary vertices require finite coordinates within ±1e6 mm and bulge within [-1,1]".into());
                }
                for k in 0..2 {
                    min[k] = min[k].min(v[k]);
                    max[k] = max[k].max(v[k]);
                }
            }
        }
        let origin = if self.components.is_empty() {
            [0.; 2]
        } else {
            [(min[0] + max[0]) / 2., (min[1] + max[1]) / 2.]
        };
        let magnitude = min
            .into_iter()
            .chain(max)
            .map(f64::abs)
            .filter(|x| x.is_finite())
            .fold(1., f64::max);
        let epsilon = options.tolerance_mm / 32.;
        if epsilon < 1e-9_f64.max(256. * f64::EPSILON * magnitude) {
            return Err("requested construction tolerance is below numerical resolution at these coordinates".into());
        }
        let mut plines = Vec::new();
        let mut canonical = self.clone();
        let mut outer_indexes = Vec::new();
        let mut component_ranges = Vec::new();
        let mut owners = Vec::new();
        let mut linear_components = Vec::new();
        let all_linear = self
            .components
            .iter()
            .flat_map(|c| std::iter::once(&c.outer).chain(&c.holes))
            .all(|b| b.vertices.iter().all(|v| v[2] == 0.));
        #[cfg(feature = "profile-planning")]
        let boundary_profile = crate::planning_profile::span("planar.prepare_boundaries");
        for (ci, c) in self.components.iter().enumerate() {
            let begin = plines.len();
            outer_indexes.push(begin);
            let component_linear = std::iter::once(&c.outer)
                .chain(&c.holes)
                .all(|b| b.vertices.iter().all(|v| v[2] == 0.));
            for (bi, b) in std::iter::once(&c.outer).chain(&c.holes).enumerate() {
                // A curved sibling must not bypass exact validation of a
                // straight loop. Fully linear components are checked together
                // below, including their hole relationships.
                if !component_linear && b.vertices.iter().all(|v| v[2] == 0.) {
                    Region::prepare(&Fixture {
                        version: 1,
                        units: "mm".into(),
                        name: self.name.clone(),
                        boundary_error_mm: self.boundary_error_mm,
                        outer: b.vertices.iter().map(|v| [v[0], v[1]]).collect(),
                        holes: vec![],
                        source: serde_json::Value::Null,
                        checks: vec![],
                    })
                    .map_err(|e| format!("component {ci}, boundary {bi}: {e}"))?;
                }
                let mut pline = boundary_to_polyline(b, origin, epsilon)
                    .map_err(|e| format!("component {ci}, boundary {bi}: {e}"))?;
                let should_ccw = bi == 0;
                if (pline.area() > 0.) != should_ccw {
                    pline.invert_direction_mut();
                }
                let canonical_boundary = polyline_to_boundary(&pline, origin);
                if bi == 0 {
                    canonical.components[ci].outer = canonical_boundary;
                } else {
                    canonical.components[ci].holes[bi - 1] = canonical_boundary;
                }
                plines.push(pline);
                owners.push(ci);
            }
            component_ranges.push(begin..plines.len());
            // Exact-predicate polygon validation remains independent of the arc
            // construction backend for straight-sided inputs.
            if component_linear {
                let f = Fixture {
                    version: 1,
                    units: "mm".into(),
                    name: self.name.clone(),
                    boundary_error_mm: self.boundary_error_mm,
                    outer: c.outer.vertices.iter().map(|v| [v[0], v[1]]).collect(),
                    holes: c
                        .holes
                        .iter()
                        .map(|b| b.vertices.iter().map(|v| [v[0], v[1]]).collect())
                        .collect(),
                    source: serde_json::Value::Null,
                    checks: vec![],
                };
                let prepared = Region::prepare(&f).map_err(|e| format!("component {ci}: {e}"))?;
                if all_linear {
                    linear_components.push(prepared);
                }
            }
        }
        #[cfg(feature = "profile-planning")]
        drop(boundary_profile);
        #[cfg(feature = "profile-planning")]
        let relations_profile = crate::planning_profile::span("planar.prepare_relations");
        let mut touching_components = std::collections::BTreeSet::new();
        let extents: Vec<_> = plines.iter().map(|p| p.extents().unwrap()).collect();
        for i in 0..plines.len() {
            for j in i + 1..plines.len() {
                let (a, b) = (&extents[i], &extents[j]);
                if a.max_x < b.min_x || b.max_x < a.min_x || a.max_y < b.min_y || b.max_y < a.min_y
                {
                    continue;
                }
                let intersects = plines[i].find_intersects_opt(
                    &plines[j],
                    &FindIntersectsOptions {
                        pos_equal_eps: epsilon,
                        ..Default::default()
                    },
                );
                if owners[i] != owners[j] && !intersects.basic_intersects.is_empty() {
                    touching_components
                        .insert((owners[i].min(owners[j]), owners[i].max(owners[j])));
                }
                // Distinct positive-area components may meet at explicit
                // vertices (e.g. XOR at a cutout corner). Such point contacts
                // do not imply an area-connected machining passage.
                let vertex_contact = owners[i] != owners[j]
                    && intersects.basic_intersects.iter().all(|hit| {
                        let endpoint = |p: &Polyline<f64>, index: usize| {
                            let a = p.at(index);
                            let b = p.at((index + 1) % p.vertex_count());
                            (a.x - hit.point.x).hypot(a.y - hit.point.y) <= epsilon
                                || (b.x - hit.point.x).hypot(b.y - hit.point.y) <= epsilon
                        };
                        endpoint(&plines[i], hit.start_index1)
                            && endpoint(&plines[j], hit.start_index2)
                    });
                if !intersects.overlapping_intersects.is_empty()
                    || !intersects.basic_intersects.is_empty() && !vertex_contact
                {
                    let at = intersects
                        .basic_intersects
                        .first()
                        .map(|hit| hit.point)
                        .or_else(|| intersects.overlapping_intersects.first().map(|o| o.point1))
                        .map(|p| format!(" near ({}, {})", p.x + origin[0], p.y + origin[1]))
                        .unwrap_or_default();
                    return Err(format!(
                        "boundaries {i} and {j} intersect or touch{at}; topology ambiguous at requested tolerance"
                    ));
                }
            }
        }
        for range in &component_ranges {
            let outer = &plines[range.start];
            for i in range.start + 1..range.end {
                let p = plines[i].at(0).pos();
                if outer.winding_number(p) == 0 {
                    return Err("hole lies outside its component outer boundary".into());
                }
                for (j, sibling) in plines
                    .iter()
                    .enumerate()
                    .take(range.end)
                    .skip(range.start + 1)
                {
                    if i != j && sibling.winding_number(p) != 0 {
                        return Err("holes overlap or nest within one component".into());
                    }
                }
            }
        }
        #[cfg(feature = "profile-planning")]
        drop(relations_profile);
        #[cfg(feature = "profile-planning")]
        let _overlap_profile = crate::planning_profile::span("planar.prepare_overlaps");
        for (i, &oi) in outer_indexes.iter().enumerate() {
            for (j, r) in component_ranges.iter().enumerate() {
                if i == j {
                    continue;
                }
                let (a, b) = (&extents[oi], &extents[r.start]);
                if a.max_x < b.min_x || b.max_x < a.min_x || a.max_y < b.min_y || b.max_y < a.min_y
                {
                    continue;
                }
                // With crossings excluded and no point contacts, winding is
                // constant along this connected outer loop. One midpoint away
                // from all boundaries proves its containment relation. Shared
                // vertices can change that relation: inspect EVERY intervening
                // segment in that case, as before. Near-boundary samples also
                // retain the exhaustive fallback rather than choosing a side.
                let disjoint_boundaries = !touching_components.contains(&(i.min(j), i.max(j)));
                for (a, b) in plines[oi].iter_segments() {
                    let p = seg_midpoint(a, b);
                    let winding = plines[r.clone()]
                        .iter()
                        .map(|b| b.winding_number(p))
                        .sum::<i32>();
                    if winding != 0 || disjoint_boundaries {
                        let distance = plines[r.clone()]
                            .iter()
                            .map(|b| b.closest_point(p, epsilon).unwrap().distance)
                            .fold(f64::INFINITY, f64::min);
                        if distance > epsilon {
                            if winding != 0 {
                                return Err(
                                    "filled components overlap or nest outside holes".into()
                                );
                            }
                            break;
                        }
                    }
                }
            }
        }
        // Shape::from_plines preserves input order within its CCW/CW partitions.
        // All validated outers are CCW and holes CW; no loop can be filtered out.
        let boundary_owners = (0..canonical.components.len())
            .chain(
                canonical
                    .components
                    .iter()
                    .enumerate()
                    .flat_map(|(i, c)| std::iter::repeat_n(i, c.holes.len())),
            )
            .collect();
        Ok(PreparedRegion {
            boundary_owners,
            definition: canonical,
            shape: Shape::from_plines(plines),
            origin,
            options,
            epsilon,
            linear_components: all_linear.then_some(linear_components),
        })
    }
}

fn boundary_to_polyline(
    b: &Boundary,
    origin: Point,
    epsilon: f64,
) -> Result<Polyline<f64>, String> {
    if b.vertices.len() < 2 {
        return Err(
            "closed boundary needs at least two arc vertices or three line vertices".into(),
        );
    }
    let mut p = Polyline::new_closed();
    for v in &b.vertices {
        p.add(v[0] - origin[0], v[1] - origin[1], v[2]);
    }
    for (index, (a, b)) in p.iter_segments().enumerate() {
        if (a.x - b.x).hypot(a.y - b.y) < 1e-6_f64.max(8. * epsilon) {
            return Err(format!(
                "boundary has a zero/short edge below construction resolution (omit closing duplicate): ({}, {}) -> ({}, {}), length {}; edge {index}/{}; local neighbors {:?}",
                a.x,
                a.y,
                b.x,
                b.y,
                (a.x - b.x).hypot(a.y - b.y),
                p.vertex_count(),
                boundary_neighbors(&p, index)
            ));
        }
        if a.bulge != 0. {
            if a.bulge_is_zero() {
                return Err("arc bulge is below backend resolution; supply an explicit bounded chord approximation".into());
            }
            let (radius, center) = seg_arc_radius_and_center(a, b);
            if !radius.is_finite()
                || !(1e-6..=1e6).contains(&radius)
                || !center.x.is_finite()
                || !center.y.is_finite()
            {
                return Err("arc radius outside supported [1e-6,1e6] mm range".into());
            }
        }
    }
    if p.vertex_count() == 2 && p.iter_vertexes().all(|v| v.bulge == 0.) {
        return Err("two straight vertices do not enclose area".into());
    }
    let ext = p.extents().ok_or("empty boundary")?;
    if [
        ext.min_x + origin[0],
        ext.max_x + origin[0],
        ext.min_y + origin[1],
        ext.max_y + origin[1],
    ]
    .iter()
    .any(|v| !v.is_finite() || v.abs() > 1e6)
    {
        return Err("boundary arc leaves supported coordinate range".into());
    }
    if p.area().abs() <= epsilon * epsilon * 64. {
        return Err("boundary has zero area or is below construction resolution".into());
    }
    // Straight loops are validated by M0's exact orientation predicates. The
    // arc backend's fuzzy self-intersection scan can misclassify short nearly
    // parallel chords as overlaps; it is not used as a polygon validator.
    if p.iter_vertexes().any(|v| v.bulge != 0.) {
        let mut intersect = None;
        p.visit_self_intersects_opt(
            &mut |i: PlineIntersect<f64>| {
                intersect = Some(i);
                cavalier_contours::core::Control::Break(())
            },
            &PlineSelfIntersectOptions {
                pos_equal_eps: epsilon,
                ..Default::default()
            },
        );
        if let Some(intersect) = intersect {
            let (a, b) = match &intersect {
                PlineIntersect::Basic(i) => (i.start_index1, i.start_index2),
                PlineIntersect::Overlapping(i) => (i.start_index1, i.start_index2),
            };
            return Err(format!(
                "boundary self-intersects or backtracks: {intersect:?}; vertices {}; local neighbors {:?} and {:?}",
                p.vertex_count(),
                boundary_neighbors(&p, a),
                boundary_neighbors(&p, b)
            ));
        }
    }
    Ok(p)
}

fn boundary_neighbors(p: &Polyline<f64>, index: usize) -> [[f64; 3]; 4] {
    std::array::from_fn(|i| {
        let v = p.at((index + p.vertex_count() - 1 + i) % p.vertex_count());
        [v.x, v.y, v.bulge]
    })
}

fn polyline_to_boundary(p: &Polyline<f64>, origin: Point) -> Boundary {
    Boundary {
        vertices: p
            .iter_vertexes()
            .map(|v| [v.x + origin[0], v.y + origin[1], v.bulge])
            .collect(),
    }
}

fn group_loops(
    mut positives: Vec<Polyline<f64>>,
    mut negatives: Vec<Polyline<f64>>,
    origin: Point,
) -> Result<Vec<Component>, String> {
    for p in &mut positives {
        if p.area() < 0. {
            p.invert_direction_mut();
        }
    }
    for p in &mut negatives {
        if p.area() > 0. {
            p.invert_direction_mut();
        }
    }
    let mut components: Vec<_> = positives
        .iter()
        .map(|p| Component {
            outer: polyline_to_boundary(p, origin),
            holes: vec![],
        })
        .collect();
    for hole in negatives {
        let point = hole.at(0).pos();
        let parent = positives
            .iter()
            .enumerate()
            .filter(|(_, p)| p.winding_number(point) != 0)
            .min_by(|(_, a), (_, b)| a.area().abs().total_cmp(&b.area().abs()))
            .map(|(i, _)| i)
            .ok_or("construction produced a hole without an enclosing component")?;
        components[parent]
            .holes
            .push(polyline_to_boundary(&hole, origin));
    }
    Ok(components)
}

impl PreparedRegion {
    pub fn definition(&self) -> &PlanarRegion {
        &self.definition
    }
    pub fn summary(&self) -> RegionSummary {
        let all = self.shape.ccw_plines.iter().chain(&self.shape.cw_plines);
        let (mut area, mut segments, mut arcs) = (0., 0, 0);
        for p in all {
            area += p.polyline.area();
            segments += p.polyline.vertex_count();
            arcs += p.polyline.iter_vertexes().filter(|v| v.bulge != 0.).count();
        }
        RegionSummary {
            components: self.definition.components.len(),
            holes: self.shape.cw_plines.len(),
            segments,
            arc_segments: arcs,
            area_mm2: area,
            numerical_guard_mm: self.epsilon,
            boundary_error_mm: self.definition.boundary_error_mm,
        }
    }
    pub fn query(&self, p: Point) -> Result<RegionQuery, String> {
        if !p.iter().all(|v| v.is_finite() && v.abs() <= 1e6) {
            return Err("query point must be finite, within ±1e6 mm".into());
        }
        let uncertainty = self.definition.boundary_error_mm + self.options.tolerance_mm;
        if self.definition.components.is_empty() {
            return Ok(RegionQuery {
                verdict: Verdict::Blocked,
                signed_clearance_mm: None,
                uncertainty_mm: uncertainty,
            });
        }
        if let Some(components) = &self.linear_components {
            let mut inside = false;
            let mut nearest = f64::INFINITY;
            for c in components {
                let d = c.clearance(p)?;
                inside |= d >= 0.;
                nearest = nearest.min(d.abs());
            }
            let signed = if inside { nearest } else { -nearest };
            return Ok(RegionQuery {
                verdict: if signed > uncertainty {
                    Verdict::Allowed
                } else if signed < -uncertainty {
                    Verdict::Blocked
                } else {
                    Verdict::Uncertain
                },
                signed_clearance_mm: Some(signed),
                uncertainty_mm: uncertainty,
            });
        }
        let p = Vector2::new(p[0] - self.origin[0], p[1] - self.origin[1]);
        let mut winding = 0;
        let mut nearest = f64::INFINITY;
        for b in self.shape.ccw_plines.iter().chain(&self.shape.cw_plines) {
            winding += b.polyline.winding_number(p);
            nearest = nearest.min(
                b.polyline
                    .closest_point(p, self.epsilon)
                    .ok_or("empty prepared boundary")?
                    .distance,
            );
        }
        let signed = if winding != 0 { nearest } else { -nearest };
        let verdict = if signed > uncertainty {
            Verdict::Allowed
        } else if signed < -uncertainty {
            Verdict::Blocked
        } else {
            Verdict::Uncertain
        };
        Ok(RegionQuery {
            verdict,
            signed_clearance_mm: Some(signed),
            uncertainty_mm: uncertainty,
        })
    }
    fn finish(
        &self,
        mut components: Vec<Component>,
        backend: &str,
        error: f64,
        delta: Option<f64>,
        chord: f64,
        grid: f64,
    ) -> Result<PreparedConstruction, String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("planar.finish");
        let (mut normalization_error_mm, mut normalized_vertices) =
            constructed::normalize(&mut components, self.options.tolerance_mm)?;
        let (curve_error, curve_vertices) = constructed::normalize_short_curves(
            &mut components,
            self.options.tolerance_mm - normalization_error_mm,
        )?;
        normalization_error_mm += curve_error;
        normalized_vertices += curve_vertices;
        let region = PlanarRegion {
            version: 1,
            units: "mm".into(),
            name: self.definition.name.clone(),
            components,
            boundary_error_mm: error + normalization_error_mm,
            source: self.definition.source.clone(),
        };
        let checked = region.prepare(self.options).map_err(|e| {
            let prefixes: Vec<_> = region
                .components
                .iter()
                .take(2)
                .flat_map(|c| std::iter::once(&c.outer).chain(&c.holes).take(3))
                .map(|b| &b.vertices[..b.vertices.len().min(64)])
                .collect();
            format!(
                "constructed region failed topology validation: {e}; boundary prefixes {prefixes:?}"
            )
        })?;
        let before = self.summary();
        let after = checked.summary();
        let mut diagnostics=vec!["finite-precision construction with an engineering error budget; not a certified interval result".into()];
        if normalized_vertices > 0 {
            diagnostics.push(format!("collapsed {normalized_vertices} generated vertices through exact duplicate straight edges, chord-shadowed lenses or empty bounded arc/line deformation patches; added boundary error {normalization_error_mm} mm"));
        }
        if before.components != after.components || before.holes != after.holes {
            diagnostics.push(format!(
                "topology changed: {} components/{} holes -> {} components/{} holes",
                before.components, before.holes, after.components, after.holes
            ));
        }
        if after.components == 0 {
            diagnostics.push("empty positive-area result; no closed machining region remains at this offset/tolerance".into());
        }
        let report = Construction {
            region: checked.definition.clone(),
            backend: backend.into(),
            tolerance_mm: self.options.tolerance_mm,
            before_components: before.components,
            after_components: after.components,
            before_holes: before.holes,
            after_holes: after.holes,
            applied_offset_mm: delta,
            requested_clearance_mm: None,
            chord_error_mm: chord,
            quantization_grid_mm: grid,
            normalization_error_mm,
            normalized_vertices,
            operand_sources: vec![self.definition.source.clone()],
            diagnostics,
        };
        Ok(PreparedConstruction {
            report,
            region: checked,
        })
    }
    /// Positive distance erodes filled area; negative distance expands it.
    pub fn offset(&self, delta_mm: f64) -> Result<Construction, String> {
        self.offset_prepared(delta_mm).map(|r| r.report)
    }
    /// Same construction/validation as `offset`, retaining its prepared result.
    pub fn offset_prepared(&self, delta_mm: f64) -> Result<PreparedConstruction, String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("planar.offset");
        if !delta_mm.is_finite() || delta_mm.abs() > 1e6 {
            return Err("offset must be finite, within ±1e6 mm".into());
        }
        if delta_mm == 0. || self.definition.components.is_empty() {
            return self.finish(
                self.definition.components.clone(),
                "identity",
                self.definition.boundary_error_mm,
                Some(delta_mm),
                0.,
                0.,
            );
        }
        let options = ShapeOffsetOptions {
            pos_equal_eps: self.epsilon,
            offset_dist_eps: 4. * self.epsilon,
            slice_join_eps: 4. * self.epsilon,
        };
        let envelope = (delta_mm > 0.).then(|| self.erosion_envelope()).flatten();
        let (shape, envelope_error, envelope_vertices) = match &envelope {
            Some((prepared, error, removed)) => (&prepared.shape, *error, *removed),
            None => (&self.shape, 0., 0),
        };
        let output = shape.parallel_offset(delta_mm, &options);
        let components = group_loops(
            output.ccw_plines.into_iter().map(|p| p.polyline).collect(),
            output.cw_plines.into_iter().map(|p| p.polyline).collect(),
            self.origin,
        )?;
        let mut result = self.finish(
            components,
            "cavalier_contours_0.9.0",
            self.definition.boundary_error_mm + self.options.tolerance_mm + envelope_error,
            Some(delta_mm),
            0.,
            0.,
        )?;
        if envelope_vertices > 0 {
            result.report.normalization_error_mm += envelope_error;
            result.report.normalized_vertices += envelope_vertices;
            result.report.diagnostics.push(format!("conservatively simplified linear boundaries before erosion (machining space only shrinks); removed {envelope_vertices} intermediate vertices with boundary displacement at most {envelope_error} mm; original clearance source retained"));
        }
        Ok(result)
    }
    /// Extra erosion accounts for source/operation error in the engineering model.
    pub fn cutter_region(&self, radius_mm: f64, allowance_mm: f64) -> Result<Construction, String> {
        if !radius_mm.is_finite()
            || !allowance_mm.is_finite()
            || radius_mm < 0.
            || allowance_mm < 0.
            || radius_mm + allowance_mm > 1e6
        {
            return Err(
                "cutter radius/allowance must be finite, nonnegative and sum to at most 1e6 mm"
                    .into(),
            );
        }
        let requested = radius_mm + allowance_mm;
        let mut result =
            self.offset(requested + self.definition.boundary_error_mm + self.options.tolerance_mm)?;
        result.requested_clearance_mm = Some(requested);
        result.diagnostics.push(
            "applied offset includes source boundary error and construction tolerance".into(),
        );
        Ok(result)
    }
    pub fn boolean(
        &self,
        other: &PreparedRegion,
        operation: Boolean,
    ) -> Result<Construction, String> {
        self.boolean_prepared(other, operation).map(|r| r.report)
    }
    /// Same construction/validation as `boolean`, retaining its prepared result.
    pub fn boolean_prepared(
        &self,
        other: &PreparedRegion,
        operation: Boolean,
    ) -> Result<PreparedConstruction, String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("planar.boolean");
        if self.options.tolerance_mm != other.options.tolerance_mm {
            return Err("boolean operands must use the same construction tolerance".into());
        }
        let error = self
            .definition
            .boundary_error_mm
            .max(other.definition.boundary_error_mm)
            + self.options.tolerance_mm;
        if self.definition.components.is_empty() || other.definition.components.is_empty() {
            let left_empty = self.definition.components.is_empty();
            let components = match operation {
                Boolean::Union | Boolean::Xor => {
                    if left_empty {
                        other.definition.components.clone()
                    } else {
                        self.definition.components.clone()
                    }
                }
                Boolean::Intersection => Vec::new(),
                Boolean::Difference => {
                    if left_empty {
                        Vec::new()
                    } else {
                        self.definition.components.clone()
                    }
                }
            };
            let mut output = self.finish(components, "set_identity", error, None, 0., 0.)?;
            output
                .report
                .operand_sources
                .push(other.definition.source.clone());
            return Ok(output);
        }
        let simple = self.definition.components.len() == 1
            && other.definition.components.len() == 1
            && self.definition.components[0].holes.is_empty()
            && other.definition.components[0].holes.is_empty()
            // The arc backend's fuzzy coincident-line stitching can create
            // tiny loops or assert on nearly coincident straight boundaries.
            // Linear inputs use the existing bounded integer kernel; retaining
            // actual arcs is the reason to choose this simple-loop backend.
            && (self.linear_components.is_none() || other.linear_components.is_none());
        if simple {
            let a = &self.shape.ccw_plines[0].polyline;
            let b = boundary_to_polyline(
                &other.definition.components[0].outer,
                self.origin,
                self.epsilon,
            )?;
            let op = match operation {
                Boolean::Union => BooleanOp::Or,
                Boolean::Intersection => BooleanOp::And,
                Boolean::Difference => BooleanOp::Not,
                Boolean::Xor => BooleanOp::Xor,
            };
            let result = a.boolean_opt(
                &b,
                op,
                &PlineBooleanOptions {
                    pos_equal_eps: self.epsilon,
                    ..Default::default()
                },
            );
            if matches!(result.result_info, BooleanResultInfo::InvalidInput) {
                return Err("arc boolean backend rejected operands".into());
            }
            let components = group_loops(
                result.pos_plines.into_iter().map(|p| p.pline).collect(),
                result.neg_plines.into_iter().map(|p| p.pline).collect(),
                self.origin,
            )?;
            let mut output =
                self.finish(components, "cavalier_contours_0.9.0", error, None, 0., 0.)?;
            output
                .report
                .operand_sources
                .push(other.definition.source.clone());
            return Ok(output);
        }
        if matches!(operation, Boolean::Difference)
            && (self.linear_components.is_none() || other.linear_components.is_none())
            && let Some(output) = difference::analytic(self, other, error)
        {
            return Ok(output);
        }
        self.integer_boolean_prepared(other, operation, error)
    }
    fn integer_boolean_prepared(
        &self,
        other: &PreparedRegion,
        operation: Boolean,
        error: f64,
    ) -> Result<PreparedConstruction, String> {
        let all_linear = self.linear_components.is_some() && other.linear_components.is_some();
        let chord_error = if all_linear {
            0.
        } else {
            self.options.tolerance_mm / 8.
        };
        let grid = (self.options.tolerance_mm / 128.).max(1e-8);
        let mut clipper = Clipper64::new();
        clipper.add_subject(&integer_paths(self, self.origin, grid, chord_error)?);
        clipper.add_clip(&integer_paths(other, self.origin, grid, chord_error)?);
        let op = match operation {
            Boolean::Union => ClipType::Union,
            Boolean::Intersection => ClipType::Intersection,
            Boolean::Difference => ClipType::Difference,
            Boolean::Xor => ClipType::Xor,
        };
        let mut closed = Vec::new();
        if !clipper.execute(op, FillRule::NonZero, &mut closed, None) {
            return Err("integer boolean backend failed".into());
        }
        let mut output = self.integer_result(
            closed,
            if all_linear {
                "clipper2_rust_1.2.0_linear"
            } else {
                "clipper2_rust_1.2.0_chorded"
            },
            error,
            chord_error,
            grid,
        )?;
        output
            .report
            .operand_sources
            .push(other.definition.source.clone());
        Ok(output)
    }
    fn integer_result(
        &self,
        closed: Paths64,
        backend: &str,
        error: f64,
        chord_error: f64,
        grid: f64,
    ) -> Result<PreparedConstruction, String> {
        let (mut positive, mut negative) = (Vec::new(), Vec::new());
        for path in closed {
            let mut p = Polyline::new_closed();
            for v in path {
                p.add(v.x as f64 * grid, v.y as f64 * grid, 0.);
            }
            if p.vertex_count() < 3 || p.area() == 0. {
                return Err("boolean backend returned a collapsed boundary".into());
            }
            if p.area() > 0. {
                positive.push(p);
            } else {
                negative.push(p);
            }
        }
        self.finish(
            group_loops(positive, negative, self.origin)?,
            backend,
            error,
            None,
            chord_error,
            grid,
        )
    }
    /// Generated linear primitives may touch at intermediate stages of a
    /// pairwise union even when their complete union has unambiguous topology.
    /// Execute that complete union before the existing final topology checks.
    pub(crate) fn union_linear(regions: &[Self]) -> Result<Construction, String> {
        Self::union_linear_filling(regions, 0.).map(|(construction, _)| construction)
    }
    /// The complete linear union with every hole smaller than
    /// `fill_holes_below_mm2` filled (left out of the result), so the result
    /// covers more than the exact union, never less. Only for callers whose
    /// region is protection (the union may grow, never shrink): it settles
    /// slivers of uncovered area that the integer backend returns touching
    /// another boundary within the construction tolerance (for example
    /// between facets of a mesh whose faces were tessellated separately).
    /// Returns the construction and the filled holes (count and area).
    pub(crate) fn union_linear_filling(
        regions: &[Self],
        fill_holes_below_mm2: f64,
    ) -> Result<(Construction, FilledHoles), String> {
        if !fill_holes_below_mm2.is_finite() || fill_holes_below_mm2 < 0. {
            return Err("hole fill area must be finite and nonnegative".into());
        }
        let first = regions.first().ok_or("linear union needs an operand")?;
        let options = first.options;
        if regions.iter().any(|r| {
            r.options.tolerance_mm != options.tolerance_mm || r.linear_components.is_none()
        }) {
            return Err("batch union needs linear regions with identical tolerance".into());
        }
        let grid = (options.tolerance_mm / 128.).max(1e-8);
        let mut clipper = Clipper64::new();
        for r in regions {
            clipper.add_subject(&integer_paths(r, first.origin, grid, 0.)?);
        }
        let mut closed = vec![];
        if !clipper.execute(ClipType::Union, FillRule::NonZero, &mut closed, None) {
            return Err("integer batch union backend failed".into());
        }
        let filled = fill_small_holes(&mut closed, grid, fill_holes_below_mm2);
        let error = regions
            .iter()
            .map(|r| r.definition.boundary_error_mm)
            .fold(0f64, f64::max)
            + options.tolerance_mm;
        first
            .integer_result(closed, "clipper2_rust_1.2.0_linear_batch", error, 0., grid)
            .map(|r| (r.report, filled))
    }
}

impl PreparedRegion {
    /// This linear region grown by `delta_mm` with square joins (each convex
    /// corner cut square at `delta_mm` from it, tangent to the circle of
    /// that radius; almost straight corners mitred; concave ones exact): it
    /// contains the region grown by a disc of `delta_mm`, with straight edges
    /// only. Holes smaller than `fill_holes_below_mm2` are filled. For
    /// protection regions (it only grows).
    pub(crate) fn inflate_square(
        &self,
        delta_mm: f64,
        fill_holes_below_mm2: f64,
    ) -> Result<(Construction, FilledHoles), String> {
        if self.linear_components.is_none() {
            return Err("square inflation needs a linear region".into());
        }
        if !delta_mm.is_finite() || !(0. ..=1e3).contains(&delta_mm) {
            return Err("square inflation distance must be finite, 0..1000 mm".into());
        }
        let grid = (self.options.tolerance_mm / 128.).max(1e-8);
        let paths = integer_paths(self, self.origin, grid, 0.)?;
        // Integer rounding of the offset vertices: half a grid unit, covered
        // by one more grid unit of distance.
        let mut closed = clipper2_rust::inflate_paths_64(
            &paths,
            delta_mm / grid + 1.,
            clipper2_rust::JoinType::Square,
            clipper2_rust::EndType::Polygon,
            2.,
            0.,
        );
        let filled = fill_small_holes(&mut closed, grid, fill_holes_below_mm2);
        let error = self.definition.boundary_error_mm + self.options.tolerance_mm;
        self.integer_result(
            closed,
            "clipper2_rust_1.2.0_square_inflate",
            error,
            0.,
            grid,
        )
        .map(|r| (r.report, filled))
    }
}

/// Drop the holes (negative paths) smaller than `below_mm2`.
fn fill_small_holes(closed: &mut Paths64, grid: f64, below_mm2: f64) -> FilledHoles {
    let mut filled = FilledHoles::default();
    if below_mm2 > 0. {
        // Exact integer shoelace sum (twice the signed area in grid units);
        // a filled hole only adds area.
        closed.retain(|path| {
            let twice: i128 = (0..path.len())
                .map(|i| {
                    let (a, b) = (path[i], path[(i + 1) % path.len()]);
                    a.x as i128 * b.y as i128 - b.x as i128 * a.y as i128
                })
                .sum();
            let area = twice as f64 * 0.5 * grid * grid;
            if area < 0. && -area < below_mm2 {
                filled.holes += 1;
                filled.area_mm2 += -area;
                false
            } else {
                true
            }
        });
    }
    filled
}

/// Holes a protective union filled (`PreparedRegion::union_linear_filling`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize)]
pub struct FilledHoles {
    pub holes: usize,
    pub area_mm2: f64,
}

/// Diagnostic/boolean conversion only; analytical arcs remain in native regions.
fn flatten(p: &Polyline<f64>, error: f64) -> Result<Vec<Point>, String> {
    let mut points = Vec::new();
    for (a, b) in p.iter_segments() {
        points.push([a.x, a.y]);
        if a.bulge != 0. {
            let (radius, center) = seg_arc_radius_and_center(a, b);
            let sweep = 4. * a.bulge.atan();
            let max_angle = 4. * (error / (2. * radius)).min(0.5).sqrt().asin();
            let count = (sweep.abs() / max_angle).ceil().max(1.) as usize;
            if points.len().saturating_add(count) > 1_000_000 {
                return Err(
                    "curve flattening exceeds one million points; use a looser tolerance".into(),
                );
            }
            let start = (a.y - center.y).atan2(a.x - center.x);
            for i in 1..count {
                let angle = start + sweep * i as f64 / count as f64;
                points.push([
                    center.x + radius * angle.cos(),
                    center.y + radius * angle.sin(),
                ]);
            }
        }
    }
    Ok(points)
}

fn integer_paths(
    r: &PreparedRegion,
    origin: Point,
    grid: f64,
    chord_error: f64,
) -> Result<Paths64, String> {
    let mut result = Vec::new();
    for c in &r.definition.components {
        for b in std::iter::once(&c.outer).chain(&c.holes) {
            let polyline = boundary_to_polyline(b, origin, r.epsilon)?;
            let vertices = flatten(&polyline, chord_error)?;
            let mut path = Vec::with_capacity(vertices.len());
            for p in vertices {
                let point =
                    Point64::new((p[0] / grid).round() as i64, (p[1] / grid).round() as i64);
                if path.last() != Some(&point) {
                    path.push(point);
                }
            }
            if path.len() > 1 && path.first() == path.last() {
                path.pop();
            }
            if path.len() < 3 {
                return Err(
                    "boolean quantization collapsed an input boundary; use tighter tolerance"
                        .into(),
                );
            }
            result.push(path);
        }
    }
    Ok(result)
}
