use super::*;
use std::f64::consts::PI;

fn circle(x: f64, y: f64, r: f64) -> Boundary {
    PlanarRegion::circle("circle", [x, y], r)
        .unwrap()
        .components
        .remove(0)
        .outer
}
fn region(outer: Boundary, holes: Vec<Boundary>) -> PlanarRegion {
    PlanarRegion {
        version: 1,
        units: "mm".into(),
        name: "analytic difference".into(),
        boundary_error_mm: 0.,
        source: serde_json::Value::Null,
        components: vec![Component { outer, holes }],
    }
}
fn prepare(r: &PlanarRegion) -> PreparedRegion {
    r.prepare(ConstructionOptions::default()).unwrap()
}
fn old(a: &PreparedRegion, b: &PreparedRegion) -> PreparedConstruction {
    a.integer_boolean_prepared(
        b,
        Boolean::Difference,
        a.definition
            .boundary_error_mm
            .max(b.definition.boundary_error_mm)
            + a.options.tolerance_mm,
    )
    .unwrap()
}
fn fast(a: &PreparedRegion, b: &PreparedRegion) -> PreparedConstruction {
    let result = a.boolean_prepared(b, Boolean::Difference).unwrap();
    assert_eq!(
        result.report.backend,
        "cavalier_contours_0.9.0_sparse_difference"
    );
    assert_eq!(result.report.chord_error_mm, 0.);
    assert_eq!(result.report.quantization_grid_mm, 0.);
    result
}

// Independent test oracle: approximate each analytic arc with a stated sagitta,
// then prove whole line segments lie inside a union of capsules around the
// reference segments. Convexity bounds a segment by BOTH endpoints against ONE
// capsule. Subdivide when the covering reference segment changes. This is not
// point sampling: every accepted interval has a complete segment enclosure.
fn chord_segments(r: &PlanarRegion, sagitta: f64) -> Vec<(Point, Point)> {
    let mut out = vec![];
    for boundary in r
        .components
        .iter()
        .flat_map(|c| std::iter::once(&c.outer).chain(&c.holes))
    {
        for i in 0..boundary.vertices.len() {
            let a = boundary.vertices[i];
            let b = boundary.vertices[(i + 1) % boundary.vertices.len()];
            if a[2] == 0. {
                out.push(([a[0], a[1]], [b[0], b[1]]));
                continue;
            }
            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
            let length = dx.hypot(dy);
            let radius = length * (1. + a[2] * a[2]) / (4. * a[2].abs());
            let scale = (1. - a[2] * a[2]) / (4. * a[2]);
            let center = [
                (a[0] + b[0]) / 2. - dy * scale,
                (a[1] + b[1]) / 2. + dx * scale,
            ];
            let sweep = 4. * a[2].atan();
            let start = (a[1] - center[1]).atan2(a[0] - center[0]);
            let step = 2. * (1. - (sagitta / radius).min(1.)).acos();
            let n = (sweep.abs() / step).ceil().max(1.) as usize;
            let mut previous = [a[0], a[1]];
            for k in 1..=n {
                let t = start + sweep * k as f64 / n as f64;
                let next = if k == n {
                    [b[0], b[1]]
                } else {
                    [center[0] + radius * t.cos(), center[1] + radius * t.sin()]
                };
                out.push((previous, next));
                previous = next;
            }
        }
    }
    out
}
fn distance(p: Point, a: Point, b: Point) -> f64 {
    let u = [b[0] - a[0], b[1] - a[1]];
    let t =
        (((p[0] - a[0]) * u[0] + (p[1] - a[1]) * u[1]) / (u[0] * u[0] + u[1] * u[1])).clamp(0., 1.);
    (p[0] - a[0] - t * u[0]).hypot(p[1] - a[1] - t * u[1])
}
struct Capsules {
    segments: Vec<(Point, Point)>,
    cells: std::collections::HashMap<(i64, i64), Vec<usize>>,
    large: Vec<usize>,
    bound: f64,
}
impl Capsules {
    const CELL: f64 = 0.02;
    fn key(p: Point) -> (i64, i64) {
        (
            (p[0] / Self::CELL).floor() as i64,
            (p[1] / Self::CELL).floor() as i64,
        )
    }
    fn new(segments: Vec<(Point, Point)>, bound: f64) -> Self {
        let mut result = Self {
            segments,
            cells: Default::default(),
            large: vec![],
            bound,
        };
        for (i, &(a, b)) in result.segments.iter().enumerate() {
            let lo = Self::key([a[0].min(b[0]) - bound, a[1].min(b[1]) - bound]);
            let hi = Self::key([a[0].max(b[0]) + bound, a[1].max(b[1]) + bound]);
            if (hi.0 - lo.0 + 1) * (hi.1 - lo.1 + 1) > 100_000 {
                result.large.push(i);
                continue;
            }
            for x in lo.0..=hi.0 {
                for y in lo.1..=hi.1 {
                    result.cells.entry((x, y)).or_default().push(i);
                }
            }
        }
        result
    }
    fn covered(&self, a: Point, b: Point, depth: usize) -> bool {
        let local = self
            .cells
            .get(&Self::key(a))
            .map(Vec::as_slice)
            .unwrap_or_default();
        if local.iter().chain(&self.large).any(|&i| {
            let (u, v) = self.segments[i];
            distance(a, u, v) <= self.bound && distance(b, u, v) <= self.bound
        }) {
            return true;
        }
        if depth == 20 {
            return false;
        }
        let m = [(a[0] + b[0]) / 2., (a[1] + b[1]) / 2.];
        self.covered(a, m, depth + 1) && self.covered(m, b, depth + 1)
    }
}
fn enclosure(a: &PreparedConstruction, b: &PreparedConstruction) {
    let budget = a.report.tolerance_mm;
    let chord = budget / 16.;
    let x = chord_segments(&a.report.region, chord);
    let y = chord_segments(&b.report.region, chord);
    let bound = budget / 2.;
    let xc = Capsules::new(x.clone(), bound);
    let yc = Capsules::new(y.clone(), bound);
    assert!(
        x.iter().all(|&(p, q)| yc.covered(p, q, 0)),
        "analytic outside reference enclosure"
    );
    assert!(
        y.iter().all(|&(p, q)| xc.covered(p, q, 0)),
        "reference outside analytic enclosure"
    );
    // Both directed bounds plus the two independent sagitta budgets remain
    // strictly below the unchanged construction tolerance.
    assert!(bound + 2. * chord < budget);
    let perimeter = x
        .iter()
        .map(|&(p, q)| (p[0] - q[0]).hypot(p[1] - q[1]))
        .sum::<f64>();
    let loops = a.report.after_components + a.report.after_holes;
    let strip = 2. * perimeter * budget + loops as f64 * PI * budget * budget;
    assert!((a.region.summary().area_mm2 - b.region.summary().area_mm2).abs() <= strip);
    assert_eq!(
        (a.report.after_components, a.report.after_holes),
        (b.report.after_components, b.report.after_holes)
    );
}

#[test]
fn concentric_annuli_and_nested_disconnected_components_have_exact_area() {
    for shift in [0., 100_000.] {
        let mut a = region(circle(shift, shift, 10.), vec![circle(shift, shift, 3.)]);
        let mut b = region(circle(shift, shift, 9.), vec![circle(shift, shift, 4.)]);
        a.components.push(Component {
            outer: circle(shift, shift, 1.),
            holes: vec![],
        });
        b.components.push(Component {
            outer: circle(shift, shift, 0.8),
            holes: vec![],
        });
        let (a, b) = (prepare(&a), prepare(&b));
        let r = fast(&a, &b);
        assert!((r.region.summary().area_mm2 - 26.36 * PI).abs() < 1e-8);
        assert_eq!((r.report.after_components, r.report.after_holes), (3, 3));
        if shift == 0. {
            enclosure(&r, &old(&a, &b));
        }
    }
}

fn rounded_rectangle_area(boundary: &Boundary) -> f64 {
    let min = std::array::from_fn::<_, 2, _>(|k| {
        boundary
            .vertices
            .iter()
            .map(|p| p[k])
            .fold(f64::INFINITY, f64::min)
    });
    let max = std::array::from_fn::<_, 2, _>(|k| {
        boundary
            .vertices
            .iter()
            .map(|p| p[k])
            .fold(f64::NEG_INFINITY, f64::max)
    });
    let mut correction = 0.;
    for i in 0..boundary.vertices.len() {
        let a = boundary.vertices[i];
        let b = boundary.vertices[(i + 1) % boundary.vertices.len()];
        if a[2] == 0. {
            assert!((a[0] - b[0]).abs().min((a[1] - b[1]).abs()) < 1e-11);
            continue;
        }
        assert!((a[2].abs() - (PI / 8.).tan()).abs() < 1e-12);
        let r = (a[0] - b[0]).hypot(a[1] - b[1]) / 2f64.sqrt();
        correction += (1. - PI / 4.) * r * r;
    }
    (max[0] - min[0]) * (max[1] - min[1]) - correction
}

#[test]
fn four_offset_layers_keep_arcs_and_match_exact_area_and_chorded_enclosure() {
    let saved: Vec<serde_json::Value> = serde_json::from_str(include_str!(
        "../../../fixtures/planar/residual-offset-layers.json"
    ))
    .unwrap();
    for layer in saved {
        let a: PlanarRegion = serde_json::from_value(layer["desired"].clone()).unwrap();
        let b: PlanarRegion = serde_json::from_value(layer["inflated"].clone()).unwrap();
        let area = |r: &PlanarRegion| {
            r.components
                .iter()
                .map(|c| {
                    rounded_rectangle_area(&c.outer)
                        - c.holes.iter().map(rounded_rectangle_area).sum::<f64>()
                })
                .sum::<f64>()
        };
        // These generated nominal reachable regions are nested inside desired;
        // the independent rectangle/circular-sector formula gives their difference.
        let expected = area(&a) - area(&b);
        let (a, b) = (prepare(&a), prepare(&b));
        let result = fast(&a, &b);
        assert!((result.region.summary().area_mm2 - expected).abs() < 2e-10);
        assert!(result.region.summary().arc_segments > 0);
        enclosure(&result, &old(&a, &b));
    }
}

#[test]
fn unmatched_and_crossing_holes_follow_set_difference() {
    let a = prepare(&region(circle(0., 0., 10.), vec![circle(-1., 0., 2.)]));
    let b = prepare(&region(
        circle(0., 0., 9.),
        vec![circle(1., 0., 2.), circle(5., 0., 0.5)],
    ));
    let result = fast(&a, &b);
    // Equal-radius disks, separation 2: independent lens intersection formula.
    let lens = 8. * (0.5f64).acos() - 3f64.sqrt() * 2.;
    let expected = 19. * PI + 4. * PI - lens + 0.25 * PI;
    assert!((result.region.summary().area_mm2 - expected).abs() < 1e-9);
    enclosure(&result, &old(&a, &b));
}

#[test]
fn ambiguous_component_and_hole_matching_falls_back_without_linear_changes() {
    let a = prepare(&region(
        circle(0., 0., 10.),
        vec![circle(-2., 0., 1.), circle(2., 0., 1.)],
    ));
    let b = prepare(&region(circle(0., 0., 9.), vec![circle(0., 0., 4.)]));
    assert!(analytic(&a, &b, 1e-5).is_none());
    let result = a.boolean_prepared(&b, Boolean::Difference).unwrap();
    assert_eq!(result.report.backend, "clipper2_rust_1.2.0_chorded");
    let a = prepare(&region(circle(0., 0., 10.), vec![circle(6., 0., 3.)]));
    let b = prepare(&region(circle(0., 0., 9.), vec![]));
    assert!(analytic(&a, &b, 1e-5).is_none()); // hole touches the other outer
    let rect = |x| Boundary {
        vertices: vec![
            [x, 0., 0.],
            [x + 10., 0., 0.],
            [x + 10., 10., 0.],
            [x, 10., 0.],
        ],
    };
    let a = prepare(&region(rect(0.), vec![]));
    let b = prepare(&region(rect(5.), vec![]));
    assert_eq!(
        a.boolean(&b, Boolean::Difference).unwrap().backend,
        "clipper2_rust_1.2.0_linear"
    );
}

#[test]
fn separation_guard_covers_complete_arcs_and_declines_exhaustion() {
    let a = boundary_to_polyline(&circle(0., 0., 2.), [0., 0.], 1e-7).unwrap();
    let cross = boundary_to_polyline(&circle(0., 2., 2.), [0., 0.], 1e-7).unwrap();
    let far = boundary_to_polyline(&circle(0., 5., 2.), [0., 0.], 1e-7).unwrap();
    assert!(!separated(&a, &cross, 1e-5, &mut 100_000));
    assert!(separated(&a, &far, 1e-5, &mut 100_000));
    assert!(!separated(&a, &far, 1e-5, &mut 0));
}

#[test]
fn a_hole_clear_of_the_subtracted_outer_stays_a_hole_with_its_arcs() {
    // A region with one hole, less a disk clear of that hole: the hole is
    // kept, the disk becomes a second hole, and both keep their arcs (the
    // chorded fallback gives each hundreds of chords).
    let a = prepare(&region(circle(0., 0., 10.), vec![circle(-5., 0., 2.)]));
    let b = prepare(&region(circle(5., 0., 2.), vec![]));
    let result = fast(&a, &b);
    assert!((result.region.summary().area_mm2 - 92. * PI).abs() < 1e-9);
    assert_eq!(
        (result.report.after_components, result.report.after_holes),
        (1, 2)
    );
    let vertices: usize = result.report.region.components[0]
        .holes
        .iter()
        .map(|h| h.vertices.len())
        .sum();
    assert_eq!(vertices, 4);
    enclosure(&result, &old(&a, &b));
    // A disk that reaches the hole is still left to the general Boolean.
    let b = prepare(&region(circle(-2., 0., 2.), vec![]));
    assert!(analytic(&a, &b, 1e-5).is_none());
}
