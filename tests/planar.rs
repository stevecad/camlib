use camlib::{
    Fixture, Verdict, fixtures,
    geometry::{Bounds, Region},
    planar::*,
};
use std::f64::consts::PI;

fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Boundary {
    Boundary {
        vertices: vec![[x0, y0, 0.], [x1, y0, 0.], [x1, y1, 0.], [x0, y1, 0.]],
    }
}
fn single(boundary: Boundary) -> PlanarRegion {
    PlanarRegion {
        version: 1,
        units: "mm".into(),
        name: "test".into(),
        boundary_error_mm: 0.,
        components: vec![Component {
            outer: boundary,
            holes: vec![],
        }],
        source: serde_json::json!({"revision":"test-1"}),
    }
}
fn prepared(r: &PlanarRegion) -> PreparedRegion {
    r.prepare(ConstructionOptions::default()).unwrap()
}
fn circle(name: &str, center: [f64; 2], radius: f64) -> PlanarRegion {
    PlanarRegion::circle(name, center, radius).unwrap()
}
fn close(a: f64, b: f64) {
    assert!((a - b).abs() < 1e-6, "{a} != {b}");
}

#[test]
fn offset_half_disk_preserves_the_trimmed_arcs_bulge() {
    let half_disk = single(Boundary {
        vertices: vec![[-117., 0., 1.], [117., 0., 0.], [0., 0., 0.]],
    });
    let result = prepared(&half_disk).offset(48.).unwrap();
    let vertices = &result.region.components[0].outer.vertices;
    let arcs: Vec<_> = vertices.iter().filter(|v| v[2] != 0.).collect();
    assert_eq!(arcs.len(), 1);
    // The circle now has radius 69 and the diameter line moved to y = -48.
    // Its half-chord is sqrt(69² - 48²), and the sagitta is 69 - 48.
    // Their ratio, the bulge, is sqrt(21 / 117).
    let [x, y, bulge] = *arcs[0];
    close(x, -(69_f64.powi(2) - 48_f64.powi(2)).sqrt());
    close(y, -48.);
    assert_eq!(bulge, (21_f64 / 117.).sqrt());
}

#[test]
fn generated_short_boolean_steps_have_bounded_cleanup_without_relaxing_input_resolution() {
    let a = prepared(&single(rect(0., 0., 10., 10.)));
    let shift = 5e-7;
    let b = prepared(&single(rect(5., shift, 15., 10. + shift)));
    let result = a.boolean(&b, Boolean::Union).unwrap();
    assert!(result.normalized_vertices > 0);
    assert!(result.normalization_error_mm > 0. && result.normalization_error_mm <= 1e-5);
    assert!(result.region.boundary_error_mm >= 1e-5 + result.normalization_error_mm);
    let checked = prepared(&result.region);
    assert_eq!(
        (checked.summary().components, checked.summary().holes),
        (1, 0)
    );
    // Independent area/membership of two rectangles, away from the reported
    // boundary uncertainty. No component, hole, or material sliver disappears.
    let exact_area = 150. + 5. * shift;
    assert!((checked.summary().area_mm2 - exact_area).abs() <= 60. * result.normalization_error_mm);
    for p in [
        [1., 1.],
        [14., 1.],
        [7., 9.],
        [-1., 1.],
        [16., 1.],
        [1., 11.],
    ] {
        let inside = (0. ..=10.).contains(&p[0]) && (0. ..=10.).contains(&p[1])
            || (5. ..=15.).contains(&p[0]) && (shift..=10. + shift).contains(&p[1]);
        assert_eq!(
            checked.query(p).unwrap().verdict,
            if inside {
                Verdict::Allowed
            } else {
                Verdict::Blocked
            }
        );
    }
    let mut authored = single(rect(0., 0., 10., 10.));
    authored.components[0]
        .outer
        .vertices
        .insert(1, [shift, 0., 0.]);
    assert!(
        authored
            .prepare(ConstructionOptions::default())
            .err()
            .unwrap()
            .contains("short edge")
    );
}

fn compare_offset_to_independent_polygon(f: &Fixture, offset: f64) {
    let source = Region::prepare(f).unwrap();
    let p = prepared(&PlanarRegion::from_fixture(f));
    let result = p.offset(offset).unwrap();
    let output = prepared(&result.region);
    let mut checked = 0;
    for point in fixtures::probes(source.bounds(), 4000) {
        let margin = source.clearance(point).unwrap() - offset;
        if margin.abs() < 1e-3 {
            continue;
        }
        let expected = if margin > 0. {
            Verdict::Allowed
        } else {
            Verdict::Blocked
        };
        let q = output.query(point).unwrap();
        if q.verdict == Verdict::Uncertain {
            continue;
        }
        assert_eq!(
            q.verdict, expected,
            "offset {offset}, {point:?}, source margin {margin}, {q:?}"
        );
        checked += 1;
    }
    assert!(checked > 3900);
}

#[test]
fn rectangle_erosion_and_rounded_expansion_match_analytic_area() {
    let p = prepared(&single(rect(0., 0., 20., 10.)));
    let inset = p.offset(2.).unwrap();
    let inner = prepared(&inset.region);
    close(inner.summary().area_mm2, 16. * 6.);
    assert_eq!(inner.summary().arc_segments, 0);
    assert_eq!(inner.query([2., 5.]).unwrap().verdict, Verdict::Uncertain);
    assert_eq!(inner.query([10., 5.]).unwrap().verdict, Verdict::Allowed);
    let expanded = p.offset(-2.).unwrap();
    let outer = prepared(&expanded.region);
    close(outer.summary().area_mm2, 200. + 120. + 4. * PI);
    assert_eq!(outer.summary().arc_segments, 4);
}

#[test]
fn circles_and_annuli_retain_arcs_and_have_exact_radius_offsets() {
    let circle = circle("circle", [0., 0.], 10.);
    let result = prepared(&circle).offset(3.).unwrap();
    let q = prepared(&result.region);
    close(q.summary().area_mm2, 49. * PI);
    assert_eq!(q.summary().arc_segments, 2);
    close(q.query([0., 0.]).unwrap().signed_clearance_mm.unwrap(), 7.);
    let mut annulus = circle.clone();
    annulus.components[0].holes.push(
        PlanarRegion::circle("hole", [0., 0.], 3.)
            .unwrap()
            .components
            .remove(0)
            .outer,
    );
    let p = prepared(&annulus);
    let shrunk = p.offset(1.).unwrap();
    close(prepared(&shrunk.region).summary().area_mm2, 65. * PI);
    assert_eq!(shrunk.after_holes, 1);
    let grown = p.offset(-4.).unwrap();
    assert_eq!(grown.after_holes, 0);
    close(prepared(&grown.region).summary().area_mm2, 196. * PI);
    assert_eq!(p.offset(3.6).unwrap().after_components, 0);
}

#[test]
fn islands_and_channel_split_agree_with_independent_clearance_oracle() {
    compare_offset_to_independent_polygon(&fixtures::rectangle_island(1), 3.);
    let f = fixtures::narrow_channel();
    compare_offset_to_independent_polygon(&f, 1.9);
    compare_offset_to_independent_polygon(&f, 2.1);
    let p = prepared(&PlanarRegion::from_fixture(&f));
    assert_eq!(p.offset(1.9).unwrap().after_components, 1);
    assert_eq!(p.offset(1.9999).unwrap().after_components, 1);
    assert_eq!(p.offset(2.0001).unwrap().after_components, 2);
    let split = p.offset(2.1).unwrap();
    assert_eq!(split.after_components, 2);
    assert!(
        split
            .diagnostics
            .iter()
            .any(|d| d.contains("topology changed"))
    );
    let gone = p.offset(11.).unwrap();
    assert_eq!(gone.after_components, 0);
    assert_eq!(
        prepared(&gone.region)
            .query([10., 10.])
            .unwrap()
            .signed_clearance_mm,
        None
    );
    assert!(
        gone.diagnostics
            .iter()
            .any(|d| d.contains("empty positive-area"))
    );
}

#[test]
fn expansion_merges_components_and_erosion_merges_holes() {
    let mut region = single(rect(0., 0., 10., 10.));
    region.components.push(Component {
        outer: rect(12., 0., 22., 10.),
        holes: vec![],
    });
    let result = prepared(&region).offset(-1.1).unwrap();
    assert_eq!(result.after_components, 1);
    assert_eq!(
        prepared(&result.region).query([11., 5.]).unwrap().verdict,
        Verdict::Allowed
    );
    let mut holes = single(rect(0., 0., 30., 20.));
    holes.components[0].holes = vec![rect(7., 7., 10., 13.), rect(12., 7., 15., 13.)];
    let result = prepared(&holes).offset(1.1).unwrap();
    assert_eq!(result.before_holes, 2);
    assert_eq!(result.after_holes, 1);
    assert_eq!(
        prepared(&result.region).query([11., 10.]).unwrap().verdict,
        Verdict::Blocked
    );
}

#[test]
fn all_boolean_operations_match_rectangle_area_identities() {
    let a = prepared(&single(rect(0., 0., 10., 10.)));
    let b = prepared(&single(rect(5., 0., 15., 10.)));
    for (op, area, components) in [
        (Boolean::Union, 150., 1),
        (Boolean::Intersection, 50., 1),
        (Boolean::Difference, 50., 1),
        (Boolean::Xor, 100., 2),
    ] {
        let r = a.boolean(&b, op).unwrap_or_else(|e| panic!("{op:?}: {e}"));
        close(prepared(&r.region).summary().area_mm2, area);
        assert_eq!(r.after_components, components);
        assert_eq!(r.backend, "clipper2_rust_1.2.0_linear");
        assert_eq!(r.operand_sources.len(), 2);
    }
}

#[test]
fn curved_siblings_cannot_hide_a_straight_loop_self_intersection() {
    // Nonzero signed area, so the area guard alone cannot reject this crossing.
    let crossing = Boundary {
        vertices: vec![[0., 0., 0.], [20., 20., 0.], [0., 20., 0.], [10., 0., 0.]],
    };
    let mut region = single(crossing.clone());
    region.components[0].holes = vec![circle("hole", [3., 15.], 0.5).components.remove(0).outer];
    let error = region
        .prepare(ConstructionOptions::default())
        .err()
        .unwrap();
    assert!(error.contains("intersect"), "{error}");

    let mut region = circle("outer", [10., 10.], 30.);
    region.components[0].holes = vec![crossing];
    let error = region
        .prepare(ConstructionOptions::default())
        .err()
        .unwrap();
    assert!(error.contains("intersect"), "{error}");
}

#[test]
fn complex_booleans_preserve_hole_semantics_and_report_quantization() {
    let mut definition = single(rect(0., 0., 10., 10.));
    definition.components[0].holes = vec![rect(4., 4., 6., 6.)];
    let a = prepared(&definition);
    let b = prepared(&single(rect(0., 0., 5., 10.)));
    for (op, expected) in [
        (Boolean::Union, 98.),
        (Boolean::Intersection, 48.),
        (Boolean::Difference, 48.),
        (Boolean::Xor, 50.),
    ] {
        let r = a.boolean(&b, op).unwrap();
        close(prepared(&r.region).summary().area_mm2, expected);
        assert!(r.quantization_grid_mm > 0.);
        assert_eq!(r.backend, "clipper2_rust_1.2.0_linear");
        assert_eq!(r.chord_error_mm, 0.);
    }
}

#[test]
fn curved_boolean_retains_arcs_when_simple_and_bounds_chord_fallback() {
    let circle = prepared(&PlanarRegion::circle("circle", [0., 0.], 10.).unwrap());
    let square = prepared(&single(rect(0., -12., 12., 12.)));
    let half = circle.boolean(&square, Boolean::Intersection).unwrap();
    let q = prepared(&half.region);
    close(q.summary().area_mm2, 50. * PI);
    assert!(q.summary().arc_segments > 0);
    assert_eq!(half.chord_error_mm, 0.);
    let mut ring = PlanarRegion::circle("ring", [0., 0.], 10.).unwrap();
    ring.components[0].holes = vec![
        PlanarRegion::circle("hole", [0., 0.], 3.)
            .unwrap()
            .components
            .remove(0)
            .outer,
    ];
    let result = prepared(&ring)
        .boolean(&square, Boolean::Intersection)
        .unwrap();
    let region = prepared(&result.region);
    assert!(result.chord_error_mm > 0.);
    assert_eq!(region.summary().arc_segments, 0);
    assert!((region.summary().area_mm2 - 45.5 * PI).abs() < 1e-3);
    let mut checked = 0;
    for p in fixtures::probes(
        Bounds {
            min: [-11., -11.],
            max: [11., 11.],
        },
        1000,
    ) {
        let r = p[0].hypot(p[1]);
        if (r - 3.).abs() < 1e-3 || (r - 10.).abs() < 1e-3 || p[0].abs() < 1e-3 {
            continue;
        }
        let expected = if r > 3. && r < 10. && p[0] > 0. {
            Verdict::Allowed
        } else {
            Verdict::Blocked
        };
        assert_eq!(region.query(p).unwrap().verdict, expected, "{p:?}");
        checked += 1;
    }
    assert!(checked > 990);
}

#[test]
fn conservative_cutter_region_accounts_for_input_and_operation_error() {
    let mut definition = single(rect(0., 0., 20., 10.));
    definition.boundary_error_mm = 0.001;
    let r = prepared(&definition).cutter_region(2., 0.2).unwrap();
    close(r.requested_clearance_mm.unwrap(), 2.2);
    close(r.applied_offset_mm.unwrap(), 2.20101);
    assert!(r.region.boundary_error_mm > definition.boundary_error_mm);
    let p = prepared(&r.region);
    assert_eq!(p.query([2.2, 5.]).unwrap().verdict, Verdict::Uncertain);
    assert_eq!(p.query([2.3, 5.]).unwrap().verdict, Verdict::Allowed);
    assert!(prepared(&definition).cutter_region(-1., 0.).is_err());
}

#[test]
fn invalid_or_below_resolution_geometry_does_not_silently_drop_features() {
    for mut r in [
        single(rect(0., 0., 10., 10.)),
        PlanarRegion::circle("circle", [0., 0.], 10.).unwrap(),
    ] {
        r.components[0].outer.vertices[0][2] = 2.;
        assert!(r.prepare(ConstructionOptions::default()).is_err());
    }
    let mut r = single(rect(0., 0., 10., 10.));
    r.components[0].outer.vertices.swap(1, 2);
    assert!(r.prepare(ConstructionOptions::default()).is_err());
    let mut r = single(rect(0., 0., 10., 10.));
    r.components[0].holes.push(rect(9., 9., 11., 11.));
    assert!(r.prepare(ConstructionOptions::default()).is_err());
    let mut r = single(rect(0., 0., 10., 10.));
    r.components[0].outer.vertices.push([0., 0., 0.]);
    assert!(r.prepare(ConstructionOptions::default()).is_err());
    let mut r = single(rect(0., 0., 10., 10.));
    r.components[0].outer.vertices[0][2] = 1e-12;
    assert!(r.prepare(ConstructionOptions::default()).is_err());
    assert!(
        single(rect(0., 0., 10., 10.))
            .prepare(ConstructionOptions { tolerance_mm: 0. })
            .is_err()
    );
}

#[test]
fn winding_translation_and_empty_results_are_explicit() {
    let mut f = fixtures::rectangle_island(1);
    f.outer.reverse();
    for h in &mut f.holes {
        h.reverse();
    }
    for p in f.outer.iter_mut().chain(f.holes.iter_mut().flatten()) {
        p[0] += 900_000.;
        p[1] -= 900_000.;
    }
    let shifted = prepared(&PlanarRegion::from_fixture(&f));
    let r = shifted.offset(3.).unwrap();
    let p = prepared(&r.region);
    assert_eq!(
        p.query([900_010., -899_990.]).unwrap().verdict,
        Verdict::Allowed
    );
    assert_eq!(
        p.query([900_040., -899_970.]).unwrap().verdict,
        Verdict::Blocked
    );
    let zero = shifted.offset(0.).unwrap();
    assert_eq!(zero.backend, "identity");
    let gone = prepared(&single(rect(0., 0., 2., 2.))).offset(2.).unwrap();
    assert!(gone.region.components.is_empty());
    assert!(
        prepared(&gone.region)
            .offset(-1.)
            .unwrap()
            .region
            .components
            .is_empty()
    );
}

#[test]
fn component_inside_another_components_hole_remains_filled() {
    let mut r = single(rect(0., 0., 20., 20.));
    r.components[0].holes.push(rect(4., 4., 16., 16.));
    r.components.push(Component {
        outer: rect(7., 7., 13., 13.),
        holes: vec![],
    });
    let p = prepared(&r);
    close(p.summary().area_mm2, 292.);
    assert_eq!(p.query([10., 10.]).unwrap().verdict, Verdict::Allowed);
    assert_eq!(p.query([5., 5.]).unwrap().verdict, Verdict::Blocked);
}

#[test]
fn cadquery_exported_regions_construct_and_keep_source_provenance() {
    for file in ["cadquery-pocket.json", "cadquery-round-island.json"] {
        let f = camlib::load_fixture(
            &std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("fixtures")
                .join(file),
        )
        .unwrap();
        let p = prepared(&PlanarRegion::from_fixture(&f));
        let r = p.cutter_region(3., 0.).unwrap();
        assert_eq!(r.region.source, f.source);
        assert!(r.after_components > 0);
    }
}

#[test]
fn empty_set_identities_and_point_contacts_are_preserved() {
    let a = prepared(&single(rect(0., 0., 10., 10.)));
    let empty = prepared(&a.offset(20.).unwrap().region);
    for op in [
        Boolean::Union,
        Boolean::Intersection,
        Boolean::Difference,
        Boolean::Xor,
    ] {
        let left = empty.boolean(&a, op).unwrap();
        let right = a.boolean(&empty, op).unwrap();
        let left_area = prepared(&left.region).summary().area_mm2;
        let right_area = prepared(&right.region).summary().area_mm2;
        close(
            left_area,
            if matches!(op, Boolean::Union | Boolean::Xor) {
                100.
            } else {
                0.
            },
        );
        close(
            right_area,
            if matches!(op, Boolean::Intersection) {
                0.
            } else {
                100.
            },
        );
    }
    let mut touch = single(rect(0., 0., 10., 10.));
    touch.components.push(Component {
        outer: rect(10., 10., 20., 20.),
        holes: vec![],
    });
    let p = prepared(&touch);
    assert_eq!(p.query([10., 10.]).unwrap().verdict, Verdict::Uncertain);
    assert_eq!(p.offset(0.1).unwrap().after_components, 2);
    assert_eq!(p.offset(-0.1).unwrap().after_components, 1);
    let mut overlap = single(rect(0., 0., 10., 10.));
    overlap.components.push(Component {
        outer: rect(5., 5., 15., 15.),
        holes: vec![],
    });
    assert!(overlap.prepare(ConstructionOptions::default()).is_err());
    assert!(PlanarRegion::circle("invalid", [0., 0.], -1.).is_err());
}

#[test]
fn rotated_regions_match_the_independent_polygon_oracle() {
    let mut fixture = fixtures::narrow_channel();
    let (sin, cos) = 0.731_f64.sin_cos();
    for p in &mut fixture.outer {
        let [x, y] = *p;
        *p = [x * cos - y * sin + 100., x * sin + y * cos - 20.];
    }
    compare_offset_to_independent_polygon(&fixture, 2.1);
}

#[test]
fn analytic_cadquery_floor_keeps_circle_geometry_through_construction() {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("fixtures/planar/cadquery-round-island.json");
    let definition: PlanarRegion = serde_json::from_slice(&std::fs::read(path).unwrap()).unwrap();
    let p = prepared(&definition);
    assert_eq!(p.summary().segments, 6);
    assert_eq!(p.summary().arc_segments, 2);
    let r = p.cutter_region(3., 0.).unwrap();
    let after = prepared(&r.region);
    let d = r.applied_offset_mm.unwrap();
    close(
        after.summary().area_mm2,
        (40. - 2. * d) * (30. - 2. * d) - (5. + d).powi(2) * PI,
    );
    assert_eq!(after.summary().arc_segments, 2);
    assert_eq!(after.query([40., 30.]).unwrap().verdict, Verdict::Blocked);
    assert_eq!(after.query([25., 20.]).unwrap().verdict, Verdict::Allowed);
}

#[test]
fn prepared_construction_keeps_analytic_annulus_offsets_and_independent_ownership() {
    let mut annulus = circle("annulus", [0., 0.], 10.);
    annulus.components[0]
        .holes
        .push(circle("hole", [0., 0.], 2.).components.remove(0).outer);
    let original = prepared(&annulus);
    for d in [-1., 0., 2.] {
        let product = original.offset_prepared(d).unwrap();
        let expected = PI * ((10. - d).powi(2) - (2. + d).powi(2));
        close(product.region().summary().area_mm2, expected);
        assert_eq!(
            serde_json::to_value(product.report()).unwrap(),
            serde_json::to_value(original.offset(d).unwrap()).unwrap()
        );
        assert_eq!(product.report().region, *product.region().definition());
        for ix in -15..=15 {
            for iy in -15..=15 {
                let p = [ix as f64 * 0.8, iy as f64 * 0.8];
                let r = p[0].hypot(p[1]);
                if (r - (10. - d)).abs() < 0.01 || (r - (2. + d)).abs() < 0.01 {
                    continue;
                }
                let inside = r < 10. - d && r > 2. + d;
                assert_eq!(
                    product.region().query(p).unwrap().verdict,
                    if inside {
                        Verdict::Allowed
                    } else {
                        Verdict::Blocked
                    }
                );
            }
        }
        let (mut raw, retained) = product.into_parts();
        let again = prepared(&raw.region);
        close(retained.summary().area_mm2, again.summary().area_mm2);
        raw.region.version = 99;
        assert!(raw.region.prepare(ConstructionOptions::default()).is_err());
        close(retained.summary().area_mm2, expected);
        assert_eq!(retained.definition().version, 1);
    }
    assert!(original.offset_prepared(f64::NAN).is_err());
    assert!(original.offset_prepared(f64::INFINITY).is_err());
    let wrong = annulus
        .prepare(ConstructionOptions { tolerance_mm: 1e-4 })
        .unwrap();
    assert!(original.boolean_prepared(&wrong, Boolean::Union).is_err());
    let invalid = single(Boundary {
        vertices: vec![[0., 0., 0.], [2., 2., 0.], [0., 2., 0.], [2., 0., 0.]],
    });
    assert!(invalid.prepare(ConstructionOptions::default()).is_err());
}

#[test]
fn prepared_construction_boolean_has_independent_circle_strip_area_and_membership() {
    let mut annulus = circle("annulus", [0., 0.], 10.);
    annulus.components[0]
        .holes
        .push(circle("hole", [0., 0.], 2.).components.remove(0).outer);
    let a = prepared(&annulus);
    let b = prepared(&single(rect(-5., -20., 5., 20.)));
    let intersection = 2. * (5. * 75f64.sqrt() + 100. * 0.5f64.asin()) - 4. * PI;
    let area_a = 96. * PI;
    for op in [
        Boolean::Union,
        Boolean::Intersection,
        Boolean::Difference,
        Boolean::Xor,
    ] {
        let result = a.boolean_prepared(&b, op).unwrap();
        let expected = match op {
            Boolean::Union => area_a + 400. - intersection,
            Boolean::Intersection => intersection,
            Boolean::Difference => area_a - intersection,
            Boolean::Xor => area_a + 400. - 2. * intersection,
        };
        assert!((result.region().summary().area_mm2 - expected).abs() < 0.02);
        let serialized = serde_json::to_value(result.report()).unwrap();
        assert_eq!(
            serialized,
            serde_json::to_value(a.boolean(&b, op).unwrap()).unwrap()
        );
        let raw = prepared(&result.report().region);
        for ix in -24..=24 {
            for iy in -44..=44 {
                let p = [ix as f64 * 0.5, iy as f64 * 0.5];
                let r = p[0].hypot(p[1]);
                if (r - 10.).abs() < 0.01
                    || (r - 2.).abs() < 0.01
                    || (p[0].abs() - 5.).abs() < 0.01
                    || (p[1].abs() - 20.).abs() < 0.01
                {
                    continue;
                }
                let (x, y) = (r < 10. && r > 2., p[0].abs() < 5. && p[1].abs() < 20.);
                let inside = match op {
                    Boolean::Union => x || y,
                    Boolean::Intersection => x && y,
                    Boolean::Difference => x && !y,
                    Boolean::Xor => x ^ y,
                };
                let verdict = result.region().query(p).unwrap().verdict;
                assert_eq!(
                    verdict,
                    if inside {
                        Verdict::Allowed
                    } else {
                        Verdict::Blocked
                    }
                );
                assert_eq!(verdict, raw.query(p).unwrap().verdict);
            }
        }
    }
}

#[test]
fn prepared_residual_pipeline_preserves_legacy_raw_geometry_at_translated_origins() {
    for origin in [[0., 0.], [10000.123, -20000.456]] {
        let mut definition = single(rect(origin[0], origin[1], origin[0] + 24., origin[1] + 16.));
        definition.components[0].holes.push(
            circle("hole", [origin[0] + 9.3, origin[1] + 8.7], 2.1)
                .components
                .remove(0)
                .outer,
        );
        let source = prepared(&definition);
        let centers = prepared(&source.cutter_region(1.5, 0.10016).unwrap().region);
        let expanded = prepared(&centers.offset(-1.5).unwrap().region);
        let desired = prepared(&source.offset(0.1).unwrap().region);
        let legacy = desired.boolean(&expanded, Boolean::Difference).unwrap();
        let expanded = centers.offset_prepared(-1.5).unwrap();
        let desired = source.offset_prepared(0.1).unwrap();
        let retained = desired
            .region()
            .boolean_prepared(expanded.region(), Boolean::Difference)
            .unwrap();
        assert_eq!(
            legacy.region,
            retained.report().region,
            "changed raw residual geometry at{origin:?}"
        );
        assert_eq!(legacy.diagnostics, retained.report().diagnostics);
        assert_eq!(
            legacy.region.boundary_error_mm,
            retained.report().region.boundary_error_mm
        );
        let before = prepared(&legacy.region).summary().area_mm2;
        let after = retained.region().summary().area_mm2;
        assert!((before - after).abs() <= 1e-9 + 1e-12 * before.abs());
    }
}

#[test]
fn component_containment_checks_keep_shared_vertex_and_near_boundary_cases() {
    // These diamonds cross the square at explicit vertices. The first edge is
    // outside the other component; later edges overlap filled material.
    let square = rect(-1., -1., 1., 1.);
    let diamond = Boundary {
        vertices: vec![[1., -1., 0.], [2., 0., 0.], [1., 1., 0.], [0., 0., 0.]],
    };
    for reverse in [false, true] {
        for swap in [false, true] {
            let mut boundaries = [square.clone(), diamond.clone()];
            if reverse {
                for b in &mut boundaries {
                    b.vertices.reverse();
                }
            }
            if swap {
                boundaries.swap(0, 1);
            }
            let mut region = single(boundaries[0].clone());
            region.components.push(Component {
                outer: boundaries[1].clone(),
                holes: vec![],
            });
            assert!(region.prepare(ConstructionOptions::default()).is_err());
        }
    }
    // A component in a hole is allowed. The same boundary in filled material
    // must reject, even when its first midpoint is near the hole boundary.
    for gap in [2e-7, 1e-4, 1.] {
        for shift in [0., 100_000.] {
            let mut region = single(rect(shift, shift, shift + 30., shift + 30.));
            region.components[0]
                .holes
                .push(rect(shift + 4., shift + 4., shift + 26., shift + 26.));
            region.components.push(Component {
                outer: rect(shift + 4. + gap, shift + 4. + gap, shift + 24., shift + 24.),
                holes: vec![],
            });
            let result = region.prepare(ConstructionOptions::default());
            if gap < ConstructionOptions::default().tolerance_mm / 32. {
                // The authored gap is below the existing topology resolution.
                // Preserve explicit ambiguity rejection; do not snap it closed.
                assert!(result.is_err());
            } else {
                assert!(result.is_ok());
            }
            region.components[0].holes.clear();
            assert!(region.prepare(ConstructionOptions::default()).is_err());
        }
    }
}

/// Regression (sliced triangle meshes): a dense polygonal island with
/// sub-micron zigzag. The offset kernel used to return two slivers (~8 mm²)
/// instead of one grown hole (~573 mm²) for an island like this, which let a
/// pocket machine straight through the island. The grown hole must be ONE
/// loop whose area matches Steiner's formula A + P d + pi d² (convex up to the
/// noise), with the declared simplification bound inside its 0.002 mm budget.
#[test]
fn eroding_around_a_noisy_dense_island_keeps_one_correct_hole() {
    let mut seed = 42u64;
    let mut rnd = || {
        seed = seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((seed >> 11) as f64) / ((1u64 << 53) as f64)
    };
    for (n, noise) in [(554usize, 4.3e-4), (1500, 1e-4), (1500, 1e-3), (200, 0.)] {
        let r = 10.2;
        // CW island (hole) around (20, 20) with radial noise, like a mesh slice.
        // Angular jitter and occasional very short edges, as in real slices.
        let mut hole: Vec<[f64; 3]> = Vec::new();
        for i in 0..n {
            let t = -(i as f64) / n as f64 * 2. * PI + rnd() * 0.2 / n as f64;
            let k = r + (rnd() - 0.5) * 2. * noise;
            hole.push([20. + k * t.cos(), 20. + k * t.sin(), 0.]);
            if rnd() < 0.05 {
                let t2 = t - 1e-4 / r;
                hole.push([20. + k * t2.cos(), 20. + k * t2.sin(), 0.]);
            }
        }
        let mut region = single(rect(-25., -25., 65., 65.));
        region.components[0].holes.push(Boundary {
            vertices: hole.clone(),
        });
        let p = prepared(&region);
        let ring = |v: &[[f64; 3]]| {
            let a: f64 = (0..v.len())
                .map(|i| {
                    let (p, q) = (v[i], v[(i + 1) % v.len()]);
                    p[0] * q[1] - q[0] * p[1]
                })
                .sum::<f64>()
                / 2.;
            let l: f64 = (0..v.len())
                .map(|i| {
                    let (p, q) = (v[i], v[(i + 1) % v.len()]);
                    (q[0] - p[0]).hypot(q[1] - p[1])
                })
                .sum();
            (a.abs(), l)
        };
        let (area, perimeter) = ring(&hole);
        for d in [1., 3.3, 6.] {
            let c = p.offset(d).unwrap();
            let holes: Vec<_> = c.region.components.iter().flat_map(|c| &c.holes).collect();
            assert_eq!(
                holes.len(),
                1,
                "n {n} noise {noise} d {d}: {} holes",
                holes.len()
            );
            let expected = area + perimeter * d + PI * d * d;
            let chord_area = ring(&holes[0].vertices).0;
            // Chord polygon of the grown hole (arcs replaced by chords) is
            // slightly smaller than the true hole: allow arc sagitta + noise.
            assert!(
                chord_area <= expected * (1. + 1e-6) + perimeter * 2. * noise
                    && chord_area >= expected * (1. - 2e-3) - perimeter * 2. * noise,
                "n {n} noise {noise} d {d}: area {chord_area} expected {expected}"
            );
            assert!(c.normalization_error_mm <= 0.002 + 1e-9);
        }
    }
}
