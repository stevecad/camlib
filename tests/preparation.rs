use camlib::{
    motion::WorkFrame,
    preparation::*,
    stock::{InitialStock, StockSpec},
    surface::SurfaceMesh,
};
fn mesh() -> SurfaceMesh {
    SurfaceMesh {
        version: 1,
        units: "mm".into(),
        triangles: vec![
            [[0.2, 0.2, 1.], [2.8, 0.2, 2.], [2.8, 2.8, 2.]],
            [[0.2, 0.2, 1.], [2.8, 2.8, 2.], [0.2, 2.8, 1.]],
        ]
        .into(),
        surface_error_mm: 0.,
        source: serde_json::Value::Null,
    }
}
fn request() -> BlankRequest {
    BlankRequest {
        surface: mesh(),
        stock: StockSpec {
            version: 1,
            units: "mm".into(),
            min_mm: [0.; 3],
            max_mm: [3.; 3],
            cell_size_mm: 1.1,
            max_cells: 100,
        },
        declaration: InitialStock {
            source_id: "scan".into(),
            revision: "1".into(),
            source_sha256: "a".repeat(64),
            source_to_setup: WorkFrame::identity(),
            uncertainty_mm: 0.,
            representation: "whole_cell_top_down".into(),
            bounds: vec![],
        },
        source_enclosure: "top_down_outer".into(),
        tolerance_mm: 1e-6,
        max_work: 100000,
    }
}
#[test]
fn blank_encloses_whole_cells_not_centre_samples() {
    let r = request();
    let result = enclose_blank(&r, &|| false).unwrap();
    assert_eq!(result.bounds.len(), 9);
    let centre = &result.bounds[4];
    assert!(centre.lower_mm <= 1. + (1. - 0.2) / 2.6);
    assert!(centre.upper_mm >= 1. + (2. - 0.2) / 2.6);
    assert!(centre.upper_mm - centre.lower_mm > 0.38);
}
#[test]
fn blank_error_is_outer_and_refuses_clipping_or_partial_work() {
    let mut r = request();
    r.declaration.uncertainty_mm = 0.1;
    let b = enclose_blank(&r, &|| false).unwrap();
    assert!(b.bounds.iter().all(|v| v.lower_mm == 0.));
    r.declaration.uncertainty_mm = 0.3;
    assert!(
        enclose_blank(&r, &|| false)
            .unwrap_err()
            .contains("outside")
    );
    r = request();
    r.max_work = 1;
    assert!(
        enclose_blank(&r, &|| false)
            .unwrap_err()
            .contains("work limit")
    );
    assert!(
        enclose_blank(&request(), &|| true)
            .unwrap_err()
            .contains("cancelled")
    );
}
fn reach_request() -> ReachRequest {
    let manifest: camlib::stream::StreamManifest =
        serde_json::from_str(include_str!("../fixtures/operations/helical-pocket.json")).unwrap();
    let mut surface = mesh();
    surface.triangles = vec![
        [[-20., -20., 1.], [20., -20., 1.], [20., 20., 1.]],
        [[-20., -20., 1.], [20., 20., 1.], [-20., 20., 1.]],
    ]
    .into();
    ReachRequest {
        surface,
        base_z_mm: 0.,
        samples: vec![ReachSample {
            id: "p".into(),
            point_mm: [0., 0., 1.],
            normal: [0., 0., 1.],
        }],
        tools: vec![ReachTool {
            family: "mill".into(),
            assembly: manifest.tools[0].clone(),
        }],
        scene: camlib::verification::Scene {
            version: 1,
            units: "mm".into(),
            id: "fixtures".into(),
            revision: "1".into(),
            work_to_machine: WorkFrame::identity(),
            bodies: vec![],
            machine: Some(camlib::verification::MachineLimits {
                min_tip_mm: Some([-100.; 3]),
                max_tip_mm: Some([100.; 3]),
                max_feed_mm_min: 10000.,
                max_normal_accel_mm_s2: None,
                max_axis_feed_mm_min: None,
                min_gauge_mm: None,
                max_gauge_mm: None,
            }),
        },
        clearance_z_mm: 30.,
        tolerance_mm: 0.02,
        max_pairs: 10,
        max_work: 100000,
        scene_options: Default::default(),
    }
}
#[test]
fn reach_uses_real_shapes_and_deterministic_tested_projection() {
    for shape in [
        camlib::tool::CutterShape::Flat,
        camlib::tool::CutterShape::Ball,
        camlib::tool::CutterShape::Bull {
            corner_radius_mm: 0.5,
        },
    ] {
        let mut r = reach_request();
        r.tools[0].assembly.cutter.shape = shape;
        let report = reach(&r, &|| false).unwrap();
        assert!(report.complete);
        assert_eq!(report.rows[0].status, "clear", "{:?}", report);
        assert_eq!(report.selected.len(), 1);
    }
}
#[test]
fn reach_never_selects_incomplete_or_unwitnessed_results() {
    let mut r = reach_request();
    r.max_work = 1;
    let report = reach(&r, &|| false).unwrap();
    assert!(!report.complete);
    assert!(report.selected.is_empty());
    assert_eq!(report.rows.len(), 1);
    r = reach_request();
    r.samples[0].point_mm[2] = 0.5;
    let report = reach(&r, &|| false).unwrap();
    assert!(report.selected.is_empty());
    assert!(
        report.rows[0]
            .reasons
            .contains(&"source_point_not_witnessed".into())
    );
    r.samples[0].normal = [0., 0., -1.];
    assert!(reach(&r, &|| false).is_err());
}

#[test]
fn reach_validates_native_mesh_metadata_facets_and_base() {
    for (name, edit) in [
        (
            "version",
            (|r: &mut ReachRequest| r.surface.version = 2) as fn(&mut ReachRequest),
        ),
        ("units", |r: &mut ReachRequest| {
            r.surface.units = "inch".into()
        }),
        ("negative error", |r: &mut ReachRequest| {
            r.surface.surface_error_mm = -0.01
        }),
        ("NaN error", |r: &mut ReachRequest| {
            r.surface.surface_error_mm = f64::NAN
        }),
        ("infinite error", |r: &mut ReachRequest| {
            r.surface.surface_error_mm = f64::INFINITY
        }),
        ("excessive error", |r: &mut ReachRequest| {
            r.surface.surface_error_mm = 0.11
        }),
        ("NaN base", |r: &mut ReachRequest| r.base_z_mm = f64::NAN),
        ("infinite base", |r: &mut ReachRequest| {
            r.base_z_mm = f64::INFINITY
        }),
        ("unbounded base", |r: &mut ReachRequest| {
            r.base_z_mm = -1_000_001.
        }),
        ("below base", |r: &mut ReachRequest| r.base_z_mm = 2.),
        ("NaN vertex", |r: &mut ReachRequest| {
            std::sync::Arc::make_mut(&mut r.surface.triangles)[0][0][0] = f64::NAN;
        }),
        ("unbounded vertex", |r: &mut ReachRequest| {
            std::sync::Arc::make_mut(&mut r.surface.triangles)[0][0][0] = 1_000_001.;
        }),
        ("degenerate facet", |r: &mut ReachRequest| {
            std::sync::Arc::make_mut(&mut r.surface.triangles)[0] = [[0., 0., 1.]; 3];
        }),
        ("empty mesh", |r: &mut ReachRequest| {
            r.surface.triangles = vec![].into()
        }),
    ] {
        let mut request = reach_request();
        edit(&mut request);
        assert!(reach(&request, &|| false).is_err(), "accepted {name}");
    }
}

#[test]
fn reach_matches_independent_inclined_plane_contacts_for_all_supported_ends() {
    // Plane z = 20 + 0.75*x has upward normal (-0.6, 0, 0.8).
    // Analytic support of a flat/rounded disk gives required tip height:
    // plane(axis) + slope*(r-rc) + rc*(sqrt(1+slope^2)-1).
    for (shape, rc) in [
        (camlib::tool::CutterShape::Flat, 0.),
        (camlib::tool::CutterShape::Ball, 2.),
        (
            camlib::tool::CutterShape::Bull {
                corner_radius_mm: 0.5,
            },
            0.5,
        ),
    ] {
        let mut request = reach_request();
        request.surface.triangles = vec![
            [[-20., -20., 5.], [20., -20., 35.], [20., 20., 35.]],
            [[-20., -20., 5.], [20., 20., 35.], [-20., 20., 5.]],
        ]
        .into();
        request.samples[0].point_mm = [0., 0., 20.];
        request.samples[0].normal = [-0.6, 0., 0.8];
        request.tools[0].assembly.cutter.shape = shape;
        let report = reach(&request, &|| false).unwrap();
        assert!(report.complete);
        assert_eq!(report.rows[0].status, "clear", "{report:?}");
        let tip = report.rows[0].tip_mm.unwrap();
        let exact = 20. + 0.75 * tip[0] + 0.75 * (2. - rc) + rc * 0.25;
        assert!((tip[2] - exact).abs() < 1e-12, "{tip:?} {exact}");
        assert!((tip[0] - (-(2. - rc) - 0.6 * rc)).abs() < 1e-12);
        assert!((tip[2] - (20. - 0.2 * rc)).abs() < 1e-12);
    }
}

fn fixture(x: [f64; 2], z: [f64; 2]) -> camlib::verification::Prism {
    use camlib::planar::{Boundary, Component, PlanarRegion};
    camlib::verification::Prism {
        id: "clamp".into(),
        role: camlib::verification::BodyRole::Fixture,
        footprint: PlanarRegion {
            version: 1,
            units: "mm".into(),
            name: "clamp".into(),
            boundary_error_mm: 0.,
            components: vec![Component {
                outer: Boundary {
                    vertices: vec![
                        [x[0], -0.5, 0.],
                        [x[1], -0.5, 0.],
                        [x[1], 0.5, 0.],
                        [x[0], 0.5, 0.],
                    ],
                },
                holes: vec![],
            }],
            source: serde_json::Value::Null,
        },
        z_min_mm: z[0],
        z_max_mm: z[1],
        z_error_mm: 0.,
        representation: Default::default(),
    }
}

#[test]
fn reach_rejects_fixture_and_holder_only_approach_obstruction() {
    for (x, z, component) in [
        ([-0.5, 0.5], [5., 6.], "Cutter"),
        ([3., 3.5], [33., 34.], "Holder"),
    ] {
        let mut request = reach_request();
        request.scene.bodies.push(fixture(x, z));
        let report = reach(&request, &|| false).unwrap();
        assert!(report.complete);
        assert_eq!(report.rows[0].status, "obstructed", "{report:?}");
        assert!(report.selected.is_empty());
        assert!(
            report.rows[0]
                .reasons
                .iter()
                .any(|reason| reason.starts_with("approach_violation:")
                    && reason.contains(component)),
            "{report:?}"
        );
    }
}

#[test]
fn reach_selects_smallest_clear_calibrated_projection_and_breaks_ties_by_id() {
    let mut request = reach_request();
    let original = request.tools[0].clone();
    request.tools = [
        ("z-long", 40.),
        ("b-short", 25.),
        ("a-short", 25.),
        ("short-blocked", 20.),
    ]
    .into_iter()
    .map(|(id, stickout)| {
        let mut tool = original.clone();
        tool.assembly.id = id.into();
        tool.assembly.stickout_mm = stickout;
        tool.assembly.holders[0].z_min_mm = stickout;
        tool
    })
    .collect();
    // At the final tip, the shortest holder reaches this fixture; its cutter
    // and shank are too narrow. Both 25 mm calibrated variants remain clear.
    request.scene.bodies.push(fixture([3., 3.5], [21.5, 22.]));
    let report = reach(&request, &|| false).unwrap();
    assert!(report.complete);
    assert_eq!(report.rows.len(), 4);
    assert_eq!(report.rows[0].tool_id, "short-blocked");
    assert_eq!(report.rows[0].status, "obstructed", "{report:?}");
    assert_eq!(report.selected.len(), 1);
    assert_eq!(report.selected[0].tool_id, "a-short");
    assert_eq!(report.rows[1].status, "clear", "{report:?}");
    assert_eq!(report.rows[2].status, "clear", "{report:?}");
}

#[test]
fn reach_index_setup_is_bounded_before_overlapping_facet_allocations() {
    use std::cell::Cell;
    let mut request = reach_request();
    let count = 4096;
    request.surface.triangles =
        vec![[[-1e6, -1e6, 1.], [1e6, -1e6, 1.], [1e6, 1e6, 1.]]; count].into();
    request.max_work = 1_000_000_000;
    let polls = Cell::new(0usize);
    let error = camlib::resources::with_memory_budget(1_048_576, || {
        reach(&request, &|| {
            polls.set(polls.get() + 1);
            false
        })
    })
    .unwrap_err();
    assert!(error.contains("triangle index needs"), "{error}");
    // Validation plus an entry-count pass; no traversal of ~17 million entries.
    assert!(polls.get() < 2 * count);
    request.max_work = 3 * count as u64;
    let report = reach(&request, &|| false).unwrap();
    assert!(!report.complete);
    assert_eq!(report.rows.len(), 1);
    assert_eq!(report.rows[0].status, "unresolved");
    assert!(report.selected.is_empty());
}

#[test]
fn reach_cancellation_inside_approach_keeps_started_and_unvisited_pair_rows() {
    use std::cell::Cell;
    let mut request = reach_request();
    let polls = Cell::new(0);
    reach(&request, &|| {
        polls.set(polls.get() + 1);
        false
    })
    .unwrap();
    // The final poll in a complete pair is inside scene.verify_motion. Using
    // the observed count avoids depending on index implementation poll counts.
    let cancel_at = polls.get();
    let mut second = request.samples[0].clone();
    second.id = "q".into();
    request.samples.push(second);
    polls.set(0);
    let report = reach(&request, &|| {
        polls.set(polls.get() + 1);
        polls.get() >= cancel_at
    })
    .unwrap();
    assert!(!report.complete);
    assert_eq!(report.rows.len(), 2);
    assert!(report.rows[0].tip_mm.is_some());
    assert!(report.rows[1].tip_mm.is_none());
    assert!(
        report.rows.iter().all(|row| row.status == "unresolved"
            && row.reasons.contains(&"cancelled_or_work_limit".into()))
    );
    assert!(report.selected.is_empty());
}

#[test]
fn reach_work_budget_covers_curved_preparation_and_pair_queries_together() {
    let mut request = reach_request();
    request.tools[0].assembly.cutter.shape = camlib::tool::CutterShape::Ball;
    let mut second = request.tools[0].clone();
    second.assembly.id = "t2".into();
    request.tools.push(second);
    request.max_work = 6000;
    let partial = reach(&request, &|| false).unwrap();
    assert!(!partial.complete);
    assert_eq!(partial.rows.len(), 2);
    assert!(partial.selected.is_empty());
    request.max_work = 10000;
    let complete = reach(&request, &|| false).unwrap();
    assert!(complete.complete);
    assert!(complete.rows.iter().all(|row| row.status == "clear"));
}

fn large_blank_request(count: usize) -> BlankRequest {
    let mut request = request();
    request.surface.triangles = vec![request.surface.triangles[0]; count].into();
    request.max_work = 1_000_000_000;
    request
}

#[test]
fn blank_tiny_work_refuses_before_domain_scan_or_surface_preparation() {
    use std::cell::Cell;
    let mut request = large_blank_request(4096);
    // A late invalid facet must not be examined when the initial scan cannot fit.
    std::sync::Arc::make_mut(&mut request.surface.triangles)[4095][0][0] = f64::NAN;
    request.max_work = 1;
    let polls = Cell::new(0usize);
    let error = enclose_blank(&request, &|| {
        polls.set(polls.get() + 1);
        false
    })
    .unwrap_err();
    assert!(error.contains("work limit"), "{error}");
    assert_eq!(
        polls.get(),
        0,
        "entered an unbudgeted facet/preparation loop"
    );
}

#[test]
fn surface_preparation_guards_triangle_and_bvh_allocation_for_both_apis() {
    let request = large_blank_request(4096);
    for bounded in [false, true] {
        let error = camlib::resources::with_memory_budget(1024, || {
            if bounded {
                request
                    .surface
                    .prepare_bounded(0., 1e-6, &mut 0, 1_000_000_000, &|| false)
            } else {
                request.surface.prepare(0., 1e-6, &|| false)
            }
            .err()
            .unwrap()
        });
        assert!(error.contains("surface preparation needs"), "{error}");
    }
    let error = camlib::resources::with_memory_budget(1024, || enclose_blank(&request, &|| false))
        .unwrap_err();
    assert!(error.contains("surface preparation needs"), "{error}");
}

#[test]
fn bounded_surface_preparation_cancels_inside_bvh_projection_and_sort() {
    use std::cell::Cell;
    let count = 512;
    let request = large_blank_request(count);
    // One reservation poll, n validation polls, one root-node poll, and n
    // bounds polls precede the first projection. Three n vertex projections
    // then precede the first heap-sort comparison.
    for cancel_at in [2 * count + 3, 5 * count + 3] {
        let polls = Cell::new(0usize);
        let mut work = 0;
        let error = request
            .surface
            .prepare_bounded(0., 1e-6, &mut work, 1_000_000_000, &|| {
                polls.set(polls.get() + 1);
                polls.get() >= cancel_at
            })
            .err()
            .unwrap();
        assert!(error.contains("cancelled"), "{error}");
        assert_eq!(
            polls.get(),
            cancel_at,
            "continued preparation after cancellation"
        );
        assert!(work < 1_000_000_000);
    }
}

#[test]
fn blank_preparation_and_material_queries_share_the_same_work_counter() {
    let mut request = request();
    let mut work = request.surface.triangles.len() as u64 + 18; // domain scan + nine allocations/validations
    request
        .surface
        .prepare_bounded(0., 1e-6, &mut work, 100000, &|| false)
        .unwrap();
    request.max_work = work;
    // Exactly enough for preparation leaves no work for even the first cell.
    assert!(
        enclose_blank(&request, &|| false)
            .unwrap_err()
            .contains("work limit")
    );
    request.max_work = work + 100000;
    assert_eq!(enclose_blank(&request, &|| false).unwrap().bounds.len(), 9);
}

#[test]
fn bounded_and_ordinary_surface_preparation_keep_analytic_query_results() {
    use camlib::surface::SurfaceWork;
    let at = |x: f64, y: f64| [x, y, 2. + 0.2 * x + 0.1 * y];
    let mut triangles = Vec::new();
    for y in 0..16 {
        for x in 0..16 {
            let (x, y) = (x as f64, y as f64);
            triangles.push([at(x, y), at(x + 1., y), at(x + 1., y + 1.)]);
            triangles.push([at(x, y), at(x + 1., y + 1.), at(x, y + 1.)]);
        }
    }
    let mesh = SurfaceMesh {
        triangles: triangles.into(),
        ..mesh()
    };
    let ordinary = mesh.prepare(0., 1e-6, &|| false).unwrap();
    let bounded = mesh
        .prepare_bounded(0., 1e-6, &mut 0, 1_000_000_000, &|| false)
        .unwrap();
    assert_eq!(ordinary.prism_count(), bounded.prism_count());
    assert_eq!(ordinary.max_z(), bounded.max_z());
    for xy in [[0.75, 1.25], [3.5, 4.5], [12.25, 11.75]] {
        let exact = at(xy[0], xy[1])[2] + 0.5 * 1.05f64.sqrt();
        for surface in [&ordinary, &bounded] {
            let contact = surface
                .contact(xy, 0.5, &mut SurfaceWork::default(), &|| false)
                .unwrap();
            assert!((contact.center_z - exact).abs() < 1e-12);
        }
    }
    for point in [
        [3.5, 4.5, 10.],
        [3.5, 4.5, 1.],
        [-2., -2., 3.],
        [18., 18., 9.],
    ] {
        let a = ordinary
            .point_distance(point, &mut SurfaceWork::default(), &|| false)
            .unwrap();
        let b = bounded
            .point_distance(point, &mut SurfaceWork::default(), &|| false)
            .unwrap();
        assert!((a.distance_mm - b.distance_mm).abs() < 1e-12);
    }
}

#[test]
fn blank_bounds_buffer_is_guarded_before_scanning_or_preparing_the_source() {
    use std::cell::Cell;
    let mut request = request();
    request.stock.cell_size_mm = 0.001;
    request.stock.max_cells = 10_000_000;
    request.max_work = 1_000_000_000;
    let polls = Cell::new(0usize);
    let error = camlib::resources::with_memory_budget(1_048_576, || {
        enclose_blank(&request, &|| {
            polls.set(polls.get() + 1);
            false
        })
    })
    .unwrap_err();
    assert!(error.contains("blank enclosure bounds needs"), "{error}");
    assert_eq!(polls.get(), 0);
}

#[test]
fn blank_placement_error_cannot_partially_or_fully_cancel_mesh_error() {
    for placement_error in [-0.05, -0.1, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let mut request = request();
        request.surface.surface_error_mm = 0.1;
        request.declaration.uncertainty_mm = placement_error;
        assert!(
            enclose_blank(&request, &|| false).is_err(),
            "accepted {placement_error}"
        );
    }
    let mut request = request();
    request.surface.surface_error_mm = 0.1;
    request.declaration.uncertainty_mm = 0.9;
    request.stock.min_mm = [-2.; 3];
    request.stock.max_mm = [5.; 3];
    let result = enclose_blank(&request, &|| false).unwrap();
    assert_eq!(result.uncertainty_mm, 1.);
    assert_eq!(result.source_id, request.declaration.source_id);
    assert_eq!(result.revision, request.declaration.revision);
    assert_eq!(result.source_sha256, request.declaration.source_sha256);
    request.declaration.uncertainty_mm = 0.91;
    assert!(
        enclose_blank(&request, &|| false)
            .unwrap_err()
            .contains("combined finite error")
    );
}

#[test]
fn blank_metadata_is_validated_before_budget_or_geometry_work() {
    use std::cell::Cell;
    for (name, edit) in [
        (
            "source ID",
            (|r: &mut BlankRequest| r.declaration.source_id = "invalid id".into())
                as fn(&mut BlankRequest),
        ),
        ("oversized ID", |r: &mut BlankRequest| {
            r.declaration.source_id = "s".repeat(1_000_000)
        }),
        ("empty revision", |r: &mut BlankRequest| {
            r.declaration.revision.clear()
        }),
        ("oversized revision", |r: &mut BlankRequest| {
            r.declaration.revision = "r".repeat(1_000_000)
        }),
        ("source hash", |r: &mut BlankRequest| {
            r.declaration.source_sha256 = "invalid".into()
        }),
        ("representation", |r: &mut BlankRequest| {
            r.declaration.representation = "closed_mesh".into()
        }),
    ] {
        let mut request = request();
        edit(&mut request);
        request.max_work = 1;
        let polls = Cell::new(0usize);
        let error = camlib::resources::with_memory_budget(1, || {
            enclose_blank(&request, &|| {
                polls.set(polls.get() + 1);
                false
            })
        })
        .unwrap_err();
        assert!(error.contains("initial stock"), "{name}: {error}");
        assert_eq!(polls.get(), 0, "examined geometry for {name}");
    }
}

#[test]
fn blank_rejects_oversized_supplied_bounds_before_geometry_or_allocation_checks() {
    use std::cell::Cell;
    let mut request = request();
    request.declaration.bounds = vec![
        camlib::stock::HeightBounds {
            lower_mm: f64::NAN,
            upper_mm: f64::NAN,
        };
        100_000
    ];
    request.max_work = 1;
    let polls = Cell::new(0usize);
    let error = camlib::resources::with_memory_budget(1, || {
        enclose_blank(&request, &|| {
            polls.set(polls.get() + 1);
            false
        })
    })
    .unwrap_err();
    assert!(error.contains("empty bounds"), "{error}");
    assert_eq!(polls.get(), 0);
}

#[test]
fn blank_accounts_for_validated_metadata_memory_before_clone_or_geometry() {
    use std::cell::Cell;
    let mut request = request();
    request.declaration.revision = "r".repeat(512);
    let polls = Cell::new(0usize);
    let error = camlib::resources::with_memory_budget(9 * 16, || {
        enclose_blank(&request, &|| {
            polls.set(polls.get() + 1);
            false
        })
    })
    .unwrap_err();
    assert!(error.contains("blank enclosure bounds needs"), "{error}");
    assert_eq!(polls.get(), 0);
}
