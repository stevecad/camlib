use super::*;

fn frustum(z: Interval, inner: Interval, outer: Interval) -> ProfileRegion {
    ProfileRegion::AnnularFrustum {
        z_mm: z,
        inner_radius_mm: inner,
        outer_radius_mm: outer,
    }
}
fn profile(regions: Vec<ProfileRegion>) -> PreparedProfile {
    RotationalProfile { regions }.prepare().unwrap()
}
fn contains(spans: &[Interval], x: f64) -> bool {
    spans.iter().any(|s| s[0] <= x && x <= s[1])
}
fn check(section: &SectionBounds, x: f64, occupied: bool) {
    assert!(
        !contains(&section.certain, x) || occupied,
        "false certain at {x}: {section:?}"
    );
    assert!(
        !occupied || contains(&section.possible, x),
        "false empty at {x}: {section:?}"
    );
    for span in &section.certain {
        assert!(
            section
                .possible
                .iter()
                .any(|p| p[0] <= span[0] && span[1] <= p[1])
        );
    }
}

#[test]
fn straight_insert_has_a_45_degree_shoulder_and_an_uncut_center() {
    let p = profile(vec![frustum([0., 4.], [16., 16.], [20., 24.])]);
    for iz in -4..=72 {
        let z = iz as f64 / 16.;
        let radial = p.radial_intervals_at(z).unwrap();
        for ir in 0..=400 {
            let r = ir as f64 / 16.;
            // Integer half-plane predicates, independent of interpolation.
            let occupied = (0..=64).contains(&iz) && ir >= 256 && ir <= 320 + iz;
            check(&radial, r, occupied);
            check(&p.axial_intervals_at(r).unwrap(), z, occupied);
        }
    }
    assert!(p.axial_intervals_at(0.).unwrap().possible.is_empty());
    assert!(p.axial_intervals_at(15.).unwrap().possible.is_empty());
}

#[test]
fn round_insert_sections_enclose_the_implicit_circle_without_filling_the_center() {
    let p = profile(vec![ProfileRegion::CircularRing {
        center_rz_mm: [10., 5.],
        radius_mm: 5.,
        z_mm: [0., 10.],
    }]);
    // All coordinates have denominator 16: the implicit circle comparison is
    // exact integer arithmetic, including Pythagorean points on its boundary.
    for iz in -8_i64..=168 {
        let z = iz as f64 / 16.;
        let radial = p.radial_intervals_at(z).unwrap();
        for ir in 0_i64..=248 {
            let r = ir as f64 / 16.;
            let occupied = (ir - 160).pow(2) + (iz - 80).pow(2) <= 80 * 80;
            check(&radial, r, occupied);
            check(&p.axial_intervals_at(r).unwrap(), z, occupied);
        }
    }
    let middle = p.radial_intervals_at(5.).unwrap();
    assert!(middle.possible[0][0] > 4.99);
    assert!(middle.certain[0][0] < 5.01);
    assert!(middle.certain[0][1] > 14.99);
}

#[test]
fn axial_clipping_keeps_the_actual_arc_and_separate_rings_stay_separate() {
    let p = profile(vec![
        ProfileRegion::CircularRing {
            center_rz_mm: [10., 5.],
            radius_mm: 5.,
            z_mm: [1., 2.],
        },
        frustum([1., 2.], [20., 20.], [22., 22.]),
    ]);
    assert!(p.radial_intervals_at(0.).unwrap().possible.is_empty());
    let s = p.radial_intervals_at(2.).unwrap();
    assert_eq!(s.possible.len(), 2);
    check(&s, 6., true);
    check(&s, 14., true);
    check(&s, 17., false);
    assert_eq!(p.axial_intervals_at(10.).unwrap().certain, vec![[1., 2.]]);
    assert!(p.axial_intervals_at(16.).unwrap().possible.is_empty());
}

#[test]
fn line_constraints_handle_taper_directions_triangles_and_thin_gaps() {
    for inner in [[2., 2.], [0., 2.], [2., 0.]] {
        for outer in [[4., 4.], [2., 4.], [4., 2.]] {
            let p = profile(vec![frustum([1., 5.], inner, outer)]);
            for iz in 0..=48 {
                for ir in 0..=40 {
                    let (z, r) = (iz as f64 / 8., ir as f64 / 8.);
                    // Clear denominators in the two line inequalities.
                    let low = 32 * inner[0] as i32 + (inner[1] - inner[0]) as i32 * (iz - 8);
                    let high = 32 * outer[0] as i32 + (outer[1] - outer[0]) as i32 * (iz - 8);
                    let occupied = (8..=40).contains(&iz) && 4 * ir >= low && 4 * ir <= high;
                    check(&p.axial_intervals_at(r).unwrap(), z, occupied);
                    check(&p.radial_intervals_at(z).unwrap(), r, occupied);
                }
            }
        }
    }
    let gap = 4_f64.next_up();
    let p = profile(vec![
        frustum([0., 1.], [1.; 2], [4.; 2]),
        frustum([0., 1.], [gap; 2], [5.; 2]),
    ]);
    assert_eq!(p.radial_intervals_at(0.5).unwrap().possible.len(), 2);
}

#[test]
fn groups_with_different_axial_settings_and_body_are_separate() {
    let g = IndexableGeometry {
        inserts: vec![
            MountedInsertGroup {
                id: "main".into(),
                count: 4,
                cutting: RotationalProfile {
                    regions: vec![frustum([0.25, 3.], [8.; 2], [10.; 2])],
                },
            },
            MountedInsertGroup {
                id: "wiper".into(),
                count: 1,
                cutting: RotationalProfile {
                    regions: vec![frustum([0., 1.], [9.; 2], [10.; 2])],
                },
            },
        ],
        noncutting: RotationalProfile {
            regions: vec![frustum([0.5, 8.], [0.; 2], [8.; 2])],
        },
    }
    .prepare()
    .unwrap();
    assert!(g.cutting().radial_intervals_at(0.1).unwrap().possible[0][0] >= 9.);
    assert!(
        g.cutting()
            .axial_intervals_at(0.)
            .unwrap()
            .possible
            .is_empty()
    );
    assert_eq!(
        g.noncutting().axial_intervals_at(0.).unwrap().certain,
        vec![[0.5, 8.]]
    );
    assert_eq!(
        g.definition().inserts.iter().map(|i| i.count).sum::<u32>(),
        5
    );
    let data = serde_json::to_string(g.definition()).unwrap();
    assert_eq!(
        serde_json::from_str::<IndexableGeometry>(&data).unwrap(),
        *g.definition()
    );
}

#[test]
fn overlaps_merge_but_axial_voids_do_not() {
    let p = profile(vec![
        frustum([0., 1.], [2.; 2], [5.; 2]),
        frustum([0., 1.], [4.; 2], [6.; 2]),
        frustum([2., 3.], [2.; 2], [6.; 2]),
    ]);
    assert_eq!(p.radial_intervals_at(0.5).unwrap().certain, vec![[2., 6.]]);
    assert_eq!(
        p.axial_intervals_at(4.5).unwrap().certain,
        vec![[0., 1.], [2., 3.]]
    );
}

#[test]
fn malformed_dimensions_and_queries_are_refused() {
    for region in [
        frustum([0., 0.], [1.; 2], [2.; 2]),
        frustum([0., 1.], [3.; 2], [2.; 2]),
        frustum([0., 1.], [2.; 2], [2.; 2]),
        frustum([0., 1.], [0.; 2], [f64::NAN; 2]),
        frustum([-1., 1.], [0.; 2], [2.; 2]),
        ProfileRegion::CircularRing {
            center_rz_mm: [1., 2.],
            radius_mm: 2.,
            z_mm: [0., 4.],
        },
        ProfileRegion::CircularRing {
            center_rz_mm: [5., 2.],
            radius_mm: 2.,
            z_mm: [4., 5.],
        },
        ProfileRegion::CircularRing {
            center_rz_mm: [5., 2.],
            radius_mm: f64::INFINITY,
            z_mm: [0., 4.],
        },
    ] {
        assert!(
            RotationalProfile {
                regions: vec![region]
            }
            .prepare()
            .is_err()
        );
    }
    assert!(RotationalProfile { regions: vec![] }.prepare().is_err());
    let p = profile(vec![frustum([0., 1.], [0.; 2], [1.; 2])]);
    for x in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(p.radial_intervals_at(x).is_err());
        assert!(p.axial_intervals_at(x).is_err());
    }
    assert!(p.axial_intervals_at(-1.).is_err());
    assert!(p.radial_intervals_at(f64::MAX).unwrap().possible.is_empty());
    assert!(p.axial_intervals_at(f64::MAX).unwrap().possible.is_empty());
    let json = r#"{"regions":[{"kind":"circular_ring","center_rz_mm":[10,5],"radius_mm":5,"z_mm":[0,10],"programming_radius_mm":3}]}"#;
    assert!(serde_json::from_str::<RotationalProfile>(json).is_err());
}

#[test]
fn range_queries_preserve_existential_and_universal_occupancy() {
    for shape in [
        frustum([0., 4.], [2., 3.], [4., 8.]),
        frustum([0., 4.], [3., 2.], [8., 4.]),
        ProfileRegion::CircularRing {
            center_rz_mm: [5., 2.],
            radius_mm: 2.,
            z_mm: [0., 4.],
        },
    ] {
        for lo in 0..32 {
            for width in [0, 1, 4, 12] {
                let radii = [lo as f64 / 4., (lo + width) as f64 / 4.];
                for all in [false, true] {
                    let bounds = shape.axial_band(radii, all);
                    for iz in -1..=17 {
                        let z = iz as f64 / 4.;
                        let truth = match shape {
                            ProfileRegion::AnnularFrustum {
                                inner_radius_mm: i,
                                outer_radius_mm: o,
                                ..
                            } => {
                                let inner = i[0] + (i[1] - i[0]) * z / 4.;
                                let outer = o[0] + (o[1] - o[0]) * z / 4.;
                                (0. ..=4.).contains(&z)
                                    && if all {
                                        radii[0] >= inner && radii[1] <= outer
                                    } else {
                                        radii[1] >= inner && radii[0] <= outer
                                    }
                            }
                            _ => {
                                let dr = if all {
                                    (radii[0] - 5.).abs().max((radii[1] - 5.).abs())
                                } else {
                                    (5. - radii[1]).max(radii[0] - 5.).max(0.)
                                };
                                dr * dr + (z - 2.) * (z - 2.) <= 4.
                            }
                        };
                        check(&bounds, z, truth);
                    }
                }
            }
        }
    }
}

#[test]
fn short_and_long_annular_sweeps_enclose_independent_distance_extrema() {
    let p = profile(vec![frustum([0., 1.], [2.; 2], [4.; 2])]);
    for end in [0., 0.25, 2., 12.] {
        for x in -20..=60 {
            for y in -20..=20 {
                let center = [x as f64 / 4., y as f64 / 4.];
                let bounds = p.sweep_column([0., 0., 3.], [end, 0., 3.], center, [0.125; 2], 1e-8);
                for ox in [-0.125, 0., 0.125] {
                    for oy in [-0.125, 0., 0.125] {
                        let (px, py) = (center[0] + ox, center[1] + oy);
                        let near2 = (px - px.clamp(0., end)).powi(2) + py * py;
                        let far2 = (px * px).max((px - end).powi(2)) + py * py;
                        for z in [2.5, 3.25, 3.75, 4.5] {
                            let truth = (3. ..=4.).contains(&z) && near2 <= 16. && far2 >= 4.;
                            check(&bounds, z, truth);
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn climbing_sweep_must_not_merge_radial_correlations_between_insert_groups() {
    let p = profile(vec![
        frustum([0., 1.], [1.; 2], [1.1; 2]),
        frustum([0.9, 2.], [1.9; 2], [2.; 2]),
    ]);
    let bounds = p.sweep_column([1., 0., 0.], [2., 0., 1.], [0., 0.], [0., 0.], 0.);
    // At the first ring the tip is at Z 0..0.1, so its top is <=1.1.
    // At the second it is at Z 0.9..1, so its bottom is >=1.8.
    // Overlapping axial projections alone would falsely fill this gap.
    assert!(!contains(&bounds.certain, 1.5), "{bounds:?}");
    assert!(contains(&bounds.possible, 1.5));
    check(&bounds, 0.5, true);
    check(&bounds, 2.5, true);
}
