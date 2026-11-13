use super::*;

/// The patch at (s, t) in [0, 1]^2, from the Bernstein basis directly.
fn evaluate(patch: &BezierPatch, s: f64, t: f64) -> [f64; 3] {
    let basis = |count: usize, x: f64| -> Vec<f64> {
        let degree = count - 1;
        let mut binomial = 1.0;
        (0..count)
            .map(|k| {
                if k > 0 {
                    binomial *= (degree + 1 - k) as f64 / k as f64;
                }
                binomial * x.powi(k as i32) * (1.0 - x).powi((degree - k) as i32)
            })
            .collect()
    };
    let (bu, bv) = (basis(patch.rows, s), basis(patch.columns, t));
    let mut sum = [0.0; 4];
    for (i, x) in bu.iter().enumerate() {
        for (j, y) in bv.iter().enumerate() {
            for (total, h) in sum.iter_mut().zip(patch.homogeneous[i * patch.columns + j]) {
                *total += x * y * h;
            }
        }
    }
    [sum[0] / sum[3], sum[1] / sum[3], sum[2] / sum[3]]
}

fn norm(a: [f64; 3]) -> f64 {
    a.iter().map(|x| x * x).sum::<f64>().sqrt()
}

/// |Fu|, |Fv|, |Fuu|, |Fuv|, |Fvv| at (s, t) by central differences, for a
/// patch that stands for the spans (su, sv).
fn sampled(patch: &BezierPatch, s: f64, t: f64, su: f64, sv: f64) -> Bounds {
    let h = 1e-4;
    let f = |ds: f64, dt: f64| evaluate(patch, s + ds * h, t + dt * h);
    let combine = |terms: &[(f64, [f64; 3])], scale: f64| {
        norm(std::array::from_fn(|n| {
            terms.iter().map(|(c, p)| c * p[n]).sum::<f64>() / scale
        }))
    };
    [
        combine(&[(1.0, f(1.0, 0.0)), (-1.0, f(-1.0, 0.0))], 2.0 * h * su),
        combine(&[(1.0, f(0.0, 1.0)), (-1.0, f(0.0, -1.0))], 2.0 * h * sv),
        combine(
            &[(1.0, f(1.0, 0.0)), (-2.0, f(0.0, 0.0)), (1.0, f(-1.0, 0.0))],
            h * h * su * su,
        ),
        combine(
            &[
                (1.0, f(1.0, 1.0)),
                (-1.0, f(1.0, -1.0)),
                (-1.0, f(-1.0, 1.0)),
                (1.0, f(-1.0, -1.0)),
            ],
            4.0 * h * h * su * sv,
        ),
        combine(
            &[(1.0, f(0.0, 1.0)), (-2.0, f(0.0, 0.0)), (1.0, f(0.0, -1.0))],
            h * h * sv * sv,
        ),
    ]
}

/// The largest sampled derivatives over the patch's interior.
fn sampled_maxima(patch: &BezierPatch, su: f64, sv: f64) -> Bounds {
    let mut found = [0.0_f64; 5];
    for a in 1..24 {
        for b in 1..24 {
            let at = sampled(patch, a as f64 / 24.0, b as f64 / 24.0, su, sv);
            for (total, x) in found.iter_mut().zip(at) {
                *total = total.max(x);
            }
        }
    }
    found
}

/// A deterministic pseudo-random sequence in [0, 1).
fn sequence(seed: u64) -> impl FnMut() -> f64 {
    let mut state = seed;
    move || {
        state = state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (state >> 11) as f64 / (1u64 << 53) as f64
    }
}

fn random_patch(seed: u64, rows: usize, columns: usize, rational: bool) -> BezierPatch {
    let mut next = sequence(seed);
    let mut poles = Vec::new();
    let mut weights = Vec::new();
    for i in 0..rows {
        for j in 0..columns {
            poles.push([
                100.0 + 10.0 * i as f64 + 4.0 * next(),
                -40.0 + 10.0 * j as f64 + 4.0 * next(),
                8.0 * next(),
            ]);
            weights.push(if rational { 0.4 + 1.6 * next() } else { 1.0 });
        }
    }
    BezierPatch::new(rows, columns, &poles, &weights).unwrap()
}

/// An octant of a sphere of radius `r` about the origin as a rational
/// biquadratic patch: u the longitude, v the latitude, the pole at v = 1.
fn sphere_octant(r: f64) -> BezierPatch {
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let arc = [([1.0, 0.0], 1.0), ([1.0, 1.0], s), ([0.0, 1.0], 1.0)];
    let mut poles = Vec::new();
    let mut weights = Vec::new();
    for (around, wu) in arc {
        for (meridian, wv) in arc {
            poles.push([
                r * around[0] * meridian[0],
                r * around[1] * meridian[0],
                r * meridian[1],
            ]);
            weights.push(wu * wv);
        }
    }
    BezierPatch::new(3, 3, &poles, &weights).unwrap()
}

#[test]
fn bounds_cover_the_derivatives_of_polynomial_and_rational_patches() {
    for seed in 0..40 {
        let rational = seed % 2 == 1;
        let (rows, columns) = (
            [2, 3, 4, 6][seed as usize % 4],
            [4, 2, 3, 5][(seed as usize / 4) % 4],
        );
        let patch = random_patch(seed, rows, columns, rational);
        let (su, sv) = (0.7, 2.5);
        let bounds = patch.bounds(su, sv);
        let actual = sampled_maxima(&patch, su, sv);
        for k in 0..5 {
            assert!(
                // Central differences of coordinates near 100 carry about 1e-5.
                bounds[k] >= actual[k] * (1.0 - 1e-5) - 1e-3,
                "seed {seed} component {k}: bound {} below sampled {}",
                bounds[k],
                actual[k]
            );
        }
    }
}

#[test]
fn halving_is_exact_and_tightens_a_rational_bound_towards_the_derivative() {
    let patch = sphere_octant(4.0);
    for axis in 0..2 {
        let (low, high) = patch.halves(axis);
        for (a, b) in [(0.1, 0.3), (0.5, 0.5), (0.9, 0.2), (1.0, 1.0), (0.0, 0.7)] {
            let (s, t) = if axis == 0 {
                (0.5 * a, b)
            } else {
                (b, 0.5 * a)
            };
            let (s2, t2) = if axis == 0 {
                (0.5 + 0.5 * a, b)
            } else {
                (b, 0.5 + 0.5 * a)
            };
            let at = if axis == 0 { (a, b) } else { (b, a) };
            for (child, parent) in [(&low, (s, t)), (&high, (s2, t2))] {
                let p = evaluate(child, at.0, at.1);
                let q = evaluate(&patch, parent.0, parent.1);
                assert!(norm(std::array::from_fn(|n| p[n] - q[n])) < 1e-13);
            }
        }
    }
    // A small piece of the octant away from the pole: the bound is within a
    // fifth of the sampled second derivatives (term-by-term bounds of the
    // quotient rule stay three times above them however small the piece).
    let mut piece = patch.clone();
    let mut span = 1.0;
    for _ in 0..4 {
        piece = piece.halves(0).0.halves(1).0;
        span *= 0.5;
    }
    let bounds = piece.bounds(span, span);
    let actual = sampled_maxima(&piece, span, span);
    for k in 2..5 {
        assert!(bounds[k] >= actual[k] * (1.0 - 1e-5));
        assert!(
            bounds[k] <= 1.2 * actual[k] + 1e-9,
            "component {k}: {} vs {}",
            bounds[k],
            actual[k]
        );
    }
}

#[test]
fn local_bounds_cover_boxes_and_vanish_towards_a_pole() {
    let radius = 4.0;
    let mut bounds = SplineBounds::new(
        vec![0.0, 1.0],
        vec![0.0, 1.0],
        vec![sphere_octant(radius)],
        Default::default(),
        [None; 2],
    )
    .unwrap();
    let patch = sphere_octant(radius);
    let whole = bounds.whole();
    let mut next = sequence(7);
    for _ in 0..300 {
        let (a, b, c, d) = (next(), next(), next(), next());
        let (u0, u1, v0, v1) = (a.min(b), a.max(b), c.min(d), c.max(d));
        let (local, kinked) = bounds.local(u0, u1, v0, v1);
        assert!(!kinked);
        for k in 0..5 {
            assert!(local[k] <= whole[k] * (1.0 + 1e-12));
        }
        for i in 0..=6 {
            for j in 0..=6 {
                let (s, t) = (
                    u0 + (u1 - u0) * i as f64 / 6.0,
                    v0 + (v1 - v0) * j as f64 / 6.0,
                );
                if !(1e-3..=1.0 - 1e-3).contains(&s) || !(1e-3..=1.0 - 1e-3).contains(&t) {
                    continue;
                }
                let at = sampled(&patch, s, t, 1.0, 1.0);
                for k in 0..5 {
                    assert!(
                        local[k] >= at[k] * (1.0 - 1e-4) - 1e-5,
                        "box {u0} {u1} {v0} {v1} component {k}"
                    );
                }
            }
        }
    }
    // Along the pole the longitude's derivatives vanish: a box there has a
    // bound on |Fuu| far below the patch's, as an analytic sphere's has.
    let (near, _) = bounds.local(0.2, 0.3, 0.999, 1.0);
    assert!(near[2] < 0.02 * whole[2], "{} vs {}", near[2], whole[2]);
    assert!(near[0] < 0.02 * whole[0]);
    // With a limit, pieces are made only until the box's facet is proven:
    // fewer of them, bounds still above the full descent's and still bounds.
    let mut fresh = SplineBounds::new(
        vec![0.0, 1.0],
        vec![0.0, 1.0],
        vec![sphere_octant(radius)],
        Default::default(),
        [None; 2],
    )
    .unwrap();
    let (coarse, _) = fresh.local_within(0.40, 0.42, 0.40, 0.42, 0.01);
    let made = fresh.pieces();
    assert!(interpolation_error(&coarse, false, 0.02, 0.02) <= 0.01);
    let (fine, _) = fresh.local(0.40, 0.42, 0.40, 0.42);
    assert!(fresh.pieces() > made);
    for k in 0..5 {
        assert!(coarse[k] >= fine[k]);
    }
    assert_eq!(fresh.local_within(0.40, 0.42, 0.40, 0.42, 0.01).0, coarse);
    // A degenerate box (a point, a segment) is answered too.
    let (point, _) = bounds.local(0.5, 0.5, 0.25, 0.25);
    assert!(point.iter().all(|x| x.is_finite()) && point[2] > 0.0);
}

#[test]
fn a_planar_patch_settles_at_once_and_a_curved_one_is_refined_only_where_asked() {
    let flat = BezierPatch::new(
        2,
        2,
        &[
            [0.0, 0.0, 0.0],
            [0.0, 5.0, 0.0],
            [7.0, 0.0, 0.0],
            [7.0, 5.0, 0.0],
        ],
        &[1.0; 4],
    )
    .unwrap();
    let mut bounds = SplineBounds::new(
        vec![0.0, 1.0],
        vec![0.0, 1.0],
        vec![flat],
        Default::default(),
        [None; 2],
    )
    .unwrap();
    assert_eq!(
        bounds.local(0.1, 0.2, 0.3, 0.4).0,
        [7.0, 5.0, 0.0, 0.0, 0.0]
    );
    assert_eq!(bounds.pieces(), 1);
    let mut curved = SplineBounds::new(
        vec![0.0, 1.0],
        vec![0.0, 1.0],
        vec![sphere_octant(4.0)],
        Default::default(),
        [None; 2],
    )
    .unwrap();
    curved.local(0.0, 1.0, 0.0, 1.0);
    assert_eq!(curved.pieces(), 1);
    curved.local(0.4, 0.45, 0.4, 0.45);
    let made = curved.pieces();
    assert!((3..200).contains(&made), "{made}");
    curved.local(0.4, 0.45, 0.4, 0.45);
    assert_eq!(curved.pieces(), made);
}

#[test]
fn periodic_axes_wrap_and_c0_knots_are_reported_only_strictly_inside_a_box() {
    let patches = (0..4).map(|seed| random_patch(seed, 3, 3, false)).collect();
    let mut bounds = SplineBounds::new(
        vec![0.0, 1.0, 2.0],
        vec![0.0, 1.0, 2.0],
        patches,
        [vec![1.0], vec![]],
        [Some(2.0), None],
    )
    .unwrap();
    assert!(!bounds.local(0.2, 1.0, 0.1, 0.4).1);
    assert!(!bounds.local(1.0, 1.7, 0.1, 0.4).1);
    assert!(bounds.local(0.9, 1.1, 0.1, 0.4).1);
    // u = 2.3 is u = 0.3: the bounds of the box one period lower (the
    // shifted box differs in the last bit, so the pieces it meets may too).
    let (upper, lower) = (
        bounds.local(2.2, 2.4, 0.5, 0.7).0,
        bounds.local(0.2, 0.4, 0.5, 0.7).0,
    );
    for k in 0..5 {
        assert!(
            (upper[k] - lower[k]).abs() <= 1e-9 * lower[k],
            "{upper:?} {lower:?}"
        );
    }
    // A box across the seam covers both ends of the period.
    let across = bounds.local(1.9, 2.1, 0.5, 0.7).0;
    let (ends, start) = (
        bounds.local(1.9, 2.0, 0.5, 0.7).0,
        bounds.local(0.0, 0.1, 0.5, 0.7).0,
    );
    for k in 0..5 {
        assert_eq!(across[k], ends[k].max(start[k]));
    }
    assert_eq!(bounds.clamp([2.5, 2.5]), [2.5, 2.0]);
    assert_eq!(bounds.domain(), [0.0, 2.0, 0.0, 2.0]);
}

#[test]
fn invalid_patches_and_knots_are_refused() {
    let poles = [[0.0; 3]; 4];
    assert!(BezierPatch::new(2, 2, &poles, &[1.0, 1.0, 0.0, 1.0]).is_err());
    assert!(BezierPatch::new(2, 3, &poles, &[1.0; 4]).is_err());
    assert!(BezierPatch::new(2, 2, &[[f64::NAN; 3]; 4], &[1.0; 4]).is_err());
    let patch = || BezierPatch::new(2, 2, &poles, &[1.0; 4]).unwrap();
    assert!(
        SplineBounds::new(
            vec![0.0, 0.0],
            vec![0.0, 1.0],
            vec![patch()],
            Default::default(),
            [None; 2]
        )
        .is_err()
    );
    assert!(
        SplineBounds::new(
            vec![0.0, 1.0, 2.0],
            vec![0.0, 1.0],
            vec![patch()],
            Default::default(),
            [None; 2]
        )
        .is_err()
    );
    assert!(
        SplineBounds::new(
            vec![0.0, 1.0],
            vec![0.0, 1.0],
            vec![patch()],
            Default::default(),
            [Some(0.0), None]
        )
        .is_err()
    );
}
