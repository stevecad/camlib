//! Axial roughing passes where the profile leaves a diameter by a taper or
//! by an arc tangent to it (#426).
//!
//! Each pass is checked against the drawing's own numbers: the nose centre
//! (these jobs program the nose centre) keeps the nose radius plus the
//! allowance from the part, and the pass at a diameter's level ends where
//! the taper or the fillet begins.
use super::chucking_tests::{face, finish, job, planned, rough};
use crate::{
    motion::{Curve, Event, MoveKind},
    turning::ops::{TurnPlan, TurningJob, material_report},
};
use serde_json::{Value, json};

/// The turning tests' bar: 40 mm diameter, 2 mm to face off.
fn bar() -> Value {
    json!({
        "version": 1, "units": "mm", "z_min_mm": -60.0, "z_max_mm": 2.0,
        "outer_radius_mm": 20.0, "cell_mm": 0.25, "max_cells": 10000,
    })
}

/// Ø20 for 20 mm, a taper up to Ø30 over 10 mm, then Ø30 ([radius, z, bulge]).
const TAPER: [[f64; 3]; 5] = [
    [0., 0., 0.],
    [10., 0., 0.],
    [10., -20., 0.],
    [15., -30., 0.],
    [15., -40., 0.],
];
/// The nose radius plus the roughing allowance.
const OFFSET: f64 = 0.8 + 0.3;

fn rough_by(depth: f64, z_to: Option<f64>) -> Value {
    let mut op = rough();
    op["depth_of_cut_mm"] = json!(depth);
    if let Some(z) = z_to {
        op["z_to_mm"] = json!(z);
    }
    op
}

/// The job with the nose centre as the programmed point, so a move's ends
/// are the nose centre's.
fn nose_job(profile: &[[f64; 3]], operations: Value) -> TurningJob {
    let mut wire = serde_json::to_value(job(bar(), profile, -50., None, operations)).unwrap();
    wire["controlled_point"] = json!("nose_center");
    serde_json::from_value(wire).unwrap()
}

/// Where each axial cutting pass of `op` ends: the nose centre's (z, r).
fn pass_ends(plan: &TurnPlan, op: &str) -> Vec<[f64; 2]> {
    plan.events
        .iter()
        .filter_map(|e| match e {
            Event::Motion { motion } if motion.operation == op && motion.kind == MoveKind::Cut => {
                let Curve::Line { start, end } = motion.curve else {
                    panic!("{motion:?}");
                };
                assert_eq!(start[0], end[0], "a roughing pass is axial: {motion:?}");
                Some([end[2], end[0]])
            }
            _ => None,
        })
        .collect()
}

/// The distance from (z, r) to the segment a-b (points (z, r)).
fn to_segment(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let (d, w) = ([b[0] - a[0], b[1] - a[1]], [p[0] - a[0], p[1] - a[1]]);
    let t = ((w[0] * d[0] + w[1] * d[1]) / (d[0] * d[0] + d[1] * d[1])).clamp(0., 1.);
    (w[0] - t * d[0]).hypot(w[1] - t * d[1])
}

/// The distance from (z, r) to a profile of straight segments
/// ([radius, z, 0] points).
fn to_profile(p: [f64; 2], profile: &[[f64; 3]]) -> f64 {
    profile
        .windows(2)
        .map(|w| to_segment(p, [w[0][1], w[0][0]], [w[1][1], w[1][0]]))
        .fold(f64::INFINITY, f64::min)
}

/// The taper's radius at z (between z -30 and -20).
fn taper_radius(z: f64) -> f64 {
    10. + (-20. - z) / 2.
}

#[test]
fn a_diameter_followed_by_a_taper_is_roughed_in_steps_that_end_on_the_taper() {
    // Where the offsets of the diameter and of the taper meet: the offset
    // times the tangent of half the taper's angle behind the corner at z -20.
    let half = (0.5_f64).atan() / 2.;
    let corner = -20. + OFFSET * half.tan();
    // The taper's line, as far as it runs.
    let taper = ([-20., 10.], [-30., 15.]);
    // Radially, the nose centre on the taper's offset is this far above it.
    let radial = OFFSET / (0.5_f64).atan().cos();
    for depth in [2., 1.7] {
        let j = nose_job(&TAPER, json!([face(), rough_by(depth, None)]));
        let plan = planned(&j).unwrap();
        let ends = pass_ends(&plan, "rough");
        let mut on_taper = 0;
        let mut at_level = 0;
        for &[z, r] in &ends {
            // No pass comes nearer the part than the nose and the allowance.
            assert!(
                to_profile([z, r], &TAPER) >= OFFSET - 1e-6,
                "{depth}: z {z} r {r}"
            );
            if (r - 11.1).abs() < 1e-9 {
                // The diameter's own pass ends where the taper begins.
                assert!(
                    (z - corner).abs() < 1e-6,
                    "{depth}: z {z}, expected {corner}"
                );
                at_level += 1;
            } else if r < 16.1 - 1e-9 {
                // A pass between the two diameters ends on the taper plus
                // the nose and the allowance, exactly.
                let away = to_segment([z, r], taper.0, taper.1);
                assert!((away - OFFSET).abs() < 1e-6, "{depth}: z {z} r {r}: {away}");
                assert!(z < corner && z > -30., "{depth}: z {z}");
                on_taper += 1;
            } else {
                // Above the large diameter: to the chuck clearance.
                assert_eq!(z, -48., "{depth}: r {r}");
            }
        }
        assert_eq!(at_level, 1, "{depth}: {ends:?}");
        assert!(on_taper >= 2, "{depth}: {ends:?}");
        // The stock roughing leaves, by the drawing: the allowance on the
        // diameter; on the taper never less than the allowance, and never
        // more than it plus one step.
        let mut seen = 0;
        for (i, c) in plan.stock.cells().iter().enumerate() {
            let [z0, z1] = plan.stock.bin(i);
            if z0 >= -18.5 && z1 <= -1.5 {
                assert!(
                    c.outer.upper_mm <= 10.3 + 1e-6 && c.outer.lower_mm >= 10.3 - 1e-6,
                    "{depth}: z {z0}: {c:?}"
                );
            }
            if z0 >= -29. && z1 <= -21. {
                seen += 1;
                let (low, high) = (taper_radius(z1), taper_radius(z0));
                assert!(
                    c.outer.lower_mm >= low + 0.3 - 1e-6,
                    "{depth}: z {z0}: {c:?}"
                );
                assert!(
                    c.outer.upper_mm <= high + radial - 0.8 + depth + 1e-6,
                    "{depth}: z {z0}: {c:?}"
                );
            }
        }
        assert!(seen > 20);
        let report = material_report(&j, &plan.stock).unwrap();
        assert_eq!(report["gouge"]["definite_cells"], 0);
    }
}

#[test]
fn the_tapered_shaft_is_finished_to_size_and_a_bound_on_the_taper_is_kept() {
    let j = nose_job(&TAPER, json!([face(), rough_by(2., None), finish()]));
    let plan = planned(&j).unwrap();
    let mut seen = 0;
    for (i, c) in plan.stock.cells().iter().enumerate() {
        let [z0, z1] = plan.stock.bin(i);
        if z0 >= -18.5 && z1 <= -1.5 {
            assert!(
                (c.outer.upper_mm - 10.).abs() < 1e-6 && (c.outer.lower_mm - 10.).abs() < 1e-6,
                "z {z0}: {c:?}"
            );
        }
        if z0 >= -29. && z1 <= -21. {
            seen += 1;
            // The taper itself, within the setup's tolerance.
            assert!(
                c.outer.upper_mm <= taper_radius(z0) + 2e-3
                    && c.outer.lower_mm >= taper_radius(z1) - 2e-3,
                "z {z0}: {c:?}"
            );
        }
    }
    assert!(seen > 20);
    let report = material_report(&j, &plan.stock).unwrap();
    assert_eq!(report["gouge"]["definite_cells"], 0);
    assert_eq!(report["complete"], true);
    // Leftover only where the 0.8 mm nose cannot reach: the inside corner
    // between the diameter and the taper at z -20.
    for f in report["findings"].as_array().unwrap() {
        if f["residual"] == "definite_excess" {
            let z = f["z_mm"].as_array().unwrap();
            assert!(
                z[0].as_f64().unwrap() >= -20.5 && z[1].as_f64().unwrap() <= -19.5,
                "{f}"
            );
        }
    }

    // Roughed to a bound on the taper: the diameter's pass still ends where
    // the taper begins, the passes above it on the taper or at the bound.
    let bounded = nose_job(&TAPER, json!([face(), rough_by(2., Some(-25.))]));
    let plan = planned(&bounded).unwrap();
    let corner = -20. + OFFSET * ((0.5_f64).atan() / 2.).tan();
    let ends = pass_ends(&plan, "rough");
    assert!(ends.len() >= 4, "{ends:?}");
    for &[z, r] in &ends {
        assert!(to_profile([z, r], &TAPER) >= OFFSET - 1e-6, "z {z} r {r}");
        assert!(z >= -25., "z {z} r {r}");
        if (r - 11.1).abs() < 1e-9 {
            assert!((z - corner).abs() < 1e-6, "z {z}");
        }
    }
    assert!(ends.iter().any(|e| (e[1] - 11.1).abs() < 1e-9), "{ends:?}");
}

#[test]
fn a_fillet_tangent_to_a_diameter_ends_the_diameters_pass_where_it_begins() {
    // Ø20 for 17 mm, a 3 mm fillet tangent to it up to a shoulder at z -20,
    // then Ø30. The fillet's centre is at z -17, r 13.
    let bulge = -(std::f64::consts::PI / 8.).tan();
    let profile = [
        [0., 0., 0.],
        [10., 0., 0.],
        [10., -17., bulge],
        [13., -20., 0.],
        [15., -20., 0.],
        [15., -40., 0.],
    ];
    let j = nose_job(&profile, json!([face(), rough_by(2., None)]));
    let plan = planned(&j).unwrap();
    let ends = pass_ends(&plan, "rough");
    let mut in_fillet = 0;
    for &[z, r] in &ends {
        if r < 13. && z < -17. {
            // Inside the fillet's quarter: the nose centre stays at least
            // the nose and the allowance inside the 3 mm arc, to within the
            // quarter of the tolerance its chords are allowed.
            let from_centre = (z + 17.).hypot(r - 13.);
            assert!(
                from_centre <= 3. - OFFSET + 1e-3 / 4. + 1e-9,
                "z {z} r {r}: {from_centre}"
            );
            in_fillet += 1;
        }
    }
    // The diameter's own pass ends at the start of the fillet (z -17), not
    // a chord further along it.
    let level: Vec<_> = ends.iter().filter(|e| (e[1] - 11.1).abs() < 1e-9).collect();
    assert_eq!(level.len(), 1, "{ends:?}");
    assert!(level[0][0] <= -17. && level[0][0] > -17.05, "{level:?}");
    assert!(in_fillet >= 2, "{ends:?}");
    let report = material_report(&j, &plan.stock).unwrap();
    assert_eq!(report["gouge"]["definite_cells"], 0);
}
