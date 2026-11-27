//! A second chucking (#418): the jaws on the part itself.
//!
//! Jobs are read from their wire form, as a stream manifest's are, so these
//! tests exercise what an application sends.
use crate::{
    motion::{Curve, Event, Motion, MoveKind},
    turning::{
        ops::{TurnPlan, TurnReplay, TurningJob, material_report, plan},
        stock::{TurnSide, TurnStock},
        tool::{Hand, Holder, Insert},
    },
};
use serde_json::{Value, json};
use std::sync::atomic::AtomicBool;

/// An 80 degree insert on a 95 degree holder (the turning tests' tool).
fn cnmg() -> Value {
    let insert = Insert::iso_turning(80., 95., 12., 0.8).unwrap();
    let holder = Holder::rectangle(&insert, 0.8, TurnSide::Outer, 20., 100., 0.).unwrap();
    json!({
        "tool": {
            "id": "cnmg",
            "insert": insert,
            "nose_radius_mm": 0.8,
            "side": TurnSide::Outer,
            "hand": Hand::Right,
            "holder": holder,
            "body": {
                "insert": {
                    "insert_thickness_mm": 3.0,
                    "insert_clearance_deg": 7.0,
                    "rake_deg": 0.0,
                    "inclination_deg": 0.0,
                    "edge_deg": 5.0,
                },
                "holder_y_mm": [-10.0, 2.0],
                "bar_axis_y_mm": 0.0,
            },
        },
        "number": 1,
        "offset": 1,
    })
}

/// A sawn bar: 32 mm diameter, 64 mm long, 2 mm to face off each end.
fn bar() -> Value {
    json!({
        "version": 1, "units": "mm", "z_min_mm": -62.0, "z_max_mm": 2.0,
        "outer_radius_mm": 16.0, "cell_mm": 0.25, "max_cells": 10000,
    })
}

fn job(
    stock: Value,
    profile: &[[f64; 3]],
    chuck: f64,
    grip: Option<[f64; 2]>,
    operations: Value,
) -> TurningJob {
    let mut job = json!({
        "id": "shaft",
        "stock": stock,
        "part": {"points": profile},
        "chuck_z_mm": chuck,
        "tools": [cnmg()],
        "operations": operations,
        "initial_position_mm": [30.0, 20.0],
        "clearance_mm": 2.0,
        "tolerance_mm": 1e-3,
    });
    if let Some([length, radius]) = grip {
        job["grip"] = json!({"length_mm": length, "radius_mm": radius});
    }
    serde_json::from_value(job).unwrap()
}

fn face() -> Value {
    json!({"kind": "face", "id": "face", "tool": "cnmg", "z_mm": 0.0, "stepdown_mm": 1.0,
           "feed_mm_rev": 0.15, "rpm": 1200.0})
}
fn rough() -> Value {
    json!({"kind": "rough", "id": "rough", "tool": "cnmg", "depth_of_cut_mm": 2.0,
           "allowance_mm": 0.3, "feed_mm_rev": 0.25, "rpm": 1200.0})
}
fn finish() -> Value {
    json!({"kind": "finish", "id": "finish", "tool": "cnmg", "feed_mm_rev": 0.1, "rpm": 1500.0})
}

fn planned(job: &TurningJob) -> Result<TurnPlan, String> {
    plan(job, &AtomicBool::new(false))
}

/// A shaft 60 mm long whose 30 mm diameter runs on to its chuck-side end.
const LONG: [[f64; 3]; 5] = [
    [0., 0., 0.],
    [10., 0., 0.],
    [10., -20., 0.],
    [15., -20., 0.],
    [15., -60., 0.],
];

/// Every cell of `stock` between z0 and z1 has the outer radius `r`.
fn radius_is(stock: &TurnStock, z0: f64, z1: f64, r: f64) {
    let mut seen = 0;
    for (i, c) in stock.cells().iter().enumerate() {
        let [a, b] = stock.bin(i);
        if a >= z0 && b <= z1 {
            seen += 1;
            assert!(
                (c.outer.upper_mm - r).abs() < 1e-6 && (c.outer.lower_mm - r).abs() < 1e-6,
                "z {a}..{b}: {c:?}, expected r {r}"
            );
        }
    }
    assert!(seen > 0, "no cell between z {z0} and z {z1}");
}

#[test]
fn a_chuck_face_along_the_part_needs_a_declared_grip() {
    // The jaws hold the bar from z -62 to -44; the part runs to z -60.
    let operations = json!([face(), rough(), finish()]);
    let undeclared = job(bar(), &LONG, -44., None, operations.clone());
    let error = planned(&undeclared).err().unwrap();
    assert!(
        error.contains("in front of the chuck face") && error.contains("TurningJob.grip"),
        "{error}"
    );
    // Declared: planned to the clearance in front of the chuck face, and
    // not a hair further.
    let held = job(bar(), &LONG, -44., Some([18., 16.]), operations);
    let plan = planned(&held).unwrap();
    radius_is(&plan.stock, -18.5, -1.5, 10.);
    radius_is(&plan.stock, -41., -21.5, 15.);
    // Behind the chuck face the bar is as it was: nothing was machined.
    radius_is(&plan.stock, -62., -44., 16.);
    // No move's programmed point (the tool tip, the nose's own extreme
    // toward the chuck) is behind the clearance in front of the face.
    let mut cuts = 0;
    for event in &plan.events {
        let Event::Motion { motion } = event else {
            continue;
        };
        let (Curve::Line { start, end } | Curve::Arc { start, end, .. }) = motion.curve else {
            panic!("{motion:?}");
        };
        assert!(start[2].min(end[2]) >= -42. - 0.8 - 1e-9, "{motion:?}");
        cuts += usize::from(motion.kind == MoveKind::Cut);
    }
    assert!(cuts > 10);
    // The finish ran to the clearance exactly (its nose centre at z -42).
    let last_cut = plan
        .events
        .iter()
        .filter_map(|e| match e {
            Event::Motion { motion }
                if motion.operation == "finish" && motion.kind == MoveKind::Cut =>
            {
                Some(motion)
            }
            _ => None,
        })
        .next_back()
        .unwrap();
    let Curve::Line { end, .. } = last_cut.curve else {
        panic!("{last_cut:?}");
    };
    assert!(
        (end[2] + 42.8).abs() < 1e-9 && (end[0] - 15.).abs() < 1e-9,
        "{end:?}"
    );
    // What the jaws hold is reported as left for another setup, by the
    // operations and by the material check (leftover, not a failure).
    for report in &plan.reports[1..] {
        assert_eq!(
            report.left_for_another_setup_z_mm,
            vec![[-60., -42.]],
            "{}",
            report.id
        );
    }
    assert!(plan.reports[0].left_for_another_setup_z_mm.is_empty());
    let material = material_report(&held, &plan.stock).unwrap();
    assert_eq!(material["held"]["z_mm"], json!([-62., -44.]));
    assert!(
        material["held"]["definite_cells"].as_u64().unwrap() > 50,
        "{}",
        material["held"]
    );
    assert_eq!(material["gouge"]["definite_cells"], 0);
    for finding in material["findings"].as_array().unwrap() {
        let z0 = finding["z_mm"][0].as_f64().unwrap();
        assert_eq!(
            finding.get("behind_chuck_face").is_some(),
            z0 < -44.,
            "{finding}"
        );
    }
}

#[test]
fn a_cut_that_would_reach_the_jaws_is_refused_by_name() {
    let held = |operations: Value| job(bar(), &LONG, -44., Some([18., 16.]), operations);
    // An operation whose own target is behind the chuck face.
    let groove = json!({"kind": "groove", "id": "groove", "tool": "cnmg", "z_from_mm": -47.0,
        "z_to_mm": -50.0, "bottom_radius_mm": 12.0, "feed_mm_rev": 0.05, "rpm": 800.0});
    let error = planned(&held(json!([face(), groove]))).err().unwrap();
    assert!(
        error.starts_with("groove: ")
            && error.contains("behind the chuck face (z -44)")
            && error.contains("jaws hold"),
        "{error}"
    );
    let deep_face = json!({"kind": "face", "id": "face", "tool": "cnmg", "z_mm": -45.0,
        "stepdown_mm": 1.0, "feed_mm_rev": 0.15, "rpm": 1200.0});
    let error = planned(&held(json!([deep_face]))).err().unwrap();
    assert!(
        error.contains("behind the chuck face") && error.contains("jaws hold"),
        "{error}"
    );
    // A move (as a posted program read back gives it) that would take the
    // tool behind the chuck face: the move kernel names the jaws.
    let j = held(json!([face(), rough(), finish()]));
    let mut replay = TurnReplay::new(j.clone()).unwrap();
    let into = |z: f64| Motion {
        id: 1,
        operation: "finish".into(),
        tool: "cnmg".into(),
        curve: Curve::Line {
            start: [20., 0., 5.],
            end: [20., 0., z],
        },
        kind: MoveKind::Rapid,
        feed_mm_min: None,
    };
    let error = replay.motion(&into(-44.5)).unwrap_err();
    assert!(
        error.contains("reach the chuck")
            && error.contains("jaws hold the work")
            && error.contains("18 mm"),
        "{error}"
    );
    // In front of the face the same move is taken.
    replay.motion(&into(-40.)).unwrap();
    // Without a grip the chuck's refusal reads as it always did.
    let mut plain = job(bar(), &LONG[..4], -44., None, json!([face()]));
    plain.part.points.push([15., -40., 0.]);
    let mut replay = TurnReplay::new(plain).unwrap();
    let error = replay.motion(&into(-44.5)).unwrap_err();
    assert!(
        error.contains("reach the chuck") && !error.contains("jaws"),
        "{error}"
    );
}

#[test]
fn a_grip_is_checked_against_the_stock_and_the_part() {
    let with = |chuck: f64, grip: [f64; 2]| {
        planned(&job(bar(), &LONG, chuck, Some(grip), json!([face()])))
    };
    // The jaws close on the bar's own radius, not another.
    let error = with(-44., [18., 15.]).err().unwrap();
    assert!(
        error.contains("parallel diameter") && error.contains("a radius between 16 and 16"),
        "{error}"
    );
    // Not past the end of the stock.
    let error = with(-44., [30., 16.]).err().unwrap();
    assert!(error.contains("the stock ends at z -62"), "{error}");
    // Something of the part is in front of the chuck clearance.
    let error = with(-1., [18., 16.]).err().unwrap();
    assert!(error.contains("in front of the chuck clearance"), "{error}");
    // A part larger, inside the jaws, than the diameter they close on.
    let mut fat = job(bar(), &LONG, -44., Some([18., 16.]), json!([face()]));
    fat.part.points[3][0] = 16.5;
    fat.part.points[4][0] = 16.5;
    let error = planned(&fat).err().unwrap();
    assert!(
        error.contains("the jaws would be inside the part"),
        "{error}"
    );
    for bad in [[0., 16.], [18., 0.], [-3., 16.]] {
        assert!(with(-44., bad).err().unwrap().contains("must be positive"));
    }
}

/// The same shaft turned around: its 30 mm diameter first, then the 20 mm
/// diameter the first chucking finished.
const LONG_TURNED: [[f64; 3]; 5] = [
    [0., 0., 0.],
    [15., 0., 0.],
    [15., -40., 0.],
    [10., -40., 0.],
    [10., -60., 0.],
];

#[test]
fn the_second_chucking_starts_from_the_stock_the_first_left() {
    let operations = json!([face(), rough(), finish()]);
    // First chucking: the bar held by 18 mm, turned to z -42.
    let first = job(bar(), &LONG, -44., Some([18., 16.]), operations.clone());
    let left = planned(&first).unwrap().stock;
    // Turned end for end: the part's chuck-side end (z -60) becomes the
    // front face (z 0), the 2 mm of bar behind it the stock to face off.
    let spec = left.carried(true, -60.).unwrap();
    assert_eq!([spec.z_min_mm, spec.z_max_mm], [-60., 2.]);
    assert_eq!([spec.bore_radius_mm, spec.outer_radius_mm], [0., 16.]);
    // It is what the first left, cell for cell, in the other order (less
    // the faced-off front, which holds nothing).
    let material: Vec<_> = left
        .cells()
        .iter()
        .filter(|c| c.outer.upper_mm > c.inner.lower_mm)
        .rev()
        .copied()
        .collect();
    let stock = TurnStock::new(spec.clone()).unwrap();
    assert_eq!(stock.cells(), &material[..]);
    radius_is(&stock, -17., 2., 16.); // the bar the jaws held
    radius_is(&stock, -39.5, -18.5, 15.); // turned in the first chucking
    radius_is(&stock, -59.5, -41.5, 10.);
    // Second chucking: gripped on the finished 20 mm diameter by 18 mm, the
    // jaw face 2 mm (the clearance) behind the shoulder.
    let carried = serde_json::to_value(&spec).unwrap();
    let second = job(
        carried.clone(),
        &LONG_TURNED,
        -42.,
        Some([18., 10.]),
        operations.clone(),
    );
    let plan = planned(&second).unwrap();
    // The stock against the part, independently of the planner: the whole
    // part is at size, both ends, and no more material is left than the
    // 0.8 mm nose's fillet at the shoulder.
    radius_is(&plan.stock, -39.5, -0.5, 15.);
    radius_is(&plan.stock, -59.5, -41.5, 10.);
    let part = std::f64::consts::PI * (15f64.powi(2) * 40. + 10f64.powi(2) * 20.);
    let [lower, upper] = plan.stock.volume();
    // (The certain bound loses the cell at each faced end and shoulder,
    // the possible bound holds the nose's fillets.)
    assert!(
        lower <= part && lower > 0.985 * part && upper >= part && upper < part + 25.,
        "{lower} {upper} {part}"
    );
    let material = material_report(&second, &plan.stock).unwrap();
    assert_eq!(material["gouge"]["definite_cells"], 0);
    assert_eq!(
        material["held"],
        json!({"z_mm": [-60., -42.], "definite_cells": 0, "possible_only_cells": 0})
    );
    let leftover: Vec<&Value> = material["findings"]
        .as_array()
        .unwrap()
        .iter()
        .filter(|f| f["residual"] == "definite_excess")
        .collect();
    assert!(!leftover.is_empty());
    for finding in leftover {
        let z = [
            finding["z_mm"][0].as_f64().unwrap(),
            finding["z_mm"][1].as_f64().unwrap(),
        ];
        assert!(z[0] >= -41.05 && z[1] <= -40., "{finding}");
    }
    // The held end is this chucking's to leave alone, and says so.
    assert_eq!(
        plan.reports[2].left_for_another_setup_z_mm,
        vec![[-60., -40.]]
    );
    // Every planned move replays on the carried stock (as a posted program
    // read back does) and ends with the planned stock.
    let mut replay = TurnReplay::new(second.clone()).unwrap();
    let mut replayed = TurnStock::new(spec).unwrap();
    for event in &plan.events {
        if let Event::Motion { motion } = event {
            let delta = replay.motion(motion).unwrap();
            replayed.apply_move(&delta.changes, &delta.rings).unwrap();
        }
    }
    assert_eq!(replayed.cells(), plan.stock.cells());
    // A wrong stock model is refused, not planned around. The untouched
    // bar where the first chucking's stock should be: the jaws would close
    // on 32 mm, not on the 20 mm declared.
    let mut blank = bar();
    blank["z_min_mm"] = json!(-60.0);
    let error = planned(&job(
        blank,
        &LONG_TURNED,
        -42.,
        Some([18., 10.]),
        operations.clone(),
    ))
    .err()
    .unwrap();
    assert!(
        error.contains("jaws hold a parallel diameter")
            && error.contains("a radius between 16 and 16"),
        "{error}"
    );
    // A chuck face set onto the shoulder: the jaws would be on its fillet
    // and on the 30 mm diameter.
    let error = planned(&job(
        carried,
        &LONG_TURNED,
        -39.,
        Some([18., 10.]),
        operations,
    ))
    .err()
    .unwrap();
    assert!(error.contains("jaws hold a parallel diameter"), "{error}");
}

#[test]
fn a_declared_stepped_blank_is_the_stock() {
    // A forging: 34 mm diameter for 22 mm, then 24 mm.
    let blank = json!({
        "version": 1, "units": "mm", "z_min_mm": -62.0, "z_max_mm": 2.0,
        "outer_radius_mm": 17.0, "cell_mm": 0.25, "max_cells": 10000,
        "sections": [
            {"z_mm": [-62.0, -20.0], "outer_mm": [17.0, 17.0], "inner_mm": [0.0, 0.0]},
            {"z_mm": [-20.0, 2.0], "outer_mm": [12.0, 12.0], "inner_mm": [0.0, 0.0]},
        ],
    });
    let shaft = &LONG[..4];
    let mut profile = shaft.to_vec();
    profile.push([15., -40., 0.]);
    let forged = job(
        blank,
        &profile,
        -50.,
        None,
        json!([face(), rough(), finish()]),
    );
    let plan = planned(&forged).unwrap();
    radius_is(&plan.stock, -18.5, -1.5, 10.);
    radius_is(&plan.stock, -38.5, -21.5, 15.);
    // The stock started as the blank, not as a bar of its largest
    // diameter: less was there to remove.
    let mut on_bar = bar();
    on_bar["outer_radius_mm"] = json!(17.0);
    let bar_job = job(
        on_bar,
        &profile,
        -50.,
        None,
        json!([face(), rough(), finish()]),
    );
    let bar_plan = planned(&bar_job).unwrap();
    assert_eq!(plan.stock.cells()[100..], bar_plan.stock.cells()[100..]);
    let removed = |job: &TurningJob, plan: &TurnPlan| {
        TurnStock::new(job.stock.clone()).unwrap().volume()[1] - plan.stock.volume()[1]
    };
    assert!(removed(&forged, &plan) < 0.7 * removed(&bar_job, &bar_plan));
}

/// A shaft 60 mm long: a 20 mm diameter, a 30 mm collar and a 24 mm
/// diameter, 20 mm each. The collar's far shoulder faces away from the
/// tool, whichever end is held.
const A_SIDE: [[f64; 3]; 7] = [
    [0., 0., 0.],
    [10., 0., 0.],
    [10., -20., 0.],
    [15., -20., 0.],
    [15., -40., 0.],
    [12., -40., 0.],
    [12., -60., 0.],
];
/// The same shaft turned around (z becomes -60 - z).
const B_SIDE: [[f64; 3]; 7] = [
    [0., 0., 0.],
    [12., 0., 0.],
    [12., -20., 0.],
    [15., -20., 0.],
    [15., -40., 0.],
    [10., -40., 0.],
    [10., -60., 0.],
];

/// An operation with the stretch of the profile it turns.
fn between(mut operation: Value, z_from: Option<f64>, z_to: Option<f64>) -> Value {
    if let Some(z) = z_from {
        operation["z_from_mm"] = json!(z);
    }
    if let Some(z) = z_to {
        operation["z_to_mm"] = json!(z);
    }
    operation
}

/// The programmed points (x = radius, z) of an operation's moves of `kind`.
fn ends(plan: &TurnPlan, operation: &str, kind: MoveKind) -> Vec<[f64; 2]> {
    plan.events
        .iter()
        .filter_map(|e| match e {
            Event::Motion { motion } if motion.operation == operation && motion.kind == kind => {
                let (Curve::Line { end, .. } | Curve::Arc { end, .. }) = motion.curve else {
                    panic!("{motion:?}");
                };
                Some([end[0], end[2]])
            }
            _ => None,
        })
        .collect()
}

#[test]
fn roughing_and_finishing_turn_the_stretch_of_the_profile_they_are_given() {
    let held = |operations: Value| job(bar(), &A_SIDE, -44., Some([18., 16.]), operations);
    // The whole profile as the target: the finish is refused at the
    // collar's far shoulder, which the insert's trailing edge cannot follow.
    let error = planned(&held(json!([face(), rough(), finish()])))
        .err()
        .unwrap();
    assert!(
        error.starts_with("finish: the insert would cut") && error.contains("into the part"),
        "{error}"
    );
    // Told to turn as far as that shoulder, both operations plan.
    let to_collar = json!([
        face(),
        between(rough(), None, Some(-40.)),
        between(finish(), None, Some(-40.))
    ]);
    let plan = planned(&held(to_collar)).unwrap();
    radius_is(&plan.stock, -18.5, -1.5, 10.);
    radius_is(&plan.stock, -39.5, -21.5, 15.);
    // Behind the bound (and the nose radius that reaches past it) the bar
    // is as it was.
    radius_is(&plan.stock, -62., -41., 16.);
    // Every pass and the finishing path end at the bound, exactly: the
    // tool tip one nose radius behind the nose centre.
    let rough_cuts = ends(&plan, "rough", MoveKind::Cut);
    assert!(
        rough_cuts.iter().all(|p| p[1] >= -40.8 - 1e-9),
        "{rough_cuts:?}"
    );
    assert!(
        rough_cuts.iter().any(|p| (p[1] + 40.8).abs() < 1e-9),
        "{rough_cuts:?}"
    );
    let last = *ends(&plan, "finish", MoveKind::Cut).last().unwrap();
    assert!(
        (last[0] - 15.).abs() < 1e-9 && (last[1] + 40.8).abs() < 1e-9,
        "{last:?}"
    );
    // The rest is reported as left for another setup: not a failure.
    for report in &plan.reports[1..] {
        assert_eq!(
            report.left_for_another_setup_z_mm,
            vec![[-60., -40.]],
            "{}",
            report.id
        );
    }
    // Refused by name: a bound behind the chuck clearance (where the jaws
    // are), bounds the wrong way round, a bound in front of the part.
    let refused = |z_from: Option<f64>, z_to: Option<f64>| {
        planned(&held(json!([face(), between(finish(), z_from, z_to)])))
            .err()
            .unwrap()
    };
    let error = refused(None, Some(-43.));
    assert!(
        error.starts_with(
            "finish: z_to_mm -43 lies behind the clearance in front of the chuck face (z -42"
        ) && error.contains("jaws hold the work"),
        "{error}"
    );
    assert!(refused(Some(-30.), Some(-20.)).contains("z_from_mm -30 must be in front of"));
    assert!(refused(None, Some(0.5)).contains("must be behind the part's front face"));
    let error = planned(&held(json!([face(), between(rough(), Some(-50.), None)])))
        .err()
        .unwrap();
    assert!(
        error.starts_with("rough: z_from_mm -50 must be in front of"),
        "{error}"
    );
}

#[test]
fn an_operation_starts_at_a_front_bound_inside_the_profile() {
    // The 20 mm end is roughed first; then the 30 mm diameter behind the
    // shoulder is roughed and finished on its own.
    let operations = json!([
        face(),
        between(rough(), None, Some(-20.)),
        {"kind": "rough", "id": "rough30", "tool": "cnmg", "depth_of_cut_mm": 2.0, "allowance_mm": 0.3,
         "feed_mm_rev": 0.25, "rpm": 1200.0, "z_from_mm": -20.0},
        between(finish(), Some(-20.), None),
    ]);
    let plan = planned(&job(bar(), &LONG, -44., Some([18., 16.]), operations)).unwrap();
    // The diameter behind the bound is finished to the chuck clearance;
    // the end in front of it keeps its roughing allowance.
    radius_is(&plan.stock, -41., -21., 15.);
    radius_is(&plan.stock, -17.5, -1.5, 10.3);
    // The second roughing's passes start a clearance in front of its
    // bound, not in front of the stock.
    let rapids = ends(&plan, "rough30", MoveKind::Rapid);
    assert!(
        rapids.iter().any(|p| (p[1] - (-18. - 0.8)).abs() < 1e-9),
        "{rapids:?}"
    );
    let cuts = ends(&plan, "rough30", MoveKind::Cut);
    assert!(
        !cuts.is_empty() && cuts.iter().all(|p| (p[1] + 42.8).abs() < 1e-9),
        "{cuts:?}"
    );
    // The finish feeds in radially at its bound and follows the path from
    // there: its first cut ends at the chuck clearance, on the diameter.
    let entry = ends(&plan, "finish", MoveKind::Plunge);
    assert_eq!(entry.len(), 1);
    assert!(
        (entry[0][0] - 15.).abs() < 1e-9 && (entry[0][1] + 20.8).abs() < 1e-9,
        "{entry:?}"
    );
    let cuts = ends(&plan, "finish", MoveKind::Cut);
    assert!(
        cuts.iter()
            .all(|p| (p[0] - 15.).abs() < 1e-9 && p[1] >= -42.8 - 1e-9),
        "{cuts:?}"
    );
    // Both ends of the profile it did not turn are reported.
    assert_eq!(
        plan.reports[3].left_for_another_setup_z_mm,
        vec![[-60., -42.], [-20., 0.]]
    );
    assert_eq!(
        plan.reports[1].left_for_another_setup_z_mm,
        vec![[-60., -20.]]
    );
    // A bounded operation reports what it left without a grip too.
    let mut short = LONG[..4].to_vec();
    short.push([15., -40., 0.]);
    let plain = job(
        bar(),
        &short,
        -50.,
        None,
        json!([face(), between(rough(), None, Some(-30.))]),
    );
    assert_eq!(
        planned(&plain).unwrap().reports[1].left_for_another_setup_z_mm,
        vec![[-40., -30.]]
    );
}

#[test]
fn a_stepped_shaft_is_turned_in_two_chuckings() {
    // First chucking: the sawn bar in the jaws by 18 mm. Faced, then the
    // 20 mm end and the collar roughed and finished as far as the collar's
    // far shoulder.
    let first = job(
        bar(),
        &A_SIDE,
        -44.,
        Some([18., 16.]),
        json!([
            face(),
            between(rough(), None, Some(-40.)),
            between(finish(), None, Some(-40.))
        ]),
    );
    let first_plan = planned(&first).unwrap();
    let after_first = material_report(&first, &first_plan.stock).unwrap();
    assert_eq!(after_first["gouge"]["definite_cells"], 0);
    // The other end is still the bar: leftover, behind the chuck face.
    assert!(
        after_first["held"]["definite_cells"].as_u64().unwrap() > 50,
        "{}",
        after_first["held"]
    );
    // Second chucking: the part turned around (the old z -60 is the new
    // front face), its stock what the first left, gripped on the finished
    // 20 mm diameter by 15 mm with 5 mm of it in front of the jaws.
    let left = first_plan.stock.carried(true, -60.).unwrap();
    let stock = TurnStock::new(left.clone()).unwrap();
    let material: Vec<_> = first_plan
        .stock
        .cells()
        .iter()
        .filter(|c| c.outer.upper_mm > c.inner.lower_mm)
        .rev()
        .copied()
        .collect();
    assert_eq!(stock.cells(), &material[..]);
    let second = job(
        serde_json::to_value(&left).unwrap(),
        &B_SIDE,
        -45.,
        Some([15., 10.]),
        json!([
            face(),
            between(rough(), None, Some(-20.)),
            between(finish(), None, Some(-20.))
        ]),
    );
    let plan = planned(&second).unwrap();
    // Faced to length and the other end turned: the stock is the part,
    // end to end, checked here against the drawing's own numbers.
    for (i, c) in plan.stock.cells().iter().enumerate() {
        assert!(
            plan.stock.bin(i)[0] < 0. || c.outer.upper_mm == 0.,
            "faced to length: {c:?}"
        );
    }
    radius_is(&plan.stock, -18.5, -0.5, 12.);
    radius_is(&plan.stock, -39.5, -20.5, 15.);
    radius_is(&plan.stock, -59.5, -41.5, 10.);
    let part = std::f64::consts::PI * 20. * (12f64.powi(2) + 15f64.powi(2) + 10f64.powi(2));
    let [lower, upper] = plan.stock.volume();
    // (The certain bound loses the cell at each faced end and shoulder,
    // the possible bound holds the nose's fillets.)
    assert!(
        lower <= part && lower > 0.985 * part && upper >= part && upper < part + 100.,
        "{lower} {upper} {part}"
    );
    // The material check agrees: no gouge, and leftover only in the two
    // fillets the 0.8 mm nose leaves at the shoulders it cut toward; none
    // of it behind the chuck face.
    let report = material_report(&second, &plan.stock).unwrap();
    assert_eq!(report["gouge"]["definite_cells"], 0);
    assert_eq!(
        report["held"],
        json!({"z_mm": [-60., -45.], "definite_cells": 0, "possible_only_cells": 0})
    );
    for finding in report["findings"].as_array().unwrap() {
        if finding["residual"] != "definite_excess" {
            continue;
        }
        let z = [
            finding["z_mm"][0].as_f64().unwrap(),
            finding["z_mm"][1].as_f64().unwrap(),
        ];
        assert!(
            (z[0] >= -41.05 && z[1] <= -40.) || (z[0] >= -20. && z[1] <= -18.95),
            "{finding}"
        );
    }
    // No programmed point of the second chucking is behind the clearance
    // in front of the jaws (z -43), and none of its cuts behind its bound.
    for operation in ["face", "rough", "finish"] {
        for kind in [
            MoveKind::Rapid,
            MoveKind::Cut,
            MoveKind::Plunge,
            MoveKind::Retract,
        ] {
            for p in ends(&plan, operation, kind) {
                assert!(p[1] >= -20.8 - 1e-9, "{operation} {kind:?} {p:?}");
            }
        }
    }
    // A cut that would reach the jaws is refused by name: turning on to
    // the diameter they hold, and a groove under them.
    let reach = |operations: Value| {
        planned(&job(
            serde_json::to_value(&left).unwrap(),
            &B_SIDE,
            -45.,
            Some([15., 10.]),
            operations,
        ))
        .err()
        .unwrap()
    };
    let error = reach(json!([face(), between(finish(), None, Some(-44.))]));
    assert!(
        error.contains("z_to_mm -44 lies behind the clearance in front of the chuck face")
            && error.contains("jaws hold the work"),
        "{error}"
    );
    let groove = json!({"kind": "groove", "id": "groove", "tool": "cnmg", "z_from_mm": -50.0,
        "z_to_mm": -53.0, "bottom_radius_mm": 8.0, "feed_mm_rev": 0.05, "rpm": 800.0});
    let error = reach(json!([groove]));
    assert!(
        error.contains("behind the chuck face (z -45)") && error.contains("jaws hold"),
        "{error}"
    );
}

/// A bar `radius` thick for the collar shaft: 2 mm to face off, 17 mm
/// behind the part for the jaws (the chuck face at z -67).
fn shelf_bar(radius: f64) -> Value {
    json!({
        "version": 1, "units": "mm", "z_min_mm": -77.0, "z_max_mm": 2.0,
        "outer_radius_mm": radius, "cell_mm": 0.25, "max_cells": 10000,
    })
}

/// An operation's motions, in order.
fn motions<'a>(plan: &'a TurnPlan, operation: &str) -> Vec<&'a Motion> {
    plan.events
        .iter()
        .filter_map(|e| match e {
            Event::Motion { motion } if motion.operation == operation => Some(motion),
            _ => None,
        })
        .collect()
}

/// A straight motion's programmed ends, each (x = radius, z).
fn line(motion: &Motion) -> [[f64; 2]; 2] {
    let Curve::Line { start, end } = motion.curve else {
        panic!("{motion:?}");
    };
    [[start[0], start[2]], [end[0], end[2]]]
}

#[test]
fn a_finish_that_ends_against_standing_stock_feeds_out_along_x() {
    // The collar shaft's first chucking from the bar on the shelf: 45 mm
    // for a 30 mm collar (#425). Roughing to the collar's far shoulder
    // leaves a wall 7.5 mm high at the bound.
    let to_collar = || {
        json!([
            face(),
            between(rough(), None, Some(-40.)),
            between(finish(), None, Some(-40.))
        ])
    };
    let first = job(shelf_bar(22.5), &A_SIDE, -67., None, to_collar());
    let plan = planned(&first).unwrap();
    // The finishing path ends at the bound on the collar; from there one
    // feed move, along X only, to the safe radius: the tool tip a
    // clearance (2 mm) above the bar. It is a cutting move at the
    // operation's feed (0.1 mm/rev at 1500 rpm), not a rapid.
    let finish_moves = motions(&plan, "finish");
    let at = finish_moves
        .iter()
        .position(|m| m.kind == MoveKind::Retract)
        .unwrap();
    let out = finish_moves[at];
    assert_eq!(finish_moves[at - 1].kind, MoveKind::Cut);
    let [from, to] = line(out);
    assert!(
        (from[0] - 15.).abs() < 1e-9 && (from[1] + 40.8).abs() < 1e-9,
        "{out:?}"
    );
    assert!(
        to[1] == from[1] && (to[0] - 24.5).abs() < 1e-9,
        "the feed out goes straight up the wall to 2 mm above the bar: {out:?}"
    );
    assert!(
        (out.feed_mm_min.unwrap() - 150.).abs() < 1e-9,
        "at the finishing feed: {out:?}"
    );
    assert_eq!(
        finish_moves
            .iter()
            .filter(|m| m.kind == MoveKind::Retract)
            .count(),
        1
    );
    // Only then the rapids, and every one of them stays a clearance above
    // the bar it was (22.5 mm): none runs along the wall. Stated from the
    // bar's own radius, not from the planner's stock.
    let after = &finish_moves[at + 1..];
    assert!(!after.is_empty() && after.iter().all(|m| m.kind == MoveKind::Rapid));
    for rapid in after {
        for p in line(rapid) {
            assert!(p[0] >= 24.5 - 1e-9, "{rapid:?}");
        }
    }
    // No rapid of the program goes behind the bound, where the bar stands.
    for event in &plan.events {
        let Event::Motion { motion } = event else {
            continue;
        };
        if motion.kind == MoveKind::Rapid {
            for p in line(motion) {
                assert!(p[1] >= -40.8 - 1e-9, "{motion:?}");
            }
        }
    }
    // What is left: the two diameters at size, the bar behind the bound as
    // it was (the feed out took nothing off it), no gouge.
    radius_is(&plan.stock, -18.5, -1.5, 10.);
    radius_is(&plan.stock, -39.5, -21.5, 15.);
    radius_is(&plan.stock, -77., -41., 22.5);
    let report = material_report(&first, &plan.stock).unwrap();
    assert_eq!(report["gouge"]["definite_cells"], 0);
    assert_eq!(
        plan.reports[2].left_for_another_setup_z_mm,
        vec![[-60., -40.]]
    );

    // The second chucking starts from that stock: turned around, held on
    // the finished 20 mm diameter, the 17 mm the jaws held faced off and
    // the 24 mm end turned up to the collar.
    let left = plan.stock.carried(true, -60.).unwrap();
    let second = job(
        serde_json::to_value(&left).unwrap(),
        &B_SIDE,
        -45.,
        Some([15., 10.]),
        json!([
            face(),
            between(rough(), None, Some(-20.)),
            between(finish(), None, Some(-20.))
        ]),
    );
    let turned = planned(&second).unwrap();
    radius_is(&turned.stock, -18.5, -0.5, 12.);
    radius_is(&turned.stock, -39.5, -20.5, 15.);
    radius_is(&turned.stock, -59.5, -41.5, 10.);
    let report = material_report(&second, &turned.stock).unwrap();
    assert_eq!(report["gouge"]["definite_cells"], 0);
    assert_eq!(
        report["held"],
        json!({"z_mm": [-60., -45.], "definite_cells": 0, "possible_only_cells": 0})
    );

    // Where no stock stands beside the nose a clearance out, the exit is
    // the one clearance it always was: a bar 1 mm over the collar, and an
    // operation without bounds.
    for (stock, operations) in [
        (shelf_bar(16.), to_collar()),
        (shelf_bar(22.5), json!([face(), rough(), finish()])),
    ] {
        let short = [A_SIDE[0], A_SIDE[1], A_SIDE[2], A_SIDE[3], A_SIDE[4]];
        let bounded = operations[2].get("z_to_mm").is_some();
        let profile: &[[f64; 3]] = if bounded { &A_SIDE } else { &short };
        let plan = planned(&job(stock, profile, -67., None, operations)).unwrap();
        let retracts: Vec<_> = motions(&plan, "finish")
            .into_iter()
            .filter(|m| m.kind == MoveKind::Retract)
            .collect();
        assert_eq!(retracts.len(), 1);
        let [from, to] = line(retracts[0]);
        assert!(
            to[1] == from[1] && (from[0] - 15.).abs() < 1e-9 && (to[0] - 17.).abs() < 1e-9,
            "{:?}",
            retracts[0]
        );
    }

    // Without bounds too: the jaws on a bar well over the part, the path
    // ending at the chuck clearance against the bar that stands there.
    let held = job(
        json!({
            "version": 1, "units": "mm", "z_min_mm": -62.0, "z_max_mm": 2.0,
            "outer_radius_mm": 22.5, "cell_mm": 0.25, "max_cells": 10000,
        }),
        &LONG,
        -44.,
        Some([18., 22.5]),
        json!([face(), rough(), finish()]),
    );
    let plan = planned(&held).unwrap();
    let retract = motions(&plan, "finish")
        .into_iter()
        .find(|m| m.kind == MoveKind::Retract)
        .unwrap();
    let [from, to] = line(retract);
    assert!(
        (from[1] + 42.8).abs() < 1e-9 && to[1] == from[1] && (to[0] - 24.5).abs() < 1e-9,
        "{retract:?}"
    );
}
