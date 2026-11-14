use camlib::post_rounding::RadiusTolerance;
use camlib::{
    motion::{Curve, Plane},
    post_decode::{Config, Decoder, format_words},
    post_rounding::circular_words,
};
use std::f64::consts::TAU;

const ABS_0_005: RadiusTolerance = RadiusTolerance {
    absolute_mm: 0.005,
    relative: 0.,
};

fn decoder() -> Decoder {
    let c: Config = serde_json::from_value(serde_json::json!({
        "commands":[{"code":"G2","effect":"arc_cw"},{"code":"G3","effect":"arc_ccw"}],
        "axes":["X","Y","Z"],"centers":["I","J","K"],"feed_word":"F",
        "parentheses_comments":true,"radius_tolerance_mm":0.005
    }))
    .unwrap();
    Decoder::new(c).unwrap()
}

#[test]
fn real_short_arc_replays_from_actual_words_in_every_plane_unit_and_sense() {
    for fixture in [
        include_str!("../fixtures/motion/rounded-short-arc.json"),
        include_str!("../fixtures/motion/rounded-front-arc.json"),
    ] {
        check_representable_arc(serde_json::from_str(fixture).unwrap());
    }
}

fn check_representable_arc(original: Curve) {
    let Curve::Arc {
        start: s,
        end: e,
        center: c,
        sweep_rad: w,
        ..
    } = original
    else {
        panic!()
    };
    let decode = decoder();
    for (unit, decimals) in [(1., 3), (25.4, 4)] {
        let step = unit * 10f64.powi(-decimals);
        for (plane, [u, v, a]) in [
            (Plane::Xy, [0, 1, 2]),
            (Plane::Xz, [2, 0, 1]),
            (Plane::Yz, [1, 2, 0]),
        ] {
            for reverse in [false, true] {
                for helix in [false, true] {
                    let remap = |p: [f64; 3]| {
                        let mut q = [0.; 3];
                        q[u] = p[0];
                        q[v] = p[1];
                        q[a] = p[2];
                        q
                    };
                    let (mut start, mut end, mut center) = (remap(s), remap(e), remap(c));
                    if reverse {
                        std::mem::swap(&mut start, &mut end);
                    }
                    center[a] = start[a];
                    if helix {
                        end[a] += 0.8;
                    }
                    let sweep_rad = if reverse { -w } else { w };
                    let source = if helix {
                        Curve::Helix {
                            start,
                            end,
                            center,
                            plane,
                            sweep_rad,
                        }
                    } else {
                        Curve::Arc {
                            start,
                            end,
                            center,
                            plane,
                            sweep_rad,
                        }
                    };
                    let actual = start.map(|x| (x / step).round() * step);
                    let (target, offset) =
                        circular_words(&source, actual, unit, decimals as usize, 1e-5, 0.01, ABS_0_005, f64::INFINITY)
                            .unwrap_or_else(|e| panic!("unit{unit} plane{plane:?} reverse{reverse} helix{helix}: {source:?}: {e}"));
                    let mut values = vec![target[u], target[v]];
                    let mut addresses = String::new();
                    for k in [u, v] {
                        addresses.push(b"XYZ"[k] as char);
                    }
                    if helix {
                        addresses.push(b"XYZ"[a] as char);
                        values.push(target[a]);
                    }
                    for k in [u, v] {
                        addresses.push(b"IJK"[k] as char);
                        values.push(offset[k]);
                    }
                    addresses.push('F');
                    values.push(600.);
                    let words = format_words(&addresses, &values, unit, decimals as usize).unwrap();
                    let text = format!("{} {}", if reverse { "G2" } else { "G3" }, words.join(" "));
                    let decoded = decode
                        .decode(&text, actual, Some(unit), Some(plane))
                        .unwrap()
                        .unwrap();
                    let p = decoded.curve.prepare(1e-5).unwrap();
                    let src = source.prepare(1e-5).unwrap();
                    // Dense independent trajectory comparison is a regression oracle, not
                    // the authority for the analytic complete-interval bound in the checker.
                    for i in 0..=1000 {
                        let t = i as f64 / 1000.;
                        let x = p.position(t).unwrap();
                        let y = src.position(t).unwrap();
                        let distance = (0..3).map(|k| (x[k] - y[k]).powi(2)).sum::<f64>().sqrt();
                        assert!(distance <= 0.01, "{text}: {distance}");
                    }
                }
            }
        }
    }
}

#[test]
fn exact_full_turn_words_are_preserved_and_invalid_budgets_reject() {
    let source = Curve::Arc {
        start: [2., 0., 3.],
        end: [2., 0., 3.],
        center: [0., 0., 3.],
        plane: Plane::Xy,
        sweep_rad: TAU,
    };
    let actual = source.start();
    assert_eq!(
        circular_words(&source, actual, 1., 3, 1e-5, 0.01, ABS_0_005, f64::INFINITY).unwrap(),
        ([2., 0., 3.], [-2., 0., 0.])
    );
    assert!(circular_words(&source, actual, 1., 1, 1e-5, 0.01, ABS_0_005, f64::INFINITY).is_err());
    assert!(circular_words(&source, actual, 1., 3, 1e-5, f64::NAN, ABS_0_005, f64::INFINITY).is_err());
    let wrong = [2.1, 0., 3.];
    assert!(circular_words(&source, wrong, 1., 3, 1e-5, 0.001, ABS_0_005, f64::INFINITY).is_err());
}
