//! #445: an arc lowered to chords from a decoded start that the block before
//! moved within the output tolerance (a native arc's exactly consistent
//! words move its end along the word grid). The uniform first chord, sized
//! for the chord tolerance alone, is charged with that offset too and was
//! refused ("continuous lowering error ... exceeds output tolerance"), so an
//! adaptive pocket posted with native arcs could not be posted at all. A
//! shorter lead chord now takes the offset; every interval is still
//! certified independently within the output tolerance.
use super::normal_limit_tests::{fixture, initialize};
use super::*;
use crate::motion::{Curve, Motion, MoveKind, Plane};
use std::f64::consts::FRAC_PI_2;

fn motion(id: u64, curve: Curve) -> Event {
    Event::Motion {
        motion: Motion {
            id,
            operation: "op".into(),
            tool: "t1".into(),
            kind: MoveKind::Cut,
            curve,
            feed_mm_min: Some(600.),
        },
    }
}
/// The quarter arc of radius 5 from (10, 0) clockwise to (15, -5) at Z5.
fn quarter() -> Event {
    motion(
        2,
        Curve::Arc {
            start: [10., 0., 5.],
            end: [15., -5., 5.],
            center: [10., -5., 5.],
            plane: Plane::Xy,
            sweep_rad: -FRAC_PI_2,
        },
    )
}
/// A session whose decoded position is `offset` mm from (10, 0, 5) along
/// +Y: the line there was decoded ending that far from its planned end, as
/// a block whose words moved its end within the output tolerance is.
fn decoded_at(offset: f64) -> IntentChecker {
    let (mut c, init, _) = fixture(None, true);
    initialize(&mut c, &init, true);
    let line = motion(
        1,
        Curve::Line {
            start: [2., 0., 5.],
            end: [10., 0., 5.],
        },
    );
    c.start(line.clone(), &|| false).unwrap();
    let mut actual = line;
    if let Event::Motion { motion } = &mut actual {
        motion.curve = Curve::Line {
            start: [2., 0., 5.],
            end: [10., offset, 5.],
        };
    }
    c.check_lowered(
        &[[0., 1.]],
        &[vec![actual]],
        &[vec!["LINE".into()]],
        &|| false,
    )
    .unwrap();
    assert_eq!(c.decoded.summary.final_position_mm, [10., offset, 5.]);
    c
}
/// Lower the queued source through the session, the decoded chords being
/// the planned ones from the decoded start. Returns each interval and its
/// certified bound.
fn lower(c: &mut IntentChecker) -> Result<Vec<([f64; 2], f64)>, String> {
    let mut out = vec![];
    while c.has_pending() {
        let preview = c.preview(64, &|| false)?;
        let ranges: Vec<_> = preview.iter().map(|p| p.source_fraction_interval).collect();
        let mut at = c.decoded.summary.final_position_mm;
        let actual: Vec<_> = preview
            .into_iter()
            .map(|p| {
                let mut e = p.event;
                if let Event::Motion { motion } = &mut e {
                    motion.curve = Curve::Line {
                        start: at,
                        end: motion.curve.end(),
                    };
                    at = motion.curve.end();
                }
                vec![e]
            })
            .collect();
        let lines = vec![vec!["CHORD".to_string()]; ranges.len()];
        let checked = c.check_lowered(&ranges, &actual, &lines, &|| false)?;
        for m in checked.mappings {
            out.push((
                m.source_fraction_interval.unwrap(),
                m.deviation_bound_mm.unwrap(),
            ));
        }
    }
    Ok(out)
}

#[test]
fn an_arc_lowered_from_a_moved_start_leads_with_a_shorter_chord() {
    let uniform = crate::post_lowering::segment_count(
        match &quarter() {
            Event::Motion { motion } => &motion.curve,
            _ => unreachable!(),
        },
        crate::post_lowering::Options::default().chord_tolerance_mm,
    )
    .unwrap();
    assert_eq!(uniform, 25);
    // The uniform first chord from a start 0.008 mm off: its sagitta and
    // the offset together exceed the 0.01 mm output tolerance. This is the
    // interval the session planned before, and its certificate refuses it.
    let Event::Motion { motion: source } = quarter() else {
        unreachable!()
    };
    let first = 1. / uniform as f64;
    let mut chord = source.clone();
    chord.curve = Curve::Line {
        start: [10., 0.008, 5.],
        end: source.curve.prepare(1e-6).unwrap().position(first).unwrap(),
    };
    let bound = crate::post_lowering::certify(&source, [0., first], &chord, 1e-6).unwrap();
    assert!(bound > 0.01, "{bound}");

    let mut c = decoded_at(0.008);
    c.start(quarter(), &|| false).unwrap();
    let lowered = lower(&mut c).unwrap();
    // One lead chord, then the uniform plan from its first boundary on.
    assert_eq!(lowered.len(), uniform as usize + 1);
    let lead = lowered[0].0[1];
    assert_eq!(lowered[0].0[0], 0.);
    assert!(lead > 0. && lead < first, "{lead}");
    assert_eq!(lowered[1].0, [lead, first]);
    for (k, (range, _)) in lowered.iter().enumerate().skip(2) {
        assert_eq!(*range, [(k - 1) as f64 / 25., k as f64 / 25.]);
    }
    assert_eq!(lowered.last().unwrap().0[1], 1.);
    // Every interval certified within the output tolerance, the lead
    // carrying the offset.
    for (range, bound) in &lowered {
        assert!(*bound <= 0.01, "{range:?}: {bound}");
    }
    assert!(lowered[0].1 >= 0.008, "{:?}", lowered[0]);
}

#[test]
fn an_arc_lowered_from_its_own_start_keeps_the_uniform_plan() {
    // A start within rounding of the planned one leaves the plan as it was.
    for offset in [0., 0.0005] {
        let mut c = decoded_at(offset);
        c.start(quarter(), &|| false).unwrap();
        let lowered = lower(&mut c).unwrap();
        assert_eq!(lowered.len(), 25);
        for (k, (range, bound)) in lowered.iter().enumerate() {
            assert_eq!(*range, [k as f64 / 25., (k + 1) as f64 / 25.]);
            assert!(*bound <= 0.01);
        }
    }
}

#[test]
fn a_curve_queued_behind_another_keeps_the_uniform_plan() {
    // Its start is where the source before it ends, not known when it is
    // queued: no lead is planned from a guess, and the certificate still
    // judges the uniform first chord from wherever the chain arrives.
    let (mut c, init, _) = fixture(None, true);
    initialize(&mut c, &init, true);
    let line = motion(
        1,
        Curve::Line {
            start: [2., 0., 5.],
            end: [10., 0., 5.],
        },
    );
    c.start(line, &|| false).unwrap();
    c.start(quarter(), &|| false).unwrap();
    let lowered = lower(&mut c).unwrap();
    assert_eq!(lowered.len(), 26);
    assert_eq!(lowered[1].0, [0., 1. / 25.]);
}
