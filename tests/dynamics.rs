use camlib::{dynamics::*, motion::*};
use std::{
    cell::Cell,
    f64::consts::{PI, TAU},
};
fn settings(mode: JunctionMode) -> Settings {
    Settings {
        policy: Policy {
            tangential_accel_mm_s2: 100.,
            rapid_feed_mm_min: 6000.,
            junction_mode: mode,
            feed_reserve_fraction: 0.,
        },
        max_feed_mm_min: 10000.,
        max_normal_accel_mm_s2: None,
        max_axis_feed_mm_min: None,
        work_to_machine: WorkFrame::identity(),
        feed_quantum_mm_min: 0.001,
        adjust_feeds: true,
    }
}
fn program(curves: Vec<Curve>, feeds: Vec<f64>) -> Program {
    let tool=serde_json::from_value(serde_json::json!({"id":"t","cutter":{"id":"flat","name":"mill","shape":{"kind":"flat"},"diameter_mm":4.,"cutting_length_mm":20.,"overall_length_mm":40.,"shank_diameter_mm":4.,"flutes":2,"center_cutting":true,"material":"carbide"},"stickout_mm":25.,"gauge_length_mm":40.,"holders":[{"z_min_mm":25.,"z_max_mm":40.,"radius_min_mm":4.,"radius_max_mm":4.}]})).unwrap();
    let start = curves[0].start();
    let mut events = vec![
        Event::WorkOffset { number: 1 },
        Event::ToolChange {
            tool: "t".into(),
            number: 1,
            length_register: 1,
            position_after_mm: start,
        },
        Event::Spindle {
            rpm: 12000.,
            direction: SpindleDirection::Cw,
        },
    ];
    events.extend(
        curves
            .into_iter()
            .zip(feeds)
            .enumerate()
            .map(|(i, (curve, feed))| Event::Motion {
                motion: Motion {
                    id: (i + 1) as u64,
                    operation: "cut".into(),
                    tool: "t".into(),
                    kind: MoveKind::Cut,
                    curve,
                    feed_mm_min: Some(feed),
                },
            }),
    );
    Program {
        version: 1,
        id: "timing".into(),
        revision: "r1".into(),
        units: "mm".into(),
        tolerance_mm: 0.001,
        frame: WorkFrame::identity(),
        initial_position_mm: start,
        tools: vec![tool],
        events,
    }
}
fn line(a: f64, b: f64) -> Curve {
    Curve::Line {
        start: [a, 0., 0.],
        end: [b, 0., 0.],
    }
}
fn near(a: f64, b: f64) {
    assert!((a - b).abs() < 1e-9, "{a} != {b}");
}
#[test]
fn exact_stop_triangle_and_trapezoid_match_integrated_velocity() {
    for (length, expected) in [(1., 0.2), (20., 1.2)] {
        // a100, programmed20mm/s
        let p = program(vec![line(0., length)], vec![1200.]);
        let r = schedule(&p, settings(JunctionMode::ExactStop), &|| false).unwrap();
        // 20mm: two .2s ramps cover4mm, remaining16mm cruise .8s.
        near(r.motion_seconds, expected);
        assert_eq!(r.feed_limited_motions, 0);
    }
}
#[test]
fn lookahead_brakes_before_short_slow_block_and_recovers_at_tangent_join() {
    let p = program(
        vec![line(0., 10.), line(10., 10.1), line(10.1, 20.1)],
        vec![6000., 60., 6000.],
    );
    let r = schedule(&p, settings(JunctionMode::Tangent), &|| false).unwrap();
    near(r.segments[0].exit_mm_s, 1.);
    near(r.segments[1].entry_mm_s, 1.);
    near(r.segments[1].exit_mm_s, 1.);
    let a = schedule(
        &program(vec![line(0., 10.), line(10., 20.)], vec![1200.; 2]),
        settings(JunctionMode::Tangent),
        &|| false,
    )
    .unwrap();
    near(a.motion_seconds, 1.2);
    near(a.segments[0].exit_mm_s, 20.);
    let stops = schedule(
        &program(vec![line(0., 10.), line(10., 20.)], vec![1200.; 2]),
        settings(JunctionMode::ExactStop),
        &|| false,
    )
    .unwrap();
    near(stops.motion_seconds, 1.4);
}
#[test]
fn corners_reversals_state_events_and_rapid_boundaries_stop() {
    let mut p = program(vec![line(0., 10.), line(10., 0.)], vec![1200.; 2]);
    for variant in 0..4 {
        if variant == 1
            && let Event::Motion { motion } = &mut p.events[4]
        {
            motion.curve = Curve::Line {
                start: [10., 0., 0.],
                end: [10., 10., 0.],
            };
        }
        if variant == 2 {
            if let Event::Motion { motion } = &mut p.events[4] {
                motion.curve = line(10., 20.);
            }
            p.events.insert(4, Event::Stop { optional: true });
        }
        if variant == 3 {
            p.events.remove(4);
            if let Event::Motion { motion } = &mut p.events[4] {
                motion.kind = MoveKind::Rapid;
                motion.feed_mm_min = None;
            }
        }
        let r = schedule(&p, settings(JunctionMode::Tangent), &|| false).unwrap();
        near(r.segments[0].exit_mm_s, 0.);
        near(r.segments[1].entry_mm_s, 0.);
        if variant == 3 {
            assert!(r.segments[1].rapid_estimate);
            assert_eq!(r.segments[1].scheduled_feed_mm_min, None);
        }
    }
}
#[test]
fn arcs_helices_planes_and_rotated_axes_obey_independent_physical_caps() {
    for plane in [Plane::Xy, Plane::Xz, Plane::Yz] {
        let [u, v, w] = plane.axes();
        for sign in [-1., 1.] {
            for pitch in [0., 3.] {
                let mut start = [0.; 3];
                start[u] = 2.;
                let mut end = start;
                end[w] = pitch;
                let c = if pitch == 0. {
                    Curve::Arc {
                        start,
                        end,
                        center: [0.; 3],
                        plane,
                        sweep_rad: sign * TAU,
                    }
                } else {
                    Curve::Helix {
                        start,
                        end,
                        center: [0.; 3],
                        plane,
                        sweep_rad: sign * TAU,
                    }
                };
                let mut s = settings(JunctionMode::Tangent);
                s.max_normal_accel_mm_s2 = Some(10.);
                s.max_axis_feed_mm_min = Some([240., 300., 360.]);
                s.work_to_machine.x = [0., 1., 0.];
                s.work_to_machine.y = [-1., 0., 0.];
                let r = schedule(&program(vec![c.clone()], vec![6000.]), s, &|| false).unwrap();
                let row = &r.segments[0];
                let feed = row.scheduled_feed_mm_min.unwrap();
                let radius = 2.;
                let pitch_per_rad = pitch / TAU;
                let curvature = radius / (radius * radius + pitch_per_rad * pitch_per_rad);
                assert!((feed / 60.).powi(2) * curvature < 10.);
                let path = c.prepare(0.001).unwrap();
                let length = path.length_mm();
                for i in 0..=1000 {
                    let angle = sign * TAU * i as f64 / 1000.;
                    let mut vel = [0.; 3];
                    vel[u] = -2. * angle.sin() * sign * TAU / length * feed;
                    vel[v] = 2. * angle.cos() * sign * TAU / length * feed;
                    vel[w] = pitch / length * feed;
                    let machine = [-vel[1], vel[0], vel[2]];
                    for (v, limit) in machine.into_iter().zip([240., 300., 360.]) {
                        assert!(v.abs() <= limit);
                    }
                }
                assert_eq!(r.feed_limited_motions, 1);
            }
        }
    }
}
#[test]
fn tangent_arc_connection_and_profile_energy_are_continuous() {
    let p = program(
        vec![
            Curve::Line {
                start: [2., -5., 0.],
                end: [2., 0., 0.],
            },
            Curve::Arc {
                start: [2., 0., 0.],
                end: [-2., 0., 0.],
                center: [0.; 3],
                plane: Plane::Xy,
                sweep_rad: PI,
            },
        ],
        vec![300.; 2],
    );
    let r = schedule(&p, settings(JunctionMode::Tangent), &|| false).unwrap();
    near(r.segments[0].exit_mm_s, 5.);
    for s in r.segments {
        assert!(s.peak_mm_s <= s.speed_cap_mm_s);
        assert!(
            (s.exit_mm_s * s.exit_mm_s - s.entry_mm_s * s.entry_mm_s).abs()
                <= 200. * s.length_mm + 1e-9
        );
        assert!(s.seconds >= s.length_mm / s.speed_cap_mm_s - 1e-12);
    }
}
#[test]
fn cancellation_invalid_source_identity_and_check_only_have_no_partial_result() {
    let p = program(
        (0..1000).map(|i| line(i as f64, (i + 1) as f64)).collect(),
        vec![1200.; 1000],
    );
    let count = Cell::new(0);
    assert!(
        schedule(&p, settings(JunctionMode::Tangent), &|| {
            count.set(count.get() + 1);
            count.get() > 40
        })
        .unwrap_err()
        .contains("cancelled")
    );
    let s = settings(JunctionMode::Tangent);
    let id = schedule(&p, s.clone(), &|| false).unwrap().identity;
    let mut changed = p.clone();
    changed.revision = "r2".into();
    assert_ne!(
        id,
        schedule(&changed, s.clone(), &|| false).unwrap().identity
    );
    let mut changed_s = s.clone();
    changed_s.policy.tangential_accel_mm_s2 = 90.;
    assert_ne!(id, schedule(&p, changed_s, &|| false).unwrap().identity);
    let mut s = s;
    s.max_feed_mm_min = 100.;
    s.adjust_feeds = false;
    assert!(
        schedule(&p, s, &|| false)
            .unwrap_err()
            .contains("adjust_feeds")
    );
    changed.events[3] = Event::Motion {
        motion: Motion {
            id: 1,
            operation: "cut".into(),
            tool: "t".into(),
            kind: MoveKind::Cut,
            curve: line(2., 3.),
            feed_mm_min: Some(100.),
        },
    };
    assert!(schedule(&changed, settings(JunctionMode::Tangent), &|| false).is_err());
}

#[test]
fn dense_finish_junction_uses_the_same_unit_directions_on_every_host() {
    // The largest component difference between the exact unit tangents is
    // 63.95651 epsilons, just inside the existing 64-epsilon join threshold.
    let a = [16.91277459536761, -1.1, 6.129397685938713];
    let b = [16.699175699916843, -1.1, 6.2005973177556335];
    let c = [16.485576804466078, -1.1, 6.271796949572558];
    let p = program(
        vec![Curve::Line { start: a, end: b }, Curve::Line { start: b, end: c }],
        vec![600.; 2],
    );
    let report = schedule(&p, settings(JunctionMode::Tangent), &|| false).unwrap();
    assert!(report.segments[0].exit_mm_s > 0.);
    assert_eq!(report.segments[0].exit_mm_s, report.segments[1].entry_mm_s);
}
