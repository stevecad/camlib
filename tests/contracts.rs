use camlib::motion::*;
use camlib::tool::{Cutter, CutterShape, HolderSection, Neck, ToolAssembly};
use std::f64::consts::{FRAC_PI_2, PI, TAU};

fn cutter(shape: CutterShape) -> Cutter {
    Cutter {
        id: "em6".into(),
        name: "6 mm cutter".into(),
        shape,
        diameter_mm: 6.,
        cutting_length_mm: 18.,
        overall_length_mm: 50.,
        shank_diameter_mm: 6.,
        flutes: 3,
        center_cutting: true,
        material: "carbide".into(),
        neck: None,
    }
}
fn assembly() -> ToolAssembly {
    ToolAssembly {
        id: "t1".into(),
        cutter: cutter(CutterShape::Flat),
        stickout_mm: 22.,
        gauge_length_mm: 70.,
        holders: vec![
            HolderSection {
                z_min_mm: 22.,
                z_max_mm: 40.,
                radius_min_mm: 10.,
                radius_max_mm: 15.,
            },
            HolderSection {
                z_min_mm: 40.,
                z_max_mm: 70.,
                radius_min_mm: 20.,
                radius_max_mm: 20.,
            },
        ],
    }
}
#[test]
fn holder_above_the_gauge_plane_keeps_geometry_and_actual_gauge_length() {
    let mut tool = assembly();
    tool.holders.last_mut().unwrap().z_max_mm = 80.;
    let prepared = tool.prepare().unwrap();
    assert_eq!(prepared.definition().gauge_length_mm, 70.);
    assert_eq!(prepared.radius_at(75.).unwrap().radius_mm, 20.);
    assert!(!prepared.radius_at(80.).unwrap().cutting);
    assert!(prepared.radius_at(81.).is_none());
    tool.holders.last_mut().unwrap().z_max_mm = 69.;
    assert!(tool.prepare().is_err());
}
fn caps() -> Capabilities {
    Capabilities {
        id: "test_profile".into(),
        planes: vec![Plane::Xy, Plane::Xz, Plane::Yz],
        arcs: true,
        helices: true,
        max_sweep_rad: TAU,
        max_feed_mm_min: 5000.,
        max_spindle_rpm: 24000.,
        max_tool_number: 99,
        max_length_register: 99,
        max_work_offset: 9,
        tool_changes: true,
        flood: true,
        mist: false,
    }
}
fn program() -> Program {
    let motion = |id, kind, curve, feed| Event::Motion {
        motion: Motion {
            id,
            operation: "pocket".into(),
            tool: "t1".into(),
            kind,
            curve,
            feed_mm_min: feed,
        },
    };
    Program {
        version: 1,
        id: "example".into(),
        revision: "input-sha-1".into(),
        units: "mm".into(),
        tolerance_mm: 1e-8,
        frame: WorkFrame::identity(),
        initial_position_mm: [1., 0., 5.],
        tools: vec![assembly()],
        events: vec![
            Event::WorkOffset { number: 1 },
            Event::ToolChange {
                tool: "t1".into(),
                number: 1,
                length_register: 1,
                position_after_mm: [1., 0., 5.],
            },
            motion(
                0,
                MoveKind::Rapid,
                Curve::Line {
                    start: [1., 0., 5.],
                    end: [1., 0., 0.],
                },
                None,
            ),
            Event::Spindle {
                rpm: 12000.,
                direction: SpindleDirection::Cw,
            },
            Event::Coolant {
                mode: Coolant::Flood,
            },
            motion(
                1,
                MoveKind::Cut,
                Curve::Arc {
                    start: [1., 0., 0.],
                    end: [0., 1., 0.],
                    center: [0.; 3],
                    plane: Plane::Xy,
                    sweep_rad: FRAC_PI_2,
                },
                Some(1200.),
            ),
            motion(
                2,
                MoveKind::Plunge,
                Curve::Helix {
                    start: [0., 1., 0.],
                    end: [0., 1., -2.],
                    center: [0.; 3],
                    plane: Plane::Xy,
                    sweep_rad: TAU,
                },
                Some(300.),
            ),
            Event::Spindle {
                rpm: 0.,
                direction: SpindleDirection::Cw,
            },
            Event::Coolant { mode: Coolant::Off },
        ],
    }
}
fn close(a: f64, b: f64) {
    assert!((a - b).abs() < 1e-10, "{a} != {b}");
}

#[test]
fn clearance_feed_is_typed_bounded_and_does_not_require_a_spindle() {
    let mut p = program();
    p.events.truncate(3);
    for feed in [None, Some(600.), Some(5000.)] {
        let Event::Motion { motion } = &mut p.events[2] else {
            unreachable!()
        };
        motion.feed_mm_min = feed;
        let s = p.validate(Some(&caps())).unwrap();
        close(s.rapid_length_mm, 5.);
        close(s.fed_length_mm, 0.);
    }
    for feed in [0., -1., f64::NAN, f64::INFINITY, 5000.001] {
        let Event::Motion { motion } = &mut p.events[2] else {
            unreachable!()
        };
        motion.feed_mm_min = Some(feed);
        assert!(p.validate(Some(&caps())).is_err(), "{feed}");
    }
    let Event::Motion { motion } = &mut p.events[2] else {
        unreachable!()
    };
    motion.feed_mm_min = Some(600.);
    motion.curve = Curve::Arc {
        start: [1., 0., 5.],
        end: [0., 1., 5.],
        center: [0., 0., 5.],
        plane: Plane::Xy,
        sweep_rad: FRAC_PI_2,
    };
    assert!(p.validate(Some(&caps())).unwrap_err().contains("line"));
}

#[test]
fn analytic_cutting_profiles_and_noncutting_regions() {
    let flat = cutter(CutterShape::Flat).prepare().unwrap();
    for z in [0., 5., 18.] {
        assert!(flat.radius_at(z).unwrap().cutting);
        close(flat.radius_at(z).unwrap().radius_mm, 3.);
    }
    assert!(!flat.radius_at(19.).unwrap().cutting);
    let ball = cutter(CutterShape::Ball).prepare().unwrap();
    close(ball.radius_at(0.).unwrap().radius_mm, 0.);
    close(ball.radius_at(1.).unwrap().radius_mm, 5_f64.sqrt());
    close(ball.radius_at(3.).unwrap().radius_mm, 3.);
    let bull = cutter(CutterShape::Bull {
        corner_radius_mm: 1.,
    })
    .prepare()
    .unwrap();
    close(bull.radius_at(0.).unwrap().radius_mm, 2.);
    close(bull.radius_at(0.5).unwrap().radius_mm, 2. + 0.75_f64.sqrt());
    let drill = cutter(CutterShape::Drill {
        point_angle_rad: FRAC_PI_2,
    })
    .prepare()
    .unwrap();
    close(drill.radius_at(1.).unwrap().radius_mm, 1.);
    let chamfer = cutter(CutterShape::Chamfer {
        included_angle_rad: FRAC_PI_2,
        tip_diameter_mm: 1.,
    })
    .prepare()
    .unwrap();
    close(chamfer.radius_at(0.).unwrap().radius_mm, 0.5);
    close(chamfer.radius_at(1.).unwrap().radius_mm, 1.5);
    for z in [-1., 51., f64::NAN] {
        assert!(flat.radius_at(z).is_none());
    }
    let mut necked = cutter(CutterShape::Flat);
    necked.neck = Some(Neck {
        diameter_mm: 4.,
        length_mm: 8.,
    });
    let n = necked.prepare().unwrap();
    assert!(!n.radius_at(22.).unwrap().cutting);
    close(n.radius_at(22.).unwrap().radius_mm, 2.);
    close(n.radius_at(27.).unwrap().radius_mm, 3.);
}

#[test]
fn invalid_cutters_and_holder_envelopes_fail_explicitly() {
    for invalid in [f64::NAN, f64::INFINITY, 0., -1.] {
        let mut c = cutter(CutterShape::Flat);
        c.diameter_mm = invalid;
        assert!(c.prepare().is_err());
    }
    let mut c = cutter(CutterShape::Ball);
    c.cutting_length_mm = 2.;
    assert!(c.prepare().is_err());
    assert!(
        cutter(CutterShape::Bull {
            corner_radius_mm: 4.
        })
        .prepare()
        .is_err()
    );
    assert!(
        cutter(CutterShape::Drill {
            point_angle_rad: PI
        })
        .prepare()
        .is_err()
    );
    assert!(
        cutter(CutterShape::Chamfer {
            included_angle_rad: FRAC_PI_2,
            tip_diameter_mm: 7.
        })
        .prepare()
        .is_err()
    );
    let mut c = cutter(CutterShape::Flat);
    c.flutes = 0;
    assert!(c.prepare().is_err());
    c = cutter(CutterShape::Flat);
    c.overall_length_mm = 18.;
    assert!(c.prepare().is_err());
    let mut t = assembly();
    t.holders.clear();
    assert!(t.prepare().is_err());
    t = assembly();
    t.stickout_mm = 17.;
    assert!(t.prepare().is_err());
    t = assembly();
    t.holders[1].z_min_mm = 41.;
    assert!(t.prepare().is_err());
    t = assembly();
    t.holders[0].radius_min_mm = 1.;
    assert!(t.prepare().is_err());
    t = assembly();
    t.holders[1].z_max_mm = 65.;
    assert!(t.prepare().is_err());
}

#[test]
fn assembled_profile_uses_tip_reference_and_joint_outer_envelope() {
    let t = assembly().prepare().unwrap();
    assert!(t.radius_at(10.).unwrap().cutting);
    assert!(!t.radius_at(20.).unwrap().cutting);
    close(t.radius_at(22.).unwrap().radius_mm, 10.);
    close(t.radius_at(31.).unwrap().radius_mm, 12.5);
    close(t.radius_at(40.).unwrap().radius_mm, 20.);
    close(t.radius_at(70.).unwrap().radius_mm, 20.);
    assert!(t.radius_at(70.1).is_none());
}

#[test]
fn oriented_arc_planes_have_expected_quadrants() {
    for (plane, start, end, expected) in [
        (
            Plane::Xy,
            [1., 0., 0.],
            [0., 1., 0.],
            [0.5_f64.sqrt(), 0.5_f64.sqrt(), 0.],
        ),
        (
            Plane::Xz,
            [0., 0., 1.],
            [1., 0., 0.],
            [0.5_f64.sqrt(), 0., 0.5_f64.sqrt()],
        ),
        (
            Plane::Yz,
            [0., 1., 0.],
            [0., 0., 1.],
            [0., 0.5_f64.sqrt(), 0.5_f64.sqrt()],
        ),
    ] {
        let curve = Curve::Arc {
            start,
            end,
            center: [0.; 3],
            plane,
            sweep_rad: FRAC_PI_2,
        }
        .prepare(1e-9)
        .unwrap();
        close(curve.length_mm(), FRAC_PI_2);
        for (a, b) in curve.position(0.5).unwrap().into_iter().zip(expected) {
            close(a, b);
        }
        assert_eq!(curve.position(0.).unwrap(), start);
        assert_eq!(curve.position(1.).unwrap(), end);
    }
}

#[test]
fn long_clockwise_arc_and_multiturn_helix_have_continuous_bounds() {
    let c = Curve::Arc {
        start: [1., 0., 0.],
        end: [0., 1., 0.],
        center: [0.; 3],
        plane: Plane::Xy,
        sweep_rad: -3. * FRAC_PI_2,
    }
    .prepare(1e-9)
    .unwrap();
    close(c.length_mm(), 3. * FRAC_PI_2);
    assert_eq!(c.bounds().min, [-1., -1., 0.]);
    assert_eq!(c.bounds().max, [1., 1., 0.]);
    let h = Curve::Helix {
        start: [1., 0., 2.],
        end: [1., 0., -3.],
        center: [0., 0., 2.],
        plane: Plane::Xy,
        sweep_rad: 2. * TAU,
    }
    .prepare(1e-9)
    .unwrap();
    close(h.length_mm(), (2. * TAU).hypot(5.));
    let mid = h.position(0.25).unwrap();
    close(mid[0], -1.);
    close(mid[1], 0.);
    close(mid[2], 0.75);
    for i in 0..1001 {
        let p = h.position(i as f64 / 1000.).unwrap();
        for (k, v) in p.into_iter().enumerate() {
            assert!(v >= h.bounds().min[k] - 1e-12 && v <= h.bounds().max[k] + 1e-12);
        }
    }
    assert!(h.position(-0.1).is_err());
}

#[test]
fn circular_bounds_include_the_nominal_endpoint_within_input_tolerance() {
    for plane in [Plane::Xy, Plane::Xz, Plane::Yz] {
        let [u, v, n] = plane.axes();
        for direction in [-1., 1.] {
            for helical in [false, true] {
                let a0: f64 = if direction > 0. { 0.2 } else { 0.5 };
                let a1: f64 = if direction > 0. { 0.5 } else { 0.2 };
                let mut start = [0.; 3];
                start[u] = 10. * a0.cos();
                start[v] = 10. * a0.sin();
                let mut nominal = [0.; 3];
                nominal[u] = 10. * a1.cos();
                nominal[v] = 10. * a1.sin();
                nominal[n] = if helical { -3. } else { 0. };
                let mut end = nominal;
                // A valid declared endpoint displaced towards the start can
                // hide an extremum of the nominal path just before t=1.
                end[u] += direction * 0.02;
                end[v] -= direction * 0.02;
                let curve = if helical {
                    Curve::Helix {
                        start,
                        end,
                        center: [0.; 3],
                        plane,
                        sweep_rad: a1 - a0,
                    }
                } else {
                    Curve::Arc {
                        start,
                        end,
                        center: [0.; 3],
                        plane,
                        sweep_rad: a1 - a0,
                    }
                };
                let prepared = curve.prepare(0.05).unwrap();
                let bounds = prepared.bounds();
                let radial = x_extent(&curve);
                for point in [start, end, nominal, prepared.position(0.999999).unwrap()] {
                    assert!(point[0] >= radial[0] - 1e-12 && point[0] <= radial[1] + 1e-12);
                    for axis in 0..3 {
                        assert!(
                            point[axis] >= bounds.min[axis] - 1e-12
                                && point[axis] <= bounds.max[axis] + 1e-12,
                            "{plane:?} {direction} {helical}: {point:?} outside {bounds:?}"
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn nominal_arc_endpoint_cannot_leave_the_supported_coordinate_range() {
    let end_angle: f64 = 0.1;
    let curve = Curve::Arc {
        start: [999_999.9, 0.12, 0.],
        end: [999_999.99, 0.12 * end_angle.sin(), 0.],
        center: [999_999.9, 0., 0.],
        plane: Plane::Xy,
        sweep_rad: end_angle - FRAC_PI_2,
    };
    // All declared coordinates fit, but the actual arc reaches x > 1e6.
    assert!(999_999.9 + 0.12 * end_angle.cos() > 1e6);
    assert!(
        curve
            .prepare(0.05)
            .err()
            .unwrap()
            .contains("coordinate range")
    );
}

#[test]
fn css_speed_bound_includes_the_nominal_radial_endpoint() {
    let start_angle: f64 = 0.5;
    let end_angle: f64 = 0.2;
    let curve = Curve::Arc {
        start: [10. * start_angle.sin(), 0., 10. * start_angle.cos()],
        end: [
            10. * end_angle.sin() + 0.02,
            0.,
            10. * end_angle.cos() - 0.02,
        ],
        center: [0.; 3],
        plane: Plane::Xz,
        sweep_rad: end_angle - start_angle,
    };
    curve.prepare(0.05).unwrap();
    let minimum_radius = 10. * end_angle.sin();
    assert!(x_extent(&curve)[0] <= minimum_radius + 1e-12);
    let expected_rpm = 100_000. / (TAU * minimum_radius);
    assert!(css_rpm_bound(&curve, 100., 12000.) >= expected_rpm - 1e-9);
}

#[test]
fn malformed_motion_is_rejected() {
    assert!(
        Curve::Line {
            start: [0.; 3],
            end: [0.; 3]
        }
        .prepare(1e-6)
        .is_err()
    );
    for (end, sweep) in [
        ([0., 2., 0.], FRAC_PI_2),
        ([0., 1., 1.], FRAC_PI_2),
        ([0., 1., 0.], -FRAC_PI_2),
        ([1., 0., 0.], 0.),
    ] {
        assert!(
            Curve::Arc {
                start: [1., 0., 0.],
                end,
                center: [0.; 3],
                plane: Plane::Xy,
                sweep_rad: sweep
            }
            .prepare(1e-6)
            .is_err()
        );
    }
    assert!(
        Curve::Helix {
            start: [1., 0., 0.],
            end: [0., 1., 0.],
            center: [0.; 3],
            plane: Plane::Xy,
            sweep_rad: FRAC_PI_2
        }
        .prepare(1e-6)
        .is_err()
    );
    assert!(
        Curve::Line {
            start: [0.; 3],
            end: [f64::NAN, 0., 1.]
        }
        .prepare(1e-6)
        .is_err()
    );
}

#[test]
fn program_round_trip_and_lengths_are_independent_of_post_formatting() {
    let p = program();
    let decoded: Program = serde_json::from_slice(&serde_json::to_vec(&p).unwrap()).unwrap();
    assert_eq!(p, decoded);
    let s = p.validate(Some(&caps())).unwrap();
    assert_eq!(s.motions, 3);
    close(s.rapid_length_mm, 5.);
    close(s.fed_length_mm, FRAC_PI_2 + TAU.hypot(2.));
    assert_eq!(s.final_position_mm, [0., 1., -2.]);
}

#[test]
fn program_detects_gaps_missing_state_and_wrong_tools() {
    for variant in 0..6 {
        let mut p = program();
        match variant {
            0 => {
                if let Event::Motion { motion: m } = &mut p.events[5] {
                    m.curve = Curve::Line {
                        start: [9., 0., 0.],
                        end: [0., 1., 0.],
                    };
                }
            }
            1 => {
                p.events.remove(3);
            }
            2 => {
                p.events.remove(0);
            }
            3 => {
                if let Event::Motion { motion: m } = &mut p.events[5] {
                    m.feed_mm_min = None;
                }
            }
            4 => {
                if let Event::Motion { motion: m } = &mut p.events[5] {
                    m.tool = "missing".into();
                }
            }
            _ => {
                if let Event::Motion { motion: m } = &mut p.events[6] {
                    m.id = 0;
                }
            }
        }
        assert!(p.validate(None).is_err(), "variant {variant}");
    }
    let mut p = program();
    p.events.push(Event::WorkOffset { number: 2 });
    assert!(p.validate(None).is_err());
    let mut p = program();
    p.events.insert(
        5,
        Event::ToolChange {
            tool: "t1".into(),
            number: 1,
            length_register: 1,
            position_after_mm: [1., 0., 0.],
        },
    );
    assert!(p.validate(None).is_err());
}

#[test]
fn controller_capabilities_reject_unsupported_or_excessive_motion() {
    for variant in 0..6 {
        let mut c = caps();
        match variant {
            0 => {
                c.arcs = false;
                c.helices = false;
            }
            1 => c.helices = false,
            2 => c.max_sweep_rad = 1.,
            3 => c.max_feed_mm_min = 100.,
            4 => c.max_spindle_rpm = 1000.,
            _ => c.flood = false,
        }
        assert!(program().validate(Some(&c)).is_err(), "variant {variant}");
    }
    let mut p = program();
    p.frame.x = [1., 0., 0.];
    p.frame.y = [0., 0., 1.];
    p.frame.z = [0., -1., 0.];
    assert!(p.validate(None).is_ok());
    assert!(p.validate(Some(&caps())).is_err());
}

#[test]
fn work_frame_is_rigid_and_right_handed() {
    let mut frame = WorkFrame {
        origin_mm: [10., 20., 30.],
        x: [0., 1., 0.],
        y: [-1., 0., 0.],
        z: [0., 0., 1.],
    };
    frame.validate().unwrap();
    assert_eq!(frame.to_design([2., 3., 4.]), [7., 22., 34.]);
    frame.z = [0., 0., -1.];
    assert!(frame.validate().is_err());
}

/// A lathe-like program at a feed per revolution (#429): spindle at 2500
/// rpm, feed per revolution `mm_rev`, one cut of the operation "face".
fn per_rev(mm_rev: f64) -> Program {
    let mut p = program();
    p.events.truncate(3);
    p.events.push(Event::Spindle {
        rpm: 2500.,
        direction: SpindleDirection::Cw,
    });
    p.events.push(Event::FeedPerRev { mm_rev });
    p.events.push(Event::Motion {
        motion: Motion {
            id: 1,
            operation: "face".into(),
            tool: "t1".into(),
            kind: MoveKind::Cut,
            curve: Curve::Line {
                start: [1., 0., 0.],
                end: [5., 0., 0.],
            },
            feed_mm_min: Some(mm_rev * 2500.),
        },
    });
    p
}

#[test]
fn machine_limits_are_the_declared_ones_and_their_refusals_name_them() {
    // A feed per revolution has no bound of camlib's own: 100 mm/rev (a
    // feed per minute typed into a per-revolution field) is a valid number,
    // and so is a thread lead of 150 mm.
    for mm_rev in [0.25, 100., 150.] {
        per_rev(mm_rev).validate(None).unwrap();
    }
    for bad in [-0.1, f64::NAN, f64::INFINITY] {
        let error = per_rev(bad).validate(None).unwrap_err();
        assert!(
            error.contains("must be finite and not negative"),
            "{bad}: {error}"
        );
    }
    // Against a machine: what the feed is made of, its value in mm/min, the
    // declared maximum and its field, and the operation; the locators after.
    let mut c = caps();
    c.max_feed_mm_min = 10000.;
    per_rev(4.).validate(Some(&c)).unwrap();
    assert_eq!(
        per_rev(100.).validate(Some(&c)).unwrap_err(),
        "face: the programmed feed, the feed per revolution 100 mm/rev at 2500 rpm (the highest along the move), 250000 mm/min, is above the machine's maximum feed 10000 mm/min (Machine.max_feed_mm_min) (move 1, event 5)"
    );
    // A feed per minute over the maximum.
    let mut p = program();
    p.events.truncate(6);
    let Event::Motion { motion } = &mut p.events[5] else {
        unreachable!()
    };
    motion.feed_mm_min = Some(6000.);
    assert_eq!(
        p.validate(Some(&caps())).unwrap_err(),
        "pocket: the programmed feed, 6000 mm/min, is above the machine's maximum feed 5000 mm/min (Machine.max_feed_mm_min) (move 1, event 5)"
    );
    // The spindle above the declared top speed names the operation it
    // starts.
    let mut c = caps();
    c.max_spindle_rpm = 10000.;
    assert_eq!(
        program().validate(Some(&c)).unwrap_err(),
        "pocket: spindle speed 12000 rpm is above the machine's maximum 10000 rpm (Machine.max_spindle_rpm) (event 3)"
    );
    // Coolant, tool table and circular motion, each by its field.
    let mut c = caps();
    c.flood = false;
    assert!(
        program()
            .validate(Some(&c))
            .unwrap_err()
            .starts_with("pocket: flood coolant is programmed, but the machine has none (Machine.flood is false)"),
    );
    let mut p = program();
    if let Event::ToolChange { number, .. } = &mut p.events[1] {
        *number = 12;
    }
    let mut c = caps();
    c.max_tool_number = 8;
    assert_eq!(
        p.validate(Some(&c)).unwrap_err(),
        "pocket: tool number 12 (tool \"t1\") is above the machine's highest tool number 8 (Machine.max_tool_number) (event 1)"
    );
    let mut c = caps();
    c.max_sweep_rad = 1.;
    assert_eq!(
        program().validate(Some(&c)).unwrap_err(),
        format!(
            "pocket: circular motion unsupported by the machine: its sweep {} rad is above the machine's largest 1 rad in one block (Machine.max_sweep_rad); post it as checked line segments (LoweringOptions) (move 1, event 5)",
            FRAC_PI_2
        )
    );
}

#[test]
fn numeric_bounds_are_the_only_bounds_without_a_machine() {
    // Long dwells, large surface speeds and large compensation amounts are
    // numbers, not refusals: their limits are the machine's or the post's.
    let mut p = program();
    p.events.truncate(5);
    p.events.push(Event::Dwell { seconds: 2e5 });
    p.validate(None).unwrap();
    let mut p = program();
    p.events.truncate(4);
    p.events.push(Event::SurfaceSpeed {
        m_min: 2e5,
        max_rpm: 3000.,
        direction: SpindleDirection::Cw,
    });
    p.validate(None).unwrap();
    for bad in [0., -1., f64::NAN] {
        let mut p = program();
        p.events.truncate(5);
        p.events.push(Event::Dwell { seconds: bad });
        assert!(p.validate(None).unwrap_err().contains("positive finite"));
    }
}
