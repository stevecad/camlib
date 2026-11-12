//! The one rule for comparing a tool position with machine travel (#246).
//!
//! Every comparison of a position with machine travel is decided here, so no
//! two checks can disagree about a point on a limit:
//! - the plan check (`verification::check_travel`);
//! - the posted-output check (`post_intent::IntentChecker`);
//! - the kinematic machine model (`machine::Session`: decoded moves with
//!   their calibration interval, route waypoints, the start position).
//!
//! **The rule.** The travel box is closed: a position on a limit is inside
//! (a machine at home sits on its limits, often on three at once). A position
//! is outside when its machine coordinate, evaluated in binary64 from the
//! declared numbers, is beyond a limit by more than the bound on the error of
//! that evaluation. The declared numbers (positions, work offset, frame axes,
//! limits) are taken as exact; nothing else is allowed for:
//!
//! - **A declared position** (the ends of every move, a tool-change pose, the
//!   start pose) is exact. Its machine coordinate on axis `i` is evaluated by
//!   `WorkFrame::to_design` as
//!   `((o_i + p_x x_i) + p_y y_i) + p_z z_i`: three products and three sums,
//!   each rounded to nearest with relative error at most `u = 2^-53`. Every
//!   term passes through at most four roundings, so the computed coordinate
//!   differs from the exact one by at most `gamma_4 T`, where
//!   `T = |o_i| + |p_x x_i| + |p_y y_i| + |p_z z_i|` and
//!   `gamma_4 = 4u / (1 - 4u)` (Higham, *Accuracy and Stability of Numerical
//!   Algorithms*, 2nd ed., section 3.1: the error bound of an inner product
//!   of n terms is `gamma_n |x|^T |y|`). The guard is `5u T`, with `T` itself
//!   evaluated in binary64
//!   (`5u (1 - 4u) > gamma_4`), plus the smallest normal number for products
//!   that underflow. That is about 1e-13 mm at a metre of travel.
//! - **A tip limit derived from the gauge line** (`limit - gauge length`) is
//!   one more rounded subtraction: `u |limit|` is added on machine Z.
//! - **A point of the nominal circle of an arc or helix** (where a machine
//!   coordinate is stationary, and the nominal end) is not a declared
//!   position. The circle is drawn from the start about the centre, and the
//!   declared end may be off it by the move's endpoint gap, which the motion
//!   tolerance bounds (`Curve::prepare` refuses a larger one). `gap` is the
//!   distance between the nominal and the declared end of this move (not the
//!   tolerance), and `E` bounds the evaluation of a circle point (below).
//!   The declared ends of the arc stay exact. What the gap means depends on
//!   whose arc it is (`ArcGap`):
//!   - **A planned arc** is camlib's own representation: the nominal circle
//!     and the declared end describe one intended curve to within the gap,
//!     so a nominal point is outside only beyond `gap + E`. With no gap a
//!     nominal point is held to `E` alone, as exactly as a line.
//!   - **A decoded arc** is what a controller will execute, and controllers
//!     differ in how they reconcile a start radius and an end radius that
//!     disagree (one averages them, one spirals between them, one keeps the
//!     centre and corrects at the end). The executed path is only known to
//!     lie within the gap of the nominal circle, so the move is inside only
//!     when every nominal point is inside by the gap: it is refused when it
//!     is beyond `E - gap`. The gap shrinks what is admissible; it is never
//!     an allowance. A gap below `E` is not distinguishable from none.
//!
//! **With a calibration interval** (the kinematic machine model,
//! `crate::machine`, declares how far the work frame and each tool's gauge
//! offset may be from their expected values) a position is an interval on
//! each machine axis, and the rule is applied to the interval:
//!
//! - inside only if the whole interval is inside the closed travel;
//! - a violation only if the whole interval is beyond a limit;
//! - otherwise uncertain: the interval straddles the limit, and the finding
//!   states the interval and the limit.
//!
//! The interval is the declared calibration interval alone (and, for a
//! nominal point of a decoded arc, the arc's gap either side of it); no
//! tolerance enters it. Its two ends are evaluated with one more rounded sum
//! each, bounded like the others. Without an interval the three cases are
//! the two of the rule above: a position is inside or a violation.
//!
//! A machine coordinate of a move is extreme at an end or where it is
//! stationary, so a move is inside exactly when those points are: a line has
//! its two ends (a rigid transform keeps it a segment and the box is convex),
//! an arc or helix adds the stationary points of each machine axis
//! (`PreparedCurve::stationary_fractions`) in any work frame.
//!
//! Posted words are rounded to the machine's decimals. The posted-output
//! check applies this rule to the decoded positions, which are what the
//! controller executes: a decoded position beyond a limit is refused, however
//! small the step, because a controller stops on its travel limit (FANUC
//! stored stroke check 1, an overtravel alarm: 16i/18i-MB Operator's Manual
//! B-63534EN/02, section 6.3; LinuxCNC `MIN_LIMIT`/`MAX_LIMIT`, "when this
//! limit is exceeded, the controller aborts axis motion": INI configuration,
//! `[AXIS_<letter>]` section). Rounding is never given a tolerance: a plan on
//! a limit whose word rounds beyond it is refused, with the remedy named.

use crate::motion::{Curve, PreparedCurve, Vec3, WorkFrame};

/// Unit roundoff of binary64 round-to-nearest.
const U: f64 = f64::EPSILON / 2.;

/// The travel of a tool's tip in machine coordinates.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Travel {
    pub min: Vec3,
    pub max: Vec3,
    /// The Z limits were computed from the gauge line (one rounding each).
    lowered: bool,
}

impl Travel {
    /// A declared tip box.
    pub fn tip(min: Vec3, max: Vec3) -> Self {
        Self {
            min,
            max,
            lowered: false,
        }
    }
    /// The gauge line's box lowered along machine Z by a tool's gauge length:
    /// the length offset moves the controlled point from the gauge line to
    /// the tip (G43).
    pub fn gauge_line(min: Vec3, max: Vec3, gauge_length_mm: f64) -> Self {
        Self {
            min: [min[0], min[1], min[2] - gauge_length_mm],
            max: [max[0], max[1], max[2] - gauge_length_mm],
            lowered: true,
        }
    }
}

/// A calibration interval on the machine position (the kinematic machine
/// model, `crate::machine`): the machine coordinate is the transformed
/// position plus `offset_mm` (the expected gauge offset), and the true
/// position lies anywhere in `[coordinate + low_mm, coordinate + high_mm]`
/// (`low_mm <= 0 <= high_mm`, per machine axis). The interval is declared
/// calibration uncertainty, never a tolerance.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Calibration {
    pub offset_mm: Vec3,
    pub low_mm: Vec3,
    pub high_mm: Vec3,
}

impl Calibration {
    /// An exactly known position: the interval is a point.
    pub const NONE: Self = Self {
        offset_mm: [0.; 3],
        low_mm: [0.; 3],
        high_mm: [0.; 3],
    };
}

/// A position that is not inside the travel.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Outside {
    /// The whole interval of the position is beyond the limit (a violation).
    /// Otherwise the interval straddles the limit: not shown inside, not
    /// shown outside. A position without calibration interval or arc gap is
    /// always certain.
    pub certain: bool,
    /// Where along the move (0 for a lone point).
    pub fraction: f64,
    /// The position in work coordinates.
    pub point_mm: Vec3,
    /// The position in machine coordinates (with the calibration offset).
    pub machine_mm: Vec3,
    /// The machine axis whose limit is passed, and whether it is the lower.
    pub axis: usize,
    pub low: bool,
    pub limit_mm: f64,
    /// The calibration interval of the position on that axis (a point
    /// without calibration).
    pub interval_mm: [f64; 2],
    /// How far beyond the limit the position is evaluated to be: the end of
    /// its interval nearest the travel when `certain`, the farthest otherwise.
    pub beyond_mm: f64,
    /// The bound that distance exceeds (for a decoded arc's nominal point
    /// that is not `certain` it is negative by the arc's gap: the point must
    /// be that far inside).
    pub guard_mm: f64,
    /// For a point of an arc's nominal circle: the arc's endpoint gap.
    pub nominal_gap_mm: Option<f64>,
}

/// Whose arc a nominal circle belongs to: see the module documentation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum ArcGap {
    /// camlib's planned move: outside only beyond the gap.
    Planned,
    /// A move decoded from posted words: inside only by the gap.
    Executed,
}

/// For a point of an arc's nominal circle: the arc's gap, and what the
/// point is allowed beyond a declared position before it is not inside
/// (`inside`, negative when it must be inside by the gap) and before it is
/// certainly outside (`outside`).
#[derive(Clone, Copy)]
struct Nominal {
    gap: f64,
    inside: f64,
    outside: f64,
}

/// `point` (work coordinates) against the travel: `None` when it is inside.
/// `nominal` is `None` for a declared position.
fn decide(
    frame: &WorkFrame,
    travel: &Travel,
    calibration: &Calibration,
    fraction: f64,
    point: Vec3,
    nominal: Option<Nominal>,
) -> Option<Outside> {
    let transformed = frame.to_design(point);
    let machine: Vec3 = std::array::from_fn(|k| transformed[k] + calibration.offset_mm[k]);
    let mut worst: Option<Outside> = None;
    for axis in 0..3 {
        let row = [frame.x[axis], frame.y[axis], frame.z[axis]];
        // T of the module documentation, and the length of this machine
        // axis in work coordinates (1 for a rotation): a displacement d of
        // the work point moves the coordinate by at most |row| |d|.
        let magnitude =
            frame.origin_mm[axis].abs() + (0..3).map(|j| (point[j] * row[j]).abs()).sum::<f64>();
        let transform = 5. * U * magnitude + f64::MIN_POSITIVE;
        let length = row[0].hypot(row[1]).hypot(row[2]);
        let (low_mm, high_mm) = (calibration.low_mm[axis], calibration.high_mm[axis]);
        // One rounded sum for the offset and one for an interval end, when
        // there is one (adding zero is exact).
        let at = machine[axis].abs();
        let calibrated = if calibration.offset_mm[axis] != 0. {
            U * at
        } else {
            0.
        } + if low_mm != 0. || high_mm != 0. {
            U * (at + low_mm.abs().max(high_mm.abs()))
        } else {
            0.
        };
        let interval = [machine[axis] + low_mm, machine[axis] + high_mm];
        for (low, limit) in [(true, travel.min[axis]), (false, travel.max[axis])] {
            // The difference of two numbers within a factor of two of each
            // other is exact (Sterbenz), and farther apart it exceeds any
            // guard here by orders of magnitude.
            // `near`: the end of the interval nearest the travel; `far`: the
            // other. Without an interval they are one number.
            let (near, far) = if low {
                (limit - interval[1], limit - interval[0])
            } else {
                (interval[0] - limit, interval[1] - limit)
            };
            let derived = if travel.lowered && axis == 2 {
                U * limit.abs()
            } else {
                0.
            };
            let evaluation = transform + derived + calibrated;
            let outside = evaluation + nominal.map_or(0., |n| n.outside * length);
            let inside = evaluation + nominal.map_or(0., |n| n.inside * length);
            let found = if near > outside {
                Some((true, near, outside))
            } else if far > inside {
                Some((false, far, inside))
            } else {
                None
            };
            let Some((certain, beyond, guard)) = found else {
                continue;
            };
            // A certain finding outranks an uncertain one; then the farthest.
            if worst.is_none_or(|w| (certain, beyond) > (w.certain, w.beyond_mm)) {
                worst = Some(Outside {
                    certain,
                    fraction,
                    point_mm: point,
                    machine_mm: machine,
                    axis,
                    low,
                    limit_mm: limit,
                    interval_mm: interval,
                    beyond_mm: beyond,
                    guard_mm: guard,
                    nominal_gap_mm: nominal.map(|n| n.gap),
                });
            }
        }
    }
    worst
}

/// A declared position (exact) against the travel.
pub(crate) fn point_outside(frame: &WorkFrame, travel: &Travel, point: Vec3) -> Option<Outside> {
    decide(frame, travel, &Calibration::NONE, 0., point, None)
}

/// A declared machine-axis position (a route waypoint, a start position of
/// the kinematic machine model) against the axis travel. It is given in
/// machine coordinates, so nothing is evaluated and the comparison is exact:
/// on a limit is inside, any binary64 step beyond is outside.
pub(crate) fn axes_outside(travel: &Travel, axes: Vec3) -> Option<Outside> {
    let mut worst: Option<Outside> = None;
    for axis in 0..3 {
        for (low, limit) in [(true, travel.min[axis]), (false, travel.max[axis])] {
            let beyond = if low {
                limit - axes[axis]
            } else {
                axes[axis] - limit
            };
            if (if low {
                axes[axis] < limit
            } else {
                axes[axis] > limit
            }) && worst.is_none_or(|w| beyond > w.beyond_mm)
            {
                worst = Some(Outside {
                    certain: true,
                    fraction: 0.,
                    point_mm: axes,
                    machine_mm: axes,
                    axis,
                    low,
                    limit_mm: limit,
                    interval_mm: [axes[axis]; 2],
                    beyond_mm: beyond,
                    guard_mm: 0.,
                    nominal_gap_mm: None,
                });
            }
        }
    }
    worst
}

/// For a point of the nominal circle of an arc or helix: the move's endpoint
/// gap, and the bound `E` on the evaluation of the point by
/// `PreparedCurve::position`/`nominal_end`.
///
/// With radius `r`, in-plane centre `c`, start angle `a` and sweep `s`, an
/// in-plane coordinate is `c + r cos(a + t s)`, evaluated with:
/// - the radius, `hypot` of two rounded differences: relative error `3u`;
/// - the start angle, `atan2` of the same differences: `u (1 + 2 pi)`, and
///   the angle `a + t s`: `u (1 + 3 pi + 2 |s|)` in all, which moves the
///   point along the circle by `r` times that;
/// - `sin`/`cos`: `2u`; the product: `u r`; the sum: `u (|c| + r)`.
///
/// That is at most `u (r (17.5 + 2 |s|) + |c|)` on each in-plane coordinate,
/// and `3u (|start_w| + |end_w|)` on the axial one (`start + t (end -
/// start)`). The library functions are taken at the 1 ULP glibc documents
/// for them (glibc manual, "Known Maximum Errors in Math Functions"), which
/// is not a guarantee of the language, so twice the sum is used.
fn nominal_gap_and_evaluation(path: &PreparedCurve) -> (f64, f64) {
    match *path.curve() {
        Curve::Line { .. } => (0., 0.),
        Curve::Arc {
            start,
            end,
            center,
            plane,
            sweep_rad,
        }
        | Curve::Helix {
            start,
            end,
            center,
            plane,
            sweep_rad,
        } => {
            let [u, v, w] = plane.axes();
            let radius = (start[u] - center[u]).hypot(start[v] - center[v]);
            let nominal = path.nominal_end();
            let gap = (nominal[u] - end[u]).hypot(nominal[v] - end[v]);
            let evaluation = 2.
                * U
                * (radius * (35. + 4. * sweep_rad.abs())
                    + center[u].abs()
                    + center[v].abs()
                    + 3. * (start[w].abs() + end[w].abs()));
            (gap, evaluation)
        }
    }
}

/// A whole move against the travel: the earliest point of it that is
/// outside (or, for a decoded arc, not inside by its gap), or `None` when the
/// move is inside (limits included).
pub(crate) fn curve_outside(
    frame: &WorkFrame,
    travel: &Travel,
    path: &PreparedCurve,
    arcs: ArcGap,
) -> Result<Option<Outside>, String> {
    curve_calibrated(frame, travel, &Calibration::NONE, path, arcs)
}

/// A whole move whose position carries a calibration interval, against the
/// travel. `None`: every tested point's whole interval is inside. Otherwise
/// the earliest point whose whole interval is beyond a limit (`certain`), or,
/// when there is none, the earliest whose interval straddles one.
pub(crate) fn curve_calibrated(
    frame: &WorkFrame,
    travel: &Travel,
    calibration: &Calibration,
    path: &PreparedCurve,
    arcs: ArcGap,
) -> Result<Option<Outside>, String> {
    let curve = path.curve();
    let mut points = vec![(0., curve.start(), None)];
    if !matches!(curve, Curve::Line { .. }) {
        let (gap, evaluation) = nominal_gap_and_evaluation(path);
        let nominal = Some(Nominal {
            gap,
            inside: match arcs {
                ArcGap::Planned => evaluation + gap,
                ArcGap::Executed => evaluation - gap,
            },
            outside: evaluation + gap,
        });
        let mut fractions = Vec::new();
        for axis in 0..3 {
            fractions.extend(path.stationary_fractions([
                frame.x[axis],
                frame.y[axis],
                frame.z[axis],
            ]));
        }
        fractions.sort_by(f64::total_cmp);
        for fraction in fractions {
            // At fraction 0 or 1 `position` returns the declared end.
            let nominal = nominal.filter(|_| fraction > 0. && fraction < 1.);
            points.push((fraction, path.position(fraction)?, nominal));
        }
        points.push((1., path.nominal_end(), nominal));
    }
    points.push((1., curve.end(), None));
    let mut uncertain = None;
    for (fraction, point, nominal) in points {
        match decide(frame, travel, calibration, fraction, point, nominal) {
            Some(found) if found.certain => return Ok(Some(found)),
            Some(found) => {
                uncertain.get_or_insert(found);
            }
            None => {}
        }
    }
    Ok(uncertain)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::Plane;

    fn turned(angle: f64, origin: Vec3) -> WorkFrame {
        let (s, c) = angle.sin_cos();
        WorkFrame {
            origin_mm: origin,
            x: [c, s, 0.],
            y: [-s, c, 0.],
            z: [0., 0., 1.],
        }
    }

    #[test]
    fn a_declared_position_is_held_to_the_evaluation_bound_only() {
        let travel = Travel::tip([0., 0., -400.], [450., 280., 0.]);
        let quarter = WorkFrame {
            origin_mm: [225.3, 140.7, -300.1],
            x: [0., 1., 0.],
            y: [-1., 0., 0.],
            z: [0., 0., 1.],
        };
        for (frame, on_limit) in [
            (WorkFrame::identity(), true),
            (turned(0., [123.456, 77.1, -250.3]), true),
            (quarter, true),
            (turned(0.3, [200., 100., -100.]), false),
            (turned(-2.1, [200., 100., -100.]), false),
        ] {
            frame.validate().unwrap();
            // A machine point brought into work coordinates (the inverse of
            // a rotation is its transpose).
            let work = |q: Vec3| -> Vec3 {
                let d: Vec3 = std::array::from_fn(|i| q[i] - frame.origin_mm[i]);
                [frame.x, frame.y, frame.z].map(|axis| (0..3).map(|i| d[i] * axis[i]).sum())
            };
            for corner in 0..8 {
                let q = std::array::from_fn(|i| {
                    if corner & (1 << i) == 0 {
                        travel.min[i]
                    } else {
                        travel.max[i]
                    }
                });
                // With axes along the machine's, the work point is one
                // rounded difference from the limit and its image one rounded
                // sum: within 2u T of the limit, so inside. Through a general
                // rotation the work point is itself a rounded computation,
                // which the rule does not allow for: there only a point
                // 1e-9 mm inside every limit is asserted.
                let inside = if on_limit {
                    q
                } else {
                    std::array::from_fn(|i| {
                        q[i] + if corner & (1 << i) == 0 { 1e-9 } else { -1e-9 }
                    })
                };
                assert!(
                    point_outside(&frame, &travel, work(inside)).is_none(),
                    "{frame:?} {q:?}"
                );
                // 1e-6 mm beyond any one limit is outside, on that axis.
                for axis in 0..3 {
                    let mut beyond = q;
                    let low = corner & (1 << axis) == 0;
                    beyond[axis] += if low { -1e-6 } else { 1e-6 };
                    let found = point_outside(&frame, &travel, work(beyond))
                        .unwrap_or_else(|| panic!("{frame:?} {beyond:?}"));
                    assert_eq!((found.axis, found.low), (axis, low));
                    assert!((found.beyond_mm - 1e-6).abs() < 1e-9, "{found:?}");
                    // The guard is the binary64 term: far below a nanometre.
                    assert!(found.guard_mm < 1e-12, "{found:?}");
                }
            }
        }
    }

    #[test]
    fn a_gauge_line_limit_is_closed_and_adds_one_rounding() {
        let travel = Travel::gauge_line([0., 0., -400.], [500., 400., 0.], 115.3);
        let frame = turned(0., [200., 150., -300.]);
        // The tool at Z home: its tip is the gauge length below machine Z 0.
        let home = [-200., -150., 300. - 115.3];
        assert!(point_outside(&frame, &travel, home).is_none());
        let above = [home[0], home[1], home[2] + 1e-6];
        let found = point_outside(&frame, &travel, above).unwrap();
        assert_eq!((found.axis, found.low), (2, false));
        assert!(found.guard_mm < 1e-12);
    }

    #[test]
    fn an_arc_keeps_its_endpoint_gap_and_no_more() {
        // A half turn tangent to the machine +Y limit at its middle.
        let arc = |reach: f64, end_off: f64| Curve::Arc {
            start: [-5., reach, 0.],
            end: [5., reach + end_off, 0.],
            center: [0., reach, 0.],
            plane: Plane::Xy,
            sweep_rad: -std::f64::consts::PI,
        };
        let travel = Travel::tip([-10., -10., -10.], [10., 15., 10.]);
        let frame = WorkFrame::identity();
        let decide = |curve: Curve, arcs: ArcGap| {
            curve_outside(&frame, &travel, &curve.prepare(1e-5).unwrap(), arcs).unwrap()
        };
        let check = |curve: Curve| decide(curve, ArcGap::Planned);
        // Tangent: on the limit, inside.
        assert!(check(arc(10., 0.)).is_none());
        // An exact arc 1e-6 mm beyond is outside at its middle: the motion
        // tolerance (1e-5 mm) is not a travel allowance.
        let found = check(arc(10. + 1e-6, 0.)).unwrap();
        assert_eq!((found.axis, found.low), (1, false));
        assert!((found.fraction - 0.5).abs() < 1e-12);
        assert!(found.guard_mm < 1e-12, "{found:?}");
        // An arc whose declared end is 4e-6 mm off its circle is known to
        // that gap only: 1e-6 mm beyond is not shown outside, 1e-5 mm is.
        assert!(check(arc(10. + 1e-6, 4e-6)).is_none());
        let found = check(arc(10. + 1e-5, 4e-6)).unwrap();
        assert!((found.guard_mm - 4e-6).abs() < 1e-9, "{found:?}");
        // The declared ends stay exact whatever the gap: an end 1e-6 mm
        // beyond a limit is outside.
        let ends = Travel::tip([-5., -10., -10.], [5. - 1e-6, 16., 10.]);
        let prepared = arc(10., 4e-6).prepare(1e-5).unwrap();
        for arcs in [ArcGap::Planned, ArcGap::Executed] {
            let found = curve_outside(&frame, &ends, &prepared, arcs)
                .unwrap()
                .unwrap();
            assert_eq!((found.axis, found.low, found.fraction), (0, false, 1.));
            assert!(found.guard_mm.abs() < 1e-12);
            assert!(found.nominal_gap_mm.is_none());
        }
        // A decoded arc: the controller's path is only known within the gap
        // of the nominal circle, so the gap shrinks what is admissible.
        let executed = |curve: Curve| decide(curve, ArcGap::Executed);
        // No gap: tangent is inside, 1e-6 mm beyond is outside.
        assert!(executed(arc(10., 0.)).is_none());
        assert!(executed(arc(10. + 1e-6, 0.)).is_some());
        // A 4e-6 mm gap: tangent is not shown inside, nor is 3e-6 mm inside
        // the limit; 5e-6 mm inside is.
        let found = executed(arc(10., 4e-6)).unwrap();
        assert_eq!((found.axis, found.low), (1, false));
        assert!((found.nominal_gap_mm.unwrap() - 4e-6).abs() < 1e-9);
        assert!((found.guard_mm + 4e-6).abs() < 1e-9, "{found:?}");
        assert!(executed(arc(10. - 3e-6, 4e-6)).is_some());
        assert!(executed(arc(10. - 5e-6, 4e-6)).is_none());
    }
}
