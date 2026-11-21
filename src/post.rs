//! Checked controller output: chunk/line/check records shared by the posts,
//! and the curve comparison and number formatting they rely on.
use crate::motion::{Curve, Event, Motion, Vec3};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize)]
pub struct Identity {
    pub program_id: String,
    pub revision: String,
    pub header_setup_sha256: String,
    pub abi: u32,
}
#[derive(Debug, Serialize, Deserialize)]
pub struct Line {
    pub number: usize,
    pub event_index: Option<usize>,
    pub operation: Option<String>,
    pub motion_id: Option<u64>,
    pub text: String,
    /// The independent decoder read this block as setting an offset or a
    /// mode without moving any axis: the axis letters it names are
    /// parameters, not positions (Haas and FANUC `G52 X0 Y0 Z0`, Haas
    /// `G100 X0 Y0 Z0`). The machine check then leaves the axes it names
    /// where they were. Only a decoder sets it; absent, every axis word in
    /// the block counts as commanding its axis.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub holds_axes: bool,
}
#[derive(Debug, Serialize, Deserialize)]
pub struct Checked {
    pub event_index: usize,
    pub line_interval: [usize; 2],
    pub motion: Option<Motion>,
    pub deviation_bound_mm: Option<f64>,
    /// Actual decoded controls before semantic comparison normalization.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub control_effects: Option<Vec<Event>>,
    /// A move written inside a canned-cycle block: the source event index of
    /// that cycle (whose lines carry it); this event writes no lines itself.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cycle_event_index: Option<usize>,
}
#[derive(Debug, Serialize)]
pub struct Chunk {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub decoded_initial_position_mm: Option<Vec3>,
    pub identity: Identity,
    pub lines: Vec<Line>,
    pub checked: Vec<Checked>,
    pub scope: &'static str,
}
// Count the real escaped JSON, including attribution, before state commits.
// No duplicate serialization buffer is retained just to enforce the quota.
pub(crate) fn check_chunk_size(chunk: &impl Serialize) -> Result<(), String> {
    struct Limit(usize);
    impl std::io::Write for Limit {
        fn write(&mut self, data: &[u8]) -> std::io::Result<usize> {
            if data.len() > 512 * 1024 - self.0 {
                return Err(std::io::Error::other("post encoded chunk exceeds 512 KiB"));
            }
            self.0 += data.len();
            Ok(data.len())
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }
    serde_json::to_writer(Limit(0), chunk).map_err(|e| e.to_string())
}
/// Full normalized-parameter bound, not sampled endpoint agreement. Rotations
/// obey |R(a)v-R(b)v| <= |v| |a-b|; axial coordinates interpolate linearly.
/// Bound on how far the controller's path for posted curve `b` strays from
/// source curve `a`. An arc whose end radius differs from its start radius
/// (rounded words) runs as a spiral between the two (LinuxCNC arc_data_ijk;
/// FANUC within parameter 3410): it strays from the start-radius circle by at
/// most that difference, which is added.
/// The arc a decoded block is replayed as. Arc words whose start and end
/// radius differ (within the controller's declared tolerance) describe no
/// single circle: the controller's path lies between the circles through
/// the start and through the end about the written centre. Checks of the
/// decoded program need one curve, so this is the circle through the written
/// start and end whose centre is nearest the written one (the written centre
/// moved along the chord onto its bisector). For an arc of at most half a
/// turn its distance from the written centre runs monotonically from the
/// start radius to the end radius, so it lies in that same band, and no
/// point of the controller's path is further from it than the mismatch,
/// which `curve_difference` has already charged to the block's deviation
/// bound. Lines, and arcs whose words agree, are returned unchanged.
pub(crate) fn replay_curve(curve: &Curve) -> Result<Curve, String> {
    let (start, end, center, plane, sweep, helix) = match *curve {
        Curve::Arc {
            start,
            end,
            center,
            plane,
            sweep_rad,
        } => (start, end, center, plane, sweep_rad, false),
        Curve::Helix {
            start,
            end,
            center,
            plane,
            sweep_rad,
        } => (start, end, center, plane, sweep_rad, true),
        Curve::Line { .. } => return Ok(curve.clone()),
    };
    let [u, v, _] = plane.axes();
    let r1 = (start[u] - center[u]).hypot(start[v] - center[v]);
    let r2 = (end[u] - center[u]).hypot(end[v] - center[v]);
    let (dx, dy) = (end[u] - start[u], end[v] - start[v]);
    let chord2 = dx * dx + dy * dy;
    if r1 == r2 || chord2 == 0. {
        return Ok(curve.clone());
    }
    if sweep.abs() > std::f64::consts::PI + 1e-9 {
        return Err("arc words that differ in radius are modelled for at most half a turn".into());
    }
    // The written centre, moved along the chord onto its bisector.
    let along = ((center[u] - 0.5 * (start[u] + end[u])) * dx
        + (center[v] - 0.5 * (start[v] + end[v])) * dy)
        / chord2;
    let mut c = center;
    c[u] -= along * dx;
    c[v] -= along * dy;
    let (x, y) = (start[u] - c[u], start[v] - c[v]);
    let (ex, ey) = (end[u] - c[u], end[v] - c[v]);
    let angle = (x * ey - y * ex).atan2(x * ex + y * ey);
    let tau = std::f64::consts::TAU;
    let w = if sweep > 0. {
        angle.rem_euclid(tau)
    } else {
        -(-angle).rem_euclid(tau)
    };
    Ok(if helix {
        Curve::Helix {
            start,
            end,
            center: c,
            plane,
            sweep_rad: w,
        }
    } else {
        Curve::Arc {
            start,
            end,
            center: c,
            plane,
            sweep_rad: w,
        }
    })
}

pub(crate) fn curve_difference(a: &Curve, b: &Curve, tolerance: f64) -> Result<f64, String> {
    let spiral = match *b {
        Curve::Arc {
            start,
            end,
            center,
            plane,
            ..
        }
        | Curve::Helix {
            start,
            end,
            center,
            plane,
            ..
        } => {
            let [u, v, _] = plane.axes();
            ((end[u] - center[u]).hypot(end[v] - center[v])
                - (start[u] - center[u]).hypot(start[v] - center[v]))
            .abs()
        }
        Curve::Line { .. } => 0.,
    };
    Ok(curve_difference_model(a, b, tolerance)? + spiral)
}
fn curve_difference_model(a: &Curve, b: &Curve, tolerance: f64) -> Result<f64, String> {
    a.prepare(tolerance)?;
    b.prepare(0.1)?;
    let distance = |p: Vec3, q: Vec3| (p[0] - q[0]).hypot(p[1] - q[1]).hypot(p[2] - q[2]);
    let scale = a
        .start()
        .iter()
        .chain(a.end().iter())
        .chain(b.start().iter())
        .chain(b.end().iter())
        .fold(1f64, |s, x| s.max(x.abs()));
    let center_scale = [a, b].iter().fold(scale, |max, curve| match curve {
        Curve::Arc { center, .. } | Curve::Helix { center, .. } => {
            center.iter().fold(max, |s, x| s.max(x.abs()))
        }
        _ => max,
    });
    let guard = 256. * f64::EPSILON * center_scale;
    match (a, b) {
        (Curve::Line { start: x, end: y }, Curve::Line { start: u, end: v }) => {
            Ok(distance(*x, *u).max(distance(*y, *v)) + guard)
        }
        (
            Curve::Arc {
                start: s,
                end: e,
                center: c,
                plane: p,
                sweep_rad: w,
            },
            Curve::Arc {
                start: t,
                end: f,
                center: d,
                plane: q,
                sweep_rad: z,
            },
        )
        | (
            Curve::Helix {
                start: s,
                end: e,
                center: c,
                plane: p,
                sweep_rad: w,
            },
            Curve::Helix {
                start: t,
                end: f,
                center: d,
                plane: q,
                sweep_rad: z,
            },
        ) if p == q => {
            // Geometric tolerance cannot authorize reversed direction or a
            // precision-collapsed partial arc that becomes an extra full turn.
            if w.signum() != z.signum()
                || (w.abs() == std::f64::consts::TAU) != (z.abs() == std::f64::consts::TAU)
            {
                return Err("posted arc direction or full-turn intent changed".into());
            }
            let [u, v, axial] = p.axes();
            let center_delta = (c[u] - d[u]).hypot(c[v] - d[v]);
            let vector_delta = ((s[u] - c[u]) - (t[u] - d[u])).hypot((s[v] - c[v]) - (t[v] - d[v]));
            let radius = (t[u] - d[u]).hypot(t[v] - d[v]);
            let angle_error = radius * (w - z).abs();
            // Anchor at the start: centre displacement cancels for short arcs.
            // ||R(theta)-I|| <= 2sin(min(|sweep|,pi)/2) over the WHOLE interval.
            // Keep the older bound as an alternative for large/full turns.
            let start_delta = (s[u] - t[u]).hypot(s[v] - t[v]);
            let anchored = start_delta
                + 2. * (w.abs().min(std::f64::consts::PI) * 0.5).sin() * vector_delta
                + angle_error;
            let xy = (center_delta + vector_delta + angle_error).min(anchored);
            let axial_error = (s[axial] - t[axial]).abs().max((e[axial] - f[axial]).abs());
            // Enclose the difference trajectory between a small fixed set of
            // anchors. This is NOT endpoint sampling: linear interpolation of
            // a twice differentiable vector curve errs by at most M*h²/8 when
            // its second derivative norm is <= M throughout that interval.
            // Rotation is 1-Lipschitz in angle; the cancellation-aware bound
            // remains useful for similar circles with slightly shifted centres.
            let source_radius = (s[u] - c[u]).hypot(s[v] - c[v]);
            let second = (w * w * (vector_delta + radius * (w - z).abs().min(2.))
                + (w * w - z * z).abs() * radius)
                .min(w * w * source_radius + z * z * radius);
            let mut anchors: f64 = 0.;
            for i in 0..=8 {
                let fraction = i as f64 / 8.;
                let (sa, ca) = (w * fraction).sin_cos();
                let (sb, cb) = (z * fraction).sin_cos();
                let da = [
                    (ca - 1.) * (s[u] - c[u]) - sa * (s[v] - c[v]),
                    sa * (s[u] - c[u]) + (ca - 1.) * (s[v] - c[v]),
                ];
                let db = [
                    (cb - 1.) * (t[u] - d[u]) - sb * (t[v] - d[v]),
                    sb * (t[u] - d[u]) + (cb - 1.) * (t[v] - d[v]),
                ];
                let dz = (1. - fraction) * (s[axial] - t[axial]) + fraction * (e[axial] - f[axial]);
                anchors = anchors.max(
                    (s[u] - t[u] + da[0] - db[0])
                        .hypot(s[v] - t[v] + da[1] - db[1])
                        .hypot(dz),
                );
            }
            let interpolated = anchors + second / (8. * 8. * 8.);
            Ok(xy
                .hypot(axial_error)
                .min(interpolated)
                .max(distance(*e, *f))
                + endpoint_gap(a)
                + endpoint_gap(b)
                + guard)
        }
        _ => Err("posted curve type/plane differs from source".into()),
    }
}
pub(crate) fn endpoint_gap(curve: &Curve) -> f64 {
    match curve {
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
            let [u, v, _] = plane.axes();
            let angle = (start[v] - center[v]).atan2(start[u] - center[u]) + sweep_rad;
            let radius = (start[u] - center[u]).hypot(start[v] - center[v]);
            (center[u] + radius * angle.cos() - end[u])
                .hypot(center[v] + radius * angle.sin() - end[v])
        }
        _ => 0.,
    }
}
pub(crate) fn format_number(value: f64, decimals: usize) -> Result<String, String> {
    if !value.is_finite() || value.abs() > 1e6 {
        return Err("invalid post number".into());
    }
    let text = format!("{value:.*}", decimals);
    Ok(if text.parse::<f64>().map_err(|e| e.to_string())? == 0. {
        format!("{:.*}", decimals, 0.)
    } else {
        text
    })
}

#[cfg(test)]
mod replay_tests {
    use super::*;
    use crate::motion::Plane;

    /// Arc words that differ in radius: the replay circle passes through the
    /// written start and end, and every point of it lies between the two
    /// circles about the written centre (the band the controller's path is
    /// in), so it is never further from that path than the mismatch.
    #[test]
    fn the_replay_circle_stays_in_the_band_between_the_written_radii() {
        let mut worst = 0f64;
        for k in 0..1200 {
            // Every plane, flat arcs and helixes.
            let plane = [Plane::Xy, Plane::Xz, Plane::Yz][k % 3];
            let helix = k % 2 == 1;
            let [u, v, a] = plane.axes();
            let a0 = 0.37 * k as f64;
            let sweep = [0.05, 0.3, 1.2, 2.8, 3.1][k % 5] * if k % 4 < 2 { 1. } else { -1. };
            let r = [0.8, 6., 60., 400.][k % 4];
            let gap = [0.0003, 0.001, 0.005][k % 3] * if k % 7 < 3 { 1. } else { -1. };
            let mut center = [0.; 3];
            (center[u], center[v], center[a]) = (12.5, -7.25, 3.);
            let mut start = center;
            start[u] += r * a0.cos();
            start[v] += r * a0.sin();
            let a1 = a0 + sweep;
            let mut end = center;
            end[u] += (r + gap) * a1.cos();
            end[v] += (r + gap) * a1.sin();
            if helix {
                end[a] -= 1.5;
            }
            let written = if helix {
                Curve::Helix {
                    start,
                    end,
                    center,
                    plane,
                    sweep_rad: sweep,
                }
            } else {
                Curve::Arc {
                    start,
                    end,
                    center,
                    plane,
                    sweep_rad: sweep,
                }
            };
            let replay = replay_curve(&written).unwrap();
            assert_eq!(
                std::mem::discriminant(&replay),
                std::mem::discriminant(&written)
            );
            let (Curve::Arc {
                start: s,
                end: e,
                center: c,
                plane: p,
                sweep_rad: w,
            }
            | Curve::Helix {
                start: s,
                end: e,
                center: c,
                plane: p,
                sweep_rad: w,
            }) = replay
            else {
                panic!()
            };
            assert_eq!((s, e, p), (start, end, plane));
            assert!(
                (w - sweep).abs() < 0.02 && w.signum() == sweep.signum(),
                "{w} {sweep}"
            );
            // An exact circle: it prepares at the tightest tolerance.
            replay.prepare(1e-9).unwrap();
            let radius = (s[u] - c[u]).hypot(s[v] - c[v]);
            let from = (s[v] - c[v]).atan2(s[u] - c[u]);
            let (low, high) = (r.min(r + gap), r.max(r + gap));
            for j in 0..=200 {
                let angle = from + w * j as f64 / 200.;
                let q = [c[u] + radius * angle.cos(), c[v] + radius * angle.sin()];
                let d = (q[0] - center[u]).hypot(q[1] - center[v]);
                worst = worst.max(low - d).max(d - high);
                assert!(
                    d >= low - 1e-9 && d <= high + 1e-9,
                    "{d} outside {low}..{high}"
                );
            }
        }
        assert!(worst <= 1e-9, "{worst}");
        // Agreeing words and lines are returned as they are; more than half
        // a turn with a mismatch has no such model.
        let exact = Curve::Arc {
            start: [1., 0., 0.],
            end: [0., 1., 0.],
            center: [0., 0., 0.],
            plane: Plane::Xy,
            sweep_rad: std::f64::consts::FRAC_PI_2,
        };
        assert_eq!(replay_curve(&exact).unwrap(), exact);
        let long = Curve::Arc {
            start: [1., 0., 0.],
            end: [0., -1.001, 0.],
            center: [0., 0., 0.],
            plane: Plane::Xy,
            sweep_rad: 1.5 * std::f64::consts::PI,
        };
        assert!(replay_curve(&long).is_err());
    }
}
