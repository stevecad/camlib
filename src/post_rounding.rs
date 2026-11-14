//! Choose representable circular words; the separate decoder still reads NC.
use crate::{
    motion::{Curve, Vec3},
    post::{curve_difference, format_number},
};
use std::f64::consts::TAU;

/// A controller's accepted mismatch between the start and end radius of
/// circular words (LinuxCNC `arc_data_ijk`, interp_arc.cc): accepted when
/// the absolute error is at most `absolute_mm`, or when the relative error
/// (to the larger radius) is at most `relative` and the absolute error at
/// most 100 x `absolute_mm`. `relative` 0 leaves only the absolute bound;
/// `absolute_mm` 0 demands exactly consistent words.
#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RadiusTolerance {
    pub absolute_mm: f64,
    #[serde(default)]
    pub relative: f64,
}
impl RadiusTolerance {
    pub fn valid(&self) -> bool {
        self.absolute_mm.is_finite()
            && (0. ..=0.005).contains(&self.absolute_mm)
            && self.relative.is_finite()
            && (0. ..=0.001).contains(&self.relative)
    }
    pub fn accepts(&self, r1: f64, r2: f64) -> bool {
        self.accepts_error((r1 - r2).abs(), r1.max(r2))
    }
    /// The same rule for a radius mismatch `error` computed elsewhere
    /// (`ArcRadii`), `larger` being the larger radius.
    pub fn accepts_error(&self, error: f64, larger: f64) -> bool {
        error <= self.absolute_mm
            || (error <= 100. * self.absolute_mm && error <= self.relative * larger)
    }
    /// Whether the controller accepts circular words with these radii.
    pub fn accepts_arc(&self, radii: &ArcRadii) -> bool {
        self.accepts_error(radii.error, radii.start.max(radii.end))
    }
}

/// Start and end radius (mm) of circular words, and their mismatch.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArcRadii {
    pub start: f64,
    pub end: f64,
    pub error: f64,
}

/// Word grid used to decide arc radii exactly: 1/(2 x 10^9) output units
/// holds every word of up to 9 decimals, and half of one (lathe X words
/// written as diameters).
const WORD_GRID: f64 = 2e9;

/// The integer `k` such that `mm` is the value a decoder gets from the word
/// k / WORD_GRID (parsed, then scaled by `unit`), or None when no such word
/// exists. k / WORD_GRID is exactly rounded, as parsing the word's text is.
fn on_word_grid(mm: f64, unit: f64) -> Option<i128> {
    let k = (mm / unit * WORD_GRID).round_ties_even();
    (k.abs() <= 9e15 && k / WORD_GRID * unit == mm).then_some(k as i128)
}

/// Radii of circular words in the plane `(u, v)`: `start` and `end` are the
/// positions (mm), `offset` the centre offset words times `unit`.
///
/// Decided without the platform's libm, so every platform, and the Python
/// decoder, gets the same answer (issue #169: glibc's `hypot` and macOS's
/// disagreed by one bit and flipped a zero-tolerance check):
/// - When every value is a word (`on_word_grid`), the squared radii are exact
///   integers on the word grid, so words with exactly equal radii have error
///   exactly 0, and a zero tolerance stays strict without an epsilon. Other
///   errors are |A - B| / (sqrt A + sqrt B), from correctly rounded operations.
/// - When the start is a position no word wrote (e.g. a declared tool-change
///   position), it is a binary64 value: the radii are computed from it as a
///   controller does (centre = start + offset), with correctly rounded
///   operations only.
pub fn arc_radii(start: [f64; 2], end: [f64; 2], offset: [f64; 2], unit: f64) -> ArcRadii {
    let grid = |mm: f64| on_word_grid(mm, unit);
    if let (Some(su), Some(sv), Some(eu), Some(ev), Some(iu), Some(iv)) = (
        grid(start[0]),
        grid(start[1]),
        grid(end[0]),
        grid(end[1]),
        grid(offset[0]),
        grid(offset[1]),
    ) {
        // |values| <= 9e15, so each squared radius is below 1.5e33 < i128::MAX.
        let a = iu * iu + iv * iv;
        let (x, y) = (eu - su - iu, ev - sv - iv);
        let b = x * x + y * y;
        let scale = unit / WORD_GRID;
        let (ra, rb) = ((a as f64).sqrt(), (b as f64).sqrt());
        let error = if a == b {
            0.
        } else {
            (a - b).abs() as f64 / (ra + rb) * scale
        };
        return ArcRadii {
            start: ra * scale,
            end: rb * scale,
            error,
        };
    }
    let center = [start[0] + offset[0], start[1] + offset[1]];
    let (x, y) = (start[0] - center[0], start[1] - center[1]);
    let (ex, ey) = (end[0] - center[0], end[1] - center[1]);
    let (r1, r2) = ((x * x + y * y).sqrt(), (ex * ex + ey * ey).sqrt());
    ArcRadii {
        start: r1,
        end: r2,
        error: (r1 - r2).abs(),
    }
}

/// `max_sweep_rad`: the largest sweep the machine accepts in one block;
/// candidates whose written sweep exceeds it are not taken.
#[allow(clippy::too_many_arguments)]
pub fn circular_words(
    source: &Curve,
    start: Vec3,
    unit: f64,
    decimals: usize,
    geometry_tolerance: f64,
    output_tolerance: f64,
    radius_tolerance: RadiusTolerance,
    max_sweep_rad: f64,
) -> Result<(Vec3, Vec3), String> {
    if ![1., 25.4].contains(&unit)
        || !(2..=9).contains(&decimals)
        || !output_tolerance.is_finite()
        || !(1e-9..=0.1).contains(&output_tolerance)
        || !radius_tolerance.valid()
        || start.iter().any(|v| !v.is_finite() || v.abs() > 1e6)
    {
        return Err("invalid circular word rounding parameters".into());
    }
    source.prepare(geometry_tolerance)?;
    let (center, plane, sweep, helix) = match source {
        Curve::Arc {
            center,
            plane,
            sweep_rad,
            ..
        } => (*center, *plane, *sweep_rad, false),
        Curve::Helix {
            center,
            plane,
            sweep_rad,
            ..
        } => (*center, *plane, *sweep_rad, true),
        _ => return Err("circular words require an arc or helix".into()),
    };
    if sweep.abs() > TAU {
        return Err("split multi-turn circular output before formatting".into());
    }
    let [u, v, a] = plane.axes();
    let quantize = |value: f64| -> Result<f64, String> {
        Ok(format_number(value / unit, decimals)?
            .parse::<f64>()
            .map_err(|e| e.to_string())?
            * unit)
    };
    let mut end = source.end();
    for value in &mut end {
        *value = quantize(*value)?;
    }
    // A source closed in its plane (a full turn: adaptive orbits, helical
    // bores) is written as a full circle: the end word repeats the actual
    // start, which is already representable (it is the decoded position).
    // That makes the start and end radii identical whatever the centre words,
    // so radius consistency is exact; only the I/J rounding remains, and the
    // independent comparison below bounds it against the source. Rounding the
    // source end instead would open the circle whenever the start was moved
    // by an earlier block's word choice. (LinuxCNC G2/G3 "full circle" when
    // the end equals the start with I/J offsets; FANUC/Haas likewise with
    // I/J, never R.)
    let closed = {
        let (a0, a1) = (source.start(), source.end());
        a0[u] == a1[u] && a0[v] == a1[v]
    };
    if closed {
        end[u] = start[u];
        end[v] = start[v];
    }
    if !helix {
        end[a] = start[a];
    }
    if helix && end[a] == start[a] {
        return Err(
            "helix pitch collapses at output precision; increase decimals or use bounded lowering"
                .into(),
        );
    }
    let mut offsets = [0.; 3];
    for k in [u, v] {
        offsets[k] = quantize(center[k] - start[k])?;
    }
    let assess = |end: Vec3, offsets: Vec3| -> Option<f64> {
        let mut c = start;
        for k in [u, v] {
            c[k] += offsets[k];
        }
        let x = start[u] - c[u];
        let y = start[v] - c[v];
        let ex = end[u] - c[u];
        let ey = end[v] - c[v];
        let radii = arc_radii(
            [start[u], start[v]],
            [end[u], end[v]],
            [offsets[u], offsets[v]],
            unit,
        );
        if radii.start < 1e-6 || !radius_tolerance.accepts_arc(&radii) {
            return None;
        }
        let angle = (x * ey - y * ex).atan2(x * ex + y * ey);
        let mut w = if sweep > 0. {
            angle.rem_euclid(TAU)
        } else {
            -(-angle).rem_euclid(TAU)
        };
        if w == 0. {
            w = sweep.signum() * TAU;
        }
        // The arc as written, not as planned, is what the controller runs: a
        // half turn whose rounded words span a hair more than the machine's
        // largest sweep is not one it accepts.
        if w.abs() > max_sweep_rad {
            return None;
        }
        let curve = if helix {
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
        };
        // Reserve rounding margin below the source/stock consistency tolerance.
        curve.prepare((geometry_tolerance * 0.5).max(1e-9)).ok()?;
        let error = curve_difference(source, &curve, geometry_tolerance).ok()?;
        (error <= output_tolerance).then_some(error)
    };
    if assess(end, offsets).is_some() {
        return Ok((end, offsets));
    }
    let step = unit * 10f64.powi(-(decimals as i32));
    if closed {
        // The end stays the start (a closed curve must close exactly); only
        // the centre words may move. Search the representable centres next to
        // the authored one, keeping the smallest independently assessed error.
        let mut best: Option<(f64, Vec3)> = None;
        for iu in -2..=2 {
            for iv in -2..=2 {
                let mut o = [0.; 3];
                o[u] = quantize(center[u] - start[u] + iu as f64 * step)?;
                o[v] = quantize(center[v] - start[v] + iv as f64 * step)?;
                if let Some(error) = assess(end, o)
                    && best.as_ref().is_none_or(|b| error < b.0)
                {
                    best = Some((error, o));
                }
            }
        }
        return best.map(|(_, o)| (end, o)).ok_or_else(|| {
            "no full-circle centre words within the error budget at configured precision; \
             increase decimals or use bounded lowering"
                .into()
        });
    }
    let mut best: Option<(f64, Vec3, Vec3)> = None;
    // At most 625 candidates. Project the authored centre onto each rounded
    // chord's bisector, then test nearby representable IJK words. Nothing in
    // the decoder is adjusted: these corrected words are what will be emitted.
    for du in -2..=2 {
        for dv in -2..=2 {
            let mut e = end;
            e[u] = quantize(end[u] + du as f64 * step)?;
            e[v] = quantize(end[v] + dv as f64 * step)?;
            let dx = e[u] - start[u];
            let dy = e[v] - start[v];
            let length2 = dx * dx + dy * dy;
            if length2 == 0. {
                continue;
            }
            let shift = ((center[u] - (start[u] + e[u]) * 0.5) * dx
                + (center[v] - (start[v] + e[v]) * 0.5) * dy)
                / length2;
            let projected = [
                center[u] - shift * dx - start[u],
                center[v] - shift * dy - start[v],
            ];
            for iu in -2..=2 {
                for iv in -2..=2 {
                    let mut o = [0.; 3];
                    o[u] = quantize(projected[0] + iu as f64 * step)?;
                    o[v] = quantize(projected[1] + iv as f64 * step)?;
                    if let Some(error) = assess(e, o)
                        && best.as_ref().is_none_or(|b| error < b.0)
                    {
                        best = Some((error, e, o));
                    }
                }
            }
        }
    }
    if best.is_none() {
        // A short local square around the projected centre can miss every
        // sufficiently radius-consistent grid point. Search along the chord's
        // bisector instead: quantize one centre word, solve the equal-radius
        // equation for the other, then quantize and independently assess it.
        // Keep already successful output unchanged. This fallback is at most
        // 169 * 129 candidates, regardless of coordinate magnitude/precision.
        // Coarse grids (e.g. 3-decimal mm) need a wider endpoint neighbourhood:
        // radius-consistent lattice pairs are sparse, but within the output
        // budget there are typically many. Every candidate is still assessed
        // against the full source curve and output tolerance.
        let steps = (4. * output_tolerance / step).ceil().clamp(2., 64.) as i32;
        let reach = (output_tolerance / step).ceil().clamp(2., 6.) as i32;
        for du in -reach..=reach {
            for dv in -reach..=reach {
                let mut e = end;
                e[u] = quantize(end[u] + du as f64 * step)?;
                e[v] = quantize(end[v] + dv as f64 * step)?;
                let d = [e[u] - start[u], e[v] - start[v]];
                let length2 = d[0] * d[0] + d[1] * d[1];
                if length2 == 0. {
                    continue;
                }
                let original = [center[u] - start[u], center[v] - start[v]];
                let shift = (original[0] * d[0] + original[1] * d[1] - length2 / 2.) / length2;
                let projected = [original[0] - shift * d[0], original[1] - shift * d[1]];
                let solved = usize::from(d[1].abs() > d[0].abs());
                let free = 1 - solved;
                for n in -steps..=steps {
                    let mut pair = [0.; 2];
                    pair[free] = quantize(projected[free] + n as f64 * step)?;
                    pair[solved] = quantize((length2 / 2. - d[free] * pair[free]) / d[solved])?;
                    let mut o = [0.; 3];
                    o[u] = pair[0];
                    o[v] = pair[1];
                    if let Some(error) = assess(e, o)
                        && best.as_ref().is_none_or(|b| error < b.0)
                    {
                        best = Some((error, e, o));
                    }
                }
            }
        }
    }
    best.map(|(_,e,o)|(e,o)).ok_or_else(||"no radius-consistent circular words at configured precision and error budget; increase decimals or use bounded lowering".into())
}

#[cfg(test)]
mod tests {
    const ABS_0_005: RadiusTolerance = RadiusTolerance {
        absolute_mm: 0.005,
        relative: 0.,
    };
    /// LinuxCNC 2.9 interp_arc.cc: rejected when abs > 100 T, or when
    /// rel > R and abs > T (T = CENTER_ARC_RADIUS_TOLERANCE_MM, R = 0.001).
    #[test]
    fn radius_tolerance_follows_linuxcnc_arc_data_ijk() {
        let linuxcnc = RadiusTolerance {
            absolute_mm: 0.00127,
            relative: 0.001,
        };
        assert!(linuxcnc.accepts(2., 2.00127));
        // r = 2: 0.005 is 0.25 % and above T: LinuxCNC refuses it.
        assert!(!linuxcnc.accepts(2., 2.005));
        // r = 10: 0.005 is 0.05 %: accepted by the relative rule.
        assert!(linuxcnc.accepts(10., 10.005));
        // r = 200: 0.15 is within 0.1 % but above 100 T = 0.127.
        assert!(!linuxcnc.accepts(200., 200.15));
        assert!(linuxcnc.accepts(200., 200.12));
        let exact = RadiusTolerance {
            absolute_mm: 0.,
            relative: 0.001,
        };
        assert!(exact.accepts(5., 5.) && !exact.accepts(5., 5.000001));
        assert!(
            !RadiusTolerance {
                absolute_mm: 0.006,
                relative: 0.
            }
            .valid()
        );
        assert!(
            !RadiusTolerance {
                absolute_mm: 0.001,
                relative: 0.002
            }
            .valid()
        );
    }

    use super::*;
    use crate::motion::Plane;
    use std::f64::consts::PI;

    /// Regression: an ordinary pocket arc around an r=4 island (Ø6 cutter).
    /// The previous segment's endpoint was already rounded (Y21.99983). At 3
    /// decimals no exactly radius-consistent words exist within the budget
    /// (chord lattice too coarse), which is why the generic Machine now
    /// defaults to 4 mm decimals; that must find words.
    #[test]
    fn four_decimal_island_arc_finds_consistent_words() {
        let source = Curve::Arc {
            start: [20.068992753242778, 21.99983, 8.5],
            end: [27.00017, 15.0, 8.5],
            center: [20.0, 15.000000000000002, 8.5],
            plane: Plane::Xy,
            sweep_rad: -1.5609402989751604,
        };
        assert!(
            circular_words(
                &source,
                [20.069, 22.0, 8.5],
                1.,
                3,
                1e-5,
                0.01,
                ABS_0_005,
                f64::INFINITY
            )
            .is_err()
        );
        let start = [20.069, 21.9998, 8.5];
        let (end, offsets) =
            circular_words(&source, start, 1., 4, 1e-5, 0.01, ABS_0_005, f64::INFINITY)
                .expect("representable words");
        // Decode the emitted words as a controller would (clockwise G2).
        let (x, y) = (-offsets[0], -offsets[1]);
        let (ex, ey) = (
            end[0] - start[0] - offsets[0],
            end[1] - start[1] - offsets[1],
        );
        let posted = Curve::Arc {
            start,
            end,
            center: [start[0] + offsets[0], start[1] + offsets[1], 8.5],
            plane: Plane::Xy,
            sweep_rad: -(-(x * ey - y * ex).atan2(x * ex + y * ey)).rem_euclid(TAU),
        };
        // Words must be exactly consistent at the engine's own tolerance...
        let r0 = offsets[0].hypot(offsets[1]);
        let r1 = (end[0] - start[0] - offsets[0]).hypot(end[1] - start[1] - offsets[1]);
        assert!(
            (r0 - r1).abs() <= 5e-6,
            "radius mismatch {}",
            (r0 - r1).abs()
        );
        for value in end.iter().chain(&offsets) {
            assert!((value * 1e4 - (value * 1e4).round()).abs() < 1e-6);
        }
        // ...and stay inside the output budget against the source arc.
        assert!(curve_difference(&source, &posted, 1e-5).unwrap() <= 0.01);
    }

    /// #58: a full orbit whose decoded start was moved off its rounded source
    /// start (by an earlier block's word choice) is still one full circle: the
    /// end word is the start, so the radii agree exactly, and the posted
    /// circle stays within the budget of the source.
    #[test]
    fn full_circle_words_close_on_a_moved_start() {
        for (plane, sweep) in [(Plane::Xy, TAU), (Plane::Xy, -TAU), (Plane::Xz, TAU)] {
            let [u, v, a] = plane.axes();
            let mut center = [0.; 3];
            center[u] = 7.5;
            center[a] = 3.;
            let mut point = center;
            point[u] += 1.764 * 0.2_f64.cos();
            point[v] += 1.764 * 0.2_f64.sin();
            let source = Curve::Arc {
                start: point,
                end: point,
                center,
                plane,
                sweep_rad: sweep,
            };
            let mut start = point;
            // Two grid steps away from the source start rounded alone.
            start[u] = (point[u] * 1e3).round() / 1e3 - 0.002;
            start[v] = (point[v] * 1e3).round() / 1e3 + 0.001;
            let (end, offsets) = circular_words(
                &source,
                start,
                1.,
                3,
                1e-5,
                0.01,
                RadiusTolerance::default(),
                f64::INFINITY,
            )
            .expect("full-circle words");
            assert_eq!(end, start);
            for value in offsets {
                assert!((value * 1e3 - (value * 1e3).round()).abs() < 1e-9);
            }
            let mut c = start;
            c[u] += offsets[u];
            c[v] += offsets[v];
            let posted = Curve::Arc {
                start,
                end,
                center: c,
                plane,
                sweep_rad: sweep,
            };
            assert!(curve_difference(&source, &posted, 1e-5).unwrap() <= 0.01);
        }
    }

    #[test]
    fn spiral_arc_words_add_their_radius_mismatch() {
        // Same circle; the posted end word sits 0.004 mm outside it, so the
        // controller cuts a spiral that strays up to 0.004 mm from the source.
        let arc = |end_radius: f64| Curve::Arc {
            start: [10., 0., 0.],
            end: [0., end_radius, 0.],
            center: [0.; 3],
            plane: Plane::Xy,
            sweep_rad: PI / 2.,
        };
        let exact = curve_difference(&arc(10.), &arc(10.), 1e-5).unwrap();
        let spiral = curve_difference(&arc(10.), &arc(10.004), 1e-5).unwrap();
        assert!(exact < 1e-4, "{exact}");
        assert!(spiral >= 0.004 + exact, "{spiral}");
    }

    #[test]
    fn anchored_bound_covers_entire_short_major_and_full_curves() {
        for plane in [Plane::Xy, Plane::Xz, Plane::Yz] {
            let [u, v, a] = plane.axes();
            for w in [0.001, 0.1, PI, 5.9, TAU] {
                for sign in [-1., 1.] {
                    for origin in [0., 100_000.] {
                        for helix in [false, true] {
                            let make = |shift: f64, sweep: f64| {
                                let mut center = [0.; 3];
                                center[u] = origin + shift;
                                center[v] = -origin;
                                center[a] = 3.;
                                let mut start = center;
                                let radius = 123.45 - shift;
                                start[u] += radius;
                                let mut end = center;
                                end[u] += radius * sweep.cos();
                                end[v] += radius * sweep.sin();
                                if helix {
                                    end[a] += 0.8 + shift * 0.01;
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
                                }
                            };
                            let source = make(0., sign * w);
                            let posted = make(0.02, sign * (w + if w == TAU { 0. } else { 1e-6 }));
                            let bound = curve_difference(&source, &posted, 1e-5).unwrap();
                            if w == 0.001 {
                                assert!(bound < 0.001);
                            }
                            // Independent analytic coordinates sampled densely are a
                            // regression oracle, not the continuous certificate.
                            let point = |shift: f64, sweep: f64, t: f64| {
                                [
                                    origin + shift + (123.45 - shift) * (sweep * t).cos(),
                                    -origin + (123.45 - shift) * (sweep * t).sin(),
                                    3. + if helix { (0.8 + shift * 0.01) * t } else { 0. },
                                ]
                            };
                            for i in 0..=2000 {
                                let t = i as f64 / 2000.;
                                let x = point(0., sign * w, t);
                                let y =
                                    point(0.02, sign * (w + if w == TAU { 0. } else { 1e-6 }), t);
                                let distance = (x[0] - y[0]).hypot(x[1] - y[1]).hypot(x[2] - y[2]);
                                assert!(distance <= bound, "distance {distance} > bound {bound}");
                            }
                        }
                    }
                }
            }
        }
    }

    const EXACT: RadiusTolerance = RadiusTolerance {
        absolute_mm: 0.,
        relative: 0.,
    };

    /// Issue #169: X29.999 Y20 I10.002 J0.006 from (9.995, 20) has exactly
    /// equal radii on the word grid. Rebuilt in binary64, the radius vectors
    /// differ in the last bit and glibc's hypot calls them unequal (macOS's
    /// equal); decided on the words, the error is exactly 0 everywhere.
    #[test]
    fn equal_radius_words_are_decided_exactly_on_the_word_grid() {
        let radii = arc_radii([9.995, 20.], [29.999, 20.], [10.002, 0.006], 1.);
        assert_eq!(radii.error, 0.);
        assert_eq!(radii.start, radii.end);
        assert!(EXACT.accepts_arc(&radii));
        // Inch words: positions are word x 25.4 in mm, as the decoders hold them.
        let inch = |w: f64| w * 25.4;
        let radii = arc_radii(
            [inch(9.995), inch(20.)],
            [inch(29.999), inch(20.)],
            [inch(10.002), inch(0.006)],
            25.4,
        );
        assert_eq!(radii.error, 0.);
        // A lathe X word written as a diameter decodes to half of it.
        let radii = arc_radii([10.001 * 0.5, 0.], [30.001 * 0.5, 0.], [5., 0.], 1.);
        assert_eq!(radii.error, 0.);
        // One grid unit off is a real mismatch: a zero tolerance refuses it.
        let radii = arc_radii([9.995, 20.], [29.999, 20.001], [10.002, 0.006], 1.);
        assert!(radii.error > 0. && !EXACT.accepts_arc(&radii));
        assert!(ABS_0_005.accepts_arc(&radii));
    }

    /// A start no word wrote is a binary64 value: the radii come from it with
    /// correctly rounded operations only (no libm), as a controller computes
    /// them (centre = start + offset).
    #[test]
    fn a_start_off_the_word_grid_is_decided_in_binary64() {
        let start = [0.1 + 0.2, 20.];
        assert!(on_word_grid(start[0], 1.).is_none());
        let radii = arc_radii(start, [10.3, 20.], [5., 0.], 1.);
        let c = [start[0] + 5., start[1]];
        let (x, ex) = (start[0] - c[0], 10.3 - c[0]);
        assert_eq!(radii.start, (x * x).sqrt());
        assert_eq!(radii.end, (ex * ex).sqrt());
        assert_eq!(radii.error, (radii.start - radii.end).abs());
    }

    /// The word chooser keeps exactly consistent authored words at zero
    /// radius tolerance (before #169 was fixed, glibc rejected these).
    #[test]
    fn zero_radius_tolerance_keeps_exactly_consistent_words() {
        let source = Curve::Arc {
            start: [9.995, 20., 0.],
            end: [29.999, 20., 0.],
            center: [19.997, 20.006, 0.],
            plane: Plane::Xy,
            sweep_rad: 3.1403928936857164,
        };
        let (end, offsets) = circular_words(
            &source,
            [9.995, 20., 0.],
            1.,
            3,
            1e-5,
            0.01,
            EXACT,
            f64::INFINITY,
        )
        .unwrap();
        assert_eq!((end[0], end[1]), (29.999, 20.));
        assert_eq!((offsets[0], offsets[1]), (10.002, 0.006));
    }

    /// The decoders decide radii with their own code (`post_decode::arc_radii`),
    /// so a mistake in the chooser's is not repeated there. The two must still
    /// give the same bits, or a program the chooser writes could be refused
    /// when it is decoded again. Words are parsed from text as a decoder does:
    /// mm and inch, lathe diameters, exactly equal radii nudged by -1, 0 or +1
    /// in the last decimal, unrelated ends, and starts no word wrote.
    #[test]
    fn decoder_and_chooser_agree_on_arc_radii() {
        let mut seed = 0x9e37_79b9_7f4a_7c15_u64;
        let mut next = |n: i64| {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed % (2 * n as u64 + 1)) as i64 - n
        };
        let (mut exact_equal, mut exact_unequal, mut binary64) = (0, 0, 0);
        for case in 0..20_000 {
            let unit = [1., 25.4][case % 2];
            let decimals = (case / 2) % 5;
            let scale = 10_i64.pow(decimals as u32);
            // The value a decoder holds for the word n / 10^decimals.
            let half = if case % 7 == 0 { 0.5 } else { 1. };
            let mm = |n: i64| {
                let text = format!("{}{}", if n < 0 { "-" } else { "" }, n.unsigned_abs());
                let text = if decimals == 0 {
                    text
                } else {
                    let digits = format!("{:0>w$}", n.unsigned_abs(), w = decimals + 1);
                    let (int, frac) = digits.split_at(digits.len() - decimals);
                    format!("{}{int}.{frac}", if n < 0 { "-" } else { "" })
                };
                text.parse::<f64>().unwrap() * unit * half
            };
            let range = 500 * scale;
            let (s, i) = ([next(range), next(range)], [next(range), next(range)]);
            let c = [s[0] + i[0], s[1] + i[1]];
            let e = match case % 4 {
                // Start rotated by 90, 180 or 270 degrees about the centre.
                0..=2 => {
                    let (x, y) = (s[0] - c[0], s[1] - c[1]);
                    let (x, y) = [(-y, x), (-x, -y), (y, -x)][case % 4];
                    [c[0] + x + next(1), c[1] + y + next(1)]
                }
                _ => [next(range), next(range)],
            };
            let mut start = [mm(s[0]), mm(s[1])];
            if case % 11 == 0 {
                start[0] = f64::from_bits(start[0].to_bits() + 1);
            }
            let end = [mm(e[0]), mm(e[1])];
            let offset = [mm(i[0]), mm(i[1])];
            let chooser = arc_radii(start, end, offset, unit);
            let (r1, r2, error) = crate::post_decode::arc_radii(start, end, offset, unit);
            let bits = |r: [f64; 3]| r.map(f64::to_bits);
            assert_eq!(
                bits([chooser.start, chooser.end, chooser.error]),
                bits([r1, r2, error]),
                "{start:?} {end:?} {offset:?} unit {unit}"
            );
            match on_word_grid(start[0], unit) {
                None => binary64 += 1,
                Some(_) if error == 0. => exact_equal += 1,
                Some(_) => exact_unequal += 1,
            }
        }
        // Every branch of the rule was exercised.
        assert!(exact_equal > 1000 && exact_unequal > 1000 && binary64 > 1000);
    }
}
