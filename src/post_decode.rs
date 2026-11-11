//! Configured actual-word motion decoding. No formatter or expected geometry input.
//! Python retains control-event semantics/custom languages; this immutable compiled
//! grammar accelerates single-motion blocks, falling back for other block forms.
use crate::motion::{Curve, Plane, Vec3};
use crate::post_rounding::RadiusTolerance;
use serde::Deserialize;
use std::{
    collections::{HashMap, HashSet},
    sync::Arc,
};

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Rule {
    pub effect: String,
    pub code: String,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub commands: Vec<Rule>,
    pub axes: [String; 3],
    pub centers: [String; 3],
    pub feed_word: String,
    pub parentheses_comments: bool,
    pub radius_tolerance_mm: f64,
    /// Relative start/end radius tolerance (see RadiusTolerance).
    #[serde(default)]
    pub radius_relative_tolerance: f64,
}
struct Compiled {
    numeric: HashMap<(u8, u64), u8>,
    keywords: HashSet<String>,
    axes: [usize; 3],
    centers: [usize; 3],
    feed: usize,
    comments: bool,
    radius_tolerance: RadiusTolerance,
}
#[derive(Clone)]
pub struct Decoder {
    compiled: Arc<Compiled>,
}
pub struct Decoded {
    pub curve: Curve,
    pub feed: Option<f64>,
    pub rapid: bool,
}
/// Start radius, end radius and their mismatch (mm) of decoded circular words
/// in one plane: `start` and `end` are positions (mm), `offset` the centre
/// offset words times `unit` (issue #169, `docs/posts.md`).
///
/// The decoders' own reading of the rule, kept apart from the word chooser's
/// `post_rounding::arc_radii` so that a mistake in one is not repeated in the
/// other; `decoder_and_chooser_agree_on_arc_radii` checks they give the same
/// bits. Only correctly rounded operations, no libm:
/// - every value is a word k / 2e9 output units: the centre and both radius
///   vectors are integers on that grid and the squared radii exact;
/// - otherwise (a start no word wrote) the binary64 positions, with the
///   centre at start + offset, as a controller computes it.
pub(crate) fn arc_radii(
    start: [f64; 2],
    end: [f64; 2],
    offset: [f64; 2],
    unit: f64,
) -> (f64, f64, f64) {
    // Steps per output unit: every word of up to 9 decimals, and half of one
    // (an X word written as a diameter).
    const STEPS: f64 = 2e9;
    // The k whose word k / STEPS decodes (parse, then times unit) to mm.
    let word = |mm: f64| {
        let k = (mm / unit * STEPS).round_ties_even();
        (k.abs() <= 9e15 && k / STEPS * unit == mm).then_some(k as i128)
    };
    let words = [start[0], start[1], end[0], end[1], offset[0], offset[1]].map(word);
    if let [Some(su), Some(sv), Some(eu), Some(ev), Some(iu), Some(iv)] = words {
        // |k| <= 9e15: centre <= 1.8e16, squared radius < 1.5e33 < i128::MAX.
        let (cu, cv) = (su + iu, sv + iv);
        let squared = |u: i128, v: i128| (u - cu).pow(2) + (v - cv).pow(2);
        let (a, b) = (squared(su, sv), squared(eu, ev));
        let (ra, rb) = ((a as f64).sqrt(), (b as f64).sqrt());
        let step = unit / STEPS;
        let error = if a == b {
            0.
        } else {
            a.abs_diff(b) as f64 / (ra + rb) * step
        };
        return (ra * step, rb * step, error);
    }
    let center = [start[0] + offset[0], start[1] + offset[1]];
    let radius = |p: [f64; 2]| {
        let (x, y) = (p[0] - center[0], p[1] - center[1]);
        (x * x + y * y).sqrt()
    };
    let (r1, r2) = (radius(start), radius(end));
    (r1, r2, (r1 - r2).abs())
}

fn key(letter: u8, value: f64) -> (u8, u64) {
    (letter, if value == 0. { 0 } else { value.to_bits() })
}
fn address(s: &str) -> Result<usize, String> {
    if s.len() != 1 || !s.as_bytes()[0].is_ascii_uppercase() || b"GMN".contains(&s.as_bytes()[0]) {
        return Err("configured word address must be one uppercase letter outside GMN".into());
    }
    Ok((s.as_bytes()[0] - b'A') as usize)
}
impl Config {
    pub fn radius_tolerance(&self) -> RadiusTolerance {
        RadiusTolerance {
            absolute_mm: self.radius_tolerance_mm,
            relative: self.radius_relative_tolerance,
        }
    }
}
impl Decoder {
    pub fn new(config: Config) -> Result<Self, String> {
        if config.commands.len() > 128 || !config.radius_tolerance().valid() {
            return Err("invalid configured motion decoder limits".into());
        }
        let axes = [
            address(&config.axes[0])?,
            address(&config.axes[1])?,
            address(&config.axes[2])?,
        ];
        let centers = [
            address(&config.centers[0])?,
            address(&config.centers[1])?,
            address(&config.centers[2])?,
        ];
        let feed = address(&config.feed_word)?;
        if axes
            .into_iter()
            .chain(centers)
            .chain([feed])
            .collect::<HashSet<_>>()
            .len()
            != 7
        {
            return Err("configured motion addresses must be distinct".into());
        }
        let mut numeric = HashMap::new();
        let mut keywords = HashSet::new();
        let radius_tolerance = config.radius_tolerance();
        for rule in config.commands {
            if rule.code.is_empty() || rule.code.len() > 128 || !rule.code.is_ascii() {
                return Err("invalid decoder command".into());
            }
            let bytes = rule.code.as_bytes();
            let mode = match rule.effect.as_str() {
                "rapid" => 0,
                "linear" => 1,
                "arc_cw" => 2,
                "arc_ccw" => 3,
                _ => 4,
            };
            if bytes.len() > 1 && bytes[0].is_ascii_uppercase() && bytes[1].is_ascii_digit() {
                if !bytes[1..].iter().all(|b| b.is_ascii_digit() || *b == b'.') {
                    return Err("invalid numeric command".into());
                }
                let value: f64 = rule.code[1..]
                    .parse()
                    .map_err(|_| "invalid numeric command")?;
                if !value.is_finite() || numeric.insert(key(bytes[0], value), mode).is_some() {
                    return Err("duplicate/invalid numeric command".into());
                }
            } else if bytes.iter().all(|b| b.is_ascii_uppercase() || *b == b'_') {
                if !keywords.insert(rule.code) {
                    return Err("duplicate keyword".into());
                }
            } else {
                return Err("invalid decoder command syntax".into());
            }
        }
        Ok(Self {
            compiled: Arc::new(Compiled {
                numeric,
                keywords,
                axes,
                centers,
                feed,
                comments: config.parentheses_comments,
                radius_tolerance,
            }),
        })
    }
    pub fn decode(
        &self,
        line: &str,
        start: Vec3,
        units: Option<f64>,
        plane: Option<Plane>,
    ) -> Result<Option<Decoded>, String> {
        if line.len() > 1024
            || !line.is_ascii()
            || line.bytes().any(|b| b < 32 && b != b'\t' || b == 127)
        {
            return Err("invalid NC physical block".into());
        }
        let c = &self.compiled;
        let before_comment = line.split(';').next().unwrap().to_ascii_uppercase();
        let bytes = before_comment.as_bytes();
        let mut body = Vec::with_capacity(bytes.len());
        let mut i = 0;
        while i < bytes.len() {
            if bytes[i] == b'(' {
                if !c.comments {
                    return Err("parentheses are not comments in this dialect".into());
                }
                i += 1;
                while i < bytes.len() && bytes[i] != b')' {
                    if bytes[i] == b'(' {
                        return Err("nested NC comment".into());
                    }
                    i += 1;
                }
                if i == bytes.len() {
                    return Err("unclosed NC comment".into());
                }
                body.push(b' ');
            } else if bytes[i] == b')' {
                return Err("unmatched NC comment".into());
            } else {
                body.push(bytes[i]);
            }
            i += 1;
        }
        let body = std::str::from_utf8(&body).unwrap();
        if c.keywords.contains(body.trim()) || body.trim().is_empty() {
            return Ok(None);
        }
        let bytes = body.as_bytes();
        let mut i = 0;
        let mut mode = None;
        let mut effects = 0;
        let mut fallback = false;
        let mut values = [None; 26];
        while i < bytes.len() {
            if bytes[i].is_ascii_whitespace() {
                i += 1;
                continue;
            }
            if !bytes[i].is_ascii_uppercase() {
                return Err("unsupported NC syntax".into());
            }
            let letter = bytes[i];
            i += 1;
            while i < bytes.len() && bytes[i].is_ascii_whitespace() {
                i += 1;
            }
            let begin = i;
            if i < bytes.len() && matches!(bytes[i], b'+' | b'-') {
                i += 1;
            }
            let mut digits = 0;
            while i < bytes.len() && bytes[i].is_ascii_digit() {
                digits += 1;
                i += 1;
            }
            if i < bytes.len() && bytes[i] == b'.' {
                i += 1;
                while i < bytes.len() && bytes[i].is_ascii_digit() {
                    digits += 1;
                    i += 1;
                }
            }
            if digits == 0 {
                return Err("invalid NC number".into());
            }
            let value: f64 = body[begin..i].parse().map_err(|_| "invalid NC number")?;
            if !value.is_finite() || value.abs() > 1e9 {
                return Err("invalid NC value".into());
            }
            if let Some(effect) = c.numeric.get(&key(letter, value)) {
                effects += 1;
                mode = Some(*effect);
            } else if b"GM".contains(&letter) {
                fallback = true;
            } else if letter == b'N' {
                if value < 0. || value.fract() != 0. {
                    return Err("invalid line number".into());
                }
            } else {
                let slot = &mut values[(letter - b'A') as usize];
                if slot.replace(value).is_some() {
                    return Err("duplicate NC word".into());
                }
            }
        }
        if fallback || effects != 1 || mode.is_none_or(|m| m > 3) {
            return Ok(None);
        }
        let mode = mode.unwrap();
        let unit = units
            .filter(|u| *u == 1. || *u == 25.4)
            .ok_or("NC motion with unknown units")?;
        let mut end = start;
        for (i, axis) in c.axes.iter().enumerate() {
            if let Some(v) = values[*axis].take() {
                end[i] = v * unit;
            }
        }
        let feed = if mode == 0 {
            None
        } else {
            let f = values[c.feed]
                .take()
                .ok_or("NC motion needs positive explicit feed")?;
            if f <= 0. {
                return Err("NC motion needs positive explicit feed".into());
            }
            Some(f * unit)
        };
        let curve = if mode <= 1 {
            Curve::Line { start, end }
        } else {
            let plane = plane.ok_or("NC motion with unknown plane")?;
            let [u, v, a] = plane.axes();
            let mut center = start;
            let mut offset = [0.; 3];
            for i in [u, v] {
                offset[i] = values[c.centers[i]]
                    .take()
                    .ok_or("NC arc needs both center offsets")?
                    * unit;
                center[i] += offset[i];
            }
            let x = start[u] - center[u];
            let y = start[v] - center[v];
            let ex = end[u] - center[u];
            let ey = end[v] - center[v];
            let (r1, r2, error) = arc_radii(
                [start[u], start[v]],
                [end[u], end[v]],
                [offset[u], offset[v]],
                unit,
            );
            if r1 < 1e-6 || !c.radius_tolerance.accepts_error(error, r1.max(r2)) {
                return Err(
                    "NC arc radius mismatch/collapse exceeds declared decoder model".into(),
                );
            }
            let angle = (x * ey - y * ex).atan2(x * ex + y * ey);
            let tau = std::f64::consts::TAU;
            let mut sweep = if mode == 3 {
                angle.rem_euclid(tau)
            } else {
                -(-angle).rem_euclid(tau)
            };
            if sweep == 0. {
                sweep = if mode == 3 { tau } else { -tau };
            }
            if end[a] == start[a] {
                Curve::Arc {
                    start,
                    end,
                    center,
                    plane,
                    sweep_rad: sweep,
                }
            } else {
                Curve::Helix {
                    start,
                    end,
                    center,
                    plane,
                    sweep_rad: sweep,
                }
            }
        };
        if values.iter().any(Option::is_some) {
            return Err("unused/unsupported NC words".into());
        }
        Ok(Some(Decoded {
            curve,
            feed,
            rapid: mode == 0,
        }))
    }
}

/// Bounded dimensional formatting helper, separate from actual-word decoding.
/// Reuses the existing binary64 formatter; no controller brand or motion policy.
pub fn format_words(
    addresses: &str,
    values: &[f64],
    unit: f64,
    decimals: usize,
) -> Result<Vec<String>, String> {
    if addresses.len() != values.len()
        || values.len() > 32
        || !addresses.bytes().all(|b| b.is_ascii_uppercase())
        || ![1., 25.4].contains(&unit)
        || !(2..=9).contains(&decimals)
    {
        return Err("invalid bounded post word format".into());
    }
    addresses
        .bytes()
        .zip(values)
        .map(|(a, v)| {
            if !v.is_finite() || v.abs() > 1e6 {
                return Err("invalid post number".into());
            }
            let text = crate::post::format_number(v / unit, decimals)?;
            let text = text.trim_end_matches('0').trim_end_matches('.');
            Ok(format!("{}{text}", a as char))
        })
        .collect()
}
