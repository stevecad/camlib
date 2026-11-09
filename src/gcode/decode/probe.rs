//! Probing cycles read back from their blocks: a port of the Python
//! `GCodeDecoder._probe_block` and its LinuxCNC G38.x and Renishaw Inspection
//! Plus readers. The moves come from the written words alone; only
//! attribution and the feature description come from the source event, and
//! every block must match one form exactly. Shares no code with the
//! formatter (`super::super::post`).
use super::{CycleItem, Decoder};
use crate::motion::{Event, MoveKind, Vec3};
use crate::probe::{Axis, ProbeCycle, ProbeFeature, ProbeResult};
use crate::tool::{CutterShape, ToolAssembly};

/// A decoded move: kind, end, feed (None: rapid).
type Move = (MoveKind, Vec3, Option<f64>);

/// `[+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)`, the probing number grammar.
fn number(text: &str) -> Option<f64> {
    let body = text.strip_prefix(['+', '-']).unwrap_or(text);
    unsigned(body)?;
    text.parse().ok()
}
/// `[0-9]+(?:\.[0-9]*)?|\.[0-9]+`.
fn unsigned(text: &str) -> Option<f64> {
    let (int, frac) = match text.split_once('.') {
        Some((i, f)) => (i, Some(f)),
        None => (text, None),
    };
    let digits = |s: &str| s.bytes().all(|b| b.is_ascii_digit());
    let ok = digits(int)
        && frac.is_none_or(digits)
        && (!int.is_empty() || frac.is_some_and(|f| !f.is_empty()));
    if ok { text.parse().ok() } else { None }
}
/// `[0-9]+`, as an integer.
fn integer(text: &str) -> Option<u64> {
    (!text.is_empty() && text.bytes().all(|b| b.is_ascii_digit()))
        .then(|| text.parse().ok())
        .flatten()
}
/// One address letter and its number (`[A-Z]` + the probing number).
fn word(text: &str) -> Option<(char, &str)> {
    let mut chars = text.chars();
    let letter = chars.next().filter(char::is_ascii_uppercase)?;
    let rest = chars.as_str();
    number(rest)?;
    Some((letter, rest))
}

/// Distinct values (exact equality), as the Python sets.
#[derive(Default)]
struct Seen(Vec<f64>);
impl Seen {
    fn add(&mut self, v: f64) {
        if !self.0.iter().any(|w| w.to_bits() == v.to_bits()) {
            self.0.push(v);
        }
    }
    fn one(&self, what: &str) -> Result<Option<f64>, String> {
        if self.0.len() > 1 {
            return Err(format!("one {what} per probing cycle"));
        }
        Ok(self.0.first().copied())
    }
}
#[derive(Default)]
struct Feeds {
    position: Seen,
    measure: Seen,
    back: Seen,
}

fn feature_kind(feature: &ProbeFeature) -> &'static str {
    match feature {
        ProbeFeature::Surface { .. } => "surface",
        ProbeFeature::OutsideCorner { .. } => "outside_corner",
        ProbeFeature::InsideCorner { .. } => "inside_corner",
        ProbeFeature::Bore { .. } => "bore",
        ProbeFeature::Boss { .. } => "boss",
        ProbeFeature::Pocket { .. } => "pocket",
        ProbeFeature::Web { .. } => "web",
        ProbeFeature::ToolLength { .. } => "tool_length",
        ProbeFeature::ToolDiameter { .. } => "tool_diameter",
    }
}
fn feature_axis(feature: &ProbeFeature) -> Option<usize> {
    match feature {
        ProbeFeature::Surface { axis, .. }
        | ProbeFeature::Pocket { axis, .. }
        | ProbeFeature::Web { axis, .. } => Some(axis.index()),
        _ => None,
    }
}
/// `GCodeDecoder._probe_axes`: the axes a feature's correction uses.
fn probe_axes(feature: &ProbeFeature) -> Vec<usize> {
    match feature {
        ProbeFeature::Surface { .. } | ProbeFeature::Pocket { .. } | ProbeFeature::Web { .. } => {
            feature_axis(feature).into_iter().collect()
        }
        ProbeFeature::OutsideCorner { .. }
        | ProbeFeature::InsideCorner { .. }
        | ProbeFeature::Bore { .. }
        | ProbeFeature::Boss { .. } => vec![0, 1],
        _ => vec![],
    }
}

impl Decoder {
    /// The program's tools: probe tools and every cutter radius.
    pub fn set_probing_tools(&mut self, tools: &[ToolAssembly]) {
        self.probes = tools
            .iter()
            .filter(|t| t.cutter.shape == CutterShape::Probe)
            .map(|t| t.id.clone())
            .collect();
        self.tool_radii = tools
            .iter()
            .map(|t| (t.id.clone(), t.cutter.diameter_mm / 2.))
            .collect();
    }

    /// `GCodeDecoder.read` for a probing event's blocks: the decoded cycle,
    /// with its moves queued for the events it covers.
    pub fn read_probe(
        &mut self,
        lines: &[String],
        cycle: &ProbeCycle,
        covers: u32,
    ) -> Result<Vec<Event>, String> {
        if !self.cycle_queue.is_empty() || self.cycle_active || self.tap_return.is_some() {
            return Err("a probing cycle cannot start inside another cycle".into());
        }
        if self.ended {
            return Err("NC after program end".into());
        }
        self.validate_initialized()?;
        let Some(active) = self
            .active
            .clone()
            .filter(|_| self.length.is_some() && self.offset.is_some() && !self.tool_pending)
        else {
            return Err("NC probing with unknown tool/length/offset".into());
        };
        let back_end = self
            .machine
            .dialect
            .probing
            .clone()
            .ok_or("this dialect declares no probing back end")?;
        let radius = *self
            .tool_radii
            .get(&active.tool)
            .ok_or("NC probing with unknown tool/length/offset")?;
        let setting = matches!(
            cycle.feature,
            ProbeFeature::ToolLength { .. } | ProbeFeature::ToolDiameter { .. }
        );
        if self.probes.contains(&active.tool) == setting {
            return Err("probing needs the probe active; tool setting a cutting tool".into());
        }
        if lines.iter().any(|l| {
            l.len() > 1024
                || l.bytes()
                    .any(|b| !(b == b'\t' || (0x20..=0x7e).contains(&b)))
        }) {
            return Err("invalid NC physical block".into());
        }
        let body: Vec<String> = lines
            .iter()
            .map(|l| l.trim().to_ascii_uppercase())
            .collect();
        let (moves, decoded) = if back_end == "g38" {
            self.probe_g38_read(&body, cycle, radius)?
        } else {
            self.probe_inspection_plus_read(&body, cycle, radius)?
        };
        if moves.len() != covers as usize {
            return Err(format!(
                "the probing blocks decode to {} moves, the cycle covers {covers}",
                moves.len()
            ));
        }
        self.cycle_queue = moves
            .into_iter()
            .map(|(kind, end, feed)| CycleItem::Probe(kind, end, feed))
            .collect();
        Ok(vec![Event::Probe {
            cycle: decoded,
            covers,
        }])
    }

    fn unit(&self) -> Result<f64, String> {
        self.units.ok_or_else(|| "NC probing before units".into())
    }

    /// `GCodeDecoder._probe_g38_read`: LinuxCNC G38.x probing blocks.
    fn probe_g38_read(
        &mut self,
        body: &[String],
        cycle: &ProbeCycle,
        radius: f64,
    ) -> Result<(Vec<Move>, ProbeCycle), String> {
        let d = self.machine.dialect.clone();
        let unit = self.unit()?;
        let axis_of = |c: char| d.axes.iter().position(|a| a.len() == 1 && a.starts_with(c));
        let feed_letter = d
            .feed_word
            .chars()
            .next()
            .filter(|_| d.feed_word.len() == 1);
        // ` A<n> B<n> ...` (axis words, distinct, at least one) from `pos`.
        let coordinates = |words: &[&str], pos: Vec3| -> Result<Vec3, String> {
            let invalid = || "invalid probing axis words".to_string();
            let mut end = pos;
            let mut seen = vec![];
            if words.is_empty() {
                return Err(invalid());
            }
            for w in words {
                let (letter, value) = word(w).ok_or_else(invalid)?;
                let axis = axis_of(letter).ok_or_else(invalid)?;
                if seen.contains(&axis) {
                    return Err(invalid());
                }
                seen.push(axis);
                end[axis] = value.parse::<f64>().map_err(|_| invalid())? * unit;
            }
            if end == pos {
                return Err("a probing move has zero length".into());
            }
            Ok(end)
        };
        // The feed word closing a block.
        let feed_of = |w: &str| -> Option<f64> {
            let (letter, value) = word(w)?;
            (Some(letter) == feed_letter).then(|| value.parse::<f64>().ok())?
        };
        let mut moves: Vec<Move> = vec![];
        let mut touches: Vec<(usize, i8)> = vec![];
        let mut feeds = Feeds::default();
        let mut search = Seen::default();
        let mut pos = self.position;
        let mut labels = self.probe_labels.clone();
        let mut result: Option<ProbeResult> = None;
        let mut i = 0;
        while i < body.len() {
            let line = body[i].as_str();
            if result.is_some() {
                return Err("nothing follows the stored probing result".into());
            }
            let tokens: Vec<&str> = line.split(' ').collect();
            let next = |k: usize| body.get(i + k).map(String::as_str).unwrap_or("");
            match tokens.as_slice() {
                ["G38.3", axes @ .., f] if !axes.is_empty() && feed_of(f).is_some() => {
                    let end = coordinates(axes, pos)?;
                    let label = next(1)
                        .strip_prefix('O')
                        .and_then(|r| r.strip_suffix(" IF [#5070 NE 0]"))
                        .filter(|l| integer(l).is_some())
                        .map(str::to_string);
                    let refused = || {
                        "a protected probe move (G38.3) must stop the program when the probe trips"
                            .to_string()
                    };
                    let label = label.ok_or_else(refused)?;
                    if next(2) != "(ABORT, CAMLIB: PROTECTED PROBE MOVE OBSTRUCTED)"
                        || next(3) != format!("O{label} ENDIF")
                        || labels.contains(&label)
                    {
                        return Err(refused());
                    }
                    labels.insert(label);
                    let feed = feed_of(f).unwrap() * unit;
                    feeds.position.add(feed);
                    moves.push((MoveKind::ProbePosition, end, Some(feed)));
                    pos = end;
                    i += 4;
                }
                ["G38.2", w, f]
                    if word(w).is_some_and(|(l, _)| axis_of(l).is_some())
                        && feed_of(f).is_some() =>
                {
                    let (letter, value) = word(w).unwrap();
                    let axis = axis_of(letter).unwrap();
                    let target = value.parse::<f64>().map_err(|e| e.to_string())? * unit;
                    let refused = || {
                        "a measuring move (G38.2) is followed by its error against the expected contact"
                            .to_string()
                    };
                    // `#<PROBEn> = [#506a s v]`
                    let err = next(1);
                    let parsed = (|| {
                        let rest = err.strip_prefix("#<PROBE")?;
                        let (n, rest) = rest.split_once('>')?;
                        let rest = rest.strip_prefix(" = [#506")?;
                        let mut chars = rest.chars();
                        let a = chars.next()?.to_digit(10)?;
                        let rest = chars.as_str().strip_prefix(' ')?;
                        let mut chars = rest.chars();
                        let sign = chars.next()?;
                        let v = chars.as_str().strip_prefix(' ')?.strip_suffix(']')?;
                        ((1..=3).contains(&a) && (sign == '+' || sign == '-')).then_some((
                            integer(n)?,
                            a as usize,
                            sign,
                            unsigned(v)?,
                        ))
                    })();
                    let Some((n, a, sign, v)) = parsed else {
                        return Err(refused());
                    };
                    if n != touches.len() as u64 + 1 || a != axis + 1 {
                        return Err(refused());
                    }
                    let contact = if sign == '+' { -1. } else { 1. } * v * unit;
                    let direction = if target > contact { 1. } else { -1. };
                    let reach = direction * (target - contact);
                    if reach <= 0. || direction * (contact - pos[axis]) <= reach {
                        return Err(
                            "the measuring move must start farther from the contact than its search"
                                .into(),
                        );
                    }
                    search.add(reach);
                    let feed = feed_of(f).unwrap() * unit;
                    feeds.measure.add(feed);
                    let (mut near, mut end) = (pos, pos);
                    near[axis] = contact - direction * reach;
                    end[axis] = contact;
                    moves.push((MoveKind::ProbeMeasure, near, Some(feed)));
                    moves.push((MoveKind::ProbeTouch, end, Some(feed)));
                    touches.push((axis, if direction > 0. { 1 } else { -1 }));
                    pos = end;
                    i += 2;
                }
                ["G1", axes @ .., f] if !axes.is_empty() && feed_of(f).is_some() => {
                    let end = coordinates(axes, pos)?;
                    let feed = feed_of(f).unwrap() * unit;
                    feeds.back.add(feed);
                    moves.push((MoveKind::ProbeReturn, end, Some(feed)));
                    pos = end;
                    i += 1;
                }
                ["G0", axes @ ..] if !axes.is_empty() => {
                    let end = coordinates(axes, pos)?;
                    moves.push((MoveKind::Rapid, end, None));
                    pos = end;
                    i += 1;
                }
                ["G10", "L2", p, ..]
                    if p.len() == 2
                        && p.starts_with('P')
                        && matches!(p.as_bytes()[1], b'1'..=b'9')
                        && line.len() > "G10 L2 P1".len() =>
                {
                    let number = p.as_bytes()[1] - b'0';
                    let rest = &line["G10 L2 P1".len()..];
                    let unsupported = || "unsupported work offset correction".to_string();
                    let offset = self
                        .offset
                        .ok_or("NC probing with unknown tool/length/offset")?;
                    let base = 5201 + 20 * offset as u64;
                    let mut used: Vec<u64> = vec![];
                    let mut axes: Vec<usize> = vec![];
                    let mut text = rest;
                    while !text.is_empty() {
                        // ` A[#p + E]`, E = `#<PROBEn>` or
                        // `[#<PROBEn> + #<PROBEm>] / 2`.
                        let part = (|| {
                            let t = text.strip_prefix(' ')?;
                            let mut chars = t.chars();
                            let letter = chars.next().filter(char::is_ascii_uppercase)?;
                            let t = chars.as_str().strip_prefix("[#")?;
                            let (param, t) = t.split_once(" + ")?;
                            let param = integer(param)?;
                            let probe = |t: &str| -> Option<(u64, usize)> {
                                let r = t.strip_prefix("#<PROBE")?;
                                let (n, _) = r.split_once('>')?;
                                Some((integer(n)?, "#<PROBE".len() + n.len() + 1))
                            };
                            let (refs, t) = if let Some(inner) = t.strip_prefix('[') {
                                let (a, la) = probe(inner)?;
                                let inner = inner[la..].strip_prefix(" + ")?;
                                let (b, lb) = probe(inner)?;
                                (vec![a, b], inner[lb..].strip_prefix("] / 2")?)
                            } else {
                                let (a, la) = probe(t)?;
                                (vec![a], &t[la..])
                            };
                            let t = t.strip_prefix(']')?;
                            Some((letter, param, refs, t))
                        })();
                        let (letter, param, refs, remaining) = part.ok_or_else(unsupported)?;
                        text = remaining;
                        let axis = axis_of(letter)
                            .filter(|a| param == base + *a as u64 && !axes.contains(a));
                        let Some(axis) = axis else {
                            return Err(
                                "a work offset correction adds each axis error to the active offset"
                                    .into(),
                            );
                        };
                        if refs.iter().any(|&r| {
                            r < 1 || r as usize > touches.len() || touches[r as usize - 1].0 != axis
                        }) {
                            return Err(
                                "a work offset axis averages the touches along that axis".into()
                            );
                        }
                        if refs.len() == 2
                            && touches[refs[0] as usize - 1].1 == touches[refs[1] as usize - 1].1
                        {
                            return Err(
                                "two touches averaged on one axis must face each other".into()
                            );
                        }
                        axes.push(axis);
                        used.extend(refs);
                    }
                    used.sort();
                    let mut sorted = axes.clone();
                    sorted.sort();
                    if used != (1..=touches.len() as u64).collect::<Vec<_>>()
                        || sorted != probe_axes(&cycle.feature)
                    {
                        return Err(format!(
                            "the correction does not use every touch of the {}",
                            feature_kind(&cycle.feature)
                        ));
                    }
                    let pairs = matches!(
                        cycle.feature,
                        ProbeFeature::Bore { .. }
                            | ProbeFeature::Boss { .. }
                            | ProbeFeature::Pocket { .. }
                            | ProbeFeature::Web { .. }
                    );
                    if axes.iter().any(|a| {
                        touches.iter().filter(|t| t.0 == *a).count() != if pairs { 2 } else { 1 }
                    }) {
                        return Err("wrong number of touches for the probed feature".into());
                    }
                    result = Some(ProbeResult::WorkOffset { number });
                    i += 1;
                }
                ["G10", "L10", p, z, "-", "#<PROBE1>]"]
                    if p.strip_prefix('P').and_then(integer).is_some()
                        && *z == format!("{}[#5422", d.axes[2]) =>
                {
                    let register = p.strip_prefix('P').and_then(integer).unwrap();
                    let reload = format!("{} {}{register}", d.code("length_on")?, d.length_word);
                    if touches != [(2, -1)]
                        || Some(register) != self.length.map(u64::from)
                        || next(1) != reload
                    {
                        return Err(
                            "a tool length is measured down onto the setter and reloaded (G43 H) at once"
                                .into(),
                        );
                    }
                    result = Some(ProbeResult::ToolLength {
                        register: register as u32,
                    });
                    i += 2;
                }
                [
                    "G10",
                    "L1",
                    p,
                    r,
                    "+",
                    "[#<PROBE1>",
                    "-",
                    "#<PROBE2>]",
                    "/",
                    "2]",
                ] if p.strip_prefix('P').and_then(integer).is_some()
                    && r.strip_prefix("R[").and_then(number).is_some() =>
                {
                    let register = p.strip_prefix('P').and_then(integer).unwrap();
                    if touches != [(0, -1), (0, 1)] || Some(register) != self.length.map(u64::from)
                    {
                        return Err(
                            "a tool radius is measured from +X, then from -X, into the tool entry"
                                .into(),
                        );
                    }
                    let stored = r.strip_prefix("R[").and_then(number).unwrap() * unit;
                    let half = 0.5 * 10f64.powi(-(self.machine.decimals as i32)) * unit + 1e-9;
                    if (stored - radius).abs() > half {
                        return Err(
                            "the stored tool radius starts from the tool's declared radius".into(),
                        );
                    }
                    result = Some(ProbeResult::ToolRadius {
                        register: register as u32,
                    });
                    i += 1;
                }
                _ => return Err(format!("unsupported block in a probing cycle: {line:?}")),
            }
        }
        let result = result.ok_or("a probing cycle stores its result")?;
        self.probe_labels = labels;
        Ok((moves, decoded(cycle, &search, &feeds, result)?))
    }

    /// `GCodeDecoder._probe_inspection_plus_read`: Renishaw Inspection Plus
    /// macro calls (H-2000-6031), read from their words with the macros'
    /// documented motion: O9811 measures from its start along the axis of
    /// its X, Y or Z word toward that surface position and returns there;
    /// O9814 measures +X, -X, +Y, -Y from the bore centre; O9812 + and -
    /// along its axis from the centre; O9815 the Y wall, then the X wall,
    /// from its start, and returns there.
    fn probe_inspection_plus_read(
        &mut self,
        body: &[String],
        cycle: &ProbeCycle,
        radius: f64,
    ) -> Result<(Vec<Move>, ProbeCycle), String> {
        let d = self.machine.dialect.clone();
        let unit = self.unit()?;
        let fast = d
            .probe_fast_feed_mm_min
            .ok_or("Inspection Plus probing needs the installed fast feed #119")?;
        let mut body = body;
        if d.probe_switching {
            if body.first().map(String::as_str) != Some("G65 P9832")
                || body.last().map(String::as_str) != Some("G65 P9833")
                || body.len() < 2
            {
                return Err(
                    "this installation switches the probe on (O9832) and off (O9833) around a cycle"
                        .into(),
                );
            }
            body = &body[1..body.len() - 1];
        }
        let axis_of = |c: char| d.axes.iter().position(|a| a.len() == 1 && a.starts_with(c));
        let feed_letter = d
            .feed_word
            .chars()
            .next()
            .filter(|_| d.feed_word.len() == 1);
        let mut moves: Vec<Move> = vec![];
        let mut calls: Vec<(&str, Option<usize>)> = vec![];
        let mut feeds = Feeds::default();
        let mut search = Seen::default();
        let mut offsets: Vec<u8> = vec![];
        let pos = self.position;
        let mut pos = pos;
        let mut position_feed: Option<f64> = None;
        // Words of a call: letter -> value, each allowed letter once;
        // `decimal` letters need their decimal point.
        let words = |text: &str,
                     allowed: &dyn Fn(char) -> bool,
                     decimal: &dyn Fn(char) -> bool|
         -> Result<Vec<(char, f64)>, String> {
            let mut values: Vec<(char, f64)> = vec![];
            for w in text.split(' ') {
                let parsed =
                    word(w).filter(|(l, _)| allowed(*l) && !values.iter().any(|v| v.0 == *l));
                let Some((letter, value)) = parsed else {
                    return Err(format!("unsupported Inspection Plus word {w:?}"));
                };
                if decimal(letter) && !value.contains('.') {
                    return Err(
                        "Inspection Plus dimensional arguments need their decimal point".into(),
                    );
                }
                values.push((letter, value.parse().map_err(|_| "invalid NC value")?));
            }
            Ok(values)
        };
        for line in body {
            let macro_text = line.strip_prefix("G65 P").filter(|r| {
                r.len() >= 4
                    && r.starts_with("98")
                    && r.as_bytes()[2..4].iter().all(u8::is_ascii_digit)
            });
            let Some(rest) = macro_text else {
                return Err(format!("unsupported block in a probing cycle: {line:?}"));
            };
            let (code, text) = rest.split_at(4);
            // `((?: \S+)*)`: each word after one space, none empty.
            if !text.is_empty()
                && (!text.starts_with(' ')
                    || text[1..]
                        .split(' ')
                        .any(|w| w.is_empty() || w.contains(char::is_whitespace)))
            {
                return Err(format!("unsupported block in a probing cycle: {line:?}"));
            }
            let text = text.trim();
            if code == "9810" {
                let is_axis = |c: char| axis_of(c).is_some() || Some(c) == feed_letter;
                let mut values = words(text, &is_axis, &is_axis)?;
                if let Some(k) = values.iter().position(|v| Some(v.0) == feed_letter) {
                    position_feed = Some(values.remove(k).1 * unit);
                }
                let Some(feed) = position_feed.filter(|_| !values.is_empty()) else {
                    return Err("O9810 needs a target and its feed".into());
                };
                let mut end = pos;
                for (letter, v) in &values {
                    end[axis_of(*letter).unwrap()] = v * unit;
                }
                if end == pos {
                    return Err("a probing move has zero length".into());
                }
                feeds.position.add(feed);
                moves.push((MoveKind::ProbePosition, end, Some(feed)));
                pos = end;
                continue;
            }
            let allowed = |c: char| axis_of(c).is_some() || matches!(c, 'D' | 'S' | 'Q');
            let decimal = |c: char| axis_of(c).is_some() || matches!(c, 'D' | 'Q');
            let mut values = words(text, &allowed, &decimal)?;
            let take = |values: &mut Vec<(char, f64)>, c: char| -> Option<f64> {
                values
                    .iter()
                    .position(|v| v.0 == c)
                    .map(|k| values.remove(k).1)
            };
            let s_word = values.iter().find(|v| v.0 == 'S').map(|v| v.1);
            let has_q = values.iter().any(|v| v.0 == 'Q');
            let Some(s_value) =
                s_word.filter(|s| has_q && *s == s.trunc() && (1. ..=6.).contains(s))
            else {
                return Err(
                    "camlib Inspection Plus calls correct work offset S1..S6 with an explicit overtravel Q"
                        .into(),
                );
            };
            take(&mut values, 'S');
            let offset = s_value as u8;
            if !offsets.contains(&offset) {
                offsets.push(offset);
            }
            let s = take(&mut values, 'Q').unwrap() * unit;
            if s.is_nan() || s <= 0. {
                return Err("the overtravel Q must be positive".into());
            }
            search.add(s);
            let start = pos;
            let axes: Vec<(usize, f64)> = values
                .iter()
                .filter_map(|(l, v)| axis_of(*l).map(|a| (a, v * unit)))
                .collect();
            let has_d = values.iter().any(|v| v.0 == 'D');
            let mut touch = |axis: usize, contact: f64, direction: f64| -> Result<(), String> {
                if direction * (contact - pos[axis]) <= s {
                    return Err(
                        "the measuring move must start farther from the surface than the overtravel"
                            .into(),
                    );
                }
                let (mut near, mut end) = (pos, pos);
                near[axis] = contact - direction * s;
                end[axis] = contact;
                let measure = 0.6 * fast;
                let back = if axis == 2 { 0.6 } else { 1. } * fast;
                feeds.measure.add(measure);
                feeds.back.add(back);
                moves.push((MoveKind::ProbeMeasure, near, Some(measure)));
                moves.push((MoveKind::ProbeTouch, end, Some(measure)));
                moves.push((MoveKind::ProbeReturn, pos, Some(back)));
                Ok(())
            };
            if code == "9811" && axes.len() == 1 && !has_d {
                let (axis, surface) = axes[0];
                if axis == 2 {
                    if surface >= pos[2] {
                        return Err("O9811 Z measures down onto a surface below the probe".into());
                    }
                    touch(2, surface, -1.)?;
                } else {
                    let direction = if surface > pos[axis] { 1. } else { -1. };
                    touch(axis, surface - direction * radius, direction)?;
                }
                calls.push(("9811", Some(axis)));
            } else if code == "9814" && values.len() == 1 && has_d {
                let diameter = values[0].1 * unit;
                let reach = diameter / 2. - radius;
                for (axis, direction) in [(0, 1.), (0, -1.), (1, 1.), (1, -1.)] {
                    touch(axis, start[axis] + direction * reach, direction)?;
                }
                calls.push(("9814", None));
            } else if code == "9812" && axes.len() == 1 && !has_d && axes[0].0 != 2 {
                let (axis, width) = axes[0];
                for direction in [1., -1.] {
                    touch(
                        axis,
                        start[axis] + direction * (width / 2. - radius),
                        direction,
                    )?;
                }
                calls.push(("9812", Some(axis)));
            } else if code == "9815"
                && axes.len() == 2
                && axes.iter().any(|a| a.0 == 0)
                && axes.iter().any(|a| a.0 == 1)
                && !has_d
            {
                for axis in [1, 0] {
                    let value = axes.iter().find(|a| a.0 == axis).unwrap().1;
                    let direction = if value > start[axis] { 1. } else { -1. };
                    touch(axis, value - direction * radius, direction)?;
                }
                calls.push(("9815", None));
            } else {
                return Err(format!(
                    "G65 P{code} {text} is not an Inspection Plus call camlib models"
                ));
            }
        }
        let kind = feature_kind(&cycle.feature);
        let axis = feature_axis(&cycle.feature);
        let expected: Option<Vec<(&str, Option<usize>)>> = match cycle.feature {
            ProbeFeature::Surface { .. } => Some(vec![("9811", axis)]),
            ProbeFeature::OutsideCorner { .. } => Some(vec![("9811", Some(1)), ("9811", Some(0))]),
            ProbeFeature::InsideCorner { .. } => Some(vec![("9815", None)]),
            ProbeFeature::Bore { .. } => Some(vec![("9814", None)]),
            ProbeFeature::Pocket { axis: a, .. } if a != Axis::Z => Some(vec![("9812", axis)]),
            _ => None,
        };
        if expected.as_ref() != Some(&calls) {
            return Err(format!(
                "these Inspection Plus calls do not measure a {kind}"
            ));
        }
        let [number] = offsets[..] else {
            return Err("one work offset per probing cycle".into());
        };
        let result = ProbeResult::WorkOffset { number };
        Ok((moves, decoded(cycle, &search, &feeds, result)?))
    }
}

/// `GCodeDecoder._probe_decoded`: the source cycle with what its blocks
/// actually say (search, feeds, stored result).
fn decoded(
    cycle: &ProbeCycle,
    search: &Seen,
    feeds: &Feeds,
    result: ProbeResult,
) -> Result<ProbeCycle, String> {
    let search_mm = search.one("search distance")?;
    let measure = feeds.measure.one("measuring feed")?;
    let back = feeds.back.one("return feed")?;
    let position = feeds.position.one("protected positioning feed")?;
    let (Some(search_mm), Some(measure), Some(back)) = (search_mm, measure, back) else {
        return Err("a probing cycle measures at least once and returns".into());
    };
    let mut decoded = cycle.clone();
    decoded.search_mm = search_mm;
    decoded.result = result;
    decoded.measure_feed_mm_min = measure;
    decoded.return_feed_mm_min = back;
    decoded.position_feed_mm_min = position;
    Ok(decoded)
}

#[cfg(test)]
mod tests {
    //! The decoder alone, on the blocks the Python post writes
    //! (python/tests/test_probing.py golden programs) and tampered copies.
    use super::super::Decoder;
    use crate::gcode::Machine;
    use crate::motion::Event;
    use crate::probe::ProbeCycle;
    use std::collections::BTreeSet;

    fn machine(probing: &str, switching: bool) -> Machine {
        let input: serde_json::Value = serde_json::from_str(include_str!(
            "../../../fixtures/posts/rust/standalone-rest.json"
        ))
        .unwrap();
        let mut m = input["machine"].clone();
        m["dialect"]["probing"] = probing.into();
        if probing == "inspection_plus" {
            m["dialect"]["probe_fast_feed_mm_min"] = 2000.into();
            m["dialect"]["probe_switching"] = switching.into();
        }
        serde_json::from_value(m).unwrap()
    }
    fn cycle(feature: serde_json::Value, measure: f64, back: f64) -> ProbeCycle {
        serde_json::from_value(serde_json::json!({
            "feature": feature,
            "retract_z_mm": 20.0,
            "search_mm": 2.0,
            "result": {"kind": "work_offset", "number": 1},
            "position_feed_mm_min": 1000.0,
            "measure_feed_mm_min": measure,
            "return_feed_mm_min": back,
        }))
        .unwrap()
    }
    fn corner(measure: f64, back: f64) -> ProbeCycle {
        cycle(
            serde_json::json!({"kind": "outside_corner", "corner_mm": [0.0, 0.0], "z_mm": 5.0,
                "material": [1, 1], "inset_mm": 5.0, "clearance_mm": 5.0}),
            measure,
            back,
        )
    }
    /// A decoder initialized with the probe (ball radius 2) active at the
    /// cycle's start, as after the program's setup blocks.
    fn decoder(m: &Machine, start: crate::motion::Vec3) -> Decoder {
        let mut d = Decoder::new(m, start, BTreeSet::new()).unwrap();
        d.ready = d.required.clone();
        d.units = Some(1.);
        d.active = Some(m.tools[0].clone());
        d.length = Some(m.tools[0].length_register);
        d.offset = Some(1);
        d.probes = [m.tools[0].tool.clone()].into();
        d.tool_radii = [(m.tools[0].tool.clone(), 2.)].into();
        d
    }
    fn protected(words: &str, label: u32) -> Vec<String> {
        vec![
            format!("G38.3 {words} F1000"),
            format!("O{label} IF [#5070 NE 0]"),
            "(ABORT, camlib: protected probe move obstructed)".into(),
            format!("O{label} ENDIF"),
        ]
    }
    fn g38_corner(first_label: u32) -> Vec<String> {
        let l = first_label;
        [
            protected("Z5", l),
            vec![
                "G38.2 Y0 F100".into(),
                "#<probe1> = [#5062 + 2]".into(),
                "G1 Y-7 F1000".into(),
            ],
            protected("Z20", l + 1),
            protected("X-7 Y5", l + 2),
            protected("Z5", l + 3),
            vec![
                "G38.2 X0 F100".into(),
                "#<probe2> = [#5061 + 2]".into(),
                "G1 X-7 F1000".into(),
            ],
            protected("Z20", l + 4),
            vec!["G10 L2 P1 X[#5221 + #<probe2>] Y[#5222 + #<probe1>]".into()],
        ]
        .concat()
    }
    fn read(m: &Machine, c: &ProbeCycle, lines: &[String]) -> Result<Vec<Event>, String> {
        let covers = c.legs(2.).unwrap().len() as u32;
        decoder(m, c.start(2.).unwrap()).read_probe(lines, c, covers)
    }

    #[test]
    fn the_python_posts_corner_decodes_to_its_cycle_and_moves() {
        let m = machine("g38", false);
        let c = corner(100., 1000.);
        let legs = c.legs(2.).unwrap();
        let mut d = decoder(&m, c.start(2.).unwrap());
        let events = d
            .read_probe(&g38_corner(100), &c, legs.len() as u32)
            .unwrap();
        assert_eq!(
            events,
            vec![Event::Probe {
                cycle: c.clone(),
                covers: legs.len() as u32
            }]
        );
        // The queued moves are the cycle's own legs.
        let queued: Vec<_> = d
            .cycle_queue
            .iter()
            .map(|item| match item {
                super::CycleItem::Probe(kind, end, feed) => (*kind, *end, *feed),
                _ => panic!("probe move expected"),
            })
            .collect();
        let expected: Vec<_> = legs
            .iter()
            .map(|l| (l.kind, l.end, l.feed_mm_min))
            .collect();
        assert_eq!(queued, expected);
        // The same labels cannot guard a second cycle.
        d.cycle_queue.clear();
        d.position = c.start(2.).unwrap();
        let again = d.read_probe(&g38_corner(100), &c, legs.len() as u32);
        assert!(again.unwrap_err().contains("must stop the program"));
    }

    #[test]
    fn tampered_g38_blocks_are_refused() {
        let m = machine("g38", false);
        let c = corner(100., 1000.);
        let golden = g38_corner(100);
        let refused = |lines: Vec<String>, message: &str| {
            let error = read(&m, &c, &lines).unwrap_err();
            assert!(error.contains(message), "{error}");
        };
        // A protected move without its trip check.
        refused(
            golden
                .iter()
                .filter(|l| !l.starts_with("O1") && !l.starts_with("(ABORT"))
                .cloned()
                .collect(),
            "must stop the program when the probe trips",
        );
        // A written contact that differs from the plan.
        refused(
            golden
                .iter()
                .map(|l| l.replace("[#5062 + 2]", "[#5062 + 2.5]"))
                .collect(),
            "search",
        );
        // Anything after the stored result; no result at all.
        refused(
            [golden.clone(), vec!["G0 Z30".into()]].concat(),
            "nothing follows",
        );
        refused(golden[..golden.len() - 1].to_vec(), "stores its result");
        // A correction that leaves a touch unused, or another offset's base.
        refused(
            [
                golden[..golden.len() - 1].to_vec(),
                vec!["G10 L2 P1 X[#5221 + #<probe2>]".into()],
            ]
            .concat(),
            "does not use every touch",
        );
        refused(
            [
                golden[..golden.len() - 1].to_vec(),
                vec!["G10 L2 P1 X[#5241 + #<probe2>] Y[#5222 + #<probe1>]".into()],
            ]
            .concat(),
            "adds each axis error to the active offset",
        );
        // Unknown blocks.
        refused(
            [vec!["M3 S1000".into()], golden.clone()].concat(),
            "unsupported block",
        );
    }

    #[test]
    fn inspection_plus_calls_decode_and_tampering_is_refused() {
        let bore = cycle(
            serde_json::json!({"kind": "bore", "center_mm": [20.0, 15.0], "z_mm": 5.0, "diameter_mm": 12.0}),
            1200.,
            2000.,
        );
        let golden: Vec<String> = [
            "G65 P9832",
            "G65 P9810 Z5. F1000.",
            "G65 P9814 D12. S1. Q2.",
            "G65 P9810 Z20. F1000.",
            "G65 P9833",
        ]
        .map(String::from)
        .into();
        let m = machine("inspection_plus", true);
        let events = read(&m, &bore, &golden).unwrap();
        assert_eq!(
            events,
            vec![Event::Probe {
                cycle: bore.clone(),
                covers: bore.legs(2.).unwrap().len() as u32
            }]
        );
        let refused = |m: &Machine, lines: Vec<String>, message: &str| {
            let error = read(m, &bore, &lines).unwrap_err();
            assert!(error.contains(message), "{error}");
        };
        // The probe is switched on and off around every cycle.
        refused(&m, golden[1..4].to_vec(), "switches the probe on");
        // Dimensional arguments carry their decimal point.
        let mut bad = golden.clone();
        bad[2] = "G65 P9814 D12 S1. Q2.".into();
        refused(&m, bad, "decimal point");
        // Another diameter measures other contacts (more moves would not
        // fit; here the moves differ, which the intent check refuses).
        let mut wider = golden.clone();
        wider[2] = "G65 P9814 D13. S1. Q2.".into();
        let mut d = decoder(&m, bore.start(2.).unwrap());
        d.read_probe(&wider, &bore, bore.legs(2.).unwrap().len() as u32)
            .unwrap();
        let ends: Vec<_> = d
            .cycle_queue
            .iter()
            .filter_map(|item| match item {
                super::CycleItem::Probe(_, end, _) => Some(*end),
                _ => None,
            })
            .collect();
        let planned: Vec<_> = bore.legs(2.).unwrap().iter().map(|l| l.end).collect();
        assert_ne!(ends, planned);
        // A call that measures another feature.
        let mut other = golden.clone();
        other[2] = "G65 P9811 X26. S1. Q2.".into();
        refused(&m, other, "do not measure a bore");
        // Without switching declared, the switching calls are refused.
        refused(
            &machine("inspection_plus", false),
            golden.clone(),
            "Inspection Plus",
        );
    }
}
