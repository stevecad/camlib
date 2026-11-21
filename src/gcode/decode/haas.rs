//! Independent Haas actual-word guard, ported from HaasNGCDecoder. Never
//! imports the formatter. The common reader reconstructs all motions/cycles.
use super::*;
#[derive(Clone, Default)]
pub(super) struct HaasState {
    opened: bool,
    number: Option<u32>,
    end_seen: bool,
    rotation_off: bool,
    scaling_off: bool,
    mirror_off: bool,
    smoothing_reset: bool,
}
impl Decoder {
    pub(super) fn haas_initialized(&self, h: &crate::gcode::haas::HaasNGC) -> Result<(), String> {
        let s = &self.haas;
        if !(s.opened
            && s.number == Some(h.program_number)
            && (s.rotation_off || !h.rotation_scaling)
            && (s.scaling_off || !h.rotation_scaling)
            && s.mirror_off
            && s.smoothing_reset)
        {
            return Err(
                "incomplete Haas program/rotation/scaling/mirror/G187 initialization".into(),
            );
        }
        Ok(())
    }
    pub(super) fn read_haas(
        &mut self,
        h: &crate::gcode::haas::HaasNGC,
        line: &str,
        context: Option<&Context>,
    ) -> Result<Vec<Event>, String> {
        if self.ended
            || line.len() > 1024
            || line
                .bytes()
                .any(|b| !(b == b'\t' || (0x20..=0x7e).contains(&b)))
        {
            return Err("invalid Haas physical block or code after closing percent".into());
        }
        let raw = line.split(';').next().unwrap().trim().to_ascii_uppercase();
        let mut body = String::new();
        let mut comment = false;
        for ch in raw.chars() {
            match ch {
                '(' if !comment => {
                    comment = true;
                    body.push(' ');
                }
                ')' if comment => comment = false,
                '(' | ')' => return Err("invalid Haas comment".into()),
                _ if !comment => body.push(ch),
                _ => (),
            }
        }
        if comment {
            return Err("invalid Haas comment".into());
        }
        let mut body = body.trim();
        if body.is_empty() {
            return Ok(vec![]);
        }
        if body == "%" {
            if !self.haas.opened {
                self.haas.opened = true;
            } else if self.haas.end_seen {
                self.ended = true;
            } else {
                return Err("closing percent requires explicit M30".into());
            }
            return Ok(vec![]);
        }
        if !self.haas.opened || self.haas.end_seen {
            return Err("Haas framing requires percent/O/program/M30/percent".into());
        }
        if let Some(number) = body.strip_prefix('O') {
            if number.is_empty()
                || number.len() > 5
                || !number.bytes().all(|b| b.is_ascii_digit())
                || number.parse::<u32>().ok() != Some(h.program_number)
                || self.haas.number.is_some()
            {
                return Err("unexpected/repeated O program".into());
            }
            self.haas.number = Some(h.program_number);
            return Ok(vec![]);
        }
        if self.haas.number.is_none() {
            return Err("Haas program number required".into());
        }
        if body.starts_with('N') {
            let end = 1 + body[1..].bytes().take_while(|b| b.is_ascii_digit()).count();
            if !(2..=6).contains(&end) {
                return Err("unsupported/duplicate Haas words".into());
            }
            body = body[end..].trim();
        }
        let mut pieces = vec![];
        let mut rest = body;
        while !rest.is_empty() {
            let address = rest.as_bytes()[0] as char;
            if !address.is_ascii_uppercase() {
                return Err("unsupported/duplicate Haas words".into());
            }
            rest = rest[1..].trim_start();
            let n = rest
                .bytes()
                .take_while(|b| b.is_ascii_digit() || b"+-.".contains(b))
                .count();
            let word = &rest[..n];
            let value = word
                .parse::<f64>()
                .map_err(|_| "unsupported/duplicate Haas words")?;
            pieces.push((address, word, value));
            rest = rest[n..].trim_start();
        }
        if h.cutter_compensation
            && pieces.iter().filter(|p| p.0 == 'G').count() == 2
            && pieces.len() >= 2
            && pieces[0].0 == 'G'
            && pieces[0].1 == "1"
            && pieces[1].0 == 'G'
            && [40., 41., 42.].contains(&pieces[1].2)
        {
            pieces.remove(1);
        }
        let values: BTreeMap<char, f64> = pieces.iter().map(|p| (p.0, p.2)).collect();
        if values.len() != pieces.len() {
            return Err("unsupported/duplicate Haas words".into());
        }
        if values.contains_key(&'N')
            || values.contains_key(&'G') && values.contains_key(&'M')
            || values.values().any(|v| !v.is_finite() || v.abs() > 1e9)
        {
            return Err("invalid/mixed Haas command".into());
        }
        let g = values.get(&'G').copied();
        let m = values.get(&'M').copied();
        if g == Some(0.) {
            return Err("G0 dogleg rejected; use configured feed traverse".into());
        }
        let precision = if self.machine.output_units == "mm" {
            3
        } else {
            4
        };
        for (a, text, v) in &pieces {
            if "XYZIJKF".contains(*a) {
                if !text.contains('.') && *v != 0. {
                    return Err("Haas nonzero dimensional words need explicit decimal points (Settings77/162)".into());
                }
                if text
                    .split_once('.')
                    .is_some_and(|(_, s)| s.len() > precision)
                {
                    return Err("Haas reference coordinate/feed precision exceeded".into());
                }
            }
            if *a == 'P' && !text.contains('.') {
                return Err("Haas dwell P without a decimal point is milliseconds; seconds need a decimal point".into());
            }
        }
        if matches!(g, Some(20. | 21.))
            && g != Some(if self.machine.output_units == "mm" {
                21.
            } else {
                20.
            })
        {
            return Err("Haas file units differ from configured output units".into());
        }
        if matches!(g, Some(52. | 100.)) {
            if values.len() != 4 || !['X', 'Y', 'Z'].iter().all(|a| values.get(a) == Some(&0.)) {
                return Err("only explicit zero XYZ G52/G100 reset supported".into());
            }
            if g == Some(52.) {
                self.ready.insert("frame_reset".into());
            } else {
                self.haas.mirror_off = true;
            }
            // Haas 96-8210 Rev N (Mill Operator's Manual, NGC), G-codes: G52
            // sets a coordinate shift (the local system with Setting 33
            // FANUC; zero clears it) and G100 cancels mirror image on the
            // axes it names. Neither commands motion: their X/Y/Z words are
            // the shift and the axes selected, not positions.
            self.hold_axes();
            return Ok(vec![]);
        }
        if matches!(g, Some(50. | 69. | 187.)) {
            if values.len() != 1 || matches!(g, Some(50. | 69.)) && !h.rotation_scaling {
                return Err("unsupported Haas option/smoothing parameters".into());
            }
            match g.unwrap() {
                50. => self.haas.scaling_off = true,
                69. => self.haas.rotation_off = true,
                _ => self.haas.smoothing_reset = true,
            }
            return Ok(vec![]);
        }
        if matches!(m, Some(0. | 1. | 6.)) && (self.rpm != 0. || self.coolant != Coolant::Off) {
            return Err(
                "Haas stop/tool change requires explicit source spindle/coolant off".into(),
            );
        }
        if m == Some(1.) && !h.optional_stop_enabled {
            return Err("optional stop not declared enabled (Setting17/panel)".into());
        }
        if values.get(&'S').is_some_and(|v| v.fract() != 0.) {
            return Err("reference spindle RPM must be integral".into());
        }
        let result = self.read_base(body, context)?;
        for e in &result {
            if let Event::Motion { motion } = e
                && matches!(motion.curve, Curve::Arc { .. } | Curve::Helix { .. })
                && motion.curve.start()[..2] == motion.curve.end()[..2]
                && (values.contains_key(&'X') || values.contains_key(&'Y'))
            {
                return Err("Haas full circle must omit planar endpoint words".into());
            }
        }
        if m == Some(30.) {
            self.haas.end_seen = true;
            self.ended = false;
        }
        Ok(result)
    }
}
