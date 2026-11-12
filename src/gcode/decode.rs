//! Independent reading of the written blocks: a port of the Python
//! `GCodeDecoder` with the `NativeGCodeDecoder` motion fast path. It reads
//! only the actual NC text, the declared dialect and machine table, and the
//! source motion's attribution (id, operation, kind); coordinates, feeds,
//! tools, offsets and spindle states come from the words. It shares no code
//! with the formatter (`super::post`). Unknown words or effects are refused.
use super::controller::Controller;
use super::machine::{Code, Machine, OPTIONAL_EFFECTS, REQUIRED, ToolSetting, code_token};
use crate::motion::{
    CompensationSide, Coolant, Curve, Event, Motion, MoveKind, Plane, SpindleDirection, Vec3,
};
use std::collections::{BTreeMap, BTreeSet, VecDeque};

mod carvera;
mod haas;
mod probe;

/// The version string of the Python decoder this one follows.
pub const VERSION: &str = "configured_motion_native_v1";
pub const CARVERA_VERSION: &str = "carvera-nominal-2";
pub const HAAS_VERSION: &str = "haas-ngc-2020-nominal-1";

pub fn plane_name(p: Plane) -> &'static str {
    match p {
        Plane::Xy => "xy",
        Plane::Xz => "xz",
        Plane::Yz => "yz",
    }
}

/// The source motion a block is read for (attribution only).
#[derive(Clone)]
pub struct Context {
    pub id: u64,
    pub operation: String,
    pub kind: MoveKind,
}

#[derive(Clone, Debug, PartialEq)]
enum Effect {
    Named(String),
    WorkOffset(u32),
}

#[derive(Clone, Debug)]
enum CycleItem {
    /// A probing cycle's move, read back from its blocks.
    Probe(MoveKind, Vec3, Option<f64>),
    Move(Vec3, Option<f64>, bool),
    Dwell(f64),
    Spindle(f64, SpindleDirection),
    Orient,
}

const DRILL_EFFECTS: [&str; 8] = [
    "drill_cycle",
    "drill_dwell_cycle",
    "peck_cycle",
    "chip_break_cycle",
    "bore_cycle",
    "bore_dwell_cycle",
    "bore_stop_cycle",
    "fine_bore_cycle",
];
const STATE_ONLY: [&str; 19] = [
    "units_mm",
    "units_inch",
    "absolute",
    "center_incremental",
    "plane_xy",
    "plane_xz",
    "plane_yz",
    "feed_per_minute",
    "rpm_mode",
    "radius_off",
    "length_off",
    "cycles_off",
    "frame_reset",
    "exact_stop",
    "cycle_return_initial",
    "language_native",
    "compression_off",
    "lathe_diameter_mode",
    "lathe_radius_mode",
];

fn group(effect: &str) -> &str {
    match effect {
        "css_on" | "rpm_mode" => "speed_mode",
        "feed_per_rev" | "feed_per_minute" => "feed_mode",
        "rapid"
        | "linear"
        | "arc_cw"
        | "arc_ccw"
        | "tap_synchronized"
        | "tap_cycle"
        | "tap_cycle_left"
        | "thread_synchronized"
        | "drill_cycle"
        | "drill_dwell_cycle"
        | "peck_cycle"
        | "chip_break_cycle"
        | "bore_cycle"
        | "bore_dwell_cycle"
        | "bore_stop_cycle"
        | "fine_bore_cycle" => "motion",
        "units_mm" | "units_inch" => "units",
        "plane_xy" | "plane_xz" | "plane_yz" => "plane",
        "spindle_cw" | "spindle_ccw" | "spindle_off" | "spindle_orient" => "spindle",
        "flood" | "mist" | "coolant_off" => "coolant",
        "length_on" | "length_off" => "length",
        "radius_off" | "radius_left" | "radius_right" => "radius",
        "stop" | "optional_stop" | "end" => "stop",
        "height_control_on" | "height_control_off" => "height_control",
        "mark_mode" | "cut_mode" => "beam_process",
        other => other,
    }
}

/// The decoded role of a move: a rapid keeps its source role only as an
/// oriented shift (`_kind`).
fn role(rapid: bool, kind: MoveKind) -> MoveKind {
    if !rapid || kind == MoveKind::Shift {
        kind
    } else {
        MoveKind::Rapid
    }
}

fn unused(values: &[(char, f64)], used: &BTreeSet<char>) -> Result<(), String> {
    let mut extra: Vec<String> = values
        .iter()
        .filter(|(w, _)| !used.contains(w))
        .map(|(w, _)| w.to_string())
        .collect();
    if extra.is_empty() {
        return Ok(());
    }
    extra.sort();
    Err(format!("unused/unsupported NC words: {extra:?}"))
}

#[derive(Clone)]
pub struct Decoder {
    haas: haas::HaasState,
    carvera_retracted: bool,
    carvera_air_sync_pending: bool,
    machine: Machine,
    pub position: Vec3,
    pub units: Option<f64>,
    pub plane: Option<Plane>,
    ready: BTreeSet<String>,
    pub active: Option<ToolSetting>,
    selected: Option<u32>,
    pub length: Option<u32>,
    pub tool_pending: bool,
    pub offset: Option<u32>,
    rpm: f64,
    coolant: Coolant,
    pub ended: bool,
    direction: SpindleDirection,
    tap_return: Option<(Vec3, Vec3, f64)>,
    cycle_active: bool,
    css: Option<(f64, f64)>,
    css_max: Option<f64>,
    feed_rev: Option<f64>,
    cycle_queue: VecDeque<CycleItem>,
    cycle_initial: Option<f64>,
    compensation: CompensationSide,
    beams: BTreeSet<String>,
    /// The beam process the decoded program selected (`beam_mark = "mode"`:
    /// marking true, cutting false, none before its first selection).
    beam_marking: Option<bool>,
    /// `beam_mark = "power"`: the source event these blocks are read for is
    /// a marking beam. With no process code the role of a beam word is the
    /// source's attribution, as a fed move's kind is; its power is the
    /// word's.
    pub marking_source: bool,
    /// Probing: the program's probe tools, every tool's cutter radius (a
    /// probe's stylus ball) and the O-word labels already used.
    probes: BTreeSet<String>,
    tool_radii: BTreeMap<String, f64>,
    probe_labels: BTreeSet<String>,
    rules: Vec<(Code, Effect)>,
    required: BTreeSet<String>,
    motions: crate::post_decode::Decoder,
}

impl Decoder {
    pub fn new(
        machine: &Machine,
        initial_position: Vec3,
        beam_tools: BTreeSet<String>,
    ) -> Result<Self, String> {
        let d = &machine.dialect;
        let mut rules = vec![];
        for c in &d.commands {
            rules.push((code_token(&c.code)?, Effect::Named(c.effect.clone())));
        }
        for (i, code) in d.work_offsets.iter().enumerate() {
            rules.push((code_token(code)?, Effect::WorkOffset(i as u32 + 1)));
        }
        let mut required: BTreeSet<String> = REQUIRED.iter().map(|s| s.to_string()).collect();
        required.extend(
            d.commands
                .iter()
                .filter(|c| OPTIONAL_EFFECTS.contains(&c.effect.as_str()))
                .map(|c| c.effect.clone()),
        );
        let motions = crate::post_decode::Decoder::new(crate::post_decode::Config {
            commands: d
                .commands
                .iter()
                .map(|c| crate::post_decode::Rule {
                    effect: c.effect.clone(),
                    code: c.code.clone(),
                })
                .collect(),
            axes: [d.axes[0].clone(), d.axes[1].clone(), d.axes[2].clone()],
            centers: [
                d.centers[0].clone(),
                d.centers[1].clone(),
                d.centers[2].clone(),
            ],
            feed_word: d.feed_word.clone(),
            parentheses_comments: d.parentheses_comments,
            radius_tolerance_mm: d.radius_tolerance_mm,
            radius_relative_tolerance: d.radius_relative_tolerance,
        })?;
        Ok(Self {
            position: initial_position,
            units: None,
            plane: None,
            ready: d
                .fixed_modes
                .iter()
                .cloned()
                .chain(
                    matches!(machine.controller, Some(Controller::Carvera(_)))
                        .then(|| "length".to_string()),
                )
                .collect(),
            active: None,
            selected: None,
            length: None,
            tool_pending: false,
            offset: None,
            rpm: 0.,
            coolant: Coolant::Off,
            ended: false,
            haas: haas::HaasState::default(),
            carvera_retracted: false,
            carvera_air_sync_pending: false,
            direction: SpindleDirection::Cw,
            tap_return: None,
            cycle_active: false,
            css: None,
            css_max: None,
            feed_rev: None,
            cycle_queue: VecDeque::new(),
            cycle_initial: None,
            compensation: CompensationSide::Off,
            beams: beam_tools,
            beam_marking: None,
            marking_source: false,
            probes: BTreeSet::new(),
            tool_radii: BTreeMap::new(),
            probe_labels: BTreeSet::new(),
            rules,
            required,
            motions,
            machine: machine.clone(),
        })
    }

    pub fn validate_initialized(&self) -> Result<(), String> {
        let missing: Vec<&String> = self
            .required
            .iter()
            .filter(|r| !self.ready.contains(*r))
            .collect();
        if missing.is_empty() {
            match &self.machine.controller {
                Some(Controller::Carvera(_)) if self.carvera_air_sync_pending => {
                    Err("Carvera M400 must immediately precede M7 or M9".into())
                }
                None | Some(Controller::Carvera(_)) => Ok(()),
                Some(Controller::HaasNGC(h)) => self.haas_initialized(h),
            }
        } else {
            Err(format!(
                "incomplete explicit decoder modal initialization: {missing:?}"
            ))
        }
    }

    fn rule(&self, code: &Code) -> Option<&Effect> {
        self.rules.iter().find(|(c, _)| c == code).map(|(_, e)| e)
    }

    /// The events a hook's blocks decode to.
    pub fn read(
        &mut self,
        lines: &[String],
        context: Option<&Context>,
    ) -> Result<Vec<Event>, String> {
        if lines.is_empty() && !self.cycle_queue.is_empty() {
            return Ok(vec![self.cycle_effect(context)?]);
        }
        if !self.cycle_queue.is_empty() {
            return Err("canned cycle moves still pending".into());
        }
        let mut result = vec![];
        let pending = self.tap_return;
        for line in lines {
            result.extend(self.read_line(line, context)?);
        }
        if let Some((start, bottom, feed)) = pending {
            // The block after a tapping cycle is its return.
            let Some(context) = context.filter(|c| c.kind == MoveKind::Tap) else {
                return Err(
                    "a tapping cycle must be followed by its return (and G80 for a canned cycle)"
                        .into(),
                );
            };
            if !result.is_empty() || self.cycle_active {
                return Err(
                    "a tapping cycle must be followed by its return (and G80 for a canned cycle)"
                        .into(),
                );
            }
            result.push(self.motion_event(
                context,
                Curve::Line {
                    start: bottom,
                    end: start,
                },
                Some(feed),
                MoveKind::Tap,
            ));
            self.position = start;
            self.tap_return = None;
        }
        Ok(result)
    }

    fn motion_event(
        &self,
        context: &Context,
        curve: Curve,
        feed: Option<f64>,
        kind: MoveKind,
    ) -> Event {
        Event::Motion {
            motion: Motion {
                id: context.id,
                operation: context.operation.clone(),
                tool: self
                    .active
                    .as_ref()
                    .map(|t| t.tool.clone())
                    .unwrap_or_default(),
                kind,
                curve,
                feed_mm_min: feed,
            },
        }
    }

    pub fn read_line(
        &mut self,
        line: &str,
        context: Option<&Context>,
    ) -> Result<Vec<Event>, String> {
        match self.machine.controller.clone() {
            None => self.read_base(line, context),
            Some(Controller::HaasNGC(h)) => self.read_haas(&h, line, context),
            Some(Controller::Carvera(c)) => self.read_carvera(&c, line, context),
        }
    }

    fn read_base(&mut self, line: &str, context: Option<&Context>) -> Result<Vec<Event>, String> {
        if self.tap_return.is_none()
            && !self.cycle_active
            && !self.machine.dialect.x_diameter
            && self.feed_rev.is_none()
        {
            // NativeGCodeDecoder: single motion blocks through the compiled
            // motion grammar; everything else through the full reader.
            if self.ended {
                return Err("NC after program end".into());
            }
            if let Some(decoded) =
                self.motions
                    .decode(line, self.position, self.units, self.plane)?
            {
                self.validate_initialized()?;
                let Some(context) = context else {
                    return Err("NC motion with unknown attribution/tool/length/offset".into());
                };
                if self.active.is_none()
                    || self.length.is_none()
                    || self.offset.is_none()
                    || self.tool_pending
                {
                    return Err("NC motion with unknown attribution/tool/length/offset".into());
                }
                let end = decoded.curve.end();
                if matches!(decoded.curve, Curve::Line { .. }) && end == self.position {
                    return Ok(vec![]);
                }
                let event = self.motion_event(
                    context,
                    decoded.curve,
                    decoded.feed,
                    role(decoded.rapid, context.kind),
                );
                self.position = end;
                return Ok(vec![event]);
            }
        }
        self.read_full(line, context)
    }

    /// `GCodeDecoder.read_line`.
    fn read_full(&mut self, line: &str, context: Option<&Context>) -> Result<Vec<Event>, String> {
        if self.ended {
            return Err("NC after program end".into());
        }
        if line.len() > 1024
            || line
                .bytes()
                .any(|b| !(b == b'\t' || (0x20..=0x7e).contains(&b)))
        {
            return Err("invalid NC physical block".into());
        }
        let mut body = line.split(';').next().unwrap().trim().to_ascii_uppercase();
        let d = self.machine.dialect.clone();
        if body.contains('(') || body.contains(')') {
            if !d.parentheses_comments {
                return Err("parentheses are not comments in this dialect".into());
            }
            // re.sub(r'\([^()]*\)', ' ', body): innermost groups, one pass.
            let mut out = String::new();
            let mut rest = body.as_str();
            while let Some(open) = rest.find('(') {
                let after = &rest[open + 1..];
                match after.find(['(', ')']) {
                    Some(close) if after.as_bytes()[close] == b')' => {
                        out.push_str(&rest[..open]);
                        out.push(' ');
                        rest = &after[close + 1..];
                    }
                    Some(next) => {
                        out.push_str(&rest[..open + 1 + next]);
                        rest = &after[next..];
                    }
                    None => {
                        out.push_str(rest);
                        rest = "";
                    }
                }
            }
            out.push_str(rest);
            body = out;
            if body.contains('(') || body.contains(')') {
                return Err("nested/unclosed NC comment".into());
            }
        }
        let trimmed = body.trim();
        if trimmed.is_empty() {
            return Ok(vec![]);
        }
        let mut effects: Vec<Effect> = vec![];
        let mut values: Vec<(char, f64)> = vec![];
        if let Some(effect) = self.rule(&Code::Keyword(trimmed.to_string())) {
            effects.push(effect.clone());
        } else {
            for (word, number) in words(&body)? {
                let value: f64 = number.parse().map_err(|_| "invalid NC value")?;
                if !value.is_finite() || value.abs() > 1e9 {
                    return Err("invalid NC value".into());
                }
                let code = Code::Word(word as u8, value);
                if word == 'G' || word == 'M' || self.rule(&code).is_some() {
                    match self.rule(&code) {
                        Some(e) => effects.push(e.clone()),
                        None => {
                            return Err(format!("undeclared controller command {word}{number}"));
                        }
                    }
                } else if word == 'N' {
                    if value < 0. || value != value.trunc() {
                        return Err("invalid line number".into());
                    }
                } else {
                    if values.iter().any(|(w, _)| *w == word) {
                        return Err("duplicate NC word".into());
                    }
                    values.push((word, value));
                }
            }
        }
        let named: Vec<&str> = effects
            .iter()
            .map(|e| match e {
                Effect::Named(n) => n.as_str(),
                Effect::WorkOffset(_) => "",
            })
            .collect();
        let only = |name: &str| effects.len() == 1 && named[0] == name;
        let get = |w: &str| {
            values
                .iter()
                .find(|(k, _)| k.to_string() == w)
                .map(|(_, v)| *v)
        };
        if self.tap_return.is_some() && !only("cycles_off") {
            return Err(
                "a tapping cycle must be followed by its return (and G80 for a canned cycle)"
                    .into(),
            );
        }
        if effects.len() == 1 && ["rapid", "linear", "arc_cw", "arc_ccw"].contains(&named[0]) {
            if self.cycle_active {
                return Err(
                    "canned cycle still active: cancel it (G80) before other motion".into(),
                );
            }
            let effect = named[0].to_string();
            return Ok(self
                .motion(&effect, &values, context)?
                .into_iter()
                .collect());
        }
        let radius = ["radius_left", "radius_right", "radius_off"];
        if d.compensation_with_move
            && effects.len() == 2
            && ["rapid", "linear"].contains(&named[0])
            && radius.contains(&named[1])
        {
            if self.cycle_active {
                return Err(
                    "canned cycle still active: cancel it (G80) before other motion".into(),
                );
            }
            let change = self.compensation(named[1], &values)?;
            let filtered: Vec<(char, f64)> = values
                .iter()
                .filter(|(k, _)| k.to_string() != d.compensation_word || named[1] == "radius_off")
                .copied()
                .collect();
            let effect = named[0].to_string();
            let motion = self.motion(&effect, &filtered, context)?;
            let Some(motion) = motion.filter(|m| match m {
                Event::Motion { motion } => motion.curve.start() != motion.curve.end(),
                _ => false,
            }) else {
                return Err("cutter compensation needs a move in its block".into());
            };
            let mut out = change;
            out.push(motion);
            return Ok(out);
        }
        // Canned cycles drill along Z only in G17 (#183); G33.1 always taps
        // along Z.
        if effects.len() == 1
            && (["tap_cycle", "tap_cycle_left"].contains(&named[0])
                || DRILL_EFFECTS.contains(&named[0]))
            && self.machine.planes.contains(&crate::motion::Plane::Xy)
            && self.plane != Some(crate::motion::Plane::Xy)
        {
            return Err(
                "canned cycles drill along Z only in G17: select G17 before the cycle".into(),
            );
        }
        if effects.len() == 1
            && ["tap_synchronized", "tap_cycle", "tap_cycle_left"].contains(&named[0])
        {
            let effect = named[0].to_string();
            return Ok(vec![self.tap(&effect, &values, context)?]);
        }
        if only("thread_synchronized") {
            return Ok(vec![self.thread(&values, context)?]);
        }
        if effects.len() == 1 && DRILL_EFFECTS.contains(&named[0]) {
            let effect = named[0].to_string();
            self.drill_cycle(&effect, &values, context)?;
            return Ok(vec![]);
        }
        if self.cycle_active && !only("cycles_off") {
            return Err("canned cycle still active: cancel it (G80) first".into());
        }
        if named.contains(&"css_on") {
            let names: Vec<String> = named.iter().map(|s| s.to_string()).collect();
            return Ok(vec![self.surface_speed(&names, &values)?]);
        }
        if only("css_limit") {
            let s = get(&d.speed_word);
            if values.len() != 1 || !s.is_some_and(|s| 0. < s && s <= self.machine.max_spindle_rpm)
            {
                return Err(
                    "the CSS limit block sets S (rpm) alone, within the machine maximum".into(),
                );
            }
            self.css_max = s;
            return Ok(vec![]);
        }
        if only("feed_per_rev") {
            let f = get(&d.feed_word);
            if values.len() != 1 || !f.is_some_and(|f| f > 0.) {
                return Err("feed per revolution needs its F word alone".into());
            }
            let units = self.units.ok_or("feed per revolution with unknown units")?;
            let mm_rev = f.unwrap() * units;
            self.feed_rev = Some(mm_rev);
            return Ok(vec![Event::FeedPerRev { mm_rev }]);
        }
        let groups: Vec<&str> = effects
            .iter()
            .map(|e| match e {
                Effect::Named(n) => group(n),
                Effect::WorkOffset(_) => "work",
            })
            .collect();
        if groups.iter().collect::<BTreeSet<_>>().len() != groups.len() {
            return Err("conflicting modal commands in one block".into());
        }
        // Independent state setters may share a block; transitions stay separate.
        let transitions = effects
            .iter()
            .filter(|e| match e {
                Effect::Named(n) => !STATE_ONLY.contains(&n.as_str()),
                Effect::WorkOffset(_) => true,
            })
            .count();
        if transitions > 1 || transitions > 0 && effects.len() > 1 {
            return Err("separate controller state transitions into individual blocks".into());
        }
        let mut output = vec![];
        let mut used: BTreeSet<char> = BTreeSet::new();
        let letter = |w: &str| w.chars().next().unwrap();
        for effect in &effects {
            let name = match effect {
                Effect::WorkOffset(number) => {
                    if *number != self.machine.work_offset {
                        return Err("NC work offset differs from machine setup".into());
                    }
                    self.offset = Some(*number);
                    output.push(Event::WorkOffset {
                        number: *number as u8,
                    });
                    continue;
                }
                Effect::Named(n) => n.as_str(),
            };
            match name {
                "units_mm" | "units_inch" => {
                    self.units = Some(if name == "units_mm" { 1. } else { 25.4 });
                    self.ready.insert("units".into());
                }
                "plane_xy" | "plane_xz" | "plane_yz" => {
                    let plane = match name {
                        "plane_xy" => Plane::Xy,
                        "plane_xz" => Plane::Xz,
                        _ => Plane::Yz,
                    };
                    self.plane = Some(plane);
                    if !self.machine.planes.is_empty() && !self.machine.planes.contains(&plane) {
                        return Err("plane not supported by machine definition".into());
                    }
                    self.ready.insert("plane".into());
                }
                "length_off" => {
                    self.length = None;
                    self.ready.insert("length".into());
                }
                "cycles_off" => {
                    self.cycle_active = false;
                    self.ready.insert(name.into());
                }
                _ if name == "radius_left"
                    || name == "radius_right"
                    || name == "radius_off" && self.compensation != CompensationSide::Off =>
                {
                    if d.compensation_with_move {
                        return Err(
                            "this controller turns cutter compensation on/off in the block of an XY move".into(),
                        );
                    }
                    used.insert(letter(&d.compensation_word));
                    output.extend(self.compensation(name, &values)?);
                }
                "rpm_mode" => {
                    self.css = None;
                    self.ready.insert(name.into());
                }
                "feed_per_minute" => {
                    self.ready.insert(name.into());
                    if self.feed_rev.is_some() {
                        self.feed_rev = None;
                        output.push(Event::FeedPerRev { mm_rev: 0. });
                    }
                }
                _ if STATE_ONLY.contains(&name) => {
                    self.ready.insert(name.into());
                }
                "tool_change" => {
                    let Some(selected) = self.selected.filter(|_| d.tool_change == "m6") else {
                        return Err("unexpected M6/tool selection".into());
                    };
                    self.activate(selected)?;
                }
                "length_on" => {
                    if d.length_activation != "explicit" {
                        return Err("unexpected global length command".into());
                    }
                    used.insert(letter(&d.length_word));
                    output.push(self.length(get(&d.length_word))?);
                }
                "spindle_orient" => {
                    let Some(edge) = d
                        .oriented_edge_deg
                        .filter(|_| self.active.is_some() && !self.tool_pending)
                    else {
                        return Err(
                            "NC oriented spindle stop without tool or declared orientation".into(),
                        );
                    };
                    self.rpm = 0.;
                    output.push(Event::SpindleOrient {
                        edge_angle_deg: edge,
                    });
                }
                "spindle_off" | "spindle_cw" | "spindle_ccw"
                    if self
                        .active
                        .as_ref()
                        .is_some_and(|t| self.beams.contains(&t.tool)) =>
                {
                    let Some(s_max) = d.beam_s_max else {
                        return Err("NC beam without a declared beam_s_max".into());
                    };
                    if name == "spindle_off" {
                        self.rpm = 0.;
                        output.push(Event::Beam { power_percent: 0. });
                    } else {
                        if name != d.beam_effect || self.tool_pending {
                            return Err("NC beam word differs from the declared beam_effect".into());
                        }
                        used.insert(letter(&d.speed_word));
                        let Some(s) = get(&d.speed_word).filter(|s| 0. < *s && *s <= s_max) else {
                            return Err("NC beam power S outside 0..beam_s_max".into());
                        };
                        self.rpm = s;
                        let power_percent = s / s_max * 100.;
                        let marking = match d.beam_mark.as_deref() {
                            Some("mode") => self.beam_marking.ok_or(
                                "NC beam on before its process (mark_mode or cut_mode) is selected",
                            )?,
                            Some(_) => self.marking_source,
                            None => false,
                        };
                        output.push(if marking {
                            Event::BeamMark { power_percent }
                        } else {
                            Event::Beam { power_percent }
                        });
                    }
                }
                "mark_mode" | "cut_mode" => {
                    if d.beam_mark.as_deref() != Some("mode")
                        || self.tool_pending
                        || self.rpm != 0.
                        || !self
                            .active
                            .as_ref()
                            .is_some_and(|t| self.beams.contains(&t.tool))
                    {
                        return Err(
                            "NC beam process selection needs beam_mark='mode', the beam tool and the beam off"
                                .into(),
                        );
                    }
                    self.beam_marking = Some(name == "mark_mode");
                }
                "height_control_on" | "height_control_off" => {
                    if let Some(word) = &d.height_control_word {
                        let w = letter(word);
                        used.insert(w);
                        let expected: f64 = word[1..]
                            .parse()
                            .map_err(|_| "invalid height control word")?;
                        if get(&w.to_string()) != Some(expected) {
                            return Err("NC height control word differs".into());
                        }
                    }
                    output.push(Event::HeightControl {
                        on: name == "height_control_on",
                    });
                }
                "spindle_off" | "spindle_cw" | "spindle_ccw" => {
                    if name == "spindle_off" {
                        self.rpm = 0.;
                        self.ready.insert("spindle_off".into());
                    } else {
                        if self.active.is_none() || self.tool_pending {
                            return Err("NC spindle without assigned tool/length".into());
                        }
                        if self.css.is_some() {
                            return Err(
                                "S is a surface speed in G96 mode: G97 before an rpm".into()
                            );
                        }
                        used.insert(letter(&d.speed_word));
                        let Some(rpm) = get(&d.speed_word)
                            .filter(|r| 0. < *r && *r <= self.machine.max_spindle_rpm)
                        else {
                            return Err("invalid NC spindle".into());
                        };
                        self.rpm = rpm;
                        self.direction = if name == "spindle_ccw" {
                            SpindleDirection::Ccw
                        } else {
                            SpindleDirection::Cw
                        };
                    }
                    output.push(Event::Spindle {
                        rpm: self.rpm,
                        direction: if name == "spindle_ccw" {
                            SpindleDirection::Ccw
                        } else {
                            SpindleDirection::Cw
                        },
                    });
                }
                "coolant_off" | "flood" | "mist" => {
                    if name == "coolant_off" {
                        self.coolant = Coolant::Off;
                        self.ready.insert("coolant_off".into());
                    } else {
                        if self.coolant != Coolant::Off {
                            return Err("exclusive coolant change requires off first".into());
                        }
                        self.coolant = if name == "flood" {
                            Coolant::Flood
                        } else {
                            Coolant::Mist
                        };
                    }
                    output.push(Event::Coolant { mode: self.coolant });
                }
                "stop" | "optional_stop" => output.push(Event::Stop {
                    optional: name == "optional_stop",
                }),
                "dwell" => {
                    used.insert(letter(&d.dwell_word));
                    let Some(seconds) =
                        get(&d.dwell_word).filter(|s| 0. < *s && *s <= d.dwell_max_s)
                    else {
                        return Err("NC dwell needs a positive time".into());
                    };
                    output.push(Event::Dwell { seconds });
                }
                "end" => {
                    if self.rpm != 0.
                        || self.coolant != Coolant::Off
                        || self.length.is_some()
                        || self.tool_pending
                        || self.compensation != CompensationSide::Off
                    {
                        return Err(
                            "program end requires stopped spindle/coolant and cancelled length"
                                .into(),
                        );
                    }
                    self.ended = true;
                }
                _ => return Err("unsupported declared effect".into()),
            }
        }
        let tool_letter = letter(&d.tool_word);
        if let Some(number) = get(&d.tool_word) {
            if !effects.is_empty() {
                return Err("select tool in its own block".into());
            }
            used.insert(tool_letter);
            if number != number.trunc() {
                return Err("tool must be integer".into());
            }
            let number = number as i64;
            self.selected = Some(number as u32);
            if d.tool_change == "lathe_t" {
                // T<nn><oo>: tool nn with its offset oo, in one word.
                self.selected = Some((number / 100) as u32);
                self.activate((number / 100) as u32)?;
                output.push(self.length(Some((number % 100) as f64))?);
            } else if d.tool_change == "direct_t" {
                self.activate(number as u32)?;
            }
        }
        let length_letter = letter(&d.length_word);
        if let Some(number) = get(&d.length_word)
            && !used.contains(&length_letter)
        {
            if d.length_activation != "tool_edge" || !effects.is_empty() && named != ["tool_change"]
            {
                return Err("unexpected length word".into());
            }
            used.insert(length_letter);
            if number == 0. {
                self.length = None;
                self.ready.insert("length".into());
            } else {
                output.push(self.length(Some(number))?);
            }
        }
        unused(&values, &used)?;
        Ok(output)
    }

    fn require_motion_state(&self, context: Option<&Context>) -> Result<(), String> {
        if context.is_none()
            || self.active.is_none()
            || self.length.is_none()
            || self.offset.is_none()
            || self.tool_pending
        {
            return Err("NC motion with unknown attribution/tool/length/offset".into());
        }
        Ok(())
    }

    /// `GCodeDecoder._motion`.
    fn motion(
        &mut self,
        effect: &str,
        values: &[(char, f64)],
        context: Option<&Context>,
    ) -> Result<Option<Event>, String> {
        let d = self.machine.dialect.clone();
        let mut used = BTreeSet::new();
        self.validate_initialized()?;
        self.require_motion_state(context)?;
        let context = context.unwrap();
        let units = self.units.ok_or("NC motion with unknown units")?;
        let get = |w: &str| {
            values
                .iter()
                .find(|(k, _)| k.to_string() == w)
                .map(|(_, v)| *v)
        };
        let start = self.position;
        let mut end = start;
        for (i, axis) in d.axes.iter().enumerate() {
            if let Some(v) = get(axis) {
                end[i] = v * units * if i == 0 && d.x_diameter { 0.5 } else { 1. };
                used.insert(axis.chars().next().unwrap());
            }
        }
        let mut feed = None;
        if effect != "rapid" {
            used.insert(d.feed_word.chars().next().unwrap());
            let f = get(&d.feed_word)
                .filter(|f| *f > 0.)
                .ok_or("NC motion needs positive explicit feed")?;
            feed = Some(f * units);
        }
        if let (Some(f), Some(rev)) = (feed, self.feed_rev)
            && (f - rev).abs() > 1e-12 * f.max(1.)
        {
            return Err("NC feed differs from the feed per revolution in effect (G95 F)".into());
        }
        let mut curve = Curve::Line { start, end };
        if effect == "arc_cw" || effect == "arc_ccw" {
            let plane = self.plane.ok_or("NC motion with unknown plane")?;
            let [u, v, a] = plane.axes();
            let mut center = start;
            let mut offset = [0.; 3];
            for i in [u, v] {
                let word = &d.centers[i];
                let value = get(word).ok_or("NC arc needs both center offsets")?;
                offset[i] = value * units;
                center[i] += offset[i];
                used.insert(word.chars().next().unwrap());
            }
            let (x, y) = (start[u] - center[u], start[v] - center[v]);
            let (ex, ey) = (end[u] - center[u], end[v] - center[v]);
            let (r1, r2, error) = crate::post_decode::arc_radii(
                [start[u], start[v]],
                [end[u], end[v]],
                [offset[u], offset[v]],
                units,
            );
            if r1 < 1e-6 || !d.radius_accepts(r1, r2, error) {
                return Err(
                    "NC arc radius mismatch/collapse exceeds declared decoder model".into(),
                );
            }
            let angle = (x * ey - y * ex).atan2(x * ex + y * ey);
            let tau = std::f64::consts::TAU;
            let mut sweep = if effect == "arc_ccw" {
                angle.rem_euclid(tau)
            } else {
                -(-angle).rem_euclid(tau)
            };
            if sweep == 0. {
                sweep = if effect == "arc_ccw" { tau } else { -tau };
            }
            curve = if end[a] == start[a] {
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
            };
        }
        if feed.is_some()
            && let Some(rev) = self.feed_rev
        {
            // Per minute: at the highest spindle speed along the move.
            let top = match self.css {
                Some((m_min, max_rpm)) => crate::motion::css_rpm_bound(&curve, m_min, max_rpm),
                None => self.rpm,
            };
            if top.is_nan() || top <= 0. {
                return Err("feed per revolution with the spindle stopped".into());
            }
            feed = Some(rev * top);
        }
        let event = self.motion_event(context, curve, feed, role(effect == "rapid", context.kind));
        unused(values, &used)?;
        self.position = end;
        // A coordinate block may round to the current position (no motion).
        Ok(
            if (effect == "rapid" || effect == "linear") && end == start {
                None
            } else {
                Some(event)
            },
        )
    }

    fn compensation_plane(&self) -> Plane {
        if self.plane == Some(Plane::Xz) {
            Plane::Xz
        } else {
            Plane::Xy
        }
    }

    fn compensation(&mut self, effect: &str, values: &[(char, f64)]) -> Result<Vec<Event>, String> {
        let d = self.machine.dialect.clone();
        if effect == "radius_off" {
            self.ready.insert(effect.into());
            if self.compensation == CompensationSide::Off {
                return Ok(vec![]);
            }
            self.compensation = CompensationSide::Off;
            return Ok(vec![Event::CutterCompensation {
                side: CompensationSide::Off,
                plane: self.compensation_plane(),
            }]);
        }
        if self.active.is_none() || self.tool_pending || self.compensation != CompensationSide::Off
        {
            return Err(
                "cutter compensation needs an active tool and must be off before it is turned on"
                    .into(),
            );
        }
        if d.compensation_register {
            if !d.compensation_word_free() {
                return Err("compensation word is not free in this dialect".into());
            }
            let register = self.active.as_ref().and_then(|t| t.diameter_register);
            let written = values
                .iter()
                .find(|(k, _)| k.to_string() == d.compensation_word)
                .map(|(_, v)| *v);
            if register.is_none() || written != register.map(f64::from) {
                return Err("cutter compensation D register differs from the active tool's declared register".into());
            }
        }
        self.compensation = if effect == "radius_left" {
            CompensationSide::Left
        } else {
            CompensationSide::Right
        };
        Ok(vec![Event::CutterCompensation {
            side: self.compensation,
            plane: self.compensation_plane(),
        }])
    }

    fn surface_speed(
        &mut self,
        effects: &[String],
        values: &[(char, f64)],
    ) -> Result<Event, String> {
        let d = self.machine.dialect.clone();
        let rest: Vec<&String> = effects.iter().filter(|e| *e != "css_on").collect();
        if rest.len() != 1 || !["spindle_cw", "spindle_ccw"].contains(&rest[0].as_str()) {
            return Err(
                "constant surface speed is written with its spindle direction in one block".into(),
            );
        }
        if self
            .active
            .as_ref()
            .is_none_or(|t| self.beams.contains(&t.tool))
            || self.tool_pending
        {
            return Err("NC constant surface speed without an assigned (non-beam) tool".into());
        }
        let get = |w: &str| {
            values
                .iter()
                .find(|(k, _)| k.to_string() == w)
                .map(|(_, v)| *v)
        };
        let mut words: BTreeSet<String> = [d.speed_word.clone()].into();
        let limit = if d.declares("css_limit") {
            self.css_max
        } else {
            let w = d.css_limit_word.clone().unwrap_or_default();
            words.insert(w.clone());
            get(&w)
        };
        let s = get(&d.speed_word);
        let written: BTreeSet<String> = values.iter().map(|(k, _)| k.to_string()).collect();
        let (Some(s), Some(limit)) = (s, limit) else {
            return Err(
                "constant surface speed needs S and its rpm limit within the machine maximum"
                    .into(),
            );
        };
        if written != words || s <= 0. || !(0. < limit && limit <= self.machine.max_spindle_rpm) {
            return Err(
                "constant surface speed needs S and its rpm limit within the machine maximum"
                    .into(),
            );
        }
        let m_min = s * if self.units == Some(25.4) { 0.3048 } else { 1. };
        self.css = Some((m_min, limit));
        self.rpm = limit;
        self.direction = if rest[0] == "spindle_ccw" {
            SpindleDirection::Ccw
        } else {
            SpindleDirection::Cw
        };
        Ok(Event::SurfaceSpeed {
            m_min,
            max_rpm: limit,
            direction: self.direction,
        })
    }

    /// Expand a drilling/boring cycle block (G98 return) into the moves and
    /// spindle/dwell effects the controller makes, from its actual words
    /// (LinuxCNC interp_cycles.cc; FANUC 0i-MB 13.1; Centroid 12.28).
    fn drill_cycle(
        &mut self,
        effect: &str,
        values: &[(char, f64)],
        context: Option<&Context>,
    ) -> Result<(), String> {
        let d = self.machine.dialect.clone();
        let mut used: BTreeSet<char> = d.axes.iter().map(|a| a.chars().next().unwrap()).collect();
        used.insert('R');
        used.insert(d.feed_word.chars().next().unwrap());
        self.validate_initialized()?;
        if context.is_some()
            || self.active.is_none()
            || self.length.is_none()
            || self.offset.is_none()
            || self.tool_pending
        {
            return Err("NC drilling cycle with unknown tool/length/offset".into());
        }
        if self.feed_rev.is_some() {
            return Err("drilling cycle in feed per revolution mode".into());
        }
        if !self.ready.contains("cycle_return_initial") {
            return Err("drilling cycle without G98 initial-level return".into());
        }
        let get = |w: &str| {
            values
                .iter()
                .find(|(k, _)| k.to_string() == w)
                .map(|(_, v)| *v)
        };
        let units = self.units.ok_or("drilling cycle with unknown units")?;
        let (Some(x), Some(y), Some(z), Some(r), Some(feed)) = (
            get(&d.axes[0]),
            get(&d.axes[1]),
            get(&d.axes[2]),
            get("R"),
            get(&d.feed_word),
        ) else {
            return Err("drilling cycle needs X Y Z R F".into());
        };
        let (x, y, z, r, feed) = (x * units, y * units, z * units, r * units, feed * units);
        let start = self.position;
        let z0 = start[2];
        if !(z < r && r <= z0) || feed <= 0. {
            return Err(
                "drilling cycle needs Z < R <= the initial level and a positive feed".into(),
            );
        }
        if self.cycle_active && Some(z0) != self.cycle_initial {
            return Err(
                "a chained canned cycle must start at the same initial level (cancel with G80 first)".into(),
            );
        }
        let boring = [
            "bore_cycle",
            "bore_dwell_cycle",
            "bore_stop_cycle",
            "fine_bore_cycle",
        ]
        .contains(&effect);
        if boring && z0 != r {
            return Err("boring cycles are written from the R plane (initial level = R)".into());
        }
        let dwell_value = |used: &mut BTreeSet<char>| -> Result<f64, String> {
            used.insert('P');
            let mut dwell = get("P").unwrap();
            if d.cycle_dwell_unit == "ms" {
                if dwell != dwell.trunc() {
                    return Err("dwell P is whole milliseconds".into());
                }
                dwell /= 1000.;
            }
            if !(0. < dwell && dwell <= d.dwell_max_s) {
                return Err("invalid drilling cycle dwell".into());
            }
            Ok(dwell)
        };
        let mut dwell = None;
        if ["drill_dwell_cycle", "bore_dwell_cycle"].contains(&effect) {
            if get("P").is_none() {
                return Err("dwell cycle needs its dwell P".into());
            }
            dwell = Some(dwell_value(&mut used)?);
        }
        let mut q = None;
        if ["peck_cycle", "chip_break_cycle", "fine_bore_cycle"].contains(&effect) {
            let Some(value) = get("Q").filter(|q| *q > 0.) else {
                return Err("pecking/fine boring cycle needs a positive Q".into());
            };
            used.insert('Q');
            q = Some(value * units);
        }
        if effect == "fine_bore_cycle" && get("P").is_some() {
            dwell = Some(dwell_value(&mut used)?);
        }
        unused(values, &used)?;
        if effect == "bore_stop_cycle" && self.rpm <= 0. {
            return Err("G86 needs the spindle running".into());
        }
        if effect == "fine_bore_cycle"
            && (self.rpm <= 0.
                || self.direction != SpindleDirection::Cw
                || d.oriented_edge_deg.is_none())
        {
            return Err(
                "G76 needs the spindle running clockwise and a declared oriented_edge_deg".into(),
            );
        }
        let mut items = vec![
            CycleItem::Move([x, y, z0], None, true),
            CycleItem::Move([x, y, r], None, true),
        ];
        if let Some(q) = q
            && effect != "fine_bore_cycle"
        {
            let mut current = r - q;
            while current > z {
                items.push(CycleItem::Move([x, y, current], Some(feed), false));
                if effect == "peck_cycle" {
                    items.push(CycleItem::Move([x, y, r], None, true));
                    items.push(CycleItem::Move(
                        [x, y, current + d.peck_reentry_mm.unwrap()],
                        None,
                        true,
                    ));
                } else {
                    items.push(CycleItem::Move(
                        [x, y, current + d.chip_break_lift_mm.unwrap()],
                        None,
                        true,
                    ));
                }
                current -= q;
            }
        }
        items.push(CycleItem::Move([x, y, z], Some(feed), false));
        if let Some(dwell) = dwell {
            items.push(CycleItem::Dwell(dwell));
        }
        if effect == "bore_stop_cycle" {
            items.extend([
                CycleItem::Spindle(0., self.direction),
                CycleItem::Move([x, y, z0], None, true),
                CycleItem::Spindle(self.rpm, self.direction),
            ]);
        } else if effect == "fine_bore_cycle" {
            // FANUC 0i-MB 13.1.3: oriented stop, shift away from the edge,
            // rapid out, shift back, spindle clockwise.
            let a = d.oriented_edge_deg.unwrap().to_radians();
            let q = q.unwrap();
            let s = [x - q * a.cos(), y - q * a.sin()];
            items.extend([
                CycleItem::Orient,
                CycleItem::Move([s[0], s[1], z], None, true),
                CycleItem::Move([s[0], s[1], z0], None, true),
                CycleItem::Move([x, y, z0], None, true),
                CycleItem::Spindle(self.rpm, SpindleDirection::Cw),
            ]);
        } else if boring {
            items.push(CycleItem::Move([x, y, r], Some(feed), false));
        }
        items.push(CycleItem::Move([x, y, z0], None, true));
        let mut queue = VecDeque::new();
        let mut position = start;
        for item in items {
            match item {
                CycleItem::Move(end, ..) if end == position => {}
                CycleItem::Move(end, ..) => {
                    position = end;
                    queue.push_back(item);
                }
                _ => queue.push_back(item),
            }
        }
        self.cycle_queue = queue;
        self.cycle_active = true;
        self.cycle_initial = Some(z0);
        Ok(())
    }

    fn cycle_effect(&mut self, context: Option<&Context>) -> Result<Event, String> {
        let item = self.cycle_queue.pop_front().unwrap();
        let control = |context: Option<&Context>| -> Result<(), String> {
            if context.is_some() {
                return Err("canned cycle control effect expected, not a move".into());
            }
            Ok(())
        };
        match item {
            CycleItem::Probe(kind, end, feed) => {
                let Some(context) = context else {
                    return Err("probing cycle move expected".into());
                };
                let event = self.motion_event(
                    context,
                    Curve::Line {
                        start: self.position,
                        end,
                    },
                    feed,
                    kind,
                );
                self.position = end;
                Ok(event)
            }
            CycleItem::Dwell(seconds) => {
                control(context)?;
                Ok(Event::Dwell { seconds })
            }
            CycleItem::Orient => {
                control(context)?;
                self.rpm = 0.;
                Ok(Event::SpindleOrient {
                    edge_angle_deg: self.machine.dialect.oriented_edge_deg.unwrap(),
                })
            }
            CycleItem::Spindle(rpm, direction) => {
                control(context)?;
                self.rpm = rpm;
                Ok(Event::Spindle { rpm, direction })
            }
            CycleItem::Move(end, feed, rapid) => {
                let Some(context) = context else {
                    return Err("canned cycle move expected, not a control event".into());
                };
                let event = self.motion_event(
                    context,
                    Curve::Line {
                        start: self.position,
                        end,
                    },
                    feed,
                    role(rapid, context.kind),
                );
                self.position = end;
                Ok(event)
            }
        }
    }

    /// Decode a tapping cycle at the current XY from its actual words.
    fn tap(
        &mut self,
        effect: &str,
        values: &[(char, f64)],
        context: Option<&Context>,
    ) -> Result<Event, String> {
        let d = self.machine.dialect.clone();
        let mut used = BTreeSet::new();
        self.validate_initialized()?;
        if context.is_none_or(|c| c.kind != MoveKind::Tap)
            || self.active.is_none()
            || self.length.is_none()
            || self.offset.is_none()
            || self.tool_pending
        {
            return Err("NC tapping cycle with unknown attribution/tool/length/offset".into());
        }
        let context = context.unwrap();
        if self.cycle_active {
            return Err("canned cycle already active".into());
        }
        if self.css.is_some() || self.feed_rev.is_some() {
            return Err("tapping needs G97 and G94".into());
        }
        let ok = match effect {
            "tap_cycle" => self.direction == SpindleDirection::Cw,
            "tap_cycle_left" => self.direction == SpindleDirection::Ccw,
            _ => true,
        };
        if self.rpm <= 0. || !ok {
            return Err(format!(
                "tapping needs the spindle running {}",
                match effect {
                    "tap_cycle" => "clockwise (M3) for G84",
                    "tap_cycle_left" => "counterclockwise (M4) for G74",
                    _ => "",
                }
            ));
        }
        let units = self.units.ok_or("tapping cycle with unknown units")?;
        let get = |w: &str| {
            values
                .iter()
                .find(|(k, _)| k.to_string() == w)
                .map(|(_, v)| *v)
        };
        let start = self.position;
        for i in [0, 1] {
            if let Some(v) = get(&d.axes[i]) {
                used.insert(d.axes[i].chars().next().unwrap());
                if v * units != start[i] {
                    return Err("tapping cycle must start at its hole position".into());
                }
            }
        }
        used.insert(d.axes[2].chars().next().unwrap());
        let z = get(&d.axes[2]).ok_or("tapping cycle needs Z")?;
        let bottom = [start[0], start[1], z * units];
        let feed = if effect == "tap_synchronized" {
            used.insert('K');
            let pitch = get("K")
                .filter(|p| *p > 0.)
                .ok_or("G33.1 needs a positive K pitch")?;
            pitch * units * self.rpm
        } else {
            used.insert('R');
            if get("R").is_none_or(|r| r * units != start[2]) {
                return Err("tapping cycle R plane must be the current Z".into());
            }
            let feed = if let Some(w) = &d.tap_pitch_word {
                used.insert(w.chars().next().unwrap());
                let lead = get(w)
                    .filter(|l| *l > 0.)
                    .ok_or("rigid tapping cycle needs a positive lead")?;
                lead * units * self.rpm
            } else {
                used.insert(d.feed_word.chars().next().unwrap());
                let f = get(&d.feed_word)
                    .filter(|f| *f > 0.)
                    .ok_or("tapping cycle needs a positive feed")?;
                f * units
            };
            self.cycle_active = true;
            feed
        };
        if bottom[2].is_nan() || bottom[2] >= start[2] {
            return Err("tapping cycle must go down".into());
        }
        unused(values, &used)?;
        self.position = bottom;
        self.tap_return = Some((start, bottom, feed));
        Ok(self.motion_event(
            context,
            Curve::Line { start, end: bottom },
            Some(feed),
            MoveKind::Tap,
        ))
    }

    /// A spindle-synchronized line: feed = lead x rpm.
    fn thread(
        &mut self,
        values: &[(char, f64)],
        context: Option<&Context>,
    ) -> Result<Event, String> {
        let d = self.machine.dialect.clone();
        self.validate_initialized()?;
        self.require_motion_state(context).map_err(|_| {
            "NC threading move with unknown attribution/tool/length/offset".to_string()
        })?;
        let context = context.unwrap();
        if self.cycle_active || self.rpm <= 0. || self.css.is_some() {
            return Err(
                "threading needs the spindle running at a fixed rpm (G97) and no active cycle"
                    .into(),
            );
        }
        let units = self.units.ok_or("threading move with unknown units")?;
        let get = |w: &str| {
            values
                .iter()
                .find(|(k, _)| k.to_string() == w)
                .map(|(_, v)| *v)
        };
        let lead = get(&d.thread_lead_word)
            .filter(|l| *l > 0.)
            .ok_or("threading move needs a positive lead")?;
        let start = self.position;
        let mut end = start;
        for (i, axis) in d.axes.iter().enumerate() {
            if let Some(v) = get(axis) {
                end[i] = v * units * if i == 0 && d.x_diameter { 0.5 } else { 1. };
            }
        }
        if values.iter().any(|(k, _)| {
            !d.axes.iter().any(|a| *a == k.to_string()) && k.to_string() != d.thread_lead_word
        }) {
            return Err("unused/unsupported NC words in threading move".into());
        }
        self.position = end;
        Ok(self.motion_event(
            context,
            Curve::Line { start, end },
            Some(lead * units * self.rpm),
            MoveKind::Thread,
        ))
    }

    fn activate(&mut self, number: u32) -> Result<(), String> {
        if self.rpm != 0. {
            return Err("stop spindle before selecting tool".into());
        }
        let choices: Vec<&ToolSetting> = self
            .machine
            .tools
            .iter()
            .filter(|t| t.number == number)
            .collect();
        let active = choices
            .iter()
            .find(|t| t.length_register == 1)
            .or(choices.first())
            .map(|t| (*t).clone())
            .ok_or("NC tool absent from machine table")?;
        match self.machine.tool_change_position_mode.as_str() {
            "table" => {
                self.position = active
                    .position_after_mm
                    .ok_or("the tool table entry has no position_after_mm")?
            }
            "z_home" => {
                // M6 sends the spindle to machine Z home; XY stays.
                let gauge_z = self.machine.tool_change_home_z_mm.unwrap()
                    - self.machine.work_to_machine.origin_mm[2];
                self.position = [
                    self.position[0],
                    self.position[1],
                    gauge_z - active.gauge_length_mm,
                ];
            }
            "preserve_spindle" => {
                // The tool is changed where the spindle stands (LinuxCNC M6
                // without a configured tool-change move): the gauge line
                // stays, the tip moves by the gauge-length difference. The
                // first tool starts at the program's declared position.
                if let Some(old) = &self.active {
                    self.position = crate::stream::in_place_tip(
                        self.position,
                        old.gauge_length_mm,
                        active.gauge_length_mm,
                    );
                }
            }
            _ => {}
        }
        self.active = Some(active);
        self.length = None;
        self.tool_pending = true;
        Ok(())
    }

    fn length(&mut self, number: Option<f64>) -> Result<Event, String> {
        match &self.machine.controller {
            Some(Controller::HaasNGC(h))
                if h.h_t_agreement
                    && self
                        .active
                        .as_ref()
                        .is_none_or(|t| number != Some(t.number as f64)) =>
            {
                return Err("Haas Setting15 T/H disagreement".into());
            }
            None | Some(Controller::HaasNGC(_)) | Some(Controller::Carvera(_)) => {}
        }

        let Some(active) = self.active.clone() else {
            return Err("NC length without active tool".into());
        };
        let number = number.ok_or("NC tool/length register differs from configured table")?;
        if self.machine.dialect.length_activation == "tool_edge" {
            let edge = self
                .machine
                .tools
                .iter()
                .find(|t| t.number == active.number && f64::from(t.length_register) == number)
                .cloned()
                .ok_or("NC cutting edge absent from active tool table")?;
            if self.machine.tool_change_position_mode == "table" {
                self.position = edge
                    .position_after_mm
                    .ok_or("the tool table entry has no position_after_mm")?;
            }
            self.active = Some(edge);
        } else if number != f64::from(active.length_register) || !self.tool_pending {
            return Err("NC tool/length register differs from configured table".into());
        }
        let register = number as u32;
        self.length = Some(register);
        self.tool_pending = false;
        self.ready.insert("length".into());
        let t = self.active.as_ref().unwrap();
        Ok(Event::ToolChange {
            tool: t.tool.clone(),
            number: t.number,
            length_register: register,
            position_after_mm: self.position,
        })
    }
}

/// `_WORD.findall` over a block, refusing anything that is not a word
/// (`_WORD.sub('', body).strip()` must be empty).
fn words(body: &str) -> Result<Vec<(char, String)>, String> {
    let b = body.as_bytes();
    let mut out = vec![];
    let mut i = 0;
    let mut leftover = false;
    while i < b.len() {
        if b[i].is_ascii_alphabetic() {
            let letter = b[i] as char;
            let mut j = i + 1;
            while j < b.len() && b[j].is_ascii_whitespace() {
                j += 1;
            }
            let begin = j;
            if j < b.len() && matches!(b[j], b'+' | b'-') {
                j += 1;
            }
            let digits_start = j;
            while j < b.len() && b[j].is_ascii_digit() {
                j += 1;
            }
            let int_digits = j - digits_start;
            let mut ok = int_digits > 0;
            if j < b.len() && b[j] == b'.' {
                let mut k = j + 1;
                while k < b.len() && b[k].is_ascii_digit() {
                    k += 1;
                }
                if int_digits > 0 || k > j + 1 {
                    ok = true;
                    j = k;
                }
            }
            if ok {
                out.push((letter.to_ascii_uppercase(), body[begin..j].to_string()));
                i = j;
                continue;
            }
            leftover = true;
            i += 1;
        } else {
            if !b[i].is_ascii_whitespace() {
                leftover = true;
            }
            i += 1;
        }
    }
    if leftover {
        return Err("unsupported NC syntax".into());
    }
    Ok(out)
}
