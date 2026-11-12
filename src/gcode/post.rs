//! The default formatter: a port of the Python `GCodePost` (and the
//! `PostProcessor` helpers it uses), hook for hook, so that both write the
//! same blocks for the same events. Nothing here is trusted: the engine
//! decodes every block with the independent decoder (`super::decode`) and
//! the native intent checker compares the result with the planned events.
use super::controller::Controller;
use super::machine::{Machine, ToolSetting};
use crate::motion::{
    CompensationSide, Coolant, Curve, DrillCycle, Event, Motion, MoveKind, SpindleDirection, Vec3,
};
use crate::probe::{Leg, ProbeCycle, ProbeFeature, ProbeResult};
use std::collections::BTreeMap;

/// What a hook is called with.
pub enum Hook<'a> {
    /// `on_begin`: the program header (its geometry tolerance).
    Begin {
        tolerance_mm: f64,
    },
    Event(&'a Event),
    Finish,
}

#[derive(Clone)]
pub struct DefaultPost {
    machine: Machine,
    lines: Vec<String>,
    limit: usize,
    modal: BTreeMap<&'static str, String>,
    /// The program's geometry tolerance (circular word selection budget).
    pub geometry_tolerance_mm: f64,
    /// The decoded controller position before the event being written.
    pub position_mm: Option<Vec3>,
    /// The source event a (possibly lowered) motion was taken from.
    pub source_event: Option<Event>,
    spindle_on: bool,
    /// The beam process the controller is in (`beam_mark = "mode"`): marking
    /// (true), cutting (false) or not yet selected by this program.
    beam_marking: Option<bool>,
    coolant_mode: Coolant,
    css: bool,
    rpm: f64,
    spindle_direction: Option<SpindleDirection>,
    feed_rev: Option<f64>,
    pending_compensation: Option<Vec<String>>,
    cycle_open: bool,
    cycle_initial: Option<f64>,
    /// The plane the last plane code selected. Plane selection is modal
    /// (G17/G18/G19, modal group 2: LinuxCNC G-code reference, "Modal
    /// Groups"), so a code is written only for a change (#204); canned
    /// cycles drill along Z only in G17 (#183).
    plane: Option<crate::motion::Plane>,
    covered: u32,
    /// The moves still covered are a probing cycle's (none of its events
    /// may be written separately).
    covered_probe: bool,
    tool_setting: Option<ToolSetting>,
    /// Each program tool's cutter radius (a probe's stylus ball), by id:
    /// probing cycles are laid out from them.
    pub radii: BTreeMap<String, f64>,
    work_offset: Option<u32>,
    /// The next LinuxCNC O-word label for a protected probe move.
    probe_label: u32,
}

/// `PostProcessor.version`/`GCodePost.version` of the Python post this one
/// follows.
pub const VERSION: &str = "2";
pub const CARVERA_VERSION: &str = "carvera-nominal-2";
pub const HAAS_VERSION: &str = "haas-ngc-2020-2";

fn invalid_block(line: &str) -> bool {
    line.bytes()
        .any(|b| !(b == b'\t' || (0x20..=0x7e).contains(&b)))
}
fn direction_name(d: SpindleDirection) -> &'static str {
    match d {
        SpindleDirection::Cw => "cw",
        SpindleDirection::Ccw => "ccw",
    }
}
fn coolant_name(c: Coolant) -> &'static str {
    match c {
        Coolant::Off => "off",
        Coolant::Flood => "flood",
        Coolant::Mist => "mist",
    }
}
pub(crate) fn cycle_effect(cycle: DrillCycle) -> &'static str {
    match cycle {
        DrillCycle::Drill => "drill_cycle",
        DrillCycle::Dwell => "drill_dwell_cycle",
        DrillCycle::Peck => "peck_cycle",
        DrillCycle::ChipBreak => "chip_break_cycle",
        DrillCycle::Bore => "bore_cycle",
        DrillCycle::BoreDwell => "bore_dwell_cycle",
        DrillCycle::BoreStop => "bore_stop_cycle",
        DrillCycle::FineBore => "fine_bore_cycle",
    }
}
/// Python's `str.rstrip(chars)`.
fn rstrip(s: &str, c: char) -> &str {
    s.trim_end_matches(c)
}
/// Whether two edge angles differ by more than 1e-9 degrees (any turn).
fn edge_differs(a: f64, b: f64) -> bool {
    let d = ((a - b + 180.).rem_euclid(360.) - 180.).abs();
    d.is_nan() || d > 1e-9
}
/// Python's `format(value, 'g')` for the feeds in refusal messages: six
/// significant digits, without trailing zeros.
fn format_g(value: f64) -> String {
    format!("{:.5e}", value)
        .parse::<f64>()
        .map_or_else(|_| value.to_string(), |v| v.to_string())
}
/// `math.floor(math.log10(step) + 1e-9)` digits for a resolution step.
fn step_digits(step: f64) -> usize {
    (-(step.log10() + 1e-9).floor()).max(0.) as usize
}

impl DefaultPost {
    pub fn new(machine: Machine) -> Self {
        Self {
            machine,
            lines: vec![],
            limit: 8,
            modal: BTreeMap::new(),
            geometry_tolerance_mm: 1e-6,
            position_mm: None,
            source_event: None,
            spindle_on: false,
            beam_marking: None,
            coolant_mode: Coolant::Off,
            css: false,
            rpm: 0.,
            spindle_direction: None,
            feed_rev: None,
            pending_compensation: None,
            cycle_open: false,
            plane: None,
            cycle_initial: None,
            covered: 0,
            covered_probe: false,
            tool_setting: None,
            radii: BTreeMap::new(),
            work_offset: None,
            probe_label: 100,
        }
    }
    pub fn machine(&self) -> &Machine {
        &self.machine
    }

    fn emit(&mut self, words: &[&str]) -> Result<(), String> {
        let line = words
            .iter()
            .filter(|w| !w.is_empty())
            .copied()
            .collect::<Vec<_>>()
            .join(" ");
        if matches!(self.machine.controller, Some(Controller::Carvera(_))) {
            if ["G17", "G18", "G19"].contains(&line.as_str())
                && self.modal("carvera_plane", &line).is_none()
            {
                return Ok(());
            }
            // Switch.cpp changes air immediately; Carvera.cps drains queued
            // motion before both on and off. Keep every air hook synchronized.
            if line == "M7" || line == "M9" {
                self.emit(&["M400"])?;
            }
        }
        if self.lines.len() >= self.limit || line.len() > 1024 || invalid_block(&line) {
            return Err("post hook output exceeds block/ASCII/1024byte limit".into());
        }
        // A block that is a plane code alone makes that plane the active one,
        // whichever hook wrote it.
        if let Some(c) = self.machine.dialect.commands.iter().find(|c| {
            c.code == line && matches!(c.effect.as_str(), "plane_xy" | "plane_xz" | "plane_yz")
        }) {
            use crate::motion::Plane;
            self.plane = Some(match c.effect.as_str() {
                "plane_xy" => Plane::Xy,
                "plane_xz" => Plane::Xz,
                _ => Plane::Yz,
            });
        }
        self.lines.push(line);
        Ok(())
    }
    /// Write the plane's code unless that plane is already active
    /// (`GCodePost.select_plane`). The decoder keeps its own plane from the
    /// blocks it reads and decodes every arc in it, so an omitted code that
    /// was needed gives an arc that differs from the plan and is refused.
    fn select_plane(&mut self, plane: crate::motion::Plane) -> Result<(), String> {
        if self.plane != Some(plane) {
            let effect = format!("plane_{}", super::decode::plane_name(plane));
            let code = self.machine.dialect.code(&effect)?.to_string();
            self.emit(&[&code])?;
            self.plane = Some(plane);
        }
        Ok(())
    }
    fn emit_owned(&mut self, words: Vec<String>) -> Result<(), String> {
        let refs: Vec<&str> = words.iter().map(String::as_str).collect();
        self.emit(&refs)
    }
    /// `PostProcessor.number`.
    pub fn number(&self, value: f64, dimensional: bool) -> Result<String, String> {
        if !value.is_finite() || value.abs() > 1e6 {
            return Err("invalid post number".into());
        }
        let value = value / if dimensional { self.machine.unit() } else { 1. };
        let text = format!("{value:.*}", self.machine.decimals as usize);
        let text = rstrip(rstrip(&text, '0'), '.');
        Ok(if text.is_empty() || text == "-0" {
            "0".into()
        } else {
            text.into()
        })
    }
    pub fn word(&self, address: &str, value: f64, dimensional: bool) -> Result<String, String> {
        Ok(format!("{address}{}", self.number(value, dimensional)?))
    }
    /// `PostProcessor.words` with the standard numeric helper (diameter X on
    /// lathes).
    pub fn words(
        &self,
        addresses: &[&str],
        values: &[f64],
        dimensional: bool,
    ) -> Result<Vec<String>, String> {
        let d = &self.machine.dialect;
        let mut values = values.to_vec();
        if d.x_diameter && dimensional {
            for (a, v) in addresses.iter().zip(values.iter_mut()) {
                if *a == d.axes[0] {
                    *v *= 2.;
                }
            }
        }
        let unit = if dimensional { self.machine.unit() } else { 1. };
        let mut words = crate::post_decode::format_words(
            &addresses.concat(),
            &values,
            unit,
            self.machine.decimals as usize,
        )?;
        match &self.machine.controller {
            Some(Controller::HaasNGC(_)) if dimensional => {
                for w in &mut words {
                    if !w.contains('.') {
                        w.push('.');
                    }
                }
            }
            None | Some(Controller::HaasNGC(_)) | Some(Controller::Carvera(_)) => {}
        }
        Ok(words)
    }
    fn modal(&mut self, group: &'static str, word: &str) -> Option<String> {
        if self.modal.get(group).map(String::as_str) == Some(word) {
            return None;
        }
        self.modal.insert(group, word.to_string());
        Some(word.to_string())
    }
    fn comment(&mut self, text: &str) -> Result<(), String> {
        self.emit(&[&format!("; {text}")])
    }

    /// `PostProcessor._call`: run one hook and return its blocks.
    pub fn call(&mut self, hook: Hook<'_>, limit: usize) -> Result<Vec<String>, String> {
        self.lines.clear();
        self.limit = limit;
        let name = match &hook {
            Hook::Begin { .. } => "on_begin",
            Hook::Finish => "on_finish",
            Hook::Event(e) => match e {
                Event::Motion { .. } => "on_motion",
                Event::ToolChange { .. } => "on_tool_change",
                Event::Spindle { .. } => "on_spindle",
                Event::Coolant { .. } => "on_coolant",
                Event::WorkOffset { .. } => "on_work_offset",
                Event::Stop { .. } => "on_stop",
                Event::Dwell { .. } => "on_dwell",
                Event::Cycle { .. } => "on_cycle",
                Event::SpindleOrient { .. } => "on_spindle_orient",
                Event::Beam { .. } => "on_beam",
                Event::BeamMark { .. } => "on_beam_mark",
                Event::HeightControl { .. } => "on_height_control",
                Event::SurfaceSpeed { .. } => "on_surface_speed",
                Event::FeedPerRev { .. } => "on_feed_per_rev",
                Event::CutterCompensation { .. } => "on_cutter_compensation",
                Event::Probe { .. } => "on_probe",
            },
        };
        if self.covered > 0 && self.covered_probe {
            // A written probing cycle carries all its moves.
            if name != "on_motion" {
                return Err("a probing cycle covers only its own moves".into());
            }
            self.covered -= 1;
            if self.covered == 0 {
                self.covered_probe = false;
            }
            return Ok(vec![]);
        }
        if name == "on_probe" {
            self.limit = 1024;
        }
        if self.covered > 0 {
            // A written canned cycle carries the moves and dwell of its hole.
            if !["on_motion", "on_dwell", "on_spindle", "on_spindle_orient"].contains(&name) {
                return Err("a canned cycle covers only the moves, dwell and spindle stop/restart of its hole".into());
            }
            if let Hook::Event(Event::SpindleOrient { edge_angle_deg }) = hook {
                let edge = self.machine.dialect.oriented_edge_deg.unwrap_or(f64::NAN);
                if edge_differs(*edge_angle_deg, edge) {
                    return Err(format!(
                        "the operation's fine-bore edge_angle_deg {edge_angle_deg} differs from this machine's oriented_edge_deg {edge}"
                    ));
                }
            }
            self.covered -= 1;
            return Ok(vec![]);
        }
        if self.cycle_open && name != "on_cycle" {
            let code = self.machine.dialect.code("cycles_off")?.to_string();
            self.emit(&[&code])?;
            self.cycle_open = false;
        }
        if let Hook::Event(Event::ToolChange {
            tool,
            number,
            length_register,
            ..
        }) = &hook
        {
            // The active tool's table entry (its D register).
            self.tool_setting = self
                .machine
                .tools
                .iter()
                .find(|t| {
                    t.tool == *tool && t.number == *number && t.length_register == *length_register
                })
                .cloned();
        }
        match hook {
            Hook::Begin { tolerance_mm } => self.on_begin(tolerance_mm)?,
            Hook::Finish => self.on_finish()?,
            Hook::Event(event) => self.on_event(event)?,
        }
        if self.lines.is_empty()
            && name != "on_cycle"
            && !(name == "on_cutter_compensation" && self.machine.dialect.compensation_with_move)
        {
            return Err(format!("{name} emitted no blocks"));
        }
        Ok(self.lines.clone())
    }

    fn on_event(&mut self, event: &Event) -> Result<(), String> {
        let d = self.machine.dialect.clone();
        match event {
            Event::Motion { motion } => self.on_motion(motion),
            Event::WorkOffset { number } => {
                self.work_offset = Some(u32::from(*number));
                let code = d
                    .work_offsets
                    .get((*number as usize).wrapping_sub(1))
                    .ok_or("work offset absent from dialect")?;
                self.emit(&[code])
            }
            Event::ToolChange {
                number,
                length_register,
                ..
            } => {
                if let Some(Controller::Carvera(_)) = &self.machine.controller {
                    self.emit(&["G53 G0 Z-3"])?;
                    return self.emit(&[&format!("T{number}"), "M6"]);
                }
                if d.tool_change == "lathe_t" {
                    if !((1..=99).contains(number) && (1..=99).contains(length_register)) {
                        return Err("T<nn><oo> needs a tool and an offset number of 1..99".into());
                    }
                    return self
                        .emit(&[&format!("{}{number:02}{length_register:02}", d.tool_word)]);
                }
                self.emit(&[&format!("{}{number}", d.tool_word)])?;
                if d.tool_change == "m6" {
                    self.emit(&[d.code("tool_change")?])?;
                }
                if d.length_activation == "explicit" {
                    self.emit(&[
                        d.code("length_on")?,
                        &format!("{}{length_register}", d.length_word),
                    ])
                } else {
                    self.emit(&[&format!("{}{length_register}", d.length_word)])
                }
            }
            Event::Spindle { rpm, direction } => {
                if *rpm != 0. && self.css {
                    // Out of constant surface speed first: S is an rpm again.
                    self.emit(&[d.code("rpm_mode")?])?;
                    self.css = false;
                }
                if *rpm == 0. {
                    self.emit(&[d.code("spindle_off")?])?;
                } else {
                    let code = d.code(&format!("spindle_{}", direction_name(*direction)))?;
                    let s = self.word(&d.speed_word, *rpm, false)?;
                    self.emit(&[code, &s])?;
                }
                self.spindle_on = *rpm != 0.;
                self.rpm = *rpm;
                if *rpm != 0. {
                    self.spindle_direction = Some(*direction);
                }
                Ok(())
            }
            Event::Coolant { mode } => {
                // M9 to stop, or before switching between two active modes.
                let current = self.coolant_mode;
                if *mode == Coolant::Off || !(current == Coolant::Off || current == *mode) {
                    self.emit(&[d.code("coolant_off")?])?;
                }
                if *mode != Coolant::Off {
                    self.emit(&[d.code(coolant_name(*mode))?])?;
                }
                self.coolant_mode = *mode;
                Ok(())
            }
            Event::Stop { optional } => {
                self.emit(&[d.code(if *optional { "optional_stop" } else { "stop" })?])
            }
            Event::CutterCompensation { side, .. } => self.on_cutter_compensation(*side),
            Event::Probe { cycle, covers } => self.on_probe(cycle, *covers),
            Event::Beam { power_percent } => {
                if matches!(self.machine.controller, Some(Controller::Carvera(_))) {
                    return Err(
                        "Carvera reference supports milling only; beam control is not declared"
                            .into(),
                    );
                }
                let s_max = d.beam_s_max.ok_or(
                    "this dialect declares no beam (beam_s_max): it cannot post plasma or laser cutting",
                )?;
                if *power_percent == 0. {
                    self.emit(&[d.code("spindle_off")?])?;
                    self.spindle_on = false;
                    return Ok(());
                }
                // After marking (or at the program's first cut) the cutting
                // process is stated before the beam fires.
                if d.beam_mark.as_deref() == Some("mode") && self.beam_marking != Some(false) {
                    self.emit(&[d.code("cut_mode")?])?;
                    self.beam_marking = Some(false);
                }
                let s = self.word(&d.speed_word, power_percent * s_max / 100., false)?;
                self.emit(&[d.code(&d.beam_effect)?, &s])?;
                self.spindle_on = true;
                Ok(())
            }
            Event::BeamMark { power_percent } => {
                if matches!(self.machine.controller, Some(Controller::Carvera(_))) {
                    return Err(
                        "Carvera reference supports milling only; beam control is not declared"
                            .into(),
                    );
                }
                let s_max = d.beam_s_max.ok_or(
                    "this dialect declares no beam (beam_s_max): it cannot post plasma or laser cutting",
                )?;
                let Some(mark) = d.beam_mark.as_deref() else {
                    return Err(
                        "this dialect declares no beam marking (beam_mark): declare how the machine marks ('power', or 'mode' with its mark_mode and cut_mode codes)"
                            .into(),
                    );
                };
                if mark == "mode" && self.beam_marking != Some(true) {
                    self.emit(&[d.code("mark_mode")?])?;
                    self.beam_marking = Some(true);
                }
                let s = self.word(&d.speed_word, power_percent * s_max / 100., false)?;
                self.emit(&[d.code(&d.beam_effect)?, &s])?;
                self.spindle_on = true;
                Ok(())
            }
            Event::HeightControl { on } => {
                let effect = if *on {
                    "height_control_on"
                } else {
                    "height_control_off"
                };
                if !d.declares(effect) {
                    return Err(
                        "this dialect declares no torch height control (height_control_on/off)"
                            .into(),
                    );
                }
                match &d.height_control_word {
                    Some(w) => self.emit(&[d.code(effect)?, w]),
                    None => self.emit(&[d.code(effect)?]),
                }
            }
            Event::SurfaceSpeed {
                m_min,
                max_rpm,
                direction,
            } => {
                if !d.declares("css_on") {
                    return Err(
                        "this dialect declares no constant surface speed (css_on, e.g. G96)".into(),
                    );
                }
                let speed = m_min
                    / if self.machine.output_units == "inch" {
                        0.3048
                    } else {
                        1.
                    };
                let limit_word = d.css_limit_word.as_deref().unwrap_or(&d.speed_word);
                let limit = self.word(limit_word, *max_rpm, false)?;
                if d.declares("css_limit") {
                    self.emit(&[d.code("css_limit")?, &limit])?;
                }
                let s = self.word(&d.speed_word, speed, false)?;
                let spindle = d.code(&format!("spindle_{}", direction_name(*direction)))?;
                if d.declares("css_limit") {
                    self.emit(&[d.code("css_on")?, &s, spindle])?;
                } else {
                    self.emit(&[d.code("css_on")?, &limit, &s, spindle])?;
                }
                self.spindle_on = true;
                self.css = true;
                self.rpm = *max_rpm;
                self.spindle_direction = Some(*direction);
                Ok(())
            }
            Event::FeedPerRev { mm_rev } => {
                let m = &self.machine;
                if *mm_rev == 0. {
                    self.emit(&[d.code("feed_per_minute")?])?;
                    self.feed_rev = None;
                    return Ok(());
                }
                if !d.declares("feed_per_rev") {
                    return Err(
                        "this dialect declares no feed per revolution (feed_per_rev, e.g. G95)"
                            .into(),
                    );
                }
                if m.schedule_post_feeds {
                    return Err(
                        "feeds per revolution are not rescheduled per minute: post without schedule_post_feeds".into(),
                    );
                }
                let word = self.word(&d.feed_word, *mm_rev, true)?;
                let written: f64 = word[d.feed_word.len()..]
                    .parse::<f64>()
                    .map_err(|e| e.to_string())?
                    * m.unit();
                if (written - mm_rev).abs() > m.feed_tolerance_mm_min / m.max_spindle_rpm.max(1.) {
                    return Err(format!(
                        "feed per revolution {mm_rev} mm is written {word}: not within the feed tolerance at {} rpm; use a feed representable at {} decimals ({})",
                        m.max_spindle_rpm, m.decimals, m.output_units
                    ));
                }
                self.emit(&[d.code("feed_per_rev")?, &word])?;
                self.feed_rev = Some(*mm_rev);
                Ok(())
            }
            Event::SpindleOrient { edge_angle_deg } => {
                let Some(edge) = d.oriented_edge_deg.filter(|_| d.declares("spindle_orient"))
                else {
                    return Err("this dialect declares no oriented spindle stop (spindle_orient, oriented_edge_deg): fine boring cannot be posted".into());
                };
                if edge_differs(*edge_angle_deg, edge) {
                    return Err(format!(
                        "the operation's fine-bore edge_angle_deg {edge_angle_deg} differs from this machine's oriented_edge_deg {edge}"
                    ));
                }
                self.emit(&[d.code("spindle_orient")?])?;
                self.spindle_on = false;
                Ok(())
            }
            Event::Dwell { seconds } => {
                // Rounded up to the controller's dwell resolution: never shorter.
                let step = d.dwell_resolution_s;
                let ticks = (seconds / step - 1e-9).ceil();
                if ticks.is_nan() || ticks < 1. || ticks * step > d.dwell_max_s + 1e-12 {
                    return Err(format!(
                        "dwell {seconds}s outside the controller range {step}..{}s",
                        d.dwell_max_s
                    ));
                }
                let text = format!("{:.*}", step_digits(step), ticks * step);
                self.emit(&[
                    d.code("dwell")?,
                    &format!("{}{}", d.dwell_word, rstrip(&text, '0')),
                ])
            }
            Event::Cycle { .. } => self.on_cycle(event),
        }
    }

    /// `GCodePost.on_probe`: a probing cycle as the dialect's probing
    /// blocks, LinuxCNC G38.x (`g38`) or Renishaw Inspection Plus macro
    /// calls (`inspection_plus`). The blocks carry every move it covers.
    fn on_probe(&mut self, cycle: &ProbeCycle, covers: u32) -> Result<(), String> {
        let back_end = self.machine.dialect.probing.clone().ok_or(
            "this machine's dialect declares no probing back end: GCodeDialect(probing='g38') for \
             LinuxCNC G38.x or probing='inspection_plus' for Renishaw Inspection Plus macros",
        )?;
        let no_tool = "a probing cycle needs an active tool from the machine table";
        let setting = self.tool_setting.as_ref().ok_or(no_tool)?;
        let radius = *self.radii.get(&setting.tool).ok_or(no_tool)?;
        let start = cycle.start(radius)?;
        let legs = cycle.legs(radius)?;
        let position = self.position_mm;
        let Some(position) = position.filter(|p| {
            (0..3).map(|k| (p[k] - start[k]).abs()).fold(0., f64::max) <= self.machine.tolerance_mm
        }) else {
            return Err(format!(
                "a probing cycle starts at {start:?}, not at {position:?}"
            ));
        };
        let lines = if back_end == "g38" {
            self.probe_g38(cycle, &legs, radius, position)?
        } else {
            self.probe_inspection_plus(cycle, &legs, position)?
        };
        for line in lines {
            self.emit(&[&line])?;
        }
        self.covered = covers;
        self.covered_probe = true;
        Ok(())
    }

    /// The axis words of a probing move from `position` to `end`.
    fn changed(&self, position: Vec3, end: Vec3) -> Result<Vec<String>, String> {
        let d = &self.machine.dialect;
        let axes: Vec<usize> = (0..3).filter(|&i| end[i] != position[i]).collect();
        if axes.is_empty() {
            return Err("a probing move has zero length".into());
        }
        let names: Vec<&str> = axes.iter().map(|&i| d.axes[i].as_str()).collect();
        let values: Vec<f64> = axes.iter().map(|&i| end[i]).collect();
        self.words(&names, &values, true)
    }

    fn feed_words(&self, feed: Option<f64>) -> Result<Vec<String>, String> {
        let feed = feed.ok_or("a probing move needs its feed")?;
        self.words(&[&self.machine.dialect.feed_word], &[feed], true)
    }

    /// `GCodePost._probe_g38`: LinuxCNC 2.9 (docs/src/gcode/g-code.adoc
    /// G38.n, G10 L1/L2/L10; overview.adoc numbered parameters). Protected
    /// moves: G38.3, then abort if #5070 = 1 (tripped). Measuring moves:
    /// G38.2 to search_mm beyond the expected contact, which errors if
    /// nothing is touched; the error of each touch is #506x minus the
    /// expected tip coordinate.
    fn probe_g38(
        &mut self,
        cycle: &ProbeCycle,
        legs: &[Leg],
        radius: f64,
        position: Vec3,
    ) -> Result<Vec<String>, String> {
        let d = self.machine.dialect.clone();
        let mut out = vec![];
        let search = cycle.search_mm;
        let mut touches: Vec<usize> = vec![];
        let mut pos = position;
        let mut i = 0;
        while i < legs.len() {
            let Leg {
                kind,
                mut end,
                feed_mm_min: feed,
            } = legs[i];
            match kind {
                MoveKind::ProbePosition => {
                    let label = self.probe_label;
                    self.probe_label += 1;
                    let mut words = vec!["G38.3".to_string()];
                    words.extend(self.changed(pos, end)?);
                    words.extend(self.feed_words(feed)?);
                    out.push(words.join(" "));
                    out.push(format!("O{label} IF [#5070 NE 0]"));
                    out.push("(ABORT, camlib: protected probe move obstructed)".into());
                    out.push(format!("O{label} ENDIF"));
                }
                MoveKind::ProbeMeasure => {
                    let touch = legs.get(i + 1).copied();
                    let Some(touch) =
                        touch.filter(|t| t.kind == MoveKind::ProbeTouch && t.feed_mm_min == feed)
                    else {
                        return Err("a measuring move continues into its touch".into());
                    };
                    let contact = touch.end;
                    let axis = (0..3)
                        .find(|&k| contact[k] != pos[k])
                        .ok_or("a measuring move has zero length")?;
                    let direction = if contact[axis] > pos[axis] { 1. } else { -1. };
                    let target = contact[axis] + direction * search;
                    touches.push(axis);
                    let mut words = vec!["G38.2".to_string()];
                    words.extend(self.words(&[&d.axes[axis]], &[target], true)?);
                    words.extend(self.feed_words(feed)?);
                    out.push(words.join(" "));
                    out.push(format!(
                        "#<probe{}> = [#{} {} {}]",
                        touches.len(),
                        5061 + axis,
                        if contact[axis] < 0. { "+" } else { "-" },
                        self.number(contact[axis].abs(), true)?
                    ));
                    end = contact;
                    i += 1;
                }
                MoveKind::ProbeReturn => {
                    let mut words = vec!["G1".to_string()];
                    words.extend(self.changed(pos, end)?);
                    words.extend(self.feed_words(feed)?);
                    out.push(words.join(" "));
                }
                MoveKind::Rapid => {
                    let mut words = vec!["G0".to_string()];
                    words.extend(self.changed(pos, end)?);
                    out.push(words.join(" "));
                }
                other => return Err(format!("unexpected probing move {other:?}")),
            }
            pos = end;
            i += 1;
        }
        match cycle.result {
            ProbeResult::WorkOffset { number } => {
                let active = self
                    .work_offset
                    .ok_or("probing a work offset needs the active work offset selected first")?;
                let mut words = vec!["G10 L2".to_string(), format!("P{number}")];
                for (axis, letter) in d.axes.iter().enumerate().take(3) {
                    let used: Vec<usize> = touches
                        .iter()
                        .enumerate()
                        .filter(|(_, a)| **a == axis)
                        .map(|(k, _)| k + 1)
                        .collect();
                    let error = match used.as_slice() {
                        [] => continue,
                        [one] => format!("#<probe{one}>"),
                        [a, b] => format!("[#<probe{a}> + #<probe{b}>] / 2"),
                        _ => return Err("at most two touches along one axis".into()),
                    };
                    words.push(format!(
                        "{}[#{} + {error}]",
                        letter,
                        5201 + 20 * active as usize + axis
                    ));
                }
                out.push(words.join(" "));
            }
            ProbeResult::ToolLength { register } => {
                out.push(format!(
                    "G10 L10 P{register} {}[#5422 - #<probe1>]",
                    d.axes[2]
                ));
                out.push(format!(
                    "{} {}{register}",
                    d.code("length_on")?,
                    d.length_word
                ));
            }
            ProbeResult::ToolRadius { register } => {
                out.push(format!(
                    "G10 L1 P{register} R[{} + [#<probe1> - #<probe2>] / 2]",
                    self.number(radius, true)?
                ));
            }
        }
        Ok(out)
    }

    /// `GCodePost._macro`: a G65 argument with its decimal point, as the
    /// Inspection Plus examples write them (H-2000-6031:
    /// "G65P9814D50.Z-10.S2.R10.").
    fn macro_number(&self, value: f64) -> Result<String, String> {
        let text = self.number(value, true)?;
        Ok(if text.contains('.') { text } else { text + "." })
    }

    /// `GCodePost._probe_inspection_plus`: Renishaw Inspection Plus
    /// (H-2000-6031 chapters 5 and 7). O9810 protected positioning (X Y Z
    /// F), O9811 single surface (X, Y or Z: the surface position), O9812
    /// pocket (X or Y: its width), O9814 bore (D), O9815 inside corner (X Y:
    /// the corner), each with S (work offset S1..S6 = G54..G59; "New work
    /// offset = active work offset + error") and Q (overtravel: the search
    /// beyond the expected position).
    fn probe_inspection_plus(
        &self,
        cycle: &ProbeCycle,
        legs: &[Leg],
        position: Vec3,
    ) -> Result<Vec<String>, String> {
        let d = &self.machine.dialect;
        let feature = &cycle.feature;
        let calls = match feature {
            ProbeFeature::Surface { .. } => 1,
            ProbeFeature::OutsideCorner { .. } => 1,
            ProbeFeature::InsideCorner { .. } => 2,
            ProbeFeature::Bore { .. } => 4,
            ProbeFeature::Pocket { .. } => 2,
            other => {
                let name = match other {
                    ProbeFeature::Boss { .. } => "boss",
                    ProbeFeature::Web { .. } => "web",
                    ProbeFeature::ToolLength { .. } => "tool_length",
                    _ => "tool_diameter",
                };
                return Err(format!(
                    "{name} probing is not modelled for the Inspection Plus macros: their moves around a boss or web \
                     (O9814/O9812 with Z and R) and their tool setting cycles are not taken from a source here; \
                     post it with probing=\"g38\" or split it into single surfaces"
                ));
            }
        };
        let ProbeResult::WorkOffset { number } = cycle.result else {
            return Err("Inspection Plus cycles correct a work offset (S1..S6)".into());
        };
        if !(1..=6).contains(&number) {
            return Err(
                "Inspection Plus S1..S6 correct G54..G59; this cycle corrects another work offset"
                    .into(),
            );
        }
        let fast = d
            .probe_fast_feed_mm_min
            .ok_or("Inspection Plus probing needs the installed fast feed #119")?;
        let z_surface = matches!(
            feature,
            ProbeFeature::Surface {
                axis: crate::probe::Axis::Z,
                ..
            }
        );
        let (measure, back) = (0.6 * fast, if z_surface { 0.6 } else { 1. } * fast);
        let tolerance = self.machine.feed_tolerance_mm_min;
        if (cycle.measure_feed_mm_min - measure).abs() > tolerance
            || (cycle.return_feed_mm_min - back).abs() > tolerance
        {
            return Err(format!(
                "Inspection Plus measures at 0.6 x the fast feed #119 ({} mm/min) and moves back at \
                 {} mm/min (#119{}): set measure_feed_mm_min and return_feed_mm_min to those",
                format_g(measure),
                format_g(back),
                if z_surface { " x 0.6 in Z" } else { " in XY" }
            ));
        }
        let tail = format!("S{number}. Q{}", self.macro_number(cycle.search_mm)?);
        let mut out = vec![];
        if d.probe_switching {
            out.push("G65 P9832".to_string());
        }
        let mut pos = position;
        let mut i = 0;
        while i < legs.len() {
            let leg = legs[i];
            if leg.kind == MoveKind::ProbePosition {
                let mut words = vec!["G65 P9810".to_string()];
                for (k, (&end, &at)) in leg.end.iter().zip(pos.iter()).enumerate() {
                    if end != at {
                        words.push(format!("{}{}", d.axes[k], self.macro_number(end)?));
                    }
                }
                let feed = leg.feed_mm_min.ok_or("a probing move needs its feed")?;
                words.push(format!("{}{}", d.feed_word, self.macro_number(feed)?));
                out.push(words.join(" "));
                pos = leg.end;
                i += 1;
                continue;
            }
            if leg.kind != MoveKind::ProbeMeasure {
                return Err(format!("unexpected probing move {:?}", leg.kind));
            }
            let group = &legs[i..(i + 3 * calls).min(legs.len())];
            let pattern = [
                MoveKind::ProbeMeasure,
                MoveKind::ProbeTouch,
                MoveKind::ProbeReturn,
            ];
            if group.len() != 3 * calls
                || group
                    .iter()
                    .enumerate()
                    .any(|(k, g)| g.kind != pattern[k % 3])
            {
                return Err("an Inspection Plus call covers whole touches".into());
            }
            let axis = (0..3)
                .find(|&k| group[1].end[k] != pos[k])
                .ok_or("a measuring move has zero length")?;
            let call = match feature {
                ProbeFeature::Surface { surface_mm, .. } => {
                    format!(
                        "G65 P9811 {}{}",
                        d.axes[axis],
                        self.macro_number(*surface_mm)?
                    )
                }
                ProbeFeature::OutsideCorner { corner_mm, .. } => {
                    let value = *corner_mm
                        .get(axis)
                        .ok_or("an outside corner is touched in X or Y")?;
                    format!("G65 P9811 {}{}", d.axes[axis], self.macro_number(value)?)
                }
                ProbeFeature::InsideCorner { corner_mm, .. } => format!(
                    "G65 P9815 {}{} {}{}",
                    d.axes[0],
                    self.macro_number(corner_mm[0])?,
                    d.axes[1],
                    self.macro_number(corner_mm[1])?
                ),
                ProbeFeature::Bore { diameter_mm, .. } => {
                    format!("G65 P9814 D{}", self.macro_number(*diameter_mm)?)
                }
                ProbeFeature::Pocket {
                    axis: a, width_mm, ..
                } => {
                    let a = match a {
                        crate::probe::Axis::X => 0,
                        crate::probe::Axis::Y => 1,
                        crate::probe::Axis::Z => {
                            return Err("an Inspection Plus pocket is measured in X or Y".into());
                        }
                    };
                    format!("G65 P9812 {}{}", d.axes[a], self.macro_number(*width_mm)?)
                }
                _ => unreachable!("refused above"),
            };
            out.push(format!("{call} {tail}"));
            pos = group[group.len() - 1].end;
            i += 3 * calls;
        }
        if d.probe_switching {
            out.push("G65 P9833".to_string());
        }
        Ok(out)
    }

    fn on_begin(&mut self, tolerance_mm: f64) -> Result<(), String> {
        if let Some(Controller::Carvera(_)) = &self.machine.controller {
            self.geometry_tolerance_mm = tolerance_mm;
            for code in ["G21", "G90", "G17", "G94", "M5", "M9"] {
                self.emit(&[code])?;
            }
            return Ok(());
        }
        match &self.machine.controller {
            Some(Controller::HaasNGC(h)) => {
                let number = h.program_number;
                self.emit(&["%"])?;
                self.emit(&[&format!("O{number}")])?;
            }
            None | Some(Controller::Carvera(_)) => {}
        }
        self.geometry_tolerance_mm = tolerance_mm;
        let d = self.machine.dialect.clone();
        for optional in ["language_native", "compression_off"] {
            if d.declares(optional) {
                self.emit(&[d.code(optional)?])?;
            }
        }
        let mode = if d.x_diameter {
            "lathe_diameter_mode"
        } else {
            "lathe_radius_mode"
        };
        if d.declares(mode) {
            self.emit(&[d.code(mode)?])?;
        }
        // The machine's first plane (G18 on a lathe).
        let plane = if self.machine.planes.contains(&crate::motion::Plane::Xy) {
            "plane_xy".to_string()
        } else {
            let first = self
                .machine
                .planes
                .first()
                .ok_or("machine declares no plane")?;
            format!("plane_{}", super::decode::plane_name(*first))
        };
        let units = if self.machine.output_units == "mm" {
            "units_mm"
        } else {
            "units_inch"
        };
        for effect in [
            units,
            "absolute",
            "center_incremental",
            plane.as_str(),
            "feed_per_minute",
            "rpm_mode",
            "radius_off",
            "length_off",
            "cycles_off",
            "frame_reset",
            "exact_stop",
            "spindle_off",
            "coolant_off",
        ] {
            if !d.fixed(effect) {
                self.emit(&[d.code(effect)?])?;
            }
        }
        match &self.machine.controller {
            Some(Controller::HaasNGC(h)) => {
                let rotation_scaling = h.rotation_scaling;
                self.lines.retain(|line| line != "G61");
                for line in &mut self.lines {
                    if line == "G52" {
                        *line = "G52 X0 Y0 Z0".into();
                    }
                }
                if rotation_scaling {
                    self.emit(&["G50"])?;
                    self.emit(&["G69"])?;
                }
                self.emit(&["G100 X0 Y0 Z0"])?;
                self.emit(&["G187"])?;
                self.emit(&["G61"])?;
            }
            None | Some(Controller::Carvera(_)) => {}
        }
        self.spindle_on = false;
        self.coolant_mode = Coolant::Off;
        Ok(())
    }

    fn on_cutter_compensation(&mut self, side: CompensationSide) -> Result<(), String> {
        let d = self.machine.dialect.clone();
        if side == CompensationSide::Off {
            if d.compensation_with_move {
                self.pending_compensation = Some(vec![d.code("radius_off")?.to_string()]);
            } else {
                self.emit(&[d.code("radius_off")?])?;
            }
            return Ok(());
        }
        let code = d
            .code(if side == CompensationSide::Left {
                "radius_left"
            } else {
                "radius_right"
            })?
            .to_string();
        let words = if !d.compensation_register {
            vec![code]
        } else {
            if !d.compensation_word_free() {
                return Err(format!(
                    "this dialect uses {} for another word: it cannot write cutter compensation registers",
                    d.compensation_word
                ));
            }
            let register = self
                .tool_setting
                .as_ref()
                .and_then(|t| t.diameter_register)
                .ok_or("cutter compensation needs the tool's D register: declare ToolSetting(diameter_register=...) holding its wear")?;
            vec![code, format!("{}{register}", d.compensation_word)]
        };
        // With the move: written by on_motion in the next (straight XY) block.
        if d.compensation_with_move {
            self.pending_compensation = Some(words);
            Ok(())
        } else {
            self.emit_owned(words)
        }
    }

    /// Canned cycles (G73-G89, G84/G74) drill along the plane's normal: in
    /// G18 LinuxCNC drills along Y ("performed with respect to the
    /// currently-selected plane", LinuxCNC G-code reference, Canned Cycles).
    /// camlib's cycles drill along Z, so G17 is selected first (#183).
    fn cycle_plane(&mut self) -> Result<(), String> {
        use crate::motion::Plane;
        if self.machine.planes.contains(&Plane::Xy) {
            self.select_plane(Plane::Xy)?;
        }
        Ok(())
    }

    fn on_cycle(&mut self, event: &Event) -> Result<(), String> {
        let Event::Cycle {
            cycle,
            center_mm,
            z_mm,
            r_mm,
            initial_z_mm,
            feed_mm_min,
            peck_mm,
            dwell_s,
            reentry_mm,
            lift_mm,
            shift_mm,
            covers,
        } = event
        else {
            unreachable!()
        };
        match &self.machine.controller {
            Some(Controller::HaasNGC(h)) => {
                if matches!(cycle, DrillCycle::Peck | DrillCycle::ChipBreak) {
                    let code = if *cycle == DrillCycle::Peck {
                        "G83"
                    } else {
                        "G73"
                    };
                    let Some(setting) = h.setting_22_mm else {
                        return Err(format!(
                            "Haas {code} needs Setting 22 (Can Cycle Delta Z): declare setting_22_mm with the installed value"
                        ));
                    };
                    if *cycle == DrillCycle::Peck {
                        let Some(s52) = h.setting_52_mm else {
                            return Err("Haas G83 needs Setting 52 (G83 Retract Above R): declare setting_52_mm".into());
                        };
                        if s52 != 0. {
                            return Err(format!(
                                "Haas G83 with Setting 52 = {s52} mm clears chips above R, which camlib does not model: set Setting 52 to 0 (setting_52_mm=0) or drill without canned_cycle"
                            ));
                        }
                    }
                    let (source, name) = if *cycle == DrillCycle::Peck {
                        (*reentry_mm, "reentry_clearance_mm")
                    } else {
                        (*lift_mm, "chip_break_lift_mm")
                    };
                    let source = source.ok_or_else(|| format!("Haas {code} requires the operation {name}: declare its return distance in mm"))?;
                    if (source - setting).abs() > 1e-9 {
                        return Err(format!(
                            "the operation {name} {} mm differs from Setting 22 (Can Cycle Delta Z) {setting} mm: set {name}={setting}",
                            source
                        ));
                    }
                }
            }
            None | Some(Controller::Carvera(_)) => {}
        }
        let d = self.machine.dialect.clone();
        let effect = cycle_effect(*cycle);
        if self.feed_rev.is_some_and(|f| f != 0.) {
            return Err(
                "canned cycles are written with feeds per minute: return to G94 first".into(),
            );
        }
        // A cycle needs its own code, G98 return and G80 cancel.
        if !(d.declares(effect) && d.declares("cycle_return_initial") && d.declares("cycles_off")) {
            return Ok(());
        }
        for (kind, source, machine, name) in [
            (
                DrillCycle::Peck,
                *reentry_mm,
                d.peck_reentry_mm,
                "reentry_clearance_mm",
            ),
            (
                DrillCycle::ChipBreak,
                *lift_mm,
                d.chip_break_lift_mm,
                "chip_break_lift_mm",
            ),
        ] {
            if *cycle == kind {
                let (Some(source), Some(machine)) = (source, machine) else {
                    return Err(format!(
                        "the operation {name} is missing for this pecking cycle"
                    ));
                };
                if (source - machine).abs() > 1e-9 {
                    return Err(format!(
                        "the operation {name} {source} mm differs from this controller's {} return amount {machine} mm: set {name}={machine}",
                        d.code(effect)?
                    ));
                }
            }
        }
        self.cycle_plane()?;
        if self.cycle_open && self.cycle_initial != Some(*initial_z_mm) {
            // FANUC keeps the initial level from where the cycle mode began.
            self.emit(&[d.code("cycles_off")?])?;
        }
        self.cycle_initial = Some(*initial_z_mm);
        if let Some(word) = self.modal("cycle_return", d.code("cycle_return_initial")?) {
            self.emit(&[&word])?;
        }
        let addresses = [
            d.axes[0].as_str(),
            d.axes[1].as_str(),
            d.axes[2].as_str(),
            "R",
        ];
        let mut words = self.words(
            &addresses,
            &[center_mm[0], center_mm[1], *z_mm, *r_mm],
            true,
        )?;
        for q in [peck_mm, shift_mm].into_iter().flatten() {
            let q = self.word("Q", *q, true)?;
            // Always with a decimal point: no control reads it as an increment count.
            words.push(if q.contains('.') { q } else { q + "." });
        }
        if let Some(s) = dwell_s {
            words.push(self.cycle_dwell(*s)?);
        }
        words.extend(self.words(&[d.feed_word.as_str()], &[*feed_mm_min], true)?);
        let mut all = vec![d.code(effect)?.to_string()];
        all.extend(words);
        self.emit_owned(all)?;
        self.cycle_open = true;
        self.covered = *covers;
        Ok(())
    }

    /// The G82 P word, rounded up to the controller resolution.
    fn cycle_dwell(&self, seconds: f64) -> Result<String, String> {
        let d = &self.machine.dialect;
        if d.cycle_dwell_unit == "ms" {
            let ms = (seconds * 1000. - 1e-9).ceil();
            if ms.is_nan() || ms < 1. || ms > d.dwell_max_s * 1000. {
                return Err(format!("dwell {seconds}s outside the controller range"));
            }
            return Ok(format!("P{ms}"));
        }
        let step = d.dwell_resolution_s;
        let ticks = (seconds / step - 1e-9).ceil();
        if ticks.is_nan() || ticks < 1. || ticks * step > d.dwell_max_s + 1e-12 {
            return Err(format!(
                "dwell {seconds}s outside the controller range {step}..{}s",
                d.dwell_max_s
            ));
        }
        let text = format!("{:.*}", step_digits(step), ticks * step);
        Ok(format!("P{}", rstrip(&text, '0')))
    }

    fn on_motion(&mut self, motion: &Motion) -> Result<(), String> {
        if motion.kind.is_probe() {
            // The program validator keeps probing moves inside their cycle,
            // whose blocks carry them.
            return Err("a probing move outside its probing cycle".into());
        }
        let d = self.machine.dialect.clone();
        let c = &motion.curve;
        if motion.kind == MoveKind::Tap {
            return self.tap(motion);
        }
        if motion.kind == MoveKind::Thread {
            // The lead (feed per spindle revolution), not a feed.
            if !d.declares("thread_synchronized") {
                return Err(
                    "this dialect declares no spindle-synchronized threading (thread_synchronized)"
                        .into(),
                );
            }
            let rpm = self.rpm;
            if rpm == 0. || !matches!(c, Curve::Line { .. }) {
                return Err("threading needs a running spindle and a straight move".into());
            }
            let end = c.end();
            let feed = motion
                .feed_mm_min
                .ok_or("threading needs its synchronized feed")?;
            let mut all = vec![d.code("thread_synchronized")?.to_string()];
            all.extend(self.words(&[&d.axes[0], &d.axes[2]], &[end[0], end[2]], true)?);
            all.extend(self.words(&[&d.thread_lead_word], &[feed / rpm], true)?);
            return self.emit_owned(all);
        }
        let circular = match c {
            Curve::Arc {
                plane, sweep_rad, ..
            }
            | Curve::Helix {
                plane, sweep_rad, ..
            } => Some((*plane, *sweep_rad, matches!(c, Curve::Helix { .. }))),
            Curve::Line { .. } => None,
        };
        match (&self.machine.controller, circular) {
            (None, _)
            | (Some(Controller::HaasNGC(_)), None)
            | (Some(Controller::Carvera(_)), _) => {}
            (Some(Controller::HaasNGC(_)), Some((plane, sweep, helix))) => {
                if plane != crate::motion::Plane::Xy {
                    return Err("native Haas reference arc requires G17".into());
                }
                let feed = motion
                    .feed_mm_min
                    .ok_or("Haas circular motion requires positive explicit feed")?;
                self.select_plane(crate::motion::Plane::Xy)?;
                let (end, offsets) = self.circular_values(c)?;
                let full = c.start()[..2] == c.end()[..2];
                let mut addresses = Vec::new();
                let mut values = Vec::new();
                if !full {
                    addresses.extend(["X", "Y"]);
                    values.extend([end[0], end[1]]);
                }
                if helix {
                    addresses.push("Z");
                    values.push(end[2]);
                }
                addresses.extend(["I", "J", "F"]);
                values.extend([offsets[0], offsets[1], feed]);
                let mut words = vec![if sweep > 0. { "G3".into() } else { "G2".into() }];
                words.extend(self.words(&addresses, &values, true)?);
                return self.emit_owned(words);
            }
        }
        let effect = if let Some((plane, sweep, _)) = circular {
            self.select_plane(plane)?;
            if sweep > 0. { "arc_ccw" } else { "arc_cw" }
        } else if motion.kind.is_rapid() && motion.feed_mm_min.is_none() {
            "rapid"
        } else {
            "linear"
        };
        let mut axes: Vec<usize> = vec![0, 1, 2];
        if let Some((plane, _, helix)) = circular
            && !helix
        {
            let normal = plane.axes()[2];
            axes.retain(|i| *i != normal);
        }
        let mut addresses: Vec<String> = axes.iter().map(|i| d.axes[*i].clone()).collect();
        let end = c.end();
        let mut values: Vec<f64> = axes.iter().map(|i| end[*i]).collect();
        // Vertical in the plan: this motion, or the source it was lowered from.
        let start = c.start();
        let source_vertical = matches!(
            &self.source_event,
            Some(Event::Motion { motion: Motion { curve: Curve::Line { start, end }, .. } })
                if start[0] == end[0] && start[1] == end[1]
        );
        let vertical = (start[0] == end[0] && start[1] == end[1]) || source_vertical;
        if circular.is_none()
            && let Some(position) = self.position_mm
            && vertical
            && (position[0] - end[0]).hypot(position[1] - end[1])
                + 0.5 * self.machine.unit() * 10f64.powi(-(self.machine.decimals as i32))
                <= self.machine.tolerance_mm
        {
            // A straight vertical move keeps the controller's current XY (an
            // arc's representable end may differ by a few words) whenever the
            // block stays within the tolerance of the source: its XY offset
            // plus half a word of Z rounding. Writing the source XY would add
            // a sideways step to a move leaving the surface it touches.
            values = axes
                .iter()
                .map(|i| if *i < 2 { position[*i] } else { end[*i] })
                .collect();
        }
        if let Some((plane, _, _)) = circular {
            let [u, v, _] = plane.axes();
            addresses.push(d.centers[u].clone());
            addresses.push(d.centers[v].clone());
            let (e, offsets) = self.circular_values(c)?;
            values = axes.iter().map(|i| e[*i]).collect();
            values.extend([offsets[u], offsets[v]]);
        }
        if let Some(feed) = motion.feed_mm_min {
            let per_rev = self.feed_rev.filter(|f| *f != 0.);
            if per_rev.is_some() && motion.kind.is_rapid() {
                return Err(
                    "a programmed traverse feed cannot be written in feed per revolution mode"
                        .into(),
                );
            }
            addresses.push(d.feed_word.clone());
            values.push(per_rev.unwrap_or(feed));
        }
        let carried = self.pending_compensation.take().unwrap_or_default();
        if !carried.is_empty() && (circular.is_some() || !["rapid", "linear"].contains(&effect)) {
            return Err("cutter compensation turns on/off over a straight G0/G1 move".into());
        }
        let refs: Vec<&str> = addresses.iter().map(String::as_str).collect();
        let mut all = vec![d.code(effect)?.to_string()];
        all.extend(carried);
        all.extend(self.words(&refs, &values, true)?);
        self.emit_owned(all)
    }

    /// A tapping pair: the downward motion is the whole cycle; the return is
    /// part of it (G33.1 returns by itself; G84 needs G80 afterwards).
    fn tap(&mut self, motion: &Motion) -> Result<(), String> {
        let d = self.machine.dialect.clone();
        let (start, end) = (motion.curve.start(), motion.curve.end());
        if self.feed_rev.is_some_and(|f| f != 0.) || self.css {
            return Err(
                "tapping is written at a fixed rpm with feeds per minute (G97, G94)".into(),
            );
        }
        let left = match self.spindle_direction {
            Some(SpindleDirection::Ccw) => true,
            Some(SpindleDirection::Cw) => false,
            None => {
                return Err(
                    "tapping needs the running spindle direction (the thread hand) known to the post".into(),
                );
            }
        };
        let cycle = if left { "tap_cycle_left" } else { "tap_cycle" };
        if end[2] > start[2] {
            if d.declares(cycle) {
                self.emit(&[d.code("cycles_off")?])?;
            } else if d.parentheses_comments {
                self.comment("G33.1 returns to the start")?;
            } else {
                return Err(
                    "tapping return needs a block: declare tap_cycle (G84/G80) or comments".into(),
                );
            }
            return Ok(());
        }
        let rpm = self.rpm;
        if rpm == 0. {
            return Err("tapping needs the spindle running".into());
        }
        if !d.declares("tap_synchronized") && d.declares(cycle) {
            self.cycle_plane()?;
        }
        let feed = motion
            .feed_mm_min
            .ok_or("tapping needs its synchronized feed")?;
        let scale = self.machine.unit();
        if d.declares("tap_synchronized") {
            // Pitch per spindle revolution.
            let pitch = feed / rpm / scale;
            let k = format!("{pitch:.6}");
            let z = self.word(&d.axes[2], end[2], true)?;
            self.emit(&[
                d.code("tap_synchronized")?,
                &z,
                &format!("K{}", rstrip(rstrip(&k, '0'), '.')),
            ])
        } else if d.declares(cycle) && d.tap_pitch_word.is_some() {
            // Lead per spindle revolution, always with a decimal point.
            let lead = format!("{:.6}", feed / rpm / scale);
            let mut all = vec![d.code(cycle)?.to_string()];
            all.extend(self.words(
                &[&d.axes[0], &d.axes[1], &d.axes[2], "R"],
                &[end[0], end[1], end[2], start[2]],
                true,
            )?);
            all.push(format!(
                "{}{}",
                d.tap_pitch_word.as_deref().unwrap(),
                rstrip(&lead, '0')
            ));
            self.emit_owned(all)
        } else if d.declares(cycle) {
            let mut all = vec![d.code(cycle)?.to_string()];
            all.extend(self.words(
                &[&d.axes[0], &d.axes[1], &d.axes[2], "R", &d.feed_word],
                &[end[0], end[1], end[2], start[2], feed],
                true,
            )?);
            self.emit_owned(all)
        } else if left {
            Err("this machine dialect declares no left-hand tapping (tap_synchronized G33.1, or tap_cycle_left such as G74): a left-hand tap cannot be written for it".into())
        } else {
            Err(
                "this machine dialect declares no tapping cycle (tap_synchronized or tap_cycle)"
                    .into(),
            )
        }
    }

    /// Representable endpoint and incremental centre offsets in mm.
    fn circular_values(&self, curve: &Curve) -> Result<(Vec3, Vec3), String> {
        let position = self
            .position_mm
            .ok_or("arc without a decoded start position")?;
        let m = &self.machine;
        crate::post_rounding::circular_words(
            curve,
            position,
            m.unit(),
            m.decimals as usize,
            self.geometry_tolerance_mm,
            m.tolerance_mm,
            crate::post_rounding::RadiusTolerance {
                absolute_mm: m.dialect.radius_tolerance_mm,
                relative: m.dialect.radius_relative_tolerance,
            },
        )
        .map_err(|error| {
            format!(
                "{error}. Arc {:?} -> {:?} cannot be written as G2/G3 at {} decimals ({}). Use lowering with native_arcs=true to emit checked line segments for only such arcs, or a machine with decimals {} if the controller accepts that precision.",
                curve.start(),
                curve.end(),
                m.decimals,
                m.output_units,
                m.decimals + 1
            )
        })
    }

    /// Whether `circular_values` finds exact words for an arc from the
    /// current position (the lowering decision for native arcs).
    pub(crate) fn writes_arc(&self, curve: &Curve) -> bool {
        self.circular_values(curve).is_ok()
    }

    fn on_finish(&mut self) -> Result<(), String> {
        if let Some(Controller::Carvera(_)) = &self.machine.controller {
            if self.spindle_on {
                self.emit(&["M5"])?;
            }
            if self.coolant_mode != Coolant::Off {
                self.emit(&["M9"])?;
            }
            self.emit(&["G53 G0 Z-3"])?;
            return self.emit(&["M30"]);
        }
        let d = self.machine.dialect.clone();
        for effect in ["spindle_off", "coolant_off", "length_off", "end"] {
            // Omit a stop only when this post knows the state is already off.
            if effect == "spindle_off" && !self.spindle_on {
                continue;
            }
            if effect == "coolant_off" && self.coolant_mode == Coolant::Off {
                continue;
            }
            self.emit(&[d.code(effect)?])?;
        }
        match &self.machine.controller {
            Some(Controller::HaasNGC(_)) => {
                self.emit(&["%"])?;
            }
            None | Some(Controller::Carvera(_)) => {}
        }
        Ok(())
    }
}
