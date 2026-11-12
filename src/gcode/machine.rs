//! The declared machine and controller dialect as JSON: exactly the form
//! `dataclasses.asdict(camlib.post.Machine)` gives (`Machine.to_dict()`),
//! which a posted run also records as its `definition.machine`. The #116
//! machine crash model is its `crash_model`. Every field is required, so a
//! JSON machine states everything the Python one does; only the optional
//! settings a posted definition leaves out when unset may be left out
//! (`max_normal_accel_mm_s2`, `max_axis_feed_mm_min`, `motion_policy`,
//! `schedule_post_feeds`, `crash_model`). The checks are those of the Python
//! dataclasses, with the same messages.
use crate::motion::{Capabilities, Plane, Vec3, WorkFrame};
use serde::{Deserialize, Deserializer, Serialize};

/// A present JSON value, which may be null (serde would otherwise accept a
/// missing `Option` field as None).
fn present<'de, D: Deserializer<'de>, T: Deserialize<'de>>(d: D) -> Result<Option<T>, D::Error> {
    Option::<T>::deserialize(d)
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Command {
    pub effect: String,
    pub code: String,
}

/// `camlib.post.GCodeDialect`: declared word semantics.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Dialect {
    pub commands: Vec<Command>,
    pub work_offsets: Vec<String>,
    pub axes: Vec<String>,
    pub centers: Vec<String>,
    pub tool_word: String,
    pub length_word: String,
    pub feed_word: String,
    pub speed_word: String,
    pub parentheses_comments: bool,
    pub tool_change: String,
    pub length_activation: String,
    pub radius_tolerance_mm: f64,
    pub radius_relative_tolerance: f64,
    pub fixed_modes: Vec<String>,
    #[serde(deserialize_with = "present")]
    pub tap_pitch_word: Option<String>,
    pub compensation_word: String,
    pub compensation_with_move: bool,
    pub compensation_register: bool,
    pub dwell_word: String,
    pub dwell_resolution_s: f64,
    pub dwell_max_s: f64,
    pub cycle_dwell_unit: String,
    #[serde(deserialize_with = "present")]
    pub peck_reentry_mm: Option<f64>,
    #[serde(deserialize_with = "present")]
    pub chip_break_lift_mm: Option<f64>,
    #[serde(deserialize_with = "present")]
    pub oriented_edge_deg: Option<f64>,
    #[serde(deserialize_with = "present")]
    pub beam_s_max: Option<f64>,
    pub beam_effect: String,
    /// How this machine marks with its beam (`ThermalMark`), declared by
    /// its owner: `'power'` (the beam word at the marking power, as GRBL
    /// laser mode scales S), `'mode'` (the controller's process selection
    /// codes `mark_mode` and `cut_mode`, written with the beam off before
    /// the beam word) or none (marking is refused). Left out when unset.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub beam_mark: Option<String>,
    pub x_diameter: bool,
    pub thread_lead_word: String,
    #[serde(deserialize_with = "present")]
    pub height_control_word: Option<String>,
    #[serde(deserialize_with = "present")]
    pub css_limit_word: Option<String>,
    /// The probing back end (`'g38'`, `'inspection_plus'`) or none. This post
    /// writes no probing cycles (they are Python posts); a declared back end
    /// is validated and recorded, and a probing cycle is refused.
    #[serde(deserialize_with = "present")]
    pub probing: Option<String>,
    #[serde(deserialize_with = "present")]
    pub probe_fast_feed_mm_min: Option<f64>,
    pub probe_switching: bool,
}

/// One entry of the machine tool table (`camlib.post.ToolSetting`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ToolSetting {
    pub tool: String,
    pub number: u32,
    pub length_register: u32,
    pub gauge_length_mm: f64,
    /// The tip after this tool's change, in work coordinates. Required by
    /// `tool_change_position_mode = "table"`; the other modes do not read it.
    #[serde(default)]
    pub position_after_mm: Option<Vec3>,
    #[serde(deserialize_with = "present")]
    pub diameter_register: Option<u32>,
}

/// `camlib.post.Machine`: a real machine as its owner declares it.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Machine {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub controller: Option<super::controller::Controller>,
    pub name: String,
    pub tools: Vec<ToolSetting>,
    pub dialect: Dialect,
    pub work_offset: u32,
    pub frame: WorkFrame,
    pub work_to_machine: WorkFrame,
    #[serde(deserialize_with = "present")]
    pub tip_travel_mm: Option<[Vec3; 2]>,
    /// The spindle gauge line's travel in machine coordinates (the axis soft
    /// limits); each tool's tip travel follows from its gauge length. Not
    /// with `tip_travel_mm`, and not on a lathe.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gauge_travel_mm: Option<[Vec3; 2]>,
    pub output_units: String,
    pub decimals: u32,
    pub tolerance_mm: f64,
    pub feed_tolerance_mm_min: f64,
    #[serde(deserialize_with = "present")]
    pub traverse_feed_mm_min: Option<f64>,
    pub max_feed_mm_min: f64,
    pub max_spindle_rpm: f64,
    pub planes: Vec<Plane>,
    pub arcs: bool,
    pub helices: bool,
    pub max_sweep_rad: f64,
    pub max_tool_number: u32,
    pub max_length_register: u32,
    pub tool_changes: bool,
    pub flood: bool,
    pub mist: bool,
    pub revision: String,
    pub tool_change_position_mode: String,
    #[serde(deserialize_with = "present")]
    pub tool_change_home_z_mm: Option<f64>,
    pub rapid_mm_min: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_normal_accel_mm_s2: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_axis_feed_mm_min: Option<Vec3>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub motion_policy: Option<serde_json::Value>,
    #[serde(default, skip_serializing_if = "is_false")]
    pub schedule_post_feeds: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub crash_model: Option<serde_json::Value>,
}
fn is_false(v: &bool) -> bool {
    !*v
}

/// Effects with a default code (`_DEFAULT_COMMANDS`).
pub(crate) const DEFAULT_EFFECTS: [&str; 41] = [
    "rapid",
    "linear",
    "arc_cw",
    "arc_ccw",
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
    "radius_left",
    "radius_right",
    "length_off",
    "cycles_off",
    "frame_reset",
    "exact_stop",
    "length_on",
    "spindle_cw",
    "spindle_ccw",
    "spindle_off",
    "tool_change",
    "mist",
    "flood",
    "coolant_off",
    "stop",
    "optional_stop",
    "end",
    "dwell",
    "drill_cycle",
    "drill_dwell_cycle",
    "peck_cycle",
    "chip_break_cycle",
    "cycle_return_initial",
    "bore_cycle",
    "bore_dwell_cycle",
    "bore_stop_cycle",
    "tap_synchronized",
];
pub(crate) const CYCLE_EFFECTS: [&str; 9] = [
    "tap_cycle",
    "tap_cycle_left",
    "fine_bore_cycle",
    "spindle_orient",
    "height_control_on",
    "height_control_off",
    "thread_synchronized",
    "mark_mode",
    "cut_mode",
];
pub(crate) const LATHE_EFFECTS: [&str; 3] = ["css_on", "css_limit", "feed_per_rev"];
pub(crate) const OPTIONAL_EFFECTS: [&str; 4] = [
    "language_native",
    "compression_off",
    "lathe_diameter_mode",
    "lathe_radius_mode",
];
/// Modal groups that every program initializes explicitly (`_REQUIRED`).
pub(crate) const REQUIRED: [&str; 13] = [
    "units",
    "absolute",
    "center_incremental",
    "plane",
    "feed_per_minute",
    "rpm_mode",
    "radius_off",
    "length",
    "cycles_off",
    "frame_reset",
    "exact_stop",
    "spindle_off",
    "coolant_off",
];

/// One uppercase ASCII letter.
pub(crate) fn letter(s: &str) -> bool {
    s.len() == 1 && s.as_bytes()[0].is_ascii_uppercase()
}

/// A command code's identity: `(letter, number)` for numeric codes (G0 and
/// G00 are the same code), the keyword otherwise.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum Code {
    Word(u8, f64),
    Keyword(String),
}
pub(crate) fn code_token(code: &str) -> Result<Code, String> {
    let b = code.as_bytes();
    if b.len() > 1 && b[1].is_ascii_digit() {
        let value: f64 = code[1..]
            .parse()
            .map_err(|_| format!("invalid command code {code}"))?;
        Ok(Code::Word(b[0], value))
    } else {
        Ok(Code::Keyword(code.to_string()))
    }
}
/// `G<digits>[.<digits>]` (or M), or a standalone keyword of A-Z and `_`.
fn command_syntax(code: &str) -> bool {
    let b = code.as_bytes();
    let numeric = || {
        if b.len() < 2 || !matches!(b[0], b'G' | b'M') {
            return false;
        }
        let rest = &code[1..];
        let (int, frac) = match rest.split_once('.') {
            Some((i, f)) => (i, Some(f)),
            None => (rest, None),
        };
        !int.is_empty()
            && int.bytes().all(|c| c.is_ascii_digit())
            && frac.is_none_or(|f| !f.is_empty() && f.bytes().all(|c| c.is_ascii_digit()))
    };
    numeric() || (!b.is_empty() && b.iter().all(|c| c.is_ascii_uppercase() || *c == b'_'))
}
fn work_offset_syntax(code: &str) -> bool {
    let b = code.as_bytes();
    if b.len() < 2 || b[0] != b'G' {
        return false;
    }
    let rest = &code[1..];
    let (int, frac) = match rest.split_once('.') {
        Some((i, f)) => (i, Some(f)),
        None => (rest, None),
    };
    !int.is_empty()
        && int.bytes().all(|c| c.is_ascii_digit())
        && frac.is_none_or(|f| !f.is_empty() && f.bytes().all(|c| c.is_ascii_digit()))
}

impl Dialect {
    /// The checks of `GCodeDialect.__post_init__`.
    pub fn validate(&self) -> Result<(), String> {
        for list in [
            self.commands.len(),
            self.work_offsets.len(),
            self.axes.len(),
            self.centers.len(),
            self.fixed_modes.len(),
        ] {
            if list > 128 {
                return Err("dialect definition cap".into());
            }
        }
        if self.axes.len() != 3
            || self.centers.len() != 3
            || !(1..=9).contains(&self.work_offsets.len())
        {
            return Err("three Cartesian axes and 1..9 work offsets required".into());
        }
        let letters: Vec<&str> = self
            .axes
            .iter()
            .chain(&self.centers)
            .map(String::as_str)
            .chain([
                self.tool_word.as_str(),
                self.length_word.as_str(),
                self.feed_word.as_str(),
                self.speed_word.as_str(),
            ])
            .collect();
        let distinct: std::collections::BTreeSet<_> = letters.iter().collect();
        if distinct.len() != letters.len()
            || letters.iter().any(|v| !letter(v) || "GMN".contains(*v))
        {
            return Err("dialect word addresses must be distinct uppercase letters".into());
        }
        let taken = |w: &str, extra: &[&str]| letters.contains(&w) || extra.contains(&w);
        if let Some(w) = &self.tap_pitch_word
            && (!letter(w) || taken(w, &["G", "M", "N", "R"]))
        {
            return Err("tap_pitch_word must be a free uppercase letter".into());
        }
        if !letter(&self.dwell_word)
            || taken(&self.dwell_word, &["G", "M", "N"])
            || !(self.dwell_resolution_s.is_finite()
                && 0. < self.dwell_resolution_s
                && self.dwell_resolution_s <= 1.)
            || !(self.dwell_max_s.is_finite()
                && self.dwell_resolution_s <= self.dwell_max_s
                && self.dwell_max_s <= 1e5)
        {
            return Err("invalid declared dwell word/resolution/maximum".into());
        }
        let lead = self.thread_lead_word.as_str();
        if !letter(lead)
            || self.axes.iter().any(|a| a == lead)
            || [
                self.tool_word.as_str(),
                self.length_word.as_str(),
                self.speed_word.as_str(),
                "G",
                "M",
                "N",
            ]
            .contains(&lead)
        {
            return Err(
                "thread_lead_word must be an uppercase letter that is not an axis, tool, length or speed word".into(),
            );
        }
        if !["s", "ms"].contains(&self.cycle_dwell_unit.as_str()) {
            return Err("cycle_dwell_unit must be 's' or 'ms'".into());
        }
        for value in [self.peck_reentry_mm, self.chip_break_lift_mm]
            .into_iter()
            .flatten()
        {
            if !(value.is_finite() && 0. < value && value <= 100.) {
                return Err("canned cycle return amounts must be positive millimetres".into());
            }
        }
        if let Some(s) = self.beam_s_max
            && !(s.is_finite() && 0. < s && s <= 1e6)
        {
            return Err("beam_s_max must be a positive S value".into());
        }
        if !["spindle_cw", "spindle_ccw"].contains(&self.beam_effect.as_str()) {
            return Err("beam_effect must be 'spindle_cw' or 'spindle_ccw'".into());
        }
        match self.beam_mark.as_deref() {
            None | Some("power") => {
                if self.declares("mark_mode") || self.declares("cut_mode") {
                    return Err(
                        "mark_mode and cut_mode are the process codes of beam_mark='mode'".into(),
                    );
                }
            }
            Some("mode") => {
                if !self.declares("mark_mode") || !self.declares("cut_mode") {
                    return Err(
                        "beam_mark='mode' needs the controller's process codes: with_commands(mark_mode=..., cut_mode=...)".into(),
                    );
                }
            }
            Some(_) => {
                return Err(
                    "beam_mark must be None, 'power' (the beam word at the marking power) or 'mode' (declared mark_mode and cut_mode codes)".into(),
                );
            }
        }
        if self.beam_mark.is_some() && self.beam_s_max.is_none() {
            return Err("beam_mark needs a declared beam (beam_s_max)".into());
        }
        if let Some(w) = &self.height_control_word {
            let b = w.as_bytes();
            if b.len() < 2 || !b[0].is_ascii_uppercase() || !b[1..].iter().all(u8::is_ascii_digit) {
                return Err(
                    "height_control_word must be one address with an integer, e.g. P2".into(),
                );
            }
        }
        if let Some(e) = self.oriented_edge_deg
            && !(e.is_finite() && e.abs() <= 360.)
        {
            return Err("oriented_edge_deg must be finite, within +-360 degrees".into());
        }
        if (self.declares("spindle_orient") || self.declares("fine_bore_cycle"))
            && self.oriented_edge_deg.is_none()
        {
            return Err("declare oriented_edge_deg (where the bar's edge points after the oriented stop) with spindle_orient/fine_bore_cycle".into());
        }
        if self.declares("peck_cycle") && self.peck_reentry_mm.is_none()
            || self.declares("chip_break_cycle") && self.chip_break_lift_mm.is_none()
        {
            return Err(
                "declare the controller return amount (peck_reentry_mm/chip_break_lift_mm) of each pecking cycle"
                    .into(),
            );
        }
        if !["m6", "direct_t", "lathe_t"].contains(&self.tool_change.as_str())
            || !["explicit", "tool_edge"].contains(&self.length_activation.as_str())
        {
            return Err("unsupported declared tool/length policy".into());
        }
        if !self.radius_tolerance_mm.is_finite()
            || !(0. ..=0.005).contains(&self.radius_tolerance_mm)
        {
            return Err("invalid declared circle radius tolerance".into());
        }
        if !self.radius_relative_tolerance.is_finite()
            || !(0. ..=0.001).contains(&self.radius_relative_tolerance)
        {
            return Err("invalid declared relative circle radius tolerance".into());
        }
        if self.declares("css_on") {
            if self.declares("css_limit") == self.css_limit_word.is_some() {
                return Err("declare the constant surface speed rpm limit one way: css_limit (G50 S, own block) or css_limit_word (in the G96 block), not both".into());
            }
            if !self.declares("rpm_mode") || self.fixed("rpm_mode") {
                return Err("constant surface speed needs the rpm mode (G97) to cancel it".into());
            }
        }
        // `GCodeDialect.__post_init__`'s probing rules.
        if !matches!(
            self.probing.as_deref(),
            None | Some("g38") | Some("inspection_plus")
        ) {
            return Err(
                "probing must be None, 'g38' (LinuxCNC) or 'inspection_plus' (Renishaw macros)"
                    .into(),
            );
        }
        let macros = self.probing.as_deref() == Some("inspection_plus");
        if macros != self.probe_fast_feed_mm_min.is_some() || (self.probe_switching && !macros) {
            return Err("declare probe_fast_feed_mm_min (#119) and probe_switching exactly with inspection_plus probing".into());
        }
        if let Some(f) = self.probe_fast_feed_mm_min
            && !(f.is_finite() && 0. < f && f <= 1e5)
        {
            return Err("probe_fast_feed_mm_min must be a positive feed".into());
        }
        if let Some(w) = &self.css_limit_word
            && (!letter(w) || taken(w, &["G", "M", "N"]))
        {
            return Err("css_limit_word must be a free uppercase letter".into());
        }
        if self.declares("feed_per_rev")
            && (!self.declares("feed_per_minute") || self.fixed("feed_per_minute"))
        {
            return Err(
                "feed per revolution needs the feed per minute mode (G94/G98) to return to".into(),
            );
        }
        let fixable = [
            "absolute",
            "center_incremental",
            "feed_per_minute",
            "rpm_mode",
            "radius_off",
            "cycles_off",
            "frame_reset",
            "exact_stop",
        ];
        if self
            .fixed_modes
            .iter()
            .any(|m| !fixable.contains(&m.as_str()))
        {
            return Err("only invariant modal semantics may be declared fixed".into());
        }
        let mut codes = Vec::new();
        let commands = self
            .commands
            .iter()
            .map(|c| (c.effect.as_str(), c.code.as_str()))
            .chain(
                self.work_offsets
                    .iter()
                    .map(|c| ("work_offset", c.as_str())),
            );
        for (effect, code) in commands {
            let known = DEFAULT_EFFECTS.contains(&effect)
                || OPTIONAL_EFFECTS.contains(&effect)
                || CYCLE_EFFECTS.contains(&effect)
                || LATHE_EFFECTS.contains(&effect)
                || effect == "work_offset";
            if !known {
                return Err("unknown command effect".into());
            }
            if !command_syntax(code)
                && !(effect == "length_off" && code == format!("{}0", self.length_word))
            {
                return Err("command must be a numeric G/M code or standalone keyword".into());
            }
            let token = code_token(code)?;
            if codes.contains(&token) {
                return Err("duplicate dialect command code".into());
            }
            codes.push(token);
        }
        if self.work_offsets.iter().any(|s| !work_offset_syntax(s)) {
            return Err("work offsets require numeric G codes".into());
        }
        Ok(())
    }
    pub fn declares(&self, effect: &str) -> bool {
        self.commands.iter().any(|c| c.effect == effect)
    }
    pub fn fixed(&self, effect: &str) -> bool {
        self.fixed_modes.iter().any(|m| m == effect)
    }
    /// The declared code of an effect (`GCodeDialect.code`).
    pub fn code(&self, effect: &str) -> Result<&str, String> {
        self.commands
            .iter()
            .find(|c| c.effect == effect)
            .map(|c| c.code.as_str())
            .ok_or_else(|| format!("dialect does not declare {effect}"))
    }
    pub fn compensation_word_free(&self) -> bool {
        let w = self.compensation_word.as_str();
        letter(w)
            && !self.axes.iter().chain(&self.centers).any(|a| a == w)
            && ![
                self.tool_word.as_str(),
                self.length_word.as_str(),
                self.feed_word.as_str(),
                self.speed_word.as_str(),
                "G",
                "M",
                "N",
            ]
            .contains(&w)
    }
    /// Whether the controller accepts arc words whose start and end radii
    /// are r1 and r2 with mismatch `error` (`post_decode::arc_radii`;
    /// LinuxCNC's rule, `GCodeDialect.radius_accepts`).
    pub fn radius_accepts(&self, r1: f64, r2: f64, error: f64) -> bool {
        error <= self.radius_tolerance_mm
            || error <= 100. * self.radius_tolerance_mm
                && error <= self.radius_relative_tolerance * r1.max(r2)
    }
}

fn frame_valid(f: &WorkFrame) -> Result<(), String> {
    f.validate()
}

impl Machine {
    /// Parse and check a JSON machine.
    pub fn from_json(value: serde_json::Value) -> Result<Self, String> {
        let machine: Machine =
            serde_json::from_value(value).map_err(|e| format!("machine: {e}"))?;
        machine.validate()?;
        Ok(machine)
    }
    /// The checks of `Machine.__post_init__` (and the dialect's).
    pub fn validate(&self) -> Result<(), String> {
        self.dialect.validate()?;
        if let Some(controller) = &self.controller {
            controller.validate(self)?;
        }
        if self.schedule_post_feeds && self.motion_policy.is_none() {
            return Err("schedule_post_feeds requires an explicit motion_policy".into());
        }
        if (0..self.planes.len()).any(|i| self.planes[i + 1..].contains(&self.planes[i])) {
            return Err("machine planes are distinct values from 'xy', 'xz', 'yz'".into());
        }
        if self.name.is_empty()
            || self.name.chars().count() > 128
            || self.revision.is_empty()
            || self.revision.chars().count() > 128
        {
            return Err("machine needs bounded name/revision".into());
        }
        if self.tools.is_empty() {
            return Err("machine needs at least one tool and a declared dialect".into());
        }
        if !["table", "preserve_tip", "preserve_spindle", "z_home"]
            .contains(&self.tool_change_position_mode.as_str())
        {
            return Err(
                "tool_change_position_mode must be table, preserve_tip, preserve_spindle or z_home"
                    .into(),
            );
        }
        if self.tool_change_position_mode == "preserve_spindle" {
            // The tip moves along the spindle axis by the difference of the
            // gauge lengths: the tool axis must be work Z, and the tool must
            // be changed by a spindle tool change (not a lathe turret index).
            if self.work_to_machine.z != [0., 0., 1.] {
                return Err("tool_change_position_mode preserve_spindle needs the work Z axis along machine Z".into());
            }
            if self.dialect.tool_change == "lathe_t" {
                return Err("tool_change_position_mode preserve_spindle is a spindle tool change; a lathe turret declares each tool's position (table)".into());
            }
            if self.dialect.length_activation == "tool_edge" {
                return Err("tool_change_position_mode preserve_spindle is not modelled for cutting-edge (T/D) length selection; declare each tool's position (table)".into());
            }
        }
        if self.tool_change_position_mode == "table"
            && let Some(t) = self.tools.iter().find(|t| t.position_after_mm.is_none())
        {
            return Err(format!(
                "tool_change_position_mode \"table\" needs each ToolSetting.position_after_mm (the tip after that tool's change), and tool {:?} has none: state it, or declare where the tool change leaves the spindle (z_home with tool_change_home_z_mm)",
                t.tool
            ));
        }
        if (self.tool_change_position_mode == "z_home") != self.tool_change_home_z_mm.is_some()
            || self
                .tool_change_home_z_mm
                .is_some_and(|z| !(z.is_finite() && z.abs() <= 1e6))
        {
            return Err("z_home tool changes need tool_change_home_z_mm (the gauge line's machine Z after M6), and only they".into());
        }
        if self.gauge_travel_mm.is_some() {
            if self.tip_travel_mm.is_some() {
                return Err("machine travel is declared once: tip_travel_mm (the tip box of one tool length) or gauge_travel_mm (the spindle gauge line's box, checked per tool)".into());
            }
            if self.work_to_machine.z != [0., 0., 1.] {
                return Err("gauge_travel_mm needs a work frame aligned with machine Z".into());
            }
            if self.dialect.tool_change == "lathe_t" || self.dialect.x_diameter {
                return Err("gauge_travel_mm is a milling spindle's gauge-line travel; a lathe declares tip_travel_mm".into());
            }
        }
        if self.tool_change_position_mode == "z_home" && self.work_to_machine.z != [0., 0., 1.] {
            return Err("z_home tool changes need a work frame aligned with machine Z".into());
        }
        if !self.rapid_mm_min.is_finite() || !(0. < self.rapid_mm_min && self.rapid_mm_min <= 1e6) {
            return Err("rapid_mm_min must be the positive, finite rapid traverse rate".into());
        }
        if !["mm", "inch"].contains(&self.output_units.as_str())
            || !(2..=9).contains(&self.decimals)
        {
            return Err("invalid machine output units/precision".into());
        }
        for (name, value) in [
            ("tolerance_mm", self.tolerance_mm),
            ("feed_tolerance_mm_min", self.feed_tolerance_mm_min),
        ] {
            if !value.is_finite() || !(1e-9..=0.1).contains(&value) {
                return Err(format!("{name} must be finite and in [1e-9, .1]"));
            }
        }
        if let Some(f) = self.traverse_feed_mm_min
            && (!f.is_finite() || !(0. < f && f <= self.max_feed_mm_min))
        {
            return Err(
                "traverse_feed_mm_min must be positive, finite and within machine feed limits"
                    .into(),
            );
        }
        if let Some(a) = self.max_normal_accel_mm_s2
            && (!a.is_finite() || !(1e-6..=1e9).contains(&a))
        {
            return Err("max_normal_accel_mm_s2 must be finite and in [1e-6, 1e9]".into());
        }
        if let Some(limits) = self.max_axis_feed_mm_min
            && limits
                .iter()
                .any(|v| !v.is_finite() || !(1e-6..=1e7).contains(v))
        {
            return Err(
                "max_axis_feed_mm_min requires three finite positive axis limits in [1e-6, 1e7]"
                    .into(),
            );
        }
        if 3f64.sqrt() * 0.5 * self.unit() * 10f64.powi(-(self.decimals as i32)) > self.tolerance_mm
        {
            return Err(
                "coordinate rounding exceeds tolerance_mm; increase decimals or tolerance".into(),
            );
        }
        if !(1..=self.dialect.work_offsets.len() as u32).contains(&self.work_offset) {
            return Err("work offset absent from dialect".into());
        }
        let ids: std::collections::BTreeSet<_> = self.tools.iter().map(|t| &t.tool).collect();
        if ids.len() != self.tools.len() {
            return Err("machine tool identities must be unique".into());
        }
        let edge = self.dialect.length_activation == "tool_edge";
        let keys: std::collections::BTreeSet<_> = self
            .tools
            .iter()
            .map(|t| (t.number, if edge { t.length_register } else { 0 }))
            .collect();
        if keys.len() != self.tools.len() {
            return Err("machine tool numbers (or tool/edge pairs) must be unique".into());
        }
        if !edge {
            let registers: std::collections::BTreeSet<_> =
                self.tools.iter().map(|t| t.length_register).collect();
            if registers.len() != self.tools.len() {
                return Err(
                    "global length registers must be unique (tool_edge registers are per tool)"
                        .into(),
                );
            }
        }
        for t in &self.tools {
            if !(1..=self.max_tool_number).contains(&t.number)
                || !(1..=self.max_length_register).contains(&t.length_register)
            {
                return Err("tool/length number exceeds machine limits".into());
            }
        }
        frame_valid(&self.frame)?;
        frame_valid(&self.work_to_machine)?;
        Ok(())
    }
    /// Millimetres per output unit.
    pub fn unit(&self) -> f64 {
        if self.output_units == "inch" {
            25.4
        } else {
            1.
        }
    }
    /// The engine/post contract (`Machine.capabilities()`).
    pub fn capabilities(&self) -> Capabilities {
        Capabilities {
            id: "declared_machine".into(),
            planes: self.planes.clone(),
            arcs: self.arcs,
            helices: self.helices,
            max_sweep_rad: self.max_sweep_rad,
            max_feed_mm_min: self.max_feed_mm_min,
            max_spindle_rpm: self.max_spindle_rpm,
            max_tool_number: self.max_tool_number,
            max_length_register: self.max_length_register,
            max_work_offset: self.dialect.work_offsets.len() as u8,
            tool_changes: self.tool_changes,
            flood: self.flood,
            mist: self.mist,
        }
    }
}
