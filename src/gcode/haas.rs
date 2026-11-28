//! Named controller contract. Haas sources: 96-8210 Rev N (December 2020),
//! sections 5.8, G04/G73/G83/G84/G85/G89 and Settings 15/22/33/52/77/162.
//! See docs/haas-ngc-post.md for page references and external setup premises.
use super::machine::Machine;
use crate::motion::Plane;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HaasNGC {
    pub coordinate_system: String,
    pub h_t_agreement: bool,
    pub g92_shift_mm: [f64; 3],
    pub rotation_scaling: bool,
    pub program_number: u32,
    pub optional_stop_enabled: bool,
    pub mirror_settings_xyz: [bool; 3],
    pub curve_policy: String,
    pub rigid_tapping: bool,
    pub cutter_compensation: bool,
    #[serde(deserialize_with = "required_option")]
    pub setting_22_mm: Option<f64>,
    #[serde(deserialize_with = "required_option")]
    pub setting_52_mm: Option<f64>,
    pub reference: String,
}
fn required_option<'de, D: serde::Deserializer<'de>>(d: D) -> Result<Option<f64>, D::Error> {
    Option::deserialize(d)
}
impl HaasNGC {
    pub fn validate(&self, m: &Machine) -> Result<(), String> {
        let h = self;
        let d = &m.dialect;
        let require = |ok: bool, message: &str| if ok { Ok(()) } else { Err(message.to_string()) };
        require(
            ["fanuc", "haas"].contains(&h.coordinate_system.as_str()),
            "declare Setting33 fanuc/haas and boolean machine options",
        )?;
        require(
            h.g92_shift_mm == [0.; 3] && h.mirror_settings_xyz == [false; 3],
            "reference requires declared zero G92 and mirror Settings45..47 OFF",
        )?;
        require(
            (1..=99999).contains(&h.program_number),
            "reference O program number must be 1..99999",
        )?;
        require(
            m.traverse_feed_mm_min.is_some(),
            "declare positive traverse_feed_mm_min; G0 dogleg is not this path model",
        )?;
        require(
            ["linear", "exact_ijk"].contains(&h.curve_policy.as_str()) && m.planes == [Plane::Xy],
            "reference uses G17 and linear/exact_ijk curves",
        )?;
        require(
            m.arcs == (h.curve_policy == "exact_ijk")
                && m.helices == m.arcs
                && m.max_sweep_rad <= std::f64::consts::TAU,
            "reference curve capabilities must match single-turn policy",
        )?;
        require(
            m.decimals <= if m.output_units == "mm" { 3 } else { 4 },
            "reference coordinate precision is at most 3 metric/4 inch decimals",
        )?;
        require(
            m.max_tool_number <= 200 && m.max_length_register <= 200,
            "reference physical T/H table subset is 1..200",
        )?;
        require(
            !h.h_t_agreement || m.tools.iter().all(|t| t.number == t.length_register),
            "Setting15 ON requires matching T/H",
        )?;
        require(
            m.work_to_machine.x == [1., 0., 0.]
                && m.work_to_machine.y == [0., 1., 0.]
                && m.work_to_machine.z == [0., 0., 1.],
            "reference stored work offsets require Cartesian translation only",
        )?;
        // Numeric safety only: finite and nonnegative; the installed Setting 22 /
        // Setting 52 values are the owner's declaration (no upper range is cited
        // from Haas 96-8210 Rev N).
        for (name, value) in [
            ("setting_22_mm", h.setting_22_mm),
            ("setting_52_mm", h.setting_52_mm),
        ] {
            require(
                value.is_none_or(|v| v.is_finite() && v >= 0.),
                &format!("{name} must be a finite, nonnegative distance in mm"),
            )?;
        }
        require(
            h.setting_22_mm != Some(0.),
            "setting_22_mm (Setting 22, Can Cycle Delta Z) must be positive",
        )?;
        require(
            d.peck_reentry_mm == h.setting_22_mm && d.chip_break_lift_mm == h.setting_22_mm,
            "G73/G83 return amounts must be Setting 22 (setting_22_mm)",
        )?;
        require(
            d.compensation_with_move == h.cutter_compensation,
            "Haas starts and cancels cutter compensation in the block of its XY move",
        )?;
        let mut expected = vec![
            ("units_mm", "G21"),
            ("units_inch", "G20"),
            ("absolute", "G90"),
            ("plane_xy", "G17"),
            ("feed_per_minute", "G94"),
            ("radius_off", "G40"),
            ("length_off", "G49"),
            ("length_on", "G43"),
            ("cycles_off", "G80"),
            ("frame_reset", "G52"),
            ("exact_stop", "G61"),
            ("linear", "G1"),
            ("arc_cw", "G2"),
            ("arc_ccw", "G3"),
            ("spindle_cw", "M3"),
            ("spindle_ccw", "M4"),
            ("spindle_off", "M5"),
            ("tool_change", "M6"),
            ("flood", "M8"),
            ("coolant_off", "M9"),
            ("stop", "M0"),
            ("optional_stop", "M1"),
            ("end", "M30"),
            ("dwell", "G4"),
            ("drill_cycle", "G81"),
            ("drill_dwell_cycle", "G82"),
            ("cycle_return_initial", "G98"),
            ("bore_cycle", "G85"),
            ("bore_dwell_cycle", "G89"),
        ];
        if h.rigid_tapping {
            expected.extend([("tap_cycle", "G84"), ("tap_cycle_left", "G74")]);
        }
        if h.cutter_compensation {
            expected.extend([("radius_left", "G41"), ("radius_right", "G42")]);
        }
        if h.setting_22_mm.is_some() {
            expected.push(("chip_break_cycle", "G73"));
            if h.setting_52_mm == Some(0.) {
                expected.push(("peck_cycle", "G83"));
            }
        }
        require(
            d.commands.len() == expected.len()
                && expected
                    .iter()
                    .all(|(e, c)| d.code(e).is_ok_and(|v| v == *c)),
            "reference command mapping differs from documented Haas subset",
        )?;
        require(
            d.axes == ["X", "Y", "Z"]
                && d.centers == ["I", "J", "K"]
                && d.tool_word == "T"
                && d.length_word == "H"
                && d.feed_word == "F"
                && d.speed_word == "S",
            "reference uses XYZ/IJK/T/H/F/S",
        )?;
        require(
            d.tool_change == "m6"
                && d.length_activation == "explicit"
                && d.radius_tolerance_mm == 0.
                && !m.mist,
            "reference uses explicit M6/G43/H, exact-IJK and flood/off (M7 is shower)",
        )?;
        require(
            d.fixed_modes.len() == 2
                && d.fixed("center_incremental")
                && d.fixed("rpm_mode")
                && d.parentheses_comments,
            "reference fixed modes are incremental IJK and RPM only",
        )?;
        require(
            d.work_offsets == ["G54", "G55", "G56", "G57", "G58", "G59"],
            "reference supports six stored work offsets",
        )
    }
}
