//! Makera Carvera nominal contract. Sources:
//! https://carvera-community.gitbook.io/docs/firmware/supported-commands
//! MakeraInc/CarveraProfiles, CAM_Post_Processors/Fusion360-profiles/Carvera.cps.
//! See docs/carvera-post.md for calibration, coordinates and retract premises.
use super::machine::Machine;
use crate::motion::Plane;
use serde::{Deserialize, Serialize};
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Carvera {
    pub initial_gauge_length_mm: f64,
    pub unmodified_coordinates: bool,
    pub automatic_tool_length: bool,
}
impl Carvera {
    pub fn validate(&self, m: &Machine) -> Result<(), String> {
        let require = |ok, message: &str| if ok { Ok(()) } else { Err(message.to_string()) };
        require(
            m.output_units == "mm",
            "Carvera requires mm: G20 halts the firmware",
        )?;
        require(
            self.unmodified_coordinates && self.automatic_tool_length,
            "Carvera requires unmodified coordinates and automatic M6 tool-length calibration",
        )?;
        require(
            self.initial_gauge_length_mm.is_finite()
                && (0. ..=1e6).contains(&self.initial_gauge_length_mm),
            "Carvera initial_gauge_length_mm must be finite and nonnegative",
        )?;
        require(
            m.work_to_machine.x == [1., 0., 0.]
                && m.work_to_machine.y == [0., 1., 0.]
                && m.work_to_machine.z == [0., 0., 1.],
            "Carvera work offsets require Cartesian translation only",
        )?;
        require(
            m.tool_change_position_mode == "table" && !m.flood,
            "Carvera requires declared table poses after M6 and no flood coolant",
        )?;
        require(
            m.planes == [Plane::Xy, Plane::Xz, Plane::Yz]
                && m.arcs
                && m.helices
                && m.max_sweep_rad <= std::f64::consts::PI,
            "Carvera requires native XYZ-plane arcs and helices of at most 180 degrees",
        )?;
        // Makera Carvera.cps: maximumCircularSweep=toRad(180); error6
        // refuses automatic tool slots beyond six (manual changes are separate).
        require(
            m.max_tool_number <= 6 && m.max_length_register <= 6,
            "Carvera reference tool and length slots must be 1..6",
        )?;
        let d = &m.dialect;
        let expected = [
            ("units_mm", "G21"),
            ("absolute", "G90"),
            ("plane_xy", "G17"),
            ("plane_xz", "G18"),
            ("plane_yz", "G19"),
            ("feed_per_minute", "G94"),
            ("rapid", "G0"),
            ("linear", "G1"),
            ("arc_cw", "G2"),
            ("arc_ccw", "G3"),
            ("spindle_cw", "M3"),
            ("spindle_off", "M5"),
            ("tool_change", "M6"),
            ("mist", "M7"),
            ("coolant_off", "M9"),
            ("end", "M30"),
            ("dwell", "G4"),
        ];
        require(
            d.commands.len() == expected.len()
                && expected
                    .iter()
                    .all(|(e, c)| d.code(e).is_ok_and(|v| v == *c)),
            "Carvera command mapping differs from the documented subset",
        )?;
        let fixed = [
            "center_incremental",
            "rpm_mode",
            "radius_off",
            "cycles_off",
            "frame_reset",
            "exact_stop",
        ];
        require(
            d.fixed_modes.len() == fixed.len()
                && fixed.iter().all(|e| d.fixed(e))
                && d.axes == ["X", "Y", "Z"]
                && d.centers == ["I", "J", "K"]
                && d.tool_word == "T"
                && d.length_word == "H"
                && d.feed_word == "F"
                && d.speed_word == "S"
                && d.tool_change == "m6"
                && d.length_activation == "explicit"
                && d.radius_tolerance_mm == 0.
                && d.work_offsets == ["G54", "G55", "G56", "G57", "G58", "G59"]
                && !d.x_diameter
                && !d.compensation_with_move
                && d.dwell_word == "P"
                && d.parentheses_comments
                && d.radius_relative_tolerance == 0.001
                && d.tap_pitch_word.is_none()
                && d.compensation_word == "D"
                && d.compensation_register
                && d.dwell_resolution_s == 0.001
                && d.dwell_max_s == 1e5
                && d.cycle_dwell_unit == "s"
                && d.peck_reentry_mm == Some(0.254)
                && d.chip_break_lift_mm == Some(0.254)
                && d.oriented_edge_deg.is_none()
                && d.beam_s_max.is_none()
                && d.beam_effect == "spindle_cw"
                && d.beam_mark.is_none()
                && d.thread_lead_word == "K"
                && d.height_control_word.is_none()
                && d.css_limit_word.is_none()
                && d.probing.is_none()
                && d.probe_fast_feed_mm_min.is_none()
                && !d.probe_switching,
            "Carvera dialect differs from the documented subset",
        )
    }
}
