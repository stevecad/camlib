//! Declared, zero-removal control state for opt-in stock manifest version2.
use super::{StreamManifest, point_valid};
use crate::motion::{Coolant, Event, Motion, SpindleDirection, Vec3};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControlCursor {
    pub count: u64,
    pub chain_sha256: [u8; 32],
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ToolSelection {
    pub tool: String,
    pub number: u32,
    pub length_register: u32,
    pub position_after_mm: Vec3,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControlState {
    pub cursor: ControlCursor,
    pub selected: Option<ToolSelection>,
    pub spindle_rpm: f64,
    pub spindle_direction: SpindleDirection,
    pub coolant: Coolant,
    pub work_offset: Option<u8>,
    /// No modeled/verified physical ATC route; not a motion-clear assessment.
    pub unverified_tool_routes: u64,
}

impl Default for ControlState {
    fn default() -> Self {
        Self {
            cursor: ControlCursor::default(),
            selected: None,
            spindle_rpm: 0.,
            spindle_direction: SpindleDirection::Cw,
            coolant: Coolant::Off,
            work_offset: None,
            unverified_tool_routes: 0,
        }
    }
}

impl ControlState {
    pub(super) fn validate(
        &self,
        manifest: &StreamManifest,
        completed: u64,
        position: Vec3,
    ) -> Result<(), String> {
        let bad_selection = self.selected.as_ref().is_some_and(|s| {
            s.number == 0
                || s.length_register == 0
                || !point_valid(s.position_after_mm)
                || !manifest.tools.iter().any(|t| t.id == s.tool)
        });
        if manifest.version != 2
            || !point_valid(position)
            || !self.spindle_rpm.is_finite()
            || self.spindle_rpm < 0.
            || bad_selection
            || self.work_offset.is_some_and(|n| !(1..=9).contains(&n))
            || self.unverified_tool_routes > self.cursor.count
            || self.selected.is_none() != (self.unverified_tool_routes == 0)
            || self.spindle_rpm > 0. && self.selected.is_none()
            || completed > 0 && (self.selected.is_none() || self.work_offset.is_none())
            || self.cursor.count == 0 && (self != &Self::default() || completed != 0)
            || completed == 0
                && position
                    != self
                        .selected
                        .as_ref()
                        .map_or(manifest.initial_position_mm, |s| s.position_after_mm)
        {
            return Err("invalid version 2 control checkpoint state".into());
        }
        Ok(())
    }

    pub(super) fn check_motion(&self, motion: &Motion) -> Result<(), String> {
        self.check_tool(&motion.tool, !motion.kind.cuts())
    }

    /// `noncutting`: a rapid, oriented shift or probing move, which needs no
    /// running spindle.
    pub(super) fn check_tool(&self, tool: &str, noncutting: bool) -> Result<(), String> {
        if self.selected.as_ref().map(|s| s.tool.as_str()) != Some(tool)
            || self.work_offset.is_none()
            || !noncutting && self.spindle_rpm <= 0.
        {
            return Err(
                "version 2 motion requires selected assembly, work offset and cutting spindle"
                    .into(),
            );
        }
        Ok(())
    }

    pub(super) fn transition(
        &self,
        event: &Event,
        manifest: &StreamManifest,
        completed: u64,
        before: Vec3,
    ) -> Result<(Self, Vec3), String> {
        let mut next = self.clone();
        let mut position = before;
        match event {
            Event::Motion { .. } => return Err("motion is not a control transfer".into()),
            Event::ToolChange {
                tool,
                number,
                length_register,
                position_after_mm,
            } => {
                if self.spindle_rpm != 0. {
                    return Err("stop spindle before a tool transfer".into());
                }
                next.selected = Some(ToolSelection {
                    tool: tool.clone(),
                    number: *number,
                    length_register: *length_register,
                    position_after_mm: *position_after_mm,
                });
                position = *position_after_mm;
                next.unverified_tool_routes = self
                    .unverified_tool_routes
                    .checked_add(1)
                    .ok_or("tool-route counter exhausted")?;
            }
            Event::Spindle { rpm, direction } => {
                next.spindle_rpm = *rpm;
                next.spindle_direction = *direction;
            }
            Event::SpindleOrient { .. } => next.spindle_rpm = 0.,
            Event::SurfaceSpeed {
                max_rpm, direction, ..
            } => {
                next.spindle_rpm = *max_rpm;
                next.spindle_direction = *direction;
            }
            Event::FeedPerRev { .. } => {}
            Event::Beam { .. } | Event::BeamMark { .. } | Event::HeightControl { .. } => {}
            Event::Coolant { mode } => next.coolant = *mode,
            Event::WorkOffset { number } => {
                if self.work_offset.is_some_and(|n| n != *number) {
                    return Err("use a separate stream for another work frame/offset".into());
                }
                next.work_offset = Some(*number);
            }
            Event::Stop { .. }
            | Event::Dwell { .. }
            | Event::Cycle { .. }
            | Event::Probe { .. }
            | Event::CutterCompensation { .. } => {}
        }
        next.cursor.count = self
            .cursor
            .count
            .checked_add(1)
            .ok_or("control cursor exhausted")?;
        let mut hash = Sha256::new();
        hash.update(self.cursor.chain_sha256);
        hash.update(completed.to_le_bytes());
        for value in before {
            hash.update(value.to_le_bytes());
        }
        hash.update(serde_json::to_vec(event).map_err(|e| e.to_string())?);
        next.cursor.chain_sha256 = hash.finalize().into();
        next.validate(manifest, completed, position)?;
        Ok((next, position))
    }
}
