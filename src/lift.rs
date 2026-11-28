//! Straight-up lifts out of a cut (#430, #441).
//!
//! With `rapid_retracts`, the stream turns a planned straight-up retract into
//! a rapid only when the whole-cell stock bounds alone show it clear
//! (`StreamManifest::retract_policy`, #430). Beside stock the cutter has just
//! cut, they do not: the cells along a wall keep the wall's height, so the
//! whole lift stayed at feed, up to the feed height.
//!
//! A planner that leaves a cut straight up therefore splits that retract
//! (#441). It feeds only until the tool's lowest point is at the highest
//! whole-cell bound within reach of any part of the tool, and lifts from
//! there to where the retract was planned to end. That second retract starts
//! above every cell within reach, so the run's retract policy makes it a
//! rapid, and the machine check decides it by its floor rule
//! (`machine::Session::floor_rise`, #412). No check changes.
use crate::{
    motion::{Curve, MoveKind, Vec3},
    stock::Stock,
    tool::ToolAssembly,
};

/// How a run's straight-up retracts of one tool are judged and split.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Lifts {
    tool: String,
    /// Per tool section (`verification::parts`): its widest radius and the
    /// height of its lowest point above the tip.
    sections: Vec<(f64, f64)>,
    /// The run's sweep tolerance (`SweepOptions.tolerance_mm`).
    tolerance_mm: f64,
}

impl Lifts {
    /// None for a tool that does not prepare: no section proves anything,
    /// so its retracts stay at feed, as planned.
    pub(crate) fn new(tool: &ToolAssembly, tolerance_mm: f64) -> Option<Self> {
        let prepared = tool.prepare().ok()?;
        let parts = crate::verification::parts(&prepared).ok()?;
        Some(Self {
            tool: tool.id.clone(),
            sections: parts
                .iter()
                .map(|p| (p.radius.max_over(p.min, p.max), p.min))
                .collect(),
            tolerance_mm,
        })
    }
    /// "Within reach" of a section: its widest radius plus 32 sweep
    /// tolerances (#430's rule, unchanged).
    fn reach_guard(&self) -> f64 {
        32. * self.tolerance_mm
    }
    /// Whether the whole-cell bounds alone show a straight-up lift of the
    /// tool from `start` clear (#430): no cell within reach of a section
    /// stands more than one sweep tolerance above that section's lowest
    /// point. One tolerance is the guard a cell cut flat at the tip's height
    /// keeps (`Stock::preview_sweep`: `upper = z + guard`).
    pub(crate) fn bounds_clear(&self, stock: &Stock, start: Vec3) -> bool {
        stock.bounds_below_tool(&self.sections, start, self.reach_guard(), self.tolerance_mm)
    }
    /// Where a straight-up retract of this tool from `start`, planned to end
    /// at `end_z`, stops feeding (#441). None: the retract stays whole.
    ///
    /// It is split only when the run's retract policy would keep the whole
    /// lift at feed for the bounds alone: the cut history proves the column
    /// above the tool empty (`Stock::departs_clear`), but a cell within reach
    /// stands above the tool. It then feeds until the tool's lowest point is
    /// at the highest whole-cell bound within reach of any section.
    ///
    /// The stop has no margin of its own above that bound, and needs none:
    /// - A whole-cell upper bound bounds all the material in its cell from
    ///   above, and for a floor the cutter cut it already includes the sweep
    ///   guard (`Stock::preview_sweep`: `upper = z + guard`). No material
    ///   within reach stands above the stop, so the tool there touches none.
    /// - Both rules that judge the next lift accept a cell up to one sweep
    ///   tolerance above the tool's lowest point: the retract policy
    ///   (`bounds_clear`) and the machine check's floor rule, which skips
    ///   cells at or below the tool's lowest point plus that guard
    ///   (`machine::Session::floor_rise`, whose `floor_guard_mm` is the
    ///   run's sweep tolerance). The tolerance is to spare for rounding.
    /// - The work error needs no height of its own: the floor rule decides a
    ///   cell at or below that level for a straight lift whatever the error,
    ///   because the lift only raises the tool from where the fed move left
    ///   it, and the machine check decides that fed move as it does any feed.
    /// - The stock cell resolution is in the bound already: a cell bounds its
    ///   whole box, and "within reach" is measured to the box.
    ///
    /// Reach is measured from every section, the holder's included, but
    /// height from the tool's lowest point: the whole tool then stands above
    /// every cell any part of it reaches, and the floor rule skips each one.
    /// A stop measured section by section would leave a shank or holder
    /// section on the bound, where the machine check boxes it, grown by the
    /// work error, and cannot decide it.
    pub(crate) fn stop(&self, stock: &Stock, start: Vec3, end_z: f64) -> Option<f64> {
        if end_z.partial_cmp(&start[2]) != Some(std::cmp::Ordering::Greater)
            || !stock.departs_clear(&self.tool, start)
            || self.bounds_clear(stock, start)
        {
            return None;
        }
        let highest =
            stock.highest_bound_within(&self.sections, [start[0], start[1]], self.reach_guard())?;
        let lowest = self
            .sections
            .iter()
            .map(|s| s.1)
            .fold(f64::INFINITY, f64::min);
        let z = highest - lowest;
        // The lift above the stop must be one the policy makes a rapid;
        // otherwise the retract stays whole, never two fed blocks in line.
        (z > start[2] && z < end_z && self.bounds_clear(stock, [start[0], start[1], z]))
            .then_some(z)
    }
}

/// The height a planner's straight-up retract from `start` to `end_z` feeds
/// to: the stop (`Lifts::stop`) when the run splits it, else `end_z`.
pub(crate) fn feed_to(lifts: Option<&Lifts>, stock: &Stock, start: Vec3, end_z: f64) -> f64 {
    lifts
        .and_then(|l| l.stop(stock, start, end_z))
        .unwrap_or(end_z)
}

/// A planned move split as `Lifts::stop` says, when it is a straight-up
/// retract: the fed part and the rest, both retracts.
pub(crate) fn split(
    lifts: Option<&Lifts>,
    stock: &Stock,
    kind: MoveKind,
    curve: &Curve,
) -> Option<(Curve, Curve)> {
    let Curve::Line { start, end } = *curve else {
        return None;
    };
    if kind != MoveKind::Retract || start[0] != end[0] || start[1] != end[1] {
        return None;
    }
    let z = lifts?.stop(stock, start, end[2])?;
    let middle = [start[0], start[1], z];
    Some((
        Curve::Line { start, end: middle },
        Curve::Line { start: middle, end },
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{motion::Motion, stock::SweepOptions};
    use serde_json::json;
    use std::sync::atomic::AtomicBool;

    /// A 12 mm flat end mill, 40 mm out of a holder of radius 12, as
    /// `tests/pocket_wall_loop.rs` declares it.
    fn tool() -> ToolAssembly {
        serde_json::from_value(json!({
            "id": "em",
            "cutter": {"id": "em", "name": "em", "shape": {"kind": "flat"},
                "diameter_mm": 12, "cutting_length_mm": 26, "overall_length_mm": 75,
                "shank_diameter_mm": 12, "flutes": 3, "center_cutting": true,
                "material": "carbide", "neck": null},
            "stickout_mm": 40, "gauge_length_mm": 90,
            "holders": [{"z_min_mm": 40, "z_max_mm": 90, "radius_min_mm": 12, "radius_max_mm": 12}]
        }))
        .unwrap()
    }
    /// A 100 x 60 x 20 mm block, Z0 on top, at 0.5 mm cells, with a slot cut
    /// at Z-5 from X20 to X50 along Y30: its end stands against uncut stock.
    fn slotted() -> Stock {
        let mut stock = Stock::new(
            serde_json::from_value(json!({"version": 1, "units": "mm",
                "min_mm": [0, 0, -20], "max_mm": [100, 60, 0],
                "cell_size_mm": 0.5, "max_cells": 1048576}))
            .unwrap(),
        )
        .unwrap();
        let cut = Motion {
            id: 1,
            operation: "slot".into(),
            tool: "em".into(),
            kind: MoveKind::Cut,
            curve: Curve::Line {
                start: [20., 30., -5.],
                end: [50., 30., -5.],
            },
            feed_mm_min: Some(1000.),
        };
        let cancel = AtomicBool::new(false);
        let prepared = tool().prepare().unwrap();
        let delta = stock
            .preview_sweep(&prepared, &cut, 1., SweepOptions::default(), &cancel)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        stock
    }

    /// At the slot's end the uncut block stands within reach, so the whole
    /// lift would stay at feed. It feeds to the block's top Z0, the highest
    /// whole-cell bound within reach of the holder, and from there the
    /// bounds show the rest of the lift clear.
    #[test]
    fn a_lift_beside_uncut_stock_stops_feeding_on_the_stock_top() {
        let lifts = Lifts::new(&tool(), 1e-5).unwrap();
        let stock = slotted();
        let end = [50., 30., -5.];
        assert!(stock.departs_clear("em", end));
        assert!(!lifts.bounds_clear(&stock, end));
        assert_eq!(lifts.stop(&stock, end, 10.), Some(0.));
        assert!(lifts.bounds_clear(&stock, [50., 30., 0.]));
        let planned = Curve::Line {
            start: end,
            end: [50., 30., 10.],
        };
        let (fed, rest) = split(Some(&lifts), &stock, MoveKind::Retract, &planned).unwrap();
        assert_eq!(
            fed,
            Curve::Line {
                start: end,
                end: [50., 30., 0.]
            }
        );
        assert_eq!(
            rest,
            Curve::Line {
                start: [50., 30., 0.],
                end: [50., 30., 10.]
            }
        );
        // Only a planned straight-up retract is split.
        assert!(split(Some(&lifts), &stock, MoveKind::Cut, &planned).is_none());
        assert!(split(None, &stock, MoveKind::Retract, &planned).is_none());
        let slanted = Curve::Line {
            start: end,
            end: [51., 30., 10.],
        };
        assert!(split(Some(&lifts), &stock, MoveKind::Retract, &slanted).is_none());
        assert_eq!(feed_to(Some(&lifts), &stock, end, 10.), 0.);
        assert_eq!(feed_to(None, &stock, end, 10.), 10.);
    }

    /// No split where the policy needs none or cannot use one: the bounds
    /// already show the lift clear (the stop would be below it), the cut
    /// history does not prove the column empty (the lift stays fed as a
    /// whole), or the stock reaches the planned end.
    #[test]
    fn a_lift_is_split_only_where_its_rest_becomes_a_rapid() {
        let lifts = Lifts::new(&tool(), 1e-5).unwrap();
        let stock = slotted();
        let end = [50., 30., -5.];
        // Not where the last cut left the tool: no history proof.
        assert!(!stock.departs_clear("em", [40., 30., -5.]));
        assert_eq!(lifts.stop(&stock, [40., 30., -5.], 10.), None);
        // Another tool's lift: no history proof for it either.
        let mut other = lifts.clone();
        other.tool = "other".into();
        assert_eq!(other.stop(&stock, end, 10.), None);
        // Planned to end at or below the stock top beside it.
        assert_eq!(lifts.stop(&stock, end, 0.), None);
        assert_eq!(lifts.stop(&stock, end, -1.), None);
        // Not up.
        assert_eq!(lifts.stop(&stock, end, -5.), None);
        // Above the block: nothing within reach stands above the tool.
        let mut above = slotted();
        let up = Motion {
            id: 2,
            operation: "slot".into(),
            tool: "em".into(),
            kind: MoveKind::Retract,
            curve: Curve::Line {
                start: end,
                end: [50., 30., 0.],
            },
            feed_mm_min: Some(1000.),
        };
        let prepared = tool().prepare().unwrap();
        let delta = above
            .preview_sweep(
                &prepared,
                &up,
                1.,
                SweepOptions::default(),
                &AtomicBool::new(false),
            )
            .unwrap();
        above.apply_delta(&delta).unwrap();
        assert!(above.departs_clear("em", [50., 30., 0.]));
        assert!(lifts.bounds_clear(&above, [50., 30., 0.]));
        assert_eq!(lifts.stop(&above, [50., 30., 0.], 10.), None);
    }
}
