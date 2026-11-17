//! Annular cutter sweeps. Profile occupancy is finite in both radius and Z:
//! removing a ring must neither erase its center nor claim material above it.
use super::*;
use std::collections::BTreeMap;

impl Stock {
    pub(super) fn indexable_sweep(
        &self,
        delta: &mut StockDelta,
        tool: &PreparedAssembly,
        motion: &Motion,
        fraction: f64,
        options: SweepOptions,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        if fraction == 0. {
            return Ok(());
        }
        let geometry = tool
            .cutter()
            .indexable()
            .ok_or("missing indexable geometry")?;
        let body = tool
            .indexable_noncutting()
            .ok_or("missing indexable body")?;
        let milling = tool.cutter().definition().shape.indexable().unwrap();
        // A profile entirely below the limit cannot engage above it. This
        // analytic bound is stronger than a sweep's deliberately expanded
        // possible-removal intervals at the profile's top boundary.
        let has_edge_above_limit = geometry.cutting().bounds()[1][1] > milling.max_depth_of_cut_mm;
        let prepared = motion.curve.prepare(options.tolerance_mm)?;
        let bounds = prepared.bounds();
        let scale = self
            .spec
            .min_mm
            .iter()
            .chain(&self.spec.max_mm)
            .chain(&bounds.min)
            .chain(&bounds.max)
            .map(|x| x.abs())
            .fold(1., f64::max);
        let guard = options.tolerance_mm.max(512. * f64::EPSILON * scale);
        let chords = segments(&motion.curve, &prepared, fraction, options, guard, cancel)?;
        let reach = geometry.cutting().bounds()[0][1].max(body.bounds()[0][1]);
        let half = [self.step[0] / 2., self.step[1] / 2.];
        let pad = reach + half[0].hypot(half[1]) + options.chord_error_mm + 2. * guard;
        let mut scratch = BTreeMap::new();
        let mut contact = false;
        let mut stats = SweepStats {
            segments: chords.len(),
            numerical_guard_mm: guard,
            ..Default::default()
        };
        for chord in &chords {
            cancelled(cancel)?;
            stats.max_path_error_mm = stats.max_path_error_mm.max(chord.error);
            let Some(rect) = self.rect(
                [
                    chord.a[0].min(chord.b[0]) - pad,
                    chord.a[1].min(chord.b[1]) - pad,
                ],
                [
                    chord.a[0].max(chord.b[0]) + pad,
                    chord.a[1].max(chord.b[1]) + pad,
                ],
            ) else {
                continue;
            };
            for j in rect.y0..=rect.y1 {
                for i in rect.x0..=rect.x1 {
                    stats.visited_cells += 1;
                    if stats.visited_cells.is_multiple_of(1024) {
                        cancelled(cancel)?;
                    }
                    if stats.visited_cells > options.max_cell_visits {
                        return Err("indexable sweep cell-visit budget exceeded".into());
                    }
                    let index = j * self.nx + i;
                    let before = self.cells[index];
                    if before.upper_mm <= self.spec.min_mm[2] {
                        continue;
                    }
                    let center = self.cell_center(index).unwrap();
                    let meets = |span: &[f64; 2]| {
                        span[0] < before.upper_mm && span[1] > self.spec.min_mm[2]
                    };
                    // Body admission is deliberately against stock before the
                    // move. No future cut is used to excuse body contact.
                    if body
                        .sweep_column(chord.a, chord.b, center, half, chord.error)
                        .possible
                        .iter()
                        .any(meets)
                    {
                        return Err(format!(
                            "indexable tool {}: noncutting body/shank/holder may meet stock on motion {}; reduce depth, change entry or declare an assembly that clears",
                            tool.definition().id,
                            motion.id
                        ));
                    }
                    let cut = geometry.cutting().sweep_column(
                        chord.a,
                        chord.b,
                        center,
                        half,
                        chord.error,
                    );
                    if !cut.possible.iter().any(meets) {
                        continue;
                    }
                    contact = true;
                    if !motion.kind.cuts() {
                        // Feed retractions can leave a just-cut boundary;
                        // the ordinary contact checker decides cutter contact.
                        // Noncutting body checks above remain mandatory.
                        continue;
                    }
                    let cap = chord.a[2].min(chord.b[2]) + milling.max_depth_of_cut_mm;
                    if has_edge_above_limit
                        && cut
                            .possible
                            .iter()
                            .any(|s| s[0].max(cap) < before.upper_mm.min(s[1]))
                    {
                        return Err(format!(
                            "indexable tool {}: stock engagement may exceed max_depth_of_cut_mm {} on motion {} near XY ({}, {}): stock upper Z {}, permitted engagement top Z {}; reduce stepdown, refine the stock bounds or clear the higher stock first",
                            tool.definition().id,
                            milling.max_depth_of_cut_mm,
                            motion.id,
                            center[0],
                            center[1],
                            before.upper_mm,
                            cap
                        ));
                    }
                    if !scratch.contains_key(&index) && scratch.len() >= options.max_scratch_cells {
                        return Err("indexable sweep scratch-cell budget exceeded".into());
                    }
                    let after = scratch.entry(index).or_insert(before);
                    for span in &cut.possible {
                        if span[0] < after.lower_mm && span[1] > self.spec.min_mm[2] {
                            after.lower_mm = span[0].max(self.spec.min_mm[2]);
                        }
                    }
                    for span in cut.certain.iter().rev() {
                        if span[1] >= after.upper_mm && span[0] < after.upper_mm {
                            after.upper_mm = span[0].max(self.spec.min_mm[2]);
                        }
                    }
                    after.lower_mm = after.lower_mm.min(after.upper_mm);
                }
            }
        }
        if contact && motion.kind.cuts() {
            crate::tool::rotational::check_cutting_descent(tool, motion)?;
        }
        stats.scratch_cells = scratch.len();
        // Finite annular sections can leave material above a cut. Their
        // height bounds remain valid, but they do not establish the global
        // top-down premise used by later axial departure certificates.
        delta.invalidates_heightfield = contact && motion.kind.cuts();
        for (index, after) in scratch {
            delta.record_change(index, self.cells[index], after, options.max_changed_cells)?;
        }
        stats.changed_cells = delta.changes.len();
        stats.delta_bytes = delta.changes.len() * std::mem::size_of::<CellChange>();
        delta.stats = stats;
        // No filled-column clearance record: an insert ring leaves its center.
        Ok(())
    }
}
