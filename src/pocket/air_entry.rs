//! Feed-controlled vertical entry through native committed empty columns.
//! No sampled poses or inferred generation increments establish clearance.
use super::*;
use crate::engagement::EngagementScene;

pub(super) fn none(n: &usize) -> bool {
    *n == 0
}

impl PocketPlan {
    /// The lowest tip height at `point` above which every whole-cell stock
    /// upper bound lies clear of the tool assembly's sections (each section's
    /// radius plus a guard, over every cell its disk overlaps). The stream's
    /// stock-contact check reads these same bounds, so a rapid that stops
    /// above this height can be certified by it, not only by the exact
    /// committed sweeps. Outside the stock grid nothing is stock.
    pub(crate) fn cell_entry_height(&self, stock: &Stock, point: Point) -> f64 {
        self.cell_sweep_height(stock, point, 0.)
    }

    /// `cell_entry_height` for every tool position within `around` of
    /// `point` (a helix bore of radius `around` about centre `point`): the
    /// lowest tip height above which no whole-cell stock upper bound meets
    /// any tool section anywhere on that disk.
    pub(crate) fn cell_sweep_height(&self, stock: &Stock, point: Point, around: f64) -> f64 {
        let [nx, ny] = stock.dimensions();
        let [wx, wy] = stock.cell_widths_mm();
        let min = stock.spec().min_mm;
        let cells = stock.cells();
        let guard = 32. * self.operation.tolerance_mm;
        let mut height = f64::NEG_INFINITY;
        for &(radius, bottom) in &self.air_entry_parts {
            let reach = radius + around + guard;
            let span = |v: f64, o: f64, w: f64, n: usize| {
                let lo = ((v - reach - o) / w).floor().max(0.);
                let hi = ((v + reach - o) / w).floor().min(n as f64 - 1.);
                (lo as usize, hi)
            };
            let (i0, i1) = span(point[0], min[0], wx, nx);
            let (j0, j1) = span(point[1], min[1], wy, ny);
            if i1 < 0. || j1 < 0. {
                continue;
            }
            for j in j0..=j1 as usize {
                for i in i0..=i1 as usize {
                    let (x0, y0) = (min[0] + i as f64 * wx, min[1] + j as f64 * wy);
                    let dx = (x0 - point[0]).max(point[0] - x0 - wx).max(0.);
                    let dy = (y0 - point[1]).max(point[1] - y0 - wy).max(0.);
                    if dx.hypot(dy) <= reach {
                        height = height.max(cells[j * nx + i].upper_mm - bottom);
                    }
                }
            }
        }
        height
    }
    /// `cell_entry_height` for every tool position on the straight paths
    /// `path` (each within `pad` of the true path: a flattened arc's chord
    /// error): the lowest tip height above which no whole-cell stock upper
    /// bound meets any tool section anywhere along them. A cell counts when
    /// its centre is within reach plus its half diagonal of a path, so no
    /// cell a section overlaps is missed.
    pub(crate) fn cell_path_height(&self, stock: &Stock, path: &[(Point, Point)], pad: f64) -> f64 {
        self.air_entry_parts
            .iter()
            .map(|&(radius, bottom)| self.part_path_top(stock, path, radius + pad) - bottom)
            .fold(f64::NEG_INFINITY, f64::max)
    }
    /// The highest whole-cell stock upper bound a disk of `radius` (plus a
    /// guard) can meet anywhere along the straight paths `path`.
    fn part_path_top(&self, stock: &Stock, path: &[(Point, Point)], radius: f64) -> f64 {
        let [nx, ny] = stock.dimensions();
        let [wx, wy] = stock.cell_widths_mm();
        let min = stock.spec().min_mm;
        let cells = stock.cells();
        let reach = radius + 32. * self.operation.tolerance_mm;
        let half = wx.hypot(wy) / 2.;
        let mut top = f64::NEG_INFINITY;
        for &(a, b) in path {
            let d = [b[0] - a[0], b[1] - a[1]];
            let l2 = d[0] * d[0] + d[1] * d[1];
            let span = |lo: f64, hi: f64, o: f64, w: f64, n: usize| {
                let first = ((lo - reach - o) / w).floor().max(0.);
                let last = ((hi + reach - o) / w).floor().min(n as f64 - 1.);
                (first as usize, last)
            };
            let (i0, i1) = span(a[0].min(b[0]), a[0].max(b[0]), min[0], wx, nx);
            let (j0, j1) = span(a[1].min(b[1]), a[1].max(b[1]), min[1], wy, ny);
            if i1 < 0. || j1 < 0. {
                continue;
            }
            for j in j0..=j1 as usize {
                for i in i0..=i1 as usize {
                    let c = [
                        min[0] + (i as f64 + 0.5) * wx,
                        min[1] + (j as f64 + 0.5) * wy,
                    ];
                    let t = if l2 > 0. {
                        (((c[0] - a[0]) * d[0] + (c[1] - a[1]) * d[1]) / l2).clamp(0., 1.)
                    } else {
                        0.
                    };
                    let near = [a[0] + t * d[0], a[1] + t * d[1]];
                    if (c[0] - near[0]).hypot(c[1] - near[1]) <= reach + half {
                        top = top.max(cells[j * nx + i].upper_mm);
                    }
                }
            }
        }
        top
    }
    /// Whether every tool section above the flutes (neck, shank, holder) is
    /// proven clear of stock along the straight stroke `a`-`b` with the tip
    /// at `z`: the section is above the stock, or under no whole-cell stock
    /// bound it could meet, or its whole sweep lies in columns the committed
    /// cuts emptied (`Stock::history_clearance`, #200: a shank as wide as
    /// the cutter where the levels above cut). Unproven is `false`.
    pub(crate) fn clear_above_flutes(
        &self,
        a: Point,
        b: Point,
        z: f64,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        let Some(tool) = &self.rest_tool else {
            return Ok(false);
        };
        let flutes = tool.cutter.cutting_length_mm;
        let motion = Motion {
            id: self.next_id,
            operation: self.operation.id.clone(),
            tool: self.operation.tool.clone(),
            kind: MoveKind::Cut,
            curve: Stroke::line(a, b).curve(z),
            feed_mm_min: Some(self.operation.feed_mm_min),
        };
        let mut work = 1_000_000;
        for &(radius, bottom) in &self.air_entry_parts {
            if bottom < flutes - 1e-9
                || z + bottom >= stock.spec().max_mm[2]
                || self.part_path_top(stock, &[(a, b)], radius) <= z + bottom
            {
                continue;
            }
            if !stock.history_clearance(&motion, radius, bottom, &mut work, cancel)? {
                return Ok(false);
            }
        }
        Ok(true)
    }
    /// The lowest tip height, between `floor` and the operation's top, at
    /// which the committed sweeps prove every tool section clear of stock
    /// along the whole of `path` (lines and XY arcs): `cleared_entry_start`
    /// for a path instead of a point. The tool may come straight down to
    /// that height anywhere on the path and travel along it there in air.
    ///
    /// The proof is `Stock::history_clearance` (#200), section by section:
    /// each section's exact sweep along the path lies in columns the
    /// committed cuts emptied, with no margin beyond the path error, so a
    /// path the same cutter has cut before is proven, right up to a wall,
    /// and so is a shank as wide as the cutter. Returns the operation's top
    /// when nothing is proven (out of work or uncertain proves nothing).
    pub(crate) fn cleared_path_start(
        &self,
        path: &[Stroke],
        floor: f64,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<f64, String> {
        let top = self.operation.top_z_mm;
        let skin = crate::entry::skin(self.operation.tolerance_mm);
        let clear = |tip: f64| -> Result<bool, String> {
            let mut work = 1_000_000;
            for &stroke in path {
                let motion = Motion {
                    id: self.next_id,
                    operation: self.operation.id.clone(),
                    tool: self.operation.tool.clone(),
                    kind: MoveKind::Cut,
                    curve: stroke.curve(tip),
                    feed_mm_min: Some(self.operation.feed_mm_min),
                };
                for &(radius, bottom) in &self.air_entry_parts {
                    if tip + bottom > stock.spec().max_mm[2] {
                        continue;
                    }
                    if !stock.history_clearance(&motion, radius, bottom, &mut work, cancel)? {
                        return Ok(false);
                    }
                }
            }
            Ok(true)
        };
        let (mut low, mut high) = (floor.min(top), top);
        // The level's own depth first: a path cut before is clear to it.
        if high - low > skin && clear(low + skin)? {
            return Ok(low + skin);
        }
        // Halve until the bracket is within the skin its callers accept:
        // each step is a proof, and the number of steps follows from the
        // depth instead of a fixed count.
        while high - low > skin {
            if cancel.is_cancelled() {
                return Err("committed entry search cancelled".into());
            }
            let middle = (low + high) / 2.;
            if !(middle > low && middle < high) {
                break;
            }
            if clear(middle)? {
                high = middle;
            } else {
                low = middle;
            }
        }
        Ok(high)
    }
    /// Use `cache` for committed-clearance entry queries (shared across plans).
    pub(crate) fn share_entry_scene(&mut self, cache: &EntrySceneCache) {
        self.entry_scene = cache.clone();
    }
    /// Whether the committed sweeps prove a horizontal stroke `a`-`b` with
    /// the tool tip at `z` clear of stock for every tool section (each
    /// section's radius plus a guard), up to a skin of 0.01 mm (or 64
    /// tolerances) above `z`, the slack a vertical entry is allowed too.
    /// Work exhaustion or any uncertainty proves nothing.
    pub(crate) fn cleared_link(
        &self,
        a: Point,
        b: Point,
        z: f64,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        let mut cache = self
            .entry_scene
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let cached = cache.take();
        let (mut scene, mut cursor) = match cached {
            Some((scene, cursor)) if scene.stock_spec() == stock.spec() => (scene, cursor),
            _ => (
                EngagementScene::new_with_cancel(stock.spec().clone(), vec![], cancel)?,
                crate::stock::ClearanceCursor::default(),
            ),
        };
        let (rebuild, sweeps) =
            stock.committed_clearance_since(&mut cursor, self.operation.tolerance_mm, cancel)?;
        if rebuild {
            scene = EngagementScene::new_with_cancel(stock.spec().clone(), sweeps, cancel)?;
        } else {
            for sweep in sweeps {
                scene.append(sweep)?;
            }
        }
        let scene = &cache.insert((scene, cursor)).0;
        let skin = crate::entry::skin(self.operation.tolerance_mm);
        let guard = 32. * self.operation.tolerance_mm
            + 4096.
                * f64::EPSILON
                * (1. + a[0].abs() + a[1].abs() + b[0].abs() + b[1].abs() + z.abs());
        let mut work = 100_000;
        for &(radius, bottom) in &self.air_entry_parts {
            let height = z + skin + bottom - guard;
            if height > stock.spec().max_mm[2] {
                continue;
            }
            match scene.clear_line(
                a,
                b,
                radius + guard,
                height,
                self.operation.tolerance_mm,
                &mut work,
                cancel,
            ) {
                Ok(true) => (),
                Ok(false) => return Ok(false),
                Err(_) if !cancel.is_cancelled() => return Ok(false),
                Err(e) => return Err(e),
            }
        }
        Ok(true)
    }
    pub(crate) fn cleared_entry_start(
        &self,
        center: Point,
        orbit: f64,
        floor: f64,
        stock: Option<&Stock>,
        cancel: &impl CancelSignal,
    ) -> Result<(f64, bool), String> {
        #[cfg(feature = "profile-planning")]
        let _profile = crate::planning_profile::span("pocket.committed_entry");
        let top = self.operation.top_z_mm;
        let Some(stock) = stock else {
            return Ok((top, false));
        };
        // Reuse the scene across entries and append only sweeps committed
        // since the last entry; rebuilding it from the whole history at every
        // entry made long jobs quadratic. A failed update drops the cache.
        let mut cache = self
            .entry_scene
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let cached = cache.take();
        let (mut scene, mut cursor) = match cached {
            Some((scene, cursor)) if scene.stock_spec() == stock.spec() => (scene, cursor),
            _ => (
                EngagementScene::new_with_cancel(stock.spec().clone(), vec![], cancel)?,
                crate::stock::ClearanceCursor::default(),
            ),
        };
        let (rebuild, sweeps) =
            stock.committed_clearance_since(&mut cursor, self.operation.tolerance_mm, cancel)?;
        if rebuild {
            scene = EngagementScene::new_with_cancel(stock.spec().clone(), sweeps, cancel)?;
        } else {
            for sweep in sweeps {
                scene.append(sweep)?;
            }
        }
        let scene = &cache.insert((scene, cursor)).0;
        // No early exit without committed sweeps: a column outside the
        // original stock is air on a fresh stock too (#167: every entry
        // from beside the stock was otherwise unproven until something had
        // been cut, so the first pass could only start with a helix).
        let height = |point: Point| -> Result<f64, String> {
            let guard = 32. * self.operation.tolerance_mm
                + 4096. * f64::EPSILON * (1. + point[0].abs() + point[1].abs() + top.abs());
            let mut work = 100_000;
            let (mut low, mut high) = (floor, top);
            // A supported flat pocket has cylindrical cutter/neck/shank sections
            // and linear holder tapers. Maximum endpoint radius encloses every
            // section, from its lowest Z through the entire vertical descent.
            // Halve until the bracket is within the skin the callers accept
            // (`entry::skin`, the one definition both sides use). A fixed 12
            // halvings left (top - floor) / 4096, over that skin once the
            // floor was ~41 mm below the top, so every deep entry was
            // refused (#237). Each step is a proof; the bracket halves, so
            // the loop ends, and a bracket that can no longer be halved (or
            // is not a number) ends it too.
            let skin = crate::entry::skin(self.operation.tolerance_mm);
            // Whether every tool section is proven clear with the tip at
            // `tip`; `None` when the work ran out (nothing is proven then).
            let mut clear_at = |tip: f64| -> Result<Option<bool>, String> {
                for &(radius, bottom) in &self.air_entry_parts {
                    let height = tip + bottom - guard;
                    if height > stock.spec().max_mm[2] {
                        continue;
                    }
                    match scene.clear_disk(
                        point,
                        radius + guard,
                        height,
                        self.operation.tolerance_mm,
                        &mut work,
                        cancel,
                    ) {
                        Ok(true) => (),
                        Ok(false) => return Ok(Some(false)),
                        Err(_) if work == 0 && !cancel.is_cancelled() => return Ok(None),
                        Err(e) => return Err(e),
                    }
                }
                Ok(Some(true))
            };
            // The level itself first: "this start reaches the level" is one
            // proof at one height, the skin above the floor, so the answer
            // does not depend on where the halving below happens to land
            // (the same column was otherwise accepted under one top and
            // refused under another).
            if high - low > skin {
                // The height its callers accept: no more than the skin above
                // the floor after rounding too.
                let mut level = low + skin;
                if level - low > skin {
                    level = level.next_down();
                }
                match clear_at(level)? {
                    Some(true) => return Ok(level),
                    Some(false) => low = level,
                    None => return Ok(high),
                }
            }
            while high - low > skin {
                if cancel.is_cancelled() {
                    return Err("committed entry search cancelled".into());
                }
                let middle = (low + high) / 2.;
                if !(middle > low && middle < high) {
                    break;
                }
                match clear_at(middle)? {
                    Some(true) => high = middle,
                    Some(false) => low = middle,
                    None => return Ok(high),
                }
            }
            // High is either the unchanged authored top or a whole-assembly proof.
            // Stock uncertainty/work exhaustion can only retain more cutting entry.
            Ok(high)
        };
        let positive = height([center[0] + orbit, center[1]])?;
        // The fitted disk admits the complete circle, independent of phase.
        // Its +X extremity may sit on an old sweep's uncertainty band. Try the
        // opposite phase before deciding to repeat a deep cutting descent.
        if positive - floor <= crate::entry::skin(self.operation.tolerance_mm) {
            return Ok((positive, false));
        }
        let negative = height([center[0] - orbit, center[1]])?;
        Ok(if negative < positive - 0.01 {
            (negative, true)
        } else {
            (positive, false)
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::stock::SweepOptions;

    fn deep_cleared_path() -> (PocketPlan, Stock, Stock, Stroke) {
        let manifest: crate::stream::StreamManifest = serde_json::from_str(include_str!(
            "../../fixtures/operations/helical-pocket.json"
        ))
        .unwrap();
        let Operation::Pocket(operation) = manifest.operations[0].clone() else {
            panic!("pocket fixture")
        };
        let mut plan = PocketPlan::prepare(
            operation,
            &manifest.tools[0],
            manifest.stock.height().unwrap(),
            manifest.initial_position_mm,
            1,
            &|| false,
        )
        .unwrap();
        let path = *plan.strokes.iter().find(|s| s.arc.is_none()).unwrap();
        let mut spec = manifest.stock.height().unwrap().clone();
        spec.max_mm[2] = 100_000.;
        spec.cell_size_mm = 100.;
        plan.operation.top_z_mm = spec.max_mm[2];
        let uncut = Stock::new(spec.clone()).unwrap();
        let mut cleared = Stock::new(spec).unwrap();
        // Actual committed cuts clear a tube wider than every part of this
        // assembly. The coarse cell also holds uncut stock beside the tube.
        let mut earlier = manifest.tools[0].clone();
        earlier.cutter.diameter_mm = 16.;
        earlier.cutter.cutting_length_mm = 100_001.;
        earlier.cutter.overall_length_mm = 100_010.;
        earlier.cutter.shank_diameter_mm = 16.;
        earlier.stickout_mm = 100_005.;
        earlier.gauge_length_mm = 100_020.;
        earlier.holders[0].z_min_mm = 100_005.;
        earlier.holders[0].z_max_mm = 100_020.;
        earlier.holders[0].radius_min_mm = 8.;
        earlier.holders[0].radius_max_mm = 8.;
        let prepared = earlier.prepare().unwrap();
        let motion = Motion {
            id: 1,
            operation: "earlier-clearance".into(),
            tool: earlier.id.clone(),
            kind: MoveKind::Cut,
            curve: path.curve(2.7),
            feed_mm_min: Some(100.),
        };
        let delta = cleared
            .preview_sweep(&prepared, &motion, 1., SweepOptions::default(), &|| false)
            .unwrap();
        cleared.apply_delta(&delta).unwrap();
        assert_eq!(
            plan.cell_path_height(&cleared, &[(path.a, path.b)], 0.005),
            100_000.
        );
        (plan, uncut, cleared, path)
    }

    #[test]
    fn committed_path_entry_search_converges_with_a_deep_height_bracket() {
        let (plan, uncut, cleared, path) = deep_cleared_path();
        let skin = crate::entry::skin(plan.operation.tolerance_mm);
        let from = plan
            .cleared_path_start(&[path], 0., &cleared, &|| false)
            .unwrap();
        assert!(from >= 2.7, "unproved entry below the material: {from}");
        assert!(from <= 2.7 + 2. * skin, "entry unnecessarily high: {from}");
        assert_eq!(
            plan.cleared_path_start(&[path], 0., &uncut, &|| false)
                .unwrap(),
            plan.operation.top_z_mm
        );
        assert!(
            plan.cleared_path_start(&[path], 0., &cleared, &|| true)
                .is_err()
        );
    }

    #[test]
    fn rest_ramp_entry_uses_the_committed_floor_instead_of_a_coarse_search_height() {
        let (mut plan, _, cleared, path) = deep_cleared_path();
        plan.operation.entry = Some(crate::entry::PocketEntry::Ramp(crate::entry::RampEntry {
            angle_rad: 3f64.to_radians(),
            min_length_mm: Some(0.1),
            feed_mm_min: Some(100.),
        }));
        plan.operation.rest_tolerance_mm = Some(0.01);
        plan.operation.clearance_z_mm = 100_002.;
        plan.position[2] = 100_002.;
        let (moves, _) = plan
            .rest_ramp_moves(path, 0., Some(&cleared), &|| false)
            .unwrap();
        let mut highest = f64::NEG_INFINITY;
        for (curve, _, _) in moves {
            let (a, b) = (curve.start(), curve.end());
            let horizontal = norm([a[0], a[1]], [b[0], b[1]]);
            if a[2] > b[2] && horizontal > 0. {
                highest = highest.max(a[2]);
                assert!(a[2] - b[2] <= horizontal * 3f64.to_radians().tan() + 1e-9);
            }
        }
        assert!(highest.is_finite());
        assert!(highest <= 2.72, "rest ramp starts in proven air: {highest}");
    }
}
