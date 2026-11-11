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
        let skin = 0.01_f64.max(64. * self.operation.tolerance_mm);
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
            for _ in 0..12 {
                if cancel.is_cancelled() {
                    return Err("committed entry search cancelled".into());
                }
                if high - low <= 0.01_f64.max(64. * self.operation.tolerance_mm) {
                    break;
                }
                let middle = (low + high) / 2.;
                let mut clear = true;
                for &(radius, bottom) in &self.air_entry_parts {
                    let height = middle + bottom - guard;
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
                        Ok(false) => {
                            clear = false;
                            break;
                        }
                        Err(_) if work == 0 && !cancel.is_cancelled() => return Ok(high),
                        Err(e) => return Err(e),
                    }
                }
                if clear {
                    high = middle;
                } else {
                    low = middle;
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
        if positive - floor <= 0.01_f64.max(64. * self.operation.tolerance_mm) {
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
