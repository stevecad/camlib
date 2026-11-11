//! Axisymmetric turning stock: a bar or tube about the Z (spindle) axis.
//!
//! The stock is split into bins along Z. In each bin the outer surface radius
//! is known within bounds for every z of the bin (`outer.lower <= R(z) <=
//! outer.upper`), and so is the bore radius (`inner`, zero for a solid bar).
//! Material lies between the bore and the outer surface. Bounds only shrink
//! the material: a cut lowers outer bounds or raises inner bounds.
//!
//! A tool nose (a circle of radius `rn` about the nose center) moving along a
//! line sweeps a capsule in the (r, z) half-plane. For an outside tool all
//! material at a radius above the capsule's lower boundary within its z span
//! is removed by the insert or would hit the insert or holder, which the
//! contact checks report separately. The capsule's lower boundary is a convex
//! function of z, so its maximum over a bin is at the bin's ends and its
//! minimum at its lowest point or an end: the bounds are updated exactly. An
//! inside (boring) tool is the mirror image (radius negated).
use crate::motion::{Curve, Plane, Vec3};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TurnStockSpec {
    pub version: u32,
    pub units: String,
    /// Z range of the stock (spindle axis, work coordinates).
    pub z_min_mm: f64,
    pub z_max_mm: f64,
    pub outer_radius_mm: f64,
    /// Bore of a tube; zero for a solid bar.
    #[serde(default)]
    pub bore_radius_mm: f64,
    /// Bin length along Z.
    pub cell_mm: f64,
    pub max_cells: usize,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RadiusBounds {
    pub lower_mm: f64,
    pub upper_mm: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TurnCell {
    pub outer: RadiusBounds,
    pub inner: RadiusBounds,
}

/// Which surface a tool cuts.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TurnSide {
    /// Turning and facing from outside: material below the nose is kept.
    Outer,
    /// Boring from inside: material above the nose is kept.
    Inner,
}

#[derive(Clone, Debug)]
pub struct TurnStock {
    spec: TurnStockSpec,
    cells: Vec<TurnCell>,
    generation: u64,
    /// Rings certainly removed inside the material (face grooves): [z0, z1,
    /// r0, r1]. The per-z radius bounds cannot hold a ring, so they keep
    /// counting it as possible material; the volume excludes it.
    rings: Vec<[f64; 4]>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TurnChange {
    pub index: usize,
    pub before: TurnCell,
    pub after: TurnCell,
}

impl TurnStockSpec {
    pub fn validate(&self) -> Result<usize, String> {
        let finite = [
            self.z_min_mm,
            self.z_max_mm,
            self.outer_radius_mm,
            self.bore_radius_mm,
            self.cell_mm,
        ]
        .iter()
        .all(|v| v.is_finite() && v.abs() <= 1e6);
        if self.version != 1
            || self.units != "mm"
            || !finite
            || self.z_max_mm <= self.z_min_mm
            || self.outer_radius_mm <= 0.
            || !(0. ..self.outer_radius_mm).contains(&self.bore_radius_mm)
            || self.cell_mm <= 0.
        {
            return Err("turning stock needs version 1, mm, z_min < z_max, 0 <= bore < outer radius and a positive cell length".into());
        }
        let n = ((self.z_max_mm - self.z_min_mm) / self.cell_mm).ceil() as usize;
        if n == 0 || n > self.max_cells {
            return Err(format!(
                "turning stock needs {n} cells, more than max_cells {}",
                self.max_cells
            ));
        }
        Ok(n)
    }
}

/// Lowest radius of the capsule (segment a-b, radius rn) on the line z = c,
/// or None if the capsule does not reach that z.
fn capsule_min_r(a: [f64; 2], b: [f64; 2], rn: f64, c: f64) -> Option<f64> {
    let mut best: Option<f64> = None;
    let mut take = |v: f64| best = Some(best.map_or(v, |b: f64| b.min(v)));
    for p in [a, b] {
        let dz = p[1] - c;
        if dz.abs() <= rn {
            take(p[0] - (rn * rn - dz * dz).max(0.).sqrt());
        }
    }
    // The capsule's straight side facing the axis: the segment shifted by
    // rn along the normal with a negative r component.
    let d = [b[0] - a[0], b[1] - a[1]];
    let l = d[0].hypot(d[1]);
    if l > 0. {
        let mut n = [d[1] / l, -d[0] / l];
        if n[0] > 0. || (n[0] == 0. && n[1] > 0.) {
            n = [-n[0], -n[1]];
        }
        let (p, q) = (
            [a[0] + rn * n[0], a[1] + rn * n[1]],
            [b[0] + rn * n[0], b[1] + rn * n[1]],
        );
        if q[1] != p[1] {
            let t = (c - p[1]) / (q[1] - p[1]);
            if (0. ..=1.).contains(&t) {
                take(p[0] + t * (q[0] - p[0]));
            }
        } else if (p[1] - c).abs() <= 1e-12 {
            take(p[0].min(q[0]));
        }
    }
    best
}

/// The capsule's lowest point: (r, z).
fn capsule_lowest(a: [f64; 2], b: [f64; 2], rn: f64) -> [f64; 2] {
    // The segment point with the least r, shifted rn toward the axis.
    let p = if a[0] < b[0] {
        a
    } else if b[0] < a[0] {
        b
    } else {
        [a[0], (a[1] + b[1]) / 2.]
    };
    [p[0] - rn, p[1]]
}

impl TurnStock {
    pub fn new(spec: TurnStockSpec) -> Result<Self, String> {
        let n = spec.validate()?;
        let cell = TurnCell {
            outer: RadiusBounds {
                lower_mm: spec.outer_radius_mm,
                upper_mm: spec.outer_radius_mm,
            },
            inner: RadiusBounds {
                lower_mm: spec.bore_radius_mm,
                upper_mm: spec.bore_radius_mm,
            },
        };
        Ok(Self {
            cells: vec![cell; n],
            spec,
            generation: 0,
            rings: vec![],
        })
    }
    pub fn spec(&self) -> &TurnStockSpec {
        &self.spec
    }
    pub fn cells(&self) -> &[TurnCell] {
        &self.cells
    }
    pub fn generation(&self) -> u64 {
        self.generation
    }
    /// The Z range of bin i.
    pub fn bin(&self, i: usize) -> [f64; 2] {
        let z0 = self.spec.z_min_mm + i as f64 * self.spec.cell_mm;
        [z0, (z0 + self.spec.cell_mm).min(self.spec.z_max_mm)]
    }
    /// Removed and remaining volume bounds (mm³).
    pub fn volume(&self) -> [f64; 2] {
        let mut v = [0.; 2];
        let pi = std::f64::consts::PI;
        for (i, c) in self.cells.iter().enumerate() {
            let [z0, z1] = self.bin(i);
            let h = z1 - z0;
            let area = |outer: f64, inner: f64| pi * (outer.max(inner).powi(2) - inner.powi(2)) * h;
            v[0] += area(c.outer.lower_mm, c.inner.upper_mm);
            v[1] += area(c.outer.upper_mm, c.inner.lower_mm);
        }
        // Less the removed rings within each bound's material band (the
        // stored rings are disjoint).
        for (k, bound) in v.iter_mut().enumerate() {
            for ring in &self.rings {
                for (i, c) in self.cells.iter().enumerate() {
                    let [z0, z1] = self.bin(i);
                    let h = z1.min(ring[1]) - z0.max(ring[0]);
                    if h <= 0. {
                        continue;
                    }
                    let (lo, hi) = if k == 0 {
                        (c.inner.upper_mm, c.outer.lower_mm)
                    } else {
                        (c.inner.lower_mm, c.outer.upper_mm)
                    };
                    let (a, b) = (ring[2].max(lo), ring[3].min(hi));
                    if b > a {
                        *bound -= pi * (b * b - a * a) * h;
                    }
                }
            }
        }
        [v[0].max(0.), v[1].max(0.)]
    }
    /// The cell changes of a convex polygon (in (u, v) = (z, r): the region
    /// an insert sweeps along a straight move) cutting the `side` surface:
    /// within each z bin, material beyond the polygon's near boundary (its
    /// lowest r outside, highest r inside) may be gone, and is certainly
    /// gone where the polygon spans the whole bin. `excess`: how far the
    /// polygon may stand outside the true tool (its circumscribed nose), kept
    /// out of the certain removal. Not applied.
    pub fn preview_polygon(
        &self,
        polygon: &[[f64; 2]],
        side: TurnSide,
        excess: f64,
    ) -> Vec<TurnChange> {
        let flip = |v: f64| match side {
            TurnSide::Outer => v,
            TurnSide::Inner => -v,
        };
        let poly: Vec<[f64; 2]> = polygon.iter().map(|p| [p[0], flip(p[1])]).collect();
        let (lo, hi) = poly
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), p| {
                (l.min(p[0]), h.max(p[0]))
            });
        // The polygon's lowest (flipped) radius at z.
        let lowest = |z: f64| -> Option<f64> {
            let n = poly.len();
            let mut best: Option<f64> = None;
            for i in 0..n {
                let (p, q) = (poly[i], poly[(i + 1) % n]);
                let (a, b) = (p[0].min(q[0]), p[0].max(q[0]));
                if z < a || z > b {
                    continue;
                }
                let v = if q[0] == p[0] {
                    p[1].min(q[1])
                } else {
                    p[1] + (z - p[0]) / (q[0] - p[0]) * (q[1] - p[1])
                };
                best = Some(best.map_or(v, |x: f64| x.min(v)));
            }
            best
        };
        let cell = self.spec.cell_mm;
        let first = (((lo - self.spec.z_min_mm) / cell).floor().max(0.)) as usize;
        let last = ((((hi - self.spec.z_min_mm) / cell).floor()) as usize)
            .min(self.cells.len().saturating_sub(1));
        let mut changes = vec![];
        if !(lo.is_finite() && hi.is_finite()) || hi < self.spec.z_min_mm || lo > self.spec.z_max_mm
        {
            return changes;
        }
        for i in first..=last {
            let cell_now = self.cells[i];
            let [za, zb] = self.bin(i);
            if zb <= lo || za >= hi {
                continue;
            }
            // Least over the bin (the lower boundary is convex: at a vertex
            // or an end), greatest at an end.
            let samples = [za.max(lo), zb.min(hi)]
                .into_iter()
                .chain(poly.iter().map(|p| p[0]).filter(|z| *z > za && *z < zb));
            let low = samples.filter_map(lowest).fold(f64::INFINITY, f64::min);
            if !low.is_finite() {
                continue;
            }
            let covers = lo <= za && hi >= zb;
            let high = if covers {
                lowest(za).zip(lowest(zb)).map(|(a, b)| a.max(b) + excess)
            } else {
                None
            };
            let mut after = cell_now;
            match side {
                TurnSide::Outer => {
                    let floor = cell_now.inner.lower_mm;
                    after.outer.lower_mm = after.outer.lower_mm.min(low.max(floor));
                    if let Some(high) = high {
                        after.outer.upper_mm = after.outer.upper_mm.min(high.max(floor));
                    }
                }
                TurnSide::Inner => {
                    let ceiling = cell_now.outer.upper_mm;
                    after.inner.upper_mm = after.inner.upper_mm.max((-low).min(ceiling));
                    if let Some(high) = high {
                        after.inner.lower_mm = after.inner.lower_mm.max((-high).min(ceiling));
                    }
                }
            }
            if after.outer.upper_mm < after.outer.lower_mm {
                after.outer.lower_mm = after.outer.upper_mm;
            }
            if after.inner.lower_mm > after.inner.upper_mm {
                after.inner.upper_mm = after.inner.lower_mm;
            }
            if after != cell_now {
                changes.push(TurnChange {
                    index: i,
                    before: cell_now,
                    after,
                });
            }
        }
        changes
    }

    /// Record a ring certainly removed inside the material (a face groove):
    /// z0..z1, r0..r1. Overlapping rings are merged into disjoint ones.
    pub fn remove_ring(&mut self, z: [f64; 2], r: [f64; 2]) {
        // Within the stock's extent.
        let z = [z[0].max(self.spec.z_min_mm), z[1].min(self.spec.z_max_mm)];
        let r = [r[0].max(0.), r[1].min(self.spec.outer_radius_mm)];
        if !(z[1] > z[0] && r[1] > r[0]) {
            return;
        }
        // Keep the stored rings disjoint: add only the parts not covered.
        let mut pieces = vec![[z[0], z[1], r[0], r[1]]];
        for ring in &self.rings {
            let mut next = vec![];
            for p in pieces {
                let (iz0, iz1) = (p[0].max(ring[0]), p[1].min(ring[1]));
                let (ir0, ir1) = (p[2].max(ring[2]), p[3].min(ring[3]));
                if iz1 <= iz0 || ir1 <= ir0 {
                    next.push(p);
                    continue;
                }
                // Up to four pieces of p outside the overlap.
                for q in [
                    [p[0], iz0, p[2], p[3]],
                    [iz1, p[1], p[2], p[3]],
                    [iz0, iz1, p[2], ir0],
                    [iz0, iz1, ir1, p[3]],
                ] {
                    if q[1] > q[0] && q[3] > q[2] {
                        next.push(q);
                    }
                }
            }
            pieces = next;
        }
        self.rings.extend(pieces);
        self.generation += 1;
    }
    pub fn rings(&self) -> &[[f64; 4]] {
        &self.rings
    }

    /// The cell changes that turn this stock into `after` (same spec), in
    /// ascending index order.
    pub fn changes_to(&self, after: &TurnStock) -> Vec<TurnChange> {
        self.cells
            .iter()
            .zip(&after.cells)
            .enumerate()
            .filter(|(_, (b, a))| b != a)
            .map(|(index, (&before, &after))| TurnChange {
                index,
                before,
                after,
            })
            .collect()
    }

    /// Apply one move's recorded change: its cells (each must match the
    /// current stock and only remove material) and the rings it removed.
    pub fn apply_move(&mut self, changes: &[TurnChange], rings: &[[f64; 4]]) -> Result<(), String> {
        for r in rings {
            if !(r.iter().all(|v| v.is_finite()) && r[1] > r[0] && r[3] > r[2]) {
                return Err("turning stock ring must be a finite nonempty z/r range".into());
            }
        }
        // apply validates strict cell ordering and every change before any
        // cells are committed; rings must remain untouched if that fails.
        self.apply(changes)?;
        for r in rings {
            self.remove_ring([r[0], r[1]], [r[2], r[3]]);
        }
        Ok(())
    }

    /// Rebuild a stock from its parts (a checkpoint): every cell inside the
    /// stock's radial extent with ordered bounds.
    pub fn from_parts(
        spec: TurnStockSpec,
        cells: Vec<TurnCell>,
        rings: Vec<[f64; 4]>,
    ) -> Result<Self, String> {
        let mut stock = Self::new(spec)?;
        if cells.len() != stock.cells.len() || !cells.iter().all(|c| stock.cell_fits(*c)) {
            return Err("turning checkpoint cells do not fit the stock".into());
        }
        stock.cells = cells;
        for r in rings {
            if !(r.iter().all(|v| v.is_finite()) && r[1] > r[0] && r[3] > r[2]) {
                return Err("turning checkpoint ring must be a finite nonempty z/r range".into());
            }
            // Use the same disjoint union as live removal: even a valid
            // checkpoint range may overlap another or extend past the stock.
            stock.remove_ring([r[0], r[1]], [r[2], r[3]]);
        }
        stock.generation = 0;
        Ok(stock)
    }

    /// The cell changes of a nose of radius `rn` moving along `curve` (an
    /// XZ line or arc, x = radius) cutting the `side` surface. Not applied.
    pub fn preview(
        &self,
        curve: &Curve,
        rn: f64,
        side: TurnSide,
        tolerance: f64,
    ) -> Result<Vec<TurnChange>, String> {
        self.preview_within(curve, rn, side, tolerance, 0.)
    }

    /// `preview` for a nose whose actual path stays within `slack` of
    /// `curve` (an arc read back with slightly different start and end
    /// radii, which a controller blends): the solid bound allows for the
    /// nose that much nearer, the removal only for what it cuts if that
    /// much farther.
    pub fn preview_within(
        &self,
        curve: &Curve,
        rn: f64,
        side: TurnSide,
        tolerance: f64,
        slack: f64,
    ) -> Result<Vec<TurnChange>, String> {
        if !(rn.is_finite() && rn >= 0.) {
            return Err("nose radius must be finite and not negative".into());
        }
        if !(slack.is_finite() && slack >= 0.) {
            return Err("path slack must be finite and not negative".into());
        }
        let rz = |p: Vec3| [p[0], p[2]];
        // Segments with the swept radius used for each bound: a chord of an
        // arc strays by its sagitta, so the solid (lower) bound uses the
        // grown radius and the removal (upper) bound the shrunk one.
        let mut segments: Vec<([f64; 2], [f64; 2], f64)> = vec![];
        match *curve {
            Curve::Line { start, end } => segments.push((rz(start), rz(end), slack)),
            Curve::Arc {
                start,
                center,
                plane: Plane::Xz,
                sweep_rad,
                ..
            } => {
                // XZ arcs (G18): angles from +Z toward +X.
                let c = rz(center);
                let s = rz(start);
                let radius = (s[0] - c[0]).hypot(s[1] - c[1]);
                let a0 = (s[0] - c[0]).atan2(s[1] - c[1]);
                // Chords within `tolerance` of the arc: a chord over the
                // angle θ strays r(1 - cos(θ/2)) <= rθ²/8.
                let n = ((sweep_rad.abs() * (radius / (8. * tolerance.max(1e-6))).sqrt()).ceil()
                    as usize)
                    // A multi-turn arc must not collapse to a chord whose
                    // cosine-based sagitta aliases one or more full circles.
                    .max((sweep_rad.abs() / std::f64::consts::PI).ceil() as usize)
                    .clamp(1, 4096);
                let sag = radius * (1. - (sweep_rad.abs() / (2. * n as f64)).cos()) + slack;
                let point = |k: usize| {
                    let a = a0 + sweep_rad * k as f64 / n as f64;
                    [c[0] + radius * a.sin(), c[1] + radius * a.cos()]
                };
                for k in 0..n {
                    segments.push((point(k), point(k + 1), sag));
                }
            }
            _ => return Err("turning moves are lines and arcs in the XZ plane".into()),
        }
        let flip = |p: [f64; 2]| match side {
            TurnSide::Outer => p,
            TurnSide::Inner => [-p[0], p[1]],
        };
        let mut changes = vec![];
        for (i, cell) in self.cells.iter().enumerate() {
            let [za, zb] = self.bin(i);
            let mut low = f64::INFINITY; // least swept radius within the bin
            let mut high = f64::INFINITY; // greatest, over z, of the swept boundary
            let mut covers = false;
            for &(a, b, sag) in &segments {
                let (a, b) = (flip(a), flip(b));
                let (zlo, zhi) = (a[1].min(b[1]), a[1].max(b[1]));
                let grown = rn + sag;
                if zhi + grown < za || zlo - grown > zb {
                    continue;
                }
                // Solid bound: the grown capsule's least radius in the bin.
                let lowest = capsule_lowest(a, b, grown);
                if (za..=zb).contains(&lowest[1]) {
                    low = low.min(lowest[0]);
                }
                for z in [za, zb] {
                    if let Some(r) = capsule_min_r(a, b, grown, z) {
                        low = low.min(r);
                    }
                }
                // Removal: the shrunk capsule must span the whole bin.
                // If the chord error exceeds the nose radius, even the chord
                // itself is not necessarily inside the true sweep. It can
                // bound possible removal, but cannot certify cleared stock.
                if sag > rn {
                    continue;
                }
                let shrunk = rn - sag;
                if let (Some(ra), Some(rb)) = (
                    capsule_min_r(a, b, shrunk, za),
                    capsule_min_r(a, b, shrunk, zb),
                ) {
                    high = high.min(ra.max(rb));
                    covers = true;
                }
            }
            if !low.is_finite() {
                continue;
            }
            let mut after = *cell;
            match side {
                TurnSide::Outer => {
                    let floor = cell.inner.lower_mm;
                    after.outer.lower_mm = after.outer.lower_mm.min(low.max(floor));
                    if covers {
                        after.outer.upper_mm = after.outer.upper_mm.min(high.max(floor));
                    }
                }
                TurnSide::Inner => {
                    // Mirrored: radii are negated.
                    let ceiling = cell.outer.upper_mm;
                    after.inner.upper_mm = after.inner.upper_mm.max((-low).min(ceiling));
                    if covers {
                        after.inner.lower_mm = after.inner.lower_mm.max((-high).min(ceiling));
                    }
                }
            }
            if after.outer.upper_mm < after.outer.lower_mm {
                after.outer.lower_mm = after.outer.upper_mm;
            }
            if after.inner.lower_mm > after.inner.upper_mm {
                after.inner.upper_mm = after.inner.lower_mm;
            }
            if after != *cell {
                changes.push(TurnChange {
                    index: i,
                    before: *cell,
                    after,
                });
            }
        }
        Ok(changes)
    }

    /// A thread cut between z0 and z1 down to `radius` (the thread's root):
    /// the crests stay, so only the solid (lower) outer bound drops.
    pub fn thread_cut(&self, z0: f64, z1: f64, radius: f64) -> Vec<TurnChange> {
        let (lo, hi) = (z0.min(z1), z0.max(z1));
        let mut changes = vec![];
        for (i, cell) in self.cells.iter().enumerate() {
            let [za, zb] = self.bin(i);
            if zb <= lo || za >= hi {
                continue;
            }
            let mut after = *cell;
            after.outer.lower_mm = after.outer.lower_mm.min(radius.max(cell.inner.lower_mm));
            if after != *cell {
                changes.push(TurnChange {
                    index: i,
                    before: *cell,
                    after,
                });
            }
        }
        changes
    }

    /// An internal thread cut between z0 and z1 out to `radius` (the
    /// thread's root): the crests stay, so only the bore's "may be opened"
    /// (upper) bound grows.
    pub fn thread_cut_inner(&self, z0: f64, z1: f64, radius: f64) -> Vec<TurnChange> {
        let (lo, hi) = (z0.min(z1), z0.max(z1));
        let mut changes = vec![];
        for (i, cell) in self.cells.iter().enumerate() {
            let [za, zb] = self.bin(i);
            if zb <= lo || za >= hi {
                continue;
            }
            let mut after = *cell;
            after.inner.upper_mm = after.inner.upper_mm.max(radius.min(cell.outer.upper_mm));
            if after != *cell {
                changes.push(TurnChange {
                    index: i,
                    before: *cell,
                    after,
                });
            }
        }
        changes
    }

    /// A drill of `radius` with a point `cone` long, its tip down to
    /// `tip_z`: the bore is the drill's radius where its full diameter has
    /// passed; in the point's cone it is opened at least to the cone's
    /// radius at the bin's lower end and at most to the drill's.
    pub fn drill(&self, tip_z: f64, radius: f64, cone: f64) -> Vec<TurnChange> {
        let mut changes = vec![];
        for (i, cell) in self.cells.iter().enumerate() {
            let [za, zb] = self.bin(i);
            if zb <= tip_z {
                continue;
            }
            let at = |z: f64| {
                if z < tip_z {
                    0.
                } else if cone <= 0. {
                    radius
                } else {
                    ((z - tip_z) / cone * radius).clamp(0., radius)
                }
            };
            let mut after = *cell;
            let ceiling = cell.outer.upper_mm;
            after.inner.upper_mm = after.inner.upper_mm.max(at(zb).min(ceiling));
            after.inner.lower_mm = after.inner.lower_mm.max(at(za).min(ceiling));
            if after.inner.lower_mm > after.inner.upper_mm {
                after.inner.upper_mm = after.inner.lower_mm;
            }
            if after != *cell {
                changes.push(TurnChange {
                    index: i,
                    before: *cell,
                    after,
                });
            }
        }
        changes
    }

    fn cell_fits(&self, cell: TurnCell) -> bool {
        [cell.outer, cell.inner].into_iter().all(|b| {
            b.lower_mm.is_finite()
                && b.upper_mm.is_finite()
                && self.spec.bore_radius_mm <= b.lower_mm
                && b.lower_mm <= b.upper_mm
                && b.upper_mm <= self.spec.outer_radius_mm
        })
    }

    pub fn apply(&mut self, changes: &[TurnChange]) -> Result<(), String> {
        if changes.windows(2).any(|w| w[0].index >= w[1].index) {
            return Err("turning stock changes must be in ascending cell order".into());
        }
        for c in changes {
            if !self.cell_fits(c.after) {
                return Err("turning stock change has invalid radius bounds".into());
            }
            if self.cells.get(c.index) != Some(&c.before)
                || c.after.outer.upper_mm > c.before.outer.upper_mm
                || c.after.outer.lower_mm > c.before.outer.lower_mm
                || c.after.inner.lower_mm < c.before.inner.lower_mm
                || c.after.inner.upper_mm < c.before.inner.upper_mm
            {
                return Err(
                    "turning stock change does not match the current stock or adds material".into(),
                );
            }
        }
        for c in changes {
            self.cells[c.index] = c.after;
        }
        self.generation += 1;
        Ok(())
    }

    /// The most material that may remain at z: (bore lower bound, outer
    /// upper bound) of its bin.
    pub fn envelope_at(&self, z: f64) -> Option<(f64, f64)> {
        if !(self.spec.z_min_mm..=self.spec.z_max_mm).contains(&z) {
            return None;
        }
        let i = (((z - self.spec.z_min_mm) / self.spec.cell_mm) as usize).min(self.cells.len() - 1);
        let c = self.cells[i];
        (c.outer.upper_mm > c.inner.lower_mm).then_some((c.inner.lower_mm, c.outer.upper_mm))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec() -> TurnStockSpec {
        TurnStockSpec {
            version: 1,
            units: "mm".into(),
            z_min_mm: -50.,
            z_max_mm: 0.,
            outer_radius_mm: 20.,
            bore_radius_mm: 0.,
            cell_mm: 0.5,
            max_cells: 10_000,
        }
    }
    fn line(a: [f64; 2], b: [f64; 2]) -> Curve {
        Curve::Line {
            start: [a[0], 0., a[1]],
            end: [b[0], 0., b[1]],
        }
    }

    #[test]
    fn a_turning_pass_lowers_the_outer_radius_along_its_length() {
        let mut s = TurnStock::new(spec()).unwrap();
        // Nose center at r = 18.8 (rn 0.8): the surface becomes r = 18.
        let c = s
            .preview(&line([18.8, 1.], [18.8, -30.]), 0.8, TurnSide::Outer, 1e-4)
            .unwrap();
        s.apply(&c).unwrap();
        for (i, cell) in s.cells().iter().enumerate() {
            let [za, zb] = s.bin(i);
            if zb <= -30. - 0.8 || za >= 1. {
                assert_eq!(cell.outer.upper_mm, 20., "{za}");
            } else if za >= -30. && zb <= 0. {
                assert!((cell.outer.upper_mm - 18.).abs() < 1e-9, "{za} {cell:?}");
                assert!((cell.outer.lower_mm - 18.).abs() < 1e-9);
            }
        }
        // The nose's round end at z = -30 leaves a fillet: in the last bins
        // the bounds differ (some material may remain up to the old radius).
        let i = s
            .cells()
            .iter()
            .position(|c| c.outer.upper_mm < 20.)
            .unwrap();
        assert!(s.cells()[i].outer.lower_mm <= s.cells()[i].outer.upper_mm);
        let [removed_min, _] = s.volume();
        assert!(removed_min < std::f64::consts::PI * 400. * 50.);
    }

    #[test]
    fn facing_to_the_center_empties_the_end_and_boring_opens_the_bore() {
        let mut s = TurnStock::new(spec()).unwrap();
        let c = s
            .preview(&line([22., -0.2], [-0.5, -0.2]), 0.4, TurnSide::Outer, 1e-4)
            .unwrap();
        s.apply(&c).unwrap();
        // The last bin (z -0.5..0) is entirely above the nose's lowest z.
        let last = s.cells().last().unwrap();
        assert_eq!(last.outer.upper_mm, 0.);
        // Boring from inside: a tube's bore grows.
        let mut t = TurnStock::new(TurnStockSpec {
            bore_radius_mm: 5.,
            ..spec()
        })
        .unwrap();
        let c = t
            .preview(&line([6.2, 1.], [6.2, -20.]), 0.4, TurnSide::Inner, 1e-4)
            .unwrap();
        t.apply(&c).unwrap();
        let mid = t.cells()[60];
        assert!((mid.inner.lower_mm - 6.6).abs() < 1e-9 && (mid.inner.upper_mm - 6.6).abs() < 1e-9);
        assert_eq!(t.cells()[0].inner.upper_mm, 5.);
    }

    #[test]
    fn flat_drill_does_not_clear_the_uncut_part_of_a_bin() {
        // Exact cylindrical removal: a flat drill stops inside a bin, on a
        // bin boundary, and outside either end of both a bar and a tube.
        for bore in [0., 2.] {
            for tip in [-51., -50., -49.75, -25.25, -0.25, 0., 1.] {
                let mut stock = TurnStock::new(TurnStockSpec {
                    bore_radius_mm: bore,
                    ..spec()
                })
                .unwrap();
                stock.apply(&stock.drill(tip, 5., 0.)).unwrap();
                let length = (stock.spec.z_max_mm - tip.max(stock.spec.z_min_mm)).max(0.);
                let exact = std::f64::consts::PI
                    * ((400. - bore * bore) * 50. - (25. - bore * bore) * length);
                let [lower, upper] = stock.volume();
                assert!(
                    lower <= exact + 1e-8 && exact <= upper + 1e-8,
                    "tip={tip}, bore={bore}: {lower} <= {exact} <= {upper}"
                );
                for (i, cell) in stock.cells().iter().enumerate() {
                    let [a, b] = stock.bin(i);
                    if a < tip && tip < b {
                        assert_eq!(cell.inner.lower_mm, bore);
                        assert_eq!(cell.inner.upper_mm, 5.);
                    }
                }
            }
        }
    }

    #[test]
    fn conical_drill_volume_bounds_enclose_the_analytic_solid() {
        for bore in [0., 2.] {
            for radius in [1., 5., 20.] {
                for cone in [1e-12, 0.3, 3., 100.] {
                    for tip in [-55., -49.75, -25.25, -0.25, 1.] {
                        let mut stock = TurnStock::new(TurnStockSpec {
                            bore_radius_mm: bore,
                            ..spec()
                        })
                        .unwrap();
                        stock.apply(&stock.drill(tip, radius, cone)).unwrap();
                        // Integral of max(drill_radius(z)^2 - bore^2, 0):
                        // a truncated cone followed by a cylinder, not sampled.
                        let integral = |z: f64| {
                            if radius <= bore {
                                return 0.;
                            }
                            let start = cone * bore / radius;
                            let u = (z - tip).max(start);
                            let v = u.min(cone);
                            radius * radius / (3. * cone * cone) * (v.powi(3) - start.powi(3))
                                - bore * bore * (v - start)
                                + (u - cone).max(0.) * (radius * radius - bore * bore)
                        };
                        let exact = std::f64::consts::PI
                            * ((400. - bore * bore) * 50. - (integral(0.) - integral(-50.)));
                        let [lower, upper] = stock.volume();
                        assert!(
                            lower <= exact + 1e-7 && exact <= upper + 1e-7,
                            "bore={bore} radius={radius} cone={cone} tip={tip}: {lower} <= {exact} <= {upper}"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn invalid_stock_changes_are_refused_transactionally() {
        let initial = TurnStock::new(spec()).unwrap();
        let before = initial.cells()[0];
        let mut valid = before;
        valid.outer.lower_mm = 19.;
        valid.outer.upper_mm = 19.;
        let change = |index, after| TurnChange {
            index,
            before,
            after,
        };
        for bounds in [
            RadiusBounds {
                lower_mm: f64::NAN,
                upper_mm: 19.,
            },
            RadiusBounds {
                lower_mm: 0.,
                upper_mm: f64::NAN,
            },
            RadiusBounds {
                lower_mm: f64::NEG_INFINITY,
                upper_mm: 19.,
            },
            RadiusBounds {
                lower_mm: -1.,
                upper_mm: 19.,
            },
            RadiusBounds {
                lower_mm: 20.,
                upper_mm: 19.,
            },
        ] {
            let mut invalid = before;
            invalid.outer = bounds;
            let mut stock = initial.clone();
            assert!(
                stock
                    .apply(&[change(0, valid), change(1, invalid)])
                    .is_err(),
                "{bounds:?}"
            );
            assert_eq!(stock.cells(), initial.cells());
            assert_eq!(stock.generation(), 0);
        }
        for bounds in [
            RadiusBounds {
                lower_mm: f64::NAN,
                upper_mm: 5.,
            },
            RadiusBounds {
                lower_mm: 0.,
                upper_mm: f64::INFINITY,
            },
            RadiusBounds {
                lower_mm: 6.,
                upper_mm: 5.,
            },
            RadiusBounds {
                lower_mm: 0.,
                upper_mm: 21.,
            },
        ] {
            let mut invalid = before;
            invalid.inner = bounds;
            let mut stock = initial.clone();
            assert!(
                stock
                    .apply_move(
                        &[change(0, valid), change(1, invalid)],
                        &[[-10., -5., 2., 3.]]
                    )
                    .is_err(),
                "{bounds:?}"
            );
            assert_eq!(stock.cells(), initial.cells());
            assert!(stock.rings().is_empty());
            assert_eq!(stock.generation(), 0);
        }
        let mut stock = initial.clone();
        assert!(stock.apply(&[change(0, valid), change(0, before)]).is_err());
        assert_eq!(stock.cells(), initial.cells());
        assert_eq!(stock.generation(), 0);
    }

    #[test]
    fn unordered_move_changes_do_not_commit_cells_or_rings() {
        let mut initial = TurnStock::new(spec()).unwrap();
        initial.remove_ring([-20., -15.], [3., 4.]);
        let mut after = initial.cells()[0];
        after.outer = RadiusBounds {
            lower_mm: 19.,
            upper_mm: 19.,
        };
        let changes = |indices: &[usize]| {
            indices
                .iter()
                .map(|&index| TurnChange {
                    index,
                    before: initial.cells()[index],
                    after,
                })
                .collect::<Vec<_>>()
        };
        let rings = [[-10., -5., 2., 3.]];
        // Duplicate, descending, and nonadjacent repeated indices. All cells
        // individually match current stock and describe valid removal.
        for indices in [&[0, 0][..], &[1, 0][..], &[0, 1, 0][..]] {
            for with_rings in [false, true] {
                let mut stock = initial.clone();
                let changes = changes(indices);
                let result = if with_rings {
                    stock.apply_move(&changes, &rings)
                } else {
                    stock.apply(&changes)
                };
                assert_eq!(
                    result.unwrap_err(),
                    "turning stock changes must be in ascending cell order"
                );
                assert_eq!(stock.cells(), initial.cells());
                assert_eq!(stock.rings(), initial.rings());
                assert_eq!(stock.generation(), initial.generation());
                assert_eq!(stock.volume(), initial.volume());
            }
        }
        // A correctly ordered move still commits both cells and the new ring.
        let mut stock = initial.clone();
        stock.apply_move(&changes(&[0, 1]), &rings).unwrap();
        assert_eq!(&stock.cells()[..2], &[after, after]);
        assert_eq!(stock.rings(), &[initial.rings()[0], rings[0]]);
        assert!(stock.volume()[1] < initial.volume()[0]);
    }

    #[test]
    fn checkpoint_cannot_add_material_inside_the_original_bore() {
        let spec = TurnStockSpec {
            bore_radius_mm: 5.,
            ..spec()
        };
        let stock = TurnStock::new(spec.clone()).unwrap();
        let mut cells = stock.cells().to_vec();
        cells[0].inner.lower_mm = 0.;
        assert!(TurnStock::from_parts(spec, cells, vec![]).is_err());
    }

    #[test]
    fn checkpoint_rings_do_not_subtract_overlaps_twice() {
        let mut expected = TurnStock::new(spec()).unwrap();
        let rings = vec![[-10., -5., 2., 4.], [-8., -3., 3., 5.], [-10., -5., 2., 4.]];
        for r in &rings {
            expected.remove_ring([r[0], r[1]], [r[2], r[3]]);
        }
        let restored = TurnStock::from_parts(spec(), expected.cells().to_vec(), rings).unwrap();
        assert_eq!(restored.volume(), expected.volume());
        assert_eq!(restored.rings(), expected.rings());
        // Inclusion-exclusion: two annular cylinders minus their overlap.
        let exact = std::f64::consts::PI * (400. * 50. - (12. * 5. + 16. * 5. - 7. * 3.));
        for bound in restored.volume() {
            assert!((bound - exact).abs() < 1e-8);
        }
        let roundtrip =
            TurnStock::from_parts(spec(), restored.cells().to_vec(), restored.rings().to_vec())
                .unwrap();
        assert_eq!(roundtrip.rings(), restored.rings());
        assert_eq!(roundtrip.volume(), restored.volume());
    }

    #[test]
    fn small_multiturn_arcs_do_not_alias_to_a_stationary_nose() {
        let mut stock = TurnStock::new(TurnStockSpec {
            z_min_mm: -10.00001,
            z_max_mm: -9.99999,
            cell_mm: 0.00002,
            ..spec()
        })
        .unwrap();
        let radius: f64 = 0.0001;
        let rn: f64 = 0.00001;
        let arc = Curve::Arc {
            start: [15. + radius, 0., -10.],
            end: [15. + radius, 0., -10.],
            center: [15., 0., -10.],
            plane: Plane::Xz,
            sweep_rad: 10. * std::f64::consts::TAU,
        };
        stock
            .apply(&stock.preview(&arc, rn, TurnSide::Outer, 1e-4).unwrap())
            .unwrap();
        // Exact lower boundary of the annulus swept by a full circle.
        for (i, cell) in stock.cells().iter().enumerate() {
            for z in stock.bin(i) {
                let truth = 15. - ((radius + rn).powi(2) - (z + 10.).powi(2)).sqrt();
                assert!(cell.outer.lower_mm <= truth && truth <= cell.outer.upper_mm);
            }
        }
    }

    #[test]
    fn arc_chords_outside_a_small_nose_cannot_certify_removal() {
        for side in [TurnSide::Outer, TurnSide::Inner] {
            for rn in [0., 1e-6] {
                let sign = if side == TurnSide::Outer { 1. } else { -1. };
                let mut stock = TurnStock::new(TurnStockSpec {
                    z_min_mm: -13.001,
                    z_max_mm: -13.,
                    cell_mm: 0.001,
                    ..spec()
                })
                .unwrap();
                let arc = Curve::Arc {
                    start: [15. + sign * 5., 0., -10.],
                    end: [15., 0., -15.],
                    center: [15., 0., -10.],
                    plane: Plane::Xz,
                    sweep_rad: sign * std::f64::consts::FRAC_PI_2,
                };
                stock
                    .apply(&stock.preview(&arc, rn, side, 0.01).unwrap())
                    .unwrap();
                // Exact circular offset, away from both endpoint caps. A chord
                // lies on the material side of this boundary in both mirrors.
                for (i, cell) in stock.cells().iter().enumerate() {
                    for z in stock.bin(i) {
                        let radius = 15. + sign * ((5. - rn).powi(2) - (z + 10.).powi(2)).sqrt();
                        let bounds = if side == TurnSide::Outer {
                            cell.outer
                        } else {
                            cell.inner
                        };
                        assert!(
                            bounds.lower_mm <= radius && radius <= bounds.upper_mm,
                            "{side:?}, rn={rn}, z={z}: {bounds:?}, true radius {radius}"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn an_arc_profile_is_bounded_on_both_sides() {
        let mut s = TurnStock::new(spec()).unwrap();
        // Nose center along a quarter circle of radius 5 about (15, -10).
        let arc = Curve::Arc {
            start: [20., 0., -10.],
            end: [15., 0., -15.],
            center: [15., 0., -10.],
            plane: Plane::Xz,
            // From +Z toward +X: (z 0, x 5) at 90 degrees to (z -5, x 0) at 180.
            sweep_rad: std::f64::consts::FRAC_PI_2,
        };
        let c = s.preview(&arc, 0.5, TurnSide::Outer, 1e-4).unwrap();
        s.apply(&c).unwrap();
        for (i, cell) in s.cells().iter().enumerate() {
            let [za, zb] = s.bin(i);
            // The swept lower boundary at z: least r over densely sampled
            // nose positions along the arc.
            let truth = |z: f64| {
                (0..=20_000)
                    .filter_map(|k| {
                        let a = -std::f64::consts::FRAC_PI_2 * k as f64 / 20_000.;
                        let (r, zc) = (15. + 5. * a.cos(), -10. + 5. * a.sin());
                        let dz = z - zc;
                        (dz.abs() <= 0.5).then(|| r - (0.25 - dz * dz).sqrt())
                    })
                    .fold(f64::INFINITY, f64::min)
            };
            if za >= -14.5 && zb <= -10. {
                for z in [za, zb, (za + zb) / 2.] {
                    let t = truth(z).min(20.);
                    assert!(cell.outer.lower_mm <= t + 1e-3, "{z} {cell:?} {t}");
                    assert!(cell.outer.upper_mm >= t - 1e-3, "{z} {cell:?} {t}");
                }
            }
        }
    }

    #[test]
    fn a_small_arc_is_bounded_within_the_tolerance() {
        // The nose (0.5) round a quarter circle of radius 0.4: the chords
        // the arc is swept as stray from it by no more than the tolerance,
        // whatever the radius (#14: a nose rolling round a sharp corner).
        let tol = 1e-3;
        let mut s = TurnStock::new(TurnStockSpec {
            z_min_mm: -11.,
            z_max_mm: -9.,
            cell_mm: 0.02,
            ..spec()
        })
        .unwrap();
        let arc = Curve::Arc {
            start: [15.4, 0., -10.],
            end: [15., 0., -10.4],
            center: [15., 0., -10.],
            plane: Plane::Xz,
            sweep_rad: std::f64::consts::FRAC_PI_2,
        };
        let c = s.preview(&arc, 0.5, TurnSide::Outer, tol).unwrap();
        s.apply(&c).unwrap();
        let truth = |z: f64| {
            (0..=20_000)
                .filter_map(|k| {
                    let a = -std::f64::consts::FRAC_PI_2 * k as f64 / 20_000.;
                    let (r, zc) = (15. + 0.4 * a.cos(), -10. + 0.4 * a.sin());
                    let dz = z - zc;
                    (dz.abs() <= 0.5).then(|| r - (0.25 - dz * dz).sqrt())
                })
                .fold(f64::INFINITY, f64::min)
        };
        let mut checked = 0;
        for (i, cell) in s.cells().iter().enumerate() {
            let [za, zb] = s.bin(i);
            // Around the arc's end, where the swept boundary is flat: the
            // bounds differ from it by the chords' sagitta itself.
            if za < -10.45 || zb > -10.35 {
                continue;
            }
            let ends = [truth(za), truth(zb)];
            let (lo, hi) = (ends[0].min(ends[1]), ends[0].max(ends[1]));
            assert!(cell.outer.lower_mm <= lo + 1e-6, "{za} {cell:?} {lo}");
            assert!(cell.outer.lower_mm >= lo - tol - 1e-6, "{za} {cell:?} {lo}");
            assert!(cell.outer.upper_mm >= hi - 1e-6, "{za} {cell:?} {hi}");
            assert!(cell.outer.upper_mm <= hi + tol + 1e-6, "{za} {cell:?} {hi}");
            checked += 1;
        }
        assert!(checked > 3);
    }
}
