//! Bounds on the first and second partial derivatives of B-spline and Bezier
//! surfaces, for meshing them with a proven interpolation error.
//!
//! A triangle's linear interpolant of a surface differs from it by at most a
//! second-derivative bound times the squared parameter extents (D. Filip,
//! R. Magedson and R. Markot, "Surface algorithms using bounds on
//! derivatives", Computer Aided Geometric Design 3 (1986) 295-311). Here the
//! derivative bounds come from control nets: a Bezier patch, and every
//! polynomial written in the Bernstein basis, lies in the convex hull of its
//! control net (G. Farin, "Curves and Surfaces for CAGD", 5th ed., sections
//! 5.3 and 14.4), so the largest norm in the net of a derivative bounds it.
//!
//! The caller supplies the surface as its Bezier patches (the CAD kernel's
//! conversion of a B-spline); nothing here evaluates a surface by sampling.

/// One rational Bezier patch: `(nu + 1) x (nv + 1)` poles, row-major in u,
/// in homogeneous form `[w x, w y, w z, w]` with every weight positive.
#[derive(Clone, Debug)]
pub struct BezierPatch {
    rows: usize,
    columns: usize,
    homogeneous: Vec<[f64; 4]>,
}

/// `[|Fu|, |Fv|, |Fuu|, |Fuv|, |Fvv|]`.
pub type Bounds = [f64; 5];

/// A tensor polynomial in the Bernstein basis with `N`-vector coefficients.
#[derive(Clone)]
struct Net<const N: usize> {
    rows: usize,
    columns: usize,
    data: Vec<[f64; N]>,
}

impl<const N: usize> Net<N> {
    fn zero(rows: usize, columns: usize) -> Self {
        Self {
            rows,
            columns,
            data: vec![[0.0; N]; rows * columns],
        }
    }

    /// The partial derivative along u (rows) over a parameter span: the
    /// hodograph's net, `degree * (b[i+1] - b[i]) / span`. A constant gives a
    /// net without rows, which every later operation treats as zero.
    fn du(&self, span: f64) -> Self {
        if self.rows < 2 {
            return Self::zero(0, self.columns);
        }
        let scale = (self.rows - 1) as f64 / span;
        let mut out = Self::zero(self.rows - 1, self.columns);
        for (index, cell) in out.data.iter_mut().enumerate() {
            let (low, high) = (self.data[index], self.data[index + self.columns]);
            *cell = std::array::from_fn(|n| scale * (high[n] - low[n]));
        }
        out
    }

    fn dv(&self, span: f64) -> Self {
        if self.columns < 2 || self.rows == 0 {
            return Self::zero(self.rows, 0);
        }
        let scale = (self.columns - 1) as f64 / span;
        let mut out = Self::zero(self.rows, self.columns - 1);
        for row in 0..self.rows {
            let source = &self.data[row * self.columns..(row + 1) * self.columns];
            let target = &mut out.data[row * (self.columns - 1)..(row + 1) * (self.columns - 1)];
            for (cell, pair) in target.iter_mut().zip(source.windows(2)) {
                *cell = std::array::from_fn(|n| scale * (pair[1][n] - pair[0][n]));
            }
        }
        out
    }

    fn is_zero(&self) -> bool {
        self.rows == 0 || self.columns == 0
    }

    /// The product with a scalar polynomial: in each parameter
    /// `c[k] = sum_{i+j=k} C(m,i) C(n,j) / C(m+n,k) a[i] b[j]`
    /// (Farin, section 6.10). Coefficients scaled by their binomials
    /// multiply as plain polynomials do.
    fn times(&self, scalar: &Net<1>) -> Self {
        if self.is_zero() || scalar.is_zero() {
            return Self::zero(0, 0);
        }
        let mut out = Self::zero(
            self.rows + scalar.rows - 1,
            self.columns + scalar.columns - 1,
        );
        let scaled = scalar.scaled();
        let (au, av) = (binomials(self.rows - 1), binomials(self.columns - 1));
        for i in 0..self.rows {
            for j in 0..self.columns {
                let a = self.data[i * self.columns + j];
                let weight = au[i] * av[j];
                for k in 0..scalar.rows {
                    let source = &scaled[k * scalar.columns..(k + 1) * scalar.columns];
                    let start = (i + k) * out.columns + j;
                    for (cell, b) in out.data[start..start + scalar.columns]
                        .iter_mut()
                        .zip(source)
                    {
                        let f = weight * b;
                        for n in 0..N {
                            cell[n] += f * a[n];
                        }
                    }
                }
            }
        }
        let (cu, cv) = (binomials(out.rows - 1), binomials(out.columns - 1));
        for (row, u) in out.data.chunks_exact_mut(out.columns).zip(cu) {
            for (cell, v) in row.iter_mut().zip(cv) {
                let f = 1.0 / (u * v);
                for x in cell.iter_mut() {
                    *x *= f;
                }
            }
        }
        out
    }

    /// `self + factor * other`; a zero net adds nothing.
    fn plus(mut self, factor: f64, other: &Self) -> Self {
        if other.is_zero() {
            return self;
        }
        if self.is_zero() {
            self = Self::zero(other.rows, other.columns);
        }
        assert_eq!(
            (self.rows, self.columns),
            (other.rows, other.columns),
            "Bezier derivative terms differ in degree"
        );
        for (cell, add) in self.data.iter_mut().zip(&other.data) {
            for n in 0..N {
                cell[n] += factor * add[n];
            }
        }
        self
    }

    /// The largest coefficient norm: a bound on the polynomial's norm.
    fn norm(&self) -> f64 {
        self.data
            .iter()
            .map(|cell| cell.iter().map(|x| x * x).sum::<f64>().sqrt())
            .fold(0.0, f64::max)
    }
}

impl Net<1> {
    /// The coefficients times their binomials `C(m,i) C(n,j)`.
    fn scaled(&self) -> Vec<f64> {
        let (u, v) = (binomials(self.rows - 1), binomials(self.columns - 1));
        let mut out = Vec::with_capacity(self.data.len());
        for (row, x) in self.data.chunks_exact(self.columns).zip(u) {
            out.extend(row.iter().zip(v).map(|(cell, y)| cell[0] * x * y));
        }
        out
    }
}

/// `C(degree, k)` for `k <= degree`. Products of nets reach three times a
/// patch's degree, which is at most 25.
fn binomials(degree: usize) -> &'static [f64] {
    static ROWS: std::sync::OnceLock<Vec<Vec<f64>>> = std::sync::OnceLock::new();
    let rows = ROWS.get_or_init(|| {
        (0..=3 * 25_usize)
            .map(|degree| {
                let mut row = vec![1.0_f64; degree + 1];
                for k in 1..=degree {
                    row[k] = row[k - 1] * (degree + 1 - k) as f64 / k as f64;
                }
                row
            })
            .collect()
    });
    &rows[degree]
}

impl BezierPatch {
    /// A patch from its poles and weights, row-major in u.
    pub fn new(
        rows: usize,
        columns: usize,
        poles: &[[f64; 3]],
        weights: &[f64],
    ) -> Result<Self, String> {
        if rows == 0
            || columns == 0
            || poles.len() != rows * columns
            || weights.len() != poles.len()
        {
            return Err("a Bezier patch needs rows x columns poles and weights".into());
        }
        if rows > 26 || columns > 26 {
            // The CAD kernel's own limit on a B-spline's degree is 25.
            return Err("a Bezier patch of degree above 25 is not supported".into());
        }
        if !weights.iter().all(|w| w.is_finite() && *w > 0.0)
            || !poles.iter().flatten().all(|x| x.is_finite())
        {
            return Err("a Bezier patch needs finite poles and positive weights".into());
        }
        Ok(Self {
            rows,
            columns,
            homogeneous: poles
                .iter()
                .zip(weights)
                .map(|(p, w)| [w * p[0], w * p[1], w * p[2], *w])
                .collect(),
        })
    }

    /// The two halves along u (`axis` 0) or v (1): de Casteljau's
    /// subdivision at the middle, in homogeneous coordinates, which is exact
    /// for a rational patch (Farin, section 13.2).
    fn halves(&self, axis: usize) -> (Self, Self) {
        let (count, lines, stride, step) = if axis == 0 {
            (self.rows, self.columns, self.columns, 1)
        } else {
            (self.columns, self.rows, 1, self.columns)
        };
        let mut low = self.clone();
        let mut high = self.clone();
        let mut work = vec![[0.0; 4]; count];
        for line in 0..lines {
            let at = |k: usize| line * step + k * stride;
            for (k, cell) in work.iter_mut().enumerate() {
                *cell = self.homogeneous[at(k)];
            }
            for level in 0..count {
                low.homogeneous[at(level)] = work[0];
                high.homogeneous[at(count - 1 - level)] = work[count - 1 - level];
                for k in 0..count - 1 - level {
                    let next = work[k + 1];
                    for (x, y) in work[k].iter_mut().zip(next) {
                        *x = 0.5 * (*x + y);
                    }
                }
            }
        }
        (low, high)
    }

    /// Bounds on the partial derivatives of the patch, which stands for the
    /// parameter spans `su`, `sv`.
    ///
    /// A polynomial patch's partials are Bezier patches whose nets are
    /// differences of its own (the hodographs), each bounded by its net. A
    /// rational patch is `F = A + N / W`, with `N` the patch of the poles
    /// `w (P - A)` about an anchor pole `A` and `W` the polynomial of the
    /// weights, and its partials are polynomials over powers of `W`:
    ///
    /// ```text
    /// F_u  = (N_u W - N W_u) / W^2
    /// F_uu = (N_uu W^2 - 2 N_u W_u W - N (W_uu W - 2 W_u^2)) / W^3
    /// F_uv = (N_uv W^2 - N_u W_v W - N_v W_u W - N (W_uv W - 2 W_u W_v)) / W^3
    /// ```
    ///
    /// Each numerator is written as one Bernstein polynomial (products of
    /// nets) and bounded by its net; `W` is at least the smallest weight.
    /// Bounding the terms one by one instead loses what they cancel: three
    /// to five times the real second derivative on a quadric written as a
    /// NURBS, and as many times the facets.
    pub fn bounds(&self, su: f64, sv: f64) -> Bounds {
        let weights = self.homogeneous.iter().map(|h| h[3]);
        let low = weights.clone().fold(f64::INFINITY, f64::min);
        let high = weights.fold(0.0, f64::max);
        let anchor: [f64; 3] =
            std::array::from_fn(|n| self.homogeneous[0][n] / self.homogeneous[0][3]);
        let n = Net::<3> {
            rows: self.rows,
            columns: self.columns,
            data: self
                .homogeneous
                .iter()
                .map(|h| std::array::from_fn(|k| h[k] - h[3] * anchor[k]))
                .collect(),
        };
        let (n_u, n_v) = (n.du(su), n.dv(sv));
        let (n_uu, n_uv, n_vv) = (n_u.du(su), n_u.dv(sv), n_v.dv(sv));
        if high - low <= 1e-14 * high {
            // Polynomial: N / w with one weight w.
            return [n_u, n_v, n_uu, n_uv, n_vv].map(|net| net.norm() / low);
        }
        let w = Net::<1> {
            rows: self.rows,
            columns: self.columns,
            data: self.homogeneous.iter().map(|h| [h[3]]).collect(),
        };
        let (w_u, w_v) = (w.du(su), w.dv(sv));
        let (w_uu, w_uv, w_vv) = (w_u.du(su), w_u.dv(sv), w_v.dv(sv));
        let (ww, w_wu, w_wv) = (w.times(&w), w_u.times(&w), w_v.times(&w));
        let first = |n_x: &Net<3>, w_x: &Net<1>| n_x.times(&w).plus(-1.0, &n.times(w_x)).norm();
        // N_xy W^2 - N_x (W_y W) - N_y (W_x W) - N (W_xy W - 2 W_x W_y)
        let second = |n_xy: &Net<3>,
                      (n_x, w_wy): (&Net<3>, &Net<1>),
                      (n_y, w_wx): (&Net<3>, &Net<1>),
                      tail: Net<1>| {
            n_xy.times(&ww)
                .plus(-1.0, &n_x.times(w_wy))
                .plus(-1.0, &n_y.times(w_wx))
                .plus(-1.0, &n.times(&tail))
                .norm()
        };
        let tail =
            |w_xy: &Net<1>, w_x: &Net<1>, w_y: &Net<1>| w_xy.times(&w).plus(-2.0, &w_x.times(w_y));
        [
            first(&n_u, &w_u) / low.powi(2),
            first(&n_v, &w_v) / low.powi(2),
            second(&n_uu, (&n_u, &w_wu), (&n_u, &w_wu), tail(&w_uu, &w_u, &w_u)) / low.powi(3),
            second(&n_uv, (&n_u, &w_wv), (&n_v, &w_wu), tail(&w_uv, &w_u, &w_v)) / low.powi(3),
            second(&n_vv, (&n_v, &w_wv), (&n_v, &w_wv), tail(&w_vv, &w_v, &w_v)) / low.powi(3),
        ]
    }
}

/// A piece of a Bezier patch in the bounds' tree.
struct Piece {
    /// `[u0, u1, v0, v1]`.
    domain: [f64; 4],
    bounds: Bounds,
    /// The patch, until the piece has been halved along both axes.
    patch: Option<BezierPatch>,
    /// Per axis: the two halves' indices once made, or that halving along it
    /// no longer tightens the second-derivative bounds.
    halves: [Halves; 2],
    depth: u8,
}

#[derive(Clone, Copy, PartialEq)]
enum Halves {
    Unknown,
    Settled,
    At(usize),
}

/// Halvings of one Bezier patch the bounds may descend, both axes together.
const MAX_DEPTH: u8 = 24;

/// A piece is not halved along an axis where neither half's bound on a
/// second derivative comes out below this share of its own: the net is as
/// close to the patch there as halving will get it.
const SETTLED: f64 = 0.97;

/// Derivative bounds of a B-spline or Bezier surface over any box of its
/// parameters.
///
/// A control net overestimates the derivatives of its patch, by less the
/// smaller the patch (the net converges to it quadratically under
/// subdivision; Farin, section 5.3). Whole-patch bounds ask for far more
/// facets than the surface needs, most of all where a patch runs into a pole
/// (the u derivatives vanish there; a net over the whole patch does not show
/// it). So each patch is a tree of its halves, made as they are asked for:
/// [`SplineBounds::local`] descends to pieces no larger than the box it is
/// asked about, and stops where halving no longer tightens the bounds.
pub struct SplineBounds {
    knots: [Vec<f64>; 2],
    /// Interior knots where the surface is only C0, per axis.
    kinks: [Vec<f64>; 2],
    periods: [Option<f64>; 2],
    pieces: Vec<Piece>,
    whole: Bounds,
}

impl SplineBounds {
    /// `patches` are the surface's Bezier patches, row-major in u, between
    /// consecutive `uknots` and `vknots`. `kinks` are the knots, per axis,
    /// where the surface is only C0; `periods` an axis's period where the
    /// surface is periodic (the patches are then one period).
    pub fn new(
        uknots: Vec<f64>,
        vknots: Vec<f64>,
        patches: Vec<BezierPatch>,
        kinks: [Vec<f64>; 2],
        periods: [Option<f64>; 2],
    ) -> Result<Self, String> {
        let increasing =
            |knots: &[f64]| knots.len() >= 2 && knots.windows(2).all(|pair| pair[0] < pair[1]);
        if !increasing(&uknots)
            || !increasing(&vknots)
            || !uknots.iter().chain(&vknots).all(|x| x.is_finite())
        {
            return Err("spline surface knots must be finite and increasing".into());
        }
        if patches.len() != (uknots.len() - 1) * (vknots.len() - 1) {
            return Err("spline surface needs one Bezier patch per knot span".into());
        }
        if periods
            .iter()
            .flatten()
            .any(|p| !(p.is_finite() && *p > 0.0))
        {
            return Err("spline surface period must be positive".into());
        }
        let columns = vknots.len() - 1;
        let mut pieces = Vec::with_capacity(patches.len());
        let mut whole = [0.0_f64; 5];
        for (index, patch) in patches.into_iter().enumerate() {
            let (i, j) = (index / columns, index % columns);
            let domain = [uknots[i], uknots[i + 1], vknots[j], vknots[j + 1]];
            let bounds = patch.bounds(domain[1] - domain[0], domain[3] - domain[2]);
            if !bounds.iter().all(|x| x.is_finite()) {
                return Err("nonfinite spline surface derivative bound".into());
            }
            for (total, bound) in whole.iter_mut().zip(bounds) {
                *total = total.max(bound);
            }
            pieces.push(Piece {
                domain,
                bounds,
                patch: Some(patch),
                halves: [Halves::Unknown; 2],
                depth: 0,
            });
        }
        Ok(Self {
            knots: [uknots, vknots],
            kinks,
            periods,
            pieces,
            whole,
        })
    }

    /// Bounds over the whole surface.
    pub fn whole(&self) -> Bounds {
        self.whole
    }

    /// The surface's parameter domain `[u0, u1, v0, v1]` (one period of a
    /// periodic axis).
    pub fn domain(&self) -> [f64; 4] {
        let [u, v] = &self.knots;
        [u[0], u[u.len() - 1], v[0], v[v.len() - 1]]
    }

    /// `(u, v)` moved into the domain; a periodic axis is left as it is.
    pub fn clamp(&self, uv: [f64; 2]) -> [f64; 2] {
        std::array::from_fn(|axis| {
            let knots = &self.knots[axis];
            if self.periods[axis].is_some() {
                uv[axis]
            } else {
                uv[axis].clamp(knots[0], knots[knots.len() - 1])
            }
        })
    }

    /// Pieces of the tree made so far (a measure of the work done).
    pub fn pieces(&self) -> usize {
        self.pieces.len()
    }

    /// The parts of `[low, high]` in the patches' own parameters, each with
    /// the range of patches it overlaps, and whether a C0 knot is strictly
    /// inside the interval.
    fn spans(&self, axis: usize, low: f64, high: f64) -> (Vec<(f64, f64, usize, usize)>, bool) {
        let knots = &self.knots[axis];
        let (first, last) = (knots[0], knots[knots.len() - 1]);
        let mut parts = vec![(low, high)];
        let mut kinked = false;
        if let Some(period) = self.periods[axis] {
            if high - low >= period {
                parts = vec![(first, last)];
                kinked = self.kinks[axis].contains(&first);
            } else {
                let shift = ((low - first) / period).floor() * period;
                let (low, high) = (low - shift, high - shift);
                if high <= last {
                    parts = vec![(low, high)];
                } else {
                    // Wraps across the seam, which is a knot like any other.
                    parts = vec![(low, last), (first, high - period)];
                    kinked = self.kinks[axis].contains(&first);
                }
            }
        }
        let spans = parts
            .into_iter()
            .map(|(low, high)| {
                kinked |= self.kinks[axis].iter().any(|k| low < *k && *k < high);
                let cells = knots.len() - 2;
                let start = knots
                    .partition_point(|k| *k <= low)
                    .saturating_sub(1)
                    .min(cells);
                let end = knots
                    .partition_point(|k| *k < high)
                    .saturating_sub(1)
                    .clamp(start, cells);
                (low, high, start, end + 1)
            })
            .collect();
        (spans, kinked)
    }

    /// The halves of a piece along an axis, made on first use; None where
    /// halving along it no longer tightens the second-derivative bounds
    /// ([`SETTLED`]) or the depth limit is reached.
    fn halves(&mut self, index: usize, axis: usize) -> Option<usize> {
        match self.pieces[index].halves[axis] {
            Halves::At(first) => return Some(first),
            Halves::Settled => return None,
            Halves::Unknown => {}
        }
        let piece = &self.pieces[index];
        let made = (piece.depth < MAX_DEPTH)
            .then_some(piece.patch.as_ref())
            .flatten()
            .map(|patch| {
                let (low, high) = patch.halves(axis);
                let [u0, u1, v0, v1] = piece.domain;
                let (um, vm) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
                let domains = if axis == 0 {
                    [[u0, um, v0, v1], [um, u1, v0, v1]]
                } else {
                    [[u0, u1, v0, vm], [u0, u1, vm, v1]]
                };
                [(low, domains[0]), (high, domains[1])].map(|(patch, domain)| Piece {
                    domain,
                    bounds: patch.bounds(domain[1] - domain[0], domain[3] - domain[2]),
                    patch: Some(patch),
                    halves: [Halves::Unknown; 2],
                    depth: piece.depth + 1,
                })
            });
        let parent = piece.bounds;
        let tighter = made.as_ref().is_some_and(|children| {
            children
                .iter()
                .all(|child| child.bounds.iter().all(|x| x.is_finite()))
                && children
                    .iter()
                    .any(|child| (2..5).any(|k| child.bounds[k] < SETTLED * parent[k]))
        });
        let Some(children) = made.filter(|_| tighter) else {
            let piece = &mut self.pieces[index];
            piece.halves[axis] = Halves::Settled;
            if piece.halves[1 - axis] != Halves::Unknown {
                piece.patch = None;
            }
            return None;
        };
        let first = self.pieces.len();
        self.pieces.extend(children);
        let piece = &mut self.pieces[index];
        piece.halves[axis] = Halves::At(first);
        if piece.halves[1 - axis] != Halves::Unknown {
            piece.patch = None;
        }
        Some(first)
    }

    /// Bounds over the box `u0 <= u <= u1`, `v0 <= v <= v1`, and whether a
    /// C0 knot line crosses it: then only the first two (the first
    /// derivatives) are bounds on anything.
    pub fn local(&mut self, u0: f64, u1: f64, v0: f64, v1: f64) -> (Bounds, bool) {
        self.local_within(u0, u1, v0, v1, f64::NEG_INFINITY)
    }

    /// [`SplineBounds::local`], from pieces only as small as it takes for
    /// [`interpolation_error`] over the box to be within `limit`: a facet
    /// that a coarse piece already proves needs no finer one. Every piece's
    /// bounds hold, so the result is a bound whatever the limit; pieces are
    /// halved a level at a time, so it depends only on the box and the limit.
    pub fn local_within(
        &mut self,
        u0: f64,
        u1: f64,
        v0: f64,
        v1: f64,
        limit: f64,
    ) -> (Bounds, bool) {
        let (us, kinked_u) = self.spans(0, u0, u1);
        let (vs, kinked_v) = self.spans(1, v0, v1);
        let kinked = kinked_u || kinked_v;
        let columns = self.knots[1].len() - 1;
        let mut front = Vec::new();
        for &(ulow, uhigh, i0, i1) in &us {
            for &(vlow, vhigh, j0, j1) in &vs {
                for i in i0..i1 {
                    for j in j0..j1 {
                        front.push((i * columns + j, [ulow, uhigh, vlow, vhigh]));
                    }
                }
            }
        }
        let mut next = Vec::new();
        loop {
            let mut found = [0.0_f64; 5];
            for &(index, _) in &front {
                for (total, bound) in found.iter_mut().zip(self.pieces[index].bounds) {
                    *total = total.max(bound);
                }
            }
            if interpolation_error(&found, kinked, u1 - u0, v1 - v0) <= limit {
                return (found, kinked);
            }
            let mut halved = false;
            next.clear();
            for &(index, query) in &front {
                let domain = self.pieces[index].domain;
                // Halve the axis along which the piece exceeds the box the most.
                let excess: [f64; 2] = std::array::from_fn(|axis| {
                    let own = domain[2 * axis + 1] - domain[2 * axis];
                    let asked = query[2 * axis + 1] - query[2 * axis];
                    if own > asked {
                        own / asked.max(f64::MIN_POSITIVE)
                    } else {
                        0.0
                    }
                });
                let order = if excess[0] >= excess[1] {
                    [0, 1]
                } else {
                    [1, 0]
                };
                let first = order
                    .into_iter()
                    .filter(|axis| excess[*axis] > 0.0)
                    .find_map(|axis| self.halves(index, axis));
                let Some(first) = first else {
                    next.push((index, query));
                    continue;
                };
                halved = true;
                for child in [first, first + 1] {
                    let [a, b, c, d] = self.pieces[child].domain;
                    let overlaps = |a: f64, b: f64, low: f64, high: f64| {
                        a.max(low) < b.min(high) || (low == high && a <= low && low <= b)
                    };
                    if overlaps(a, b, query[0], query[1]) && overlaps(c, d, query[2], query[3]) {
                        next.push((child, query));
                    }
                }
            }
            if !halved {
                return (found, kinked);
            }
            std::mem::swap(&mut front, &mut next);
        }
    }
}

/// The distance between a surface and the linear interpolant of a triangle
/// whose parameters span `du` by `dv`, from derivative bounds that hold on
/// it. Taylor's remainder with the barycentric weights cancels the first
/// derivative; the weights' variance over a span is at most a quarter of its
/// square (Filip, Magedson and Markot 1986, cited above). Across a C0 line
/// (`kinked`) only the first derivatives are bounded: the surface point is
/// within `|Fu| |du| + |Fv| |dv|` of each corner's, and the weights' mean
/// absolute deviation over a span is at most half of it.
pub fn interpolation_error(bounds: &Bounds, kinked: bool, du: f64, dv: f64) -> f64 {
    if kinked {
        0.5 * (bounds[0] * du + bounds[1] * dv)
    } else {
        (bounds[2] * du * du + 2.0 * bounds[3] * du * dv + bounds[4] * dv * dv) / 8.0
    }
}

#[cfg(test)]
mod tests;
