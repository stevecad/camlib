//! Cavalier's offset algorithm uses its scalar's trigonometry to construct
//! and split bulge arcs. Use the same binary64 operations on every host,
//! including inside that algorithm, before the coordinates become toolpaths.
use super::*;
use cavalier_contours::{
    core::traits::{FuzzyEq, FuzzyOrd, Real},
    static_aabb2d_index::IndexableNum,
};
use num_traits::{Bounded, Num, NumCast, One, ToPrimitive, Zero};
use std::{cmp::Ordering, ops};

#[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd)]
struct Coord(f64);

impl Real for Coord {}
impl FuzzyEq for Coord {
    fn fuzzy_epsilon() -> Self {
        Self(f64::fuzzy_epsilon())
    }
    fn fuzzy_eq_eps(&self, other: Self, eps: Self) -> bool {
        self.0.fuzzy_eq_eps(other.0, eps.0)
    }
    fn fuzzy_eq_zero_eps(&self, eps: Self) -> bool {
        self.0.fuzzy_eq_zero_eps(eps.0)
    }
}
impl FuzzyOrd for Coord {
    fn fuzzy_gt_eps(&self, other: Self, eps: Self) -> bool {
        self.0.fuzzy_gt_eps(other.0, eps.0)
    }
    fn fuzzy_lt_eps(&self, other: Self, eps: Self) -> bool {
        self.0.fuzzy_lt_eps(other.0, eps.0)
    }
}
impl IndexableNum for Coord {
    fn total_cmp(&self, other: &Self) -> Ordering {
        self.0.total_cmp(&other.0)
    }
}
impl Bounded for Coord {
    fn min_value() -> Self {
        Self(f64::MIN)
    }
    fn max_value() -> Self {
        Self(f64::MAX)
    }
}
impl ToPrimitive for Coord {
    fn to_i64(&self) -> Option<i64> {
        self.0.to_i64()
    }
    fn to_u64(&self) -> Option<u64> {
        self.0.to_u64()
    }
    fn to_f64(&self) -> Option<f64> {
        Some(self.0)
    }
    fn to_f32(&self) -> Option<f32> {
        self.0.to_f32()
    }
}
impl NumCast for Coord {
    fn from<T: ToPrimitive>(n: T) -> Option<Self> {
        n.to_f64().map(Self)
    }
}
impl Zero for Coord {
    fn zero() -> Self {
        Self(0.)
    }
    fn is_zero(&self) -> bool {
        self.0 == 0.
    }
}
impl One for Coord {
    fn one() -> Self {
        Self(1.)
    }
}
impl Num for Coord {
    type FromStrRadixErr = <f64 as Num>::FromStrRadixErr;
    fn from_str_radix(s: &str, radix: u32) -> Result<Self, Self::FromStrRadixErr> {
        f64::from_str_radix(s, radix).map(Self)
    }
}
impl ops::Neg for Coord {
    type Output = Self;
    fn neg(self) -> Self {
        Self(-self.0)
    }
}
macro_rules! binary_op {
    ($trait:ident, $method:ident, $op:tt) => {
        impl ops::$trait for Coord {
            type Output = Self;
            fn $method(self, rhs: Self) -> Self { Self(self.0 $op rhs.0) }
        }
    };
}
binary_op!(Add, add, +);
binary_op!(Sub, sub, -);
binary_op!(Mul, mul, *);
binary_op!(Div, div, /);
binary_op!(Rem, rem, %);

macro_rules! unary {
    ($($method:ident => $implementation:path),+ $(,)?) => {
        $(fn $method(self) -> Self { Self($implementation(self.0)) })+
    };
}
impl num_traits::real::Real for Coord {
    fn min_value() -> Self {
        Self(f64::MIN)
    }
    fn min_positive_value() -> Self {
        Self(f64::MIN_POSITIVE)
    }
    fn epsilon() -> Self {
        Self(f64::EPSILON)
    }
    fn max_value() -> Self {
        Self(f64::MAX)
    }
    unary! {
        floor => f64::floor, ceil => f64::ceil, round => f64::round, trunc => f64::trunc,
        fract => f64::fract, abs => f64::abs, signum => f64::signum, recip => f64::recip,
        sqrt => f64::sqrt, to_degrees => f64::to_degrees, to_radians => f64::to_radians,
        exp => metallic::exp, exp2 => metallic::exp2, ln => metallic::log,
        log2 => metallic::log2, log10 => metallic::log10, cbrt => metallic::cbrt,
        sin => metallic::sin, cos => metallic::cos, tan => metallic::tan,
        asin => metallic::asin, acos => metallic::acos, atan => metallic::atan,
        exp_m1 => metallic::expm1, ln_1p => metallic::log1p, sinh => metallic::sinh,
        cosh => metallic::cosh, tanh => metallic::tanh, asinh => metallic::asinh,
        acosh => metallic::acosh, atanh => metallic::atanh,
    }
    fn is_sign_positive(self) -> bool {
        self.0.is_sign_positive()
    }
    fn is_sign_negative(self) -> bool {
        self.0.is_sign_negative()
    }
    fn mul_add(self, a: Self, b: Self) -> Self {
        Self(self.0.mul_add(a.0, b.0))
    }
    fn powi(self, n: i32) -> Self {
        Self(self.0.powi(n))
    }
    fn powf(self, n: Self) -> Self {
        Self(metallic::pow(self.0, n.0))
    }
    fn log(self, base: Self) -> Self {
        Self(metallic::log(self.0) / metallic::log(base.0))
    }
    fn max(self, other: Self) -> Self {
        Self(self.0.max(other.0))
    }
    fn min(self, other: Self) -> Self {
        Self(self.0.min(other.0))
    }
    fn abs_sub(self, other: Self) -> Self {
        Self(num_traits::real::Real::abs_sub(self.0, other.0))
    }
    fn hypot(self, other: Self) -> Self {
        Self(metallic::hypot(self.0, other.0))
    }
    fn atan2(self, other: Self) -> Self {
        Self(metallic::atan2(self.0, other.0))
    }
    fn sin_cos(self) -> (Self, Self) {
        let (s, c) = metallic::sincos(self.0);
        (Self(s), Self(c))
    }
}

fn convert_polyline<A: Real, B: Real>(p: &Polyline<A>, convert: impl Fn(A) -> B) -> Polyline<B> {
    Polyline {
        vertex_data: p
            .iter_vertexes()
            .map(|v| PlineVertex::new(convert(v.x), convert(v.y), convert(v.bulge)))
            .collect(),
        is_closed: p.is_closed,
        userdata: p.userdata.clone(),
    }
}

pub(super) fn offset_components(
    shape: &Shape<f64>,
    delta: f64,
    eps: f64,
    origin: Point,
) -> Result<Vec<Component>, String> {
    let loops = shape
        .ccw_plines
        .iter()
        .chain(&shape.cw_plines)
        .map(|p| convert_polyline(&p.polyline, Coord));
    let output = Shape::from_plines(loops).parallel_offset(
        Coord(delta),
        &ShapeOffsetOptions {
            pos_equal_eps: Coord(eps),
            offset_dist_eps: Coord(4. * eps),
            slice_join_eps: Coord(4. * eps),
        },
    );
    let unwrap = |p: cavalier_contours::shape_algorithms::IndexedPolyline<Coord>| {
        convert_polyline(&p.polyline, |v| v.0)
    };
    group_loops(
        output.ccw_plines.into_iter().map(unwrap).collect(),
        output.cw_plines.into_iter().map(unwrap).collect(),
        origin,
    )
}
