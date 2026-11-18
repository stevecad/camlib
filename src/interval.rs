//! Outward bounds for elementary binary64 geometry calculations.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Bounds {
    pub(crate) lo: f64,
    pub(crate) hi: f64,
}
impl Bounds {
    pub(crate) fn point(x: f64) -> Self {
        Self { lo: x, hi: x }
    }
    pub(crate) fn add(self, b: Self) -> Self {
        Self {
            lo: (self.lo + b.lo).next_down(),
            hi: (self.hi + b.hi).next_up(),
        }
    }
    pub(crate) fn sub(self, b: Self) -> Self {
        Self {
            lo: (self.lo - b.hi).next_down(),
            hi: (self.hi - b.lo).next_up(),
        }
    }
    pub(crate) fn mul(self, b: Self) -> Self {
        let products = [
            self.lo * b.lo,
            self.lo * b.hi,
            self.hi * b.lo,
            self.hi * b.hi,
        ];
        Self {
            lo: products
                .iter()
                .copied()
                .fold(f64::INFINITY, f64::min)
                .next_down(),
            hi: products
                .iter()
                .copied()
                .fold(f64::NEG_INFINITY, f64::max)
                .next_up(),
        }
    }
    pub(crate) fn div(self, b: Self) -> Self {
        if b.lo <= 0. && b.hi >= 0. {
            return Self {
                lo: f64::NEG_INFINITY,
                hi: f64::INFINITY,
            };
        }
        self.mul(Self {
            lo: (1. / b.hi).next_down(),
            hi: (1. / b.lo).next_up(),
        })
    }
    pub(crate) fn square(self) -> Self {
        let min = if self.lo <= 0. && self.hi >= 0. {
            0.
        } else {
            self.lo.abs().min(self.hi.abs())
        };
        let max = self.lo.abs().max(self.hi.abs());
        Self {
            lo: (min * min).next_down().max(0.),
            hi: (max * max).next_up(),
        }
    }
}
