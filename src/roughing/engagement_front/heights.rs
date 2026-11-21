//! Exact storage for fine-grid heights. A tile covers consecutive cells in
//! the shared row-major lattice. It has no geometric boundary of its own.
const CELLS: usize = 256;

enum Tile {
    Uniform(f64),
    Pair(Box<Pair>),
    Mixed(Box<[f64; CELLS]>),
}

struct Pair {
    first: f64,
    second: f64,
    second_bits: [u64; CELLS / 64],
}

impl Pair {
    fn at(&self, offset: usize) -> f64 {
        if self.second_bits[offset / 64] & (1 << (offset % 64)) == 0 {
            self.first
        } else {
            self.second
        }
    }
}

impl Tile {
    fn at(&self, offset: usize) -> f64 {
        match self {
            Self::Uniform(value) => *value,
            Self::Pair(pair) => pair.at(offset),
            Self::Mixed(values) => values[offset],
        }
    }

    fn set(&mut self, offset: usize, value: f64) {
        match self {
            Self::Uniform(first) if first.to_bits() == value.to_bits() => {}
            Self::Uniform(first) => {
                let mut second_bits = [0; CELLS / 64];
                second_bits[offset / 64] = 1 << (offset % 64);
                *self = Self::Pair(Box::new(Pair {
                    first: *first,
                    second: value,
                    second_bits,
                }));
            }
            Self::Pair(pair) if value.to_bits() == pair.first.to_bits() => {
                pair.second_bits[offset / 64] &= !(1 << (offset % 64));
                if pair.second_bits[offset / 64] == 0
                    && pair.second_bits.iter().all(|&word| word == 0)
                {
                    *self = Self::Uniform(pair.first);
                }
            }
            Self::Pair(pair) if value.to_bits() == pair.second.to_bits() => {
                pair.second_bits[offset / 64] |= 1 << (offset % 64);
                if pair.second_bits[offset / 64] == u64::MAX
                    && pair.second_bits.iter().all(|&word| word == u64::MAX)
                {
                    *self = Self::Uniform(pair.second);
                }
            }
            Self::Pair(pair) => {
                let mut cells = Box::new(std::array::from_fn(|i| pair.at(i)));
                cells[offset] = value;
                *self = Self::Mixed(cells);
            }
            Self::Mixed(cells) => cells[offset] = value,
        }
    }
}

pub(super) enum HeightsGrid {
    Dense(Vec<f64>),
    Tiled(TiledHeights),
}

pub(super) struct TiledHeights {
    tiles: Vec<Tile>,
}

impl HeightsGrid {
    pub(super) fn filled(n: usize, value: f64, tiled: bool) -> Self {
        if tiled {
            Self::Tiled(TiledHeights {
                tiles: (0..n.div_ceil(CELLS))
                    .map(|_| Tile::Uniform(value))
                    .collect(),
            })
        } else {
            Self::Dense(vec![value; n])
        }
    }

    pub(super) fn at(&self, k: usize) -> f64 {
        match self {
            Self::Dense(data) => data[k],
            Self::Tiled(data) => data.tiles[k / CELLS].at(k % CELLS),
        }
    }

    pub(super) fn set(&mut self, k: usize, value: f64) {
        match self {
            Self::Dense(data) => data[k] = value,
            Self::Tiled(data) => data.tiles[k / CELLS].set(k % CELLS, value),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sparse_and_mixed_tiles_retain_every_height_bit() {
        let count = CELLS * 2 + 13;
        let mut dense = vec![f64::INFINITY; count];
        let mut tiles = HeightsGrid::filled(count, f64::INFINITY, true);
        for value in [9., 7., -0., 0., f64::INFINITY] {
            for i in (1..count).step_by(3) {
                tiles.set(i, value);
                dense[i] = value;
            }
            for (i, &expected) in dense.iter().enumerate() {
                assert_eq!(tiles.at(i).to_bits(), expected.to_bits());
            }
        }
    }
}
