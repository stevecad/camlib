//! Find starts on a large layer using bounded fine-grid windows. Output
//! belongs to the core of each tile. Its halo supplies all material that
//! could influence either disk predicate inside the core.
use super::{CancelSignal, Layer, Near, cells2_within};

pub(super) fn candidates(
    layer: &Layer,
    radius: f64,
    cancel: &impl CancelSignal,
) -> Result<Vec<usize>, String> {
    const SIDE: usize = 256;
    let g = layer.g;
    let stride = layer.bits.stride;
    let blocked = cells2_within(g.c, radius + g.c, true).expect("positive blocking distance");
    let beside = cells2_within(g.c, radius + 3. * g.c, true).expect("positive reach distance");
    let halo = beside.isqrt();
    let mut starts = Vec::new();
    let mut row_starts: [Vec<usize>; SIDE] = std::array::from_fn(|_| Vec::new());
    for y in (0..g.ny).step_by(SIDE) {
        let bottom = (y + SIDE).min(g.ny);
        for x in (0..g.nx).step_by(SIDE) {
            if cancel.is_cancelled() {
                return Err("engagement_front planning cancelled".into());
            }
            let right = (x + SIDE).min(g.nx);
            let words = x / 64..right.div_ceil(64);
            // A start is allowed and its own cell must be free of material.
            if !(y..bottom).any(|j| {
                words.clone().any(|w| {
                    let k = j * stride + w;
                    layer.bits.allowed[k] & !layer.bits.any[k] != 0
                })
            }) {
                continue;
            }
            let i = [
                x.saturating_sub(halo),
                (right - 1).saturating_add(halo).min(g.nx - 1),
            ];
            let j = [
                y.saturating_sub(halo),
                (bottom - 1).saturating_add(halo).min(g.ny - 1),
            ];
            if !(j[0]..=j[1]).any(|row| {
                let a = row * stride + i[0] / 64;
                let b = row * stride + i[1] / 64;
                layer.bits.reach[a..=b].iter().any(|&word| word != 0)
            }) {
                continue;
            }
            let clear_of = Near::new(&layer.bits.any, stride, i, j, blocked, cancel)?;
            let close_to = Near::new(&layer.bits.reach, stride, i, j, beside, cancel)?;
            for row in y..bottom {
                for w in words.clone() {
                    let t = (row - j[0]) * close_to.words + w - close_to.first_word;
                    let mut bits = close_to.grown[t]
                        & !clear_of.grown[t]
                        & layer.bits.allowed[row * stride + w];
                    while bits != 0 {
                        let k = g.idx(w * 64 + bits.trailing_zeros() as usize, row);
                        bits &= bits - 1;
                        if !layer.dead[k] {
                            row_starts[row - y].push(k);
                        }
                    }
                }
            }
        }
        // Preserve global row-major tie breaking across adjacent tile cores.
        for row in &mut row_starts[..bottom - y] {
            starts.append(row);
        }
    }
    Ok(starts)
}
