//! Only the best entry in each tile can win the whole-layer search. Removing
//! material makes stored maxima upper bounds. Rescan the winning tile if its
//! old winner is gone. Rollback and revived starts can raise a bound again.
use super::{CancelSignal, Layer};
use std::{cmp::Ordering, collections::BTreeSet};

const CELLS: usize = 1024;

#[derive(Clone, Copy, Debug)]
struct Entry {
    room: f64,
    cell: usize,
}

impl Ord for Entry {
    fn cmp(&self, rhs: &Self) -> Ordering {
        self.room
            .total_cmp(&rhs.room)
            .then(rhs.cell.cmp(&self.cell))
    }
}
impl PartialOrd for Entry {
    fn partial_cmp(&self, rhs: &Self) -> Option<Ordering> {
        Some(self.cmp(rhs))
    }
}
impl PartialEq for Entry {
    fn eq(&self, rhs: &Self) -> bool {
        self.cmp(rhs).is_eq()
    }
}
impl Eq for Entry {}

pub(super) struct EntryTiles {
    pub(super) need: f64,
    maxima: Vec<Option<Entry>>,
    ordered: BTreeSet<Entry>,
}

impl EntryTiles {
    pub(super) fn build(
        layer: &Layer,
        need: f64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        let count = layer.allowed.len().div_ceil(CELLS);
        let mut order = Self {
            need,
            maxima: vec![None; count],
            ordered: BTreeSet::new(),
        };
        for tile in 0..count {
            if cancel.is_cancelled() {
                return Err("engagement_front planning cancelled".into());
            }
            order.refresh(layer, tile);
        }
        Ok(order)
    }

    fn refresh(&mut self, layer: &Layer, tile: usize) {
        if let Some(old) = self.maxima[tile].take() {
            self.ordered.remove(&old);
        }
        let start = tile * CELLS;
        let room = layer.room();
        let best = (start..(start + CELLS).min(layer.allowed.len()))
            .filter(|&k| layer.allowed[k] && !layer.dead[k] && layer.material(k))
            .map(|cell| Entry {
                room: room[cell] as f64 - self.need,
                cell,
            })
            .max();
        self.maxima[tile] = best;
        if let Some(best) = best {
            self.ordered.insert(best);
        }
    }

    pub(super) fn best(
        &mut self,
        layer: &Layer,
        cancel: &impl CancelSignal,
    ) -> Result<Option<(f64, usize)>, String> {
        while let Some(&best) = self.ordered.last() {
            if cancel.is_cancelled() {
                return Err("engagement_front planning cancelled".into());
            }
            if !layer.dead[best.cell] && layer.material(best.cell) {
                return Ok(Some((best.room, best.cell)));
            }
            self.refresh(layer, best.cell / CELLS);
        }
        Ok(None)
    }

    pub(super) fn reopen(&mut self, layer: &Layer, cell: usize) {
        if !layer.allowed[cell] || layer.dead[cell] || !layer.material(cell) {
            return;
        }
        let candidate = Entry {
            room: layer.room()[cell] as f64 - self.need,
            cell,
        };
        let bound = &mut self.maxima[cell / CELLS];
        if bound.is_none_or(|old| candidate > old) {
            if let Some(old) = bound.replace(candidate) {
                self.ordered.remove(&old);
            }
            self.ordered.insert(candidate);
        }
    }
}
