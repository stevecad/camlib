//! Sparse component differences that retain analytic arcs. Unsupported or
//! ambiguous topology uses the unchanged general integer/chord Boolean.
use super::*;

struct Loops {
    outer: Polyline<f64>,
    holes: Vec<Polyline<f64>>,
}
impl Loops {
    fn boundaries(&self) -> impl Iterator<Item = &Polyline<f64>> {
        std::iter::once(&self.outer).chain(&self.holes)
    }
    fn contains(&self, p: Vector2<f64>) -> bool {
        self.outer.winding_number(p) != 0 && self.holes.iter().all(|h| h.winding_number(p) == 0)
    }
}
fn loops(region: &PreparedRegion, origin: Point) -> Option<Vec<Loops>> {
    let positive = |b: &Boundary| {
        let mut p = boundary_to_polyline(b, origin, region.epsilon).ok()?;
        if p.area() < 0. {
            p.invert_direction_mut();
        }
        Some(p)
    };
    region
        .definition
        .components
        .iter()
        .map(|c| {
            Some(Loops {
                outer: positive(&c.outer)?,
                holes: c.holes.iter().map(positive).collect::<Option<_>>()?,
            })
        })
        .collect()
}

// Each box encloses an ENTIRE minor arc (chord plus sagitta), not just samples.
// Subdivision refines only a topology guard. Exhaustion/near-contact declines the
// shortcut; it never supplies a positive separation certificate. The reserve
// covers the existing construction uncertainty and repeated floating operations.
fn separate_segments(
    a: PlineVertex<f64>,
    b: PlineVertex<f64>,
    c: PlineVertex<f64>,
    d: PlineVertex<f64>,
    reserve: f64,
    work: &mut usize,
    depth: usize,
) -> bool {
    if *work == 0 {
        return false;
    }
    *work -= 1;
    let x = seg_fast_approx_bounding_box(a, b);
    let y = seg_fast_approx_bounding_box(c, d);
    if x.max_x + reserve < y.min_x
        || y.max_x + reserve < x.min_x
        || x.max_y + reserve < y.min_y
        || y.max_y + reserve < x.min_y
    {
        return true;
    }
    if depth == 32 {
        return false;
    }
    let split = |u: PlineVertex<f64>, v: PlineVertex<f64>| {
        let m = seg_midpoint(u, v);
        let half = u.bulge / ((1. + u.bulge * u.bulge).sqrt() + 1.);
        (
            PlineVertex::new(u.x, u.y, half),
            PlineVertex::new(m.x, m.y, half),
        )
    };
    if (x.max_x - x.min_x).max(x.max_y - x.min_y) >= (y.max_x - y.min_x).max(y.max_y - y.min_y) {
        let (u, m) = split(a, b);
        separate_segments(u, m, c, d, reserve, work, depth + 1)
            && separate_segments(m, b, c, d, reserve, work, depth + 1)
    } else {
        let (u, m) = split(c, d);
        separate_segments(a, b, u, m, reserve, work, depth + 1)
            && separate_segments(a, b, m, d, reserve, work, depth + 1)
    }
}
fn separated(a: &Polyline<f64>, b: &Polyline<f64>, reserve: f64, work: &mut usize) -> bool {
    a.iter_segments().all(|(p, q)| {
        b.iter_segments()
            .all(|(r, s)| separate_segments(p, q, r, s, reserve, work, 0))
    })
}
fn disjoint(a: &Loops, b: &Loops, reserve: f64, work: &mut usize) -> bool {
    // Winding can reject separation cheaply, but never establishes it alone.
    !a.contains(b.outer.at(0).pos())
        && !b.contains(a.outer.at(0).pos())
        && a.boundaries()
            .all(|x| b.boundaries().all(|y| separated(x, y, reserve, work)))
}
fn inside(a: &Polyline<f64>, b: &Polyline<f64>, reserve: f64, work: &mut usize) -> bool {
    b.winding_number(a.at(0).pos()) != 0 && separated(a, b, reserve, work)
}
fn disjoint_disks(a: &Polyline<f64>, b: &Polyline<f64>, reserve: f64, work: &mut usize) -> bool {
    a.winding_number(b.at(0).pos()) == 0
        && b.winding_number(a.at(0).pos()) == 0
        && separated(a, b, reserve, work)
}
fn subtract(
    a: &Polyline<f64>,
    b: &Polyline<f64>,
    epsilon: f64,
    positives: &mut Vec<Polyline<f64>>,
    negatives: &mut Vec<Polyline<f64>>,
) -> Option<()> {
    // Retain the integer backend for entirely linear Boolean operands; the
    // Cavalier coincident-line path has different numerical behavior.
    if a.iter_vertexes()
        .chain(b.iter_vertexes())
        .all(|v| v.bulge == 0.)
    {
        return None;
    }
    let result = a.boolean_opt(
        b,
        BooleanOp::Not,
        &PlineBooleanOptions {
            pos_equal_eps: epsilon,
            ..Default::default()
        },
    );
    if matches!(result.result_info, BooleanResultInfo::InvalidInput) {
        return None;
    }
    positives.extend(result.pos_plines.into_iter().map(|p| p.pline));
    negatives.extend(result.neg_plines.into_iter().map(|p| p.pline));
    Some(())
}

pub(super) fn analytic(
    a: &PreparedRegion,
    b: &PreparedRegion,
    error: f64,
) -> Option<PreparedConstruction> {
    let left = loops(a, a.origin)?;
    let right = loops(b, a.origin)?;
    let reserve =
        error + 8192. * f64::EPSILON * a.origin.iter().map(|v| v.abs()).fold(1., f64::max);
    let mut work = 100_000;
    let mut matches = vec![None; left.len()];
    let mut used = vec![false; right.len()];
    for (i, x) in left.iter().enumerate() {
        for (j, y) in right.iter().enumerate() {
            if disjoint(x, y, reserve, &mut work) {
                continue;
            }
            if matches[i].is_some() || used[j] {
                return None;
            }
            matches[i] = Some(j);
            used[j] = true;
        }
    }
    let (mut positives, mut negatives) = (Vec::new(), Vec::new());
    for (i, x) in left.iter().enumerate() {
        let Some(j) = matches[i] else {
            positives.push(x.outer.clone());
            negatives.extend(x.holes.iter().cloned().map(|mut h| {
                h.invert_direction_mut();
                h
            }));
            continue;
        };
        let y = &right[j];
        // Each hole of x lies inside y's outer boundary (removed with it),
        // or, when y has no holes, wholly clear of it: then
        // (OA\H)\OB = (OA\OB)\H with H inside OA\OB, so it stays a hole.
        let mut kept = vec![];
        for h in &x.holes {
            if inside(h, &y.outer, reserve, &mut work) {
                continue;
            }
            if y.holes.is_empty() && disjoint_disks(h, &y.outer, reserve, &mut work) {
                let mut h = h.clone();
                h.invert_direction_mut();
                kept.push(h);
                continue;
            }
            return None;
        }
        if !y
            .holes
            .iter()
            .all(|h| inside(h, &x.outer, reserve, &mut work))
        {
            return None;
        }
        negatives.extend(kept);
        // Under the whole-boundary guards:
        // (OA\HA)\(OB\HB) = (OA\OB) union_j (HB_j\HA).
        subtract(
            &x.outer,
            &y.outer,
            a.epsilon,
            &mut positives,
            &mut negatives,
        )?;
        for hole in &y.holes {
            let mut partner = None;
            for other in &x.holes {
                if disjoint_disks(hole, other, reserve, &mut work) {
                    continue;
                }
                if partner.is_some() {
                    return None;
                }
                partner = Some(other);
            }
            if let Some(other) = partner {
                subtract(hole, other, a.epsilon, &mut positives, &mut negatives)?;
            } else {
                positives.push(hole.clone());
            }
        }
    }
    let components = group_loops(positives, negatives, a.origin).ok()?;
    // Final normalization and ordinary independent topology checks remain
    // authoritative, including any point-contact/numerical ambiguity.
    let mut result = a
        .finish(
            components,
            "cavalier_contours_0.9.0_sparse_difference",
            error,
            None,
            0.,
            0.,
        )
        .ok()?;
    result
        .report
        .operand_sources
        .push(b.definition.source.clone());
    Some(result)
}

#[cfg(test)]
mod tests;
