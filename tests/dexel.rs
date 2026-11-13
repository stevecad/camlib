//! Tri-dexel stock (issue #1): removal bounds checked against an independent
//! sampled sweep of the analytic tool, the height-field kernel's results on
//! three-axis moves, and an analytic T-slot.
use camlib::{dexel::*, motion::*, stock::*, tool::*};
use std::f64::consts::PI;

fn spec(min: Vec3, max: Vec3, cell: f64) -> DexelStockSpec {
    DexelStockSpec {
        version: 1,
        units: "mm".into(),
        model: DexelModel::TriDexel,
        min_mm: min,
        max_mm: max,
        cell_size_mm: cell,
        max_rays: 10_000_000,
        max_intervals: 100_000_000,
    }
}
fn height_spec(s: &DexelStockSpec) -> StockSpec {
    s.view_spec().unwrap()
}
fn cutter(shape: CutterShape, diameter: f64, cutting: f64) -> Cutter {
    Cutter {
        id: "cutter".into(),
        name: "test".into(),
        shape,
        diameter_mm: diameter,
        cutting_length_mm: cutting,
        overall_length_mm: 60.,
        shank_diameter_mm: diameter,
        flutes: 2,
        center_cutting: true,
        material: "carbide".into(),
        neck: None,
    }
}
fn assembly(id: &str, cutter: Cutter) -> PreparedAssembly {
    let r = cutter.shank_diameter_mm.max(cutter.diameter_mm) / 2. + 1.;
    ToolAssembly {
        id: id.into(),
        cutter,
        stickout_mm: 40.,
        gauge_length_mm: 70.,
        holders: vec![HolderSection {
            z_min_mm: 40.,
            z_max_mm: 70.,
            radius_min_mm: r,
            radius_max_mm: r,
        }],
    }
    .prepare()
    .unwrap()
}
fn cut(id: u64, tool: &str, curve: Curve) -> Motion {
    Motion {
        id,
        operation: "op".into(),
        tool: tool.into(),
        kind: MoveKind::Cut,
        curve,
        feed_mm_min: Some(500.),
    }
}
fn line(id: u64, a: Vec3, b: Vec3) -> Motion {
    cut(id, "t", Curve::Line { start: a, end: b })
}
fn sweep(stock: &mut DexelStock, tool: &PreparedAssembly, m: &Motion) -> DexelDelta {
    let d = stock
        .preview_sweep(tool, m, 1., SweepOptions::default(), &|| false)
        .unwrap();
    stock.apply_delta(&d).unwrap();
    d
}

/// Whether `q` lies inside the cutting part at tip `p` (tool axis +Z), with
/// the tool grown (`grow` > 0) or shrunk (`grow` < 0) by |grow|.
fn inside(tool: &PreparedAssembly, p: Vec3, q: Vec3, grow: f64) -> bool {
    let c = tool.cutter();
    let lc = c.definition().cutting_length_mm;
    let h = q[2] - p[2];
    let rho = (q[0] - p[0]).hypot(q[1] - p[1]);
    if grow >= 0. {
        if h < -grow || h > lc + grow {
            return false;
        }
        let r = c.radius_at((h + grow).clamp(0., lc)).unwrap().radius_mm;
        rho <= r + grow
    } else {
        let g = -grow;
        if h < g || h > lc - g {
            return false;
        }
        let r = c.radius_at(h - g).unwrap().radius_mm;
        rho + g <= r
    }
}
/// Poses along the moves, at most `step` apart.
fn poses(moves: &[Motion], step: f64) -> Vec<Vec3> {
    let mut out = vec![];
    for m in moves {
        let p = m.curve.prepare(1e-5).unwrap();
        let length = {
            let (a, b) = (m.curve.start(), m.curve.end());
            let chord =
                ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt();
            match m.curve {
                Curve::Line { .. } => chord,
                Curve::Arc {
                    sweep_rad,
                    center,
                    plane,
                    start,
                    ..
                }
                | Curve::Helix {
                    sweep_rad,
                    center,
                    plane,
                    start,
                    ..
                } => {
                    let [u, v, w] = plane.axes();
                    let r = (start[u] - center[u]).hypot(start[v] - center[v]);
                    (r * sweep_rad.abs()).hypot(b[w] - a[w])
                }
            }
        };
        let n = (length / step).ceil().max(1.) as usize;
        for i in 0..=n {
            out.push(p.position(i as f64 / n as f64).unwrap());
        }
    }
    out
}

struct Lcg(u64);
impl Lcg {
    fn next(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// Every certain interval holds only material the sampled sweep did not
/// remove (by more than `tol`), and every point outside the possible
/// intervals was removed (within `tol`), at random points of each ray's
/// prism.
fn assert_sound(stock: &DexelStock, tool: &PreparedAssembly, moves: &[Motion], tol: f64) {
    let all = poses(moves, tol / 2.);
    let spec = stock.spec().clone();
    let grid = stock.grid();
    let mut rng = Lcg(7);
    let mut checked = [0usize; 2];
    for a in 0..3 {
        let [u, v] = DexelGrid::cross(a);
        for (index, ray) in stock.rays(a).iter().enumerate() {
            let c = stock.ray_center(a, index);
            for _ in 0..3 {
                let mut q = [0.; 3];
                q[u] = c[0] + (rng.next() - 0.5) * grid.step_mm[u];
                q[v] = c[1] + (rng.next() - 0.5) * grid.step_mm[v];
                q[a] = spec.min_mm[a] + rng.next() * (spec.max_mm[a] - spec.min_mm[a]);
                let near: Vec<&Vec3> = all
                    .iter()
                    .filter(|p| (p[0] - q[0]).hypot(p[1] - q[1]) < 10.)
                    .collect();
                let t = q[a];
                let within = |s: &Spans| s.as_slice().iter().any(|i| i[0] <= t && t <= i[1]);
                if within(&ray.certain) {
                    checked[0] += 1;
                    assert!(
                        !near.iter().any(|p| inside(tool, **p, q, -tol)),
                        "grid {a} ray {index}: certain material at {q:?} was removed"
                    );
                }
                if !within(&ray.possible) {
                    checked[1] += 1;
                    assert!(
                        near.iter().any(|p| inside(tool, **p, q, tol)),
                        "grid {a} ray {index}: {q:?} outside possible material was never cut"
                    );
                }
            }
        }
    }
    assert!(checked[0] > 100 && checked[1] > 100, "{checked:?}");
}

fn shapes() -> Vec<(&'static str, CutterShape)> {
    vec![
        ("flat", CutterShape::Flat),
        ("ball", CutterShape::Ball),
        (
            "bull",
            CutterShape::Bull {
                corner_radius_mm: 0.5,
            },
        ),
        (
            "drill",
            CutterShape::Drill {
                point_angle_rad: 118f64.to_radians(),
            },
        ),
        (
            "chamfer",
            CutterShape::Chamfer {
                included_angle_rad: PI / 2.,
                tip_diameter_mm: 0.5,
            },
        ),
    ]
}

#[test]
fn removal_bounds_enclose_the_sampled_sweep() {
    for (name, shape) in shapes() {
        let tool = assembly("t", cutter(shape, 3., 12.));
        let mut stock = DexelStock::new(spec([0., 0., 0.], [8., 8., 5.], 0.3)).unwrap();
        let moves = vec![
            line(1, [-2., 2.1, 3.], [10., 2.1, 3.]),
            line(2, [1., -1., 4.2], [7., 9., 2.]),
            cut(
                3,
                "t",
                Curve::Arc {
                    start: [6., 4., 3.5],
                    end: [2., 4., 3.5],
                    center: [4., 4., 3.5],
                    plane: Plane::Xy,
                    sweep_rad: PI,
                },
            ),
            cut(
                4,
                "t",
                Curve::Helix {
                    start: [5.5, 6., 4.5],
                    end: [3.5, 6., 1.5],
                    center: [4.5, 6., 4.5],
                    plane: Plane::Xy,
                    sweep_rad: -3. * PI,
                },
            ),
            line(5, [6.5, 1.5, 6.], [6.5, 1.5, 1.]),
        ];
        for m in &moves {
            sweep(&mut stock, &tool, m);
        }
        let v = stock.volume();
        assert!(v.remaining_lower_mm3 <= v.remaining_upper_mm3, "{name}");
        assert!(v.removed_lower_mm3 > 10., "{name}: {v:?}");
        assert_sound(&stock, &tool, &moves, 0.01);
    }
}

/// Each `level` move's XY distance from `p` to its path, with its height.
fn flat_reach(p: [f64; 2]) -> [(f64, f64); 3] {
    let segment = |a: [f64; 2], b: [f64; 2]| {
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let t = (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)).clamp(0., 1.);
        (p[0] - a[0] - t * dx).hypot(p[1] - a[1] - t * dy)
    };
    [
        (segment([-2., 3.], [12., 3.]), 5.),
        (segment([-2., 6.], [12., 6.5]), 4.),
        (((p[0] - 5.).hypot(p[1] - 5.) - 3.).abs(), 4.5),
    ]
}

/// On three-axis moves the height view bounds the same material as the
/// height field the existing kernel computes: identical lower bounds for a
/// flat end mill at constant height (both use the exact distance to the
/// path; the height field may prove a tighter upper bound in cells no single
/// whole-cell test covers), and
/// overlapping elsewhere (the dexel columns skip cells the chord's padded
/// box cannot reach, where the height kernel pads by the cell's circle).
#[test]
fn height_view_matches_the_height_field() {
    for (name, shape) in shapes() {
        let tool = assembly("t", cutter(shape, 3., 12.));
        let s = spec([0., 0., 0.], [10., 10., 6.], 0.2);
        let level = [
            line(1, [-2., 3., 5.], [12., 3., 5.]),
            line(2, [-2., 6., 4.], [12., 6.5, 4.]),
            cut(
                3,
                "t",
                Curve::Arc {
                    start: [8., 5., 4.5],
                    end: [8., 5., 4.5],
                    center: [5., 5., 4.5],
                    plane: Plane::Xy,
                    sweep_rad: 2. * PI,
                },
            ),
        ];
        let ramp = [line(4, [2., -2., 5.5], [2., 12., 3.])];
        for (moves, exact) in [(&level[..], name == "flat"), (&ramp[..], false)] {
            let mut dexel = DexelStock::new(s.clone()).unwrap();
            let mut height = Stock::new(height_spec(&s)).unwrap();
            for m in moves {
                sweep(&mut dexel, &tool, m);
                let d = height
                    .preview_sweep(&tool, m, 1., SweepOptions::default(), &|| false)
                    .unwrap();
                height.apply_delta(&d).unwrap();
            }
            for (i, (a, b)) in dexel.view().cells().iter().zip(height.cells()).enumerate() {
                assert!(
                    a.lower_mm <= b.upper_mm + 1e-9 && b.lower_mm <= a.upper_mm + 1e-9,
                    "{name}: cell {i} bounds do not overlap: {a:?} {b:?}"
                );
                if exact {
                    // Lower bounds are always identical. An upper bound is
                    // identical too, unless the union rule lowered the cell:
                    // it proves a partly covered cell cleared by splitting
                    // it (`Stock::union_covered`). Then the height field's
                    // bound is tighter than the dexel's, but never below the
                    // true top. Both are checked against the moves
                    // themselves: the highest surface left over any point of
                    // the cell.
                    assert_eq!(a.lower_mm, b.lower_mm, "{name}: cell {i}");
                    let [cx, cy] = height.cell_center(i).unwrap();
                    let half = s.cell_size_mm / 2.;
                    let mut top = f64::MIN;
                    for u in 0..=6 {
                        for v in 0..=6 {
                            let p = [
                                cx - half + s.cell_size_mm * u as f64 / 6.,
                                cy - half + s.cell_size_mm * v as f64 / 6.,
                            ];
                            let mut z = s.max_mm[2];
                            for (d, height) in flat_reach(p) {
                                if d <= 1.5 {
                                    z = z.min(height);
                                }
                            }
                            top = top.max(z);
                        }
                    }
                    assert!(b.upper_mm >= top - 1e-9, "{name}: cell {i}: {b:?} top {top}");
                    if b.upper_mm != a.upper_mm {
                        assert!(b.upper_mm < a.upper_mm, "{name}: cell {i}: {a:?} {b:?}");
                        // A lowered cell is covered whole by the moves, if
                        // by no single one of them (a move's own test is
                        // conservative at a curved edge).
                        assert!(top < s.max_mm[2], "{name}: cell {i}: {a:?} {b:?}");
                    }
                }
            }
            let (h, d) = (height.volume(), dexel.volume());
            assert!(d.remaining_lower_mm3 <= h.remaining_upper_mm3 + 1e-6);
            assert!(h.remaining_lower_mm3 <= d.remaining_upper_mm3 + 1e-6);
        }
    }
}

fn t_slot_cutter(neck: f64) -> PreparedAssembly {
    let mut c = cutter(CutterShape::Flat, 10., 2.);
    c.shank_diameter_mm = 6.;
    c.neck = Some(Neck {
        diameter_mm: neck,
        length_mm: 10.,
    });
    ToolAssembly {
        id: "tslot".into(),
        cutter: c,
        stickout_mm: 20.,
        gauge_length_mm: 60.,
        holders: vec![HolderSection {
            z_min_mm: 20.,
            z_max_mm: 60.,
            radius_min_mm: 8.,
            radius_max_mm: 8.,
        }],
    }
    .prepare()
    .unwrap()
}
fn slotted_block(cell: f64) -> DexelStock {
    // A 30 x 20 x 10 block with a 4 mm slot 7 mm deep along X at Y 10: the
    // T-slot's full depth, as T-slots are cut (the T-slot cutter then
    // widens its bottom 2 mm).
    let mut stock = DexelStock::new(spec([0., 0., 0.], [30., 20., 10.], cell)).unwrap();
    let mill = assembly("t", cutter(CutterShape::Flat, 4., 15.));
    sweep(&mut stock, &mill, &line(1, [-5., 10., 3.], [35., 10., 3.]));
    stock
}

/// Acceptance: a T-slot cut under an existing slot leaves an undercut the
/// stock represents within tolerance, and its volume bounds enclose the
/// analytic solid.
#[test]
fn t_slot_undercut_is_represented() {
    let mut stock = slotted_block(0.1);
    let tool = t_slot_cutter(3.);
    let mut t = cut(
        2,
        "tslot",
        Curve::Line {
            start: [-8., 10., 3.],
            end: [38., 10., 3.],
        },
    );
    t.operation = "tslot".into();
    let d = sweep(&mut stock, &tool, &t);
    assert!(
        d.stats().undercut_columns > 0,
        "the disk ran under material"
    );
    // Block 6000, slot 30 x 4 x 7 = 840, T wings 30 x 6 x 2 = 360.
    let exact = 6000. - 840. - 360.;
    let v = stock.volume();
    assert!(
        v.remaining_lower_mm3 <= exact && exact <= v.remaining_upper_mm3,
        "{v:?}"
    );
    assert!(
        v.remaining_upper_mm3 - v.remaining_lower_mm3 < 0.02 * exact,
        "{v:?}"
    );
    // A ray along Y through the T band: material on both sides, the
    // undercut's walls at Y 5 and 15 within the sweep tolerance.
    let grid = stock.grid();
    let ray = |a: usize, p: [f64; 2]| {
        let [u, v] = DexelGrid::cross(a);
        let iu = (p[0] / grid.step_mm[u]).floor() as usize;
        let iv = (p[1] / grid.step_mm[v]).floor() as usize;
        stock.rays(a)[iv * grid.cells[u] + iu].clone()
    };
    // Y rays: cross axes (Z, X).
    let r = ray(1, [4.05, 15.05]);
    for s in [&r.possible, &r.certain] {
        let s = s.as_slice();
        assert_eq!(s.len(), 2, "{r:?}");
        assert!((s[0][0] - 0.).abs() < 1e-12 && (s[1][1] - 20.).abs() < 1e-12);
        assert!(
            (s[0][1] - 5.).abs() < 1e-4 && (s[1][0] - 15.).abs() < 1e-4,
            "{r:?}"
        );
    }
    // A Z column under the overhang: material below and above the T.
    let z = ray(2, [6.05, 6.05]);
    let p = z.possible.as_slice();
    assert_eq!(p.len(), 2, "{z:?}");
    assert!(
        (p[0][1] - 3.).abs() < 1e-4 && (p[1][0] - 5.).abs() < 1e-4,
        "{z:?}"
    );
    // The height view cannot hold the undercut: it keeps the column's top
    // and the certain material below the T.
    let cell = stock.view().cells()[60 * grid.cells[0] + 60];
    assert!((cell.upper_mm - 10.).abs() < 1e-12 && (cell.lower_mm - 3.).abs() < 1e-4);
}

/// A neck wider than the slot would rub its walls: refused, nothing changes.
#[test]
fn t_slot_neck_touching_the_slot_is_refused() {
    let stock = slotted_block(0.1);
    let tool = t_slot_cutter(4.2);
    let mut t = cut(
        2,
        "tslot",
        Curve::Line {
            start: [-8., 10., 3.],
            end: [38., 10., 3.],
        },
    );
    t.operation = "tslot".into();
    let error = stock
        .preview_sweep(&tool, &t, 1., SweepOptions::default(), &|| false)
        .unwrap_err();
    assert!(error.contains("neck, shank or holder"), "{error}");
    // A neck 0.05 mm narrower than the slot on each side: the rays across
    // the slot resolve the walls exactly, although every column at the
    // walls straddles them.
    let tool = t_slot_cutter(3.9);
    stock
        .preview_sweep(&tool, &t, 1., SweepOptions::default(), &|| false)
        .unwrap();
}

/// The same cut with the tool along +X on a block turned onto its side
/// removes the same material (axis permutation).
#[test]
fn fixed_axes_agree_by_symmetry() {
    let tool = assembly("t", cutter(CutterShape::Ball, 3., 12.));
    let mut upright = DexelStock::new(spec([0., 0., 0.], [8., 6., 5.], 0.25)).unwrap();
    let m = line(1, [-2., 2.5, 3.5], [10., 3.5, 3.]);
    sweep(&mut upright, &tool, &m);
    // Local (x, y, z) = world (y, z, x) for +X: a world point (x, y, z) of
    // the upright block maps to (z, x, y).
    let mut side = DexelStock::new(spec([0., 0., 0.], [5., 8., 6.], 0.25)).unwrap();
    let turned = |p: Vec3| [p[2], p[0], p[1]];
    let ms = line(1, turned(m.curve.start()), turned(m.curve.end()));
    let d = side
        .preview_sweep_axis(&tool, &ms, 1., Axis::PosX, SweepOptions::default(), &|| {
            false
        })
        .unwrap();
    side.apply_delta(&d).unwrap();
    let (a, b) = (upright.volume(), side.volume());
    // Grid X of `side` is grid Z of `upright`, and so on.
    for (k, l) in [(0, 1), (1, 2), (2, 0)] {
        for j in 0..2 {
            assert!(
                (a.grids_mm3[k][j] - b.grids_mm3[l][j]).abs() < 1e-6,
                "{a:?} {b:?}"
            );
        }
    }
    assert!(a.removed_lower_mm3 > 5.);
}

/// Rapids and zero-length prefixes change nothing; a budget or attribution
/// problem is refused before any change.
#[test]
fn rapids_prefixes_and_refusals() {
    let tool = assembly("t", cutter(CutterShape::Flat, 3., 12.));
    let stock = DexelStock::new(spec([0., 0., 0.], [8., 8., 5.], 0.5)).unwrap();
    let mut m = line(1, [-2., 2., 3.], [10., 2., 3.]);
    m.kind = MoveKind::Rapid;
    m.feed_mm_min = None;
    let d = stock
        .preview_sweep(&tool, &m, 1., SweepOptions::default(), &|| false)
        .unwrap();
    assert!(d.changes().is_empty());
    let m = line(1, [-2., 2., 3.], [10., 2., 3.]);
    let half = stock
        .preview_sweep(&tool, &m, 0.5, SweepOptions::default(), &|| false)
        .unwrap();
    let full = stock
        .preview_sweep(&tool, &m, 1., SweepOptions::default(), &|| false)
        .unwrap();
    assert!(!half.changes().is_empty() && half.changes().len() < full.changes().len());
    let mut wrong = m.clone();
    wrong.tool = "other".into();
    assert!(
        stock
            .preview_sweep(&tool, &wrong, 1., SweepOptions::default(), &|| false)
            .is_err()
    );
    let options = SweepOptions {
        max_cell_visits: 10,
        ..Default::default()
    };
    assert!(
        stock
            .preview_sweep(&tool, &m, 1., options, &|| false)
            .unwrap_err()
            .contains("budget")
    );
    // A delta applies once, on the stock it was prepared from.
    let mut s = stock.clone();
    s.apply_delta(&full).unwrap();
    assert!(s.apply_delta(&full).is_err());
}

/// A closed mesh of a prism: the simple polygon `profile` in the YZ plane
/// (counter-clockwise), extruded along X from `x0` to `x1`.
fn extrusion(profile: &[[f64; 2]], x0: f64, x1: f64) -> Vec<[Vec3; 3]> {
    let cross = |o: [f64; 2], a: [f64; 2], b: [f64; 2]| {
        (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    };
    // Ear clipping.
    let mut idx: Vec<usize> = (0..profile.len()).collect();
    let mut caps = vec![];
    while idx.len() > 3 {
        let n = idx.len();
        let ear = (0..n)
            .find(|&i| {
                let (a, b, c) = (
                    profile[idx[(i + n - 1) % n]],
                    profile[idx[i]],
                    profile[idx[(i + 1) % n]],
                );
                cross(a, b, c) > 0.
                    && idx.iter().all(|&j| {
                        let p = profile[j];
                        p == a
                            || p == b
                            || p == c
                            || !(cross(a, b, p) >= 0.
                                && cross(b, c, p) >= 0.
                                && cross(c, a, p) >= 0.)
                    })
            })
            .unwrap();
        caps.push([idx[(ear + n - 1) % n], idx[ear], idx[(ear + 1) % n]]);
        idx.remove(ear);
    }
    caps.push([idx[0], idx[1], idx[2]]);
    let at = |x: f64, p: [f64; 2]| [x, p[0], p[1]];
    let mut out = vec![];
    for [a, b, c] in caps {
        out.push([at(x0, profile[a]), at(x0, profile[c]), at(x0, profile[b])]);
        out.push([at(x1, profile[a]), at(x1, profile[b]), at(x1, profile[c])]);
    }
    for i in 0..profile.len() {
        let (p, q) = (profile[i], profile[(i + 1) % profile.len()]);
        out.push([at(x0, p), at(x0, q), at(x1, q)]);
        out.push([at(x0, p), at(x1, q), at(x1, p)]);
    }
    out
}
/// The T-slotted block's cross-section (Y, Z): a 4 mm slot 7 mm deep with
/// 2 mm wings 10 mm wide at its bottom.
const T_PROFILE: [[f64; 2]; 12] = [
    [0., 0.],
    [20., 0.],
    [20., 10.],
    [12., 10.],
    [12., 5.],
    [15., 5.],
    [15., 3.],
    [5., 3.],
    [5., 5.],
    [8., 5.],
    [8., 10.],
    [0., 10.],
];

/// The material check on dexel stock against a closed solid: the T-slot
/// leaves the part within tolerance (the grids together decide the walls
/// each grid alone cannot), an over-cut into it is a definite gouge, and
/// uncut wings are definite leftover material. Where two cut faces meet,
/// every grid's bound spans a whole cell, so the tolerance the rays can
/// prove is about a cell's diagonal (0.15 mm on 0.1 mm cells).
#[test]
fn solid_material_check_decides_undercuts() {
    let part = extrusion(&T_PROFILE, 0., 30.);
    let mut stock = slotted_block(0.1);
    let mut t = cut(
        2,
        "tslot",
        Curve::Line {
            start: [-8., 10., 3.],
            end: [38., 10., 3.],
        },
    );
    t.operation = "tslot".into();
    // Before the T-slot: the wings are left, certainly.
    let r = stock.compare_solid(&part, 0., 0.15, &|| false).unwrap();
    assert_eq!(
        r.gouge.assessment,
        SolidAssessment::WithinTolerance,
        "{r:?}"
    );
    assert_eq!(
        r.residual.assessment,
        SolidAssessment::DefiniteExcess,
        "{r:?}"
    );
    assert!(
        r.residual.volume_mm3[0] > 300. && r.residual.volume_mm3[1] >= 360. - 1e-6,
        "{r:?}"
    );
    sweep(&mut stock, &t_slot_cutter(3.), &t);
    let r = stock.compare_solid(&part, 0., 0.15, &|| false).unwrap();
    assert_eq!(
        r.gouge.assessment,
        SolidAssessment::WithinTolerance,
        "{r:?}"
    );
    assert_eq!(
        r.residual.assessment,
        SolidAssessment::WithinTolerance,
        "{r:?}"
    );
    assert!(r.residual.volume_mm3[0] < 1e-6, "{r:?}");
    // The T 0.5 mm too low: a definite gouge under the wings.
    let mut low = slotted_block(0.1);
    let mut t = t.clone();
    t.curve = Curve::Line {
        start: [-8., 10., 2.5],
        end: [38., 10., 2.5],
    };
    let mill = assembly("t", cutter(CutterShape::Flat, 4., 15.));
    sweep(&mut low, &mill, &line(1, [-5., 10., 2.5], [35., 10., 2.5]));
    sweep(&mut low, &t_slot_cutter(3.), &t);
    let r = low.compare_solid(&part, 0., 0.15, &|| false).unwrap();
    assert_eq!(r.gouge.assessment, SolidAssessment::DefiniteExcess, "{r:?}");
    assert!(r.gouge.witness_depth_mm.unwrap() > 0.15);
    // An open mesh is refused.
    assert!(
        stock
            .compare_solid(&part[1..], 0., 0.15, &|| false)
            .is_err()
    );
}
