//! Mesh stock in, finished stock out (#121). The checks here do not trust the
//! conversion: points are classified against the mesh by ray casting written
//! in this file, and expected bounds of simple solids are worked out by hand.
use camlib::{motion::*, stock::*, stock_mesh::*, stream::*, tool::*};
use std::sync::atomic::AtomicBool;

fn never() -> AtomicBool {
    AtomicBool::new(false)
}
fn grid(min: Vec3, max: Vec3, cell: f64) -> StockSpec {
    StockSpec {
        version: 1,
        units: "mm".into(),
        min_mm: min,
        max_mm: max,
        cell_size_mm: cell,
        max_cells: 4_000_000,
    }
}

/// A box, triangles counter-clockwise seen from outside.
fn box_triangles(a: Vec3, b: Vec3) -> Vec<[Vec3; 3]> {
    let p = |i: usize| -> Vec3 {
        [
            if i & 1 == 0 { a[0] } else { b[0] },
            if i & 2 == 0 { a[1] } else { b[1] },
            if i & 4 == 0 { a[2] } else { b[2] },
        ]
    };
    // Faces as corner indices, counter-clockwise seen from outside.
    let faces = [
        [0, 2, 3, 1], // bottom (-z)
        [4, 5, 7, 6], // top (+z)
        [0, 1, 5, 4], // -y
        [2, 6, 7, 3], // +y
        [0, 4, 6, 2], // -x
        [1, 3, 7, 5], // +x
    ];
    faces
        .iter()
        .flat_map(|f| [[p(f[0]), p(f[1]), p(f[2])], [p(f[0]), p(f[2]), p(f[3])]])
        .collect()
}

/// A prism: a profile in the XZ plane (counter-clockwise with x right and z
/// up), extruded from y0 to y1. `caps` triangulates the profile.
fn extrude(profile: &[[f64; 2]], caps: &[[usize; 3]], y0: f64, y1: f64) -> Vec<[Vec3; 3]> {
    let at = |k: usize, y: f64| [profile[k][0], y, profile[k][1]];
    let mut t = Vec::new();
    for c in caps {
        t.push([at(c[0], y0), at(c[1], y0), at(c[2], y0)]);
        t.push([at(c[0], y1), at(c[2], y1), at(c[1], y1)]);
    }
    for k in 0..profile.len() {
        let n = (k + 1) % profile.len();
        let (a0, a1, b0, b1) = (at(k, y0), at(k, y1), at(n, y0), at(n, y1));
        t.push([a0, a1, b1]);
        t.push([a0, b1, b0]);
    }
    t
}

/// Stepped blank: z = 10 for x < 20, z = 20 for x > 20, 30 mm deep.
fn stepped() -> Vec<[Vec3; 3]> {
    let profile = [
        [0., 0.],
        [20., 0.],
        [40., 0.],
        [40., 20.],
        [20., 20.],
        [20., 10.],
        [0., 10.],
    ];
    let caps = [[0, 1, 5], [0, 5, 6], [2, 3, 4], [2, 4, 5], [2, 5, 1]];
    extrude(&profile, &caps, 0., 30.)
}
/// A T lying on its stem: an overhang on both sides (material above air).
fn overhang() -> Vec<[Vec3; 3]> {
    let profile = [
        [15., 0.],
        [25., 0.],
        [25., 10.],
        [40., 10.],
        [40., 20.],
        [0., 20.],
        [0., 10.],
        [15., 10.],
    ];
    let caps = [
        [0, 1, 2],
        [0, 2, 7],
        [4, 5, 6],
        [4, 6, 7],
        [4, 7, 2],
        [4, 2, 3],
    ];
    extrude(&profile, &caps, 0., 30.)
}
/// A wedge: a sloped top from z = 20 at x = 0 to z = 0 at x = 40.
fn wedge() -> Vec<[Vec3; 3]> {
    extrude(&[[0., 0.], [40., 0.], [0., 20.]], &[[0, 1, 2]], -5., 25.)
}
fn rotate(triangles: &[[Vec3; 3]], axis: usize, angle: f64) -> Vec<[Vec3; 3]> {
    let (s, c) = angle.sin_cos();
    let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
    triangles
        .iter()
        .map(|t| {
            t.map(|p| {
                let mut q = p;
                q[u] = c * p[u] - s * p[v];
                q[v] = s * p[u] + c * p[v];
                q
            })
        })
        .collect()
}

/// Independent point classification: crossings of a fixed oblique ray
/// (Möller–Trumbore), so a ray through an edge is practically impossible.
fn inside(triangles: &[[Vec3; 3]], p: Vec3) -> bool {
    let d = [0.5773502691896258, 0.6123724356957945, 0.5400617248673217];
    let sub = |a: Vec3, b: Vec3| [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    let cross = |a: Vec3, b: Vec3| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let dot = |a: Vec3, b: Vec3| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let mut hits = 0;
    for t in triangles {
        let (e1, e2) = (sub(t[1], t[0]), sub(t[2], t[0]));
        let h = cross(d, e2);
        let det = dot(e1, h);
        if det.abs() < 1e-14 {
            continue;
        }
        let s = sub(p, t[0]);
        let u = dot(s, h) / det;
        let q = cross(s, e1);
        let v = dot(d, q) / det;
        if u < 0. || v < 0. || u + v > 1. {
            continue;
        }
        if dot(e2, q) / det > 0. {
            hits += 1;
        }
    }
    hits % 2 == 1
}

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
}

fn cell_of(stock: &Stock, p: Vec3) -> Option<usize> {
    let spec = stock.spec();
    let [dx, dy] = stock.cell_widths_mm();
    let [nx, ny] = stock.dimensions();
    let fx = (p[0] - spec.min_mm[0]) / dx;
    let fy = (p[1] - spec.min_mm[1]) / dy;
    // Stay clear of cell sides (a point on one belongs to two cells).
    if (fx - fx.round()).abs() < 1e-6 || (fy - fy.round()).abs() < 1e-6 {
        return None;
    }
    let (i, j) = (fx.floor() as usize, fy.floor() as usize);
    (i < nx && j < ny).then_some(j * nx + i)
}

/// The conversion's guarantee, sampled: material lies at or below each
/// cell's upper bound, and everything below the lower bound is material.
fn assert_conservative(triangles: &[[Vec3; 3]], cell: f64, samples: usize) -> Stock {
    let mesh = StockMesh::from_triangles(triangles).unwrap();
    let spec = MeshStockSpec::from_mesh(mesh, cell, 4_000_000).unwrap();
    let stock = spec.initial_stock(&never()).unwrap();
    let [lo, hi] = spec.mesh().bounds();
    let mut rng = Rng(7);
    let (mut solid, mut certain) = (0, 0);
    for _ in 0..samples {
        let p: Vec3 = std::array::from_fn(|k| lo[k] + rng.next() * (hi[k] - lo[k]));
        let Some(c) = cell_of(&stock, p) else {
            continue;
        };
        let h = stock.cells()[c];
        if inside(triangles, p) {
            solid += 1;
            assert!(p[2] <= h.upper_mm + 1e-9, "material at {p:?} above {h:?}");
        }
        if p[2] < h.lower_mm - 1e-9 {
            certain += 1;
            assert!(inside(triangles, p), "{p:?} below {h:?} is not material");
        }
    }
    assert!(solid > samples / 10 && certain > 0, "{solid} {certain}");
    stock
}

#[test]
fn a_box_mesh_is_the_same_stock_as_the_box() {
    for cell in [1., 0.7, 0.3, 3.] {
        let (a, b) = ([-3., 2., -12.5], [17.25, 31., 0.]);
        let mesh = StockMesh::from_triangles(&box_triangles(a, b)).unwrap();
        assert!((mesh.volume_mm3() - 20.25 * 29. * 12.5).abs() < 1e-9);
        let spec = MeshStockSpec::from_mesh(mesh, cell, 4_000_000).unwrap();
        assert_eq!(*spec.grid(), grid(a, b, cell));
        let from_mesh = spec.initial_stock(&never()).unwrap();
        let from_box = Stock::new(grid(a, b, cell)).unwrap();
        assert_eq!(from_mesh.cells(), from_box.cells(), "cell {cell}");
        assert_eq!(
            from_mesh.volume().remaining_lower_mm3,
            from_box.volume().remaining_lower_mm3
        );
    }
}

#[test]
fn a_stepped_blank_is_exact_on_an_aligned_grid_and_bounded_across_the_step() {
    let mesh = StockMesh::from_triangles(&stepped()).unwrap();
    assert!((mesh.volume_mm3() - 18000.).abs() < 1e-9);
    let spec = MeshStockSpec::from_mesh(mesh.clone(), 1., 1_000_000).unwrap();
    let stock = spec.initial_stock(&never()).unwrap();
    let [nx, _] = stock.dimensions();
    for (c, h) in stock.cells().iter().enumerate() {
        let z = if c % nx < 20 { 10. } else { 20. };
        assert_eq!((h.lower_mm, h.upper_mm), (z, z), "cell {c}");
    }
    // 0.7 mm cells: the column holding the step (x 19.6..20.3) may hold
    // material up to 20 and certainly holds it up to 10.
    let spec = MeshStockSpec::from_mesh(mesh, 0.7, 1_000_000).unwrap();
    let stock = spec.initial_stock(&never()).unwrap();
    let [nx, _] = stock.dimensions();
    let w = stock.cell_widths_mm()[0];
    for (c, h) in stock.cells().iter().enumerate() {
        let (x0, x1) = ((c % nx) as f64 * w, (c % nx + 1) as f64 * w);
        let expected = if x1 <= 20. {
            (10., 10.)
        } else if x0 >= 20. {
            (20., 20.)
        } else {
            (10., 20.)
        };
        assert_eq!((h.lower_mm, h.upper_mm), expected, "cell {c} from x {x0}");
    }
}

#[test]
fn conversion_never_drops_material_or_claims_air() {
    assert_conservative(&stepped(), 0.7, 40_000);
    // Over the overhangs the air below is unknown: only possibly material.
    let stock = assert_conservative(&overhang(), 1.3, 40_000);
    let [nx, _] = stock.dimensions();
    let h = stock.cells()[3 * nx + 2];
    assert_eq!((h.lower_mm, h.upper_mm), (0., 20.));
    let h = stock.cells()[3 * nx + 15];
    assert_eq!((h.lower_mm, h.upper_mm), (20., 20.));
    assert_conservative(&wedge(), 0.9, 40_000);
    assert_conservative(
        &rotate(&box_triangles([0.; 3], [30., 10., 8.]), 2, 0.5),
        0.4,
        40_000,
    );
    // Tilted: the bottom is not on the grid's bottom plane, so nothing is
    // certain, but nothing is dropped either.
    let tilted = rotate(&stepped(), 0, 0.2);
    let stock = assert_conservative_upper(&tilted, 0.8, 40_000);
    assert!(
        stock
            .cells()
            .iter()
            .all(|h| h.lower_mm == stock.spec().min_mm[2])
    );
}
fn assert_conservative_upper(triangles: &[[Vec3; 3]], cell: f64, samples: usize) -> Stock {
    let mesh = StockMesh::from_triangles(triangles).unwrap();
    let spec = MeshStockSpec::from_mesh(mesh, cell, 4_000_000).unwrap();
    let stock = spec.initial_stock(&never()).unwrap();
    let [lo, hi] = spec.mesh().bounds();
    let mut rng = Rng(11);
    for _ in 0..samples {
        let p: Vec3 = std::array::from_fn(|k| lo[k] + rng.next() * (hi[k] - lo[k]));
        if let Some(c) = cell_of(&stock, p)
            && inside(triangles, p)
        {
            assert!(p[2] <= stock.cells()[c].upper_mm + 1e-9, "{p:?}");
        }
    }
    stock
}

#[test]
fn open_inside_out_and_misplaced_meshes_are_refused() {
    let mut open = box_triangles([0.; 3], [10.; 3]);
    open.pop();
    let e = StockMesh::from_triangles(&open).unwrap_err();
    assert!(e.contains("not closed") && e.contains("3 edge"), "{e}");
    let mut mixed = box_triangles([0.; 3], [10.; 3]);
    mixed[0].swap(1, 2);
    assert!(
        StockMesh::from_triangles(&mixed)
            .unwrap_err()
            .contains("not closed")
    );
    let inverted: Vec<_> = box_triangles([0.; 3], [10.; 3])
        .into_iter()
        .map(|[a, b, c]| [a, c, b])
        .collect();
    assert!(
        StockMesh::from_triangles(&inverted)
            .unwrap_err()
            .contains("inside out")
    );
    let mut far = box_triangles([0.; 3], [10.; 3]);
    far[0][0][0] = f64::NAN;
    assert!(
        StockMesh::from_triangles(&far)
            .unwrap_err()
            .contains("finite")
    );
    let mesh = StockMesh::from_triangles(&box_triangles([0.; 3], [10.; 3])).unwrap();
    let e = MeshStockSpec::new(grid([0.; 3], [10., 10., 9.], 1.), mesh).unwrap_err();
    assert!(e.contains("outside the stock box"), "{e}");
    // A larger box is fine: the space around the mesh is empty.
    let mesh = StockMesh::from_triangles(&box_triangles([0.; 3], [10.; 3])).unwrap();
    let stock = MeshStockSpec::new(grid([-2., 0., -1.], [10., 10., 12.], 1.), mesh)
        .unwrap()
        .initial_stock(&never())
        .unwrap();
    let [nx, _] = stock.dimensions();
    assert_eq!(stock.cells()[0].upper_mm, -1.);
    // Nothing certain: the mesh's bottom is above the grid's bottom plane.
    assert_eq!(
        (stock.cells()[5].lower_mm, stock.cells()[5].upper_mm),
        (-1., 10.)
    );
    assert_eq!(nx, 12);
}

fn manifest(stock: RunStockSpec) -> StreamManifest {
    StreamManifest {
        tool_change: None,
        initial_stock: None,
        rapid_retracts: false,
        turning: None,
        end_position: None,
        version: 1,
        run: RunIdentity {
            id: "run-1".into(),
            revision: "r1".into(),
        },
        stock,
        tools: vec![assembly()],
        initial_position_mm: [0., 0., 40.],
        sweep: SweepOptions::default(),
        operations: vec![],
        verification: None,
    }
}
fn assembly() -> ToolAssembly {
    ToolAssembly {
        id: "t1".into(),
        cutter: Cutter {
            id: "cutter".into(),
            name: "4 mm flat".into(),
            shape: CutterShape::Flat,
            diameter_mm: 4.,
            cutting_length_mm: 30.,
            overall_length_mm: 50.,
            shank_diameter_mm: 4.,
            flutes: 2,
            center_cutting: true,
            material: "carbide".into(),
            neck: None,
        },
        stickout_mm: 35.,
        gauge_length_mm: 60.,
        holders: vec![HolderSection {
            z_min_mm: 35.,
            z_max_mm: 60.,
            radius_min_mm: 4.,
            radius_max_mm: 4.,
        }],
    }
}

#[test]
fn a_run_starts_from_the_mesh_and_its_manifest_carries_it() {
    let spec =
        MeshStockSpec::from_mesh(StockMesh::from_triangles(&stepped()).unwrap(), 0.5, 1 << 20)
            .unwrap();
    let expected = spec.initial_stock(&never()).unwrap();
    let m = manifest(spec.clone().into());
    let bytes = m.canonical_bytes().unwrap();
    let read: StreamManifest = serde_json::from_slice(&bytes).unwrap();
    assert_eq!(read.stock, m.stock);
    assert_eq!(read.canonical_bytes().unwrap(), bytes);
    assert_eq!(read.stock.height().unwrap(), spec.grid());
    let receiver = StreamReceiver::new(read).unwrap();
    assert_eq!(receiver.stock().unwrap().cells(), expected.cells());
    // A different mesh is a different run input.
    let other = MeshStockSpec::from_mesh(
        StockMesh::from_triangles(&overhang()).unwrap(),
        0.5,
        1 << 20,
    )
    .unwrap();
    assert_ne!(
        manifest(other.into()).digest().unwrap(),
        m.digest().unwrap()
    );
    // The mesh's own refusal reaches whoever reads the manifest.
    let mut value: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
    value["stock"]["mesh"]["indices"]
        .as_array_mut()
        .unwrap()
        .pop();
    let e = serde_json::from_value::<StreamManifest>(value).unwrap_err();
    assert!(e.to_string().contains("not closed"), "{e}");
    // A box run's manifest is unchanged by the mesh form.
    let boxed = manifest(grid([0.; 3], [20., 20., 10.], 0.2).into());
    let text = String::from_utf8(boxed.canonical_bytes().unwrap()).unwrap();
    assert!(!text.contains("mesh"));
}

#[test]
fn checks_see_the_real_stock_not_the_box() {
    let spec =
        MeshStockSpec::from_mesh(StockMesh::from_triangles(&stepped()).unwrap(), 0.5, 1 << 20)
            .unwrap();
    let stepped = spec.initial_stock(&never()).unwrap();
    let block = Stock::new(spec.grid().clone()).unwrap();
    let cut = |a: Vec3, b: Vec3| Motion {
        id: 1,
        operation: "op".into(),
        tool: "t1".into(),
        kind: MoveKind::Cut,
        curve: Curve::Line { start: a, end: b },
        feed_mm_min: Some(1000.),
    };
    let may_cut = |stock: &Stock, m: &Motion| {
        stock
            .may_remove_material(m, 2., 0., 1e-5, &mut usize::MAX.clone(), &never())
            .unwrap()
    };
    // At z 15 over the low step: air in the blank, material in a box.
    let air = cut([3., 5., 15.], [15., 25., 15.]);
    assert!(!may_cut(&stepped, &air));
    assert!(may_cut(&block, &air));
    // Over the high step both hold material.
    let high = cut([25., 5., 15.], [35., 25., 15.]);
    assert!(may_cut(&stepped, &high));
}

/// A 4 mm flat cutter swept along a line at z 6 in a 20 x 20 x 10 block.
fn slotted() -> Stock {
    let mut stock = Stock::new(grid([0.; 3], [20., 20., 10.], 0.25)).unwrap();
    let tool = assembly().prepare().unwrap();
    let m = Motion {
        id: 1,
        operation: "op".into(),
        tool: "t1".into(),
        kind: MoveKind::Cut,
        curve: Curve::Line {
            start: [3., 10., 6.],
            end: [17., 10., 6.],
        },
        feed_mm_min: Some(1000.),
    };
    let delta = stock
        .preview_sweep(&tool, &m, 1., SweepOptions::default(), &never())
        .unwrap();
    stock.apply_delta(&delta).unwrap();
    stock
}

#[test]
fn the_finished_mesh_holds_all_remaining_material_and_reads_back_the_same() {
    let stock = slotted();
    let finished = stock.finished_mesh(&never()).unwrap();
    assert!(finished.approximation.unwrap().starts_with("approximate"));
    let mesh = &finished.mesh;
    let triangles: Vec<[Vec3; 3]> = mesh
        .indices()
        .iter()
        .map(|t| t.map(|i| mesh.vertices()[i as usize]))
        .collect();
    // Its volume is the stock model's possible material, exactly.
    let v = stock.volume();
    assert!((mesh.volume_mm3() - v.remaining_upper_mm3).abs() < 1e-6 * v.remaining_upper_mm3);
    assert!(mesh.volume_mm3() < 20. * 20. * 10. - 14. * 4. * 4.);
    // All the material really left (the block less the cutter's swept
    // stadium above z 6, worked out here) is inside the mesh.
    let mut rng = Rng(3);
    let mut checked = 0;
    for _ in 0..20_000 {
        let p: Vec3 = [20. * rng.next(), 20. * rng.next(), 10. * rng.next()];
        let dx = (p[0] - p[0].clamp(3., 17.)).abs();
        let in_slot = p[2] >= 6. && (dx * dx + (p[1] - 10.).powi(2)) < 4.;
        if !in_slot && cell_of(&stock, p).is_some() {
            checked += 1;
            assert!(inside(&triangles, p), "material at {p:?} outside the mesh");
        }
    }
    assert!(checked > 15_000);
    // On the same grid it reads back as the same upper bounds, now certain.
    let back = MeshStockSpec::new(stock.spec().clone(), mesh.clone())
        .unwrap()
        .initial_stock(&never())
        .unwrap();
    for (a, b) in stock.cells().iter().zip(back.cells()) {
        assert_eq!((b.lower_mm, b.upper_mm), (a.upper_mm, a.upper_mm));
    }
}

#[test]
fn a_finished_stock_turned_over_starts_the_second_setup() {
    let first = slotted();
    let mesh = first.finished_mesh(&never()).unwrap().mesh;
    // Turn it over about X into the second setup's frame: y -> 20 - y,
    // z -> 10 - z (exact in floating point, orientation kept).
    let turned = StockMesh::new(
        mesh.vertices()
            .iter()
            .map(|v| [v[0], 20. - v[1], 10. - v[2]])
            .collect(),
        mesh.indices().to_vec(),
    )
    .unwrap();
    let triangles: Vec<[Vec3; 3]> = turned
        .indices()
        .iter()
        .map(|t| t.map(|i| turned.vertices()[i as usize]))
        .collect();
    let spec = MeshStockSpec::from_mesh(turned, 0.25, 1 << 20).unwrap();
    let second = spec.initial_stock(&never()).unwrap();
    // The slot now opens downward: those columns may hold material up to the
    // top but are certain only where the bottom is whole.
    let v = second.volume();
    let real = 20. * 20. * 10. - first.volume().removed_lower_mm3;
    assert!(v.remaining_lower_mm3 < real && real <= v.remaining_upper_mm3 + 1e-6);
    assert!((v.remaining_upper_mm3 - 4000.).abs() < 1e-6);
    let mut rng = Rng(5);
    for _ in 0..20_000 {
        let p: Vec3 = [20. * rng.next(), 20. * rng.next(), 10. * rng.next()];
        if let Some(c) = cell_of(&second, p) {
            let h = second.cells()[c];
            if inside(&triangles, p) {
                assert!(p[2] <= h.upper_mm);
            }
            if p[2] < h.lower_mm {
                assert!(inside(&triangles, p));
            }
        }
    }
}

#[test]
fn an_empty_stock_has_no_mesh() {
    let spec = grid([0.; 3], [2., 2., 1.], 1.);
    let cells = vec![
        HeightBounds {
            lower_mm: 0.,
            upper_mm: 0.
        };
        4
    ];
    let e = height_field_mesh(&spec, &cells, &never()).unwrap_err();
    assert!(e.contains("no material"), "{e}");
}

#[test]
fn a_mesh_stock_with_declared_initial_bounds_is_refused() {
    // Two declarations of the starting material, a stock mesh (#121) and
    // whole-cell initial bounds (#133), are ambiguous: refused, not merged.
    let mesh = StockMesh::from_triangles(&box_triangles([0.; 3], [10.; 3])).unwrap();
    let spec = MeshStockSpec::new(grid([0.; 3], [10.; 3], 1.), mesh).unwrap();
    let mut m = manifest(spec.into());
    m.initial_stock = Some(InitialStock {
        source_id: "measured-blank".into(),
        revision: "1".into(),
        source_sha256: "a".repeat(64),
        source_to_setup: WorkFrame::identity(),
        uncertainty_mm: 0.,
        representation: "whole_cell_top_down".into(),
        bounds: vec![
            HeightBounds {
                lower_mm: 0.,
                upper_mm: 10.,
            };
            100
        ],
    });
    let e = m.validate().unwrap_err();
    assert!(e.contains("declare the starting stock one way"), "{e}");
    m.initial_stock = None;
    m.validate().unwrap();
}
