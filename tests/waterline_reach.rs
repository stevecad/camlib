//! Issue #161: waterline admits each feed move by the depth it actually cuts.
//!
//! A cone `z = 40 - 3r` down to a flat at Z1 (30 x 30 mm) is taller than the
//! 12 mm flutes of a 6 mm ball, but it narrows upward: cut top-down from
//! stock left 0.5 mm over the part, the shank and holder always have the
//! finished wall receding from them. From a box, or beside an undrafted wall
//! where the shank would rub, the same passes really cut deeper than the
//! flutes and are refused with a typed reach refusal.
use camlib::{
    motion::MoveKind,
    operation::OperationPlan,
    pocket::Operation,
    reach::{ReachLimit, ReachRefusal},
    stock::{Stock, SweepOptions},
    stock_contact::{self, Options},
    stock_mesh::{MeshStockSpec, StockMesh},
    tool::{Cutter, CutterShape, HolderSection, ToolAssembly},
    verification::Status,
};
use std::sync::atomic::AtomicBool;

const CENTER: f64 = 15.;
const SEGMENTS: usize = 64;
/// The 0.5 mm normal offset of the cone's 3:1 flank, measured vertically.
const LIFT: f64 = 0.5 * 3.162_277_660_168_379_5;

type Vec3 = [f64; 3];

fn never() -> AtomicBool {
    AtomicBool::new(false)
}
fn ring(radius: f64, z: f64) -> Vec<Vec3> {
    (0..SEGMENTS)
        .map(|i| {
            let a = i as f64 * std::f64::consts::TAU / SEGMENTS as f64;
            [CENTER + radius * a.cos(), CENTER + radius * a.sin(), z]
        })
        .collect()
}
/// The 30 x 30 square's outline at the ring's angles (corners included).
fn square(z: f64) -> Vec<Vec3> {
    (0..SEGMENTS)
        .map(|i| {
            let a = i as f64 * std::f64::consts::TAU / SEGMENTS as f64;
            let (c, s) = (a.cos(), a.sin());
            let t = CENTER / c.abs().max(s.abs());
            [
                (CENTER + t * c).clamp(0., 30.),
                (CENTER + t * s).clamp(0., 30.),
                z,
            ]
        })
        .collect()
}
/// Faces between two rings, normals outward (and up) for a profile that
/// runs outward or downward.
fn band(a: &[Vec3], b: &[Vec3], out: &mut Vec<[Vec3; 3]>) {
    for i in 0..SEGMENTS {
        let j = (i + 1) % SEGMENTS;
        out.push([a[i], b[i], b[j]]);
        out.push([a[i], b[j], a[j]]);
    }
}
/// A turned top surface: an apex (or flat top) then rings of (radius, z)
/// down and out, then the square at the last ring's Z. With `base`, closed
/// by the square's walls down to Z0 and a bottom: a stock solid.
fn turned(apex: f64, profile: &[(f64, f64)], base: bool) -> Vec<[Vec3; 3]> {
    let mut out = vec![];
    let top = [CENTER, CENTER, apex];
    let mut rings: Vec<Vec<Vec3>> = profile.iter().map(|&(r, z)| ring(r, z)).collect();
    let z = profile.last().unwrap().1;
    rings.push(square(z));
    if base {
        rings.push(square(0.));
    }
    for i in 0..SEGMENTS {
        out.push([top, rings[0][i], rings[0][(i + 1) % SEGMENTS]]);
    }
    for w in rings.windows(2) {
        band(&w[0], &w[1], &mut out);
    }
    if base {
        let bottom = rings.last().unwrap();
        let c = [CENTER, CENTER, 0.];
        for i in 0..SEGMENTS {
            out.push([c, bottom[(i + 1) % SEGMENTS], bottom[i]]);
        }
    }
    out
}
fn cone() -> Vec<[Vec3; 3]> {
    turned(40., &[(13., 1.)], false)
}
fn cone_stock() -> Vec<[Vec3; 3]> {
    let top = 40. + LIFT;
    turned(top, &[((top - 1.5) / 3., 1.5)], true)
}
/// A 6 mm radius boss with vertical walls up to Z25 on the Z1 flat.
fn boss() -> Vec<[Vec3; 3]> {
    turned(25., &[(6., 25.), (6., 1.)], false)
}
fn boss_stock() -> Vec<[Vec3; 3]> {
    turned(25.5, &[(6.5, 25.5), (6.5, 1.5)], true)
}
fn mesh_stock(triangles: &[[Vec3; 3]]) -> Stock {
    MeshStockSpec::from_mesh(StockMesh::from_triangles(triangles).unwrap(), 0.5, 1 << 20)
        .unwrap()
        .initial_stock(&never())
        .unwrap()
}
/// A 6 mm ball, 12 mm flutes, 6 mm shank, 25 mm stick-out, holder radius 8.
fn ball() -> ToolAssembly {
    ToolAssembly {
        id: "ball6".into(),
        cutter: Cutter {
            id: "b6".into(),
            name: "6 mm ball".into(),
            shape: CutterShape::Ball,
            diameter_mm: 6.,
            cutting_length_mm: 12.,
            overall_length_mm: 50.,
            shank_diameter_mm: 6.,
            flutes: 2,
            center_cutting: true,
            material: "carbide".into(),
            neck: None,
        },
        stickout_mm: 25.,
        gauge_length_mm: 60.,
        holders: vec![HolderSection {
            z_min_mm: 25.,
            z_max_mm: 60.,
            radius_min_mm: 8.,
            radius_max_mm: 8.,
        }],
    }
}
fn waterline(surface: &[[Vec3; 3]], top: f64, extra: serde_json::Value) -> Operation {
    let mut op = serde_json::json!({
        "kind": "waterline",
        "id": "wl",
        "tool": "ball6",
        "surface": {
            "version": 1,
            "units": "mm",
            "triangles": surface,
            "surface_error_mm": 0,
            "source": null
        },
        "floor_z_mm": 0,
        "top_z_mm": top,
        "bottom_z_mm": 1.5,
        "stepdown_mm": 0.5,
        "clearance_z_mm": 50,
        "contour_error_mm": 0.01,
        "safety_margin_mm": 0.005,
        "stock_to_leave_mm": 0,
        "reverse": false,
        "simplify": true,
        "tolerance_mm": 1e-5,
        "feed_mm_min": 1200,
        "plunge_mm_min": 300,
        "spindle_rpm": 12000,
        "work_offset": 1,
        "tool_number": 1,
        "length_register": 1,
        "coolant": "flood"
    });
    op.as_object_mut()
        .unwrap()
        .extend(extra.as_object().unwrap().clone());
    serde_json::from_value(op).unwrap()
}
fn parallel(surface: &[[Vec3; 3]], slope_to: f64) -> Operation {
    serde_json::from_value(serde_json::json!({
        "kind": "parallel",
        "id": "par",
        "tool": "ball6",
        "surface": {
            "version": 1,
            "units": "mm",
            "triangles": surface,
            "surface_error_mm": 0,
            "source": null
        },
        "floor_z_mm": 0,
        "stepover_mm": 1,
        "clearance_z_mm": 50,
        "slope_to_deg": slope_to,
        "entry": {"kind": "ramp", "angle_rad": 10f64.to_radians(), "min_length_mm": 1.0},
        "axis": "x",
        "reverse": false,
        "stock_to_leave_mm": 0,
        "path_error_mm": 0.005,
        "safety_margin_mm": 0.005,
        "entry_air_gap_mm": 0.5,
        "tolerance_mm": 1e-5,
        "work_offset": 1,
        "feed_mm_min": 1200,
        "plunge_mm_min": 300,
        "spindle_rpm": 12000,
        "tool_number": 1,
        "length_register": 1,
        "coolant": "flood"
    }))
    .unwrap()
}

/// Plans and commits every move; with `contact`, the independent stock
/// contact check judges each against the stock it is planned from.
fn run(op: Operation, stock: &mut Stock, contact: bool) -> Result<usize, String> {
    let tool = ball();
    let prepared = tool.prepare().unwrap();
    let mut plan =
        OperationPlan::prepare_with_stock(op, &tool, stock, [CENTER, CENTER, 60.], 1, &never())?;
    let mut cuts = 0;
    while let Some(m) = plan.next_motion_with_stock(stock, &never())? {
        if contact {
            let r =
                stock_contact::check(stock, &prepared, &m, Options::default(), &never()).unwrap();
            assert_eq!(r.status, Status::Clear, "{m:?} {:?}", r.findings);
        }
        cuts += usize::from(m.kind == MoveKind::Cut);
        let delta = stock
            .preview_sweep(&prepared, &m, 1., SweepOptions::default(), &never())
            .map_err(|e| format!("sweep of {m:?}: {e}"))?;
        stock.apply_delta(&delta).unwrap();
    }
    Ok(cuts)
}

#[test]
fn cone_taller_than_the_flutes_is_finished_from_roughed_stock() {
    let mut stock = mesh_stock(&cone_stock());
    stock.enable_contact_queries(&never()).unwrap();
    // The remaining stock stands 40 mm above the lowest level, past the
    // 12 mm flutes: the depth is checked move by move.
    let tool = ball();
    let plan = OperationPlan::prepare_with_stock(
        waterline(&cone(), 40., serde_json::json!({})),
        &tool,
        &stock,
        [CENTER, CENTER, 60.],
        1,
        &never(),
    )
    .unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    assert!(
        report["reach_proof"]
            .as_str()
            .unwrap()
            .contains("every feed move")
    );
    assert!(report["stock_top"]["maximum_upper_z_mm"].as_f64().unwrap() > 41.);
    let cuts = run(
        waterline(&cone(), 40., serde_json::json!({})),
        &mut stock,
        true,
    )
    .unwrap();
    assert!(cuts > 1000, "{cuts}");
    // The flank is finished down to the flat: 0.5 mm above the cone at r 10
    // before, within a cell's slope of it after.
    let [nx, _] = stock.dimensions();
    let at = |x: f64, y: f64| stock.cells()[(y / 0.5) as usize * nx + (x / 0.5) as usize];
    let flank = at(CENTER + 10.25, CENTER + 0.25);
    assert!(flank.upper_mm < 40. - 3. * 10. + 1.6, "{flank:?}");
}

#[test]
fn steep_and_shallow_on_the_cone_is_accepted() {
    let mut stock = mesh_stock(&cone_stock());
    stock.enable_contact_queries(&never()).unwrap();
    // Steep: waterline above 40 degrees; shallow: parallel below 50.
    let steep = waterline(&cone(), 40., serde_json::json!({"slope_from_deg": 40}));
    assert!(run(steep, &mut stock, true).unwrap() > 500);
    assert!(run(parallel(&cone(), 50.), &mut stock, true).unwrap() > 100);
}

#[test]
fn cone_from_the_box_is_refused_with_a_typed_reach_refusal() {
    let spec = MeshStockSpec::from_mesh(
        StockMesh::from_triangles(&cone_stock()).unwrap(),
        0.5,
        1 << 20,
    )
    .unwrap();
    let mut block = Stock::new(spec.grid().clone()).unwrap();
    let top = spec.grid().max_mm[2];
    let error = run(
        waterline(&cone(), 40., serde_json::json!({})),
        &mut block,
        false,
    )
    .unwrap_err();
    assert!(
        error.contains("cuts deeper than the cutting length 12"),
        "{error}"
    );
    assert!(error.contains("shank"), "{error}");
    let r = ReachRefusal::parse(&error).expect(&error);
    assert_eq!((r.operation.as_str(), r.tool.as_str()), ("wl", "ball6"));
    assert_eq!((r.limit, r.holder_section), (ReachLimit::Shank, None));
    // The first level whose shank (12 mm above the tip) is below the box top.
    assert_eq!(r.top_z_mm, top);
    assert!(r.z_mm <= top - 12. && r.z_mm > top - 12.5, "{r:?}");
    assert_eq!(r.depth_available_mm, 12.);
    assert_eq!(r.depth_needed_mm, r.top_z_mm - r.z_mm);
}

#[test]
fn undrafted_wall_where_the_shank_rubs_is_refused() {
    let mut stock = mesh_stock(&boss_stock());
    let error = run(
        waterline(&boss(), 25., serde_json::json!({})),
        &mut stock,
        false,
    )
    .unwrap_err();
    let r = ReachRefusal::parse(&error).expect(&error);
    assert_eq!((r.limit, r.holder_section), (ReachLimit::Shank, None));
    // The shank (as wide as the ball) runs down the finished wall: refused
    // once its bottom is below the wall's remaining top.
    assert!(r.top_z_mm >= 25. && r.top_z_mm <= 25.5 + 1e-9, "{r:?}");
    assert!(
        r.z_mm < r.top_z_mm - 12. && r.z_mm >= r.top_z_mm - 12.5,
        "{r:?}"
    );
}
