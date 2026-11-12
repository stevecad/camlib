//! Automatic nesting for thermal cutting: layouts are deterministic, proven
//! by the independent check, report what does not fit, and feed a thermal
//! cut.
use camlib::{
    motion::Event,
    nesting::{NestPart, NestRequest, NestResult, Placement, nest, verify},
    planar::{Boundary, Component, PlanarRegion},
    stock::StockSpec,
    thermal::{ThermalCutOperation, ThermalCutPlan},
    tool::{Cutter, CutterShape, HolderSection, ToolAssembly},
};
use std::sync::atomic::AtomicBool;

const KERF: f64 = 1.5;

fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Boundary {
    Boundary {
        vertices: vec![[x0, y0, 0.], [x1, y0, 0.], [x1, y1, 0.], [x0, y1, 0.]],
    }
}
fn circle(c: [f64; 2], r: f64) -> Boundary {
    Boundary {
        vertices: vec![[c[0] + r, c[1], 1.], [c[0] - r, c[1], 1.]],
    }
}
fn region(outer: Boundary, holes: Vec<Boundary>) -> PlanarRegion {
    PlanarRegion {
        version: 1,
        units: "mm".into(),
        name: "part".into(),
        boundary_error_mm: 0.,
        components: vec![Component { outer, holes }],
        source: serde_json::Value::Null,
    }
}
fn part(region: PlanarRegion, quantity: u32, rotations_deg: &[f64]) -> NestPart {
    NestPart {
        region,
        quantity,
        rotations_deg: rotations_deg.to_vec(),
        mirror: false,
    }
}
/// A plate with a large round hole, a slot with rounded ends and a bolt hole.
fn plate() -> PlanarRegion {
    region(
        rect(0., 0., 100., 80.),
        vec![
            circle([40., 40.], 27.),
            Boundary {
                vertices: vec![[80., 20., 0.], [80., 60., -1.], [90., 60., 0.], [90., 20., -1.]],
            },
            circle([10., 10.], 4.),
        ],
    )
}
fn washer() -> PlanarRegion {
    region(circle([0., 0.], 12.), vec![circle([0., 0.], 5.)])
}
/// A bracket with a filleted inner corner (an arc in its outline) and a
/// chamfer.
fn bracket() -> PlanarRegion {
    region(
        Boundary {
            vertices: vec![
                [0., 0., 0.],
                [60., 0., 0.],
                [60., 15., 0.],
                [25., 15., -(std::f64::consts::PI / 8.).tan()],
                [15., 25., 0.],
                [15., 50., 0.],
                [5., 50., 0.],
                [0., 45., 0.],
            ],
        },
        vec![circle([8., 8.], 3.)],
    )
}
fn request(sheet: PlanarRegion, parts: Vec<NestPart>) -> NestRequest {
    serde_json::from_value(serde_json::json!({
        "sheet": sheet, "parts": parts, "kerf_mm": KERF,
        "lead_in": {"length_mm": 2., "radius_mm": 3.},
        "lead_out": {"length_mm": 1., "radius_mm": 2.},
        "part_spacing_mm": 2., "edge_margin_mm": 5., "resolution_mm": 1.,
        "tolerance_mm": 1e-5
    }))
    .unwrap()
}
fn run(request: &NestRequest) -> Result<NestResult, String> {
    nest(request, &AtomicBool::new(false))
}
fn mixed() -> NestRequest {
    request(
        region(rect(0., 0., 400., 250.), vec![]),
        vec![
            part(plate(), 2, &[0., 90.]),
            part(washer(), 6, &[0.]),
            part(bracket(), 5, &[0., 90., 180., 270.]),
        ],
    )
}

/// What the layout must keep, computed here from the declared values:
/// everything cut for a part lies within kerf + allowance + lead reach.
fn cut_reach(r: &NestRequest) -> f64 {
    let reach = |radius: f64, length: f64| radius.hypot(radius + length);
    let lead = (reach(r.lead_in.radius_mm, r.lead_in.length_mm)
        + r.pierce_clearance_mm.unwrap_or(r.kerf_mm))
    .max(reach(r.lead_out.radius_mm, r.lead_out.length_mm));
    r.kerf_mm + r.boundary_tolerance_mm / 2. + lead
}

#[test]
fn parts_with_holes_and_arcs_nest_with_the_declared_distances_proven() {
    let request = mixed();
    let result = run(&request).unwrap();
    assert_eq!(result.strategy, "bottom_left_fill");
    assert!(result.unplaced.is_empty());
    assert_eq!(result.placements.len(), 13);
    assert_eq!(result.region.components.len(), 13);
    // Every copy once, parts in the declared order.
    let copies: Vec<(usize, u32)> = result.placements.iter().map(|p| (p.part, p.copy)).collect();
    let expected: Vec<(usize, u32)> = [(0, 2), (1, 6), (2, 5)]
        .into_iter()
        .flat_map(|(part, n)| (0..n).map(move |c| (part, c)))
        .collect();
    assert_eq!(copies, expected);
    let reach = cut_reach(&request);
    let check = &result.check;
    assert_eq!(check.status, "proven");
    assert!((check.part_distance_mm - (2. * reach + 2.)).abs() < 1e-12);
    assert!((check.sheet_distance_mm - (reach + 5.)).abs() < 1e-12);
    assert!(check.min_part_distance_mm.unwrap() >= check.part_distance_mm);
    assert!(check.min_sheet_distance_mm.unwrap() >= check.sheet_distance_mm);
    // The planner wastes little beyond the requirement at the tightest pair.
    assert!(check.min_part_distance_mm.unwrap() < check.part_distance_mm + 0.5);
    // Utilisation is the placed area over the sheet area.
    let plate_area = 100. * 80.
        - std::f64::consts::PI * (27. * 27. + 16.)
        - (10. * 40. + std::f64::consts::PI * 25.);
    let washer_area = std::f64::consts::PI * (144. - 25.);
    // Its polygon, less the fillet's segment (radius 10, a quarter turn)
    // and the hole.
    let bracket_area =
        1462.5 - 100. * (std::f64::consts::FRAC_PI_2 - 1.) / 2. - std::f64::consts::PI * 9.;
    let placed = 2. * plate_area + 6. * washer_area + 5. * bracket_area;
    assert!((result.placed_area_mm2 - placed).abs() < 1e-6, "{}", result.placed_area_mm2);
    assert!((result.sheet_area_mm2 - 100_000.).abs() < 1e-9);
    assert!((result.utilisation - placed / 100_000.).abs() < 1e-12);
    let used = result.used_bounds_mm.unwrap();
    assert!(used[0] >= reach + 5. && used[1] >= reach + 5.);
    assert!(used[2] <= 400. - reach - 5. && used[3] <= 250. - reach - 5.);
    // The independent check accepts the layout on its own, too.
    assert_eq!(verify(&request, &result.placements, &result.region).unwrap(), *check);
}

#[test]
fn the_same_request_gives_the_same_nest() {
    let mut request = mixed();
    let first = run(&request).unwrap();
    let again = run(&request).unwrap();
    assert_eq!(first.placements, again.placements);
    assert_eq!(first.region, again.region);
    // With a search: the same seed gives the same nest, and never a worse one.
    request.search_iterations = 12;
    request.seed = 42;
    let searched = run(&request).unwrap();
    let repeated = run(&request).unwrap();
    assert_eq!(searched.placements, repeated.placements);
    assert_eq!(searched.search_improvements, repeated.search_improvements);
    assert_eq!(searched.search_iterations, 12);
    assert!(searched.used_bounds_mm.unwrap()[2] <= first.used_bounds_mm.unwrap()[2] + 1e-9);
    assert_eq!(searched.check.status, "proven");
}

#[test]
fn washers_are_placed_in_a_plate_hole_only_when_proven_and_allowed() {
    // A sheet with room for the plate alone: the washer fits only in the
    // plate's 27 mm hole (12 mm washer + the part distance).
    let mut request = request(
        region(rect(0., 0., 132., 112.), vec![]),
        vec![part(plate(), 1, &[0.]), part(washer(), 1, &[0.])],
    );
    request.lead_in = serde_json::from_value(serde_json::json!({"length_mm": 0., "radius_mm": 1.})).unwrap();
    request.lead_out = request.lead_in;
    request.pierce_clearance_mm = Some(1.);
    request.part_spacing_mm = 1.;
    let result = run(&request).unwrap();
    assert_eq!(result.check.parts_in_holes, 1);
    let washer_at = result.placements[1].translation_mm;
    let plate_at = result.placements[0].translation_mm;
    let center = [plate_at[0] + 40., plate_at[1] + 40.];
    let off = (washer_at[0] - center[0]).hypot(washer_at[1] - center[1]);
    // Inside the hole with the required distance to its edge.
    assert!(off + 12. + result.check.part_distance_mm <= 27. + 1e-9, "{off}");
    // Not allowed: the washer has nowhere to go and the nest is refused.
    request.part_in_part = false;
    let error = run(&request).unwrap_err();
    assert!(error.contains("do not fit") && error.contains("part 1: 1 of 1"), "{error}");
    // And the check refuses the hole layout under that rule.
    let error = verify(&request, &result.placements, &result.region).unwrap_err();
    assert!(error.contains("lies in a hole"), "{error}");
}

#[test]
fn parts_that_do_not_fit_are_refused_or_listed_never_dropped() {
    let mut request = request(
        region(rect(0., 0., 150., 120.), vec![]),
        vec![part(bracket(), 9, &[0.]), part(washer(), 3, &[0.])],
    );
    let error = run(&request).unwrap_err();
    assert!(error.contains("do not fit on the sheet"), "{error}");
    assert!(error.contains("allow_unplaced"), "{error}");
    request.allow_unplaced = true;
    let result = run(&request).unwrap();
    assert!(!result.unplaced.is_empty());
    for (i, part) in request.parts.iter().enumerate() {
        let placed = result.placements.iter().filter(|p| p.part == i).count() as u32;
        let left = result.unplaced.iter().find(|u| u.part == i).map_or(0, |u| u.quantity);
        assert_eq!(placed + left, part.quantity, "part {i}");
    }
    assert!(result.diagnostics.iter().any(|d| d.contains("unplaced")));
    assert_eq!(result.check.status, "proven");
    // A part larger than the sheet: nothing placed, everything listed.
    let big = request_big();
    let result = run(&big).unwrap();
    assert!(result.placements.is_empty() && result.region.components.is_empty());
    assert_eq!(result.unplaced[0].quantity, 2);
    assert_eq!(result.utilisation, 0.);
}
fn request_big() -> NestRequest {
    let mut r = request(
        region(rect(0., 0., 90., 90.), vec![]),
        vec![part(plate(), 2, &[0., 90.])],
    );
    r.allow_unplaced = true;
    r
}

#[test]
fn rotations_and_mirror_are_used_only_as_declared() {
    // A 30 x 120 strip on a 160 x 62 sheet fits only when turned.
    let strip = region(rect(0., 0., 30., 120.), vec![]);
    let sheet = region(rect(0., 0., 160., 62.), vec![]);
    let fixed = request(sheet.clone(), vec![part(strip.clone(), 1, &[0.])]);
    assert!(run(&fixed).unwrap_err().contains("do not fit"));
    let turned = request(sheet, vec![part(strip, 1, &[0., 90.])]);
    let result = run(&turned).unwrap();
    assert_eq!(result.placements[0].rotation_deg, 90.);
    // Quarter turns are exact: the strip's corners are whole coordinates
    // moved by the translation.
    let t = result.placements[0].translation_mm;
    assert_eq!(result.region.components[0].outer.vertices[2], [t[0] - 120., t[1] + 30., 0.]);

    // An L-shaped sheet corner takes the bracket only mirrored.
    let mut mirrored = request(
        region(rect(0., 0., 400., 250.), vec![]),
        vec![part(bracket(), 1, &[0.])],
    );
    mirrored.parts[0].mirror = true;
    let result = run(&mirrored).unwrap();
    // Both images were candidates; whichever was taken, the check proved it
    // a rigid (mirrored) copy, and a forged flag is refused.
    let mut forged = result.placements.clone();
    forged[0].mirrored = !forged[0].mirrored;
    let error = verify(&mirrored, &forged, &result.region).unwrap_err();
    assert!(error.contains("vertices and arcs") || error.contains("rigid copy"), "{error}");
    mirrored.parts[0].mirror = false;
    let plain = run(&mirrored).unwrap();
    assert!(!plain.placements[0].mirrored);
    let mut flipped = plain.placements.clone();
    flipped[0].mirrored = true;
    assert!(verify(&mirrored, &flipped, &plain.region).unwrap_err().contains("does not allow"));
}

#[test]
fn a_remnant_sheet_keeps_parts_off_its_holes_and_notches() {
    // An L-shaped remnant with a round hole already cut out of it.
    let sheet = region(
        Boundary {
            vertices: vec![
                [0., 0., 0.],
                [300., 0., 0.],
                [300., 120., 0.],
                [150., 120., 0.],
                [150., 250., 0.],
                [0., 250., 0.],
            ],
        },
        vec![circle([70., 70.], 30.)],
    );
    let request = request(
        sheet,
        vec![part(bracket(), 6, &[0., 90., 180., 270.]), part(washer(), 4, &[0.])],
    );
    let result = run(&request).unwrap();
    assert!(result.unplaced.is_empty());
    assert!(result.check.min_sheet_distance_mm.unwrap() >= result.check.sheet_distance_mm);
    assert!((result.sheet_area_mm2 - (300. * 120. + 150. * 130. - std::f64::consts::PI * 900.)).abs() < 1e-6);
}

fn shifted(result: &NestResult, k: usize, by: [f64; 2]) -> (Vec<Placement>, PlanarRegion) {
    let mut placements = result.placements.clone();
    placements[k].translation_mm[0] += by[0];
    placements[k].translation_mm[1] += by[1];
    let mut region = result.region.clone();
    let c = &mut region.components[k];
    for v in c.outer.vertices.iter_mut().chain(c.holes.iter_mut().flat_map(|h| &mut h.vertices)) {
        v[0] += by[0];
        v[1] += by[1];
    }
    (placements, region)
}

#[test]
fn the_independent_check_refuses_layouts_that_break_a_distance() {
    let request = request(
        // One row only: the plates lie side by side.
        region(rect(0., 0., 300., 120.), vec![]),
        vec![part(plate(), 2, &[0.]), part(washer(), 2, &[0.])],
    );
    let mut request = request;
    request.part_in_part = false;
    let result = run(&request).unwrap();
    let need = result.check.part_distance_mm;
    let found = result.check.min_part_distance_mm.unwrap();
    // Move the second plate toward the first until they are just too close.
    let (a, b) = (result.placements[0].translation_mm, result.placements[1].translation_mm);
    let d = [a[0] - b[0], a[1] - b[1]];
    let unit = [d[0] / d[0].hypot(d[1]), d[1] / d[0].hypot(d[1])];
    assert!(unit[0].abs() == 1. || unit[1].abs() == 1., "plates are axis-aligned neighbours: {a:?} {b:?}");
    let gap = if unit[0] != 0. { d[0].abs() - 100. } else { d[1].abs() - 80. };
    assert!(gap >= need && found >= need);
    let step = gap - need + 0.001;
    let (placements, region) = shifted(&result, 1, [unit[0] * step, unit[1] * step]);
    let error = verify(&request, &placements, &region).unwrap_err();
    assert!(error.contains("apart"), "{error}");
    // Just inside the requirement is still accepted.
    let step = gap - need - 0.001;
    let (placements, region) = shifted(&result, 1, [unit[0] * step, unit[1] * step]);
    let near = verify(&request, &placements, &region).unwrap();
    let least = near.min_part_distance_mm.unwrap();
    assert!(least >= need && least <= need + 0.001 + 1e-9, "{least}");
    // One plate wholly inside the other's outline... is impossible for equal
    // plates, so drop a washer onto a plate's material instead.
    let washer = result.placements[2].translation_mm;
    let onto = [a[0] + 85. - washer[0], a[1] + 70. - washer[1]];
    let (placements, region) = shifted(&result, 2, onto);
    assert!(verify(&request, &placements, &region).is_err());
    // Off the sheet's margin, and off the sheet altogether.
    let (placements, region) = shifted(&result, 0, [-(a[0] - 2.), 0.]);
    assert!(verify(&request, &placements, &region).unwrap_err().contains("sheet's edge"));
    let (placements, region) = shifted(&result, 0, [-1000., 0.]);
    assert!(verify(&request, &placements, &region).unwrap_err().contains("not on the sheet"));
    // A region that is not the placed part (stretched) is refused.
    let mut region = result.region.clone();
    region.components[0].outer.vertices[1][0] += 0.01;
    assert!(verify(&request, &result.placements, &region).unwrap_err().contains("rigid copy"));
    // A missing or duplicated copy is refused.
    let mut placements = result.placements.clone();
    placements[1].copy = 0;
    assert!(verify(&request, &placements, &result.region).unwrap_err().contains("placed twice"));
}

#[test]
fn invalid_requests_are_refused_with_the_reason() {
    let base = || request(region(rect(0., 0., 300., 200.), vec![]), vec![part(washer(), 1, &[0.])]);
    let refused = |change: fn(&mut NestRequest), text: &str| {
        let mut r = base();
        change(&mut r);
        let error = run(&r).unwrap_err();
        assert!(error.contains(text), "{error}");
    };
    refused(|r| r.kerf_mm = 0., "kerf_mm");
    refused(|r| r.resolution_mm = 0., "resolution_mm");
    refused(
        |r| {
            r.sheet = region(rect(0., 0., 2000., 200.), vec![]);
            r.resolution_mm = 0.001;
        },
        "million columns",
    );
    refused(|r| r.part_spacing_mm = -1., "part_spacing_mm");
    refused(|r| r.parts[0].quantity = 0, "quantity");
    refused(|r| r.parts[0].rotations_deg.clear(), "rotations_deg");
    refused(|r| r.parts.clear(), "no parts");
    refused(
        |r| {
            let extra = r.parts[0].region.components[0].clone();
            r.parts[0].region.components.push(extra);
        },
        "one outline",
    );
    // Rotations, spacing, margin and the kerf have no defaults.
    for key in ["kerf_mm", "part_spacing_mm", "edge_margin_mm", "resolution_mm", "lead_in", "sheet"] {
        let mut value = serde_json::to_value(base()).unwrap();
        value.as_object_mut().unwrap().remove(key);
        assert!(serde_json::from_value::<NestRequest>(value).is_err(), "{key}");
    }
    let mut value = serde_json::to_value(base()).unwrap();
    value["parts"][0].as_object_mut().unwrap().remove("rotations_deg");
    assert!(serde_json::from_value::<NestRequest>(value).is_err());
    // Cancelled before it starts.
    assert!(nest(&base(), &AtomicBool::new(true)).unwrap_err().contains("cancelled"));
}

fn torch() -> ToolAssembly {
    ToolAssembly {
        id: "torch".into(),
        cutter: Cutter {
            id: "nozzle".into(),
            name: String::new(),
            shape: CutterShape::Beam,
            diameter_mm: KERF,
            cutting_length_mm: 1.,
            overall_length_mm: 40.,
            shank_diameter_mm: 8.,
            flutes: 1,
            center_cutting: true,
            material: String::new(),
            neck: None,
        },
        stickout_mm: 30.,
        gauge_length_mm: 60.,
        holders: vec![HolderSection {
            z_min_mm: 30.,
            z_max_mm: 60.,
            radius_min_mm: 15.,
            radius_max_mm: 15.,
        }],
    }
}

/// The distance from a point to a placed part's material (zero inside).
fn distance_to_part(p: [f64; 2], c: &Component) -> f64 {
    // Dense points along every loop: a test oracle, not the library's check.
    let mut best = f64::INFINITY;
    for b in std::iter::once(&c.outer).chain(&c.holes) {
        let n = b.vertices.len();
        for i in 0..n {
            let (a, q) = (b.vertices[i], b.vertices[(i + 1) % n]);
            let sweep = 4. * a[2].atan();
            for k in 0..=200 {
                let t = k as f64 / 200.;
                let point = if a[2] == 0. {
                    [a[0] + t * (q[0] - a[0]), a[1] + t * (q[1] - a[1])]
                } else {
                    let h = (1. - a[2] * a[2]) / (4. * a[2]);
                    let c = [
                        (a[0] + q[0]) / 2. - (q[1] - a[1]) * h,
                        (a[1] + q[1]) / 2. + (q[0] - a[0]) * h,
                    ];
                    let r = (a[0] - c[0]).hypot(a[1] - c[1]);
                    let angle = (a[1] - c[1]).atan2(a[0] - c[0]) + t * sweep;
                    [c[0] + r * angle.cos(), c[1] + r * angle.sin()]
                };
                best = best.min((p[0] - point[0]).hypot(p[1] - point[1]));
            }
        }
    }
    best
}

#[test]
fn a_nest_feeds_a_thermal_cut_whose_leads_stay_in_their_reserved_room() {
    let request = mixed();
    let result = run(&request).unwrap();
    let operation: ThermalCutOperation = serde_json::from_value(serde_json::json!({
        "id": "plasma", "tool": "torch", "parts": result.region, "feed_mm_min": 2500.,
        "cut_height_mm": 1.5, "pierce_delay_s": 0.5,
        "lead_in": request.lead_in, "lead_out": request.lead_out,
        "tolerance_mm": 1e-5, "work_offset": 1, "tool_number": 1, "length_register": 1
    }))
    .unwrap();
    let spec = StockSpec {
        version: 1,
        units: "mm".into(),
        min_mm: [0., 0., 0.],
        max_mm: [400., 250., 6.],
        cell_size_mm: 0.5,
        max_cells: 1 << 22,
    };
    let cancel = AtomicBool::new(false);
    // The cut plans (its own proofs: every lead, kerf path and pierce crater
    // clear of every part).
    let mut plan =
        ThermalCutPlan::prepare(operation, &torch(), &spec, [0., 0., 40.], 1, &cancel).unwrap();
    let mut stock = camlib::stock::Stock::new(spec).unwrap();
    let prepared = torch().prepare().unwrap();
    // Which loop each cutting move belongs to, from the report's order.
    let mut cutting = vec![];
    let mut beam = false;
    let mut events = plan.prefix_events();
    events.extend(plan.take_events());
    while let Some(m) = plan.next_motion_with_stock(&stock, &cancel).unwrap() {
        let delta = stock
            .preview_sweep(&prepared, &m, 1., camlib::stock::SweepOptions::default(), &cancel)
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        events.push(Event::Motion { motion: m });
        events.extend(plan.take_events());
    }
    let mut loops = 0;
    for e in &events {
        match e {
            Event::Beam { power_percent } => {
                let on = *power_percent > 0.;
                if on && !beam {
                    loops += 1;
                }
                beam = on;
            }
            Event::Motion { motion } if beam => cutting.push((loops - 1, motion.curve.clone())),
            _ => {}
        }
    }
    let order = plan.report().order.clone();
    assert_eq!(loops, order.len());
    assert!(!cutting.is_empty());
    // Every beam-on point (leads, pierce points and loops) stays within the
    // reserved reach of its own part: kerf center within reach - kerf / 2.
    let reach = cut_reach(&request);
    let (mut worst, mut leads_beyond_kerf) = (0f64, 0);
    for (l, curve) in &cutting {
        let part = &result.region.components[order[*l].0];
        for p in [curve.start(), curve.end()] {
            let d = distance_to_part([p[0], p[1]], part);
            worst = worst.max(d);
            leads_beyond_kerf += usize::from(d > KERF);
            assert!(d <= reach - KERF / 2. + 1e-6, "loop {l}: {d} > {reach}");
            // And on the sheet, inside the edge margin.
            assert!(p[0] >= 5. && p[0] <= 395. && p[1] >= 5. && p[1] <= 245., "{p:?}");
        }
    }
    assert!(leads_beyond_kerf > 0 && worst > KERF, "leads were exercised: {worst}");
}
