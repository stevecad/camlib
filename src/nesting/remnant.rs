//! The remnant of a sheet after a thermal cut: what is left to cut the next
//! job from.
//!
//! Everything a thermal cut removes for a part lies within
//! `kerf + allowance + lead reach` of the part (the module documentation of
//! `nesting`: the kerf beside the part, its leads and its pierce craters).
//! The remnant is the sheet less every part's outline grown by that distance
//! and by the declared `remnant_margin_mm` (for the heat-affected edge, dross
//! and how exactly the sheet is put back): a part's own area, its holes'
//! slugs and anything nested inside it are gone with it. The band is taken
//! all round each part, also where no lead was placed, so the remnant never
//! claims material the cut may have removed; it can leave out a little that
//! is still there.
//!
//! The result is checked without the construction: sample points of every
//! part's outline must lie at least the grown distance outside the remnant,
//! and the remnant's own corners must lie on the sheet.
use super::{Lead, cut_reach};
use crate::Verdict;
use crate::planar::{Boolean, Boundary, Component, ConstructionOptions, PlanarRegion};
use serde::{Deserialize, Serialize};

fn boundary_tolerance() -> f64 {
    0.01
}

/// The tolerance the remnant's outline is constructed to.
const REMNANT_TOLERANCE_MM: f64 = 0.01;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RemnantRequest {
    /// The sheet that was cut: one outline, with holes where it was already
    /// a remnant.
    pub sheet: PlanarRegion,
    /// The parts that were cut from it (the thermal cut's `parts`).
    pub parts: PlanarRegion,
    pub kerf_mm: f64,
    pub lead_in: Lead,
    pub lead_out: Lead,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pierce_clearance_mm: Option<f64>,
    /// The thermal cut's corner loops: what they cut is removed too.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub corner_loops: Option<crate::thermal::CornerLoops>,
    #[serde(default = "boundary_tolerance")]
    pub boundary_tolerance_mm: f64,
    pub tolerance_mm: f64,
    /// Material given up beyond everything the cut can have removed. No
    /// default: it is the shop's judgement of its edges and its fixturing.
    pub remnant_margin_mm: f64,
    /// The crop cut made after the parts (the thermal cut's `crop`): its
    /// kerf and pierce crater are removed too, so an offcut it frees is a
    /// piece of its own.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub crop: Option<crate::thermal::CropCut>,
}

/// A box round the segment a-b, `r` to each side and past each end.
fn band(a: [f64; 2], b: [f64; 2], r: f64) -> Boundary {
    let length = (b[0] - a[0]).hypot(b[1] - a[1]);
    let t = [(b[0] - a[0]) / length, (b[1] - a[1]) / length];
    let n = [-t[1], t[0]];
    let corner = |p: [f64; 2], along: f64, across: f64| {
        [
            p[0] + along * r * t[0] + across * r * n[0],
            p[1] + along * r * t[1] + across * r * n[1],
            0.,
        ]
    };
    Boundary {
        vertices: vec![
            corner(a, -1., -1.),
            corner(b, 1., -1.),
            corner(b, 1., 1.),
            corner(a, -1., 1.),
        ],
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct RemnantCheck {
    pub status: &'static str,
    /// Points of the parts' outlines tested against the remnant.
    pub part_points: usize,
    /// The least distance from a tested point to the remnant.
    pub min_part_distance_mm: f64,
    /// The distance required: cut reach plus margin.
    pub required_distance_mm: f64,
    /// Points of the crop cut tested, the least distance found from one to
    /// the remnant, and the distance required of them (half the kerf, the
    /// pierce clearance and the margin).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub crop: Option<(usize, f64, f64)>,
}

#[derive(Clone, Debug, Serialize)]
pub struct Remnant {
    /// What is left, in the sheet's own coordinates: one component per
    /// piece, largest first. Empty when nothing is left.
    pub region: PlanarRegion,
    pub sheet_area_mm2: f64,
    pub area_mm2: f64,
    /// How far from a part the cut can have removed material.
    pub cut_reach_mm: f64,
    pub remnant_margin_mm: f64,
    pub check: RemnantCheck,
    pub diagnostics: Vec<String>,
}

/// Points along a boundary: its vertices and, on arcs, points no more than a
/// sixteenth of a turn apart.
fn samples(b: &Boundary) -> Vec<[f64; 2]> {
    let n = b.vertices.len();
    let mut out = vec![];
    for i in 0..n {
        let (a, c) = (b.vertices[i], b.vertices[(i + 1) % n]);
        out.push([a[0], a[1]]);
        if a[2] != 0. {
            let sweep = 4. * a[2].atan();
            let chord = (c[0] - a[0]).hypot(c[1] - a[1]);
            if chord == 0. {
                continue;
            }
            let radius = chord / (2. * (sweep.abs() / 2.).sin());
            let offset = radius * (sweep / 2.).cos() * sweep.signum();
            let normal = [-(c[1] - a[1]) / chord, (c[0] - a[0]) / chord];
            let center = [
                (a[0] + c[0]) / 2. + normal[0] * offset,
                (a[1] + c[1]) / 2. + normal[1] * offset,
            ];
            let start = (a[1] - center[1]).atan2(a[0] - center[0]);
            let steps = (sweep.abs() / (std::f64::consts::PI / 8.)).ceil().max(2.) as usize;
            for k in 1..steps {
                let angle = start + sweep * k as f64 / steps as f64;
                out.push([
                    center[0] + radius * angle.cos(),
                    center[1] + radius * angle.sin(),
                ]);
            }
        }
    }
    out
}

/// The remnant of `request.sheet` after `request.parts` are cut from it.
pub fn remnant(request: &RemnantRequest) -> Result<Remnant, String> {
    let ok = |v: f64, max: f64| v.is_finite() && (0. ..=max).contains(&v);
    crate::heights::require(
        "remnant",
        [
            (
                request.kerf_mm.is_finite() && request.kerf_mm > 0. && request.kerf_mm <= 1e3,
                format!(
                    "kerf_mm {} must be positive (the beam tool's kerf width)",
                    request.kerf_mm
                ),
            ),
            (
                [request.lead_in, request.lead_out]
                    .iter()
                    .all(|l| ok(l.length_mm, 1e3) && ok(l.radius_mm, 1e3)),
                "lead lengths and radii must not be negative".into(),
            ),
            (
                request.pierce_clearance_mm.is_none_or(|c| ok(c, 1e3)),
                "pierce_clearance_mm must not be negative".into(),
            ),
            (
                ok(request.remnant_margin_mm, 1e3),
                format!(
                    "remnant_margin_mm {} must be zero or more (material given up round every cut: the heat-affected edge, dross, and how exactly the sheet is put back)",
                    request.remnant_margin_mm
                ),
            ),
            (
                ok(request.boundary_tolerance_mm, 1e3),
                "boundary_tolerance_mm must not be negative".into(),
            ),
            (
                request.tolerance_mm.is_finite() && request.tolerance_mm > 0.,
                "tolerance_mm must be positive".into(),
            ),
            (
                request.sheet.components.len() == 1,
                format!(
                    "the sheet must be one outline (with holes); it has {} components: take the remnant of each piece on its own",
                    request.sheet.components.len()
                ),
            ),
        ],
    )?;
    // The remnant is built to REMNANT_TOLERANCE_MM, not to the cut's own
    // tolerance: its curved edges are then a few hundred chords instead of
    // tens of thousands, in a file that is drawn and nested on again. The
    // parts are grown by eight times that first, so the coarser edge only
    // ever gives material up.
    let tolerance = request.tolerance_mm.max(REMNANT_TOLERANCE_MM);
    let options = ConstructionOptions {
        tolerance_mm: tolerance,
    };
    let sheet = request
        .sheet
        .prepare(options)
        .map_err(|e| format!("remnant sheet: {e}"))?;
    let sheet_area_mm2 = super::component_area(&request.sheet.components[0]);
    let (_, cut_reach_mm) = cut_reach(
        request.kerf_mm,
        request.parts.boundary_error_mm + request.sheet.boundary_error_mm,
        request.boundary_tolerance_mm,
        &request.lead_in,
        &request.lead_out,
        request.pierce_clearance_mm,
        request.corner_loops.as_ref(),
    );
    let required = cut_reach_mm + request.remnant_margin_mm;
    let empty = |components| PlanarRegion {
        version: 1,
        units: "mm".into(),
        name: format!("{}-remnant", request.sheet.name),
        boundary_error_mm: 0.,
        components,
        source: serde_json::json!({
            "remnant_of": request.sheet.name,
            "frame": "the cut sheet's own coordinates",
            "cut_reach_mm": cut_reach_mm,
            "remnant_margin_mm": request.remnant_margin_mm,
        }),
    };
    let mut diagnostics = vec![];
    let mut crop_check: Option<(usize, f64, f64)> = None;
    if request.parts.components.is_empty() && request.crop.is_none() {
        let mut region = empty(request.sheet.components.clone());
        region.boundary_error_mm = request.sheet.boundary_error_mm;
        return Ok(Remnant {
            region,
            sheet_area_mm2,
            area_mm2: sheet_area_mm2,
            cut_reach_mm,
            remnant_margin_mm: request.remnant_margin_mm,
            check: RemnantCheck {
                status: "nothing_cut",
                part_points: 0,
                min_part_distance_mm: f64::INFINITY,
                required_distance_mm: required,
                crop: crop_check,
            },
            diagnostics,
        });
    }
    // The parts as solids (a part goes with its holes' slugs and whatever was
    // nested inside it), each grown by everything its cut can reach and the
    // margin, a little more than the construction's own tolerance, and
    // united. A part inside another part's outline adds nothing.
    let slack = 8. * tolerance;
    let mut removed: Option<crate::planar::PreparedRegion> = None;
    for (i, c) in request.parts.components.iter().enumerate() {
        let solid = PlanarRegion {
            version: 1,
            units: "mm".into(),
            name: format!("part {i}"),
            boundary_error_mm: request.parts.boundary_error_mm,
            components: vec![Component {
                outer: c.outer.clone(),
                holes: vec![],
            }],
            source: serde_json::Value::Null,
        };
        let grown = solid
            .prepare(options)
            .map_err(|e| format!("remnant: part {i}: {e}"))?
            .offset_prepared(-(required + slack))
            .map_err(|e| format!("remnant: part {i}: {e}"))?
            .into_parts()
            .1;
        removed = Some(match removed {
            None => grown,
            Some(all) => {
                all.boolean_prepared(&grown, Boolean::Union)
                    .map_err(|e| format!("remnant: uniting what the cut removed (part {i}): {e}"))?
                    .into_parts()
                    .1
            }
        });
    }
    // The crop cut's kerf and its pierce crater, as boxes round its
    // segments.
    let crop_required = request.kerf_mm / 2.
        + request.pierce_clearance_mm.unwrap_or(request.kerf_mm)
        + request.remnant_margin_mm;
    if let Some(crop) = &request.crop {
        let points = &crop.points_mm;
        if !(2..=3).contains(&points.len())
            || !points
                .iter()
                .flatten()
                .all(|v| v.is_finite() && v.abs() <= 1e6)
            || points
                .windows(2)
                .any(|w| (w[0][0] - w[1][0]).hypot(w[0][1] - w[1][1]) <= request.kerf_mm)
        {
            return Err("remnant: crop needs two points (a straight cut) or three (an L), each more than a kerf from the next".into());
        }
        for (i, w) in points.windows(2).enumerate() {
            let cut = PlanarRegion {
                version: 1,
                units: "mm".into(),
                name: format!("crop {i}"),
                boundary_error_mm: 0.,
                components: vec![Component {
                    outer: band(w[0], w[1], crop_required + slack),
                    holes: vec![],
                }],
                source: serde_json::Value::Null,
            }
            .prepare(options)
            .map_err(|e| format!("remnant: crop cut: {e}"))?;
            removed = Some(match removed {
                None => cut,
                Some(all) => {
                    all.boolean_prepared(&cut, Boolean::Union)
                        .map_err(|e| format!("remnant: uniting what the crop cut removed: {e}"))?
                        .into_parts()
                        .1
                }
            });
        }
    }
    let removed = removed.expect("parts or a crop cut");
    let left = sheet
        .boolean_prepared(&removed, Boolean::Difference)
        .map_err(|e| format!("remnant: the sheet less what the cut removed: {e}"))?;
    let (report, prepared) = left.into_parts();
    let mut components = report.region.components;
    components.sort_by(|a, b| super::component_area(b).total_cmp(&super::component_area(a)));
    let area_mm2: f64 = components.iter().map(super::component_area).sum();
    let mut region = empty(components);
    region.boundary_error_mm = report.region.boundary_error_mm;
    // The check: points of every part's outline lie the required distance
    // outside the remnant; the remnant's corners lie on the sheet.
    let uncertainty = region.boundary_error_mm + tolerance;
    let mut part_points = 0;
    let mut least = f64::INFINITY;
    if !region.components.is_empty() {
        for (i, c) in request.parts.components.iter().enumerate() {
            for p in samples(&c.outer) {
                let q = prepared.query(p)?;
                let distance = -q.signed_clearance_mm.unwrap_or(f64::NEG_INFINITY);
                part_points += 1;
                least = least.min(distance);
                // The full distance, with no allowance for the construction:
                // the slack it was grown by covers that.
                if matches!(q.verdict, Verdict::Allowed) || distance < required {
                    return Err(format!(
                        "remnant: the construction failed its check and is not returned: part {i} at ({:.4}, {:.4}) is {distance:.4} mm from the remnant, less than the {required:.4} mm its cut can reach with the margin; this is a defect: report it",
                        p[0], p[1]
                    ));
                }
            }
        }
        if let Some(crop) = &request.crop {
            let (mut points, mut nearest) = (0, f64::INFINITY);
            for w in crop.points_mm.windows(2) {
                let length = (w[0][0] - w[1][0]).hypot(w[0][1] - w[1][1]);
                let steps = (length / crop_required.max(tolerance))
                    .ceil()
                    .clamp(1., 1e5) as usize;
                for k in 0..=steps {
                    let f = k as f64 / steps as f64;
                    let p = [
                        w[0][0] + f * (w[1][0] - w[0][0]),
                        w[0][1] + f * (w[1][1] - w[0][1]),
                    ];
                    let q = prepared.query(p)?;
                    let distance = -q.signed_clearance_mm.unwrap_or(f64::NEG_INFINITY);
                    points += 1;
                    nearest = nearest.min(distance);
                    if matches!(q.verdict, Verdict::Allowed) || distance < crop_required {
                        return Err(format!(
                            "remnant: the construction failed its check and is not returned: the crop cut at ({:.4}, {:.4}) is {distance:.4} mm from the remnant, less than the {crop_required:.4} mm it removes with the margin; this is a defect: report it",
                            p[0], p[1]
                        ));
                    }
                }
            }
            crop_check = Some((points, nearest, crop_required));
        }
        let sheet_uncertainty = request.sheet.boundary_error_mm + 2. * uncertainty;
        for c in &region.components {
            for b in std::iter::once(&c.outer).chain(&c.holes) {
                for v in &b.vertices {
                    let q = sheet.query([v[0], v[1]])?;
                    if q.signed_clearance_mm.is_none_or(|d| d < -sheet_uncertainty) {
                        return Err(format!(
                            "remnant: the construction failed its check and is not returned: the remnant reaches ({:.4}, {:.4}), off the sheet; this is a defect: report it",
                            v[0], v[1]
                        ));
                    }
                }
            }
        }
    }
    if region.components.len() > 1 {
        diagnostics.push(format!(
            "the remnant is {} separate pieces; nest on one piece at a time",
            region.components.len()
        ));
    }
    if region.components.is_empty() {
        diagnostics.push("nothing is left of the sheet beyond the cuts and the margin".into());
    }
    Ok(Remnant {
        region,
        sheet_area_mm2,
        area_mm2,
        cut_reach_mm,
        remnant_margin_mm: request.remnant_margin_mm,
        check: RemnantCheck {
            status: "checked_at_points",
            part_points,
            min_part_distance_mm: least,
            required_distance_mm: required,
            crop: crop_check,
        },
        diagnostics,
    })
}
