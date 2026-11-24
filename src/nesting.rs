//! Automatic nesting of sheet parts for thermal cutting (plasma, laser).
//!
//! A nest places copies of parts (each an outline with holes) on a sheet (an
//! outline with remnant holes) and returns their placements and the placed
//! parts as one planar region, ready to be the `parts` of a thermal cut.
//! Nothing about the machine is assumed: the sheet, the kerf, the leads, the
//! spacing between parts, the margin to the sheet's edge, the rotations each
//! part may take and every quantity are declared by the caller.
//!
//! # What a layout guarantees
//!
//! A thermal cut runs the beam center half a kerf plus an allowance (the
//! parts' boundary error and half the boundary tolerance) outside each part,
//! and its leads and pierce craters reach a bounded distance further into
//! the scrap (`lead_reach_mm`, from the declared leads). So everything cut
//! for a part lies within `kerf + allowance + lead reach` of it. A layout
//! keeps
//!
//! - any two parts `2 (kerf + allowance + lead reach) + part_spacing_mm`
//!   apart, so the kerfs, leads and pierce craters of two parts stay
//!   `part_spacing_mm` apart wherever the cut starts its loops (but see the
//!   center pierce under Limits);
//! - every part `kerf + allowance + lead reach + edge_margin_mm` (plus the
//!   sheet's boundary error) from the sheet's outline and holes.
//!
//! Both are proven on the finished layout by the independent check in
//! [`check`], which works on the exact lines and arcs and trusts nothing of
//! the planner. A layout that fails the check is refused, never returned.
//!
//! # Placement
//!
//! Bottom-left fill (Burke, Hellier, Kendall and Whitwell, "A new
//! bottom-left-fill heuristic algorithm for the two-dimensional irregular
//! packing problem", Operations Research 54(3), 2006): parts are placed one
//! at a time, largest area first, each at the lowest free position of the
//! leftmost column that has one. Columns are `resolution_mm` apart, as in
//! the paper; the lowest free position of a column is exact for the
//! planning outlines: the positions where the part would overlap a placed
//! part are a slice of their no-fit polygon (Bennell and Oliveira, "The
//! geometry of nesting problems: a tutorial", European Journal of
//! Operational Research 184(2), 2008), computed here as the union of
//! intervals of the convex pieces of both (the decomposition approach to the
//! no-fit polygon in the same tutorial, with trapezoids as the pieces).
//! Of a part's allowed rotations (and its mirror image, where allowed) the
//! one that ends furthest left is taken. A part falls into a hole of a
//! placed part, or of the sheet's free area, whenever that is the lowest
//! free position.
//!
//! An optional local search (`search_iterations`, `seed`) exchanges two
//! parts of the placement order and keeps the exchange when the nest gets
//! better, as the paper's hill climbing does. It is bounded by iterations,
//! not time, so the same request gives the same nest.
//!
//! # Limits
//!
//! This is a heuristic: it does not find the densest nest and makes no claim
//! about how close it comes. Columns are `resolution_mm` apart, and curved
//! planning outlines stand up to a quarter of that outside the required
//! clearance. Room for leads is reserved all round every part rather than
//! where the cut will put them. In a hole too small for its leads the cut
//! pierces at the hole's center, further from the loop than the reserved
//! reach: a part nested in such a hole keeps the part distance to the hole's
//! edge, but its spacing to that pierce is not part of the nest's proof (the
//! cut proves the pierce and lead clear of the part, or refuses). One sheet
//! is filled; parts that do not fit
//! are refused, or reported when `allow_unplaced` is set, never dropped.
//! Rotations are the declared angles only.
use crate::{
    planar::{Boundary, Component, ConstructionOptions, PlanarRegion},
    stock::CancelSignal,
    thermal::Lead,
    trace::{TracePath, TraceSegment},
};
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

pub mod check;
pub mod remnant;
pub use check::NestCheck;

type P2 = [f64; 2];

/// A part to place: one outline with holes.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NestPart {
    pub region: PlanarRegion,
    /// How many copies to place.
    pub quantity: u32,
    /// The rotations the part may take, degrees counter-clockwise.
    pub rotations_deg: Vec<f64>,
    /// Whether the part may also be placed mirrored (turned over).
    #[serde(default)]
    pub mirror: bool,
    /// Parts of a higher priority are placed before those of a lower one
    /// (so nearer the start corner); within a priority, largest first.
    #[serde(default)]
    pub priority: i32,
    /// A filler: placed after every other part, as many copies as fit up to
    /// `quantity`. Copies that do not fit are not a refusal; the result
    /// lists how many were placed.
    #[serde(default)]
    pub fill: bool,
    /// A kit: parts with the same kit name are placed one after another, so
    /// they lie together on the sheet, and all of them or none (a kit is
    /// never split between this sheet and what is left over). Kits are
    /// placed in the order of their priority (the highest of their parts'),
    /// then largest part first, like parts on their own.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub kit: Option<String>,
    /// Marks on the part (a part number, a bend line), in the part's own
    /// coordinates: placed with every copy and returned as the result's
    /// `marks`, the paths of a `ThermalMark`, which proves each inside its
    /// part at the mark's width.
    #[serde(default)]
    pub marks: Vec<TracePath>,
    /// Stretches of the part's outline that must not carry a lead (a seen
    /// or welded edge), in the part's own coordinates: placed with every
    /// copy and returned as the result's `no_lead`, the `keep_off` of the
    /// thermal cut's `lead_placement`.
    #[serde(default)]
    pub no_lead: Vec<TracePath>,
}

/// One part mark as placed with a copy of its part.
#[derive(Clone, Debug, Serialize)]
pub struct PlacedMark {
    /// The placed copy it belongs to: an index into `placements` (and the
    /// component of `region`).
    pub placement: usize,
    /// Which of the part's marks.
    pub mark: usize,
    pub path: TracePath,
}

/// A part's mark moved with a placement. Turning a part over mirrors its
/// marks with it, so an arc then runs the other way.
fn placed_mark(path: &TracePath, p: &Placement) -> TracePath {
    let turn = rotation(p.rotation_deg);
    let point = |v: P2| {
        let q = apply(p.mirrored, turn, p.translation_mm, [v[0], v[1], 0.]);
        [q[0], q[1]]
    };
    TracePath {
        start_mm: point(path.start_mm),
        segments: path
            .segments
            .iter()
            .map(|segment| match *segment {
                TraceSegment::Line { end_mm } => TraceSegment::Line {
                    end_mm: point(end_mm),
                },
                TraceSegment::Arc {
                    center_mm,
                    sweep_rad,
                } => TraceSegment::Arc {
                    center_mm: point(center_mm),
                    sweep_rad: if p.mirrored { -sweep_rad } else { sweep_rad },
                },
            })
            .collect(),
    }
}

fn yes() -> bool {
    true
}

/// The corner of the sheet's bounding box the nest fills from.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StartCorner {
    #[default]
    LowerLeft,
    LowerRight,
    UpperLeft,
    UpperRight,
}

/// The axis parts are stacked along from the start corner before the nest
/// advances along the other: `Y` fills a column of the sheet, then the next
/// column in x; `X` fills a row, then the next row in y.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FillAxis {
    X,
    #[default]
    Y,
}

/// The planner always fills from the lower left, up a column and then to
/// the next column. Another start corner or fill axis is the same fill in a
/// frame where the sheet is reflected and/or its axes are exchanged: a
/// sheet point p is planned at `L p`, with L one of the eight symmetries of
/// the square. Each orientation of a part is planned as L applied to that
/// orientation, so a part placed at t' in the frame lies at `L^-1 t'` on the
/// sheet with the same rotation and mirror flag. The independent check then
/// runs on the sheet itself and knows nothing of the frame.
#[derive(Clone, Copy)]
struct Frame {
    /// Rows of L.
    l: [P2; 2],
}
impl Frame {
    fn new(corner: StartCorner, axis: FillAxis) -> Self {
        let sx = match corner {
            StartCorner::LowerLeft | StartCorner::UpperLeft => 1.,
            StartCorner::LowerRight | StartCorner::UpperRight => -1.,
        };
        let sy = match corner {
            StartCorner::LowerLeft | StartCorner::LowerRight => 1.,
            StartCorner::UpperLeft | StartCorner::UpperRight => -1.,
        };
        Self {
            l: match axis {
                // Columns advance along the sheet's x, parts stack along y.
                FillAxis::Y => [[sx, 0.], [0., sy]],
                // Columns advance along the sheet's y, parts stack along x.
                FillAxis::X => [[0., sy], [sx, 0.]],
            },
        }
    }
    fn identity(&self) -> bool {
        self.l == [[1., 0.], [0., 1.]]
    }
    /// Whether L turns a loop round (a reflection).
    fn reversing(&self) -> bool {
        self.l[0][0] * self.l[1][1] - self.l[0][1] * self.l[1][0] < 0.
    }
    fn to(&self, p: P2) -> P2 {
        [
            self.l[0][0] * p[0] + self.l[0][1] * p[1],
            self.l[1][0] * p[0] + self.l[1][1] * p[1],
        ]
    }
    /// L is orthogonal: its inverse is its transpose.
    fn from(&self, p: P2) -> P2 {
        [
            self.l[0][0] * p[0] + self.l[1][0] * p[1],
            self.l[0][1] * p[0] + self.l[1][1] * p[1],
        ]
    }
    /// Loops in the frame, material still on their left.
    fn loops(&self, loops: Vec<Vec<P2>>) -> Vec<Vec<P2>> {
        if self.identity() {
            return loops;
        }
        loops
            .into_iter()
            .map(|l| {
                let mut points: Vec<P2> = l.into_iter().map(|p| self.to(p)).collect();
                if self.reversing() {
                    points.reverse();
                }
                points
            })
            .collect()
    }
}
fn boundary_tolerance() -> f64 {
    0.01
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NestRequest {
    /// The sheet: one outline, with holes where a remnant is already cut.
    pub sheet: PlanarRegion,
    pub parts: Vec<NestPart>,
    /// The kerf width: the beam tool's diameter in the thermal cut.
    pub kerf_mm: f64,
    /// The leads of the thermal cut that will cut the nest.
    pub lead_in: Lead,
    pub lead_out: Lead,
    /// As in the thermal cut. Default: the kerf width.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pierce_clearance_mm: Option<f64>,
    /// The thermal cut's corner loops: room is kept for them as for leads.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub corner_loops: Option<crate::thermal::CornerLoops>,
    /// Material left between what is cut for two parts.
    pub part_spacing_mm: f64,
    /// Material left between what is cut for a part and the sheet's edge.
    pub edge_margin_mm: f64,
    /// The column spacing of the placement.
    pub resolution_mm: f64,
    /// Whether a part may be placed in a hole of another part.
    #[serde(default = "yes")]
    pub part_in_part: bool,
    /// Return a partial nest listing the parts that did not fit, instead of
    /// refusing.
    #[serde(default)]
    pub allow_unplaced: bool,
    /// Exchanges of the placement order to try.
    #[serde(default)]
    pub search_iterations: u32,
    #[serde(default)]
    pub seed: u64,
    /// The corner the nest fills from (default: the lower left).
    #[serde(default)]
    pub start_corner: StartCorner,
    /// The axis parts are stacked along before the nest advances along the
    /// other (default: y, columns left to right from a left corner).
    #[serde(default)]
    pub fill_axis: FillAxis,
    /// As in the thermal cut that will cut the nest.
    #[serde(default = "boundary_tolerance")]
    pub boundary_tolerance_mm: f64,
    pub tolerance_mm: f64,
    /// Where room is kept for the cut's leads and pierce craters: all round
    /// every part (the default: the cut may start its loops anywhere), or
    /// only at the one place each outline will start (`lead_starts` of the
    /// result, which the cut must be given), so parts stand a kerf apart
    /// elsewhere.
    #[serde(default)]
    pub lead_room: LeadRoom,
    /// The thermal cut's `overcut_mm` (its lead-out starts that far past
    /// the loop's start, or before it when negative): part of the room kept
    /// with `lead_room` at the leads.
    #[serde(default)]
    pub overcut_mm: f64,
}

/// Where the nest keeps room for leads.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LeadRoom {
    #[default]
    AllRound,
    AtLeads,
}

/// Where one copy of a part lies: a point p of the part moves to
/// `R(rotation_deg) * M(p) + translation_mm`, M mirroring x when `mirrored`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Placement {
    /// Index into the request's parts.
    pub part: usize,
    /// Which copy of the part (0-based).
    pub copy: u32,
    pub rotation_deg: f64,
    pub mirrored: bool,
    pub translation_mm: [f64; 2],
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Unplaced {
    pub part: usize,
    pub quantity: u32,
}

#[derive(Clone, Debug, Serialize)]
pub struct NestResult {
    pub strategy: &'static str,
    /// One per placed copy, in the order of `region`'s components.
    pub placements: Vec<Placement>,
    /// Parts that did not fit (only with `allow_unplaced`).
    pub unplaced: Vec<Unplaced>,
    /// The placed parts: the `parts` of the thermal cut.
    pub region: PlanarRegion,
    pub sheet_area_mm2: f64,
    pub placed_area_mm2: f64,
    /// Placed part area over sheet area.
    pub utilisation: f64,
    /// The box around the placed parts `[min x, min y, max x, max y]`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub used_bounds_mm: Option<[f64; 4]>,
    /// How far leads and pierce craters may reach beyond a kerf's edge.
    pub lead_reach_mm: f64,
    /// The independent check of this layout.
    pub check: NestCheck,
    pub search_iterations: u32,
    pub search_improvements: u32,
    /// The copies in the order they were placed: (part, copy).
    pub order: Vec<(usize, u32)>,
    /// With `lead_room` at the leads: where each placed part's outline must
    /// start, a point on its drawn edge, in the order of `placements`. The
    /// `starts` of the thermal cut's `lead_placement`.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub lead_starts: Vec<P2>,
    /// The kits: their parts, whether they were placed, and the box round
    /// their copies.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub kits: Vec<KitReport>,
    /// Copies placed of each filler part: (part, copies).
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub filled: Vec<(usize, u32)>,
    /// The parts' marks, placed with every copy, in placement order.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub marks: Vec<PlacedMark>,
    /// The parts' edges kept free of leads, placed with every copy.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub no_lead: Vec<TracePath>,
    pub diagnostics: Vec<String>,
}

/// A kit as placed.
#[derive(Clone, Debug, Serialize)]
pub struct KitReport {
    pub name: String,
    /// Its parts (indices into the request's `parts`).
    pub parts: Vec<usize>,
    /// All of its copies are on the sheet (false: none is).
    pub placed: bool,
    /// The box round its placed copies `[min x, min y, max x, max y]`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bounds_mm: Option<[f64; 4]>,
}

/// The distances a layout must keep.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Clearances {
    lead_reach_mm: f64,
    part_distance_mm: f64,
    sheet_distance_mm: f64,
    /// With `lead_room` at the leads: the room kept at each outline's start.
    zones: Option<Zones>,
}

/// The room kept for one outline's leads: a rectangle on the drawn edge at
/// the loop's start, `half_width_mm` either way along the edge and
/// `height_mm` out from it, which holds the lead-in, the pierce crater and
/// the lead-out whichever way the loop is cut; and the distances it keeps.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Zones {
    pub half_width_mm: f64,
    pub height_mm: f64,
    /// From a zone to another part.
    pub part_distance_mm: f64,
    /// Between two zones.
    pub zone_distance_mm: f64,
    /// From a zone to the sheet's edge.
    pub sheet_distance_mm: f64,
}

/// Validates the request's numbers and derives the distances to keep.
fn clearances(request: &NestRequest) -> Result<Clearances, String> {
    let lead_ok = |l: &Lead| {
        l.length_mm.is_finite() && l.radius_mm.is_finite() && l.length_mm >= 0. && l.radius_mm >= 0.
    };
    for (which, lead) in [
        ("lead_in", &request.lead_in),
        ("lead_out", &request.lead_out),
    ] {
        if lead_ok(lead) {
            lead.validate(which).map_err(|e| format!("nest: {e}"))?;
        }
    }
    let within = |v: f64, max: f64| v.is_finite() && (0. ..=max).contains(&v);
    crate::heights::require(
        "nest",
        [
            (
                request.kerf_mm.is_finite() && request.kerf_mm > 0. && request.kerf_mm <= 1e3,
                format!("kerf_mm {} must be positive (the beam tool's kerf width)", request.kerf_mm),
            ),
            (
                lead_ok(&request.lead_in) && lead_ok(&request.lead_out),
                "lead lengths and radii must not be negative".into(),
            ),
            (
                request.pierce_clearance_mm.is_none_or(|c| within(c, 1e3)),
                "pierce_clearance_mm must not be negative".into(),
            ),
            (
                request.corner_loops.is_none_or(|c| c.validate().is_ok()),
                request
                    .corner_loops
                    .and_then(|c| c.validate().err())
                    .unwrap_or_default(),
            ),
            (
                within(request.part_spacing_mm, 1e4) && within(request.edge_margin_mm, 1e4),
                "part_spacing_mm and edge_margin_mm must not be negative".into(),
            ),
            (
                request.resolution_mm.is_finite() && (1e-3..=1e4).contains(&request.resolution_mm),
                format!("resolution_mm {} must be within 0.001..10000", request.resolution_mm),
            ),
            (
                (1e-7..=0.01).contains(&request.tolerance_mm)
                    && (0.0001..=0.1).contains(&request.boundary_tolerance_mm),
                "tolerance_mm must be within 1e-7..0.01 and boundary_tolerance_mm within 0.0001..0.1".into(),
            ),
            (!request.parts.is_empty(), "there are no parts to place".into()),
            (
                request.sheet.components.len() == 1,
                format!(
                    "the sheet must be one outline (with holes); it has {} components",
                    request.sheet.components.len()
                ),
            ),
        ],
    )?;
    for (i, part) in request.parts.iter().enumerate() {
        crate::heights::require(
            "nest",
            [
                (
                    part.region.components.len() == 1,
                    format!(
                        "part {i} must be one outline (with holes); it has {} components: declare each as its own part",
                        part.region.components.len()
                    ),
                ),
                (
                    part.quantity > 0,
                    format!("part {i} needs a quantity of at least 1"),
                ),
                (
                    !part.rotations_deg.is_empty()
                        && part
                            .rotations_deg
                            .iter()
                            .all(|a| a.is_finite() && a.abs() <= 360.),
                    format!(
                        "part {i} needs its allowed rotations_deg (each within -360..360; [0] keeps it as drawn)"
                    ),
                ),
            ],
        )?;
    }
    let kerf = request.kerf_mm;
    // The cut's offset beyond half a kerf (thermal.rs: the parts' boundary
    // error plus half the boundary tolerance).
    let error = request
        .parts
        .iter()
        .map(|p| p.region.boundary_error_mm)
        .fold(0., f64::max);
    let (lead_reach_mm, cut) = cut_reach(
        kerf,
        error,
        request.boundary_tolerance_mm,
        &request.lead_in,
        &request.lead_out,
        request.pierce_clearance_mm,
        request.corner_loops.as_ref(),
    );
    // A lead starts where its loop starts: the pierce point lies its arc
    // radius along the loop's tangent and radius + length along its normal
    // from there (thermal.rs; for the other shapes, no further from it than
    // the farthest point of the lead, `Lead::reach_mm`), and the crater
    // reaches the pierce clearance
    // beyond half a kerf round it. Leads shortened to fit a hole come
    // closer to the loop. The one exception is the cut's center pierce in a
    // hole too small for its leads: it stays inside that hole, where only a
    // part nested in the same hole could meet it, and the cut proves it
    // clear of every part or refuses.
    if request.lead_room == LeadRoom::AtLeads {
        if request.part_in_part {
            return Err("nest: lead_room at the leads is not planned with part_in_part (a hole's leads point at the part nested in it); set part_in_part to false, or keep the lead room all round".into());
        }
        if !(request.overcut_mm.is_finite() && request.overcut_mm.abs() <= 1e3) {
            return Err(format!(
                "nest: overcut_mm {} must be the thermal cut's overcut",
                request.overcut_mm
            ));
        }
        // All round a part: its kerf, the allowance and any corner loops.
        let none = Lead {
            length_mm: 0.,
            radius_mm: 0.,
            sweep_deg: None,
            angle_deg: None,
        };
        let (_, body) = cut_reach(
            kerf,
            error,
            request.boundary_tolerance_mm,
            &none,
            &none,
            Some(0.),
            request.corner_loops.as_ref(),
        );
        // The leads as the cut builds them (thermal.rs), from the loop's
        // start S on the kerf path, t along the cut and n into the scrap:
        // the lead-in's arc ends at S from a = S + r n - r t, its straight
        // part from the pierce a + L n; the lead-out leaves the loop the
        // overcut past S (before it when negative) and mirrors that. The
        // crater reaches the pierce clearance beyond half a kerf round the
        // pierce; a lead's kerf half a kerf beside it.
        let allowance = error + request.boundary_tolerance_mm / 2.;
        let offset = kerf / 2. + allowance;
        let crater = kerf / 2. + request.pierce_clearance_mm.unwrap_or(kerf);
        let (lin, lout) = (&request.lead_in, &request.lead_out);
        // Other shapes (thermal/leads.rs): the lead-in's points reach back
        // along the cut and out from it at most its extent, half a kerf
        // beyond for the kerf and the crater's reach round the pierce; the
        // lead-out likewise from the overcut. In-line leads at a corner
        // leave along the previous edge's extension, from the kerf path
        // round the drawn corner: their room is the lead's length and the
        // offset every way from the start.
        let (lead_in_width, lead_in_height) = if lin.plain() {
            (
                lin.radius_mm + crater,
                lin.radius_mm + lin.length_mm + crater,
            )
        } else {
            let ([t, n], p) = (lin.extent(), lin.pierce_tn());
            (
                (t + kerf / 2.).max(p[0].abs() + crater),
                (n + kerf / 2.).max(p[1] + crater),
            )
        };
        let (lead_out_width, lead_out_height) = if lout.plain() {
            (
                request.overcut_mm.abs() + lout.radius_mm + kerf / 2.,
                lout.radius_mm + lout.length_mm + kerf / 2.,
            )
        } else if lin.in_line() && lout.in_line() {
            (
                request.overcut_mm.abs() + offset + lout.length_mm + kerf / 2.,
                lout.length_mm + kerf / 2.,
            )
        } else {
            let [t, n] = lout.extent();
            (request.overcut_mm.abs() + t + kerf / 2., n + kerf / 2.)
        };
        return Ok(Clearances {
            lead_reach_mm,
            part_distance_mm: 2. * body + request.part_spacing_mm,
            sheet_distance_mm: body + request.edge_margin_mm + request.sheet.boundary_error_mm,
            zones: Some(Zones {
                half_width_mm: lead_in_width.max(lead_out_width),
                height_mm: offset + lead_in_height.max(lead_out_height),
                part_distance_mm: body + request.part_spacing_mm,
                zone_distance_mm: request.part_spacing_mm,
                sheet_distance_mm: request.edge_margin_mm + request.sheet.boundary_error_mm,
            }),
        });
    }
    Ok(Clearances {
        lead_reach_mm,
        zones: None,
        part_distance_mm: 2. * cut + request.part_spacing_mm,
        sheet_distance_mm: cut + request.edge_margin_mm + request.sheet.boundary_error_mm,
    })
}

/// How far leads, pierce craters and corner loops reach beyond a kerf's
/// edge (`lead_reach_mm`), and how far from a part its cut can remove
/// material: the kerf, the allowance (`error`: the boundaries' error; half
/// the cut's boundary tolerance) and that reach.
pub(crate) fn cut_reach(
    kerf: f64,
    error: f64,
    boundary_tolerance: f64,
    lead_in: &Lead,
    lead_out: &Lead,
    pierce_clearance: Option<f64>,
    corner_loops: Option<&crate::thermal::CornerLoops>,
) -> (f64, f64) {
    let allowance = error + boundary_tolerance / 2.;
    let reach = |l: &Lead| l.reach_mm();
    // A corner loop's kerf, from the edge of the kerf of the cut it leaves
    // (thermal/corners.rs), at the cut's own offset.
    let loops = corner_loops.map_or(0., |l| {
        crate::thermal::loop_reach_mm(l, kerf / 2. + allowance)
    });
    let lead_reach_mm = (reach(lead_in) + pierce_clearance.unwrap_or(kerf))
        .max(reach(lead_out))
        .max(loops);
    (lead_reach_mm, kerf + allowance + lead_reach_mm)
}

/// (cos, sin) of a rotation; exact for quarter turns.
pub(crate) fn rotation(deg: f64) -> (f64, f64) {
    let quarter = deg.rem_euclid(360.) / 90.;
    if quarter == 0. {
        (1., 0.)
    } else if quarter == 1. {
        (0., 1.)
    } else if quarter == 2. {
        (-1., 0.)
    } else if quarter == 3. {
        (0., -1.)
    } else {
        let a = deg.to_radians();
        (a.cos(), a.sin())
    }
}

/// A boundary vertex of a part, placed.
pub(crate) fn apply(mirror: bool, (c, s): (f64, f64), t: P2, v: [f64; 3]) -> [f64; 3] {
    let (x, bulge) = if mirror { (-v[0], -v[2]) } else { (v[0], v[2]) };
    [c * x - s * v[1] + t[0], s * x + c * v[1] + t[1], bulge]
}

fn sweep(bulge: f64) -> f64 {
    4. * bulge.atan()
}

/// The signed area of a line/arc loop (positive counter-clockwise).
fn loop_area(b: &Boundary) -> f64 {
    let v = &b.vertices;
    let mut area = 0.;
    for i in 0..v.len() {
        let (p, q) = (v[i], v[(i + 1) % v.len()]);
        area += (p[0] * q[1] - q[0] * p[1]) / 2.;
        if p[2] != 0. {
            // The circular segment between the chord and its arc.
            let angle = sweep(p[2]);
            let r = (q[0] - p[0]).hypot(q[1] - p[1]) / (2. * (angle / 2.).sin());
            area += r * r * (angle - angle.sin()) / 2.;
        }
    }
    area
}

fn component_area(c: &Component) -> f64 {
    loop_area(&c.outer).abs() - c.holes.iter().map(|h| loop_area(h).abs()).sum::<f64>()
}

/// A polygon round a loop for planning: with `enclose` it contains the
/// loop's material (arcs bulging out of the material get tangent polylines,
/// arcs bulging into it get chords), without it the material contains the
/// polygon. Either stands at most `deviation` off the arcs. The polygon runs
/// with the material on its left.
fn polygon(b: &Boundary, outer: bool, enclose: bool, deviation: f64) -> Vec<P2> {
    let n = b.vertices.len();
    // Material on the left: outlines counter-clockwise, holes clockwise.
    let forward = (loop_area(b) > 0.) == outer;
    let vertex = |k: usize| -> [f64; 3] {
        if forward {
            b.vertices[k]
        } else {
            // Reversed: edge k runs back along edge n - 2 - k.
            let p = b.vertices[n - 1 - k];
            [p[0], p[1], -b.vertices[(2 * n - 2 - k) % n][2]]
        }
    };
    let mut points = vec![];
    for k in 0..n {
        let (p, q) = (vertex(k), vertex((k + 1) % n));
        points.push([p[0], p[1]]);
        if p[2] == 0. {
            continue;
        }
        let angle = sweep(p[2]);
        let d = [q[0] - p[0], q[1] - p[1]];
        let h = (1. - p[2] * p[2]) / (4. * p[2]);
        let c = [(p[0] + q[0]) / 2. - d[1] * h, (p[1] + q[1]) / 2. + d[0] * h];
        let r = (p[0] - c[0]).hypot(p[1] - c[1]);
        // A tangent polyline over a step of this angle stands
        // r / cos(step / 2) - r outside the arc; chords stay closer.
        let step = (2. * (r / (r + deviation)).acos()).min(PI / 3.);
        let pieces = (angle.abs() / step).ceil().max(1.) as usize;
        let start = (p[1] - c[1]).atan2(p[0] - c[0]);
        let part = angle / pieces as f64;
        // A counter-clockwise arc has its center, so the material, on its
        // left: it bulges out of the material.
        if (angle > 0.) == enclose {
            let radius = r / (part / 2.).cos();
            for i in 0..pieces {
                let a = start + (i as f64 + 0.5) * part;
                points.push([c[0] + radius * a.cos(), c[1] + radius * a.sin()]);
            }
        } else {
            for i in 1..pieces {
                let a = start + i as f64 * part;
                points.push([c[0] + r * a.cos(), c[1] + r * a.sin()]);
            }
        }
    }
    points
}

fn polygons(region: &PlanarRegion, holes: bool, enclose: bool, deviation: f64) -> Vec<Vec<P2>> {
    let mut result = vec![];
    for c in &region.components {
        result.push(polygon(&c.outer, true, enclose, deviation));
        if holes {
            result.extend(
                c.holes
                    .iter()
                    .map(|h| polygon(h, false, enclose, deviation)),
            );
        }
    }
    result
}

fn bounds(loops: &[Vec<P2>]) -> [f64; 4] {
    let mut b = [
        f64::INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NEG_INFINITY,
    ];
    for p in loops.iter().flatten() {
        b = [
            b[0].min(p[0]),
            b[1].min(p[1]),
            b[2].max(p[0]),
            b[3].max(p[1]),
        ];
    }
    b
}

/// A convex piece: between x0 and x1, from a lower to an upper edge.
#[derive(Clone, Copy, Debug)]
struct Trapezoid {
    x0: f64,
    x1: f64,
    /// Lower and upper y at x0 and at x1.
    low: [f64; 2],
    high: [f64; 2],
}
impl Trapezoid {
    /// (lower, upper) y at x.
    fn at(&self, x: f64) -> (f64, f64) {
        if x <= self.x0 {
            (self.low[0], self.high[0])
        } else if x >= self.x1 {
            (self.low[1], self.high[1])
        } else {
            let t = (x - self.x0) / (self.x1 - self.x0);
            (
                self.low[0] + t * (self.low[1] - self.low[0]),
                self.high[0] + t * (self.high[1] - self.high[0]),
            )
        }
    }
    fn moved(&self, x: f64, y: f64) -> Self {
        Self {
            x0: self.x0 + x,
            x1: self.x1 + x,
            low: [self.low[0] + y, self.low[1] + y],
            high: [self.high[0] + y, self.high[1] + y],
        }
    }
}

/// The material of polygons (material on the left of each, counted by
/// winding so that overlapping polygons stay material) as trapezoids between
/// the vertical lines through every vertex; with `frame`, the rest of that
/// box instead.
fn trapezoids(loops: &[Vec<P2>], frame: Option<[f64; 4]>) -> Vec<Trapezoid> {
    let mut xs: Vec<f64> = loops.iter().flatten().map(|p| p[0]).collect();
    if let Some(f) = frame {
        xs.extend([f[0], f[2]]);
    }
    xs.sort_by(f64::total_cmp);
    xs.dedup();
    let edges: Vec<(P2, P2)> = loops
        .iter()
        .flat_map(|l| (0..l.len()).map(move |i| (l[i], l[(i + 1) % l.len()])))
        .filter(|(p, q)| p[0] != q[0])
        .collect();
    let mut result = vec![];
    let mut crossing: Vec<(f64, f64, i32)> = vec![];
    for w in xs.windows(2) {
        let (xa, xb) = (w[0], w[1]);
        crossing.clear();
        for (p, q) in &edges {
            if p[0].min(q[0]) <= xa && p[0].max(q[0]) >= xb {
                let y = |x: f64| {
                    if x == p[0] {
                        p[1]
                    } else if x == q[0] {
                        q[1]
                    } else {
                        p[1] + (x - p[0]) / (q[0] - p[0]) * (q[1] - p[1])
                    }
                };
                // An edge running in +x has the material above it.
                crossing.push((y(xa), y(xb), if q[0] > p[0] { 1 } else { -1 }));
            }
        }
        crossing.sort_by(|a, b| {
            (a.0 + a.1)
                .total_cmp(&(b.0 + b.1))
                .then(b.2.cmp(&a.2))
                .then(a.0.total_cmp(&b.0))
        });
        let mut emit = |low: (f64, f64), high: (f64, f64)| {
            let piece = Trapezoid {
                x0: xa,
                x1: xb,
                low: [low.0.min(high.0), low.1.min(high.1)],
                high: [low.0.max(high.0), low.1.max(high.1)],
            };
            if piece.high[0] > piece.low[0] || piece.high[1] > piece.low[1] {
                result.push(piece);
            }
        };
        let mut winding = 0;
        // The lower edge of the open piece: of material, or of the frame's
        // free space.
        let mut open = frame.map(|f| (f[1], f[1]));
        for &(ya, yb, direction) in &crossing {
            let before = winding > 0;
            winding += direction;
            if before == (winding > 0) {
                continue;
            }
            match (frame.is_some(), winding > 0) {
                (false, true) | (true, false) => open = Some((ya, yb)),
                (false, false) | (true, true) => {
                    if let Some(low) = open.take() {
                        emit(low, (ya, yb));
                    }
                }
            }
        }
        if let (Some(f), Some(low)) = (frame, open) {
            emit(low, (f[3], f[3]));
        }
    }
    result
}

/// Trapezoids by the x they cover, for overlap queries.
struct Index {
    x0: f64,
    width: f64,
    buckets: Vec<Vec<u32>>,
    pieces: Vec<Trapezoid>,
}
impl Index {
    fn new(x0: f64, x1: f64) -> Self {
        let count = 512;
        Self {
            x0,
            width: ((x1 - x0) / count as f64).max(f64::MIN_POSITIVE),
            buckets: vec![vec![]; count],
            pieces: vec![],
        }
    }
    fn bucket(&self, x: f64) -> usize {
        (((x - self.x0) / self.width).max(0.) as usize).min(self.buckets.len() - 1)
    }
    fn add(&mut self, piece: Trapezoid) {
        let id = self.pieces.len() as u32;
        self.pieces.push(piece);
        for b in self.bucket(piece.x0)..=self.bucket(piece.x1) {
            self.buckets[b].push(id);
        }
    }
    /// The y translations at which `moving` (moved by x) would overlap a
    /// piece here: one open interval per pair of convex pieces.
    fn forbidden(
        &self,
        moving: &[Trapezoid],
        x: f64,
        (y_low, y_high): (f64, f64),
        out: &mut Vec<(f64, f64)>,
    ) {
        if self.pieces.is_empty() {
            return;
        }
        for q in moving {
            let (q0, q1) = (q.x0 + x, q.x1 + x);
            let (first, last) = (self.bucket(q0), self.bucket(q1));
            for bucket in first..=last {
                for &id in &self.buckets[bucket] {
                    let p = &self.pieces[id as usize];
                    // Listed once: in the first bucket both cover.
                    if bucket != first.max(self.bucket(p.x0)) {
                        continue;
                    }
                    let (a, b) = (p.x0.max(q0), p.x1.min(q1));
                    if a >= b {
                        continue;
                    }
                    let (pa, pb) = (p.at(a), p.at(b));
                    let (qa, qb) = (q.at(a - x), q.at(b - x));
                    // Overlap at some x' needs p's bottom below q's top and
                    // q's bottom below p's top there; both are linear in x'.
                    let low = (pa.0 - qa.1).min(pb.0 - qb.1);
                    let high = (pa.1 - qa.0).max(pb.1 - qb.0);
                    if low < high && high > y_low && low < y_high {
                        out.push((low, high));
                    }
                }
            }
        }
    }
}

/// A part in one allowed orientation, as planning pieces round the origin.
struct Shape {
    rotation_deg: f64,
    mirrored: bool,
    /// The part with half the part distance round it (against other parts).
    apart: Vec<Trapezoid>,
    /// The part with the sheet distance round it (against the sheet).
    inset: Vec<Trapezoid>,
    inset_bounds: [f64; 4],
}

struct Planner<'a, C: CancelSignal> {
    shapes: Vec<Vec<Shape>>,
    areas: Vec<f64>,
    sheet: Index,
    sheet_bounds: [f64; 4],
    resolution: f64,
    cancel: &'a C,
}

struct Layout {
    /// Per entry of the order: (orientation, x, y) or None when it did not fit.
    placed: Vec<Option<(usize, f64, f64)>>,
    unplaced_area: f64,
    right: f64,
}
impl Layout {
    fn better_than(&self, other: &Layout) -> bool {
        self.unplaced_area
            .total_cmp(&other.unplaced_area)
            .then(self.right.total_cmp(&other.right))
            .is_lt()
    }
}

impl<C: CancelSignal> Planner<'_, C> {
    /// Bottom-left fill of the parts in this order.
    fn fill(&self, order: &[usize]) -> Result<Layout, String> {
        let sb = self.sheet_bounds;
        let mut placed_parts = Index::new(sb[0], sb[2]);
        // Placed parts only add obstacles: a column that had no free
        // position for a shape never gets one, so the next copy of the same
        // shape starts where the last one stopped.
        let mut next_column: Vec<Vec<usize>> =
            self.shapes.iter().map(|s| vec![0; s.len()]).collect();
        let mut intervals = vec![];
        let mut layout = Layout {
            placed: vec![],
            unplaced_area: 0.,
            right: sb[0],
        };
        for &part in order {
            let mut best: Option<(f64, f64, usize, f64, f64)> = None;
            for (o, shape) in self.shapes[part].iter().enumerate() {
                let b = shape.inset_bounds;
                let (x_low, x_high) = (sb[0] - b[0], sb[2] - b[2]);
                let ys = (sb[1] - b[1], sb[3] - b[3]);
                if x_high < x_low || ys.1 < ys.0 {
                    next_column[part][o] = usize::MAX;
                    continue;
                }
                let columns = ((x_high - x_low) / self.resolution).floor() as usize;
                // One more at the far end, where a part can still fit.
                let last = if x_low + columns as f64 * self.resolution < x_high {
                    columns + 1
                } else {
                    columns
                };
                let mut column = next_column[part][o];
                let mut found = None;
                while column <= last {
                    if self.cancel.is_cancelled() {
                        return Err("nesting cancelled".into());
                    }
                    let x = if column > columns {
                        x_high
                    } else {
                        x_low + column as f64 * self.resolution
                    };
                    intervals.clear();
                    self.sheet.forbidden(&shape.inset, x, ys, &mut intervals);
                    placed_parts.forbidden(&shape.apart, x, ys, &mut intervals);
                    intervals.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)));
                    // The lowest y in no open interval.
                    let mut y = ys.0;
                    for &(low, high) in &intervals {
                        if low >= y {
                            break;
                        }
                        y = y.max(high);
                    }
                    if y <= ys.1 {
                        found = Some((x, y));
                        break;
                    }
                    column += 1;
                }
                next_column[part][o] = if found.is_some() { column } else { usize::MAX };
                if let Some((x, y)) = found {
                    let key = (x + b[2], y + b[3]);
                    if best.is_none_or(|(right, top, ..)| {
                        key.0.total_cmp(&right).then(key.1.total_cmp(&top)).is_lt()
                    }) {
                        best = Some((key.0, key.1, o, x, y));
                    }
                }
            }
            match best {
                Some((right, _, o, x, y)) => {
                    for piece in &self.shapes[part][o].apart {
                        placed_parts.add(piece.moved(x, y));
                    }
                    layout.right = layout.right.max(right);
                    layout.placed.push(Some((o, x, y)));
                }
                None => {
                    layout.unplaced_area += self.areas[part];
                    layout.placed.push(None);
                }
            }
        }
        Ok(layout)
    }
}

/// splitmix64 (Steele, Lea and Flood, "Fast splittable pseudorandom number
/// generators", OOPSLA 2014): the same sequence on every platform.
fn next_random(state: &mut u64) -> u64 {
    *state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = *state;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

fn placed_component(source: &Component, p: &Placement) -> Component {
    let turn = rotation(p.rotation_deg);
    let boundary = |b: &Boundary| Boundary {
        vertices: b
            .vertices
            .iter()
            .map(|v| apply(p.mirrored, turn, p.translation_mm, *v))
            .collect(),
    };
    Component {
        outer: boundary(&source.outer),
        holes: source.holes.iter().map(boundary).collect(),
    }
}

/// Checks a layout against a request with the independent check alone: the
/// placements and the placed region may come from anywhere (an edited nest,
/// a manual layout).
pub fn verify(
    request: &NestRequest,
    placements: &[Placement],
    region: &PlanarRegion,
    lead_starts: &[P2],
) -> Result<NestCheck, String> {
    let clearances = clearances(request)?;
    let options = ConstructionOptions {
        tolerance_mm: request.tolerance_mm,
    };
    request.sheet.prepare(options)?;
    for part in &request.parts {
        part.region.prepare(options)?;
    }
    check::verify(request, &clearances, placements, region, lead_starts)
}

/// Where a part's outline starts when the lead room is kept at the leads:
/// the middle of its longest straight edge that holds the room's width, or,
/// with no such edge (a round part), its first vertex. In the part's own
/// coordinates, with the edge's direction there and the outward normal.
struct LeadStart {
    point: P2,
    along: P2,
    out: P2,
}
impl LeadStart {
    /// The room: `half_width` either way along the edge, `height` out.
    fn zone(&self, half_width: f64, height: f64) -> Boundary {
        let at = |a: f64, o: f64| {
            [
                self.point[0] + a * self.along[0] + o * self.out[0],
                self.point[1] + a * self.along[1] + o * self.out[1],
                0.,
            ]
        };
        // Counter-clockwise: `out` is to the right of `along` or to its
        // left, so order the corners by the sign of that turn.
        let left = self.along[0] * self.out[1] - self.along[1] * self.out[0] > 0.;
        let corners = [
            at(-half_width, 0.),
            at(half_width, 0.),
            at(half_width, height),
            at(-half_width, height),
        ];
        Boundary {
            vertices: if left {
                corners.to_vec()
            } else {
                corners.iter().rev().copied().collect()
            },
        }
    }
}
fn lead_start(component: &Component, half_width: f64) -> Result<LeadStart, String> {
    let edges = check::edges(&component.outer)?;
    // The outline counter-clockwise has the part on its left.
    let outward = if loop_area(&component.outer) > 0. {
        -1.
    } else {
        1.
    };
    let frame = |point: P2, along: P2| LeadStart {
        point,
        along,
        out: [-outward * along[1], outward * along[0]],
    };
    let mut best: Option<(f64, LeadStart)> = None;
    for e in &edges {
        if let check::Edge::Line(a, b) = *e {
            let length = (b[0] - a[0]).hypot(b[1] - a[1]);
            if length >= 2. * half_width && best.as_ref().is_none_or(|(l, _)| length > *l) {
                let along = [(b[0] - a[0]) / length, (b[1] - a[1]) / length];
                best = Some((
                    length,
                    frame([0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])], along),
                ));
            }
        }
    }
    if let Some((_, start)) = best {
        return Ok(start);
    }
    Ok(match edges[0] {
        check::Edge::Line(a, b) => {
            let length = (b[0] - a[0]).hypot(b[1] - a[1]);
            frame(a, [(b[0] - a[0]) / length, (b[1] - a[1]) / length])
        }
        check::Edge::Arc { a, c, r, ccw, .. } => {
            let radial = [(a[0] - c[0]) / r, (a[1] - c[1]) / r];
            let turn = if ccw { 1. } else { -1. };
            frame(a, [-turn * radial[1], turn * radial[0]])
        }
    })
}

/// Nests the request's parts on its sheet. See the module documentation.
pub fn nest(request: &NestRequest, cancel: &impl CancelSignal) -> Result<NestResult, String> {
    if cancel.is_cancelled() {
        return Err("nesting cancelled".into());
    }
    let clearances = clearances(request)?;
    let options = ConstructionOptions {
        tolerance_mm: request.tolerance_mm,
    };
    // Planning outlines stand this much beyond the required distances, so
    // the construction's own tolerance cannot bring a layout below them.
    let slack = 8. * request.tolerance_mm;
    let deviation = request.resolution_mm / 4.;
    let sheet = request
        .sheet
        .prepare(options)
        .map_err(|e| format!("nest sheet: {e}"))?;
    let frame = Frame::new(request.start_corner, request.fill_axis);
    let sheet_loops = frame.loops(polygons(sheet.definition(), true, false, deviation));
    let sheet_bounds = bounds(&sheet_loops);
    if (sheet_bounds[2] - sheet_bounds[0]) / request.resolution_mm > 1e6 {
        return Err(format!(
            "nest: resolution_mm {} gives more than a million columns across this sheet (along its fill's advance); use a coarser resolution",
            request.resolution_mm
        ));
    }
    let mut sheet_index = Index::new(sheet_bounds[0], sheet_bounds[2]);
    for piece in trapezoids(&sheet_loops, Some(sheet_bounds)) {
        sheet_index.add(piece);
    }
    let mut shapes = vec![];
    let mut areas = vec![];
    let mut starts: Vec<Option<P2>> = vec![];
    for (i, part) in request.parts.iter().enumerate() {
        let prepared = part
            .region
            .prepare(options)
            .map_err(|e| format!("nest part {i}: {e}"))?;
        areas.push(component_area(&part.region.components[0]));
        // Negative offsets grow the part (the kerf compensation's offset).
        let grown = |distance: f64, holes: bool| -> Result<Vec<Vec<P2>>, String> {
            let region = prepared
                .offset(-(distance + slack))
                .map_err(|e| format!("nest part {i}: {e}"))?
                .region;
            Ok(polygons(&region, holes, true, deviation))
        };
        // With the lead room at the leads: the part's start and its zone,
        // which is part of both shapes.
        let start = match &clearances.zones {
            Some(zones) => Some(lead_start(&part.region.components[0], zones.half_width_mm)?),
            None => None,
        };
        starts.push(start.as_ref().map(|s| s.point));
        let with_zone = |body: f64, zone: f64, holes: bool| -> Result<Vec<Vec<P2>>, String> {
            let (Some(start), Some(zones)) = (&start, &clearances.zones) else {
                return grown(body, holes);
            };
            let region = prepared
                .offset(-(body + slack))
                .map_err(|e| format!("nest part {i}: {e}"))?
                .region
                .prepare(options)
                .map_err(|e| format!("nest part {i}: {e}"))?;
            let room = PlanarRegion {
                version: 1,
                units: "mm".into(),
                name: "lead room".into(),
                boundary_error_mm: 0.,
                components: vec![Component {
                    outer: start.zone(
                        zones.half_width_mm + zone + slack,
                        zones.height_mm + zone + slack,
                    ),
                    holes: vec![],
                }],
                source: serde_json::Value::Null,
            }
            .prepare(options)
            .map_err(|e| format!("nest part {i}: lead room: {e}"))?;
            let both = region
                .boolean_prepared(&room, crate::planar::Boolean::Union)
                .map_err(|e| format!("nest part {i}: lead room: {e}"))?
                .into_parts()
                .1;
            Ok(polygons(both.definition(), false, true, deviation))
        };
        let (apart, inset) = match &clearances.zones {
            Some(zones) => (
                with_zone(
                    clearances.part_distance_mm / 2.,
                    zones.zone_distance_mm / 2.,
                    false,
                )?,
                with_zone(clearances.sheet_distance_mm, zones.sheet_distance_mm, true)?,
            ),
            None => (
                grown(clearances.part_distance_mm / 2., request.part_in_part)?,
                grown(clearances.sheet_distance_mm, true)?,
            ),
        };
        let mut orientations = vec![];
        for mirrored in [false, true] {
            if mirrored && !part.mirror {
                continue;
            }
            for &rotation_deg in &part.rotations_deg {
                let turn = rotation(rotation_deg);
                let turned = |loops: &[Vec<P2>]| -> Vec<Vec<P2>> {
                    let oriented = loops
                        .iter()
                        .map(|l| {
                            let mut points: Vec<P2> = l
                                .iter()
                                .map(|p| {
                                    let q = apply(mirrored, turn, [0.; 2], [p[0], p[1], 0.]);
                                    [q[0], q[1]]
                                })
                                .collect();
                            // Mirroring turns a loop round: keep the
                            // material on its left.
                            if mirrored {
                                points.reverse();
                            }
                            points
                        })
                        .collect();
                    frame.loops(oriented)
                };
                let inset = turned(&inset);
                orientations.push(Shape {
                    rotation_deg,
                    mirrored,
                    apart: trapezoids(&turned(&apart), None),
                    inset_bounds: bounds(&inset),
                    inset: trapezoids(&inset, None),
                });
            }
        }
        shapes.push(orientations);
    }
    let planner = Planner {
        shapes,
        areas,
        sheet: sheet_index,
        sheet_bounds,
        resolution: request.resolution_mm,
        cancel,
    };
    // Kits: parts placed one after another. A part with no kit is a block
    // of its own.
    let mut kit_names: Vec<&str> = vec![];
    let block: Vec<usize> = request
        .parts
        .iter()
        .enumerate()
        .map(|(i, p)| match p.kit.as_deref() {
            Some(name) => {
                let k = kit_names
                    .iter()
                    .position(|n| *n == name)
                    .unwrap_or_else(|| {
                        kit_names.push(name);
                        kit_names.len() - 1
                    });
                request.parts.len() + k
            }
            None => i,
        })
        .collect();
    for (i, p) in request.parts.iter().enumerate() {
        if let Some(name) = &p.kit
            && (name.is_empty() || name.len() > 200 || p.fill)
        {
            return Err(format!(
                "nest: part {i}: a kit needs a name of at most 200 characters, and a filler part belongs to none"
            ));
        }
    }
    // A block's place among the others: its highest priority, its largest
    // part, its first part.
    let block = &block;
    let members = |b: usize| (0..request.parts.len()).filter(move |&i| block[i] == b);
    let block_key = |b: usize| {
        (
            std::cmp::Reverse(members(b).map(|i| request.parts[i].priority).max().unwrap()),
            members(b).map(|i| planner.areas[i]).fold(0., f64::max),
            members(b).next().unwrap(),
        )
    };
    // Required parts before fillers, higher priority first, then largest
    // area first; equal ones in the declared order. A kit's parts stay
    // together, in that same order among themselves.
    let class = |i: usize| {
        (
            request.parts[i].fill,
            std::cmp::Reverse(request.parts[i].priority),
        )
    };
    let mut order: Vec<usize> = request
        .parts
        .iter()
        .enumerate()
        .flat_map(|(i, p)| std::iter::repeat_n(i, p.quantity as usize))
        .collect();
    order.sort_by(|&a, &b| {
        let (ka, kb) = (block_key(block[a]), block_key(block[b]));
        request.parts[a]
            .fill
            .cmp(&request.parts[b].fill)
            .then(ka.0.cmp(&kb.0))
            .then(kb.1.total_cmp(&ka.1))
            .then(ka.2.cmp(&kb.2))
            .then(class(a).cmp(&class(b)))
            .then(planner.areas[b].total_cmp(&planner.areas[a]))
            .then(a.cmp(&b))
    });
    let mut best = planner.fill(&order)?;
    // A kit is placed whole or not at all: one that does not fit is taken
    // out and the rest placed again.
    let mut dropped: Vec<usize> = vec![];
    loop {
        let incomplete: Vec<usize> = (0..kit_names.len())
            .map(|k| request.parts.len() + k)
            .filter(|b| !dropped.contains(b))
            .filter(|&b| {
                order
                    .iter()
                    .zip(&best.placed)
                    .any(|(&part, slot)| block[part] == b && slot.is_none())
            })
            .collect();
        if incomplete.is_empty() {
            break;
        }
        dropped.extend(&incomplete);
        order.retain(|&part| !incomplete.contains(&block[part]));
        best = planner.fill(&order)?;
    }
    let mut improvements = 0;
    let mut state = request.seed;
    if order.len() > 1 {
        for _ in 0..request.search_iterations {
            let i = (next_random(&mut state) % order.len() as u64) as usize;
            let j = (next_random(&mut state) % order.len() as u64) as usize;
            // An exchange never crosses a priority, puts a filler before a
            // required part or moves a part out of its kit.
            if order[i] == order[j]
                || class(order[i]) != class(order[j])
                || (block[order[i]] != block[order[j]]
                    && (block[order[i]] >= request.parts.len()
                        || block[order[j]] >= request.parts.len()))
            {
                continue;
            }
            order.swap(i, j);
            let candidate = planner.fill(&order)?;
            // An exchange inside a kit must still place all of it.
            let whole = order
                .iter()
                .zip(&candidate.placed)
                .all(|(&part, slot)| block[part] < request.parts.len() || slot.is_some());
            if whole && candidate.better_than(&best) {
                best = candidate;
                improvements += 1;
            } else {
                order.swap(i, j);
            }
        }
    }
    // Copies are numbered in placement order; the result lists parts in the
    // declared order.
    let mut copies = vec![0u32; request.parts.len()];
    let mut placements = vec![];
    let mut placed_order = vec![];
    for (&part, slot) in order.iter().zip(&best.placed) {
        if let Some((o, x, y)) = *slot {
            placed_order.push((part, copies[part]));
            let shape = &planner.shapes[part][o];
            placements.push(Placement {
                part,
                copy: copies[part],
                rotation_deg: shape.rotation_deg,
                mirrored: shape.mirrored,
                // Out of the planning frame (the rotation and the mirror
                // flag are the sheet's already: see `Frame`).
                translation_mm: if frame.identity() {
                    [x, y]
                } else {
                    frame.from([x, y])
                },
            });
            copies[part] += 1;
        }
    }
    placements.sort_by_key(|p| (p.part, p.copy));
    let unplaced: Vec<Unplaced> = request
        .parts
        .iter()
        .enumerate()
        .filter(|(i, p)| !p.fill && copies[*i] < p.quantity)
        .map(|(i, p)| Unplaced {
            part: i,
            quantity: p.quantity - copies[i],
        })
        .collect();
    if !unplaced.is_empty() && !request.allow_unplaced {
        let list: Vec<String> = unplaced
            .iter()
            .map(|u| {
                format!(
                    "part {}: {} of {}",
                    u.part, u.quantity, request.parts[u.part].quantity
                )
            })
            .collect();
        return Err(format!(
            "nest: these parts do not fit on the sheet with the declared spacing, margin and rotations ({}); use a larger sheet, fewer parts or more rotations, or set allow_unplaced to get the partial nest with the rest listed",
            list.join(", ")
        ));
    }
    let region = PlanarRegion {
        version: 1,
        units: "mm".into(),
        name: "nest".into(),
        boundary_error_mm: request
            .parts
            .iter()
            .map(|p| p.region.boundary_error_mm)
            .fold(0., f64::max),
        components: placements
            .iter()
            .map(|p| placed_component(&request.parts[p.part].region.components[0], p))
            .collect(),
        source: serde_json::Value::Null,
    };
    let lead_starts: Vec<P2> = placements
        .iter()
        .filter_map(|p| {
            starts[p.part].map(|s| {
                let q = apply(
                    p.mirrored,
                    rotation(p.rotation_deg),
                    p.translation_mm,
                    [s[0], s[1], 0.],
                );
                [q[0], q[1]]
            })
        })
        .collect();
    let check = check::verify(request, &clearances, &placements, &region, &lead_starts).map_err(|e| {
        format!(
            "nest: the planned layout failed its independent check and is not returned ({e}); this is a planner defect: report it, and try another resolution_mm meanwhile"
        )
    })?;
    let mut used: Option<[f64; 4]> = None;
    for c in &region.components {
        let b = check::Loop::new(&c.outer)?.bounds;
        used = Some(used.map_or(b, |u| {
            [
                u[0].min(b[0]),
                u[1].min(b[1]),
                u[2].max(b[2]),
                u[3].max(b[3]),
            ]
        }));
    }
    let sheet_area_mm2 = component_area(&request.sheet.components[0]);
    let placed_area_mm2: f64 = placements.iter().map(|p| planner.areas[p.part]).sum();
    let mut diagnostics = vec![];
    if !unplaced.is_empty() {
        diagnostics.push(format!(
            "{} parts did not fit and are listed as unplaced",
            unplaced.iter().map(|u| u.quantity).sum::<u32>()
        ));
    }
    if check.parts_in_holes > 0 {
        diagnostics.push(format!(
            "{} parts lie in holes of other parts",
            check.parts_in_holes
        ));
    }
    let marks = placements
        .iter()
        .enumerate()
        .flat_map(|(placement, p)| {
            request.parts[p.part]
                .marks
                .iter()
                .enumerate()
                .map(move |(mark, path)| PlacedMark {
                    placement,
                    mark,
                    path: placed_mark(path, p),
                })
        })
        .collect();
    let no_lead = placements
        .iter()
        .flat_map(|p| {
            request.parts[p.part]
                .no_lead
                .iter()
                .map(move |path| placed_mark(path, p))
        })
        .collect();
    let mut kits = vec![];
    for (k, name) in kit_names.iter().enumerate() {
        let b = request.parts.len() + k;
        let mut bounds: Option<[f64; 4]> = None;
        for (p, c) in placements.iter().zip(&region.components) {
            if block[p.part] == b {
                let own = check::Loop::new(&c.outer)?.bounds;
                bounds = Some(bounds.map_or(own, |u| {
                    [
                        u[0].min(own[0]),
                        u[1].min(own[1]),
                        u[2].max(own[2]),
                        u[3].max(own[3]),
                    ]
                }));
            }
        }
        kits.push(KitReport {
            name: name.to_string(),
            parts: members(b).collect(),
            placed: !dropped.contains(&b),
            bounds_mm: bounds,
        });
    }
    if !dropped.is_empty() {
        diagnostics.push(format!(
            "{} kit(s) did not fit whole and were left out",
            dropped.len()
        ));
    }
    Ok(NestResult {
        strategy: "bottom_left_fill",
        order: placed_order,
        lead_starts,
        kits,
        marks,
        no_lead,
        placements,
        unplaced,
        region,
        sheet_area_mm2,
        placed_area_mm2,
        utilisation: placed_area_mm2 / sheet_area_mm2,
        used_bounds_mm: used,
        lead_reach_mm: clearances.lead_reach_mm,
        check,
        search_iterations: request.search_iterations,
        search_improvements: improvements,
        filled: request
            .parts
            .iter()
            .enumerate()
            .filter(|(_, p)| p.fill)
            .map(|(i, _)| (i, copies[i]))
            .collect(),
        diagnostics,
    })
}

/// One sheet of a nest over several sheets.
#[derive(Clone, Debug, Serialize)]
pub struct SheetNest {
    /// Index into the sheets given.
    pub sheet: usize,
    /// The nest on that sheet; its placements and unplaced parts name the
    /// request's parts by their own indices.
    pub nest: NestResult,
}

#[derive(Clone, Debug, Serialize)]
pub struct SheetsResult {
    /// The sheets that took parts, in the order given.
    pub sheets: Vec<SheetNest>,
    /// Required parts left over after the last sheet (only with
    /// `allow_unplaced`).
    pub unplaced: Vec<Unplaced>,
}

/// Nests the request's parts over `sheets` in the order given (the
/// request's own sheet is not used): each sheet takes what fits of what is
/// left, the next takes the rest. A filler part fills every sheet that is
/// used. Required parts left after the last sheet are refused, naming them,
/// or listed with `allow_unplaced`; none is dropped. A sheet reached with
/// nothing required left is not used.
pub fn nest_sheets(
    request: &NestRequest,
    sheets: &[PlanarRegion],
    cancel: &impl CancelSignal,
) -> Result<SheetsResult, String> {
    if sheets.is_empty() {
        return Err("nest: give at least one sheet".into());
    }
    let mut left: Vec<u32> = request.parts.iter().map(|p| p.quantity).collect();
    let mut used = vec![];
    for (s, sheet) in sheets.iter().enumerate() {
        // The parts still wanted, with where each stands in the request.
        let wanted: Vec<usize> = (0..request.parts.len()).filter(|&i| left[i] > 0).collect();
        if !wanted.iter().any(|&i| !request.parts[i].fill) {
            break;
        }
        let mut one = request.clone();
        one.sheet = sheet.clone();
        one.allow_unplaced = true;
        one.parts = wanted
            .iter()
            .map(|&i| NestPart {
                quantity: left[i],
                ..request.parts[i].clone()
            })
            .collect();
        let mut result = nest(&one, cancel).map_err(|e| format!("sheet {s}: {e}"))?;
        // Back to the request's part indices.
        for p in &mut result.placements {
            p.part = wanted[p.part];
        }
        for u in &mut result.unplaced {
            u.part = wanted[u.part];
        }
        for f in &mut result.filled {
            f.0 = wanted[f.0];
        }
        for &i in &wanted {
            if request.parts[i].fill {
                continue;
            }
            let placed = result.placements.iter().filter(|p| p.part == i).count() as u32;
            left[i] -= placed;
        }
        if !result.placements.is_empty() {
            used.push(SheetNest {
                sheet: s,
                nest: result,
            });
        }
    }
    let unplaced: Vec<Unplaced> = (0..request.parts.len())
        .filter(|&i| !request.parts[i].fill && left[i] > 0)
        .map(|i| Unplaced {
            part: i,
            quantity: left[i],
        })
        .collect();
    if !unplaced.is_empty() && !request.allow_unplaced {
        let list: Vec<String> = unplaced
            .iter()
            .map(|u| {
                format!(
                    "part {}: {} of {}",
                    u.part, u.quantity, request.parts[u.part].quantity
                )
            })
            .collect();
        return Err(format!(
            "nest: these parts are left over after {} sheet(s) ({}); give more sheets or fewer parts, or set allow_unplaced to get the nests with the rest listed",
            sheets.len(),
            list.join(", ")
        ));
    }
    Ok(SheetsResult {
        sheets: used,
        unplaced,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn a_mark_on_a_part_turned_over_is_mirrored_with_it() {
        // A quarter arc from (2, 0) counter-clockwise about the origin, then
        // a line to (0, 5): mirrored in x, turned a quarter turn and moved.
        let path = TracePath {
            start_mm: [2., 0.],
            segments: vec![
                TraceSegment::Arc {
                    center_mm: [0., 0.],
                    sweep_rad: PI / 2.,
                },
                TraceSegment::Line { end_mm: [0., 5.] },
            ],
        };
        let placement = Placement {
            part: 0,
            copy: 0,
            rotation_deg: 90.,
            mirrored: true,
            translation_mm: [10., 20.],
        };
        let placed = placed_mark(&path, &placement);
        // (x, y) -> (-x, y) -> (-y, -x) + (10, 20).
        let close = |a: P2, b: P2| (a[0] - b[0]).hypot(a[1] - b[1]) < 1e-12;
        assert!(close(placed.start_mm, [10., 18.]));
        let TraceSegment::Arc {
            center_mm,
            sweep_rad,
        } = placed.segments[0]
        else {
            panic!()
        };
        assert!(close(center_mm, [10., 20.]) && sweep_rad == -PI / 2.);
        // The arc's end, walked from the placed start: the source's end
        // (0, 2) goes to (8, 20).
        let a = (placed.start_mm[1] - center_mm[1]).atan2(placed.start_mm[0] - center_mm[0])
            + sweep_rad;
        assert!(close(
            [center_mm[0] + 2. * a.cos(), center_mm[1] + 2. * a.sin()],
            [8., 20.]
        ));
        let TraceSegment::Line { end_mm } = placed.segments[1] else {
            panic!()
        };
        assert!(close(end_mm, [5., 20.]));
    }

    fn area(pieces: &[Trapezoid]) -> f64 {
        pieces
            .iter()
            .map(|t| (t.x1 - t.x0) * (t.high[0] - t.low[0] + t.high[1] - t.low[1]) / 2.)
            .sum()
    }

    #[test]
    fn trapezoids_cover_the_material_and_the_frame_covers_the_rest() {
        // An L shape with a square hole, counter-clockwise and clockwise.
        let outline = vec![
            [0., 0.],
            [10., 0.],
            [10., 4.],
            [4., 4.],
            [4., 10.],
            [0., 10.],
        ];
        let hole = vec![[1., 1.], [1., 3.], [3., 3.], [3., 1.]];
        let loops = [outline, hole];
        assert!((area(&trapezoids(&loops, None)) - 60.).abs() < 1e-12);
        let frame = [0., 0., 10., 10.];
        assert!((area(&trapezoids(&loops, Some(frame))) - 40.).abs() < 1e-12);
    }

    #[test]
    fn planning_polygons_enclose_or_stay_inside_arcs() {
        let disc = Boundary {
            vertices: vec![[10., 0., 1.], [-10., 0., 1.]],
        };
        let radii = |enclose: bool, outer: bool| -> (f64, f64) {
            let p = polygon(&disc, outer, enclose, 0.1);
            // The nearest point of each edge and the farthest vertex.
            let near = (0..p.len())
                .map(|i| {
                    let (a, b) = (p[i], p[(i + 1) % p.len()]);
                    let d = [b[0] - a[0], b[1] - a[1]];
                    (a[0] * d[1] - a[1] * d[0]).abs() / d[0].hypot(d[1])
                })
                .fold(f64::INFINITY, f64::min);
            let far = p.iter().map(|v| v[0].hypot(v[1])).fold(0., f64::max);
            (near, far)
        };
        // A disc of material: enclosed by a polygon outside it.
        let (near, far) = radii(true, true);
        assert!(near >= 10. - 1e-12 && far <= 10.1 + 1e-12, "{near} {far}");
        // As the material's subset, and as a hole to enclose material round
        // it: inside the circle.
        for (enclose, outer) in [(false, true), (true, false)] {
            let (near, far) = radii(enclose, outer);
            assert!(near >= 9.9 - 1e-12 && far <= 10. + 1e-12, "{near} {far}");
        }
        // A hole as a subset of the material round it: outside the circle.
        let (near, far) = radii(false, false);
        assert!(near >= 10. - 1e-12 && far <= 10.1 + 1e-12, "{near} {far}");
    }

    #[test]
    fn quarter_turns_are_exact() {
        assert_eq!(rotation(90.), (0., 1.));
        assert_eq!(rotation(-90.), (0., -1.));
        assert_eq!(rotation(180.), (-1., 0.));
        assert_eq!(rotation(360.), (1., 0.));
        assert_eq!(
            apply(true, rotation(90.), [1., 2.], [3., 4., 0.5]),
            [-3., -1., -0.5]
        );
    }
}
