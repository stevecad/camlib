//! Flat-area finishing (the usual Flat/Horizontal strategy): finds the
//! horizontal faces of a part and finishes each at its exact height with a
//! flat end mill, in contour-parallel passes that offset each flat's tool
//! centre region inward, outside in, linked at depth where the link is
//! proven, and entered from the side in air where a pass runs along an open
//! edge — the plunge always coming down beside the level's face, never onto
//! it.
//!
//! Proof: every cut is a horizontal line at the cut height. It is kept only
//! where the highest point of the part (and the machining base) under the
//! tool's swept disk (a capsule in XY) is not above the cut height plus the
//! tolerance. That maximum is computed exactly per triangle (a linear
//! function over the convex triangle-capsule intersection peaks at one of its
//! extreme points); chords that fail are halved until they pass or vanish.
use crate::{
    facing::TraverseAxis,
    motion::{Coolant, Curve, Event, Motion, MoveKind, SpindleDirection, Vec3},
    pocket::ResidualStatus,
    stock::{CancelSignal, Stock, StockSpec},
    surface::SurfaceMesh,
    tool::{CutterShape, ToolAssembly, valid_id},
};
use serde::{Deserialize, Serialize};

/// How far behind the pass start (in cutter radii) an arc ramp may begin
/// (`FlatPlan::arc_ramp`), as far as a waterline side entry is searched.
const ARC_RAMP_REACH_RADII: f64 = 4.;

fn x_axis() -> TraverseAxis {
    TraverseAxis::X
}
fn level_tolerance() -> f64 {
    0.001
}

/// Emitted arc moves are split so each move's chord stays within this of the
/// arc: the arc itself is exact, but moves are checked (and posted programs
/// read) through their endpoints, so a move must not sweep so far that its
/// chord cuts the corner its endpoints describe.
const ARC_MOVE_SAGITTA: f64 = 0.005;

/// The arc stroke split into equal sub-arcs whose chords stay within
/// `sagitta` of the arc (a straight stroke passes through unchanged).
fn split_arc(seg: &crate::pocket::Stroke, sagitta: f64) -> Vec<crate::pocket::Stroke> {
    let Some((c, sweep)) = seg.arc else {
        return vec![*seg];
    };
    let rho = (seg.a[0] - c[0]).hypot(seg.a[1] - c[1]);
    let half = (1. - sagitta / rho).clamp(-1., 1.).acos().max(1e-9);
    let n = ((sweep.abs() / (2. * half)).ceil().max(1.) as usize).max(1);
    let start = (seg.a[1] - c[1]).atan2(seg.a[0] - c[0]);
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let t0 = i as f64 / n as f64;
        let t1 = (i + 1) as f64 / n as f64;
        let p = if i == 0 {
            seg.a
        } else {
            let angle = start + sweep * t0;
            [c[0] + rho * angle.cos(), c[1] + rho * angle.sin()]
        };
        let b = if i + 1 == n {
            seg.b
        } else {
            let angle = start + sweep * t1;
            [c[0] + rho * angle.cos(), c[1] + rho * angle.sin()]
        };
        out.push(crate::pocket::Stroke {
            a: p,
            b,
            arc: Some((c, sweep * (t1 - t0))),
        });
    }
    out
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlatFloor {
    pub z_mm: f64,
    pub region: crate::planar::PlanarRegion,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlatOperation<G = SurfaceMesh> {
    pub id: String,
    pub tool: String,
    pub surface: G,
    /// Optional CAD trims of the selected horizontal faces in work coordinates.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub floors: Vec<FlatFloor>,
    /// Cut the outermost wall-following contours after the floor, keeping the radial allowance.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub finish_walls: bool,
    /// Machining base: flats at or below it are not machined.
    pub floor_z_mm: f64,
    pub stepover_mm: f64,
    pub clearance_z_mm: f64,
    #[serde(default = "x_axis")]
    pub axis: TraverseAxis,
    /// Axial stock left on the flats.
    #[serde(default)]
    pub stock_to_leave_mm: f64,
    /// Radial stock left on the walls: the passes keep this much farther
    /// from every surface standing above the flat than the cutter's radius,
    /// so they cut only the horizontal faces.
    #[serde(default)]
    pub radial_stock_to_leave_mm: f64,
    /// Faces within this height of each other form one flat level.
    #[serde(default = "level_tolerance")]
    pub level_tolerance_mm: f64,
    pub feed_mm_min: f64,
    pub plunge_mm_min: f64,
    pub tolerance_mm: f64,
    pub spindle_rpm: f64,
    pub work_offset: u8,
    pub tool_number: u32,
    pub length_register: u32,
    pub coolant: Coolant,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub retract_z_mm: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub feed_z_mm: Option<f64>,
    /// Containment and avoid areas (clamps, fixtures, features to leave alone).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub limits: Option<crate::limits::Limits>,
    /// How a run is entered through stock left above it: a ramp along the
    /// run (`RampEntry`: declared angle, feed and shortest reversal). None
    /// declares that no stock stands above the runs; a run that has stock
    /// above its start is then refused (see `FlatPlan::plunge_or_ramp`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub entry: Option<crate::entry::PocketEntry>,
}
impl<G> FlatOperation<G> {
    pub(crate) fn map_geometry<H, E>(
        self,
        f: &mut impl FnMut(G) -> Result<H, E>,
    ) -> Result<FlatOperation<H>, E> {
        Ok(FlatOperation {
            limits: self.limits,
            entry: self.entry,
            id: self.id,
            tool: self.tool,
            surface: f(self.surface)?,
            floors: self.floors,
            finish_walls: self.finish_walls,
            floor_z_mm: self.floor_z_mm,
            stepover_mm: self.stepover_mm,
            clearance_z_mm: self.clearance_z_mm,
            axis: self.axis,
            stock_to_leave_mm: self.stock_to_leave_mm,
            radial_stock_to_leave_mm: self.radial_stock_to_leave_mm,
            level_tolerance_mm: self.level_tolerance_mm,
            feed_mm_min: self.feed_mm_min,
            plunge_mm_min: self.plunge_mm_min,
            tolerance_mm: self.tolerance_mm,
            spindle_rpm: self.spindle_rpm,
            work_offset: self.work_offset,
            tool_number: self.tool_number,
            length_register: self.length_register,
            coolant: self.coolant,
            retract_z_mm: self.retract_z_mm,
            feed_z_mm: self.feed_z_mm,
        })
    }
}

#[derive(Debug, Serialize)]
pub struct FlatReport {
    pub strategy: &'static str,
    pub geometric_residual_status: ResidualStatus,
    pub levels_z_mm: Vec<f64>,
    /// Loops cut (closed offset loops and the open stretches a loop was split
    /// into by dropped chords).
    pub rows: usize,
    pub runs: usize,
    /// Closed offset loops planned, before any chord was dropped.
    pub loops: usize,
    #[serde(skip_serializing_if = "is_zero")]
    pub wall_passes: usize,
    /// Links cut at depth between the passes of one chain.
    #[serde(skip_serializing_if = "is_zero")]
    pub links_at_depth: usize,
    pub cut_length_mm: f64,
    /// Row length near flats that could not be proven clear and was dropped.
    pub dropped_length_mm: f64,
    pub runs_completed: usize,
    /// Runs entered by a shallow zigzag ramp along the run because the
    /// live stock stood above the run's height at its start.
    #[serde(skip_serializing_if = "is_zero")]
    pub ramped_entries: usize,
    /// Runs entered by a vertical plunge in air and a tangent arc lead-in
    /// whose radius is the tool's radius (see `FlatPlan::arc_lead_in`).
    #[serde(skip_serializing_if = "is_zero")]
    pub arc_lead_ins: usize,
    /// Runs left by a tangent arc lead-out of the tool's radius before the
    /// retract (see `FlatPlan::arc_lead_out`).
    #[serde(skip_serializing_if = "is_zero")]
    pub arc_lead_outs: usize,
    /// Vertical plunges that turn to horizontal through a quarter circle of
    /// the tool's radius in the vertical plane (see `FlatPlan::plunge_turn`).
    #[serde(skip_serializing_if = "is_zero")]
    pub rounded_plunges: usize,
    /// Entries through stock on one smooth descent along the pass's line
    /// (see `FlatPlan::arc_ramp`): counted in `ramped_entries` too.
    #[serde(skip_serializing_if = "is_zero")]
    pub arc_ramps: usize,
    pub clearance_proof: &'static str,
}

fn is_zero(n: &usize) -> bool {
    *n == 0
}

#[derive(Clone, Copy)]
struct Tri {
    p: [Vec3; 3],
    min: [f64; 2],
    max: [f64; 3],
}

/// Triangles binned on an XY grid for capsule queries.
pub(crate) struct Index {
    tris: Vec<Tri>,
    origin: [f64; 2],
    cell: f64,
    dims: [usize; 2],
    bins: Vec<Vec<u32>>,
    entries: u64,
    floor: f64,
    eps: f64,
}

fn dist2_to_segment(x: [f64; 2], p: [f64; 2], q: [f64; 2]) -> f64 {
    let d = [q[0] - p[0], q[1] - p[1]];
    let l2 = d[0] * d[0] + d[1] * d[1];
    let t = if l2 > 0. {
        (((x[0] - p[0]) * d[0] + (x[1] - p[1]) * d[1]) / l2).clamp(0., 1.)
    } else {
        0.
    };
    let c = [p[0] + t * d[0] - x[0], p[1] + t * d[1] - x[1]];
    c[0] * c[0] + c[1] * c[1]
}

/// Maximum of a unimodal (concave) function on [lo, hi]: golden-section
/// search reusing one interior point per step; returns the best of the
/// final point and both ends.
fn golden_max(mut f: impl FnMut(f64) -> f64, lo: f64, hi: f64, steps: usize) -> f64 {
    let phi = (5f64.sqrt() - 1.) / 2.;
    let (fa, fb) = (f(lo), f(hi));
    let (mut a, mut b) = (lo, hi);
    let mut x1 = b - phi * (b - a);
    let mut x2 = a + phi * (b - a);
    let (mut f1, mut f2) = (f(x1), f(x2));
    for _ in 0..steps {
        if f1 < f2 {
            a = x1;
            x1 = x2;
            f1 = f2;
            x2 = a + phi * (b - a);
            f2 = f(x2);
        } else {
            b = x2;
            x2 = x1;
            f2 = f1;
            x1 = b - phi * (b - a);
            f1 = f(x1);
        }
    }
    fa.max(fb).max(f1).max(f2)
}
/// Where a unimodal (convex) function is least on [lo, hi].
fn golden_argmin(mut f: impl FnMut(f64) -> f64, lo: f64, hi: f64, steps: usize) -> f64 {
    let phi = (5f64.sqrt() - 1.) / 2.;
    let (mut a, mut b) = (lo, hi);
    let mut x1 = b - phi * (b - a);
    let mut x2 = a + phi * (b - a);
    let (mut f1, mut f2) = (f(x1), f(x2));
    for _ in 0..steps {
        if f1 > f2 {
            a = x1;
            x1 = x2;
            f1 = f2;
            x2 = a + phi * (b - a);
            f2 = f(x2);
        } else {
            b = x2;
            x2 = x1;
            f2 = f1;
            x1 = b - phi * (b - a);
            f1 = f(x1);
        }
    }
    (a + b) / 2.
}
/// Height of a bull-nose (corner radius rc, radius r) bottom above its tip at
/// distance d from the axis: flat to r - rc, then the corner arc.
fn bull_profile(d: f64, r: f64, rc: f64) -> f64 {
    let s = d - (r - rc);
    if s <= 0. {
        0.
    } else {
        rc - (rc * rc - s * s).max(0.).sqrt()
    }
}
/// One rotationally symmetric part of a tool assembly, seen from below: its
/// lower surface is flat out to `r0` at `z0` above the tool tip, then rises
/// out to `r1`, either linearly (`rise` mm of height per mm of radius, with
/// `arc` 0) or along a circular arc of radius `arc` centred at radius `r0`,
/// height `z0 + arc` (with `rise` 0 and `r1 - r0` at most `arc`). A chamfer
/// mill's cone (tip radius r0, rise = 1 / tan(half angle)), a flat end
/// (rise 0, r0 = r1), a ball end (r0 0, arc = r1 = the radius), a bull-nose
/// end (r0 = radius - corner radius, arc = the corner radius) and every
/// cylinder or frustum of a shank, neck or holder (at its lower end) are
/// sections. The profile is convex and non-decreasing in the radius, so
/// contact on a triangle is a concave maximisation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Section {
    pub z0: f64,
    pub r0: f64,
    pub r1: f64,
    pub rise: f64,
    pub arc: f64,
}
impl Section {
    /// Height of the lower surface above the tip at radius `d` (None
    /// beyond `r1`).
    pub(crate) fn profile(&self, d: f64) -> Option<f64> {
        if d <= self.r0 {
            Some(self.z0)
        } else if d <= self.r1 {
            Some(if self.arc > 0. {
                let s = d - self.r0;
                self.z0 + self.arc - (self.arc * self.arc - s * s).max(0.).sqrt()
            } else {
                self.z0 + (d - self.r0) * self.rise
            })
        } else {
            None
        }
    }
    /// Where along the direction of steepest ascent (slope `g`) of a plane
    /// the tip rests highest: the radius at which the profile's slope first
    /// reaches `g` (r0 for a linear rise at least `g`, r1 when the profile
    /// never gets that steep).
    fn reach_for_slope(&self, g: f64) -> f64 {
        if self.arc > 0. {
            // h'(r0 + s) = s / sqrt(arc^2 - s^2) = g.
            (self.r0 + self.arc * g / (1. + g * g).sqrt()).min(self.r1)
        } else if g <= self.rise {
            self.r0
        } else {
            self.r1
        }
    }
    pub(crate) fn valid(&self) -> bool {
        [self.z0, self.r0, self.r1, self.rise, self.arc]
            .iter()
            .all(|v| v.is_finite())
            && self.z0 >= 0.
            && 0. <= self.r0
            && self.r0 <= self.r1
            && self.r1 > 0.
            && self.rise >= 0.
            && self.arc >= 0.
            && (self.arc == 0. || (self.rise == 0. && self.r1 - self.r0 <= self.arc * (1. + 1e-12)))
    }
}

/// The sections of a tool assembly, tip upward: the cutter's cutting end
/// (a chamfer mill's or drill's cone, or a flat end), then the neck and
/// shank cylinders and every holder frustum at their lower ends. Each
/// section bounds the tool from below wherever it is the lowest part at a
/// radius; higher parts at the same radius only add dominated candidates.
/// Ball and bull-nose ends are not sections (curved): refused here.
pub(crate) fn assembly_sections(
    tool: &crate::tool::PreparedAssembly,
) -> Result<Vec<Section>, String> {
    sections_of(tool, false)
}
/// As `assembly_sections`, with a ball or bull-nose end replaced by the flat
/// end of the same radius. That cylinder contains the curved end (whose
/// lower surface is never below the tip), so a proof that the enclosing
/// sections stay clear holds for the real tool; it is only conservative.
pub(crate) fn enclosing_sections(
    tool: &crate::tool::PreparedAssembly,
) -> Result<Vec<Section>, String> {
    sections_of(tool, true)
}

/// Outer staircase of the real curved lower profile. Each cylinder uses the
/// radius at its band's upper edge and height at its lower edge, so their union
/// contains the physical cutter. Never a sampled clearance certificate.
pub(crate) fn bounded_sections(
    tool: &crate::tool::PreparedAssembly,
    height_step: f64,
) -> Result<Vec<Section>, String> {
    let mut sections = enclosing_sections(tool)?;
    let r = tool.cutter().definition().diameter_mm / 2.;
    let rc = match tool.cutter().definition().shape {
        CutterShape::Ball => r,
        CutterShape::Bull { corner_radius_mm } => corner_radius_mm,
        _ => return Ok(sections),
    };
    if !height_step.is_finite() || height_step <= 0. || (rc / height_step).ceil() > 4096. {
        return Err("curved cutter enclosure exceeds section budget".into());
    }
    let count = (rc / height_step).ceil().max(1.) as usize;
    sections.remove(0);
    for i in 0..count {
        let z0 = rc * i as f64 / count as f64;
        let z1 = rc * (i + 1) as f64 / count as f64;
        // Round outward in radius; arithmetic margin is also charged by callers.
        let radius =
            (r - rc + (z1 * (2. * rc - z1)).max(0.).sqrt()) + 64. * f64::EPSILON * (1. + r);
        sections.push(Section {
            z0,
            r0: radius,
            r1: radius,
            rise: 0.,
            arc: 0.,
        });
    }
    Ok(sections)
}
fn sections_of(
    tool: &crate::tool::PreparedAssembly,
    enclose_curved: bool,
) -> Result<Vec<Section>, String> {
    if let Some(g) = tool.cutter().indexable() {
        // Enclosing cylinders are sufficient for these separation proofs,
        // never for stock removal. Keep each region's own bottom height.
        return Ok(g
            .cutting()
            .definition()
            .regions
            .iter()
            .chain(&tool.indexable_noncutting().unwrap().definition().regions)
            .map(|region| {
                let [r, z] = region.bounds();
                Section {
                    z0: z[0],
                    r0: r[1],
                    r1: r[1],
                    rise: 0.,
                    arc: 0.,
                }
            })
            .collect());
    }
    let def = tool.definition();
    let cutter = tool.cutter();
    let c = cutter.definition();
    let radius = c.diameter_mm / 2.;
    let mut out = vec![match c.shape {
        CutterShape::Flat => Section {
            z0: 0.,
            r0: radius,
            r1: radius,
            rise: 0.,
            arc: 0.,
        },
        CutterShape::Chamfer {
            tip_diameter_mm, ..
        } => {
            let r0 = tip_diameter_mm / 2.;
            let top = cutter
                .tip_height_at_radius(radius)
                .ok_or("chamfer cutter profile")?;
            Section {
                z0: 0.,
                r0,
                r1: radius,
                rise: top / (radius - r0),
                arc: 0.,
            }
        }
        CutterShape::Drill { .. } => Section {
            z0: 0.,
            r0: 0.,
            r1: radius,
            rise: cutter.tip_height_at_radius(radius).ok_or("drill profile")? / radius,
            arc: 0.,
        },
        // A lollipop or dovetail lies inside the cylinder of its widest
        // radius from the tip up to its cutting length.
        CutterShape::Ball
        | CutterShape::Bull { .. }
        | CutterShape::Lollipop
        | CutterShape::Dovetail { .. }
            if enclose_curved =>
        {
            Section {
                z0: 0.,
                r0: radius,
                r1: radius,
                rise: 0.,
                arc: 0.,
            }
        }
        // The corner arc: a quarter circle from the flat (none for a ball)
        // out to the cutter radius.
        CutterShape::Ball => Section {
            z0: 0.,
            r0: 0.,
            r1: radius,
            rise: 0.,
            arc: radius,
        },
        CutterShape::Bull { corner_radius_mm } => Section {
            z0: 0.,
            r0: radius - corner_radius_mm,
            r1: radius,
            rise: 0.,
            arc: corner_radius_mm,
        },
        _ => {
            return Err("tool sections cover flat, chamfer, drill, ball and bull-nose ends".into());
        }
    }];
    // Neck and shank below the holder: cylinders at their lower ends.
    let mut breaks = vec![c.cutting_length_mm];
    if let Some(neck) = &c.neck {
        breaks.push(c.cutting_length_mm + neck.length_mm);
    }
    for z0 in breaks.into_iter().filter(|z| *z < def.stickout_mm) {
        let r = tool
            .radius_at((z0 + 1e-9).min(def.stickout_mm))
            .ok_or("tool body profile")?
            .radius_mm;
        out.push(Section {
            z0,
            r0: r,
            r1: r,
            rise: 0.,
            arc: 0.,
        });
    }
    for h in &def.holders {
        out.push(if h.radius_max_mm > h.radius_min_mm {
            Section {
                z0: h.z_min_mm,
                r0: h.radius_min_mm,
                r1: h.radius_max_mm,
                rise: (h.z_max_mm - h.z_min_mm) / (h.radius_max_mm - h.radius_min_mm),
                arc: 0.,
            }
        } else {
            Section {
                z0: h.z_min_mm,
                r0: h.radius_min_mm,
                r1: h.radius_min_mm,
                rise: 0.,
                arc: 0.,
            }
        });
    }
    if out.iter().any(|s| !s.valid()) {
        return Err("tool sections must be finite with nonnegative heights and radii".into());
    }
    Ok(out)
}

/// Upper bound of the maximum of a concave function on [lo, hi]: the best
/// golden-section value plus `lipschitz` times the final bracket (the
/// maximum lies in it, and the function changes by at most that there).
fn golden_max_bound(f: impl FnMut(f64) -> f64, lo: f64, hi: f64, lipschitz: f64) -> f64 {
    const STEPS: usize = 60;
    let bracket = (hi - lo) * ((5f64.sqrt() - 1.) / 2.).powi(STEPS as i32);
    golden_max(f, lo, hi, STEPS) + lipschitz * bracket
}

/// Upper bound of the maximum of a concave function on [lo, hi] (values may
/// be -infinity outside its domain only at the ends). Golden-section search
/// keeps a bracket [a, b] with interior points x1 < x2 holding the maximum;
/// concavity puts the function below every secant's extension, which bounds
/// it on each of [a, x1], [x1, x2] and [x2, b].
fn concave_max_upper(mut f: impl FnMut(f64) -> f64, lo: f64, hi: f64) -> f64 {
    // Not an interval (or NaN): just the ends.
    if hi.partial_cmp(&lo) != Some(std::cmp::Ordering::Greater) {
        return f(lo).max(f(hi));
    }
    let phi = (5f64.sqrt() - 1.) / 2.;
    let (mut a, mut b) = (lo, hi);
    let (mut fa, mut fb) = (f(a), f(b));
    let mut x1 = b - phi * (b - a);
    let mut x2 = a + phi * (b - a);
    let (mut f1, mut f2) = (f(x1), f(x2));
    for _ in 0..60 {
        if f1 < f2 {
            (a, fa) = (x1, f1);
            (x1, f1) = (x2, f2);
            x2 = a + phi * (b - a);
            f2 = f(x2);
        } else {
            (b, fb) = (x2, f2);
            (x2, f2) = (x1, f1);
            x1 = b - phi * (b - a);
            f1 = f(x1);
        }
        if x1.partial_cmp(&x2) != Some(std::cmp::Ordering::Less) {
            break;
        }
    }
    let slope = |(xa, ya): (f64, f64), (xb, yb): (f64, f64)| (yb - ya) / (xb - xa);
    let best = fa.max(fb).max(f1).max(f2);
    if !f1.is_finite() || !f2.is_finite() {
        return if f1 == f64::NEG_INFINITY && f2 == f64::NEG_INFINITY && best == f64::NEG_INFINITY {
            best
        } else {
            f64::INFINITY
        };
    }
    if x1.partial_cmp(&x2) != Some(std::cmp::Ordering::Less) {
        return best;
    }
    let s12 = slope((x1, f1), (x2, f2));
    // [a, x1]: below the (x1, x2) secant extended leftward.
    let left = f1 + (-s12).max(0.) * (x1 - a);
    // [x2, b]: below the same secant extended rightward.
    let right = f2 + s12.max(0.) * (b - x2);
    // [x1, x2]: below the (a, x1) secant extended right and the (x2, b)
    // secant extended left.
    let from_a = if fa.is_finite() {
        f1 + slope((a, fa), (x1, f1)).max(0.) * (x2 - x1)
    } else {
        f64::INFINITY
    };
    let from_b = if fb.is_finite() {
        f2 + (-slope((x2, f2), (b, fb))).max(0.) * (x2 - x1)
    } else {
        f64::INFINITY
    };
    let middle = from_a.min(from_b);
    // Without a finite end there is no bound: not proven (+infinity).
    best.max(left).max(right).max(middle)
}

impl Tri {
    /// Highest tip of a tool section with its axis at `p` resting on this
    /// triangle: an upper bound of max over triangle points q within `r1`
    /// of `p` of z(q) - profile(|q - p|) (None if the triangle does not reach
    /// the section). Concave objective on a convex domain: the maximum is at
    /// the facet's critical point (closed form), on an edge (golden section
    /// with a Lipschitz bound) or at a vertex; the disk's rim needs no
    /// candidate of its own (from the critical point outward the objective
    /// does not increase, so a rim maximum inside the triangle is matched by
    /// a triangle edge).
    fn section_contact(&self, p: [f64; 2], s: &Section) -> Option<f64> {
        let mut best: Option<f64> = None;
        fn take(best: &mut Option<f64>, z: f64) {
            *best = Some(best.map_or(z, |b| b.max(z)));
        }
        for v in self.p {
            if let Some(h) = s.profile((v[0] - p[0]).hypot(v[1] - p[1])) {
                take(&mut best, v[2] - h);
            }
        }
        for k in 0..3 {
            let (a, b) = (self.p[k], self.p[(k + 1) % 3]);
            let e = [b[0] - a[0], b[1] - a[1]];
            let w = [a[0] - p[0], a[1] - p[1]];
            let (qa, qb, qc) = (
                e[0] * e[0] + e[1] * e[1],
                2. * (e[0] * w[0] + e[1] * w[1]),
                w[0] * w[0] + w[1] * w[1] - s.r1 * s.r1,
            );
            if qa <= 0. {
                continue;
            }
            let disc = qb * qb - 4. * qa * qc;
            if disc < 0. {
                continue;
            }
            let root = disc.sqrt();
            let (t0, t1) = (
                ((-qb - root) / (2. * qa)).max(0.),
                ((-qb + root) / (2. * qa)).min(1.),
            );
            if t0 > t1 {
                continue;
            }
            let at = |t: f64| {
                let q: Vec3 = std::array::from_fn(|i| a[i] + t * (b[i] - a[i]));
                let d = (q[0] - p[0]).hypot(q[1] - p[1]).min(s.r1);
                q[2] - s.profile(d).unwrap_or(f64::INFINITY)
            };
            if s.arc > 0. {
                take(&mut best, concave_max_upper(at, t0, t1));
            } else {
                let lipschitz = (b[2] - a[2]).abs() + s.rise * qa.sqrt();
                take(&mut best, golden_max_bound(at, t0, t1, lipschitz));
            }
        }
        // Facet z = A x + B y + C: along the gradient (slope g) the objective
        // grows at g until r0, then at g - h'(d): a linear rise peaks at r0
        // when g <= rise, at r1 otherwise; an arc where its slope reaches g
        // (any point within r0 when g = 0).
        let [a, b, c] = self.p;
        let det = (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]);
        if det.abs() > 1e-300 {
            let gx = ((b[2] - a[2]) * (c[1] - a[1]) - (c[2] - a[2]) * (b[1] - a[1])) / det;
            let gy = ((c[2] - a[2]) * (b[0] - a[0]) - (b[2] - a[2]) * (c[0] - a[0])) / det;
            let g = gx.hypot(gy);
            let (q, h) = if g < 1e-15 {
                (p, s.z0)
            } else {
                let reach = s.reach_for_slope(g);
                (
                    [p[0] + gx / g * reach, p[1] + gy / g * reach],
                    s.profile(reach).unwrap(),
                )
            };
            if let Some(w) = self.bary(q)
                && w.iter().all(|w| *w >= -1e-12)
            {
                take(&mut best, w[0] * a[2] + w[1] * b[2] + w[2] * c[2] - h);
            }
        }
        best
    }
}

impl Tri {
    /// Highest tip of a bull-nose cutter (radius r, corner rc; rc = 0 flat,
    /// rc = r ball) at axis `p` resting on this triangle: the maximum of
    /// z(q) - profile(|q - p|) over triangle points q within r of `p`. The
    /// objective is concave (linear height minus a convex increasing profile
    /// of a convex distance), so its maximum is at the facet's critical point
    /// (closed form), on an edge (unique maximum, golden-section search), or
    /// at a vertex. None if the triangle does not reach the disk.
    fn bull_contact(&self, p: [f64; 2], r: f64, rc: f64) -> Option<f64> {
        self.bull_contact_above(p, r, rc, f64::NEG_INFINITY)
    }
    /// As `bull_contact`, an upper bound for proofs: each edge's objective is
    /// concave (linear height minus a convex non-decreasing profile of a
    /// convex distance), so it is bounded by `concave_max_upper`, with no
    /// Lipschitz assumption (the corner's slope is unbounded at the rim).
    /// Vertices and the facet's critical point are closed form.
    fn bull_contact_bound(&self, p: [f64; 2], r: f64, rc: f64) -> Option<f64> {
        self.bull_contact_with(p, r, rc, f64::NEG_INFINITY, true)
    }
    /// As `bull_contact`, skipping edges that cannot exceed `floor` (the
    /// profile only lowers heights, so an edge never beats its higher end).
    fn bull_contact_above(&self, p: [f64; 2], r: f64, rc: f64, floor: f64) -> Option<f64> {
        self.bull_contact_with(p, r, rc, floor, false)
    }
    fn bull_contact_with(
        &self,
        p: [f64; 2],
        r: f64,
        rc: f64,
        floor: f64,
        bound: bool,
    ) -> Option<f64> {
        if bound && self.p[0][2] == self.p[1][2] && self.p[1][2] == self.p[2][2] {
            // A level facet: the objective is its height less the profile of
            // the distance, which does not decrease outward, so the maximum
            // is at the least horizontal distance (exact, no search). Fine
            // flat floors are most of what a tangent pass reaches.
            let d = self.distance_xy(p);
            return (d <= r).then(|| self.p[0][2] - bull_profile(d, r, rc));
        }
        let mut best: Option<f64> = None;
        fn take(best: &mut Option<f64>, z: f64) {
            *best = Some(best.map_or(z, |b| b.max(z)));
        }
        let value = |q: Vec3| {
            let d = (q[0] - p[0]).hypot(q[1] - p[1]);
            (d <= r).then(|| q[2] - bull_profile(d, r, rc))
        };
        for v in self.p {
            if let Some(z) = value(v) {
                take(&mut best, z);
            }
        }
        for k in 0..3 {
            let (a, b) = (self.p[k], self.p[(k + 1) % 3]);
            if a[2].max(b[2]) <= floor.max(best.unwrap_or(f64::NEG_INFINITY)) {
                continue;
            }
            let e = [b[0] - a[0], b[1] - a[1]];
            let w = [a[0] - p[0], a[1] - p[1]];
            let (qa, qb, qc) = (
                e[0] * e[0] + e[1] * e[1],
                2. * (e[0] * w[0] + e[1] * w[1]),
                w[0] * w[0] + w[1] * w[1] - r * r,
            );
            if qa <= 0. {
                continue;
            }
            let disc = qb * qb - 4. * qa * qc;
            if disc < 0. {
                continue;
            }
            let root = disc.sqrt();
            let (t0, t1) = (
                ((-qb - root) / (2. * qa)).max(0.),
                ((-qb + root) / (2. * qa)).min(1.),
            );
            if t0 > t1 {
                continue;
            }
            let at = |t: f64| {
                let q: Vec3 = std::array::from_fn(|i| a[i] + t * (b[i] - a[i]));
                let d = (q[0] - p[0]).hypot(q[1] - p[1]).min(r);
                q[2] - bull_profile(d, r, rc)
            };
            take(
                &mut best,
                if bound {
                    concave_max_upper(at, t0, t1)
                } else {
                    golden_max(at, t0, t1, 40)
                },
            );
        }
        // Facet: the plane z = A x + B y + C; the critical point lies along
        // the gradient where the corner's slope equals the plane's.
        let [a, b, c] = self.p;
        let det = (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]);
        if det.abs() > 1e-300 {
            let gx = ((b[2] - a[2]) * (c[1] - a[1]) - (c[2] - a[2]) * (b[1] - a[1])) / det;
            let gy = ((c[2] - a[2]) * (b[0] - a[0]) - (b[2] - a[2]) * (c[0] - a[0])) / det;
            let g = gx.hypot(gy);
            let (q, drop) = if g < 1e-15 {
                (p, 0.)
            } else {
                let root = (1. + g * g).sqrt();
                let s = rc * g / root;
                let reach = r - rc + s;
                (
                    [p[0] + gx / g * reach, p[1] + gy / g * reach],
                    rc - rc / root,
                )
            };
            if let Some(w) = self.bary(q)
                && w.iter().all(|w| *w >= -1e-12)
            {
                take(&mut best, w[0] * a[2] + w[1] * b[2] + w[2] * c[2] - drop);
            }
        }
        best
    }
    /// Horizontal distance from `x` to the triangle (0 inside).
    fn distance_xy(&self, x: [f64; 2]) -> f64 {
        if let Some(w) = self.bary(x)
            && w.iter().all(|w| *w >= 0.)
        {
            return 0.;
        }
        (0..3)
            .map(|k| {
                let (p, q) = (self.p[k], self.p[(k + 1) % 3]);
                dist2_to_segment(x, [p[0], p[1]], [q[0], q[1]])
            })
            .fold(f64::INFINITY, f64::min)
            .sqrt()
    }
    /// Barycentric coordinates of an XY point, if the triangle is not vertical.
    fn bary(&self, x: [f64; 2]) -> Option<[f64; 3]> {
        let [a, b, c] = self.p;
        let det = (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]);
        if det.abs() < 1e-300 {
            return None;
        }
        let l1 = ((x[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (x[1] - a[1])) / det;
        let l2 = ((b[0] - a[0]) * (x[1] - a[1]) - (x[0] - a[0]) * (b[1] - a[1])) / det;
        Some([1. - l1 - l2, l1, l2])
    }
    /// Exact maximum height of the triangle over the XY capsule of radius r
    /// around segment p-q (None if they do not meet). Candidates are all
    /// extreme points of the intersection where a linear height can peak.
    fn max_in_capsule(&self, p: [f64; 2], q: [f64; 2], r: f64, eps: f64) -> Option<f64> {
        let inside_capsule = |x: [f64; 2]| dist2_to_segment(x, p, q) <= (r + eps) * (r + eps);
        let mut best: Option<f64> = None;
        let mut take = |z: f64| best = Some(best.map_or(z, |b: f64| b.max(z)));
        // Vertices.
        for v in self.p {
            if inside_capsule([v[0], v[1]]) {
                take(v[2]);
            }
        }
        // Edges crossing the capsule boundary: circles at p and q, and the
        // two straight sides.
        let d = [q[0] - p[0], q[1] - p[1]];
        let len = d[0].hypot(d[1]);
        let n = if len > 0. {
            [-d[1] / len, d[0] / len]
        } else {
            [0., 0.]
        };
        for k in 0..3 {
            let a = self.p[k];
            let b = self.p[(k + 1) % 3];
            let e = [b[0] - a[0], b[1] - a[1]];
            let mut at = |t: f64| {
                if (-1e-12..=1. + 1e-12).contains(&t) {
                    let t = t.clamp(0., 1.);
                    let x = [a[0] + t * e[0], a[1] + t * e[1]];
                    if inside_capsule(x) {
                        take(a[2] + t * (b[2] - a[2]));
                    }
                }
            };
            let ee = e[0] * e[0] + e[1] * e[1];
            if ee > 0. {
                for c in [p, q] {
                    // |a + t e - c|^2 = r^2
                    let f = [a[0] - c[0], a[1] - c[1]];
                    let bq = 2. * (f[0] * e[0] + f[1] * e[1]);
                    let cq = f[0] * f[0] + f[1] * f[1] - r * r;
                    let disc = bq * bq - 4. * ee * cq;
                    if disc >= 0. {
                        let s = disc.sqrt();
                        at((-bq - s) / (2. * ee));
                        at((-bq + s) / (2. * ee));
                    }
                }
                if len > 0. {
                    for side in [-1., 1.] {
                        // (a + t e - p - side r n) . n = 0
                        let den = e[0] * n[0] + e[1] * n[1];
                        if den != 0. {
                            let off = side * r - ((a[0] - p[0]) * n[0] + (a[1] - p[1]) * n[1]);
                            at(off / den);
                        }
                    }
                }
            }
        }
        // Interior extreme points: the tangent point of each end circle in
        // the gradient direction, the side ends, and the centers.
        let [a, b, c] = self.p;
        let nz = (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]);
        if nz.abs() > 1e-300 {
            let gx = ((b[2] - a[2]) * (c[1] - a[1]) - (c[2] - a[2]) * (b[1] - a[1])) / nz;
            let gy = ((b[0] - a[0]) * (c[2] - a[2]) - (c[0] - a[0]) * (b[2] - a[2])) / nz;
            let g = gx.hypot(gy);
            let mut points = vec![p, q];
            if g > 0. {
                for e in [p, q] {
                    points.push([e[0] + r * gx / g, e[1] + r * gy / g]);
                }
            }
            for e in [p, q] {
                for side in [-1., 1.] {
                    points.push([e[0] + side * r * n[0], e[1] + side * r * n[1]]);
                }
            }
            for x in points {
                if let Some(w) = self.bary(x)
                    && w.iter().all(|w| *w >= -1e-12)
                    && inside_capsule(x)
                {
                    take(w[0] * a[2] + w[1] * b[2] + w[2] * c[2]);
                }
            }
        }
        best
    }
}

impl Index {
    pub(crate) fn len(&self) -> usize {
        self.tris.len()
    }
    /// Proven: a bull-nose cutter (radius r, corner rc; rc = 0 flat) moving
    /// straight from tip `a` to tip `b`, lifted by `lift`, stays above every
    /// triangle. For one triangle the highest contact along the chord is a
    /// concave function of the position (the maximum over the triangle of a
    /// jointly concave function), and the tip height is linear, so their
    /// difference has one maximum: found by golden-section search over the
    /// stretch where the triangle is within reach (also convex to locate).
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn bull_chord_clear(
        &self,
        a: Vec3,
        b: Vec3,
        r: f64,
        rc: f64,
        lift: f64,
        seen: &mut [u32],
        stamp: &mut u32,
    ) -> bool {
        if a[2].min(b[2]) < self.floor + lift {
            return false;
        }
        *stamp = stamp.wrapping_add(1);
        if *stamp == 0 {
            seen.iter_mut().for_each(|s| *s = 0);
            *stamp = 1;
        }
        let low = a[2].min(b[2]);
        let at = |t: f64| [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
        let tip = |t: f64| a[2] + t * (b[2] - a[2]);
        let range = |k: usize| {
            let lo = ((a[k].min(b[k]) - r - self.origin[k]) / self.cell).floor();
            let hi = ((a[k].max(b[k]) + r - self.origin[k]) / self.cell).floor();
            let n = self.dims[k] as f64;
            if hi < 0. || lo >= n {
                return 0..0;
            }
            lo.max(0.) as usize..(hi.min(n - 1.) as usize + 1)
        };
        for y in range(1) {
            for x in range(0) {
                for &i in &self.bins[y * self.dims[0] + x] {
                    if seen[i as usize] == *stamp {
                        continue;
                    }
                    seen[i as usize] = *stamp;
                    let t = &self.tris[i as usize];
                    // Contacts never exceed the triangle's top.
                    if t.max[2] + lift <= low {
                        continue;
                    }
                    let dist = |s: f64| t.distance_xy(at(s));
                    // The closest approach (convex), then the reachable stretch.
                    let near = golden_argmin(dist, 0., 1., 50);
                    if dist(near) > r {
                        continue;
                    }
                    let edge = |inside: f64, outside: f64| {
                        if dist(outside) <= r {
                            return outside;
                        }
                        let (mut i, mut o) = (inside, outside);
                        for _ in 0..60 {
                            let m = (i + o) / 2.;
                            if dist(m) <= r {
                                i = m;
                            } else {
                                o = m;
                            }
                        }
                        i
                    };
                    let (s0, s1) = (edge(near, 0.), edge(near, 1.));
                    let gap = |s: f64| {
                        t.bull_contact(at(s), r, rc)
                            .map_or(f64::NEG_INFINITY, |c| c + lift - tip(s))
                    };
                    if golden_max(gap, s0, s1, 40) > 0. {
                        return false;
                    }
                }
            }
        }
        true
    }
    /// Highest tip of a tool made of `sections` with its axis at `p`
    /// resting on the part (an upper bound), at least the base.
    pub(crate) fn max_sections(
        &self,
        p: [f64; 2],
        sections: &[Section],
        seen: &mut [u32],
        stamp: &mut u32,
    ) -> f64 {
        self.max_sections_cancel(p, sections, seen, stamp, &|| false)
            .expect("uncancelled section query")
    }
    /// Pessimistic traversal and facet/section work, including duplicate bin entries.
    pub(crate) fn section_work(&self, sections: usize) -> u64 {
        self.entries
            .saturating_add(self.bins.len() as u64)
            .saturating_add(sections as u64)
            .saturating_add((self.tris.len() as u64).saturating_mul(sections as u64))
    }
    pub(crate) fn max_sections_cancel(
        &self,
        p: [f64; 2],
        sections: &[Section],
        seen: &mut [u32],
        stamp: &mut u32,
        cancel: &impl CancelSignal,
    ) -> Result<f64, String> {
        let mut reach = 0f64;
        let mut lowest = f64::INFINITY;
        for section in sections {
            if cancel.is_cancelled() {
                return Err("flat-area section query cancelled".into());
            }
            reach = reach.max(section.r1);
            lowest = lowest.min(section.z0);
        }
        let mut best = self.floor;
        let mut interrupted = false;
        self.visit_cancel(p, p, reach, seen, stamp, cancel, |t| {
            if t.max[2] - lowest <= best {
                return true;
            }
            for s in sections {
                if cancel.is_cancelled() {
                    interrupted = true;
                    return false;
                }
                if let Some(z) = t.section_contact(p, s) {
                    best = best.max(z);
                }
            }
            true
        })?;
        if interrupted {
            Err("flat-area section query cancelled".into())
        } else {
            Ok(best)
        }
    }
    /// Proven: a tool made of `sections` moving straight from tip `a` to
    /// tip `b`, lifted by `lift`, stays above every triangle. Per triangle and
    /// section the contact along the chord is concave in the position (the
    /// maximum over the triangle of a jointly concave function) and the tip
    /// height is linear, so the gap has one maximum; it is bounded from the
    /// final golden-section bracket by concavity (secant extensions), with no
    /// Lipschitz assumption (vertical walls included).
    pub(crate) fn sections_chord_clear(
        &self,
        a: Vec3,
        b: Vec3,
        sections: &[Section],
        lift: f64,
        seen: &mut [u32],
        stamp: &mut u32,
    ) -> bool {
        if a[2].min(b[2]) < self.floor + lift {
            return false;
        }
        let reach = sections.iter().map(|s| s.r1).fold(0., f64::max);
        let low = a[2].min(b[2]);
        let lowest = sections.iter().map(|s| s.z0).fold(f64::INFINITY, f64::min);
        let at = |t: f64| [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
        let tip = |t: f64| a[2] + t * (b[2] - a[2]);
        let mut clear = true;
        self.visit([a[0], a[1]], [b[0], b[1]], reach, seen, stamp, |t| {
            if t.max[2] - lowest + lift <= low {
                return true;
            }
            for s in sections {
                // The section's contact height is at most the triangle's
                // top less the section's lowest height above the tip.
                if t.max[2] - s.z0 + lift <= low {
                    continue;
                }
                // Triangle box farther than the section's reach from the
                // move's box: no contact.
                let gap = |k: usize| {
                    (t.min[k] - a[k].max(b[k]))
                        .max(a[k].min(b[k]) - t.max[k])
                        .max(0.)
                };
                if gap(0).hypot(gap(1)) > s.r1 {
                    continue;
                }
                let dist = |u: f64| t.distance_xy(at(u));
                let near = golden_argmin(dist, 0., 1., 60);
                if dist(near) > s.r1 {
                    continue;
                }
                let edge = |inside: f64, outside: f64| {
                    if dist(outside) <= s.r1 {
                        return outside;
                    }
                    let (mut i, mut o) = (inside, outside);
                    for _ in 0..60 {
                        let m = (i + o) / 2.;
                        if dist(m) <= s.r1 { i = m } else { o = m }
                    }
                    i
                };
                let (u0, u1) = (edge(near, 0.), edge(near, 1.));
                let gap = |u: f64| {
                    t.section_contact(at(u), s)
                        .map_or(f64::NEG_INFINITY, |c| c + lift - tip(u))
                };
                if concave_max_upper(gap, u0, u1) > 0. {
                    clear = false;
                    return false;
                }
            }
            true
        });
        clear
    }
    /// Proven: a bull-nose end (radius r, corner rc; rc = r a ball, rc = 0
    /// flat) moving straight from tip `a` to tip `b`, lifted by `lift`,
    /// stays above every triangle. As `sections_chord_clear`, with the
    /// curved profile itself (`Tri::bull_contact_bound`): per triangle the
    /// contact along the chord is concave in the position (the maximum over
    /// the triangle of a jointly concave function) and the tip height is
    /// linear, so the gap is bounded by `concave_max_upper`.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn bull_chord_bound_clear(
        &self,
        a: Vec3,
        b: Vec3,
        r: f64,
        rc: f64,
        lift: f64,
        seen: &mut [u32],
        stamp: &mut u32,
    ) -> bool {
        if a[2].min(b[2]) < self.floor + lift {
            return false;
        }
        let low = a[2].min(b[2]);
        let length = (b[0] - a[0]).hypot(b[1] - a[1]);
        let at = |t: f64| [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
        let tip = |t: f64| a[2] + t * (b[2] - a[2]);
        let mut clear = true;
        self.visit([a[0], a[1]], [b[0], b[1]], r, seen, stamp, |t| {
            // Contacts never exceed the triangle's top.
            if t.max[2] + lift <= low {
                return true;
            }
            let gap = |k: usize| {
                (t.min[k] - a[k].max(b[k]))
                    .max(a[k].min(b[k]) - t.max[k])
                    .max(0.)
            };
            if gap(0).hypot(gap(1)) > r {
                return true;
            }
            let dist = |u: f64| t.distance_xy(at(u));
            let near = golden_argmin(dist, 0., 1., 60);
            let closest = dist(near);
            if closest > r {
                return true;
            }
            // No contact along the chord is higher than the triangle's top
            // less the profile at the least horizontal distance (the profile
            // does not decrease outward). The distance is convex along the
            // chord, so its minimum lies in the final golden bracket around
            // `near`; the distance is 1-Lipschitz, so it is at least
            // `closest` less the chord's length times that bracket (bounded
            // generously here). Most triangles of a fine mesh that a
            // tangent pass reaches are dismissed by this alone, without the
            // contact search.
            let least = (closest - 1e-9 * (1. + length)).max(0.);
            if t.max[2] - bull_profile(least, r, rc) + lift <= low {
                return true;
            }
            let edge = |inside: f64, outside: f64| {
                if dist(outside) <= r {
                    return outside;
                }
                let (mut i, mut o) = (inside, outside);
                for _ in 0..60 {
                    let m = (i + o) / 2.;
                    if dist(m) <= r { i = m } else { o = m }
                }
                i
            };
            let (u0, u1) = (edge(near, 0.), edge(near, 1.));
            let gap = |u: f64| {
                t.bull_contact_bound(at(u), r, rc)
                    .map_or(f64::NEG_INFINITY, |c| c + lift - tip(u))
            };
            if concave_max_upper(gap, u0, u1) > 0. {
                clear = false;
                return false;
            }
            true
        });
        clear
    }
    /// Calls `f` once for each triangle binned near the XY capsule of radius
    /// `r` around p-q, until it returns false.
    fn visit(
        &self,
        p: [f64; 2],
        q: [f64; 2],
        r: f64,
        seen: &mut [u32],
        stamp: &mut u32,
        f: impl FnMut(&Tri) -> bool,
    ) {
        self.visit_cancel(p, q, r, seen, stamp, &|| false, f)
            .expect("uncancelled triangle query");
    }
    #[allow(clippy::too_many_arguments)]
    fn visit_cancel(
        &self,
        p: [f64; 2],
        q: [f64; 2],
        r: f64,
        seen: &mut [u32],
        stamp: &mut u32,
        cancel: &impl CancelSignal,
        mut f: impl FnMut(&Tri) -> bool,
    ) -> Result<(), String> {
        if cancel.is_cancelled() {
            return Err("flat-area section query cancelled".into());
        }
        *stamp = stamp.wrapping_add(1);
        if *stamp == 0 {
            seen.iter_mut().for_each(|s| *s = 0);
            *stamp = 1;
        }
        let range = |k: usize| {
            let lo = ((p[k].min(q[k]) - r - self.origin[k]) / self.cell).floor();
            let hi = ((p[k].max(q[k]) + r - self.origin[k]) / self.cell).floor();
            let n = self.dims[k] as f64;
            if hi < 0. || lo >= n {
                return 0..0;
            }
            lo.max(0.) as usize..(hi.min(n - 1.) as usize + 1)
        };
        for y in range(1) {
            for x in range(0) {
                if cancel.is_cancelled() {
                    return Err("flat-area section query cancelled".into());
                }
                for &i in &self.bins[y * self.dims[0] + x] {
                    if cancel.is_cancelled() {
                        return Err("flat-area section query cancelled".into());
                    }
                    if seen[i as usize] == *stamp {
                        continue;
                    }
                    seen[i as usize] = *stamp;
                    if !f(&self.tris[i as usize]) {
                        return Ok(());
                    }
                }
            }
        }
        Ok(())
    }
    /// Highest tip of a bull-nose cutter (radius r, corner rc) at axis `p`
    /// resting on the part, at least the base.
    pub(crate) fn max_bull(
        &self,
        p: [f64; 2],
        r: f64,
        rc: f64,
        seen: &mut [u32],
        stamp: &mut u32,
    ) -> f64 {
        *stamp = stamp.wrapping_add(1);
        if *stamp == 0 {
            seen.iter_mut().for_each(|s| *s = 0);
            *stamp = 1;
        }
        let mut best = self.floor;
        let range = |k: usize| {
            let lo = ((p[k] - r - self.origin[k]) / self.cell).floor();
            let hi = ((p[k] + r - self.origin[k]) / self.cell).floor();
            let n = self.dims[k] as f64;
            if hi < 0. || lo >= n {
                return 0..0;
            }
            lo.max(0.) as usize..(hi.min(n - 1.) as usize + 1)
        };
        for y in range(1) {
            for x in range(0) {
                for &i in &self.bins[y * self.dims[0] + x] {
                    if seen[i as usize] == *stamp {
                        continue;
                    }
                    seen[i as usize] = *stamp;
                    let t = &self.tris[i as usize];
                    if t.max[2] <= best {
                        continue;
                    }
                    if let Some(z) = t.bull_contact_above(p, r, rc, best) {
                        best = best.max(z);
                    }
                }
            }
        }
        best
    }
    pub(crate) fn new(
        mesh: &SurfaceMesh,
        floor: f64,
        r: f64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        Self::build(mesh, floor, r, None, &mut 0, u64::MAX, cancel)?
            .ok_or_else(|| "flat-area triangle index work overflow".into())
    }
    /// Refuse before index allocation when preparation exceeds the caller's work
    /// budget. Standard native surface validation is part of the bounds pass.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new_bounded(
        mesh: &SurfaceMesh,
        floor: f64,
        r: f64,
        tolerance: f64,
        work: &mut u64,
        max_work: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Self>, String> {
        mesh.validate_input(floor, tolerance)?;
        Self::build(mesh, floor, r, Some(tolerance), work, max_work, cancel)
    }
    #[allow(clippy::too_many_arguments)]
    fn build(
        mesh: &SurfaceMesh,
        floor: f64,
        r: f64,
        tolerance: Option<f64>,
        work: &mut u64,
        max_work: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Self>, String> {
        let count = mesh.triangles.len();
        if count == 0 || count > u32::MAX as usize {
            return Err("surface triangle index needs 1..=u32::MAX facets".into());
        }
        if !floor.is_finite() || !r.is_finite() || r <= 0. {
            return Err("surface index needs a finite base and positive finite radius".into());
        }
        let charge = |work: &mut u64, cost: u64| {
            if cost > max_work.saturating_sub(*work) {
                false
            } else {
                *work += cost;
                true
            }
        };
        // No index-sized allocations until all entry storage and work are known.
        if !charge(work, count as u64) {
            return Ok(None);
        }
        let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
        let mut scale = 1f64;
        for &t in mesh.triangles.iter() {
            if cancel.is_cancelled() {
                return Err("flat-area preparation cancelled".into());
            }
            if let Some(tolerance) = tolerance {
                SurfaceMesh::validate_triangle(t, floor, tolerance)?;
            } else if t.iter().flatten().any(|v| !v.is_finite() || v.abs() > 1e6) {
                return Err("surface vertices must be finite and bounded".into());
            }
            for p in t {
                for k in 0..2 {
                    lo[k] = lo[k].min(p[k]);
                    hi[k] = hi[k].max(p[k]);
                }
                scale = scale.max(p.iter().fold(0., |m: f64, v| m.max(v.abs())));
            }
        }
        let span = (hi[0] - lo[0]).max(hi[1] - lo[1]).max(r);
        // About one triangle per bin on average, never finer than r / 4.
        let cell = (span / (count as f64).sqrt().max(1.)).max(r / 4.).max(1e-6);
        let dims = [
            ((hi[0] - lo[0]) / cell).floor() as usize + 1,
            ((hi[1] - lo[1]) / cell).floor() as usize + 1,
        ];
        let bin_count = dims[0]
            .checked_mul(dims[1])
            .ok_or("surface index size overflow")?;
        let fixed_bytes = bin_count as u128
            * (std::mem::size_of::<Vec<u32>>() + std::mem::size_of::<usize>()) as u128
            // Include the per-facet query stamp array used alongside the index.
            + count as u128 * (std::mem::size_of::<Tri>() + std::mem::size_of::<u32>()) as u128;
        crate::resources::check_memory(fixed_bytes, "flat-area triangle index")?;
        let memory_budget = crate::resources::memory_budget();
        let range = |t: &[Vec3; 3]| {
            std::array::from_fn::<_, 2, _>(|k| {
                let min = t.iter().map(|p| p[k]).fold(f64::INFINITY, f64::min);
                let max = t.iter().map(|p| p[k]).fold(f64::NEG_INFINITY, f64::max);
                [
                    (((min - lo[k]) / cell) as usize).min(dims[k] - 1),
                    (((max - lo[k]) / cell) as usize).min(dims[k] - 1),
                ]
            })
        };
        if !charge(work, count as u64) {
            return Ok(None);
        }
        let mut entries = 0u64;
        for t in mesh.triangles.iter() {
            if cancel.is_cancelled() {
                return Err("flat-area preparation cancelled".into());
            }
            let [[x0, x1], [y0, y1]] = range(t);
            let n = (x1 - x0 + 1) as u64 * (y1 - y0 + 1) as u64;
            entries = entries
                .checked_add(n)
                .ok_or("surface index entry overflow")?;
            // Account for exact bin capacities, not just the vector headers.
            let bytes = fixed_bytes + entries as u128 * std::mem::size_of::<u32>() as u128;
            if memory_budget.is_some_and(|budget| bytes > budget as u128) {
                crate::resources::check_memory(bytes, "flat-area triangle index")?;
            }
            if entries.saturating_mul(2) > max_work.saturating_sub(*work) {
                return Ok(None);
            }
        }
        // Count then fill every entry; build each facet and initialise each bin.
        let allocation_work = entries
            .saturating_mul(2)
            .saturating_add(count as u64)
            .saturating_add(bin_count as u64);
        if !charge(work, allocation_work) {
            return Ok(None);
        }
        crate::resources::check_memory(
            fixed_bytes + entries as u128 * std::mem::size_of::<u32>() as u128,
            "flat-area triangle index",
        )?;
        let mut counts = vec![0usize; bin_count];
        for t in mesh.triangles.iter() {
            let [[x0, x1], [y0, y1]] = range(t);
            for y in y0..=y1 {
                for x in x0..=x1 {
                    if cancel.is_cancelled() {
                        return Err("flat-area preparation cancelled".into());
                    }
                    counts[y * dims[0] + x] += 1;
                }
            }
        }
        let mut bins = Vec::with_capacity(bin_count);
        for count in counts {
            if cancel.is_cancelled() {
                return Err("flat-area preparation cancelled".into());
            }
            bins.push(Vec::with_capacity(count));
        }
        let mut tris = Vec::with_capacity(count);
        for (i, t) in mesh.triangles.iter().enumerate() {
            let [[x0, x1], [y0, y1]] = range(t);
            for y in y0..=y1 {
                for x in x0..=x1 {
                    if cancel.is_cancelled() {
                        return Err("flat-area preparation cancelled".into());
                    }
                    bins[y * dims[0] + x].push(i as u32);
                }
            }
            tris.push(Tri {
                p: *t,
                min: std::array::from_fn(|k| t.iter().map(|p| p[k]).fold(f64::INFINITY, f64::min)),
                max: std::array::from_fn(|k| {
                    t.iter().map(|p| p[k]).fold(f64::NEG_INFINITY, f64::max)
                }),
            });
        }
        Ok(Some(Self {
            tris,
            origin: lo,
            cell,
            dims,
            bins,
            entries,
            floor,
            eps: 1e-12 * scale.max(1.),
        }))
    }
    /// Highest part point under the swept disk (radius r) from p to q, at
    /// least the base.
    pub(crate) fn max_height(
        &self,
        p: [f64; 2],
        q: [f64; 2],
        r: f64,
        seen: &mut [u32],
        stamp: &mut u32,
    ) -> f64 {
        *stamp = stamp.wrapping_add(1);
        if *stamp == 0 {
            seen.iter_mut().for_each(|s| *s = 0);
            *stamp = 1;
        }
        let mut best = self.floor;
        let range = |k: usize, a: f64, b: f64| {
            let lo = ((a.min(b) - r - self.origin[k]) / self.cell).floor();
            let hi = ((a.max(b) + r - self.origin[k]) / self.cell).floor();
            let n = self.dims[k] as f64;
            if hi < 0. || lo >= n {
                return 0..0;
            }
            lo.max(0.) as usize..(hi.min(n - 1.) as usize + 1)
        };
        for y in range(1, p[1], q[1]) {
            for x in range(0, p[0], q[0]) {
                for &i in &self.bins[y * self.dims[0] + x] {
                    if seen[i as usize] == *stamp {
                        continue;
                    }
                    seen[i as usize] = *stamp;
                    let t = &self.tris[i as usize];
                    if t.max[2] <= best {
                        continue;
                    }
                    if let Some(z) = t.max_in_capsule(p, q, r, self.eps) {
                        best = best.max(z);
                    }
                }
            }
        }
        best
    }
}

/// One cut pass at a fixed height: the segments of a closed loop in travel
/// order (an open piece when only stretches of the loop were proven and
/// allowed). A segment is a straight chord or an arc of the offset loop
/// (cavalier contours' bulge), proven as a whole or as its chords.
struct Piece {
    /// The pass's height; every segment runs at this Z.
    z: f64,
    segs: Vec<crate::pocket::Stroke>,
}
impl Piece {
    fn start(&self) -> Vec3 {
        let a = self.segs[0].a;
        [a[0], a[1], self.z]
    }
    fn end(&self) -> Vec3 {
        let b = self.segs[self.segs.len() - 1].b;
        [b[0], b[1], self.z]
    }
    fn length(&self) -> f64 {
        self.segs.iter().map(crate::profiles::stroke_length).sum()
    }
    /// The first `length` of the piece (all of it when shorter).
    fn prefix(&self, length: f64) -> Piece {
        let mut segs = vec![];
        let mut left = length;
        for seg in &self.segs {
            if left <= 0. {
                break;
            }
            let l = crate::profiles::stroke_length(seg);
            if l <= left {
                segs.push(*seg);
            } else {
                segs.push(sub_stroke(seg, 0., left / l));
            }
            left -= l;
        }
        Piece { z: self.z, segs }
    }
    fn strokes(&self) -> Vec<crate::pocket::Stroke> {
        // The ramp walks the pass stroke by stroke; its arcs are split like
        // the cut moves so no ramp leg sweeps a chord that cuts its corner.
        self.segs
            .iter()
            .flat_map(|s| split_arc(s, ARC_MOVE_SAGITTA))
            .collect()
    }
}

/// One entry and the passes cut before the tool retracts: consecutive passes
/// are joined by links at depth.
struct Chain {
    pieces: Vec<Piece>,
    /// Side-entry candidates of the first pass: vertices (at most one per
    /// 1 mm of the pass) with the outward normal, away from the region (left
    /// of travel).
    side: Vec<([f64; 2], [f64; 2])>,
    /// The level's face (the flat the pass finishes): an entry may only plunge
    /// down beside it, never onto it.
    face: std::sync::Arc<crate::planar::PreparedRegion>,
}

/// How the chain is entered, decided against the live stock when reached.
enum Entry {
    /// Vertical plunge in air at `p`, then a lead at cutting feed to the
    /// pass's start.
    Side { p: [f64; 2] },
    /// Vertical plunge at the pass's start.
    Plunge,
    /// Ramp down from `top` (reached through air) to the pass's start: the
    /// zigzag's curves down the first pass (lines and arcs on the pass,
    /// helices where a leg descends along an arc), or an arc ramp's descent
    /// along the pass's line (`FlatPlan::arc_ramp`), whose first `air`
    /// moves run through air above the stock (at the cutting feed, like a
    /// lead) and the rest at the ramp's feed; or an arc ramp onto the start of
    /// a flat arc lead-in, `then`.
    Ramp {
        top: f64,
        curves: Vec<Curve>,
        air: usize,
        /// A flat arc lead-in from the descent's end to the pass's start
        /// (`FlatPlan::ramped_arc_lead_in`).
        then: Option<Curve>,
    },
    /// Vertical plunge in air at `lead`'s start, then the tangent arc lead-in
    /// `lead` to the pass's start: flat at the pass's height (a lead at
    /// cutting feed) or, when the plunge point is over the level's finished
    /// face, a helix descending to it at the declared ramp's angle (at the
    /// ramp's feed).
    Arc { lead: Curve, helical: bool },
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase {
    Lift,
    Approach,
    Descend,
    Enter,
    Turn,
    Lead,
    ArcIn,
    Ramp,
    Cut,
    PieceEnd,
    Leave,
    Rise,
    Done,
}

pub struct FlatPlan {
    op: FlatOperation,
    spec: StockSpec,
    heights: crate::heights::Heights,
    index: Index,
    seen: Vec<u32>,
    stamp: u32,
    chains: Vec<Chain>,
    chain: usize,
    piece: usize,
    seg: usize,
    phase: Phase,
    radius: f64,
    limits: Option<crate::limits::PreparedLimits>,
    entry: Option<Entry>,
    ramp: std::collections::VecDeque<Curve>,
    /// How many of `ramp`'s first moves run through air (`Entry::Ramp`).
    ramp_air: usize,
    lead: Option<(Curve, bool)>,
    turn: std::collections::VecDeque<Curve>,
    after_turn: Phase,
    initial: Vec3,
    position: Vec3,
    next_id: u64,
    expected_generation: Option<u64>,
    drained: bool,
    report: FlatReport,
}

impl FlatPlan {
    pub fn prepare(
        op: FlatOperation,
        tool: &ToolAssembly,
        spec: &StockSpec,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        spec.grid()?;
        let prepared = tool.prepare()?;
        let cutter = prepared.cutter().definition();
        let r = cutter.diameter_mm / 2.;
        let guard = 16. * op.tolerance_mm;
        let values = [
            op.floor_z_mm,
            op.stepover_mm,
            op.clearance_z_mm,
            op.stock_to_leave_mm,
            op.radial_stock_to_leave_mm,
            op.level_tolerance_mm,
            op.feed_mm_min,
            op.plunge_mm_min,
            op.tolerance_mm,
            op.spindle_rpm,
        ];
        crate::heights::require(
            "flat-area finishing",
            [
                (
                    valid_id(&op.id),
                    format!("id {:?} is not a valid identifier", op.id),
                ),
                (
                    op.tool == tool.id,
                    format!(
                        "tool {:?} does not match the assembly {:?}",
                        op.tool, tool.id
                    ),
                ),
                (
                    matches!(cutter.shape, CutterShape::Flat) && cutter.center_cutting,
                    "this operation needs a flat, center-cutting end mill".into(),
                ),
                (
                    values.iter().all(|x| x.is_finite() && x.abs() <= 1e6),
                    "all numeric parameters must be finite".into(),
                ),
                (
                    (1e-7..=0.01).contains(&op.tolerance_mm),
                    format!("tolerance_mm {} must be within 1e-7..0.01", op.tolerance_mm),
                ),
                (
                    op.stepover_mm > guard && op.stepover_mm <= 2. * r,
                    format!(
                        "stepover_mm {} must be positive and at most the cutter diameter {}",
                        op.stepover_mm,
                        2. * r
                    ),
                ),
                (
                    op.stock_to_leave_mm >= 0.,
                    format!(
                        "stock_to_leave_mm {} must not be negative",
                        op.stock_to_leave_mm
                    ),
                ),
                (
                    op.radial_stock_to_leave_mm >= 0.,
                    format!(
                        "radial_stock_to_leave_mm {} must not be negative",
                        op.radial_stock_to_leave_mm
                    ),
                ),
                (
                    op.level_tolerance_mm >= op.tolerance_mm && op.level_tolerance_mm <= 0.1,
                    format!(
                        "level_tolerance_mm {} must be within tolerance_mm..0.1",
                        op.level_tolerance_mm
                    ),
                ),
                (
                    op.floor_z_mm >= spec.min_mm[2] && op.floor_z_mm < spec.max_mm[2],
                    format!(
                        "floor_z_mm {} must be within the stock Z range {}..{}",
                        op.floor_z_mm, spec.min_mm[2], spec.max_mm[2]
                    ),
                ),
                (
                    initial[2] >= op.clearance_z_mm,
                    format!(
                        "start Z {} is below clearance_z_mm {}",
                        initial[2], op.clearance_z_mm
                    ),
                ),
                (
                    op.feed_mm_min > 0. && op.plunge_mm_min > 0. && op.spindle_rpm > 0.,
                    "feeds and spindle_rpm must be positive".into(),
                ),
                (
                    (1..=9).contains(&op.work_offset),
                    format!("work_offset {} must be 1..9", op.work_offset),
                ),
                (
                    op.tool_number > 0 && op.length_register > 0,
                    "tool_number and length_register must be positive".into(),
                ),
                (
                    first_id != 0 && first_id.checked_add(1).is_some(),
                    "motion ID budget exhausted".into(),
                ),
            ],
        )?;
        if let Some(entry) = &op.entry {
            let Some(ramp) = entry.ramp() else {
                return Err(
                    "flat-area finishing: only a RampEntry enters a run (a helix has no place on a straight run); use entry=RampEntry(angle_deg, ...) or declare none"
                        .into(),
                );
            };
            ramp.validate(op.tolerance_mm, op.plunge_mm_min)?;
        }
        let heights = crate::heights::Heights::resolve(
            op.clearance_z_mm,
            op.retract_z_mm,
            op.feed_z_mm,
            spec.max_mm[2],
            guard,
        )?;
        let index = Index::new(&op.surface, op.floor_z_mm, r, cancel)?;
        // Flat levels: horizontal triangles above the base, grouped.
        let mut flats: Vec<(f64, [f64; 2], [f64; 2])> = vec![];
        for t in &index.tris {
            let z = t.p[0][2];
            if t.p.iter().all(|v| (v[2] - z).abs() <= op.tolerance_mm)
                && t.bary([t.p[0][0], t.p[0][1]]).is_some()
                && z > op.floor_z_mm + op.level_tolerance_mm
            {
                flats.push((z, t.min, [t.max[0], t.max[1]]));
            }
        }
        flats.sort_by(|a, b| b.0.total_cmp(&a.0));
        let mut levels: Vec<(f64, [f64; 2], [f64; 2])> = vec![];
        for (z, lo, hi) in flats {
            match levels.last_mut() {
                Some(l) if l.0 - z <= op.level_tolerance_mm => {
                    for k in 0..2 {
                        l.1[k] = l.1[k].min(lo[k]);
                        l.2[k] = l.2[k].max(hi[k]);
                    }
                }
                _ => levels.push((z, lo, hi)),
            }
        }
        let mut selected = Vec::new();
        if op.floors.is_empty() {
            selected.extend(levels.iter().map(|l| (l.0, None)));
        } else {
            for floor in &op.floors {
                if cancel.is_cancelled() {
                    return Err("flat-area planning cancelled".into());
                }
                if !floor.z_mm.is_finite()
                    || floor.z_mm <= op.floor_z_mm + op.level_tolerance_mm
                    || !levels
                        .iter()
                        .any(|l| (l.0 - floor.z_mm).abs() <= op.level_tolerance_mm)
                {
                    return Err(format!(
                        "flat-area CAD floor Z{} has no matching horizontal mesh level",
                        floor.z_mm
                    ));
                }
                selected.push((floor.z_mm, Some(&floor.region)));
            }
            selected.sort_by(|a, b| b.0.total_cmp(&a.0));
        }
        if let Some((z, _)) = selected.iter().find(|(z, _)| {
            *z > spec.max_mm[2] || spec.max_mm[2] - *z > cutter.cutting_length_mm - guard
        }) {
            return Err(format!(
                "invalid flat-area finishing parameters: flat level Z {} is above the stock top or deeper below it than the cutting length {}",
                z, cutter.cutting_length_mm
            ));
        }
        let region_limits = op
            .limits
            .as_ref()
            .filter(|l| !l.is_empty())
            .map(|l| crate::limits::PreparedLimits::new(l, r, op.tolerance_mm))
            .transpose()?;
        // The part as a height field over XY, for the per-chord proofs and the
        // side entries' leads.
        let target = crate::roughing::PreparedRoughingTarget::prepare(
            op.surface.clone(),
            op.tolerance_mm,
            cancel,
        )?;
        let mut seen = vec![0u32; index.len()];
        let mut stamp = 0u32;
        let h = op.stepover_mm.min(r) / 4.;
        let mut chains: Vec<Chain> = vec![];
        let (mut rows, mut loops, mut links, mut dropped) = (0usize, 0usize, 0usize, 0f64);
        let mut wall_passes = 0;
        let mut tool_xy = [initial[0], initial[1]];
        for &(level, cad) in &selected {
            if cancel.is_cancelled() {
                return Err("flat-area planning cancelled".into());
            }
            let z = level + op.stock_to_leave_mm;
            // The tool centres resting on this level, as a planar region, and
            // the level's face (for keeping entries off it).
            let level_region = if let Some(cad) = cad {
                Some(cad_level_region(&target, &op, level, r, cad, cancel)?)
            } else {
                level_region(
                    &target,
                    level,
                    op.level_tolerance_mm,
                    r,
                    op.radial_stock_to_leave_mm,
                    op.stepover_mm,
                    op.tolerance_mm,
                    cancel,
                )?
            };
            let Some(LevelRegion {
                centres: region,
                face,
                shadow_wall_reserve_mm,
            }) = level_region
            else {
                continue;
            };
            // Offset loops per connected component, outside in.
            // A loop cut only in stretches is cut through a gap between two
            // of them when that takes no longer than a new entry's descent
            // from the feed height at the plunge feed (and through any gap of
            // up to two stepovers, as a link at depth would).
            let bridge = (2. * op.stepover_mm)
                .max((heights.feed - z).max(0.) * op.feed_mm_min / op.plunge_mm_min);
            let groups = level_loops(&region, r, op.stepover_mm, op.tolerance_mm, bridge, cancel)?;
            let level_chains = chains.len();
            let mut walls = Vec::new();
            // Mesh-derived contours include the reserve added around the
            // raised shadow. Include it only when identifying which contours
            // follow a wall. The path proof below still uses the cutter.
            let wall_probe = r
                + op.radial_stock_to_leave_mm
                + shadow_wall_reserve_mm
                + region.definition().boundary_error_mm
                + op.surface.surface_error_mm
                + guard;
            // Nearest component first.
            let mut order: Vec<usize> = (0..groups.len()).collect();
            order.sort_by(|&a, &b| {
                groups[a]
                    .distance(tool_xy)
                    .total_cmp(&groups[b].distance(tool_xy))
                    .then(a.cmp(&b))
            });
            for gi in order {
                loops += groups[gi].outers.len() + groups[gi].holes.len();
                for (depth, ring, runs) in groups[gi].ordered(tool_xy, 2. * r, 2. * op.stepover_mm)
                {
                    if cancel.is_cancelled() {
                        return Err("flat-area planning cancelled".into());
                    }
                    // An inner loop is cut only where its disk reaches floor
                    // the loops before it left; a short stretch between two
                    // such places is cut through rather than left for a link.
                    let ring = match runs {
                        // Each closed loop starts at its vertex nearest the tool.
                        None => rotate_to_nearest(ring, tool_xy),
                        // Open stretches in loop order from the one starting
                        // nearest the tool; prove_ring splits them into
                        // pieces where they do not meet.
                        Some(runs) => open_runs_from_nearest(runs, tool_xy),
                    };
                    if ring.is_empty() {
                        continue;
                    }
                    if op.finish_walls
                        && depth == 0.
                        && ring.iter().any(|s| {
                            let mid = stroke_point(s, 0.5);
                            index.max_height(mid, mid, wall_probe, &mut seen, &mut stamp)
                                > z + op.tolerance_mm
                        })
                    {
                        walls.push(ring);
                        continue;
                    }
                    // Prove each chord; halve failures down to h / 16, keep
                    // only the stretches inside the limits.
                    let pieces = prove_ring(
                        &ring,
                        z,
                        r,
                        &index,
                        &mut seen,
                        &mut stamp,
                        h,
                        op.tolerance_mm,
                        region_limits.as_ref(),
                        &mut dropped,
                        cancel,
                    )?;
                    for piece in pieces {
                        rows += 1;
                        // Consecutive passes join into one chain by a link at
                        // depth when the link is short, proven and allowed;
                        // otherwise the chain ends and a new one begins.
                        let linked = chains.len() > level_chains && {
                            let prev = chains[chains.len() - 1].pieces.last().unwrap().end();
                            let next = piece.start();
                            let length = (next[0] - prev[0]).hypot(next[1] - prev[1]);
                            length <= 2. * op.stepover_mm
                                && index.max_height(
                                    [prev[0], prev[1]],
                                    [next[0], next[1]],
                                    r,
                                    &mut seen,
                                    &mut stamp,
                                ) <= z + op.tolerance_mm
                                && allows_segment(
                                    region_limits.as_ref(),
                                    [prev[0], prev[1]],
                                    [next[0], next[1]],
                                    z,
                                    op.tolerance_mm,
                                )?
                        };
                        if linked {
                            let last = chains.len() - 1;
                            chains[last].pieces.push(piece);
                            links += 1;
                        } else {
                            let side = side_candidates(&piece);
                            chains.push(Chain {
                                pieces: vec![piece],
                                side,
                                face: std::sync::Arc::clone(&face),
                            });
                        }
                        let end = chains[chains.len() - 1].pieces.last().unwrap().end();
                        tool_xy = [end[0], end[1]];
                    }
                }
            }
            if let Some(cad) = cad {
                // The nominal outline is reachable at an open edge even at
                // a sharp corner. Use an enlarged disk for this proof so
                // an overhanging pass cannot take the declared wall stock.
                let reserve = op.radial_stock_to_leave_mm + cad.boundary_error_mm + op.tolerance_mm;
                for component in &cad.components {
                    let outline = boundary_strokes(&component.outer, false, op.tolerance_mm)?;
                    let mut omitted = 0.;
                    let pieces = prove_ring(
                        &outline,
                        z,
                        r + reserve,
                        &index,
                        &mut seen,
                        &mut stamp,
                        h,
                        op.tolerance_mm,
                        region_limits.as_ref(),
                        &mut omitted,
                        cancel,
                    )?;
                    for piece in pieces {
                        let side = side_candidates(&piece);
                        tool_xy = [piece.end()[0], piece.end()[1]];
                        chains.push(Chain {
                            pieces: vec![piece],
                            side,
                            face: face.clone(),
                        });
                        rows += 1;
                    }
                }
            }
            for ring in walls {
                let ring = rotate_to_nearest(ring, tool_xy);
                let mut omitted = 0.;
                let pieces = prove_ring(
                    &ring,
                    z,
                    r,
                    &index,
                    &mut seen,
                    &mut stamp,
                    h,
                    op.tolerance_mm,
                    region_limits.as_ref(),
                    &mut omitted,
                    cancel,
                )?;
                if omitted > 0. || pieces.is_empty() {
                    return Err(format!(
                        "flat-area wall pass at Z{level} could not be proven within the part and limits"
                    ));
                }
                wall_passes += 1;
                for piece in pieces {
                    tool_xy = [piece.end()[0], piece.end()[1]];
                    let side = side_candidates(&piece);
                    chains.push(Chain {
                        pieces: vec![piece],
                        side,
                        face: face.clone(),
                    });
                    rows += 1;
                }
            }
            if cad.is_some() && chains.len() == level_chains {
                return Err(format!(
                    "flat-area CAD floor Z{level} has no proven pass for this cutter"
                ));
            }
        }
        if op.finish_walls && wall_passes == 0 {
            return Err(
                "flat-area wall finishing needs a reachable wall beside a selected flat".into(),
            );
        }
        let mut reported_levels: Vec<_> = selected.iter().map(|(z, _)| *z).collect();
        reported_levels.dedup();
        let report = FlatReport {
            strategy: "flat",
            geometric_residual_status: ResidualStatus::Pending,
            levels_z_mm: reported_levels,
            rows,
            runs: chains.len(),
            loops,
            wall_passes,
            links_at_depth: links,
            cut_length_mm: chains
                .iter()
                .flat_map(|c| c.pieces.iter())
                .map(Piece::length)
                .sum(),
            dropped_length_mm: dropped,
            runs_completed: 0,
            ramped_entries: 0,
            arc_lead_ins: 0,
            arc_lead_outs: 0,
            rounded_plunges: 0,
            arc_ramps: 0,
            clearance_proof: "exact maximum part height under each horizontal cut's swept disk (triangle-capsule extreme points) at most the cut height plus tolerance; contour-parallel offset passes outside in, side entry in air on open edges and every vertical plunge beside the level's face (a radius outside it, the face region a superset of the flat within the shadows' stated errors), links at depth and when chains break",
        };
        Ok(Self {
            op,
            spec: spec.clone(),
            heights,
            index,
            seen,
            stamp: 0,
            chains,
            chain: 0,
            piece: 0,
            seg: 0,
            phase: Phase::Lift,
            radius: r,
            limits: region_limits,
            entry: None,
            ramp: Default::default(),
            ramp_air: 0,
            lead: None,
            turn: Default::default(),
            after_turn: Phase::Cut,
            initial,
            position: initial,
            next_id: first_id,
            expected_generation: None,
            drained: false,
            report,
        })
    }
    pub fn report(&self) -> &FlatReport {
        &self.report
    }
    /// What enters the pass at its start after the rapid descent: a straight
    /// plunge where no live stock stands above the pass's height there (the
    /// cells' upper bounds, within a tolerance) and the disk comes down beside
    /// the level's face, otherwise a ramp from the stock's top over the whole
    /// pass, down the pass by the declared `RampEntry`. When the pass starts
    /// over the level's face with no stock above (so the plain plunge is
    /// refused) and the stock over the pass is that face's already-cut floor,
    /// the ramp leaves from the cut height plus the depth the declared ramp
    /// drops over its `min_length_mm` instead: the descent stops in air above
    /// the face and the ramp comes down along the pass, never straight onto
    /// the finished flat. Each ramp point lies vertically above the proven
    /// pass, so it clears the part as the pass does. A pass the ramp does not
    /// fit, or stock above a pass with no entry declared, is refused: a flat
    /// end mill is never plunged into stock.
    fn plunge_or_ramp(
        &self,
        piece: &Piece,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<(f64, Vec<Curve>)>, String> {
        let a = piece.start();
        let at_start = stock.top_under_disk([a[0], a[1]], self.radius);
        if at_start <= a[2] + self.op.tolerance_mm && self.beside_face([a[0], a[1]])? {
            return Ok(None);
        }
        let length = piece.length();
        let top = piece_stock_top(piece, stock, self.radius, cancel)?;
        let b = piece.end();
        let which = format!(
            "flat-area run from ({:.3}, {:.3}) to ({:.3}, {:.3}) at Z{:.3}, {length:.3} mm long, has stock up to Z{top:.3} ({:.3} mm above it, {:.3} mm above it at its start)",
            a[0],
            a[1],
            b[0],
            b[1],
            a[2],
            top - a[2],
            at_start - a[2]
        );
        let Some(entry) = self.op.entry.as_ref().and_then(|e| e.ramp()) else {
            return Err(format!(
                "{which}, and the operation declares no entry: a flat end mill is not plunged into stock. Declare entry=RampEntry(angle_deg, ...) for the tool, or rough or rest-rough this area first"
            ));
        };
        if length <= 16. * self.op.tolerance_mm {
            return Err(format!(
                "{which}: the run is shorter than 16 tolerances, too short to ramp on. Rough or rest-rough this area first"
            ));
        }
        // Moves are read through their endpoints (posted programs, checks):
        // a helix leg along an arc of the pass drops its depth over the arc,
        // so over its chord it is steeper by the arc's length over the
        // chord's. The ramp descends at the angle whose slope over every
        // move's chord stays within the declared one.
        let chain = piece.strokes();
        let stretch = chain
            .iter()
            .filter(|s| s.arc.is_some())
            .map(|s| {
                let chord = (s.b[0] - s.a[0]).hypot(s.b[1] - s.a[1]);
                if chord > 0. {
                    crate::profiles::stroke_length(s) / chord
                } else {
                    1.
                }
            })
            .fold(1., f64::max);
        let entry = &crate::entry::RampEntry {
            angle_rad: (entry.angle_rad.tan() / stretch).atan(),
            ..entry.clone()
        };
        // Reaching here with no stock above the start means the start is over
        // the level's face (the plain plunge above refused it for that). When
        // the stock over the whole pass is that face's already-cut floor (the
        // cells' upper bounds sit a tolerance above the cut height after a
        // cut), a ramp from the stock top has no depth: the descent to it
        // would end straight down on the finished face. The ramp then leaves
        // from the cut height plus the depth the declared ramp drops over its
        // `min_length_mm` — `min_length_mm * tan(angle)`, the drop of one leg
        // at the declared angle over the shortest span the tool may ramp on
        // (the whole pass when none is declared: a one-leg ramp reverses on
        // no span) — and descends along the pass as for stock. The stock top
        // wins when it is higher (real stock along the pass): the descent to
        // it stops at or above every stock cell under the pass, in air.
        //
        // The ramp travels only the start of the pass when one leg descends
        // the whole depth (the leg's length is the depth over the angle's
        // tangent): it starts above the stock over that stretch, not over the
        // whole pass, whose far side may run past part standing far above
        // (cells along a taller wall). Starting from the floor above, the
        // start height rises to the stock top over the stretch the ramp from
        // it would travel until that stretch has no stock above it; when the
        // stretch reaches the whole pass (the ramp reverses on it) the stock
        // top over the whole pass is the floor.
        let tan = entry.angle_rad.tan();
        let mut target = if at_start <= a[2] + self.op.tolerance_mm {
            let span = entry.min_length_mm.unwrap_or(length);
            a[2] + span * tan
        } else {
            at_start
        };
        loop {
            if cancel.is_cancelled() {
                return Err("flat-area planning cancelled".into());
            }
            let span = (target - a[2]) / tan;
            if span >= length {
                target = target.max(top);
                break;
            }
            let over = piece_stock_top(&piece.prefix(span), stock, self.radius, cancel)?;
            if over <= target {
                break;
            }
            target = over;
        }
        if target > self.heights.feed {
            return Err(format!(
                "{which}, ramp start Z{target:.3} above the feed height Z{:.3}: raise feed_z_mm/retract_z_mm or rough this area first",
                self.heights.feed
            ));
        }
        let curves = entry
            .zigzag(&chain, target, a[2])
            .map_err(|why| format!("{which}: {why}. Use a steeper declared angle, a longer run, or rough this area first"))?;
        // The curves run from the ramp's start (at the ramp's start height)
        // to the pass's start; the vertical descent to the ramp's start
        // happens in `next_motion_with_stock`.
        Ok(Some((target, curves)))
    }
    /// Whether the disk of radius `self.radius` swept along the arc about
    /// `c` of radius `rho` from angle `start` through `sweep` stays clear of
    /// the part at height `z`: the arc's chords at the tolerance's sagitta,
    /// each proven with the capsule of radius r + sagitta, which contains the
    /// arc's swept disk (as for the arcs of a pass in `prove_ring`).
    fn arc_clear(&mut self, c: [f64; 2], rho: f64, start: f64, sweep: f64, z: f64) -> bool {
        let tol = self.op.tolerance_mm;
        let half = (1. - tol / rho).clamp(-1., 1.).acos().max(1e-9);
        let count = ((sweep.abs() / (2. * half)).ceil().max(1.) as usize).min(4096);
        let sagitta = rho * (1. - (sweep.abs() / (2. * count as f64)).cos());
        let at = |t: f64| {
            let angle = start + sweep * t;
            [c[0] + rho * angle.cos(), c[1] + rho * angle.sin()]
        };
        (0..count).all(|i| {
            let (p, q) = (
                at(i as f64 / count as f64),
                at((i + 1) as f64 / count as f64),
            );
            self.index
                .max_height(p, q, self.radius + sagitta, &mut self.seen, &mut self.stamp)
                <= z + tol
        })
    }

    /// The arc lead-in to the pass starting at `v` (height `z`) along the unit
    /// tangent `t`, as in established CAM practice: a quarter circle of the
    /// tool's radius, tangent to the pass at `v`, whose centre lies on the
    /// normal at `v`, entered by a vertical plunge at its start. Of the two
    /// sides of the pass, a side qualifies when the plunge point's disk
    /// touches no stock above the pass's height (on an inner pass that is
    /// the floor the passes before it finished, which lies more than a
    /// stepover from the pass once the radius exceeds the stepover), the arc
    /// is proven clear of the part, and the limits allow it. A plunge point a
    /// radius beside the level's face comes down in air beside the flat and
    /// the lead is flat at the pass's height; over the face the plunge stops
    /// above the floor and the lead descends to the pass as a helix at the
    /// declared ramp's angle over its chord (no ramp declared: no such side).
    /// Beside the face wins; `None` when no side qualifies.
    fn arc_lead_in(
        &mut self,
        stock: &Stock,
        v: [f64; 2],
        t: [f64; 2],
        z: f64,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Entry>, String> {
        let tol = self.op.tolerance_mm;
        let rho = self.radius;
        let mut found: Option<Entry> = None;
        for s in [1., -1.] {
            let n = [-t[1] * s, t[0] * s];
            let c = [v[0] + rho * n[0], v[1] + rho * n[1]];
            let p = [c[0] - rho * t[0], c[1] - rho * t[1]];
            if stock.top_under_disk(p, rho) > z + tol {
                continue;
            }
            // Over the face, a plunge whose last radius turns to horizontal
            // (`plunge_turn`, along the arc's tangent at `p`, which is -n)
            // reaches the floor moving sideways and never dwells on it: the
            // lead is flat there too.
            let beside = self.beside_face(p)? || {
                let d = [-n[0], -n[1]];
                self.plunge_turn(stock, p, d, z, cancel)?.is_some()
            };
            let rise = if beside {
                0.
            } else {
                match self.op.entry.as_ref().and_then(|e| e.ramp()) {
                    Some(ramp) => rho * std::f64::consts::SQRT_2 * ramp.angle_rad.tan(),
                    None => continue,
                }
            };
            if z + rise > self.heights.feed {
                continue;
            }
            let start = (p[1] - c[1]).atan2(p[0] - c[0]);
            let sweep = s * std::f64::consts::FRAC_PI_2;
            if !self.arc_clear(c, rho, start, sweep, z) {
                continue;
            }
            let lead = if beside {
                Curve::Arc {
                    start: [p[0], p[1], z],
                    end: [v[0], v[1], z],
                    center: [c[0], c[1], z],
                    plane: crate::motion::Plane::Xy,
                    sweep_rad: sweep,
                }
            } else {
                Curve::Helix {
                    start: [p[0], p[1], z + rise],
                    end: [v[0], v[1], z],
                    center: [c[0], c[1], z + rise],
                    plane: crate::motion::Plane::Xy,
                    sweep_rad: sweep,
                }
            };
            if let Some(l) = &self.limits
                && !l.allows_curve(&lead, true)?
            {
                continue;
            }
            let better = matches!(found, None | Some(Entry::Arc { helical: true, .. })) && beside;
            if found.is_none() || better {
                found = Some(Entry::Arc {
                    lead,
                    helical: !beside,
                });
            }
        }
        Ok(found)
    }

    /// The arc lead-in to the pass starting at `v` (height `z`, unit tangent
    /// `t`) entered through shallow stock: the flat quarter circle of
    /// `arc_lead_in`, on a side where stock stands above the pass at its
    /// start, reached by an arc ramp (`arc_ramp`) that descends along the
    /// arc's tangent there (or, where the part is in the way, along the
    /// pass's direction) and ends level at its start. The descent stays a
    /// radius and more away from the pass, so it reads the stock beside the
    /// pass rather than at the pass (next to a wall the stock grid carries
    /// the wall's height in the cells beside it); the arc itself runs level
    /// at the pass's height, as the passes do. `None` when neither side
    /// qualifies.
    fn ramped_arc_lead_in(
        &mut self,
        stock: &Stock,
        v: [f64; 2],
        t: [f64; 2],
        z: f64,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Entry>, String> {
        let tol = self.op.tolerance_mm;
        let rho = self.radius;
        let n = t[0].hypot(t[1]);
        if n <= 0. {
            return Ok(None);
        }
        let t = [t[0] / n, t[1] / n];
        for s in [1., -1.] {
            let n = [-t[1] * s, t[0] * s];
            let c = [v[0] + rho * n[0], v[1] + rho * n[1]];
            let p = [c[0] - rho * t[0], c[1] - rho * t[1]];
            if stock.top_under_disk(p, rho) <= z + tol {
                continue;
            }
            let start = (p[1] - c[1]).atan2(p[0] - c[0]);
            let sweep = s * std::f64::consts::FRAC_PI_2;
            if !self.arc_clear(c, rho, start, sweep, z) {
                continue;
            }
            let lead = Curve::Arc {
                start: [p[0], p[1], z],
                end: [v[0], v[1], z],
                center: [c[0], c[1], z],
                plane: crate::motion::Plane::Xy,
                sweep_rad: sweep,
            };
            if let Some(l) = &self.limits
                && !l.allows_curve(&lead, true)?
            {
                continue;
            }
            // Along the arc's tangent at its start first (one smooth path);
            // where the part is in the way, along the pass's direction either
            // way (the descent then meets the arc at a corner, at depth, as
            // passes meet at their corners).
            for d in [[-n[0], -n[1]], t, [-t[0], -t[1]]] {
                if let Some((top, curves, air)) = self.arc_ramp(stock, p, d, z, cancel)? {
                    return Ok(Some(Entry::Ramp {
                        top,
                        curves,
                        air,
                        then: Some(lead),
                    }));
                }
            }
        }
        Ok(None)
    }

    /// The arc lead-out from the pass ending at `v` (height `z`) along the
    /// unit tangent `t`: a quarter circle of the tool's radius, tangent to the
    /// pass at `v`, flat at the pass's height, on a side where the disk at its
    /// end (where the retract lifts) touches no stock above the pass, proven
    /// clear of the part and allowed by the limits; `None` when neither side
    /// qualifies (the retract then lifts at `v`).
    fn arc_lead_out(
        &mut self,
        stock: &Stock,
        v: [f64; 2],
        t: [f64; 2],
        z: f64,
    ) -> Result<Option<Curve>, String> {
        let tol = self.op.tolerance_mm;
        let rho = self.radius;
        for s in [1., -1.] {
            let n = [-t[1] * s, t[0] * s];
            let c = [v[0] + rho * n[0], v[1] + rho * n[1]];
            let e = [c[0] + rho * t[0], c[1] + rho * t[1]];
            if stock.top_under_disk(e, rho) > z + tol {
                continue;
            }
            let start = (v[1] - c[1]).atan2(v[0] - c[0]);
            let sweep = s * std::f64::consts::FRAC_PI_2;
            if !self.arc_clear(c, rho, start, sweep, z) {
                continue;
            }
            let lead = Curve::Arc {
                start: [v[0], v[1], z],
                end: [e[0], e[1], z],
                center: [c[0], c[1], z],
                plane: crate::motion::Plane::Xy,
                sweep_rad: sweep,
            };
            if let Some(l) = &self.limits
                && !l.allows_curve(&lead, true)?
            {
                continue;
            }
            return Ok(Some(lead));
        }
        Ok(None)
    }

    /// The turn at the bottom of a vertical plunge onto `p` at height `z0`,
    /// where the move that follows leaves along the unit direction `d`: the
    /// plunge stops a tool radius higher, a radius back along `d`, and a
    /// quarter circle of the tool's radius in the vertical plane through `d`
    /// turns it to horizontal at `p`, tangent to the plunge above and to the
    /// move after (one arc in the XZ or YZ plane when `d` runs along X or Y,
    /// otherwise its chords at `ARC_MOVE_SAGITTA`, inside the circle).
    /// Taken where the disk over the whole turn's span at `z0` is clear of
    /// live stock above `z0` and of the part, the span lies inside the
    /// limits, and the plunge's new bottom is below the feed height; `None`
    /// otherwise (the plunge then comes straight down onto `p`).
    fn plunge_turn(
        &mut self,
        stock: &Stock,
        p: [f64; 2],
        d: [f64; 2],
        z0: f64,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Vec<Curve>>, String> {
        let tol = self.op.tolerance_mm;
        let rho = self.radius;
        if z0 + rho > self.heights.feed {
            return Ok(None);
        }
        let along_x = d[1].abs() <= 1e-12;
        let along_y = d[0].abs() <= 1e-12;
        let s = if along_x {
            [p[0] - rho * d[0].signum(), p[1]]
        } else if along_y {
            [p[0], p[1] - rho * d[1].signum()]
        } else {
            [p[0] - rho * d[0], p[1] - rho * d[1]]
        };
        if segment_stock_top(stock, s, p, rho, cancel)? > z0 + tol
            || self
                .index
                .max_height(s, p, rho, &mut self.seen, &mut self.stamp)
                > z0 + tol
            || !allows_segment(self.limits.as_ref(), s, p, z0, tol)?
        {
            return Ok(None);
        }
        let start = [s[0], s[1], z0 + rho];
        let end = [p[0], p[1], z0];
        let center = [p[0], p[1], z0 + rho];
        if along_x || along_y {
            let plane = if along_x {
                crate::motion::Plane::Xz
            } else {
                crate::motion::Plane::Yz
            };
            let [u, v, _] = plane.axes();
            let angle = |q: [f64; 3]| (q[v] - center[v]).atan2(q[u] - center[u]);
            let mut sweep = angle(end) - angle(start);
            if sweep > std::f64::consts::PI {
                sweep -= 2. * std::f64::consts::PI;
            } else if sweep < -std::f64::consts::PI {
                sweep += 2. * std::f64::consts::PI;
            }
            return Ok(Some(vec![Curve::Arc {
                start,
                end,
                center,
                plane,
                sweep_rad: sweep,
            }]));
        }
        let half = (1. - ARC_MOVE_SAGITTA / rho)
            .clamp(-1., 1.)
            .acos()
            .max(1e-9);
        let count = ((std::f64::consts::FRAC_PI_2 / (2. * half)).ceil().max(1.) as usize).min(4096);
        let at = |k: usize| {
            if k == 0 {
                return start;
            }
            if k == count {
                return end;
            }
            let theta = std::f64::consts::FRAC_PI_2 * k as f64 / count as f64;
            [
                p[0] - rho * theta.cos() * d[0],
                p[1] - rho * theta.cos() * d[1],
                z0 + rho - rho * theta.sin(),
            ]
        };
        Ok(Some(
            (0..count)
                .map(|k| Curve::Line {
                    start: at(k),
                    end: at(k + 1),
                })
                .collect(),
        ))
    }

    /// A ramp into stock as one smooth descent onto the pass start `p`
    /// (height `z`) along the pass's direction `d`, in the vertical plane
    /// through `d`, as in established CAM practice for a plunge with an arc
    /// lead-in: a vertical plunge through air, a quarter-circle-like arc of
    /// the tool's radius turning it to the declared ramp angle above the
    /// stock, a straight leg at that angle through the stock, and an arc of
    /// the tool's radius turning it to horizontal, tangent to the pass at `p`.
    /// The stock is met only on the leg and the last arc, never steeper than
    /// the declared angle. Taken where a ramp is declared, live stock stands
    /// above `z` under the span (at most the stock top over the whole span;
    /// the upper arc ends above it), the disk over the span is clear of the
    /// part at `z` (every point of the path is at or above `z`), the span lies
    /// inside the limits and the plunge's bottom is below the feed height;
    /// `None` otherwise (the zigzag ramp along the pass, `plunge_or_ramp`).
    /// At most `ARC_RAMP_REACH_RADII` radii long (about half a millimetre of
    /// stock at 3 degrees); deeper stock is ramped along the pass. Returns
    /// the plunge's bottom, the descent's moves, and how many of them (the
    /// upper arc) run through air.
    fn arc_ramp(
        &mut self,
        stock: &Stock,
        p: [f64; 2],
        d: [f64; 2],
        z: f64,
        cancel: &impl CancelSignal,
    ) -> Result<Option<(f64, Vec<Curve>, usize)>, String> {
        let tol = self.op.tolerance_mm;
        let rho = self.radius;
        let Some(ramp) = self.op.entry.as_ref().and_then(|e| e.ramp()) else {
            return Ok(None);
        };
        let n = d[0].hypot(d[1]);
        if n <= 0. {
            return Ok(None);
        }
        let d = [d[0] / n, d[1] / n];
        let along_x = d[1].abs() <= 1e-12;
        let along_y = d[0].abs() <= 1e-12;
        let d = if along_x {
            [d[0].signum(), 0.]
        } else if along_y {
            [0., d[1].signum()]
        } else {
            d
        };
        let (sin, cos, tan) = (
            ramp.angle_rad.sin(),
            ramp.angle_rad.cos(),
            ramp.angle_rad.tan(),
        );
        // Heights above z, distances along d (0 at p, negative behind it).
        let bottom = rho * (1. - cos);
        let mut stock_top = stock.top_under_disk(p, rho) - z;
        if stock_top <= tol {
            return Ok(None);
        }
        let mut shape = None;
        for _ in 0..4 {
            if cancel.is_cancelled() {
                return Err("flat-area planning cancelled".into());
            }
            // The upper arc ends a tolerance above the stock.
            let h_a = (stock_top + tol).max(bottom);
            let u_b = -rho * sin;
            let u_a = u_b - (h_a - bottom) / tan;
            let u_t = u_a - rho * (1. - sin);
            let h_t = h_a + rho * cos;
            let t = [p[0] + u_t * d[0], p[1] + u_t * d[1]];
            let top = segment_stock_top(stock, t, p, rho, cancel)? - z;
            if top <= stock_top + 1e-9 {
                shape = Some((h_a, u_b, u_a, u_t, h_t, t));
                break;
            }
            stock_top = top;
        }
        let Some((h_a, u_b, u_a, u_t, h_t, t)) = shape else {
            return Ok(None);
        };
        // No farther out than four radii, as far as a waterline side entry
        // is searched: deeper stock is ramped along the pass instead.
        if -u_t > ARC_RAMP_REACH_RADII * rho
            || z + h_t > self.heights.feed
            || self
                .index
                .max_height(t, p, rho, &mut self.seen, &mut self.stamp)
                > z + tol
            || !allows_segment(self.limits.as_ref(), t, p, z, tol)?
        {
            return Ok(None);
        }
        let at = |u: f64, h: f64| [p[0] + u * d[0], p[1] + u * d[1], z + h];
        let arc = |start: [f64; 3], end: [f64; 3], center: [f64; 3]| -> Vec<Curve> {
            if along_x || along_y {
                let plane = if along_x {
                    crate::motion::Plane::Xz
                } else {
                    crate::motion::Plane::Yz
                };
                let [u, v, _] = plane.axes();
                let angle = |q: [f64; 3]| (q[v] - center[v]).atan2(q[u] - center[u]);
                let mut sweep = angle(end) - angle(start);
                if sweep > std::f64::consts::PI {
                    sweep -= 2. * std::f64::consts::PI;
                } else if sweep < -std::f64::consts::PI {
                    sweep += 2. * std::f64::consts::PI;
                }
                return vec![Curve::Arc {
                    start,
                    end,
                    center,
                    plane,
                    sweep_rad: sweep,
                }];
            }
            // Chords at ARC_MOVE_SAGITTA, inside the circle, in (u, h).
            let to_uh = |q: [f64; 3]| ((q[0] - p[0]) * d[0] + (q[1] - p[1]) * d[1], q[2] - z);
            let (cu, ch) = to_uh(center);
            let (su, sh) = to_uh(start);
            let (eu, eh) = to_uh(end);
            let a0 = (sh - ch).atan2(su - cu);
            let mut sweep = (eh - ch).atan2(eu - cu) - a0;
            if sweep > std::f64::consts::PI {
                sweep -= 2. * std::f64::consts::PI;
            } else if sweep < -std::f64::consts::PI {
                sweep += 2. * std::f64::consts::PI;
            }
            let half = (1. - ARC_MOVE_SAGITTA / rho)
                .clamp(-1., 1.)
                .acos()
                .max(1e-9);
            let count = ((sweep.abs() / (2. * half)).ceil().max(1.) as usize).min(4096);
            let point = |k: usize| {
                if k == 0 {
                    return start;
                }
                if k == count {
                    return end;
                }
                let a = a0 + sweep * k as f64 / count as f64;
                at(cu + rho * a.cos(), ch + rho * a.sin())
            };
            (0..count)
                .map(|k| Curve::Line {
                    start: point(k),
                    end: point(k + 1),
                })
                .collect()
        };
        let mut curves = arc(at(u_t, h_t), at(u_a, h_a), at(u_t + rho, h_t));
        let air = curves.len();
        if u_a < u_b - 1e-9 {
            curves.push(Curve::Line {
                start: at(u_a, h_a),
                end: at(u_b, bottom),
            });
        }
        curves.extend(arc(at(u_b, bottom), at(0., 0.), at(0., rho)));
        Ok(Some((z + h_t, curves, air)))
    }

    /// Whether the disk at `p` comes down beside this chain's level's face: a
    /// radius (less the tolerance) outside the face region, so a vertical
    /// plunge at `p` never rests the disk on the flat the pass finishes. An
    /// empty face region (no flat at the level) leaves the point free.
    fn beside_face(&self, p: [f64; 2]) -> Result<bool, String> {
        let q = self.chains[self.chain].face.query(p)?;
        Ok(q.signed_clearance_mm
            .is_none_or(|s| s <= self.op.tolerance_mm - self.radius))
    }

    /// How the current chain is entered, decided against the live stock: from
    /// the side in air where the first pass runs along an open edge (the
    /// plunge point's disk touches no stock above the cut, stays beside the
    /// level's face, and the lead is proven and allowed), otherwise the
    /// declared rule (`plunge_or_ramp`).
    fn decide_entry(&mut self, stock: &Stock, cancel: &impl CancelSignal) -> Result<Entry, String> {
        let piece = &self.chains[self.chain].pieces[self.piece];
        let z = piece.start()[2];
        let tol = self.op.tolerance_mm;
        let first = piece.segs[0].a;
        let closed = piece.segs.last().is_some_and(|s| s.b == first);
        let mut best: Option<([f64; 2], [f64; 2])> = None;
        for (v, n) in &self.chains[self.chain].side {
            if cancel.is_cancelled() {
                return Err("flat-area planning cancelled".into());
            }
            if !closed && *v != first {
                continue;
            }
            // Rotating the closed first piece to `v` moves its end, and with
            // it the chain's link at depth to the next piece: `prepare`
            // proved that link from the piece's original end, the link from
            // `v` never was. Accept the candidate only when the new link
            // passes the same three tests as a link in `prepare`; otherwise
            // try the next vertex.
            if self.piece + 1 < self.chains[self.chain].pieces.len() && *v != first {
                let next = self.chains[self.chain].pieces[self.piece + 1].start();
                let length = (next[0] - v[0]).hypot(next[1] - v[1]);
                if length > 2. * self.op.stepover_mm
                    || self.index.max_height(
                        *v,
                        [next[0], next[1]],
                        self.radius,
                        &mut self.seen,
                        &mut self.stamp,
                    ) > z + tol
                    || !allows_segment(self.limits.as_ref(), *v, [next[0], next[1]], z, tol)?
                {
                    continue;
                }
            }
            // Outward, trying the shortest offset first.
            for l in [
                0.25 * self.radius,
                0.5 * self.radius,
                self.radius,
                1.5 * self.radius,
                2. * self.radius,
            ] {
                let p = [v[0] + l * n[0], v[1] + l * n[1]];
                if stock.top_under_disk(p, self.radius) > z + tol {
                    continue;
                }
                if !self.beside_face(p)? {
                    continue;
                }
                if self
                    .index
                    .max_height(p, *v, self.radius, &mut self.seen, &mut self.stamp)
                    > z + tol
                {
                    continue;
                }
                if !allows_segment(self.limits.as_ref(), p, *v, z, tol)? {
                    continue;
                }
                let nearer = best.is_none_or(|(bv, _)| {
                    (v[0] - self.position[0]).hypot(v[1] - self.position[1])
                        < (bv[0] - self.position[0]).hypot(bv[1] - self.position[1])
                });
                if nearer {
                    best = Some((*v, p));
                }
                break;
            }
        }
        if let Some((v, p)) = best {
            // The loop is cut from the entry vertex.
            if closed && (first[0] != v[0] || first[1] != v[1]) {
                let piece = &mut self.chains[self.chain].pieces[self.piece];
                rotate_piece(piece, v);
            }
            let t = self.chains[self.chain].pieces[self.piece].segs[0].tangent(false);
            if let Some(entry) = self.arc_lead_in(stock, v, t, z, cancel)? {
                self.report.arc_lead_ins += 1;
                return Ok(entry);
            }
            return Ok(Entry::Side { p });
        }
        let t = self.chains[self.chain].pieces[self.piece].segs[0].tangent(false);
        if let Some(entry) = self.arc_lead_in(stock, first, t, z, cancel)? {
            self.report.arc_lead_ins += 1;
            return Ok(entry);
        }
        // A pass start over the face with no stock above it is plunged when
        // the plunge turns to horizontal there (`plunge_turn`): the disk never
        // dwells on the floor it finishes.
        let a = self.chains[self.chain].pieces[self.piece].start();
        if stock.top_under_disk([a[0], a[1]], self.radius) <= z + tol
            && self
                .plunge_turn(stock, [a[0], a[1]], t, z, cancel)?
                .is_some()
        {
            return Ok(Entry::Plunge);
        }
        if let Some((top, curves, air)) = self.arc_ramp(stock, [a[0], a[1]], t, z, cancel)? {
            self.report.ramped_entries += 1;
            self.report.arc_ramps += 1;
            return Ok(Entry::Ramp {
                top,
                curves,
                air,
                then: None,
            });
        }
        if let Some(entry) = self.ramped_arc_lead_in(stock, first, t, z, cancel)? {
            self.report.ramped_entries += 1;
            self.report.arc_ramps += 1;
            self.report.arc_lead_ins += 1;
            return Ok(entry);
        }
        // A closed first pass may also be entered at the middle of one of its
        // straight strokes, where the arc lead-in fits there and not at its
        // corner: each half of the split stroke is part of the proven
        // stroke. Alone in its chain the pass starts and ends there; with a
        // pass linked after it (proven from its original end), it runs from
        // there to its original start and then once round in full, so it
        // still ends where the link was proven (the stretch before its
        // original start is cut twice). The strokes nearest the tool first,
        // then the shortest such lead-up.
        if closed {
            let segs = self.chains[self.chain].pieces[self.piece].segs.clone();
            let linked = self.piece + 1 < self.chains[self.chain].pieces.len();
            let mut order: Vec<usize> = (0..segs.len())
                .filter(|&i| {
                    segs[i].arc.is_none()
                        && crate::profiles::stroke_length(&segs[i]) >= 4. * self.radius
                })
                .collect();
            let middle = |i: usize| stroke_point(&segs[i], 0.5);
            let here = [self.position[0], self.position[1]];
            if linked {
                order.reverse();
            } else {
                order.sort_by(|&a, &b| {
                    let (ma, mb) = (middle(a), middle(b));
                    (ma[0] - here[0])
                        .hypot(ma[1] - here[1])
                        .total_cmp(&(mb[0] - here[0]).hypot(mb[1] - here[1]))
                        .then(a.cmp(&b))
                });
            }
            for i in order {
                if cancel.is_cancelled() {
                    return Err("flat-area planning cancelled".into());
                }
                let m = middle(i);
                let t = segs[i].tangent(false);
                if let Some(entry) = self.ramped_arc_lead_in(stock, m, t, z, cancel)? {
                    let mut split = vec![sub_stroke(&segs[i], 0.5, 1.)];
                    split.extend_from_slice(&segs[i + 1..]);
                    if linked {
                        split.extend_from_slice(&segs);
                    } else {
                        split.extend_from_slice(&segs[..i]);
                        split.push(sub_stroke(&segs[i], 0., 0.5));
                    }
                    self.chains[self.chain].pieces[self.piece].segs = split;
                    self.report.ramped_entries += 1;
                    self.report.arc_ramps += 1;
                    self.report.arc_lead_ins += 1;
                    return Ok(entry);
                }
            }
        }
        let piece = &self.chains[self.chain].pieces[self.piece];
        match self.plunge_or_ramp(piece, stock, cancel)? {
            None => Ok(Entry::Plunge),
            Some((top, curves)) => {
                self.report.ramped_entries += 1;
                Ok(Entry::Ramp {
                    top,
                    curves,
                    air: 0,
                    then: None,
                })
            }
        }
    }
    pub fn prefix_events(&self) -> Vec<Event> {
        vec![
            Event::WorkOffset {
                number: self.op.work_offset,
            },
            Event::ToolChange {
                tool: self.op.tool.clone(),
                number: self.op.tool_number,
                length_register: self.op.length_register,
                position_after_mm: self.initial,
            },
            Event::Spindle {
                rpm: self.op.spindle_rpm,
                direction: SpindleDirection::Cw,
            },
            Event::Coolant {
                mode: self.op.coolant,
            },
        ]
    }
    pub fn suffix_events(&self) -> Vec<Event> {
        vec![
            Event::Coolant { mode: Coolant::Off },
            Event::Spindle {
                rpm: 0.,
                direction: SpindleDirection::Cw,
            },
        ]
    }
    pub fn finish_report(&mut self, cancel: &impl CancelSignal) -> Result<(), String> {
        if cancel.is_cancelled() {
            return Err("flat-area report cancelled".into());
        }
        if !self.drained {
            return Err("drain committed flat-area motions before the final report".into());
        }
        self.report.geometric_residual_status = ResidualStatus::Unknown;
        Ok(())
    }
    pub fn next_motion_with_stock(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        if stock.spec() != &self.spec
            || stock.generation() == u64::MAX
            || self
                .expected_generation
                .is_some_and(|g| g != stock.generation())
        {
            return Err("commit preceding flat-area motion to matching stock".into());
        }
        loop {
            if cancel.is_cancelled() {
                return Err("flat-area planning cancelled".into());
            }
            let p = self.position;
            let mut ramp_feed = None;
            // The move's curve: a straight line from the current position
            // unless the phase carries one (a ramp leg or an arc of the pass).
            let (end, kind, next, carried) = match self.phase {
                Phase::Done => {
                    self.drained = true;
                    return Ok(None);
                }
                Phase::Lift => (
                    [p[0], p[1], self.heights.clearance],
                    MoveKind::Rapid,
                    if self.chains.is_empty() {
                        Phase::Done
                    } else {
                        Phase::Approach
                    },
                    None,
                ),
                Phase::Approach => {
                    // The entry is decided here, against the live stock: the
                    // rapid goes over the plunge point (a side entry), the
                    // ramp's start, or the pass's start.
                    let entry = self.decide_entry(stock, cancel)?;
                    // Where the plunge lands, the direction the move after it
                    // leaves in, and the phase after the plunge.
                    let landing = {
                        let piece = &self.chains[self.chain].pieces[self.piece];
                        let a = piece.start();
                        match &entry {
                            Entry::Side { p: q } => {
                                let (dx, dy) = (a[0] - q[0], a[1] - q[1]);
                                let n = dx.hypot(dy);
                                (n > 0.).then(|| (*q, [dx / n, dy / n], a[2], Phase::Lead))
                            }
                            Entry::Arc { lead, .. } => {
                                let (Curve::Arc {
                                    start,
                                    center,
                                    sweep_rad,
                                    ..
                                }
                                | Curve::Helix {
                                    start,
                                    center,
                                    sweep_rad,
                                    ..
                                }) = lead
                                else {
                                    unreachable!("an arc lead is an arc or a helix")
                                };
                                let sign = sweep_rad.signum();
                                let (rx, ry) = (start[0] - center[0], start[1] - center[1]);
                                let n = rx.hypot(ry);
                                Some((
                                    [start[0], start[1]],
                                    [-ry * sign / n, rx * sign / n],
                                    start[2],
                                    Phase::ArcIn,
                                ))
                            }
                            Entry::Plunge => {
                                Some(([a[0], a[1]], piece.segs[0].tangent(false), a[2], Phase::Cut))
                            }
                            Entry::Ramp { .. } => None,
                        }
                    };
                    self.turn.clear();
                    if let Some((q, d, z0, after)) = landing
                        && let Some(turn) = self.plunge_turn(stock, q, d, z0, cancel)?
                    {
                        self.report.rounded_plunges += 1;
                        self.turn = turn.into();
                        self.after_turn = after;
                    }
                    let over = match &entry {
                        Entry::Side { p: over } => *over,
                        Entry::Ramp { curves, .. } => {
                            let f = curves[0].start();
                            [f[0], f[1]]
                        }
                        Entry::Plunge => {
                            let a = self.chains[self.chain].pieces[self.piece].start();
                            [a[0], a[1]]
                        }
                        Entry::Arc { lead, .. } => {
                            let f = lead.start();
                            [f[0], f[1]]
                        }
                    };
                    let over = match self.turn.front() {
                        Some(first) => {
                            let f = first.start();
                            [f[0], f[1]]
                        }
                        None => over,
                    };
                    self.entry = Some(entry);
                    (
                        [over[0], over[1], p[2]],
                        MoveKind::Rapid,
                        Phase::Descend,
                        None,
                    )
                }
                Phase::Descend => (
                    [p[0], p[1], self.heights.feed],
                    MoveKind::Rapid,
                    Phase::Enter,
                    None,
                ),
                Phase::Enter => {
                    let a = self.chains[self.chain].pieces[self.piece].start();
                    let planned = match self.entry.take().expect("entry plan") {
                        Entry::Side { .. } => {
                            ([p[0], p[1], a[2]], MoveKind::Plunge, Phase::Lead, None)
                        }
                        Entry::Plunge => ([p[0], p[1], a[2]], MoveKind::Plunge, Phase::Cut, None),
                        Entry::Arc { lead, helical } => {
                            // Straight down in air to the lead's start.
                            let f = lead.start();
                            self.lead = Some((lead, helical));
                            ([p[0], p[1], f[2]], MoveKind::Plunge, Phase::ArcIn, None)
                        }
                        Entry::Ramp {
                            top,
                            curves,
                            air,
                            then,
                        } => {
                            self.ramp = curves.into();
                            self.ramp_air = air;
                            self.lead = then.map(|lead| (lead, false));
                            // Down through air to the stock's top over the
                            // pass, where the ramp's first curve begins.
                            (
                                [p[0], p[1], top.min(p[2]).max(a[2])],
                                MoveKind::Plunge,
                                if self.ramp.is_empty() {
                                    Phase::Cut
                                } else {
                                    Phase::Ramp
                                },
                                None,
                            )
                        }
                    };
                    // A rounded plunge stops at the turn's top; the turn
                    // lands where the plain plunge would have.
                    match self.turn.front() {
                        Some(first) => {
                            let f = first.start();
                            ([p[0], p[1], f[2]], MoveKind::Plunge, Phase::Turn, None)
                        }
                        None => planned,
                    }
                }
                Phase::Turn => {
                    let curve = self.turn.pop_front().expect("turn move");
                    let next = if self.turn.is_empty() {
                        self.after_turn
                    } else {
                        Phase::Turn
                    };
                    (curve.end(), MoveKind::Lead, next, Some(curve))
                }
                Phase::Lead => {
                    let a = self.chains[self.chain].pieces[self.piece].start();
                    ([a[0], a[1], a[2]], MoveKind::Lead, Phase::Cut, None)
                }
                Phase::ArcIn => {
                    let (lead, helical) = self.lead.take().expect("arc lead-in");
                    let kind = if helical {
                        // A helical lead descends as a ramp: at the ramp's feed.
                        ramp_feed = Some(
                            self.op
                                .entry
                                .as_ref()
                                .and_then(|e| e.feed_mm_min())
                                .unwrap_or(self.op.plunge_mm_min),
                        );
                        MoveKind::Plunge
                    } else {
                        MoveKind::Lead
                    };
                    (lead.end(), kind, Phase::Cut, Some(lead))
                }
                Phase::Ramp if self.ramp_air > 0 => {
                    // Through air above the stock: like a lead.
                    self.ramp_air -= 1;
                    let curve = self.ramp.pop_front().expect("ramp move");
                    (curve.end(), MoveKind::Lead, Phase::Ramp, Some(curve))
                }
                Phase::Ramp => {
                    let curve = self.ramp.pop_front().expect("ramp move");
                    ramp_feed = Some(
                        self.op
                            .entry
                            .as_ref()
                            .and_then(|e| e.feed_mm_min())
                            .unwrap_or(self.op.plunge_mm_min),
                    );
                    let end = curve.end();
                    (
                        end,
                        MoveKind::Plunge,
                        if !self.ramp.is_empty() {
                            Phase::Ramp
                        } else if self.lead.is_some() {
                            Phase::ArcIn
                        } else {
                            Phase::Cut
                        },
                        Some(curve),
                    )
                }
                Phase::Cut => {
                    let piece = &self.chains[self.chain].pieces[self.piece];
                    let seg = piece.segs[self.seg];
                    let end = [seg.b[0], seg.b[1], piece.z];
                    let curve = Some(seg.curve(piece.z));
                    self.seg += 1;
                    (
                        end,
                        MoveKind::Cut,
                        if self.seg < piece.segs.len() {
                            Phase::Cut
                        } else {
                            Phase::PieceEnd
                        },
                        curve,
                    )
                }
                Phase::PieceEnd => {
                    if self.piece + 1 < self.chains[self.chain].pieces.len() {
                        // A proven link at depth to the next pass.
                        let next = self.chains[self.chain].pieces[self.piece + 1].start();
                        self.piece += 1;
                        self.seg = 0;
                        (next, MoveKind::Link, Phase::Cut, None)
                    } else {
                        let piece = &self.chains[self.chain].pieces[self.piece];
                        let (v, z) = ([p[0], p[1]], piece.z);
                        let t = piece.segs[piece.segs.len() - 1].tangent(true);
                        match self.arc_lead_out(stock, v, t, z)? {
                            Some(lead) => {
                                self.report.arc_lead_outs += 1;
                                (lead.end(), MoveKind::Lead, Phase::Leave, Some(lead))
                            }
                            None => (
                                [p[0], p[1], self.heights.feed.max(p[2])],
                                MoveKind::Retract,
                                Phase::Rise,
                                None,
                            ),
                        }
                    }
                }
                Phase::Leave => (
                    [p[0], p[1], self.heights.feed.max(p[2])],
                    MoveKind::Retract,
                    Phase::Rise,
                    None,
                ),
                Phase::Rise => {
                    self.chain += 1;
                    self.piece = 0;
                    self.seg = 0;
                    self.report.runs_completed += 1;
                    let last = self.chain == self.chains.len();
                    let z = if last {
                        self.heights.clearance
                    } else {
                        self.heights.retract
                    };
                    (
                        [p[0], p[1], z.max(p[2])],
                        MoveKind::Rapid,
                        if last { Phase::Done } else { Phase::Approach },
                        None,
                    )
                }
            };
            self.phase = next;
            if end == p {
                continue;
            }
            let curve = carried.unwrap_or(Curve::Line { start: p, end });
            curve.prepare(self.op.tolerance_mm)?;
            let motion = Motion {
                id: self.next_id,
                operation: self.op.id.clone(),
                tool: self.op.tool.clone(),
                kind,
                curve,
                feed_mm_min: match kind {
                    MoveKind::Rapid => None,
                    MoveKind::Plunge => Some(ramp_feed.unwrap_or(self.op.plunge_mm_min)),
                    _ => Some(self.op.feed_mm_min),
                },
            };
            self.position = end;
            self.next_id += 1;
            self.expected_generation = Some(stock.generation() + 1);
            return Ok(Some(motion));
        }
    }
}

/// Highest live stock upper bound over the cells the cutter's disk touches
/// anywhere along a pass: sampled at no more than half a cell apart along
/// each chord, each sample's disk widened to cover the capsule between
/// samples.
fn piece_stock_top(
    piece: &Piece,
    stock: &Stock,
    r: f64,
    cancel: &impl CancelSignal,
) -> Result<f64, String> {
    let [wx, wy] = stock.cell_widths_mm();
    let step = 0.5 * wx.min(wy);
    let mut top = f64::MIN;
    for seg in &piece.segs {
        if cancel.is_cancelled() {
            return Err("planning cancelled while reading the stock".into());
        }
        match seg.arc {
            None => {
                top = top.max(segment_stock_top(stock, seg.a, seg.b, r, cancel)?);
            }
            Some((c, sweep)) => {
                // The disk the tool sweeps along a sub-arc is contained in the
                // capsule of radius r + sagitta around its chord, so the stock
                // is read over each chord with that wider radius.
                let rho = (seg.a[0] - c[0]).hypot(seg.a[1] - c[1]);
                let length = rho * sweep.abs();
                let n = (length / step).ceil().max(1.) as usize;
                let start = (seg.a[1] - c[1]).atan2(seg.a[0] - c[0]);
                let mut a = seg.a;
                for i in 1..=n {
                    let angle = start + sweep * (i as f64 / n as f64);
                    let b = [c[0] + rho * angle.cos(), c[1] + rho * angle.sin()];
                    let half = sweep.abs() / (2. * n as f64);
                    let sagitta = rho * (1. - half.cos());
                    top = top.max(segment_stock_top(stock, a, b, r + sagitta, cancel)?);
                    a = b;
                }
            }
        }
    }
    Ok(top)
}

/// Highest live stock upper bound over the cells a disk of radius `r`
/// touches anywhere along the segment from `a` to `b` (see `piece_stock_top`).
pub(crate) fn segment_stock_top(
    stock: &Stock,
    a: [f64; 2],
    b: [f64; 2],
    r: f64,
    cancel: &impl CancelSignal,
) -> Result<f64, String> {
    let [wx, wy] = stock.cell_widths_mm();
    let step = 0.5 * wx.min(wy);
    let length = (b[0] - a[0]).hypot(b[1] - a[1]);
    let n = (length / step).ceil().max(1.) as usize;
    if n > 1_000_000 {
        return Err("path too long to read the stock over".into());
    }
    let wide = r.hypot(0.5 * length / n as f64);
    let mut top = f64::MIN;
    for i in 0..=n {
        if i % 1024 == 0 && cancel.is_cancelled() {
            return Err("planning cancelled while reading the stock".into());
        }
        let f = i as f64 / n as f64;
        let xy = [a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])];
        top = top.max(stock.top_under_disk(xy, wide));
    }
    Ok(top)
}

struct LevelRegion {
    centres: crate::planar::PreparedRegion,
    face: std::sync::Arc<crate::planar::PreparedRegion>,
    shadow_wall_reserve_mm: f64,
}

/// The region of tool centres the passes on the flat level `z` run in: the
/// shadow of the part standing above `z - level_tolerance`, grown by the
/// overhang `r - stepover` (none at a stepover of a radius or more), minus
/// the shadow of the part standing above the level, grown by the radius plus
/// `radial` (the radial stock left on the walls) plus the shadows' stated
/// errors (the pattern of `waterline::flat_level_loops`): every position
/// whose disk would come within `radial` of the part above the level. The
/// "above" shadow's tip is picked so its `protected_from_z_mm` is strictly
/// above `z + level_tolerance`; nothing standing above the level leaves the
/// region alone. Every point of the region rests the disk on the level.
///
/// The overhang sets where the outermost pass runs past an open edge of the
/// face (one where the part drops away): its centre `r - stepover` outside
/// the edge, so its disk reaches `stepover` onto the face — the same width
/// every later pass takes — instead of rolling along the edge a radius
/// outside it and cutting nothing (the face is still covered: it lies inside
/// the region, and the passes cover the region). This is the usual facing
/// practice of running the tool part-way past the edge. Next to part standing
/// above the level the region still ends a radius plus `radial` (plus
/// errors) from it, so the outermost pass there runs along the wall, the
/// wall's radial stock left standing.
#[allow(clippy::too_many_arguments)]
fn level_region(
    target: &crate::roughing::PreparedRoughingTarget,
    z: f64,
    level_tolerance: f64,
    r: f64,
    radial: f64,
    stepover: f64,
    tolerance: f64,
    cancel: &impl CancelSignal,
) -> Result<Option<LevelRegion>, String> {
    use crate::planar::{Boolean, ConstructionOptions};
    let options = ConstructionOptions {
        tolerance_mm: tolerance,
    };
    let source_error = target.mesh().surface_error_mm;
    let mut work = 0usize;
    let below = target
        .shadow_for_cutter(z - level_tolerance, 0., r, &mut work, cancel)
        .map_err(|e| format!("flat-area shadow at Z{z}: {e}"))?;
    if below.region.components.is_empty() {
        return Ok(None);
    }
    // The shadow's numerical reserve, from its own report.
    let numerical = (z - level_tolerance) - below.report.protected_from_z_mm - source_error;
    let resting = below
        .region
        .prepare(options)
        .map_err(|e| format!("flat-area region at Z{z}: {e}"))?
        .offset_prepared(-(r - stepover).max(0.))
        .map_err(|e| format!("flat-area region at Z{z}: {e}"))?
        .into_parts()
        .1;
    let above_tip = z + level_tolerance + source_error + numerical + 16. * tolerance;
    let above = target
        .shadow_for_cutter(above_tip, 0., r, &mut work, cancel)
        .map_err(|e| format!("flat-area shadow at Z{z}: {e}"))?;
    let (region, face, shadow_wall_reserve_mm) = if above.region.components.is_empty() {
        let face = below
            .region
            .prepare(options)
            .map_err(|e| format!("flat-area face at Z{z}: {e}"))?;
        (resting, face, 0.)
    } else {
        // A radius plus the radial stock left on the walls off everything
        // standing above the level.
        let shadow_wall_reserve_mm = above.report.boundary_error_mm;
        let reach = r + radial + (shadow_wall_reserve_mm - source_error).max(0.);
        let above_region = above
            .region
            .prepare(options)
            .map_err(|e| format!("flat-area region at Z{z}: {e}"))?;
        let envelope = above_region.protective_outer_envelope();
        let blocked = envelope
            .as_ref()
            .unwrap_or(&above_region)
            .offset_prepared(-reach)
            .map_err(|e| format!("flat-area region at Z{z}: {e}"))?
            .into_parts()
            .1;
        let region = resting
            .boolean_prepared(&blocked, Boolean::Difference)
            .map_err(|e| format!("flat-area region at Z{z}: {e}"))?
            .into_parts()
            .1;
        // The level's face: the flat the pass finishes. The shadow standing
        // above is eroded by its stated boundary error plus the tolerance
        // before it is subtracted, so what is removed stays inside the part
        // that really stands above and the face kept is a superset of the
        // true flat: a point a radius outside this region is a radius
        // outside the true face too.
        let core = above_region
            .offset_prepared(above.report.boundary_error_mm + tolerance)
            .map_err(|e| format!("flat-area face at Z{z}: {e}"))?
            .into_parts()
            .1;
        let face = below
            .region
            .prepare(options)
            .map_err(|e| format!("flat-area face at Z{z}: {e}"))?
            .boolean_prepared(&core, Boolean::Difference)
            .map_err(|e| format!("flat-area face at Z{z}: {e}"))?
            .into_parts()
            .1;
        (region, face, shadow_wall_reserve_mm)
    };
    if region.definition().components.is_empty() {
        return Ok(None);
    }
    Ok(Some(LevelRegion {
        centres: region,
        face: std::sync::Arc::new(face),
        shadow_wall_reserve_mm,
    }))
}

fn cad_level_region(
    target: &crate::roughing::PreparedRoughingTarget,
    op: &FlatOperation,
    z: f64,
    r: f64,
    cad: &crate::planar::PlanarRegion,
    cancel: &impl CancelSignal,
) -> Result<LevelRegion, String> {
    let options = crate::planar::ConstructionOptions {
        tolerance_mm: op.tolerance_mm,
    };
    let cad = cad
        .prepare(options)
        .map_err(|e| format!("flat-area CAD floor at Z{z}: {e}"))?;
    let bounds = cad
        .bounds()
        .ok_or_else(|| format!("flat-area CAD floor at Z{z} is empty"))?;
    // Entries continue to use a conservative shadow of the actual mesh.
    // The caller's trim must never authorize a plunge onto the part.
    let shadow = target.shadow_for_cutter(z - op.level_tolerance_mm, 0., r, &mut 0, cancel)?;
    let face = shadow.region.prepare(options)?;
    let mesh = face
        .bounds()
        .ok_or_else(|| format!("flat-area CAD floor at Z{z} has no mesh shadow"))?;
    let error = cad.definition().boundary_error_mm + face.definition().boundary_error_mm;
    if bounds[0] < mesh[0] - error
        || bounds[1] < mesh[1] - error
        || bounds[2] > mesh[2] + error
        || bounds[3] > mesh[3] + error
    {
        return Err(format!(
            "flat-area CAD floor at Z{z} extends beyond the mesh shadow"
        ));
    }
    let reach =
        r + op.radial_stock_to_leave_mm + cad.definition().boundary_error_mm + op.tolerance_mm;
    let region = cad
        .offset_prepared(reach)
        .map_err(|e| format!("flat-area CAD floor offset at Z{z}: {e}"))?
        .into_parts()
        .1;
    Ok(LevelRegion {
        centres: region,
        face: std::sync::Arc::new(face),
        shadow_wall_reserve_mm: 0.,
    })
}

/// The offset loops of one connected component of a level's region: the
/// outer-boundary loops and the hole loops, each with its inward offset.
/// One offset loop: its offset depth, its closed strokes, and the stretches
/// of it that are cut when only part of it finishes floor no other loop
/// covers (`None`: the whole loop; see `trim_ring`).
type Ring = (
    f64,
    Vec<crate::pocket::Stroke>,
    Option<Vec<Vec<crate::pocket::Stroke>>>,
);

struct LoopGroup {
    outers: Vec<Ring>,
    holes: Vec<Ring>,
}
impl LoopGroup {
    fn distance(&self, p: [f64; 2]) -> f64 {
        let nearest = |rings: &[Ring]| {
            rings
                .iter()
                .flat_map(|(_, ring, _)| ring.iter())
                .map(|s| (s.a[0] - p[0]).hypot(s.a[1] - p[1]))
                .fold(f64::INFINITY, f64::min)
        };
        nearest(&self.outers).min(nearest(&self.holes))
    }
    /// The rings in cut order: outer loops before hole loops, offset
    /// ascending (outside in), nearest the tool first at the same offset;
    /// then, within the outer and within the hole loops, area by area
    /// (`by_area`) where that takes fewer entries, so each area is finished,
    /// outside in, before the tool crosses to the next.
    fn ordered(&self, tool: [f64; 2], gap: f64, link: f64) -> Vec<Ring> {
        let near = |ring: &[crate::pocket::Stroke]| {
            ring.iter()
                .map(|s| (s.a[0] - tool[0]).hypot(s.a[1] - tool[1]))
                .fold(f64::INFINITY, f64::min)
        };
        let sort = |rings: &[Ring]| -> Vec<Ring> {
            let mut order: Vec<usize> = (0..rings.len()).collect();
            order.sort_by(|&a, &b| {
                rings[a]
                    .0
                    .total_cmp(&rings[b].0)
                    .then(near(&rings[a].1).total_cmp(&near(&rings[b].1)))
                    .then(a.cmp(&b))
            });
            order.into_iter().map(|i| rings[i].clone()).collect()
        };
        let mut all = by_area(sort(&self.outers), tool, gap, link);
        let at = all
            .last()
            .and_then(|(_, ring, runs)| match runs {
                Some(runs) => runs.last().and_then(|r| r.last()).map(|s| s.b),
                None => ring.first().map(|s| s.a),
            })
            .unwrap_or(tool);
        all.extend(by_area(sort(&self.holes), at, gap, link));
        all
    }
}

/// `rings` (in cut order: offset ascending) reordered area by area when
/// that takes fewer entries. Each full ring and each kept stretch of a
/// trimmed ring is one item; an item waits for every shallower item within
/// `gap` of it (the cutter's diameter: every shallower item whose swept
/// disk overlaps its own), so every area is still cut outside in and no
/// item runs over floor a deeper one finished first; of the items whose
/// turn has come, the one nearest the tool is cut next. Where the offsets
/// split a level into separate areas (the strips either side of a boss),
/// the tool then finishes one area before crossing to the next instead of
/// visiting every area once per offset. The new order is taken only when
/// it needs fewer entries (moves to the next item longer than `link`, the
/// longest link at depth), or as many over less travel; otherwise the
/// rings keep their order. Cut order does not change what is cut: each item
/// was kept for the floor it alone reaches, and its proof and entry are
/// decided against the part and the live stock as before.
fn by_area(rings: Vec<Ring>, tool: [f64; 2], gap: f64, link: f64) -> Vec<Ring> {
    let mut items: Vec<Ring> = vec![];
    for (d, ring, runs) in &rings {
        match runs {
            None => items.push((*d, ring.clone(), None)),
            Some(runs) => {
                for run in runs.iter().filter(|r| !r.is_empty()) {
                    items.push((*d, ring.clone(), Some(vec![run.clone()])));
                }
            }
        }
    }
    let lines: Vec<Vec<crate::pocket::Stroke>> = items
        .iter()
        .map(|item| match &item.2 {
            None => item.1.clone(),
            Some(runs) => runs.concat(),
        })
        .collect();
    let points: Vec<Vec<[f64; 2]>> = lines
        .iter()
        .map(|run| {
            run.iter()
                .flat_map(|s| (0..4).map(move |k| stroke_point(s, k as f64 / 4.)))
                .chain(run.last().map(|s| s.b))
                .collect()
        })
        .collect();
    let distance = |p: [f64; 2], s: &crate::pocket::Stroke| {
        if s.arc.is_none() {
            dist2_to_segment(p, s.a, s.b).sqrt()
        } else {
            (0..=16)
                .map(|k| {
                    let x = stroke_point(s, k as f64 / 16.);
                    (x[0] - p[0]).hypot(x[1] - p[1])
                })
                .fold(f64::INFINITY, f64::min)
        }
    };
    let near = |i: usize, j: usize| {
        points[i]
            .iter()
            .any(|p| lines[j].iter().any(|s| distance(*p, s) <= gap))
            || points[j]
                .iter()
                .any(|p| lines[i].iter().any(|s| distance(*p, s) <= gap))
    };
    let n = items.len();
    let mut waits: Vec<Vec<usize>> = vec![vec![]; n];
    for j in 0..n {
        for i in 0..n {
            if items[i].0 < items[j].0 && near(i, j) {
                waits[j].push(i);
            }
        }
    }
    let hypot = |p: [f64; 2], q: [f64; 2]| (p[0] - q[0]).hypot(p[1] - q[1]);
    // Where an item is entered and left from `at`: a closed ring at its
    // vertex nearest the tool (`rotate_to_nearest`), a stretch from its first
    // stroke to its last.
    let ends = |strokes: &[crate::pocket::Stroke], closed: bool, at: [f64; 2]| {
        if closed {
            let v = strokes
                .iter()
                .map(|s| s.a)
                .min_by(|p, q| hypot(*p, at).total_cmp(&hypot(*q, at)))
                .unwrap_or(at);
            (v, v)
        } else {
            (
                strokes.first().map_or(at, |s| s.a),
                strokes.last().map_or(at, |s| s.b),
            )
        }
    };
    // (entries, travel) of a walk over (strokes, closed) pieces.
    let cost = |walk: &[(Vec<crate::pocket::Stroke>, bool)]| {
        let (mut entries, mut travel, mut at) = (0usize, 0f64, tool);
        for (strokes, closed) in walk {
            let (start, end) = ends(strokes, *closed, at);
            let jump = hypot(at, start);
            if jump > link {
                entries += 1;
            }
            travel += jump;
            at = end;
        }
        (entries, travel)
    };
    let mut done = vec![false; n];
    let mut order: Vec<usize> = vec![];
    let mut at = tool;
    while order.len() < n {
        let next = (0..n)
            .filter(|&j| !done[j] && waits[j].iter().all(|&i| done[i]))
            .min_by(|&a, &b| {
                let d = |k: usize| {
                    points[k]
                        .iter()
                        .map(|p| hypot(*p, at))
                        .fold(f64::INFINITY, f64::min)
                };
                d(a).total_cmp(&d(b)).then(a.cmp(&b))
            })
            .expect("the shallowest item left waits for nothing");
        done[next] = true;
        order.push(next);
        at = ends(&lines[next], items[next].2.is_none(), at).1;
    }
    let by_area: Vec<(Vec<crate::pocket::Stroke>, bool)> = order
        .iter()
        .map(|&i| (lines[i].clone(), items[i].2.is_none()))
        .collect();
    // The rings as they are: a trimmed ring's stretches in loop order from
    // the one starting nearest the tool (`open_runs_from_nearest`).
    let mut as_is: Vec<(Vec<crate::pocket::Stroke>, bool)> = vec![];
    let mut at = tool;
    for (_, ring, runs) in &rings {
        match runs {
            None => {
                at = ends(ring, true, at).1;
                as_is.push((ring.clone(), true));
            }
            Some(runs) => {
                let runs: Vec<&Vec<crate::pocket::Stroke>> =
                    runs.iter().filter(|r| !r.is_empty()).collect();
                let first = (0..runs.len())
                    .min_by(|&a, &b| hypot(runs[a][0].a, at).total_cmp(&hypot(runs[b][0].a, at)))
                    .unwrap_or(0);
                for k in 0..runs.len() {
                    let run = runs[(first + k) % runs.len()];
                    at = run.last().map_or(at, |s| s.b);
                    as_is.push((run.clone(), false));
                }
            }
        }
    }
    let (new, old) = (cost(&by_area), cost(&as_is));
    if new.0 < old.0 || (new.0 == old.0 && new.1 < old.1) {
        order.into_iter().map(|i| items[i].clone()).collect()
    } else {
        rings
    }
}

/// The offset loops of `region`, grouped per connected component: offsets
/// inward by 0, stepover, 2 stepover, ... while non-empty, then the core:
/// after the last non-empty offset `d`, while the region offset by
/// `d + r - 16 tolerances` is still non-empty, the loops at `d + stepover/2`
/// are added and the check repeats, so the disk covers every reachable point.
/// With a stepover above the radius, an interval whose next offset pinched
/// off (a corner between two boundaries of the region) also gets a ring at
/// half the stepover, so the disk covers every reachable point there too
/// (stepover up to the cutter diameter). Each loop keeps its arcs (cavalier
/// contours' bulges); outer boundaries end up clockwise and holes
/// counter-clockwise: the passes run outside in, so the stock still to cut
/// lies inward of each ring, on the right of travel — climb milling with a
/// clockwise spindle.
fn level_loops(
    region: &crate::planar::PreparedRegion,
    r: f64,
    stepover: f64,
    tolerance: f64,
    bridge: f64,
    cancel: &impl CancelSignal,
) -> Result<Vec<LoopGroup>, String> {
    use crate::planar::Boolean;
    let options = crate::planar::ConstructionOptions {
        tolerance_mm: tolerance,
    };
    let guard = 16. * tolerance;
    let mut groups = vec![];
    for component in &region.definition().components {
        if cancel.is_cancelled() {
            return Err("flat-area planning cancelled".into());
        }
        let single = crate::planar::PlanarRegion {
            version: 1,
            units: "mm".into(),
            name: "flat-level-region".into(),
            boundary_error_mm: region.definition().boundary_error_mm,
            components: vec![component.clone()],
            source: serde_json::Value::Null,
        };
        let prepared = single
            .prepare(options)
            .map_err(|e| format!("flat-area loop region: {e}"))?;
        // A disk inside this component cannot be wider than its bounding
        // box. Reserve the stated boundary error and the construction guard
        // for a near pinch, then refuse inconsistent nonempty erosions.
        let [xmin, ymin, xmax, ymax] = prepared.bounds().ok_or("flat-area loop bounds missing")?;
        let max_depth =
            0.5 * (xmax - xmin).min(ymax - ymin) + prepared.definition().boundary_error_mm + guard;
        // Loops as (depth, strokes, the depth the loops before them reach,
        // when they are only cut where they reach past it).
        type Pending = (f64, Vec<crate::pocket::Stroke>, Option<f64>);
        let mut outers: Vec<Pending> = vec![];
        let mut holes: Vec<Pending> = vec![];
        let mut vertices = 0usize;
        let mut collect = |eroded: &crate::planar::PreparedRegion,
                           d: f64,
                           need: Option<f64>|
         -> Result<(), String> {
            // A loop the kernel vouches no geometry for is not cut: shorter
            // than the guard (16 tolerances), or — the offset kernel
            // quantizes vertices to its construction grid (32 tolerances per
            // coordinate), so an offset that pinches to a point comes back as
            // a square of side 4 * guard, 16 times the guard — every extent
            // of the loop fits inside the kernel's own stated boundary error
            // for this offset (8.027e-4 mm against the square's 6.4e-4 on the
            // stepped test part's boss top at d = 6). Such a loop is the
            // kernel's representation of a point or line, not a feature. Its
            // points lie at depth d, within the stepover (at most the radius)
            // of the ring at d - stepover along their ray to the boundary, so
            // the disk covers them from that ring.
            let error = eroded.definition().boundary_error_mm;
            for c in &eroded.definition().components {
                let outer = boundary_strokes(&c.outer, false, tolerance)?;
                if !outer.is_empty() && !degenerate_loop(&outer, guard, error) {
                    outers.push((d, outer, need));
                }
                for hole in &c.holes {
                    let strokes = boundary_strokes(hole, true, tolerance)?;
                    if !strokes.is_empty() && !degenerate_loop(&strokes, guard, error) {
                        holes.push((d, strokes, need));
                    }
                }
                vertices += c.outer.vertices.len()
                    + c.holes.iter().map(|h| h.vertices.len()).sum::<usize>();
            }
            crate::resources::check_memory(vertices as u128 * 24, "flat-area offset loops")?;
            Ok(())
        };
        let mut d = 0.;
        let mut eroded = prepared
            .offset_prepared(d)
            .map_err(|e| format!("flat-area loop offset: {e}"))?
            .into_parts()
            .1;
        while !eroded.definition().components.is_empty() {
            if cancel.is_cancelled() {
                return Err("flat-area planning cancelled".into());
            }
            if d > max_depth {
                return Err(
                    "flat-area offset remains nonempty past the component's erosion limit".into(),
                );
            }
            // The rings so far (at 0, s, ..., d - s) already cover every point
            // of the region within d - s + r of its boundary: a point at depth
            // t >= d' lies t - d' from the boundary of the offset at d' (the
            // erosion's complement is the boundary's complement grown by d').
            // A ring at d is cut only while points deeper than that remain;
            // otherwise it would only run over floor the earlier rings have
            // finished (a narrow rim offset from both sides, a small top).
            // What is left must be more than the kernel's picture of a line
            // (`uncovered`).
            // Where such points remain, the ring is cut only where its disk
            // reaches them.
            let need = if d > 0. {
                if uncovered(&prepared, d - stepover + r, tolerance, guard)?.is_none() {
                    break;
                }
                Some(d - stepover + r)
            } else {
                None
            };
            collect(&eroded, d, need)?;
            let next_depth = d + stepover;
            if next_depth <= d {
                return Err("flat-area offset depth cannot advance by the stepover".into());
            }
            let next = prepared
                .offset_prepared(next_depth)
                .map_err(|e| format!("flat-area loop offset: {e}"))?
                .into_parts()
                .1;
            if stepover > r && !next.definition().components.is_empty() {
                // With the stepover above the radius, points deeper than
                // d + r but shallower than the next ring at d + stepover are
                // covered by that ring from below only where its offset
                // survives; where the offset pinched off (a corner between
                // two boundaries of the region) the surviving ring stretches
                // on either side can be more than 2r apart and such points
                // are out of reach of every ring. They are exactly the band
                // deeper than d + r - guard, outside the next offset, and
                // farther than r from the next offset's area (the disk
                // cannot reach them from there either): halve the gap with a
                // ring at d + stepover/2, which lies at or below their depth
                // (stepover <= 2r) and within stepover/2 <= r of them along
                // their ray to the boundary. At a stepover at or below the
                // radius no interval is deeper than d + r before the next
                // ring, so this never fires.
                let shallow = prepared
                    .offset_prepared(d + r - guard)
                    .map_err(|e| format!("flat-area loop offset: {e}"))?
                    .into_parts()
                    .1;
                let band = shallow
                    .boolean_prepared(&next, Boolean::Difference)
                    .map_err(|e| format!("flat-area loop offset: {e}"))?
                    .into_parts()
                    .1;
                if !band.definition().components.is_empty() {
                    let reach = next
                        .offset_prepared(-r)
                        .map_err(|e| format!("flat-area loop offset: {e}"))?
                        .into_parts()
                        .1;
                    let gap = band
                        .boolean_prepared(&reach, Boolean::Difference)
                        .map_err(|e| format!("flat-area loop offset: {e}"))?
                        .into_parts()
                        .1;
                    if !gap.definition().components.is_empty() {
                        let half = d + stepover / 2.;
                        let mid = prepared
                            .offset_prepared(half)
                            .map_err(|e| format!("flat-area loop offset: {e}"))?
                            .into_parts()
                            .1;
                        if !mid.definition().components.is_empty() {
                            collect(&mid, half, None)?;
                        }
                    }
                }
            }
            d = next_depth;
            eroded = next;
        }
        let mut core = d - stepover;
        loop {
            if cancel.is_cancelled() {
                return Err("flat-area planning cancelled".into());
            }
            if uncovered(&prepared, core + r, tolerance, guard)?.is_none() {
                break;
            }
            let step = core + stepover / 2.;
            if step <= core || step > max_depth {
                return Err(
                    "flat-area remaining floor cannot be resolved within the erosion limit".into(),
                );
            }
            let eroded = prepared
                .offset_prepared(step)
                .map_err(|e| format!("flat-area loop offset: {e}"))?
                .into_parts()
                .1;
            collect(&eroded, step, Some(core + r))?;
            core = step;
        }
        // Loops cut only where they finish floor nothing else covers: per
        // depth reached, the points deeper than it, less what each loop's
        // kept stretches sweep (outer loops first, as they are cut).
        let (mut done_outers, mut done_holes): (Vec<Ring>, Vec<Ring>) = (vec![], vec![]);
        let mut depths: Vec<f64> = outers.iter().chain(&holes).filter_map(|p| p.2).collect();
        depths.sort_by(f64::total_cmp);
        depths.dedup();
        let mut needs: Vec<(f64, Option<crate::planar::PreparedRegion>)> = vec![];
        for depth in depths {
            needs.push((depth, uncovered(&prepared, depth, tolerance, guard)?));
        }
        let outer_count = outers.len();
        for (index, (d, strokes, reach)) in outers.into_iter().chain(holes).enumerate() {
            if cancel.is_cancelled() {
                return Err("flat-area planning cancelled".into());
            }
            let runs = match reach {
                None => None,
                Some(reach) => {
                    let slot = needs
                        .iter()
                        .position(|(depth, _)| *depth == reach)
                        .expect("every reach has its region");
                    match &needs[slot].1 {
                        None => Some(vec![]),
                        Some(need) => trim_ring(&strokes, need, r, bridge, guard, cancel)?,
                    }
                }
            };
            if runs.as_ref().is_some_and(|runs| runs.is_empty()) {
                continue;
            }
            // What this loop's kept stretches sweep is finished for every later loop,
            // whatever depth that loop reaches past.
            let swept = match &runs {
                None => vec![strokes.clone()],
                Some(runs) => runs.clone(),
            };
            for (_, need) in needs.iter_mut() {
                if let Some(region) = need.take() {
                    *need = subtract_swept(region, &swept, r, tolerance, cancel)?;
                }
            }
            if index < outer_count {
                done_outers.push((d, strokes, runs));
            } else {
                done_holes.push((d, strokes, runs));
            }
        }
        groups.push(LoopGroup {
            outers: done_outers,
            holes: done_holes,
        });
    }
    Ok(groups)
}

/// The points of `prepared` deeper than `depth` (the reach of the rings cut
/// so far), or `None` when there are none: the offset by `depth` plus the
/// tolerance, without what is only the kernel's picture of a line
/// (`without_slivers`). Such a sliver is left where two rings' reach meets
/// exactly (a rim offset from both sides whose width is a whole number of
/// passes): the offset returns the meeting line as a strip of no width the
/// kernel can vouch for, which no ring needs to cover.
fn uncovered(
    prepared: &crate::planar::PreparedRegion,
    depth: f64,
    tolerance: f64,
    guard: f64,
) -> Result<Option<crate::planar::PreparedRegion>, String> {
    let deeper = prepared
        .offset_prepared(depth + tolerance)
        .map_err(|e| format!("flat-area loop offset: {e}"))?
        .into_parts()
        .1;
    without_slivers(deeper, guard)
}

/// `region` without its slivers — strips thinner than twice its own stated
/// boundary error plus `guard` (16 tolerances, as for degenerate loops),
/// which the kernel cannot tell from a line: the morphological opening
/// (eroded by that width `e`, grown back by it), or `None` when nothing is
/// left. The opening keeps every point that lies in a disk of radius `e`
/// inside the region; a point it drops at a corner tip lies within
/// `e / sin(angle / 2)` of what is kept (twice `e` down to 60 degrees), and
/// `trim_ring` keeps loops within that margin.
fn without_slivers(
    region: crate::planar::PreparedRegion,
    guard: f64,
) -> Result<Option<crate::planar::PreparedRegion>, String> {
    if region.definition().components.is_empty() {
        return Ok(None);
    }
    let e = region.definition().boundary_error_mm + guard;
    let core = region
        .offset_prepared(e)
        .map_err(|e| format!("flat-area loop offset: {e}"))?
        .into_parts()
        .1;
    if core.definition().components.is_empty() {
        return Ok(None);
    }
    let opened = core
        .offset_prepared(-e)
        .map_err(|e| format!("flat-area loop offset: {e}"))?
        .into_parts()
        .1;
    Ok((!opened.definition().components.is_empty()).then_some(opened))
}

/// Arcs chorded within the chord error in at most this many chords stay in
/// their run's chain (`subtract_swept`); longer ones are swept on their own.
const SWEPT_FINE_CHORDS: usize = 16;

/// Sagitta of the chords of an arc swept on its own (`subtract_swept`).
const SWEPT_ARC_SAGITTA_MM: f64 = 0.01;

/// `arc`'s chord points (after its start) for chords of sagitta at most
/// `sagitta`, and the largest sagitta they have.
fn arc_chords(seg: &crate::pocket::Stroke, sagitta: f64) -> (Vec<[f64; 2]>, f64) {
    let Some((c, sweep)) = seg.arc else {
        return (vec![seg.b], 0.);
    };
    let rho = (seg.a[0] - c[0]).hypot(seg.a[1] - c[1]);
    let n = arc_chord_count(rho, sweep, sagitta);
    let mut points: Vec<[f64; 2]> = (1..n)
        .map(|k| stroke_point(seg, k as f64 / n as f64))
        .collect();
    points.push(seg.b);
    (points, rho * (1. - (sweep.abs() / (2. * n as f64)).cos()))
}

fn arc_chord_count(rho: f64, sweep: f64, sagitta: f64) -> usize {
    let half = (1. - sagitta.min(rho) / rho)
        .clamp(-1., 1.)
        .acos()
        .max(1e-9);
    ((sweep.abs() / (2. * half)).ceil() as usize).clamp(1, 1 << 16)
}

/// `need` without what the disk of radius `r` sweeps along the strokes of
/// `runs` (`PreparedRegion::subtract_chain_sweeps`), never more than the
/// disk covers: each run of connected straight strokes and short arcs is
/// one chain of points, its arcs by chords within the region's chord error
/// (`tolerance / 8`) and its radius reduced by their largest sagitta; an
/// arc that takes more than `SWEPT_FINE_CHORDS` such chords is its own
/// chain of chords of sagitta at most `SWEPT_ARC_SAGITTA_MM`, its radius
/// reduced by theirs. One sweep per chain, not one capsule per stroke: a
/// ring along a finely faceted wall has thousands of strokes (#393). `None`
/// when nothing is left.
fn subtract_swept(
    need: crate::planar::PreparedRegion,
    runs: &[Vec<crate::pocket::Stroke>],
    r: f64,
    tolerance: f64,
    cancel: &impl CancelSignal,
) -> Result<Option<crate::planar::PreparedRegion>, String> {
    let fine = tolerance / 8.;
    let mut chains: Vec<(Vec<[f64; 2]>, f64)> = vec![];
    for run in runs {
        // The open chain: its points and the largest sagitta of its arcs.
        let mut chain: Option<(Vec<[f64; 2]>, f64)> = None;
        for seg in run {
            if cancel.is_cancelled() {
                return Err("flat-area planning cancelled".into());
            }
            let long = seg.arc.is_some_and(|(c, sweep)| {
                let rho = (seg.a[0] - c[0]).hypot(seg.a[1] - c[1]);
                arc_chord_count(rho, sweep, fine) > SWEPT_FINE_CHORDS
            });
            let broken = chain
                .as_ref()
                .is_some_and(|(p, _)| *p.last().unwrap() != seg.a);
            if (long || broken)
                && let Some((points, sag)) = chain.take()
            {
                chains.push((points, r - sag));
            }
            if long {
                let (mut points, sag) = arc_chords(seg, SWEPT_ARC_SAGITTA_MM);
                points.insert(0, seg.a);
                chains.push((points, r - sag));
                continue;
            }
            let (points, sag) = chain.get_or_insert_with(|| (vec![seg.a], 0.));
            let (more, s) = arc_chords(seg, fine);
            points.extend(more);
            *sag = sag.max(s);
        }
        if let Some((points, sag)) = chain {
            chains.push((points, r - sag));
        }
    }
    if chains.is_empty() {
        return Ok(Some(need));
    }
    let need = need
        .subtract_chain_sweeps(&chains, cancel)
        .map_err(|e| format!("flat-area swept region: {e}"))?
        .into_parts()
        .1;
    // The remaining floor may be narrow where two sweeps nearly meet.
    // Keep it for trim_ring: eroding and regrowing it here would discard
    // material that neither cutter sweep covered.
    Ok((!need.definition().components.is_empty()).then_some(need))
}

/// Whether a loop carries no geometry the kernel vouches for: its total
/// length is below `guard` (16 tolerances), or every extent of the loop fits
/// inside `error`, the kernel's own stated boundary error for the
/// construction (see `collect` in `level_loops`). Extents cover the stroke
/// endpoints and, for arcs, the cardinal directions the sweep crosses — an
/// arc bulges past its endpoints' bounding box.
fn degenerate_loop(strokes: &[crate::pocket::Stroke], guard: f64, error: f64) -> bool {
    let mut total = 0.;
    let mut min = [f64::INFINITY; 2];
    let mut max = [f64::NEG_INFINITY; 2];
    for s in strokes {
        total += crate::profiles::stroke_length(s);
        for p in [s.a, s.b] {
            for k in 0..2 {
                min[k] = min[k].min(p[k]);
                max[k] = max[k].max(p[k]);
            }
        }
        if let Some((c, sweep)) = s.arc {
            let rho = (s.a[0] - c[0]).hypot(s.a[1] - c[1]);
            let start = (s.a[1] - c[1]).atan2(s.a[0] - c[0]);
            let tau = 2. * std::f64::consts::PI;
            for k in 0..4 {
                let angle = k as f64 * std::f64::consts::FRAC_PI_2;
                let forward = if sweep >= 0. {
                    (angle - start).rem_euclid(tau)
                } else {
                    (start - angle).rem_euclid(tau)
                };
                if forward <= sweep.abs() {
                    let p = [c[0] + rho * angle.cos(), c[1] + rho * angle.sin()];
                    for k in 0..2 {
                        min[k] = min[k].min(p[k]);
                        max[k] = max[k].max(p[k]);
                    }
                }
            }
        }
    }
    total < guard || (max[0] - min[0]).max(max[1] - min[1]) <= error
}

/// The closed segment loop of a boundary: straight chords and arcs (cavalier
/// contours' bulges, as `Stroke::from_boundary`), oriented counter-clockwise
/// when `ccw` and clockwise otherwise. Degenerate segments (zero length or zero
/// sweep) carry no geometry and are dropped or straightened. Arcs a boolean
/// or offset kernel flattened to a chord polygon are refitted from their
/// chords (`refit_arcs`) so a ring along them is one proven move instead of
/// one move per chord.
fn boundary_strokes(
    boundary: &crate::planar::Boundary,
    ccw: bool,
    tolerance: f64,
) -> Result<Vec<crate::pocket::Stroke>, String> {
    let n = boundary.vertices.len();
    if n < 2 {
        return Err("flat-area loop boundary needs at least two vertices".into());
    }
    let segs: Vec<crate::pocket::Stroke> = crate::pocket::Stroke::from_boundary(boundary)
        .into_iter()
        .filter(|s| (s.b[0] - s.a[0]).hypot(s.b[1] - s.a[1]) > 1e-12)
        .map(|mut s| {
            if s.arc.is_some_and(|(_, sweep)| sweep.abs() < 1e-9) {
                s.arc = None;
            }
            s
        })
        .collect();
    if segs.len() < 2 {
        return Ok(vec![]);
    }
    // Orientation from the closed chord polygon (an inscribed polygon has the
    // arcs' orientation).
    let mut pts: Vec<[f64; 2]> = segs.iter().map(|s| s.a).collect();
    pts.push(segs[0].a);
    let flipped = (ccw && signed_area(&pts) < 0.) || (!ccw && signed_area(&pts) > 0.);
    let segs = if flipped {
        segs.into_iter()
            .rev()
            .map(crate::pocket::Stroke::reversed)
            .collect()
    } else {
        segs
    };
    Ok(absorb_stubs(refit_arcs(segs, tolerance / 4.)))
}

/// Absorb the straight stubs of a closed ring — a stroke shorter than
/// `ARC_MOVE_SAGITTA`, where a kernel snapped two vertices of one corner a
/// few microns apart — into a neighbouring straight stroke, which then runs
/// to (or from) the stub's far end: the ring moves by less than the stub's
/// length, within what the arcs' moves already stray, and is proven as it
/// is afterwards. A stub between two arcs stays.
fn absorb_stubs(mut segs: Vec<crate::pocket::Stroke>) -> Vec<crate::pocket::Stroke> {
    let mut k = 0;
    while segs.len() > 3 && k < segs.len() {
        let s = segs[k];
        let n = segs.len();
        if s.arc.is_some() || (s.b[0] - s.a[0]).hypot(s.b[1] - s.a[1]) >= ARC_MOVE_SAGITTA {
            k += 1;
            continue;
        }
        let (next, prev) = ((k + 1) % n, (k + n - 1) % n);
        if segs[next].arc.is_none() {
            segs[next].a = s.a;
        } else if segs[prev].arc.is_none() {
            segs[prev].b = s.b;
        } else {
            k += 1;
            continue;
        }
        segs.remove(k);
    }
    segs
}

/// Refit the arcs a boolean or offset kernel flattened to a chord polygon: a
/// maximal run of strokes whose vertices share one circle (within `fit`)
/// becomes one arc stroke again, so a ring along it is one proven move — or
/// the circumscribed polygon of the arc — instead of one move per flattened
/// chord. A run may mix straight chords with short arcs (the kernels also
/// return a flattened circle as alternating micro-arcs and micro-chords): an
/// arc joins a run through its end and mid points, which must sit on the
/// fitted circle like every vertex. Only runs fully on the circle, with
/// chords short against the radius, a steady one-sided turn and a sweep under
/// a full circle are accepted; anything else stays as it was, and every
/// emitted piece is still proven against the part before it is cut.
fn refit_arcs(segs: Vec<crate::pocket::Stroke>, fit: f64) -> Vec<crate::pocket::Stroke> {
    const MIN_RUN: usize = 4;
    if segs.len() < MIN_RUN {
        return segs;
    }
    // A stroke's points before its end: the start, and an arc's mid point.
    let inner = |s: &crate::pocket::Stroke| -> Vec<[f64; 2]> {
        match s.arc {
            None => vec![s.a],
            Some((c, sweep)) => {
                let rho = (s.a[0] - c[0]).hypot(s.a[1] - c[1]);
                let mid = (s.a[1] - c[1]).atan2(s.a[0] - c[0]) + sweep / 2.;
                vec![s.a, [c[0] + rho * mid.cos(), c[1] + rho * mid.sin()]]
            }
        }
    };
    let mut out: Vec<crate::pocket::Stroke> = Vec::with_capacity(segs.len());
    let mut i = 0;
    while i < segs.len() {
        // The longest run of strokes from `i` whose points share one circle:
        // extend while the whole run still fits, and keep the last fit (it was
        // checked against every point of the accepted run).
        let mut pts = inner(&segs[i]);
        let mut j = i;
        let mut fitted: Option<([f64; 2], f64)> = None;
        while j + 1 < segs.len() {
            let mut trial = pts.clone();
            trial.extend(inner(&segs[j + 1]));
            trial.push(segs[j + 1].b);
            let Some(f) = fit_circle_run(&trial, fit) else {
                break;
            };
            trial.pop();
            pts = trial;
            fitted = Some(f);
            j += 1;
        }
        if j - i + 1 >= MIN_RUN
            && let Some((centre, sweep)) = fitted
        {
            out.push(crate::pocket::Stroke {
                a: segs[i].a,
                b: segs[j].b,
                arc: Some((centre, sweep)),
            });
            i = j + 1;
            continue;
        }
        out.push(segs[i]);
        i += 1;
    }
    out
}

/// The circle through the first, middle and last of `pts`, when every point
/// sits on it within `fit`, the chords are short against the radius, the turn
/// is steady and one-sided and the sweep stays under a full circle: the
/// centre and the signed sweep from the first to the last point.
fn fit_circle_run(pts: &[[f64; 2]], fit: f64) -> Option<([f64; 2], f64)> {
    let n = pts.len();
    if n < 3 {
        return None;
    }
    let (p, q, r) = (pts[0], pts[n / 2], pts[n - 1]);
    let cross = (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
    let long = (q[0] - p[0]).hypot(q[1] - p[1]) * (r[0] - p[0]).hypot(r[1] - p[1]);
    // Collinear to the same 1e-9 rad a merged straight stretch may keep.
    if long <= 0. || cross.abs() <= 1e-9 * long {
        return None;
    }
    let (p2, q2, r2) = (
        p[0] * p[0] + p[1] * p[1],
        q[0] * q[0] + q[1] * q[1],
        r[0] * r[0] + r[1] * r[1],
    );
    let centre = [
        (p2 * (q[1] - r[1]) + q2 * (r[1] - p[1]) + r2 * (p[1] - q[1])) / (2. * cross),
        (p2 * (r[0] - q[0]) + q2 * (p[0] - r[0]) + r2 * (q[0] - p[0])) / (2. * cross),
    ];
    let radius = (p[0] - centre[0]).hypot(p[1] - centre[1]);
    if !radius.is_finite() || !(1e-9..=1e6).contains(&radius) {
        return None;
    }
    // Every vertex on the circle, every chord short against the radius.
    for s in pts {
        if ((s[0] - centre[0]).hypot(s[1] - centre[1]) - radius).abs() > fit {
            return None;
        }
    }
    for w in pts.windows(2) {
        if (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]) > 0.1 * radius {
            return None;
        }
    }
    // A steady one-sided turn: no chord turns against the run's direction.
    let mut side = 0i32;
    for w in pts.windows(3) {
        let t =
            (w[1][0] - w[0][0]) * (w[2][1] - w[1][1]) - (w[1][1] - w[0][1]) * (w[2][0] - w[1][0]);
        let sign = t.signum() as i32;
        if sign != 0 {
            if side == 0 {
                side = sign;
            } else if side != sign {
                return None;
            }
        }
    }
    // The signed sweep from the first to the last point around the centre.
    let mut sweep = 0.;
    let mut prev = [pts[0][0] - centre[0], pts[0][1] - centre[1]];
    for &s in &pts[1..] {
        let cur = [s[0] - centre[0], s[1] - centre[1]];
        sweep += (prev[0] * cur[1] - prev[1] * cur[0]).atan2(prev[0] * cur[0] + prev[1] * cur[1]);
        prev = cur;
    }
    if !sweep.is_finite() || sweep.abs() >= 2. * std::f64::consts::PI {
        return None;
    }
    Some((centre, sweep))
}

/// Signed area of a closed polyline (the first point repeated at the end).
fn signed_area(pts: &[[f64; 2]]) -> f64 {
    let mut area = 0.;
    for w in pts.windows(2) {
        area += w[0][0] * w[1][1] - w[1][0] * w[0][1];
    }
    area / 2.
}

/// Rotate a closed piece (the last point repeating the first) so it starts at
/// the vertex `v` (one of its own points) and ends there too.
fn rotate_piece(piece: &mut Piece, v: [f64; 2]) {
    let Some(i) = piece.segs.iter().position(|s| s.a == v) else {
        return;
    };
    let mut rotated: Vec<crate::pocket::Stroke> = piece.segs[i..].to_vec();
    rotated.extend_from_slice(&piece.segs[..i]);
    piece.segs = rotated;
}

/// Sampling step along a loop when trimming it to the floor it must finish.
const TRIM_STEP_MM: f64 = 0.25;

/// The stretches of the closed `ring` whose disk of radius `r` can reach
/// `need` (the points the loops before it leave uncovered), in loop order and
/// each in the loop's direction, or `None` when the whole loop is needed.
/// The loop is read in pieces of at most `TRIM_STEP_MM`; a piece is kept
/// when its middle lies within `r` plus half the piece's length of `need`
/// (the region's signed clearance, a distance, so no point of a dropped
/// piece comes within `r` of `need`). A dropped stretch no longer than
/// `bridge` between kept ones is kept as well: the pass is proven along it,
/// and cutting through is shorter than leaving and coming back. Keeping is
/// the conservative side: every point of `need` the whole loop would cover,
/// the kept stretches cover.
fn trim_ring(
    ring: &[crate::pocket::Stroke],
    need: &crate::planar::PreparedRegion,
    r: f64,
    bridge: f64,
    guard: f64,
    cancel: &impl CancelSignal,
) -> Result<Option<Vec<Vec<crate::pocket::Stroke>>>, String> {
    if need.definition().components.is_empty() {
        return Ok(Some(vec![]));
    }
    // The reach kept around `need`: the radius, plus twice the sliver width
    // `without_slivers` may have opened off a corner tip.
    let reach = r + 2. * (need.definition().boundary_error_mm + guard);
    // Pieces as (stroke index, t0, t1, length, kept).
    let mut pieces: Vec<(usize, f64, f64, f64, bool)> = vec![];
    for (i, seg) in ring.iter().enumerate() {
        if cancel.is_cancelled() {
            return Err("flat-area planning cancelled".into());
        }
        let length = crate::profiles::stroke_length(seg);
        let n = ((length / TRIM_STEP_MM).ceil() as usize).max(1);
        for k in 0..n {
            let (t0, t1) = (k as f64 / n as f64, (k + 1) as f64 / n as f64);
            let mid = stroke_point(seg, (t0 + t1) / 2.);
            let q = need.query(mid)?;
            let distance = (-q.signed_clearance_mm.unwrap_or(f64::NEG_INFINITY)).max(0.);
            pieces.push((
                i,
                t0,
                t1,
                length / n as f64,
                distance <= reach + length / n as f64 / 2.,
            ));
        }
    }
    let count = pieces.len();
    let Some(first_kept) = pieces.iter().position(|p| p.4) else {
        return Ok(Some(vec![]));
    };
    if pieces.iter().all(|p| p.4) {
        return Ok(None);
    }
    // Bridge short dropped stretches between kept ones, around the loop.
    let mut k = 0;
    while k < count {
        let at = (first_kept + k) % count;
        if pieces[at].4 {
            k += 1;
            continue;
        }
        let mut end = k;
        let mut gap = 0.;
        while end < count && !pieces[(first_kept + end) % count].4 {
            gap += pieces[(first_kept + end) % count].3;
            end += 1;
        }
        if gap <= bridge {
            for j in k..end {
                pieces[(first_kept + j) % count].4 = true;
            }
        }
        k = end;
    }
    if pieces.iter().all(|p| p.4) {
        return Ok(None);
    }
    // Start right after a dropped piece, so no stretch wraps the seam.
    let start = (0..count)
        .map(|k| (first_kept + k) % count)
        .find(|&at| pieces[at].4 && !pieces[(at + count - 1) % count].4)
        .unwrap_or(first_kept);
    let mut runs: Vec<Vec<crate::pocket::Stroke>> = vec![];
    let mut current: Vec<crate::pocket::Stroke> = vec![];
    let mut open: Option<(usize, f64, f64)> = None;
    let flush = |open: &mut Option<(usize, f64, f64)>, current: &mut Vec<crate::pocket::Stroke>| {
        if let Some((i, t0, t1)) = open.take() {
            current.push(sub_stroke(&ring[i], t0, t1));
        }
    };
    for k in 0..count {
        let (i, t0, t1, _, kept) = pieces[(start + k) % count];
        if !kept {
            flush(&mut open, &mut current);
            if !current.is_empty() {
                runs.push(std::mem::take(&mut current));
            }
            continue;
        }
        match open {
            Some((j, a, _)) if j == i => open = Some((j, a, t1)),
            _ => {
                flush(&mut open, &mut current);
                open = Some((i, t0, t1));
            }
        }
    }
    flush(&mut open, &mut current);
    if !current.is_empty() {
        runs.push(current);
    }
    Ok(Some(runs))
}

/// The point of a stroke at parameter `t` (0 at its start, 1 at its end).
fn stroke_point(seg: &crate::pocket::Stroke, t: f64) -> [f64; 2] {
    if t <= 0. {
        return seg.a;
    }
    if t >= 1. {
        return seg.b;
    }
    match seg.arc {
        None => [
            seg.a[0] + t * (seg.b[0] - seg.a[0]),
            seg.a[1] + t * (seg.b[1] - seg.a[1]),
        ],
        Some((c, sweep)) => {
            let rho = (seg.a[0] - c[0]).hypot(seg.a[1] - c[1]);
            let angle = (seg.a[1] - c[1]).atan2(seg.a[0] - c[0]) + sweep * t;
            [c[0] + rho * angle.cos(), c[1] + rho * angle.sin()]
        }
    }
}

/// The part of a stroke between parameters `t0` and `t1`, ending exactly on
/// the stroke's own endpoints where it reaches them.
fn sub_stroke(seg: &crate::pocket::Stroke, t0: f64, t1: f64) -> crate::pocket::Stroke {
    crate::pocket::Stroke {
        a: stroke_point(seg, t0),
        b: stroke_point(seg, t1),
        arc: seg.arc.map(|(c, sweep)| (c, sweep * (t1 - t0))),
    }
}

/// Open stretches of one loop, in loop order, starting from the stretch whose
/// start is nearest `tool`, as one stroke list (stretches that do not meet
/// are split into separate pieces when proven).
fn open_runs_from_nearest(
    runs: Vec<Vec<crate::pocket::Stroke>>,
    tool: [f64; 2],
) -> Vec<crate::pocket::Stroke> {
    let first = runs
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| {
            let d = |s: &[crate::pocket::Stroke]| (s[0].a[0] - tool[0]).hypot(s[0].a[1] - tool[1]);
            d(a).total_cmp(&d(b))
        })
        .map_or(0, |(i, _)| i);
    let n = runs.len();
    (0..n)
        .flat_map(|k| runs[(first + k) % n].iter().copied())
        .collect()
}

/// Rotate a closed ring of segments (the last ending on the first's start) so
/// it starts at the vertex nearest `tool` and ends there too.
fn rotate_to_nearest(
    ring: Vec<crate::pocket::Stroke>,
    tool: [f64; 2],
) -> Vec<crate::pocket::Stroke> {
    if ring.len() < 2 {
        return ring;
    }
    let (mut best, mut index) = (f64::INFINITY, 0usize);
    for (i, s) in ring.iter().enumerate() {
        let d = (s.a[0] - tool[0]).hypot(s.a[1] - tool[1]);
        if d < best {
            (best, index) = (d, i);
        }
    }
    if index == 0 {
        return ring;
    }
    let mut rotated: Vec<crate::pocket::Stroke> = ring[index..].to_vec();
    rotated.extend_from_slice(&ring[..index]);
    rotated
}

/// Prove every chord of `ring` at height `z` (the highest part point under
/// the swept disk of radius `r` at most `z + tolerance`), halving failures
/// down to `h / 16`, and keep only the stretches inside the limits. Adjacent
/// proven chords merge into pieces; what cannot be proven or allowed is
/// dropped, splitting the loop into open pieces.
#[allow(clippy::too_many_arguments)]
fn prove_ring(
    ring: &[crate::pocket::Stroke],
    z: f64,
    r: f64,
    index: &Index,
    seen: &mut [u32],
    stamp: &mut u32,
    h: f64,
    tolerance: f64,
    limits: Option<&crate::limits::PreparedLimits>,
    dropped: &mut f64,
    cancel: &impl CancelSignal,
) -> Result<Vec<Piece>, String> {
    let mut pieces: Vec<Piece> = vec![];

    // Append a segment, merging consecutive collinear chords of the same loop
    // (direction change below 1e-9 rad, both proven, one straight move): a
    // proven straight edge is not split into many chords.
    fn push(pieces: &mut Vec<Piece>, seg: crate::pocket::Stroke, z: f64) {
        if let Some(last) = pieces.last_mut()
            && last.z == z
            && last.segs.last().is_some_and(|s| s.b == seg.a)
        {
            let l = last.segs.last_mut().unwrap();
            if l.arc.is_none() && seg.arc.is_none() {
                let d0 = [l.b[0] - l.a[0], l.b[1] - l.a[1]];
                let d1 = [seg.b[0] - seg.a[0], seg.b[1] - seg.a[1]];
                let (l0, l1) = (d0[0].hypot(d0[1]), d1[0].hypot(d1[1]));
                if l0 > 0.
                    && l1 > 0.
                    && (d0[0] * d1[1] - d0[1] * d1[0]).abs() <= 1e-9 * l0 * l1
                    && d0[0] * d1[0] + d0[1] * d1[1] > 0.
                {
                    l.b = seg.b;
                    return;
                }
            }
            last.segs.push(seg);
            return;
        }
        pieces.push(Piece { z, segs: vec![seg] });
    }
    // Prove the straight stretch from `a` to `b` with the capsule radius `rc`:
    // kept where the highest part point under the capsule is not above the cut
    // height plus the tolerance (split by the limits into kept spans), halved
    // down to h / 16 otherwise, what cannot be proven dropped.
    fn prove_line(
        pieces: &mut Vec<Piece>,
        index: &Index,
        seen: &mut [u32],
        stamp: &mut u32,
        a: [f64; 2],
        b: [f64; 2],
        rc: f64,
        z: f64,
        h: f64,
        tolerance: f64,
        limits: Option<&crate::limits::PreparedLimits>,
        dropped: &mut f64,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        let mut stack = vec![(a, b, 0u32)];
        while let Some((a, b, depth)) = stack.pop() {
            if cancel.is_cancelled() {
                return Err("flat-area planning cancelled".into());
            }
            let length = (b[0] - a[0]).hypot(b[1] - a[1]);
            if index.max_height(a, b, rc, seen, stamp) <= z + tolerance {
                match limits {
                    None => push(pieces, crate::pocket::Stroke::line(a, b), z),
                    Some(limits) => {
                        // Only the stretches proven inside the limits.
                        let line = crate::motion::Curve::Line {
                            start: [a[0], a[1], z],
                            end: [b[0], b[1], z],
                        };
                        let mut kept = 0.;
                        for [t0, t1] in limits.allowed_spans(&line, true)? {
                            let p = [a[0] + t0 * (b[0] - a[0]), a[1] + t0 * (b[1] - a[1])];
                            let q = [a[0] + t1 * (b[0] - a[0]), a[1] + t1 * (b[1] - a[1])];
                            if (q[0] - p[0]).hypot(q[1] - p[1]) > tolerance {
                                push(pieces, crate::pocket::Stroke::line(p, q), z);
                                kept += (t1 - t0) * length;
                            }
                        }
                        *dropped += length - kept;
                    }
                }
                continue;
            }
            if length <= h / 16. || depth >= 32 {
                *dropped += length;
                continue;
            }
            let m = [(a[0] + b[0]) / 2., (a[1] + b[1]) / 2.];
            stack.push((m, b, depth + 1));
            stack.push((a, m, depth + 1));
        }
        Ok(())
    }
    for seg in ring {
        if cancel.is_cancelled() {
            return Err("flat-area planning cancelled".into());
        }
        let Some((c, sweep)) = seg.arc else {
            prove_line(
                &mut pieces,
                index,
                seen,
                stamp,
                seg.a,
                seg.b,
                r,
                z,
                h,
                tolerance,
                limits,
                dropped,
                cancel,
            )?;
            continue;
        };
        // An arc of the offset loop: covered by its chords (sagitta at most
        // the tolerance, refined while chords fail), each proven with the
        // capsule of radius r + sagitta, which contains the arc's swept disk.
        // Every chord proven, the arc is one move; otherwise the proven
        // chords stand in for it.
        let rho = (seg.a[0] - c[0]).hypot(seg.a[1] - c[1]);
        let start = (seg.a[1] - c[1]).atan2(seg.a[0] - c[0]);
        // Chords no shorter than the halving floor, so the refinement (and the
        // fallback's chords) stay above what a straight chord could prove.
        let count_cap = ((sweep.abs() * rho / (h / 16.)).ceil() as usize)
            .max(1)
            .next_power_of_two();
        let mut count = {
            let half = (1. - tolerance / rho).clamp(-1., 1.).acos().max(1e-9);
            ((sweep.abs() / (2. * half)).ceil().max(1.) as usize).min(count_cap)
        };
        let chord = |i: usize, count: usize| {
            let t0 = i as f64 / count as f64;
            let t1 = (i + 1) as f64 / count as f64;
            let p = [
                c[0] + rho * (start + sweep * t0).cos(),
                c[1] + rho * (start + sweep * t0).sin(),
            ];
            let q = [
                c[0] + rho * (start + sweep * t1).cos(),
                c[1] + rho * (start + sweep * t1).sin(),
            ];
            let sagitta = rho * (1. - (sweep.abs() / (2. * count as f64)).cos());
            (p, q, sagitta)
        };
        let mut proven = false;
        while count <= count_cap {
            if cancel.is_cancelled() {
                return Err("flat-area planning cancelled".into());
            }
            let all_clear = (0..count).all(|i| {
                let (p, q, sagitta) = chord(i, count);
                index.max_height(p, q, r + sagitta, seen, stamp) <= z + tolerance
            });
            if all_clear {
                proven = true;
                break;
            }
            count *= 2;
        }
        // The whole arc must also sit inside the limits; otherwise its chords
        // are checked against them one by one below.
        let allowed = limits.map_or(Ok(true), |l| l.allows_curve(&seg.curve(z), true))?;
        if proven && allowed {
            // One proven arc piece, split so every move's chord stays within
            // `ARC_MOVE_SAGITTA` of the arc; the proof above covers the whole
            // arc, every piece of it included.
            for sub in split_arc(seg, ARC_MOVE_SAGITTA) {
                push(&mut pieces, sub, z);
            }
            continue;
        }
        // Before the chord fallback: the arc's circumscribed polygon. Its
        // `sides + 1` edges are the tangents at start + k*sweep/sides
        // (k = 0..sides), so the path runs a -> the intersections of
        // consecutive tangents (at the mid angles, radius
        // rho / cos(sweep/(2*sides))) -> b, touching the circle at the
        // tangent points and ending on the arc's own endpoints. For an arc
        // whose centre is on the obstacle side (a convex corner of the
        // blocked region, as round the grown shadow of a boss standing
        // above) every edge stays on or outside the circle, away from the
        // centre, so the disk along an edge keeps at least the clearance the
        // planned arc's own disk has, and the edge proves as a straight cut
        // at radius r with no sagitta: the move IS the segment. The polygon
        // strays from the planned ring at most
        // rho * (1/cos(sweep/(2*sides)) - 1) at its vertices, held to
        // `ARC_MOVE_SAGITTA` through `sides`. An edge that fails the proof
        // (the centre on the region side, the polygon bulging toward the
        // obstacle) sends the arc to the chords below.
        let sides = {
            let step = (rho / (rho + ARC_MOVE_SAGITTA)).acos().max(1e-9);
            ((sweep.abs() / (2. * step)).ceil().max(1.) as usize).max(1)
        };
        let reach = rho / (sweep.abs() / (2. * sides as f64)).cos();
        let mut points = Vec::with_capacity(sides + 2);
        points.push(seg.a);
        for k in 0..sides {
            let mid = start + sweep * (k as f64 + 0.5) / sides as f64;
            points.push([c[0] + reach * mid.cos(), c[1] + reach * mid.sin()]);
        }
        points.push(seg.b);
        let mut circumscribed = true;
        for pair in points.windows(2) {
            if cancel.is_cancelled() {
                return Err("flat-area planning cancelled".into());
            }
            if index.max_height(pair[0], pair[1], r, seen, stamp) > z + tolerance
                || !allows_segment(limits, pair[0], pair[1], z, tolerance)?
            {
                circumscribed = false;
                break;
            }
        }
        if circumscribed {
            for pair in points.windows(2) {
                push(
                    &mut pieces,
                    crate::pocket::Stroke::line(pair[0], pair[1]),
                    z,
                );
            }
            continue;
        }
        // The arc's chords at the tolerance's sagitta, each proven with the
        // arc's capsule radius; failures are halved like straight chords.
        let half = (1. - tolerance / rho).clamp(-1., 1.).acos().max(1e-9);
        let count = ((sweep.abs() / (2. * half)).ceil().max(1.) as usize).min(count_cap);
        let sagitta = rho * (1. - (sweep.abs() / (2. * count as f64)).cos());
        for i in 0..count {
            let (p, q, _) = chord(i, count);
            prove_line(
                &mut pieces,
                index,
                seen,
                stamp,
                p,
                q,
                r + sagitta,
                z,
                h,
                tolerance,
                limits,
                dropped,
                cancel,
            )?;
        }
    }
    Ok(pieces)
}

/// Side-entry candidates of a piece: its segment starts, at most one per 1 mm
/// of the piece, each with the outward normal (left of travel: away from the
/// region on a clockwise outer loop, into the hole on a counter-clockwise
/// hole loop). On an arc the tangent at its start stands in for the chord.
fn side_candidates(piece: &Piece) -> Vec<([f64; 2], [f64; 2])> {
    let mut side = vec![];
    let mut taken: Option<[f64; 2]> = None;
    for seg in &piece.segs {
        let v = seg.a;
        if taken.is_some_and(|p| (v[0] - p[0]).hypot(v[1] - p[1]) < 1.) {
            continue;
        }
        let d = seg.tangent(false);
        if !(d[0]).is_finite() || d[0].hypot(d[1]) <= 0. {
            continue;
        }
        side.push((v, [-d[1], d[0]]));
        taken = Some(v);
    }
    side
}

/// Whether a straight move at `z` lies entirely inside the limits (always
/// true without limits, and for a degenerate move between proven endpoints).
fn allows_segment(
    limits: Option<&crate::limits::PreparedLimits>,
    a: [f64; 2],
    b: [f64; 2],
    z: f64,
    tolerance: f64,
) -> Result<bool, String> {
    let Some(limits) = limits else {
        return Ok(true);
    };
    if (b[0] - a[0]).hypot(b[1] - a[1]) <= tolerance {
        return Ok(true);
    }
    let line = crate::motion::Curve::Line {
        start: [a[0], a[1], z],
        end: [b[0], b[1], z],
    };
    Ok(limits.allowed_spans(&line, true)?.as_slice() == [[0., 1.]])
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The exact distance from `p` to a stroke (line or arc): the oracle
    /// for `subtract_swept`, sharing none of its chording.
    fn stroke_distance(p: [f64; 2], s: &crate::pocket::Stroke) -> f64 {
        let ends = (p[0] - s.a[0])
            .hypot(p[1] - s.a[1])
            .min((p[0] - s.b[0]).hypot(p[1] - s.b[1]));
        match s.arc {
            None => crate::geometry::point_segment_distance_sq(p, s.a, s.b).sqrt(),
            Some((c, sweep)) => {
                let rho = (s.a[0] - c[0]).hypot(s.a[1] - c[1]);
                let start = (s.a[1] - c[1]).atan2(s.a[0] - c[0]);
                let at = (p[1] - c[1]).atan2(p[0] - c[0]);
                let tau = std::f64::consts::TAU;
                let along = if sweep >= 0. {
                    (at - start).rem_euclid(tau)
                } else {
                    (start - at).rem_euclid(tau)
                };
                if along <= sweep.abs() {
                    ((p[0] - c[0]).hypot(p[1] - c[1]) - rho).abs()
                } else {
                    ends
                }
            }
        }
    }

    #[test]
    fn swept_rings_subtract_no_more_than_the_disk_covers() {
        use crate::pocket::Stroke;
        let arc = |a: [f64; 2], c: [f64; 2], sweep: f64| {
            let (s, k) = (sweep.sin(), sweep.cos());
            let d = [a[0] - c[0], a[1] - c[1]];
            Stroke {
                a,
                b: [c[0] + d[0] * k - d[1] * s, c[1] + d[0] * s + d[1] * k],
                arc: Some((c, sweep)),
            }
        };
        // A line, a half circle (chorded on its own), a line, a micro-arc
        // (chorded in the chain), and a separate run.
        let half = arc([10., 2.], [10., 8.], std::f64::consts::PI);
        let micro = arc([4., 14.], [4., 20.], -0.01);
        let run = vec![
            Stroke::line([2., 2.], [10., 2.]),
            half,
            Stroke::line(half.b, [4., 14.]),
            micro,
            Stroke::line(micro.b, [2., 18.]),
        ];
        let other = vec![Stroke::line([18., 1.], [18., 19.])];
        let runs = vec![run, other];
        let floor = crate::planar::PlanarRegion {
            version: 1,
            units: "mm".into(),
            name: "floor".into(),
            boundary_error_mm: 0.,
            components: vec![crate::planar::Component {
                outer: crate::planar::Boundary {
                    vertices: vec![[0., 0., 0.], [20., 0., 0.], [20., 20., 0.], [0., 20., 0.]],
                },
                holes: vec![],
            }],
            source: serde_json::Value::Null,
        }
        .prepare(crate::planar::ConstructionOptions::default())
        .unwrap();
        let (r, tolerance) = (1.5, 1e-5);
        let cancel = std::sync::atomic::AtomicBool::new(false);
        let left = subtract_swept(floor, &runs, r, tolerance, &cancel)
            .unwrap()
            .unwrap();
        let (mut kept, mut removed) = (0, 0);
        for i in 0..=100 {
            for j in 0..=100 {
                let p = [0.1 + i as f64 * 0.198, 0.1 + j as f64 * 0.198];
                let beyond = runs
                    .iter()
                    .flatten()
                    .map(|s| stroke_distance(p, s) - r)
                    .fold(f64::INFINITY, f64::min);
                let inside = left.query(p).unwrap().signed_clearance_mm.unwrap();
                if beyond > 2e-5 {
                    assert!(inside > 0., "{p:?} {beyond}: removed beyond the disk");
                    kept += 1;
                } else if beyond < -(2. * SWEPT_ARC_SAGITTA_MM + 2e-5) {
                    // Outside a long arc its chords run up to one sagitta
                    // inside it and sweep a radius one sagitta smaller.
                    assert!(inside < 0., "{p:?} {beyond}: swept but kept");
                    removed += 1;
                }
            }
        }
        assert!(kept > 4000 && removed > 1500, "{kept} {removed}");
    }

    #[test]
    fn cad_floor_retains_the_radial_reserve_and_rejects_an_outside_trim() {
        let op: FlatOperation = serde_json::from_value(serde_json::json!({
            "id":"flat", "tool":"mill", "floor_z_mm":0., "stepover_mm":1., "clearance_z_mm":8.,
            "feed_mm_min":300., "plunge_mm_min":100., "tolerance_mm":0.00001, "spindle_rpm":6000.,
            "tool_number":1, "length_register":1, "work_offset":1, "coolant":"flood",
            "radial_stock_to_leave_mm":0.2, "surface":{
                "version":1, "units":"mm", "surface_error_mm":0., "source":null,
                "triangles":[[[0.,0.,4.],[10.,0.,4.],[10.,10.,4.]],[[0.,0.,4.],[10.,10.,4.],[0.,10.,4.]]]
            }
        })).unwrap();
        let cancel = std::sync::atomic::AtomicBool::new(false);
        let target = crate::roughing::PreparedRoughingTarget::prepare(
            op.surface.clone(),
            op.tolerance_mm,
            &cancel,
        )
        .unwrap();
        let circle = crate::planar::PlanarRegion::circle("floor", [5., 5.], 2.).unwrap();
        let centres = cad_level_region(&target, &op, 4., 1., &circle, &cancel)
            .unwrap()
            .centres;
        let reach = 2. - 1. - op.radial_stock_to_leave_mm - op.tolerance_mm;
        assert!((centres.summary().area_mm2 - std::f64::consts::PI * reach.powi(2)).abs() < 1e-8);
        let outside = crate::planar::PlanarRegion::circle("outside", [25., 5.], 2.).unwrap();
        assert!(
            cad_level_region(&target, &op, 4., 1., &outside, &cancel)
                .err()
                .unwrap()
                .contains("beyond the mesh shadow")
        );
    }

    fn tri(p: [Vec3; 3]) -> Tri {
        Tri {
            p,
            min: [
                p[0][0].min(p[1][0]).min(p[2][0]),
                p[0][1].min(p[1][1]).min(p[2][1]),
            ],
            max: [
                p[0][0].max(p[1][0]).max(p[2][0]),
                p[0][1].max(p[1][1]).max(p[2][1]),
                p[0][2].max(p[1][2]).max(p[2][2]),
            ],
        }
    }

    #[test]
    fn capsule_maximum_matches_dense_sampling() {
        // A sloped triangle partly under a capsule.
        let t = tri([[0., 0., 0.], [10., 0., 5.], [0., 10., 2.]]);
        let (p, q, r) = ([2., 2.], [6., 3.], 1.5);
        let exact = t.max_in_capsule(p, q, r, 1e-12).unwrap();
        let mut sampled = f64::NEG_INFINITY;
        for i in 0..=400 {
            for j in 0..=400 {
                let x = [i as f64 * 0.025, j as f64 * 0.025];
                if dist2_to_segment(x, p, q) <= r * r
                    && let Some(w) = t.bary(x)
                    && w.iter().all(|w| *w >= 0.)
                {
                    sampled = sampled.max(w[0] * 0. + w[1] * 5. + w[2] * 2.);
                }
            }
        }
        assert!(
            exact >= sampled - 1e-12 && exact - sampled < 0.02,
            "{exact} {sampled}"
        );
        // Vertical triangle: the maximum lies on an edge.
        let wall = tri([[5., 0., 0.], [5., 10., 0.], [5., 5., 4.]]);
        let top = wall.max_in_capsule([4., 5.], [4., 5.], 1.2, 1e-12).unwrap();
        assert!((top - 4.).abs() < 1e-9, "{top}");
        assert!(wall.max_in_capsule([0., 5.], [1., 5.], 1., 1e-12).is_none());
    }

    #[test]
    fn bull_chord_clearance_agrees_with_dense_sampling() {
        let corners = [[0., 0., 0.], [10., 0., 5.], [0., 10., 2.]];
        let mesh = crate::surface::SurfaceMesh {
            version: 1,
            units: "mm".into(),
            triangles: vec![corners].into(),
            surface_error_mm: 0.,
            source: serde_json::Value::Null,
        };
        let never = std::sync::atomic::AtomicBool::new(false);
        let index = Index::new(&mesh, -10., 2., &never).unwrap();
        let mut seen = vec![0; index.len()];
        let mut stamp = 0;
        let t = tri(corners);
        for (r, rc) in [(2., 0.), (2., 0.7), (2., 2.)] {
            // A chord over the facet (where the contact is linear), its tips
            // at the exact contacts plus a margin: clear above, refused below.
            let (pa, pb) = ([1., 2.], [4., 2.5]);
            let contact = |p: [f64; 2]| t.bull_contact(p, r, rc).unwrap();
            for (margin, clear) in [(0.01, true), (-0.01, false)] {
                let mut a = [pa[0], pa[1], contact(pa) + margin];
                let mut b = [pb[0], pb[1], contact(pb) + margin];
                // Keep both ends above: only the middle may dip.
                if !clear {
                    a[2] += 0.02;
                    b[2] += 0.02;
                }
                let dense = (0..=2000).all(|k| {
                    let s = k as f64 / 2000.;
                    let p = [a[0] + s * (b[0] - a[0]), a[1] + s * (b[1] - a[1])];
                    contact(p) <= a[2] + s * (b[2] - a[2])
                });
                let proven = index.bull_chord_clear(a, b, r, rc, 0., &mut seen, &mut stamp);
                // Never proven when a sample shows contact below the chord.
                assert!(!proven || dense, "{r} {rc} {margin}");
                if clear {
                    assert!(proven, "{r} {rc}: a chord above the slope must be proven");
                }
            }
        }
        // A chord whose middle dips into a ridge is refused.
        let ridge = tri([[0., 0., 0.], [5., 5., 4.], [10., 0., 0.]]);
        let peak = ridge.bull_contact([5., 2.], 2., 0.7).unwrap();
        let mesh = crate::surface::SurfaceMesh {
            version: 1,
            units: "mm".into(),
            triangles: vec![[[0., 0., 0.], [5., 5., 4.], [10., 0., 0.]]].into(),
            surface_error_mm: 0.,
            source: serde_json::Value::Null,
        };
        let index = Index::new(&mesh, -10., 2., &never).unwrap();
        let mut seen = vec![0; index.len()];
        let (a, b) = ([1., 2., peak - 0.5], [9., 2., peak - 0.5]);
        assert!(ridge.bull_contact([1., 2.], 2., 0.7).unwrap() < a[2]);
        assert!(!index.bull_chord_clear(a, b, 2., 0.7, 0., &mut seen, &mut stamp));
    }

    #[test]
    fn bull_contact_bound_is_never_below_dense_sampling() {
        // As the dense check of `bull_contact`, for the proof's bound: never
        // below any sampled contact, and close to it.
        let cases = [
            ([[0., 0., 0.], [10., 0., 5.], [0., 10., 2.]], [3., 3.]),
            ([[0., 0., 0.], [10., 0., 5.], [0., 10., 2.]], [11., 1.]),
            ([[2., 2., 3.], [8., 3., 1.], [4., 9., 6.]], [5., 5.]),
            ([[2., 2., 3.], [8., 3., 1.], [4., 9., 6.]], [0.5, 4.]),
            ([[0., 0., 4.], [6., 0., 4.], [0., 6., 4.]], [7.2, 1.]),
            // A steep facet and a vertical one (the corner's slope is
            // unbounded at the rim: no Lipschitz bound holds there).
            ([[0., 0., 0.], [1., 0., 9.], [0., 6., 0.]], [1.8, 3.]),
            ([[3., 0., 0.], [3., 6., 0.], [3., 3., 8.]], [1.2, 3.]),
        ];
        for (corners, p) in cases {
            let t = tri(corners);
            for (r, rc) in [(2., 0.), (2., 0.5), (2., 1.5), (2., 2.)] {
                let bound = t.bull_contact_bound(p, r, rc);
                let mut sampled = f64::NEG_INFINITY;
                let n = 600;
                for i in 0..=n {
                    for j in 0..=n - i {
                        let (u, v) = (i as f64 / n as f64, j as f64 / n as f64);
                        let w = [1. - u - v, u, v];
                        let q: Vec3 = std::array::from_fn(|k| {
                            w[0] * corners[0][k] + w[1] * corners[1][k] + w[2] * corners[2][k]
                        });
                        let d = (q[0] - p[0]).hypot(q[1] - p[1]);
                        if d <= r {
                            sampled = sampled.max(q[2] - bull_profile(d, r, rc));
                        }
                    }
                }
                match bound {
                    None => assert!(sampled == f64::NEG_INFINITY, "{p:?} {r} {rc} {sampled}"),
                    Some(z) => assert!(
                        z >= sampled - 1e-9 && z - sampled < 0.05,
                        "{corners:?} {p:?} {r} {rc}: bound {z}, sampled {sampled}"
                    ),
                }
            }
        }
    }

    #[test]
    fn bull_chord_bound_clearance_agrees_with_dense_sampling() {
        let never = std::sync::atomic::AtomicBool::new(false);
        let index_of = |corners: [Vec3; 3]| {
            let mesh = crate::surface::SurfaceMesh {
                version: 1,
                units: "mm".into(),
                triangles: vec![corners].into(),
                surface_error_mm: 0.,
                source: serde_json::Value::Null,
            };
            Index::new(&mesh, -10., 2., &never).unwrap()
        };
        let corners = [[0., 0., 0.], [10., 0., 5.], [0., 10., 2.]];
        let index = index_of(corners);
        let mut seen = vec![0; index.len()];
        let mut stamp = 0;
        let t = tri(corners);
        for (r, rc) in [(2., 0.), (2., 0.7), (2., 2.)] {
            let (pa, pb) = ([1., 2.], [4., 2.5]);
            let contact = |p: [f64; 2]| t.bull_contact(p, r, rc).unwrap();
            for (margin, clear) in [(0.01, true), (-0.01, false)] {
                let mut a = [pa[0], pa[1], contact(pa) + margin];
                let mut b = [pb[0], pb[1], contact(pb) + margin];
                if !clear {
                    a[2] += 0.02;
                    b[2] += 0.02;
                }
                let dense = (0..=2000).all(|k| {
                    let s = k as f64 / 2000.;
                    let p = [a[0] + s * (b[0] - a[0]), a[1] + s * (b[1] - a[1])];
                    contact(p) <= a[2] + s * (b[2] - a[2])
                });
                let proven = index.bull_chord_bound_clear(a, b, r, rc, 0., &mut seen, &mut stamp);
                assert!(!proven || dense, "{r} {rc} {margin}");
                if clear {
                    assert!(proven, "{r} {rc}: a chord above the slope must be proven");
                }
            }
        }
        // A ball rolled level across a ridge (the tent's cross-section at
        // Y 2 peaks at X 5): refused when its middle dips into it, proven
        // with a small clearance, both judged by dense sampling.
        let ridge = [[0., 0., 0.], [5., 5., 4.], [10., 0., 0.]];
        let index = index_of(ridge);
        let mut seen = vec![0; index.len()];
        let t = tri(ridge);
        let peak = t.bull_contact([5., 2.], 2., 2.).unwrap();
        for (lift, expect) in [(-0.02, false), (0.002, true)] {
            let (a, b) = ([1., 2., peak + lift], [9., 2., peak + lift]);
            let dense = (0..=4000).all(|k| {
                let s = k as f64 / 4000.;
                t.bull_contact([1. + 8. * s, 2.], 2., 2.)
                    .is_none_or(|c| c <= peak + lift)
            });
            let proven = index.bull_chord_bound_clear(a, b, 2., 2., 0., &mut seen, &mut stamp);
            assert_eq!(proven, expect, "lift {lift}");
            assert!(!proven || dense);
        }
    }

    /// Dense oracle: the highest tip over many triangle points, from the
    /// section's own profile (independent of the closed-form candidates).
    fn sampled_section(corners: [Vec3; 3], p: [f64; 2], s: &Section, n: usize) -> f64 {
        let mut best = f64::NEG_INFINITY;
        for i in 0..=n {
            for j in 0..=n - i {
                let (u, v) = (i as f64 / n as f64, j as f64 / n as f64);
                let w = [1. - u - v, u, v];
                let q: Vec3 = std::array::from_fn(|k| {
                    w[0] * corners[0][k] + w[1] * corners[1][k] + w[2] * corners[2][k]
                });
                let d = (q[0] - p[0]).hypot(q[1] - p[1]);
                let h = if d <= s.r0 {
                    s.z0
                } else if d > s.r1 {
                    continue;
                } else if s.arc > 0. {
                    // The arc's centre is at (r0, z0 + arc): the lower
                    // half of that circle.
                    let centre = [s.r0, s.z0 + s.arc];
                    centre[1] - (s.arc * s.arc - (d - centre[0]).powi(2)).max(0.).sqrt()
                } else {
                    s.z0 + (d - s.r0) * s.rise
                };
                best = best.max(q[2] - h);
            }
        }
        best
    }

    #[test]
    fn section_contact_bounds_dense_sampling_for_cones_flats_and_frusta() {
        let triangles = [
            [[0., 0., 0.], [10., 0., 5.], [0., 10., 2.]],
            [[2., 2., 3.], [8., 3., 1.], [4., 9., 6.]],
            [[0., 0., 4.], [6., 0., 4.], [0., 6., 4.]],
            // Steep (nearly vertical) wall.
            [[5., 0., 0.], [5., 8., 0.], [5.0001, 4., 9.]],
        ];
        let sections = [
            // 90 degree chamfer mill, 0.2 tip, 6 mm.
            Section {
                z0: 0.,
                r0: 0.1,
                r1: 3.,
                rise: 1.,
                arc: 0.,
            },
            // 60 degree spot drill.
            Section {
                z0: 0.,
                r0: 0.,
                r1: 2.,
                rise: 1. / (30f64.to_radians()).tan(),
                arc: 0.,
            },
            // Flat end, and a holder frustum higher up.
            Section {
                z0: 0.,
                r0: 2.,
                r1: 2.,
                rise: 0.,
                arc: 0.,
            },
            Section {
                z0: 20.,
                r0: 8.,
                r1: 14.,
                rise: 2.5,
                arc: 0.,
            },
        ];
        for corners in triangles {
            let t = tri(corners);
            for p in [
                [3., 3.],
                [11., 1.],
                [5., 5.],
                [0.5, 4.],
                [4.2, 4.],
                [7.2, 1.],
            ] {
                for s in &sections {
                    let sampled = sampled_section(corners, p, s, 500);
                    match t.section_contact(p, s) {
                        None => assert_eq!(sampled, f64::NEG_INFINITY, "{corners:?} {p:?} {s:?}"),
                        // Reached only at the rim, between samples.
                        Some(_) if sampled == f64::NEG_INFINITY => {
                            assert!(t.distance_xy(p) > s.r1 - 0.05, "{corners:?} {p:?} {s:?}")
                        }
                        // Never below a sampled point (an upper bound); close to the
                        // samples within their spacing times the steepest slope.
                        Some(z) => assert!(
                            z >= sampled - 1e-9 && z - sampled < 0.2,
                            "{corners:?} {p:?} {s:?}: {z} vs {sampled}"
                        ),
                    }
                }
            }
        }
    }

    /// Ball (r 2), bull-nose (r 2, corner 0.7), the ball grown radially by
    /// 0.3 (as deburr grows sections) and a part arc (r1 - r0 < arc).
    fn arc_sections() -> [Section; 4] {
        [
            Section {
                z0: 0.,
                r0: 0.,
                r1: 2.,
                rise: 0.,
                arc: 2.,
            },
            Section {
                z0: 0.,
                r0: 1.3,
                r1: 2.,
                rise: 0.,
                arc: 0.7,
            },
            Section {
                z0: 0.,
                r0: 0.3,
                r1: 2.3,
                rise: 0.,
                arc: 2.,
            },
            Section {
                z0: 1.,
                r0: 0.5,
                r1: 1.5,
                rise: 0.,
                arc: 2.,
            },
        ]
    }

    #[test]
    fn section_contact_bounds_dense_sampling_for_ball_and_bull_arcs() {
        let triangles = [
            [[0., 0., 0.], [10., 0., 5.], [0., 10., 2.]],
            [[2., 2., 3.], [8., 3., 1.], [4., 9., 6.]],
            [[0., 0., 4.], [6., 0., 4.], [0., 6., 4.]],
            // Steep (nearly vertical) wall: the arc's steep rim touches it.
            [[5., 0., 0.], [5., 8., 0.], [5.0001, 4., 9.]],
            // Very steep facet (slope 40).
            [[3., 0., 0.], [3.25, 0., 10.], [3., 9., 0.]],
        ];
        for corners in triangles {
            let t = tri(corners);
            for p in [
                [3., 3.],
                [11., 1.],
                [5., 5.],
                [0.5, 4.],
                [4.2, 4.],
                [7.2, 1.],
                [5.6, 4.],
                [1.4, 2.],
            ] {
                for s in &arc_sections() {
                    assert!(s.valid());
                    let sampled = sampled_section(corners, p, s, 600);
                    match t.section_contact(p, s) {
                        None => assert_eq!(sampled, f64::NEG_INFINITY, "{corners:?} {p:?} {s:?}"),
                        Some(_) if sampled == f64::NEG_INFINITY => {
                            assert!(t.distance_xy(p) > s.r1 - 0.05, "{corners:?} {p:?} {s:?}")
                        }
                        // An upper bound of every sample, and close to them.
                        // At a full quarter arc's rim the slope is unbounded,
                        // so samples one spacing apart can differ by
                        // sqrt(2 arc spacing) in height: allow that.
                        Some(z) => assert!(
                            z >= sampled - 1e-9 && z - sampled < 0.25,
                            "{corners:?} {p:?} {s:?}: {z} vs {sampled}"
                        ),
                    }
                }
            }
        }
    }

    #[test]
    fn arc_section_contact_agrees_with_the_bull_nose_closed_form() {
        // `bull_contact` (used by the surface finishing strategies, tested
        // above against dense sampling) computes the same contact by its
        // own code: the two must agree for ball and bull-nose ends.
        let triangles = [
            [[0., 0., 0.], [10., 0., 5.], [0., 10., 2.]],
            [[2., 2., 3.], [8., 3., 1.], [4., 9., 6.]],
            [[0., 0., 4.], [6., 0., 4.], [0., 6., 4.]],
            [[5., 0., 0.], [5., 8., 0.], [5.0001, 4., 9.]],
            [[3., 0., 0.], [3.25, 0., 10.], [3., 9., 0.]],
        ];
        for corners in triangles {
            let t = tri(corners);
            for i in 0..=24 {
                for j in 0..=24 {
                    let p = [-1. + 0.5 * i as f64, -1. + 0.5 * j as f64];
                    for (r, rc) in [(2., 2.), (2., 0.7), (3., 1.)] {
                        let s = Section {
                            z0: 0.,
                            r0: r - rc,
                            r1: r,
                            rise: 0.,
                            arc: rc,
                        };
                        match (t.section_contact(p, &s), t.bull_contact(p, r, rc)) {
                            (None, None) => {}
                            (Some(a), Some(b)) => assert!(
                                a >= b - 1e-6 && a - b < 1e-6,
                                "{corners:?} {p:?} {r} {rc}: section {a}, bull {b}"
                            ),
                            other => panic!("{corners:?} {p:?} {r} {rc}: {other:?}"),
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn arc_chord_clearance_agrees_with_dense_positions() {
        let mesh = SurfaceMesh {
            version: 1,
            units: "mm".into(),
            triangles: std::sync::Arc::new(vec![
                [[0., 0., 0.], [20., 0., 0.], [0., 20., 0.]],
                // A wall rising to Z 6 along x = 10.
                [[10., 0., 0.], [10., 20., 0.], [10.0001, 10., 6.]],
                // A ridge.
                [[12., 2., 0.], [16., 10., 3.], [20., 2., 0.]],
            ]),
            surface_error_mm: 0.,
            source: serde_json::json!({}),
        };
        let index = Index::new(&mesh, -1., 3., &|| false).unwrap();
        let mut seen = vec![0; index.len()];
        let mut stamp = 0;
        let mut checked = (0, 0);
        for (r, rc) in [(2., 2.), (2., 0.7)] {
            let arc = [Section {
                z0: 0.,
                r0: r - rc,
                r1: r,
                rise: 0.,
                arc: rc,
            }];
            for (a, b) in [
                ([2., 5., 0.5], [6., 15., 0.5]),
                ([4., 5., 1.], [9.5, 15., 1.]),
                ([7.5, 5., 0.2], [7.5, 15., 0.2]),
                ([7.9, 5., 0.6], [8.3, 15., 0.6]),
                ([12., 2., 1.], [16., 18., 2.]),
                ([11., 6., 2.], [21., 6., 2.]),
                ([11., 6., 3.2], [21., 6., 3.2]),
                ([2., 10., 4.], [12., 10., 4.]),
                ([2., 10., 6.5], [14., 10., 6.5]),
            ] {
                let proven = index.sections_chord_clear(a, b, &arc, 0., &mut seen, &mut stamp);
                // Independent: the bull-nose closed form at dense positions.
                let mut lowest = f64::INFINITY;
                for i in 0..=4000 {
                    let t = i as f64 / 4000.;
                    let p = [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
                    let need = index.max_bull(p, r, rc, &mut seen, &mut stamp);
                    lowest = lowest.min(a[2] + t * (b[2] - a[2]) - need);
                }
                if proven {
                    checked.0 += 1;
                    assert!(
                        lowest >= -1e-9,
                        "{r} {rc} {a:?}->{b:?} proven but gouges by {lowest}"
                    );
                } else {
                    checked.1 += 1;
                }
                if lowest > 1e-3 {
                    assert!(
                        proven,
                        "{r} {rc} {a:?}->{b:?} clear by {lowest} but not proven"
                    );
                }
            }
        }
        // Both outcomes occur.
        assert!(checked.0 > 3 && checked.1 > 3, "{checked:?}");
    }

    #[test]
    fn ball_and_bull_assembly_sections_follow_their_profile() {
        use crate::tool::{Cutter, HolderSection, ToolAssembly};
        for (shape, flat) in [
            (CutterShape::Ball, 0.),
            (
                CutterShape::Bull {
                    corner_radius_mm: 0.5,
                },
                1.,
            ),
        ] {
            let tool = ToolAssembly {
                id: "t".into(),
                cutter: Cutter {
                    id: "t".into(),
                    name: String::new(),
                    shape: shape.clone(),
                    diameter_mm: 3.,
                    cutting_length_mm: 6.,
                    overall_length_mm: 50.,
                    shank_diameter_mm: 6.,
                    flutes: 2,
                    center_cutting: true,
                    material: "carbide".into(),
                    neck: None,
                },
                stickout_mm: 20.,
                gauge_length_mm: 60.,
                holders: vec![HolderSection {
                    z_min_mm: 20.,
                    z_max_mm: 60.,
                    radius_min_mm: 8.,
                    radius_max_mm: 16.,
                }],
            };
            let prepared = tool.prepare().unwrap();
            let sections = assembly_sections(&prepared).unwrap();
            let end = sections[0];
            assert_eq!((end.z0, end.r0, end.r1), (0., flat, 1.5));
            assert!(end.valid() && end.arc == 1.5 - flat);
            // The assembly's own radius at each height on the end matches
            // the section's profile there.
            for k in 1..50 {
                let z = (1.5 - flat) * k as f64 / 50.;
                let r = prepared.radius_at(z).unwrap().radius_mm;
                let h = end.profile(r).unwrap();
                assert!((h - z).abs() < 1e-9, "{shape:?} z {z}: r {r} -> {h}");
            }
            // The enclosing sections still replace the end by its cylinder.
            assert_eq!(enclosing_sections(&prepared).unwrap()[0].arc, 0.);
            // The shank above the flutes.
            assert_eq!((sections[1].z0, sections[1].r0), (6., 3.));
        }
    }

    #[test]
    fn chamfer_assembly_sections_follow_its_profile() {
        use crate::tool::{Cutter, HolderSection, ToolAssembly};
        let tool = ToolAssembly {
            id: "c90".into(),
            cutter: Cutter {
                id: "c90".into(),
                name: String::new(),
                shape: CutterShape::Chamfer {
                    included_angle_rad: std::f64::consts::FRAC_PI_2,
                    tip_diameter_mm: 0.2,
                },
                diameter_mm: 6.,
                cutting_length_mm: 4.,
                overall_length_mm: 50.,
                shank_diameter_mm: 6.,
                flutes: 2,
                center_cutting: false,
                material: "carbide".into(),
                neck: None,
            },
            stickout_mm: 20.,
            gauge_length_mm: 60.,
            holders: vec![HolderSection {
                z_min_mm: 20.,
                z_max_mm: 60.,
                radius_min_mm: 8.,
                radius_max_mm: 16.,
            }],
        };
        let sections = assembly_sections(&tool.prepare().unwrap()).unwrap();
        // 90 degrees: rise 1 from the 0.1 tip radius to the 3 mm rim.
        let s0 = sections[0];
        assert_eq!((s0.z0, s0.r0, s0.r1), (0., 0.1, 3.));
        assert!((s0.rise - 1.).abs() < 1e-12);
        assert_eq!(
            sections[1],
            Section {
                z0: 4.,
                r0: 3.,
                r1: 3.,
                rise: 0.,
                arc: 0.,
            }
        );
        assert_eq!(
            sections[2],
            Section {
                z0: 20.,
                r0: 8.,
                r1: 16.,
                rise: 5.,
                arc: 0.,
            }
        );
        // Each section agrees with the assembly's radius at its heights.
        let prepared = tool.prepare().unwrap();
        for z in [0.5, 1.9, 2.5] {
            let r = prepared.radius_at(z).unwrap().radius_mm;
            assert!((sections[0].r0 + z / sections[0].rise - r).abs() < 1e-12);
        }
        let r = prepared.radius_at(40.).unwrap().radius_mm;
        assert!((sections[2].r0 + (40. - 20.) / sections[2].rise - r).abs() < 1e-12);
    }

    #[test]
    fn concave_upper_bound_holds_for_kinked_and_steep_functions() {
        for (f, max) in [
            (
                Box::new(|x: f64| -(x - 0.3).abs()) as Box<dyn Fn(f64) -> f64>,
                0.,
            ),
            (
                Box::new(|x: f64| (1000. * x).min(10. - x)),
                10. - 0.01 / 1.001,
            ),
            (Box::new(|x: f64| -(x - 0.71) * (x - 0.71)), 0.),
            (Box::new(|x: f64| 5. - x), 5.),
        ] {
            let bound = concave_max_upper(&f, 0., 1.);
            assert!(bound >= max - 1e-12, "{bound} < {max}");
            assert!(bound - max < 1e-6, "{bound} far above {max}");
        }
    }

    #[test]
    fn sections_chord_clearance_agrees_with_dense_positions() {
        let mesh = SurfaceMesh {
            version: 1,
            units: "mm".into(),
            triangles: std::sync::Arc::new(vec![
                [[0., 0., 0.], [20., 0., 0.], [0., 20., 0.]],
                // A wall rising to Z 6 along x = 10.
                [[10., 0., 0.], [10., 20., 0.], [10.0001, 10., 6.]],
            ]),
            surface_error_mm: 0.,
            source: serde_json::json!({}),
        };
        let index = Index::new(&mesh, -1., 3., &|| false).unwrap();
        let mut seen = vec![0; index.len()];
        let mut stamp = 0;
        let cone = [Section {
            z0: 0.,
            r0: 0.1,
            r1: 3.,
            rise: 1.,
            arc: 0.,
        }];
        for (a, b) in [
            ([2., 5., 0.5], [6., 15., 0.5]),
            ([4., 5., 1.], [9.5, 15., 1.]),
            ([12., 2., 1.], [16., 18., 2.]),
            ([2., 10., 4.], [12., 10., 4.]),
        ] {
            let proven = index.sections_chord_clear(a, b, &cone, 0., &mut seen, &mut stamp);
            let mut lowest = f64::INFINITY;
            for i in 0..=2000 {
                let t = i as f64 / 2000.;
                let p = [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
                let need = index.max_sections(p, &cone, &mut seen, &mut stamp);
                lowest = lowest.min(a[2] + t * (b[2] - a[2]) - need);
            }
            // Proven clear only if every sampled position clears; a clearly
            // clear chord is proven.
            if proven {
                assert!(
                    lowest >= -1e-9,
                    "{a:?}->{b:?} proven but gouges by {lowest}"
                );
            }
            if lowest > 1e-3 {
                assert!(proven, "{a:?}->{b:?} clear by {lowest} but not proven");
            }
        }
    }

    #[test]
    fn bull_contact_matches_dense_sampling_for_flat_bull_and_ball() {
        let cases = [
            ([[0., 0., 0.], [10., 0., 5.], [0., 10., 2.]], [3., 3.]),
            ([[0., 0., 0.], [10., 0., 5.], [0., 10., 2.]], [11., 1.]),
            ([[2., 2., 3.], [8., 3., 1.], [4., 9., 6.]], [5., 5.]),
            ([[2., 2., 3.], [8., 3., 1.], [4., 9., 6.]], [0.5, 4.]),
            ([[0., 0., 4.], [6., 0., 4.], [0., 6., 4.]], [7.2, 1.]),
        ];
        for (corners, p) in cases {
            let t = tri(corners);
            for (r, rc) in [(2., 0.), (2., 0.5), (2., 1.5), (2., 2.)] {
                let exact = t.bull_contact(p, r, rc);
                let mut sampled = f64::NEG_INFINITY;
                let n = 600;
                for i in 0..=n {
                    for j in 0..=n - i {
                        let (u, v) = (i as f64 / n as f64, j as f64 / n as f64);
                        let w = [1. - u - v, u, v];
                        let q: Vec3 = std::array::from_fn(|k| {
                            w[0] * corners[0][k] + w[1] * corners[1][k] + w[2] * corners[2][k]
                        });
                        let d = (q[0] - p[0]).hypot(q[1] - p[1]);
                        if d <= r {
                            sampled = sampled.max(q[2] - bull_profile(d, r, rc));
                        }
                    }
                }
                match exact {
                    None => assert!(sampled == f64::NEG_INFINITY, "{p:?} {r} {rc} {sampled}"),
                    // Never below a sampled point; within the sample spacing.
                    Some(z) => assert!(
                        z >= sampled - 1e-9 && z - sampled < 0.05,
                        "{corners:?} {p:?} {r} {rc}: exact {z}, sampled {sampled}"
                    ),
                }
            }
        }
    }
    #[test]
    fn every_triangle_index_user_accounts_for_bin_entry_storage() {
        let mesh = SurfaceMesh {
            version: 1,
            units: "mm".into(),
            triangles: vec![[[-1e6, -1e6, 1.], [1e6, -1e6, 1.], [1e6, 1e6, 1.]]; 4096].into(),
            surface_error_mm: 0.,
            source: serde_json::Value::Null,
        };
        let error = crate::resources::with_memory_budget(1_048_576, || {
            Index::new(&mesh, 0., 1., &|| false).err().unwrap()
        });
        assert!(error.contains("triangle index needs"), "{error}");
    }
    #[test]
    fn index_construction_and_queries_poll_inside_large_bin_loops() {
        use std::cell::Cell;
        let mesh = SurfaceMesh {
            version: 1,
            units: "mm".into(),
            triangles: vec![[[-100., -100., 1.], [100., -100., 1.], [100., 100., 1.]]; 100].into(),
            surface_error_mm: 0.,
            source: serde_json::Value::Null,
        };
        let polls = Cell::new(0usize);
        let cancelled = || {
            polls.set(polls.get() + 1);
            polls.get() >= 300
        };
        assert!(
            Index::new(&mesh, 0., 1., &cancelled)
                .err()
                .unwrap()
                .contains("cancelled")
        );
        assert_eq!(polls.get(), 300);
        let index = Index::new(&mesh, 0., 1., &|| false).unwrap();
        polls.set(0);
        let error = index
            .max_sections_cancel(
                [0., 0.],
                &[Section {
                    z0: 0.,
                    r0: 300.,
                    r1: 300.,
                    rise: 0.,
                    arc: 0.,
                }],
                &mut vec![0; index.len()],
                &mut 0,
                &cancelled,
            )
            .unwrap_err();
        assert!(error.contains("cancelled"));
        assert_eq!(polls.get(), 300);
    }
}
