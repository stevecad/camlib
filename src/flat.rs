//! Flat-area finishing (the usual Flat/Horizontal strategy): finds the
//! horizontal faces of a part and finishes each at its exact height with a
//! flat end mill, in raster rows, entering each run with a vertical plunge.
//!
//! Proof: every cut is a horizontal line at the cut height. It is kept only
//! where the highest point of the part (and the machining base) under the
//! tool's swept disk (a capsule in XY) is not above the cut height plus the
//! tolerance. That maximum is computed exactly per triangle (a linear
//! function over the convex triangle-capsule intersection peaks at one of its
//! extreme points); runs that fail are halved until they pass or vanish.
use crate::{
    facing::TraverseAxis,
    motion::{Coolant, Curve, Event, Motion, MoveKind, SpindleDirection, Vec3},
    pocket::ResidualStatus,
    stock::{CancelSignal, Stock, StockSpec},
    surface::SurfaceMesh,
    tool::{CutterShape, ToolAssembly, valid_id},
};
use serde::{Deserialize, Serialize};

fn x_axis() -> TraverseAxis {
    TraverseAxis::X
}
fn level_tolerance() -> f64 {
    0.001
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlatOperation<G = SurfaceMesh> {
    pub id: String,
    pub tool: String,
    pub surface: G,
    /// Machining base: flats at or below it are not machined.
    pub floor_z_mm: f64,
    pub stepover_mm: f64,
    pub clearance_z_mm: f64,
    #[serde(default = "x_axis")]
    pub axis: TraverseAxis,
    /// Axial stock left on the flats.
    #[serde(default)]
    pub stock_to_leave_mm: f64,
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
            floor_z_mm: self.floor_z_mm,
            stepover_mm: self.stepover_mm,
            clearance_z_mm: self.clearance_z_mm,
            axis: self.axis,
            stock_to_leave_mm: self.stock_to_leave_mm,
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
    pub rows: usize,
    pub runs: usize,
    pub cut_length_mm: f64,
    /// Row length near flats that could not be proven clear and was dropped.
    pub dropped_length_mm: f64,
    pub runs_completed: usize,
    /// Runs entered by a shallow zigzag ramp along the run because the
    /// live stock stood above the run's height at its start.
    #[serde(skip_serializing_if = "is_zero")]
    pub ramped_entries: usize,
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

#[derive(Clone, Copy)]
struct Run {
    a: [f64; 2],
    b: [f64; 2],
    z: f64,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase {
    Lift,
    Approach,
    Descend,
    Plunge,
    Ramp,
    Cut,
    Retract,
    Rise,
    Done,
}

pub struct FlatPlan {
    op: FlatOperation,
    spec: StockSpec,
    heights: crate::heights::Heights,
    runs: Vec<Run>,
    run: usize,
    phase: Phase,
    radius: f64,
    ramp: std::collections::VecDeque<Vec3>,
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
        if let Some(l) = levels.iter().find(|l| {
            l.0 > spec.max_mm[2] || spec.max_mm[2] - l.0 > cutter.cutting_length_mm - guard
        }) {
            return Err(format!(
                "invalid flat-area finishing parameters: flat level Z {} is above the stock top or deeper below it than the cutting length {}",
                l.0, cutter.cutting_length_mm
            ));
        }
        let axis = if op.axis == TraverseAxis::X { 0 } else { 1 };
        let cross = 1 - axis;
        let region_limits = op
            .limits
            .as_ref()
            .filter(|l| !l.is_empty())
            .map(|l| crate::limits::PreparedLimits::new(l, r, op.tolerance_mm))
            .transpose()?;
        let mut seen = vec![0u32; index.tris.len()];
        let mut stamp = 0u32;
        let mut runs = vec![];
        let (mut rows, mut dropped) = (0usize, 0f64);
        for &(level, lo, hi) in &levels {
            let z = level + op.stock_to_leave_mm;
            let (c0, c1) = (lo[cross] - r, hi[cross] + r);
            let (a0, a1) = (lo[axis] - r, hi[axis] + r);
            let intervals = ((c1 - c0) / op.stepover_mm).ceil().max(1.) as usize;
            let pitch = (c1 - c0) / intervals as f64;
            let h = op.stepover_mm.min(r) / 4.;
            let samples = ((a1 - a0) / h).ceil().max(1.) as usize;
            crate::resources::check_memory(
                (intervals as u128 + 1) * (samples as u128 + 1) * 8,
                "flat-area rows",
            )?;
            for row in 0..=intervals {
                if cancel.is_cancelled() {
                    return Err("flat-area planning cancelled".into());
                }
                let c = c0 + row as f64 * pitch;
                let point = |s: usize| {
                    let mut p = [0.; 2];
                    p[axis] = a0 + (a1 - a0) * s as f64 / samples as f64;
                    p[cross] = c;
                    p
                };
                // Samples where the tool rests on this flat.
                let on = |p: [f64; 2], seen: &mut [u32], stamp: &mut u32| {
                    let height = index.max_height(p, p, r, seen, stamp);
                    (height - level).abs() <= op.level_tolerance_mm
                };
                let mut row_runs = vec![];
                let mut start = None;
                for s in 0..=samples {
                    let inside = on(point(s), &mut seen, &mut stamp);
                    match (inside, start) {
                        (true, None) => start = Some(s),
                        (false, Some(s0)) => {
                            if s - 1 > s0 {
                                row_runs.push((point(s0), point(s - 1)));
                            }
                            start = None;
                        }
                        _ => {}
                    }
                }
                if let Some(s0) = start
                    && samples > s0
                {
                    row_runs.push((point(s0), point(samples)));
                }
                if row_runs.is_empty() {
                    continue;
                }
                rows += 1;
                if rows % 2 == 0 {
                    row_runs.reverse();
                    for run in &mut row_runs {
                        *run = (run.1, run.0);
                    }
                }
                // Prove each run; halve failures down to h / 16.
                let mut stack: Vec<([f64; 2], [f64; 2], u32)> =
                    row_runs.into_iter().rev().map(|(a, b)| (a, b, 0)).collect();
                while let Some((a, b, depth)) = stack.pop() {
                    let length = (b[0] - a[0]).hypot(b[1] - a[1]);
                    if index.max_height(a, b, r, &mut seen, &mut stamp) <= z + op.tolerance_mm {
                        let Some(limits) = &region_limits else {
                            runs.push(Run { a, b, z });
                            continue;
                        };
                        // Only the stretches proven inside the limits.
                        let line = crate::motion::Curve::Line {
                            start: [a[0], a[1], z],
                            end: [b[0], b[1], z],
                        };
                        let mut kept = 0.;
                        for [t0, t1] in limits.allowed_spans(&line, true)? {
                            let at = |t: f64| [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
                            let (p, q) = (at(t0), at(t1));
                            if (q[0] - p[0]).hypot(q[1] - p[1]) > op.tolerance_mm {
                                runs.push(Run { a: p, b: q, z });
                                kept += (t1 - t0) * length;
                            }
                        }
                        dropped += length - kept;
                        continue;
                    }
                    if length <= h / 16. || depth >= 32 {
                        dropped += length;
                        continue;
                    }
                    let m = [(a[0] + b[0]) / 2., (a[1] + b[1]) / 2.];
                    stack.push((m, b, depth + 1));
                    stack.push((a, m, depth + 1));
                }
            }
        }
        // Rejoin halves that were both proven (same line, touching).
        let mut merged: Vec<Run> = Vec::with_capacity(runs.len());
        for run in runs {
            if let Some(last) = merged.last_mut()
                && last.b == run.a
                && last.z == run.z
            {
                last.b = run.b;
                continue;
            }
            merged.push(run);
        }
        let report = FlatReport {
            strategy: "flat",
            geometric_residual_status: ResidualStatus::Pending,
            levels_z_mm: levels.iter().map(|l| l.0).collect(),
            rows,
            runs: merged.len(),
            cut_length_mm: merged
                .iter()
                .map(|r| (r.b[0] - r.a[0]).hypot(r.b[1] - r.a[1]))
                .sum(),
            dropped_length_mm: dropped,
            runs_completed: 0,
            ramped_entries: 0,
            clearance_proof: "exact maximum part height under each horizontal cut's swept disk (triangle-capsule extreme points) at most the cut height plus tolerance; vertical entries above proven runs",
        };
        Ok(Self {
            op,
            spec: spec.clone(),
            heights,
            runs: merged,
            run: 0,
            phase: Phase::Lift,
            radius: r,
            ramp: Default::default(),
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
    /// What enters the run at `run.a` after the rapid descent: a straight
    /// plunge where no live stock stands above the run's height there (the
    /// cells' upper bounds, within a tolerance), otherwise a ramp from the
    /// stock's top over the whole run, down the run by the declared
    /// `RampEntry`. Each ramp point lies vertically above the proven run, so
    /// it clears the part as the run does. A run the ramp does not fit, or
    /// stock above a run with no entry declared, is refused: a flat end mill
    /// is never plunged into stock.
    fn plunge_or_ramp(
        &self,
        run: Run,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<(f64, Vec<Vec3>)>, String> {
        let at_start = stock.top_under_disk(run.a, self.radius);
        if at_start <= run.z + self.op.tolerance_mm {
            return Ok(None);
        }
        let length = (run.b[0] - run.a[0]).hypot(run.b[1] - run.a[1]);
        let top = run_stock_top(stock, run, self.radius, cancel)?;
        let which = format!(
            "flat-area run from ({:.3}, {:.3}) to ({:.3}, {:.3}) at Z{:.3}, {length:.3} mm long, has stock up to Z{top:.3} ({:.3} mm above it, {:.3} mm above it at its start)",
            run.a[0],
            run.a[1],
            run.b[0],
            run.b[1],
            run.z,
            top - run.z,
            at_start - run.z
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
        if top > self.heights.feed {
            return Err(format!(
                "{which}, above the feed height Z{:.3}: raise feed_z_mm/retract_z_mm or rough this area first",
                self.heights.feed
            ));
        }
        let chain = [crate::pocket::Stroke::line(run.a, run.b)];
        let curves = entry
            .zigzag(&chain, top, run.z)
            .map_err(|why| format!("{which}: {why}. Use a steeper declared angle, a longer run, or rough this area first"))?;
        let mut points = vec![];
        let first = curves.first().map_or(run.a, |c| {
            let q = c.start();
            [q[0], q[1]]
        });
        // Over the run at the stock's top, then down the legs.
        points.push([first[0], first[1], top]);
        points.extend(curves.iter().map(|c| c.end()));
        Ok(Some((top, points)))
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
            let (end, kind, next) = match self.phase {
                Phase::Done => {
                    self.drained = true;
                    return Ok(None);
                }
                Phase::Lift => (
                    [p[0], p[1], self.heights.clearance],
                    MoveKind::Rapid,
                    if self.runs.is_empty() {
                        Phase::Done
                    } else {
                        Phase::Approach
                    },
                ),
                Phase::Approach => {
                    let a = self.runs[self.run].a;
                    ([a[0], a[1], p[2]], MoveKind::Rapid, Phase::Descend)
                }
                Phase::Descend => (
                    [p[0], p[1], self.heights.feed],
                    MoveKind::Rapid,
                    Phase::Plunge,
                ),
                Phase::Plunge => {
                    let run = self.runs[self.run];
                    match self.plunge_or_ramp(run, stock, cancel)? {
                        Some((top, points)) => {
                            self.ramp = points.into();
                            self.report.ramped_entries += 1;
                            // Down through air to the stock's top over the run.
                            (
                                [p[0], p[1], top.min(p[2]).max(run.z)],
                                MoveKind::Plunge,
                                Phase::Ramp,
                            )
                        }
                        None => ([p[0], p[1], run.z], MoveKind::Plunge, Phase::Cut),
                    }
                }
                Phase::Ramp => {
                    let end = self.ramp.pop_front().expect("ramp move");
                    ramp_feed = Some(
                        self.op
                            .entry
                            .as_ref()
                            .and_then(|e| e.feed_mm_min())
                            .unwrap_or(self.op.plunge_mm_min),
                    );
                    (
                        end,
                        MoveKind::Plunge,
                        if self.ramp.is_empty() {
                            Phase::Cut
                        } else {
                            Phase::Ramp
                        },
                    )
                }
                Phase::Cut => {
                    let run = self.runs[self.run];
                    ([run.b[0], run.b[1], run.z], MoveKind::Cut, Phase::Retract)
                }
                Phase::Retract => (
                    [p[0], p[1], self.heights.feed.max(p[2])],
                    MoveKind::Retract,
                    Phase::Rise,
                ),
                Phase::Rise => {
                    self.run += 1;
                    self.report.runs_completed += 1;
                    let last = self.run == self.runs.len();
                    let z = if last {
                        self.heights.clearance
                    } else {
                        self.heights.retract
                    };
                    (
                        [p[0], p[1], z.max(p[2])],
                        MoveKind::Rapid,
                        if last { Phase::Done } else { Phase::Approach },
                    )
                }
            };
            self.phase = next;
            if end == p {
                continue;
            }
            let curve = Curve::Line { start: p, end };
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
/// anywhere along a run: sampled at no more than half a cell apart, each
/// sample's disk widened to cover the capsule between samples.
fn run_stock_top(
    stock: &Stock,
    run: Run,
    r: f64,
    cancel: &impl CancelSignal,
) -> Result<f64, String> {
    segment_stock_top(stock, run.a, run.b, r, cancel)
}

/// Highest live stock upper bound over the cells a disk of radius `r`
/// touches anywhere along the segment from `a` to `b` (see `run_stock_top`).
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

#[cfg(test)]
mod tests {
    use super::*;

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
