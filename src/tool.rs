//! Rotational tool envelopes. z=0 is the tool tip; +z points toward the spindle.
//! Validate/prepare once, then query analytic radii without allocation.
use serde::{Deserialize, Serialize};

pub mod rotational;

pub(crate) fn valid_id(id: &str) -> bool {
    !id.is_empty()
        && id.len() <= 128
        && id
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b"_-.:".contains(&b))
}

fn positive(value: f64) -> bool {
    value.is_finite() && (1e-6..=1e6).contains(&value)
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum CutterShape {
    Flat,
    Ball,
    Bull {
        corner_radius_mm: f64,
    },
    Drill {
        point_angle_rad: f64,
    },
    Chamfer {
        included_angle_rad: f64,
        tip_diameter_mm: f64,
    },
    /// A plasma arc or laser beam: the diameter is the kerf width. It cuts
    /// through the whole sheet below the nozzle (the tip) wherever it moves
    /// while on; it never cuts above the tip.
    Beam,
    /// A touch probe: the diameter is the stylus ball's (its calibrated
    /// effective size), the shank is the stylus stem and the holder sections
    /// are the probe body. Its controlled point is the bottom of the ball.
    /// It never cuts: only probing operations use it, and its moves never
    /// change the stock. The cutting length is the whole ball (the envelope
    /// is the ball's lower half and a cylinder of its radius above), and
    /// the stem must be thinner than the ball.
    Probe,
    /// A lollipop (ball-shank undercut) cutter: a sphere of `diameter_mm` on
    /// a neck. The cutting length is the sphere's height from its tip to
    /// where it meets the neck (between its radius and its diameter).
    /// Needs a neck and tri-dexel stock: it cuts above its widest section.
    Lollipop,
    /// A dovetail cutter: widest at its flat tip (`diameter_mm`), its flank
    /// tapering toward the spindle at `flank_angle_rad` from the tip face
    /// (a 60 degree cutter cuts a 60 degree dovetail). Needs tri-dexel stock:
    /// it cuts under material above its tip.
    Dovetail {
        flank_angle_rad: f64,
    },
    /// Mounted insert geometry and operating limits. Filled-disk cutter
    /// queries do not describe this shape; use the prepared indexable profile.
    Indexable {
        milling: Box<rotational::IndexableMilling>,
    },
}
impl CutterShape {
    pub fn indexable(&self) -> Option<&rotational::IndexableMilling> {
        if let Self::Indexable { milling } = self {
            Some(milling)
        } else {
            None
        }
    }
    /// Whether the cutting part narrows toward the spindle: it then cuts
    /// under material above its widest section, which only tri-dexel stock
    /// represents.
    pub fn undercuts(&self) -> bool {
        matches!(self, Self::Lollipop | Self::Dovetail { .. })
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Neck {
    pub diameter_mm: f64,
    pub length_mm: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Cutter {
    pub id: String,
    #[serde(default)]
    pub name: String,
    pub shape: CutterShape,
    pub diameter_mm: f64,
    pub cutting_length_mm: f64,
    pub overall_length_mm: f64,
    pub shank_diameter_mm: f64,
    pub flutes: u16,
    pub center_cutting: bool,
    #[serde(default)]
    pub material: String,
    #[serde(default)]
    pub neck: Option<Neck>,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct ProfileSlice {
    pub radius_mm: f64,
    pub cutting: bool,
}

pub struct PreparedCutter {
    definition: Cutter,
    radius: f64,
    tip_height: f64,
    slope: f64,
    shoulder: f64,
    indexable: Option<rotational::PreparedIndexableGeometry>,
}

impl Cutter {
    pub fn prepare(&self) -> Result<PreparedCutter, String> {
        if !valid_id(&self.id) {
            return Err(
                "cutter.id must be a stable identifier (ASCII letters/digits/_-.:, 1..128 bytes)"
                    .into(),
            );
        }
        for (field, value) in [
            ("diameter_mm", self.diameter_mm),
            ("cutting_length_mm", self.cutting_length_mm),
            ("overall_length_mm", self.overall_length_mm),
            ("shank_diameter_mm", self.shank_diameter_mm),
        ] {
            if !positive(value) {
                return Err(format!(
                    "cutter.{}: {field} must be finite, in [1e-6, 1e6] mm",
                    self.id
                ));
            }
        }
        if self.flutes == 0 || self.flutes > 1000 {
            return Err("cutter.flutes must be in 1..=1000".into());
        }
        let neck_length = if let Some(neck) = &self.neck {
            if !positive(neck.diameter_mm)
                || !positive(neck.length_mm)
                || neck.diameter_mm > self.diameter_mm
                || neck.diameter_mm > self.shank_diameter_mm
            {
                return Err("neck must have positive finite dimensions and fit within cutter and shank diameters".into());
            }
            neck.length_mm
        } else {
            0.0
        };
        let indexable = self.shape.indexable().map(|m| m.validate()).transpose()?;
        let shoulder = indexable
            .as_ref()
            .map_or(self.cutting_length_mm + neck_length, |g| {
                self.cutting_length_mm.max(g.noncutting().bounds()[1][1])
            });
        if shoulder >= self.overall_length_mm {
            return Err("cutting length plus neck must leave a positive shank length".into());
        }
        let radius = self.diameter_mm / 2.;
        if let Some(g) = &indexable {
            let bounds = g.cutting().bounds();
            let guard = 128. * f64::EPSILON * (1. + radius + self.cutting_length_mm);
            if self.center_cutting
                || self.neck.is_some()
                || bounds[0][1] > radius + guard
                || bounds[1][1] > self.cutting_length_mm + guard
                || self.shape.indexable().unwrap().max_depth_of_cut_mm > self.cutting_length_mm
            {
                return Err("indexable cutter must be non-center-cutting, have no neck, enclose its cutting profile within diameter/cutting length, and keep maximum depth of cut within cutting height".into());
            }
            let count: u64 = g
                .definition()
                .inserts
                .iter()
                .map(|i| u64::from(i.count))
                .sum();
            if count != u64::from(self.flutes) {
                return Err(
                    "indexable cutter flutes must equal the declared mounted insert count".into(),
                );
            }
        }
        let angle_slope = |angle: f64| -> Result<f64, String> {
            if !angle.is_finite() || !(0.001..std::f64::consts::PI - 0.001).contains(&angle) {
                return Err("tool included angle must be finite and strictly between 0.001 and pi-0.001 radians".into());
            }
            Ok(metallic::tan(angle / 2.))
        };
        if self.shape == CutterShape::Probe
            && (self.neck.is_some()
                || self.center_cutting
                || self.shank_diameter_mm >= self.diameter_mm
                || self.cutting_length_mm != self.diameter_mm)
        {
            return Err(format!(
                "probe {}: the stylus is a ball (cutting length = ball diameter) on a stem (shank) thinner than the ball, with no neck, not center cutting",
                self.id
            ));
        }
        let (tip_height, slope) = match self.shape {
            CutterShape::Indexable { .. } => (0., 0.),
            CutterShape::Flat | CutterShape::Beam => (0., 0.),
            CutterShape::Lollipop => {
                let neck = self.neck.as_ref().ok_or(
                    "a lollipop cutter needs a neck: the sphere's top joins it below the shank",
                )?;
                let top = self.cutting_length_mm;
                if top < radius || top > 2. * radius {
                    return Err("a lollipop cutter's cutting length is the sphere's height up to the neck: between its radius and its diameter".into());
                }
                if neck.diameter_mm / 2. > (top * (2. * radius - top)).max(0.).sqrt() + 1e-9 {
                    return Err("a lollipop cutter's neck must fit the sphere where they meet (the cutting length)".into());
                }
                (radius, 0.)
            }
            CutterShape::Dovetail { flank_angle_rad } => {
                if !flank_angle_rad.is_finite()
                    || !(0.01..std::f64::consts::FRAC_PI_2 - 0.001).contains(&flank_angle_rad)
                {
                    return Err("a dovetail cutter's flank angle must be finite and between 0.01 rad and just under 90 degrees".into());
                }
                let slope = metallic::tan(flank_angle_rad);
                let top = radius - self.cutting_length_mm / slope;
                let waist = self
                    .neck
                    .as_ref()
                    .map_or(self.shank_diameter_mm, |n| n.diameter_mm)
                    / 2.;
                if top <= 0. || waist > top + 1e-9 {
                    return Err("a dovetail cutter's flank must end (at its cutting length) no narrower than the neck or shank above it".into());
                }
                (0., slope)
            }
            CutterShape::Ball | CutterShape::Probe => (radius, 0.),
            CutterShape::Bull { corner_radius_mm } => {
                if !positive(corner_radius_mm) || corner_radius_mm > radius {
                    return Err(
                        "corner radius must be positive and no larger than cutter radius".into(),
                    );
                }
                (corner_radius_mm, 0.)
            }
            CutterShape::Drill { point_angle_rad } => {
                let slope = angle_slope(point_angle_rad)?;
                (radius / slope, slope)
            }
            CutterShape::Chamfer {
                included_angle_rad,
                tip_diameter_mm,
            } => {
                if !tip_diameter_mm.is_finite()
                    || !(0.0..self.diameter_mm).contains(&tip_diameter_mm)
                {
                    return Err("chamfer tip diameter must be finite, nonnegative and smaller than cutter diameter".into());
                }
                let slope = angle_slope(included_angle_rad)?;
                ((radius - tip_diameter_mm / 2.) / slope, slope)
            }
        };
        if tip_height > self.cutting_length_mm {
            return Err("cutting length does not contain the complete tool tip geometry".into());
        }
        Ok(PreparedCutter {
            definition: self.clone(),
            radius,
            tip_height,
            slope,
            shoulder,
            indexable,
        })
    }
}

impl PreparedCutter {
    pub fn indexable(&self) -> Option<&rotational::PreparedIndexableGeometry> {
        self.indexable.as_ref()
    }
    pub fn definition(&self) -> &Cutter {
        &self.definition
    }
    /// Analytic cutting lower envelope at radial distance from the tool axis.
    /// This excludes shank/holder geometry and does not establish tool access.
    pub fn tip_height_at_radius(&self, radius_mm: f64) -> Option<f64> {
        if !radius_mm.is_finite() || !(0.0..=self.radius).contains(&radius_mm) {
            return None;
        }
        Some(match self.definition.shape {
            CutterShape::Indexable { .. } => return None,
            CutterShape::Flat | CutterShape::Beam | CutterShape::Dovetail { .. } => 0.,
            CutterShape::Ball | CutterShape::Probe | CutterShape::Lollipop => {
                self.radius
                    - ((self.radius - radius_mm) * (self.radius + radius_mm))
                        .max(0.)
                        .sqrt()
            }
            CutterShape::Bull {
                corner_radius_mm: c,
            } => {
                let r = (radius_mm - (self.radius - c)).max(0.);
                c - ((c - r) * (c + r)).max(0.).sqrt()
            }
            CutterShape::Drill { .. } => radius_mm / self.slope,
            CutterShape::Chamfer {
                tip_diameter_mm, ..
            } => (radius_mm - tip_diameter_mm / 2.).max(0.) / self.slope,
        })
    }
    /// Heights of the cutting part whose radius reaches `radius_mm`: one
    /// interval, the cutting profile being unimodal (it widens from the tip
    /// to its widest section, then may narrow toward the spindle).
    pub fn heights_at_radius(&self, radius_mm: f64) -> Option<[f64; 2]> {
        let low = self.tip_height_at_radius(radius_mm)?;
        let top = self.definition.cutting_length_mm;
        let high = match self.definition.shape {
            CutterShape::Lollipop => (self.radius
                + ((self.radius - radius_mm) * (self.radius + radius_mm))
                    .max(0.)
                    .sqrt())
            .min(top),
            CutterShape::Dovetail { .. } => ((self.radius - radius_mm) * self.slope).min(top),
            _ => top,
        };
        Some([low, high])
    }
    /// The height of the cutting part's widest section (its lowest one).
    pub fn widest_height(&self) -> f64 {
        self.tip_height
    }
    pub fn radius_at(&self, z_mm: f64) -> Option<ProfileSlice> {
        // A single radius cannot state which annular regions cut. Refuse this
        // query instead of silently filling the center of an indexable cutter.
        if self.indexable.is_some() {
            return None;
        }
        if !z_mm.is_finite() || z_mm < 0. || z_mm > self.definition.overall_length_mm {
            return None;
        }
        if z_mm <= self.definition.cutting_length_mm {
            let radius_mm = if let CutterShape::Lollipop = self.definition.shape {
                (z_mm * (2. * self.radius - z_mm)).max(0.).sqrt()
            } else if let CutterShape::Dovetail { .. } = self.definition.shape {
                self.radius - z_mm / self.slope
            } else if z_mm >= self.tip_height {
                self.radius
            } else {
                match self.definition.shape {
                    CutterShape::Indexable { .. } => return None,
                    CutterShape::Flat
                    | CutterShape::Beam
                    | CutterShape::Lollipop
                    | CutterShape::Dovetail { .. } => self.radius,
                    CutterShape::Ball | CutterShape::Probe => {
                        (z_mm * (2. * self.radius - z_mm)).max(0.).sqrt()
                    }
                    CutterShape::Bull {
                        corner_radius_mm: c,
                    } => self.radius - c + (z_mm * (2. * c - z_mm)).max(0.).sqrt(),
                    CutterShape::Drill { .. } => z_mm * self.slope,
                    CutterShape::Chamfer {
                        tip_diameter_mm, ..
                    } => tip_diameter_mm / 2. + z_mm * self.slope,
                }
            };
            Some(ProfileSlice {
                radius_mm,
                cutting: true,
            })
        } else {
            let diameter = if z_mm <= self.shoulder {
                self.definition
                    .neck
                    .as_ref()
                    .map_or(self.definition.shank_diameter_mm, |n| n.diameter_mm)
            } else {
                self.definition.shank_diameter_mm
            };
            Some(ProfileSlice {
                radius_mm: diameter / 2.,
                cutting: false,
            })
        }
    }
}

/// Tip-relative conical/cylindrical outer-envelope section, ordered tip to gauge.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HolderSection {
    pub z_min_mm: f64,
    pub z_max_mm: f64,
    pub radius_min_mm: f64,
    pub radius_max_mm: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ToolAssembly {
    pub id: String,
    pub cutter: Cutter,
    pub stickout_mm: f64,
    pub gauge_length_mm: f64,
    pub holders: Vec<HolderSection>,
}

pub struct PreparedAssembly {
    definition: ToolAssembly,
    cutter: PreparedCutter,
    indexable_noncutting: Option<rotational::PreparedProfile>,
}

impl ToolAssembly {
    pub fn prepare(&self) -> Result<PreparedAssembly, String> {
        if !valid_id(&self.id) {
            return Err("tool assembly needs a stable id".into());
        }
        let cutter = self.cutter.prepare()?;
        if !positive(self.stickout_mm)
            || !positive(self.gauge_length_mm)
            || self.stickout_mm < cutter.shoulder
            || self.stickout_mm >= self.cutter.overall_length_mm
            || self.stickout_mm >= self.gauge_length_mm
        {
            return Err("stickout must expose cutting/neck regions, leave shank clamping length, and be shorter than gauge length".into());
        }
        if self.holders.is_empty() {
            return Err(
                "holder envelope required; missing holder geometry cannot be treated as clear"
                    .into(),
            );
        }
        let mut end = self.stickout_mm;
        for section in &self.holders {
            if section.z_min_mm != end
                || !positive(section.z_max_mm)
                || section.z_max_mm <= section.z_min_mm
                || !positive(section.radius_min_mm)
                || !positive(section.radius_max_mm)
            {
                return Err(
                    "holder sections must be finite, positive, contiguous and ordered tip to gauge"
                        .into(),
                );
            }
            end = section.z_max_mm;
        }
        if end < self.gauge_length_mm {
            return Err("holder envelope must reach the gauge plane".into());
        }
        // Every holder interval occupied by the inserted shank must enclose it.
        for s in &self.holders {
            if s.z_min_mm < self.cutter.overall_length_mm {
                let t = ((self.cutter.overall_length_mm.min(s.z_max_mm) - s.z_min_mm)
                    / (s.z_max_mm - s.z_min_mm))
                    .clamp(0., 1.);
                let r_end = s.radius_min_mm + t * (s.radius_max_mm - s.radius_min_mm);
                if s.radius_min_mm.min(r_end) < self.cutter.shank_diameter_mm / 2. {
                    return Err(
                        "holder outer envelope cannot be smaller than the inserted shank".into(),
                    );
                }
            }
        }
        let indexable_noncutting = if let Some(g) = cutter.indexable() {
            let mut regions = g.noncutting().definition().regions.clone();
            let body_top = g.noncutting().bounds()[1][1];
            if body_top < self.stickout_mm {
                regions.push(rotational::ProfileRegion::AnnularFrustum {
                    z_mm: [body_top, self.stickout_mm],
                    inner_radius_mm: [0.; 2],
                    outer_radius_mm: [self.cutter.shank_diameter_mm / 2.; 2],
                });
            }
            for h in &self.holders {
                regions.push(rotational::ProfileRegion::AnnularFrustum {
                    z_mm: [h.z_min_mm, h.z_max_mm],
                    inner_radius_mm: [0.; 2],
                    outer_radius_mm: [h.radius_min_mm, h.radius_max_mm],
                });
            }
            Some(rotational::RotationalProfile { regions }.prepare()?)
        } else {
            None
        };
        Ok(PreparedAssembly {
            definition: self.clone(),
            cutter,
            indexable_noncutting,
        })
    }
}

impl PreparedAssembly {
    pub(crate) fn indexable_noncutting(&self) -> Option<&rotational::PreparedProfile> {
        self.indexable_noncutting.as_ref()
    }
    pub fn cutter(&self) -> &PreparedCutter {
        &self.cutter
    }
    pub fn definition(&self) -> &ToolAssembly {
        &self.definition
    }
    pub fn radius_at(&self, z_mm: f64) -> Option<ProfileSlice> {
        if z_mm < self.definition.stickout_mm {
            return self.cutter.radius_at(z_mm);
        }
        if !z_mm.is_finite() {
            return None;
        }
        let radius = self
            .definition
            .holders
            .iter()
            .filter(|s| s.z_min_mm <= z_mm && z_mm <= s.z_max_mm)
            .map(|s| {
                s.radius_min_mm
                    + (z_mm - s.z_min_mm) / (s.z_max_mm - s.z_min_mm)
                        * (s.radius_max_mm - s.radius_min_mm)
            })
            .reduce(f64::max)?;
        Some(ProfileSlice {
            radius_mm: radius,
            cutting: false,
        })
    }
}
