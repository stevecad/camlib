use super::*;
use crate::surface::{PreparedSurface, SurfaceMesh};
use std::sync::atomic::AtomicBool;

/// Source premises are explicit. Hausdorff distance alone only proves the
/// outer inclusion; neither an inner mesh nor a signed offset is inferred.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SurfaceEnclosure {
    #[default]
    Bounded,
    Inscribed,
    Offset,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SurfaceTarget<G = SurfaceMesh> {
    pub version: u32,
    pub units: String,
    pub id: String,
    pub revision: String,
    pub surface: G,
    pub base_z_mm: f64,
    #[serde(default)]
    pub source_enclosure: SurfaceEnclosure,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub contact_witnesses: Option<ContactWitnesses>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub inner_patches: Vec<InnerPlanarPatch>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub finishing: Option<FinishingScope>,
}
impl<G> SurfaceTarget<G> {
    pub(crate) fn map_geometry<H, E>(
        self,
        f: &mut impl FnMut(G) -> Result<H, E>,
    ) -> Result<SurfaceTarget<H>, E> {
        Ok(SurfaceTarget {
            version: self.version,
            units: self.units,
            id: self.id,
            revision: self.revision,
            surface: f(self.surface)?,
            base_z_mm: self.base_z_mm,
            source_enclosure: self.source_enclosure,
            contact_witnesses: self.contact_witnesses,
            inner_patches: self.inner_patches,
            finishing: self.finishing,
        })
    }
}

pub struct PreparedSurfaceTarget {
    definition: SurfaceTarget,
    pub(super) surface: PreparedSurface,
    pub(super) access_surface: Option<PreparedSurface>,
    pub(super) contacts: Option<super::contact_points::PreparedContacts>,
    pub(super) patches: super::inner_patches::PreparedPatches,
    options: Options,
    digest: String,
}
impl SurfaceTarget {
    pub fn prepare(&self, options: Options) -> Result<PreparedSurfaceTarget, String> {
        self.prepare_cancel(options, &AtomicBool::new(false))
    }
    pub fn prepare_cancel(
        &self,
        options: Options,
        cancel: &impl CancelSignal,
    ) -> Result<PreparedSurfaceTarget, String> {
        options.validate()?;
        if self.version != 1
            || self.units != "mm"
            || !valid_id(&self.id)
            || self.revision.is_empty()
            || self.revision.len() > 512
            || !self.base_z_mm.is_finite()
            || self.base_z_mm.abs() > 1e6
        {
            return Err("invalid material surface target identity/units/base".into());
        }
        if let Some(scope) = self.finishing {
            scope.validate(self.base_z_mm)?;
        }
        let surface =
            self.surface
                .prepare(self.base_z_mm, options.geometry_tolerance_mm, cancel)?;
        let access_surface = if let Some(scope) = self.finishing {
            // StockSpec bounds every coordinate by 1e6; this encloses the
            // accumulator's actual stock-dependent radius padding for any
            // accepted analysis box. This is an arithmetic guard, not a new
            // geometric tolerance or modified target.
            let maximum_guard = 1024. * f64::EPSILON * (1. + 6e6 + scope.tool_radius_mm);
            surface.contact_window(
                scope.floor_z_mm + scope.tool_radius_mm,
                scope.tool_radius_mm
                    + scope.stock_to_leave_mm
                    + self.surface.surface_error_mm
                    + maximum_guard,
                cancel,
            )?
        } else {
            None
        };
        let contacts = self
            .contact_witnesses
            .as_ref()
            .map(|w| w.prepare(cancel))
            .transpose()?;
        let patches =
            super::inner_patches::PreparedPatches::new(&self.inner_patches, options, cancel)?;
        Ok(PreparedSurfaceTarget {
            patches,
            contacts,
            access_surface,
            definition: self.clone(),
            surface,
            options,
            digest: identity(&(self, options))?,
        })
    }
}
impl PreparedSurfaceTarget {
    pub fn digest(&self) -> &str {
        &self.digest
    }
    pub fn definition(&self) -> &SurfaceTarget {
        &self.definition
    }
    pub fn check_stock(&self, stock: &Stock, cancel: &impl CancelSignal) -> Result<Report, String> {
        self.check(
            stock,
            &Context {
                stock: stock.spec().clone(),
                generation: stock.generation(),
                fraction: 0.,
                run: None,
            },
            cancel,
        )
    }
    pub fn check(
        &self,
        heights: &impl HeightRead,
        context: &Context,
        cancel: &impl CancelSignal,
    ) -> Result<Report, String> {
        check_material(self, heights, context, None, cancel).map(|(r, _)| r)
    }
    /// As `check`, plus the per-move proof's account of the possible-only
    /// gouge cells it decides (the report itself is unchanged).
    pub(crate) fn check_proven(
        &self,
        heights: &impl HeightRead,
        context: &Context,
        proof: &SweepProof,
        cancel: &impl CancelSignal,
    ) -> Result<(Report, SweepProofReport), String> {
        let (report, decided) = check_material(self, heights, context, Some(proof), cancel)?;
        Ok((report, proof.report(decided)))
    }
}
impl MaterialGeometry for PreparedSurfaceTarget {
    fn finishing_target(&self) -> Option<&PreparedSurfaceTarget> {
        self.definition.finishing.map(|_| self)
    }
    fn accepts_stock_z(&self, stock: &StockSpec) -> bool {
        // The source solid may extend below the stock analysis box. Leaf bounds
        // already intersect BOTH target heights with that box; no geometry is
        // moved and no volume outside the box is assessed. A pocket floor has
        // different semantics and retains the default in-stock restriction.
        self.floor() <= stock.max_mm[2]
    }
    fn options(&self) -> Options {
        self.options
    }
    fn floor(&self) -> f64 {
        self.definition.base_z_mm
    }
    fn id(&self) -> &str {
        &self.definition.id
    }
    fn revision(&self) -> &str {
        &self.definition.revision
    }
    fn digest(&self) -> &str {
        &self.digest
    }
    fn algorithm(&self) -> &'static str {
        "surface_material_v1"
    }
    fn policy(&self) -> &'static str {
        if !self.definition.inner_patches.is_empty() {
            return "Whole-cell stock bounds with declared source enclosure and source-backed inner planar patches. Boundary/plane errors guard full footprints; uncovered curved or mixed regions retain uncertainty. Binary64 engineering bounds, not stock provenance or full motion verification.";
        }
        match self.definition.source_enclosure {
            SurfaceEnclosure::Bounded => {
                "Whole-cell stock premise; faceted upper envelope with asserted Hausdorff outer error. Nonzero source error supplies no inner inclusion beyond the exact base. Binary64 engineering bounds; not stock provenance or tool-contact verification."
            }
            SurfaceEnclosure::Inscribed => {
                "Whole-cell stock premise; caller asserts mesh solid inside target, target inside mesh solid offset by surface_error_mm; exact base. Whole-footprint enclosures with binary64 engineering guards; not stock provenance or tool-contact verification."
            }
            SurfaceEnclosure::Offset => {
                "Whole-cell stock premise; caller asserts target between erosion and dilation of mesh solid by surface_error_mm, with exact protected base. Whole-footprint enclosures with binary64 engineering guards; not stock provenance or tool-contact verification."
            }
        }
    }
    fn bounds(
        &self,
        xy: [f64; 4],
        leaf: bool,
        spec: &StockSpec,
        arithmetic: f64,
        stats: &mut Stats,
        cancel: &impl CancelSignal,
    ) -> Result<Option<(FootprintClass, [f64; 2])>, String> {
        if !leaf {
            return Ok(Some((
                FootprintClass::Mixed,
                [self.floor(), spec.max_mm[2]],
            )));
        }
        stats.geometry_queries += 1;
        let error = self.definition.surface.surface_error_mm;
        let outer = [xy[0] - error, xy[1] - error, xy[2] + error, xy[3] + error];
        let Some(mut height) = self.surface.material_bounds(
            outer,
            self.options.geometry_tolerance_mm + arithmetic,
            &mut stats.estimated_boundary_tests,
            self.options.max_boundary_tests,
            cancel,
        )?
        else {
            return Ok(None);
        };
        height[1] += error;
        match self.definition.source_enclosure {
            SurfaceEnclosure::Bounded if error > 0. => height[0] = self.floor(),
            SurfaceEnclosure::Offset => height[0] = (height[0] - error).max(self.floor()),
            SurfaceEnclosure::Inscribed | SurfaceEnclosure::Bounded => (),
        }
        let Some(inner) = self.patches.lower_height(
            xy,
            height[0],
            arithmetic,
            (
                &mut stats.estimated_boundary_tests,
                self.options.max_boundary_tests,
            ),
            cancel,
        )?
        else {
            return Ok(None);
        };
        if inner > height[1] + self.options.geometry_tolerance_mm + arithmetic {
            return Err("inner CAD patch contradicts outer surface enclosure".into());
        }
        height[0] = inner.min(height[1]);
        // The declared analysis domain, like the prismatic verifier, is the
        // original stock box; material outside it is not assessed.
        height = height.map(|z| z.clamp(spec.min_mm[2], spec.max_mm[2]));
        Ok(Some((FootprintClass::Inside, height)))
    }
}

pub enum PreparedMaterial {
    Prismatic(Box<PreparedTarget>),
    Surface(Box<PreparedSurfaceTarget>),
}
impl PreparedMaterial {
    pub fn digest(&self) -> &str {
        match self {
            Self::Prismatic(t) => t.digest(),
            Self::Surface(t) => t.digest(),
        }
    }
    pub fn check(
        &self,
        heights: &impl HeightRead,
        context: &Context,
        cancel: &impl CancelSignal,
    ) -> Result<Report, String> {
        match self {
            Self::Prismatic(t) => t.check(heights, context, cancel),
            Self::Surface(t) => t.check(heights, context, cancel),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Definition<G = SurfaceMesh> {
    Prismatic(Box<Target>),
    Surface(Box<SurfaceTarget<G>>),
}
impl<G> Definition<G> {
    pub(crate) fn map_geometry<H, E>(
        self,
        f: &mut impl FnMut(G) -> Result<H, E>,
    ) -> Result<Definition<H>, E> {
        Ok(match self {
            Self::Prismatic(t) => Definition::Prismatic(t),
            Self::Surface(t) => Definition::Surface(Box::new(t.map_geometry(f)?)),
        })
    }
}
impl From<Target> for Definition {
    fn from(value: Target) -> Self {
        Self::Prismatic(Box::new(value))
    }
}
impl From<SurfaceTarget> for Definition {
    fn from(value: SurfaceTarget) -> Self {
        Self::Surface(Box::new(value))
    }
}
impl Definition {
    pub(crate) fn finishing_scope(&self) -> Option<FinishingScope> {
        match self {
            Self::Prismatic(_) => None,
            Self::Surface(s) => s.finishing,
        }
    }
    pub fn id(&self) -> &str {
        match self {
            Self::Prismatic(t) => &t.id,
            Self::Surface(t) => &t.id,
        }
    }
    pub fn revision(&self) -> &str {
        match self {
            Self::Prismatic(t) => &t.revision,
            Self::Surface(t) => &t.revision,
        }
    }
    pub fn algorithm(&self) -> &'static str {
        match self {
            Self::Prismatic(_) => "prismatic_material_v1",
            Self::Surface(_) => "surface_material_v1",
        }
    }
    pub fn prepare(&self, options: Options) -> Result<PreparedMaterial, String> {
        match self {
            Self::Prismatic(t) => Ok(PreparedMaterial::Prismatic(Box::new(t.prepare(options)?))),
            Self::Surface(t) => Ok(PreparedMaterial::Surface(Box::new(t.prepare(options)?))),
        }
    }
}
