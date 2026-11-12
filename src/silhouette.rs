//! The plan-view silhouette of a surface target at a depth, as a planar
//! region for the 2D operations (issue #173).
//!
//! The silhouette at `z_mm` is the projection onto XY (the shadow) of every
//! part of the target at or above that height: the area a flat end mill whose
//! tip is at `z_mm` must stay beside, because its flutes and shank stand over
//! the tip. It is not the cross-section at `z_mm`: overhangs and everything
//! higher are included.
//!
//! Reference: Autodesk Fusion's "Silhouette" geometry selection for 2D
//! operations is "the boundary of the part when viewed from the Z axis of the
//! work coordinate system", usable on mesh bodies; FreeCAD CAM's Profile on a
//! whole model uses the model's projected outline (`PathUtils.getEnvelope`:
//! `makeSections(heights=[0.0], project=True)`). Both take one outline for the
//! whole part, which is this silhouette at or below the part's lowest point.
//! The outline that follows the part at each depth is, in both, a 3D
//! constant-Z finishing pass: `Waterline` here, whose flat end mill levels
//! are offsets of this same shadow.
//!
//! The construction is the one 3D roughing protects each layer with
//! (`roughing::PreparedRoughingTarget::shadow`): each facet clipped at the
//! level, projected and joined by the exact planar kernel. It is conservative:
//! the level is lowered by the mesh's declared error and a numerical reserve,
//! edge-on facets are enclosed and never dropped, and the region's
//! `boundary_error_mm` states how far the intended surface at or above `z_mm`
//! may lie outside the region. Operations add that error to their clearance
//! (`Contour` cuts at radius + allowance + error + margin).
//!
//! Like every shadow, the silhouette is closed by the mesh's declared surface
//! error (`PreparedRoughingTarget::shadow`, issue #241): seams between facets
//! that do not share their edge vertices are bridged when they are narrower
//! than twice that error, a channel that narrow is part of the silhouette and
//! a concave corner is rounded to the error. It only grows the region, by
//! less than the error the operations already keep clear, so their cutter
//! positions are those of the exact silhouette.
use crate::{
    planar::{ConstructionOptions, PlanarRegion},
    roughing::PreparedRoughingTarget,
    stock::CancelSignal,
    surface::SurfaceMesh,
};
use serde::Serialize;

#[derive(Clone, Debug, Serialize)]
pub struct SilhouetteReport {
    pub z_mm: f64,
    /// Target facets at or above this height are in the silhouette: `z_mm`
    /// less the mesh's declared error and the numerical reserve.
    pub protected_from_z_mm: f64,
    /// The mesh's declared `surface_error_mm`.
    pub source_error_mm: f64,
    pub numerical_z_reserve_mm: f64,
    /// The intended surface at or above `z_mm` lies inside the region or at
    /// most this far outside it (the region's `boundary_error_mm`).
    pub boundary_error_mm: f64,
    pub max_error_mm: f64,
    pub projected_triangles: usize,
    /// Facets seen edge-on (vertical walls) or as a point, enclosed in a box
    /// of `projection_enclosure_mm` instead of being dropped.
    pub enclosed_degenerate_projections: usize,
    pub projection_enclosure_mm: f64,
    pub components: usize,
    pub holes: usize,
    pub vertices: usize,
    pub area_mm2: f64,
}
pub struct Silhouette {
    pub region: PlanarRegion,
    pub report: SilhouetteReport,
}

/// The silhouette of `mesh` at `z_mm` (see the module documentation).
///
/// Refused, never approximated: an invalid mesh or level, no target at or
/// above `z_mm`, facets whose projections the planar kernel cannot join
/// unambiguously (a mesh that is not watertight, with seams wider than twice
/// its declared error), and a result whose stated error exceeds
/// `max_error_mm`.
pub fn silhouette_region(
    mesh: &SurfaceMesh,
    z_mm: f64,
    max_error_mm: f64,
    options: ConstructionOptions,
    cancel: &impl CancelSignal,
) -> Result<Silhouette, String> {
    if !z_mm.is_finite() || z_mm.abs() > 1e6 {
        return Err(format!(
            "silhouette z_mm {z_mm} must be finite and within 1e6"
        ));
    }
    if !max_error_mm.is_finite() || max_error_mm <= 0. {
        return Err(format!(
            "silhouette max_error_mm {max_error_mm} must be finite and positive"
        ));
    }
    let target = PreparedRoughingTarget::prepare(mesh.clone(), options.tolerance_mm, cancel)
        .map_err(|e| format!("silhouette target: {e}"))?;
    let mut work = 0;
    let shadow = target
        .shadow(z_mm, 0., &mut work, cancel)
        .map_err(|e| format!("silhouette at Z{z_mm}: {e}"))?;
    let report = shadow.report;
    let mut region = shadow.region;
    if region.components.is_empty() {
        return Err(format!(
            "silhouette at Z{z_mm}: the target has nothing at or above Z{} (z_mm less its declared \
             error and the numerical reserve); choose a z_mm at or below the top of the part",
            report.protected_from_z_mm
        ));
    }
    if report.boundary_error_mm > max_error_mm {
        return Err(format!(
            "silhouette at Z{z_mm}: its error {} mm exceeds max_error_mm {max_error_mm} (the mesh's \
             surface_error_mm {}, the numerical reserve {} and the enclosure of edge-on facets {}); \
             supply a mesh with a smaller declared error, use a smaller tolerance_mm, or allow a \
             larger max_error_mm",
            report.boundary_error_mm,
            report.source_error_mm,
            report.numerical_z_reserve_mm,
            report.projection_enclosure_mm
        ));
    }
    // The prepared region is what an operation will see: a silhouette it
    // would refuse is refused here.
    let summary = region
        .prepare(options)
        .map_err(|e| format!("silhouette at Z{z_mm}: {e}"))?
        .summary();
    region.name = "silhouette".into();
    if let Some(source) = region.source.as_object_mut() {
        source.insert("adapter".into(), "surface_silhouette_v1".into());
        source.insert("z_mm".into(), z_mm.into());
        source.insert(
            "protected_from_z_mm".into(),
            report.protected_from_z_mm.into(),
        );
    }
    Ok(Silhouette {
        report: SilhouetteReport {
            z_mm,
            protected_from_z_mm: report.protected_from_z_mm,
            source_error_mm: report.source_error_mm,
            numerical_z_reserve_mm: report.numerical_z_reserve_mm,
            boundary_error_mm: region.boundary_error_mm,
            max_error_mm,
            projected_triangles: report.projected_triangles,
            enclosed_degenerate_projections: report.enclosed_degenerate_projections,
            projection_enclosure_mm: report.projection_enclosure_mm,
            components: summary.components,
            holes: summary.holes,
            vertices: report.output_vertices,
            area_mm2: summary.area_mm2,
        },
        region,
    })
}
