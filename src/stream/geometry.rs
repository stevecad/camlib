//! Manifest-local, acyclic mesh references. Resolve typed geometry once; clones
//! share facet storage. A stream manifest always writes its meshes to one
//! shared table (none without meshes); inline meshes are read as well.
//! Standalone operations/targets keep their inline format.
use super::*;
use crate::{
    stream_verification::Config,
    surface::{SurfaceGeometry, SurfaceMesh},
};
use serde::de::Error;

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields, bound(deserialize = "G: Deserialize<'de>"))]
struct Wire<G> {
    version: u32,
    run: RunIdentity,
    stock: RunStockSpec,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    initial_stock: Option<crate::stock::InitialStock>,
    tools: Vec<ToolAssembly>,
    initial_position_mm: Vec3,
    sweep: SweepOptions,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    operations: Vec<Operation<G>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    verification: Option<Config<G>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    tool_change: Option<ToolChangePose>,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    rapid_retracts: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    turning: Option<TurnSetup>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    end_position: Option<GaugePose>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    geometry: Option<Vec<SurfaceGeometry>>,
}
#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    geometry: usize,
    version: u32,
    units: String,
    surface_error_mm: f64,
    source: serde_json::Value,
}
#[derive(Serialize, Deserialize)]
#[serde(untagged)]
enum Surface {
    Reference(Reference),
    Inline(SurfaceMesh),
}
impl StreamManifest {
    fn wire<G, E>(self, f: &mut impl FnMut(SurfaceMesh) -> Result<G, E>) -> Result<Wire<G>, E> {
        Ok(Wire {
            version: self.version,
            run: self.run,
            stock: self.stock,
            initial_stock: self.initial_stock,
            tools: self.tools,
            initial_position_mm: self.initial_position_mm,
            sweep: self.sweep,
            operations: self
                .operations
                .into_iter()
                .map(|op| op.map_geometry(f))
                .collect::<Result<_, _>>()?,
            verification: self.verification.map(|v| v.map_geometry(f)).transpose()?,
            tool_change: self.tool_change,
            rapid_retracts: self.rapid_retracts,
            turning: self.turning,
            end_position: self.end_position,
            geometry: None,
        })
    }
}
impl Serialize for StreamManifest {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut geometry = Vec::new();
        let mut lookup = HashMap::new();
        let mut intern = |mesh: SurfaceMesh| -> Result<Surface, S::Error> {
            let facets = SurfaceGeometry {
                triangles: mesh.triangles,
            };
            let bytes = serde_json::to_vec(&facets).map_err(serde::ser::Error::custom)?;
            // Exact facet bytes include signed zero and triangle/vertex order.
            // This is storage sharing, never approximate geometry equivalence.
            let index = if let Some(&index) = lookup.get(&bytes) {
                index
            } else {
                let index = geometry.len();
                lookup.insert(bytes, index);
                geometry.push(facets);
                index
            };
            Ok(Surface::Reference(Reference {
                geometry: index,
                version: mesh.version,
                units: mesh.units,
                surface_error_mm: mesh.surface_error_mm,
                source: mesh.source,
            }))
        };
        let mut wire = self.clone().wire(&mut intern)?;
        wire.geometry = (!geometry.is_empty()).then_some(geometry);
        wire.serialize(serializer)
    }
}
impl<'de> Deserialize<'de> for StreamManifest {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let wire = Wire::<Surface>::deserialize(deserializer)?;
        let geometry = wire.geometry.unwrap_or_default();
        let mut used = vec![false; geometry.len()];
        let mut resolve = |surface| match surface {
            Surface::Inline(mesh) => Ok(mesh),
            Surface::Reference(reference) => {
                let index = reference.geometry;
                let mesh = geometry
                    .get(index)
                    .ok_or_else(|| D::Error::custom("unknown shared geometry reference"))?;
                used[index] = true;
                Ok(SurfaceMesh {
                    version: reference.version,
                    units: reference.units,
                    triangles: mesh.triangles.clone(),
                    surface_error_mm: reference.surface_error_mm,
                    source: reference.source,
                })
            }
        };
        let operations = wire
            .operations
            .into_iter()
            .map(|op| op.map_geometry(&mut resolve))
            .collect::<Result<_, _>>()?;
        let verification = wire
            .verification
            .map(|v| v.map_geometry(&mut resolve))
            .transpose()?;
        if used.iter().any(|used| !used) {
            return Err(D::Error::custom("unused shared geometry definition"));
        }
        Ok(Self {
            version: wire.version,
            run: wire.run,
            stock: wire.stock,
            initial_stock: wire.initial_stock,
            tools: wire.tools,
            initial_position_mm: wire.initial_position_mm,
            sweep: wire.sweep,
            operations,
            verification,
            tool_change: wire.tool_change,
            rapid_retracts: wire.rapid_retracts,
            turning: wire.turning,
            end_position: wire.end_position,
        })
    }
}
