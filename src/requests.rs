//! The region/geometry request interface: load fixtures, prepare and query
//! planar regions, inspect tools, validate programs and verify motions on one
//! stateful session. The Python bindings expose it as camlib.Engine.
use crate::session::Session;
use crate::{
    Fixture, Point,
    motion::{Capabilities, Program},
    planar::{Boolean, ConstructionOptions, PlanarRegion},
    tool::ToolAssembly,
};
use serde::Deserialize;
use serde_json::{Value, json};
use std::sync::atomic::AtomicBool;

#[derive(Deserialize)]
#[serde(tag = "command", rename_all = "snake_case", deny_unknown_fields)]
pub enum Request {
    Load {
        fixture: Fixture,
    },
    Probe {
        points: Vec<Point>,
    },
    Classify {
        radius_mm: f64,
    },
    SetFeed {
        feed_mm_min: f64,
    },
    Stats,
    InspectTool {
        tool: ToolAssembly,
        heights_mm: Vec<f64>,
    },
    ValidateProgram {
        program: Program,
        capabilities: Option<Capabilities>,
    },
    PrepareRegion {
        region: PlanarRegion,
        #[serde(default)]
        options: ConstructionOptions,
    },
    OffsetRegion {
        delta_mm: f64,
    },
    CutterRegion {
        radius_mm: f64,
        allowance_mm: f64,
    },
    BooleanRegion {
        other: PlanarRegion,
        operation: Boolean,
        #[serde(default)]
        options: ConstructionOptions,
    },
    QueryRegion {
        points: Vec<Point>,
    },
    SilhouetteRegion {
        surface: crate::surface::SurfaceMesh,
        z_mm: f64,
        max_error_mm: f64,
        #[serde(default)]
        options: ConstructionOptions,
    },
    PrepareVerification {
        scene: crate::verification::Scene,
        #[serde(default)]
        options: crate::verification::Options,
    },
    VerifyMotion {
        scene_id: String,
        revision: String,
        scene_sha256: String,
        tool: ToolAssembly,
        motion: crate::motion::Motion,
    },
}

impl Session {
    /// One request of the region/geometry interface (JSON in, JSON data out).
    pub fn handle(&mut self, request: &str) -> Result<Value, String> {
        let request: Request = serde_json::from_str(request).map_err(|e| e.to_string())?;
        execute(self, request)
    }
}

fn execute(session: &mut Session, request: Request) -> Result<Value, String> {
    match request {
        Request::Load { fixture } => {
            session.load(&fixture)?;
            Ok(json!({"loaded":fixture.name}))
        }
        Request::Probe { points } => {
            session.probe(&points)?;
            Ok(json!({"points":points.len()}))
        }
        Request::Classify { radius_mm } => {
            Ok(json!({"results":session.classify(radius_mm)?.as_ref()}))
        }
        Request::SetFeed { feed_mm_min } => {
            session.set_feed(feed_mm_min)?;
            Ok(json!({"feed_mm_min":feed_mm_min}))
        }
        Request::Stats => Ok(json!(session.counters)),
        Request::InspectTool { tool, heights_mm } => {
            let prepared = tool.prepare()?;
            Ok(
                json!({"tool":prepared.definition(),"profile":heights_mm.iter().map(|&z|prepared.radius_at(z)).collect::<Vec<_>>()}),
            )
        }
        Request::ValidateProgram {
            program,
            capabilities,
        } => {
            let summary = program.validate(capabilities.as_ref())?;
            Ok(json!({"program":program,"summary":summary}))
        }
        Request::PrepareRegion { region, options } => {
            let summary = session.prepare_planar(&region, options)?;
            Ok(json!({"region":session.planar()?.definition(),"summary":summary}))
        }
        Request::OffsetRegion { delta_mm } => Ok(json!(session.offset_planar(delta_mm)?)),
        Request::CutterRegion {
            radius_mm,
            allowance_mm,
        } => Ok(json!(session.cutter_planar(radius_mm, allowance_mm)?)),
        Request::BooleanRegion {
            other,
            operation,
            options,
        } => Ok(json!(session.boolean_planar(&other, operation, options)?)),
        Request::QueryRegion { points } => Ok(json!({"results":session.query_planar(&points)?})),
        Request::SilhouetteRegion {
            surface,
            z_mm,
            max_error_mm,
            options,
        } => {
            let silhouette = crate::silhouette::silhouette_region(
                &surface,
                z_mm,
                max_error_mm,
                options,
                &AtomicBool::new(false),
            )?;
            Ok(json!({"region":silhouette.region,"report":silhouette.report}))
        }
        Request::PrepareVerification { scene, options } => {
            session.prepare_verification(&scene, options)?;
            Ok(
                json!({"scene_id":scene.id,"revision":scene.revision,"scene_sha256":session.verifier()?.digest(),"bodies":scene.bodies.len()}),
            )
        }
        Request::VerifyMotion {
            scene_id,
            revision,
            scene_sha256,
            tool,
            motion,
        } => {
            let verifier = session.verifier()?;
            let scene = verifier.definition();
            if scene.id != scene_id
                || scene.revision != revision
                || verifier.digest() != scene_sha256
            {
                return Err("verification scene identity/revision changed".into());
            }
            Ok(json!(verifier.verify_motion(
                &tool.prepare()?,
                &motion,
                &AtomicBool::new(false)
            )?))
        }
    }
}
