//! camlib: a three-axis CAM engine.
//!
//! camlib turns stock, tools and operations into typed tool motion, simulates
//! the material each motion removes, checks it, and posts G-code. This crate is
//! the native core; most applications use it through the Python API
//! (`import camlib`), which adds CadQuery input, jobs, posts and playback.
//!
//! # Pieces
//!
//! * [`stream::StreamManifest`]: a job as data (stock, tools, operations,
//!   start position). Operations are [`pocket::Operation`] variants: facing,
//!   pocket, adaptive pocket, 3D roughing, contour, trace, slot, drilling, bore,
//!   chamfer, parallel and waterline finishing.
//! * [`operation::OperationPlan`]: plans one operation and yields one
//!   [`motion::Motion`] at a time against the current stock.
//! * [`stock::Stock`]: the material model. Each motion is swept through it
//!   ([`stock::Stock::preview_sweep`]) and committed ([`stock::Stock::apply_delta`]);
//!   later motions see the stock left by earlier ones.
//! * [`post`], [`post_lowering`], [`post_intent`]: G-code output and an
//!   independent decoder that checks what was actually written.
//! * [`verification`], [`stock_contact`], [`material`]: geometric checks of
//!   motions against fixtures, protected parts and the evolving stock.
//!
//! Planning is bounded by cancellation (every long call takes a
//! [`stock::CancelSignal`]) and by available memory ([`resources`]), not by
//! fixed caps on job size.
//!
//! # Example
//!
//! Plan and simulate a job, motion by motion:
//!
//! ```
//! use camlib::{operation::OperationPlan, stock::Stock, stream::StreamManifest};
//!
//! let job: StreamManifest = serde_json::from_str(include_str!(
//!     "../fixtures/operations/helical-pocket.json"
//! ))
//! .unwrap();
//! job.validate().unwrap();
//! let never = || false; // a cancel signal that is never raised
//! let mut stock = Stock::new(job.stock.height().unwrap().clone()).unwrap();
//! let (mut position, mut next_id) = (job.initial_position_mm, 1);
//! for operation in job.operations {
//!     let tool = job.tools.iter().find(|t| t.id == operation.tool()).unwrap();
//!     let prepared = tool.prepare().unwrap();
//!     let mut plan =
//!         OperationPlan::prepare_with_stock(operation, tool, &stock, position, next_id, &never)
//!             .unwrap();
//!     while let Some(motion) = plan.next_motion_with_stock(&stock, &never).unwrap() {
//!         let delta = stock.preview_sweep(&prepared, &motion, 1., job.sweep, &never).unwrap();
//!         stock.apply_delta(&delta).unwrap();
//!         position = motion.curve.end();
//!         next_id = motion.id + 1;
//!     }
//!     plan.finish_report(&never).unwrap();
//! }
//! assert!(stock.volume().removed_upper_mm3 > 0.);
//! ```
pub mod adaptive;
mod adaptive_front;
pub mod buffers;
pub mod chamfer;
pub mod contour;
pub mod deburr;
pub mod dynamics;
pub mod empty_cut;
pub mod engagement;
pub mod facing;
pub mod finishing;
pub mod fixtures;
pub mod flat;
pub mod geometry;
pub mod grbl;
mod heights;
pub mod holes;
pub mod limits;
pub mod material;
pub mod motion;
pub mod operation;
pub mod planar;
#[cfg(feature = "profile-planning")]
pub mod planning_profile;
pub mod pocket;
pub mod post;
pub mod post_decode;
pub mod post_intent;
pub mod post_lowering;
pub mod post_rounding;
mod post_scheduling;
mod profiles;
pub mod regeneration;
pub mod requests;
pub mod resources;
pub mod session;
pub mod slot;
pub mod stock;
pub mod stock_contact;
pub mod stream;
pub mod stream_verification;
pub mod surface;
pub mod thermal;
pub mod tool;
pub mod trace;
pub mod turning;
pub mod vcarve;
pub mod verification;
pub mod waterline;

use serde::{Deserialize, Serialize};

pub type Point = [f64; 2];

/// Coordinates are millimetres in work XY. Rings omit their repeated closing point.
/// The adapter supplies an absolute boundary approximation error, separate from
/// the numerical guard used for floating-point distance calculations.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Fixture {
    pub version: u32,
    pub units: String,
    pub name: String,
    pub outer: Vec<Point>,
    #[serde(default)]
    pub holes: Vec<Vec<Point>>,
    pub boundary_error_mm: f64,
    #[serde(default)]
    pub source: serde_json::Value,
    #[serde(default)]
    pub checks: Vec<Check>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Check {
    pub point: Point,
    pub radius_mm: f64,
    pub expected: Verdict,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Verdict {
    Allowed,
    Blocked,
    Uncertain,
}

#[derive(Clone, Copy, Debug, Serialize)]
pub struct Clearance {
    pub verdict: Verdict,
    /// Positive inside the pocket, negative in an island or outside the pocket.
    pub signed_clearance_mm: f64,
    pub guard_mm: f64,
}

pub fn load_fixture(path: &std::path::Path) -> Result<Fixture, String> {
    let input = std::fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
    serde_json::from_slice(&input).map_err(|e| format!("{}: {e}", path.display()))
}

mod navigation;
pub mod roughing;

mod entry;
