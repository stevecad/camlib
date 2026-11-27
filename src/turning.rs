//! Turning (lathe): axisymmetric stock, turning tools and operations in the
//! XZ plane (x = radius, z = the spindle axis).
pub mod body;
pub mod chucking;
#[cfg(test)]
mod chucking_tests;
pub mod model;
pub mod ops;
pub mod profile;
#[cfg(test)]
mod rough_tests;
mod rub;
pub mod size;
pub mod stock;
pub mod taper;
pub mod tool;
