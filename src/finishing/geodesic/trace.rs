//! A record of how a geodesic toolpath is built, for investigating a plan:
//! phases and their times, coverage rounds and the fills they add, cutter
//! positions that cannot touch their contact (and why), long straight
//! moves, links that retract, and verification samples narrower than the
//! stepover.
//!
//! Off unless `CAMLIB_GEODESIC_TRACE` names a file to append to (`-` for
//! standard error; a run through the streaming driver does not show its
//! standard error, so use a file there). When off, each trace point costs
//! one cached check and builds no text.

use std::io::Write;
use std::sync::{Mutex, OnceLock};

fn sink() -> &'static Option<Mutex<Box<dyn Write + Send>>> {
    static SINK: OnceLock<Option<Mutex<Box<dyn Write + Send>>>> = OnceLock::new();
    SINK.get_or_init(|| {
        let target = std::env::var("CAMLIB_GEODESIC_TRACE").ok()?;
        let out: Box<dyn Write + Send> = if target == "-" {
            Box::new(std::io::stderr())
        } else {
            Box::new(
                std::fs::OpenOptions::new()
                    .create(true)
                    .append(true)
                    .open(&target)
                    .ok()?,
            )
        };
        Some(Mutex::new(out))
    })
}

/// Whether the trace is on.
pub(super) fn on() -> bool {
    sink().is_some()
}

/// Writes one line, built only when the trace is on.
pub(super) fn line(text: impl FnOnce() -> String) {
    if let Some(sink) = sink()
        && let Ok(mut out) = sink.lock()
    {
        let _ = writeln!(out, "{}", text());
    }
}
