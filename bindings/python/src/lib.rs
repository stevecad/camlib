//! Safe buffer leases; the GIL remains held through validation and commit.
use camlib::buffers::{self, HeightRead, HeightWrite};
use pyo3::{
    buffer::{PyBuffer, ReadOnlyCell},
    exceptions::PyValueError,
    prelude::*,
    types::{PyBytes, PyDict},
};
use std::cell::Cell;
use std::sync::atomic::AtomicBool;

struct Writable<'a>(&'a [Cell<f64>]);
impl HeightRead for Writable<'_> {
    fn scalar_len(&self) -> usize {
        self.0.len()
    }
    fn read(&self, index: usize) -> f64 {
        self.0[index].get()
    }
}
impl HeightWrite for Writable<'_> {
    fn write(&mut self, index: usize, value: f64) {
        self.0[index].set(value);
    }
}
struct Readable<'a>(&'a [ReadOnlyCell<f64>]);
impl HeightRead for Readable<'_> {
    fn scalar_len(&self) -> usize {
        self.0.len()
    }
    fn read(&self, index: usize) -> f64 {
        self.0[index].get()
    }
}

#[pyfunction]
fn apply_delta(
    py: Python<'_>,
    heights: &Bound<'_, PyAny>,
    raw: &[u8],
    offset: usize,
    count: usize,
    nx: usize,
    limits: (f64, f64),
) -> PyResult<Option<(usize, usize, usize, usize)>> {
    let (bottom, top) = limits;
    let lease = PyBuffer::<f64>::get(heights)?;
    if lease.dimensions() != 1 {
        return Err(PyValueError::new_err(
            "height buffer must be one-dimensional",
        ));
    }
    let cells = lease
        .as_mut_slice(py)
        .ok_or_else(|| PyValueError::new_err("height buffer must be writable contiguous f64"))?;
    let mut buffer = Writable(cells);
    buffers::apply_packed_delta(&mut buffer, raw, offset, count, nx, bottom, top)
        .map(|rect| rect.map(|r| (r[0], r[1], r[2], r[3])))
        .map_err(PyValueError::new_err)
}
/// Native decoding of a committed-motion frame payload (kind 1): identity,
/// cursor, optional control prefix and motion. Same layout and validation as
/// the Python `_Decoder` (strings u16-length-prefixed UTF-8 with byte limits,
/// discriminators, feed flag). Returns (sequence, start, end, motion, offset)
/// where `offset` is the payload position (relative to `start`) of the stock
/// delta. `raw` is the whole frame; the payload begins at `start`.
struct Wire<'a> {
    data: &'a [u8],
    offset: usize,
}
impl<'a> Wire<'a> {
    fn take(&mut self, count: usize) -> PyResult<&'a [u8]> {
        if count > self.data.len() - self.offset {
            return Err(PyValueError::new_err("truncated binary field"));
        }
        let value = &self.data[self.offset..self.offset + count];
        self.offset += count;
        Ok(value)
    }
    fn u8(&mut self) -> PyResult<u8> {
        Ok(self.take(1)?[0])
    }
    fn u16(&mut self) -> PyResult<u16> {
        Ok(u16::from_le_bytes(self.take(2)?.try_into().unwrap()))
    }
    fn u64(&mut self) -> PyResult<u64> {
        Ok(u64::from_le_bytes(self.take(8)?.try_into().unwrap()))
    }
    fn f64(&mut self) -> PyResult<f64> {
        Ok(f64::from_le_bytes(self.take(8)?.try_into().unwrap()))
    }
    fn point(&mut self) -> PyResult<(f64, f64, f64)> {
        Ok((self.f64()?, self.f64()?, self.f64()?))
    }
    fn string(&mut self, limit: usize) -> PyResult<&'a str> {
        let count = self.u16()? as usize;
        if count > limit {
            return Err(PyValueError::new_err("binary string exceeds byte limit"));
        }
        std::str::from_utf8(self.take(count)?)
            .map_err(|_| PyValueError::new_err("invalid UTF-8 identifier"))
    }
}
#[pyfunction]
#[pyo3(signature = (raw, start, run_id, revision, digest, control_wire=None, tolerance=None))]
#[allow(clippy::too_many_arguments)]
fn decode_committed_chunk<'py>(
    py: Python<'py>,
    raw: &[u8],
    start: usize,
    run_id: &str,
    revision: &str,
    digest: &[u8],
    control_wire: Option<&[u8]>,
    tolerance: Option<f64>,
) -> PyResult<(u64, u64, u64, Bound<'py, PyDict>, usize)> {
    if start > raw.len() {
        return Err(PyValueError::new_err("truncated binary field"));
    }
    let mut d = Wire {
        data: raw,
        offset: start,
    };
    if d.string(128)? != run_id || d.string(512)? != revision || d.take(32)? != digest {
        return Err(PyValueError::new_err(
            "stale/foreign run, revision or manifest",
        ));
    }
    let (sequence, first, end) = (d.u64()?, d.u64()?, d.u64()?);
    if let Some(wire) = control_wire
        && d.take(40)? != wire
    {
        return Err(PyValueError::new_err(
            "control prefix does not match committed state",
        ));
    }
    let identifier = d.u64()?;
    let (operation, tool) = (d.string(128)?, d.string(128)?);
    let kind = d.u8()?;
    let (feed_flag, feed) = (d.u8()?, d.f64()?);
    let curve_kind = d.u8()?;
    let (a, b) = (d.point()?, d.point()?);
    if kind > 13 || curve_kind > 2 || feed_flag > 1 || (feed_flag == 0 && feed != 0.) {
        return Err(PyValueError::new_err(
            "invalid motion/curve/feed discriminator",
        ));
    }
    let curve = PyDict::new(py);
    curve.set_item("kind", ["line", "arc", "helix"][curve_kind as usize])?;
    curve.set_item("start", a)?;
    curve.set_item("end", b)?;
    if let Some(tolerance) = tolerance {
        // Independent consumer checks (the Python curve_length rules), done
        // here so a stream does not re-walk every decoded motion in Python.
        let rapid = kind == 0 || kind == 7;
        if rapid && curve_kind != 0 {
            return Err(PyValueError::new_err("rapid clearance requires a line"));
        }
        if (feed_flag == 0 && !rapid) || (feed_flag == 1 && !(feed.is_finite() && feed > 0.)) {
            return Err(PyValueError::new_err(
                "programmed feed must be positive and finite",
            ));
        }
        let circular = if curve_kind != 0 {
            let mut probe = Wire {
                data: raw,
                offset: d.offset,
            };
            let center = probe.point()?;
            let (plane, sweep) = (probe.u8()?, probe.f64()?);
            Some((center, plane, sweep))
        } else {
            None
        };
        let arr = |p: (f64, f64, f64)| [p.0, p.1, p.2];
        check_curve(
            arr(a),
            arr(b),
            curve_kind,
            circular.map(|(c, plane, sweep)| (arr(c), plane, sweep)),
            tolerance,
        )?;
    }
    if curve_kind != 0 {
        let center = d.point()?;
        let (plane, sweep) = (d.u8()?, d.f64()?);
        if plane > 2 {
            return Err(PyValueError::new_err("invalid arc plane"));
        }
        curve.set_item("center", center)?;
        curve.set_item("plane", ["xy", "xz", "yz"][plane as usize])?;
        curve.set_item("sweep_rad", sweep)?;
    }
    let motion = PyDict::new(py);
    motion.set_item("id", identifier)?;
    motion.set_item("operation", operation)?;
    motion.set_item("tool", tool)?;
    motion.set_item(
        "kind",
        [
            "rapid",
            "cut",
            "plunge",
            "lead",
            "link",
            "retract",
            "tap",
            "shift",
            "thread",
            "probe_position",
            "probe_measure",
            "probe_touch",
            "probe_return",
            "mark",
        ][kind as usize],
    )?;
    motion.set_item("feed_mm_min", (feed_flag == 1).then_some(feed))?;
    motion.set_item("curve", curve)?;
    Ok((sequence, first, end, motion, d.offset - start))
}
/// Accept a run of committed-motion frames (kind 1), and the verification
/// record (kind 8) following each when `verification` is given, from
/// `buffer` at `offset`. Each motion frame gets exactly the checks of
/// `StockState.accept`: frame header, length and checksum; decoding with the
/// consumer curve checks; sequence; expected attribution; identifier, tool,
/// continuity and ID order; the control tool check; delta generation and
/// size. Each verification record is accepted by the tracker against the
/// motion before it, as `StreamVerification.accept` does. The caller applies
/// each stock delta, in order, when it hands that motion on. Stops before an
/// incomplete frame, any other kind, or after `limit` motions; the first
/// invalid frame ends the run with its error after the valid items.
/// Items: (1, raw, motion, end, changed count, changes offset) or
/// (8, raw, completed, progress JSON, check JSON).
#[pyfunction]
#[pyo3(signature = (buffer, offset, run_id, revision, digest, control_wire, tolerance, cells, completed, position, last_id, tools, expect=None, control=None, verification=None, last_motion=None, limit=4096, tool_change=None, last_tool=None, tool_change_tips=None, tool_change_in_place=None))]
#[allow(clippy::too_many_arguments, clippy::type_complexity)]
fn accept_motion_frames<'py>(
    py: Python<'py>,
    buffer: &[u8],
    mut offset: usize,
    run_id: &str,
    revision: &str,
    digest: &[u8],
    control_wire: Option<&[u8]>,
    tolerance: f64,
    cells: usize,
    mut completed: u64,
    mut position: (f64, f64, f64),
    mut last_id: Option<u64>,
    tools: std::collections::HashSet<String>,
    expect: Option<(String, String)>,
    control: Option<PyRef<'py, StreamControlContext>>,
    mut verification: Option<PyRefMut<'py, StreamVerification>>,
    last_motion: Option<Vec<u8>>,
    limit: usize,
    // The manifest's tool-change pose: (gauge Z, fixed XY, gauge length per tool).
    tool_change: Option<(
        f64,
        Option<(f64, f64)>,
        std::collections::HashMap<String, f64>,
    )>,
    mut last_tool: Option<String>,
    // A tool-change table instead: each tool's tip after its change.
    tool_change_tips: Option<std::collections::HashMap<String, (f64, f64, f64)>>,
    // A tool change in place instead: (gauge length per tool, the spindle
    // tool is unknown: restored from a checkpoint that does not record it).
    tool_change_in_place: Option<(std::collections::HashMap<String, f64>, bool)>,
) -> PyResult<(usize, Vec<Bound<'py, pyo3::types::PyTuple>>, Option<String>)> {
    let mut items = vec![];
    let mut motions = 0;
    let mut previous: Option<Vec<u8>> = last_motion;
    let valid_id = |s: &str| {
        (1..=128).contains(&s.len())
            && s.bytes()
                .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'_' | b'.' | b':' | b'-'))
    };
    let mut failure = None;
    while buffer.len() - offset >= 20 {
        let header = &buffer[offset..offset + 20];
        if &header[..8] != b"CAMLSTRM" || u16::from_le_bytes([header[8], header[9]]) != 1 {
            if items.is_empty() {
                return Err(PyValueError::new_err(
                    "unsupported stream magic/version/kind",
                ));
            }
            failure = Some("unsupported stream magic/version/kind".to_string());
            break;
        }
        let kind = u16::from_le_bytes([header[10], header[11]]);
        let length = u32::from_le_bytes(header[12..16].try_into().unwrap()) as usize;
        let wanted = match kind {
            1 => motions < limit,
            8 => verification.is_some() && previous.is_some(),
            _ => false,
        };
        if !wanted || buffer.len() - offset - 20 < length {
            break;
        }
        let raw = &buffer[offset..offset + 20 + length];
        let mut step = || -> PyResult<Bound<'py, pyo3::types::PyTuple>> {
            let checksum = u32::from_le_bytes(header[16..20].try_into().unwrap());
            let mut crc = crc32fast::Hasher::new();
            crc.update(&raw[..16]);
            crc.update(&raw[20..]);
            if crc.finalize() != checksum {
                return Err(PyValueError::new_err("frame checksum mismatch"));
            }
            if kind == 8 {
                let v = verification.as_mut().unwrap();
                let motion = camlib::stream::decode_chunk_motion(
                    &v.manifest,
                    &v.identity,
                    previous.as_ref().unwrap(),
                    camlib::stream::DEFAULT_FRAME_LIMIT,
                )
                .map_err(PyValueError::new_err)?;
                let record = v
                    .tracker
                    .accept(raw, completed, Some(&motion))
                    .map_err(PyValueError::new_err)?;
                let (done, progress, check, status) = record_parts(py, &record)?;
                previous = None;
                return (8u8, PyBytes::new(py, raw), done, progress, check, status)
                    .into_pyobject(py);
            }
            let (sequence, first, end, motion, delta) = decode_committed_chunk(
                py,
                raw,
                20,
                run_id,
                revision,
                digest,
                control_wire,
                Some(tolerance),
            )?;
            if sequence != completed || first != sequence || end != first + 1 {
                return Err(PyValueError::new_err(
                    "missing, repeated or out-of-order chunk",
                ));
            }
            let get = |key: &str| -> PyResult<Bound<'py, PyAny>> {
                motion
                    .get_item(key)?
                    .ok_or_else(|| PyValueError::new_err("decoded motion field missing"))
            };
            let operation: String = get("operation")?.extract()?;
            let tool: String = get("tool")?.extract()?;
            let identifier: u64 = get("id")?.extract()?;
            let rapid = matches!(
                get("kind")?.extract::<String>()?.as_str(),
                "rapid" | "shift"
            );
            if let Some((op, t)) = &expect
                && (op != &operation || t != &tool)
            {
                return Err(PyValueError::new_err(
                    "generated motion attribution mismatch",
                ));
            }
            if !valid_id(&operation) || !tools.contains(&tool) {
                return Err(PyValueError::new_err(
                    "motion is not applicable to this committed state",
                ));
            }
            let curve = get("curve")?;
            let start: (f64, f64, f64) = curve.get_item("start")?.extract()?;
            let finish: (f64, f64, f64) = curve.get_item("end")?.extract()?;
            // The first motion with a new tool may start at the declared
            // tool-change pose (as the stream state allows).
            if start != position
                && last_tool.as_deref() != Some(tool.as_str())
                && tool_change_in_place
                    .as_ref()
                    .is_some_and(|(_, unknown)| *unknown)
            {
                return Err(PyValueError::new_err(format!(
                    "tool {tool} starts away from the last position: {}",
                    camlib::stream::UNKNOWN_SPINDLE_TOOL
                )));
            }
            let changed = last_tool.as_deref() != Some(tool.as_str())
                && (tool_change.as_ref().is_some_and(|(z, xy, gauges)| {
                    let (x, y) = xy.unwrap_or((position.0, position.1));
                    gauges.get(&tool).is_some_and(|g| start == (x, y, z - g))
                }) || tool_change_tips
                    .as_ref()
                    .is_some_and(|tips| tips.get(&tool) == Some(&start))
                    || tool_change_in_place.as_ref().is_some_and(|(gauges, _)| {
                        // The spindle stays: the tip moves by the old
                        // gauge length minus the new, measured from
                        // exactly the tool in the spindle (as the stream
                        // state allows).
                        let Some(new) = gauges.get(&tool) else {
                            return false;
                        };
                        let from = |old: &f64| {
                            let p = camlib::stream::in_place_tip(
                                [position.0, position.1, position.2],
                                *old,
                                *new,
                            );
                            start == (p[0], p[1], p[2])
                        };
                        match &last_tool {
                            Some(last) => gauges.get(last).is_some_and(from),
                            // No tool has moved: the run's first tool
                            // starts where the run does.
                            None => false,
                        }
                    }));
            if (start != position && !changed) || last_id.is_some_and(|l| identifier <= l) {
                return Err(PyValueError::new_err(
                    "motion continuity or ID order mismatch",
                ));
            }
            if let Some(c) = &control {
                c.inner
                    .check_tool(&tool, rapid)
                    .map_err(PyValueError::new_err)?;
            }
            // Stock delta: generation, count, then count x 36 bytes.
            let at = 20 + delta;
            if raw.len() < at + 12 {
                return Err(PyValueError::new_err("truncated binary field"));
            }
            let generation = u64::from_le_bytes(raw[at..at + 8].try_into().unwrap());
            let count = u32::from_le_bytes(raw[at + 8..at + 12].try_into().unwrap()) as usize;
            if generation != completed || count > cells || raw.len() - (at + 12) != count * 36 {
                return Err(PyValueError::new_err(
                    "delta generation/count/length mismatch",
                ));
            }
            completed = end;
            last_id = Some(identifier);
            position = finish;
            last_tool = Some(tool.clone());
            previous = Some(raw.to_vec());
            (1u8, PyBytes::new(py, raw), motion, end, count, at + 12).into_pyobject(py)
        };
        match step() {
            Ok(item) => {
                motions += usize::from(kind == 1);
                items.push(item);
                offset += 20 + length;
            }
            Err(error) if items.is_empty() => return Err(error),
            Err(error) => {
                failure = Some(error.to_string());
                break;
            }
        }
    }
    Ok((offset, items, failure))
}
fn consumer_point_ok(p: [f64; 3]) -> bool {
    p.iter().all(|x| x.is_finite() && x.abs() <= 1e6)
}

/// Mirror of `camlib.simulation.curve_length` validation: same thresholds,
/// formulas and messages, applied by the consumer, not the planner.
fn check_curve(
    a: [f64; 3],
    b: [f64; 3],
    curve_kind: u8,
    circular: Option<([f64; 3], u8, f64)>,
    tolerance: f64,
) -> PyResult<()> {
    if !consumer_point_ok(a) || !consumer_point_ok(b) {
        return Err(PyValueError::new_err("invalid motion endpoints"));
    }
    let Some((c, plane, sweep)) = circular else {
        let length = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2) + (b[2] - a[2]).powi(2)).sqrt();
        if length < 1e-9 {
            return Err(PyValueError::new_err("zero-length motion"));
        }
        return Ok(());
    };
    let (u, v, w) = match plane {
        0 => (0, 1, 2),
        1 => (2, 0, 1),
        2 => (1, 2, 0),
        _ => return Err(PyValueError::new_err("invalid circular geometry")),
    };
    if !consumer_point_ok(c)
        || !sweep.is_finite()
        || !(1e-12..=1000. * std::f64::consts::TAU).contains(&sweep.abs())
    {
        return Err(PyValueError::new_err("invalid circular geometry"));
    }
    let radius = (a[u] - c[u]).hypot(a[v] - c[v]);
    let axial = b[w] - a[w];
    if radius < 1e-6
        || c[w] != a[w]
        || (curve_kind == 1 && axial != 0.)
        || (curve_kind == 2 && axial == 0.)
    {
        return Err(PyValueError::new_err(
            "invalid arc/helix radius or axial displacement",
        ));
    }
    let angle = (a[v] - c[v]).atan2(a[u] - c[u]) + sweep;
    if (c[u] + radius * angle.cos() - b[u]).hypot(c[v] + radius * angle.sin() - b[v]) > tolerance {
        return Err(PyValueError::new_err(
            "circular endpoint disagrees with sweep",
        ));
    }
    Ok(())
}

/// The posting stand-ins (tool assemblies) of a lathe setup's turning
/// tools, as a lathe run manifest's `tools` (JSON in, JSON out).
#[pyfunction]
fn turn_proxy_tools(setup: &str) -> PyResult<String> {
    let setup: camlib::turning::ops::TurnSetup =
        serde_json::from_str(setup).map_err(|e| PyValueError::new_err(e.to_string()))?;
    serde_json::to_string(&setup.proxy_tools()).map_err(|e| PyValueError::new_err(e.to_string()))
}

/// An ISO-style turning insert outline (`Insert::iso_turning`), as JSON.
#[pyfunction]
fn turning_iso_insert(
    included_angle_deg: f64,
    approach_angle_deg: f64,
    edge_length_mm: f64,
    nose_radius_mm: f64,
) -> PyResult<String> {
    let insert = camlib::turning::tool::Insert::iso_turning(
        included_angle_deg,
        approach_angle_deg,
        edge_length_mm,
        nose_radius_mm,
    )
    .map_err(PyValueError::new_err)?;
    serde_json::to_string(&insert).map_err(|e| PyValueError::new_err(e.to_string()))
}

/// An insert from its ISO 1832 code (`Insert::iso_code`): (JSON, nose radius).
#[pyfunction]
fn turning_iso_code(code: &str, approach_angle_deg: f64) -> PyResult<(String, f64)> {
    let (insert, rn) = camlib::turning::tool::Insert::iso_code(code, approach_angle_deg)
        .map_err(PyValueError::new_err)?;
    Ok((
        serde_json::to_string(&insert).map_err(|e| PyValueError::new_err(e.to_string()))?,
        rn,
    ))
}

/// An ISO 1832 insert's worst-case thickness and clearance (#103).
#[pyfunction]
fn turning_iso_insert_body(code: &str) -> PyResult<(f64, f64)> {
    camlib::turning::body::iso_insert_body(code).map_err(PyValueError::new_err)
}

/// An ISO 1832 insert's in-plane size tolerance (#113): ± on d and on m
/// (mm), from its tolerance class, shape and size; None for a round insert.
#[pyfunction]
fn turning_iso_size_tolerance(code: &str) -> PyResult<Option<(f64, f64)>> {
    camlib::turning::size::iso_size_tolerance(code)
        .map(|t| t.map(|t| (t.d_mm, t.m_mm)))
        .map_err(PyValueError::new_err)
}

/// The approach angle of an ISO 5608 holder designation.
#[pyfunction]
fn turning_holder_approach(code: &str) -> PyResult<f64> {
    camlib::turning::tool::iso_holder_approach_deg(code).map_err(PyValueError::new_err)
}

/// camlib's rectangular holder for an insert (`Holder::rectangle`), as JSON.
#[pyfunction]
fn turning_rectangle_holder(
    insert: &str,
    nose_radius_mm: f64,
    side: &str,
    width_mm: f64,
    length_mm: f64,
    offset_mm: f64,
) -> PyResult<String> {
    let insert: camlib::turning::tool::Insert =
        serde_json::from_str(insert).map_err(|e| PyValueError::new_err(e.to_string()))?;
    let side: camlib::turning::stock::TurnSide =
        serde_json::from_value(serde_json::Value::String(side.into()))
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
    let holder = camlib::turning::tool::Holder::rectangle(
        &insert,
        nose_radius_mm,
        side,
        width_mm,
        length_mm,
        offset_mm,
    )
    .map_err(PyValueError::new_err)?;
    serde_json::to_string(&holder).map_err(|e| PyValueError::new_err(e.to_string()))
}

/// Check a turning tool (JSON): its outlines, corner, holder and values;
/// with a supplied model (#110), its mesh, placement and insert cross-check.
#[pyfunction]
fn turning_validate_tool(py: Python<'_>, tool: &str) -> PyResult<()> {
    let tool: camlib::turning::tool::TurningTool =
        serde_json::from_str(tool).map_err(|e| PyValueError::new_err(e.to_string()))?;
    py.detach(|| camlib::turning::ops::validate_tool(&tool))
        .map_err(PyValueError::new_err)
}

/// The envelope of a holder model turning about the tool axis (#110):
/// (z0, z1, radius) cylinders from a mesh (JSON) and its declared placement.
#[pyfunction]
fn holder_envelope_from_mesh(
    py: Python<'_>,
    mesh: &str,
    tip_mm: [f64; 3],
    axis: [f64; 3],
    z_range_mm: [f64; 2],
    axial_resolution_mm: f64,
) -> PyResult<Vec<[f64; 3]>> {
    let mesh: camlib::surface::SurfaceMesh =
        serde_json::from_str(mesh).map_err(|e| PyValueError::new_err(e.to_string()))?;
    py.detach(|| {
        camlib::turning::model::revolved_envelope(
            &mesh,
            tip_mm,
            axis,
            z_range_mm,
            axial_resolution_mm,
        )
    })
    .map_err(PyValueError::new_err)
}

#[pyfunction]
fn validate_heights(
    py: Python<'_>,
    heights: &Bound<'_, PyAny>,
    bottom: f64,
    top: f64,
) -> PyResult<()> {
    let lease = PyBuffer::<f64>::get(heights)?;
    if lease.dimensions() != 1 {
        return Err(PyValueError::new_err(
            "height buffer must be one-dimensional",
        ));
    }
    let cells = lease
        .as_slice(py)
        .ok_or_else(|| PyValueError::new_err("height buffer must be contiguous f64"))?;
    buffers::validate_heights(&Readable(cells), bottom, top).map_err(PyValueError::new_err)
}
#[pyfunction]
fn volume(
    py: Python<'_>,
    heights: &Bound<'_, PyAny>,
    bottom: f64,
    top: f64,
    area: f64,
) -> PyResult<(f64, f64, f64, f64)> {
    let lease = PyBuffer::<f64>::get(heights)?;
    if lease.dimensions() != 1 {
        return Err(PyValueError::new_err(
            "height buffer must be one-dimensional",
        ));
    }
    let cells = lease
        .as_slice(py)
        .ok_or_else(|| PyValueError::new_err("height buffer must be contiguous f64"))?;
    buffers::volume(&Readable(cells), bottom, top, area)
        .map(|v| (v[0], v[1], v[2], v[3]))
        .map_err(PyValueError::new_err)
}
#[pyfunction]
#[pyo3(signature = (heights, nx, display, dirty, limits))]
fn display_patch<'py>(
    py: Python<'py>,
    heights: &Bound<'py, PyAny>,
    nx: usize,
    display: (usize, usize),
    dirty: Option<(usize, usize, usize, usize)>,
    limits: (f64, f64),
) -> PyResult<Bound<'py, PyBytes>> {
    let lease = PyBuffer::<f64>::get(heights)?;
    if lease.dimensions() != 1 {
        return Err(PyValueError::new_err(
            "height buffer must be one-dimensional",
        ));
    }
    let cells = lease
        .as_slice(py)
        .ok_or_else(|| PyValueError::new_err("height buffer must be contiguous f64"))?;
    let raw = buffers::display_patch(
        &Readable(cells),
        nx,
        [display.0, display.1],
        dirty.map(|r| [r.0, r.1, r.2, r.3]),
        [limits.0, limits.1],
    )
    .map_err(PyValueError::new_err)?;
    Ok(PyBytes::new(py, &raw))
}
#[pyfunction]
fn initialise_heights(
    py: Python<'_>,
    heights: &Bound<'_, PyAny>,
    bottom: f64,
    top: f64,
) -> PyResult<()> {
    let lease = PyBuffer::<f64>::get(heights)?;
    if lease.dimensions() != 1 {
        return Err(PyValueError::new_err(
            "height buffer must be one-dimensional",
        ));
    }
    let cells = lease
        .as_mut_slice(py)
        .ok_or_else(|| PyValueError::new_err("height buffer must be writable contiguous f64"))?;
    buffers::initialise_heights(&mut Writable(cells), bottom, top).map_err(PyValueError::new_err)
}

#[pyfunction]
fn initialise_declared_stock(
    py: Python<'_>,
    heights: &Bound<'_, PyAny>,
    manifest: &str,
) -> PyResult<()> {
    let manifest: camlib::stream::StreamManifest =
        serde_json::from_str(manifest).map_err(|e| PyValueError::new_err(e.to_string()))?;
    manifest.validate().map_err(PyValueError::new_err)?;
    let spec = manifest.stock.height().map_err(PyValueError::new_err)?;
    let initial = manifest
        .initial_stock
        .as_ref()
        .ok_or_else(|| PyValueError::new_err("manifest has no declared initial stock"))?;
    initial.validate(spec).map_err(PyValueError::new_err)?;
    let lease = PyBuffer::<f64>::get(heights)?;
    if lease.dimensions() != 1 {
        return Err(PyValueError::new_err(
            "height buffer must be one-dimensional",
        ));
    }
    let cells = lease
        .as_mut_slice(py)
        .ok_or_else(|| PyValueError::new_err("height buffer must be writable contiguous f64"))?;
    if cells.len() != initial.bounds.len() * 2 {
        return Err(PyValueError::new_err(
            "initial stock buffer/grid length mismatch",
        ));
    }
    for (pair, bounds) in cells.as_chunks::<2>().0.iter().zip(&initial.bounds) {
        pair[0].set(bounds.lower_mm);
        pair[1].set(bounds.upper_mm);
    }
    Ok(())
}
/// A run's starting stock (a box or a mesh stock, as JSON) into a fresh
/// height buffer.
#[pyfunction]
fn initialise_stock(py: Python<'_>, heights: &Bound<'_, PyAny>, stock_json: &str) -> PyResult<()> {
    let lease = PyBuffer::<f64>::get(heights)?;
    if lease.dimensions() != 1 {
        return Err(PyValueError::new_err(
            "height buffer must be one-dimensional",
        ));
    }
    let cells = lease
        .as_mut_slice(py)
        .ok_or_else(|| PyValueError::new_err("height buffer must be writable contiguous f64"))?;
    camlib::stock_mesh::initialise_stock(&mut Writable(cells), stock_json, &|| false)
        .map_err(PyValueError::new_err)
}
/// Check a stock mesh (little-endian f64 vertices, u32 triangle indices):
/// its bounds and enclosed volume.
#[pyfunction]
fn stock_mesh_check(vertices: &[u8], indices: &[u8]) -> PyResult<([f64; 3], [f64; 3], f64)> {
    let mesh =
        camlib::stock_mesh::mesh_from_bytes(vertices, indices).map_err(PyValueError::new_err)?;
    let [lo, hi] = mesh.bounds();
    Ok((lo, hi, mesh.volume_mm3()))
}
/// The closed mesh of a height buffer's upper bounds on the grid given as
/// JSON: vertex bytes, index bytes, bounds, enclosed volume and what the
/// mesh approximates.
#[pyfunction]
#[allow(clippy::type_complexity)]
fn stock_mesh<'py>(
    py: Python<'py>,
    heights: &Bound<'py, PyAny>,
    grid_json: &str,
) -> PyResult<(
    Bound<'py, PyBytes>,
    Bound<'py, PyBytes>,
    [f64; 3],
    [f64; 3],
    f64,
    &'static str,
)> {
    let lease = PyBuffer::<f64>::get(heights)?;
    if lease.dimensions() != 1 {
        return Err(PyValueError::new_err(
            "height buffer must be one-dimensional",
        ));
    }
    let cells = lease
        .as_slice(py)
        .ok_or_else(|| PyValueError::new_err("height buffer must be contiguous f64"))?;
    let mesh = camlib::stock_mesh::buffer_mesh(&Readable(cells), grid_json, &|| false)
        .map_err(PyValueError::new_err)?;
    let (vertices, indices) = camlib::stock_mesh::mesh_to_bytes(&mesh);
    let [lo, hi] = mesh.bounds();
    Ok((
        PyBytes::new(py, &vertices),
        PyBytes::new(py, &indices),
        lo,
        hi,
        mesh.volume_mm3(),
        camlib::stock_mesh::HEIGHT_FIELD_APPROXIMATION,
    ))
}
#[pyfunction]
fn validate_delta(
    py: Python<'_>,
    heights: &Bound<'_, PyAny>,
    raw: &[u8],
    offset: usize,
    count: usize,
    nx: usize,
    limits: (f64, f64),
) -> PyResult<Option<(usize, usize, usize, usize)>> {
    let lease = PyBuffer::<f64>::get(heights)?;
    if lease.dimensions() != 1 {
        return Err(PyValueError::new_err(
            "height buffer must be one-dimensional",
        ));
    }
    let cells = lease
        .as_slice(py)
        .ok_or_else(|| PyValueError::new_err("height buffer must be contiguous f64"))?;
    buffers::validate_packed_delta(&Readable(cells), raw, offset, count, nx, limits.0, limits.1)
        .map(|r| r.map(|r| (r[0], r[1], r[2], r[3])))
        .map_err(PyValueError::new_err)
}
#[pyfunction]
#[pyo3(signature=(heights,raw,delta,grid,dirty,limits))]
fn display_overlay_patch<'py>(
    py: Python<'py>,
    heights: &Bound<'py, PyAny>,
    raw: &[u8],
    delta: (usize, usize),
    grid: (usize, usize, usize),
    dirty: Option<(usize, usize, usize, usize)>,
    limits: (f64, f64),
) -> PyResult<Bound<'py, PyBytes>> {
    let lease = PyBuffer::<f64>::get(heights)?;
    if lease.dimensions() != 1 {
        return Err(PyValueError::new_err(
            "height buffer must be one-dimensional",
        ));
    }
    let cells = lease
        .as_slice(py)
        .ok_or_else(|| PyValueError::new_err("height buffer must be contiguous f64"))?;
    let patch = buffers::display_overlay_patch(
        &Readable(cells),
        raw,
        [delta.0, delta.1],
        grid.0,
        [grid.1, grid.2],
        dirty.map(|r| [r.0, r.1, r.2, r.3]),
        [limits.0, limits.1],
    )
    .map_err(PyValueError::new_err)?;
    Ok(PyBytes::new(py, &patch))
}
/// Derivative bounds of a B-spline or Bezier surface, from its Bezier
/// patches: what the CadQuery adapter proves a spline face's facets with
/// (#240). See `camlib::surface::spline_bounds`.
#[pyclass]
struct SplineSurfaceBounds {
    inner: camlib::surface::spline_bounds::SplineBounds,
}
#[pymethods]
impl SplineSurfaceBounds {
    /// `patches`: per Bezier patch, row-major in u between consecutive
    /// knots, `(rows, columns, poles, weights)` with the poles row-major.
    #[new]
    #[pyo3(signature = (uknots, vknots, patches, kinks_u, kinks_v, period_u=None, period_v=None))]
    #[allow(clippy::type_complexity)]
    fn new(
        uknots: Vec<f64>,
        vknots: Vec<f64>,
        patches: Vec<(usize, usize, Vec<[f64; 3]>, Vec<f64>)>,
        kinks_u: Vec<f64>,
        kinks_v: Vec<f64>,
        period_u: Option<f64>,
        period_v: Option<f64>,
    ) -> PyResult<Self> {
        use camlib::surface::spline_bounds::{BezierPatch, SplineBounds};
        let patches = patches
            .iter()
            .map(|(rows, columns, poles, weights)| {
                BezierPatch::new(*rows, *columns, poles, weights)
            })
            .collect::<Result<Vec<_>, _>>()
            .map_err(PyValueError::new_err)?;
        SplineBounds::new(
            uknots,
            vknots,
            patches,
            [kinks_u, kinks_v],
            [period_u, period_v],
        )
        .map(|inner| Self { inner })
        .map_err(PyValueError::new_err)
    }
    /// `(|Fu|, |Fv|, |Fuu|, |Fuv|, |Fvv|)` over the whole surface.
    fn whole(&self) -> [f64; 5] {
        self.inner.whole()
    }
    /// `(u0, u1, v0, v1)`.
    fn domain(&self) -> [f64; 4] {
        self.inner.domain()
    }
    fn clamp(&self, u: f64, v: f64) -> (f64, f64) {
        let [u, v] = self.inner.clamp([u, v]);
        (u, v)
    }
    /// The bounds over a UV box, and whether a C0 knot line crosses it;
    /// with `limit`, from pieces only as fine as it takes to prove the
    /// box's facet within it.
    #[pyo3(signature = (u0, u1, v0, v1, limit=None))]
    fn local(
        &mut self,
        u0: f64,
        u1: f64,
        v0: f64,
        v1: f64,
        limit: Option<f64>,
    ) -> ([f64; 5], bool) {
        self.inner
            .local_within(u0, u1, v0, v1, limit.unwrap_or(f64::NEG_INFINITY))
    }
    fn pieces(&self) -> usize {
        self.inner.pieces()
    }
}

#[pyclass(frozen)]
struct MaterialTarget {
    inner: camlib::material::PreparedMaterial,
}
#[pyclass]
struct StreamVerification {
    tracker: camlib::stream_verification::Tracker,
    manifest: camlib::stream::StreamManifest,
    identity: Vec<u8>,
}

#[pyclass(frozen)]
struct StreamControlContext {
    inner: camlib::stream::ControlContext,
}
#[pymethods]
impl StreamControlContext {
    #[new]
    fn new(manifest_json: &str) -> PyResult<Self> {
        if manifest_json.len() > camlib::stream::MANIFEST_LIMIT {
            return Err(PyValueError::new_err(
                "manifest exceeds the manifest size limit",
            ));
        }
        let manifest: camlib::stream::StreamManifest = serde_json::from_str(manifest_json)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        if manifest.canonical_bytes().map_err(PyValueError::new_err)? != manifest_json.as_bytes() {
            return Err(PyValueError::new_err(
                "control manifest must contain canonical setup bytes",
            ));
        }
        Ok(Self {
            inner: camlib::stream::ControlContext::new(manifest).map_err(PyValueError::new_err)?,
        })
    }
    fn accepted(
        &self,
        raw: &[u8],
        completed: u64,
        position: (f64, f64, f64),
    ) -> PyResult<(Self, (f64, f64, f64), String)> {
        let (inner, after, event) = self
            .inner
            .accepted(
                raw,
                completed,
                [position.0, position.1, position.2],
                camlib::stream::DEFAULT_FRAME_LIMIT,
            )
            .map_err(PyValueError::new_err)?;
        Ok((
            Self { inner },
            (after[0], after[1], after[2]),
            serde_json::to_string(&event).map_err(|e| PyValueError::new_err(e.to_string()))?,
        ))
    }
    fn restored(&self, raw: &[u8], completed: u64, position: (f64, f64, f64)) -> PyResult<Self> {
        Ok(Self {
            inner: self
                .inner
                .restored(raw, completed, [position.0, position.1, position.2])
                .map_err(PyValueError::new_err)?,
        })
    }
    fn state_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        Ok(PyBytes::new(
            py,
            &self.inner.state_bytes().map_err(PyValueError::new_err)?,
        ))
    }
    fn cursor_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, &self.inner.cursor_bytes())
    }
    fn check_tool(&self, tool: &str, rapid: bool) -> PyResult<()> {
        self.inner
            .check_tool(tool, rapid)
            .map_err(PyValueError::new_err)
    }
}
/// A record's parts: completed, progress JSON, check JSON, the check's kind
/// and the motion check's combined status (violation, uncertain or clear;
/// None when it is not a motion check or nothing was checked), so callers
/// need not parse the check for those.
type RecordParts<'py> = (
    u64,
    Bound<'py, PyBytes>,
    Bound<'py, PyBytes>,
    (&'static str, Option<&'static str>),
);
fn record_parts<'py>(
    py: Python<'py>,
    record: &camlib::stream_verification::Record,
) -> PyResult<RecordParts<'py>> {
    let progress =
        serde_json::to_vec(&record.progress).map_err(|e| PyValueError::new_err(e.to_string()))?;
    let check =
        serde_json::to_vec(&record.check).map_err(|e| PyValueError::new_err(e.to_string()))?;
    use camlib::stream_verification::Check;
    let kind = match &record.check {
        Check::Motion { .. } => "motion",
        Check::MaterialStarted { .. } => "material_started",
        Check::Material { .. } => "material",
    };
    let status = match &record.check {
        camlib::stream_verification::Check::Motion {
            report,
            stock_contact,
        } => {
            use camlib::verification::Status;
            let statuses: Vec<Status> = report
                .iter()
                .map(|r| r.status)
                .chain(stock_contact.iter().map(|r| r.status))
                .collect();
            if statuses.is_empty() {
                None
            } else if statuses.contains(&Status::Violation) {
                Some("violation")
            } else if statuses.contains(&Status::Uncertain) {
                Some("uncertain")
            } else {
                Some("clear")
            }
        }
        _ => None,
    };
    Ok((
        record.completed,
        PyBytes::new(py, &progress),
        PyBytes::new(py, &check),
        (kind, status),
    ))
}
fn source_motion(json: Option<&str>) -> PyResult<Option<camlib::motion::Motion>> {
    json.map(|s| {
        if s.len() > 16384 {
            return Err(PyValueError::new_err("motion metadata exceeds 16 KiB"));
        }
        serde_json::from_str(s).map_err(|e| PyValueError::new_err(e.to_string()))
    })
    .transpose()
}
#[pymethods]
impl StreamVerification {
    #[new]
    fn new(manifest_json: &str) -> PyResult<Self> {
        if manifest_json.len() > camlib::stream::MANIFEST_LIMIT {
            return Err(PyValueError::new_err(
                "manifest exceeds the manifest size limit",
            ));
        }
        // The parsed, validated manifest and its prepared (immutable, shared)
        // binding for these exact bytes: each playback session and seek
        // builds a stream of the same run.
        type Prepared = (
            String,
            camlib::stream::StreamManifest,
            std::sync::Arc<camlib::stream_verification::Binding>,
            Vec<u8>,
        );
        static PREPARED: std::sync::Mutex<Vec<Prepared>> = std::sync::Mutex::new(Vec::new());
        let cached = PREPARED.lock().ok().and_then(|c| {
            c.iter()
                .find(|e| e.0 == manifest_json)
                .map(|e| (e.1.clone(), e.2.clone(), e.3.clone()))
        });
        let (manifest, binding, identity) = match cached {
            Some(hit) => hit,
            None => {
                let manifest: camlib::stream::StreamManifest = serde_json::from_str(manifest_json)
                    .map_err(|e| PyValueError::new_err(e.to_string()))?;
                manifest.validate().map_err(PyValueError::new_err)?;
                if manifest.canonical_bytes().map_err(PyValueError::new_err)?
                    != manifest_json.as_bytes()
                {
                    return Err(PyValueError::new_err(
                        "verification requires canonical native manifest",
                    ));
                }
                let binding = std::sync::Arc::new(
                    camlib::stream_verification::Binding::new(&manifest)
                        .map_err(PyValueError::new_err)?,
                );
                let identity =
                    camlib::stream::identity_bytes(&manifest).map_err(PyValueError::new_err)?;
                if let Ok(mut c) = PREPARED.lock() {
                    c.push((
                        manifest_json.to_owned(),
                        manifest.clone(),
                        binding.clone(),
                        identity.clone(),
                    ));
                    if c.len() > 2 {
                        c.remove(0);
                    }
                }
                (manifest, binding, identity)
            }
        };
        Ok(Self {
            tracker: camlib::stream_verification::Tracker::new(binding, 0),
            manifest,
            identity,
        })
    }
    /// `motion_frame`: the raw committed chunk frame of the source motion
    /// (decoded natively); `motion_json` remains for retained-journal reads.
    /// Returns (completed, progress JSON, check JSON).
    #[pyo3(signature=(raw,completed,motion_json=None,motion_frame=None))]
    fn accept<'py>(
        &mut self,
        py: Python<'py>,
        raw: &[u8],
        completed: u64,
        motion_json: Option<&str>,
        motion_frame: Option<&[u8]>,
    ) -> PyResult<RecordParts<'py>> {
        let motion = match motion_frame {
            Some(frame) => Some(
                camlib::stream::decode_chunk_motion(
                    &self.manifest,
                    &self.identity,
                    frame,
                    camlib::stream::DEFAULT_FRAME_LIMIT,
                )
                .map_err(PyValueError::new_err)?,
            ),
            None => source_motion(motion_json)?,
        };
        let record = self
            .tracker
            .accept(raw, completed, motion.as_ref())
            .map_err(PyValueError::new_err)?;
        record_parts(py, &record)
    }
    #[pyo3(signature=(raw,completed,motion_json=None))]
    fn decode<'py>(
        &self,
        py: Python<'py>,
        raw: &[u8],
        completed: u64,
        motion_json: Option<&str>,
    ) -> PyResult<RecordParts<'py>> {
        let motion = source_motion(motion_json)?;
        let record = self
            .tracker
            .binding
            .decode(raw, completed, motion.as_ref())
            .map_err(PyValueError::new_err)?;
        record_parts(py, &record)
    }
    fn finish(&self, completed: u64) -> PyResult<()> {
        self.tracker
            .finish(completed)
            .map_err(PyValueError::new_err)
    }
}
#[pymethods]
impl MaterialTarget {
    #[new]
    fn new(target_json: &str, options_json: &str) -> PyResult<Self> {
        if target_json.len() > camlib::stream::MANIFEST_LIMIT || options_json.len() > 16384 {
            return Err(PyValueError::new_err(
                "material definition exceeds bounded JSON metadata",
            ));
        }
        let target: camlib::material::Definition =
            serde_json::from_str(target_json).map_err(|e| PyValueError::new_err(e.to_string()))?;
        let options: camlib::material::Options =
            serde_json::from_str(options_json).map_err(|e| PyValueError::new_err(e.to_string()))?;
        let inner = target.prepare(options).map_err(PyValueError::new_err)?;
        Ok(Self { inner })
    }
    #[getter]
    fn digest(&self) -> &str {
        self.inner.digest()
    }

    #[pyo3(signature=(heights,context_json,overlay=None))]
    fn check<'py>(
        &self,
        py: Python<'py>,
        heights: &Bound<'py, PyAny>,
        context_json: &str,
        overlay: Option<(&[u8], usize, usize)>,
    ) -> PyResult<Bound<'py, PyBytes>> {
        if context_json.len() > 65536 {
            return Err(PyValueError::new_err("material context exceeds 64 KiB"));
        }
        let context: camlib::material::Context =
            serde_json::from_str(context_json).map_err(|e| PyValueError::new_err(e.to_string()))?;
        let lease = PyBuffer::<f64>::get(heights)?;
        if lease.dimensions() != 1 {
            return Err(PyValueError::new_err(
                "height buffer must be one-dimensional",
            ));
        }
        let cells = lease
            .as_slice(py)
            .ok_or_else(|| PyValueError::new_err("height buffer must be contiguous f64"))?;
        let buffer = Readable(cells);
        let cancel = AtomicBool::new(false);
        let report = if let Some((raw, offset, count)) = overlay {
            let stock = &context.stock;
            let nx = context.dimensions().map_err(PyValueError::new_err)?[0];
            let view = buffers::DeltaOverlay::new(
                &buffer,
                raw,
                [offset, count],
                nx,
                [stock.min_mm[2], stock.max_mm[2]],
            )
            .map_err(PyValueError::new_err)?;
            self.inner.check(&view, &context, &cancel)
        } else {
            self.inner.check(&buffer, &context, &cancel)
        }
        .map_err(PyValueError::new_err)?;
        let bytes =
            serde_json::to_vec(&report).map_err(|e| PyValueError::new_err(e.to_string()))?;
        if bytes.len() > 8 * 1024 * 1024 {
            return Err(PyValueError::new_err("material report exceeds 8 MiB"));
        }
        Ok(PyBytes::new(py, &bytes))
    }
}

/// The committed state of a lathe run: every turning frame is checked
/// (cursor, attribution, continuity, each cell's old value, material only
/// removed) against this receiver's own turning stock before it commits.
#[pyclass]
#[derive(Clone)]
struct TurnReceiver {
    inner: camlib::stream::StreamReceiver,
}
fn turn_manifest(manifest: &str) -> PyResult<camlib::stream::StreamManifest> {
    let manifest: camlib::stream::StreamManifest =
        serde_json::from_str(manifest).map_err(|e| PyValueError::new_err(e.to_string()))?;
    if manifest.stock.turn().is_none() {
        return Err(PyValueError::new_err(
            "a turning receiver needs a lathe run manifest",
        ));
    }
    Ok(manifest)
}
#[pymethods]
impl TurnReceiver {
    #[new]
    fn new(manifest: &str) -> PyResult<Self> {
        let manifest = turn_manifest(manifest)?;
        Ok(Self {
            inner: camlib::stream::StreamReceiver::new(manifest).map_err(PyValueError::new_err)?,
        })
    }
    /// Restore from a checkpoint frame of this run.
    #[staticmethod]
    fn restore(manifest: &str, checkpoint: &[u8]) -> PyResult<Self> {
        let manifest = turn_manifest(manifest)?;
        Ok(Self {
            inner: camlib::stream::StreamReceiver::from_checkpoint(
                manifest,
                checkpoint,
                camlib::stream::CHECKPOINT_FRAME_LIMIT,
            )
            .map_err(PyValueError::new_err)?,
        })
    }
    /// Commit one turning move frame; returns (motion JSON, changed cells).
    fn accept(&mut self, frame: &[u8]) -> PyResult<(String, usize)> {
        let before = self.inner.run_stock().turn().map(|s| s.cells().to_vec());
        let motion = self
            .inner
            .accept(frame, camlib::stream::DEFAULT_FRAME_LIMIT)
            .map_err(PyValueError::new_err)?;
        let changed = match (before, self.inner.run_stock().turn()) {
            (Ok(b), Ok(a)) => b.iter().zip(a.cells()).filter(|(x, y)| x != y).count(),
            _ => 0,
        };
        Ok((
            serde_json::to_string(&motion).map_err(|e| PyValueError::new_err(e.to_string()))?,
            changed,
        ))
    }
    fn accept_end(&mut self, frame: &[u8]) -> PyResult<()> {
        self.inner
            .accept_end(frame, camlib::stream::DEFAULT_FRAME_LIMIT)
            .map_err(PyValueError::new_err)
    }
    fn checkpoint<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let bytes = self
            .inner
            .checkpoint(camlib::stream::CHECKPOINT_FRAME_LIMIT)
            .map_err(PyValueError::new_err)?;
        Ok(PyBytes::new(py, &bytes))
    }
    fn copy(&self) -> Self {
        self.clone()
    }
    #[getter]
    fn completed(&self) -> u64 {
        self.inner.completed()
    }
    #[getter]
    fn finished(&self) -> bool {
        self.inner.is_finished()
    }
    #[getter]
    fn position(&self) -> (f64, f64, f64) {
        let p = self.inner.position();
        (p[0], p[1], p[2])
    }
    fn stock_json(&self) -> PyResult<String> {
        turn_stock_json(
            self.inner
                .run_stock()
                .turn()
                .map_err(PyValueError::new_err)?,
        )
    }
    /// The committed stock compared with the part (the lathe material check).
    fn material(&self) -> PyResult<String> {
        let m = self.inner.manifest();
        let turning = m
            .turning
            .as_ref()
            .ok_or_else(|| PyValueError::new_err("not a lathe run"))?;
        let spec = m
            .stock
            .turn()
            .ok_or_else(|| PyValueError::new_err("not a lathe run"))?;
        let p = m.initial_position_mm;
        let job = turning.job(&m.run.id, spec, [p[0], p[2]]);
        let stock = self
            .inner
            .run_stock()
            .turn()
            .map_err(PyValueError::new_err)?;
        let report =
            camlib::turning::ops::material_report(&job, stock).map_err(PyValueError::new_err)?;
        serde_json::to_string(&report).map_err(|e| PyValueError::new_err(e.to_string()))
    }
    /// The stock after the first `fraction` of the next move (given as its
    /// committed frame), as JSON; this receiver is unchanged.
    fn preview(&self, frame: &[u8], fraction: f64) -> PyResult<String> {
        let chunk = camlib::stream::decode_turn_chunk_frame(
            self.inner.manifest(),
            frame,
            camlib::stream::DEFAULT_FRAME_LIMIT,
        )
        .map_err(PyValueError::new_err)?;
        if chunk.sequence != self.inner.completed() {
            return Err(PyValueError::new_err("preview frame is not the next move"));
        }
        let stock = self
            .inner
            .preview_turn(&chunk.motion, fraction)
            .map_err(PyValueError::new_err)?;
        turn_stock_json(&stock)
    }
}
fn turn_stock_json(stock: &camlib::turning::stock::TurnStock) -> PyResult<String> {
    let [lower, upper] = stock.volume();
    serde_json::to_string(&serde_json::json!({
        "spec": stock.spec(),
        "cells": stock.cells(),
        "rings": stock.rings(),
        "remaining_lower_mm3": lower,
        "remaining_upper_mm3": upper,
    }))
    .map_err(|e| PyValueError::new_err(e.to_string()))
}

/// The committed state of a tri-dexel run: every move frame is checked
/// (cursor, attribution, continuity, each changed ray's old value, material
/// only removed) against this receiver's own stock before it commits.
#[pyclass]
#[derive(Clone)]
struct DexelReceiver {
    inner: camlib::stream::StreamReceiver,
}
/// A read-only tri-dexel stock (a partial move's preview).
#[pyclass]
struct DexelView {
    stock: camlib::dexel::DexelStock,
}
fn dexel_manifest(manifest: &str) -> PyResult<camlib::stream::StreamManifest> {
    let manifest: camlib::stream::StreamManifest =
        serde_json::from_str(manifest).map_err(|e| PyValueError::new_err(e.to_string()))?;
    if manifest.stock.dexel().is_none() {
        return Err(PyValueError::new_err(
            "a tri-dexel receiver needs a tri-dexel run manifest",
        ));
    }
    Ok(manifest)
}
fn dexel_volume_json(stock: &camlib::dexel::DexelStock) -> PyResult<String> {
    serde_json::to_string(&stock.volume()).map_err(|e| PyValueError::new_err(e.to_string()))
}
/// The height view's cells as little-endian f64 (lower, upper) pairs.
fn dexel_view_bytes<'py>(
    py: Python<'py>,
    stock: &camlib::dexel::DexelStock,
) -> Bound<'py, PyBytes> {
    let cells = stock.view().cells();
    let mut out = Vec::with_capacity(cells.len() * 16);
    for c in cells {
        out.extend_from_slice(&c.lower_mm.to_le_bytes());
        out.extend_from_slice(&c.upper_mm.to_le_bytes());
    }
    PyBytes::new(py, &out)
}
type Intervals = Vec<(f64, f64)>;
fn dexel_ray(
    stock: &camlib::dexel::DexelStock,
    axis: usize,
    index: usize,
) -> PyResult<(Intervals, Intervals)> {
    let ray = (axis < 3)
        .then(|| stock.rays(axis).get(index))
        .flatten()
        .ok_or_else(|| {
            PyValueError::new_err("ray axis must be 0, 1 or 2 and index within its grid")
        })?;
    let list = |s: &camlib::dexel::Spans| s.as_slice().iter().map(|i| (i[0], i[1])).collect();
    Ok((list(&ray.possible), list(&ray.certain)))
}
/// One grid's rays: per ray, its possible and certain interval counts
/// (u32 pairs), then every interval's (start, end) as f64, in ray order.
fn dexel_rays<'py>(
    py: Python<'py>,
    stock: &camlib::dexel::DexelStock,
    axis: usize,
) -> PyResult<(Bound<'py, PyBytes>, Bound<'py, PyBytes>)> {
    if axis > 2 {
        return Err(PyValueError::new_err("ray axis must be 0, 1 or 2"));
    }
    let rays = stock.rays(axis);
    let mut counts = Vec::with_capacity(rays.len() * 8);
    let mut values = vec![];
    for r in rays {
        for s in [&r.possible, &r.certain] {
            counts.extend_from_slice(&(s.len() as u32).to_le_bytes());
            for i in s.as_slice() {
                values.extend_from_slice(&i[0].to_le_bytes());
                values.extend_from_slice(&i[1].to_le_bytes());
            }
        }
    }
    Ok((PyBytes::new(py, &counts), PyBytes::new(py, &values)))
}
fn dexel_compare_solid(
    py: Python<'_>,
    stock: &camlib::dexel::DexelStock,
    triangles: &[u8],
    surface_error_mm: f64,
    tolerance_mm: f64,
) -> PyResult<String> {
    if triangles.is_empty() || !triangles.len().is_multiple_of(72) {
        return Err(PyValueError::new_err(
            "solid triangles are little-endian f64 vertex triples, 72 bytes per facet",
        ));
    }
    let facets: Vec<[[f64; 3]; 3]> = triangles
        .as_chunks::<72>()
        .0
        .iter()
        .map(|t| {
            std::array::from_fn(|v| {
                std::array::from_fn(|k| {
                    let i = (3 * v + k) * 8;
                    f64::from_le_bytes(t[i..i + 8].try_into().unwrap())
                })
            })
        })
        .collect();
    let report = py
        .detach(|| {
            stock.compare_solid(
                &facets,
                surface_error_mm,
                tolerance_mm,
                &AtomicBool::new(false),
            )
        })
        .map_err(PyValueError::new_err)?;
    serde_json::to_string(&report).map_err(|e| PyValueError::new_err(e.to_string()))
}
fn dexel_grid_json(stock: &camlib::dexel::DexelStock) -> PyResult<String> {
    let g = stock.grid();
    serde_json::to_string(&serde_json::json!({
        "spec": stock.spec(),
        "cells": g.cells,
        "step_mm": g.step_mm,
        "rays": [g.rays(0), g.rays(1), g.rays(2)],
        "intervals": stock.intervals(),
        "view": stock.view().spec(),
    }))
    .map_err(|e| PyValueError::new_err(e.to_string()))
}
#[pymethods]
impl DexelView {
    fn compare_solid(
        &self,
        py: Python<'_>,
        triangles: &[u8],
        surface_error_mm: f64,
        tolerance_mm: f64,
    ) -> PyResult<String> {
        dexel_compare_solid(py, &self.stock, triangles, surface_error_mm, tolerance_mm)
    }
    fn volume_json(&self) -> PyResult<String> {
        dexel_volume_json(&self.stock)
    }
    fn view_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        dexel_view_bytes(py, &self.stock)
    }
    fn ray(&self, axis: usize, index: usize) -> PyResult<(Intervals, Intervals)> {
        dexel_ray(&self.stock, axis, index)
    }
    fn rays<'py>(
        &self,
        py: Python<'py>,
        axis: usize,
    ) -> PyResult<(Bound<'py, PyBytes>, Bound<'py, PyBytes>)> {
        dexel_rays(py, &self.stock, axis)
    }
    fn grid_json(&self) -> PyResult<String> {
        dexel_grid_json(&self.stock)
    }
}
impl DexelReceiver {
    fn stock(&self) -> PyResult<&camlib::dexel::DexelStock> {
        self.inner
            .run_stock()
            .dexel()
            .map_err(PyValueError::new_err)
    }
}
#[pymethods]
impl DexelReceiver {
    #[new]
    fn new(manifest: &str) -> PyResult<Self> {
        let manifest = dexel_manifest(manifest)?;
        Ok(Self {
            inner: camlib::stream::StreamReceiver::new(manifest).map_err(PyValueError::new_err)?,
        })
    }
    /// Restore from a checkpoint frame of this run.
    #[staticmethod]
    fn restore(manifest: &str, checkpoint: &[u8]) -> PyResult<Self> {
        let manifest = dexel_manifest(manifest)?;
        Ok(Self {
            inner: camlib::stream::StreamReceiver::from_checkpoint(
                manifest,
                checkpoint,
                camlib::stream::CHECKPOINT_FRAME_LIMIT,
            )
            .map_err(PyValueError::new_err)?,
        })
    }
    /// Commit one move frame; returns (motion JSON, changed rays).
    fn accept(&mut self, frame: &[u8]) -> PyResult<(String, usize)> {
        let chunk = camlib::stream::decode_dexel_chunk_frame(
            self.inner.manifest(),
            frame,
            camlib::stream::DEFAULT_FRAME_LIMIT,
        )
        .map_err(PyValueError::new_err)?;
        let changed = chunk.changes.len();
        let motion = self
            .inner
            .accept(frame, camlib::stream::DEFAULT_FRAME_LIMIT)
            .map_err(PyValueError::new_err)?;
        Ok((
            serde_json::to_string(&motion).map_err(|e| PyValueError::new_err(e.to_string()))?,
            changed,
        ))
    }
    fn accept_end(&mut self, frame: &[u8]) -> PyResult<()> {
        self.inner
            .accept_end(frame, camlib::stream::DEFAULT_FRAME_LIMIT)
            .map_err(PyValueError::new_err)
    }
    fn checkpoint<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let bytes = self
            .inner
            .checkpoint(camlib::stream::CHECKPOINT_FRAME_LIMIT)
            .map_err(PyValueError::new_err)?;
        Ok(PyBytes::new(py, &bytes))
    }
    fn copy(&self) -> Self {
        self.clone()
    }
    #[getter]
    fn completed(&self) -> u64 {
        self.inner.completed()
    }
    #[getter]
    fn finished(&self) -> bool {
        self.inner.is_finished()
    }
    #[getter]
    fn position(&self) -> (f64, f64, f64) {
        let p = self.inner.position();
        (p[0], p[1], p[2])
    }
    /// The tool in the spindle at the current position (None: none yet).
    #[getter]
    fn last_tool(&self) -> Option<String> {
        self.inner.last_tool().map(str::to_string)
    }
    fn volume_json(&self) -> PyResult<String> {
        dexel_volume_json(self.stock()?)
    }
    fn view_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        Ok(dexel_view_bytes(py, self.stock()?))
    }
    fn ray(&self, axis: usize, index: usize) -> PyResult<(Intervals, Intervals)> {
        dexel_ray(self.stock()?, axis, index)
    }
    fn rays<'py>(
        &self,
        py: Python<'py>,
        axis: usize,
    ) -> PyResult<(Bound<'py, PyBytes>, Bound<'py, PyBytes>)> {
        dexel_rays(py, self.stock()?, axis)
    }
    fn grid_json(&self) -> PyResult<String> {
        dexel_grid_json(self.stock()?)
    }
    fn compare_solid(
        &self,
        py: Python<'_>,
        triangles: &[u8],
        surface_error_mm: f64,
        tolerance_mm: f64,
    ) -> PyResult<String> {
        dexel_compare_solid(py, self.stock()?, triangles, surface_error_mm, tolerance_mm)
    }
    /// The stock after the first `fraction` of the next move (given as its
    /// committed frame); this receiver is unchanged.
    fn preview(&self, py: Python<'_>, frame: &[u8], fraction: f64) -> PyResult<DexelView> {
        let chunk = camlib::stream::decode_dexel_chunk_frame(
            self.inner.manifest(),
            frame,
            camlib::stream::DEFAULT_FRAME_LIMIT,
        )
        .map_err(PyValueError::new_err)?;
        if chunk.sequence != self.inner.completed() {
            return Err(PyValueError::new_err("preview frame is not the next move"));
        }
        let stock = py
            .detach(|| {
                self.inner
                    .preview_dexel(&chunk.motion, fraction, &AtomicBool::new(false))
            })
            .map_err(PyValueError::new_err)?;
        Ok(DexelView { stock })
    }
}

#[pyclass]
struct PostCancellation {
    flag: std::sync::Arc<AtomicBool>,
}
#[pymethods]
impl PostCancellation {
    #[new]
    fn new() -> Self {
        Self {
            flag: std::sync::Arc::new(AtomicBool::new(false)),
        }
    }
    fn cancel(&self) {
        self.flag.store(true, std::sync::atomic::Ordering::Relaxed);
    }
    fn is_cancelled(&self) -> bool {
        self.flag.load(std::sync::atomic::Ordering::Relaxed)
    }
}
#[pyfunction]
#[pyo3(signature=(program_json, settings_json, cancel=None))]
fn schedule_motion<'py>(
    py: Python<'py>,
    program_json: &str,
    settings_json: &str,
    cancel: Option<PyRef<'_, PostCancellation>>,
) -> PyResult<Bound<'py, PyBytes>> {
    if program_json.len() > 256 * 1024 * 1024 || settings_json.len() > 64 * 1024 {
        return Err(PyValueError::new_err("motion scheduling input byte cap"));
    }
    let flag = cancel.map(|c| c.flag.clone()).unwrap_or_default();
    let bytes = py.detach(|| {
        if flag.load(std::sync::atomic::Ordering::Relaxed) {
            return Err(PyValueError::new_err(
                "motion scheduling cancelled; no result committed",
            ));
        }
        let program = serde_json::from_str(program_json)
            .map_err(|e| PyValueError::new_err(format!("program: {e}")))?;
        let settings = serde_json::from_str(settings_json)
            .map_err(|e| PyValueError::new_err(format!("policy: {e}")))?;
        let report = camlib::dynamics::schedule(&program, settings, &*flag)
            .map_err(PyValueError::new_err)?;
        let bytes =
            serde_json::to_vec(&report).map_err(|e| PyValueError::new_err(e.to_string()))?;
        if flag.load(std::sync::atomic::Ordering::Relaxed) {
            return Err(PyValueError::new_err(
                "motion scheduling cancelled; no result committed",
            ));
        }
        Ok(bytes)
    })?;
    Ok(PyBytes::new(py, &bytes))
}

#[pyfunction]
#[pyo3(signature = (program_json, stock_json, policy_json, cancel=None))]
fn schedule_engagement<'py>(
    py: Python<'py>,
    program_json: &str,
    stock_json: &str,
    policy_json: &str,
    cancel: Option<PyRef<'_, PostCancellation>>,
) -> PyResult<Bound<'py, PyBytes>> {
    if program_json
        .len()
        .saturating_add(stock_json.len())
        .saturating_add(policy_json.len())
        > 256 * 1024 * 1024
    {
        return Err(PyValueError::new_err("engagement input byte cap"));
    }
    let flag = cancel.map(|c| c.flag.clone()).unwrap_or_default();
    let bytes = py.detach(|| {
        let program = serde_json::from_str(program_json)
            .map_err(|e| PyValueError::new_err(format!("program: {e}")))?;
        let (stock, initial, sweep): (
            camlib::stock::StockSpec,
            Option<camlib::stock::InitialStock>,
            camlib::stock::SweepOptions,
        ) = serde_json::from_str(stock_json)
            .map_err(|e| PyValueError::new_err(format!("stock: {e}")))?;
        let policy = serde_json::from_str(policy_json)
            .map_err(|e| PyValueError::new_err(format!("feed policy: {e}")))?;
        let report = camlib::feed_policy::apply(&program, stock, initial, sweep, &policy, &*flag)
            .map_err(PyValueError::new_err)?;
        if flag.load(std::sync::atomic::Ordering::Relaxed) {
            return Err(PyValueError::new_err("engagement feed analysis cancelled"));
        }
        serde_json::to_vec(&report).map_err(|e| PyValueError::new_err(e.to_string()))
    })?;
    Ok(PyBytes::new(py, &bytes))
}
#[pyfunction]
#[pyo3(signature = (kind, request_json, cancel=None))]
fn prepare_geometry<'py>(
    py: Python<'py>,
    kind: &str,
    request_json: &str,
    cancel: Option<PyRef<'_, PostCancellation>>,
) -> PyResult<Bound<'py, PyBytes>> {
    if request_json.len() > 256 * 1024 * 1024 {
        return Err(PyValueError::new_err("preparation input byte cap"));
    }
    let flag = cancel.map(|c| c.flag.clone()).unwrap_or_default();
    let bytes = py.detach(|| {
        let value = match kind {
            "reach" => {
                let request = serde_json::from_str(request_json)
                    .map_err(|e| PyValueError::new_err(format!("reach request: {e}")))?;
                serde_json::to_vec(
                    &camlib::preparation::reach(&request, &*flag).map_err(PyValueError::new_err)?,
                )
            }
            "blank" => {
                let request = serde_json::from_str(request_json)
                    .map_err(|e| PyValueError::new_err(format!("blank request: {e}")))?;
                serde_json::to_vec(
                    &camlib::preparation::enclose_blank(&request, &*flag)
                        .map_err(PyValueError::new_err)?,
                )
            }
            _ => return Err(PyValueError::new_err("unknown preparation query")),
        };
        value.map_err(|e| PyValueError::new_err(e.to_string()))
    })?;
    Ok(PyBytes::new(py, &bytes))
}

/// Generic syntax-independent geometry checker used by ordinary Python posts.
#[pyclass]
struct IntentSession {
    state: camlib::post_intent::IntentChecker,
    initial: Vec<u8>,
}
#[pymethods]
impl IntentSession {
    #[new]
    fn new(header_json: &str, settings_json: &str, lines_json: &str) -> PyResult<Self> {
        if [header_json.len(), settings_json.len(), lines_json.len()]
            .iter()
            .any(|n| *n > 512 * 1024)
        {
            return Err(PyValueError::new_err("generic post header byte cap"));
        }
        let header =
            serde_json::from_str(header_json).map_err(|e| PyValueError::new_err(e.to_string()))?;
        let settings = serde_json::from_str(settings_json)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        let lines: Vec<String> =
            serde_json::from_str(lines_json).map_err(|e| PyValueError::new_err(e.to_string()))?;
        let (state, first) = camlib::post_intent::IntentChecker::begin(&header, settings, &lines)
            .map_err(PyValueError::new_err)?;
        let initial =
            serde_json::to_vec(&first).map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(Self { state, initial })
    }
    fn take_begin<'py>(&mut self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        if self.initial.is_empty() {
            return Err(PyValueError::new_err("generic post begin already consumed"));
        }
        Ok(PyBytes::new(py, &std::mem::take(&mut self.initial)))
    }
    #[pyo3(signature=(cancel=None))]
    fn check_cancel(&self, cancel: Option<PyRef<'_, PostCancellation>>) -> PyResult<()> {
        if cancel.is_some_and(|c| c.flag.load(std::sync::atomic::Ordering::Relaxed)) {
            Err(PyValueError::new_err("post cancelled; batch not committed"))
        } else {
            Ok(())
        }
    }
    fn has_pending(&self) -> bool {
        self.state.has_pending()
    }
    #[pyo3(signature=(events_json,cancel=None))]
    fn preview_source<'py>(
        &self,
        py: Python<'py>,
        events_json: &str,
        cancel: Option<PyRef<'_, PostCancellation>>,
    ) -> PyResult<Bound<'py, PyBytes>> {
        if !self.initial.is_empty() || events_json.len() > 512 * 1024 {
            return Err(PyValueError::new_err(
                "direct preview lifecycle/input byte cap",
            ));
        }
        let events: Vec<camlib::motion::Event> =
            serde_json::from_str(events_json).map_err(|e| PyValueError::new_err(e.to_string()))?;
        let flag = cancel.map(|c| c.flag.clone()).unwrap_or_default();
        let bytes = py
            .detach(|| {
                let result = self.state.preview_source(&events, &*flag)?;
                serde_json::to_vec(&result).map_err(|e| e.to_string())
            })
            .map_err(PyValueError::new_err)?;
        Ok(PyBytes::new(py, &bytes))
    }
    #[pyo3(signature=(event_json,cancel=None,whole=false))]
    fn start(
        &mut self,
        py: Python<'_>,
        event_json: &str,
        cancel: Option<PyRef<'_, PostCancellation>>,
        whole: bool,
    ) -> PyResult<()> {
        if !self.initial.is_empty() || event_json.len() > 512 * 1024 {
            return Err(PyValueError::new_err(
                "generic start lifecycle/input byte cap",
            ));
        }
        let event =
            serde_json::from_str(event_json).map_err(|e| PyValueError::new_err(e.to_string()))?;
        let flag = cancel.map(|c| c.flag.clone()).unwrap_or_default();
        py.detach(|| self.state.start_with(event, whole, &*flag))
            .map_err(PyValueError::new_err)
    }
    /// Start several source events (each with its own `whole` flag) in one
    /// call, all or none: on an error nothing is started.
    #[pyo3(signature=(events_json, wholes, cancel=None))]
    fn start_many(
        &mut self,
        py: Python<'_>,
        events_json: &str,
        wholes: Vec<bool>,
        cancel: Option<PyRef<'_, PostCancellation>>,
    ) -> PyResult<()> {
        if !self.initial.is_empty() || events_json.len() > 512 * 1024 || wholes.len() > 64 {
            return Err(PyValueError::new_err(
                "generic start lifecycle/input byte cap",
            ));
        }
        let events: Vec<camlib::motion::Event> =
            serde_json::from_str(events_json).map_err(|e| PyValueError::new_err(e.to_string()))?;
        if events.len() != wholes.len() {
            return Err(PyValueError::new_err("one whole flag per event"));
        }
        let flag = cancel.map(|c| c.flag.clone()).unwrap_or_default();
        py.detach(|| {
            let mut next = self.state.clone();
            for (event, whole) in events.into_iter().zip(wholes) {
                next.start_with(event, whole, &*flag)?;
            }
            self.state = next;
            Ok::<(), String>(())
        })
        .map_err(PyValueError::new_err)
    }
    #[pyo3(signature=(limit=64,cancel=None))]
    fn preview<'py>(
        &self,
        py: Python<'py>,
        limit: usize,
        cancel: Option<PyRef<'_, PostCancellation>>,
    ) -> PyResult<Bound<'py, PyBytes>> {
        if !self.initial.is_empty() {
            return Err(PyValueError::new_err("consume begin before preview"));
        }
        let flag = cancel.map(|c| c.flag.clone()).unwrap_or_default();
        let out = py
            .detach(|| self.state.preview(limit, &*flag))
            .map_err(PyValueError::new_err)?;
        let bytes = serde_json::to_vec(&out).map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(PyBytes::new(py, &bytes))
    }
    #[pyo3(signature=(ranges_json,decoded_json,blocks_json,cancel=None))]
    fn check_lowered<'py>(
        &mut self,
        py: Python<'py>,
        ranges_json: &str,
        decoded_json: &str,
        blocks_json: &str,
        cancel: Option<PyRef<'_, PostCancellation>>,
    ) -> PyResult<Bound<'py, PyBytes>> {
        if !self.initial.is_empty()
            || [ranges_json.len(), decoded_json.len(), blocks_json.len()]
                .iter()
                .any(|n| *n > 512 * 1024)
        {
            return Err(PyValueError::new_err(
                "generic lowered lifecycle/input byte cap",
            ));
        }
        let ranges: Vec<[f64; 2]> =
            serde_json::from_str(ranges_json).map_err(|e| PyValueError::new_err(e.to_string()))?;
        let decoded: Vec<Vec<camlib::motion::Event>> =
            serde_json::from_str(decoded_json).map_err(|e| PyValueError::new_err(e.to_string()))?;
        let blocks: Vec<Vec<String>> =
            serde_json::from_str(blocks_json).map_err(|e| PyValueError::new_err(e.to_string()))?;
        let flag = cancel.map(|c| c.flag.clone()).unwrap_or_default();
        let out = py
            .detach(|| self.state.check_lowered(&ranges, &decoded, &blocks, &*flag))
            .map_err(PyValueError::new_err)?;
        let bytes = serde_json::to_vec(&out).map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(PyBytes::new(py, &bytes))
    }
    #[pyo3(signature=(events_json,decoded_json,blocks_json,cancel=None))]
    fn consume<'py>(
        &mut self,
        py: Python<'py>,
        events_json: &str,
        decoded_json: &str,
        blocks_json: &str,
        cancel: Option<PyRef<'_, PostCancellation>>,
    ) -> PyResult<Bound<'py, PyBytes>> {
        if !self.initial.is_empty()
            || [events_json.len(), decoded_json.len(), blocks_json.len()]
                .iter()
                .any(|n| *n > 512 * 1024)
        {
            return Err(PyValueError::new_err(
                "generic post lifecycle/input byte cap",
            ));
        }
        let events: Vec<camlib::motion::Event> =
            serde_json::from_str(events_json).map_err(|e| PyValueError::new_err(e.to_string()))?;
        let decoded: Vec<Vec<camlib::motion::Event>> =
            serde_json::from_str(decoded_json).map_err(|e| PyValueError::new_err(e.to_string()))?;
        let blocks: Vec<Vec<String>> =
            serde_json::from_str(blocks_json).map_err(|e| PyValueError::new_err(e.to_string()))?;
        let flag = cancel.map(|c| c.flag.clone()).unwrap_or_default();
        let bytes = py
            .detach(|| {
                let out = self.state.consume(&events, &decoded, &blocks, &*flag)?;
                serde_json::to_vec(&out).map_err(|e| e.to_string())
            })
            .map_err(PyValueError::new_err)?;
        Ok(PyBytes::new(py, &bytes))
    }
    /// `consume` for a batch of non-tapping motion events through the
    /// unmodified default post: formats and independently decodes them with
    /// `formatter`, then runs the same checks. Returns the chunk, and the
    /// decoder's position and plane after it.
    #[pyo3(signature=(events_json,formatter,position,units,plane,tool,cancel=None))]
    #[allow(clippy::too_many_arguments, clippy::type_complexity)]
    fn consume_default<'py>(
        &mut self,
        py: Python<'py>,
        events_json: &str,
        formatter: PyRef<'_, DefaultMotionPost>,
        position: (f64, f64, f64),
        units: f64,
        plane: Option<&str>,
        tool: &str,
        cancel: Option<PyRef<'_, PostCancellation>>,
    ) -> PyResult<(Bound<'py, PyBytes>, (f64, f64, f64), Option<String>)> {
        use camlib::motion::{Event, Motion, MoveKind};
        if !self.initial.is_empty() || events_json.len() > 512 * 1024 {
            return Err(PyValueError::new_err(
                "generic post lifecycle/input byte cap",
            ));
        }
        let events: Vec<Event> =
            serde_json::from_str(events_json).map_err(|e| PyValueError::new_err(e.to_string()))?;
        let motions = events
            .iter()
            .map(|e| match e {
                Event::Motion { motion } => Ok(motion.clone()),
                _ => Err(PyValueError::new_err(
                    "the default batch takes motion events only",
                )),
            })
            .collect::<PyResult<Vec<Motion>>>()?;
        let names = ["xy", "xz", "yz"];
        let plane = plane
            .map(|p| {
                names
                    .iter()
                    .position(|n| *n == p)
                    .ok_or_else(|| PyValueError::new_err("invalid decoder plane"))
            })
            .transpose()?;
        let (blocks, decoded, end, plane) = formatter.format_decode(
            &motions,
            [position.0, position.1, position.2],
            units,
            plane,
        )?;
        let decoded: Vec<Vec<Event>> = motions
            .iter()
            .zip(decoded)
            .map(|(m, d)| {
                d.map(|(curve, feed, rapid)| Event::Motion {
                    motion: Motion {
                        id: m.id,
                        operation: m.operation.clone(),
                        tool: tool.to_string(),
                        // A rapid while the spindle is oriented is that
                        // shift (the program validator requires the stop).
                        kind: if rapid && m.kind != MoveKind::Shift {
                            MoveKind::Rapid
                        } else {
                            m.kind
                        },
                        curve,
                        feed_mm_min: feed,
                    },
                })
                .into_iter()
                .collect()
            })
            .collect();
        let flag = cancel.map(|c| c.flag.clone()).unwrap_or_default();
        let bytes = py
            .detach(|| {
                let out = self.state.consume(&events, &decoded, &blocks, &*flag)?;
                serde_json::to_vec(&out).map_err(|e| e.to_string())
            })
            .map_err(PyValueError::new_err)?;
        Ok((
            PyBytes::new(py, &bytes),
            (end[0], end[1], end[2]),
            plane.map(|i| names[i].to_string()),
        ))
    }
    #[pyo3(signature=(lines_json,cancel=None))]
    fn finish<'py>(
        &mut self,
        py: Python<'py>,
        lines_json: &str,
        cancel: Option<PyRef<'_, PostCancellation>>,
    ) -> PyResult<Bound<'py, PyBytes>> {
        if !self.initial.is_empty() || lines_json.len() > 32768 {
            return Err(PyValueError::new_err("generic finish byte cap"));
        }
        let lines: Vec<String> =
            serde_json::from_str(lines_json).map_err(|e| PyValueError::new_err(e.to_string()))?;
        let flag = cancel.map(|c| c.flag.clone()).unwrap_or_default();
        let out = py
            .detach(|| self.state.finish(&lines, &*flag))
            .map_err(PyValueError::new_err)?;
        let bytes = serde_json::to_vec(&out).map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(PyBytes::new(py, &bytes))
    }
}

#[pyfunction]
fn format_post_words(
    addresses: &str,
    values: Vec<f64>,
    unit: f64,
    decimals: usize,
) -> PyResult<Vec<String>> {
    camlib::post_decode::format_words(addresses, &values, unit, decimals)
        .map_err(PyValueError::new_err)
}

type MotionWords = (
    u8,
    (f64, f64, f64),
    Option<(f64, f64, f64)>,
    f64,
    Option<f64>,
    bool,
);

/// Post a setup from its JSON job with the Rust default post
/// (`camlib::post_job`); returns the JSON report.
#[pyfunction]
#[pyo3(signature=(input_json, cancel=None))]
fn post_job(
    py: Python<'_>,
    input_json: &str,
    cancel: Option<PyRef<'_, PostCancellation>>,
) -> PyResult<String> {
    let flag = cancel.map(|c| c.flag.clone()).unwrap_or_default();
    py.detach(|| camlib::post_job::run(input_json, &*flag))
        .map_err(PyValueError::new_err)
}

/// Post an explicit program's events with the Rust default post
/// (`camlib::gcode::post_program`); returns JSON.
#[pyfunction]
#[pyo3(signature=(input_json, cancel=None))]
fn post_program(
    py: Python<'_>,
    input_json: &str,
    cancel: Option<PyRef<'_, PostCancellation>>,
) -> PyResult<String> {
    let flag = cancel.map(|c| c.flag.clone()).unwrap_or_default();
    py.detach(|| camlib::gcode::post_program(input_json, &*flag))
        .map_err(PyValueError::new_err)
}

/// The highest spindle speed along a move at constant surface speed (the
/// engine's own bound, so posts and decoders agree with the validator).
#[pyfunction]
fn css_rpm_bound(curve_json: &str, m_min: f64, max_rpm: f64) -> PyResult<f64> {
    if curve_json.len() > 4096 {
        return Err(PyValueError::new_err("curve input exceeds 4096 bytes"));
    }
    let curve: camlib::motion::Curve =
        serde_json::from_str(curve_json).map_err(|e| PyValueError::new_err(e.to_string()))?;
    Ok(camlib::motion::css_rpm_bound(&curve, m_min, max_rpm))
}

/// The moves of a probing cycle (`camlib::probe::ProbeCycle::legs`) from
/// its start, for posts that write the cycle as controller blocks: (kind,
/// end, feed or None). `radius_mm` is the stylus ball radius, or the
/// measured cutter's for tool setting. Decoders must not use this: they read
/// the moves back from the written blocks.
#[pyfunction]
#[allow(clippy::type_complexity)]
fn probe_legs(
    cycle_json: &str,
    radius_mm: f64,
) -> PyResult<((f64, f64, f64), Vec<(String, (f64, f64, f64), Option<f64>)>)> {
    if cycle_json.len() > 16384 {
        return Err(PyValueError::new_err(
            "probe cycle input exceeds 16384 bytes",
        ));
    }
    let cycle: camlib::probe::ProbeCycle =
        serde_json::from_str(cycle_json).map_err(|e| PyValueError::new_err(e.to_string()))?;
    let start = cycle.start(radius_mm).map_err(PyValueError::new_err)?;
    let legs = cycle.legs(radius_mm).map_err(PyValueError::new_err)?;
    Ok((
        (start[0], start[1], start[2]),
        legs.into_iter()
            .map(|l| {
                let kind = serde_json::to_value(l.kind)
                    .ok()
                    .and_then(|v| v.as_str().map(str::to_string))
                    .unwrap_or_default();
                (kind, (l.end[0], l.end[1], l.end[2]), l.feed_mm_min)
            })
            .collect(),
    ))
}

#[pyfunction]
#[pyo3(signature = (curve_json, start, unit, decimals, geometry_tolerance, output_tolerance, radius_tolerance, radius_relative_tolerance=0.0))]
#[allow(clippy::too_many_arguments)]
fn round_circular_words(
    curve_json: &str,
    start: (f64, f64, f64),
    unit: f64,
    decimals: usize,
    geometry_tolerance: f64,
    output_tolerance: f64,
    radius_tolerance: f64,
    radius_relative_tolerance: f64,
) -> PyResult<(Vec<f64>, Vec<f64>)> {
    if curve_json.len() > 4096 {
        return Err(PyValueError::new_err(
            "circular rounding input exceeds 4096 bytes",
        ));
    }
    let curve =
        serde_json::from_str(curve_json).map_err(|e| PyValueError::new_err(e.to_string()))?;
    let (end, offsets) = camlib::post_rounding::circular_words(
        &curve,
        [start.0, start.1, start.2],
        unit,
        decimals,
        geometry_tolerance,
        output_tolerance,
        camlib::post_rounding::RadiusTolerance {
            absolute_mm: radius_tolerance,
            relative: radius_relative_tolerance,
        },
    )
    .map_err(PyValueError::new_err)?;
    Ok((end.to_vec(), offsets.to_vec()))
}
#[pyclass]
#[derive(Clone)]
struct MotionDecoder {
    decoder: camlib::post_decode::Decoder,
}
#[pymethods]
impl MotionDecoder {
    #[new]
    fn new(config_json: &str) -> PyResult<Self> {
        if config_json.len() > 512 * 1024 {
            return Err(PyValueError::new_err("motion decoder config byte cap"));
        }
        let config =
            serde_json::from_str(config_json).map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(Self {
            decoder: camlib::post_decode::Decoder::new(config).map_err(PyValueError::new_err)?,
        })
    }
    fn __deepcopy__(&self, _memo: &Bound<'_, PyAny>) -> Self {
        self.clone()
    }
    fn decode(
        &self,
        line: &str,
        start: (f64, f64, f64),
        units: Option<f64>,
        plane: Option<&str>,
    ) -> PyResult<Option<MotionWords>> {
        let plane = match plane {
            None => None,
            Some("xy") => Some(camlib::motion::Plane::Xy),
            Some("xz") => Some(camlib::motion::Plane::Xz),
            Some("yz") => Some(camlib::motion::Plane::Yz),
            _ => return Err(PyValueError::new_err("invalid decoder plane")),
        };
        let result = self
            .decoder
            .decode(line, [start.0, start.1, start.2], units, plane)
            .map_err(PyValueError::new_err)?;
        let point = |p: camlib::motion::Vec3| (p[0], p[1], p[2]);
        Ok(result.map(|d| {
            let (kind, end, center, sweep) = match d.curve {
                camlib::motion::Curve::Line { end, .. } => (0, end, None, 0.),
                camlib::motion::Curve::Arc {
                    end,
                    center,
                    sweep_rad,
                    ..
                } => (1, end, Some(center), sweep_rad),
                camlib::motion::Curve::Helix {
                    end,
                    center,
                    sweep_rad,
                    ..
                } => (2, end, Some(center), sweep_rad),
            };
            (kind, point(end), center.map(point), sweep, d.feed, d.rapid)
        }))
    }
}

/// Native mirror of the unmodified default `GCodePost.on_motion` (and
/// `circular_values`/`words`) plus independent decoding of the written blocks
/// with the configured motion decoder. Used only when a post and decoder are
/// exactly the defaults; the native intent session still checks every
/// decoded motion against its source.
#[pyclass]
struct DefaultMotionPost {
    decoder: camlib::post_decode::Decoder,
    rapid: Option<String>,
    linear: Option<String>,
    arc_cw: Option<String>,
    arc_ccw: Option<String>,
    planes: [Option<String>; 3],
    allowed_planes: Vec<String>,
    axes: [String; 3],
    centers: [String; 3],
    feed: String,
    unit: f64,
    decimals: usize,
    geometry_tolerance: f64,
    output_tolerance: f64,
    radius_tolerance: camlib::post_rounding::RadiusTolerance,
}
#[derive(serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct DefaultMotionConfig {
    decoder: camlib::post_decode::Config,
    codes: std::collections::HashMap<String, String>,
    allowed_planes: Vec<String>,
    unit: f64,
    decimals: usize,
    geometry_tolerance: f64,
    output_tolerance: f64,
}
#[pymethods]
impl DefaultMotionPost {
    #[new]
    fn new(config_json: &str) -> PyResult<Self> {
        if config_json.len() > 512 * 1024 {
            return Err(PyValueError::new_err("default post config byte cap"));
        }
        let c: DefaultMotionConfig =
            serde_json::from_str(config_json).map_err(|e| PyValueError::new_err(e.to_string()))?;
        let code = |k: &str| c.codes.get(k).cloned();
        Ok(Self {
            rapid: code("rapid"),
            linear: code("linear"),
            arc_cw: code("arc_cw"),
            arc_ccw: code("arc_ccw"),
            planes: [code("plane_xy"), code("plane_xz"), code("plane_yz")],
            allowed_planes: c.allowed_planes,
            axes: c.decoder.axes.clone(),
            centers: c.decoder.centers.clone(),
            feed: c.decoder.feed_word.clone(),
            unit: c.unit,
            decimals: c.decimals,
            geometry_tolerance: c.geometry_tolerance,
            output_tolerance: c.output_tolerance,
            radius_tolerance: c.decoder.radius_tolerance(),
            decoder: camlib::post_decode::Decoder::new(c.decoder).map_err(PyValueError::new_err)?,
        })
    }
    /// Blocks for each source motion and its decoded motion words (None when
    /// a linear block rounds to the current position), the decoded end
    /// position and plane. `units` is the decoder's active unit scale.
    #[allow(clippy::type_complexity)]
    fn motions(
        &self,
        motions_json: &str,
        position: (f64, f64, f64),
        units: f64,
        plane: Option<&str>,
    ) -> PyResult<(
        Vec<Vec<String>>,
        Vec<Option<(MotionWords, Option<&'static str>)>>,
        (f64, f64, f64),
        Option<String>,
    )> {
        use camlib::motion::Curve;
        let names = ["xy", "xz", "yz"];
        let motions: Vec<camlib::motion::Motion> =
            serde_json::from_str(motions_json).map_err(|e| PyValueError::new_err(e.to_string()))?;
        let plane = plane
            .map(|p| {
                names
                    .iter()
                    .position(|n| *n == p)
                    .ok_or_else(|| PyValueError::new_err("invalid decoder plane"))
            })
            .transpose()?;
        let (blocks, decoded, position, plane) =
            self.format_decode(&motions, [position.0, position.1, position.2], units, plane)?;
        let point = |p: camlib::motion::Vec3| (p[0], p[1], p[2]);
        let words = decoded
            .into_iter()
            .map(|d| {
                d.map(|(curve, feed, rapid)| {
                    let (kind, center, sweep, named) = match curve {
                        Curve::Line { .. } => (0, None, 0., None),
                        Curve::Arc {
                            center,
                            sweep_rad,
                            plane,
                            ..
                        } => (1, Some(point(center)), sweep_rad, Some(plane)),
                        Curve::Helix {
                            center,
                            sweep_rad,
                            plane,
                            ..
                        } => (2, Some(point(center)), sweep_rad, Some(plane)),
                    };
                    let named = named.map(|p| match p {
                        camlib::motion::Plane::Xy => "xy",
                        camlib::motion::Plane::Xz => "xz",
                        camlib::motion::Plane::Yz => "yz",
                    });
                    (
                        (kind, point(curve.end()), center, sweep, feed, rapid),
                        named,
                    )
                })
            })
            .collect();
        Ok((
            blocks,
            words,
            point(position),
            plane.map(|i| names[i].to_string()),
        ))
    }
}
impl DefaultMotionPost {
    /// Blocks for each source motion, its independently decoded curve, feed
    /// and rapid flag (None when a linear block rounds to the current
    /// position), the decoded end position and plane index.
    #[allow(clippy::type_complexity)]
    fn format_decode(
        &self,
        motions: &[camlib::motion::Motion],
        position: camlib::motion::Vec3,
        units: f64,
        plane: Option<usize>,
    ) -> PyResult<(
        Vec<Vec<String>>,
        Vec<Option<(camlib::motion::Curve, Option<f64>, bool)>>,
        camlib::motion::Vec3,
        Option<usize>,
    )> {
        use camlib::motion::{Curve, MoveKind, Plane};
        let names = ["xy", "xz", "yz"];
        let mut plane = plane;
        // The plane the program has selected: the decoder's on entry (what
        // the blocks written so far select), then each plane code written
        // here. Plane selection is modal (G17/G18/G19, modal group 2:
        // LinuxCNC G-code reference, "Modal Groups"), so a code is written
        // only for a change (#204). The decoding below keeps its own plane
        // from the blocks it reads; the session checks each decoded arc
        // against its source, so a missing code is refused.
        let mut selected = plane;
        let mut position = position;
        let declared = |code: &Option<String>, effect: &str| {
            code.clone()
                .ok_or_else(|| PyValueError::new_err(format!("dialect does not declare {effect}")))
        };
        let (mut blocks, mut decoded) = (Vec::with_capacity(motions.len()), Vec::new());
        for m in motions {
            if m.kind == MoveKind::Tap {
                return Err(PyValueError::new_err(
                    "tapping is not a default motion block",
                ));
            }
            let mut lines = vec![];
            let (code, axes, circular) = match &m.curve {
                Curve::Line { .. } => (
                    if m.kind.is_rapid() && m.feed_mm_min.is_none() {
                        declared(&self.rapid, "rapid")?
                    } else {
                        declared(&self.linear, "linear")?
                    },
                    vec![0, 1, 2],
                    None,
                ),
                Curve::Arc {
                    plane: p,
                    sweep_rad,
                    ..
                }
                | Curve::Helix {
                    plane: p,
                    sweep_rad,
                    ..
                } => {
                    let index = match p {
                        Plane::Xy => 0,
                        Plane::Xz => 1,
                        Plane::Yz => 2,
                    };
                    if selected != Some(index) {
                        lines.push(declared(
                            &self.planes[index],
                            &format!("plane_{}", names[index]),
                        )?);
                        selected = Some(index);
                    }
                    let [u, v, normal] = p.axes();
                    let axes = if matches!(m.curve, Curve::Helix { .. }) {
                        vec![0, 1, 2]
                    } else {
                        (0..3).filter(|i| *i != normal).collect()
                    };
                    let code = if *sweep_rad > 0. {
                        declared(&self.arc_ccw, "arc_ccw")?
                    } else {
                        declared(&self.arc_cw, "arc_cw")?
                    };
                    (code, axes, Some((u, v)))
                }
            };
            let mut addresses = String::new();
            let mut values = vec![];
            let end = m.curve.end();
            if let Some((u, v)) = circular {
                let (e, offsets) = camlib::post_rounding::circular_words(
                    &m.curve,
                    position,
                    self.unit,
                    self.decimals,
                    self.geometry_tolerance,
                    self.output_tolerance,
                    self.radius_tolerance,
                )
                .map_err(|error| {
                    PyValueError::new_err(format!(
                        "{error}. Arc {:?} -> {:?} cannot be written as G2/G3 at {} decimals. Use lowering=LoweringOptions(native_arcs=True) to emit checked line segments for only such arcs, or a machine with more decimals if the controller accepts that precision.",
                        m.curve.start(),
                        end,
                        self.decimals
                    ))
                })?;
                for &i in &axes {
                    addresses.push_str(&self.axes[i]);
                    values.push(e[i]);
                }
                addresses.push_str(&self.centers[u]);
                addresses.push_str(&self.centers[v]);
                values.extend([offsets[u], offsets[v]]);
            } else {
                // A straight vertical move keeps the controller's current XY
                // when the block stays within the output tolerance of the
                // source, its XY offset plus half a word of Z rounding (an
                // arc's representable end may differ by a few words): writing
                // the source XY instead would add a sideways step to a move
                // leaving the surface it touches. The check still compares
                // the decoded block with the source motion.
                let vertical = matches!(m.curve, Curve::Line { start, end }
                    if start[0] == end[0] && start[1] == end[1])
                    && (position[0] - end[0]).hypot(position[1] - end[1])
                        + 0.5 * self.unit * 10f64.powi(-(self.decimals as i32))
                        <= self.output_tolerance;
                for &i in &axes {
                    addresses.push_str(&self.axes[i]);
                    values.push(if vertical && i < 2 {
                        position[i]
                    } else {
                        end[i]
                    });
                }
            }
            if let Some(f) = m.feed_mm_min {
                addresses.push_str(&self.feed);
                values.push(f);
            }
            let words =
                camlib::post_decode::format_words(&addresses, &values, self.unit, self.decimals)
                    .map_err(PyValueError::new_err)?;
            let line = std::iter::once(code)
                .chain(words.into_iter().filter(|w| !w.is_empty()))
                .collect::<Vec<_>>()
                .join(" ");
            if line.len() > 1024
                || !line
                    .bytes()
                    .all(|b| (0x20..0x7f).contains(&b) || b == b'\t')
            {
                return Err(PyValueError::new_err(
                    "post hook output exceeds block/ASCII/1024byte limit",
                ));
            }
            lines.push(line);
            // Independent decoding of the written blocks.
            let mut result = None;
            for line in &lines {
                if let Some(index) = self
                    .planes
                    .iter()
                    .position(|c| c.as_deref() == Some(line.as_str()))
                {
                    if !self.allowed_planes.is_empty()
                        && !self.allowed_planes.iter().any(|p| p == names[index])
                    {
                        return Err(PyValueError::new_err(
                            "plane not supported by machine definition",
                        ));
                    }
                    plane = Some(index);
                    continue;
                }
                let decoded_plane = plane.map(|i| [Plane::Xy, Plane::Xz, Plane::Yz][i]);
                let d = self
                    .decoder
                    .decode(line, position, Some(units), decoded_plane)
                    .map_err(PyValueError::new_err)?
                    .ok_or_else(|| {
                        PyValueError::new_err("default motion block did not decode as one motion")
                    })?;
                let e = d.curve.end();
                let same = matches!(d.curve, Curve::Line { .. }) && e == position;
                position = e;
                result = (!same).then_some((d.curve, d.feed, d.rapid));
            }
            blocks.push(lines);
            decoded.push(result);
        }
        Ok((blocks, decoded, position, plane))
    }
}

/// GRBL 1.1h's own reading of a numeric word (binary32).
#[pyfunction]
fn grbl_number(text: &str) -> PyResult<f64> {
    camlib::grbl::grbl_number(text).map_err(PyValueError::new_err)
}
/// GRBL work coordinate of an axis word (binary32 machine space).
#[pyfunction]
fn grbl_axis_mm(value: f64, unit: f64, origin_mm: f64) -> f64 {
    camlib::grbl::axis_mm(value, unit, origin_mm)
}
/// GRBL feed in mm/min (binary32 scaling).
#[pyfunction]
fn grbl_feed_mm_min(value: f64, unit: f64) -> f64 {
    camlib::grbl::feed_mm_min(value, unit)
}
/// GRBL work coordinate of the initial machine position (binary32).
#[pyfunction]
fn grbl_start_mm(machine_mm: f64, origin_mm: f64) -> f64 {
    camlib::grbl::start_mm(machine_mm, origin_mm)
}

/// The region/geometry interface (camlib.Engine): one stateful session.
#[pyclass]
struct RegionSession {
    session: camlib::session::Session,
}
#[pymethods]
impl RegionSession {
    #[new]
    fn new() -> Self {
        Self {
            session: camlib::session::Session::default(),
        }
    }
    /// One JSON request; returns the response data as JSON.
    fn request(&mut self, py: Python<'_>, request: &str) -> PyResult<String> {
        let session = &mut self.session;
        let data = py
            .detach(|| session.handle(request))
            .map_err(PyValueError::new_err)?;
        serde_json::to_string(&data).map_err(|e| PyValueError::new_err(e.to_string()))
    }
}

/// Crash checking uses the same authoritative native stock replay as playback.
#[pyclass]
struct MachineCrashSession {
    inner: camlib::machine::Session,
    failed: bool,
}
#[derive(serde::Deserialize)]
struct MachineDecodedChunk {
    checked: Vec<camlib::post::Checked>,
}
#[pymethods]
impl MachineCrashSession {
    #[new]
    fn new(py: Python<'_>, model: &str, manifest: &str, definition: &str) -> PyResult<Self> {
        if model.len() > 2 * 1024 * 1024
            || manifest.len() > 16 * 1024 * 1024
            || definition.len() > 4 * 1024 * 1024
        {
            return Err(PyValueError::new_err("machine check input byte cap"));
        }
        let inner = py
            .detach(|| {
                let model =
                    serde_json::from_str(model).map_err(|e| format!("machine model: {e}"))?;
                let manifest = serde_json::from_str(manifest)
                    .map_err(|e| format!("machine stock manifest: {e}"))?;
                let definition = serde_json::from_str(definition)
                    .map_err(|e| format!("machine post definition: {e}"))?;
                camlib::machine::Session::new(model, manifest, &definition, &AtomicBool::new(false))
            })
            .map_err(PyValueError::new_err)?;
        Ok(Self {
            inner,
            failed: false,
        })
    }
    #[pyo3(signature=(chunk, cancel=None))]
    fn push(
        &mut self,
        py: Python<'_>,
        chunk: &str,
        cancel: Option<PyRef<'_, PostCancellation>>,
    ) -> PyResult<()> {
        if self.failed {
            return Err(PyValueError::new_err("machine session is failed"));
        }
        self.failed = true;
        if chunk.len() > 512 * 1024 {
            return Err(PyValueError::new_err("machine decoded chunk byte cap"));
        }
        let flag = cancel.map(|c| c.flag.clone()).unwrap_or_default();
        py.detach(|| {
            // Consume actual decoded controls, never normalized source controls.
            let chunk: MachineDecodedChunk =
                serde_json::from_str(chunk).map_err(|e| e.to_string())?;
            for check in chunk.checked {
                for event in check.control_effects.unwrap_or_default() {
                    self.inner.event(event, check.event_index, &*flag)?;
                }
                if let Some(motion) = check.motion {
                    self.inner.event(
                        camlib::motion::Event::Motion { motion },
                        check.event_index,
                        &*flag,
                    )?;
                }
            }
            Ok::<_, String>(())
        })
        .map_err(PyValueError::new_err)?;
        self.failed = false;
        Ok(())
    }
    fn incomplete(&self, reason: String) -> PyResult<String> {
        serde_json::to_string(&self.inner.incomplete(reason))
            .map_err(|e| PyValueError::new_err(e.to_string()))
    }
    #[pyo3(signature=(motions, cancel=None))]
    fn finish(
        &mut self,
        py: Python<'_>,
        motions: u64,
        cancel: Option<PyRef<'_, PostCancellation>>,
    ) -> PyResult<String> {
        if self.failed {
            return Err(PyValueError::new_err("machine session is failed"));
        }
        let flag = cancel.map(|c| c.flag.clone()).unwrap_or_default();
        py.detach(|| {
            let report = self.inner.finish(motions, &*flag)?;
            serde_json::to_string(report).map_err(|e| e.to_string())
        })
        .map_err(PyValueError::new_err)
    }
}

#[pymodule]
fn _native(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<RegionSession>()?;
    module.add_class::<MachineCrashSession>()?;
    module.add_class::<TurnReceiver>()?;
    module.add_class::<DexelReceiver>()?;
    module.add_class::<DexelView>()?;
    module.add_function(wrap_pyfunction!(grbl_number, module)?)?;
    module.add_function(wrap_pyfunction!(grbl_axis_mm, module)?)?;
    module.add_function(wrap_pyfunction!(grbl_feed_mm_min, module)?)?;
    module.add_function(wrap_pyfunction!(grbl_start_mm, module)?)?;
    module.add_function(wrap_pyfunction!(apply_delta, module)?)?;
    module.add_function(wrap_pyfunction!(turn_proxy_tools, module)?)?;
    module.add_function(wrap_pyfunction!(turning_iso_insert, module)?)?;
    module.add_function(wrap_pyfunction!(turning_iso_code, module)?)?;
    module.add_function(wrap_pyfunction!(turning_holder_approach, module)?)?;
    module.add_function(wrap_pyfunction!(turning_iso_insert_body, module)?)?;
    module.add_function(wrap_pyfunction!(turning_iso_size_tolerance, module)?)?;
    module.add_function(wrap_pyfunction!(turning_rectangle_holder, module)?)?;
    module.add_function(wrap_pyfunction!(turning_validate_tool, module)?)?;
    module.add_function(wrap_pyfunction!(holder_envelope_from_mesh, module)?)?;
    module.add_function(wrap_pyfunction!(schedule_motion, module)?)?;
    module.add_function(wrap_pyfunction!(schedule_engagement, module)?)?;
    module.add_function(wrap_pyfunction!(validate_heights, module)?)?;
    module.add_function(wrap_pyfunction!(decode_committed_chunk, module)?)?;
    module.add_function(wrap_pyfunction!(accept_motion_frames, module)?)?;
    module.add_function(wrap_pyfunction!(volume, module)?)?;
    module.add_function(wrap_pyfunction!(display_patch, module)?)?;
    module.add_function(wrap_pyfunction!(initialise_heights, module)?)?;
    module.add_function(wrap_pyfunction!(initialise_declared_stock, module)?)?;
    module.add_function(wrap_pyfunction!(prepare_geometry, module)?)?;
    module.add_function(wrap_pyfunction!(initialise_stock, module)?)?;
    module.add_function(wrap_pyfunction!(stock_mesh_check, module)?)?;
    module.add_function(wrap_pyfunction!(stock_mesh, module)?)?;
    module.add_function(wrap_pyfunction!(validate_delta, module)?)?;
    module.add_function(wrap_pyfunction!(display_overlay_patch, module)?)?;
    module.add_class::<SplineSurfaceBounds>()?;
    module.add_class::<MaterialTarget>()?;
    module.add_class::<IntentSession>()?;
    module.add_class::<MotionDecoder>()?;
    module.add_class::<DefaultMotionPost>()?;
    module.add_function(wrap_pyfunction!(format_post_words, module)?)?;
    module.add_function(wrap_pyfunction!(round_circular_words, module)?)?;
    module.add_function(wrap_pyfunction!(css_rpm_bound, module)?)?;
    module.add_function(wrap_pyfunction!(post_job, module)?)?;
    module.add_function(wrap_pyfunction!(post_program, module)?)?;
    module.add_function(wrap_pyfunction!(probe_legs, module)?)?;
    module.add_class::<PostCancellation>()?;
    module.add_class::<StreamVerification>()?;
    module.add_class::<StreamControlContext>()?;
    Ok(())
}
