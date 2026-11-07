//! Turning tool bodies off the centre plane (#103).
//!
//! A turning tool's outline is its section on the centre plane through the
//! spindle axis. The real insert has thickness below its cutting face, its
//! sides may taper inward by its clearance angle, the holder's seat may tilt
//! it (rake and inclination), and holders and bars have width across the
//! plane. A body point at (x, y) off the axis lies at radius √(x² + y²): in
//! a bore it reaches farther out than the outline shows.
//!
//! Every turning tool declares its body (`ToolBody`). Its convex pieces,
//! in the tool frame (x radial, z axial, y across the centre plane, the
//! cutting face at y = 0 and the insert body below it), are checked as 3D
//! vertex sets:
//! - against material outside the tool's radius (outside turning: the
//!   part, the stock), by their projection on the XZ plane: a point's
//!   radius is at least its |x|, so the projection is conservative;
//! - against material beyond the tool's radius (in a bore), by their radial
//!   image (z, √(x² + y²)) of the vertices: the radius is a convex function,
//!   so every point of a convex piece is at most the image hull's radius at
//!   its z (exact at vertices, never under-reporting).
//!
//! Sources:
//! - ISO 1832:1985 §3.2: the clearance letter is the normal clearance of
//!   the major cutting edge (N 0, A 3, B 5, C 7, D 15, E 20, F 25, G 30,
//!   P 11 degrees; O other, specified separately).
//! - ISO 1832:1985 §3.3, §3.6: the thickness s is "the distance between the
//!   cutting edge of the corner and the opposing supporting surface"; its
//!   symbol is s in mm disregarding decimals (T1, T3 for 1.98, 3.97 mm), so a
//!   code bounds it only to [n, n + 1); the tolerance class adds ±0.025 mm
//!   (A, F, C, H, E, J, K, L, N) or ±0.13 mm (G, M, U).
//! - ISO 5610 (holder figures): the rake and inclination angles are
//!   "recommended values; they can vary according to the application": a
//!   holder's seat angles are its maker's data and are declared.
use super::tool::{TurningTool, convex_pieces};
use serde::{Deserialize, Serialize};

/// A turning tool's body off the centre plane. Every value is the caller's
/// (or, for thickness and clearance, an ISO 1832 code's worst case).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ToolBody {
    /// The insert's body and seat: required for every insert but a drill
    /// (a drill on the axis is a solid of revolution: its outline is its
    /// exact radial section).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub insert: Option<InsertBody>,
    /// The holder outline's extent across the centre plane, [low, high] mm
    /// (the cutting face at 0, the insert body toward negative).
    pub holder_y_mm: [f64; 2],
    /// A round boring bar's axis offset across the centre plane (mm).
    #[serde(default)]
    pub bar_axis_y_mm: f64,
}

/// The insert's body below its cutting face and the seat that tilts it.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InsertBody {
    /// The insert's thickness below its cutting face (mm).
    pub insert_thickness_mm: f64,
    /// The insert's normal clearance (degrees): its sides taper inward below
    /// the face by thickness x tan(clearance). 0 for a negative insert.
    pub insert_clearance_deg: f64,
    /// The seat's orthogonal rake and inclination (degrees, ISO 3002-1),
    /// about `edge_deg`: negative rake tilts the face up away from the edge
    /// (the flank below the edge backs off); positive inclination raises the
    /// edge away from the nose.
    pub rake_deg: f64,
    pub inclination_deg: f64,
    /// The major cutting edge's direction in the tool frame (degrees from +x
    /// toward +z, leaving the nose): the axis the rake turns about.
    pub edge_deg: f64,
    /// A face grooving blade curved to follow the groove (its maker's
    /// diameter range for it, [min, max] mm): off the centre plane it stays
    /// at the radius of its outline, so its body reaches no farther than
    /// the outline. Every face groove it cuts must lie in the range. None:
    /// a straight blade, checked as its straight body.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub curved_for_diameter_mm: Option<[f64; 2]>,
}

impl ToolBody {
    pub fn validate(&self, id: &str, drill: bool) -> Result<(), String> {
        let finite = |v: f64| v.is_finite() && v.abs() <= 1e4;
        if !(finite(self.holder_y_mm[0]) && finite(self.holder_y_mm[1]))
            || self.holder_y_mm[0] >= self.holder_y_mm[1]
            || !finite(self.bar_axis_y_mm)
        {
            return Err(format!(
                "turning tool {id:?}: its body needs the holder's extent across the centre plane from low to high and a finite bar axis offset"
            ));
        }
        match (&self.insert, drill) {
            (Some(_), true) => Err(format!(
                "turning tool {id:?}: a drill's outline is its exact section (a solid of revolution on the axis): give no insert body"
            )),
            (None, false) => Err(missing_body(id)),
            (None, true) => Ok(()),
            (Some(b), false) => {
                if !(finite(b.insert_thickness_mm) && b.insert_thickness_mm > 0.)
                    || !(0. ..60.).contains(&b.insert_clearance_deg)
                    || !(-45. ..=45.).contains(&b.rake_deg)
                    || !(-45. ..=45.).contains(&b.inclination_deg)
                    || !finite(b.edge_deg)
                    || b.curved_for_diameter_mm
                        .is_some_and(|[lo, hi]| !(finite(lo) && finite(hi) && 0. < lo && lo <= hi))
                {
                    return Err(format!(
                        "turning tool {id:?}: its insert body needs a positive thickness, a clearance 0..60 degrees, seat rake and inclination within ±45 degrees and a finite edge direction"
                    ));
                }
                Ok(())
            }
        }
    }
}

/// An insert's thickness bound and clearance from its ISO 1832 designation:
/// the largest thickness its symbol and tolerance class allow, and the
/// normal clearance of its letter.
pub fn iso_insert_body(code: &str) -> Result<(f64, f64), String> {
    let code = code.trim().to_ascii_uppercase();
    let c: Vec<char> = code.chars().collect();
    if c.len() < 10 {
        return Err(format!(
            "insert code {code:?}: give the full ISO 1832 designation"
        ));
    }
    let clearance = match c[1] {
        'N' => 0.,
        'A' => 3.,
        'B' => 5.,
        'C' => 7.,
        'P' => 11.,
        'D' => 15.,
        'E' => 20.,
        'F' => 25.,
        'G' => 30.,
        other => {
            return Err(format!(
                "insert code {code:?}: clearance {other:?} is not a standard ISO 1832 letter (O means a special clearance): declare insert_clearance_deg"
            ));
        }
    };
    let tolerance = match c[2] {
        'A' | 'F' | 'C' | 'H' | 'E' | 'J' | 'K' | 'L' | 'N' => 0.025,
        'G' | 'M' | 'U' => 0.13,
        other => {
            return Err(format!(
                "insert code {code:?}: tolerance class {other:?} is not an ISO 1832 class: declare insert_thickness_mm"
            ));
        }
    };
    let symbol: String = c[6..8].iter().collect();
    // [n, n + 1) mm by the symbol rule; T1/T3 are 1.98 and 3.97 mm.
    let below = match symbol.as_str() {
        "T1" => 2.,
        "T3" => 4.,
        s if s.chars().all(|d| d.is_ascii_digit()) => {
            let n: u32 = s
                .parse()
                .map_err(|_| format!("insert code {code:?}: thickness"))?;
            if n == 0 {
                return Err(format!("insert code {code:?}: thickness symbol 00"));
            }
            f64::from(n) + 1.
        }
        _ => {
            return Err(format!(
                "insert code {code:?}: letters 7-8 must be an ISO 1832 thickness symbol (01, T1, 02, 03, T3, 04, 05, 06, 07, 09, ...)"
            ));
        }
    };
    Ok((below + tolerance, clearance))
}

/// A convex polygon eroded by `d` (each side moved inward by d): the
/// intersection of the inward half-planes, by clipping. None when nothing
/// is left.
fn erode_convex(polygon: &[[f64; 2]], d: f64) -> Option<Vec<[f64; 2]>> {
    let n = polygon.len();
    let area2: f64 = (0..n)
        .map(|i| {
            let (a, b) = (polygon[i], polygon[(i + 1) % n]);
            a[0] * b[1] - b[0] * a[1]
        })
        .sum();
    let ccw = area2 > 0.;
    let mut out = polygon.to_vec();
    for i in 0..n {
        let (a, b) = (polygon[i], polygon[(i + 1) % n]);
        let (ex, ez) = (b[0] - a[0], b[1] - a[1]);
        let len = ex.hypot(ez);
        if len == 0. {
            continue;
        }
        // Inward unit normal.
        let nrm = if ccw {
            [-ez / len, ex / len]
        } else {
            [ez / len, -ex / len]
        };
        let offset = nrm[0] * a[0] + nrm[1] * a[1] + d;
        let inside = |p: [f64; 2]| nrm[0] * p[0] + nrm[1] * p[1] - offset;
        let mut clipped = vec![];
        for k in 0..out.len() {
            let (p, q) = (out[k], out[(k + 1) % out.len()]);
            let (sp, sq) = (inside(p), inside(q));
            if sp >= 0. {
                clipped.push(p);
            }
            if (sp >= 0.) != (sq >= 0.) {
                let t = sp / (sp - sq);
                clipped.push([p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])]);
            }
        }
        if clipped.len() < 3 {
            return None;
        }
        out = clipped;
    }
    Some(out)
}

/// The tool's body as convex 3D pieces in its frame (x, z, y), the nose
/// center at the origin, for an outside right-hand tool (the caller mirrors
/// as for the outline): the insert's pieces, then the holder's (and bar's).
/// Convex 3D pieces (x, z, y) of a body.
pub type Pieces3 = Vec<Vec<[f64; 3]>>;

pub fn body_pieces(tool: &TurningTool) -> Result<(Pieces3, Pieces3), String> {
    let tool_body = tool.body.as_ref().ok_or_else(|| missing_body(&tool.id))?;
    let drill = matches!(tool.insert, super::tool::Insert::Drill { .. });
    tool_body.validate(&tool.id, drill)?;
    let curved = matches!(tool.insert, super::tool::Insert::FaceGroove { .. })
        && tool_body
            .insert
            .as_ref()
            .is_some_and(|b| b.curved_for_diameter_mm.is_some());
    let shape = tool.shape()?;
    let polygon = &shape.insert;
    let pieces = convex_pieces(polygon);
    let mut insert: Vec<Vec<[f64; 3]>>;
    let Some(body) = tool_body.insert.as_ref().filter(|_| !curved) else {
        // A drill: its outline is its radial section. A curved face
        // grooving blade: it follows the groove's radius off the plane.
        insert = pieces
            .iter()
            .map(|piece| piece.iter().map(|p| [p[0], p[1], 0.]).collect())
            .collect();
        return Ok((insert, holder_pieces(tool, &shape, tool_body)));
    };
    let s = body.insert_thickness_mm;
    insert = if pieces.len() == 1 {
        let bottom = erode_convex(polygon, s * body.insert_clearance_deg.to_radians().tan())
            .ok_or_else(|| {
                format!(
                    "turning tool {:?}: its clearance taper ({} degrees over {s} mm) leaves no insert bottom",
                    tool.id, body.insert_clearance_deg
                )
            })?;
        vec![
            polygon
                .iter()
                .map(|p| [p[0], p[1], 0.])
                .chain(bottom.iter().map(|p| [p[0], p[1], -s]))
                .collect(),
        ]
    } else {
        // A concave outline: its convex pieces extruded straight (a prism
        // contains a tapering body).
        pieces
            .iter()
            .map(|piece| {
                piece
                    .iter()
                    .flat_map(|p| [[p[0], p[1], 0.], [p[0], p[1], -s]])
                    .collect()
            })
            .collect()
    };
    // The seat: rake about the major edge, then inclination about the
    // in-plane direction across it (into the insert), both about the nose.
    let e = [
        body.edge_deg.to_radians().cos(),
        body.edge_deg.to_radians().sin(),
    ];
    let centroid = polygon
        .iter()
        .fold([0., 0.], |c, p| [c[0] + p[0], c[1] + p[1]]);
    let mut q = [-e[1], e[0]];
    if q[0] * centroid[0] + q[1] * centroid[1] < 0. {
        q = [-q[0], -q[1]];
    }
    let lambda = body.inclination_deg.to_radians();
    // ISO 3002-1: tan(normal rake) = tan(orthogonal rake) cos(inclination).
    let gamma_n = (body.rake_deg.to_radians().tan() * lambda.cos()).atan();
    let (theta1, theta2) = (-gamma_n, lambda);
    for piece in &mut insert {
        for v in piece.iter_mut() {
            let (a, b, c) = (v[0] * e[0] + v[1] * e[1], v[0] * q[0] + v[1] * q[1], v[2]);
            let (b, c) = (
                b * theta1.cos() - c * theta1.sin(),
                b * theta1.sin() + c * theta1.cos(),
            );
            let (a, c) = (
                a * theta2.cos() - c * theta2.sin(),
                a * theta2.sin() + c * theta2.cos(),
            );
            *v = [a * e[0] + b * q[0], a * e[1] + b * q[1], c];
        }
    }
    Ok((insert, holder_pieces(tool, &shape, tool_body)))
}

fn holder_pieces(
    tool: &TurningTool,
    shape: &super::tool::ToolShape,
    body: &ToolBody,
) -> Vec<Vec<[f64; 3]>> {
    let [y0, y1] = body.holder_y_mm;
    let mut holder: Vec<Vec<[f64; 3]>> = convex_pieces(&shape.holder)
        .iter()
        .map(|piece| {
            piece
                .iter()
                .flat_map(|p| [[p[0], p[1], y0], [p[0], p[1], y1]])
                .collect()
        })
        .collect();
    if let (Some(bar), Some(section)) = (&tool.holder.bar, &shape.bar) {
        // The round bar (axis parallel to z): a circumscribed 32-gon prism.
        let r = bar.diameter_mm / 2. / (std::f64::consts::PI / 32.).cos();
        let x = (section[0][0] + section[1][0]) / 2.;
        let (z0, z1) = (section[0][1], section[2][1]);
        holder.push(
            (0..32)
                .flat_map(|k| {
                    let t = std::f64::consts::TAU * (k as f64 + 0.5) / 32.;
                    let (px, py) = (x + r * t.cos(), body.bar_axis_y_mm + r * t.sin());
                    [[px, z0, py], [px, z1, py]]
                })
                .collect(),
        );
    }
    holder
}

pub(crate) fn missing_body(id: &str) -> String {
    format!(
        "turning tool {id:?} needs its body (#103): the insert's thickness and clearance (from its ISO code, or declared), the holder's seat rake and inclination (its maker's data; 0 and 0 for a flat seat) and the holder's extent across the centre plane. camlib assumes no tool is flat"
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::turning::stock::TurnSide;
    use crate::turning::tool::{Hand, Holder, Insert};

    #[test]
    fn iso_codes_give_worst_case_thickness_and_clearance() {
        let (s, a) = iso_insert_body("CNMG120408").unwrap();
        assert!((s - 5.13).abs() < 1e-12 && a == 0.);
        let (s, a) = iso_insert_body("CCMT09T304").unwrap();
        assert!((s - 4.13).abs() < 1e-12 && a == 7.);
        let (s, a) = iso_insert_body("DCGT11T302").unwrap();
        assert!((s - 4.13).abs() < 1e-12 && a == 7.);
        let (s, a) = iso_insert_body("TPGN160308").unwrap();
        assert!((s - 4.13).abs() < 1e-12 && a == 11.);
        assert!(
            iso_insert_body("COMT120408")
                .unwrap_err()
                .contains("special clearance")
        );
        assert!(
            iso_insert_body("CNZG120408")
                .unwrap_err()
                .contains("tolerance class")
        );
    }

    #[test]
    fn erosion_moves_every_side_inward() {
        let square = [[0., 0.], [10., 0.], [10., 10.], [0., 10.]];
        let e = erode_convex(&square, 1.).unwrap();
        for p in &e {
            assert!(p.iter().all(|v| (1. - 1e-12..=9. + 1e-12).contains(v)));
        }
        assert!(erode_convex(&square, 6.).is_none());
    }

    fn tool(insert: Insert, rn: f64, side: TurnSide, body: ToolBody) -> TurningTool {
        let holder = Holder::rectangle(&insert, rn, side, 10., 60., 1.).unwrap();
        TurningTool {
            id: "t".into(),
            insert,
            nose_radius_mm: rn,
            side,
            hand: Hand::Right,
            holder,
            body: Some(body),
        }
    }

    fn body(s: f64, alpha: f64, rake: f64, incl: f64, edge: f64) -> ToolBody {
        ToolBody {
            insert: Some(InsertBody {
                insert_thickness_mm: s,
                insert_clearance_deg: alpha,
                rake_deg: rake,
                inclination_deg: incl,
                edge_deg: edge,
                curved_for_diameter_mm: None,
            }),
            holder_y_mm: [-10., 0.],
            bar_axis_y_mm: 0.,
        }
    }

    /// Independent oracle: dense points of the insert body, built directly
    /// from the face polygon (sampled inside), the taper and the rotation,
    /// not from the pieces' vertices.
    fn oracle_points(t: &TurningTool) -> Vec<[f64; 3]> {
        let b = t.body.as_ref().unwrap().insert.as_ref().unwrap();
        let shape = t.shape().unwrap();
        let poly = &shape.insert;
        let (lo, hi) = poly.iter().fold(
            ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]),
            |(lo, hi), p| {
                (
                    [lo[0].min(p[0]), lo[1].min(p[1])],
                    [hi[0].max(p[0]), hi[1].max(p[1])],
                )
            },
        );
        let inside = |p: [f64; 2], d: f64| {
            // Inside the polygon by at least d (convex polygons only here).
            let n = poly.len();
            let area2: f64 = (0..n)
                .map(|i| poly[i][0] * poly[(i + 1) % n][1] - poly[(i + 1) % n][0] * poly[i][1])
                .sum();
            (0..n).all(|i| {
                let (a, c) = (poly[i], poly[(i + 1) % n]);
                let (ex, ez) = (c[0] - a[0], c[1] - a[1]);
                let len = ex.hypot(ez);
                len == 0. || {
                    let cross = ex * (p[1] - a[1]) - ez * (p[0] - a[0]);
                    (if area2 > 0. { cross } else { -cross }) / len >= d
                }
            })
        };
        let e = [b.edge_deg.to_radians().cos(), b.edge_deg.to_radians().sin()];
        let c = poly
            .iter()
            .fold([0., 0.], |c, p| [c[0] + p[0], c[1] + p[1]]);
        let mut q = [-e[1], e[0]];
        if q[0] * c[0] + q[1] * c[1] < 0. {
            q = [-q[0], -q[1]];
        }
        let l = b.inclination_deg.to_radians();
        let g = (b.rake_deg.to_radians().tan() * l.cos()).atan();
        let mut out = vec![];
        for i in 0..=60 {
            for j in 0..=60 {
                let p = [
                    lo[0] + (hi[0] - lo[0]) * i as f64 / 60.,
                    lo[1] + (hi[1] - lo[1]) * j as f64 / 60.,
                ];
                for k in 0..=8 {
                    let y = -b.insert_thickness_mm * k as f64 / 8.;
                    if !inside(p, -y * b.insert_clearance_deg.to_radians().tan()) {
                        continue;
                    }
                    let (a0, b0) = (p[0] * e[0] + p[1] * e[1], p[0] * q[0] + p[1] * q[1]);
                    // Rotate: rake (-g) about e, then inclination l about q.
                    let (b1, c1) = (
                        b0 * (-g).cos() - y * (-g).sin(),
                        b0 * (-g).sin() + y * (-g).cos(),
                    );
                    let (a2, c2) = (a0 * l.cos() - c1 * l.sin(), a0 * l.sin() + c1 * l.cos());
                    out.push([a2 * e[0] + b1 * q[0], a2 * e[1] + b1 * q[1], c2]);
                }
            }
        }
        out
    }

    #[test]
    fn insert_pieces_contain_the_body_and_reach_its_corners() {
        let (insert, rn) = Insert::iso_code("CCMT09T304", 95.).unwrap();
        for (rake, incl) in [(0., 0.), (-6., -6.), (6., 3.), (0., -8.)] {
            let t = tool(
                insert.clone(),
                rn,
                TurnSide::Outer,
                body(3.97, 7., rake, incl, 5.),
            );
            let (pieces, _) = body_pieces(&t).unwrap();
            let points = oracle_points(&t);
            assert!(points.len() > 100);
            // Every oracle point lies in the convex hull of the piece
            // vertices: checked through the radial bound at any offset (the
            // property the checks rely on) and the projection extents.
            for offset in [0., 8., 11.] {
                let hull_max = pieces[0]
                    .iter()
                    .map(|v| (v[0] + offset).hypot(v[2]))
                    .fold(f64::NEG_INFINITY, f64::max);
                let oracle_max = points
                    .iter()
                    .map(|v| (v[0] + offset).hypot(v[2]))
                    .fold(f64::NEG_INFINITY, f64::max);
                assert!(oracle_max <= hull_max + 1e-9, "{rake} {incl} {offset}");
                // Tight: the oracle (sampled) gets within the sampling step.
                assert!(
                    hull_max - oracle_max < 0.25,
                    "{rake} {incl} {offset}: {hull_max} {oracle_max}"
                );
            }
        }
    }

    #[test]
    fn a_round_bar_is_contained_off_the_plane() {
        use crate::turning::tool::Bar;
        let (insert, rn) = Insert::iso_code("CCMT060204", 95.).unwrap();
        let mut t = tool(insert, rn, TurnSide::Inner, body(2.38, 7., 0., 0., 5.));
        t.holder.bar = Some(Bar {
            diameter_mm: 10.,
            axis_x_mm: 6.,
            z_mm: [0., 60.],
        });
        for axis_y in [0., 3., -2.5] {
            t.body.as_mut().unwrap().bar_axis_y_mm = axis_y;
            let (_, holder) = body_pieces(&t).unwrap();
            let bar = holder.last().unwrap();
            let nose = t.shape().unwrap().bar.unwrap()[0][0] + 5. - 6.;
            for offset in [0., 4., 9.] {
                let hull = bar
                    .iter()
                    .map(|v| (v[0] + offset).hypot(v[2]))
                    .fold(f64::NEG_INFINITY, f64::max);
                // The true cylinder: radius 5 about (axis x, axis y).
                let truth = (0..3600)
                    .map(|k| {
                        let a = std::f64::consts::TAU * k as f64 / 3600.;
                        (6. + nose + 5. * a.cos() + offset).hypot(axis_y + 5. * a.sin())
                    })
                    .fold(f64::NEG_INFINITY, f64::max);
                assert!(
                    truth <= hull + 1e-9 && hull - truth < 0.05,
                    "{axis_y} {offset}: {truth} {hull}"
                );
            }
        }
    }

    #[test]
    fn the_issues_boring_insert_reaches_past_its_outline() {
        // A 4.76 mm thick negative insert, its back corner 10.6 mm out on
        // the face: 4.76 mm below, √(10.6² + 4.76²) = 11.62 mm from the axis.
        let face: f64 = 10.6;
        let r = face.hypot(4.76);
        assert!((r - 11.6196).abs() < 1e-3);
        let (insert, rn) = Insert::iso_code("CNMG120408", 95.).unwrap();
        let t = tool(insert, rn, TurnSide::Inner, body(4.76, 0., 0., 0., 5.));
        let (pieces, _) = body_pieces(&t).unwrap();
        let reach_face = pieces[0]
            .iter()
            .filter(|v| v[2] == 0.)
            .map(|v| v[0])
            .fold(f64::NEG_INFINITY, f64::max);
        let reach_body = pieces[0]
            .iter()
            .map(|v| v[0].hypot(v[2]))
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(reach_body > reach_face + 0.5);
    }
}
