//! Turning inserts: the in-plane size tolerance (#113).
//!
//! An indexable insert is not exactly its nominal size. Its ISO 1832
//! tolerance class (the third letter of its designation) bounds three
//! dimensions: the inscribed circle d, the corner dimension m and the
//! thickness s. The thickness is applied with the body (`body.rs`); this
//! module applies d and m to the insert's outline.
//!
//! Sources (read for this module; see docs/operations.md):
//! - ISO 1832:2017, 5.3 "Symbol for tolerance class", as shown in the
//!   standard's public preview (standards.iteh.ai sample of ISO 1832:2017):
//!   table 3 (classes A, F, C, H, E, G, J, K, L, M, N, U: tolerances on d,
//!   m and s), table 4 (d for classes J, K, L, M, N and U of shapes H, O, P,
//!   S, T, C, E, M, W and R, and m for classes M, N and U of shapes H, O, P,
//!   S, T, C, E, M and W, by inscribed circle), table 5 (d and m for classes
//!   M and N of shapes D and V), and figures 1 and 2, which define m. The
//!   preview of ISO 1832:1991 (4.3, tables 3 to 5) gives the same values
//!   (its table 5 has shape D only).
//! - ISO 883:1985 (indexable hardmetal inserts with rounded corners,
//!   without fixing hole: triangular and square), annex B "Method of
//!   measurement of m dimension" (an integral part of that standard), as
//!   shown in its public preview (standards.iteh.ai sample of ISO
//!   883:1985), with its figures 2 and 3.
//! - Mitsubishi Materials, technical information "3. Symbol for Tolerance
//!   Class" (turning insert identification): the same class table and, for
//!   class M, the same d and m by inscribed circle and shape (triangular,
//!   square, 80, 55 and 35 degree rhombic, round).
//! - Kyocera, general catalogue, section B "Turning Indexable Inserts
//!   Identification System": the same class table.
//!
//! What d and m are (ISO 1832:2017, 5.3, figures 1 and 2):
//! - d is the diameter of the insert's inscribed circle;
//! - m, for an insert with an odd number of sides (figure 1: triangle,
//!   pentagon), is the distance from a corner's tip to the side opposite
//!   it; for an insert with an even number of sides (figure 2: square,
//!   rhombic, the 80 degree hexagon W), the distance along the corner's
//!   bisector from the inscribed circle (its tangent on the corner's side)
//!   to the corner's tip.
//!
//! How m is measured (ISO 883:1985 annex B):
//! - B.1, triangular inserts: "Dimension m is related to the side opposite
//!   the corner that is to be measured. The insert is placed on a surface
//!   plate" and a dial gauge, zeroed on a gauge block of m, reads the
//!   corner's tip (figure 2).
//! - B.2, square inserts: "Dimension m is checked by reference to the
//!   diameter d of a precision roller, where d corresponds to the nominal
//!   diameter of the inscribed circle of the insert. The insert is mounted
//!   on a 90 degree vee-block" (figure 3: resting on the two sides of the
//!   corner opposite the one measured), and the dial gauge, zeroed to m on
//!   the roller lying in the same vee, reads the corner's tip.
//!
//! The figures of ISO 1832 draw an insert of perfect form, whose inscribed
//! circle touches every side; on a real insert "the inscribed circle" of
//! figure 2 can be read as the insert's own largest circle or, as ISO 883
//! measures it, as a roller of the nominal d against the two opposite
//! sides. ISO 883 settles the method for the triangle and the square, the
//! shapes it covers. For rhombic and hexagonal inserts no annex was read
//! (the public previews of ISO 3364 name its annex B but do not show it):
//! the vee-block reading is applied to them by analogy (a vee of the
//! opposite corner's angle). Because the envelope must not be smaller than the real
//! insert under any of these readings, it is the union of all of them, for
//! every shape; it can be tightened only with a source that settles the
//! measurement for the shape in question.
//!
//! The class does not tolerance the corner radius or the corner angles. The
//! nose radius is the tool's declared value (what the program compensates),
//! and the sides keep their directions. (A side that tilts is not bounded by
//! d and m at all where one of its ends is a corner m does not check, the
//! obtuse corners of a rhombic insert: that needs the angle's tolerance from
//! the insert's dimensional standard, which the designation does not carry.)
//!
//! The frame. The tool is measured at its nose (presetting, touch-off): the
//! planner's controlled point is the nose's imaginary tip or its centre, and
//! the operator's offsets put the real nose there whatever the insert's
//! size. The seat locates the insert by its flanks, so the two cutting edges
//! at the nose keep their lines. Relative to the measured nose, therefore,
//! the nose arc and the two edge lines are fixed, and the tolerance moves
//! the rest of the insert: its far sides and far corners.
//!
//! The readings. With u the unit vector from the nose into the insert along
//! its bisector, ε the nose's corner angle, a = n·u for a far side of
//! outward normal n, and c = b·u for a far corner of outward bisector b:
//!
//! (A) Size. A true-to-form insert of inscribed circle d' in [d - Δd,
//!     d + Δd], its nose on the fixed lines, is the nominal one scaled about
//!     the point P where the two edge lines meet, by d'/d. A far side lies
//!     (d/2)(a / sin(ε/2) + 1) from P, so it moves out by
//!     Δd (a / sin(ε/2) + 1)/2: Δd for the far sides of a rhombic or square
//!     insert (the distance between parallel sides is d), 1.5 Δd for a
//!     triangle's. A far corner's rounded tip moves Δd (c + 1)/(2 sin(ε/2)).
//!     The scaled inserts are nested (a convex set scaled about one of its
//!     points grows; rounding every corner with one radius keeps the
//!     order), so their union is the largest.
//! (B) m from the insert's own inscribed circle (figure 2 read literally;
//!     for the triangle, figure 1 and ISO 883 B.1 exactly). m locates a
//!     corner's tip against its datum. The nose is such a corner, and it is
//!     where the tool is measured: a nose standing Δm farther from its
//!     datum puts the datum, and the insert's body with it, Δm farther
//!     back along the nose's bisector.
//!     - Even: the tip is t = m' + d'/2 from the circle's centre C, so
//!       C = u (m₀' + d'/2 - rε) from the nose centre, and a far side
//!       (tangent to the circle) has its support n·C + d'/2 =
//!       a (m₀' - rε) + (1 + a) d'/2: at most |a| Δm + (1 + a) Δd/2 beyond
//!       nominal. A far corner like the nose has its tip at
//!       b·C + m' + d'/2: at most (|c| + 1) Δm + (1 + c) Δd/2 beyond
//!       (and never beyond the sharp corner of its two sides, which is
//!       rε (1/sin(ε/2) - 1) + |c| Δm + c Δd/2 + Δd/(2 sin(ε/2)) beyond).
//!     - Triangle: the far side is the nose's own datum: at most Δm beyond
//!       nominal. Each far corner's datum is the edge line opposite it,
//!       which is fixed: its tip at most Δm beyond nominal.
//! (C) m by the vee-block and roller (ISO 883 B.2), even numbers of sides.
//!     The far sides keep their directions but each has its own
//!     displacement e (the widths of a rhombic insert need not be equal;
//!     this is also the reading in which d moves a far side and m, at the
//!     corners at its ends, moves it further). The corners carry their
//!     radius. Then:
//!     - the inscribed circle is at least d - Δd: the circle between the
//!       two edge lines at the nose and far side k alone must be that
//!       large, so e_k >= -Δd (a_k / sin(ε/2) + 1)/2 (for a rhombic
//!       insert: each width at least d - Δd);
//!     - at every corner j like the nose (the nose included), the dial
//!       reads within ±Δm of m. The reading is the corner's tip less the
//!       roller's top, along the corner's bisector. The tip moves with the
//!       meeting point of the corner's two sides; the roller, of the
//!       nominal d, with the meeting point of the two sides it rests on,
//!       those of the corner opposite. Both are linear in e.
//!     The largest displacement of each far side and of each checked far
//!     corner's tip under these linear conditions is found exactly (at a
//!     vertex of the region they bound). For a rhombic or square insert the
//!     nose's reading gives e₁ + e₂ <= 2 Δm sin(ε/2), so one far side can
//!     be 2 Δm sin(ε/2) + Δd out while the other is Δd in; the far corner's
//!     tip is at most Δm out.
//!
//! The envelope is the union: each far side is moved out by the largest of
//! its displacements under (A), (B) and (C), E; the two edge lines at the
//! nose and the nose arc stay as they are; a far corner unlike the nose (m
//! does not check it) is rounded with its radius between its (moved) sides;
//! and a far corner like the nose is the sharp corner of its (moved) sides
//! cut off square at the farthest its tip can be under any reading: m
//! locates that corner's tip, not its form, so where the tip may stand
//! farther than the rounding the corner may be any sharper form within its
//! sides. Every insert of every reading lies inside the envelope (its sides
//! lie within the moved sides, its corner tips within their limits), and
//! each face of it is reached by an insert of one of them (the tests sample
//! the three readings and compare). For a CNMG1204 of class M (d ±0.08,
//! m ±0.13): far sides (A) 0.08, (B) 0.149, (C) 0.247 mm; the far corner's
//! tip (A) 0.124, (B) 0.34, (C) 0.13 mm.
//!
//! The least insert. An insert at the small end has its far corners nearer
//! the nose: a far side at most max((A), (B)) in ((C)'s least is (A)'s).
//! The zones the far sides and corners can lie in, between the least and
//! the largest, must be clear of the stock where they lead; and what the
//! planner counts as certainly cut is what every insert cuts with: the
//! nose and the two edges at it, each only as far as the least insert's
//! reaches.
//!
//! Not covered: round inserts (R), whose d is the nose diameter itself: its
//! tolerance is a nose radius error, which changes the cut, not the far side
//! of the body; inserts with five, eight or more sides; and outlines that
//! are not of this form (a side not tangent to the inscribed circle:
//! parallelograms, rectangles, wiper edges). They are refused by name when
//! a size tolerance is declared for them.
use super::tool::{Outline, Segment, circumscribed};
use serde::{Deserialize, Serialize};

/// The insert's in-plane size tolerance: ± on its inscribed circle d and on
/// its corner dimension m (ISO 1832 tolerance class), in mm.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SizeTolerance {
    pub d_mm: f64,
    pub m_mm: f64,
}

impl SizeTolerance {
    pub fn validate(&self, id: &str) -> Result<(), String> {
        let ok = |v: f64| v.is_finite() && (0. ..=1.).contains(&v);
        if ok(self.d_mm) && ok(self.m_mm) {
            Ok(())
        } else {
            Err(format!(
                "turning tool {id:?}: its insert's size tolerance needs d and m tolerances of 0..1 mm (ISO 1832 classes give ±0.005 to ±0.38 mm)"
            ))
        }
    }
    /// An insert measured to be exactly its outline.
    pub fn exact(&self) -> bool {
        self.d_mm == 0. && self.m_mm == 0.
    }
}

#[derive(Clone, Copy)]
enum Tol {
    Fixed(f64),
    /// By inscribed circle and shape: tables 4 and 5.
    BySize,
}

/// An ISO 1832 tolerance class: ± on d, m and s (mm).
pub(crate) struct Class {
    d: Tol,
    m: Tol,
    pub s_mm: f64,
}

/// ISO 1832:2017 table 3.
pub(crate) fn class(letter: char) -> Option<Class> {
    use Tol::{BySize, Fixed};
    let (d, m, s_mm) = match letter {
        'A' => (Fixed(0.025), Fixed(0.005), 0.025),
        'F' => (Fixed(0.013), Fixed(0.005), 0.025),
        'C' => (Fixed(0.025), Fixed(0.013), 0.025),
        'H' => (Fixed(0.013), Fixed(0.013), 0.025),
        'E' => (Fixed(0.025), Fixed(0.025), 0.025),
        'G' => (Fixed(0.025), Fixed(0.025), 0.13),
        'J' => (BySize, Fixed(0.005), 0.025),
        'K' => (BySize, Fixed(0.013), 0.025),
        'L' => (BySize, Fixed(0.025), 0.025),
        'M' => (BySize, BySize, 0.13),
        'N' => (BySize, BySize, 0.025),
        'U' => (BySize, BySize, 0.13),
        _ => return None,
    };
    Some(Class { d, m, s_mm })
}

/// ISO 1832:2017 table 4, for shapes H, O, P, S, T, C, E, M, W (and R, d
/// only): inscribed circles (mm); ± on d for classes J, K, L, M, N and for
/// class U; ± on m for classes M, N and for class U.
const TABLE_4: &[(&[f64], f64, f64, f64, f64)] = &[
    (
        &[4.76, 5.56, 6., 6.35, 7.94, 8., 9.525, 10.],
        0.05,
        0.08,
        0.08,
        0.13,
    ),
    (&[12., 12.7], 0.08, 0.13, 0.13, 0.2),
    (&[15.875, 16., 19.05, 20.], 0.1, 0.18, 0.15, 0.27),
    (&[25., 25.4], 0.13, 0.25, 0.18, 0.38),
    (&[31.75, 32.], 0.15, 0.25, 0.2, 0.38),
];

/// ISO 1832:2017 table 5, classes M and N of shapes D (55 degrees) and V
/// (35 degrees): inscribed circles (mm), ± on d, ± on m.
const TABLE_5: &[(char, &[f64], f64, f64)] = &[
    ('D', &[5.56, 6.35, 7.94, 9.525], 0.05, 0.11),
    ('D', &[12.7], 0.08, 0.15),
    ('D', &[15.875, 19.05], 0.1, 0.18),
    ('V', &[6.35, 7.94, 9.525], 0.05, 0.16),
    ('V', &[12.7], 0.08, 0.25),
];

fn listed(sizes: &[f64], d: f64) -> bool {
    sizes.iter().any(|s| (s - d).abs() < 1e-9)
}

/// The size tolerance of an insert from its ISO 1832 designation: its
/// tolerance class (letter 3) and, for the classes whose tolerance depends
/// on the size, its shape and inscribed circle. None for a round insert
/// (its d is its nose diameter: see the module documentation).
pub fn iso_size_tolerance(code: &str) -> Result<Option<SizeTolerance>, String> {
    let code = code.trim().to_ascii_uppercase();
    let c: Vec<char> = code.chars().collect();
    if c.len() < 10 || !c.iter().all(|ch| ch.is_ascii_alphanumeric()) {
        return Err(format!(
            "insert code {code:?}: give the full ISO 1832 designation"
        ));
    }
    let (shape, letter) = (c[0], c[2]);
    let Some(class) = class(letter) else {
        return Err(format!(
            "insert code {code:?}: tolerance class {letter:?} is not an ISO 1832 class: give size_tolerance_mm=(d, m) from the insert maker's data"
        ));
    };
    if shape == 'R' {
        return Ok(None);
    }
    let size: String = c[4..6].iter().collect();
    let d = size
        .parse::<u32>()
        .ok()
        .and_then(|size| super::tool::iso_inscribed_circle(shape, size))
        .ok_or_else(|| {
            format!(
                "insert code {code:?}: shape {shape} of size {size} is not one camlib knows the inscribed circle of; give an Outline insert instead"
            )
        })?;
    let none = |what: &str| {
        format!(
            "insert code {code:?}: ISO 1832 (tables 4 and 5) gives no tolerance on {what} for class {letter} of shape {shape} with a {d} mm inscribed circle: give size_tolerance_mm=(d, m) from the insert maker's data"
        )
    };
    let rhombic_5 = matches!(shape, 'D' | 'V');
    let table_5 = || {
        TABLE_5
            .iter()
            .find(|(s, sizes, _, _)| *s == shape && listed(sizes, d))
            .filter(|_| matches!(letter, 'M' | 'N'))
    };
    let table_4 = || TABLE_4.iter().find(|(sizes, ..)| listed(sizes, d));
    let d_mm = match class.d {
        Tol::Fixed(v) => v,
        Tol::BySize if rhombic_5 => table_5().ok_or_else(|| none("d"))?.2,
        Tol::BySize => {
            let row = table_4().ok_or_else(|| none("d"))?;
            if letter == 'U' { row.2 } else { row.1 }
        }
    };
    let m_mm = match class.m {
        Tol::Fixed(v) => v,
        Tol::BySize if rhombic_5 => table_5().ok_or_else(|| none("m"))?.3,
        Tol::BySize => {
            let row = table_4().ok_or_else(|| none("m"))?;
            if letter == 'U' { row.4 } else { row.3 }
        }
    };
    Ok(Some(SizeTolerance { d_mm, m_mm }))
}

fn dot(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
fn sub(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] - b[0], a[1] - b[1]]
}
fn along(p: [f64; 2], d: [f64; 2], t: f64) -> [f64; 2] {
    [p[0] + t * d[0], p[1] + t * d[1]]
}
/// Where the lines n1·x = c1 and n2·x = c2 meet.
fn meet(n1: [f64; 2], c1: f64, n2: [f64; 2], c2: f64) -> [f64; 2] {
    let det = n1[0] * n2[1] - n1[1] * n2[0];
    [
        (c1 * n2[1] - n1[1] * c2) / det,
        (n1[0] * c2 - c1 * n2[0]) / det,
    ]
}
fn degrees(n: [f64; 2]) -> f64 {
    n[1].atan2(n[0]).to_degrees()
}

/// The part of the convex polygon where n·x >= c.
fn clip(polygon: &[[f64; 2]], n: [f64; 2], c: f64) -> Vec<[f64; 2]> {
    let mut out = vec![];
    for k in 0..polygon.len() {
        let (p, q) = (polygon[k], polygon[(k + 1) % polygon.len()]);
        let (sp, sq) = (dot(n, p) - c, dot(n, q) - c);
        if sp >= 0. {
            out.push(p);
        }
        if (sp >= 0.) != (sq >= 0.) {
            out.push(along(p, sub(q, p), sp / (sp - sq)));
        }
    }
    out
}

/// An outline of the ISO form, counterclockwise from its nose: corner j (an
/// arc), then side j to corner j + 1; the nose is corner 0, the edges at the
/// nose sides 0 and n - 1.
struct Form {
    n: usize,
    /// Each side's unit direction, outward unit normal and support (the
    /// side lies on normal·x = support).
    dirs: Vec<[f64; 2]>,
    normals: Vec<[f64; 2]>,
    supports: Vec<f64>,
    /// Each side's two ends, in its direction.
    ends: Vec<[[f64; 2]; 2]>,
    centers: Vec<[f64; 2]>,
    radii: Vec<f64>,
    /// The outline's segment of each side and of each corner.
    side_segment: Vec<usize>,
    corner_segment: Vec<usize>,
    /// The inscribed circle's centre.
    circle: [f64; 2],
    /// The nose arc's angles (degrees, increasing), as the outline has
    /// them: the envelope's nose is the outline's, point for point.
    nose: [f64; 2],
    /// From the nose into the insert along its bisector.
    u: [f64; 2],
    /// sin(ε/2) for the nose's corner angle ε.
    sin_half: f64,
}

const NOT_ISO_FORM: &str = "a size tolerance applies to an outline of the ISO 1832 form: straight sides, each tangent to the insert's inscribed circle, joined by rounded corners (a rhombic, square, triangular or 80 degree hexagonal insert); for any other outline, state the largest outline the insert can have and give no size tolerance";

fn form(outline: &Outline, corner: usize) -> Result<Form, String> {
    let count = outline.segments.len();
    let n = count / 2;
    if !count.is_multiple_of(2) || n < 3 || corner >= count {
        return Err(NOT_ISO_FORM.into());
    }
    let ends = outline.ends();
    let ccw = outline.signed_area2() > 0.;
    let (mut dirs, mut normals, mut supports) = (vec![], vec![], vec![]);
    let (mut centers, mut radii, mut side_ends) = (vec![], vec![], vec![]);
    let (mut side_segment, mut corner_segment) = (vec![], vec![]);
    for j in 0..n {
        // Counterclockwise from the nose: with the outline, or against it.
        let (arc, line) = if ccw {
            ((corner + 2 * j) % count, (corner + 2 * j + 1) % count)
        } else {
            (
                (corner + 2 * (n - j)) % count,
                (corner + 2 * (n - j - 1) + 1) % count,
            )
        };
        let (
            Segment::Arc {
                center_mm,
                radius_mm,
                ..
            },
            Segment::Line { .. },
        ) = (&outline.segments[arc], &outline.segments[line])
        else {
            return Err(NOT_ISO_FORM.into());
        };
        let (a, b) = if ccw {
            ends[line]
        } else {
            (ends[line].1, ends[line].0)
        };
        let length = (b[0] - a[0]).hypot(b[1] - a[1]);
        if length <= 1e-9 {
            return Err(NOT_ISO_FORM.into());
        }
        let d = [(b[0] - a[0]) / length, (b[1] - a[1]) / length];
        let normal = [d[1], -d[0]];
        side_ends.push([a, b]);
        dirs.push(d);
        normals.push(normal);
        supports.push(dot(normal, a));
        centers.push(*center_mm);
        radii.push(*radius_mm);
        side_segment.push(line);
        corner_segment.push(arc);
    }
    let scale = centers
        .iter()
        .fold(1., |m: f64, c| m.max(c[0].abs()).max(c[1].abs()));
    let gap = 1e-6 * scale;
    for j in 0..n {
        let before = (j + n - 1) % n;
        // Convex, and each corner's arc tangent to both its sides.
        let turn = dirs[before][0] * dirs[j][1] - dirs[before][1] * dirs[j][0];
        let tangent =
            |side: usize| (dot(normals[side], centers[j]) + radii[j] - supports[side]).abs() <= gap;
        if turn <= 1e-9 || !tangent(before) || !tangent(j) {
            return Err(NOT_ISO_FORM.into());
        }
    }
    // The inscribed circle: n·C + rho = support for every side (least
    // squares over the sides; every side must then be tangent).
    let mut ata = [[0.; 3]; 3];
    let mut atb = [0.; 3];
    for j in 0..n {
        let row = [normals[j][0], normals[j][1], 1.];
        for r in 0..3 {
            for c in 0..3 {
                ata[r][c] += row[r] * row[c];
            }
            atb[r] += row[r] * supports[j];
        }
    }
    let det3 = |m: &[[f64; 3]; 3]| {
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    };
    let det = det3(&ata);
    if det.abs() < 1e-9 {
        return Err(NOT_ISO_FORM.into());
    }
    let mut solution = [0.; 3];
    for (k, value) in solution.iter_mut().enumerate() {
        let mut m = ata;
        for r in 0..3 {
            m[r][k] = atb[r];
        }
        *value = det3(&m) / det;
    }
    let (circle, rho) = ([solution[0], solution[1]], solution[2]);
    if (0..n).any(|j| (dot(normals[j], circle) + rho - supports[j]).abs() > gap) {
        return Err(NOT_ISO_FORM.into());
    }
    let Segment::Arc {
        start_deg, end_deg, ..
    } = &outline.segments[corner]
    else {
        return Err(NOT_ISO_FORM.into());
    };
    let nose = [start_deg.min(*end_deg), start_deg.max(*end_deg)];
    let from_nose = sub(circle, centers[0]);
    let reach = from_nose[0].hypot(from_nose[1]);
    if rho <= radii[0] + gap || reach <= gap {
        return Err(NOT_ISO_FORM.into());
    }
    let sin_half = (rho - radii[0]) / reach;
    Ok(Form {
        n,
        dirs,
        normals,
        supports,
        ends: side_ends,
        centers,
        radii,
        side_segment,
        corner_segment,
        circle,
        nose,
        u: [from_nose[0] / reach, from_nose[1] / reach],
        sin_half,
    })
}

/// A part of the insert that must not lead into the stock: a region, and
/// the outward normals its boundary can have there (it leads when the
/// motion has a component along any of them).
pub type BackZone = (Vec<[f64; 2]>, Vec<[f64; 2]>);

/// What an insert of a size tolerance can occupy, relative to its measured
/// nose (the outline's frame).
pub(crate) struct Envelope {
    /// A convex polygon containing every insert of the tolerance (arcs
    /// circumscribed).
    pub largest: Vec<[f64; 2]>,
    /// Where the inserts' non-cutting sides and corners can be: the band
    /// each far side moves through between the least and the largest
    /// insert, and each corner's zone.
    pub back: Vec<BackZone>,
    /// The straight cutting edges every insert of the tolerance has: the
    /// two edges at the nose, each as far as the least insert's reaches
    /// (where its far corner's rounding, or blunter tip, can begin).
    pub certain: Vec<[[f64; 2]; 2]>,
    /// The straight sides of the largest outline: what may sweep material.
    pub sides: Vec<[[f64; 2]; 2]>,
}

/// The largest value of each of `objectives` over the points x of `k`
/// coordinates with row·x <= bound for every (row, bound) of `rows` (a
/// bounded region): at one of its vertices, each where `k` rows hold with
/// equality.
fn maxima(rows: &[(Vec<f64>, f64)], objectives: &[Vec<f64>], k: usize) -> Vec<f64> {
    let mut best = vec![f64::NEG_INFINITY; objectives.len()];
    let mut chosen: Vec<usize> = (0..k).collect();
    if rows.len() < k || k == 0 {
        return best;
    }
    loop {
        // Solve the chosen rows as equalities (Gaussian elimination with
        // partial pivoting).
        let mut m: Vec<Vec<f64>> = chosen
            .iter()
            .map(|r| {
                let mut row = rows[*r].0.clone();
                row.push(rows[*r].1);
                row
            })
            .collect();
        let mut regular = true;
        for c in 0..k {
            let pivot = (c..k)
                .max_by(|a, b| m[*a][c].abs().total_cmp(&m[*b][c].abs()))
                .unwrap();
            if m[pivot][c].abs() < 1e-12 {
                regular = false;
                break;
            }
            m.swap(c, pivot);
            let pivot_row = m[c].clone();
            for (r, row) in m.iter_mut().enumerate() {
                if r != c {
                    let factor = row[c] / pivot_row[c];
                    for (value, p) in row.iter_mut().zip(&pivot_row).skip(c) {
                        *value -= factor * p;
                    }
                }
            }
        }
        if regular {
            let x: Vec<f64> = (0..k).map(|c| m[c][k] / m[c][c]).collect();
            let value = |row: &[f64]| row.iter().zip(&x).map(|(a, b)| a * b).sum::<f64>();
            if rows.iter().all(|(row, bound)| value(row) <= bound + 1e-9) {
                for (b, objective) in best.iter_mut().zip(objectives) {
                    *b = b.max(value(objective));
                }
            }
        }
        // The next choice of k rows.
        let mut i = k;
        loop {
            if i == 0 {
                return best;
            }
            i -= 1;
            if chosen[i] < rows.len() - (k - i) {
                chosen[i] += 1;
                for q in i + 1..k {
                    chosen[q] = chosen[q - 1] + 1;
                }
                break;
            }
        }
    }
}

/// The envelope of `outline` (an ISO form, its nose the arc segment
/// `corner`, `cutting` its cutting segments) under `tolerance`: see the
/// module documentation for the derivation.
pub(crate) fn envelope(
    outline: &Outline,
    corner: usize,
    cutting: &[usize],
    tolerance: &SizeTolerance,
) -> Result<Envelope, String> {
    let f = form(outline, corner)?;
    let n = f.n;
    if !matches!(n, 3 | 4 | 6) {
        return Err(format!(
            "a size tolerance is not worked out for an insert with {n} sides (camlib covers three, four and six: the triangle, the square and rhombic inserts, and the hexagons); state the largest outline the insert can have and give no size tolerance"
        ));
    }
    // Only the nose and the two edges on its fixed lines may cut: a far
    // side's position is not known.
    let fixed = [
        f.corner_segment[0],
        f.side_segment[0],
        f.side_segment[n - 1],
    ];
    if cutting.iter().any(|c| !fixed.contains(c)) {
        return Err(
            "with a size tolerance only the nose and the two edges next to it can cut (their lines are fixed by measuring the nose): the insert's other sides move with its size"
                .into(),
        );
    }
    let (dd, dm) = (tolerance.d_mm, tolerance.m_mm);
    let odd = n == 3;
    // The corner angles; a corner like the nose is checked by m.
    let angle = |j: usize| {
        let (p, q) = (f.dirs[(j + n - 1) % n], f.dirs[j]);
        std::f64::consts::PI - (p[0] * q[1] - p[1] * q[0]).atan2(dot(p, q))
    };
    let checked = |j: usize| (angle(j) - angle(0)).abs() <= 1e-6;
    let bisector = |j: usize| {
        let v = sub(f.centers[j], f.circle);
        let l = v[0].hypot(v[1]);
        [v[0] / l, v[1] / l]
    };
    // (A) and (B): each far side's displacement for the scaled insert,
    // and for the nose standing off the inscribed circle (even) or the
    // opposite side (triangle).
    let scaled = |i: usize| (dd * (dot(f.normals[i], f.u) / f.sin_half + 1.) / 2.).max(0.);
    let floating = |i: usize| {
        let a = dot(f.normals[i], f.u);
        if odd {
            dm
        } else {
            a.abs() * dm + (1. + a) * dd / 2.
        }
    };
    // (C), even: the vee-block reading (ISO 883:1985 annex B.2). The far
    // sides' displacements e are free but for: e_k >= -scaled_k (the
    // inscribed circle no smaller than d - Δd: the circle between the two
    // edge lines at the nose and side k); and at every checked corner j,
    // |reading_j(e)| <= Δm, the reading being the corner's tip less the
    // roller's, both along the corner's bisector: the tip moves with the
    // corner's two sides, the roller with the two sides of the corner
    // opposite, in which it lies. Linear in e: the largest displacement of
    // each side, and of each checked far corner's tip, is at a vertex.
    let far = n - 2;
    // How the meeting point of sides p and q moves, along `b`, with their
    // displacements: a row over the far sides (side i is coordinate i - 1).
    let carried = |j: usize, b: [f64; 2]| -> Vec<f64> {
        let (p, q) = ((j + n - 1) % n, j);
        let mut row = vec![0.; far];
        for (side, shift) in [
            (p, meet(f.normals[p], 1., f.normals[q], 0.)),
            (q, meet(f.normals[p], 0., f.normals[q], 1.)),
        ] {
            if side != 0 && side != n - 1 {
                row[side - 1] += dot(b, shift);
            }
        }
        row
    };
    const UNBOUNDED: f64 = 10.;
    let (mut gauged, mut gauged_tip, mut gauged_short) = (vec![0.; n], vec![0.; n], vec![0.; n]);
    if !odd {
        let mut rows: Vec<(Vec<f64>, f64)> = vec![];
        for i in 1..=far {
            let mut unit = vec![0.; far];
            unit[i - 1] = 1.;
            rows.push((unit.iter().map(|v| -v).collect(), scaled(i)));
            rows.push((unit, UNBOUNDED));
        }
        for j in (0..n).filter(|j| checked(*j)) {
            let b = bisector(j);
            let (tip, roller) = (carried(j, b), carried((j + n / 2) % n, b));
            let reading: Vec<f64> = tip.iter().zip(&roller).map(|(t, r)| t - r).collect();
            rows.push((reading.iter().map(|v| -v).collect(), dm));
            rows.push((reading, dm));
        }
        let mut objectives: Vec<Vec<f64>> = (1..=far)
            .map(|i| {
                let mut unit = vec![0.; far];
                unit[i - 1] = 1.;
                unit
            })
            .collect();
        let tips: Vec<usize> = (1..n).filter(|j| checked(*j)).collect();
        objectives.extend(tips.iter().map(|j| carried(*j, bisector(*j))));
        // ... and how far short of nominal each tip can be.
        objectives.extend(tips.iter().map(|j| {
            carried(*j, bisector(*j))
                .iter()
                .map(|v| -v)
                .collect::<Vec<_>>()
        }));
        let best = maxima(&rows, &objectives, far);
        if best[..far]
            .iter()
            .any(|v| !v.is_finite() || *v >= UNBOUNDED / 2.)
        {
            return Err(
                "the insert's d and m tolerances do not bound one of its sides (ISO 883 annex B's vee-block reading leaves it free): state the largest outline the insert can have and give no size tolerance"
                    .into(),
            );
        }
        gauged[1..=far].copy_from_slice(&best[..far]);
        for (k, j) in tips.iter().enumerate() {
            gauged_tip[*j] = best[far + k];
            gauged_short[*j] = best[far + tips.len() + k];
        }
    }
    // Each far side's largest displacement: the largest over the readings.
    let moved: Vec<f64> = (0..n)
        .map(|i| {
            if i == 0 || i == n - 1 {
                0.
            } else {
                scaled(i).max(floating(i)).max(gauged[i])
            }
        })
        .collect();
    // Each far side's largest displacement inward (the least insert): (A)
    // and (B) are symmetric; the vee-block reading's is (A)'s.
    let inward: Vec<f64> = (0..n)
        .map(|i| {
            if i == 0 || i == n - 1 {
                0.
            } else {
                scaled(i).max(floating(i))
            }
        })
        .collect();
    // The farthest (and nearest) a far corner's tip can be along its
    // bisector, where m checks it: the largest over the readings of (A)
    // the scaled insert's rounded tip, (B) the tip at its farthest m from
    // the inscribed circle at its farthest (even) or from the fixed edge
    // line opposite it (triangle), and (C) the vee-block reading's.
    let tip = |j: usize| -> Option<(f64, f64)> {
        checked(j).then(|| {
            let b = bisector(j);
            let nominal = dot(b, f.centers[j]) + f.radii[j];
            let c = dot(b, f.u);
            let scaled = dd * (c + 1.) / f.sin_half / 2.;
            // (B): the tip at its farthest (or nearest) m; but no farther
            // than the sharp corner of its sides (tangent to the same
            // circle).
            let (out, back) = if odd {
                (dm, dm)
            } else {
                let by_m = (c.abs() + 1.) * dm + (1. + c) * dd / 2.;
                let sharp = f.radii[j] * (1. / f.sin_half - 1.);
                (
                    by_m.min(sharp + c.abs() * dm + c * dd / 2. + dd / f.sin_half / 2.),
                    by_m,
                )
            };
            (
                nominal - scaled.max(back).max(gauged_short[j]),
                nominal + scaled.max(out).max(gauged_tip[j]),
            )
        })
    };
    // Corner j's centre with its two sides displaced by `e`.
    let center = |j: usize, e: [f64; 2]| {
        let before = (j + n - 1) % n;
        meet(
            f.normals[before],
            f.supports[before] + e[0] - f.radii[j],
            f.normals[j],
            f.supports[j] + e[1] - f.radii[j],
        )
    };
    let arc = |c: [f64; 2], j: usize| {
        let before = (j + n - 1) % n;
        let from = degrees(f.normals[before]);
        let mut to = degrees(f.normals[j]);
        while to <= from {
            to += 360.;
        }
        if f.radii[j] == 0. {
            vec![c]
        } else {
            circumscribed(c, f.radii[j], from, to)
        }
    };
    // The largest outline, corner by corner.
    let mut corners: Vec<Vec<[f64; 2]>> = vec![if f.radii[0] == 0. {
        vec![f.centers[0]]
    } else {
        circumscribed(f.centers[0], f.radii[0], f.nose[0], f.nose[1])
    }];
    for j in 1..n {
        let before = j - 1;
        let c = center(j, [moved[before], moved[j]]);
        let b = bisector(j);
        let points = match tip(j) {
            Some((_, farthest)) => {
                // m locates this corner's tip, not its form: the sharp
                // corner of the moved sides, cut off square at the farthest
                // tip.
                let apex = meet(
                    f.normals[before],
                    f.supports[before] + moved[before],
                    f.normals[j],
                    f.supports[j] + moved[j],
                );
                let beyond = dot(b, apex) - farthest;
                if beyond <= 1e-12 {
                    vec![apex]
                } else {
                    vec![
                        along(apex, f.dirs[before], -beyond / dot(b, f.dirs[before])),
                        along(apex, f.dirs[j], beyond / -dot(b, f.dirs[j])),
                    ]
                }
            }
            None => arc(c, j),
        };
        corners.push(points);
    }
    // Each side must keep its length between its corners.
    for j in 0..n {
        let (from, to) = (
            *corners[j].last().unwrap(),
            *corners[(j + 1) % n].first().unwrap(),
        );
        if dot(sub(to, from), f.dirs[j]) <= 1e-9 {
            return Err(
                "the size tolerance leaves no length on one of the insert's sides: it is too large for this outline"
                    .into(),
            );
        }
    }
    let mut largest: Vec<[f64; 2]> = vec![];
    for p in corners.iter().flatten() {
        let near = largest.last().is_some_and(|q: &[f64; 2]| {
            (p[0] - q[0]).hypot(p[1] - q[1]) <= 1e-12 * (1. + p[0].abs().max(p[1].abs()))
        });
        if !near {
            largest.push(*p);
        }
    }
    // Where the non-cutting sides and corners of any insert of the
    // tolerance can be. A far side lies between its least and its largest
    // position, inside the envelope. A corner's rounding (or its blunter
    // tip, where m checks it) lies beyond the least of its tangent points
    // along its bisector, over every position of its two sides.
    let mut back: Vec<BackZone> = vec![];
    for i in 1..n - 1 {
        let band = clip(&largest, f.normals[i], f.supports[i] - inward[i]);
        if band.len() >= 3 {
            back.push((band, vec![f.normals[i]]));
        } else {
            back.push((
                vec![
                    *corners[i].last().unwrap(),
                    *corners[i + 1].first().unwrap(),
                ],
                vec![f.normals[i]],
            ));
        }
    }
    let mut begins = vec![f64::INFINITY; n];
    for j in 1..n {
        let before = j - 1;
        let b = bisector(j);
        let mut least = f64::INFINITY;
        for e0 in [-inward[before], moved[before]] {
            for e1 in [-inward[j], moved[j]] {
                let c = center(j, [e0, e1]);
                for normal in [f.normals[before], f.normals[j]] {
                    least = least.min(dot(b, along(c, normal, f.radii[j])));
                }
            }
        }
        if let Some((nearest, _)) = tip(j) {
            least = least.min(nearest);
        }
        begins[j] = least;
        let zone = clip(&largest, b, least);
        if zone.len() >= 3 {
            back.push((zone, vec![f.normals[before], f.normals[j]]));
        }
    }
    // What every insert of the tolerance certainly cuts with: the nose, and
    // each edge at it from the nose to where the corner at its far end can
    // begin (the corner's zone above). Beyond that point a smaller insert
    // has its corner, not its edge.
    let mut certain = vec![];
    for (side, corner_at_end, nose_end) in [(0, 1, 0), (n - 1, n - 1, 1)] {
        let from = f.ends[side][nose_end];
        // Away from the nose along the side.
        let away = if nose_end == 0 {
            f.dirs[side]
        } else {
            [-f.dirs[side][0], -f.dirs[side][1]]
        };
        let b = bisector(corner_at_end);
        let full = dot(sub(f.ends[side][1 - nose_end], from), away);
        let length = ((begins[corner_at_end] - dot(b, from)) / dot(b, away)).min(full);
        if length > 1e-9 {
            certain.push([from, along(from, away, length)]);
        }
    }
    let sides = (0..n)
        .map(|j| {
            [
                *corners[j].last().unwrap(),
                *corners[(j + 1) % n].first().unwrap(),
            ]
        })
        .collect();
    Ok(Envelope {
        largest,
        back,
        certain,
        sides,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::turning::tool::{Insert, at};

    #[test]
    fn classes_and_sizes_give_the_standards_tolerances() {
        let t = |code: &str| iso_size_tolerance(code).unwrap().unwrap();
        // Table 3: fixed classes. Table 4: classes M, N (m by size), J, K,
        // L (d by size) and U. Table 5: D and V of classes M and N.
        for (code, d, m) in [
            ("TPGN160308", 0.025, 0.025),
            ("DCGT11T302", 0.025, 0.025),
            ("CNMA120408", 0.08, 0.13),
            ("CNMG120408", 0.08, 0.13),
            ("CCMT060204", 0.05, 0.08),
            ("CCMT09T304", 0.05, 0.08),
            ("CNMG160612", 0.1, 0.15),
            ("CNMG190612", 0.1, 0.15),
            ("CNMG250924", 0.13, 0.18),
            ("SNMG310924", 0.15, 0.2),
            ("SNUN120408", 0.13, 0.2),
            ("SNUN190412", 0.18, 0.27),
            ("SNUN250724", 0.25, 0.38),
            ("SNUN310924", 0.25, 0.38),
            ("TNUN160408", 0.08, 0.13),
            ("TNMG160408", 0.05, 0.08),
            ("TNMG220408", 0.08, 0.13),
            ("WNMG080408", 0.08, 0.13),
            ("SNJN120408", 0.08, 0.005),
            ("SNKN120408", 0.08, 0.013),
            ("SNLN120408", 0.08, 0.025),
            ("SNNN120408", 0.08, 0.13),
            ("CNAN120408", 0.025, 0.005),
            ("CNFN120408", 0.013, 0.005),
            ("CNCN120408", 0.025, 0.013),
            ("CNHN120408", 0.013, 0.013),
            ("CNEN120408", 0.025, 0.025),
            ("DNMG110408", 0.05, 0.11),
            ("DNMG150608", 0.08, 0.15),
            ("DNMG190608", 0.1, 0.18),
            ("DCMT070204", 0.05, 0.11),
            ("VNMG160408", 0.05, 0.16),
            ("VBMT110304", 0.05, 0.16),
            ("VNMG220408", 0.08, 0.25),
        ] {
            assert_eq!(t(code), SizeTolerance { d_mm: d, m_mm: m }, "{code}");
        }
        // A round insert's d is its nose.
        assert_eq!(iso_size_tolerance("RCMT10T3M0").unwrap(), None);
        // No value in the standard: refused, naming the fix.
        for code in [
            "DNUG150608", // class U of shape D: tables 4 and 5 give none
            "VNJG160408", // class J of shape V: no d
            "VBMT080204", // V of 4.76 mm: not in table 5
            "TNMG060204", // T of 3.97 mm: not in table 4
            "CNZG120408", // not a class
        ] {
            let e = iso_size_tolerance(code).unwrap_err();
            assert!(e.contains("size_tolerance_mm"), "{code}: {e}");
        }
    }

    // The oracle: inserts of the model built from the shape's angles, its
    // inscribed circle and the definitions of d and m alone (not from the
    // outline, and not by `envelope`).

    fn unit(angle_deg: f64) -> [f64; 2] {
        let a = angle_deg.to_radians();
        [a.cos(), a.sin()]
    }
    /// How far `p` lies outside the convex counterclockwise `polygon`.
    fn outside(polygon: &[[f64; 2]], p: [f64; 2]) -> f64 {
        (0..polygon.len())
            .map(|i| {
                let (a, b) = (polygon[i], polygon[(i + 1) % polygon.len()]);
                let l = (b[0] - a[0]).hypot(b[1] - a[1]);
                if l < 1e-12 {
                    return f64::NEG_INFINITY;
                }
                ((b[1] - a[1]) * (p[0] - a[0]) - (b[0] - a[0]) * (p[1] - a[1])) / l
            })
            .fold(f64::NEG_INFINITY, f64::max)
    }
    fn support(polygon: &[[f64; 2]], n: [f64; 2]) -> f64 {
        polygon
            .iter()
            .map(|p| dot(n, *p))
            .fold(f64::NEG_INFINITY, f64::max)
    }

    /// An insert of the model: its sides counterclockwise from side 0
    /// (outward normal, support), corner j between sides j - 1 and j,
    /// every corner rounded with `r`; where `tips[j]` is given, corner j's
    /// tip is there along its bisector instead (sharper within its sides
    /// if that is beyond its rounding, cut off square if short of it).
    fn member(sides: &[([f64; 2], f64)], r: f64, tips: &[Option<f64>]) -> Vec<[f64; 2]> {
        let n = sides.len();
        let mut points = vec![];
        let mut cuts = vec![];
        for j in 0..n {
            let (p, q) = (sides[(j + n - 1) % n], sides[j]);
            let v = [p.0[0] + q.0[0], p.0[1] + q.0[1]];
            let l = v[0].hypot(v[1]);
            let b = [v[0] / l, v[1] / l];
            let center = meet(p.0, p.1 - r, q.0, q.1 - r);
            if let Some(t) = tips[j] {
                cuts.push((b, t));
                if t > dot(b, center) + r + 1e-9 {
                    points.push(meet(p.0, p.1, q.0, q.1));
                    continue;
                }
            }
            let from = degrees(p.0);
            let mut to = degrees(q.0);
            while to <= from {
                to += 360.;
            }
            points.extend((0..=16).map(|k| at(center, r, from + (to - from) * k as f64 / 16.)));
        }
        for (b, t) in cuts {
            points = clip(&points, [-b[0], -b[1]], -t);
        }
        points
    }

    /// The oracle's inserts for a shape of inscribed circle `d`, corner
    /// radius `r`, approach angle `k` and tolerances (dd, dm): the sampled
    /// inserts of the model, nose centre at the origin, with the directions
    /// their far sides and checked corners face.
    fn model(
        corners: &[f64],
        d: f64,
        r: f64,
        k: f64,
        dd: f64,
        dm: f64,
    ) -> (Vec<Vec<[f64; 2]>>, Vec<[f64; 2]>) {
        let n = corners.len();
        let e = corners[0];
        let half = (e / 2.).to_radians().sin();
        // Side j leaves corner j at heading[j]; its outward normal is the
        // heading turned by -90 degrees.
        let mut heading = vec![k - 90.];
        for c in &corners[1..] {
            heading.push(heading.last().unwrap() + 180. - c);
        }
        let normal = |j: usize| unit(heading[j] - 90.);
        // The nose's bisector into the insert, and the outward bisector of
        // corner j (between the normals of its two sides).
        let u = unit(k - 90. + e / 2.);
        let bisector = |j: usize| unit(heading[j] - 90. - (180. - corners[j]) / 2.);
        let odd = n % 2 == 1;
        let checked: Vec<usize> = (1..n).filter(|j| (corners[*j] - e).abs() < 1e-9).collect();
        let steps = 4;
        let sample = |lo: f64, hi: f64, i: usize| lo + (hi - lo) * i as f64 / steps as f64;
        // The nose centre is the origin: the edge lines at the nose are r
        // from it; the far sides are tangent to the circle at `center`.
        let sides = |center: [f64; 2], radius: f64| -> Vec<([f64; 2], f64)> {
            (0..n)
                .map(|j| {
                    if j == 0 || j == n - 1 {
                        (normal(j), r)
                    } else {
                        (normal(j), dot(normal(j), center) + radius)
                    }
                })
                .collect()
        };
        let mut inserts = vec![];
        // (A) True to form, of every inscribed circle in tolerance: scaled
        // about the point where the edge lines meet.
        let apex = [-u[0] * r / half, -u[1] * r / half];
        for i in 0..=steps {
            let di = sample(d - dd, d + dd, i);
            let center = along(apex, u, di / 2. / half);
            inserts.push(member(&sides(center, di / 2.), r, &vec![None; n]));
        }
        // (B) The corners at any m in tolerance from their datum.
        if odd {
            // Triangle: m is a tip's height above the side opposite it:
            // nominal 1.5 d - r. The far side is the nose's datum (the
            // nose's tip is r beyond the origin); each far corner's datum
            // is an edge line at the nose (r beyond the origin too).
            let m = 1.5 * d - r;
            for i in 0..=steps {
                for q in 0..=steps {
                    let (m0, mj) = (sample(m - dm, m + dm, i), sample(m - dm, m + dm, q));
                    let sides = vec![(normal(0), r), (normal(1), m0 - r), (normal(2), r)];
                    inserts.push(member(&sides, r, &[None, Some(mj - r), Some(mj - r)]));
                }
            }
        } else {
            // Even: m from the inscribed circle (its tangent on the
            // corner's side) along the bisector to the tip: nominal
            // (d/2 - r)/sin(e/2) + r - d/2.
            let m = (d / 2. - r) / half + r - d / 2.;
            for i in 0..=steps {
                for q in 0..=steps {
                    for w in 0..=steps {
                        let di = sample(d - dd, d + dd, i);
                        let (m0, mj) = (sample(m - dm, m + dm, q), sample(m - dm, m + dm, w));
                        // The circle's centre: m0 + di/2 from the nose's
                        // tip, which is r from the origin against u.
                        let center = [u[0] * (m0 + di / 2. - r), u[1] * (m0 + di / 2. - r)];
                        let tips: Vec<Option<f64>> = (0..n)
                            .map(|j| {
                                checked
                                    .contains(&j)
                                    .then(|| dot(bisector(j), center) + mj + di / 2.)
                            })
                            .collect();
                        inserts.push(member(&sides(center, di / 2.), r, &tips));
                    }
                }
            }
        }
        // (C) The vee-block reading (ISO 883:1985 annex B.2, figure 3), for
        // an even number of sides: the far sides anywhere (their directions
        // kept, corners rounded with r), such that at every corner like the
        // nose the dial reads within the m tolerance: the insert rests in
        // a vee on the two sides of the corner opposite, the dial on the
        // corner's tip along its bisector, zeroed on a roller of the
        // nominal d lying in the same vee. And the inscribed circle no
        // smaller than d - dd: the circle between the edge lines at the
        // nose and far side k alone must be that large.
        if !odd {
            let base = sides(along(apex, u, d / 2. / half), d / 2.);
            let reading = |s: &[([f64; 2], f64)], j: usize| {
                let (p, q) = ((j + n - 1) % n, j);
                let o = (j + n / 2) % n;
                let (po, qo) = ((o + n - 1) % n, o);
                let b = bisector(j);
                let tip = dot(b, meet(s[p].0, s[p].1 - r, s[q].0, s[q].1 - r)) + r;
                let roller = dot(
                    b,
                    meet(s[po].0, s[po].1 - d / 2., s[qo].0, s[qo].1 - d / 2.),
                ) + d / 2.;
                tip - roller
            };
            // On the nominal insert the dial reads figure 2's m.
            let m = (d / 2. - r) / half + r - d / 2.;
            let corners_checked: Vec<usize> = std::iter::once(0).chain(checked.clone()).collect();
            for j in &corners_checked {
                assert!((reading(&base, *j) - m).abs() < 1e-9, "{j}");
            }
            let least: Vec<f64> = (1..n - 1)
                .map(|k| dd * (dot(normal(k), u) / half + 1.) / 2.)
                .collect();
            let far = n - 2;
            let grid = if far == 2 { 40 } else { 8 };
            let top = 3. * (dm + dd);
            let mut candidates: Vec<Vec<f64>> = vec![];
            let mut index = vec![0usize; far];
            'grid: loop {
                candidates.push(
                    (0..far)
                        .map(|k| -least[k] + (top + least[k]) * index[k] as f64 / grid as f64)
                        .collect(),
                );
                for step in index.iter_mut() {
                    *step += 1;
                    if *step <= grid {
                        continue 'grid;
                    }
                    *step = 0;
                }
                break;
            }
            if far == 2 {
                // A rhombic or square insert: one far side at its least,
                // the other as far as the readings at the nose and at the
                // corner opposite let it go.
                let most = 2. * dm * half;
                candidates.push(vec![most + least[1], -least[1]]);
                candidates.push(vec![-least[0], most + least[0]]);
            }
            for e in candidates {
                let mut moved = base.clone();
                for k in 0..far {
                    moved[k + 1].1 += e[k];
                }
                if corners_checked
                    .iter()
                    .all(|j| (reading(&moved, *j) - m).abs() <= dm + 1e-9)
                {
                    inserts.push(member(&moved, r, &vec![None; n]));
                }
            }
        }
        let mut directions: Vec<[f64; 2]> = (1..n - 1).map(normal).collect();
        directions.extend(checked.iter().map(|j| bisector(*j)));
        (inserts, directions)
    }

    #[test]
    fn the_envelope_contains_every_insert_of_the_model_and_no_more() {
        // ISO codes, and a regular hexagon drawn by hand (shape H: an
        // outline of the form with a declared tolerance, table 4's for a
        // 12.7 mm class M insert).
        let cases: [(&str, &[f64], f64); 9] = [
            ("CNMG120408", &[80., 100., 80., 100.], 12.7),
            ("CNUG120402", &[80., 100., 80., 100.], 12.7),
            ("CCGT09T304", &[80., 100., 80., 100.], 9.525),
            ("DNMG150608", &[55., 125., 55., 125.], 12.7),
            ("VNMG160408", &[35., 145., 35., 145.], 9.525),
            ("SNMG120408", &[90., 90., 90., 90.], 12.7),
            ("TNMG160408", &[60., 60., 60.], 9.525),
            ("WNMG080408", &[80., 160., 80., 160., 80., 160.], 12.7),
            ("hexagon", &[120.; 6], 12.7),
        ];
        for (code, corners, d) in cases {
            for k in [95., 93., 75., 62.5, 50.] {
                if k + corners[0] >= 180. {
                    continue;
                }
                let (insert, r, tolerance) = if code == "hexagon" {
                    let side = d / 3f64.sqrt();
                    (
                        Insert::polygon_insert(corners, side, 0.8, k).unwrap(),
                        0.8,
                        SizeTolerance {
                            d_mm: 0.08,
                            m_mm: 0.13,
                        },
                    )
                } else {
                    let (insert, r) = Insert::iso_code(code, k).unwrap();
                    (insert, r, iso_size_tolerance(code).unwrap().unwrap())
                };
                let Insert::Outline {
                    outline,
                    corner,
                    cutting,
                    ..
                } = &insert
                else {
                    unreachable!()
                };
                let env = envelope(outline, *corner, cutting, &tolerance).unwrap();
                let (inserts, directions) = model(corners, d, r, k, tolerance.d_mm, tolerance.m_mm);
                // Every insert of the model lies inside the envelope.
                let mut worst = f64::NEG_INFINITY;
                for insert in &inserts {
                    assert!(insert.len() >= 3, "{code} {k}");
                    for p in insert {
                        worst = worst.max(outside(&env.largest, *p));
                    }
                }
                assert!(worst <= 1e-9, "{code} {k}: {worst} mm outside");
                // And the envelope goes no farther than they do, on every
                // far side and every checked corner.
                for n in directions {
                    let reached = inserts
                        .iter()
                        .map(|insert| support(insert, n))
                        .fold(f64::NEG_INFINITY, f64::max);
                    let bound = support(&env.largest, n);
                    // (A hexagon's vee-block readings are sampled on a
                    // coarse grid of its four far sides: it is reached to
                    // within the grid there.)
                    let slack = if corners.len() == 6 { 0.12 } else { 1e-9 };
                    assert!(
                        bound >= reached - 1e-9 && bound <= reached + slack,
                        "{code} {k}: the envelope reaches {bound}, the model {reached} along {n:?}"
                    );
                }
                // What certainly cuts is on every insert of the model: the
                // two edges at the nose, each ending where the least
                // insert's does.
                assert_eq!(env.certain.len(), 2, "{code} {k}");
                for edge in &env.certain {
                    let length = (edge[1][0] - edge[0][0]).hypot(edge[1][1] - edge[0][1]);
                    assert!(length > r, "{code} {k}: {length}");
                    for p in edge {
                        for insert in &inserts {
                            let off = outside(insert, *p);
                            assert!(off.abs() <= 1e-9, "{code} {k}: {p:?} {off}");
                        }
                    }
                }
                let reach = |points: &mut dyn Iterator<Item = [f64; 2]>, n: [f64; 2]| {
                    points.map(|p| dot(n, p)).fold(f64::NEG_INFINITY, f64::max)
                };
                // ... and no shorter than that: some insert of the model
                // has its edge end within a hair of each certain edge's.
                for edge in &env.certain {
                    let far = edge
                        .iter()
                        .copied()
                        .max_by(|a, b| a[0].hypot(a[1]).total_cmp(&b[0].hypot(b[1])))
                        .unwrap();
                    let near = edge.iter().copied().find(|p| *p != far).unwrap();
                    let l = (far[0] - near[0]).hypot(far[1] - near[1]);
                    let d = [(far[0] - near[0]) / l, (far[1] - near[1]) / l];
                    let normal = [d[1], -d[0]];
                    // Points of a model insert on this edge's line.
                    let shortest = inserts
                        .iter()
                        .map(|insert| {
                            let side = dot(normal, far);
                            let on_line = insert
                                .iter()
                                .copied()
                                .filter(|p| (dot(normal, *p) - side).abs() <= 1e-9);
                            reach(&mut on_line.into_iter(), d)
                        })
                        .fold(f64::INFINITY, f64::min);
                    let slack = if corners.len() == 6 { 0.2 } else { 1e-6 };
                    assert!(
                        shortest >= dot(d, far) - 1e-9 && shortest <= dot(d, far) + slack,
                        "{code} {k}: the certain edge ends at {}, the shortest of the model at {shortest}",
                        dot(d, far)
                    );
                }
                // The nominal outline is inside it, and the nose and the
                // edges at it are on its boundary (nothing grows there).
                let nominal = outline.polygon_points();
                for p in &nominal {
                    assert!(outside(&env.largest, *p) <= 1e-9, "{code} {k}: {p:?}");
                }
                let ends = outline.ends();
                for segment in cutting {
                    for p in [ends[*segment].0, ends[*segment].1] {
                        assert!(outside(&env.largest, p).abs() <= 1e-9, "{code} {k}");
                    }
                }
                // Every zone lies in the envelope.
                for (zone, normals) in &env.back {
                    assert!(!normals.is_empty());
                    for p in zone {
                        assert!(outside(&env.largest, *p) <= 1e-9, "{code} {k}");
                    }
                }
            }
        }
    }

    #[test]
    fn a_class_m_cnmg_grows_by_the_derived_amounts() {
        // CNMG120408, class M, 12.7 mm: d ±0.08, m ±0.13. Far sides
        // (a = sin 40): (A) 0.08; (B) 0.13 sin 40 + 0.08 (1 + sin 40)/2.
        // Far corner (c = 1): 2 x 0.13 + 0.08 beyond its nominal tip.
        let (insert, _) = Insert::iso_code("CNMG120408", 95.).unwrap();
        let Insert::Outline {
            outline,
            corner,
            cutting,
            ..
        } = &insert
        else {
            unreachable!()
        };
        let tolerance = SizeTolerance {
            d_mm: 0.08,
            m_mm: 0.13,
        };
        let env = envelope(outline, *corner, cutting, &tolerance).unwrap();
        let nominal = outline.polygon_points();
        let s = 40f64.to_radians().sin();
        // (C), the vee-block reading: one far side at -0.08, the other up
        // to 2 x 0.13 sin 40 + 0.08 = 0.247: the largest of the three.
        assert!(0.13 * s + 0.08 * (1. + s) / 2. > 0.08);
        let side = 2. * 0.13 * s + 0.08;
        assert!(side > 0.13 * s + 0.08 * (1. + s) / 2. && (side - 0.2471).abs() < 1e-4);
        // The far sides face 95 - 90 + 80 + 90 = 175 and 95 - 180 = -85
        // degrees... as outward normals: the major edge's far parallel
        // faces +z (5 degrees toward +x of it), the minor edge's +x.
        for n in [unit(5. + 90.), unit(85. - 90.)] {
            let grown = support(&env.largest, n) - support(&nominal, n);
            assert!((grown - side).abs() < 1e-9, "{grown} {side}");
        }
        let u = unit(5. + 40.);
        let grown = support(&env.largest, u) - support(&nominal, u);
        assert!((grown - (2. * 0.13 + 0.08)).abs() < 1e-3, "{grown}");
        // Along the edges at the nose, and at the nose, nothing grows.
        for n in [unit(5. - 90.), unit(85. + 90.), unit(5. + 40. + 180.)] {
            let grown = support(&env.largest, n) - support(&nominal, n);
            assert!(grown.abs() < 1e-9, "{grown}");
        }
    }

    #[test]
    fn outlines_that_are_not_of_the_iso_form_are_refused_by_name() {
        let tolerance = SizeTolerance {
            d_mm: 0.08,
            m_mm: 0.13,
        };
        // The wedge (straight across its back: its sides share no circle).
        let Insert::Outline {
            outline,
            corner,
            cutting,
            ..
        } = Insert::iso_turning(80., 95., 12., 0.8).unwrap()
        else {
            unreachable!()
        };
        let e = envelope(&outline, corner, &cutting, &tolerance)
            .err()
            .unwrap();
        assert!(e.contains("ISO 1832 form"), "{e}");
        // A pentagon and an octagon: not worked out.
        for (corners, k) in [(&[108.; 5][..], 60.), (&[135.; 8][..], 40.)] {
            let sides = corners.len();
            let side = 12.7 * (std::f64::consts::PI / sides as f64).tan();
            let Insert::Outline {
                outline,
                corner,
                cutting,
                ..
            } = Insert::polygon_insert(corners, side, 0.8, k).unwrap()
            else {
                unreachable!()
            };
            let e = envelope(&outline, corner, &cutting, &tolerance)
                .err()
                .unwrap();
            assert!(e.contains(&format!("with {sides} sides")), "{e}");
        }
        // A round insert.
        let (round, _) = Insert::iso_code("RCMT10T3M0", 95.).unwrap();
        let Insert::Outline {
            outline,
            corner,
            cutting,
            ..
        } = &round
        else {
            unreachable!()
        };
        let e = envelope(outline, *corner, cutting, &tolerance)
            .err()
            .unwrap();
        assert!(e.contains("ISO 1832 form"), "{e}");
        // A far side that cuts.
        let (insert, _) = Insert::iso_code("CNMG120408", 95.).unwrap();
        let Insert::Outline {
            outline, corner, ..
        } = &insert
        else {
            unreachable!()
        };
        let e = envelope(outline, *corner, &[0, 1, 2, 4], &tolerance)
            .err()
            .unwrap();
        assert!(e.contains("only the nose"), "{e}");
        // A tolerance larger than the insert's sides allow.
        let (small, _) = Insert::iso_code("CCMT060204", 95.).unwrap();
        let Insert::Outline {
            outline,
            corner,
            cutting,
            ..
        } = &small
        else {
            unreachable!()
        };
        // Clockwise, the same outline gives the same envelope.
        let reversed = reverse(outline);
        let a = envelope(outline, *corner, cutting, &tolerance).unwrap();
        let count = outline.segments.len();
        let b = envelope(
            &reversed.0,
            reversed.1(*corner),
            &cutting.iter().map(|c| reversed.1(*c)).collect::<Vec<_>>(),
            &tolerance,
        )
        .unwrap();
        assert_eq!(count, reversed.0.segments.len());
        assert_eq!(a.largest.len(), b.largest.len());
        for (p, q) in a.largest.iter().zip(&b.largest) {
            assert!((p[0] - q[0]).hypot(p[1] - q[1]) < 1e-9, "{p:?} {q:?}");
        }
    }

    /// The outline traversed the other way, and where each segment went.
    fn reverse(outline: &Outline) -> (Outline, impl Fn(usize) -> usize) {
        let ends = outline.ends();
        let count = outline.segments.len();
        let segments = (0..count)
            .rev()
            .map(|k| match &outline.segments[k] {
                Segment::Line { .. } => Segment::Line { to_mm: ends[k].0 },
                Segment::Arc {
                    center_mm,
                    radius_mm,
                    start_deg,
                    end_deg,
                } => Segment::Arc {
                    center_mm: *center_mm,
                    radius_mm: *radius_mm,
                    start_deg: *end_deg,
                    end_deg: *start_deg,
                },
            })
            .collect();
        (
            Outline {
                start_mm: outline.start_mm,
                segments,
            },
            move |k| count - 1 - k,
        )
    }
}
