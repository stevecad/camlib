//! Per-move gouge proof against a surface target (#72).
//!
//! The whole-cell height comparison cannot decide a cell that a vertical
//! wall crosses: a cutter run tangent to the wall and one a micrometre into
//! it leave identical cell bounds. This module proves each motion instead,
//! against the target mesh, and the material check then decides a
//! possible-gouge cell as within tolerance when no unproven motion's
//! footprint touches it.
//!
//! Claim, for comparison tolerance τ and the mesh's declared surface error
//! e: no point the tool sweeps lies deeper than τ inside the true part P*.
//!
//! - M is the solid under the mesh (the height-field semantics of the
//!   material check, with its base plane). P* lies within e of M; both are
//!   down-closed, so P* eroded by e lies inside M: from a point p whose
//!   e-ball is in P*, go straight down (the ball stays in P*) until the ray
//!   enters M; that boundary point would be at least e from P*'s boundary.
//! - Every tool section (flat to r0, then rising linearly; `flat::Section`)
//!   is shrunk radially by a and lifted by a. Each point of the real tool is
//!   within k·a of that eroded tool: k = √2 where the section is flat out
//!   to r0 >= a (move the point a inward and a up), and √(1 + (1 + rise)²)
//!   near a pointed tip. Ball and bull ends are replaced by their enclosing
//!   flat end (a superset: sound, only conservative).
//! - With k·a < τ - e, an eroded tool proven clear of M along the move
//!   (`Index::sections_chord_clear`: contact against every triangle, the
//!   gap bounded by concavity) proves the claim. A swept point q deeper
//!   than τ in P* has an eroded-tool point p within k·a, deeper than e in
//!   P*, hence strictly inside M, which the eroded tool does not enter.
//! - Arcs and helices are proven chord by chord. A point at fraction f of a
//!   sub-arc of half angle θ is within R[(1 - cos θ) + (θ - sin θ)] of the
//!   chord's point at f, so the chord's sections grow radially by that
//!   (and lift by it when the arc's plane is not XY).
//!
//! A cell is decided only when the stream started from the raw stock (the
//! whole history was proven), and never from a definite gouge.
use super::*;
use crate::flat::{Index, Section, enclosing_sections};
use crate::motion::{Curve, Motion, Plane, Vec3};
use crate::tool::PreparedAssembly;
use std::collections::HashMap;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SweepProofReport {
    /// The proven depth bound: no swept point deeper than this inside the
    /// true part (the comparison tolerance).
    pub tolerance_mm: f64,
    /// The target mesh's declared surface error, spent from the tolerance.
    pub surface_error_mm: f64,
    pub proven_moves: u64,
    pub unproven_moves: u64,
    /// Whether every motion since the raw stock was seen (a resumed stream
    /// proves nothing about the motions before it).
    pub history_complete: bool,
    /// The report's possible-only gouge cells the proof decides (no
    /// unproven move touches them): when it equals the report's
    /// `possible_only_cells`, the gouge is within tolerance.
    pub decided_cells: usize,
    /// Why no move could be proven, when none could.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub unavailable: Option<String>,
}

struct ToolProof {
    /// The eroded sections and the erosion a.
    sections: Vec<Section>,
    erosion: f64,
    /// The real assembly's largest radius (unproven footprints).
    reach: f64,
}

pub(crate) struct SweepProof {
    index: Option<Index>,
    unavailable: Option<String>,
    tolerance: f64,
    surface_error: f64,
    budget: f64,
    spec: StockSpec,
    nx: usize,
    ny: usize,
    step: [f64; 2],
    /// Stock cells some unproven motion's footprint touches.
    touched: Vec<bool>,
    history_complete: bool,
    proven: u64,
    unproven: u64,
    tools: HashMap<String, Option<ToolProof>>,
    seen: Vec<u32>,
    stamp: u32,
}

impl SweepProof {
    pub(crate) fn new(
        target: &SurfaceTarget,
        options: Options,
        spec: &StockSpec,
        history_complete: bool,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        let (nx, ny, step) = spec.grid()?;
        let tolerance = options.comparison_tolerance_mm;
        let surface_error = target.surface.surface_error_mm;
        let scale = spec
            .min_mm
            .into_iter()
            .chain(spec.max_mm)
            .map(f64::abs)
            .fold(1., f64::max);
        // Strict margin for the containment argument and binary64 rounding.
        let budget = tolerance - surface_error - 1024. * f64::EPSILON * scale;
        let (index, unavailable) = if budget > 0. {
            // Below the stock box there is no stock to gouge: the base plane
            // binds only when it lies above the stock's bottom.
            let floor = if target.base_z_mm > spec.min_mm[2] {
                target.base_z_mm
            } else {
                spec.min_mm[2] - 1e6
            };
            (Some(Index::new(&target.surface, floor, 1., cancel)?), None)
        } else {
            (
                None,
                Some(format!(
                    "the comparison tolerance {tolerance} mm does not exceed the target mesh's surface error {surface_error} mm: no move can be proven within it (raise comparison_tolerance_mm or export a finer mesh)"
                )),
            )
        };
        let seen = vec![0; index.as_ref().map_or(0, Index::len)];
        Ok(Self {
            index,
            unavailable,
            tolerance,
            surface_error,
            budget,
            spec: spec.clone(),
            nx,
            ny,
            step,
            touched: vec![false; nx * ny],
            history_complete,
            proven: 0,
            unproven: 0,
            tools: HashMap::new(),
            seen,
            stamp: 0,
        })
    }

    fn tool(&mut self, tool: &PreparedAssembly) -> Result<(), String> {
        let id = &tool.definition().id;
        if self.tools.contains_key(id) {
            return Ok(());
        }
        let proof = match enclosing_sections(tool) {
            Err(_) => None,
            Ok(sections) => {
                let reach = sections.iter().map(|s| s.r1).fold(0., f64::max);
                let flat = self.budget / std::f64::consts::SQRT_2;
                let k = sections
                    .iter()
                    .map(|s| {
                        if s.r0 >= flat {
                            std::f64::consts::SQRT_2
                        } else {
                            (1. + (1. + s.rise).powi(2)).sqrt()
                        }
                    })
                    .fold(std::f64::consts::SQRT_2, f64::max);
                let a = self.budget / k;
                let sections = sections
                    .iter()
                    .filter(|s| s.r1 > a)
                    .map(|s| Section {
                        z0: s.z0 + s.rise * (a - s.r0).max(0.),
                        r0: (s.r0 - a).max(0.),
                        r1: s.r1 - a,
                        rise: s.rise,
                    })
                    .collect::<Vec<_>>();
                (!sections.is_empty() && sections.iter().all(Section::valid)).then_some(ToolProof {
                    sections,
                    erosion: a,
                    reach,
                })
            }
        };
        self.tools.insert(id.clone(), proof);
        Ok(())
    }

    /// Prove one motion; an unproven motion marks the stock cells its
    /// footprint (the whole assembly's reach) touches.
    /// `tolerance_mm`: the stream's curve tolerance (`SweepOptions`), which
    /// validated the motion already (a posted arc's rounded endpoints agree
    /// with its centre only to that).
    pub(crate) fn motion(
        &mut self,
        tool: &PreparedAssembly,
        motion: &Motion,
        tolerance_mm: f64,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        self.tool(tool)?;
        let proven = self.prove(&tool.definition().id, &motion.curve, tolerance_mm, cancel)?;
        if proven {
            self.proven += 1;
            return Ok(());
        }
        self.unproven += 1;
        let reach = match self.tools.get(&tool.definition().id) {
            Some(Some(p)) => p.reach,
            _ => tool
                .definition()
                .holders
                .iter()
                .map(|h| h.radius_max_mm.max(h.radius_min_mm))
                .fold(tool.cutter().definition().diameter_mm / 2., f64::max),
        };
        let bounds = motion.curve.prepare(tolerance_mm)?.bounds();
        let pad = reach + self.step[0].max(self.step[1]);
        let cell = |v: f64, k: usize, n: usize| {
            (((v - self.spec.min_mm[k]) / self.step[k]).floor().max(0.) as usize).min(n - 1)
        };
        let (x0, x1) = (
            cell(bounds.min[0] - pad, 0, self.nx),
            cell(bounds.max[0] + pad, 0, self.nx),
        );
        let (y0, y1) = (
            cell(bounds.min[1] - pad, 1, self.ny),
            cell(bounds.max[1] + pad, 1, self.ny),
        );
        if bounds.max[0] + pad >= self.spec.min_mm[0]
            && bounds.min[0] - pad <= self.spec.max_mm[0]
            && bounds.max[1] + pad >= self.spec.min_mm[1]
            && bounds.min[1] - pad <= self.spec.max_mm[1]
        {
            for y in y0..=y1 {
                self.touched[y * self.nx + x0..=y * self.nx + x1].fill(true);
            }
        }
        Ok(())
    }

    fn prove(
        &mut self,
        tool: &str,
        curve: &Curve,
        tolerance_mm: f64,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        let (Some(index), Some(Some(proof))) = (&self.index, self.tools.get(tool)) else {
            return Ok(false);
        };
        let a = proof.erosion;
        let chord = |p: Vec3, q: Vec3, grow: f64, lift: f64, seen: &mut [u32], stamp: &mut u32| {
            let sections = proof
                .sections
                .iter()
                .map(|s| Section {
                    z0: s.z0,
                    r0: s.r0 + grow,
                    r1: s.r1 + grow,
                    rise: s.rise,
                })
                .collect::<Vec<_>>();
            // The eroded tool is lifted by a; any extra lift is a required
            // clearance for sub-arc deviation out of the XY plane.
            index.sections_chord_clear(p, q, &sections, lift - a, seen, stamp)
        };
        match *curve {
            Curve::Line { start, end } => {
                Ok(chord(start, end, 0., 0., &mut self.seen, &mut self.stamp))
            }
            Curve::Arc {
                start,
                center,
                plane,
                sweep_rad,
                ..
            }
            | Curve::Helix {
                start,
                center,
                plane,
                sweep_rad,
                ..
            } => {
                let [u, v, _] = plane.axes();
                let r = (start[u] - center[u]).hypot(start[v] - center[v]);
                let prepared = curve.prepare(tolerance_mm)?;
                // Adaptive sub-chords: a piece of half angle θ lies within
                // d(θ) = R[(1 - cos θ) + (θ - sin θ)] of its chord at equal
                // fractions (θ <= π/4 here), so its chord is proven with the
                // sections grown by d (and lifted by d out of the XY
                // plane). A piece that fails is halved (d falls about 4x)
                // until d is a quarter of the erosion; failing there, the
                // move is unproven. Open-air arcs prove in one chord.
                let deviation = |f0: f64, f1: f64| {
                    let half = sweep_rad.abs() * (f1 - f0) / 2.;
                    r * ((1. - half.cos()) + (half - half.sin()))
                };
                let quarters = (sweep_rad.abs() / std::f64::consts::FRAC_PI_2)
                    .ceil()
                    .max(1.) as usize;
                let mut stack: Vec<(f64, f64)> = (0..quarters)
                    .rev()
                    .map(|k| (k as f64 / quarters as f64, (k + 1) as f64 / quarters as f64))
                    .collect();
                let mut chords = 0u32;
                while let Some((f0, f1)) = stack.pop() {
                    chords += 1;
                    if chords > 100_000 {
                        return Ok(false);
                    }
                    if chords.is_multiple_of(256) && cancel.is_cancelled() {
                        return Err("material sweep proof cancelled".into());
                    }
                    let d = deviation(f0, f1);
                    let lift = if plane == Plane::Xy { 0. } else { d };
                    let (p, q) = (prepared.position(f0)?, prepared.position(f1)?);
                    if chord(p, q, d, lift, &mut self.seen, &mut self.stamp) {
                        continue;
                    }
                    if d <= a / 4. {
                        return Ok(false);
                    }
                    let m = (f0 + f1) / 2.;
                    stack.push((m, f1));
                    stack.push((f0, m));
                }
                Ok(true)
            }
        }
    }

    /// Cells the proof decides: the whole history proven and no unproven
    /// motion's footprint on the cell.
    pub(crate) fn decides(&self, cell: usize) -> bool {
        self.history_complete && self.index.is_some() && !self.touched[cell]
    }

    pub(crate) fn report(&self, decided_cells: usize) -> SweepProofReport {
        SweepProofReport {
            tolerance_mm: self.tolerance,
            surface_error_mm: self.surface_error,
            proven_moves: self.proven,
            unproven_moves: self.unproven,
            history_complete: self.history_complete,
            decided_cells,
            unavailable: self.unavailable.clone().or_else(|| {
                (!self.history_complete)
                    .then(|| "the stream resumed after motions this check did not see".to_string())
            }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::MoveKind;
    use crate::surface::SurfaceMesh;
    use crate::tool::{Cutter, CutterShape, HolderSection, ToolAssembly};
    use std::sync::atomic::AtomicBool;

    const PART: (Vec3, Vec3) = ([0., 0., 0.], [20., 20., 10.]);

    fn cuboid(lo: Vec3, hi: Vec3) -> Vec<[Vec3; 3]> {
        let p = |x: usize, y: usize, z: usize| {
            [[lo[0], hi[0]][x], [lo[1], hi[1]][y], [lo[2], hi[2]][z]]
        };
        let quad = |a: Vec3, b: Vec3, c: Vec3, d: Vec3| vec![[a, b, c], [a, c, d]];
        [
            quad(p(0, 0, 1), p(1, 0, 1), p(1, 1, 1), p(0, 1, 1)),
            quad(p(0, 0, 0), p(0, 1, 0), p(1, 1, 0), p(1, 0, 0)),
            quad(p(0, 0, 0), p(1, 0, 0), p(1, 0, 1), p(0, 0, 1)),
            quad(p(0, 1, 0), p(0, 1, 1), p(1, 1, 1), p(1, 1, 0)),
            quad(p(0, 0, 0), p(0, 0, 1), p(0, 1, 1), p(0, 1, 0)),
            quad(p(1, 0, 0), p(1, 1, 0), p(1, 1, 1), p(1, 0, 1)),
        ]
        .concat()
    }

    fn target(error: f64) -> SurfaceTarget {
        SurfaceTarget {
            version: 1,
            units: "mm".into(),
            id: "box".into(),
            revision: "1".into(),
            surface: SurfaceMesh {
                version: 1,
                units: "mm".into(),
                triangles: std::sync::Arc::new(cuboid(PART.0, PART.1)),
                surface_error_mm: error,
                source: serde_json::json!({}),
            },
            base_z_mm: 0.,
            source_enclosure: SurfaceEnclosure::Bounded,
            contact_witnesses: None,
            inner_patches: vec![],
            finishing: None,
        }
    }

    fn spec() -> StockSpec {
        StockSpec {
            version: 1,
            units: "mm".into(),
            min_mm: [-15., -15., 0.],
            max_mm: [35., 35., 12.],
            cell_size_mm: 0.5,
            max_cells: u32::MAX as usize,
        }
    }

    fn tool(shape: CutterShape) -> ToolAssembly {
        ToolAssembly {
            id: "t".into(),
            cutter: Cutter {
                id: "t".into(),
                name: String::new(),
                shape,
                diameter_mm: 6.,
                cutting_length_mm: 16.,
                overall_length_mm: 60.,
                shank_diameter_mm: 6.,
                flutes: 2,
                center_cutting: true,
                material: "carbide".into(),
                neck: None,
            },
            stickout_mm: 30.,
            gauge_length_mm: 80.,
            holders: vec![HolderSection {
                z_min_mm: 30.,
                z_max_mm: 80.,
                radius_min_mm: 12.,
                radius_max_mm: 20.,
            }],
        }
    }

    fn motion(curve: Curve) -> Motion {
        Motion {
            id: 1,
            operation: "op".into(),
            tool: "t".into(),
            kind: MoveKind::Cut,
            curve,
            feed_mm_min: Some(600.),
        }
    }

    /// Independent oracle: the deepest sampled point of the real tool (its
    /// own radius profile, solid) inside the box, along the move.
    fn deepest(tool: &PreparedAssembly, curve: &Curve) -> f64 {
        let prepared = curve.prepare(1e-6).unwrap();
        let (lo, hi) = PART;
        let mut worst: f64 = 0.;
        for i in 0..=100 {
            let c = prepared.position(i as f64 / 100.).unwrap();
            for k in 0..=60 {
                let h = 12. * k as f64 / 60.;
                let Some(slice) = tool.radius_at(h) else {
                    continue;
                };
                for j in 0..=12 {
                    let rho = slice.radius_mm * j as f64 / 12.;
                    for m in 0..72 {
                        let phi = std::f64::consts::TAU * m as f64 / 72.;
                        let p = [c[0] + rho * phi.cos(), c[1] + rho * phi.sin(), c[2] + h];
                        let depth = (0..3)
                            .map(|a| (p[a] - lo[a]).min(hi[a] - p[a]))
                            .fold(f64::INFINITY, f64::min);
                        worst = worst.max(depth);
                    }
                }
            }
        }
        worst
    }

    fn line(a: Vec3, b: Vec3) -> Curve {
        Curve::Line { start: a, end: b }
    }

    fn proves(proof: &mut SweepProof, tool: &PreparedAssembly, curve: Curve) -> bool {
        let before = proof.proven;
        proof
            .motion(tool, &motion(curve), 1e-5, &AtomicBool::new(false))
            .unwrap();
        proof.proven > before
    }

    fn proof(tolerance: f64, error: f64, complete: bool) -> SweepProof {
        let options = Options {
            comparison_tolerance_mm: tolerance,
            ..Options::default()
        };
        SweepProof::new(
            &target(error),
            options,
            &spec(),
            complete,
            &AtomicBool::new(false),
        )
        .unwrap()
    }

    #[test]
    fn tangent_wall_passes_prove_and_cuts_into_the_wall_do_not() {
        let flat = tool(CutterShape::Flat).prepare().unwrap();
        // A contour pass 3 mm off the x = 0 wall (radius 3), down at Z 4;
        // a positive offset moves it into the wall.
        for (offset, tolerance, error, expect) in [
            (0., 0.005, 0.0005, true),
            (0.0007, 0.005, 0.0005, true),
            (0.01, 0.005, 0.0005, false),
            (0.0007, 0.001, 0.00088, false),
            (-0.01, 0.001, 0.00088, true),
        ] {
            let mut p = proof(tolerance, error, true);
            let x = -3. + offset;
            let curve = line([x, -5., 4.], [x, 25., 4.]);
            let proven = proves(&mut p, &flat, curve.clone());
            assert_eq!(proven, expect, "offset {offset} tolerance {tolerance}");
            if proven {
                assert!(deepest(&flat, &curve) <= tolerance - error);
            }
        }
    }

    #[test]
    fn proven_moves_never_penetrate_deeper_than_the_tolerance() {
        // Lines, XY arcs and helices around the box, near its walls and
        // top, for flat, drill and ball ends; every proven move is checked
        // against the oracle, and clear moves well away must prove.
        let tools = [
            tool(CutterShape::Flat),
            tool(CutterShape::Drill {
                point_angle_rad: 118f64.to_radians(),
            }),
            tool(CutterShape::Ball),
        ]
        .map(|t| t.prepare().unwrap());
        let (tolerance, error) = (0.005, 0.0005);
        let mut seed = 0x9e3779b97f4a7c15u64;
        let mut rand = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        let (mut proven, mut refused) = (0, 0);
        for i in 0..90 {
            let t = &tools[i % 3];
            let mut p = proof(tolerance, error, true);
            // A centre 3 mm (+/- 0.02) off a wall, or over the top.
            let off = 3. + (rand() - 0.5) * 0.04;
            let z = if i % 4 == 0 {
                10. + (rand() - 0.5) * 0.04
            } else {
                2. + 6. * rand()
            };
            let x = if i % 2 == 0 { -off } else { 20. + off };
            let curve = match i % 3 {
                0 => line(
                    [x, -4. + 8. * rand(), z],
                    [x, 16. + 8. * rand(), z + (rand() - 0.5) * 0.02],
                ),
                1 => {
                    // An XY arc of radius 3 + small about a centre on the
                    // wall line's outside, sweeping past the wall.
                    let r = 2. + rand();
                    let center = [
                        x + if x < 0. {
                            -(off - 3.) - r + 3.
                        } else {
                            (off - 3.) + r - 3.
                        },
                        10.,
                        z,
                    ];
                    let a0: f64 = if x < 0. {
                        -1.
                    } else {
                        std::f64::consts::PI - 1.
                    };
                    let start = [center[0] + r * a0.cos(), center[1] + r * a0.sin(), z];
                    let sweep = 2.;
                    let end = [
                        center[0] + r * (a0 + sweep).cos(),
                        center[1] + r * (a0 + sweep).sin(),
                        z,
                    ];
                    Curve::Arc {
                        start,
                        end,
                        center,
                        plane: Plane::Xy,
                        sweep_rad: sweep,
                    }
                }
                _ => {
                    let r = 1.5;
                    let cx = if x < 0. { x - r } else { x + r };
                    let a0: f64 = if x < 0. { 0. } else { std::f64::consts::PI };
                    let start = [cx + r * a0.cos(), 10. + r * a0.sin(), z];
                    let sweep = std::f64::consts::TAU;
                    let end = [start[0], start[1], z - 0.5];
                    Curve::Helix {
                        start,
                        end,
                        center: [cx, 10., z],
                        plane: Plane::Xy,
                        sweep_rad: sweep,
                    }
                }
            };
            if proves(&mut p, t, curve.clone()) {
                proven += 1;
                let depth = deepest(t, &curve);
                assert!(
                    depth <= tolerance - error,
                    "move {i}: proven, oracle depth {depth}"
                );
            } else {
                refused += 1;
            }
        }
        assert!(
            proven > 20 && refused > 5,
            "proven {proven}, refused {refused}"
        );
        // Well clear (0.05 mm off the walls): always proven.
        for t in &tools {
            let mut p = proof(tolerance, error, true);
            assert!(proves(&mut p, t, line([-3.05, -5., 3.], [-3.05, 25., 3.])));
            assert!(proves(
                &mut p,
                t,
                line([-5., -5., 10.05], [25., -5., 10.05])
            ));
        }
    }

    #[test]
    fn posted_arcs_are_proven_at_the_stream_tolerance() {
        // A 3-decimal arc's end agrees with its centre only to the stream's
        // tolerance (here 5e-6 mm off the radius): proven, never an error.
        let flat = tool(CutterShape::Flat).prepare().unwrap();
        let mut p = proof(0.005, 0.0005, true);
        let arc = Curve::Arc {
            start: [-8., 10., 11.],
            end: [-8., 15.000005, 11.],
            center: [-8., 12.5, 11.],
            plane: Plane::Xy,
            sweep_rad: std::f64::consts::PI,
        };
        assert!(arc.prepare(1e-6).is_err());
        assert!(proves(&mut p, &flat, arc));
    }

    #[test]
    fn resumed_history_or_spent_tolerance_decides_nothing() {
        let flat = tool(CutterShape::Flat).prepare().unwrap();
        let mut resumed = proof(0.005, 0.0005, false);
        assert!(proves(
            &mut resumed,
            &flat,
            line([-5., -5., 11.], [25., -5., 11.])
        ));
        assert!(!resumed.decides(0));
        let mut spent = proof(0.001, 0.001, true);
        assert!(!proves(
            &mut spent,
            &flat,
            line([-10., -10., 11.], [-9., -10., 11.])
        ));
        assert!(
            spent
                .report(0)
                .unavailable
                .unwrap()
                .contains("surface error")
        );
        // An unproven move marks the cells under its footprint only.
        let mut p = proof(0.005, 0.0005, true);
        assert!(!proves(&mut p, &flat, line([-2., 5., 4.], [-2., 6., 4.])));
        let cell = |x: f64, y: f64| (((y + 15.) / 0.5) as usize) * 100 + ((x + 15.) / 0.5) as usize;
        assert!(!p.decides(cell(-2., 5.5)));
        assert!(p.decides(cell(30., 30.)));
    }
}
