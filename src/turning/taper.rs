//! Radial extent of a circular cutting edge swept along a fixed XZ line.
//! X is radius in mm, including when the controller writes diameter words.
use super::ops::{ControlledPoint, TurnOperation, TurnSetup};
use super::profile::Seg;
use super::stock::TurnSide;
use super::tool::{Hand, Insert, Segment, TurningTool};
use crate::contour::Compensation;
use crate::interval::Bounds;
use crate::motion::{Curve, Motion, MoveKind};

/// Evaluate a decoded finishing move against one segment of the drawing.
pub fn section(
    setup: &TurnSetup,
    motion: &Motion,
    segment: usize,
    radii: [f64; 2],
    z: f64,
) -> Result<[[f64; 2]; 2], String> {
    let operation = setup
        .operations
        .iter()
        .find(|o| o.id() == motion.operation && o.tool() == motion.tool)
        .ok_or("decoded operation and tool must match the turning setup")?;
    let profile = match operation {
        TurnOperation::Finish {
            compensation: Compensation::Computer,
            ..
        } => &setup.part,
        TurnOperation::BoreFinish {
            compensation: Compensation::Computer,
            ..
        } => setup
            .bore
            .as_ref()
            .ok_or("bore finishing requires a bore profile")?,
        _ => {
            return Err(
                "taper analysis requires a computer-compensated finishing operation".into(),
            );
        }
    };
    let segments = profile.segments()?;
    let Some(Seg::Line { a, b }) = segments.get(segment) else {
        return Err("profile_segment must select a straight segment of the drawing".into());
    };
    if !(a[0].min(b[0]) < z && z < a[0].max(b[0])) {
        return Err("Z station must be inside the selected drawing segment".into());
    }
    let point = Bounds::point;
    let target = point(a[1]).add(
        point(z)
            .sub(point(a[0]))
            .div(point(b[0]).sub(point(a[0])))
            .mul(point(b[1]).sub(point(a[1]))),
    );
    let tool = &setup
        .tools
        .iter()
        .find(|s| s.tool.id == motion.tool)
        .ok_or("missing turning tool")?
        .tool;
    let radius = round_taper_radius(tool, motion, setup.controlled_point, radii, z)?;
    let error = Bounds {
        lo: radius[0],
        hi: radius[1],
    }
    .sub(target);
    if !error.lo.is_finite() || !error.hi.is_finite() {
        return Err("radial error exceeds numerical range".into());
    }
    Ok([radius, [error.lo, error.hi]])
}

/// Bounds on the radius reached by this cut at a single axial station.
/// The station must lie on the straight flank for every allowed radius.
fn round_taper_radius(
    tool: &TurningTool,
    motion: &Motion,
    datum: ControlledPoint,
    radii: [f64; 2],
    z: f64,
) -> Result<[f64; 2], String> {
    tool.validate()?;
    let Insert::Outline {
        outline,
        programming_point_mm,
        cutting,
        ..
    } = &tool.insert
    else {
        return Err("taper analysis requires a circular cutting insert".into());
    };
    let nominal = tool.nose_radius_mm;
    if !matches!(outline.segments.as_slice(), [Segment::Arc {center_mm, radius_mm, start_deg, end_deg}]
        if *center_mm == [0., 0.] && *radius_mm == nominal && (*end_deg - *start_deg).abs() == 360.)
        || cutting.as_slice() != [0]
    {
        return Err(
            "taper analysis requires one fully cutting circular edge centered at the tool origin"
                .into(),
        );
    }
    if datum == ControlledPoint::Tip && *programming_point_mm != [-nominal, -nominal] {
        return Err("tip datum needs the intersection of the diameter and face tangents".into());
    }
    let Curve::Line { start, end } = motion.curve else {
        return Err("taper analysis requires a straight cutting move".into());
    };
    if motion.tool != tool.id
        || motion.kind != MoveKind::Cut
        || start[1] != 0.
        || end[1] != 0.
        || !start
            .into_iter()
            .chain(end)
            .chain(radii)
            .chain([z])
            .all(f64::is_finite)
        || start[2] == end[2]
        || radii[0] <= 0.
        || radii[0] > nominal
        || radii[1] < nominal
    {
        return Err("give an XZ cutting line of this tool with axial travel, a finite station and positive radius limits containing its nominal radius".into());
    }

    let side = if tool.side == TurnSide::Outer {
        1.
    } else {
        -1.
    };
    let hand = if tool.hand == Hand::Right { 1. } else { -1. };
    surface(start, end, side, hand, datum, radii, z)
}

fn surface(
    start: [f64; 3],
    end: [f64; 3],
    side: f64,
    hand: f64,
    datum: ControlledPoint,
    radii: [f64; 2],
    z: f64,
) -> Result<[f64; 2], String> {
    let b = Bounds::point;
    let slope = b(end[0]).sub(b(start[0])).div(b(end[2]).sub(b(start[2])));
    let norm2 = slope.square().add(b(1.));
    let norm = Bounds {
        lo: norm2.lo.sqrt().next_down(),
        hi: norm2.hi.sqrt().next_up(),
    };
    if !norm.hi.is_finite() {
        return Err("taper slope exceeds numerical range".into());
    }
    let (dx, dz) = if datum == ControlledPoint::Tip {
        (side, hand)
    } else {
        (0., 0.)
    };
    let rr = Bounds {
        lo: radii[0],
        hi: radii[1],
    };
    // The outward unit normal of the target line is side*(1,-slope)/norm.
    // Contact is one radius along its negative from the nose centre.
    // Stay on the positive-radius side of the spindle. A circle crossing
    // the axis cannot be interpreted as a single radial flank.
    if b(start[0].min(end[0])).add(rr.mul(b(dx - 1.))).lo <= 0. {
        return Err("round-insert sweep may reach the spindle axis".into());
    }
    let program_z = b(z).sub(rr.mul(b(dz).add(b(side).mul(slope).div(norm))));
    if !program_z.lo.is_finite()
        || !program_z.hi.is_finite()
        || program_z.lo < start[2].min(end[2])
        || program_z.hi > start[2].max(end[2])
    {
        return Err(
            "station reaches the move end for a possible insert; choose an interior station".into(),
        );
    }
    // Substituting the contact point's Z in the centre line leaves an
    // affine function of the *same* radius in the datum shift and edge.
    let coeff = b(dx).sub(slope.mul(b(dz))).sub(b(side).mul(norm));
    let at_z = b(start[0]).add(slope.mul(b(z).sub(b(start[2]))));
    let result = at_z.add(rr.mul(coeff));
    if !result.lo.is_finite() || !result.hi.is_finite() || result.lo < 0. {
        return Err("round-insert surface crosses the axis or exceeds numerical range".into());
    }
    Ok([result.lo, result.hi])
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn offset_centres_and_finite_tangency() {
        // A 3:4:5 line. For each radius the radial offset at fixed Z is
        // 5r/4. Touch-off additionally translates the line in X and Z.
        for side in [-1., 1.] {
            for hand in [-1., 1.] {
                for datum in [ControlledPoint::Tip, ControlledPoint::NoseCenter] {
                    let got = surface(
                        [20., 0., 0.],
                        [26., 0., 8.],
                        side,
                        hand,
                        datum,
                        [0.5, 1.5],
                        4.,
                    )
                    .unwrap();
                    let mut exact = [0.; 2];
                    for (i, r) in [0.5, 1.5].into_iter().enumerate() {
                        let shift = if datum == ControlledPoint::Tip {
                            r * (side - 0.75 * hand)
                        } else {
                            0.
                        };
                        exact[i] = 23. + shift - side * 1.25 * r;
                    }
                    exact.sort_by(f64::total_cmp);
                    assert!(got[0] <= exact[0] && got[1] >= exact[1]);
                    assert!(exact[0] - got[0] < 1e-12 && got[1] - exact[1] < 1e-12);
                }
            }
        }
        assert!(
            surface(
                [20., 0., 0.],
                [26., 0., 8.],
                1.,
                1.,
                ControlledPoint::Tip,
                [0.5, 1.5],
                2.
            )
            .is_err()
        );
    }
}
