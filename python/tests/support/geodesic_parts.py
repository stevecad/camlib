"""Synthetic CAD parts for geodesic finishing (#12), the source of truth for
the STEP files in fixtures/geodesic/ (regenerate those with `export_steps`).

- rounded_block: a 40 x 30 x 10 mm block, its top edges filleted 4 mm: flat,
  cylindrical and spherical faces meeting tangentially.
- bowl: a 44 x 44 x 12 mm block with a spherical bowl (radius 25 mm, 9 mm
  deep) cut into its top: concave and wider than a 6 mm ball. The sphere's
  axis is horizontal, so its poles lie outside the part.
- plate_with_hole: a 40 x 30 x 6 mm plate with a 10 x 8 mm through-hole:
  geodesics bend around the hole, and the boundary has two loops.
- v_groove_block: a 40 x 30 x 10 mm block with a 90-degree V-groove 4 mm deep
  along X in its top: a sharp internal corner (concave crease) along the
  groove's bottom, which a ball cannot reach into.
"""

from pathlib import Path

STEP_DIR = Path(__file__).resolve().parents[3] / "fixtures" / "geodesic"


def rounded_block():
    import cadquery as cq

    return cq.Workplane("XY").box(40, 30, 10, centered=False).edges(">Z").fillet(4)


def bowl():
    import cadquery as cq

    block = cq.Workplane("XY").box(44, 44, 12, centered=False)
    # The sphere's axis horizontal: its poles (where a tessellator fans
    # degenerate slivers) lie outside the part.
    sphere = cq.Workplane("XY").add(
        cq.Solid.makeSphere(25, cq.Vector(22, 22, 12 + 25 - 9), cq.Vector(1, 0, 0), -90, 90, 360)
    )
    return block.cut(sphere)


def plate_with_hole():
    import cadquery as cq

    plate = cq.Workplane("XY").box(40, 30, 6, centered=False)
    return plate.cut(cq.Workplane("XY").center(22, 14).rect(10, 8).extrude(6))


def v_groove_block():
    import cadquery as cq

    block = cq.Workplane("XY").box(40, 30, 10, centered=False)
    groove = cq.Workplane("YZ").polyline([(11, 10), (15, 6), (19, 10)]).close().extrude(40)
    return block.cut(groove)


PARTS = {
    "rounded_block": rounded_block,
    "bowl": bowl,
    "plate_with_hole": plate_with_hole,
    "v_groove_block": v_groove_block,
}


def export_steps(directory=STEP_DIR):
    """Write each part to <directory>/<name>.step."""
    import cadquery as cq

    directory.mkdir(parents=True, exist_ok=True)
    for name, make in PARTS.items():
        cq.exporters.export(make(), str(directory / f"{name}.step"))


if __name__ == "__main__":
    export_steps()
