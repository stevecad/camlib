"""Tool bodies from STEP models through the CadQuery adapter (#110).

The STEP files are written by the tests from CadQuery primitives (a blade
carrier, a clamp block, a screw boss, the blade), in a maker's frame that is
not the lathe's. Imported with a declared placement they give the same
checks as the equivalent hand-made mesh; the model's blade is cross-checked
against the declared one.
"""

import math
import tempfile
import unittest
from pathlib import Path

import cadquery as cq
from OCP.gp import gp_Trsf

import camlib
from camlib.cadquery import step_mesh, step_solids, tool_body_from_shape, tool_body_from_step
from camlib.tools import holder_sections_from_mesh
from test_turning_models import CARRIER, PLACEMENT, TOLERANCES, box, grooving, groover

# Lathe (X, Y, Z) from the nose centre to the maker's frame of
# ``test_turning_models.PLACEMENT``: (100 + Z, 50 - X, 20 - Y).
MAKER = gp_Trsf()
MAKER.SetValues(0, 0, 1, 100, -1, 0, 0, 50, 0, -1, 0, 20)


def solid_box(lo, hi):
    size = [hi[k] - lo[k] for k in range(3)]
    return cq.Workplane("XY").box(*size, centered=False).translate(tuple(lo)).val()


def blade_solid(width, depth, radius, thickness):
    """The grooving blade as a solid: its outline on the cutting face (the
    XZ plane, the leading corner's centre at the origin), true arcs at the
    corners, extruded ``thickness`` toward -Y."""
    w, d = width - 2 * radius, radius / math.sqrt(2)
    return (
        cq.Workplane("XZ")
        .moveTo(depth, w + radius)
        .lineTo(depth, -radius)
        .lineTo(0, -radius)
        .threePointArc((-d, -d), (-radius, 0))
        .lineTo(-radius, w)
        .threePointArc((-d, w + d), (0, w + radius))
        .close()
        .extrude(thickness)
        .val()
    )


def in_maker_frame(*solids):
    return [solid.moved(cq.Location(MAKER)) for solid in solids]


CARRIER_SOLID = solid_box((22, -10, -0.2), (82, 2, 1.8))
BLOCK = ((40, -10, -8), (82, 6, 10))


def write_step(directory, name, solids):
    path = Path(directory) / name
    cq.Compound.makeCompound(solids).exportStep(str(path))
    return path


def step_groover(path, **changes):
    solids = step_solids(path)
    blade = min(range(len(solids)), key=lambda i: solids[i].Volume())
    values = dict(
        max_error_mm=0.002, placement=PLACEMENT, axial_resolution_mm=0.5, insert_solids=(blade,), **TOLERANCES
    )
    values.update(changes)
    model = tool_body_from_step(path, **values)
    return camlib.TurningTool(
        "groove2",
        insert=camlib.GrooveInsert(2, 22),
        nose_radius_mm=0.2,
        side="outer",
        hand="right",
        holder=camlib.TurningHolder.from_model(model),
        seat=camlib.TurningSeat(0, 0),
        insert_thickness_mm=3,
        insert_clearance_deg=7,
    )


class StepToolBodyTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)

    def test_a_step_holder_gives_the_same_checks_as_the_equivalent_mesh(self):
        path = write_step(
            self.directory.name,
            "blade holder.step",
            in_maker_frame(CARRIER_SOLID, solid_box(*BLOCK), blade_solid(2, 22, 0.2, 3)),
        )
        self.assertEqual(len(step_solids(path)), 3)
        from_step = step_groover(path)
        model = from_step.holder.model
        self.assertIsInstance(model.insert_check, camlib.TurningModelInsertCheck)
        self.assertLessEqual(model.body.surface_error_mm, 0.002)
        self.assertLessEqual(model.insert_check.insert.surface_error_mm, 0.002)
        from_step.validate()
        plan = grooving(from_step).plan()
        # The same body as a hand-made mesh, with the hand-made blade.
        by_hand = grooving(
            groover(CARRIER + box(*BLOCK), from_step.holder.model.insert_check, error=model.body.surface_error_mm)
        ).plan()
        for a, b in zip(by_hand.reports, plan.reports, strict=True):
            self.assertAlmostEqual(a["min_holder_clearance_mm"], b["min_holder_clearance_mm"], places=6)
            self.assertEqual(a["min_part_clearance_mm"], b["min_part_clearance_mm"])
            self.assertEqual(a["moves"], b["moves"])
        self.assertEqual(by_hand.stock, plan.stock)

    def test_a_step_model_off_its_declared_nose_is_refused(self):
        solids = (CARRIER_SOLID, solid_box(*BLOCK), blade_solid(2, 22, 0.2, 3))
        path = write_step(self.directory.name, "holder.step", in_maker_frame(*solids))
        off = camlib.TurningModelPlacement(nose_center_mm=(100.5, 50, 20), x_axis=(0, -1, 0), z_axis=(1, 0, 0))
        with self.assertRaisesRegex(ValueError, r"cutting corner is 0\.(49|50)\d\d mm .* misplaced"):
            step_groover(path, placement=off).validate()
        with self.assertRaisesRegex(camlib.SimulationError, "cutting corner is"):
            grooving(step_groover(path, placement=off)).plan()
        # Another blade in the file (0.4 mm corners).
        other = write_step(
            self.directory.name,
            "other.step",
            in_maker_frame(CARRIER_SOLID, solid_box(*BLOCK), blade_solid(2, 22, 0.4, 3)),
        )
        with self.assertRaisesRegex(ValueError, r"nose radius measures 0\.(39|40|41)"):
            step_groover(other).validate()

    def test_a_curved_holder_and_a_block_too_far_forward(self):
        # A screw boss (a cylinder along Y) on the block: tessellated within
        # the declared error, which is grown onto the body.
        boss = cq.Workplane("XZ").center(60, 1).circle(4).extrude(-12).translate((0, 6, 0)).val()
        path = write_step(
            self.directory.name,
            "boss.step",
            in_maker_frame(CARRIER_SOLID, solid_box(*BLOCK), boss, blade_solid(2, 22, 0.2, 3)),
        )
        made = step_groover(path)
        self.assertGreater(made.holder.model.body.surface_error_mm, 1e-6)
        (x0, x1), (y0, y1), (z0, z1) = made.holder.model.bounds_mm()
        self.assertAlmostEqual(y1, 18, delta=0.01)
        self.assertAlmostEqual(x0, 22, delta=0.01)
        grooving(made).plan()
        forward = write_step(
            self.directory.name,
            "forward.step",
            in_maker_frame(CARRIER_SOLID, solid_box((10, -10, -8), (82, 6, 10)), blade_solid(2, 22, 0.2, 3)),
        )
        with self.assertRaisesRegex(camlib.SimulationError, "the holder would hit"):
            grooving(step_groover(forward)).plan()

    def test_every_value_is_declared(self):
        solids = in_maker_frame(CARRIER_SOLID, blade_solid(2, 22, 0.2, 3))
        common = dict(max_error_mm=0.002, placement=PLACEMENT, axial_resolution_mm=0.5)
        with self.assertRaisesRegex(TypeError, "position_tolerance_mm"):
            tool_body_from_shape(solids, insert_solids=(1,), corner_region_mm=1, **common)
        with self.assertRaisesRegex(TypeError, "without_insert_reason"):
            tool_body_from_shape(solids[:1], insert_solids=(), **common)
        with self.assertRaises(TypeError):
            tool_body_from_shape(solids, insert_solids=(1,), **TOLERANCES, max_error_mm=0.002, placement=PLACEMENT)
        with self.assertRaisesRegex(TypeError, "placement"):
            tool_body_from_shape(
                solids, insert_solids=(1,), **TOLERANCES, max_error_mm=0.002, placement=None, axial_resolution_mm=1
            )
        with self.assertRaisesRegex(ValueError, "insert_solids"):
            tool_body_from_shape(solids, insert_solids=(2,), **TOLERANCES, **common)
        with self.assertRaisesRegex(ValueError, "every solid is the insert"):
            tool_body_from_shape(solids, insert_solids=(0, 1), **TOLERANCES, **common)
        model = tool_body_from_shape(
            solids[:1], insert_solids=(), without_insert_reason="maker's cutting-point frame", **common
        )
        self.assertEqual(model.insert_check, camlib.TurningModelWithoutInsert(reason="maker's cutting-point frame"))
        # A surface model has no body.
        face = cq.Workplane("XY").rect(4, 4).val()
        with self.assertRaisesRegex(ValueError, "needs valid solids"):
            tool_body_from_shape(cq.Face.makeFromWires(face), insert_solids=(), without_insert_reason="x", **common)


class StepMillingHolderTest(unittest.TestCase):
    def test_a_step_holder_gives_its_swept_envelope(self):
        # A Ø40 holder body from 30 to 80 mm above the tip along +z, with a
        # drive block out to 26 mm on one side.
        body = cq.Workplane("XY").circle(20).extrude(50).translate((0, 0, 30)).val()
        block = solid_box((0, -6, 40), (26, 6, 52))
        with tempfile.TemporaryDirectory() as directory:
            path = write_step(directory, "holder.step", [body, block])
            holder = step_mesh(path, max_error_mm=0.005)
        self.assertLessEqual(holder.surface_error_mm, 0.005)
        sections = holder_sections_from_mesh(
            holder, tip_mm=(0, 0, 0), axis=(0, 0, 1), stickout_mm=29.9, gauge_length_mm=75, axial_resolution_mm=2
        )
        self.assertEqual((sections[0].z_min_mm, sections[-1].z_max_mm), (29.9, 75))
        error = holder.surface_error_mm
        for s in sections:
            lowest, highest = 20, 20 + 2 * error
            if s.z_max_mm > 40 - error and s.z_min_mm < 52 + error:
                lowest, highest = math.hypot(26, 6), math.hypot(26, 6) + 2 * error
            self.assertGreaterEqual(s.radius_max_mm, lowest - 1e-9)
            self.assertLessEqual(s.radius_max_mm, highest + 1e-9)


if __name__ == "__main__":
    unittest.main()
