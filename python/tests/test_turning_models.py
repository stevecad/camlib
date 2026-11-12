"""Tool and holder bodies from supplied models (#110), as meshes.

A grooving blade's carrier is given as a mesh in its maker's coordinates
with a declared placement; the blade itself stays the declared
``GrooveInsert``. The job grooves a shaft and parts it off:

- the carrier's real body (a block behind the blade) clears; the same block
  reaching forward meets the stock when parting off and is refused;
- the model's insert is cross-checked against the declared one: placed
  0.5 mm off its declared nose, turned 2 degrees, or drawn with another
  corner radius, it is refused before any move;
- an open mesh, a missing placement or tolerance: refused by name.

A boring bar as a mesh is checked in its bore: a coolant block that stays in
front of the face clears where a body filled out to its hull would not.
A milling holder's model gives the envelope it sweeps about the tool axis.
"""

import math
import unittest

import camlib
from camlib.tools import holder_sections_from_mesh
from test_turning import cnmg, job, tool
from test_turning_bodies import CONE, boring

# The maker's frame of these tests: the nose centre at (100, 50, 20), the
# lathe's +X along the model's -y, its +Z along the model's +x (so its +Y
# along the model's -z).
PLACEMENT = camlib.TurningModelPlacement(nose_center_mm=(100, 50, 20), x_axis=(0, -1, 0), z_axis=(1, 0, 0))


def to_model(p):
    """A lathe point (X, Y, Z) from the nose centre, in the maker's frame."""
    x, y, z = p
    return (100 + z, 50 - x, 20 - y)


def box(lo, hi):
    """A closed box from (X, Y, Z) ``lo`` to ``hi``: 12 triangles."""
    c = [(x, y, z) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
    quads = ((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3))
    return [tuple(c[i] for i in tri) for q in quads for tri in ((q[0], q[1], q[2]), (q[0], q[2], q[3]))]


def prism(polygon, y0, y1):
    """A closed prism over a convex (X, Z) polygon from Y ``y0`` to ``y1``."""
    n = len(polygon)

    def at(i, y):
        return (polygon[i % n][0], y, polygon[i % n][1])

    triangles = []
    for i in range(1, n - 1):
        triangles.append((at(0, y1), at(i, y1), at(i + 1, y1)))
        triangles.append((at(0, y0), at(i + 1, y0), at(i, y0)))
    for i in range(n):
        triangles.append((at(i, y0), at(i + 1, y0), at(i + 1, y1)))
        triangles.append((at(i, y0), at(i + 1, y1), at(i, y1)))
    return triangles


def mesh(triangles, error=0.0):
    return camlib.SurfaceMesh([tuple(to_model(p) for p in t) for t in triangles], error)


def blade(width, depth, radius, thickness, *, shift=(0, 0, 0), turn_deg=0.0, steps=24):
    """A grooving blade as its maker would draw it: ``width`` along Z,
    reaching ``depth`` along X, its two front corners rounded (chords of
    ``steps`` per corner), ``thickness`` below its cutting face; the leading
    corner's centre at the origin. Moved by ``shift`` and turned about Y."""
    w = width - 2 * radius

    def arc(center_z, start, end):
        return [
            (
                radius * math.cos(math.radians(start + (end - start) * k / steps)),
                center_z + radius * math.sin(math.radians(start + (end - start) * k / steps)),
            )
            for k in range(steps + 1)
        ]

    outline = [(depth, w + radius), (depth, -radius), *arc(0, 270, 180), *arc(w, 180, 90)]
    c, s = math.cos(math.radians(turn_deg)), math.sin(math.radians(turn_deg))
    return [
        tuple((x * c - z * s + shift[0], y + shift[1], x * s + z * c + shift[2]) for x, y, z in triangle)
        for triangle in prism(outline, -thickness, 0)
    ]


# The carrier of the 2 mm blade: camlib's own rectangle for it (22 mm behind
# the front, 60 long, the blade's width, 10 below the face to 2 above).
CARRIER = box((22, -10, -0.2), (82, 2, 1.8))
TOLERANCES = dict(
    corner_region_mm=1, position_tolerance_mm=0.02, nose_radius_tolerance_mm=0.02, edge_angle_tolerance_deg=0.25
)


def blade_check(**drawn):
    return camlib.TurningModelInsertCheck(insert=mesh(blade(2, 22, 0.2, 3, **drawn), 2e-4), **TOLERANCES)


def groover(body, insert_check, error=0.0):
    model = camlib.TurningToolModel(
        body=mesh(body, error), placement=PLACEMENT, axial_resolution_mm=0.5, insert_check=insert_check
    )
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


def grooving(groove_tool):
    profile = (
        (0, 0, 0),
        (10, 0, 0),
        (10, -20, 0),
        (15, -20, 0),
        (15, -25, 0),
        (12, -25, 0),
        (12, -28, 0),
        (15, -28, 0),
        (15, -40, 0),
    )
    return job(
        profile=profile,
        tools=(camlib.TurnToolSetting(cnmg(), 1, 1), camlib.TurnToolSetting(groove_tool, 2, 2)),
        operations=(
            camlib.TurnFace("face", "cnmg", 0, feed_mm_rev=0.15, rpm=1000),
            camlib.TurnRough("rough", "cnmg", feed_mm_rev=0.25, rpm=1000),
            camlib.TurnGroove("groove", "groove2", -25, -28, 12, peck_mm=1, feed_mm_rev=0.05, rpm=800),
            camlib.TurnPartOff("part", "groove2", -40, feed_mm_rev=0.05, rpm=800),
        ),
    )


class ModelHolderTest(unittest.TestCase):
    def test_a_mesh_carrier_equal_to_the_rectangle_gives_the_same_checks(self):
        built_in = tool("groove2", camlib.GrooveInsert(2, 22), nose_radius_mm=0.2, width_mm=2, length_mm=60)
        expected = grooving(built_in).plan()
        plan = grooving(groover(CARRIER, blade_check())).plan()
        for a, b in zip(expected.reports, plan.reports, strict=True):
            self.assertAlmostEqual(a["min_holder_clearance_mm"], b["min_holder_clearance_mm"], places=9)
            self.assertEqual(a["min_part_clearance_mm"], b["min_part_clearance_mm"])
            self.assertEqual(a["moves"], b["moves"])
        self.assertEqual(expected.stock, plan.stock)

    def test_the_real_body_clears_and_a_block_too_far_forward_is_refused(self):
        # A clamp block 18 mm wide behind the blade: from 40 mm it stays
        # outside the bar when the blade parts off at the axis.
        behind = CARRIER + box((40, -10, -8), (82, 6, 10))
        grooving(groover(behind, blade_check())).plan()
        # From 10 mm it is inside the Ø30 bar beside the part-off.
        forward = CARRIER + box((10, -10, -8), (82, 6, 10))
        with self.assertRaisesRegex(camlib.SimulationError, "the holder would hit"):
            grooving(groover(forward, blade_check())).plan()

    def test_the_mesh_error_is_grown_onto_the_body(self):
        exact = grooving(groover(CARRIER, blade_check())).plan()
        grown = grooving(groover(CARRIER, blade_check(), error=0.05)).plan()
        for a, b in zip(exact.reports[2:], grown.reports[2:], strict=True):
            self.assertLessEqual(b["min_holder_clearance_mm"], a["min_holder_clearance_mm"] - 0.05 + 1e-9)

    def test_a_model_off_its_declared_nose_is_refused_by_the_cross_check(self):
        groover(CARRIER, blade_check()).validate()
        # Within the declared tolerances.
        groover(CARRIER, blade_check(shift=(0.005, -0.005, 0.005), turn_deg=0.1)).validate()
        for shift in ((0.5, 0, 0), (0, 0.5, 0), (0, 0, 0.5), (-0.5, 0, 0)):
            with self.subTest(shift=shift), self.assertRaisesRegex(ValueError, "cutting corner is .* misplaced"):
                groover(CARRIER, blade_check(shift=shift)).validate()
        with self.assertRaisesRegex(ValueError, "degrees off the declared insert's edge"):
            groover(CARRIER, blade_check(turn_deg=2)).validate()
        # Another blade: 0.4 mm corners where 0.2 are declared.
        other = camlib.TurningModelInsertCheck(insert=mesh(blade(2, 22, 0.4, 3), 2e-4), **TOLERANCES)
        with self.assertRaisesRegex(ValueError, r"nose radius measures 0\.(39|40|41)"):
            groover(CARRIER, other).validate()
        # The job refuses it too, before its first move.
        with self.assertRaisesRegex(camlib.SimulationError, "cutting corner is"):
            grooving(groover(CARRIER, blade_check(shift=(0, 0, 0.5)))).plan()

    def test_an_open_mesh_and_missing_declarations_are_refused(self):
        with self.assertRaisesRegex(ValueError, "not a closed surface"):
            groover(CARRIER[:-2], blade_check()).validate()
        open_insert = camlib.TurningModelInsertCheck(insert=mesh(blade(2, 22, 0.2, 3)[:-2], 2e-4), **TOLERANCES)
        with self.assertRaisesRegex(ValueError, "insert mesh is not a closed surface"):
            groover(CARRIER, open_insert).validate()
        with self.assertRaisesRegex(ValueError, "state the reason"):
            groover(CARRIER, camlib.TurningModelWithoutInsert(reason="")).validate()
        groover(CARRIER, camlib.TurningModelWithoutInsert(reason="maker's cutting-point frame")).validate()
        # No placement, resolution or tolerance has a default.
        with self.assertRaises(TypeError):
            camlib.TurningToolModel(body=mesh(CARRIER), placement=PLACEMENT, axial_resolution_mm=0.5)
        with self.assertRaises(TypeError):
            camlib.TurningToolModel(
                body=mesh(CARRIER), placement=None, axial_resolution_mm=0.5, insert_check=blade_check()
            )
        with self.assertRaises(TypeError):
            camlib.TurningModelInsertCheck(insert=mesh(blade(2, 22, 0.2, 3)), corner_region_mm=1)
        with self.assertRaises(TypeError):
            camlib.TurningModelPlacement(nose_center_mm=(0, 0, 0))
        skewed = camlib.TurningModelPlacement(nose_center_mm=(0, 0, 0), x_axis=(1, 0.1, 0), z_axis=(0, 0, 1))
        model = camlib.TurningToolModel(
            body=mesh(CARRIER),
            placement=skewed,
            axial_resolution_mm=0.5,
            insert_check=camlib.TurningModelWithoutInsert(reason="test"),
        )
        with self.assertRaisesRegex(ValueError, "placement"):
            camlib.TurningTool(
                "t",
                insert=camlib.GrooveInsert(2, 22),
                nose_radius_mm=0.2,
                side="outer",
                hand="right",
                holder=camlib.TurningHolder.from_model(model),
                seat=camlib.TurningSeat(0, 0),
                insert_thickness_mm=3,
                insert_clearance_deg=7,
            ).validate()


def bored(body):
    """The boring tests' job, its bar's holder given as ``body`` (triangles
    in lathe coordinates from the nose centre)."""
    model = camlib.TurningToolModel(
        body=mesh(body),
        placement=PLACEMENT,
        axial_resolution_mm=0.5,
        insert_check=camlib.TurningModelWithoutInsert(reason="synthetic test bar drawn about its nose centre"),
    )
    insert = camlib.TurningInsert.iso(
        included_angle_deg=55, approach_angle_deg=95, edge_length_mm=6, nose_radius_mm=0.4
    )
    made = camlib.TurningTool(
        "bar",
        insert=insert,
        nose_radius_mm=0.4,
        side="inner",
        hand="right",
        holder=camlib.TurningHolder.from_model(model),
        seat=camlib.TurningSeat(0, 0),
        insert_thickness_mm=1.5,
        insert_clearance_deg=11,
    )
    drill = tool("drill16", camlib.DrillInsert(16, 118, 50), nose_radius_mm=0, width_mm=40, length_mm=60)
    return job(
        stock=camlib.TurnStockSpec(-60, 0, 20),
        profile=((0, 0, 0), (18, 0, 0), (18, -40, 0)),
        bore=((11, 0, 0), (11, -25, 0), (8, -25, 0), (0, -25 - CONE, 0)),
        tools=(camlib.TurnToolSetting(drill, 1, 1), camlib.TurnToolSetting(made, 2, 2)),
        operations=(
            camlib.TurnDrill("drill", "drill16", -25 - CONE, peck_mm=10, feed_mm_rev=0.12, rpm=900),
            camlib.TurnBoreRough("bore_rough", "bar", -24, feed_mm_rev=0.15, rpm=1000),
            camlib.TurnBoreFinish("bore_finish", "bar", -24, feed_mm_rev=0.08, rpm=1200),
        ),
    )


class ModelBarTest(unittest.TestCase):
    # The bar of the boring tests: 10 mm across, its top 1 mm under the nose
    # centre (toward the axis), 1.5 mm below the face to 1 above, 80 long.
    BAR = box((-11, -1.5, 0), (-1, 1, 80))

    def test_a_mesh_bar_is_checked_in_its_bore_like_the_rectangle(self):
        expected = boring().plan()
        plan = bored(self.BAR).plan()
        for a, b in zip(expected.reports, plan.reports, strict=True):
            self.assertAlmostEqual(a["min_holder_clearance_mm"], b["min_holder_clearance_mm"], places=9)
        # 18 mm across it does not fit the Ø22 bore.
        with self.assertRaisesRegex(camlib.SimulationError, "the holder would hit"):
            bored(box((-19, -1.5, 0), (-1, 1, 80))).plan()
        # Nor 9 mm high off the centre plane: √(x² + y²) reaches the wall.
        with self.assertRaisesRegex(camlib.SimulationError, "the holder would hit"):
            bored(box((-11, -9, 0), (-1, 1, 80))).plan()

    def test_a_coolant_block_in_front_of_the_face_clears_where_its_hull_would_not(self):
        # The bore is 24 mm deep: a block from 30 mm behind the nose stays
        # in front of the face.
        block = box((-17, -9.5, 30), (8, 9, 45))
        bored(self.BAR + block).plan()
        # The body filled out from the bar's tip to the block (a part of
        # its hull), as one convex solid.
        lo, hi = ((-11, -1.5), (-1, 1)), ((-17, -9.5), (8, 9))
        ends = [
            [(x, y, z) for x, y in ((r[0][0], r[0][1]), (r[1][0], r[0][1]), (r[1][0], r[1][1]), (r[0][0], r[1][1]))]
            for r, z in ((lo, 0), (hi, 30))
        ]
        c = ends[0] + ends[1]
        quads = ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7))
        filled = [tuple(c[i] for i in tri) for q in quads for tri in ((q[0], q[1], q[2]), (q[0], q[2], q[3]))]
        with self.assertRaisesRegex(camlib.SimulationError, "the holder would hit"):
            bored(filled + block).plan()


class MillingHolderEnvelopeTest(unittest.TestCase):
    def test_the_envelope_is_what_the_holder_sweeps_about_the_axis(self):
        # A holder drawn along the model's +x from the tip at (5, 0, 0): a
        # 20 mm square shank from 30 to 80 mm, a 10 x 12 block out to 26 mm
        # from the axis between 40 and 52.
        triangles = box((35, -10, -10), (85, 10, 10)) + box((45, 0, -6), (57, 26, 6))
        holder = camlib.SurfaceMesh(triangles, 0.01)
        sections = holder_sections_from_mesh(
            holder, tip_mm=(5, 0, 0), axis=(1, 0, 0), stickout_mm=29, gauge_length_mm=75, axial_resolution_mm=2
        )
        self.assertEqual(sections[0].z_min_mm, 29)
        self.assertEqual(sections[-1].z_max_mm, 75)
        for a, b in zip(sections, sections[1:]):
            self.assertEqual(a.z_max_mm, b.z_min_mm)
        radius = {s.z_min_mm: s.radius_max_mm for s in sections}
        self.assertAlmostEqual(sections[0].radius_max_mm, math.hypot(10, 10) + 0.01, places=9)
        over = max(s.radius_max_mm for s in sections)
        self.assertAlmostEqual(over, math.hypot(26, 6) + 0.01, places=9)
        self.assertTrue(all(r >= math.hypot(10, 10) for r in radius.values()))
        # It is a valid assembly's holder.
        self.assertTrue(all(s.radius_min_mm == s.radius_max_mm for s in sections))
        # A model that starts below the declared stickout, one that does
        # not reach the gauge plane, an open one, an unplaced one.
        for change, message in (
            (dict(stickout_mm=32), "below the range's start"),
            (dict(gauge_length_mm=90), "no surface"),
            (dict(axis=(2, 0, 0)), "unit vector"),
        ):
            values = dict(tip_mm=(5, 0, 0), axis=(1, 0, 0), stickout_mm=29, gauge_length_mm=75, axial_resolution_mm=2)
            with self.subTest(change=change), self.assertRaisesRegex(ValueError, message):
                holder_sections_from_mesh(holder, **{**values, **change})
        with self.assertRaisesRegex(ValueError, "not a closed surface"):
            holder_sections_from_mesh(
                camlib.SurfaceMesh(triangles[:-1], 0.01),
                tip_mm=(5, 0, 0),
                axis=(1, 0, 0),
                stickout_mm=29,
                gauge_length_mm=75,
                axial_resolution_mm=2,
            )


if __name__ == "__main__":
    unittest.main()
