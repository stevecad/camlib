"""Material check of a machined prismatic plate (issue #72).

A 100 x 60 x 20 mm plate with a 50 x 30 x 8 mm pocket and four 6.6 mm holes:
face the top, clear the pocket, cut the outer profile, drill the holes. The
stock is compared with the CAD solid (SurfaceTarget with inner patches).

- No cell wholly inside a through hole is a possible gouge: the part has no
  material there (the target's upper bound over such a cell is its base).
- What the check cannot decide stays undecided, and real errors are found:
  a top faced 1 mm into the part is a violation, a profile cut 0.1 mm into the
  part's walls is never clear, and drilling through a blind hole is a gouge.
- Cells along walls the tools ran tangent to are decided by the per-move
  proof (every motion proven against the target mesh) when the comparison
  tolerance exceeds the mesh's surface error and the program's resolution;
  a real cut into the walls is never proven.
"""

import math
import unittest

from camlib.export import material_status
from camlib.material import Options, Verifier
from camlib.operations import Contour, Drilling, Facing, HelicalEntry, Pocket
from camlib.playback import Cursor, Player
from camlib.post import GCodeDialect, LoweringOptions, Machine, ToolSetting
from camlib.jobs import Job, Setup
from camlib.simulation import StockSpec
from camlib.tools import Drill, EndMill, HolderSection, ToolAssembly
from camlib.verification import MaterialCheck, StreamChecks

try:
    import cadquery as cq
except ImportError:  # pragma: no cover - CI installs CadQuery
    cq = None

HOLES = ((10.0, 10.0), (90.0, 10.0), (10.0, 50.0), (90.0, 50.0))
HOLE_R = 3.3
TOOLS = {
    "em10": ToolAssembly(
        "em10",
        EndMill("em10", diameter=10, flute_length=30, overall_length=75),
        40,
        110,
        (HolderSection(40, 110, 17.5, 23),),
    ),
    "em6": ToolAssembly(
        "em6",
        EndMill("em6", diameter=6, flute_length=18, overall_length=60),
        30,
        100,
        (HolderSection(30, 100, 17.5, 23),),
    ),
    "dr66": ToolAssembly(
        "dr66",
        Drill("dr66", diameter=6.6, flute_length=60, overall_length=100),
        70,
        140,
        (HolderSection(70, 140, 17.5, 23),),
    ),
}
NUMBERS = {"em10": 1, "em6": 2, "dr66": 3}


def plate(blind_holes=False):
    solid = cq.Workplane("XY").box(100, 60, 20, centered=False)
    solid = solid.cut(cq.Workplane("XY").box(50, 30, 8, centered=False).translate((25, 15, 12)))
    if blind_holes:
        # 10 mm deep from the top: the hole floor is at Z10.
        return solid.faces(">Z").workplane().pushPoints([(x - 50, y - 30) for x, y in HOLES]).hole(2 * HOLE_R, 10).val()
    return solid.faces("<Z").workplane().pushPoints([(x, -y) for x, y in HOLES]).hole(2 * HOLE_R).val()


def cut(tool):
    return dict(
        feed_mm_min=200 if tool == "dr66" else 900,
        spindle_rpm=1500 if tool == "dr66" else 9000,
        coolant="flood",
        tool_number=NUMBERS[tool],
        length_register=NUMBERS[tool],
    )


def run(cell, *, solid=None, face_z=20, outline=None, tolerance=None, export=False):
    from camlib.cadquery import export_floor_region, export_inner_patches, export_surface_mesh
    from camlib.material import SurfaceTarget

    part = plate() if solid is None else solid
    machined = plate()
    floor = next(
        f
        for f in machined.Faces()
        if f.geomType() == "PLANE" and f.normalAt().z > 0.999 and abs(f.Center().z - 12) < 1e-6
    )
    bottom = next(
        f for f in machined.Faces() if f.geomType() == "PLANE" and f.normalAt().z < -0.999 and abs(f.Center().z) < 1e-6
    )
    profile = export_floor_region(bottom, name="outline", source={"test": "plate"})
    if outline is not None:
        profile = export_floor_region(outline, name="outline", source={"test": "plate"})
    target = SurfaceTarget("plate", "1", export_surface_mesh(part), 0, inner_patches=export_inner_patches(part))
    em10, em6, drill = TOOLS["em10"], TOOLS["em6"], TOOLS["dr66"]
    operations = (
        Facing(
            "face",
            em10,
            floor_z_mm=face_z,
            stepdown_mm=1,
            stepover_mm=4,
            clearance_z_mm=30,
            plunge_mm_min=400,
            **cut("em10"),
        ),
        Pocket(
            "pocket",
            em6,
            export_floor_region(floor, name="pocket", source={"test": "plate"}),
            top_z_mm=20,
            floor_z_mm=12,
            stepdown_mm=2,
            stepover_mm=2.5,
            clearance_z_mm=30,
            entry=HelicalEntry(),
            plunge_mm_min=300,
            **cut("em6"),
        ),
        Contour(
            "outline",
            em10,
            profile,
            floor_z_mm=0,
            stepdown_mm=5,
            clearance_z_mm=30,
            lead_in_mm=3,
            lead_out_mm=3,
            plunge_mm_min=300,
            **cut("em10"),
        ),
        Drilling(
            "holes",
            drill,
            HOLES,
            depth_z_mm=-1,
            clearance_z_mm=30,
            allow_breakthrough_mm=1.5,
            peck_mm=4,
            retract_feed_mm_min=600,
            **cut("dr66"),
        ),
    )
    setup = Setup(
        "top",
        StockSpec((-5, -5, 0), (105, 65, 21), cell),
        tuple(TOOLS.values()),
        operations,
        (0, 0, 60),
        verification=StreamChecks(
            material=(
                MaterialCheck(
                    target, options=None if tolerance is None else Options(comparison_tolerance_mm=tolerance)
                ),
            )
        ),
    )
    job = Job("plate", "1", (setup,))
    header = job.compile().setup("top").program_header()
    machine = Machine(
        "test mill",
        tuple(
            ToolSetting.for_tool(
                t,
                number=NUMBERS[t.id],
                length_register=NUMBERS[t.id],
                position_after_mm=header.initial_position_mm,
            )
            for t in header.tools
        ),
        dialect=GCodeDialect.linuxcnc(),
        output_units="mm",
        decimals=3,
        max_feed_mm_min=10000,
        max_spindle_rpm=12000,
        rapid_mm_min=10000,
        planes=("xy", "xz", "yz"),
        arcs=True,
        helices=True,
        max_sweep_rad=math.tau,
        max_tool_number=99,
        max_length_register=99,
        tool_changes=True,
        flood=True,
        mist=False,
        tool_change_position_mode="preserve_tip",
    )
    with job.post("top", machine=machine, lowering=LoweringOptions(native_arcs=True)) as posted:
        posted.wait(600)
        status = posted.status()
        if status["error"]:
            raise AssertionError(status["error"])
        material = posted.verification_status(sample_limit=0)["material"]
        if export:
            import tempfile
            from pathlib import Path

            from camlib.export import ExportRefused

            with tempfile.TemporaryDirectory() as directory:
                try:
                    posted.export(Path(directory) / "plate.nc")
                    material["exported"] = True
                except ExportRefused:
                    material["exported"] = False
        player = Player(posted)
        player.seek(Cursor(status["available"]))
        report = Verifier(target, Options(max_findings=4096)).check(player.state)
    return material, report


def inside_a_hole(xy):
    x0, y0, x1, y1 = xy
    return any(all(math.hypot(x - cx, y - cy) < HOLE_R for x in (x0, x1) for y in (y0, y1)) for cx, cy in HOLES)


@unittest.skipIf(cq is None, "needs CadQuery")
class PrismaticMaterialTest(unittest.TestCase):
    def test_cells_inside_through_holes_are_not_possible_gouges(self):
        material, report = run(0.5)
        self.assertTrue(report["complete"])
        self.assertEqual(material["status"], "complete")
        self.assertFalse(report["stats"]["findings_truncated"])
        self.assertEqual(material["gouge"]["definite_cells"], 0)
        inside = [f for f in report["findings"] if inside_a_hole(f["xy_mm"])]
        self.assertFalse([f for f in inside if f["gouge"] != "within_tolerance"])
        # Wholly inside a through hole the target's upper bound is its base,
        # within the source error (it was the plate top, Z20).
        for f in report["findings"]:
            if inside_a_hole(f["xy_mm"]):
                self.assertLess(f["target_height_mm"][1], 0.01)
        # Cells along the vertical walls stay undecided: the stock's whole-cell
        # bounds cannot tell a cut tangent to a wall from one into it.
        self.assertEqual(material_status(report), "uncertain")

    def test_a_top_faced_into_the_part_is_a_violation(self):
        material, report = run(0.5, face_z=19)
        self.assertGreater(material["gouge"]["definite_cells"], 0)
        self.assertEqual(material_status(report), "violation")

    def test_a_profile_cut_into_the_walls_is_never_clear(self):
        # The outer profile follows an outline 0.1 mm inside the part's.
        inner = cq.Workplane("XY").box(99.8, 59.8, 1, centered=False).translate((0.1, 0.1, 0)).val()
        bottom = next(f for f in inner.Faces() if f.normalAt().z < -0.999)
        material, report = run(0.5, outline=bottom)
        self.assertNotEqual(material_status(report), "clear")
        self.assertTrue(material["gouge"]["definite_cells"] or material["gouge"]["possible_only_cells"])

    def test_tangent_walls_are_decided_by_the_per_move_proof(self):
        # The program is posted at 3 decimals (up to 0.0007 mm of XY
        # rounding) and the mesh carries 0.00088 mm of surface error: at
        # 0.005 mm every move is proven and the part is decided.
        material, report = run(0.5, tolerance=0.005, export=True)
        proof = material["sweep_proof"]
        self.assertEqual((proof["unproven_moves"], proof["history_complete"]), (0, True))
        self.assertGreater(proof["proven_moves"], 100)
        # The report stays the cell comparison; the proof decides every
        # possible-only gouge cell in it, so the program exports.
        self.assertEqual(material["gouge"]["definite_cells"], 0)
        self.assertGreater(material["gouge"]["possible_only_cells"], 1000)
        self.assertEqual(proof["decided_cells"], material["gouge"]["possible_only_cells"])
        self.assertEqual(material_status({**report, "gouge": material["gouge"]}, proof), "clear")
        self.assertTrue(material["exported"])

    def test_default_tolerance_cannot_prove_tangent_walls(self):
        # 0.001 mm is below the mesh error plus the posted rounding: the
        # wall passes stay unproven (and say so); other cells are decided.
        material, report = run(0.5, export=True)
        proof = material["sweep_proof"]
        self.assertGreater(proof["unproven_moves"], 0)
        self.assertGreater(proof["decided_cells"], 0)
        self.assertLess(proof["decided_cells"], material["gouge"]["possible_only_cells"])
        self.assertEqual(material_status({**report, "gouge": material["gouge"]}, proof), "uncertain")
        self.assertFalse(material["exported"])
        from camlib.export import describe

        self.assertIn("per-move proof", describe({"findings": [{**report, **material, "complete": True}]}))

    def test_a_profile_cut_into_the_walls_is_never_proven(self):
        inner = cq.Workplane("XY").box(99.8, 59.8, 1, centered=False).translate((0.1, 0.1, 0)).val()
        bottom = next(f for f in inner.Faces() if f.normalAt().z < -0.999)
        material, report = run(0.5, outline=bottom, tolerance=0.005, export=True)
        proof = material["sweep_proof"]
        self.assertGreater(proof["unproven_moves"], 0)
        self.assertNotEqual(material_status({**report, "gouge": material["gouge"]}, proof), "clear")
        self.assertFalse(material["exported"])

    def test_drilling_through_a_blind_hole_is_a_gouge(self):
        material, report = run(0.5, solid=plate(blind_holes=True))
        self.assertGreater(material["gouge"]["definite_cells"], 0)
        self.assertEqual(material_status(report), "violation")
        # Inside the hole the target keeps its floor at Z10.
        inside = [f for f in report["findings"] if inside_a_hole(f["xy_mm"])]
        self.assertTrue(inside)
        self.assertTrue(all(f["target_height_mm"][1] >= 10 for f in inside))


if __name__ == "__main__":
    unittest.main()
