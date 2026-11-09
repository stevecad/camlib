"""The per-move gouge proof for prismatic material targets (#106).

A 20 x 10 pocket with a round island, cleared to Z 6 in a 40 x 30 x 10 block
and posted at 3 decimals. The target is prismatic (the region and its floor:
below the floor, and outside the region up to the stock top, is the part).
The cell comparison cannot decide the cells the pocket's walls cross; the
proof decides them, and a pocket cut past its boundary is never proven.
"""

import math
import tempfile
import unittest
from pathlib import Path

from camlib.export import ExportRefused, material_status
from camlib.jobs import Job, Setup
from camlib.material import Options, Target
from camlib.operations import HelicalEntry, Pocket
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.post import GCodeDialect, LoweringOptions, Machine, ToolSetting
from camlib.simulation import StockSpec
from camlib.tools import EndMill, HolderSection, ToolAssembly
from camlib.verification import MaterialCheck, StreamChecks

MILL = ToolAssembly(
    "em4", EndMill("em4", diameter=4, flute_length=12, overall_length=50), 25, 90, (HolderSection(25, 90, 10, 15),)
)


def region(grow=0.0):
    g = grow
    return PlanarRegion(
        "pocket",
        (
            Component(
                Boundary.polygon(((10 - g, 10 - g), (30 + g, 10 - g), (30 + g, 20 + g), (10 - g, 20 + g))),
                holes=(Boundary.circle((20, 15), 2 - g),),
            ),
        ),
    )


def run(tolerance, *, cut_past=0.0):
    target = Target("pocket", "1", region(), 6)
    pocket = Pocket(
        "pocket",
        MILL,
        region(cut_past),
        top_z_mm=10,
        floor_z_mm=6,
        stepdown_mm=2,
        stepover_mm=1.5,
        clearance_z_mm=15,
        entry=HelicalEntry(),
        feed_mm_min=800,
        plunge_mm_min=300,
        spindle_rpm=10000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    options = None if tolerance is None else Options(comparison_tolerance_mm=tolerance)
    setup = Setup(
        "top",
        StockSpec((0, 0, 0), (40, 30, 10), 0.25),
        (MILL,),
        (pocket,),
        (0, 0, 15),
        verification=StreamChecks(material=(MaterialCheck(target, options=options),)),
    )
    job = Job("pocket", "1", (setup,))
    header = job.compile().setup("top").program_header()
    machine = Machine(
        "test mill",
        (ToolSetting.for_tool(MILL, number=1, length_register=1, position_after_mm=header.initial_position_mm),),
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
        with tempfile.TemporaryDirectory() as directory:
            try:
                posted.export(Path(directory) / "pocket.nc", accept={"unchecked_machine"})
                material["exported"] = True
            except ExportRefused:
                material["exported"] = False
    return material


def status(material):
    report = dict(gouge=material["gouge"], complete=material["status"] == "complete")
    return material_status(report, material.get("sweep_proof"))


class PrismaticProofTest(unittest.TestCase):
    def test_pocket_walls_are_decided_by_the_per_move_proof(self):
        material = run(0.005)
        proof = material["sweep_proof"]
        self.assertEqual((proof["unproven_moves"], proof["history_complete"]), (0, True))
        self.assertGreater(proof["proven_moves"], 10)
        # The cell comparison alone leaves the wall cells possible.
        self.assertEqual(material["gouge"]["definite_cells"], 0)
        self.assertGreater(material["gouge"]["possible_only_cells"], 0)
        self.assertEqual(proof["decided_cells"], material["gouge"]["possible_only_cells"])
        self.assertEqual(status(material), "clear")
        self.assertTrue(material["exported"])

    def test_a_pocket_cut_past_its_boundary_is_never_proven(self):
        material = run(0.005, cut_past=0.05)
        self.assertGreater(material["sweep_proof"]["unproven_moves"], 0)
        self.assertNotEqual(status(material), "clear")
        self.assertFalse(material["exported"])

    def test_default_tolerance_leaves_tangent_walls_unproven(self):
        # Posted at 3 decimals a tangent pass can sit 0.0007 mm into a wall:
        # 0.001 mm (less the region's guard) cannot prove it.
        material = run(None)
        self.assertGreater(material["sweep_proof"]["unproven_moves"], 0)
        self.assertEqual(status(material), "uncertain")
        self.assertFalse(material["exported"])


if __name__ == "__main__":
    unittest.main()
