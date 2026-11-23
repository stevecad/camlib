"""Issue #381: an adaptive pocket from a floor a Facing just cut.

A 100 x 60 plate in stock 1 mm taller is faced to exactly Z20 with a 6 mm end
mill; an AdaptivePocket then starts at that faced floor and goes down to Z12,
a whole number of stepdowns. The faced stock model stands its sweep guard
above Z20. The levels used to be planned from that guard, which left a last
level a guard deep at the floor, refused by its engagement proof. The run is
streamed end to end (every frame checked by the receiver, every move checked
against the stock for contact) and must finish with whole stepdowns."""

import unittest

import camlib
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.simulation import OperationUpdate, Simulation, StockSpec, VerificationUpdate
from camlib.tools import EndMill, HolderSection, ToolAssembly
from camlib.verification import StockContactOptions, StreamChecks

CUTTING = dict(feed_mm_min=600, plunge_mm_min=150, spindle_rpm=12000, coolant="flood", tool_number=1, length_register=1)


def run(stepdown):
    tool = ToolAssembly(
        "em6", EndMill("em6", diameter=6, flute_length=30, overall_length=50), 35, 60, (HolderSection(35, 60, 5, 5),)
    )
    face = camlib.Facing("face", tool, floor_z_mm=20, stepdown_mm=1, stepover_mm=4, clearance_z_mm=25, **CUTTING)
    region = PlanarRegion("pocket", (Component(Boundary.polygon(((20, 15), (80, 15), (80, 45), (20, 45)))),))
    pocket = camlib.AdaptivePocket("pocket", tool, region, 20, 12, stepdown, 1.5, 25, **CUTTING)
    return Simulation(
        StockSpec((0, 0, 0), (100, 60, 21), 0.5),
        (tool,),
        operations=(face, pocket),
        revision="1",
        initial_position_mm=(0, 0, 25),
        verification=StreamChecks(stock_contact=StockContactOptions()),
    )


class FacedPocketTopTest(unittest.TestCase):
    def test_adaptive_pocket_from_a_faced_floor_is_released_in_whole_stepdowns(self):
        for stepdown, tips in ((2, [18, 16, 14, 12]), (8, [12])):
            with self.subTest(stepdown=stepdown), run(stepdown) as sim:
                updates = list(sim.run_operations())
                self.assertTrue(sim.state.finished)
                ends = [
                    u
                    for u in updates
                    if isinstance(u, OperationUpdate) and u.stage == "end" and u.operation == "pocket"
                ]
                report = ends[0].report
                self.assertEqual([layer["tip_z_mm"] for layer in report["layers"]], tips)
                self.assertEqual(report["completed_layers"], len(tips))
                self.assertEqual(report["stock_top"]["authored_top_z_mm"], 20)
                contact = [u for u in updates if isinstance(u, VerificationUpdate) and u.check["kind"] == "motion"]
                self.assertTrue(contact)
                self.assertEqual({u.status for u in contact}, {"clear"})


if __name__ == "__main__":
    unittest.main()
