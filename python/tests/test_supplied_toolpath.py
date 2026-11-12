"""A caller-supplied toolpath (#157) is one more operation of a job.

The strongest statement is made with camlib's own moves: a planned pocket's
streamed moves, handed back as a ``SuppliedToolpath`` in its place, give the
same stream, the same stock for the operation after it, the same findings
under every configured check and the same posted program. Then moves no
planner made: findings that gate export, and refusals that name the move.
"""

from dataclasses import replace
import importlib.util
import math
import os
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import camlib
from camlib import SuppliedToolpath, ToolpathMove
from camlib.export import ExportRefused
from camlib.jobs import RegenerationCache
from camlib.material import Target
from camlib.motion import Arc, Helix, Line, Motion
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.simulation import OperationUpdate, SimulationError, Update
from camlib.verification import MachineLimits, MaterialCheck, Prism, Scene, StockContactOptions, StreamChecks

ROOT = Path(__file__).resolve().parents[2]
DECLARED = dict(spindle_rpm=12000, coolant="flood", tool_number=7, length_register=9)
ACCEPT = {"unchecked_machine"}


def example():
    spec = importlib.util.spec_from_file_location("standalone_supplied", ROOT / "examples/standalone_job.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def collect(job, cache=None):
    """The run's motions and operation updates, per operation, and its final stock volume."""
    motions, controls = {}, {}
    with job.regenerate(cache, executable=os.environ.get("CAMLIB_TEST_ENGINE")) as run:
        for event in run:
            update = event.update
            if isinstance(update, Update):
                motions.setdefault(update.motion["operation"], []).append(update.motion)
            elif isinstance(update, OperationUpdate):
                controls.setdefault(update.operation, {})[update.stage] = update
        volume = run.states["top"].volume()
    return motions, controls, volume


def with_operation(job, index, operation, **setup_changes):
    setup = job.setups[0]
    operations = list(setup.operations)
    operations[index] = operation
    return replace(job, setups=(replace(setup, operations=tuple(operations), **setup_changes),))


def with_checks(job, checks):
    return replace(job, setups=(replace(job.setups[0], verification=checks),))


def comparable(value):
    """Findings without what names the run they came from (its id, its
    manifest digest and the stock digest bound to them)."""
    if isinstance(value, dict):
        return {k: comparable(v) for k, v in value.items() if k not in ("run", "stock_sha256")}
    if isinstance(value, (list, tuple)):
        return [comparable(v) for v in value]
    return value


def slot(y=6.123456789012345):
    """A slot at Z 4 in the faced block (top Z 5), with a half circle, a
    helix down to Z 3.5 and a fed retract."""
    return (
        ToolpathMove("rapid", Line((6, y, 8), (6, y, 5.5))),
        ToolpathMove("plunge", Line((6, y, 5.5), (6, y, 4)), feed_mm_min=150),
        ToolpathMove("cut", Line((6, y, 4), (12, y, 4)), feed_mm_min=612.5),
        ToolpathMove("cut", Arc((12, y, 4), (12, y + 4, 4), (12, y + 2, 4), math.pi), feed_mm_min=500),
        ToolpathMove("cut", Line((12, y + 4, 4), (6, y + 4, 4)), feed_mm_min=612.5),
        ToolpathMove("lead", Helix((6, y + 4, 4), (6, y, 3.5), (6, y + 2, 4), math.pi), feed_mm_min=200),
        ToolpathMove("retract", Line((6, y, 3.5), (6, y, 8)), feed_mm_min=300),
    )


class SuppliedToolpathTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = example()
        # Face (mill4), pocket (mill4), rest pocket (mill2); stock contact checked.
        cls.planned = cls.module.build_rest_job()
        cls.machine = cls.module.rest_machine_for(cls.planned)
        cls.planned_motions, cls.planned_controls, cls.planned_volume = collect(cls.planned)
        tool = cls.planned.setups[0].tools[0]
        cls.tool = tool
        # The planned pocket's own moves, as the stream gave them, supplied
        # in its place (same id, so the two programs can be compared whole).
        cls.resupplied_operation = SuppliedToolpath(
            "pocket",
            tool,
            [ToolpathMove.from_motion(m) for m in cls.planned_motions["pocket"]],
            clearance_z_mm=8,
            **DECLARED,
        )
        cls.resupplied = with_operation(cls.planned, 1, cls.resupplied_operation)

    def test_planned_moves_supplied_again_are_the_same_run_and_the_same_program(self):
        motions, controls, volume = collect(self.resupplied)
        report = controls["pocket"]["end"].report
        self.assertEqual(report["strategy"], "supplied_toolpath")
        # The pocket's first move starts where facing ended: no travel is added.
        self.assertEqual(report["approach_moves"], 0)
        self.assertEqual(report["supplied_moves"], len(self.planned_motions["pocket"]))
        # Out of the stream exactly as supplied: every field of every move
        # (ids included, since nothing was added), every float the same value.
        self.assertEqual(motions["pocket"], self.planned_motions["pocket"])
        self.assertEqual(motions["pocket"][0]["id"], report["first_supplied_motion_id"])
        for streamed, move in zip(motions["pocket"], self.resupplied_operation.motions):
            for name, value in move.curve.to_dict().items():
                got = streamed["curve"][name]
                self.assertEqual(got if isinstance(value, (str, float)) else tuple(got), value)
            self.assertEqual((streamed["kind"], streamed["feed_mm_min"]), (move.kind, move.feed_mm_min))
        # The same setup and shutdown events as the planned operation's.
        for stage in ("begin", "end"):
            self.assertEqual(controls["pocket"][stage].events, self.planned_controls["pocket"][stage].events)
        # The rest pocket plans from the stock the supplied moves left: the
        # same stock, so the same moves, and the same stock at the end.
        self.assertEqual(motions["rest"], self.planned_motions["rest"])
        self.assertEqual(motions["face"], self.planned_motions["face"])
        self.assertEqual(volume, self.planned_volume)
        # Posted, decoded back and compared: the same program, byte for byte.
        planned = self.planned.post_nc("top", machine=self.machine, accept=ACCEPT)
        supplied = self.resupplied.post_nc("top", machine=self.machine, accept=ACCEPT)
        self.assertEqual(supplied.text, planned.text)
        self.assertEqual((supplied.motions, supplied.findings), (planned.motions, ()))
        self.assertEqual(supplied.unchecked, planned.unchecked)

    def test_a_supplied_move_gets_the_finding_a_planned_move_gets_and_export_is_refused(self):
        pocket = self.planned.setups[0].operations[1]
        footprint = PlanarRegion("clamp", (Component(Boundary.polygon(((8, 6), (10, 6), (10, 8), (8, 8)))),))
        contact = StockContactOptions()
        cases = dict(
            # A fixture standing in the pocket.
            fixture=StreamChecks(
                scene=Scene("s", "1", bodies=(Prism("clamp", footprint, 0, 10, role="fixture"),)),
                stock_contact=contact,
            ),
            # Travel that ends below the clearance height, and a feed limit
            # below the cutting feed.
            machine=StreamChecks(
                scene=Scene("s", "1", machine=MachineLimits((-50, -50, 0), (50, 50, 7.5), 500)),
                stock_contact=contact,
            ),
            # A part that keeps 1 mm more than the pocket leaves: a gouge.
            material=StreamChecks(
                material=(MaterialCheck(Target("part", "1", pocket.region, 4)),), stock_contact=contact
            ),
        )
        for name, checks in cases.items():
            with self.subTest(name):
                refusals = []
                for job in (self.planned, self.resupplied):
                    job = with_checks(job, checks)
                    with TemporaryDirectory() as directory:
                        target = Path(directory) / "part.nc"
                        with self.assertRaises(ExportRefused) as refused:
                            job.post_nc("top", machine=self.machine, path=target, accept=ACCEPT)
                        self.assertFalse(target.exists())
                    refusals.append(refused.exception)
                planned, supplied = refusals
                self.assertIn("violation", supplied.statuses)
                self.assertEqual(supplied.statuses, planned.statuses)
                # The same results on the same moves, finding for finding
                # (apart from each run's own identity).
                self.assertEqual(comparable(supplied.findings), comparable(planned.findings))
                self.assertEqual(str(supplied), str(planned))
                self.assertTrue(supplied.findings)
                if name != "material":
                    # Among them, findings on the supplied moves themselves.
                    ids = [m["id"] for m in self.planned_motions["pocket"]]
                    on_supplied = [r for r in supplied.findings if ids[0] <= r["completed"] <= ids[-1]]
                    self.assertTrue(on_supplied)
                    kinds = {f["kind"] for r in on_supplied for f in r["findings"]}
                    self.assertEqual(
                        kinds, {"fixture_contact"} if name == "fixture" else {"feed_limit", "machine_travel"}
                    )
                # Accepted on purpose, it is released, as the planned job is.
                job = with_checks(self.resupplied, checks)
                accept = ACCEPT | set(supplied.statuses)
                released = job.post_nc("top", machine=self.machine, accept=accept)
                self.assertEqual(released.accepted, tuple(sorted(accept)))
                self.assertTrue(released.text.endswith("M2\n"))

    def test_moves_no_planner_made_run_between_planned_operations_and_post(self):
        operation = SuppliedToolpath("outside", self.tool, slot(), clearance_z_mm=8, **{**DECLARED, "coolant": "mist"})
        job = with_operation(self.planned, 1, operation)
        motions, controls, _ = collect(job)
        report = controls["outside"]["end"].report
        # Facing ends at Z 8 elsewhere: one rapid across at that height.
        self.assertEqual((report["approach_moves"], report["supplied_moves"]), (1, 7))
        travel, *streamed = motions["outside"]
        self.assertEqual((travel["kind"], travel["feed_mm_min"]), ("rapid", None))
        self.assertEqual(travel["curve"]["start"], motions["face"][-1]["curve"]["end"])
        self.assertEqual(tuple(travel["curve"]["end"]), slot()[0].curve.start)
        self.assertEqual([m["id"] for m in streamed], [report["first_supplied_motion_id"] + i for i in range(7)])
        for got, move in zip(streamed, slot()):
            self.assertEqual((got["kind"], got["feed_mm_min"]), (move.kind, move.feed_mm_min))
            self.assertEqual(got["curve"]["kind"], move.curve.to_dict()["kind"])
            for name in ("start", "end"):
                self.assertEqual(tuple(got["curve"][name]), getattr(move.curve, name))
        # The rest pocket plans from the slot's stock, not from the planned
        # pocket's: it has more left to cut.
        self.assertNotEqual(motions["rest"], self.planned_motions["rest"])
        released = job.post_nc("top", machine=self.machine, accept=ACCEPT)
        self.assertEqual(released.findings, ())
        lines = released.text.splitlines()
        expected = [
            "M3 S12000",
            "M7",
            "G0 X6 Y6.123 Z8",
            "G0 X6 Y6.123 Z5.5",
            "G1 X6 Y6.123 Z4 F150",
            "G1 X12 Y6.123 Z4 F612.5",
            # The default post selects the plane before each arc; an arc in
            # XY carries no Z word.
            "G17",
            "G3 X12 Y10.123 I0 J2 F500",
            "G1 X6 Y10.123 Z4 F612.5",
            "G17",
            "G3 X6 Y6.123 Z3.5 I0 J-2 F200",
            "G1 X6 Y6.123 Z8 F300",
        ]
        at = lines.index("M7") - 1
        self.assertEqual(lines[at : at + len(expected)], expected)

    def test_a_supplied_rapid_through_stock_is_a_violation_that_gates_export(self):
        y = 6.5
        moves = (
            # Declared a rapid, it ends 1 mm inside the faced stock.
            ToolpathMove("rapid", Line((6, y, 8), (6, y, 4))),
            ToolpathMove("cut", Line((6, y, 4), (12, y, 4)), feed_mm_min=600),
            ToolpathMove("retract", Line((12, y, 4), (12, y, 8)), feed_mm_min=300),
        )
        job = with_operation(
            self.planned, 1, SuppliedToolpath("outside", self.tool, moves, clearance_z_mm=8, **DECLARED)
        )
        with self.assertRaises(ExportRefused) as refused:
            job.post_nc("top", machine=self.machine, accept=ACCEPT)
        self.assertEqual(refused.exception.statuses, ("violation",))
        (result,) = refused.exception.findings
        self.assertEqual((result["findings"][0]["kind"], result["findings"][0]["part"]), ("stock_contact", "cutter"))
        # The travel move, then supplied move 0.
        self.assertEqual(result["completed"], len(self.planned_motions["face"]) + 2)
        released = job.post_nc("top", machine=self.machine, accept=ACCEPT | {"violation"})
        self.assertIn("G0 X6 Y6.5 Z4\n", released.text)

    def run_error(self, moves, **changes):
        operation = SuppliedToolpath("outside", self.tool, moves, **{"clearance_z_mm": 8, **DECLARED, **changes})
        with self.assertRaises(SimulationError) as caught:
            collect(with_operation(self.planned, 1, operation))
        return str(caught.exception)

    def test_malformed_moves_are_refused_with_their_index(self):
        good = slot()

        def changed(index, **fields):
            return (*good[:index], replace(good[index], **fields), *good[index + 1 :])

        y = good[0].curve.start[1]
        cases = (
            (changed(2, feed_mm_min=None), "move 2 of 7: a cut move needs a feed"),
            (changed(4, feed_mm_min=-600), "move 4 of 7: feed_mm_min -600 must be positive and finite"),
            (changed(3, kind="rapid", feed_mm_min=None), "move 3 of 7: a rapid must be a line"),
            (changed(1, kind="tap"), "move 1 of 7: kind \"tap\" belongs to a cycle"),
            (
                changed(2, curve=Line((6, math.nextafter(y, 7), 4), (12, y, 4))),
                "move 2 of 7: starts at",
            ),
            (
                changed(3, curve=Arc((12, y, 4), (12, y + 4, 4), (12, y + 2.001, 4), math.pi)),
                "move 3 of 7: circular endpoint disagrees",
            ),
            (good[1:], "move 0 of 6: starts at Z 5.5, below clearance_z_mm 8"),
            (good[:-1], "move 5 of 6: ends at Z 3.5, below clearance_z_mm 8"),
        )
        for moves, message in cases:
            with self.subTest(message):
                error = self.run_error(moves)
                self.assertIn("supplied toolpath outside: " + message, error)
        self.assertIn("consecutive moves must share that point exactly", self.run_error(cases[4][0]))
        # What JSON cannot carry is refused as the operation is built.
        for value in (math.nan, math.inf):
            with self.assertRaisesRegex(ValueError, "supplied toolpath outside: move 2: .* not a finite number"):
                SuppliedToolpath("outside", self.tool, changed(2, curve=Line((6, y, 4), (value, y, 4))), 8, **DECLARED)
        with self.assertRaisesRegex(ValueError, "move 1: nan is not a finite number"):
            SuppliedToolpath("outside", self.tool, changed(1, feed_mm_min=math.nan), 8, **DECLARED)
        with self.assertRaisesRegex(TypeError, "move 1: expected a ToolpathMove"):
            SuppliedToolpath("outside", self.tool, (good[0], "G1 X1"), 8, **DECLARED)
        with self.assertRaisesRegex(ValueError, "move 0: the curve must be a Line, Arc or Helix"):
            SuppliedToolpath("outside", self.tool, (ToolpathMove("cut", (1, 2, 3), 100),), 8, **DECLARED)
        with self.assertRaisesRegex(TypeError, "finite collection"):
            SuppliedToolpath("outside", self.tool, iter(good), 8, **DECLARED)

    def test_the_operation_declares_its_machine_values_and_the_run_checks_stock_contact(self):
        for missing in DECLARED:
            declared = {k: v for k, v in DECLARED.items() if k != missing}
            with self.assertRaisesRegex(TypeError, missing):
                SuppliedToolpath("outside", self.tool, slot(), 8, **declared)
        self.assertIn("spindle_rpm 0 must be positive", self.run_error(slot(), spindle_rpm=0))
        self.assertIn("clearance_z_mm 5 must be above the stock top 6", self.run_error(slot(), clearance_z_mm=5))
        operation = SuppliedToolpath("outside", self.tool, slot(), 8, **DECLARED)
        job = with_operation(self.planned, 1, operation, verification=None)
        with self.assertRaisesRegex(SimulationError, "supplied toolpath outside.*stock_contact"):
            collect(job)

    def test_camlib_motion_objects_are_taken_as_moves(self):
        moves = [Motion(900 + i, "elsewhere", "t9", m.curve, m.feed_mm_min, m.kind) for i, m in enumerate(slot())]
        operation = SuppliedToolpath("outside", self.tool, moves, 8, **DECLARED)
        self.assertEqual(
            operation.to_dict()["motions"],
            SuppliedToolpath("outside", self.tool, slot(), 8, **DECLARED).to_dict()["motions"],
        )
        self.assertEqual(ToolpathMove.from_motion(moves[3].to_dict()), slot()[3])

    def test_the_moves_are_part_of_the_job_identity_and_the_cache_key(self):
        operation = SuppliedToolpath("outside", self.tool, slot(), 8, **DECLARED)
        # Serialised once: the same list each time.
        self.assertIs(operation.to_dict()["motions"], operation.to_dict()["motions"])
        self.assertEqual(operation.to_dict()["kind"], "supplied_toolpath")
        job = with_operation(self.planned, 1, operation)
        key = job.compile().setup("top").dependencies["outside"]
        same = with_operation(self.planned, 1, SuppliedToolpath("outside", self.tool, list(slot()), 8, **DECLARED))
        self.assertEqual(same.compile().setup("top").dependencies["outside"], key)
        # One coordinate of one move, one unit in the last place.
        y = slot()[0].curve.start[1]
        moved = list(slot())
        moved[4] = replace(moved[4], curve=Line((12, y + 4, 4), (math.nextafter(6, 7), y + 4, 4)))
        moved[5] = replace(moved[5], curve=Helix((math.nextafter(6, 7), y + 4, 4), (6, y, 3.5), (6, y + 2, 4), math.pi))
        other = with_operation(self.planned, 1, SuppliedToolpath("outside", self.tool, moved, 8, **DECLARED))
        self.assertNotEqual(other.compile().setup("top").dependencies["outside"], key)
        refed = with_operation(
            self.planned,
            1,
            SuppliedToolpath(
                "outside", self.tool, (*slot()[:2], replace(slot()[2], feed_mm_min=613), *slot()[3:]), 8, **DECLARED
            ),
        )
        self.assertNotEqual(refed.compile().setup("top").dependencies["outside"], key)
        with TemporaryDirectory() as directory:
            cache = RegenerationCache(Path(directory) / "cache")
            first, controls, _ = collect(job, cache)
            self.assertFalse(controls["outside"]["end"].report["regeneration"]["cache_hit"])
            again, controls, _ = collect(job, cache)
            self.assertTrue(controls["outside"]["end"].report["regeneration"]["cache_hit"])
            self.assertEqual(again["outside"], first["outside"])
            changed, controls, _ = collect(refed, cache)
            self.assertFalse(controls["outside"]["end"].report["regeneration"]["cache_hit"])
            self.assertEqual(changed["outside"][3]["feed_mm_min"], 613)

    def test_a_lathe_setup_takes_no_supplied_toolpath(self):
        import test_turning

        operation = SuppliedToolpath("outside", "cnmg", slot(), 8, **DECLARED)
        lathe = test_turning.job(operations=(camlib.TurnFace("face", "cnmg", 0, feed_mm_rev=0.15, rpm=1000), operation))
        with self.assertRaisesRegex(ValueError, "supplied toolpath outside: .*three-axis milling"):
            lathe.job.compile()

    def test_a_beam_takes_no_supplied_toolpath(self):
        torch = camlib.Torch("torch", kerf_width=1.5)
        setup = self.planned.setups[0]
        operation = SuppliedToolpath("outside", torch, slot(), 8, **DECLARED)
        job = replace(self.planned, setups=(replace(setup, tools=(torch,), operations=(operation,)),))
        with self.assertRaisesRegex(SimulationError, "three-axis milling"):
            collect(job)


if __name__ == "__main__":
    unittest.main()
