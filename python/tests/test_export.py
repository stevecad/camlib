"""Export acceptance: no machine-ready program with unaccepted check results.

Built on the QA review's reproduction: a fixture prism in the standalone
example's cutting path.
"""

from dataclasses import replace
import importlib.util
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from camlib.export import ExportRefused, describe
from camlib.material import Target
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.verification import MaterialCheck, Prism, Scene, StockContactOptions, StreamChecks

ROOT = Path(__file__).resolve().parents[2]


def example():
    spec = importlib.util.spec_from_file_location("standalone_export", ROOT / "examples/standalone_job.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def with_fixture(job):
    footprint = PlanarRegion("fixture-footprint", (Component(Boundary.polygon(((4, 4), (14, 4), (14, 12), (4, 12)))),))
    fixture = Prism("blocking-fixture", footprint, 0, 10, role="fixture")
    checks = StreamChecks(scene=Scene("export-scene", "1", bodies=(fixture,)), stock_contact=StockContactOptions())
    return replace(job, setups=(replace(job.setups[0], verification=checks),))


class ExportTest(unittest.TestCase):
    def setUp(self):
        self.module = example()

    def test_violations_refuse_text_and_file_and_keep_the_old_file(self):
        job = with_fixture(self.module.build_job())
        machine = self.module.machine_for(job)
        with TemporaryDirectory() as directory:
            target = Path(directory) / "part.nc"
            with self.assertRaises(ExportRefused) as refused:
                job.post_nc("top", machine=machine, path=target, accept={"unchecked_machine"})
            self.assertFalse(target.exists())
            self.assertEqual(refused.exception.statuses, ("violation",))
            first = refused.exception.findings[0]["findings"][0]
            self.assertEqual((first["kind"], first["object_id"]), ("fixture_contact", "blocking-fixture"))
            self.assertIn("blocking-fixture", str(refused.exception))
            target.write_text("previous program\n")
            with self.assertRaises(ExportRefused):
                job.post_nc("top", machine=machine, path=target, accept={"unchecked_machine"})
            self.assertEqual(target.read_text(), "previous program\n")
            self.assertFalse(list(Path(directory).glob("*.writing")))
        with self.assertRaises(ExportRefused):
            job.post_nc("top", machine=machine, accept={"unchecked_machine"})

    def test_explicit_acceptance_releases_and_is_recorded(self):
        job = with_fixture(self.module.build_job())
        with TemporaryDirectory() as directory:
            target = Path(directory) / "part.nc"
            result = job.post_nc(
                "top",
                machine=self.module.machine_for(job),
                path=target,
                accept={"violation", "unchecked_machine"},
            )
            self.assertTrue(target.read_text().endswith("M2\n"))
            self.assertEqual(result.accepted, ("unchecked_machine", "violation"))
            self.assertTrue(all(f["status"] == "violation" for f in result.findings))
        with self.assertRaisesRegex(ValueError, "accept takes"):
            job.post_nc(
                "top", machine=self.module.machine_for(job), accept=(set({"everything"}) | {"unchecked_machine"})
            )

    def test_uncertain_stock_checks_require_their_own_explicit_acceptance(self):
        job = self.module.build_job()
        checks = StreamChecks(stock_contact=StockContactOptions(max_nodes=1, max_cell_reads=1))
        job = replace(job, setups=(replace(job.setups[0], verification=checks),))
        machine = self.module.machine_for(job)
        with TemporaryDirectory() as directory:
            target = Path(directory) / "uncertain.nc"
            target.write_text("previous program\n")
            for accept in ((), {"violation"}):
                with self.assertRaises(ExportRefused) as refused:
                    job.post_nc("top", machine=machine, path=target, accept=(set(accept) | {"unchecked_machine"}))
                self.assertEqual(refused.exception.statuses, ("uncertain",))
                self.assertEqual(target.read_text(), "previous program\n")
            result = job.post_nc("top", machine=machine, accept={"uncertain", "unchecked_machine"})
            self.assertEqual(result.accepted, ("uncertain", "unchecked_machine"))
            self.assertTrue(result.findings)
            self.assertTrue(all(f["status"] == "uncertain" for f in result.findings))
            with job.post("top", machine=machine) as run:
                with self.assertRaises(ExportRefused):
                    run.export(target, accept={"violation", "unchecked_machine"})
                self.assertEqual(target.read_text(), "previous program\n")
                released = run.export(target, accept={"uncertain", "unchecked_machine"})
                self.assertEqual(released["accepted"], ("uncertain", "unchecked_machine"))
                self.assertEqual(target.read_text(), result.text)

    def test_unconfigured_checks_are_not_misrepresented_as_clear(self):
        job = self.module.build_job()
        job = replace(job, setups=(replace(job.setups[0], verification=None),))
        result = job.post_nc("top", machine=self.module.machine_for(job), accept={"unchecked_machine"})
        self.assertEqual(result.unchecked, ("scene", "stock_contact", "material", "machine"))

    def test_clean_job_exports_and_names_unconfigured_checks(self):
        job = self.module.build_job()
        result = job.post_nc("top", machine=self.module.machine_for(job), accept={"unchecked_machine"})
        self.assertEqual((result.findings, result.accepted), ((), ("unchecked_machine",)))
        self.assertEqual(result.unchecked, ("scene", "material", "machine"))
        self.assertTrue(result.text.endswith("M2\n"))

    def test_posted_run_export_applies_the_same_rule(self):
        clean = self.module.build_job()
        bad = with_fixture(clean)
        with TemporaryDirectory() as directory:
            with clean.post("top", machine=self.module.machine_for(clean)) as run:
                written = run.export(Path(directory) / "clean.nc", accept={"unchecked_machine"})
                text = Path(written["path"]).read_text()
                self.assertEqual(text.count("\n"), written["lines"])
                self.assertEqual(
                    text,
                    clean.post_nc("top", machine=self.module.machine_for(clean), accept={"unchecked_machine"}).text,
                )
            target = Path(directory) / "bad.nc"
            with bad.post("top", machine=self.module.machine_for(bad)) as run:
                with self.assertRaises(ExportRefused):
                    run.export(target, accept={"unchecked_machine"})
                self.assertFalse(target.exists())
                released = run.export(target, accept={"violation", "unchecked_machine"})
                self.assertEqual(released["accepted"], ("unchecked_machine", "violation"))
                self.assertTrue(target.is_file())

    def test_refused_material_result_names_its_target_and_assessment(self):
        # The target claims material 1 mm above the pocket floor the job cuts
        # (Z 3): the final stock is a definite gouge, refused at export.
        job = self.module.build_job()
        setup = job.setups[0]
        pocket = getattr(setup.operations[-1], "operation", setup.operations[-1])
        checks = StreamChecks(material=(MaterialCheck(Target("pocket-target", "1", pocket.region, 4)),))
        job = replace(job, setups=(replace(setup, verification=checks),))
        with TemporaryDirectory() as directory:
            target = Path(directory) / "gouged.nc"
            with job.post("top", machine=self.module.machine_for(job)) as run:
                with self.assertRaises(ExportRefused) as refused:
                    run.export(target, accept={"unchecked_machine"})
                self.assertFalse(target.exists())
        message = str(refused.exception)
        self.assertEqual(refused.exception.statuses, ("violation",))
        report = refused.exception.findings[0]["findings"][0]
        self.assertEqual(report["target_id"], "pocket-target")
        self.assertIn("material gouge definite_excess on target pocket-target", message)
        self.assertNotIn("None", message)


class DescribeTest(unittest.TestCase):
    """Every shape a gated result's findings take prints meaningfully."""

    @staticmethod
    def summary(assessment, definite=0, possible=0):
        return dict(assessment=assessment, definite_cells=definite, possible_only_cells=possible, volume_mm3=[0, 1])

    def test_milling_material_report_names_target_gouge_and_residual(self):
        report = dict(
            target_id="plate",
            complete=True,
            gouge=self.summary("possible_excess", possible=12),
            residual=self.summary("definite_excess", definite=3),
            findings=[],
        )
        text = describe(dict(completed=352, status="uncertain", findings=[report]))
        self.assertEqual(
            text,
            "material gouge possible_excess on target plate (definite 0 cells, possible-only 12 cells), "
            "residual definite_excess (definite 3 cells, possible-only 0 cells)",
        )

    def test_material_residual_within_tolerance_and_incomplete_analysis(self):
        report = dict(
            target_id="plate",
            complete=False,
            gouge=self.summary("incomplete", possible=4),
            residual=self.summary("within_tolerance"),
        )
        text = describe(dict(completed=1, status="uncertain", findings=[report]))
        self.assertEqual(
            text,
            "material gouge incomplete on target plate (definite 0 cells, possible-only 4 cells), analysis incomplete",
        )

    def test_lathe_material_report_names_the_turned_part(self):
        report = dict(
            algorithm="turn-radius-bounds-v1",
            complete=True,
            gouge=self.summary("within_tolerance"),
            residual=self.summary("possible_excess", possible=2),
            findings=[],
        )
        text = describe(dict(completed=9, status="uncertain", findings=[report]))
        self.assertIn("material gouge within_tolerance on the turned part", text)
        self.assertIn("residual possible_excess", text)

    def test_contact_findings_print_kind_and_object_as_before(self):
        scene = dict(kind="fixture_contact", object_id="vise", source="scene")
        stock = dict(finding=dict(kind="stock_contact", object_id="stock"), cell_range=[0, 1, 0, 1])
        self.assertEqual(describe(dict(status="violation", findings=[scene])), "fixture_contact vise")
        self.assertEqual(describe(dict(status="uncertain", findings=[stock])), "stock_contact stock")

    def test_result_without_findings_never_prints_none(self):
        text = describe(dict(completed=4, status="uncertain", findings=[]))
        self.assertEqual(text, "uncertain result without a localized finding")
        refused = ExportRefused([dict(completed=4, status="uncertain", findings=[])], {"uncertain"})
        self.assertNotIn("None", str(refused))


if __name__ == "__main__":
    unittest.main()
