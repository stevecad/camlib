import importlib.util
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]


def example():
    spec = importlib.util.spec_from_file_location("standalone_post_nc", ROOT / "examples/standalone_job.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class PostNcTest(unittest.TestCase):
    def test_direct_posting_writes_the_same_checked_program_as_post(self):
        ex = example()
        job = ex.build_job()
        machine = ex.machine_for(job)
        with job.post("top", machine=machine) as posted:
            posted.wait()
            nc, start = [], 1
            while page := posted.nc_lines(start, limit=256):
                nc.extend(line["text"] for line in page)
                start = page[-1]["number"] + 1
            motions = posted.status()["posting"]["checked_motions"]
        program = job.post_nc("top", machine=machine, accept={"unchecked_machine"})
        self.assertEqual(program.text, "".join(line + "\n" for line in nc))
        self.assertEqual((program.lines, program.motions, program.findings), (len(nc), motions, ()))
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "job.nc"
            written = job.post_nc("top", machine=machine, path=target, accept={"unchecked_machine"})
            self.assertEqual(target.read_text(), program.text)
            self.assertEqual((written.text, written.path, written.lines), ("", str(target), program.lines))

    def test_material_checks_use_the_posted_stock(self):
        ex = example()
        job = ex.build_job()
        setup = job.setups[0]
        from camlib.verification import MaterialCheck, StreamChecks
        from camlib.material import Target
        from dataclasses import replace

        op = getattr(setup.operations[-1], "operation", setup.operations[-1])
        checks = StreamChecks(material=(MaterialCheck(Target("floor", "1", op.region, 3)),))
        checked = replace(job, setups=(replace(setup, verification=checks),))
        with self.assertRaisesRegex(ValueError, "material gouge definite_excess on target floor"):
            checked.post_nc("top", machine=ex.machine_for(job), accept={"unchecked_machine"})


if __name__ == "__main__":
    unittest.main()
