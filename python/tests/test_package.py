import importlib.util
import json
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
import unittest

import camlib

ROOT = Path(__file__).resolve().parents[2]


class PackageTest(unittest.TestCase):
    def test_top_level_names_resolve_lazily_without_cad(self):
        for name in camlib.__all__:
            self.assertTrue(hasattr(camlib, name), name)
        from camlib import jobs

        self.assertIs(camlib.Setup, jobs.Setup)
        with self.assertRaises(AttributeError):
            camlib.not_a_name
        # A fresh interpreter: importing camlib must not pull in CadQuery.
        code = "import sys, camlib; camlib.Pocket; print('cadquery' in sys.modules)"
        out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, check=True)
        self.assertEqual(out.stdout.strip(), "False")

    def check_example(self, name, accept):
        with TemporaryDirectory(prefix="camlib example ") as folder:
            output = Path(folder) / "pocket.nc"
            command = [sys.executable, str(ROOT / "examples" / name), folder]
            if accept:
                command.append("--accept-unchecked-machine")
            result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=120)
            if accept:
                self.assertEqual(result.returncode, 0, result.stderr)
                report = json.loads(result.stdout)
                self.assertIn("machine", report["unchecked"])
                self.assertEqual(Path(report["output"]), output)
                self.assertGreater(output.stat().st_size, 0)
            else:
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("ExportRefused", result.stderr)
                self.assertIn("unchecked_machine", result.stderr)
                self.assertFalse(output.exists())

    def test_standalone_cli_requires_machine_coverage(self):
        self.check_example("standalone_job.py", False)

    def test_standalone_cli_accepts_explicit_inspection_export(self):
        self.check_example("standalone_job.py", True)

    @unittest.skipUnless(importlib.util.find_spec("cadquery"), "CadQuery is optional")
    def test_cadquery_cli_requires_machine_coverage(self):
        self.check_example("cadquery_job.py", False)

    @unittest.skipUnless(importlib.util.find_spec("cadquery"), "CadQuery is optional")
    def test_cadquery_cli_accepts_explicit_inspection_export(self):
        self.check_example("cadquery_job.py", True)


if __name__ == "__main__":
    unittest.main()
