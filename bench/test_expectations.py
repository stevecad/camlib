"""Regression coverage for reviewed benchmark expectations."""

import copy
import json
from pathlib import Path
import tempfile
import unittest

import compare
from expectations import project, update


class ExpectationsTest(unittest.TestCase):
    def setUp(self):
        self.data = json.loads((Path(__file__).parent / "fixtures/report-sample.json").read_text(encoding="utf-8"))
        self.row = self.data["workloads"][0]

    def test_projection_retains_comparison_semantics(self):
        base = {"workloads": [project(self.row)]}
        self.assertNotIn("engine", base["workloads"][0])
        self.assertEqual(compare.toolpath_changes(base, self.data), [])
        changed = copy.deepcopy(self.data)
        toolpath = changed["workloads"][0]["toolpath"]
        toolpath["cycle_seconds_tangent"] += 1
        toolpath["material"]["leftover_mm3"][1] += 1
        self.assertEqual(compare.toolpath_changes(base, changed), compare.toolpath_changes(self.data, changed))
        self.assertEqual(len(compare.toolpath_changes(base, changed)), 2)
        self.assertEqual(compare.toolpath_changes(base, {"workloads": []})[0][2], "workload")

    def test_update_is_selective_and_validates_before_writing(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            unrelated = root / "other.json"
            unrelated.write_text("unrelated")
            name = self.row["name"]
            update(self.data, [name], root)
            expected = root / f"{name}.json"
            original = expected.read_bytes()
            self.row["toolpath"]["moves"] += 1
            with self.assertRaises(ValueError):
                update(self.data, [name, "absent"], root)
            self.assertEqual(expected.read_bytes(), original)
            update(self.data, [name], root)
            self.assertNotEqual(expected.read_bytes(), original)
            self.assertEqual(unrelated.read_text(), "unrelated")
            self.row["operations"].clear()
            with self.assertRaises(ValueError):
                update(self.data, [name], root)
            self.data["failures"] = {name: ["failed"]}
            with self.assertRaises(ValueError):
                update(self.data, [name], root)
