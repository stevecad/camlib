"""Platform regression tests: python -m unittest discover -s bench -p test_*.py."""

import contextlib
import io
import json
from pathlib import Path
import unittest
from unittest.mock import patch

import report
import run


class PlatformTest(unittest.TestCase):
    def test_windows_measurement_reports_missing_rss(self):
        output = io.StringIO()
        with (
            patch.object(run.sys, 'platform', 'win32'),
            patch.object(run, 'measure', return_value={'engine': {}}),
            contextlib.redirect_stdout(output),
        ):
            run._child('synthetic', 1)
        self.assertEqual(json.loads(output.getvalue())['engine']['peak_rss_kib'], {'python': None, 'native': None})

    def test_report_accepts_windows_measurements(self):
        data = json.loads((Path(__file__).parent / 'results' / 'baseline.json').read_text(encoding='utf-8'))
        for workload in data['workloads']:
            workload['engine']['peak_rss_kib'] = {'python': None, 'native': None}
        self.assertIn('unavailable', report.render(data))
