"""Axis words a straight move leaves out (#367): an axis still where the
controller placed it (the start pose, a tool change's machine position) gets
no word until the program writes it, since a word, even of the same value,
commands the axis through the controller's actual work offset. The Python
default post, its native formatter and the Rust post write the same blocks."""

from dataclasses import replace
import json
import unittest

import camlib
from camlib import ToolChangeInPlace, ToolChangePosition, ToolSetting
from camlib.export import ExportRefused
from camlib.machine import (
    AllowedContact,
    Axis,
    Box,
    MachineBody,
    MachineCheckOptions,
    MachineModel,
    ToolCalibration,
    ToolChangeRoute,
)
from camlib.motion import WorkFrame
from camlib.post import GCodePost, post_job, post_job_input
from camlib.verification import StockContactOptions
from support.facing import facing_job
from test_rust_post import ROOT, plain
from test_tool_change_pose import job as drilling_job, machine as mill, tools as drilling_tools

# Work zero in machine coordinates: the mill homes at X 0, Y 0 (its travel
# minimums) and Z 0 (its maximum), 250, 200 and 150 mm from work zero.
ORIGIN = (250.0, 200.0, -150.0)


def homed_model(tool, register, work_error_mm):
    """A mill homed on its limits, the work frame known to +-work_error_mm."""
    g = tool.gauge_length_mm
    return MachineModel(
        1,
        'mm',
        'homed-at-limits',
        '1',
        'milling',
        (Axis('head', 0, 500), Axis('head', 0, 400), Axis('head', -300, 0)),
        1,
        WorkFrame(ORIGIN),
        (-work_error_mm,) * 3,
        (work_error_mm,) * 3,
        (0, 0, 0),
        None,
        'linear',
        'synthetic exact-stop controller',
        (ToolCalibration(tool.id, register, (), (0, 0, g), (0, 0, g)),),
        (MachineBody('head', 'spindle_head', 'head', (Box((-1, -1, 0), (1, 1, 10)),), {}),),
        {'column': 'synthetic stand', 'table': 'remote support', 'enclosure': 'open stand'},
        (AllowedContact(('head', '@tool'), 'tool mounted in spindle'),),
        {
            tool.id: ToolChangeRoute(
                'synthetic complete indexing enclosure', (), (Box((-20, -20, -g - 1), (20, 20, 1)),), ('@tool',), ()
            )
        },
        (),
        MachineCheckOptions(0.00001, 0.05, 1_000_000, 128, StockContactOptions()),
    )


def facing_from_home(work_error_mm=0.02):
    """A Facing whose start is the home pose, and the homed LinuxCNC mill."""
    spec = facing_job()
    tool = spec['tools'][0]
    home = (-ORIGIN[0], -ORIGIN[1], -ORIGIN[2] - tool.gauge_length_mm)
    setup = camlib.Setup(
        'top',
        stock=spec['stock'],
        tools=spec['tools'],
        operations=tuple(camlib.OperationNode(op, sources=()) for op in spec['operations']),
        initial_position_mm=home,
        tool_change=ToolChangeInPlace(),
    )
    job = camlib.Job('facing-from-home', '1', (setup,))
    setting = ToolSetting.for_tool(tool, number=1, length_register=1)
    m = mill(
        {},
        mode='preserve_spindle',
        tool_table=(setting,),
        work_to_machine=WorkFrame(ORIGIN),
        crash_model=homed_model(tool, 1, work_error_mm),
    )
    return job, m, home


def motion_blocks(text):
    return [line for line in text.splitlines() if line.split()[:1] in (['G0'], ['G1'], ['G2'], ['G3'])]


class StartPoseTest(unittest.TestCase):
    def test_first_move_from_a_pose_on_the_travel_limits_names_only_the_axis_it_moves(self):
        job, m, home = facing_from_home()
        released = job.post_nc('top', machine=m)
        blocks = motion_blocks(released.text)
        # Z down from home: X and Y are where the controller homed them, so
        # the block names Z only. Then the first XY traverse names X and Y,
        # and Z, which the first block wrote.
        self.assertEqual(blocks[0], 'G0 Z12')
        self.assertTrue(blocks[1].startswith('G0 X') and ' Y' in blocks[1] and blocks[1].endswith(' Z12'), blocks)
        self.assertNotIn(f'X{home[0]:g}', released.text)
        # The machine check holds the unwritten X and Y exactly at home, on
        # their minimums, with a nonzero calibration interval: clear.
        self.assertEqual(released.machine['status'], 'clear', released.machine)
        self.assertTrue(released.machine['complete'])
        self.assertEqual(tuple(released.findings), ())
        # The Rust post writes the same program and releases it too.
        source = post_job_input(job, 'top', machine=m, run_id='homed-facing-fixture')
        source['manifest']['run']['id'] = 'homed-facing-fixture'
        rust = post_job(source)
        self.assertTrue(rust['released'], rust.get('refused'))
        self.assertEqual(rust['program'], released.text)
        self.assertEqual(rust['machine']['status'], 'clear')
        # tests/machine.rs posts this input without Python.
        path = ROOT / 'fixtures/posts/rust/homed-facing'
        self.assertEqual(plain(source), json.loads(path.with_suffix('.json').read_text()))
        self.assertEqual(released.text, path.with_suffix('.nc').read_text())

    def test_python_hooks_and_native_formatter_write_the_same_blocks(self):
        job, m, _ = facing_from_home()
        native = job.post_nc('top', machine=m).text

        class Hooks(GCodePost):
            # The same post through the Python hooks (an override turns the
            # native formatter off).
            def on_motion(self, motion):
                return super().on_motion(motion)

        self.assertEqual(job.post_nc('top', machine=m, post=Hooks).text, native)

    def test_without_the_rule_the_machine_check_refuses_the_first_block(self):
        # The program as posted before #367 (every word on every block): the
        # first block commands X and Y at home through the work offset, which
        # straddles the minimums by the calibration interval.
        job, m, home = facing_from_home()

        class EveryWord(GCodePost):
            def on_motion(self, motion):
                self._placed = (False, False, False)
                return super().on_motion(motion)

        with self.assertRaises(ExportRefused) as refused:
            job.post_nc('top', machine=m, post=EveryWord)
        self.assertEqual(refused.exception.statuses, ('uncertain',))
        posted = job.post_nc('top', machine=m, post=EveryWord, accept={'uncertain'})
        self.assertEqual(motion_blocks(posted.text)[0], f'G0 X{home[0]:g} Y{home[1]:g} Z12')
        details = ' '.join(f['detail'] for f in posted.machine['findings'])
        self.assertIn('straddles the minimum 0.000000 mm', details)


class ToolChangeTest(unittest.TestCase):
    def test_z_home_tool_change_places_z_until_the_program_moves_it(self):
        m = mill(
            {},
            mode='z_home',
            tool_table=tuple(
                ToolSetting.for_tool(t, number=k, length_register=k)
                for k, t in enumerate(reversed(drilling_tools()), 1)
            ),
            tool_change_home_z_mm=92,
            work_to_machine=WorkFrame(),
        )
        # Z home puts the second tool's tip (gauge 80) at the drilling
        # clearance, 12: its first move keeps Z.
        job = drilling_job(ToolChangePosition(gauge_z_mm=92))
        released = job.post_nc('top', machine=m, accept={'unchecked_machine'})
        lines = released.text.splitlines()
        changes = [i for i, line in enumerate(lines) if 'M6' in line.split()]
        self.assertEqual(len(changes), 2)
        first = [next(line for line in lines[c:] if line.split()[0] in ('G0', 'G1')) for c in changes]
        # The first tool (gauge 75) comes down from Z home to the clearance:
        # X and Y are still where the controller started, so Z only. The
        # second keeps the Z M6 sent it to and writes no Z word; X and Y,
        # written since, are written.
        self.assertEqual(first, ['G0 Z12', 'G0 X8 Y8'], lines)
        rust = post_job(post_job_input(job, 'top', machine=m, accept={'unchecked_machine'}))
        self.assertTrue(rust['released'])
        self.assertEqual(rust['program'], released.text)

    def test_placed_axes_follow_the_blocks_each_hook_writes(self):
        from camlib.motion import Cycle, ToolChange

        def post(mode, **settings):
            m = mill({'drill3': (0.0, 0.0, 40.0), 'drill5': (0.0, 0.0, 40.0)}, mode=mode, **settings)
            p = GCodePost(m)
            p._call(
                'on_begin', replace(drilling_job(None).compile().setup('top').program_header(), events=()), limit=32
            )
            return p

        p = post('table')
        self.assertEqual(p._placed, (True, True, True))
        p._call('on_tool_change', ToolChange('drill3', 1, 1, (0, 0, 40)))
        self.assertEqual(p._placed, (True, True, True))
        p._call('on_cycle', Cycle('drill', (8, 8), 4, 11, 40, 150, 3))
        self.assertEqual(p._placed, (False, False, False))
        p = post('z_home', tool_change_home_z_mm=100, work_to_machine=WorkFrame())
        p._placed = (False, False, False)
        p._call('on_tool_change', ToolChange('drill3', 1, 1, (0, 0, 25)))
        self.assertEqual(p._placed, (False, False, True))
        p = post('preserve_tip')
        p._placed = (False, False, False)
        p._call('on_tool_change', ToolChange('drill3', 1, 1, (0, 0, 40)))
        self.assertEqual(p._placed, (False, False, False))


if __name__ == '__main__':
    unittest.main()
