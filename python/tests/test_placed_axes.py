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
from test_rust_post import ROOT, example, plain
from test_tool_change_pose import job as drilling_job, machine as mill, tools as drilling_tools

# Work zero in machine coordinates: the mill homes at X 0, Y 0 (its travel
# minimums) and Z 0 (its maximum), 250, 200 and 150 mm from work zero.
ORIGIN = (250.0, 200.0, -150.0)


def homed_model(tool, register, work_error_mm, z_max_mm=0):
    """A mill homed on its limits, the work frame known to +-work_error_mm.
    z_max_mm: the Z travel maximum (home is Z 0)."""
    g = tool.gauge_length_mm
    return MachineModel(
        1,
        'mm',
        'homed-at-limits',
        '1',
        'milling',
        (Axis('head', 0, 500), Axis('head', 0, 400), Axis('head', -300, z_max_mm)),
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


def facing_from_home(work_error_mm=0.02, tool_change=None):
    """A Facing whose start is the home pose, and the homed LinuxCNC mill.
    tool_change: the setup's (ToolChangeInPlace unless given), from the
    home pose."""
    spec = facing_job()
    tool = spec['tools'][0]
    home = (-ORIGIN[0], -ORIGIN[1], -ORIGIN[2] - tool.gauge_length_mm)
    setup = camlib.Setup(
        'top',
        stock=spec['stock'],
        tools=spec['tools'],
        operations=tuple(camlib.OperationNode(op, sources=()) for op in spec['operations']),
        initial_position_mm=home,
        tool_change=ToolChangeInPlace() if tool_change is None else tool_change(tool, home),
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


class OffsetBlockTest(unittest.TestCase):
    """#369: Haas and FANUC headers clear the local shift with G52 X0 Y0 Z0
    (Haas also mirror image, G100 X0 Y0 Z0). Those blocks set an offset or
    a mode and move no axis; their decoders read them so (hold_axes), and
    the machine check keeps the axes they name exactly at home."""

    def facing(self, controller):
        # The FANUC reference declares where M6 leaves the spindle: here the
        # table pose of the tool is the home pose the program starts at.
        table = controller == 'fanuc_0imb_post'
        job, _, home = facing_from_home(
            tool_change=(lambda tool, home: camlib.ToolChangeTable({tool.id: home})) if table else None
        )
        tool = facing_job()['tools'][0]
        declared = dict(
            output_units='mm',
            decimals=3,
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            max_sweep_rad=6.283185307179586,
            tool_changes=True,
            flood=True,
            rapid_mm_min=6000,
            tool_change_position_mode='preserve_spindle',
            # FANUC activates the length offset with G1 G43 H Z (B-63844EN
            # pp. 199-206), which commands Z: there Z is homed below its
            # maximum, X and Y still on their minimums.
            crash_model=homed_model(tool, 1, 0.02, z_max_mm=50 if table else 0),
        )
        setting = ToolSetting.for_tool(tool, number=1, length_register=1)
        module = example(controller)
        if controller == 'haas_ngc_post':
            m = module.define_machine(
                'Haas homed at its limits',
                (setting,),
                traverse_feed_mm_min=3000,
                coordinate_system='fanuc',
                h_t_agreement=True,
                g92_shift_mm=(0, 0, 0),
                rotation_scaling=False,
                work_to_machine=WorkFrame(ORIGIN),
                **declared,
            )
            return job, m, module.HaasNGCPost, module.HaasNGCDecoder
        declared['tool_change_position_mode'] = 'table'
        m = module.define_machine(
            'FANUC homed at its limits',
            (replace(setting, position_after_mm=home),),
            startup_units='mm',
            startup_standard_modes=True,
            g92_shift_mm=(0, 0, 0),
            m_codes=dict(
                tool_change='M6', spindle_cw='M3', spindle_ccw='M4', spindle_off='M5', coolant_off='M9', flood='M8'
            ),
            traverse_feed_mm_min=3000,
            work_to_machine=WorkFrame(ORIGIN),
            **declared,
        )
        return job, m, module.Fanuc0iMBPost, module.Fanuc0iMBDecoder

    def test_haas_and_fanuc_headers_leave_the_axes_at_home(self):
        for controller, block in (('haas_ngc_post', 'G52 X0 Y0 Z0'), ('fanuc_0imb_post', 'G52 X0. Y0. Z0.')):
            with self.subTest(controller):
                job, m, post, decoder = self.facing(controller)
                released = job.post_nc('top', machine=m, post=post, decoder=decoder)
                self.assertIn(block, released.text.splitlines())
                # X and Y are not written before the first traverse; the
                # header names them only as G52's (and G100's) parameters.
                self.assertEqual(released.machine['status'], 'clear', released.machine)
                self.assertTrue(released.machine['complete'])
                self.assertEqual(tuple(released.findings), ())
                if controller == 'haas_ngc_post':
                    self.assertIn('G100 X0 Y0 Z0', released.text.splitlines())
                    # The Rust Haas post and decoder release the same program.
                    rust = post_job(post_job_input(job, 'top', machine=m))
                    self.assertTrue(rust['released'], rust.get('refused'))
                    self.assertEqual(rust['program'], released.text)
                    self.assertEqual(rust['machine']['status'], 'clear')

    def test_a_block_no_decoder_classifies_still_commands_its_axes(self):
        # The same header read by a decoder that does not classify it: the
        # axis letters count as words, which command the axes at home
        # through the work offset, straddling the minimums by the interval.
        for controller in ('haas_ngc_post', 'fanuc_0imb_post'):
            with self.subTest(controller):
                job, m, post, decoder = self.facing(controller)

                class Unclassified(decoder):
                    def hold_axes(self):
                        pass

                with self.assertRaises(ExportRefused) as refused:
                    job.post_nc('top', machine=m, post=post, decoder=Unclassified)
                self.assertEqual(refused.exception.statuses, ('uncertain',))
                posted = job.post_nc('top', machine=m, post=post, decoder=Unclassified, accept={'uncertain'})
                details = ' '.join(f['detail'] for f in posted.machine['findings'])
                self.assertIn('straddles the minimum 0.000000 mm of declared [0, 500] machine travel', details)


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

    def test_a_block_the_decoder_held_leaves_placed_axes_placed(self):
        # The post keeps placed axes as the machine check reads the blocks:
        # after the decoder held a block, its axis letters are parameters.
        p = GCodePost(mill({'drill3': (0.0, 0.0, 40.0)}, mode='preserve_tip'))
        p._placed = (True, True, True)
        p._lines = ['G52 X0 Y0 Z0', 'G4 P1']
        p._track_placed('on_dwell', (True, False))
        self.assertEqual(p._placed, (True, True, True))
        p._track_placed('on_dwell')
        self.assertEqual(p._placed, (False, False, False))
        # _hold_placed tracks the hook's blocks again from the state before
        # it, leaving out those held; with none held it changes nothing.
        p._lines = ['G52 X0 Y0 Z0', 'G0 Y5']
        p._hold_placed('on_dwell', (True, True, True), (True, False))
        self.assertEqual(p._placed, (True, False, True))
        p._hold_placed('on_dwell', (True, True, True), ())
        self.assertEqual(p._placed, (True, False, True))


if __name__ == '__main__':
    unittest.main()
