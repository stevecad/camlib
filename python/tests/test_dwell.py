from camlib.postprocessor import GCodeDialect
import importlib.util
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import camlib
from camlib.post import Machine, ToolSetting
from camlib import RegenerationCache
from camlib.simulation import OperationUpdate
from test_tapping import posted_text, tools

ROOT = Path(__file__).resolve().parents[2]
HOLES = ((8, 8), (20, 8))


def job(dwell_s=0.5, **drilling):
    drill, _ = tools()
    op = camlib.Drilling(
        "spot",
        drill,
        HOLES,
        7,
        12,
        dwell_s=dwell_s,
        **{
            'feed_mm_min': 150,
            'retract_feed_mm_min': 600,
            'spindle_rpm': 6000,
            'tool_number': 1,
            'length_register': 1,
            'coolant': "flood",
            **drilling,
        },
    )
    setup = camlib.Setup(
        "top",
        stock=camlib.StockSpec((0, 0, 0), (28, 16, 10), 0.1),
        tools=(drill,),
        operations=(camlib.OperationNode(op, sources=()),),
        initial_position_mm=(0, 0, 12),
        verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
    )
    return camlib.Job("dwell", "1", (setup,))


def tool_table(j):
    header = j.compile().setup("top").program_header()
    return tuple(
        ToolSetting.for_tool(t, number=i, length_register=i, position_after_mm=header.initial_position_mm)
        for i, t in enumerate(header.tools, 1)
    )


def example(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / f"examples/{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class DwellTest(unittest.TestCase):
    def test_dwell_follows_each_full_depth_cut(self):
        j = job()
        nc = posted_text(
            j,
            Machine(
                "mill",
                tool_table(j),
                tool_change_position_mode="preserve_tip",
                dialect=GCodeDialect.linuxcnc(),
                output_units='mm',
                decimals=3,
                max_feed_mm_min=10000,
                max_spindle_rpm=30000,
                planes=('xy', 'xz', 'yz'),
                arcs=True,
                helices=True,
                max_sweep_rad=6.283185307179586,
                max_tool_number=99999,
                max_length_register=99999,
                tool_changes=True,
                flood=True,
                mist=True,
                rapid_mm_min=6000,
            ),
        )
        dwells = [i for i, line in enumerate(nc) if line.startswith("G4 ")]
        self.assertEqual([nc[i] for i in dwells], ["G4 P0.5"] * 2)
        for i in dwells:
            self.assertTrue(nc[i - 1].startswith("G1") and "Z7" in nc[i - 1], nc[i - 3 : i + 1])
        self.assertFalse(
            any(
                line.startswith("G4 ")
                for line in posted_text(
                    job(None),
                    Machine(
                        "mill",
                        tool_table(j),
                        tool_change_position_mode="preserve_tip",
                        dialect=GCodeDialect.linuxcnc(),
                        output_units='mm',
                        decimals=3,
                        max_feed_mm_min=10000,
                        max_spindle_rpm=30000,
                        planes=('xy', 'xz', 'yz'),
                        arcs=True,
                        helices=True,
                        max_sweep_rad=6.283185307179586,
                        max_tool_number=99999,
                        max_length_register=99999,
                        tool_changes=True,
                        flood=True,
                        mist=True,
                        rapid_mm_min=6000,
                    ),
                )
            )
        )

    def test_post_nc_and_cached_regeneration_keep_the_events(self):
        j = job(1.25, peck_mm=2)
        m = Machine(
            "mill",
            tool_table(j),
            tool_change_position_mode="preserve_tip",
            dialect=GCodeDialect.linuxcnc(),
            output_units='mm',
            decimals=3,
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            planes=('xy', 'xz', 'yz'),
            arcs=True,
            helices=True,
            max_sweep_rad=6.283185307179586,
            max_tool_number=99999,
            max_length_register=99999,
            tool_changes=True,
            flood=True,
            mist=True,
            rapid_mm_min=6000,
        )
        self.assertEqual(j.post_nc("top", machine=m, accept={"unchecked_machine"}).text.count("G4 P1.25\n"), 2)
        with TemporaryDirectory() as directory:
            cache = RegenerationCache(directory)
            for hit in (False, True):
                with j.regenerate(cache) as run:
                    updates = [u.update for u in run]
                ops = [u for u in updates if isinstance(u, OperationUpdate)]
                self.assertEqual(ops[-1].report["regeneration"]["cache_hit"], hit)
                events = [u for u in ops if u.stage == "events"]
                self.assertEqual([e["seconds"] for u in events for e in u.events], [1.25, 1.25])
                self.assertTrue(all(u.report is None for u in events))

    def test_controller_resolution_rounds_up_and_unreviewed_posts_refuse(self):
        centroid = example("centroid_post")
        j = job(0.333)
        tools = tool_table(j)
        from camlib.motion import WorkFrame

        m = centroid.define_machine(
            "Acorn",
            tools,
            traverse_feed_mm_min=3000,
            work_to_machine=WorkFrame(),
            decimals=3,
            output_units='mm',
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            planes=('xy', 'xz', 'yz'),
            arcs=True,
            helices=True,
            max_sweep_rad=6.283185307179586,
            max_tool_number=99999,
            max_length_register=99999,
            tool_changes=True,
            flood=True,
            mist=True,
            rapid_mm_min=6000,
        )
        nc = posted_text(j, m, centroid.CentroidPost, centroid.CentroidDecoder)
        self.assertEqual([line for line in nc if line.startswith("G4 ")], ["G4 P0.34"] * 2)
        with self.assertRaisesRegex(Exception, "outside the controller range"):
            posted_text(job(400), m, centroid.CentroidPost, centroid.CentroidDecoder)
        haas = example("haas_ngc_post")
        h = haas.define_machine(
            "Haas",
            tools,
            traverse_feed_mm_min=5000,
            coordinate_system="fanuc",
            h_t_agreement=True,
            g92_shift_mm=(0, 0, 0),
            rotation_scaling=False,
            work_to_machine=WorkFrame(),
            decimals=3,
            tool_change_position_mode="preserve_tip",
            output_units='mm',
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            max_sweep_rad=6.283185307179586,
            tool_changes=True,
            flood=True,
            rapid_mm_min=6000,
        )
        # Haas G04: P in seconds with a decimal point (96-8210 Rev N, G04
        # note), millisecond resolution: the planned 0.333 s exactly.
        nc = posted_text(j, h, haas.HaasNGCPost, haas.HaasNGCDecoder)
        self.assertEqual([line for line in nc if line.startswith("G4 ")], ["G4 P0.333"] * 2)


if __name__ == "__main__":
    unittest.main()
