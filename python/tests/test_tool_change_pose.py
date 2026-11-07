"""Multi-tool jobs and the machine's tool-change pose (#65): the setup plans
each new tool where the machine's tool change leaves it, the posted replay
accepts a tool table's poses, and a setup that disagrees with the machine is
refused before posting with both poses and the fix named."""

import unittest

import camlib
from camlib import GCodeDialect, Machine, ToolChangePosition, ToolChangeTable, ToolSetting
from camlib.motion import WorkFrame

HOLES = ((8, 8), (20, 8))


def tools():
    drill = camlib.ToolAssembly(
        "drill5",
        camlib.Drill("drill5", diameter=5, point_angle=118, flute_length=20, overall_length=60),
        40,
        80,
        (camlib.HolderSection(40, 80, 10, 10),),
    )
    spot = camlib.ToolAssembly(
        "drill3",
        camlib.Drill("drill3", diameter=3, point_angle=118, flute_length=12, overall_length=50),
        30,
        75,
        (camlib.HolderSection(30, 75, 10, 10),),
    )
    return drill, spot


def job(tool_change):
    drill, spot = tools()
    ops = tuple(
        camlib.Drilling(
            f"drill{k}",
            tool,
            HOLES,
            depth,
            12,
            tool_number=k,
            length_register=k,
            feed_mm_min=150,
            retract_feed_mm_min=600,
            spindle_rpm=6000,
            coolant="flood",
        )
        for k, (tool, depth) in enumerate(((spot, 8.0), (drill, 4.0)), 1)
    )
    setup = camlib.Setup(
        "top",
        stock=camlib.StockSpec((0, 0, 0), (28, 16, 10), 0.1),
        tools=(drill, spot),
        operations=tuple(camlib.OperationNode(op, sources=()) for op in ops),
        initial_position_mm=(0, 0, 40),
        verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
        tool_change=tool_change,
    )
    return camlib.Job("tool-change", "1", (setup,))


def machine(table, mode="table", **settings):
    drill, spot = tools()
    by_id = {"drill5": drill, "drill3": spot}
    numbers = {"drill3": 1, "drill5": 2}
    return Machine(
        "Three-axis mill",
        tuple(
            ToolSetting.for_tool(by_id[t], number=numbers[t], length_register=numbers[t], position_after_mm=tip)
            for t, tip in table.items()
        ),
        tool_change_position_mode=mode,
        **{
            "dialect": GCodeDialect.linuxcnc(),
            "output_units": "mm",
            "decimals": 3,
            "max_feed_mm_min": 10000,
            "max_spindle_rpm": 30000,
            "planes": ("xy", "xz", "yz"),
            "arcs": True,
            "helices": True,
            "max_sweep_rad": 6.283185307179586,
            "max_tool_number": 99,
            "max_length_register": 99,
            "tool_changes": True,
            "flood": True,
            "mist": False,
            "rapid_mm_min": 6000,
            **settings,
        },
    )


# A tool table whose poses share no gauge line or XY: only a table can plan it.
TABLE = {"drill3": (0.0, 0.0, 45.0), "drill5": (5.0, -3.0, 38.0)}


def posted(j, m):
    with j.post("top", machine=m) as run:
        run.wait()
        nc, start = [], 1
        while page := run.nc_lines(start, limit=256):
            nc.extend(line["text"] for line in page)
            start = page[-1]["number"] + 1
        status = run.status()
        # The posted program was replayed into stock to the end.
        assert status["finished"] and status["error"] is None, status
        return nc


class ToolChangeTableTest(unittest.TestCase):
    def test_table_machine_posts_and_replays_a_multi_tool_job(self):
        m = machine(TABLE)
        j = job(ToolChangeTable.from_machine(m))
        nc = posted(j, m)
        self.assertEqual(sum("M6" in line.split() for line in nc), 2)
        # The same job without the posted replay agrees.
        self.assertEqual(len(j.post_nc("top", machine=m).text.splitlines()), len(nc))

    def test_planner_starts_each_new_tool_at_its_table_pose(self):
        from camlib.simulation import OperationUpdate

        m = machine(TABLE)
        starts = {}
        with job(ToolChangeTable.from_machine(m)).regenerate() as run:
            for event in run:
                update = event.update
                if isinstance(update, OperationUpdate) and update.stage == "begin":
                    change = next(e for e in update.events if e["kind"] == "tool_change")
                    starts[change["tool"]] = tuple(change["position_after_mm"])
        self.assertEqual(starts, TABLE)

    def test_gauge_position_matching_the_table_also_posts(self):
        # A table whose tips are a common gauge line over a fixed XY is the
        # same machine as ToolChangePosition: both plans post.
        drill, spot = tools()
        table = {t.id: (0.0, 0.0, 100 - t.gauge_length_mm) for t in (drill, spot)}
        m = machine(table)
        a = posted(job(ToolChangePosition(gauge_z_mm=100, xy_mm=(0, 0))), m)
        b = posted(job(ToolChangeTable.from_machine(m)), m)
        self.assertEqual(a, b)


class PrePostCheckTest(unittest.TestCase):
    def test_table_machine_without_setup_pose_is_refused_naming_the_fix(self):
        with self.assertRaisesRegex(ValueError, r"tool drill3: .*\(0, 0, 40\).*\(0\.0, 0\.0, 45\.0\).*from_machine"):
            job(None).post_nc("top", machine=machine(TABLE))

    def test_wrong_gauge_line_is_refused_before_posting(self):
        drill, spot = tools()
        m = machine({t.id: (0.0, 0.0, 100 - t.gauge_length_mm) for t in (drill, spot)})
        with self.assertRaisesRegex(
            ValueError, "tool drill3: the setup starts it at .*Setup.tool_change is ToolChangePosition"
        ):
            job(ToolChangePosition(gauge_z_mm=90, xy_mm=(0, 0))).post_nc("top", machine=m)
        # The replayed post is checked the same way, before anything runs.
        with self.assertRaisesRegex(ValueError, "tool drill3"):
            job(ToolChangePosition(gauge_z_mm=90, xy_mm=(0, 0))).post("top", machine=m)

    def test_preserve_tip_machine_refuses_a_setup_pose(self):
        m = machine(dict.fromkeys(TABLE, (0.0, 0.0, 40.0)), mode="preserve_tip")
        with self.assertRaisesRegex(ValueError, "preserve_tip.*Setup\\(tool_change=None\\)"):
            job(ToolChangeTable(TABLE)).post_nc("top", machine=m)

    def test_z_home_machine_refuses_a_fixed_xy_and_a_table(self):
        m = machine(
            dict.fromkeys(TABLE, (0.0, 0.0, 40.0)),
            mode="z_home",
            tool_change_home_z_mm=100,
            work_to_machine=WorkFrame(),
        )
        for change in (ToolChangePosition(gauge_z_mm=100, xy_mm=(0, 0)), ToolChangeTable(TABLE)):
            with self.assertRaisesRegex(ValueError, r"Z home.*ToolChangePosition\(gauge_z_mm=100"):
                job(change).post_nc("top", machine=m)
        with self.assertRaisesRegex(ValueError, "tool drill3: .*Z home: gauge line at work Z 100"):
            job(ToolChangePosition(gauge_z_mm=95)).post_nc("top", machine=m)


class TableValueTest(unittest.TestCase):
    def test_validation_and_machine_table(self):
        with self.assertRaises(ValueError):
            ToolChangeTable({})
        with self.assertRaises(ValueError):
            ToolChangeTable({"a": (0, 0, float("nan"))})
        table = ToolChangeTable({"b": (1, 2, 3), "a": (4, 5, 6)})
        self.assertEqual(table.tips_mm, (("a", (4, 5, 6)), ("b", (1, 2, 3))))
        self.assertEqual(table.to_dict(), {"tips_mm": {"a": [4.0, 5.0, 6.0], "b": [1.0, 2.0, 3.0]}})
        self.assertEqual(ToolChangeTable.from_machine(machine(TABLE)).tip("drill5"), (5.0, -3.0, 38.0))
        with self.assertRaisesRegex(ValueError, "not a table"):
            ToolChangeTable.from_machine(machine(TABLE, mode="preserve_tip"))
        with self.assertRaisesRegex(ValueError, "no entry for tool"):
            ToolChangeTable.from_machine(machine(TABLE), tools=("drill5", "other"))

    def test_engine_refuses_a_table_for_an_undeclared_tool(self):
        with self.assertRaisesRegex(Exception, "tool change pose needs finite work coordinates, for declared tools"):
            with job(ToolChangeTable({**TABLE, "ghost": (0, 0, 50)})).regenerate() as run:
                list(run)


if __name__ == "__main__":
    unittest.main()
