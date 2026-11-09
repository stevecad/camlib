"""Fine boring (G76): oriented spindle stop, shift away from the wall, rapid out."""

import unittest

import camlib
from camlib.motion import WorkFrame
from camlib.post import GCodeDialect, Machine, ToolSetting
from camlib.simulation import OperationUpdate, VerificationUpdate
from test_drill_cycles import HOLES, example, tool_table
from test_tapping import posted_text, tools

OEM = dict(tool_change='M6', spindle_cw='M3', spindle_ccw='M4', spindle_off='M5', coolant_off='M9', flood='M8')


def bar():
    holder = (camlib.HolderSection(35, 60, 6, 6),)
    # 8 mm bore from a 4 mm bar (inside the 5 mm pilot), 12 mm reach.
    return camlib.ToolAssembly(
        "bar8",
        camlib.BoringBar("b8", diameter=8, bar_diameter=4, reach=12, overall_length=60, shank_diameter=8),
        35,
        60,
        holder,
    )


def job(edge=0, **reaming):
    drill, _ = tools()
    boring = bar()
    values = dict(
        retract_z_mm=11,
        canned_cycle=True,
        tool_number=2,
        length_register=2,
        dwell_s=0.25,
        fine_bore=camlib.FineBore(shift_mm=0.5, edge_angle_deg=edge),
    )
    values.update(reaming)
    ops = (
        camlib.Drilling(
            "drill",
            drill,
            HOLES,
            0.5,
            12,
            tool_number=1,
            length_register=1,
            feed_mm_min=150,
            retract_feed_mm_min=600,
            spindle_rpm=6000,
            coolant="flood",
        ),
        camlib.Reaming(
            "bore",
            boring,
            HOLES,
            5,
            4,
            12,
            **{
                'feed_mm_min': 100,
                'retract_feed_mm_min': 100,
                'spindle_rpm': 1000,
                'tool_number': 1,
                'length_register': 1,
                'coolant': "flood",
                **values,
            },
        ),
    )
    setup = camlib.Setup(
        "top",
        stock=camlib.StockSpec((0, 0, 0), (28, 16, 10), 0.1),
        tools=(drill, boring),
        operations=tuple(camlib.OperationNode(op, sources=()) for op in ops),
        initial_position_mm=(0, 0, 12),
        verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
    )
    return camlib.Job("fine", "1", (setup,))


def table(j):
    """Tip poses after M6: the drill at the start, the bar over the last hole."""
    header = j.compile().setup("top").program_header()
    return tuple(
        ToolSetting.for_tool(t, number=i, length_register=i, position_after_mm=pose)
        for i, (t, pose) in enumerate(zip(header.tools, (header.initial_position_mm, (20, 8, 12))), 1)
    )


def fanuc(j, edge):
    f = example("fanuc_0imb_post")
    m = f.define_machine(
        "FANUC",
        table(j),
        startup_units="mm",
        startup_standard_modes=True,
        g92_shift_mm=(0, 0, 0),
        m_codes=OEM,
        traverse_feed_mm_min=3000,
        work_to_machine=WorkFrame(),
        decimals=3,
        tool_change_position_mode="table",
        fine_bore_edge_deg=edge,
        output_units='mm',
        max_feed_mm_min=10000,
        max_spindle_rpm=30000,
        max_sweep_rad=6.283185307179586,
        tool_changes=True,
        flood=True,
        rapid_mm_min=6000,
    )
    return m, f.Fanuc0iMBPost, f.Fanuc0iMBDecoder


class FineBoringTest(unittest.TestCase):
    def test_every_check_is_clear_and_the_spindle_orients_at_depth(self):
        for canned in (True, False):
            statuses, events = set(), []
            with job(canned_cycle=canned).regenerate() as run:
                for u in run:
                    v = u.update
                    if isinstance(v, VerificationUpdate):
                        statuses.add(v.status)
                    elif isinstance(v, OperationUpdate) and v.operation == "bore" and v.stage == "events":
                        events += [e["kind"] for e in v.events if e["kind"] != "cycle"]
            self.assertEqual(statuses, {"clear"}, canned)
            self.assertEqual(events, ["dwell", "spindle_orient", "spindle"] * 2, canned)

    def test_fanuc_writes_g76_with_the_declared_shift_direction(self):
        j = job()
        nc = posted_text(j, *fanuc(j, 0))
        bores = [line for line in nc if line.startswith("G76")]
        self.assertEqual(len(bores), 2, nc)
        self.assertTrue(bores[0].startswith("G76 X8. Y8. Z4. R11. Q"), bores)
        self.assertIn("P250", bores[0])
        # Edge along +Y in the operation, +X on the machine: refused.
        with self.assertRaisesRegex(Exception, "edge_angle_deg 90"):
            posted_text(job(edge=90), *fanuc(j, 0))
        # Without a declared orientation: no G76, and no M19 to expand it.
        j = job(dwell_s=None)
        with self.assertRaisesRegex(Exception, "no oriented spindle stop"):
            posted_text(j, *fanuc(j, None))

    def test_m19_and_explicit_shifts_on_centroid_and_linuxcnc(self):
        j = job(edge=90, canned_cycle=False)
        centroid = example("centroid_post")
        # CNC12 M6 moves Z home: the table states each tool's tip pose after M6.
        m = centroid.define_machine(
            "Acorn",
            table(j),
            traverse_feed_mm_min=3000,
            work_to_machine=WorkFrame(),
            decimals=3,
            oriented_edge_deg=90,
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
        i = nc.index("M19")
        # Shift toward -Y at depth, out, back over the hole, spindle on.
        self.assertEqual(
            nc[i + 1 : i + 5], ["G1 X8 Y7.5 Z4 F3000", "G1 X8 Y7.5 Z11 F3000", "G1 X8 Y8 Z11 F3000", "M3 S1000"]
        )
        self.assertFalse(any(line.startswith("G76") for line in nc))
        dialect = GCodeDialect(oriented_edge_deg=90).with_commands(spindle_orient="M19")
        linuxcnc = Machine(
            "mill",
            tool_table(j),
            dialect=dialect,
            tool_change_position_mode="preserve_tip",
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
        nc = posted_text(j, linuxcnc)
        i = nc.index("M19")
        self.assertEqual(nc[i + 1 : i + 4], ["G0 X8 Y7.5 Z4", "G0 X8 Y7.5 Z11", "G0 X8 Y8 Z11"])
        # The same job canned: LinuxCNC has no fine boring cycle (its G76
        # threads), so the moves are explicit there too.
        self.assertIn("M19", posted_text(job(edge=90), linuxcnc))

    def test_refusals(self):
        j = job()
        with self.assertRaisesRegex(Exception, "no oriented spindle stop"):
            posted_text(
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
        with self.assertRaisesRegex(Exception, "oriented spindle stop"):
            j.post_nc(
                "top",
                machine=Machine(
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
                accept={"unchecked_machine"},
            )
        with self.assertRaisesRegex(Exception, "at most"):
            with job(fine_bore=camlib.FineBore(shift_mm=2.5, edge_angle_deg=0)).regenerate() as run:
                list(run)
        with self.assertRaisesRegex(ValueError, "oriented_edge_deg"):
            GCodeDialect().with_commands(spindle_orient="M19")


if __name__ == "__main__":
    unittest.main()
