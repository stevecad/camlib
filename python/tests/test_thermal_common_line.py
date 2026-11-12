"""Common-line cutting through the Python API: two parts one kerf path apart
share one cut, checked on the stream (every move clear), in the report and
in the posted program (decoded back by the post)."""

import unittest

import camlib
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.post import GCodeDialect, Machine, ToolSetting
from camlib.simulation import OperationUpdate, SimulationError, VerificationUpdate

THICKNESS = 6
KERF = 1.5


def rect(x0, y0, x1, y1):
    return Component(Boundary.polygon(((x0, y0), (x1, y0), (x1, y1), (x0, y1))))


def cut(gap=None, count=2, **changes):
    tool = camlib.Torch("torch", kerf_width=KERF)
    values = dict(
        height_mode="program",
        pierce_height_mm=3.8,
        travel_height_mm=10,
        plunge_mm_min=1000,
        pierce_delay_s=0.5,
        power_percent=100,
        lead_in=camlib.Lead(3, 3),
        lead_out=camlib.Lead(1, 2),
        height_control=True,
        common_line=camlib.CommonLine(min_length_mm=5),
        tool_number=1,
        length_register=1,
    )
    values.update(changes)
    op = camlib.ThermalCut("plasma", tool, PlanarRegion("parts", (rect(20, 20, 60, 50),), 0), 2500, 1.5, **values)
    gap = op.common_line_spacing_mm() if gap is None else gap
    parts = tuple(rect(20 + k * (40 + gap), 20, 60 + k * (40 + gap), 50) for k in range(count))
    from dataclasses import replace

    return replace(op, parts=PlanarRegion("parts", parts, 0))


def job(op):
    setup = camlib.Setup(
        "sheet",
        stock=camlib.StockSpec((0, 0, 0), (200, 120, THICKNESS), 0.25),
        tools=(op.tool,),
        operations=(camlib.OperationNode(op, sources=()),),
        initial_position_mm=(0, 0, 40),
        verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
    )
    return camlib.Job("plasma", "1", (setup,))


def run(op):
    statuses, events, report = set(), [], None
    with job(op).regenerate() as stream:
        for u in stream:
            v = u.update
            if isinstance(v, VerificationUpdate):
                statuses.add(v.status)
            elif isinstance(v, OperationUpdate):
                if v.stage == "events":
                    events += v.events
                if v.stage == "end":
                    report = v.report
    return statuses, events, report


class CommonLineTest(unittest.TestCase):
    def test_spacing_is_twice_the_applied_offset(self):
        op = cut()
        self.assertAlmostEqual(op.common_line_spacing_mm(), KERF + 0.01, places=12)
        self.assertEqual(op.to_dict()["common_line"], {"min_length_mm": 5})
        # Off by default, and then absent from the operation's data.
        self.assertNotIn("common_line", cut(common_line=None).to_dict())

    def test_a_pair_is_cut_with_one_shared_line(self):
        op = cut()
        statuses, events, report = run(op)
        self.assertEqual(statuses, {"clear"})
        self.assertAlmostEqual(report["applied_offset_mm"] * 2, op.common_line_spacing_mm(), places=12)
        self.assertEqual(len(report["common_lines"]), 1)
        line = report["common_lines"][0]
        self.assertEqual(line["parts"], [0, 1])
        self.assertAlmostEqual(line["shared_length_mm"], 30)
        # The cut lies midway between the parts, between the outline's
        # bottom and top.
        x = 60 + op.common_line_spacing_mm() / 2
        self.assertAlmostEqual(line["start_mm"][0], x)
        self.assertAlmostEqual(line["end_mm"][0], x)
        self.assertAlmostEqual(abs(line["end_mm"][1] - line["start_mm"][1]), 30 + op.common_line_spacing_mm())
        # Two pierces: the common line, then the outline of the pair.
        self.assertEqual((report["loops"], report["pierces"]), (1, 2))
        self.assertEqual(report["order"], [[0, None]])
        self.assertEqual([e["kind"] for e in events].count("beam"), 4)

    def test_the_posted_program_has_two_pierces_and_decodes(self):
        op = cut(count=3)
        j = job(op)
        header = j.compile().setup("sheet").program_header()
        table = tuple(
            ToolSetting.for_tool(t, number=1, length_register=1, position_after_mm=header.initial_position_mm)
            for t in header.tools
        )
        dialect = GCodeDialect(beam_s_max=1, height_control_word="P2").with_commands(
            height_control_on="M63", height_control_off="M62"
        )
        machine = Machine(
            "plasma",
            table,
            dialect=dialect,
            tool_change_position_mode="preserve_tip",
            output_units="mm",
            decimals=3,
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            planes=("xy", "xz", "yz"),
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
        nc = j.post_nc(
            "sheet", machine=machine, lowering=camlib.LoweringOptions(native_arcs=True), accept={"unchecked_machine"}
        ).text.splitlines()
        # Three parts in a row: two common lines and one outline.
        self.assertEqual(nc.count("M3 S1"), 3)
        self.assertEqual(nc.count("M63 P2"), 3)

    def test_parts_not_exactly_a_kerf_path_apart_are_refused(self):
        with self.assertRaisesRegex(SimulationError, r"common_line: no two parts are exactly 1\.51 mm apart"):
            run(cut(gap=KERF))
        with self.assertRaisesRegex(SimulationError, "compensation"):
            run(cut(compensation="wear"))
        # Without the option the same placement is refused as before.
        with self.assertRaisesRegex(SimulationError, "topology"):
            run(cut(common_line=None))


if __name__ == "__main__":
    unittest.main()
