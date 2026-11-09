"""Plasma/laser profile cutting through the Python API: planning with
checks, posting (generic G-code with M3 S/M5 and THC words, GRBL laser M4)."""

from pathlib import Path
import sys
import unittest

import camlib
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.post import GCodeDialect, Machine, ToolSetting, post_events
from camlib.posted import OperationEvents
from camlib.simulation import OperationUpdate, Simulation, VerificationUpdate

THICKNESS = 6


def circle(c, r):
    return Boundary(((c[0] + r, c[1], 1), (c[0] - r, c[1], 1)))


def parts():
    plate = Component(
        Boundary.polygon(((20, 20), (120, 20), (120, 100), (20, 100))),
        (circle((60, 60), 25), circle((100, 40), 6), circle((100, 80), 3)),
    )
    washer = Component(circle((60, 60), 15), (circle((60, 60), 5),))
    return PlanarRegion("parts", (plate, washer), 0)


def job(tool=None, **cut):
    tool = tool or camlib.Torch("torch", kerf_width=1.5)
    values = dict(
        height_mode="program",
        pierce_height_mm=3.8,
        travel_height_mm=10,
        plunge_mm_min=1000,
        pierce_delay_s=0.5,
        lead_in=camlib.Lead(2, 3),
        lead_out=camlib.Lead(1, 2),
        overcut_mm=-0.5,
        height_control=True,
        small_holes=camlib.SmallHoles(feed_percent=60),
    )
    values.update(cut)
    op = camlib.ThermalCut(
        "plasma",
        tool,
        parts(),
        2500,
        values.pop("cut_height_mm", 1.5),
        **{'pierce_delay_s': 0, 'power_percent': 100, 'tool_number': 1, 'length_register': 1, **values},
    )
    setup = camlib.Setup(
        "sheet",
        stock=camlib.StockSpec((0, 0, 0), (200, 120, THICKNESS), 0.25),
        tools=(tool,),
        operations=(camlib.OperationNode(op, sources=()),),
        initial_position_mm=(0, 0, 40),
        verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
    )
    return camlib.Job("plasma", "1", (setup,))


def table(j):
    header = j.compile().setup("sheet").program_header()
    return tuple(
        ToolSetting.for_tool(t, number=1, length_register=1, position_after_mm=header.initial_position_mm)
        for t in header.tools
    )


class ThermalCutTest(unittest.TestCase):
    def test_setup_limits_reach_thermal_planning(self):
        from dataclasses import replace
        from camlib.simulation import SimulationError

        original = job()
        area = PlanarRegion("limit", (Component(Boundary.polygon(((-5, -5), (205, -5), (205, 125), (-5, 125)))),), 0)
        clear = replace(original, setups=(replace(original.setups[0], contain=area),))
        with clear.regenerate() as run:
            list(run)
        blocked = replace(original, setups=(replace(original.setups[0], avoid=(camlib.AvoidArea(area, 1),)),))
        with self.assertRaisesRegex(SimulationError, "thermal cut.*avoid area"):
            with blocked.regenerate() as run:
                list(run)

    def test_every_move_is_clear_and_events_follow_each_loop(self):
        statuses, events, report = set(), [], None
        with job().regenerate() as run:
            for u in run:
                v = u.update
                if isinstance(v, VerificationUpdate):
                    statuses.add(v.status)
                elif isinstance(v, OperationUpdate):
                    if v.stage == "events":
                        events += [e["kind"] for e in v.events]
                    if v.stage == "end":
                        report = v.report
        self.assertEqual(statuses, {"clear"})
        self.assertEqual(events.count("beam"), 12)  # on and off, six loops
        self.assertEqual(events.count("height_control"), 10)  # not in the small hole
        self.assertEqual(report["order"][0], [1, 0])  # the washer's hole first
        self.assertEqual(report["order"][-1], [0, None])  # the plate outline last
        self.assertEqual(report["small_holes"], 1)

    def test_generic_post_writes_torch_and_thc_words(self):
        j = job()
        dialect = GCodeDialect(beam_s_max=1, height_control_word="P2").with_commands(
            height_control_on="M63", height_control_off="M62"
        )
        m = Machine(
            "plasma",
            table(j),
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
        nc = j.post_nc(
            "sheet", machine=m, lowering=camlib.LoweringOptions(native_arcs=True), accept={"unchecked_machine"}
        ).text.splitlines()
        self.assertEqual(nc.count("M3 S1"), 6)
        self.assertEqual(nc.count("M63 P2"), 5)
        self.assertEqual(nc.count("M62 P2"), 5)
        self.assertGreaterEqual(nc.count("M5"), 6)
        first = nc.index("M3 S1")
        self.assertEqual(nc[first + 1], "G4 P0.5")  # pierce delay
        # Without a declared beam the post refuses.
        with self.assertRaisesRegex(Exception, "declares no beam"):
            j.post_nc(
                "sheet",
                machine=Machine(
                    "mill",
                    table(j),
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
                lowering=camlib.LoweringOptions(native_arcs=True),
                accept={"unchecked_machine"},
            )

    def test_qtplasmac_example(self):
        from test_drill_cycles import example

        q = example("qtplasmac_post")
        tool = camlib.Torch("torch", kerf_width=1.5)
        j = job(
            tool,
            height_mode="fixed",
            pierce_height_mm=None,
            travel_height_mm=None,
            plunge_mm_min=None,
            pierce_delay_s=0,
        )
        header = j.compile().setup("sheet").program_header()
        setting = ToolSetting.for_tool(tool, number=1, length_register=1, position_after_mm=header.initial_position_mm)
        with self.assertRaisesRegex(ValueError, "one torch"):
            q.define_machine(
                "table",
                ToolSetting.for_tool(tool, number=2, length_register=1, position_after_mm=header.initial_position_mm),
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
                rapid_mm_min=6000,
            )
        m = q.define_machine(
            "table",
            setting,
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
            rapid_mm_min=6000,
        )
        nc = j.post_nc(
            "sheet",
            machine=m,
            post=q.QtPlasmaCPost,
            decoder=q.QtPlasmaCDecoder,
            lowering=camlib.LoweringOptions(native_arcs=True),
            accept={"unchecked_machine"},
        ).text.splitlines()
        for line in ("M52 P1", "G64 P0.005", "T0 M6", "G43 H0"):
            self.assertIn(line, nc)
        self.assertEqual(nc.count("M3 $0 S1"), 6)
        self.assertGreaterEqual(nc.count("M5 $0"), 6)
        self.assertEqual(nc.count("M63 P2"), 5)

    def test_centroid_plasma_example(self):
        from test_drill_cycles import example

        c = example("centroid_plasma_post")
        tool = camlib.Torch("torch", kerf_width=1.5)
        fixed = dict(
            height_mode="fixed",
            pierce_height_mm=None,
            travel_height_mm=None,
            plunge_mm_min=None,
            pierce_delay_s=0,
            height_control=False,
        )
        j = job(tool, **fixed)
        # The torch starts at the cut height: no Z word anywhere.
        setup = j.setups[0]
        from dataclasses import replace

        j = camlib.Job("plasma", "1", (replace(setup, initial_position_mm=(0, 0, THICKNESS + 1.5)),))
        header = j.compile().setup("sheet").program_header()
        m = c.define_machine(
            "table",
            ToolSetting.for_tool(tool, number=1, length_register=1, position_after_mm=header.initial_position_mm),
            output_units='mm',
            decimals=3,
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            arcs=True,
            helices=True,
            max_sweep_rad=6.283185307179586,
            max_tool_number=99999,
            max_length_register=99999,
            tool_changes=True,
            rapid_mm_min=6000,
        )
        nc = j.post_nc(
            "sheet",
            machine=m,
            post=c.CentroidPlasmaPost,
            decoder=c.CentroidPlasmaDecoder,
            lowering=camlib.LoweringOptions(native_arcs=True),
            accept={"unchecked_machine"},
        ).text.splitlines()
        self.assertEqual(nc.count("M65"), 1)
        self.assertEqual(nc.count("M61"), 6)
        self.assertEqual(nc.count("M62"), 6)
        self.assertEqual(nc[-2:], ["M62", "M30"])
        moves = [line for line in nc if line.split(" ")[0] in ("G0", "G1", "G2", "G3")]
        self.assertFalse(any(w.startswith("Z") for line in moves for w in line.split(" ")[1:]), moves[:5])
        # The profile owns the pierce delay and THC.
        with self.assertRaisesRegex(Exception, "pierce_delay_s=0"):
            j2 = camlib.Job(
                "plasma",
                "1",
                (
                    replace(
                        job(tool, **dict(fixed, pierce_delay_s=0.5)).setups[0],
                        initial_position_mm=(0, 0, THICKNESS + 1.5),
                    ),
                ),
            )
            j2.post_nc(
                "sheet",
                machine=m,
                post=c.CentroidPlasmaPost,
                decoder=c.CentroidPlasmaDecoder,
                lowering=camlib.LoweringOptions(native_arcs=True),
                accept={"unchecked_machine"},
            )

    def test_wincnc_plasma_example(self):
        from test_drill_cycles import example
        from dataclasses import replace

        w = example("wincnc_plasma_post")
        tool = camlib.Torch("torch", kerf_width=1.5)
        fixed = dict(
            height_mode="fixed",
            pierce_height_mm=None,
            travel_height_mm=None,
            plunge_mm_min=None,
            pierce_delay_s=0,
            height_control=False,
        )
        j = job(tool, **fixed)
        j = camlib.Job("plasma", "1", (replace(j.setups[0], initial_position_mm=(0, 0, THICKNESS + 1.5)),))
        header = j.compile().setup("sheet").program_header()
        m = w.define_machine(
            "table",
            ToolSetting.for_tool(tool, number=1, length_register=1, position_after_mm=header.initial_position_mm),
            output_units='mm',
            decimals=3,
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            arcs=True,
            helices=True,
            max_tool_number=99999,
            max_length_register=99999,
            tool_changes=True,
            rapid_mm_min=6000,
        )
        nc = j.post_nc(
            "sheet",
            machine=m,
            post=w.WinCNCPlasmaPost,
            decoder=w.WinCNCPlasmaDecoder,
            lowering=camlib.LoweringOptions(native_arcs=True),
            accept={"unchecked_machine"},
        ).text.splitlines()
        self.assertEqual(nc[:2], ["G90", "G22"])  # G21 would be centimetres on WinCNC
        self.assertEqual(nc.count("G92"), 1)  # once, before any motion
        self.assertLess(nc.index("G92"), nc.index("M65"))
        self.assertEqual(nc.count("M65"), 1)
        self.assertEqual(nc.count("M61"), 6)
        self.assertEqual(nc.count("M62"), 6)
        self.assertEqual(nc[-2:], ["M62", "M66"])
        moves = [line for line in nc if line.split(" ")[0] in ("G0", "G1", "G2", "G3")]
        self.assertFalse(any(w_.startswith("Z") for line in moves for w_ in line.split(" ")[1:]))
        self.assertNotIn("G17", nc)
        self.assertNotIn("G54", nc)
        # The machine's own macro codes.
        m2 = w.define_machine(
            "table",
            ToolSetting.for_tool(tool, number=1, length_register=1, position_after_mm=header.initial_position_mm),
            torch_on_code="M71",
            torch_off_code="M72",
            output_units='mm',
            decimals=3,
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            arcs=True,
            helices=True,
            max_tool_number=99999,
            max_length_register=99999,
            tool_changes=True,
            rapid_mm_min=6000,
        )
        nc2 = j.post_nc(
            "sheet",
            machine=m2,
            post=w.WinCNCPlasmaPost,
            decoder=w.WinCNCPlasmaDecoder,
            lowering=camlib.LoweringOptions(native_arcs=True),
            accept={"unchecked_machine"},
        ).text.splitlines()
        self.assertEqual(nc2.count("M71"), 6)

    def test_micro_joints(self):
        report = None
        with job(micro_joints=camlib.MicroJoints(1, count=3)).regenerate() as run:
            for u in run:
                if isinstance(u.update, OperationUpdate) and u.update.stage == "end":
                    report = u.update.report
        self.assertEqual(report["micro_joints"], 6)

    def test_grbl_laser_m4_power(self):
        laser = camlib.LaserHead("laser", kerf_width=0.2)
        j = job(
            laser,
            height_mode="fixed",
            pierce_height_mm=None,
            travel_height_mm=None,
            plunge_mm_min=None,
            pierce_delay_s=0,
            power_percent=80,
            height_control=False,
            small_holes=None,
            lead_in=camlib.Lead(1, 1),
            lead_out=camlib.Lead(0.5, 0.5),
            cut_height_mm=1,
        )
        setup = j.compile().setup("sheet")
        header = setup.program_header()
        examples = str(Path(__file__).resolve().parents[2] / "examples")
        sys.path.insert(0, examples)
        try:
            from grbl_post import GrblDecoder, GrblPost, define_machine
        finally:
            sys.path.remove(examples)
        machine = define_machine(
            "GRBL laser",
            ToolSetting.for_tool(
                header.tools[0], number=1, length_register=1, position_after_mm=header.initial_position_mm
            ),
            work_offset_machine_mm=(0, 0, 0),
            initial_machine_mm=header.initial_position_mm,
            beam_s_max=1000,
            output_units="mm",
            decimals=6,
            max_feed_mm_min=10000,
            max_spindle_rpm=1000,
            rapid_mm_min=10000,
            flood=False,
            mist=False,
        )
        simulation = Simulation(**setup.simulation_options(None))
        source = OperationEvents(simulation, reuse_active_tool=True)
        try:
            chunks = post_events(
                header,
                source,
                machine=machine,
                post=GrblPost,
                decoder=GrblDecoder,
                lowering=camlib.LoweringOptions(chord_tolerance_mm=0.001),
            )
            text = "".join(chunk.text for chunk in chunks)
        finally:
            source.close()
        lines = text.splitlines()
        self.assertEqual(lines.count("M4 S800"), 6)  # laser mode: M4 at 80 % of $30 = 1000
        self.assertGreaterEqual(lines.count("M5"), 6)
        # Fixed height: one Z (the cut height) from the first move on.
        moves = [line for line in lines if line.startswith(("G0 ", "G1 "))]
        self.assertEqual({w for line in moves for w in line.split() if w[0] == "Z"}, {"Z7"})


def polygon(center, radius, n, start_deg=0):
    import math

    return tuple(
        (
            center[0] + radius * math.cos(math.radians(start_deg) + 2 * math.pi * k / n),
            center[1] + radius * math.sin(math.radians(start_deg) + 2 * math.pi * k / n),
        )
        for k in range(n)
    )


def inside_polygon(p, vertices):
    inside = False
    for (x0, y0), (x1, y1) in zip(vertices, vertices[1:] + vertices[:1]):
        if (y0 > p[1]) != (y1 > p[1]) and p[0] < x0 + (p[1] - y0) * (x1 - x0) / (y1 - y0):
            inside = not inside
    return inside


class StraightEdgedHolesTest(unittest.TestCase):
    """Holes with straight edges get their leads and pierce in the hole
    (issue #78): rectangles, a slot, polygons, small holes near the lead size
    and arcs, all in one plate."""

    def test_every_hole_is_pierced_inside_and_the_post_decodes(self):
        import math

        rect = ((10, 10), (50, 10), (50, 30), (10, 30))
        square = ((42, 62), (49, 62), (49, 69), (42, 69))  # 7 mm: near the leads
        triangle = ((80, 60), (100, 60), (90, 80))
        hexagon = polygon((20, 70), 4.5, 6)  # 9 mm across corners
        gon64 = polygon((130, 30), 10, 64)
        holes = {
            "rect": (Boundary.polygon(rect), lambda p: inside_polygon(p, rect)),
            "slot": (
                Boundary(((70, 14, 0), (100, 14, 1), (100, 26, 0), (70, 26, 1))),
                lambda p: (
                    70 <= p[0] <= 100 and 14 < p[1] < 26 or min(math.dist(p, (70, 20)), math.dist(p, (100, 20))) < 6
                ),
            ),
            "square": (Boundary.polygon(square), lambda p: inside_polygon(p, square)),
            "triangle": (Boundary.polygon(triangle), lambda p: inside_polygon(p, triangle)),
            "hexagon": (Boundary.polygon(hexagon), lambda p: inside_polygon(p, hexagon)),
            "64-gon": (Boundary.polygon(gon64), lambda p: inside_polygon(p, gon64)),
            "circle": (circle((130, 70), 8), lambda p: math.dist(p, (130, 70)) < 8),
        }
        names = list(holes)
        plate = Component(Boundary.polygon(((0, 0), (160, 0), (160, 100), (0, 100))), tuple(holes[n][0] for n in names))
        torch = camlib.Torch("torch", kerf_width=1.5)
        op = camlib.ThermalCut(
            "plasma",
            torch,
            PlanarRegion("parts", (plate,), 0),
            2500,
            1.5,
            lead_in=camlib.Lead(2, 3),
            lead_out=camlib.Lead(1, 2),
            pierce_delay_s=0,
            power_percent=100,
            tool_number=1,
            length_register=1,
        )
        setup = camlib.Setup(
            "sheet",
            stock=camlib.StockSpec((-20, -20, 0), (180, 120, 3), 0.5),
            tools=(torch,),
            operations=(op,),
            initial_position_mm=(0, 0, 40),
            verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
        )
        j = camlib.Job("holes", "1", (setup,))
        starts, report, statuses, previous = [], None, set(), None
        with j.regenerate() as run:
            for u in run:
                v = u.update
                if isinstance(v, VerificationUpdate):
                    statuses.add(v.status)
                elif isinstance(v, OperationUpdate):
                    if v.stage == "end":
                        report = v.report
                else:
                    # A loop's pierce: where its first cutting move (a lead)
                    # starts after the rapids that brought the torch there.
                    m = v.motion
                    if m["kind"] != "rapid" and previous == "rapid":
                        starts.append(tuple(m["curve"]["start"][:2]))
                    previous = m["kind"]
        self.assertEqual(statuses, {"clear"})
        self.assertEqual(len(starts), len(report["order"]))
        for (part, hole), point in zip(report["order"], starts):
            if hole is not None:
                name = names[hole]
                self.assertTrue(holes[name][1](point), (name, point))
        # The post decodes every block back to the plan.
        machine = Machine(
            "plasma",
            table_for(j),
            dialect=GCodeDialect(beam_s_max=1),
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
        program = j.post_nc(
            "sheet", machine=machine, lowering=camlib.LoweringOptions(native_arcs=True), accept={"unchecked_machine"}
        )
        self.assertEqual(program.text.count("M3 S1"), len(holes) + 1)


def table_for(j):
    header = j.compile().setup("sheet").program_header()
    return tuple(
        ToolSetting.for_tool(t, number=1, length_register=1, position_after_mm=header.initial_position_mm)
        for t in header.tools
    )


if __name__ == "__main__":
    unittest.main()
