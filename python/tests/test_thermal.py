"""Plasma/laser profile cutting through the Python API: planning with
checks, posting (generic G-code with M3 S/M5 and THC words, GRBL laser M4)."""

from pathlib import Path
import sys
import math
import unittest

import camlib
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.post import GCodeDialect, Machine, ToolSetting, post_events
from camlib.posted import OperationEvents
from camlib.simulation import OperationUpdate, Simulation, SimulationError, VerificationUpdate

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
        values.pop("parts", None) or parts(),
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
    def test_edge_start_events_precede_leads_and_unsupported_posts_refuse(self):
        from dataclasses import replace
        from test_drill_cycles import example

        region = PlanarRegion("edge_part", (Component(Boundary.polygon(((10, 12), (36, 12), (36, 38), (10, 38)))),), 0)
        options = camlib.EdgeStarts(run_on_mm=3, max_lead_mm=20)
        j = job(
            parts=region,
            edge_starts=options,
            height_mode="fixed",
            pierce_height_mm=None,
            travel_height_mm=None,
            plunge_mm_min=None,
            pierce_delay_s=0,
            height_control=False,
            small_holes=None,
        )
        j = replace(j, setups=(replace(j.setups[0], initial_position_mm=(0, 0, 7.5)),))
        on, starts, report = False, [], None
        position = (0, 0, 7.5)
        with j.regenerate() as run:
            for item in run:
                update = item.update
                if isinstance(update, OperationUpdate):
                    for event in update.events:
                        if event["kind"] == "beam_start":
                            self.assertFalse(on)
                            self.assertTrue(position[0] < 0 or position[1] < 0)
                            self.assertEqual(position[2], 7.5)
                            starts.append(position)
                            on = True
                        elif event["kind"] == "beam":
                            self.assertEqual(event["power_percent"], 0)
                            on = False
                    if update.stage == "end":
                        report = update.report
                elif isinstance(update, VerificationUpdate):
                    self.assertEqual(update.status, "clear")
                else:
                    motion = update.motion
                    if motion["kind"] != "rapid":
                        self.assertTrue(on, "BeamStart must reach consumers before the first lead")
                    position = motion["curve"]["end"]
        self.assertEqual(len(starts), 1)
        self.assertEqual(report["edge_starts"], 1)
        self.assertEqual(report["pierces"], 0)
        w = example("wincnc_plasma_post")
        machine = w.define_machine(
            "table",
            table(j)[0],
            output_units="mm",
            decimals=3,
            max_feed_mm_min=10000,
            max_spindle_rpm=1,
            arcs=True,
            helices=False,
            max_tool_number=1,
            max_length_register=1,
            tool_changes=True,
            rapid_mm_min=6000,
        )
        with self.assertRaisesRegex(SimulationError, "no start in open material"):
            j.post_nc(
                "sheet",
                machine=machine,
                post=w.WinCNCPlasmaPost,
                decoder=w.WinCNCPlasmaDecoder,
                lowering=camlib.LoweringOptions(native_arcs=True),
            )

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
        dialect = GCodeDialect(beam_s_max=1, height_control_word="P2", dwell_max_s=1e5).with_commands(
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
        # A table whose own programs carry no start macro and run in the
        # operator's local coordinates (ShopSabre with ProNest's post: G90,
        # the cuts between M61 and M62, M66): neither M65 nor G92 is written,
        # a comment states each premise instead, and the program still
        # decodes back to the plan.
        m3 = w.define_machine(
            "table",
            ToolSetting.for_tool(tool, number=1, length_register=1, position_after_mm=header.initial_position_mm),
            start_code=None,
            local_coordinates="keep",
            output_units='inch',
            decimals=4,
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            arcs=True,
            helices=False,
            max_tool_number=1,
            max_length_register=1,
            tool_changes=True,
            rapid_mm_min=6000,
        )
        nc3 = j.post_nc(
            "sheet",
            machine=m3,
            post=w.WinCNCPlasmaPost,
            decoder=w.WinCNCPlasmaDecoder,
            lowering=camlib.LoweringOptions(native_arcs=True),
            accept={"unchecked_machine"},
        ).text.splitlines()
        self.assertEqual(nc3[:6], ["G90", "G20", "G40", "G49", "(WORK ZERO AS SET AT THE MACHINE)", "(TORCH TORCH)"])
        self.assertTrue(nc3[6].startswith("G0 X"), nc3[6])
        self.assertNotIn("G92", nc3)
        self.assertNotIn("M65", nc3)
        self.assertEqual((nc3.count("M61"), nc3.count("M62"), nc3[-1]), (6, 6, "M66"))
        # Every other line is the same program as with the macro and G92.
        m4 = replace(m3, start_code="M65", local_coordinates="cancel")
        nc4 = j.post_nc(
            "sheet",
            machine=m4,
            post=w.WinCNCPlasmaPost,
            decoder=w.WinCNCPlasmaDecoder,
            lowering=camlib.LoweringOptions(native_arcs=True),
            accept={"unchecked_machine"},
        ).text.splitlines()
        self.assertEqual([line for line in nc4 if line not in ("G92", "M65")], [line for line in nc3 if line[0] != "("])
        # Height control from the program (WinCNC's L36 on, L37 off): off
        # before the first torch-on, on over each loop's cut and off again
        # before its lead-out (never on in the small hole), and left on at
        # the end. Without the declaration the command is refused.
        jh = job(tool, **{**fixed, "height_control": True})
        jh = camlib.Job("plasma", "1", (replace(jh.setups[0], initial_position_mm=(0, 0, THICKNESS + 1.5)),))
        nch = jh.post_nc(
            "sheet",
            machine=replace(m3, program_height_control=True),
            post=w.WinCNCPlasmaPost,
            decoder=w.WinCNCPlasmaDecoder,
            lowering=camlib.LoweringOptions(native_arcs=True),
            accept={"unchecked_machine"},
        ).text.splitlines()
        self.assertEqual(nch[nch.index("M61") - 1], "L37")
        self.assertEqual((nch.count("L36"), nch.count("L37")), (5 + 1, 5 + 1))
        self.assertEqual(nch[-3:], ["M62", "L36", "M66"])
        state, torch = None, False
        for line in nch:
            if line in ("M61", "M62"):
                # Never on when the torch fires or goes out.
                self.assertIs(state, False, line)
                torch = line == "M61"
            elif line in ("L36", "L37"):
                state = line == "L36"
                self.assertTrue(torch or line == "L37" or nch[-2] == line)
        self.assertEqual([line for line in nch if line not in ("L36", "L37")], nc3)
        with self.assertRaisesRegex(Exception, "program_height_control=True"):
            jh.post_nc(
                "sheet",
                machine=m3,
                post=w.WinCNCPlasmaPost,
                decoder=w.WinCNCPlasmaDecoder,
                lowering=camlib.LoweringOptions(native_arcs=True),
                accept={"unchecked_machine"},
            )
        # Arcs. A plate whose edges are shallow arcs of a large radius, at a
        # position four decimals of an inch do not state exactly: with
        # exactly consistent words required most of those arcs have none and
        # are posted as checked lines. WinCNC takes arcs within its arc_err
        # setting; declared, every arc stays an arc, and the program is
        # still decoded and certified against the plan.
        bowed = PlanarRegion(
            "bowed",
            (
                Component(
                    Boundary(
                        (
                            (33.337, 27.219, 0.04),
                            (153.337, 27.219, 0.04),
                            (153.337, 97.219, 0.04),
                            (33.337, 97.219, 0.04),
                        )
                    )
                ),
            ),
            0,
        )
        jb = job(tool, parts=bowed, **fixed)
        jb = camlib.Job("plasma", "1", (replace(jb.setups[0], initial_position_mm=(0, 0, THICKNESS + 1.5)),))
        exact, tolerant = (
            jb.post_nc(
                "sheet",
                machine=machine,
                post=w.WinCNCPlasmaPost,
                decoder=w.WinCNCPlasmaDecoder,
                lowering=camlib.LoweringOptions(native_arcs=True),
                accept={"unchecked_machine"},
            ).text.splitlines()
            for machine in (m3, replace(m3, arc_radius_tolerance_mm=0.005, dialect=w._dialect("M66", 0.005)))
        )
        count = lambda lines, codes: sum(line[:2] in codes for line in lines)  # noqa: E731
        self.assertGreater(count(exact, ("G1",)), 20)
        # The four edges and the leads' arcs; the only lines are the leads' own.
        self.assertLessEqual(count(tolerant, ("G1",)), 2)
        self.assertGreaterEqual(count(tolerant, ("G2", "G3")), 4)
        self.assertLess(len(tolerant), len(exact))
        with self.assertRaisesRegex(ValueError, "radius tolerance"):
            w._dialect("M66", 0.05)
        # The feed from the control's material setup: declared, no F word is
        # written, the program still decodes to the plan (the decoder cuts at
        # the declared feed), and it is the F-word program less its F words.
        # A cut planned at another feed anywhere (here the small hole's 60 %)
        # is refused: the program could not say it.
        material = replace(m3, material_feed_mm_min=2500)
        js = job(tool, **{**fixed, "small_holes": None})
        js = camlib.Job("plasma", "1", (replace(js.setups[0], initial_position_mm=(0, 0, THICKNESS + 1.5)),))
        with_f, without_f = (
            js.post_nc(
                "sheet",
                machine=machine,
                post=w.WinCNCPlasmaPost,
                decoder=w.WinCNCPlasmaDecoder,
                lowering=camlib.LoweringOptions(native_arcs=True),
                accept={"unchecked_machine"},
            ).text.splitlines()
            for machine in (m3, material)
        )
        self.assertTrue(any(" F" in line for line in with_f))
        self.assertFalse(any("F" in line for line in without_f if line[0] != "("), without_f[:12])
        self.assertEqual([" ".join(word for word in line.split(" ") if word[0] != "F") for line in with_f], without_f)
        with self.assertRaisesRegex(Exception, "planned at 1500 mm/min.*material_feed_mm_min"):
            j.post_nc(
                "sheet",
                machine=material,
                post=w.WinCNCPlasmaPost,
                decoder=w.WinCNCPlasmaDecoder,
                lowering=camlib.LoweringOptions(native_arcs=True),
                accept={"unchecked_machine"},
            )
        with self.assertRaisesRegex(ValueError, "material_feed_mm_min"):
            replace(m3, material_feed_mm_min=0)
        with self.assertRaisesRegex(ValueError, "local_coordinates"):
            replace(m3, local_coordinates="machine")
        with self.assertRaisesRegex(ValueError, "distinct M macro codes"):
            replace(m3, torch_off_code="M61")

    def test_micro_joints(self):
        report = None
        with job(micro_joints=camlib.MicroJoints(1, count=3)).regenerate() as run:
            for u in run:
                if isinstance(u.update, OperationUpdate) and u.update.stage == "end":
                    report = u.update.report
        self.assertEqual(report["micro_joints"], 6)

    def test_the_part_order_is_declared_and_reported(self):
        def report_of(**cut):
            with job(**cut).regenerate() as run:
                for u in run:
                    if isinstance(u.update, OperationUpdate) and u.update.stage == "end":
                        return u.update.report

        nearest = report_of()
        self.assertEqual(nearest["part_order"], {"kind": "nearest"})
        self.assertGreater(nearest["rapid_length_mm"], 0)
        # The washer lies in the plate's hole: it is cut first in every order.
        for order in ("given", camlib.SpreadHeat(min_distance_mm=50)):
            report = report_of(part_order=order)
            outlines = [part for part, hole in report["order"] if hole is None]
            self.assertEqual(outlines, [1, 0])
        self.assertEqual(report["part_order"], {"kind": "spread", "min_distance_mm": 50})
        with self.assertRaisesRegex(ValueError, "part_order 'random'.*SpreadHeat"):
            report_of(part_order="random")
        with self.assertRaises(TypeError):
            camlib.SpreadHeat()  # no default distance

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


def wincnc_edge_start_job(units="mm", material_feed=False, from_kerfs=False, delay_s=0.125):
    from dataclasses import replace
    from test_drill_cycles import example

    w = example("wincnc_plasma_post")
    region = PlanarRegion(
        "mixed_starts",
        tuple(Component(Boundary.polygon(((80, y), (106, y), (106, y + 26), (80, y + 26)))) for y in (12, 48)),
        0,
    )
    j = job(
        parts=region,
        edge_starts=camlib.EdgeStarts(run_on_mm=3, max_lead_mm=20, from_kerfs=from_kerfs, delay_s=delay_s),
        height_mode="fixed",
        pierce_height_mm=None,
        travel_height_mm=None,
        plunge_mm_min=None,
        pierce_delay_s=0,
        height_control=True,
        small_holes=None,
    )
    j = replace(j, setups=(replace(j.setups[0], initial_position_mm=(0, 0, 7.5)),))
    machine = w.define_machine(
        "synthetic edge-start table",
        table(j)[0],
        output_units=units,
        decimals=4 if units == "inch" else 3,
        max_feed_mm_min=10000,
        max_spindle_rpm=1,
        arcs=True,
        helices=False,
        max_tool_number=1,
        max_length_register=1,
        tool_changes=True,
        rapid_mm_min=6000,
        start_code=None,
        local_coordinates="keep",
        program_height_control=True,
        edge_start_cut_z_mm=7.5,
        material_feed_mm_min=2500 if material_feed else None,
        dwell_max_s=1e5,
    )
    return w, j, machine


class WinCNCEdgeStartsTest(unittest.TestCase):
    setup_edge_job = staticmethod(wincnc_edge_start_job)

    def post(self, w, j, machine, post=None):
        return j.post_nc(
            "sheet",
            machine=machine,
            post=post or w.WinCNCPlasmaPost,
            decoder=w.WinCNCPlasmaDecoder,
            lowering=camlib.LoweringOptions(native_arcs=True),
            accept={"unchecked_machine"},
        )

    def test_mixed_pierces_and_edge_starts_round_trip_in_both_units(self):
        for units in ("mm", "inch"):
            for material_feed in (False, True):
                with self.subTest(units=units, material_feed=material_feed):
                    w, j, m = self.setup_edge_job(units, material_feed)
                    lines = self.post(w, j, m).text.splitlines()
                    self.assertEqual(lines.count('~L38'), 1)
                    self.assertEqual(lines.count('M61'), 1)
                    self.assertEqual(lines.count('M62'), 2)
                    start = lines.index('~L38')
                    self.assertEqual(lines[start - 2 : start], ['L37', 'L90 G0 Z{THR}'])
                    self.assertEqual(lines.count('G4 D0.125'), 1)
                    self.assertEqual(lines.count('F{THF}'), int(material_feed))
                    self.assertNotIn('M63', lines)
                    self.assertFalse(any('THS' in line or 'THP' in line or 'THZ' in line for line in lines))
                    self.assertNotIn('G92', lines)

    def test_decoder_rejects_missing_or_substituted_edge_start_steps(self):
        w, j, m = self.setup_edge_job(material_feed=True)
        mutations = (
            ('L90 G0 Z{THR}', None, 'cut-height positioning'),
            ('~L38', 'M61', 'native ~L38'),
            ('F{THF}', None, 'material feed'),
            ('L90 G0 Z{THR}', 'L36', 'cut-height positioning'),
            ('L37', 'L36', 'disable THC'),
            ('G4 D0.125', 'G4 D0.25', 'decoded effects differ from source'),
        )
        for before, after, message in mutations:
            with self.subTest(before=before, after=after):

                class BrokenPost(w.WinCNCPlasmaPost):
                    def emit(self, *words):
                        if words == (before,):
                            if after is not None:
                                super().emit(after)
                        else:
                            super().emit(*words)

                with self.assertRaisesRegex((ValueError, SimulationError), message):
                    self.post(w, j, m, BrokenPost)

    def test_starts_in_earlier_kerfs_and_zero_delay(self):
        w, j, m = self.setup_edge_job(from_kerfs=True, delay_s=0)
        lines = self.post(w, j, m).text.splitlines()
        self.assertEqual(lines.count('~L38'), 2)
        self.assertNotIn('M61', lines)
        self.assertEqual(lines.count('M62'), 2)
        self.assertFalse(any(line.startswith('G4 ') for line in lines))

    def test_edge_start_after_a_pierce_and_at_the_initial_position(self):
        from dataclasses import replace

        w, j, m = self.setup_edge_job()

        def at(position):
            moved_job = replace(j, setups=(replace(j.setups[0], initial_position_mm=position),))
            moved_machine = replace(m, tools=(replace(m.tools[0], position_after_mm=position),))
            return self.post(w, moved_job, moved_machine).text.splitlines()

        # Starting near the upper square makes the ordinary pierce occur
        # first. Its shutdown must not prevent the following native start.
        lines = at((100, 100, 7.5))
        self.assertLess(lines.index('M61'), lines.index('~L38'))

        # Determine the planned start from streamed motion, independently
        # of the post, then begin there: no XY positioning move is needed.
        position = (0, 0, 7.5)
        with j.regenerate() as run:
            for item in run:
                update = item.update
                if isinstance(update, OperationUpdate):
                    if any(e['kind'] == 'beam_start' for e in update.events):
                        break
                elif not isinstance(update, VerificationUpdate):
                    position = tuple(update.motion['curve']['end'])
        lines = at(position)
        first_xy = next(i for i, line in enumerate(lines) if line.startswith(('G0 ', 'G1 ', 'G2 ', 'G3 ')))
        self.assertLess(lines.index('~L38'), first_xy)

    def test_edge_start_requires_declared_matching_height_and_thc_control(self):
        from dataclasses import replace

        w, j, m = self.setup_edge_job()
        with self.assertRaisesRegex(ValueError, 'program_height_control'):
            replace(m, program_height_control=False)
        with self.assertRaisesRegex((ValueError, SimulationError), 'planned Z differs'):
            self.post(w, j, replace(m, edge_start_cut_z_mm=8))
        with self.assertRaisesRegex((ValueError, SimulationError), 'no start in open material'):
            self.post(w, j, replace(m, edge_start_cut_z_mm=None))


class BeamPostsInInchesTest(unittest.TestCase):
    """Every beam post on a controller that runs in inches: the same cuts are
    posted in inches (G20) at four decimals and certified against the plan
    like any program. Rings at positions that are not round in inches: their
    half-turn arcs, rounded to inch words, must still be arcs the machine
    accepts (a half turn at most on WinCNC) or checked lines."""

    FIXED = dict(
        height_mode="fixed",
        pierce_height_mm=None,
        travel_height_mm=None,
        plunge_mm_min=None,
        pierce_delay_s=0,
    )
    LIMITS = dict(
        max_feed_mm_min=10000,
        max_spindle_rpm=30000,
        arcs=True,
        max_tool_number=1,
        max_length_register=1,
        tool_changes=True,
        rapid_mm_min=6000,
    )

    def rings(self, tool, k, **cut):
        from dataclasses import replace

        cx, cy = 60 + 3.137 * k, 55 + 1.713 * k
        parts = PlanarRegion("rings", (Component(circle((cx, cy), 24), (circle((cx, cy), 10),)),), 0)
        j = job(tool, **{**self.FIXED, **cut})
        setup = j.setups[0]
        op = replace(setup.operations[0].operation, parts=parts)
        setup = replace(
            setup, operations=(camlib.OperationNode(op, sources=()),), initial_position_mm=(0, 0, THICKNESS + 1.5)
        )
        return camlib.Job("plasma", "1", (setup,))

    def posted(self, module, post, decoder, tool, machine, **cut):
        from test_drill_cycles import example

        m = example(module)
        arcs = 0
        for k in range(12):
            j = self.rings(tool, k, **cut)
            header = j.compile().setup("sheet").program_header()
            setting = ToolSetting.for_tool(
                tool, number=1, length_register=1, position_after_mm=header.initial_position_mm
            )
            lines = j.post_nc(
                "sheet",
                machine=m.define_machine("table", setting, output_units="inch", decimals=4, **self.LIMITS, **machine),
                post=getattr(m, post),
                decoder=getattr(m, decoder),
                lowering=camlib.LoweringOptions(native_arcs=True),
                accept={"unchecked_machine"},
            ).text.splitlines()
            self.assertIn("G20", lines[:6], (module, k))
            words = [w for line in lines for w in line.split() if w[0] == "X" and line[:2] in ("G0", "G1", "G2", "G3")]
            xs = [float(w[1:]) for w in words]
            # The rings and their leads span about 30..125 mm in x: 1.2..4.9 in. Nothing is posted in millimetres.
            self.assertTrue(xs and 1.0 < min(xs) and max(xs) < 5.2, (module, k, min(xs), max(xs)))
            arcs += sum(line[:2] in ("G2", "G3") for line in lines)
        return arcs

    def test_qtplasmac_in_inches(self):
        arcs = self.posted(
            "qtplasmac_post",
            "QtPlasmaCPost",
            "QtPlasmaCDecoder",
            camlib.Torch("torch", kerf_width=1.5),
            dict(planes=("xy",), helices=False, max_sweep_rad=math.tau),
        )
        self.assertGreater(arcs, 0)

    def test_centroid_plasma_in_inches(self):
        arcs = self.posted(
            "centroid_plasma_post",
            "CentroidPlasmaPost",
            "CentroidPlasmaDecoder",
            camlib.Torch("torch", kerf_width=1.5),
            dict(helices=False, max_sweep_rad=math.tau),
            height_control=False,
        )
        self.assertGreater(arcs, 0)

    def test_wincnc_in_inches(self):
        # WinCNC takes at most half a turn per arc: a ring's half turns are
        # written as arcs only where their inch words do not span more.
        arcs = self.posted(
            "wincnc_plasma_post",
            "WinCNCPlasmaPost",
            "WinCNCPlasmaDecoder",
            camlib.Torch("torch", kerf_width=1.5),
            dict(helices=False),
            height_control=False,
        )
        self.assertGreater(arcs, 0)

    def test_a_crop_cut_is_the_end_of_every_posted_program(self):
        # The crop cut (#296 D3) through each plasma post, in inches: the
        # program's last cutting block is the crop line, decoded and
        # certified against the plan like the rest.
        from test_drill_cycles import example

        tool = camlib.Torch("torch", kerf_width=1.5)
        crop = camlib.CropCut(((150, 0), (150, 120)))
        for module, post, decoder, machine, cut in (
            ("qtplasmac_post", "QtPlasmaCPost", "QtPlasmaCDecoder", dict(planes=("xy",), max_sweep_rad=math.tau), {}),
            (
                "centroid_plasma_post",
                "CentroidPlasmaPost",
                "CentroidPlasmaDecoder",
                dict(max_sweep_rad=math.tau),
                dict(height_control=False),
            ),
            ("wincnc_plasma_post", "WinCNCPlasmaPost", "WinCNCPlasmaDecoder", {}, dict(height_control=False)),
        ):
            m = example(module)
            j = self.rings(tool, 0, crop=crop, **cut)
            header = j.compile().setup("sheet").program_header()
            setting = ToolSetting.for_tool(
                tool, number=1, length_register=1, position_after_mm=header.initial_position_mm
            )
            lines = j.post_nc(
                "sheet",
                machine=m.define_machine(
                    "table", setting, output_units="inch", decimals=4, helices=False, **self.LIMITS, **machine
                ),
                post=getattr(m, post),
                decoder=getattr(m, decoder),
                lowering=camlib.LoweringOptions(native_arcs=True),
                accept={"unchecked_machine"},
            ).text.splitlines()
            moves = [line.split() for line in lines if line[:2] in ("G0", "G1", "G2", "G3")]
            fed = [words for words in moves if words[0] != "G0"]
            # To (150, 120) mm, from a block that started at x 150 mm.
            self.assertTrue({"X5.9055", "Y4.7244"} <= set(fed[-1]), (module, fed[-1]))
            self.assertTrue(any("X5.9055" in words for words in moves[:-1]), module)

    def test_corner_loops_and_slow_downs_post_through_every_plasma_post(self):
        # Corner loops and a corner slow-down (#296 A3), in inches: the loops
        # are ordinary arcs and the slow-down an ordinary feed word, decoded
        # and certified against the plan like the rest. Holding height
        # control off round a corner is posted where the control switches it
        # from the program (QtPlasmaC).
        from dataclasses import replace

        from test_drill_cycles import example

        tool = camlib.Torch("torch", kerf_width=1.5)
        corners = dict(
            corner_loops=camlib.CornerLoops(3, min_angle_deg=45, max_angle_deg=135),
            corner_slowdown=camlib.CornerSlowdown(4, feed_percent=50, min_angle_deg=60),
        )
        for module, post, decoder, machine, cut in (
            ("qtplasmac_post", "QtPlasmaCPost", "QtPlasmaCDecoder", dict(planes=("xy",), max_sweep_rad=math.tau), {}),
            (
                "centroid_plasma_post",
                "CentroidPlasmaPost",
                "CentroidPlasmaDecoder",
                dict(max_sweep_rad=math.tau),
                dict(height_control=False),
            ),
            ("wincnc_plasma_post", "WinCNCPlasmaPost", "WinCNCPlasmaDecoder", {}, dict(height_control=False)),
        ):
            m = example(module)
            texts = []
            for options in ({}, corners):
                j = job(tool, **{**self.FIXED, **cut, **options})
                # The torch starts at the cut height (a fixed-height program carries no Z).
                start = replace(j.setups[0], initial_position_mm=(0, 0, THICKNESS + 1.5))
                j = camlib.Job("plasma", "1", (start,))
                header = j.compile().setup("sheet").program_header()
                setting = ToolSetting.for_tool(
                    tool, number=1, length_register=1, position_after_mm=header.initial_position_mm
                )
                texts.append(
                    j.post_nc(
                        "sheet",
                        machine=m.define_machine(
                            "table", setting, output_units="inch", decimals=4, helices=False, **self.LIMITS, **machine
                        ),
                        post=getattr(m, post),
                        decoder=getattr(m, decoder),
                        lowering=camlib.LoweringOptions(native_arcs=True),
                        accept={"unchecked_machine"},
                    ).text.splitlines()
                )
            plain, treated = texts
            feeds = lambda lines: {w for line in lines for w in line.split() if w[0] == "F"}  # noqa: E731
            # 2500 mm/min is 98.4252 in/min; half of it round the corner the
            # plate's outline starts at (the other three have loops).
            self.assertEqual(feeds(treated) - feeds(plain), {"F49.2126"}, module)
            arcs = lambda lines: sum(line[:2] in ("G2", "G3") for line in lines)  # noqa: E731
            # Three loops of three quarter arcs each replace three quarter
            # arcs: every one a native arc, in inches too.
            self.assertEqual(arcs(treated) - arcs(plain), 6, module)
            xs = lambda lines: max(float(w[1:]) for line in lines for w in line.split() if w[0] == "X")  # noqa: E731
            # The plate's right edge is cut at 120.755 mm; the loops there run
            # on 3 mm and turn at 3 mm radius.
            self.assertEqual(xs(plain), round(120.755 / 25.4, 4))
            # (an arc's end word may be the neighbouring count: start and end radius agree exactly)
            self.assertAlmostEqual(xs(treated), 126.755 / 25.4, delta=1.01e-4)
            if module == "qtplasmac_post":
                # QtPlasmaC: M62 P2 holds height control off, M63 P2 lets it
                # run. Every block at the corner feed is cut with it off.
                held, slow = True, 0
                for line in treated:
                    if line in ("M62 P2", "M63 P2"):
                        held = line == "M62 P2"
                    elif "F49.2126" in line:
                        self.assertTrue(held, line)
                        slow += 1
                self.assertGreaterEqual(slow, 2)
                self.assertIn("M63 P2", treated)


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


class ChainCuttingTest(unittest.TestCase):
    """Chain cutting (#296 B2): one pierce for a row of parts, the links cut
    through the scrap with the torch on."""

    @staticmethod
    def row():
        squares = [(10 + 36 * k, 40, 36 + 36 * k, 66) for k in range(5)]
        return squares, PlanarRegion(
            "row",
            tuple(
                Component(
                    Boundary.polygon(((x0, y0), (x1, y0), (x1, y1), (x0, y1))),
                    (circle((x0 + 13, y0 + 13), 5),),
                )
                for x0, y0, x1, y1 in squares
            ),
            0,
        )

    def cut(self, **options):
        squares, region = self.row()
        beam, links, report = False, [], None
        pierces = 0
        with job(parts=region, **options).regenerate() as run:
            for u in run:
                update = u.update
                if isinstance(update, VerificationUpdate):
                    self.assertEqual(update.status, "clear", update)
                if isinstance(update, OperationUpdate) and update.stage == "events":
                    for event in update.events:
                        if event["kind"] == "beam":
                            beam = event["power_percent"] > 0
                            pierces += beam
                motion = getattr(update, "motion", None)
                if motion is not None and motion["kind"] == "lead":
                    self.assertTrue(beam, motion)
                    a, b = motion["curve"]["start"], motion["curve"]["end"]
                    if motion["curve"]["kind"] == "line" and math.dist(a[:2], b[:2]) > 5:
                        links.append((a, b))
                if isinstance(update, OperationUpdate) and update.stage == "end":
                    report = update.report
        return squares, pierces, links, report

    def test_a_row_is_cut_on_one_pierce_and_every_link_is_clear(self):
        _, pierces, links, plain = self.cut()
        self.assertEqual((pierces, links, plain["pierces"]), (10, [], 10))
        self.assertNotIn("chain_links", plain)
        squares, pierces, links, report = self.cut(chain=camlib.ChainCutting(45))
        self.assertEqual((pierces, len(links)), (6, 4))
        self.assertEqual((report["pierces"], report["chain_links"]), (6, 4))
        self.assertAlmostEqual(report["chain_length_mm"], sum(math.dist(a[:2], b[:2]) for a, b in links), places=9)
        # Four pierce delays fewer, and the travel between the outlines gone.
        self.assertAlmostEqual(plain["dwell_seconds"] - report["dwell_seconds"], 4 * 0.5, places=9)
        self.assertLess(report["rapid_length_mm"], plain["rapid_length_mm"])
        # The holes before the outlines, every part's hole before its outline.
        self.assertTrue(all(hole is not None for _, hole in report["order"][:5]))
        self.assertTrue(all(hole is None for _, hole in report["order"][5:]))
        # Independently of the planner: every link's kerf (0.75 mm either
        # side) stays off every part, and none is longer than declared.
        for a, b in links:
            self.assertLessEqual(math.dist(a[:2], b[:2]), 45)
            for k in range(401):
                x, y = (a[i] + k / 400 * (b[i] - a[i]) for i in range(2))
                for x0, y0, x1, y1 in squares:
                    off = math.hypot(max(x0 - x, x - x1, 0), max(y0 - y, y - y1, 0))
                    self.assertGreater(off, 0.75, (a, b, x, y))

    def test_a_link_that_is_too_long_or_not_declared_sensibly(self):
        _, pierces, links, report = self.cut(chain=camlib.ChainCutting(5))
        self.assertEqual((pierces, links, report["pierces"], report["chain_links"]), (10, [], 10, 0))
        with self.assertRaisesRegex(Exception, "max_link_mm"):
            self.cut(chain=camlib.ChainCutting(0))
        with self.assertRaisesRegex(Exception, "chain is not planned together with common_line"):
            self.cut(chain=camlib.ChainCutting(45), common_line=camlib.CommonLine(5))

    def test_a_chain_posts_through_every_plasma_post_in_inches(self):
        # The links are ordinary feed moves with the torch on: each post
        # writes four torch-on codes fewer, and its program decodes back to
        # the plan.
        from dataclasses import replace

        from test_drill_cycles import example

        _, region = self.row()
        tool = camlib.Torch("torch", kerf_width=1.5)
        fixed, limits = BeamPostsInInchesTest.FIXED, BeamPostsInInchesTest.LIMITS
        for module, post, decoder, machine, cut, on in (
            (
                "qtplasmac_post",
                "QtPlasmaCPost",
                "QtPlasmaCDecoder",
                dict(planes=("xy",), max_sweep_rad=math.tau),
                {},
                "M3",
            ),
            (
                "centroid_plasma_post",
                "CentroidPlasmaPost",
                "CentroidPlasmaDecoder",
                dict(max_sweep_rad=math.tau),
                dict(height_control=False),
                "M61",
            ),
            ("wincnc_plasma_post", "WinCNCPlasmaPost", "WinCNCPlasmaDecoder", {}, dict(height_control=False), "M61"),
        ):
            m = example(module)
            counts = []
            for options in ({}, dict(chain=camlib.ChainCutting(45))):
                j = job(tool, parts=region, **{**fixed, **cut, **options})
                start = replace(j.setups[0], initial_position_mm=(0, 0, THICKNESS + 1.5))
                j = camlib.Job("plasma", "1", (start,))
                header = j.compile().setup("sheet").program_header()
                setting = ToolSetting.for_tool(
                    tool, number=1, length_register=1, position_after_mm=header.initial_position_mm
                )
                posted = j.post_nc(
                    "sheet",
                    machine=m.define_machine(
                        "table", setting, output_units="inch", decimals=4, helices=False, **limits, **machine
                    ),
                    post=getattr(m, post),
                    decoder=getattr(m, decoder),
                    lowering=camlib.LoweringOptions(native_arcs=True),
                    accept={"unchecked_machine"},
                )
                counts.append(sum(line.split()[0] == on for line in posted.text.splitlines() if line.split()))
            self.assertEqual(counts, [10, 6], module)


class BridgesTest(unittest.TestCase):
    """Bridges between parts (#296 B3): neighbouring parts stay joined by an
    uncut strip and are cut as one loop."""

    BRIDGES = camlib.Bridges(3, max_length_mm=12, max_parts=8)

    def test_a_row_is_one_loop_and_the_strips_are_never_crossed(self):
        squares, region = ChainCuttingTest.row()
        beam, pierces, report, cuts = False, 0, None, []
        with job(parts=region, bridges=self.BRIDGES).regenerate() as run:
            for u in run:
                update = u.update
                if isinstance(update, VerificationUpdate):
                    self.assertEqual(update.status, "clear", update)
                if isinstance(update, OperationUpdate) and update.stage == "events":
                    for event in update.events:
                        if event["kind"] == "beam":
                            beam = event["power_percent"] > 0
                            pierces += beam
                motion = getattr(update, "motion", None)
                if motion is not None and motion["kind"] != "rapid":
                    self.assertTrue(beam, motion)
                    cuts.append(motion["curve"])
                if isinstance(update, OperationUpdate) and update.stage == "end":
                    report = update.report
        self.assertEqual((pierces, report["pierces"]), (6, 6))
        self.assertEqual(report["bridged_groups"], [[0, 1, 2, 3, 4]])
        self.assertEqual([b["parts"] for b in report["bridges"]], [[k, k + 1] for k in range(4)])
        # Independently of the planner: no beam-on move comes nearer the
        # middle line of a bridge than half the strip and half the kerf.
        for k, bridge in enumerate(report["bridges"]):
            x0, x1 = squares[k][2], squares[k + 1][0]
            self.assertAlmostEqual(bridge["length_mm"], x1 - x0)
            self.assertEqual(bridge["at_mm"], [(x0 + x1) / 2, 53])
            for curve in cuts:
                a, b = curve["start"][:2], curve["end"][:2]
                for i in range(41):
                    x, y = (a[j] + i / 40 * (b[j] - a[j]) for j in range(2))
                    if curve["kind"] != "line":
                        continue
                    if x0 - 0.7 < x < x1 + 0.7:
                        self.assertGreaterEqual(abs(y - 53), 1.5 + 0.75 - 1e-6, (curve, x, y))

    def test_bridges_post_through_every_plasma_post_in_inches(self):
        from dataclasses import replace

        from test_drill_cycles import example

        _, region = ChainCuttingTest.row()
        tool = camlib.Torch("torch", kerf_width=1.5)
        fixed, limits = BeamPostsInInchesTest.FIXED, BeamPostsInInchesTest.LIMITS
        for module, post, decoder, machine, cut, on in (
            (
                "qtplasmac_post",
                "QtPlasmaCPost",
                "QtPlasmaCDecoder",
                dict(planes=("xy",), max_sweep_rad=math.tau),
                {},
                "M3",
            ),
            (
                "centroid_plasma_post",
                "CentroidPlasmaPost",
                "CentroidPlasmaDecoder",
                dict(max_sweep_rad=math.tau),
                dict(height_control=False),
                "M61",
            ),
            ("wincnc_plasma_post", "WinCNCPlasmaPost", "WinCNCPlasmaDecoder", {}, dict(height_control=False), "M61"),
        ):
            m = example(module)
            j = job(tool, parts=region, **{**fixed, **cut, "bridges": self.BRIDGES})
            start = replace(j.setups[0], initial_position_mm=(0, 0, THICKNESS + 1.5))
            j = camlib.Job("plasma", "1", (start,))
            header = j.compile().setup("sheet").program_header()
            setting = ToolSetting.for_tool(
                tool, number=1, length_register=1, position_after_mm=header.initial_position_mm
            )
            posted = j.post_nc(
                "sheet",
                machine=m.define_machine(
                    "table", setting, output_units="inch", decimals=4, helices=False, **limits, **machine
                ),
                post=getattr(m, post),
                decoder=getattr(m, decoder),
                lowering=camlib.LoweringOptions(native_arcs=True),
                accept={"unchecked_machine"},
            )
            lines = [line.split() for line in posted.text.splitlines() if line.split()]
            self.assertEqual(sum(words[0] == on for words in lines), 6, module)

    def test_values_are_declared(self):
        _, region = ChainCuttingTest.row()
        with self.assertRaises(TypeError):
            camlib.Bridges(3)
        for bad in (camlib.Bridges(0, max_length_mm=12, max_parts=8), camlib.Bridges(3, max_length_mm=12, max_parts=1)):
            with self.assertRaisesRegex(Exception, "bridges: "):
                with job(parts=region, bridges=bad).regenerate() as run:
                    for _ in run:
                        pass


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
