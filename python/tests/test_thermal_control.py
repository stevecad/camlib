"""ThermalCut(compensation="control") (#384): the program carries the part
side of the kerf path and the controller applies the whole kerf, posted
with the WinCNC plasma post (G41 O<kerf/2> and G40, or a control declared to
apply its material setup's kerf with no word)."""

from dataclasses import replace
import math
import unittest

import camlib
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.post import GCodeDialect, Machine, ToolSetting
from test_drill_cycles import example

KERF = 1.6
HOLE = ((70, 50), 10)
PLATE = ((20, 20), (120, 80))
# The kerf path keeps half the default boundary tolerance beyond half a kerf.
MARGIN = 0.005


def plate():
    return PlanarRegion(
        "plate",
        (
            Component(
                Boundary.polygon(((20, 20), (120, 20), (120, 80), (20, 80))),
                (Boundary(((80, 50, 1), (60, 50, 1))),),
            ),
        ),
        0,
    )


def setup_job(compensation="control", parts=None, **cut):
    tool = camlib.Torch("torch", kerf_width=KERF)
    op = camlib.ThermalCut(
        "plasma",
        tool,
        parts or plate(),
        2500,
        1.5,
        lead_in=camlib.Lead(3, 2),
        lead_out=camlib.Lead(1.5, 1.5),
        pierce_delay_s=0,
        power_percent=100,
        tool_number=1,
        length_register=1,
        compensation=compensation,
        **cut,
    )
    setup = camlib.Setup(
        "sheet",
        stock=camlib.StockSpec((0, 0, 0), (200, 120, 6), 0.25),
        tools=(tool,),
        operations=(camlib.OperationNode(op, sources=()),),
        initial_position_mm=(0, 0, 7.5),
        verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
    )
    return tool, camlib.Job("plasma", "1", (setup,))


def wincnc_machine(w, j, tool, units="inch", diameter_register=None, **declared):
    header = j.compile().setup("sheet").program_header()
    setting = ToolSetting.for_tool(
        tool,
        number=1,
        length_register=1,
        position_after_mm=header.initial_position_mm,
        **({} if diameter_register is None else {"diameter_register": diameter_register}),
    )
    return w.define_machine(
        "table",
        setting,
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
        torch_on_code="M61",
        torch_off_code="M62",
        end_code="M66",
        **declared,
    )


def post(w, j, machine, post_class=None):
    return j.post_nc(
        "sheet",
        machine=machine,
        post=post_class or w.WinCNCPlasmaPost,
        decoder=w.WinCNCPlasmaDecoder,
        lowering=camlib.LoweringOptions(native_arcs=True),
        accept={"unchecked_machine"},
    ).text.splitlines()


def moves_of(lines, unit):
    """(code, end, center) of each move in the lines, in mm."""
    out, at = [], None
    for line in lines:
        words = line.split(" ")
        if words[0] not in ("G0", "G1", "G2", "G3"):
            continue
        values = {w[0]: float(w[1:]) * unit for w in words[1:] if w[0] in "XYIJ"}
        end = (values.get("X", at and at[0]), values.get("Y", at and at[1]))
        center = (at[0] + values["I"], at[1] + values["J"]) if words[0] in ("G2", "G3") else None
        out.append((words[0], end, center))
        at = end
    return out


def to_plate(p):
    """Distance from p to the plate's outline (outside it)."""
    dx = max(PLATE[0][0] - p[0], p[0] - PLATE[1][0], 0)
    dy = max(PLATE[0][1] - p[1], p[1] - PLATE[1][1], 0)
    return math.hypot(dx, dy)


class ControllerKerfTest(unittest.TestCase):
    def setUp(self):
        self.w = example("wincnc_plasma_post")

    def check_program(self, lines, units, reverse):
        unit = 25.4 if units == "inch" else 1
        resolution = 0.5 * 10 ** -(4 if units == "inch" else 3) * unit
        word = "G42" if reverse else "G41"
        amount = "O" + (f"{KERF / 2 / 25.4:.4f}".rstrip("0") if units == "inch" else "0.8")
        starts = [k for k, line in enumerate(lines) if line.startswith(("G41", "G42"))]
        self.assertEqual(len(starts), 2, "the hole and the outline")
        for k in starts:
            # G41 O<kerf/2> alone, at the resolution, before the straight lead-in.
            self.assertEqual(lines[k], f"{word} {amount}")
            self.assertTrue(lines[k + 1].startswith("G1 "), lines[k + 1])
        cancels = [k for k, line in enumerate(lines) if line == "G40"][1:]  # after the header's
        self.assertEqual(len(cancels), 2)
        for k in cancels:
            # G40 alone, then the straight lead-out, then the torch off.
            self.assertTrue(lines[k + 1].startswith("G1 "), lines[k + 1])
            self.assertEqual(lines[k + 2], "M62")
        for index, (k, end) in enumerate(zip(starts, cancels)):
            moves = moves_of(lines[k : end + 2], unit)
            # The cut: from the end of the lead-in arc to the start of the
            # lead-out arc. Its written XY is the part outline (beyond the
            # boundary margin).
            loop = moves[1:-2]
            if index == 0:
                for code, p, center in loop:
                    self.assertAlmostEqual(math.dist(p, HOLE[0]), HOLE[1] - MARGIN, delta=2 * resolution)
                arcs = [m for m in loop[1:] if m[0] in ("G2", "G3")]
                self.assertTrue(arcs)
                for code, p, center in arcs:
                    self.assertLess(math.dist(center, HOLE[0]), 2 * resolution)
                    # Holes counter-clockwise; reversed, clockwise.
                    self.assertEqual(code, "G2" if reverse else "G3")
            else:
                corners = 0
                # (the first is the lead-in arc, which ends on the edge)
                for code, p, center in loop:
                    self.assertIn(code, ("G1",) if p is not loop[0][1] else ("G2", "G3"))
                    d = to_plate(p)
                    self.assertGreaterEqual(d, MARGIN - 2 * resolution, p)
                    self.assertLessEqual(d, MARGIN * math.sqrt(2) + 2 * resolution, p)
                    corners += d > MARGIN * 1.2
                self.assertGreaterEqual(corners, 4)
                points = [p for _, p, _ in loop]
                area = sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(points, points[1:] + points[:1]))
                # Outlines clockwise; reversed, counter-clockwise.
                self.assertGreater(area, 0) if reverse else self.assertLess(area, 0)

    def test_g41_words_on_their_own_blocks_and_the_part_outline(self):
        for units in ("inch", "mm"):
            for reverse in (False, True):
                with self.subTest(units=units, reverse=reverse):
                    tool, j = setup_job(reverse=reverse)
                    m = wincnc_machine(self.w, j, tool, units, control_kerf="g41", cutcomp_full_circle_mm=0)
                    self.check_program(post(self.w, j, m), units, reverse)

    def test_a_control_that_applies_its_material_kerf_gets_no_compensation_word(self):
        tool, j = setup_job()
        g41 = post(self.w, j, wincnc_machine(self.w, j, tool, control_kerf="g41", cutcomp_full_circle_mm=0))
        machine = wincnc_machine(
            self.w, j, tool, control_kerf="material", material_kerf_mm=KERF, cutcomp_full_circle_mm=0
        )
        material = post(self.w, j, machine)
        # The same program less its compensation words (the header's G40
        # stays: it changes nothing).
        header = g41.index("G40")
        expected = [line for k, line in enumerate(g41) if k == header or not line.startswith(("G41", "G40"))]
        self.assertEqual(material, expected)
        self.assertFalse(any(line.startswith(("G41", "G42")) for line in material))
        # Its premises: the plan's kerf is the material's, cut to the left of
        # travel, and only compensated loops (the control offsets every cut).
        for declared, cut, message in (
            (dict(material_kerf_mm=2.0), {}, "material setup"),
            ({}, dict(reverse=True), "left of travel"),
            ({}, dict(crop=camlib.CropCut(((150, 0), (150, 120)))), "every cut"),
        ):
            with self.subTest(message):
                tool, j = setup_job(**cut)
                values = dict(control_kerf="material", material_kerf_mm=KERF, cutcomp_full_circle_mm=0)
                values.update(declared)
                with self.assertRaisesRegex(Exception, message):
                    post(self.w, j, wincnc_machine(self.w, j, tool, **values))

    def test_wear_compensation_still_posts_and_decodes(self):
        # #391: G41/G42 with the head's D register (wear) on a WinCNC table
        # that declares no control kerf, as before #384; refused where the
        # control applies its material setup's kerf (it would offset twice).
        tool, j = setup_job(compensation="wear")
        lines = post(self.w, j, wincnc_machine(self.w, j, tool, diameter_register=3))
        starts = [line for line in lines if line.startswith(("G41", "G42"))]
        self.assertTrue(starts, lines)
        self.assertTrue(all(line.split(" ")[:2] == ["G41", "D3"] for line in starts), starts)
        self.assertIn("G40", lines[lines.index(starts[0]) :])
        machine = wincnc_machine(
            self.w,
            j,
            tool,
            diameter_register=3,
            control_kerf="material",
            material_kerf_mm=KERF,
            cutcomp_full_circle_mm=0,
        )
        with self.assertRaisesRegex(Exception, "applies its material setup's kerf itself"):
            post(self.w, j, machine)

    def test_the_decoder_replays_the_offset_of_what_was_written(self):
        w = self.w

        class FullKerf(w.WinCNCPlasmaPost):
            # Writes the kerf, not its half: the control would cut a kerf
            # path half a kerf wider.
            def on_cutter_compensation(self, event):
                if event.amount_mm is not None:
                    event = replace(event, amount_mm=2 * event.amount_mm)
                    self._stretch_open = True
                    self._control_compensation = True
                    self.emit("G41", *self.words(("O",), (event.amount_mm,)))
                    return
                return super().on_cutter_compensation(event)

        tool, j = setup_job()
        m = wincnc_machine(w, j, tool, control_kerf="g41", cutcomp_full_circle_mm=0)
        with self.assertRaisesRegex(Exception, "differ|deviation"):
            post(w, j, m, FullKerf)

    def test_controls_without_an_amount_word_refuse(self):
        tool, j = setup_job()
        # WinCNC without a declared way of taking the kerf.
        with self.assertRaisesRegex(Exception, "control_kerf"):
            post(self.w, j, wincnc_machine(self.w, j, tool, cutcomp_full_circle_mm=0))
        # Compensated arcs need WinCNC's full-circle setting declared.
        with self.assertRaisesRegex(Exception, "cutcomp_full_circle_mm"):
            post(self.w, j, wincnc_machine(self.w, j, tool, control_kerf="g41"))
        # A dialect with no compensation amount word.
        header = j.compile().setup("sheet").program_header()
        setting = ToolSetting.for_tool(tool, number=1, length_register=1, position_after_mm=header.initial_position_mm)
        generic = Machine(
            "laser",
            (setting,),
            dialect=GCodeDialect.linuxcnc(beam_s_max=1000),
            output_units="mm",
            decimals=3,
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            arcs=True,
            helices=False,
            planes=("xy",),
            max_sweep_rad=math.pi,
            flood=False,
            mist=False,
            tool_change_position_mode="preserve_tip",
            max_tool_number=1,
            max_length_register=1,
            tool_changes=True,
            rapid_mm_min=6000,
        )
        with self.assertRaisesRegex(Exception, "compensation_amount_word"):
            j.post_nc(
                "sheet",
                machine=generic,
                lowering=camlib.LoweringOptions(native_arcs=True),
                accept={"unchecked_machine"},
            )

    def test_refused_where_the_controller_cannot_form_the_offset(self):
        slot = PlanarRegion(
            "slot",
            (
                Component(
                    Boundary.polygon(
                        ((20, 20), (120, 20), (120, 80), (70.5, 80), (70.5, 60), (69.5, 60), (69.5, 80), (20, 80))
                    )
                ),
            ),
            0,
        )
        for parts, cut, message in (
            (slot, {}, "narrower than the kerf"),
            (None, dict(common_line=camlib.CommonLine(5)), "common_line"),
            (None, dict(micro_joints=camlib.MicroJoints(1, count=2)), "micro-joints"),
            (None, dict(corner_loops=camlib.CornerLoops(3, min_angle_deg=45, max_angle_deg=135)), "corner_loops"),
        ):
            with self.subTest(message):
                tool, j = setup_job(parts=parts, **cut)
                with self.assertRaisesRegex(Exception, message):
                    post(self.w, j, wincnc_machine(self.w, j, tool, control_kerf="g41", cutcomp_full_circle_mm=0))


if __name__ == "__main__":
    unittest.main()
