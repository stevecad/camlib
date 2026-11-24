"""ThermalCut lead shapes (#400): Lead.line at an angle and in line with the
edge, Lead.arc of a set sweep, Lead.line_arc and Lead.arc_line, and in-line
leads at a corner. Each is posted and decoded on the generic, QtPlasmaC and
WinCNC posts, and its angle, length and sweep are measured from the posted
program, with its kerf in the scrap."""

from dataclasses import replace
import math
import unittest

import camlib
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.post import GCodeDialect, Machine, ToolSetting
from test_drill_cycles import example

KERF = 1.5
PLATE = (40, 40, 70, 60)
# The kerf path: half a kerf and half the default boundary tolerance out.
OFFSET = KERF / 2 + 0.005


def to_plate(p, r=PLATE):
    dx = max(r[0] - p[0], p[0] - r[2], 0)
    dy = max(r[1] - p[1], p[1] - r[3], 0)
    return math.hypot(dx, dy)


def cut_job(lead_in, lead_out, start, **cut):
    tool = camlib.Torch("torch", kerf_width=KERF)
    x0, y0, x1, y1 = PLATE
    op = camlib.ThermalCut(
        "plasma",
        tool,
        PlanarRegion("plate", (Component(Boundary.polygon(((x0, y0), (x1, y0), (x1, y1), (x0, y1)))),), 0),
        2500,
        1.5,
        lead_in=lead_in,
        lead_out=lead_out,
        pierce_delay_s=0,
        power_percent=100,
        tool_number=1,
        length_register=1,
        lead_placement=camlib.LeadPlacement(keep_off_distance_mm=0, starts=(start,)),
        **cut,
    )
    setup = camlib.Setup(
        "sheet",
        stock=camlib.StockSpec((0, 0, 0), (120, 100, 6), 0.25),
        tools=(tool,),
        operations=(camlib.OperationNode(op, sources=()),),
        initial_position_mm=(0, 0, 7.5),
        verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
    )
    return tool, camlib.Job("plasma", "1", (setup,))


LIMITS = dict(
    max_feed_mm_min=10000,
    max_spindle_rpm=30000,
    arcs=True,
    helices=False,
    max_tool_number=99999,
    max_length_register=99999,
    tool_changes=True,
    rapid_mm_min=6000,
)


def programs(lead_in, lead_out, start, **cut):
    """The program from each post: (name, lines, torch-on word, torch-off word)."""
    tool, j = cut_job(lead_in, lead_out, start, **cut)
    header = j.compile().setup("sheet").program_header()
    setting = ToolSetting.for_tool(tool, number=1, length_register=1, position_after_mm=header.initial_position_mm)
    generic = Machine(
        "plasma",
        (setting,),
        dialect=GCodeDialect(beam_s_max=1),
        tool_change_position_mode="preserve_tip",
        output_units="mm",
        decimals=3,
        planes=("xy", "xz", "yz"),
        max_sweep_rad=math.tau,
        flood=False,
        mist=False,
        **LIMITS,
    )
    out = []
    text = j.post_nc(
        "sheet", machine=generic, lowering=camlib.LoweringOptions(native_arcs=True), accept={"unchecked_machine"}
    )
    out.append(("generic", text.text.splitlines(), "M3 S1", "M5"))
    for module, post, decoder, machine, on, off in (
        (
            "qtplasmac_post",
            "QtPlasmaCPost",
            "QtPlasmaCDecoder",
            dict(planes=("xy",), max_sweep_rad=math.tau),
            "M3 $0 S1",
            "M5 $0",
        ),
        # WinCNC accepts arc words within its arc_err (wincnc-plasma-post.md).
        (
            "wincnc_plasma_post",
            "WinCNCPlasmaPost",
            "WinCNCPlasmaDecoder",
            dict(arc_radius_tolerance_mm=0.005),
            "M61",
            "M62",
        ),
    ):
        m = example(module)
        text = j.post_nc(
            "sheet",
            machine=m.define_machine("table", setting, output_units="mm", decimals=3, **LIMITS, **machine),
            post=getattr(m, post),
            decoder=getattr(m, decoder),
            lowering=camlib.LoweringOptions(native_arcs=True),
            accept={"unchecked_machine"},
        )
        out.append((module, text.text.splitlines(), on, off))
    return out


def burns(lines, on, off):
    """Each stretch with the torch on: its moves (code, start, end, centre)."""
    at, out, moves = None, [], None
    for line in lines:
        words = line.split()
        if line == on:
            moves = []
            continue
        if line == off or line.startswith(off + " "):
            if moves:
                out.append(moves)
            moves = None
            continue
        if not words or words[0] not in ("G0", "G1", "G2", "G3"):
            continue
        values = {w[0]: float(w[1:]) for w in words[1:] if w[0] in "XYIJ"}
        end = (values.get("X", at and at[0]), values.get("Y", at and at[1]))
        center = (at[0] + values["I"], at[1] + values["J"]) if words[0] in ("G2", "G3") else None
        if moves is not None and words[0] != "G0":
            moves.append((words[0], at, end, center))
        at = end
    return out


def sweep_of(move):
    """An arc's signed sweep in degrees (G3 counter-clockwise)."""
    code, a, b, c = move
    start = math.atan2(a[1] - c[1], a[0] - c[0])
    end = math.atan2(b[1] - c[1], b[0] - c[0])
    ccw = (end - start) % math.tau
    return math.degrees(ccw if code == "G3" else -((start - end) % math.tau))


def points(move, n=32):
    code, a, b, c = move
    if c is None:
        return [(a[0] + k / n * (b[0] - a[0]), a[1] + k / n * (b[1] - a[1])) for k in range(n + 1)]
    sweep = math.radians(sweep_of(move))
    u = (a[0] - c[0], a[1] - c[1])
    return [
        (
            c[0] + math.cos(k / n * sweep) * u[0] - math.sin(k / n * sweep) * u[1],
            c[1] + math.sin(k / n * sweep) * u[0] + math.cos(k / n * sweep) * u[1],
        )
        for k in range(n + 1)
    ]


class LeadShapesTest(unittest.TestCase):
    # Decimals 3: what is measured from a program is known to its rounding,
    # and an arc's words are made consistent by moving its centre by up to a
    # hundredth (posts.md: arc words).
    TOLERANCE = 1e-2

    def test_each_shape_posts_decodes_and_measures_as_declared(self):
        # The plate's outline is cut clockwise; its loop starts in the middle
        # of the bottom edge, cut towards -x with the scrap below.
        s = (55, PLATE[1] - OFFSET)
        along = lambda p: (s[0] - p[0], s[1] - p[1])  # noqa: E731 (t = -x, n = -y)
        cases = (
            (camlib.Lead.line(5, 30), camlib.Lead.line(4, 60)),
            (camlib.Lead.line_arc(2, 3), camlib.Lead.line(5, 0)),
            (camlib.Lead.arc(6, 45), camlib.Lead.arc(3, 150)),
            (camlib.Lead.line_arc(3, 2, 60), camlib.Lead.arc_line(2, 3, 120)),
            (camlib.Lead.line_arc(3, 2), camlib.Lead.arc_line(2, 1)),
        )
        for lead_in, lead_out in cases:
            for name, lines, on, off in programs(lead_in, lead_out, (55, 40)):
                with self.subTest(lead_in=lead_in, lead_out=lead_out, post=name):
                    (moves,) = burns(lines, on, off)
                    # Every lead and cut in the scrap: the kerf off the plate.
                    for move in moves:
                        for p in points(move):
                            self.assertGreaterEqual(to_plate(p), KERF / 2 - self.TOLERANCE, (move, p))
                    k_in = (lead_in.length_mm > 0) + (lead_in.radius_mm > 0)
                    k_out = (lead_out.length_mm > 0) + (lead_out.radius_mm > 0)
                    lin, lout = moves[:k_in], moves[-k_out:]
                    self.assertLess(math.dist(lin[-1][2], s), self.TOLERANCE)
                    self.assertLess(math.dist(lout[0][1], s), self.TOLERANCE)
                    for lead, legs, inward in ((lead_in, lin, True), (lead_out, lout, False)):
                        self.measure(lead, legs, inward, along)

    def measure(self, lead, legs, inward, along):
        if lead.angle_deg is not None:
            ((code, a, b, _),) = legs
            self.assertEqual(code, "G1")
            self.assertAlmostEqual(math.dist(a, b), lead.length_mm, delta=self.TOLERANCE)
            u, v = along(a if inward else b)
            self.assertAlmostEqual(math.degrees(math.atan2(v, abs(u))), lead.angle_deg, delta=0.2)
            # A lead-in comes from behind the start, a lead-out goes on.
            self.assertTrue(u <= self.TOLERANCE if inward else u >= -self.TOLERANCE)
            return
        arcs = [m for m in legs if m[3] is not None]
        lines = [m for m in legs if m[3] is None]
        (arc,) = arcs
        code, a, b, c = arc
        self.assertAlmostEqual(math.dist(a, c), lead.radius_mm, delta=self.TOLERANCE)
        # A centre moved by the tolerance turns each end's radius by up to
        # tolerance / radius either way.
        self.assertAlmostEqual(
            abs(sweep_of(arc)), lead.sweep_deg or 90, delta=math.degrees(4 * self.TOLERANCE / lead.radius_mm)
        )
        # The centre on the scrap side, square below the start: tangent to the
        # cut (the written centre is moved along the chord, posts.md, by up to
        # the rounding of both ends).
        u, v = along(c)
        self.assertAlmostEqual(u, 0, delta=2 * self.TOLERANCE)
        self.assertAlmostEqual(v, lead.radius_mm, delta=2 * self.TOLERANCE)
        self.assertEqual(len(lines), int(lead.length_mm > 0))
        if lines:
            (_, p, q, _) = lines[0]
            self.assertAlmostEqual(math.dist(p, q), lead.length_mm, delta=self.TOLERANCE)
            # Tangent to the arc where they meet: square to its radius there.
            meet = q if inward else p
            radial = (meet[0] - c[0], meet[1] - c[1])
            d = (q[0] - p[0], q[1] - p[1])
            cos = (radial[0] * d[0] + radial[1] * d[1]) / math.hypot(*radial) / math.hypot(*d)
            self.assertAlmostEqual(cos, 0, delta=self.TOLERANCE / lead.length_mm)

    def test_in_line_leads_at_a_corner_cross_beyond_it_on_every_post(self):
        # The corner (70, 40): clockwise, the right edge comes down to it and
        # the bottom edge leaves it. The lead-in comes in along the bottom
        # edge's extension, the lead-out runs on down the right edge's.
        x, y = PLATE[2] + OFFSET, PLATE[1] - OFFSET
        for name, lines, on, off in programs(camlib.Lead.line(6, 0), camlib.Lead.line(6, 0), (70, 40)):
            with self.subTest(post=name):
                (moves,) = burns(lines, on, off)
                (code, p, s, _), (code_out, e, q, _) = moves[0], moves[-1]
                self.assertEqual((code, code_out), ("G1", "G1"))
                self.assertAlmostEqual(p[1], y, delta=self.TOLERANCE)
                self.assertAlmostEqual(s[1], y, delta=self.TOLERANCE)
                self.assertAlmostEqual(p[0] - s[0], 6, delta=self.TOLERANCE)
                self.assertAlmostEqual(e[0], x, delta=self.TOLERANCE)
                self.assertAlmostEqual(q[0], x, delta=self.TOLERANCE)
                self.assertAlmostEqual(e[1] - q[1], 6, delta=self.TOLERANCE)
                # Both pass the kerf path's corner by more than half a kerf,
                # so the corner is cut through; no arc is cut round it.
                self.assertGreater(p[0] - x, KERF / 2)
                self.assertGreater(y - q[1], KERF / 2)
                corner = (PLATE[2], PLATE[1])
                self.assertFalse(any(m[3] is not None and math.dist(m[3], corner) < 0.1 for m in moves), moves)

    def test_shapes_serialize_only_what_they_declare(self):
        self.assertEqual(camlib.Lead(2, 3).to_dict(), dict(length_mm=2, radius_mm=3))
        self.assertEqual(camlib.Lead.line(4, 0).to_dict(), dict(length_mm=4, radius_mm=0, angle_deg=0))
        self.assertEqual(camlib.Lead.arc(2, 45).to_dict(), dict(length_mm=0, radius_mm=2, sweep_deg=45))
        self.assertEqual(camlib.Lead.line_arc(3, 2).to_dict(), dict(length_mm=3, radius_mm=2, sweep_deg=90))
        self.assertEqual(camlib.Lead.arc_line(2, 3, 120), camlib.Lead.line_arc(3, 2, 120))

    def test_refusals_name_the_shape(self):
        from camlib.simulation import SimulationError

        for lead_in, cut, message in (
            (camlib.Lead.line(3, 120), {}, "angle_deg"),
            (camlib.Lead.arc(2, 270), {}, "at most 180"),
            (camlib.Lead.arc(2, 90), dict(compensation="wear"), "Lead.arc has none"),
        ):
            with self.subTest(message):
                _, j = cut_job(lead_in, camlib.Lead(1, 1), (55, 40), **cut)
                with self.assertRaisesRegex(SimulationError, message):
                    with j.regenerate() as run:
                        list(run)

    def test_the_nest_keeps_room_for_the_shapes(self):
        # Lead room at the leads: the room holds each shape's reach along the
        # edge and out from it (kerf 1.5, pierce clearance one kerf: the
        # crater reaches 2.25 round the pierce).
        sheet = PlanarRegion("sheet", (Component(Boundary.polygon(((0, 0), (200, 0), (200, 120), (0, 120)))),), 0)
        x0, y0, x1, y1 = PLATE
        part = camlib.NestPart(
            PlanarRegion("plate", (Component(Boundary.polygon(((x0, y0), (x1, y0), (x1, y1), (x0, y1)))),), 0), 2, (0,)
        )
        tool, j = cut_job(camlib.Lead.line(5, 30), camlib.Lead.arc(3, 150), (55, 40))
        cut = j.setups[0].operations[0].operation
        planned = camlib.Nest.for_cut(
            replace(cut, lead_placement=None),
            sheet,
            (part,),
            part_spacing_mm=1,
            edge_margin_mm=3,
            resolution_mm=1,
            part_in_part=False,
            lead_room="at_leads",
        )
        result = planned.run()
        room = result.check["lead_room"]
        crater = KERF / 2 + KERF
        lead_in_width = 5 * math.cos(math.radians(30)) + crater
        lead_out_width = 3 + KERF / 2  # a quarter turn's reach along the edge
        self.assertAlmostEqual(room["half_width_mm"], max(lead_in_width, lead_out_width))
        lead_in_height = 5 * math.sin(math.radians(30)) + crater
        lead_out_height = 3 * (1 - math.cos(math.radians(150))) + KERF / 2
        self.assertAlmostEqual(room["height_mm"], OFFSET + max(lead_in_height, lead_out_height))
        # And the cut from the nest's starts is clear.
        op = replace(
            cut,
            parts=result.region,
            lead_placement=camlib.LeadPlacement(keep_off_distance_mm=0, starts=result.lead_starts),
        )
        setup = replace(j.setups[0], operations=(camlib.OperationNode(op, sources=()),))
        from camlib.simulation import VerificationUpdate

        statuses = set()
        with camlib.Job("nest", "1", (setup,)).regenerate() as run:
            for u in run:
                if isinstance(u.update, VerificationUpdate):
                    statuses.add(u.update.status)
        self.assertEqual(statuses, {"clear"})


if __name__ == "__main__":
    unittest.main()
