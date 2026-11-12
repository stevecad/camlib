"""Turning: a profile's arcs are posted as native G2/G3 (G18), not as chords.

The posted arcs are checked against the part's drawing with this file's own
geometry (nothing of the planner's): every point of a posted arc, moved from
the imaginary tip to the nose center, is one nose radius from the profile.
"""

import math
import unittest

from camlib.post import GCodeDialect, LoweringOptions

from test_turning import job, lathe

RN = 0.8
BULGE = math.tan(math.pi / 8)  # a quarter turn
# A ball end (radius 6), a concave fillet (radius 3) into a shoulder and a
# sharp outer corner: (radius, z, bulge).
ROUNDED = ((0, 0, BULGE), (6, -6, 0), (6, -20, -BULGE), (9, -23, 0), (14, -23, 0), (14, -40, 0))


def distance(z, r):
    """From (z, r) to the ROUNDED profile, from its drawing."""

    def segment(a, b):
        d = (b[0] - a[0], b[1] - a[1])
        t = max(0.0, min(1.0, ((z - a[0]) * d[0] + (r - a[1]) * d[1]) / (d[0] ** 2 + d[1] ** 2)))
        return math.hypot(z - a[0] - t * d[0], r - a[1] - t * d[1])

    def quarter(c, radius, a0):
        # A quarter circle from angle a0 (from +z toward +r); its ends
        # belong to the neighbouring lines as well.
        a = math.atan2(r - c[1], z - c[0])
        if (a - a0) % math.tau <= math.pi / 2:
            return abs(math.hypot(z - c[0], r - c[1]) - radius)
        return math.inf

    return min(
        math.hypot(z, r),
        quarter((-6, 0), 6, 0),
        segment((-6, 6), (-20, 6)),
        quarter((-20, 9), 3, math.pi),
        segment((-23, 9), (-23, 14)),
        segment((-23, 14), (-40, 14)),
    )


def blocks(nc, *, diameter=False):
    """The motion blocks of a lathe program, read with a plain modal reader:
    (code, start, end, centre) in (x radius, z); centre None for a line."""
    x = z = None
    code = None
    out = []
    for line in nc:
        words = line.split()
        if not words or words[0].startswith("("):
            continue
        values = {}
        for w in words:
            if w in ("G0", "G1", "G2", "G3"):
                code = w
            elif w[0] in "XZIK" and w[1:].replace(".", "").replace("-", "").isdigit():
                values[w[0]] = float(w[1:])
        if not values.keys() & set("XZ"):
            continue
        end = (values.get("X", 0) / (2 if diameter else 1) if "X" in values else x, values.get("Z", z))
        centre = None
        if code in ("G2", "G3"):
            # I and K are incremental from the start, I a radius value.
            centre = (x + values.get("I", 0), z + values.get("K", 0))
        out.append((code, (x, z), end, centre))
        x, z = end
    return out


def arc_points(code, start, end, centre, n=100):
    """Points (x, z) along a G18 arc: G3 turns from +Z toward +X (counter-
    clockwise seen from +Y), G2 the other way."""
    radius = math.hypot(start[0] - centre[0], start[1] - centre[1])
    a0 = math.atan2(start[0] - centre[0], start[1] - centre[1])
    a1 = math.atan2(end[0] - centre[0], end[1] - centre[1])
    sweep = (a1 - a0) % math.tau if code == "G3" else -((a0 - a1) % math.tau)
    return [
        (centre[0] + radius * math.sin(a0 + sweep * k / n), centre[1] + radius * math.cos(a0 + sweep * k / n))
        for k in range(n + 1)
    ]


class TurningArcsTest(unittest.TestCase):
    def setUp(self):
        self.plan = job(profile=ROUNDED).plan()
        self.machine = lathe(self.plan)

    def post(self, machine=None, **options):
        return self.plan.job.post_nc("lathe", machine=machine or self.machine, accept={"unchecked_machine"}, **options)

    def test_finish_posts_the_ball_fillet_and_corner_as_arcs(self):
        posted = self.post()
        nc = posted.text.splitlines()
        arcs = [b for b in blocks(nc) if b[3] is not None]
        # The ball end (G3), the fillet (G2) and the nose rolling round the
        # sharp corner at r 14 (G3), each one block.
        self.assertEqual([b[0] for b in arcs], ["G3", "G2", "G3"], nc)
        radii = [math.hypot(b[1][0] - b[3][0], b[1][1] - b[3][1]) for b in arcs]
        for got, want in zip(radii, (6 + RN, 3 - RN, RN)):
            self.assertAlmostEqual(got, want, delta=2e-3)
        # Every posted arc, at the nose center (the imaginary tip is one nose
        # radius toward the axis and toward -z of it), is one nose radius
        # from the drawn profile: within the 3-decimal words.
        for code, start, end, centre in arcs:
            for x, z in arc_points(code, start, end, centre):
                self.assertAlmostEqual(distance(z + RN, x + RN), RN, delta=2e-3, msg=(code, start, end, centre))
        # The material and part checks pass on the posted program.
        self.assertEqual(posted.material["gouge"]["assessment"], "within_tolerance")
        self.assertEqual(posted.material["residual"]["definite_cells"], 0, posted.material["findings"][:3])

    def test_the_plane_is_selected_once_for_all_arcs(self):
        """#204: G18 is modal (modal group 2), so the header's G18 is the only
        one in a lathe program; the decoder reads every arc in XZ from it."""
        from camlib.motion import Arc, Motion
        from camlib.post import NativeGCodeDecoder

        nc = self.post().text.splitlines()
        self.assertEqual(sum(line.split()[0] in ("G2", "G3") for line in nc if line), 3)
        self.assertEqual([line for line in nc if line in ("G17", "G18", "G19")], ["G18"], nc)

        def planes(lines):
            decoder = NativeGCodeDecoder(self.machine, self.plan.program)
            events = [e for x in lines for e in decoder.read_line(x, dict(id=1, operation="file", kind="cut"))]
            return [e.curve.plane for e in events if isinstance(e, Motion) and isinstance(e.curve, Arc)]

        self.assertEqual(planes(nc), ["xz"] * 3)
        # Without that one G18 no arc has a plane: the program is refused.
        with self.assertRaises(ValueError):
            planes([line for line in nc if line != "G18"])

    def test_arcs_take_far_fewer_blocks_than_their_chords(self):
        native = self.post().text.splitlines()
        # The same plan with every curve as checked line segments (chords
        # of a convex arc pass inside it, toward the part: they must stay
        # within the setup's tolerance, 0.001 here, or the replay refuses).
        chords = self.post(lowering=LoweringOptions(chord_tolerance_mm=2e-4)).text.splitlines()
        self.assertFalse([b for b in blocks(chords) if b[3] is not None])
        self.assertLess(len(native) + 30, len(chords), (len(native), len(chords)))

    def test_diameter_mode_keeps_arc_centres_as_radius_values(self):
        machine = lathe(self.plan, GCodeDialect(x_diameter=True))
        nc = self.post(machine).text.splitlines()
        arcs = [line for line in nc if line.split()[0] in ("G2", "G3")]
        self.assertEqual(len(arcs), 3, nc)
        # The fillet: from the tip on the r 6 diameter round to r 8.2 at
        # the shoulder (X a diameter), its centre 2.2 above the start (I a
        # radius value, as FANUC and Centroid lathes read it).
        fillet = arcs[1].split()
        self.assertEqual(fillet[0], "G2")
        self.assertIn("X16.4", fillet)
        self.assertIn("I2.2", fillet)
        # Read back with X halved, the arcs are the radius-mode ones.
        radius_mode = [b for b in blocks(self.post().text.splitlines()) if b[3] is not None]
        diameter_mode = [b for b in blocks(nc, diameter=True) if b[3] is not None]
        self.assertEqual(len(diameter_mode), 3)
        for a, b in zip(radius_mode, diameter_mode):
            self.assertEqual(a[0], b[0])
            for p, q in zip(a[1:], b[1:]):
                self.assertAlmostEqual(p[0], q[0], delta=1e-9)
                self.assertAlmostEqual(p[1], q[1], delta=1e-9)

    def test_fanuc_and_centroid_lathe_posts_write_the_arcs(self):
        from camlib.post import ToolSetting
        from test_drill_cycles import example

        tools = tuple(
            ToolSetting.for_tool(
                t, number=i, length_register=i, position_after_mm=self.plan.program.initial_position_mm
            )
            for i, t in enumerate(self.plan.program.tools, 1)
        )
        declared = dict(
            output_units="mm",
            decimals=3,
            max_feed_mm_min=5000,
            arcs=True,
            helices=False,
            max_sweep_rad=math.tau,
            max_tool_number=12,
            max_length_register=12,
            tool_changes=True,
            flood=True,
            mist=False,
            rapid_mm_min=8000,
            max_spindle_rpm=4000,
        )
        for name, point in (("fanuc_lathe_post", "."), ("centroid_lathe_post", "")):
            module = example(name)
            fanuc = "fanuc" in name
            machine = module.define_machine("Lathe", tools, **declared, **(dict(program_number=1001) if fanuc else {}))
            with self.subTest(post=name):
                # Both posts demand exactly consistent arc words
                # (radius_tolerance_mm=0) and their own decoders read the
                # program back: no lowering is needed for these arcs.
                nc = self.post(
                    machine,
                    post=module.FanucLathePost if fanuc else module.CentroidLathePost,
                    decoder=module.FanucLatheDecoder if fanuc else module.CentroidLatheDecoder,
                ).text.splitlines()
                arcs = [line.split() for line in nc if line.split()[0] in ("G2", "G3")]
                # X a diameter, I and K radius values from the start.
                self.assertEqual(
                    [a[:5] for a in arcs],
                    [
                        ["G3", f"X12{point}", "Z-6.8", "K-6.8", f"I0{point}"],
                        ["G2", "X16.4", f"Z-23{point}", f"K0{point}", "I2.2"],
                        ["G3", f"X28{point}", "Z-23.8", "K-0.8", f"I0{point}"],
                    ],
                )
                for code, start, end, centre in blocks(nc, diameter=True):
                    if centre is None:
                        continue
                    for x, z in arc_points(code, start, end, centre):
                        self.assertAlmostEqual(distance(z + RN, x + RN), RN, delta=2e-3)

    def test_posted_arcs_replay_on_the_stock(self):
        from camlib.playback import Cursor, Player

        with self.plan.job.post("lathe", machine=self.machine) as run:
            run.wait()
            player = Player(run)
            player.seek(Cursor(run.status()["available"]))
            posted = player.state.stock()
        # The decoded program (3-decimal arcs) removes the planned stock to
        # within its output rounding.
        worst = max(
            abs(a["outer"]["upper_mm"] - b["outer"]["upper_mm"])
            for a, b in zip(posted["cells"], self.plan.stock["cells"])
        )
        self.assertLess(worst, 2e-3)

    def test_an_arc_plays_back_part_way(self):
        from camlib.playback import Cursor, Player

        with self.plan.job.record("lathe", rapid_mm_min=6000) as run:
            run.wait()
            player = Player(run)
            total = run.status()["available"]
            # Half way along each finishing arc: the stock there lies
            # between the stock before and after the whole move (strictly
            # on the ball end; the first half of the roll round the corner
            # only passes where the shoulder's pass already cut).
            arcs = strict = 0
            for k in range(total):
                player.seek(Cursor(k, 0.5))
                motion = player.motion
                if motion["curve"]["kind"] != "arc":
                    continue
                self.assertEqual((motion["operation"], motion["curve"]["plane"]), ("finish", "xz"))
                half = player.state.volume()["removed_lower_mm3"]
                player.seek(Cursor(k))
                before = player.state.volume()["removed_lower_mm3"]
                player.seek(Cursor(k + 1))
                after = player.state.volume()["removed_lower_mm3"]
                self.assertTrue(before <= half <= after and before < after, (k, before, half, after))
                strict += before < half < after
                player.seek(Cursor(k, 1.0))
                self.assertAlmostEqual(player.state.volume()["removed_lower_mm3"], after, places=6)
                arcs += 1
            self.assertEqual(arcs, 3)
            self.assertGreaterEqual(strict, 1)


if __name__ == "__main__":
    unittest.main()
