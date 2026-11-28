"""Rapid retracts: a straight-up retract from the end of a cut is posted as a
rapid where the stock proves it clear, cut history and whole-cell bounds both
(#430), and the stock-contact check proves it clear. Where stock stands beside
the tool, the retract feeds only through it and the rest is a rapid (#441)."""

import math
import unittest

import camlib
from camlib.simulation import Update, VerificationUpdate
from test_limits import square, tool


def job(rapid, holes=False):
    t = tool()
    ops = [
        # Faced first: its passes end past the stock edge, where nothing stands
        # beside the tool, so its lifts become rapids (#430).
        camlib.OperationNode(
            camlib.Facing(
                "face",
                t,
                9.5,
                0.5,
                3,
                20,
                feed_mm_min=600,
                plunge_mm_min=150,
                spindle_rpm=12000,
                tool_number=1,
                length_register=1,
                coolant="flood",
            ),
            sources=(),
        ),
        camlib.OperationNode(
            camlib.Pocket(
                "pocket",
                t,
                square(4, 4, 36, 36),
                10,
                6,
                2,
                2.5,
                20,
                feed_mm_min=600,
                plunge_mm_min=150,
                spindle_rpm=12000,
                tool_number=1,
                length_register=1,
                coolant="flood",
            )
        ),
        camlib.OperationNode(
            camlib.Contour(
                "walls",
                t,
                square(10, 10, 30, 30, name="part"),
                6,
                2,
                20,
                lead_in_mm=2,
                lead_out_mm=2,
                feed_mm_min=600,
                plunge_mm_min=150,
                spindle_rpm=12000,
                tool_number=1,
                length_register=1,
                coolant="flood",
            )
        ),
    ]
    setup = camlib.Setup(
        "top",
        stock=camlib.StockSpec((0, 0, 0), (40, 40, 10), 0.25),
        tools=(t,),
        operations=tuple(ops[:2]) if holes else tuple(ops),
        initial_position_mm=(0, 0, 20),
        verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
        rapid_retracts=rapid,
    )
    return camlib.Job("retracts", "1", (setup,))


def run(j, beside=None):
    moves, statuses = [], {}
    with j.regenerate() as r:
        for u in r:
            if isinstance(u.update, Update):
                moves.append(u.update.motion)
                if beside is not None and u.update.motion["kind"] in ("rapid", "retract"):
                    beside[u.update.motion["id"]] = stands_beside(r._simulation.state, u.update.motion)
            elif isinstance(u.update, VerificationUpdate):
                statuses[u.update.status] = statuses.get(u.update.status, 0) + 1
    return moves, statuses


def stands_beside(state, motion):
    """Whether a stock cell's whole-cell bound stands above the cutter where
    `motion` starts: within the cutter's radius (plus 32 sweep tolerances) of
    its axis and more than one sweep tolerance above its tip. Read from the
    stock after the move, which a straight lift leaves as it was."""
    tolerance = state.manifest["sweep"]["tolerance_mm"]
    x, y, z = motion["curve"]["start"]
    reach = tool().cutter.diameter_mm / 2 + 32 * tolerance
    heights = state.heights
    for iy in range(max(0, int((y - reach - state.min_mm[1]) // state.dy)), state.ny):
        y0 = state.min_mm[1] + iy * state.dy
        if y0 > y + reach:
            break
        for ix in range(max(0, int((x - reach - state.min_mm[0]) // state.dx)), state.nx):
            x0 = state.min_mm[0] + ix * state.dx
            if x0 > x + reach:
                break
            dx = max(x0 - x, x - x0 - state.dx, 0.0)
            dy = max(y0 - y, y - y0 - state.dy, 0.0)
            if math.hypot(dx, dy) <= reach and heights[2 * (iy * state.nx + ix) + 1] > z + tolerance:
                return True
    return False


def lifts(slow, fast):
    """Pair each move of the run without rapid retracts with the moves the
    run with them made of it: the same move, or a retract split in two
    (#441), a fed part and a straight-up rest, which together run the same
    line."""
    groups, j = [], 0
    for m in slow:
        f = fast[j]
        g = fast[j + 1] if j + 1 < len(fast) else None
        if (
            m["kind"] == "retract"
            and f["kind"] == "retract"
            and g is not None
            and f["curve"]["start"] == m["curve"]["start"]
            and f["curve"]["end"] != m["curve"]["end"]
            and g["curve"]["start"] == f["curve"]["end"]
            and g["curve"]["end"] == m["curve"]["end"]
        ):
            groups.append((m, [f, g]))
            j += 2
        else:
            groups.append((m, [f]))
            j += 1
    assert j == len(fast), (j, len(fast))
    return groups


class RapidRetractTest(unittest.TestCase):
    def test_retracts_become_rapids_and_check_clear(self):
        slow, slow_status = run(job(False))
        beside = {}
        fast, fast_status = run(job(True), beside)
        self.assertEqual(set(slow_status), {"clear"})
        self.assertEqual(set(fast_status), {"clear"}, fast_status)
        retracts = [m for m in slow if m["kind"] == "retract"]
        self.assertTrue(retracts)
        groups = lifts(slow, fast)
        # Same path: only the retracts change, in kind and feed, and where the
        # stock stands beside the tool a retract is split in two on its line.
        for m, made in groups:
            if m["kind"] != "retract":
                self.assertEqual([m["curve"]], [f["curve"] for f in made])
                self.assertEqual(made[0]["kind"], m["kind"])
                continue
            for f in made:
                a, b = f["curve"]["start"], f["curve"]["end"]
                self.assertEqual((a[0], a[1]), (b[0], b[1]), f)
                self.assertGreater(b[2], a[2], f)
        whole = [made[0] for m, made in groups if m["kind"] == "retract" and len(made) == 1]
        split = [made for m, made in groups if m["kind"] == "retract" and len(made) == 2]
        rapids = [f for f in whole if f["kind"] == "rapid"] + [rest for _, rest in split]
        kept = [f for f in whole if f["kind"] == "retract"] + [fed for fed, _ in split]
        self.assertTrue(rapids)
        self.assertTrue(split)
        self.assertEqual(len(fast), len(slow) + len(split))
        # A retract feeds exactly where stock stands beside the cutter above
        # its tip, which the whole-cell bounds cannot show clear (#430): here
        # the pocket's and the contour's lifts. Each feeds only through that
        # stock, and the rest of it, which starts where nothing stands beside
        # the cutter any more, is a rapid (#441). The facing's lift past the
        # stock edge is one rapid.
        for m in rapids:
            self.assertEqual(m["kind"], "rapid", m)
            self.assertFalse(beside[m["id"]], m)
            self.assertIsNone(m["feed_mm_min"])
        for m in kept:
            self.assertEqual(m["kind"], "retract", m)
            self.assertTrue(beside[m["id"]], m)
            self.assertEqual(m["feed_mm_min"], 600, m)


if __name__ == "__main__":
    unittest.main()


class AdaptiveHeightsTest(unittest.TestCase):
    def test_adaptive_links_at_the_retract_height(self):
        from test_limits import square, tool

        t = tool()
        op = camlib.AdaptivePocket(
            "adaptive",
            t,
            square(4, 4, 36, 36),
            10,
            6,
            2,
            2.5,
            20,
            retract_z_mm=13,
            feed_z_mm=11,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        setup = camlib.Setup(
            "top",
            stock=camlib.StockSpec((0, 0, 0), (40, 40, 10), 0.25),
            tools=(t,),
            operations=(camlib.OperationNode(op),),
            initial_position_mm=(0, 0, 20),
            verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
        )
        moves, statuses = run(camlib.Job("adaptive", "1", (setup,)))
        self.assertEqual(set(statuses), {"clear"}, statuses)
        inner = moves[1:-1]
        # Travel within a level is at Z13. The engagement front plans each level on its own and goes to the
        # clearance height between them (the one trip up, across and down at Z20 below); the removed orbit planner
        # stayed at Z13 there too (#190).
        travel = inner[1:]
        high = [i for i, m in enumerate(travel) if max(m["curve"]["start"][2], m["curve"]["end"][2]) > 13 + 1e-9]
        self.assertEqual(high, [high[0], high[0] + 1, high[0] + 2], high)
        self.assertTrue(all(travel[i]["kind"] == "rapid" for i in high))
        # Rapid descents stop at the feed height; below it the tool feeds.
        for m in moves:
            a, b = m["curve"]["start"], m["curve"]["end"]
            if m["kind"] == "rapid" and b[2] < a[2]:
                self.assertGreaterEqual(b[2], 11 - 1e-9, m)
