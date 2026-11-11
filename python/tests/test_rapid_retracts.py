"""Rapid retracts: a straight-up retract from the end of a cut is posted as a
rapid and the stock-contact check proves it clear."""

import unittest

import camlib
from camlib.simulation import Update, VerificationUpdate
from test_limits import square, tool


def job(rapid, holes=False):
    t = tool()
    ops = [
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
        operations=tuple(ops[:1]) if holes else tuple(ops),
        initial_position_mm=(0, 0, 20),
        verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
        rapid_retracts=rapid,
    )
    return camlib.Job("retracts", "1", (setup,))


def run(j):
    moves, statuses = [], {}
    with j.regenerate() as r:
        for u in r:
            if isinstance(u.update, Update):
                moves.append(u.update.motion)
            elif isinstance(u.update, VerificationUpdate):
                statuses[u.update.status] = statuses.get(u.update.status, 0) + 1
    return moves, statuses


class RapidRetractTest(unittest.TestCase):
    def test_retracts_become_rapids_and_check_clear(self):
        slow, slow_status = run(job(False))
        fast, fast_status = run(job(True))
        self.assertEqual(set(slow_status), {"clear"})
        self.assertEqual(set(fast_status), {"clear"}, fast_status)
        retracts = [m for m in slow if m["kind"] == "retract"]
        self.assertTrue(retracts)
        self.assertEqual([m for m in fast if m["kind"] == "retract"], [])
        # Same path, same count: only the kind and feed of the retracts change.
        self.assertEqual([m["curve"] for m in slow], [m["curve"] for m in fast])
        self.assertEqual(
            sum(1 for m in fast if m["kind"] == "rapid"), sum(1 for m in slow if m["kind"] == "rapid") + len(retracts)
        )


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
            # This test is of the orbit planner's links.
            path_strategy="disk_front",
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
        # Only the first and last moves reach clearance; travel is at Z13.
        self.assertTrue(all(max(m["curve"]["start"][2], m["curve"]["end"][2]) <= 13 + 1e-9 for m in inner[1:]))
        # Rapid descents stop at the feed height; below it the tool feeds.
        for m in moves:
            a, b = m["curve"]["start"], m["curve"]["end"]
            if m["kind"] == "rapid" and b[2] < a[2]:
                self.assertGreaterEqual(b[2], 11 - 1e-9, m)
