"""Motion checks hold a program to the machine's declared limits and name
them (#429): no bound of camlib's own stands in for a machine fact."""

import math
import unittest

import camlib
from camlib.post import ToolSetting
from test_turning import job
from test_turning_second_chucking import example


def facing(feed_mm_rev, rpm=2500):
    """A lathe facing job at a feed per revolution."""
    return job(
        feed_per_rev=True,
        operations=(camlib.TurnFace("face", "cnmg", 0, feed_mm_rev=feed_mm_rev, rpm=rpm),),
    )


def posted(lathe, name, max_feed_mm_min=10000):
    module = example(name)
    fanuc = "fanuc" in name
    header = lathe.job.compile().setups[0].program_header()
    tools = tuple(
        ToolSetting.for_tool(t, number=i, length_register=i, position_after_mm=header.initial_position_mm)
        for i, t in enumerate(header.tools, 1)
    )
    machine = module.define_machine(
        "Lathe",
        tools,
        output_units="mm",
        decimals=3,
        max_feed_mm_min=max_feed_mm_min,
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
        **(dict(program_number=1001) if fanuc else {}),
    )
    return lathe.job.post_nc(
        "lathe",
        machine=machine,
        post=module.FanucLathePost if fanuc else module.CentroidLathePost,
        decoder=module.FanucLatheDecoder if fanuc else module.CentroidLatheDecoder,
        accept={"unchecked_machine"},
    )


class MachineLimitTest(unittest.TestCase):
    def test_a_feed_per_revolution_is_held_to_the_declared_maximum_feed(self):
        # A feed per minute typed as a feed per revolution: 250 mm/rev. It
        # plans (no machine is declared there), and posting refuses it by
        # the machine's maximum feed, in mm/min, naming the field.
        lathe = facing(250)
        lathe.plan()
        expected = (
            r"face: the programmed feed, the feed per revolution 250 mm/rev at 2500 rpm \(the highest along "
            r"the move\), 625000 mm/min, is above the machine's maximum feed 10000 mm/min "
            r"\(Machine\.max_feed_mm_min\) \(move \d+, event \d+\)"
        )
        for name in ("fanuc_lathe_post", "centroid_lathe_post"):
            with self.subTest(post=name), self.assertRaisesRegex(Exception, expected):
                posted(lathe, name)
        # Within it (0.2 mm/rev at 2500 rpm: 500 mm/min), it posts.
        for name in ("fanuc_lathe_post", "centroid_lathe_post"):
            with self.subTest(post=name):
                self.assertIn("F0.2", posted(facing(0.2), name).text)

    def test_a_feed_per_minute_over_the_maximum_names_it(self):
        lathe = job(
            operations=(camlib.TurnFace("face", "cnmg", 0, feed_mm_rev=0.15, rpm=1000),),
        )
        # Feeds per minute: 0.15 mm/rev x 1000 rpm = 150 mm/min, over a
        # declared maximum of 100 mm/min.
        expected = (
            r"face: the programmed feed, 150(\.0)? mm/min, is above the machine's maximum feed 100(\.0)? mm/min "
            r"\(Machine\.max_feed_mm_min\)"
        )
        for name in ("fanuc_lathe_post", "centroid_lathe_post"):
            with self.subTest(post=name), self.assertRaisesRegex(Exception, expected):
                posted(lathe, name, max_feed_mm_min=100)


if __name__ == "__main__":
    unittest.main()
