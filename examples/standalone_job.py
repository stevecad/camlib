"""Standalone milling job: face a block, cut a pocket, write checked G-code.

Needs only camlib (no CAD package). Run:

    python standalone_job.py output-dir --accept-unchecked-machine  # inspection only
    python standalone_job.py output-dir --rest --accept-unchecked-machine

All lengths are millimetres. Z is absolute in the work coordinate system:
the stock occupies Z 0..6, so its top face is Z 6.
"""

import argparse
from dataclasses import replace
import json
import math
from pathlib import Path

from camlib.jobs import Job, OperationNode, RegenerationCache, Setup
from camlib.operations import Facing, Pocket, RestPocket
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.playback import Cursor, Player
from camlib.post import GCodeDialect, Machine, ToolSetting
from camlib.simulation import StockSpec
from camlib.tools import EndMill, HolderSection, ToolAssembly
from camlib.verification import StockContactOptions, StreamChecks


# Cutting data has no defaults: it depends on the tool, material and machine.
# These suit a 4 mm (and 2 mm) carbide end mill in aluminium on a small mill;
# camlib.cutting.cutting_data can compute them from chip load and surface speed.
CUTTING = dict(feed_mm_min=600, plunge_mm_min=150, spindle_rpm=12000, coolant="flood")


# The example machine. camlib assumes nothing about a machine: these describe a
# small three-axis LinuxCNC mill. Replace them with your controller and limits.
MACHINE = dict(
    dialect=GCodeDialect.linuxcnc(),
    output_units="mm",
    decimals=3,
    max_feed_mm_min=10000,
    max_spindle_rpm=30000,
    rapid_mm_min=6000,
    planes=("xy", "xz", "yz"),
    arcs=True,
    helices=True,
    max_sweep_rad=math.tau,
    max_tool_number=99999,
    max_length_register=99999,
    tool_changes=True,
    flood=True,
    mist=True,
)


def end_mill_assembly(id, diameter):
    """A flat end mill held 30 mm out of a 4 mm radius holder (gauge length 60 mm)."""
    return ToolAssembly(
        id,
        EndMill(f"em{diameter:g}", diameter=diameter, flute_length=20, overall_length=50),
        stickout_mm=30,
        gauge_length_mm=60,
        holders=(HolderSection(z_min_mm=30, z_max_mm=60, radius_min_mm=4, radius_max_mm=4),),
    )


def build_job(width=10, *, revision=1, feed_mm_min=600):
    """Face the 24 x 16 x 6 block down 1 mm, then pocket a width x 8 mm area 2 mm deep."""
    pocket_area = PlanarRegion(
        "pocket-floor", (Component(Boundary.polygon(((4, 4), (4 + width, 4), (4 + width, 12), (4, 12)))),)
    )
    tool = end_mill_assembly("mill4", 4)
    face = Facing(
        "face",
        tool,
        floor_z_mm=5,
        stepdown_mm=1,
        stepover_mm=2,
        clearance_z_mm=8,
        **CUTTING,
        tool_number=7,
        length_register=9,
    )
    pocket = Pocket(
        "pocket",
        tool,
        pocket_area,
        top_z_mm=5,
        floor_z_mm=3,
        stepdown_mm=1,
        stepover_mm=1.5,
        clearance_z_mm=8,
        **{**CUTTING, "feed_mm_min": feed_mm_min},
        tool_number=7,
        length_register=9,
    )
    # Check every streamed move against the evolving stock (rapids must not touch it).
    checks = StreamChecks(stock_contact=StockContactOptions())
    setup = Setup(
        "top",
        stock=StockSpec(min_mm=(0, 0, 0), max_mm=(24, 16, 6), cell_size_mm=0.5),
        tools=(tool,),
        # Facing has no CAD source dependency (sources=()).
        operations=(OperationNode(face, sources=()), pocket),
        initial_position_mm=(0, 0, 8),
        verification=checks,
    )
    return Job("standalone-pocket", str(revision), (setup,))


def machine_for(job):
    """Example machine: tool 7 in the pocket, length offset H9.

    The tool-change pose is an explicit premise here, not read from a machine.
    """
    header = job.compile().setup("top").program_header()
    return Machine(
        "Example three-axis mill",
        (
            ToolSetting.for_tool(
                header.tools[0], number=7, length_register=9, position_after_mm=header.initial_position_mm
            ),
        ),
        **MACHINE,
        tool_change_position_mode="table",
    )


def build_rest_job(width=10, *, revision=1, rough_floor=3, small_diameter=2):
    """Face, clear with a 4 mm mill, then cut only the remaining stock with a smaller mill."""
    job = build_job(width, revision=revision)
    setup = job.setups[0]
    large = setup.tools[0]
    small = replace(
        large, id="mill2", cutter=EndMill("rest-mill", diameter=small_diameter, flute_length=20, overall_length=50)
    )
    rough = replace(setup.operations[1], floor_z_mm=rough_floor)
    rest = RestPocket(
        "rest",
        small,
        rough.region,
        top_z_mm=5,
        floor_z_mm=3,
        stepdown_mm=1,
        stepover_mm=small_diameter * 0.375,
        clearance_z_mm=8,
        # Rapid down to just above the stock (top 6): leaving a cleared area
        # and plunging again costs little, so links stay at depth only where
        # that is actually faster.
        feed_z_mm=6.5,
        **CUTTING,
        tool_number=8,
        length_register=10,
    )
    return replace(
        job,
        id="standalone-rest",
        setups=(replace(setup, tools=(large, small), operations=(setup.operations[0], rough, rest)),),
    )


def rest_machine_for(job):
    """Two tools; assumes each tool change returns the compensated tip to its prior pose."""
    header = job.compile().setup("top").program_header()
    return Machine(
        "Example two-tool mill",
        tuple(
            ToolSetting.for_tool(
                tool, number=number, length_register=length, position_after_mm=header.initial_position_mm
            )
            for tool, number, length in zip(header.tools, (7, 8), (9, 10))
        ),
        tool_change_position_mode="preserve_tip",
        **MACHINE,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("output", type=Path)
    parser.add_argument("--rest", action="store_true", help="run the two-tool rest job")
    parser.add_argument(
        "--accept-unchecked-machine",
        action="store_true",
        help="export this synthetic example for inspection without a physical machine model",
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    job = build_rest_job() if args.rest else build_job()
    cache = RegenerationCache(args.output / "cache")  # reuses unchanged operations next run
    machine = rest_machine_for(job) if args.rest else machine_for(job)
    with job.post("top", machine=machine, cache=cache) as posted:
        posted.wait()
        # Replay to the end to read final stock and check results.
        player = Player(posted)
        player.seek(Cursor(posted.status()["available"]))
        # Written only if every check result is clear (or accepted on purpose);
        # otherwise camlib.export.ExportRefused names the findings.
        exported = posted.export(
            args.output / "pocket.nc",
            accept={"unchecked_machine"} if args.accept_unchecked_machine else (),
        )
        print(
            json.dumps(
                dict(
                    motions=player.state.completed,
                    stock=player.state.volume(),
                    checks=player.verification,
                    output=exported["path"],
                    unchecked=exported["unchecked"],
                ),
                indent=2,
            )
        )


if __name__ == "__main__":
    main()
