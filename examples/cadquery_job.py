"""Ordinary CadQuery -> native CAM -> streamed stock -> checked Python post.

Install camlib's platform wheel into your CadQuery Python, then run this file.
No cadquery document server, frontend or source-checkout imports are needed.
"""

import argparse
from dataclasses import replace
import json
import math
from pathlib import Path

import cadquery as cq

from camlib.cadquery import export_floor_region
from camlib.jobs import CadSource, Job, OperationNode, RegenerationCache, Setup
from camlib.operations import Facing, Pocket, RestPocket
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


def build_job(width=10, *, revision=1, feed_mm_min=600):
    part = (
        cq.Workplane("XY")
        .box(24, 16, 5, centered=False)
        .cut(cq.Workplane("XY").box(width, 8, 2, centered=False).translate((4, 4, 3)))
    )
    floors = [
        f
        for f in part.val().Faces()
        if f.geomType() == "PLANE" and f.normalAt().z > 0.999 and abs(f.Center().z - 3) < 1e-7
    ]
    if len(floors) != 1:
        raise ValueError("the pocket floor must resolve to one horizontal CAD face")
    source = CadSource.from_shape("part", revision, part, reference="pocket-floor")
    region = export_floor_region(
        floors[0], name="pocket-floor", source=dict(document_id="part", revision=str(revision))
    )
    tool = ToolAssembly(
        "mill4", EndMill("em4", diameter=4, flute_length=20, overall_length=50), 30, 60, (HolderSection(30, 60, 4, 4),)
    )
    face = Facing("face", tool, 5, 1, 2, 8, **CUTTING, tool_number=7, length_register=9)
    pocket = Pocket(
        "pocket",
        tool,
        region,
        5,
        3,
        1,
        1.5,
        8,
        **{**CUTTING, "feed_mm_min": feed_mm_min},
        tool_number=7,
        length_register=9,
    )
    checks = StreamChecks(stock_contact=StockContactOptions())
    setup = Setup(
        "top",
        StockSpec((0, 0, 0), (24, 16, 6), 0.5),
        (tool,),
        (OperationNode(face, sources=()), pocket),
        (0, 0, 8),
        sources=(source,),
        verification=checks,
    )
    return Job("cadquery-pocket", str(revision), (setup,))


def machine_for(job):
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
    """Face, clear with a 4 mm mill, then cut remaining stock with a smaller mill."""
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
        5,
        3,
        1,
        small_diameter * 0.375,
        8,
        feed_z_mm=6.5,  # rapid down to just above the stock (top 6)
        **CUTTING,
        tool_number=8,
        length_register=10,
    )
    return replace(
        job,
        id="cadquery-rest",
        setups=(replace(setup, tools=(large, small), operations=(setup.operations[0], rough, rest)),),
    )


def rest_machine_for(job):
    """Example assumes each tool change returns the compensated tip to its prior pose."""
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
    parser = argparse.ArgumentParser(description=__doc__)
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
    cache = RegenerationCache(args.output / "cache")
    machine = rest_machine_for(job) if args.rest else machine_for(job)
    with job.post("top", machine=machine, cache=cache) as posted:
        posted.wait()
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
