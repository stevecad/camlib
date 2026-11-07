"""Benchmark workloads: the regression suite's synthetic parts, at least one per
toolpath, each mapped to its optimization issue.

Every workload is built from the published test fixtures and support
builders (synthetic geometry, verified by the test suite), so a benchmark
number always describes a toolpath whose correctness is tested.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT / "python", ROOT / "python" / "tests"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

# CAD-derived meshes and regions come from committed fixtures (identical
# inputs on every machine, whatever CadQuery/OCP version is installed).
from geometry import install as _freeze_geometry  # noqa: E402

_freeze_geometry()


@dataclass(frozen=True)
class Workload:
    """``build`` returns a ``camlib.simulation.Simulation`` configuration.
    ``toolpaths``: operation id -> (toolpath, optimization issue number).
    ``target`` (optional) returns what the stock is compared with:
    ("solid", cadquery shape), ("surface", SurfaceMesh, base_z) or
    ("region", PlanarRegion, floor_z)."""

    name: str
    family: str
    description: str
    toolpaths: dict
    build: object
    target: object = None
    tags: tuple = field(default=())


def _options(job, setup_id=None):
    """A job's setup as a Simulation configuration (checks removed: the
    benchmark measures planning and simulation, and checks materials itself)."""
    from camlib.jobs import Job

    if isinstance(job, Job):
        setup = job.compile().setup(setup_id or job.setups[0].id)
        config = setup.simulation_options()
    else:
        config = dict(job)
    config.pop("verification", None)
    config.pop("regeneration", None)
    return config


def _support(name):
    import importlib

    return importlib.import_module(f"support.{name}")


def _solid(module, name="cadquery_target"):
    def target():
        shape = getattr(_support(module), name)()
        return ("solid", shape.val() if hasattr(shape, "val") else shape)

    return target


CUT = dict(feed_mm_min=600, plunge_mm_min=150, spindle_rpm=12000, coolant="flood")


def _mill4():
    import camlib
    from camlib.tools import HolderSection, ToolAssembly

    return ToolAssembly(
        "mill4",
        camlib.EndMill("em4", diameter=4, flute_length=20, overall_length=50),
        30,
        60,
        (HolderSection(30, 60, 4, 4),),
    )


def _ball4():
    from camlib.tools import BallEndMill, HolderSection, ToolAssembly

    return ToolAssembly(
        "ball4",
        BallEndMill("be4", diameter=4, flute_length=20, overall_length=50),
        30,
        60,
        (HolderSection(30, 60, 4, 4),),
    )


# --- milling: 2D and holes ------------------------------------------------


def _facing():
    return _options(_support("facing").facing_job())


def _pocket():
    return _options(_support("pocket").cadquery_job())


def _pocket_target():
    config = _pocket()
    op = config["operations"][-1]
    return ("region", op.region, op.floor_z_mm)


def _rest_pocket():
    import importlib.util

    spec = importlib.util.spec_from_file_location("standalone_bench", ROOT / "examples" / "standalone_job.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return _options(module.build_rest_job())


def _rest_target():
    """The faced block (top at Z 5) with the 10 x 8 pocket down to Z 3."""
    import cadquery as cq

    block = cq.Workplane("XY").box(24, 16, 5, centered=False)
    pocket = cq.Workplane("XY").box(10, 8, 2, centered=False).translate((4, 4, 3))
    return ("solid", block.cut(pocket).val())


def _adaptive():
    return _options(_support("adaptive_pocket").adaptive_job())


def _roughing(**kind):
    return lambda: _options(_support("roughing").roughing_job(**kind))


def _roughing_target():
    return ("solid", _support("roughing").part().val())


def _boss_roughing():
    # A fine mesh (0.03 mm; the frozen fixture stays small): roughing's moves
    # and time must not follow its facets (issue #89).
    return _options(_support("filleted_boss").roughing_job(0.03))


def _boss_target():
    # The roughed mesh itself (a default-error export would be a large
    # fixture).
    return ("surface", _boss_roughing()["operations"][0].target, 0)


def _contour():
    return _options(_support("contour").contour_job())


def _holes():
    return _options(_support("holes").hole_job())


def _thread_milling():
    import camlib
    import test_thread_milling as t

    mill, tool = t.tools()
    bore = camlib.Bore(
        "bore",
        mill,
        ((10, 12),),
        12,
        4,
        2,
        12,
        entry_feed_mm_min=150,
        feed_mm_min=600,
        retract_feed_mm_min=600,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    return dict(
        stock=camlib.StockSpec((0, 0, 0), (40, 24, 10), 0.1),
        tools=(mill, tool),
        operations=(bore, t.thread(tool)),
        revision="bench",
        initial_position_mm=(0, 0, 12),
    )


def _reaming():
    import test_drill_cycles as t

    return _options(t.boring_job())


def _tapping():
    import test_tapping as t

    return _options(t.job(t.tapping))


def _slot():
    return _options(_support("trochoidal_slot").slot_job())


def _chamfer():
    return _options(_support("chamfer").chamfer_job())


def _trace():
    import camlib
    import test_trace as t

    tool = _mill4()
    return dict(
        stock=camlib.StockSpec((0, 0, 0), (24, 16, 6), 0.25),
        tools=(tool,),
        operations=(t.trace(tool),),
        revision="bench",
        initial_position_mm=(0, 0, 8),
    )


def _vcarve():
    import camlib
    import test_vcarve as t

    op = camlib.VCarve(
        "carve",
        t.vbit(),
        t.region(),
        10,
        20,
        **CUT,
        tool_number=1,
        length_register=1,
    )
    return _options(t.config(op))


# --- milling: surfaces ------------------------------------------------------


def _mesh(part):
    from camlib.cadquery import export_surface_mesh

    return export_surface_mesh(part(), source=dict(model="bench", revision="1"))


def _stepped():
    import test_flat

    return test_flat.part()


def _flat():
    import camlib

    tool = _mill4()
    op = camlib.Flat("flats", tool, _mesh(_stepped), 0, 1.5, 12, **CUT, tool_number=1, length_register=1)
    return dict(
        stock=camlib.StockSpec((0, 0, 0), (24, 16, 8), 0.25),
        tools=(tool,),
        operations=(op,),
        revision="bench",
        initial_position_mm=(0, 0, 12),
    )


def _stepped_target():
    return ("surface", _mesh(_stepped), 0)


def _pencil():
    import camlib

    tool = _ball4()
    op = camlib.Pencil("pencil", tool, _mesh(_stepped), 0, 12, **CUT, tool_number=1, length_register=1)
    return dict(
        stock=camlib.StockSpec((0, 0, 0), (24, 16, 8), 0.25),
        tools=(tool,),
        operations=(op,),
        revision="bench",
        initial_position_mm=(0, 0, 12),
    )


def _roof():
    return _support("parallel").parallel_job()


def _roof_target():
    config = _roof()
    return ("surface", config["operations"][0].surface, 0)


def _pattern(name):
    def build():
        import test_surface_patterns as t

        config = _options(_roof())
        tool, surface = config["tools"][0], config["operations"][0].surface
        return dict(config, operations=(t.patterns(tool, surface)[name],))

    return build


def _parallel():
    return _options(_roof())


def _morph():
    import camlib

    config = _options(_roof())
    tool, surface = config["tools"][0], config["operations"][0].surface
    a = camlib.TracePath((1, 2), (camlib.TraceLine((23, 2)),))
    b = camlib.TracePath((1, 14), (camlib.TraceLine((23, 10)),))
    op = camlib.Morph("morph", tool, surface, 0, 0.5, 12, a, b, **CUT, tool_number=1, length_register=1)
    return dict(config, operations=(op,))


def _steep_and_shallow():
    import camlib

    config = _options(_roof())
    tool, surface = config["tools"][0], config["operations"][0].surface
    steep, shallow = camlib.steep_and_shallow(
        "finish",
        tool,
        surface,
        0,
        12,
        top_z_mm=7.5,
        bottom_z_mm=0.5,
        stepdown_mm=0.5,
        stepover_mm=0.75,
        threshold_deg=30,
        overlap_deg=5,
        **CUT,
        tool_number=1,
        length_register=1,
    )
    return dict(config, operations=(steep, shallow))


def _rough_finish():
    return _options(_support("parallel").build_finishing_job(finish="parallel"))


def _rough_finish_target():
    return ("solid", _support("parallel").target().val())


def _waterline():
    return _options(_support("waterline").waterline_job())


# --- production-density variants (engine performance at realistic density) ---


def _dense(build, stock_max=None, **changes):
    """The same workload with finer parameters on every operation that has
    them (and optionally a larger stock), for planning-time measurement."""

    def dense():
        config = build()
        ops = []
        for op in config["operations"]:
            fields = getattr(op, "__dataclass_fields__", {})
            ops.append(replace(op, **{k: v for k, v in changes.items() if k in fields}))
        config = dict(config, operations=tuple(ops))
        if stock_max is not None:
            config["stock"] = replace(config["stock"], max_mm=stock_max)
        return config

    return dense


# --- thermal ----------------------------------------------------------------


def _plasma():
    import test_thermal as t

    return _options(t.job())


# --- turning ----------------------------------------------------------------


def _lathe(**changes):
    def build():
        import test_turning as t

        return _options(t.job(**changes).job)

    return build


def _lathe_groove():
    import camlib
    from test_turning import cnmg, tool

    groove = tool("groove2", camlib.GrooveInsert(2, 22), nose_radius_mm=0.2, width_mm=2, length_mm=60)
    profile = (
        (0, 0, 0),
        (10, 0, 0),
        (10, -20, 0),
        (15, -20, 0),
        (15, -25, 0),
        (12, -25, 0),
        (12, -28, 0),
        (15, -28, 0),
        (15, -40, 0),
    )
    return _lathe(
        profile=profile,
        tools=(camlib.TurnToolSetting(cnmg(), 1, 1), camlib.TurnToolSetting(groove, 2, 2)),
        operations=(
            camlib.TurnFace("face", "cnmg", 0, feed_mm_rev=0.15, rpm=1000),
            camlib.TurnRough("rough", "cnmg", feed_mm_rev=0.25, rpm=1000),
            camlib.TurnGroove("groove", "groove2", -25, -28, 12, peck_mm=1, feed_mm_rev=0.05, rpm=800),
            camlib.TurnPartOff("part", "groove2", -40, feed_mm_rev=0.05, rpm=800),
        ),
    )()


def _lathe_thread():
    import camlib
    from test_turning import cnmg, tool

    thread = tool("thread60", camlib.ThreadInsert(60, 3), nose_radius_mm=0.1, width_mm=16, length_mm=60)
    return _lathe(
        tools=(camlib.TurnToolSetting(cnmg(), 1, 1), camlib.TurnToolSetting(thread, 2, 2)),
        operations=(
            camlib.TurnFace("face", "cnmg", 0, feed_mm_rev=0.15, rpm=1000),
            camlib.TurnRough("rough", "cnmg", feed_mm_rev=0.25, rpm=1000),
            camlib.TurnFinish("finish", "cnmg", feed_mm_rev=0.1, rpm=1200),
            camlib.TurnThread("thread", "thread60", -20, -38, 15, 2, 1.227, rpm=600),
        ),
    )()


def _lathe_bore():
    import camlib
    from test_turning import tool

    cone = 8 / math.tan(math.radians(59))
    drill = tool("drill16", camlib.DrillInsert(16, 118, 50), nose_radius_mm=0, width_mm=40, length_mm=60)
    bar = tool(
        "bar",
        camlib.TurningInsert.iso(included_angle_deg=55, approach_angle_deg=95, edge_length_mm=6, nose_radius_mm=0.4),
        nose_radius_mm=0.4,
        width_mm=10,
        length_mm=80,
        offset_mm=1,
        side="inner",
    )
    return _lathe(
        stock=camlib.TurnStockSpec(-60, 0, 20),
        profile=((0, 0, 0), (18, 0, 0), (18, -40, 0)),
        bore=((11, 0, 0), (11, -25, 0), (8, -25, 0), (0, -25 - cone, 0)),
        tools=(camlib.TurnToolSetting(drill, 1, 1), camlib.TurnToolSetting(bar, 2, 2)),
        operations=(
            camlib.TurnDrill("drill", "drill16", -25 - cone, peck_mm=10, feed_mm_rev=0.12, rpm=900),
            camlib.TurnBoreRough("bore_rough", "bar", -24, feed_mm_rev=0.15, rpm=1000),
            camlib.TurnBoreFinish("bore_finish", "bar", -24, feed_mm_rev=0.08, rpm=1200),
        ),
    )()


def _lathe_face_groove():
    import camlib
    from test_turning import cnmg, tool

    blade = tool("fg", camlib.FaceGrooveInsert(2, 8), nose_radius_mm=0.2, width_mm=2, length_mm=40)
    return _lathe(
        stock=camlib.TurnStockSpec(-60, 1, 13),
        profile=((0, 0, 0), (12, 0, 0), (12, -40, 0)),
        tools=(camlib.TurnToolSetting(cnmg(), 1, 1), camlib.TurnToolSetting(blade, 8, 8)),
        face_grooves=(camlib.FaceGroove(4, 7, -3),),
        operations=(
            camlib.TurnFace("face", "cnmg", 0, feed_mm_rev=0.15, rpm=1000),
            camlib.TurnFaceGroove("fg", "fg", 4, 7, -3, peck_mm=1, feed_mm_rev=0.03, rpm=600),
        ),
    )()


WORKLOADS = (
    Workload("facing", "mill", "Face a 40 x 24 block 2 mm", {"face": ("Facing", 24)}, _facing, _solid("facing")),
    Workload(
        "pocket",
        "mill",
        "Pocket around a round island (CAD region)",
        {"clear-pocket": ("Pocket", 25)},
        _pocket,
        _pocket_target,
    ),
    Workload(
        "rest-pocket",
        "mill",
        "Face, 4 mm pocket, 2 mm rest pocket",
        {"face": ("Facing", 24), "pocket": ("Pocket", 25), "rest": ("RestPocket", 25)},
        _rest_pocket,
        _rest_target,
    ),
    Workload(
        "adaptive-pocket",
        "mill",
        "Adaptive clearing of an L pocket",
        {"adaptive": ("AdaptivePocket", 26)},
        _adaptive,
        _solid("adaptive_pocket", "target"),
    ),
    Workload(
        "roughing-raster",
        "mill",
        "3D raster roughing around a frustum",
        {"rough": ("Roughing", 27)},
        _roughing(),
        _roughing_target,
    ),
    Workload(
        "roughing-offset",
        "mill",
        "3D offset roughing around a frustum",
        {"rough": ("Roughing", 27)},
        _roughing(offset=True),
        _roughing_target,
    ),
    Workload(
        "roughing-adaptive",
        "mill",
        "3D adaptive roughing around a frustum",
        {"rough": ("Roughing", 27)},
        _roughing(adaptive=True),
        _roughing_target,
    ),
    Workload(
        "roughing-fine-mesh",
        "mill",
        "3D offset roughing of a filleted boss from a fine mesh",
        {"rough": ("Roughing", 27)},
        _boss_roughing,
        _boss_target,
    ),
    Workload(
        "contour",
        "mill",
        "Outer and inner walls of an annulus",
        {"profile": ("Contour", 28)},
        _contour,
        _solid("contour"),
    ),
    Workload(
        "holes",
        "mill",
        "Pilot drilling then helical bore milling",
        {"pilot": ("Drilling", 29), "finish-holes": ("Bore", 30)},
        _holes,
        _solid("holes"),
    ),
    Workload(
        "reaming", "mill", "Drill then ream two holes", {"drill": ("Drilling", 29), "bore": ("Reaming", 31)}, _reaming
    ),
    Workload(
        "tapping",
        "mill",
        "Drill then rigid-tap two holes",
        {"drill": ("Drilling", 29), "tap": ("Tapping", 32)},
        _tapping,
    ),
    Workload(
        "thread-milling",
        "mill",
        "Bore then thread-mill M14 x 1.5",
        {"bore": ("Bore", 30), "thread": ("ThreadMilling", 33)},
        _thread_milling,
    ),
    Workload(
        "trochoidal-slot",
        "mill",
        "Straight trochoidal slot",
        {"slot": ("TrochoidalSlot", 34)},
        _slot,
        _solid("trochoidal_slot"),
    ),
    Workload(
        "chamfer",
        "mill",
        "Chamfer the top edges of an annulus",
        {"rough-profile": ("Contour", 28), "top-bevel": ("Chamfer", 35)},
        _chamfer,
        _solid("chamfer"),
    ),
    Workload("trace", "mill", "Engrave a rectangle and a hook", {"engrave": ("Trace", 36)}, _trace),
    Workload("vcarve", "mill", "V-carve an L-shaped letter", {"carve": ("VCarve", 37)}, _vcarve),
    Workload("flat", "mill", "Flat areas of a stepped part", {"flats": ("Flat", 38)}, _flat, _stepped_target),
    Workload(
        "rough-parallel",
        "mill",
        "Rough then parallel ball finish a roof",
        {"rough": ("Roughing", 27), "parallel": ("Parallel", 39)},
        _rough_finish,
        _rough_finish_target,
    ),
    Workload(
        "parallel", "mill", "Parallel finishing of a roof", {"finish-roof": ("Parallel", 39)}, _parallel, _roof_target
    ),
    Workload(
        "spiral", "mill", "Spiral finishing of a roof", {"spiral": ("Spiral", 40)}, _pattern("spiral"), _roof_target
    ),
    Workload(
        "radial", "mill", "Radial finishing of a roof", {"radial": ("Radial", 41)}, _pattern("radial"), _roof_target
    ),
    Workload(
        "project", "mill", "Project curves onto a roof", {"project": ("Project", 42)}, _pattern("project"), _roof_target
    ),
    Workload("morph", "mill", "Morph between two edges of a roof", {"morph": ("Morph", 43)}, _morph, _roof_target),
    Workload(
        "scallop",
        "mill",
        "Scallop finishing of a roof",
        {"scallop": ("Scallop", 44)},
        _pattern("scallop"),
        _roof_target,
    ),
    Workload(
        "pencil",
        "mill",
        "Pencil the inside corner of a stepped part",
        {"pencil": ("Pencil", 45)},
        _pencil,
        _stepped_target,
    ),
    Workload(
        "waterline",
        "mill",
        "Waterline finishing of a tapered island",
        {"finish-levels": ("Waterline", 46)},
        _waterline,
        _solid("waterline", "target"),
    ),
    Workload(
        "steep-and-shallow",
        "mill",
        "Steep-and-shallow finishing of a roof",
        {"finish-steep": ("Steep-and-shallow", 47), "finish-shallow": ("Steep-and-shallow", 47)},
        _steep_and_shallow,
        _roof_target,
    ),
    Workload(
        "facing-large",
        "mill",
        "Face a 300 x 200 block 2 mm (production size)",
        {"face": ("Facing", 24)},
        _dense(_facing, stock_max=(300, 200, 10)),
        tags=("dense",),
    ),
    Workload(
        "pocket-dense",
        "mill",
        "Pocket around a round island, 0.4 mm stepover, 0.25 mm stepdown",
        {"clear-pocket": ("Pocket", 25)},
        _dense(_pocket, stepover_mm=0.4, stepdown_mm=0.25),
        _pocket_target,
        tags=("dense",),
    ),
    Workload(
        "parallel-dense",
        "mill",
        "Parallel finishing of a roof, 0.1 mm stepover",
        {"finish-roof": ("Parallel", 39)},
        _dense(_parallel, stepover_mm=0.1),
        _roof_target,
        tags=("dense",),
    ),
    Workload(
        "spiral-dense",
        "mill",
        "Spiral finishing of a roof, 0.15 mm stepover",
        {"spiral": ("Spiral", 40)},
        _dense(_pattern("spiral"), stepover_mm=0.15),
        _roof_target,
        tags=("dense",),
    ),
    Workload(
        "scallop-dense",
        "mill",
        "Scallop finishing of a roof, 0.15 mm stepover",
        {"scallop": ("Scallop", 44)},
        _dense(_pattern("scallop"), stepover_mm=0.15),
        _roof_target,
        tags=("dense",),
    ),
    Workload(
        "waterline-dense",
        "mill",
        "Waterline finishing of a tapered island, 0.1 mm stepdown",
        {"finish-levels": ("Waterline", 46)},
        _dense(_waterline, stepdown_mm=0.1),
        _solid("waterline", "target"),
        tags=("dense",),
    ),
    Workload(
        "plasma",
        "thermal",
        "Plasma-cut a plate with holes and a nested washer",
        {"plasma": ("ThermalCut", 48)},
        _plasma,
    ),
    Workload(
        "lathe-shaft",
        "lathe",
        "Face, rough and finish a stepped shaft",
        {"face": ("TurnFace", 49), "rough": ("TurnRough", 50), "finish": ("TurnFinish", 51)},
        _lathe(),
    ),
    Workload(
        "lathe-groove",
        "lathe",
        "Rough, groove and part off a shaft",
        {
            "face": ("TurnFace", 49),
            "rough": ("TurnRough", 50),
            "groove": ("TurnGroove", 52),
            "part": ("TurnPartOff", 53),
        },
        _lathe_groove,
    ),
    Workload(
        "lathe-thread",
        "lathe",
        "Turn and single-point thread M30 x 2",
        {
            "face": ("TurnFace", 49),
            "rough": ("TurnRough", 50),
            "finish": ("TurnFinish", 51),
            "thread": ("TurnThread", 54),
        },
        _lathe_thread,
    ),
    Workload(
        "lathe-bore",
        "lathe",
        "Drill on the axis, rough and finish bore",
        {"drill": ("TurnDrill", 55), "bore_rough": ("TurnBore", 56), "bore_finish": ("TurnBore", 56)},
        _lathe_bore,
    ),
    Workload(
        "lathe-face-groove",
        "lathe",
        "Face and cut a face groove",
        {"face": ("TurnFace", 49), "fg": ("TurnFaceGroove", 57)},
        _lathe_face_groove,
    ),
)

BY_NAME = {w.name: w for w in WORKLOADS}
