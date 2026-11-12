"""camlib: a three-axis CAM engine with a Rust core and a Python API.

The common names are available from the top level::

    import camlib
    job = camlib.Job("bracket", "1", (camlib.Setup(...),))

They load on first use (``import camlib`` stays cheap and never imports
CadQuery). Submodules hold the rest: ``camlib.cadquery`` (CAD export),
``camlib.post`` (machines and posts), ``camlib.verification``, and so on.
``Setup`` is the job setup (``camlib.jobs.Setup``).
"""

from __future__ import annotations

import importlib as _importlib
import json

_EXPORTS = {
    **dict.fromkeys(
        (
            'PlanningCandidate',
            'OperationChoices',
            'ComparisonPolicy',
            'ComparisonReport',
            'CorrectionTemplate',
            'CorrectionReport',
            'correction_candidate',
            'correct_residuals',
            'recipe_candidates',
            'compare_candidates',
        ),
        'planning',
    ),
    **dict.fromkeys(("Job", "Setup", "OperationNode", "RegenerationCache", "CadSource", "PostedProgram"), "jobs"),
    **dict.fromkeys(
        (
            "Facing",
            "Pocket",
            "RestPocket",
            "AdaptivePocket",
            "Roughing",
            "RestRoughing",
            "Contour",
            "Tabs",
            "VCarve",
            "Chamfer",
            "Deburr",
            "DeburrLevel",
            "DeburrBrokenEdge",
            "DeburrEdge",
            "Drilling",
            "Bore",
            "TrochoidalSlot",
            "Parallel",
            "Waterline",
            "HelicalEntry",
            "RampEntry",
            "StockEdgeEntry",
            "RasterClearing",
            "OffsetClearing",
            "AdaptiveClearing",
            "Trace",
            "Undercut",
            "TracePath",
            "TraceLine",
            "TraceArc",
            "TraceSideOffset",
            "Spiral",
            "Radial",
            "Project",
            "Morph",
            "ThreadMilling",
            "Tapping",
            "Reaming",
            "FineBore",
            "ThermalCut",
            "Lead",
            "SmallHoles",
            "MicroJoints",
            "Flat",
            "Pencil",
            "Scallop",
            "Geodesic",
            "steep_and_shallow",
            "ProbeSurface",
            "ProbeOutsideCorner",
            "ProbeInsideCorner",
            "ProbeBore",
            "ProbeBoss",
            "ProbePocket",
            "ProbeWeb",
            "ToolSetLength",
            "ToolSetDiameter",
            "SuppliedToolpath",
            "ToolpathMove",
        ),
        "operations",
    ),
    **dict.fromkeys(
        (
            "EndMill",
            "BallEndMill",
            "BullEndMill",
            "Drill",
            "ChamferMill",
            "ThreadMill",
            "TSlotCutter",
            "KeyseatCutter",
            "LollipopCutter",
            "DovetailCutter",
            "BoringBar",
            "Torch",
            "LaserHead",
            "Tap",
            "ProbeTool",
            "ToolAssembly",
            "HolderSection",
        ),
        "tools",
    ),
    **dict.fromkeys(
        (
            "Simulation",
            "StockSpec",
            "InitialStock",
            "StockMesh",
            "DexelStockSpec",
            "SweepOptions",
            "SimulationError",
            "ReachRefusal",
            "ReachRefused",
            "CamWarning",
            "ToolChangeInPlace",
            "ToolChangePosition",
            "ToolChangeTable",
        ),
        "simulation",
    ),
    **dict.fromkeys(("Player", "Cursor", "RecordedRun"), "playback"),
    "DexelStockState": "dexel_state",
    **dict.fromkeys(("Limits", "AvoidArea"), "limits"),
    **dict.fromkeys(
        ("Machine", "ToolSetting", "LoweringOptions", "GCodeDialect", "GCodePost", "NativeGCodeDecoder", "post_events"),
        "post",
    ),
    **dict.fromkeys(
        ("StreamChecks", "StockContactOptions", "Scene", "Prism", "MaterialCheck", "MachineLimits"), "verification"
    ),
    "SurfaceMesh": "surface",
    **dict.fromkeys(
        (
            "TurningSetup",
            "TurnStockSpec",
            "TurningTool",
            "TurningInsert",
            "InsertOutline",
            "TurningHolder",
            "TurningBar",
            "TurningSeat",
            "SolidBarHead",
            "TurningToolModel",
            "TurningModelPlacement",
            "TurningModelInsertCheck",
            "TurningModelWithoutInsert",
            "GrooveInsert",
            "ThreadInsert",
            "TurnToolSetting",
            "TurnFace",
            "TurnRough",
            "TurnFinish",
            "TurnGroove",
            "TurnPartOff",
            "TurnThread",
            "TurnDrill",
            "DrillInsert",
            "TurnBoreRough",
            "TurnBoreFinish",
            "FaceGroove",
            "FaceGrooveInsert",
            "TurnFaceGroove",
        ),
        "turning",
    ),
    **dict.fromkeys(("PlanarRegion", "Boundary", "Component"), "planar"),
    "WorkFrame": "motion",
    **dict.fromkeys(("cutting_data", "CuttingData"), "cutting"),
    **dict.fromkeys(
        ('EngagementFeedBand', 'EngagementFeedPolicy', 'EngagementScheduledProgram', 'schedule_engagement'), 'cutting'
    ),
}

__all__ = ["Engine", *sorted(_EXPORTS)]


def __getattr__(name):
    module = _EXPORTS.get(name)
    if module is None:
        raise AttributeError(f"module 'camlib' has no attribute {name!r}")
    value = getattr(_importlib.import_module(f".{module}", __name__), name)
    globals()[name] = value
    return value


def __dir__():
    return sorted(set(globals()) | set(_EXPORTS))


class Engine:
    """The region/geometry interface: fixtures, planar regions, tool inspection,
    program validation and motion verification on one in-process native session."""

    def __init__(self):
        from ._buffers import native

        self._session = native.RegionSession()

    def _request(self, command, **fields):
        if self._session is None:
            raise RuntimeError("engine closed")
        return json.loads(self._session.request(json.dumps({"command": command, **fields}, allow_nan=False)))

    def load(self, fixture):
        return self._request("load", fixture=fixture)

    def probe(self, points):
        return self._request("probe", points=points)

    def classify(self, radius_mm):
        return self._request("classify", radius_mm=radius_mm)["results"]

    def set_feed(self, feed_mm_min):
        return self._request("set_feed", feed_mm_min=feed_mm_min)

    def stats(self):
        return self._request("stats")

    def inspect_tool(self, tool, heights_mm):
        """Validate/prepare a tool and query its tip-relative cutting/outer profile."""
        return self._request("inspect_tool", tool=tool.to_dict(), heights_mm=list(heights_mm))

    def validate_program(self, program, capabilities=None):
        """Validate program geometry/state/capabilities; this is not collision verification."""
        return self._request("validate_program", program=program.to_dict(), capabilities=capabilities)

    def prepare_region(self, region, *, tolerance_mm=1e-5):
        data = region.to_dict() if hasattr(region, "to_dict") else region
        return self._request("prepare_region", region=data, options={"tolerance_mm": tolerance_mm})

    def offset_region(self, delta_mm):
        """Positive delta erodes prepared filled area; negative delta expands it."""
        return self._request("offset_region", delta_mm=delta_mm)

    def cutter_region(self, radius_mm, allowance_mm=0):
        return self._request("cutter_region", radius_mm=radius_mm, allowance_mm=allowance_mm)

    def boolean_region(self, other, operation, *, tolerance_mm=1e-5):
        data = other.to_dict() if hasattr(other, "to_dict") else other
        return self._request("boolean_region", other=data, operation=operation, options={"tolerance_mm": tolerance_mm})

    def query_region(self, points):
        return self._request("query_region", points=points)["results"]

    def silhouette_region(self, surface, z_mm, *, max_error_mm, tolerance_mm=1e-5):
        """The plan-view silhouette of a ``SurfaceMesh`` at a depth: the shadow
        of everything at or above ``z_mm``, as a planar region for 2D operations
        (``Contour(region=...)`` at that floor).

        Returns ``{"region": ..., "report": ...}``. The region's
        ``boundary_error_mm`` (the report's too) bounds how far the intended
        surface at or above ``z_mm`` may lie outside it: the mesh's declared
        error, a numerical reserve and the enclosure of vertical walls.
        Operations add it to their clearance. Refused when that bound exceeds
        ``max_error_mm``, when nothing is at or above ``z_mm``, or when the
        facets' projections cannot be joined unambiguously."""
        data = surface.to_dict() if hasattr(surface, "to_dict") else surface
        return self._request(
            "silhouette_region",
            surface=data,
            z_mm=z_mm,
            max_error_mm=max_error_mm,
            options={"tolerance_mm": tolerance_mm},
        )

    def close(self):
        self._session = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
