"""Extract bounded CAM geometry directly from a live CadQuery build.

Planar regions preserve lines/circles and bound ellipse/Bezier/B-spline chords.
The older polygon-fixture exporter retains its line/circle contract. Curved solids
use bounded surface adapters. A floor region does not certify tool access above it.
"""

from __future__ import annotations

import math
import hashlib
from pathlib import Path
import platform
import time
from dataclasses import asdict, dataclass, replace
import io
import json
import re
import sys


def export_surface_mesh(shape, *, source=None, max_error_mm=0.005, max_facets=None, cancel=None):
    """Export CAD solids with explicit source approximation bounds.

    Planar/straight-edged solids retain their exact fast path. Analytic curved
    faces and supported curve extrusions use checked parametric interpolation.
    A mesher's requested deflection alone never supplies the error bound.
    """
    import cadquery as cq
    import OCP
    from OCP.BRep import BRep_Tool
    from .surface import SurfaceMesh

    started = time.perf_counter()
    if not math.isfinite(max_error_mm) or not 1e-6 <= max_error_mm <= 0.1:
        raise ValueError("max_error_mm must be finite in [1e-6, .1]")
    if max_facets is not None and (type(max_facets) is not int or max_facets < 4):
        raise ValueError("max_facets must be None (no cap) or an integer >= 4")
    if cancel is not None and cancel():
        raise ValueError("CAD surface export cancelled")
    if isinstance(shape, cq.Workplane):
        shape = shape.val()
    if not isinstance(shape, cq.Shape) or not shape.isValid() or not shape.Solids():
        raise ValueError("surface export needs valid CadQuery solids")
    faces = shape.Faces()
    if any(face.geomType() != "PLANE" for face in faces) or any(edge.geomType() != "LINE" for edge in shape.Edges()):
        from ._cad_surface import export_curved_solid

        return export_curved_solid(
            shape, max_error_mm=max_error_mm, max_facets=max_facets, source=source, cancel=cancel
        )
    tolerance = max(
        [BRep_Tool.Tolerance_s(v.wrapped) for v in shape.Vertices()]
        + [BRep_Tool.Tolerance_s(e.wrapped) for e in shape.Edges()]
        + [BRep_Tool.Tolerance_s(f.wrapped) for f in faces]
    )
    triangles = []
    for face in faces:
        vertices, indices = face.tessellate(0.005)
        triangles.extend(tuple(tuple(vertices[i].toTuple()) for i in tri) for tri in indices)
        if max_facets is not None and len(triangles) > max_facets:
            raise ValueError("surface exceeds max_facets")
        if cancel is not None and cancel():
            raise ValueError("CAD surface export cancelled")
    if 2 * tolerance + 1e-9 > max_error_mm:
        raise ValueError("CAD topology error exceeds requested surface error")
    return SurfaceMesh(
        triangles,
        2 * tolerance + 1e-9,
        dict(
            source or {},
            adapter="planar_cad_solid_facets_v1",
            cadquery_version=cq.__version__,
            ocp_version=OCP.__version__,
            faces=len(faces),
            triangle_count=len(triangles),
            error_basis="planar faces, straight trim edges, twice CAD topology tolerance plus numerical guard",
            extraction_ms=(time.perf_counter() - started) * 1000,
        ),
    )


def export_hemisphere_surface(face, *, max_error_mm=0.005, max_facets=None, source=None):
    """Bound a selected outward upper hemisphere; its equator is the base Z.

    Full analytic UV trimming is checked. Every facet has a radial Hausdorff
    bound; requested tessellator deflection is never used as evidence.
    """
    import cadquery as cq
    import OCP
    from OCP.BRep import BRep_Tool
    from OCP.BRepAdaptor import BRepAdaptor_Surface, BRepAdaptor_Curve2d
    from OCP.BRepTools import BRepTools_WireExplorer
    from OCP.GeomAbs import GeomAbs_Line
    from OCP.TopAbs import TopAbs_FORWARD
    from .surface import SurfaceMesh

    started = time.perf_counter()
    if isinstance(face, cq.Workplane):
        if face.size() != 1:
            raise ValueError("select exactly one upper-hemisphere face")
        face = face.val()
    if (
        not isinstance(face, cq.Face)
        or not face.isValid()
        or face.geomType() != "SPHERE"
        or face.wrapped.Orientation() != TopAbs_FORWARD
        or len(face.Wires()) != 1
    ):
        raise ValueError("select one valid outward, unholed upper-hemisphere SPHERE face")
    if not math.isfinite(max_error_mm) or not 0 < max_error_mm <= 0.1:
        raise ValueError("max_error_mm must be finite in (0, .1]")
    if max_facets is not None and (type(max_facets) is not int or max_facets < 4):
        raise ValueError("max_facets must be None (no cap) or an integer >= 4")
    adaptor = BRepAdaptor_Surface(face.wrapped)
    sphere = adaptor.Sphere()
    radius = sphere.Radius()
    center = tuple(sphere.Location().Coord())
    axis = tuple(sphere.Position().Direction().Coord())
    axis_deviation = math.dist(axis, (0, 0, 1))
    if not math.isfinite(radius) or radius <= 0 or axis_deviation > 1e-12:
        raise ValueError("hemisphere axis must point along work +Z")
    if (
        any(not math.isfinite(x) for x in center)
        or any(abs(center[k]) + radius > 1e6 for k in (0, 1))
        or abs(center[2]) > 1e6
        or abs(center[2] + radius) > 1e6
    ):
        raise ValueError("hemisphere exceeds native coordinate bounds")
    u0, u1 = adaptor.FirstUParameter(), adaptor.LastUParameter()
    v0, v1 = adaptor.FirstVParameter(), adaptor.LastVParameter()
    angular_guard = 1e-12
    if (
        not all(math.isfinite(x) for x in (u0, u1, v0, v1))
        or abs(u1 - u0 - math.tau) > angular_guard
        or abs(v0) > angular_guard
        or abs(v1 - math.pi / 2) > angular_guard
    ):
        raise ValueError("hemisphere needs a full U turn and equator-to-north-pole V trim")
    # Complete boundary intervals certify the parameter domain; area/bounds
    # alone would miss a notch or a small hole. Degenerate pole edge stays in UV.
    sides = {"bottom": [], "top": [], "left": [], "right": []}
    uv_error = abs(u1 - u0 - math.tau) + abs(v0) + abs(v1 - math.pi / 2)
    explorer = BRepTools_WireExplorer(face.outerWire().wrapped, face.wrapped)
    count = 0
    while explorer.More():
        count += 1
        if count > 4096:
            raise ValueError("hemisphere trimming exceeds boundary work limit")
        curve = BRepAdaptor_Curve2d(explorer.Current(), face.wrapped)
        if curve.GetType() != GeomAbs_Line:
            raise ValueError("hemisphere trimming must be the analytic UV rectangle")
        a = tuple(curve.Value(curve.FirstParameter()).Coord())
        b = tuple(curve.Value(curve.LastParameter()).Coord())
        if not all(math.isfinite(x) for x in (*a, *b)):
            raise ValueError("nonfinite hemisphere UV trimming")
        matched = False
        for label, fixed_axis, value, varying in (
            ("bottom", 1, v0, 0),
            ("top", 1, v1, 0),
            ("left", 0, u0, 1),
            ("right", 0, u1, 1),
        ):
            deviations = abs(a[fixed_axis] - value) + abs(b[fixed_axis] - value)
            if deviations <= 2 * angular_guard:
                low, high = sorted((a[varying], b[varying]))
                if high - low <= angular_guard:
                    raise ValueError("degenerate hemisphere UV side")
                sides[label].append((low, high))
                uv_error += deviations
                matched = True
                break
        if not matched:
            raise ValueError("hemisphere has a nonrectangular trim/notch")
        explorer.Next()
    for label, intervals in sides.items():
        low, high = (u0, u1) if label in ("bottom", "top") else (v0, v1)
        position = low
        for a, b in sorted(intervals):
            if abs(a - position) > angular_guard or b > high + angular_guard:
                raise ValueError("hemisphere UV side has a gap or overlap")
            uv_error += abs(a - position)
            position = b
        if abs(position - high) > angular_guard:
            raise ValueError("hemisphere UV rectangle is incomplete")
        uv_error += abs(position - high)
    topology = max(
        [BRep_Tool.Tolerance_s(v.wrapped) for v in face.Vertices()]
        + [BRep_Tool.Tolerance_s(e.wrapped) for e in face.Edges()]
        + [BRep_Tool.Tolerance_s(face.wrapped)]
    )
    expected_area = 2 * math.pi * radius * radius
    if abs(face.Area() - expected_area) > max(16 * topology * radius, 256 * math.ulp(expected_area)):
        raise ValueError("hemisphere area disagrees with full analytic trim")
    numerical = max(1e-9, 512 * sys.float_info.epsilon * (max(map(abs, center)) + radius))
    errors = dict(
        cad_topology=2 * topology, uv_trim=radius * uv_error, axis=radius * axis_deviation, numerical=numerical
    )
    remaining = max_error_mm - sum(errors.values())
    if remaining <= 0:
        raise ValueError("requested hemisphere error is below CAD/trim/numerical uncertainty")
    triangles, facet_error, level, vertices = _hemisphere_facets(center, radius, remaining, max_facets)
    errors["facets"] = facet_error
    return SurfaceMesh(
        triangles,
        sum(errors.values()),
        dict(
            source or {},
            adapter="analytic_upper_hemisphere_facets_v1",
            center_mm=center,
            radius_mm=radius,
            base_z_mm=center[2],
            cadquery_version=cq.__version__,
            ocp_version=OCP.__version__,
            requested_max_error_mm=max_error_mm,
            error_components_mm=errors,
            subdivision_level=level,
            vertex_count=vertices,
            triangle_count=len(triangles),
            error_basis=(
                "complete analytic UV hemisphere; uniform radial facets; "
                "R minus minimum facet-plane distance plus CAD/trim/axis/numerical bounds"
            ),
            extraction_ms=(time.perf_counter() - started) * 1000,
        ),
    )


def _hemisphere_facets(center, radius, budget, max_facets):
    vertices = [(0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0)]
    faces = [(0, 1, 2), (0, 2, 3), (0, 3, 4), (0, 4, 1)]
    level = 0

    def error(face):
        a, b, c = (vertices[i] for i in face)
        u = tuple(b[k] - a[k] for k in range(3))
        v = tuple(c[k] - a[k] for k in range(3))
        n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
        distance = abs(sum(n[k] * a[k] for k in range(3))) / math.sqrt(sum(x * x for x in n))
        return radius * max(0.0, 1 - distance)

    while True:
        facet_error = max(map(error, faces))
        if facet_error <= budget:
            points = [tuple(center[k] + radius * v[k] for k in range(3)) for v in vertices]
            return [tuple(points[i] for i in f) for f in faces], facet_error, level, len(vertices)
        if max_facets is not None and len(faces) * 4 > max_facets:
            raise ValueError("requested hemisphere error exceeds bounded facet limit")
        shared = {}

        def midpoint(a, b):
            edge = tuple(sorted((a, b)))
            if edge not in shared:
                p = tuple(vertices[a][k] + vertices[b][k] for k in range(3))
                length = math.sqrt(sum(x * x for x in p))
                shared[edge] = len(vertices)
                vertices.append(tuple(x / length for x in p))
            return shared[edge]

        refined = []
        for a, b, c in faces:
            ab, bc, ca = midpoint(a, b), midpoint(b, c), midpoint(c, a)
            refined.extend(((a, ab, ca), (ab, b, bc), (ca, bc, c), (ab, bc, ca)))
        faces = refined
        level += 1


def export_floor(face, *, name, source, chord_error_mm=0.001):
    import cadquery as cq
    import OCP
    from OCP.BRep import BRep_Tool
    from OCP.BRepAdaptor import BRepAdaptor_Curve
    from OCP.BRepTools import BRepTools_WireExplorer
    from OCP.GeomAbs import GeomAbs_Circle, GeomAbs_Line
    from OCP.TopAbs import TopAbs_REVERSED

    start = time.perf_counter()
    if not isinstance(face, cq.Face) or not face.isValid() or face.geomType() != "PLANE":
        raise ValueError("select one valid planar pocket floor")
    normal = face.normalAt()
    if math.hypot(normal.x, normal.y) > 1e-12:
        raise ValueError("initial adapter requires a floor parallel to work XY")
    if not math.isfinite(chord_error_mm) or not 0 < chord_error_mm <= 0.1:
        raise ValueError("chord_error_mm must be in (0, 0.1]")

    outer_wire = face.outerWire()
    wires = [outer_wire] + [w for w in face.Wires() if not w.isSame(outer_wire)]
    joining_error = 0.0
    arc_error = 0.0
    curve_types = {}
    topology_tolerance = max(
        [BRep_Tool.Tolerance_s(v.wrapped) for v in face.Vertices()]
        + [BRep_Tool.Tolerance_s(e.wrapped) for e in face.Edges()]
        + [BRep_Tool.Tolerance_s(face.wrapped)]
    )

    def sample_wire(wire):
        nonlocal joining_error, arc_error
        points = []
        explorer = BRepTools_WireExplorer(wire.wrapped)
        while explorer.More():
            edge = cq.Edge(explorer.Current())
            adaptor = BRepAdaptor_Curve(edge.wrapped)
            u0, u1 = adaptor.FirstParameter(), adaptor.LastParameter()
            kind = adaptor.GetType()
            if kind == GeomAbs_Line:
                count = 1
                label = "line"
            elif kind == GeomAbs_Circle:
                radius = adaptor.Circle().Radius()
                max_angle = min(math.pi / 2, 4 * math.asin(math.sqrt(min(chord_error_mm / (2 * radius), 0.5))))
                count = max(1, math.ceil(abs(u1 - u0) / max_angle))
                if count > 1_000_000:
                    raise ValueError("circle subdivision exceeds the fixture limit")
                arc_error = max(arc_error, chord_error_mm)
                label = "circle"
            else:
                raise ValueError(f"unsupported boundary curve {edge.geomType()}; no approximation bound implemented")
            curve_types[label] = curve_types.get(label, 0) + 1
            samples = []
            for i in range(count + 1):
                p = adaptor.Value(u0 + (u1 - u0) * i / count)
                samples.append([p.X(), p.Y()])
            if edge.wrapped.Orientation() == TopAbs_REVERSED:
                samples.reverse()
            if points:
                gap = math.dist(points[-1], samples[0])
                if gap > max(topology_tolerance * 2, 1e-7):
                    raise ValueError(f"wire is disconnected by {gap:g} mm")
                joining_error = max(joining_error, gap)
                points.extend(samples[1:])
            else:
                points.extend(samples)
            explorer.Next()
        if len(points) < 4:
            raise ValueError("wire has fewer than three boundary segments")
        gap = math.dist(points[-1], points[0])
        if gap > max(topology_tolerance * 2, 1e-7):
            raise ValueError("wire is not closed")
        joining_error = max(joining_error, gap)
        return points[:-1]

    rings = [sample_wire(w) for w in wires]
    elapsed_ms = (time.perf_counter() - start) * 1000
    return {
        "version": 1,
        "units": "mm",
        "name": name,
        "outer": rings[0],
        "holes": rings[1:],
        "boundary_error_mm": arc_error + joining_error + topology_tolerance,
        "source": {
            **source,
            "cadquery_version": cq.__version__,
            "ocp_version": OCP.__version__,
            "python_version": platform.python_version(),
            "adapter_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "floor_z_mm": face.Center().z,
            "curve_counts": curve_types,
            "adapter": "horizontal_lines_circles_v1",
            "error_components_mm": {"chord": arc_error, "joining": joining_error, "cad_topology": topology_tolerance},
            "extraction_ms": elapsed_ms,
        },
        "checks": [],
    }


def export_floor_region(face, *, name, source, work_frame=None, max_error_mm=0.005, max_segments=None, cancel=None):
    """Export a planar floor in setup coordinates, preserving lines/circles.

    Ellipse/Bezier/B-spline edges use whole-interval derivative bounds. The
    requested error includes CAD topology, joining and rigid-frame roundoff.
    A valid region is not a certificate of cutter access above its floor.
    """
    import cadquery as cq
    from OCP.BRep import BRep_Tool

    started = time.perf_counter()
    if not isinstance(face, cq.Face) or not face.isValid() or face.geomType() != "PLANE":
        raise ValueError("select one valid planar pocket floor")
    if not math.isfinite(max_error_mm) or not 1e-6 <= max_error_mm <= 0.1:
        raise ValueError("max_error_mm must be finite in [1e-6, .1]")
    if max_segments is not None and (type(max_segments) is not int or max_segments < 4):
        raise ValueError("max_segments must be None (no cap) or an integer >= 4")
    _fixture_cancel(cancel)
    frame_error = 0.0
    if work_frame is not None:
        face, frame, matrix, frame_error, _ = _fixture_frame(face, work_frame)
        source = dict(source, work_frame=asdict(frame), model_to_work=matrix)
    normal = face.normalAt()
    if math.hypot(normal.x, normal.y) > 1e-12:
        raise ValueError("analytic floor export requires a floor parallel to work XY")
    topology_tolerance = max(
        [BRep_Tool.Tolerance_s(v.wrapped) for v in face.Vertices()]
        + [BRep_Tool.Tolerance_s(e.wrapped) for e in face.Edges()]
        + [BRep_Tool.Tolerance_s(face.wrapped)]
    )
    numerical = max(1e-9, 512 * sys.float_info.epsilon * max(1.0, *map(abs, _geometric_bounds(face))))
    region = _analytic_floor_region(
        face,
        name=name,
        source=source,
        topology_tolerance=topology_tolerance,
        started=started,
        chord_budget=(max_error_mm - topology_tolerance - frame_error - numerical) / 2,
        max_segments=max_segments,
        cancel=cancel,
    )
    region.source['error_components_mm']['numerical'] = numerical
    region = replace(region, boundary_error_mm=region.boundary_error_mm + numerical)
    if work_frame is not None:
        region.source['error_components_mm']['frame'] = frame_error
        region = replace(region, boundary_error_mm=region.boundary_error_mm + frame_error)
    if region.boundary_error_mm > max_error_mm:
        raise ValueError("CAD boundary uncertainty exceeds max_error_mm")
    _fixture_cancel(cancel)
    return region


def deburr_levels(shape, *, source=None, max_error_mm=0.005, level_tolerance_mm=1e-6, cancel=None):
    """The upward-facing planar faces of a solid, grouped by height, as
    ``camlib.operations.DeburrLevel`` (highest first), for ``camlib.Deburr``.

    A face counts when its outward normal is +Z (a top, a step, a pocket
    floor); its boundary is exported exactly (lines and circles kept) with
    ``export_floor_region``. Faces within ``level_tolerance_mm`` in height
    form one level. Which boundary stretches are convex edges to break, and
    which are concave (a wall rises there), the operation decides against the
    part itself.
    """
    from .operations import DeburrLevel

    solid = shape.val() if hasattr(shape, "val") else shape
    groups = []
    for index, face in enumerate(solid.Faces()):
        _fixture_cancel(cancel)
        if face.geomType() != "PLANE":
            continue
        normal = face.normalAt()
        if math.hypot(normal.x, normal.y) > 1e-9 or normal.z <= 0:
            continue
        z = face.Center().z
        region = export_floor_region(
            face,
            name=f"deburr-face-{index}",
            source=dict(source or {}, face=index),
            max_error_mm=max_error_mm,
            cancel=cancel,
        )
        for group in groups:
            if abs(group[0] - z) <= level_tolerance_mm:
                group[1].append(region)
                break
        else:
            groups.append((z, [region]))
    return tuple(DeburrLevel(tuple(regions), z) for z, regions in sorted(groups, key=lambda g: -g[0]))


def _analytic_floor_region(
    face, *, name, source, topology_tolerance, started, chord_budget=None, max_segments=None, cancel=None
):
    """Shared extraction after the caller validates the face or enclosing solid."""
    import cadquery as cq
    import OCP
    from OCP.BRepAdaptor import BRepAdaptor_Curve
    from OCP.BRepTools import BRepTools_WireExplorer
    from OCP.GeomAbs import GeomAbs_Circle, GeomAbs_Line
    from OCP.TopAbs import TopAbs_REVERSED
    from .planar import Boundary, Component, PlanarRegion

    outer = face.outerWire()
    wires = [outer] + [w for w in face.Wires() if not w.isSame(outer)]
    joining_error = 0.0
    chord_error = 0.0
    segments = 0
    curve_counts = {}
    if chord_budget is not None and chord_budget <= 0:
        raise ValueError("CAD boundary uncertainty exceeds max_error_mm")

    def boundary(wire):
        nonlocal joining_error, chord_error, segments
        vertices = []
        previous_end = None
        explorer = BRepTools_WireExplorer(wire.wrapped)
        while explorer.More():
            _fixture_cancel(cancel)
            edge = explorer.Current()
            curve = BRepAdaptor_Curve(edge)
            a, b = curve.FirstParameter(), curve.LastParameter()
            reversed_edge = edge.Orientation() == TopAbs_REVERSED
            if curve.GetType() == GeomAbs_Line:
                count, bulge, label = 1, 0.0, "line"
            elif curve.GetType() == GeomAbs_Circle:
                axis_z = curve.Circle().Axis().Direction().Z()
                if abs(abs(axis_z) - 1) > 1e-12:
                    raise ValueError("boundary circle is not parallel to XY")
                count = max(1, math.ceil(abs(b - a) / math.pi))
                if count > 1_000_000:
                    raise ValueError("circle subdivision exceeds analytic export limit")
                sweep = (b - a) * axis_z * (-1 if reversed_edge else 1) / count
                bulge = math.tan(sweep / 4)
                label = "circle"
            else:
                if chord_budget is None:
                    raise ValueError("analytic export supports only line/circle boundary curves")
                from ._cad_surface import _curve_bounds

                speed, accel = _curve_bounds(curve, 3)
                if (
                    not math.isfinite(speed)
                    or speed < 0
                    or (accel is not None and (not math.isfinite(accel) or accel < 0))
                ):
                    raise ValueError("nonfinite CAD boundary derivative bound")
                span = abs(b - a)
                # Linear interpolation remainder M*h^2/8. A C0 spline instead
                # uses the global Lipschitz bound M*h/2 across knot corners.
                demand = (
                    span * speed / (2 * chord_budget) if accel is None else span * math.sqrt(accel / (8 * chord_budget))
                )
                if not math.isfinite(demand) or (max_segments is not None and demand > max_segments - segments):
                    raise ValueError("CAD boundary exceeds max_segments")
                count, bulge, label = max(1, math.ceil(demand)), 0.0, cq.Edge(edge).geomType().lower()
                error = speed * span / (2 * count) if accel is None else accel * (span / count) ** 2 / 8
                chord_error = max(chord_error, error)
            segments += count
            if max_segments is not None and segments > max_segments:
                raise ValueError("CAD boundary exceeds max_segments")
            curve_counts[label] = curve_counts.get(label, 0) + 1
            parameters = [a + (b - a) * i / count for i in range(count + 1)]
            if reversed_edge:
                parameters.reverse()
            points = []
            for i, u in enumerate(parameters):
                if i % 256 == 0:
                    _fixture_cancel(cancel)
                p = curve.Value(u)
                points.append((p.X(), p.Y()))
            if previous_end is not None:
                gap = math.dist(previous_end, points[0])
                if gap > max(topology_tolerance * 2, 1e-7):
                    raise ValueError("analytic boundary wire is disconnected")
                joining_error = max(joining_error, gap)
            vertices.extend((p[0], p[1], bulge) for p in points[:-1])
            previous_end = points[-1]
            explorer.Next()
        if not vertices or previous_end is None:
            raise ValueError("empty analytic boundary")
        gap = math.dist(previous_end, vertices[0][:2])
        if gap > max(topology_tolerance * 2, 1e-7):
            raise ValueError("analytic boundary wire is open")
        joining_error = max(joining_error, gap)
        return Boundary(tuple(vertices))

    boundaries = [boundary(w) for w in wires]
    metadata = {
        **source,
        "cadquery_version": cq.__version__,
        "ocp_version": OCP.__version__,
        "python_version": platform.python_version(),
        "floor_z_mm": face.Center().z,
        "adapter": "bounded_planar_curves_v1" if chord_error else "analytic_horizontal_lines_circles_v1",
        "curve_counts": curve_counts,
        "adapter_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "error_components_mm": {"chord": chord_error, "joining": joining_error, "cad_topology": topology_tolerance},
        "extraction_ms": (time.perf_counter() - started) * 1000,
    }
    return PlanarRegion(
        name,
        (Component(boundaries[0], tuple(boundaries[1:])),),
        chord_error + joining_error + topology_tolerance,
        metadata,
    )


@dataclass(frozen=True)
class FixtureLimits:
    """Optional caller caps on CAD source size (None = no cap).

    max_cad_tolerance_mm is an accuracy requirement, not a size cap: the CAD
    kernel's own tolerance must not exceed it.
    """

    max_faces: int | None = None
    max_edges: int | None = None
    max_vertices: int | None = None
    max_brep_bytes: int | None = None
    max_body_bytes: int | None = None
    max_cad_tolerance_mm: float = 0.01

    def validate(self):
        for name in ("max_faces", "max_edges", "max_vertices", "max_brep_bytes", "max_body_bytes"):
            value = getattr(self, name)
            if value is not None and (type(value) is not int or value < 1):
                raise ValueError(f"fixture {name} must be None (no cap) or a positive integer")
        if not math.isfinite(self.max_cad_tolerance_mm) or not 0 < self.max_cad_tolerance_mm <= 0.1:
            raise ValueError("invalid fixture CAD tolerance limit")


def _over(value, cap):
    return cap is not None and value > cap


class _FingerprintSink(io.BytesIO):
    """OCCT writes through the stream interface; no BRep byte copy retained."""

    def __init__(self, limit, cancel):
        super().__init__()
        self.limit = limit
        self.cancel = cancel
        self.count = 0
        self.max_chunk = 0
        self.digest = hashlib.sha256()
        self.failure = None

    def write(self, data):
        # Throwing from OCCT's Python ostream callback can crash the wrapper.
        # Discard after failure and raise only once native serialization returns.
        if self.failure is not None:
            return len(data)
        try:
            _fixture_cancel(self.cancel)
            if self.limit is not None and self.count + len(data) > self.limit:
                raise ValueError("fixture BRep serialization exceeds byte budget")
        except BaseException as exc:
            self.failure = exc
            return len(data)
        self.count += len(data)
        self.max_chunk = max(self.max_chunk, len(data))
        self.digest.update(data)
        return len(data)


def _fixture_cancel(cancel):
    if cancel is not None and cancel():
        raise ValueError("CAD fixture extraction cancelled; no body published")


def _geometric_bounds(shape):
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib

    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape.wrapped, box, False, True)
    if box.IsVoid():
        raise ValueError("fixture has empty geometric bounds")
    # OCP8's Bnd_Box.Get returns an unregistered Limits type; corners are bound.
    values = (*box.CornerMin().Coord(), *box.CornerMax().Coord())
    if any(not math.isfinite(x) or abs(x) > 1e6 for x in values):
        raise ValueError("fixture geometric bounds exceed supported coordinates")
    return values


def _fixture_frame(shape, frame):
    from .motion import WorkFrame
    import cadquery as cq
    from OCP.gp import gp_Trsf

    if frame is None:
        frame = WorkFrame()
    if not isinstance(frame, WorkFrame):
        raise TypeError("fixture work_frame must be WorkFrame")
    o = frame.origin_mm
    axes = (frame.x, frame.y, frame.z)
    if any(len(v) != 3 or any(type(x) not in (int, float) or not math.isfinite(x) for x in v) for v in (o, *axes)):
        raise ValueError("fixture frame needs finite three-component vectors")

    def dot(a, b):
        return sum(x * y for x, y in zip(a, b))

    def cross(a, b):
        return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])

    if (
        any(abs(x) > 1e6 for x in o)
        or any(abs(dot(a, b) - int(i == j)) > 1e-12 for i, a in enumerate(axes) for j, b in enumerate(axes))
        or abs(dot(cross(axes[0], axes[1]), axes[2]) - 1) > 1e-12
    ):
        raise ValueError("fixture frame must be a bounded proper rigid frame")
    expected = [[*(float(x) for x in axis), -dot(axis, o)] for axis in axes]
    transform = gp_Trsf()
    transform.SetValues(*(x for row in expected for x in row))
    applied = [[transform.Value(i + 1, j + 1) for j in range(4)] for i in range(3)]
    bounds = _geometric_bounds(shape)
    scale = [max(abs(bounds[k]), abs(bounds[k + 3]), 1.0) for k in range(3)]
    error = math.sqrt(
        sum(
            (
                sum(abs(applied[i][j] - expected[i][j]) * scale[j] for j in range(3))
                + abs(applied[i][3] - expected[i][3])
            )
            ** 2
            for i in range(3)
        )
    )
    return shape.moved(cq.Location(transform)), frame, applied, error, bounds


def _fixture_fingerprint(shape, limits, cancel):
    from cadquery.occ_impl.shapes import TopTools_IndexedMapOfShape
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Copy
    from OCP.BRepTools import BRepTools
    from OCP.TopExp import TopExp
    from OCP.TopTools import TopTools_FormatVersion

    # Geometry is shared read-only; topology is copied so cache-status flags can
    # be normalized without changing the caller. Mesh caches are not copied.
    copied = BRepBuilderAPI_Copy(shape.wrapped, False, False).Shape()
    parts = TopTools_IndexedMapOfShape()
    TopExp.MapShapes_s(copied, parts)
    for i in range(1, parts.Extent() + 1):
        _fixture_cancel(cancel)
        part = parts.FindKey(i)
        part.Checked(False)
        part.Modified(False)
        part.Free(False)
        part.Convex(False)
    sink = _FingerprintSink(limits.max_brep_bytes, cancel)
    BRepTools.Write_s(copied, sink, False, False, TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3)
    if sink.failure is not None:
        raise sink.failure
    if not sink.count:
        raise ValueError("CAD kernel wrote no fixture fingerprint data")
    return dict(
        brep_sha256=sink.digest.hexdigest(),
        brep_bytes=sink.count,
        hash_max_chunk_bytes=sink.max_chunk,
        brep_format="occt_ascii_v3_no_mesh_normalized_cache_flags",
    )


def export_fixture(
    solid,
    *,
    id,
    revision,
    source=None,
    role="fixture",
    work_frame=None,
    unsupported="reject",
    limits=FixtureLimits(),
    cancel=None,
):
    """Export a proven vertical CAD prism, or an explicit outer-only enclosure.

    work_frame maps work coordinates to model coordinates. Default rejection never
    silently turns an unsupported solid into a filled prism. No source mutation.
    """
    import cadquery as cq
    import OCP
    from OCP.BRep import BRep_Tool
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Plane, GeomAbs_Cylinder
    from .planar import PlanarRegion, Component, Boundary
    from .verification import Prism

    limits.validate()
    _fixture_cancel(cancel)
    if (
        not isinstance(id, str)
        or not re.fullmatch(r"[A-Za-z0-9_.:-]{1,128}", id)
        or not isinstance(revision, str)
        or not 0 < len(revision.encode()) <= 512
    ):
        raise ValueError("fixture needs a bounded body ID and revision")
    if role not in ("fixture", "protected_target") or unsupported not in ("reject", "enclose"):
        raise ValueError("invalid fixture role or unsupported-body policy")
    original = solid
    # CadQuery boolean results often wrap a single solid in a compound. Unwrap
    # only single-child containers; never discard other solids or loose geometry.
    for _ in range(16):
        if isinstance(solid, cq.Compound):
            children = list(solid)
            if len(children) != 1:
                raise ValueError("select one CAD solid, not an assembly or loose geometry")
            solid = children[0]
        else:
            break
    if not isinstance(solid, cq.Solid) or solid.wrapped.IsNull():
        raise ValueError("select one CAD solid, not a face, shell, workplane or assembly")
    faces, edges, vertices = solid.Faces(), solid.Edges(), solid.Vertices()
    if (
        not (faces and edges and vertices)
        or _over(len(faces), limits.max_faces)
        or _over(len(edges), limits.max_edges)
        or _over(len(vertices), limits.max_vertices)
    ):
        raise ValueError("fixture topology is empty or exceeds the caller's face/edge/vertex caps")
    shells = solid.Shells()
    if not solid.isValid() or not shells or any(not s.Closed() for s in shells):
        raise ValueError("fixture must be a valid closed positively oriented solid")
    volume = solid.Volume()
    if not math.isfinite(volume) or volume <= 0:
        raise ValueError("fixture must be a valid closed positively oriented solid")
    _fixture_cancel(cancel)
    topology = max([BRep_Tool.Tolerance_s(p.wrapped) for p in (*faces, *edges, *vertices)])
    if not math.isfinite(topology) or not 0 <= topology <= limits.max_cad_tolerance_mm:
        raise ValueError("fixture CAD tolerance exceeds configured limit")
    work, frame, matrix, frame_error, model_bounds = _fixture_frame(solid, work_frame)
    bounds = _geometric_bounds(work)
    scale = max(1.0, *(abs(x) for x in bounds), *(abs(x) for x in model_bounds))
    numeric = 256 * sys.float_info.epsilon * scale
    axis_error = 0.0
    horizontal = []
    counts = {}
    reason = None
    for face in work.Faces():
        _fixture_cancel(cancel)
        adaptor = BRepAdaptor_Surface(face.wrapped)
        kind = adaptor.GetType()
        if kind == GeomAbs_Plane:
            normal = adaptor.Plane().Axis().Direction()
            xy = math.hypot(normal.X(), normal.Y())
            z = abs(normal.Z())
            counts["plane"] = counts.get("plane", 0) + 1
            if xy <= 1e-12:
                horizontal.append((adaptor.Plane().Location().Z(), face))
                axis_error = max(axis_error, xy * math.hypot(bounds[3] - bounds[0], bounds[4] - bounds[1]))
            elif z <= 1e-12:
                axis_error = max(axis_error, z * (bounds[5] - bounds[2]))
            else:
                reason = reason or "nonvertical planar side or tilted cap"
        elif kind == GeomAbs_Cylinder:
            axis = adaptor.Cylinder().Axis().Direction()
            tilt = math.hypot(axis.X(), axis.Y())
            counts["cylinder"] = counts.get("cylinder", 0) + 1
            if tilt > 1e-12:
                reason = reason or "cylinder axis is not parallel to work Z"
            else:
                axis_error = max(axis_error, tilt * (bounds[5] - bounds[2]))
        else:
            label = face.geomType()
            counts[label] = counts.get(label, 0) + 1
            reason = reason or f"unsupported side surface {label}"
    if len(horizontal) != 2:
        reason = reason or "prism needs exactly two horizontal cap faces"
    region = None
    if reason is None:
        horizontal.sort(key=lambda item: item[0])
        bottom, top = horizontal
        zmin, zmax = bottom[0], top[0]
        error = topology + axis_error + frame_error + numeric
        if zmax <= zmin or abs(bounds[2] - zmin) > 4 * error or abs(bounds[5] - zmax) > 4 * error:
            reason = "cap heights do not bound the complete solid"
        elif bottom[1].normalAt().z >= 0 or top[1].normalAt().z <= 0:
            reason = "cap orientation does not enclose positive prism material"
        else:
            # The complete solid is already valid and its rigidly moved cap was
            # classified above. Reuse its tolerance bound and avoid revalidating
            # that face or traversing its topology again.
            try:
                region = _analytic_floor_region(
                    bottom[1], name=id, source={}, topology_tolerance=topology, started=time.perf_counter()
                )
            except ValueError as exc:
                reason = f"unsupported cap boundary: {exc}"
    if reason is not None and unsupported == "reject":
        raise ValueError(f"CAD fixture is not a supported solid prism: {reason}")
    if reason is not None:
        axis_error = 0.0  # the geometric envelope already bounds unsupported tilt
    _fixture_cancel(cancel)
    fingerprint = _fixture_fingerprint(original, limits, cancel)
    # Deep-copy caller provenance and omit internal performance clocks. Timing is
    # measured externally, never a changing part of a geometry/scene identity.
    provenance = json.loads(json.dumps({} if source is None else source, allow_nan=False))
    metadata = dict(
        source_reference=provenance,
        body_id=id,
        revision=revision,
        **fingerprint,
        adapter="cad_fixture_prism_or_envelope_v1",
        adapter_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        cadquery_version=cq.__version__,
        ocp_version=OCP.__version__,
        python_version=platform.python_version(),
        work_to_model=asdict(frame),
        applied_model_to_work=matrix,
        model_bounds_mm=model_bounds,
        work_bounds_mm=bounds,
        topology_counts=dict(faces=len(faces), edges=len(edges), vertices=len(vertices)),
        surface_counts=counts,
        error_components_mm=dict(
            cad_topology=topology, axis_alignment=axis_error, frame=frame_error, numerical=numeric
        ),
        classification="solid_prism" if region is not None else "outer_envelope",
        unsupported_reason=reason,
    )
    error = topology + axis_error + frame_error + numeric
    if region is None:
        region = PlanarRegion(
            id,
            (
                Component(
                    Boundary.polygon(
                        ((bounds[0], bounds[1]), (bounds[3], bounds[1]), (bounds[3], bounds[4]), (bounds[0], bounds[4]))
                    )
                ),
            ),
            error,
            dict(cad_body=metadata),
        )
        zmin, zmax = bounds[2], bounds[5]
    else:
        region_source = dict(region.source)
        region_source.pop("extraction_ms", None)
        region_source["cad_body"] = metadata
        region = replace(
            region,
            boundary_error_mm=region.boundary_error_mm + axis_error + frame_error + numeric,
            source=region_source,
        )
    result = Prism(id, region, zmin, zmax, role, error, "solid_prism" if reason is None else "outer_envelope")
    if _over(len(json.dumps(result.to_dict(), allow_nan=False, separators=(",", ":")).encode()), limits.max_body_bytes):
        raise ValueError("CAD fixture body exceeds JSON byte budget")
    _fixture_cancel(cancel)
    return result


def export_inner_patches(shape, *, source=None, max_error_mm=0.005, max_patches=None, max_segments=None, cancel=None):
    """Extract source-backed nonvertical planar faces for target lower bounds.

    Holes and curved trims use the existing bounded region exporter. The native
    verifier erodes the boundary uncertainty when proving whole XY footprints.
    Curved faces and vertical planes contribute no patches; an empty result
    makes no inner-enclosure claim. This is not a full solid-volume certificate.
    """
    import cadquery as cq
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from .material import InnerPlanarPatch
    from .motion import WorkFrame

    if (max_patches is not None and (type(max_patches) is not int or max_patches < 1)) or (
        max_segments is not None and (type(max_segments) is not int or max_segments < 4)
    ):
        raise ValueError('invalid inner patch face/segment budget')
    if not math.isfinite(max_error_mm) or not 1e-6 <= max_error_mm <= 0.1:
        raise ValueError('max_error_mm must be finite in [1e-6,.1]')
    _fixture_cancel(cancel)
    if isinstance(shape, cq.Workplane):
        shape = shape.val()
    if not isinstance(shape, cq.Shape) or not shape.isValid() or not shape.Solids():
        raise ValueError('inner patches need valid CadQuery solids')
    faces = shape.Faces()
    if len(faces) > 4096:
        raise ValueError('CAD inner patches exceed source face budget')
    solid_faces = {}
    for solid in shape.Solids():
        _fixture_cancel(cancel)
        for face in solid.Faces():
            solid_faces.setdefault(face.hashCode(), []).append(face)
    if any(not any(face.isSame(other) for other in solid_faces.get(face.hashCode(), ())) for face in faces):
        raise ValueError('inner patch source contains faces outside its solids')
    patches = []
    segments = 0
    omitted = {'curved_faces': 0, 'vertical_faces': 0}
    for face in faces:
        _fixture_cancel(cancel)
        if face.geomType() != 'PLANE':
            omitted['curved_faces'] += 1
            continue
        plane = BRepAdaptor_Surface(face.wrapped).Plane()
        axes = plane.Position()
        # OCCT may store reflected plane parameter axes (notably chamfers).
        # Reverse the normal for a proper CAM frame; the plane itself is
        # unchanged and extraction reprojects the original trim into this frame.
        normal_sign = 1 if axes.Direct() else -1
        frame = WorkFrame(
            tuple(plane.Location().Coord()),
            tuple(axes.XDirection().Coord()),
            tuple(axes.YDirection().Coord()),
            tuple(normal_sign * v for v in axes.Direction().Coord()),
        )
        if abs(frame.z[2]) <= 1e-12:
            omitted['vertical_faces'] += 1
            continue
        if max_patches is not None and len(patches) == max_patches:
            raise ValueError('CAD inner patches exceed max_patches')
        if max_segments is not None and max_segments - segments < 4:
            raise ValueError('CAD inner patches exceed max_segments')
        region = export_floor_region(
            face,
            name=f'planar-face-{len(patches)}',
            source=dict(source or {}, adapter='cad_inner_planar_patch_v1'),
            work_frame=frame,
            max_error_mm=max_error_mm,
            max_segments=None if max_segments is None else max_segments - segments,
            cancel=cancel,
        )
        segments += sum(
            len(boundary.vertices)
            for component in region.components
            for boundary in (component.outer, *component.holes)
        )
        if max_segments is not None and segments > max_segments:
            raise ValueError('CAD inner patches exceed max_segments')
        # The frame places the nominal source plane at local zero. Include
        # measured placement residual, topology and transform/evaluation error.
        errors = region.source['error_components_mm']
        normal_error = (
            abs(region.source['floor_z_mm']) + 2 * errors['cad_topology'] + errors['numerical'] + errors.get('frame', 0)
        )
        if normal_error > max_error_mm:
            raise ValueError('CAD inner patch placement uncertainty exceeds max_error_mm')
        patches.append(InnerPlanarPatch(region, frame, normal_error))
    for patch in patches:
        patch.region.source.update(
            omitted,
            included_planar_faces=len(patches),
            patch_adapter='cad_inner_planar_patch_v1',
            error_basis=(
                'trimmed source plane; full-footprint boundary erosion and normal-placement error; '
                'uncovered faces retain uncertainty'
            ),
        )
    _fixture_cancel(cancel)
    return tuple(patches)


def export_contact_witnesses(
    shape, *, spacing_mm=0.5, min_z_mm=None, max_points=None, max_candidates=None, source=None, cancel=None
):
    """Sample classified CAD faces for rigorous ball-access exclusion bounds.

    Each accepted sample is evaluated on the original trimmed parametric face;
    it is within the returned engineering error of a CAD target point. Spacing
    controls density, not a surface coverage guarantee or machining tolerance.
    The native verifier bounds contact for entire center rectangles, never just
    sampled cutter positions. Missing/thin features can only weaken exclusion.
    This does not certify a watertight mesh or an inner volume enclosure.
    """
    import cadquery as cq
    import OCP
    from OCP.BRep import BRep_Tool
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.BRepClass import BRepClass_FaceClassifier
    from OCP.TopAbs import TopAbs_IN, TopAbs_ON
    from OCP.gp import gp_Pnt2d
    from ._cad_surface import _check_cancel, _surface_bounds
    from .material import ContactWitnesses

    if not math.isfinite(spacing_mm) or not 0.001 <= spacing_mm <= 100:
        raise ValueError('witness spacing_mm must be finite in [.001,100]')
    if min_z_mm is not None and (not math.isfinite(min_z_mm) or abs(min_z_mm) > 1e6):
        raise ValueError('invalid witness minimum Z')
    if max_points is not None and (type(max_points) is not int or max_points < 1):
        raise ValueError('witness max_points must be None (no cap) or >= 1')
    if max_candidates is not None and (type(max_candidates) is not int or max_candidates < 1):
        raise ValueError('witness max_candidates must be None (no cap) or >= 1')
    _check_cancel(cancel)
    if isinstance(shape, cq.Workplane):
        shape = shape.val()
    if not isinstance(shape, cq.Shape) or not shape.isValid() or not shape.Solids():
        raise ValueError('contact witnesses need valid CadQuery solids')
    faces = shape.Faces()
    solid_faces = {}
    for solid in shape.Solids():
        for face in solid.Faces():
            solid_faces.setdefault(face.hashCode(), []).append(face)
    if any(not any(face.isSame(other) for other in solid_faces.get(face.hashCode(), ())) for face in faces):
        raise ValueError('contact witness source contains faces outside its solids')
    if len(faces) > 4096:
        raise ValueError('CAD witnesses exceed face budget')
    box = shape.BoundingBox()
    magnitude = max(1.0, *(abs(v) for v in (box.xmin, box.xmax, box.ymin, box.ymax, box.zmin, box.zmax)))
    numerical = 8192 * math.ulp(magnitude) + 1e-10
    topology = max(
        [BRep_Tool.Tolerance_s(v.wrapped) for v in shape.Vertices()]
        + [BRep_Tool.Tolerance_s(e.wrapped) for e in shape.Edges()]
        + [BRep_Tool.Tolerance_s(f.wrapped) for f in faces]
    )
    points = {}
    maximum_error = 0.0
    candidates = 0
    sampled_faces = 0
    for face in faces:
        _check_cancel(cancel)
        if min_z_mm is not None and face.BoundingBox().zmax < min_z_mm:
            continue
        adaptor = BRepAdaptor_Surface(face.wrapped)
        bounds = _surface_bounds(adaptor)
        u0, u1 = adaptor.FirstUParameter(), adaptor.LastUParameter()
        v0, v1 = adaptor.FirstVParameter(), adaptor.LastVParameter()
        if not all(math.isfinite(x) for x in (u0, u1, v0, v1, *bounds)):
            raise ValueError('nonfinite CAD witness parameter bounds')
        # Limit both physical parameter steps; no inference about unsampled trim.
        nu = max(1, math.ceil((u1 - u0) * bounds[0] / spacing_mm))
        nv = max(1, math.ceil((v1 - v0) * bounds[1] / spacing_mm))
        size = (nu + 1) * (nv + 1)
        if max_candidates is not None and candidates + size > max_candidates:
            raise ValueError('CAD witnesses exceed candidate budget; increase spacing or narrow Z scope')
        uv_tolerance = max(1e-12, 64 * math.ulp(max(1.0, abs(u0), abs(u1), abs(v0), abs(v1))))
        error = 2 * topology + numerical + math.hypot(bounds[0], bounds[1]) * uv_tolerance
        maximum_error = max(maximum_error, error)
        if maximum_error > 0.1:
            raise ValueError('CAD witness point uncertainty exceeds .1mm')
        sampled_faces += 1
        for i in range(nu + 1):
            u = u0 + (u1 - u0) * i / nu
            for j in range(nv + 1):
                if candidates % 256 == 0:
                    _check_cancel(cancel)
                candidates += 1
                v = v0 + (v1 - v0) * j / nv
                classified = BRepClass_FaceClassifier(face.wrapped, gp_Pnt2d(u, v), uv_tolerance)
                if classified.State() not in (TopAbs_IN, TopAbs_ON):
                    continue
                point = tuple(adaptor.Value(u, v).Coord())
                if min_z_mm is not None and point[2] + error < min_z_mm:
                    continue
                points[point] = None
                if max_points is not None and len(points) > max_points:
                    raise ValueError('CAD witnesses exceed max_points; increase spacing or narrow Z scope')
    _check_cancel(cancel)
    if not points:
        raise ValueError('no CAD contact witnesses in requested scope')
    return ContactWitnesses(
        tuple(points),
        maximum_error,
        dict(
            source or {},
            adapter='classified_cad_contact_points_v1',
            cadquery_version=cq.__version__,
            ocp_version=OCP.__version__,
            spacing_mm=spacing_mm,
            min_z_mm=min_z_mm,
            sampled_faces=sampled_faces,
            candidates=candidates,
            accepted_points=len(points),
            error_basis=(
                'classified trimmed CAD face points; topology, UV classification and evaluation guards; '
                'exclusion only, no coverage claim'
            ),
        ),
    )


def flow_paths(face, *, stepover_mm, along="u", sample_mm=0.5, zigzag=True):
    """Flow passes for ``camlib.Project``: the face's own parameter lines
    (``along`` "u" or "v"), seen from above, at most ``stepover_mm`` apart.

    Each next line is the farthest one still within the stepover of the
    previous (bisection on the parameter, the XY gap measured at
    ``sample_mm`` along both lines). Project then follows the paths on the
    surface mesh with its own proven clearance; the face only shapes them.
    A line that coincides with the previous one (a closed face's seam, or
    the last line reached but for rounding) is not repeated.
    """
    from camlib.operations import TracePath, TraceLine

    if along not in ("u", "v"):
        raise ValueError("along must be 'u' or 'v'")
    if not (stepover_mm > 0 and sample_mm > 0):
        raise ValueError("stepover_mm and sample_mm must be positive")
    u0, u1, v0, v1 = face._uvBounds()
    if along == "v":
        (u0, u1), (v0, v1) = (v0, v1), (u0, u1)

    def point(s, t):
        u, v = (s, t) if along == "u" else (t, s)
        p = face.positionAt(u, v)
        return (p.x, p.y)

    def line(t, n):
        return [point(u0 + (u1 - u0) * i / n, t) for i in range(n + 1)]

    probe = line(v0, 64) + line(v1, 64)
    span = max(math.dist(a, b) for a, b in zip(probe[:65], probe[1:65])) * 64
    n = max(2, math.ceil(span / sample_mm))

    def gap(a, b):
        return max(math.dist(p, q) for p, q in zip(a, b))

    lines = [line(v0, n)]
    t = v0
    # A line within this of the previous one adds no pass (the bisection can
    # stop a rounding error short of the last line).
    same = 1e-6
    while t < v1:
        lo, hi = t, v1
        last = line(v1, n)
        if gap(lines[-1], last) <= same:
            break
        if gap(lines[-1], last) <= stepover_mm:
            t = v1
        else:
            for _ in range(40):
                mid = (lo + hi) / 2
                if gap(lines[-1], line(mid, n)) <= stepover_mm:
                    lo = mid
                else:
                    hi = mid
            if lo == t:
                raise ValueError("flow lines cannot meet the stepover (degenerate face parameterization)")
            t = lo
            if gap(line(t, n), last) <= same:
                # The farthest line within the stepover is the last one, but
                # for rounding: take the last one itself.
                t = v1
        lines.append(line(t, n))
    paths = []
    for k, points in enumerate(lines):
        if zigzag and k % 2:
            points = points[::-1]
        segments = tuple(TraceLine(q) for p, q in zip(points, points[1:]) if math.dist(p, q) > 1e-9)
        if segments:
            paths.append(TracePath(points[0], segments))
    return tuple(paths)


@dataclass(frozen=True)
class TurnSection:
    """A turned part's half-section for a lathe setup (``camlib.TurningSetup.from_section``): ``profile``
    (radius, z, bulge) from the front face on the axis along the outside to
    the back corner, ``bore`` (the hole from the front face to its deepest
    point, or None), the front and back z, and the solid's volume against the
    volume of revolution of the section (their difference proves the part is
    turned)."""

    profile: tuple
    bore: tuple | None
    z_front_mm: float
    z_back_mm: float
    volume_mm3: float
    revolved_volume_mm3: float
    face_grooves: tuple = ()


def turning_section(solid, *, tolerance_mm=1e-6):
    """The half-section of a solid turned about its Z axis (front face toward
    +Z), for lathe planning: ``TurningSetup.from_section(s, chuck_z_mm=...,
    ...)``.

    Every section edge must be a line or a circular arc (kept exact as
    bulges). Refused: a solid that is not a solid of revolution about Z (its
    volume differs from 2 pi x section area x centroid radius: flats, cross
    holes, keyways), closed internal voids, undercuts (a profile or bore
    turning back toward the front) and other curve types."""
    import cadquery as cq
    from OCP.BRepAdaptor import BRepAdaptor_Curve
    from OCP.GeomAbs import GeomAbs_Circle, GeomAbs_Line

    if not isinstance(solid, cq.Shape) or not solid.isValid() or not solid.Solids():
        raise ValueError("turning_section needs a valid CadQuery solid")
    if not (math.isfinite(tolerance_mm) and 0 < tolerance_mm <= 1e-3):
        raise ValueError("tolerance_mm must be in (0, 0.001]")
    tol = tolerance_mm
    bb = solid.BoundingBox()
    reach = max(abs(bb.xmin), abs(bb.xmax), abs(bb.ymin), abs(bb.ymax)) * 2 + 1
    plane = cq.Face.makeFromWires(
        cq.Wire.makePolygon(
            [
                cq.Vector(0, 0, bb.zmin - 1),
                cq.Vector(reach, 0, bb.zmin - 1),
                cq.Vector(reach, 0, bb.zmax + 1),
                cq.Vector(0, 0, bb.zmax + 1),
            ],
            close=True,
        )
    )
    faces = solid.intersect(plane).Faces()
    if len(faces) != 1:
        raise ValueError(f"the half-section has {len(faces)} faces: one connected turned part is needed")
    face = faces[0]
    if face.innerWires():
        raise ValueError("the half-section has a closed internal void, which a lathe cannot reach")
    # Pappus: a solid of revolution has volume 2 pi A r (A the half-section
    # area, r its centroid's radius).
    area = face.Area()
    revolved = 2 * math.pi * area * face.Center().x
    volume = solid.Volume()
    if abs(volume - revolved) > 1e-6 * max(volume, 1) + 1e-6:
        raise ValueError(
            f"not a solid of revolution about Z: volume {volume:.6f} mm3, the section revolved "
            f"{revolved:.6f} mm3 (flats, cross holes or keyways are not turned)"
        )

    # Segments in (r, z): (start, end, center or None, sweep).
    def ang(v):
        return math.atan2(v[0], v[1])  # from +Z toward +X (r)

    segments = []
    for edge in face.outerWire().Edges():
        adaptor = BRepAdaptor_Curve(edge.wrapped)
        u0, u1 = adaptor.FirstParameter(), adaptor.LastParameter()
        p = [adaptor.Value(u) for u in (u0, (u0 + u1) / 2, u1)]
        a, m, b = ((q.X(), q.Z()) for q in p)
        if any(abs(q.Y()) > tol for q in p):
            raise ValueError("section edge leaves the XZ plane")
        kind = adaptor.GetType()
        if kind == GeomAbs_Line:
            segments.append((a, b, None, 0.0))
        elif kind == GeomAbs_Circle:
            c3 = adaptor.Circle().Location()
            c = (c3.X(), c3.Z())
            a0, am, a1 = (ang((q[0] - c[0], q[1] - c[1])) for q in (a, m, b))
            d1, d2 = (am - a0) % math.tau, (a1 - a0) % math.tau
            segments.append((a, b, c, d2 if d1 < d2 else d2 - math.tau))
        else:
            raise ValueError(
                "a section edge is neither a line nor a circular arc: model the turned profile with lines and arcs"
            )

    # One closed loop, each segment's end at the next one's start.
    def near(p, q):
        return math.hypot(p[0] - q[0], p[1] - q[1]) <= 1e3 * tol

    loop = [segments.pop(0)]
    while segments:
        end = loop[-1][1]
        k = next((i for i, s in enumerate(segments) if near(s[0], end) or near(s[1], end)), None)
        if k is None:
            raise ValueError("the half-section boundary is not one closed loop")
        a, b, c, sweep = segments.pop(k)
        loop.append((a, b, c, sweep) if near(a, end) else (b, a, c, -sweep))
    if not near(loop[-1][1], loop[0][0]):
        raise ValueError("the half-section boundary is not closed")
    points = [s[0] for s in loop]
    z_front = max(p[1] for p in points)
    z_back = min(p[1] for p in points)
    front = min((i for i, p in enumerate(points) if abs(p[1] - z_front) <= tol), key=lambda i: points[i][0])
    n = len(loop)

    def tangent_r(i, forward):
        # The r component of the unit direction leaving vertex i.
        a, b, c, sweep = loop[i] if forward else loop[(i - 1) % n]
        start = a if forward else b
        if c is None:
            other = b if forward else a
            d = (other[0] - start[0], other[1] - start[1])
        else:
            # Tangent of the arc at start, in its direction of travel.
            v = (start[0] - c[0], start[1] - c[1])
            s = (1 if sweep > 0 else -1) * (1 if forward else -1)
            d = (s * v[1], -s * v[0])
        return d[0] / math.hypot(*d)

    used = set()

    def walk(start, step, stop):
        chain = []
        i = start
        for _ in range(n):
            k = i if step > 0 else (i - 1) % n
            seg = loop[k]
            used.add(k)
            a, b, c, sweep = seg if step > 0 else (seg[1], seg[0], seg[2], -seg[3])
            chain.append((a, b, sweep))
            i = (i + step) % n
            if stop(points[i]):
                return chain
        raise ValueError("the half-section boundary has no back end")

    outward = 1 if tangent_r(front, True) >= tangent_r(front, False) else -1
    back = max((p for p in points if abs(p[1] - z_back) <= tol), key=lambda p: p[0])
    outer = walk(front, outward, lambda p: near(p, back))
    bore = None
    if points[front][0] > tol:
        bore = walk(front, -outward, lambda p: p[0] <= tol or abs(p[1] - z_back) <= tol)

    # The rest of the boundary: the back face and the axis only. Anything
    # else is a cavity on the axis behind solid material (not reachable).
    for k in set(range(n)) - used:
        a, b = loop[k][0], loop[k][1]
        on_axis = a[0] <= tol and b[0] <= tol
        on_back = abs(a[1] - z_back) <= tol and abs(b[1] - z_back) <= tol
        if not (on_axis or on_back):
            raise ValueError(
                "the half-section has an internal cavity on the axis behind solid material, which a lathe cannot reach"
            )

    def to_points(chain, what):
        out = []
        for a, b, sweep in chain:
            if b[1] > a[1] + tol or sweep and _arc_turns_back(a, b, sweep):
                raise ValueError(
                    f"the {what} turns back toward the front face (an undercut), which "
                    "turning toward the chuck cannot follow"
                )
            out.append((a[0], a[1], math.tan(sweep / 4) if sweep else 0.0))
        out.append((chain[-1][1][0], chain[-1][1][1], 0.0))
        return tuple(out)

    # Rectangular face grooves: straight down from the front face, across a
    # flat bottom, straight back up to it. The profile keeps the face
    # continuous; the grooves are declared separately.
    from .turning import FaceGroove

    grooves = []
    k = 0
    while k + 2 < len(outer):
        (a, b, s0), (c, d, s1), (e, f, s2) = outer[k], outer[k + 1], outer[k + 2]
        if (
            not (s0 or s1 or s2)
            and abs(a[1] - z_front) <= tol
            and abs(a[0] - b[0]) <= tol
            and b[1] < a[1] - tol
            and abs(c[1] - d[1]) <= tol
            and d[0] > c[0] + tol
            and abs(e[0] - f[0]) <= tol
            and abs(f[1] - z_front) <= tol
        ):
            grooves.append(FaceGroove(a[0], e[0], b[1]))
            outer[k : k + 3] = [(a, f, 0.0)]
        k += 1
    profile = to_points(outer, "profile")
    if profile[0][0] > tol:
        # The front face continues across the hole (the bore is its own chain).
        profile = ((0.0, z_front, 0.0),) + profile
    return TurnSection(
        profile, None if bore is None else to_points(bore, "bore"), z_front, z_back, volume, revolved, tuple(grooves)
    )


def _arc_turns_back(a, b, sweep):
    """Whether a (r, z) arc from a sweeping ``sweep`` (from +Z toward +r)
    passes an extreme of z inside it (z rising again)."""
    # Center from the chord and the sweep.
    bulge = math.tan(sweep / 4)
    f = (1 - bulge * bulge) / (4 * bulge)
    u = (a[1], a[0]), (b[1], b[0])  # (z, r)
    c = ((u[0][0] + u[1][0]) / 2 - f * (u[1][1] - u[0][1]), (u[0][1] + u[1][1]) / 2 + f * (u[1][0] - u[0][0]))
    a0 = math.atan2(u[0][1] - c[1], u[0][0] - c[0])
    for extreme in (0.0, math.pi):  # z greatest / least
        d = (extreme - a0) % math.tau if sweep > 0 else (a0 - extreme) % math.tau
        if 1e-9 < d < abs(sweep) - 1e-9:
            # z turns around inside the arc.
            return True
    return False
