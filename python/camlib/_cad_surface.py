"""CAD-kernel triangulation with independently bounded parametric interpolation.

The CAD kernel supplies valid trimmed-face topology. Requested mesher deflection
is only a seed: surface Hessians and trimming-curve derivative bounds determine
acceptance. No sampled distance is used as an error certificate.
"""

from collections import Counter
import math
import time


def _norm(values):
    return math.sqrt(sum(x * x for x in values))


def _bezier_bounds(curve, span=1.0):
    """Convex-hull bounds for rational Bezier first/second derivatives."""
    points = [tuple(curve.Pole(i).Coord()) for i in range(1, curve.NbPoles() + 1)]
    weights = [curve.Weight(i) for i in range(1, curve.NbPoles() + 1)]
    if not points or min(weights) <= 0 or not math.isfinite(span) or span <= 0:
        raise ValueError('invalid bounded Bezier curve')
    anchor = points[0]
    radius = max(_norm(tuple(a - b for a, b in zip(p, anchor))) for p in points)
    numerator = [tuple(w * (a - b) for a, b in zip(p, anchor)) for p, w in zip(points, weights)]

    def derivative(poles):
        degree = len(poles) - 1
        return [tuple(degree * (b - a) / span for a, b in zip(p, q)) for p, q in zip(poles, poles[1:])]

    n1 = derivative(numerator)
    n2 = derivative(n1)
    w1 = derivative([(w,) for w in weights])
    w2 = derivative(w1)
    n1max = max(map(_norm, n1), default=0.0)
    n2max = max(map(_norm, n2), default=0.0)
    w1max = max(map(_norm, w1), default=0.0)
    w2max = max(map(_norm, w2), default=0.0)
    speed = (n1max + w1max * radius) / min(weights)
    accel = (n2max + w2max * radius + 2 * w1max * speed) / min(weights)
    return speed, accel


def _curve_bounds(adaptor, dimension):
    """Global derivative bounds over the represented analytic/Bezier spans."""
    from OCP.GeomAbs import GeomAbs_Line, GeomAbs_Circle, GeomAbs_Ellipse, GeomAbs_BezierCurve, GeomAbs_BSplineCurve

    kind = adaptor.GetType()
    if kind == GeomAbs_Line:
        return 1.0, 0.0
    if kind == GeomAbs_Circle:
        r = adaptor.Circle().Radius()
        return r, r
    if kind == GeomAbs_Ellipse:
        r = adaptor.Ellipse().MajorRadius()
        return r, r
    if kind == GeomAbs_BezierCurve:
        return _bezier_bounds(adaptor.Bezier())
    if kind == GeomAbs_BSplineCurve:
        try:
            from OCP.TColStd import TColStd_Array1OfReal
        except ImportError:  # OCP 8 moved collection aliases.
            from OCP.collections import Array1_double as TColStd_Array1OfReal
        if dimension == 2:
            from OCP.Geom2dConvert import Geom2dConvert_BSplineCurveToBezierCurve as Converter
        else:
            from OCP.GeomConvert import GeomConvert_BSplineCurveToBezierCurve as Converter
        converter = Converter(adaptor.BSpline(), adaptor.FirstParameter(), adaptor.LastParameter(), 1e-12)
        count = converter.NbArcs()
        if count < 1:
            raise ValueError('CAD curve has no Bezier spans')
        knots = TColStd_Array1OfReal(1, count + 1)
        converter.Knots(knots)
        bounds = [_bezier_bounds(converter.Arc(i), knots.Value(i + 1) - knots.Value(i)) for i in range(1, count + 1)]
        spline = adaptor.BSpline()
        c1 = all(spline.Multiplicity(i) <= spline.Degree() - 1 for i in range(2, spline.NbKnots()))
        return max(b[0] for b in bounds), max(b[1] for b in bounds) if c1 else None
    raise ValueError(f'no derivative bound for CAD trimming/basis curve {kind}')


def _surface_bounds(adaptor):
    """Bounds for |Fu|, |Fv|, |Fuu|, |Fuv|, |Fvv| in world coordinates."""
    from OCP.GeomAbs import (
        GeomAbs_Plane,
        GeomAbs_Cylinder,
        GeomAbs_Cone,
        GeomAbs_Sphere,
        GeomAbs_Torus,
        GeomAbs_SurfaceOfExtrusion,
    )

    kind = adaptor.GetType()
    if kind == GeomAbs_Plane:
        return 1.0, 1.0, 0.0, 0.0, 0.0
    if kind == GeomAbs_Cylinder:
        r = adaptor.Cylinder().Radius()
        return r, 1.0, r, 0.0, 0.0
    if kind == GeomAbs_Cone:
        cone = adaptor.Cone()
        sine = abs(math.sin(cone.SemiAngle()))
        r = max(
            abs(cone.RefRadius() + v * math.sin(cone.SemiAngle()))
            for v in (adaptor.FirstVParameter(), adaptor.LastVParameter())
        )
        return r, 1.0, r, sine, 0.0
    if kind == GeomAbs_Sphere:
        r = adaptor.Sphere().Radius()
        return r, r, r, r, r
    if kind == GeomAbs_Torus:
        torus = adaptor.Torus()
        r = torus.MinorRadius()
        total = torus.MajorRadius() + r
        return total, r, total, r, r
    if kind == GeomAbs_SurfaceOfExtrusion:
        speed, accel = _curve_bounds(adaptor.BasisCurve(), 3)
        return speed, 1.0, accel or 0.0, 0.0, 0.0, accel is None
    raise ValueError(f'no parametric interpolation bound for CAD surface {kind}')


def _local_bounds(adaptor):
    """Second-derivative bounds over a band of v, where the face-wide ones are
    far too large: `f(v0, v1)` gives `_surface_bounds`' tuple for v0 <= v <= v1,
    or None when the face-wide bounds are used.

    The interpolation remainder at a point of a UV triangle only involves the
    second derivatives on the segments from that point to the corners, all
    inside the triangle, so bounds over the triangle's own v range are enough.
    A sphere's and a cone's u derivatives vanish towards the pole and the
    apex: face-wide bounds there ask for facets far smaller than the surface
    needs (and, bisected, for facets below the numeric resolution).
    """
    from OCP.GeomAbs import GeomAbs_Cone, GeomAbs_Sphere

    kind = adaptor.GetType()
    if kind == GeomAbs_Sphere:
        # F = C + r (cos v cos u, cos v sin u, sin v):
        # |Fu| = |Fuu| = r |cos v|, |Fuv| = r |sin v|, |Fv| = |Fvv| = r.
        r = adaptor.Sphere().Radius()

        def sphere(v0, v1):
            nearest = 0.0 if v0 <= 0 <= v1 else min(abs(v0), abs(v1))
            farthest = max(abs(v0), abs(v1))
            cosine = math.cos(nearest) if nearest < math.pi / 2 else 0.0
            sine = math.sin(farthest) if farthest < math.pi / 2 else 1.0
            return r * cosine, r, r * cosine, r * sine, r

        return sphere
    if kind == GeomAbs_Cone:
        # F = L + (R + v sin a)(cos u, sin u, 0) + v cos a Z:
        # |Fu| = |Fuu| = |R + v sin a|, |Fuv| = |sin a|, Fvv = 0.
        cone = adaptor.Cone()
        reference, sine = cone.RefRadius(), math.sin(cone.SemiAngle())

        def conical(v0, v1):
            r = max(abs(reference + v0 * sine), abs(reference + v1 * sine))
            return r, 1.0, r, abs(sine), 0.0

        return conical
    return None


def _interpolation_error(uv, bounds):
    # Taylor's integral remainder and barycentric interpolation cancel the
    # first derivative. Variance <= span^2/4; |covariance| <= du*dv/4.
    du = max(p[0] for p in uv) - min(p[0] for p in uv)
    dv = max(p[1] for p in uv) - min(p[1] for p in uv)
    if len(bounds) > 5 and bounds[5]:
        return bounds[0] * du / 2  # C0 spline: first-derivative Lipschitz bound.
    return (bounds[2] * du * du + 2 * bounds[3] * du * dv + bounds[4] * dv * dv) / 8


def _check_cancel(cancel):
    if cancel is not None and cancel():
        raise ValueError('CAD surface export cancelled')


class _FaceMesh:
    """One face's UV triangulation while it is refined.

    Triangles are counter-clockwise in UV and index the face's own nodes;
    `gid` maps a node to its vertex of the whole solid. Every directed edge
    belongs to at most one triangle (`owner`), so the triangle across an
    edge (a, b) is the owner of (b, a). `trim` holds the directed edges on a
    CAD edge: (edge id, parameter at a, parameter at b, pcurve, UV shift).
    """

    def __init__(self, adaptor, bounds, uv, indices, remaining, reserve):
        self.adaptor = adaptor
        self.bounds = bounds
        self.local = _local_bounds(adaptor)
        self.uv = list(uv)
        self.gid = [None] * len(uv)
        self.triangles = list(indices)
        self.depth = [0] * len(indices)
        self.owner = {}
        for k, (a, b, c) in enumerate(indices):
            self.owner[(a, b)] = self.owner[(b, c)] = self.owner[(c, a)] = k
        self.trim = {}
        self.remaining = remaining
        self.reserve = reserve
        self.weld = 0.0

    def limits(self, triangle):
        """Derivative bounds that hold on a triangle."""
        if self.local is None:
            return self.bounds
        v = [self.uv[i][1] for i in triangle]
        return self.local(min(v), max(v))

    def cost(self, a, b, bounds):
        a, b = self.uv[a], self.uv[b]
        du, dv = abs(a[0] - b[0]), abs(a[1] - b[1])
        if len(bounds) > 5 and bounds[5]:
            return bounds[0] * du
        return bounds[2] * du * du + 2 * bounds[3] * du * dv + bounds[4] * dv * dv

    def error(self, k):
        triangle = self.triangles[k]
        return _interpolation_error([self.uv[i] for i in triangle], self.limits(triangle))

    def third(self, a, b):
        triangle = self.triangles[self.owner[(a, b)]]
        return triangle[(triangle.index(a) + 2) % 3]

    def area(self, a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    def node(self, uv, gid, point):
        """A new node at `uv` that is the solid's vertex `gid` at `point`.

        The facets at this node interpolate `point`, not the surface point
        F(uv): the distance between the two is part of the face's error."""
        actual = tuple(self.adaptor.Value(*uv).Coord())
        self.weld = max(self.weld, math.dist(actual, point))
        self.uv.append(uv)
        self.gid.append(gid)
        return len(self.uv) - 1

    def split(self, a, b, m, asked):
        """Bisect the triangle that owns the directed edge (a, b) at node m.

        `depth` counts the bisections a facet's own error asked for (`asked`);
        one made for a neighbour never increases a facet's UV extents."""
        k = self.owner.pop((a, b))
        c = self.triangles[k][(self.triangles[k].index(a) + 2) % 3]
        other = len(self.triangles)
        self.triangles[k] = (a, m, c)
        self.triangles.append((m, b, c))
        self.depth[k] += asked
        self.depth.append(self.depth[k])
        self.owner[(a, m)] = self.owner[(m, c)] = k
        self.owner[(m, b)] = self.owner[(b, c)] = self.owner[(c, m)] = other
        return k, other


def _face_data(face, cancel):
    """Check UV mesh orientation/incidence and every trim's parameter chain."""
    from OCP.BRep import BRep_Tool
    from OCP.BRepAdaptor import BRepAdaptor_Surface, BRepAdaptor_Curve2d
    from OCP.BRepTools import BRepTools_WireExplorer
    from OCP.TopLoc import TopLoc_Location

    adaptor = BRepAdaptor_Surface(face.wrapped)
    bounds = _surface_bounds(adaptor)
    if not all(math.isfinite(x) and x >= 0 for x in bounds):
        raise ValueError('nonfinite CAD derivative bound')
    location = TopLoc_Location()
    mesh = BRep_Tool.Triangulation_s(face.wrapped, location)
    if mesh is None or not mesh.HasUVNodes():
        raise ValueError('CAD mesher omitted face UV nodes')
    uv = [None] + [tuple(mesh.UVNode(i).Coord()) for i in range(1, mesh.NbNodes() + 1)]
    if any(not all(math.isfinite(v) for v in p) for p in uv[1:]):
        raise ValueError('nonfinite CAD UV mesh')
    indices = []
    edges = Counter()
    directions = Counter()
    for i in range(1, mesh.NbTriangles() + 1):
        if i % 1024 == 0:
            _check_cancel(cancel)
        triangle = tuple(mesh.Triangle(i).Get())
        a, b, c = (uv[j] for j in triangle)
        area = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        if area <= 0 or not math.isfinite(area):
            raise ValueError('CAD UV mesh has reversed/degenerate triangles')
        indices.append(triangle)
        for p, q in zip(triangle, (*triangle[1:], triangle[0])):
            edges[tuple(sorted((p, q)))] += 1
            directions[(p, q)] += 1
    if any(n not in (1, 2) for n in edges.values()) or any(
        n == 2 and (directions[(p, q)] != 1 or directions[(q, p)] != 1) for (p, q), n in edges.items()
    ):
        raise ValueError('CAD UV mesh has inconsistent edge incidence')
    boundary = {edge for edge, n in edges.items() if n == 1}
    represented = Counter()
    trims = []
    trim_error = 0.0
    trim_segments = 0
    # The input BRep is valid; matching every topological trim once to the mesh
    # boundary retains its holes/seams. Interior edges cancel in opposite pairs.
    for wire in face.Wires():
        explorer = BRepTools_WireExplorer(wire.wrapped, face.wrapped)
        while explorer.More():
            _check_cancel(cancel)
            edge = explorer.Current()
            curve = BRepAdaptor_Curve2d(edge, face.wrapped)
            polygon = BRep_Tool.PolygonOnTriangulation_s(edge, mesh, location)
            if polygon is None or not polygon.HasParameters():
                raise ValueError('CAD trim has no mesh parameter correspondence')
            count = polygon.NbNodes()
            if count < 2:
                raise ValueError('invalid CAD trim node count')
            first, last = curve.FirstParameter(), curve.LastParameter()
            params = [polygon.Parameter(i) for i in range(1, count + 1)]
            guard = 1e-10 * max(1.0, abs(first), abs(last))
            if (
                abs(params[0] - first) > guard
                or abs(params[-1] - last) > guard
                or any(b < a for a, b in zip(params, params[1:]))
            ):
                raise ValueError('CAD trim parameter chain is incomplete')
            speed, accel = _curve_bounds(curve, 2)
            ids = [polygon.Node(i) for i in range(1, count + 1)]
            first_uv = tuple(curve.Value(params[0]).Coord())
            shift = [0.0, 0.0]
            for axis, periodic, period in (
                (0, adaptor.IsUPeriodic(), adaptor.UPeriod),
                (1, adaptor.IsVPeriodic(), adaptor.VPeriod),
            ):
                if periodic:
                    value = period()
                    shift[axis] = round((uv[ids[0]][axis] - first_uv[axis]) / value) * value
            gap = max(
                math.dist(uv[j], tuple(v + delta for v, delta in zip(curve.Value(t).Coord(), shift)))
                for j, t in zip(ids, params)
            )
            gap += speed * (abs(params[0] - first) + abs(params[-1] - last))
            for p, q, t0, t1 in zip(ids, ids[1:], params, params[1:]):
                represented[tuple(sorted((p, q)))] += 1
                deviation = speed * (t1 - t0) / 2 if accel is None else accel * (t1 - t0) ** 2 / 8
                trim_error = max(trim_error, math.hypot(bounds[0], bounds[1]) * (gap + deviation))
                trim_segments += 1
            trims.append((edge, curve, tuple(shift), ids, params))
            explorer.Next()
    if set(represented) != boundary or any(n != 1 for n in represented.values()):
        raise ValueError('CAD trim chains do not cover mesh boundary exactly once')
    return adaptor, bounds, uv, indices, trim_error, trim_segments, trims


def export_curved_solid(shape, *, max_error_mm, max_facets, source, cancel):
    import cadquery as cq
    import OCP
    from OCP.BRep import BRep_Tool
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Copy
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.BRepTools import BRepTools
    from OCP.TopAbs import TopAbs_REVERSED
    from .surface import SurfaceMesh

    started = time.perf_counter()
    _check_cancel(cancel)
    if not math.isfinite(max_error_mm) or not 1e-6 <= max_error_mm <= 0.1:
        raise ValueError('max_error_mm must be finite in [1e-6, .1]')
    if max_facets is not None and (type(max_facets) is not int or max_facets < 4):
        raise ValueError('max_facets must be None (no cap) or an integer >= 4')
    # Meshing mutates kernel caches: use an owned copy, never the caller's BRep.
    shape = cq.Shape.cast(BRepBuilderAPI_Copy(shape.wrapped, True, False).Shape())
    faces = shape.Faces()
    if not faces:
        raise ValueError('CAD surface has no faces')
    topology = max(
        [BRep_Tool.Tolerance_s(v.wrapped) for v in shape.Vertices()]
        + [BRep_Tool.Tolerance_s(e.wrapped) for e in shape.Edges()]
        + [BRep_Tool.Tolerance_s(f.wrapped) for f in faces]
    )
    box = shape.BoundingBox()
    magnitude = max(1.0, *(abs(v) for v in (box.xmin, box.xmax, box.ymin, box.ymax, box.zmin, box.zmax)))
    numerical = 8192 * math.ulp(magnitude) + 1e-10
    fixed_error = 2 * topology + numerical
    budget = max_error_mm - fixed_error
    if budget <= 0:
        raise ValueError('CAD topology/numerical error exceeds requested surface error')
    # Check supported surface definitions before invoking a potentially costly mesh.
    from OCP.BRepAdaptor import BRepAdaptor_Surface

    for face in faces:
        _surface_bounds(BRepAdaptor_Surface(face.wrapped))
    seed_deflection = budget / 4
    for attempt in range(5):
        _check_cancel(cancel)
        BRepTools.Clean_s(shape.wrapped)
        BRepMesh_IncrementalMesh(shape.wrapped, seed_deflection, False, 0.5, True)
        data = [_face_data(f, cancel) for f in faces]
        if max(row[4] for row in data) <= budget / 2:
            break
        seed_deflection /= 4
    else:
        raise ValueError('CAD trim approximation exceeds requested bound')
    # Conforming refinement of the whole solid (issue #241). The kernel's UV
    # mesh of a face is conforming, and the kernel discretises a CAD edge once
    # for all its faces. Bisecting each facet on its own, and evaluating a
    # shared edge's nodes on each face's own surface, broke both: neighbours
    # kept the unsplit edge (a T-junction; the new vertex is on the surface,
    # the old edge is a chord), and the two faces of an edge differed by the
    # kernel's edge tolerance. So here:
    #   * every mesh vertex of the solid is one 3D point. A node on a CAD edge
    #     is the edge curve's point at its parameter (the CAD vertex's point at
    #     an end), for every face of that edge; an interior node is F(u, v);
    #   * an edge is bisected in every triangle that has it, at one point: the
    #     conforming bisection step of Rivara, "Algorithms for refining
    #     triangular grids suitable for adaptive and multigrid techniques",
    #     Int. J. Numer. Methods Eng. 20 (1984) 745-756, with the neighbour
    #     closed by one bisection of the same edge (the "green" closure of
    #     Bank, Sherman and Weiser, "Refinement algorithms and data structures
    #     for regular local mesh refinement", Scientific Computing, 1983). An
    #     edge on a CAD edge is bisected at the mid parameter, in all faces of
    #     that edge at once.
    # Rivara's propagation to the neighbour's longest edge is not needed: it
    # keeps angles bounded, while this bound depends only on a facet's UV
    # extents, which no bisection of any of its edges increases.
    #
    # The bound, per facet actually produced: its vertices are within `weld`
    # (measured at every node on a CAD edge, zero elsewhere) of F at its UV
    # nodes, so by convexity the facet is within `weld` of the interpolant of
    # F on its UV triangle, which is within `_interpolation_error` of F
    # there. `reserve` is what `weld` may reach; beyond it the export is
    # refused. `trim_error` bounds the UV domain's own chords as before: a
    # new node on a trim is on the pcurve between two nodes already there, or
    # on their chord.
    from cadquery.occ_impl.shapes import TopTools_IndexedMapOfShape
    from OCP.BRepAdaptor import BRepAdaptor_Curve
    from OCP.TopExp import TopExp

    shapes = TopTools_IndexedMapOfShape()
    TopExp.MapShapes_s(shape.wrapped, shapes)
    points = []
    keys = {}
    evaluations = 0

    def vertex(key, make):
        index = keys.get(key)
        if index is None:
            index = keys[key] = len(points)
            point = tuple(make())
            if not all(math.isfinite(v) and abs(v) <= 1e6 for v in point):
                raise ValueError('CAD surface exceeds bounded coordinates')
            points.append(point)
        return index

    edges = {}

    def edge_record(edge):
        eid = shapes.FindIndex(edge)
        if eid not in edges:
            ends = []
            for end in (TopExp.FirstVertex_s(edge, False), TopExp.LastVertex_s(edge, False)):
                ends.append(None if end.IsNull() else (('v', shapes.FindIndex(end)), BRep_Tool.Pnt_s(end).Coord))
            degenerated = BRep_Tool.Degenerated_s(edge)
            if degenerated and ends[0] is None:
                raise ValueError('CAD degenerated edge has no vertex')
            edges[eid] = (None if degenerated else BRepAdaptor_Curve(edge), ends, None)
        return eid, edges[eid]

    def edge_vertex(eid, t, end=None):
        """The solid's vertex at parameter t of an edge (end: 0 first, 1 last)."""
        curve, ends, _ = edges[eid]
        if curve is None:
            return vertex(*ends[0])
        if end is None:
            return vertex(('e', eid, t), curve.Value(t).Coord)
        if ends[end] is not None:
            return vertex(*ends[end])
        # An edge without that vertex: its own end point, one for a closed edge.
        if end == 1 and not curve.IsClosed():
            return vertex(('e', eid, 'last'), curve.Value(curve.LastParameter()).Coord)
        return vertex(('e', eid, 'first'), curve.Value(curve.FirstParameter()).Coord)

    meshes = []
    shared = {}
    for index, (face, (adaptor, bounds, uv, indices, trim_error, trim_segments, trims)) in enumerate(zip(faces, data)):
        _check_cancel(cancel)
        reserve = min(1e-4, (budget - trim_error) / 4)
        mesh = _FaceMesh(adaptor, bounds, uv, indices, budget - trim_error - reserve, reserve)
        meshes.append(mesh)
        for edge, curve, shift, ids, params in trims:
            eid, record = edge_record(edge)
            if record[2] is None:
                edges[eid] = (*record[:2], params)
            elif record[2] != params:
                raise ValueError(
                    f'CAD mesher discretised an edge of face {index} differently on its faces; '
                    'no watertight mesh can be built from this triangulation'
                )
            last = len(ids) - 1
            for position, (node, t) in enumerate(zip(ids, params)):
                gid = edge_vertex(eid, t, 0 if position == 0 else 1 if position == last else None)
                if mesh.gid[node] not in (None, gid):
                    raise ValueError(f'CAD trims of face {index} disagree at a shared mesh node')
                mesh.gid[node] = gid
            for p, q, t0, t1 in zip(ids, ids[1:], params, params[1:]):
                a, b, ta, tb = (p, q, t0, t1) if (p, q) in mesh.owner else (q, p, t1, t0)
                mesh.trim[(a, b)] = (eid, ta, tb, curve, shift)
                shared.setdefault((eid, t0, t1), []).append((mesh, a, b))
        for node in range(1, len(mesh.uv)):
            actual = tuple(adaptor.Value(*mesh.uv[node]).Coord())
            evaluations += 1
            if mesh.gid[node] is None:
                mesh.gid[node] = len(points)
                if not all(math.isfinite(v) and abs(v) <= 1e6 for v in actual):
                    raise ValueError('CAD surface exceeds bounded coordinates')
                points.append(actual)
            else:
                mesh.weld = max(mesh.weld, math.dist(actual, points[mesh.gid[node]]))

    def check_weld(mesh):
        if mesh.weld > mesh.reserve:
            raise ValueError(
                f'CAD edge curves lie {mesh.weld:g} mm from face {meshes.index(mesh)}, more than the '
                f'{mesh.reserve:g} mm the requested error leaves for it; request a larger max_error_mm '
                'or repair the edge tolerances of the solid'
            )

    for mesh in meshes:
        check_weld(mesh)
    pending = [(mesh, k) for mesh in meshes for k in range(len(mesh.triangles))]
    count = len(pending)
    steps = 0
    while pending:
        if max_facets is not None and count > max_facets:
            raise ValueError('CAD surface exceeds max_facets at requested error')
        steps += 1
        if steps % 1024 == 0:
            _check_cancel(cancel)
        mesh, k = pending.pop()
        if mesh.error(k) <= mesh.remaining:
            continue
        if mesh.depth[k] >= 24:
            raise ValueError('CAD surface interpolation exceeds refinement depth')
        triangle = mesh.triangles[k]
        limits = mesh.limits(triangle)
        a, b = max(((triangle[i], triangle[(i + 1) % 3]) for i in range(3)), key=lambda edge: mesh.cost(*edge, limits))
        if (a, b) not in mesh.trim:
            if (b, a) not in mesh.owner:
                raise ValueError('CAD UV mesh has inconsistent edge incidence')
            middle = tuple((x + y) / 2 for x, y in zip(mesh.uv[a], mesh.uv[b]))
            point = tuple(mesh.adaptor.Value(*middle).Coord())
            evaluations += 1
            if not all(math.isfinite(v) and abs(v) <= 1e6 for v in point):
                raise ValueError('CAD surface exceeds bounded coordinates')
            mesh.uv.append(middle)
            mesh.gid.append(len(points))
            points.append(point)
            m = len(mesh.uv) - 1
            pending.extend((mesh, n) for n in (*mesh.split(a, b, m, True), *mesh.split(b, a, m, False)))
            count += 2
            continue
        eid, ta, tb, _, _ = mesh.trim[(a, b)]
        low, high = min(ta, tb), max(ta, tb)
        tm = (low + high) / 2
        if not low < tm < high:
            raise ValueError('CAD trim refinement is below its parameter resolution')
        gid = edge_vertex(eid, tm)
        asking = (mesh, a, b)
        for holder, a, b in shared.pop((eid, low, high)):
            _, ta, tb, curve, shift = holder.trim.pop((a, b))
            ua, ub, uc = holder.uv[a], holder.uv[b], holder.uv[holder.third(a, b)]
            # On the trim itself where that keeps both halves counter-clockwise
            # (always, unless the facet is thinner than the trim's own sag);
            # else on the chord. Either way the vertex is the edge's point and
            # its distance to F there is measured.
            middle = tuple(v + delta for v, delta in zip(curve.Value(tm).Coord(), shift))
            if holder.area(ua, middle, uc) <= 0 or holder.area(middle, ub, uc) <= 0:
                middle = tuple((x + y) / 2 for x, y in zip(ua, ub))
            m = holder.node(middle, gid, points[gid])
            evaluations += 1
            check_weld(holder)
            holder.trim[(a, m)] = (eid, ta, tm, curve, shift)
            holder.trim[(m, b)] = (eid, tm, tb, curve, shift)
            shared.setdefault((eid, min(ta, tm), max(ta, tm)), []).append((holder, a, m))
            shared.setdefault((eid, min(tm, tb), max(tm, tb)), []).append((holder, m, b))
            pending.extend((holder, n) for n in holder.split(a, b, m, (holder, a, b) == asking))
            count += 1
    # The facets of the solid, as triples of its vertices. `side` maps each
    # directed facet edge to its facet: adding a second facet with the same
    # directed edge is refused, so the result is checked, not assumed.
    facets = []
    side = {}

    def area2(a, b, c):
        ab = tuple(y - x for x, y in zip(points[a], points[b]))
        ac = tuple(y - x for x, y in zip(points[a], points[c]))
        return _norm((ab[1] * ac[2] - ab[2] * ac[1], ab[2] * ac[0] - ab[0] * ac[2], ab[0] * ac[1] - ab[1] * ac[0]))

    def remove(k):
        a, b, c, _ = facets[k]
        del side[(a, b)], side[(b, c)], side[(c, a)]
        facets[k] = None

    def add(a, b, c, face_index):
        """Add a facet; a facet and its mirror image enclose nothing and
        cancel. Returns the facet's index, or None when it cancelled."""
        for p, q, r in ((a, b, c), (b, c, a), (c, a, b)):
            k = side.get((q, p))
            if k is not None and r in facets[k][:3] and (r, q) in side and side[(r, q)] == k:
                remove(k)
                return None
        if (a, b) in side or (b, c) in side or (c, a) in side:
            raise ValueError(
                f'CAD surface facets overlap along an edge at {points[a]} in face {face_index}: '
                'the solid cannot be meshed watertight at the requested error'
            )
        side[(a, b)] = side[(b, c)] = side[(c, a)] = len(facets)
        facets.append((a, b, c, face_index))
        return len(facets) - 1

    face_rows = []
    for index, (face, mesh, row) in enumerate(zip(faces, meshes, data)):
        _check_cancel(cancel)
        reversed_face = face.wrapped.Orientation() == TopAbs_REVERSED
        collapsed = 0
        max_interior = 0.0
        for k, triangle in enumerate(mesh.triangles):
            max_interior = max(max_interior, mesh.error(k))
            a, b, c = (mesh.gid[i] for i in triangle)
            if a == b or b == c or c == a:
                # A facet on a pole (a degenerated edge is one point): it is
                # the segment its two neighbours share, so leaving it out
                # joins them along that segment.
                collapsed += 1
                continue
            if reversed_face:
                b, c = c, b
            add(a, b, c, index)
        face_rows.append((collapsed, max_interior))
    # A facet whose three vertices are in line (a pole or apex and two nodes
    # of one meridian, in UV a proper triangle) has no area: the engine takes
    # no such facet, and leaving it out would leave its middle vertex on the
    # neighbour's long edge, a T-junction. The middle vertex b is on that edge
    # a-c, so the neighbour (a, c, w) is the two facets (a, b, w) and
    # (b, c, w): the same points, but for b's measured distance from the
    # edge, which is added to the error.
    collinear = 0.0
    work = [k for k, facet in enumerate(facets) if facet is not None and area2(*facet[:3]) <= 2e-10]
    budget_steps = 8 * len(facets) + 64
    while work:
        budget_steps -= 1
        if budget_steps < 0:
            raise ValueError('CAD surface has facets below the numeric resolution that cannot be resolved')
        k = work.pop()
        if facets[k] is None:
            continue
        a, b, c, index = facets[k]
        # Rotate so that c-a is the longest edge and b the vertex beside it.
        a, b, c = max(((a, b, c), (b, c, a), (c, a, b)), key=lambda t: math.dist(points[t[2]], points[t[0]]))
        length = math.dist(points[a], points[c])
        if length > 0:
            direction = tuple((y - x) / length for x, y in zip(points[a], points[c]))
            along = min(length, max(0.0, sum((q - p) * d for p, q, d in zip(points[a], points[b], direction))))
            foot = tuple(p + along * d for p, d in zip(points[a], direction))
            collinear = max(collinear, math.dist(points[b], foot))
        else:
            collinear = max(collinear, math.dist(points[a], points[b]))
        neighbour = side.get((a, c))
        if neighbour is None:
            raise ValueError(
                f'face {index} needs a facet below the numeric resolution (area {area2(a, b, c) / 2:g} mm2) '
                'on the rim of an open shell at the requested error; request a larger max_error_mm'
            )
        na, nb, nc, owner = facets[neighbour]
        w = next(v for v in (na, nb, nc) if v not in (a, c))
        remove(k)
        remove(neighbour)
        if w != b:
            for facet in ((a, b, w), (b, c, w)):
                added = add(*facet, owner)
                if added is not None and area2(*facet) <= 2e-10:
                    work.append(added)
    result = []
    counts = Counter()
    for facet in facets:
        if facet is not None:
            a, b, c, index = facet
            result.append((points[a], points[b], points[c]))
            counts[index] += 1
    if not result:
        raise ValueError('CAD surface has no facets')
    # Every facet edge is matched by one facet running the other way between
    # the same two vertices, except on a CAD edge that has one face (the rim
    # of an open shell).
    free = set()
    for holders in shared.values():
        if len(holders) == 1:
            holder, a, b = holders[0]
            free.add(frozenset((holder.gid[a], holder.gid[b])))
    for a, b in side:
        if (b, a) not in side and frozenset((a, b)) not in free:
            raise ValueError(
                f'CAD surface facets do not close along the edge {points[a]} - {points[b]}: '
                'the solid cannot be meshed watertight at the requested error'
            )
    face_reports = []
    max_error = 0.0
    for index, (face, mesh, row, (collapsed, max_interior)) in enumerate(zip(faces, meshes, data, face_rows)):
        trim_error, trim_segments = row[4], row[5]
        if mesh.weld + collinear > mesh.reserve:
            raise ValueError(
                f'face {index}: facets in line at a pole move the mesh by {collinear:g} mm, more than the '
                'requested error leaves for it; request a larger max_error_mm'
            )
        max_error = max(max_error, max_interior + trim_error + mesh.weld + collinear + fixed_error)
        face_reports.append(
            dict(
                kind=face.geomType(),
                facets=counts[index],
                collapsed_pole_facets=collapsed,
                trim_segments=trim_segments,
                interpolation_error_mm=max_interior,
                trim_error_mm=trim_error,
                edge_weld_mm=mesh.weld,
            )
        )
    _check_cancel(cancel)
    return SurfaceMesh(
        result,
        max_error,
        dict(
            source or {},
            adapter='bounded_parametric_cad_solid_v1',
            cadquery_version=cq.__version__,
            ocp_version=OCP.__version__,
            requested_error_mm=max_error_mm,
            error_basis=(
                'CAD-kernel valid trim topology; matched UV boundary chains; '
                'analytic/Bezier derivative bounds for trim and barycentric surface interpolation; '
                'one vertex per mesh node of the solid (edge-curve points on CAD edges, their distance '
                'to each face measured), every edge bisected in all its facets'
            ),
            cad_topology_error_mm=2 * topology,
            numerical_guard_mm=numerical,
            collinear_facet_error_mm=collinear,
            face_bounds=face_reports,
            triangle_count=len(result),
            surface_evaluations=evaluations,
            seed_deflection_mm=seed_deflection,
            extraction_ms=(time.perf_counter() - started) * 1000,
        ),
    )
