"""CAD-kernel triangulation with independently bounded parametric interpolation.

The CAD kernel supplies valid trimmed-face topology. Requested mesher deflection
is only a seed: surface Hessians and trimming-curve derivative bounds determine
acceptance. No sampled distance is used as an error certificate.

The interpolation bound is the classical one from bounds on derivatives: a
triangle's linear interpolant of a surface differs from it by at most a
second-derivative bound times the squared parameter extents. See D. Filip,
R. Magedson and R. Markot, "Surface algorithms using bounds on derivatives",
Computer Aided Geometric Design 3 (1986) 295-311. B-spline and Bezier faces
get those derivative bounds from the control nets of their Bezier patches'
hodographs (G. Farin, "Curves and Surfaces for CAGD", 5th ed., sections 5.3
and 14.4: a Bezier curve or patch, and each of its derivatives, lies in the
convex hull of its control net), and trimming curves are bounded against
their chords by the same hull property.
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


def _real_array(low, high):
    try:
        from OCP.TColStd import TColStd_Array1OfReal
    except ImportError:  # OCP 8 moved collection aliases.
        from OCP.collections import Array1_double as TColStd_Array1OfReal
    return TColStd_Array1OfReal(low, high)


class _SplineBounds:
    """Derivative bounds of a B-spline or Bezier face, from the control nets
    of its Bezier patches (the engine's `surface::spline_bounds`).

    `whole` is `_surface_bounds`' tuple for the face; `local` gives it for a
    UV box, from pieces of the patches no larger than the box, with
    'lipschitz' appended where a C0 knot line crosses the box: there no
    second-derivative bound exists and only the first two entries hold."""

    def __init__(self, uknots, vknots, patches, c0_u, c0_v, periods=(None, None)):
        """`patches`: per Bezier patch, row-major in u between consecutive
        knots, (rows, columns, poles row-major in u, weights)."""
        from ._buffers import native

        rows = patches
        self.native = native.SplineSurfaceBounds(
            list(uknots), list(vknots), rows, list(c0_u), list(c0_v), periods[0], periods[1]
        )
        self.whole = tuple(self.native.whole())
        self.domain = tuple(self.native.domain())
        self.periods = periods

    def local(self, u0, u1, v0, v1, limit=None):
        """Bounds on the UV box; with `limit`, from pieces only as fine as it
        takes to prove a facet spanning the box within it."""
        found, kinked = self.native.local(u0, u1, v0, v1, limit)
        return (*found, 'lipschitz') if kinked else tuple(found)

    def clamp(self, uv):
        """`uv` moved into the surface's domain (the surface, and so its
        bounds, exist only there); a periodic axis is left as it is."""
        return self.native.clamp(uv[0], uv[1])


def _patch_row(patch):
    nu, nv = patch.NbUPoles(), patch.NbVPoles()
    return (
        nu,
        nv,
        [patch.Pole(i, j).Coord() for i in range(1, nu + 1) for j in range(1, nv + 1)],
        [patch.Weight(i, j) for i in range(1, nu + 1) for j in range(1, nv + 1)],
    )


def _spline_bounds(adaptor):
    """`_SplineBounds` of a face on a B-spline or Bezier surface, or on the
    extrusion of a B-spline or Bezier curve (which is such a surface, linear
    along the extrusion); None for any other surface."""
    from OCP.GeomAbs import GeomAbs_BezierSurface, GeomAbs_BSplineSurface, GeomAbs_SurfaceOfExtrusion
    from OCP.GeomConvert import GeomConvert_BSplineSurfaceToBezierSurface

    kind = adaptor.GetType()
    if kind == GeomAbs_SurfaceOfExtrusion:
        return _extrusion_bounds(adaptor)
    if kind == GeomAbs_BezierSurface:
        patch = adaptor.Bezier()
        u0, u1, v0, v1 = patch.Bounds()
        return _SplineBounds([u0, u1], [v0, v1], [_patch_row(patch)], [], [])
    if kind != GeomAbs_BSplineSurface:
        return None
    spline = adaptor.BSpline()
    periods = (
        spline.UPeriod() if spline.IsUPeriodic() else None,
        spline.VPeriod() if spline.IsVPeriodic() else None,
    )
    c0_u = [spline.UKnot(i) for i in range(2, spline.NbUKnots()) if spline.UMultiplicity(i) >= spline.UDegree()]
    c0_v = [spline.VKnot(i) for i in range(2, spline.NbVKnots()) if spline.VMultiplicity(i) >= spline.VDegree()]
    if periods != (None, None):
        # The same geometry and parameters, written as one non-periodic
        # period. A periodic spline's first knot is a knot like any other:
        # only C0 there if its multiplicity says so.
        from OCP.Geom import Geom_BSplineSurface

        if periods[0] is not None and spline.UMultiplicity(1) >= spline.UDegree():
            c0_u.append(spline.UKnot(1))
        if periods[1] is not None and spline.VMultiplicity(1) >= spline.VDegree():
            c0_v.append(spline.VKnot(1))
        spline = spline.Copy()
        if not isinstance(spline, Geom_BSplineSurface):
            raise ValueError('CAD B-spline surface copy failed')
        if periods[0] is not None:
            spline.SetUNotPeriodic()
        if periods[1] is not None:
            spline.SetVNotPeriodic()
    converter = GeomConvert_BSplineSurfaceToBezierSurface(spline)
    nu, nv = converter.NbUPatches(), converter.NbVPatches()
    if nu < 1 or nv < 1:
        raise ValueError('CAD surface has no Bezier patches')
    uk, vk = _real_array(1, nu + 1), _real_array(1, nv + 1)
    converter.UKnots(uk)
    converter.VKnots(vk)
    return _SplineBounds(
        [uk.Value(i) for i in range(1, nu + 2)],
        [vk.Value(i) for i in range(1, nv + 2)],
        [_patch_row(converter.Patch(i, j)) for i in range(1, nu + 1) for j in range(1, nv + 1)],
        c0_u,
        c0_v,
        periods,
    )


def _extrusion_bounds(adaptor):
    """`_SplineBounds` of the extrusion F(u, v) = C(u) + v D of a B-spline or
    Bezier curve C: each Bezier arc of C with its poles at the two ends of
    the face's v range is a Bezier patch of F, of degree 1 in v. None for
    another curve, or a face unbounded in v.

    The derivatives of the whole curve at once (`_surface_bounds`) put the
    sharpest bend's facets on every span of it."""
    from OCP.GeomAbs import GeomAbs_BezierCurve, GeomAbs_BSplineCurve
    from OCP.GeomConvert import GeomConvert_BSplineCurveToBezierCurve

    basis = adaptor.BasisCurve()
    v0, v1 = adaptor.FirstVParameter(), adaptor.LastVParameter()
    if not (math.isfinite(v0) and math.isfinite(v1) and v1 > v0):
        return None
    # Linear in v, so a wider v range changes no bound, and a node a trim
    # puts a hair outside the face's own range is on the surface as it is.
    v0, v1 = v0 - (v1 - v0), v1 + (v1 - v0)
    direction = adaptor.Direction().Coord()

    def row(arc):
        poles = [arc.Pole(i).Coord() for i in range(1, arc.NbPoles() + 1)]
        weights = [arc.Weight(i) for i in range(1, arc.NbPoles() + 1)]
        return (
            len(poles),
            2,
            [tuple(x + v * d for x, d in zip(pole, direction)) for pole in poles for v in (v0, v1)],
            [weight for weight in weights for _ in (v0, v1)],
        )

    if basis.GetType() == GeomAbs_BezierCurve:
        arc = basis.Bezier()
        return _SplineBounds([arc.FirstParameter(), arc.LastParameter()], [v0, v1], [row(arc)], [], [])
    if basis.GetType() != GeomAbs_BSplineCurve:
        return None
    spline = basis.BSpline()
    period = spline.Period() if spline.IsPeriodic() else None
    c0 = [spline.Knot(i) for i in range(2, spline.NbKnots()) if spline.Multiplicity(i) >= spline.Degree()]
    if period is not None:
        from OCP.Geom import Geom_BSplineCurve

        if spline.Multiplicity(1) >= spline.Degree():
            c0.append(spline.Knot(1))
        spline = spline.Copy()
        if not isinstance(spline, Geom_BSplineCurve):
            raise ValueError('CAD B-spline curve copy failed')
        spline.SetNotPeriodic()
    converter = GeomConvert_BSplineCurveToBezierCurve(spline)
    count = converter.NbArcs()
    if count < 1:
        raise ValueError('CAD curve has no Bezier spans')
    knots = _real_array(1, count + 1)
    converter.Knots(knots)
    return _SplineBounds(
        [knots.Value(i) for i in range(1, count + 2)],
        [v0, v1],
        [row(converter.Arc(i)) for i in range(1, count + 1)],
        c0,
        [],
        (period, None),
    )


def _surface_bounds(adaptor, spline=None):
    """Bounds for |Fu|, |Fv|, |Fuu|, |Fuv|, |Fvv| in world coordinates.

    `spline` is the face's `_SplineBounds` where the caller already has it."""
    from OCP.GeomAbs import (
        GeomAbs_Plane,
        GeomAbs_Cylinder,
        GeomAbs_Cone,
        GeomAbs_Sphere,
        GeomAbs_Torus,
        GeomAbs_SurfaceOfExtrusion,
        GeomAbs_BSplineSurface,
        GeomAbs_BezierSurface,
    )

    kind = adaptor.GetType()
    if kind in (GeomAbs_BSplineSurface, GeomAbs_BezierSurface):
        return (spline or _spline_bounds(adaptor)).whole
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
    if len(bounds) > 5 and bounds[5] == 'lipschitz':
        # A triangle across a C0 line of a spline surface. F(p) differs from
        # each corner's F by at most |Fu| |du| + |Fv| |dv|, and the mean
        # absolute deviation of barycentric weights over a span is half it.
        return (bounds[0] * du + bounds[1] * dv) / 2
    if len(bounds) > 5 and bounds[5]:
        return bounds[0] * du / 2  # C0 spline: first-derivative Lipschitz bound.
    return (bounds[2] * du * du + 2 * bounds[3] * du * dv + bounds[4] * dv * dv) / 8


def _segment_distance(p, a, b):
    """Distance in the UV plane from p to the segment a-b."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    along = 0.0 if length2 <= 0 else min(1.0, max(0.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length2))
    return math.hypot(p[0] - a[0] - along * dx, p[1] - a[1] - along * dy)


def _trim_reach(curve):
    """`reach(t0, t1, a, b, shift)`: a bound on the distance, either way,
    between a trim's pcurve over [t0, t1] (moved by `shift`) and the chord
    a-b that stands for it in the UV mesh.

    A B-spline or Bezier pcurve is bounded by its own control polygon over
    just that span: the span is a chain of Bezier arcs, each inside the
    convex hull of its poles (weights are positive), and the distance to a
    segment is a convex function, so no point of the span is farther from the
    chord than its farthest pole. That converges quadratically to the span's
    real sag as the span shortens; a derivative bound of the whole curve does
    not (it was 50 to 400 times the real sag on imported spline trims, and
    the mesh was refined to match). The chord in turn is within that
    distance, plus the gap between its ends and the span's, of the span: the
    span is connected and joins the two ends, so every normal of the chord
    between the ends' feet meets it.

    Lines, circles and ellipses keep their closed-form sag."""
    from OCP.GeomAbs import GeomAbs_BSplineCurve, GeomAbs_BezierCurve

    speed, accel = _curve_bounds(curve, 2)
    kind = curve.GetType()
    poles = None
    if kind == GeomAbs_BSplineCurve:
        from OCP.Geom2dConvert import Geom2dConvert_BSplineCurveToBezierCurve

        spline = curve.BSpline()

        def poles(t0, t1):
            converter = Geom2dConvert_BSplineCurveToBezierCurve(spline, t0, t1, 1e-12)
            found = []
            for n in range(1, converter.NbArcs() + 1):
                arc = converter.Arc(n)
                found.extend(arc.Pole(i).Coord() for i in range(1, arc.NbPoles() + 1))
            return found

    elif kind == GeomAbs_BezierCurve:
        bezier = curve.Bezier()
        low, high = bezier.FirstParameter(), bezier.LastParameter()

        def poles(t0, t1):
            arc = bezier.Copy()
            arc.Segment((t0 - low) / (high - low), (t1 - low) / (high - low))
            return [arc.Pole(i).Coord() for i in range(1, arc.NbPoles() + 1)]

    def reach(t0, t1, a, b, shift):
        gap = max(
            math.dist(node, tuple(v + delta for v, delta in zip(curve.Value(t).Coord(), shift)))
            for node, t in ((a, t0), (b, t1))
        )
        if t1 <= t0:
            return gap
        if poles is not None:
            try:
                found = poles(t0, t1)
            except Exception:  # a span below the kernel's parametric resolution
                found = None
            if found:
                return gap + max(_segment_distance((u + shift[0], v + shift[1]), a, b) for u, v in found)
        return gap + (speed * (t1 - t0) / 2 if accel is None else accel * (t1 - t0) ** 2 / 8)

    return reach


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

    def __init__(self, adaptor, bounds, uv, indices, remaining, reserve, spline=None):
        self.adaptor = adaptor
        self.bounds = bounds
        self.spline = spline
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
        self.clamped = 0.0
        self.kept = {}

    def limits(self, triangle):
        """Derivative bounds that hold on a triangle."""
        if self.spline is not None:
            u = [self.uv[i][0] for i in triangle]
            v = [self.uv[i][1] for i in triangle]
            return self.spline.local(min(u), max(u), min(v), max(v), self.remaining)
        if self.local is None:
            return self.bounds
        v = [self.uv[i][1] for i in triangle]
        return self.local(min(v), max(v))

    def cost(self, a, b, bounds):
        a, b = self.uv[a], self.uv[b]
        du, dv = abs(a[0] - b[0]), abs(a[1] - b[1])
        if len(bounds) > 5 and bounds[5] == 'lipschitz':
            return bounds[0] * du + bounds[1] * dv
        if len(bounds) > 5 and bounds[5]:
            return bounds[0] * du
        return bounds[2] * du * du + 2 * bounds[3] * du * dv + bounds[4] * dv * dv

    def limits_of(self, k):
        """`limits` of facet k, kept until the facet is bisected."""
        triangle = self.triangles[k]
        kept = self.kept.get(k)
        if kept is None or kept[0] is not triangle:
            kept = self.kept[k] = (triangle, self.limits(triangle))
        return kept[1]

    def error(self, k):
        return _interpolation_error([self.uv[i] for i in self.triangles[k]], self.limits_of(k))

    def third(self, a, b):
        triangle = self.triangles[self.owner[(a, b)]]
        return triangle[(triangle.index(a) + 2) % 3]

    def area(self, a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    def inside(self, uv):
        """`uv` on the surface's domain. A spline surface exists only there:
        a point a trim puts a hair outside it is moved onto it, which moves
        the UV mesh's boundary by that much (charged at the surface's
        first-derivative bound)."""
        if self.spline is None:
            return uv
        inside = self.spline.clamp(uv)
        if inside != uv:
            self.clamped = max(self.clamped, math.hypot(*self.bounds[:2]) * math.dist(uv, inside))
        return inside

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


class _MesherMismatch(ValueError):
    """The kernel's triangulation of a face does not match the face's trims
    (or is missing): another triangulation of the solid may."""


def _face_data(face, cancel, spline=None):
    """Check UV mesh orientation/incidence and every trim's parameter chain."""
    from OCP.BRep import BRep_Tool
    from OCP.BRepAdaptor import BRepAdaptor_Surface, BRepAdaptor_Curve2d
    from OCP.BRepTools import BRepTools_WireExplorer
    from OCP.TopLoc import TopLoc_Location

    adaptor = BRepAdaptor_Surface(face.wrapped)
    if spline is None:
        spline = _spline_bounds(adaptor)
    bounds = _surface_bounds(adaptor, spline)
    if not all(math.isfinite(x) and x >= 0 for x in bounds[:5]):
        raise ValueError('nonfinite CAD derivative bound')
    local = _local_bounds(adaptor)
    floor = (
        0.0 if spline is None else max(spline.domain[1] - spline.domain[0], spline.domain[3] - spline.domain[2]) / 128
    )

    def speed(a, b, reach):
        """|F'| over the UV box that holds a and b, grown by `reach`."""
        if spline is not None:
            # No finer than 1/64 of the surface: a first-derivative bound
            # over a wider box holds, and a chord is no reason to resolve
            # the control nets below it.
            reach = max(reach, floor)
            low = spline.clamp((min(a[0], b[0]) - reach, min(a[1], b[1]) - reach))
            high = spline.clamp((max(a[0], b[0]) + reach, max(a[1], b[1]) + reach))
            found = spline.local(low[0], high[0], low[1], high[1])
        elif local is not None:
            found = local(min(a[1], b[1]) - reach, max(a[1], b[1]) + reach)
        else:
            found = bounds
        return math.hypot(found[0], found[1])

    location = TopLoc_Location()
    mesh = BRep_Tool.Triangulation_s(face.wrapped, location)
    if mesh is None or not mesh.HasUVNodes():
        raise _MesherMismatch('CAD mesher omitted face UV nodes')
    uv = [None] + [tuple(mesh.UVNode(i).Coord()) for i in range(1, mesh.NbNodes() + 1)]
    if any(not all(math.isfinite(v) for v in p) for p in uv[1:]):
        raise _MesherMismatch('nonfinite CAD UV mesh')
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
            raise _MesherMismatch('CAD UV mesh has reversed/degenerate triangles')
        indices.append(triangle)
        for p, q in zip(triangle, (*triangle[1:], triangle[0])):
            edges[tuple(sorted((p, q)))] += 1
            directions[(p, q)] += 1
    if any(n not in (1, 2) for n in edges.values()) or any(
        n == 2 and (directions[(p, q)] != 1 or directions[(q, p)] != 1) for (p, q), n in edges.items()
    ):
        raise _MesherMismatch('CAD UV mesh has inconsistent edge incidence')
    boundary = {edge for edge, n in edges.items() if n == 1}
    clamp_error = 0.0
    if spline is not None:
        # The mesher may put a node a hair outside a spline surface's domain
        # (it follows the pcurves, which loose CAD tolerances let stray). The
        # surface, and its control-net bounds, exist only on the domain, so
        # the node is moved onto it: 1e-4 of the span on the boundary, 1e-6
        # inside, more is refused. The move changes the UV mesh by that much,
        # charged at the first-derivative bound there; where a boundary node
        # then sits relative to its trim is measured below like any other.
        on_boundary = {i for edge in boundary for i in edge}
        low_u, high_u, low_v, high_v = spline.domain
        span = max(high_u - low_u, high_v - low_v)
        for i in range(1, len(uv)):
            inside = spline.clamp(uv[i])
            if inside != uv[i]:
                moved = math.dist(uv[i], inside)
                if moved > (1e-4 if i in on_boundary else 1e-6) * span:
                    raise _MesherMismatch(
                        f'CAD UV mesh node {uv[i]} is outside the spline surface domain '
                        f'[{low_u:g}, {high_u:g}] x [{low_v:g}, {high_v:g}]'
                    )
                clamp_error = max(clamp_error, speed(inside, inside, moved) * moved)
                uv[i] = inside
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
                raise _MesherMismatch('CAD trim has no mesh parameter correspondence')
            count = polygon.NbNodes()
            if count < 2:
                raise _MesherMismatch('invalid CAD trim node count')
            first, last = curve.FirstParameter(), curve.LastParameter()
            params = [polygon.Parameter(i) for i in range(1, count + 1)]
            guard = 1e-10 * max(1.0, abs(first), abs(last))
            if (
                abs(params[0] - first) > guard
                or abs(params[-1] - last) > guard
                or any(b < a for a, b in zip(params, params[1:]))
            ):
                raise _MesherMismatch('CAD trim parameter chain is incomplete')
            reach_of = _trim_reach(curve)
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
            # The chain's ends are within `guard` of the trim's: what the
            # pcurve can travel in that is part of every chord's reach.
            ends = _curve_bounds(curve, 2)[0] * (abs(params[0] - first) + abs(params[-1] - last))
            for p, q, t0, t1 in zip(ids, ids[1:], params, params[1:]):
                represented[tuple(sorted((p, q)))] += 1
                reach = ends + reach_of(t0, t1, uv[p], uv[q], shift)
                trim_error = max(trim_error, speed(uv[p], uv[q], reach) * reach)
                trim_segments += 1
            trims.append((edge, curve, tuple(shift), ids, params))
            explorer.Next()
    if set(represented) != boundary or any(n != 1 for n in represented.values()):
        raise _MesherMismatch('CAD trim chains do not cover mesh boundary exactly once')
    return adaptor, bounds, uv, indices, trim_error + clamp_error, trim_segments, trims


def _divide_at_c0(shape):
    """The solid with every face that is only C0 along a line divided there
    into C1 faces, and how many faces that added: a B-spline face with a
    knot of full multiplicity, or the extrusion of a B-spline curve with one.

    A second-derivative bound needs a C1 surface; across a C0 line only the
    first derivatives bound a facet, which takes facets as small as the error
    itself, over the whole face where the surface is an extrusion. The
    kernel's shape upgrade (ShapeUpgrade_ShapeDivideContinuity, the Open
    CASCADE tool for handing C0 geometry to formats and algorithms that need
    C1) splits such a face along the line into faces that share the new
    edge, each on its own segment of the surface, so the pieces are meshed
    like any other faces of the solid. It is not trusted: a C0 line it
    leaves inside a face is still bounded from first derivatives
    (`_SplineBounds.local`, `_surface_bounds`), and the result is meshed and
    checked for closure like any input. Solids without such faces are
    returned as they are."""
    import cadquery as cq
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_BSplineCurve, GeomAbs_BSplineSurface, GeomAbs_C1, GeomAbs_SurfaceOfExtrusion
    from OCP.ShapeUpgrade import ShapeUpgrade_ShapeDivideContinuity

    def kinked(face):
        adaptor = BRepAdaptor_Surface(face.wrapped)
        if adaptor.GetType() == GeomAbs_BSplineSurface:
            spline = adaptor.BSpline()
            return any(spline.UMultiplicity(i) >= spline.UDegree() for i in range(2, spline.NbUKnots())) or any(
                spline.VMultiplicity(i) >= spline.VDegree() for i in range(2, spline.NbVKnots())
            )
        if adaptor.GetType() == GeomAbs_SurfaceOfExtrusion:
            basis = adaptor.BasisCurve()
            if basis.GetType() == GeomAbs_BSplineCurve:
                spline = basis.BSpline()
                return any(spline.Multiplicity(i) >= spline.Degree() for i in range(2, spline.NbKnots()))
        return False

    faces = shape.Faces()
    if not any(kinked(face) for face in faces):
        return shape, 0
    divide = ShapeUpgrade_ShapeDivideContinuity(shape.wrapped)
    divide.SetSurfaceCriterion(GeomAbs_C1)
    if not divide.Perform():
        return shape, 0
    result = cq.Shape.cast(divide.Result())
    return result, len(result.Faces()) - len(faces)


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
    if not shape.Faces():
        raise ValueError('CAD surface has no faces')
    shape, divided = _divide_at_c0(shape)
    faces = shape.Faces()
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

    def name(index):
        return f'face {index} ({faces[index].geomType()}, {faces[index].Area():.3f} mm2)'

    splines = []
    for index, face in enumerate(faces):
        _check_cancel(cancel)
        adaptor = BRepAdaptor_Surface(face.wrapped)
        try:
            splines.append(_spline_bounds(adaptor))
            _surface_bounds(adaptor, splines[-1])
        except ValueError as error:
            raise ValueError(f'{name(index)}: {error}') from None
    # The kernel's triangulation is a seed that has to match every face's
    # trims, with the trims within half the budget. Where it does not, the
    # whole solid is triangulated again (never one face on its own: the faces
    # of an edge must share its nodes), finer, and then with the tolerances of
    # this private copy tightened: the kernel merges trim nodes closer than an
    # edge's tolerance, which on loosely toleranced imports loses short trims.
    # Tightening changes no geometry, and the error still charges the original
    # tolerances (`topology` above).
    #
    # A solid with spline faces is seeded coarser, edges and interior apart
    # (IMeshTools_Parameters): the trims' control-polygon bound needs no more
    # than the kernel's own edge deflection, and facets the kernel adds inside
    # a face at a fraction of the budget are facets the bound below never
    # asked for (twice to four times as many, on every spline solid tried).
    # Solids of planes and quadrics keep the seed they had.
    coarse = any(spline is not None for spline in splines)
    first_seed, step = (0.45 * budget, 2.0) if coarse else (budget / 4, 4.0)
    seed_deflection = first_seed
    problem = None
    tightened = False
    for attempt in range(8):
        _check_cancel(cancel)
        if attempt == 4:
            from OCP.ShapeFix import ShapeFix_ShapeTolerance
            from OCP.TopAbs import TopAbs_SHAPE

            ShapeFix_ShapeTolerance().LimitTolerance(shape.wrapped, 1e-7, 1e-7, TopAbs_SHAPE)
            tightened = True
            seed_deflection = first_seed
        BRepTools.Clean_s(shape.wrapped)
        if coarse:
            from OCP.IMeshTools import IMeshTools_Parameters

            parameters = IMeshTools_Parameters()
            parameters.Deflection = seed_deflection
            parameters.DeflectionInterior = 2 * budget
            parameters.Angle = 0.5
            parameters.AngleInterior = 1.0
            parameters.Relative = False
            parameters.InParallel = True
            BRepMesh_IncrementalMesh(shape.wrapped, parameters)
        else:
            BRepMesh_IncrementalMesh(shape.wrapped, seed_deflection, False, 0.5, True)
        data = []
        problem = None
        for index, face in enumerate(faces):
            try:
                row = _face_data(face, cancel, splines[index])
            except _MesherMismatch as error:
                problem = f'the CAD mesher cannot triangulate {name(index)} consistently with its trims: {error}'
                break
            except ValueError as error:
                if 'cancelled' in str(error):
                    raise
                raise ValueError(f'{name(index)}: {error}') from None
            if row[4] > budget / 2:
                problem = (
                    f'the trim approximation of {name(index)} is {row[4]:.3g} mm, more than half the '
                    f'{budget:.3g} mm that max_error_mm leaves after the CAD tolerances ({2 * topology:.3g} mm)'
                )
                break
            data.append(row)
        if problem is None:
            break
        seed_deflection /= step
    else:
        raise ValueError(
            f'{problem}, at every triangulation tried (down to a deflection of {seed_deflection * step:.3g} mm, '
            'also with tightened tolerances); request a larger max_error_mm or repair the face'
        )
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
        # What the edge curves may lie from this face: 0.1 um, or the CAD
        # tolerance where the solid declares more, within a quarter of what
        # is left. The distance is measured at every node on an edge.
        reserve = min(max(1e-4, topology), (budget - trim_error) / 4)
        mesh = _FaceMesh(adaptor, bounds, uv, indices, budget - trim_error - reserve, reserve, splines[index])
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
            worst = max(range(len(meshes)), key=lambda n: len(meshes[n].triangles))
            raise ValueError(
                f'CAD surface exceeds max_facets {max_facets} at requested error; '
                f'{name(worst)} has the most, {len(meshes[worst].triangles)} facets so far'
            )
        steps += 1
        if steps % 1024 == 0:
            _check_cancel(cancel)
        mesh, k = pending.pop()
        if mesh.error(k) <= mesh.remaining:
            continue
        if mesh.depth[k] >= 24:
            raise ValueError(f'{name(meshes.index(mesh))}: CAD surface interpolation exceeds refinement depth')
        triangle = mesh.triangles[k]
        limits = mesh.limits_of(k)
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
            middle = holder.inside(tuple(v + delta for v, delta in zip(curve.Value(tm).Coord(), shift)))
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
        if mesh.weld + mesh.clamped + collinear > mesh.reserve:
            raise ValueError(
                f'face {index}: facets in line at a pole move the mesh by {collinear:g} mm, more than the '
                'requested error leaves for it; request a larger max_error_mm'
            )
        max_error = max(max_error, max_interior + trim_error + mesh.weld + mesh.clamped + collinear + fixed_error)
        face_reports.append(
            dict(
                kind=face.geomType(),
                facets=counts[index],
                collapsed_pole_facets=collapsed,
                trim_segments=trim_segments,
                interpolation_error_mm=max_interior,
                trim_error_mm=trim_error + mesh.clamped,
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
                'analytic/Bezier derivative bounds for barycentric surface interpolation (B-spline surfaces '
                'per piece of each Bezier patch, divided into C1 faces at C0 knot lines); trims bounded by '
                'their closed-form sag or the control polygon of the span of each chord; '
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
            tolerances_tightened_for_meshing=tightened,
            faces_divided_at_c0=divided,
            extraction_ms=(time.perf_counter() - started) * 1000,
        ),
    )
