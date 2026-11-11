"""Synthetic CAM regression input: a stepped and drafted boss, as triangles.

A base slab with vertical walls from Z0 to Z3 and, on it, a boss whose walls
draft inward from Z3 to its top at Z9. Its outline differs at every depth.
"""

from camlib.surface import SurfaceMesh

BASE = (4.0, 6.0, 36.0, 34.0)
BASE_TOP = 3.0
FOOT = (10.0, 12.0, 30.0, 28.0)
CROWN = (14.0, 16.0, 26.0, 24.0)
CROWN_Z = 9.0


def _corners(rect, z):
    x0, y0, x1, y1 = rect
    return ((x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z))


def _quad(a, b, c, d):
    return [(a, b, c), (a, c, d)]


def _band(low, high):
    triangles = []
    for i in range(4):
        j = (i + 1) % 4
        triangles += _quad(low[i], low[j], high[j], high[i])
    return triangles


def part(surface_error_mm=0.001):
    triangles = _band(_corners(BASE, 0.0), _corners(BASE, BASE_TOP))
    triangles += _quad(*_corners(BASE, BASE_TOP))
    triangles += _band(_corners(FOOT, BASE_TOP), _corners(CROWN, CROWN_Z))
    triangles += _quad(*_corners(CROWN, CROWN_Z))
    return SurfaceMesh(tuple(triangles), surface_error_mm, source={"model": "stepped drafted boss"})


def outline(z):
    """The part's exact shadow at or above z: one rectangle (None above the top)."""
    if z <= BASE_TOP:
        return BASE
    if z <= CROWN_Z:
        t = (z - BASE_TOP) / (CROWN_Z - BASE_TOP)
        return tuple(f + t * (c - f) for f, c in zip(FOOT, CROWN))
    return None
