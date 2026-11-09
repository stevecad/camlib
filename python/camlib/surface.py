"""Authoritative faceted target, with a separate asserted smooth-target error."""

from dataclasses import dataclass
import itertools
import math

from ._resources import bounded_tuple


@dataclass(frozen=True)
class SurfaceMesh:
    """Triangle target with a finite asserted error in the inclusive range 0–0.1 mm."""

    triangles: object
    surface_error_mm: float = 0
    source: object = None

    def __post_init__(self):
        if not math.isfinite(self.surface_error_mm) or not 0 <= self.surface_error_mm <= 0.1:
            raise ValueError(
                f"surface_error_mm {self.surface_error_mm} must be finite and in 0..=0.1 mm; "
                "supply a mesh within that error limit"
            )
        # Any finite iterable; refused before it outgrows memory (~512 B/triangle).
        triangles = bounded_tuple(self.triangles, 512, "surface mesh triangles")
        if not triangles:
            raise ValueError("surface needs at least one triangle")
        frozen = []
        for triangle in triangles:
            points = tuple(itertools.islice(iter(triangle), 4))
            if len(points) != 3:
                raise ValueError("triangle needs exactly three points")
            points = tuple(tuple(itertools.islice(iter(p), 4)) for p in points)
            if any(len(p) != 3 for p in points):
                raise ValueError("surface point needs exactly three coordinates")
            frozen.append(points)
        object.__setattr__(self, "triangles", tuple(frozen))

    def to_dict(self):
        result = dict(version=1, units="mm", surface_error_mm=self.surface_error_mm, source=self.source)
        if len(self.triangles) < 1024:
            result["triangles"] = self.triangles
        else:
            # Indexed once per (immutable) mesh; shared read-only tuples.
            indexed = self.__dict__.get("_indexed")
            if indexed is None:
                import struct

                lookup = {}
                vertices = []
                indices = []
                for triangle in self.triangles:
                    row = []
                    for point in triangle:
                        key = struct.pack("<3d", *point)
                        if key not in lookup:
                            lookup[key] = len(vertices)
                            vertices.append(point)
                        row.append(lookup[key])
                    indices.append(tuple(row))
                indexed = (tuple(vertices), tuple(indices))
                object.__setattr__(self, "_indexed", indexed)
            result.update(vertices=indexed[0], indices=indexed[1])
        return result
