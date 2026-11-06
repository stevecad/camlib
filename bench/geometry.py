"""Frozen CAD-derived geometry for reproducible benchmarks.

Several workloads build their parts with CadQuery and export them with
``camlib.cadquery.export_surface_mesh`` / ``export_floor_region``. Different
CadQuery/OCP versions tessellate the same solid differently, which changes
the toolpaths slightly. So every exported mesh and region is recorded once
under ``bench/fixtures/`` and loaded from there on every machine: the inputs,
and therefore the toolpath data, are identical everywhere.

``install()`` replaces the two exporters (call it before any workload code
imports them). A missing fixture is an error; regenerate deliberately with
``CAMLIB_BENCH_REFRESH=1`` (and commit the result), never silently.
"""

from __future__ import annotations

import hashlib
import json
import os
from collections import Counter
from pathlib import Path

FIXTURES = Path(__file__).resolve().parent / "fixtures"
_seen = Counter()


def _path(kind, kwargs):
    text = json.dumps({k: v for k, v in kwargs.items() if k != "cancel"}, sort_keys=True, default=repr)
    # The same arguments may export different shapes: number the calls (the
    # order is deterministic within one workload process).
    _seen[(kind, text)] += 1
    key = f"{kind}|{text}|{_seen[(kind, text)]}"
    return FIXTURES / f"{kind}-{hashlib.sha256(key.encode()).hexdigest()[:16]}.json", key


def reset():
    """Start a new build: calls are numbered from 1 again. The runner calls
    this before every workload and target build (also between the timed
    repeats in one process), so every build loads the same fixtures."""
    _seen.clear()


def _cached(kind, export, encode, decode):
    def cached(shape, **kwargs):
        path, key = _path(kind, kwargs)
        if os.environ.get("CAMLIB_BENCH_REFRESH") == "1":
            value = export(shape, **kwargs)
            FIXTURES.mkdir(exist_ok=True)
            path.write_text(json.dumps(dict(key=key, value=encode(value))) + "\n")
            return value
        if not path.exists():
            raise FileNotFoundError(
                f"benchmark geometry fixture {path.name} is missing ({key}); "
                "regenerate with CAMLIB_BENCH_REFRESH=1 and commit it"
            )
        return decode(json.loads(path.read_text())["value"])

    return cached


def _encode_mesh(mesh):
    return dict(triangles=mesh.triangles, surface_error_mm=mesh.surface_error_mm, source=mesh.source)


def _decode_mesh(data):
    from camlib.surface import SurfaceMesh

    return SurfaceMesh(data["triangles"], data["surface_error_mm"], data["source"])


def _decode_region(data):
    from camlib.planar import PlanarRegion

    return PlanarRegion.from_dict(data)


def install():
    import camlib.cadquery as adapter

    if getattr(adapter, "_bench_frozen", False):
        return
    adapter.export_surface_mesh = _cached("mesh", adapter.export_surface_mesh, _encode_mesh, _decode_mesh)
    adapter.export_floor_region = _cached("region", adapter.export_floor_region, lambda r: r.to_dict(), _decode_region)
    adapter._bench_frozen = True
