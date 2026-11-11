"""Committed stock of a tri-dexel run (issue #1).

The same interface as ``simulation.StockState`` (the height-field stock of
milling and thermal runs), over the native tri-dexel receiver: every move
frame is checked (cursor, attribution, continuity, each changed ray's old
value, material only removed) against the receiver's own stock before it
commits.

Tri-dexel stock is three grids of rays through the stock box (along X, Y and
Z). Each ray covers its cell's rectangle and holds two sorted interval lists
along the ray: *possible* material (outside it the whole cross-section is
empty) and *certain* material (inside it the whole cross-section is
material). It holds material below empty space (undercuts), which a height
field cannot. ``heights`` is the run's height view: per Z ray, the top of
the certain material that starts at the stock bottom and the top of the
possible material, a valid height-field bound of the same material that
planners and height-field checks read.
"""

from __future__ import annotations

import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import tempfile

from ._buffers import native
from .simulation import (
    CHECKPOINT_LIMIT,
    Frame,
    ProtocolError,
    SimulationError,
    Update,
    _Decoder,
    _parsed_manifest,
)

DEXEL_CHUNK = 11
_AXES = {"x": 0, "y": 1, "z": 2, 0: 0, 1: 1, 2: 2}


def is_dexel(manifest):
    """A tri-dexel run manifest."""
    return (
        isinstance(manifest, dict)
        and isinstance(manifest.get("stock"), dict)
        and manifest["stock"].get("model") == "tri_dexel"
    )


class DexelView:
    """Read-only tri-dexel stock: committed (``DexelStockState``) or a
    partial move's preview."""

    def __init__(self, native_stock):
        self._stock = native_stock
        self._grid = json.loads(native_stock.grid_json())

    @property
    def grid(self):
        """Cells per axis, their widths, rays per grid and stored intervals."""
        return dict(self._grid)

    @property
    def heights(self):
        """The height view: little-endian f64 (lower, upper) per Z ray, in
        ``j * nx + i`` order (the height field's cell order)."""
        return memoryview(self._stock.view_bytes()).toreadonly()

    def ray(self, axis, index):
        """One ray's (possible, certain) intervals along its axis."""
        possible, certain = self._stock.ray(_AXES[axis], index)
        return tuple(possible), tuple(certain)

    def rays(self, axis):
        """One grid's rays as two buffers: per ray its possible and certain
        interval counts (little-endian u32 pairs), then every interval's
        (start, end) as little-endian f64, in ray order. A ray along axis
        ``a`` has index ``iv * cells[u] + iu`` with (u, v) = (a + 1, a + 2)
        mod 3."""
        counts, values = self._stock.rays(_AXES[axis])
        return memoryview(counts).toreadonly(), memoryview(values).toreadonly()

    def compare_solid(self, mesh, tolerance_mm):
        """This stock against a closed triangle mesh of the part
        (``camlib.surface.SurfaceMesh`` of a solid, e.g. from
        ``camlib.cadquery.export_surface_mesh``), whose true surface lies
        within the mesh's ``surface_error_mm``. Leftover material
        (``residual``) and over-cut (``gouge``) are each ``within_tolerance``
        (proven by the rays), ``definite_excess`` (with a witness point at
        least that deep) or ``possible_excess``, with raw volume bounds. It
        decides undercuts, which the height-field material verifier cannot."""
        data = bytearray()
        for triangle in mesh.triangles:
            for vertex in triangle:
                data += struct.pack("<3d", *vertex)
        try:
            return json.loads(self._stock.compare_solid(bytes(data), mesh.surface_error_mm, tolerance_mm))
        except ValueError as error:
            raise ProtocolError(str(error)) from error

    def volume(self):
        """Remaining and removed volume bounds: the largest certain and the
        smallest possible volume of the three grids (``grids_mm3``: per grid
        [certain, possible])."""
        return json.loads(self._stock.volume_json())


class DexelStockState(DexelView):
    """Transactional consumer of a tri-dexel run's frames."""

    # Partial moves are swept from this receiver's stock (see Player.seek).
    local_preview = True

    def __init__(self, manifest_frame, *, max_cells=0xFFFFFFFF):
        frame = Frame.parse(manifest_frame.raw) if isinstance(manifest_frame, Frame) else Frame.parse(manifest_frame)
        if frame.kind != 3:
            raise ProtocolError("expected bounded canonical manifest")
        self.canonical_manifest = frame.payload.tobytes()
        self._digest = hashlib.sha256(self.canonical_manifest).digest()
        self._manifest = _parsed_manifest(self._digest, self.canonical_manifest)
        if not is_dexel(self._manifest):
            raise ProtocolError("a tri-dexel stock state needs a tri-dexel run manifest")
        try:
            receiver = native.DexelReceiver(self.canonical_manifest.decode())
        except (ValueError, TypeError) as error:
            raise ProtocolError(str(error)) from error
        super().__init__(receiver)
        if sum(self._grid["rays"]) > max_cells:
            raise ProtocolError("stock exceeds explicit consumer cell budget")
        self._receiver = receiver
        m = self._manifest
        self._tools = frozenset(t["id"] for t in m["tools"])
        self._gauges = {t["id"]: t["gauge_length_mm"] for t in m["tools"]}
        self._tool_change = m.get("tool_change")
        view = self._grid["view"]
        self.min_mm, self.max_mm = tuple(view["min_mm"]), tuple(view["max_mm"])
        self.nx, self.ny = self._grid["cells"][0], self._grid["cells"][1]
        self.dx, self.dy = self._grid["step_mm"][0], self._grid["step_mm"][1]
        self.fraction = 0.0
        self._token = object()

    @property
    def completed(self):
        return self._receiver.completed

    @property
    def finished(self):
        return self._receiver.finished

    @property
    def position(self):
        return self._receiver.position

    @position.setter
    def position(self, value):
        # A tool change puts the tip at the manifest's tool-change pose; the
        # receiver accepts the next tool's first move from there itself.
        value = tuple(value)
        if value != tuple(self._receiver.position) and not any(
            # A change in place measures from the tool in the spindle (the
            # receiver's, which also checks the next move itself).
            self.tool_change_tip(tool, self._receiver.position, previous=self._receiver.last_tool) == value
            for tool in self._tools
        ):
            raise ProtocolError("position change is neither the cursor nor a declared tool-change pose")

    @property
    def checkpoint_bytes(self):
        """The rays' bytes in a checkpoint."""
        return 8 * sum(self._grid["rays"]) + 16 * self._grid["intervals"]

    @property
    def manifest(self):
        from ._geometry import resolve_geometry

        return resolve_geometry(json.loads(self.canonical_manifest))

    @property
    def view_spec(self):
        """The height view's stock specification (the Z rays' cells)."""
        return dict(self._grid["view"])

    @property
    def controls(self):
        return None

    @property
    def control_cursor(self):
        return None

    def tool_change_tip(self, tool, position=None, previous=None):
        """The tip position after a change to ``tool`` (the manifest's
        tool-change pose), or None when the tip stays where it is.
        ``previous``: the tool in the spindle before the change, which a
        change in place measures from (None: the run's first tool)."""
        if tool not in self._tools:
            raise ProtocolError("tool change to an undeclared tool")
        change = self._tool_change
        if change is None:
            return None
        if "in_place" in change:
            position = tuple(self.position if position is None else position)
            if previous is None:
                return position
            return (position[0], position[1], position[2] + (self._gauges[previous] - self._gauges[tool]))
        if "tips_mm" in change:
            if tool not in change["tips_mm"]:
                raise ProtocolError(f"the tool-change table declares no position after a change to tool {tool}")
            return tuple(float(v) for v in change["tips_mm"][tool])
        position = self.position if position is None else position
        xy = change.get("xy_mm") or position[:2]
        return (float(xy[0]), float(xy[1]), change["gauge_z_mm"] - self._gauges[tool])

    def _identity(self, decoder):
        run = self._manifest["run"]
        if (decoder.string(128), decoder.string(512), decoder.take(32).tobytes()) != (
            run["id"],
            run["revision"],
            self._digest,
        ):
            raise ProtocolError("stale/foreign run, revision or manifest")

    def _metadata_clone(self):
        return self.clone()

    def clone(self):
        other = object.__new__(DexelStockState)
        other.__dict__ = self.__dict__.copy()
        other.fraction = 0.0
        other._receiver = self._receiver.copy()
        other._stock = other._receiver
        return other

    def accept(self, frame, *, expect=None):
        frame = Frame.checked(frame)
        if frame.kind == 4:
            try:
                self._receiver.accept_end(frame.raw)
            except ValueError as error:
                raise ProtocolError(str(error)) from error
            return None
        if frame.kind != DEXEL_CHUNK:
            raise ProtocolError("expected a committed tri-dexel move or explicit finish")
        if expect is not None:
            d = _Decoder(frame.payload)
            self._identity(d)
            d.unpack("<3Q")
            motion = d.motion()
            if (motion["operation"], motion["tool"]) != expect:
                raise ProtocolError("generated motion attribution mismatch")
        try:
            text, count = self._receiver.accept(frame.raw)
        except ValueError as error:
            raise ProtocolError(str(error)) from error
        self._grid["intervals"] = json.loads(self._receiver.grid_json())["intervals"]
        motion = json.loads(text)
        self._token = object()
        return Update(motion, self.completed, 0.0, count, memoryview(b""), frame, None)

    def accept_many(self, buffer, offset, *, expect=None, limit=4096, verification=None, last_motion=None):
        """The consecutive tri-dexel move frames of ``buffer`` from ``offset``,
        with the verification record after each move when ``verification``
        (the native stream verifier) is given: (new offset, updates), each
        applied when it is taken (as ``StockState.accept_many``)."""
        frames = []
        view = memoryview(buffer)
        motion_frame = last_motion
        moves = 0
        while moves < limit and len(view) - offset >= 20:
            kind = int.from_bytes(view[offset + 10 : offset + 12], "little")
            length = int.from_bytes(view[offset + 12 : offset + 16], "little")
            if len(view) - offset < 20 + length:
                break
            if kind == DEXEL_CHUNK:
                moves += 1
            elif not (kind == 8 and verification is not None and (frames or motion_frame is not None)):
                break
            frames.append((kind, bytes(view[offset : offset + 20 + length])))
            offset += 20 + length

        self._buffered_motions = moves

        def updates():
            last = motion_frame
            for kind, raw in frames:
                if kind == 8:
                    try:
                        done, progress, check, kind_status = verification.accept(raw, self.completed, None, last)
                    except (ValueError, TypeError, OverflowError) as error:
                        raise ProtocolError(str(error)) from error
                    yield raw, done, progress, check, kind_status
                    continue
                update = self.accept(Frame.parse(raw), expect=expect)
                last = raw
                self._buffered_motions -= 1
                yield update

        return offset, updates()

    def preview(self, frame, fraction):
        """The stock shown ``fraction`` of the way through the next move
        (its committed frame): the dexel kernel on the move's prefix, from
        this stock. Returns a read-only ``DexelView``; this state is
        unchanged."""
        frame = Frame.checked(frame)
        if frame.kind != DEXEL_CHUNK or not 0 <= fraction <= 1:
            raise ProtocolError("a tri-dexel preview needs the next move's frame and a fraction in [0, 1]")
        try:
            shown = self._receiver.preview(frame.raw, fraction)
        except ValueError as error:
            raise ProtocolError(str(error)) from error
        view = DexelView(shown)
        view.fraction = fraction
        return view

    def write_checkpoint(self, path, *, compressed=False, cancel=None):
        if cancel is not None and cancel():
            raise SimulationError("checkpoint cancelled; committed prefix remains partial")
        data = bytes(self._receiver.checkpoint())
        if len(data) > CHECKPOINT_LIMIT:
            raise ProtocolError("checkpoint exceeds file byte budget")
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = tempfile.NamedTemporaryFile(dir=path.parent, prefix=".checkpoint-", delete=False)
        try:
            with temporary as f:
                if compressed:
                    with gzip.GzipFile(fileobj=f, mode="wb", compresslevel=1, mtime=0) as target:
                        target.write(data)
                else:
                    f.write(data)
            os.replace(temporary.name, path)
        finally:
            Path(temporary.name).unlink(missing_ok=True)

    def restore_checkpoint(self, path, *, compressed=False):
        opener = gzip.open(path, "rb") if compressed else Path(path).open("rb")
        with opener as f:
            data = f.read(CHECKPOINT_LIMIT + 1)
        if len(data) > CHECKPOINT_LIMIT:
            raise ProtocolError("checkpoint exceeds file byte budget")
        try:
            self._receiver = native.DexelReceiver.restore(self.canonical_manifest.decode(), data)
        except ValueError as error:
            raise ProtocolError(str(error)) from error
        self._stock = self._receiver
        self._grid = json.loads(self._receiver.grid_json())
        self.fraction = 0.0
        self._token = object()


def heights_at(view, x, y):
    """The height view's (lower, upper) bounds of the cell holding (x, y)."""
    g = view.grid
    spec = g["view"]
    nx, ny = g["cells"][0], g["cells"][1]
    i = min(nx - 1, max(0, math.floor((x - spec["min_mm"][0]) / g["step_mm"][0])))
    j = min(ny - 1, max(0, math.floor((y - spec["min_mm"][1]) / g["step_mm"][1])))
    return struct.unpack_from("<2d", view.heights, 16 * (j * nx + i))
