"""Committed stock of a lathe run.

The same interface as ``simulation.StockState`` (the height-field stock of
milling and thermal runs), over the native turning receiver: every turning
move frame is checked (cursor, attribution, continuity, each changed cell's
old value, material only removed) against the receiver's own stock before it
commits. Stock is axisymmetric: per z bin, bounds of the outer radius and of
the bore radius, plus rings certainly removed inside the material (face
grooves).
"""

from __future__ import annotations

import gzip
import hashlib
import json
import os
from pathlib import Path
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

TURN_CHUNK = 10


def is_turning(manifest):
    """A lathe run manifest (turning stock and setup)."""
    return isinstance(manifest, dict) and "turning" in manifest


class TurnStockState:
    """Transactional consumer of a lathe run's frames."""

    def __init__(self, manifest_frame, *, max_cells=0xFFFFFFFF):
        frame = Frame.parse(manifest_frame.raw) if isinstance(manifest_frame, Frame) else Frame.parse(manifest_frame)
        if frame.kind != 3:
            raise ProtocolError("expected bounded canonical manifest")
        self.canonical_manifest = frame.payload.tobytes()
        self._digest = hashlib.sha256(self.canonical_manifest).digest()
        self._manifest = _parsed_manifest(self._digest, self.canonical_manifest)
        if not is_turning(self._manifest):
            raise ProtocolError("a lathe stock state needs a lathe run manifest")
        try:
            self._receiver = native.TurnReceiver(self.canonical_manifest.decode())
        except (ValueError, TypeError) as error:
            raise ProtocolError(str(error)) from error
        stock = json.loads(self._receiver.stock_json())
        if len(stock["cells"]) > max_cells:
            raise ProtocolError("stock exceeds explicit consumer cell budget")
        self._initial = stock["remaining_upper_mm3"]
        self._tools = frozenset(t["id"] for t in self._manifest["tools"])
        self.fraction = 0.0
        self._token = object()

    # The committed cursor, from the receiver.
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
        # A tool change puts the controlled point at the turret pose; the
        # receiver accepts the next tool's first move from there itself.
        if tuple(value) != tuple(self._manifest["initial_position_mm"]):
            raise ProtocolError("a lathe tool change starts at the turret (start) position")

    @property
    def checkpoint_bytes(self):
        """The stock cells' and rings' bytes in a checkpoint."""
        stock = self.stock()
        return 32 * (len(stock["cells"]) + len(stock["rings"])) + 4

    @property
    def manifest(self):
        return json.loads(self.canonical_manifest)

    @property
    def controls(self):
        return None

    @property
    def control_cursor(self):
        return None

    def tool_change_tip(self, tool, position=None, previous=None):
        """A lathe indexes the turret at the start position: every tool's
        controlled point arrives there."""
        if tool not in self._tools:
            raise ProtocolError("tool change to an undeclared tool")
        return tuple(self._manifest["initial_position_mm"])

    def _identity(self, decoder):
        run = self._manifest["run"]
        if (decoder.string(128), decoder.string(512), decoder.take(32).tobytes()) != (
            run["id"],
            run["revision"],
            self._digest,
        ):
            raise ProtocolError("stale/foreign run, revision or manifest")

    # Lathe stock: shown at move boundaries (see Player.seek).
    turning = True

    def _metadata_clone(self):
        return self.clone()

    def clone(self):
        other = object.__new__(TurnStockState)
        other.__dict__ = self.__dict__.copy()
        other.__dict__.pop("_shown", None)
        other.fraction = 0.0
        other._receiver = self._receiver.copy()
        return other

    def accept(self, frame, *, expect=None):
        if "_shown" in self.__dict__:
            raise ProtocolError("a partial-move view accepts no frames")
        frame = Frame.checked(frame)
        if frame.kind == 4:
            try:
                self._receiver.accept_end(frame.raw)
            except ValueError as error:
                raise ProtocolError(str(error)) from error
            return None
        if frame.kind != TURN_CHUNK:
            raise ProtocolError("expected a committed turning move or explicit finish")
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
        motion = json.loads(text)
        self._token = object()
        return Update(motion, self.completed, 0.0, count, memoryview(b""), frame, None)

    def accept_many(self, buffer, offset, *, expect=None, limit=4096, verification=None, last_motion=None):
        """The consecutive turning move frames of ``buffer`` from ``offset``:
        (new offset, updates), each applied when it is taken (as
        ``StockState.accept_many``)."""
        if verification is not None:
            raise ProtocolError("lathe runs carry no milling verification records")
        frames = []
        view = memoryview(buffer)
        while len(frames) < limit and len(view) - offset >= 20:
            kind = int.from_bytes(view[offset + 10 : offset + 12], "little")
            length = int.from_bytes(view[offset + 12 : offset + 16], "little")
            if kind != TURN_CHUNK or len(view) - offset < 20 + length:
                break
            frames.append(bytes(view[offset : offset + 20 + length]))
            offset += 20 + length

        self._buffered_motions = len(frames)

        def updates():
            for raw in frames:
                update = self.accept(Frame.parse(raw), expect=expect)
                self._buffered_motions -= 1
                yield update

        return offset, updates()

    def preview(self, frame, fraction):
        """The stock shown ``fraction`` of the way through the next move
        (its committed frame): the turning planner's move kernel on the
        shortened move, from this stock. Returns a read-only view; this state
        is unchanged."""
        frame = Frame.checked(frame)
        if frame.kind != TURN_CHUNK or not 0 <= fraction <= 1:
            raise ProtocolError("a lathe preview needs the next move's frame and a fraction in [0, 1]")
        try:
            stock = json.loads(self._receiver.preview(frame.raw, fraction))
        except ValueError as error:
            raise ProtocolError(str(error)) from error
        shown = self.clone()
        shown._shown, shown.fraction = stock, fraction
        return shown

    def stock(self):
        """The turning stock: spec, cells (outer and bore radius bounds per z
        bin, each [lower, upper]), removed rings [z0, z1, r0, r1] and the
        remaining volume bounds."""
        shown = self.__dict__.get("_shown")
        return shown if shown is not None else json.loads(self._receiver.stock_json())

    def material(self):
        """The committed stock compared with the part: gouges are excluded
        per move as each is made (``gouge.proof``); leftover material per z
        bin, outside and in the bore, and per face groove (``residual``)."""
        if "_shown" in self.__dict__:
            raise ProtocolError("the material check reads committed stock")
        return json.loads(self._receiver.material())

    def volume(self):
        stock = self.stock()
        lower, upper = stock["remaining_lower_mm3"], stock["remaining_upper_mm3"]
        return dict(
            remaining_lower_mm3=lower,
            remaining_upper_mm3=upper,
            removed_lower_mm3=max(0.0, self._initial - upper),
            removed_upper_mm3=max(0.0, self._initial - lower),
        )

    def write_checkpoint(self, path, *, compressed=False, cancel=None):
        if "_shown" in self.__dict__:
            raise ProtocolError("a partial-move view cannot write a committed checkpoint")
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
            self._receiver = native.TurnReceiver.restore(self.canonical_manifest.decode(), data)
        except ValueError as error:
            raise ProtocolError(str(error)) from error
        self.__dict__.pop("_shown", None)
        self.fraction = 0.0
        self._token = object()
