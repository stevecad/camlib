"""Headless native stock streams. Geometry/sweeps stay in Rust; cell buffers are
binary f64 bounds. A consumer commits only complete, validated motion prefixes.
"""

from __future__ import annotations

from array import array
from dataclasses import asdict, dataclass
from contextlib import nullcontext
import gzip
import hashlib
import collections
import itertools
import json
import math
import mmap
import os
from pathlib import Path
import re
import struct
import subprocess
import sys
import tempfile
import threading
import uuid
import warnings
import zlib

from . import _resources
from ._buffers import native as _native_buffers
from ._runtime import executable as _executable

# Mirrors camlib::stream: the frame payload length field is a u32, the only
# frame-size invariant of the format.
FRAME_LIMIT = 20 + 0xFFFFFFFF
CHECKPOINT_LIMIT = FRAME_LIMIT
MANIFEST_LIMIT = 0xFFFFFFFF
_HEADER = struct.Struct("<8sHHII")
_CHANGE = struct.Struct("<Idddd")
_PAIR = struct.Struct("<dd")
_ID = re.compile(r"[A-Za-z0-9_.:-]{1,128}\Z")


class ProtocolError(ValueError):
    pass


class CamWarning(UserWarning):
    """A plan that is valid but probably not what was intended.

    Example: an operation that produced no cutting motion or removed no stock.
    Use warnings.simplefilter("error", CamWarning) to treat these as errors.
    """


class SimulationError(RuntimeError):
    pass


@dataclass(frozen=True)
class StockSpec:
    min_mm: tuple
    max_mm: tuple
    cell_size_mm: float = 0.1
    # Optional per-job budget; default is the format invariant (u32 cell
    # indices). Memory: 16 bytes per cell in the engine plus a Python copy.
    max_cells: int = 0xFFFFFFFF

    def to_dict(self):
        return {"version": 1, "units": "mm", **asdict(self)}


@dataclass(frozen=True)
class ToolChangePosition:
    """Where the machine leaves a new tool, in work coordinates: the spindle
    gauge line at ``gauge_z_mm`` (e.g. Z home, where M6 sends it) over
    ``xy_mm``, or over the position where the change was called (None).
    Operations with a new tool start from there; posts must declare the same
    machine behaviour (``Machine(tool_change_position_mode="z_home")``)."""

    gauge_z_mm: float
    xy_mm: tuple | None = None

    def __post_init__(self):
        values = (self.gauge_z_mm, *(self.xy_mm or ()))
        if (self.xy_mm is not None and len(self.xy_mm) != 2) or not all(
            isinstance(v, (int, float)) and math.isfinite(v) and abs(v) <= 1e6 for v in values
        ):
            raise ValueError("tool change position needs finite work coordinates")

    def to_dict(self):
        data = {"gauge_z_mm": float(self.gauge_z_mm)}
        if self.xy_mm is not None:
            data["xy_mm"] = [float(v) for v in self.xy_mm]
        return data


@dataclass(frozen=True)
class ToolChangeTable:
    """Where a tool change leaves each tool: its tip in work coordinates, per
    tool assembly id. This is the machine behaviour of posts with
    ``Machine(tool_change_position_mode="table")``, where each
    ``ToolSetting.position_after_mm`` states the tip after the change;
    ``ToolChangeTable.from_machine(machine)`` takes the poses from there.
    Operations with a new tool start at its pose."""

    tips_mm: tuple

    def __post_init__(self):
        items = self.tips_mm.items() if isinstance(self.tips_mm, dict) else self.tips_mm
        tips = tuple(sorted((str(tool), tuple(tip)) for tool, tip in items))
        if (
            not tips
            or len({tool for tool, _ in tips}) != len(tips)
            or not all(
                len(tip) == 3 and all(isinstance(v, (int, float)) and math.isfinite(v) and abs(v) <= 1e6 for v in tip)
                for _, tip in tips
            )
        ):
            raise ValueError("a tool-change table needs one finite tip position per tool")
        object.__setattr__(self, "tips_mm", tips)

    @classmethod
    def from_machine(cls, machine, tools=None):
        """The poses of a ``tool_change_position_mode="table"`` machine's tool
        table, for ``tools`` (assembly ids or assemblies; default: all)."""
        if machine.tool_change_position_mode != "table":
            raise ValueError(
                f"the machine's tool changes are {machine.tool_change_position_mode!r}, not a table of poses"
            )
        wanted = None if tools is None else {getattr(t, "id", t) for t in tools}
        # A machine lists each tool assembly once (Machine checks it).
        tips = {
            s.tool: tuple(float(v) for v in s.position_after_mm)
            for s in machine.tools
            if wanted is None or s.tool in wanted
        }
        missing = None if wanted is None else wanted - set(tips)
        if missing:
            raise ValueError(f"the machine's tool table has no entry for tool(s) {sorted(missing)}")
        return cls(tips)

    def tip(self, tool):
        return dict(self.tips_mm).get(tool)

    def to_dict(self):
        return {"tips_mm": {tool: [float(v) for v in tip] for tool, tip in self.tips_mm}}


@dataclass(frozen=True)
class SweepOptions:
    tolerance_mm: float = 1e-5
    chord_error_mm: float = 0.002
    max_tip_step_mm: float = 0.1
    max_segments: int = 100_000
    # u32 cell-index invariant; StockSpec.max_cells is the binding size limit.
    max_scratch_cells: int = 0xFFFFFFFF
    max_changed_cells: int = 0xFFFFFFFF
    max_cell_visits: int = 100_000_000


@dataclass(frozen=True)
class Frame:
    raw: bytes
    kind: int

    @property
    def payload(self):
        return memoryview(self.raw)[20:]

    @classmethod
    def parse(cls, raw, limit=FRAME_LIMIT):
        if len(raw) < 20:
            raise ProtocolError("truncated frame header")
        magic, version, kind, length, checksum = _HEADER.unpack_from(raw)
        if magic != b"CAMLSTRM" or version != 1 or kind not in (1, 2, 3, 4, 5, 6, 7, 8, 9, 10):
            raise ProtocolError("unsupported stream magic/version/kind")
        if not 20 <= limit <= CHECKPOINT_LIMIT or length > limit - 20:
            raise ProtocolError("frame exceeds explicit byte budget")
        if len(raw) != 20 + length:
            raise ProtocolError("frame payload length mismatch")
        crc = zlib.crc32(memoryview(raw)[:16])
        if zlib.crc32(memoryview(raw)[20:], crc) != checksum:
            raise ProtocolError("frame checksum mismatch")
        frame = cls(raw if type(raw) is bytes else bytes(raw), kind)
        # Only frames produced here are known-valid; directly constructed ones
        # are re-parsed by consumers.
        object.__setattr__(frame, "_validated", True)
        return frame

    @classmethod
    def checked(cls, frame):
        """Validated frame for raw bytes or a Frame; parse each frame once."""
        if isinstance(frame, Frame):
            return frame if frame.__dict__.get("_validated") else cls.parse(frame.raw)
        return cls.parse(frame)

    @classmethod
    def read(cls, source, limit=FRAME_LIMIT):
        header = _read_exact(source, 20, eof=True)
        if header is None:
            return None
        magic, version, kind, length, _ = _HEADER.unpack(header)
        if magic != b"CAMLSTRM" or version != 1 or kind not in (1, 2, 3, 4, 5, 6, 7, 8, 9, 10):
            raise ProtocolError("unsupported stream header")
        if not 20 <= limit <= CHECKPOINT_LIMIT or length > limit - 20:
            raise ProtocolError("frame exceeds explicit byte budget")
        return cls.parse(header + _read_exact(source, length), limit)


def _read_exact(source, count, *, eof=False):
    # Common case: one bounded read returns everything (no copies).
    if count <= 1 << 20:
        first = source.read(count)
        if len(first) == count:
            return first
        if not first and eof:
            return None
        output = bytearray(first)
    else:
        output = bytearray()
    # Grow with the bytes that actually arrive (bounded chunks): a corrupt
    # header claiming a huge length must not allocate that length up front.
    while len(output) < count:
        part = source.read(min(count - len(output), 1 << 20))
        if not part:
            if eof and not output:
                return None
            raise ProtocolError("truncated simulation stream")
        output += part
    return bytes(output)


class _Decoder:
    def __init__(self, data):
        self.data = memoryview(data)
        self.offset = 0

    def take(self, count):
        if count < 0 or count > len(self.data) - self.offset:
            raise ProtocolError("truncated binary field")
        value = self.data[self.offset : self.offset + count]
        self.offset += count
        return value

    def unpack(self, fmt):
        return struct.unpack(fmt, self.take(struct.calcsize(fmt)))

    def number(self, fmt):
        return self.unpack(fmt)[0]

    def string(self, limit):
        count = self.number("<H")
        if count > limit:
            raise ProtocolError("binary string exceeds byte limit")
        try:
            return self.take(count).tobytes().decode("utf-8")
        except UnicodeDecodeError as error:
            raise ProtocolError("invalid UTF-8 identifier") from error

    def finish(self):
        if self.offset != len(self.data):
            raise ProtocolError("trailing binary fields")

    def motion(self):
        identifier = self.number("<Q")
        operation, tool = self.string(128), self.string(128)
        kind = self.number("<B")
        feed_flag, feed = self.unpack("<Bd")
        curve_kind = self.number("<B")
        start, end = self.unpack("<3d"), self.unpack("<3d")
        if kind > 8 or curve_kind > 2 or feed_flag > 1 or (not feed_flag and feed != 0):
            raise ProtocolError("invalid motion/curve/feed discriminator")
        curve = {"kind": ("line", "arc", "helix")[curve_kind], "start": start, "end": end}
        if curve_kind:
            center = self.unpack("<3d")
            plane, sweep = self.unpack("<Bd")
            if plane > 2:
                raise ProtocolError("invalid arc plane")
            curve.update(center=center, plane=("xy", "xz", "yz")[plane], sweep_rad=sweep)
        return {
            "id": identifier,
            "operation": operation,
            "tool": tool,
            "kind": ("rapid", "cut", "plunge", "lead", "link", "retract", "tap", "shift", "thread")[kind],
            "feed_mm_min": feed if feed_flag else None,
            "curve": curve,
        }


def _point(point):
    return len(point) == 3 and all(
        isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) and abs(x) <= 1e6 for x in point
    )


def curve_length(curve, tolerance=1e-5):
    """Independent decoding checks; the native engine remains geometry authority."""
    start, end = curve["start"], curve["end"]
    if not _point(start) or not _point(end):
        raise ProtocolError("invalid motion endpoints")
    if curve["kind"] == "line":
        length = math.dist(start, end)
        if length < 1e-9:
            raise ProtocolError("zero-length motion")
        return length
    center, sweep = curve["center"], curve["sweep_rad"]
    axes = {"xy": (0, 1, 2), "xz": (2, 0, 1), "yz": (1, 2, 0)}
    if (
        curve["plane"] not in axes
        or not _point(center)
        or not math.isfinite(sweep)
        or not 1e-12 <= abs(sweep) <= 1000 * math.tau
    ):
        raise ProtocolError("invalid circular geometry")
    u, v, w = axes[curve["plane"]]
    radius = math.hypot(start[u] - center[u], start[v] - center[v])
    axial = end[w] - start[w]
    if (
        radius < 1e-6
        or center[w] != start[w]
        or (curve["kind"] == "arc" and axial != 0)
        or (curve["kind"] == "helix" and axial == 0)
    ):
        raise ProtocolError("invalid arc/helix radius or axial displacement")
    angle = math.atan2(start[v] - center[v], start[u] - center[u]) + sweep
    if (
        math.hypot(center[u] + radius * math.cos(angle) - end[u], center[v] + radius * math.sin(angle) - end[v])
        > tolerance
    ):
        raise ProtocolError("circular endpoint disagrees with sweep")
    return math.hypot(radius * sweep, axial)


def checked_length(curve):
    """Length of a curve already validated by decoding (no re-validation)."""
    start, end = curve["start"], curve["end"]
    if curve["kind"] == "line":
        return math.dist(start, end)
    u, v, w = {"xy": (0, 1, 2), "xz": (2, 0, 1), "yz": (1, 2, 0)}[curve["plane"]]
    center = curve["center"]
    radius = math.hypot(start[u] - center[u], start[v] - center[v])
    return math.hypot(radius * curve["sweep_rad"], end[w] - start[w])


def curve_position(curve, fraction):
    if not math.isfinite(fraction) or not 0 <= fraction <= 1:
        raise ProtocolError("invalid motion fraction")
    start, end = curve["start"], curve["end"]
    if fraction in (0, 1):
        return tuple(start if fraction == 0 else end)
    if curve["kind"] == "line":
        return tuple(a + fraction * (b - a) for a, b in zip(start, end))
    u, v, w = {"xy": (0, 1, 2), "xz": (2, 0, 1), "yz": (1, 2, 0)}[curve["plane"]]
    center = curve["center"]
    radius = math.hypot(start[u] - center[u], start[v] - center[v])
    angle = math.atan2(start[v] - center[v], start[u] - center[u]) + fraction * curve["sweep_rad"]
    p = list(start)
    p[u], p[v] = center[u] + radius * math.cos(angle), center[v] + radius * math.sin(angle)
    p[w] += fraction * (end[w] - start[w])
    return tuple(p)


@dataclass(frozen=True)
class Update:
    motion: dict
    completed: int
    fraction: float
    changed_count: int
    changes: memoryview
    frame: Frame
    dirty_rect: tuple | None = None


@dataclass(frozen=True)
class OperationUpdate:
    """Cursor-bound setup/shutdown events and geometric planning diagnostics."""

    operation: str
    completed: int
    stage: str
    events: tuple
    report: dict | None
    frame: Frame


@dataclass(frozen=True)
class ControlUpdate:
    """Declared state/pose effect with unchanged stock and completed-motion count.

    Physical tool-changer paths remain unverified; no connecting sweep is implied.
    The event dictionary is an independent snapshot, not the native state owner.
    """

    event: dict
    completed: int
    control_cursor: tuple
    position: tuple
    frame: Frame


@dataclass(frozen=True)
class VerificationUpdate:
    """Passive partial-check metadata, independent of stock/playback progress.

    ``check`` and ``progress`` are decoded from their native JSON on first
    access (most callers read neither for most motions).
    """

    completed: int
    _progress_json: bytes
    _check_json: bytes
    frame: Frame
    # The check's kind and the motion check's status as the native decoder
    # read them (None: not given, read from the check).
    _kind_status: tuple | None = None

    @property
    def kind(self):
        """The check's kind: motion, material_started or material."""
        return self._kind_status[0] if self._kind_status else self.check["kind"]

    @property
    def progress(self):
        cached = self.__dict__.get("_progress")
        if cached is None:
            cached = json.loads(self._progress_json)
            object.__setattr__(self, "_progress", cached)
        return cached

    @property
    def check(self):
        cached = self.__dict__.get("_check")
        if cached is None:
            cached = json.loads(self._check_json)
            object.__setattr__(self, "_check", cached)
        return cached

    @property
    def status(self):
        if self._kind_status:
            return self._kind_status[1]
        if self.check["kind"] != "motion":
            return None
        statuses = [r["status"] for r in (self.check.get("report"), self.check.get("stock_contact")) if r]
        if not statuses:
            return None
        return "violation" if "violation" in statuses else "uncertain" if "uncertain" in statuses else "clear"

    def findings(self):
        """Localized motion findings from each configured independent checker."""
        if self.check["kind"] != "motion":
            return
        if report := self.check.get("report"):
            for finding in report["findings"]:
                yield dict(finding, source="scene")
        if report := self.check.get("stock_contact"):
            for finding in report["findings"]:
                yield dict(
                    finding["finding"],
                    source="stock",
                    cell_range=finding["cell_range"],
                    contact_point_mm=finding["contact_point_mm"],
                )


class VerificationStream:
    def __init__(self, manifest_frame):
        frame = Frame.parse(manifest_frame.raw)
        if frame.kind != 3:
            raise ProtocolError("verification requires canonical manifest")
        self._native = _native_buffers.StreamVerification(frame.payload.tobytes().decode())

    def read(self, frame, completed, motion=None, *, retained=False, motion_frame=None):
        """``motion_frame``: raw committed chunk of the source motion (preferred,
        decoded natively); ``motion``: decoded motion dict (retained journals)."""
        try:
            motion_json = (
                None
                if motion is None or motion_frame is not None
                else json.dumps(motion, allow_nan=False, separators=(",", ":"))
            )
            if retained:
                done, progress, check, kind_status = self._native.decode(frame.raw, completed, motion_json)
            else:
                done, progress, check, kind_status = self._native.accept(
                    frame.raw, completed, motion_json, motion_frame
                )
        except (ValueError, TypeError, OverflowError) as error:
            raise ProtocolError(str(error)) from error
        return VerificationUpdate(done, bytes(progress), bytes(check), frame, kind_status)

    def finish(self, completed):
        try:
            self._native.finish(completed)
        except ValueError as error:
            raise ProtocolError(str(error)) from error


memory_budget = _resources.memory_budget


def _height_storage(scalars):
    budget = memory_budget()
    if budget is not None and scalars * 8 > budget:
        raise SimulationError(
            f"stock buffer needs about {scalars * 8 / 2**30:.1f} GiB but only {budget / 2**30:.1f} GiB "
            "is available; use a coarser cell_size_mm or raise CAMLIB_MEMORY_BUDGET_BYTES"
        )
    # OS-backed ownership returns pages when the final view is released. Large
    # short-lived seek arrays must not accumulate in libc's per-thread arenas.
    return memoryview(mmap.mmap(-1, scalars * 8)).cast("d")


_MANIFESTS = collections.OrderedDict()
_MANIFESTS_LOCK = threading.Lock()


def _parsed_manifest(digest, raw):
    """The parsed canonical manifest for these exact bytes (by their SHA-256),
    shared by the stock states of one run (each seek and playback session makes
    one). Read only: nothing writes into a stock state's manifest."""
    with _MANIFESTS_LOCK:
        parsed = _MANIFESTS.get(digest)
        if parsed is not None:
            _MANIFESTS.move_to_end(digest)
            return parsed
    parsed = json.loads(raw)
    with _MANIFESTS_LOCK:
        _MANIFESTS[digest] = parsed
        while len(_MANIFESTS) > 4:
            _MANIFESTS.popitem(last=False)
    return parsed


class StockState:
    """Transactional Python consumer of native bounds, with a read-only cell view."""

    def __init__(self, manifest_frame, *, max_cells=0xFFFFFFFF):
        frame = Frame.parse(manifest_frame.raw) if isinstance(manifest_frame, Frame) else Frame.parse(manifest_frame)
        if frame.kind != 3 or len(frame.payload) > MANIFEST_LIMIT:
            raise ProtocolError("expected bounded canonical manifest")
        self.canonical_manifest = frame.payload.tobytes()
        self._digest = hashlib.sha256(self.canonical_manifest).digest()
        self._manifest = _parsed_manifest(self._digest, self.canonical_manifest)
        m = self._manifest
        required = {"version", "run", "stock", "tools", "initial_position_mm", "sweep"}
        if (
            not required
            <= set(m)
            <= required | {"operations", "verification", "geometry", "tool_change", "rapid_retracts"}
            or type(m["version"]) is not int
            or m["version"] not in (1, 2)
        ):
            raise ProtocolError("unsupported manifest schema")
        run, spec = m["run"], m["stock"]
        if not _ID.fullmatch(run["id"]) or not 0 < len(run["revision"].encode()) <= 512:
            raise ProtocolError("invalid run identity")
        if spec["version"] != 1 or spec["units"] != "mm" or not _point(spec["min_mm"]) or not _point(spec["max_mm"]):
            raise ProtocolError("invalid stock specification")
        self.min_mm, self.max_mm = tuple(spec["min_mm"]), tuple(spec["max_mm"])
        spacing = spec["cell_size_mm"]
        if (
            not math.isfinite(spacing)
            or not 1e-5 <= spacing <= 1e6
            or any(b - a < 1e-6 for a, b in zip(self.min_mm, self.max_mm))
        ):
            raise ProtocolError("invalid stock extents/spacing")
        self.nx, self.ny = [max(1, math.ceil((self.max_mm[k] - self.min_mm[k]) / spacing)) for k in (0, 1)]
        count = self.nx * self.ny
        if type(spec["max_cells"]) is not int or not 0 < count <= spec["max_cells"] <= 0xFFFFFFFF or count > max_cells:
            raise ProtocolError("stock exceeds explicit consumer cell budget")
        self.dx, self.dy = [(self.max_mm[k] - self.min_mm[k]) / n for k, n in enumerate((self.nx, self.ny))]
        if not _point(m["initial_position_mm"]) or not m["tools"]:
            raise ProtocolError("invalid stream initial position/tools")
        self._tools = frozenset(t["id"] for t in m["tools"])
        if len(self._tools) != len(m["tools"]) or any(not _ID.fullmatch(t) for t in self._tools):
            raise ProtocolError("invalid/duplicate stream tool IDs")
        self.tolerance = m["sweep"]["tolerance_mm"]
        if not math.isfinite(self.tolerance) or not 1e-7 <= self.tolerance <= 0.1:
            raise ProtocolError("invalid sweep tolerance")
        self._control_context, self._control_wire = None, None
        if m["version"] == 2:
            try:
                self._control_context = _native_buffers.StreamControlContext(self.canonical_manifest.decode())
                self._control_wire = self._control_context.cursor_bytes()
            except (ValueError, TypeError, UnicodeError) as error:
                raise ProtocolError(str(error)) from error
        self._heights = _height_storage(count * 2)
        _native_buffers.initialise_heights(self._heights, self.min_mm[2], self.max_mm[2])
        self.completed, self.fraction, self.finished = 0, 0.0, False
        self.position = tuple(m["initial_position_mm"])
        self.last_motion_id = None
        self._last_tool = None
        self._gauges = {t["id"]: t["gauge_length_mm"] for t in m["tools"]}
        change = m.get("tool_change")
        if change is not None and (
            m["version"] != 1
            or (
                set(change) != {"tips_mm"}
                or not change["tips_mm"]
                or not set(change["tips_mm"]) <= set(self._gauges)
                or not all(len(t) == 3 and all(map(math.isfinite, t)) for t in change["tips_mm"].values())
            )
            if "tips_mm" in change
            else (set(change) - {"gauge_z_mm", "xy_mm"} or not math.isfinite(change["gauge_z_mm"]))
        ):
            raise ProtocolError("invalid tool change pose")
        self._tool_change = change
        self._token, self._initial_full_stock = object(), True

    def tool_change_tip(self, tool, position=None):
        """The tip position after a change to ``tool`` (the manifest's
        tool-change pose), or None when the tip stays where it is."""
        change = self._tool_change
        if change is None:
            return None
        if "tips_mm" in change:
            if tool not in change["tips_mm"]:
                raise ProtocolError(f"the tool-change table declares no position after a change to tool {tool}")
            return tuple(float(v) for v in change["tips_mm"][tool])
        position = self.position if position is None else position
        xy = change.get("xy_mm") or position[:2]
        return (float(xy[0]), float(xy[1]), change["gauge_z_mm"] - self._gauges[tool])

    @property
    def manifest(self):
        from ._geometry import resolve_geometry

        return resolve_geometry(json.loads(self.canonical_manifest))

    @property
    def heights(self):
        return memoryview(self._heights).toreadonly()

    @property
    def checkpoint_bytes(self):
        """The stock cells' bytes in a checkpoint."""
        return self.nx * self.ny * 16

    @property
    def controls(self):
        """Fresh declared control-state snapshot, or None for motion-only v1."""
        return None if self._control_context is None else json.loads(self._control_context.state_bytes())

    @property
    def control_cursor(self):
        return (
            None
            if self._control_wire is None
            else (struct.unpack_from('<Q', self._control_wire)[0], self._control_wire[8:].hex())
        )

    def _control_prefix(self, decoder):
        if self._control_wire is not None and decoder.take(40).tobytes() != self._control_wire:
            raise ProtocolError("control prefix does not match committed state")

    def clone(self):
        other = self._metadata_clone()
        other._heights = _height_storage(len(self._heights))
        other._heights[:] = self._heights
        return other

    def _metadata_clone(self):
        other = object.__new__(StockState)
        other.__dict__ = self.__dict__.copy()
        other._heights = array("d")
        return other

    def _identity(self, decoder):
        run = self._manifest["run"]
        if (decoder.string(128), decoder.string(512), decoder.take(32).tobytes()) != (
            run["id"],
            run["revision"],
            self._digest,
        ):
            raise ProtocolError("stale/foreign run, revision or manifest")

    def _motion(self, motion, *, curve_checked=False):
        if (
            self.finished
            or self.fraction != 0
            or not _ID.fullmatch(motion["operation"])
            or motion["tool"] not in self._tools
        ):
            raise ProtocolError("motion is not applicable to this committed state")
        start = tuple(motion["curve"]["start"])
        if (
            start != self.position
            and motion["tool"] != self._last_tool
            and start == self.tool_change_tip(motion["tool"])
        ):
            # The first motion with a new tool: the declared tool-change pose.
            self.position = start
        if start != self.position or self.last_motion_id is not None and motion["id"] <= self.last_motion_id:
            raise ProtocolError("motion continuity or ID order mismatch")
        if not curve_checked:
            # Native decoding applies these same rules when given a tolerance.
            curve_length(motion["curve"], self.tolerance)
            rapid = motion["kind"] in ("rapid", "shift")
            if rapid and motion["curve"]["kind"] != "line":
                raise ProtocolError("rapid clearance requires a line")
            feed = motion["feed_mm_min"]
            if feed is None and not rapid or feed is not None and (not math.isfinite(feed) or feed <= 0):
                raise ProtocolError("programmed feed must be positive and finite")
        if self._control_context is not None:
            try:
                self._control_context.check_tool(motion['tool'], motion['kind'] in ('rapid', 'shift'))
            except ValueError as error:
                raise ProtocolError(str(error)) from error

    def _delta(self, decoder, *, apply=True):
        generation, count = decoder.unpack("<QI")
        if (
            generation != self.completed
            or count > self.nx * self.ny
            or len(decoder.data) - decoder.offset != count * 36
        ):
            raise ProtocolError("delta generation/count/length mismatch")
        offset = 20 + decoder.offset
        raw = decoder.data.obj
        if not isinstance(raw, bytes) or len(raw) != len(decoder.data) + 20:
            raise ProtocolError("delta must borrow an immutable complete frame")
        changes = decoder.take(count * 36)
        decoder.finish()
        try:
            kernel = _native_buffers.apply_delta if apply else _native_buffers.validate_delta
            dirty = kernel(self._heights, raw, offset, count, self.nx, (self.min_mm[2], self.max_mm[2]))
        except (ValueError, TypeError, BufferError) as error:
            raise ProtocolError(str(error)) from error
        if apply:
            self._token, self._initial_full_stock = object(), False
        return count, changes, dirty

    def _check_end(self, frame):
        if frame.kind != 4:
            raise ProtocolError("expected explicit finish frame")
        d = _Decoder(frame.payload)
        self._identity(d)
        (count,) = d.unpack("<Q")
        position = d.unpack("<3d")
        self._control_prefix(d)
        d.finish()
        if self.finished or self.fraction != 0 or count != self.completed or position != self.position:
            raise ProtocolError("finish does not match complete committed prefix")

    def accept(self, frame, *, expect=None):
        """``expect``: optional (operation, tool) the chunk must be attributed
        to, checked after decoding and before any state change."""
        frame = Frame.checked(frame)
        if frame.kind == 4:
            self._check_end(frame)
            self.finished = True
            return None
        if frame.kind == 9:
            if self._control_context is None or self.finished or self.fraction:
                raise ProtocolError("control transfer requires a live version 2 committed state")
            try:
                candidate, position, event = self._control_context.accepted(frame.raw, self.completed, self.position)
                wire = candidate.cursor_bytes()
                event = json.loads(event)
            except (ValueError, TypeError, OverflowError) as error:
                raise ProtocolError(str(error)) from error
            cursor = (struct.unpack_from('<Q', wire)[0], wire[8:].hex())
            update = ControlUpdate(event, self.completed, cursor, position, frame)
            self._control_context, self._control_wire, self.position = candidate, wire, position
            self._token = object()
            return update
        if frame.kind != 1:
            raise ProtocolError("expected committed chunk or explicit finish")
        payload = frame.payload
        run = self._manifest["run"]
        try:
            sequence, start, end, motion, offset = _native_buffers.decode_committed_chunk(
                frame.raw, 20, run["id"], run["revision"], self._digest, self._control_wire, self.tolerance
            )
        except ValueError as error:
            raise ProtocolError(str(error)) from error
        if sequence != self.completed or start != sequence or end != start + 1:
            raise ProtocolError("missing, repeated or out-of-order chunk")
        if expect is not None and (motion["operation"], motion["tool"]) != expect:
            raise ProtocolError("generated motion attribution mismatch")
        self._motion(motion, curve_checked=True)
        d = _Decoder(payload)
        d.offset = offset
        count, changes, dirty = self._delta(d)
        self.completed, self.last_motion_id = end, motion["id"]
        self.position = tuple(motion["curve"]["end"])
        self._last_tool = motion["tool"]
        return Update(motion, end, 0.0, count, changes, frame, dirty)

    def accept_many(self, buffer, offset, *, expect=None, limit=4096, verification=None, last_motion=None):
        """Check the consecutive committed-motion frames of ``buffer`` from
        ``offset`` natively, with exactly the checks of ``accept``. Returns
        (new offset, updates): each update's stock delta is applied when it
        is taken from the ``updates`` generator, so the state always matches
        the last update handed out; the first invalid frame ends the run with
        its error, raised after the valid updates. With ``verification`` (the
        native stream verifier) the record after each motion is accepted too
        and handed out as (raw, completed, progress JSON, check JSON).
        """
        if self.finished or self.fraction != 0:
            raise ProtocolError("motion is not applicable to this committed state")
        run = self._manifest["run"]
        try:
            offset, accepted, failure = _native_buffers.accept_motion_frames(
                buffer,
                offset,
                run["id"],
                run["revision"],
                self._digest,
                self._control_wire,
                self.tolerance,
                self.nx * self.ny,
                self.completed,
                self.position,
                self.last_motion_id,
                set(self._tools),
                expect,
                self._control_context,
                verification,
                last_motion,
                limit,
                None
                if self._tool_change is None or "tips_mm" in self._tool_change
                else (
                    float(self._tool_change["gauge_z_mm"]),
                    None if self._tool_change.get("xy_mm") is None else tuple(self._tool_change["xy_mm"]),
                    self._gauges,
                ),
                self._last_tool,
                None
                if self._tool_change is None or "tips_mm" not in self._tool_change
                else {k: tuple(map(float, v)) for k, v in self._tool_change["tips_mm"].items()},
            )
        except (ValueError, TypeError) as error:
            raise ProtocolError(str(error)) from error

        self._buffered_motions = sum(item[0] != 8 for item in accepted)

        def updates():
            limits = (self.min_mm[2], self.max_mm[2])
            for item in accepted:
                if item[0] == 8:
                    yield item[1:]
                    continue
                _, raw, motion, end, count, at = item
                try:
                    dirty = _native_buffers.apply_delta(self._heights, raw, at, count, self.nx, limits)
                except (ValueError, TypeError, BufferError) as error:
                    raise ProtocolError(str(error)) from error
                self._token, self._initial_full_stock = object(), False
                self.completed, self.last_motion_id = end, motion["id"]
                self.position = tuple(motion["curve"]["end"])
                self._last_tool = motion["tool"]
                frame = Frame(raw, 1)
                object.__setattr__(frame, "_validated", True)
                self._buffered_motions -= 1
                yield Update(motion, end, 0.0, count, memoryview(raw)[at : at + count * 36], frame, dirty)
            if failure is not None:
                raise ProtocolError(failure)

        return offset, updates()

    def partial(self, frame, *, compact=False):
        frame = Frame.parse(frame.raw) if isinstance(frame, Frame) else Frame.parse(frame)
        if frame.kind != 5:
            raise ProtocolError("expected noncommitting prefix preview")
        d = _Decoder(frame.payload)
        self._identity(d)
        sequence = d.number("<Q")
        self._control_prefix(d)
        motion = d.motion()
        fraction = d.number("<d")
        position = d.unpack("<3d")
        if sequence != self.completed or not math.isfinite(fraction) or not 0 <= fraction <= 1:
            raise ProtocolError("partial cursor mismatch")
        self._motion(motion)
        expected = curve_position(motion["curve"], fraction)
        if not _point(position) or math.dist(expected, position) > self.tolerance:
            raise ProtocolError("partial tool position disagrees with cursor")
        if compact and fraction:
            offset = 20 + d.offset + 12
            count, changes, dirty = self._delta(d, apply=False)
            shown = PartialStockView._from_validated(self, frame, offset, count, fraction, position)
        else:
            shown = self.clone()
            count, changes, dirty = shown._delta(d)
            shown.fraction, shown.position = fraction, position
        return shown, Update(motion, sequence, fraction, count, changes, frame, dirty)

    def volume(self):
        values = _native_buffers.volume(self._heights, self.min_mm[2], self.max_mm[2], self.dx * self.dy)
        return dict(
            zip(("remaining_lower_mm3", "remaining_upper_mm3", "removed_lower_mm3", "removed_upper_mm3"), values)
        )

    def write_checkpoint(self, path, *, compressed=False, cancel=None):
        """Write the negotiated checkpoint to disk without a dense pipe frame."""
        if self.fraction != 0:
            raise ProtocolError("checkpoint requires a complete move cursor")

        def check_cancel():
            if cancel is not None and cancel():
                raise SimulationError("checkpoint cancelled; committed prefix remains partial")

        check_cancel()
        run = self._manifest["run"]
        prefix = bytearray()
        for s in (run["id"], run["revision"]):
            raw = s.encode()
            prefix.extend(struct.pack("<H", len(raw)))
            prefix.extend(raw)
        prefix.extend(self._digest)
        prefix.extend(
            struct.pack(
                "<Q3dBQQI",
                self.completed,
                *self.position,
                int(self.last_motion_id is not None),
                self.last_motion_id or 0,
                self.completed,
                self.nx * self.ny,
            )
        )
        if self._control_context is not None:
            state = self._control_context.state_bytes()
            prefix.extend(struct.pack('<I', len(state)))
            prefix.extend(state)
        heights = self._heights
        if sys.byteorder != "little":
            heights = array("d", heights)
            heights.byteswap()
        data = memoryview(heights).cast("B")
        length = len(prefix) + len(data)
        if length + 20 > CHECKPOINT_LIMIT:
            raise ProtocolError("checkpoint exceeds file byte budget")
        header = _HEADER.pack(b"CAMLSTRM", 1, 2, length, 0)
        crc = zlib.crc32(prefix, zlib.crc32(header[:16]))
        for offset in range(0, len(data), 1024 * 1024):
            check_cancel()
            crc = zlib.crc32(data[offset : offset + 1024 * 1024], crc)
        check_cancel()
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = tempfile.NamedTemporaryFile(dir=path.parent, prefix=".checkpoint-", delete=False)
        try:
            with temporary as f:
                wrapper = (
                    gzip.GzipFile(fileobj=f, mode="wb", compresslevel=1, mtime=0) if compressed else nullcontext(f)
                )
                with wrapper as target:
                    target.write(header[:16] + struct.pack("<I", crc))
                    target.write(prefix)
                    for offset in range(0, len(data), 1024 * 1024):
                        check_cancel()
                        target.write(data[offset : offset + 1024 * 1024])
            check_cancel()
            os.replace(temporary.name, path)
        finally:
            Path(temporary.name).unlink(missing_ok=True)

    def restore_checkpoint(self, path, *, compressed=False):
        # Read bounded disk blocks; ordinary motion transport remains <=8 MiB.
        opener = gzip.open(path, "rb") if compressed else Path(path).open("rb")
        with opener as f:
            header = _read_exact(f, 20)
            magic, version, kind, length, checksum = _HEADER.unpack(header)
            if (magic, version, kind) != (b"CAMLSTRM", 1, 2) or length + 20 > CHECKPOINT_LIMIT:
                raise ProtocolError("invalid checkpoint header/budget")
            # Identity lengths are bounded before reading metadata/cell storage.
            crc = zlib.crc32(header[:16])
            pieces = []
            for maximum in (128, 512):
                size = _read_exact(f, 2)
                (count,) = struct.unpack("<H", size)
                if count > maximum:
                    raise ProtocolError("checkpoint identity exceeds byte limit")
                pieces.extend((size, _read_exact(f, count)))
            pieces.append(_read_exact(f, 32 + struct.calcsize("<Q3dBQQI")))
            prefix = b"".join(pieces)
            d = _Decoder(prefix)
            self._identity(d)
            complete, x, y, z, has_last, last, generation, count = d.unpack("<Q3dBQQI")
            d.finish()
            candidate = self._control_context
            if candidate is not None:
                size = _read_exact(f, 4)
                (n,) = struct.unpack('<I', size)
                if n > 4096 or len(prefix) + 4 + n > length:
                    raise ProtocolError('control checkpoint state exceeds bounded size')
                raw = _read_exact(f, n)
                prefix += size + raw
                try:
                    candidate = candidate.restored(raw, complete, (x, y, z))
                except (ValueError, TypeError, OverflowError) as error:
                    raise ProtocolError(str(error)) from error
            if (
                generation != complete
                or has_last not in (0, 1)
                or (has_last == 0) != (complete == 0)
                or not _point((x, y, z))
                or count != self.nx * self.ny
                or length != len(prefix) + count * 16
                or complete == 0
                and last != 0
                or complete == 0
                and candidate is None
                and (x, y, z) != tuple(self._manifest["initial_position_mm"])
            ):
                raise ProtocolError("invalid checkpoint cursor/grid")
            crc = zlib.crc32(prefix, crc)
            heights = _height_storage(count * 2)
            destination = memoryview(heights).cast("B")
            remaining = count * 16
            offset = 0
            while remaining:
                size = min(1024 * 1024, remaining)
                block = destination[offset : offset + size]
                filled = 0
                while filled < size:
                    got = f.readinto(block[filled:])
                    if not got:
                        raise ProtocolError("truncated checkpoint cells")
                    filled += got
                crc = zlib.crc32(block, crc)
                remaining -= size
                offset += size
            if f.read(1) or crc != checksum:
                raise ProtocolError("checkpoint checksum/length mismatch")
        if sys.byteorder != "little":
            swapped = array("d", heights)
            swapped.byteswap()
            heights[:] = swapped
        try:
            _native_buffers.validate_heights(heights, self.min_mm[2], self.max_mm[2])
        except (ValueError, TypeError, BufferError) as error:
            raise ProtocolError(str(error)) from error
        wire = None if candidate is None else candidate.cursor_bytes()
        self._heights, self.completed, self.position = heights, complete, (x, y, z)
        self._control_context, self._control_wire = candidate, wire
        self.last_motion_id, self.fraction, self.finished = last if has_last else None, 0.0, False
        self._token, self._initial_full_stock = object(), False


def stock_state(manifest_frame, *, max_cells=0xFFFFFFFF):
    """The committed-stock consumer of a run: a height field (milling,
    thermal cutting) or turning stock (a lathe run)."""
    frame = Frame.checked(manifest_frame)
    from .turn_state import TurnStockState, is_turning

    if frame.kind == 3 and is_turning(
        _parsed_manifest(hashlib.sha256(frame.payload).digest(), frame.payload.tobytes())
    ):
        return TurnStockState(frame, max_cells=max_cells)
    return StockState(frame, max_cells=max_cells)


class PartialStockView:
    """Borrow a committed base and validated prefix; materialize only on request.

    Player keeps the base unchanged while this view is displayed. Direct callers
    must also keep that base unchanged; mutation invalidates the borrowed view.
    """

    def __init__(self, *_, **__):
        raise TypeError("use StockState.partial(..., compact=True) to validate a preview")

    def __setattr__(self, name, value):
        raise AttributeError("partial stock views are read-only")

    @classmethod
    def _from_validated(cls, base, frame, offset, count, fraction, position):
        view = object.__new__(cls)
        fields = dict(
            _base=base,
            _frame=frame,
            _base_token=base._token,
            _offset=offset,
            _count=count,
            fraction=fraction,
            position=position,
            finished=False,
            _token=object(),
        )
        for name in (
            "canonical_manifest",
            "_digest",
            "nx",
            "ny",
            "dx",
            "dy",
            "min_mm",
            "max_mm",
            "tolerance",
            "completed",
            "last_motion_id",
            "_control_context",
            "_control_wire",
        ):
            fields[name] = getattr(base, name)
        for name, value in fields.items():
            object.__setattr__(view, name, value)
        return view

    def _check_base(self):
        if self._base._token is not self._base_token:
            raise ProtocolError("partial view base changed; reconstruct from its move-start stock")

    @property
    def manifest(self):
        from ._geometry import resolve_geometry

        return resolve_geometry(json.loads(self.canonical_manifest))

    @property
    def controls(self):
        self._check_base()
        return StockState.controls.fget(self)

    @property
    def control_cursor(self):
        self._check_base()
        return StockState.control_cursor.fget(self)

    @property
    def heights(self):
        return self.clone().heights

    def clone(self):
        self._check_base()
        return self._base.partial(self._frame)[0]

    def volume(self):
        return self.clone().volume()

    def display_patch(self, display, dirty):
        self._check_base()
        try:
            return _native_buffers.display_overlay_patch(
                self._base.heights,
                self._frame.raw,
                (self._offset, self._count),
                (self.nx, *display),
                dirty,
                (self.min_mm[2], self.max_mm[2]),
            )
        except (ValueError, TypeError, BufferError) as error:
            raise ProtocolError(str(error)) from error

    def write_checkpoint(self, *_, **__):
        raise ProtocolError("checkpoint requires a complete move cursor")


def motion_dict(motion):
    data = motion.to_dict() if hasattr(motion, "to_dict") else motion
    return data["motion"] if data.get("kind") == "motion" else data


def event_dict(event):
    data = event.to_dict() if hasattr(event, 'to_dict') else event
    return dict(kind='motion', motion=data) if isinstance(data, dict) and 'curve' in data else data


@dataclass(frozen=True)
class MaterialStage:
    """Check a manifest-declared stage after all preceding explicit motions."""

    after_operation: str


class Simulation:
    """Synchronous headless API; each yielded update includes authoritative stock.

    Drain a batch iterator before another command. Use a retained Run/Player for
    independent production and playback. Only cancel/close may cross threads.
    """

    def __init__(
        self,
        stock,
        tools,
        *,
        revision,
        initial_position_mm,
        run_id=None,
        sweep=None,
        executable=None,
        operations=(),
        verification=None,
        stream_version=1,
        regeneration=None,
        tool_change=None,
        rapid_retracts=True,
        turning=None,
        _on_spawn=None,
    ):
        if type(stream_version) is not int or stream_version not in (1, 2) or stream_version == 2 and operations:
            raise ValueError('stream_version must be 1 or 2; version 2 uses authored typed events')
        manifest = {
            "version": stream_version,
            "run": {"id": run_id or uuid.uuid4().hex, "revision": revision},
            "stock": stock.to_dict() if hasattr(stock, "to_dict") else stock,
            "tools": [t.to_dict() if hasattr(t, "to_dict") else t for t in tools],
            "initial_position_mm": initial_position_mm,
            "sweep": dict(sweep) if isinstance(sweep, dict) else asdict(sweep or SweepOptions()),
        }
        if operations:
            if not isinstance(operations, (list, tuple)):
                raise TypeError("operations must be a list or tuple (a finite collection)")
            definitions = list(operations)
            manifest["operations"] = [op.to_dict() if hasattr(op, "to_dict") else op for op in definitions]
        if verification is not None:
            manifest["verification"] = verification.to_dict() if hasattr(verification, "to_dict") else verification
        if tool_change is not None:
            manifest["tool_change"] = tool_change.to_dict() if hasattr(tool_change, "to_dict") else dict(tool_change)
        if type(rapid_retracts) is not bool:
            raise ValueError("rapid_retracts must be a bool")
        if turning is not None:
            # A lathe run: its turning setup plans and checks every move.
            if operations or verification is not None or tool_change is not None:
                raise ValueError("a lathe run takes its operations from its turning setup (no milling checks/poses)")
            manifest["turning"] = turning
        elif rapid_retracts:
            manifest["rapid_retracts"] = True
        from ._geometry import share_geometry

        manifest = share_geometry(manifest)
        self._regeneration = regeneration
        self.executable = _executable("cam-sim-stream", executable)
        self._errors = tempfile.TemporaryFile()
        try:
            self._process = subprocess.Popen(
                [str(self.executable)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self._errors
            )
        except BaseException:
            self._errors.close()
            raise
        self._pending, self.cancelled, self._closed = 0, False, False
        self._buffer, self._at = b"", 0
        try:
            if _on_spawn is not None:
                _on_spawn(self)
            self._send(manifest)
            frame = self._next()
            self.manifest_frame = frame
            self.state = stock_state(frame, max_cells=manifest["stock"]["max_cells"])
            self._verification = VerificationStream(frame) if "verification" in self.state._manifest else None
            self._last_motion, self.verification_update = None, None
            self._last_motion_frame = None
        except BaseException:
            self.close()
            raise

    def _send(self, value):
        raw = json.dumps(value, allow_nan=False, separators=(",", ":")).encode() + b"\n"
        try:
            self._process.stdin.write(raw)
            self._process.stdin.flush()
        except (OSError, ValueError) as error:
            raise SimulationError(self._diagnostic()) from error

    def _diagnostic(self):
        if self.cancelled:
            return "simulation cancelled; prefix remains partial"
        self._errors.seek(0)
        return (
            self._errors.read(4096).decode(errors="replace").strip() or "native simulation ended before expected output"
        )

    def _fill(self, count):
        """Buffer at least ``count`` unread bytes (reading what arrives, in
        bounded chunks, never preallocating a claimed length). False at EOF."""
        while len(self._buffer) - self._at < count:
            try:
                part = self._process.stdout.read1(1 << 20)
            except (OSError, ValueError) as error:
                if self.cancelled:
                    raise SimulationError("simulation cancelled; prefix remains partial") from error
                raise
            if not part:
                return False
            self._buffer = self._buffer[self._at :] + part
            self._at = 0
        return True

    def _next(self):
        if not self._fill(20):
            if len(self._buffer) > self._at:
                raise ProtocolError("truncated simulation stream")
            raise SimulationError(self._diagnostic())
        magic, version, kind, length, _ = _HEADER.unpack_from(self._buffer, self._at)
        if magic != b"CAMLSTRM" or version != 1 or kind not in (1, 2, 3, 4, 5, 6, 7, 8, 9, 10):
            raise ProtocolError("unsupported stream header")
        if length > FRAME_LIMIT - 20:
            raise ProtocolError("frame exceeds explicit byte budget")
        if not self._fill(20 + length):
            raise ProtocolError("truncated simulation stream")
        raw = self._buffer[self._at : self._at + 20 + length]
        self._at += 20 + length
        return Frame.parse(raw)

    def _next_motions(self, expect=None, limit=4096, checks=False):
        """Updates (a generator; see StockState.accept_many) for the
        committed-motion frames already buffered (at least one, reading if
        needed); None when the next frame is another kind."""
        if not self._fill(20):
            raise SimulationError(self._diagnostic())
        kind = _HEADER.unpack_from(self._buffer, self._at)[2]
        if kind not in (1, 10) and not (kind == 8 and checks and self._last_motion_frame is not None):
            return None
        length = _HEADER.unpack_from(self._buffer, self._at)[3]
        if length > FRAME_LIMIT - 20 or not self._fill(20 + length):
            raise ProtocolError("truncated simulation stream")
        verification = self._verification._native if checks else None
        self._at, items = self.state.accept_many(
            self._buffer,
            self._at,
            expect=expect,
            limit=limit,
            verification=verification,
            last_motion=self._last_motion_frame if checks else None,
        )
        return self._handed_out(items)

    def _handed_out(self, items):
        # Advancing this iterator commits stock. Never eagerly materialize it:
        # each yielded motion must still describe the current stock/cursor.
        for item in items:
            if isinstance(item, Update):
                self._last_motion = item.motion
                self._last_motion_frame = item.frame.raw
                yield item
            else:
                raw, done, progress, check, kind_status = item
                frame = Frame(raw, 8)
                object.__setattr__(frame, "_validated", True)
                update = VerificationUpdate(done, bytes(progress), bytes(check), frame, kind_status)
                self.verification_update = update
                yield update

    def buffered_motion(self):
        """Whether a committed motion is already decoded in memory, after
        at most its verification records: the stream yields it without
        reading the process's output."""
        return getattr(self.state, "_buffered_motions", 0) > 0

    def _motion_checks(self):
        config = self.state._manifest.get("verification", {})
        return self._verification is not None and (
            config["scene"] is not None or config.get("stock_contact") is not None
        )

    def _ready(self):
        if self._pending or self.state.finished or self._closed:
            raise SimulationError("drain outstanding updates before issuing another live command")

    def motions(self, motions):
        self._ready()
        batch = list(itertools.islice(iter(motions), 4097))
        if len(batch) > 4096:
            raise ValueError("batch exceeds 4096 motions")
        self._send({"command": "motions", "motions": [motion_dict(m) for m in batch]})
        config = self.state._manifest.get("verification", {})
        checks = self._verification is not None and (
            config["scene"] is not None or config.get("stock_contact") is not None
        )
        self._pending = len(batch) * (1 + int(checks))
        while self._pending:
            limit = self._pending // 2 if checks else self._pending
            updates = self._next_motions(limit=max(limit, 1), checks=checks)
            if updates is None:
                if checks and self._pending % 2:
                    # The record of a motion handed out by the previous call.
                    verification = self._verification_update(self._next())
                    self._pending -= 1
                    yield verification
                    continue
                raise ProtocolError('unexpected frame inside requested motion batch')
            for update in updates:
                self._pending -= 1
                yield update

    def events(self, events):
        """Consume <=4096 authored typed events in an explicit version 2 run."""
        self._ready()
        if self.state._control_context is None:
            raise ValueError('typed events require stream_version=2')
        batch = []
        source = iter(events)
        for _ in range(4097):
            self._ready()
            try:
                event = next(source)
            except StopIteration:
                break
            batch.append(event_dict(event))
        if len(batch) > 4096:
            raise ValueError('batch exceeds 4096 events')
        self._ready()
        self._send(dict(command='events', events=batch))
        config = self.state._manifest.get('verification', {})
        checks = self._verification is not None and (
            config['scene'] is not None or config.get('stock_contact') is not None
        )
        self._pending = len(batch) + sum(e.get('kind') == 'motion' for e in batch) * int(checks)
        for event in batch:
            frame = self._next()
            if frame.kind != (1 if event.get('kind') == 'motion' else 9):
                raise ProtocolError('unexpected frame inside requested event batch')
            update = self.state.accept(frame)
            self._pending -= 1
            if isinstance(update, Update):
                self._last_motion, self._last_motion_frame = update.motion, update.frame.raw
            yield update
            if checks and isinstance(update, Update):
                verification = self._verification_update(self._next())
                self._pending -= 1
                yield verification

    def run_events(self, events, *, batch_size=128):
        """Own/close a typed event iterator, poll before pulls and finish explicitly."""
        if not 1 <= batch_size <= 4096:
            raise ValueError('invalid bounded batch size')
        self._ready()
        if self.state._control_context is None:
            raise ValueError('typed events require stream_version=2')
        source = iter(events)
        try:
            while True:
                batch = []
                for _ in range(batch_size):
                    self._ready()
                    try:
                        batch.append(next(source))
                    except StopIteration:
                        break
                if not batch:
                    break
                yield from self.events(batch)
            yield from self.finish_updates()
        finally:
            close = getattr(source, 'close', None)
            if close is not None:
                close()

    def _verification_update(self, frame):
        if self._verification is None or frame.kind != 8:
            raise ProtocolError("unexpected verification metadata")
        update = self._verification.read(
            frame, self.state.completed, self._last_motion, motion_frame=self._last_motion_frame
        )
        self.verification_update = update
        return update

    def run(self, motions, *, batch_size=128):
        if not 1 <= batch_size <= 4096:
            raise ValueError("invalid bounded batch size")
        batch = []
        for item in motions:
            if isinstance(item, MaterialStage):
                if batch:
                    yield from self.motions(batch)
                    batch = []
                yield from self.check_material(item.after_operation)
            else:
                batch.append(item)
                if len(batch) == batch_size:
                    yield from self.motions(batch)
                    batch = []
        if batch:
            yield from self.motions(batch)
        yield from self.finish_updates()

    def check_material(self, after_operation):
        """Retain started/result frames at the current committed stock cursor."""
        self._ready()
        if self._verification is None:
            raise SimulationError("material stage requires configured verification")
        self._send(dict(command="material", after_operation=after_operation))
        self._pending = 2
        while self._pending:
            frame = self._next()
            if frame.kind != 8:
                raise ProtocolError("material stage expected verification frame")
            update = self._verification_update(frame)
            self._pending -= 1
            yield update

    def run_operations(self):
        """Yield native operation controls and committed stock/motion updates.

        Definitions are immutable manifest inputs. The native process generates
        the complete ordered job and applies pipe backpressure between motions.
        """
        self._ready()
        manifest = self.state.manifest
        definitions = manifest.get("operations") or manifest.get("turning", {}).get("operations", ())
        if not definitions or self.state.completed:
            raise SimulationError("native operations require a new manifest-bound operation run")
        command = {"command": "operations"}
        if self._regeneration is not None:
            command["cache"] = (
                self._regeneration.to_dict() if hasattr(self._regeneration, "to_dict") else self._regeneration
            )
        self._send(command)
        self._pending, active, completed_operations, active_tool = -1, None, 0, None
        # Per-operation counters, reset when each operation starts.
        active_cuts = active_taps = 0
        checks = self._motion_checks()
        while True:
            if active is not None:
                # The buffered run of motion frames (and their verification
                # records), accepted natively in one call.
                updates = self._next_motions(expect=(active, definitions[completed_operations]["tool"]), checks=checks)
                for update in updates or ():
                    if isinstance(update, Update):
                        # Moves that cut material: drilling plunges, tapping and
                        # thread passes count.
                        active_cuts += update.motion["kind"] in ("cut", "plunge", "lead", "tap", "thread")
                        active_taps += update.motion["kind"] == "tap"
                    yield update
                if updates is not None:
                    continue
            frame = self._next()
            if frame.kind == 7:
                decoder = _Decoder(frame.payload)
                self.state._identity(decoder)
                try:
                    data = json.loads(decoder.take(len(decoder.data) - decoder.offset).tobytes())
                except (ValueError, TypeError) as error:
                    raise ProtocolError("invalid operation control JSON") from error
                if (
                    set(data) != {"operation", "completed", "stage", "events", "report"}
                    or data["completed"] != self.state.completed
                ):
                    raise ProtocolError("operation control schema/cursor mismatch")
                if data["stage"] == "begin":
                    if (
                        active is not None
                        or completed_operations >= len(definitions)
                        or data["operation"] != definitions[completed_operations]["id"]
                        or not isinstance(data["report"], dict)
                    ):
                        raise ProtocolError("operation begin order/report mismatch")
                    active = data["operation"]
                    active_cuts, active_taps, removed_before = 0, 0, self.state.volume()["removed_upper_mm3"]
                    tool = definitions[completed_operations]["tool"]
                    if tool != active_tool:
                        # The declared tool-change pose (recomputed here); the
                        # operation's tool change must state the same.
                        tip = self.state.tool_change_tip(tool)
                        if tip is not None:
                            stated = [
                                tuple(e["position_after_mm"]) for e in data["events"] if e["kind"] == "tool_change"
                            ]
                            if stated != [tip]:
                                raise ProtocolError("operation tool change differs from the declared tool-change pose")
                            self.state.position = tip
                        active_tool = tool
                elif data["stage"] == "end":
                    if (
                        active is None
                        or data["operation"] != active
                        or not isinstance(data["report"], dict)
                        or data["report"].get("geometric_residual_status") not in ("estimated", "unknown")
                    ):
                        raise ProtocolError("operation end order mismatch")
                    if active_cuts == 0:
                        warnings.warn(
                            f"operation {active!r} produced no cutting motion (region smaller "
                            "than the tool, outside the stock, or already cleared)",
                            CamWarning,
                            stacklevel=2,
                        )
                    elif active_taps < active_cuts and self.state.volume()["removed_upper_mm3"] <= removed_before:
                        # A tap is modeled by its drill diameter: it removes
                        # no stock in the model by design.
                        warnings.warn(
                            f"operation {active!r} removed no stock: every cut was in air "
                            "(region outside the stock or already cleared)",
                            CamWarning,
                            stacklevel=2,
                        )
                    active, completed_operations = None, completed_operations + 1
                elif data["stage"] == "events":
                    # Mid-operation events (a dwell at the bottom of a hole),
                    # due after the motion just completed.
                    if (
                        active is None
                        or data["operation"] != active
                        or data["report"] is not None
                        or not data["events"]
                    ):
                        raise ProtocolError("operation events order mismatch")
                else:
                    raise ProtocolError("unknown operation control stage")
                yield OperationUpdate(
                    data["operation"], data["completed"], data["stage"], tuple(data["events"]), data["report"], frame
                )
            elif frame.kind in (1, 10):
                raise ProtocolError("motion arrived outside an operation")
            elif frame.kind == 8:
                yield self._verification_update(frame)
            elif frame.kind == 4:
                if active is not None or completed_operations != len(definitions):
                    raise ProtocolError("operation job ended before all operations completed")
                self.state.accept(frame)
                if self._verification is not None:
                    self._verification.finish(self.state.completed)
                if self._process.wait(timeout=5) != 0:
                    raise SimulationError(self._diagnostic())
                self._pending, self.end_frame = 0, frame
                return
            else:
                raise ProtocolError("unexpected native operation output")

    def finish(self):
        """Finish synchronously; use finish_updates() to consume material reports."""
        for _ in self.finish_updates():
            pass
        return self.end_frame

    def finish_updates(self):
        self._ready()
        self._send({"command": "finish"})
        self._pending = -1
        frame = self._next()
        while frame.kind == 8:
            yield self._verification_update(frame)
            frame = self._next()
        self.state.accept(frame)
        if not self.state.finished:
            raise ProtocolError("native simulation did not send explicit finish")
        if self._process.wait(timeout=5) != 0:
            raise SimulationError(self._diagnostic())
        self.end_frame = frame
        if self._verification is not None:
            self._verification.finish(self.state.completed)
        self._pending = 0

    def cancel(self):
        self.cancelled = True
        self.close()

    def close(self):
        if self._closed:
            return
        if self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=2)
        for f in (self._process.stdin, self._process.stdout, self._errors):
            try:
                f.close()
            except OSError:
                pass
        self._closed = True

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


class PrefixSession:
    """One bounded read-only native cache, reusable across fractional frames."""

    def __init__(self, canonical_manifest, *, executable=None, directory=None):
        self.canonical_manifest = bytes(canonical_manifest)
        self._digest = hashlib.sha256(self.canonical_manifest).digest()
        self._directory, self._lock, self._closed = directory, threading.Lock(), False
        self._loaded_token, self._initial_cache = None, True
        self.checkpoint_loads, self.queries, self.replayed_chunks = 0, 0, 0
        self._cached_complete = 0
        definition = json.loads(self.canonical_manifest)
        self._cached_position = tuple(definition["initial_position_mm"])
        self._cached_control_wire = bytes(40) if definition['version'] == 2 else None
        executable = _executable("cam-sim-stream", executable)
        self._errors = tempfile.TemporaryFile()
        try:
            self._process = subprocess.Popen(
                [str(executable), "--query"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self._errors
            )
        except BaseException:
            self._errors.close()
            raise
        try:
            self._process.stdin.write(self.canonical_manifest + b"\n")
            self._process.stdin.flush()
            manifest = self._next()
            if manifest.kind != 3 or hashlib.sha256(manifest.payload).digest() != self._digest:
                raise ProtocolError("query worker manifest mismatch")
        except BaseException:
            self.close()
            raise

    def _diagnostic(self):
        if self._closed:
            return "prefix query worker closed"
        self._errors.seek(0)
        return self._errors.read(4096).decode(errors="replace").strip() or "prefix query worker ended"

    def _next(self):
        if self._closed:
            raise SimulationError(self._diagnostic())
        frame = Frame.read(self._process.stdout)
        if frame is None:
            raise SimulationError(self._diagnostic())
        return frame

    def _send(self, command):
        if self._closed:
            raise SimulationError(self._diagnostic())
        raw = json.dumps(command, allow_nan=False, separators=(",", ":")).encode() + b"\n"
        try:
            self._process.stdin.write(raw)
            self._process.stdin.flush()
        except (OSError, ValueError) as error:
            raise SimulationError(self._diagnostic()) from error

    def _matching_state(self, state):
        if state.fraction or state.finished or state._digest != self._digest:
            raise ProtocolError('prefix query needs a matching committed move-start state')

    def _invalidate_cache(self):
        self._loaded_token, self._initial_cache = None, False
        self._cached_complete, self._cached_position, self._cached_control_wire = -1, None, None

    def _remember(self, state):
        self._loaded_token, self._initial_cache = state._token, False
        self._cached_complete, self._cached_position, self._cached_control_wire = (
            state.completed,
            state.position,
            state._control_wire,
        )

    def replay(self, state, path, offset, length):
        """Replay a bounded committed mixed-frame file span into the query cache.

        Exact target move/control cursors and pose must be acknowledged before
        synchronization is published. The supplied StockState is never mutated.
        """
        with self._lock:
            self._replay(state, path, offset, length)

    def _replay(self, state, path, offset, length):
        self._matching_state(state)
        if type(offset) is not int or offset < 0 or type(length) is not int or not 0 < length <= FRAME_LIMIT:
            raise ValueError('query cache span requires nonnegative offset and 1..8 MiB bytes')
        previous = max(0, self._cached_complete)
        controls = 0 if self._cached_control_wire is None else struct.unpack_from('<Q', self._cached_control_wire)[0]
        self._invalidate_cache()
        self._send(dict(command='cache', path=str(path), offset=offset, length=length))
        complete, position = self._ack(state)
        if complete != state.completed or position != state.position:
            raise ProtocolError('query history cursor mismatch')
        self.replayed_chunks += max(0, complete - previous)
        if state.control_cursor is not None:
            self.replayed_chunks += max(0, state.control_cursor[0] - controls)
        self._remember(state)

    def preview(self, state, motion, fraction, *, history=None, compact=False):
        with self._lock:
            self._matching_state(state)
            if history is not None and hashlib.sha256(history.wait_ready().payload).digest() != self._digest:
                raise ProtocolError('query history manifest mismatch')
            if state._token is not self._loaded_token:
                synced = False
                if (
                    history is not None
                    and self._cached_control_wire is None
                    and state._control_wire is None
                    and 0 <= self._cached_complete <= state.completed
                ):
                    # Small forward steps reuse history. A long jump restores
                    # the already reconstructed state instead of replaying an
                    # arbitrarily large history into this disposable cache.
                    if self._cached_complete < state.completed:
                        path, offset, length, complete = history.cache_span(self._cached_complete, state.completed)
                        if complete == state.completed:
                            self._replay(state, path, offset, length)
                    synced = self._cached_complete == state.completed and self._cached_position == state.position
                    self._initial_cache = False
                initial_match = (
                    self._initial_cache
                    and state._initial_full_stock
                    and self._cached_complete == state.completed
                    and self._cached_position == state.position
                    and self._cached_control_wire == state._control_wire
                )
                if not synced and not initial_match:
                    with tempfile.TemporaryDirectory(prefix="cam-prefix-", dir=self._directory) as temp:
                        checkpoint = Path(temp) / "base.bin"
                        state.write_checkpoint(checkpoint, cancel=lambda: self._closed)
                        self._invalidate_cache()
                        self._send({"command": "restore", "path": str(checkpoint)})
                        complete, position = self._ack(state)
                        if complete != state.completed or position != state.position:
                            raise ProtocolError("query cache reset cursor mismatch")
                    self.checkpoint_loads += 1
                self._remember(state)
            # Preview commands are noncommitting. A rejected preview cannot
            # invalidate a previously acknowledged base; failed SYNC paths above
            # already invalidate before sending their state-changing commands.
            self._send({"command": "partial", "motion": motion_dict(motion), "fraction": fraction})
            shown, update = state.partial(self._next(), compact=compact)
            expected = motion_dict(motion)
            curve = dict(expected["curve"])
            for name in ("start", "end", "center"):
                if name in curve:
                    curve[name] = tuple(curve[name])
            expected = {**expected, "curve": curve, "feed_mm_min": expected.get("feed_mm_min")}
            if update.fraction != fraction or update.motion != expected:
                raise ProtocolError("prefix response does not match requested motion/fraction")
            self.queries += 1
            return shown, update

    def _ack(self, state):
        ack = self._next()
        if ack.kind != 6:
            raise ProtocolError("query cache reset lacks cursor acknowledgement")
        d = _Decoder(ack.payload)
        state._identity(d)
        (complete,) = d.unpack("<Q")
        position = d.unpack("<3d")
        state._control_prefix(d)
        d.finish()
        return complete, position

    def close(self):
        if self._closed:
            return
        self._closed = True
        if self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=2)
        self._process.stdin.close()
        self._process.stdout.close()
        self._errors.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def preview_prefix(state, motion, fraction, *, executable=None, directory=None):
    """One-shot convenience query; Player retains a PrefixSession for playback."""
    with PrefixSession(state.canonical_manifest, executable=executable, directory=directory) as query:
        return query.preview(state, motion, fraction)
