"""Bounded retained simulation with an independent playback cursor/clock.

Only chunks/checkpoint indices live on disk; no growing in-memory motion history.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
import hashlib
import itertools
import json
from pathlib import Path
import shutil
import struct
import tempfile
import threading
import time

from .simulation import (
    FRAME_LIMIT,
    Frame,
    ProtocolError,
    Simulation,
    SimulationError,
    PrefixSession,
    stock_state,
    OperationUpdate,
    VerificationUpdate,
    VerificationStream,
    _Decoder,
    checked_length,
)


class NotReady(SimulationError):
    pass


@dataclass(frozen=True)
class Cursor:
    completed: int = 0
    fraction: float = 0.0

    def normalized(self):
        if (
            type(self.completed) is not int
            or self.completed < 0
            or not math.isfinite(self.fraction)
            or not 0 <= self.fraction <= 1
        ):
            raise ValueError("cursor requires nonnegative complete count and fraction in [0,1]")
        return Cursor(self.completed + int(self.fraction == 1), 0.0 if self.fraction == 1 else self.fraction)


class _JournalFiles:
    """Journal files opened once for a page of reads: each opened on first
    use, its size taken then (records read were published before)."""

    def __init__(self, directory):
        self._directory = directory
        self._open = {}

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        for handle, _ in self._open.values():
            handle.close()
        self._open.clear()

    def _get(self, name):
        entry = self._open.get(name)
        if entry is None:
            handle = (self._directory / name).open("rb")
            entry = (handle, os.fstat(handle.fileno()).st_size)
            self._open[name] = entry
        return entry

    def size(self, name):
        return self._get(name)[1]

    def read(self, name, offset, length):
        handle = self._get(name)[0]
        handle.seek(offset)
        return handle.read(length)


class RecordedRun:
    # Defaults: sources are pulled 128 motions at a time; a stock checkpoint
    # is kept every 1024 motions (or 100k changed cells), so a seek replays
    # at most 1024 motions (about 30 ms) from its nearest checkpoint.
    def __init__(
        self,
        stock,
        tools,
        motions=None,
        *,
        revision,
        initial_position_mm,
        operations=(),
        directory=None,
        keep=False,
        batch_size=128,
        checkpoint_every=1024,
        checkpoint_changed_cells=100_000,
        retained_bytes_limit=8 * 1024**3,
        # The machine's rapid traverse rate (nominal time of rapids); required.
        rapid_mm_min,
        sweep=None,
        executable=None,
        run_id=None,
        verification=None,
        regeneration=None,
        tool_change=None,
        rapid_retracts=True,
        turning=None,
    ):
        if not 1 <= batch_size <= 4096 or checkpoint_every <= 0 or checkpoint_changed_cells <= 0:
            raise ValueError("invalid bounded batching/checkpoint policy")
        if retained_bytes_limit < 1024 or not math.isfinite(rapid_mm_min) or rapid_mm_min <= 0:
            raise ValueError("invalid history budget or nominal rapid rate")
        if not isinstance(operations, (list, tuple)):
            raise TypeError("operations must be a list or tuple (a finite collection)")
        operations = tuple(operations)
        if operations and motions is not None:
            raise ValueError("provide either native operations or authored motions")
        if directory is not None:
            Path(directory).mkdir(parents=True, exist_ok=True)
        self.directory = Path(tempfile.mkdtemp(prefix="cam-run-", dir=directory))
        self.keep, self.batch_size = keep, batch_size
        self.checkpoint_every, self.checkpoint_changed_cells = checkpoint_every, checkpoint_changed_cells
        self.retained_bytes_limit, self.rapid_mm_min = retained_bytes_limit, rapid_mm_min
        self._condition = threading.Condition()
        self._cancel = threading.Event()
        # A lathe run's operations are its turning setup's; given motions
        # (a posted program read back) are replayed on its stock instead.
        self._operations = bool(operations) or turning is not None and motions is None
        self._source = iter(()) if motions is None else iter(motions)
        self._available, self._checkpoint_count, self._bytes = 0, 0, 0
        self._control_bytes = 0
        self._operation_progress = None
        self._verification_count = 0
        self._verification_reader = None
        self._verification_coverage, self._verification_identity = None, None
        self._finished, self._error = False, None
        self._elapsed, self._first_stock, self._largest_frame = 0.0, None, 0
        self._manifest_frame, self._simulation = None, None
        self._preview_session = None
        self._started = time.perf_counter()
        self._config = dict(
            stock=stock,
            tools=tools,
            revision=revision,
            initial_position_mm=initial_position_mm,
            sweep=sweep,
            executable=executable,
            run_id=run_id,
            operations=operations,
            verification=verification,
            regeneration=regeneration,
            tool_change=tool_change,
            rapid_retracts=rapid_retracts,
            turning=turning,
            _on_spawn=self._spawned,
        )
        self._verification = verification.to_dict() if hasattr(verification, "to_dict") else verification
        self._thread = threading.Thread(target=self._produce, name="cam-simulation", daemon=True)
        self._thread.start()

    def save(self, destination, *, cancel=None, max_bytes=8 * 1024**3):
        """Save a completed run independently of this producer's lifetime."""
        from .persistence import save_run

        return save_run(self, destination, cancel=cancel, max_bytes=max_bytes)

    @classmethod
    def open(cls, directory, *, executable=None, cancel=None, max_bytes=8 * 1024**3):
        """Reopen a completed native or posted snapshot without replanning."""
        from .persistence import open_run

        run = open_run(directory, executable=executable, cancel=cancel, max_bytes=max_bytes)
        if not isinstance(run, cls):
            run.close()
            raise ValueError("saved run kind does not match requested replay type")
        return run

    def _spawned(self, simulation):
        with self._condition:
            self._simulation = simulation
        if self._cancel.is_set():
            simulation.cancel()

    def _quota(self, count):
        if self._bytes + count > self.retained_bytes_limit:
            raise SimulationError("retained simulation history exceeds configured byte budget")

    def _store(self, path, raw):
        self._quota(len(raw))
        temporary = path.with_suffix(path.suffix + ".writing")
        try:
            with temporary.open("xb") as f:
                f.write(raw)
            temporary.replace(path)
        finally:
            temporary.unlink(missing_ok=True)
        self._bytes += len(raw)

    def _entry(self, index, files=None):
        with self._condition:
            if not 0 <= index < self._available:
                raise NotReady("retained motion has not been published")
        if files is not None:
            raw = files.read("motions.idx", index * 32, 32)
        else:
            with (self.directory / "motions.idx").open("rb") as f:
                f.seek(index * 32)
                raw = f.read(32)
        if len(raw) != 32:
            raise ProtocolError("missing retained motion index entry")
        offset, length, reserved, start, end = struct.unpack("<QIIdd", raw)
        if (
            reserved
            or not 20 <= length <= FRAME_LIMIT
            or not math.isfinite(start)
            or not math.isfinite(end)
            or not 0 <= start <= end
        ):
            raise ProtocolError("invalid retained motion index entry")
        return offset, length, start, end

    def read_chunk(self, index, files=None):
        offset, length, _, _ = self._entry(index, files)
        if files is not None:
            if offset + length > files.size("motions.bin"):
                raise ProtocolError("retained frame extent is missing")
            return Frame.parse(files.read("motions.bin", offset, length))
        path = self.directory / "motions.bin"
        if offset + length > path.stat().st_size:
            raise ProtocolError("retained frame extent is missing")
        with path.open("rb") as f:
            f.seek(offset)
            raw = f.read(length)
        return Frame.parse(raw)

    def operation_controls(self):
        """Read published operation metadata; a live writer may add records later."""
        canonical = self.wait_ready().payload
        manifest = json.loads(canonical.tobytes())
        identity = (manifest["run"]["id"], manifest["run"]["revision"], hashlib.sha256(canonical).digest())
        with self._condition:
            published = self._control_bytes
        with (self.directory / "controls.bin").open("rb") as source:
            while source.tell() < published:
                frame = Frame.read(source)
                if frame is None or source.tell() > published:
                    raise ProtocolError("missing published operation metadata")
                decoder = _Decoder(frame.payload)
                if (
                    frame.kind != 7
                    or (decoder.string(128), decoder.string(512), decoder.take(32).tobytes()) != identity
                ):
                    raise ProtocolError("foreign retained operation metadata")
                yield json.loads(decoder.take(len(decoder.data) - decoder.offset).tobytes())

    def cache_span(self, start, target):
        """A bounded contiguous published binary span for read-only query replay."""
        if not 0 <= start < target <= self.status()["available"]:
            raise NotReady("query cache span has not been published")
        offset, length, _, _ = self._entry(start)
        end, complete = offset + length, start + 1
        while complete < target and complete - start < 4096:
            next_offset, next_length, _, _ = self._entry(complete)
            if next_offset != end:
                raise ProtocolError("noncontiguous retained cache span")
            if end + next_length - offset > FRAME_LIMIT:
                break
            end, complete = end + next_length, complete + 1
        return self.directory / "motions.bin", offset, end - offset, complete

    def motion_time(self, index):
        return self._entry(index)[2:]

    def checkpoint_path(self, complete):
        return self.directory / f"{complete:016x}.checkpoint.gz"

    def _checkpoint(self, state, index):
        # Reserve a worst-case compressed snapshot before creating its temp file.
        # Compression uses <=1 MiB blocks, not a second full stock array.
        raw_bytes = state.checkpoint_bytes
        self._quota(raw_bytes + raw_bytes // 100 + 65536)
        path = self.checkpoint_path(state.completed)
        state.write_checkpoint(path, compressed=True, cancel=self._cancel.is_set)
        self._bytes += path.stat().st_size
        self._quota(8)
        index.write(struct.pack("<Q", state.completed))
        index.flush()
        self._bytes += 8
        with self._condition:
            self._checkpoint_count += 1

    def _guarded_source(self):
        while not self._cancel.is_set():
            # The source may block: publish what is already committed first.
            publish = self.__dict__.get("_publish")
            if publish is not None:
                publish()
            try:
                motion = next(self._source)
            except StopIteration:
                return
            if not self._cancel.is_set():
                yield motion

    def cursor_metadata(self, cursor, motion):
        """Optional retained attribution, validated before a seek is published."""
        return None

    def _produce(self):
        try:
            with Simulation(**self._config) as simulation:
                if self._cancel.is_set():
                    return
                self._store(self.directory / "manifest.bin", simulation.manifest_frame.raw)
                with (
                    (self.directory / "checkpoints.idx").open("xb") as checkpoints,
                    (self.directory / "motions.idx").open("xb") as index,
                    (self.directory / "motions.bin").open("xb") as frames,
                    (self.directory / "controls.bin").open("xb") as controls,
                    (self.directory / "verification.bin").open("xb") as checks,
                    (self.directory / "verification.idx").open("xb") as check_index,
                ):
                    self._quota(8)
                    checkpoints.write(struct.pack("<Q", 0))
                    checkpoints.flush()
                    self._bytes += 8
                    with self._condition:
                        if simulation._verification is not None:
                            self._verification_reader = VerificationStream(simulation.manifest_frame)
                            config = simulation.state._manifest["verification"]
                            scene = config["scene"]
                            self._verification_coverage = dict(
                                configured_prism_bodies=0 if scene is None else len(scene["scene"]["bodies"]),
                                enclosure_only_bodies=0
                                if scene is None
                                else sum(b.get("representation") == "outer_envelope" for b in scene["scene"]["bodies"]),
                                motion_checks=scene is not None or config.get("stock_contact") is not None,
                                material_stages=len(config["material"]),
                                machine_tip_and_feed_configured=scene is not None
                                and scene["scene"]["machine"] is not None,
                                evolving_stock_contact=config.get("stock_contact") is not None,
                                controller_execution=False,
                            )
                            run = simulation.state._manifest["run"]
                            self._verification_identity = (run["id"], run["revision"], simulation.state._digest)
                        self._manifest_frame, self._checkpoint_count = simulation.manifest_frame, 1
                        self._condition.notify_all()
                    last_checkpoint, changed_since = 0, 0
                    previous_issue = previous_material = 0
                    # Motion frames are written as they arrive and published
                    # (flushed, then made available to readers) in groups,
                    # before any other record and at the end.
                    pending = []
                    published_at = [time.perf_counter()]

                    def publish():
                        published_at[0] = time.perf_counter()
                        if not pending:
                            return
                        frames.flush()
                        index.flush()
                        with self._condition:
                            for completed, end_time, size, changed in pending:
                                self._available, self._elapsed = completed, end_time
                                self._largest_frame = max(self._largest_frame, size)
                                if changed and self._first_stock is None:
                                    self._first_stock = time.perf_counter() - self._started
                            self._condition.notify_all()
                        pending.clear()

                    self._publish = publish
                    updates = (
                        simulation.run_operations()
                        if self._operations
                        else simulation.run(self._guarded_source(), batch_size=self.batch_size)
                    )
                    for update in updates:
                        if self._cancel.is_set():
                            return
                        if not hasattr(update, "motion"):
                            publish()
                        if isinstance(update, OperationUpdate):
                            self._quota(len(update.frame.raw))
                            controls.write(update.frame.raw)
                            controls.flush()
                            self._bytes += len(update.frame.raw)
                            with self._condition:
                                self._control_bytes = controls.tell()
                                self._operation_progress = dict(
                                    operation=update.operation, stage=update.stage, completed=update.completed
                                )
                            continue
                        if isinstance(update, VerificationUpdate):
                            data = update.check
                            material = data["kind"] in ("material", "material_started")
                            report = data.get("report") or {}
                            issue = (
                                data["kind"] == "motion"
                                and update.status != "clear"
                                or data["kind"] == "material"
                                and any(report[k]["assessment"] != "within_tolerance" for k in ("residual", "gouge"))
                            )
                            self._quota(len(update.frame.raw) + 48)
                            number = self._verification_count
                            check_index.write(
                                struct.pack(
                                    "<QIIQQQQ",
                                    checks.tell(),
                                    len(update.frame.raw),
                                    int(issue) + 2 * int(material),
                                    update.completed,
                                    number + 1,
                                    previous_issue,
                                    previous_material,
                                )
                            )
                            checks.write(update.frame.raw)
                            checks.flush()
                            check_index.flush()
                            self._bytes += len(update.frame.raw) + 48
                            if issue:
                                previous_issue = number + 1
                            if material:
                                previous_material = number + 1
                            with self._condition:
                                self._verification_count = number + 1
                                self._condition.notify_all()
                            continue
                        rate = update.motion["feed_mm_min"]
                        if rate is None:
                            rate = self.rapid_mm_min
                        # The committed chunk was decoded with its curve checks.
                        duration = checked_length(update.motion["curve"]) / rate * 60
                        if not math.isfinite(duration):
                            raise SimulationError("nominal playback duration exceeds numeric range")
                        start_time = pending[-1][1] if pending else self._elapsed
                        end_time = start_time + duration
                        self._quota(len(update.frame.raw) + 32)
                        offset = frames.tell()
                        frames.write(update.frame.raw)
                        index.write(struct.pack("<QIIdd", offset, len(update.frame.raw), 0, start_time, end_time))
                        self._bytes += len(update.frame.raw) + 32
                        pending.append((update.completed, end_time, len(update.frame.raw), update.changed_count))
                        changed_since += update.changed_count
                        if len(pending) >= 64 or time.perf_counter() - published_at[0] > 0.025:
                            publish()
                        if (
                            update.completed - last_checkpoint >= self.checkpoint_every
                            or changed_since >= self.checkpoint_changed_cells
                        ):
                            publish()
                            self._checkpoint(simulation.state, checkpoints)
                            # The checkpoint holds the stock as decoded, which
                            # may be ahead of this update within its batch.
                            last_checkpoint, changed_since = simulation.state.completed, 0
                    publish()
                    if self._cancel.is_set():
                        return
                    if simulation.state.completed != last_checkpoint:
                        self._checkpoint(simulation.state, checkpoints)
                    self._store(self.directory / "end.bin", simulation.end_frame.raw)
                    with self._condition:
                        self._finished = True
        except BaseException as error:
            # Motions committed before the failure stay available.
            publish = self.__dict__.pop("_publish", None)
            if publish is not None and not self._cancel.is_set():
                try:
                    publish()
                except BaseException:
                    pass
            with self._condition:
                if not self._cancel.is_set():
                    self._error = f"{type(error).__name__}: {error}"
        finally:
            self.__dict__.pop("_publish", None)
            try:
                close = getattr(self._source, "close", None)
                if close is not None:
                    close()
            except BaseException as error:
                with self._condition:
                    if not self._cancel.is_set() and self._error is None:
                        self._error = f"source cleanup: {type(error).__name__}: {error}"
            with self._condition:
                self._simulation, self._source = None, None
                self._condition.notify_all()

    def status(self):
        with self._condition:
            return dict(
                available=self._available,
                seconds=self._elapsed,
                finished=self._finished,
                cancelled=self._cancel.is_set(),
                error=self._error,
                retained_bytes=self._bytes,
                checkpoints=self._checkpoint_count,
                first_stock_seconds=self._first_stock,
                largest_frame_bytes=self._largest_frame,
                producing=self._thread.is_alive(),
                operation=None if self._operation_progress is None else dict(self._operation_progress),
            )

    def _verification_entry(self, index, count, files=None):
        if not 0 <= index < count:
            raise NotReady("verification record has not been published")
        if files is not None:
            raw = files.read("verification.idx", index * 48, 48)
        else:
            with (self.directory / "verification.idx").open("rb") as f:
                f.seek(index * 48)
                raw = f.read(48)
        if len(raw) != 48:
            raise ProtocolError("missing verification index entry")
        entry = struct.unpack("<QIIQQQQ", raw)
        offset, length, flags, completed, number, issue, material = entry
        if not 20 <= length <= FRAME_LIMIT or flags > 3 or number != index + 1 or issue > index or material > index:
            raise ProtocolError("invalid retained verification index")
        return entry

    def _verification_record(self, index, count, files=None):
        entry = self._verification_entry(index, count, files)
        offset, length, _, completed, number, _, _ = entry
        if files is not None:
            if offset + length > files.size("verification.bin"):
                raise ProtocolError("retained verification extent is missing")
            frame = Frame.parse(files.read("verification.bin", offset, length))
        else:
            path = self.directory / "verification.bin"
            if offset + length > path.stat().st_size:
                raise ProtocolError("retained verification extent is missing")
            with path.open("rb") as f:
                f.seek(offset)
                frame = Frame.parse(f.read(length))
        motion = None
        if completed:
            source = self.read_chunk(completed - 1, files)
            decoder = _Decoder(source.payload)
            if (
                source.kind != 1
                or (decoder.string(128), decoder.string(512), decoder.take(32).tobytes()) != self._verification_identity
                or decoder.unpack("<3Q") != (completed - 1, completed - 1, completed)
            ):
                raise ProtocolError("verification source motion attribution mismatch")
            motion = decoder.motion()
        update = self._verification_reader.read(frame, completed, motion, retained=True)
        if update.progress["records"] != number:
            raise ProtocolError("verification index/record mismatch")
        data, report = update.check, (update.check.get("report") or {})
        material = data["kind"] in ("material", "material_started")
        issue = (
            data["kind"] == "motion"
            and update.status != "clear"
            or data["kind"] == "material"
            and any(report[k]["assessment"] != "within_tolerance" for k in ("residual", "gouge"))
        )
        if entry[2] != int(issue) + 2 * int(material):
            raise ProtocolError("verification index/result flags mismatch")
        return update, entry

    def _verification_config(self):
        return getattr(self, "_verification", None)

    def verification_records(self, start=0, *, limit=64):
        """Bounded report page. Full findings stay on disk, separate from motions."""
        if type(start) is not int or start < 0 or type(limit) is not int or not 1 <= limit <= 256:
            raise ValueError("verification page requires start>=0 and limit in[1,256]")
        self.wait_ready()
        with self._condition:
            count = self._verification_count
        # One set of open files per page (the journal files only grow; every
        # record of the page was published before it started).
        with _JournalFiles(self.directory) as files:
            for index in range(start, min(count, start + limit)):
                yield self._verification_record(index, count, files)[0]

    def verification_status(self, completed=None, *, sample_limit=8):
        """Coverage at a committed cursor; generation's checked prefix is separate."""
        if type(sample_limit) is not int or not 0 <= sample_limit <= 8:
            raise ValueError("verification sample limit must lie in[0,8]")
        self.wait_ready()
        with self._condition:
            count, available = self._verification_count, self._available
        if completed is None:
            completed = available
        if type(completed) is not int or not 0 <= completed <= available:
            raise ValueError("verification cursor is outside produced stock")
        if self._verification_coverage is None:
            return dict(status="not_configured", records=0, samples=[])
        low, high = 0, count
        while low < high:
            middle = (low + high) // 2
            if self._verification_entry(middle, count)[3] <= completed:
                low = middle + 1
            else:
                high = middle
        result = dict(
            status="partial",
            records=low,
            available_records=count,
            at_completed=completed,
            progress=dict(
                records=0, from_completed=0, through_completed=0, clear=0, violation=0, uncertain=0, material_reports=0
            ),
            coverage=dict(self._verification_coverage),
            material=None,
            samples=[],
            samples_truncated=False,
        )
        if not low:
            return result
        update, entry = self._verification_record(low - 1, count)
        result["progress"] = update.progress
        material = low if entry[2] & 2 else entry[6]
        if material:
            value = update if material == low else self._verification_record(material - 1, count)[0]
            data = value.check
            report = data.get("report")
            result["material"] = dict(
                completed=value.completed,
                after_operation=data["after_operation"],
                status="checking" if report is None else "complete" if report["complete"] else "incomplete",
                target_id=data["target_id"] if report is None else report["target_id"],
                residual=None if report is None else report["residual"],
                gouge=None if report is None else report["gouge"],
            )
            if data.get("sweep_proof") is not None:
                # The per-move gouge proof against the target (decided cells,
                # proven and unproven moves), recorded beside the report.
                result["material"]["sweep_proof"] = dict(data["sweep_proof"])
            if report is not None and report.get("finishing") is not None:
                # Bounded status projection; localized cell findings stay in
                # the existing paged verification journal.
                result["material"]["finishing"] = {k: v for k, v in report["finishing"].items() if k != "findings"}

        issue = low if entry[2] & 1 else entry[5]
        while issue and len(result["samples"]) < sample_limit:
            value, issue_entry = self._verification_record(issue - 1, count)
            data, report = value.check, value.check["report"]
            remaining = sample_limit - len(result["samples"])
            findings = (
                list(itertools.islice(value.findings(), remaining + 1))
                if data["kind"] == "motion"
                else report["findings"][: remaining + 1]
            )
            for finding in findings[:remaining]:
                if data["kind"] == "motion":
                    source = data.get("report") or data.get("stock_contact")
                    sample = dict(
                        kind="motion",
                        completed=value.completed,
                        motion_id=source["motion_id"],
                        operation=source["operation"],
                        tool=source["tool"],
                        status=finding["status"],
                        finding_kind=finding["kind"],
                        object_id=finding["object_id"],
                        part=finding["part"],
                        fraction_interval=finding["fraction_interval"],
                        detail=finding["detail"][:256],
                        source=finding["source"],
                    )
                    if "cell_range" in finding:
                        sample.update(cell_range=finding["cell_range"], contact_point_mm=finding["contact_point_mm"])
                else:
                    sample = dict(
                        kind="material",
                        completed=value.completed,
                        target_id=report["target_id"],
                        cell_index=finding["cell_index"],
                        xy_mm=finding["xy_mm"],
                        residual=finding["residual"],
                        gouge=finding["gouge"],
                    )
                if value.completed:
                    a, b = self.motion_time(value.completed - 1)
                    fraction = (finding.get("witness") or {}).get(
                        "fraction", sum(finding.get("fraction_interval", [1, 1])) / 2
                    )
                    sample["seconds"] = a + (b - a) * fraction
                else:
                    sample["seconds"] = 0.0
                result["samples"].append(sample)
            if not findings:
                result["samples"].append(
                    dict(
                        kind=data["kind"],
                        completed=value.completed,
                        detail="Work limit or finding cap; inspect the full report",
                    )
                )
            reports = [r for r in (data.get("report"), data.get("stock_contact")) if r]
            result["samples_truncated"] |= (
                any(r["stats"]["findings_truncated"] for r in reports) or len(findings) > remaining
            )
            issue = issue_entry[5]
        result["samples_truncated"] |= bool(issue)
        return result

    def preview_session(self):
        manifest = self.wait_ready()
        with self._condition:
            if self._preview_session is None:
                self._preview_session = PrefixSession(manifest.payload, executable=self._config["executable"])
            return self._preview_session

    def wait_ready(self, timeout=5):
        deadline = time.monotonic() + timeout
        with self._condition:
            while self._manifest_frame is None:
                self._raise_if_stopped()
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("simulation setup not ready")
                self._condition.wait(remaining)
            return self._manifest_frame

    def _raise_if_stopped(self):
        if self._error:
            raise SimulationError(self._error)
        if self._cancel.is_set():
            raise SimulationError("simulation cancelled; retained prefix is partial")

    def wait_available(self, complete, timeout=5):
        deadline = time.monotonic() + timeout
        with self._condition:
            while self._available < complete:
                self._raise_if_stopped()
                if self._finished:
                    raise NotReady("requested cursor lies beyond completed run")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("motion prefix not yet available")
                self._condition.wait(remaining)

    def wait(self, timeout=30):
        self._thread.join(timeout)
        if self._thread.is_alive():
            raise TimeoutError("simulation is still producing")
        with self._condition:
            self._raise_if_stopped()
            if not self._finished:
                raise SimulationError("simulation ended without explicit finish")
        return self.status()

    def cancel(self):
        with self._condition:
            if self._finished:
                return
            self._cancel.set()
            simulation = self._simulation
            query = self._preview_session
            self._condition.notify_all()
        if simulation is not None:
            simulation.cancel()
        if query is not None:
            query.close()

    def wait_stopped(self, timeout=5):
        """Wait for owned production to exit, including a cancelled partial job."""
        if threading.current_thread() is self._thread:
            raise RuntimeError("producer cannot wait for itself")
        self._thread.join(timeout)
        return not self._thread.is_alive()

    def close(self):
        self.cancel()
        with self._condition:
            query, self._preview_session = self._preview_session, None
        if query is not None:
            query.close()
        self._thread.join(2)
        if not self._thread.is_alive() and not self.keep:
            shutil.rmtree(self.directory, ignore_errors=True)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


class Player:
    def __init__(self, run: RecordedRun, *, compact=False):
        self.compact = compact
        self.run = run
        manifest = run.wait_ready()
        # Start the retained worker before allocating a second full stock view.
        self._prefix = run.preview_session()
        self._base = stock_state(manifest, max_cells=0xFFFFFFFF)
        self._shown = self._base
        self.cursor = Cursor()
        self.playing, self.speed, self.waiting_for_data = False, 1.0, False
        self.motion = None
        self.posting = run.cursor_metadata(self.cursor, None)
        self.last_seek = dict(checkpoint=0, replayed=0)
        self.full_refresh, self.dirty_rect, self._partial_dirty = True, None, None

    @property
    def state(self):
        return self._shown

    @property
    def verification(self):
        """Only checks attributed to the displayed committed prefix."""
        return self.run.verification_status(self.cursor.completed)

    def _frame(self, index):
        frame = self.run.read_chunk(index)
        d = _Decoder(frame.payload)
        self._base._identity(d)
        if frame.kind not in (1, 10) or d.unpack("<3Q") != (index, index, index + 1):
            raise ProtocolError("retained motion sequence/range mismatch")
        return frame

    def _checkpoint_before(self, complete):
        count = self.run.status()["checkpoints"]
        with (self.run.directory / "checkpoints.idx").open("rb") as f:
            low, high = 0, count
            while low < high:
                middle = (low + high) // 2
                f.seek(middle * 8)
                (value,) = struct.unpack("<Q", f.read(8))
                if value <= complete:
                    low = middle + 1
                else:
                    high = middle
            f.seek(max(0, low - 1) * 8)
            return struct.unpack("<Q", f.read(8))[0]

    def _motion_from(self, frame, state):
        d = _Decoder(frame.payload)
        state._identity(d)
        if frame.kind not in (1, 10):
            raise ProtocolError("history record is not a committed motion")
        d.unpack("<3Q")
        return d.motion()

    def seek(self, cursor):
        cursor = cursor.normalized()
        status = self.run.status()
        if cursor.completed > status["available"] or cursor.fraction and cursor.completed >= status["available"]:
            raise NotReady("motion/stock cursor has not been produced")
        checkpoint = self._checkpoint_before(cursor.completed)
        start = self._base.completed
        loaded_checkpoint = None
        origin = start
        if cursor.completed == start:
            work = self._base
        elif start <= cursor.completed and start >= checkpoint:
            work = self._base.clone()
        else:
            loaded_checkpoint = origin = checkpoint
            if checkpoint:
                work = self._base._metadata_clone()
                work.restore_checkpoint(self.run.checkpoint_path(checkpoint), compressed=True)
            else:
                work = stock_state(self.run.wait_ready(), max_cells=0xFFFFFFFF)
        replayed = 0
        dirty = self._partial_dirty

        def include(rect):
            nonlocal dirty
            if rect is not None:
                dirty = (
                    rect
                    if dirty is None
                    else (
                        min(dirty[0], rect[0]),
                        min(dirty[1], rect[1]),
                        max(dirty[2], rect[2]),
                        max(dirty[3], rect[3]),
                    )
                )

        while work.completed < cursor.completed:
            update = work.accept(self._frame(work.completed))
            replayed += 1
            include(update.dirty_rect)
        motion = None
        partial_dirty = None
        if cursor.fraction and getattr(work, "turning", False):
            # Within a lathe move: the turning kernel on the move's first
            # fraction, from the stock at its start.
            frame = self._frame(cursor.completed)
            motion = self._motion_from(frame, work)
            shown = work.preview(frame, cursor.fraction)
        elif cursor.fraction:
            motion = self._motion_from(self._frame(cursor.completed), work)
            shown, update = self._prefix.preview(work, motion, cursor.fraction, history=self.run, compact=self.compact)
            partial_dirty = update.dirty_rect
            include(partial_dirty)
        else:
            shown = work
            if cursor.completed:
                motion = self._motion_from(self._frame(cursor.completed - 1), work)
        posting = self.run.cursor_metadata(cursor, motion)
        if not cursor.fraction and status["finished"] and cursor.completed == status["available"] and not work.finished:
            with (self.run.directory / "end.bin").open("rb") as f:
                work.accept(Frame.read(f))
        # Publish only after stock AND optional attribution validate successfully.
        self._base, self._shown, self.cursor, self.motion = work, shown, cursor, motion
        self.posting = posting
        self.last_seek = dict(checkpoint=loaded_checkpoint, from_complete=origin, replayed=replayed)
        self.full_refresh, self.dirty_rect, self._partial_dirty = loaded_checkpoint is not None, dirty, partial_dirty
        return self.state

    def play(self):
        self.playing, self.waiting_for_data = True, False

    def pause(self):
        self.playing, self.waiting_for_data = False, False

    def _time(self, index):
        return self.run.motion_time(index)

    @property
    def seconds(self):
        if self.cursor.fraction:
            a, b = self._time(self.cursor.completed)
            return a + (b - a) * self.cursor.fraction
        return self._time(self.cursor.completed - 1)[1] if self.cursor.completed else 0.0

    def seek_seconds(self, seconds):
        if not math.isfinite(seconds) or seconds < 0:
            raise ValueError("playback time must be finite and nonnegative")
        status = self.run.status()
        if seconds > status["seconds"]:
            raise NotReady("requested playback time has not been produced")
        if seconds == status["seconds"]:
            return self.seek(Cursor(status["available"]))
        low, high = 0, status["available"]
        while low < high:
            middle = (low + high) // 2
            if self._time(middle)[1] <= seconds:
                low = middle + 1
            else:
                high = middle
        a, b = self._time(low)
        return self.seek(Cursor(low, (seconds - a) / (b - a)))

    def set_speed(self, speed):
        if not math.isfinite(speed) or not 0.01 <= speed <= 1000:
            raise ValueError("playback speed must lie in [0.01,1000]")
        self.speed = speed

    def tick(self, wall_seconds):
        if not math.isfinite(wall_seconds) or wall_seconds < 0:
            raise ValueError("wall-time step must be finite and nonnegative")
        if not self.playing:
            return self.state
        status = self.run.status()
        target = self.seconds + wall_seconds * self.speed
        self.waiting_for_data = target > status["seconds"] and not status["finished"]
        target = min(target, status["seconds"])
        if target != self.seconds:
            self.seek_seconds(target)
        if status["finished"] and self.cursor == Cursor(status["available"]):
            self.playing = False
        return self.state
