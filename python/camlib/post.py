"""Programmable, bounded controller posting and independent NC checking.

All machine table/pose values are explicit external premises. This module emits
files/chunks only. It never connects to or controls a machine.
"""

from dataclasses import dataclass
from itertools import islice
from functools import cached_property
import json

from ._buffers import native
from .motion import Arc, Helix, Line, Motion, Program

Cancellation = native.PostCancellation


@dataclass(frozen=True)
class ToolSetting:
    tool: str
    number: int
    length_register: int
    gauge_length_mm: float
    # The tip after this tool's change, in work coordinates. Required by
    # Machine(tool_change_position_mode="table"); the other modes do not read
    # it (the pose follows from the declared machine behaviour).
    position_after_mm: tuple | None = None
    # D register holding this tool's wear for cutter compensation (G41/G42).
    diameter_register: int | None = None

    @classmethod
    def for_tool(cls, tool, *, number, length_register, position_after_mm=None, diameter_register=None):
        return cls(tool.id, number, length_register, tool.gauge_length_mm, position_after_mm, diameter_register)


def _json(value):
    text = json.dumps(value, allow_nan=False, separators=(',', ':'))
    if len(text.encode()) > 512 * 1024:
        raise ValueError('post value exceeds 512 KiB')
    return text


def _event_data(event):
    data = event.to_dict() if hasattr(event, 'to_dict') else event
    # Native simulation publishes the Motion body; operation controls publish
    # typed Event dictionaries. Both retain the same M1 meanings.
    return {'kind': 'motion', 'motion': data} if isinstance(data, dict) and 'curve' in data else data


def _batch(events):
    events = tuple(islice(iter(events), 65))
    if len(events) > 64:
        raise ValueError('post consume accepts at most 64 events')
    return events


@dataclass(frozen=True)
class Chunk:
    raw: bytes

    @property
    def data(self):
        return json.loads(self.raw)

    @cached_property
    def _parsed(self):
        return json.loads(self.raw)

    @cached_property
    def text(self):
        return ''.join(line['text'] + '\n' for line in self._parsed['lines'])

    @cached_property
    def events(self):
        """Actual checked source-event effects, in decoded order (generic ABI5/6).

        Initial/final NC chunks have no source checks and return (). Older wire
        versions omitted controls and cannot supply a complete event view.
        Grouping, source hashes and NC ranges remain in checked/mappings.
        """
        if self._parsed['identity']['abi'] not in (5, 6):
            raise ValueError('Chunk.events requires generic ABI5/6 decoded controls')
        from .postprocessor import _typed

        result = []
        for check in self._parsed['checked']:
            controls = check['control_effects']
            motion = check['motion']
            if not isinstance(controls, list) or len(controls) > 8 or bool(controls) == (motion is not None):
                raise ValueError('invalid checked motion/control effect layout')
            if motion is not None:
                result.append(_typed(dict(kind='motion', motion=motion)))
            else:
                if any(e['kind'] == 'motion' for e in controls):
                    raise ValueError('motion cannot appear inside control effects')
                result.extend(_typed(e) for e in controls)
        return tuple(result)

    @cached_property
    def motions(self):
        result = []
        for check in self._parsed['checked']:
            if check['motion'] is not None:
                data = dict(check['motion'])
                curve = dict(data.pop('curve'))
                kind = curve.pop('kind')
                cls = {'line': Line, 'arc': Arc, 'helix': Helix}[kind]
                for field in ('start', 'end', 'center'):
                    if field in curve:
                        curve[field] = tuple(curve[field])
                result.append(Motion(curve=cls(**curve), **data))
        return tuple(result)


def post_job_input(job, setup_id, *, machine, lowering=None, accept=(), run_id=None):
    """The JSON input of the Rust post (``post_job``, the ``cam-post``
    command) for one setup of ``job``: its stream manifest (what
    ``cam-sim-stream`` reads), program id and work frame, the machine
    (``Machine.to_dict()``), optional ``LoweringOptions`` and the finding
    kinds accepted for release. Plain JSON-ready data."""
    from dataclasses import asdict

    from .export import accepted
    from .simulation import simulation_manifest

    if not isinstance(machine, Machine):
        raise TypeError('post_job_input needs a camlib.post.Machine')
    setup = job.compile().setup(setup_id)
    options = setup.simulation_options()
    data = dict(
        manifest=simulation_manifest(**options),
        program_id=setup.id,
        frame=asdict(setup.frame),
        machine=machine.to_dict(),
        accept=sorted(accepted(accept)),
    )
    if lowering is not None:
        data['lowering'] = asdict(lowering)
    if run_id is not None:
        data['run_id'] = run_id
    return data


def post_job(input, *, cancel=None):
    """Post a setup with the Rust post selected by its machine declaration: ``input``
    as ``post_job_input`` gives it (or its JSON text). Returns the report:
    ``released``, ``program`` (the NC text, only when released), ``refused``,
    ``findings``, ``accepted``, ``unchecked``, ``machine``, ``posting``.
    With no controller, this matches Python's default ``GCodePost``. A named
    controller matches ``Job.post_nc`` with its explicit matching ``post``
    and ``decoder`` classes; Python refuses omitted or mismatched classes.
    See docs/posts.md."""
    text = input if isinstance(input, str) else json.dumps(input, allow_nan=False)
    return json.loads(native.post_job(text, cancel))


def post_events(
    header: Program,
    events,
    machine,
    *,
    cancel=None,
    lowering=None,
    run_id=None,
    post=None,
    decoder=None,
):
    """Yield checked attributed chunks while an ordinary Python event iterator runs.

    A failure leaves only a partial prefix; completion requires the finish chunk.
    Cancellation is observed before begin and inside each native event/batch.
    """
    if not isinstance(machine, Machine):
        raise TypeError('post_events needs a camlib.post.Machine')
    yield from stream_program(
        header,
        events,
        machine,
        post=GCodePost if post is None else post,
        decoder=NativeGCodeDecoder if decoder is None else decoder,
        cancel=cancel,
        run_id=run_id,
        lowering=lowering,
    )


def post_program(program, machine, *, cancel=None, lowering=None, run_id=None, post=None, decoder=None):
    return post_events(
        program,
        program.events,
        machine=machine,
        cancel=cancel,
        lowering=lowering,
        run_id=run_id,
        post=post,
        decoder=decoder,
    )


@dataclass(frozen=True)
class LoweringOptions:
    """Checked conversion of curves to line segments within chord_tolerance_mm.

    native_arcs=True keeps each arc/helix as one G2/G3 block when the machine
    accepts it (planes, helices, max_sweep_rad) and the post can write exactly
    consistent words; only the others become lines. Every block, arc or line,
    is decoded and certified against the planned curve either way.
    """

    chord_tolerance_mm: float = 0.0025
    max_segments_per_source: int = 1_000_000
    max_total_segments: int = 10_000_000
    native_arcs: bool = False


# Public authoring API: machines and posts are ordinary Python definitions,
# checked by the generic intent checker.
from .postprocessor import (  # noqa: E402 (postprocessor imports this module's helpers)
    Command as Command,
    GCodeDialect as GCodeDialect,
    Machine as Machine,
    PostProcessor as PostProcessor,
    GCodePost as GCodePost,
    GCodeDecoder as GCodeDecoder,
    NativeGCodeDecoder as NativeGCodeDecoder,
    PostEngine as PostEngine,
    stream_program as stream_program,
)
