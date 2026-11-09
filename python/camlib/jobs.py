"""Ordinary Python CAM jobs, immutable input snapshots and native regeneration.

Geometry, stock sweeps, cached delta validation and frame generation stay native.
See docs/python.md for dependency/identity boundaries and current limits.
"""

from dataclasses import asdict, dataclass, is_dataclass
import hashlib
import json
import re
import threading
from pathlib import Path

from .motion import Program, WorkFrame
from .playback import RecordedRun
from .simulation import Simulation, OperationUpdate, SweepOptions

_ID = re.compile(r"[A-Za-z0-9_.-]{1,128}\Z")


def _plain(value):
    if hasattr(value, "to_dict"):
        value = value.to_dict()
    elif is_dataclass(value):
        value = asdict(value)
    return json.loads(json.dumps(value, allow_nan=False, separators=(",", ":")))


def _digest(value):
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, allow_nan=False, separators=(",", ":")).encode()
    ).hexdigest()


def _identifier(value):
    if not isinstance(value, str) or not _ID.fullmatch(value):
        raise ValueError("job identifiers require 1..128 letters/digits/_.-")
    return value


def _semantic_operation(operation):
    # Mesh coordinate/index arrays are numbers by schema (the engine rejects
    # anything else): nothing to strip, so they are not walked number by number.
    arrays = ("triangles", "vertices", "indices")

    def strip(value):
        if isinstance(value, dict):
            return {
                k: (v if k in arrays else strip(v))
                for k, v in value.items()
                if not (k == "source" and ("components" in value or "triangles" in value or "vertices" in value))
            }
        if isinstance(value, list):
            return [strip(v) for v in value]
        return value

    return strip(operation)


@dataclass(frozen=True)
class CadSource:
    id: str
    document_id: str
    revision: str
    reference: str
    geometry_sha256: str
    cadquery_version: str
    ocp_version: str
    brep_format: str

    @classmethod
    def from_shape(cls, id, revision, shape, *, document_id=None, reference="body", limits=None, cancel=None):
        import cadquery as cq
        import OCP
        from .cadquery import FixtureLimits, _fixture_fingerprint

        _identifier(id)
        document_id = document_id or id
        if not isinstance(revision, (str, int)) or not 0 < len(str(revision).encode()) <= 512:
            raise ValueError("invalid CAD source revision")
        if not isinstance(reference, str) or not 0 < len(reference) <= 512:
            raise ValueError("invalid CAD reference")
        limits = limits or FixtureLimits()
        limits.validate()
        if isinstance(shape, cq.Workplane):
            if shape.size() != 1:
                raise ValueError("CAD source needs one resolved shape")
            shape = shape.val()
        if not isinstance(shape, cq.Shape) or not shape.isValid():
            raise ValueError("CAD source needs valid resolved geometry")
        caps = (
            (len(shape.Faces()), limits.max_faces),
            (len(shape.Edges()), limits.max_edges),
            (len(shape.Vertices()), limits.max_vertices),
        )
        if any(cap is not None and n > cap for n, cap in caps):
            raise ValueError("CAD source exceeds the caller's topology caps")
        fingerprint = _fixture_fingerprint(shape, limits, cancel)
        return cls(
            id,
            str(document_id),
            str(revision),
            reference,
            fingerprint['brep_sha256'],
            cq.__version__,
            OCP.__version__,
            fingerprint['brep_format'],
        )

    def semantic(self):
        return dict(
            id=self.id,
            document_id=self.document_id,
            reference=self.reference,
            geometry_sha256=self.geometry_sha256,
            cadquery_version=self.cadquery_version,
            ocp_version=self.ocp_version,
            brep_format=self.brep_format,
        )

    def provenance(self):
        return dict(self.semantic(), revision=self.revision)


@dataclass(frozen=True)
class OperationNode:
    operation: object
    sources: tuple | None = None
    depends_on: tuple = ()
    inputs: object = None
    # camlib.Limits for this operation (added to the setup's).
    limits: object = None


@dataclass(frozen=True)
class Setup:
    id: str
    stock: object
    tools: tuple
    operations: tuple
    initial_position_mm: tuple
    sources: tuple = ()
    frame: WorkFrame = WorkFrame()
    sweep: SweepOptions = SweepOptions()
    verification: object | None = None
    # Where a tool change leaves the new tool (ToolChangePosition); None:
    # the tip stays where the previous operation ended.
    tool_change: object | None = None
    # Limits for every operation: avoid areas (camlib.AvoidArea: clamps,
    # fixtures) and a containment region (the cutter stays inside).
    avoid: tuple = ()
    contain: object = None
    # Leave cuts straight up at rapid (as commercial CAM does) instead of at
    # feed. A retract becomes a rapid only where the stock proves it clear: it
    # leaves from where the last cut left the tool, empty space with only empty
    # space above it in a height field. False keeps every retract at feed.
    rapid_retracts: bool = True
    # A lathe setup: its turning definition (camlib.turning.TurningSetup:
    # the part's profile and bore, the chuck, clearance and tolerance). Its
    # stock is a TurnStockSpec, its tools TurnToolSetting and its operations
    # the Turn* operations; initial_position_mm is (radius, 0, z), the turret
    # index position every tool's controlled point starts from.
    turning: object | None = None


@dataclass(frozen=True)
class RegenerationCache:
    directory: object
    max_entry_bytes: int = 64 * 1024 * 1024
    max_total_bytes: int = 1024 * 1024 * 1024

    def options(self, dependencies):
        return dict(
            directory=str(Path(self.directory).resolve()),
            max_entry_bytes=self.max_entry_bytes,
            max_total_bytes=self.max_total_bytes,
            dependencies=dict(dependencies),
        )


@dataclass(frozen=True)
class CompiledSetup:
    id: str
    _config_json: str
    frame: WorkFrame
    _dependencies: tuple
    _provenance_json: str

    @property
    def config(self):
        return json.loads(self._config_json)

    @property
    def dependencies(self):
        return dict(self._dependencies)

    @property
    def provenance(self):
        return tuple(json.loads(self._provenance_json))

    def simulation_options(self, cache=None):
        # A fresh plain-JSON object: one parse of the compiled configuration.
        result = json.loads(self._config_json)
        if cache is not None:
            result['regeneration'] = cache.options(self.dependencies)
        return result

    def program_header(self):
        from .tools import ToolAssembly

        # Header fields only, read from one cached parse (read-only here).
        c = self.__dict__.get('_header_config')
        if c is None:
            full = self.config
            c = dict(
                revision=full['revision'],
                tools=full['tools'],
                initial_position_mm=full['initial_position_mm'],
                sweep=dict(tolerance_mm=full['sweep']['tolerance_mm']),
            )
            object.__setattr__(self, '_header_config', c)
        return Program(
            self.id,
            c['revision'],
            tuple(ToolAssembly.from_dict(t) for t in c['tools']),
            tuple(c['initial_position_mm']),
            (),
            tolerance_mm=c['sweep']['tolerance_mm'],
            frame=self.frame,
        )


@dataclass(frozen=True)
class CompiledJob:
    id: str
    revision: str
    setups: tuple

    def setup(self, id):
        return next(s for s in self.setups if s.id == id)


def _finite(values, what):
    """Any number of items, but a finite collection: an endless iterator is
    refused instead of being consumed (and nothing is ever truncated)."""
    if not isinstance(values, (list, tuple)):
        raise TypeError(f"{what} must be a list or tuple (a finite collection)")
    return tuple(values)


@dataclass(frozen=True)
class Job:
    id: str
    revision: str
    setups: tuple

    def compile(self):
        _identifier(self.id)
        if not isinstance(self.revision, str) or not 0 < len(self.revision.encode()) <= 400:
            raise ValueError("job revision requires 1..400 UTF8 bytes")
        setups = _finite(self.setups, "job setups")
        if not setups:
            raise ValueError("job requires at least one setup")
        seen = set()
        compiled = []
        snapshot = []
        for setup in setups:
            _identifier(setup.id)
            if setup.id in seen:
                raise ValueError("duplicate setup id")
            seen.add(setup.id)
            source_list = _finite(setup.sources, "setup sources")
            if any(not isinstance(s, CadSource) for s in source_list):
                raise ValueError("setup sources must be CAD snapshots")
            sources = {s.id: s for s in source_list}
            if len(sources) != len(source_list):
                raise ValueError("duplicate CAD source ids")
            tools = tuple(_plain(t) for t in _finite(setup.tools, "setup tools"))
            if not tools:
                raise ValueError("setup requires at least one tool")
            lathe = setup.turning is not None
            if lathe and (
                setup.avoid
                or setup.contain is not None
                or setup.verification is not None
                or setup.tool_change is not None
            ):
                raise ValueError(
                    "a lathe setup checks every move against its part, bore, chuck and stock; "
                    "milling limits, checks and tool-change poses do not apply"
                )
            toolmap = {(t['tool']['id'] if lathe else t['id']): t for t in tools}
            if len(toolmap) != len(tools):
                raise ValueError("duplicate tool id")
            stock = _plain(setup.stock)
            frame = _plain(setup.frame)
            nodes = _finite(setup.operations, "setup operations")
            if not nodes:
                raise ValueError("setup requires at least one operation")
            definitions = []
            dependencies = {}
            provenance = []
            for entry in nodes:
                node = entry if isinstance(entry, OperationNode) else OperationNode(entry)
                definition = _plain(node.operation)
                id = _identifier(definition['id'])
                from .limits import merged

                limits = None if lathe else merged(setup.contain, setup.avoid, node.limits)
                if lathe and node.limits is not None:
                    raise ValueError("turning operations take no milling limits")
                if limits is not None:
                    if 'limits' in definition:
                        raise ValueError(
                            "give limits with OperationNode(limits=...) or the setup, not in the operation"
                        )
                    definition['limits'] = _plain(limits)
                if id in dependencies:
                    raise ValueError("duplicate operation id")
                if definition['tool'] not in toolmap:
                    raise ValueError("operation references unknown tool")
                selected = tuple(sources) if node.sources is None else _finite(node.sources, "operation sources")
                if len(set(selected)) != len(selected) or any(s not in sources for s in selected):
                    raise ValueError("unknown/duplicate CAD source dependency")
                parents = _finite(node.depends_on, "operation dependencies")
                if len(set(parents)) != len(parents) or any(p not in dependencies for p in parents):
                    raise ValueError(
                        "explicit dependencies must name distinct preceding operations (no cycles/forward links)"
                    )
                source_provenance = {s: sources[s].provenance() for s in selected}
                for field in ('region', 'surface'):
                    if field in definition:
                        definition[field].setdefault('source', {})
                        if definition[field]['source'] is None:
                            definition[field]['source'] = {}
                        if not isinstance(definition[field]['source'], dict):
                            definition[field]['source'] = {'authored_source': definition[field]['source']}
                        definition[field]['source']['job_sources'] = source_provenance
                key_inputs = dict(
                    operation=_semantic_operation(definition),
                    tool=toolmap[definition['tool']],
                    stock=stock,
                    frame=frame,
                    sources={s: sources[s].semantic() for s in selected},
                    parents={p: dependencies[p] for p in parents},
                )
                external = None
                if node.inputs is not None:
                    external = _plain(node.inputs)
                    if len(json.dumps(external, allow_nan=False).encode()) > 65536:
                        raise ValueError("operation external inputs exceed 64 KiB")
                    key_inputs['external'] = external
                key = _digest(key_inputs)
                dependencies[id] = key
                definitions.append(definition)
                provenance.append(dict(operation=id, sources=source_provenance, depends_on=parents, input_digest=key))
                if external is not None:
                    provenance[-1]['external'] = external
            if lathe:
                from .turning import proxy_tools

                turning = setup.turning.to_dict(tools, definitions)
                config = dict(
                    stock=stock,
                    tools=proxy_tools(turning),
                    turning=turning,
                    initial_position_mm=tuple(setup.initial_position_mm),
                    sweep=_plain(setup.sweep),
                )
            else:
                config = dict(
                    stock=stock,
                    tools=tools,
                    operations=definitions,
                    initial_position_mm=tuple(setup.initial_position_mm),
                    sweep=_plain(setup.sweep),
                    verification=None if setup.verification is None else _plain(setup.verification),
                )
                if setup.tool_change is not None:
                    config['tool_change'] = setup.tool_change.to_dict()
                # Always explicit: the engine default is on.
                config['rapid_retracts'] = setup.rapid_retracts
            # Snapshot all authored values before spawning a native process. Native
            # run identity binds CAD revisions even when semantic cache keys agree.
            semantic_config = dict(config, operations=[_semantic_operation(op) for op in definitions])
            if lathe:
                # The turning setup (part, chuck, operations) binds the revision.
                semantic_config.pop('operations')
            snapshot.append(dict(id=setup.id, config=semantic_config, frame=frame, provenance=provenance))
            compiled.append((setup, config, dependencies, tuple(provenance)))
        revision = self.revision + '@' + _digest(dict(id=self.id, setups=snapshot))
        result = []
        for setup, config, dependencies, provenance in compiled:
            config['revision'] = revision
            frame = WorkFrame(**{k: tuple(v) for k, v in _plain(setup.frame).items()})
            result.append(
                CompiledSetup(
                    setup.id,
                    json.dumps(config, allow_nan=False),
                    frame,
                    tuple(dependencies.items()),
                    json.dumps(provenance, allow_nan=False),
                )
            )
        return CompiledJob(self.id, revision, tuple(result))

    def regenerate(self, cache=None, *, executable=None):
        return JobRun(self.compile(), cache, executable=executable)

    def record(self, setup_id, cache=None, **options):
        setup = self.compile().setup(setup_id)
        return RecordedRun(**setup.simulation_options(cache), **options)

    def post_nc(
        self, setup_id, *, machine, path=None, accept=(), cache=None, executable=None, on_operation=None, **options
    ):
        """Release actual decoded NC after posted-stock and machine checks.

        Missing machine checking refuses by default. Explicitly accepting
        ``unchecked_machine`` permits formatter-only output. ``path`` is
        replaced atomically after every configured check has passed or its
        findings were explicitly accepted. Options go to ``Job.post``.
        """
        from .export import accepted

        accepted(accept)
        with self.post(
            setup_id, machine=machine, cache=cache, executable=executable, on_operation=on_operation, **options
        ) as run:
            release = run.export(path, accept=accept)
        return PostedProgram(
            release["text"] or "",
            release["lines"],
            release["motions"],
            release["findings"],
            release["path"],
            accepted=release["accepted"],
            unchecked=release["unchecked"],
            material=release["material"],
            machine=release["machine"],
        )

    def post(self, setup_id, *, machine, cache=None, **options):
        from .posted import PostedRun, OperationEvents, check_tool_change_pose

        setup = self.compile().setup(setup_id)
        check_tool_change_pose(machine, setup.config)
        simulation = Simulation(**setup.simulation_options(cache), executable=options.get('executable'))
        source = OperationEvents(simulation, reuse_active_tool=True, on_operation=options.pop('on_operation', None))
        try:
            if 'verification' not in options:
                options['verification'] = setup.config.get('verification')
            if 'sweep' not in options:
                options['sweep'] = setup.config['sweep']
            if 'turning' in setup.config:
                # A lathe setup: the posted moves are replayed on its stock
                # with the turning planner's move kernel and checks.
                options['turning'] = setup.config['turning']
            return PostedRun(setup.program_header(), source, machine=machine, stock=setup.config['stock'], **options)
        except BaseException:
            source.close()
            raise


@dataclass(frozen=True)
class PostedProgram:
    """Checked NC program from Job.post_nc: ``text`` (or written to ``path``),
    its line and motion counts, actual decoded-output checks' findings, the
    finding kinds that were accepted to release it, and the geometric checks
    the setup did not configure (not run: no claim either way)."""

    text: str
    lines: int
    motions: int
    findings: tuple
    path: str | None = None
    accepted: tuple = ()
    unchecked: tuple = ()
    # A lathe setup's material check (the final stock against the part).
    material: dict | None = None
    machine: dict | None = None


@dataclass(frozen=True)
class JobUpdate:
    setup: str
    update: object


class JobRun:
    """Owned sequential setup generation; cancellation closes the live worker.

    Per-operation statistics are retained, not all motion frames. Final states
    are bounded by16 authored setups; use RecordedRun for retained history.
    """

    def __init__(self, job, cache, *, executable=None):
        self.job = job
        self.cache = cache
        self.executable = executable
        self.states = {}
        self.statistics = {}
        self.finished = False
        self._cancel = threading.Event()
        self._simulation = None
        self._started = False

    def run(self):
        if self._started:
            raise RuntimeError("a JobRun can only be consumed once")
        self._started = True
        try:
            for setup in self.job.setups:
                if self._cancel.is_set():
                    raise RuntimeError("job cancelled; retained prefix is partial")
                with Simulation(**setup.simulation_options(self.cache), executable=self.executable) as sim:
                    self._simulation = sim
                    if self._cancel.is_set():
                        sim.cancel()
                    for update in sim.run_operations():
                        if isinstance(update, OperationUpdate) and update.stage == 'end':
                            self.statistics[(setup.id, update.operation)] = dict(update.report.get('regeneration', {}))
                        yield JobUpdate(setup.id, update)
                    self.states[setup.id] = sim.state.clone()
                self._simulation = None
            self.finished = True
        finally:
            if self._simulation is not None:
                self._simulation.close()
                self._simulation = None

    def __iter__(self):
        return self.run()

    def cancel(self):
        self._cancel.set()
        if self._simulation is not None:
            self._simulation.cancel()

    def close(self):
        if not self.finished:
            self.cancel()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
