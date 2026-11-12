"""Opt-in comparison of supplied milling recipes through ordinary checked jobs.

No geometry is reimplemented here. Native planning, decoded stock replay and
the existing release gate decide eligibility. Motion time is the declared
machine's exact-stop estimate, not a prediction of shop-floor cycle time.
"""

from copy import deepcopy
from dataclasses import asdict, dataclass, replace
from itertools import product
import math
import threading
import time

from .jobs import Job, OperationNode, _digest, _finite, _identifier, _plain
from .post import Cancellation, Machine


@dataclass(frozen=True)
class PlanningCandidate:
    id: str
    job: Job
    setup_id: str
    machine: Machine


@dataclass(frozen=True)
class OperationChoices:
    """Explicit operation recipes for one region/tool assignment.

    Each recipe declares its own cutting data and references an assembly and
    register already in the setup/machine tool pool. This permits supplied
    projection variants without guessing tool settings or changing calibration.
    """

    operation_id: str
    recipes: tuple


def recipe_candidates(reference, choices, *, max_candidates):
    """Enumerate declared recipe/tool combinations, retaining the baseline.

    This is proposal generation only. Whole-recipe native checking in
    compare_candidates establishes feasibility; enumeration claims no reach
    or safety, and does not mutate the source job or its tool library.
    """
    if not isinstance(reference, PlanningCandidate):
        raise TypeError('reference must be a PlanningCandidate')
    if type(max_candidates) is not int or max_candidates < 1:
        raise ValueError('max_candidates must be a positive integer')
    choices = _finite(choices, 'operation choices')
    setup = next(s for s in reference.job.setups if s.id == reference.setup_id)
    nodes = tuple(entry if isinstance(entry, OperationNode) else OperationNode(entry) for entry in setup.operations)
    known = {node.operation.id for node in nodes}
    selected, seen = [], set()
    settings = {setting.tool: setting for setting in reference.machine.tools}
    tools = {tool.id for tool in setup.tools}
    baseline = {node.operation.id: _digest(_plain(node.operation)) for node in nodes}
    includes_baseline = True
    for choice in choices:
        if not isinstance(choice, OperationChoices) or choice.operation_id not in known or choice.operation_id in seen:
            raise ValueError('choices must name distinct existing operation ids')
        seen.add(choice.operation_id)
        recipes = _finite(choice.recipes, 'operation recipes')
        if not recipes:
            raise ValueError('each operation choice needs at least one explicit recipe')
        for recipe in recipes:
            if recipe.id != choice.operation_id:
                raise ValueError('replacement recipe must retain the operation id')
            tool = getattr(recipe.tool, 'id', recipe.tool)
            setting = settings.get(tool)
            if tool not in tools or setting is None:
                raise ValueError('recipe tool must already be declared in setup and machine')
            if (recipe.tool_number, recipe.length_register) != (setting.number, setting.length_register):
                raise ValueError('recipe tool number/length register differs from the declared setting')
        unique = {}
        for recipe in recipes:
            unique.setdefault(_digest(_plain(recipe)), recipe)
        selected.append(tuple(unique.values()))
        includes_baseline &= baseline[choice.operation_id] in unique
    count = math.prod(len(recipes) for recipes in selected) + (not includes_baseline)
    if count > max_candidates:
        raise ValueError(f'{count} recipe candidates exceed the declared max_candidates={max_candidates}')
    result, fingerprints = [reference], {_digest([_plain(node.operation) for node in nodes])}
    for combination in product(*selected):
        replacements = dict(zip((choice.operation_id for choice in choices), combination))
        operations = tuple(
            replace(node, operation=replacements.get(node.operation.id, node.operation)) for node in nodes
        )
        fingerprint = _digest([_plain(node.operation) for node in operations])
        if fingerprint in fingerprints:
            continue
        fingerprints.add(fingerprint)
        variant = replace(setup, operations=operations)
        job = replace(reference.job, setups=tuple(variant if s.id == setup.id else s for s in reference.job.setups))
        id = f'{reference.id}-recipe-{len(result)}'
        _identifier(id)
        result.append(PlanningCandidate(id, job, reference.setup_id, reference.machine))
    return tuple(result)


@dataclass(frozen=True)
class ComparisonPolicy:
    """Explicit completion limits and total search budget.

    ``residual_limits_mm3`` is a finite collection of ``(target_id, upper)``
    pairs, one per final material target. A roughing recipe can deliberately
    retain stock; a finishing recipe can ask for a tighter upper bound.
    """

    residual_limits_mm3: tuple
    max_seconds: float

    def __post_init__(self):
        pairs = _finite(self.residual_limits_mm3, 'residual limits')
        limits = []
        for pair in pairs:
            if not isinstance(pair, (list, tuple)) or len(pair) != 2:
                raise ValueError('residual limits need (target_id, upper_mm3) pairs')
            target, upper = pair
            _identifier(target)
            _number(upper, 'residual upper volume', zero=True)
            limits.append((target, float(upper)))
        if not limits or len(dict(limits)) != len(limits):
            raise ValueError('give one residual limit for each distinct final target')
        _number(self.max_seconds, 'planning time budget')
        object.__setattr__(self, 'residual_limits_mm3', tuple(sorted(limits)))


@dataclass(frozen=True)
class CandidateEvaluation:
    id: str
    input_sha256: str
    status: str
    reasons: tuple = ()
    motion_seconds: float | None = None
    tool_changes: int | None = None
    tool_count: int | None = None
    posted_blocks: int | None = None
    residual_mm3: tuple = ()
    findings: tuple = ()
    # Actual recipe-used assemblies: id, tested stick-out, gauge length.
    assemblies: tuple = ()
    release_checked: bool = False
    correction_regions: tuple = ()


@dataclass(frozen=True)
class CorrectionRegion:
    target_id: str
    boxes_mm: tuple
    complete: bool


@dataclass(frozen=True)
class CorrectionTemplate:
    target_id: str
    operation: object


@dataclass(frozen=True)
class CorrectionReport:
    candidate: PlanningCandidate | None
    evaluations: tuple
    reason: str


@dataclass(frozen=True)
class ComparisonReport:
    input_sha256: str
    reference: str
    evaluations: tuple
    selected: str | None
    best_tradeoffs: tuple
    complete: bool

    def to_dict(self):
        return asdict(self)


def _number(value, name, *, zero=False):
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value < 0
        or (not zero and value == 0)
    ):
        raise ValueError(f'{name} must be finite and {"nonnegative" if zero else "positive"}')


def _snapshot(candidate, policy):
    if not isinstance(candidate, PlanningCandidate):
        raise TypeError('candidates must be PlanningCandidate values')
    _identifier(candidate.id)
    if not isinstance(candidate.job, Job) or not isinstance(candidate.machine, Machine):
        raise TypeError('a candidate needs an ordinary Job and declared Machine')
    setup = candidate.job.compile().setup(candidate.setup_id)
    config = setup.config
    if 'turning' in config or any(op['kind'] in ('thermal_cut', 'thermal_mark') for op in config['operations']):
        raise ValueError('recipe comparison currently supports three-axis milling')
    checks = config.get('verification') or {}
    if not all(checks.get(key) for key in ('scene', 'stock_contact', 'material')):
        raise ValueError('comparison requires scene, stock-contact and material checks')
    final = [item['target']['id'] for item in checks['material'] if item['after_operation'] is None]
    if len(final) != len(set(final)) or set(final) != dict(policy.residual_limits_mm3).keys():
        raise ValueError('residual limits must match distinct final material-check targets')
    machine = deepcopy(candidate.machine)
    if machine.crash_model is None:
        raise ValueError('comparison requires a declared physical machine model')
    if not machine.schedule_post_feeds or machine.motion_policy is None:
        raise ValueError('comparison requires explicit motion_policy and schedule_post_feeds=True')
    if machine.motion_policy.junction_mode != 'exact_stop':
        raise ValueError('comparison uses exact-stop posted timing; declare junction_mode="exact_stop"')
    # The full supplied tool pool/table can be shared while recipes choose
    # different tools. A candidate may not quietly change a verification premise.
    common = {key: value for key, value in config.items() if key not in ('operations', 'revision', 'run_id')}
    common.update(frame=asdict(setup.frame), machine=asdict(machine))
    sources = {}
    for entry in setup.provenance:
        sources.update(entry['sources'])
    common['sources'] = sources
    identity = _digest(dict(setup=config, frame=asdict(setup.frame), machine=asdict(machine), policy=asdict(policy)))
    return setup, machine, _digest(common), identity


def _residuals(run, policy):
    reports = {}
    offset = 0
    while page := list(run.verification_records(offset, limit=256)):
        for update in page:
            if update.kind == 'material':
                report = update.check['report']
                reports[report['target_id']] = report
        offset += len(page)
    residuals, reasons, regions = [], [], []
    for target, maximum in policy.residual_limits_mm3:
        report = reports.get(target)
        if report is None or not report['complete']:
            reasons.append(f'{target}: final material analysis is missing or incomplete')
            continue
        lower, upper = report['residual']['volume_mm3']
        residuals.append((target, lower, upper))
        if upper > maximum:
            reasons.append(f'{target}: residual upper volume {upper} mm3 exceeds {maximum} mm3')
            regions.append(
                CorrectionRegion(
                    target,
                    tuple(
                        tuple(finding['xy_mm'])
                        for finding in report['findings']
                        if finding['residual'] != 'within_tolerance'
                    ),
                    report['complete'] and not report['stats']['findings_truncated'],
                )
            )
    return tuple(residuals), tuple(reasons), tuple(regions)


def compare_candidates(candidates, *, reference, policy, cache=None, cancel=None, executable=None):
    """Compare finite, explicitly supplied recipes without releasing NC.

    The reference is evaluated first and wins exact ties. All candidates must
    share stock, source, tool pool, machine and verification premises. No
    acceptance flag or verification override is available through this API.
    Interrupted searches retain diagnostics but select no result.
    """
    from .export import ExportRefused
    from .simulation import SimulationError

    if not isinstance(policy, ComparisonPolicy):
        raise TypeError('declare a ComparisonPolicy')
    deadline = time.monotonic() + policy.max_seconds
    candidates = _finite(candidates, 'planning candidates')
    if any(not isinstance(candidate, PlanningCandidate) for candidate in candidates):
        raise TypeError('candidates must be PlanningCandidate values')
    ids = [candidate.id for candidate in candidates]
    if len(set(ids)) != len(ids) or reference not in ids:
        raise ValueError('candidates need distinct ids and an existing reference')
    ordered = sorted(candidates, key=lambda candidate: candidate.id != reference)
    token = cancel if cancel is not None else Cancellation()
    prepared = []
    for candidate in ordered:
        if token.is_cancelled() or time.monotonic() >= deadline:
            break
        prepared.append((candidate, *_snapshot(candidate, policy)))
        if token.is_cancelled() or time.monotonic() >= deadline:
            break
    if len(prepared) != len(ordered) or token.is_cancelled() or time.monotonic() >= deadline:
        # A request digest explicitly differs from a fully compiled identity.
        # No candidate is evaluated until every common premise was checked.
        digests = [(c.id, _digest(dict(stage='request', candidate=_plain(c)))) for c in ordered]
        return ComparisonReport(
            _digest(dict(stage='preparation_interrupted', candidates=digests, policy=asdict(policy))),
            reference,
            tuple(
                CandidateEvaluation(id, digest, 'incomplete', ('preparation interrupted',)) for id, digest in digests
            ),
            None,
            (),
            False,
        )
    if len({item[3] for item in prepared}) != 1:
        raise ValueError('candidates differ in stock, sources, tool pool, machine or verification premises')
    identity = _digest(dict(candidates=[(item[0].id, item[4]) for item in prepared], reference=reference))
    evaluations = []
    for candidate, setup, machine, _, digest in prepared:
        remaining = deadline - time.monotonic()
        if remaining <= 0 or token.is_cancelled():
            evaluations.append(
                CandidateEvaluation(candidate.id, digest, 'incomplete', ('search cancelled or time budget exhausted',))
            )
            continue
        # A PostedRun owns/cancels its token when closed. Keep the caller's
        # search token separate so finishing one candidate cannot cancel all
        # later candidates, while propagating user cancellation during native work.
        run_token, stopped = Cancellation(), threading.Event()

        def watch():
            while not stopped.wait(min(0.05, max(0, deadline - time.monotonic()))):
                if token.is_cancelled() or time.monotonic() >= deadline:
                    run_token.cancel()
                    return

        watcher = threading.Thread(target=watch, name='cam-planning-cancel', daemon=True)
        watcher.start()
        try:
            with setup.post(machine=machine, cache=cache, cancel=run_token, executable=executable) as run:
                release = run.check_release(timeout=remaining)
                if token.is_cancelled() or time.monotonic() >= deadline:
                    raise TimeoutError('search cancelled or time budget exhausted')
                if release['unchecked']:
                    raise ValueError(f'unchecked verification coverage: {release["unchecked"]}')
                residuals, reasons, regions = _residuals(run, policy)
                status = run.posting_status()
                seconds = status['motion_scheduling']['decoded_exact_stop_seconds']
                _number(seconds, 'decoded motion time', zero=True)
                evaluations.append(
                    CandidateEvaluation(
                        candidate.id,
                        digest,
                        'rejected' if reasons else 'eligible',
                        reasons,
                        seconds,
                        release['machine']['tool_changes'],
                        len({op['tool'] for op in setup.config['operations']}),
                        status['lines'],
                        residuals,
                        release_checked=True,
                        correction_regions=regions,
                    )
                )
        except ExportRefused as error:
            interrupted = token.is_cancelled() or time.monotonic() >= deadline
            evaluations.append(
                CandidateEvaluation(
                    candidate.id,
                    digest,
                    'incomplete' if interrupted else 'rejected',
                    (str(error),),
                    findings=error.findings,
                )
            )
        except (ValueError, SimulationError, TimeoutError) as error:
            interrupted = token.is_cancelled() or time.monotonic() >= deadline or isinstance(error, TimeoutError)
            evaluations.append(
                CandidateEvaluation(candidate.id, digest, 'incomplete' if interrupted else 'rejected', (str(error),))
            )
        finally:
            stopped.set()
            watcher.join()
        used = {op['tool'] for op in setup.config['operations']}
        assemblies = tuple(
            sorted(
                (tool['id'], tool['stickout_mm'], tool['gauge_length_mm'])
                for tool in setup.config['tools']
                if tool['id'] in used
            )
        )
        evaluations[-1] = replace(evaluations[-1], assemblies=assemblies)
    eligible = [item for item in evaluations if item.status == 'eligible']
    complete = (
        not token.is_cancelled()
        and time.monotonic() < deadline
        and all(item.status != 'incomplete' for item in evaluations)
    )
    best = min(eligible, key=lambda item: (item.motion_seconds, item.tool_changes, item.id != reference), default=None)
    frontier = tuple(
        item.id
        for item in eligible
        if not any(
            other.motion_seconds <= item.motion_seconds
            and other.tool_changes <= item.tool_changes
            and (other.motion_seconds < item.motion_seconds or other.tool_changes < item.tool_changes)
            for other in eligible
        )
    )
    return ComparisonReport(
        identity, reference, tuple(evaluations), best.id if best and complete else None, frontier, complete
    )


def correction_candidate(candidate, evaluation, templates, *, policy, cancel=None, _deadline=None):
    """Append native rest operations confined to reported residual regions.

    This is an unverified proposal. It must go through compare_candidates
    again. No tolerance, stock premise, tool setting or source target changes.
    Native region unions/offsets/intersections preserve existing containment.
    """
    from . import Engine
    from .limits import Limits
    from .planar import Boundary, Component, PlanarRegion

    deadline = time.monotonic() + policy.max_seconds
    if _deadline is not None:
        deadline = min(deadline, _deadline)

    def check_budget():
        if time.monotonic() >= deadline or cancel is not None and cancel.is_cancelled():
            raise ValueError('correction cancelled or time budget exhausted')

    check_budget()
    if not isinstance(evaluation, CandidateEvaluation) or not evaluation.release_checked:
        raise ValueError('correction needs a completed release check, not a collision or uncertain result')
    if _snapshot(candidate, policy)[3] != evaluation.input_sha256:
        raise ValueError('correction findings belong to different inputs or policy')
    check_budget()
    templates = _finite(templates, 'correction templates')
    mapping = {}
    for template in templates:
        if not isinstance(template, CorrectionTemplate) or template.target_id in mapping:
            raise ValueError('supply distinct CorrectionTemplate targets')
        mapping[template.target_id] = template.operation
    setup = next(s for s in candidate.job.setups if s.id == candidate.setup_id)
    tools = {tool.id: tool for tool in setup.tools}
    operations = list(setup.operations)
    for region in evaluation.correction_regions:
        check_budget()
        if not region.complete or not region.boxes_mm:
            raise ValueError(f'{region.target_id}: residual locations are incomplete or truncated')
        if region.target_id not in mapping:
            raise ValueError(f'{region.target_id}: supply an explicit rest-operation template')
        template = mapping[region.target_id]
        node = template if isinstance(template, OperationNode) else OperationNode(template)
        operation = node.operation
        if (
            getattr(operation, 'rest_threshold_mm', None) is None
            and getattr(operation, 'rest_tolerance_mm', None) is None
        ):
            raise ValueError('correction requires an explicit rest-finishing or RestPocket operation')
        tool_id = getattr(operation.tool, 'id', operation.tool)
        if tool_id not in tools:
            raise ValueError('correction tool must be in the existing declared pool')

        def rectangle(box):
            x0, y0, x1, y1 = box
            return PlanarRegion('residual', (Component(Boundary.polygon(((x0, y0), (x1, y0), (x1, y1), (x0, y1)))),))

        patch = rectangle(region.boxes_mm[0])
        with Engine() as engine:
            for box in region.boxes_mm[1:]:
                check_budget()
                engine.prepare_region(patch)
                check_budget()
                patch = PlanarRegion.from_dict(engine.boolean_region(rectangle(box), 'union')['region'])
            check_budget()
            engine.prepare_region(patch)
            check_budget()
            # A centre may lie a radius from a residual point; containing its
            # whole footprint needs a further radius. The native operation
            # and all independent checks still decide whether it can cut.
            patch = PlanarRegion.from_dict(engine.offset_region(-tools[tool_id].cutter.diameter_mm)['region'])
            for contain in (setup.contain, node.limits.contain if node.limits else None):
                if contain is not None:
                    check_budget()
                    engine.prepare_region(patch)
                    check_budget()
                    patch = PlanarRegion.from_dict(engine.boolean_region(contain, 'intersection')['region'])
        check_budget()
        if not patch.components:
            raise ValueError(f'{region.target_id}: no residual correction region inside declared containment')
        id = f'{operation.id}-correction-{len(operations)}'
        _identifier(id)
        previous = operations[-1]
        previous = previous.operation if isinstance(previous, OperationNode) else previous
        operations.append(
            replace(
                node,
                operation=replace(operation, id=id),
                depends_on=tuple(dict.fromkeys((*node.depends_on, previous.id))),
                limits=Limits(patch, node.limits.avoid if node.limits else ()),
                inputs=dict(
                    authored=_plain(node.inputs),
                    correction_input_sha256=evaluation.input_sha256,
                    target=region.target_id,
                ),
            )
        )
    if len(operations) == len(setup.operations):
        raise ValueError('no residual regions require correction')
    revised = replace(setup, operations=tuple(operations))
    job = replace(candidate.job, setups=tuple(revised if s.id == setup.id else s for s in candidate.job.setups))
    return replace(candidate, job=job)


def correct_residuals(candidate, templates, *, policy, max_passes, cache=None, cancel=None, executable=None):
    """Bounded feedback loop using ordinary jobs and independent final checks.

    Stops on safety findings, incomplete location reports, no decrease in the
    residual upper volume, cancellation or explicit pass/time limits. Returns
    a candidate only after every comparison requirement is met.
    """
    if type(max_passes) is not int or max_passes < 0:
        raise ValueError('max_passes must be a nonnegative integer')
    if not isinstance(policy, ComparisonPolicy):
        raise TypeError('declare a ComparisonPolicy')
    templates = _finite(templates, 'correction templates')
    deadline = time.monotonic() + policy.max_seconds
    history, previous = [], None
    for attempt in range(max_passes + 1):
        remaining = deadline - time.monotonic()
        if remaining <= 0 or cancel is not None and cancel.is_cancelled():
            return CorrectionReport(None, tuple(history), 'cancelled or time budget exhausted')
        active = replace(policy, max_seconds=remaining)
        report = compare_candidates(
            (candidate,), reference=candidate.id, policy=active, cache=cache, cancel=cancel, executable=executable
        )
        result = report.evaluations[0]
        history.append(result)
        if not report.complete or not result.release_checked:
            return CorrectionReport(None, tuple(history), 'safety check failed or analysis incomplete')
        current = {target: upper for target, _, upper in result.residual_mm3}
        if current.keys() != dict(active.residual_limits_mm3).keys():
            return CorrectionReport(None, tuple(history), 'final residual analysis incomplete')
        if previous is not None and any(upper > previous[target] for target, upper in current.items()):
            return CorrectionReport(None, tuple(history), 'residual upper volume increased')
        if current == previous:
            return CorrectionReport(None, tuple(history), 'correction made no progress')
        if report.selected is not None:
            return CorrectionReport(candidate, tuple(history), 'completion limits satisfied')
        if attempt == max_passes:
            break
        previous = current
        try:
            candidate = correction_candidate(
                candidate, result, templates, policy=active, cancel=cancel, _deadline=deadline
            )
        except ValueError as error:
            return CorrectionReport(None, tuple(history), str(error))
    return CorrectionReport(None, tuple(history), 'correction pass budget exhausted')
