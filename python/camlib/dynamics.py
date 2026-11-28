"""Explicit machine feed scheduling and nominal acceleration-aware motion time.

This complete-program API leaves CAD geometry and machining order unchanged.
It does not predict controller blending, jerk, rapid routing or operator waits.
"""

from dataclasses import asdict, dataclass, replace
import json
import math

from .motion import Motion, Program


def _positive_finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and value > 0


@dataclass(frozen=True)
class MotionPolicy:
    tangential_accel_mm_s2: float
    rapid_feed_mm_min: float
    junction_mode: str = 'exact_stop'
    feed_reserve_fraction: float = 0.01
    # Seconds added at every stop (a segment ending at rest): a controller's
    # per-block cost tangential acceleration does not model, such as
    # acceleration/deceleration after interpolation, block processing and
    # in-position checks. 0 keeps the schedule and its identity unchanged.
    per_block_s: float = 0.0

    def __post_init__(self):
        # Numeric safety only: the machine's declarations (#433).
        if not _positive_finite(self.tangential_accel_mm_s2):
            raise ValueError(
                f'motion policy: tangential_accel_mm_s2 {self.tangential_accel_mm_s2} must be positive and finite '
                '(MotionPolicy.tangential_accel_mm_s2)'
            )
        if not _positive_finite(self.rapid_feed_mm_min):
            raise ValueError(
                f'motion policy: rapid_feed_mm_min {self.rapid_feed_mm_min} must be positive and finite '
                '(MotionPolicy.rapid_feed_mm_min)'
            )
        if self.junction_mode not in ('exact_stop', 'tangent'):
            raise ValueError(
                f'motion policy: junction_mode {self.junction_mode!r} must be "exact_stop" or "tangent" '
                '(MotionPolicy.junction_mode)'
            )
        # Definition: a fraction in [0, 1) so (1 - reserve) stays positive.
        if (
            isinstance(self.feed_reserve_fraction, bool)
            or not isinstance(self.feed_reserve_fraction, (int, float))
            or not math.isfinite(self.feed_reserve_fraction)
            or not 0 <= self.feed_reserve_fraction < 1
        ):
            raise ValueError(
                f'motion policy: feed_reserve_fraction {self.feed_reserve_fraction} must be finite and in [0, 1) '
                '(MotionPolicy.feed_reserve_fraction)'
            )
        # Numeric safety only: finite and nonnegative (#433).
        if (
            isinstance(self.per_block_s, bool)
            or not isinstance(self.per_block_s, (int, float))
            or not math.isfinite(self.per_block_s)
            or self.per_block_s < 0
        ):
            raise ValueError(
                f'motion policy: per_block_s {self.per_block_s} must be finite and nonnegative '
                '(MotionPolicy.per_block_s)'
            )


@dataclass(frozen=True)
class ScheduledProgram:
    program: Program
    report: dict

    @property
    def motion_seconds(self):
        return self.report['motion_seconds']


def _settings(machine, adjust_feeds):
    # Numeric safety only: validate machine feed/accel limits before JSON
    # serialization so non-finite values name the field and value (#433).
    if not _positive_finite(machine.max_feed_mm_min):
        raise ValueError(
            f'motion policy: max_feed_mm_min {machine.max_feed_mm_min} must be positive and finite '
            '(Machine.max_feed_mm_min)'
        )
    if machine.max_normal_accel_mm_s2 is not None and not _positive_finite(machine.max_normal_accel_mm_s2):
        raise ValueError(
            f'motion policy: max_normal_accel_mm_s2 {machine.max_normal_accel_mm_s2} must be positive and finite '
            '(Machine.max_normal_accel_mm_s2)'
        )
    if machine.max_axis_feed_mm_min is not None:
        for k, x in enumerate(machine.max_axis_feed_mm_min):
            if not _positive_finite(x):
                raise ValueError(
                    f'motion policy: max_axis_feed_mm_min[{k}] {x} must be positive and finite '
                    '(Machine.max_axis_feed_mm_min)'
                )
    feed_quantum_mm_min = (25.4 if machine.output_units == 'inch' else 1) * 10**-machine.decimals
    if not _positive_finite(feed_quantum_mm_min):
        raise ValueError(f'motion policy: feed_quantum_mm_min {feed_quantum_mm_min} must be positive and finite')
    settings = dict(
        policy=asdict(machine.motion_policy),
        max_feed_mm_min=machine.max_feed_mm_min,
        max_normal_accel_mm_s2=machine.max_normal_accel_mm_s2,
        max_axis_feed_mm_min=machine.max_axis_feed_mm_min,
        work_to_machine=asdict(machine.work_to_machine),
        feed_quantum_mm_min=feed_quantum_mm_min,
        adjust_feeds=adjust_feeds,
    )
    return settings


def schedule(program, machine, *, adjust_feeds=True, cancel=None):
    """Return a new typed program plus timing; never mutate the source program.

    ``exact_stop`` stops every block; ``tangent`` carries speed only across
    numerically tangent fed moves. Every state event/rapid boundary/corner stops.
    Both use scalar constant tangential acceleration, without jerk or axis
    acceleration. Optional existing machine normal-acceleration/axis-speed limits
    constrain feed. A small explicit reserve accounts for output sensitivity;
    independent checked posting can still reject rounded geometry.

    Only feed words change. This is an offline complete-program lookahead, with
    at most one million events. It does not buffer the live simulation stream.
    Retain the returned program as the source for posting/replay, and the report's
    identity with derived artifacts. A policy or source revision changes identity.
    """
    from ._buffers import native

    if not isinstance(program, Program):
        raise TypeError('schedule requires a typed Program')
    if not isinstance(machine.motion_policy, MotionPolicy):
        raise ValueError('configure Machine.motion_policy before scheduling')
    if program.frame != machine.frame:
        raise ValueError('program and machine work frame differ')
    if len(program.events) > 1_000_000:
        raise ValueError('motion scheduling exceeds one million events')
    settings = _settings(machine, adjust_feeds)
    if cancel is not None and cancel.is_cancelled():
        raise ValueError('motion scheduling cancelled; no result committed')
    report = json.loads(
        native.schedule_motion(
            json.dumps(program.to_dict(), allow_nan=False), json.dumps(settings, allow_nan=False), cancel
        )
    )
    events = list(program.events)
    for segment in report['segments']:
        i = segment['event_index']
        event = events[i]
        if not isinstance(event, Motion) or event.id != segment['motion_id']:
            raise RuntimeError('native scheduling event identity mismatch')
        events[i] = replace(event, feed_mm_min=segment['scheduled_feed_mm_min'])
    return ScheduledProgram(replace(program, events=tuple(events)), report)


def schedule_events(header, events, machine, *, adjust_feeds=True, cancel=None):
    """Collect a complete typed/native event source for offline lookahead.

    Owns/closes the iterator. Cancellation is polled before pulls; a blocking
    external producer must also expose/use its own cancellation (OperationEvents
    does). Live posting/simulation remain separate bounded streaming APIs.
    """
    from .postprocessor import _typed, _event_data

    if header.events:
        raise ValueError('schedule_events requires an empty program header')
    source = iter(events)
    collected = []
    try:
        while True:
            if cancel is not None and cancel.is_cancelled():
                raise ValueError('motion scheduling cancelled; no result committed')
            try:
                event = next(source)
            except StopIteration:
                break
            if len(collected) >= 1_000_000:
                raise ValueError('motion scheduling exceeds one million events')
            collected.append(_typed(_event_data(event)))
        return schedule(replace(header, events=tuple(collected)), machine, adjust_feeds=adjust_feeds, cancel=cancel)
    finally:
        close = getattr(source, 'close', None)
        if close is not None:
            close()
