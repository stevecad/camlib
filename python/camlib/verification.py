"""Prepared native continuous checks, independent of planner/stock removal.

Clear applies only to the configured prism/travel/feed/nominal-curve checks named in a report.
Evolving stock, remaining material and controller execution are not checked here.
"""

from dataclasses import asdict, dataclass
import itertools

from . import Engine
from .motion import WorkFrame


def _finite(values, what):
    """Any number of items, but a finite collection: an endless iterator is
    refused instead of being consumed."""
    if not isinstance(values, (list, tuple)):
        raise TypeError(f"{what} must be a list or tuple (a finite collection)")
    return tuple(values)


@dataclass(frozen=True)
class Prism:
    id: str
    footprint: object
    z_min_mm: float
    z_max_mm: float
    role: str = "fixture"
    z_error_mm: float = 0
    representation: str = "solid_prism"

    def to_dict(self):
        data = dict(
            id=self.id,
            role=self.role,
            footprint=self.footprint.to_dict() if hasattr(self.footprint, "to_dict") else self.footprint,
            z_min_mm=self.z_min_mm,
            z_max_mm=self.z_max_mm,
            z_error_mm=self.z_error_mm,
        )
        if self.representation != "solid_prism":
            data["representation"] = self.representation
        return data


@dataclass(frozen=True)
class MachineLimits:
    """A machine's travel and feed limits for the planned-move checks.

    Travel is declared once, in machine coordinates, limits included:

    - ``min_tip_mm``/``max_tip_mm``: the travel of one tool's tip. It holds
      for one gauge length, so it fits a job whose tools share it.
    - ``min_gauge_mm``/``max_gauge_mm`` (``MachineLimits.gauge_line``): the
      travel of the spindle gauge line, the machine's own axis travel (its
      soft limits act on the machine position whatever length offset is
      active). Each tool's tip is checked against that box lowered along
      machine Z by the tool's ``gauge_length_mm``, so tools of different
      lengths each get their own travel. Needs the work Z axis along machine
      Z, and tools held in a spindle (not a beam).

    A position on a limit is inside, and a declared end is compared exactly:
    the motion tolerance is not a travel allowance (a line end 1e-6 mm beyond
    a limit is a violation). The posted-program check holds the decoded words
    to the same rule; see "The travel rule" in docs/python.md."""

    min_tip_mm: tuple | None
    max_tip_mm: tuple | None
    max_feed_mm_min: float
    max_normal_accel_mm_s2: float | None = None
    max_axis_feed_mm_min: tuple | None = None
    min_gauge_mm: tuple | None = None
    max_gauge_mm: tuple | None = None

    @classmethod
    def gauge_line(
        cls, min_gauge_mm, max_gauge_mm, max_feed_mm_min, max_normal_accel_mm_s2=None, max_axis_feed_mm_min=None
    ):
        """Limits with the travel declared on the spindle gauge line."""
        return cls(
            None, None, max_feed_mm_min, max_normal_accel_mm_s2, max_axis_feed_mm_min, min_gauge_mm, max_gauge_mm
        )

    def __post_init__(self):
        tip = (self.min_tip_mm is not None, self.max_tip_mm is not None)
        gauge = (self.min_gauge_mm is not None, self.max_gauge_mm is not None)
        if (all(tip), all(gauge)) not in ((True, False), (False, True)) or any(tip) and any(gauge):
            raise ValueError(
                "machine travel is declared once: either the tip box of one tool length (min_tip_mm and "
                "max_tip_mm) or the spindle gauge line's box (min_gauge_mm and max_gauge_mm, "
                "MachineLimits.gauge_line), which gives every tool its own tip travel from its gauge length"
            )
        if self.max_axis_feed_mm_min is not None:
            import math

            values = tuple(itertools.islice(iter(self.max_axis_feed_mm_min), 4))
            if len(values) != 3 or any(
                isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or not 1e-6 <= v <= 1e7
                for v in values
            ):
                raise ValueError("max_axis_feed_mm_min requires three finite positive axis limits in [1e-6, 1e7]")
            object.__setattr__(self, "max_axis_feed_mm_min", values)

    def to_dict(self):
        data = asdict(self)
        if self.max_normal_accel_mm_s2 is None:
            data.pop("max_normal_accel_mm_s2")
        if self.max_axis_feed_mm_min is None:
            data.pop("max_axis_feed_mm_min")
        for key in ("min_tip_mm", "max_tip_mm", "min_gauge_mm", "max_gauge_mm"):
            if data[key] is None:
                data.pop(key)
        return data


@dataclass(frozen=True)
class Scene:
    id: str
    revision: str
    bodies: tuple = ()
    machine: MachineLimits | None = None
    work_to_machine: WorkFrame = WorkFrame()

    def __post_init__(self):
        object.__setattr__(self, "bodies", _finite(self.bodies, "scene bodies"))

    def to_dict(self):
        return dict(
            version=1,
            units="mm",
            id=self.id,
            revision=self.revision,
            work_to_machine=asdict(self.work_to_machine),
            bodies=[b.to_dict() for b in self.bodies],
            machine=None if self.machine is None else self.machine.to_dict(),
        )


@dataclass(frozen=True)
class Options:
    tolerance_mm: float = 1e-5
    spatial_resolution_mm: float = 0.001
    max_nodes: int = 100_000
    max_boundary_tests: int = 10_000_000
    max_findings: int = 1024

    # Authored floor machining allowance for fed cutters on protected targets.
    # None keeps strict contact checks; fixtures/noncutting parts stay strict.
    target_floor_tolerance_mm: float | None = None


@dataclass(frozen=True)
class StockContactOptions:
    tolerance_mm: float = 1e-5
    spatial_resolution_mm: float = 0.001
    max_nodes: int = 200_000
    max_cell_reads: int = 2_000_000
    max_findings: int = 128


@dataclass(frozen=True)
class MaterialCheck:
    """Inspect the actual stock after an operation, or at run end (None)."""

    target: object
    after_operation: str | None = None
    options: object = None

    def to_dict(self):
        result = dict(after_operation=self.after_operation, target=self.target.to_dict())
        if self.options is not None:
            result["options"] = asdict(self.options)
        return result


@dataclass(frozen=True)
class StreamChecks:
    """Immutable inputs for partial streamed geometry/material checking."""

    scene: Scene | None = None
    options: Options = Options()
    material: tuple = ()
    stock_contact: StockContactOptions | None = None

    def __post_init__(self):
        object.__setattr__(self, "material", _finite(self.material, "material checks"))

    def to_dict(self):
        result = dict(
            scene=None if self.scene is None else dict(scene=self.scene.to_dict(), options=asdict(self.options)),
            material=[m.to_dict() for m in self.material],
        )
        if self.stock_contact is not None:
            result["stock_contact"] = asdict(self.stock_contact)
        return result


class Verifier:
    """Prepare bodies once and check motion incrementally (in-process native session)."""

    def __init__(self, scene, options=Options()):
        self._engine = Engine()
        try:
            self.prepare(scene, options)
        except BaseException:
            self.close()
            raise

    def prepare(self, scene, options=Options()):
        # Both native state and client identity change only after validation.
        prepared = self._engine._request("prepare_verification", scene=scene.to_dict(), options=asdict(options))
        self._identity = {k: prepared[k] for k in ("scene_id", "revision", "scene_sha256")}
        return dict(self._identity)

    def check(self, tool, motion):
        data = motion.to_dict() if hasattr(motion, "to_dict") else motion
        if data.get("kind") == "motion":
            data = data["motion"]
        return self._engine._request(
            "verify_motion", **self._identity, tool=tool.to_dict() if hasattr(tool, "to_dict") else tool, motion=data
        )

    def check_motions(self, tool, motions):
        for motion in motions:
            yield self.check(tool, motion)

    def close(self):
        self._engine.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
