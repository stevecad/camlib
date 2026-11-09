"""Declared physical machine crash model (#116).

All dimensions, poses, calibration ranges, component absences and controller
routes are caller supplied, in mm. Boxes and closed outward convex meshes
form rigid unions. Mounts: fixed, head, table, or workpiece. Axis drives:
head, table, or fixed. No connection to a controller is made.
"""

from dataclasses import asdict, dataclass, field

from .motion import WorkFrame
from .verification import StockContactOptions


@dataclass(frozen=True)
class Box:
    min_mm: tuple
    max_mm: tuple
    kind: str = field(default='box', init=False)


@dataclass(frozen=True)
class ConvexMesh:
    vertices_mm: tuple
    triangles: tuple
    kind: str = field(default='convex_mesh', init=False)


@dataclass(frozen=True)
class Axis:
    drive: str
    min_mm: float
    max_mm: float


@dataclass(frozen=True)
class MachineBody:
    id: str
    component: str
    mount: str
    solids: tuple
    indexed_solids: dict


@dataclass(frozen=True)
class ToolCalibration:
    tool: str
    length_register: int
    body_solids: tuple
    gauge_offset_min_mm: tuple
    gauge_offset_max_mm: tuple


@dataclass(frozen=True)
class AllowedContact:
    bodies: tuple
    reason: str


@dataclass(frozen=True)
class ToolChangeRoute:
    reference: str
    before_index_mm: tuple
    index_envelopes: tuple
    replaces: tuple
    after_index_mm: tuple


@dataclass(frozen=True)
class MachineCheckOptions:
    tolerance_mm: float
    spatial_resolution_mm: float
    max_axis_tests: int
    max_findings: int
    stock_contact: StockContactOptions


@dataclass(frozen=True)
class MachineModel:
    version: int
    units: str
    id: str
    revision: str
    process: str
    axes: tuple
    work_offset: int
    work_to_machine: WorkFrame
    work_error_min_mm: tuple
    work_error_max_mm: tuple
    initial_axes_mm: tuple
    initial_tool: str | None
    rapid_mode: str
    controller_reference: str
    tools: tuple
    bodies: tuple
    absent_components: dict
    allowed_contacts: tuple
    tool_change_routes: dict
    program_end_route_mm: tuple
    options: MachineCheckOptions

    def to_dict(self):
        return asdict(self)
