"""Automatic nesting of sheet parts for plasma and laser cutting.

A ``Nest`` places copies of parts on a sheet and returns a ``NestResult``:
the placements and the placed parts as one ``PlanarRegion``, ready to be the
``parts`` of a ``ThermalCut``. The engine plans the layout (bottom-left fill)
and proves it with an independent check; this module only declares the
inputs and reads the result. See docs/nesting.md.
"""

from dataclasses import dataclass
import json

from .operations import Lead
from .planar import PlanarRegion


def _request(command, **fields):
    from ._buffers import native

    return json.loads(native.RegionSession().request(json.dumps({"command": command, **fields}, allow_nan=False)))


def _region(region):
    return region.to_dict() if hasattr(region, "to_dict") else region


@dataclass(frozen=True)
class NestPart:
    """A part to nest: ``region`` (a PlanarRegion of one outline with holes),
    how many copies (``quantity``) and the rotations it may take
    (``rotations_deg``, counter-clockwise; ``(0,)`` keeps it as drawn, for
    grained or brushed sheet). ``mirror=True`` also allows the part turned
    over. There are no defaults for the quantity or the rotations.

    ``priority``: parts of a higher priority are placed first, so nearest the
    start corner (within a priority, largest first). ``fill=True`` makes the
    part a filler: placed after every other part, as many copies as fit up to
    ``quantity``; copies that do not fit are not a refusal, and the result's
    ``filled`` says how many were placed."""

    region: object
    quantity: int
    rotations_deg: tuple
    mirror: bool = False
    priority: int = 0
    fill: bool = False

    def to_dict(self):
        return dict(
            region=_region(self.region),
            quantity=self.quantity,
            rotations_deg=list(self.rotations_deg),
            mirror=self.mirror,
            priority=self.priority,
            fill=self.fill,
        )


@dataclass(frozen=True)
class NestPlacement:
    """Where one copy of a part lies: a point p of part ``part`` moves to
    R(``rotation_deg``) M(p) + ``translation_mm``, where M mirrors x when
    ``mirrored``."""

    part: int
    copy: int
    rotation_deg: float
    mirrored: bool
    translation_mm: tuple

    def to_dict(self):
        return dict(
            part=self.part,
            copy=self.copy,
            rotation_deg=self.rotation_deg,
            mirrored=self.mirrored,
            translation_mm=list(self.translation_mm),
        )


@dataclass(frozen=True)
class NestResult:
    """A proven nest. ``region`` holds the placed parts (one component per
    entry of ``placements``, in the same order): pass it as the ``parts`` of
    the ThermalCut. ``unplaced`` lists ``(part, quantity)`` that did not fit
    (only with ``allow_unplaced``). ``utilisation`` is the placed part area
    over the sheet area. ``check`` is the independent check's report: the
    required and the smallest found distances between parts and to the
    sheet's edge."""

    placements: tuple
    unplaced: tuple
    region: PlanarRegion
    utilisation: float
    placed_area_mm2: float
    sheet_area_mm2: float
    used_bounds_mm: tuple | None
    lead_reach_mm: float
    check: dict
    report: dict
    # Copies placed of each filler part: ((part, copies), ...).
    filled: tuple = ()


def _nest_result(report):
    return NestResult(
        placements=tuple(
            NestPlacement(p["part"], p["copy"], p["rotation_deg"], p["mirrored"], tuple(p["translation_mm"]))
            for p in report["placements"]
        ),
        unplaced=tuple((u["part"], u["quantity"]) for u in report["unplaced"]),
        region=PlanarRegion.from_dict(report["region"]),
        utilisation=report["utilisation"],
        placed_area_mm2=report["placed_area_mm2"],
        sheet_area_mm2=report["sheet_area_mm2"],
        used_bounds_mm=tuple(report["used_bounds_mm"]) if report.get("used_bounds_mm") else None,
        lead_reach_mm=report["lead_reach_mm"],
        check=report["check"],
        report=report,
        filled=tuple((part, copies) for part, copies in report.get("filled", ())),
    )


@dataclass(frozen=True)
class Remnant:
    """What is left of a sheet after a cut (``Nest.remnant``). ``region`` is
    all of it, in the cut sheet's own coordinates; ``pieces`` are its
    separate pieces, largest first, each a one-outline ``PlanarRegion`` that
    can be the ``sheet`` of the next Nest. ``check`` is the engine's check of
    the result at points of every part's outline."""

    region: object
    pieces: tuple
    area_mm2: float
    sheet_area_mm2: float
    cut_reach_mm: float
    remnant_margin_mm: float
    check: dict
    report: dict


@dataclass(frozen=True)
class Nest:
    """Nest ``parts`` (NestPart) on ``sheet`` (a PlanarRegion: one outline,
    with holes where a remnant is already cut) for a thermal cut.

    Declare what the cut will use: ``kerf_mm`` (the beam tool's kerf width),
    its ``lead_in`` and ``lead_out`` and ``pierce_clearance_mm`` (default one
    kerf, as in ThermalCut); ``Nest.for_cut`` reads them from a ThermalCut.
    ``part_spacing_mm`` is the material left between what is cut for two
    parts (kerfs, leads and pierce craters), ``edge_margin_mm`` the material
    left to the sheet's outline and holes. ``resolution_mm`` is the column
    spacing of the placement: finer nests tighter and takes longer.

    Parts are placed largest first, each at the lowest free position of the
    leftmost column that has one (bottom-left fill), also in holes of placed
    parts unless ``part_in_part=False``. ``search_iterations`` exchanges of
    the placement order are tried with ``seed``; the same request always
    gives the same nest. The result is a heuristic layout, not the densest
    possible one.

    ``start_corner`` ("lower_left", "lower_right", "upper_left" or
    "upper_right" of the sheet's bounding box) is where the fill starts, and
    ``fill_axis`` the axis parts are stacked along from there before the nest
    advances along the other: "y" fills a column, then the next column in x;
    "x" fills a row, then the next row in y. The remnant is left on the far
    side. Neither turns or mirrors a part.

    ``run()`` returns a NestResult whose distances are proven by a check
    that is independent of the planner. Parts that do not fit are refused
    with a ValueError naming them, or listed in the result with
    ``allow_unplaced=True``; none is dropped silently."""

    sheet: object
    parts: tuple
    kerf_mm: float
    lead_in: Lead
    lead_out: Lead
    part_spacing_mm: float
    edge_margin_mm: float
    resolution_mm: float
    pierce_clearance_mm: float | None = None
    part_in_part: bool = True
    allow_unplaced: bool = False
    search_iterations: int = 0
    seed: int = 0
    start_corner: str = "lower_left"
    fill_axis: str = "y"
    boundary_tolerance_mm: float = 0.01
    tolerance_mm: float = 1e-5

    @classmethod
    def for_cut(cls, cut, sheet, parts, *, part_spacing_mm, edge_margin_mm, resolution_mm, **options):
        """A nest for the ThermalCut ``cut``: its tool's kerf, leads, pierce
        clearance and tolerances. Then cut the result with
        ``dataclasses.replace(cut, parts=result.region)``."""
        return cls(
            sheet,
            tuple(parts),
            cut.tool.cutter.diameter_mm,
            cut.lead_in,
            cut.lead_out,
            part_spacing_mm,
            edge_margin_mm,
            resolution_mm,
            pierce_clearance_mm=cut.pierce_clearance_mm,
            boundary_tolerance_mm=cut.boundary_tolerance_mm,
            tolerance_mm=cut.tolerance_mm,
            **options,
        )

    def to_dict(self):
        result = dict(
            sheet=_region(self.sheet),
            parts=[p.to_dict() for p in self.parts],
            kerf_mm=self.kerf_mm,
            lead_in=dict(length_mm=self.lead_in.length_mm, radius_mm=self.lead_in.radius_mm),
            lead_out=dict(length_mm=self.lead_out.length_mm, radius_mm=self.lead_out.radius_mm),
            part_spacing_mm=self.part_spacing_mm,
            edge_margin_mm=self.edge_margin_mm,
            resolution_mm=self.resolution_mm,
            part_in_part=self.part_in_part,
            allow_unplaced=self.allow_unplaced,
            search_iterations=self.search_iterations,
            seed=self.seed,
            start_corner=self.start_corner,
            fill_axis=self.fill_axis,
            boundary_tolerance_mm=self.boundary_tolerance_mm,
            tolerance_mm=self.tolerance_mm,
        )
        if self.pierce_clearance_mm is not None:
            result["pierce_clearance_mm"] = self.pierce_clearance_mm
        return result

    def run(self):
        """Plan and prove the nest. Raises ValueError when the request is
        invalid or parts do not fit (unless ``allow_unplaced``)."""
        return _nest_result(_request("nest", nest=self.to_dict()))

    def run_sheets(self, sheets):
        """Nest the parts over several ``sheets`` (PlanarRegions: full sheets
        and remnants) in the order given; this nest's own ``sheet`` is not
        used. Each sheet takes what fits of what is left. Returns
        ``((sheet index, NestResult), ...)`` for the sheets that took parts
        and the required parts left over as ``((part, quantity), ...)``:
        refused, naming them, unless ``allow_unplaced``. A filler part fills
        every sheet that is used."""
        report = _request("nest_sheets", nest=self.to_dict(), sheets=[_region(s) for s in sheets])
        return (
            tuple((entry["sheet"], _nest_result(entry["nest"])) for entry in report["sheets"]),
            tuple((u["part"], u["quantity"]) for u in report["unplaced"]),
        )

    def remnant(self, parts, *, remnant_margin_mm):
        """What is left of the sheet after ``parts`` (a NestResult, or the
        ``PlanarRegion`` a ThermalCut cut) are cut from it with this nest's
        kerf and leads: a ``Remnant`` whose ``region`` is the sheet less every
        part grown by all its cut can reach (kerf, leads, pierce craters) and
        by ``remnant_margin_mm`` (no default: the edge you give up for heat,
        dross and how exactly the sheet is put back). It is in the sheet's own
        coordinates, so it is the ``sheet`` of the next Nest when the remnant
        is put back against the same reference; ``camlib.dxf.write_regions``
        saves it as a DXF."""
        region = parts.region if isinstance(parts, NestResult) else parts
        fields = self.to_dict()
        report = _request(
            "remnant",
            remnant=dict(
                sheet=fields["sheet"],
                parts=_region(region),
                remnant_margin_mm=remnant_margin_mm,
                **{
                    key: fields[key]
                    for key in (
                        "kerf_mm",
                        "lead_in",
                        "lead_out",
                        "pierce_clearance_mm",
                        "boundary_tolerance_mm",
                        "tolerance_mm",
                    )
                    if key in fields
                },
            ),
        )
        whole = PlanarRegion.from_dict(report["region"])
        return Remnant(
            region=whole,
            pieces=tuple(
                PlanarRegion(f"{whole.name}-{k + 1}", (component,), whole.boundary_error_mm, whole.source)
                for k, component in enumerate(whole.components)
            ),
            area_mm2=report["area_mm2"],
            sheet_area_mm2=report["sheet_area_mm2"],
            cut_reach_mm=report["cut_reach_mm"],
            remnant_margin_mm=report["remnant_margin_mm"],
            check=report["check"],
            report=report,
        )

    def verify(self, placements, region):
        """The independent check alone, for a layout from anywhere (an edited
        nest, a manual layout): returns its report, or raises ValueError
        naming the parts that are too close, overlap or leave the sheet."""
        return _request(
            "verify_nest",
            nest=self.to_dict(),
            placements=[p.to_dict() for p in placements],
            region=_region(region),
        )
