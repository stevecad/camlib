"""Automatic nesting of sheet parts for plasma and laser cutting.

A ``Nest`` places copies of parts on a sheet and returns a ``NestResult``:
the placements and the placed parts as one ``PlanarRegion``, ready to be the
``parts`` of a ``ThermalCut``. The engine plans the layout (bottom-left fill)
and proves it with an independent check; this module only declares the
inputs and reads the result. See docs/nesting.md.
"""

from dataclasses import dataclass
import json
import math

from .operations import Lead, TraceArc, TraceLine, TracePath
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
    ``filled`` says how many were placed.

    ``marks``: TracePath lines and arcs in the part's own coordinates (a part
    number in your own stroke letters, a bend line). The nest places them
    with every copy (turned, and mirrored with a part that is turned over)
    and returns them as the result's ``marks``: the ``paths`` of a
    ThermalMark, which proves each inside its part at the mark's width.

    ``no_lead``: stretches of the part's outline that must not carry a lead
    (a seen or welded edge), as TracePath lines and arcs along the drawn
    edge in the part's coordinates. They are placed the same way and
    returned as the result's ``no_lead``: the ``keep_off`` of the cut's
    ``LeadPlacement``."""

    region: object
    quantity: int
    rotations_deg: tuple
    mirror: bool = False
    priority: int = 0
    fill: bool = False
    marks: tuple = ()
    no_lead: tuple = ()

    def to_dict(self):
        return dict(
            region=_region(self.region),
            quantity=self.quantity,
            rotations_deg=list(self.rotations_deg),
            mirror=self.mirror,
            priority=self.priority,
            fill=self.fill,
            marks=[mark.to_dict() if hasattr(mark, "to_dict") else mark for mark in self.marks],
            no_lead=[path.to_dict() if hasattr(path, "to_dict") else path for path in self.no_lead],
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
    # The parts' marks as placed (TracePath, in placement order): the
    # ``paths`` of a ThermalMark. ``mark_placements`` gives for each the
    # (placement, mark of its part) it came from.
    marks: tuple = ()
    mark_placements: tuple = ()
    # The parts' edges kept free of leads, as placed (TracePath): the
    # ``keep_off`` of the cut's LeadPlacement.
    no_lead: tuple = ()


def _trace_path(path):
    return TracePath(
        tuple(path["start_mm"]),
        tuple(
            TraceLine(tuple(segment["end_mm"]))
            if segment["kind"] == "line"
            else TraceArc(tuple(segment["center_mm"]), math.degrees(segment["sweep_rad"]))
            for segment in path["segments"]
        ),
    )


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
        marks=tuple(_trace_path(m["path"]) for m in report.get("marks", ())),
        mark_placements=tuple((m["placement"], m["mark"]) for m in report.get("marks", ())),
        no_lead=tuple(_trace_path(path) for path in report.get("no_lead", ())),
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
    # The cut's CornerLoops: room is kept for the loops as for the leads.
    corner_loops: object | None = None
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
            corner_loops=cut.corner_loops,
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
        if self.corner_loops is not None:
            result["corner_loops"] = self.corner_loops.to_dict()
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

    def remnant(self, parts, *, remnant_margin_mm, crop=None):
        """What is left of the sheet after ``parts`` (a NestResult, or the
        ``PlanarRegion`` a ThermalCut cut) are cut from it with this nest's
        kerf and leads: a ``Remnant`` whose ``region`` is the sheet less every
        part grown by all its cut can reach (kerf, leads, pierce craters) and
        by ``remnant_margin_mm`` (no default: the edge you give up for heat,
        dross and how exactly the sheet is put back). It is in the sheet's own
        coordinates, so it is the ``sheet`` of the next Nest when the remnant
        is put back against the same reference; ``camlib.dxf.write_regions``
        saves it as a DXF. ``crop``: the cut's CropCut; its kerf is removed
        too, so the offcut it frees is one of the remnant's ``pieces``."""
        region = parts.region if isinstance(parts, NestResult) else parts
        fields = self.to_dict()
        report = _request(
            "remnant",
            remnant=dict(
                sheet=fields["sheet"],
                parts=_region(region),
                remnant_margin_mm=remnant_margin_mm,
                **({} if crop is None else dict(crop=crop.to_dict())),
                **{
                    key: fields[key]
                    for key in (
                        "kerf_mm",
                        "lead_in",
                        "lead_out",
                        "pierce_clearance_mm",
                        "corner_loops",
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


@dataclass(frozen=True)
class Consumable:
    """A wearing part of the torch or head (electrode, nozzle, shield, lens)
    and the life you declare for it: a number of pierces and/or a cut length.
    camlib has no figures of its own; take them from the maker's data for
    your current and material, or from your own records."""

    name: str
    life_pierces: float | None = None
    life_cut_mm: float | None = None

    def __post_init__(self):
        lives = (self.life_pierces, self.life_cut_mm)
        if not isinstance(self.name, str) or not self.name:
            raise ValueError("a consumable needs a name")
        if all(life is None for life in lives):
            raise ValueError(f"consumable {self.name!r}: declare life_pierces and/or life_cut_mm")
        for life in lives:
            if life is not None and not (isinstance(life, (int, float)) and math.isfinite(life) and life > 0):
                raise ValueError(f"consumable {self.name!r}: a life must be a positive number")


@dataclass(frozen=True)
class SheetReport:
    """What cutting one sheet takes (``sheet_report``). Lengths are those the
    operations planned. Times are nominal: every fed move at its programmed
    feed, the pierce delays, and the rapids at the declared rapid rate;
    acceleration, height moves and what the controller does on its own
    (height sensing, corner slow-down) are not in them. ``rapid_seconds`` and
    ``seconds`` are None without a rapid rate. ``utilisation`` and the areas
    are None without a nest, ``remnant_area_mm2`` without a remnant.
    ``consumables`` holds, per declared Consumable, the share of each
    declared life this sheet uses."""

    operations: tuple
    parts: int
    pierces: int
    cut_length_mm: float
    mark_starts: int
    mark_length_mm: float
    rapid_length_mm: float
    feed_seconds: float
    dwell_seconds: float
    rapid_seconds: float | None
    seconds: float | None
    part_area_mm2: float | None
    sheet_area_mm2: float | None
    utilisation: float | None
    remnant_area_mm2: float | None
    consumables: tuple

    def to_dict(self):
        return {name: getattr(self, name) for name in self.__dataclass_fields__}


def sheet_report(reports, *, nest=None, remnant=None, rapid_mm_min=None, consumables=()):
    """The cut report of one sheet from the reports of its operations.

    ``reports``: the report of every ThermalCut and ThermalMark run on the
    sheet (``OperationUpdate.report`` of each operation's last update), in any
    order; anything else is refused. ``nest`` (a NestResult) adds the part
    and sheet areas and the utilisation, ``remnant`` (a Remnant) its area.
    ``rapid_mm_min`` is the table's rapid rate (as declared on its Machine);
    without it no rapid or total time is given. ``consumables``: Consumable
    entries; each gets the share of its declared lives the sheet uses, by
    pierces and by cut length separately (camlib does not combine them)."""
    reports = tuple(reports)
    if not reports:
        raise ValueError("sheet_report needs the report of at least one operation")
    cuts, marks = [], []
    for report in reports:
        strategy = report.get("strategy") if isinstance(report, dict) else None
        if strategy == "thermal_cut":
            cuts.append(report)
        elif strategy == "thermal_mark":
            marks.append(report)
        else:
            raise ValueError(f"sheet_report takes ThermalCut and ThermalMark reports, not {strategy!r}")
    if rapid_mm_min is not None and not (
        isinstance(rapid_mm_min, (int, float)) and math.isfinite(rapid_mm_min) and rapid_mm_min > 0
    ):
        raise ValueError("rapid_mm_min must be a positive number")
    if nest is not None and not isinstance(nest, NestResult):
        raise ValueError("nest must be a NestResult")
    if remnant is not None and not isinstance(remnant, Remnant):
        raise ValueError("remnant must be a Remnant")
    pierces = sum(r["pierces"] for r in cuts)
    cut_length = math.fsum(r["cut_length_mm"] for r in cuts)
    rapid_length = math.fsum(r["rapid_length_mm"] for r in reports)
    feed_seconds = math.fsum(r["feed_seconds"] for r in reports)
    dwell_seconds = math.fsum(r["dwell_seconds"] for r in cuts)
    rapid_seconds = None if rapid_mm_min is None else rapid_length / rapid_mm_min * 60
    used = []
    for consumable in consumables:
        if not isinstance(consumable, Consumable):
            raise ValueError("consumables must be Consumable entries")
        used.append(
            dict(
                name=consumable.name,
                pierces=None if consumable.life_pierces is None else pierces / consumable.life_pierces,
                cut_length=None if consumable.life_cut_mm is None else cut_length / consumable.life_cut_mm,
            )
        )
    return SheetReport(
        operations=tuple(r["strategy"] for r in reports),
        parts=sum(r["parts"] for r in cuts),
        pierces=pierces,
        cut_length_mm=cut_length,
        mark_starts=sum(r["paths"] for r in marks),
        mark_length_mm=math.fsum(r["mark_length_mm"] for r in marks),
        rapid_length_mm=rapid_length,
        feed_seconds=feed_seconds,
        dwell_seconds=dwell_seconds,
        rapid_seconds=rapid_seconds,
        seconds=None if rapid_seconds is None else feed_seconds + dwell_seconds + rapid_seconds,
        part_area_mm2=None if nest is None else nest.placed_area_mm2,
        sheet_area_mm2=None if nest is None else nest.sheet_area_mm2,
        utilisation=None if nest is None else nest.utilisation,
        remnant_area_mm2=None if remnant is None else remnant.area_mm2,
        consumables=tuple(used),
    )
