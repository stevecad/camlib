"""Export acceptance: a machine-ready program is released only when every
non-clear verification result has been accepted on purpose.

Posting already refuses output that does not match the planned motion. The
stream's geometric checks (scene bodies such as fixtures, stock contact,
material) report per move; this gate turns their violations and uncertain
results into a refusal unless the caller accepts that kind explicitly.
"""

from __future__ import annotations

ACCEPTABLE = frozenset(("uncertain", "violation"))


class ExportRefused(ValueError):
    """The program has unaccepted verification findings; nothing was written.

    ``findings``: the non-clear results (each with its move, status and
    detail). ``statuses``: the result kinds that were not accepted."""

    def __init__(self, findings, statuses):
        self.findings = tuple(findings)
        self.statuses = tuple(sorted(statuses))
        first = self.findings[0] if self.findings else {}
        super().__init__(
            f"export refused: {len(self.findings)} move(s) with unaccepted "
            f"{'/'.join(self.statuses)} verification results (first at move {first.get('completed')}: "
            f"{describe(first)}); fix the job, or pass "
            f"accept={{{', '.join(repr(s) for s in self.statuses)}}} to release it anyway"
        )


def describe(result):
    """One line naming what a gated result found.

    A gated result's ``findings`` hold either motion findings (a scene or
    stock contact: its kind and the object it touches) or one material
    report (the stock compared with the part: its target and the gouge
    assessment, and the residual one when material is left)."""
    detail = next(iter(result.get("findings") or ()), None)
    if detail is None:
        return f"{result.get('status', 'unclear')} result without a localized finding"
    if "gouge" in detail and "residual" in detail:
        return _material(detail)
    detail = detail.get("finding", detail)
    return f"{detail.get('kind', 'finding')} {detail.get('object_id', '(no object)')}"


def _material(report):
    # A lathe report has no target identity: its target is the turned part.
    target = f"target {report['target_id']}" if report.get("target_id") else "the turned part"
    gouge, residual = report["gouge"], report["residual"]
    text = f"material gouge {gouge['assessment']} on {target} ({_cells(gouge)})"
    if residual["assessment"] != "within_tolerance":
        text += f", residual {residual['assessment']} ({_cells(residual)})"
    if not report["complete"]:
        text += ", analysis incomplete"
    proof = report.get("sweep_proof")
    if proof is not None and proof["decided_cells"] != gouge["possible_only_cells"]:
        if proof.get("unavailable"):
            text += f"; per-move proof unavailable: {proof['unavailable']}"
        else:
            text += (
                f"; per-move proof: {proof['unproven_moves']} of {proof['proven_moves'] + proof['unproven_moves']} "
                f"moves not proven within {proof['tolerance_mm']} mm (a larger comparison_tolerance_mm can decide "
                "cells along walls the tool runs tangent to)"
            )
    return text


def _cells(summary):
    return f"definite {summary['definite_cells']} cells, possible-only {summary['possible_only_cells']} cells"


def accepted(accept):
    """Validate an ``accept`` argument: a set drawn from uncertain/violation."""
    kinds = frozenset(accept or ())
    if not kinds <= ACCEPTABLE:
        raise ValueError(f"accept takes {sorted(ACCEPTABLE)}, not {sorted(kinds - ACCEPTABLE)}")
    return kinds


def gate(findings, accept):
    """Raise ExportRefused when any finding's status is not accepted."""
    kinds = accepted(accept)
    refused = [f for f in findings if f["status"] not in kinds]
    if refused:
        raise ExportRefused(refused, {f["status"] for f in refused})


def material_status(report, sweep_proof=None):
    """A material report's export status: a definite gouge is a violation; a
    possible gouge or an incomplete analysis is uncertain. Leftover material
    is reported, not refused (a roughing setup leaves its allowance).

    ``sweep_proof``: the run's per-move gouge proof recorded beside the
    report (issue #72). Possible-only gouge cells it decides (every motion
    that touches them proven not to cut deeper than the tolerance into the
    part) are within tolerance; the gouge is clear when it decides all of
    them."""
    gouge = report["gouge"]
    if gouge["definite_cells"]:
        return "violation"
    if not report["complete"]:
        return "uncertain"
    possible = gouge["possible_only_cells"]
    if possible and (sweep_proof is None or sweep_proof["decided_cells"] != possible):
        return "uncertain"
    return "clear"


def unchecked(verification, *, turning=False):
    """The geometric checks this setup did not configure (so not run).

    A lathe setup (``turning``) runs all of its checks always: every move is
    checked as it is planned (and a posted program as it is replayed), the
    insert against the part, bore and chuck, the holder and rapids against the
    stock, and the stock is compared with the part (material)."""
    if turning:
        return ()
    config = verification or {}
    missing = []
    if not config.get("scene"):
        missing.append("scene")
    if not config.get("stock_contact"):
        missing.append("stock_contact")
    if not config.get("material"):
        missing.append("material")
    return tuple(missing)
