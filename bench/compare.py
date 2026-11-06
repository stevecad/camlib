"""Compare two benchmark results: what changed in the toolpaths, and how fast.

    python bench/compare.py BASELINE.json NEW.json [--toolpath-only] [--markdown]

Toolpath data (moves, lengths, removal, cycle time, blocks, material) is
deterministic and independent of the host, so any difference is a real
change in the toolpath. Engine times only compare on the same host; they are
shown as ratios (new / baseline) when both runs are from the same CPU.

Exit status 1 when the toolpath data differs (CI uses this against the
committed baseline: a PR that changes toolpaths updates the baseline with it).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

OPERATION_FIELDS = (
    "moves",
    "cut_mm",
    "air_cut_mm",
    "rapid_mm",
    "plunges",
    "retracts",
    "removed_mm3",
    "cycle_seconds_exact_stop",
    "cycle_seconds_tangent",
)
WORKLOAD_FIELDS = ("moves", "cycle_seconds_exact_stop", "cycle_seconds_tangent", "posted_blocks", "post_error")


def _material(t):
    m = t.get("material") or {}
    return {k: m.get(k) for k in ("leftover_mm3", "overcut_mm3")}


def toolpath_changes(base, new):
    """(workload, operation or None, field, baseline value, new value)."""
    changes = []
    old = {w["name"]: w for w in base["workloads"]}
    for w in new["workloads"]:
        b = old.get(w["name"])
        if b is None:
            changes.append((w["name"], None, "workload", None, "added"))
            continue
        for field in WORKLOAD_FIELDS:
            if b["toolpath"].get(field) != w["toolpath"].get(field):
                changes.append((w["name"], None, field, b["toolpath"].get(field), w["toolpath"].get(field)))
        for field, value in _material(w["toolpath"]).items():
            if _material(b["toolpath"])[field] != value:
                changes.append((w["name"], None, field, _material(b["toolpath"])[field], value))
        for op_id, o in w["operations"].items():
            p = b["operations"].get(op_id)
            if p is None:
                changes.append((w["name"], op_id, "operation", None, "added"))
                continue
            for field in OPERATION_FIELDS:
                if p[field] != o[field]:
                    changes.append((w["name"], op_id, field, p[field], o[field]))
    for name in sorted(set(old) - {w["name"] for w in new["workloads"]}):
        changes.append((name, None, "workload", "present", "removed"))
    return changes


def engine_ratios(base, new):
    old = {w["name"]: w for w in base["workloads"]}
    rows = []
    for w in new["workloads"]:
        b = old.get(w["name"])
        if b is None:
            continue
        for op_id, o in w["operations"].items():
            p = b["operations"].get(op_id)
            if p and p["engine_seconds"] > 0:
                rows.append((w["name"], op_id, p["engine_seconds"], o["engine_seconds"]))
        rows.append((w["name"], "(run)", b["engine"]["run_seconds"], w["engine"]["run_seconds"]))
    return rows


def _fmt(v):
    if isinstance(v, float):
        return f"{v:.3f}"
    if isinstance(v, list):
        return "[" + ", ".join(_fmt(x) for x in v) + "]"
    return str(v)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("new", type=Path)
    parser.add_argument("--toolpath-only", action="store_true")
    args = parser.parse_args()
    base, new = json.loads(args.baseline.read_text()), json.loads(args.new.read_text())
    changes = toolpath_changes(base, new)
    print(f"# Toolpath changes ({args.baseline.name} -> {args.new.name})\n")
    if changes:
        print("| Workload | Operation | Field | Baseline | New |\n|---|---|---|---:|---:|")
        for name, op, field, a, b in changes:
            print(f"| `{name}` | {op or '–'} | {field} | {_fmt(a)} | {_fmt(b)} |")
    else:
        print("None: every toolpath is identical to the baseline.")
    if not args.toolpath_only:
        same_host = base["environment"].get("cpu") == new["environment"].get("cpu")
        print("\n# Engine time (new / baseline)\n")
        if not same_host:
            print(
                f"Different CPUs ({base['environment'].get('cpu')} vs {new['environment'].get('cpu')}): "
                "times are not comparable; compare on the same machine."
            )
        else:
            print("| Workload | Operation | Baseline s | New s | Ratio |\n|---|---|---:|---:|---:|")
            for name, op, a, b in engine_ratios(base, new):
                print(f"| `{name}` | {op} | {a:.4f} | {b:.4f} | {b / a:.2f}× |")
    return 1 if changes else 0


if __name__ == "__main__":
    sys.exit(main())
