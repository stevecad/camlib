"""Render bench/results/<label>.json as docs/benchmarks.md.

python bench/report.py bench/results/<label>.json [--out docs/benchmarks.md]
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
ISSUES = "https://github.com/stevecad/camlib/issues"


def _bounds(pair, digits=1):
    if pair is None:
        return "–"
    low, high = pair
    return f"{low:.{digits}f} / {high:.{digits}f}"


def _pct(part, whole):
    return f"{100 * part / whole:.0f}%" if whole else "–"


def render(data):
    env = data["environment"]
    lines = [
        "# Toolpath benchmarks",
        "",
        "Measured toolpath quality and engine performance for every camlib toolpath, so",
        "a change can be shown to be better or worse, and why. Each toolpath's",
        "optimization issue links here.",
        "",
        "## How these numbers are made",
        "",
        "- **Workloads** are the regression suite's synthetic parts (`bench/workloads.py`).",
        "  Their CAD-derived meshes and regions are frozen in `bench/fixtures/`, so every",
        "  machine plans from identical geometry and the toolpath data is reproducible",
        "  exactly. `python bench/compare.py BASELINE.json NEW.json` lists every toolpath",
        "  change and, on the same machine, engine-time ratios.",
        "  Every number describes a toolpath whose correctness is tested. Each maps",
        "  its operations to toolpaths and issues. `dense` workloads use the same parts",
        "  at production density (finer stepover or stepdown, or a large stock) to make",
        "  planning time meaningful.",
        "- **Toolpath data** is deterministic for the same inputs:",
        "  - *Cut / rapid mm*: path length of feed moves and of rapids.",
        "  - *Air cut*: feed moves that removed no stock in the simulation (cutting air).",
        "    Every feed move counts, so this includes the stay-down links of adaptive",
        "    clearing: moves at depth that take the cutter to its next pass through",
        "    stock already cut. Those are repositioning, not a pass that missed;",
        "    results that record engagement (`bench/engagement.py`) show them",
        "    separately in an engagement table.",
        "  - *Removed*: certain material removed by the operation (simulated stock).",
        "    Tapping and single-point threading show none (and their moves count as air",
        "    cutting): the stock model conservatively keeps thread flanks, the tap being",
        "    modeled by its drill diameter and a thread keeping its crests.",
        "  - *Cycle time*: `camlib.dynamics` on a declared benchmark machine",
        f"    ({env['machine']['accel_mm_s2']:g} mm/s² tangential acceleration, rapids "
        f"{env['machine']['rapid_mm_min']:g} mm/min).",
        "    *Exact stop* stops at every block; *tangent* carries speed through tangent",
        "    moves. Neither models controller blending or jerk.",
        "  - *Blocks*: lines of G-code posted for a LinuxCNC-style controller at 3",
        "    decimals (arcs not exactly writable there become checked segments).",
        "  - *Leftover / over-cut*: material left above, and removed below, the",
        "    workload's target part, from camlib's material verifier, as certain /",
        "    possible bounds in mm³. Certain values are proven; possible values include",
        "    the stock grid's and the target mesh's resolution. A workload that",
        "    deliberately leaves material (allowances, a partial setup) shows it as",
        "    leftover. Compare these between runs, not against zero.",
        f"- **Engine performance** is wall-clock time, the median of {_repeats(data)} runs each, run sequentially:",
        "  - *Engine s* (per operation): from the operation's begin to its end in the",
        "    stream, planning and streamed simulation together.",
        "  - *Run s*: the whole run.",
        "  - *First move s*: time until the first move arrived.",
        "  - *Post s*: posting the whole program with independent decoding and checks.",
        "  - *Peak memory*: the native engine's peak resident memory.",
        "",
        "Reproduce: `python bench/run.py --label <name>` (needs CadQuery for the target",
        "parts), then `python bench/report.py bench/results/<name>.json`. Results are",
        "comparable only on the same machine; CI runs the same benchmark on every push.",
        "",
        "## Environment",
        "",
        f"- Commit `{(env['commit'] or '?')[:12]}`" + (" (with uncommitted changes)" if env.get("dirty") else ""),
        f"- {env['cpu']} ({env['cpus']} threads), {env['os']}",
        f"- {env['rustc']}, Python {env['python']}",
        "",
        "## Per toolpath",
        "",
        "| Toolpath | Issue | Workload | Moves | Cut mm | Air cut | Rapid mm | Plunges | Retracts | Removed mm³ |"
        " Cycle s (exact / tangent) | Engine s |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    rows = []
    for w in data["workloads"]:
        for op_id, o in w["operations"].items():
            rows.append((o["issue"], o["toolpath"], w["name"], op_id, o))
    for issue, toolpath, name, op_id, o in sorted(rows, key=lambda r: (r[0], r[2], r[3])):
        lines.append(
            f"| {toolpath} | [#{issue}]({ISSUES}/{issue}) | `{name}` ({op_id}) | {o['moves']} | {o['cut_mm']:.0f} |"
            f" {o['air_cut_mm']:.0f} ({_pct(o['air_cut_mm'], o['cut_mm'])}) | {o['rapid_mm']:.0f} | {o['plunges']} |"
            f" {o['retracts']} | {o['removed_mm3']:.0f} | {o['cycle_seconds_exact_stop']:.1f} /"
            f" {o['cycle_seconds_tangent']:.1f} | {o['engine_seconds']:.3f} |"
        )
    lines += _engagement(data)
    lines += [
        "",
        "## Per workload",
        "",
        "| Workload | What | Moves | Cycle s (exact / tangent) | Blocks | Leftover mm³ | Over-cut mm³ |"
        " Run s | First move s | Post s | Peak memory MiB |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for w in data["workloads"]:
        t, e = w["toolpath"], w["engine"]
        m = t["material"] or {}
        blocks = t["posted_blocks"] if t["posted_blocks"] is not None else "refused ¹"
        post = f"{e['post_seconds']:.3f}" if e["post_seconds"] is not None else "–"
        native_rss = e['peak_rss_kib']['native']
        memory = f"{native_rss / 1024:.0f}" if native_rss is not None else "unavailable"
        lines.append(
            f"| `{w['name']}` | {w['description']} | {t['moves']} | {t['cycle_seconds_exact_stop']:.1f} /"
            f" {t['cycle_seconds_tangent']:.1f} | {blocks} | {_bounds(m.get('leftover_mm3'))} |"
            f" {_bounds(m.get('overcut_mm3'))} | {e['run_seconds']:.3f} | {e['first_move_seconds']:.3f} | {post} |"
            f" {memory} |"
        )
    refused = [w for w in data["workloads"] if w["toolpath"]["post_error"]]
    if refused:
        lines += ["", "¹ The posting engine refused these programs at 3 decimals:", ""]
        for w in refused:
            lines.append(f"- `{w['name']}`: {w['toolpath']['post_error'].split('.')[0]}.")
    if data.get("failures"):
        lines += ["", "## Failed workloads", ""]
        for name, message in data["failures"].items():
            lines.append(f"- `{name}`: {' '.join(message)}")
    return "\n".join(lines) + "\n"


def _share(value):
    return "–" if value is None else f"{100 * value:.1f}%"


def _engagement(data):
    """The contact-angle table, for results that record it (bench/engagement.py)."""
    rows = []
    for w in data["workloads"]:
        for op_id, o in w["operations"].items():
            if o.get("engagement"):
                rows.append((o["issue"], o["toolpath"], w["name"], op_id, o["engagement"]))
    if not rows:
        return []
    method = data.get("engagement_method", {})
    sample, over, step = (method.get(k, float("nan")) for k in ("sample_deg", "over_deg", "step_per_radius"))
    lines = [
        "",
        "## Engagement of adaptive clearing",
        "",
        "Adaptive clearing holds the cutter's *contact angle*: the part of its rim that is",
        "in material. Each operation declares a limit, acos(1 − w/r) for its radial width",
        "w, and proves every cutting stroke within it. This table shows how close to that",
        "limit the cutter actually runs, measured from the streamed moves alone",
        "(`bench/engagement.py`, which reads no plan and no planner report):",
        "",
        "- The contact angle at a point of the path is the part of the leading half of",
        "  the rim (the 180° ahead of the direction of travel) that lies in material:",
        "  inside the stock the operation started on, and not within the cutter radius of",
        "  any earlier move's centre line where that move's tip was at or below this",
        "  one's. Earlier moves are tested in closed form (lines, arcs, helices), not on",
        "  a grid; the starting stock is the simulated stock when the operation began.",
        f"- The rim's leading half is sampled every {sample:g}°, so one contact arc reads within",
        f"  {sample:g}° of its true angle, without bias. The path is evaluated at steps of {step:g} of",
        "  the cutter radius, each weighted by its length.",
        "- *Cutting mm*: horizontal moves of kind `cut` (wall passes included). The",
        f"  shares are of this length: *over* is more than {over:g}° above the limit, *within",
        f"  10°* is from 10° under the limit to {over:g}° over it, then 10–20° under, more than",
        "  20° under, and *nothing*: cutting moves whose rim touches no material.",
        "- *Links*: stay-down moves at depth between passes, and the share of their",
        "  length that touches nothing. *Entry feed*: horizontal moves made at the",
        "  entry feed because their contact is not proven within the limit, with their",
        "  mean contact angle. Helical entries are not measured (a helix cuts a full",
        "  slot).",
        "- The measure goes through the platform's sin/cos/atan2, so the last digit can",
        "  differ between platforms. `bench/compare.py` lists differences without",
        "  failing on them.",
        "",
        "| Toolpath | Issue | Workload | Limit ° | Cutting mm | Mean ° | Max ° | Over | Within 10° | 10–20° under |"
        " >20° under | Nothing | Link mm (touching nothing) | Entry feed mm (mean °) |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for issue, toolpath, name, op_id, e in sorted(rows, key=lambda r: (r[0], r[2], r[3])):
        mean = "–" if e["mean_deg"] is None else f"{e['mean_deg']:.1f}"
        peak = "–" if e["max_deg"] is None else f"{e['max_deg']:.1f}"
        entry = f"{e['entry_feed_mm']:.0f}"
        if e["entry_feed_mean_deg"] is not None:
            entry += f" ({e['entry_feed_mean_deg']:.1f})"
        lines.append(
            f"| {toolpath} | [#{issue}]({ISSUES}/{issue}) | `{name}` ({op_id}) | {e['limit_deg']:.1f} |"
            f" {e['cut_mm']:.0f} | {mean} | {peak} | {_share(e['over_share'])} | {_share(e['within_10_share'])} |"
            f" {_share(e['under_10_20_share'])} | {_share(e['under_20_share'])} | {_share(e['none_share'])} |"
            f" {e['link_mm']:.0f} ({_share(e['link_none_share'])}) | {entry} |"
        )
    return lines


def _repeats(data):
    values = {w["engine"]["repeats"] for w in data["workloads"]}
    return "/".join(str(v) for v in sorted(values)) or "?"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results", type=Path)
    parser.add_argument("--out", type=Path, default=ROOT / "docs" / "benchmarks.md")
    args = parser.parse_args()
    args.out.write_text(render(json.loads(args.results.read_text())))
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
