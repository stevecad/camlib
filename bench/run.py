"""Run the toolpath benchmarks and record toolpath quality and engine performance.

    python bench/run.py --label <host> [--only NAME ...] [--repeat 3]

Each workload runs in its own process (isolated memory measurement), its
timed runs sequentially. Results go to bench/results/<label>.json; render the
published table with bench/report.py.

Toolpath data (deterministic for the same inputs):
  cut / rapid / air-cut length, moves, plunges and retracts per operation;
  cycle time from camlib.dynamics (exact-stop and tangent junctions, the
  benchmark machine's acceleration); posted G-code blocks; material removed;
  leftover material and over-cut against the workload's target.
Engine performance (wall clock, median of --repeat runs):
  total run time (planning plus streamed simulation), time to the first move,
  each operation's time from its begin to its end in the stream, posting time,
  and the process's peak memory.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
import os
import platform
import statistics
import subprocess
import sys
import time
from pathlib import Path

if sys.platform != "win32":
    import resource

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from geometry import reset as reset_geometry  # noqa: E402
from workloads import BY_NAME, ROOT, WORKLOADS  # noqa: E402

ACCEL_MM_S2 = 1000.0
RAPID_MM_MIN = 6000.0


def _machine(family, program):
    """The benchmark machine: declared, never assumed (camlib has no defaults)."""
    from camlib.dynamics import MotionPolicy
    from camlib.motion import ToolChange
    from camlib.post import GCodeDialect, Machine, ToolSetting

    numbers = {}
    for e in program.events:
        if isinstance(e, ToolChange):
            numbers.setdefault(e.tool, (e.number, e.length_register))
    tools = tuple(
        ToolSetting.for_tool(
            t,
            number=numbers.get(t.id, (i, i))[0],
            length_register=numbers.get(t.id, (i, i))[1],
            position_after_mm=program.initial_position_mm,
        )
        for i, t in enumerate(program.tools, 1)
    )
    if family == "lathe":
        # LinuxCNC lathe: radius-mode X, spindle-synchronized threading.
        dialect = GCodeDialect().with_commands(lathe_radius_mode="G8", thread_synchronized="G33")
        planes = ("xz",)
    elif family == "thermal":
        dialect = GCodeDialect(beam_s_max=1, height_control_word="P2").with_commands(
            height_control_on="M63", height_control_off="M62"
        )
        planes = ("xy", "xz", "yz")
    else:
        dialect, planes = GCodeDialect.linuxcnc(), ("xy", "xz", "yz")
    return Machine(
        f"benchmark {family}",
        tools,
        dialect=dialect,
        planes=planes,
        tool_change_position_mode="preserve_tip",
        output_units="mm",
        decimals=3,
        max_feed_mm_min=10000,
        max_spindle_rpm=30000,
        arcs=True,
        helices=True,
        max_sweep_rad=6.283185307179586,
        max_tool_number=99999,
        max_length_register=99999,
        tool_changes=True,
        flood=True,
        mist=True,
        rapid_mm_min=RAPID_MM_MIN,
        motion_policy=MotionPolicy(tangential_accel_mm_s2=ACCEL_MM_S2, rapid_feed_mm_min=RAPID_MM_MIN),
    )


def _material(workload, state):
    """Leftover material and over-cut against the target, from the engine's own
    material verifier (a lathe run's own material check for turning)."""
    if workload.family == "lathe":
        report = state.material()
        return dict(
            leftover_mm3=report["residual"]["volume_mm3"],
            overcut_mm3=report["gouge"]["volume_mm3"],
            leftover_cells=[report["residual"]["definite_cells"], report["residual"]["possible_only_cells"]],
        )
    if workload.target is None:
        return None
    from camlib.cadquery import export_surface_mesh
    from camlib.material import SurfaceTarget, Target, Verifier

    reset_geometry()
    kind, *rest = workload.target()
    if kind == "solid":
        surface = export_surface_mesh(rest[0], source=dict(model=workload.name, revision="bench"))
        # A tessellated CAD solid: the target lies within the mesh's stated error.
        target = SurfaceTarget(workload.name, "bench", surface, state.min_mm[2], source_enclosure="offset")
    elif kind == "surface":
        target = SurfaceTarget(workload.name, "bench", rest[0], rest[1], source_enclosure="offset")
    else:
        target = Target(workload.name, "bench", rest[0], rest[1])
    report = Verifier(target).check(state)
    heights = [f["residual_height_mm"][1] for f in report.get("findings", ())]
    return dict(
        leftover_mm3=report["residual"]["volume_mm3"],
        overcut_mm3=report["gouge"]["volume_mm3"],
        leftover_cells=[report["residual"]["definite_cells"], report["residual"]["possible_only_cells"]],
        overcut_cells=[report["gouge"]["definite_cells"], report["gouge"]["possible_only_cells"]],
        max_residual_height_mm=max(heights) if heights else 0.0,
        complete=report["complete"],
    )


def _run_once(workload):
    """One streamed run: toolpath data, per-operation wall times, final state."""
    from camlib.motion import Program
    from camlib.postprocessor import _typed
    from camlib.simulation import OperationUpdate, Simulation, Update, curve_length
    from camlib.tools import ToolAssembly

    reset_geometry()
    config = workload.build()
    ops = defaultdict(lambda: defaultdict(float))
    events, op_of = [], {}
    begin_at, removed_at = {}, {}
    start = time.perf_counter()
    first_motion = None
    with Simulation(**config) as sim:
        manifest = sim.state.manifest
        for u in sim.run_operations():
            now = time.perf_counter() - start
            if isinstance(u, OperationUpdate):
                events.extend(_typed(e) for e in u.events)
                if u.stage == "begin":
                    begin_at[u.operation] = now
                    removed_at[u.operation] = sim.state.volume()["removed_lower_mm3"]
                elif u.stage == "end":
                    o = ops[u.operation]
                    o["engine_seconds"] = now - begin_at[u.operation]
                    o["removed_mm3"] = sim.state.volume()["removed_lower_mm3"] - removed_at[u.operation]
            elif isinstance(u, Update):
                if first_motion is None:
                    first_motion = now
                m = u.motion
                events.append(_typed(dict(kind="motion", motion=m)))
                op_of[m["id"]] = m["operation"]
                length = curve_length(m["curve"])
                o = ops[m["operation"]]
                o["moves"] += 1
                if m["kind"] == "rapid":
                    o["rapid_mm"] += length
                else:
                    o["cut_mm"] += length
                    if not u.changed_count:
                        o["air_cut_mm"] += length
                o["plunges"] += m["kind"] == "plunge"
                o["retracts"] += m["kind"] in ("retract", "rapid") and m["curve"]["end"][2] > m["curve"]["start"][2]
        total = time.perf_counter() - start
        state = sim.state.clone()
        volume = sim.state.volume()
    tools = tuple(ToolAssembly.from_dict(t) for t in manifest["tools"])
    program = Program(workload.name, "bench", tools, tuple(manifest["initial_position_mm"]), tuple(events))
    return dict(total=total, first=first_motion, ops=ops, op_of=op_of, state=state, volume=volume, program=program)


def measure(name, repeat):
    workload = BY_NAME[name]
    runs = [_run_once(workload) for _ in range(repeat)]
    run = runs[-1]
    program, machine = run["program"], _machine(workload.family, run["program"])

    from dataclasses import replace

    from camlib.dynamics import schedule
    from camlib.post import LoweringOptions, post_program

    timing = {}
    for junction in ("exact_stop", "tangent"):
        m = replace(machine, motion_policy=replace(machine.motion_policy, junction_mode=junction))
        report = schedule(program, m, adjust_feeds=False).report
        per_op = defaultdict(float)
        for s in report["segments"]:
            per_op[run["op_of"].get(s["motion_id"], "?")] += s["seconds"]
        timing[junction] = dict(total=report["motion_seconds"], per_op=per_op)
    post_seconds, blocks, post_error = [], None, None
    for _ in range(repeat):
        t = time.perf_counter()
        try:
            # Arcs not exactly writable at the machine's decimals become
            # checked segments (the real output of a 3-decimal controller).
            text = "".join(
                c.text for c in post_program(program, machine=machine, lowering=LoweringOptions(native_arcs=True))
            )
        except ValueError as error:
            # A program the posting engine refuses is a result, recorded.
            post_error = str(error)
            break
        post_seconds.append(time.perf_counter() - t)
        blocks = text.count("\n")

    unknown = set(run["ops"]) - set(workload.toolpaths)
    if unknown:
        raise SystemExit(f"{name}: operations {sorted(unknown)} are not mapped to toolpaths")
    operations = {}
    for op_id, (toolpath, issue) in workload.toolpaths.items():
        o = run["ops"].get(op_id)
        if o is None:
            raise SystemExit(f"{name}: operation {op_id!r} did not run")
        operations[op_id] = dict(
            toolpath=toolpath,
            issue=issue,
            moves=int(o["moves"]),
            cut_mm=round(o["cut_mm"], 3),
            rapid_mm=round(o["rapid_mm"], 3),
            air_cut_mm=round(o["air_cut_mm"], 3),
            plunges=int(o["plunges"]),
            retracts=int(o["retracts"]),
            removed_mm3=round(o["removed_mm3"], 3),
            cycle_seconds_exact_stop=round(timing["exact_stop"]["per_op"].get(op_id, 0.0), 3),
            cycle_seconds_tangent=round(timing["tangent"]["per_op"].get(op_id, 0.0), 3),
            engine_seconds=round(statistics.median(r["ops"][op_id]["engine_seconds"] for r in runs), 4),
        )
    return dict(
        name=name,
        family=workload.family,
        description=workload.description,
        operations=operations,
        toolpath=dict(
            moves=sum(o["moves"] for o in operations.values()),
            cycle_seconds_exact_stop=round(timing["exact_stop"]["total"], 3),
            cycle_seconds_tangent=round(timing["tangent"]["total"], 3),
            posted_blocks=blocks,
            post_error=post_error,
            removed_mm3=[round(v, 3) for v in (run["volume"]["removed_lower_mm3"], run["volume"]["removed_upper_mm3"])],
            material=_material(workload, run["state"]),
        ),
        engine=dict(
            run_seconds=round(statistics.median(r["total"] for r in runs), 4),
            first_move_seconds=round(statistics.median(r["first"] for r in runs if r["first"] is not None), 4),
            post_seconds=round(statistics.median(post_seconds), 4) if post_seconds else None,
            repeats=repeat,
        ),
    )


def _child(name, repeat):
    result = measure(name, repeat)
    # Peak memory of this process and the native engine processes it ran.
    # Windows has no getrusage; report unavailable measurements explicitly.
    own = engines = None
    if sys.platform != "win32":
        scale = 1024 if sys.platform == "darwin" else 1
        own = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / scale
        engines = resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss / scale
    result["engine"]["peak_rss_kib"] = dict(python=own, native=engines)
    print(json.dumps(result))


def _environment():
    def run(*cmd):
        try:
            return subprocess.run(cmd, capture_output=True, text=True, cwd=ROOT).stdout.strip()
        except OSError:
            return None

    cpu = None
    try:
        for line in Path("/proc/cpuinfo").read_text().splitlines():
            if line.startswith("model name"):
                cpu = line.split(":", 1)[1].strip()
                break
    except OSError:
        pass
    try:
        import cadquery
        import OCP

        cad = f"CadQuery {cadquery.__version__}, OCP {getattr(OCP, '__version__', '?')} (geometry from fixtures)"
    except ImportError:
        cad = None
    return dict(
        cad=cad,
        commit=run("git", "rev-parse", "HEAD"),
        # Uncommitted changes to the engine itself (docs and benchmark files do
        # not change what is measured).
        dirty=bool(
            run(
                "git",
                "status",
                "--porcelain",
                "--untracked-files=no",
                "--",
                "src",
                "bindings",
                "python/camlib",
                "Cargo.toml",
                "Cargo.lock",
            )
        ),
        rustc=run("rustc", "--version"),
        python=platform.python_version(),
        os=platform.platform(),
        cpu=cpu,
        cpus=os.cpu_count(),
        machine=dict(accel_mm_s2=ACCEL_MM_S2, rapid_mm_min=RAPID_MM_MIN),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--label", default="local", help="results file name (default: local)")
    parser.add_argument("--only", nargs="*")
    parser.add_argument("--repeat", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=1800)
    parser.add_argument("--child")
    args = parser.parse_args()
    if args.child:
        return _child(args.child, args.repeat)
    names = args.only or [w.name for w in WORKLOADS]
    results, failures = [], {}
    for name in names:
        if name not in BY_NAME:
            raise SystemExit(f"unknown workload {name!r}")
        print(f"{name} ...", file=sys.stderr, flush=True)
        proc = subprocess.run(
            [sys.executable, __file__, "--child", name, "--repeat", str(args.repeat)],
            capture_output=True,
            text=True,
            timeout=args.timeout,
        )
        if proc.returncode:
            failures[name] = proc.stderr.strip().splitlines()[-1:] or ["failed"]
            print(f"  FAILED: {failures[name]}", file=sys.stderr)
            continue
        results.append(json.loads(proc.stdout.strip().splitlines()[-1]))
    out = HERE / "results" / f"{args.label}.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(
        json.dumps(dict(label=args.label, environment=_environment(), workloads=results, failures=failures), indent=1)
        + "\n"
    )
    print(f"wrote {out} ({len(results)} workloads, {len(failures)} failed)", file=sys.stderr)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
