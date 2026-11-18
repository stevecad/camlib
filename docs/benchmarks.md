# Toolpath benchmarks

Read the [latest validated main report](https://github.com/stevecad/camlib/blob/benchmark-report/README.md)
for measured toolpaths, modeled cycle times, material bounds, planning times and memory.
Each publication identifies the measured commit, platform, toolchain and UTC measurement time.
[Report history](https://github.com/stevecad/camlib/commits/benchmark-report/) retains previous
published main reports. Each [CI run](https://github.com/stevecad/camlib/actions/workflows/ci.yml)
shows its comparison in the benchmark job summary and retains the measured JSON, report
and comparison as a downloadable artifact for seven days.

Run the release engine with CadQuery installed. The runner uses frozen geometry and measures
workloads sequentially. To reproduce the full comparison and render a local report:

```sh
python bench/run.py --label local
python bench/compare.py bench/expectations bench/results/local.json
python bench/report.py bench/results/local.json
```

The Markdown report is written beside the measured JSON. Compare wall-clock performance
only between sequential release runs on the same machine, using
`python bench/compare.py OLD.json NEW.json`. Modeled cycle times and material bounds are
reviewed toolpath expectations. Sampled engagement differences are informational.

Two measurements can change even when the number of moves and total cut length agree.
A feed move counts toward `air_cut_mm` when it changes zero stock cells. A single
cell-bound update changes that count. Tangent cycle time includes the stops selected
by the motion scheduler. To investigate a difference, compare the changed-cell count
and entry/exit speeds by motion ID on the same input.

For an intentional change, measure and update just the affected workloads:

```sh
python bench/run.py --label facing-change --only indexable-facing
python bench/expectations.py bench/results/facing-change.json indexable-facing
```

Review the changed files under `bench/expectations/` with the implementation. New workloads
use the same command. When retiring a workload, remove its expectation in the same PR.
CI compares the full suite on PRs and in the merge queue. After merge, automation publishes
the successful queue report for that main commit on the dedicated report branch.
