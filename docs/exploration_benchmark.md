# L17 bonus — frontier-greedy vs information-gain exploration

The assignment bonus asks for a **quantitative comparison of greedy frontier
exploration vs information-gain exploration, on coverage over time**
(`doc/orig/IA712… _FR.md`). This is implemented end-to-end in `bl`.

## The two strategies

Both reuse the same frontier detection + clustering (`frontier_search.py`); only
the **goal-selection rule** differs. Selected via the `strategy` parameter of
`frontier_explorer_node`, or the `IA712_EXPLORE_STRATEGY` env var (used by the
benchmark):

| strategy | rule | notes |
|---|---|---|
| `greedy` | drive to the **nearest** reachable frontier | classic `explore_lite` baseline |
| `info_gain` | argmax **`gain(f) − λ·cost(f)`** | Stachniss et al., ICRA 2005 |
| `size_dist` | best `size/(dist+1)` (legacy hybrid) | previous default |

For `info_gain` (`frontier_search.choose_frontier_infogain`):
- **`gain(f)`** = number of `unknown` cells within `info_gain_radius_m` of the
  frontier (≈ area it will reveal), `count_unknown_in_radius`.
- **`cost(f)`** = travel cost. Euclidean by default (pure/fast); the node can
  inject a **Nav2 `ComputePathToPose` path length** via `cost_fn` (frontiers with
  no path return `None` and are skipped — a reachability filter).
- **`λ`** (`info_gain_lambda`, default 1.0): small → go far for big map gain;
  large → behaves like greedy.

The chosen goal logs its `gain`, `cost`, `score`.

## SpinAndScan (fixes "explores but sees no victim")

Frontier goals point the camera along the direction of travel, so the OAK-D
rarely faces the wall-mounted AprilTags. After **each reached goal** the explorer
rotates in place (`spin_and_scan`, `spin_scan_speed`, `spin_scan_duration`) to
sweep the camera across the walls, so victims are detected during normal
autonomous exploration. Disable with `spin_and_scan:=false`.

## Metrics (per run)

`result_exporter_node` writes, under `IA712_RESULTS_DIR` (default `results/`):

- `coverage_over_time.csv` — `time, coverage, path_length_m`
- `run_summary.json` — `strategy, final_coverage, success_coverage_90,
  victims_detected, path_length_m, duration_s, time_to_50/75/90_s`

Path length is the integral of `/turtlebot4/odom`; `time_to_X` is the sim-time to
first reach X% coverage.

## Running the benchmark

From the execution copy, in a conda-free env (see [`running_on_wsl.md`](running_on_wsl.md)):

```bash
# 2 strategies × 3 runs on rescue_arena, headless (≈ tunable, long → see .wslconfig)
env -i HOME="$HOME" PATH=/usr/bin:/bin TERM=xterm DISPLAY=:0 \
  bash scripts/sh/run_benchmark.sh
# tunables: IA712_BENCH_ALGOS, IA712_BENCH_RUNS, IA712_BENCH_DURATION, IA712_BENCH_DIR

# build the plots + summary table
python3 scripts/plot_benchmark.py experiments
```

Outputs:
- `experiments/<algo>_run<n>/` — per-run CSV + summary + log
- `experiments/plots/coverage_over_time.png` — coverage(t), mean ± std per strategy
- `experiments/plots/summary_bars.png` — time-to-90%, path length, victims
- `experiments/summary.md` — comparison table

## Hypothesis (to confirm/refute with the data)

> Information-gain reaches 90 % coverage faster than greedy (target: > 15 %
> faster), at the cost of a longer travelled path.

A clean quantitative result is the deliverable **even if information-gain does not
win** — the report concludes with the measured numbers.

## Quick single-run check (no full benchmark)

```bash
IA712_TB4_WORLD=rescue_arena IA712_EXPLORE=1 IA712_EXPLORE_STRATEGY=info_gain \
IA712_TB4_GUI=0 IA712_RVIZ=0 ./scripts/run.sh demo-tb4
# then inspect results/run_summary.json
```
