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

### Course grounding (CM8 / TP08)

These rules are exactly the goal-selection heuristics taught in **CM8 "Lecture 8:
Exploration"** (Next-Best-View utility functions):

| CM8 heuristic | CM8 utility | our strategy |
|---|---|---|
| H1 Proximity | `U(f) = −Cost(robot,f)`, Cost = **A\* path length** | `greedy` |
| H2 Size | `U(f) = Size(f)` (# frontier cells) | size term of `size_dist` |
| H3 Combined | `U(f) = α·Size(f) − β·Cost(robot,f)` | `size_dist`, and `info_gain` |

`info_gain` is H3 with a richer gain proxy: instead of `Size(f)` (frontier-cell
count) it uses the **unknown area revealed** within a radius, and instead of a
Euclidean cost it uses the **Nav2 path length** (CM8 explicitly says the cost
should be a planner path, not a straight line).

**Metric definition** follows TP08: exploration duration is measured *from the
start of autonomous motion until no accessible frontier remains*. TP08 also names
the failure mode we mitigate — *"oscillation / local greedy failure"* (the robot
re-picks the same nearby frontier) — and its remedy: *ignore repeatedly-selected
frontiers / enforce commitment timers*. That is our inaccessible-frontier
**blacklist** + `info_gain_min_gain` (skip frontiers that reveal too little).

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

## Measured results (n=3, rescue_arena, headless GPU)

Campaign of **3 valid runs per strategy**, 400 s budget each (see
`experiments/summary.md` + `plots/` for the full table and figures — regenerated,
not hard-coded here):

| strategy | final coverage | reaches 90 % | path | victims |
|---|---|---|---|---|
| greedy | 0.72 ± 0.04 | 0/3 (plateaus ~68 %) | 5.5 ± 0.5 m | 0 |
| info_gain | **0.85 ± 0.08** | 1/3 (t≈300 s) | 13.2 ± 2.2 m | **1.3** |

Reading: information-gain **covers more of the arena (+13 pts) and is the only
strategy that ever reaches 90 %**, and — by driving toward distant high-gain
frontiers — it sweeps the walls and **detects victims that greedy never sees**.
The price is the CM8 trade-off made concrete: it **travels ~2.4× further** and has
higher run-to-run variance (the long journeys are higher risk/reward). Greedy stays
local, cheap and repeatable, but saturates below the 90 % target. This supports the
hypothesis on coverage and victim discovery; 90 % is not yet *reliable* within
400 s (1/3), which is the honest limit to report.

## Quick single-run check (no full benchmark)

```bash
IA712_TB4_WORLD=rescue_arena IA712_EXPLORE=1 IA712_EXPLORE_STRATEGY=info_gain \
IA712_TB4_GUI=0 IA712_RVIZ=0 ./scripts/run.sh demo-tb4
# then inspect results/run_summary.json
```
