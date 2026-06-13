#!/usr/bin/env bash
# L17 bonus — exploration benchmark: greedy vs information-gain.
#
# Runs the headless demo N times per strategy on rescue_arena (same world, same
# spawn) and drops each run's metrics into experiments/<algo>_run<n>/ :
#   coverage_over_time.csv  (time, coverage, path_length_m)
#   run_summary.json        (time_to_50/75/90, path_length, victims, strategy, ...)
# Then `python3 scripts/plot_benchmark.py experiments` builds the plots + table.
#
# Run from the EXECUTION copy in a conda-free env (see docs/running_on_wsl.md):
#   env -i HOME="$HOME" PATH=/usr/bin:/bin TERM=xterm DISPLAY=:0 \
#     bash scripts/sh/run_benchmark.sh
#
# Tunables (env): IA712_BENCH_ALGOS, IA712_BENCH_RUNS, IA712_BENCH_DURATION,
#                 IA712_BENCH_DIR.
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${REPO}"

ALGOS="${IA712_BENCH_ALGOS:-greedy info_gain}"
RUNS="${IA712_BENCH_RUNS:-3}"
DURATION="${IA712_BENCH_DURATION:-900}"
EXP_DIR="${IA712_BENCH_DIR:-experiments}"

source /opt/ros/humble/setup.bash 2>/dev/null || true

_kill() { pkill -9 -f "ign gazebo|ign_gazebo|slam_toolbox|nav2|bt_navigator|controller_server|planner_server|frontier_explorer|robot_state_publisher|parameter_bridge|apriltag_node|victim_registry" 2>/dev/null || true; }

echo "=== L17 benchmark: algos=[${ALGOS}] runs=${RUNS} duration=${DURATION}s → ${EXP_DIR}/ ==="
for algo in ${ALGOS}; do
  for run in $(seq 1 "${RUNS}"); do
    outdir="${EXP_DIR}/${algo}_run${run}"
    mkdir -p "${outdir}"
    echo ">>> ${algo} run ${run}  →  ${outdir}"
    _kill; ros2 daemon stop >/dev/null 2>&1 || true; sleep 2
    IA712_TB4_WORLD=rescue_arena IA712_TB4_GUI=0 IA712_RVIZ=0 IA712_EXPLORE=1 \
    IA712_EXPLORE_STRATEGY="${algo}" IA712_RESULTS_DIR="${outdir}" \
      timeout "${DURATION}" ./scripts/run.sh demo-tb4 > "${outdir}/run.log" 2>&1 || true
    _kill; sleep 3
    echo "    summary: $(tr -d '\n' < "${outdir}/run_summary.json" 2>/dev/null || echo '(absent)')"
  done
done
echo "=== terminé. Génère les plots : python3 scripts/plot_benchmark.py ${EXP_DIR} ==="
