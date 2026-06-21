#!/usr/bin/env bash
# L17 bonus — exploration benchmark: greedy vs information-gain.
#
# Runs the headless demo N times per strategy on rescue_arena (same world, same
# spawn) and drops each run's metrics into experiments/<algo>_run<n>/ :
#   coverage_over_time.csv  (time, coverage, path_length_m)
#   run_summary.json        (time_to_50/75/90, path_length, victims, strategy, ...)
#   run_status.json         (exit_code, timed_out, summary_present, valid_run, ...)
# Then `python3 scripts/plot_benchmark.py experiments` builds the plots + table.
#
# Run from the EXECUTION copy in a conda-free env (see docs/running_on_wsl.md):
#   env -i HOME="$HOME" PATH=/usr/bin:/bin TERM=xterm DISPLAY=:0 \
#     bash scripts/sh/run_benchmark.sh
#
# ── RESUMABLE (survives a WSL/host reboot) ───────────────────────────────────
# On WSL a long sequence of heavy GPU runs can reboot the host mid-benchmark.
# This script is therefore idempotent: a run whose <outdir>/ already holds a
# VALID result is SKIPPED. So after any reboot you just relaunch the SAME
# command and the benchmark continues where it stopped — completed runs are
# never re-run or lost. It NEVER wipes experiments/ itself; pass
# IA712_BENCH_FRESH=1 to start a clean campaign on purpose.
#
# Tunables (env): IA712_BENCH_ALGOS, IA712_BENCH_RUNS, IA712_BENCH_DURATION,
#                 IA712_BENCH_DIR, IA712_BENCH_COOLDOWN, IA712_BENCH_FRESH,
#                 IA712_BENCH_VALID_MIN.
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${REPO}"

ALGOS="${IA712_BENCH_ALGOS:-greedy info_gain}"
RUNS="${IA712_BENCH_RUNS:-3}"
DURATION="${IA712_BENCH_DURATION:-900}"
EXP_DIR="${IA712_BENCH_DIR:-experiments}"
COOLDOWN="${IA712_BENCH_COOLDOWN:-30}"        # s between runs: let WSL reclaim RAM
FRESH="${IA712_BENCH_FRESH:-0}"               # 1 → wipe EXP_DIR before starting
# A run counts as VALID (and is skipped on resume) once it reached 90 % OR ran
# long enough to be a natural full-budget end. A full 400 s-budget run measures
# ~265-315 s of odom time; a reboot-truncated run measures less (e.g. 213 s) — so
# 250 s cleanly separates "ran to the end" from "interrupted", and the latter is
# retried on the next launch.
VALID_MIN="${IA712_BENCH_VALID_MIN:-250}"

source /opt/ros/humble/setup.bash 2>/dev/null || true

# Thorough cleanup so memory/processes don't ACCUMULATE across runs — the cause of
# the host reboot on long sequences (clean up before and after).
_kill() {
  pkill -9 -f "ign gazebo|ign_gazebo|gz sim|slam_toolbox|nav2|bt_navigator|controller_server|planner_server|behavior_server|lifecycle_manager|velocity_smoother|frontier_explorer|robot_state_publisher|parameter_bridge|ros_gz_bridge|apriltag_node|victim_registry|coverage_evaluator|result_exporter|rviz|waypoint_follower|tf_relay|cmd_vel_relay|scan_throttle|turtlebot4|irobot|create3|spawner" 2>/dev/null || true
  ros2 daemon stop >/dev/null 2>&1 || true
  find /dev/shm -maxdepth 1 \( -name 'fastrtps*' -o -name 'sem.fastrtps*' \) -delete 2>/dev/null || true
}
_freemb() { free -m 2>/dev/null | awk '/Mem/{print $7}'; }

# Is <outdir> already a valid, finished run? (resume guard)
_is_valid() {
  python3 - "$1" "${VALID_MIN}" <<'PY' 2>/dev/null
import json, sys
try:
    d = json.load(open(sys.argv[1] + "/run_summary.json"))
except Exception:
    sys.exit(1)
fc = d.get("final_coverage")
dur = d.get("duration_s") or 0
ok = isinstance(fc, (int, float)) and (d.get("success_coverage_90") or dur >= float(sys.argv[2]))
sys.exit(0 if ok else 1)
PY
}

if [ "${FRESH}" = "1" ]; then
  echo "[bench] IA712_BENCH_FRESH=1 → wipe ${EXP_DIR}/"
  rm -rf "${EXP_DIR}"
fi

echo "L17 benchmark: algos=[${ALGOS}] runs=${RUNS} duration=${DURATION}s cooldown=${COOLDOWN}s → ${EXP_DIR}/ (resumable)"
for algo in ${ALGOS}; do
  for run in $(seq 1 "${RUNS}"); do
    outdir="${EXP_DIR}/${algo}_run${run}"
    if _is_valid "${outdir}"; then
      echo ">>> ${algo} run ${run}  →  ${outdir}  [SKIP: déjà valide]"
      continue
    fi
    mkdir -p "${outdir}"
    free_before="$(_freemb)"
    echo ">>> ${algo} run ${run}  →  ${outdir}  (free ${free_before}MB)"
    _kill; sleep 2
    IA712_TB4_WORLD=rescue_arena IA712_TB4_GUI=0 IA712_RVIZ=0 IA712_EXPLORE=1 \
    IA712_EXPLORE_STRATEGY="${algo}" IA712_RESULTS_DIR="${outdir}" \
      timeout "${DURATION}" ./scripts/run.sh demo-tb4 > "${outdir}/run.log" 2>&1
    rc=$?
    _kill
    # rc 124 = timeout reached (NORMAL: the explorer keeps running until the
    # budget; the result_exporter has already flushed its CSV/JSON on SIGTERM).
    [ "${rc}" = "124" ] && timed_out=true || timed_out=false
    [ -f "${outdir}/run_summary.json" ] && summary=true || summary=false
    if _is_valid "${outdir}"; then valid=true; reason=""; else
      valid=false
      if [ "${summary}" = "false" ]; then reason="no_summary (crash/reboot before flush)"
      else reason="run too short (<${VALID_MIN}s) — interrupted"; fi
    fi
    printf '{\n  "algo": "%s",\n  "run": %s,\n  "exit_code": %s,\n  "timed_out": %s,\n  "summary_present": %s,\n  "valid_run": %s,\n  "reason_if_invalid": "%s",\n  "duration_budget_s": %s,\n  "free_mb_before": "%s"\n}\n' \
      "${algo}" "${run}" "${rc}" "${timed_out}" "${summary}" "${valid}" "${reason}" "${DURATION}" "${free_before}" \
      > "${outdir}/run_status.json"
    echo "    valid=${valid} timed_out=${timed_out} summary=${summary} ${reason:+(${reason})}"
    echo "    summary: $(tr -d '\n' < "${outdir}/run_summary.json" 2>/dev/null || echo '(absent)')"
    # recovery window: full cleanup done, idle so WSL's autoMemoryReclaim frees RAM
    echo "    cooldown ${COOLDOWN}s (free before ${free_before}MB)"
    sleep "${COOLDOWN}"
    echo "    free after cooldown: $(_freemb)MB"
  done
done

# Campaign recap: how many valid runs per algo (so a reboot-truncated campaign is
# obvious instead of silently averaging too few runs).
echo "bilan campagne"
for algo in ${ALGOS}; do
  ok=0; tot=0
  for run in $(seq 1 "${RUNS}"); do
    tot=$((tot+1)); _is_valid "${EXP_DIR}/${algo}_run${run}" && ok=$((ok+1))
  done
  echo "  ${algo}: ${ok}/${tot} runs valides"
done
echo "terminé. Si tous valides → python3 scripts/plot_benchmark.py ${EXP_DIR}"
echo "sinon relance la MÊME commande : les runs valides seront sautés (resume)."
