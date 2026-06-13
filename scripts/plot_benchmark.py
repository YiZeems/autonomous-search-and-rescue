#!/usr/bin/env python3
"""L17 bonus — plot the greedy-vs-information-gain exploration comparison.

Reads the per-run artefacts produced by scripts/sh/run_benchmark.sh:
    <exp_dir>/<algo>_run<n>/coverage_over_time.csv   (time, coverage, path_length_m)
    <exp_dir>/<algo>_run<n>/run_summary.json         (time_to_90_s, path_length_m, ...)

Produces (in <exp_dir>/plots/):
    coverage_over_time.png   — coverage(t), mean ± std band per strategy
    summary_bars.png         — time-to-90%, path length, victims per strategy
and a markdown table <exp_dir>/summary.md.

Usage:  python3 scripts/plot_benchmark.py [exp_dir=experiments]
Needs:  numpy, matplotlib (python3-numpy + python3-matplotlib, or pip).
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless
import matplotlib.pyplot as plt  # noqa: E402

COLORS = {"greedy": "tab:blue", "info_gain": "tab:orange"}


def _read_coverage(csv_path: Path):
    t, cov = [], []
    with csv_path.open() as f:
        for row in csv.DictReader(f):
            try:
                t.append(float(row["time"]))
                cov.append(float(row["coverage"]))
            except (KeyError, ValueError):
                continue
    return np.array(t), np.array(cov)


def _collect(exp_dir: Path):
    """Return {algo: {"runs": [(t, cov), ...], "summaries": [dict, ...]}}."""
    data: dict[str, dict] = {}
    for run_dir in sorted(exp_dir.glob("*_run*")):
        if not run_dir.is_dir():
            continue
        algo = run_dir.name.rsplit("_run", 1)[0]
        d = data.setdefault(algo, {"runs": [], "summaries": []})
        cov_csv = run_dir / "coverage_over_time.csv"
        if cov_csv.exists():
            t, cov = _read_coverage(cov_csv)
            if t.size:
                d["runs"].append((t, cov))
        summ = run_dir / "run_summary.json"
        if summ.exists():
            try:
                d["summaries"].append(json.loads(summ.read_text()))
            except json.JSONDecodeError:
                pass
    return data


def plot_coverage(data, out_png: Path):
    plt.figure(figsize=(8, 5))
    for algo, d in sorted(data.items()):
        runs = d["runs"]
        if not runs:
            continue
        t_max = min(t[-1] for t, _ in runs)
        grid = np.linspace(0, t_max, 200)
        stack = np.vstack([np.interp(grid, t, cov) for t, cov in runs])
        mean, std = stack.mean(axis=0), stack.std(axis=0)
        color = COLORS.get(algo)
        plt.plot(grid, mean * 100, label=f"{algo} (n={len(runs)})", color=color)
        plt.fill_between(grid, (mean - std) * 100, (mean + std) * 100, alpha=0.2, color=color)
    plt.axhline(90, ls="--", c="gray", lw=1, label="90% target")
    plt.xlabel("time (s)")
    plt.ylabel("map coverage (%)")
    plt.title("Exploration coverage over time — greedy vs information-gain")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_png, dpi=120)
    plt.close()


def _mean(values):
    vals = [v for v in values if v is not None]
    return float(np.mean(vals)) if vals else float("nan")


def plot_summary_bars(data, out_png: Path):
    algos = sorted(data)
    metrics = [
        ("time_to_90_s", "time to 90% (s)"),
        ("path_length_m", "path length (m)"),
        ("victims_detected", "victims found"),
    ]
    fig, axes = plt.subplots(1, len(metrics), figsize=(12, 4))
    for ax, (key, label) in zip(axes, metrics):
        means = [_mean([s.get(key) for s in data[a]["summaries"]]) for a in algos]
        ax.bar(algos, means, color=[COLORS.get(a, "gray") for a in algos])
        ax.set_title(label)
        ax.grid(True, axis="y", alpha=0.3)
        for i, m in enumerate(means):
            if not np.isnan(m):
                ax.text(i, m, f"{m:.1f}", ha="center", va="bottom")
    fig.suptitle("Summary metrics (mean over runs)")
    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    plt.close(fig)


def write_table(data, out_md: Path):
    keys = ["final_coverage", "time_to_50_s", "time_to_75_s", "time_to_90_s",
            "path_length_m", "victims_detected", "duration_s"]
    lines = ["# Exploration benchmark — greedy vs information-gain", "",
             "Mean over runs (per strategy).", "",
             "| strategy | runs | " + " | ".join(keys) + " |",
             "|" + "---|" * (len(keys) + 2)]
    for algo in sorted(data):
        s = data[algo]["summaries"]
        cells = [f"{_mean([x.get(k) for x in s]):.2f}" if s else "—" for k in keys]
        lines.append(f"| {algo} | {len(s)} | " + " | ".join(cells) + " |")
    lines += ["", "_Hypothesis: information-gain reaches 90% coverage faster than "
              "greedy, at the cost of a longer path._"]
    out_md.write_text("\n".join(lines) + "\n")


def main():
    exp_dir = Path(sys.argv[1] if len(sys.argv) > 1 else "experiments")
    if not exp_dir.exists():
        sys.exit(f"no such dir: {exp_dir} (run scripts/sh/run_benchmark.sh first)")
    data = _collect(exp_dir)
    if not data:
        sys.exit(f"no <algo>_run* runs found under {exp_dir}")
    plots = exp_dir / "plots"
    plots.mkdir(exist_ok=True)
    plot_coverage(data, plots / "coverage_over_time.png")
    plot_summary_bars(data, plots / "summary_bars.png")
    write_table(data, exp_dir / "summary.md")
    print(f"wrote {plots}/coverage_over_time.png, {plots}/summary_bars.png, {exp_dir}/summary.md")
    for algo in sorted(data):
        n = len(data[algo]["summaries"])
        print(f"  {algo}: {n} run(s), "
              f"mean time_to_90={_mean([s.get('time_to_90_s') for s in data[algo]['summaries']]):.1f}s, "
              f"mean path={_mean([s.get('path_length_m') for s in data[algo]['summaries']]):.1f}m")


if __name__ == "__main__":
    main()
